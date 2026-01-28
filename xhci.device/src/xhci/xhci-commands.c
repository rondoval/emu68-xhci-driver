#include <debug.h>

#include <xhci/usb.h>
#include <xhci/xhci.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-debug.h>
#include <xhci/xhci-events.h>
#include <xhci/xhci-td.h>
#include <xhci/xhci-usb.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-udev.h>
#include <devices/usbhardware.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-commands] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci-commands] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

struct pending_command; /* forward declaration */
typedef void (*command_handler)(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event);

struct pending_command
{
    struct MinNode node;
    dma_addr_t cmd_trb_dma;  /* value from queue_trb */
    struct usb_device *udev; /* for slot/endpoint checks */
    u32 ep_index;            /* endpoint index encoded into the command */
    command_handler complete;
    struct IOUsbHWReq *req; /* to continue control transfers */
#ifdef DEBUG
    trb_type type; /* command type for diagnostics */
#endif
};

static const command_handler command_handlers[];

#ifdef DEBUG_HIGH
static const char *xhci_command_type_name(trb_type type)
{
    switch (type)
    {
    case TRB_ENABLE_SLOT:
        return "ENABLE_SLOT";
    case TRB_DISABLE_SLOT:
        return "DISABLE_SLOT";
    case TRB_ADDR_DEV:
        return "ADDRESS_DEVICE";
    case TRB_CONFIG_EP:
        return "CONFIGURE_ENDPOINT";
    case TRB_EVAL_CONTEXT:
        return "EVALUATE_CONTEXT";
    case TRB_RESET_EP:
        return "RESET_ENDPOINT";
    case TRB_STOP_RING:
        return "STOP_RING";
    case TRB_SET_DEQ:
        return "SET_DEQ";
    case TRB_RESET_DEV:
        return "RESET_DEVICE";
    default:
        return "UNKNOWN";
    }
}
#endif

/**
 * Generic function for queueing a command TRB on the command ring.
 * Check to make sure there's room on the command ring for one command TRB.
 *
 * @param ctrl		Host controller data structure
 * @param ptr		Pointer address to write in the first two fields (opt.)
 * @param slot_id	Slot ID to encode in the flags field (opt.)
 * @param ep_index	Endpoint index to encode in the flags field (opt.)
 * @param cmd		Command type to enqueue
 * @param req       Optional IOUsbHWReq to continue control transfers after configuring endpoints
 * @param udev      Optional usb_device for slot/endpoint checks
 * Return: none
 */
static void xhci_queue_command(struct xhci_ctrl *ctrl, dma_addr_t addr, u32 slot_id, u32 ep_index, trb_type cmd, struct IOUsbHWReq *req, struct usb_device *udev)
{
    u32 fields[4];

    int ret = prepare_ring(ctrl, ctrl->cmd_ring, EP_STATE_RUNNING);
    if (ret)
    {
        Kprintf("FATAL ERROR command ring not ready: cmd %ld, ret=%ld\n", cmd, ret);
        return;
    }

    fields[0] = lower_32_bits(addr);
    fields[1] = upper_32_bits(addr);
    fields[2] = 0;
    fields[3] = TRB_TYPE(cmd) | SLOT_ID_FOR_TRB(slot_id) |
                ctrl->cmd_ring->cycle_state;

    /*
     * Only 'reset endpoint', 'stop endpoint' and 'set TR dequeue pointer'
     * commands need endpoint id encoded.
     */
    if (cmd >= TRB_RESET_EP && cmd <= TRB_SET_DEQ)
        fields[3] |= EP_ID_FOR_TRB(ep_index);

    dma_addr_t trb_dma = queue_trb(ctrl, ctrl->cmd_ring, FALSE, fields);

    /* Add command handler to pending list */
    struct pending_command *pending_cmd = AllocVecPooled(ctrl->memoryPool, sizeof(struct pending_command));
    if (!pending_cmd)
    {
        Kprintf("Failed to allocate pending command\n");
        return;
    }

    pending_cmd->cmd_trb_dma = trb_dma;
    pending_cmd->udev = udev;
    pending_cmd->ep_index = ep_index;
    pending_cmd->req = req;
#ifdef DEBUG
    pending_cmd->type = cmd;
#endif

    pending_cmd->complete = command_handlers[cmd];
    AddTailMinList(&ctrl->pending_commands, (struct MinNode *)pending_cmd);

    /* Ring the command ring doorbell */
    xhci_writel(&ctrl->dba->doorbell[0], DB_VALUE_HOST);
}

/*
 * Command handlers
 */

static void handle_reset_ep(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    (void)ctrl;
    u32 flags = LE32(event->event_cmd.flags);
    ULONG slot_id = cmd->udev->slot_id;
    ULONG ep_index = cmd->ep_index;
    ULONG endpoint = EP_INDEX_TO_ENDPOINT(ep_index);

    if (TRB_TO_SLOT_ID(flags) != slot_id)
    {
        Kprintf("Expected a TRB for slot %ld, got %ld\n", slot_id, TRB_TO_SLOT_ID(flags));
        struct ep_context *ep_ctx = xhci_ep_get_context(cmd->udev, endpoint);
        xhci_ep_set_failed(ep_ctx);
        return;
    }

    KprintfH("Reset EP %ld completed successfully\n", endpoint);
    xhci_set_deq_pointer(cmd->udev, ep_index);
}

static void handle_set_deq(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    (void)ctrl;
    u32 flags = LE32(event->event_cmd.flags);
    ULONG slot_id = cmd->udev->slot_id;
    ULONG ep_index = cmd->ep_index;
    ULONG endpoint = EP_INDEX_TO_ENDPOINT(ep_index);
    xhci_comp_code comp = GET_COMP_CODE(LE32(event->event_cmd.status));
    struct ep_context *ep_ctx = xhci_ep_get_context(cmd->udev, endpoint);

    if (TRB_TO_SLOT_ID(flags) != slot_id || comp != COMP_SUCCESS)
    {
        Kprintf("Expected a TRB for slot %ld with SUCCESS, got %ld with %ld\n",
                slot_id,
                TRB_TO_SLOT_ID(flags),
                comp);
        xhci_ep_set_failed(ep_ctx);
        return;
    }
    KprintfH("Set DEQ for EP %ld completed successfully, status code %ld (success=1)\n", endpoint, comp);

    if (xhci_ep_get_state(ep_ctx) == USB_DEV_EP_STATE_RESETTING)
    {
        KprintfH("EP %ld was resetting, completing reset\n", endpoint);
        /*
         * If this is due to e.g. STALL recovery, we need to sort out the device itself...:
         * issue ClearFeature(CLEAR_TT_BUFFER) to the hub if its control or bulk ep and dev is behind a TT
         * if not control ep,  issue ClearFeature(ENDPOINT_HALT) to the device.
         * We'll do that by pushing these to fron of the pending queue.
         */
        int ep_type = xhci_ep_type_for_index(cmd->udev, ep_index);

        if (endpoint != 0 && ep_type != USB_ENDPOINT_XFER_CONTROL)
        {
            /* Dispatch deferred device-side CLEAR_FEATURE after host recovery. */
            usb_glue_clear_feature_halt_internal(cmd->udev, ep_index);
        }

        /* For control/bulk endpoints behind a TT, clear the TT buffer on the hub. */
        if (cmd->udev->speed == USB_SPEED_FULL || cmd->udev->speed == USB_SPEED_LOW)
        {
            if (ep_type == USB_ENDPOINT_XFER_CONTROL || ep_type == USB_ENDPOINT_XFER_BULK)
            {
                usb_glue_clear_tt_buffer_internal(cmd->udev, ep_index, ep_type);
            }
        }
    }

    xhci_ep_set_idle(ep_ctx);
}

static void handle_stop_ring(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    (void)ctrl;
    u32 flags = LE32(event->event_cmd.flags);
    trb_type type = TRB_FIELD_TO_TYPE(flags);
    xhci_comp_code comp = GET_COMP_CODE(LE32(event->event_cmd.status));
    ULONG slot_id = cmd->udev->slot_id;
    ULONG ep_index = cmd->ep_index;
    u32 endpoint = EP_INDEX_TO_ENDPOINT(ep_index);

    if (type != TRB_COMPLETION || TRB_TO_SLOT_ID(flags) != slot_id || comp != COMP_SUCCESS)
    {
        Kprintf("Expected a TRB for slot %ld with SUCCESS, got %ld with %ld\n", slot_id, TRB_TO_SLOT_ID(flags), comp);
        return;
    }

    KprintfH("Stopped EP %ld...\n", endpoint);

    /* Fail all active TDs on this endpoint */
    struct ep_context *ep_ctx = xhci_ep_get_context(cmd->udev, endpoint);
    xhci_ep_set_failed(ep_ctx);

    xhci_set_deq_pointer(cmd->udev, ep_index);
}

/*
 * so this one is awkward, because we do this just before doing control xfers
 * to make sure the endpoint is in the right state.
 * So after this is handled, we need to continue with the control xfer.
 */
static void handle_config_ep(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    (void)ctrl;
#ifdef DEBUG_HIGH
    trb_type type = cmd->type;
    const char *type_name = xhci_command_type_name(type);
#endif
    xhci_comp_code comp = GET_COMP_CODE(LE32(event->event_cmd.status));

    if (comp != COMP_SUCCESS)
    {
#ifdef DEBUG_HIGH
        ULONG slot_id = TRB_TO_SLOT_ID(LE32(event->event_cmd.flags));
        Kprintf("ERROR: %s command for slot %ld returned completion code 0x%lx.\n",
                type_name, slot_id, (ULONG)comp);

        if (cmd->udev)
        {
            struct xhci_virt_device *virt_dev = ctrl->devs[cmd->udev->slot_id];
            if (virt_dev)
            {
                xhci_inval_cache((uintptr_t)virt_dev->in_ctx->bytes, virt_dev->in_ctx->size);
                xhci_dump_slot_ctx("[xhci-commands] handle_config_ep:", xhci_get_slot_ctx(ctrl, virt_dev->in_ctx));
                xhci_inval_cache((uintptr_t)virt_dev->out_ctx->bytes, virt_dev->out_ctx->size);
                xhci_dump_slot_ctx("[xhci-commands] handle_config_ep:", xhci_get_slot_ctx(ctrl, virt_dev->out_ctx));
            }
        }
#endif
        return;
    }

#ifdef DEBUG_HIGH
    KprintfH("%s command for slot %ld completed successfully\n", type_name, TRB_TO_SLOT_ID(LE32(event->event_cmd.flags)));

    if (type == TRB_EVAL_CONTEXT && cmd->udev)
    {
        struct xhci_virt_device *virt_dev = ctrl->devs[cmd->udev->slot_id];
        if (virt_dev)
        {
            xhci_inval_cache((uintptr_t)virt_dev->in_ctx->bytes, virt_dev->in_ctx->size);
            xhci_dump_slot_ctx("[xhci-commands] handle_config_ep:", xhci_get_slot_ctx(ctrl, virt_dev->in_ctx));
            xhci_inval_cache((uintptr_t)virt_dev->out_ctx->bytes, virt_dev->out_ctx->size);
            xhci_dump_slot_ctx("[xhci-commands] handle_config_ep:", xhci_get_slot_ctx(ctrl, virt_dev->out_ctx));
        }
    }
#endif

    cmd->udev->slot_state = USB_DEV_SLOT_STATE_CONFIGURED;

    if (cmd->req)
    {
        unsigned int timeout = XHCI_TIMEOUT;
        if (cmd->req->iouh_Flags & UHFF_NAKTIMEOUT)
            timeout = cmd->req->iouh_NakTimeout;

        xhci_ctrl_tx(cmd->udev, cmd->req, timeout);
    }
}

static void handle_enable_slot(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    KprintfH("event status=%08lx flags=%08lx\n", (ULONG)LE32(event->event_cmd.status), (ULONG)LE32(event->event_cmd.flags));
    if (GET_COMP_CODE(LE32(event->event_cmd.status)) != COMP_SUCCESS)
    {
        Kprintf("ERROR: Enable Slot command failed.\n");
        io_reply_failed(cmd->req, UHIOERR_HOSTERROR);
        return;
    }

    cmd->udev->slot_id = TRB_TO_SLOT_ID(LE32(event->event_cmd.flags));
    cmd->udev->slot_state = USB_DEV_SLOT_STATE_ENABLED;
    KprintfH("assigned slot_id=%ld for addr=%lu\n", (ULONG)cmd->udev->slot_id, (ULONG)cmd->udev->poseidon_address);

    int ret = xhci_alloc_virt_device(ctrl, cmd->udev->slot_id);
    if (ret < 0)
    {
        /*
         * TODO: Unsuccessful Address Device command shall leave
         * the slot in default. So, issue Disable Slot command now.
         */
        Kprintf("Could not allocate xHCI USB device data structures\n");
        io_reply_failed(cmd->req, UHIOERR_OUTOFMEMORY);
        return;
    }

    if (ctrl->devs[cmd->udev->slot_id])
        ctrl->devs[cmd->udev->slot_id]->udev = cmd->udev;

    // Continue with Address Device command, passing cmd->req
    xhci_address_device(cmd->udev, cmd->req);
}

static void handle_disable_slot(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    KprintfH("event status=%08lx flags=%08lx\n", (ULONG)LE32(event->event_cmd.status), (ULONG)LE32(event->event_cmd.flags));
    xhci_comp_code comp = GET_COMP_CODE(LE32(event->event_cmd.status));

    if (comp != COMP_SUCCESS)
    {
        Kprintf("ERROR: Disable Slot command failed for slot %ld (comp=%ld).\n",
                (ULONG)cmd->udev->slot_id, comp);
        return;
    }

    KprintfH("Disabled slot %ld successfully (comp=%ld).\n", cmd->udev->slot_id, comp);
    cmd->udev->slot_state = USB_DEV_SLOT_STATE_DISABLED;
    xhci_free_virt_device(ctrl, cmd->udev->slot_id);

    if (cmd->udev)
    {
        usb_glue_free_udev_slot(cmd->udev);
    }
}

static void handle_address_device(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    KprintfH("event status=%08lx flags=%08lx\n", (ULONG)LE32(event->event_cmd.status), (ULONG)LE32(event->event_cmd.flags));

    int err = UHIOERR_NO_ERROR;
    switch (GET_COMP_CODE(LE32(event->event_cmd.status)))
    {
    case COMP_CTX_STATE:
    case COMP_EBADSLT:
        Kprintf("Setup ERROR: address device command for slot %ld.\n", cmd->udev->slot_id);
        err = UHIOERR_HOSTERROR;
        break;
    case COMP_TX_ERR:
        Kprintf("Device not responding to set address.\n");
        err = UHIOERR_TIMEOUT;
        break;
    case COMP_DEV_ERR:
        Kprintf("ERROR: Incompatible device for address device command.\n");
        err = UHIOERR_BADPARAMS;
        break;
    case COMP_SUCCESS:
        KprintfH("Successful Address Device command\n");
        break;
    default:
        Kprintf("ERROR: unexpected command completion code 0x%lx.\n",
                GET_COMP_CODE(LE32(event->event_cmd.status)));
        err = UHIOERR_HOSTERROR;
        break;
    }

    if (err)
    {
        if (cmd->udev)
        {
            struct xhci_virt_device *virt_dev = ctrl->devs[cmd->udev->slot_id];
            if (virt_dev)
            {
                Kprintf("Address Device failure for slot %ld (code %ld)\n", (ULONG)cmd->udev->slot_id, (ULONG)err);
                xhci_inval_cache((uintptr_t)virt_dev->in_ctx->bytes, virt_dev->in_ctx->size);
                xhci_dump_slot_ctx("[xhci-commands] handle_address_device:", xhci_get_slot_ctx(ctrl, virt_dev->in_ctx));
                xhci_inval_cache((uintptr_t)virt_dev->out_ctx->bytes, virt_dev->out_ctx->size);
                xhci_dump_slot_ctx("[xhci-commands] handle_address_device:", xhci_get_slot_ctx(ctrl, virt_dev->out_ctx));
            }
        }
        /*
         * Unsuccessful Address Device command shall leave the
         * slot in default state. So, issue Disable Slot command now.
         */
        xhci_disable_slot(cmd->udev);
        if (cmd->req)
            io_reply_failed(cmd->req, err);
        return;
    }

    struct xhci_virt_device *virt_dev = ctrl->devs[cmd->udev->slot_id];

    xhci_inval_cache((uintptr_t)virt_dev->out_ctx->bytes,
                     virt_dev->out_ctx->size);

    cmd->udev->xhci_address = LE32(xhci_get_slot_ctx(ctrl, virt_dev->out_ctx)->dev_state) & DEV_ADDR_MASK;
    cmd->udev->slot_state = USB_DEV_SLOT_STATE_ADDRESSED;
    KprintfH("Assigned xHCI address %ld to slot %ld (Poseidon address %ld)\n",
             (ULONG)cmd->udev->xhci_address, (ULONG)cmd->udev->slot_id, (cmd->req) ? (ULONG)cmd->req->iouh_DevAddr : (ULONG)0);

#ifdef DEBUG_HIGH
    struct xhci_slot_ctx *slot_ctx = xhci_get_slot_ctx(ctrl, virt_dev->out_ctx);

    KprintfH("xHC internal address is: %ld dev_info=%08lx dev_info2=%08lx dev_state=%08lx\n",
             (ULONG)(LE32(slot_ctx->dev_state) & DEV_ADDR_MASK),
             (ULONG)LE32(slot_ctx->dev_info), (ULONG)LE32(slot_ctx->dev_info2), (ULONG)LE32(slot_ctx->dev_state));
#endif

    /* Continue the original Poseidon request after the device is addressed. */
    if (cmd->req)
    {
        struct UsbSetupData *setup = &cmd->req->iouh_SetupData;
        BOOL is_set_address = (setup->bRequest == USB_REQ_SET_ADDRESS) &&
                              ((setup->bmRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD);

        if (is_set_address)
        {
            /* Upper layer will migrate the context based on Poseidon address. */
            io_reply_data(cmd->udev, cmd->req, UHIOERR_NO_ERROR, 0);
        }
        else
        {
            int ret = usb_glue_ctrl_after_address(cmd->udev, cmd->req);
            if (ret != UHIOERR_NO_ERROR)
                io_reply_failed(cmd->req, ret);
        }
    }
}

static void handle_reset_device(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    (void)ctrl;
    KprintfH("event status=%08lx flags=%08lx\n", (ULONG)LE32(event->event_cmd.status), (ULONG)LE32(event->event_cmd.flags));
    if (GET_COMP_CODE(LE32(event->event_cmd.status)) != COMP_SUCCESS)
    {
        Kprintf("ERROR: Reset Device command failed for slot %ld.\n", cmd->udev->slot_id);
        return;
    }

    KprintfH("Reset Device for slot %ld completed successfully.\n", cmd->udev->slot_id);
}

/*
 * Mapping of commands to their handlers
 */
static const command_handler command_handlers[] = {
    [TRB_ENABLE_SLOT] = handle_enable_slot, /* Enable Slot Command */
    [TRB_DISABLE_SLOT] = handle_disable_slot,
    /* Disable Slot Command */              // TODO should do DISABLE_SLOT when device is disconnected
    [TRB_ADDR_DEV] = handle_address_device, /* Address Device Command */
    [TRB_CONFIG_EP] = handle_config_ep,     /* Configure Endpoint Command */
    [TRB_EVAL_CONTEXT] = handle_config_ep,  /* Evaluate Context Command */
    [TRB_RESET_EP] = handle_reset_ep,       /* Reset Endpoint Command */
    [TRB_STOP_RING] = handle_stop_ring,     /* Stop Transfer Ring Command */
    [TRB_SET_DEQ] = handle_set_deq,         /* Set Transfer Ring Dequeue Pointer Command */
    [TRB_RESET_DEV] = handle_reset_device,  /* Reset Device Command */
};

/* Command event dispatcher
 * Called when a Command Completion Event TRB is received
 */
void xhci_dispatch_command_event(struct xhci_ctrl *ctrl, union xhci_trb *event)
{
    for (struct MinNode *node = ctrl->pending_commands.mlh_Head; node->mln_Succ; node = node->mln_Succ)
    {
        struct pending_command *cmd = (struct pending_command *)node;
        dma_addr_t trb_addr = (dma_addr_t)LE64(event->event_cmd.cmd_trb);
        if ((trb_addr == cmd->cmd_trb_dma))
        {
            // Found the command
            if (cmd->complete)
            {
                cmd->complete(ctrl, cmd, event);
            }
            else
            {
                Kprintf("No handler for command TRB %lx\n", cmd->cmd_trb_dma);
            }
            Remove((struct Node *)cmd);
            FreeVecPooled(ctrl->memoryPool, cmd);
            return;
        }
    }
    Kprintf("No matching pending command for command completion event TRB (%08lx %08lx %08lx %08lx)\n",
            (ULONG)LE32(event->generic.field[0]),
            (ULONG)LE32(event->generic.field[1]),
            (ULONG)LE32(event->generic.field[2]),
            (ULONG)LE32(event->generic.field[3]));
}

/*
 * Send reset endpoint command for given endpoint. This recovers from a
 * halted endpoint (e.g. due to a stall error).
 */
void xhci_reset_ep(struct usb_device *udev, u32 ep_index)
{
    // TODO for error state, just set deq pointer to current enqueue pointer
    // ep needs be in halted state, otherwise this command will fail

    struct xhci_ctrl *ctrl = udev->controller;
    u32 ep = EP_INDEX_TO_ENDPOINT(ep_index);
    struct ep_context *ep_ctx = xhci_ep_get_context(udev, ep);

    KprintfH("Resetting addr %ld slot=%ld EP %ld (index=%ld)...\n",
             (LONG)udev->poseidon_address, (LONG)udev->slot_id, (LONG)ep, (LONG)ep_index);
    xhci_ep_set_resetting(ep_ctx);

    // set TSP=0 - reset split transaction, flush cached TDs
    xhci_queue_command(ctrl, 0, udev->slot_id, ep_index, TRB_RESET_EP, NULL, udev); // handle_reset_ep
}

/*
 * Stops transfer processing for an endpoint and throws away all unprocessed
 * TRBs by setting the xHC's dequeue pointer to our enqueue pointer. The next
 * xhci_bulk_tx/xhci_ctrl_tx on this enpoint will add new transfers there and
 * ring the doorbell, causing this endpoint to start working again.
 */
void xhci_stop_endpoint(struct usb_device *udev, u32 ep_index)
{
    u32 endpoint = EP_INDEX_TO_ENDPOINT(ep_index);
    KprintfH("Stop EP addr=%ld ep=%ld (index=%ld)\n",
             (LONG)udev->poseidon_address, (LONG)endpoint, (LONG)ep_index);
    struct xhci_ctrl *ctrl = udev->controller;

    enum ep_state state = xhci_ep_get_state(xhci_ep_get_context(udev, endpoint));
    if (state == USB_DEV_EP_STATE_RECEIVING ||
        state == USB_DEV_EP_STATE_RECEIVING_CONTROL_SHORT ||
        state == USB_DEV_EP_STATE_RT_ISO_RUNNING)
    {
        struct ep_context *ep_ctx = xhci_ep_get_context(udev, endpoint);
        xhci_ep_set_aborting(ep_ctx);
        KprintfH("Stopping EP %ld...\n", endpoint);

        // TODO suspend bit support
        xhci_queue_command(ctrl, 0, udev->slot_id, ep_index, TRB_STOP_RING, NULL, udev);
    }
}

/*
 * Sets the transfer ring dequeue pointer for the given endpoint.
 * Used after a reset endpoint command to continue processing.
 * The endpoint needs to be either in Error or Stopped state.
 */
void xhci_set_deq_pointer(struct usb_device *udev, u32 ep_index)
{
    struct xhci_ctrl *ctrl = udev->controller;
    ULONG slot_id = udev->slot_id;

    struct xhci_ring *ring = ctrl->devs[slot_id]->eps[ep_index].ring;

    u64 deq_ptr = xhci_trb_virt_to_dma(ring->enq_seg, (void *)((uintptr_t)ring->enqueue)) | ring->cycle_state;

    KprintfH("Setting DEQ pointer for EP index %ld to %lx\n", (LONG)ep_index, (ULONG)deq_ptr);
    xhci_queue_command(ctrl, deq_ptr, udev->slot_id, ep_index, TRB_SET_DEQ, NULL, udev); // handle_set_deq
}

/*
 * Issues a reset device command to inform the xHCI controller
 * that the device has been reset by software (e.g. via USB port
 * reset on the root hub). The xHC will reinitialize the device
 * and its endpoints.
 */
void xhci_reset_device(struct usb_device *udev)
{
    struct xhci_ctrl *ctrl = udev->controller;
    KprintfH("Resetting device on slot %ld\n", (LONG)udev->slot_id);
    // set slot id, cycle bit; clear other fields and issue reset device command
    xhci_queue_command(ctrl, 0, udev->slot_id, 0, TRB_RESET_DEV, NULL, udev); // TODO handle_reset_device - wait for command completion
}

/**
 * Issue a configure endpoint command or evaluate context command
 *
 * @param udev	pointer to the Device Data Structure
 * @param ctx_change	flag to indicate the Context has changed or NOT
 * @param req       Optional IOUsbHWReq to continue control transfers after configuring endpoints
 * Return: 0 on success, -1 on failure
 */
void xhci_configure_endpoints(struct usb_device *udev, BOOL ctx_change, struct IOUsbHWReq *req)
{
    struct xhci_container_ctx *in_ctx;
    struct xhci_virt_device *virt_dev;
    struct xhci_ctrl *ctrl = udev->controller;

    virt_dev = ctrl->devs[udev->slot_id];
    in_ctx = virt_dev->in_ctx;

    KprintfH("about to issue %s for addr=%lu slot=%lu\n",
             ctx_change ? "EVAL_CONTEXT" : "CONFIG_EP",
             (ULONG)udev->poseidon_address,
             (ULONG)udev->slot_id);
    xhci_flush_cache((uintptr_t)in_ctx->bytes, in_ctx->size);
    // TODO support deconfigure - DC flag?
    xhci_queue_command(ctrl, in_ctx->dma, udev->slot_id, 0, ctx_change ? TRB_EVAL_CONTEXT : TRB_CONFIG_EP, req, udev);
}

/**
 * Issue Enable slot command to the controller to allocate
 * device slot and assign the slot id. It fails if the xHC
 * ran out of device slots, the Enable Slot command timed out,
 * or allocating memory failed.
 *
 * @param udev	pointer to the Device Data Structure
 * @param req   ioreq to reply to
 */
void xhci_enable_slot(struct usb_device *udev, struct IOUsbHWReq *req)
{
    struct xhci_ctrl *ctrl = udev->controller;

    KprintfH("queue ENABLE_SLOT addr=%lu route=0x%lx parent_port=%lu\n",
             (ULONG)udev->poseidon_address,
             (ULONG)udev->route,
             (ULONG)udev->parent_port);
    xhci_queue_command(ctrl, 0, 0, 0, TRB_ENABLE_SLOT, req, udev);
}

void xhci_disable_slot(struct usb_device *udev)
{
    struct xhci_ctrl *ctrl = udev->controller;
    for (int ep = 0; ep < USB_MAXENDPOINTS; ep++)
    {
        xhci_ep_set_aborting(xhci_ep_get_context(udev, ep));
    }

    if (udev->slot_state == USB_DEV_SLOT_STATE_DISABLED)
    {
        KprintfH("queue DISABLE_SLOT skipped; slot_id=%ld already disabled\n", (ULONG)udev->slot_id);
        return;
    }

    udev->slot_state = USB_DEV_SLOT_STATE_DISABLED;
    KprintfH("queue DISABLE_SLOT for slot_id=%ld addr=%lu\n", (ULONG)udev->slot_id, (ULONG)udev->poseidon_address);
    xhci_queue_command(ctrl, 0, udev->slot_id, 0, TRB_DISABLE_SLOT, NULL, udev);
}

static void xhci_set_address(struct usb_device *udev, struct IOUsbHWReq *req)
{
    struct xhci_ctrl *ctrl = udev->controller;
    struct xhci_input_control_ctx *ctrl_ctx;
    struct xhci_virt_device *virt_dev;
    unsigned int slot_id = udev->slot_id;

    virt_dev = ctrl->devs[slot_id];
    KprintfH("virt_dev=%lx, slot_id=%ld\n", (ULONG)virt_dev, (LONG)slot_id);
    if (!virt_dev)
    {
        Kprintf("ERROR: no virt_dev for slot %ld\n", (ULONG)slot_id);
        if (req)
            io_reply_failed(req, UHIOERR_OUTOFMEMORY);
        return;
    }

    /* If already addressed (internal address non-zero), don't re-issue. */
    {
        xhci_inval_cache((uintptr_t)virt_dev->out_ctx->bytes, virt_dev->out_ctx->size);
        struct xhci_slot_ctx *sc = xhci_get_slot_ctx(ctrl, virt_dev->out_ctx);
        if ((LE32(sc->dev_state) & DEV_ADDR_MASK) != 0)
        {
            KprintfH("slot %ld already addressed (dev_state=%08lx), skipping.\n",
                     (ULONG)slot_id, (ULONG)LE32(sc->dev_state));
            if (req)
                io_reply_data(udev, req, UHIOERR_NO_ERROR, 0);
            return;
        }
    }

    /*
     * This is the first Set Address since device plug-in
     * so setting up the slot context.
     */
    KprintfH("Setting up addressable device (slot %ld)\n", (ULONG)slot_id);
    xhci_setup_addressable_virt_dev(ctrl, udev);

    ctrl_ctx = xhci_get_input_control_ctx(virt_dev->in_ctx);
    KprintfH("ctrl_ctx=%lx in_ctx=%lx out_ctx=%lx\n",
             (ULONG)ctrl_ctx, (ULONG)virt_dev->in_ctx, (ULONG)virt_dev->out_ctx);
    ctrl_ctx->add_flags = LE32(SLOT_FLAG | EP0_FLAG);
    ctrl_ctx->drop_flags = 0;

    xhci_flush_cache((uintptr_t)ctrl_ctx, sizeof(struct xhci_input_control_ctx));

    KprintfH("queue ADDR_DEV cmd, in_ctx->dma=%lx addr=%lu slot=%lu parent_addr=%lu parent_port=%lu route=0x%lx\n",
             (ULONG)virt_dev->in_ctx->dma,
             (ULONG)udev->poseidon_address,
             (ULONG)slot_id,
             (ULONG)(udev->parent ? udev->parent->poseidon_address : 0),
             (ULONG)udev->parent_port,
             (ULONG)udev->route);
    xhci_queue_command(ctrl, virt_dev->in_ctx->dma, slot_id, 0, TRB_ADDR_DEV, req, udev);
}

void xhci_address_device(struct usb_device *udev, struct IOUsbHWReq *req)
{
    /* If we don't have a slot yet, enable one and allocate Virt Dev */
    if (udev->slot_id == 0)
    {
        KprintfH("no slot_id yet; enabling slot...\n");
        xhci_enable_slot(udev, req);
        return;
    }

    xhci_set_address(udev, req);
}
