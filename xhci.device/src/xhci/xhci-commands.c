#include <debug.h>
#include <config.h>
#include <compat.h>

#include <xhci/xhci.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-descriptors.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci-ring.h>
#include <xhci/xhci-context.h>
#include <devices/hcd_api.h>

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
    struct USBIORequest *req; /* to continue control transfers */
    trb_type type;            /* command type */
    BOOL deadline_active;
    ULONG deadline_us;
};

static const command_handler command_handlers[];

#ifdef DEBUG_HIGH
static ULONG xhci_pending_command_count(struct xhci_ctrl *ctrl)
{
    ULONG count = 0;

    for (struct MinNode *node = ctrl->pending_commands.mlh_Head; node->mln_Succ; node = node->mln_Succ)
        count++;

    return count;
}
#endif

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

static inline struct pending_command *xhci_find_pending_command_by_dma(struct xhci_ctrl *ctrl, dma_addr_t trb_dma)
{
    for (struct MinNode *node = ctrl->pending_commands.mlh_Head; node->mln_Succ; node = node->mln_Succ)
    {
        struct pending_command *cmd = (struct pending_command *)node;
        if (cmd->cmd_trb_dma == trb_dma)
            return cmd;
    }

    return NULL;
}

static void xhci_fail_timed_out_command(struct xhci_ctrl *ctrl, struct pending_command *cmd)
{
    if (!cmd)
        return;

    switch (cmd->type)
    {
    case TRB_ENABLE_SLOT:
    case TRB_DISABLE_SLOT:
        Kprintf("%s Slot timed out for slot %ld%s\n",
                cmd->type == TRB_ENABLE_SLOT ? "Enable" : "Disable",
                cmd->udev ? (LONG)cmd->udev->slot_id : -1L,
                cmd->type == TRB_DISABLE_SLOT ? "; forcing software teardown" : "");
        if (cmd->udev)
            xhci_udev_free(cmd->udev);
        break;

    case TRB_ADDR_DEV:
        if (cmd->udev)
        {
            Kprintf("Address Device timed out for slot %ld\n", (LONG)cmd->udev->slot_id);
            xhci_dump_slot_ctx("[xhci-commands] timeout cleanup:", cmd->udev, TRUE);
            xhci_dump_slot_ctx("[xhci-commands] timeout cleanup:", cmd->udev, FALSE);
            xhci_disable_slot(cmd->udev);
        }
        break;

    case TRB_RESET_EP:
    case TRB_STOP_RING:
    case TRB_SET_DEQ:
        if (cmd->udev)
        {
            struct ep_context *ep_ctx = xhci_ep_get_context_for_index(cmd->udev, cmd->ep_index);
            if (ep_ctx)
                xhci_ep_set_failed(ep_ctx);
        }
        break;

    default:
        break;
    }

    if (cmd->req)
        xhci_udev_io_reply_failed(ctrl, cmd->req, ERR_TIMEOUT);
}

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
static void xhci_queue_command(struct xhci_ctrl *ctrl, dma_addr_t addr, u32 slot_id, u32 ep_index, trb_type cmd, struct USBIORequest *req, struct usb_device *udev)
{

    dma_addr_t trb_dma = xhci_ring_enqueue_command(ctrl->cmd_ring, addr, slot_id, ep_index, cmd);
    if (trb_dma == NULL)
    {
        Kprintf("Failed to queue command TRB for cmd %s\n", xhci_command_type_name(cmd));
        return;
    }

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
    pending_cmd->type = cmd;
    pending_cmd->deadline_us = get_time() + CMD_TIMEOUT_MS * 1000UL;
    pending_cmd->deadline_active = TRUE;

    pending_cmd->complete = command_handlers[cmd];
    AddTailMinList(&ctrl->pending_commands, (struct MinNode *)pending_cmd);

    KprintfH("Queued command type=%s trb_dma=%lx ptr=%lx slot=%lu ep=%lu vaddr=%ld pending=%lu abort=%ld\n",
             xhci_command_type_name(cmd),
             (ULONG)trb_dma,
             (ULONG)addr,
             (ULONG)slot_id,
             (ULONG)ep_index,
             (ULONG)(udev ? udev->virtual_address : 0),
             xhci_pending_command_count(ctrl),
             (LONG)ctrl->cmd_abort_pending);

    /* Ring the command ring doorbell — suppressed while an abort is in
     * progress; COMP_CMD_STOP will restart the ring once the HC has stopped. */
    if (!ctrl->cmd_abort_pending)
        writel(DB_VALUE_HOST, &ctrl->dba->doorbell[0]);
}

/*
 * Command handlers
 */

static void handle_reset_ep(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    (void)ctrl;
    const u32 flags = LE32(event->event_cmd.flags);
    const ULONG slot_id = cmd->udev->slot_id;
    const ULONG ep_index = cmd->ep_index;

    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(cmd->udev, ep_index);
    if (!ep_ctx)
    {
        Kprintf("No ep context for addr %ld ep %ld\n", (LONG)cmd->udev->virtual_address, (LONG)ep_index);
        return;
    }

    if (TRB_TO_SLOT_ID(flags) != slot_id)
    {
        Kprintf("Expected a TRB for slot %ld, got %ld\n", slot_id, TRB_TO_SLOT_ID(flags));
        xhci_ep_set_failed(ep_ctx);
        return;
    }
    struct xhci_ring *ring = xhci_ep_get_ring(ep_ctx);
    u32 deq_ptr = xhci_ring_get_new_dequeue_ptr(ring);

    KprintfH("Reset EP %ld completed successfully\n", ep_index);
    xhci_set_deq_pointer(cmd->udev, ep_index, deq_ptr);
}

static void handle_set_deq(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    (void)ctrl;
    u32 flags = LE32(event->event_cmd.flags);
    ULONG slot_id = cmd->udev->slot_id;
    ULONG ep_index = cmd->ep_index;
    xhci_comp_code comp = GET_COMP_CODE(LE32(event->event_cmd.status));
    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(cmd->udev, ep_index);
    if (!ep_ctx)
    {
        Kprintf("No ep context for addr %ld ep %ld\n", (LONG)cmd->udev->virtual_address, (LONG)ep_index);
        return;
    }

    if (TRB_TO_SLOT_ID(flags) != slot_id || comp != COMP_SUCCESS)
    {
        Kprintf("Expected a TRB for slot %ld with SUCCESS, got %ld with %ld\n",
                slot_id,
                TRB_TO_SLOT_ID(flags),
                comp);
        xhci_ep_set_failed(ep_ctx);
        return;
    }
    KprintfH("Set DEQ for EP %ld completed successfully, status code %ld (success=1)\n", ep_index, comp);

    if (xhci_ep_get_state(ep_ctx) == USB_DEV_EP_STATE_RESETTING)
    {
        KprintfH("EP %ld was resetting, completing reset\n", ep_index);
        /*
         * If this is due to e.g. STALL recovery, we need to sort out the device itself...:
         * issue ClearFeature(CLEAR_TT_BUFFER) to the hub if its control or bulk ep and dev is behind a TT
         * if not control ep,  issue ClearFeature(ENDPOINT_HALT) to the device.
         * We'll do that by pushing these to fron of the pending queue.
         */
        int ep_type = xhci_ep_type_for_index(cmd->udev, ep_index);

        if (ep_index != 0 && ep_type != USB_ENDPOINT_XFER_CONTROL)
        {
            /* Dispatch deferred device-side CLEAR_FEATURE after host recovery. */
            xhci_udev_clear_feature_halt(cmd->udev, ep_index);
        }

        /* For control/bulk endpoints behind a TT, clear the TT buffer on the hub. */
        if (cmd->udev->speed == USB_SPEED_FULL || cmd->udev->speed == USB_SPEED_LOW)
        {
            if (ep_type == USB_ENDPOINT_XFER_CONTROL || ep_type == USB_ENDPOINT_XFER_BULK)
            {
                xhci_udev_clear_tt_buffer(cmd->udev, ep_index, ep_type);
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
    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(cmd->udev, ep_index);
    if (!ep_ctx)
    {
        Kprintf("No ep context for addr %ld ep %ld\n", (LONG)cmd->udev->virtual_address, (LONG)ep_index);
        return;
    }

    if (type != TRB_COMPLETION || TRB_TO_SLOT_ID(flags) != slot_id)
    {
        Kprintf("Expected a TRB for slot %ld completion, got %ld with %ld\n", slot_id, TRB_TO_SLOT_ID(flags), comp);
        xhci_ep_set_failed(ep_ctx);
        return;
    }

    if (comp != COMP_SUCCESS && comp != COMP_CTX_STATE)
    {
        Kprintf("Stop EP %ld failed with completion code %ld\n", (LONG)ep_index, (LONG)comp);
        xhci_ep_set_failed(ep_ctx);
        return;
    }

    KprintfH("Stopped EP %ld with completion code %ld\n", ep_index, comp);

    u32 deq_ptr = 0;
    xhci_ep_process_stop(ep_ctx, &deq_ptr);

    if (deq_ptr)
    {
        xhci_set_deq_pointer(cmd->udev, ep_index, deq_ptr);
        return;
    }

    /* ordinary stop command */
    xhci_ep_set_failed(ep_ctx);

    struct xhci_ring *ring = xhci_ep_get_ring(ep_ctx);
    deq_ptr = xhci_ring_get_new_dequeue_ptr(ring);
    xhci_set_deq_pointer(cmd->udev, ep_index, deq_ptr);
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

#ifdef DEBUG_HIGH
    if (type == TRB_EVAL_CONTEXT && cmd->udev)
    {
        xhci_dump_slot_ctx("[xhci-commands] handle_config_ep:", cmd->udev, TRUE);
        xhci_dump_slot_ctx("[xhci-commands] handle_config_ep:", cmd->udev, FALSE);
    }
#endif

    if (comp != COMP_SUCCESS)
    {
        KprintfH("ERROR: %s command for slot %ld returned completion code 0x%lx.\n", type_name, TRB_TO_SLOT_ID(LE32(event->event_cmd.flags)), (ULONG)comp);
        return;
    }

    KprintfH("%s command for slot %ld completed successfully\n", type_name, TRB_TO_SLOT_ID(LE32(event->event_cmd.flags)));

    cmd->udev->slot_state = USB_DEV_SLOT_STATE_CONFIGURED;

    if (cmd->req)
    {
        unsigned int timeout = XHCI_TIMEOUT;
        if (cmd->req->flags & DRIVER_FLAG_TIMEOUT_DEFINED)
            timeout = cmd->req->timeout;

        xhci_ring_enqueue_td(cmd->udev, cmd->req, timeout, FALSE);
    }
}

static void handle_enable_slot(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    KprintfH("event status=%08lx flags=%08lx\n", (ULONG)LE32(event->event_cmd.status), (ULONG)LE32(event->event_cmd.flags));
    if (GET_COMP_CODE(LE32(event->event_cmd.status)) != COMP_SUCCESS)
    {
        Kprintf("ERROR: Enable Slot command failed.\n");
        xhci_udev_io_reply_failed(ctrl, cmd->req, ERR_HCI_ERROR);
        return;
    }

    struct usb_device *udev = cmd->udev;
    int slot_id = TRB_TO_SLOT_ID(LE32(event->event_cmd.flags));

    udev->slot_id = slot_id;
    udev->slot_state = USB_DEV_SLOT_STATE_ENABLED;
    ctrl->devices_by_slot_id[slot_id] = udev;
    KprintfH("assigned slot_id=%ld for addr=%lu\n", (ULONG)slot_id, (ULONG)udev->virtual_address);

    /* Point to output device context in dcbaa. */
    ctrl->dcbaa->dev_context_ptrs[slot_id] = LE64((dma_addr_t)udev->out_ctx->bytes);

    xhci_flush_cache(&ctrl->dcbaa->dev_context_ptrs[slot_id], sizeof(__le64));
    KprintfH("DCBAA[%ld]=%lx\n", (ULONG)slot_id, (ULONG)LE64(ctrl->dcbaa->dev_context_ptrs[slot_id]));

    // Continue with Address Device command, passing cmd->req
    xhci_address_device(udev, cmd->req);
}

static void handle_disable_slot(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    (void)ctrl;
    KprintfH("event status=%08lx flags=%08lx\n", (ULONG)LE32(event->event_cmd.status), (ULONG)LE32(event->event_cmd.flags));
    xhci_comp_code comp = GET_COMP_CODE(LE32(event->event_cmd.status));

    if (comp != COMP_SUCCESS)
    {
        Kprintf("ERROR: Disable Slot command failed for slot %ld (comp=%ld).\n",
                (ULONG)cmd->udev->slot_id, comp);
    }
    else
    {
        KprintfH("Disabled slot %ld successfully (comp=%ld).\n", cmd->udev->slot_id, comp);
    }
    cmd->udev->slot_state = USB_DEV_SLOT_STATE_DISABLED;
    xhci_udev_free(cmd->udev);
}

static void handle_address_device(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    (void)ctrl;
    KprintfH("event status=%08lx flags=%08lx\n", (ULONG)LE32(event->event_cmd.status), (ULONG)LE32(event->event_cmd.flags));

    int err = ERR_NO_ERROR;
    switch (GET_COMP_CODE(LE32(event->event_cmd.status)))
    {
    case COMP_CTX_STATE:
    case COMP_EBADSLT:
        Kprintf("Setup ERROR: address device command for slot %ld.\n", cmd->udev->slot_id);
        err = ERR_HCI_ERROR;
        break;
    case COMP_TX_ERR:
        Kprintf("Device not responding to set address.\n");
        err = ERR_TIMEOUT;
        break;
    case COMP_DEV_ERR:
        Kprintf("ERROR: Incompatible device for address device command.\n");
        err = ERR_BAD_PARAMETERS;
        break;
    case COMP_SUCCESS:
        KprintfH("Successful Address Device command\n");
        break;
    default:
        Kprintf("ERROR: unexpected command completion code 0x%lx.\n",
                GET_COMP_CODE(LE32(event->event_cmd.status)));
        err = ERR_HCI_ERROR;
        break;
    }

    if (err)
    {
        if (cmd->udev)
        {
            Kprintf("Address Device failure for slot %ld (code %ld)\n", (ULONG)cmd->udev->slot_id, (ULONG)err);
            xhci_dump_slot_ctx("[xhci-commands] handle_address_device:", cmd->udev, TRUE);
            xhci_dump_slot_ctx("[xhci-commands] handle_address_device:", cmd->udev, FALSE);
        }
        /*
         * Unsuccessful Address Device command shall leave the
         * slot in default state. So, issue Disable Slot command now.
         */
        xhci_disable_slot(cmd->udev);
        if (cmd->req)
            xhci_udev_io_reply_failed(ctrl, cmd->req, err);
        return;
    }

    cmd->udev->xhci_address = xhci_get_hardware_address(cmd->udev);
    cmd->udev->slot_state = USB_DEV_SLOT_STATE_ADDRESSED;
    KprintfH("Assigned xHCI address %ld to slot %ld (Virtual address %ld)\n",
             (ULONG)cmd->udev->xhci_address, (ULONG)cmd->udev->slot_id, (cmd->req) ? (ULONG)cmd->req->virtual_address : (ULONG)0);

    /* Continue the original request after the device is addressed. */
    if (cmd->req)
    {
        struct USBSetupPacket *setup = &cmd->req->setup;
        BOOL is_set_address = (setup->bRequest == USB_REQ_SET_ADDRESS) &&
                              ((setup->bmRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD);

        if (is_set_address)
        {
            /* Upper layer will migrate the context based on virtual address. */
            xhci_udev_io_reply_data(cmd->udev, cmd->req, ERR_NO_ERROR, 0);
        }
        else
        {
            int ret = xhci_udev_send_ctrl(cmd->udev, cmd->req);
            if (ret != ERR_NO_ERROR)
                xhci_udev_io_reply_failed(ctrl, cmd->req, ret);
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

/* Command ring timeout handler
 * Called every UNIT_TASK_POLL_DELAY_MS ticks.
 *
 * Only the HEAD command is ever executing; the rest queue behind it.
 * When the head has been outstanding for CMD_TIMEOUT_TICKS without a
 * hardware completion event, we initiate the xHCI Command Abort sequence
 * per spec §4.6.1.2:
 *   1. Assert CA bit in CRCR.
 *   2. HC generates COMP_CMD_ABORT for the stuck command.
 *   3. HC generates COMP_CMD_STOP.
 *
 * While cmd_abort_pending is TRUE, xhci_queue_command suppresses the
 * doorbell so newly queued TRBs stay queued and are executed when the ring restarts.
 */
void xhci_process_command_timeouts(struct xhci_ctrl *ctrl)
{
    /* Already waiting for COMP_CMD_ABORT + COMP_CMD_STOP events — do nothing. */
    if (ctrl->cmd_abort_pending)
        return;

    /* Only the head command is executing; ignore the rest. */
    struct MinNode *head = ctrl->pending_commands.mlh_Head;
    if (!head->mln_Succ)
        return; /* empty list */

    struct pending_command *cmd = (struct pending_command *)head;
    if (!cmd->deadline_active)
        return; /* abort already fired */

    ULONG now_us = get_time();
    if ((int32_t)(now_us - cmd->deadline_us) < 0)
        return; /* not timed out yet */

    Kprintf("Command timeout: type=%s trb_dma=%lx slot=%ld — asserting Command Abort (CA)\n",
            xhci_command_type_name(cmd->type),
            (ULONG)cmd->cmd_trb_dma,
            cmd->udev ? (LONG)cmd->udev->slot_id : -1L);

    u64 crcr = xhci_readq(&ctrl->hcor->or_crcr);
    if (!(crcr & CMD_RING_RUNNING))
    {
        Kprintf("Command ring already stopped\n");
        // TODO restart controller?
        return;
    }

    ctrl->cmd_abort_pending = TRUE;
    cmd->deadline_active = FALSE;
    /* Ring is running — assert CA and wait for hardware events. */
    xhci_writeq(&ctrl->hcor->or_crcr, CMD_RING_ABORT);
}

/* Command event dispatcher
 * Called when a Command Completion Event TRB is received
 */
void xhci_dispatch_command_event(struct xhci_ctrl *ctrl, union xhci_trb *event)
{
    const xhci_comp_code comp = (xhci_comp_code)GET_COMP_CODE(LE32(event->event_cmd.status));
    const dma_addr_t trb_addr = (dma_addr_t)LE64(event->event_cmd.cmd_trb);

    /* COMP_CMD_STOP means the command ring has stopped after a CA abort.
     * The aborted command is handled by COMP_CMD_ABORT below; here we only
     * clear the abort state and restart the ring for queued commands. */
    if (comp == COMP_CMD_STOP)
    {
        Kprintf("Command Ring Stopped; restarting\n");
        ctrl->cmd_abort_pending = FALSE;
        /* Restart only if there are still pending commands. */
        if (ctrl->pending_commands.mlh_Head->mln_Succ)
            writel(DB_VALUE_HOST, &ctrl->dba->doorbell[0]);
        return;
    }

    struct pending_command *cmd = xhci_find_pending_command_by_dma(ctrl, trb_addr);

    if (comp == COMP_CMD_ABORT)
    {
        if (!cmd)
        {
            Kprintf("No matching pending command for aborted command TRB %lx\n", (ULONG)trb_addr);
            return;
        }

        Kprintf("Command Abort completion for %s slot=%ld trb_dma=%lx\n",
                xhci_command_type_name(cmd->type),
                cmd->udev ? (LONG)cmd->udev->slot_id : -1L,
                (ULONG)cmd->cmd_trb_dma);
        Remove((struct Node *)cmd);
        xhci_fail_timed_out_command(ctrl, cmd);
        FreeVecPooled(ctrl->memoryPool, cmd);
        return;
    }

    if (cmd)
    {
        if (cmd->complete)
            cmd->complete(ctrl, cmd, event);
        else
            Kprintf("No handler for command TRB %lx\n", cmd->cmd_trb_dma);

        Remove((struct Node *)cmd);
        FreeVecPooled(ctrl->memoryPool, cmd);
        return;
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
    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
    if (!ep_ctx)
    {
        Kprintf("No ep context for addr %ld ep %ld\n", (LONG)udev->virtual_address, (LONG)ep_index);
        return;
    }

    xhci_ep_set_resetting(ep_ctx);

    // set TSP=0 - reset split transaction, flush cached TDs
    xhci_queue_command(ctrl, 0, udev->slot_id, ep_index, TRB_RESET_EP, NULL, udev); // handle_reset_ep
}

/*
 * Stops transfer processing for an endpoint/ring. Used for endpoint reset and stall recovery, and also for aborting transfers on disconnect.
 * After the ring is stopped, we can set the dequeue pointer to the current enqueue pointer to flush any pending transfers and then restart the ring to continue processing new transfers.
 * The endpoint needs be in either Running or Halted state, otherwise this command will fail.
 */
void xhci_stop_ring(struct usb_device *udev, u32 ep_index)
{
    if (!udev || !udev->controller)
        return;

    xhci_queue_command(udev->controller, 0, udev->slot_id, ep_index, TRB_STOP_RING, NULL, udev);
}

/*
 * Sets the transfer ring dequeue pointer for the given endpoint.
 * Used after a reset endpoint command to continue processing.
 * The endpoint needs to be either in Error or Stopped state.
 */
void xhci_set_deq_pointer(struct usb_device *udev, u32 ep_index, u32 deq_ptr)
{
    struct xhci_ctrl *ctrl = udev->controller;
    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
    if (!ep_ctx)
    {
        Kprintf("No ep context for addr %ld ep %ld\n", (LONG)udev->virtual_address, (LONG)ep_index);
        return;
    }

    xhci_queue_command(ctrl, deq_ptr, udev->slot_id, ep_index, TRB_SET_DEQ, NULL, udev);
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
void xhci_configure_endpoints(struct usb_device *udev, BOOL ctx_change, struct USBIORequest *req)
{
    struct xhci_ctrl *ctrl = udev->controller;
    struct xhci_container_ctx *in_ctx = udev->in_ctx;

    xhci_flush_cache(in_ctx->bytes, in_ctx->size);
    // TODO support deconfigure - DC flag?
    xhci_queue_command(ctrl, (dma_addr_t)in_ctx->bytes, udev->slot_id, 0, ctx_change ? TRB_EVAL_CONTEXT : TRB_CONFIG_EP, req, udev);
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
void xhci_enable_slot(struct usb_device *udev, struct USBIORequest *req)
{
    struct xhci_ctrl *ctrl = udev->controller;
    xhci_queue_command(ctrl, 0, 0, 0, TRB_ENABLE_SLOT, req, udev);
}

void xhci_disable_slot(struct usb_device *udev)
{
    struct xhci_ctrl *ctrl = udev->controller;
    for (int ep_index = 0; ep_index < USB_MAX_ENDPOINT_CONTEXTS; ep_index++)
    {
        struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
        if (ep_ctx)
            xhci_ep_set_aborting(ep_ctx);
    }

    if (udev->slot_state == USB_DEV_SLOT_STATE_DISABLED)
    {
        KprintfH("queue DISABLE_SLOT skipped; slot_id=%ld already disabled\n", (ULONG)udev->slot_id);
        return;
    }

    udev->slot_state = USB_DEV_SLOT_STATE_DISABLED;
    xhci_queue_command(ctrl, 0, udev->slot_id, 0, TRB_DISABLE_SLOT, NULL, udev);
}

static void xhci_set_address(struct usb_device *udev, struct USBIORequest *req)
{
    struct xhci_ctrl *ctrl = udev->controller;
    unsigned int slot_id = udev->slot_id;

    /* If already addressed (internal address non-zero), don't re-issue. */
    if (udev->slot_state >= USB_DEV_SLOT_STATE_ADDRESSED)
    {
        KprintfH("slot %ld already addressed (xhci_address=0x%lx), skipping.\n",
                 (ULONG)slot_id, (ULONG)udev->xhci_address);
        if (req)
            xhci_udev_io_reply_data(udev, req, ERR_NO_ERROR, 0);
        return;
    }

    udev->parent = ctrl->pending_parent;
    udev->parent_port = ctrl->pending_parent_port;
    udev->speed = ctrl->pending_parent_speed;

    /*
     * This is the first Set Address since device plug-in
     * so setting up the slot context.
     */
    xhci_setup_addressable_virt_dev(ctrl, udev);

    KprintfH("queue ADDR_DEV cmd, in_ctx->bytes=%lx addr=%lu slot=%lu parent_addr=%lu parent_port=%lu route=0x%lx, depth=%ld\n",
             (ULONG)udev->in_ctx->bytes,
             (ULONG)udev->virtual_address,
             (ULONG)slot_id,
             (ULONG)(udev->parent ? udev->parent->virtual_address : 0),
             (ULONG)udev->parent_port,
             (ULONG)udev->route,
             (ULONG)udev->route_depth);

#ifdef DEBUG_HIGH
    /* Dump parent hub's slot context so we can verify DEV_HUB is set */
    if (udev->parent && udev->parent->out_ctx && udev->parent->slot_id != 0)
        xhci_dump_slot_ctx("[xhci-commands] ADDR_DEV parent hub:", udev->parent, FALSE);
#endif

    xhci_queue_command(ctrl, (dma_addr_t)udev->in_ctx->bytes, slot_id, 0, TRB_ADDR_DEV, req, udev);
}

void xhci_address_device(struct usb_device *udev, struct USBIORequest *req)
{
    /*
     * A non-zero slot_id with slot_state==DISABLED means teardown is still in
     * flight (e.g. after a failed/timed-out ADDRESS_DEVICE). Don't reuse that
     * slot for a fresh address attempt until DISABLE_SLOT completes and frees
     * the device context.
     */
    if (udev->slot_id != 0 && udev->slot_state == USB_DEV_SLOT_STATE_DISABLED)
    {
        Kprintf("Refusing Address Device while slot teardown is pending: slot=%lu addr=%lu\n",
                (ULONG)udev->slot_id,
                (ULONG)udev->virtual_address);
        if (req)
            xhci_udev_io_reply_failed(udev->controller, req, ERR_HCI_ERROR);
        return;
    }

    /* If we don't have a slot yet, enable one and allocate Virt Dev */
    if (udev->slot_id == 0)
    {
        KprintfH("no slot_id yet; enabling slot...\n");
        xhci_enable_slot(udev, req);
        return;
    }

    xhci_set_address(udev, req);
}
