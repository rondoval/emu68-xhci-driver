/* SPDX-License-Identifier: GPL-2.0-only */

#include <debug.h>
#include <config.h>

#include <iomem.h>
#include <memory.h>
#include <timing.h>

#include <xhci/xhci.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-context.h>
#include <xhci/xhci-descriptors.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci-ring.h>
#include <devices/hcd_api.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-commands] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef TRACE
#undef KprintfT
#define KprintfT(fmt, ...) PrintPistorm("[xhci-commands] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

struct pending_command; /* forward declaration */
typedef void (*command_handler)(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event);

struct pending_command
{
    struct MinNode node;
    dma_addr_t cmd_trb_dma;  /* value from queue_trb */
    struct usb_device *udev; /* for slot/endpoint checks */
    u8 ep_index;             /* endpoint index encoded into the command */
    command_handler complete;
    struct USBIORequest *req; /* to continue control transfers */
    trb_type type;            /* command type */
    BOOL deadline_active;
    u32 deadline_us;
};

static const command_handler command_handlers[];

#ifdef TRACE
static u32 xhci_pending_command_count(struct xhci_ctrl *ctrl)
{
    u32 count = 0;

    for (struct MinNode *node = ctrl->pending_commands.mlh_Head; node->mln_Succ; node = node->mln_Succ)
        count++;

    return count;
}
#endif

#ifdef DEBUG
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
#endif /* DEBUG (xhci_command_type_name) */

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
            Kprintf("Address Device timed out for slot %lu\n", (ULONG)cmd->udev->slot_id);
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
static void xhci_queue_command(struct xhci_ctrl *ctrl, dma_addr_t addr, u32 slot_id, u8 ep_index, trb_type cmd, struct USBIORequest *req, struct usb_device *udev)
{

    dma_addr_t trb_dma = xhci_ring_enqueue_command(ctrl->cmd_ring, addr, slot_id, ep_index, cmd);
    if (trb_dma == NULL)
    {
        Kprintf("Failed to queue command TRB for cmd %s\n", xhci_command_type_name(cmd));
        return;
    }

    /* Add command handler to pending list */
    struct pending_command *pending_cmd = pool_zalloc(ctrl->metaPool, sizeof(struct pending_command));
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

    KprintfT("Queued command type=%s trb_dma=%lx ptr=%lx slot=%lu ep=%lu vaddr=%ld pending=%lu abort=%ld\n",
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
        xhci_db_ring(ctrl->dba, 0, DB_VALUE_HOST);
}

/*
 * Command handlers
 */

static void handle_reset_ep(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    (void)ctrl;
    const u32 flags = le32(event->event_cmd.flags);
    const u32 slot_id = cmd->udev->slot_id;
    const u8 ep_index = cmd->ep_index;

    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(cmd->udev, ep_index);
    if (!ep_ctx)
    {
        Kprintf("No ep context for addr %lu ep %lu\n", (ULONG)cmd->udev->virtual_address, (ULONG)ep_index);
        return;
    }

    if (TRB_TO_SLOT_ID(flags) != slot_id)
    {
        Kprintf("Expected a TRB for slot %lu, got %lu\n", (ULONG)slot_id, (ULONG)TRB_TO_SLOT_ID(flags));
        xhci_ep_set_failed(ep_ctx);
        return;
    }

    /* COMP_CTX_STATE = the endpoint was not Halted (raced out of it, or is in
     * Error state) - the ring flush via Set TR Dequeue is still the right
     * recovery, so fall through instead of wedging in RESETTING. */
    xhci_comp_code comp = GET_COMP_CODE(le32(event->event_cmd.status));
    if (comp != COMP_SUCCESS && comp != COMP_CTX_STATE)
    {
        Kprintf("Reset EP %lu failed with completion code %lu\n", (ULONG)ep_index, (ULONG)comp);
        xhci_ep_set_failed(ep_ctx);
        return;
    }

    struct xhci_ring *ring = xhci_ep_get_ring(ep_ctx);
    u32 deq_ptr = xhci_ring_get_new_dequeue_ptr(ring);

    KprintfT("Reset EP %lu completed (comp=%lu)\n", (ULONG)ep_index, (ULONG)comp);
    xhci_set_deq_pointer(cmd->udev, ep_index, deq_ptr);
}

static void handle_set_deq(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    (void)ctrl;
    u32 flags = le32(event->event_cmd.flags);
    u32 slot_id = cmd->udev->slot_id;
    u8 ep_index = cmd->ep_index;
    xhci_comp_code comp = GET_COMP_CODE(le32(event->event_cmd.status));
    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(cmd->udev, ep_index);
    if (!ep_ctx)
    {
        Kprintf("No ep context for addr %lu ep %lu\n", (ULONG)cmd->udev->virtual_address, (ULONG)ep_index);
        return;
    }

    if (TRB_TO_SLOT_ID(flags) != slot_id || comp != COMP_SUCCESS)
    {
        Kprintf("Expected a TRB for slot %lu with SUCCESS, got %lu with %lu\n",
                (ULONG)slot_id,
                (ULONG)TRB_TO_SLOT_ID(flags),
                (ULONG)comp);
        xhci_ep_set_failed(ep_ctx);
        return;
    }
    KprintfT("Set DEQ for EP %lu completed successfully, status code %lu (success=1)\n", (ULONG)ep_index, (ULONG)comp);

    if (xhci_ep_get_state(ep_ctx) == USB_DEV_EP_STATE_RESETTING)
    {
        KprintfT("EP %lu was resetting, completing reset\n", (ULONG)ep_index);
        /*
         * If this is due to e.g. STALL recovery, we need to sort out the device itself...:
         * issue ClearFeature(CLEAR_TT_BUFFER) to the hub if its control or bulk ep and dev is behind a TT
         * if not control ep,  issue ClearFeature(ENDPOINT_HALT) to the device.
         * We'll do that by pushing these to fron of the pending queue.
         */
        s32 ep_type = xhci_ep_type_for_index(cmd->udev, ep_index);

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

    /* A BOS/hub pre-fetch on EP0 may have STALLed or timed out mid-enumeration,
     * deferring the device's SET_CONFIGURATION until EP0 was recovered.  Both
     * the STALL (handle_reset_ep) and timeout (handle_stop_ring) recovery paths
     * converge here with EP0 now idle and its ring clean (dequeue == enqueue),
     * so this is the one safe point to resume a deferred configuration. */
    if (ep_index == 0)
        xhci_udev_op_advance(cmd->udev, UDEV_OP_EVENT_EP0_RECOVERED);
}

static void handle_stop_ring(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    (void)ctrl;
    u32 flags = le32(event->event_cmd.flags);
    trb_type type = TRB_FIELD_TO_TYPE(flags);
    xhci_comp_code comp = GET_COMP_CODE(le32(event->event_cmd.status));
    u32 slot_id = cmd->udev->slot_id;
    u8 ep_index = cmd->ep_index;
    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(cmd->udev, ep_index);
    if (!ep_ctx)
    {
        Kprintf("No ep context for addr %lu ep %lu\n", (ULONG)cmd->udev->virtual_address, (ULONG)ep_index);
        return;
    }

    if (type != TRB_COMPLETION || TRB_TO_SLOT_ID(flags) != slot_id)
    {
        Kprintf("Expected a TRB for slot %lu completion, got %lu with %lu\n", (ULONG)slot_id, (ULONG)TRB_TO_SLOT_ID(flags), (ULONG)comp);
        xhci_ep_set_failed(ep_ctx);
        return;
    }

    /* Suspend stop (port about to be directed to U3): the ring and its queued
     * TDs stay untouched for the resume; just advance the suspend sequence.
     * Counted even on an unexpected completion code so the port suspend can't
     * wedge. */
    if (xhci_ep_get_state(ep_ctx) == USB_DEV_EP_STATE_SUSPENDED)
    {
        if (comp != COMP_SUCCESS && comp != COMP_CTX_STATE)
            Kprintf("Suspend stop EP %lu: unexpected completion code %lu\n", (ULONG)ep_index, (ULONG)comp);
        xhci_udev_op_advance(cmd->udev, UDEV_OP_EVENT_STOP_DONE);
        return;
    }

    if (comp != COMP_SUCCESS && comp != COMP_CTX_STATE)
    {
        Kprintf("Stop EP %lu failed with completion code %lu\n", (ULONG)ep_index, (ULONG)comp);
        xhci_ep_set_failed(ep_ctx);
        return;
    }

    KprintfT("Stopped EP %lu with completion code %lu\n", (ULONG)ep_index, (ULONG)comp);

    dma_addr_t deq_ptr = 0;
    xhci_ep_process_stop(ep_ctx, &deq_ptr);

    if (deq_ptr)
    {
        xhci_set_deq_pointer(cmd->udev, ep_index, (u32)deq_ptr);
        return;
    }

    /* ordinary stop command */
    xhci_ep_set_failed(ep_ctx);

    struct xhci_ring *ring = xhci_ep_get_ring(ep_ctx);
    deq_ptr = xhci_ring_get_new_dequeue_ptr(ring);
    xhci_set_deq_pointer(cmd->udev, ep_index, (u32)deq_ptr);
}

/*
 * so this one is awkward, because we do this just before doing control xfers
 * to make sure the endpoint is in the right state.
 * So after this is handled, we need to continue with the control xfer.
 */
static void handle_config_ep(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    (void)ctrl;
    const u32 status = le32(event->event_cmd.status);
    xhci_comp_code comp = GET_COMP_CODE(status);
#ifdef TRACE
    const u32 flags = le32(event->event_cmd.flags);
    const u32 slot_id = TRB_TO_SLOT_ID(flags);
    trb_type type = cmd->type;
    const char *type_name = xhci_command_type_name(type);
    if (type == TRB_EVAL_CONTEXT && cmd->udev)
    {
        xhci_dump_slot_ctx("[xhci-commands] handle_config_ep:", cmd->udev, TRUE);
        xhci_dump_slot_ctx("[xhci-commands] handle_config_ep:", cmd->udev, FALSE);
    }
#endif

    /* COMP_MEL_ERR (29): xHC rejected MAX_EXIT as too large for the current
     * schedule.  The ELD in bits 23:0 of status tells us by how much to
     * reduce.  Patch in_ctx and re-issue TRB_CONFIG_EP (spec §4.23.5.2). */
    if (comp == COMP_MEL_ERR &&
        (cmd->type == TRB_CONFIG_EP || cmd->type == TRB_EVAL_CONTEXT) && cmd->udev)
    {
        struct usb_device *udev = cmd->udev;
        u32 eld = EVENT_TRB_LEN(status);
        if (udev->mel_retry_count >= 3)
        {
            Kprintf("MEL retry limit reached for slot %lu (eld=%lu), giving up\n",
                    (ULONG)udev->slot_id, (ULONG)eld);
            xhci_udev_op_cancel(udev, UDEV_OP_LPM_ENABLE, ERR_NO_ERROR);
            if (cmd->req)
                xhci_udev_io_reply_failed(udev->controller, cmd->req, ERR_HCI_ERROR);
            return;
        }
        udev->max_exit_latency_us = (udev->max_exit_latency_us > eld)
                                        ? udev->max_exit_latency_us - eld
                                        : 0;
        udev->mel_retry_count++;
        KprintfT("COMP_MEL_ERR slot %lu: eld=%lu new_mel=%lu retry=%lu\n",
                 (ULONG)udev->slot_id, (ULONG)eld,
                 (ULONG)udev->max_exit_latency_us, (ULONG)udev->mel_retry_count);
        xhci_update_mel_in_input_ctx(udev);
        xhci_configure_endpoints(udev, cmd->type == TRB_EVAL_CONTEXT, cmd->req);
        return;
    }

    if (comp != COMP_SUCCESS)
    {
        KprintfT("ERROR: %s command for slot %lu returned completion code 0x%lx.\n", type_name, (ULONG)slot_id, (ULONG)comp);
        if (cmd->type == TRB_EVAL_CONTEXT && cmd->udev)
            xhci_udev_op_cancel(cmd->udev, UDEV_OP_LPM_ENABLE, ERR_NO_ERROR);
        return;
    }

    KprintfT("%s command for slot %lu completed successfully\n", type_name, (ULONG)slot_id);

    cmd->udev->mel_retry_count = 0;

    cmd->udev->slot_state = USB_DEV_SLOT_STATE_CONFIGURED;

    /* LPM enable starts from the SET_CONFIGURATION wire completion
     * (xhci_udev_parse_control_message): the device rejects
     * SET_FEATURE(U1/U2_ENABLE) until it is in the Configured state.  The MEL
     * Evaluate Context it issues resumes the sequence here. */
    if (cmd->type == TRB_EVAL_CONTEXT)
        xhci_udev_op_advance(cmd->udev, UDEV_OP_EVENT_MEL_EVAL_DONE);

    if (cmd->req)
    {
        u32 timeout = XHCI_TIMEOUT;
        if (cmd->req->flags & DRIVER_FLAG_TIMEOUT_DEFINED)
            timeout = cmd->req->timeout;

        xhci_ring_enqueue_td(cmd->udev, cmd->req, timeout, FALSE);
    }
}

static void handle_enable_slot(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    const u32 status = le32(event->event_cmd.status);
    const u32 flags = le32(event->event_cmd.flags);

    KprintfT("event status=%08lx flags=%08lx\n", (ULONG)status, (ULONG)flags);
    if (GET_COMP_CODE(status) != COMP_SUCCESS)
    {
        Kprintf("ERROR: Enable Slot command failed.\n");
        xhci_udev_io_reply_failed(ctrl, cmd->req, ERR_HCI_ERROR);
        return;
    }

    struct usb_device *udev = cmd->udev;
    const u32 slot_id = TRB_TO_SLOT_ID(flags);

    udev->slot_id = slot_id & 0xffU;
    udev->slot_state = USB_DEV_SLOT_STATE_ENABLED;
    ctrl->devices_by_slot_id[slot_id] = udev;
    KprintfT("assigned slot_id=%lu for addr=%lu\n", (ULONG)slot_id, (ULONG)udev->virtual_address);

    /* Point to output device context in dcbaa. */
    ctrl->dcbaa->dev_context_ptrs[slot_id] = le64((dma_addr_t)udev->out_ctx->bytes);

    xhci_flush_cache(&ctrl->dcbaa->dev_context_ptrs[slot_id], sizeof(__le64), 0);
    KprintfT("DCBAA[%lu]=%lx\n", (ULONG)slot_id, (ULONG)le64(ctrl->dcbaa->dev_context_ptrs[slot_id]));

    // Continue with Address Device command, passing cmd->req
    xhci_address_device(udev, cmd->req);
}

static void handle_disable_slot(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    (void)ctrl;
    const u32 status = le32(event->event_cmd.status);
    const xhci_comp_code comp = GET_COMP_CODE(status);

    KprintfT("event status=%08lx flags=%08lx\n", (ULONG)status, (ULONG)le32(event->event_cmd.flags));
    if (comp != COMP_SUCCESS)
    {
        Kprintf("ERROR: Disable Slot command failed for slot %lu (comp=%lu).\n",
                (ULONG)cmd->udev->slot_id, (ULONG)comp);
    }
    else
    {
        KprintfT("Disabled slot %lu successfully (comp=%lu).\n", (ULONG)cmd->udev->slot_id, (ULONG)comp);
    }
    cmd->udev->slot_state = USB_DEV_SLOT_STATE_DISABLED;
    xhci_udev_free(cmd->udev);
}

static void handle_address_device(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    (void)ctrl;
    const u32 status = le32(event->event_cmd.status);
    const xhci_comp_code comp = GET_COMP_CODE(status);

    KprintfT("event status=%08lx flags=%08lx\n", (ULONG)status, (ULONG)le32(event->event_cmd.flags));

    s8 err = ERR_NO_ERROR;
    switch (comp)
    {
    case COMP_CTX_STATE:
    case COMP_EBADSLT:
        Kprintf("Setup ERROR: address device command for slot %lu.\n", (ULONG)cmd->udev->slot_id);
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
        KprintfT("Successful Address Device command\n");
        break;
    default:
        Kprintf("ERROR: unexpected command completion code 0x%lx.\n",
                GET_COMP_CODE(le32(event->event_cmd.status)));
        err = ERR_HCI_ERROR;
        break;
    }

    if (err)
    {
        if (cmd->udev)
        {
            Kprintf("Address Device failure for slot %lu (code %lu)\n", (ULONG)cmd->udev->slot_id, (ULONG)err);
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

    cmd->udev->xhci_address = xhci_get_hardware_address(cmd->udev) & 0xffU;
    cmd->udev->slot_state = USB_DEV_SLOT_STATE_ADDRESSED;
    KprintfT("Assigned xHCI address %lu to slot %lu (Virtual address %lu)\n",
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
            s8 ret = xhci_udev_send_ctrl(cmd->udev, cmd->req);
            if (ret != ERR_NO_ERROR)
                xhci_udev_io_reply_failed(ctrl, cmd->req, ret);
        }
    }
}

static void handle_reset_device(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event)
{
    (void)ctrl;
#ifndef DEBUG
    (void)cmd; /* only referenced by debug logging below */
#endif
    const u32 status = le32(event->event_cmd.status);

    KprintfT("event status=%08lx flags=%08lx\n", (ULONG)status, (ULONG)le32(event->event_cmd.flags));
    if (GET_COMP_CODE(status) != COMP_SUCCESS)
    {
        Kprintf("ERROR: Reset Device command failed for slot %lu.\n", (ULONG)cmd->udev->slot_id);
        return;
    }

    KprintfT("Reset Device for slot %lu completed successfully.\n", (ULONG)cmd->udev->slot_id);
}

/*
 * Mapping of commands to their handlers
 */
static const command_handler command_handlers[] = {
    [TRB_ENABLE_SLOT] = handle_enable_slot,   /* Enable Slot Command */
    [TRB_DISABLE_SLOT] = handle_disable_slot, /* Disable Slot Command */
    [TRB_ADDR_DEV] = handle_address_device,   /* Address Device Command */
    [TRB_CONFIG_EP] = handle_config_ep,       /* Configure Endpoint Command */
    [TRB_EVAL_CONTEXT] = handle_config_ep,    /* Evaluate Context Command */
    [TRB_RESET_EP] = handle_reset_ep,         /* Reset Endpoint Command */
    [TRB_STOP_RING] = handle_stop_ring,       /* Stop Transfer Ring Command */
    [TRB_SET_DEQ] = handle_set_deq,           /* Set Transfer Ring Dequeue Pointer Command */
    [TRB_RESET_DEV] = handle_reset_device,    /* Reset Device Command */
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

    u32 now_us = get_time();
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
    const xhci_comp_code comp = (xhci_comp_code)GET_COMP_CODE(le32(event->event_cmd.status));
    const dma_addr_t trb_addr = (dma_addr_t)le64(event->event_cmd.cmd_trb);

    /* COMP_CMD_STOP means the command ring has stopped after a CA abort.
     * The aborted command is handled by COMP_CMD_ABORT below; here we only
     * clear the abort state and restart the ring for queued commands. */
    if (comp == COMP_CMD_STOP)
    {
        Kprintf("Command Ring Stopped; restarting\n");

        struct pending_command *timed_out_cmd = NULL;
        struct MinNode *head = ctrl->pending_commands.mlh_Head;
        if (head->mln_Succ)
        {
            struct pending_command *head_cmd = (struct pending_command *)head;
            if (!head_cmd->deadline_active)
                timed_out_cmd = head_cmd;
        }

        ctrl->cmd_abort_pending = FALSE;

        if (timed_out_cmd)
        {
            Kprintf("Missing Command Abort completion; failing timed out %s slot=%ld trb_dma=%lx on Command Ring Stop\n",
                    xhci_command_type_name(timed_out_cmd->type),
                    timed_out_cmd->udev ? (LONG)timed_out_cmd->udev->slot_id : -1L,
                    (ULONG)timed_out_cmd->cmd_trb_dma);
            Remove((struct Node *)timed_out_cmd);
            xhci_fail_timed_out_command(ctrl, timed_out_cmd);
            pool_free(ctrl->metaPool, timed_out_cmd);
        }

        /*
         * After a Command Abort the HC has stopped on the old command ring
         * dequeue pointer. Reprogram CRCR to the next software enqueue
         * position so restart resumes with queued commands instead of the
         * aborted TRB.
         */
        u64 trb_64 = xhci_ring_get_new_dequeue_ptr(ctrl->cmd_ring);
        xhci_writeq(&ctrl->hcor->or_crcr,
                    trb_64 & (u64)~CMD_RING_ADDR_MASK);

        /* Restart only if there are still pending commands. */
        if (ctrl->pending_commands.mlh_Head->mln_Succ)
            xhci_db_ring(ctrl->dba, 0, DB_VALUE_HOST);
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
        pool_free(ctrl->metaPool, cmd);
        return;
    }

    if (cmd)
    {
        if (cmd->complete)
            cmd->complete(ctrl, cmd, event);
        else
            Kprintf("No handler for command TRB %lx\n", cmd->cmd_trb_dma);

        Remove((struct Node *)cmd);
        pool_free(ctrl->metaPool, cmd);
        return;
    }

    Kprintf("No matching pending command for command completion event TRB (%08lx %08lx %08lx %08lx)\n",
            (ULONG)le32(event->generic.field[0]),
            (ULONG)le32(event->generic.field[1]),
            (ULONG)le32(event->generic.field[2]),
            (ULONG)le32(event->generic.field[3]));
}

/*
 * Recover a halted or errored endpoint.  Reset Endpoint is only valid in the
 * Halted state (xHCI 4.6.8); an endpoint in the Error state (control endpoints,
 * xHCI 4.8.3) skips it and goes straight to Set TR Dequeue - issuing the reset
 * there would just fail with COMP_CTX_STATE.  Both paths converge in
 * handle_set_deq() with the ring flushed.
 */
void xhci_reset_ep(struct usb_device *udev, u8 ep_index)
{
    struct xhci_ctrl *ctrl = udev->controller;
    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
    if (!ep_ctx)
    {
        Kprintf("No ep context for addr %lu ep %lu\n", (ULONG)udev->virtual_address, (ULONG)ep_index);
        return;
    }

    xhci_ep_set_resetting(ep_ctx);

    if (xhci_read_hw_ep_state(udev, ep_index) == EP_STATE_ERROR)
    {
        KprintfT("EP %lu in Error state, skipping Reset Endpoint\n", (ULONG)ep_index);
        struct xhci_ring *ring = xhci_ep_get_ring(ep_ctx);
        xhci_set_deq_pointer(udev, ep_index, xhci_ring_get_new_dequeue_ptr(ring));
        return;
    }

    // set TSP=0 - reset split transaction, flush cached TDs
    xhci_queue_command(ctrl, 0, udev->slot_id, ep_index, TRB_RESET_EP, NULL, udev); // handle_reset_ep
}

/*
 * Stops transfer processing for an endpoint/ring. Used for endpoint reset and stall recovery, and also for aborting transfers on disconnect.
 * After the ring is stopped, we can set the dequeue pointer to the current enqueue pointer to flush any pending transfers and then restart the ring to continue processing new transfers.
 * The endpoint needs be in either Running or Halted state, otherwise this command will fail.
 */
void xhci_stop_ring(struct usb_device *udev, u8 ep_index)
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
void xhci_set_deq_pointer(struct usb_device *udev, u8 ep_index, u32 deq_ptr)
{
    struct xhci_ctrl *ctrl = udev->controller;
    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
    if (!ep_ctx)
    {
        Kprintf("No ep context for addr %lu ep %lu\n", (ULONG)udev->virtual_address, (ULONG)ep_index);
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
    xhci_queue_command(ctrl, 0, udev->slot_id, 0, TRB_RESET_DEV, NULL, udev);
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

    xhci_flush_cache(in_ctx->bytes, in_ctx->size, 0);
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
    for (u8 ep_index = 0; ep_index < USB_MAX_ENDPOINT_CONTEXTS; ep_index++)
    {
        struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
        if (ep_ctx)
            xhci_ep_set_aborting(ep_ctx);
    }

    if (udev->slot_state == USB_DEV_SLOT_STATE_DISABLED)
    {
        KprintfT("queue DISABLE_SLOT skipped; slot_id=%lu already disabled\n", (ULONG)udev->slot_id);
        return;
    }

    udev->slot_state = USB_DEV_SLOT_STATE_DISABLED;
    xhci_queue_command(ctrl, 0, udev->slot_id, 0, TRB_DISABLE_SLOT, NULL, udev);
}

static void xhci_set_address(struct usb_device *udev, struct USBIORequest *req)
{
    struct xhci_ctrl *ctrl = udev->controller;
    u32 slot_id = udev->slot_id;

    /* If already addressed (internal address non-zero), don't re-issue. */
    if (udev->slot_state >= USB_DEV_SLOT_STATE_ADDRESSED)
    {
        KprintfT("slot %lu already addressed (xhci_address=0x%lx), skipping.\n",
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
    xhci_setup_addressable_virt_dev(udev);

    KprintfT("queue ADDR_DEV cmd, in_ctx->bytes=%lx addr=%lu slot=%lu parent_addr=%lu parent_port=%lu route=0x%lx, depth=%lu\n",
             (ULONG)udev->in_ctx->bytes,
             (ULONG)udev->virtual_address,
             (ULONG)slot_id,
             (ULONG)(udev->parent ? udev->parent->virtual_address : 0),
             (ULONG)udev->parent_port,
             (ULONG)udev->route,
             (ULONG)udev->route_depth);

#ifdef TRACE
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
        KprintfT("no slot_id yet; enabling slot...\n");
        xhci_enable_slot(udev, req);
        return;
    }

    xhci_set_address(udev, req);
}
