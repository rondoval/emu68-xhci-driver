// SPDX-License-Identifier: GPL-2.0+
/*
 * Based on xhci-ring.c from uboot
 * USB HOST XHCI Controller stack
 *
 * Based on xHCI host controller driver in linux-kernel
 * by Sarah Sharp.
 *
 * Copyright (C) 2008 Intel Corp.
 * Author: Sarah Sharp
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 * Authors: Vivek Gautam <gautam.vivek@samsung.com>
 *	    Vikas Sajjan <vikas.sajjan@samsung.com>
 */
#include <compat.h>
#include <debug.h>
#include <usb_glue.h>

#include <xhci/usb.h>
#include <devices/usbhardware.h>
#include <xhci/xhci.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-events.h>
#include <xhci/xhci-debug.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-event] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci-event] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

typedef void (*ep_state_handler)(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event);

static void ep_handle_default(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event);
static void ep_handle_receiving_control(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event);
static void ep_handle_receiving_control_short(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event);
static void ep_handle_receiving_bulk(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event);
static void ep_handle_aborting(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event);

static const ep_state_handler ep_state_dispatch[] = {
    [USB_DEV_EP_STATE_IDLE] = NULL, /* use default handler */
    [USB_DEV_EP_STATE_RECEIVING_CONTROL] = ep_handle_receiving_control,
    [USB_DEV_EP_STATE_RECEIVING_CONTROL_SHORT] = ep_handle_receiving_control_short,
    [USB_DEV_EP_STATE_RECEIVING_BULK] = ep_handle_receiving_bulk,
    [USB_DEV_EP_STATE_RECEIVING_INT] = ep_handle_receiving_bulk,
    [USB_DEV_EP_STATE_RECEIVING_ISOC] = NULL,
    [USB_DEV_EP_STATE_ABORTING] = ep_handle_aborting,
    [USB_DEV_EP_STATE_RESETTING] = NULL,
    [USB_DEV_EP_STATE_FAILED] = NULL,
};

/*
 * So generally, XHCI has got separate endpoint context per direction
 * except for control endpoints.
 * But Poseidon assumes there is only one transfer ongoing per endpoint number,
 * regardless of direction (I think).
 * So we sort of treat both contexts as one.. at least for now.
 * Perhaps they should be separate and just treated as one for scheduling new TDs.
 * endpoint - endpoint number (0-15)
 * ep_index - endpoint context index (0-30)
 * DCI - context index (1-31)
 * endpoint = DCI >> 1
 * DCI = ep_index + 1
 */

void xhci_ep_set_failed(struct usb_device *udev, int endpoint)
{
    if (endpoint < 0 || endpoint >= USB_MAXENDPOINTS)
    {
        Kprintf("Invalid endpoint %ld\n", (LONG)endpoint);
        return;
    }
    struct ep_context *ep_ctx = &udev->ep_context[endpoint];
    Kprintf("EP %ld state %ld -> FAILED\n", (LONG)endpoint, (LONG)ep_ctx->state);
    ep_ctx->state = USB_DEV_EP_STATE_FAILED;

    if (ep_ctx->current_req)
        Kprintf("WARNING: current_req not NULL (%lx)\n", (LONG)ep_ctx->current_req);

    ep_ctx->trb_addr = 0;
    ep_ctx->trb_length = 0;
    ep_ctx->trb_available_length = 0;
    ep_ctx->timeout_stamp = 0;
    ep_ctx->timeout_active = FALSE;
    ep_ctx->current_req = NULL;
}

void xhci_ep_set_idle(struct usb_device *udev, int endpoint)
{
    if (endpoint < 0 || endpoint >= USB_MAXENDPOINTS)
    {
        Kprintf("Invalid endpoint %ld\n", (LONG)endpoint);
        return;
    }
    struct ep_context *ep_ctx = &udev->ep_context[endpoint];
    KprintfH("EP %ld state %ld -> IDLE\n", (LONG)endpoint, (LONG)ep_ctx->state);
    ep_ctx->state = USB_DEV_EP_STATE_IDLE;
    ep_ctx->trb_addr = 0;
    ep_ctx->trb_length = 0;
    ep_ctx->trb_available_length = 0;
    ep_ctx->timeout_stamp = 0;
    ep_ctx->timeout_active = FALSE;
    if (ep_ctx->current_req)
        Kprintf("WARNING: current_req not NULL (%lx)\n", (LONG)ep_ctx->current_req);
    ep_ctx->current_req = NULL;
    xhci_ep_schedule_next(udev, endpoint);
}

void xhci_ep_set_receiving(struct usb_device *udev, struct IOUsbHWReq *req, enum ep_state state, dma_addr_t trb_addr, ULONG timeout_ms)
{
    int endpoint = req->iouh_Endpoint & 0xf;
    if (endpoint < 0 || endpoint >= USB_MAXENDPOINTS)
    {
        Kprintf("Invalid endpoint %ld\n", (LONG)endpoint);
        return;
    }

    struct ep_context *ep_ctx = &udev->ep_context[endpoint];

    if (ep_ctx->state != USB_DEV_EP_STATE_IDLE)
    {
        Kprintf("EP %ld not IDLE (state %ld)\n", (LONG)endpoint, (LONG)ep_ctx->state);
        xhci_ep_set_failed(udev, endpoint);
        return;
    }

    ep_ctx->state = state;
    ep_ctx->current_req = req;
    ep_ctx->trb_addr = trb_addr;
    ep_ctx->trb_length = req->iouh_Length;
    ep_ctx->trb_available_length = req->iouh_Length;
    if (timeout_ms == 0)
    {
        ep_ctx->timeout_stamp = 0;
        ep_ctx->timeout_active = FALSE;
    }
    else
    {
        ep_ctx->timeout_stamp = get_time() + timeout_ms * 1000;
        ep_ctx->timeout_active = TRUE;
    }
}

void xhci_ep_set_resetting(struct usb_device *udev, int endpoint)
{
    if (endpoint < 0 || endpoint >= USB_MAXENDPOINTS)
    {
        Kprintf("Invalid endpoint %ld\n", (LONG)endpoint);
        return;
    }

    struct ep_context *ep_ctx = &udev->ep_context[endpoint];

    ep_ctx->state = USB_DEV_EP_STATE_RESETTING;
    ep_ctx->timeout_stamp = get_time() + XHCI_TIMEOUT * 1000;
    ep_ctx->timeout_active = TRUE;
}

void xhci_ep_set_aborting(struct usb_device *udev, int endpoint)
{
    if (endpoint < 0 || endpoint >= USB_MAXENDPOINTS)
    {
        Kprintf("Invalid endpoint %ld\n", (LONG)endpoint);
        return;
    }

    struct ep_context *ep_ctx = &udev->ep_context[endpoint];

    if (ep_ctx->state != USB_DEV_EP_STATE_RECEIVING_CONTROL &&
        ep_ctx->state != USB_DEV_EP_STATE_RECEIVING_CONTROL_SHORT &&
        ep_ctx->state != USB_DEV_EP_STATE_RECEIVING_BULK &&
        ep_ctx->state != USB_DEV_EP_STATE_RECEIVING_INT &&
        ep_ctx->state != USB_DEV_EP_STATE_RECEIVING_ISOC)
    {
        Kprintf("EP %ld not RECEIVING (state %ld)\n", (LONG)endpoint, (LONG)ep_ctx->state);
        xhci_ep_set_failed(udev, endpoint);
        return;
    }

    if (ep_ctx->current_req == NULL)
    {
        Kprintf("EP %ld current_req is NULL\n", (LONG)endpoint);
        xhci_ep_set_failed(udev, endpoint);
        return;
    }

    ep_ctx->state = USB_DEV_EP_STATE_ABORTING;
    ep_ctx->timeout_stamp = get_time() + XHCI_TIMEOUT * 1000;
    ep_ctx->timeout_active = TRUE;
}

/**
 * Finalizes a handled event TRB by advancing our dequeue pointer and giving
 * the TRB back to the hardware for recycling. Must call this exactly once at
 * the end of each event handler, and not touch the TRB again afterwards.
 *
 * @param ctrl	Host controller data structure
 * Return: none
 */
void xhci_acknowledge_event(struct xhci_ctrl *ctrl)
{
    dma_addr_t deq;

    /* Advance our dequeue pointer to the next event */
    inc_deq(ctrl, ctrl->event_ring);

    /* Inform the hardware */
    deq = xhci_trb_virt_to_dma(ctrl->event_ring->deq_seg, ctrl->event_ring->dequeue);
    xhci_writeq(&ctrl->ir_set->erst_dequeue, deq | ERST_EHB);
}

/* Endpoint event dispatcher
 * Called when a Transfer Event TRB is received
 */
static void dispatch_ep_event(struct usb_device *udev, union xhci_trb *event)
{
    UWORD endpoint = TRB_TO_ENDPOINT(LE32(event->trans_event.flags));
    if (endpoint >= USB_MAXENDPOINTS)
    {
        Kprintf("Invalid endpoint %ld\n", endpoint);
        return;
    }

    struct ep_context *ep_ctx = &udev->ep_context[endpoint];

    if (ep_state_dispatch[ep_ctx->state])
    {
        KprintfH("addr %ld EP %ld state %ld -> handling event\n", udev->devnum, (LONG)endpoint, (LONG)ep_ctx->state);
        ep_state_handler handler = ep_state_dispatch[ep_ctx->state];
        handler(udev, ep_ctx, event);
    }
    else
    {
        ep_handle_default(udev, ep_ctx, event);
    }
}

/**
 * Checks if there is a new event to handle on the event ring.
 *
 * @param ctrl	Host controller data structure
 * Return: 0 if failure else 1 on success
 */
static BOOL event_ready(struct xhci_ctrl *ctrl)
{
    xhci_inval_cache((uintptr_t)ctrl->event_ring->dequeue,
                     sizeof(union xhci_trb));

    union xhci_trb *event = ctrl->event_ring->dequeue;

    /* Does the HC or OS own the TRB? */
    if ((LE32(event->event_cmd.flags) & TRB_CYCLE) != ctrl->event_ring->cycle_state)
        return FALSE;

    return TRUE;
}

static union xhci_trb *get_event_trb(struct xhci_ctrl *ctrl)
{
    union xhci_trb *event = ctrl->event_ring->dequeue;
    if (!event_ready(ctrl))
        return NULL;
    return event;
}

BOOL xhci_process_event_trb(struct xhci_ctrl *ctrl)
{
    BOOL activity = FALSE;
    union xhci_trb *event;
    while ((event = get_event_trb(ctrl)))
    {
        activity = TRUE;
        trb_type type = TRB_FIELD_TO_TYPE(LE32(event->event_cmd.flags));

        switch (type)
        {
        case TRB_TRANSFER:
        {
            int slot = TRB_TO_SLOT_ID(LE32(event->trans_event.flags));
            KprintfH("Transfer Event TRB detected: slot %ld ep %ld (%08lx %08lx %08lx %08lx)\n",
                     (LONG)slot,
                     (LONG)TRB_TO_ENDPOINT(LE32(event->trans_event.flags)),
                     (LONG)LE32(event->generic.field[0]),
                     (LONG)LE32(event->generic.field[1]),
                     (LONG)LE32(event->generic.field[2]),
                     (LONG)LE32(event->generic.field[3]));

            struct usb_device *udev = ctrl->devs[slot]->udev;
            if (!udev)
            {
                Kprintf("No usb_device for slot %ld\n", slot);
                break;
            }
            KprintfH("USB device addr %ld on slot %ld\n", (LONG)udev->devnum, (LONG)slot);

            dispatch_ep_event(udev, event);
        }
        break;

        case TRB_COMPLETION:
            xhci_dispatch_command_event(ctrl, event);
            break;

        case TRB_PORT_STATUS:
            KprintfH("Port Status Change Event TRB: (%08lx %08lx %08lx %08lx)\n",
                     (ULONG)LE32(event->generic.field[0]),
                     (ULONG)LE32(event->generic.field[1]),
                     (ULONG)LE32(event->generic.field[2]),
                     (ULONG)LE32(event->generic.field[3]));
            xhci_acknowledge_event(ctrl);
            xhci_roothub_maybe_complete(ctrl);
            break;
        default:
            Kprintf("Unexpected XHCI event type %ld, skipping... (%08lx %08lx %08lx %08lx)\n",
                    (ULONG)type,
                    (ULONG)LE32(event->generic.field[0]),
                    (ULONG)LE32(event->generic.field[1]),
                    (ULONG)LE32(event->generic.field[2]),
                    (ULONG)LE32(event->generic.field[3]));
            xhci_acknowledge_event(ctrl);
            break;
        }
    }
    return activity;
}

void xhci_process_event_timeouts(struct xhci_ctrl *ctrl)
{
    ULONG now = get_time();
    for (int i = 0; i < MAX_HC_SLOTS; i++)
    {
        struct xhci_virt_device *virt_dev = ctrl->devs[i];
        if (!virt_dev)
            continue;
        struct usb_device *udev = virt_dev->udev;
        if (!udev)
            continue;

        for (int ep = 0; ep < USB_MAXENDPOINTS; ep++)
        {
            struct ep_context *ep_ctx = &udev->ep_context[ep];
            if (ep_ctx->state == USB_DEV_EP_STATE_IDLE)
                continue;
            if (ep_ctx->state == USB_DEV_EP_STATE_FAILED)
            {
                // xhci_reset_ep(udev, ep);
                continue;
            }
            if (!ep_ctx->timeout_active)
                continue;
            if (ep_ctx->timeout_stamp > now)
                continue;

            Kprintf("XHCI event timeout on slot %ld ep %ld state %ld\n", (LONG)udev->slot_id, (LONG)ep, (LONG)ep_ctx->state);
            xhci_abort_td(udev, ep);
        }
    }
}

static void record_transfer_result(union xhci_trb *event, int length,
                                   ULONG *status, int *act_len)
{
    int comp = GET_COMP_CODE(LE32(event->trans_event.transfer_len));
    *act_len = min(length, length - (int)EVENT_TRB_LEN(LE32(event->trans_event.transfer_len)));

    KprintfH("comp=%ld length=%ld act_len=%ld transfer_len=%ld\n",
             (LONG)comp, (LONG)length, (LONG)*act_len,
             (LONG)EVENT_TRB_LEN(LE32(event->trans_event.transfer_len)));

    switch (comp)
    {
    case COMP_SUCCESS:
        if (*act_len != length)
        {
            Kprintf("Short transfer: expected %ld, got %ld\n",
                    length, *act_len);
        }
        /* fallthrough */
    case COMP_SHORT_TX:
        *status = UHIOERR_NO_ERROR;
        break;
    case COMP_STALL:
        *status = UHIOERR_STALL;
        break;
    case COMP_DB_ERR:
    case COMP_TRB_ERR:
        *status = UHIOERR_HOSTERROR;
        break;
    case COMP_BABBLE:
        *status = UHIOERR_OVERFLOW;
        break;
    default:
        *status = UHIOERR_HOSTERROR;
    }

    KprintfH("mapped status=%ld (0=OK)\n", *status);
}

/*
 * EP handlers
 */

/* Default endpoint event handler
 * Called when no specific handler is registered for the current endpoint state
 */
static void ep_handle_default(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{
    Kprintf("No handler for endpoint state %ld\n", ep_ctx->state);
    Kprintf("Event TRB: (%08lx %08lx %08lx %08lx)\n",
            (ULONG)LE32(event->generic.field[0]),
            (ULONG)LE32(event->generic.field[1]),
            (ULONG)LE32(event->generic.field[2]),
            (ULONG)LE32(event->generic.field[3]));

    xhci_acknowledge_event(udev->controller);
    xhci_ep_set_failed(udev, TRB_TO_ENDPOINT(LE32(event->trans_event.flags)));
}

static void ep_handle_receiving_control(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{
    struct xhci_ctrl *ctrl = udev->controller;

    u32 flags = LE32(event->trans_event.flags);
    int endpoint = TRB_TO_ENDPOINT(flags);
    u64 trb_addr = LE64(event->trans_event.buffer);
    int length = ep_ctx->trb_length;

    KprintfH("event flags=%08lx xfer_len=%08lx buf=%08lx%08lx\n",
             (ULONG)flags,
             (ULONG)LE32(event->trans_event.transfer_len),
             (ULONG)upper_32_bits(trb_addr),
             (ULONG)lower_32_bits(trb_addr));

    ULONG status;
    int act_len;
    record_transfer_result(event, length, &status, &act_len);
    KprintfH("result status=%ld act_len=%ld comp=%ld\n",
             (LONG)status, (LONG)act_len,
             (LONG)GET_COMP_CODE(LE32(event->trans_event.transfer_len)));

    xhci_acknowledge_event(ctrl);

    /* Invalidate buffer to make it available to usb-core */
    if (length > 0)
    {
        // TODO is this required
        xhci_inval_cache((uintptr_t)trb_addr, length); // TODO dma_map should not realocate buffer... then revert to buffer
        xhci_dma_unmap(ctrl, (dma_addr_t)ep_ctx->current_req->iouh_Data, length);
    }

    if (status == UHIOERR_STALL)
    {
        // xhci_reset_ep(udev, endpoint);
        io_reply_failed(ep_ctx->current_req, UHIOERR_STALL);
        return;
    }

    io_reply_data(udev, ep_ctx->current_req, status, act_len);
    ep_ctx->current_req = NULL;
    if (GET_COMP_CODE(LE32(event->trans_event.transfer_len)) == COMP_SHORT_TX)
    {
        ep_ctx->state = USB_DEV_EP_STATE_RECEIVING_CONTROL_SHORT;
    }
    else
    {
        xhci_ep_set_idle(udev, endpoint);
    }
}

void ep_handle_receiving_control_short(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{
    (void)ep_ctx;

    /* Short data stage, clear up additional status stage event */
    KprintfH("short-tx status event flags=%08lx status=%08lx\n",
             (ULONG)LE32(event->generic.field[3]),
             (ULONG)LE32(event->generic.field[2]));

    // no need to confirm slot and ep as this is done by event handler
    xhci_acknowledge_event(udev->controller);
    xhci_ep_set_idle(udev, TRB_TO_ENDPOINT(LE32(event->generic.field[3])));
}

void ep_handle_receiving_bulk(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{
    // TODO make it common with ep_handle_receiving_control
    struct xhci_ctrl *ctrl = udev->controller;

    u32 flags = LE32(event->trans_event.flags);
    int endpoint = TRB_TO_ENDPOINT(flags);
    u64 trb_addr = LE64(event->trans_event.buffer);
    int length = ep_ctx->trb_length;

#ifdef DEBUG_HIGH
    struct xhci_container_ctx *out_ctx = ctrl->devs[udev->slot_id]->out_ctx;
    xhci_dump_slot_ctx("xhci_bulk_tx: ", xhci_get_slot_ctx(ctrl, out_ctx));
    xhci_dump_ep_ctx("xhci_bulk_tx: ", endpoint, xhci_get_ep_ctx(ctrl, out_ctx, TRB_TO_EP_INDEX(flags)));
#endif

    if (trb_addr != ep_ctx->trb_addr)
    {
        Kprintf("addr %ld slot %ld EP %ld\n",
                (LONG)udev->devnum, (LONG)udev->slot_id, (LONG)endpoint);
        Kprintf("Unexpected buffer, expected %08lx%08lx got %08lx%08lx\n",
                (ULONG)upper_32_bits(ep_ctx->trb_addr),
                (ULONG)lower_32_bits(ep_ctx->trb_addr),
                (ULONG)upper_32_bits(trb_addr),
                (ULONG)lower_32_bits(trb_addr));
        ep_ctx->trb_addr = trb_addr;
    }

    KprintfH("event flags=%08lx xfer_len=%08lx buf=%08lx%08lx\n",
            (ULONG)flags,
            (ULONG)LE32(event->trans_event.transfer_len),
            (ULONG)upper_32_bits(trb_addr),
            (ULONG)lower_32_bits(trb_addr));

    ULONG status;
    int act_len;
    record_transfer_result(event, length, &status, &act_len);
    KprintfH("result status=%ld act_len=%ld comp=%ld\n",
            (LONG)status, (LONG)act_len,
            (LONG)GET_COMP_CODE(LE32(event->trans_event.transfer_len)));

    xhci_acknowledge_event(ctrl);

    xhci_dma_unmap(ctrl, (dma_addr_t)ep_ctx->current_req->iouh_Data, length);
    if (act_len > 0 && ep_ctx->current_req->iouh_Dir == UHDIR_IN)
        xhci_inval_cache((uintptr_t)ep_ctx->current_req->iouh_Data, (ULONG)act_len);

    if (status == UHIOERR_STALL)
    {
        // xhci_reset_ep(udev, endpoint);
        io_reply_failed(ep_ctx->current_req, UHIOERR_STALL);
        return;
    }

    io_reply_data(udev, ep_ctx->current_req, status, act_len);
    ep_ctx->current_req = NULL;
    xhci_ep_set_idle(udev, endpoint);
}

void ep_handle_aborting(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{
    (void)ep_ctx;

    struct xhci_ctrl *ctrl = udev->controller;

    u32 flags = LE32(event->trans_event.flags);
    if (TRB_TO_SLOT_ID(flags) != udev->slot_id)
    {
        Kprintf("Expected a TRB for slot %ld, got %ld\n", udev->slot_id, TRB_TO_SLOT_ID(flags));
        return;
    }
    if (GET_COMP_CODE(LE32(event->trans_event.transfer_len)) != COMP_STOP)
    {
        Kprintf("Expected a TRB with STOP, got %ld with %ld\n", GET_COMP_CODE(LE32(event->trans_event.transfer_len)));
    }

    io_reply_failed(ep_ctx->current_req, IOERR_ABORTED);
    ep_ctx->current_req = NULL;

    xhci_acknowledge_event(ctrl);
    /* no state change - that is done by handle_abort_stop_ring */
}
