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

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-event] %s: " fmt, __func__, ##__VA_ARGS__)
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
    [USB_DEV_EP_STATE_RECEIVING_ISOC] = NULL, // TODO ep_handle_receiving_isoc,
    [USB_DEV_EP_STATE_ABORTING] = ep_handle_aborting,
    [USB_DEV_EP_STATE_RESETTING] = NULL, // TODO ep_handle_resetting,
    [USB_DEV_EP_STATE_FAILED] = NULL,    // TODO ep_handle_failed,
};

void xhci_ep_set_failed(struct usb_device *udev, int ep_index)
{
    if (ep_index < 0 || ep_index >= USB_MAXENDPOINTS)
    {
        Kprintf("xhci_ep_set_failed: Invalid endpoint %ld\n", (LONG)ep_index);
        return;
    }
    struct ep_context *ep_ctx = &udev->ep_context[ep_index];
    Kprintf("xhci_ep_set_failed: EP %ld state %ld -> FAILED\n", (LONG)ep_index, (LONG)ep_ctx->state);
    ep_ctx->state = USB_DEV_EP_STATE_FAILED;

    if (ep_ctx->current_req)
        Kprintf("xhci_ep_set_failed: WARNING: current_req not NULL (%lx)\n", (LONG)ep_ctx->current_req);

    ep_ctx->trb_addr = 0;
    ep_ctx->trb_length = 0;
    ep_ctx->trb_available_length = 0;
    ep_ctx->timeout_stamp = 0;
    ep_ctx->current_req = NULL;
}

void xhci_ep_set_idle(struct usb_device *udev, int ep_index)
{
    if (ep_index < 0 || ep_index >= USB_MAXENDPOINTS)
    {
        Kprintf("xhci_ep_set_idle: Invalid endpoint %ld\n", (LONG)ep_index);
        return;
    }
    struct ep_context *ep_ctx = &udev->ep_context[ep_index];
    Kprintf("xhci_ep_set_idle: EP %ld state %ld -> IDLE\n", (LONG)ep_index, (LONG)ep_ctx->state);
    ep_ctx->state = USB_DEV_EP_STATE_IDLE;
    ep_ctx->trb_addr = 0;
    ep_ctx->trb_length = 0;
    ep_ctx->trb_available_length = 0;
    ep_ctx->timeout_stamp = 0;
    if (ep_ctx->current_req)
        Kprintf("xhci_ep_set_idle: WARNING: current_req not NULL (%lx)\n", (LONG)ep_ctx->current_req);
    ep_ctx->current_req = NULL;
}

void xhci_ep_set_receiving(struct usb_device *udev, int ep_index, enum ep_state state, dma_addr_t trb_addr, ULONG trb_length, ULONG timeout_ms)
{
    if (ep_index < 0 || ep_index >= USB_MAXENDPOINTS)
    {
        Kprintf("xhci_ep_set_receiving: Invalid endpoint %ld\n", (LONG)ep_index);
        return;
    }

    struct ep_context *ep_ctx = &udev->ep_context[ep_index];

    if (ep_ctx->state != USB_DEV_EP_STATE_IDLE)
    {
        Kprintf("xhci_ep_set_receiving: EP %ld not IDLE (state %ld)\n", (LONG)ep_index, (LONG)ep_ctx->state);
        xhci_ep_set_failed(udev, ep_index);
        return;
    }

    if (ep_ctx->current_req == NULL)
    {
        Kprintf("xhci_ep_set_receiving: EP %ld current_req is NULL\n", (LONG)ep_index);
        xhci_ep_set_failed(udev, ep_index);
        return;
    }

    ep_ctx->state = state;
    ep_ctx->trb_addr = trb_addr;
    ep_ctx->trb_length = trb_length;
    ep_ctx->trb_available_length = trb_length;
    ep_ctx->timeout_stamp = get_time() + timeout_ms * 1000;
}

void xhci_ep_set_resetting(struct usb_device *udev, int ep_index)
{
    if (ep_index < 0 || ep_index >= USB_MAXENDPOINTS)
    {
        Kprintf("xhci_ep_set_resetting: Invalid endpoint %ld\n", (LONG)ep_index);
        return;
    }

    struct ep_context *ep_ctx = &udev->ep_context[ep_index];

    ep_ctx->state = USB_DEV_EP_STATE_RESETTING;
    ep_ctx->timeout_stamp = get_time() + XHCI_TIMEOUT * 1000;
}

void xhci_ep_set_aborting(struct usb_device *udev, int ep_index)
{
    if (ep_index < 0 || ep_index >= USB_MAXENDPOINTS)
    {
        Kprintf("xhci_ep_set_aborting: Invalid endpoint %ld\n", (LONG)ep_index);
        return;
    }

    struct ep_context *ep_ctx = &udev->ep_context[ep_index];

    if (ep_ctx->state != USB_DEV_EP_STATE_RECEIVING_CONTROL &&
        ep_ctx->state != USB_DEV_EP_STATE_RECEIVING_CONTROL_SHORT &&
        ep_ctx->state != USB_DEV_EP_STATE_RECEIVING_BULK &&
        ep_ctx->state != USB_DEV_EP_STATE_RECEIVING_INT &&
        ep_ctx->state != USB_DEV_EP_STATE_RECEIVING_ISOC)
    {
        Kprintf("xhci_ep_set_aborting: EP %ld not RECEIVING (state %ld)\n", (LONG)ep_index, (LONG)ep_ctx->state);
        xhci_ep_set_failed(udev, ep_index);
        return;
    }

    if (ep_ctx->current_req == NULL)
    {
        Kprintf("xhci_ep_set_aborting: EP %ld current_req is NULL\n", (LONG)ep_index);
        xhci_ep_set_failed(udev, ep_index);
        return;
    }

    ep_ctx->state = USB_DEV_EP_STATE_ABORTING;
    ep_ctx->timeout_stamp = get_time() + XHCI_TIMEOUT * 1000;
}

struct IOUsbHWReq *xhci_ep_get_next_request(struct usb_device *udev, int ep_index)
{
    if (ep_index < 0 || ep_index >= USB_MAXENDPOINTS)
    {
        Kprintf("xhci_ep_get_next_request: Invalid endpoint %ld\n", (LONG)ep_index);
        return NULL;
    }

    struct ep_context *ep_ctx = &udev->ep_context[ep_index];
    if (ep_ctx->state != USB_DEV_EP_STATE_IDLE || ep_ctx->current_req)
    {
        Kprintf("xhci_ep_get_next_request: EP %ld not IDLE (state %ld, req %lx)\n", (LONG)ep_index, (LONG)ep_ctx->state, (LONG)ep_ctx->current_req);
        return NULL;
    }

    struct MinNode *req = RemHeadMinList(&ep_ctx->req_list);
    if (!req)
        return NULL;
    ep_ctx->current_req = (struct IOUsbHWReq *)req;
    return ep_ctx->current_req;
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
    UWORD ep = TRB_TO_EP_INDEX(LE32(event->trans_event.flags));
    if (ep >= USB_MAXENDPOINTS)
    {
        Kprintf("Invalid endpoint %ld\n", ep);
        return;
    }

    struct ep_context *ep_ctx = &udev->ep_context[ep];

    if (ep_state_dispatch[ep_ctx->state])
    {
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

            struct usb_device *udev = ctrl->devs[slot]->udev;
            if (!udev)
            {
                Kprintf("No usb_device for slot %ld\n", slot);
                break;
            }

            dispatch_ep_event(udev, event);
        }
        break;

        case TRB_COMPLETION:
            xhci_dispatch_command_event(ctrl, event);
            break;

        case TRB_PORT_STATUS:
            // TODO handle port status change events
            //  likely somehow pass to INT transfer on root hub
            Kprintf("Port Status Change Event TRB detected: (%08lx %08lx %08lx %08lx)\n",
                    (ULONG)LE32(event->generic.field[0]),
                    (ULONG)LE32(event->generic.field[1]),
                    (ULONG)LE32(event->generic.field[2]),
                    (ULONG)LE32(event->generic.field[3]));
            break;
        default:
            Kprintf("Unexpected XHCI event type %ld, skipping... (%08lx %08lx %08lx %08lx)\n",
                    (ULONG)type,
                    (ULONG)LE32(event->generic.field[0]),
                    (ULONG)LE32(event->generic.field[1]),
                    (ULONG)LE32(event->generic.field[2]),
                    (ULONG)LE32(event->generic.field[3]));
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
                xhci_reset_ep(udev, ep);
                continue;
            }
            if (ep_ctx->timeout_stamp > now)
                continue;

            Kprintf("XHCI event timeout on slot %ld ep %ld state %ld\n", (LONG)udev->slot_id, (LONG)ep, (LONG)ep_ctx->state);
            Kprintf("XHCI bulk transfer timed out, aborting...\n");
            xhci_abort_td(udev, ep);
        }
    }
}

static void record_transfer_result(union xhci_trb *event, int length,
                                   ULONG *status, int *act_len)
{
    int comp = GET_COMP_CODE(LE32(event->trans_event.transfer_len));
    *act_len = min(length, length - (int)EVENT_TRB_LEN(LE32(event->trans_event.transfer_len)));

    KprintfH("record_transfer_result: comp=%ld length=%ld act_len=%ld transfer_len=%ld\n",
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

    KprintfH("record_transfer_result: mapped status=%ld (0=OK)\n", *status);
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
    xhci_ep_set_failed(udev, TRB_TO_EP_INDEX(LE32(event->trans_event.flags)));
}

static void ep_handle_receiving_control(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{
    struct xhci_ctrl *ctrl = udev->controller;

    u32 flags = LE32(event->trans_event.flags);
    int ep_index = TRB_TO_EP_INDEX(flags);
    u64 buf_64 = event->trans_event.buffer;
    int length = udev->ep_context[ep_index].trb_length;

    KprintfH("xhci_ctrl_tx: first event flags=%08lx status=%08lx buf=%08lx%08lx\n",
             (ULONG)LE32(event->generic.field[3]),
             (ULONG)LE32(event->generic.field[2]),
             (ULONG)upper_32_bits(LE64(event->trans_event.buffer)),
             (ULONG)lower_32_bits(LE64(event->trans_event.buffer)));

    ULONG status;
    int act_len;
    record_transfer_result(event, length, &status, &act_len);
    KprintfH("xhci_ctrl_tx: result status=%ld act_len=%ld comp=%ld\n",
             (LONG)status, (LONG)act_len,
             (LONG)GET_COMP_CODE(LE32(event->trans_event.transfer_len)));

    xhci_acknowledge_event(ctrl);

    /* Invalidate buffer to make it available to usb-core */
    if (length > 0)
    {
        xhci_inval_cache((uintptr_t)buf_64, length); // TODO dma_map should not realocate buffer... then revert to buffer
        xhci_dma_unmap(ctrl, buf_64, length);
    }

    if (status == UHIOERR_STALL)
    {
        xhci_reset_ep(udev, ep_index);
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
        xhci_ep_set_idle(udev, ep_index);
    }
}

void ep_handle_receiving_control_short(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{
    (void)ep_ctx;

    /* Short data stage, clear up additional status stage event */
    Kprintf("xhci_ctrl_tx: short-tx status event flags=%08lx status=%08lx\n",
            (ULONG)LE32(event->generic.field[3]),
            (ULONG)LE32(event->generic.field[2]));

    // no need to confirm slot and ep as this is done by event handler
    xhci_acknowledge_event(udev->controller);
    xhci_ep_set_idle(udev, TRB_TO_EP_INDEX(LE32(event->generic.field[3])));
}

void ep_handle_receiving_bulk(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{
    struct xhci_ctrl *ctrl = udev->controller;

    if (LE64(event->trans_event.buffer) != ep_ctx->trb_addr)
    {
        ep_ctx->trb_available_length -= (int)EVENT_TRB_LEN(LE32(event->trans_event.transfer_len));
        Kprintf("xhci_bulk_tx: skipping intermediate event buf=%08lx%08lx new_avail=%ld\n",
                (ULONG)upper_32_bits(LE64(event->trans_event.buffer)),
                (ULONG)lower_32_bits(LE64(event->trans_event.buffer)),
                (LONG)ep_ctx->trb_available_length);
        xhci_acknowledge_event(ctrl);
        /* No state change, more events to come */
        return;
    }

    u32 flags = LE32(event->trans_event.flags);
    u64 buf_64 = LE64(event->trans_event.buffer);
    int length = ep_ctx->trb_length;

    Kprintf("xhci_bulk_tx: event flags=%08lx xfer_len=%08lx buf=%08lx%08lx\n",
            (ULONG)LE32(flags),
            (ULONG)LE32(event->trans_event.transfer_len),
            (ULONG)upper_32_bits(buf_64),
            (ULONG)lower_32_bits(buf_64));

    ULONG status;
    int act_len;
    record_transfer_result(event, ep_ctx->trb_available_length, &status, &act_len);
    xhci_acknowledge_event(ctrl);
    xhci_inval_cache((uintptr_t)buf_64, length);
    xhci_dma_unmap(ctrl, buf_64, length);
    Kprintf("xhci_bulk_tx: done status=%ld act_len=%ld\n", (LONG)status, (LONG)act_len);

    io_reply_data(udev, ep_ctx->current_req, status, act_len);
    ep_ctx->current_req = NULL;
    xhci_ep_set_idle(udev, TRB_TO_EP_INDEX(flags));
}

void ep_handle_aborting(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{
    (void)ep_ctx;
    
    struct xhci_ctrl *ctrl = udev->controller;

    u32 flags = LE32(event->trans_event.flags);
    if (TRB_TO_SLOT_ID(flags) != udev->slot_id)
    {
        Kprintf("abort_td: Expected a TRB for slot %ld, got %ld\n", udev->slot_id, TRB_TO_SLOT_ID(flags));
        return;
    }
    if (GET_COMP_CODE(LE32(event->trans_event.transfer_len)) != COMP_STOP)
    {
        Kprintf("abort_td: Expected a TRB with STOP, got %ld with %ld\n", GET_COMP_CODE(LE32(event->trans_event.transfer_len)));
    }

    io_reply_failed(ep_ctx->current_req, IOERR_ABORTED);
    ep_ctx->current_req = NULL;

    xhci_acknowledge_event(ctrl);
    /* no state change - that is done by handle_abort_stop_ring */
}
