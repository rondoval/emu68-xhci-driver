// SPDX-License-Identifier: GPL-2.0-only
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
#ifdef __INTELLISENSE__
#else
#endif

#include <debug.h>

#include <bits.h>
#include <byteorder.h>
#include <iomem.h>
#include <memory.h>

#include <xhci/xhci.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-events.h>
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-descriptors.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci-ring.h>
#include <xhci/xhci-context.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-event] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef TRACE
#undef KprintfT
#define KprintfT(fmt, ...) PrintPistorm("[xhci-event] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

typedef void (*ep_state_handler)(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event);

/* Default handler only logs an unexpected-state warning; compiled out (calls
 * included) without DEBUG. */
#ifdef DEBUG
static void ep_handle_default(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event);
#else
#define ep_handle_default(udev, ep_ctx, event) ((void)0)
#endif
static void ep_handle_receiving_generic(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event);
static void ep_handle_rt_iso(struct ep_context *ep_ctx, const struct xhci_td_completion *done, xhci_comp_code comp);
static void ep_handle_aborting(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event);
static void ep_handle_suspended(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event);

static const ep_state_handler ep_state_dispatch[] = {
    [USB_DEV_EP_STATE_IDLE] = NULL, /* use default handler */
    [USB_DEV_EP_STATE_RECEIVING] = ep_handle_receiving_generic,
    [USB_DEV_EP_STATE_ABORTING] = ep_handle_aborting,
    [USB_DEV_EP_STATE_RESETTING] = NULL,
    [USB_DEV_EP_STATE_FAILED] = NULL,
    [USB_DEV_EP_STATE_SUSPENDED] = ep_handle_suspended,
    [USB_DEV_EP_STATE_RT_ISO_STOPPED] = NULL,
    [USB_DEV_EP_STATE_RT_ISO_RUNNING] = ep_handle_receiving_generic,
    [USB_DEV_EP_STATE_RT_ISO_STOPPING] = ep_handle_receiving_generic};

/* Endpoint event dispatcher
 * Called when a Transfer Event TRB is received
 */
static void dispatch_ep_event(struct usb_device *udev, union xhci_trb *event)
{
    const u32 flags = le32(event->trans_event.flags);
    const u8 ep_index = TRB_TO_EP_INDEX(flags);

    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
    if (!ep_ctx)
    {
        KprintfT("No ep context for slot %lu ep %lu\n", (ULONG)udev->slot_id, (ULONG)ep_index);
        return;
    }
    enum ep_state state = xhci_ep_get_state(ep_ctx);

    if (ep_state_dispatch[state])
    {
        KprintfT("slot %lu EP %lu state %lu -> handling event\n", (ULONG)udev->slot_id, (ULONG)ep_index, (ULONG)state);
        ep_state_handler handler = ep_state_dispatch[state];
        handler(udev, ep_ctx, event);
    }
    else
    {
        ep_handle_default(udev, ep_ctx, event);
    }
}

#ifdef TRACE
static void debug_port_status_event(struct xhci_ctrl *ctrl, union xhci_trb *event)
{
    const u32 port_field = le32(event->generic.field[0]);
    const u32 field1 = le32(event->generic.field[1]);
    const u32 field2 = le32(event->generic.field[2]);
    const u32 flags = le32(event->generic.field[3]);
    const u32 port_id = GET_PORT_ID(port_field);
    const u32 usbsts = mmio_read32(&ctrl->hcor->or_usbsts);
    u32 portsc = 0;

    if (port_id > 0 && port_id <= MAX_HC_PORTS)
        portsc = mmio_read32(&ctrl->hcor->portregs[port_id - 1].or_portsc);

    Kprintf("Port Status Change Event port=%lu portsc=%08lx usbsts=%08lx trb=(%08lx %08lx %08lx %08lx)\n",
            port_id,
            portsc,
            usbsts,
            port_field,
            field1,
            field2,
            flags);
}
#else
#define debug_port_status_event(ctrl, event) ((void)0)
#endif

BOOL xhci_process_event_trb(struct xhci_ctrl *ctrl)
{
    BOOL activity = FALSE;
    union xhci_trb *event;
    while ((event = xhci_ring_get_event_trb(ctrl->event_ring)))
    {
        activity = TRUE;
        trb_type type = TRB_FIELD_TO_TYPE(le32(event->event_cmd.flags));

        switch (type)
        {
        case TRB_TRANSFER:
        {
            const u32 flags = le32(event->trans_event.flags);
            const u32 slot = TRB_TO_SLOT_ID(flags);
            KprintfT("Transfer Event TRB detected: slot %lu ep %lu (%08lx %08lx %08lx %08lx)\n",
                     (ULONG)slot,
                     (ULONG)TRB_TO_EP_INDEX(flags),
                     (ULONG)le32(event->generic.field[0]),
                     (ULONG)le32(event->generic.field[1]),
                     (ULONG)le32(event->generic.field[2]),
                     (ULONG)le32(event->generic.field[3]));

            struct usb_device *udev = ctrl->devices_by_slot_id[slot];
            if (!udev)
            {
                Kprintf("No usb_device for slot %lu\n", (ULONG)slot);
                break;
            }
            dispatch_ep_event(udev, event);
        }
        break;

        case TRB_COMPLETION:
            xhci_dispatch_command_event(ctrl, event);
            break;

        case TRB_PORT_STATUS:
            debug_port_status_event(ctrl, event);
            xhci_roothub_complete_int_request(ctrl->root_hub);
            break;
        default:
            Kprintf("Unexpected XHCI event type %lu, skipping... (%08lx %08lx %08lx %08lx)\n",
                    (ULONG)type,
                    (ULONG)le32(event->generic.field[0]),
                    (ULONG)le32(event->generic.field[1]),
                    (ULONG)le32(event->generic.field[2]),
                    (ULONG)le32(event->generic.field[3]));
            break;
        }

        /* Retire only after the handler finishes reading this TRB. */
        xhci_ring_consume_event(ctrl);
    }

    if (activity)
        xhci_ring_ack_events(ctrl);

    return activity;
}

void xhci_process_event_timeouts(struct xhci_ctrl *ctrl)
{
    /* The slot map holds every ring-bearing device.  The root hub has no
     * slot and no ring TDs to expire. */
    for (u32 i = 1; i < MAX_HC_SLOTS; i++)
    {
        struct usb_device *udev = ctrl->devices_by_slot_id[i];
        if (!udev)
            continue;

        for (u8 ep_index = 0; ep_index < USB_MAX_ENDPOINT_CONTEXTS; ep_index++)
        {
            struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
            if (ep_ctx && xhci_ep_is_expired(ep_ctx))
            {
                KprintfT("XHCI TD timeout on slot %lu ep %lu\n", (ULONG)udev->slot_id, (ULONG)ep_index);
                xhci_ep_request_timeout_recovery(ep_ctx);
            }
        }
    }
}

inline static s8 translate_status(xhci_comp_code comp)
{
    s8 status;

    switch (comp)
    {
    case COMP_SUCCESS:
    case COMP_SHORT_TX:
    case COMP_UNDERRUN:
    case COMP_OVERRUN:
    case COMP_MISSED_INT:
        status = UHIOERR_NO_ERROR;
        break;
    case COMP_STALL:
        KprintfT("Device stalled\n");
        status = UHIOERR_STALL;
        break;
    case COMP_TX_ERR:
        /* CRC/bit-stuffing/no-response per xHCI: report as a transaction
         * error, not TIMEOUT - Poseidon's dead-count treats TIMEOUT three
         * times worse than a CRC error. */
        Kprintf("USB transaction error\n");
        status = UHIOERR_CRCERROR;
        break;
    case COMP_DB_ERR:
    case COMP_TRB_ERR:
        // Data Buffer Error or TRB Error
        Kprintf("TRB error\n");
        status = UHIOERR_HOSTERROR;
        break;
    case COMP_BABBLE:
        KprintfT("Babble detected\n");
        status = UHIOERR_BABBLE;
        break;
    case COMP_BUFF_OVER:
        KprintfT("Isoc buffer overrun\n");
        status = UHIOERR_OVERFLOW;
        break;
    case COMP_BW_OVER:
        KprintfT("Bandwidth overrun\n");
        status = UHIOERR_HOSTERROR;
        break;
    case COMP_SPLIT_ERR:
        KprintfT("Split transaction error\n");
        status = UHIOERR_TIMEOUT;
        break;
    default:
        Kprintf("Unhandled completion code %lu\n", (ULONG)comp);
        status = UHIOERR_HOSTERROR;
    }

    return status;
}

/*
 * EP handlers
 */

/* Default endpoint event handler
 * Called when no specific handler is registered for the current endpoint state
 */
#ifdef DEBUG
static void ep_handle_default(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{
    (void)event;
    enum ep_state state = xhci_ep_get_state(ep_ctx);
    const u8 ep_index = xhci_ep_get_ep_index(ep_ctx);

    Kprintf("No handler for slot %lu endpoint %lu state %lu\n", (ULONG)udev->slot_id, (ULONG)ep_index, (ULONG)state);
    KprintfT("Event TRB: (%08lx %08lx %08lx %08lx)\n",
             (ULONG)le32(event->generic.field[0]),
             (ULONG)le32(event->generic.field[1]),
             (ULONG)le32(event->generic.field[2]),
             (ULONG)le32(event->generic.field[3]));
}
#endif /* DEBUG (ep_handle_default) */

static void ep_handle_receiving_generic(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{

    const u32 flags = le32(event->trans_event.flags);
    const u8 ep_index = TRB_TO_EP_INDEX(flags);
    const u64 trb_addr = le64(event->trans_event.buffer);
    const u32 transfer_len = le32(event->trans_event.transfer_len);
    const xhci_comp_code comp = GET_COMP_CODE(transfer_len);

#ifdef DEBUG_CONTEXT
    KprintfT("event flags=%08lx xfer_len=%08lx buf=%08lx%08lx\n",
             (ULONG)flags,
             (ULONG)transfer_len,
             (ULONG)u64_hi32(trb_addr),
             (ULONG)u64_lo32(trb_addr));
    xhci_dump_slot_ctx("[xhci-event] ep_handle_receiving_generic:", udev, FALSE);
    xhci_dump_ep_ctx("[xhci-event] ep_handle_receiving_generic:", udev, ep_index);
#endif

    /* RT ISO ring status events are controller/schedule feedback, not TD completion. */
    if ((comp == COMP_UNDERRUN || comp == COMP_OVERRUN) &&
        xhci_ep_get_state(ep_ctx) == USB_DEV_EP_STATE_RT_ISO_RUNNING)
    {
        Kprintf("RT ISO ring %s addr=%lu ep=%lu flags=0x%08lx xfer=0x%08lx trb=%08lx%08lx\n",
                (comp == COMP_UNDERRUN) ? "underrun" : "overrun",
                (ULONG)udev->slot_id,
                (ULONG)ep_index,
                (ULONG)flags,
                (ULONG)transfer_len,
                (ULONG)u64_hi32(trb_addr),
                (ULONG)u64_lo32(trb_addr));

        xhci_ep_schedule_rt_iso(ep_ctx);

        return;
    }

    /* A short packet on a non-final TRB only records the exact transferred
     * length; the controller follows up with the final-TRB event, which
     * completes the TD (and, for control TDs, the status stage).  All other
     * events consume the TD with an exact act_len. */
    struct xhci_td_completion done;
    BOOL deferred;
    if (!xhci_ep_complete_by_trb(ep_ctx, (dma_addr_t)trb_addr,
                                 EVENT_TRB_LEN(transfer_len),
                                 comp == COMP_SHORT_TX,
                                 &done, &deferred))
    {
        if (!deferred)
            Kprintf("No TD found for TRB %08lx%08lx  %08lx %08lx on EP %lu\n",
                    (ULONG)u64_hi32(trb_addr), (ULONG)u64_lo32(trb_addr), (ULONG)flags, (ULONG)transfer_len, (ULONG)ep_index);
        return;
    }

    if (done.rt)
    {
        if (comp == COMP_MISSED_INT)
            Kprintf("RT ISO missed-service addr=%lu ep=%lu frame=%lu len=%lu\n",
                    (ULONG)udev->slot_id,
                    (ULONG)ep_index,
                    (ULONG)done.rt_frame,
                    (ULONG)done.rt_length);

        ep_handle_rt_iso(ep_ctx, &done, comp);
        return;
    }

    struct xhci_xfer *req = done.req;
    u32 act_len = done.act_len;

    s8 status = translate_status(comp);
    KprintfT("result status=%ld act_len=%lu comp=%lu\n", (LONG)status, (ULONG)act_len, (ULONG)comp);

    /* Flag short IN transfers as runts unless explicitly allowed or expected (control). */
    if (status == UHIOERR_NO_ERROR && act_len < req->data_length &&
        req->direction == XHCI_DIR_IN && req->type != UHCD_EPTYPE_CONTROL &&
        (req->flags & XHCI_XF_ALLOWRUNT) == 0)
    {
        status = UHIOERR_RUNTPACKET;
    }

    BOOL halted = (comp == COMP_STALL || comp == COMP_BABBLE || comp == COMP_SPLIT_ERR || comp == COMP_TX_ERR);
    if (halted)
    {
        xhci_xfer_complete(udev, req, status, act_len);
        xhci_reset_ep(udev, ep_index);
        return;
    }

    xhci_xfer_complete(udev, req, status, act_len);
    xhci_ep_set_idle(ep_ctx);
}

static void ep_handle_rt_iso(struct ep_context *ep_ctx, const struct xhci_td_completion *done, xhci_comp_code comp)
{
    /* the *_done hooks' buffer request reports the wire status (§10.3) */
    const u16 ubr_flags = (comp == COMP_SUCCESS || comp == COMP_SHORT_TX ||
                           comp == COMP_STOP || comp == COMP_STOP_INVAL || comp == COMP_STOP_SHORT)
                              ? 0
                              : UHCD_UBF_XFER_ERROR;

    if (done->rt_dir == XHCI_DIR_IN)
    {
        if (done->act_len > 0)
            xhci_ep_rt_iso_in(ep_ctx, done->rt_buffer, done->rt_length, done->act_len, done->rt_frame, ubr_flags);

        xhci_ep_free_rt_iso_buffer(ep_ctx, done->rt_buffer);
    }
    else
        xhci_ep_rt_iso_out(ep_ctx, done->rt_buffer, done->rt_length, done->act_len, done->rt_frame, ubr_flags);

    xhci_ep_schedule_rt_iso(ep_ctx);
}

/* Stop Endpoint completions ahead of a port suspend (U3): the TD stays queued
 * on the ring for the resume - drop the event quietly. */
static void ep_handle_suspended(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{
#ifndef DEBUG
    /* only referenced by debug logging / the default handler */
    (void)udev;
    (void)ep_ctx;
#endif
    const xhci_comp_code comp = GET_COMP_CODE(le32(event->trans_event.transfer_len));

    if (comp == COMP_STOP || comp == COMP_STOP_INVAL || comp == COMP_STOP_SHORT)
    {
        KprintfT("slot %lu EP %lu stopped for suspend (comp=%lu)\n",
                 (ULONG)udev->slot_id,
                 (ULONG)xhci_ep_get_ep_index(ep_ctx), (ULONG)comp);
        return;
    }

    ep_handle_default(udev, ep_ctx, event);
}

static void ep_handle_aborting(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{
    (void)ep_ctx;

    u32 flags = le32(event->trans_event.flags);
    if (TRB_TO_SLOT_ID(flags) != udev->slot_id)
    {
        Kprintf("Expected a TRB for slot %lu, got %lu\n", (ULONG)udev->slot_id, (ULONG)TRB_TO_SLOT_ID(flags));
        return;
    }

    const xhci_comp_code comp = GET_COMP_CODE(le32(event->trans_event.transfer_len));
    switch(comp)
    {
        case COMP_STOP:
            KprintfT("Transfer stopped successfully\n");
            break;
        case COMP_STOP_INVAL:
            KprintfT("Transfer stopped with invalid length\n");
            break;
        case COMP_STOP_SHORT:
            KprintfT("Transfer stopped after short packet\n");
            break;
        default:
            Kprintf("Expected a TRB with STOP, got %lu\n", (ULONG)comp);
    }

    /* no state change - that is done by handle_abort_stop_ring */
}
