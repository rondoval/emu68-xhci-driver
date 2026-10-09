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
#define __NOLIBBASE__
#define EXEC_BASE_NAME SysBase /* a local in every function, from its context's sysBase */

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
static void rt_iso_deliver(struct ep_context *ep_ctx, const struct xhci_td_completion *done, xhci_comp_code comp);

/* IDLE, PARKED and FAILED expect no transfer event: the default handler logs
 * one.  RECOVERING does: the "stopped" event of a Stop Endpoint, and the
 * completions of TDs that finished before the stop took effect. */
static const ep_state_handler ep_state_dispatch[] = {
    [USB_DEV_EP_STATE_IDLE] = NULL,
    [USB_DEV_EP_STATE_RECEIVING] = ep_handle_receiving_generic,
    [USB_DEV_EP_STATE_RECOVERING] = ep_handle_receiving_generic,
    [USB_DEV_EP_STATE_PARKED] = NULL,
    [USB_DEV_EP_STATE_FAILED] = NULL,
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
            if (ep_ctx)
                xhci_ep_check_timeouts(ep_ctx);
        }
    }
}

/* What happened on the wire, whatever the endpoint type: what a code means
 * for a control, bulk/interrupt or iso endpoint is the stack's to decide. */
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
        /* CRC/bit-stuffing/no-response per xHCI - not TIMEOUT, which the
         * stack reads as "device gone" */
        Kprintf("USB transaction error\n");
        status = UHIOERR_XACTERROR;
        break;
    case COMP_DB_ERR:
        /* the xHC could not keep the data buffer fed/drained - host side */
        Kprintf("Data Buffer Error (host DMA/buffer underrun)\n");
        status = UHIOERR_HOSTERROR;
        break;
    case COMP_TRB_ERR:
        /* an illegal TRB parameter for this ring or endpoint: a driver bug,
         * never something the device did */
        Kprintf("TRB Error (illegal TRB for this ring/endpoint)\n");
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
        /* COMP_TX_ERR-like failure one hop away, behind a hub's TT */
        KprintfT("Split transaction error\n");
        status = UHIOERR_SPLITERROR;
        break;
    case COMP_ISSUES:
    case COMP_STREAM_ERR:
    case COMP_STRID_ERR:
        /* Event Lost Error: the controller could not report all of the TD's
         * events (xHCI 4.10.1).  Invalid Stream Type / ID Error: the stream
         * a packet named has no valid context (xHCI 4.12.2.1).  Either way
         * the controller halted the endpoint and what the transfer did is
         * unknown - to the stack the same as a transaction error: clear the
         * halt, try again. */
        Kprintf("Endpoint halted by the controller, completion code %lu\n", (ULONG)comp);
        status = UHIOERR_XACTERROR;
        break;
    default:
        Kprintf("Unhandled completion code %lu\n", (ULONG)comp);
        status = UHIOERR_HOSTERROR;
    }

    return status;
}

/* Did the controller halt the endpoint on this event?  After these nothing
 * more runs on it until it is recovered (xHCI 4.8.3).  An isoch endpoint
 * never halts: an error fails that one TD and the ring runs on. */
static BOOL comp_halts_endpoint(xhci_comp_code comp, u8 xfer_type)
{
    switch (comp)
    {
    case COMP_STALL:
    case COMP_STREAM_ERR: /* Invalid Stream Type Error */
    case COMP_STRID_ERR:  /* Invalid Stream ID Error */
    case COMP_ISSUES:     /* Event Lost Error */
        return TRUE;
    case COMP_BABBLE:
    case COMP_TX_ERR:
    case COMP_SPLIT_ERR:
        return xfer_type != UHCD_EPTYPE_ISO;
    default:
        return FALSE;
    }
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

    /* While the endpoint is being stopped or re-armed its TDs still complete:
     * a transfer that finished before the stop took effect is answered like
     * any other.  Two things differ.  The endpoint's state is not this
     * handler's to change then.  And the "stopped" event of the Stop Endpoint
     * itself names a TD that is still queued: what becomes of that one is
     * for the recovery to decide. */
    const BOOL recovering = xhci_ep_get_state(ep_ctx) == USB_DEV_EP_STATE_RECOVERING;
    if (recovering && (comp == COMP_STOP || comp == COMP_STOP_INVAL || comp == COMP_STOP_SHORT))
        return;

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
    if ((comp == COMP_UNDERRUN || comp == COMP_OVERRUN) &&
        xhci_ep_get_state(ep_ctx) == USB_DEV_EP_STATE_RT_ISO_STOPPING)
    {
        xhci_ep_rt_iso_ring_empty(ep_ctx);
        return;
    }

    /* A transaction error may have been a passing one: the same transaction
     * is tried again a few times before the transfer is failed for it. */
    if (comp == COMP_TX_ERR && xhci_ep_soft_retry(ep_ctx))
        return;

    /* A short packet on a non-final TRB only records the exact transferred
     * length; the controller follows up with the final-TRB event, which
     * completes the TD (and, for control TDs, the status stage).  All other
     * events consume the TD with an exact act_len.
     * On an iso stream the TDs the controller passed over on the way to the
     * event's own come out first, as failed intervals, so the class sees the
     * stream in order. */
    struct xhci_td_completion done;
    BOOL deferred;
    do
    {
        if (!xhci_ep_complete_by_trb(ep_ctx, (dma_addr_t)trb_addr,
                                     EVENT_TRB_LEN(transfer_len),
                                     comp == COMP_SHORT_TX,
                                     &done, &deferred))
        {
            if (deferred)
                return;

            Kprintf("No TD found for TRB %08lx%08lx  %08lx %08lx on EP %lu\n",
                    (ULONG)u64_hi32(trb_addr), (ULONG)u64_lo32(trb_addr), (ULONG)flags, (ULONG)transfer_len, (ULONG)ep_index);

            /* A halt that no transfer of ours accounts for.  Stream protocol
             * errors are reported that way, and so is a stall or a transaction
             * error while a stream pipe is being primed: their events name no
             * TRB (xHCI 4.17.4).  With no transfer to retire and to tell the
             * stack through, the endpoint is given up: everything on it is
             * answered, and the next configuration builds it anew.  (The
             * transfer type does not matter here: the endpoint context says
             * whether it is halted.) */
            if (comp_halts_endpoint(comp, UHCD_EPTYPE_BULK) &&
                xhci_read_hw_ep_state(udev, ep_index) == EP_STATE_HALTED)
            {
                Kprintf("EP %lu halted (completion code %lu) with no transfer named\n", (ULONG)ep_index, (ULONG)comp);
                xhci_ep_set_failed(ep_ctx);
            }
            return;
        }

        if (done.rt)
        {
            const xhci_comp_code td_comp = done.missed ? COMP_MISSED_INT : comp;
            if (td_comp == COMP_MISSED_INT)
                Kprintf("RT ISO missed-service (%s) addr=%lu ep=%lu frame=%lu len=%lu\n",
                        done.missed ? "skipped" : "reported",
                        (ULONG)udev->slot_id,
                        (ULONG)ep_index,
                        (ULONG)done.rt_frame,
                        (ULONG)done.rt_length);

            rt_iso_deliver(ep_ctx, &done, td_comp);
        }
    } while (done.missed);

    if (done.rt)
    {
        if (!recovering)
            xhci_ep_schedule_rt_iso(ep_ctx);
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

    /* A TRB Error "should" leave the endpoint in the Error state, which only
     * a Set TR Dequeue ends (xHCI 4.8.3) - the same recovery without the
     * reset.  A controller that carries on instead needs none. */
    const BOOL halted = comp_halts_endpoint(comp, req->type) ||
                        (comp == COMP_TRB_ERR && xhci_read_hw_ep_state(udev, ep_index) == EP_STATE_ERROR);
    if (halted && recovering)
    {
        /* A halt on top of a recovery that is under way is not sorted out -
         * it would take a reset in the middle of a stop sequence.  The
         * transfer keeps its error and the endpoint is given up. */
        xhci_xfer_complete(udev, req, status, act_len);
        xhci_ep_set_failed(ep_ctx);
        return;
    }
    if (halted)
    {
        /* answered by the recovery, once the endpoint is ready for the
         * stack's clear-halt */
        req->error = status;
        req->actual = act_len;
        xhci_ep_halted(ep_ctx, req);
        return;
    }

    /* A successful clear-halt may hand its reply to the toggle follow-up, which
     * retires it once the xHC's data toggle matches the device's again.  EP0
     * itself moves on either way. */
    BOOL handed_off = status == UHIOERR_NO_ERROR && req->type == UHCD_EPTYPE_CONTROL &&
                      xhci_ep_clear_halt_follow(udev, req);
    if (!handed_off)
        xhci_xfer_complete(udev, req, status, act_len);
    if (!recovering)
        xhci_ep_set_idle(ep_ctx);
}

/* Hand one retired RT ISO TD to the stream's hooks. */
static void rt_iso_deliver(struct ep_context *ep_ctx, const struct xhci_td_completion *done, xhci_comp_code comp)
{
    /* the *_done hooks' buffer request reports the wire status (§10.3) */
    const u16 ubr_flags = (comp == COMP_SUCCESS || comp == COMP_SHORT_TX ||
                           comp == COMP_STOP || comp == COMP_STOP_INVAL || comp == COMP_STOP_SHORT)
                              ? 0
                              : UHCD_UBF_XFER_ERROR;
    /* a missed interval moved nothing, whatever its event's length field says */
    const u32 act_len = (comp == COMP_MISSED_INT) ? 0 : done->act_len;

    if (done->rt_dir == XHCI_DIR_IN)
    {
        xhci_ep_rt_iso_in(ep_ctx, done->rt_buffer, done->rt_length, act_len, done->rt_frame, ubr_flags);
        xhci_ep_free_rt_iso_buffer(ep_ctx, done->rt_buffer);
    }
    else
        xhci_ep_rt_iso_out(ep_ctx, done->rt_buffer, done->rt_length, act_len, done->rt_frame, ubr_flags);
}
