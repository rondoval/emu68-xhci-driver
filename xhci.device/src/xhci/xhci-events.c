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
#ifdef __INTELLISENSE__
#include <clib/utility_protos.h>
#else
#include <proto/utility.h>
#endif

#include <compat.h>
#include <debug.h>
#include <usb_glue.h>
#include <exec/semaphores.h>

#include <xhci/usb.h>
#include <devices/usbhardware.h>
#include <xhci/xhci.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-events.h>
#include <xhci/xhci-debug.h>
#include <xhci/xhci-td.h>

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
static void ep_handle_receiving_generic(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event);
static void ep_handle_receiving_control_short(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event);
static void ep_handle_aborting(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event);

static const ep_state_handler ep_state_dispatch[] = {
    [USB_DEV_EP_STATE_IDLE] = NULL, /* use default handler */
    [USB_DEV_EP_STATE_RECEIVING_CONTROL_SHORT] = ep_handle_receiving_control_short,
    [USB_DEV_EP_STATE_RECEIVING] = ep_handle_receiving_generic,
    [USB_DEV_EP_STATE_ABORTING] = ep_handle_aborting,
    [USB_DEV_EP_STATE_RESETTING] = NULL,
    [USB_DEV_EP_STATE_FAILED] = NULL,
    [USB_DEV_EP_STATE_RT_ISO_STOPPED] = NULL,
    [USB_DEV_EP_STATE_RT_ISO_RUNNING] = ep_handle_receiving_generic,
    [USB_DEV_EP_STATE_RT_ISO_STOPPING] = ep_handle_receiving_generic};

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
    xhci_td_fail_all(ep_ctx->active_tds, UHIOERR_HOSTERROR);
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
    xhci_ep_schedule_next(udev, endpoint);
}

void xhci_ep_set_receiving(struct usb_device *udev, struct IOUsbHWReq *req, enum ep_state state, dma_addr_t *trb_addrs, ULONG timeout_ms, unsigned int trb_count)
{
    int endpoint = req->iouh_Endpoint & 0xf;

    struct ep_context *ep_ctx = &udev->ep_context[endpoint];
    struct xhci_ctrl *ctrl = udev->controller;

    if (!trb_addrs || trb_count == 0)
    {
        Kprintf("Invalid TRB list for EP %ld\n", (LONG)endpoint);
        xhci_ep_set_failed(udev, endpoint);
        return;
    }

    BOOL result = xhci_td_add(ep_ctx->active_tds,
                              req,
                              timeout_ms,
                              (state == USB_DEV_EP_STATE_RT_ISO_RUNNING),
                              trb_addrs,
                              trb_count);
    if (!result)
    {
        Kprintf("Failed to add TD to active list\n");
        FreeVecPooled(ctrl->memoryPool, trb_addrs);
        xhci_ep_set_failed(udev, endpoint);
        return;
    }

    ep_ctx->state = state;
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
}

void xhci_ep_set_aborting(struct usb_device *udev, int endpoint)
{
    if (endpoint < 0 || endpoint >= USB_MAXENDPOINTS)
    {
        Kprintf("Invalid endpoint %ld\n", (LONG)endpoint);
        return;
    }

    struct ep_context *ep_ctx = &udev->ep_context[endpoint];

    if (ep_ctx->state != USB_DEV_EP_STATE_RECEIVING &&
        ep_ctx->state != USB_DEV_EP_STATE_RECEIVING_CONTROL_SHORT &&
        ep_ctx->state != USB_DEV_EP_STATE_RT_ISO_RUNNING)
    {
        Kprintf("EP %ld not RECEIVING (state %ld)\n", (LONG)endpoint, (LONG)ep_ctx->state);
        xhci_ep_set_failed(udev, endpoint);
        return;
    }

    ep_ctx->state = USB_DEV_EP_STATE_ABORTING;
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
        KprintfH("addr %ld EP %ld state %ld -> handling event\n", udev->poseidon_address, (LONG)endpoint, (LONG)ep_ctx->state);
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
            KprintfH("USB device addr %ld on slot %ld\n", (LONG)udev->poseidon_address, (LONG)slot);

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

/*
 * This is supposed to implement NAK timeouts.
 * It scans all active TDs for expired timeouts,
 * if it finds one, it stops the endpoint, aborts all active TDs
 * and resets dequeue pointer.
 * I guess we could use something more sophisticated, like
 * stopping, removing the expired TD and restarting.
 */
void xhci_process_event_timeouts(struct xhci_ctrl *ctrl)
{
    for (int i = 0; i < USB_MAX_ADDRESS; i++)
    {
        struct usb_device *udev = ctrl->devices_by_poseidon_address[i];
        if (!udev)
            continue;

        for (int ep = 0; ep < USB_MAXENDPOINTS; ep++)
        {
            struct ep_context *ep_ctx = &udev->ep_context[ep];
            BOOL expired = xhci_td_is_expired(ep_ctx->active_tds);
            if (expired)
            {
                KprintfH("XHCI TD timeout on slot %ld ep %ld\n", (LONG)udev->slot_id, (LONG)ep);

                xhci_stop_endpoint(udev, ep);
            }
        }
    }
}

static void record_transfer_result(union xhci_trb *event, ULONG length,
                                   ULONG *status, ULONG *act_len)
{
    xhci_comp_code comp = GET_COMP_CODE(LE32(event->trans_event.transfer_len));
    *act_len = min(length, length - (int)EVENT_TRB_LEN(LE32(event->trans_event.transfer_len)));

    KprintfH("comp=%ld length=%ld act_len=%ld transfer_len=%ld\n",
             (LONG)comp, (LONG)length, (LONG)*act_len,
             (LONG)EVENT_TRB_LEN(LE32(event->trans_event.transfer_len)));

    switch (comp)
    {
    case COMP_SUCCESS:
        if (*act_len != length)
        {
            KprintfH("Short transfer: expected %ld, got %ld\n",
                     length, *act_len);
        }
        /* fallthrough */
    case COMP_SHORT_TX:
        *status = UHIOERR_NO_ERROR;
        break;
    case COMP_STALL:
        KprintfH("Device stalled\n");
        *status = UHIOERR_STALL;
        break;
    case COMP_TX_ERR:
        Kprintf("USB transaction error\n");
        *status = UHIOERR_TIMEOUT;
        break;
    case COMP_DB_ERR:
    case COMP_TRB_ERR:
        // Data Buffer Error or TRB Error
        Kprintf("TRB error\n");
        *status = UHIOERR_HOSTERROR;
        break;
    case COMP_BABBLE:
        KprintfH("Babble detected\n");
        *status = UHIOERR_BABBLE;
        break;
        // TODO more codes, e.g. underrun/overrun
    case COMP_BUFF_OVER:
        KprintfH("Isoc buffer overrun\n");
        *status = UHIOERR_OVERFLOW;
        break;
    case COMP_BW_OVER:
        KprintfH("Bandwidth overrun\n");
        *status = UHIOERR_HOSTERROR;
        break;
    case COMP_SPLIT_ERR:
        KprintfH("Split transaction error\n");
        *status = UHIOERR_TIMEOUT;
        break;
    default:
        Kprintf("Unhandled completion code %ld\n", (LONG)comp);
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
    (void)ep_ctx;
    (void)event;
    Kprintf("No handler for endpoint state %ld addr %ld\n", ep_ctx->state, udev->poseidon_address);
    KprintfH("Event TRB: (%08lx %08lx %08lx %08lx)\n",
             (ULONG)LE32(event->generic.field[0]),
             (ULONG)LE32(event->generic.field[1]),
             (ULONG)LE32(event->generic.field[2]),
             (ULONG)LE32(event->generic.field[3]));

    xhci_acknowledge_event(udev->controller);
}

static void ep_handle_receiving_generic(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{
    struct xhci_ctrl *ctrl = udev->controller;

    u32 flags = LE32(event->trans_event.flags);
    int endpoint = TRB_TO_ENDPOINT(flags);
    u64 trb_addr = LE64(event->trans_event.buffer);

#ifdef DEBUG_HIGH
    KprintfH("event flags=%08lx xfer_len=%08lx buf=%08lx%08lx\n",
             (ULONG)flags,
             (ULONG)LE32(event->trans_event.transfer_len),
             (ULONG)upper_32_bits(trb_addr),
             (ULONG)lower_32_bits(trb_addr));
    struct xhci_container_ctx *out_ctx = ctrl->devs[udev->slot_id]->out_ctx;
    xhci_dump_slot_ctx("xhci_bulk_tx: ", xhci_get_slot_ctx(ctrl, out_ctx));
    xhci_dump_ep_ctx("xhci_bulk_tx: ", endpoint, xhci_get_ep_ctx(ctrl, out_ctx, TRB_TO_EP_INDEX(flags)));
#endif

    xhci_acknowledge_event(ctrl);

    /* Isoch OUT rings signal underrun/overrun with null TRB pointers. Do not treat those as lost TDs. */
    xhci_comp_code comp = GET_COMP_CODE(LE32(event->trans_event.transfer_len));
    if ((comp == COMP_UNDERRUN || comp == COMP_OVERRUN) && trb_addr == 0)
    {
        Kprintf("Ring %s on addr %ld EP %ld state=%ld inflight_tds=%lu inflight_bytes=%lu\n",
                (comp == COMP_UNDERRUN) ? "underrun" : "overrun",
                (LONG)udev->poseidon_address,
                (LONG)endpoint,
                (LONG)ep_ctx->state,
                (ULONG)ep_ctx->rt_inflight_tds,
                (ULONG)ep_ctx->rt_inflight_bytes);

        if (ep_ctx->state == USB_DEV_EP_STATE_RT_ISO_RUNNING)
            xhci_ep_schedule_rt_iso(udev, endpoint);

        return;
    }

    struct IOUsbHWReq *req = xhci_td_get_by_trb(ep_ctx->active_tds, trb_addr);
    if (!req)
    {
        Kprintf("No TD found for TRB %08lx%08lx  %08lx %08lx on EP %ld\n",
                (ULONG)upper_32_bits(trb_addr), (ULONG)lower_32_bits(trb_addr), (ULONG)flags, (ULONG)LE32(event->trans_event.transfer_len), (LONG)endpoint);
        return;
    }

    ULONG length = req->iouh_Length;
    ULONG status;
    ULONG act_len;
    record_transfer_result(event, length, &status, &act_len);
    KprintfH("result status=%ld act_len=%ld comp=%ld\n", (LONG)status, (LONG)act_len, (LONG)comp);

    BOOL is_rt_iso = req->iouh_Req.io_Command == UHCMD_ADDISOHANDLER;
    if (is_rt_iso)
    {
        KprintfH("RT ISO complete dir=%s len=%lu act_len=%lu inflight_bytes=%lu inflight_tds=%lu\n",
                 req->iouh_Dir == UHDIR_IN ? "IN" : "OUT",
                 (ULONG)length,
                 (ULONG)act_len,
                 (ULONG)ep_ctx->rt_inflight_bytes,
                 (ULONG)ep_ctx->rt_inflight_tds);

        if (req->iouh_Dir == UHDIR_IN)
        {
            if (act_len > 0)
            {
                /* Pull a destination buffer from the class, copy staged DMA into it, then signal completion. */
                struct IOUsbHWBufferReq rt_buffer_req;
                rt_buffer_req.ubr_Frame = req->iouh_Frame;
                rt_buffer_req.ubr_Flags = 0;
                rt_buffer_req.ubr_Length = (ULONG)act_len;
                rt_buffer_req.ubr_Buffer = NULL;

                CallHookPkt(ep_ctx->rt_req->urti_InReqHook, ep_ctx->rt_req, &rt_buffer_req);

                if (rt_buffer_req.ubr_Buffer)
                {
                    ULONG copy_len = min((ULONG)act_len, rt_buffer_req.ubr_Length);
                    // xhci_inval_cache((uintptr_t)req->iouh_Data, copy_len);
                    CopyMem(req->iouh_Data, rt_buffer_req.ubr_Buffer, copy_len);
                    rt_buffer_req.ubr_Length = copy_len;

                    CallHookPkt(ep_ctx->rt_req->urti_InDoneHook, ep_ctx->rt_req, &rt_buffer_req);
                }
            }

            FreeVecPooled(ctrl->memoryPool, req->iouh_Data);
        }
        else
        {
            /* OUT path: completion hook only. OutReqHook is before transfer. */
            struct IOUsbHWBufferReq rt_buffer_req;
            rt_buffer_req.ubr_Buffer = req->iouh_Data;
            rt_buffer_req.ubr_Frame = req->iouh_Frame;
            rt_buffer_req.ubr_Length = act_len;
            rt_buffer_req.ubr_Flags = 0;
            if (ep_ctx->rt_req->urti_OutDoneHook)
                CallHookPkt(ep_ctx->rt_req->urti_OutDoneHook, ep_ctx->rt_req, &rt_buffer_req);
        }

        /* RT ISO TDs clone IO requests; free them after completion to avoid leaks. */
        FreeVecPooled(ctrl->memoryPool, req);

        /* Maintain RT ISO inflight counters. */
        if (ep_ctx->rt_inflight_tds > 0)
            ep_ctx->rt_inflight_tds--;
        if (ep_ctx->rt_inflight_bytes >= length)
            ep_ctx->rt_inflight_bytes -= length;

        if (ep_ctx->state == USB_DEV_EP_STATE_RT_ISO_RUNNING)
        {
            xhci_ep_schedule_rt_iso(udev, endpoint);
        }
        else if (xhci_td_is_empty(ep_ctx->active_tds))
        {
            ep_ctx->state = USB_DEV_EP_STATE_RT_ISO_STOPPED;
            usb_glue_notify_rt_iso_stopped(udev, endpoint);
        }
        return;
    }

    /* Flag short IN transfers as runts unless explicitly allowed or expected (control). */
    if (status == UHIOERR_NO_ERROR && act_len < req->iouh_Length &&
        req->iouh_Dir == UHDIR_IN && req->iouh_Req.io_Command != UHCMD_CONTROLXFER &&
        (req->iouh_Flags & UHFF_ALLOWRUNTPKTS) == 0)
    {
        status = UHIOERR_RUNTPACKET;
    }

    BOOL halted = (comp == COMP_STALL || comp == COMP_BABBLE || comp == COMP_SPLIT_ERR || comp == COMP_TX_ERR);
    if (halted)
    {
        io_reply_failed(req, status);

        /* Track endpoint address for a device-side CLEAR_FEATURE */
        ep_ctx->clear_halt_pending = TRUE;
        xhci_reset_ep(udev, xhci_ep_index_from_parts(req->iouh_Endpoint, req->iouh_Dir));
        return;
    }

    io_reply_data(udev, req, status, act_len);
    if (req->iouh_Req.io_Command == UHCMD_CONTROLXFER && comp == COMP_SHORT_TX)
        ep_ctx->state = USB_DEV_EP_STATE_RECEIVING_CONTROL_SHORT;
    else if (xhci_td_is_empty(ep_ctx->active_tds))
        xhci_ep_set_idle(udev, endpoint);
    else
        xhci_ep_schedule_next(udev, endpoint);
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

    xhci_acknowledge_event(ctrl);
    /* no state change - that is done by handle_abort_stop_ring */
}
