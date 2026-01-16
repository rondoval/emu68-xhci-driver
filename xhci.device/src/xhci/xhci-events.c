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

static void td_complete(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event, struct xhci_td *td, BOOL allow_control_short);

static const ep_state_handler ep_state_dispatch[] = {
    [USB_DEV_EP_STATE_IDLE] = NULL, /* use default handler */
    [USB_DEV_EP_STATE_RECEIVING_CONTROL] = ep_handle_receiving_control,
    [USB_DEV_EP_STATE_RECEIVING_CONTROL_SHORT] = ep_handle_receiving_control_short,
    [USB_DEV_EP_STATE_RECEIVING_BULK] = ep_handle_receiving_bulk,
    [USB_DEV_EP_STATE_RECEIVING_INT] = ep_handle_receiving_bulk,
    [USB_DEV_EP_STATE_RECEIVING_ISOC] = ep_handle_receiving_bulk,
    [USB_DEV_EP_STATE_ABORTING] = ep_handle_aborting,
    [USB_DEV_EP_STATE_RESETTING] = NULL,
    [USB_DEV_EP_STATE_FAILED] = NULL,
    [USB_DEV_EP_STATE_RT_ISO_STOPPED] = NULL,
    [USB_DEV_EP_STATE_RT_ISO_RUNNING] = ep_handle_receiving_bulk,
    [USB_DEV_EP_STATE_RT_ISO_STOPPING] = ep_handle_receiving_bulk};

static void td_free_all(struct usb_device *udev, struct ep_context *ep_ctx)
{
    struct xhci_ctrl *ctrl = udev->controller;
    struct MinNode *n;

    ObtainSemaphore(&ep_ctx->active_tds_lock);
    while ((n = RemHeadMinList(&ep_ctx->active_tds)) != NULL)
    {
        struct xhci_td *td = (struct xhci_td *)n;
        if (td->req && td->req->iouh_Data)
            xhci_dma_unmap(ctrl, (dma_addr_t)td->req->iouh_Data, td->length);
        xhci_td_release_trbs(td);
        xhci_td_free(ctrl, td);
    }
    ReleaseSemaphore(&ep_ctx->active_tds_lock);
}

static struct xhci_td *td_find_by_trb(struct ep_context *ep_ctx, dma_addr_t trb_addr)
{
    struct MinNode *n = ep_ctx->active_tds.mlh_Head;
    while (n && n->mln_Succ)
    {
        struct xhci_td *td = (struct xhci_td *)n;
        if (td->completion_trb == trb_addr)
            return td;
        if (td->trb_addrs)
        {
            for (unsigned int i = 0; i < td->trb_count; ++i)
            {
                if (td->trb_addrs[i] == trb_addr)
                    return td;
            }
        }
        n = n->mln_Succ;
    }
    return NULL;
}

static BOOL minlist_is_empty(const struct MinList *list)
{
    return list->mlh_Head == (struct MinNode *)&list->mlh_Tail;
}

static void td_remove(struct ep_context *ep_ctx, struct xhci_td *td)
{
    (void)ep_ctx;
    struct MinNode *pred = td->node.mln_Pred;
    struct MinNode *succ = td->node.mln_Succ;
    if (pred)
        pred->mln_Succ = succ;
    if (succ)
        succ->mln_Pred = pred;
    td->node.mln_Pred = NULL;
    td->node.mln_Succ = NULL;
}

static void ep_refresh_timeout(struct ep_context *ep_ctx)
{
    ULONG earliest = 0;
    ObtainSemaphore(&ep_ctx->active_tds_lock);
    struct MinNode *n = ep_ctx->active_tds.mlh_Head;
    while (n && n->mln_Succ)
    {
        struct xhci_td *td = (struct xhci_td *)n;
        if (td->deadline_us != 0 && (earliest == 0 || td->deadline_us < earliest))
            earliest = td->deadline_us;
        n = n->mln_Succ;
    }
    ReleaseSemaphore(&ep_ctx->active_tds_lock);
}

void xhci_td_release_trbs(struct xhci_td *td)
{
    if (!td || !td->ring || td->trb_count == 0)
        return;

    if (td->ring->queued_trbs >= td->trb_count)
        td->ring->queued_trbs -= td->trb_count;
    else
        td->ring->queued_trbs = 0;

    td->ring = NULL;
    td->trb_count = 0;
}

void xhci_td_free(struct xhci_ctrl *ctrl, struct xhci_td *td)
{
    if (!td || !ctrl)
        return;

    if (td->trb_addrs)
    {
        FreeVecPooled(ctrl->memoryPool, td->trb_addrs);
        td->trb_addrs = NULL;
    }

    FreeVecPooled(ctrl->memoryPool, td);
}

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
    td_free_all(udev, ep_ctx);
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
    td_free_all(udev, ep_ctx);
    xhci_ep_schedule_next(udev, endpoint);
}

void xhci_ep_set_receiving(struct usb_device *udev, struct IOUsbHWReq *req, enum ep_state state, dma_addr_t *trb_addrs, ULONG timeout_ms, struct xhci_ring *ring, unsigned int trb_count)
{
    int endpoint = req->iouh_Endpoint & 0xf;
    if (endpoint < 0 || endpoint >= USB_MAXENDPOINTS)
    {
        Kprintf("Invalid endpoint %ld\n", (LONG)endpoint);
        return;
    }

    struct ep_context *ep_ctx = &udev->ep_context[endpoint];
    struct xhci_ctrl *ctrl = udev->controller;

    if (!trb_addrs || trb_count == 0)
    {
        Kprintf("Invalid TRB list for EP %ld\n", (LONG)endpoint);
        xhci_ep_set_failed(udev, endpoint);
        return;
    }

    struct xhci_td *td = AllocVecPooled(ctrl->memoryPool, sizeof(struct xhci_td));
    if (!td)
    {
        Kprintf("Failed to alloc TD\n");
        FreeVecPooled(ctrl->memoryPool, trb_addrs);
        xhci_ep_set_failed(udev, endpoint);
        return;
    }

    td->req = req;

    /*
    * This is a hack for abortio()...
    * We'll put in the TD pointer into DriverPrivate1 so that abortio can find it later.
    */
    req->iouh_Req.io_Message.mn_Node.ln_Pred = NULL;
    if (req->iouh_DriverPrivate1 != (APTR)0xDEAD001)
        req->iouh_DriverPrivate1 = (APTR)td;

    td->trb_addrs = trb_addrs;
    td->trb_count = trb_count;
    td->completion_trb = trb_addrs[trb_count - 1];
    td->length = req->iouh_Length;
    td->rt_iso = (state == USB_DEV_EP_STATE_RT_ISO_RUNNING);
    td->ring = ring;
    if (ring && trb_count)
        ring->queued_trbs += trb_count;
    if (timeout_ms == 0)
    {
        td->deadline_us = 0;
    }
    else
    {
        td->deadline_us = get_time() + timeout_ms * 1000;
    }

    td->node.mln_Pred = td->node.mln_Succ = NULL;   

    ObtainSemaphore(&ep_ctx->active_tds_lock);
    AddTailMinList(&ep_ctx->active_tds, (struct MinNode *)td);
    ReleaseSemaphore(&ep_ctx->active_tds_lock);

    ep_ctx->state = state;

    ep_refresh_timeout(ep_ctx);
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

    if (ep_ctx->state != USB_DEV_EP_STATE_RECEIVING_CONTROL &&
        ep_ctx->state != USB_DEV_EP_STATE_RECEIVING_CONTROL_SHORT &&
        ep_ctx->state != USB_DEV_EP_STATE_RECEIVING_BULK &&
        ep_ctx->state != USB_DEV_EP_STATE_RECEIVING_INT &&
        ep_ctx->state != USB_DEV_EP_STATE_RECEIVING_ISOC &&
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
            if (ep_ctx->state == USB_DEV_EP_STATE_IDLE || ep_ctx->state == USB_DEV_EP_STATE_FAILED)
                continue;

            /* Find earliest-expired TD */
            struct xhci_td *expired = NULL;
            ULONG earliest = 0;
            ObtainSemaphore(&ep_ctx->active_tds_lock);
            struct MinNode *n = ep_ctx->active_tds.mlh_Head;
            while (n && n->mln_Succ)
            {
                struct xhci_td *td = (struct xhci_td *)n;
                if (td->deadline_us != 0 && td->deadline_us <= now)
                {
                    if (!expired || td->deadline_us < earliest)
                    {
                        expired = td;
                        earliest = td->deadline_us;
                    }
                }
                n = n->mln_Succ;
            }

            if (!expired)
            {
                ReleaseSemaphore(&ep_ctx->active_tds_lock);
                ep_refresh_timeout(ep_ctx);
                continue;
            }

            Kprintf("XHCI TD timeout on slot %ld ep %ld (req=%lx deadline=%ld now=%ld)\n",
                    (LONG)udev->slot_id, (LONG)ep, (LONG)expired->req, (LONG)expired->deadline_us, (LONG)now);

            struct IOUsbHWReq *expired_req = expired->req;
            if (expired_req && expired_req->iouh_Data)
                xhci_dma_unmap(ctrl, (dma_addr_t)expired_req->iouh_Data, expired->length);
            td_remove(ep_ctx, expired);

            if (expired->rt_iso)
            {
                if (ep_ctx->rt_inflight_tds > 0)
                    ep_ctx->rt_inflight_tds--;
                if (expired_req)
                {
                    if (ep_ctx->rt_inflight_bytes >= expired->length)
                        ep_ctx->rt_inflight_bytes -= expired->length;
                    else
                        ep_ctx->rt_inflight_bytes = 0;
                    if (expired_req->iouh_Dir == UHDIR_IN)
                        FreeVecPooled(ctrl->memoryPool, expired_req->iouh_Data);
                    FreeVecPooled(ctrl->memoryPool, expired_req);
                }
            }
            else
            {
                io_reply_failed(expired_req, UHIOERR_TIMEOUT);
            }

            xhci_td_free(ctrl, expired);
            ep_refresh_timeout(ep_ctx);

            ReleaseSemaphore(&ep_ctx->active_tds_lock);

            if (minlist_is_empty(&ep_ctx->active_tds))
                xhci_ep_set_idle(udev, ep);
            else
                xhci_ep_schedule_next(udev, ep);
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
        Kprintf("Device stalled\n");
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
        Kprintf("Babble detected\n");
        *status = UHIOERR_BABBLE;
        break;
        //TODO more codes, e.g. underrun/overrun
    case COMP_BUFF_OVER:
        Kprintf("Isoc buffer overrun\n");
        *status = UHIOERR_OVERFLOW;
        break;
    case COMP_BW_OVER:
        Kprintf("Bandwidth overrun\n");
        *status = UHIOERR_HOSTERROR;
        break;
    case COMP_SPLIT_ERR:
        Kprintf("Split transaction error\n");
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
    Kprintf("No handler for endpoint state %ld\n", ep_ctx->state);
    Kprintf("Event TRB: (%08lx %08lx %08lx %08lx)\n",
            (ULONG)LE32(event->generic.field[0]),
            (ULONG)LE32(event->generic.field[1]),
            (ULONG)LE32(event->generic.field[2]),
            (ULONG)LE32(event->generic.field[3]));

    xhci_acknowledge_event(udev->controller);
}

/* Shared TD completion helper for control/bulk/int/isoc (non-RT) and RT ISO hooks. */
static void td_complete(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event, struct xhci_td *td, BOOL allow_control_short)
{
    struct xhci_ctrl *ctrl = udev->controller;
    struct IOUsbHWReq *req = td->req;
    int length = td->length;
    ULONG status;
    int act_len;
    xhci_comp_code comp = GET_COMP_CODE(LE32(event->trans_event.transfer_len));
    ObtainSemaphore(&ep_ctx->active_tds_lock);

    record_transfer_result(event, length, &status, &act_len);
    KprintfH("result status=%ld act_len=%ld comp=%ld\n", (LONG)status, (LONG)act_len, (LONG)comp);
    xhci_acknowledge_event(ctrl);
    xhci_td_release_trbs(td);

    if (req)
    {
        xhci_dma_unmap(ctrl, (dma_addr_t)req->iouh_Data, length);
        //TOOD dir may be wrong for control?
        //if (act_len > 0 && req->iouh_Dir == UHDIR_IN)
            xhci_inval_cache((uintptr_t)req->iouh_Data, (ULONG)act_len);
    }

    if (td->rt_iso)
    {
        KprintfH("RT ISO complete dir=%s len=%ld act_len=%ld inflight_bytes=%lu inflight_tds=%lu\n",
                 req->iouh_Dir == UHDIR_IN ? "IN" : "OUT",
                 (LONG)length,
                 (LONG)act_len,
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
                    xhci_inval_cache((uintptr_t)req->iouh_Data, copy_len);
                    CopyMem(req->iouh_Data, rt_buffer_req.ubr_Buffer, copy_len);
                    rt_buffer_req.ubr_Length = copy_len;

                    CallHookPkt(ep_ctx->rt_req->urti_InDoneHook, ep_ctx->rt_req, &rt_buffer_req);
                }
            }

            FreeVecPooled(ctrl->memoryPool, req->iouh_Data);
        }
        else
        {
            /* OUT path: completion hook only. */
            struct IOUsbHWBufferReq rt_buffer_req;
            rt_buffer_req.ubr_Buffer = req->iouh_Data;
            rt_buffer_req.ubr_Frame = req->iouh_Frame;
            rt_buffer_req.ubr_Length = act_len;
            rt_buffer_req.ubr_Flags = 0;
            if (ep_ctx->rt_req->urti_OutDoneHook)
                CallHookPkt(ep_ctx->rt_req->urti_OutDoneHook, ep_ctx->rt_req, &rt_buffer_req);
        }


        td_remove(ep_ctx, td);
        xhci_td_free(ctrl, td);
        ep_refresh_timeout(ep_ctx);

        int endpoint = req ? (req->iouh_Endpoint & 0xf) : 0;

        /* RT ISO TDs clone IO requests; free them after completion to avoid leaks. */
        if (req)
        {
            FreeVecPooled(ctrl->memoryPool, req);
            td->req = NULL;
        }
        /* Maintain RT ISO inflight counters. */
        if (ep_ctx->rt_inflight_tds > 0)
            ep_ctx->rt_inflight_tds--;
        if (ep_ctx->rt_inflight_bytes >= (ULONG)length)
            ep_ctx->rt_inflight_bytes -= (ULONG)length;

        ReleaseSemaphore(&ep_ctx->active_tds_lock);

        if (ep_ctx->state == USB_DEV_EP_STATE_RT_ISO_RUNNING)
        {
            xhci_ep_schedule_rt_iso(udev, endpoint);
        }
        else if (minlist_is_empty(&ep_ctx->active_tds))
        {
            ep_ctx->state = USB_DEV_EP_STATE_RT_ISO_STOPPED;
            usb_glue_notify_rt_iso_stopped(udev, endpoint);
        }
        return;
    }

    BOOL halted = (comp == COMP_STALL || comp == COMP_BABBLE || comp == COMP_SPLIT_ERR || comp == COMP_TX_ERR);

    /* Flag short IN transfers as runts unless explicitly allowed or expected (control). */
    if (req && status == UHIOERR_NO_ERROR && act_len < length &&
        req->iouh_Dir == UHDIR_IN && !allow_control_short &&
        (req->iouh_Flags & UHFF_ALLOWRUNTPKTS) == 0)
    {
        status = UHIOERR_RUNTPACKET;
    }

    if (req)
    {
        if (halted)
        {
            io_reply_failed(req, status);
        }
        else
        {
            io_reply_data(udev, req, status, act_len);
            if (allow_control_short && comp == COMP_SHORT_TX)
                ep_ctx->state = USB_DEV_EP_STATE_RECEIVING_CONTROL_SHORT;
        }
    }

    td_remove(ep_ctx, td);
    xhci_td_free(ctrl, td);
    ep_refresh_timeout(ep_ctx);

    if (halted)
    {
        /* Track endpoint address for a device-side CLEAR_FEATURE */
        ep_ctx->clear_halt_pending = TRUE;

        xhci_reset_ep(udev, xhci_ep_index_from_parts(req->iouh_Endpoint, req->iouh_Dir));

        ReleaseSemaphore(&ep_ctx->active_tds_lock);
        return;
    }

    ReleaseSemaphore(&ep_ctx->active_tds_lock);
    if (minlist_is_empty(&ep_ctx->active_tds))
        xhci_ep_set_idle(udev, req->iouh_Endpoint);
    else
        xhci_ep_schedule_next(udev, req->iouh_Endpoint);
        //TODO TRB_TO_ENDPOINT(flags)?
}

static void ep_handle_receiving_control(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{
    struct xhci_ctrl *ctrl = udev->controller;

    u32 flags = LE32(event->trans_event.flags);
    int endpoint = TRB_TO_ENDPOINT(flags);
    u64 trb_addr = LE64(event->trans_event.buffer);

    KprintfH("event flags=%08lx xfer_len=%08lx buf=%08lx%08lx\n",
             (ULONG)flags,
             (ULONG)LE32(event->trans_event.transfer_len),
             (ULONG)upper_32_bits(trb_addr),
             (ULONG)lower_32_bits(trb_addr));

    ObtainSemaphore(&ep_ctx->active_tds_lock);
    struct xhci_td *td = td_find_by_trb(ep_ctx, trb_addr);
    ReleaseSemaphore(&ep_ctx->active_tds_lock);
    if (!td)
    {
        /* Late/stale event: if EP not actively receiving, just ack and continue. */
        if (ep_ctx->state == USB_DEV_EP_STATE_RESETTING || ep_ctx->state == USB_DEV_EP_STATE_IDLE || ep_ctx->state == USB_DEV_EP_STATE_FAILED)
        {
            KprintfH("Stale control event for TRB %08lx%08lx on EP %ld state=%ld\n",
                     (ULONG)upper_32_bits(trb_addr), (ULONG)lower_32_bits(trb_addr), (LONG)endpoint, (LONG)ep_ctx->state);
        }
        else
        {
            Kprintf("No TD found for TRB %08lx%08lx on EP %ld (state=%ld)\n",
                    (ULONG)upper_32_bits(trb_addr), (ULONG)lower_32_bits(trb_addr), (LONG)endpoint, (LONG)ep_ctx->state);
        }
        xhci_acknowledge_event(ctrl);
        return;
    }

    /* Reuse shared completion helper for control paths */
    td_complete(udev, ep_ctx, event, td, TRUE);
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

#ifdef DEBUG_HIGH
    struct xhci_container_ctx *out_ctx = ctrl->devs[udev->slot_id]->out_ctx;
    xhci_dump_slot_ctx("xhci_bulk_tx: ", xhci_get_slot_ctx(ctrl, out_ctx));
    xhci_dump_ep_ctx("xhci_bulk_tx: ", endpoint, xhci_get_ep_ctx(ctrl, out_ctx, TRB_TO_EP_INDEX(flags)));
#endif

    KprintfH("event flags=%08lx xfer_len=%08lx buf=%08lx%08lx\n",
             (ULONG)flags,
             (ULONG)LE32(event->trans_event.transfer_len),
             (ULONG)upper_32_bits(trb_addr),
             (ULONG)lower_32_bits(trb_addr));

    /* Isoch OUT rings signal underrun/overrun with null TRB pointers. Do not treat those as lost TDs. */
    int comp = GET_COMP_CODE(LE32(event->trans_event.transfer_len));
    if ((comp == COMP_UNDERRUN || comp == COMP_OVERRUN) && trb_addr == 0)
    {
        Kprintf("Ring %s on EP %ld state=%ld\n",
                (comp == COMP_UNDERRUN) ? "underrun" : "overrun",
                (LONG)endpoint, (LONG)ep_ctx->state);

        xhci_acknowledge_event(ctrl);

        if (ep_ctx->state == USB_DEV_EP_STATE_RT_ISO_RUNNING)
            xhci_ep_schedule_rt_iso(udev, endpoint);

        return;
    }

    ObtainSemaphore(&ep_ctx->active_tds_lock);
    struct xhci_td *td = td_find_by_trb(ep_ctx, trb_addr);
    ReleaseSemaphore(&ep_ctx->active_tds_lock);
    if (!td)
    {
        Kprintf("No TD found for TRB %08lx%08lx  %08lx %08lx on EP %ld\n",
                (ULONG)upper_32_bits(trb_addr), (ULONG)lower_32_bits(trb_addr), (ULONG)flags, (ULONG)LE32(event->trans_event.transfer_len), (LONG)endpoint);
        xhci_acknowledge_event(ctrl);
        return;
    }

    td_complete(udev, ep_ctx, event, td, FALSE);
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
