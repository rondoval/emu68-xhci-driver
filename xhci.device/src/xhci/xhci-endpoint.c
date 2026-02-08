#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#include <clib/utility_protos.h>
#else
#include <proto/exec.h>
#include <proto/utility.h>
#endif

#include <compat.h>
#include <debug.h>
#include <config.h>
#include <minlist.h>

#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-td.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci.h>
#include <xhci/xhci-ring.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-endpoint] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci-endpoint] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

struct ep_context
{
    struct usb_device *udev; /* back reference to device */
    int ep_index;            /* Endpoint context index (0-30) */
    enum ep_state state;     /* Current endpoint state */
    APTR memoryPool;         /* Memory pool for this endpoint */

    IOReqList pending_reqs;             /* list of pending requests */
    TransferDescriptorList *active_tds; /* list of in-flight TDs */

    struct xhci_ring *ring; /* ring for this endpoint */

    /* RT ISO data */
    /*
     * The idea here is that if this is filled, the event handlers will use
     * hooks to get more data.
     * As such, we will fail an attempt to enter RT ISO mode if there are requests ongoing.
     */
    struct IOUsbHWRTIso *rt_req;
    struct IOUsbHWReq *rt_template_req; /* template for cloning per TD */

    /* Pending STOPRTISO command to reply once the pipe is fully stopped */
    struct IOUsbHWReq *rt_stop_pending;

    /* RT ISO frame tracking (monotonic frame number modulo 2^16) */
    ULONG rt_next_frame;

    /* Cache the last RT ISO buffer so hooks can omit repeating it */
    APTR rt_last_buffer;
    ULONG rt_last_filled;

    /* RT ISO inflight accounting */
    ULONG rt_inflight_bytes;
};

BOOL xhci_ep_create_context(struct usb_device *udev, int ep_index, APTR memoryPool)
{
    struct ep_context *ep_ctx = AllocVecPooled(memoryPool, sizeof(struct ep_context));
    if (!ep_ctx)
    {
        Kprintf("Failed to allocate ep_context for EP %d\n", ep_index);
        return FALSE;
    }
    ep_ctx->udev = udev;
    ep_ctx->ep_index = ep_index;
    ep_ctx->memoryPool = memoryPool;
    ep_ctx->state = USB_DEV_EP_STATE_IDLE;
    _NewMinList(&ep_ctx->pending_reqs);
    ep_ctx->active_tds = xhci_td_create_list(udev->controller);
    ep_ctx->ring = xhci_ring_alloc(udev->controller, XHCI_SEGMENTS_PER_RING, /*link_trbs*/ TRUE, /*is_event_ring*/ FALSE, ep_index);
    if (!ep_ctx->active_tds || !ep_ctx->ring)
    {
        Kprintf("Failed to create resources for EP %d\n", ep_index);
        if (ep_ctx->active_tds)
            xhci_td_destroy_list(ep_ctx->active_tds, UHIOERR_OUTOFMEMORY);
        if (ep_ctx->ring)
            xhci_ring_free(udev->controller, ep_ctx->ring);
        FreeVecPooled(memoryPool, ep_ctx);
        return FALSE;
    }

    udev->ep_context[ep_index] = ep_ctx;
    return TRUE;
}

void xhci_ep_destroy_contexts(struct usb_device *udev, BYTE reply_code)
{
    for (int i = 0; i < USB_MAX_ENDPOINT_CONTEXTS; ++i)
    {
        struct ep_context *ep_ctx = udev->ep_context[i];
        if (ep_ctx)
        {
            KprintfH("tearing down addr %ld EP %ld context, state %ld\n", (LONG)udev->poseidon_address, (LONG)i, (LONG)ep_ctx->state);
            struct MinNode *node;
            while ((node = RemHeadMinList(&ep_ctx->pending_reqs)) != NULL)
            {
                struct IOUsbHWReq *req = (struct IOUsbHWReq *)node;
                xhci_udev_io_reply_failed(req, reply_code);
            }

            xhci_td_destroy_list(ep_ctx->active_tds, reply_code);

            if (ep_ctx->rt_template_req)
            {
                FreeVecPooled(ep_ctx->memoryPool, ep_ctx->rt_template_req);
                ep_ctx->rt_template_req = NULL;
            }
            if (ep_ctx->rt_stop_pending)
            {
                xhci_udev_io_reply_failed(ep_ctx->rt_stop_pending, reply_code);
                ep_ctx->rt_stop_pending = NULL;
            }
            ep_ctx->rt_req = NULL;
            ep_ctx->state = USB_DEV_EP_STATE_IDLE;

            if (ep_ctx->ring)
                xhci_ring_free(udev->controller, ep_ctx->ring);

            FreeVecPooled(ep_ctx->memoryPool, ep_ctx);
            udev->ep_context[i] = NULL;
        }
    }
}

struct ep_context *xhci_ep_get_context_for_index(struct usb_device *udev, int ep_index)
{
    if (ep_index < 0 || ep_index >= USB_MAX_ENDPOINT_CONTEXTS)
    {
        Kprintf("Invalid endpoint index %ld\n", (LONG)ep_index);
        return NULL;
    }

    return udev->ep_context[ep_index];
}

/*
 * So generally, XHCI has got separate endpoint context per direction
 * except for control endpoints.
 * But Poseidon assumes there is only one transfer ongoing per endpoint number,
 * regardless of direction (I think).
 * We're currently treating each endpoint context separately, so it may cause issues.
 * endpoint - endpoint number (0-15)
 * ep_index - endpoint context index (0-30)
 * DCI - context index (1-31)
 * endpoint = DCI >> 1
 * DCI = ep_index + 1
 */

void xhci_ep_set_failed(struct ep_context *ep_ctx)
{
    Kprintf("EP %ld state %ld -> FAILED\n", (LONG)ep_ctx->ep_index, (LONG)ep_ctx->state);
    ep_ctx->state = USB_DEV_EP_STATE_FAILED;
    xhci_td_fail_all(ep_ctx->active_tds, UHIOERR_HOSTERROR);
}

void xhci_ep_enqueue(struct ep_context *ep_ctx, struct IOUsbHWReq *io)
{
    if ((ULONG)io->iouh_DriverPrivate1 & REQ_ENQUEUED)
        AddHeadMinList(&ep_ctx->pending_reqs, (struct MinNode *)io);
    else
    {
        io->iouh_DriverPrivate1 = (APTR)((ULONG)io->iouh_DriverPrivate1 | REQ_ENQUEUED);
        AddTailMinList(&ep_ctx->pending_reqs, (struct MinNode *)io);
    }

    KprintfH("Ring busy, queued request cmd=%ld ep=%ld\n",
             (LONG)io->iouh_Req.io_Command,
             (LONG)(io->iouh_Endpoint & 0x0F));
}

static void xhci_ep_schedule_next(struct ep_context *ep_ctx)
{
    struct MinNode *node;
    while ((node = RemHeadMinList(&ep_ctx->pending_reqs)))
    {
        struct IOUsbHWReq *req = (struct IOUsbHWReq *)node;

        KprintfH("starting queued request cmd=%ld ep=%ld\n",
                 (LONG)req->iouh_Req.io_Command,
                 (LONG)(req->iouh_Endpoint & 0x0F));

        int err = xhci_udev_send(req);
        if (err != UHIOERR_NO_ERROR)
        {
            req->iouh_Req.io_Error = err;
            xhci_udev_io_reply_failed(req, err);
            continue;
        }

        /* If the endpoint went idle immediately (e.g. zero-length or error), keep draining */
        if (xhci_ep_get_state(ep_ctx) == USB_DEV_EP_STATE_FAILED)
            return;
    }
}

void xhci_ep_set_idle(struct ep_context *ep_ctx)
{
    KprintfH("EP %ld state %ld -> IDLE\n", (LONG)ep_ctx->ep_index, (LONG)ep_ctx->state);
    ep_ctx->state = USB_DEV_EP_STATE_IDLE;
    if (!xhci_td_is_empty(ep_ctx->active_tds))
        xhci_ep_schedule_next(ep_ctx);
}

void xhci_ep_set_receiving(struct ep_context *ep_ctx, struct IOUsbHWReq *req, dma_addr_t *trb_addrs, ULONG timeout_ms, unsigned int trb_count)
{
    if (!trb_addrs || trb_count == 0)
    {
        Kprintf("Invalid TRB list for EP %ld\n", (LONG)ep_ctx->ep_index);
        xhci_ep_set_failed(ep_ctx);
        return;
    }

    enum ep_state new_state;
    BOOL is_rt;
    if (ep_ctx->state == USB_DEV_EP_STATE_RT_ISO_STOPPED ||
        ep_ctx->state == USB_DEV_EP_STATE_RT_ISO_RUNNING ||
        ep_ctx->state == USB_DEV_EP_STATE_RT_ISO_STOPPING)
    {
        new_state = USB_DEV_EP_STATE_RT_ISO_RUNNING;
        is_rt = TRUE;
    }
    else
    {
        new_state = USB_DEV_EP_STATE_RECEIVING;
        is_rt = FALSE;
    }

    BOOL result = xhci_td_add(ep_ctx->active_tds,
                              req,
                              timeout_ms,
                              is_rt,
                              trb_addrs,
                              trb_count);
    if (!result)
    {
        Kprintf("Failed to add TD to active list\n");
        FreeVecPooled(ep_ctx->memoryPool, trb_addrs);
        xhci_ep_set_failed(ep_ctx);
        return;
    }

    KprintfH("EP %ld state %ld -> %ld\n", (LONG)ep_ctx->ep_index, (LONG)ep_ctx->state, (LONG)new_state);
    ep_ctx->state = new_state;
}

void xhci_ep_set_receiving_control_short(struct ep_context *ep_ctx)
{
    KprintfH("EP %ld state %ld -> RECEIVING_CONTROL_SHORT\n", (LONG)ep_ctx->ep_index, (LONG)ep_ctx->state);
    ep_ctx->state = USB_DEV_EP_STATE_RECEIVING_CONTROL_SHORT;
}

void xhci_ep_set_resetting(struct ep_context *ep_ctx)
{
    ep_ctx->state = USB_DEV_EP_STATE_RESETTING;

    /* Fail and free any in-flight TDs so callers get a reply before reset. */
    xhci_td_fail_all(ep_ctx->active_tds, UHIOERR_TIMEOUT);
}

void xhci_ep_set_aborting(struct ep_context *ep_ctx)
{
    ep_ctx->state = USB_DEV_EP_STATE_ABORTING;
}

enum ep_state xhci_ep_get_state(struct ep_context *ep_ctx)
{
    return ep_ctx->state;
}

int xhci_ep_get_ep_index(struct ep_context *ep_ctx)
{
    return ep_ctx->ep_index;
}

BOOL xhci_ep_is_expired(struct ep_context *ep_ctx)
{
    TransferDescriptorList *td_list = ep_ctx->active_tds;

    if (!td_list)
        return FALSE;

    return xhci_td_is_expired(td_list);
}

struct xhci_ring *xhci_ep_get_ring(struct ep_context *ep_ctx)
{
    return ep_ctx->ring;
}

struct IOUsbHWReq *xhci_ep_get_by_trb(struct ep_context *ep_ctx, dma_addr_t trb_addr)
{
    TransferDescriptorList *td_list = ep_ctx->active_tds;

    if (!td_list)
        return NULL;

    return xhci_td_get_by_trb(td_list, trb_addr);
}

int xhci_ep_get_active_trb_count(struct ep_context *ep_ctx)
{
    return xhci_td_get_queued_trb_count(ep_ctx->active_tds);
}

inline static int xhci_ep_get_active_td_count(struct ep_context *ep_ctx)
{
    return xhci_td_get_queued_td_count(ep_ctx->active_tds);
}

void xhci_ep_flush(struct ep_context *ep_ctx, BYTE reply_code)
{
    struct MinNode *node;
    while ((node = RemHeadMinList(&ep_ctx->pending_reqs)) != NULL)
    {
        struct IOUsbHWReq *req = (struct IOUsbHWReq *)node;
        xhci_udev_io_reply_failed(req, reply_code);
    }

    xhci_td_fail_all(ep_ctx->active_tds, reply_code);
}

/*
 * RT ISO functions
 */

inline static void xhci_ep_rt_iso_update_counters(struct ep_context *ep_ctx, struct IOUsbHWReq *req)
{
    /* Maintain RT ISO inflight counters. */
    if (ep_ctx->rt_inflight_bytes >= req->iouh_Length)
        ep_ctx->rt_inflight_bytes -= req->iouh_Length;
}

inline static void xhci_ep_rt_iso_zero_counters(struct ep_context *ep_ctx)
{
    ep_ctx->rt_inflight_bytes = 0;
}

static void xhci_ep_set_rt_stopped(struct ep_context *ep_ctx)
{
    ep_ctx->state = USB_DEV_EP_STATE_RT_ISO_STOPPED;
    ep_ctx->rt_stop_pending = NULL;
    ep_ctx->rt_last_buffer = NULL;
    ep_ctx->rt_last_filled = 0;
    xhci_ep_rt_iso_zero_counters(ep_ctx);
}

BYTE xhci_ep_rt_iso_add_handler(struct ep_context *ep_ctx, struct IOUsbHWReq *req)
{
    if (!req || !ep_ctx)
        return FALSE;

    if (ep_ctx->state != USB_DEV_EP_STATE_IDLE)
    {
        /* can only enable RT ISO if EP is idle */
        Kprintf("EP not idle. Current state: %ld\n", ep_ctx->state);
        return UHIOERR_HOSTERROR;
    }

    xhci_ep_set_rt_stopped(ep_ctx);

    ep_ctx->rt_req = (struct IOUsbHWRTIso *)req->iouh_Data;
    ep_ctx->rt_template_req = AllocVecPooled(ep_ctx->memoryPool, sizeof(struct IOUsbHWReq));
    if (!ep_ctx->rt_template_req)
    {
        Kprintf("Failed to allocate memory\n");
        return UHIOERR_OUTOFMEMORY;
    }

    CopyMem(req, ep_ctx->rt_template_req, sizeof(struct IOUsbHWReq));

#ifdef DEBUG_HIGH
    struct IOUsbHWRTIso *rt = (struct IOUsbHWRTIso *)req->iouh_Data;
    if (req->iouh_Dir == UHDIR_IN)
    {
        KprintfH("Added ISO handler: EP %ld in req hook: %lx, in done hook: %lx, prefetch: %ld\n", ep_ctx->ep_index, rt->urti_InReqHook, rt->urti_InDoneHook, rt->urti_OutPrefetch);
    }
    else
    {
        KprintfH("Added ISO handler: EP %ld out req hook: %lx, out done hook: %lx, prefetch: %ld\n", ep_ctx->ep_index, rt->urti_OutReqHook, rt->urti_OutDoneHook, rt->urti_OutPrefetch);
    }
#endif
    return UHIOERR_NO_ERROR;
}

BYTE xhci_ep_rt_iso_rem_handler(struct ep_context *ep_ctx, struct IOUsbHWReq *req)
{
    if (!req || !ep_ctx)
    {
        Kprintf("Invalid parameters to remove RT ISO handler\n");
        return UHIOERR_BADPARAMS;
    }

    if (ep_ctx->state != USB_DEV_EP_STATE_RT_ISO_STOPPED)
    {
        Kprintf("EP not in RT_ISO_STOPPED/IDLE state. Current state: %ld\n", ep_ctx->state);
        return UHIOERR_HOSTERROR;
    }

    if (req->iouh_Data != ep_ctx->rt_req)
    {
        Kprintf("Mismatched RT ISO handler removal request\n");
        return UHIOERR_BADPARAMS;
    }

    ep_ctx->rt_req = NULL;
    if (ep_ctx->rt_template_req)
    {
        FreeVecPooled(ep_ctx->memoryPool, ep_ctx->rt_template_req);
        ep_ctx->rt_template_req = NULL;
    }
    xhci_ep_set_idle(ep_ctx);
    KprintfH("Successfully removed ISO handler (state reset to IDLE)\n");
    return UHIOERR_NO_ERROR;
}

void xhci_ep_rt_iso_in(struct ep_context *ep_ctx, struct IOUsbHWReq *req, ULONG act_len)
{
    /* Pull a destination buffer from the class, copy staged DMA into it, then signal completion. */
    struct IOUsbHWBufferReq rt_buffer_req;
    rt_buffer_req.ubr_Frame = req->iouh_Frame;
    rt_buffer_req.ubr_Flags = 0;
    rt_buffer_req.ubr_Length = act_len;
    rt_buffer_req.ubr_Buffer = NULL;

    CallHookPkt(ep_ctx->rt_req->urti_InReqHook, ep_ctx->rt_req, &rt_buffer_req);

    if (rt_buffer_req.ubr_Buffer)
    {
        ULONG copy_len = min(act_len, rt_buffer_req.ubr_Length);
        CopyMem(req->iouh_Data, rt_buffer_req.ubr_Buffer, copy_len);
        rt_buffer_req.ubr_Length = copy_len;

        CallHookPkt(ep_ctx->rt_req->urti_InDoneHook, ep_ctx->rt_req, &rt_buffer_req);
    }

    xhci_ep_rt_iso_update_counters(ep_ctx, req);
}

void xhci_ep_rt_iso_out(struct ep_context *ep_ctx, struct IOUsbHWReq *req, ULONG act_len)
{
    if (ep_ctx->rt_req->urti_OutDoneHook)
    {
        /* OUT path: completion hook only. OutReqHook is before transfer. */
        struct IOUsbHWBufferReq rt_buffer_req;
        rt_buffer_req.ubr_Buffer = req->iouh_Data;
        rt_buffer_req.ubr_Frame = req->iouh_Frame;
        rt_buffer_req.ubr_Length = act_len;
        rt_buffer_req.ubr_Flags = 0;
        CallHookPkt(ep_ctx->rt_req->urti_OutDoneHook, ep_ctx->rt_req, &rt_buffer_req);
    }

    xhci_ep_rt_iso_update_counters(ep_ctx, req);
}

static void xhci_ep_schedule_rt_iso_out(struct ep_context *ep_ctx)
{
    struct IOUsbHWReq *template = ep_ctx->rt_template_req;
    ULONG prefetch_bytes = ep_ctx->rt_req->urti_OutPrefetch;

    while (ep_ctx->rt_inflight_bytes < prefetch_bytes)
    {
        ULONG frame = ep_ctx->rt_next_frame;
        struct IOUsbHWReq *rt_io = AllocVecPooled(ep_ctx->memoryPool, sizeof(struct IOUsbHWReq));
        if (!rt_io)
        {
            Kprintf("Failed to alloc RT ISO IO req\n");
            break;
        }
        CopyMem(template, rt_io, sizeof(struct IOUsbHWReq));

        struct IOUsbHWBufferReq rt_buffer_req;
        rt_buffer_req.ubr_Length = prefetch_bytes;
        rt_buffer_req.ubr_Buffer = ep_ctx->rt_last_buffer;
        rt_buffer_req.ubr_Flags = 0;
        rt_buffer_req.ubr_Frame = frame; /* monotonic frame counter to avoid jumps */

        KprintfH("RT ISO OUT sched frame=%lu len=%lu inflight_bytes=%lu inflight_tds=%lu\n",
                 (ULONG)frame,
                 (ULONG)rt_buffer_req.ubr_Length,
                 (ULONG)ep_ctx->rt_inflight_bytes,
                 (ULONG)xhci_ep_get_active_td_count(ep_ctx));

        CallHookPkt(ep_ctx->rt_req->urti_OutReqHook, ep_ctx->rt_req, &rt_buffer_req);

        if (!rt_buffer_req.ubr_Buffer || rt_buffer_req.ubr_Length == 0)
        {
            KprintfH("RT ISO hook provided no buffer/length\n");
            FreeVecPooled(ep_ctx->memoryPool, rt_io);
            break;
        }

        ULONG offset = 0;
        if (rt_buffer_req.ubr_Buffer == ep_ctx->rt_last_buffer)
        {
            offset = ep_ctx->rt_last_filled;
        }
        else
        {
            ep_ctx->rt_last_buffer = rt_buffer_req.ubr_Buffer;
        }
        ep_ctx->rt_last_filled = offset + rt_buffer_req.ubr_Length;

        if (rt_buffer_req.ubr_Flags & UBFF_CONTBUFFER)
        {
            Kprintf("Continuous buffer not supported yet\n");
        }

        rt_io->iouh_Data = (APTR)((ULONG)rt_buffer_req.ubr_Buffer + offset);
        rt_io->iouh_Length = rt_buffer_req.ubr_Length;
        rt_io->iouh_Frame = (UWORD)frame;

        int ret = xhci_ring_enqueue_td(ep_ctx->udev, rt_io, 0, TRUE /* RT ISO defers doorbell to per-run giveback */);
        if (ret != UHIOERR_NO_ERROR)
        {
            FreeVecPooled(ep_ctx->memoryPool, rt_io);
            Kprintf("RT ISO submit failed %ld\n", (LONG)ret);
            break;
        }

        ep_ctx->rt_next_frame = (frame + 1) & 0xffff;
        ep_ctx->rt_inflight_bytes += rt_io->iouh_Length;
        KprintfH("RT ISO OUT queued frame=%lu len=%lu inflight_bytes=%lu inflight_tds=%lu\n",
                 (ULONG)frame,
                 (ULONG)rt_io->iouh_Length,
                 (ULONG)ep_ctx->rt_inflight_bytes,
                 (ULONG)xhci_ep_get_active_td_count(ep_ctx));
    }

    xhci_ring_giveback(ep_ctx->udev, ep_ctx);
}

static void xhci_ep_schedule_rt_iso_in(struct ep_context *ep_ctx)
{
    struct IOUsbHWReq *template = ep_ctx->rt_template_req;

    int inflight = xhci_ep_get_active_td_count(ep_ctx);
    while (inflight < RT_ISO_IN_TARGET_TDS)
    {
        ULONG frame = ep_ctx->rt_next_frame;
        struct IOUsbHWReq *rt_io = AllocVecPooled(ep_ctx->memoryPool, sizeof(struct IOUsbHWReq));
        if (!rt_io)
        {
            Kprintf("Failed to alloc RT ISO IO req\n");
            break;
        }
        CopyMem(template, rt_io, sizeof(struct IOUsbHWReq));

        rt_io->iouh_Data = AllocVecPooled(ep_ctx->memoryPool, template->iouh_MaxPktSize);
        if (!rt_io->iouh_Data)
        {
            Kprintf("Failed to alloc RT ISO staging buffer\n");
            FreeVecPooled(ep_ctx->memoryPool, rt_io);
            break;
        }
        rt_io->iouh_Length = template->iouh_MaxPktSize;
        rt_io->iouh_Frame = (UWORD)frame;

        KprintfH("RT ISO IN sched frame=%lu maxpkt=%lu inflight_bytes=%lu inflight_tds=%lu\n",
                 (ULONG)frame,
                 (ULONG)rt_io->iouh_Length,
                 (ULONG)ep_ctx->rt_inflight_bytes,
                 (ULONG)xhci_ep_get_active_td_count(ep_ctx));

        int ret = xhci_ring_enqueue_td(ep_ctx->udev, rt_io, 0, TRUE /* RT ISO defers doorbell to per-run giveback */);
        if (ret != UHIOERR_NO_ERROR)
        {
            FreeVecPooled(ep_ctx->memoryPool, rt_io->iouh_Data);
            FreeVecPooled(ep_ctx->memoryPool, rt_io);
            Kprintf("RT ISO submit failed %ld\n", (LONG)ret);
            break;
        }
        ++inflight;

        ep_ctx->rt_next_frame = (frame + 1) & 0xffff;
        ep_ctx->rt_inflight_bytes += rt_io->iouh_Length;
        KprintfH("RT ISO IN queued frame=%lu len=%lu inflight_bytes=%lu inflight_tds=%lu\n",
                 (ULONG)frame,
                 (ULONG)rt_io->iouh_Length,
                 (ULONG)ep_ctx->rt_inflight_bytes,
                 (ULONG)xhci_ep_get_active_td_count(ep_ctx));
    }

    xhci_ring_giveback(ep_ctx->udev, ep_ctx);
}

static void xhci_ep_notify_rt_iso_stopped(struct ep_context *ep_ctx)
{
    if (!ep_ctx)
        return;

    struct IOUsbHWReq *stop_req = ep_ctx->rt_stop_pending;

    if (!stop_req)
        return;

    ep_ctx->rt_stop_pending = NULL;

    stop_req->iouh_Req.io_Error = UHIOERR_NO_ERROR;
    ReplyMsg((struct Message *)stop_req);
}

void xhci_ep_schedule_rt_iso(struct ep_context *ep_ctx)
{
    if (ep_ctx->state != USB_DEV_EP_STATE_RT_ISO_RUNNING)
    {
        if (xhci_td_is_empty(ep_ctx->active_tds))
        {
            ep_ctx->state = USB_DEV_EP_STATE_RT_ISO_STOPPED;
            xhci_ep_notify_rt_iso_stopped(ep_ctx);
        }
        return;
    }

    /* handle deferred queue - pending */
    xhci_ep_schedule_next(ep_ctx);

    struct IOUsbHWReq *template = ep_ctx->rt_template_req;
    if (!template)
    {
        Kprintf("No RT ISO template request\n");
        xhci_ep_set_failed(ep_ctx);
        return;
    }

    if (template->iouh_Dir == UHDIR_IN)
        xhci_ep_schedule_rt_iso_in(ep_ctx);
    else
        xhci_ep_schedule_rt_iso_out(ep_ctx);
}

BYTE xhci_ep_rt_iso_start(struct ep_context *ep_ctx)
{
    if (ep_ctx->state != USB_DEV_EP_STATE_RT_ISO_STOPPED)
    {
        Kprintf("EP not in RT_ISO_STOPPED\n");
        return UHIOERR_HOSTERROR;
    }

    /* microframe_index is in 125us units; for FS frames use the frame number (divide by 8). */
    ep_ctx->rt_next_frame = (readl(ep_ctx->udev->controller->run_regs->microframe_index) >> 3) & 0xffff;
    ep_ctx->state = USB_DEV_EP_STATE_RT_ISO_RUNNING;
    xhci_ep_schedule_rt_iso(ep_ctx);
    return UHIOERR_NO_ERROR;
}

BYTE xhci_ep_rt_iso_stop(struct ep_context *ep_ctx, struct IOUsbHWReq *req)
{
    if (ep_ctx->state != USB_DEV_EP_STATE_RT_ISO_RUNNING)
    {
        Kprintf("EP not in RT_ISO_RUNNING\n");
        return UHIOERR_HOSTERROR;
    }

    if (ep_ctx->rt_req != req->iouh_Data)
    {
        Kprintf("bad params\n");
        return UHIOERR_BADPARAMS;
    }

    if (ep_ctx->rt_stop_pending)
    {
        Kprintf("STOPRTISO already pending\n");
        return UHIOERR_HOSTERROR;
    }

    ep_ctx->rt_stop_pending = req;

    if (xhci_td_is_empty(ep_ctx->active_tds))
    {
        ep_ctx->state = USB_DEV_EP_STATE_RT_ISO_STOPPED;
        xhci_ep_notify_rt_iso_stopped(ep_ctx);
    }
    else
    {
        KprintfH("RT ISO stopping addr=%ld ep=%ld inflight_tds=%lu inflight_bytes=%lu\n",
                 (LONG)req->iouh_DevAddr,
                 (LONG)ep_ctx->ep_index,
                 (ULONG)xhci_ep_get_active_td_count(ep_ctx),
                 (ULONG)ep_ctx->rt_inflight_bytes);
        ep_ctx->state = USB_DEV_EP_STATE_RT_ISO_STOPPING;
    }
    return UHIOERR_NO_ERROR;
}