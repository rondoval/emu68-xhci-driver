/* SPDX-License-Identifier: GPL-2.0-only */

#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#include <clib/utility_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME (*(struct ExecBase **)4UL)
#include <proto/exec.h>
#define UTILITY_BASE_NAME ep_ctx->udev->controller->utilityBase
#include <proto/utility.h>
#endif

#include <exec/errors.h>

#include <memory.h>
#include <debug.h>
#include <config.h>
#include <minlist.h>

#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-td.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci.h>
#include <xhci/xhci-ring.h>
#include <xhci/xhci-context.h>

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
    u8 ep_index;             /* Endpoint context index (0-30) */
    enum ep_state state;     /* Current endpoint state */
    u32 max_packet_size;     /* Cached max packet size for this endpoint */
    u8 max_burst;            /* bMaxBurst (zero-based: 0 = 1 packet/burst) */

    IOReqList pending_reqs;             /* list of pending requests */
    TransferDescriptorList *active_tds; /* list of in-flight TDs */

    struct xhci_ring *ring; /* ring for this endpoint */

    IOReqList stop_abort_reqs;
    BOOL stop_process_timeouts;

    /* Driver-initiated STALL recovery already sent CLEAR_FEATURE(HALT) to the
     * device; the next stack-issued clear-halt is a duplicate and is answered
     * without a wire request (consumed by xhci_ep_consume_halt_synced). */
    BOOL halt_cleared_internally;

    /* xHCI EP Context Interval decoded to microframes-per-ESIT; set for every
     * endpoint at context creation (needed before RT hooks register). */
    u16 rt_uframes_per_esit;

    /* RT ISO streaming state - allocated when isochronous hooks are
     * registered; non-iso endpoints don't carry it. */
    struct rt_iso_state *rt;
};

/* Per-endpoint RT ISO streaming state (CMD_REGISTER_ISOCHRONOUS_HOOKS .. 
 * CMD_UNREGISTER_ISOCHRONOUS_HOOKS lifetime). */
struct rt_iso_state
{
    struct USBRealtimeHooks *hooks;     /* class-provided request/done hooks */
    struct USBIORequest *stop_pending;  /* STOPRTISO to reply once the pipe fully stopped */

    u16 direction; /* DIRECTION_IN / DIRECTION_OUT of the streaming endpoint */

    /* Frame tracking.  Microframe-resolution to satisfy ESIT rules:
     *  - ESIT >= 1ms: Frame ID begins on ESIT boundary (xHCI 4.11.2.5).
     *  - ESIT  < 1ms: all TDs in the same frame share a Frame ID.
     * next_uframe is the microframe offset for the next TD, mod 16384
     * (Frame ID is bits 13:3, an 11-bit field); advance by rt_uframes_per_esit. */
    u16 next_uframe;

    /* Cache the last RT ISO buffer so hooks can omit repeating it */
    APTR last_buffer;
    u32 last_filled;

    /* Inflight accounting */
    u32 inflight_tds_target; /* target number of inflight TDs based on scheduling horizon */
    u32 inflight_bytes;

    u32 ist; /* IST decoded to microframes, cached at RT ISO start */

    /* IN staging slab: created in xhci_ep_rt_iso_start, object size =
     * max_packet_size, capacity = inflight_tds_target.  Destroyed in
     * xhci_ep_set_rt_stopped and as a safety net on endpoint teardown. */
    struct slab_cache in_staging_slab;
    BOOL in_staging_active;
};

static void xhci_ep_clear_stop_processing(struct ep_context *ep_ctx);

/* Sole writer of ep_ctx->state - keeps every transition observable in one place. */
static void xhci_ep_transition(struct ep_context *ep_ctx, enum ep_state new_state)
{
    if (ep_ctx->state != new_state)
    {
        KprintfH("EP %lu state %lu -> %lu\n", (ULONG)ep_ctx->ep_index,
                 (ULONG)ep_ctx->state, (ULONG)new_state);
    }
    ep_ctx->state = new_state;
}

static void xhci_ep_destroy_rt_staging_slab(struct ep_context *ep_ctx)
{
    if (!ep_ctx->rt || !ep_ctx->rt->in_staging_active)
        return;
    slab_cache_destroy(&ep_ctx->rt->in_staging_slab);
    ep_ctx->rt->in_staging_active = FALSE;
}

BOOL xhci_ep_create_context(struct usb_device *udev, u8 ep_index, u32 max_packet_size, u8 max_burst)
{
    struct ep_context *ep_ctx = pool_zalloc(udev->controller->metaPool, sizeof(struct ep_context));
    if (!ep_ctx)
    {
        Kprintf("Failed to allocate ep_context for EP %d\n", ep_index);
        return FALSE;
    }
    ep_ctx->udev = udev;
    ep_ctx->ep_index = ep_index;
    xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_IDLE);
    ep_ctx->max_packet_size = max_packet_size;
    ep_ctx->max_burst = max_burst;
    _NewMinList(&ep_ctx->pending_reqs);
    _NewMinList(&ep_ctx->stop_abort_reqs);
    ep_ctx->active_tds = xhci_td_create_list(udev->controller, ep_ctx);
    ep_ctx->ring = xhci_ring_alloc(udev->controller, XHCI_INITIAL_SEGMENTS_PER_RING, /*link_trbs*/ TRUE, /*is_event_ring*/ FALSE, ep_index, max_packet_size);
    if (!ep_ctx->active_tds || !ep_ctx->ring)
    {
        Kprintf("Failed to create resources for EP %d\n", ep_index);
        if (ep_ctx->active_tds)
            xhci_td_destroy_list(ep_ctx->active_tds, ERR_ALLOC_ERROR);
        if (ep_ctx->ring)
            xhci_ring_free(udev->controller, ep_ctx->ring);
        pool_free(udev->controller->metaPool, ep_ctx);
        return FALSE;
    }

    xhci_ep_clear_stop_processing(ep_ctx);

    udev->ep_context[ep_index] = ep_ctx;
    return TRUE;
}

void xhci_ep_destroy_contexts(struct usb_device *udev, s8 reply_code)
{
    for (int i = 0; i < USB_MAX_ENDPOINT_CONTEXTS; ++i)
    {
        struct ep_context *ep_ctx = udev->ep_context[i];
        if (ep_ctx)
        {
            KprintfH("tearing down addr %lu EP %lu context, state %lu\n", (ULONG)udev->virtual_address, (ULONG)i, (ULONG)ep_ctx->state);
            struct MinNode *node;
            while ((node = RemHeadMinList(&ep_ctx->pending_reqs)) != NULL)
            {
                struct USBIORequest *req = (struct USBIORequest *)node;
                xhci_udev_io_reply_failed(udev->controller, req, reply_code);
            }

            xhci_td_destroy_list(ep_ctx->active_tds, reply_code);

            if (ep_ctx->rt)
            {
                if (ep_ctx->rt->stop_pending)
                    xhci_udev_io_reply_failed(udev->controller, ep_ctx->rt->stop_pending, reply_code);
                xhci_ep_destroy_rt_staging_slab(ep_ctx);
                pool_free(ep_ctx->udev->controller->metaPool, ep_ctx->rt);
                ep_ctx->rt = NULL;
            }

            xhci_ep_clear_stop_processing(ep_ctx);
            xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_IDLE);

            if (ep_ctx->ring)
                xhci_ring_free(udev->controller, ep_ctx->ring);

            pool_free(ep_ctx->udev->controller->metaPool, ep_ctx);
            udev->ep_context[i] = NULL;
        }
    }
}

struct ep_context *xhci_ep_get_context_for_index(struct usb_device *udev, u8 ep_index)
{
    if (ep_index >= USB_MAX_ENDPOINT_CONTEXTS)
    {
        Kprintf("Invalid endpoint index %lu\n", (ULONG)ep_index);
        return NULL;
    }

    return udev->ep_context[ep_index];
}

void xhci_ep_set_max_packet_size(struct ep_context *ep_ctx, u32 max_packet_size)
{
    if (!ep_ctx || !ep_ctx->ring)
        return;

    xhci_ring_set_max_packet_size(ep_ctx->ring, max_packet_size);
    ep_ctx->max_packet_size = max_packet_size;
}

u32 xhci_ep_get_max_packet_size(struct ep_context *ep_ctx)
{
    return ep_ctx->max_packet_size;
}

u8 xhci_ep_get_max_burst(struct ep_context *ep_ctx)
{
    return ep_ctx->max_burst;
}

/*
 * endpoint - endpoint number (0-15)
 * ep_index - endpoint context index (0-30)
 * DCI - context index (1-31)
 * endpoint = DCI >> 1
 * DCI = ep_index + 1
 */

/* The endpoint is unusable until a recovery cycle (Set TR Deq -> set_idle) or
 * reconfiguration: fail everything in flight *and* queued so nothing
 * accumulates silently - new submissions are rejected by enqueue_td_internal
 * while in this state. */
void xhci_ep_set_failed(struct ep_context *ep_ctx)
{
    Kprintf("EP %lu state %lu -> FAILED\n", (ULONG)ep_ctx->ep_index, (ULONG)ep_ctx->state);
    xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_FAILED);
    xhci_ep_clear_stop_processing(ep_ctx);
    xhci_td_fail_all(ep_ctx->active_tds, ERR_HCI_ERROR);
    xhci_ep_flush(ep_ctx, ERR_HCI_ERROR);
}

void xhci_ep_enqueue(struct ep_context *ep_ctx, struct USBIORequest *io)
{
    if (io->driver_private_flags & REQ_ENQUEUED)
        AddHeadMinList(&ep_ctx->pending_reqs, (struct MinNode *)io);
    else
    {
        io->driver_private_flags |= REQ_ENQUEUED;
        AddTailMinList(&ep_ctx->pending_reqs, (struct MinNode *)io);
    }

    KprintfH("Ring busy, queued request cmd=%lu ep=%lu\n",
             (ULONG)io->req.io_Command,
             (ULONG)(io->endpoint & 0x0F));
}

static void xhci_ep_schedule_next(struct ep_context *ep_ctx)
{
    struct MinNode *node;
    while ((node = RemHeadMinList(&ep_ctx->pending_reqs)))
    {
        struct USBIORequest *req = (struct USBIORequest *)node;

        KprintfH("starting queued request cmd=%lu ep=%lu\n",
                 (ULONG)req->req.io_Command,
                 (ULONG)(req->endpoint & 0x0F));

        s8 err;
        if (req->driver_private_flags & REQ_INTERNAL)
            err = xhci_udev_send_ctrl(ep_ctx->udev, req);
        else
            err = xhci_udev_send(req);
        if (err != ERR_NO_ERROR)
        {
            req->req.io_Error = err;
            xhci_udev_io_reply_failed(ep_ctx->udev->controller, req, err);
            continue;
        }

        /* If the request was deferred (ring full or ep busy), stop and wait for a
         * completion to free ring space. xhci_ep_set_idle() will re-enter here. */
        if ((struct MinNode *)req == ep_ctx->pending_reqs.mlh_Head)
            break;

        /* If the endpoint went idle immediately (e.g. zero-length or error), keep draining */
        if (xhci_ep_get_state(ep_ctx) == USB_DEV_EP_STATE_FAILED)
            return;
    }
}

void xhci_ep_set_idle(struct ep_context *ep_ctx)
{
    if (xhci_td_is_empty(ep_ctx->active_tds))
        xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_IDLE);
    else if (ep_ctx->state == USB_DEV_EP_STATE_ABORTING)
        xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RECEIVING);

    if (ep_ctx->pending_reqs.mlh_Head != (struct MinNode *)&ep_ctx->pending_reqs.mlh_Tail)
        xhci_ep_schedule_next(ep_ctx);
}

void xhci_ep_set_receiving(struct ep_context *ep_ctx, struct USBIORequest *req, dma_addr_t *trb_addrs, u32 timeout_ms, u32 trb_count)
{
    if (!trb_addrs || trb_count == 0)
    {
        Kprintf("Invalid TRB list for EP %lu\n", (ULONG)ep_ctx->ep_index);
        xhci_ep_set_failed(ep_ctx);
        return;
    }

    BOOL result = xhci_td_add(ep_ctx->active_tds,
                              req,
                              timeout_ms,
                              trb_addrs,
                              trb_count);
    if (!result)
    {
        Kprintf("Failed to add TD to active list\n");
        pool_free(ep_ctx->udev->controller->metaPool, trb_addrs);
        xhci_ep_set_failed(ep_ctx);
        return;
    }

    xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RECEIVING);
}

/* RT ISO submission: the TD owns the mapped span; the endpoint stays in
 * RT_ISO_RUNNING. */
BOOL xhci_ep_set_receiving_rt(struct ep_context *ep_ctx, const struct xhci_dma_span *span,
                              u16 frame, u16 dir, BOOL staging, dma_addr_t *trb_addrs, u32 trb_count)
{
    if (!xhci_td_add_rt(ep_ctx->active_tds, span, frame, dir, staging, trb_addrs, trb_count))
    {
        Kprintf("Failed to add RT TD to active list\n");
        struct xhci_ctrl *ctrl = ep_ctx->udev->controller;
        if (trb_count <= XHCI_TD_SMALL_TRBS)
            slab_free(&ctrl->trb_addr_slab, trb_addrs);
        else
            pool_free(ctrl->metaPool, trb_addrs);
        xhci_ep_set_failed(ep_ctx);
        return FALSE;
    }

    xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RT_ISO_RUNNING);
    return TRUE;
}

void xhci_ep_set_resetting(struct ep_context *ep_ctx)
{
    xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RESETTING);

    /* Fail and free any in-flight TDs so callers get a reply before reset.
     * These are collateral of the recovery, not timeouts - ERR_TIMEOUT here
     * would feed Poseidon's dead-device counter +3 per request. */
    xhci_td_fail_all(ep_ctx->active_tds, IOERR_ABORTED);
}

void xhci_ep_set_aborting(struct ep_context *ep_ctx)
{
    xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_ABORTING);
}

static BOOL xhci_ep_has_stop_abort_requests(struct ep_context *ep_ctx)
{
    return ep_ctx->stop_abort_reqs.mlh_Head != (struct MinNode *)&ep_ctx->stop_abort_reqs.mlh_Tail;
}

static BOOL xhci_ep_append_stop_abort_request(struct ep_context *ep_ctx, struct USBIORequest *abort_req)
{
    IOReqNode *node = pool_alloc(ep_ctx->udev->controller->metaPool, sizeof(*node));
    if (!node)
        return FALSE;

    node->req = abort_req;
    AddTailMinList(&ep_ctx->stop_abort_reqs, (struct MinNode *)node);
    return TRUE;
}

static void xhci_ep_clear_stop_processing(struct ep_context *ep_ctx)
{
    struct MinNode *node;
    while ((node = RemHeadMinList(&ep_ctx->stop_abort_reqs)) != NULL)
        pool_free(ep_ctx->udev->controller->metaPool, node);

    ep_ctx->stop_process_timeouts = FALSE;
}

static void xhci_ep_prepare_stop_processing(struct ep_context *ep_ctx, struct USBIORequest *abort_req, BOOL process_timeouts)
{
    enum ep_state state = xhci_ep_get_state(ep_ctx);
    if (state != USB_DEV_EP_STATE_RECEIVING &&
        state != USB_DEV_EP_STATE_ABORTING)
        return;

    if (abort_req)
    {
        if (abort_req->req.io_Command == CMD_REQUEST_ISOCHRONOUS ||
            abort_req->req.io_Command == CMD_REGISTER_ISOCHRONOUS_HOOKS)
            return;

        if (!xhci_td_has_request(ep_ctx->active_tds, abort_req))
            return;

        if (!xhci_ep_append_stop_abort_request(ep_ctx, abort_req))
            return;
    }

    ep_ctx->stop_process_timeouts |= process_timeouts;

    if (ep_ctx->state != USB_DEV_EP_STATE_ABORTING)
    {
        xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_ABORTING);
        xhci_stop_ring(ep_ctx->udev, ep_ctx->ep_index);
    }
}

void xhci_ep_request_abort(struct ep_context *ep_ctx, struct USBIORequest *abort_req)
{
    if (!ep_ctx || !abort_req)
        return;

    xhci_ep_prepare_stop_processing(ep_ctx, abort_req, FALSE);
}

void xhci_ep_request_timeout_recovery(struct ep_context *ep_ctx)
{
    if (!ep_ctx)
        return;

    xhci_ep_prepare_stop_processing(ep_ctx, NULL, TRUE);
}

void xhci_ep_request_stop(struct ep_context *ep_ctx)
{
    if (!ep_ctx)
        return;

    enum ep_state state = xhci_ep_get_state(ep_ctx);
    if (state == USB_DEV_EP_STATE_ABORTING)
        return;

    if (state != USB_DEV_EP_STATE_RECEIVING &&
        state != USB_DEV_EP_STATE_RT_ISO_RUNNING)
        return;

    KprintfH("EP %lu state %lu -> ABORTING (stop requested)\n", (ULONG)ep_ctx->ep_index, (ULONG)state);
    xhci_ep_set_aborting(ep_ctx);
    xhci_stop_ring(ep_ctx->udev, ep_ctx->ep_index);
}

/* Stop the endpoint's ring ahead of a port suspend (U3).  Unlike abort/reset
 * flows the queued TDs stay on the ring untouched; xhci_ep_resume() restarts
 * them after the port returns to U0.  Returns TRUE if a Stop Endpoint command
 * was issued (the caller counts outstanding completions).  Endpoints already
 * in a recovery or RT ISO state are left alone. */
BOOL xhci_ep_request_suspend(struct ep_context *ep_ctx)
{
    if (!ep_ctx)
        return FALSE;

    switch (ep_ctx->state)
    {
    case USB_DEV_EP_STATE_IDLE:
    case USB_DEV_EP_STATE_RECEIVING:
        xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_SUSPENDED);
        xhci_stop_ring(ep_ctx->udev, ep_ctx->ep_index);
        return TRUE;
    default:
        return FALSE;
    }
}

/* Restart an endpoint after port resume: restore the pre-suspend state, kick
 * the doorbell if TDs are still queued, and drain any requests deferred while
 * suspended. */
void xhci_ep_resume(struct ep_context *ep_ctx)
{
    if (!ep_ctx || ep_ctx->state != USB_DEV_EP_STATE_SUSPENDED)
        return;

    if (xhci_td_is_empty(ep_ctx->active_tds))
        xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_IDLE);
    else
    {
        xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RECEIVING);
        xhci_ring_kick_ep(ep_ctx->udev, ep_ctx->ep_index);
    }

    xhci_ep_schedule_next(ep_ctx);
}

void xhci_ep_process_stop(struct ep_context *ep_ctx, dma_addr_t *deq_ptr)
{
    if (!ep_ctx || !deq_ptr)
        return;

    *deq_ptr = 0;

    if (!xhci_ep_has_stop_abort_requests(ep_ctx) && !ep_ctx->stop_process_timeouts)
        return;

    dma_addr_t stopped_deq_ptr = (dma_addr_t)xhci_get_endpoint_deq_ptr(ep_ctx->udev, ep_ctx->ep_index);

    xhci_td_patch_recovery(ep_ctx->active_tds,
                           ep_ctx->ring,
                           &ep_ctx->stop_abort_reqs,
                           stopped_deq_ptr,
                           deq_ptr);

    xhci_ep_clear_stop_processing(ep_ctx);
}

enum ep_state xhci_ep_get_state(struct ep_context *ep_ctx)
{
    return ep_ctx->state;
}

u8 xhci_ep_get_ep_index(struct ep_context *ep_ctx)
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

BOOL xhci_ep_complete_by_trb(struct ep_context *ep_ctx, dma_addr_t trb_addr,
                             u32 residue, BOOL short_packet,
                             struct xhci_td_completion *out, BOOL *deferred)
{
    *deferred = FALSE;

    if (!ep_ctx->active_tds)
        return FALSE;

    return xhci_td_complete_by_trb(ep_ctx->active_tds, trb_addr, residue, short_packet,
                                   out, deferred);
}

u32 xhci_ep_get_active_trb_count(struct ep_context *ep_ctx)
{
    return xhci_td_get_queued_trb_count(ep_ctx->active_tds);
}

inline static u32 xhci_ep_get_active_td_count(struct ep_context *ep_ctx)
{
    return xhci_td_get_queued_td_count(ep_ctx->active_tds);
}

void xhci_ep_flush(struct ep_context *ep_ctx, s8 reply_code)
{
    struct MinNode *node;
    while ((node = RemHeadMinList(&ep_ctx->pending_reqs)) != NULL)
    {
        struct USBIORequest *req = (struct USBIORequest *)node;
        xhci_udev_io_reply_failed(ep_ctx->udev->controller, req, reply_code);
    }
}

void xhci_ep_mark_halt_synced(struct ep_context *ep_ctx)
{
    if (ep_ctx)
        ep_ctx->halt_cleared_internally = TRUE;
}

BOOL xhci_ep_consume_halt_synced(struct ep_context *ep_ctx)
{
    if (!ep_ctx || !ep_ctx->halt_cleared_internally)
        return FALSE;
    ep_ctx->halt_cleared_internally = FALSE;
    return TRUE;
}

/*
 * RT ISO functions
 */
void xhci_ep_set_rt_interval(struct ep_context *ep_ctx, u8 interval)
{
    /* xHCI EP Context Interval is log2 of microframes-per-ESIT. */
    ep_ctx->rt_uframes_per_esit = (u16)(1U << interval);
}

#define RT_ISO_SCHED_OFFSET_UFRAMES 32U
#define RT_ISO_FRAME_MASK 0x7ffU /* Frame ID modulus: 2048 frames */
#define RT_ISO_UF_MASK 0x3fffU   /* microframe modulus: 2048 frames * 8 = 16384 */
#define RT_UFRAME_TO_FRAME(uf) (((uf) >> 3) & RT_ISO_FRAME_MASK)

/* Earliest microframe HW can accept, rounded UP to the next ESIT boundary. */
static u16 xhci_rt_iso_min_uf(struct ep_context *ep_ctx)
{
    u32 mfindex = mmio_read32(&ep_ctx->udev->controller->run_regs->microframe_index);
    u32 raw = mfindex + ep_ctx->rt->ist + RT_ISO_SCHED_OFFSET_UFRAMES;
    u32 ivf_mask = (u32)ep_ctx->rt_uframes_per_esit - 1U;
    return (u16)(((raw + ivf_mask) & ~ivf_mask) & RT_ISO_UF_MASK);
}

/* If rt_next_uframe has fallen behind the HW window, advance it by whole
 * ESITs to the next valid slot >= min. Updates rt_next_uframe in place
 * so the caller's post-submit advance keeps the stream contiguous.
 * Returns the 11-bit Frame ID for the resulting microframe. */
static u16 xhci_rt_iso_clamp_frame(struct ep_context *ep_ctx)
{
    u16 uf = ep_ctx->rt->next_uframe;
    u16 min_uf = xhci_rt_iso_min_uf(ep_ctx);
    /* 14-bit signed delta to handle wraparound. */
    u16 raw_delta = (u16)(min_uf - uf) & RT_ISO_UF_MASK;
    s32 delta = (raw_delta >= 0x2000U) ? ((s32)raw_delta - 0x4000) : (s32)raw_delta;

    if (delta > 0)
    {
        u32 ivf_mask = (u32)ep_ctx->rt_uframes_per_esit - 1U;
        u32 advance = ((u32)delta + ivf_mask) & ~ivf_mask;
        uf = (u16)((uf + advance) & RT_ISO_UF_MASK);
        ep_ctx->rt->next_uframe = uf;
    }
    return RT_UFRAME_TO_FRAME(uf);
}

inline static void xhci_ep_rt_iso_update_counters(struct ep_context *ep_ctx, u32 length)
{
    /* Maintain RT ISO inflight counters. */
    if (ep_ctx->rt->inflight_bytes >= length)
        ep_ctx->rt->inflight_bytes -= length;
}

inline static void xhci_ep_rt_iso_zero_counters(struct ep_context *ep_ctx)
{
    ep_ctx->rt->inflight_bytes = 0;
}

/* Free a staging buffer that was allocated from this endpoint's iso_in_staging_slab.
 * Falls back to pool_free if the slab is no longer active (edge-case teardown guard). */
void xhci_ep_free_rt_iso_buffer(struct ep_context *ep_ctx, APTR data_buffer)
{
    if (!data_buffer)
        return;
    if (ep_ctx && ep_ctx->rt && ep_ctx->rt->in_staging_active)
        slab_free(&ep_ctx->rt->in_staging_slab, data_buffer);
}

static void xhci_ep_set_rt_stopped(struct ep_context *ep_ctx)
{
    xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RT_ISO_STOPPED);
    if (ep_ctx->rt)
    {
        ep_ctx->rt->stop_pending = NULL;
        ep_ctx->rt->last_buffer = NULL;
        ep_ctx->rt->last_filled = 0;
        xhci_ep_rt_iso_zero_counters(ep_ctx);
    }
    xhci_ep_destroy_rt_staging_slab(ep_ctx);
}

s8 xhci_ep_rt_iso_add_handler(struct ep_context *ep_ctx, struct USBIORequest *req)
{
    if (!req || !ep_ctx)
        return FALSE;

    if (ep_ctx->state != USB_DEV_EP_STATE_IDLE)
    {
        /* can only enable RT ISO if EP is idle */
        Kprintf("EP not idle. Current state: %lu\n", (ULONG)ep_ctx->state);
        return ERR_HCI_ERROR;
    }

    if (!ep_ctx->rt)
    {
        ep_ctx->rt = pool_zalloc(ep_ctx->udev->controller->metaPool, sizeof(struct rt_iso_state));
        if (!ep_ctx->rt)
        {
            Kprintf("Failed to allocate RT ISO state\n");
            return ERR_ALLOC_ERROR;
        }
    }

    xhci_ep_set_rt_stopped(ep_ctx);

    ep_ctx->rt->hooks = (struct USBRealtimeHooks *)req->data_buffer;
    ep_ctx->rt->direction = req->direction;

#ifdef DEBUG_HIGH
    struct USBRealtimeHooks *rt = (struct USBRealtimeHooks *)req->data_buffer;
    if (req->direction == DIRECTION_IN)
    {
        KprintfH("Added ISO handler: EP %lu in req hook: %lx, in done hook: %lx, prefetch: %lu\n", (ULONG)ep_ctx->ep_index, (ULONG)rt->input_request_hook, (ULONG)rt->input_done_hook, (ULONG)rt->max_output_prefetch);
    }
    else
    {
        KprintfH("Added ISO handler: EP %lu out req hook: %lx, out done hook: %lx, prefetch: %lu\n", (ULONG)ep_ctx->ep_index, (ULONG)rt->output_request_hook, (ULONG)rt->output_done_hook, (ULONG)rt->max_output_prefetch);
    }
#endif
    return ERR_NO_ERROR;
}

s8 xhci_ep_rt_iso_rem_handler(struct ep_context *ep_ctx, struct USBIORequest *req)
{
    if (!req || !ep_ctx)
    {
        Kprintf("Invalid parameters to remove RT ISO handler\n");
        return ERR_BAD_PARAMETERS;
    }

    if (ep_ctx->state != USB_DEV_EP_STATE_RT_ISO_STOPPED)
    {
        Kprintf("EP not in RT_ISO_STOPPED/IDLE state. Current state: %lu\n", (ULONG)ep_ctx->state);
        return ERR_HCI_ERROR;
    }

    if (!ep_ctx->rt || req->data_buffer != ep_ctx->rt->hooks)
    {
        Kprintf("Mismatched RT ISO handler removal request\n");
        return ERR_BAD_PARAMETERS;
    }

    xhci_ep_destroy_rt_staging_slab(ep_ctx);
    pool_free(ep_ctx->udev->controller->metaPool, ep_ctx->rt);
    ep_ctx->rt = NULL;
    xhci_ep_set_idle(ep_ctx);
    KprintfH("Successfully removed ISO handler (state reset to IDLE)\n");
    return ERR_NO_ERROR;
}

void xhci_ep_rt_iso_in(struct ep_context *ep_ctx, APTR buffer, u32 length, u32 act_len, u16 rt_frame)
{
    /* Pull a destination buffer from the class, copy staged DMA into it, then signal completion. */
    struct USBBufferRequest rt_buffer_req;
    rt_buffer_req.frame = rt_frame;
    rt_buffer_req.flags = 0;
    rt_buffer_req.length = act_len;
    rt_buffer_req.data = NULL;

    CallHookPkt(ep_ctx->rt->hooks->input_request_hook, ep_ctx->rt->hooks, &rt_buffer_req);

    if (rt_buffer_req.data)
    {
        u32 copy_len = act_len < rt_buffer_req.length ? act_len : rt_buffer_req.length;
        CopyMem(buffer, rt_buffer_req.data, copy_len);
        rt_buffer_req.length = copy_len;

        CallHookPkt(ep_ctx->rt->hooks->input_done_hook, ep_ctx->rt->hooks, &rt_buffer_req);
    }

    xhci_ep_rt_iso_update_counters(ep_ctx, length);
}

void xhci_ep_rt_iso_out(struct ep_context *ep_ctx, APTR buffer, u32 length, u32 act_len, u16 rt_frame)
{
    (void)buffer;

    if (ep_ctx->rt->hooks->output_done_hook)
    {
        /* OUT path: completion hook only. OutReqHook is before transfer. */
        struct USBBufferRequest rt_buffer_req;
        rt_buffer_req.data = buffer;
        rt_buffer_req.frame = rt_frame;
        rt_buffer_req.length = act_len;
        rt_buffer_req.flags = 0;
        CallHookPkt(ep_ctx->rt->hooks->output_done_hook, ep_ctx->rt->hooks, &rt_buffer_req);
    }

    xhci_ep_rt_iso_update_counters(ep_ctx, length);
}

static void xhci_ep_schedule_rt_iso_out(struct ep_context *ep_ctx)
{
    u32 prefetch_bytes = ep_ctx->rt->hooks->max_output_prefetch;
    struct xhci_ctrl *ctrl = ep_ctx->udev->controller;

    while (ep_ctx->rt->inflight_bytes < prefetch_bytes)
    {
        /* Don't enqueue if the ring can't fit one more TRB+spare; try to grow first */
        if (!xhci_ring_has_room(ep_ctx, 2))
        {
            if (!xhci_ring_grow(ctrl, xhci_ep_get_ring(ep_ctx), XHCI_SEGMENTS_PER_RING) ||
                !xhci_ring_has_room(ep_ctx, 2))
                break;
        }

        /* CFC controllers pin Frame ID to a slot; without CFC the HW chooses
         * via SIA, but we still hand the user hook a monotonic frame counter. */
        u16 frame = ctrl->cfc_supported
                        ? xhci_rt_iso_clamp_frame(ep_ctx)
                        : RT_UFRAME_TO_FRAME(ep_ctx->rt->next_uframe);

        struct USBBufferRequest rt_buffer_req;
        rt_buffer_req.length = prefetch_bytes;
        rt_buffer_req.data = ep_ctx->rt->last_buffer;
        rt_buffer_req.flags = 0;
        rt_buffer_req.frame = frame; /* monotonic frame counter to avoid jumps */

        KprintfH("RT ISO OUT sched frame=%lu len=%lu inflight_bytes=%lu inflight_tds=%lu\n",
                 (ULONG)frame,
                 (ULONG)rt_buffer_req.length,
                 (ULONG)ep_ctx->rt->inflight_bytes,
                 (ULONG)xhci_ep_get_active_td_count(ep_ctx));

        CallHookPkt(ep_ctx->rt->hooks->output_request_hook, ep_ctx->rt->hooks, &rt_buffer_req);

        if (!rt_buffer_req.data || rt_buffer_req.length == 0)
        {
            KprintfH("RT ISO hook provided no buffer/length\n");
            break;
        }

        u32 offset = 0;
        if (rt_buffer_req.data == ep_ctx->rt->last_buffer)
        {
            offset = ep_ctx->rt->last_filled;
        }
        else
        {
            ep_ctx->rt->last_buffer = rt_buffer_req.data;
        }
        ep_ctx->rt->last_filled = offset + rt_buffer_req.length;

        u32 length = rt_buffer_req.length;
        s8 ret = xhci_ring_enqueue_rt_td(ep_ctx->udev, ep_ctx->ep_index,
                                         (APTR)((u8 *)rt_buffer_req.data + offset), length,
                                         frame, DIRECTION_OUT, FALSE,
                                         TRUE /* RT ISO defers doorbell to per-run giveback */);
        if (ret != ERR_NO_ERROR)
        {
            Kprintf("RT ISO submit failed %ld\n", (LONG)ret);
            break;
        }

        ep_ctx->rt->next_uframe = (u16)(((u32)ep_ctx->rt->next_uframe + ep_ctx->rt_uframes_per_esit) & RT_ISO_UF_MASK);
        ep_ctx->rt->inflight_bytes += length;
        KprintfH("RT ISO OUT queued frame=%lu len=%lu inflight_bytes=%lu inflight_tds=%lu\n",
                 (ULONG)frame,
                 (ULONG)length,
                 (ULONG)ep_ctx->rt->inflight_bytes,
                 (ULONG)xhci_ep_get_active_td_count(ep_ctx));
    }

    xhci_ring_giveback(ep_ctx->udev, ep_ctx);
}

static void xhci_ep_schedule_rt_iso_in(struct ep_context *ep_ctx)
{
    struct xhci_ctrl *ctrl = ep_ctx->udev->controller;

    u32 inflight = xhci_ep_get_active_td_count(ep_ctx);
    while (inflight < ep_ctx->rt->inflight_tds_target)
    {
        /* Same backpressure rule as the OUT path: bail before alloc if no room. */
        if (!xhci_ring_has_room(ep_ctx, 2))
        {
            if (!xhci_ring_grow(ctrl, xhci_ep_get_ring(ep_ctx), XHCI_SEGMENTS_PER_RING) ||
                !xhci_ring_has_room(ep_ctx, 2))
                break;
        }

        u16 frame = ctrl->cfc_supported
                        ? xhci_rt_iso_clamp_frame(ep_ctx)
                        : RT_UFRAME_TO_FRAME(ep_ctx->rt->next_uframe);
        const u32 packet_size = ep_ctx->max_packet_size;

        APTR staging = slab_alloc(&ep_ctx->rt->in_staging_slab);
        if (!staging)
        {
            Kprintf("Failed to alloc RT ISO staging buffer\n");
            break;
        }

        KprintfH("RT ISO IN sched frame=%lu maxpkt=%lu inflight_bytes=%lu inflight_tds=%lu\n",
                 (ULONG)frame,
                 (ULONG)packet_size,
                 (ULONG)ep_ctx->rt->inflight_bytes,
                 (ULONG)xhci_ep_get_active_td_count(ep_ctx));

        s8 ret = xhci_ring_enqueue_rt_td(ep_ctx->udev, ep_ctx->ep_index,
                                         staging, packet_size, frame, DIRECTION_IN,
                                         TRUE /* staging buffer, freed on completion */,
                                         TRUE /* RT ISO defers doorbell to per-run giveback */);
        if (ret != ERR_NO_ERROR)
        {
            slab_free(&ep_ctx->rt->in_staging_slab, staging);
            Kprintf("RT ISO submit failed %ld\n", (LONG)ret);
            break;
        }
        ++inflight;

        ep_ctx->rt->next_uframe = (u16)(((u32)ep_ctx->rt->next_uframe + ep_ctx->rt_uframes_per_esit) & RT_ISO_UF_MASK);
        ep_ctx->rt->inflight_bytes += packet_size;
        KprintfH("RT ISO IN queued frame=%lu len=%lu inflight_bytes=%lu inflight_tds=%lu\n",
                 (ULONG)frame,
                 (ULONG)packet_size,
                 (ULONG)ep_ctx->rt->inflight_bytes,
                 (ULONG)xhci_ep_get_active_td_count(ep_ctx));
    }

    xhci_ring_giveback(ep_ctx->udev, ep_ctx);
}

static void xhci_ep_notify_rt_iso_stopped(struct ep_context *ep_ctx)
{
    if (!ep_ctx || !ep_ctx->rt)
        return;

    struct USBIORequest *stop_req = ep_ctx->rt->stop_pending;

    if (!stop_req)
        return;

    ep_ctx->rt->stop_pending = NULL;

    stop_req->req.io_Error = ERR_NO_ERROR;
    ReplyMsg((struct Message *)stop_req);
}

void xhci_ep_schedule_rt_iso(struct ep_context *ep_ctx)
{
    if (ep_ctx->state != USB_DEV_EP_STATE_RT_ISO_RUNNING)
    {
        if (xhci_td_is_empty(ep_ctx->active_tds))
        {
            xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RT_ISO_STOPPED);
            xhci_ep_notify_rt_iso_stopped(ep_ctx);
        }
        return;
    }

    /* handle deferred queue - pending */
    xhci_ep_schedule_next(ep_ctx);

    if (!ep_ctx->rt)
    {
        Kprintf("No RT ISO state\n");
        xhci_ep_set_failed(ep_ctx);
        return;
    }

    if (ep_ctx->rt->direction == DIRECTION_IN)
        xhci_ep_schedule_rt_iso_in(ep_ctx);
    else
        xhci_ep_schedule_rt_iso_out(ep_ctx);
}

s8 xhci_ep_rt_iso_start(struct ep_context *ep_ctx)
{
    if (ep_ctx->state != USB_DEV_EP_STATE_RT_ISO_STOPPED)
    {
        Kprintf("EP not in RT_ISO_STOPPED\n");
        return ERR_HCI_ERROR;
    }

    /* HCSPARAMS2.IST is 4 bits (xHCI 5.3.6): bit 3 selects the unit of bits[2:0]
     * — 0 = microframes (0..7), 1 = frames (0..7, i.e. 0..56 microframes).
     * Normalize to microframes once so users can add to MFINDEX directly. */
    u32 ist_raw = HCS_IST(mmio_read32(&ep_ctx->udev->controller->hccr->cr_hcsparams2));
    ep_ctx->rt->ist = (ist_raw & 0x8U) ? ((ist_raw & 0x7U) << 3) : (ist_raw & 0x7U);
    ep_ctx->rt->next_uframe = xhci_rt_iso_min_uf(ep_ctx);
    xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RT_ISO_RUNNING);

    const u32 uframes_per_td = ep_ctx->rt_uframes_per_esit ? (u32)ep_ctx->rt_uframes_per_esit : 1U;
    const u32 target_uframes = RT_ISO_IN_TARGET_FRAMES * 8U;
    ep_ctx->rt->inflight_tds_target = (target_uframes + uframes_per_td - 1U) / uframes_per_td;

    /* Build the per-endpoint IN staging slab now that we know both the packet size
     * and the target inflight depth.  OUT endpoints have no staging buffer. */
    if (ep_ctx->rt->direction == DIRECTION_IN && ep_ctx->max_packet_size > 0)
    {
        slab_cache_init(&ep_ctx->rt->in_staging_slab,
                        ep_ctx->udev->controller->metaPool,
                        ep_ctx->udev->controller->dmaPool,
                        ep_ctx->max_packet_size,
                        DMA_ALIGN_MIN,
                        ep_ctx->rt->inflight_tds_target);
        ep_ctx->rt->in_staging_active = TRUE;
    }

    KprintfH("Starting RT ISO stream: IST=%lu uframes rt_next_uframe=%lu target_ms=%lu target_uframes=%lu uframes_per_td=%lu target_tds=%lu\n",
             (ULONG)ep_ctx->rt->ist,
             (ULONG)ep_ctx->rt->next_uframe,
             (ULONG)RT_ISO_IN_TARGET_FRAMES,
             (ULONG)target_uframes,
             (ULONG)uframes_per_td,
             (ULONG)ep_ctx->rt->inflight_tds_target);

    xhci_ep_schedule_rt_iso(ep_ctx);
    return ERR_NO_ERROR;
}

s8 xhci_ep_rt_iso_stop(struct ep_context *ep_ctx, struct USBIORequest *req)
{
    if (ep_ctx->state != USB_DEV_EP_STATE_RT_ISO_RUNNING)
    {
        Kprintf("EP not in RT_ISO_RUNNING\n");
        return ERR_HCI_ERROR;
    }

    if (ep_ctx->rt->hooks != req->data_buffer)
    {
        Kprintf("bad params\n");
        return ERR_BAD_PARAMETERS;
    }

    if (ep_ctx->rt->stop_pending)
    {
        Kprintf("STOPRTISO already pending\n");
        return ERR_HCI_ERROR;
    }

    ep_ctx->rt->stop_pending = req;

    if (xhci_td_is_empty(ep_ctx->active_tds))
    {
        xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RT_ISO_STOPPED);
        xhci_ep_notify_rt_iso_stopped(ep_ctx);
    }
    else
    {
        KprintfH("RT ISO stopping addr=%lu ep=%lu inflight_tds=%lu inflight_bytes=%lu\n",
                 (ULONG)req->virtual_address,
                 (ULONG)ep_ctx->ep_index,
                 (ULONG)xhci_ep_get_active_td_count(ep_ctx),
                 (ULONG)ep_ctx->rt->inflight_bytes);
        xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RT_ISO_STOPPING);
    }
    return ERR_NO_ERROR;
}