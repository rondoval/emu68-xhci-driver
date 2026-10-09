/* SPDX-License-Identifier: GPL-2.0-only */

#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#include <clib/utility_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME SysBase /* a local in every function, from its context's sysBase */
#include <proto/exec.h>
#define UTILITY_BASE_NAME ep_ctx->udev->controller->utilityBase
#include <proto/utility.h>
#endif

#include <exec/errors.h>

#include <memory.h>
#include <debug.h>
#include <config.h>
#include <minlist.h>
#include <timing.h>

#include <xhci/xhci-endpoint.h>
#include "xhci-endpoint-priv.h"
#include <xhci/xhci-commands.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci.h>
#include <xhci/xhci-ring.h>
#include <xhci/xhci-submit.h>
#include <xhci/xhci-context.h>
#include <xhci/xhci-descriptors.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-endpoint] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef TRACE
#undef KprintfT
#define KprintfT(fmt, ...) PrintPistorm("[xhci-endpoint] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

static void ep_clear_wishes(struct ep_context *ep_ctx);
static void ep_suspend_parked(struct ep_context *ep_ctx);
static void ep_service(struct ep_context *ep_ctx);

/* Sole writer of ep_ctx->state - keeps every transition observable in one place. */
void xhci_ep_transition(struct ep_context *ep_ctx, enum ep_state new_state)
{
    if (ep_ctx->state != new_state)
    {
        KprintfT("EP %lu state %lu -> %lu\n", (ULONG)ep_ctx->ep_index,
                 (ULONG)ep_ctx->state, (ULONG)new_state);
    }
    ep_ctx->state = new_state;
}

/* Iterate the endpoint's transfer rings: slot 0..ep_ring_count()-1 is the
 * default ring, or - in stream mode - the stream rings for ids
 * 1..num_streams (an LSA endpoint's default ring carries no TDs). */
static u16 ep_ring_count(struct ep_context *ep_ctx)
{
    return ep_ctx->streams ? ep_ctx->streams->num_streams : 1;
}

static struct xhci_ring *ep_ring_by_slot(struct ep_context *ep_ctx, u16 slot)
{
    return ep_ctx->streams ? ep_ctx->streams->rings[slot + 1] : ep_ctx->ring;
}

static BOOL ep_tds_empty(struct ep_context *ep_ctx)
{
    for (u16 slot = 0; slot < ep_ring_count(ep_ctx); ++slot)
    {
        if (!xhci_td_is_empty(xhci_ring_get_td_list(ep_ring_by_slot(ep_ctx, slot))))
            return FALSE;
    }

    return TRUE;
}

static void ep_fail_all_tds(struct ep_context *ep_ctx, s8 io_Error)
{
    for (u16 slot = 0; slot < ep_ring_count(ep_ctx); ++slot)
        xhci_td_fail_all(xhci_ring_get_td_list(ep_ring_by_slot(ep_ctx, slot)), io_Error);
}

/* Retire a transfer ring together with its TD list (replies anything still
 * in flight; a list already destroyed separately leaves NULL behind). */
static void ep_ring_free_with_tds(struct xhci_ctrl *ctrl, struct xhci_ring *ring, s8 reply_code)
{
    if (!ring)
        return;

    xhci_td_destroy_list(xhci_ring_get_td_list(ring), reply_code);
    xhci_ring_free(ctrl, ring);
}

BOOL xhci_ep_create_context(struct usb_device *udev, u8 ep_index, u32 max_packet_size, u8 max_burst)
{
    struct ExecBase *SysBase = udev->sysBase;
    /* a re-add over a live context (alt-setting switch): retire the old one
     * first — anything still pending fails with device-gone semantics */
    if (udev->ep_context[ep_index])
        xhci_ep_destroy_context(udev, ep_index, UHIOERR_TIMEOUT);

    struct ep_context *ep_ctx = pool_zalloc(udev->controller->metaPool, sizeof(struct ep_context));
    if (!ep_ctx)
    {
        Kprintf("Failed to allocate ep_context for EP %ld\n", ep_index);
        return FALSE;
    }
    ep_ctx->udev = udev;
    ep_ctx->sysBase = udev->sysBase;
    ep_ctx->ep_index = ep_index;
    xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_IDLE);
    ep_ctx->max_packet_size = max_packet_size;
    ep_ctx->max_burst = max_burst;
    _NewMinList(&ep_ctx->pending_reqs);
    _NewMinList(&ep_ctx->abort_reqs);
    ep_ctx->ring = xhci_ring_alloc(udev->controller, XHCI_INITIAL_SEGMENTS_PER_RING, /*link_trbs*/ TRUE, /*is_event_ring*/ FALSE, ep_index, max_packet_size);
    if (!ep_ctx->ring || !xhci_td_create_list(udev->controller, ep_ctx, ep_ctx->ring))
    {
        Kprintf("Failed to create resources for EP %ld\n", ep_index);
        ep_ring_free_with_tds(udev->controller, ep_ctx->ring, UHIOERR_OUTOFMEMORY);
        pool_free(udev->controller->metaPool, ep_ctx);
        return FALSE;
    }

    udev->ep_context[ep_index] = ep_ctx;
    return TRUE;
}

void xhci_ep_destroy_context(struct usb_device *udev, u8 ep_index, s8 reply_code)
{
    struct ExecBase *SysBase = udev->sysBase;
    struct ep_context *ep_ctx = (ep_index < USB_MAX_ENDPOINT_CONTEXTS)
                                    ? udev->ep_context[ep_index]
                                    : NULL;
    if (!ep_ctx)
        return;

    KprintfT("tearing down slot %lu EP %lu context, state %lu\n",
             (ULONG)udev->slot_id, (ULONG)ep_index, (ULONG)ep_ctx->state);
    xhci_ep_flush(ep_ctx, reply_code);

    /* Reply every in-flight TD before the RT staging slab goes away below
     * (an RT TD's teardown returns its IN staging buffer to the slab). */
    for (u16 slot = 0; slot < ep_ring_count(ep_ctx); ++slot)
    {
        struct xhci_ring *ring = ep_ring_by_slot(ep_ctx, slot);
        xhci_td_destroy_list(xhci_ring_get_td_list(ring), reply_code);
    }

    if (ep_ctx->rt)
    {
        if (ep_ctx->rt->stop_pending)
            xhci_xfer_complete(udev, ep_ctx->rt->stop_pending, reply_code, 0);
        else
            xhci_ep_rt_iso_fire_release(ep_ctx); /* stream died without a STOP */
        xhci_ep_destroy_rt_staging_slab(ep_ctx);
        pool_free(ep_ctx->udev->controller->metaPool, ep_ctx->rt);
        ep_ctx->rt = NULL;
    }

    /* a halt recovery cut short: its transfer still carries its own result */
    if (ep_ctx->halted_req)
        xhci_xfer_complete(udev, ep_ctx->halted_req, ep_ctx->halted_req->error, ep_ctx->halted_req->actual);

    ep_clear_wishes(ep_ctx);
    xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_IDLE);

    xhci_ep_streams_destroy(ep_ctx, reply_code);
    ep_ring_free_with_tds(udev->controller, ep_ctx->ring, reply_code);

    pool_free(ep_ctx->udev->controller->metaPool, ep_ctx);
    udev->ep_context[ep_index] = NULL;
}

void xhci_ep_destroy_contexts(struct usb_device *udev, s8 reply_code)
{
    for (u8 i = 0; i < USB_MAX_ENDPOINT_CONTEXTS; ++i)
        xhci_ep_destroy_context(udev, i, reply_code);
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

void xhci_ep_set_hw_type(struct ep_context *ep_ctx, u8 hw_ep_type)
{
    ep_ctx->hw_ep_type = hw_ep_type;
}

u8 xhci_ep_get_hw_type(struct ep_context *ep_ctx)
{
    return ep_ctx->hw_ep_type;
}

/*
 * endpoint - endpoint number (0-15)
 * ep_index - endpoint context index (0-30)
 * DCI - context index (1-31)
 * endpoint = DCI >> 1
 * DCI = ep_index + 1
 */

/* The endpoint is unusable until it is configured anew: fail everything in
 * flight *and* queued so nothing accumulates silently - new submissions are
 * rejected by xhci_ep_submit() while in this state.  A recovery that was
 * under way ends here: its commands' completions find a failed endpoint and
 * do nothing. */
void xhci_ep_set_failed(struct ep_context *ep_ctx)
{
    Kprintf("EP %lu state %lu -> FAILED\n", (ULONG)ep_ctx->ep_index, (ULONG)ep_ctx->state);
    xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_FAILED);
    ep_clear_wishes(ep_ctx);
    ep_ctx->hold = 0;
    ep_ctx->cmds_pending = 0;
    ep_fail_all_tds(ep_ctx, UHIOERR_HOSTERROR);
    xhci_ep_flush(ep_ctx, UHIOERR_HOSTERROR);
    xhci_ep_rt_iso_fire_release(ep_ctx); /* a registered iso stream is dead with the endpoint */
    ep_suspend_parked(ep_ctx);           /* a port suspend must not wait for this endpoint for ever */

    /* the transfer a halt happened on keeps its own error */
    struct xhci_xfer *halted = ep_ctx->halted_req;
    ep_ctx->halted_req = NULL;
    if (halted)
        xhci_xfer_complete(ep_ctx->udev, halted, halted->error, halted->actual);
}

static void xhci_ep_enqueue(struct ep_context *ep_ctx, struct xhci_xfer *io)
{
    struct ExecBase *SysBase = ep_ctx->sysBase;
    if (io->priv_flags & REQ_ENQUEUED)
        AddHeadMinList(&ep_ctx->pending_reqs, (struct MinNode *)io);
    else
    {
        io->priv_flags |= REQ_ENQUEUED;
        AddTailMinList(&ep_ctx->pending_reqs, (struct MinNode *)io);
    }

    KprintfT("Ring busy, queued request type=%lu ep=%lu\n",
             (ULONG)io->type,
             (ULONG)(io->endpoint & 0x0F));
}

/* THE transfer submit entry (direct path, pending-drain, internal EP0): the
 * endpoint owns the policy — state gate, pending-queue deferral, stream-ring
 * selection — and hands the mechanics to xhci_submit_td().  The NAK timeout
 * rides the xfer (XHCI_XF_TIMEOUT + timeout_ms). */
s8 xhci_ep_submit(struct ep_context *ep_ctx, struct xhci_xfer *io)
{
#ifdef DEBUG
    /* every entry runs under the transfer-plane lock */
    struct ExecBase *SysBase = ep_ctx->sysBase;
    if (ep_ctx->udev->controller->xfer_lock.ss_Owner != FindTask(NULL))
        Kprintf("xfer_lock NOT HELD on submit path!\n");

    /* The transfer and the endpoint must agree on type: the ring decides how
     * the TRBs are read, so a control TD on a bulk ring is rejected by the xHC
     * with TRB Error.  Type, not index - a device may expose a control
     * endpoint at a non-zero DCI. */
    const s32 dbg_ep_type = xhci_ep_type_for_index(ep_ctx->udev, ep_ctx->ep_index);
    if (dbg_ep_type >= 0 &&
        (io->type == UHCD_EPTYPE_CONTROL) != (dbg_ep_type == USB_ENDPOINT_XFER_CONTROL))
        Kprintf("type mismatch: xfer type %lu submitted on ep %lu (type %ld)\n",
                (ULONG)io->type, (ULONG)ep_ctx->ep_index, (LONG)dbg_ep_type);
#endif
    const enum ep_state state = ep_ctx->state;

    /* A FAILED endpoint stays dead until recovery/reconfiguration - reject
     * instead of queueing into a list nothing will ever drain. */
    if (state == USB_DEV_EP_STATE_FAILED)
    {
        KprintfT("Rejecting transfer, ep %lu failed\n", (ULONG)ep_ctx->ep_index);
        io->error = UHIOERR_HOSTERROR;
        return UHIOERR_HOSTERROR;
    }

    if (state == USB_DEV_EP_STATE_RECOVERING || state == USB_DEV_EP_STATE_PARKED)
    {
        KprintfT("Cannot submit transfer, ep in state %ld\n", state);
        xhci_ep_enqueue(ep_ctx, io);
        return UHIOERR_NO_ERROR;
    }

    /* Streams: the request's stream id picks the ring (SS bulk / UAS); a
     * stream id on a single-ring endpoint rides along ignored. */
    struct xhci_ring *ep_ring = xhci_ep_get_ring_for_stream(ep_ctx, io->stream_id);
    if (!ep_ring)
    {
        Kprintf("No ring for ep %lu stream %lu\n", (ULONG)ep_ctx->ep_index, (ULONG)io->stream_id);
        io->error = UHIOERR_BADPARAMS;
        return UHIOERR_BADPARAMS;
    }

    const u32 timeout_ms = (io->flags & XHCI_XF_TIMEOUT) ? io->timeout_ms : 0;
    s8 err = UHIOERR_NO_ERROR;
    switch (xhci_submit_td(ep_ctx->udev, ep_ctx, ep_ring, io, timeout_ms, &err))
    {
    case XHCI_SUBMIT_NO_ROOM:
        /* the io keeps its mapping while queued; resubmission reuses it */
        xhci_ep_enqueue(ep_ctx, io);
        return UHIOERR_NO_ERROR;
    case XHCI_SUBMIT_FAILED:
        return err;
    default:
        return UHIOERR_NO_ERROR;
    }
}

void xhci_ep_schedule_next(struct ep_context *ep_ctx)
{
    struct ExecBase *SysBase = ep_ctx->sysBase;
    struct MinNode *node;
    while ((node = RemHeadMinList(&ep_ctx->pending_reqs)))
    {
        struct xhci_xfer *req = (struct xhci_xfer *)node;

        KprintfT("starting queued request type=%lu ep=%lu\n",
                 (ULONG)req->type,
                 (ULONG)(req->endpoint & 0x0F));

        /* Straight back through the submit entry, on this endpoint: a pending
         * queue never holds another endpoint's transfer. */
        s8 err = xhci_ep_submit(ep_ctx, req);
        if (err != UHIOERR_NO_ERROR)
        {
            req->error = err;
            xhci_xfer_complete(ep_ctx->udev, req, err, 0);
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

/* A running endpoint after a TD left its rings: IDLE when none is left, and
 * whatever waits in the pending queue gets its turn. */
void xhci_ep_set_idle(struct ep_context *ep_ctx)
{
    if (ep_tds_empty(ep_ctx))
        xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_IDLE);

    if (ep_ctx->pending_reqs.mlh_Head != (struct MinNode *)&ep_ctx->pending_reqs.mlh_Tail)
        xhci_ep_schedule_next(ep_ctx);
}

/* FALSE = the endpoint is FAILED and trb_addrs is already freed: the caller
 * must not touch the array or hand the TD to the hardware, and still owns the
 * request's disposal (same contract as the RT variant below).  The TD lands
 * on ep_ring's own list. */
BOOL xhci_ep_set_receiving(struct ep_context *ep_ctx, struct xhci_ring *ep_ring,
                           struct xhci_xfer *req, dma_addr_t *trb_addrs, u32 timeout_ms, u32 trb_count)
{
    if (!trb_addrs || trb_count == 0)
    {
        Kprintf("Invalid TRB list for EP %lu\n", (ULONG)ep_ctx->ep_index);
        xhci_ep_set_failed(ep_ctx);
        return FALSE;
    }

    BOOL result = xhci_td_add(xhci_ring_get_td_list(ep_ring),
                              req,
                              timeout_ms,
                              trb_addrs,
                              trb_count);
    if (!result)
    {
        Kprintf("Failed to add TD to active list\n");
        xhci_td_trb_addrs_free(ep_ctx->udev->controller, trb_addrs, trb_count);
        xhci_ep_set_failed(ep_ctx);
        return FALSE;
    }

    xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RECEIVING);
    return TRUE;
}

/* RT ISO submission: the TD owns the mapped span; the endpoint stays in
 * RT_ISO_RUNNING.  Default ring only — RT-ISO endpoints never have streams. */
BOOL xhci_ep_set_receiving_rt(struct ep_context *ep_ctx, const struct xhci_dma_span *span,
                              u16 frame, u16 dir, BOOL staging, dma_addr_t *trb_addrs, u32 trb_count)
{
    if (!xhci_td_add_rt(ep_default_tds(ep_ctx), span, frame, dir, staging, trb_addrs, trb_count))
    {
        Kprintf("Failed to add RT TD to active list\n");
        xhci_td_trb_addrs_free(ep_ctx->udev->controller, trb_addrs, trb_count);
        xhci_ep_set_failed(ep_ctx);
        return FALSE;
    }

    xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RT_ISO_RUNNING);
    return TRUE;
}

/*
 * Recovery
 *
 * Taking TDs off a ring the controller may be working on - an abort, a NAK
 * timeout, a flush, a halt - needs the endpoint stopped first; so does a port
 * suspend.  All of them run the same course:
 *
 *   1. note what is wanted (a wish), in whatever state the endpoint is;
 *   2. get the endpoint stopped: Stop Endpoint from RECEIVING, Reset Endpoint
 *      after a halt, nothing when it is PARKED;
 *   3. ep_service(): retire the victims, re-arm the rings that lost TDs, then
 *      run again - unless something holds the endpoint.
 *
 * While a command of steps 2 or 3 is in flight the endpoint is RECOVERING:
 * new wishes are only noted, and the completion of the last command
 * (xhci_ep_command_done) is where they are served.
 */

static BOOL ep_wishes(struct ep_context *ep_ctx)
{
    return ep_ctx->want_flush || ep_ctx->want_timeouts ||
           ep_ctx->abort_reqs.mlh_Head != (struct MinNode *)&ep_ctx->abort_reqs.mlh_Tail;
}

static void ep_clear_wishes(struct ep_context *ep_ctx)
{
    struct ExecBase *SysBase = ep_ctx->sysBase;
    struct MinNode *node;
    while ((node = RemHeadMinList(&ep_ctx->abort_reqs)) != NULL)
        pool_free(ep_ctx->udev->controller->metaPool, node);

    ep_ctx->want_timeouts = FALSE;
    ep_ctx->want_flush = FALSE;
}

/* Tell the device's suspend sequence that this endpoint is down (it counted
 * the Stop Endpoint that xhci_ep_request_suspend issued). */
static void ep_suspend_parked(struct ep_context *ep_ctx)
{
    if (!ep_ctx->suspend_notify)
        return;

    ep_ctx->suspend_notify = FALSE;
    xhci_udev_suspend_stop_done(ep_ctx->udev);
}

/* Issue one command of the recovery.  flags are the command's own TRB bits
 * (TRB_SP, TRB_TSP), 0 for most.  FALSE = it could not be queued and the
 * endpoint is FAILED. */
static BOOL ep_command(struct ep_context *ep_ctx, trb_type cmd, u32 flags, struct xhci_ring *ring, dma_addr_t deq)
{
    if (!xhci_queue_ep_command(ep_ctx->udev, ep_ctx->ep_index, cmd, flags, ring, deq))
    {
        xhci_ep_set_failed(ep_ctx);
        return FALSE;
    }

    ++ep_ctx->cmds_pending;
    xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RECOVERING);
    return TRUE;
}

/* A wish was noted: see that it gets served. */
static void ep_want_stop(struct ep_context *ep_ctx)
{
    switch (ep_ctx->state)
    {
    case USB_DEV_EP_STATE_RECEIVING:
    case USB_DEV_EP_STATE_RT_ISO_RUNNING:
        ep_command(ep_ctx, TRB_STOP_RING, 0, NULL, 0);
        break;
    case USB_DEV_EP_STATE_PARKED:
        ep_service(ep_ctx); /* the rings are stopped already */
        break;
    default:
        break; /* RECOVERING: the command in flight ends in ep_service() */
    }
}

/* The dequeue pointer the controller saved for one ring when the endpoint
 * stopped or halted - only then is it defined (xHCI 6.2.3, 6.2.4.1).  Plain
 * ring: the output endpoint context.  Stream ring: the stream context array,
 * which the controller brings up to date for every stream before the endpoint
 * leaves the Running state (xHCI 4.12) - the caller ran the invalidate over
 * that device-written array.  SCT and DCS are stripped: the DCS write-back is not
 * trusted (VL805 broken-DCS), the TD tracker takes the cycle from the TRB
 * itself. */
static dma_addr_t ep_ring_stopped_deq(struct ep_context *ep_ctx, u16 slot)
{
    if (!ep_ctx->streams)
        return (dma_addr_t)xhci_get_endpoint_deq_ptr(ep_ctx->udev, ep_ctx->ep_index);

    u64 entry = le64(ep_ctx->streams->ctx_array[slot + 1].stream_ring);
    return (dma_addr_t)(entry & ~0xFULL);
}

/* Start a Stopped endpoint on the transfers that stayed queued on it.
 *
 * A Stopped endpoint does nothing until a doorbell is rung for it, and a
 * doorbell names one ring: the default ring, or one stream's ring.  A new
 * submission rings it as a matter of course - but here nothing new is being
 * submitted.  The transfers are already on the rings: they survived an abort
 * or a timeout of their neighbours, sat behind a halted transfer, or waited
 * out a port suspend.  So every ring that holds any gets its doorbell here,
 * and rings without transfers are left alone (Linux:
 * ring_doorbell_for_active_rings). */
static void ep_restart_queued(struct ep_context *ep_ctx)
{
    for (u16 slot = 0; slot < ep_ring_count(ep_ctx); ++slot)
    {
        struct xhci_ring *ring = ep_ring_by_slot(ep_ctx, slot);

        if (!xhci_td_is_empty(xhci_ring_get_td_list(ring)))
            xhci_submit_ring_doorbell(ep_ctx->udev, ring);
    }
}

/* Hold a stopped endpoint for a request that has to cross the wire first. */
static void ep_hold_wait(struct ep_context *ep_ctx, u8 holds)
{
    ep_ctx->hold |= holds;
    ep_ctx->hold_until_us = get_time() + EP_HOLD_WAIT_MS * 1000UL;
}

/* A reason to hold the endpoint is gone: if it was parked, see whether it can
 * run now. */
static void ep_release(struct ep_context *ep_ctx, u8 holds)
{
    ep_ctx->hold &= (u8)~holds;
    if (ep_ctx->state == USB_DEV_EP_STATE_PARKED)
        ep_service(ep_ctx);
}

/* The host side of a halt is recovered: the ring is re-armed past the failed
 * transfer and that transfer can be answered.  One thing is left that only
 * this end can do: behind a transaction translator, a failed split leaves a
 * control or bulk transfer in the hub's TT buffer, and the endpoint stays
 * shut until the hub has cleared it (xHCI 4.6.8).
 *
 * Returns the transfer; the caller answers it last, when the endpoint is in
 * its next state - the stack's clear-halt is caused by that answer, so it
 * always finds the endpoint ready for it. */
static struct xhci_xfer *ep_halt_recovered(struct ep_context *ep_ctx)
{
    struct usb_device *udev = ep_ctx->udev;
    struct xhci_xfer *req = ep_ctx->halted_req;
    const s32 ep_type = xhci_ep_type_for_index(udev, ep_ctx->ep_index);

    ep_ctx->halted_req = NULL;

    /* held first: a request that fails at once reports back from inside the
     * call */
    ep_hold_wait(ep_ctx, EP_HOLD_TT);
    if (!xhci_udev_clear_tt_buffer(udev, ep_ctx->ep_index, ep_type))
        ep_ctx->hold &= (u8)~EP_HOLD_TT; /* no translator behind this endpoint, or nothing went out */

    return req;
}

/*
 * The one place where a stopped endpoint is dealt with.  The rings are
 * stopped and no endpoint command is in flight.
 */
static void ep_service(struct ep_context *ep_ctx)
{
    /* 1. What was wished for, ring by ring: retire the victims.  What is
     * left on a ring stays queued, and the ring is re-armed - one Set TR
     * Dequeue - only if the TD the controller stopped in went with them. */
    if (ep_wishes(ep_ctx))
    {
        const BOOL flush = ep_ctx->want_flush;
        const u32 now_us = get_time();

        /* the stream context array is device-written: one invalidate covers
         * every stopped-dequeue read below */
        if (ep_ctx->streams)
        {
            struct ExecBase *SysBase = ep_ctx->sysBase;
            cache_post_dma(ep_ctx->streams->ctx_array, ep_ctx->streams->ctx_array_bytes, 0);
        }

        for (u16 slot = 0; slot < ep_ring_count(ep_ctx); ++slot)
        {
            struct xhci_ring *ring = ep_ring_by_slot(ep_ctx, slot);
            const dma_addr_t deq = xhci_td_retire(xhci_ring_get_td_list(ring), &ep_ctx->abort_reqs, now_us,
                                                  ep_ring_stopped_deq(ep_ctx, slot), flush);

            if (deq && !ep_command(ep_ctx, TRB_SET_DEQ, 0, ring, deq))
                return; /* FAILED */
        }
        ep_clear_wishes(ep_ctx);

        if (flush)
        {
            /* an iso stream that was flushed away is dead without a STOP.
             * What a halt waits for is still owed (EP_HOLD_WAITS). */
            ep_ctx->hold &= EP_HOLD_WAITS;
            xhci_ep_rt_iso_fire_release(ep_ctx);
        }

        if (ep_ctx->cmds_pending)
            return; /* the last Set TR Dequeue comes back here */
    }

    /* 2. A halt: the host side is done. */
    struct xhci_xfer *halted = ep_ctx->halted_req ? ep_halt_recovered(ep_ctx) : NULL;

    /* 3. Held?  Then new transfers wait in the pending queue with the ones
     * kept on the rings. */
    ep_suspend_parked(ep_ctx);

    if (ep_ctx->hold)
        xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_PARKED);
    else
    {
        /* 4. Run. */
        ep_restart_queued(ep_ctx);
        xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RECEIVING);
        xhci_ep_set_idle(ep_ctx);
    }

    if (halted)
        xhci_xfer_complete(ep_ctx->udev, halted, halted->error, halted->actual);
}

/* Halt recovery, once the endpoint is Stopped: the TD the controller stopped
 * in - the failed transfer's - is off the list already, so its ring is
 * re-armed (xhci_td_first_deq).  The rings of other streams keep the dequeue
 * the controller saved. */
static void ep_rearm_halted(struct ep_context *ep_ctx)
{
    struct xhci_ring *ring = xhci_ep_get_ring_for_stream(ep_ctx, ep_ctx->halted_req->stream_id);
    if (!ring)
    {
        xhci_ep_set_failed(ep_ctx);
        return;
    }

    ep_command(ep_ctx, TRB_SET_DEQ, 0, ring, xhci_td_first_deq(xhci_ring_get_td_list(ring)));
}

void xhci_ep_halted(struct ep_context *ep_ctx, struct xhci_xfer *req)
{
    struct usb_device *udev = ep_ctx->udev;
    const u32 hw_state = xhci_read_hw_ep_state(udev, ep_ctx->ep_index);

    Kprintf("recovering slot %lu ep %lu (hw ep state %lu)\n",
            (ULONG)udev->slot_id, (ULONG)ep_ctx->ep_index, (ULONG)hw_state);

    ep_ctx->halted_req = req;

    /* Reset Endpoint is only valid in the Halted state (xHCI 4.6.8).  An
     * endpoint in the Error state (where a TRB Error leaves it, xHCI 4.8.3)
     * is ready for the Set TR Dequeue as it is, and nothing about its data
     * toggle has changed. */
    if (hw_state == EP_STATE_ERROR)
    {
        ep_rearm_halted(ep_ctx);
        return;
    }

    /* Reset Endpoint zeroes the host's data toggle or sequence number; the
     * device's is zeroed by CLEAR_FEATURE(ENDPOINT_HALT) only, which also
     * ends a STALL and restarts a stream endpoint's state machine (USB 3.2
     * 4.4.6.4).  Nothing may go out in between (xHCI 4.10.2.1.1), so a bulk
     * or interrupt endpoint is held until the stack's clear has come:
     * xhci_ep_clear_halt_follow().  A control endpoint's stall ends with the
     * next SETUP. */
    const s32 ep_type = xhci_ep_type_for_index(udev, ep_ctx->ep_index);
    if (ep_type == USB_ENDPOINT_XFER_BULK || ep_type == USB_ENDPOINT_XFER_INT)
        ep_hold_wait(ep_ctx, EP_HOLD_HALT);

    ep_command(ep_ctx, TRB_RESET_EP, 0, NULL, 0); /* TSP=0: also drops a cached split */
}

/* Soft retry (xHCI 4.6.8.1).  The controller has given up on a transaction
 * after its own retries and halted the endpoint; the device knows nothing of
 * that and waits for the next attempt.  Reset Endpoint with the transfer
 * state preserved, then the doorbell, has the controller try the same
 * transaction again - same data toggle, same place in the buffer.  A passing
 * disturbance on the bus is over by then, and neither the class nor the
 * stack hears of it.  The transfer stays where it is, on the ring and on the
 * list; if the retries run out it fails the ordinary way (xhci_ep_halted).
 *
 * Bulk and interrupt endpoints only - never isoch - and not behind a
 * transaction translator, where the hub's buffer needs clearing first.
 *
 * TRUE = the event is dealt with (also when the command could not be queued
 * and the endpoint failed); FALSE = no retry, treat it as the halt it is. */
BOOL xhci_ep_soft_retry(struct ep_context *ep_ctx)
{
    struct usb_device *udev = ep_ctx->udev;
    const s32 ep_type = xhci_ep_type_for_index(udev, ep_ctx->ep_index);
    u8 tt_port;

    if (ep_ctx->state != USB_DEV_EP_STATE_RECEIVING ||
        (ep_type != USB_ENDPOINT_XFER_BULK && ep_type != USB_ENDPOINT_XFER_INT) ||
        ep_ctx->soft_retries >= EP_SOFT_RETRIES ||
        xhci_tt_hub(udev, &tt_port))
        return FALSE;

    ++ep_ctx->soft_retries;
    Kprintf("slot %lu ep %lu: transaction error, soft retry %lu of %lu\n",
            (ULONG)udev->slot_id, (ULONG)ep_ctx->ep_index,
            (ULONG)ep_ctx->soft_retries, (ULONG)EP_SOFT_RETRIES);

    /* its completion ends in ep_service(), which rings the doorbells */
    ep_command(ep_ctx, TRB_RESET_EP, TRB_TSP, NULL, 0);
    return TRUE;
}

void xhci_ep_tt_cleared(struct ep_context *ep_ctx)
{
    if (ep_ctx->hold & EP_HOLD_TT)
        ep_release(ep_ctx, EP_HOLD_TT);
}

void xhci_ep_command_done(struct ep_context *ep_ctx, trb_type cmd, BOOL ok)
{
    /* failed, or torn down and built anew, while the command was in flight */
    if (ep_ctx->state != USB_DEV_EP_STATE_RECOVERING || !ep_ctx->cmds_pending)
        return;

    if (!ok)
    {
        xhci_ep_set_failed(ep_ctx);
        return;
    }

    if (--ep_ctx->cmds_pending)
        return; /* a recovery re-arms one ring per command: wait for the last */

    /* after the reset of a halt its ring is carried past the failed transfer;
     * after the reset of a soft retry there is nothing to move */
    if (cmd == TRB_RESET_EP && ep_ctx->halted_req)
        ep_rearm_halted(ep_ctx);
    else
        ep_service(ep_ctx);
}

/* Abort the in-flight or queued direct transfer with this cookie (a wish;
 * called from xhci_direct_abort under the lock). */
void xhci_ep_abort_cookie(struct ep_context *ep_ctx, APTR cookie)
{
    struct ExecBase *SysBase = ep_ctx->sysBase;
    /* still software-queued (ring was busy): retire it without wire work */
    for (struct MinNode *node = ep_ctx->pending_reqs.mlh_Head; node->mln_Succ; node = node->mln_Succ)
    {
        struct xhci_xfer *req = (struct xhci_xfer *)node;
        if ((req->priv_flags & REQ_DIRECT) && req->cookie == cookie)
        {
            Remove((struct Node *)node);
            /* the funnel: a queued request still owns its mapping */
            xhci_xfer_complete(ep_ctx->udev, req, IOERR_ABORTED, 0);
            return;
        }
    }

    /* on a ring: note it and get the endpoint stopped.  RT ISO is stopped
     * through the hook API, not aborted here. */
    struct xhci_xfer *req = NULL;
    for (u16 slot = 0; slot < ep_ring_count(ep_ctx) && !req; ++slot)
        req = xhci_td_find_cookie_request(xhci_ring_get_td_list(ep_ring_by_slot(ep_ctx, slot)), cookie);
    if (!req || req->type == UHCD_EPTYPE_ISO)
        return;

    IOReqNode *wish = pool_alloc(ep_ctx->udev->controller->metaPool, sizeof(*wish));
    if (wish)
    {
        wish->req = req;
        AddTailMinList(&ep_ctx->abort_reqs, (struct MinNode *)wish);
    }
    else
    {
        /* The completion contract holds even out of memory: without a node
         * to name the one transfer, everything on the rings goes. */
        Kprintf("EP %lu: no memory to note an abort, flushing\n", (ULONG)ep_ctx->ep_index);
        ep_ctx->want_flush = TRUE;
    }

    ep_want_stop(ep_ctx);
}

void xhci_ep_check_timeouts(struct ep_context *ep_ctx)
{
    /* what a halted endpoint waits for has not come: run on without it */
    if ((ep_ctx->hold & EP_HOLD_WAITS) && (s32)(get_time() - ep_ctx->hold_until_us) >= 0)
    {
        Kprintf("EP %lu: not cleared within %lu ms (hold %02lx), running on\n",
                (ULONG)ep_ctx->ep_index, (ULONG)EP_HOLD_WAIT_MS, (ULONG)ep_ctx->hold);
        ep_release(ep_ctx, EP_HOLD_WAITS);
    }

    for (u16 slot = 0; slot < ep_ring_count(ep_ctx); ++slot)
    {
        if (xhci_td_is_expired(xhci_ring_get_td_list(ep_ring_by_slot(ep_ctx, slot))))
        {
            KprintfT("TD timeout on slot %lu ep %lu\n", (ULONG)ep_ctx->udev->slot_id, (ULONG)ep_ctx->ep_index);
            ep_ctx->want_timeouts = TRUE;
            ep_want_stop(ep_ctx);
            return;
        }
    }
}

void xhci_ep_request_stop(struct ep_context *ep_ctx)
{
    switch (ep_ctx->state)
    {
    case USB_DEV_EP_STATE_RECEIVING:
    case USB_DEV_EP_STATE_RECOVERING:
    case USB_DEV_EP_STATE_PARKED:
    case USB_DEV_EP_STATE_RT_ISO_RUNNING:
        ep_ctx->want_flush = TRUE;
        ep_want_stop(ep_ctx);
        break;
    default:
        break; /* nothing on the rings */
    }
}

void xhci_ep_quiesce(struct ep_context *ep_ctx)
{
    xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RECOVERING);
}

/* Stop the endpoint's rings ahead of a port suspend (U3).  The TDs stay on
 * the rings untouched; xhci_ep_resume() restarts them after the port is back
 * in U0.  TRUE = a Stop Endpoint was issued and the caller hears from
 * xhci_udev_suspend_stop_done() when the endpoint is parked.  An endpoint
 * that is stopped already, or in the middle of a recovery, is held all the
 * same and needs no waiting for; iso streams are left alone. */
BOOL xhci_ep_request_suspend(struct ep_context *ep_ctx)
{
    switch (ep_ctx->state)
    {
    case USB_DEV_EP_STATE_IDLE:
    case USB_DEV_EP_STATE_RECEIVING:
        /* the Suspend flag: the controller may power-manage an endpoint
         * stopped for this reason (xHCI 4.6.9, 4.15.1.1) */
        if (!ep_command(ep_ctx, TRB_STOP_RING, TRB_SP, NULL, 0))
            return FALSE;
        ep_ctx->hold |= EP_HOLD_SUSPEND;
        ep_ctx->suspend_notify = TRUE;
        return TRUE;
    case USB_DEV_EP_STATE_RECOVERING:
    case USB_DEV_EP_STATE_PARKED:
        ep_ctx->hold |= EP_HOLD_SUSPEND;
        return FALSE;
    default:
        return FALSE;
    }
}

/* The port is back in U0: what was kept on the rings runs on, and what was
 * submitted meanwhile follows. */
void xhci_ep_resume(struct ep_context *ep_ctx)
{
    if (!(ep_ctx->hold & EP_HOLD_SUSPEND))
        return;

    ep_release(ep_ctx, EP_HOLD_SUSPEND);
}

/* The host data toggle follows a device-side clear-halt.
 *
 * CLEAR_FEATURE(ENDPOINT_HALT) zeroes the DEVICE's data toggle / sequence
 * number; the xHC keeps its own, and a mismatch costs the next transfer a
 * packet.  (Legacy HCDs honour the same contract by snooping the request.)  A
 * clear that answers a STALL needs nothing, Reset Endpoint has zeroed the host
 * side - this is for a clear on a healthy endpoint: Reset Recovery on the pipe
 * that did not stall, a serial adapter's open sequence.  Reset Endpoint is
 * Halted-only (xHCI 4.6.8), so the route is a Configure Endpoint that drops and
 * adds the endpoint, re-initialising its context.
 *
 * The clear-halt is not replied until that command has completed: its xfer
 * rides as the command's request, which handle_config_ep - or the command
 * watchdog - retires.  The owner is therefore still blocked in its clear, and
 * that is what keeps the target's ring quiet meanwhile; nothing is parked and
 * no state is kept.
 *
 * TRUE = the request now belongs to the command.  FALSE = nothing was done and
 * the caller replies as usual: every guard is a reason to leave the endpoint
 * exactly as it is. */
BOOL xhci_ep_clear_halt_follow(struct usb_device *udev, struct xhci_xfer *req)
{
    if (!xhci_setup_is_clear_halt(&req->setup))
        return FALSE;

    const u8 addr = (u8)(le16(req->setup.usd_Index) & 0xffU);
    if ((addr & 0x0fU) == 0)
        return FALSE;

    const u8 ep_index = xhci_ep_index_from_address(addr);
    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
    if (!ep_ctx)
        return FALSE;

    /* the clear a halted endpoint was waiting for; its host side needs
     * nothing more */
    if (ep_ctx->hold & EP_HOLD_HALT)
    {
        Kprintf("EP %lu: halt cleared\n", (ULONG)ep_ctx->ep_index);
        ep_release(ep_ctx, EP_HOLD_HALT);
        return FALSE;
    }

    /* control and iso carry no data toggle; a stream endpoint is never cleared
     * blind (UAS recovers through task management) */
    const s32 ep_type = xhci_ep_type_for_index(udev, ep_index);
    if ((ep_type != USB_ENDPOINT_XFER_BULK && ep_type != USB_ENDPOINT_XFER_INT) ||
        xhci_ep_streams_count(ep_ctx))
        return FALSE;

    /* a quiet endpoint only: re-arming at the software enqueue strands nothing */
    if (ep_ctx->state != USB_DEV_EP_STATE_IDLE || !ep_tds_empty(ep_ctx) ||
        ep_ctx->pending_reqs.mlh_Head != (struct MinNode *)&ep_ctx->pending_reqs.mlh_Tail)
        return FALSE;

    /* a command still on the ring may change the slot context ours is a copy of */
    if (xhci_device_command_pending(udev))
        return FALSE;

    Kprintf("clear-halt on slot %lu ep %lu: resetting the host data toggle\n",
            (ULONG)udev->slot_id, (ULONG)ep_index);

    xhci_build_ep_toggle_reset_ctx(udev, ep_index);
    return xhci_configure_endpoints(udev, udev->toggle_in_ctx, FALSE, req);
}

enum ep_state xhci_ep_get_state(struct ep_context *ep_ctx)
{
    return ep_ctx->state;
}

u8 xhci_ep_get_ep_index(struct ep_context *ep_ctx)
{
    return ep_ctx->ep_index;
}

struct xhci_ring *xhci_ep_get_ring(struct ep_context *ep_ctx)
{
    return ep_ctx->ring;
}

/*
 * SS bulk streams (NSCMD_USB_ALLOC/FREE_STREAMS)
 */

void xhci_ep_set_max_streams(struct ep_context *ep_ctx, u16 max_streams)
{
    ep_ctx->max_streams = max_streams;
}

u16 xhci_ep_get_max_streams(struct ep_context *ep_ctx)
{
    return ep_ctx->max_streams;
}

BOOL xhci_ep_streams_active(struct ep_context *ep_ctx)
{
    return ep_ctx->streams != NULL;
}

u16 xhci_ep_streams_count(struct ep_context *ep_ctx)
{
    return ep_ctx->streams ? ep_ctx->streams->num_streams : 0;
}

u8 xhci_ep_streams_max_pstreams(struct ep_context *ep_ctx)
{
    return ep_ctx->streams ? ep_ctx->streams->max_pstreams : 0;
}

dma_addr_t xhci_ep_streams_array_dma(struct ep_context *ep_ctx)
{
    return ep_ctx->streams ? (dma_addr_t)ep_ctx->streams->ctx_array : 0;
}

struct xhci_ring *xhci_ep_get_ring_for_stream(struct ep_context *ep_ctx, u16 stream_id)
{
    if (!ep_ctx->streams)
        return ep_ctx->ring; /* single-ring mode: stream ids ride along ignored */

    if (stream_id == 0 || stream_id > ep_ctx->streams->num_streams)
        return NULL; /* an LSA endpoint has no default ring */

    return ep_ctx->streams->rings[stream_id];
}

void xhci_ep_streams_destroy(struct ep_context *ep_ctx, s8 reply_code)
{
    struct ExecBase *SysBase = ep_ctx->sysBase;
    struct ep_streams *st = ep_ctx->streams;
    if (!st)
        return;

    struct xhci_ctrl *ctrl = ep_ctx->udev->controller;
    ep_ctx->streams = NULL;

    if (st->rings)
    {
        for (u16 id = 1; id <= st->num_streams; ++id)
            ep_ring_free_with_tds(ctrl, st->rings[id], reply_code);
        pool_free(ctrl->metaPool, st->rings);
    }
    if (st->ctx_array)
        dma_free(ctrl->dmaPool, st->ctx_array);
    pool_free(ctrl->metaPool, st);
}

/* Build the software half of stream mode: one ring per stream id plus the
 * linear stream context array pointing at them.  The endpoint context switch
 * (MaxPStreams/LSA/deq) is the caller's Configure Endpoint. */
s8 xhci_ep_streams_build(struct ep_context *ep_ctx, u16 num_streams, u8 max_pstreams_cap)
{
    struct xhci_ctrl *ctrl = ep_ctx->udev->controller;
    struct ExecBase *SysBase = ctrl->sysBase;

    /* Array entries = 2^(p+1) including the reserved entry 0, so p is the
     * smallest exponent with 2^(p+1) > num_streams (p >= 1 per spec). */
    u8 p = 1;
    while ((1UL << (p + 1)) <= num_streams)
        ++p;
    if (p > max_pstreams_cap)
        return UHIOERR_BADPARAMS;

    struct ep_streams *st = pool_zalloc(ctrl->metaPool, sizeof(*st));
    if (!st)
        return UHIOERR_OUTOFMEMORY;

    st->num_streams = num_streams;
    st->max_pstreams = p;

    const u32 entries = 1UL << (p + 1);
    st->ctx_array_bytes = entries * sizeof(struct xhci_stream_ctx);
    st->ctx_array = xhci_malloc_page_bounded(ctrl, st->ctx_array_bytes, sizeof(struct xhci_stream_ctx));
    st->rings = pool_zalloc(ctrl->metaPool, ((u32)num_streams + 1) * sizeof(struct xhci_ring *));
    if (!st->ctx_array || !st->rings)
        goto fail;

    for (u16 id = 1; id <= num_streams; ++id)
    {
        struct xhci_ring *ring = xhci_ring_alloc(ctrl, XHCI_INITIAL_SEGMENTS_PER_RING,
                                                 /*link_trbs*/ TRUE, /*is_event_ring*/ FALSE,
                                                 ep_ctx->ep_index, ep_ctx->max_packet_size);
        if (!ring)
            goto fail;
        xhci_ring_set_stream_id(ring, id);
        st->rings[id] = ring;
        if (!xhci_td_create_list(ctrl, ep_ctx, ring))
            goto fail;

        /* deq | DCS from the fresh ring, plus the primary-TR context type */
        st->ctx_array[id].stream_ring =
            le64((u64)xhci_ring_get_new_dequeue_ptr(ring) | SCT_FOR_CTX(SCT_PRI_TR));
    }
    /* MUST stay clean+invalidate (flags 0): the xHC WRITES stream contexts
     * (it saves TR Dequeue Pointers into this array on stream switches and
     * when the endpoint stops, xHCI 4.12) — this is a device-written
     * structure's pre-arm, not a device-read flush. Do not "optimize" to
     * DMA_ReadFromRAM. */
    cache_pre_dma(st->ctx_array, st->ctx_array_bytes, 0);

    ep_ctx->streams = st;
    KprintfT("EP %lu: built %lu stream rings (MaxPStreams=%lu, %lu ctx entries)\n",
             (ULONG)ep_ctx->ep_index, (ULONG)num_streams, (ULONG)p, (ULONG)entries);
    return UHIOERR_NO_ERROR;

fail:
    ep_ctx->streams = st;       /* let the destroy path free the partial state */
    xhci_ep_streams_destroy(ep_ctx, UHIOERR_OUTOFMEMORY);
    return UHIOERR_OUTOFMEMORY;
}

/* Transfer events carry no stream id, so the TRB resolves to its TD by a
 * bounded scan of the endpoint's rings (≤63 on a streams endpoint, each with
 * a final-TRB fast pass).  A mid-TD short sets *deferred on the owning list
 * and stops the scan. */
BOOL xhci_ep_complete_by_trb(struct ep_context *ep_ctx, dma_addr_t trb_addr,
                             u32 residue, BOOL short_packet,
                             struct xhci_td_completion *out, BOOL *deferred)
{
    *deferred = FALSE;

    for (u16 slot = 0; slot < ep_ring_count(ep_ctx); ++slot)
    {
        TransferDescriptorList *tds = xhci_ring_get_td_list(ep_ring_by_slot(ep_ctx, slot));
        if (xhci_td_complete_by_trb(tds, trb_addr, residue, short_packet, out, deferred))
        {
            ep_ctx->soft_retries = 0; /* the next transfer has its own */
            return TRUE;
        }
        if (*deferred)
            return FALSE; /* found: length recorded, TD kept for its final-TRB event */
    }

    return FALSE; /* no ring owns this TRB */
}

u32 xhci_ep_get_active_trb_count(struct ep_context *ep_ctx)
{
    u32 count = 0;
    for (u16 slot = 0; slot < ep_ring_count(ep_ctx); ++slot)
        count += xhci_ring_get_queued_trbs(ep_ring_by_slot(ep_ctx, slot));
    return count;
}

void xhci_ep_flush(struct ep_context *ep_ctx, s8 reply_code)
{
    struct ExecBase *SysBase = ep_ctx->sysBase;
    struct MinNode *node;
    while ((node = RemHeadMinList(&ep_ctx->pending_reqs)) != NULL)
    {
        struct xhci_xfer *req = (struct xhci_xfer *)node;
        xhci_xfer_complete(ep_ctx->udev, req, reply_code, 0);
    }
}

