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
#include <timing.h>

#include <xhci/xhci-endpoint.h>
#include "xhci-endpoint-priv.h"
#include <xhci/xhci-commands.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci.h>
#include <xhci/xhci-ring.h>
#include <xhci/xhci-submit.h>
#include <xhci/xhci-context.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-endpoint] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef TRACE
#undef KprintfT
#define KprintfT(fmt, ...) PrintPistorm("[xhci-endpoint] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

static void xhci_ep_clear_stop_processing(struct ep_context *ep_ctx);

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

static BOOL ep_has_request(struct ep_context *ep_ctx, struct xhci_xfer *req)
{
    for (u16 slot = 0; slot < ep_ring_count(ep_ctx); ++slot)
    {
        if (xhci_td_has_request(xhci_ring_get_td_list(ep_ring_by_slot(ep_ctx, slot)), req))
            return TRUE;
    }

    return FALSE;
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
    /* a re-add over a live context (alt-setting switch): retire the old one
     * first — anything still pending fails with device-gone semantics */
    if (udev->ep_context[ep_index])
        xhci_ep_destroy_context(udev, ep_index, UHIOERR_TIMEOUT);

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
    ep_ctx->ring = xhci_ring_alloc(udev->controller, XHCI_INITIAL_SEGMENTS_PER_RING, /*link_trbs*/ TRUE, /*is_event_ring*/ FALSE, ep_index, max_packet_size);
    if (!ep_ctx->ring || !xhci_td_create_list(udev->controller, ep_ctx, ep_ctx->ring))
    {
        Kprintf("Failed to create resources for EP %d\n", ep_index);
        ep_ring_free_with_tds(udev->controller, ep_ctx->ring, UHIOERR_OUTOFMEMORY);
        pool_free(udev->controller->metaPool, ep_ctx);
        return FALSE;
    }

    xhci_ep_clear_stop_processing(ep_ctx);

    udev->ep_context[ep_index] = ep_ctx;
    return TRUE;
}

void xhci_ep_destroy_context(struct usb_device *udev, u8 ep_index, s8 reply_code)
{
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

    xhci_ep_clear_stop_processing(ep_ctx);
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

/* The endpoint is unusable until a recovery cycle (Set TR Deq -> set_idle) or
 * reconfiguration: fail everything in flight *and* queued so nothing
 * accumulates silently - new submissions are rejected by enqueue_td_internal
 * while in this state. */
void xhci_ep_set_failed(struct ep_context *ep_ctx)
{
    Kprintf("EP %lu state %lu -> FAILED\n", (ULONG)ep_ctx->ep_index, (ULONG)ep_ctx->state);
    xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_FAILED);
    xhci_ep_clear_stop_processing(ep_ctx);
    ep_fail_all_tds(ep_ctx, UHIOERR_HOSTERROR);
    xhci_ep_flush(ep_ctx, UHIOERR_HOSTERROR);
    xhci_ep_rt_iso_fire_release(ep_ctx); /* a registered iso stream is dead with the endpoint */
}

void xhci_ep_enqueue(struct ep_context *ep_ctx, struct xhci_xfer *io)
{
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
    if (ep_ctx->udev->controller->xfer_lock.ss_Owner != FindTask(NULL))
        Kprintf("xfer_lock NOT HELD on submit path!\n");
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

    if (state == USB_DEV_EP_STATE_ABORTING ||
        state == USB_DEV_EP_STATE_RESETTING ||
        state == USB_DEV_EP_STATE_SUSPENDED)
    {
        KprintfT("Cannot submit transfer, ep in state %d\n", state);
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
    struct MinNode *node;
    while ((node = RemHeadMinList(&ep_ctx->pending_reqs)))
    {
        struct xhci_xfer *req = (struct xhci_xfer *)node;

        KprintfT("starting queued request type=%lu ep=%lu\n",
                 (ULONG)req->type,
                 (ULONG)(req->endpoint & 0x0F));

        /* internal EP0 requests and deferred ctx shadows alike: straight
         * back through the submit entry */
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

void xhci_ep_set_idle(struct ep_context *ep_ctx)
{
    if (ep_tds_empty(ep_ctx))
        xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_IDLE);
    else if (ep_ctx->state == USB_DEV_EP_STATE_ABORTING)
        xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RECEIVING);

    if (ep_ctx->pending_reqs.mlh_Head != (struct MinNode *)&ep_ctx->pending_reqs.mlh_Tail)
        xhci_ep_schedule_next(ep_ctx);
}

/* Endpoint restart at the end of a ring-flush recovery (the last Set TR Deq
 * completed, or a raced-out streams abort found nothing to reset).  TDs that
 * survived a surgical recovery still sit on unreset stream rings the Stop
 * Endpoint descheduled — the per-ring TD lists say exactly which, so only
 * those rings get a doorbell (mirrors Linux ring_doorbell_for_active_rings).
 * Non-stream endpoints skip the kicks: their path is xhci_ep_set_idle,
 * unchanged. */
static void xhci_ep_kick_active_streams(struct ep_context *ep_ctx)
{
    for (u16 id = 1; id <= ep_ctx->streams->num_streams; ++id)
    {
        if (!xhci_td_is_empty(xhci_ring_get_td_list(ep_ctx->streams->rings[id])))
            xhci_submit_kick_ep(ep_ctx->udev, ep_ctx->ep_index, id);
    }
}

void xhci_ep_flush_complete(struct ep_context *ep_ctx)
{
    /* A recovery finishing on a still-suspended endpoint must not restart
     * anything: survivors and pending stay parked until xhci_ep_resume(). */
    if (ep_ctx->state == USB_DEV_EP_STATE_SUSPENDED)
        return;

    if (ep_ctx->streams)
        xhci_ep_kick_active_streams(ep_ctx);

    xhci_ep_set_idle(ep_ctx);
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

void xhci_ep_set_resetting(struct ep_context *ep_ctx)
{
    xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RESETTING);

    /* Fail and free any in-flight TDs so callers get a reply before reset.
     * These are collateral of the recovery, not timeouts - UHIOERR_TIMEOUT here
     * would feed Poseidon's dead-device counter +3 per request. */
    ep_fail_all_tds(ep_ctx, IOERR_ABORTED);
}

void xhci_ep_set_aborting(struct ep_context *ep_ctx)
{
    xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_ABORTING);
}

static BOOL xhci_ep_has_stop_abort_requests(struct ep_context *ep_ctx)
{
    return ep_ctx->stop_abort_reqs.mlh_Head != (struct MinNode *)&ep_ctx->stop_abort_reqs.mlh_Tail;
}

static BOOL xhci_ep_append_stop_abort_request(struct ep_context *ep_ctx, struct xhci_xfer *abort_req)
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

/* Abort/timeout recovery for an endpoint whose ring the suspend path already
 * stopped: identical mechanics to handle_stop_ring's recovery half minus any
 * restart — the endpoint stays SUSPENDED and xhci_ep_resume() restarts the
 * survivors after U0.  An anomalous stopped dequeue degrades to the coarse
 * retire-everything-in-place recovery (the rings are already stopped). */
static void xhci_ep_recover_stopped(struct ep_context *ep_ctx)
{
    if (xhci_ep_process_stop(ep_ctx))
        return;

    if (xhci_ep_has_stop_abort_requests(ep_ctx) || ep_ctx->stop_process_timeouts)
    {
        xhci_ep_set_failed(ep_ctx);
        xhci_flush_ep_rings(ep_ctx->udev, ep_ctx);
    }
}

static void xhci_ep_prepare_stop_processing(struct ep_context *ep_ctx, struct xhci_xfer *abort_req, BOOL process_timeouts)
{
    enum ep_state state = xhci_ep_get_state(ep_ctx);
    if (state != USB_DEV_EP_STATE_RECEIVING &&
        state != USB_DEV_EP_STATE_ABORTING &&
        state != USB_DEV_EP_STATE_SUSPENDED)
        return;

    if (abort_req)
    {
        if (abort_req->type == UHCD_EPTYPE_ISO)
            return; /* RT ISO is stopped through the hook API, not aborted here */

        if (!ep_has_request(ep_ctx, abort_req))
            return;

        if (!xhci_ep_append_stop_abort_request(ep_ctx, abort_req))
        {
            /* No node for a surgical abort: degrade to whole-endpoint
             * recovery rather than dropping the request — the completion
             * contract must hold even out of memory.  Suspended: the ring is
             * already stopped, retire-and-re-arm right here.  Otherwise drop
             * the surgical plan; the stop lands with no markers and the
             * ordinary-stop fallback (set_failed + ring flush) retires
             * everything. */
            xhci_ep_clear_stop_processing(ep_ctx);
            if (state == USB_DEV_EP_STATE_SUSPENDED)
            {
                xhci_ep_set_failed(ep_ctx);
                xhci_flush_ep_rings(ep_ctx->udev, ep_ctx);
                return;
            }
        }
    }

    ep_ctx->stop_process_timeouts |= process_timeouts;

    if (state == USB_DEV_EP_STATE_SUSPENDED)
    {
        /* Ring already stopped for U3: no command, no state excursion, no
         * doorbell.  A suspend stop still sequencing runs the recovery from
         * its completion (xhci_ep_suspend_stop_complete). */
        if (!ep_ctx->suspend_stop_pending)
            xhci_ep_recover_stopped(ep_ctx);
        return;
    }

    if (ep_ctx->state != USB_DEV_EP_STATE_ABORTING)
    {
        xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_ABORTING);
        xhci_stop_ring(ep_ctx->udev, ep_ctx->ep_index);
    }
}

static void xhci_ep_request_abort(struct ep_context *ep_ctx, struct xhci_xfer *abort_req)
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

    if (state == USB_DEV_EP_STATE_SUSPENDED)
    {
        /* CMD_FLUSH on a parked endpoint: the ring is already stopped —
         * retire everything in place and re-arm to the software enqueue
         * (the ordinary-stop fallback minus the redundant Stop Endpoint). */
        xhci_ep_set_failed(ep_ctx);
        xhci_flush_ep_rings(ep_ctx->udev, ep_ctx);
        return;
    }

    if (state != USB_DEV_EP_STATE_RECEIVING &&
        state != USB_DEV_EP_STATE_RT_ISO_RUNNING)
        return;

    KprintfT("EP %lu state %lu -> ABORTING (stop requested)\n", (ULONG)ep_ctx->ep_index, (ULONG)state);
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
        ep_ctx->suspend_stop_pending = TRUE;
        xhci_stop_ring(ep_ctx->udev, ep_ctx->ep_index);
        return TRUE;
    default:
        return FALSE;
    }
}

/* The suspend path's Stop Endpoint completed (handle_stop_ring, SUSPENDED
 * branch): the ring is now known stopped — run any abort/timeout recovery
 * queued while the stop was sequencing. */
void xhci_ep_suspend_stop_complete(struct ep_context *ep_ctx)
{
    ep_ctx->suspend_stop_pending = FALSE;
    xhci_ep_recover_stopped(ep_ctx);
}

/* Restart an endpoint after port resume: restore the pre-suspend state, kick
 * the doorbell if TDs are still queued, and drain any requests deferred while
 * suspended. */
void xhci_ep_resume(struct ep_context *ep_ctx)
{
    if (!ep_ctx || ep_ctx->state != USB_DEV_EP_STATE_SUSPENDED)
        return;

    if (ep_tds_empty(ep_ctx))
        xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_IDLE);
    else
    {
        xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RECEIVING);
        if (ep_ctx->streams)
            xhci_ep_kick_active_streams(ep_ctx);
        else
            xhci_submit_kick_ep(ep_ctx->udev, ep_ctx->ep_index, 0);
    }

    xhci_ep_schedule_next(ep_ctx);
}

/* The hardware's stopped dequeue for one ring of the endpoint.  Stream
 * rings: the xHC saves each stream's TR Dequeue Pointer into the stream
 * context array when the endpoint stops (xHCI 4.12.2) — the caller ran the
 * invalidate over the (device-written) array.  SCT and DCS are stripped: the
 * DCS write-back is not trusted (VL805 broken-DCS), the recovery machinery
 * takes the cycle from the TRB itself. */
static dma_addr_t ep_ring_stopped_deq(struct ep_context *ep_ctx, u16 stream_id)
{
    if (!stream_id)
        return (dma_addr_t)xhci_get_endpoint_deq_ptr(ep_ctx->udev, ep_ctx->ep_index);

    u64 entry = le64(ep_ctx->streams->ctx_array[stream_id].stream_ring);
    return (dma_addr_t)(entry & ~0xFULL);
}

BOOL xhci_ep_process_stop(struct ep_context *ep_ctx)
{
    if (!ep_ctx)
        return FALSE;

    if (!xhci_ep_has_stop_abort_requests(ep_ctx) && !ep_ctx->stop_process_timeouts)
        return FALSE;

    struct ep_streams *st = ep_ctx->streams;
    const u32 now_us = get_time();

    /* One invalidate covers every stopped-dequeue read below: the stream
     * context array is device-written (the xHC saved the per-stream
     * dequeues into it when the endpoint stopped). */
    if (st)
        cache_post_dma(st->ctx_array, st->ctx_array_bytes, 0);

    /* Pass 1 - validate and count, mutating nothing: every victim ring must
     * resolve a re-arm dequeue.  A stopped dequeue outside every tracked TD
     * is an anomaly; bail with the rings untouched and let the caller run the
     * coarse whole-endpoint recovery.  Both passes share one now_us, so the
     * victim set cannot shift between them. */
    u16 victims = 0;
    for (u16 slot = 0; slot < ep_ring_count(ep_ctx); ++slot)
    {
        struct xhci_ring *ring = ep_ring_by_slot(ep_ctx, slot);
        TransferDescriptorList *tds = xhci_ring_get_td_list(ring);
        const u16 stream_id = st ? (u16)(slot + 1) : 0;

        if (!xhci_td_has_recovery_victim(tds, &ep_ctx->stop_abort_reqs, now_us))
            continue;

        if (!xhci_td_resolve_recovery(tds, ring, &ep_ctx->stop_abort_reqs, now_us,
                                      ep_ring_stopped_deq(ep_ctx, stream_id)))
        {
            Kprintf("EP %lu stream %lu: stopped deq outside every TD, degrading to coarse recovery\n",
                    (ULONG)ep_ctx->ep_index, (ULONG)stream_id);
            return FALSE;
        }
        ++victims;
    }

    /* Arm the completion counter BEFORE any command goes out, so a Set TR Deq
     * completion can never see a zero count and restart the endpoint early. */
    ep_ctx->pending_setdeq = victims;

    /* Pass 2 - surgical recovery, ring by ring: rings without a victim are
     * skipped whole and their TDs keep running.  On a victim ring the
     * machinery No-Ops the victims' TRBs and replies them; the Set TR Deq
     * re-arms the ring at the resolved dequeue, so survivors on the SAME ring
     * keep running too. */
    for (u16 slot = 0; slot < ep_ring_count(ep_ctx); ++slot)
    {
        struct xhci_ring *ring = ep_ring_by_slot(ep_ctx, slot);
        TransferDescriptorList *tds = xhci_ring_get_td_list(ring);
        const u16 stream_id = st ? (u16)(slot + 1) : 0;

        if (!xhci_td_has_recovery_victim(tds, &ep_ctx->stop_abort_reqs, now_us))
            continue;

        const dma_addr_t stopped_deq = ep_ring_stopped_deq(ep_ctx, stream_id);
        const dma_addr_t new_deq = xhci_td_resolve_recovery(tds, ring, &ep_ctx->stop_abort_reqs,
                                                            now_us, stopped_deq);
        xhci_td_abort_recovery(tds, &ep_ctx->stop_abort_reqs, now_us, stopped_deq);

        KprintfT("EP %lu stream %lu recovery: HW stopped deq %08lx -> new deq %08lx\n",
                 (ULONG)ep_ctx->ep_index, (ULONG)stream_id,
                 (ULONG)stopped_deq, (ULONG)new_deq);
        xhci_set_deq_pointer(ep_ctx->udev, ep_ctx->ep_index, (u32)new_deq, stream_id);
    }

    xhci_ep_clear_stop_processing(ep_ctx);

    if (!victims)
    {
        /* the targeted TDs completed before the stop landed: nothing to
         * reset, restart the survivors right away */
        xhci_ep_flush_complete(ep_ctx);
    }

    return TRUE;
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
    for (u16 slot = 0; slot < ep_ring_count(ep_ctx); ++slot)
    {
        if (xhci_td_is_expired(xhci_ring_get_td_list(ep_ring_by_slot(ep_ctx, slot))))
            return TRUE;
    }

    return FALSE;
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
     * (it saves TR Dequeue Pointers into this array on stream switches,
     * xHCI 4.12.2) — this is a device-written structure's pre-arm, not a
     * device-read flush. Do not "optimize" to DMA_ReadFromRAM. */
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

/* Abort the in-flight or queued direct transfer with this cookie (a wish;
 * called from xhci_direct_abort under the lock). */
void xhci_ep_abort_cookie(struct ep_context *ep_ctx, APTR cookie)
{
    /* still software-queued (ring was busy): retire it without wire work */
    for (struct MinNode *node = ep_ctx->pending_reqs.mlh_Head; node->mln_Succ; node = node->mln_Succ)
    {
        struct xhci_xfer *req = (struct xhci_xfer *)node;
        if ((req->priv_flags & REQ_DIRECT) && req->cookie == cookie)
        {
            Remove((struct Node *)node);
            req->error = IOERR_ABORTED;
            xhci_xfer_reply(req); /* completes through the done hook */
            return;
        }
    }

    /* on a ring: the ordinary abort machinery (stop + recovery) */
    struct xhci_xfer *req = NULL;
    for (u16 slot = 0; slot < ep_ring_count(ep_ctx) && !req; ++slot)
        req = xhci_td_find_cookie_request(xhci_ring_get_td_list(ep_ring_by_slot(ep_ctx, slot)), cookie);
    if (req)
        xhci_ep_request_abort(ep_ctx, req);
}

void xhci_ep_setdeq_begin(struct ep_context *ep_ctx, u16 count)
{
    ep_ctx->pending_setdeq = count;
}

BOOL xhci_ep_setdeq_consume(struct ep_context *ep_ctx)
{
    if (ep_ctx->pending_setdeq == 0)
        return TRUE; /* untracked single command: defensive */
    return --ep_ctx->pending_setdeq == 0;
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
            return TRUE;
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
    struct MinNode *node;
    while ((node = RemHeadMinList(&ep_ctx->pending_reqs)) != NULL)
    {
        struct xhci_xfer *req = (struct xhci_xfer *)node;
        xhci_xfer_complete(ep_ctx->udev, req, reply_code, 0);
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

