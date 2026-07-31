/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Endpoint internals shared by exactly two translation units: xhci-endpoint.c
 * (lifecycle, state machine, streams, recovery bookkeeping) and
 * xhci-ep-rtiso.c (the clock-driven RT-ISO engine).  Deliberately in src/,
 * not include/ — nothing else may look inside an ep_context (the rest of the
 * driver goes through the xhci-endpoint.h accessors).
 */

#ifndef __XHCI_ENDPOINT_PRIV_H
#define __XHCI_ENDPOINT_PRIV_H

#include <slab.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-ring.h>
#include "xhci-td-priv.h"

struct ep_context
{
    struct usb_device *udev; /* back reference to device */
    u8 ep_index;             /* Endpoint context index (0-30) */
    enum ep_state state;     /* Current endpoint state */
    u32 max_packet_size;     /* Cached max packet size for this endpoint */
    u8 max_burst;            /* bMaxBurst (zero-based: 0 = 1 packet/burst) */
    u8 hw_ep_type;           /* xHCI EP Type (usb xfer type | dir<<2), cached at
                              * context wiring so submits skip the device-context
                              * invalidate+read; 0 = not wired */

    IOReqList pending_reqs; /* list of pending requests */

    struct xhci_ring *ring; /* default ring for this endpoint; in-flight TDs
                             * live on each ring's own TD list */

    IOReqList stop_abort_reqs;
    BOOL stop_process_timeouts;

    /* Outstanding Set TR Deq commands of the recovery/flush in flight (one
     * per targeted ring; a plain endpoint's recovery is just count 1).
     * handle_set_deq consumes them and restarts the endpoint after the
     * last. */
    u16 pending_setdeq;

    /* TRUE between the suspend path's Stop Endpoint and its completion: an
     * abort arriving in that window is queued on stop_abort_reqs and recovered
     * from the stop's completion; once clear, the ring is known stopped and
     * recovery runs synchronously against the output-context dequeue. */
    BOOL suspend_stop_pending;

    /* Driver-initiated STALL recovery already sent CLEAR_FEATURE(HALT) to the
     * device; the next stack-issued clear-halt is a duplicate and is answered
     * without a wire request (consumed by xhci_ep_consume_halt_synced). */
    BOOL halt_cleared_internally;

    /* xHCI EP Context Interval decoded to microframes-per-ESIT; set for every
     * endpoint at context creation (needed before RT hooks register). */
    u16 rt_uframes_per_esit;

    /* SS bulk: highest stream id the endpoint supports (from the configure
     * op's ed_MaxStreams); 0 = endpoint has no stream capability. */
    u16 max_streams;

    /* Stream mode (NSCMD_USB_ALLOC_STREAMS .. FREE_STREAMS lifetime); NULL
     * while the endpoint runs its default single ring. */
    struct ep_streams *streams;

    /* RT ISO streaming state - allocated when isochronous hooks are
     * registered; non-iso endpoints don't carry it. */
    struct rt_iso_state *rt;
};

/* Per-endpoint stream mode: the linear stream context array the endpoint
 * context points at while in stream mode, plus one transfer ring per stream
 * id.  Each stream ring carries its own TD list, so recovery and restarts
 * know exactly which rings hold TDs; transfer events (which carry no stream
 * id) resolve TRB → ring by a bounded scan of the rings' lists. */
struct ep_streams
{
    u16 num_streams;                   /* valid stream ids: 1..num_streams */
    u8 max_pstreams;                   /* MaxPStreams exponent programmed into the EP context */
    struct xhci_stream_ctx *ctx_array; /* DMA: 2^(max_pstreams+1) entries */
    u32 ctx_array_bytes;
    struct xhci_ring **rings;          /* [0..num_streams]; [0] unused */
};

/* Per-endpoint clock-driven iso streaming state (NSCMD_USB_REGISTER_HOOKS ..
 * UNREGISTER_HOOKS lifetime). */
struct rt_iso_state
{
    struct USBIsoHooks *hooks;          /* stack-provided request/done/release hooks */
    struct xhci_xfer *stop_pending;  /* STOP_STREAM to reply once the pipe fully stopped */
    BOOL release_fired;                 /* uih_ReleaseHook fired (stream died without a STOP) */

    u16 direction; /* XHCI_DIR_IN / XHCI_DIR_OUT of the streaming endpoint */

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

/* Sole writer of ep_ctx->state (xhci-endpoint.c) - keeps every transition
 * observable in one place. */
void xhci_ep_transition(struct ep_context *ep_ctx, enum ep_state new_state);

/* Drain the pending queue back through the submit entry (xhci-endpoint.c). */
void xhci_ep_schedule_next(struct ep_context *ep_ctx);

/* RT-ISO teardown hooks the endpoint lifecycle calls into (xhci-ep-rtiso.c):
 * uih_ReleaseHook exactly once when the stream dies WITHOUT a client STOP,
 * and the IN staging slab destruction. */
void xhci_ep_rt_iso_fire_release(struct ep_context *ep_ctx);
void xhci_ep_destroy_rt_staging_slab(struct ep_context *ep_ctx);

/* The default ring's TD list (RT-ISO and single-ring paths; RT-ISO endpoints
 * never have streams). */
static inline TransferDescriptorList *ep_default_tds(struct ep_context *ep_ctx)
{
    return xhci_ring_get_td_list(ep_ctx->ring);
}

static inline u32 xhci_ep_get_active_td_count(struct ep_context *ep_ctx)
{
    return xhci_td_get_queued_td_count(ep_default_tds(ep_ctx));
}

#endif /* __XHCI_ENDPOINT_PRIV_H */
