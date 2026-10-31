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
    struct ExecBase *sysBase; /* udev->sysBase, copied at create */
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

    /* Why a stopped endpoint must not run again yet (EP_HOLD_*).  Several
     * reasons can hold at once; the endpoint restarts when the last is gone.
     * The ones that wait for a request on the wire (EP_HOLD_WAITS) lapse at
     * hold_until_us: an endpoint is not shut for good by a request that
     * never comes. */
    u8 hold;
    u32 hold_until_us;

    /* Endpoint commands in flight: Stop Endpoint, Reset Endpoint, or one Set
     * TR Dequeue per ring being re-armed.  The completion of the last one is
     * where the stopped endpoint is dealt with (ep_service). */
    u16 cmds_pending;

    /* Wishes: what to take off the rings once they are stopped.  Noted in any
     * state, served by ep_service(). */
    IOReqList abort_reqs; /* these requests' TDs */
    BOOL want_timeouts;   /* the TDs past their NAK deadline */
    BOOL want_flush;      /* every TD */

    /* The transfer a halt happened on, from the halt until the host side is
     * recovered - then it is answered.  Its TD is off the ring already. */
    struct xhci_xfer *halted_req;

    /* The device's suspend sequence counts this endpoint's Stop Endpoint and
     * waits to hear that the endpoint is parked. */
    BOOL suspend_notify;

    /* Soft retries spent since a TD last left the endpoint through its own
     * event (xhci_ep_soft_retry). */
    u8 soft_retries;

    /* Service-interval facts, set for every periodic endpoint at context
     * creation (needed before RT hooks register): the xHCI EP Context Interval
     * decoded to microframes-per-ESIT, and the Max ESIT Payload - the most one
     * interval can carry (packet size x high-bandwidth multiplier, or the
     * SuperSpeed wBytesPerInterval), which is the length of every RT-ISO IN TD. */
    u16 rt_uframes_per_esit;
    u32 rt_esit_payload;

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

#define EP_HOLD_SUSPEND 0x01 /* the port is in U3, or on its way there */
#define EP_HOLD_HALT    0x02 /* a halt's device side waits for CLEAR_FEATURE(ENDPOINT_HALT) */
#define EP_HOLD_TT      0x04 /* a halt's hub side waits for CLEAR_TT_BUFFER */
#define EP_HOLD_WAITS   (EP_HOLD_HALT | EP_HOLD_TT)
#define EP_HOLD_WAIT_MS 2000U /* the library's own clear-halt gives up after 1000 */
#define EP_SOFT_RETRIES 3     /* per transfer, each good for another CErr tries on the wire */

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

    /* IN: TDs per completion interrupt - 1 (a power of two - 1).  0 = every
     * TD interrupts; see xhci_ep_schedule_rt_iso_in. */
    u32 in_irq_batch_mask;

    u32 ist; /* IST decoded to microframes, cached at RT ISO start */

    /* IN staging slab: created in xhci_ep_rt_iso_start, one object per TD
     * (rt_esit_payload rounded up to a power of two, see there), capacity =
     * inflight_tds_target.  Destroyed in xhci_ep_set_rt_stopped and as a
     * safety net on endpoint teardown. */
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
