/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The endpoint state machine, streams and RT-ISO entries.  Every function
 * here requires ctrl->xfer_lock held; the lock sites are the unit-task work
 * blocks and the direct entries (xhci-direct.c).
 */

#ifndef __XHCI_ENDPOINT_H__
#define __XHCI_ENDPOINT_H__

#include <xhci/xhci-xfer.h>

enum ep_state
{
    USB_DEV_EP_STATE_IDLE = 0,
    USB_DEV_EP_STATE_RECEIVING,
    USB_DEV_EP_STATE_ABORTING,
    USB_DEV_EP_STATE_RESETTING,
    USB_DEV_EP_STATE_FAILED,
    USB_DEV_EP_STATE_SUSPENDED, /* rings stopped for port U3; TDs stay queued for resume */
    USB_DEV_EP_STATE_RT_ISO_STOPPED,
    USB_DEV_EP_STATE_RT_ISO_RUNNING,
    USB_DEV_EP_STATE_RT_ISO_STOPPING
};

struct usb_device;
struct ep_context;
struct xhci_dma_span;
struct xhci_ring;

/* Result of completing a TD (the out-param of xhci_ep_complete_by_trb):
 * either a request to reply (rt == FALSE) or the payload of an RT ISO TD (no
 * request object exists for those). */
struct xhci_td_completion
{
    BOOL rt;
    struct xhci_xfer *req; /* !rt */
    APTR rt_buffer;           /* rt: CPU buffer (staging for IN, class buffer for OUT) */
    u32 rt_length;            /* rt: submitted length */
    u16 rt_frame;
    u16 rt_dir;               /* XHCI_DIR_IN / XHCI_DIR_OUT */
    u32 act_len;
};

BOOL xhci_ep_create_context(struct usb_device *udev, u8 ep_index, u32 max_packet_size, u8 max_burst);
void xhci_ep_set_rt_interval(struct ep_context *ep_ctx, u8 interval);
void xhci_ep_destroy_context(struct usb_device *udev, u8 ep_index, s8 reply_code);
void xhci_ep_destroy_contexts(struct usb_device *udev, s8 reply_code);
struct ep_context *xhci_ep_get_context_for_index(struct usb_device *udev, u8 ep_index);

/* SS bulk streams (NSCMD_USB_ALLOC/FREE_STREAMS).  Software state only: build
 * allocates the stream rings + the linear stream context array, destroy frees
 * them; the endpoint-context switch is the Configure Endpoint the ctx-ops
 * layer issues around these. */
void xhci_ep_set_max_streams(struct ep_context *ep_ctx, u16 max_streams);
u16 xhci_ep_get_max_streams(struct ep_context *ep_ctx);
BOOL xhci_ep_streams_active(struct ep_context *ep_ctx);
u16 xhci_ep_streams_count(struct ep_context *ep_ctx);
u8 xhci_ep_streams_max_pstreams(struct ep_context *ep_ctx);
dma_addr_t xhci_ep_streams_array_dma(struct ep_context *ep_ctx);
s8 xhci_ep_streams_build(struct ep_context *ep_ctx, u16 num_streams, u8 max_pstreams_cap);
void xhci_ep_streams_destroy(struct ep_context *ep_ctx, s8 reply_code);
/* Set TR Dequeue bookkeeping shared by every recovery/flush (coarse = every
 * ring, surgical = victim rings only; a plain endpoint is just count 1): the
 * issuer counts one command per targeted ring and handle_set_deq consumes
 * them. */
void xhci_ep_setdeq_begin(struct ep_context *ep_ctx, u16 count);
BOOL xhci_ep_setdeq_consume(struct ep_context *ep_ctx); /* TRUE = last one */

void xhci_ep_set_max_packet_size(struct ep_context *ep_ctx, u32 max_packet_size);
u32 xhci_ep_get_max_packet_size(struct ep_context *ep_ctx);
u8 xhci_ep_get_max_burst(struct ep_context *ep_ctx);
/* xHCI EP Type (usb xfer type | dir<<2), cached at context wiring — the
 * submit-path type check reads this instead of the hardware context. */
void xhci_ep_set_hw_type(struct ep_context *ep_ctx, u8 hw_ep_type);
u8 xhci_ep_get_hw_type(struct ep_context *ep_ctx);

void xhci_ep_set_failed(struct ep_context *ep_ctx);
void xhci_ep_set_idle(struct ep_context *ep_ctx);
/* set_idle plus a kick of the stream rings still holding TDs — the restart
 * at the end of every ring flush (tail of handle_set_deq). */
void xhci_ep_flush_complete(struct ep_context *ep_ctx);
/* Both variants: FALSE = the endpoint is FAILED and trb_addrs is already
 * freed — the caller must not touch the ring or the array, and still owns
 * the request's/span's disposal.  The TD lands on ep_ring's own list. */
BOOL xhci_ep_set_receiving(struct ep_context *ep_ctx, struct xhci_ring *ep_ring,
                           struct xhci_xfer *req, dma_addr_t *trb_addrs, u32 timeout_ms, u32 trb_count);
BOOL xhci_ep_set_receiving_rt(struct ep_context *ep_ctx, const struct xhci_dma_span *span,
                              u16 frame, u16 dir, BOOL staging, dma_addr_t *trb_addrs, u32 trb_count);
void xhci_ep_set_resetting(struct ep_context *ep_ctx);
void xhci_ep_set_aborting(struct ep_context *ep_ctx);

void xhci_ep_request_timeout_recovery(struct ep_context *ep_ctx);
void xhci_ep_request_stop(struct ep_context *ep_ctx);
/* Surgical abort/timeout recovery over the endpoint's rings: rings without a
 * victim are skipped whole; on each victim ring the victims' TRBs are
 * No-Op'd, their requests replied, and one Set TR Deq re-arms the ring —
 * survivors on the same ring keep running.  Returns TRUE when the stop was
 * consumed (commands queued, or a raced-out abort restarted the endpoint
 * synchronously); FALSE = nothing to recover OR an anomalous stopped
 * dequeue — the caller runs the coarse ordinary-stop recovery. */
BOOL xhci_ep_process_stop(struct ep_context *ep_ctx);
BOOL xhci_ep_request_suspend(struct ep_context *ep_ctx);
/* The suspend path's Stop Endpoint completed (handle_stop_ring, SUSPENDED
 * branch): run any abort/timeout recovery queued while the stop sequenced. */
void xhci_ep_suspend_stop_complete(struct ep_context *ep_ctx);
void xhci_ep_resume(struct ep_context *ep_ctx);

/* Clear-halt deduplication for the driver's own STALL recovery (see
 * xhci_udev_clear_feature_halt) */
void xhci_ep_mark_halt_synced(struct ep_context *ep_ctx);
BOOL xhci_ep_consume_halt_synced(struct ep_context *ep_ctx);

BOOL xhci_ep_is_expired(struct ep_context *ep_ctx);
enum ep_state xhci_ep_get_state(struct ep_context *ep_ctx);
u8 xhci_ep_get_ep_index(struct ep_context *ep_ctx);
u32 xhci_ep_get_active_trb_count(struct ep_context *ep_ctx);
struct xhci_ring *xhci_ep_get_ring(struct ep_context *ep_ctx);

/* Transfer-time ring selection.  No streams: the default ring regardless of
 * stream_id (a stack that assigned stream ids without a successful
 * ALLOC_STREAMS keeps today's single-ring behavior).  Streams active:
 * stream_id must name an allocated stream — 0 or out-of-range returns NULL
 * (an LSA endpoint has no default ring). */
struct xhci_ring *xhci_ep_get_ring_for_stream(struct ep_context *ep_ctx, u16 stream_id);

/* Abort the in-flight or queued direct transfer with this cookie (a wish;
 * xhci_direct_abort). */
void xhci_ep_abort_cookie(struct ep_context *ep_ctx, APTR cookie);

BOOL xhci_ep_complete_by_trb(struct ep_context *ep_ctx, dma_addr_t trb_addr,
                             u32 residue, BOOL short_packet,
                             struct xhci_td_completion *out, BOOL *deferred);

/* THE transfer submit entry (direct path, pending-drain, internal EP0):
 * state gate, pending-queue deferral and stream-ring selection here; TD
 * mechanics in xhci_submit_td().  The NAK timeout rides the xfer
 * (XHCI_XF_TIMEOUT + timeout_ms). */
s8 xhci_ep_submit(struct ep_context *ep_ctx, struct xhci_xfer *io);

void xhci_ep_enqueue(struct ep_context *ep_ctx, struct xhci_xfer *io);
void xhci_ep_flush(struct ep_context *ep_ctx, s8 reply_code);

/* RT ISO functions.  The iso hooks are passed by typed parameter (not packed
 * into a request); STOP takes the client xfer as its deferred reply token. */
s8 xhci_ep_rt_iso_add_handler(struct ep_context *ep_ctx, struct USBIsoHooks *hooks, u8 direction);
s8 xhci_ep_rt_iso_rem_handler(struct ep_context *ep_ctx, struct USBIsoHooks *hooks);

s8 xhci_ep_rt_iso_start(struct ep_context *ep_ctx);
s8 xhci_ep_rt_iso_stop(struct ep_context *ep_ctx, struct USBIsoHooks *hooks, struct xhci_xfer *stop_token);

/* ubr_flags rides the *_done hook's buffer request (UHCD_UBF_XFER_ERROR when
 * the interval failed on the wire) */
void xhci_ep_rt_iso_in(struct ep_context *ep_ctx, APTR buffer, u32 length, u32 act_len, u16 rt_frame, u16 ubr_flags);
void xhci_ep_rt_iso_out(struct ep_context *ep_ctx, APTR buffer, u32 length, u32 act_len, u16 rt_frame, u16 ubr_flags);

void xhci_ep_schedule_rt_iso(struct ep_context *ep_ctx);

/* Free a staging IN buffer back to the endpoint's per-endpoint slab. */
void xhci_ep_free_rt_iso_buffer(struct ep_context *ep_ctx, APTR data_buffer);

#endif /* __XHCI_ENDPOINT_H__ */