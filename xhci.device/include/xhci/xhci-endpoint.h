/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The endpoint state machine, streams and RT-ISO entries.  Every function
 * here requires ctrl->xfer_lock held; the lock sites are the unit-task work
 * blocks and the direct entries (xhci-direct.c).
 */

#ifndef __XHCI_ENDPOINT_H__
#define __XHCI_ENDPOINT_H__

#include <xhci/xhci-xfer.h>
#include <xhci/xhci-ring.h>

/*
 * The driver's view of an endpoint.
 *
 * IDLE and RECEIVING: the controller may work on the rings (nothing queued /
 * TDs queued).  RECOVERING and PARKED: it does not - a command that stops,
 * resets or re-arms the endpoint is in flight, or the rings are stopped and
 * something holds the endpoint (EP_HOLD_*: its port is suspended, or after a
 * halt the device's endpoint or the hub's TT buffer is not cleared yet).  In
 * both, new transfers wait in the pending queue and the TDs on the rings stay
 * where they are.
 */
enum ep_state
{
    USB_DEV_EP_STATE_IDLE = 0,
    USB_DEV_EP_STATE_RECEIVING,
    USB_DEV_EP_STATE_RECOVERING,
    USB_DEV_EP_STATE_PARKED,
    USB_DEV_EP_STATE_FAILED,
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
    BOOL missed;              /* rt: passed over by the controller; the event's own TD is still queued */
    struct xhci_xfer *req; /* !rt */
    APTR rt_buffer;           /* rt: CPU buffer (staging for IN, class buffer for OUT) */
    u32 rt_length;            /* rt: submitted length */
    u16 rt_frame;
    u16 rt_dir;               /* XHCI_DIR_IN / XHCI_DIR_OUT */
    u32 act_len;
};

BOOL xhci_ep_create_context(struct usb_device *udev, u8 ep_index, u32 max_packet_size, u8 max_burst);
void xhci_ep_set_rt_service(struct ep_context *ep_ctx, u8 interval, u32 max_esit_payload);
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
void xhci_ep_set_max_packet_size(struct ep_context *ep_ctx, u32 max_packet_size);
u32 xhci_ep_get_max_packet_size(struct ep_context *ep_ctx);
u8 xhci_ep_get_max_burst(struct ep_context *ep_ctx);
/* xHCI EP Type (usb xfer type | dir<<2), cached at context wiring — the
 * submit-path type check reads this instead of the hardware context. */
void xhci_ep_set_hw_type(struct ep_context *ep_ctx, u8 hw_ep_type);
u8 xhci_ep_get_hw_type(struct ep_context *ep_ctx);

void xhci_ep_set_failed(struct ep_context *ep_ctx);
void xhci_ep_set_idle(struct ep_context *ep_ctx);
/* Both variants: FALSE = the endpoint is FAILED and trb_addrs is already
 * freed — the caller must not touch the ring or the array, and still owns
 * the request's/span's disposal.  The TD lands on ep_ring's own list. */
BOOL xhci_ep_set_receiving(struct ep_context *ep_ctx, struct xhci_ring *ep_ring,
                           struct xhci_xfer *req, dma_addr_t *trb_addrs, u32 timeout_ms, u32 trb_count);
BOOL xhci_ep_set_receiving_rt(struct ep_context *ep_ctx, const struct xhci_dma_span *span,
                              u16 frame, u16 dir, BOOL staging, dma_addr_t *trb_addrs, u32 trb_count);

/*
 * Recovery.  Whatever takes TDs off a ring that the controller may be working
 * on - an abort, a NAK timeout, a flush, a halt - needs the endpoint stopped
 * first, and so does a port suspend.  The endpoint sequences that itself and
 * deals with the stopped rings in one place (ep_service() in
 * xhci-endpoint.c).  These are the ways in.
 */

/* A transfer was answered with a halt (STALL, or one the controller raised).
 * req is that transfer, its TD already off the ring's list and its error and
 * actual set.  Only this transfer is retired: it is answered once the host
 * side is recovered, and what was queued behind it stays queued.  A bulk or
 * interrupt endpoint then sends nothing until the stack's
 * CLEAR_FEATURE(ENDPOINT_HALT) has completed (xhci_ep_clear_halt_follow), a
 * control or bulk endpoint behind a transaction translator nothing until the
 * hub has cleared its buffer (xhci_ep_tt_cleared) - or until 2 seconds have
 * passed without either. */
void xhci_ep_halted(struct ep_context *ep_ctx, struct xhci_xfer *req);
/* A transfer event said USB Transaction Error: try the same transaction again
 * without anybody hearing of it, up to three times per transfer (bulk and
 * interrupt, not behind a TT).  The event's TD is left alone.  TRUE = the
 * event is dealt with; FALSE = no retry, go on and treat it as a halt. */
BOOL xhci_ep_soft_retry(struct ep_context *ep_ctx);
/* The CLEAR_TT_BUFFER the recovery of a halt sent to the hub is done, whatever
 * came of it. */
void xhci_ep_tt_cleared(struct ep_context *ep_ctx);
/* One of the endpoint's own commands (xhci_queue_ep_command) completed - or
 * failed, or timed out: ok FALSE. */
void xhci_ep_command_done(struct ep_context *ep_ctx, trb_type cmd, BOOL ok);
/* Abort the in-flight or queued direct transfer with this cookie (a wish;
 * xhci_direct_abort). */
void xhci_ep_abort_cookie(struct ep_context *ep_ctx, APTR cookie);
/* The unit task's tick: retire the TDs past their NAK deadline, and let an
 * endpoint run on that has waited too long for a halt to be cleared. */
void xhci_ep_check_timeouts(struct ep_context *ep_ctx);
/* CMD_FLUSH: give up every TD on the rings (IOERR_ABORTED).  The pending
 * queue is the caller's: xhci_ep_flush(). */
void xhci_ep_request_stop(struct ep_context *ep_ctx);
/* The device is being torn down: nothing new reaches the rings from here on;
 * what is on them is answered when the context goes. */
void xhci_ep_quiesce(struct ep_context *ep_ctx);
/* Port suspend and resume.  request_suspend: TRUE = a Stop Endpoint went out
 * and xhci_udev_suspend_stop_done() is called once the endpoint is parked.
 * The TDs stay on the rings and run on after xhci_ep_resume(). */
BOOL xhci_ep_request_suspend(struct ep_context *ep_ctx);
void xhci_ep_resume(struct ep_context *ep_ctx);
/* A successful EP0 transfer: if it was a CLEAR_FEATURE(ENDPOINT_HALT), release
 * what a halted target endpoint kept, or make the xHC's data toggle for a
 * healthy one follow the device's.  TRUE = req was handed to a Configure
 * Endpoint command that will retire it; FALSE = reply it as usual. */
BOOL xhci_ep_clear_halt_follow(struct usb_device *udev, struct xhci_xfer *req);

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

BOOL xhci_ep_complete_by_trb(struct ep_context *ep_ctx, dma_addr_t trb_addr,
                             u32 residue, BOOL short_packet,
                             struct xhci_td_completion *out, BOOL *deferred);

/* THE transfer submit entry (direct path, pending-drain, internal EP0):
 * state gate, pending-queue deferral and stream-ring selection here; TD
 * mechanics in xhci_submit_td().  The NAK timeout rides the xfer
 * (XHCI_XF_TIMEOUT + timeout_ms). */
s8 xhci_ep_submit(struct ep_context *ep_ctx, struct xhci_xfer *io);

/* Answer everything in the pending queue (never on a ring) with reply_code. */
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
/* A ring underrun / overrun event arrived for a stream that is being stopped. */
void xhci_ep_rt_iso_ring_empty(struct ep_context *ep_ctx);

/* Free a staging IN buffer back to the endpoint's per-endpoint slab. */
void xhci_ep_free_rt_iso_buffer(struct ep_context *ep_ctx, APTR data_buffer);

#endif /* __XHCI_ENDPOINT_H__ */