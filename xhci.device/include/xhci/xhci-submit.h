/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * TD construction layer (xhci-submit.c): DMA mapping/bounce buffers, TRB
 * emission for control/bulk/interrupt/iso TDs, ring-room policy and the
 * doorbell/giveback protocol.  Sits above the pure ring/segment mechanics
 * (xhci-ring.h) and below the endpoint state machine (xhci-endpoint.h),
 * which owns every submit-policy decision — xhci_ep_submit() is the only
 * caller of xhci_submit_td().
 *
 * Every function here requires ctrl->xfer_lock held; the lock sites are the
 * unit-task work blocks and the direct entries (xhci-direct.c).
 */

#ifndef __XHCI_SUBMIT_H
#define __XHCI_SUBMIT_H

#include <exec/types.h>
#include <xhci/xhci-xfer.h>

struct xhci_ctrl;
struct usb_device;
struct ep_context;
struct xhci_ring;

/* A mapped DMA buffer: either direct (bounce == NULL, cache-maintained) or
 * bounced through one of the controller's bounce slabs. */
struct xhci_dma_span
{
	APTR cpu;        /* caller's buffer */
	u32 length;
	APTR bounce;     /* bounce buffer; NULL when mapped directly */
	u8 bounce_class; /* REQ_BOUNCE_CLASS_* when bounced */
};

void xhci_dma_span_unmap(struct xhci_ctrl *ctrl, struct xhci_dma_span *span, BOOL copy_back);
void xhci_dma_unmap(struct xhci_ctrl *ctrl, struct xhci_xfer *req, BOOL copy);

/* Two-phase request map for the direct path: premap (lock held - decide
 * direct-vs-bounce, allocate) then map_sync (lock NOT required - bounce copy
 * + cache maintenance touch only caller-owned buffers).  The one-call map
 * inside the submit machinery is idempotent over a premapped request. */
dma_addr_t xhci_dma_premap(struct xhci_ctrl *ctrl, struct xhci_xfer *req, BOOL to_device);
void xhci_dma_map_sync(struct xhci_xfer *req, BOOL to_device);

/* Per-ring TRB accounting: TDs retire their TRBs as they leave the ring. */
void xhci_submit_release_trbs(struct ep_context *ep_ctx, u16 stream_id, u32 trb_count);

/* xhci_submit_td outcome.  NO_ROOM leaves the io fully intact (its DMA
 * mapping included — the map is idempotent) so the endpoint can park it on
 * the pending queue; FAILED sets io->error and *err and has released the
 * mapping. */
enum xhci_submit_status
{
	XHCI_SUBMIT_OK = 0,
	XHCI_SUBMIT_NO_ROOM,
	XHCI_SUBMIT_FAILED,
};

/* Build and hand over one TD on ep_ring: map the payload, emit the TRBs,
 * register the TD (xhci_ep_set_receiving) and give the first TRB back with
 * the doorbell.  Policy (state gate, ring selection, queue-on-busy) lives in
 * the caller, xhci_ep_submit(). */
enum xhci_submit_status xhci_submit_td(struct usb_device *udev, struct ep_context *ep_ctx,
                                       struct xhci_ring *ep_ring, struct xhci_xfer *io,
                                       u32 timeout_ms, s8 *err);

/* RT ISO TD: no request object - the TD itself carries the payload.
 * staging_in marks an IN buffer owned by the endpoint's staging slab
 * (freed on completion/teardown).  Rides the endpoint's default ring —
 * RT-ISO endpoints never have streams. */
s8 xhci_submit_rt_td(struct usb_device *udev, struct ep_context *ep_ctx,
                     APTR buffer, u32 length,
                     u16 frame, u16 dir, BOOL staging_in, BOOL defer_doorbell);

/* Room check against the endpoint's DEFAULT ring only (RT-ISO backpressure). */
BOOL xhci_submit_has_room(struct ep_context *ep_ctx, u32 needed_trbs);

/* Ring the deferred-giveback doorbell closing an RT-ISO scheduling run.
 * Default ring only: deferred doorbells are RT-ISO-only, and RT-ISO
 * endpoints never have streams. */
void xhci_submit_giveback(struct usb_device *udev, struct ep_context *ep_ctx);

/* Ring an endpoint's doorbell without touching the ring contents - restarts
 * a Stopped endpoint whose TDs are still queued (e.g. after a port resume
 * from U3).  stream_id targets one stream ring; 0 = the default ring. */
void xhci_submit_kick_ep(struct usb_device *udev, u8 ep_index, u16 stream_id);

#endif /* __XHCI_SUBMIT_H */
