// SPDX-License-Identifier: GPL-2.0-only
/*
 * TD submission layer: DMA mapping (bounce buffers for PCIe-unreachable or
 * unaligned spans), TRB emission for control/bulk/interrupt/iso TDs, the
 * ring-room policy (grow-on-demand) and the doorbell/giveback protocol.
 *
 * Sits above the pure ring/segment mechanics (xhci-ring.c, shared internals
 * in xhci-ring-priv.h) and drives the endpoint state machine
 * (xhci-endpoint.h).
 *
 * Based on the xHCI host controller driver in linux-kernel by Sarah Sharp.
 * Copyright (C) 2008 Intel Corp.  Copyright (C) 2013 Samsung Electronics.
 */

#include <config.h>
#include <debug.h>
#include <memory.h>

#include <xhci/xhci-ring.h>
#include <xhci/xhci-submit.h>
#include "xhci-ring-priv.h"

#include <xhci/xhci.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-descriptors.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci-context.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-submit] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef TRACE
#undef KprintfT
#define KprintfT(fmt, ...) PrintPistorm("[xhci-submit] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

/* Bounce-buffer copy, either direction: CopyMemQuick when everything is
 * longword-aligned, CopyMem otherwise. */
static inline void xhci_copy_aligned(CONST_APTR src, APTR dst, u32 size)
{
	if ((((uintptr_t)src | (uintptr_t)dst | size) & (sizeof(ULONG) - 1)) == 0)
	{
		CopyMemQuick((ULONG *)src, (ULONG *)dst, size);
		return;
	}

	CopyMem(src, dst, size);
}

/* Decision + allocation half of a span map (needs ctrl->xfer_lock: it touches
 * the bounce slabs and the DMA pool).  No payload copy, no cache ops - those
 * live in xhci_dma_span_sync so a caller may run them without the lock. */
static dma_addr_t xhci_dma_span_prepare(struct xhci_ctrl *ctrl, struct xhci_dma_span *span,
										APTR buffer, u32 length, BOOL to_device, BOOL owns_lines)
{
	span->cpu = buffer;
	span->length = length;
	span->bounce = NULL;
	span->bounce_class = REQ_BOUNCE_CLASS_NONE;

	if (!ctrl || !ctrl->dmaPool || !buffer || length == 0)
		return (dma_addr_t)buffer;

	/* The PCIe engine can only DMA into Emu68 (Pi-DRAM) RAM; Chip RAM and any
	 * Zorro/accelerator Fast RAM are unreachable and must be bounced.
	 * For OUT transfers (device reads RAM), CachePreDMA's clean-only path is safe
	 * on arbitrary byte ranges, so the only direct-DMA gate here is reachability.
	 * For IN transfers (device writes RAM), direct DMA is safe only when the
	 * buffer covers whole cache lines; otherwise the post-DMA invalidate would
	 * drop neighbouring boundary data.  owns_lines asserts exactly that fact for
	 * buffers whose allocator hands out line-aligned, line-rounded slots (the
	 * RT-ISO staging slab) regardless of the requested length. */
	BOOL direct_dma_safe = dma_addr_reachable(&ctrl->dma_ctx, buffer, length);
	if (!to_device && !owns_lines)
		direct_dma_safe = direct_dma_safe &&
						  (((uintptr_t)buffer & DMA_ALIGN_MIN_MASK) == 0) &&
						  ((length & DMA_ALIGN_MIN_MASK) == 0);

	if (direct_dma_safe)
		return (dma_addr_t)buffer;

	void *aligned = NULL;
	u8 bounce_class = REQ_BOUNCE_CLASS_NONE;
	if (length <= XHCI_BOUNCE_SMALL_SIZE)
	{
		aligned = slab_alloc(&ctrl->bounce_small);
		bounce_class = REQ_BOUNCE_CLASS_SMALL;
	}
	else if (length <= XHCI_BOUNCE_MED_SIZE)
	{
		aligned = slab_alloc(&ctrl->bounce_med);
		bounce_class = REQ_BOUNCE_CLASS_MED;
	}
	else if (length <= XHCI_BOUNCE_LARGE_SIZE)
	{
		aligned = slab_alloc(&ctrl->bounce_large);
		bounce_class = REQ_BOUNCE_CLASS_LARGE;
	}
	if (!aligned)
	{
		aligned = dma_alloc(ctrl->dmaPool, DMA_ALIGN_MIN, length);
		bounce_class = REQ_BOUNCE_CLASS_NONE;
	}
	if (!aligned)
	{
		/* No bounce buffer for an unreachable/unaligned span: fail the map (0)
		 * so the caller rejects the transfer. Returning the original pointer
		 * would let DMA proceed against unreachable memory and silently corrupt
		 * it. A successful map never yields 0 (buffer/aligned are non-NULL). */
		Kprintf("failed to allocate bounce buffer for %lx len=%lu; rejecting transfer\n", (ULONG)buffer, (ULONG)length);
		return 0;
	}

	span->bounce = aligned;
	span->bounce_class = bounce_class;
	return (dma_addr_t)aligned;
}

/* Payload half of a span map: the bounce copy (OUT) and the pre-DMA cache
 * maintenance.  Touches only the caller-owned buffers - safe WITHOUT
 * ctrl->xfer_lock.
 *
 * IN spans pre-arm with DMA_WriteToRAM (invalidate): the destination's CPU
 * content is disposable - the device overwrites it - so dirty lines (a bounce
 * slot last used for OUT, a reused user buffer) are dropped instead of written
 * back to DRAM just before being replaced.  Whole-line ownership is
 * guaranteed by the direct-DMA gate / owns_lines in span_prepare, and by the
 * line-owned bounce slabs.  On a short IN, bytes past the actual length are
 * now whatever RAM held rather than the cleaned-out CPU content - both are
 * undefined territory per USB semantics (class drivers honour ioActual). */
static void xhci_dma_span_sync(struct xhci_dma_span *span, BOOL to_device)
{
	if (!span->cpu || span->length == 0)
		return;

	if (span->bounce)
	{
		if (to_device)
			xhci_copy_aligned(span->cpu, span->bounce, span->length);
		cache_pre_dma(span->bounce, span->length, to_device ? DMA_ReadFromRAM : DMA_WriteToRAM);
	}
	else
		cache_pre_dma(span->cpu, span->length, to_device ? DMA_ReadFromRAM : DMA_WriteToRAM);
}

static dma_addr_t xhci_dma_span_map(struct xhci_ctrl *ctrl, struct xhci_dma_span *span,
									APTR buffer, u32 length, BOOL to_device, BOOL owns_lines)
{
	dma_addr_t mapped = xhci_dma_span_prepare(ctrl, span, buffer, length, to_device, owns_lines);
	if (mapped)
		xhci_dma_span_sync(span, to_device);
	return mapped;
}

void xhci_dma_span_unmap(struct xhci_ctrl *ctrl, struct xhci_dma_span *span, BOOL copy_back)
{
	if (!ctrl || !span->cpu || span->length == 0)
		return;

	if (!span->bounce)
	{
		if (copy_back)
			cache_post_dma(span->cpu, span->length, 0);
		return;
	}

	if (copy_back)
	{
		cache_post_dma(span->bounce, span->length, 0);
		xhci_copy_aligned(span->bounce, span->cpu, span->length);
	}

	switch (span->bounce_class)
	{
	case REQ_BOUNCE_CLASS_SMALL:
		slab_free(&ctrl->bounce_small, span->bounce);
		break;
	case REQ_BOUNCE_CLASS_MED:
		slab_free(&ctrl->bounce_med, span->bounce);
		break;
	case REQ_BOUNCE_CLASS_LARGE:
		slab_free(&ctrl->bounce_large, span->bounce);
		break;
	default:
		dma_free(ctrl->dmaPool, span->bounce);
		break;
	}
	span->bounce = NULL;
	span->bounce_class = REQ_BOUNCE_CLASS_NONE;
}

/* xfer DMA adapters: keep the bounce bookkeeping on the request.
 * The map is split so the direct path can run the payload work (bounce copy +
 * cache maintenance) without ctrl->xfer_lock: premap (locked: decide + alloc)
 * -> map_sync (unlocked: copy + cache ops).  xhci_dma_map remains the
 * one-call form and is idempotent - a premapped or requeued request returns
 * its existing mapping. */
dma_addr_t xhci_dma_premap(struct xhci_ctrl *ctrl, struct xhci_xfer *req, BOOL to_device)
{
	struct xhci_dma_span span;
	dma_addr_t mapped = xhci_dma_span_prepare(ctrl, &span, req->data, req->data_length, to_device, FALSE);
	if (!mapped)
		return 0;

	if (span.bounce)
	{
		req->priv_flags = (req->priv_flags & ~REQ_BOUNCE_CLASS_MASK) |
						  ((u32)span.bounce_class << REQ_BOUNCE_CLASS_SHIFT) | REQ_DMA_MAPPED;
		req->dma_address = span.bounce;
	}
	else if (req->data_length)
	{
		req->priv_flags |= REQ_DMA_MAPPED | REQ_DMA_DIRECT;
		req->dma_address = req->data;
	}
	return mapped;
}

/* Rebuild the span from the request's bookkeeping (no shared state - callable
 * without the lock). */
static void req_to_span(const struct xhci_xfer *req, struct xhci_dma_span *span)
{
	span->cpu = req->data;
	span->length = req->data_length;
	span->bounce = ((req->priv_flags & REQ_DMA_MAPPED) && !(req->priv_flags & REQ_DMA_DIRECT))
					   ? req->dma_address
					   : NULL;
	span->bounce_class = (u8)((req->priv_flags & REQ_BOUNCE_CLASS_MASK) >> REQ_BOUNCE_CLASS_SHIFT);
}

void xhci_dma_map_sync(struct xhci_xfer *req, BOOL to_device)
{
	struct xhci_dma_span span;
	req_to_span(req, &span);
	xhci_dma_span_sync(&span, to_device);
}

inline static dma_addr_t xhci_dma_map(struct xhci_ctrl *ctrl, struct xhci_xfer *req, BOOL copy)
{
	if (!req)
		return 0;

	/* Idempotent: a premapped direct submit, or a request requeued by the
	 * ring-room policy, keeps its mapping (and its already-copied bounce
	 * payload) - reuse it. */
	if (req->priv_flags & REQ_DMA_MAPPED)
		return (dma_addr_t)req->dma_address;

	dma_addr_t mapped = xhci_dma_premap(ctrl, req, copy);
	if (mapped)
		xhci_dma_map_sync(req, copy);
	return mapped;
}

void xhci_dma_unmap(struct xhci_ctrl *ctrl, struct xhci_xfer *req, BOOL copy)
{
	if (!ctrl || !req)
		return;

	struct xhci_dma_span span;
	req_to_span(req, &span);

	xhci_dma_span_unmap(ctrl, &span, copy);

	req->priv_flags &= (u32) ~(REQ_DMA_MAPPED | REQ_DMA_DIRECT | REQ_BOUNCE_CLASS_MASK);
	req->dma_address = NULL;
}

inline static BOOL ring_has_room(struct xhci_ring *ring, u32 needed)
{
	/* ring->queued_trbs counts THIS ring's in-flight TDs, so a streams
	 * endpoint sizes each stream ring by its own load instead of the
	 * endpoint-wide total. */
	u32 capacity = ring->num_segs * (TRBS_PER_SEGMENT - 1);
	if (capacity == 0)
		return FALSE;
	return ring->queued_trbs + needed <= capacity;
}

BOOL xhci_submit_has_room(struct ep_context *ep_ctx, u32 needed_trbs)
{
	if (!ep_ctx)
		return FALSE;
	struct xhci_ring *ring = xhci_ep_get_ring(ep_ctx);
	if (!ring)
		return FALSE;
	return ring_has_room(ring, needed_trbs);
}

/* Retire trb_count TRBs from the accounting of the ring serving stream_id
 * (the TD tracker calls this as TDs leave the ring). */
void xhci_submit_release_trbs(struct ep_context *ep_ctx, u16 stream_id, u32 trb_count)
{
	struct xhci_ring *ring = xhci_ep_get_ring_for_stream(ep_ctx, stream_id);
	if (!ring)
		return;
	ring->queued_trbs = (ring->queued_trbs >= trb_count) ? ring->queued_trbs - trb_count : 0;
}

inline static void prime_first_trb(struct xhci_generic_trb *start_trb)
{
	start_trb->field[3] ^= le32(TRB_CYCLE);
	/* the TD's closing barrier: every TRB of the TD was flushed with
	 * DMAF_NoSync (xhci_ring_enqueue_trb_ns); this non-NoSync clean
	 * completes them all before the doorbell */
	cache_pre_dma(start_trb, sizeof(struct xhci_generic_trb), DMA_ReadFromRAM);
}

/* Ring an endpoint's doorbell without touching the ring contents - used to
 * restart a Stopped endpoint whose TDs are still queued (e.g. after a port
 * resume from U3).  stream_id targets one stream ring; 0 = the default ring. */
void xhci_submit_kick_ep(struct usb_device *udev, u8 ep_index, u16 stream_id)
{
	xhci_db_ring(udev->controller->dba, udev->slot_id, DB_VALUE(ep_index, stream_id));
}

inline static void giveback_first_trb(struct usb_device *udev, struct xhci_ring *ring,
									  struct xhci_generic_trb *start_trb)
{
	prime_first_trb(start_trb);
	xhci_submit_kick_ep(udev, ring->ep_index, ring->stream_id);
}

void xhci_submit_giveback(struct usb_device *udev, struct ep_context *ep_ctx)
{
	if (!ep_ctx)
		return;

	struct xhci_ring *ring = xhci_ep_get_ring(ep_ctx);
	if (ring && ring->deferred_giveback)
	{
		giveback_first_trb(udev, ring, ring->deferred_giveback);
		ring->deferred_giveback = NULL;
	}
}

inline static dma_addr_t xhci_ring_enqueue_setup_trb(struct xhci_ring *ep_ring, struct xhci_xfer *io)
{
	/* Queue setup TRB - see section 6.4.1.2.1 */
	u32 field3 = TRB_IDT | TRB_TYPE(TRB_SETUP);
	/*
	 * Don't give the first TRB to the hardware (by toggling the cycle bit)
	 * until we've finished creating all the other TRBs.  The ring's cycle
	 * state may change as we enqueue the other TRBs, so save it too.
	 */
	if (ep_ring->cycle_state == 0)
		field3 |= 0x1;

	/* xHCI 1.0 6.4.1.2.1: Transfer Type field */
	if (io->data_length > 0)
	{
		if (io->setup.usd_RequestType & USB_DIR_IN)
			field3 |= TRB_TX_TYPE(TRB_DATA_IN);
		else
			field3 |= TRB_TX_TYPE(TRB_DATA_OUT);
	}

	return xhci_ring_enqueue_trb_ns(ep_ring, TRUE,
								 io->setup.usd_RequestType | ((u32)io->setup.usd_Request << 8) | ((u32)le16(io->setup.usd_Value) << 16), /* field 0 */
								 le16(io->setup.usd_Index) | ((u32)le16(io->setup.usd_Length) << 16),									 /* field 1 */
								 TRB_LEN(8) | TRB_INTR_TARGET(0),																 /* field 2 */
								 field3);																						 /* field 3 */
}

inline static dma_addr_t xhci_ring_enqueue_status_trb(struct xhci_ring *ep_ring, struct xhci_xfer *io)
{
	/*
	 * Queue status TRB -
	 * see Table 7 and sections 4.11.2.2 and 6.4.1.2.3
	 */

	/* If the device sent data, the status stage is an OUT transfer */
	u32 field3 = (io->data_length > 0 && io->setup.usd_RequestType & USB_DIR_IN) ? 0 : TRB_DIR_IN;

	/* Event on completion */
	field3 |= TRB_IOC | TRB_TYPE(TRB_STATUS) | ep_ring->cycle_state;

	return xhci_ring_enqueue_trb_ns(ep_ring, FALSE,
								 0,					 /* field 0 */
								 0,					 /* field 1 */
								 TRB_INTR_TARGET(0), /* field 2 */
								 field3);			 /* field 3 */
}

/*
 * For xHCI 1.0 host controllers, TD size is the number of max packet sized
 * packets remaining in the TD (*not* including this TRB).
 *
 * Total TD packet count = total_packet_count =
 *     ceil(TD size in bytes / wMaxPacketSize)
 *
 * Packets transferred up to and including this TRB = packets_transferred =
 *     rounddown(total bytes transferred including this TRB / wMaxPacketSize)
 *
 * TD size = total_packet_count - packets_transferred
 *
 * For xHCI 0.96 and older, TD size field should be the remaining bytes
 * including this TRB, right shifted by 10
 *
 * For all hosts it must fit in bits 21:17, so it can't be bigger than 31.
 * This is taken care of in the TRB_TD_SIZE() macro
 *
 * The last TRB in a TD must have the TD size set to zero.
 *
 * @param transferred	total size sent so far
 * @param trb_buff_len	length of the TRB Buffer
 * @param td_total_len	total packet count
 * @param maxp	max packet size of current pipe
 * @param more_trbs_coming	indicate last trb in TD
 * Return: remainder
 */
inline static u32 xhci_td_remainder(u32 transferred,
									u32 trb_buff_len, u32 td_total_len,
									u32 maxp, BOOL more_trbs_coming)
{
	/* One TRB with a zero-length data packet. */
	if (!more_trbs_coming || (transferred == 0 && trb_buff_len == 0) ||
		trb_buff_len == td_total_len)
		return 0;

	u32 total_packet_count = DIV_CEIL(td_total_len, maxp);

	/* Queueing functions don't count the current TRB into transferred */
	return (total_packet_count - ((transferred + trb_buff_len) / maxp));
}

/**
 * Emit the (possibly chained) data TRBs of a TD, splitting at 64 KB boundaries
 * (xHCI Table 49 / 6.4.1: a TRB buffer must not span one).
 *
 * @param ep_ring          Endpoint transfer ring to enqueue onto.
 * @param addr             Bus address of the data buffer; advanced by each
 *                         TRB's chunk length as the chain is emitted.
 * @param length           Total byte length of the transfer across the whole TD.
 * @param num_trbs         Number of TRBs in the chain, pre-computed by
 *                         xhci_ring_calc_data_trbs() to honour the 64 KB split.
 *                         Drives chaining (>1) vs. a lone IOC TRB and feeds the
 *                         TD-size remainder field.
 * @param trb_buff_len     Byte length of the *first* TRB - addr up to the next
 *                         64 KB boundary (clamped to length).  Subsequent chunk
 *                         lengths are recomputed here, capped at 64 KB
 *                         (TRB_MAX_BUFF_SIZE).
 * @param td_trb_addrs     Output array of num_trbs entries; each receives the
 *                         ring address of its emitted TRB (used to finalize the
 *                         first TRB and for TD teardown/cancellation).
 * @param first_type_bits  TRB type (+ extras) for the first TRB: NORMAL,
 *                         ISOC | iso bits, or DATA | DIR for a control data
 *                         stage.  Chain TRBs are always NORMAL.
 * @param isp              TRB_ISP or 0; applied to chained TRBs, and - inside
 *                         a control TD - to the last data TRB as well.
 * @param within_td        TRUE for a control data stage: the setup TRB owns
 *                         the deferred cycle-bit giveback, the status TD
 *                         carries the IOC, and more TRBs follow the last one.
 */
inline static void xhci_ring_enqueue_data_trbs(struct xhci_ring *ep_ring, dma_addr_t addr, u32 length,
											   u32 num_trbs, u32 trb_buff_len,
											   dma_addr_t *td_trb_addrs,
											   u32 first_type_bits, u32 isp, BOOL within_td)
{
	const u32 chain_trb_type_bits = TRB_TYPE(TRB_NORMAL);

	u32 running_total = 0;
	u32 td_trb_index = 0;

	if (trb_buff_len > length)
		trb_buff_len = length;

	BOOL first_trb = TRUE;

	/* Queue each TRB, chaining when necessary. Queue the first TRB even if
	 * it's zero-length. */
	do
	{
		u32 field3;
		if (first_trb)
		{
			field3 = first_type_bits;
			first_trb = FALSE;
			if (within_td)
				field3 |= ep_ring->cycle_state; /* setup TRB holds the giveback */
			else if (ep_ring->cycle_state == 0)
				field3 |= TRB_CYCLE; /* inverted: given back in finalize */
		}
		else
		{
			field3 = chain_trb_type_bits | ep_ring->cycle_state;
		}

		/*
		 * Chain all the TRBs together; the last TRB ends the chain and - for
		 * a standalone TD - raises the completion interrupt.
		 */
		if (num_trbs > 1)
			field3 |= TRB_CHAIN | isp;
		else
			field3 |= within_td ? isp : TRB_IOC;

		u32 remainder = xhci_td_remainder(running_total, trb_buff_len,
										  length, ep_ring->max_packet_size,
										  num_trbs > 1);

		u32 length_field = TRB_LEN(trb_buff_len) | TRB_TD_SIZE(remainder) | TRB_INTR_TARGET(0);

		dma_addr_t trb_addr = xhci_ring_enqueue_trb_ns(ep_ring, within_td || (num_trbs > 1),
													addr,			/* field 0 (dma_addr_t is 32-bit) */
													0,				/* field 1 */
													length_field,	/* field 2 */
													field3);		/* field 3 */
		td_trb_addrs[td_trb_index++] = trb_addr;
		--num_trbs;
		running_total += trb_buff_len;

		/* Calculate length for next transfer */
		addr += trb_buff_len;
		trb_buff_len = (length - running_total < TRB_MAX_BUFF_SIZE) ? (length - running_total) : TRB_MAX_BUFF_SIZE;
	} while (running_total < length);
}

inline static void xhci_ring_enqueue_control_trbs(struct xhci_ring *ep_ring, struct xhci_xfer *io, dma_addr_t addr, u32 num_trbs, u32 trb_buff_len, dma_addr_t *td_trb_addrs)
{
	td_trb_addrs[0] = xhci_ring_enqueue_setup_trb(ep_ring, io);

	if (io->data_length > 0)
	{
		BOOL in = (io->setup.usd_RequestType & USB_DIR_IN) != 0;
		xhci_ring_enqueue_data_trbs(ep_ring, addr, io->data_length,
									num_trbs - 2, trb_buff_len, &td_trb_addrs[1],
									TRB_TYPE(TRB_DATA) | (in ? TRB_DIR_IN : 0),
									in ? TRB_ISP : 0, TRUE);
	}

	td_trb_addrs[num_trbs - 1] = xhci_ring_enqueue_status_trb(ep_ring, io);
}

inline static void xhci_ring_enqueue_non_control_trbs(struct xhci_ring *ep_ring, struct xhci_xfer *io, dma_addr_t addr, u32 num_trbs, u32 trb_buff_len, dma_addr_t *td_trb_addrs, u32 iso_extra_bits)
{
	const BOOL is_iso = io->type == UHCD_EPTYPE_ISO;

	/* xHCI 4.11.2.3: only the first TRB in an ISO TD carries the ISOC type and
	 * iso-specific bits (Frame ID/SIA, TBC, TLBPC). Chain TRBs are NORMAL,
	 * without ISP. */
	xhci_ring_enqueue_data_trbs(ep_ring, addr, io->data_length,
								num_trbs, trb_buff_len, td_trb_addrs,
								is_iso ? (TRB_TYPE(TRB_ISOC) | iso_extra_bits) : TRB_TYPE(TRB_NORMAL),
								(io->direction == XHCI_DIR_IN && !is_iso) ? TRB_ISP : 0,
								FALSE);
}

inline static void xhci_ring_finalize_first_trb(struct usb_device *udev, struct xhci_ring *ep_ring, struct xhci_generic_trb *start_trb, BOOL defer_doorbell)
{
	/* Hand the first TRB back to the controller once the TD is ready. */
	if (defer_doorbell)
	{
		/* First TD of a run: record TRB for later doorbell */
		if (!ep_ring->deferred_giveback)
			ep_ring->deferred_giveback = start_trb;
		else
			/* Additional TDs: make TRBs visible now but do not ring the doorbell*/
			prime_first_trb(start_trb);
	}
	else
		giveback_first_trb(udev, ep_ring, start_trb);
}

/*
 * How many TRBs does a data buffer need, and how long is the first one?
 * XHCI Spec (Table 49 / 6.4.1) requires we avoid spanning the 64KB boundary,
 * so we may need several chained TRBs if the buffer crosses it.
 */
inline static u32 xhci_ring_calc_data_trbs(dma_addr_t addr, u32 length, u32 *trb_buff_len)
{
	u32 running_total = TRB_MAX_BUFF_SIZE - ((u32)addr & (TRB_MAX_BUFF_SIZE - 1));
	*trb_buff_len = running_total;
	running_total &= TRB_MAX_BUFF_SIZE - 1;

	u32 num_trbs = 0;
	/* If we already cover bytes in this 64KB chunk, or the transfer is zero length,
	 * we schedule at least one TRB now.
	 */
	if (running_total != 0 || length == 0)
		num_trbs++;

	/* Account for remaining 64KB windows, adding more TRBs as needed. */
	num_trbs += DIV_CEIL(length - running_total, TRB_MAX_BUFF_SIZE);
	return num_trbs;
}

inline static u32 xhci_ring_calc_num_trbs(struct xhci_ctrl *ctrl, struct xhci_xfer *io, u32 *trb_buff_len, dma_addr_t *addr)
{
	if (io->type == UHCD_EPTYPE_CONTROL)
	{
		/* Setup and status TDs, plus a chained data stage like any other
		 * transfer (direction comes from the setup packet). */
		if (io->data_length == 0)
			return 2;
		BOOL in = (io->setup.usd_RequestType & USB_DIR_IN) != 0;
		*addr = xhci_dma_map(ctrl, io, !in);
		return 2 + xhci_ring_calc_data_trbs(*addr, io->data_length, trb_buff_len);
	}

	*addr = xhci_dma_map(ctrl, io, io->direction == XHCI_DIR_OUT);
	return xhci_ring_calc_data_trbs(*addr, io->data_length, trb_buff_len);
}

#ifdef DEBUG
static void __attribute__((unused)) xhci_dump_request(const char *tag, const struct xhci_xfer *req)
{
	if (!req)
		return;

	const char *pfx = tag ? tag : "";

	Kprintf("%s Request dump:\n", pfx);
	Kprintf("%s  Endpoint=0x%02lx Dir=%s Type=%s\n",
			pfx, (ULONG)req->endpoint,
			(req->direction == XHCI_DIR_IN) ? "IN" : "OUT",
			(req->type == UHCD_EPTYPE_CONTROL) ? "Control" : (req->type == UHCD_EPTYPE_BULK)			 ? "Bulk"
																   : (req->type == UHCD_EPTYPE_INTERRUPT)			 ? "Interrupt"
																   : (req->type == UHCD_EPTYPE_ISO)		 ? "Isochronous"
																															 : "Unknown");
	if (req->type == UHCD_EPTYPE_CONTROL)
		Kprintf("%s  SetupData: bmRequestType=0x%02lx bRequest=0x%02lx wValue=0x%04lx wIndex=0x%04lx wLength=%lu\n",
				pfx, (ULONG)req->setup.usd_RequestType, (ULONG)req->setup.usd_Request,
				(ULONG)le16(req->setup.usd_Value), (ULONG)le16(req->setup.usd_Index),
				(ULONG)le16(req->setup.usd_Length));
}
#endif /* DEBUG (xhci_dump_request) */

/*
 * xHCI 4.11.2.3: compute TBC and TLBPC for an ISO TD.
 * Pre-1.0 controllers leave both fields RsvdZ; older controllers can't burst anyway.
 */
static inline u32 iso_burst_bits(struct usb_device *udev,
								 struct ep_context *ep_ctx, u32 td_length)
{
	if (udev->controller->hci_version < 0x100)
		return 0;

	u32 max_packet = xhci_ep_get_max_packet_size(ep_ctx);
	if (max_packet == 0)
		max_packet = 1;
	u32 total_pkts = (td_length + max_packet - 1) / max_packet;
	if (total_pkts == 0)
		total_pkts = 1;

	if (udev->speed >= USB_SPEED_SUPER)
	{
		const u32 max_burst = xhci_ep_get_max_burst(ep_ctx);
		const u32 burst = max_burst + 1U;
		const u32 tbc = ((total_pkts + burst - 1) / burst) - 1;
		const u32 residue = total_pkts % burst;
		const u32 tlbpc = (residue == 0) ? max_burst : (residue - 1);
		return TRB_TBC(tbc) | TRB_TLBPC(tlbpc);
	}
	/* USB 2.0 / 1.1: one burst per service interval; TLBPC = total_pkts - 1. */
	return TRB_TBC(total_pkts - 1);
}

enum td_reserve_status
{
	TD_RESERVE_OK,
	TD_RESERVE_NO_ROOM,
	TD_RESERVE_NO_MEM,
};

/* Ring-room policy and TRB-address bookkeeping shared by the request and
 * RT-ISO submit paths: room check with one grow attempt and recheck, the
 * td_trb_addrs allocation, and ring preparation.  Failure *disposition*
 * (queue-for-later vs unmap-and-reject) stays with the caller. */
inline static enum td_reserve_status td_reserve_and_prepare(struct xhci_ctrl *ctrl,
															struct xhci_ring *ep_ring,
															u32 num_trbs,
															dma_addr_t **td_trb_addrs_out)
{
	if (!ring_has_room(ep_ring, num_trbs + 1))
	{
		KprintfT("Ring full, needed %lu TRBs, attempting grow\n", (ULONG)num_trbs);
		if (!xhci_ring_grow(ctrl, ep_ring, XHCI_SEGMENTS_PER_RING) ||
			!ring_has_room(ep_ring, num_trbs + 1))
			return TD_RESERVE_NO_ROOM;
	}

	dma_addr_t *td_trb_addrs = xhci_td_trb_addrs_alloc(ctrl, num_trbs);
	if (!td_trb_addrs)
	{
		Kprintf("Failed to alloc TD TRB list\n");
		return TD_RESERVE_NO_MEM;
	}

	/* Walk the enqueue pointer off any link TRB so TRB emission starts on a
	 * data slot with the correct cycle state. */
	prepare_ring(ep_ring);
	ep_ring->queued_trbs += num_trbs; /* released by the TD tracker at retire */
	*td_trb_addrs_out = td_trb_addrs;
	return TD_RESERVE_OK;
}

/* Build and hand over one TD (io transfer path).  Policy — state gate, ring
 * selection, queue-on-busy — lives in xhci_ep_submit(); this is pure
 * mechanics: map, emit, register (xhci_ep_set_receiving), give back.  Non-RT
 * iso TDs use SIA scheduling. */
enum xhci_submit_status xhci_submit_td(struct usb_device *udev, struct ep_context *ep_ctx,
                                       struct xhci_ring *ep_ring, struct xhci_xfer *io,
                                       u32 timeout_ms, s8 *err)
{
	struct xhci_ctrl *ctrl = udev->controller;

#ifdef DEBUG_CONTEXT
	xhci_dump_slot_ctx("[xhci-submit] xhci_submit_td:", udev, FALSE);
	xhci_dump_ep_ctx("[xhci-submit] xhci_submit_td:", udev, io->endpoint);
#endif

	u32 trb_buff_len = 0; // non-control only
	dma_addr_t addr = 0;  // non-control only
	PERF_T0(map_t0);
	u32 num_trbs = xhci_ring_calc_num_trbs(ctrl, io, &trb_buff_len, &addr);
	PERF_ADD(&ctrl->perf, XP_SUBMIT_MAP, map_t0);

	/* A zero DMA address with data present means xhci_dma_map failed (no bounce
	 * buffer for an unreachable/unaligned span); reject rather than DMA to a bad
	 * address. Nothing was queued or allocated yet, so returning is clean. */
	if (io->data_length && !addr)
	{
		Kprintf("DMA map failed for ep %lu len %lu; rejecting transfer\n",
				(ULONG)io->endpoint, (ULONG)io->data_length);
		*err = io->error = UHIOERR_OUTOFMEMORY;
		return XHCI_SUBMIT_FAILED;
	}

	PERF_T0(emit_t0);
	dma_addr_t *td_trb_addrs;
	switch (td_reserve_and_prepare(ctrl, ep_ring, num_trbs, &td_trb_addrs))
	{
	case TD_RESERVE_NO_ROOM:
		/* The io keeps its mapping (and its already-copied bounce payload)
		 * while queued; xhci_dma_map's idempotence reuses it on resubmit. */
		KprintfT("No ring room for ep=%lu, deferring\n", (ULONG)io->endpoint);
		return XHCI_SUBMIT_NO_ROOM;
	case TD_RESERVE_NO_MEM:
		xhci_dma_unmap(ctrl, io, FALSE);
		*err = io->error = UHIOERR_OUTOFMEMORY;
		return XHCI_SUBMIT_FAILED;
	default:
		break;
	}

	if (io->type == UHCD_EPTYPE_CONTROL)
		xhci_ring_enqueue_control_trbs(ep_ring, io, addr, num_trbs, trb_buff_len, td_trb_addrs);
	else
	{
		u32 iso_extra_bits = 0;
		if (io->type == UHCD_EPTYPE_ISO)
			iso_extra_bits = TRB_SIA | iso_burst_bits(udev, ep_ctx, io->data_length);
		xhci_ring_enqueue_non_control_trbs(ep_ring, io, addr, num_trbs, trb_buff_len, td_trb_addrs, iso_extra_bits);
	}

	if (!xhci_ep_set_receiving(ep_ctx, io, td_trb_addrs, timeout_ms, num_trbs))
	{
		/* The endpoint is FAILED and td_trb_addrs is freed; the emitted TRBs
		 * are never handed over (the first TRB's cycle bit stays inverted) and
		 * the recovery flush re-arms the ring past them. */
		ep_ring->queued_trbs = (ep_ring->queued_trbs >= num_trbs) ? ep_ring->queued_trbs - num_trbs : 0;
		xhci_dma_unmap(ctrl, io, FALSE);
		*err = io->error = UHIOERR_OUTOFMEMORY;
		return XHCI_SUBMIT_FAILED;
	}
	xhci_ring_finalize_first_trb(udev, ep_ring, (struct xhci_generic_trb *)td_trb_addrs[0], FALSE);
	PERF_ADD(&ctrl->perf, XP_SUBMIT_EMIT, emit_t0);

	return XHCI_SUBMIT_OK;
}

/*
 * RT ISO TD submission: no request object - the TD itself owns the mapped
 * span and carries {frame, dir, staging}.  Mirrors the data-stage portion of
 * xhci_submit_td for a single ISOC TD; the doorbell is deferred to the
 * scheduler's per-run xhci_submit_giveback() when defer_doorbell is set.
 */
s8 xhci_submit_rt_td(struct usb_device *udev, struct ep_context *ep_ctx,
                     APTR buffer, u32 length,
                     u16 frame, u16 dir, BOOL staging_in, BOOL defer_doorbell)
{
	struct xhci_ctrl *ctrl = udev->controller;

	struct xhci_ring *ep_ring = xhci_ep_get_ring(ep_ctx);
	if (!ep_ring)
		return UHIOERR_HOSTERROR;

	struct xhci_dma_span span;
	/* staging-slab buffers own whole cache lines by construction, so an IN
	 * map never bounces on a non-line-multiple max packet */
	dma_addr_t addr = xhci_dma_span_map(ctrl, &span, buffer, length,
										dir == XHCI_DIR_OUT, staging_in);
	if (length && !addr)
	{
		/* map failed (no bounce for an unreachable/unaligned span): nothing was
		 * allocated or flushed, so reject without unmapping. */
		Kprintf("RT-ISO DMA map failed len %lu; rejecting transfer\n", (ULONG)length);
		return UHIOERR_OUTOFMEMORY;
	}

	u32 trb_buff_len = 0;
	u32 num_trbs = xhci_ring_calc_data_trbs(addr, length, &trb_buff_len);

	dma_addr_t *td_trb_addrs;
	switch (td_reserve_and_prepare(ctrl, ep_ring, num_trbs, &td_trb_addrs))
	{
	case TD_RESERVE_NO_ROOM:
		xhci_dma_span_unmap(ctrl, &span, FALSE);
		return UHIOERR_HOSTERROR;
	case TD_RESERVE_NO_MEM:
		xhci_dma_span_unmap(ctrl, &span, FALSE);
		return UHIOERR_OUTOFMEMORY;
	default:
		break;
	}

	/* CFC controllers pin the TD to its Frame ID; without CFC the hardware
	 * schedules ASAP (SIA). */
	u32 iso_bits = (ctrl->cfc_supported ? TRB_FRAME_ID(frame) : TRB_SIA) |
				   iso_burst_bits(udev, ep_ctx, length);
	xhci_ring_enqueue_data_trbs(ep_ring, addr, length, num_trbs, trb_buff_len, td_trb_addrs,
								TRB_TYPE(TRB_ISOC) | iso_bits, 0, FALSE);

	if (!xhci_ep_set_receiving_rt(ep_ctx, &span, frame, dir, staging_in, td_trb_addrs, num_trbs))
	{
		/* the endpoint is FAILED now (set_receiving_rt freed td_trb_addrs);
		 * the span is still this function's to release */
		ep_ring->queued_trbs = (ep_ring->queued_trbs >= num_trbs) ? ep_ring->queued_trbs - num_trbs : 0;
		xhci_dma_span_unmap(ctrl, &span, FALSE);
		return UHIOERR_OUTOFMEMORY;
	}

	xhci_ring_finalize_first_trb(udev, ep_ring, (struct xhci_generic_trb *)td_trb_addrs[0], defer_doorbell);
	return UHIOERR_NO_ERROR;
}
