// SPDX-License-Identifier: GPL-2.0-only
/*
 * TD submission layer: TRB emission for control/bulk/interrupt/iso TDs over
 * the DMA segments of a mapped buffer (xhci-dma.c), the ring-room policy
 * (grow-on-demand) and the doorbell/giveback protocol.
 *
 * Sits above the pure ring/segment mechanics (xhci-ring.c, shared internals
 * in xhci-ring-priv.h) and drives the endpoint state machine
 * (xhci-endpoint.h).
 *
 * Based on the xHCI host controller driver in linux-kernel by Sarah Sharp.
 * Copyright (C) 2008 Intel Corp.  Copyright (C) 2013 Samsung Electronics.
 */

#define __NOLIBBASE__
#define EXEC_BASE_NAME SysBase /* a local in every function, from its context's sysBase */
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

BOOL xhci_submit_has_room(struct ep_context *ep_ctx, u32 needed_trbs)
{
	if (!ep_ctx)
		return FALSE;
	struct xhci_ring *ring = xhci_ep_get_ring(ep_ctx);
	if (!ring)
		return FALSE;
	return xhci_ring_td_fits(ring, needed_trbs, xhci_ep_ring_oldest_trb(ring));
}

inline static void prime_first_trb(struct xhci_ring *ring, struct xhci_generic_trb *start_trb)
{
	struct ExecBase *SysBase = ring->sysBase;
	start_trb->field[3] ^= le32(TRB_CYCLE);
	/* the TD's closing barrier: every TRB of the TD was flushed with
	 * DMAF_NoSync (xhci_ring_enqueue_trb_ns); this non-NoSync clean
	 * completes them all before the doorbell */
	cache_pre_dma(start_trb, sizeof(struct xhci_generic_trb), DMA_ReadFromRAM);
}

/* Ring the doorbell for one transfer ring without touching its contents: the
 * controller (re)starts working on what is queued there.  stream_id is 0 on a
 * default ring. */
void xhci_submit_ring_doorbell(struct usb_device *udev, struct xhci_ring *ring)
{
	xhci_db_ring(udev->controller->dba, udev->slot_id, DB_VALUE(ring->ep_index, ring->stream_id));
}

inline static void giveback_first_trb(struct usb_device *udev, struct xhci_ring *ring,
									  struct xhci_generic_trb *start_trb)
{
	prime_first_trb(ring, start_trb);
	xhci_submit_ring_doorbell(udev, ring);
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

/* How xhci_ring_enqueue_data_trbs() shapes a TD's data TRBs.  Callers name
 * only what they need; the rest (0, FALSE, NULL) is the plain case. */
struct data_trb_opts
{
	u32 first_type_bits;		 /* TRB type (+ extras) of the first TRB: NORMAL,
								  * ISOC | iso bits, or DATA | DIR for a control
								  * data stage; chained TRBs are NORMAL */
	u32 isp;					 /* TRB_ISP or 0: on chained TRBs, and inside a
								  * control TD on the last data TRB too */
	u32 ioc_bits;				 /* TRB_BEI or 0: goes with the IOC onto the TRB
								  * that ends a standalone TD */
	BOOL within_td;				 /* a control data stage: the setup TRB holds the
								  * deferred cycle-bit giveback, the status TRB
								  * the IOC, and more TRBs follow the last one */
	struct xhci_dma_span *align; /* align the TD at Link TRBs; the span is the
								  * TD's data, and holds any link piece taken
								  * for it.  NULL: no alignment */
	struct xhci_ctrl *ctrl;		 /* needed only with align: the controller whose
								  * bounce_link pool supplies the link-piece
								  * slot */
};

/* A read position in a TD's DMA extents. */
struct extent_cursor
{
	const struct xhci_dma_extent *extents;
	u32 n_extents;
	u32 ext; /* the extent it is in */
	u32 off; /* bytes of that extent behind it */
};

/**
 * extent_cursor_advance() - Move a cursor on past bytes taken by a TRB.
 * @cur: the cursor.
 * @step: bytes to move on; a link piece can take them from two extents.
 *
 * The cursor leaves every extent it reaches the end of, except the last.
 */
static inline void extent_cursor_advance(struct extent_cursor *cur, u32 step)
{
	cur->off += step;
	while (cur->off >= cur->extents[cur->ext].length && cur->ext + 1 < cur->n_extents)
	{
		cur->off -= cur->extents[cur->ext].length;
		cur->ext++;
	}
}

/**
 * extent_cursor_trb_buf() - The buffer of the next TRB.
 * @cur: the cursor.
 *
 * Return: the rest of the cursor's extent, up to the next 64 KB address
 * boundary (xHCI 6.4.1: a TRB buffer may not cross one).
 */
static inline struct xhci_dma_extent extent_cursor_trb_buf(const struct extent_cursor *cur)
{
	const struct xhci_dma_extent *ext = &cur->extents[cur->ext];
	const dma_addr_t addr = ext->addr + cur->off;
	const u32 rest = ext->length - cur->off;
	const u32 to_boundary = TRB_MAX_BUFF_SIZE - (addr & (TRB_MAX_BUFF_SIZE - 1));
	return (struct xhci_dma_extent){addr, (rest < to_boundary) ? rest : to_boundary};
}

/**
 * align_at_link() - End a TD's bytes before a Link TRB on a packet boundary.
 * @ep_ring: transfer ring.
 * @opts: the TD's options; only opts->align TDs are aligned.
 * @offset: TD bytes emitted before this TRB.
 * @td_len: the TD's length.
 * @buf: this TRB's buffer, changed in place.
 *
 * xHCI 4.11.7.1: a TD fragment may not span segments.  Applies to a TRB that
 * has TD bytes after it and a Link TRB in the next slot.  A TRB longer than its
 * bytes past the last packet boundary is cut back to that boundary; the cut
 * bytes go after the link, one TRB more.  A shorter one is replaced by a TRB
 * from a link-piece slot of the span's own (xhci_dma_span_link_piece()), which
 * covers the TD's bytes up to the next boundary.  With no slot to be had, or a
 * second piece in one TD, the TRB goes as it is.
 */
static inline void align_at_link(struct xhci_ring *ep_ring, const struct data_trb_opts *opts,
								 u32 offset, u32 td_len, struct xhci_dma_extent *buf)
{
	const u32 maxp = ep_ring->max_packet_size;
	if (!opts->align || !maxp || offset + buf->length == td_len ||
		!last_trb(ep_ring, ep_ring->enq_seg, ep_ring->enqueue + 1))
		return;

	const u32 unalign = (offset + buf->length) % maxp;
	if (!unalign)
		return;

	PERF_T0(link_t0);
	if (buf->length > unalign)
	{
		buf->length -= unalign;
		PERF_ADD(&opts->ctrl->perf, XP_LINK_CUT, link_t0);
		return;
	}

	if (maxp > XHCI_LINK_SLOT_SIZE) /* ed_maxp() allows 2047 */
		return;
	u32 piece_len = maxp - offset % maxp;
	if (piece_len > td_len - offset)
		piece_len = td_len - offset;
	const APTR slot = xhci_dma_span_link_piece(opts->ctrl, opts->align, offset, piece_len);
	if (!slot)
		return;
	buf->addr = (dma_addr_t)slot;
	buf->length = piece_len;
	PERF_ADD(&opts->ctrl->perf, XP_LINK_PIECE, link_t0);
}

/**
 * data_trb_control() - The control field of a TD's data TRB.
 * @ep_ring: transfer ring; its cycle state.
 * @opts: the TD's options.
 * @first: the TD's first data TRB.
 * @more: more data TRBs follow.
 *
 * The first TRB of a standalone TD gets the inverted cycle bit, handed over
 * once the whole TD is on the ring (xhci_ring_finalize_first_trb()); inside a
 * control TD the setup TRB is that first TRB, so every data TRB gets the
 * ring's cycle state.  The last TRB of a standalone TD raises the completion
 * interrupt; inside a control TD the status TRB does.
 *
 * Return: field 3 of the TRB.
 */
static inline u32 data_trb_control(const struct xhci_ring *ep_ring, const struct data_trb_opts *opts,
								   BOOL first, BOOL more)
{
	u32 control;
	if (!first)
		control = TRB_TYPE(TRB_NORMAL) | ep_ring->cycle_state;
	else if (opts->within_td)
		control = opts->first_type_bits | ep_ring->cycle_state;
	else
		control = opts->first_type_bits | (ep_ring->cycle_state ^ TRB_CYCLE);

	if (more)
		return control | TRB_CHAIN | opts->isp;
	if (opts->within_td)
		return control | opts->isp;
	return control | TRB_IOC | opts->ioc_bits;
}

/**
 * xhci_ring_enqueue_data_trbs() - Emit the data TRBs of a TD over its DMA extents.
 * @ep_ring: transfer ring.
 * @extents: the mapped buffer's DMA extents (xhci_dma_span_extents()).
 * @n_extents: entries in @extents.
 * @td_trb_addrs: receives the ring address of each TRB emitted (finalizing the
 *	first TRB, TD teardown and cancellation use them).
 * @opts: how to shape them.
 *
 * One TRB per 64 KB address window of each extent; in an opts->align TD the
 * TRB before a Link TRB may be cut or replaced (align_at_link()).  Chain, TD
 * Size and the interrupt bits follow the bytes still to come, not a
 * precomputed count.  A zero-length TD is one TRB.
 *
 * Return: the number of TRBs emitted; with opts->align at most the count
 * xhci_ring_calc_data_trbs() gives plus one per Link TRB crossed.
 */
inline static u32 xhci_ring_enqueue_data_trbs(struct xhci_ring *ep_ring,
											  const struct xhci_dma_extent *extents, u32 n_extents,
											  dma_addr_t *td_trb_addrs, const struct data_trb_opts *opts)
{
	u32 td_len = 0;
	for (u32 i = 0; i < n_extents; ++i)
		td_len += extents[i].length;

	struct extent_cursor cur = {.extents = extents, .n_extents = n_extents};
	u32 emitted = 0;
	u32 offset = 0;
	do
	{
		struct xhci_dma_extent buf = extent_cursor_trb_buf(&cur);
		align_at_link(ep_ring, opts, offset, td_len, &buf);

		const BOOL more = offset + buf.length < td_len;
		const u32 control = data_trb_control(ep_ring, opts, emitted == 0, more);
		const u32 remainder = xhci_td_remainder(offset, buf.length, td_len, ep_ring->max_packet_size, more);
		td_trb_addrs[emitted++] = xhci_ring_enqueue_trb_ns(ep_ring, opts->within_td || more,
														   buf.addr, /* field 0 (dma_addr_t is 32-bit) */
														   0,		 /* field 1 */
														   TRB_LEN(buf.length) | TRB_TD_SIZE(remainder) | TRB_INTR_TARGET(0),
														   control);

		offset += buf.length;
		extent_cursor_advance(&cur, buf.length);
	} while (offset < td_len);

	return emitted;
}

/* Setup, data stage, status: return the TRBs emitted. */
inline static u32 xhci_ring_enqueue_control_trbs(struct xhci_ring *ep_ring, struct xhci_xfer *io,
												 const struct xhci_dma_extent *extents, u32 n_extents,
												 dma_addr_t *td_trb_addrs)
{
	u32 n = 0;
	td_trb_addrs[n++] = xhci_ring_enqueue_setup_trb(ep_ring, io);

	if (io->data_length > 0)
	{
		const BOOL is_in = (io->setup.usd_RequestType & USB_DIR_IN) != 0;
		const struct data_trb_opts opts = {
			.first_type_bits = TRB_TYPE(TRB_DATA) | (is_in ? TRB_DIR_IN : 0),
			.isp = is_in ? TRB_ISP : 0,
			.within_td = TRUE, /* between the setup and status TRBs */
		};
		n += xhci_ring_enqueue_data_trbs(ep_ring, extents, n_extents, &td_trb_addrs[n], &opts);
	}

	td_trb_addrs[n++] = xhci_ring_enqueue_status_trb(ep_ring, io);
	return n;
}

/* A bulk, interrupt or (non-RT) iso TD: return the TRBs emitted. */
inline static u32 xhci_ring_enqueue_non_control_trbs(struct xhci_ring *ep_ring, struct xhci_xfer *io,
													 const struct xhci_dma_extent *extents, u32 n_extents,
													 dma_addr_t *td_trb_addrs, u32 iso_extra_bits)
{
	struct data_trb_opts opts = {.first_type_bits = TRB_TYPE(TRB_NORMAL)};
	if (io->type == UHCD_EPTYPE_ISO)
	{
		/* xHCI 4.11.2.3: only the first TRB is ISOC, with the iso bits
		 * (Frame ID/SIA, TBC, TLBPC); chained TRBs are NORMAL, without ISP.
		 * Not aligned at Link TRBs. */
		opts.first_type_bits = TRB_TYPE(TRB_ISOC) | iso_extra_bits;
	}
	else
	{
		/* bulk, interrupt: ISP on an IN, aligned at Link TRBs */
		opts.isp = (io->direction == XHCI_DIR_IN) ? TRB_ISP : 0;
		opts.align = &io->data_span;
		opts.ctrl = io->ctrl;
	}
	return xhci_ring_enqueue_data_trbs(ep_ring, extents, n_extents, td_trb_addrs, &opts);
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
			prime_first_trb(ep_ring, start_trb);
	}
	else
		giveback_first_trb(udev, ep_ring, start_trb);
}

/**
 * xhci_ring_calc_data_trbs() - Count the data TRBs a TD's segments need.
 * @extents: the mapped buffer's DMA extents.
 * @n_extents: entries in @extents.
 *
 * One TRB per 64 KB address window each segment touches (xHCI 6.4.1).
 *
 * Return: the TRB count; 1 for a zero-length TD.
 */
inline static u32 xhci_ring_calc_data_trbs(const struct xhci_dma_extent *extents, u32 n_extents)
{
	u32 num_trbs = 0;
	for (u32 i = 0; i < n_extents; ++i)
	{
		if (extents[i].length)
			num_trbs += DIV_CEIL((extents[i].addr & (TRB_MAX_BUFF_SIZE - 1)) + extents[i].length, TRB_MAX_BUFF_SIZE);
	}
	return num_trbs ? num_trbs : 1;
}

/**
 * xhci_submit_map() - Map a request's data and list its DMA extents.
 * @ctrl: host controller.
 * @io: the request; keeps an existing mapping (xhci_dma_req_map()).
 * @extents: XHCI_DMA_EXTENTS_MAX entries, filled in.
 * @n_extents: set to the entries filled in.
 *
 * A control transfer's direction comes from its setup packet.
 *
 * Return: the most TRBs the TD can take, setup and status included for a
 * control transfer, link alignment included for bulk and interrupt (one more
 * per Link TRB it can cross); 0 if the data could not be mapped (no bounce
 * buffer).
 */
inline static u32 xhci_submit_map(struct xhci_ctrl *ctrl, struct xhci_xfer *io,
								  struct xhci_dma_extent *extents, u32 *n_extents)
{
	const BOOL control = io->type == UHCD_EPTYPE_CONTROL;
	const BOOL to_device = control ? (io->setup.usd_RequestType & USB_DIR_IN) == 0
								   : io->direction == XHCI_DIR_OUT;
	if (!xhci_dma_req_map(ctrl, io, to_device))
		return 0;

	*n_extents = xhci_dma_span_extents(&io->data_span, extents);
	if (control)
		return (io->data_length > 0) ? 2 + xhci_ring_calc_data_trbs(extents, *n_extents) : 2;

	/* cut at 64 KB boundaries only; exact for iso, which is not aligned */
	const u32 num_trbs = xhci_ring_calc_data_trbs(extents, *n_extents);
	if (io->type == UHCD_EPTYPE_ISO)
		return num_trbs;

	/* Link alignment can add one TRB per Link TRB crossed.  Worst case the TD
	 * crosses after its first TRB, then every 255 slots, one of which goes to
	 * that crossing's extra TRB: at most DIV_CEIL(n, 254) crossings. */
	return num_trbs + DIV_CEIL(num_trbs, TRBS_PER_SEGMENT - 2);
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
 * xHCI 4.11.2.3: compute TBC and TLBPC for an ISO TD from its packet count and
 * the endpoint's Max Burst Size.  One rule serves every speed: a high-speed
 * endpoint's Max Burst Size is its additional transactions per microframe
 * (6.2.3) and a full-speed one has none, so below SuperSpeed TBC comes out 0
 * and TLBPC is the TD's packet count - 1.
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

	const u32 max_burst = xhci_ep_get_max_burst(ep_ctx);
	const u32 burst = max_burst + 1U;
	const u32 tbc = ((total_pkts + burst - 1) / burst) - 1;
	const u32 residue = total_pkts % burst;
	const u32 tlbpc = (residue == 0) ? max_burst : (residue - 1);
	return TRB_TBC(tbc) | TRB_TLBPC(tlbpc);
}

enum td_reserve_status
{
	TD_RESERVE_OK,
	TD_RESERVE_NO_ROOM,
	TD_RESERVE_NO_MEM,
};

/* Ring-room policy and TRB-address bookkeeping shared by the request and
 * RT-ISO submit paths: room check by position (xhci_ring_td_fits()) with one
 * grow attempt and recheck, the td_trb_addrs allocation, and ring preparation.
 * Failure *disposition* (queue-for-later vs unmap-and-reject) stays with the
 * caller. */
inline static enum td_reserve_status td_reserve_and_prepare(struct xhci_ctrl *ctrl,
															struct xhci_ring *ep_ring,
															u32 num_trbs,
															dma_addr_t **td_trb_addrs_out)
{
	const dma_addr_t oldest = xhci_ep_ring_oldest_trb(ep_ring);
	if (!xhci_ring_td_fits(ep_ring, num_trbs, oldest))
	{
		KprintfT("Ring full, needed %lu TRBs, attempting grow\n", (ULONG)num_trbs);
		if (!xhci_ring_grow(ctrl, ep_ring, XHCI_SEGMENTS_PER_RING) ||
			!xhci_ring_td_fits(ep_ring, num_trbs, oldest))
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
	*td_trb_addrs_out = td_trb_addrs;
	return TD_RESERVE_OK;
}

/* Build and hand over one TD (io transfer path).  Policy — state gate, ring
 * selection, queue-on-busy — lives in xhci_ep_submit(); this is pure
 * mechanics: map, emit, register (xhci_ep_set_receiving), give back.  Non-RT
 * iso TDs use SIA scheduling. */
enum xhci_submit_status xhci_submit_td(struct usb_device *udev, struct ep_context *ep_ctx,
                                       struct xhci_ring *ep_ring, struct xhci_xfer *io,
                                       s8 *err)
{
	struct xhci_ctrl *ctrl = udev->controller;

#ifdef DEBUG_CONTEXT
	xhci_dump_slot_ctx("[xhci-submit] xhci_submit_td:", udev, FALSE);
	xhci_dump_ep_ctx("[xhci-submit] xhci_submit_td:", udev, io->endpoint);
#endif

	struct xhci_dma_extent extents[XHCI_DMA_EXTENTS_MAX];
	u32 n_extents = 0;
	PERF_T0(map_t0);
	u32 num_trbs = xhci_submit_map(ctrl, io, extents, &n_extents);
	PERF_ADD(&ctrl->perf, XP_SUBMIT_MAP, map_t0);

	/* No TRB count: no bounce buffer for an unreachable/unaligned span; reject
	 * rather than DMA to a bad address.  Nothing was queued or allocated yet,
	 * so returning is clean. */
	if (num_trbs == 0)
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
		 * while queued; xhci_dma_req_map() keeps it for the resubmit. */
		KprintfT("No ring room for ep=%lu, deferring\n", (ULONG)io->endpoint);
		return XHCI_SUBMIT_NO_ROOM;
	case TD_RESERVE_NO_MEM:
		xhci_dma_span_unmap(ctrl, &io->data_span, 0);
		*err = io->error = UHIOERR_OUTOFMEMORY;
		return XHCI_SUBMIT_FAILED;
	default:
		break;
	}

	u32 emitted;
	if (io->type == UHCD_EPTYPE_CONTROL)
		emitted = xhci_ring_enqueue_control_trbs(ep_ring, io, extents, n_extents, td_trb_addrs);
	else
	{
		const u32 iso_extra_bits = (io->type == UHCD_EPTYPE_ISO)
									   ? TRB_SIA | iso_burst_bits(udev, ep_ctx, io->data_length)
									   : 0;
		emitted = xhci_ring_enqueue_non_control_trbs(ep_ring, io, extents, n_extents, td_trb_addrs, iso_extra_bits);
	}

	if (!xhci_ep_set_receiving(ep_ctx, ep_ring, io, td_trb_addrs, emitted, num_trbs))
	{
		/* The endpoint is FAILED and td_trb_addrs is freed; the emitted TRBs
		 * are never handed over (the first TRB's cycle bit stays inverted) and
		 * the recovery flush re-arms the ring past them. */
		xhci_dma_span_unmap(ctrl, &io->data_span, 0);
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
 * xhci_submit_td for a single ISOC TD.  An IN buffer is always one of the
 * endpoint's staging slab; the doorbell is always left to the scheduler's
 * per-run xhci_submit_giveback().
 */
s8 xhci_submit_rt_td(struct usb_device *udev, struct ep_context *ep_ctx,
                     APTR buffer, u32 length, u16 frame, u16 dir, BOOL silent)
{
	struct xhci_ctrl *ctrl = udev->controller;
	const BOOL staging_in = (dir == XHCI_DIR_IN);

	struct xhci_ring *ep_ring = xhci_ep_get_ring(ep_ctx);
	if (!ep_ring)
		return UHIOERR_HOSTERROR;

	struct xhci_dma_span span;
	/* staging-slab buffers own whole cache lines by construction, so an IN
	 * map never bounces on a non-line-multiple TD length */
	const u32 flags = ((dir == XHCI_DIR_OUT) ? XHCI_MAP_TO_DEVICE : 0) |
					  (staging_in ? XHCI_MAP_OWNS_LINES : 0);
	if (!xhci_dma_span_map(ctrl, &span, buffer, length, flags))
	{
		/* map failed (no bounce for an unreachable/unaligned span): nothing was
		 * allocated or flushed, so reject without unmapping. */
		Kprintf("RT-ISO DMA map failed len %lu; rejecting transfer\n", (ULONG)length);
		return UHIOERR_OUTOFMEMORY;
	}

	struct xhci_dma_extent extents[XHCI_DMA_EXTENTS_MAX];
	const u32 n_extents = xhci_dma_span_extents(&span, extents);
	const u32 num_trbs = xhci_ring_calc_data_trbs(extents, n_extents);

	dma_addr_t *td_trb_addrs;
	switch (td_reserve_and_prepare(ctrl, ep_ring, num_trbs, &td_trb_addrs))
	{
	case TD_RESERVE_NO_ROOM:
		xhci_dma_span_unmap(ctrl, &span, 0);
		return UHIOERR_HOSTERROR;
	case TD_RESERVE_NO_MEM:
		xhci_dma_span_unmap(ctrl, &span, 0);
		return UHIOERR_OUTOFMEMORY;
	default:
		break;
	}

	/* CFC controllers pin the TD to its Frame ID; without CFC the hardware
	 * schedules ASAP (SIA). */
	const u32 iso_bits = (ctrl->cfc_supported ? TRB_FRAME_ID(frame) : TRB_SIA) |
						 iso_burst_bits(udev, ep_ctx, length);
	const struct data_trb_opts opts = {
		.first_type_bits = TRB_TYPE(TRB_ISOC) | iso_bits,
		.ioc_bits = silent ? TRB_BEI : 0, /* a silent TD completes without an interrupt */
	};
	const u32 emitted = xhci_ring_enqueue_data_trbs(ep_ring, extents, n_extents, td_trb_addrs, &opts);

	if (!xhci_ep_set_receiving_rt(ep_ctx, &span, frame, dir, staging_in, td_trb_addrs, emitted))
	{
		/* the endpoint is FAILED now (set_receiving_rt freed td_trb_addrs);
		 * the span is still this function's to release */
		xhci_dma_span_unmap(ctrl, &span, 0);
		return UHIOERR_OUTOFMEMORY;
	}

	xhci_ring_finalize_first_trb(udev, ep_ring, (struct xhci_generic_trb *)td_trb_addrs[0], TRUE);
	return UHIOERR_NO_ERROR;
}
