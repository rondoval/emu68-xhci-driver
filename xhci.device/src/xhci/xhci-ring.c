// SPDX-License-Identifier: GPL-2.0+
/*
 * USB HOST XHCI Controller stack
 *
 * Based on xHCI host controller driver in linux-kernel
 * by Sarah Sharp.
 *
 * Copyright (C) 2008 Intel Corp.
 * Author: Sarah Sharp
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 * Authors: Vivek Gautam <gautam.vivek@samsung.com>
 *	    Vikas Sajjan <vikas.sajjan@samsung.com>
 */

#include <debug.h>

#include <xhci/xhci-ring.h>

#include <xhci/xhci.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-debug.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-usb.h>
#include <xhci/xhci-udev.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-ring] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci-ring] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

struct xhci_segment
{
	union xhci_trb *trbs;
	/* private to HCD */
	struct xhci_segment *next;
};

struct xhci_ring
{
	BOOL is_event_ring;
	int ep_index; /* for transfer rings, the endpoint index this ring is associated with. For event rings, unused and set to 0. */
	unsigned int num_segs;
	APTR memoryPool;

	struct xhci_segment *first_seg;
	union xhci_trb *enqueue;
	struct xhci_segment *enq_seg;
	union xhci_trb *dequeue;
	struct xhci_segment *deq_seg;
	/*
	 * Write the cycle state into the TRB cycle field to give ownership of
	 * the TRB to the host controller (if we are the producer), or to check
	 * if we own the TRB (if we are the consumer).  See section 4.9.1.
	 */
	volatile u32 cycle_state;

	struct xhci_generic_trb *deferred_giveback;
};

/**
 * frees the "segment" pointer passed
 *
 * @param ptr	pointer to "segement" to be freed
 * Return: none
 */
static void xhci_segment_free(struct xhci_ctrl *ctrl, struct xhci_segment *seg)
{
	memalign_free(ctrl->memoryPool, seg->trbs);
	seg->trbs = NULL;

	FreeVecPooled(ctrl->memoryPool, seg);
}

/**
 * frees the "ring" pointer passed
 *
 * @param ptr	pointer to "ring" to be freed
 * Return: none
 */
void xhci_ring_free(struct xhci_ctrl *ctrl, struct xhci_ring *ring)
{
	if (!ring)
	{
		Kprintf("Ring is NULL, nothing to free\n");
		return;
	}

	struct xhci_segment *first_seg = ring->first_seg;
	struct xhci_segment *seg = first_seg->next;
	while (seg != first_seg)
	{
		struct xhci_segment *next = seg->next;
		xhci_segment_free(ctrl, seg);
		seg = next;
	}
	xhci_segment_free(ctrl, first_seg);

	FreeVecPooled(ctrl->memoryPool, ring);
}

/**
 * Make the prev segment point to the next segment.
 * Change the last TRB in the prev segment to be a Link TRB which points to the
 * address of the next segment.  The caller needs to set any Link TRB
 * related flags, such as End TRB, Toggle Cycle, and no snoop.
 *
 * @param prev	pointer to the previous segment
 * @param next	pointer to the next segment
 * @param link_trbs	flag to indicate whether to link the trbs or NOT
 * Return: none
 */
static void xhci_link_segments(struct xhci_segment *prev,
							   struct xhci_segment *next, BOOL link_trbs)
{
	if (!prev || !next)
		return;
	prev->next = next;
	if (link_trbs)
	{
		prev->trbs[TRBS_PER_SEGMENT - 1].link.segment_ptr =
			LE64((dma_addr_t)next->trbs);

		/*
		 * Set the last TRB in the segment to
		 * have a TRB type ID of Link TRB
		 */
		u32 val = LE32(prev->trbs[TRBS_PER_SEGMENT - 1].link.control);
		val &= ~TRB_TYPE_BITMASK;
		val |= TRB_TYPE(TRB_LINK);
		prev->trbs[TRBS_PER_SEGMENT - 1].link.control = LE32(val);
	}
}

/**
 * Initialises the Ring's enqueue,dequeue,enq_seg pointers
 *
 * @param ring	pointer to the RING to be intialised
 * Return: none
 */
static void xhci_initialize_ring_info(struct xhci_ring *ring)
{
	/*
	 * The ring is empty, so the enqueue pointer == dequeue pointer
	 */
	ring->enqueue = ring->first_seg->trbs;
	ring->enq_seg = ring->first_seg;
	ring->dequeue = ring->enqueue;
	ring->deq_seg = ring->first_seg;

	/*
	 * The ring is initialized to 0. The producer must write 1 to the
	 * cycle bit to handover ownership of the TRB, so PCS = 1.
	 * The consumer must compare CCS to the cycle bit to
	 * check ownership, so CCS = 1.
	 */
	ring->cycle_state = 1;

	ring->deferred_giveback = NULL;
}

/**
 * Allocates a generic ring segment from the ring pool, sets the dma address,
 * initializes the segment to zero, and sets the private next pointer to NULL.
 * Section 4.11.1.1:
 * "All components of all Command and Transfer TRBs shall be initialized to '0'"
 *
 * @param	none
 * Return: pointer to the newly allocated SEGMENT
 */
static struct xhci_segment *xhci_segment_alloc(struct xhci_ctrl *ctrl)
{
	struct xhci_segment *seg = AllocVecPooled(ctrl->memoryPool, sizeof(struct xhci_segment));
	if (!seg)
	{
		Kprintf("AllocVecPooled failed for size %lu\n",
				(ULONG)sizeof(struct xhci_segment));
		return NULL;
	}

	seg->trbs = xhci_malloc(ctrl, SEGMENT_SIZE);
	seg->next = NULL;

	return seg;
}

/**
 * Create a new ring with zero or more segments.
 * TODO: current code only uses one-time-allocated single-segment rings
 * of 1KB anyway, so we might as well get rid of all the segment and
 * linking code (and maybe increase the size a bit, e.g. 4KB).
 *
 *
 * Link each segment together into a ring.
 * Set the end flag and the cycle toggle bit on the last segment.
 * See section 4.9.2 and figures 15 and 16 of XHCI spec rev1.0.
 *
 * @param num_segs	number of segments in the ring
 * @param link_trbs	flag to indicate whether to link the trbs or NOT
 * Return: pointer to the newly created RING
 */
struct xhci_ring *xhci_ring_alloc(struct xhci_ctrl *ctrl, unsigned int num_segs,
								  BOOL link_trbs, BOOL is_event_ring, int ep_index)
{
	unsigned int remaining = num_segs;

	struct xhci_ring *ring = AllocVecPooled(ctrl->memoryPool, sizeof(struct xhci_ring));
	if (!ring)
	{
		Kprintf("AllocVecPooled failed for size %lu\n",
				(ULONG)sizeof(struct xhci_ring));
		return NULL;
	}
	_memset(ring, 0, sizeof(struct xhci_ring));
	ring->num_segs = num_segs;
	ring->is_event_ring = is_event_ring;
	ring->memoryPool = ctrl->memoryPool;
	ring->ep_index = ep_index;

	if (remaining == 0)
		return ring;

	ring->first_seg = xhci_segment_alloc(ctrl);
	if (!ring->first_seg)
	{
		Kprintf("xhci_segment_alloc failed\n");
		FreeVecPooled(ctrl->memoryPool, ring);
		return NULL;
	}

	remaining--;

	struct xhci_segment *prev = ring->first_seg;
	while (remaining > 0)
	{
		struct xhci_segment *next;

		next = xhci_segment_alloc(ctrl);
		if (!next)
		{
			Kprintf("xhci_segment_alloc failed\n");
			xhci_ring_free(ctrl, ring);
			return NULL;
		}

		xhci_link_segments(prev, next, link_trbs);

		prev = next;
		remaining--;
	}
	xhci_link_segments(prev, ring->first_seg, link_trbs);
	if (link_trbs)
	{
		/* See section 4.9.2.1 and 6.4.4.1 */
		prev->trbs[TRBS_PER_SEGMENT - 1].link.control |=
			LE32(LINK_TOGGLE);
	}
	xhci_initialize_ring_info(ring);

	return ring;
}

void xhci_ring_setup_erst(struct xhci_ring *ring, struct xhci_erst *erst, struct xhci_intr_reg *ir_set)
{
	uint32_t val;
	struct xhci_segment *seg;
	erst->num_entries = ERST_NUM_SEGS;

	for (val = 0, seg = ring->first_seg;
		 val < ERST_NUM_SEGS;
		 val++)
	{
		struct xhci_erst_entry *entry = &erst->entries[val];
		entry->seg_addr = LE64((dma_addr_t)seg->trbs);
		entry->seg_size = LE32(TRBS_PER_SEGMENT);
		entry->rsvd = 0;
		seg = seg->next;
	}
	xhci_flush_cache(erst->entries, ERST_NUM_SEGS * sizeof(struct xhci_erst_entry));

	/* Update HC event ring dequeue pointer */
	xhci_writeq(&ir_set->erst_dequeue,
				(u64)(uintptr_t)ring->dequeue & (u64)~ERST_PTR_MASK);

	/* set ERST count with the number of entries in the segment table */
	val = readl(&ir_set->erst_size);
	val &= ERST_SIZE_MASK;
	val |= ERST_NUM_SEGS;
	writel(val, &ir_set->erst_size);

	/* this is the event ring segment table pointer */
	u64 val_64 = xhci_readq(&ir_set->erst_base);
	val_64 &= ERST_PTR_MASK;
	val_64 |= (dma_addr_t)erst->entries & ~ERST_PTR_MASK;

	xhci_writeq(&ir_set->erst_base, val_64);
}

/**
 * Is this TRB a link TRB or was the last TRB the last TRB in this event ring
 * segment?  I.e. would the updated event TRB pointer step off the end of the
 * event seg ?
 *
 * @param ring	pointer to the ring
 * @param seg	poniter to the segment to which TRB belongs
 * @param trb	poniter to the ring trb
 * Return: 1 if this TRB a link TRB else 0
 */
inline static int last_trb(struct xhci_ring *ring,
						   struct xhci_segment *seg, union xhci_trb *trb)
{
	if (ring->is_event_ring)
		return trb == &seg->trbs[TRBS_PER_SEGMENT];
	else
		return TRB_TYPE_LINK_LE32(trb->link.control);
}

/**
 * Does this link TRB point to the first segment in a ring,
 * or was the previous TRB the last TRB on the last segment in the ERST?
 *
 * @param ring	pointer to the ring
 * @param seg	poniter to the segment to which TRB belongs
 * @param trb	poniter to the ring trb
 * Return: 1 if this TRB is the last TRB on the last segment else 0
 */
inline static BOOL last_trb_on_last_seg(struct xhci_ring *ring,
										struct xhci_segment *seg,
										union xhci_trb *trb)
{
	if (ring->is_event_ring)
		return ((trb == &seg->trbs[TRBS_PER_SEGMENT]) &&
				(seg->next == ring->first_seg));
	else
		return LE32(trb->link.control) & LINK_TOGGLE;
}

/**
 * See Cycle bit rules. SW is the consumer for the event ring only.
 * Don't make a ring full of link TRBs.  That would be dumb and this would loop.
 *
 * If we've just enqueued a TRB that is in the middle of a TD (meaning the
 * chain bit is set), then set the chain bit in all the following link TRBs.
 * If we've enqueued the last TRB in a TD, make sure the following link TRBs
 * have their chain bit cleared (so that each Link TRB is a separate TD).
 *
 * Section 6.4.4.1 of the 0.95 spec says link TRBs cannot have the chain bit
 * set, but other sections talk about dealing with the chain bit set.  This was
 * fixed in the 0.96 specification errata, but we have to assume that all 0.95
 * xHCI hardware can't handle the chain bit being cleared on a link TRB.
 *
 * @param ring	pointer to the ring
 * @param more_trbs_coming	flag to indicate whether more trbs
 *				are expected or NOT.
 *				Will you enqueue more TRBs before calling
 *				prepare_ring()?
 * Return: none
 */
inline static void inc_enq(struct xhci_ring *ring, BOOL more_trbs_coming)
{
	u32 chain = LE32(ring->enqueue->generic.field[3]) & TRB_CHAIN;
	union xhci_trb *next = ++(ring->enqueue);

	/*
	 * Update the dequeue pointer further if that was a link TRB or we're at
	 * the end of an event ring segment (which doesn't have link TRBS)
	 */
	while (last_trb(ring, ring->enq_seg, next))
	{
		if (!ring->is_event_ring)
		{
			/*
			 * If the caller doesn't plan on enqueueing more
			 * TDs before ringing the doorbell, then we
			 * don't want to give the link TRB to the
			 * hardware just yet.  We'll give the link TRB
			 * back in prepare_ring() just before we enqueue
			 * the TD at the top of the ring.
			 */
			if (!chain && !more_trbs_coming)
				break;

			/*
			 * If we're not dealing with 0.95 hardware or
			 * isoc rings on AMD 0.96 host,
			 * carry over the chain bit of the previous TRB
			 * (which may mean the chain bit is cleared).
			 */
			next->link.control &= LE32(~TRB_CHAIN);
			next->link.control |= LE32(chain);

			next->link.control ^= LE32(TRB_CYCLE);
			xhci_flush_cache(next,
							 sizeof(union xhci_trb));
		}
		/* Toggle the cycle bit after the last ring segment. */
		if (last_trb_on_last_seg(ring,
								 ring->enq_seg, next))
			ring->cycle_state = (ring->cycle_state ? 0 : 1);

		ring->enq_seg = ring->enq_seg->next;
		ring->enqueue = ring->enq_seg->trbs;
		next = ring->enqueue;
	}
}

/**
 * See Cycle bit rules. SW is the consumer for the event ring only.
 * Don't make a ring full of link TRBs.  That would be dumb and this would loop.
 *
 * @param ring	Ring whose Dequeue TRB pointer needs to be incremented.
 * return none
 */
inline static void inc_deq(struct xhci_ring *ring)
{
	do
	{
		/*
		 * Update the dequeue pointer further if that was a link TRB or
		 * we're at the end of an event ring segment (which doesn't have
		 * link TRBS)
		 */
		if (last_trb(ring, ring->deq_seg, ring->dequeue))
		{
			if (ring->is_event_ring &&
				last_trb_on_last_seg(ring, ring->deq_seg, ring->dequeue))
			{
				ring->cycle_state = (ring->cycle_state ? 0 : 1);
			}
			ring->deq_seg = ring->deq_seg->next;
			ring->dequeue = ring->deq_seg->trbs;
		}
		else
		{
			ring->dequeue++;
		}
	} while (last_trb(ring, ring->deq_seg, ring->dequeue));
}

/**
 * Generic function for queueing a TRB on a ring.
 * The caller must have checked to make sure there's room on the ring.
 *
 * @param	more_trbs_coming:   Will you enqueue more TRBs before calling
 *				prepare_ring()?
 * @param ring	pointer to the ring
 * @param more_trbs_coming	flag to indicate whether more trbs
 * @param trb_fields	pointer to trb field array containing TRB contents
 * Return: pointer to the enqueued trb
 */
inline static dma_addr_t xhci_ring_enqueue_trb(struct xhci_ring *ring,
											   BOOL more_trbs_coming,
											   u32 field0, u32 field1, u32 field2, u32 field3)
{
	struct xhci_generic_trb *trb = &ring->enqueue->generic;

	trb->field[0] = LE32(field0);
	trb->field[1] = LE32(field1);
	trb->field[2] = LE32(field2);
	trb->field[3] = LE32(field3);

	xhci_flush_cache(trb, sizeof(struct xhci_generic_trb));

	inc_enq(ring, more_trbs_coming);

	return (dma_addr_t)trb;
}

/**
 * Does various checks on the endpoint ring, and makes it ready
 * to queue num_trbs.
 *
 * @param ep_ring	pointer to the EP Transfer Ring
 * Return: none
 */
inline static void prepare_ring(struct xhci_ring *ep_ring)
{
	union xhci_trb *next = ep_ring->enqueue;

	while (last_trb(ep_ring, ep_ring->enq_seg, next))
	{
		/*
		 * If we're not dealing with 0.95 hardware or isoc rings
		 * on AMD 0.96 host, clear the chain bit.
		 */
		next->link.control &= LE32(~TRB_CHAIN);

		next->link.control ^= LE32(TRB_CYCLE);

		xhci_flush_cache(next, sizeof(union xhci_trb));

		/* Toggle the cycle bit after the last ring segment. */
		if (last_trb_on_last_seg(ep_ring, ep_ring->enq_seg, next))
			ep_ring->cycle_state = (ep_ring->cycle_state ? 0 : 1);
		ep_ring->enq_seg = ep_ring->enq_seg->next;
		ep_ring->enqueue = ep_ring->enq_seg->trbs;
		next = ep_ring->enqueue;
	}
}

/**
 * Checks if there is a new event to handle on the event ring.
 *
 * @param ring	pointer to the event RING
 * Return: pointer to the event TRB if ready, NULL otherwise
 */
union xhci_trb *xhci_ring_get_event_trb(struct xhci_ring *ring)
{
	xhci_inval_cache(ring->dequeue, sizeof(union xhci_trb));
	union xhci_trb *event = ring->dequeue;

	/* Does the HC or OS own the TRB? */
	if ((LE32(event->event_cmd.flags) & TRB_CYCLE) != ring->cycle_state)
		return NULL;

	return event;
}

/**
 * Finalizes a handled event TRB by advancing our dequeue pointer and giving
 * the TRB back to the hardware for recycling. Must call this exactly once at
 * the end of each event handler, and not touch the TRB again afterwards.
 *
 * @param ctrl	Host controller data structure
 * Return: none
 */
void xhci_ring_acknowledge_event(struct xhci_ctrl *ctrl)
{
	// TODO get rid of this function once xhci registers are refactored
	/* Advance our dequeue pointer to the next event */
	inc_deq(ctrl->event_ring);

	/* Inform the hardware */
	xhci_writeq(&ctrl->ir_set->erst_dequeue, (dma_addr_t)ctrl->event_ring->dequeue | ERST_EHB);
}

u32 xhci_ring_get_new_dequeue_ptr(struct xhci_ring *ring)
{
	return (u32)ring->enqueue | ring->cycle_state;
}

dma_addr_t xhci_ring_enqueue_command(struct xhci_ring *ring, u64 address, u32 slot_id, u32 ep_index, trb_type cmd)
{
	prepare_ring(ring);

	u32 field3 = TRB_TYPE(cmd) | SLOT_ID_FOR_TRB(slot_id) | ring->cycle_state;

	/*
	 * Only 'reset endpoint', 'stop endpoint' and 'set TR dequeue pointer'
	 * commands need endpoint id encoded.
	 */
	if (cmd >= TRB_RESET_EP && cmd <= TRB_SET_DEQ)
		field3 |= EP_ID_FOR_TRB(ep_index);

	dma_addr_t trb_dma = xhci_ring_enqueue_trb(ring, FALSE,
											   lower_32_bits(address), /* field0 */
											   upper_32_bits(address), /* field1 */
											   0,					   /* field2 */
											   field3);				   /* field3 */
	return trb_dma;
}

/*
 * For xHCI 1.0 host controllers, TD size is the number of max packet sized
 * packets remaining in the TD (*not* including this TRB).
 *
 * Total TD packet count = total_packet_count =
 *     DIV_ROUND_UP(TD size in bytes / wMaxPacketSize)
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
 * @param ctrl	host controller data structure
 * @param transferred	total size sent so far
 * @param trb_buff_len	length of the TRB Buffer
 * @param td_total_len	total packet count
 * @param maxp	max packet size of current pipe
 * @param more_trbs_coming	indicate last trb in TD
 * Return: remainder
 */
inline static u32 xhci_td_remainder(int transferred,
									unsigned int trb_buff_len, unsigned int td_total_len,
									int maxp, BOOL more_trbs_coming)
{
	/* One TRB with a zero-length data packet. */
	if (!more_trbs_coming || (transferred == 0 && trb_buff_len == 0) ||
		trb_buff_len == td_total_len)
		return 0;

	u32 total_packet_count = DIV_ROUND_UP(td_total_len, maxp);

	/* Queueing functions don't count the current TRB into transferred */
	return (total_packet_count - ((transferred + trb_buff_len) / maxp));
}

inline static int ring_has_room(struct xhci_ring *ring, struct ep_context *ep_ctx, unsigned int needed)
{
	unsigned int capacity = ring->num_segs * (TRBS_PER_SEGMENT - 1);
	if (capacity == 0)
		return 0;
	return xhci_ep_get_active_trb_count(ep_ctx) + needed <= capacity;
}

inline static void prime_first_trb(struct xhci_generic_trb *start_trb)
{
	start_trb->field[3] ^= LE32(TRB_CYCLE);
	xhci_flush_cache(start_trb, sizeof(struct xhci_generic_trb));
}

inline static void giveback_first_trb(struct usb_device *udev, int ep_index,
									  struct xhci_generic_trb *start_trb)
{
	struct xhci_ctrl *ctrl = udev->controller;

	prime_first_trb(start_trb);

	/* Ringing EP doorbell here */
	writel(DB_VALUE(ep_index, 0), &ctrl->dba->doorbell[udev->slot_id]);

	return;
}

void xhci_ring_giveback(struct usb_device *udev, struct ep_context *ep_ctx)
{
	if (!ep_ctx)
		return;

	struct xhci_ring *ring = xhci_ep_get_ring(ep_ctx);
	giveback_first_trb(udev, ring->ep_index, ring->deferred_giveback);
}

inline static dma_addr_t xhci_ring_enqueue_setup_trb(struct xhci_ring *ep_ring, struct IOUsbHWReq *io)
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
	if (io->iouh_Length > 0)
	{
		if (io->iouh_SetupData.bmRequestType & USB_DIR_IN)
			field3 |= TRB_TX_TYPE(TRB_DATA_IN);
		else
			field3 |= TRB_TX_TYPE(TRB_DATA_OUT);
	}

	return xhci_ring_enqueue_trb(ep_ring, TRUE,
								 io->iouh_SetupData.bmRequestType | io->iouh_SetupData.bRequest << 8 | LE16(io->iouh_SetupData.wValue) << 16, /* field 0 */
								 LE16(io->iouh_SetupData.wIndex) | LE16(io->iouh_SetupData.wLength) << 16,									  /* field 1 */
								 TRB_LEN(8) | TRB_INTR_TARGET(0),																			  /* field 2 */
								 field3);																									  /* field 3 */
}

inline static dma_addr_t xhci_ring_enqueue_data_trb(struct xhci_ctrl *ctrl, struct xhci_ring *ep_ring, struct IOUsbHWReq *io)
{
	/* If there's data, queue data TRBs */
	/* Only set interrupt on short packet for IN endpoints */

	u32 remainder = xhci_td_remainder(0, io->iouh_Length, io->iouh_Length, io->iouh_MaxPktSize, TRUE);
	u32 length_field = TRB_LEN(io->iouh_Length) | TRB_TD_SIZE(remainder) | TRB_INTR_TARGET(0);
	KprintfH("length_field = %ld, length = %ld,"
			 "xhci_td_remainder(length) = %ld , TRB_INTR_TARGET(0) = %ld\n",
			 length_field, TRB_LEN(io->iouh_Length),
			 TRB_TD_SIZE(remainder), 0);

	BOOL is_direction_in = (io->iouh_SetupData.bmRequestType & USB_DIR_IN) != 0;
	u64 buf_64 = xhci_dma_map(ctrl, io, !is_direction_in);

	u32 field3 = (is_direction_in) ? TRB_ISP | TRB_DIR_IN | TRB_TYPE(TRB_DATA) : TRB_TYPE(TRB_DATA);
	field3 |= ep_ring->cycle_state;

	return xhci_ring_enqueue_trb(ep_ring, TRUE,
								 lower_32_bits(buf_64), /* field 0 */
								 upper_32_bits(buf_64), /* field 1 */
								 length_field,			/* field 2 */
								 field3);				/* field 3 */
}

inline static dma_addr_t xhci_ring_enqueue_status_trb(struct xhci_ring *ep_ring, struct IOUsbHWReq *io)
{
	/*
	 * Queue status TRB -
	 * see Table 7 and sections 4.11.2.2 and 6.4.1.2.3
	 */

	/* If the device sent data, the status stage is an OUT transfer */
	u32 field3 = (io->iouh_Length > 0 && io->iouh_SetupData.bmRequestType & USB_DIR_IN) ? 0 : TRB_DIR_IN;

	/* Event on completion */
	field3 |= TRB_IOC | TRB_TYPE(TRB_STATUS) | ep_ring->cycle_state;

	return xhci_ring_enqueue_trb(ep_ring, FALSE,
								 0,					 /* field 0 */
								 0,					 /* field 1 */
								 TRB_INTR_TARGET(0), /* field 2 */
								 field3);			 /* field 3 */
}

inline static void xhci_ring_enqueue_control_trbs(struct xhci_ctrl *ctrl, struct xhci_ring *ep_ring, struct IOUsbHWReq *io, dma_addr_t *td_trb_addrs)
{
	unsigned int td_trb_index = 0;
	const unsigned int length = io->iouh_Length;

	dma_addr_t setup_trb = xhci_ring_enqueue_setup_trb(ep_ring, io);
	td_trb_addrs[td_trb_index++] = setup_trb;

	if (length > 0)
	{
		dma_addr_t data_trb = xhci_ring_enqueue_data_trb(ctrl, ep_ring, io);
		td_trb_addrs[td_trb_index++] = data_trb;
	}

	dma_addr_t status_trb = xhci_ring_enqueue_status_trb(ep_ring, io);
	td_trb_addrs[td_trb_index++] = status_trb;
}

inline static void xhci_ring_enqueue_non_control_trbs(struct xhci_ring *ep_ring, struct IOUsbHWReq *io, u64 addr, int num_trbs, unsigned int trb_buff_len, dma_addr_t *td_trb_addrs)
{
	const BOOL is_iso = io->iouh_Req.io_Command == UHCMD_ADDISOHANDLER ||
						io->iouh_Req.io_Command == UHCMD_ISOXFER;

	const unsigned int length = io->iouh_Length;
	const u32 enable_short_packet = (!is_iso && (io->iouh_Dir == UHDIR_IN)) ? TRB_ISP : 0;
	const u32 trb_type_bits = (is_iso) ? ((u32)TRB_TYPE(TRB_ISOC) | TRB_SIA) : (TRB_TYPE(TRB_NORMAL) | enable_short_packet);

	unsigned int running_total = 0;
	unsigned int td_trb_index = 0;

	/* How much data is in the first TRB? */
	/*
	 * How much data is (potentially) left before the 64KB boundary?
	 * XHCI Spec puts restriction( TABLE 49 and 6.4.1 section of XHCI Spec)
	 * that the buffer should not span 64KB boundary. if so
	 * we send request in more than 1 TRB by chaining them.
	 */
	if (trb_buff_len > length)
		trb_buff_len = length;

	BOOL first_trb = TRUE;

	/* Queue each TRB, chaining when necessary and marking the final one IOC. */
	/* Queue the first TRB, even if it's zero-length. */
	do
	{
		u32 field3 = trb_type_bits;
		/* Don't change the cycle bit of the first TRB until later */
		if (first_trb)
		{
			first_trb = FALSE;
			if (ep_ring->cycle_state == 0)
				field3 |= TRB_CYCLE;
		}
		else
		{
			field3 |= ep_ring->cycle_state;
		}

		/*
		 * Chain all the TRBs together; clear the chain bit in the last
		 * TRB to indicate it's the last TRB in the chain.
		 */
		field3 |= (num_trbs > 1) ? TRB_CHAIN : TRB_IOC;

		/* Set the TRB length, TD size, and interrupter fields. */
		u32 remainder = xhci_td_remainder(running_total, trb_buff_len,
										  length, (int)io->iouh_MaxPktSize,
										  num_trbs > 1);

		u32 length_field = (TRB_LEN(trb_buff_len) | TRB_TD_SIZE(remainder) | TRB_INTR_TARGET(0));

		dma_addr_t last_transfer_trb_addr = xhci_ring_enqueue_trb(ep_ring, (num_trbs > 1),
																  lower_32_bits(addr), /* field 0 */
																  upper_32_bits(addr), /* field 1 */
																  length_field,		   /* field 2 */
																  field3);			   /* field 3 */
		td_trb_addrs[td_trb_index++] = last_transfer_trb_addr;
		--num_trbs;
		running_total += trb_buff_len;

		/* Calculate length for next transfer */
		addr += trb_buff_len;
		trb_buff_len = min((length - running_total), TRB_MAX_BUFF_SIZE);
	} while (running_total < length);
}

inline static void xhci_ring_finalize_first_trb(struct usb_device *udev, int ep_index, struct xhci_ring *ep_ring, struct xhci_generic_trb *start_trb, BOOL defer_doorbell)
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
		giveback_first_trb(udev, ep_index, start_trb);
}

inline static int xhci_ring_calc_num_trbs(struct xhci_ctrl *ctrl, struct IOUsbHWReq *io, int *trb_buff_len, u64 *addr)
{
	if (io->iouh_Req.io_Command == UHCMD_CONTROLXFER)
		/* 1 TRB for setup, 1 for status, 1 optional for data */
		return (io->iouh_Length > 0) ? 3 : 2;

	*addr = xhci_dma_map(ctrl, io, io->iouh_Dir == UHDIR_OUT);

	/*
	 * How much data is (potentially) left before the 64KB boundary?
	 * XHCI Spec (Table 49 / 6.4.1) requires we avoid spanning that boundary, so
	 * we may need several chained TRBs if the buffer crosses it.
	 */
	int running_total = TRB_MAX_BUFF_SIZE - (lower_32_bits(*addr) & (TRB_MAX_BUFF_SIZE - 1));
	*trb_buff_len = running_total;
	running_total &= TRB_MAX_BUFF_SIZE - 1;

	int num_trbs = 0;
	/* If we already cover bytes in this 64KB chunk, or the transfer is zero length,
	 * we schedule at least one TRB now.
	 */
	if (running_total != 0 || io->iouh_Length == 0)
		num_trbs++;

	/* Account for remaining 64KB windows, adding more TRBs as needed. */
	num_trbs += DIV_ROUND_UP(io->iouh_Length - running_total, TRB_MAX_BUFF_SIZE);
	return num_trbs;
}

int xhci_ring_enqueue_td(struct usb_device *udev, struct IOUsbHWReq *io, unsigned int timeout_ms, BOOL defer_doorbell)
{
	struct xhci_ctrl *ctrl = udev->controller;

	const int ep_index = xhci_ep_index_from_parts(io->iouh_Endpoint, io->iouh_Dir);
	struct ep_context *udev_ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
	if (!udev_ep_ctx)
	{
		Kprintf("No ep context for ep %d\n", ep_index);
		return UHIOERR_BADPARAMS;
	}

	enum ep_state cur_state = xhci_ep_get_state(udev_ep_ctx);
	if (cur_state == USB_DEV_EP_STATE_ABORTING ||
		cur_state == USB_DEV_EP_STATE_RESETTING ||
		cur_state == USB_DEV_EP_STATE_FAILED)
	{
		KprintfH("Cannot submit transfer, ep in state %d\n", cur_state);
		xhci_ep_enqueue(udev_ep_ctx, io);
		return UHIOERR_NO_ERROR;
	}

#ifdef DEBUG_HIGH
	xhci_inval_cache(udev->out_ctx->bytes,
					 udev->out_ctx->size);

	struct xhci_ep_ctx *xep_ctx = xhci_get_ep_ctx(ctrl, udev->out_ctx, ep_index);
	xhci_dump_slot_ctx("[xhci-ring] xhci_stream_tx:", xhci_get_slot_ctx(ctrl, udev->out_ctx));
	xhci_dump_ep_ctx("[xhci-ring] xhci_stream_tx:", io->iouh_Endpoint, xep_ctx);
#endif

	int trb_buff_len = 0;
	u64 addr = 0;
	int num_trbs = xhci_ring_calc_num_trbs(ctrl, io, &trb_buff_len, &addr);

	struct xhci_ring *ep_ring = xhci_ep_get_ring(udev_ep_ctx);
	if (!ep_ring)
	{
		Kprintf("No ring for ep %d\n", ep_index);
		return UHIOERR_HOSTERROR;
	}

	if (!ring_has_room(ep_ring, udev_ep_ctx, num_trbs + 1))
	{
		KprintfH("Ring full ep=%ld needed %ld TRBs\n", 0L, (LONG)num_trbs);
		xhci_ep_enqueue(udev_ep_ctx, io);
		return UHIOERR_NO_ERROR;
	}

	dma_addr_t *td_trb_addrs = AllocVecPooled(ctrl->memoryPool, num_trbs * sizeof(dma_addr_t));
	if (!td_trb_addrs)
	{
		Kprintf("Failed to alloc TD TRB list\n");
		io->iouh_Req.io_Error = UHIOERR_OUTOFMEMORY;
		return UHIOERR_OUTOFMEMORY;
	}

	/*
	 * XXX: Calling routine prepare_ring() called in place of
	 * prepare_trasfer() as there in 'Linux' since we are not
	 * maintaining multiple TDs/transfer at the same time.
	 */
	prepare_ring(ep_ring);

	if (io->iouh_Req.io_Command == UHCMD_CONTROLXFER)
		xhci_ring_enqueue_control_trbs(ctrl, ep_ring, io, td_trb_addrs);
	else
		xhci_ring_enqueue_non_control_trbs(ep_ring, io, addr, num_trbs, trb_buff_len, td_trb_addrs);

	xhci_ep_set_receiving(udev_ep_ctx, io, td_trb_addrs, timeout_ms, num_trbs);
	xhci_ring_finalize_first_trb(udev, ep_index, ep_ring, (struct xhci_generic_trb *)td_trb_addrs[0], defer_doorbell);

	return UHIOERR_NO_ERROR;
}
