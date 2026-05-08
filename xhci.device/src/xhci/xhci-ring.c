// SPDX-License-Identifier: GPL-2.0-only
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

#include <config.h>
#include <debug.h>
#include <memory.h>

#include <xhci/xhci-ring.h>

#include <xhci/xhci.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-descriptors.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci-context.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-ring] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci-ring] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

static inline void xhci_copy_to_bounce_buffer(CONST_APTR src, APTR dst, u32 size)
{
	if ((((uintptr_t)src | (uintptr_t)dst | size) & (sizeof(ULONG) - 1)) == 0)
	{
		CopyMemQuick((ULONG *)src, (ULONG *)dst, size);
		return;
	}

	CopyMem(src, dst, size);
}

struct xhci_segment
{
	union xhci_trb *trbs;
	/* private to HCD */
	struct xhci_segment *next;
};

struct xhci_ring
{
	BOOL is_event_ring;
	u8 ep_index; /* for transfer rings, the endpoint index this ring is associated with. For event rings, unused and set to 0. */
	u32 num_segs;
	u32 max_packet_size;
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
	dma_free(ctrl->memoryPool, seg->trbs);
	seg->trbs = NULL;

	pool_free(ctrl->memoryPool, seg);
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

	pool_free(ctrl->memoryPool, ring);
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
			le64((dma_addr_t)next->trbs);

		/*
		 * Set the last TRB in the segment to
		 * have a TRB type ID of Link TRB
		 */
		u32 val = le32(prev->trbs[TRBS_PER_SEGMENT - 1].link.control);
		val &= (u32)~TRB_TYPE_BITMASK;
		val |= TRB_TYPE(TRB_LINK);
		prev->trbs[TRBS_PER_SEGMENT - 1].link.control = le32(val);
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
	struct xhci_segment *seg = pool_zalloc(ctrl->memoryPool, sizeof(struct xhci_segment));
	if (!seg)
	{
		Kprintf("pool_zalloc failed for size %lu\n",
				(ULONG)sizeof(struct xhci_segment));
		return NULL;
	}

	seg->trbs = xhci_malloc(ctrl, SEGMENT_SIZE);

	return seg;
}

/**
 * Dynamically grow a transfer ring by inserting num_new_segs new segments
 * immediately after ring->enq_seg.
 * the hardware will follow the updated Link TRB once it drains the current
 * enq_seg.
 *
 * @param ctrl         pointer to the xhci controller
 * @param ring         the transfer ring to grow
 * @param num_new_segs number of segments to add
 * Return: TRUE on success, FALSE if the ring is already at its maximum size
 *         or allocation fails (ring is left unmodified in that case).
 */
BOOL xhci_ring_grow(struct xhci_ctrl *ctrl, struct xhci_ring *ring, u32 num_new_segs)
{
	if (!ring || num_new_segs == 0 ||
	    ring->num_segs + num_new_segs > XHCI_MAX_SEGMENTS_PER_RING)
		return FALSE;

	/*
	 * Allocate and link the new segments as a linear chain.
	 * xhci_link_segments sets seg->next and the Link TRB for each pair.
	 * Each Link TRB is flushed immediately so the hardware will see the
	 * correct pointer once we splice the chain into the ring.
	 */
	struct xhci_segment *new_first = NULL, *new_last = NULL, *prev = NULL;
	for (u32 i = 0; i < num_new_segs; ++i)
	{
		struct xhci_segment *seg = xhci_segment_alloc(ctrl);
		if (!seg)
		{
			/* Free already-allocated segments via the ->next chain */
			struct xhci_segment *s = new_first;
			while (s)
			{
				struct xhci_segment *nxt = s->next;
				xhci_segment_free(ctrl, s);
				s = nxt;
			}
			return FALSE;
		}
		seg->next = NULL;
		if (prev)
		{
			xhci_link_segments(prev, seg, TRUE);
			xhci_flush_cache(&prev->trbs[TRBS_PER_SEGMENT - 1], sizeof(union xhci_trb));
		}
		else
			new_first = seg;
		prev = seg;
		new_last = seg;
	}

	/*
	 * xhci_malloc zeroes TRBs (cycle=0).  When ring->cycle_state==0 the
	 * HC treats cycle=0 as valid — pre-set data TRBs to cycle=1 so the HC
	 * won't process empty slots if it follows the new chain before we have
	 * written real TRBs.  The Link TRB (last slot) is managed separately.
	 */
	if (ring->cycle_state == 0)
	{
		struct xhci_segment *seg = new_first;
		while (seg)
		{
			for (u32 j = 0; j < TRBS_PER_SEGMENT - 1; ++j)
				seg->trbs[j].generic.field[3] |= le32(TRB_CYCLE);
			seg = seg->next;
		}
	}

	/*
	 * Splice the new chain between ring->enq_seg and its current successor.
	 *
	 *   1. Link new_last → old_next (HC cannot reach here yet)
	 *   2. Transfer LINK_TOGGLE from enq_seg to new_last
	 *   3. Re-point enq_seg Link TRB → new_first (HC now enters new chain)
	 *   4. Update the software ->next pointer for enq_seg
	 */
	struct xhci_segment *old_next = ring->enq_seg->next;

	/* Step 1 */
	xhci_link_segments(new_last, old_next, TRUE);
	xhci_flush_cache(&new_last->trbs[TRBS_PER_SEGMENT - 1], sizeof(union xhci_trb));

	/* Step 2 */
	u32 enq_ctrl = le32(ring->enq_seg->trbs[TRBS_PER_SEGMENT - 1].link.control);
	if (enq_ctrl & LINK_TOGGLE)
	{
		enq_ctrl &= ~(u32)LINK_TOGGLE;
		ring->enq_seg->trbs[TRBS_PER_SEGMENT - 1].link.control = le32(enq_ctrl);
		u32 last_ctrl = le32(new_last->trbs[TRBS_PER_SEGMENT - 1].link.control);
		last_ctrl |= LINK_TOGGLE;
		new_last->trbs[TRBS_PER_SEGMENT - 1].link.control = le32(last_ctrl);
		xhci_flush_cache(&new_last->trbs[TRBS_PER_SEGMENT - 1], sizeof(union xhci_trb));
	}

	/* Step 3 */
	ring->enq_seg->trbs[TRBS_PER_SEGMENT - 1].link.segment_ptr =
		le64((dma_addr_t)new_first->trbs);
	xhci_flush_cache(&ring->enq_seg->trbs[TRBS_PER_SEGMENT - 1], sizeof(union xhci_trb));

	/* Step 4 */
	ring->enq_seg->next = new_first;

	ring->num_segs += num_new_segs;
	return TRUE;
}

/**
 * Create a new ring with zero or more segments.
 *
 * Link each segment together into a ring.
 * Set the end flag and the cycle toggle bit on the last segment.
 * See section 4.9.2 and figures 15 and 16 of XHCI spec rev1.0.
 *
 * @param ctrl	pointer to the xhci controller
 * @param num_segs	number of segments in the ring
 * @param link_trbs	flag to indicate whether to link the trbs or NOT
 * @param is_event_ring	flag to indicate whether this ring is an event ring or not, which affects how we determine the last TRB in a segment
 * @param ep_index	endpoint index for transfer rings, unused for event rings
 * @param maxpacketsize maximum packet size for the endpoint, unused for event rings
 * Return: pointer to the newly created RING
 */
struct xhci_ring *xhci_ring_alloc(struct xhci_ctrl *ctrl, u32 num_segs,
								  BOOL link_trbs, BOOL is_event_ring, u8 ep_index, u32 max_packet_size)
{
	u32 remaining = num_segs;

	struct xhci_ring *ring = pool_zalloc(ctrl->memoryPool, sizeof(struct xhci_ring));
	if (!ring)
	{
		Kprintf("pool_zalloc failed for size %lu\n",
				(ULONG)sizeof(struct xhci_ring));
		return NULL;
	}
	ring->num_segs = num_segs;
	ring->is_event_ring = is_event_ring;
	ring->memoryPool = ctrl->memoryPool;
	ring->ep_index = ep_index;
	ring->max_packet_size = max_packet_size;
	if (remaining == 0)
		return ring;

	ring->first_seg = xhci_segment_alloc(ctrl);
	if (!ring->first_seg)
	{
		Kprintf("xhci_segment_alloc failed\n");
		pool_free(ctrl->memoryPool, ring);
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
			le32(LINK_TOGGLE);
	}
	xhci_initialize_ring_info(ring);

	return ring;
}

void xhci_ring_setup_erst(struct xhci_ring *ring, struct xhci_erst *erst, struct xhci_intr_reg *ir_set)
{
	u32 val;
	struct xhci_segment *seg;
	erst->num_entries = XHCI_INITIAL_SEGS_PER_EVENT_RING;

	for (val = 0, seg = ring->first_seg;
		 val < XHCI_INITIAL_SEGS_PER_EVENT_RING;
		 val++)
	{
		struct xhci_erst_entry *entry = &erst->entries[val];
		entry->seg_addr = le64((dma_addr_t)seg->trbs);
		entry->seg_size = le32(TRBS_PER_SEGMENT);
		entry->rsvd = 0;
		seg = seg->next;
	}
	xhci_flush_cache(erst->entries, XHCI_INITIAL_SEGS_PER_EVENT_RING * sizeof(struct xhci_erst_entry));

	/* Update HC event ring dequeue pointer */
	xhci_writeq(&ir_set->erst_dequeue,
				(u64)(uintptr_t)ring->dequeue & (u64)~ERST_PTR_MASK);

	/* set ERST count with the number of entries in the segment table */
	val = mmio_read32(&ir_set->erst_size);
	val &= ERST_SIZE_MASK;
	val |= XHCI_INITIAL_SEGS_PER_EVENT_RING;
	mmio_write32(val, &ir_set->erst_size);

	/* this is the event ring segment table pointer */
	u64 val_64 = xhci_readq(&ir_set->erst_base);
	val_64 &= ERST_PTR_MASK;
	val_64 |= ((dma_addr_t)erst->entries & ~ERST_PTR_MASK);

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
		return le32(trb->link.control) & LINK_TOGGLE;
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
	u32 chain = le32(ring->enqueue->generic.field[3]) & TRB_CHAIN;
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
			next->link.control &= le32(~TRB_CHAIN);
			next->link.control |= le32(chain);

			next->link.control ^= le32(TRB_CYCLE);
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

	trb->field[0] = le32(field0);
	trb->field[1] = le32(field1);
	trb->field[2] = le32(field2);
	trb->field[3] = le32(field3);

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
		next->link.control &= le32(~TRB_CHAIN);

		next->link.control ^= le32(TRB_CYCLE);

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
	if ((le32(event->event_cmd.flags) & TRB_CYCLE) != ring->cycle_state)
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
	xhci_writeq(&ctrl->ir_set->erst_dequeue, ((dma_addr_t)ctrl->event_ring->dequeue) | ERST_EHB);
}

u32 xhci_ring_get_new_dequeue_ptr(struct xhci_ring *ring)
{
	return (u32)ring->enqueue | ring->cycle_state;
}

u32 xhci_ring_get_deq_ptr_for_trb(dma_addr_t trb_addr)
{
	if (!trb_addr)
		return 0;

	union xhci_trb *trb = (union xhci_trb *)(uintptr_t)trb_addr;
	xhci_inval_cache(trb, sizeof(*trb));
	return trb_addr | (le32(trb->generic.field[3]) & TRB_CYCLE);
}

void xhci_ring_patch_trbs_to_noop(dma_addr_t *trb_addrs, u32 trb_count, u32 start_index)
{
	if (!trb_addrs || start_index >= trb_count)
		return;

	for (u32 index = start_index; index < trb_count; ++index)
	{
		union xhci_trb *trb = (union xhci_trb *)(uintptr_t)trb_addrs[index];
		if (!trb)
			return;

		xhci_inval_cache(trb, sizeof(*trb));
		u32 cycle = le32(trb->generic.field[3]) & TRB_CYCLE;
		trb->generic.field[0] = 0;
		trb->generic.field[1] = 0;
		trb->generic.field[2] = 0;
		trb->generic.field[3] = le32(TRB_TYPE(TRB_TR_NOOP) | cycle);
		xhci_flush_cache(trb, sizeof(*trb));
	}
}

u32 xhci_ring_get_max_packet_size(struct xhci_ring *ring)
{
	return ring->max_packet_size;
}

void xhci_ring_set_max_packet_size(struct xhci_ring *ring, u32 max_packet_size)
{
	ring->max_packet_size = max_packet_size;
}

dma_addr_t xhci_ring_enqueue_command(struct xhci_ring *ring, u64 address, u32 slot_id, u8 ep_index, trb_type cmd)
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
											   u64_lo32(address), /* field0 */
											   u64_hi32(address), /* field1 */
											   0,				  /* field2 */
											   field3);			  /* field3 */
	return trb_dma;
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
 * @param ctrl	host controller data structure
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

inline static BOOL ring_has_room(struct xhci_ring *ring, struct ep_context *ep_ctx, u32 needed)
{
	u32 capacity = ring->num_segs * (TRBS_PER_SEGMENT - 1);
	if (capacity == 0)
		return FALSE;
	return xhci_ep_get_active_trb_count(ep_ctx) + needed <= capacity;
}

BOOL xhci_ring_has_room(struct ep_context *ep_ctx, u32 needed_trbs)
{
	if (!ep_ctx)
		return FALSE;
	struct xhci_ring *ring = xhci_ep_get_ring(ep_ctx);
	if (!ring)
		return FALSE;
	return ring_has_room(ring, ep_ctx, needed_trbs);
}

inline static void prime_first_trb(struct xhci_generic_trb *start_trb)
{
	start_trb->field[3] ^= le32(TRB_CYCLE);
	xhci_flush_cache(start_trb, sizeof(struct xhci_generic_trb));
}

inline static void giveback_first_trb(struct usb_device *udev, u8 ep_index,
									  struct xhci_generic_trb *start_trb)
{
	struct xhci_ctrl *ctrl = udev->controller;

	prime_first_trb(start_trb);

	/* Ringing EP doorbell here */
	mmio_write32(DB_VALUE(ep_index, 0), &ctrl->dba->doorbell[udev->slot_id]);

	return;
}

void xhci_ring_giveback(struct usb_device *udev, struct ep_context *ep_ctx)
{
	if (!ep_ctx)
		return;

	struct xhci_ring *ring = xhci_ep_get_ring(ep_ctx);
	if (ring && ring->deferred_giveback)
	{
		giveback_first_trb(udev, ring->ep_index, ring->deferred_giveback);
		ring->deferred_giveback = NULL;
	}
}

inline static dma_addr_t xhci_dma_map(struct xhci_ctrl *ctrl, struct USBIORequest *req, BOOL copy)
{
	if (!req)
		return NULL;

	APTR addr = req->data_buffer;
	u32 size = req->data_buffer_length;

	if (!ctrl || !ctrl->memoryPool || !addr || size == 0)
		return (dma_addr_t)addr;

	/* TODO this should also check if the memory region is on Pistorm RAM */
	if (unlikely(addr > (APTR)0x1FFFFF && (((uintptr_t)addr & DMA_ALIGN_MIN_MASK) == 0)))
	{
		xhci_flush_cache(addr, size);
		return (dma_addr_t)addr;
	}

	u32 alloc_len = ALIGN_UP(size, DMA_ALIGN_MIN);

	void *aligned = NULL;
	u32 bounce_class = REQ_BOUNCE_CLASS_NONE;
	if (alloc_len <= XHCI_BOUNCE_SMALL_SIZE)
	{
		aligned = slab_alloc(&ctrl->bounce_small);
		bounce_class = REQ_BOUNCE_CLASS_SMALL;
	}
	else if (alloc_len <= XHCI_BOUNCE_MED_SIZE)
	{
		aligned = slab_alloc(&ctrl->bounce_med);
		bounce_class = REQ_BOUNCE_CLASS_MED;
	}
	else if (alloc_len <= XHCI_BOUNCE_LARGE_SIZE)
	{
		aligned = slab_alloc(&ctrl->bounce_large);
		bounce_class = REQ_BOUNCE_CLASS_LARGE;
	}
	if (!aligned)
	{
		aligned = dma_alloc(ctrl->memoryPool, DMA_ALIGN_MIN, alloc_len);
		bounce_class = REQ_BOUNCE_CLASS_NONE;
	}
	if (!aligned)
	{
		Kprintf("failed to allocate bounce buffer for %lx len=%lu\n", (ULONG)addr, (ULONG)size);
		return (dma_addr_t)addr;
	}

	req->driver_private_flags = (req->driver_private_flags & ~REQ_BOUNCE_CLASS_MASK) | (bounce_class << REQ_BOUNCE_CLASS_SHIFT) | REQ_DMA_MAPPED;

	if (copy)
	{
		xhci_copy_to_bounce_buffer(addr, aligned, size);
	}
	xhci_flush_cache(aligned, alloc_len);

	req->driver_private_dma_address = aligned;
	return (dma_addr_t)aligned;
}

inline static dma_addr_t xhci_ring_enqueue_setup_trb(struct xhci_ring *ep_ring, struct USBIORequest *io)
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
	if (io->data_buffer_length > 0)
	{
		if (io->setup.bmRequestType & USB_DIR_IN)
			field3 |= TRB_TX_TYPE(TRB_DATA_IN);
		else
			field3 |= TRB_TX_TYPE(TRB_DATA_OUT);
	}

	return xhci_ring_enqueue_trb(ep_ring, TRUE,
								 io->setup.bmRequestType | ((u32)io->setup.bRequest << 8) | ((u32)le16(io->setup.wValue) << 16), /* field 0 */
								 le16(io->setup.wIndex) | ((u32)le16(io->setup.wLength) << 16),									 /* field 1 */
								 TRB_LEN(8) | TRB_INTR_TARGET(0),																 /* field 2 */
								 field3);																						 /* field 3 */
}

inline static dma_addr_t xhci_ring_enqueue_data_trb(struct xhci_ctrl *ctrl, struct xhci_ring *ep_ring, struct USBIORequest *io)
{
	/* If there's data, queue data TRBs */
	/* Only set interrupt on short packet for IN endpoints */

	u32 remainder = xhci_td_remainder(0, io->data_buffer_length, io->data_buffer_length, ep_ring->max_packet_size, TRUE);
	u32 length_field = TRB_LEN(io->data_buffer_length) | TRB_TD_SIZE(remainder) | TRB_INTR_TARGET(0);
	// KprintfH("length_field = %ld, length = %ld,"
	// 		 "xhci_td_remainder(length) = %ld , TRB_INTR_TARGET(0) = %ld\n",
	// 		 length_field, TRB_LEN(io->data_buffer_length),
	// 		 TRB_TD_SIZE(remainder), 0);

	BOOL is_direction_in = (io->setup.bmRequestType & USB_DIR_IN) != 0;
	u64 buf_64 = xhci_dma_map(ctrl, io, !is_direction_in);

	u32 field3 = (is_direction_in) ? TRB_ISP | TRB_DIR_IN | TRB_TYPE(TRB_DATA) : TRB_TYPE(TRB_DATA);
	field3 |= ep_ring->cycle_state;

	return xhci_ring_enqueue_trb(ep_ring, TRUE,
								 u64_lo32(buf_64), /* field 0 */
								 u64_hi32(buf_64), /* field 1 */
								 length_field,	   /* field 2 */
								 field3);		   /* field 3 */
}

inline static dma_addr_t xhci_ring_enqueue_status_trb(struct xhci_ring *ep_ring, struct USBIORequest *io)
{
	/*
	 * Queue status TRB -
	 * see Table 7 and sections 4.11.2.2 and 6.4.1.2.3
	 */

	/* If the device sent data, the status stage is an OUT transfer */
	u32 field3 = (io->data_buffer_length > 0 && io->setup.bmRequestType & USB_DIR_IN) ? 0 : TRB_DIR_IN;

	/* Event on completion */
	field3 |= TRB_IOC | TRB_TYPE(TRB_STATUS) | ep_ring->cycle_state;

	return xhci_ring_enqueue_trb(ep_ring, FALSE,
								 0,					 /* field 0 */
								 0,					 /* field 1 */
								 TRB_INTR_TARGET(0), /* field 2 */
								 field3);			 /* field 3 */
}

inline static void xhci_ring_enqueue_control_trbs(struct xhci_ctrl *ctrl, struct xhci_ring *ep_ring, struct USBIORequest *io, dma_addr_t *td_trb_addrs)
{
	u32 td_trb_index = 0;
	const u32 length = io->data_buffer_length;

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

inline static void xhci_ring_enqueue_non_control_trbs(struct xhci_ring *ep_ring, struct USBIORequest *io, u64 addr, u32 num_trbs, u32 trb_buff_len, dma_addr_t *td_trb_addrs, u32 iso_extra_bits)
{
	// KprintfH("num_trbs = %lu, trb_buff_len = %lu\n", (ULONG)num_trbs, (ULONG)trb_buff_len);
	const BOOL is_iso = io->req.io_Command == CMD_REGISTER_ISOCHRONOUS_HOOKS ||
						io->req.io_Command == CMD_REQUEST_ISOCHRONOUS;

	const u32 length = io->data_buffer_length;
	const u32 isp_for_in = (io->direction == DIRECTION_IN && !is_iso) ? TRB_ISP : 0;

	/* xHCI 4.11.2.3: only the first TRB in an ISO TD carries the ISOC type and
	 * iso-specific bits (Frame ID/SIA, TBC, TLBPC). Chain TRBs are NORMAL.
	 */
	const u32 first_trb_type_bits = (is_iso ? (TRB_TYPE(TRB_ISOC) | iso_extra_bits)
											: TRB_TYPE(TRB_NORMAL));
	const u32 chain_trb_type_bits = TRB_TYPE(TRB_NORMAL); /* no ISP on chain TRBs */

	u32 running_total = 0;
	u32 td_trb_index = 0;

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
		u32 field3;
		/* Don't change the cycle bit of the first TRB until later */
		if (first_trb)
		{
			field3 = first_trb_type_bits;
			first_trb = FALSE;
			if (ep_ring->cycle_state == 0)
				field3 |= TRB_CYCLE;
		}
		else
		{
			field3 = chain_trb_type_bits | ep_ring->cycle_state;
		}

		/*
		 * Chain all the TRBs together; clear the chain bit in the last
		 * TRB to indicate it's the last TRB in the chain.
		 */
		if (num_trbs > 1)
			field3 |= TRB_CHAIN | isp_for_in;
		else
			field3 |= TRB_IOC;

		/* Set the TRB length, TD size, and interrupter fields. */
		u32 remainder = xhci_td_remainder(running_total, trb_buff_len,
										  length, ep_ring->max_packet_size,
										  num_trbs > 1);

		u32 length_field = TRB_LEN(trb_buff_len) | TRB_TD_SIZE(remainder) | TRB_INTR_TARGET(0);

		dma_addr_t last_transfer_trb_addr = xhci_ring_enqueue_trb(ep_ring, (num_trbs > 1),
																  u64_lo32(addr), /* field 0 */
																  u64_hi32(addr), /* field 1 */
																  length_field,	  /* field 2 */
																  field3);		  /* field 3 */
		td_trb_addrs[td_trb_index++] = last_transfer_trb_addr;
		--num_trbs;
		running_total += trb_buff_len;

		/* Calculate length for next transfer */
		addr += trb_buff_len;
		trb_buff_len = (length - running_total < TRB_MAX_BUFF_SIZE) ? (length - running_total) : TRB_MAX_BUFF_SIZE;
	} while (running_total < length);
}

inline static void xhci_ring_finalize_first_trb(struct usb_device *udev, u8 ep_index, struct xhci_ring *ep_ring, struct xhci_generic_trb *start_trb, BOOL defer_doorbell)
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

inline static u32 xhci_ring_calc_num_trbs(struct xhci_ctrl *ctrl, struct USBIORequest *io, u32 *trb_buff_len, u64 *addr)
{
	if (io->req.io_Command == CMD_REQUEST_CONTROL)
		/* 1 TRB for setup, 1 for status, 1 optional for data */
		return (io->data_buffer_length > 0) ? 3 : 2;

	const u32 length = io->data_buffer_length;
	*addr = xhci_dma_map(ctrl, io, io->direction == DIRECTION_OUT);

	/*
	 * How much data is (potentially) left before the 64KB boundary?
	 * XHCI Spec (Table 49 / 6.4.1) requires we avoid spanning that boundary, so
	 * we may need several chained TRBs if the buffer crosses it.
	 */
	u32 running_total = TRB_MAX_BUFF_SIZE - (u64_lo32(*addr) & (TRB_MAX_BUFF_SIZE - 1));
	*trb_buff_len = running_total;
	running_total &= TRB_MAX_BUFF_SIZE - 1;

	u32 num_trbs = 0;
	/* If we already cover bytes in this 64KB chunk, or the transfer is zero length,
	 * we schedule at least one TRB now.
	 */
	if (running_total != 0 || length == 0)
		num_trbs++;

	/* Account for remaining 64KB windows, adding more TRBs as needed. */
	num_trbs += DIV_CEIL(io->data_buffer_length - running_total, TRB_MAX_BUFF_SIZE);
	return num_trbs;
}

static void __attribute__((unused)) xhci_dump_request(const char *tag, const struct USBIORequest *req)
{
	if (!req)
		return;

	const char *pfx = tag ? tag : "";

	Kprintf("%s Request dump:\n", pfx);
	Kprintf("%s  Endpoint=0x%02lx Dir=%s Type=%s\n",
			pfx, (ULONG)req->endpoint,
			(req->direction == DIRECTION_IN) ? "IN" : "OUT",
			(req->req.io_Command == CMD_REQUEST_CONTROL) ? "Control" : (req->req.io_Command == CMD_REQUEST_BULK)			 ? "Bulk"
																   : (req->req.io_Command == CMD_REQUEST_INTERRUPT)			 ? "Interrupt"
																   : (req->req.io_Command == CMD_REQUEST_ISOCHRONOUS)		 ? "Isochronous"
																   : (req->req.io_Command == CMD_REGISTER_ISOCHRONOUS_HOOKS) ? "RT Isochronous"
																															 : "Unknown");
	if (req->req.io_Command == CMD_REQUEST_CONTROL)
		Kprintf("%s  SetupData: bmRequestType=0x%02lx bRequest=0x%02lx wValue=0x%04lx wIndex=0x%04lx wLength=%lu\n",
				pfx, (ULONG)req->setup.bmRequestType, (ULONG)req->setup.bRequest,
				(ULONG)le16(req->setup.wValue), (ULONG)le16(req->setup.wIndex),
				(ULONG)le16(req->setup.wLength));
}

/*
 * xHCI 4.11.2.3: compute TBC and TLBPC for an ISO TD.
 * Pre-1.0 controllers leave both fields RsvdZ; older controllers can't burst anyway.
 */
static inline u32 iso_burst_bits(struct xhci_ctrl *ctrl, struct usb_device *udev,
								 struct ep_context *ep_ctx, u32 td_length)
{
	if (ctrl->hci_version < 0x100)
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

inline static s8 enqueue_td_internal(struct usb_device *udev, struct USBIORequest *io, u32 timeout_ms, BOOL defer_doorbell, u32 iso_extra_bits)
{
	// #ifdef DEBUG_HIGH
	// 	xhci_dump_request("[xhci-ring] xhci_ring_enqueue_td: ", io);
	// #endif
	struct xhci_ctrl *ctrl = udev->controller;

	const u8 ep_index = xhci_ep_index_from_parts(io->endpoint, io->direction);
	struct ep_context *udev_ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
	if (!udev_ep_ctx)
	{
		Kprintf("No ep context for ep %d\n", ep_index);
		return ERR_BAD_PARAMETERS;
	}

	enum ep_state cur_state = xhci_ep_get_state(udev_ep_ctx);
	if (cur_state == USB_DEV_EP_STATE_ABORTING ||
		cur_state == USB_DEV_EP_STATE_RESETTING ||
		cur_state == USB_DEV_EP_STATE_FAILED)
	{
		KprintfH("Cannot submit transfer, ep in state %d\n", cur_state);
		xhci_ep_enqueue(udev_ep_ctx, io);
		return ERR_NO_ERROR;
	}

#ifdef DEBUG_CONTEXT
	xhci_dump_slot_ctx("[xhci-ring] xhci_ring_enqueue_td:", udev, FALSE);
	xhci_dump_ep_ctx("[xhci-ring] xhci_ring_enqueue_td:", udev, io->endpoint);
#endif

	u32 trb_buff_len = 0; // non-control only
	u64 addr = 0;		  // non-control only
	u32 num_trbs = xhci_ring_calc_num_trbs(ctrl, io, &trb_buff_len, &addr);
	// KprintfH("Calculated num_trbs=%lu trb_buff_len=%lu addr=%lx\n", (ULONG)num_trbs, (ULONG)trb_buff_len, (ULONG)addr);

	struct xhci_ring *ep_ring = xhci_ep_get_ring(udev_ep_ctx);
	if (!ep_ring)
	{
		Kprintf("No ring for ep %d\n", ep_index);
		return ERR_HCI_ERROR;
	}

	if (!ring_has_room(ep_ring, udev_ep_ctx, num_trbs + 1))
	{
		KprintfH("Ring full ep=%lu needed %lu TRBs, attempting grow\n", (ULONG)ep_index, (ULONG)num_trbs);
		if (!xhci_ring_grow(ctrl, ep_ring, XHCI_SEGMENTS_PER_RING)) {
			KprintfH("Ring grow failed, queueing request\n");
			xhci_ep_enqueue(udev_ep_ctx, io);
			return ERR_NO_ERROR;
		}
		KprintfH("Ring grew, retrying room check\n");
		if (!ring_has_room(ep_ring, udev_ep_ctx, num_trbs + 1)) {
			KprintfH("Still no room after grow, queueing\n");
			xhci_ep_enqueue(udev_ep_ctx, io);
			return ERR_NO_ERROR;
		}
	}

	dma_addr_t *td_trb_addrs;
	if (likely(num_trbs <= XHCI_TD_SMALL_TRBS))
		td_trb_addrs = slab_alloc(&ctrl->trb_addr_slab);
	else
		td_trb_addrs = pool_alloc(ctrl->memoryPool, num_trbs * sizeof(dma_addr_t));
	if (!td_trb_addrs)
	{
		Kprintf("Failed to alloc TD TRB list\n");
		io->req.io_Error = ERR_ALLOC_ERROR;
		return ERR_ALLOC_ERROR;
	}

	/*
	 * XXX: Calling routine prepare_ring() called in place of
	 * prepare_trasfer() as there in 'Linux' since we are not
	 * maintaining multiple TDs/transfer at the same time.
	 */
	prepare_ring(ep_ring);

	if (io->req.io_Command == CMD_REQUEST_CONTROL)
		xhci_ring_enqueue_control_trbs(ctrl, ep_ring, io, td_trb_addrs);
	else
	{
		if (io->req.io_Command == CMD_REQUEST_ISOCHRONOUS ||
			io->req.io_Command == CMD_REGISTER_ISOCHRONOUS_HOOKS)
			iso_extra_bits |= iso_burst_bits(ctrl, udev, udev_ep_ctx, io->data_buffer_length);
		xhci_ring_enqueue_non_control_trbs(ep_ring, io, addr, num_trbs, trb_buff_len, td_trb_addrs, iso_extra_bits);
	}

	xhci_ep_set_receiving(udev_ep_ctx, io, td_trb_addrs, timeout_ms, num_trbs);
	xhci_ring_finalize_first_trb(udev, ep_index, ep_ring, (struct xhci_generic_trb *)td_trb_addrs[0], defer_doorbell);

	return ERR_NO_ERROR;
}

s8 xhci_ring_enqueue_td(struct usb_device *udev, struct USBIORequest *io, u32 timeout_ms, BOOL defer_doorbell)
{
	/* ISO transfers scheduled here use SIA; RT ISO callers go through xhci_ring_enqueue_td_at_frame. */
	return enqueue_td_internal(udev, io, timeout_ms, defer_doorbell, TRB_SIA);
}

s8 xhci_ring_enqueue_td_at_frame(struct usb_device *udev, struct USBIORequest *io, u32 timeout_ms, BOOL defer_doorbell, u16 frame)
{
	return enqueue_td_internal(udev, io, timeout_ms, defer_doorbell, TRB_FRAME_ID(frame));
}
