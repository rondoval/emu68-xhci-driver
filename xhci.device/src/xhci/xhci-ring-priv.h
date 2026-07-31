/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Ring internals shared by exactly two translation units: xhci-ring.c (ring
 * and segment mechanics) and xhci-submit.c (TD submission).  Deliberately in
 * src/, not include/ — nothing else may look inside a ring.  Everything here
 * is static inline so each TU gets full inlining of the enqueue helpers.
 */

#ifndef __XHCI_RING_PRIV_H
#define __XHCI_RING_PRIV_H

#include <xhci/xhci.h>
#include <xhci/xhci-ring.h>

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
	u16 stream_id; /* SS bulk stream ring: the stream id this ring serves (doorbell target); 0 = default ring */
	u32 num_segs;
	u32 max_packet_size;
	u32 queued_trbs; /* TRBs of in-flight TDs on THIS ring (per-stream-accurate
	                  * room accounting; xhci-ring.c owns every access -
	                  * xhci_ring_reserve_trbs / _release_trbs / _has_room) */

	/* This ring's in-flight TDs.  Owned and tracked by the endpoint layer
	 * (xhci-td.c is the only code that looks inside); transfer rings only,
	 * NULL for event/command rings. */
	struct TransferDescriptorList *td_list;

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
 * Is this TRB a link TRB or was the last TRB the last TRB in this event ring
 * segment?  I.e. would the updated event TRB pointer step off the end of the
 * event seg ?
 *
 * @param ring	pointer to the ring
 * @param seg	poniter to the segment to which TRB belongs
 * @param trb	poniter to the ring trb
 * Return: 1 if this TRB a link TRB else 0
 */
static inline int last_trb(struct xhci_ring *ring,
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
static inline BOOL last_trb_on_last_seg(struct xhci_ring *ring,
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
static inline void inc_enq(struct xhci_ring *ring, BOOL more_trbs_coming)
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
			/* device only READS ring TRBs: clean-only keeps the line valid
			 * for the CPU's next rewrite (barrier kept — the command path
			 * rings its doorbell right after enqueue, with no later flush) */
			cache_pre_dma(next, sizeof(union xhci_trb), DMA_ReadFromRAM);
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
static inline dma_addr_t xhci_ring_enqueue_trb_flags(struct xhci_ring *ring,
													 BOOL more_trbs_coming,
													 u32 field0, u32 field1, u32 field2, u32 field3,
													 ULONG cache_flags)
{
	struct xhci_generic_trb *trb = &ring->enqueue->generic;

	trb->field[0] = le32(field0);
	trb->field[1] = le32(field1);
	trb->field[2] = le32(field2);
	trb->field[3] = le32(field3);

	cache_pre_dma(trb, sizeof(struct xhci_generic_trb), cache_flags);

	inc_enq(ring, more_trbs_coming);

	return (dma_addr_t)trb;
}

/* Barrier-per-TRB variant: for the command ring, whose doorbell rings right
 * after a single enqueue with no later flush to close a batch. */
static inline dma_addr_t xhci_ring_enqueue_trb(struct xhci_ring *ring,
											   BOOL more_trbs_coming,
											   u32 field0, u32 field1, u32 field2, u32 field3)
{
	return xhci_ring_enqueue_trb_flags(ring, more_trbs_coming,
									   field0, field1, field2, field3,
									   DMA_ReadFromRAM);
}

/* NoSync variant for TD construction: the hardware cannot see these TRBs
 * until prime_first_trb flips the first TRB's cycle bit — and THAT flush
 * (non-NoSync) is the batch's closing barrier before the doorbell. */
static inline dma_addr_t xhci_ring_enqueue_trb_ns(struct xhci_ring *ring,
												  BOOL more_trbs_coming,
												  u32 field0, u32 field1, u32 field2, u32 field3)
{
	return xhci_ring_enqueue_trb_flags(ring, more_trbs_coming,
									   field0, field1, field2, field3,
									   DMA_ReadFromRAM | DMAF_NoSync);
}

/**
 * Does various checks on the endpoint ring, and makes it ready
 * to queue num_trbs.
 *
 * @param ep_ring	pointer to the EP Transfer Ring
 * Return: none
 */
static inline void prepare_ring(struct xhci_ring *ep_ring)
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

		/* device-read link TRB: clean-only, barrier kept (see inc_enq) */
		cache_pre_dma(next, sizeof(union xhci_trb), DMA_ReadFromRAM);

		/* Toggle the cycle bit after the last ring segment. */
		if (last_trb_on_last_seg(ep_ring, ep_ring->enq_seg, next))
			ep_ring->cycle_state = (ep_ring->cycle_state ? 0 : 1);
		ep_ring->enq_seg = ep_ring->enq_seg->next;
		ep_ring->enqueue = ep_ring->enq_seg->trbs;
		next = ep_ring->enqueue;
	}
}

#endif /* __XHCI_RING_PRIV_H */
