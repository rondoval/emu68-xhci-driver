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
#include "xhci-ring-priv.h"

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-ring] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef TRACE
#undef KprintfT
#define KprintfT(fmt, ...) PrintPistorm("[xhci-ring] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

/**
 * frees the "segment" pointer passed
 *
 * @param ptr	pointer to "segement" to be freed
 * Return: none
 */
static void xhci_segment_free(struct xhci_ctrl *ctrl, struct xhci_segment *seg)
{
	if (seg->trbs)
		slab_free(&ctrl->seg_slab, seg->trbs);
	seg->trbs = NULL;

	pool_free(ctrl->metaPool, seg);
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

	pool_free(ctrl->metaPool, ring);
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

	ring->queued_trbs = 0;
	ring->deferred_giveback = NULL;
}

/**
 * Allocates a generic ring segment from the ring pool, sets the dma address,
 * initializes the segment to zero, and sets the private next pointer to NULL.
 * Section 4.11.1.1:
 * "All components of all Command and Transfer TRBs shall be initialized to '0'"
 *
 * @param	align alignment requirement for the segment backing store
 * Return: pointer to the newly allocated SEGMENT
 */
static struct xhci_segment *xhci_segment_alloc(struct xhci_ctrl *ctrl)
{
	struct xhci_segment *seg = pool_zalloc(ctrl->metaPool, sizeof(struct xhci_segment));
	if (!seg)
	{
		Kprintf("pool_zalloc failed for size %lu\n",
				(ULONG)sizeof(struct xhci_segment));
		return NULL;
	}

	/* Segments come from a per-controller slab whose slots are seg_size-sized and
	 * seg_size-aligned (4096, or 8192 under XHCI_QUIRK_TRB_OVERFETCH), so each slot is
	 * boundary-safe (never crosses a 64 KB page) by construction.  Ring math still uses
	 * SEGMENT_SIZE; the doubled slot under the quirk just gives the VL805 owned,
	 * harmless memory to prefetch. */
	seg->trbs = slab_zalloc(&ctrl->seg_slab);
	if (!seg->trbs)
	{
		pool_free(ctrl->metaPool, seg);
		return NULL;
	}
	/* MUST stay clean+invalidate (flags 0): segments serve transfer rings AND
	 * event rings — the latter are xHC-WRITTEN, and this is their pre-arm. */
	cache_pre_dma(seg->trbs, ctrl->seg_slab.obj_size, 0);

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
			cache_pre_dma(&prev->trbs[TRBS_PER_SEGMENT - 1], sizeof(union xhci_trb), DMA_ReadFromRAM);
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
	cache_pre_dma(&new_last->trbs[TRBS_PER_SEGMENT - 1], sizeof(union xhci_trb), DMA_ReadFromRAM);

	/* Step 2 */
	u32 enq_ctrl = le32(ring->enq_seg->trbs[TRBS_PER_SEGMENT - 1].link.control);
	if (enq_ctrl & LINK_TOGGLE)
	{
		enq_ctrl &= ~(u32)LINK_TOGGLE;
		ring->enq_seg->trbs[TRBS_PER_SEGMENT - 1].link.control = le32(enq_ctrl);
		u32 last_ctrl = le32(new_last->trbs[TRBS_PER_SEGMENT - 1].link.control);
		last_ctrl |= LINK_TOGGLE;
		new_last->trbs[TRBS_PER_SEGMENT - 1].link.control = le32(last_ctrl);
		cache_pre_dma(&new_last->trbs[TRBS_PER_SEGMENT - 1], sizeof(union xhci_trb), DMA_ReadFromRAM);
	}

	/* Step 3 */
	ring->enq_seg->trbs[TRBS_PER_SEGMENT - 1].link.segment_ptr =
		le64((dma_addr_t)new_first->trbs);
	cache_pre_dma(&ring->enq_seg->trbs[TRBS_PER_SEGMENT - 1], sizeof(union xhci_trb), DMA_ReadFromRAM);

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

	struct xhci_ring *ring = pool_zalloc(ctrl->metaPool, sizeof(struct xhci_ring));
	if (!ring)
	{
		Kprintf("pool_zalloc failed for size %lu\n",
				(ULONG)sizeof(struct xhci_ring));
		return NULL;
	}
	ring->num_segs = num_segs;
	ring->is_event_ring = is_event_ring;
	ring->ep_index = ep_index;
	ring->max_packet_size = max_packet_size;
	if (remaining == 0)
		return ring;

	ring->first_seg = xhci_segment_alloc(ctrl);
	if (!ring->first_seg)
	{
		Kprintf("xhci_segment_alloc failed\n");
		pool_free(ctrl->metaPool, ring);
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
	const u32 entry_count = erst->num_entries;

	for (val = 0, seg = ring->first_seg;
		 val < entry_count;
		 val++)
	{
		struct xhci_erst_entry *entry = &erst->entries[val];
		entry->seg_addr = le64((dma_addr_t)seg->trbs);
		entry->seg_size = le32(TRBS_PER_SEGMENT);
		entry->rsvd = 0;
		seg = seg->next;
	}
	cache_pre_dma(erst->entries, entry_count * sizeof(struct xhci_erst_entry), DMA_ReadFromRAM);

	/* Update HC event ring dequeue pointer */
	xhci_writeq(&ir_set->erst_dequeue,
				(u64)(uintptr_t)ring->dequeue & (u64)~ERST_PTR_MASK);

	/* set ERST count with the number of entries in the segment table */
	val = mmio_read32(&ir_set->erst_size);
	val &= ERST_SIZE_MASK;
	val |= entry_count;
	mmio_write32(val, &ir_set->erst_size);

	/* this is the event ring segment table pointer */
	u64 val_64 = xhci_readq(&ir_set->erst_base);
	val_64 &= ERST_PTR_MASK;
	val_64 |= ((dma_addr_t)erst->entries & ~ERST_PTR_MASK);

	xhci_writeq(&ir_set->erst_base, val_64);
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
 * Checks if there is a new event to handle on the event ring.
 *
 * @param ring	pointer to the event RING
 * Return: pointer to the event TRB if ready, NULL otherwise
 */
union xhci_trb *xhci_ring_get_event_trb(struct xhci_ring *ring)
{
	cache_post_dma(ring->dequeue, sizeof(union xhci_trb), 0);
	union xhci_trb *event = ring->dequeue;

	/* Does the HC or OS own the TRB? */
	if ((le32(event->event_cmd.flags) & TRB_CYCLE) != ring->cycle_state)
		return NULL;

	return event;
}

/**
 * Retires a handled event TRB by advancing the software dequeue pointer.
 * Must run exactly once at the end of each event handler; the TRB must not
 * be touched afterwards.  The hardware learns the new dequeue position from
 * the single xhci_ring_ack_events() closing the drain.
 *
 * @param ctrl	Host controller data structure
 * Return: none
 */
void xhci_ring_consume_event(struct xhci_ctrl *ctrl)
{
	inc_deq(ctrl->event_ring);
}

/* Write ERDP (+EHB clear) once for a whole drain — one MMIO pair instead of
 * one per event. */
void xhci_ring_ack_events(struct xhci_ctrl *ctrl)
{
	xhci_intr_ack_events(ctrl->ir_set, (dma_addr_t)ctrl->event_ring->dequeue);
}

/* Dequeue-pointer value (with DCS in bit 0) for Set TR Dequeue, taken from the
 * software enqueue state.  Never returns a link TRB: inc_enq() parks the
 * enqueue pointer on the link TRB between TDs, but pointing the hardware
 * dequeue there - while spec-legal - is never useful and the VL805 mis-parses
 * it (Linux XHCI_AVOID_DQ_ON_LINK; also XHCI_VLI_HUB_TT_QUIRK would gate here
 * on VL805 fw < 0x0138C0, not implemented).  The cycle state is software-owned
 * throughout, so the controller's broken DCS write-back
 * (XHCI_EP_CTX_BROKEN_DCS) cannot affect us. */
u32 xhci_ring_get_new_dequeue_ptr(struct xhci_ring *ring)
{
	union xhci_trb *deq = ring->enqueue;
	struct xhci_segment *seg = ring->enq_seg;
	u32 cycle = ring->cycle_state;

	while (last_trb(ring, seg, deq))
	{
		if (last_trb_on_last_seg(ring, seg, deq))
			cycle ^= 1U;
		seg = seg->next;
		deq = seg->trbs;
	}

	return (u32)deq | cycle;
}

u32 xhci_ring_get_deq_ptr_for_trb(dma_addr_t trb_addr)
{
	if (!trb_addr)
		return 0;

	union xhci_trb *trb = (union xhci_trb *)(uintptr_t)trb_addr;
	cache_post_dma(trb, sizeof(*trb), 0);
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

		cache_post_dma(trb, sizeof(*trb), 0);
		u32 cycle = le32(trb->generic.field[3]) & TRB_CYCLE;
		trb->generic.field[0] = 0;
		trb->generic.field[1] = 0;
		trb->generic.field[2] = 0;
		trb->generic.field[3] = le32(TRB_TYPE(TRB_TR_NOOP) | cycle);
		cache_pre_dma(trb, sizeof(*trb), DMA_ReadFromRAM);
	}
}

void xhci_ring_set_max_packet_size(struct xhci_ring *ring, u32 max_packet_size)
{
	ring->max_packet_size = max_packet_size;
}

dma_addr_t xhci_ring_enqueue_command(struct xhci_ring *ring, u64 address, u32 slot_id, u8 ep_index, u16 stream_id, trb_type cmd)
{
	prepare_ring(ring);

	u32 field3 = TRB_TYPE(cmd) | SLOT_ID_FOR_TRB(slot_id) | ring->cycle_state;

	/*
	 * Only 'reset endpoint', 'stop endpoint' and 'set TR dequeue pointer'
	 * commands need endpoint id encoded.
	 */
	if (cmd >= TRB_RESET_EP && cmd <= TRB_SET_DEQ)
		field3 |= EP_ID_FOR_TRB(ep_index);

	/* Set TR Dequeue carries the stream ring being repositioned (6.4.3.9). */
	u32 field2 = (cmd == TRB_SET_DEQ) ? STREAM_ID_FOR_TRB(stream_id) : 0;

	dma_addr_t trb_dma = xhci_ring_enqueue_trb(ring, FALSE,
											   u64_lo32(address), /* field0 */
											   u64_hi32(address), /* field1 */
											   field2,			  /* field2 */
											   field3);			  /* field3 */
	return trb_dma;
}

void xhci_ring_set_stream_id(struct xhci_ring *ring, u16 stream_id)
{
	ring->stream_id = stream_id;
}

