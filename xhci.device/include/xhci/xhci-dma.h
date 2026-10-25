/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * DMA mapping of transfer buffers (xhci-dma.c).
 *
 * A span is one buffer as the controller gets to it.  How is its mapping: in
 * place (DIRECT), through a bounce buffer (BOUNCE), or, for a bulk IN into a
 * buffer that doesn't own its cache lines, in place but for the partial lines
 * at either end, which go through an edge slot (SPLIT).  The TRBs address the
 * span's extents: the contiguous DMA ranges that makes.  A link piece is the
 * few bytes before a Link TRB that TD emission sends through a slot of the
 * span's own, so the ring segment ends on a packet boundary.
 *
 * Two levels:
 *  - the span API works on any buffer; the RT ISO path maps its staging
 *    buffers with it directly;
 *  - the request API maps a transfer's data (xf->data_span) with the flags the
 *    request implies.  Everything after that is the span API on
 *    &xf->data_span.
 */

#ifndef __XHCI_DMA_H
#define __XHCI_DMA_H

#include <exec/types.h>
#include <types.h>
#include <bits.h>

struct xhci_ctrl;
struct xhci_xfer;

/* ---- Spans ---------------------------------------------------------------- */

/* How a span reaches the controller. */
enum xhci_dma_mapping
{
	XHCI_DMA_NONE = 0, /* unmapped: a zeroed span, or a zero-length one */
	XHCI_DMA_DIRECT,   /* in place */
	XHCI_DMA_BOUNCE,   /* the whole payload through a bounce buffer */
	XHCI_DMA_SPLIT,    /* IN: the line-aligned middle in place, the partial
						* lines at either end through the edge slot */
};

/* Where a bounce buffer came from, so it goes back there. */
enum xhci_bounce_pool
{
	XHCI_BOUNCE_DMA_ALLOC = 0, /* dma_alloc(): too big for the slabs, or they ran dry */
	XHCI_BOUNCE_SLAB_SMALL,    /* ctrl->bounce_small */
	XHCI_BOUNCE_SLAB_MED,      /* ctrl->bounce_med */
	XHCI_BOUNCE_SLAB_LARGE,    /* ctrl->bounce_large */
};

/* Mapping flags (xhci_dma_span_prepare()). */
#define XHCI_MAP_TO_DEVICE BIT(0)  /* OUT: the controller reads the buffer */
#define XHCI_MAP_OWNS_LINES BIT(1) /* the buffer owns its cache lines whatever its
									* length (RT ISO staging) */
#define XHCI_MAP_MAY_SPLIT BIT(2)  /* a misaligned IN may split (bulk) */

/* Shortest IN that splits: below it, copying the payload costs less than two
 * extra TRBs and an edge slot. */
#define XHCI_SPLIT_MIN 4096

/* The edge slot, a bounce_small slot, holds the head at offset 0 and the tail
 * at this one: a cache line further, so their maintenance never shares a
 * line. */
#define XHCI_EDGE_TAIL_OFFSET DMA_ALIGN_MIN

/* A buffer as mapped for the controller. */
struct xhci_dma_span
{
	APTR buffer;	/* the caller's */
	u32 length;		/* bytes to transfer */
	BOOL to_device; /* OUT: the controller reads the buffer */
	enum xhci_dma_mapping mapping;

	/* BOUNCE: the whole payload's buffer; SPLIT: the edge slot; else NULL */
	APTR bounce_buf;
	enum xhci_bounce_pool bounce_pool;

	/* SPLIT: the partial lines before and after the line-aligned middle,
	 * each shorter than DMA_ALIGN_MIN */
	u16 head_len;
	u16 tail_len;

	/* The link piece (xhci_dma_span_link_piece()); slot NULL = none. */
	struct
	{
		APTR slot;	/* a bounce_link slot, held until unmap */
		u32 offset; /* where the piece sits in the transfer */
		u16 length;
	} link;
};

/* One contiguous DMA range of a span, as TRBs address it (TRB emission cuts it
 * further at 64 KB boundaries). */
struct xhci_dma_extent
{
	dma_addr_t addr;
	u32 length;
};

#define XHCI_DMA_EXTENTS_MAX 3 /* a split's head, middle and tail */

/* ---- Span API ----------------------------------------------------------------
 * In the order a transfer uses it.  Needs ctrl->xfer_lock (the bounce pools),
 * except xhci_dma_span_sync(), which touches only memory the transfer owns:
 * a caller can run it with the lock dropped.
 */

/* Choose the mapping and allocate its bounce buffer or edge slot; no copy and
 * no cache maintenance yet.  flags: XHCI_MAP_*.  FALSE: no bounce buffer to be
 * had, or no buffer for a length; reject the transfer. */
BOOL xhci_dma_span_prepare(struct xhci_ctrl *ctrl, struct xhci_dma_span *span,
						   APTR buffer, u32 length, u32 flags);

/* Before the DMA: copy an OUT's payload into its bounce buffer, and clean (OUT)
 * or invalidate (IN) the lines the controller will use. */
void xhci_dma_span_sync(struct xhci_ctrl *ctrl, const struct xhci_dma_span *span);

/* xhci_dma_span_prepare() and xhci_dma_span_sync() in one. */
BOOL xhci_dma_span_map(struct xhci_ctrl *ctrl, struct xhci_dma_span *span,
					   APTR buffer, u32 length, u32 flags);

/* The extents the TRBs address, at most XHCI_DMA_EXTENTS_MAX; returns how
 * many.  An unmapped span gives one empty extent: a zero-length TD. */
u32 xhci_dma_span_extents(const struct xhci_dma_span *span, struct xhci_dma_extent *extents);

/* TD emission: send length bytes at offset of the transfer through a link slot
 * of the span's own.  Returns the slot for the TRB to address, or NULL: the
 * span has one already, or there is none to be had. */
APTR xhci_dma_span_link_piece(struct xhci_ctrl *ctrl, struct xhci_dma_span *span,
							  u32 offset, u32 length);

/* After the DMA, or instead of it: copy the delivered bytes of an IN home
 * (0: none, or the transfer was abandoned) and free the bounce buffer, edge
 * slot and link slot.  Leaves the span unmapped; an unmapped span is left
 * alone. */
void xhci_dma_span_unmap(struct xhci_ctrl *ctrl, struct xhci_dma_span *span, u32 delivered);

/* ---- Request API -------------------------------------------------------------
 * Maps xf->data_span with the flags the request implies: only bulk may split.
 * The rest is the span API on &xf->data_span.
 */

/* Prepare the request's span with the lock held, to sync it with the lock
 * dropped (xhci_dma_span_sync(); the direct submit path). */
BOOL xhci_dma_req_prepare(struct xhci_ctrl *ctrl, struct xhci_xfer *req, BOOL to_device);

/* Prepare and sync the request's span, unless it is mapped already: a request
 * requeued for ring room keeps its mapping and its copied payload. */
BOOL xhci_dma_req_map(struct xhci_ctrl *ctrl, struct xhci_xfer *req, BOOL to_device);

#endif /* __XHCI_DMA_H */
