// SPDX-License-Identifier: GPL-2.0-only
/*
 * DMA mapping of transfer buffers; the vocabulary (span, mapping, extent,
 * link piece) is in xhci-dma.h.
 *
 * The PCIe engine reaches only Emu68 (Pi DRAM) RAM, so Chip RAM and Zorro or
 * accelerator Fast RAM always bounce.  A device-writes-RAM (IN) DMA is
 * invalidated before and after, which is safe only on whole cache lines: a
 * caller's line shared with other data would lose that data's writes.  So an
 * IN into a buffer that doesn't own its lines either bounces whole or, for
 * bulk, splits: the line-aligned middle is DMA'd in place, and the partial
 * lines at either end go through an edge slot and are copied in by the CPU.
 * The caller's boundary lines are never invalidated.  An OUT is only cleaned,
 * which is safe on any byte range.
 */

#define __NOLIBBASE__
#define EXEC_BASE_NAME SysBase /* a local in every function, from its context's sysBase */
#include <config.h>
#include <debug.h>
#include <memory.h>

#include <xhci/xhci.h>
#include <xhci/xhci-dma.h>
#include <xhci/xhci-xfer.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-dma] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef TRACE
#undef KprintfT
#define KprintfT(fmt, ...) PrintPistorm("[xhci-dma] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

/**
 * bounce_alloc() - Allocate a bounce buffer from the smallest pool that holds it.
 * @ctrl: host controller.
 * @length: bytes needed.
 * @pool: set to the pool it came from.
 *
 * A slab that has run dry, or a length past the largest slab, falls back to
 * dma_alloc().
 *
 * Return: the buffer, line-aligned and owning whole lines, or NULL.
 */
static void *bounce_alloc(struct xhci_ctrl *ctrl, u32 length, enum xhci_bounce_pool *pool)
{
	void *buf = NULL;

	if (length <= XHCI_BOUNCE_SMALL_SIZE)
	{
		buf = slab_alloc(&ctrl->bounce_small);
		*pool = XHCI_BOUNCE_SLAB_SMALL;
	}
	else if (length <= XHCI_BOUNCE_MED_SIZE)
	{
		buf = slab_alloc(&ctrl->bounce_med);
		*pool = XHCI_BOUNCE_SLAB_MED;
	}
	else if (length <= XHCI_BOUNCE_LARGE_SIZE)
	{
		buf = slab_alloc(&ctrl->bounce_large);
		*pool = XHCI_BOUNCE_SLAB_LARGE;
	}
	if (!buf)
	{
		buf = dma_alloc(ctrl->dmaPool, DMA_ALIGN_MIN, length);
		*pool = XHCI_BOUNCE_DMA_ALLOC;
	}
	return buf;
}

/**
 * bounce_free() - Return a bounce buffer to its pool.
 * @ctrl: host controller.
 * @buf: the buffer.
 * @pool: the pool bounce_alloc() reported.
 */
static void bounce_free(struct xhci_ctrl *ctrl, void *buf, enum xhci_bounce_pool pool)
{
	switch (pool)
	{
	case XHCI_BOUNCE_SLAB_SMALL:
		slab_free(&ctrl->bounce_small, buf);
		break;
	case XHCI_BOUNCE_SLAB_MED:
		slab_free(&ctrl->bounce_med, buf);
		break;
	case XHCI_BOUNCE_SLAB_LARGE:
		slab_free(&ctrl->bounce_large, buf);
		break;
	case XHCI_BOUNCE_DMA_ALLOC:
		dma_free(ctrl->dmaPool, buf);
		break;
	}
}

/* A split span's middle: the bytes DMA'd in place, between head and tail. */
static inline u32 split_middle_len(const struct xhci_dma_span *span)
{
	return span->length - span->head_len - span->tail_len;
}

/**
 * xhci_dma_span_prepare() - Choose how a buffer reaches the controller, and allocate.
 * @ctrl: host controller.
 * @span: filled in.
 * @buffer: the caller's buffer.
 * @length: bytes to transfer.
 * @flags: XHCI_MAP_*.
 *
 * In order: a buffer the PCIe engine can't reach bounces whole; an OUT, or an
 * IN that owns its cache lines (XHCI_MAP_OWNS_LINES, or both ends on a line),
 * goes direct; an IN with XHCI_MAP_MAY_SPLIT and at least XHCI_SPLIT_MIN bytes
 * splits, if an edge slot is to be had; any other IN bounces whole.  No payload
 * copy and no cache maintenance here (xhci_dma_span_sync()), so a caller can
 * run those without the lock.
 *
 * Return: FALSE if no bounce buffer could be had (or @buffer is NULL with a
 * length): the transfer must be rejected, not DMA'd.  A zero length maps to
 * nothing (XHCI_DMA_NONE) and succeeds.
 */
BOOL xhci_dma_span_prepare(struct xhci_ctrl *ctrl, struct xhci_dma_span *span,
						   APTR buffer, u32 length, u32 flags)
{
	*span = (struct xhci_dma_span){
		.buffer = buffer,
		.length = length,
		.to_device = (flags & XHCI_MAP_TO_DEVICE) != 0,
	};

	if (length == 0)
		return TRUE;
	if (!buffer)
		return FALSE;

	const BOOL reachable = dma_addr_reachable(&ctrl->dma_ctx, buffer, length);
	const BOOL owns_lines = span->to_device || (flags & XHCI_MAP_OWNS_LINES) ||
							((((uintptr_t)buffer | length) & DMA_ALIGN_MIN_MASK) == 0);
	if (reachable && owns_lines)
	{
		span->mapping = XHCI_DMA_DIRECT;
		return TRUE;
	}

	if (reachable && (flags & XHCI_MAP_MAY_SPLIT) && length >= XHCI_SPLIT_MIN)
	{
		PERF_T0(split_t0);
		void *edge = slab_alloc(&ctrl->bounce_small);
		if (edge)
		{
			/* the partial lines: from the start up to the first line
			 * boundary, and from the last one to the end */
			const uintptr_t start = (uintptr_t)buffer;
			const uintptr_t end = start + length;
			span->head_len = (u16)(ALIGN_UP(start, DMA_ALIGN_MIN) - start);
			span->tail_len = (u16)(end & DMA_ALIGN_MIN_MASK);
			span->bounce_buf = edge;
			span->bounce_pool = XHCI_BOUNCE_SLAB_SMALL;
			span->mapping = XHCI_DMA_SPLIT;
			PERF_ADD(&ctrl->perf, XP_DMA_SPLIT, split_t0);
			return TRUE;
		}
		/* no edge slot: bounce whole */
	}

	span->bounce_buf = bounce_alloc(ctrl, length, &span->bounce_pool);
	if (!span->bounce_buf)
	{
		Kprintf("no bounce buffer for %lx len=%lu; rejecting transfer\n", (ULONG)buffer, (ULONG)length);
		return FALSE;
	}
	span->mapping = XHCI_DMA_BOUNCE;
	return TRUE;
}

/**
 * xhci_dma_span_sync() - Get a prepared span ready for the DMA.
 * @ctrl: host controller.
 * @span: a prepared span.
 *
 * An OUT's payload is copied into its bounce buffer, and the lines the
 * controller reads are cleaned (DMA_ReadFromRAM).  An IN's are invalidated
 * (DMA_WriteToRAM): their CPU content is disposable, since the device
 * overwrites it, so dirty lines (a bounce slot last used for an OUT, a reused
 * buffer) are dropped instead of written back just before being replaced.
 * That is safe only on lines the transfer owns: the direct gate in
 * xhci_dma_span_prepare(), the line-owned bounce slots, and a split's
 * line-aligned middle.  On a short IN, the bytes past the actual length are
 * whatever RAM held: undefined, as in USB (class drivers honour the actual
 * length).  Touches only memory the transfer owns, so it needs no lock.
 */
void xhci_dma_span_sync(struct xhci_ctrl *ctrl, const struct xhci_dma_span *span)
{
	struct ExecBase *SysBase = ctrl->sysBase;
	const ULONG dir = span->to_device ? DMA_ReadFromRAM : DMA_WriteToRAM;

	switch (span->mapping)
	{
	case XHCI_DMA_DIRECT:
		cache_pre_dma(span->buffer, span->length, dir);
		break;
	case XHCI_DMA_BOUNCE:
		if (span->to_device)
			memcpy(span->bounce_buf, span->buffer, span->length);
		cache_pre_dma(span->bounce_buf, span->length, dir);
		break;
	case XHCI_DMA_SPLIT:
		cache_pre_dma(span->bounce_buf, XHCI_EDGE_TAIL_OFFSET + DMA_ALIGN_MIN, DMA_WriteToRAM);
		cache_pre_dma((UBYTE *)span->buffer + span->head_len, split_middle_len(span), DMA_WriteToRAM);
		break;
	case XHCI_DMA_NONE:
		break;
	}
}

/**
 * xhci_dma_span_map() - Prepare and sync a span in one call.
 * @ctrl: host controller.
 * @span: filled in.
 * @buffer: the caller's buffer.
 * @length: bytes to transfer.
 * @flags: XHCI_MAP_*.
 *
 * Return: as xhci_dma_span_prepare().
 */
BOOL xhci_dma_span_map(struct xhci_ctrl *ctrl, struct xhci_dma_span *span,
					   APTR buffer, u32 length, u32 flags)
{
	if (!xhci_dma_span_prepare(ctrl, span, buffer, length, flags))
		return FALSE;
	xhci_dma_span_sync(ctrl, span);
	return TRUE;
}

/**
 * xhci_dma_span_extents() - List the DMA ranges of a span, as TRBs address them.
 * @span: a prepared span.
 * @extents: XHCI_DMA_EXTENTS_MAX entries, filled in.
 *
 * Direct and whole-bounce spans are one extent; a split is head (edge slot),
 * middle (in place) and tail (edge slot), leaving out an empty head or tail.
 * An unmapped span gives one empty extent: a zero-length TD.
 *
 * Return: the number of entries filled in.
 */
u32 xhci_dma_span_extents(const struct xhci_dma_span *span, struct xhci_dma_extent *extents)
{
	switch (span->mapping)
	{
	case XHCI_DMA_SPLIT:
	{
		u32 n = 0;
		if (span->head_len)
			extents[n++] = (struct xhci_dma_extent){(dma_addr_t)span->bounce_buf, span->head_len};
		extents[n++] = (struct xhci_dma_extent){(dma_addr_t)span->buffer + span->head_len,
												split_middle_len(span)};
		if (span->tail_len)
			extents[n++] = (struct xhci_dma_extent){(dma_addr_t)span->bounce_buf + XHCI_EDGE_TAIL_OFFSET,
													span->tail_len};
		return n;
	}
	case XHCI_DMA_BOUNCE:
		extents[0] = (struct xhci_dma_extent){(dma_addr_t)span->bounce_buf, span->length};
		return 1;
	case XHCI_DMA_DIRECT:
		extents[0] = (struct xhci_dma_extent){(dma_addr_t)span->buffer, span->length};
		return 1;
	case XHCI_DMA_NONE:
		break;
	}
	extents[0] = (struct xhci_dma_extent){(dma_addr_t)span->buffer, 0};
	return 1;
}

/**
 * xhci_dma_span_link_piece() - Send a piece of a span through a slot of its own.
 * @ctrl: host controller.
 * @span: the mapped span; an OUT's is DIRECT or BOUNCE.
 * @offset: where the piece sits in the transfer.
 * @length: bytes, at most XHCI_LINK_SLOT_SIZE.
 *
 * TD emission's alignment at a Link TRB: a TRB before the link that is too
 * short to end on a packet boundary is replaced by one that covers the bytes up
 * to the boundary from a bounce_link slot, which the span holds until
 * xhci_dma_span_unmap().  An OUT's bytes are copied in from the span's source
 * (the caller's buffer, or the whole bounce holding its copy) and cleaned; an
 * IN's slot is invalidated, and its bytes are copied home at unmap.
 *
 * Return: the slot, or NULL if the span holds one already (a TD crossing
 * several Link TRBs) or there is none to be had; the caller then leaves the
 * TRB as it is.
 */
APTR xhci_dma_span_link_piece(struct xhci_ctrl *ctrl, struct xhci_dma_span *span,
							  u32 offset, u32 length)
{
	struct ExecBase *SysBase = ctrl->sysBase;

	if (span->link.slot)
		return NULL;

	APTR slot = slab_alloc(&ctrl->bounce_link);
	if (!slot)
		return NULL;

	span->link.slot = slot;
	span->link.offset = offset;
	span->link.length = (u16)length;
	if (span->to_device)
	{
		const UBYTE *src = (span->mapping == XHCI_DMA_BOUNCE) ? span->bounce_buf : span->buffer;
		memcpy(slot, src + offset, length);
		cache_pre_dma(slot, length, DMA_ReadFromRAM);
	}
	else
		cache_pre_dma(slot, length, DMA_WriteToRAM);
	return slot;
}

/* The bytes of a span's transfer there are to copy home: none for an OUT, at
 * most the span's length for an IN. */
static inline u32 to_copy_home(const struct xhci_dma_span *span, u32 delivered)
{
	if (span->to_device)
		return 0;
	return (delivered < span->length) ? delivered : span->length;
}

/**
 * copy_piece_home() - Copy the delivered bytes of one piece of a transfer home.
 * @ctrl: host controller.
 * @span: the span the piece belongs to.
 * @device_copy: where the device wrote the piece: the bounce buffer, an edge
 *	slot half or the link slot.
 * @piece_offset: where the piece starts in the transfer, and so in the
 *	caller's buffer.
 * @piece_length: the piece's length in bytes.
 * @delivered: bytes of the whole transfer the device delivered.
 *
 * Copies the part of the piece below @delivered into the caller's buffer,
 * invalidating @device_copy first so the copy reads what the device wrote.
 */
static void copy_piece_home(struct xhci_ctrl *ctrl, const struct xhci_dma_span *span,
							APTR device_copy, u32 piece_offset, u32 piece_length, u32 delivered)
{
	struct ExecBase *SysBase = ctrl->sysBase;
	if (delivered <= piece_offset || piece_length == 0)
		return;

	const u32 n = (delivered - piece_offset < piece_length) ? delivered - piece_offset : piece_length;
	cache_post_dma(device_copy, n, 0);
	memcpy((UBYTE *)span->buffer + piece_offset, device_copy, n);
}

/**
 * xhci_dma_span_unmap() - End a span's DMA: copy an IN's data home, free its buffers.
 * @ctrl: host controller.
 * @span: the span; left unmapped (XHCI_DMA_NONE).  An unmapped one is left
 *	alone, so a request's span (xf->data_span) can be unmapped on any exit path.
 * @delivered: bytes the device delivered (0: none, or the transfer was
 *	abandoned).
 *
 * Of an IN, only the delivered bytes are copied home, each piece from where
 * the device wrote it (copy_piece_home()); a direct or split span's in-place
 * lines are invalidated again.  The link piece comes last: the device wrote it
 * only to its slot, so the copies before put stale bytes over it, and the
 * invalidates before would drop it.  The bounce buffer, edge slot and link slot go back to
 * their pools, an OUT's too.
 */
void xhci_dma_span_unmap(struct xhci_ctrl *ctrl, struct xhci_dma_span *span, u32 delivered)
{
	struct ExecBase *SysBase = ctrl->sysBase;
	delivered = to_copy_home(span, delivered);

	switch (span->mapping)
	{
	case XHCI_DMA_DIRECT:
		if (delivered)
			cache_post_dma(span->buffer, span->length, 0);
		break;
	case XHCI_DMA_BOUNCE:
		copy_piece_home(ctrl, span, span->bounce_buf, 0, span->length, delivered);
		bounce_free(ctrl, span->bounce_buf, span->bounce_pool);
		break;
	case XHCI_DMA_SPLIT:
		if (delivered)
			cache_post_dma((UBYTE *)span->buffer + span->head_len, split_middle_len(span), 0);
		copy_piece_home(ctrl, span, span->bounce_buf, 0, span->head_len, delivered);
		copy_piece_home(ctrl, span, (UBYTE *)span->bounce_buf + XHCI_EDGE_TAIL_OFFSET,
						span->length - span->tail_len, span->tail_len, delivered);
		bounce_free(ctrl, span->bounce_buf, span->bounce_pool);
		break;
	case XHCI_DMA_NONE:
		break;
	}

	if (span->link.slot)
	{
		copy_piece_home(ctrl, span, span->link.slot, span->link.offset, span->link.length, delivered);
		slab_free(&ctrl->bounce_link, span->link.slot);
		span->link.slot = NULL;
	}
	span->bounce_buf = NULL;
	span->mapping = XHCI_DMA_NONE;
}

/**
 * xhci_dma_req_prepare() - Prepare a request's span, lock held.
 * @ctrl: host controller.
 * @req: the request; its data and data_length are mapped into req->data_span.
 * @to_device: TRUE for an OUT (a control transfer's direction comes from its
 *	setup packet).
 *
 * Only bulk requests may split.  The caller syncs the span afterwards,
 * xhci_dma_span_sync(), with the lock dropped if it likes.
 *
 * Return: as xhci_dma_span_prepare().
 */
BOOL xhci_dma_req_prepare(struct xhci_ctrl *ctrl, struct xhci_xfer *req, BOOL to_device)
{
	const u32 flags = (to_device ? XHCI_MAP_TO_DEVICE : 0) |
					  ((req->type == UHCD_EPTYPE_BULK) ? XHCI_MAP_MAY_SPLIT : 0);
	return xhci_dma_span_prepare(ctrl, &req->data_span, req->data, req->data_length, flags);
}

/**
 * xhci_dma_req_map() - Map a request's span unless it is mapped already.
 * @ctrl: host controller.
 * @req: the request.
 * @to_device: TRUE for an OUT.
 *
 * A request prepared by the direct submit, or requeued for ring room, keeps
 * its mapping (and its copied bounce payload).
 *
 * Return: as xhci_dma_span_prepare().
 */
BOOL xhci_dma_req_map(struct xhci_ctrl *ctrl, struct xhci_xfer *req, BOOL to_device)
{
	if (req->data_span.mapping != XHCI_DMA_NONE)
		return TRUE;
	if (!xhci_dma_req_prepare(ctrl, req, to_device))
		return FALSE;
	xhci_dma_span_sync(ctrl, &req->data_span);
	return TRUE;
}
