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

 #ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#else
#include <proto/exec.h>
#endif

#include <compat.h>
#include <debug.h>

#include <usb.h>
#include <xhci.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-mem] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#define CACHELINE_SIZE		64

/**
 * flushes the address passed till the length
 *
 * @param addr	pointer to memory region to be flushed
 * @param len	the length of the cache line to be flushed
 * Return: none
 */
void xhci_flush_cache(uintptr_t addr, u32 len)
{
	if (addr == NULL || len == 0)
		return;

	CachePreDMA((APTR)addr, &len, 0);
	// flush_dcache_range(addr & ~(CACHELINE_SIZE - 1),
	// 			ALIGN(addr + len, CACHELINE_SIZE));
}

/**
 * invalidates the address passed till the length
 *
 * @param addr	pointer to memory region to be invalidates
 * @param len	the length of the cache line to be invalidated
 * Return: none
 */
void xhci_inval_cache(uintptr_t addr, u32 len)
{
	if (addr == NULL || len == 0)
		return;

	// invalidate_dcache_range(addr & ~(CACHELINE_SIZE - 1),
	// 			ALIGN(addr + len, CACHELINE_SIZE));
	CachePostDMA((APTR)addr, &len, 0);
}

/**
 * frees the "segment" pointer passed
 *
 * @param ptr	pointer to "segement" to be freed
 * Return: none
 */
static void xhci_segment_free(struct xhci_ctrl *ctrl, struct xhci_segment *seg)
{
	xhci_dma_unmap(ctrl, seg->dma, SEGMENT_SIZE);
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
static void xhci_ring_free(struct xhci_ctrl *ctrl, struct xhci_ring *ring)
{
	struct xhci_segment *seg;
	struct xhci_segment *first_seg;

	if (!ring) {
		Kprintf("Ring is NULL, nothing to free\n");
		return;
	}

	first_seg = ring->first_seg;
	seg = first_seg->next;
	while (seg != first_seg) {
		struct xhci_segment *next = seg->next;
		xhci_segment_free(ctrl, seg);
		seg = next;
	}
	xhci_segment_free(ctrl, first_seg);

	FreeVecPooled(ctrl->memoryPool, ring);
}

/**
 * Free the scratchpad buffer array and scratchpad buffers
 *
 * @ctrl	host controller data structure
 * Return:	none
 */
static void xhci_scratchpad_free(struct xhci_ctrl *ctrl)
{
	struct xhci_hccr *hccr = ctrl->hccr;
	int num_sp;

	if (!ctrl->scratchpad)
		return;

	num_sp = HCS_MAX_SCRATCHPAD(xhci_readl(&hccr->cr_hcsparams2));
	xhci_dma_unmap(ctrl, ctrl->scratchpad->sp_array[0],
		       num_sp * ctrl->page_size);
	xhci_dma_unmap(ctrl, ctrl->dcbaa->dev_context_ptrs[0],
		       num_sp * sizeof(u64));
	ctrl->dcbaa->dev_context_ptrs[0] = 0;

	memalign_free(ctrl->memoryPool, ctrl->scratchpad->scratchpad);
	memalign_free(ctrl->memoryPool, ctrl->scratchpad->sp_array);
	FreeVecPooled(ctrl->memoryPool, ctrl->scratchpad);
	ctrl->scratchpad = NULL;
}

/**
 * frees the "xhci_container_ctx" pointer passed
 *
 * @param ptr	pointer to "xhci_container_ctx" to be freed
 * Return: none
 */
static void xhci_free_container_ctx(struct xhci_ctrl *ctrl,
				    struct xhci_container_ctx *ctx)
{
	xhci_dma_unmap(ctrl, ctx->dma, ctx->size);
	memalign_free(ctrl->memoryPool, ctx->bytes);
	FreeVecPooled(ctrl->memoryPool, ctx);
}

/**
 * frees the virtual devices for "xhci_ctrl" pointer passed
 *
 * @param ptr	pointer to "xhci_ctrl" whose virtual devices are to be freed
 * Return: none
 */
static void xhci_free_virt_devices(struct xhci_ctrl *ctrl)
{
	int i;
	int slot_id;
	struct xhci_virt_device *virt_dev;

	/*
	 * refactored here to loop through all virt_dev
	 * Slot ID 0 is reserved
	 */
	for (slot_id = 0; slot_id < MAX_HC_SLOTS; slot_id++) {
		virt_dev = ctrl->devs[slot_id];
		if (!virt_dev)
			continue;

		ctrl->dcbaa->dev_context_ptrs[slot_id] = 0;

		for (i = 0; i < 31; ++i)
			if (virt_dev->eps[i].ring)
				xhci_ring_free(ctrl, virt_dev->eps[i].ring);

		if (virt_dev->in_ctx)
			xhci_free_container_ctx(ctrl, virt_dev->in_ctx);
		if (virt_dev->out_ctx)
			xhci_free_container_ctx(ctrl, virt_dev->out_ctx);

		FreeVecPooled(ctrl->memoryPool, virt_dev);
		/* make sure we are pointing to NULL */
		ctrl->devs[slot_id] = NULL;
	}
}

/**
 * frees all the memory allocated
 *
 * @param ptr	pointer to "xhci_ctrl" to be cleaned up
 * Return: none
 */
void xhci_cleanup(struct xhci_ctrl *ctrl)
{
	xhci_ring_free(ctrl, ctrl->event_ring);
	xhci_ring_free(ctrl, ctrl->cmd_ring);
	xhci_scratchpad_free(ctrl);
	xhci_free_virt_devices(ctrl);
	xhci_dma_unmap(ctrl, ctrl->erst.erst_dma_addr,
		       sizeof(struct xhci_erst_entry) * ERST_NUM_SEGS);
	memalign_free(ctrl->memoryPool, ctrl->erst.entries);
	xhci_dma_unmap(ctrl, ctrl->dcbaa->dma,
		       sizeof(struct xhci_device_context_array));
	memalign_free(ctrl->memoryPool, ctrl->dcbaa);
	_memset(ctrl, '\0', sizeof(struct xhci_ctrl));
}

/**
 * Malloc the aligned memory
 *
 * @param size	size of memory to be allocated
 * Return: allocates the memory and returns the aligned pointer
 */
static void *xhci_malloc(struct xhci_ctrl *ctrl, unsigned int size)
{
	void *ptr;
	size_t cacheline_size = max(XHCI_ALIGNMENT, CACHELINE_SIZE);

	ptr = memalign(ctrl->memoryPool, cacheline_size, ALIGN(size, cacheline_size));
	if( !ptr )
	{
		Kprintf("memalign failed for size %lu\n", (ULONG)size);
		return NULL;
	}
	_memset(ptr, '\0', size);

	xhci_flush_cache((uintptr_t)ptr, size);

	return ptr;
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
	u32 val;

	if (!prev || !next)
		return;
	prev->next = next;
	if (link_trbs) {
		prev->trbs[TRBS_PER_SEGMENT-1].link.segment_ptr =
			LE64(next->dma);

		/*
		 * Set the last TRB in the segment to
		 * have a TRB type ID of Link TRB
		 */
		val = LE32(prev->trbs[TRBS_PER_SEGMENT-1].link.control);
		val &= ~TRB_TYPE_BITMASK;
		val |= TRB_TYPE(TRB_LINK);
		prev->trbs[TRBS_PER_SEGMENT-1].link.control = LE32(val);
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
	struct xhci_segment *seg;

	seg = AllocVecPooled(ctrl->memoryPool, sizeof(struct xhci_segment));
	if (!seg) {
		Kprintf("AllocVecPooled failed for size %lu\n",
			(ULONG)sizeof(struct xhci_segment));
		return NULL;
	}

	seg->trbs = xhci_malloc(ctrl, SEGMENT_SIZE);
	seg->dma = xhci_dma_map(ctrl, seg->trbs, SEGMENT_SIZE);

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
				  BOOL link_trbs)
{
	struct xhci_ring *ring;
	struct xhci_segment *prev;

	ring = AllocVecPooled(ctrl->memoryPool, sizeof(struct xhci_ring));
	if (!ring) {
		Kprintf("AllocVecPooled failed for size %lu\n",
			(ULONG)sizeof(struct xhci_ring));
		return NULL;
	}

	if (num_segs == 0)
		return ring;

	ring->first_seg = xhci_segment_alloc(ctrl);
	if (!ring->first_seg) {
		Kprintf("xhci_segment_alloc failed\n");
		FreeVecPooled(ctrl->memoryPool, ring);
		return NULL;
	}

	num_segs--;

	prev = ring->first_seg;
	while (num_segs > 0) {
		struct xhci_segment *next;

		next = xhci_segment_alloc(ctrl);
		if (!next) {
			Kprintf("xhci_segment_alloc failed\n");
			xhci_ring_free(ctrl, ring);
			return NULL;
		}

		xhci_link_segments(prev, next, link_trbs);

		prev = next;
		num_segs--;
	}
	xhci_link_segments(prev, ring->first_seg, link_trbs);
	if (link_trbs) {
		/* See section 4.9.2.1 and 6.4.4.1 */
		prev->trbs[TRBS_PER_SEGMENT-1].link.control |=
					LE32(LINK_TOGGLE);
	}
	xhci_initialize_ring_info(ring);

	return ring;
}

/**
 * Set up the scratchpad buffer array and scratchpad buffers
 *
 * @ctrl	host controller data structure
 * Return:	-ENOMEM if buffer allocation fails, 0 on success
 */
static int xhci_scratchpad_alloc(struct xhci_ctrl *ctrl)
{
	struct xhci_hccr *hccr = ctrl->hccr;
	struct xhci_hcor *hcor = ctrl->hcor;
	struct xhci_scratchpad *scratchpad;
	uint64_t val_64;
	int num_sp;
	uint32_t page_size;
	void *buf;
	int i;

	num_sp = HCS_MAX_SCRATCHPAD(xhci_readl(&hccr->cr_hcsparams2));
	if (!num_sp)
		return 0;

	scratchpad = AllocVecPooled(ctrl->memoryPool,
					sizeof(struct xhci_scratchpad));
	if (!scratchpad)
		goto fail_sp;
	ctrl->scratchpad = scratchpad;

	scratchpad->sp_array = xhci_malloc(ctrl, num_sp * sizeof(u64));
	if (!scratchpad->sp_array)
		goto fail_sp2;

	val_64 = xhci_dma_map(ctrl, scratchpad->sp_array,
			      num_sp * sizeof(u64));
	ctrl->dcbaa->dev_context_ptrs[0] = LE64(val_64);

	xhci_flush_cache((uintptr_t)&ctrl->dcbaa->dev_context_ptrs[0],
		sizeof(ctrl->dcbaa->dev_context_ptrs[0]));

	page_size = xhci_readl(&hcor->or_pagesize) & 0xffff;
	for (i = 0; i < 16; i++) {
		if ((0x1 & page_size) != 0)
			break;
		page_size = page_size >> 1;
	}
	if(i==16) {
		Kprintf("Invalid page size\n");
		goto fail_sp3;
	}

	ctrl->page_size = 1 << (i + 12);
	buf = memalign(ctrl->memoryPool, ctrl->page_size, num_sp * ctrl->page_size);
	if (!buf)
		goto fail_sp3;
	_memset(buf, '\0', num_sp * ctrl->page_size);
	xhci_flush_cache((uintptr_t)buf, num_sp * ctrl->page_size);

	scratchpad->scratchpad = buf;
	val_64 = xhci_dma_map(ctrl, buf, num_sp * ctrl->page_size);
	for (i = 0; i < num_sp; i++) {
		scratchpad->sp_array[i] = LE64(val_64);
		val_64 += ctrl->page_size;
	}

	xhci_flush_cache((uintptr_t)scratchpad->sp_array,
			 sizeof(u64) * num_sp);

	return 0;

fail_sp3:
	memalign_free(ctrl->memoryPool, scratchpad->sp_array);

fail_sp2:
	FreeVecPooled(ctrl->memoryPool, scratchpad);
	ctrl->scratchpad = NULL;

fail_sp:
	return -ENOMEM;
}

/**
 * Allocates the Container context
 *
 * @param ctrl	Host controller data structure
 * @param type type of XHCI Container Context
 * Return: NULL if failed else pointer to the context on success
 */
static struct xhci_container_ctx
		*xhci_alloc_container_ctx(struct xhci_ctrl *ctrl, int type)
{
	struct xhci_container_ctx *ctx;

	ctx = AllocVecPooled(ctrl->memoryPool, sizeof(struct xhci_container_ctx));
	if (!ctx) {
		Kprintf("Failed to allocate container context\n");
		return NULL;
	}

	if ((type != XHCI_CTX_TYPE_DEVICE) && (type != XHCI_CTX_TYPE_INPUT)) {
		Kprintf("Invalid context type\n");
		FreeVecPooled(ctrl->memoryPool, ctx);
		return NULL;
	}

	ctx->type = type;
	ctx->size = (MAX_EP_CTX_NUM + 1) *
			CTX_SIZE(xhci_readl(&ctrl->hccr->cr_hccparams));
	if (type == XHCI_CTX_TYPE_INPUT)
		ctx->size += CTX_SIZE(xhci_readl(&ctrl->hccr->cr_hccparams));

	ctx->bytes = xhci_malloc(ctrl, ctx->size);
	ctx->dma = xhci_dma_map(ctrl, ctx->bytes, ctx->size);

	return ctx;
}

/**
 * Allocating virtual device
 *
 * @param udev	pointer to USB deivce structure
 * Return: 0 on success else -1 on failure
 */
int xhci_alloc_virt_device(struct xhci_ctrl *ctrl, unsigned int slot_id)
{
	u64 byte_64 = 0;
	struct xhci_virt_device *virt_dev;

	/* Slot ID 0 is reserved */
	if (ctrl->devs[slot_id]) {
		Kprintf("Virt dev for slot[%ld] already allocated\n", slot_id);
		return -EEXIST;
	}

	Kprintf("xhci_alloc_virt_device: slot_id=%ld\n", (ULONG)slot_id);
	ctrl->devs[slot_id] = AllocVecPooled(ctrl->memoryPool,
					sizeof(struct xhci_virt_device));

	if (!ctrl->devs[slot_id]) {
		Kprintf("Failed to allocate virtual device\n");
		return -ENOMEM;
	}

	_memset(ctrl->devs[slot_id], 0, sizeof(struct xhci_virt_device));
	virt_dev = ctrl->devs[slot_id];

	/* Allocate the (output) device context that will be used in the HC. */
	virt_dev->out_ctx = xhci_alloc_container_ctx(ctrl,
					XHCI_CTX_TYPE_DEVICE);
	if (!virt_dev->out_ctx) {
		Kprintf("Failed to allocate out context for virt dev\n");
		return -ENOMEM;
	}
	KprintfH("xhci_alloc_virt_device: out_ctx bytes=%lx dma=%lx size=%ld\n",
		(ULONG)virt_dev->out_ctx->bytes, (ULONG)virt_dev->out_ctx->dma, (ULONG)virt_dev->out_ctx->size);

	/* Allocate the (input) device context for address device command */
	virt_dev->in_ctx = xhci_alloc_container_ctx(ctrl,
					XHCI_CTX_TYPE_INPUT);
	if (!virt_dev->in_ctx) {
		Kprintf("Failed to allocate in context for virt dev\n");
		return -ENOMEM;
	}
	Kprintf("xhci_alloc_virt_device: in_ctx bytes=%lx dma=%lx size=%ld\n",
		(ULONG)virt_dev->in_ctx->bytes, (ULONG)virt_dev->in_ctx->dma, (ULONG)virt_dev->in_ctx->size);

	/* Allocate endpoint 0 ring */
	virt_dev->eps[0].ring = xhci_ring_alloc(ctrl, 1, true);
	if (!virt_dev->eps[0].ring) {
		Kprintf("xhci_alloc_virt_device: EP0 ring alloc failed\n");
		return -ENOMEM;
	}
	KprintfH("xhci_alloc_virt_device: ep0 ring first_seg=%lx dma=%lx cycle=%ld\n",
		(ULONG)virt_dev->eps[0].ring->first_seg,
		(ULONG)virt_dev->eps[0].ring->first_seg->dma,
		(ULONG)virt_dev->eps[0].ring->cycle_state);

	byte_64 = virt_dev->out_ctx->dma;

	/* Point to output device context in dcbaa. */
	ctrl->dcbaa->dev_context_ptrs[slot_id] = LE64(byte_64);

	xhci_flush_cache((uintptr_t)&ctrl->dcbaa->dev_context_ptrs[slot_id],
			 sizeof(__le64));
	KprintfH("xhci_alloc_virt_device: DCBAA[%ld]=%lx (dcbaap=%lx)\n",
		(ULONG)slot_id,
		(ULONG)LE64(ctrl->dcbaa->dev_context_ptrs[slot_id]),
		(ULONG)ctrl->dcbaa->dma);
	return 0;
}

/**
 * Allocates the necessary data structures
 * for XHCI host controller
 *
 * @param ctrl	Host controller data structure
 * @param hccr	pointer to HOST Controller Control Registers
 * @param hcor	pointer to HOST Controller Operational Registers
 * Return: 0 if successful else -1 on failure
 */
int xhci_mem_init(struct xhci_ctrl *ctrl, struct xhci_hccr *hccr,
					struct xhci_hcor *hcor)
{
	uint64_t val_64;
	uint64_t trb_64;
	uint32_t val;
	uint64_t deq;
	int i;
	struct xhci_segment *seg;

	/* DCBAA initialization */
	ctrl->dcbaa = xhci_malloc(ctrl, sizeof(struct xhci_device_context_array));
	if (ctrl->dcbaa == NULL) {
		Kprintf("unable to allocate DCBA\n");
		return -ENOMEM;
	}

	ctrl->dcbaa->dma = xhci_dma_map(ctrl, ctrl->dcbaa,
				sizeof(struct xhci_device_context_array));
	/* Set the pointer in DCBAA register */
	xhci_writeq(&hcor->or_dcbaap, ctrl->dcbaa->dma);

	/* Command ring control pointer register initialization */
	ctrl->cmd_ring = xhci_ring_alloc(ctrl, 1, true);

	/* Set the address in the Command Ring Control register */
	trb_64 = ctrl->cmd_ring->first_seg->dma;
	val_64 = xhci_readq(&hcor->or_crcr);
	val_64 = (val_64 & (u64) CMD_RING_RSVD_BITS) |
		(trb_64 & (u64) ~CMD_RING_RSVD_BITS) |
		ctrl->cmd_ring->cycle_state;
	xhci_writeq(&hcor->or_crcr, val_64);

	/* write the address of db register */
	val = xhci_readl(&hccr->cr_dboff);
	val &= DBOFF_MASK;
	ctrl->dba = (struct xhci_doorbell_array *)((char *)hccr + val);

	/* write the address of runtime register */
	val = xhci_readl(&hccr->cr_rtsoff);
	val &= RTSOFF_MASK;
	ctrl->run_regs = (struct xhci_run_regs *)((char *)hccr + val);

	/* writting the address of ir_set structure */
	ctrl->ir_set = &ctrl->run_regs->ir_set[0];

	/* Event ring does not maintain link TRB */
	ctrl->event_ring = xhci_ring_alloc(ctrl, ERST_NUM_SEGS, false);
	ctrl->erst.entries = xhci_malloc(ctrl, sizeof(struct xhci_erst_entry) *
					 ERST_NUM_SEGS);
	ctrl->erst.erst_dma_addr = xhci_dma_map(ctrl, ctrl->erst.entries,
			sizeof(struct xhci_erst_entry) * ERST_NUM_SEGS);

	ctrl->erst.num_entries = ERST_NUM_SEGS;

	for (val = 0, seg = ctrl->event_ring->first_seg;
			val < ERST_NUM_SEGS;
			val++) {
		struct xhci_erst_entry *entry = &ctrl->erst.entries[val];
		trb_64 = seg->dma;
		entry->seg_addr = LE64(trb_64);
		entry->seg_size = LE32(TRBS_PER_SEGMENT);
		entry->rsvd = 0;
		seg = seg->next;
	}
	xhci_flush_cache((uintptr_t)ctrl->erst.entries,
			 ERST_NUM_SEGS * sizeof(struct xhci_erst_entry));

	deq = xhci_trb_virt_to_dma(ctrl->event_ring->deq_seg,
				   ctrl->event_ring->dequeue);

	/* Update HC event ring dequeue pointer */
	xhci_writeq(&ctrl->ir_set->erst_dequeue,
				(u64)deq & (u64)~ERST_PTR_MASK);

	/* set ERST count with the number of entries in the segment table */
	val = xhci_readl(&ctrl->ir_set->erst_size);
	val &= ERST_SIZE_MASK;
	val |= ERST_NUM_SEGS;
	xhci_writel(&ctrl->ir_set->erst_size, val);

	/* this is the event ring segment table pointer */
	val_64 = xhci_readq(&ctrl->ir_set->erst_base);
	val_64 &= ERST_PTR_MASK;
	val_64 |= ctrl->erst.erst_dma_addr & ~ERST_PTR_MASK;

	xhci_writeq(&ctrl->ir_set->erst_base, val_64);

	/* set up the scratchpad buffer array and scratchpad buffers */
	xhci_scratchpad_alloc(ctrl);

	/* initializing the virtual devices to NULL */
	for (i = 0; i < MAX_HC_SLOTS; ++i)
		ctrl->devs[i] = NULL;

	/*
	 * Just Zero'ing this register completely,
	 * or some spurious Device Notification Events
	 * might screw things here.
	 */
	xhci_writel(&hcor->or_dnctrl, 0x0);

	return 0;
}

/**
 * Give the input control context for the passed container context
 *
 * @param ctx	pointer to the context
 * Return: pointer to the Input control context data
 */
struct xhci_input_control_ctx
		*xhci_get_input_control_ctx(struct xhci_container_ctx *ctx)
{
	if (ctx->type != XHCI_CTX_TYPE_INPUT) {
		Kprintf("Invalid context type\n");
		return NULL;
	}
	return (struct xhci_input_control_ctx *)ctx->bytes;
}

/**
 * Give the slot context for the passed container context
 *
 * @param ctrl	Host controller data structure
 * @param ctx	pointer to the context
 * Return: pointer to the slot control context data
 */
struct xhci_slot_ctx *xhci_get_slot_ctx(struct xhci_ctrl *ctrl,
				struct xhci_container_ctx *ctx)
{
	if (ctx->type == XHCI_CTX_TYPE_DEVICE)
		return (struct xhci_slot_ctx *)ctx->bytes;

	return (struct xhci_slot_ctx *)
		(ctx->bytes + CTX_SIZE(xhci_readl(&ctrl->hccr->cr_hccparams)));
}

/**
 * Gets the EP context from based on the ep_index
 *
 * @param ctrl	Host controller data structure
 * @param ctx	context container
 * @param ep_index	index of the endpoint
 * Return: pointer to the End point context
 */
struct xhci_ep_ctx *xhci_get_ep_ctx(struct xhci_ctrl *ctrl,
				    struct xhci_container_ctx *ctx,
				    unsigned int ep_index)
{
	/* increment ep index by offset of start of ep ctx array */
	ep_index++;
	if (ctx->type == XHCI_CTX_TYPE_INPUT)
		ep_index++;

	return (struct xhci_ep_ctx *)
		(ctx->bytes +
		(ep_index * CTX_SIZE(xhci_readl(&ctrl->hccr->cr_hccparams))));
}

/**
 * Copy output xhci_ep_ctx to the input xhci_ep_ctx copy.
 * Useful when you want to change one particular aspect of the endpoint
 * and then issue a configure endpoint command.
 *
 * @param ctrl	Host controller data structure
 * @param in_ctx contains the input context
 * @param out_ctx contains the input context
 * @param ep_index index of the end point
 * Return: none
 */
void xhci_endpoint_copy(struct xhci_ctrl *ctrl,
			struct xhci_container_ctx *in_ctx,
			struct xhci_container_ctx *out_ctx,
			unsigned int ep_index)
{
	struct xhci_ep_ctx *out_ep_ctx;
	struct xhci_ep_ctx *in_ep_ctx;

	out_ep_ctx = xhci_get_ep_ctx(ctrl, out_ctx, ep_index);
	in_ep_ctx = xhci_get_ep_ctx(ctrl, in_ctx, ep_index);

	in_ep_ctx->ep_info = out_ep_ctx->ep_info;
	in_ep_ctx->ep_info2 = out_ep_ctx->ep_info2;
	in_ep_ctx->deq = out_ep_ctx->deq;
	in_ep_ctx->tx_info = out_ep_ctx->tx_info;
}

/**
 * Copy output xhci_slot_ctx to the input xhci_slot_ctx.
 * Useful when you want to change one particular aspect of the endpoint
 * and then issue a configure endpoint command.
 * Only the context entries field matters, but
 * we'll copy the whole thing anyway.
 *
 * @param ctrl	Host controller data structure
 * @param in_ctx contains the inpout context
 * @param out_ctx contains the inpout context
 * Return: none
 */
void xhci_slot_copy(struct xhci_ctrl *ctrl, struct xhci_container_ctx *in_ctx,
					struct xhci_container_ctx *out_ctx)
{
	struct xhci_slot_ctx *in_slot_ctx;
	struct xhci_slot_ctx *out_slot_ctx;

	in_slot_ctx = xhci_get_slot_ctx(ctrl, in_ctx);
	out_slot_ctx = xhci_get_slot_ctx(ctrl, out_ctx);

	in_slot_ctx->dev_info = out_slot_ctx->dev_info;
	in_slot_ctx->dev_info2 = out_slot_ctx->dev_info2;
	in_slot_ctx->tt_info = out_slot_ctx->tt_info;
	in_slot_ctx->dev_state = out_slot_ctx->dev_state;
}

/**
 * Setup an xHCI virtual device for a Set Address command
 *
 * @param udev pointer to the Device Data Structure
 * Return: returns negative value on failure else 0 on success
 */
void xhci_setup_addressable_virt_dev(struct xhci_ctrl *ctrl,
				     struct usb_device *udev, int hop_portnr)
{
	struct xhci_virt_device *virt_dev;
	struct xhci_ep_ctx *ep0_ctx;
	struct xhci_slot_ctx *slot_ctx;
	u32 port_num = 0;
	u64 trb_64 = 0;
	int slot_id = udev->slot_id;
	int speed = udev->speed;

	virt_dev = ctrl->devs[slot_id];

	if (!virt_dev) {
		Kprintf("Invalid virtual device\n");
		return;
	}

	/* Extract the EP0 and Slot Ctrl */
	ep0_ctx = xhci_get_ep_ctx(ctrl, virt_dev->in_ctx, 0);
	slot_ctx = xhci_get_slot_ctx(ctrl, virt_dev->in_ctx);
	Kprintf("xhci_setup_addressable_virt_dev: slot=%ld in_ctx=%lx out_ctx=%lx ep0_ctx=%lx slot_ctx=%lx\n",
		(ULONG)slot_id, (ULONG)virt_dev->in_ctx, (ULONG)virt_dev->out_ctx,
		(ULONG)ep0_ctx, (ULONG)slot_ctx);

	/* Only the control endpoint is valid - one endpoint context */
	u32 dev_info = LE32(slot_ctx->dev_info);
	dev_info &= ~(ROUTE_STRING_MASK | DEV_SPEED | DEV_MTT | LAST_CTX_MASK);
	dev_info |= LAST_CTX(1);
	dev_info |= (udev->route & ROUTE_STRING_MASK);

	switch (speed) {
	case USB_SPEED_SUPER:
	case USB_SPEED_SUPER_PLUS:
		dev_info |= SLOT_SPEED_SS;
		break;
	case USB_SPEED_HIGH:
		dev_info |= SLOT_SPEED_HS;
		break;
	case USB_SPEED_FULL:
		dev_info |= SLOT_SPEED_FS;
		break;
	case USB_SPEED_LOW:
		dev_info |= SLOT_SPEED_LS;
		break;
	default:
		/* Speed was set earlier, this shouldn't happen. */
		Kprintf("Unknown device speed %ld\n", (ULONG)speed);
	}

	/* Low/full-speed devices behind a high-speed hub need TT info */
	bool needs_tt = (speed == USB_SPEED_FULL || speed == USB_SPEED_LOW) &&
		udev->parent &&
		(udev->parent->speed == USB_SPEED_HIGH ||
		 udev->parent->speed == USB_SPEED_SUPER ||
		 udev->parent->speed == USB_SPEED_SUPER_PLUS);
	bool have_tt_info = needs_tt && udev->tt_slot && udev->tt_port;

	slot_ctx->dev_info = LE32(dev_info);

	port_num = hop_portnr;
	Kprintf("xhci_setup_addressable_virt_dev: port_num=%ld speed=%ld route=%ld\n",
		port_num, (ULONG)speed, (ULONG)udev->route);

	u32 dev_info2 = LE32(slot_ctx->dev_info2);
	dev_info2 &= ~((ROOT_HUB_PORT_MASK) << ROOT_HUB_PORT_SHIFT);
	dev_info2 |= ROOT_HUB_PORT(port_num);
	slot_ctx->dev_info2 = LE32(dev_info2);

	u32 tt_info = 0;
	if (have_tt_info)
		tt_info = TT_SLOT(udev->tt_slot) | TT_PORT(udev->tt_port);
	Kprintf("xhci_setup_addressable_virt_dev: needs_tt=%ld have_tt_info=%ld tt_slot=%ld tt_port=%ld tt_info=%08lx\n",
		(int)needs_tt, (int)have_tt_info,
		(int)udev->tt_slot, (int)udev->tt_port, (ULONG)tt_info);
	slot_ctx->tt_info = LE32(tt_info);

	/* Step 4 - ring already allocated */
	/* Step 5 */
	ep0_ctx->ep_info2 = LE32(EP_TYPE(CTRL_EP));
	Kprintf("xhci_setup_addressable_virt_dev: SPEED=%ld\n", (ULONG)speed);

	switch (speed) {
	case USB_SPEED_SUPER:
		ep0_ctx->ep_info2 |= LE32(MAX_PACKET(512));
	Kprintf("xhci_setup_addressable_virt_dev: MPS=512\n");
		break;
	case USB_SPEED_HIGH:
	/* USB core guesses at a 64-byte max packet first for FS devices */
	case USB_SPEED_FULL:
		ep0_ctx->ep_info2 |= LE32(MAX_PACKET(64));
	Kprintf("xhci_setup_addressable_virt_dev: MPS=64\n");
		break;
	case USB_SPEED_LOW:
		ep0_ctx->ep_info2 |= LE32(MAX_PACKET(8));
	Kprintf("xhci_setup_addressable_virt_dev: MPS=8\n");
		break;
	default:
		/* New speed? */
		Kprintf("Unknown device speed %ld\n", (ULONG)speed);
	}

	/* EP 0 can handle "burst" sizes of 1, so Max Burst Size field is 0 */
	ep0_ctx->ep_info2 |= LE32(MAX_BURST(0) | ERROR_COUNT(3));

	trb_64 = virt_dev->eps[0].ring->first_seg->dma;
	ep0_ctx->deq = LE64(trb_64 | virt_dev->eps[0].ring->cycle_state);

	/*
	 * xHCI spec 6.2.3:
	 * software shall set 'Average TRB Length' to 8 for control endpoints.
	 */
	ep0_ctx->tx_info = LE32(EP_AVG_TRB_LENGTH(8));

	/* Steps 7 and 8 were done in xhci_alloc_virt_device() */

	xhci_flush_cache((uintptr_t)ep0_ctx, sizeof(struct xhci_ep_ctx));
	xhci_flush_cache((uintptr_t)slot_ctx, sizeof(struct xhci_slot_ctx));
	Kprintf("xhci_setup_addressable_virt_dev: ep0 deq=%lx tx_info=%08lx dev_info=%08lx dev_info2=%08lx\n",
		(ULONG)LE64(ep0_ctx->deq), (ULONG)LE32(ep0_ctx->tx_info),
		(ULONG)LE32(slot_ctx->dev_info), (ULONG)LE32(slot_ctx->dev_info2));
}
