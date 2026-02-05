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

#include <xhci/usb.h>
#include <xhci/xhci.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-debug.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-ring.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-mem] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci-mem] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#define CACHELINE_SIZE 64

/**
 * Free the scratchpad buffer array and scratchpad buffers
 *
 * @ctrl	host controller data structure
 * Return:	none
 */
static void xhci_scratchpad_free(struct xhci_ctrl *ctrl)
{
	if (!ctrl->scratchpad)
		return;

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
void xhci_free_container_ctx(struct xhci_ctrl *ctrl, struct xhci_container_ctx *ctx)
{
	memalign_free(ctrl->memoryPool, ctx->bytes);
	FreeVecPooled(ctrl->memoryPool, ctx);
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
	memalign_free(ctrl->memoryPool, ctrl->erst.entries);
	memalign_free(ctrl->memoryPool, ctrl->dcbaa);
	_memset(ctrl, 0, sizeof(struct xhci_ctrl));
}

/**
 * Malloc the aligned memory
 *
 * @param size	size of memory to be allocated
 * Return: allocates the memory and returns the aligned pointer
 */
void *xhci_malloc(struct xhci_ctrl *ctrl, unsigned int size)
{
	void *ptr;
	ULONG cacheline_size = max(XHCI_ALIGNMENT, CACHELINE_SIZE);

	ptr = memalign(ctrl->memoryPool, cacheline_size, ALIGN(size, cacheline_size));
	if (!ptr)
	{
		Kprintf("memalign failed for size %lu\n", (ULONG)size);
		return NULL;
	}
	_memset(ptr, '\0', size);

	xhci_flush_cache(ptr, size);

	return ptr;
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
	int num_sp;
	uint32_t page_size;
	void *buf;
	int i;

	num_sp = HCS_MAX_SCRATCHPAD(readl(&hccr->cr_hcsparams2));
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

	ctrl->dcbaa->dev_context_ptrs[0] = LE64((dma_addr_t)scratchpad->sp_array);

	xhci_flush_cache(&ctrl->dcbaa->dev_context_ptrs[0],
					 sizeof(ctrl->dcbaa->dev_context_ptrs[0]));

	page_size = readl(&hcor->or_pagesize) & 0xffff;
	for (i = 0; i < 16; i++)
	{
		if ((0x1 & page_size) != 0)
			break;
		page_size = page_size >> 1;
	}
	if (i == 16)
	{
		Kprintf("Invalid page size\n");
		goto fail_sp3;
	}

	ctrl->page_size = 1 << (i + 12);
	buf = memalign(ctrl->memoryPool, ctrl->page_size, num_sp * ctrl->page_size);
	if (!buf)
		goto fail_sp3;
	_memset(buf, '\0', num_sp * ctrl->page_size);
	xhci_flush_cache(buf, num_sp * ctrl->page_size);

	scratchpad->scratchpad = buf;
	for (i = 0; i < num_sp; i++)
	{
		scratchpad->sp_array[i] = LE64((dma_addr_t)buf);
		buf += ctrl->page_size;
	}

	xhci_flush_cache(scratchpad->sp_array,
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
struct xhci_container_ctx *xhci_alloc_container_ctx(struct xhci_ctrl *ctrl, int type)
{
	struct xhci_container_ctx *ctx;

	ctx = AllocVecPooled(ctrl->memoryPool, sizeof(struct xhci_container_ctx));
	if (!ctx)
	{
		Kprintf("Failed to allocate container context\n");
		return NULL;
	}

	if ((type != XHCI_CTX_TYPE_DEVICE) && (type != XHCI_CTX_TYPE_INPUT))
	{
		Kprintf("Invalid context type\n");
		FreeVecPooled(ctrl->memoryPool, ctx);
		return NULL;
	}

	ctx->type = type;
	ctx->size = (USB_MAX_ENDPOINT_CONTEXTS + 1) *
				CTX_SIZE(readl(&ctrl->hccr->cr_hccparams));
	if (type == XHCI_CTX_TYPE_INPUT)
		ctx->size += CTX_SIZE(readl(&ctrl->hccr->cr_hccparams));

	ctx->bytes = xhci_malloc(ctrl, ctx->size);

	return ctx;
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
	uint32_t val;

	/* DCBAA initialization */
	ctrl->dcbaa = xhci_malloc(ctrl, sizeof(struct xhci_device_context_array));
	if (ctrl->dcbaa == NULL)
	{
		Kprintf("unable to allocate DCBA\n");
		return -ENOMEM;
	}

	/* Set the pointer in DCBAA register */
	xhci_writeq(&hcor->or_dcbaap, (dma_addr_t)ctrl->dcbaa);

	/* Command ring control pointer register initialization */
	ctrl->cmd_ring = xhci_ring_alloc(ctrl, 1, TRUE, FALSE, 0);

	/* Set the address in the Command Ring Control register */
	u64 trb_64 = xhci_ring_get_new_dequeue_ptr(ctrl->cmd_ring);
	u64 val_64 = xhci_readq(&hcor->or_crcr);
	val_64 = (val_64 & (u64)(CMD_RING_ADDR_MASK | 1)) |
			 (trb_64 & (u64) ~(CMD_RING_ADDR_MASK));
	xhci_writeq(&hcor->or_crcr, val_64);

	/* write the address of db register */
	val = readl(&hccr->cr_dboff);
	val &= DBOFF_MASK;
	ctrl->dba = (struct xhci_doorbell_array *)((char *)hccr + val);

	/* write the address of runtime register */
	val = readl(&hccr->cr_rtsoff);
	val &= RTSOFF_MASK;
	ctrl->run_regs = (struct xhci_run_regs *)((char *)hccr + val);

	/* writting the address of ir_set structure */
	ctrl->ir_set = &ctrl->run_regs->ir_set[0];

	ctrl->erst.entries = xhci_malloc(ctrl, sizeof(struct xhci_erst_entry) *
											   ERST_NUM_SEGS);
	/* Event ring does not maintain link TRB */
	ctrl->event_ring = xhci_ring_alloc(ctrl, ERST_NUM_SEGS, FALSE, TRUE, 0);

	xhci_ring_setup_erst(ctrl->event_ring, &ctrl->erst, ctrl->ir_set);

	/* set up the scratchpad buffer array and scratchpad buffers */
	xhci_scratchpad_alloc(ctrl);

	/*
	 * Just Zero'ing this register completely,
	 * or some spurious Device Notification Events
	 * might screw things here.
	 */
	writel(0x0, &hcor->or_dnctrl);

	return 0;
}

/**
 * Give the input control context for the passed container context
 *
 * @param ctx	pointer to the context
 * Return: pointer to the Input control context data
 */
struct xhci_input_control_ctx *xhci_get_input_control_ctx(struct xhci_container_ctx *ctx)
{
	if (ctx->type != XHCI_CTX_TYPE_INPUT)
	{
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

	return (struct xhci_slot_ctx *)(ctx->bytes + CTX_SIZE(readl(&ctrl->hccr->cr_hccparams)));
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

	return (struct xhci_ep_ctx *)(ctx->bytes +
								  (ep_index * CTX_SIZE(readl(&ctrl->hccr->cr_hccparams))));
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

static unsigned int route_depth(unsigned int route)
{
	unsigned int depth = 0;
	while (route && depth < 5)
	{
		route >>= 4;
		depth++;
	}
	return depth;
}

static unsigned int build_route_string(struct usb_device *parent, unsigned int port)
{
	if (!parent)
		return 0;

	unsigned int depth = route_depth(parent->route);
	if (depth >= 5)
		return parent->route;

	unsigned int nibble = port & 0xF;
	if (nibble == 0)
		return parent->route;

	return parent->route | (nibble << (depth * 4));
}

/**
 * Setup an xHCI virtual device for a Set Address command
 *
 * @param udev pointer to the Device Data Structure
 * Return: returns negative value on failure else 0 on success
 */
void xhci_setup_addressable_virt_dev(struct xhci_ctrl *ctrl, struct usb_device *udev)
{
	udev->parent = ctrl->pending_parent;
	udev->parent_port = ctrl->pending_parent_port;
	udev->route = build_route_string(udev->parent, udev->parent_port);

	/* Extract the EP0 and Slot Ctrl */
	struct xhci_ep_ctx *ep0_ctx = xhci_get_ep_ctx(ctrl, udev->in_ctx, 0);
	struct xhci_slot_ctx *slot_ctx = xhci_get_slot_ctx(ctrl, udev->in_ctx);
	KprintfH("slot=%ld in_ctx=%lx out_ctx=%lx ep0_ctx=%lx slot_ctx=%lx\n",
			 (ULONG)udev->slot_id, (ULONG)udev->in_ctx, (ULONG)udev->out_ctx,
			 (ULONG)ep0_ctx, (ULONG)slot_ctx);

	/* Only the control endpoint is valid - one endpoint context */
	u32 dev_info = LE32(slot_ctx->dev_info);
	dev_info &= ~(ROUTE_STRING_MASK | DEV_SPEED | DEV_MTT | LAST_CTX_MASK);
	dev_info |= (udev->route & ROUTE_STRING_MASK);
	dev_info |= LAST_CTX(1);

	switch (udev->speed)
	{
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
		Kprintf("Unknown device speed %ld\n", (ULONG)udev->speed);
	}

	// Find root hub port number
	u32 root_port = ctrl->pending_parent_port;
	if (udev->parent)
	{
		struct usb_device *ancestor = udev->parent;

		while (ancestor)
		{
			if (ancestor->parent_port)
				root_port = ancestor->parent_port;

			if (!ancestor->parent)
				break;

			ancestor = ancestor->parent;
		}
	}

	/* Low/full-speed devices behind a high-speed hub need TT info */
	BOOL needs_tt = (udev->speed == USB_SPEED_FULL || udev->speed == USB_SPEED_LOW) &&
					udev->parent &&
					(udev->parent->speed == USB_SPEED_HIGH ||
					 udev->parent->speed == USB_SPEED_SUPER ||
					 udev->parent->speed == USB_SPEED_SUPER_PLUS);

	slot_ctx->dev_info = LE32(dev_info);

	KprintfH("xhci_setup_addressable_virt_dev: parent_addr=%ld port_num=%ld root_port_num=%ld speed=%ld route=%lx\n",
			 (ULONG)udev->parent->poseidon_address, udev->parent_port, root_port, (ULONG)udev->speed, (ULONG)udev->route);

	u32 dev_info2 = LE32(slot_ctx->dev_info2);
	dev_info2 &= ~((ROOT_HUB_PORT_MASK) << ROOT_HUB_PORT_SHIFT);
	dev_info2 |= ROOT_HUB_PORT(root_port);
	slot_ctx->dev_info2 = LE32(dev_info2);

	u32 tt_info = 0;
	if (needs_tt)
	{
		tt_info = TT_SLOT(udev->parent->slot_id) |
				  TT_PORT(udev->parent_port);
	}

	KprintfH("xhci_setup_addressable_virt_dev: needs_tt=%ld tt_slot=%ld tt_port=%ld tt_info=%08lx\n",
			 (int)needs_tt,
			 (int)udev->parent->slot_id, (int)udev->parent_port,
			 (ULONG)tt_info);
	slot_ctx->tt_info = LE32(tt_info);

	/* Step 4 - ring already allocated */
	/* Step 5 */
	ep0_ctx->ep_info2 = LE32(EP_TYPE(CTRL_EP));
	KprintfH("xhci_setup_addressable_virt_dev: SPEED=%ld\n", (ULONG)udev->speed);

	switch (udev->speed)
	{
	case USB_SPEED_SUPER:
		ep0_ctx->ep_info2 |= LE32(MAX_PACKET(512));
		KprintfH("xhci_setup_addressable_virt_dev: MPS=512\n");
		break;
	case USB_SPEED_HIGH:
	/* USB core guesses at a 64-byte max packet first for FS devices */
	case USB_SPEED_FULL:
		ep0_ctx->ep_info2 |= LE32(MAX_PACKET(64));
		KprintfH("xhci_setup_addressable_virt_dev: MPS=64\n");
		break;
	case USB_SPEED_LOW:
		ep0_ctx->ep_info2 |= LE32(MAX_PACKET(8));
		KprintfH("xhci_setup_addressable_virt_dev: MPS=8\n");
		break;
	default:
		/* New speed? */
		Kprintf("Unknown device speed %ld\n", (ULONG)udev->speed);
	}

	/* EP 0 can handle "burst" sizes of 1, so Max Burst Size field is 0 */
	ep0_ctx->ep_info2 |= LE32(MAX_BURST(0) | ERROR_COUNT(3));

	struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, 0);
	struct xhci_ring *ring = xhci_ep_get_ring(ep_ctx);
	ep0_ctx->deq = LE64(xhci_ring_get_new_dequeue_ptr(ring));

	/*
	 * xHCI spec 6.2.3:
	 * software shall set 'Average TRB Length' to 8 for control endpoints.
	 */
	ep0_ctx->tx_info = LE32(EP_AVG_TRB_LENGTH(8));

	/* Steps 7 and 8 were done in xhci_alloc_virt_device() */

	xhci_flush_cache(ep0_ctx, sizeof(struct xhci_ep_ctx));
	xhci_flush_cache(slot_ctx, sizeof(struct xhci_slot_ctx));
	KprintfH("xhci_setup_addressable_virt_dev: ep0 deq=%lx tx_info=%08lx dev_info=%08lx dev_info2=%08lx\n",
			 (ULONG)LE64(ep0_ctx->deq), (ULONG)LE32(ep0_ctx->tx_info),
			 (ULONG)LE32(slot_ctx->dev_info), (ULONG)LE32(slot_ctx->dev_info2));
}

void xhci_update_hub_tt(struct usb_device *udev)
{
	if (!udev || !udev->controller)
		return;

	if (udev->slot_id == 0)
		return;

	udev->route = build_route_string(udev->parent, udev->parent_port);

	struct xhci_ctrl *ctrl = udev->controller;

	xhci_inval_cache(udev->out_ctx->bytes, udev->out_ctx->size);

	/* Start from the controller's current view of the slot context */
	xhci_slot_copy(ctrl, udev->in_ctx, udev->out_ctx);

	struct xhci_slot_ctx *slot_ctx = xhci_get_slot_ctx(ctrl, udev->in_ctx);
	struct xhci_input_control_ctx *ctrl_ctx = xhci_get_input_control_ctx(udev->in_ctx);
	if (!slot_ctx || !ctrl_ctx)
		return;

	u32 dev_info = LE32(slot_ctx->dev_info);
	u32 tt_info = LE32(slot_ctx->tt_info);

	dev_info |= DEV_HUB;

	tt_info &= ~(TT_SLOT(0xff) | TT_PORT(0xff) | TT_THINK_TIME(0x3));
	tt_info = TT_SLOT(udev->parent->slot_id) |
			  TT_PORT(udev->parent_port) |
			  TT_THINK_TIME(udev->tt_think_time & 0x3);

	slot_ctx->dev_info = LE32(dev_info);
	slot_ctx->tt_info = LE32(tt_info);

	ctrl_ctx->add_flags = LE32(SLOT_FLAG);
	ctrl_ctx->drop_flags = 0;

	xhci_flush_cache(slot_ctx, sizeof(*slot_ctx));
	xhci_flush_cache(ctrl_ctx, sizeof(*ctrl_ctx));
	/* xhci_configure_endpoints will flush the full input context */

	KprintfH("xhci_update_hub_tt: addr=%ld tt_code=%ld\n",
			 (ULONG)udev->poseidon_address, (LONG)udev->tt_think_time);

	xhci_configure_endpoints(udev, TRUE, NULL);
}
