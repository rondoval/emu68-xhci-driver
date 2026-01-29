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

/**
 * This file gives the xhci stack for usb3.0 looking into
 * xhci specification Rev1.0 (5/21/10).
 * The quirk devices support hasn't been given yet.
 */

#include <exec/memory.h>

#include <debug.h>

#include <xhci/xhci.h>
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-udev.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

dma_addr_t xhci_dma_map(struct xhci_ctrl *ctrl, void *addr, size_t size)
{
	// TODO this is wrong, move mapping up, do it on Poseidon buffers in command handling
	if (!ctrl || !addr || size == 0)
		return (dma_addr_t)(uintptr_t)addr;

	if ((((uintptr_t)addr) & ARCH_DMA_MINALIGN_MASK) == 0)
		return (dma_addr_t)(uintptr_t)addr;

	if (!ctrl->memoryPool)
		return (dma_addr_t)(uintptr_t)addr;

	size_t alloc_len = ALIGN(size, ARCH_DMA_MINALIGN);
	struct xhci_dma_bounce *bounce = AllocVecPooled(ctrl->memoryPool, sizeof(*bounce));
	if (!bounce)
	{
		Kprintf("failed to allocate metadata for %lx len=%ld\n", (ULONG)addr, (LONG)size);
		return (dma_addr_t)(uintptr_t)addr;
	}

	void *aligned = memalign(ctrl->memoryPool, ARCH_DMA_MINALIGN, alloc_len);
	if (!aligned)
	{
		Kprintf("failed to allocate bounce buffer for %lx len=%ld\n", (ULONG)addr, (LONG)size);
		FreeVecPooled(ctrl->memoryPool, bounce);
		return (dma_addr_t)(uintptr_t)addr;
	}

	CopyMem(addr, aligned, size);
	xhci_flush_cache((uintptr_t)aligned, alloc_len);

	bounce->orig = addr;
	bounce->bounce = aligned;
	bounce->size = size;
	bounce->alloc_len = alloc_len;
	bounce->next = ctrl->dma_bounce_list;
	ctrl->dma_bounce_list = bounce;

	return (dma_addr_t)(uintptr_t)aligned;
}

void xhci_dma_unmap(struct xhci_ctrl *ctrl, dma_addr_t addr, size_t size)
{
	if (!ctrl || addr == 0)
		return;

	(void)size;

	struct xhci_dma_bounce **link = &ctrl->dma_bounce_list;
	while (*link)
	{
		struct xhci_dma_bounce *node = *link;
		if ((dma_addr_t)(uintptr_t)node->orig == addr)
		{
			xhci_inval_cache((uintptr_t)node->bounce, node->alloc_len);
			if (node->size)
				CopyMem(node->bounce, node->orig, node->size);
			memalign_free(ctrl->memoryPool, node->bounce);
			*link = node->next;
			FreeVecPooled(ctrl->memoryPool, node);
			return;
		}
		link = &(*link)->next;
	}
}

/**
 * Waits for as per specified amount of time
 * for the "result" to match with "done"
 *
 * @param ptr	pointer to the register to be read
 * @param mask	mask for the value read
 * @param done	value to be campared with result
 * @param usec	time to wait till
 * Return: 0 if handshake is success else < 0 on failure
 */
static int handshake(uint32_t volatile *ptr, uint32_t mask, uint32_t done, int usec)
{
	uint32_t result;
	int ret;

	// TODO fixed value ULONG_MAX
	ret = readx_poll_timeout(readl, ptr, result, (result & mask) == done || result == 0xffffffff, usec);
	if (result == 0xffffffff) /* card removed */
		return -ENODEV;

	return ret;
}

/**
 * Set the run bit and wait for the host to be running.
 *
 * @param hcor	pointer to host controller operation registers
 * Return: status of the Handshake
 */
static int xhci_start(struct xhci_hcor *hcor)
{
	u32 temp;
	int ret;

	Kprintf("Starting the controller\n");
	temp = readl(&hcor->or_usbcmd);
	temp |= (CMD_RUN);
	writel(temp, &hcor->or_usbcmd);

	/*
	 * Wait for the HCHalted Status bit to be 0 to indicate the host is
	 * running.
	 */
	ret = handshake(&hcor->or_usbsts, STS_HALT, 0, XHCI_MAX_HALT_USEC);
	if (ret)
		Kprintf("Host took too long to start, waited %lu microseconds.\n", XHCI_MAX_HALT_USEC);
	return ret;
}

/**
 * Resets the XHCI Controller
 *
 * @param hcor	pointer to host controller operation registers
 * Return: -EBUSY if XHCI Controller is not halted else status of handshake
 */
static int xhci_reset(struct xhci_hcor *hcor)
{
	u32 cmd;
	u32 state;
	int ret;

	/* Halting the Host first */
	Kprintf("// Halt the HC: %lx\n", hcor);
	state = readl(&hcor->or_usbsts) & STS_HALT;
	if (!state)
	{
		cmd = readl(&hcor->or_usbcmd);
		cmd &= ~CMD_RUN;
		writel(cmd, &hcor->or_usbcmd);
	}

	ret = handshake(&hcor->or_usbsts,
					STS_HALT, STS_HALT, XHCI_MAX_HALT_USEC);
	if (ret)
	{
		Kprintf("Host not halted after %lu microseconds.\n", XHCI_MAX_HALT_USEC);
		return -EBUSY;
	}

	Kprintf("// Reset the HC\n");
	cmd = readl(&hcor->or_usbcmd);
	cmd |= CMD_RESET_USB;
	writel(cmd, &hcor->or_usbcmd);

	ret = handshake(&hcor->or_usbcmd, CMD_RESET_USB, 0, XHCI_MAX_RESET_USEC);
	if (ret)
		return ret;

	/*
	 * xHCI cannot write to any doorbells or operational registers other
	 * than status until the "Controller Not Ready" flag is cleared.
	 */
	return handshake(&hcor->or_usbsts, STS_CNR, 0, XHCI_MAX_RESET_USEC);
}

static int xhci_lowlevel_init(struct xhci_ctrl *ctrl)
{
	struct xhci_hccr *hccr;
	struct xhci_hcor *hcor;
	uint32_t val;
	uint32_t val2;
	uint32_t reg;

	hccr = ctrl->hccr;
	hcor = ctrl->hcor;
	/*
	 * Program the Number of Device Slots Enabled field in the CONFIG
	 * register with the max value of slots the HC can handle.
	 */
	val = (readl(&hccr->cr_hcsparams1) & HCS_SLOTS_MASK);
	val2 = readl(&hcor->or_config);
	val |= (val2 & ~HCS_SLOTS_MASK);
	writel(val, &hcor->or_config);

	/* initializing xhci data structures */
	if (xhci_mem_init(ctrl, hccr, hcor) < 0)
		return -ENOMEM;

	ctrl->devices_by_poseidon_address[0] = xhci_udev_alloc(ctrl, 0);
	ctrl->root_hub = xhci_roothub_create(ctrl->devices_by_poseidon_address[0], xhci_udev_io_reply_data);
	if (!ctrl->root_hub)
	{
		return -ENOMEM;
	}

	if (xhci_start(hcor))
	{
		xhci_reset(hcor);
		return -ENODEV;
	}

	/* Zero'ing IRQ control register and IRQ pending register */
	writel(0x0, &ctrl->ir_set->irq_control);
	writel(0x0, &ctrl->ir_set->irq_pending);

	reg = HC_VERSION(readl(&hccr->cr_capbase));
	Kprintf("USB XHCI %lx.%02lx\n", reg >> 8, reg & 0xff);
	ctrl->hci_version = reg;

	return 0;
}

static int xhci_lowlevel_stop(struct xhci_ctrl *ctrl)
{
	u32 temp;

	xhci_reset(ctrl->hcor);

	Kprintf("// Disabling event ring interrupts\n");
	temp = readl(&ctrl->hcor->or_usbsts);
	writel(temp & ~STS_EINT, &ctrl->hcor->or_usbsts);
	temp = readl(&ctrl->ir_set->irq_pending);
	writel(ER_IRQ_DISABLE(temp), &ctrl->ir_set->irq_pending);

	xhci_roothub_destroy(ctrl->root_hub);
	ctrl->root_hub = NULL;

	return 0;
}

int xhci_register(struct xhci_ctrl *ctrl, struct xhci_hccr *hccr,
				  struct xhci_hcor *hcor)
{
	int ret;

	Kprintf("ctrl=%lx, hccr=%lx, hcor=%lx\n", ctrl, hccr, hcor);

	ret = xhci_reset(hcor);
	if (ret)
		goto err;

	ctrl->memoryPool = CreatePool(MEMF_FAST | MEMF_CLEAR, 16384, 8192);
	if (ctrl->memoryPool == NULL)
	{
		ret = -ENOMEM;
		goto err;
	}
	Kprintf("memory pool created: %lx\n", ctrl->memoryPool);

	ctrl->hccr = hccr;
	ctrl->hcor = hcor;
	ret = xhci_lowlevel_init(ctrl);
	if (ret)
		goto err_pool;

	return 0;

err_pool:
	DeletePool(ctrl->memoryPool);
	ctrl->memoryPool = NULL;

err:
	Kprintf("failed, ret=%ld\n", ret);
	return ret;
}

int xhci_deregister(struct xhci_ctrl *ctrl)
{
	xhci_lowlevel_stop(ctrl);
	xhci_cleanup(ctrl);

	if (ctrl->memoryPool)
	{
		DeletePool(ctrl->memoryPool);
		ctrl->memoryPool = NULL;
	}

	return 0;
}
