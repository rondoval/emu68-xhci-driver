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
#include <xhci/xhci-ring.h>
#include <devices/usbhardware.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#define CACHELINE_SIZE 64

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

	int num_sp = HCS_MAX_SCRATCHPAD(readl(&hccr->cr_hcsparams2));
	if (!num_sp)
		return 0;

	struct xhci_scratchpad *scratchpad = AllocVecPooled(ctrl->memoryPool, sizeof(struct xhci_scratchpad));
	if (!scratchpad)
		goto fail_sp;
	ctrl->scratchpad = scratchpad;

	scratchpad->sp_array = xhci_malloc(ctrl, num_sp * sizeof(u64));
	if (!scratchpad->sp_array)
		goto fail_sp2;

	ctrl->dcbaa->dev_context_ptrs[0] = LE64((dma_addr_t)scratchpad->sp_array);
	xhci_flush_cache(&ctrl->dcbaa->dev_context_ptrs[0], sizeof(ctrl->dcbaa->dev_context_ptrs[0]));

	u32 page_size = readl(&hcor->or_pagesize) & 0xffff;
	int i;
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

	page_size = 1 << (i + 12);
	void *buf = memalign(ctrl->memoryPool, page_size, num_sp * page_size);
	if (!buf)
		goto fail_sp3;
	_memset(buf, '\0', num_sp * page_size);
	xhci_flush_cache(buf, num_sp * page_size);

	scratchpad->scratchpad = buf;
	for (i = 0; i < num_sp; i++)
	{
		scratchpad->sp_array[i] = LE64((dma_addr_t)buf);
		buf += page_size;
	}

	xhci_flush_cache(scratchpad->sp_array, sizeof(u64) * num_sp);
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
 * Allocates the necessary data structures
 * for XHCI host controller
 *
 * @param ctrl	Host controller data structure
 * @param hccr	pointer to HOST Controller Control Registers
 * @param hcor	pointer to HOST Controller Operational Registers
 * Return: 0 if successful else -1 on failure
 */
static int xhci_mem_init(struct xhci_ctrl *ctrl, struct xhci_hccr *hccr,
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
	ctrl->cmd_ring = xhci_ring_alloc(ctrl, 1, TRUE, FALSE, 0, 0);

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
	ctrl->event_ring = xhci_ring_alloc(ctrl, ERST_NUM_SEGS, FALSE, TRUE, 0, 0);

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
 * frees all the memory allocated
 *
 * @param ptr	pointer to "xhci_ctrl" to be cleaned up
 * Return: none
 */
static void xhci_cleanup(struct xhci_ctrl *ctrl)
{
	xhci_ring_free(ctrl, ctrl->event_ring);
	xhci_ring_free(ctrl, ctrl->cmd_ring);
	xhci_scratchpad_free(ctrl);
	memalign_free(ctrl->memoryPool, ctrl->erst.entries);
	memalign_free(ctrl->memoryPool, ctrl->dcbaa);
	_memset(ctrl, 0, sizeof(struct xhci_ctrl));
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

	// TODO fixed value ULONG_MAX
	int ret = readx_poll_timeout(readl, ptr, result, (result & mask) == done || result == 0xffffffff, usec);
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
	Kprintf("Starting the controller\n");
	u32 temp = readl(&hcor->or_usbcmd);
	temp |= (CMD_RUN);
	writel(temp, &hcor->or_usbcmd);

	/*
	 * Wait for the HCHalted Status bit to be 0 to indicate the host is
	 * running.
	 */
	int ret = handshake(&hcor->or_usbsts, STS_HALT, 0, XHCI_MAX_HALT_USEC);
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

	/* Halting the Host first */
	Kprintf("// Halt the HC: %lx\n", hcor);
	u32 state = readl(&hcor->or_usbsts) & STS_HALT;
	if (!state)
	{
		cmd = readl(&hcor->or_usbcmd);
		cmd &= ~CMD_RUN;
		writel(cmd, &hcor->or_usbcmd);
	}

	int ret = handshake(&hcor->or_usbsts, STS_HALT, STS_HALT, XHCI_MAX_HALT_USEC);
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
	struct xhci_hccr *hccr = ctrl->hccr;
	struct xhci_hcor *hcor = ctrl->hcor;
	/*
	 * Program the Number of Device Slots Enabled field in the CONFIG
	 * register with the max value of slots the HC can handle.
	 */
	u32 val = (readl(&hccr->cr_hcsparams1) & HCS_SLOTS_MASK);
	u32 val2 = readl(&hcor->or_config);
	val |= (val2 & ~HCS_SLOTS_MASK);
	writel(val, &hcor->or_config);

	/* initializing xhci data structures */
	if (xhci_mem_init(ctrl, hccr, hcor) < 0)
		return -ENOMEM;

	ctrl->devices_by_poseidon_address[0] = xhci_udev_alloc(ctrl, 0);
	ctrl->root_hub = xhci_roothub_create(ctrl->devices_by_poseidon_address[0], xhci_udev_io_reply_data);
	if (!ctrl->root_hub)
		return -ENOMEM;

	if (xhci_start(hcor))
	{
		xhci_reset(hcor);
		return -ENODEV;
	}

	/* Zero'ing IRQ control register and IRQ pending register */
	writel(0x0, &ctrl->ir_set->irq_control);
	writel(0x0, &ctrl->ir_set->irq_pending);

	u32 reg = HC_VERSION(readl(&hccr->cr_capbase));
	Kprintf("USB XHCI %lx.%02lx\n", reg >> 8, reg & 0xff);
	ctrl->hci_version = reg;

	return 0;
}

static int xhci_lowlevel_stop(struct xhci_ctrl *ctrl)
{
	xhci_reset(ctrl->hcor);

	Kprintf("// Disabling event ring interrupts\n");
	u32 temp = readl(&ctrl->hcor->or_usbsts);
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
	Kprintf("ctrl=%lx, hccr=%lx, hcor=%lx\n", ctrl, hccr, hcor);

	int ret = xhci_reset(hcor);
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
