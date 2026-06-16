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

/**
 * This file gives the xhci stack for usb3.0 looking into
 * xhci specification Rev1.0 (5/21/10).
 * The quirk devices support hasn't been given yet.
 */

#include <exec/memory.h>

#include <debug.h>
#include <errors.h>
#include <memory.h>
#include <bits.h>
#include <timing.h>
#include <minlist.h>

#include <config.h>
#include <xhci/xhci.h>
#include <xhci/xhci-td.h>
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci-ring.h>
#include <devices/hcd_api.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#define CACHELINE_SIZE 64
#define XHCI_EXT_CAPS_SEARCH_DONE ((u32)~0U)

static u32 xhci_get_page_size(struct xhci_ctrl *ctrl)
{
	if (!ctrl || !ctrl->hcor)
		return 0;

	u32 page_bits = mmio_read32(&ctrl->hcor->or_pagesize) & 0xffffU;
	for (u32 shift = 0; shift < 16; ++shift)
	{
		if ((page_bits & 0x1U) != 0)
			return (u32)1U << (shift + 12U);
		page_bits >>= 1;
	}

	return 0;
}

/**
 * Set up the scratchpad buffer array and scratchpad buffers
 *
 * @ctrl	host controller data structure
 * Return:	-ENOMEM if buffer allocation fails, 0 on success
 */
static s32 xhci_scratchpad_alloc(struct xhci_ctrl *ctrl)
{
	struct xhci_hccr *hccr = ctrl->hccr;

	u32 num_sp = HCS_MAX_SCRATCHPAD(mmio_read32(&hccr->cr_hcsparams2));
	if (!num_sp)
		return 0;

	struct xhci_scratchpad *scratchpad = pool_zalloc(ctrl->metaPool, sizeof(struct xhci_scratchpad));
	if (!scratchpad)
		goto fail_sp;
	ctrl->scratchpad = scratchpad;

	scratchpad->sp_array = xhci_malloc_page_bounded(ctrl, num_sp * sizeof(u64), XHCI_ALIGNMENT);
	if (!scratchpad->sp_array)
		goto fail_sp2;

	ctrl->dcbaa->dev_context_ptrs[0] = le64(scratchpad->sp_array);
	xhci_flush_cache(&ctrl->dcbaa->dev_context_ptrs[0], sizeof(ctrl->dcbaa->dev_context_ptrs[0]), 0);

	const u32 page_size = ctrl->page_size;
	void *buf = dma_zalloc(ctrl->dmaPool, page_size, num_sp * page_size);
	if (!buf)
		goto fail_sp3;
	xhci_flush_cache(buf, num_sp * page_size, 0);

	scratchpad->scratchpad = buf;
	for (u32 i = 0; i < num_sp; i++)
	{
		scratchpad->sp_array[i] = le64(scratchpad->scratchpad + (i * page_size));
		buf += page_size;
	}

	xhci_flush_cache(scratchpad->sp_array, sizeof(u64) * num_sp, 0);
	return 0;

fail_sp3:
	dma_free(ctrl->dmaPool, scratchpad->sp_array);

fail_sp2:
	pool_free(ctrl->metaPool, scratchpad);
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

	dma_free(ctrl->dmaPool, ctrl->scratchpad->scratchpad);
	dma_free(ctrl->dmaPool, ctrl->scratchpad->sp_array);
	pool_free(ctrl->metaPool, ctrl->scratchpad);
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
static s32 xhci_mem_init(struct xhci_ctrl *ctrl, struct xhci_hccr *hccr,
						 struct xhci_hcor *hcor)
{
	uint32_t val;

	/* DCBAA initialization */
	ctrl->dcbaa = xhci_malloc_page_bounded(ctrl, sizeof(struct xhci_device_context_array), XHCI_ALIGNMENT);
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
	val = mmio_read32(&hccr->cr_dboff);
	val &= DBOFF_MASK;
	ctrl->dba = (struct xhci_doorbell_array *)((char *)hccr + val);

	/* write the address of runtime register */
	val = mmio_read32(&hccr->cr_rtsoff);
	val &= RTSOFF_MASK;
	ctrl->run_regs = (struct xhci_run_regs *)((char *)hccr + val);

	/* writting the address of ir_set structure */
	ctrl->ir_set = &ctrl->run_regs->ir_set[0];
	const u32 erst_max = HCS_ERST_MAX(mmio_read32(&hccr->cr_hcsparams2));

	const u32 event_ring_segs = (XHCI_INITIAL_SEGS_PER_EVENT_RING <= erst_max)
									? XHCI_INITIAL_SEGS_PER_EVENT_RING
									: erst_max;
	if (event_ring_segs != XHCI_INITIAL_SEGS_PER_EVENT_RING)
	{
		Kprintf("clamping ERST entries from %lu to controller max %lu\n",
				(ULONG)XHCI_INITIAL_SEGS_PER_EVENT_RING,
				(ULONG)event_ring_segs);
	}
	ctrl->erst.erst_size = erst_max;
	ctrl->erst.num_entries = event_ring_segs;

	const u32 erst_bytes = sizeof(struct xhci_erst_entry) * event_ring_segs;
	ctrl->erst.entries = dma_zalloc(ctrl->dmaPool, XHCI_ALIGNMENT, erst_bytes);
	if (!ctrl->erst.entries)
	{
		Kprintf("unable to allocate ERST entries\n");
		return -ENOMEM;
	}
	xhci_flush_cache(ctrl->erst.entries, erst_bytes, 0);

	/* Event ring does not maintain link TRB */
	ctrl->event_ring = xhci_ring_alloc(ctrl, event_ring_segs, FALSE, TRUE, 0, 0);
	if (!ctrl->event_ring)
	{
		Kprintf("unable to allocate event ring\n");
		return -ENOMEM;
	}

	xhci_ring_setup_erst(ctrl->event_ring, &ctrl->erst, ctrl->ir_set);

	/* set up the scratchpad buffer array and scratchpad buffers */
	if (xhci_scratchpad_alloc(ctrl) < 0)
		return -ENOMEM;

	/*
	 * Just Zero'ing this register completely,
	 * or some spurious Device Notification Events
	 * might screw things here.
	 */
	mmio_write32(0x0, &hcor->or_dnctrl);

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
	dma_free(ctrl->dmaPool, ctrl->erst.entries);
	dma_free(ctrl->dmaPool, ctrl->dcbaa);
	mem_zero(ctrl, sizeof(struct xhci_ctrl));
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
static s32 handshake(volatile u32 *ptr, u32 mask, u32 done, u32 usec)
{
	u32 result;
	u32 deadline = get_time() + usec;

	for (;;)
	{
		result = mmio_read32(ptr);
		if ((result & mask) == done)
			return 0;
		if (result == 0xffffffff)
			return -ENODEV;
		if (usec && time_deadline_passed(get_time(), deadline))
			break;
	}

	return -ETIMEDOUT;
}

/**
 * Set the run bit and wait for the host to be running.
 *
 * @param hcor	pointer to host controller operation registers
 * Return: status of the Handshake
 */
static s32 xhci_start(struct xhci_hcor *hcor)
{
	Kprintf("Starting the controller\n");
	u32 temp = mmio_read32(&hcor->or_usbcmd);
	temp |= (CMD_RUN);
	mmio_write32(temp, &hcor->or_usbcmd);

	/*
	 * Wait for the HCHalted Status bit to be 0 to indicate the host is
	 * running.
	 */
	s32 ret = handshake(&hcor->or_usbsts, STS_HALT, 0, XHCI_MAX_HALT_USEC);
	if (ret)
		Kprintf("Host took too long to start, waited %lu microseconds.\n", XHCI_MAX_HALT_USEC);
	return ret;
}

/**
 * Halt the XHCI Controller: clear Run/Stop and wait for HCHalted.  A halted
 * HC processes no TRBs and writes no events or MSIs — all DMA stops.
 *
 * @param hcor	pointer to host controller operation registers
 * Return: 0 when halted, < 0 on handshake timeout
 */
static s32 xhci_halt(struct xhci_hcor *hcor)
{
	KprintfH("// Halt the HC: %lx\n", hcor);
	u32 state = mmio_read32(&hcor->or_usbsts) & STS_HALT;
	if (!state)
	{
		u32 cmd = mmio_read32(&hcor->or_usbcmd);
		cmd &= ~CMD_RUN;
		mmio_write32(cmd, &hcor->or_usbcmd);
	}

	return handshake(&hcor->or_usbsts, STS_HALT, STS_HALT, XHCI_MAX_HALT_USEC);
}

/*
 * xhci_reset_quiesce - stop all controller DMA without touching driver state.
 *
 * The pre-reset quiesce (reset_guard): the machine is about to reset and the
 * HC must stop writing into RAM the next OS session reuses.
 */
void xhci_reset_quiesce(struct xhci_ctrl *ctrl)
{
	if (xhci_halt(ctrl->hcor) != 0)
		Kprintf("xhci: HC did not halt for reset quiesce\n");
}

/**
 * Resets the XHCI Controller
 *
 * @param hcor	pointer to host controller operation registers
 * Return: -EBUSY if XHCI Controller is not halted else status of handshake
 */
static s32 xhci_reset(struct xhci_hcor *hcor)
{
	u32 cmd;

	s32 ret = xhci_halt(hcor);
	if (ret)
	{
		Kprintf("Host not halted after %lu microseconds.\n", XHCI_MAX_HALT_USEC);
		return -EBUSY;
	}

	KprintfH("// Reset the HC\n");
	cmd = mmio_read32(&hcor->or_usbcmd);
	cmd |= CMD_RESET_USB;
	mmio_write32(cmd, &hcor->or_usbcmd);

	ret = handshake(&hcor->or_usbcmd, CMD_RESET_USB, 0, XHCI_MAX_RESET_USEC);
	if (ret)
		return ret;

	/*
	 * xHCI cannot write to any doorbells or operational registers other
	 * than status until the "Controller Not Ready" flag is cleared.
	 */
	return handshake(&hcor->or_usbsts, STS_CNR, 0, XHCI_MAX_RESET_USEC);
}

/**
 * find_next_capability - Find the next XHCI extended capability
 * @ctrl: Pointer to the XHCI controller structure
 * @cap_id: The capability ID to search for
 * @init_offset: Pointer to the offset of the next capability to check;
 *               on input, 0 starts from the beginning, non-zero continues
 *               from a previous search; on output, contains the offset of
 *               the next capability in the chain, or
 *               XHCI_EXT_CAPS_SEARCH_DONE when search is exhausted
 *
 * Description:
 * Searches through the extended capability list in the XHCI host controller
 * to find the next capability matching the specified ID. The function supports
 * iterative searching through the capability chain by maintaining the offset
 * position between calls.
 *
 * Return:
 * On success, returns a pointer to the capability structure at the found offset.
 * If the capability is not found or next_offset is NULL, returns NULL.
 * The next_offset output parameter is updated with the offset of the next
 * capability in the chain for continued searching.
 */
u32 *xhci_find_next_capability(struct xhci_ctrl *ctrl, u32 cap_id, u32 *init_offset)
{
	if (init_offset == NULL || *init_offset == XHCI_EXT_CAPS_SEARCH_DONE)
		return NULL;

	struct xhci_hccr *hccr = ctrl->hccr;
	u32 hccParams = mmio_read32(&hccr->cr_hccparams1);

	u32 current_offset = (*init_offset != 0) ? *init_offset : HCC_EXT_CAPS(hccParams) << 2;
	while (current_offset != XHCI_EXT_CAPS_SEARCH_DONE)
	{
		u32 *current = (u32 *)((u8 *)hccr + current_offset);
		u32 ext_cap = mmio_read32(current);

		u32 next_offset = current_offset + (XHCI_EXT_CAPS_NEXT(ext_cap) << 2);
		if (next_offset == current_offset)
			next_offset = XHCI_EXT_CAPS_SEARCH_DONE; /* That was the last capability */

		if (XHCI_EXT_CAPS_ID(ext_cap) == cap_id)
		{
			*init_offset = next_offset;
			return current;
		}

		current_offset = next_offset;
	}

	*init_offset = XHCI_EXT_CAPS_SEARCH_DONE;
	return NULL;
}

struct xhci_protocol_caps xhci_get_protocol_caps(u32 *base_address)
{
	struct xhci_protocol_caps caps = {0};
	u32 cap00 = mmio_read32(base_address + 0);
	u32 cap08 = mmio_read32(base_address + 2);
	u32 cap0c = mmio_read32(base_address + 3);

	caps.minor_revision = XHCI_PROTOCOL_CAP_MINOR_REV(cap00);
	caps.major_revision = XHCI_PROTOCOL_CAP_MAJOR_REV(cap00);
	caps.port_offset = XHCI_PROTOCOL_CAP_PORT_OFFSET(cap08);
	caps.port_count = XHCI_PROTOCOL_CAP_PORT_COUNT(cap08);
	if (caps.major_revision >= 0x3)
	{
		caps.usb3_lsecc = XHCI_PROTOCOL_CAP_USB3_LSECC(cap08);
		caps.max_hub_depth = XHCI_PROTOCOL_CAP_USB3_MHD(cap08);
	}
	else
	{
		caps.usb2_hs_only = XHCI_PROTOCOL_CAP_USB2_HSO(cap08);
		caps.usb2_integrated_hub = XHCI_PROTOCOL_CAP_USB2_IHI(cap08);
		caps.usb2_hw_lpm = XHCI_PROTOCOL_CAP_USB2_HLC(cap08);
		caps.usb2_besl_lpm = XHCI_PROTOCOL_CAP_USB2_BLC(cap08);
		caps.max_hub_depth = XHCI_PROTOCOL_CAP_USB2_MHD(cap08);
	}
	caps.protocol_speed_id_count = XHCI_PROTOCOL_CAP_SPEED_ID_COUNT(cap08);
	caps.protocol_slot_type = (u8)(caps.protocol_slot_type | XHCI_PROTOCOL_CAP_SLOT_TYPE(cap0c));

	return caps;
}

static void xhci_dump_caps(struct xhci_ctrl *ctrl)
{
	struct xhci_hccr *hccr = ctrl->hccr;
	u32 reg = mmio_read32(&hccr->cr_hccparams1);
	if (HCC_64BIT_ADDR(reg))
		Kprintf("Host controller supports 64-bit addressing\n");
	if (HCC_BANDWIDTH_NEG(reg))
		Kprintf("Host controller supports bandwidth negotiation\n");
	if (HCC_64BYTE_CONTEXT(reg))
		Kprintf("Host controller supports 64-byte context structures\n");
	if (HCC_LIGHT_RESET(reg))
		Kprintf("Host controller supports Light HC Reset Capability\n");
	if (HCC_LTC(reg))
		Kprintf("Host controller supports latency tolerance messaging\n");
	if (HCC_NSS(reg))
		Kprintf("Host controller does not support secondary Stream ID\n");
	if (HCC_PAE(reg))
		Kprintf("Host controller supports Parse All Event Data\n");
	if (HCC_SPC(reg))
		Kprintf("Host controller supports Stopped - Short Packet Capability\n");
	if (HCC_SEC(reg))
		Kprintf("Host controller supports Stopped EDTLA Capability\n");
	if (HCC_CFC(reg))
		Kprintf("Host controller supports Contiguous Frame ID Capability\n");

	reg = mmio_read32(&hccr->cr_hccparams2);
	if (HCC_U3C(reg))
		Kprintf("Host controller supports U3 Entry Capability\n");
	if (HCC_CMC(reg))
		Kprintf("Host controller supports Configure Endpoint Command Max Exit Latency Too Large Capability\n");
	if (HCC_FSC(reg))
		Kprintf("Host controller supports Force Save Context Capability\n");
	if (HCC_CTC(reg))
		Kprintf("Host controller supports Compliance Transition Capability\n");
	if (HCC_LEC(reg))
		Kprintf("Host controller supports Large ESIT Payload Capability\n");
	if (HCC_CIC(reg))
		Kprintf("Host controller supports Configuration Information Capability\n");
	if (HCC_ETC(reg))
		Kprintf("Host controller supports Extended TBC Capability\n");
	if (HCC_ETC_TSC(reg))
		Kprintf("Host controller supports Extended TBC TRB Status Capability\n");
	if (HCC_GSC(reg))
		Kprintf("Host controller supports Get/Set Extended Property Capability\n");
	if (HCC_VTC(reg))
		Kprintf("Host controller supports Virtualization Based Trusted I/O Capability\n");
}

static s32 xhci_lowlevel_init(struct xhci_ctrl *ctrl)
{
	struct xhci_hccr *hccr = ctrl->hccr;
	struct xhci_hcor *hcor = ctrl->hcor;

	/* Decode the controller page size once; every page-bounded allocation
	 * below relies on it. */
	ctrl->page_size = xhci_get_page_size(ctrl);
	if (ctrl->page_size == 0)
	{
		Kprintf("invalid controller page size\n");
		return -ENODEV;
	}

	/*
	 * Program the Number of Device Slots Enabled field in the CONFIG
	 * register with the max value of slots the HC can handle.
	 */
	u32 val = (mmio_read32(&hccr->cr_hcsparams1) & HCS_SLOTS_MASK);
	u32 val2 = mmio_read32(&hcor->or_config);
	val |= (val2 & ~HCS_SLOTS_MASK);
	mmio_write32(val, &hcor->or_config);

	/* initializing xhci data structures */
	if (xhci_mem_init(ctrl, hccr, hcor) < 0)
		return -ENOMEM;

	ctrl->devices_by_virtual_address[0] = xhci_udev_alloc(ctrl, 0);
	ctrl->root_hub = xhci_roothub_create(ctrl->devices_by_virtual_address[0], xhci_udev_io_reply_data);
	if (!ctrl->root_hub)
		return -ENOMEM;

	if (xhci_start(hcor))
	{
		xhci_reset(hcor);
		return -ENODEV;
	}

	/* Zero'ing IRQ control register and IRQ pending register */
	mmio_write32(IRQ_INTERVAL & ER_IRQ_INTERVAL_MASK, &ctrl->ir_set->irq_control);
	mmio_write32(0x0, &ctrl->ir_set->irq_pending);

	u32 reg = HC_VERSION(mmio_read32(&hccr->cr_capbase));
	Kprintf("USB XHCI %lx.%02lx\n", reg >> 8, reg & 0xff);
	ctrl->hci_version = reg & 0xffffU;

	u32 hccp1 = mmio_read32(&hccr->cr_hccparams1);
	ctrl->cfc_supported = HCC_CFC(hccp1) ? TRUE : FALSE;
	ctrl->ltc_supported = HCC_LTC(hccp1) ? TRUE : FALSE;
	u32 hccp2 = mmio_read32(&hccr->cr_hccparams2);
	ctrl->cmc_supported = HCC_CMC(hccp2) ? TRUE : FALSE;
	if (ctrl->cmc_supported)
	{
		u32 cmd = mmio_read32(&hcor->or_usbcmd);
		cmd |= CMD_CME;
		mmio_write32(cmd, &hcor->or_usbcmd);
	}

	u32 hcsp3 = mmio_read32(&hccr->cr_hcsparams3);
	ctrl->u1_host_exit_lat = HCS_U1_LATENCY(hcsp3);
	ctrl->u2_host_exit_lat = (u16)HCS_U2_LATENCY(hcsp3);

	xhci_dump_caps(ctrl);

	return 0;
}

static void xhci_lowlevel_stop(struct xhci_ctrl *ctrl)
{
	xhci_reset(ctrl->hcor);

	KprintfH("// Disabling event ring interrupts\n");
	u32 temp = mmio_read32(&ctrl->hcor->or_usbsts);
	mmio_write32(temp & ~STS_EINT, &ctrl->hcor->or_usbsts);
	temp = mmio_read32(&ctrl->ir_set->irq_pending);
	mmio_write32(ER_IRQ_DISABLE(temp), &ctrl->ir_set->irq_pending);

	xhci_roothub_destroy(ctrl->root_hub);
	ctrl->root_hub = NULL;
}

s32 xhci_register(struct xhci_ctrl *ctrl, struct xhci_hccr *hccr, struct xhci_hcor *hcor)
{
	KprintfH("ctrl=%lx, hccr=%lx, hcor=%lx\n", ctrl, hccr, hcor);

	s32 ret = xhci_reset(hcor);
	if (ret)
		goto err;

	/* DMA buffers (rings, DCBAA, contexts, scratchpad, bounce slabs) must live in
	 * Emu68 (Pi-DRAM) RAM the PCIe engine can reach, so the DMA pool is region-restricted;
	 * with no device tree there is no reachable region and we refuse to attach.  CPU-only
	 * metadata uses a separate ordinary Exec pool. */
	dma_mem_init(&ctrl->dma_ctx);
	ctrl->dmaPool = dma_pool_create(&ctrl->dma_ctx);
	ctrl->metaPool = CreatePool(MEMF_FAST | MEMF_PUBLIC, 16384, 8192);
	if (ctrl->dmaPool == NULL || ctrl->metaPool == NULL)
	{
		ret = -ENOMEM;
		goto err_pool;
	}
	KprintfH("memory pools created: dma=%lx meta=%lx\n", (ULONG)ctrl->dmaPool, (ULONG)ctrl->metaPool);

	xhci_td_slab_init(ctrl);
	slab_cache_init(&ctrl->trb_addr_slab, ctrl->metaPool, NULL,
					XHCI_TD_SMALL_TRBS * sizeof(dma_addr_t), DMA_ALIGN_MIN, 256);

	/* Ring segments: one slab, slot = obj_align = seg_size (a power of two), so every
	 * slot is self-aligned and never crosses a 64 KB page boundary.  quirks are set
	 * before xhci_register, so the size is known here. */
	u32 seg_size = (ctrl->quirks & XHCI_QUIRK_TRB_OVERFETCH) ? 2U * SEGMENT_SIZE : SEGMENT_SIZE;
	slab_cache_init(&ctrl->seg_slab, ctrl->metaPool, ctrl->dmaPool, seg_size, seg_size, 8);
	slab_cache_init(&ctrl->bounce_small, ctrl->metaPool, ctrl->dmaPool,
					XHCI_BOUNCE_SMALL_SIZE, DMA_ALIGN_MIN, XHCI_BOUNCE_SMALL_CAP);
	slab_cache_init(&ctrl->bounce_med, ctrl->metaPool, ctrl->dmaPool,
					XHCI_BOUNCE_MED_SIZE, DMA_ALIGN_MIN, XHCI_BOUNCE_MED_CAP);
	slab_cache_init(&ctrl->bounce_large, ctrl->metaPool, ctrl->dmaPool,
					XHCI_BOUNCE_LARGE_SIZE, DMA_ALIGN_MIN, XHCI_BOUNCE_LARGE_CAP);

	_NewMinList(&ctrl->pending_commands);

	ctrl->hccr = hccr;
	ctrl->hcor = hcor;
	ret = xhci_lowlevel_init(ctrl);
	if (ret)
		goto err_pool;

	return 0;

err_pool:
	dma_pool_delete(ctrl->dmaPool);
	ctrl->dmaPool = NULL;
	if (ctrl->metaPool)
	{
		DeletePool(ctrl->metaPool);
		ctrl->metaPool = NULL;
	}

err:
	Kprintf("failed, ret=%ld\n", ret);
	return ret;
}

void xhci_deregister(struct xhci_ctrl *ctrl)
{
	xhci_lowlevel_stop(ctrl);
	xhci_cleanup(ctrl);

	slab_cache_destroy(&ctrl->bounce_large);
	slab_cache_destroy(&ctrl->bounce_med);
	slab_cache_destroy(&ctrl->bounce_small);
	slab_cache_destroy(&ctrl->seg_slab);
	slab_cache_destroy(&ctrl->trb_addr_slab);
	xhci_td_slab_destroy(ctrl);

	if (ctrl->dmaPool)
	{
		dma_pool_delete(ctrl->dmaPool);
		ctrl->dmaPool = NULL;
	}
	if (ctrl->metaPool)
	{
		DeletePool(ctrl->metaPool);
		ctrl->metaPool = NULL;
	}
}
