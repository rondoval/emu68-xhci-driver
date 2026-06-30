/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * USB HOST XHCI Controller
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

#ifndef HOST_XHCI_H_
#define HOST_XHCI_H_

#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME (*(struct ExecBase **)4UL)
#include <proto/exec.h>
#endif

#include <bits.h>
#include <iomem.h>
#include <slab.h>
#include <dma_mem.h>
#include <devices/hcd_api.h>
#include <xhci/xhci-udev.h>

struct pci_dev;

#define XHCI_ALIGNMENT 64
/* Generic timeout for XHCI events */
#define XHCI_TIMEOUT 5000
/* Up to 16 ms to halt an HC */
#define XHCI_MAX_HALT_USEC (16 * 1000)

#define XHCI_MAX_RESET_USEC (250 * 1000)

#include <xhci/xhci-regs.h>

/* USB3 LPM exit-latency timing constants (USB 3.1 Appendix C, all ns) */
#define USB_TP_TRANSMISSION_DELAY 40
#define USB_PING_RESPONSE_TIME 400
/* USB2 hardware LPM defaults */
#define XHCI_L1_TIMEOUT 512 /* us; programmed into PORTHLPMC in 256us units */
#define XHCI_DEFAULT_BESL 4

/**
 * struct xhci_device_context_array
 * @dev_context_ptr	array of 64-bit DMA addresses for device contexts
 */
struct xhci_device_context_array
{
	/* 64-bit device addresses; we only write 32-bit addresses */
	__le64 dev_context_ptrs[MAX_HC_SLOTS];
};

/*
 * TRBS_PER_SEGMENT must be a multiple of 4,
 * since the command ring is 64-byte aligned.
 * It must also be greater than 16.
 */
#define TRBS_PER_SEGMENT 256
/* Allow two commands + a link TRB, along with any reserved command TRBs */
#define MAX_RSVD_CMD_TRBS (TRBS_PER_SEGMENT - 3)
#define SEGMENT_SIZE (TRBS_PER_SEGMENT * 16)
/* SEGMENT_SHIFT should be log2(SEGMENT_SIZE).
 * Change this if you change TRBS_PER_SEGMENT!
 */
#define SEGMENT_SHIFT 12
/* TRB buffer pointers can't cross 64KB boundaries */
#define TRB_MAX_BUFF_SHIFT 16
#define TRB_MAX_BUFF_SIZE (1 << TRB_MAX_BUFF_SHIFT)

struct xhci_erst_entry
{
	/* 64-bit event ring segment address */
	__le64 seg_addr;
	__le32 seg_size;
	/* Set to zero */
	__le32 rsvd;
};

struct xhci_erst
{
	struct xhci_erst_entry *entries;
	u32 num_entries;
	/* Num entries the ERST can contain */
	u32 erst_size;
};

struct xhci_scratchpad
{
	void *scratchpad;
	u64 *sp_array;
};

/* Poll every 60 seconds */
#define POLL_TIMEOUT 60
/* Stop endpoint command timeout (secs) for URB cancellation watchdog timer */
#define XHCI_STOP_EP_CMD_TIMEOUT 5
/* XXX: Make these module parameters */

#define CTX_SIZE(_hcc) (HCC_64BYTE_CONTEXT(_hcc) ? 64 : 32)

/* Controller quirks: set once at probe from the PCI identity (the onboard
 * BCM2711 controller needs none), checked as a bitmask.  Only bits with an
 * implementation behind them are defined. */
#define XHCI_QUIRK_TRB_OVERFETCH BIT(0) /* VL805: HC prefetches past segment end; pad ring segment allocations */
#define XHCI_QUIRK_SS_BULK_OUT   BIT(1) /* VL805: SS bulk OUT bursts corrupt for mass-storage behind a hub */

struct xhci_ctrl
{
	struct xhci_hccr *hccr; /* R/O registers, not need for volatile */
	struct xhci_hcor *hcor;
	struct xhci_doorbell_array *dba;
	struct xhci_run_regs *run_regs;
	struct xhci_device_context_array *dcbaa __attribute__((aligned(DMA_ALIGN_MIN)));
	struct xhci_ring *event_ring;
	struct xhci_ring *cmd_ring;
	struct xhci_intr_reg *ir_set;
	struct xhci_erst erst;
	struct xhci_scratchpad *scratchpad;
	struct xhci_root_hub *root_hub;
	u16 hci_version;
	u32 quirks;			  /* XHCI_QUIRK_* bitmask, set at probe */
	u32 vl805_fw_version; /* VL805 MCU firmware version (PCI cfg 0x50); 0 for other controllers */
	BOOL cfc_supported;	  /* HCC_CFC: per-TRB Frame ID is reliable */
	BOOL cmc_supported;	  /* HCC_CMC: controller enforces MEL on ConfigEP/EvalCtx */
	BOOL ltc_supported;	  /* HCC_LTC: controller consumes device LTM packets */
	u8 u1_host_exit_lat;  /* HCSPARAMS3 bits 7:0: root hub U1->U0 latency (us) */
	u16 u2_host_exit_lat; /* HCSPARAMS3 bits 31:16: root hub U2->U0 latency (us) */
	u32 page_size;		  /* PAGESIZE register decoded to bytes; cached at init */

	struct dma_mem_ctx dma_ctx; /* Emu68 (DMA-reachable) RAM regions; backs dmaPool */
	struct dma_pool *dmaPool;	/* region-restricted DMA pool (Emu68 RAM) for DMA buffers */
	APTR metaPool;				/* ordinary Exec pool for CPU-only metadata */
#define XHCI_TD_SMALL_TRBS 8	   /* trb_addr_slab covers up to this many TRBs */
#define XHCI_BOUNCE_SMALL_SIZE 256 /* covers RT ISO 192 + tiny ctrl/desc */
#define XHCI_BOUNCE_SMALL_CAP 256
#define XHCI_BOUNCE_MED_SIZE (32 * 1024) /* covers ≤32KiB bulk reads */
#define XHCI_BOUNCE_MED_CAP 8
#define XHCI_BOUNCE_LARGE_SIZE (2 * 1024 * 1024) /* mass storage 2MB transfers */
#define XHCI_BOUNCE_LARGE_CAP 2					 /* Poseidon 1 bulk/EP */
	struct slab_cache td_slab;					 /* one struct xhci_td per slot */
	struct slab_cache trb_addr_slab;			 /* XHCI_TD_SMALL_TRBS * sizeof(dma_addr_t) per slot */
	struct slab_cache seg_slab;					 /* one ring segment (seg_size bytes, self-aligned) per slot */
	struct slab_cache bounce_small;				 /* XHCI_BOUNCE_SMALL_SIZE bytes per slot */
	struct slab_cache bounce_med;				 /* XHCI_BOUNCE_MED_SIZE bytes per slot */
	struct slab_cache bounce_large;				 /* XHCI_BOUNCE_LARGE_SIZE bytes per slot */
	struct Library *utilityBase;
	struct pci_dev *pci_dev;
	struct usb_device *devices_by_virtual_address[USB_MAX_ADDRESS + 1];
	struct usb_device *devices_by_slot_id[MAX_HC_SLOTS];

	struct usb_device *pending_parent; /* parent hub pending for next default-address child */
	u8 pending_parent_port;
	enum usb_device_speed pending_parent_speed;

	struct MinList pending_commands; /* list of pending commands */
	BOOL cmd_abort_pending;			 /* TRUE while CA bit is asserted; doorbell suppressed */
};

/* Pre-DMA flush for a buffer.  @flags is passed straight to CachePreDMA: 0 for a
 * plain clean+invalidate, or DMA_ReadFromRAM for an OUT buffer (device reads RAM)
 * which only needs a clean. */
inline void xhci_flush_cache(void *addr, ULONG len, ULONG flags)
{
	CachePreDMA((APTR)addr, &len, flags);
}

inline void xhci_inval_cache(void *addr, ULONG len)
{
	CachePostDMA((APTR)addr, &len, 0);
}

static inline void *xhci_malloc_page_bounded(struct xhci_ctrl *ctrl, u32 size, u32 align)
{
	/* A block aligned to round_up_pow2(size) cannot cross any power-of-two boundary
	 * >= size (it nests inside one self-aligned slot, and that slot divides the coarser
	 * boundary), so rounding the alignment up to cover the size makes the buffer never
	 * cross the controller PAGESIZE boundary.  align must be a power of two (callers pass
	 * XHCI_ALIGNMENT); size <= page_size for every caller. */
	u32 eff = round_up_pow2_u32(size);
	if (eff < align)
		eff = align;

	void *ptr = dma_zalloc(ctrl->dmaPool, eff, size);
	if (ptr)
		xhci_flush_cache(ptr, size, 0);
	return ptr;
}

u32 *xhci_find_next_capability(struct xhci_ctrl *ctrl, u32 cap_id, u32 *init_offset);
struct xhci_protocol_caps xhci_get_protocol_caps(u32 *base_address);

/**
 * xhci_deregister() - Unregister an XHCI controller
 *
 * @dev:	Controller device
 * Return: 0 if registered, -ve on error
 */
void xhci_deregister(struct xhci_ctrl *ctrl);

/**
 * xhci_register() - Register a new XHCI controller
 *
 * @dev:	Controller device
 * @hccr:	Host controller control registers
 * @hcor:	Not sure what this means
 * Return: 0 if registered, -ve on error
 */
s32 xhci_register(struct xhci_ctrl *ctrl, struct xhci_hccr *hccr,
				  struct xhci_hcor *hcor);

/**
 * xhci_reset_quiesce() - halt the HC so all DMA stops (pre-reset quiesce)
 *
 * @ctrl:	Controller
 */
void xhci_reset_quiesce(struct xhci_ctrl *ctrl);

#endif /* HOST_XHCI_H_ */
