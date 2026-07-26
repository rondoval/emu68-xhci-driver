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

#include <exec/semaphores.h>

#include <bits.h>
#include <cache_ops.h>
#include <iomem.h>
#include <memory.h>
#include <perf.h>
#include <slab.h>
#include <dma_mem.h>
#include <drv_timer.h>
#include <devices/usbhcd_context.h>

struct usb_device;

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

/* Perf slots (emu68-common <perf.h>), reported as [xhci] every ~2 s from the
 * unit-task tick (XHCI_PROF_REPORT_TICKS).  Order must match
 * xhci_perf_names[] in xhci.c.  xfer_lock wait/hold rides the lock_prof
 * instance alongside. */
enum XhciProfSlot
{
	XP_SUBMIT_MAP,  /* DMA map: reachability test, bounce alloc+copy, payload clean */
	XP_SUBMIT_EMIT, /* ring reserve + TRB emission + TD bookkeeping + giveback */
	XP_EVT_DRAIN,   /* whole xhci_process_event_trb drain */
	XP_EVT_HOOK,    /* done-hook CallHookPkt (the stack's completion work) */
	XP_IRQ_TO_TASK, /* ISR signal -> unit-task pickup (IMOD + scheduling) */
	XP_SLOT_COUNT
};

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
	u16 ctx_size;		  /* CTX_SIZE(HCCPARAMS1) cached at register: 32 or 64 bytes */
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
#define XHCI_TD_SMALL_TRBS 34	   /* trb_addr_slab covers up to this many TRBs
                                    * (a 2 MiB TD = 33 data TRBs stays off the Exec pool) */
#define XHCI_BOUNCE_SMALL_SIZE 256 /* covers RT ISO 192 + tiny ctrl/desc */
#define XHCI_BOUNCE_SMALL_CAP 256
#define XHCI_BOUNCE_MED_SIZE (32 * 1024) /* covers ≤32KiB bulk reads */
#define XHCI_BOUNCE_MED_CAP 8
#define XHCI_BOUNCE_LARGE_SIZE (2 * 1024 * 1024) /* mass storage 2MB transfers */
#define XHCI_BOUNCE_LARGE_CAP 2					 /* Poseidon 1 bulk/EP */
	struct slab_cache td_slab;					 /* one struct xhci_td per slot */
	struct slab_cache xfer_slab;				 /* one struct xhci_xfer per slot (direct + internal xfers) */
	struct slab_cache trb_addr_slab;			 /* XHCI_TD_SMALL_TRBS * sizeof(dma_addr_t) per slot */
	struct slab_cache seg_slab;					 /* one ring segment (seg_size bytes, self-aligned) per slot */
	struct slab_cache bounce_small;				 /* XHCI_BOUNCE_SMALL_SIZE bytes per slot */
	struct slab_cache bounce_med;				 /* XHCI_BOUNCE_MED_SIZE bytes per slot */
	struct slab_cache bounce_large;				 /* XHCI_BOUNCE_LARGE_SIZE bytes per slot */
	struct Library *utilityBase;
	struct pci_dev *pci_dev;
	struct usb_device *devices_by_slot_id[MAX_HC_SLOTS];

	struct MinList pending_commands; /* list of pending commands */
	BOOL cmd_abort_pending;			 /* TRUE while CA bit is asserted; doorbell suppressed */

	/* Transfer-plane lock: the unit task holds it around each of its work
	 * blocks (event processing, command dispatch, timeout scans) and the
	 * caller-context direct entries (xhci_direct_submit/ctrl_submit/abort)
	 * hold it around theirs, so every ring/TD/pool touch is serialized.
	 * Exec semaphores nest within one task, so lock-held paths may call each
	 * other freely. */
	struct SignalSemaphore xfer_lock;

	/* IMAN with IP/IE masked out, captured once at interrupt start: the ISR
	 * and the rearm write constants instead of read-modify-write over PCIe
	 * (IMAN's reserved bits are RsvdP; IP is W1C). */
	u32 iman_base;

	/* The unit task's persistent sleep timer for the root-hub port waits
	 * (rh_sleep_unlocked).  MsgPorts are task-bound, so the unit task alone
	 * opens and closes it — every port handler runs on the unit task.  req ==
	 * NULL (timer.device unavailable) degrades the waits to hot polls. */
	struct drv_timer sleep_timer;

	/* The direct transfer path (xhci-direct.c): the stack's completion hook
	 * from NSCMD_USB_ATTACH and the per-create token generation counter. */
	struct Hook *stack_done_hook;
	APTR stack_done_obj;
	u32 token_gen_counter;

	/* Perf instance (emu68-common <perf.h>): probes write under PROFILE;
	 * storage is unconditional so all tiers share one struct layout.  The
	 * unit-task tick reports both instances every ~2 s. */
	struct perf_counter perfSlots[XP_SLOT_COUNT];
	struct perf perf;
	struct lock_prof lockProf; /* xfer_lock wait/hold (outermost only) */
	u32 profTicks;             /* tick divider for the report cadence */
	u32 irq_t0;                /* ISR timestamp feeding XP_IRQ_TO_TASK */
};

/* DMA cache maintenance comes straight from emu68-common's cache_ops.h
 * (cache_pre_dma / cache_post_dma / DMAF_NoSync), included above. */

/* A TD's TRB-address bookkeeping array: slab-backed for small TDs (the common
 * case), metaPool otherwise.  Alloc and free must agree on the size class. */
static inline dma_addr_t *xhci_td_trb_addrs_alloc(struct xhci_ctrl *ctrl, u32 num_trbs)
{
	if (likely(num_trbs <= XHCI_TD_SMALL_TRBS))
		return slab_alloc(&ctrl->trb_addr_slab);
	return pool_alloc(ctrl->metaPool, num_trbs * sizeof(dma_addr_t));
}

static inline void xhci_td_trb_addrs_free(struct xhci_ctrl *ctrl, dma_addr_t *trb_addrs, u32 num_trbs)
{
	if (likely(num_trbs <= XHCI_TD_SMALL_TRBS))
		slab_free(&ctrl->trb_addr_slab, trb_addrs);
	else
		pool_free(ctrl->metaPool, trb_addrs);
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
		cache_pre_dma(ptr, size, 0);
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
