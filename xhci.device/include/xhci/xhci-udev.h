/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * (C) Copyright 2001
 * Denis Peter, MPL AG Switzerland
 *
 * Adapted for U-Boot driver model
 * (C) Copyright 2015 Google, Inc
 * Note: Part of this code has been derived from linux
 *
 */
#ifndef __XHCI_UDEV_H__
#define __XHCI_UDEV_H__

#include <exec/types.h>
#include <xhci/xhci-xfer.h>
#include <xhci/ch9.h>

/* Everything is aribtrary */
#define USB_MAX_ENDPOINT_CONTEXTS 	31

#define EP_INDEX_TO_ENDPOINT(p) (((p) + 1) >> 1)

/* The xfer descriptor priv_flags (REQ_*) and bounce-class constants live in
 * <xhci/xhci-xfer.h>. */

enum slot_state {
	USB_DEV_SLOT_STATE_DISABLED = 0,
	USB_DEV_SLOT_STATE_ENABLED,
	USB_DEV_SLOT_STATE_DEFAULT,
	USB_DEV_SLOT_STATE_ADDRESSED,
	USB_DEV_SLOT_STATE_CONFIGURED,
};

/* Link Power Management state, embedded in struct usb_device. */
struct udev_lpm_state {
	/* BOS facts adopted from NSCMD_USB_SET_LINK_POWER */
	u8   u1_dev_exit_lat;         /* USB3: bU1DevExitLat (µs); for hubs = upstream link exit lat */
	u16  u2_dev_exit_lat;         /* USB3: wU2DevExitLat (µs) */
	BOOL capable;                 /* USB3: exit latencies + LPM-capable path; USB2: UHCD_LPF_USB2_LPM */
	BOOL besl_supported;          /* USB2: BESL supported */
	u8   besl_baseline;           /* USB2: baseline BESL selector (0-15) */
	u8   besl_deep;               /* USB2: deep BESL selector (0-15) */
	BOOL besl_baseline_valid;
	BOOL besl_deep_valid;

	/* computed */
	u32  max_exit_latency_us;     /* MEL (µs) written to the slot context; 0 = not yet set */
	u8   mel_retry_count;         /* COMP_MEL_ERR ELD retry counter; capped at 3 */
	u32  u1_sel, u1_pel, u1_mel;  /* USB 3.1 Appendix C (ns) */
	u32  u2_sel, u2_pel, u2_mel;

	/* NSCMD_USB_SET_LINK_POWER overrides (0 = driver computes) */
	u16  u1_timeout_override, u2_timeout_override;
	u32  mel_override_us;

	/* USB2 hardware LPM (L1) */
	BOOL usb2_hw_lpm_capable;      /* root-hub port advertises HLC and device is eligible */
	BOOL usb2_hw_lpm_besl_capable; /* root-hub port advertises BLC */

	BOOL setup_done;               /* LPM enable sequence already run after CONFIG_EP */
	BOOL ltm_capable;              /* USB3: BOS SS Device Cap advertises LTM */
};

/* Port-suspend (U3) sequencing state — the one multi-step operation a device
 * carries (xHCI 4.15.1: stop every endpoint ring before the port goes to U3).
 * stops_pending counts the outstanding Stop Endpoint completions; when they
 * drain, u3_port names the root port to write U3 to (0 = device behind an
 * external hub: reply the stashed NSCMD_USB_SET_SUSPEND instead and let the
 * hub class drive the port). */
struct udev_suspend {
	u8 stops_pending;
	u8 u3_port;
	struct xhci_xfer *stash; /* request to reply when the stops drain */
};

/**
 * struct usb_device - information about a USB device
 *
 * With driver model both UCLASS_USB (the USB controllers) and UCLASS_USB_HUB
 * (the hubs) have this as parent data. Hubs are children of controllers or
 * other hubs and there is always a single root hub for each controller.
 * Therefore struct usb_device can always be accessed with
 * dev_get_parent_priv(dev), where dev is a USB device.
 *
 * Pointers exist for obtaining both the device (could be any uclass) and
 * controller (UCLASS_USB) from this structure. The controller does not have
 * a struct usb_device since it is not a device.
 */
struct usb_device {
	u8    xhci_address;				/* Device address as seen by xHCI */
	u8	slot_id;		/* Slot ID for xHCI */
	enum usb_device_speed speed;	/* full/low/high */
	enum slot_state  slot_state;	/* current slot state */

	/* Hub topology facts (NSCMD_USB_UPDATE_HUB) */
	BOOL is_hub;
	u8 hub_num_ports;

	/* Context-ABI (NSCMD_USB_*) device: created via NSCMD_USB_CREATE_DEVICE,
	 * keyed by handle (== slot_id).  ctx_mtt is the stack-supplied multi-TT
	 * state (0 = unknown; 1 = off; 2 = on). */
	BOOL ctx_mode;
	u8 ctx_mtt;

	/* Direct-path token generation, stamped at create: a stale endpoint token
	 * never matches the slot's next tenant (xhci-direct.h). */
	u32 token_gen;

	/* In-flight port-suspend sequencing (the device's only multi-step op). */
	struct udev_suspend suspend;

	/* Split routing data */
	struct usb_device *parent;    /* Parent hub device, NULL for root */
	u8 parent_port;               /* Parent hub downstream port (all speeds) */
	u32 route;                    /* xHCI route string nibble-packed */
	u8 route_depth;
	u8 tt_think_time;             /* Hub TT think time encoding (0-3 -> 8/16/24/32 bit times) */

	/* Link Power Management state (struct udev_lpm_state above): xhci-lpm.c
	 * owns every field; the documented crossers are max_exit_latency_us /
	 * mel_retry_count (MEL command plumbing in xhci-commands.c and
	 * xhci-context.c) and a parent hub's facts, read for the exit-latency
	 * math of its downstream devices. */
	struct udev_lpm_state lpm;

	/* SS hubs: header-decode latency (0.1µs units) and wHubDelay (ns) for the
	 * exit-latency math of downstream devices; filled from
	 * NSCMD_USB_UPDATE_HUB. */
	u8   hub_hdr_dec_lat;
	u16  hub_delay;

	/* Requests state data */
	struct ep_context *ep_context[USB_MAX_ENDPOINT_CONTEXTS];

	/*
	 * Commands to the hardware are passed an "input context" that
	 * tells the hardware what to change in its data structures.
	 * The hardware will return changes in an "output context" that
	 * software must allocate for the hardware.  We need to keep
	 * track of input and output contexts separately because
	 * these commands might fail and we don't trust the hardware.
	 */
	struct xhci_container_ctx *out_ctx;
	/* Used for addressing devices and configuration changes */
	struct xhci_container_ctx *in_ctx;
	
	struct xhci_ctrl *controller; /* xHCI controller */
};

struct XHCIUnit;
struct xhci_ctrl;

/* Access udev */
struct usb_device *xhci_udev_alloc_ctx(struct xhci_ctrl *ctrl); /* context-ABI device: keyed by handle (== slot id) */
struct usb_device *xhci_udev_alloc_root(struct xhci_ctrl *ctrl); /* root-hub anchor: owned by the emulation */
void xhci_udev_free(struct usb_device *udev);

/* Device-tree lifecycle helpers */
void xhci_udev_disconnect(struct usb_device *udev, BOOL recursive);

/* Dispatch */

/* THE completion funnel: sets the result, releases a bounce still mapped on a
 * request that died while queued, replies context-ABI lifecycle ops through
 * their op epilogue, retires everything else through io->complete.  udev may
 * be NULL (late/timeout paths) — only the ctx-op epilogues consume it. */
void xhci_xfer_complete(struct usb_device *udev, struct xhci_xfer *io, s8 err, u32 actual);

/* Send commands to device */
void xhci_udev_clear_feature_halt(struct usb_device *udev, u8 ep_index);
void xhci_udev_clear_tt_buffer(struct usb_device *udev, u8 ep_index, int ep_type);

/* Port suspend (U3) sequencing (struct udev_suspend above) */
BOOL xhci_udev_suspend_device(struct usb_device *udev, u8 root_port, struct xhci_xfer *deferred_req);
void xhci_udev_suspend_stop_done(struct usb_device *udev); /* one Stop Endpoint completed */
/* Abort an in-flight suspend sequence, replying the stashed request with err
 * so the stack isn't left waiting (no-op when nothing is sequencing). */
void xhci_udev_suspend_cancel(struct usb_device *udev, s8 err);
void xhci_udev_resume_device(struct usb_device *udev);
BOOL xhci_udev_suspend_port(struct usb_device *hub_udev, u8 port);
void xhci_udev_resume_port(struct usb_device *hub_udev, u8 port);

/* TRUE while a suspend sequence is mid-flight (endpoint stops outstanding). */
static inline BOOL xhci_udev_suspend_pending(const struct usb_device *udev)
{
	return udev->suspend.stops_pending != 0;
}

/* Endpoint transfer type (USB_ENDPOINT_XFER_*) from the hardware EP context */
s32 xhci_ep_type_for_index(struct usb_device *udev, u8 ep_index);

#endif /* __XHCI_UDEV_H__ */