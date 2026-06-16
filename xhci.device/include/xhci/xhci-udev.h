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
#include <devices/hcd_api.h>
#include <xhci/ch9.h>
/*
 * The EHCI spec says that we must align to at least 32 bytes.  However,
 * some platforms require larger alignment.
 */
#if DMA_ALIGN_MIN > 32
#define USB_DMA_MINALIGN	DMA_ALIGN_MIN
#else
#define USB_DMA_MINALIGN	32
#endif

/* Everything is aribtrary */
#define USB_MAX_ENDPOINT_CONTEXTS 	31
#define USB_ALTSETTINGALLOC			16
#define USB_MAXINTERFACES			16
#define USB_MAXENDPOINTS			16
#define USB_MAXCHILDREN				8	/* This is arbitrary */

#define USB_MAX_ADDRESS 		127

#define USB_CNTL_TIMEOUT 100 /* 100ms timeout */

#define EP_INDEX_TO_ENDPOINT(p) (((p) + 1) >> 1)

struct usb_interface_altsetting {
	struct usb_interface_descriptor desc;

	u8 no_of_ep;

	struct usb_endpoint_descriptor ep_desc[USB_MAXENDPOINTS];
	struct usb_ss_ep_comp_descriptor ss_ep_comp_desc[USB_MAXENDPOINTS];
};

/* Interface */
struct usb_interface {
	u8 interface_number;
	u8 num_altsetting;
	struct usb_interface_altsetting *active_altsetting;

	struct usb_interface_altsetting altsetting[USB_ALTSETTINGALLOC];
};

/* Configuration information.. */
struct usb_config {
	struct MinNode node;
	struct usb_config_descriptor desc;

	u8	no_of_if;	/* number of interfaces */
	struct usb_interface if_desc[USB_MAXINTERFACES];
};

enum {
	/* Maximum packet size; encoded as 0,1,2,3 = 8,16,32,64 */
	PACKET_SIZE_8   = 0,
	PACKET_SIZE_16  = 1,
	PACKET_SIZE_32  = 2,
	PACKET_SIZE_64  = 3,
};

/*************************************************************************
 * Hub Stuff
 */

/*
 * Hub Device descriptor
 * USB Hub class device protocols
 */
#define USB_HUB_PR_FS		0 /* Full speed hub */
#define USB_HUB_PR_HS_NO_TT	0 /* Hi-speed hub without TT */
#define USB_HUB_PR_HS_SINGLE_TT	1 /* Hi-speed hub with single TT */
#define USB_HUB_PR_HS_MULTI_TT	2 /* Hi-speed hub with multiple TT */
#define USB_HUB_PR_SS		3 /* Super speed hub */

/* Transaction Translator Think Times, in bits */
#define HUB_TTTT_8_BITS		0x00
#define HUB_TTTT_16_BITS	0x20
#define HUB_TTTT_24_BITS	0x40
#define HUB_TTTT_32_BITS	0x60

/* Hub descriptor */
struct usb_hub_descriptor {
	__le8  bLength;
	__le8  bDescriptorType;
	__le8  bNbrPorts;
	__le16 wHubCharacteristics;
	__le8  bPwrOn2PwrGood;
	__le8  bHubContrCurrent;
	/* 2.0 and 3.0 hubs differ here */
	union {
		struct {
			/* add 1 bit for hub status change; round to bytes */
			__le8 DeviceRemovable[(USB_MAXCHILDREN + 1 + 7) / 8];
			__le8 PortPowerCtrlMask[(USB_MAXCHILDREN + 1 + 7) / 8];
		} __attribute__ ((packed)) hs;

		struct {
			__le8 bHubHdrDecLat;
			__le16 wHubDelay;
			__le16 DeviceRemovable;
		} __attribute__ ((packed)) ss;
	} u;
} __attribute__ ((packed));

/* Flags for use in DriverPrivate1 */
#define REQ_INTERNAL 0x1       /* Internal request, free instead of reply */
#define REQ_ENQUEUED 0x2       /* Request was already enqueued to EP */
#define REQ_ON_RING 0x4        /* Request is currently on the transfer ring */
#define REQ_DMA_MAPPED 0x8      /* Request data buffer is DMA mapped */
#define REQ_HUB_DESC_FETCH 0x10 /* Internal hub descriptor fetch before CONFIG_EP */
#define REQ_BOS_FETCH      0x20 /* Internal BOS descriptor fetch (phase 1+2) before CONFIG_EP */
#define REQ_SET_SEL      0x80   /* Internal SET_SEL OUT transfer for USB3 LPM */

/* bounce class — which bounce slab the bounce buffer came from (0 = dma_alloc fallback) */
#define REQ_BOUNCE_CLASS_SHIFT 8
#define REQ_BOUNCE_CLASS_MASK  (0x7U << REQ_BOUNCE_CLASS_SHIFT)
#define REQ_BOUNCE_CLASS_NONE  0
#define REQ_BOUNCE_CLASS_SMALL 1
#define REQ_BOUNCE_CLASS_MED   2
#define REQ_BOUNCE_CLASS_LARGE 3

enum slot_state {
	USB_DEV_SLOT_STATE_DISABLED = 0,
	USB_DEV_SLOT_STATE_ENABLED,
	USB_DEV_SLOT_STATE_DEFAULT,
	USB_DEV_SLOT_STATE_ADDRESSED,
	USB_DEV_SLOT_STATE_CONFIGURED,
};

/* Multi-step device operations - exactly one in flight per device.  Each op is
 * a chain of async completions; the cross-module completion sites feed
 * xhci_udev_op_advance() and the op-specific steps live in one dispatch. */
enum udev_op {
	UDEV_OP_NONE = 0,
	UDEV_OP_CONFIGURE,  /* descriptor prefetch (hub/BOS) ahead of SET_CONFIGURATION */
	UDEV_OP_LPM_ENABLE, /* MEL Evaluate Context -> port timeouts + SET_SEL -> device-initiated U1/U2 */
	UDEV_OP_SUSPEND,    /* endpoint ring stops -> port U3 / forward SetPortFeature to hub */
};

enum udev_op_event {
	UDEV_OP_EVENT_EP0_RECOVERED, /* EP0 stall/timeout recovery finished (Set TR Deq done) */
	UDEV_OP_EVENT_MEL_EVAL_DONE, /* Evaluate Context (MEL) succeeded */
	UDEV_OP_EVENT_SET_SEL_DONE,  /* SET_SEL completed on the wire */
	UDEV_OP_EVENT_STOP_DONE,     /* one Stop Endpoint completion (suspend) */
};

struct udev_operation {
	enum udev_op op;
	u8 step;                    /* op-specific progress */
	u8 waits;                   /* outstanding completions within the current step */
	u8 arg;                     /* SUSPEND: root port to write U3 to (0 = behind a hub) */
	struct USBIORequest *stash; /* request to resume/forward when the op completes */
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
	u16	virtual_address;			/* Device address as seen by the driver user */
	u8    xhci_address;				/* Device address as seen by xHCI */
	u8	slot_id;		/* Slot ID for xHCI */
	enum usb_device_speed speed;	/* full/low/high */
	enum slot_state  slot_state;	/* current slot state */

	struct MinList configurations; /* configurations captured from GET_CONFIGURATION replies */
	struct usb_config *active_config;
	u8 product_string_index; /* iProduct from device descriptor */

	u8 device_protocol;      /* bDeviceProtocol from the device descriptor
	                          * (HS hubs: 2 = multi-TT capable) */

	/* Hub translation support */
	BOOL is_hub;
	BOOL ss_hub_emulation;
	BOOL ss_hub_depth_set;
	u8 hub_num_ports;
	struct usb_hub_descriptor ss_hub_desc;

	/* The single in-flight multi-step operation (configure prefetch, LPM
	 * enable, port suspend) - see enum udev_op / xhci_udev_op_advance(). */
	struct udev_operation op;

	/* Split routing data */
	struct usb_device *parent;    /* Parent hub device, NULL for root */
	u8 parent_port;               /* Parent hub downstream port (all speeds) */
	u32 route;                    /* xHCI route string nibble-packed */
	u8 route_depth;
	u8 tt_think_time;             /* Hub TT think time encoding (0-3 -> 8/16/24/32 bit times) */

	/* Max Exit Latency / LPM fields (populated from BOS descriptor before CONFIG_EP) */
	u8   u1_dev_exit_lat;         /* USB3: bU1DevExitLat from BOS SS Device Cap (µs); for hubs = upstream link exit lat */
	u16  u2_dev_exit_lat;         /* USB3: wU2DevExitLat from BOS SS Device Cap (µs) */
	BOOL lpm_capable;             /* USB2: LPM (L1) supported per USB_20_EXTENSION_ATT_LINK_POWER_MANAGEMENT */
	BOOL besl_supported;          /* USB2: BESL supported per USB_20_EXTENSION_ATT_BESL_SUPPORTED */
	u8   besl_baseline;           /* USB2: baseline BESL selector (0-15), BOS USB2 Ext bmAttributes bits 11:8 */
	u8   besl_deep;               /* USB2: deep BESL selector (0-15), bits 15:12 */
	BOOL besl_baseline_valid;     /* USB2: baseline BESL value present (bit 3) */
	BOOL besl_deep_valid;         /* USB2: deep BESL value present (bit 4) */
	u32  max_exit_latency_us;     /* Calculated MEL in µs written to slot context; 0 = not yet set */
	u8   mel_retry_count;         /* COMP_MEL_ERR ELD retry counter; capped at 3 */

	/* USB3 LPM parameters, computed per USB 3.1 Appendix C (all in nanoseconds) */
	u32  u1_sel, u1_pel, u1_mel;
	u32  u2_sel, u2_pel, u2_mel;

	/* Hub-encoded U1/U2 inactivity timeouts programmed on the parent port;
	 * USB3_LPM_DISABLED if the state was not enabled */
	u16  u1_timeout, u2_timeout;

	/* USB2 hardware LPM (L1) */
	BOOL usb2_hw_lpm_capable;      /* root-hub port advertises HLC and device is eligible */
	BOOL usb2_hw_lpm_besl_capable; /* root-hub port advertises BLC */

	BOOL lpm_setup_done;           /* LPM enable sequence already run after CONFIG_EP */
	BOOL ltm_capable;              /* USB3: BOS SS Device Cap advertises LTM */
	BOOL ltm_setup_done;           /* SET_FEATURE(LTM_ENABLE) already sent */

	BOOL bos_fetched;              /* BOS pre-fetch already completed; reuse cached LPM data */
	BOOL hub_desc_fetched;         /* Hub descriptor pre-fetch already completed; reuse cached ss_hub_desc */

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
struct usb_device *xhci_udev_alloc(struct xhci_ctrl *ctrl, u16 virtual_address);
struct usb_device *xhci_udev_get(struct XHCIUnit *unit, u16 virtual_address);
void xhci_udev_free(struct usb_device *udev);

/* Device-tree lifecycle helpers (also used by the SS-hub emulation in xhci-hub.c) */
struct usb_device *xhci_udev_find_child_on_port(struct usb_device *hub, u32 port);
void xhci_udev_disconnect(struct usb_device *udev, BOOL recursive);

/* Dispatch */
s8 xhci_udev_send_ctrl(struct usb_device *udev, struct USBIORequest *io);
s8 xhci_udev_send(struct USBIORequest *req);

/* Track replies */
void xhci_udev_io_reply_failed(struct xhci_ctrl *ctrl, struct USBIORequest *io, s8 err);
void xhci_udev_io_reply_data(struct usb_device *udev, struct USBIORequest *io, s8 err, u32 actual);

/* Resume a SET_CONFIGURATION deferred behind a BOS/hub pre-fetch (EP0 must be idle) */
void xhci_udev_run_pending_set_config(struct usb_device *udev);

/* Send commands to device */
void xhci_udev_clear_feature_halt(struct usb_device *udev, u8 ep_index);
void xhci_udev_clear_tt_buffer(struct usb_device *udev, u8 ep_index, int ep_type);

/* Build and submit a fire-and-forget internal control request on EP0 (shared by
 * the LPM senders in xhci-lpm.c and the halt/TT recovery senders here). */
void xhci_udev_send_control_request(struct usb_device *udev, u8 ep_index,
                                    u8 bmRequestType, u8 bRequest,
                                    u16 wValue, u16 wIndex, u16 wLength,
                                    BOOL enqueue);

/* Multi-step operation sequencing (see enum udev_op) */
BOOL xhci_udev_op_begin(struct usb_device *udev, enum udev_op op, struct USBIORequest *stash);
void xhci_udev_op_advance(struct usb_device *udev, enum udev_op_event event);
void xhci_udev_op_cancel(struct usb_device *udev, enum udev_op which, s8 err); /* UDEV_OP_NONE = any */

/* Port suspend (U3) sequencing */
BOOL xhci_udev_suspend_port(struct usb_device *hub_udev, u8 port);
void xhci_udev_resume_port(struct usb_device *hub_udev, u8 port);

/* Descriptor access */
s32 xhci_ep_type_for_index(struct usb_device *udev, u8 ep_index);

#endif /* __XHCI_UDEV_H__ */