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
#define REQ_RT_ISO_CLONE 0x40   /* Cloned IO req for RT ISO; pool_free instead of ReplyMsg */

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

	/* Hub translation support */
	BOOL is_hub;
	BOOL ss_hub_emulation;
	BOOL ss_hub_depth_set;
	u8 hub_num_ports;
	struct usb_hub_descriptor ss_hub_desc;

	/* Deferred CONFIG_EP: stash IOReq while we pre-fetch hub descriptor */
	struct USBIORequest *pending_set_config_req;

	/* Split routing data */
	struct usb_device *parent;    /* Parent hub device, NULL for root */
	u8 parent_port;               /* Parent hub downstream port (all speeds) */
	u32 route;                    /* xHCI route string nibble-packed */
	u8 route_depth;
	u8 tt_think_time;             /* Hub TT think time encoding (0-3 -> 8/16/24/32 bit times) */

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

/* Dispatch */
s8 xhci_udev_send_ctrl(struct usb_device *udev, struct USBIORequest *io);
s8 xhci_udev_send(struct USBIORequest *req);

/* Track replies */
void xhci_udev_io_reply_failed(struct xhci_ctrl *ctrl, struct USBIORequest *io, s8 err);
void xhci_udev_io_reply_data(struct usb_device *udev, struct USBIORequest *io, s8 err, u32 actual);

/* Send commands to device */
void xhci_udev_clear_feature_halt(struct usb_device *udev, u8 ep_index);
void xhci_udev_clear_tt_buffer(struct usb_device *udev, u8 ep_index, int ep_type);

/* Descriptor access */
s32 xhci_ep_type_for_index(struct usb_device *udev, u8 ep_index);

#endif /* __XHCI_UDEV_H__ */