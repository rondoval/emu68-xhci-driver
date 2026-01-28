/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2001
 * Denis Peter, MPL AG Switzerland
 *
 * Adapted for U-Boot driver model
 * (C) Copyright 2015 Google, Inc
 * Note: Part of this code has been derived from linux
 *
 */
#ifndef _USB_H_
#define _USB_H_

#include <exec/lists.h>

#include <compat.h>
#include <xhci/usb_defs.h>
#include <xhci/ch9.h>
#include <xhci/xhci-events.h>
#include <devices/usbhardware.h>

/*
 * The EHCI spec says that we must align to at least 32 bytes.  However,
 * some platforms require larger alignment.
 */
#if ARCH_DMA_MINALIGN > 32
#define USB_DMA_MINALIGN	ARCH_DMA_MINALIGN
#else
#define USB_DMA_MINALIGN	32
#endif

/* Everything is aribtrary */
#define USB_ALTSETTINGALLOC		16
#define USB_MAXINTERFACES		16
#define USB_MAXENDPOINTS		16
#define USB_MAXCHILDREN			8	/* This is arbitrary */

#define USB_MAX_ADDRESS 		127

#define USB_CNTL_TIMEOUT 100 /* 100ms timeout */

struct usb_interface_altsetting {
	struct usb_interface_descriptor desc;

	__u8 no_of_ep;

	struct usb_endpoint_descriptor ep_desc[USB_MAXENDPOINTS];
	struct usb_ss_ep_comp_descriptor ss_ep_comp_desc[USB_MAXENDPOINTS];
};

/* Interface */
struct usb_interface {
	__u8 interface_number;
	__u8 num_altsetting;
	struct usb_interface_altsetting *active_altsetting;

	struct usb_interface_altsetting altsetting[USB_ALTSETTINGALLOC];
};

/* Configuration information.. */
struct usb_config {
	struct MinNode node;
	struct usb_config_descriptor desc;

	__u8	no_of_if;	/* number of interfaces */
	struct usb_interface if_desc[USB_MAXINTERFACES];
};

enum {
	/* Maximum packet size; encoded as 0,1,2,3 = 8,16,32,64 */
	PACKET_SIZE_8   = 0,
	PACKET_SIZE_16  = 1,
	PACKET_SIZE_32  = 2,
	PACKET_SIZE_64  = 3,
};

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
	BOOL used;
	unsigned int	poseidon_address;			/* Device address as seen by Poseidon */
	unsigned int    xhci_address;				/* Device address as seen by xHCI */
	unsigned int	slot_id;		/* Slot ID for xHCI */
	enum usb_device_speed speed;	/* full/low/high */
	enum slot_state  slot_state;	/* current slot state */

	struct MinList configurations; /* configurations captured from GET_CONFIGURATION replies */
	struct usb_config *active_config;

	/* Split routing data */
	struct usb_device *parent;    /* Parent hub device, NULL for root */
	unsigned int parent_port;     /* Parent hub downstream port (all speeds) */
	unsigned int route;           /* xHCI route string nibble-packed */
	unsigned int tt_think_time;   /* Hub TT think time encoding (0-3 -> 8/16/24/32 bit times) */

	/* Requests state data */
	struct ep_context *ep_context[USB_MAXENDPOINTS];
	
	struct xhci_ctrl *controller; /* xHCI controller */
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
	unsigned char  bLength;
	unsigned char  bDescriptorType;
	unsigned char  bNbrPorts;
	unsigned short wHubCharacteristics;
	unsigned char  bPwrOn2PwrGood;
	unsigned char  bHubContrCurrent;
	/* 2.0 and 3.0 hubs differ here */
	union {
		struct {
			/* add 1 bit for hub status change; round to bytes */
			__u8 DeviceRemovable[(USB_MAXCHILDREN + 1 + 7) / 8];
			__u8 PortPowerCtrlMask[(USB_MAXCHILDREN + 1 + 7) / 8];
		} __attribute__ ((packed)) hs;

		struct {
			__u8 bHubHdrDecLat;
			__le16 wHubDelay;
			__le16 DeviceRemovable;
		} __attribute__ ((packed)) ss;
	} u;
} __attribute__ ((packed));


struct xhci_ctrl;

#endif /*_USB_H_ */
