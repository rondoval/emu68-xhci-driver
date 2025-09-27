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

#include <stdbool.h>
#include <compat.h>
#include <usb_defs.h>
#include <ch9.h>

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
#define USB_ALTSETTINGALLOC		4
#define USB_MAXALTSETTING		128	/* Hard limit */

#define USB_MAX_DEVICE			32
#define USB_MAXCONFIG			8
#define USB_MAXINTERFACES		8
#define USB_MAXENDPOINTS		16
#define USB_MAXCHILDREN			8	/* This is arbitrary */
#define USB_MAX_HUB			16

#define USB_CNTL_TIMEOUT 100 /* 100ms timeout */

/*
 * This is the timeout to allow for submitting an urb in ms. We allow more
 * time for a BULK device to react - some are slow.
 */
#define USB_TIMEOUT_MS(pipe) (usb_pipebulk(pipe) ? 5000 : 1000)

/*
 * The xhcd hcd driver prepares only a limited number interfaces / endpoints.
 * Define this limit so that drivers do not exceed it.
 */
#define USB_MAX_ACTIVE_INTERFACES	2

/* device request (setup) */
struct devrequest {
	__u8	requesttype;
	__u8	request;
	__le16	value;
	__le16	index;
	__le16	length;
} __attribute__ ((packed));

/* Interface */
struct usb_interface {
	struct usb_interface_descriptor desc;

	__u8	no_of_ep;
	__u8	num_altsetting;
	__u8	act_altsetting;

	struct usb_endpoint_descriptor ep_desc[USB_MAXENDPOINTS];
	/*
	 * Super Speed Device will have Super Speed Endpoint
	 * Companion Descriptor  (section 9.6.7 of usb 3.0 spec)
	 * Revision 1.0 June 6th 2011
	 */
	struct usb_ss_ep_comp_descriptor ss_ep_comp_desc[USB_MAXENDPOINTS];
} __attribute__ ((packed));

/* Configuration information.. */
struct usb_config {
	struct MinNode node;
	struct usb_config_descriptor desc;

	__u8	no_of_if;	/* number of interfaces */
	struct usb_interface if_desc[USB_MAXINTERFACES];
} __attribute__ ((packed));

enum {
	/* Maximum packet size; encoded as 0,1,2,3 = 8,16,32,64 */
	PACKET_SIZE_8   = 0,
	PACKET_SIZE_16  = 1,
	PACKET_SIZE_32  = 2,
	PACKET_SIZE_64  = 3,
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
	unsigned int	devnum;			/* Device number on USB bus */
	enum usb_device_speed speed;	/* full/low/high */

	/* Maximum packet size; one of: PACKET_SIZE_* */
	int maxpacketsize0;

	struct MinList configurations; /* configurations captured from GET_CONFIGURATION replies */

	struct usb_device *parent;    /* Parent hub device, NULL for root */
	unsigned int portnr;          /* Downstream port number on parent hub (1-based) */
	unsigned int route;           /* xHCI route string nibble-packed */
	unsigned int tt_slot;         /* Parent hub slot ID for TT scheduling */
	unsigned int tt_port;         /* Parent hub downstream port for TT scheduling */

	/*
	 * Child devices -  if this is a hub device
	 * Each instance needs its own set of data structures.
	 */
	unsigned long status;
	int act_len;			/* transferred bytes */
	/* slot_id - for xHCI enabled devices */
	unsigned int slot_id;

	struct xhci_ctrl *controller; /* xHCI controller */
};


/**********************************************************************
 * this is how the lowlevel part communicate with the outer world
 */

#define usb_reset_root_port(dev)

int submit_bulk_msg(struct usb_device *dev, unsigned long pipe,
		void *buffer, int transfer_len, unsigned int maxpacket,
		unsigned int timeout_ms);
int submit_control_msg(struct usb_device *dev, unsigned long pipe, void *buffer,
		int transfer_len, struct devrequest *setup, unsigned int maxpacket,
		unsigned int timeout_ms);
int submit_int_msg(struct usb_device *dev, unsigned long pipe, void *buffer,
		int transfer_len, unsigned int maxpacket, int interval, bool nonblock);


/* Defines */

/*
 * Calling this entity a "pipe" is glorifying it. A USB pipe
 * is something embarrassingly simple: it basically consists
 * of the following information:
 *  - device number (7 bits)
 *  - endpoint number (4 bits)
 *  - current Data0/1 state (1 bit)
 *  - direction (1 bit)
 *  - speed (2 bits)
 *  - max packet size (2 bits: 8, 16, 32 or 64)
 *  - pipe type (2 bits: control, interrupt, bulk, isochronous)
 *
 * That's 18 bits. Really. Nothing more. And the USB people have
 * documented these eighteen bits as some kind of glorious
 * virtual data structure.
 *
 * Let's not fall in that trap. We'll just encode it as a simple
 * unsigned int. The encoding is:
 *
 *  - max size:		bits 0-1	(00 = 8, 01 = 16, 10 = 32, 11 = 64)
 *  - direction:	bit 7		(0 = Host-to-Device [Out],
 *					(1 = Device-to-Host [In])
 *  - device:		bits 8-14
 *  - endpoint:		bits 15-18
 *  - Data0/1:		bit 19
 *  - pipe type:	bits 30-31	(00 = isochronous, 01 = interrupt,
 *					 10 = control, 11 = bulk)
 *
 * Why? Because it's arbitrary, and whatever encoding we select is really
 * up to us. This one happens to share a lot of bit positions with the UHCI
 * specification, so that much of the uhci driver can just mask the bits
 * appropriately.
 */
/* Create various pipes... */
#define create_pipe(dev,endpoint) \
		(((dev)->devnum << 8) | ((endpoint) << 15) | \
		(dev)->maxpacketsize0)
#define default_pipe(dev) ((dev)->speed << 26)

#define usb_sndctrlpipe(dev, endpoint)	((PIPE_CONTROL << 30) | \
					 create_pipe(dev, endpoint))
#define usb_rcvctrlpipe(dev, endpoint)	((PIPE_CONTROL << 30) | \
					 create_pipe(dev, endpoint) | \
					 USB_DIR_IN)
#define usb_sndisocpipe(dev, endpoint)	((PIPE_ISOCHRONOUS << 30) | \
					 create_pipe(dev, endpoint))
#define usb_rcvisocpipe(dev, endpoint)	((PIPE_ISOCHRONOUS << 30) | \
					 create_pipe(dev, endpoint) | \
					 USB_DIR_IN)
#define usb_sndbulkpipe(dev, endpoint)	((PIPE_BULK << 30) | \
					 create_pipe(dev, endpoint))
#define usb_rcvbulkpipe(dev, endpoint)	((PIPE_BULK << 30) | \
					 create_pipe(dev, endpoint) | \
					 USB_DIR_IN)
#define usb_sndintpipe(dev, endpoint)	((PIPE_INTERRUPT << 30) | \
					 create_pipe(dev, endpoint))
#define usb_rcvintpipe(dev, endpoint)	((PIPE_INTERRUPT << 30) | \
					 create_pipe(dev, endpoint) | \
					 USB_DIR_IN)
#define usb_snddefctrl(dev)		((PIPE_CONTROL << 30) | \
					 default_pipe(dev))
#define usb_rcvdefctrl(dev)		((PIPE_CONTROL << 30) | \
					 default_pipe(dev) | \
					 USB_DIR_IN)

/* The D0/D1 toggle bits */
#define usb_gettoggle(dev, ep, out) (((dev)->toggle[out] >> ep) & 1)
#define usb_dotoggle(dev, ep, out)  ((dev)->toggle[out] ^= (1 << ep))
#define usb_settoggle(dev, ep, out, bit) ((dev)->toggle[out] = \
						((dev)->toggle[out] & \
						 ~(1 << ep)) | ((bit) << ep))

/* Endpoint halt control/status */
#define usb_endpoint_out(ep_dir)	(((ep_dir >> 7) & 1) ^ 1)
#define usb_endpoint_halt(dev, ep, out) ((dev)->halted[out] |= (1 << (ep)))
#define usb_endpoint_running(dev, ep, out) ((dev)->halted[out] &= ~(1 << (ep)))
#define usb_endpoint_halted(dev, ep, out) ((dev)->halted[out] & (1 << (ep)))

#define usb_packetid(pipe)	(((pipe) & USB_DIR_IN) ? USB_PID_IN : \
				 USB_PID_OUT)

#define usb_pipeout(pipe)	((((pipe) >> 7) & 1) ^ 1)
#define usb_pipein(pipe)	(((pipe) >> 7) & 1)
#define usb_pipedevice(pipe)	(((pipe) >> 8) & 0x7f)
#define usb_pipe_endpdev(pipe)	(((pipe) >> 8) & 0x7ff)
#define usb_pipeendpoint(pipe)	(((pipe) >> 15) & 0xf)
#define usb_pipedata(pipe)	(((pipe) >> 19) & 1)
#define usb_pipetype(pipe)	(((pipe) >> 30) & 3)
#define usb_pipeisoc(pipe)	(usb_pipetype((pipe)) == PIPE_ISOCHRONOUS)
#define usb_pipeint(pipe)	(usb_pipetype((pipe)) == PIPE_INTERRUPT)
#define usb_pipecontrol(pipe)	(usb_pipetype((pipe)) == PIPE_CONTROL)
#define usb_pipebulk(pipe)	(usb_pipetype((pipe)) == PIPE_BULK)

#define usb_pipe_ep_index(pipe)	\
		usb_pipecontrol(pipe) ? (usb_pipeendpoint(pipe) * 2) : \
				((usb_pipeendpoint(pipe) * 2) - \
				 (usb_pipein(pipe) ? 0 : 1))


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
