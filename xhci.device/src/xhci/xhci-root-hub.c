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

#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#include <clib/timer_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME (*(struct ExecBase **)4UL)
#include <proto/exec.h>
#include <proto/timer.h>
#endif

#include <devices/timer.h>
#include <exec/errors.h>

#include <debug.h>
#include <drv_timer.h>
#include <memory.h>
#include <xhci/ch9.h>
#include <xhci/usb_defs.h>
#include <xhci/xhci.h>
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-udev.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci_root_hub] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef TRACE
#undef KprintfT
#define KprintfT(fmt, ...) PrintPistorm("[xhci_root_hub] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#define STATUS_CHANGE_BITMAP_LENGTH 2 /* in bytes; supports up to 15 ports */

#define USB_MAXCHILDREN 8 /* sizes the USB2 hub-descriptor bitmaps */

/* USB Hub class device protocols (bDeviceProtocol values used by the
 * emulated root hubs) */
#define USB_HUB_PR_HS_SINGLE_TT 1 /* Hi-speed hub with single TT */
#define USB_HUB_PR_SS 3           /* Super speed hub */

#pragma pack(2)
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
#pragma pack()

static const struct descriptor
{
	struct usb_hub_descriptor hub_30;
	struct usb_hub_descriptor hub_20;
	struct usb_device_descriptor device_30;
	struct usb_device_descriptor device_20;
	struct usb_config_descriptor config;
	struct usb_interface_descriptor interface;
	struct usb_endpoint_descriptor endpoint;
	struct usb_ss_ep_comp_descriptor ss_ep_comp;
	struct usb_bos_descriptor bos;
	struct usb_2_0_extension_capability_descriptor ext_cap;
	struct usb_ss_device_capability_descriptor ss_dev_cap;
	struct usb_container_id_capability_descriptor container_id_cap;
} __attribute__((packed)) prototype_descriptor = {
	.hub_30 = {
		.bLength = 12,
		.bDescriptorType = USB_DT_SS_HUB, /* hub descriptor */
		.bNbrPorts = 2,					  /* patched to real port count during init */
		.wHubCharacteristics = le16(HUB_CHAR_INDV_PORT_LPSM |
									HUB_CHAR_INDV_PORT_OCPM), /* per-port power + OC */
		.bPwrOn2PwrGood = 10,								  /* 20 ms between power on and usable */
		.bHubContrCurrent = 0,								  /* self-powered: no bus draw */
		.u.ss = {
			.bHubHdrDecLat = 0,	  /* no hub delay */
			.wHubDelay = le16(0), /* no hub delay */
			.DeviceRemovable = 0, /* all ports permanently wired */
		},
	},
	.hub_20 = {
		.bLength = 9,
		.bDescriptorType = USB_DT_HUB,													/* hub descriptor */
		.bNbrPorts = 2,																	/* patched to real port count during init */
		.wHubCharacteristics = le16(HUB_CHAR_INDV_PORT_LPSM | HUB_CHAR_INDV_PORT_OCPM), /* per-port power + OC */
		.bPwrOn2PwrGood = 10,															/* 20 ms between power on and usable */
		.bHubContrCurrent = 0,															/* self-powered: no bus draw */
		.u.hs = {
			.DeviceRemovable = {0},		 /* all ports permanently wired */
			.PortPowerCtrlMask = {0xff}, /* all ports always have power */
		},
	},
	.device_30 = {
		.bLength = sizeof(struct usb_device_descriptor), /* size of device descriptor */
		.bDescriptorType = USB_DT_DEVICE,				 /* device descriptor */
		.bcdUSB = le16(0x0310),							 /* advertise as USB 3.2 */
		.bDeviceClass = USB_CLASS_HUB,					 /* hub */
		.bDeviceSubClass = 0,							 /* no subclass */
		.bDeviceProtocol = USB_HUB_PR_SS,				 /* super-speed hub */
		.bMaxPacketSize0 = 9,							 /* control endpoint max packet */
		.idVendor = 0x0000,								 /* virtual root hub: leave VID zero */
		.idProduct = 0x0000,							 /* virtual root hub: leave PID zero */
		.bcdDevice = le16(0x0200),						 /* device revision */
		.iManufacturer = 1,								 /* string index */
		.iProduct = 2,									 /* string index */
		.iSerialNumber = 0,								 /* no serial */
		.bNumConfigurations = 1,						 /* single configuration */
	},
	.device_20 = {
		.bLength = sizeof(struct usb_device_descriptor), /* size of device descriptor */
		.bDescriptorType = USB_DT_DEVICE,				 /* device descriptor */
		.bcdUSB = le16(0x0200),							 /* advertise as USB 2.0 */
		.bDeviceClass = USB_CLASS_HUB,					 /* hub */
		.bDeviceSubClass = 0,
		.bDeviceProtocol = USB_HUB_PR_HS_SINGLE_TT, /* high-speed hub */
		.bMaxPacketSize0 = 64,						/* control endpoint max packet */
		.idVendor = 0x0000,							/* virtual root hub: leave VID zero */
		.idProduct = 0x0000,						/* virtual root hub: leave PID zero */
		.bcdDevice = le16(0x0200),					/* device revision */
		.iManufacturer = 1,							/* string index */
		.iProduct = 2,								/* string index */
		.iSerialNumber = 0,							/* no serial */
		.bNumConfigurations = 1,					/* single configuration */
	},
	.config = {
		.bLength = sizeof(struct usb_config_descriptor),																																		  /* size of configuration descriptor */
		.bDescriptorType = USB_DT_CONFIG,																																						  /* configuration descriptor */
		.wTotalLength = le16(sizeof(struct usb_config_descriptor) + sizeof(struct usb_interface_descriptor) + sizeof(struct usb_endpoint_descriptor) + sizeof(struct usb_ss_ep_comp_descriptor)), /* config + interface + endpoint + ss companion */
		.bNumInterfaces = 1,																																									  /* single interface */
		.bConfigurationValue = 1,																																								  /* configuration ID */
		.iConfiguration = 0,																																									  /* no string descriptor */
		.bmAttributes = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,																															  /* must-set + self-powered */
		.bMaxPower = 0,																																											  /* no bus power drawn */
	},
	.interface = {
		.bLength = sizeof(struct usb_interface_descriptor), /* size of interface descriptor */
		.bDescriptorType = USB_DT_INTERFACE,				/* interface descriptor */
		.bInterfaceNumber = 0,								/* interface index */
		.bAlternateSetting = 0,								/* only one setting */
		.bNumEndpoints = 1,									/* interrupt endpoint only */
		.bInterfaceClass = USB_CLASS_HUB,					/* hub functional interface */
		.bInterfaceSubClass = 0,							/* full/high-speed hub */
		.bInterfaceProtocol = 0,
		.iInterface = 0, /* no string */
	},
	.endpoint = {
		.bLength = sizeof(struct usb_endpoint_descriptor),						/* size of endpoint descriptor */
		.bDescriptorType = USB_DT_ENDPOINT,										/* endpoint descriptor */
		.bEndpointAddress = (USB_DIR_IN | 1),									/* INT IN endpoint 1 */
		.bmAttributes = USB_ENDPOINT_XFER_INT | USB_ENDPOINT_INTR_NOTIFICATION, /* interrupt */
		.wMaxPacketSize = le16(STATUS_CHANGE_BITMAP_LENGTH),
		.bInterval = 8,
	},
	.ss_ep_comp = {
		.bLength = sizeof(struct usb_ss_ep_comp_descriptor), /* size of SS endpoint companion descriptor */
		.bDescriptorType = USB_DT_SS_ENDPOINT_COMP,			 /* SS endpoint companion descriptor */
		.bMaxBurst = 0,										 /* no bursting */
		.bmAttributes = 0,									 /* no streams */
		.wBytesPerInterval = le16(STATUS_CHANGE_BITMAP_LENGTH),
	},
	.bos = {
		.bLength = sizeof(struct usb_bos_descriptor),																																												   /* size of BOS descriptor */
		.bDescriptorType = USB_DT_BOS,																																																   /* BOS descriptor */
		.wTotalLength = le16(sizeof(struct usb_bos_descriptor) + sizeof(struct usb_2_0_extension_capability_descriptor) + sizeof(struct usb_ss_device_capability_descriptor) + sizeof(struct usb_container_id_capability_descriptor)), /* total length of all BOS descriptors */
		.bNumDeviceCaps = 3,																																																		   /* number of device capability descriptors */
	},
	.ext_cap = {
		.bLength = sizeof(struct usb_2_0_extension_capability_descriptor), /* size of USB 2.0 extension descriptor */
		.bDescriptorType = USB_DT_DEVICE_CAPABILITY,					   /* device capability descriptor */
		.bDevCapabilityType = USB_CAP_DESC_USB20_EXTENSION,				   /* USB 2.0 extension capability */
		.bmAttributes = le32(0),
	},
	.ss_dev_cap = {
		.bLength = sizeof(struct usb_ss_device_capability_descriptor), /* size of SuperSpeed USB device capability descriptor */
		.bDescriptorType = USB_DT_DEVICE_CAPABILITY,				   /* device capability descriptor */
		.bDevCapabilityType = USB_CAP_DESC_SS_USB_DEVICE,			   /* SuperSpeed USB device capability */
		.bmAttributes = 0,
		.wSpeedsSupported = le16(USB_SS_DEVICE_LOWSPEED_SUPPORT | USB_SS_DEVICE_FULLSPEED_SUPPORT | USB_SS_DEVICE_HIGHSPEED_SUPPORT | USB_SS_DEVICE_SUPERSPEED_SUPPORT), /* supports all speeds */
		.bFunctionalitySupport = 1,																																		 /* lowest speed is 1 (low speed) */
		.bU1DevExitLat = 0,																																				 /* no U1 exit latency */
		.wU2DevExitLat = le16(0),																																		 /* no U2 exit latency */
	},
	.container_id_cap = {
		.bLength = sizeof(struct usb_container_id_capability_descriptor), /* size of Container ID capability descriptor */
		.bDescriptorType = USB_DT_DEVICE_CAPABILITY,					  /* device capability descriptor */
		.bDevCapabilityType = USB_CAP_DESC_CONTAINER_ID,				  /* Container ID capability */
		.bReserved = 0,													  /* reserved: must be zero */
		.ContainerID = {0},												  /* fill in at runtime */
	},
};

struct xhci_root_hub_port
{
	u8 major_revision;
	u8 minor_revision;

	u8 slot_type;
	u8 max_hub_depth;
	BOOL usb3_lsecc;		  /* Link Soft Error Count Capability */
	BOOL usb2_integrated_hub; /* Integrated Hub Implemented */
	BOOL usb2_hs_only;		  /* High-Speed Only Capability */
	BOOL usb2_hw_lpm;		  /* Hardware LPM Capability */
	BOOL usb2_besl_lpm;		  /* BESL LPM Capability */
};

struct xhci_root_hub_view
{
	struct xhci_root_hub *rh;
	u8 format;                 /* RH_VIEW_* */
	u8 num_ports;
	u8 *port_map;              /* local idx (0-based) -> global 1-based port; NULL = identity */
	struct xhci_xfer *int_req; /* pending status-change interrupt request */
};

struct xhci_root_hub
{
	struct usb_device *udev;

	struct descriptor descriptor;

	struct xhci_root_hub_port *ports;
	u8 num_ports;
	BOOL is_super_speed;
	BOOL remote_wakeup; /* DEVICE_REMOTE_WAKEUP feature state (bookkeeping only -
						 * the xHC manages root-port wake hardware itself) */

	/* the two protocol-pure presentations of the port register file
	 * (see xhci-root-hub.h) */
	struct xhci_root_hub_view view_ss;
	struct xhci_root_hub_view view_usb2;
};

#ifdef TRACE
static void xhci_roothub_debug_port(struct xhci_root_hub *rh, u32 port)
{
	struct xhci_ctrl *ctrl = rh->udev->controller;
	u32 portsc = mmio_read32(&ctrl->hcor->portregs[port].or_portsc);
	KprintfT("port %lu status: 0x%08lx\n", (ULONG)port + 1, (ULONG)portsc);

	if (portsc & PORT_CONNECT) // ROS
		KprintfT("  device connected\n");
	else
		KprintfT("  no device\n");

	if (portsc & PORT_PE) // RW1CS don't disable 3.0 ports
		KprintfT("  port enabled\n");
	else
		KprintfT("  port disabled\n");

	if (portsc & PORT_OC) // RO
		KprintfT("  over-current condition\n");

	if (portsc & PORT_RESET) // RW1S
		KprintfT("  port in reset\n");
	else
		KprintfT("  port not in reset\n");

	u32 pls = portsc & PORT_PLS_MASK; // RWS
	switch (pls)
	{
	case XDEV_U0:
		KprintfT("  link state: U0 (active)\n");
		break;
	case XDEV_U1:
		KprintfT("  link state: U1\n");
		break;
	case XDEV_U2:
		KprintfT("  link state: U2\n");
		break;
	case XDEV_U3:
		KprintfT("  link state: U3 (suspended)\n");
		break;
	case XDEV_DISABLED:
		KprintfT("  link state: Disabled\n");
		break;
	case XDEV_RXDETECT:
		KprintfT("  link state: RxDetect\n");
		break;
	case XDEV_INACTIVE:
		KprintfT("  link state: Inactive\n");
		break;
	case XDEV_POLLING:
		KprintfT("  link state: Polling\n");
		break;
	case XDEV_RECOVERY:
		KprintfT("  link state: Recovery\n");
		break;
	case XDEV_HOTRESET:
		KprintfT("  link state: Hot Reset\n");
		break;
	case XDEV_COMPLIANCE:
		KprintfT("  link state: Compliance Mode\n");
		break;
	case XDEV_TESTMODE:
		KprintfT("  link state: Test Mode\n");
		break;
	case XDEV_RESUME:
		KprintfT("  link state: Resume\n");
		break;
	default:
		KprintfT("  link state: unknown (%lu)\n", (ULONG)(pls >> 5));
		break;
	}

	if (portsc & PORT_POWER) // RWS
		KprintfT("  port has power\n");
	else
		KprintfT("  port has no power\n");

	u32 speed = (portsc & DEV_SPEED_MASK); // ROS
	switch (speed)
	{
	case XDEV_FS:
		KprintfT("  device speed: Full Speed\n");
		break;
	case XDEV_LS:
		KprintfT("  device speed: Low Speed\n");
		break;
	case XDEV_HS:
		KprintfT("  device speed: High Speed\n");
		break;
	case XDEV_SS:
		KprintfT("  device speed: Super Speed\n");
		break;
	default:
		KprintfT("  device speed: unknown (%lu)\n", (ULONG)(speed >> 10));
		break;
	}

	if (portsc & PORT_CSC) // RW1CS connect or disconnect sets this to 1
		KprintfT("  connect status change\n");
	if (portsc & PORT_PEC) // RW1CS set to 1 on transition to disabled from enabled USB 2.0 only
		KprintfT("  port enable change\n");
	if (portsc & PORT_WRC) // RW1CS 1 when warm reset completes (WPR 1->0)
		KprintfT("  warm reset change\n");
	if (portsc & PORT_OCC) // RW1CS when OC transitions to 1
		KprintfT("  over-current change\n");
	if (portsc & PORT_RC) // RW1CS 1 when reset completes (PR 1->0 or WR 1->0)
		KprintfT("  reset change\n");
	if (portsc & PORT_PLC) // RW1CS when PLS changes - 4.19.1
		KprintfT("  port link state change\n");
	if (portsc & PORT_CEC) // RW1CS
		KprintfT("  port config error change\n");

	if (portsc & PORT_WR) // RW1S
		KprintfT("  warm reset in progress\n");
}
#endif

struct xhci_root_hub *xhci_roothub_create(struct usb_device *udev)
{
	struct xhci_ctrl *ctrl = udev->controller;
	struct xhci_root_hub *rh = pool_zalloc(ctrl->metaPool, sizeof(struct xhci_root_hub));
	if (!rh)
		return NULL;

	rh->udev = udev;

	CopyMem(&prototype_descriptor, &rh->descriptor, sizeof(prototype_descriptor));

	const u8 ports = HCS_MAX_PORTS(mmio_read32(&ctrl->hccr->cr_hcsparams1));
	rh->num_ports = ports;
	rh->descriptor.hub_30.bNbrPorts = ports;
	rh->descriptor.hub_20.bNbrPorts = ports;

	Kprintf("Initializing root hub with %lu ports\n", (ULONG)ports);

	/* Port Indicators */
	const u32 hccParams1 = mmio_read32(&ctrl->hccr->cr_hccparams1);
	u16 wHubCharacteristics = le16(rh->descriptor.hub_30.wHubCharacteristics);
	if (HCS_INDICATOR(hccParams1))
	{
		wHubCharacteristics |= HUB_CHAR_PORTIND;
		Kprintf("Host controller supports per-port indicators\n");
	}

	/* Port Power Control */
	if (HCC_PPC(hccParams1))
	{
		wHubCharacteristics |= HUB_CHAR_INDV_PORT_LPSM;
		Kprintf("Host controller supports per-port power control\n");
	}

	/* Advertise LTM when the controller consumes it (devices are enabled by
	 * the stack via SET_FEATURE(LTM_ENABLE)).  The Set Latency Tolerance Value
	 * *command* is the reverse direction - host-platform latency reporting -
	 * which neither we nor Linux ever issue. */
	if (HCC_LTC(hccParams1))
		rh->descriptor.ss_dev_cap.bmAttributes |= USB_SS_DEVICE_ATT_LATENCY_TOLERANCE_MESSAGES;

	rh->descriptor.hub_30.wHubCharacteristics = le16(wHubCharacteristics);
	rh->descriptor.hub_20.wHubCharacteristics = le16(wHubCharacteristics);

	/* Exit latencies */
	const u32 hcsParams3 = mmio_read32(&ctrl->hccr->cr_hcsparams3);
	rh->descriptor.ss_dev_cap.bU1DevExitLat = HCS_U1_LATENCY(hcsParams3);
	rh->descriptor.ss_dev_cap.wU2DevExitLat = le16(HCS_U2_LATENCY(hcsParams3));
	Kprintf("Host controller U1 exit latency: %lu microseconds\n", (ULONG)rh->descriptor.ss_dev_cap.bU1DevExitLat);
	Kprintf("Host controller U2 exit latency: %lu microseconds\n", (ULONG)le16(rh->descriptor.ss_dev_cap.wU2DevExitLat));

	/* Create port structs and fill with data from protool extended capability */
	rh->ports = pool_zalloc(ctrl->metaPool, ports * sizeof(struct xhci_root_hub_port));
	if (!rh->ports)
	{
		pool_free(ctrl->metaPool, rh);
		Kprintf("Failed to allocate root hub port data\n");
		return NULL;
	}

	u32 next_offset = 0;
	u32 *cap_base;
	rh->is_super_speed = FALSE;

	while ((cap_base = xhci_find_next_capability(ctrl, XHCI_EXT_CAPS_PROTOCOL, &next_offset)) != NULL)
	{
		struct xhci_protocol_caps caps = xhci_get_protocol_caps(cap_base);
		for (u32 port_index = (u32)caps.port_offset - 1u; port_index < (u32)(caps.port_offset - 1 + caps.port_count) && port_index < ports; ++port_index)
		{
			rh->ports[port_index].major_revision = caps.major_revision;
			rh->ports[port_index].minor_revision = caps.minor_revision;
			rh->ports[port_index].slot_type = caps.protocol_slot_type;
			rh->ports[port_index].max_hub_depth = caps.max_hub_depth;
			rh->ports[port_index].usb3_lsecc = caps.usb3_lsecc;
			rh->ports[port_index].usb2_integrated_hub = caps.usb2_integrated_hub;
			rh->ports[port_index].usb2_hs_only = caps.usb2_hs_only;
			rh->ports[port_index].usb2_hw_lpm = caps.usb2_hw_lpm;
			rh->ports[port_index].usb2_besl_lpm = caps.usb2_besl_lpm;
			Kprintf("Port %lu: USB %lu.%lu, slot type %lu, max hub depth %lu\n",
					(ULONG)port_index + 1,
					(ULONG)caps.major_revision, (ULONG)caps.minor_revision,
					(ULONG)caps.protocol_slot_type, (ULONG)caps.max_hub_depth);
			if (caps.major_revision >= 3)
			{
				rh->is_super_speed = TRUE;
				if (caps.usb3_lsecc)
					Kprintf("  USB 3.0 Link Soft Error Count Capability\n");
			}
			else
			{
				if (caps.usb2_hs_only)
					Kprintf("  Supports only USB 2.0 High-Speed devices\n");
				if (caps.usb2_integrated_hub)
					Kprintf("  USB 2.0 Integrated Hub\n");
				if (caps.usb2_hw_lpm)
					Kprintf("  USB 2.0 Hardware LPM Capability\n");
				if (caps.usb2_besl_lpm)
					Kprintf("  USB 2.0 BESL LPM Capability\n");
			}
		}
	}

	rh->udev->speed = rh->is_super_speed ? USB_SPEED_SUPER : USB_SPEED_HIGH;

	/* Build the protocol views: each maps its protocol's port subset to
	 * controller-global port numbers. */
	rh->view_ss.rh = rh;
	rh->view_ss.format = RH_VIEW_SS;
	rh->view_usb2.rh = rh;
	rh->view_usb2.format = RH_VIEW_USB2;
	if (ports)
	{
		u8 *maps = pool_zalloc(ctrl->metaPool, 2u * ports);
		if (!maps)
		{
			pool_free(ctrl->metaPool, rh->ports);
			pool_free(ctrl->metaPool, rh);
			Kprintf("Failed to allocate root hub view maps\n");
			return NULL;
		}
		rh->view_ss.port_map = maps;
		rh->view_usb2.port_map = maps + ports;
		for (u8 i = 0; i < ports; ++i)
		{
			if (rh->ports[i].major_revision >= 3)
				rh->view_ss.port_map[rh->view_ss.num_ports++] = (u8)(i + 1);
			else
				rh->view_usb2.port_map[rh->view_usb2.num_ports++] = (u8)(i + 1);
		}
	}
	Kprintf("Root hub views: %lu USB3-protocol + %lu USB2-protocol ports\n",
			(ULONG)rh->view_ss.num_ports, (ULONG)rh->view_usb2.num_ports);

#ifdef TRACE
	for (u32 i = 0; i < ports; ++i)
		xhci_roothub_debug_port(rh, i);
#endif

	return rh;
}

void xhci_roothub_destroy(struct xhci_root_hub *rh)
{
	if (!rh)
		return;

	xhci_roothub_abort_int_request(rh);

	if (rh->view_ss.port_map)
		pool_free(rh->udev->controller->metaPool, rh->view_ss.port_map); /* one block for both views */
	if (rh->ports)
		pool_free(rh->udev->controller->metaPool, rh->ports);

	pool_free(rh->udev->controller->metaPool, rh);
}

struct usb_device *xhci_roothub_udev(struct xhci_root_hub *rh)
{
	return rh ? rh->udev : NULL;
}

/* Report the USB2 hardware-LPM (HLC) and BESL-LPM (BLC) capability of a 1-based
 * root-hub port. */
void xhci_roothub_port_lpm_caps(struct xhci_root_hub *rh, u32 port, BOOL *hw_lpm, BOOL *besl_lpm)
{
	if (hw_lpm)
		*hw_lpm = FALSE;
	if (besl_lpm)
		*besl_lpm = FALSE;
	if (!rh || !rh->ports || port == 0 || port > rh->num_ports)
		return;

	if (hw_lpm)
		*hw_lpm = rh->ports[port - 1].usb2_hw_lpm;
	if (besl_lpm)
		*besl_lpm = rh->ports[port - 1].usb2_besl_lpm;
}

/* Program the U1 or U2 inactivity timeout in a 1-based root-hub port's PORTPMSC
 * (driven by the view's SetPortFeature(U1/U2_TIMEOUT) handlers). */
static void xhci_roothub_set_usb3_port_timeout(struct xhci_root_hub *rh, u32 port, BOOL u2, u16 timeout)
{
	if (!rh || port == 0 || port > rh->num_ports)
		return;

	KprintfT("Setting USB3 port %lu timeout: U%lu timeout=%lu %s\n", (ULONG)port, (ULONG)(u2 ? 2 : 1),
			 (ULONG)timeout, u2 ? "x256 microseconds" : "microseconds");

	volatile u32 *pmsc = &rh->udev->controller->hcor->portregs[port - 1].or_portpmsc;
	u32 reg = mmio_read32(pmsc);
	if (u2)
	{
		reg &= ~(u32)PORT_U2_TIMEOUT(0xff);
		reg |= (u32)PORT_U2_TIMEOUT(timeout);
	}
	else
	{
		reg &= ~(u32)PORT_U1_TIMEOUT(0xff);
		reg |= (u32)PORT_U1_TIMEOUT(timeout);
	}
	mmio_write32(reg, pmsc);
	mmio_read32(pmsc); /* flush */
}

/* Program USB2 hardware LPM (L1) on a 1-based root-hub port.  In BESL mode also
 * writes PORTHLPMC (deep BESL + L1 timeout in 256us units + HIRDM=BESL). */
void xhci_roothub_set_usb2_hw_lpm(struct xhci_root_hub *rh, u32 port, u8 hird, u8 slot_id,
								  BOOL besl_mode, u8 besld, u16 l1_timeout_us)
{
	if (!rh || port == 0 || port > rh->num_ports)
		return;

	struct xhci_hcor_port_regs *pr = &rh->udev->controller->hcor->portregs[port - 1];

	KprintfT("Setting USB2 hardware LPM on port %lu: HIRD=%u, slot ID=%u, BESL mode=%u, BESLD=%u, L1 timeout=%u microseconds\n",
			 (ULONG)port, (ULONG)hird, (ULONG)slot_id, (ULONG)besl_mode, (ULONG)besld, (ULONG)l1_timeout_us);

	if (besl_mode)
	{
		u32 hv = (u32)PORT_BESLD(besld) | (u32)PORT_L1_TIMEOUT(l1_timeout_us / 256u) | (u32)PORT_HIRDM(1);
		mmio_write32(hv, &pr->or_porthlpmc);
		mmio_read32(&pr->or_porthlpmc); /* flush */
	}

	u32 pm = mmio_read32(&pr->or_portpmsc);
	pm &= ~((u32)PORT_HIRD_MASK | (u32)PORT_L1DS_MASK);
	pm |= (u32)PORT_HIRD(hird) | (u32)PORT_RWE | (u32)PORT_L1DS(slot_id);
	mmio_write32(pm, &pr->or_portpmsc);
	pm = mmio_read32(&pr->or_portpmsc);
	pm |= (u32)PORT_HLE;
	mmio_write32(pm, &pr->or_portpmsc);
	mmio_read32(&pr->or_portpmsc); /* flush */
}

/* Disable USB2 hardware LPM (L1) on a 1-based root-hub port, clearing HLE, RWE,
 * HIRD and the stale L1 device slot.  Mirrors xhci_set_usb2_hardware_lpm(enable=0);
 * used on device teardown so PORTPMSC.L1DS no longer points at a freed slot. */
void xhci_roothub_clear_usb2_hw_lpm(struct xhci_root_hub *rh, u32 port)
{
	if (!rh || port == 0 || port > rh->num_ports)
		return;

	struct xhci_hcor_port_regs *pr = &rh->udev->controller->hcor->portregs[port - 1];

	u32 pm = mmio_read32(&pr->or_portpmsc);
	pm &= ~((u32)PORT_HLE | (u32)PORT_RWE | (u32)PORT_HIRD_MASK | (u32)PORT_L1DS_MASK);
	mmio_write32(pm, &pr->or_portpmsc);
	mmio_read32(&pr->or_portpmsc); /* flush */
}

/* ---- PORTSC change bit <-> hub protocol mapping --------------------------
 * One row per RW1C change bit.  usb2_c/ss_c: the wPortChange bit each
 * protocol view reports (0 = not reported there).  clear_usb2/clear_ss: the
 * ClearPortFeature selector that clears the bit from that view (0 = none).
 * Selector values are globally distinct, so the clear-feature lookup needs no
 * view parameter.  Asymmetries that don't fit a row stay in code: USB2
 * C_SUSPEND is *synthesized* from PLC && PLS==U0 in get-status (the PLC row
 * reports nothing on USB2), and the C_RESET companion WRC clear for rev3
 * ports lives in the clear-feature handler. */
struct rh_portsc_change
{
	u32 portsc_bit;
	u16 usb2_c;      /* wPortChange bit, USB2 view */
	u16 ss_c;        /* wPortChange bit, SS view */
	u16 clear_usb2;  /* ClearPortFeature selector, USB2 view */
	u16 clear_ss;    /* ClearPortFeature selector, SS view */
};

static const struct rh_portsc_change rh_changes[] = {
	{PORT_CSC, USB_PORT_STAT_C_CONNECTION, USB_PORT_STAT_C_CONNECTION, USB_PORT_FEAT_C_CONNECTION, USB_PORT_FEAT_C_CONNECTION},
	{PORT_PEC, USB_PORT_STAT_C_ENABLE, 0, USB_PORT_FEAT_C_ENABLE, 0},
	{PORT_OCC, USB_PORT_STAT_C_OVERCURRENT, USB_PORT_STAT_C_OVERCURRENT, USB_PORT_FEAT_C_OVER_CURRENT, USB_PORT_FEAT_C_OVER_CURRENT},
	{PORT_RC, USB_PORT_STAT_C_RESET, USB_PORT_STAT_C_RESET, USB_PORT_FEAT_C_RESET, USB_PORT_FEAT_C_RESET},
	{PORT_WRC, 0, USB_SS_PORT_STAT_C_BH_RESET, 0, USB_SS_PORT_FEAT_C_BH_RESET},
	{PORT_PLC, 0 /* C_SUSPEND is synthesized */, USB_SS_PORT_STAT_C_LINK_STATE, USB_PORT_FEAT_C_SUSPEND, USB_SS_PORT_FEAT_C_LINK_STATE},
	{PORT_CEC, 0, USB_SS_PORT_STAT_C_CONFIG_ERROR, 0, USB_SS_PORT_FEAT_C_CONFIG_ERROR},
};

#define RH_NUM_CHANGES (sizeof(rh_changes) / sizeof(rh_changes[0]))

/* Which PORTSC change bits raise the view's status-change interrupt: exactly
 * the bits the view can clear (i.e. rows with a ClearPortFeature selector). */
static u32 rh_view_change_mask(u8 format)
{
	u32 mask = 0;
	for (u32 i = 0; i < RH_NUM_CHANGES; ++i)
		if ((format == RH_VIEW_SS) ? rh_changes[i].clear_ss : rh_changes[i].clear_usb2)
			mask |= rh_changes[i].portsc_bit;
	return mask;
}

/* wPortChange bits for a view from a PORTSC sample (the USB2 C_SUSPEND
 * synthesis is the caller's, see the table comment). */
static u16 rh_view_port_change(u8 format, u32 reg)
{
	u16 change = 0;
	for (u32 i = 0; i < RH_NUM_CHANGES; ++i)
		if (reg & rh_changes[i].portsc_bit)
			change = (u16)(change | ((format == RH_VIEW_SS) ? rh_changes[i].ss_c : rh_changes[i].usb2_c));
	return change;
}

static void xhci_roothub_view_complete_int_request(struct xhci_root_hub_view *v);

s8 xhci_roothub_view_submit_int_request(struct xhci_root_hub_view *v, struct xhci_xfer *req)
{
	if (v->int_req)
	{
		Kprintf("root hub interrupt request already pending\n");
		return UHIOERR_HOSTERROR;
	}

	v->int_req = req;
	xhci_roothub_view_complete_int_request(v);
	return UHIOERR_NO_ERROR;
}

static void xhci_roothub_view_complete_int_request(struct xhci_root_hub_view *v)
{
	struct xhci_root_hub *rh = v->rh;

	if (!v->int_req)
		return;

	struct xhci_ctrl *ctrl = rh->udev->controller;

	u8 *buffer = (u8 *)v->int_req->data;

	const u8 num_ports = v->num_ports;
	/* USB 3.0 spec is always two bytes, however the stack may request fewer bytes */
	const u8 need_bytes = (u8)(((u32)num_ports + 7U) / 8U);
	if (!buffer || v->int_req->data_length < need_bytes)
	{
		xhci_xfer_complete(rh->udev, v->int_req, UHIOERR_STALL, 0);
		v->int_req = NULL;
		return;
	}

	buffer[0] = 0; /* port 1-7 status change bitmap */
	if (need_bytes > 1)
		buffer[1] = 0; /* port 8-15 status change bitmap */

	/* which change bits raise the interrupt, per protocol view */
	const u32 change_mask = rh_view_change_mask(v->format);

	BOOL change = FALSE;
	for (u8 i = 0; i < num_ports; ++i)
	{
		const u8 global = v->port_map ? v->port_map[i] : (u8)(i + 1);
		u32 portsc = mmio_read32(&ctrl->hcor->portregs[global - 1].or_portsc);
		u32 change_bits = portsc & change_mask;
		if (!change_bits)
			continue;

		u32 index = (i + 1U) >> 3;
		if (index >= need_bytes)
			continue;

		buffer[index] |= (u8)(1U << (((u32)i + 1U) & 7U));
		KprintfT("port %lu (global %lu) status change detected: changebits=0x%08lx\n",
				 (ULONG)(i + 1), (ULONG)global, (ULONG)change_bits);
		change = TRUE;
	}

	if (!change)
	{
		KprintfT("no status change, not completing root hub interrupt\n");
		return;
	}

	xhci_xfer_complete(rh->udev, v->int_req, UHIOERR_NO_ERROR, need_bytes);
	v->int_req = NULL;
}

void xhci_roothub_complete_int_request(struct xhci_root_hub *rh)
{
	if (!rh)
		return;

	xhci_roothub_view_complete_int_request(&rh->view_ss);
	xhci_roothub_view_complete_int_request(&rh->view_usb2);
}

static void xhci_roothub_view_abort_int_request(struct xhci_root_hub_view *v)
{
	struct xhci_root_hub *rh = v->rh;

	if (!v->int_req)
		return;

	xhci_xfer_complete(rh->udev, v->int_req, IOERR_ABORTED, 0);
	v->int_req = NULL;
}

void xhci_roothub_abort_int_request(struct xhci_root_hub *rh)
{
	if (!rh)
		return;

	xhci_roothub_view_abort_int_request(&rh->view_ss);
	xhci_roothub_view_abort_int_request(&rh->view_usb2);
}

void xhci_roothub_view_abort_int_cookie(struct xhci_root_hub_view *v, APTR cookie)
{
	if (v->int_req && v->int_req->cookie == cookie)
		xhci_roothub_view_abort_int_request(v);
}

inline static void xhci_roothub_stall(struct xhci_xfer *req)
{
	if (req)
	{
		KprintfT("stall\n");
		req->actual = 0;
		req->error = UHIOERR_STALL;
	}
}

inline static void xhci_roothub_no_error(struct xhci_xfer *req)
{
	if (req)
	{
		req->error = UHIOERR_NO_ERROR;
	}
}

inline static void xhci_roothub_reply(struct xhci_xfer *req, void *data, u32 length)
{
	if (!req)
		return;

	u32 req_length = le16(req->setup.usd_Length);
	length = length < req_length ? length : req_length;

	if (data && length > 0)
		CopyMem(data, req->data, length);

	req->actual = length;
	req->error = UHIOERR_NO_ERROR;
}

/* Sleep on the unit task's persistent sleep timer with the transfer-plane
 * lock RELEASED.  The port handlers run on the unit task holding xfer_lock
 * exactly once, PORTSC is re-read after every sleep, and direct
 * submits/aborts proceeding during the wait is the point — a port reset or
 * resume no longer stalls the whole transfer plane.  (The timer is
 * task-bound, which is why the unit task owns it — see ctrl->sleep_timer;
 * without one the wait degrades to a hot poll.) */
static void rh_sleep_unlocked(struct xhci_ctrl *ctrl, u32 milliseconds)
{
	if (milliseconds == 0 || !ctrl->sleep_timer.req)
		return;

	lock_prof_release(&ctrl->lockProf, &ctrl->xfer_lock);
	drv_timer_sleep_ms(&ctrl->sleep_timer, milliseconds);
	lock_prof_obtain(&ctrl->lockProf, &ctrl->xfer_lock);
}

static void xhci_roothub_handle_device_get_configuration(struct xhci_root_hub *rh, struct xhci_xfer *req)
{
	KprintfT("USB_REQ_GET_CONFIGURATION\n");
	xhci_roothub_reply(req, &rh->descriptor.config.bConfigurationValue, 1);
}

static void xhci_roothub_handle_device_get_descriptor(struct xhci_root_hub_view *v, struct xhci_xfer *req)
{
	struct xhci_root_hub *rh = v->rh;
	struct UhcdSetupData *setup = &req->setup;

	switch (le16(setup->usd_Value) >> 8)
	{
	case USB_DT_BOS:
		KprintfT("get USB_DT_BOS\n");
		xhci_roothub_reply(req, &rh->descriptor.bos, le16(rh->descriptor.bos.wTotalLength));
		break;
	case USB_DT_DEVICE:
	{
		KprintfT("get USB_DT_DEVICE\n");
		BOOL serve_ss = (v->format == RH_VIEW_SS);
		if (serve_ss)
			xhci_roothub_reply(req, &rh->descriptor.device_30, rh->descriptor.device_30.bLength);
		else
			xhci_roothub_reply(req, &rh->descriptor.device_20, rh->descriptor.device_20.bLength);
		break;
	}
	case USB_DT_CONFIG:
		KprintfT("get USB_DT_CONFIG\n");
		xhci_roothub_reply(req, &rh->descriptor.config, le16(rh->descriptor.config.wTotalLength));
		break;
	case USB_DT_STRING:
		KprintfT("get USB_DT_STRING\n");
		switch (le16(setup->usd_Value) & 0xff)
		{
		case 0: /* Language */
			xhci_roothub_reply(req, "\4\3\11\4", 4);
			break;
		case 1: /* Vendor String  */
			xhci_roothub_reply(req, "\20\3P\0i\0s\0t\0o\0r\0m\0", 16);
			break;
		case 2: /* Product Name */
			xhci_roothub_reply(req, "\52\3X\0H\0C\0I\0 \0H\0o\0s\0t\0 \0C\0o\0n\0t\0r\0o\0l\0l\0e\0r\0", 42);
			break;
		default:
			Kprintf("unknown value DT_STRING %lx\n", le16(setup->usd_Value));
			xhci_roothub_stall(req);
		}
		break;
	default:
		Kprintf("get unknown wValue %lx\n", le16(setup->usd_Value));
		xhci_roothub_stall(req);
	}
}

static void xhci_roothub_handle_device_get_status(struct xhci_root_hub *rh, struct xhci_xfer *req)
{
	KprintfT("USB_REQ_GET_STATUS\n");
	/* Device GET_STATUS: bit0=self-powered, bit1=remote-wakeup */
	u8 status[2] = {(u8)(1 | (rh->remote_wakeup ? 2 : 0)), 0};
	xhci_roothub_reply(req, status, 2);
}

/* Direct a 1-based root-hub port to U3.  Deferred tail of the port-suspend
 * sequence (xhci_udev_suspend_finish): the attached device's endpoint rings
 * are already stopped when this runs. */
void xhci_roothub_set_port_u3(struct xhci_root_hub *rh, u8 port)
{
	if (!rh || port == 0 || port > rh->num_ports)
		return;

	xhci_port_set_link_state(rh->udev->controller->hcor, port, XDEV_U3);
	KprintfT("port %lu -> U3 standby (endpoints stopped)\n", (ULONG)port);
}

/**
 * Clears the Change bits of the Port Status Register
 *
 * @param wValue	request value
 * @param wIndex	request index
 * @param addr		address of port status register
 * @param port_status	state of port status register
 * Return: none
 */
inline static void xhci_roothub_clear_port_change_bit(u16 wValue, u8 portNo, volatile u32 *addr, u32 port_status)
{
	u32 status = 0;
	for (u32 i = 0; i < RH_NUM_CHANGES; ++i)
	{
		if ((rh_changes[i].clear_usb2 && rh_changes[i].clear_usb2 == wValue) ||
			(rh_changes[i].clear_ss && rh_changes[i].clear_ss == wValue))
		{
			status = rh_changes[i].portsc_bit;
			break;
		}
	}
	if (!status)
		return; /* Should never happen */

	/* Change bits are all write 1 to clear */
	mmio_write32(port_status | status, addr);

#ifdef TRACE
	port_status = mmio_read32(addr);
	KprintfT("clear port change (feature %lu), actual port %lu status = 0x%lx\n",
			 (ULONG)wValue, (ULONG)portNo, (ULONG)port_status);
#else
	(void)portNo;
#endif
}

inline static struct xhci_hcor_port_regs *xhci_roothub_get_port(struct xhci_root_hub *rh, struct xhci_xfer *req)
{
	struct xhci_ctrl *ctrl = rh->udev->controller;
	u8 port = le16(req->setup.usd_Index) & 0xffU; // port number is in low byte of wIndex;

	if (port == 0 || port > rh->num_ports)
		return NULL;

	return &ctrl->hcor->portregs[port - 1];
}

static void xhci_roothub_handle_port_clear_feature(struct xhci_root_hub *rh, struct xhci_xfer *req)
{
	const u16 wValue = le16(req->setup.usd_Value); // feature selector
	const u16 wIndex = le16(req->setup.usd_Index); // selector | port
	const u8 portNo = wIndex & 0xffU;
#ifdef TRACE
	xhci_roothub_debug_port(rh, portNo - 1u);
#endif

	struct xhci_hcor_port_regs *port = xhci_roothub_get_port(rh, req);
	u32 reg = mmio_read32(&port->or_portsc);
	reg = xhci_port_state_to_neutral(reg);

	switch (wValue)
	{
	// Common for USB2 and USB3 ports
	case USB_PORT_FEAT_POWER:
		KprintfT("Clear port %lu PORT_POWER\n", (ULONG)portNo);
		reg &= ~PORT_POWER;
		mmio_write32(reg, &port->or_portsc);
		break;
	case USB_PORT_FEAT_C_CONNECTION:
	case USB_PORT_FEAT_C_OVER_CURRENT:
		xhci_roothub_clear_port_change_bit(wValue, portNo, &port->or_portsc, reg);
		break;
	case USB_PORT_FEAT_C_RESET:
		xhci_roothub_clear_port_change_bit(wValue, portNo, &port->or_portsc, reg);
		/* For USB 3.0 ports: a warm reset sets both PRC and WRC.
		 * The USB 2.0 stack won't send CLEAR_FEATURE(C_BH_PORT_RESET)
		 * so we clear WRC here alongside PRC. */
		if (rh->ports[portNo - 1].major_revision >= 3)
		{
			u32 tmp = mmio_read32(&port->or_portsc);
			if (tmp & PORT_WRC)
			{
				tmp = xhci_port_state_to_neutral(tmp);
				mmio_write32(tmp | PORT_WRC, &port->or_portsc);
			}
		}
		break;
	case USB_PORT_FEAT_CONNECTION:
	case USB_PORT_FEAT_OVER_CURRENT:
	case USB_PORT_FEAT_RESET:
		/* No-op */
		break;

		// USB3 specific features
	case USB_SS_PORT_FEAT_FORCE_LINKPM_ACCEPT:
		KprintfT("Clear port %lu FORCE_LINKPM_ACCEPT\n", (ULONG)portNo);
		reg = mmio_read32(&port->or_portpmsc);
		reg &= ~PORT_FLA;
		mmio_write32(reg, &port->or_portpmsc);
		break;
	case USB_SS_PORT_FEAT_C_LINK_STATE:
	case USB_SS_PORT_FEAT_C_CONFIG_ERROR:
	case USB_SS_PORT_FEAT_C_BH_RESET:
		xhci_roothub_clear_port_change_bit(wValue, portNo, &port->or_portsc, reg);
		break;
	case USB_PORT_FEAT_LINK_STATE:
		/* No-op */
		break;

	// USB2 specific features
	case USB_PORT_FEAT_ENABLE:
		KprintfT("Clear port %lu PORT_PE\n", (ULONG)portNo);
		if (rh->ports[portNo - 1].major_revision >= 3)
		{
			Kprintf("Can't disable USB 3.0 port\n");
			xhci_roothub_stall(req);
			return;
		}
		else
		{
			reg |= PORT_PE;
			mmio_write32(reg, &port->or_portsc);
		}
		break;
	case USB_PORT_FEAT_SUSPEND:
		KprintfT("Clear port %lu PORT_SUSPEND\n", (ULONG)portNo);
		/* For USB2, need to write 15 (XDEV_RESUME) first, wait 20ms, then write U0 */
		if (rh->ports[portNo - 1].major_revision < 3)
		{
			xhci_port_set_link_state(rh->udev->controller->hcor, portNo, XDEV_RESUME);
			rh_sleep_unlocked(rh->udev->controller, 25); // wait at least 20ms for resume to take effect
		}
		/* put port back to U0 (active) state */
		xhci_port_set_link_state(rh->udev->controller->hcor, portNo, XDEV_U0);
		/* Restart the attached device's endpoint rings (TDs kept across U3). */
		xhci_udev_resume_port(rh->udev, (u8)portNo);
		break;
	case USB_PORT_FEAT_C_ENABLE:
	case USB_PORT_FEAT_C_SUSPEND:
		xhci_roothub_clear_port_change_bit(wValue, portNo, &port->or_portsc, reg);
		break;

	default:
		Kprintf("Clear port %lu:Unknown feature 0x%lx\n", (ULONG)portNo, (ULONG)wValue);
		xhci_roothub_stall(req);
		return;
	}

	xhci_roothub_no_error(req);
}

static void xhci_roothub_handle_hub_get_descriptor(struct xhci_root_hub_view *v, struct xhci_xfer *req)
{
	struct xhci_root_hub *rh = v->rh;
	KprintfT("USB_REQ_GET_DESCRIPTOR HUB\n");
	const u16 wValue = le16(req->setup.usd_Value);
	const BOOL want_ss = (wValue >> 8) == USB_DT_SS_HUB;
	const BOOL serve_ss = (v->format == RH_VIEW_SS);

	if (want_ss != serve_ss || (!want_ss && (wValue >> 8) != USB_DT_HUB))
	{
		Kprintf("get unknown value %lx\n", wValue);
		xhci_roothub_stall(req);
		return;
	}

	/* patch the port count to the view's protocol subset */
	struct usb_hub_descriptor hub = serve_ss ? rh->descriptor.hub_30 : rh->descriptor.hub_20;
	hub.bNbrPorts = v->num_ports;
	xhci_roothub_reply(req, &hub, hub.bLength);
}

static void xhci_roothub_handle_hub_get_status(struct xhci_root_hub *rh, struct xhci_xfer *req)
{
	(void)rh;
	KprintfT("USB_REQ_GET_STATUS HUB\n");
	/* Hub GET_STATUS: bit0=local power source, bit1=over-current */
	u8 status[4] = {0, 0, 0, 0};
	xhci_roothub_reply(req, status, 4);
}

static void xhci_roothub_handle_port_get_status(struct xhci_root_hub_view *v, struct xhci_xfer *req)
{
	struct xhci_root_hub *rh = v->rh;
	const u16 wIndex = le16(req->setup.usd_Index);
	const u16 wValue = le16(req->setup.usd_Value);
	const u16 wLength = le16(req->setup.usd_Length);
	const u8 portNo = wIndex & 0xffU; /* controller-global (translated at dispatch) */
	const u8 portStatusType = wValue & 0xffU;

	if (portStatusType != 0 || wLength != 4)
	{
		Kprintf("invalid get port status request value 0x%lx length %lu\n", (ULONG)wValue, (ULONG)wLength);
		xhci_roothub_stall(req);
		return;
	}

#ifdef TRACE
	xhci_roothub_debug_port(rh, portNo - 1u);
#endif

	struct xhci_hcor_port_regs *port = xhci_roothub_get_port(rh, req);
	u32 reg = mmio_read32(&port->or_portsc);

	/* Device-initiated resume parks a USB2-protocol port in the Resume link
	 * state until software completes it (xHCI 4.15.2.2: wait TDRSMDN = 20 ms,
	 * then direct the port to U0).  Complete it here so the decode below sees
	 * the resume — C_SUSPEND is only synthesized at PLS==U0.  SS ports retrain
	 * to U0 in hardware and never sit in Resume, so this is inherently
	 * USB2-only.  Runs on the unit task (same blocking budget as the software
	 * resume dance in the CLEAR_FEATURE handler). */
	if ((reg & PORT_PLS_MASK) == XDEV_RESUME)
	{
		Kprintf("port %lu: completing device-initiated resume (Resume -> U0)\n", (ULONG)portNo);
		rh_sleep_unlocked(rh->udev->controller, 20);
		xhci_port_set_link_state(rh->udev->controller->hcor, portNo, XDEV_U0);
		rh_sleep_unlocked(rh->udev->controller, 3); /* let the transition settle before sampling */
		reg = mmio_read32(&port->or_portsc);
	}

	u16 wPortStatus;
	u16 wPortChange = 0;

	switch (v->format)
	{
	case RH_VIEW_SS:
		/* USB3-spec-pure: bits 0-9 track PORTSC (same layout as the spec),
		 * and the speed field (bits 12:10) stays 0 = 5 Gbps — everything on
		 * a SuperSpeed hub IS SuperSpeed, exactly like an external SS hub. */
		wPortStatus = reg & 0x3ff;
		wPortChange = rh_view_port_change(RH_VIEW_SS, reg);
		break;

	default: /* RH_VIEW_USB2 */
		/* classic USB 2.0 hub port status */
		wPortStatus = reg & (PORT_CONNECT | PORT_PE | PORT_OC | PORT_RESET);
		if (reg & PORT_POWER)
			wPortStatus |= USB_PORT_STAT_POWER;
		if ((reg & PORT_PLS_MASK) == XDEV_U3)
			wPortStatus |= USB_PORT_STAT_SUSPEND;
		switch (reg & DEV_SPEED_MASK)
		{
		case XDEV_FS:
			/* USB 2.0 full-speed is represented by neither LS nor HS bits set. */
			break;
		case XDEV_LS:
			wPortStatus |= USB_PORT_STAT_LOW_SPEED;
			break;
		case XDEV_HS:
			wPortStatus |= USB_PORT_STAT_HIGH_SPEED;
			break;
		}

		wPortChange = rh_view_port_change(RH_VIEW_USB2, reg);
		/* report C_PORT_SUSPEND when the port resumed to U0 */
		if ((reg & PORT_PLC) && (reg & PORT_PLS_MASK) == XDEV_U0)
			wPortChange |= USB_PORT_STAT_C_SUSPEND;
		break;

	}

	u8 tmpbuf[4] = {wPortStatus & 0xffU, (wPortStatus >> 8) & 0xffU, wPortChange & 0xffU, (wPortChange >> 8) & 0xffU};
	KprintfT("USB_REQ_GET_STATUS PORT %lu status=0x%lx\n", (ULONG)portNo, (ULONG)reg);
	xhci_roothub_reply(req, tmpbuf, 4);
}

static void xhci_roothub_handle_get_port_error_count(struct xhci_root_hub *rh, struct xhci_xfer *req)
{
	const u16 wIndex = le16(req->setup.usd_Index);
	const u8 portNo = wIndex & 0xffU;

	if (rh->ports[portNo - 1].major_revision < 3)
	{
		Kprintf("get port error count not supported on USB 2.0 port %lu\n", (ULONG)portNo);
		xhci_roothub_stall(req);
		return;
	}
	if (req->setup.usd_Value != 0 || le16(req->setup.usd_Length) < 2)
	{
		Kprintf("invalid get port error count request value 0x%lx length %lu\n", le16(req->setup.usd_Value), (ULONG)le16(req->setup.usd_Length));
		xhci_roothub_stall(req);
		return;
	}

	struct xhci_hcor_port_regs *port = xhci_roothub_get_port(rh, req);
	const u16 errors = mmio_read32(&port->or_portli) & 0xffff;

	KprintfT("USB_REQ_GET_PORT_ERROR_COUNT PORT %lu error count=%u\n", (ULONG)wIndex, errors);
	u8 tmpbuf[2] = {errors & 0xffU, (u8)(errors >> 8)};
	xhci_roothub_reply(req, tmpbuf, 2);
}

/* Warm-reset a SuperSpeed root port, with the VL805 workaround.  Shared by the
 * hot-reset upgrade (SetPortFeature PORT_RESET on an SS port) and the explicit
 * warm reset (SetPortFeature BH_PORT_RESET) so both get the same robust path. */
static void xhci_roothub_warm_reset_port(struct xhci_ctrl *ctrl, struct xhci_hcor_port_regs *port, u8 portNo)
{
	(void)portNo; /* debug prints only */
	u32 reg = mmio_read32(&port->or_portsc);
	reg = xhci_port_state_to_neutral(reg);

	KprintfT("SS port %lu warm reset (PLS=%lu portsc=%08lx)\n",
			 (ULONG)portNo, (ULONG)((reg & PORT_PLS_MASK) >> 5),
			 (ULONG)mmio_read32(&port->or_portsc));

	/* Clear all pending change bits before the warm reset.
	 * Stale change bits (especially PLC from a Compliance
	 * transition) can cause the VL805 to botch the warm reset:
	 * the link bounces through disconnect/reconnect and recovers
	 * via normal link training instead, leaving WRC=0 and the
	 * device in a state where ADDRESS_DEVICE times out. */
	mmio_write32(reg | PORT_CSC | PORT_PEC | PORT_WRC |
					 PORT_OCC | PORT_RC | PORT_PLC | PORT_CEC,
				 &port->or_portsc);

	/* Re-read and re-neutralize after clearing change bits */
	reg = mmio_read32(&port->or_portsc);
	reg = xhci_port_state_to_neutral(reg);

	/* Initiate warm reset */
	mmio_write32(reg | PORT_WR, &port->or_portsc);

	/* Poll until the warm reset completes (PORT_RESET clears)
	 * or we time out.  Linux's hub_port_wait_reset() does the
	 * same at the hub-driver level. */
	{
		int attempts;
		u32 temp;
		for (attempts = 0; attempts < 100; attempts++)
		{
			rh_sleep_unlocked(ctrl, 10);
			temp = mmio_read32(&port->or_portsc);
			if (!(temp & PORT_RESET))
				break;
		}

		if (attempts >= 100)
		{
			Kprintf("SS port %lu warm reset timed out after 1s "
					"(portsc=%08lx)\n",
					(ULONG)portNo, (ULONG)temp);
		}
		else
		{
			KprintfT("SS port %lu warm reset completed in %ld0ms "
					 "(portsc=%08lx)\n",
					 (ULONG)portNo, (LONG)attempts, (ULONG)temp);
		}
	}
}

static void xhci_roothub_handle_port_set_feature(struct xhci_root_hub *rh, struct xhci_xfer *req)
{
	const u16 wValue = le16(req->setup.usd_Value);
	const u16 wIndex = le16(req->setup.usd_Index);
	const u8 portNo = wIndex & 0xffU;

#ifdef TRACE
	xhci_roothub_debug_port(rh, portNo - 1u);
#endif

	struct xhci_hcor_port_regs *port = xhci_roothub_get_port(rh, req);
	u32 reg = mmio_read32(&port->or_portsc);
	reg = xhci_port_state_to_neutral(reg);

	switch (wValue)
	{
	// Common for USB2 and USB3 ports
	case USB_PORT_FEAT_RESET:
	{
		/* Always warm-reset SuperSpeed root ports. A hot reset can leave the
		 * SuperSpeed link trained-but-dysfunctional (the port reports enabled
		 * /U0, yet ADDRESS_DEVICE times out), and that bad state is not
		 * observable from PORTSC - it is not limited to Compliance/SS.Inactive.
		 * A warm reset forces full link retraining and is the reliable path;
		 * it is a superset of a hot reset, so anything that worked after a hot
		 * reset still works after a warm reset. */
		if (rh->ports[portNo - 1].major_revision >= 3)
		{
			xhci_roothub_warm_reset_port(rh->udev->controller, port, portNo);
		}
		else
		{
			KprintfT("Set port %lu PORT_RESET\n", (ULONG)portNo);
			reg |= PORT_RESET;
			mmio_write32(reg, &port->or_portsc);
		}

		/* Allow the link partner to stabilise before
		 * the stack tries ADDRESS_DEVICE. */
		rh_sleep_unlocked(rh->udev->controller, 50);
		break;
	}
	case USB_PORT_FEAT_POWER:
		KprintfT("Set port %lu PORT_POWER\n", (ULONG)portNo);
		reg |= PORT_POWER;
		mmio_write32(reg, &port->or_portsc);
		break;
	case USB_PORT_FEAT_CONNECTION:
	case USB_PORT_FEAT_OVER_CURRENT:
	case USB_PORT_FEAT_LOWSPEED:
	case USB_PORT_FEAT_HIGHSPEED:
		/* No-op */
		break;

	// USB3 specific features
	case USB_SS_PORT_FEAT_BH_RESET:
		KprintfT("Set port %lu PORT_BH_RESET\n", (ULONG)portNo);
		xhci_roothub_warm_reset_port(rh->udev->controller, port, portNo);
		/* stabilise before the stack re-enumerates */
		rh_sleep_unlocked(rh->udev->controller, 50);
		break;
	case USB_SS_PORT_FEAT_U1_TIMEOUT:
		xhci_roothub_set_usb3_port_timeout(rh, portNo, /*u2*/ FALSE, (u16)(wIndex >> 8));
		break;
	case USB_SS_PORT_FEAT_U2_TIMEOUT:
		xhci_roothub_set_usb3_port_timeout(rh, portNo, /*u2*/ TRUE, (u16)(wIndex >> 8));
		break;
	case USB_PORT_FEAT_LINK_STATE:
	{
		const u32 link_state = wIndex >> 8;
		KprintfT("Set port %lu PORT_LINK_STATE to %lu\n", (ULONG)portNo, (ULONG)link_state);
		if (link_state <= 5 || link_state == 10)
		{
			reg &= ~PORT_PLS_MASK;
			reg |= PORT_LINK_STROBE;
			reg |= link_state << 5;
			mmio_write32(reg, &port->or_portsc);
		}
		else
		{
			Kprintf("invalid link state %lu\n", (ULONG)link_state);
			xhci_roothub_stall(req);
			return;
		}
		break;
	}
	case USB_SS_PORT_FEAT_REMOTE_WAKE_MASK:
	{
		const u32 wake_mask = (wIndex >> 8) & 7;
		KprintfT("Set port %lu REMOTE_WAKE_MASK to %lx\n", (ULONG)portNo, (ULONG)wake_mask);
		reg &= ~(PORT_WKCONN_E | PORT_WKDISC_E | PORT_WKOC_E);
		reg |= wake_mask << 25;
		mmio_write32(reg, &port->or_portsc);
		break;
	}
	case USB_SS_PORT_FEAT_FORCE_LINKPM_ACCEPT:
		KprintfT("Set port %lu FORCE_LINKPM_ACCEPT\n", (ULONG)portNo);
		reg = mmio_read32(&port->or_portpmsc);
		reg |= PORT_FLA;
		mmio_write32(reg, &port->or_portpmsc);
		break;

	// USB2 specific features
	case USB_PORT_FEAT_SUSPEND:
		/* xHCI 4.15.1: the attached device's endpoint rings must be stopped
		 * before the port goes to U3.  With a device present the U3 write is
		 * deferred to xhci_udev_suspend_finish() (the request completes now;
		 * port status reflects U3 moments later).  Empty port, or a device
		 * with nothing to stop: suspend immediately. */
		if (xhci_udev_suspend_port(rh->udev, (u8)portNo))
			break;
		KprintfT("Putting port %lu link to U3 standby\n", (ULONG)portNo);
		xhci_port_set_link_state(rh->udev->controller->hcor, portNo, XDEV_U3);
		break;
	case USB_PORT_FEAT_TEST:
		/* No-op */
		break;

	default:
		Kprintf("Set port %lu: unknown feature %lx\n", (ULONG)portNo, (ULONG)wValue);
		xhci_roothub_stall(req);
		return;
	}

	xhci_roothub_no_error(req);
}

/**
 * Handle a control request addressed to this root-hub view
 *
 * @param v  root-hub view the request targets
 * @param io pointer to the xfer structure
 */
void xhci_roothub_view_submit_ctrl_request(struct xhci_root_hub_view *v, struct xhci_xfer *io)
{
	struct xhci_root_hub *rh = v->rh;
	u16 wIndex = le16(io->setup.usd_Index);
	const u16 wValue = le16(io->setup.usd_Value);

	struct UhcdSetupData *setup = &io->setup;

	/* Port-recipient requests only (hub- and device-recipient requests carry
	 * no port in wIndex).  Ports are 1-based; the port handlers index
	 * rh->ports[port - 1] and dereference the port register block, so 0 must
	 * not get through. */
	if ((setup->usd_RequestType & USB_RECIP_MASK) == USB_RECIP_OTHER)
	{
		const u8 local = wIndex & 0xffU;
		if (local == 0 || local > v->num_ports)
		{
			Kprintf("The request port(%lu) is outside 1..%lu\n", (ULONG)wIndex, (ULONG)v->num_ports);
			xhci_roothub_stall(io);
			return;
		}
		if (v->port_map && local)
		{
			/* rewrite the view-local port to the controller-global number;
			 * only driver-owned shadow requests reach non-identity views, so
			 * a client's packet is never modified */
			setup->usd_Index = le16((u16)((wIndex & 0xff00U) | v->port_map[local - 1]));
			wIndex = le16(setup->usd_Index);
		}
	}

	const u16 typeReq = (u16)(((u16)setup->usd_RequestType << 8) | setup->usd_Request);
	switch (typeReq)
	{
	/* Standard device requests */
	case DeviceOutRequest | USB_REQ_CLEAR_FEATURE:
		/* Remote wakeup is bookkeeping only (the xHC manages root-port wake
		 * hardware); everything else is unsupported. */
		if (wValue == USB_DEVICE_REMOTE_WAKEUP)
		{
			rh->remote_wakeup = FALSE;
			xhci_roothub_no_error(io);
		}
		else
			xhci_roothub_stall(io);
		break;
	case DeviceRequest | USB_REQ_GET_CONFIGURATION:
		xhci_roothub_handle_device_get_configuration(rh, io);
		break;
	case DeviceRequest | USB_REQ_GET_DESCRIPTOR:
		xhci_roothub_handle_device_get_descriptor(v, io);
		break;
	case DeviceRequest | USB_REQ_GET_STATUS:
		xhci_roothub_handle_device_get_status(rh, io);
		break;
	case DeviceOutRequest | USB_REQ_SET_ADDRESS:
		KprintfT("USB_REQ_SET_ADDRESS rootdev=%lu\n", (ULONG)wValue);
		/* Do nothing, higher layer will handle context migration */
		xhci_roothub_no_error(io);
		break;
	case DeviceOutRequest | USB_REQ_SET_CONFIGURATION:
		KprintfT("USB_REQ_SET_CONFIGURATION\n");
		/* Do nothing */
		xhci_roothub_no_error(io);
		break;
	case DeviceOutRequest | USB_REQ_SET_FEATURE:
		/* TEST_MODE is compliance-lab only; remote wakeup as above. */
		if (wValue == USB_DEVICE_REMOTE_WAKEUP)
		{
			rh->remote_wakeup = TRUE;
			xhci_roothub_no_error(io);
		}
		else
			xhci_roothub_stall(io);
		break;
	case DeviceOutRequest | USB_REQ_SET_ISOCH_DELAY:
		KprintfT("USB_REQ_SET_ISOCH_DELAY\n");
		/* Do nothing */
		xhci_roothub_no_error(io);
		break;
	case DeviceOutRequest | USB_REQ_SET_SEL:
		KprintfT("USB_REQ_SET_SEL\n");
		/* Do nothing */
		xhci_roothub_no_error(io);
		break;

	/* Hub class requests */
	case ClearHubFeature:
		KprintfT("CLEAR_FEATURE HUB feature=%lx\n", wValue);
		/* The xHC handles hub power/over-current in hardware and our hub
		 * GET_STATUS never reports the change bits - ACK as no-ops (mirrors
		 * Linux rh_call_control). */
		if (wValue == C_HUB_LOCAL_POWER || wValue == C_HUB_OVER_CURRENT)
			xhci_roothub_no_error(io);
		else
			xhci_roothub_stall(io);
		break;
	case ClearPortFeature:
		xhci_roothub_handle_port_clear_feature(rh, io);
		break;
	case GetHubDescriptor:
		xhci_roothub_handle_hub_get_descriptor(v, io);
		break;
	case GetHubStatus:
		xhci_roothub_handle_hub_get_status(rh, io);
		break;
	case GetPortStatus:
		xhci_roothub_handle_port_get_status(v, io);
		break;
	case GetPortErrorCount:
		KprintfT("USB_REQ_GET_PORT_ERROR_COUNT\n");
		xhci_roothub_handle_get_port_error_count(rh, io);
		break;
	case SetHubFeature:
		KprintfT("SET_FEATURE HUB feature=%lx\n", wValue);
		/* Same two features as ClearHubFeature; setting them is nonsensical
		 * for a root hub but harmless - ACK (mirrors Linux). */
		if (wValue == C_HUB_LOCAL_POWER || wValue == C_HUB_OVER_CURRENT)
			xhci_roothub_no_error(io);
		else
			xhci_roothub_stall(io);
		break;
	case SetHubDepth:
		KprintfT("SET_HUB_DEPTH depth=%lx\n", wValue);
		/* Do nothing */
		xhci_roothub_no_error(io);
		break;
	case SetPortFeature:
		xhci_roothub_handle_port_set_feature(rh, io);
		break;
	default:
		Kprintf("Unknown request\n");
		xhci_roothub_stall(io);
	}
}

struct xhci_root_hub_view *xhci_roothub_view(struct xhci_root_hub *rh, u8 format)
{
	switch (format)
	{
	case RH_VIEW_SS:
		return rh->view_ss.num_ports ? &rh->view_ss : NULL;
	case RH_VIEW_USB2:
		return rh->view_usb2.num_ports ? &rh->view_usb2 : NULL;
	default:
		return NULL;
	}
}

BOOL xhci_roothub_has_usb3_ports(struct xhci_root_hub *rh)
{
	return rh->view_ss.num_ports > 0;
}

BOOL xhci_roothub_has_usb2_ports(struct xhci_root_hub *rh)
{
	return rh->view_usb2.num_ports > 0;
}

u8 xhci_roothub_view_global_port(struct xhci_root_hub_view *v, u8 local)
{
	if (local == 0 || local > v->num_ports)
		return 0;
	return v->port_map ? v->port_map[local - 1] : local;
}
