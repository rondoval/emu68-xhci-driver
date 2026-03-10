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

#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#include <clib/timer_protos.h>
#else
#include <proto/exec.h>
#include <proto/timer.h>
#endif

#include <devices/timer.h>

#include <debug.h>
#include <xhci/ch9.h>
#include <xhci/usb_defs.h>
#include <xhci/xhci.h>
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-udev.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci_root_hub] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci_root_hub] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#define STATUS_CHANGE_BITMAP_LENGTH 2 /* in bytes; supports up to 15 ports */

static struct descriptor
{
	struct usb_hub_descriptor hub;
	struct usb_hub_descriptor hub_20;
	struct usb_device_descriptor device;
	struct usb_config_descriptor config;
	struct usb_interface_descriptor interface;
	struct usb_endpoint_descriptor endpoint;
	struct usb_ss_ep_comp_descriptor ss_ep_comp;
	struct usb_bos_descriptor bos;
	struct usb_2_0_extension_capability_descriptor ext_cap;
	struct usb_ss_device_capability_descriptor ss_dev_cap;
	struct usb_container_id_capability_descriptor container_id_cap;
} __attribute__((packed)) prototype_descriptor = {
	.hub = {
		.bLength = 12,
		.bDescriptorType = USB_DT_SS_HUB, /* hub descriptor */
		.bNbrPorts = 2,					  /* patched to real port count during init */
		.wHubCharacteristics = cpu_to_le16(HUB_CHAR_INDV_PORT_LPSM |
										   HUB_CHAR_INDV_PORT_OCPM), /* per-port power + OC */
		.bPwrOn2PwrGood = 10,										 /* 20 ms between power on and usable */
		.bHubContrCurrent = 0,										 /* self-powered: no bus draw */
		.u.ss = {
			.bHubHdrDecLat = 0,			 /* no hub delay */
			.wHubDelay = cpu_to_le16(0), /* no hub delay */
			.DeviceRemovable = 0,		 /* all ports permanently wired */
		},
	},
	.device = {
		.bLength = sizeof(struct usb_device_descriptor), /* size of device descriptor */
		.bDescriptorType = USB_DT_DEVICE,				 /* device descriptor */
		.bcdUSB = cpu_to_le16(0x0310),					 /* advertise as USB 3.2 */
		.bDeviceClass = USB_CLASS_HUB,					 /* hub */
		.bDeviceSubClass = 0,							 /* no subclass */
		.bDeviceProtocol = USB_HUB_PR_SS,				 /* super-speed hub */
		.bMaxPacketSize0 = 9,							 /* control endpoint max packet */
		.idVendor = 0x0000,								 /* virtual root hub: leave VID zero */
		.idProduct = 0x0000,							 /* virtual root hub: leave PID zero */
		.bcdDevice = cpu_to_le16(0x0200),				 /* device revision */
		.iManufacturer = 1,								 /* string index */
		.iProduct = 2,									 /* string index */
		.iSerialNumber = 0,								 /* no serial */
		.bNumConfigurations = 1,						 /* single configuration */
	},
	.config = {
		.bLength = sizeof(struct usb_config_descriptor),																																				 /* size of configuration descriptor */
		.bDescriptorType = USB_DT_CONFIG,																																								 /* configuration descriptor */
		.wTotalLength = cpu_to_le16(sizeof(struct usb_config_descriptor) + sizeof(struct usb_interface_descriptor) + sizeof(struct usb_endpoint_descriptor) + sizeof(struct usb_ss_ep_comp_descriptor)), /* config + interface + endpoint + ss companion */
		.bNumInterfaces = 1,																																											 /* single interface */
		.bConfigurationValue = 1,																																										 /* configuration ID */
		.iConfiguration = 0,																																											 /* no string descriptor */
		.bmAttributes = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,																																	 /* must-set + self-powered */
		.bMaxPower = 0,																																													 /* no bus power drawn */
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
		.wMaxPacketSize = cpu_to_le16(STATUS_CHANGE_BITMAP_LENGTH),
		.bInterval = 8,
	},
	.ss_ep_comp = {
		.bLength = sizeof(struct usb_ss_ep_comp_descriptor), /* size of SS endpoint companion descriptor */
		.bDescriptorType = USB_DT_SS_ENDPOINT_COMP,			 /* SS endpoint companion descriptor */
		.bMaxBurst = 0,										 /* no bursting */
		.bmAttributes = 0,									 /* no streams */
		.wBytesPerInterval = cpu_to_le16(STATUS_CHANGE_BITMAP_LENGTH),
	},
	.bos = {
		.bLength = sizeof(struct usb_bos_descriptor),																																														  /* size of BOS descriptor */
		.bDescriptorType = USB_DT_BOS,																																																		  /* BOS descriptor */
		.wTotalLength = cpu_to_le16(sizeof(struct usb_bos_descriptor) + sizeof(struct usb_2_0_extension_capability_descriptor) + sizeof(struct usb_ss_device_capability_descriptor) + sizeof(struct usb_container_id_capability_descriptor)), /* total length of all BOS descriptors */
		.bNumDeviceCaps = 3,																																																				  /* number of device capability descriptors */
	},
	.ext_cap = {
		.bLength = sizeof(struct usb_2_0_extension_capability_descriptor), /* size of USB 2.0 extension descriptor */
		.bDescriptorType = USB_DT_DEVICE_CAPABILITY,					   /* device capability descriptor */
		.bDevCapabilityType = USB_CAP_DESC_USB20_EXTENSION,				   /* USB 2.0 extension capability */
		.bmAttributes = cpu_to_le32(0),
	},
	.ss_dev_cap = {
		.bLength = sizeof(struct usb_ss_device_capability_descriptor), /* size of SuperSpeed USB device capability descriptor */
		.bDescriptorType = USB_DT_DEVICE_CAPABILITY,				   /* device capability descriptor */
		.bDevCapabilityType = USB_CAP_DESC_SS_USB_DEVICE,			   /* SuperSpeed USB device capability */
		.bmAttributes = 0,
		.wSpeedsSupported = cpu_to_le16(USB_SS_DEVICE_LOWSPEED_SUPPORT | USB_SS_DEVICE_FULLSPEED_SUPPORT | USB_SS_DEVICE_HIGHSPEED_SUPPORT | USB_SS_DEVICE_SUPERSPEED_SUPPORT), /* supports all speeds */
		.bFunctionalitySupport = 1,																																				/* lowest speed is 1 (low speed) */
		.bU1DevExitLat = 0,																																						/* no U1 exit latency */
		.wU2DevExitLat = cpu_to_le16(0),																																		/* no U2 exit latency */
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

struct xhci_root_hub
{
	struct usb_device *udev;
	io_reply_data_fn io_reply_data;

	struct descriptor descriptor;

	/* INT endpoint request */
	struct USBIORequest *int_req;

	struct xhci_root_hub_port *ports;
};

#ifdef DEBUG_HIGH
static void xhci_roothub_debug_port(struct xhci_root_hub *rh, int port)
{
	struct xhci_ctrl *ctrl = rh->udev->controller;
	u32 portsc = readl(&ctrl->hcor->portregs[port].or_portsc);
	KprintfH("port %ld status: 0x%08lx\n", (LONG)port + 1, portsc);

	if (portsc & PORT_CONNECT) // ROS
		KprintfH("  device connected\n");
	else
		KprintfH("  no device\n");

	if (portsc & PORT_PE) // RW1CS don't disable 3.0 ports
		KprintfH("  port enabled\n");
	else
		KprintfH("  port disabled\n");

	if (portsc & PORT_OC) // RO
		KprintfH("  over-current condition\n");

	if (portsc & PORT_RESET) // RW1S
		KprintfH("  port in reset\n");
	else
		KprintfH("  port not in reset\n");

	u32 pls = portsc & PORT_PLS_MASK; // RWS
	switch (pls)
	{
	case XDEV_U0:
		KprintfH("  link state: U0 (active)\n");
		break;
	case XDEV_U1:
		KprintfH("  link state: U1 (suspended, can wake)\n");
		break;
	case XDEV_U2:
		KprintfH("  link state: U2 (suspended, can wake)\n");
		break;
	case XDEV_U3:
		KprintfH("  link state: U3 (suspended, can't wake)\n");
		break;
	case XDEV_DISABLED:
		KprintfH("  link state: Disabled\n");
		break;
	case XDEV_RXDETECT:
		KprintfH("  link state: RxDetect\n");
		break;
	case XDEV_INACTIVE:
		KprintfH("  link state: Inactive\n");
		break;
	case XDEV_POLLING:
		KprintfH("  link state: Polling\n");
		break;
	case XDEV_RECOVERY:
		KprintfH("  link state: Recovery\n");
		break;
	case XDEV_HOTRESET:
		KprintfH("  link state: Hot Reset\n");
		break;
	case XDEV_COMPLIANCE:
		KprintfH("  link state: Compliance Mode\n");
		break;
	case XDEV_TESTMODE:
		KprintfH("  link state: Test Mode\n");
		break;
	case XDEV_RESUME:
		KprintfH("  link state: Resume\n");
		break;
	default:
		KprintfH("  link state: unknown (%ld)\n", (LONG)pls >> 5);
		break;
	}

	if (portsc & PORT_POWER) // RWS
		KprintfH("  port has power\n");
	else
		KprintfH("  port has no power\n");

	u32 speed = (portsc & DEV_SPEED_MASK); // ROS
	switch (speed)
	{
	case XDEV_FS:
		KprintfH("  device speed: Full Speed\n");
		break;
	case XDEV_LS:
		KprintfH("  device speed: Low Speed\n");
		break;
	case XDEV_HS:
		KprintfH("  device speed: High Speed\n");
		break;
	case XDEV_SS:
		KprintfH("  device speed: Super Speed\n");
		break;
	default:
		KprintfH("  device speed: unknown (%ld)\n", (LONG)speed >> 10);
		break;
	}

	if (portsc & PORT_CSC) // RW1CS connect or disconnect sets this to 1
		KprintfH("  connect status change\n");
	if (portsc & PORT_PEC) // RW1CS set to 1 on transition to disabled from enabled USB 2.0 only
		KprintfH("  port enable change\n");
	if (portsc & PORT_WRC) // RW1CS 1 when warm reset completes (WPR 1->0)
		KprintfH("  warm reset change\n");
	if (portsc & PORT_OCC) // RW1CS when OC transitions to 1
		KprintfH("  over-current change\n");
	if (portsc & PORT_RC) // RW1CS 1 when reset completes (PR 1->0 or WR 1->0)
		KprintfH("  reset change\n");
	if (portsc & PORT_PLC) // RW1CS when PLS changes - 4.19.1
		KprintfH("  port link state change\n");
	if (portsc & PORT_CEC) // RW1CS
		KprintfH("  port config error change\n");

	if (portsc & PORT_WR) // RW1S
		KprintfH("  warm reset in progress\n");
}
#endif

struct xhci_root_hub *xhci_roothub_create(struct usb_device *udev, io_reply_data_fn io_reply_data)
{
	struct xhci_ctrl *ctrl = udev->controller;
	struct xhci_root_hub *rh = AllocVecPooled(ctrl->memoryPool, sizeof(struct xhci_root_hub));
	if (!rh)
		return NULL;

	rh->udev = udev;
	rh->udev->speed = USB_SPEED_SUPER;
	rh->io_reply_data = io_reply_data;

	CopyMem(&prototype_descriptor, &rh->descriptor, sizeof(prototype_descriptor));

	const u8 ports = HCS_MAX_PORTS(readl(&ctrl->hccr->cr_hcsparams1));
	rh->descriptor.hub.bNbrPorts = ports;

	Kprintf("Initializing root hub with %ld ports\n", (LONG)ports);

	/* Port Indicators */
	const u32 hccParams1 = readl(&ctrl->hccr->cr_hccparams1);
	u16 wHubCharacteristics = LE16(rh->descriptor.hub.wHubCharacteristics);
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

	if (HCC_LTC(hccParams1))
		rh->descriptor.ss_dev_cap.bmAttributes |= USB_SS_DEVICE_ATT_LATENCY_TOLERANCE_MESSAGES;
	// TODO add support for Set Latency Tolerance Value command

	rh->descriptor.hub.wHubCharacteristics = LE16(wHubCharacteristics);

	/* Exit latencies */
	const u32 hcsParams3 = readl(&ctrl->hccr->cr_hcsparams3);
	rh->descriptor.ss_dev_cap.bU1DevExitLat = HCS_U1_LATENCY(hcsParams3);
	rh->descriptor.ss_dev_cap.wU2DevExitLat = HCS_U2_LATENCY(hcsParams3);
	Kprintf("Host controller U1 exit latency: %ld microseconds\n", (LONG)rh->descriptor.ss_dev_cap.bU1DevExitLat);
	Kprintf("Host controller U2 exit latency: %ld microseconds\n", (LONG)rh->descriptor.ss_dev_cap.wU2DevExitLat);

	/* Create port structs and fill with data from protool extended capability */
	rh->ports = AllocVecPooled(ctrl->memoryPool, ports * sizeof(struct xhci_root_hub_port));
	if (!rh->ports)
	{
		FreeVecPooled(ctrl->memoryPool, rh);
		Kprintf("Failed to allocate root hub port data\n");
		return NULL;
	}

	u32 next_offset = 0;
	u32 *cap_base;

	while ((cap_base = xhci_find_next_capability(ctrl, XHCI_EXT_CAPS_PROTOCOL, &next_offset)) != NULL)
	{
		struct xhci_protocol_caps caps = xhci_get_protocol_caps(cap_base);
		for (int port_index = caps.port_offset - 1; port_index < caps.port_offset - 1 + caps.port_count && port_index < ports; ++port_index)
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
			Kprintf("Port %ld: USB %ld.%ld, slot type %ld, max hub depth %ld\n",
					(LONG)port_index + 1,
					caps.major_revision, caps.minor_revision,
					caps.protocol_slot_type, caps.max_hub_depth);
			if (caps.major_revision >= 3)
			{
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

#ifdef DEBUG_HIGH
	for (int i = 0; i < ports; ++i)
		xhci_roothub_debug_port(rh, i);
#endif

	return rh;
}

void xhci_roothub_destroy(struct xhci_root_hub *rh)
{
	if (!rh)
		return;

	if (rh->ports)
		FreeVecPooled(rh->udev->controller->memoryPool, rh->ports);

	FreeVecPooled(rh->udev->controller->memoryPool, rh);
}

unsigned int xhci_roothub_get_address(struct xhci_root_hub *rh)
{
	if (!rh || !rh->udev)
		return 0;

	return rh->udev->virtual_address;
}

UBYTE xhci_roothub_get_num_ports(struct xhci_root_hub *rh)
{
	if (!rh)
		return 0;

	return rh->descriptor.hub.bNbrPorts;
}

int xhci_roothub_submit_int_request(struct xhci_root_hub *rh, struct USBIORequest *req)
{
	if (rh->int_req)
	{
		Kprintf("root hub interrupt request already pending\n");
		return ERR_HCI_ERROR;
	}

	rh->int_req = req;
	xhci_roothub_complete_int_request(rh);
	return ERR_NO_ERROR;
}

void xhci_roothub_complete_int_request(struct xhci_root_hub *rh)
{
	if (!rh || !rh->int_req)
		return;

	struct xhci_ctrl *ctrl = rh->udev->controller;

	u8 *buffer = (u8 *)rh->int_req->data_buffer;

	const u8 num_ports = rh->descriptor.hub.bNbrPorts;
	/* USB 3.0 spec is always two bytes, however the stack may request fewer bytes */
	const u8 need_bytes = (num_ports + 7) / 8;
	if (!buffer || rh->int_req->data_buffer_length < need_bytes)
	{
		rh->io_reply_data(rh->udev, rh->int_req, ERR_DEVICE_STALL, 0);
		rh->int_req = NULL;
		return;
	}

	buffer[0] = 0; /* port 1-7 status change bitmap */
	if (need_bytes > 1)
		buffer[1] = 0; /* port 8-15 status change bitmap */

	const u32 change_mask = PORT_CSC | PORT_OCC | PORT_RC | PORT_WRC | PORT_PLC | PORT_CEC; /* status change bits to report in interrupt */
	BOOL change = FALSE;
	for (u8 port = 0; port < num_ports; ++port)
	{
		u32 portsc = readl(&ctrl->hcor->portregs[port].or_portsc);
		u32 change_bits = portsc & change_mask;
		if (!change_bits)
			continue;

		u32 index = (port + 1) >> 3;
		if (index >= need_bytes)
			continue;

		buffer[index] |= 1U << ((port + 1) & 7);
		KprintfH("port %ld status change detected: changebits=0x%08lx\n", (LONG)(port + 1), (ULONG)change_bits);
		change = TRUE;
	}

	if (!change)
	{
		KprintfH("no status change, not completing root hub interrupt\n");
		return;
	}

	rh->io_reply_data(rh->udev, rh->int_req, ERR_NO_ERROR, need_bytes);
	rh->int_req = NULL;
}

inline static void xhci_roothub_stall(struct USBIORequest *req)
{
	if (req)
	{
		KprintfH("stall\n");
		req->actual_length = 0;
		req->req.io_Error = ERR_DEVICE_STALL;
	}
}

inline static void xhci_roothub_no_error(struct USBIORequest *req)
{
	if (req)
	{
		req->req.io_Error = ERR_NO_ERROR;
	}
}

inline static void xhci_roothub_reply(struct USBIORequest *req, void *data, u32 length)
{
	if (!req)
		return;

	length = min(length, LE16(req->setup.wLength));
	if (data && length > 0)
		CopyMem(data, req->data_buffer, length);

	req->actual_length = length;
	req->req.io_Error = ERR_NO_ERROR;
}

inline static void xhci_roothub_delay_ms(ULONG milliseconds)
{
	if (milliseconds == 0)
		return;

	struct MsgPort *timer_port = CreateMsgPort();
	if (!timer_port)
		return;

	struct timerequest *timer_req = (struct timerequest *)CreateIORequest(timer_port, sizeof(struct timerequest));
	if (!timer_req)
	{
		DeleteMsgPort(timer_port);
		return;
	}

	if (OpenDevice((CONST_STRPTR)TIMERNAME, UNIT_MICROHZ, (struct IORequest *)timer_req, 0) == 0)
	{
		timer_req->tr_node.io_Command = TR_ADDREQUEST;
		timer_req->tr_time.tv_secs = milliseconds / 1000UL;
		timer_req->tr_time.tv_micro = (milliseconds % 1000UL) * 1000UL;
		DoIO((struct IORequest *)timer_req);
		CloseDevice((struct IORequest *)timer_req);
	}

	DeleteIORequest((struct IORequest *)timer_req);
	DeleteMsgPort(timer_port);
}

static void xhci_roothub_handle_device_get_configuration(struct xhci_root_hub *rh, struct USBIORequest *req)
{
	KprintfH("USB_REQ_GET_CONFIGURATION\n");
	xhci_roothub_reply(req, &rh->descriptor.config.bConfigurationValue, 1);
}

static void xhci_roothub_handle_device_get_descriptor(struct xhci_root_hub *rh, struct USBIORequest *req)
{
	struct USBSetupPacket *setup = &req->setup;

	switch (LE16(setup->wValue) >> 8)
	{
	case USB_DT_BOS:
		KprintfH("get USB_DT_BOS\n");
		xhci_roothub_reply(req, &rh->descriptor.bos, LE16(rh->descriptor.bos.wTotalLength));
		break;
	case USB_DT_DEVICE:
		KprintfH("get USB_DT_DEVICE\n");
		xhci_roothub_reply(req, &rh->descriptor.device, rh->descriptor.device.bLength);
		break;
	case USB_DT_CONFIG:
		KprintfH("get USB_DT_CONFIG\n");
		xhci_roothub_reply(req, &rh->descriptor.config, LE16(rh->descriptor.config.wTotalLength));
		break;
	case USB_DT_STRING:
		KprintfH("get USB_DT_STRING\n");
		switch (LE16(setup->wValue) & 0xff)
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
			Kprintf("unknown value DT_STRING %lx\n", LE16(setup->wValue));
			xhci_roothub_stall(req);
		}
		break;
	default:
		Kprintf("get unknown wValue %lx\n", LE16(setup->wValue));
		xhci_roothub_stall(req);
	}
}

static void xhci_roothub_handle_device_get_status(struct xhci_root_hub *rh, struct USBIORequest *req)
{
	(void)rh;
	KprintfH("USB_REQ_GET_STATUS\n");
	/* Device GET_STATUS: bit0=self-powered, bit1=remote-wakeup */
	u8 status[2] = {1, 0}; /* self-powered, remote-wakeup disabled */
	xhci_roothub_reply(req, status, 2);
}

/**
 * Save Read Only (RO) bits and save read/write bits where
 * writing a 0 clears the bit and writing a 1 sets the bit (RWS).
 * For all other types (RW1S, RW1CS, RW, and RZ), writing a '0' has no effect.
 *
 * @param state	state of the Port Status and Control Regsiter
 * Return: a value that would result in the port being in the
 *	   same state, if the value was written to the port
 *	   status control register.
 */
inline static u32 xhci_roothub_port_state_to_neutral(u32 state)
{
	/* Save read-only status and port state */
	return (state & XHCI_PORT_RO) | (state & XHCI_PORT_RWS);
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
inline static void xhci_roothub_clear_port_change_bit(u16 wValue, u8 portNo, volatile uint32_t *addr, u32 port_status)
{
	char *port_change_bit;
	u32 status;
	switch (wValue)
	{
	case USB_PORT_FEAT_C_CONNECTION:
		status = PORT_CSC;
		port_change_bit = "connect";
		break;
	case USB_PORT_FEAT_C_RESET:
		status = PORT_RC;
		port_change_bit = "reset";
		break;
	case USB_PORT_FEAT_C_OVER_CURRENT:
		status = PORT_OCC;
		port_change_bit = "over-current";
		break;
	case USB_SS_PORT_FEAT_C_LINK_STATE:
		status = PORT_PLC;
		port_change_bit = "port link state";
		break;
	case USB_SS_PORT_FEAT_C_CONFIG_ERROR:
		status = PORT_CEC;
		port_change_bit = "config error";
		break;
	case USB_SS_PORT_FEAT_C_BH_RESET:
		status = PORT_WRC;
		port_change_bit = "warm reset";
		break;
	case USB_PORT_FEAT_C_ENABLE:
		status = PORT_PEC;
		port_change_bit = "port enable";
		break;
	case USB_PORT_FEAT_C_SUSPEND:
		status = PORT_PLC;
		port_change_bit = "port suspend (PLC)";
		break;
	default:
		/* Should never happen */
		return;
	}

	/* Change bits are all write 1 to clear */
	writel(port_status | status, addr);

#ifdef DEBUG_HIGH
	port_status = readl(addr);
	KprintfH("clear port %s change, actual port %ld status  = 0x%lx\n", port_change_bit, portNo, port_status);
#else
	(void)port_change_bit;
	(void)portNo;
#endif
}

inline static struct xhci_hcor_port_regs *xhci_roothub_get_port(struct xhci_root_hub *rh, struct USBIORequest *req)
{
	struct xhci_ctrl *ctrl = rh->udev->controller;
	u8 port = LE16(req->setup.wIndex) & 0xff; // port number is in low byte of wIndex;

	if (port == 0 || port > rh->descriptor.hub.bNbrPorts)
		return NULL;

	return &ctrl->hcor->portregs[port - 1];
}

static void xhci_roothub_handle_port_clear_feature(struct xhci_root_hub *rh, struct USBIORequest *req)
{
	const u16 wValue = LE16(req->setup.wValue); // feature selector
	const u16 wIndex = LE16(req->setup.wIndex); // selector | port
	const u8 portNo = wIndex & 0xff;
#ifdef DEBUG_HIGH
	xhci_roothub_debug_port(rh, portNo - 1);
#endif

	struct xhci_hcor_port_regs *port = xhci_roothub_get_port(rh, req);
	u32 reg = readl(&port->or_portsc);
	reg = xhci_roothub_port_state_to_neutral(reg);

	switch (wValue)
	{
	// Common for USB2 and USB3 ports
	case USB_PORT_FEAT_POWER:
		KprintfH("Clear port %ld PORT_POWER\n", portNo);
		reg &= ~PORT_POWER;
		writel(reg, &port->or_portsc);
		break;
	case USB_PORT_FEAT_C_CONNECTION:
	case USB_PORT_FEAT_C_RESET:
	case USB_PORT_FEAT_C_OVER_CURRENT:
		xhci_roothub_clear_port_change_bit(wValue, portNo, &port->or_portsc, reg);
		break;
	case USB_PORT_FEAT_CONNECTION:
	case USB_PORT_FEAT_OVER_CURRENT:
	case USB_PORT_FEAT_RESET:
		/* No-op */
		break;

		// USB3 specific features
	case USB_SS_PORT_FEAT_FORCE_LINKPM_ACCEPT:
		KprintfH("Clear port %ld FORCE_LINKPM_ACCEPT\n", portNo);
		reg = readl(&port->or_portpmsc);
		reg &= ~PORT_FLA;
		writel(reg, &port->or_portpmsc);
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
		KprintfH("Clear port %ld PORT_PE\n", portNo);
		if (rh->ports[portNo - 1].major_revision >= 3)
		{
			Kprintf("Can't disable USB 3.0 port\n");
			xhci_roothub_stall(req);
			return;
		}
		else
		{
			reg |= PORT_PE;
			writel(reg, &port->or_portsc);
		}
		break;
	case USB_PORT_FEAT_SUSPEND:
		KprintfH("Clear port %ld PORT_SUSPEND\n", portNo);
		/* For USB2, need to write 15 (XDEV_RESUME) first, wait 20ms, then write U0 */
		if (rh->ports[portNo - 1].major_revision < 3)
		{
			reg |= XDEV_RESUME;
			reg |= PORT_LINK_STROBE;
			writel(reg, &port->or_portsc);
			xhci_roothub_delay_ms(20);
			reg = readl(&port->or_portsc);
			reg = xhci_roothub_port_state_to_neutral(reg);
		}
		reg |= XDEV_U0; // put port back to U0 (active) state
		reg |= PORT_LINK_STROBE;
		writel(reg, &port->or_portsc);
		break;
	case USB_PORT_FEAT_C_ENABLE:
	case USB_PORT_FEAT_C_SUSPEND:
		xhci_roothub_clear_port_change_bit(wValue, portNo, &port->or_portsc, reg);
		break;

	default:
		Kprintf("Clear port %ld:Unknown feature 0x%lx\n", portNo, wValue);
		xhci_roothub_stall(req);
		return;
	}

	xhci_roothub_no_error(req);
}

static void xhci_roothub_handle_hub_get_descriptor(struct xhci_root_hub *rh, struct USBIORequest *req)
{
	(void)rh;
	KprintfH("USB_REQ_GET_DESCRIPTOR HUB\n");
	const u16 wValue = LE16(req->setup.wValue);
	if (wValue >> 8 != USB_DT_SS_HUB)
	{
		Kprintf("get unknown value %lx\n", wValue);
		xhci_roothub_stall(req);
		return;
	}
	xhci_roothub_reply(req, &rh->descriptor.hub, rh->descriptor.hub.bLength);
}

static void xhci_roothub_handle_hub_get_status(struct xhci_root_hub *rh, struct USBIORequest *req)
{
	(void)rh;
	KprintfH("USB_REQ_GET_STATUS HUB\n");
	/* Hub GET_STATUS: bit0=local power source, bit1=over-current */
	u8 status[4] = {0, 0, 0, 0};
	xhci_roothub_reply(req, status, 4);
}

static void xhci_roothub_handle_port_get_status(struct xhci_root_hub *rh, struct USBIORequest *req)
{
	const u16 wIndex = LE16(req->setup.wIndex);
	const u16 wValue = LE16(req->setup.wValue);
	const u16 wLength = LE16(req->setup.wLength);
	const u8 portNo = wIndex & 0xff;
	const u8 portStatusType = wValue & 0xff;

	if (portStatusType != 0 || wLength != 4)
	{
		Kprintf("invalid get port status request value 0x%lx length %ld\n", wValue, wLength);
		xhci_roothub_stall(req);
		return;
	}

#ifdef DEBUG_HIGH
	xhci_roothub_debug_port(rh, portNo - 1);
#endif

	struct xhci_hcor_port_regs *port = xhci_roothub_get_port(rh, req);
	const u32 reg = readl(&port->or_portsc);

	u16 wPortStatus = reg & 0x3ff; // bits 0-9 same as portsc
	// bits 10-12 are speed, for USB3 it's bit 10=1 if enhanced superspeed
	// fake low/full/high speed bits are used to convey info about USB 2.0 ports to higher layers
	switch (reg & DEV_SPEED_MASK)
	{
	case XDEV_FS:
		wPortStatus |= USB_SS_PORT_STAT_SPEED_FULL;
		break;
	case XDEV_LS:
		wPortStatus |= USB_SS_PORT_STAT_SPEED_LOW;
		break;
	case XDEV_HS:
		wPortStatus |= USB_SS_PORT_STAT_SPEED_HIGH;
		break;
	case XDEV_SS:
		wPortStatus |= USB_SS_PORT_STAT_SPEED_5GBPS;
		break;
	}

	u16 wPortChange = 0;
	if (reg & PORT_CSC)
		wPortChange |= USB_PORT_STAT_C_CONNECTION;
	if (reg & PORT_OCC)
		wPortChange |= USB_PORT_STAT_C_OVERCURRENT;
	if (reg & PORT_RC)
		wPortChange |= USB_PORT_STAT_C_RESET;
	if (reg & PORT_WRC)
		wPortChange |= USB_SS_PORT_STAT_C_BH_RESET;
	if (reg & PORT_PLC)
		wPortChange |= USB_SS_PORT_STAT_C_LINK_STATE;
	if (reg & PORT_CEC)
		wPortChange |= USB_SS_PORT_STAT_C_CONFIG_ERROR;
	if (reg & PORT_PEC && rh->ports[portNo - 1].major_revision < 3) // only report enable change for USB 2.0 ports, for USB 3.0 ports this is reserved
		wPortChange |= USB_PORT_STAT_C_ENABLE;

	u8 tmpbuf[4] = {wPortStatus & 0xff, (wPortStatus >> 8) & 0xff, wPortChange & 0xff, (wPortChange >> 8) & 0xff};
	KprintfH("USB_REQ_GET_STATUS PORT %ld status=0x%lx\n", portNo, reg);
	xhci_roothub_reply(req, tmpbuf, 4);
}

static void xhci_roothub_handle_get_port_error_count(struct xhci_root_hub *rh, struct USBIORequest *req)
{
	const u16 wIndex = LE16(req->setup.wIndex);
	const u8 portNo = wIndex & 0xff;

	if (rh->ports[portNo - 1].major_revision < 3)
	{
		Kprintf("get port error count not supported on USB 2.0 port %ld\n", portNo);
		xhci_roothub_stall(req);
		return;
	}
	if (req->setup.wValue != 0 || LE16(req->setup.wLength) < 2)
	{
		Kprintf("invalid get port error count request value 0x%lx length %ld\n", LE16(req->setup.wValue), LE16(req->setup.wLength));
		xhci_roothub_stall(req);
		return;
	}

	struct xhci_hcor_port_regs *port = xhci_roothub_get_port(rh, req);
	const u16 errors = readl(&port->or_portli) & 0xffff;

	KprintfH("USB_REQ_GET_PORT_ERROR_COUNT PORT %ld error count=%u\n", wIndex, errors);
	u8 tmpbuf[2] = {errors & 0xff, errors >> 8};
	xhci_roothub_reply(req, tmpbuf, 2);
}

static void xhci_roothub_handle_port_set_feature(struct xhci_root_hub *rh, struct USBIORequest *req)
{
	const u16 wValue = LE16(req->setup.wValue);
	const u16 wIndex = LE16(req->setup.wIndex);
	const u8 portNo = wIndex & 0xff;

#ifdef DEBUG_HIGH
	xhci_roothub_debug_port(rh, portNo - 1);
#endif

	struct xhci_hcor_port_regs *port = xhci_roothub_get_port(rh, req);
	u32 reg = readl(&port->or_portsc);
	reg = xhci_roothub_port_state_to_neutral(reg);

	switch (wValue)
	{
	// Common for USB2 and USB3 ports
	case USB_PORT_FEAT_RESET:
		KprintfH("Set port %ld PORT_RESET\n", portNo);
		reg |= PORT_RESET;
		writel(reg, &port->or_portsc);
		break;
	case USB_PORT_FEAT_POWER:
		KprintfH("Set port %ld PORT_POWER\n", portNo);
		reg |= PORT_POWER;
		writel(reg, &port->or_portsc);
		break;
	case USB_PORT_FEAT_CONNECTION:
	case USB_PORT_FEAT_OVER_CURRENT:
	case USB_PORT_FEAT_LOWSPEED:
	case USB_PORT_FEAT_HIGHSPEED:
		/* No-op */
		break;

	// USB3 specific features
	case USB_SS_PORT_FEAT_BH_RESET:
		KprintfH("Set port %ld PORT_BH_RESET\n", portNo);
		reg |= PORT_WR;
		writel(reg, &port->or_portsc);
		break;
	case USB_SS_PORT_FEAT_U1_TIMEOUT:
		KprintfH("Setting port %ld U1 timeout to %ld microseconds\n", portNo, wIndex >> 8);
		reg = readl(&port->or_portpmsc);
		reg &= ~0xff;
		reg |= PORT_U1_TIMEOUT(wIndex >> 8);
		writel(reg, &port->or_portpmsc);
		break;
	case USB_SS_PORT_FEAT_U2_TIMEOUT:
		KprintfH("Setting port %ld U2 timeout to %ld microseconds\n", portNo, wIndex >> 8);
		reg = readl(&port->or_portpmsc);
		reg &= ~0xff00;
		reg |= PORT_U2_TIMEOUT(wIndex >> 8);
		writel(reg, &port->or_portpmsc);
		break;
	case USB_PORT_FEAT_LINK_STATE:
	{
		const u32 link_state = wIndex >> 8;
		KprintfH("Set port %ld PORT_LINK_STATE to %ld\n", portNo, link_state);
		if (link_state <= 5 || link_state == 10)
		{
			reg |= PORT_LINK_STROBE;
			reg |= link_state << 5;
			writel(reg, &port->or_portsc);
		}
		else
		{
			Kprintf("invalid link state %ld\n", link_state);
			xhci_roothub_stall(req);
			return;
		}
		break;
	}
	case USB_SS_PORT_FEAT_REMOTE_WAKE_MASK:
	{
		const u32 wake_mask = (wIndex >> 8) & 7;
		KprintfH("Set port %ld REMOTE_WAKE_MASK to %lx\n", portNo, wake_mask);
		reg &= ~(PORT_WKCONN_E | PORT_WKDISC_E | PORT_WKOC_E);
		reg |= wake_mask << 25;
		writel(reg, &port->or_portsc);
		break;
	}
	case USB_SS_PORT_FEAT_FORCE_LINKPM_ACCEPT:
		KprintfH("Set port %ld FORCE_LINKPM_ACCEPT\n", portNo);
		reg = readl(&port->or_portpmsc);
		reg |= PORT_FLA;
		writel(reg, &port->or_portpmsc);
		break;

	// USB2 specific features
	case USB_PORT_FEAT_SUSPEND:
		KprintfH("Putting port %ld link to U3 standby\n", portNo);
		reg |= XDEV_U3;
		reg |= PORT_LINK_STROBE;
		writel(reg, &port->or_portsc);
		break;
	case USB_PORT_FEAT_TEST:
		/* No-op */
		break;

	default:
		Kprintf("Set port %ld: unknown feature %lx\n", portNo, wValue);
		xhci_roothub_stall(req);
		return;
	}

	xhci_roothub_no_error(req);
}

/**
 * Submits the Requests to the XHCI Host Controller
 *
 * @param udev pointer to the USB device structure
 * @param io  pointer to the IOUsbHWReq structure
 */
void xhci_roothub_submit_ctrl_request(struct xhci_root_hub *rh, struct USBIORequest *io)
{
	const u16 wIndex = LE16(io->setup.wIndex);
#ifdef DEBUG_HIGH
	const u16 wValue = LE16(io->setup.wValue);
#endif

	struct USBSetupPacket *setup = &io->setup;

	if ((setup->bmRequestType & USB_RT_PORT) && (wIndex & 0xff) > rh->descriptor.hub.bNbrPorts)
	{
		Kprintf("The request port(%ld) exceeds maximum port number\n", wIndex);
		xhci_roothub_stall(io);
		return;
	}

	const u16 typeReq = setup->bRequest | setup->bmRequestType << 8;
	switch (typeReq)
	{
	/* Standard device requests */
	case DeviceOutRequest | USB_REQ_CLEAR_FEATURE:
		// TODO
		xhci_roothub_stall(io);
		break;
	case DeviceRequest | USB_REQ_GET_CONFIGURATION:
		xhci_roothub_handle_device_get_configuration(rh, io);
		break;
	case DeviceRequest | USB_REQ_GET_DESCRIPTOR:
		xhci_roothub_handle_device_get_descriptor(rh, io);
		break;
	case DeviceRequest | USB_REQ_GET_STATUS:
		xhci_roothub_handle_device_get_status(rh, io);
		break;
	case DeviceOutRequest | USB_REQ_SET_ADDRESS:
		KprintfH("USB_REQ_SET_ADDRESS rootdev=%ld\n", wValue);
		/* Do nothing, higher layer will handle context migration */
		xhci_roothub_no_error(io);
		break;
	case DeviceOutRequest | USB_REQ_SET_CONFIGURATION:
		KprintfH("USB_REQ_SET_CONFIGURATION\n");
		/* Do nothing */
		xhci_roothub_no_error(io);
		break;
	case DeviceOutRequest | USB_REQ_SET_FEATURE:
		// TODO
		xhci_roothub_stall(io);
		break;
	case DeviceOutRequest | USB_REQ_SET_ISOCH_DELAY:
		KprintfH("USB_REQ_SET_ISOCH_DELAY\n");
		/* Do nothing */
		xhci_roothub_no_error(io);
		break;
	case DeviceOutRequest | USB_REQ_SET_SEL:
		KprintfH("USB_REQ_SET_SEL\n");
		/* Do nothing */
		xhci_roothub_no_error(io);
		break;

	/* Hub class requests */
	case ClearHubFeature:
		KprintfH("CLEAR_FEATURE HUB feature=%lx\n", wValue);
		// TODO
		// C_HUB_LOCAL_POWER
		// C_HUB_OVER_CURRENT
		xhci_roothub_stall(io);
		break;
	case ClearPortFeature:
		xhci_roothub_handle_port_clear_feature(rh, io);
		break;
	case GetHubDescriptor:
		xhci_roothub_handle_hub_get_descriptor(rh, io);
		break;
	case GetHubStatus:
		xhci_roothub_handle_hub_get_status(rh, io);
		break;
	case GetPortStatus:
		xhci_roothub_handle_port_get_status(rh, io);
		break;
	case GetPortErrorCount:
		KprintfH("USB_REQ_GET_PORT_ERROR_COUNT\n");
		xhci_roothub_handle_get_port_error_count(rh, io);
		break;
	case SetHubFeature:
		KprintfH("SET_FEATURE HUB feature=%lx\n", wValue);
		// TODO
		xhci_roothub_stall(io);
		break;
	case SetHubDepth:
		KprintfH("SET_HUB_DEPTH depth=%lx\n", wValue);
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
