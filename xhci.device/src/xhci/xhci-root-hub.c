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

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci_root_hub] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#define STATUS_CHANGE_BITMAP_LENGTH 2 /* in bytes; supports up to 15 ports */

static struct descriptor
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

struct xhci_root_hub
{
	struct usb_device *udev;
	io_reply_data_fn io_reply_data;

	struct descriptor descriptor;

	/* INT endpoint request */
	struct USBIORequest *int_req;

	struct xhci_root_hub_port *ports;
	BOOL is_super_speed;
};

#ifdef DEBUG_HIGH
static void xhci_roothub_debug_port(struct xhci_root_hub *rh, u32 port)
{
	struct xhci_ctrl *ctrl = rh->udev->controller;
	u32 portsc = mmio_read32(&ctrl->hcor->portregs[port].or_portsc);
	KprintfH("port %lu status: 0x%08lx\n", (ULONG)port + 1, (ULONG)portsc);

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
		KprintfH("  link state: U1\n");
		break;
	case XDEV_U2:
		KprintfH("  link state: U2\n");
		break;
	case XDEV_U3:
		KprintfH("  link state: U3 (suspended)\n");
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
		KprintfH("  link state: unknown (%lu)\n", (ULONG)(pls >> 5));
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
		KprintfH("  device speed: unknown (%lu)\n", (ULONG)(speed >> 10));
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
	struct xhci_root_hub *rh = pool_zalloc(ctrl->memoryPool, sizeof(struct xhci_root_hub));
	if (!rh)
		return NULL;

	rh->udev = udev;
	rh->io_reply_data = io_reply_data;

	CopyMem(&prototype_descriptor, &rh->descriptor, sizeof(prototype_descriptor));

	const u8 ports = HCS_MAX_PORTS(mmio_read32(&ctrl->hccr->cr_hcsparams1));
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

	if (HCC_LTC(hccParams1))
		rh->descriptor.ss_dev_cap.bmAttributes |= USB_SS_DEVICE_ATT_LATENCY_TOLERANCE_MESSAGES;
	// TODO add support for Set Latency Tolerance Value command

	rh->descriptor.hub_30.wHubCharacteristics = le16(wHubCharacteristics);
	rh->descriptor.hub_20.wHubCharacteristics = le16(wHubCharacteristics);

	/* Exit latencies */
	const u32 hcsParams3 = mmio_read32(&ctrl->hccr->cr_hcsparams3);
	rh->descriptor.ss_dev_cap.bU1DevExitLat = HCS_U1_LATENCY(hcsParams3);
	rh->descriptor.ss_dev_cap.wU2DevExitLat = le16(HCS_U2_LATENCY(hcsParams3));
	Kprintf("Host controller U1 exit latency: %lu microseconds\n", (ULONG)rh->descriptor.ss_dev_cap.bU1DevExitLat);
	Kprintf("Host controller U2 exit latency: %lu microseconds\n", (ULONG)le16(rh->descriptor.ss_dev_cap.wU2DevExitLat));

	/* Create port structs and fill with data from protool extended capability */
	rh->ports = pool_zalloc(ctrl->memoryPool, ports * sizeof(struct xhci_root_hub_port));
	if (!rh->ports)
	{
		pool_free(ctrl->memoryPool, rh);
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

#ifdef DEBUG_HIGH
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

	if (rh->ports)
		pool_free(rh->udev->controller->memoryPool, rh->ports);

	pool_free(rh->udev->controller->memoryPool, rh);
}

u16 xhci_roothub_get_address(struct xhci_root_hub *rh)
{
	if (!rh || !rh->udev)
		return 0;

	return rh->udev->virtual_address;
}

u8 xhci_roothub_get_num_ports(struct xhci_root_hub *rh)
{
	if (!rh)
		return 0;

	return rh->descriptor.hub_30.bNbrPorts;
}

s8 xhci_roothub_submit_int_request(struct xhci_root_hub *rh, struct USBIORequest *req)
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

	const u8 num_ports = rh->descriptor.hub_30.bNbrPorts;
	/* USB 3.0 spec is always two bytes, however the stack may request fewer bytes */
	const u8 need_bytes = (u8)(((u32)num_ports + 7U) / 8U);
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
		u32 portsc = mmio_read32(&ctrl->hcor->portregs[port].or_portsc);
		u32 change_bits = portsc & change_mask;
		if (!change_bits)
			continue;

		u32 index = (port + 1U) >> 3;
		if (index >= need_bytes)
			continue;

		buffer[index] |= (u8)(1U << (((u32)port + 1U) & 7U));
		KprintfH("port %lu status change detected: changebits=0x%08lx\n", (ULONG)(port + 1), (ULONG)change_bits);
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

void xhci_roothub_abort_int_request(struct xhci_root_hub *rh)
{
	if (!rh || !rh->int_req)
		return;

	rh->io_reply_data(rh->udev, rh->int_req, IOERR_ABORTED, 0);
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

	u32 req_length = le16(req->setup.wLength);
	length = length < req_length ? length : req_length;

	if (data && length > 0)
		CopyMem(data, req->data_buffer, length);

	req->actual_length = length;
	req->req.io_Error = ERR_NO_ERROR;
}

inline static void xhci_roothub_delay_ms(u32 milliseconds)
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

	switch (le16(setup->wValue) >> 8)
	{
	case USB_DT_BOS:
		KprintfH("get USB_DT_BOS\n");
		xhci_roothub_reply(req, &rh->descriptor.bos, le16(rh->descriptor.bos.wTotalLength));
		break;
	case USB_DT_DEVICE:
		KprintfH("get USB_DT_DEVICE\n");
		if (rh->is_super_speed)
			xhci_roothub_reply(req, &rh->descriptor.device_30, rh->descriptor.device_30.bLength);
		else
			xhci_roothub_reply(req, &rh->descriptor.device_20, rh->descriptor.device_20.bLength);
		break;
	case USB_DT_CONFIG:
		KprintfH("get USB_DT_CONFIG\n");
		xhci_roothub_reply(req, &rh->descriptor.config, le16(rh->descriptor.config.wTotalLength));
		break;
	case USB_DT_STRING:
		KprintfH("get USB_DT_STRING\n");
		switch (le16(setup->wValue) & 0xff)
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
			Kprintf("unknown value DT_STRING %lx\n", le16(setup->wValue));
			xhci_roothub_stall(req);
		}
		break;
	default:
		Kprintf("get unknown wValue %lx\n", le16(setup->wValue));
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
inline static void xhci_roothub_clear_port_change_bit(u16 wValue, u8 portNo, volatile u32 *addr, u32 port_status)
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
	mmio_write32(port_status | status, addr);

#ifdef DEBUG_HIGH
	port_status = mmio_read32(addr);
	KprintfH("clear port %s change, actual port %lu status  = 0x%lx\n", port_change_bit, (ULONG)portNo, (ULONG)port_status);
#else
	(void)port_change_bit;
	(void)portNo;
#endif
}

inline static struct xhci_hcor_port_regs *xhci_roothub_get_port(struct xhci_root_hub *rh, struct USBIORequest *req)
{
	struct xhci_ctrl *ctrl = rh->udev->controller;
	u8 port = le16(req->setup.wIndex) & 0xffU; // port number is in low byte of wIndex;

	if (port == 0 || port > rh->descriptor.hub_30.bNbrPorts)
		return NULL;

	return &ctrl->hcor->portregs[port - 1];
}

static void xhci_roothub_handle_port_clear_feature(struct xhci_root_hub *rh, struct USBIORequest *req)
{
	const u16 wValue = le16(req->setup.wValue); // feature selector
	const u16 wIndex = le16(req->setup.wIndex); // selector | port
	const u8 portNo = wIndex & 0xffU;
#ifdef DEBUG_HIGH
	xhci_roothub_debug_port(rh, portNo - 1u);
#endif

	struct xhci_hcor_port_regs *port = xhci_roothub_get_port(rh, req);
	u32 reg = mmio_read32(&port->or_portsc);
	reg = xhci_roothub_port_state_to_neutral(reg);

	switch (wValue)
	{
	// Common for USB2 and USB3 ports
	case USB_PORT_FEAT_POWER:
		KprintfH("Clear port %lu PORT_POWER\n", (ULONG)portNo);
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
				tmp = xhci_roothub_port_state_to_neutral(tmp);
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
		KprintfH("Clear port %lu FORCE_LINKPM_ACCEPT\n", (ULONG)portNo);
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
		KprintfH("Clear port %lu PORT_PE\n", (ULONG)portNo);
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
		KprintfH("Clear port %lu PORT_SUSPEND\n", (ULONG)portNo);
		/* For USB2, need to write 15 (XDEV_RESUME) first, wait 20ms, then write U0 */
		if (rh->ports[portNo - 1].major_revision < 3)
		{
			reg &= ~PORT_PLS_MASK;
			reg |= XDEV_RESUME;
			reg |= PORT_LINK_STROBE;
			mmio_write32(reg, &port->or_portsc);
			xhci_roothub_delay_ms(25); // wait at least 20ms for resume to take effect
			reg = mmio_read32(&port->or_portsc);
			reg = xhci_roothub_port_state_to_neutral(reg);
		}
		reg &= ~PORT_PLS_MASK;
		reg |= XDEV_U0; // put port back to U0 (active) state
		reg |= PORT_LINK_STROBE;
		mmio_write32(reg, &port->or_portsc);
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

static void xhci_roothub_handle_hub_get_descriptor(struct xhci_root_hub *rh, struct USBIORequest *req)
{
	(void)rh;
	KprintfH("USB_REQ_GET_DESCRIPTOR HUB\n");
	const u16 wValue = le16(req->setup.wValue);
	if (wValue >> 8 == USB_DT_SS_HUB && rh->is_super_speed)
	{
		xhci_roothub_reply(req, &rh->descriptor.hub_30, rh->descriptor.hub_30.bLength);
	}
	else if (wValue >> 8 == USB_DT_HUB && !rh->is_super_speed)
	{
		xhci_roothub_reply(req, &rh->descriptor.hub_20, rh->descriptor.hub_20.bLength);
	}
	else
	{
		Kprintf("get unknown value %lx\n", wValue);
		xhci_roothub_stall(req);
		return;
	}
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
	const u16 wIndex = le16(req->setup.wIndex);
	const u16 wValue = le16(req->setup.wValue);
	const u16 wLength = le16(req->setup.wLength);
	const u8 portNo = wIndex & 0xffU;
	const u8 portStatusType = wValue & 0xffU;

	if (portStatusType != 0 || wLength != 4)
	{
		Kprintf("invalid get port status request value 0x%lx length %lu\n", (ULONG)wValue, (ULONG)wLength);
		xhci_roothub_stall(req);
		return;
	}

#ifdef DEBUG_HIGH
	xhci_roothub_debug_port(rh, portNo - 1u);
#endif

	struct xhci_hcor_port_regs *port = xhci_roothub_get_port(rh, req);
	const u32 reg = mmio_read32(&port->or_portsc);

	u16 wPortStatus;
	if (rh->is_super_speed)
	{
		wPortStatus = reg & 0x3ff; // bits 0-9 same as portsc
		// bits 10-12 are speed for USB3, it's bit 10=1 if enhanced superspeed
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
	}
	else
	{
		wPortStatus = reg & (PORT_CONNECT | PORT_PE | PORT_OC | PORT_RESET); // map bits that are the same in portsc to port status
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
	}

	u16 wPortChange = 0;
	if (reg & PORT_CSC)
		wPortChange |= USB_PORT_STAT_C_CONNECTION;
	if (reg & PORT_OCC)
		wPortChange |= USB_PORT_STAT_C_OVERCURRENT;
	if (reg & PORT_RC)
		wPortChange |= USB_PORT_STAT_C_RESET;

	if (rh->is_super_speed)
	{
		if (reg & PORT_WRC)
			wPortChange |= USB_SS_PORT_STAT_C_BH_RESET;
		if (reg & PORT_PLC)
			wPortChange |= USB_SS_PORT_STAT_C_LINK_STATE;
		if (reg & PORT_CEC)
			wPortChange |= USB_SS_PORT_STAT_C_CONFIG_ERROR;
	}

	// only report enable change for USB 2.0 ports, for USB 3.0 ports this is reserved
	if (rh->ports[portNo - 1].major_revision < 3 && (reg & PORT_PEC))
		wPortChange |= USB_PORT_STAT_C_ENABLE;

	// report C_PORT_SUSPEND for USB 2.0 hubs when port is resumed from suspend
	if (!rh->is_super_speed && (reg & PORT_PLC) && (reg & PORT_PLS_MASK) == XDEV_U0)
		wPortChange |= USB_PORT_STAT_C_SUSPEND;

	u8 tmpbuf[4] = {wPortStatus & 0xffU, (wPortStatus >> 8) & 0xffU, wPortChange & 0xffU, (wPortChange >> 8) & 0xffU};
	KprintfH("USB_REQ_GET_STATUS PORT %lu status=0x%lx\n", (ULONG)portNo, (ULONG)reg);
	xhci_roothub_reply(req, tmpbuf, 4);
}

static void xhci_roothub_handle_get_port_error_count(struct xhci_root_hub *rh, struct USBIORequest *req)
{
	const u16 wIndex = le16(req->setup.wIndex);
	const u8 portNo = wIndex & 0xffU;

	if (rh->ports[portNo - 1].major_revision < 3)
	{
		Kprintf("get port error count not supported on USB 2.0 port %lu\n", (ULONG)portNo);
		xhci_roothub_stall(req);
		return;
	}
	if (req->setup.wValue != 0 || le16(req->setup.wLength) < 2)
	{
		Kprintf("invalid get port error count request value 0x%lx length %lu\n", le16(req->setup.wValue), (ULONG)le16(req->setup.wLength));
		xhci_roothub_stall(req);
		return;
	}

	struct xhci_hcor_port_regs *port = xhci_roothub_get_port(rh, req);
	const u16 errors = mmio_read32(&port->or_portli) & 0xffff;

	KprintfH("USB_REQ_GET_PORT_ERROR_COUNT PORT %lu error count=%u\n", (ULONG)wIndex, errors);
	u8 tmpbuf[2] = {errors & 0xffU, (u8)(errors >> 8)};
	xhci_roothub_reply(req, tmpbuf, 2);
}

static void xhci_roothub_handle_port_set_feature(struct xhci_root_hub *rh, struct USBIORequest *req)
{
	const u16 wValue = le16(req->setup.wValue);
	const u16 wIndex = le16(req->setup.wIndex);
	const u8 portNo = wIndex & 0xffU;

#ifdef DEBUG_HIGH
	xhci_roothub_debug_port(rh, portNo - 1u);
#endif

	struct xhci_hcor_port_regs *port = xhci_roothub_get_port(rh, req);
	u32 reg = mmio_read32(&port->or_portsc);
	reg = xhci_roothub_port_state_to_neutral(reg);

	switch (wValue)
	{
	// Common for USB2 and USB3 ports
	case USB_PORT_FEAT_RESET:
	{
		/* USB 3.0 ports in Compliance or SS.Inactive state need a warm reset
		 * to recover from link training failure. A hot reset alone may briefly
		 * train the link but leave it unstable. This mirrors Linux's
		 * hub_port_warm_reset_required() logic in hub.c. */
		u32 pls = reg & PORT_PLS_MASK;
		if (rh->ports[portNo - 1].major_revision >= 3 &&
			(pls == XDEV_COMPLIANCE || pls == XDEV_INACTIVE))
		{
			Kprintf("SS port %lu PLS=%lu (%s); upgrading to warm reset (portsc=%08lx)\n",
					(ULONG)portNo, (ULONG)(pls >> 5),
					pls == XDEV_COMPLIANCE ? "Compliance" : "SS.Inactive",
					(ULONG)mmio_read32(&port->or_portsc));

			/* Clear all pending change bits before the warm reset.
			 * Stale change bits (especially PLC from the Compliance
			 * transition) can cause the VL805 to botch the warm reset:
			 * the link bounces through disconnect/reconnect and recovers
			 * via normal link training instead, leaving WRC=0 and the
			 * device in a state where ADDRESS_DEVICE times out. */
			mmio_write32(reg | PORT_CSC | PORT_PEC | PORT_WRC |
							 PORT_OCC | PORT_RC | PORT_PLC | PORT_CEC,
						 &port->or_portsc);

			/* Re-read and re-neutralize after clearing change bits */
			reg = mmio_read32(&port->or_portsc);
			reg = xhci_roothub_port_state_to_neutral(reg);

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
					xhci_roothub_delay_ms(10);
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
					Kprintf("SS port %lu warm reset completed in %ld0ms "
							"(portsc=%08lx)\n",
							(ULONG)portNo, (LONG)attempts, (ULONG)temp);
				}
			}
		}
		else
		{
			KprintfH("Set port %lu PORT_RESET\n", (ULONG)portNo);
			reg |= PORT_RESET;
			mmio_write32(reg, &port->or_portsc);
		}

		/* Allow the link partner to stabilise before
		 * the stack tries ADDRESS_DEVICE. */
		xhci_roothub_delay_ms(50);
		break;
	}
	case USB_PORT_FEAT_POWER:
		KprintfH("Set port %lu PORT_POWER\n", (ULONG)portNo);
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
		KprintfH("Set port %lu PORT_BH_RESET\n", (ULONG)portNo);
		reg |= PORT_WR;
		mmio_write32(reg, &port->or_portsc);
		break;
	case USB_SS_PORT_FEAT_U1_TIMEOUT:
		KprintfH("Setting port %lu U1 timeout to %lu microseconds\n", (ULONG)portNo, (ULONG)(wIndex >> 8));
		reg = mmio_read32(&port->or_portpmsc);
		reg &= ~0xffU;
		reg |= PORT_U1_TIMEOUT(wIndex >> 8);
		mmio_write32(reg, &port->or_portpmsc);
		break;
	case USB_SS_PORT_FEAT_U2_TIMEOUT:
		KprintfH("Setting port %lu U2 timeout to %lu microseconds\n", (ULONG)portNo, (ULONG)(wIndex >> 8));
		reg = mmio_read32(&port->or_portpmsc);
		reg &= ~0xff00U;
		reg |= PORT_U2_TIMEOUT(wIndex >> 8);
		mmio_write32(reg, &port->or_portpmsc);
		break;
	case USB_PORT_FEAT_LINK_STATE:
	{
		const u32 link_state = wIndex >> 8;
		KprintfH("Set port %lu PORT_LINK_STATE to %lu\n", (ULONG)portNo, (ULONG)link_state);
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
		KprintfH("Set port %lu REMOTE_WAKE_MASK to %lx\n", (ULONG)portNo, (ULONG)wake_mask);
		reg &= ~(PORT_WKCONN_E | PORT_WKDISC_E | PORT_WKOC_E);
		reg |= wake_mask << 25;
		mmio_write32(reg, &port->or_portsc);
		break;
	}
	case USB_SS_PORT_FEAT_FORCE_LINKPM_ACCEPT:
		KprintfH("Set port %lu FORCE_LINKPM_ACCEPT\n", (ULONG)portNo);
		reg = mmio_read32(&port->or_portpmsc);
		reg |= PORT_FLA;
		mmio_write32(reg, &port->or_portpmsc);
		break;

	// USB2 specific features
	case USB_PORT_FEAT_SUSPEND:
		KprintfH("Putting port %lu link to U3 standby\n", (ULONG)portNo);
		reg &= ~PORT_PLS_MASK;
		reg |= XDEV_U3;
		reg |= PORT_LINK_STROBE;
		mmio_write32(reg, &port->or_portsc);
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
 * Submits the Requests to the XHCI Host Controller
 *
 * @param udev pointer to the USB device structure
 * @param io  pointer to the IOUsbHWReq structure
 */
void xhci_roothub_submit_ctrl_request(struct xhci_root_hub *rh, struct USBIORequest *io)
{
	const u16 wIndex = le16(io->setup.wIndex);
#ifdef DEBUG_HIGH
	const u16 wValue = le16(io->setup.wValue);
#endif

	struct USBSetupPacket *setup = &io->setup;

	if ((setup->bmRequestType & USB_RT_PORT) && (wIndex & 0xff) > rh->descriptor.hub_30.bNbrPorts)
	{
		Kprintf("The request port(%lu) exceeds maximum port number\n", (ULONG)wIndex);
		xhci_roothub_stall(io);
		return;
	}

	const u16 typeReq = (u16)(((u16)setup->bmRequestType << 8) | setup->bRequest);
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
		KprintfH("USB_REQ_SET_ADDRESS rootdev=%lu\n", (ULONG)wValue);
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
