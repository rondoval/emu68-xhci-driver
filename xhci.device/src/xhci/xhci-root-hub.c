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
#else
#include <proto/exec.h>
#endif

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

static struct descriptor
{
	struct usb_hub_descriptor hub;
	struct usb_device_descriptor device;
	struct usb_config_descriptor config;
	struct usb_interface_descriptor interface;
	struct usb_endpoint_descriptor endpoint;
} __attribute__((packed)) prototype_descriptor = {
	.hub = {
		.bLength = 0x0b,			   /* placeholder, recalculated once ports known */
		.bDescriptorType = USB_DT_HUB, /* hub descriptor */
		.bNbrPorts = 2,				   /* patched to real port count during init */
		.wHubCharacteristics = cpu_to_le16(HUB_CHAR_INDV_PORT_LPSM |
										   HUB_CHAR_INDV_PORT_OCPM), /* per-port power + OC */
		.bPwrOn2PwrGood = 10,										 /* 20 ms between power on and usable */
		.bHubContrCurrent = 0,										 /* self-powered: no bus draw */
		.u.hs = {
			.DeviceRemovable = {0x00, 0x00},   /* all ports permanently wired */
			.PortPowerCtrlMask = {0x00, 0x00}, /* patched in init when ports known */
		},
	},
	.device = {
		.bLength = sizeof(struct usb_device_descriptor), /* size of device descriptor */
		.bDescriptorType = USB_DT_DEVICE,				 /* device descriptor */
		.bcdUSB = cpu_to_le16(0x0320),					 /* advertise as USB 3.2 */
		.bDeviceClass = USB_CLASS_HUB,					 /* hub */
		.bDeviceSubClass = 0,							 /* no subclass */
		.bDeviceProtocol = USB_HUB_PR_HS_SINGLE_TT,		 /* single-TT */
		.bMaxPacketSize0 = 64,							 /* control endpoint max packet */
		.idVendor = 0x0000,								 /* virtual root hub: leave VID zero */
		.idProduct = 0x0000,							 /* virtual root hub: leave PID zero */
		.bcdDevice = cpu_to_le16(0x0101),				 /* device revision */
		.iManufacturer = 1,								 /* string index */
		.iProduct = 2,									 /* string index */
		.iSerialNumber = 0,								 /* no serial */
		.bNumConfigurations = 1,						 /* single configuration */
	},
	.config = {
		.bLength = sizeof(struct usb_config_descriptor),																									  /* size of configuration descriptor */
		.bDescriptorType = USB_DT_CONFIG,																													  /* configuration descriptor */
		.wTotalLength = cpu_to_le16(sizeof(struct usb_config_descriptor) + sizeof(struct usb_interface_descriptor) + sizeof(struct usb_endpoint_descriptor)), /* config + interface + endpoint */
		.bNumInterfaces = 1,																																  /* single interface */
		.bConfigurationValue = 1,																															  /* configuration ID */
		.iConfiguration = 0,																																  /* no string descriptor */
		.bmAttributes = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER,																						  /* must-set + self-powered */
		.bMaxPower = 0,																																		  /* no bus power drawn */
	},
	.interface = {
		.bLength = sizeof(struct usb_interface_descriptor), /* size of interface descriptor */
		.bDescriptorType = USB_DT_INTERFACE,				/* interface descriptor */
		.bInterfaceNumber = 0,								/* interface index */
		.bAlternateSetting = 0,								/* only one setting */
		.bNumEndpoints = 1,									/* interrupt endpoint only */
		.bInterfaceClass = USB_CLASS_HUB,					/* hub functional interface */
		.bInterfaceSubClass = 0,							/* full/high-speed hub */
		.bInterfaceProtocol = USB_HUB_PR_HS_SINGLE_TT,		/* single-TT */
		.iInterface = 0,									/* no string */
	},
	.endpoint = {
		.bLength = sizeof(struct usb_endpoint_descriptor), /* size of endpoint descriptor */
		.bDescriptorType = USB_DT_ENDPOINT,				   /* endpoint descriptor */
		.bEndpointAddress = (USB_DIR_IN | 1),			   /* INT IN endpoint 1 */
		.bmAttributes = USB_ENDPOINT_XFER_INT,			   /* interrupt */
		.wMaxPacketSize = cpu_to_le16(8),				   /* patched during init */
		.bInterval = 12,								   /* 1ms poll per USB 2.0 hub spec */
		.bRefresh = 0,									   /* unused for INT */
		.bSynchAddress = 0,								   /* not used */
	},
};

struct xhci_root_hub
{
	struct usb_device *udev;
	io_reply_data_fn io_reply_data;

	struct descriptor descriptor;
	u32 ep1_mps;

	struct IOUsbHWReq *int_req;
};

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

	u8 ports = HCS_MAX_PORTS(readl(&ctrl->hccr->cr_hcsparams1));
	if (ports > USB_MAXCHILDREN)
		ports = USB_MAXCHILDREN;

	rh->descriptor.hub.bNbrPorts = ports;

	u8 avail_size = sizeof(rh->descriptor.hub.u.hs.DeviceRemovable);
	u8 needed_size = (ports + 1U + 7U) / 8U;
	if (needed_size > avail_size)
		needed_size = avail_size;

	rh->descriptor.hub.bLength = 7U + 2U * needed_size;
	for (u8 i = 0; i < avail_size; ++i)
	{
		rh->descriptor.hub.u.hs.DeviceRemovable[i] = 0x00;
		rh->descriptor.hub.u.hs.PortPowerCtrlMask[i] = (i < needed_size) ? 0xFF : 0x00;
	}

	rh->ep1_mps = 1U + (ports + 7U) / 8U;
	rh->descriptor.endpoint.wMaxPacketSize = LE16(rh->ep1_mps);

	Kprintf("Initialized root hub with %ld ports\n", (LONG)ports);

	/* Port Indicators */
	u32 reg = readl(&ctrl->hccr->cr_hccparams);
	u16 wHubCharacteristics = LE16(rh->descriptor.hub.wHubCharacteristics);
	if (HCS_INDICATOR(reg))
		wHubCharacteristics |= HUB_CHAR_PORTIND;

	/* Port Power Control */
	if (HCC_PPC(reg))
		wHubCharacteristics |= HUB_CHAR_INDV_PORT_LPSM;

	rh->descriptor.hub.wHubCharacteristics = LE16(wHubCharacteristics);

	return rh;
}

void xhci_roothub_destroy(struct xhci_root_hub *rh)
{
	if (rh)
		FreeVecPooled(rh->udev->controller->memoryPool, rh);
}

unsigned int xhci_roothub_get_address(struct xhci_root_hub *rh)
{
	if (!rh || !rh->udev)
		return 0;

	return rh->udev->poseidon_address;
}

UBYTE xhci_roothub_get_num_ports(struct xhci_root_hub *rh)
{
	if (!rh)
		return 0;

	return rh->descriptor.hub.bNbrPorts;
}

int xhci_roothub_submit_int_request(struct xhci_root_hub *rh, struct IOUsbHWReq *req)
{
	if (rh->int_req)
	{
		Kprintf("root hub interrupt request already pending\n");
		return UHIOERR_HOSTERROR;
	}

	rh->int_req = req;
	xhci_roothub_complete_int_request(rh);
	return UHIOERR_NO_ERROR;
}

/**
 * Clears the Change bits of the Port Status Register
 *
 * @param wValue	request value
 * @param wIndex	request index
 * @param addr		address of posrt status register
 * @param port_status	state of port status register
 * Return: none
 */
static void xhci_roothub_clear_port_change_bit(u16 wValue, u16 wIndex, volatile uint32_t *addr, u32 port_status)
{
	char *port_change_bit;
	u32 status;

	switch (wValue)
	{
	case USB_PORT_FEAT_C_RESET:
		status = PORT_RC | PORT_WRC;
		port_change_bit = "reset";
		break;
	case USB_PORT_FEAT_C_CONNECTION:
		status = PORT_CSC;
		port_change_bit = "connect";
		break;
	case USB_PORT_FEAT_C_OVER_CURRENT:
		status = PORT_OCC;
		port_change_bit = "over-current";
		break;
	case USB_PORT_FEAT_C_ENABLE:
		status = PORT_PEC;
		port_change_bit = "enable/disable";
		break;
	case USB_PORT_FEAT_C_SUSPEND:
		status = PORT_PLC;
		port_change_bit = "suspend/resume";
		break;
	default:
		/* Should never happen */
		return;
	}

	/* Change bits are all write 1 to clear */
	writel(port_status | status, addr);

	port_status = readl(addr);
	(void)port_change_bit;
	(void)wIndex;
	KprintfH("clear port %s change, actual port %ld status  = 0x%lx\n", port_change_bit, wIndex, port_status);
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
static u32 xhci_roothub_port_state_to_neutral(u32 state)
{
	/* Save read-only status and port state */
	return (state & XHCI_PORT_RO) | (state & XHCI_PORT_RWS);
}

void xhci_roothub_complete_int_request(struct xhci_root_hub *rh)
{
	if (!rh || !rh->int_req)
		return;

	struct xhci_ctrl *ctrl = rh->udev->controller;

	u8 *buffer = (u8 *)rh->int_req->iouh_Data;
	u32 buffer_len = rh->int_req->iouh_Length;
	if (!buffer || buffer_len == 0)
		return;

	const u32 actual_len = min(buffer_len, rh->ep1_mps);
	_memset(buffer, 0, buffer_len);

	const u8 num_ports = rh->descriptor.hub.bNbrPorts;
	const u8 need_bytes = (num_ports + 8U) / 8U;
	if (need_bytes > actual_len)
	{
		KprintfH("root hub status truncated: need %ld bytes, have %ld\n",
				 (LONG)need_bytes, (LONG)actual_len);
	}

	BOOL change = FALSE;
	const u32 change_mask = PORT_CSC | PORT_PEC | PORT_OCC | PORT_RC |
							PORT_PLC;
	for (u8 port = 0; port < num_ports; ++port)
	{
		u32 portsc = readl(&ctrl->hcor->portregs[port].or_portsc);
		u32 change_bits = portsc & change_mask;
		if (!change_bits)
			continue;

		u32 index = (port + 1) >> 3;
		if (index >= actual_len)
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

	KprintfH("completing root hub interrupt, actual=%ld, first byte=0x%02lx\n", (LONG)actual_len, buffer[0]);

	rh->io_reply_data(rh->udev, rh->int_req, UHIOERR_NO_ERROR, actual_len);
	rh->int_req = NULL;
}

/**
 * Submits the Requests to the XHCI Host Controller
 *
 * @param udev pointer to the USB device structure
 * @param io  pointer to the IOUsbHWReq structure
 */
void xhci_roothub_submit_ctrl_request(struct xhci_root_hub *rh, struct IOUsbHWReq *io)
{
	struct UsbSetupData *req = &io->iouh_SetupData;
	KprintfH("type=%02lx req=%02lx val=%04lx idx=%04lx len=%04lx\n",
			 (ULONG)req->bmRequestType, (ULONG)req->bRequest,
			 (ULONG)LE16(req->wValue), (ULONG)LE16(req->wIndex), (ULONG)LE16(req->wLength));

	u8 tmpbuf[4];
	void *srcptr = NULL;
	u32 reg;
	struct xhci_hcor *hcor = rh->udev->controller->hcor;

	if ((req->bmRequestType & USB_RT_PORT) && LE16(req->wIndex) > rh->descriptor.hub.bNbrPorts)
	{
		Kprintf("The request port(%ld) exceeds maximum port number\n", LE16(req->wIndex) - 1);
		goto unknown;
	}

	volatile u32 *status_reg = (volatile u32 *)(&hcor->portregs[LE16(req->wIndex) - 1].or_portsc);
	int srclen = 0;

	u16 typeReq = req->bRequest | req->bmRequestType << 8;
	switch (typeReq)
	{
	case DeviceRequest | USB_REQ_GET_DESCRIPTOR:
		switch (LE16(req->wValue) >> 8)
		{
		case USB_DT_DEVICE:
			KprintfH("get USB_DT_DEVICE\n");
			srcptr = &rh->descriptor.device;
			srclen = 0x12;
			break;
		case USB_DT_CONFIG:
			KprintfH("get USB_DT_CONFIG\n");
			srcptr = &rh->descriptor.config;
			srclen = LE16(rh->descriptor.config.wTotalLength);
			break;
		case USB_DT_STRING:
			KprintfH("get USB_DT_STRING\n");
			switch (LE16(req->wValue) & 0xff)
			{
			case 0: /* Language */
				srcptr = "\4\3\11\4";
				srclen = 4;
				break;
			case 1: /* Vendor String  */
				srcptr = "\20\3P\0i\0s\0t\0o\0r\0m\0";
				srclen = 16;
				break;
			case 2: /* Product Name */
				srcptr = "\52\3X\0H\0C\0I\0 "
						 "\0H\0o\0s\0t\0 "
						 "\0C\0o\0n\0t\0r\0o\0l\0l\0e\0r\0";
				srclen = 42;
				break;
			default:
				Kprintf("unknown value DT_STRING %lx\n",
						LE16(req->wValue));
				goto unknown;
			}
			break;
		default:
			Kprintf("get unknown value %lx\n", LE16(req->wValue));
			goto unknown;
		}
		break;
	case DeviceRequest | USB_REQ_GET_STATUS:
		/* Device GET_STATUS: bit0=self-powered, bit1=remote-wakeup */
		tmpbuf[0] = 0;
		tmpbuf[1] = 1; /* self-powered, remote-wakeup disabled */
		srcptr = tmpbuf;
		srclen = 2;
		KprintfH("USB_REQ_GET_STATUS\n");
		break;
	case USB_REQ_GET_DESCRIPTOR | ((USB_DIR_IN | USB_RT_HUB) << 8):
		switch (LE16(req->wValue) >> 8)
		{
		case USB_DT_HUB:
		case USB_DT_SS_HUB:
			KprintfH("get USB_DT_HUB config\n");
			srcptr = &rh->descriptor.hub;
			srclen = rh->descriptor.hub.bLength;
			break;
		default:
			Kprintf("get unknown value %lx\n", LE16(req->wValue));
			goto unknown;
		}
		break;
	case USB_REQ_SET_ADDRESS | (USB_RECIP_DEVICE << 8):
		KprintfH("USB_REQ_SET_ADDRESS rootdev=%ld\n", (LONG)LE16(req->wValue));
		/* Do nothing, higher layer will handle context migration */
		break;
	case DeviceOutRequest | USB_REQ_SET_CONFIGURATION:
		KprintfH("USB_REQ_SET_CONFIGURATION\n");
		/* Do nothing */
		break;
	case USB_REQ_GET_STATUS | ((USB_DIR_IN | USB_RT_HUB) << 8):
		tmpbuf[0] = 0;
		tmpbuf[1] = 1; /* self-powered, remote-wakeup disabled */
		srcptr = tmpbuf;
		srclen = 2;
		KprintfH("USB_REQ_GET_STATUS HUB\n");
		break;
	case USB_REQ_GET_STATUS | ((USB_RT_PORT | USB_DIR_IN) << 8):
		_memset(tmpbuf, 0, 4);
		reg = readl(status_reg);
		if (reg & PORT_CONNECT)
		{
			tmpbuf[0] |= USB_PORT_STAT_CONNECTION;
			switch (reg & DEV_SPEED_MASK)
			{
			case XDEV_FS:
				KprintfH("SPEED = FULLSPEED\n");
				break;
			case XDEV_LS:
				KprintfH("SPEED = LOWSPEED\n");
				tmpbuf[1] |= USB_PORT_STAT_LOW_SPEED >> 8;
				break;
			case XDEV_HS:
				KprintfH("SPEED = HIGHSPEED\n");
				tmpbuf[1] |= USB_PORT_STAT_HIGH_SPEED >> 8;
				break;
			case XDEV_SS:
				KprintfH("SPEED = SUPERSPEED\n");
				tmpbuf[1] |= USB_PORT_STAT_SUPER_SPEED >> 8;
				break;
			}
		}
		if (reg & PORT_PE)
			tmpbuf[0] |= USB_PORT_STAT_ENABLE;
		if ((reg & PORT_PLS_MASK) == XDEV_U3)
			tmpbuf[0] |= USB_PORT_STAT_SUSPEND;
		if (reg & PORT_OC)
			tmpbuf[0] |= USB_PORT_STAT_OVERCURRENT;
		if (reg & (PORT_RESET | PORT_WR))
			tmpbuf[0] |= USB_PORT_STAT_RESET;
		if (reg & PORT_POWER)
			/*
			 * XXX: This Port power bit (for USB 3.0 hub)
			 * we are faking in USB 2.0 hub port status;
			 * since there's a change in bit positions in
			 * two:
			 * USB 2.0 port status PP is at position[8]
			 * USB 3.0 port status PP is at position[9]
			 * So, we are still keeping it at position [8]
			 */
			tmpbuf[1] |= USB_PORT_STAT_POWER >> 8;
		if (reg & PORT_CSC)
			tmpbuf[2] |= USB_PORT_STAT_C_CONNECTION;
		if (reg & PORT_PEC)
			tmpbuf[2] |= USB_PORT_STAT_C_ENABLE;
		if (reg & PORT_OCC)
			tmpbuf[2] |= USB_PORT_STAT_C_OVERCURRENT;
		if (reg & (PORT_RC | PORT_WRC))
			tmpbuf[2] |= USB_PORT_STAT_C_RESET;

		srcptr = tmpbuf;
		srclen = 4;
		KprintfH("USB_REQ_GET_STATUS PORT %ld status=0x%lx\n",
				 LE16(req->wIndex) - 1, reg);
		break;
	case USB_REQ_SET_FEATURE | ((USB_DIR_OUT | USB_RT_PORT) << 8):
		reg = readl(status_reg);
		reg = xhci_roothub_port_state_to_neutral(reg);
		KprintfH("SET_FEATURE PORT %ld status=0x%lx feature=0x%lx\n",
				 LE16(req->wIndex) - 1, reg, LE16(req->wValue));
		switch (LE16(req->wValue))
		{
		case USB_PORT_FEAT_ENABLE:
			KprintfH("Set PORT_PE\n");
			reg |= PORT_PE;
			writel(reg, status_reg);
			break;
		case USB_PORT_FEAT_POWER:
			KprintfH("Set PORT_POWER\n");
			reg |= PORT_POWER;
			writel(reg, status_reg);
			break;
		case USB_PORT_FEAT_RESET:
			if ((reg & DEV_SPEED_MASK) == XDEV_SS)
			{
				KprintfH("Set PORT_WR (warm reset)\n");
				reg |= PORT_WR;
			}
			else
			{
				KprintfH("Set PORT_RESET\n");
				reg |= PORT_RESET;
			}
			writel(reg, status_reg);
			break;
		case USB_PORT_FEAT_SUSPEND:
			/* Put link into U3 suspend */
			KprintfH("Putting link to U3 standby\n");
			reg &= ~PORT_PLS_MASK;
			reg |= XDEV_U3;
			writel(reg, status_reg);
			break;
		default:
			Kprintf("unknown feature %lx\n", LE16(req->wValue));
			goto unknown;
		}
		break;
	case USB_REQ_CLEAR_FEATURE | ((USB_DIR_OUT | USB_RT_PORT) << 8):
		reg = readl(status_reg);
		reg = xhci_roothub_port_state_to_neutral(reg);
		KprintfH("CLEAR_FEATURE PORT %ld status=0x%lx feature=0x%lx\n",
				 LE16(req->wIndex) - 1, reg, LE16(req->wValue));
		switch (LE16(req->wValue))
		{
		case USB_PORT_FEAT_ENABLE:
			KprintfH("Clear PORT_PE\n");
			/* PED is RW1CS: write a '1' to disable/clear it. */
			reg |= PORT_PE;
			break;
		case USB_PORT_FEAT_POWER:
			KprintfH("Clear PORT_POWER\n");
			reg &= ~PORT_POWER;
			break;
		case USB_PORT_FEAT_SUSPEND:
			KprintfH("Resume from suspend, put link to U0\n");
			/* Resume link to U0 */
			reg &= ~PORT_PLS_MASK;
			reg |= XDEV_U0;
			break;
		case USB_PORT_FEAT_C_CONNECTION:
		case USB_PORT_FEAT_C_ENABLE:
		case USB_PORT_FEAT_C_SUSPEND:
		case USB_PORT_FEAT_C_OVER_CURRENT:
		case USB_PORT_FEAT_C_RESET:
			KprintfH("Clear port feat 0x%lx change\n", LE16(req->wValue));
			xhci_roothub_clear_port_change_bit((LE16(req->wValue)),
											   LE16(req->wIndex),
											   status_reg, reg);
			break;
		default:
			Kprintf("Unknown feature 0x%lx\n", LE16(req->wValue));
			goto unknown;
		}
		writel(reg, status_reg);
		break;
	default:
		Kprintf("Unknown request\n");
		goto unknown;
	}

	KprintfH("scrlen = %ld req->length = %ld\n",
			 srclen, LE16(req->wLength));

	int len = min(srclen, (int)LE16(req->wLength));
	if (srcptr != NULL && len > 0)
		CopyMem(srcptr, io->iouh_Data, len);

	io->iouh_Actual = len;
	io->iouh_Req.io_Error = UHIOERR_NO_ERROR;
	return;

unknown:
	io->iouh_Actual = 0;
	io->iouh_Req.io_Error = UHIOERR_STALL;
	return;
}
