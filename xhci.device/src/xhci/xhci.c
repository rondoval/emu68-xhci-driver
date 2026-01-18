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

#include <exec/memory.h>

#include <compat.h>
#include <debug.h>
#include <device.h>

#include <xhci/usb.h>
#include <usb_glue.h>
#include <devices/usbhardware.h>
#include <xhci/xhci.h>
#include <xhci/xhci-commands.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

static struct descriptor
{
	struct usb_hub_descriptor hub;
	struct usb_device_descriptor device;
	struct usb_config_descriptor config;
	struct usb_interface_descriptor interface;
	struct usb_endpoint_descriptor endpoint;
} __attribute__((packed)) descriptor = {
	.hub = {
		.bLength            = 0x0b,                  /* placeholder, recalculated once ports known */
		.bDescriptorType    = USB_DT_HUB,            /* hub descriptor */
		.bNbrPorts          = 2,                     /* patched to real port count during init */
		.wHubCharacteristics= cpu_to_le16(HUB_CHAR_INDV_PORT_LPSM |
						 HUB_CHAR_INDV_PORT_OCPM), /* per-port power + OC */
		.bPwrOn2PwrGood     = 10,                    /* 20 ms between power on and usable */
		.bHubContrCurrent   = 0,                     /* self-powered: no bus draw */
		.u.hs = {
			.DeviceRemovable   = {0x00, 0x00},         /* all ports permanently wired */
			.PortPowerCtrlMask = {0x00, 0x00},         /* patched in init when ports known */
		},
	},
	.device = {
		.bLength            = sizeof(struct usb_device_descriptor), /* size of device descriptor */
		.bDescriptorType    = USB_DT_DEVICE,         /* device descriptor */
		.bcdUSB             = cpu_to_le16(0x0200),   /* advertise as USB 2.0 */
		.bDeviceClass       = USB_CLASS_HUB,         /* hub */
		.bDeviceSubClass    = 0,                     /* no subclass */
		.bDeviceProtocol    = USB_HUB_PR_HS_SINGLE_TT, /* single-TT */
		.bMaxPacketSize0    = 64,                    /* control endpoint max packet */
		.idVendor           = 0x0000,                /* virtual root hub: leave VID zero */
		.idProduct          = 0x0000,                /* virtual root hub: leave PID zero */
		.bcdDevice          = cpu_to_le16(0x0100),   /* device revision */
		.iManufacturer      = 1,                     /* string index */
		.iProduct           = 2,                     /* string index */
		.iSerialNumber      = 0,                     /* no serial */
		.bNumConfigurations = 1,                     /* single configuration */
	},
	.config = {
		.bLength            = sizeof(struct usb_config_descriptor), /* size of configuration descriptor */
		.bDescriptorType    = USB_DT_CONFIG,         /* configuration descriptor */
		.wTotalLength       = cpu_to_le16(sizeof(struct usb_config_descriptor) +
					 sizeof(struct usb_interface_descriptor) +
					 sizeof(struct usb_endpoint_descriptor)), /* config + interface + endpoint */
		.bNumInterfaces     = 1,                     /* single interface */
		.bConfigurationValue= 1,                     /* configuration ID */
		.iConfiguration     = 0,                     /* no string descriptor */
		.bmAttributes       = USB_CONFIG_ATT_ONE | USB_CONFIG_ATT_SELFPOWER, /* must-set + self-powered */
		.bMaxPower          = 0,                     /* no bus power drawn */
	},
	.interface = {
		.bLength            = sizeof(struct usb_interface_descriptor), /* size of interface descriptor */
		.bDescriptorType    = USB_DT_INTERFACE,      /* interface descriptor */
		.bInterfaceNumber   = 0,                     /* interface index */
		.bAlternateSetting  = 0,                     /* only one setting */
		.bNumEndpoints      = 1,                     /* interrupt endpoint only */
		.bInterfaceClass    = USB_CLASS_HUB,         /* hub functional interface */
		.bInterfaceSubClass = 0,                     /* full/high-speed hub */
		.bInterfaceProtocol = USB_HUB_PR_HS_SINGLE_TT, /* single-TT */
		.iInterface         = 0,                     /* no string */
	},
	.endpoint = {
		.bLength            = sizeof(struct usb_endpoint_descriptor), /* size of endpoint descriptor */
		.bDescriptorType    = USB_DT_ENDPOINT,       /* endpoint descriptor */
		.bEndpointAddress   = (USB_DIR_IN | 1),      /* INT IN endpoint 1 */
		.bmAttributes       = USB_ENDPOINT_XFER_INT, /* interrupt */
		.wMaxPacketSize     = cpu_to_le16(8),        /* patched during init */
		.bInterval          = 12,                    /* 1ms poll per USB 2.0 hub spec */
		.bRefresh           = 0,                     /* unused for INT */
		.bSynchAddress      = 0,                     /* not used */
	},
};

dma_addr_t xhci_dma_map(struct xhci_ctrl *ctrl, void *addr, size_t size)
{
	// TODO this is wrong, move mapping up, do it on Poseidon buffers in command handling
	if (!ctrl || !addr || size == 0)
		return (dma_addr_t)(uintptr_t)addr;

	if ((((uintptr_t)addr) & ARCH_DMA_MINALIGN_MASK) == 0)
		return (dma_addr_t)(uintptr_t)addr;

	if (!ctrl->memoryPool)
		return (dma_addr_t)(uintptr_t)addr;

	size_t alloc_len = ALIGN(size, ARCH_DMA_MINALIGN);
	struct xhci_dma_bounce *bounce = AllocVecPooled(ctrl->memoryPool, sizeof(*bounce));
	if (!bounce)
	{
		Kprintf("failed to allocate metadata for %lx len=%ld\n", (ULONG)addr, (LONG)size);
		return (dma_addr_t)(uintptr_t)addr;
	}

	void *aligned = memalign(ctrl->memoryPool, ARCH_DMA_MINALIGN, alloc_len);
	if (!aligned)
	{
		Kprintf("failed to allocate bounce buffer for %lx len=%ld\n", (ULONG)addr, (LONG)size);
		FreeVecPooled(ctrl->memoryPool, bounce);
		return (dma_addr_t)(uintptr_t)addr;
	}

	CopyMem(addr, aligned, size);
	xhci_flush_cache((uintptr_t)aligned, alloc_len);

	bounce->orig = addr;
	bounce->bounce = aligned;
	bounce->size = size;
	bounce->alloc_len = alloc_len;
	bounce->next = ctrl->dma_bounce_list;
	ctrl->dma_bounce_list = bounce;

	return (dma_addr_t)(uintptr_t)aligned;
}

void xhci_dma_unmap(struct xhci_ctrl *ctrl, dma_addr_t addr, size_t size)
{
	if (!ctrl || addr == 0)
		return;

	(void)size;

	struct xhci_dma_bounce **link = &ctrl->dma_bounce_list;
	while (*link)
	{
		struct xhci_dma_bounce *node = *link;
		if ((dma_addr_t)(uintptr_t)node->orig == addr)
		{
			xhci_inval_cache((uintptr_t)node->bounce, node->alloc_len);
			if (node->size)
				CopyMem(node->bounce, node->orig, node->size);
			memalign_free(ctrl->memoryPool, node->bounce);
			*link = node->next;
			FreeVecPooled(ctrl->memoryPool, node);
			return;
		}
		link = &(*link)->next;
	}
}

/**
 * Waits for as per specified amount of time
 * for the "result" to match with "done"
 *
 * @param ptr	pointer to the register to be read
 * @param mask	mask for the value read
 * @param done	value to be campared with result
 * @param usec	time to wait till
 * Return: 0 if handshake is success else < 0 on failure
 */
static int handshake(uint32_t volatile *ptr, uint32_t mask, uint32_t done, int usec)
{
	uint32_t result;
	int ret;

	// TODO fixed value ULONG_MAX
	ret = readx_poll_timeout(xhci_readl, ptr, result, (result & mask) == done || result == 0xffffffff, usec);
	if (result == 0xffffffff) /* card removed */
		return -ENODEV;

	return ret;
}

/**
 * Set the run bit and wait for the host to be running.
 *
 * @param hcor	pointer to host controller operation registers
 * Return: status of the Handshake
 */
static int xhci_start(struct xhci_hcor *hcor)
{
	u32 temp;
	int ret;

	Kprintf("Starting the controller\n");
	temp = xhci_readl(&hcor->or_usbcmd);
	temp |= (CMD_RUN);
	xhci_writel(&hcor->or_usbcmd, temp);

	/*
	 * Wait for the HCHalted Status bit to be 0 to indicate the host is
	 * running.
	 */
	ret = handshake(&hcor->or_usbsts, STS_HALT, 0, XHCI_MAX_HALT_USEC);
	if (ret)
		Kprintf("Host took too long to start, waited %lu microseconds.\n", XHCI_MAX_HALT_USEC);
	return ret;
}

/**
 * Resets the XHCI Controller
 *
 * @param hcor	pointer to host controller operation registers
 * Return: -EBUSY if XHCI Controller is not halted else status of handshake
 */
static int xhci_reset(struct xhci_hcor *hcor)
{
	u32 cmd;
	u32 state;
	int ret;

	/* Halting the Host first */
	Kprintf("// Halt the HC: %lx\n", hcor);
	state = xhci_readl(&hcor->or_usbsts) & STS_HALT;
	if (!state)
	{
		cmd = xhci_readl(&hcor->or_usbcmd);
		cmd &= ~CMD_RUN;
		xhci_writel(&hcor->or_usbcmd, cmd);
	}

	ret = handshake(&hcor->or_usbsts,
					STS_HALT, STS_HALT, XHCI_MAX_HALT_USEC);
	if (ret)
	{
		Kprintf("Host not halted after %lu microseconds.\n", XHCI_MAX_HALT_USEC);
		return -EBUSY;
	}

	Kprintf("// Reset the HC\n");
	cmd = xhci_readl(&hcor->or_usbcmd);
	cmd |= CMD_RESET_USB;
	xhci_writel(&hcor->or_usbcmd, cmd);

	ret = handshake(&hcor->or_usbcmd, CMD_RESET_USB, 0, XHCI_MAX_RESET_USEC);
	if (ret)
		return ret;

	/*
	 * xHCI cannot write to any doorbells or operational registers other
	 * than status until the "Controller Not Ready" flag is cleared.
	 */
	return handshake(&hcor->or_usbsts, STS_CNR, 0, XHCI_MAX_RESET_USEC);
}

/**
 * Used for passing endpoint bitmasks between the core and HCDs.
 * Find the index for an endpoint given its descriptor.
 * Use the return value to right shift 1 for the bitmask.
 *
 * Index  = (epnum * 2) + direction - 1,
 * where direction = 0 for OUT, 1 for IN.
 * For control endpoints, the IN index is used (OUT index is unused), so
 * index = (epnum * 2) + direction - 1 = (epnum * 2) + 1 - 1 = (epnum * 2)
 *
 * @param desc	USB enpdoint Descriptor
 * Return: index of the Endpoint
 */
static unsigned int xhci_get_ep_index(struct usb_endpoint_descriptor *desc)
{
	unsigned int index;

	if (usb_endpoint_xfer_control(desc))
		index = (unsigned int)(usb_endpoint_num(desc) * 2);
	else
		index = (unsigned int)((usb_endpoint_num(desc) * 2) -
							   (usb_endpoint_dir_in(desc) ? 0 : 1));

	return index;
}

static struct usb_interface *xhci_find_interface(struct usb_config *cfg, unsigned int iface_number)
{
	if (!cfg)
		return NULL;

	for (unsigned int i = 0; i < cfg->no_of_if; ++i)
	{
		if (cfg->if_desc[i].interface_number == iface_number)
			return &cfg->if_desc[i];
	}

	return NULL;
}

static struct usb_interface_altsetting *xhci_find_altsetting(struct usb_interface *iface, unsigned int alt_setting)
{
	if (!iface)
		return NULL;

	for (unsigned int idx = 0; idx < iface->num_altsetting; ++idx)
	{
		struct usb_interface_altsetting *candidate = &iface->altsetting[idx];
		if (candidate->desc.bAlternateSetting == alt_setting)
			return candidate;
	}

	return NULL;
}

static u32 xhci_collect_ep_mask(struct xhci_virt_device *virt_dev,
								const struct usb_interface_altsetting *alt,
								int reset_state,
								unsigned int *max_flag)
{
	if (!alt)
		return 0;

	u32 mask = 0;

	for (unsigned int i = 0; i < alt->no_of_ep; ++i)
	{
		const struct usb_endpoint_descriptor *epd = &alt->ep_desc[i];
		unsigned int ep_index = xhci_get_ep_index((struct usb_endpoint_descriptor *)epd);
		mask |= 1U << (ep_index + 1);
		if (reset_state && virt_dev)
			virt_dev->eps[ep_index].ep_state = 0;
		if (max_flag && ep_index > *max_flag)
			*max_flag = ep_index;
	}

	return mask;
}

static u32 xhci_collect_config_masks(const struct usb_config *cfg,
									 unsigned int limit,
									 unsigned int *max_flag,
									 int log_prepare)
{
	if (!cfg)
		return 0;

	u32 mask = 0;
	unsigned int max_if = min(limit, cfg->no_of_if);

	for (unsigned int ifnum = 0; ifnum < max_if; ++ifnum)
	{
		const struct usb_interface *iface = &cfg->if_desc[ifnum];
		const struct usb_interface_altsetting *active_alt = iface->active_altsetting;
		if (!active_alt)
			continue;

		if (log_prepare)
		{
			KprintfH("Preparing iface=%ld alt=%ld\n",
					 (ULONG)ifnum, (LONG)active_alt->desc.bAlternateSetting);
		}

		mask |= xhci_collect_ep_mask(NULL, active_alt, 0, max_flag);
	}

	return mask;
}

static struct usb_config *xhci_find_config(struct usb_device *udev, int config_value)
{
	if (!udev)
		return NULL;

	for (struct MinNode *node = udev->configurations.mlh_Head; node->mln_Succ != NULL; node = node->mln_Succ)
	{
		struct usb_config *cfg = (struct usb_config *)node;
		KprintfH("  found config: bConfigurationValue=%ld\n", (LONG)cfg->desc.bConfigurationValue);
		if (cfg->desc.bConfigurationValue == config_value)
			return cfg;
	}

	return NULL;
}

static struct usb_interface_altsetting *xhci_select_active_alt(struct usb_interface *iface)
{
	if (!iface)
		return NULL;

	if (iface->active_altsetting)
		return iface->active_altsetting;

	if (iface->num_altsetting == 0)
		return NULL;

	struct usb_interface_altsetting *fallback = NULL;
	for (unsigned int idx = 0; idx < iface->num_altsetting; ++idx)
	{
		struct usb_interface_altsetting *candidate = &iface->altsetting[idx];
		if (candidate->desc.bAlternateSetting == 0)
		{
			fallback = candidate;
			break;
		}
	}

	if (!fallback)
		fallback = &iface->altsetting[0];

	iface->active_altsetting = fallback;
	return fallback;
}

#ifdef DEBUG_HIGH
static void xhci_dump_interface(unsigned int index, const struct usb_interface *iface)
{
	if (!iface)
		return;

	const struct usb_interface_altsetting *active_alt = iface->active_altsetting;
	const struct usb_interface_altsetting *desc_alt = active_alt;
	if (!desc_alt && iface->num_altsetting > 0)
		desc_alt = &iface->altsetting[0];

	Kprintf("  Interface %ld:\n", (ULONG)index);
	if (desc_alt)
	{
		Kprintf("    bLength=%ld bDescriptorType=%ld bInterfaceNumber=%ld bAlternateSetting=%ld\n",
				(ULONG)desc_alt->desc.bLength, (ULONG)desc_alt->desc.bDescriptorType,
				(ULONG)desc_alt->desc.bInterfaceNumber, (ULONG)desc_alt->desc.bAlternateSetting);
		Kprintf("    bNumEndpoints=%ld bInterfaceClass=%ld bInterfaceSubClass=%ld bInterfaceProtocol=%ld\n",
				(ULONG)desc_alt->desc.bNumEndpoints, (ULONG)desc_alt->desc.bInterfaceClass,
				(ULONG)desc_alt->desc.bInterfaceSubClass, (ULONG)desc_alt->desc.bInterfaceProtocol);
		Kprintf("    iInterface=%ld no_of_ep=%ld\n",
				(ULONG)desc_alt->desc.iInterface,
				(ULONG)(active_alt ? active_alt->no_of_ep : desc_alt->no_of_ep));
	}
	else
	{
		Kprintf("    (no descriptors captured for this interface)\n");
	}

	if (!active_alt)
	{
		Kprintf("    (no active alternate setting)\n");
		return;
	}

	for (unsigned int j = 0; j < active_alt->no_of_ep; ++j)
	{
		const struct usb_endpoint_descriptor *ep = &active_alt->ep_desc[j];
		const struct usb_ss_ep_comp_descriptor *ss_ep = &active_alt->ss_ep_comp_desc[j];
		Kprintf("    Endpoint %ld:\n", (ULONG)j);
		Kprintf("      bLength=%ld bDescriptorType=%ld bEndpointAddress=0x%02lx bmAttributes=0x%02lx\n",
				(ULONG)ep->bLength, (ULONG)ep->bDescriptorType,
				(ULONG)ep->bEndpointAddress, (ULONG)ep->bmAttributes);
		Kprintf("      wMaxPacketSize=%ld bInterval=%ld\n",
				(ULONG)LE16(ep->wMaxPacketSize), (ULONG)ep->bInterval);
		Kprintf("      SS Companion: bLength=%ld bDescriptorType=%ld bMaxBurst=%ld bmAttributes=0x%02lx\n",
				(ULONG)ss_ep->bLength, (ULONG)ss_ep->bDescriptorType,
				(ULONG)ss_ep->bMaxBurst, (ULONG)ss_ep->bmAttributes);
		Kprintf("      SS Companion: wBytesPerInterval=%ld\n",
				(ULONG)LE16(ss_ep->wBytesPerInterval));
	}
}

static void xhci_dump_config(const struct usb_config *cfg, LONG devnum)
{
	if (!cfg)
		return;

	Kprintf("Addr %ld configuration dump:\n", devnum);
	Kprintf("  bLength=%ld bDescriptorType=%ld wTotalLength=%ld bNumInterfaces=%ld\n",
			(ULONG)cfg->desc.bLength, (ULONG)cfg->desc.bDescriptorType,
			(ULONG)LE16(cfg->desc.wTotalLength), (ULONG)cfg->desc.bNumInterfaces);
	Kprintf("  bConfigurationValue=%ld iConfiguration=%ld bmAttributes=0x%02lx bMaxPower=%ld\n",
			(ULONG)cfg->desc.bConfigurationValue, (ULONG)cfg->desc.iConfiguration,
			(ULONG)cfg->desc.bmAttributes, (ULONG)cfg->desc.bMaxPower);
	Kprintf("  no_of_if=%ld\n", (ULONG)cfg->no_of_if);

	for (unsigned int i = 0; i < cfg->no_of_if; ++i)
		xhci_dump_interface(i, &cfg->if_desc[i]);
}
#endif

static int xhci_init_ep_contexts_if(struct usb_device *udev,
									struct xhci_ctrl *ctrl,
									struct xhci_virt_device *virt_dev,
									struct usb_interface *ifdesc);

static void xhci_update_slot_last_ctx(struct xhci_ctrl *ctrl,
									  struct xhci_virt_device *virt_dev,
									  unsigned int max_ep_flag)
{
	if (!ctrl || !virt_dev)
		return;

	struct xhci_slot_ctx *slot_ctx = xhci_get_slot_ctx(ctrl, virt_dev->in_ctx);
	if (!slot_ctx)
		return;

	u32 dev_info = LE32(slot_ctx->dev_info);
	dev_info &= ~LAST_CTX_MASK;
	dev_info |= LAST_CTX(max_ep_flag + 1);
	slot_ctx->dev_info = LE32(dev_info);
}

static unsigned int compute_max_ep_flag(const struct usb_config *cfg)
{
	if (!cfg)
		return 0;

	unsigned int max_flag = 0;
	xhci_collect_config_masks(cfg, cfg->no_of_if, &max_flag, 0);

	return max_flag;
}

int xhci_set_interface(struct usb_device *udev, unsigned int iface_number, unsigned int alt_setting)
{
	if (!udev || !udev->controller)
	{
		Kprintf("xhci_set_interface: invalid usb_device pointer\n");
		return UHIOERR_BADPARAMS;
	}

	struct usb_config *cfg = udev->active_config;
	if (!cfg)
	{
		Kprintf("xhci_set_interface: no active config for addr %ld\n", (LONG)udev->poseidon_address);
		return UHIOERR_BADPARAMS;
	}

	struct usb_interface *iface = xhci_find_interface(cfg, iface_number);
	if (!iface)
	{
		Kprintf("xhci_set_interface: interface %ld not found in config %ld\n",
				(LONG)iface_number, (LONG)cfg->desc.bConfigurationValue);
		return UHIOERR_BADPARAMS;
	}

	struct usb_interface_altsetting *current_alt = iface->active_altsetting;

	if (current_alt && current_alt->desc.bAlternateSetting == alt_setting)
	{
		KprintfH("xhci_set_interface: iface %ld already at alt %ld\n",
				 (LONG)iface_number, (LONG)alt_setting);
		return UHIOERR_NO_ERROR;
	}

	struct usb_interface_altsetting *new_alt = xhci_find_altsetting(iface, alt_setting);
	if (!new_alt)
	{
		Kprintf("xhci_set_interface: alt %ld missing for iface %ld\n",
				(LONG)alt_setting, (LONG)iface_number);
		return UHIOERR_BADPARAMS;
	}

	struct xhci_ctrl *ctrl = udev->controller;
	struct xhci_virt_device *virt_dev = ctrl->devs[udev->slot_id];
	if (!virt_dev || !virt_dev->in_ctx || !virt_dev->out_ctx)
	{
		Kprintf("xhci_set_interface: missing virtual device for slot %ld\n", (LONG)udev->slot_id);
		return UHIOERR_HOSTERROR;
	}

	/* Poseidon stack guarantees endpoint queues are idle before switching. */
	u32 drop_mask = xhci_collect_ep_mask(virt_dev, current_alt, 1, NULL);

	iface->active_altsetting = new_alt;

	u32 add_mask = xhci_collect_ep_mask(NULL, new_alt, 0, NULL);

	xhci_inval_cache((uintptr_t)virt_dev->out_ctx->bytes, virt_dev->out_ctx->size);
	xhci_slot_copy(ctrl, virt_dev->in_ctx, virt_dev->out_ctx);
	xhci_endpoint_copy(ctrl, virt_dev->in_ctx, virt_dev->out_ctx, 0);

	struct xhci_input_control_ctx *ctrl_ctx = xhci_get_input_control_ctx(virt_dev->in_ctx);
	if (!ctrl_ctx)
	{
		Kprintf("xhci_set_interface: missing input control context\n");
		iface->active_altsetting = current_alt;
		return UHIOERR_HOSTERROR;
	}

	int err = UHIOERR_NO_ERROR;
	if (new_alt->no_of_ep > 0)
	{
		err = xhci_init_ep_contexts_if(udev, ctrl, virt_dev, iface);
		if (err != UHIOERR_NO_ERROR)
		{
			Kprintf("xhci_set_interface: failed to init ep contexts (err=%ld)\n", (LONG)err);
			iface->active_altsetting = current_alt;
			return err;
		}
	}

	u32 add_flags = SLOT_FLAG | add_mask;
	u32 drop_flags = drop_mask;
	ctrl_ctx->add_flags = LE32(add_flags);
	ctrl_ctx->drop_flags = LE32(drop_flags);

	unsigned int max_ep_flag = compute_max_ep_flag(cfg);
	xhci_update_slot_last_ctx(ctrl, virt_dev, max_ep_flag);

	KprintfH("xhci_set_interface: updating device context for addr=%ld iface=%ld alt=%ld drop=0x%lx add=0x%lx\n",
			 (LONG)udev->poseidon_address,
			 (LONG)iface_number,
			 (LONG)alt_setting,
			 (ULONG)drop_mask,
			 (ULONG)add_mask);

	xhci_configure_endpoints(udev, FALSE, NULL);

	return err;
}

/*
 * Convert bInterval expressed in microframes (in 1-255 range) to exponent of
 * microframes, rounded down to nearest power of 2.
 */
static unsigned int xhci_microframes_to_exponent(unsigned int desc_interval,
												 unsigned int min_exponent,
												 unsigned int max_exponent)
{
	unsigned int interval;

	interval = fls(desc_interval) - 1;
	interval = clamp_val(interval, min_exponent, max_exponent);
#ifdef DEBUG_HIGH
	if ((1U << interval) != desc_interval)
		KprintfH("rounding interval to %ld microframes, ep desc says %ld microframes\n",
				1U << interval, desc_interval);
#endif

	return interval;
}

static unsigned int xhci_parse_microframe_interval(struct usb_endpoint_descriptor *endpt_desc)
{
	if (endpt_desc->bInterval == 0)
		return 0;

	return xhci_microframes_to_exponent(endpt_desc->bInterval, 0, 15);
}

static unsigned int xhci_parse_frame_interval(struct usb_endpoint_descriptor *endpt_desc)
{
	return xhci_microframes_to_exponent(endpt_desc->bInterval * 8, 3, 10);
}

/*
 * Convert interval expressed as 2^(bInterval - 1) == interval into
 * straight exponent value 2^n == interval.
 */
static unsigned int xhci_parse_exponent_interval(struct usb_device *udev,
												 struct usb_endpoint_descriptor *endpt_desc)
{
	unsigned int interval;

	interval = clamp_val(endpt_desc->bInterval, 1, 16) - 1;
	if (interval != endpt_desc->bInterval - 1U)
		Kprintf("ep %#lx - rounding interval to %ld %sframes\n",
				endpt_desc->bEndpointAddress, 1 << interval,
				udev->speed == USB_SPEED_FULL ? "" : "micro");

	if (udev->speed == USB_SPEED_FULL)
	{
		/*
		 * Full speed isoc endpoints specify interval in frames,
		 * not microframes. We are using microframes everywhere,
		 * so adjust accordingly.
		 */
		interval += 3; /* 1 frame = 2^3 uframes */
	}

	return interval;
}

/*
 * Return the polling or NAK interval.
 *
 * The polling interval is expressed in "microframes". If xHCI's Interval field
 * is set to N, it will service the endpoint every 2^(Interval)*125us.
 *
 * The NAK interval is one NAK per 1 to 255 microframes, or no NAKs if interval
 * is set to 0.
 */
static unsigned int xhci_get_endpoint_interval(struct usb_device *udev,
											   struct usb_endpoint_descriptor *endpt_desc)
{
	unsigned int interval = 0;

	switch (udev->speed)
	{
	case USB_SPEED_HIGH:
		/* Max NAK rate */
		if (usb_endpoint_xfer_control(endpt_desc) ||
			usb_endpoint_xfer_bulk(endpt_desc))
		{
			interval = xhci_parse_microframe_interval(endpt_desc);
			break;
		}
		/* Fall through - SS and HS isoc/int have same decoding */

	case USB_SPEED_SUPER:
		if (usb_endpoint_xfer_int(endpt_desc) ||
			usb_endpoint_xfer_isoc(endpt_desc))
		{
			interval = xhci_parse_exponent_interval(udev,
													endpt_desc);
		}
		break;

	case USB_SPEED_FULL:
		if (usb_endpoint_xfer_isoc(endpt_desc))
		{
			interval = xhci_parse_exponent_interval(udev,
													endpt_desc);
			break;
		}
		/*
		 * Fall through for interrupt endpoint interval decoding
		 * since it uses the same rules as low speed interrupt
		 * endpoints.
		 */
		/*fallthrough;*/
	case USB_SPEED_LOW:
		if (usb_endpoint_xfer_int(endpt_desc) ||
			usb_endpoint_xfer_isoc(endpt_desc))
		{
			interval = xhci_parse_frame_interval(endpt_desc);
		}
		break;

	default:
		Kprintf("Unsupported USB speed: %ld\n", udev->speed);
	}

	return interval;
}

/*
 * The "Mult" field in the endpoint context is only set for SuperSpeed isoc eps.
 * High speed endpoint descriptors can define "the number of additional
 * transaction opportunities per microframe", but that goes in the Max Burst
 * endpoint context field.
 */
static u32 xhci_get_endpoint_mult(struct usb_device *udev,
								  struct usb_endpoint_descriptor *endpt_desc,
								  struct usb_ss_ep_comp_descriptor *ss_ep_comp_desc)
{
	if (udev->speed < USB_SPEED_SUPER || !usb_endpoint_xfer_isoc(endpt_desc))
		return 0;

	return ss_ep_comp_desc->bmAttributes;
}

static u32 xhci_get_endpoint_max_burst(struct usb_device *udev,
									   struct usb_endpoint_descriptor *endpt_desc,
									   struct usb_ss_ep_comp_descriptor *ss_ep_comp_desc)
{
	/* Super speed and Plus have max burst in ep companion desc */
	if (udev->speed >= USB_SPEED_SUPER)
		return ss_ep_comp_desc->bMaxBurst;

	if (udev->speed == USB_SPEED_HIGH && (usb_endpoint_xfer_isoc(endpt_desc) || usb_endpoint_xfer_int(endpt_desc)))
		return usb_endpoint_maxp_mult(endpt_desc) - 1;

	return 0;
}

/*
 * Return the maximum endpoint service interval time (ESIT) payload.
 * Basically, this is the maxpacket size, multiplied by the burst size
 * and mult size.
 */
static u32 xhci_get_max_esit_payload(struct usb_device *udev,
									 struct usb_endpoint_descriptor *endpt_desc,
									 struct usb_ss_ep_comp_descriptor *ss_ep_comp_desc)
{
	int max_burst;
	int max_packet;

	/* Only applies for interrupt or isochronous endpoints */
	if (usb_endpoint_xfer_control(endpt_desc) || usb_endpoint_xfer_bulk(endpt_desc))
		return 0;

	/* SuperSpeed Isoc ep with less than 48k per esit */
	if (udev->speed >= USB_SPEED_SUPER)
		return LE16(ss_ep_comp_desc->wBytesPerInterval);

	max_packet = usb_endpoint_maxp(endpt_desc);
	max_burst = usb_endpoint_maxp_mult(endpt_desc);

	/* A 0 in max burst means 1 transfer per ESIT */
	return max_packet * max_burst;
}

/**
 * Fill endpoint contexts for interface descriptor ifdesc.
 *
 * @param udev		pointer to the USB device structure
 * @param ctrl		pointer to the xhci pravte device structure
 * @param virt_dev	pointer to the xhci virtual device structure
 * @param ifdesc	pointer to the USB interface config descriptor
 * Return: returns the status of xhci_init_ep_contexts_if
 */
static int xhci_init_ep_contexts_if(struct usb_device *udev,
									struct xhci_ctrl *ctrl,
									struct xhci_virt_device *virt_dev,
									struct usb_interface *ifdesc)
{
	KprintfH("xhci_init_ep_contexts_if: enter\n");
	struct xhci_ep_ctx *ep_ctx[MAX_EP_CTX_NUM];
	int cur_ep;
	int ep_index;
	unsigned int dir;
	unsigned int ep_type;
	u64 trb_64 = 0;
	u32 max_esit_payload;
	unsigned int interval;
	unsigned int mult;
	unsigned int max_burst;
	unsigned int avg_trb_len;
	unsigned int err_count = 0;
	struct usb_interface_altsetting *active_alt = ifdesc->active_altsetting;
	if (!active_alt)
	{
		Kprintf("xhci_init_ep_contexts_if: no active altsetting for iface %ld\n",
				(ULONG)ifdesc->interface_number);
		return UHIOERR_BADPARAMS;
	}

	int num_of_ep = active_alt->no_of_ep;

	for (cur_ep = 0; cur_ep < num_of_ep; cur_ep++)
	{
		struct usb_endpoint_descriptor *endpt_desc = NULL;
		struct usb_ss_ep_comp_descriptor *ss_ep_comp_desc = NULL;

		endpt_desc = &active_alt->ep_desc[cur_ep];
		ss_ep_comp_desc = &active_alt->ss_ep_comp_desc[cur_ep];
		trb_64 = 0;

		/*
		 * Get values to fill the endpoint context, mostly from ep
		 * descriptor. The average TRB buffer lengt for bulk endpoints
		 * is unclear as we have no clue on scatter gather list entry
		 * size. For Isoc and Int, set it to max available.
		 * See xHCI 1.1 spec 4.14.1.1 for details.
		 */
		max_esit_payload = xhci_get_max_esit_payload(udev, endpt_desc, ss_ep_comp_desc);
		interval = xhci_get_endpoint_interval(udev, endpt_desc);
		mult = xhci_get_endpoint_mult(udev, endpt_desc, ss_ep_comp_desc);
		max_burst = xhci_get_endpoint_max_burst(udev, endpt_desc, ss_ep_comp_desc);
		avg_trb_len = max_esit_payload;

		ep_index = xhci_get_ep_index(endpt_desc);
		ep_ctx[ep_index] = xhci_get_ep_ctx(ctrl, virt_dev->in_ctx, ep_index);

		/* Allocate the ep rings */
		virt_dev->eps[ep_index].ring = xhci_ring_alloc(ctrl, 1, TRUE);
		if (!virt_dev->eps[ep_index].ring)
			return UHIOERR_OUTOFMEMORY;

		/*NOTE: ep_desc[0] actually represents EP1 and so on */
		dir = (((endpt_desc->bEndpointAddress) & (0x80)) >> 7);
		ep_type = (((endpt_desc->bmAttributes) & (0x3)) | (dir << 2));

		ep_ctx[ep_index]->ep_info = LE32(EP_MAX_ESIT_PAYLOAD_HI(max_esit_payload) | EP_INTERVAL(interval) | EP_MULT(mult));

		ep_ctx[ep_index]->ep_info2 = LE32(EP_TYPE(ep_type));
		ep_ctx[ep_index]->ep_info2 |= LE32(MAX_PACKET(LE16(get_unaligned(&endpt_desc->wMaxPacketSize))));

		/* Allow 3 retries for everything but isoc, set CErr = 3 */
		if (!usb_endpoint_xfer_isoc(endpt_desc))
			err_count = 3;
		ep_ctx[ep_index]->ep_info2 |= LE32(MAX_BURST(max_burst) | ERROR_COUNT(err_count));

		trb_64 = xhci_trb_virt_to_dma(virt_dev->eps[ep_index].ring->enq_seg,
									  virt_dev->eps[ep_index].ring->enqueue);
		ep_ctx[ep_index]->deq = LE64(trb_64 | virt_dev->eps[ep_index].ring->cycle_state);

		/*
		 * xHCI spec 6.2.3:
		 * 'Average TRB Length' should be 8 for control endpoints.
		 */
		if (usb_endpoint_xfer_control(endpt_desc))
			avg_trb_len = 8;
		ep_ctx[ep_index]->tx_info = LE32(EP_MAX_ESIT_PAYLOAD_LO(max_esit_payload) | EP_AVG_TRB_LENGTH(avg_trb_len));

		KprintfH("EP%ld %s: type=%ld maxp=%ld maxesit=%ld "
				 "interval=%ld mult=%ld maxburst=%ld\n",
				 (ULONG)usb_endpoint_num(endpt_desc),
				 dir ? "IN" : "OUT",
				 (ULONG)ep_type,
				 (ULONG)usb_endpoint_maxp(endpt_desc),
				 (ULONG)max_esit_payload,
				 (ULONG)interval,
				 (ULONG)(mult + 1),
				 (ULONG)(max_burst + 1));
	}

	return UHIOERR_NO_ERROR;
}

/**
 * Configure the endpoint, programming the device contexts.
 *
 * @param udev	pointer to the USB device structure
 * Return: returns the status of the xhci_configure_endpoints
 */
int xhci_set_configuration(struct usb_device *udev, int config_value)
{
	KprintfH("xhci_set_configuration: config_val=%ld\n", (LONG)config_value);

	struct usb_config *cfg = xhci_find_config(udev, config_value);
	if (!cfg)
	{
		Kprintf("xhci_set_configuration: config_val=%ld not found!\n", (LONG)config_value);
		return UHIOERR_BADPARAMS;
	}

	udev->active_config = cfg;
	for (unsigned int i = 0; i < cfg->no_of_if; ++i)
		xhci_select_active_alt(&cfg->if_desc[i]);

#ifdef DEBUG_HIGH
		/* Dump entire cfg using kprintf (all fields and all interfaces and endpoints) */
	xhci_dump_config(cfg, (LONG)udev->poseidon_address);
#endif

	struct xhci_container_ctx *out_ctx;
	struct xhci_container_ctx *in_ctx;
	struct xhci_input_control_ctx *ctrl_ctx;
	struct xhci_ctrl *ctrl = udev->controller;
	int slot_id = udev->slot_id;
	struct xhci_virt_device *virt_dev = ctrl->devs[slot_id];
	unsigned int max_ifnum = cfg->no_of_if;
	unsigned int max_ep_flag = 0;

	out_ctx = virt_dev->out_ctx;
	in_ctx = virt_dev->in_ctx;

	ctrl_ctx = xhci_get_input_control_ctx(in_ctx);
	u32 add_flags = SLOT_FLAG;
	u32 mask = xhci_collect_config_masks(cfg, max_ifnum, &max_ep_flag, 1);
	add_flags |= mask;
	ctrl_ctx->add_flags = LE32(add_flags);
	ctrl_ctx->drop_flags = 0;

	xhci_inval_cache((uintptr_t)out_ctx->bytes, out_ctx->size);

	/* slot context */
	xhci_slot_copy(ctrl, in_ctx, out_ctx);
	xhci_update_slot_last_ctx(ctrl, virt_dev, max_ep_flag);

	xhci_endpoint_copy(ctrl, in_ctx, out_ctx, 0);

	/* filling up ep contexts */
	for (unsigned int ifnum = 0; ifnum < max_ifnum; ++ifnum)
	{
		struct usb_interface *ifdesc = &cfg->if_desc[ifnum];
		int err = xhci_init_ep_contexts_if(udev, ctrl, virt_dev, ifdesc);
		if (err != UHIOERR_NO_ERROR)
		{
			return err;
		}
	}
	return UHIOERR_NO_ERROR;
}

/*
 * Full speed devices may have a max packet size greater than 8 bytes, but the
 * USB core doesn't know that until it reads the first 8 bytes of the
 * descriptor.  If the usb_device's max packet size changes after that point,
 * we need to issue an evaluate context command and wait on it.
 *
 * @param udev	pointer to the Device Data Structure
 * Return: returns the status of the xhci_configure_endpoints
 */
int xhci_check_maxpacket(struct usb_device *udev, unsigned int max_packet_size)
{
	struct xhci_ctrl *ctrl = udev->controller;
	unsigned int slot_id = udev->slot_id;
	int ep_index = 0; /* control endpoint */
	struct xhci_container_ctx *in_ctx;
	struct xhci_container_ctx *out_ctx;
	struct xhci_input_control_ctx *ctrl_ctx;
	struct xhci_ep_ctx *ep_ctx;
	unsigned int hw_max_packet_size;

	out_ctx = ctrl->devs[slot_id]->out_ctx;
	xhci_inval_cache((uintptr_t)out_ctx->bytes, out_ctx->size);
	KprintfH("Checking max packet size for ep 0\n");

	ep_ctx = xhci_get_ep_ctx(ctrl, out_ctx, ep_index);
	hw_max_packet_size = MAX_PACKET_DECODED(LE32(ep_ctx->ep_info2));
	if (hw_max_packet_size != max_packet_size)
	{
		KprintfH("Max Packet Size for ep 0 changed to %ld.\n", max_packet_size);
		KprintfH("Max packet size in xHCI HW = %ld\n", hw_max_packet_size);
		KprintfH("Issuing evaluate context command.\n");

		/* Set up the modified control endpoint 0 */
		xhci_endpoint_copy(ctrl, ctrl->devs[slot_id]->in_ctx,
						   ctrl->devs[slot_id]->out_ctx, ep_index);
		in_ctx = ctrl->devs[slot_id]->in_ctx;
		ep_ctx = xhci_get_ep_ctx(ctrl, in_ctx, ep_index);
		ep_ctx->ep_info2 &= LE32(~MAX_PACKET(MAX_PACKET_MASK));
		ep_ctx->ep_info2 |= LE32(MAX_PACKET(max_packet_size));

		/*
		 * Set up the input context flags for the command
		 * FIXME: This won't work if a non-default control endpoint
		 * changes max packet sizes.
		 */
		ctrl_ctx = xhci_get_input_control_ctx(in_ctx);
		ctrl_ctx->add_flags = LE32(EP0_FLAG);
		ctrl_ctx->drop_flags = 0;

		return 1;
	}
	return 0;
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
static void xhci_clear_port_change_bit(u16 wValue, u16 wIndex, volatile uint32_t *addr, u32 port_status)
{
	char *port_change_bit;
	u32 status;

	switch (wValue)
	{
	case USB_PORT_FEAT_C_RESET:
		status = PORT_RC;
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
	xhci_writel(addr, port_status | status);

	port_status = xhci_readl(addr);
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
static u32 xhci_port_state_to_neutral(u32 state)
{
	/* Save read-only status and port state */
	return (state & XHCI_PORT_RO) | (state & XHCI_PORT_RWS);
}

static size_t xhci_roothub_interrupt_bytes(struct xhci_ctrl *ctrl)
{
	if (!ctrl)
		return 0;

	int ports = HCS_MAX_PORTS(xhci_readl(&ctrl->hccr->cr_hcsparams1));
	if (ports < 0)
		ports = 0;
	if (ports > USB_MAXCHILDREN)
		ports = USB_MAXCHILDREN;
	return 1 + ((size_t)ports + 7U) / 8U;
}

static BOOL xhci_roothub_collect_status(struct xhci_ctrl *ctrl,
										uint8_t *buffer,
										size_t buffer_len,
										ULONG *actual_len)
{
	if (!buffer || buffer_len == 0)
		return FALSE;

	const size_t needed = xhci_roothub_interrupt_bytes(ctrl);
	const size_t out_len = min(buffer_len, needed);
	_memset(buffer, 0, buffer_len);

	BOOL change = FALSE;
	BOOL truncated = FALSE;
	int ports = HCS_MAX_PORTS(xhci_readl(&ctrl->hccr->cr_hcsparams1));
	if (ports > USB_MAXCHILDREN)
		ports = USB_MAXCHILDREN;
	const u32 change_mask = PORT_CSC | PORT_PEC | PORT_OCC | PORT_RC |
							PORT_PLC | PORT_WRC | PORT_CEC;
	for (int port = 0; port < ports; ++port)
	{
		u32 portsc = xhci_readl(&ctrl->hcor->portregs[port].or_portsc);
		u32 change_bits = portsc & change_mask;
		if (!change_bits)
			continue;

		size_t index = 1U + (port / 8U);
		if (index >= out_len)
		{
			truncated = TRUE;
			continue;
		}

		buffer[index] |= 1U << (port % 8);
		change = TRUE;
	}

#ifdef DEBUG_HIGH
	if (!change && truncated)
		KprintfH("roothub status truncated: need %ld bytes, have %ld\n",
				(LONG)needed, (LONG)buffer_len);
#else
	(void)truncated;
#endif

	if (change && actual_len)
		*actual_len = (ULONG)out_len;

	return change;
}

void xhci_roothub_maybe_complete(struct xhci_ctrl *ctrl)
{
	if (!ctrl || !ctrl->root_int_req)
		return;

	struct IOUsbHWReq *req = ctrl->root_int_req;
	struct usb_device *udev = ctrl->devices_by_poseidon_address[req->iouh_DevAddr];
	if (!udev)
	{
		Kprintf("missing root hub context for addr %ld\n", (LONG)req->iouh_DevAddr);
		ctrl->root_int_req = NULL;
		io_reply_failed(req, UHIOERR_HOSTERROR);
		return;
	}

	ULONG actual = 0;
	if (!xhci_roothub_collect_status(ctrl, (uint8_t *)req->iouh_Data,
									 req->iouh_Length, &actual))
	{
		KprintfH("xhci_roothub_maybe_complete: no status change, not completing root hub interrupt\n");
		return;
	}

	KprintfH("xhci_roothub_maybe_complete: completing root hub interrupt, actual=%ld\n", (LONG)actual);

	ctrl->root_int_req = NULL;
	io_reply_data(udev, req, UHIOERR_NO_ERROR, actual);
}

/**
 * Submits the Requests to the XHCI Host Controller
 *
 * @param udev pointer to the USB device structure
 * @param io  pointer to the IOUsbHWReq structure
 */
void xhci_submit_root(struct usb_device *udev, struct IOUsbHWReq *io)
{
	struct UsbSetupData *req = &io->iouh_SetupData;
	KprintfH("xhci_submit_root: type=%02lx req=%02lx val=%04lx idx=%04lx len=%04lx\n",
			 (ULONG)req->bmRequestType, (ULONG)req->bRequest,
			 (ULONG)LE16(req->wValue), (ULONG)LE16(req->wIndex), (ULONG)LE16(req->wLength));

	uint8_t tmpbuf[4];
	void *srcptr = NULL;
	uint32_t reg;
	struct xhci_ctrl *ctrl = udev->controller;
	struct xhci_hccr *hccr = ctrl->hccr;
	struct xhci_hcor *hcor = ctrl->hcor;
	int max_ports = HCS_MAX_PORTS(xhci_readl(&hccr->cr_hcsparams1));

	if ((req->bmRequestType & USB_RT_PORT) && LE16(req->wIndex) > max_ports)
	{
		Kprintf("The request port(%ld) exceeds maximum port number\n", LE16(req->wIndex) - 1);
		goto unknown;
	}

	volatile uint32_t *status_reg = (volatile uint32_t *)(&hcor->portregs[LE16(req->wIndex) - 1].or_portsc);
	int srclen = 0;

	u16 typeReq = req->bRequest | req->bmRequestType << 8;

	switch (typeReq)
	{
	case DeviceRequest | USB_REQ_GET_DESCRIPTOR:
		switch (LE16(req->wValue) >> 8)
		{
		case USB_DT_DEVICE:
			KprintfH("get USB_DT_DEVICE\n");
			srcptr = &descriptor.device;
			srclen = 0x12;
			break;
		case USB_DT_CONFIG:
			KprintfH("get USB_DT_CONFIG\n");
			srcptr = &descriptor.config;
			srclen = LE16(descriptor.config.wTotalLength);
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
		tmpbuf[0] = 1; /* self-powered */
		tmpbuf[1] = 0; /* remote-wakeup disabled */
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
			srcptr = &ctrl->hub_desc;
			srclen = ctrl->hub_desc.bLength;
			break;
		default:
			Kprintf("get unknown value %lx\n", LE16(req->wValue));
			goto unknown;
		}
		break;
	case USB_REQ_SET_ADDRESS | (USB_RECIP_DEVICE << 8):
		KprintfH("USB_REQ_SET_ADDRESS rootdev=%ld\n", (LONG)LE16(req->wValue));
		udev->speed = USB_SPEED_HIGH;
		UWORD new_addr = (UWORD)(LE16(req->wValue) & 0x7F);
		UWORD old_addr = (UWORD)(udev->poseidon_address & 0x7F);

		if (ctrl->devices_by_poseidon_address[new_addr] && ctrl->devices_by_poseidon_address[new_addr] != udev)
			Kprintf("root hub addr %ld already in use, overwriting\n", (LONG)new_addr);

		ctrl->devices_by_poseidon_address[new_addr] = udev;
		if (ctrl->devices_by_poseidon_address[old_addr] == udev)
			ctrl->devices_by_poseidon_address[old_addr] = NULL;

		ctrl->rootdev = new_addr;
		udev->poseidon_address = new_addr;
		break;
	case DeviceOutRequest | USB_REQ_SET_CONFIGURATION:
		KprintfH("USB_REQ_SET_CONFIGURATION\n");
		/* Do nothing */
		break;
	case USB_REQ_GET_STATUS | ((USB_DIR_IN | USB_RT_HUB) << 8):
		tmpbuf[0] = 1; /* USB_STATUS_SELFPOWERED */
		tmpbuf[1] = 0;
		srcptr = tmpbuf;
		srclen = 2;
		KprintfH("USB_REQ_GET_STATUS HUB\n");
		break;
	case USB_REQ_GET_STATUS | ((USB_RT_PORT | USB_DIR_IN) << 8):
		_memset(tmpbuf, 0, 4);
		reg = xhci_readl(status_reg);
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
		if (reg & PORT_RESET)
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
		if (reg & PORT_RC)
			tmpbuf[2] |= USB_PORT_STAT_C_RESET;

		srcptr = tmpbuf;
		srclen = 4;
		KprintfH("USB_REQ_GET_STATUS PORT%ld status = 0x%lx\n",
				 LE16(req->wIndex) - 1, reg);
		break;
	case USB_REQ_SET_FEATURE | ((USB_DIR_OUT | USB_RT_PORT) << 8):
		reg = xhci_readl(status_reg);
		reg = xhci_port_state_to_neutral(reg);
		KprintfH("SET_FEATURE PORT%ld status = 0x%lx feature=%lx\n",
				 LE16(req->wIndex) - 1, reg, LE16(req->wValue));
		switch (LE16(req->wValue))
		{
		case USB_PORT_FEAT_ENABLE:
			KprintfH("Set PORT_PE\n");
			reg |= PORT_PE;
			xhci_writel(status_reg, reg);
			break;
		case USB_PORT_FEAT_POWER:
			KprintfH("Set PORT_POWER\n");
			reg |= PORT_POWER;
			xhci_writel(status_reg, reg);
			break;
		case USB_PORT_FEAT_RESET:
			KprintfH("Set PORT_RESET\n");
			reg |= PORT_RESET;
			xhci_writel(status_reg, reg);
			break;
		case USB_PORT_FEAT_SUSPEND:
			/* Put link into U3 suspend */
			KprintfH("Putting link to U3 standby\n");
			reg &= ~PORT_PLS_MASK;
			reg |= XDEV_U3;
			xhci_writel(status_reg, reg);
			break;
		default:
			Kprintf("unknown feature %lx\n", LE16(req->wValue));
			goto unknown;
		}
		break;
	case USB_REQ_CLEAR_FEATURE | ((USB_DIR_OUT | USB_RT_PORT) << 8):
		reg = xhci_readl(status_reg);
		reg = xhci_port_state_to_neutral(reg);
		KprintfH("CLEAR_FEATURE PORT%ld status = 0x%lx feature=%lx\n",
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
		case USB_PORT_FEAT_C_RESET:
		case USB_PORT_FEAT_C_CONNECTION:
		case USB_PORT_FEAT_C_OVER_CURRENT:
		case USB_PORT_FEAT_C_ENABLE:
		case USB_PORT_FEAT_C_SUSPEND:
			KprintfH("Clear port %lx change\n", LE16(req->wValue));
			xhci_clear_port_change_bit((LE16(req->wValue)),
									   LE16(req->wIndex),
									   status_reg, reg);
			break;
		default:
			Kprintf("Unknown feature %lx\n", LE16(req->wValue));
			goto unknown;
		}
		xhci_writel(status_reg, reg);
		break;
	default:
		Kprintf("Unknown request\n");
		goto unknown;
	}

	KprintfH("scrlen = %ld req->length = %ld\n",
			 srclen, LE16(req->wLength));

	int len = min(srclen, (int)LE16(req->wLength));

	if (srcptr != NULL && len > 0)
	{
		CopyMem(srcptr, io->iouh_Data, len);
	}
	else
	{
		KprintfH("Len is 0\n");
	}

	io->iouh_Actual = len;
	io->iouh_Req.io_Error = UHIOERR_NO_ERROR;

	return;

unknown:
	io->iouh_Actual = 0;
	io->iouh_Req.io_Error = UHIOERR_STALL;

	return;
}

static int xhci_lowlevel_init(struct xhci_ctrl *ctrl)
{
	struct xhci_hccr *hccr;
	struct xhci_hcor *hcor;
	uint32_t val;
	uint32_t val2;
	uint32_t reg;

	hccr = ctrl->hccr;
	hcor = ctrl->hcor;
	/*
	 * Program the Number of Device Slots Enabled field in the CONFIG
	 * register with the max value of slots the HC can handle.
	 */
	val = (xhci_readl(&hccr->cr_hcsparams1) & HCS_SLOTS_MASK);
	val2 = xhci_readl(&hcor->or_config);
	val |= (val2 & ~HCS_SLOTS_MASK);
	xhci_writel(&hcor->or_config, val);

	/* initializing xhci data structures */
	if (xhci_mem_init(ctrl, hccr, hcor) < 0)
		return -ENOMEM;
	ctrl->hub_desc = descriptor.hub;

	reg = xhci_readl(&hccr->cr_hcsparams1);
	ctrl->hub_desc.bNbrPorts = HCS_MAX_PORTS(reg);
	if (ctrl->hub_desc.bNbrPorts > USB_MAXCHILDREN)
		ctrl->hub_desc.bNbrPorts = USB_MAXCHILDREN;
	{
		unsigned int removable_bytes = (ctrl->hub_desc.bNbrPorts + 1U + 7U) / 8U;
		const unsigned int max_bitmap = sizeof(ctrl->hub_desc.u.hs.DeviceRemovable);
		if (removable_bytes == 0)
			removable_bytes = 1;
		if (removable_bytes > max_bitmap)
			removable_bytes = max_bitmap;
		ctrl->hub_desc.bLength = (unsigned char)(7U + (removable_bytes * 2U));
		for (unsigned int i = 0; i < max_bitmap; ++i)
		{
			ctrl->hub_desc.u.hs.DeviceRemovable[i] = 0x00;
			ctrl->hub_desc.u.hs.PortPowerCtrlMask[i] = (i < removable_bytes) ? 0xFF : 0x00;
		}
		unsigned int intr_bitmap_bytes = (ctrl->hub_desc.bNbrPorts + 7U) / 8U;
		if (intr_bitmap_bytes == 0)
			intr_bitmap_bytes = 1;
		descriptor.endpoint.wMaxPacketSize = cpu_to_le16(1U + intr_bitmap_bytes);
		KprintfH("root hub ports=%ld status-bytes=%ld removable-bytes=%ld\n",
			 (LONG)ctrl->hub_desc.bNbrPorts,
			 (LONG)(1U + intr_bitmap_bytes),
			 (LONG)removable_bytes);
	}
	Kprintf("Register %lx NbrPorts %ld\n", reg, ctrl->hub_desc.bNbrPorts);

	/* Port Indicators */
	reg = xhci_readl(&hccr->cr_hccparams);
	if (HCS_INDICATOR(reg))
		put_unaligned(get_unaligned(&ctrl->hub_desc.wHubCharacteristics) | 0x80, &ctrl->hub_desc.wHubCharacteristics);

	/* Port Power Control */
	if (HCC_PPC(reg))
		put_unaligned(get_unaligned(&ctrl->hub_desc.wHubCharacteristics) | 0x01, &ctrl->hub_desc.wHubCharacteristics);

	if (xhci_start(hcor))
	{
		xhci_reset(hcor);
		return -ENODEV;
	}

	/* Zero'ing IRQ control register and IRQ pending register */
	xhci_writel(&ctrl->ir_set->irq_control, 0x0);
	xhci_writel(&ctrl->ir_set->irq_pending, 0x0);

	reg = HC_VERSION(xhci_readl(&hccr->cr_capbase));
	Kprintf("USB XHCI %lx.%02lx\n", reg >> 8, reg & 0xff);
	ctrl->hci_version = reg;

	return 0;
}

static int xhci_lowlevel_stop(struct xhci_ctrl *ctrl)
{
	u32 temp;

	xhci_reset(ctrl->hcor);

	Kprintf("// Disabling event ring interrupts\n");
	temp = xhci_readl(&ctrl->hcor->or_usbsts);
	xhci_writel(&ctrl->hcor->or_usbsts, temp & ~STS_EINT);
	temp = xhci_readl(&ctrl->ir_set->irq_pending);
	xhci_writel(&ctrl->ir_set->irq_pending, ER_IRQ_DISABLE(temp));

	return 0;
}

int xhci_register(struct xhci_ctrl *ctrl, struct xhci_hccr *hccr,
				  struct xhci_hcor *hcor)
{
	int ret;

	Kprintf("ctrl=%lx, hccr=%lx, hcor=%lx\n", ctrl, hccr, hcor);

	ret = xhci_reset(hcor);
	if (ret)
		goto err;

	ctrl->memoryPool = CreatePool(MEMF_FAST | MEMF_CLEAR, 16384, 8192);
	if (ctrl->memoryPool == NULL)
	{
		ret = -ENOMEM;
		goto err;
	}
	Kprintf("memory pool created: %lx\n", ctrl->memoryPool);

	ctrl->hccr = hccr;
	ctrl->hcor = hcor;
	ret = xhci_lowlevel_init(ctrl);
	if (ret)
		goto err_pool;

	return 0;

err_pool:
	DeletePool(ctrl->memoryPool);
	ctrl->memoryPool = NULL;

err:
	Kprintf("failed, ret=%ld\n", ret);
	return ret;
}

int xhci_deregister(struct xhci_ctrl *ctrl)
{
	xhci_lowlevel_stop(ctrl);
	xhci_cleanup(ctrl);

	if (ctrl->memoryPool)
	{
		DeletePool(ctrl->memoryPool);
		ctrl->memoryPool = NULL;
	}

	return 0;
}
