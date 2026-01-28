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
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-debug.h>
#include <usb_glue.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

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
	xhci_dump_config("[xhci] xhci_set_configuration:", cfg, (LONG)udev->poseidon_address);
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

	ctrl->devices_by_poseidon_address[0] = usb_glue_alloc_udev(ctrl, 0);
	ctrl->root_hub = xhci_roothub_create(ctrl->devices_by_poseidon_address[0], io_reply_data);
	if (!ctrl->root_hub)
	{
		return -ENOMEM;
	}

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

	xhci_roothub_destroy(ctrl->root_hub);
	ctrl->root_hub = NULL;

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
