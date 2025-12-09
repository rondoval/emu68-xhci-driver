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

#include <compat.h>
#include <debug.h>
#include <xhci/usb.h>
#include <devices/usbhardware.h>

#include <xhci/xhci.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-debug.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-ring] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci-ring] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

/*
 * Returns zero if the TRB isn't in this segment, otherwise it returns the DMA
 * address of the TRB.
 */
dma_addr_t xhci_trb_virt_to_dma(struct xhci_segment *seg,
								union xhci_trb *trb)
{
	unsigned long segment_offset;

	if (!seg || !trb || trb < seg->trbs)
		return 0;
	/* offset in TRBs */
	segment_offset = trb - seg->trbs;
	if (segment_offset >= TRBS_PER_SEGMENT)
		return 0;
	return seg->dma + (segment_offset * sizeof(*trb));
}

/**
 * Is this TRB a link TRB or was the last TRB the last TRB in this event ring
 * segment?  I.e. would the updated event TRB pointer step off the end of the
 * event seg ?
 *
 * @param ctrl	Host controller data structure
 * @param ring	pointer to the ring
 * @param seg	poniter to the segment to which TRB belongs
 * @param trb	poniter to the ring trb
 * Return: 1 if this TRB a link TRB else 0
 */
static int last_trb(struct xhci_ctrl *ctrl, struct xhci_ring *ring,
					struct xhci_segment *seg, union xhci_trb *trb)
{
	if (ring == ctrl->event_ring)
		return trb == &seg->trbs[TRBS_PER_SEGMENT];
	else
		return TRB_TYPE_LINK_LE32(trb->link.control);
}

/**
 * Does this link TRB point to the first segment in a ring,
 * or was the previous TRB the last TRB on the last segment in the ERST?
 *
 * @param ctrl	Host controller data structure
 * @param ring	pointer to the ring
 * @param seg	poniter to the segment to which TRB belongs
 * @param trb	poniter to the ring trb
 * Return: 1 if this TRB is the last TRB on the last segment else 0
 */
static BOOL last_trb_on_last_seg(struct xhci_ctrl *ctrl,
								 struct xhci_ring *ring,
								 struct xhci_segment *seg,
								 union xhci_trb *trb)
{
	if (ring == ctrl->event_ring)
		return ((trb == &seg->trbs[TRBS_PER_SEGMENT]) &&
				(seg->next == ring->first_seg));
	else
		return LE32(trb->link.control) & LINK_TOGGLE;
}

/**
 * See Cycle bit rules. SW is the consumer for the event ring only.
 * Don't make a ring full of link TRBs.  That would be dumb and this would loop.
 *
 * If we've just enqueued a TRB that is in the middle of a TD (meaning the
 * chain bit is set), then set the chain bit in all the following link TRBs.
 * If we've enqueued the last TRB in a TD, make sure the following link TRBs
 * have their chain bit cleared (so that each Link TRB is a separate TD).
 *
 * Section 6.4.4.1 of the 0.95 spec says link TRBs cannot have the chain bit
 * set, but other sections talk about dealing with the chain bit set.  This was
 * fixed in the 0.96 specification errata, but we have to assume that all 0.95
 * xHCI hardware can't handle the chain bit being cleared on a link TRB.
 *
 * @param ctrl	Host controller data structure
 * @param ring	pointer to the ring
 * @param more_trbs_coming	flag to indicate whether more trbs
 *				are expected or NOT.
 *				Will you enqueue more TRBs before calling
 *				prepare_ring()?
 * Return: none
 */
static void inc_enq(struct xhci_ctrl *ctrl, struct xhci_ring *ring,
					BOOL more_trbs_coming)
{
	u32 chain;
	union xhci_trb *next;

	chain = LE32(ring->enqueue->generic.field[3]) & TRB_CHAIN;
	next = ++(ring->enqueue);

	/*
	 * Update the dequeue pointer further if that was a link TRB or we're at
	 * the end of an event ring segment (which doesn't have link TRBS)
	 */
	while (last_trb(ctrl, ring, ring->enq_seg, next))
	{
		if (ring != ctrl->event_ring)
		{
			/*
			 * If the caller doesn't plan on enqueueing more
			 * TDs before ringing the doorbell, then we
			 * don't want to give the link TRB to the
			 * hardware just yet.  We'll give the link TRB
			 * back in prepare_ring() just before we enqueue
			 * the TD at the top of the ring.
			 */
			if (!chain && !more_trbs_coming)
				break;

			/*
			 * If we're not dealing with 0.95 hardware or
			 * isoc rings on AMD 0.96 host,
			 * carry over the chain bit of the previous TRB
			 * (which may mean the chain bit is cleared).
			 */
			next->link.control &= LE32(~TRB_CHAIN);
			next->link.control |= LE32(chain);

			next->link.control ^= LE32(TRB_CYCLE);
			xhci_flush_cache((uintptr_t)next,
							 sizeof(union xhci_trb));
		}
		/* Toggle the cycle bit after the last ring segment. */
		if (last_trb_on_last_seg(ctrl, ring,
								 ring->enq_seg, next))
			ring->cycle_state = (ring->cycle_state ? 0 : 1);

		ring->enq_seg = ring->enq_seg->next;
		ring->enqueue = ring->enq_seg->trbs;
		next = ring->enqueue;
	}
}

/**
 * See Cycle bit rules. SW is the consumer for the event ring only.
 * Don't make a ring full of link TRBs.  That would be dumb and this would loop.
 *
 * @param ctrl	Host controller data structure
 * @param ring	Ring whose Dequeue TRB pointer needs to be incremented.
 * return none
 */
void inc_deq(struct xhci_ctrl *ctrl, struct xhci_ring *ring)
{
	do
	{
		/*
		 * Update the dequeue pointer further if that was a link TRB or
		 * we're at the end of an event ring segment (which doesn't have
		 * link TRBS)
		 */
		if (last_trb(ctrl, ring, ring->deq_seg, ring->dequeue))
		{
			if (ring == ctrl->event_ring &&
				last_trb_on_last_seg(ctrl, ring,
									 ring->deq_seg, ring->dequeue))
			{
				ring->cycle_state = (ring->cycle_state ? 0 : 1);
			}
			ring->deq_seg = ring->deq_seg->next;
			ring->dequeue = ring->deq_seg->trbs;
		}
		else
		{
			ring->dequeue++;
		}
	} while (last_trb(ctrl, ring, ring->deq_seg, ring->dequeue));
}

/**
 * Generic function for queueing a TRB on a ring.
 * The caller must have checked to make sure there's room on the ring.
 *
 * @param	more_trbs_coming:   Will you enqueue more TRBs before calling
 *				prepare_ring()?
 * @param ctrl	Host controller data structure
 * @param ring	pointer to the ring
 * @param more_trbs_coming	flag to indicate whether more trbs
 * @param trb_fields	pointer to trb field array containing TRB contents
 * Return: pointer to the enqueued trb
 */
dma_addr_t queue_trb(struct xhci_ctrl *ctrl, struct xhci_ring *ring,
					 BOOL more_trbs_coming, unsigned int *trb_fields)
{
	struct xhci_generic_trb *trb;
	dma_addr_t addr;
	int i;

	trb = &ring->enqueue->generic;

	for (i = 0; i < 4; i++)
		trb->field[i] = LE32(trb_fields[i]);

	xhci_flush_cache((uintptr_t)trb, sizeof(struct xhci_generic_trb));

	addr = xhci_trb_virt_to_dma(ring->enq_seg, (union xhci_trb *)trb);

	inc_enq(ctrl, ring, more_trbs_coming);

	return addr;
}

/**
 * Does various checks on the endpoint ring, and makes it ready
 * to queue num_trbs.
 *
 * @param ctrl		Host controller data structure
 * @param ep_ring	pointer to the EP Transfer Ring
 * @param ep_state	State of the End Point
 * Return: error code in case of invalid ep_state, 0 on success
 */
int prepare_ring(struct xhci_ctrl *ctrl, struct xhci_ring *ep_ring,
				 u32 ep_state)
{
	union xhci_trb *next = ep_ring->enqueue;

	/* Make sure the endpoint has been added to xHC schedule */
	switch (ep_state)
	{
	case EP_STATE_DISABLED:
		/*
		 * USB core changed config/interfaces without notifying us,
		 * or hardware is reporting the wrong state.
		 */
		Kprintf("WARN urb submitted to disabled ep\n");
		return UHIOERR_BADPARAMS;
	case EP_STATE_ERROR:
		Kprintf("WARN waiting for error on ep to be cleared\n");
		return UHIOERR_HOSTERROR;
	case EP_STATE_HALTED:
		Kprintf("WARN endpoint is halted\n");
		return UHIOERR_HOSTERROR;
	case EP_STATE_STOPPED:
	case EP_STATE_RUNNING:
		KprintfH("EP STATE RUNNING.\n");
		break;
	default:
		Kprintf("ERROR unknown endpoint state for ep state=%ld\n", (LONG)ep_state);
		return UHIOERR_BADPARAMS;
	}

	while (last_trb(ctrl, ep_ring, ep_ring->enq_seg, next))
	{
		/*
		 * If we're not dealing with 0.95 hardware or isoc rings
		 * on AMD 0.96 host, clear the chain bit.
		 */
		next->link.control &= LE32(~TRB_CHAIN);

		next->link.control ^= LE32(TRB_CYCLE);

		xhci_flush_cache((uintptr_t)next, sizeof(union xhci_trb));

		/* Toggle the cycle bit after the last ring segment. */
		if (last_trb_on_last_seg(ctrl, ep_ring, ep_ring->enq_seg, next))
			ep_ring->cycle_state = (ep_ring->cycle_state ? 0 : 1);
		ep_ring->enq_seg = ep_ring->enq_seg->next;
		ep_ring->enqueue = ep_ring->enq_seg->trbs;
		next = ep_ring->enqueue;
	}

	return UHIOERR_NO_ERROR;
}

/*
 * For xHCI 1.0 host controllers, TD size is the number of max packet sized
 * packets remaining in the TD (*not* including this TRB).
 *
 * Total TD packet count = total_packet_count =
 *     DIV_ROUND_UP(TD size in bytes / wMaxPacketSize)
 *
 * Packets transferred up to and including this TRB = packets_transferred =
 *     rounddown(total bytes transferred including this TRB / wMaxPacketSize)
 *
 * TD size = total_packet_count - packets_transferred
 *
 * For xHCI 0.96 and older, TD size field should be the remaining bytes
 * including this TRB, right shifted by 10
 *
 * For all hosts it must fit in bits 21:17, so it can't be bigger than 31.
 * This is taken care of in the TRB_TD_SIZE() macro
 *
 * The last TRB in a TD must have the TD size set to zero.
 *
 * @param ctrl	host controller data structure
 * @param transferred	total size sent so far
 * @param trb_buff_len	length of the TRB Buffer
 * @param td_total_len	total packet count
 * @param maxp	max packet size of current pipe
 * @param more_trbs_coming	indicate last trb in TD
 * Return: remainder
 */
static u32 xhci_td_remainder(struct xhci_ctrl *ctrl, int transferred,
							 unsigned int trb_buff_len, unsigned int td_total_len,
							 int maxp, BOOL more_trbs_coming)
{
	u32 total_packet_count;

	/* MTK xHCI 0.96 contains some features from 1.0 */
	if (ctrl->hci_version < 0x100)
		return ((td_total_len - transferred) >> 10);

	/* One TRB with a zero-length data packet. */
	if (!more_trbs_coming || (transferred == 0 && trb_buff_len == 0) ||
		trb_buff_len == td_total_len)
		return 0;

	total_packet_count = DIV_ROUND_UP(td_total_len, maxp);

	/* Queueing functions don't count the current TRB into transferred */
	return (total_packet_count - ((transferred + trb_buff_len) / maxp));
}

/**
 * Ring the doorbell of the End Point
 *
 * @param udev		pointer to the USB device structure
 * @param ep_index	index of the endpoint
 * @param start_cycle	cycle flag of the first TRB
 * @param start_trb	pionter to the first TRB
 * Return: none
 */
static void giveback_first_trb(struct usb_device *udev, int ep_index,
							   int start_cycle,
							   struct xhci_generic_trb *start_trb)
{
	struct xhci_ctrl *ctrl = udev->controller;

	/*
	 * Pass all the TRBs to the hardware at once and make sure this write
	 * isn't reordered.
	 */
	if (start_cycle)
		start_trb->field[3] |= LE32(start_cycle);
	else
		start_trb->field[3] &= LE32(~TRB_CYCLE);

	xhci_flush_cache((uintptr_t)start_trb, sizeof(struct xhci_generic_trb));

	/* Ringing EP doorbell here */
	xhci_writel(&ctrl->dba->doorbell[udev->slot_id],
				DB_VALUE(ep_index, 0));

	return;
}

/*
 * Common helper for bulk/int/isoc rings.
 */
static int xhci_stream_tx(struct usb_device *udev, struct IOUsbHWReq *io,
						  unsigned int timeout_ms, enum ep_state state,
						  u32 trb_type_bits, BOOL enable_short_packet)
{
	int num_trbs = 0;
	struct xhci_generic_trb *start_trb;
	BOOL first_trb = FALSE;
	int start_cycle;
	u32 field = 0;
	u32 length_field = 0;
	struct xhci_ctrl *ctrl = udev->controller;
	unsigned int slot_id = udev->slot_id;
	struct xhci_virt_device *virt_dev;
	struct xhci_ep_ctx *ep_ctx;
	struct xhci_ring *ring; /* EP transfer ring */

	int running_total, trb_buff_len;
	BOOL more_trbs_coming = TRUE;
	u64 addr;
	int ret;
	u32 trb_fields[4];
	const int length = io->iouh_Length;
	unsigned int ep = io->iouh_Endpoint & 0xf;
	if (ep == 0)
	{
		Kprintf("stream transfer on EP0 not supported\n");
		return UHIOERR_BADPARAMS;
	}
	unsigned int ep_index = ep * 2 - ((io->iouh_Dir == UHDIR_IN) ? 0 : 1);
	int max_packet = io->iouh_MaxPktSize ? io->iouh_MaxPktSize : 1;
	u64 buf_64 = xhci_dma_map(ctrl, io->iouh_Data, length);
	dma_addr_t last_transfer_trb_addr;

	virt_dev = ctrl->devs[slot_id];
	if (!virt_dev)
		return UHIOERR_HOSTERROR;

	xhci_inval_cache((uintptr_t)virt_dev->out_ctx->bytes,
					 virt_dev->out_ctx->size);

	ep_ctx = xhci_get_ep_ctx(ctrl, virt_dev->out_ctx, ep_index);
#ifdef DEBUG_HIGH
	xhci_dump_slot_ctx(__func__, xhci_get_slot_ctx(ctrl, virt_dev->out_ctx));
	xhci_dump_ep_ctx(__func__, io->iouh_Endpoint, ep_ctx);
#endif

	/*
	 * If the endpoint was halted due to a prior error, resume it before
	 * the next transfer. It is the responsibility of the upper layer to
	 * have dealt with whatever caused the error.
	 */
	if ((LE32(ep_ctx->ep_info) & EP_STATE_MASK) == EP_STATE_HALTED)
	{
		xhci_reset_ep(udev, ep);
		return UHIOERR_HOSTERROR;
	}

	ring = virt_dev->eps[ep_index].ring;
	if (!ring)
	{
		Kprintf("no ring for ep %ld\n", (LONG)ep);
		return UHIOERR_HOSTERROR;
	}

	/*
	 * How much data is (potentially) left before the 64KB boundary?
	 * XHCI Spec (Table 49 / 6.4.1) requires we avoid spanning that boundary, so
	 * we may need several chained TRBs if the buffer crosses it.
	 */
	running_total = TRB_MAX_BUFF_SIZE - (lower_32_bits(buf_64) & (TRB_MAX_BUFF_SIZE - 1));
	trb_buff_len = running_total;
	running_total &= TRB_MAX_BUFF_SIZE - 1;

	/* If we already cover bytes in this 64KB chunk, or the transfer is zero length,
	 * we schedule at least one TRB now.
	 */
	if (running_total != 0 || length == 0)
		num_trbs++;

	/* Account for remaining 64KB windows, adding more TRBs as needed. */
	while (running_total < length)
	{
		num_trbs++;
		running_total += TRB_MAX_BUFF_SIZE;
	}

	/* Prepare the ring (resumes from stopped state, sets dequeue pointers, etc.). */
	ret = prepare_ring(ctrl, ring, LE32(ep_ctx->ep_info) & EP_STATE_MASK);
	if (ret < 0)
		return ret;

	/* Defer toggling the cycle bit on the first TRB until the full TD is ready. */
	start_trb = &ring->enqueue->generic;
	start_cycle = ring->cycle_state;

	running_total = 0;

	/* How much data is in the first TRB? */
	/*
	 * How much data is (potentially) left before the 64KB boundary?
	 * XHCI Spec puts restriction( TABLE 49 and 6.4.1 section of XHCI Spec)
	 * that the buffer should not span 64KB boundary. if so
	 * we send request in more than 1 TRB by chaining them.
	 */
	addr = buf_64;

	if (trb_buff_len > length)
		trb_buff_len = length;

	first_trb = TRUE;
	KprintfH("maxpacketsize=%ld num_trbs=%ld\n",
			 (LONG)io->iouh_MaxPktSize, (LONG)num_trbs);

	if (length > 0)
		/* flush the buffer before handing it to the controller */
		xhci_flush_cache((ULONG)io->iouh_Data, length);

	/* Queue each TRB, chaining when necessary and marking the final one IOC. */
	/* Queue the first TRB, even if it's zero-length. */
	do
	{
		u32 remainder = 0;
		field = 0;
		/* Don't change the cycle bit of the first TRB until later */
		if (first_trb)
		{
			first_trb = FALSE;
			if (start_cycle == 0)
				field |= TRB_CYCLE;
		}
		else
		{
			field |= ring->cycle_state;
		}

		/*
		 * Chain all the TRBs together; clear the chain bit in the last
		 * TRB to indicate it's the last TRB in the chain.
		 */
		if (num_trbs > 1)
		{
			field |= TRB_CHAIN;
		}
		else
		{
			field |= TRB_IOC;
			more_trbs_coming = FALSE;
		}

		/* Only set interrupt on short packet for IN endpoints */
		if (enable_short_packet && io->iouh_Dir == UHDIR_IN)
			field |= TRB_ISP;

		/* Set the TRB length, TD size, and interrupter fields. */
		remainder = xhci_td_remainder(ctrl, running_total, trb_buff_len,
									  length, max_packet,
									  more_trbs_coming);

		length_field = (TRB_LEN(trb_buff_len) |
						TRB_TD_SIZE(remainder) |
						TRB_INTR_TARGET(0));

		trb_fields[0] = lower_32_bits(addr);
		trb_fields[1] = upper_32_bits(addr);
		trb_fields[2] = length_field;
		trb_fields[3] = field | trb_type_bits;

		KprintfH("queue TRB addr=%08lx%08lx lenf=%08lx flags=%08lx num_trbs=%ld\n",
				 (ULONG)trb_fields[1], (ULONG)trb_fields[0], (ULONG)trb_fields[2], (ULONG)trb_fields[3], (LONG)num_trbs);

		last_transfer_trb_addr = queue_trb(ctrl, ring, (num_trbs > 1), trb_fields);

		--num_trbs;

		running_total += trb_buff_len;

		/* Calculate length for next transfer */
		addr += trb_buff_len;
		trb_buff_len = min((length - running_total), TRB_MAX_BUFF_SIZE);
	} while (running_total < length);

	/* Hand the first TRB back to the controller once the TD is ready. */
	giveback_first_trb(udev, ep_index, start_cycle, start_trb);
	KprintfH("waiting, last_trb_addr=%08lx%08lx start_cycle=%ld\n",
			 (ULONG)upper_32_bits((u64)last_transfer_trb_addr),
			 (ULONG)lower_32_bits((u64)last_transfer_trb_addr), (LONG)start_cycle);

	xhci_ep_set_receiving(udev, io, state, last_transfer_trb_addr, timeout_ms);
	return UHIOERR_NO_ERROR;
}

/**** Bulk and Control transfer methods ****/

/**
 * Queues up the BULK Request
 *
 * @param udev		pointer to the USB device structure
 * @param io		pointer to the IOUsbHWReq structure
 * @param timeout_ms	timeout in milliseconds
 * Return: returns 0 if successful else error code
 */
int xhci_bulk_tx(struct usb_device *udev, struct IOUsbHWReq *io, unsigned int timeout_ms)
{
	KprintfH("slot=%ld ep=%ld dir=%s buf=%lx len=%ld\n",
			 (LONG)udev->slot_id,
			 (LONG)(io->iouh_Endpoint & 0xf),
			 (io->iouh_Dir == UHDIR_IN) ? "IN" : "OUT",
			 io->iouh_Data, io->iouh_Length);

	return xhci_stream_tx(udev, io, timeout_ms,
						  USB_DEV_EP_STATE_RECEIVING_BULK,
						  TRB_TYPE(TRB_NORMAL), TRUE);
}

int xhci_int_tx(struct usb_device *udev, struct IOUsbHWReq *io, unsigned int timeout_ms)
{
	KprintfH("slot=%ld ep=%ld dir=%s buf=%lx len=%ld\n",
			 (LONG)udev->slot_id,
			 (LONG)(io->iouh_Endpoint & 0xf),
			 (io->iouh_Dir == UHDIR_IN) ? "IN" : "OUT",
			 io->iouh_Data, io->iouh_Length);

	return xhci_stream_tx(udev, io, timeout_ms,
						  USB_DEV_EP_STATE_RECEIVING_INT,
						  TRB_TYPE(TRB_NORMAL), TRUE);
}

int xhci_iso_tx(struct usb_device *udev, struct IOUsbHWReq *io, unsigned int timeout_ms)
{
	KprintfH("slot=%ld ep=%ld dir=%s buf=%lx len=%ld interval=%ld\n",
			 (LONG)udev->slot_id,
			 (LONG)(io->iouh_Endpoint & 0xf),
			 (io->iouh_Dir == UHDIR_IN) ? "IN" : "OUT",
			 io->iouh_Data, io->iouh_Length, (LONG)io->iouh_Interval);

	return xhci_stream_tx(udev, io, timeout_ms,
						  USB_DEV_EP_STATE_RECEIVING_ISOC,
						  TRB_TYPE(TRB_ISOC) | TRB_SIA, FALSE);
}

int xhci_rt_iso_tx(struct usb_device *udev, struct IOUsbHWReq *io)
{
	KprintfH("slot=%ld ep=%ld dir=%s buf=%lx len=%ld interval=%ld\n",
			 (LONG)udev->slot_id,
			 (LONG)(io->iouh_Endpoint & 0xf),
			 (io->iouh_Dir == UHDIR_IN) ? "IN" : "OUT",
			 io->iouh_Data, io->iouh_Length, (LONG)io->iouh_Interval);

	return xhci_stream_tx(udev, io, 0,
						  USB_DEV_EP_STATE_RT_ISO_RUNNING,
						  TRB_TYPE(TRB_ISOC) | TRB_SIA | TRB_IOC, FALSE);
}

/**
 * Queues up the Control Transfer Request
 *
 * @param udev	pointer to the USB device structure
 * @param io		pointer to the IOUsbHWReq structure
 * @param timeout_ms	Timeout in milliseconds
 * Return: returns 0 if successful else error code on failure
 */
int xhci_ctrl_tx(struct usb_device *udev, struct IOUsbHWReq *io, unsigned int timeout_ms)
{
	KprintfH("slot=%ld ep=%ld length=%ld dir=%s\n",
			 (LONG)udev->slot_id, io->iouh_Endpoint, io->iouh_Length,
			 (io->iouh_SetupData.bmRequestType & USB_DIR_IN) ? "IN" : "OUT");
	int ret;
	int start_cycle;
	int num_trbs;
	u32 field;
	u32 length_field;
	u64 buf_64 = 0;
	struct xhci_generic_trb *start_trb;
	struct xhci_ctrl *ctrl = udev->controller;
	unsigned int slot_id = udev->slot_id;
	unsigned int ep_index;
	u32 trb_fields[4];
	struct xhci_virt_device *virt_dev = ctrl->devs[slot_id];
	struct xhci_ring *ep_ring;
	u32 remainder;
	const int length = io->iouh_Length;

	ep_index = (io->iouh_Endpoint & 0xf) * 2; // ep_index is DCI-1 for control endpoints

	ep_ring = virt_dev->eps[ep_index].ring;
	if (!ep_ring)
		return UHIOERR_HOSTERROR;

	/*
	 * Check to see if the max packet size for the default control
	 * endpoint changed during FS device enumeration
	 */
	if (udev->speed == USB_SPEED_FULL && slot_id != 0)
	{
		if (xhci_check_maxpacket(udev, io->iouh_MaxPktSize))
		{
			xhci_configure_endpoints(udev, TRUE, io);
			return UHIOERR_NO_ERROR;
		}
	}

	xhci_inval_cache((uintptr_t)virt_dev->out_ctx->bytes, virt_dev->out_ctx->size);

	struct xhci_ep_ctx *ep_ctx = NULL;
	ep_ctx = xhci_get_ep_ctx(ctrl, virt_dev->out_ctx, ep_index);

	/* 1 TRB for setup, 1 for status */
	num_trbs = 2;
	/*
	 * Don't need to check if we need additional event data and normal TRBs,
	 * since data in control transfers will never get bigger than 16MB
	 * XXX: can we get a buffer that crosses 64KB boundaries?
	 */

	if (length > 0)
		num_trbs++;
	/*
	 * XXX: Calling routine prepare_ring() called in place of
	 * prepare_trasfer() as there in 'Linux' since we are not
	 * maintaining multiple TDs/transfer at the same time.
	 */
	KprintfH("prepare_ring ep_state=%lx\n", (ULONG)(LE32(ep_ctx->ep_info) & EP_STATE_MASK));
	ret = prepare_ring(ctrl, ep_ring, LE32(ep_ctx->ep_info) & EP_STATE_MASK);

	if (ret != UHIOERR_NO_ERROR)
		return ret;

	/*
	 * Don't give the first TRB to the hardware (by toggling the cycle bit)
	 * until we've finished creating all the other TRBs.  The ring's cycle
	 * state may change as we enqueue the other TRBs, so save it too.
	 */
	start_trb = &ep_ring->enqueue->generic;
	start_cycle = ep_ring->cycle_state;
	KprintfH("start_trb=%lx start_cycle=%ld\n", (ULONG)start_trb, (LONG)start_cycle);

	/* Queue setup TRB - see section 6.4.1.2.1 */
	/* FIXME better way to translate setup_packet into two u32 fields? */
	field = 0;
	field |= TRB_IDT | TRB_TYPE(TRB_SETUP);
	if (start_cycle == 0)
		field |= 0x1;

	/* xHCI 1.0 6.4.1.2.1: Transfer Type field */
	if (ctrl->hci_version >= 0x100)
	{
		if (length > 0)
		{
			if (io->iouh_SetupData.bmRequestType & USB_DIR_IN)
				field |= TRB_TX_TYPE(TRB_DATA_IN);
			else
				field |= TRB_TX_TYPE(TRB_DATA_OUT);
		}
	}

	trb_fields[0] = io->iouh_SetupData.bmRequestType | io->iouh_SetupData.bRequest << 8 |
					LE16(io->iouh_SetupData.wValue) << 16;
	trb_fields[1] = LE16(io->iouh_SetupData.wIndex) |
					LE16(io->iouh_SetupData.wLength) << 16;
	/* TRB_LEN | (TRB_INTR_TARGET) */
	trb_fields[2] = (TRB_LEN(8) | TRB_INTR_TARGET(0));
	/* Immediate data in pointer */
	trb_fields[3] = field;
	KprintfH("setup TRB fields: %08lx %08lx %08lx %08lx\n",
			 (ULONG)trb_fields[0], (ULONG)trb_fields[1], (ULONG)trb_fields[2], (ULONG)trb_fields[3]);
	queue_trb(ctrl, ep_ring, TRUE, trb_fields);

	/* Re-initializing field to zero */
	field = 0;
	/* If there's data, queue data TRBs */
	/* Only set interrupt on short packet for IN endpoints */
	if (io->iouh_SetupData.bmRequestType & USB_DIR_IN)
		field = TRB_ISP | TRB_TYPE(TRB_DATA);
	else
		field = TRB_TYPE(TRB_DATA);

	remainder = xhci_td_remainder(ctrl, 0, length, length, (int)io->iouh_MaxPktSize, TRUE);
	length_field = TRB_LEN(length) | TRB_TD_SIZE(remainder) |
				   TRB_INTR_TARGET(0);
	KprintfH("length_field = %ld, length = %ld,"
			 "xhci_td_remainder(length) = %ld , TRB_INTR_TARGET(0) = %ld\n",
			 length_field, TRB_LEN(length),
			 TRB_TD_SIZE(remainder), 0);

	if (length > 0)
	{
		if (io->iouh_SetupData.bmRequestType & USB_DIR_IN)
			field |= TRB_DIR_IN;
		buf_64 = xhci_dma_map(ctrl, io->iouh_Data, length);

		trb_fields[0] = lower_32_bits(buf_64);
		trb_fields[1] = upper_32_bits(buf_64);
		trb_fields[2] = length_field;
		trb_fields[3] = field | ep_ring->cycle_state;

		xhci_flush_cache((uintptr_t)buf_64, length); // TODO dma_map should not realocate buffer... then revert to buffer
		KprintfH("data TRB fields: %08lx %08lx %08lx %08lx\n",
				 (ULONG)trb_fields[0], (ULONG)trb_fields[1], (ULONG)trb_fields[2], (ULONG)trb_fields[3]);
		queue_trb(ctrl, ep_ring, TRUE, trb_fields);
	}

	/*
	 * Queue status TRB -
	 * see Table 7 and sections 4.11.2.2 and 6.4.1.2.3
	 */

	/* If the device sent data, the status stage is an OUT transfer */
	field = 0;
	if (length > 0 && io->iouh_SetupData.bmRequestType & USB_DIR_IN)
		field = 0;
	else
		field = TRB_DIR_IN;

	trb_fields[0] = 0;
	trb_fields[1] = 0;
	trb_fields[2] = TRB_INTR_TARGET(0);
	/* Event on completion */
	trb_fields[3] = field | TRB_IOC |
					TRB_TYPE(TRB_STATUS) | ep_ring->cycle_state;

	KprintfH("status TRB fields: %08lx %08lx %08lx %08lx\n",
			 (ULONG)trb_fields[0], (ULONG)trb_fields[1], (ULONG)trb_fields[2], (ULONG)trb_fields[3]);
	dma_addr_t event_trb = queue_trb(ctrl, ep_ring, FALSE, trb_fields);

	giveback_first_trb(udev, ep_index, start_cycle, start_trb);

	xhci_ep_set_receiving(udev, io, USB_DEV_EP_STATE_RECEIVING_CONTROL, event_trb, timeout_ms);
	return UHIOERR_NO_ERROR;
}
