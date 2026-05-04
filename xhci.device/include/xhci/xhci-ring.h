/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __XHCI_RING_H
#define __XHCI_RING_H

#include <exec/types.h>
#include <devices/hcd_api.h>
#include <bits.h>
#include <byteorder.h>

struct xhci_transfer_event
{
	/* 64-bit buffer address, or immediate data */
	__le64 buffer;
	__le32 transfer_len;
	/* This field is interpreted differently based on the type of TRB */
	volatile __le32 flags;
};

/* Transfer event TRB length bit mask */
/* bits 0:23 */
#define EVENT_TRB_LEN(p) ((p) & 0xffffffU)

/** Transfer Event bit fields **/
#define TRB_TO_EP_ID(p) (((p) >> 16) & 0x1fU)

/* Completion Code - only applicable for some types of TRBs */
#define COMP_CODE_MASK (0xffU << 24)
#define COMP_CODE_SHIFT (24)
#define GET_COMP_CODE(p) (((p) & COMP_CODE_MASK) >> 24)

typedef enum
{
	COMP_SUCCESS = 1,
	/* Data Buffer Error */
	COMP_DB_ERR, /* 2 */
	/* Babble Detected Error */
	COMP_BABBLE, /* 3 */
	/* USB Transaction Error */
	COMP_TX_ERR, /* 4 */
	/* TRB Error - some TRB field is invalid */
	COMP_TRB_ERR, /* 5 */
	/* Stall Error - USB device is stalled */
	COMP_STALL, /* 6 */
	/* Resource Error - HC doesn't have memory for that device configuration */
	COMP_ENOMEM, /* 7 */
	/* Bandwidth Error - not enough room in schedule for this dev config */
	COMP_BW_ERR, /* 8 */
	/* No Slots Available Error - HC ran out of device slots */
	COMP_ENOSLOTS, /* 9 */
	/* Invalid Stream Type Error */
	COMP_STREAM_ERR, /* 10 */
	/* Slot Not Enabled Error - doorbell rung for disabled device slot */
	COMP_EBADSLT, /* 11 */
	/* Endpoint Not Enabled Error */
	COMP_EBADEP, /* 12 */
	/* Short Packet */
	COMP_SHORT_TX, /* 13 */
	/* Ring Underrun - doorbell rung for an empty isoc OUT ep ring */
	COMP_UNDERRUN, /* 14 */
	/* Ring Overrun - isoc IN ep ring is empty when ep is scheduled to RX */
	COMP_OVERRUN, /* 15 */
	/* Virtual Function Event Ring Full Error */
	COMP_VF_FULL, /* 16 */
	/* Parameter Error - Context parameter is invalid */
	COMP_EINVAL, /* 17 */
	/* Bandwidth Overrun Error - isoc ep exceeded its allocated bandwidth */
	COMP_BW_OVER, /* 18 */
	/* Context State Error - illegal context state transition requested */
	COMP_CTX_STATE, /* 19 */
	/* No Ping Response Error - HC didn't get PING_RESPONSE in time to TX */
	COMP_PING_ERR, /* 20 */
	/* Event Ring is full */
	COMP_ER_FULL, /* 21 */
	/* Incompatible Device Error */
	COMP_DEV_ERR, /* 22 */
	/* Missed Service Error - HC couldn't service an isoc ep within interval */
	COMP_MISSED_INT, /* 23 */
	/* Successfully stopped command ring */
	COMP_CMD_STOP, /* 24 */
	/* Successfully aborted current command and stopped command ring */
	COMP_CMD_ABORT, /* 25 */
	/* Stopped - transfer was terminated by a stop endpoint command */
	COMP_STOP, /* 26 */
	/* Same as COMP_STOP, but the transferred length in the event is invalid */
	COMP_STOP_INVAL, /* 27*/
	/* Same as COMP_STOP, but the transfer was stopped after Short Packet condition */
	COMP_STOP_SHORT, /* 28 */
	/* Max Exit Latency Too Large Error */
	COMP_MEL_ERR, /* 29 */
	/* TRB type 30 reserved */
	/* Isoc Buffer Overrun - an isoc IN ep sent more data than could fit in TD */
	COMP_BUFF_OVER = 31,
	/* Event Lost Error - xHC has an "internal event overrun condition" */
	COMP_ISSUES, /* 32 */
	/* Undefined Error - reported when other error codes don't apply */
	COMP_UNKNOWN, /* 33 */
	/* Invalid Stream ID Error */
	COMP_STRID_ERR, /* 34 */
	/* Secondary Bandwidth Error - may be returned by a Configure Endpoint cmd */
	COMP_2ND_BW_ERR, /* 35 */
	/* Split Transaction Error */
	COMP_SPLIT_ERR /* 36 */

} xhci_comp_code;

/* flags bitmasks */
/* bits 16:23 are the virtual function ID */
/* bits 24:31 are the slot ID */
#define TRB_TO_SLOT_ID(p) (((p) & (0xffU << 24)) >> 24)
#define TRB_TO_SLOT_ID_SHIFT (24)
#define TRB_TO_SLOT_ID_MASK (0xffU << TRB_TO_SLOT_ID_SHIFT)
#define SLOT_ID_FOR_TRB(p) (((u32)(p) & 0xffU) << 24)
#define SLOT_ID_FOR_TRB_MASK (0xffU)
#define SLOT_ID_FOR_TRB_SHIFT (24)

/* Stop Endpoint TRB - ep_index to endpoint ID for this TRB */
#define TRB_TO_EP_INDEX(p) (u8)((((p) & (0x1fU << 16)) >> 16) - 1U)
#define TRB_TO_ENDPOINT(p) ((((p) & (0x1fU << 16)) >> 17))
#define EP_ID_FOR_TRB(p) ((((u32)(p) + 1U) & 0x1fU) << 16)

#define SUSPEND_PORT_FOR_TRB(p) (((u32)(p) & 1U) << 23)
#define TRB_TO_SUSPEND_PORT(p) (((p) & BIT(23)) >> 23)
#define LAST_EP_INDEX 30

/* Set TR Dequeue Pointer command TRB fields */
#define TRB_TO_STREAM_ID(p) ((((p) & (0xffffU << 16)) >> 16))
#define STREAM_ID_FOR_TRB(p) (((u32)(p) & 0xffffU) << 16)

/* Port Status Change Event TRB fields */
/* Port ID - bits 31:24 */
#define GET_PORT_ID(p) (((p) & (0xffU << 24)) >> 24)
#define PORT_ID_SHIFT (24)
#define PORT_ID_MASK (0xffU << PORT_ID_SHIFT)

/* Normal TRB fields */
/* transfer_len bitmasks - bits 0:16 */
#define TRB_LEN(p) ((p) & 0x1ffff)
/* TD Size, packets remaining in this TD, bits 21:17 (5 bits, so max 31) */
#define TRB_TD_SIZE(p) (((p) > 31U ? 31U : (u32)(p)) << 17)
/* Interrupter Target - which MSI-X vector to target the completion event at */
#define TRB_INTR_TARGET(p) (((p) & 0x3ffU) << 22)
#define GET_INTR_TARGET(p) (((p) >> 22) & 0x3ff)
#define TRB_TBC(p) (((p) & 0x3) << 7)
#define TRB_TLBPC(p) (((p) & 0xf) << 16)

/* Cycle bit - indicates TRB ownership by HC or HCD */
#define TRB_CYCLE BIT(0)
/*
 * Force next event data TRB to be evaluated before task switch.
 * Used to pass OS data back after a TD completes.
 */
#define TRB_ENT BIT(1)
/* Interrupt on short packet */
#define TRB_ISP BIT(2)
/* Set PCIe no snoop attribute */
#define TRB_NO_SNOOP BIT(3)
/* Chain multiple TRBs into a TD */
#define TRB_CHAIN BIT(4)
/* Interrupt on completion */
#define TRB_IOC BIT(5)
/* The buffer pointer contains immediate data */
#define TRB_IDT BIT(6)

/* Block Event Interrupt */
#define TRB_BEI BIT(9)

/* Control transfer TRB specific fields */
#define TRB_DIR_IN BIT(16)
#define TRB_TX_TYPE(p) ((u32)(p) << 16)
#define TRB_DATA_OUT 2
#define TRB_DATA_IN 3

/* Isochronous TRB specific fields */
#define TRB_FRAME_ID(p) (((u32)(p) & 0x7ffU) << 20)
#define GET_TRB_FRAME_ID(p) ((u16)(((p) >> 20) & 0x7ffU))
#define TRB_SIA BIT(31)

struct xhci_link_trb
{
	/* 64-bit segment pointer*/
	volatile __le64 segment_ptr;
	volatile __le32 intr_target;
	volatile __le32 control;
};

/* control bitfields */
#define LINK_TOGGLE BIT(1)

/* Command completion event TRB */
struct xhci_event_cmd
{
	/* Pointer to command TRB, or the value passed by the event data trb */
	volatile __le64 cmd_trb;
	volatile __le32 status;
	volatile __le32 flags;
};

struct xhci_generic_trb
{
	volatile __le32 field[4];
};

union xhci_trb
{
	struct xhci_link_trb link;
	struct xhci_transfer_event trans_event;
	struct xhci_event_cmd event_cmd;
	struct xhci_generic_trb generic;
};

/* TRB bit mask */
#define TRB_TYPE_BITMASK (0xfc00U)
#define TRB_TYPE(p) ((u32)(p) << 10)
#define TRB_FIELD_TO_TYPE(p) (((p) & TRB_TYPE_BITMASK) >> 10)

/* TRB type IDs */
typedef enum
{
	/* reserved, used as a software sentinel */
	TRB_NONE = 0,
	/* bulk, interrupt, isoc scatter/gather, and control data stage */
	TRB_NORMAL = 1,
	/* setup stage for control transfers */
	TRB_SETUP, /* 2 */
	/* data stage for control transfers */
	TRB_DATA, /* 3 */
	/* status stage for control transfers */
	TRB_STATUS, /* 4 */
	/* isoc transfers */
	TRB_ISOC, /* 5 */
	/* TRB for linking ring segments */
	TRB_LINK, /* 6 */
	/* TRB for EVENT DATA */
	TRB_EVENT_DATA, /* 7 */
	/* Transfer Ring No-op (not for the command ring) */
	TRB_TR_NOOP, /* 8 */
	/* Command TRBs */
	/* Enable Slot Command */
	TRB_ENABLE_SLOT, /* 9 */
	/* Disable Slot Command */
	TRB_DISABLE_SLOT, /* 10 */
	/* Address Device Command */
	TRB_ADDR_DEV, /* 11 */
	/* Configure Endpoint Command */
	TRB_CONFIG_EP, /* 12 */
	/* Evaluate Context Command */
	TRB_EVAL_CONTEXT, /* 13 */
	/* Reset Endpoint Command */
	TRB_RESET_EP, /* 14 */
	/* Stop Transfer Ring Command */
	TRB_STOP_RING, /* 15 */
	/* Set Transfer Ring Dequeue Pointer Command */
	TRB_SET_DEQ, /* 16 */
	/* Reset Device Command */
	TRB_RESET_DEV, /* 17 */
	/* Force Event Command (opt) */
	TRB_FORCE_EVENT, /* 18 */
	/* Negotiate Bandwidth Command (opt) */
	TRB_NEG_BANDWIDTH, /* 19 */
	/* Set Latency Tolerance Value Command (opt) */
	TRB_SET_LT, /* 20 */
	/* Get port bandwidth Command */
	TRB_GET_BW, /* 21 */
	/* Force Header Command - generate a transaction or link management packet */
	TRB_FORCE_HEADER, /* 22 */
	/* No-op Command - not for transfer rings */
	TRB_CMD_NOOP, /* 23 */
	/* TRB IDs 24-31 reserved */
	/* Event TRBS */
	/* Transfer Event */
	TRB_TRANSFER = 32,
	/* Command Completion Event */
	TRB_COMPLETION, /* 33 */
	/* Port Status Change Event */
	TRB_PORT_STATUS, /* 34 */
	/* Bandwidth Request Event (opt) */
	TRB_BANDWIDTH_EVENT, /* 35 */
	/* Doorbell Event (opt) */
	TRB_DOORBELL, /* 36 */
	/* Host Controller Event */
	TRB_HC_EVENT, /* 37 */
	/* Device Notification Event - device sent function wake notification */
	TRB_DEV_NOTE, /* 38 */
	/* MFINDEX Wrap Event - microframe counter wrapped */
	TRB_MFINDEX_WRAP, /* 39 */
	/* TRB IDs 40-47 reserved, 48-63 is vendor-defined */
	/* Nec vendor-specific command completion event. */
	TRB_NEC_CMD_COMP = 48, /* 48 */
	/* Get NEC firmware revision. */
	TRB_NEC_GET_FW, /* 49 */
} trb_type;

#define TRB_TYPE_LINK(x) (((x) & TRB_TYPE_BITMASK) == TRB_TYPE(TRB_LINK))
/* Above, but for __le32 types -- can avoid work by swapping constants: */
#define TRB_TYPE_LINK_LE32(x) (((x) & le32(TRB_TYPE_BITMASK)) == \
							   le32(TRB_TYPE(TRB_LINK)))
#define TRB_TYPE_NOOP_LE32(x) (((x) & le32(TRB_TYPE_BITMASK)) == \
							   le32(TRB_TYPE(TRB_TR_NOOP)))

struct xhci_ctrl;
struct xhci_erst;
struct xhci_intr_reg;
struct usb_device;
struct ep_context;

struct xhci_ring *xhci_ring_alloc(struct xhci_ctrl *ctrl, u32 num_segs,
							  BOOL link_trbs, BOOL is_event_ring, u8 ep_index, u32 max_packet_size);
void xhci_ring_free(struct xhci_ctrl *ctrl, struct xhci_ring *ring);								  

s8 xhci_ring_enqueue_td(struct usb_device *udev, struct USBIORequest *io, u32 timeout_ms, BOOL defer_doorbell);
s8 xhci_ring_enqueue_td_at_frame(struct usb_device *udev, struct USBIORequest *io, u32 timeout_ms, BOOL defer_doorbell, u16 frame);
BOOL xhci_ring_has_room(struct ep_context *ep_ctx, u32 needed_trbs);
void xhci_ring_giveback(struct usb_device *udev, struct ep_context *ep_ctx);

void xhci_ring_acknowledge_event(struct xhci_ctrl *ctrl);
union xhci_trb *xhci_ring_get_event_trb(struct xhci_ring *ring);

u32 xhci_ring_get_new_dequeue_ptr(struct xhci_ring *ring);
u32 xhci_ring_get_deq_ptr_for_trb(dma_addr_t trb_addr);

u32 xhci_ring_get_max_packet_size(struct xhci_ring *ring);
void xhci_ring_set_max_packet_size(struct xhci_ring *ring, u32 max_packet_size);
void xhci_ring_patch_trbs_to_noop(dma_addr_t *trb_addrs, u32 trb_count, u32 start_index);

dma_addr_t xhci_ring_enqueue_command(struct xhci_ring *ring, u64 address, u32 slot_id, u8 ep_index, trb_type cmd);
void xhci_ring_setup_erst(struct xhci_ring *ring, struct xhci_erst *erst, struct xhci_intr_reg *ir_set);

#endif /* __XHCI_RING_H */