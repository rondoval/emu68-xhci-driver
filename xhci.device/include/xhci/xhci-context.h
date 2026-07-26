/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __XHCI_CONTEXT_H
#define __XHCI_CONTEXT_H

struct xhci_xfer;
struct xhci_ctrl;

#include <types.h>
#include <bits.h>
#include <xhci/ch9.h> /* enum usb_device_speed */

/**
 * struct xhci_container_ctx
 * @type: Type of context.  Used to calculated offsets to contained contexts.
 * @size: Size of the context data
 * @bytes: The raw context data given to HW
 *
 * Represents either a Device or Input context.  Holds a pointer to the raw
 * memory used for the context (bytes).
 */
struct xhci_container_ctx
{
	u32 type;
#define XHCI_CTX_TYPE_DEVICE 0x1
#define XHCI_CTX_TYPE_INPUT 0x2

	u32 size;
	u8 *bytes;
};

/**
 * struct xhci_slot_ctx
 * @dev_info:	Route string, device speed, hub info, and last valid endpoint
 * @dev_info2:	Max exit latency for device number, root hub port number
 * @tt_info:	tt_info is used to construct split transaction tokens
 * @dev_state:	slot state and device address
 *
 * Slot Context - section 6.2.1.1.  This assumes the HC uses 32-byte context
 * structures.  If the HC uses 64-byte contexts, there is an additional 32 bytes
 * reserved at the end of the slot context for HC internal use.
 */
struct xhci_slot_ctx
{
	__le32 dev_info;
	__le32 dev_info2;
	__le32 tt_info;
	__le32 dev_state;
	/* offset 0x10 to 0x1f reserved for HC internal use */
	__le32 reserved[4];
};

/* dev_info bitmasks */
/* Route String - 0:19 */
#define ROUTE_STRING_MASK (0xfffffU)
/* Device speed - values defined by PORTSC Device Speed field - 20:23 */
#define DEV_SPEED (0xfU << 20)
/* bit 24 reserved */
/* Is this LS/FS device connected through a HS hub? - bit 25 */
#define DEV_MTT (0x1U << 25)
/* Set if the device is a hub - bit 26 */
#define DEV_HUB (0x1U << 26)
/* Index of the last valid endpoint context in this device context - 27:31 */
#define LAST_CTX_MASK (0x1fU << 27)
#define LAST_CTX(p) ((u32)(p) << 27)
#define SLOT_FLAG BIT(0)
#define EP0_FLAG BIT(1)

/* dev_info2 bitmasks */
/* Max Exit Latency (µs) - worst case time to wake up all links in dev path */
#define MAX_EXIT (0xffffU)
/* Root hub port number that is needed to access the USB device */
#define ROOT_HUB_PORT(p) (((u32)(p) & 0xffU) << 16)
#define ROOT_HUB_PORT_MASK (0xffU)
#define ROOT_HUB_PORT_SHIFT (16)
#define DEVINFO_TO_ROOT_HUB_PORT(p) (((p) >> 16) & 0xff)
/* Maximum number of ports under a hub device */
#define XHCI_MAX_PORTS(p) (((u32)(p) & 0xffU) << 24)

/* tt_info bitmasks */
/*
 * TT Hub Slot ID - for low or full speed devices attached to a high-speed hub
 * The Slot ID of the hub that isolates the high speed signaling from
 * this low or full-speed device.  '0' if attached to root hub port.
 */
#define TT_SLOT(p) (((u32)(p) & 0xffU) << 0)
/*
 * The number of the downstream facing port of the high-speed hub
 * '0' if the device is not low or full speed.
 */
#define TT_PORT(p) (((u32)(p) & 0xffU) << 8)
#define TT_THINK_TIME(p) (((u32)(p) & 0x3U) << 16)

/* dev_state bitmasks */
/* USB device address - assigned by the HC */
#define DEV_ADDR_MASK (0xffU)
/* bits 8:26 reserved */
/* Slot state */
#define SLOT_STATE (0x1fU << 27)
#define GET_SLOT_STATE(p) (((p) & (0x1fU << 27)) >> 27)

#define SLOT_STATE_DISABLED 0
#define SLOT_STATE_ENABLED SLOT_STATE_DISABLED
#define SLOT_STATE_DEFAULT 1
#define SLOT_STATE_ADDRESSED 2
#define SLOT_STATE_CONFIGURED 3

/**
 * struct xhci_ep_ctx
 * @ep_info:	endpoint state, streams, mult, and interval information.
 * @ep_info2:	information on endpoint type, max packet size, max burst size,
 *		error count, and whether the HC will force an event for all
 *		transactions.
 * @deq:	64-bit ring dequeue pointer address.  If the endpoint only
 *		defines one stream, this points to the endpoint transfer ring.
 *		Otherwise, it points to a stream context array, which has a
 *		ring pointer for each flow.
 * @tx_info:
 *		Average TRB lengths for the endpoint ring and
 *		max payload within an Endpoint Service Interval Time (ESIT).
 *
 * Endpoint Context - section 6.2.1.2.This assumes the HC uses 32-byte context
 * structures.If the HC uses 64-byte contexts, there is an additional 32 bytes
 * reserved at the end of the endpoint context for HC internal use.
 */
struct xhci_ep_ctx
{
	__le32 ep_info;
	__le32 ep_info2;
	__le64 deq;
	__le32 tx_info;
	/* offset 0x14 - 0x1f reserved for HC internal use */
	__le32 reserved[3];
};

/* ep_info bitmasks */
/*
 * Endpoint State - bits 0:2
 * 0 - disabled
 * 1 - running
 * 2 - halted due to halt condition - ok to manipulate endpoint ring
 * 3 - stopped
 * 4 - TRB error
 * 5-7 - reserved
 */
#define EP_STATE_MASK (0xf)
#define EP_STATE_DISABLED 0
#define EP_STATE_RUNNING 1
#define EP_STATE_HALTED 2
#define EP_STATE_STOPPED 3
#define EP_STATE_ERROR 4
/* Mult - Max number of burtst within an interval, in EP companion desc. */
#define EP_MULT(p) (((u32)(p) & 0x3U) << 8)
#define CTX_TO_EP_MULT(p) (((p) >> 8) & 0x3)
/* bits 10:14 are Max Primary Streams */
/* bit 15 is Linear Stream Array */
/* Interval - period between requests to an endpoint - 125u increments. */
#define EP_INTERVAL(p) (((u32)(p) & 0xffU) << 16)
#define CTX_TO_EP_INTERVAL(p) (((p) >> 16) & 0xff)
#define EP_MAXPSTREAMS_MASK (0x1fU << 10)
#define EP_MAXPSTREAMS(p) (((u32)(p) << 10) & EP_MAXPSTREAMS_MASK)
/* Endpoint is set up with a Linear Stream Array (vs. Secondary Stream Array) */
#define EP_HAS_LSA BIT(15)

/* ep_info2 bitmasks */
/*
 * Force Event - generate transfer events for all TRBs for this endpoint
 * This will tell the HC to ignore the IOC and ISP flags (for debugging only).
 */
#define FORCE_EVENT (0x1U)
#define ERROR_COUNT(p) (((u32)(p) & 0x3U) << 1)
#define CTX_TO_EP_TYPE(p) (((p) >> 3) & 0x7)
#define EP_TYPE(p) ((u32)(p) << 3)
#define ISOC_OUT_EP 1
#define BULK_OUT_EP 2
#define INT_OUT_EP 3
#define CTRL_EP 4
#define ISOC_IN_EP 5
#define BULK_IN_EP 6
#define INT_IN_EP 7
/* bit 6 reserved */
/* bit 7 is Host Initiate Disable - for disabling stream selection */
#define MAX_BURST(p) (((u32)(p) & 0xffU) << 8)
#define CTX_TO_MAX_BURST(p) (((p) >> 8) & 0xff)
#define MAX_PACKET(p) (((p) & 0xffff) << 16)
#define MAX_PACKET_MASK (0xffff)
#define MAX_PACKET_DECODED(p) (((p) >> 16) & 0xffff)

/* tx_info bitmasks */
#define EP_AVG_TRB_LENGTH(p) ((p) & 0xffff)
#define EP_MAX_ESIT_PAYLOAD_LO(p) (((p) & 0xffff) << 16)
#define EP_MAX_ESIT_PAYLOAD_HI(p) ((((p) >> 16) & 0xff) << 24)

/* deq bitmasks */
#define EP_CTX_CYCLE_MASK BIT(0)

/**
 * struct xhci_stream_ctx - one entry of a Primary Stream Context Array
 * (section 6.2.4.1; linear arrays only — secondary arrays are not used).
 * @stream_ring: 64-bit stream ring dequeue pointer | SCT | DCS
 */
struct xhci_stream_ctx
{
	__le64 stream_ring;
	__le32 reserved[2];
};

/* stream_ring bitmasks: SCT (bits 3:1) — 1 = entry points at a Primary
 * Transfer Ring (the only type a linear stream array carries) */
#define SCT_FOR_CTX(p) (((u32)(p) & 0x7U) << 1)
#define SCT_PRI_TR 1

struct usb_device;

/**
 * struct xhci_input_control_context
 * Input control context; see section 6.2.5.
 *
 * @drop_context:	set the bit of the endpoint context you want to disable
 * @add_context:	set the bit of the endpoint context you want to enable
 */
struct xhci_input_control_ctx
{
	volatile __le32 drop_flags;
	volatile __le32 add_flags;
	__le32 rsvd2[6];
};

struct xhci_container_ctx *xhci_alloc_container_ctx(struct xhci_ctrl *ctrl, u32 type);
void xhci_free_container_ctx(struct xhci_ctrl *ctrl, struct xhci_container_ctx *ctx);

u32 xhci_get_hardware_address(struct usb_device *udev);
u32 xhci_read_hw_ep_state(struct usb_device *udev, u8 ep_index);
u32 xhci_read_hw_ep_type(struct usb_device *udev, u8 ep_index);
u32 xhci_read_hw_ep_interval(struct usb_device *udev, u8 ep_index);
u64 xhci_get_endpoint_deq_ptr(struct usb_device *udev, u8 ep_index);

void xhci_setup_addressable_virt_dev(struct usb_device *udev);

/* Walk the device tree to the root-hub port this device hangs off. */
u32 xhci_find_root_port(struct usb_device *udev);

void xhci_update_mel_in_input_ctx(struct usb_device *udev);
/* Build an input slot context and issue Evaluate Context to latch MAX_EXIT.
 * req (may be NULL) is the context-ABI op replied from the command completion. */
void xhci_evaluate_mel(struct usb_device *udev, struct xhci_xfer *req);

/* Returns TRUE when an Evaluate Context was issued (req then belongs to the
 * completion path), FALSE when the hardware already matched. */
BOOL xhci_update_maxpacket(struct usb_device *udev, u16 max_packet_size, struct xhci_xfer *req);

/* Context-ABI (NSCMD_USB_*) entry points — input contexts built from the
 * stack-supplied endpoint list, no descriptor model involved. */
struct UhcdEndpointDesc;
s8 xhci_configure_endpoints_from_list(struct usb_device *udev,
                                      const struct UhcdEndpointDesc *add, u16 num_add,
                                      const u8 *drop_addresses, u16 num_drop,
                                      struct xhci_xfer *req);
void xhci_deconfigure(struct usb_device *udev, struct xhci_xfer *req);
void xhci_apply_hub_update(struct usb_device *udev, struct xhci_xfer *req);

/* SS bulk streams (NSCMD_USB_ALLOC/FREE_STREAMS): issue the Configure Endpoint
 * that switches the endpoint context into stream mode (enable: MaxPStreams +
 * LSA, deq = the pre-built stream context array) or back to the default single
 * ring.  The ep_context's stream state must already be built (alloc) or still
 * present (free — destroyed by the op completion). */
void xhci_configure_ep_stream_mode(struct usb_device *udev, u8 ep_index, BOOL enable, struct xhci_xfer *req);

/* Default EP0 max packet size for a device speed; 0 = unknown speed. */
u16 xhci_ep0_default_mps(enum usb_device_speed speed);

/* Context dumps are debug-only; compiled out (calls included) without DEBUG. */
#ifdef DEBUG
void xhci_dump_ep_ctx(const char *tag, struct usb_device *udev, u8 ep_index);
void xhci_dump_slot_ctx(const char *tag, struct usb_device *udev, BOOL in_ctx);
#else
#define xhci_dump_ep_ctx(tag, udev, ep_index) ((void)0)
#define xhci_dump_slot_ctx(tag, udev, in_ctx) ((void)0)
#endif

#endif /* __XHCI_CONTEXT_H */