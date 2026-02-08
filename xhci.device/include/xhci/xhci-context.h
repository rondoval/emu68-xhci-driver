#ifndef __XHCI_CONTEXT_H
#define __XHCI_CONTEXT_H

#include <compat.h>

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
	unsigned type;
#define XHCI_CTX_TYPE_DEVICE 0x1
#define XHCI_CTX_TYPE_INPUT 0x2

	int size;
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
#define ROUTE_STRING_MASK (0xfffff)
/* Device speed - values defined by PORTSC Device Speed field - 20:23 */
#define DEV_SPEED (0xf << 20)
/* bit 24 reserved */
/* Is this LS/FS device connected through a HS hub? - bit 25 */
#define DEV_MTT (0x1 << 25)
/* Set if the device is a hub - bit 26 */
#define DEV_HUB (0x1 << 26)
/* Index of the last valid endpoint context in this device context - 27:31 */
#define LAST_CTX_MASK (0x1f << 27)
#define LAST_CTX(p) ((p) << 27)
#define LAST_CTX_TO_EP_NUM(p) (((p) >> 27) - 1)
#define SLOT_FLAG (1 << 0)
#define EP0_FLAG (1 << 1)

/* dev_info2 bitmasks */
/* Max Exit Latency (ms) - worst case time to wake up all links in dev path */
#define MAX_EXIT (0xffff)
/* Root hub port number that is needed to access the USB device */
#define ROOT_HUB_PORT(p) (((p) & 0xff) << 16)
#define ROOT_HUB_PORT_MASK (0xff)
#define ROOT_HUB_PORT_SHIFT (16)
#define DEVINFO_TO_ROOT_HUB_PORT(p) (((p) >> 16) & 0xff)
/* Maximum number of ports under a hub device */
#define XHCI_MAX_PORTS(p) (((p) & 0xff) << 24)

/* tt_info bitmasks */
/*
 * TT Hub Slot ID - for low or full speed devices attached to a high-speed hub
 * The Slot ID of the hub that isolates the high speed signaling from
 * this low or full-speed device.  '0' if attached to root hub port.
 */
#define TT_SLOT(p) (((p) & 0xff) << 0)
/*
 * The number of the downstream facing port of the high-speed hub
 * '0' if the device is not low or full speed.
 */
#define TT_PORT(p) (((p) & 0xff) << 8)
#define TT_THINK_TIME(p) (((p) & 0x3) << 16)

/* dev_state bitmasks */
/* USB device address - assigned by the HC */
#define DEV_ADDR_MASK (0xff)
/* bits 8:26 reserved */
/* Slot state */
#define SLOT_STATE (0x1f << 27)
#define GET_SLOT_STATE(p) (((p) & (0x1f << 27)) >> 27)

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
#define EP_MULT(p) (((p) & 0x3) << 8)
#define CTX_TO_EP_MULT(p) (((p) >> 8) & 0x3)
/* bits 10:14 are Max Primary Streams */
/* bit 15 is Linear Stream Array */
/* Interval - period between requests to an endpoint - 125u increments. */
#define EP_INTERVAL(p) (((p) & 0xff) << 16)
#define EP_INTERVAL_TO_UFRAMES(p) (1 << (((p) >> 16) & 0xff))
#define CTX_TO_EP_INTERVAL(p) (((p) >> 16) & 0xff)
#define EP_MAXPSTREAMS_MASK (0x1f << 10)
#define EP_MAXPSTREAMS(p) (((p) << 10) & EP_MAXPSTREAMS_MASK)
/* Endpoint is set up with a Linear Stream Array (vs. Secondary Stream Array) */
#define EP_HAS_LSA (1 << 15)

/* ep_info2 bitmasks */
/*
 * Force Event - generate transfer events for all TRBs for this endpoint
 * This will tell the HC to ignore the IOC and ISP flags (for debugging only).
 */
#define FORCE_EVENT (0x1)
#define ERROR_COUNT(p) (((p) & 0x3) << 1)
#define CTX_TO_EP_TYPE(p) (((p) >> 3) & 0x7)
#define EP_TYPE(p) ((p) << 3)
#define ISOC_OUT_EP 1
#define BULK_OUT_EP 2
#define INT_OUT_EP 3
#define CTRL_EP 4
#define ISOC_IN_EP 5
#define BULK_IN_EP 6
#define INT_IN_EP 7
/* bit 6 reserved */
/* bit 7 is Host Initiate Disable - for disabling stream selection */
#define MAX_BURST(p) (((p) & 0xff) << 8)
#define CTX_TO_MAX_BURST(p) (((p) >> 8) & 0xff)
#define MAX_PACKET(p) (((p) & 0xffff) << 16)
#define MAX_PACKET_MASK (0xffff)
#define MAX_PACKET_DECODED(p) (((p) >> 16) & 0xffff)

/* Get max packet size from ep desc. Bit 10..0 specify the max packet size.
 * USB2.0 spec 9.6.6.
 */
#define GET_MAX_PACKET(p) ((p) & 0x7ff)

/* tx_info bitmasks */
#define EP_AVG_TRB_LENGTH(p) ((p) & 0xffff)
#define EP_MAX_ESIT_PAYLOAD_LO(p) (((p) & 0xffff) << 16)
#define EP_MAX_ESIT_PAYLOAD_HI(p) ((((p) >> 16) & 0xff) << 24)
#define CTX_TO_MAX_ESIT_PAYLOAD(p) (((p) >> 16) & 0xffff)

/* deq bitmasks */
#define EP_CTX_CYCLE_MASK (1 << 0)

/* reserved[0] bitmasks, MediaTek xHCI used */
#define EP_BPKTS(p) (((p) & 0x7f) << 0)
#define EP_BBM(p) (((p) & 0x1) << 11)

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

struct xhci_container_ctx *xhci_alloc_container_ctx(struct xhci_ctrl *ctrl, int type);
void xhci_free_container_ctx(struct xhci_ctrl *ctrl, struct xhci_container_ctx *ctx);

u32 xhci_get_hardware_address(struct usb_device *udev);

void xhci_setup_addressable_virt_dev(struct xhci_ctrl *ctrl, struct usb_device *udev);
void xhci_update_hub_tt(struct usb_device *udev);

int xhci_check_maxpacket(struct usb_device *udev, unsigned int max_packet_size);
int xhci_set_configuration(struct usb_device *udev, int config_value);
int xhci_set_interface(struct usb_device *udev, unsigned int iface_number, unsigned int alt_setting);

void xhci_dump_ep_ctx(const char *tag, struct usb_device *udev, UBYTE ep_index);
void xhci_dump_slot_ctx(const char *tag, struct usb_device *udev, BOOL in_ctx);

#endif /* __XHCI_CONTEXT_H */