/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __XHCI_DESCRIPTORS_H__
#define __XHCI_DESCRIPTORS_H__

#include <xhci/xhci-xfer.h>
#include <types.h>
#include <devices/usbhcd_context.h> /* struct UhcdEndpointDesc, UHCD_EPTYPE_* */

struct usb_device;

static inline u8 xhci_ep_index_from_parts(u16 endpoint, u16 direction)
{
    u8 ep = endpoint & 0x0F;
    if (ep == 0)
        return 0;
    BOOL dir_in = (direction == XHCI_DIR_IN);
    return (u8)(((u32)ep << 1) - (u32)(dir_in ? 0 : 1));
}

/* DCI index straight from a USB endpoint address (bEndpointAddress layout:
 * bit 7 = IN, bits 3:0 = endpoint number). */
static inline u8 xhci_ep_index_from_address(u8 addr)
{
    return xhci_ep_index_from_parts(addr & 0x0f,
                                    (addr & 0x80) ? XHCI_DIR_IN : XHCI_DIR_OUT);
}

/* ---- struct UhcdEndpointDesc field interpreters --------------------------
 * The context ABI delivers endpoint facts already parsed by the stack, in host
 * byte order.  Read them straight (no synthetic wire descriptor, no byteswap);
 * ed_Type numerically equals the USB bmAttributes transfer type. */

static inline u8 ed_xfer_type(const struct UhcdEndpointDesc *ed)
{
    return ed->ed_Type & 0x3;
}
static inline BOOL ed_is_control(const struct UhcdEndpointDesc *ed) { return ed_xfer_type(ed) == UHCD_EPTYPE_CONTROL; }
static inline BOOL ed_is_isoc(const struct UhcdEndpointDesc *ed)    { return ed_xfer_type(ed) == UHCD_EPTYPE_ISO; }
static inline BOOL ed_is_bulk(const struct UhcdEndpointDesc *ed)    { return ed_xfer_type(ed) == UHCD_EPTYPE_BULK; }
static inline BOOL ed_is_int(const struct UhcdEndpointDesc *ed)     { return ed_xfer_type(ed) == UHCD_EPTYPE_INTERRUPT; }

static inline u8   ed_num(const struct UhcdEndpointDesc *ed)      { return ed->ed_Address & 0x0f; }
static inline BOOL ed_dir_in(const struct UhcdEndpointDesc *ed)   { return (ed->ed_Address & 0x80) != 0; }
static inline u16  ed_maxp(const struct UhcdEndpointDesc *ed)     { return ed->ed_MaxPacket & 0x7ff; }
/* wMaxPacketSize[12:11] + 1 — HS periodic extra transactions per uframe */
static inline u8   ed_maxp_mult(const struct UhcdEndpointDesc *ed){ return (u8)(((ed->ed_MaxPacket >> 11) & 0x3) + 1); }

static inline u8 xhci_ep_index(const struct UhcdEndpointDesc *ed)
{
    return xhci_ep_index_from_parts(ed_num(ed), ed_dir_in(ed) ? XHCI_DIR_IN : XHCI_DIR_OUT);
}

#endif /* __XHCI_DESCRIPTORS_H__ */
