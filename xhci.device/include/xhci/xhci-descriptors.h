/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __XHCI_DESCRIPTORS_H__
#define __XHCI_DESCRIPTORS_H__

#include <devices/hcd_api.h>
#include <types.h>
#include <xhci/ch9.h>

struct usb_device;

inline u8 xhci_ep_index_from_parts(u16 endpoint, u16 direction)
{
    u8 ep = endpoint & 0x0F;
    if (ep == 0)
        return 0;
    BOOL dir_in = (direction == DIRECTION_IN);
    return (u8)(((u32)ep << 1) - (u32)(dir_in ? 0 : 1));
}

inline static u8 xhci_address_to_ep_index(const struct usb_endpoint_descriptor *descriptor)
{
    u32 ep_num = usb_endpoint_num(descriptor);
    if (ep_num == 0)
        return 0;
    return (u8)((ep_num << 1) - (u32)(usb_endpoint_dir_in(descriptor) ? 0 : 1));
}

/* Parse a configuration descriptor blob into a usb_config and store it on the
 * device (replacing any prior config with the same bConfigurationValue). */
void xhci_parse_config_descriptor(struct usb_device *udev, u8 *data, u16 len);

struct usb_config *xhci_find_config(struct usb_device *udev, int config_value);
struct usb_interface *xhci_find_interface(struct usb_config *cfg, u32 iface_number);
struct usb_interface_altsetting *xhci_find_altsetting(struct usb_interface *iface, u8 alt_setting);

struct usb_interface_altsetting *xhci_select_active_alt(struct usb_interface *iface);

u32 compute_max_ep_flag(const struct usb_config *cfg);
u32 xhci_collect_ep_mask(const struct usb_interface_altsetting *alt, u32 *max_flag);
u32 xhci_collect_config_masks(const struct usb_config *cfg, u32 limit, u32 *max_flag);

u8 xhci_get_ep_index(struct usb_endpoint_descriptor *desc);
u8 xhci_get_endpoint_interval(struct usb_device *udev, struct usb_endpoint_descriptor *endpt_desc);
u8 xhci_get_endpoint_mult(struct usb_device *udev, struct usb_endpoint_descriptor *endpt_desc, struct usb_ss_ep_comp_descriptor *ss_ep_comp_desc);
u8 xhci_get_endpoint_max_burst(struct usb_device *udev, struct usb_endpoint_descriptor *endpt_desc, struct usb_ss_ep_comp_descriptor *ss_ep_comp_desc);
u32 xhci_get_max_esit_payload(struct usb_device *udev, struct usb_endpoint_descriptor *endpt_desc, struct usb_ss_ep_comp_descriptor *ss_ep_comp_desc);

void xhci_dump_config(const char *tag, const struct usb_config *cfg, u16 addr);

#endif /* __XHCI_DESCRIPTORS_H__ */