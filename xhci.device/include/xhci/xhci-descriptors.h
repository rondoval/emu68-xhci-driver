#ifndef __XHCI_DESCRIPTORS_H__
#define __XHCI_DESCRIPTORS_H___

#include <devices/usbhardware.h>
#include <compat.h>
#include <xhci/ch9.h>

inline int xhci_ep_index_from_parts(UWORD iouh_endpoint, UWORD iouh_dir)
{
    UBYTE ep = iouh_endpoint & 0x0F;
    if (ep == 0)
        return 0;
    BOOL dir_in = (iouh_dir == UHDIR_IN);
    return (ep << 1) - (dir_in ? 0 : 1);
}

inline static int xhci_address_to_ep_index(const struct usb_endpoint_descriptor *descriptor)
{
    int ep_num = usb_endpoint_num(descriptor);
    if (ep_num == 0)
        return 0;
    return (ep_num << 1) - (usb_endpoint_dir_in(descriptor) ? 0 : 1);
}

struct usb_config *xhci_find_config(struct usb_device *udev, int config_value);
struct usb_interface *xhci_find_interface(struct usb_config *cfg, unsigned int iface_number);
struct usb_interface_altsetting *xhci_find_altsetting(struct usb_interface *iface, unsigned int alt_setting);

struct usb_interface_altsetting *xhci_select_active_alt(struct usb_interface *iface);

unsigned int compute_max_ep_flag(const struct usb_config *cfg);
u32 xhci_collect_ep_mask(const struct usb_interface_altsetting *alt, unsigned int *max_flag);
u32 xhci_collect_config_masks(const struct usb_config *cfg, unsigned int limit, unsigned int *max_flag);

unsigned int xhci_get_ep_index(struct usb_endpoint_descriptor *desc);
unsigned int xhci_get_endpoint_interval(struct usb_device *udev, struct usb_endpoint_descriptor *endpt_desc);
u32 xhci_get_endpoint_mult(struct usb_device *udev, struct usb_endpoint_descriptor *endpt_desc, struct usb_ss_ep_comp_descriptor *ss_ep_comp_desc);
u32 xhci_get_endpoint_max_burst(struct usb_device *udev, struct usb_endpoint_descriptor *endpt_desc, struct usb_ss_ep_comp_descriptor *ss_ep_comp_desc);
u32 xhci_get_max_esit_payload(struct usb_device *udev, struct usb_endpoint_descriptor *endpt_desc, struct usb_ss_ep_comp_descriptor *ss_ep_comp_desc);

void xhci_dump_config(const char *tag, const struct usb_config *cfg, UBYTE addr);

#endif /* __XHCI_DESCRIPTORS_H__ */