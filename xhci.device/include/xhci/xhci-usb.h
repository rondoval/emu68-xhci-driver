#ifndef __XHCI_USB_H__
#define __XHCI_USB_H___

#include <devices/usbhardware.h>
#include <compat.h>

inline int xhci_ep_index_from_parts(UWORD iouh_endpoint, UWORD iouh_dir)
{
    UBYTE ep = iouh_endpoint & 0x0F;
    if (ep == 0)
        return 0;
    BOOL dir_in = (iouh_dir == UHDIR_IN);
    return (ep << 1) - (dir_in ? 0 : 1);
}

inline UBYTE xhci_ep_index_to_address(u32 ep_index)
{
    if (ep_index == 0)
        return 0;
    return EP_INDEX_TO_ENDPOINT(ep_index) | ((ep_index & 0x1) ? USB_DIR_OUT : USB_DIR_IN);
}

inline static int xhci_address_to_ep_index(const struct usb_endpoint_descriptor *descriptor)
{
    int ep_num = usb_endpoint_num(descriptor);
    if (ep_num == 0)
        return 0;
    return (ep_num << 1) - (usb_endpoint_dir_in(descriptor) ? 0 : 1);
}

int xhci_ep_type_for_index(struct usb_device *udev, u32 ep_index);
void xhci_usb_parse_control_message(struct usb_device *udev, struct IOUsbHWReq *io);


int xhci_set_interface(struct usb_device *udev, unsigned int interface_number, unsigned int alt_setting);
int xhci_set_configuration(struct usb_device *udev, int config_value);
int xhci_check_maxpacket(struct usb_device *udev, unsigned int maxpacket);

#endif