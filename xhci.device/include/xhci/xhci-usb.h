#ifndef __XHCI_USB_H__
#define __XHCI_USB_H___

#include <devices/usbhardware.h>
#include <compat.h>

u32 xhci_ep_index_from_parts(u8 iouh_endpoint, u8 iouh_dir);
int xhci_ep_type_for_index(struct usb_device *udev, u32 ep_index);
void xhci_usb_parse_control_message(struct usb_device *udev, struct IOUsbHWReq *io);


int xhci_set_interface(struct usb_device *udev, unsigned int interface_number, unsigned int alt_setting);
int xhci_set_configuration(struct usb_device *udev, int config_value);
int xhci_check_maxpacket(struct usb_device *udev, unsigned int maxpacket);

#endif