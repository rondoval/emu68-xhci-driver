#ifndef __XHCI_USB_H__
#define __XHCI_USB_H___

#include <devices/usbhardware.h>
#include <compat.h>

void xhci_usb_parse_control_message(struct usb_device *udev, struct IOUsbHWReq *io);
u32 xhci_ep_index_from_parts(u8 iouh_endpoint, u8 iouh_dir);
int xhci_ep_type_for_index(struct usb_device *udev, u32 ep_index);

#endif