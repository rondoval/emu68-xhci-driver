#ifndef __XHCI_ROOT_HUB_H
#define __XHCI_ROOT_HUB_H

#include <devices/usbhardware.h>

struct xhci_root_hub;

typedef void (*io_reply_data_fn)(struct usb_device *udev, struct IOUsbHWReq *io, int err, ULONG actual);

struct xhci_root_hub *xhci_roothub_create(struct usb_device *udev,
                                         io_reply_data_fn io_reply_data);
void xhci_roothub_destroy(struct xhci_root_hub *rh);

void xhci_roothub_submit_ctrl_request(struct xhci_root_hub *rh, struct IOUsbHWReq *req);
BOOL xhci_roothub_submit_int_request(struct xhci_root_hub *rh, struct IOUsbHWReq *req);
void xhci_roothub_complete_int_request(struct xhci_root_hub *rh);
unsigned int xhci_roothub_get_address(struct xhci_root_hub *rh);

#endif