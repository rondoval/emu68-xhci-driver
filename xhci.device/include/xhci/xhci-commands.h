#ifndef XHCI_COMMANDS_H
#define XHCI_COMMANDS_H

#include <exec/lists.h>
#include <compat.h>
#include <xhci/xhci.h>

void xhci_dispatch_command_event(struct xhci_ctrl *ctrl, union xhci_trb *event);

void xhci_reset_ep(struct usb_device *udev, u32 ep_index);
void xhci_stop_endpoint(struct usb_device *udev, u32 ep_index);
void xhci_configure_endpoints(struct usb_device *udev, BOOL ctx_change, struct IOUsbHWReq *req);
void xhci_address_device(struct usb_device *udev, struct IOUsbHWReq *req);
void xhci_reset_device(struct usb_device *udev);
void xhci_set_deq_pointer(struct usb_device *udev, u32 ep_index);
void xhci_disable_slot(struct usb_device *udev);
void xhci_enable_slot(struct usb_device *udev, struct IOUsbHWReq *req);

#endif /* XHCI_COMMANDS_H */