/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef XHCI_COMMANDS_H
#define XHCI_COMMANDS_H

#include <xhci/xhci-udev.h>

union xhci_trb;

void xhci_dispatch_command_event(struct xhci_ctrl *ctrl, union xhci_trb *event);
void xhci_process_command_timeouts(struct xhci_ctrl *ctrl);

void xhci_reset_ep(struct usb_device *udev, u32 ep_index);
void xhci_stop_ring(struct usb_device *udev, u32 ep_index);
void xhci_configure_endpoints(struct usb_device *udev, BOOL ctx_change, struct USBIORequest *req);
void xhci_address_device(struct usb_device *udev, struct USBIORequest *req);
void xhci_reset_device(struct usb_device *udev);
void xhci_set_deq_pointer(struct usb_device *udev, u32 ep_index, u32 deq_ptr);
void xhci_disable_slot(struct usb_device *udev);
void xhci_enable_slot(struct usb_device *udev, struct USBIORequest *req);

#endif /* XHCI_COMMANDS_H */