/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __XHCI_ROOT_HUB_H
#define __XHCI_ROOT_HUB_H

#include <devices/hcd_api.h>

struct xhci_root_hub;

typedef void (*io_reply_data_fn)(struct usb_device *udev, struct USBIORequest *io, s8 err, ULONG actual);

struct xhci_root_hub *xhci_roothub_create(struct usb_device *udev,
                                         io_reply_data_fn io_reply_data);
void xhci_roothub_destroy(struct xhci_root_hub *rh);

void xhci_roothub_submit_ctrl_request(struct xhci_root_hub *rh, struct USBIORequest *req);
s8 xhci_roothub_submit_int_request(struct xhci_root_hub *rh, struct USBIORequest *req);
void xhci_roothub_complete_int_request(struct xhci_root_hub *rh);
void xhci_roothub_abort_int_request(struct xhci_root_hub *rh);

u16 xhci_roothub_get_address(struct xhci_root_hub *rh);
u8 xhci_roothub_get_num_ports(struct xhci_root_hub *rh);

#endif