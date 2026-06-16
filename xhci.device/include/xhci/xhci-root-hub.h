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
void xhci_roothub_port_lpm_caps(struct xhci_root_hub *rh, u32 port, BOOL *hw_lpm, BOOL *besl_lpm);

/* Program LPM timing into a 1-based root-hub port's PORTPMSC/PORTHLPMC registers.
 * Policy (which values) lives in the LPM code; these are the register writers. */
void xhci_roothub_set_usb3_port_timeout(struct xhci_root_hub *rh, u32 port, BOOL u2, u16 timeout);
void xhci_roothub_set_usb2_hw_lpm(struct xhci_root_hub *rh, u32 port, u8 hird, u8 slot_id,
                                  BOOL besl_mode, u8 besld, u16 l1_timeout_us);
void xhci_roothub_clear_usb2_hw_lpm(struct xhci_root_hub *rh, u32 port);

/* Direct a 1-based root-hub port to U3 (deferred tail of the port-suspend
 * sequence, after the device's endpoint rings have been stopped). */
void xhci_roothub_set_port_u3(struct xhci_root_hub *rh, u8 port);

#endif