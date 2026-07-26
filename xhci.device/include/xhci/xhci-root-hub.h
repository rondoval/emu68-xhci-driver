/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __XHCI_ROOT_HUB_H
#define __XHCI_ROOT_HUB_H

#include <xhci/xhci-xfer.h>

struct usb_device;
struct xhci_root_hub;
struct xhci_root_hub_view;

struct xhci_root_hub *xhci_roothub_create(struct usb_device *udev);
void xhci_roothub_destroy(struct xhci_root_hub *rh);

/* The root hub exposes two protocol-pure VIEWS of the same port register
 * file:
 *  - RH_VIEW_SS:    the USB3-protocol port subset as a protocol-pure
 *    SuperSpeed hub (USB3-spec port statuses, speed field 0 = 5 Gbps).
 *  - RH_VIEW_USB2:  the USB2-protocol port subset as a classic USB2 hub.
 * They serve UHCD_HANDLE_ROOTHUB / UHCD_HANDLE_ROOTHUB_USB2; ports are
 * view-local 1..N and translated to controller-global numbers at dispatch.
 * Each view has its own pending status-change interrupt request. */
#define RH_VIEW_SS      1
#define RH_VIEW_USB2    2

/* NULL when the view has no ports. */
struct xhci_root_hub_view *xhci_roothub_view(struct xhci_root_hub *rh, u8 format);
BOOL xhci_roothub_has_usb3_ports(struct xhci_root_hub *rh);
BOOL xhci_roothub_has_usb2_ports(struct xhci_root_hub *rh);
/* view-local (1-based) -> controller-global (1-based); 0 = out of range */
u8 xhci_roothub_view_global_port(struct xhci_root_hub_view *v, u8 local);

void xhci_roothub_view_submit_ctrl_request(struct xhci_root_hub_view *v, struct xhci_xfer *req);
s8 xhci_roothub_view_submit_int_request(struct xhci_root_hub_view *v, struct xhci_xfer *req);

/* complete/abort cover BOTH views (event processing, CMD_FLUSH, teardown). */
void xhci_roothub_complete_int_request(struct xhci_root_hub *rh);
void xhci_roothub_abort_int_request(struct xhci_root_hub *rh);
/* Abort the view's pending interrupt request if it carries this cookie
 * (xhci_direct_abort; a wish). */
void xhci_roothub_view_abort_int_cookie(struct xhci_root_hub_view *v, APTR cookie);

/* The emulation's device anchor (owned by the root-hub object). */
struct usb_device *xhci_roothub_udev(struct xhci_root_hub *rh);
void xhci_roothub_port_lpm_caps(struct xhci_root_hub *rh, u32 port, BOOL *hw_lpm, BOOL *besl_lpm);

/* USB2 hardware LPM (L1) register writers for a 1-based root-hub port; the
 * policy (which values) lives in xhci-lpm.c.  U1/U2 port timeouts are written
 * by the view's own SetPortFeature handlers. */
void xhci_roothub_set_usb2_hw_lpm(struct xhci_root_hub *rh, u32 port, u8 hird, u8 slot_id,
                                  BOOL besl_mode, u8 besld, u16 l1_timeout_us);
void xhci_roothub_clear_usb2_hw_lpm(struct xhci_root_hub *rh, u32 port);

/* Direct a 1-based root-hub port to U3 (deferred tail of the port-suspend
 * sequence, after the device's endpoint rings have been stopped). */
void xhci_roothub_set_port_u3(struct xhci_root_hub *rh, u8 port);

#endif