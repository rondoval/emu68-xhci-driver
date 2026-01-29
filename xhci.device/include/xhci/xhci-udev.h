// SPDX-License-Identifier: GPL-2.0+
#pragma once

#include <exec/types.h>
#include <devices/usbhardware.h>

/* Flags for use in DriverPrivate1 */
#define REQ_INTERNAL 0x1 /* Internal request, free instead of reply */
#define REQ_ENQUEUED 0x2 /* Request was already enqueued to EP */
#define REQ_ON_RING 0x4  /* Request is currently on the transfer ring */


struct XHCIUnit;
struct xhci_ctrl;

/* Access udev */
struct usb_device *xhci_udev_alloc(struct xhci_ctrl *ctrl, UWORD poseidon_address);
struct usb_device *xhci_udev_get(struct XHCIUnit *unit, UWORD poseidon_address);
void xhci_udev_free(struct usb_device *udev);

/* Dispatch */
int xhci_udev_send_ctrl(struct usb_device *udev, struct IOUsbHWReq *io);
int xhci_udev_send(struct IOUsbHWReq *req);

/* Track replies */
void xhci_udev_io_reply_failed(struct IOUsbHWReq *io, int err);
void xhci_udev_io_reply_data(struct usb_device *udev, struct IOUsbHWReq *io, int err, ULONG actual);

/* Send commands to device */
void xhci_udev_clear_feature_halt(struct usb_device *udev, ULONG ep_index);
void xhci_udev_clear_tt_buffer(struct usb_device *udev, ULONG ep_index, int ep_type);
