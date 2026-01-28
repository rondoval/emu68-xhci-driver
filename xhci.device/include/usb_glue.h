// SPDX-License-Identifier: GPL-2.0+
#pragma once

#include <exec/types.h>
#include <devices/usbhardware.h>
#include <xhci/usb.h>

struct XHCIUnit;

#define XHCI_REQ_RING_BUSY ((APTR)~0UL)

/* Transfer helpers */
int usb_glue_ctrl(struct XHCIUnit *unit, struct IOUsbHWReq *io);
int usb_glue_bulk(struct XHCIUnit *unit, struct IOUsbHWReq *io);
int usb_glue_int(struct XHCIUnit *unit, struct IOUsbHWReq *io);
int usb_glue_iso(struct XHCIUnit *unit, struct IOUsbHWReq *io);
int usb_glue_ctrl_after_address(struct usb_device *udev, struct IOUsbHWReq *io);

void usb_glue_free_udev_slot(struct usb_device *udev);

void io_reply_failed(struct IOUsbHWReq *io, int err);
void io_reply_data(struct usb_device *udev, struct IOUsbHWReq *io, int err, ULONG actual);

/* Internal helper: async CLEAR_FEATURE(ENDPOINT_HALT) for recovery paths */
void usb_glue_clear_feature_halt_internal(struct usb_device *udev, u32 ep_index);
void usb_glue_clear_tt_buffer_internal(struct usb_device *udev, u32 ep_index, int ep_type);

struct usb_device *usb_glue_alloc_udev(struct xhci_ctrl *ctrl, UWORD poseidon_address);
struct usb_device *get_or_init_udev(struct XHCIUnit *unit, UWORD poseidon_address);

void dispatch_request(struct IOUsbHWReq *req);