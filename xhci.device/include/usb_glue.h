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

/* RT ISO handling */
int usb_glue_add_iso_handler(struct XHCIUnit *unit, struct IOUsbHWReq *io);
int usb_glue_rem_iso_handler(struct XHCIUnit *unit, struct IOUsbHWReq *io);
int usb_glue_start_rt_iso(struct XHCIUnit *unit, struct IOUsbHWReq *io);
int usb_glue_stop_rt_iso(struct XHCIUnit *unit, struct IOUsbHWReq *io);
void usb_glue_notify_rt_iso_stopped(struct usb_device *udev, int endpoint);

/* Root hub bus ops */
int usb_glue_bus_reset(struct XHCIUnit *unit);
int usb_glue_bus_suspend(struct XHCIUnit *unit);
int usb_glue_bus_resume(struct XHCIUnit *unit);
int usb_glue_bus_oper(struct XHCIUnit *unit);

void usb_glue_free_udev_slot(struct xhci_ctrl *ctrl, UWORD addr);
void usb_glue_disconnect_device(struct xhci_ctrl *ctrl, struct usb_device *udev, BOOL recursive);
void usb_glue_flush_queues(struct XHCIUnit *unit);

void io_reply_failed(struct IOUsbHWReq *io, int err);
void io_reply_data(struct usb_device *udev, struct IOUsbHWReq *io, int err, ULONG actual);

/* Internal helper: async CLEAR_FEATURE(ENDPOINT_HALT) for recovery paths */
void usb_glue_clear_feature_halt_internal(struct usb_device *udev, u32 ep_index);
void usb_glue_clear_tt_buffer_internal(struct usb_device *udev, u32 ep_index, int ep_type);

void xhci_ep_schedule_next(struct usb_device *udev, int endpoint);
void xhci_ep_schedule_rt_iso(struct usb_device *udev, int endpoint);
