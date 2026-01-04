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

/* Root hub bus ops */
int usb_glue_bus_reset(struct XHCIUnit *unit);
int usb_glue_bus_suspend(struct XHCIUnit *unit);
int usb_glue_bus_resume(struct XHCIUnit *unit);
int usb_glue_bus_oper(struct XHCIUnit *unit);

void usb_glue_flush_queues(struct XHCIUnit *unit);

void io_reply_failed(struct IOUsbHWReq *io, int err);
void io_reply_data(struct usb_device *udev, struct IOUsbHWReq *io, int err, ULONG actual);

void xhci_ep_schedule_next(struct usb_device *udev, int endpoint);
void xhci_ep_schedule_rt_iso(struct usb_device *udev, int endpoint);
