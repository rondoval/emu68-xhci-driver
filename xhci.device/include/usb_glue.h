// SPDX-License-Identifier: GPL-2.0+
#pragma once

#include <exec/types.h>
#include <devices/usbhardware.h>

struct XHCIUnit;

/* Initialize/teardown glue per unit */
int usb_glue_init(struct XHCIUnit *unit);
void usb_glue_shutdown(struct XHCIUnit *unit);

/* Synchronous transfer helpers (initial minimal API) */
int usb_glue_ctrl(struct XHCIUnit *unit, struct IOUsbHWReq *io);
int usb_glue_bulk(struct XHCIUnit *unit, struct IOUsbHWReq *io);
int usb_glue_int(struct XHCIUnit *unit, struct IOUsbHWReq *io);

/* Root hub bus ops */
int usb_glue_bus_reset(struct XHCIUnit *unit);
int usb_glue_bus_suspend(struct XHCIUnit *unit);
int usb_glue_bus_resume(struct XHCIUnit *unit);
int usb_glue_bus_oper(struct XHCIUnit *unit);
