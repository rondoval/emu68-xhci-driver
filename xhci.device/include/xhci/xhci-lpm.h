// SPDX-License-Identifier: GPL-2.0-only
/*
 * USB HOST XHCI Controller stack — Link Power Management (LPM)
 *
 * USB2 hardware LPM (L1) and USB3 U1/U2 device-initiated power management:
 * SEL/PEL/MEL parameter computation, the multi-step enable sequence, and the
 * LPM device-command senders (SET_SEL, SET_FEATURE(U1/U2/LTM_ENABLE), port
 * U1/U2 timeout programming).  Mirrors the relevant parts of Linux usbcore
 * (usb_set_lpm_parameters, usb_enable_lpm) and the xHCI driver.
 */
#ifndef __XHCI_LPM_H__
#define __XHCI_LPM_H__

#include <types.h>

struct usb_device;

/* Device Max Exit Latency in us for the slot context (0 = none).  Called by
 * xhci-context.c (xhci_compute_and_apply_mel) as well as internally. */
u32 xhci_calculate_mel(struct usb_device *udev);

/* Parse a fully-fetched BOS descriptor (buf/len) into the device's LPM fields
 * (lpm_capable, BESL, U1/U2 exit latencies, LTM) and compute the SEL/PEL/MEL
 * parameters.  No-op on a malformed/non-BOS buffer. */
void xhci_lpm_parse_bos_caps(struct usb_device *udev, const u8 *buf, u32 len);

/* LPM enable state machine (mirror usb_enable_lpm).  Entry point runs after the
 * SET_CONFIGURATION control transfer completes on the wire; it continues in
 * xhci_lpm_enable_stage2() once the MEL Evaluate Context completes, and
 * device-initiated enable chains from the SET_SEL completion. */
void xhci_lpm_enable(struct usb_device *udev);
/* Resumed from xhci_udev_op_advance(MEL_EVAL_DONE); TRUE if SET_SEL submitted. */
BOOL xhci_lpm_enable_stage2(struct usb_device *udev);
/* Resumed from xhci_udev_op_advance(SET_SEL_DONE). */
void xhci_lpm_devinit_enable(struct usb_device *udev);
/* Disconnect teardown (clears USB2 hardware LPM). */
void xhci_lpm_disable(struct usb_device *udev);

#endif /* __XHCI_LPM_H__ */
