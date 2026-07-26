// SPDX-License-Identifier: GPL-2.0-only
/*
 * USB HOST XHCI Controller stack — Link Power Management (LPM)
 *
 * Controller-side LPM policy: SEL/PEL/MEL/timeout computation, USB2 hardware
 * LPM (L1) programming, and MEL-error recovery.  The device/hub control
 * transfers (SET_SEL, SET_FEATURE(U1/U2/LTM_ENABLE), port SetPortFeature) are
 * issued by the stack from the wire parameters NSCMD_USB_SET_LINK_POWER
 * returns.  Mirrors the relevant parts of Linux usbcore
 * (usb_set_lpm_parameters, usb_enable_lpm) and the xHCI driver.
 */
#ifndef __XHCI_LPM_H__
#define __XHCI_LPM_H__

#include <types.h>

struct usb_device;
struct UhcdSetLinkPower;

/* NSCMD_USB_SET_LINK_POWER: adopt the stack-parsed BOS facts + policy, program
 * the controller-side LPM state, and write the computed wire parameters back
 * into *op for the stack to issue (SET_SEL, SET_FEATURE(U1/U2/LTM_ENABLE), port
 * SetPortFeature).  Returns TRUE if a MEL Evaluate Context must be issued (the
 * caller replies the op from its completion); FALSE to reply synchronously. */
BOOL xhci_lpm_set_link_power(struct usb_device *udev, struct UhcdSetLinkPower *op);

/* COMP_MEL_ERR recovery: shrink the device MEL by the controller-reported ELD
 * and update the input context; FALSE = retry budget exhausted. */
BOOL xhci_lpm_handle_mel_err(struct usb_device *udev, u32 eld);

/* Disconnect teardown (clears USB2 hardware LPM). */
void xhci_lpm_disable(struct usb_device *udev);

#endif /* __XHCI_LPM_H__ */
