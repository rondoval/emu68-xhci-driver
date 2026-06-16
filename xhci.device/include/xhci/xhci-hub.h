// SPDX-License-Identifier: GPL-2.0-only
/*
 * USB HOST XHCI Controller stack — SuperSpeed hub emulation
 *
 * Presents a USB-2.0 hub view of a SuperSpeed hub to the upstream USB stack:
 * translates hub-descriptor and port-feature requests to/from SS format, maps
 * SS port status into USB-2.0 status/change bits, and strips SS endpoint
 * companion descriptors from configuration descriptors.
 */
#ifndef __XHCI_HUB_H__
#define __XHCI_HUB_H__

#include <types.h>

struct usb_device;
struct USBIORequest;

/* Rewrite a USB-2.0 GetHubDescriptor into the SS form before it is sent on the
 * wire (the response is translated back in xhci_hub_handle_get_descriptor). */
void xhci_hub_translate_descriptor_request(struct usb_device *udev, struct USBIORequest *io);

/* Intercept SS-hub class port-feature requests that need SS<->USB2 translation
 * or local emulation.  Returns TRUE if the request was fully handled (replied),
 * FALSE if it should proceed (possibly after in-place rewriting). */
BOOL xhci_hub_filter_emulated_ctrl_request(struct usb_device *udev, struct USBIORequest *io);

/* Parse a hub-descriptor response: update TT think time / port count and, for an
 * SS descriptor under emulation, cache it and synthesize the USB-2.0 form. */
void xhci_hub_handle_get_descriptor(struct usb_device *udev, struct USBIORequest *io, u8 descriptorType);

/* Parse a hub GetPortStatus response: map SS status to USB-2.0, tear down a lost
 * child, and arm a pending attach for the next SET_ADDRESS. */
void xhci_hub_handle_get_port_status(struct usb_device *udev, struct USBIORequest *io);

/* Strip SS endpoint companion descriptors from a configuration descriptor so the
 * USB-2.0-only upstream stack parses the remaining descriptors cleanly. */
void xhci_hub_filter_ss_ep_companion_desc(struct USBIORequest *io);

#endif /* __XHCI_HUB_H__ */
