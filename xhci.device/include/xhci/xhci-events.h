#ifndef _XHCI_EVENTS_H_
#define _XHCI_EVENTS_H_

#include <exec/lists.h>
#include <exec/types.h>
#include <compat.h>
#include <exec/semaphores.h>
#include <devices/usbhardware.h>
#include <xhci/xhci-td.h>


struct xhci_ring; /* forward declaration */
struct usb_device; /* forward declaration */
struct xhci_ctrl;  /* forward declaration */

BOOL xhci_process_event_trb(struct xhci_ctrl *ctrl);
void xhci_process_event_timeouts(struct xhci_ctrl *ctrl);

#endif /* _XHCI_EVENTS_H_ */