/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Command-ring issuers and the command completion dispatcher.  Every function
 * here requires ctrl->xfer_lock held (the unit-task work blocks and the
 * direct entries are the only lock sites).
 */

#ifndef XHCI_COMMANDS_H
#define XHCI_COMMANDS_H

#include <types.h>
#include <xhci/xhci-xfer.h>
#include <xhci/xhci-ring.h>

struct xhci_ctrl;
struct usb_device;
struct ep_context;
struct xhci_container_ctx;
union xhci_trb;

void xhci_dispatch_command_event(struct xhci_ctrl *ctrl, union xhci_trb *event);
void xhci_process_command_timeouts(struct xhci_ctrl *ctrl);

/* One command of an endpoint's recovery: Stop Endpoint, Reset Endpoint (ring
 * NULL, deq 0) or Set TR Dequeue for one of its transfer rings.  flags are
 * the command's own TRB bits: TRB_SP on the Stop Endpoint ahead of a suspend,
 * TRB_TSP on the Reset Endpoint of a soft retry, else 0.  The endpoint sequences these itself (xhci-endpoint.c); its
 * completion - or its timeout - comes back as xhci_ep_command_done().
 * FALSE = nothing went out. */
BOOL xhci_queue_ep_command(struct usb_device *udev, u8 ep_index, trb_type cmd, u32 flags, struct xhci_ring *ring, dma_addr_t deq);
/* Configure Endpoint / Evaluate Context carrying in_ctx.  TRUE = queued (req,
 * if any, is retired by the command); FALSE = nothing went out and req is
 * still the caller's. */
BOOL xhci_configure_endpoints(struct usb_device *udev, struct xhci_container_ctx *in_ctx, BOOL ctx_change, struct xhci_xfer *req);
/* TRUE while any command of this device is still on the command ring. */
BOOL xhci_device_command_pending(struct usb_device *udev);
void xhci_address_device(struct usb_device *udev, struct xhci_xfer *req);
/* xHCI 4.6.11 Reset Device chained into a BSR=0 re-address; req is the
 * NSCMD_USB_RESET_DEVICE op being served (replied from the chain). */
void xhci_reset_device(struct usb_device *udev, struct xhci_xfer *req);
void xhci_disable_slot(struct usb_device *udev);

#endif /* XHCI_COMMANDS_H */
