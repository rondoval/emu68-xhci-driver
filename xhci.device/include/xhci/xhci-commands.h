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

struct xhci_ctrl;
struct usb_device;
struct ep_context;
union xhci_trb;

void xhci_dispatch_command_event(struct xhci_ctrl *ctrl, union xhci_trb *event);
void xhci_process_command_timeouts(struct xhci_ctrl *ctrl);

void xhci_reset_ep(struct usb_device *udev, u8 ep_index);
void xhci_stop_ring(struct usb_device *udev, u8 ep_index);
/* Set TR Deq for one ring (endpoint must be Stopped or in Error); stream_id
 * 0 = the default ring. */
void xhci_set_deq_pointer(struct usb_device *udev, u8 ep_index, u32 deq_ptr, u16 stream_id);
/* Reset every transfer ring of the endpoint to its software enqueue position
 * — the ring-flush half of every recovery path. */
void xhci_flush_ep_rings(struct usb_device *udev, struct ep_context *ep_ctx);
void xhci_configure_endpoints(struct usb_device *udev, BOOL ctx_change, struct xhci_xfer *req);
void xhci_address_device(struct usb_device *udev, struct xhci_xfer *req);
void xhci_reset_device(struct usb_device *udev);
/* Set TR Deq only the stream rings marked in map (bit id set) — the surgical
 * recovery flush; handle_set_deq restarts the endpoint after the last one. */
void xhci_flush_ep_streams_marked(struct usb_device *udev, struct ep_context *ep_ctx, const u32 *map);
void xhci_disable_slot(struct usb_device *udev);

#endif /* XHCI_COMMANDS_H */
