/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Context HCD ABI (NSCMD_USB_*) ingress layer — the xHCI-native interface
 * designed in poseidon-backport/docs/poseidon-context-hcd-abi.md.
 *
 * Lifecycle ops arrive as struct IOStdReq (io_Data -> Uhcd* op block, see
 * devices/usbhcd_context.h); transfers travel the direct call path
 * (xhci-direct.c) and never pass through here. Internally the driver keeps
 * processing struct xhci_xfer: every op gets a driver-owned SHADOW xfer at
 * ingress, the existing machinery runs on the shadow, and its completion
 * callback (ctx_shadow_complete) copies the results back and replies the
 * client. The shadow also protects client memory — a lifecycle IOStdReq is
 * 48 bytes, so writing xfer-sized fields onto the client would corrupt it.
 */

#ifndef __XHCI_CTX_OPS_H__
#define __XHCI_CTX_OPS_H__

#include <xhci/xhci-xfer.h>

struct XHCIUnit;
struct usb_device;
struct xhci_ctrl;

/* The driver-internal stand-in for a context-ABI client request. The embedded
 * xfer MUST stay first: the machinery sees &sh->io and the completion callback
 * recovers the shadow by downcasting the xfer pointer. */
struct xhci_ctx_shadow
{
    struct xhci_xfer io;        /* what the driver core operates on (carries ctrl) */
    struct IORequest *client;   /* the IOStdReq that arrived on the wire */
};

/* Unit-task dispatcher entry for lifecycle ops (NSCMD_USBHCD_BASE..+0x0f);
 * io is the client's struct IOStdReq. Returns COMMAND_PROCESSED or
 * COMMAND_SCHEDULED (see device.h). */
u32 xhci_ctxops_process(struct IOStdReq *io);

/* Completion hook: finish a REQ_CTX_OP request (fills op OUT fields, restores
 * lifecycle state for state-neutral ops) and route it through the funnel.
 * Called from the command completion handlers in xhci-commands.c. */
void xhci_ctxops_complete(struct usb_device *udev, struct xhci_xfer *io, s8 err);

#endif /* __XHCI_CTX_OPS_H__ */
