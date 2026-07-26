/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __XHCI_DIRECT_H__
#define __XHCI_DIRECT_H__

#include <xhci/xhci-xfer.h>

struct xhci_ctrl;
struct usb_device;
struct IOStdReq;
struct IORequest;
struct XHCIUnit;

/*
 * The direct transfer path (usbhcd_context.h "The transfer path").
 *
 * NSCMD_USB_ATTACH hands the stack this module's submit/abort entries plus
 * the opaque HCD context (the unit) they anchor on, and registers the
 * stack's done hook; every control/bulk/interrupt/iso transfer is then a
 * direct call in the caller's task — no IORequest, no message-port round
 * trip.  Submissions ride the ordinary transfer machinery (rings, TDs,
 * timeouts, recovery, bounce buffers, stream selection) as bare struct
 * xhci_xfer work items whose completion callback (xhci_direct_done) calls
 * the done hook.  (The device image is ROM-able — no writable data — so the
 * HCD context argument is the entries' only anchor.)
 *
 * Tokens are packed 32-bit values, not pointers — nothing to dereference, so
 * a stale token (endpoint dropped, device destroyed, slot reused) can never
 * crash: it fails validation under the lock and the submit rejects with
 * UHIOERR_TIMEOUT (device-gone semantics).
 *
 *   bit 31    validity mark (a token is never 0/NULL)
 *   bit 30    root-hub token
 *   device:   bits29:13 generation | bits12:5 slot id | bits4:0 ep_index
 *   root hub: bits6:5 view (RH_VIEW_*) | bits4:0 ep_index (0 = EP0)
 *
 * The generation is stamped per CREATE_DEVICE (usb_device.token_gen), so a
 * token of a destroyed device never matches the slot's next tenant.
 *
 * Root-hub submits are handed to the unit task (CMD_INTERNAL_RH_SUBMIT) so
 * client tasks never carry the port handlers' ms-scale register work; the
 * traffic is cold.
 *
 * Concurrency: every entry serializes against the unit task through
 * ctrl->xfer_lock; the done hook runs with the lock held and may re-enter
 * submit() (exec semaphores nest within a task).
 */

/* CMD_INTERNAL_RH_SUBMIT message: a root-hub transfer deferred to the unit
 * task.  Driver-owned, never replied as a message; the embedded xfer
 * completes through the done hook (its callback frees the whole message). */
struct xhci_rh_submit_msg
{
    struct IORequest rs_Req;    /* rides the unit port */
    u8 rs_ViewId;               /* RH_VIEW_* */
    u8 rs_Pad;
    struct xhci_xfer rs_Xfer;
};

/* NSCMD_USB_ATTACH (unit task; io_Data -> UhcdAttach) */
u32 xhci_direct_attach(struct IOStdReq *client);
/* Unit teardown: drop the hook so a late completion can never call into a
 * freed stack context. */
void xhci_direct_detach(struct XHCIUnit *unit);

/* Token minting for the lifecycle ops' OUT fields. */
APTR xhci_direct_device_token(const struct usb_device *udev, u8 ep_index);
APTR xhci_direct_roothub_token(u8 view_id, u8 ep_index);

/* The entries handed out by ATTACH (UhcdSubmitFunc / UhcdCtrlSubmitFunc /
 * UhcdAbortFunc shape; hcd = the XHCIUnit from ato_HcdContext; callable from
 * any task, never from interrupts). */
LONG xhci_direct_submit(APTR hcd, APTR ep_token, APTR data, ULONG length,
                        ULONG naktimeout_ms, UWORD stream_id,
                        UWORD flags, APTR cookie);
LONG xhci_direct_ctrl_submit(APTR hcd, APTR ep0_token,
                             const struct UhcdSetupData *setup,
                             APTR data, ULONG length,
                             ULONG naktimeout_ms, APTR cookie);
LONG xhci_direct_abort(APTR hcd, APTR ep_token, APTR cookie);

/* Completion callback of every direct device transfer (io.complete): reply
 * through the done hook (frees the xfer first; never ReplyMsg). */
void xhci_direct_done(struct xhci_xfer *io);

/* CMD_INTERNAL_RH_SUBMIT dispatcher entry (unit task). */
u32 xhci_direct_rh_submit(struct IORequest *ioreq);

#endif /* __XHCI_DIRECT_H__ */
