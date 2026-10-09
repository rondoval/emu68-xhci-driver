// SPDX-License-Identifier: GPL-2.0-only
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME SysBase /* a local in every function, from its context's sysBase */
#include <proto/exec.h>
#endif

#include <exec/errors.h>

#include <xhci/xhci.h>
#include <xhci/ch9.h>
#include <xhci/usb_defs.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-descriptors.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci-submit.h>
#include <xhci/xhci-context.h>
#include <xhci/xhci-ctx-ops.h>
#include <xhci/xhci-lpm.h>

#include <device.h>
#include <debug.h>
#include <bits.h>
#include <byteorder.h>
#include <memory.h>
#include <minlist.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-udev] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef TRACE
#undef KprintfT
#define KprintfT(fmt, ...) PrintPistorm("[xhci-udev] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

/* TU-local helper used before its definition */
static struct usb_device *xhci_udev_find_child_on_port(struct usb_device *hub, u32 port);

/* Shared allocation core for both device kinds. */
static struct usb_device *xhci_udev_alloc_common(struct xhci_ctrl *ctrl)
{
    struct ExecBase *SysBase = ctrl->sysBase;
    struct usb_device *udev = pool_zalloc(ctrl->metaPool, sizeof(*udev));
    if (!udev)
    {
        Kprintf("Failed to allocate usb_device\n");
        goto nothing;
    }

    udev->controller = ctrl;
    udev->sysBase = ctrl->sysBase;

    /* Allocate the (output) device context that will be used in the HC. */
    udev->out_ctx = xhci_alloc_container_ctx(ctrl, XHCI_CTX_TYPE_DEVICE);
    if (!udev->out_ctx)
    {
        Kprintf("Failed to allocate out context\n");
        goto free_udev;
    }
    KprintfT("out_ctx bytes=%lx size=%lu\n",
             (ULONG)udev->out_ctx->bytes, (ULONG)udev->out_ctx->size);

    /* Allocate the (input) device context for address device command */
    udev->in_ctx = xhci_alloc_container_ctx(ctrl, XHCI_CTX_TYPE_INPUT);
    if (!udev->in_ctx)
    {
        Kprintf("Failed to allocate in context\n");
        goto destroy_out_ctx;
    }
    KprintfT("in_ctx bytes=%lx size=%lu\n",
             (ULONG)udev->in_ctx->bytes, (ULONG)udev->in_ctx->size);

    udev->toggle_in_ctx = xhci_alloc_container_ctx(ctrl, XHCI_CTX_TYPE_INPUT);
    if (!udev->toggle_in_ctx)
    {
        Kprintf("Failed to allocate toggle-reset in context\n");
        goto destroy_in_ctx;
    }

    return udev;

destroy_in_ctx:
    xhci_free_container_ctx(ctrl, udev->in_ctx);
destroy_out_ctx:
    xhci_free_container_ctx(ctrl, udev->out_ctx);
free_udev:
    pool_free(ctrl->metaPool, udev);
nothing:
    return NULL;
}

/* Context-ABI device: identity is the handle (slot id).  Parent, port and
 * speed are set explicitly by the create op. */
struct usb_device *xhci_udev_alloc_ctx(struct xhci_ctrl *ctrl)
{
    if (!ctrl)
        return NULL;

    struct usb_device *udev = xhci_udev_alloc_common(ctrl);
    if (!udev)
        return NULL;

    udev->ctx_mode = TRUE;
    udev->token_gen = ++ctrl->token_gen_counter;
    return udev;
}

/* The root-hub emulation's device anchor: owned by the root-hub object
 * (xhci_roothub_udev).  The context path addresses the emulation by reserved
 * handle only; the udev exists as the internal anchor of the views and the
 * synthetic-request machinery. */
struct usb_device *xhci_udev_alloc_root(struct xhci_ctrl *ctrl)
{
    if (!ctrl)
        return NULL;

    return xhci_udev_alloc_common(ctrl);
}

static void xhci_udev_flush(struct usb_device *udev, s8 reply_code)
{
    if (!udev)
        return;

    struct xhci_ctrl *ctrl = udev->controller;
    if (!ctrl)
        return;
    KprintfT("flushing device slot=%lu\n", (ULONG)udev->slot_id);

    xhci_ep_destroy_contexts(udev, reply_code);
}

void xhci_udev_free(struct usb_device *udev)
{
    if (!udev)
        return;
    struct ExecBase *SysBase = udev->sysBase;

    struct xhci_ctrl *ctrl = udev->controller;
    if (!ctrl)
        return;

    xhci_udev_flush(udev, UHIOERR_TIMEOUT);

    if (udev->toggle_in_ctx)
        xhci_free_container_ctx(udev->controller, udev->toggle_in_ctx);
    if (udev->in_ctx)
        xhci_free_container_ctx(udev->controller, udev->in_ctx);
    if (udev->out_ctx)
        xhci_free_container_ctx(udev->controller, udev->out_ctx);

    ctrl->dcbaa->dev_context_ptrs[udev->slot_id] = 0;
    ctrl->devices_by_slot_id[udev->slot_id] = NULL;
    pool_free(ctrl->metaPool, udev);
}

/* THE completion funnel: every non-hardware-retire completion routes through
 * here.  Sets the result, releases a bounce still mapped on a request that
 * died while queued, replies context-ABI lifecycle ops through their op
 * epilogue (xhci_ctxops_complete), and retires everything else through the
 * request's completion callback.  udev may be NULL (late/timeout paths) —
 * only the ctx-op epilogues consume it. */
void xhci_xfer_complete(struct usb_device *udev, struct xhci_xfer *io, s8 err, u32 actual)
{
    if (!io)
        return;

    io->actual = actual;
    io->error = err;

    /* a request that dies while queued still owns its mapping */
    if (io->priv_flags & REQ_DMA_MAPPED)
        xhci_dma_unmap(io->ctrl, io, FALSE);

    if (io->priv_flags & REQ_CTX_OP)
    {
        xhci_ctxops_complete(udev, io, err);
        return;
    }

    KprintfT("EP %lu err=%ld actual=%lu\n", (ULONG)io->endpoint, (LONG)err, (ULONG)actual);
    io->complete(io); /* internal-complete / direct-done / ctx-shadow-complete */
}

/* Reset the suspend-sequencing slot to idle. */
static void xhci_udev_suspend_clear(struct usb_device *udev)
{
    udev->suspend.stops_pending = 0;
    udev->suspend.u3_port = 0;
    udev->suspend.stash = NULL;
}

/**
 * The internal (fire-and-forget EP0) xfer completion callback, installed as
 * io.complete by the internal request builders (CLEAR_TT_BUFFER).  Handles
 * both success and failure: it tells the endpoint the request was about, if
 * one waits for it, and frees the transient payload and the xfer.
 */
static void xhci_udev_internal_complete(struct xhci_xfer *io)
{
    struct xhci_ctrl *ctrl = io->ctrl;
    const BOOL ok = (io->error == UHIOERR_NO_ERROR);

    if (!ok)
    {
        /* "device gone" (UHIOERR_TIMEOUT) and "cancelled" (IOERR_ABORTED) are the
         * expected outcome when a hub or device is torn down mid-recovery —
         * keep those quiet; a genuine device-level reject stays loud. */
        if (io->error == UHIOERR_TIMEOUT || io->error == IOERR_ABORTED)
            KprintfT("internal EP0 request retired (device gone): bmReqType=%02lx bReq=%02lx wValue=%lu err=%ld\n",
                     (ULONG)io->setup.usd_RequestType,
                     (ULONG)io->setup.usd_Request, (ULONG)le16(io->setup.usd_Value), (LONG)io->error);
        else
            Kprintf("internal EP0 request failed: bmReqType=%02lx bReq=%02lx wValue=%lu err=%ld\n",
                    (ULONG)io->setup.usd_RequestType,
                    (ULONG)io->setup.usd_Request, (ULONG)le16(io->setup.usd_Value), (LONG)io->error);
    }

    /* By slot and index, not by pointer: the device may be gone by now.  (A
     * slot handed out anew within the request's timeout could see another
     * endpoint's TT hold lifted early - which only costs that one the wait.) */
    if (ctrl && io->about_slot)
    {
        struct usb_device *about = ctrl->devices_by_slot_id[io->about_slot];
        struct ep_context *ep_ctx = about ? xhci_ep_get_context_for_index(about, io->about_ep) : NULL;
        if (ep_ctx)
            xhci_ep_tt_cleared(ep_ctx);
    }

    if (ctrl && io->data)
        dma_free(ctrl->dmaPool, io->data);
    if (ctrl)
        slab_free(&ctrl->xfer_slab, io);
}

/* Build and issue an internal (driver-originated) request.
 *
 * An internal request is always EP0 control traffic on udev: the endpoint it is
 * *about* travels in the setup packet, never in the ring it rides.  A control
 * TD is only legal on a control ring, so it goes to DCI 0's context - the
 * endpoint whose recovery prompted the request owns no part of the routing.
 *
 * about_slot / about_ep name an endpoint that waits for the request (slot 0 =
 * none): it hears of the completion through xhci_ep_tt_cleared().  TRUE = the
 * request went out, or failed at once - either way its completion has been or
 * will be delivered; FALSE = there never was a request. */
static BOOL xhci_udev_send_control_request(struct usb_device *udev,
                                    u8 bmRequestType, u8 bRequest,
                                    u16 wValue, u16 wIndex, u16 wLength,
                                    u8 about_slot, u8 about_ep)
{
    if (!udev || !udev->controller)
        return FALSE;

    struct xhci_ctrl *ctrl = udev->controller;
    struct xhci_xfer *io = slab_zalloc(&ctrl->xfer_slab);
    if (!io)
        return FALSE;

    io->ctrl = ctrl;
    io->sysBase = ctrl->sysBase;
    io->complete = xhci_udev_internal_complete; /* fire-and-forget: free on completion */
    io->owner_slot = udev->slot_id;
    io->type = UHCD_EPTYPE_CONTROL;
    io->priv_flags = REQ_ENQUEUED;

    io->setup.usd_RequestType = bmRequestType;
    io->setup.usd_Request = bRequest;
    io->setup.usd_Value = le16(wValue);
    io->setup.usd_Index = le16(wIndex);
    io->setup.usd_Length = le16(wLength);

    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, 0);
    if (!ep_ctx)
    {
        Kprintf("No EP0 context on slot %lu\n", (ULONG)udev->slot_id);
        slab_free(&ctrl->xfer_slab, io);
        return FALSE;
    }
    io->timeout_ms = 1000;
    io->flags |= XHCI_XF_TIMEOUT;
    io->about_slot = about_slot;
    io->about_ep = about_ep;

    s8 err = xhci_ep_submit(ep_ctx, io);
    if (err != UHIOERR_NO_ERROR)
    {
        /* fire-and-forget: retire through the internal completer, which
         * frees the payload and the xfer */
        io->error = err;
        xhci_xfer_reply(io);
    }
    return TRUE;
}

inline static u8 xhci_ep_index_to_address(u8 ep_index)
{
    if (ep_index == 0)
        return 0;
    return (u8)(EP_INDEX_TO_ENDPOINT(ep_index) | ((ep_index & 0x1U) ? USB_DIR_OUT : USB_DIR_IN));
}

/* ---- Port suspend (U3) sequencing -----------------------------------------
 * xHCI 4.15.1: all of a device's endpoints shall be stopped before its port is
 * directed to U3.  Stop Endpoint completes asynchronously, so the suspend is
 * sequenced through struct udev_suspend: stop every endpoint ring (queued TDs
 * stay on the rings), count the completions, and only then act - a root port
 * by writing PLS=U3, a NSCMD_USB_SET_SUSPEND op by replying it (the port
 * transition is the stack's job on external hubs and root-hub views alike).
 * Mirrors Linux xhci_stop_device()/xhci_ring_device().  Suspending a hub does
 * not recurse into its children: the stack suspends leaf devices first (as
 * usbcore does).
 *
 * Aborts while suspended keep the completion contract: the endpoint stays
 * parked and the targeted TDs are retired by the regular recovery against the
 * saved dequeue pointers - at once when the rings are stopped already, else
 * when the suspend's Stop Endpoint has completed.  No doorbell rings until
 * xhci_ep_resume().
 */

/* Abort an in-flight suspend sequence, replying the stashed request so the
 * stack isn't left waiting; late Stop Endpoint completions land on a cleared
 * counter and are ignored. */
void xhci_udev_suspend_cancel(struct usb_device *udev, s8 err)
{
    if (udev->suspend.stops_pending == 0 && !udev->suspend.stash)
        return;

    KprintfT("slot %lu: cancelling suspend (%lu stops pending)\n",
             (ULONG)udev->slot_id, (ULONG)udev->suspend.stops_pending);

    struct xhci_xfer *stash = udev->suspend.stash;
    xhci_udev_suspend_clear(udev);
    if (stash)
        xhci_xfer_complete(udev, stash, err, 0);
}

/* Tail of the suspend sequence, run when the last Stop Endpoint completes. */
static void xhci_udev_suspend_finish(struct usb_device *udev)
{
    udev->suspend.stopped_us = get_time(); /* the last stop is in */

    u8 port = udev->suspend.u3_port;
    struct xhci_xfer *req = udev->suspend.stash;
    xhci_udev_suspend_clear(udev);

    if (port != 0)
    {
        xhci_roothub_set_port_u3(udev->controller->root_hub, port);
        return;
    }

    if (!req)
        return;

    /* NSCMD_USB_SET_SUSPEND: rings quiesced — the hub class drives the port
     * to U3 after this reply. */
    req->error = UHIOERR_NO_ERROR;
    xhci_xfer_reply(req);
}

/* Stop all endpoint rings of udev ahead of a port suspend.  Returns TRUE if
 * Stop Endpoint commands are in flight and the port suspend is deferred to
 * xhci_udev_suspend_finish(); FALSE if there is nothing to wait for (caller
 * suspends the port synchronously). */
BOOL xhci_udev_suspend_device(struct usb_device *udev, u8 root_port, struct xhci_xfer *deferred_req)
{
    if (xhci_udev_suspend_pending(udev))
    {
        Kprintf("slot %lu: suspend already in flight\n", (ULONG)udev->slot_id);
        return FALSE;
    }

    /* Slot-state gate (xHCI 4.5.3): endpoint rings exist only once addressed. */
    if (udev->slot_state < USB_DEV_SLOT_STATE_ADDRESSED)
    {
        Kprintf("slot %lu: suspend needs an addressed slot (state %lu)\n",
                (ULONG)udev->slot_id, (ULONG)udev->slot_state);
        return FALSE;
    }

    u8 stops = 0;
    for (u8 i = 0; i < USB_MAX_ENDPOINT_CONTEXTS; ++i)
        if (udev->ep_context[i] && xhci_ep_request_suspend(udev->ep_context[i]))
            stops++;

    /* now if nothing had to be stopped; else xhci_udev_suspend_finish() says
     * when the last stop was in */
    udev->suspend.stopped_us = get_time();

    if (stops == 0)
        return FALSE; /* nothing to wait for; caller acts synchronously */

    udev->suspend.stops_pending = stops;
    udev->suspend.u3_port = root_port;
    udev->suspend.stash = deferred_req;
    KprintfT("suspending slot %lu: %lu endpoint stops pending\n",
             (ULONG)udev->slot_id, (ULONG)stops);
    return TRUE;
}

/* Root-hub entry: stop the rings of the device on 1-based root port.  TRUE if
 * the U3 write is deferred until the stops complete. */
BOOL xhci_udev_suspend_port(struct usb_device *hub_udev, u8 port)
{
    struct usb_device *child = xhci_udev_find_child_on_port(hub_udev, port);
    if (!child)
        return FALSE;
    return xhci_udev_suspend_device(child, port, NULL);
}

/* Restart the device's stopped endpoint rings after its port returned to U0
 * (xhci_ep_resume is a no-op on endpoints that were never suspended).
 *
 * An endpoint stopped for a suspend stays stopped for at least 10 ms (xHCI
 * 6.4.3.8, the Suspend flag of Stop Endpoint).  A resume the stack starts
 * comes later than that by itself; a device that wakes itself over a
 * SuperSpeed link right after being suspended may not.  What is left of the
 * 10 ms is waited out here, the one place every restart passes.  (The stamp
 * of a suspend long past can look recent once in 71 minutes, when the
 * microsecond clock wraps: that costs one needless wait.) */
#define SUSPEND_STOPPED_MIN_US 10000U
void xhci_udev_resume_device(struct usb_device *udev)
{
    const u32 stopped_for_us = get_time() - udev->suspend.stopped_us;
    if (stopped_for_us < SUSPEND_STOPPED_MIN_US)
        xhci_sleep_unlocked(udev->controller, (SUSPEND_STOPPED_MIN_US - stopped_for_us + 999U) / 1000U);

    for (u8 i = 0; i < USB_MAX_ENDPOINT_CONTEXTS; ++i)
        if (udev->ep_context[i])
            xhci_ep_resume(udev->ep_context[i]);
}

/* Port resumed (root hub U0 write, or ClearPortFeature(SUSPEND) completion on
 * an external hub): restart the attached device's stopped endpoint rings. */
void xhci_udev_resume_port(struct usb_device *hub_udev, u8 port)
{
    struct usb_device *child = xhci_udev_find_child_on_port(hub_udev, port);
    if (!child)
        return;

    xhci_udev_resume_device(child);
}

/* One endpoint the suspend sequence waited for is parked (or failed): counts
 * toward the drain; a call with no sequence in flight (cancelled suspend,
 * device gone) is ignored. */
void xhci_udev_suspend_stop_done(struct usb_device *udev)
{
    if (udev->suspend.stops_pending != 0 && --udev->suspend.stops_pending == 0)
        xhci_udev_suspend_finish(udev);
}

/* After a halt, clear what the failed transfer left in the buffer of the hub's
 * transaction translator (xHCI 4.6.8; the request: USB 2.0 11.24.2.3, defined
 * for control and bulk endpoints only).  It goes to the hub that holds the
 * translator, which need not be the device's parent, and names the
 * translator by port - port 1 on a hub that has a single one.
 *
 * TRUE = the endpoint hears through xhci_ep_tt_cleared() when the hub is
 * done, or has heard already; FALSE = there is no translator, or nothing went
 * out. */
BOOL xhci_udev_clear_tt_buffer(struct usb_device *udev, u8 ep_index, s32 ep_type)
{
    if (ep_type != USB_ENDPOINT_XFER_CONTROL && ep_type != USB_ENDPOINT_XFER_BULK)
        return FALSE;

    u8 tt_port;
    struct usb_device *hub = xhci_tt_hub(udev, &tt_port);
    if (!hub)
        return FALSE;
    if (!xhci_hub_multi_tt_enabled(hub))
        tt_port = 1;

    const u16 epnum = (u16)EP_INDEX_TO_ENDPOINT(ep_index);
    const BOOL out = (ep_index & 0x1) != 0;

    u16 devinfo = epnum;
    devinfo |= (u16)((u16)udev->xhci_address << 4);
    devinfo |= (u16)((u16)ep_type << 11);
    if (!out)
        devinfo |= 1U << 15;

    KprintfT("CLEAR_TT_BUFFER to hub slot %lu TT port %lu (dev slot %lu ep %lu type %ld)\n",
             (ULONG)hub->slot_id, (ULONG)tt_port,
             (ULONG)udev->slot_id, (ULONG)epnum, (LONG)ep_type);

    /* A control endpoint has a buffer for either direction: one request each,
     * in order on the hub's EP0, and the endpoint waits for the last. */
    if (ep_type == USB_ENDPOINT_XFER_CONTROL)
        xhci_udev_send_control_request(hub,
                                       USB_DIR_OUT | USB_RT_PORT,
                                       HUB_CLEAR_TT_BUFFER,
                                       devinfo ^ (1 << 15), /* the other direction */
                                       tt_port,
                                       0 /* wLength */,
                                       0, 0);

    return xhci_udev_send_control_request(hub,
                                          USB_DIR_OUT | USB_RT_PORT,
                                          HUB_CLEAR_TT_BUFFER,
                                          devinfo,
                                          tt_port,
                                          0 /* wLength */,
                                          udev->slot_id, ep_index);
}

s32 xhci_ep_type_for_index(struct usb_device *udev, u8 ep_index)
{
    if (!udev)
        return -1;

    if (ep_index == 0)
        return USB_ENDPOINT_XFER_CONTROL;

    /* The type is cached at context wiring (xhci_ep_set_hw_type) — no
     * device-context invalidate on the submit path.  The xHCI EP Type field
     * encodes (usb type | dir << 2). */
    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
    if (!ep_ctx)
        return -1;

    u8 hw_type = xhci_ep_get_hw_type(ep_ctx);
    if (hw_type == 0)
        return -1;

    return (s32)(hw_type & USB_ENDPOINT_XFERTYPE_MASK);
}

void xhci_udev_disconnect(struct usb_device *udev, BOOL recursive)
{
    if (!udev || !udev->slot_id)
        return;

    /* Disconnect downstream devices first so hubs drain their children before
     * vanishing.  Walk the slot map — it covers every slotted device. */
    if (recursive)
    {
        struct xhci_ctrl *ctrl = udev->controller;

        for (u32 i = 1; i < MAX_HC_SLOTS; ++i)
        {
            struct usb_device *child = ctrl->devices_by_slot_id[i];
            if (!child || child == udev)
                continue;

            if (child->parent == udev)
                xhci_udev_disconnect(child, TRUE);
        }
    }

    KprintfT("disconnect device slot=%lu port=%lu\n",
             (ULONG)udev->slot_id,
             (ULONG)udev->parent_port);

    /* A suspend sequence in flight dies with the device: reply its stashed
     * request (an NSCMD_USB_SET_SUSPEND mid-quiesce) so the stack isn't left
     * waiting; late Stop Endpoint completions are ignored. */
    xhci_udev_suspend_cancel(udev, UHIOERR_TIMEOUT);

    /* Clear USB2 hardware LPM on the root port before the slot goes away so
     * PORTPMSC.L1DS no longer references this (about to be freed) slot. */
    xhci_lpm_disable(udev);

    xhci_disable_slot(udev);
}

static struct usb_device *xhci_udev_find_child_on_port(struct usb_device *hub, u32 port)
{
    if (!hub)
        return NULL;

    struct xhci_ctrl *ctrl = hub->controller;
    /* Root-port devices carry parent == NULL plus the controller-global
     * port from CREATE_DEVICE; only external-hub children link to a parent
     * udev. */
    const BOOL root = (hub == xhci_roothub_udev(ctrl->root_hub));
    for (u32 i = 1; i < MAX_HC_SLOTS; ++i)
    {
        struct usb_device *cand = ctrl->devices_by_slot_id[i];
        if (!cand || cand == hub)
            continue;

        if ((cand->parent == hub || (root && cand->ctx_mode && !cand->parent)) &&
            cand->parent_port == port)
            return cand;
    }

    return NULL;
}


