// SPDX-License-Identifier: GPL-2.0-only
/*
 * The direct transfer path (usbhcd_context.h "The transfer path") — see
 * xhci-direct.h for the token format and the concurrency rules.
 *
 * NSCMD_USB_ATTACH exchanges the stack's done hook for the submit/abort
 * entries below (anchored on the unit passed back as the hcd context); the
 * lifecycle ops mint the endpoint tokens (xhci_direct_device_token /
 * xhci_direct_roothub_token) that key every submit.  Device transfers
 * enqueue straight onto the rings from the caller's task; root-hub
 * transfers defer to the unit task via CMD_INTERNAL_RH_SUBMIT so client
 * tasks never carry the port handlers' ms-scale register work (the
 * concurrency invariant everywhere is ctrl->xfer_lock, and the traffic is
 * cold).
 */

#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#include <clib/utility_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME (*(struct ExecBase **)4UL)
#include <proto/exec.h>
#define UTILITY_BASE_NAME ctrl->utilityBase
#include <proto/utility.h>
#endif

#include <exec/errors.h>

#include <debug.h>
#include <device.h>
#include <memory.h>

#include <xhci/ch9.h>
#include <xhci/xhci.h>
#include <xhci/xhci-descriptors.h>
#include <xhci/xhci-direct.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-submit.h>
#include <xhci/xhci-udev.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-direct] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef TRACE
#undef KprintfT
#define KprintfT(fmt, ...) PrintPistorm("[xhci-direct] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

/* Token fields (format in xhci-direct.h) */
#define DTOK_VALID      0x80000000UL
#define DTOK_ROOTHUB    0x40000000UL
#define DTOK_GEN_SHIFT  13
#define DTOK_GEN_MASK   0x1FFFFUL
#define DTOK_SLOT_SHIFT 5
#define DTOK_SLOT_MASK  0xFFUL
#define DTOK_VIEW_SHIFT 5
#define DTOK_VIEW_MASK  0x3UL
#define DTOK_EP_MASK    0x1FUL

APTR xhci_direct_device_token(const struct usb_device *udev, u8 ep_index)
{
    return (APTR)(DTOK_VALID |
                  ((udev->token_gen & DTOK_GEN_MASK) << DTOK_GEN_SHIFT) |
                  (((u32)udev->slot_id & DTOK_SLOT_MASK) << DTOK_SLOT_SHIFT) |
                  ((u32)ep_index & DTOK_EP_MASK));
}

APTR xhci_direct_roothub_token(u8 view_id, u8 ep_index)
{
    return (APTR)(DTOK_VALID | DTOK_ROOTHUB |
                  (((u32)view_id & DTOK_VIEW_MASK) << DTOK_VIEW_SHIFT) |
                  ((u32)ep_index & DTOK_EP_MASK));
}

static struct xhci_ctrl *hcd_ctrl(APTR hcd, u32 tok)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)hcd;

    if (!unit || !(tok & DTOK_VALID))
        return NULL;
    return unit->xhci_ctrl;
}

/* Resolve a device token under the lock; NULL = stale (destroyed device,
 * reused slot, dropped endpoint). */
static struct ep_context *token_resolve(struct xhci_ctrl *ctrl, u32 tok,
                                        struct usb_device **udev_out)
{
    struct usb_device *udev =
        ctrl->devices_by_slot_id[(tok >> DTOK_SLOT_SHIFT) & DTOK_SLOT_MASK];
    if (!udev || !udev->ctx_mode ||
        ((udev->token_gen & DTOK_GEN_MASK) != ((tok >> DTOK_GEN_SHIFT) & DTOK_GEN_MASK)))
        return NULL;

    *udev_out = udev;
    return xhci_ep_get_context_for_index(udev, (u8)(tok & DTOK_EP_MASK));
}

u32 xhci_direct_attach(struct IOStdReq *client)
{
    struct UhcdAttach *op = client->io_Data;
    struct XHCIUnit *unit = (struct XHCIUnit *)client->io_Unit;
    struct xhci_ctrl *ctrl = unit ? unit->xhci_ctrl : NULL;

    if (!ctrl || !op->ato_DoneHook)
    {
        client->io_Error = UHIOERR_BADPARAMS;
        return COMMAND_PROCESSED;
    }

    /* idempotent: a re-attach updates the hook identity in place */
    ctrl->stack_done_hook = op->ato_DoneHook;
    ctrl->stack_done_obj = op->ato_UserData;
    op->ato_HcdContext = unit;
    op->ato_Submit = (APTR)xhci_direct_submit;
    op->ato_CtrlSubmit = (APTR)xhci_direct_ctrl_submit;
    op->ato_Abort = (APTR)xhci_direct_abort;
    client->io_Error = UHIOERR_NO_ERROR;
    Kprintf("attached\n");
    return COMMAND_PROCESSED;
}

void xhci_direct_detach(struct XHCIUnit *unit)
{
    if (unit->xhci_ctrl)
        unit->xhci_ctrl->stack_done_hook = NULL;
}

/* Deliver a completion to the stack's done hook.  The carrying allocation is
 * already freed — the hook may submit and reuse the slot. */
static void direct_call_hook(struct xhci_ctrl *ctrl, APTR cookie, u32 actual, s8 error)
{
    struct Hook *hook = ctrl->stack_done_hook;

    struct UhcdXferDone done;
    done.uxd_Cookie = cookie;
    done.uxd_Actual = actual;
    done.uxd_ExtError = 0;
    done.uxd_Error = (UBYTE)error;
    done.uxd_Pad = 0;

    KprintfT("cookie %08lx err %ld actual %lu\n",
             (ULONG)done.uxd_Cookie, (LONG)(BYTE)done.uxd_Error, (ULONG)done.uxd_Actual);

    if (hook)
    {
        PERF_T0(hook_t0);
        CallHookPkt(hook, ctrl->stack_done_obj, &done);
        PERF_ADD(&ctrl->perf, XP_EVT_HOOK, hook_t0);
    }
}

void xhci_direct_done(struct xhci_xfer *io)
{
    struct xhci_ctrl *ctrl = io->ctrl;
    APTR cookie = io->cookie;
    u32 actual = io->actual;
    s8 error = io->error;

    slab_free(&ctrl->xfer_slab, io);
    direct_call_hook(ctrl, cookie, actual, error);
}

/* Completion of a deferred root-hub transfer: same delivery, but the xfer is
 * embedded in its (pooled) CMD_INTERNAL_RH_SUBMIT message. */
static void xhci_direct_rh_done(struct xhci_xfer *io)
{
    struct xhci_ctrl *ctrl = io->ctrl;
    APTR cookie = io->cookie;
    u32 actual = io->actual;
    s8 error = io->error;

    pool_free(ctrl->metaPool, (u8 *)io - offsetof(struct xhci_rh_submit_msg, rs_Xfer));
    direct_call_hook(ctrl, cookie, actual, error);
}

/* Hand a root-hub transfer to the unit task.  The cookie is set before the
 * post so an abort can match the request as soon as it parks. */
static LONG direct_rh_defer(struct XHCIUnit *unit, u32 tok,
                            const struct UhcdSetupData *setup,
                            APTR data, ULONG length, APTR cookie)
{
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    const u8 ep_index = (u8)(tok & DTOK_EP_MASK);

    /* EP0 is control-only; everything else on a root hub is the
     * status-change interrupt pipe */
    if ((setup && ep_index != 0) || (!setup && ep_index == 0))
        return UHIOERR_BADPARAMS;

    lock_prof_obtain(&ctrl->lockProf, &ctrl->xfer_lock);
    struct xhci_rh_submit_msg *msg = pool_zalloc(ctrl->metaPool, sizeof(*msg));
    lock_prof_release(&ctrl->lockProf, &ctrl->xfer_lock);
    if (!msg)
        return UHIOERR_OUTOFMEMORY;

    msg->rs_Req.io_Message.mn_Length = sizeof(*msg);
    msg->rs_Req.io_Unit = (struct Unit *)unit;
    msg->rs_Req.io_Command = CMD_INTERNAL_RH_SUBMIT;
    msg->rs_ViewId = (u8)((tok >> DTOK_VIEW_SHIFT) & DTOK_VIEW_MASK);

    struct xhci_xfer *io = &msg->rs_Xfer;
    io->ctrl = ctrl;
    io->complete = xhci_direct_rh_done;
    io->endpoint = (u8)((ep_index + 1) >> 1);
    if (setup)
    {
        io->type = UHCD_EPTYPE_CONTROL;
        io->setup = *setup;
        io->direction = (setup->usd_RequestType & 0x80U) ? XHCI_DIR_IN : XHCI_DIR_OUT;
    }
    else
    {
        io->type = UHCD_EPTYPE_INTERRUPT;
        io->direction = XHCI_DIR_IN;
    }
    io->data = data;
    io->data_length = length;
    io->cookie = cookie;

    PutMsg(&unit->unit.unit_MsgPort, (struct Message *)msg);
    return UHIOERR_NO_ERROR;
}

u32 xhci_direct_rh_submit(struct IORequest *ioreq)
{
    struct xhci_rh_submit_msg *msg = (struct xhci_rh_submit_msg *)ioreq;
    struct XHCIUnit *unit = (struct XHCIUnit *)ioreq->io_Unit;
    struct xhci_xfer *io = &msg->rs_Xfer;
    struct xhci_root_hub_view *v =
        xhci_roothub_view(unit->xhci_ctrl->root_hub, msg->rs_ViewId);

    if (!v)
    {
        io->error = UHIOERR_BADPARAMS;
        xhci_xfer_reply(io);
    }
    else if (io->type == UHCD_EPTYPE_CONTROL)
    {
        xhci_roothub_view_submit_ctrl_request(v, io); /* fills synchronously */
        xhci_xfer_reply(io);
    }
    else
    {
        s8 err = xhci_roothub_view_submit_int_request(v, io);
        if (err != UHIOERR_NO_ERROR)
        {
            io->error = err;
            xhci_xfer_reply(io);
        }
        /* else parked in v->int_req until a port change (or abort/flush) */
    }
    return COMMAND_SCHEDULED; /* driver-owned message: never replied */
}

/* Locked tail shared by the two submit entries: resolve the token, validate
 * the endpoint against the entry used (setup != NULL means ctrl_submit),
 * build the xfer and hand it to the ring.  Static with two call sites — the
 * compiler clones it per caller, so the setup branches fold away. */
static LONG direct_device_submit(struct xhci_ctrl *ctrl, u32 tok,
                                 const struct UhcdSetupData *setup,
                                 APTR data, ULONG length,
                                 ULONG naktimeout_ms, UWORD stream_id,
                                 UWORD flags, APTR cookie)
{
    lock_prof_obtain(&ctrl->lockProf, &ctrl->xfer_lock);

    struct usb_device *udev = NULL;
    struct ep_context *ep_ctx = token_resolve(ctrl, tok, &udev);
    if (!ep_ctx)
    {
        lock_prof_release(&ctrl->lockProf, &ctrl->xfer_lock);
        return UHIOERR_TIMEOUT; /* stale token — device-gone semantics */
    }
    (void)udev; /* logging only before the re-resolve below */

    /* control travels through ctrl_submit, everything else through submit */
    const u8 ep_index = (u8)(tok & DTOK_EP_MASK);
    const s32 ep_type = xhci_ep_type_for_index(udev, ep_index);
    if (setup ? (ep_type != USB_ENDPOINT_XFER_CONTROL)
              : (ep_type < 0 || ep_type == USB_ENDPOINT_XFER_CONTROL))
    {
        lock_prof_release(&ctrl->lockProf, &ctrl->xfer_lock);
        return UHIOERR_BADPARAMS;
    }

    /* The driver's STALL recovery already sent CLEAR_FEATURE(ENDPOINT_HALT)
     * to the device (xhci_udev_clear_feature_halt); the stack's follow-up
     * clear-halt is a duplicate — answer it without wire traffic, since a
     * second clear would reset the device's data toggle under traffic that
     * already resumed. */
    if (setup &&
        setup->usd_RequestType == (USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_ENDPOINT) &&
        setup->usd_Request == USB_REQ_CLEAR_FEATURE &&
        le16(setup->usd_Value) == USB_ENDPOINT_HALT)
    {
        const u8 target = xhci_ep_index_from_address((u8)(le16(setup->usd_Index) & 0xffU));
        if (xhci_ep_consume_halt_synced(xhci_ep_get_context_for_index(udev, target)))
        {
            struct Hook *hook = ctrl->stack_done_hook;
            KprintfT("slot %lu ep %lu: clear-halt already synced, completing locally\n",
                     (ULONG)udev->slot_id, (ULONG)target);
            if (hook)
            {
                struct UhcdXferDone done;
                done.uxd_Cookie = cookie;
                done.uxd_Actual = 0;
                done.uxd_ExtError = 0;
                done.uxd_Error = UHIOERR_NO_ERROR;
                done.uxd_Pad = 0;
                CallHookPkt(hook, ctrl->stack_done_obj, &done);
            }
            lock_prof_release(&ctrl->lockProf, &ctrl->xfer_lock);
            return UHIOERR_NO_ERROR;
        }
    }

    struct xhci_xfer *io = slab_zalloc(&ctrl->xfer_slab);
    if (!io)
    {
        lock_prof_release(&ctrl->lockProf, &ctrl->xfer_lock);
        return UHIOERR_OUTOFMEMORY;
    }

    io->ctrl = ctrl;
    io->complete = xhci_direct_done;
    io->priv_flags = REQ_DIRECT;
    io->endpoint = (u8)((ep_index + 1) >> 1);
    io->data = data;
    io->data_length = length;
    io->timeout_ms = naktimeout_ms;
    io->cookie = cookie;
    if (naktimeout_ms)
        io->flags |= XHCI_XF_TIMEOUT;

    if (setup)
    {
        io->type = UHCD_EPTYPE_CONTROL;
        io->setup = *setup;
        io->direction = (setup->usd_RequestType & 0x80U) ? XHCI_DIR_IN : XHCI_DIR_OUT;

        KprintfT("slot %lu ep %lu bmReq %02lx bReq %02lx len %lu cookie %08lx\n",
                 (ULONG)udev->slot_id, (ULONG)ep_index,
                 (ULONG)setup->usd_RequestType, (ULONG)setup->usd_Request,
                 (ULONG)length, (ULONG)cookie);
    }
    else
    {
        io->type = (u8)ep_type; /* USB_ENDPOINT_XFER_* == UHCD_EPTYPE_* numerically */
        io->direction = (ep_index & 1) ? XHCI_DIR_OUT : XHCI_DIR_IN;
        io->stream_id = stream_id;
        if (flags & UHCD_XFF_ALLOWRUNT)
            io->flags |= XHCI_XF_ALLOWRUNT;

        KprintfT("slot %lu ep %lu stream %lu len %lu cookie %08lx\n",
                 (ULONG)udev->slot_id, (ULONG)ep_index, (ULONG)stream_id,
                 (ULONG)length, (ULONG)cookie);
    }

    /* Two-phase DMA map: allocate under the lock, then run the payload work
     * (bounce copy + cache maintenance — the ms-scale part of a large
     * transfer) with the lock RELEASED so the transfer plane keeps moving. */
    const BOOL to_device = (io->direction == XHCI_DIR_OUT) ||
                           (setup && !(setup->usd_RequestType & 0x80U));
    if (length && !xhci_dma_premap(ctrl, io, to_device))
    {
        slab_free(&ctrl->xfer_slab, io);
        lock_prof_release(&ctrl->lockProf, &ctrl->xfer_lock);
        return UHIOERR_OUTOFMEMORY;
    }

    lock_prof_release(&ctrl->lockProf, &ctrl->xfer_lock);
    PERF_T0(map_t0);
    if (length)
        xhci_dma_map_sync(io, to_device);
    PERF_ADD(&ctrl->perf, XP_SUBMIT_MAP, map_t0);
    lock_prof_obtain(&ctrl->lockProf, &ctrl->xfer_lock);

    /* The device may have died while the lock was released — resolve again;
     * a stale token retires the prepared request without touching hardware. */
    udev = NULL;
    ep_ctx = token_resolve(ctrl, tok, &udev);
    if (!ep_ctx)
    {
        xhci_dma_unmap(ctrl, io, FALSE);
        slab_free(&ctrl->xfer_slab, io);
        lock_prof_release(&ctrl->lockProf, &ctrl->xfer_lock);
        return UHIOERR_TIMEOUT; /* device-gone semantics */
    }

    s8 err = xhci_ep_submit(ep_ctx, io);
    if (err != UHIOERR_NO_ERROR)
    {
        xhci_dma_unmap(ctrl, io, FALSE); /* state-gate rejects leave the map live */
        slab_free(&ctrl->xfer_slab, io);
    }

    lock_prof_release(&ctrl->lockProf, &ctrl->xfer_lock);
    return err;
}

LONG xhci_direct_submit(APTR hcd, APTR ep_token, APTR data, ULONG length,
                        ULONG naktimeout_ms, UWORD stream_id,
                        UWORD flags, APTR cookie)
{
    const u32 tok = (u32)ep_token;
    struct xhci_ctrl *ctrl = hcd_ctrl(hcd, tok);
    if (!ctrl)
        return UHIOERR_TIMEOUT;
    if (tok & DTOK_ROOTHUB)
        return direct_rh_defer((struct XHCIUnit *)hcd, tok, NULL, data, length, cookie);

    return direct_device_submit(ctrl, tok, NULL, data, length,
                                naktimeout_ms, stream_id, flags, cookie);
}

LONG xhci_direct_ctrl_submit(APTR hcd, APTR ep0_token,
                             const struct UhcdSetupData *setup,
                             APTR data, ULONG length,
                             ULONG naktimeout_ms, APTR cookie)
{
    const u32 tok = (u32)ep0_token;

    if (!setup)
        return UHIOERR_BADPARAMS;

    struct xhci_ctrl *ctrl = hcd_ctrl(hcd, tok);
    if (!ctrl)
        return UHIOERR_TIMEOUT;
    if (tok & DTOK_ROOTHUB)
        return direct_rh_defer((struct XHCIUnit *)hcd, tok, setup, data, length, cookie);

    return direct_device_submit(ctrl, tok, setup, data, length,
                                naktimeout_ms, 0, 0, cookie);
}

LONG xhci_direct_abort(APTR hcd, APTR ep_token, APTR cookie)
{
    const u32 tok = (u32)ep_token;
    struct xhci_ctrl *ctrl = hcd_ctrl(hcd, tok);
    if (!ctrl)
        return UHIOERR_NO_ERROR; /* a wish */

    lock_prof_obtain(&ctrl->lockProf, &ctrl->xfer_lock);

    if (tok & DTOK_ROOTHUB)
    {
        struct xhci_root_hub_view *v = xhci_roothub_view(ctrl->root_hub,
            (u8)((tok >> DTOK_VIEW_SHIFT) & DTOK_VIEW_MASK));
        if (v)
            xhci_roothub_view_abort_int_cookie(v, cookie);
    }
    else
    {
        struct usb_device *udev = NULL;
        struct ep_context *ep_ctx = token_resolve(ctrl, tok, &udev);
        if (ep_ctx)
            xhci_ep_abort_cookie(ep_ctx, cookie);
    }

    lock_prof_release(&ctrl->lockProf, &ctrl->xfer_lock);
    return UHIOERR_NO_ERROR; /* a wish, like AbortIO */
}
