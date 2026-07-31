// SPDX-License-Identifier: GPL-2.0-only
/*
 * Context HCD ABI (NSCMD_USB_*) ingress layer.
 *
 * Lifecycle ops arrive as plain IOStdReq commands (io_Data -> Uhcd* param
 * block); transfers never arrive here — they travel the direct call path
 * (xhci-direct.c), keyed by the endpoint tokens this layer writes into the
 * create/configure op OUT fields. At ingress every op gets a driver-owned
 * SHADOW xfer (struct xhci_ctx_shadow) so the existing machinery — command
 * ring, timeouts, queues — runs unchanged; the shadow's completion callback
 * (ctx_shadow_complete) copies the results back and replies the client.
 *
 * Devices created here are keyed by an opaque handle (the xHCI slot id).
 * Ops that issue command-ring work are flagged REQ_CTX_OP and replied from
 * the command completion handlers via xhci_ctxops_complete().
 *
 * The root hubs are emulated (no hardware slot): CREATE_DEVICE(parent=0)
 * returns UHCD_HANDLE_ROOTHUB (the SuperSpeed root hub) or
 * UHCD_HANDLE_ROOTHUB_USB2 keyed on cdo_Speed, every implemented lifecycle op
 * on a reserved handle is a successful no-op (delivering root-hub-flavored
 * endpoint tokens where the op returns tokens), and root-hub transfers route
 * to the matching protocol-pure view through the direct path's deferral.
 *
 * Design: poseidon-backport/docs/poseidon-context-hcd-abi.md.
 */

#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME (*(struct ExecBase **)4UL)
#include <proto/exec.h>
#endif

#include <exec/errors.h>

#include <debug.h>
#include <device.h>
#include <memory.h>

#include <xhci/xhci.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-context.h>
#include <xhci/xhci-ctx-ops.h>
#include <xhci/xhci-descriptors.h>
#include <xhci/xhci-direct.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-lpm.h>
#include <xhci/xhci-ring.h>
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-udev.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-ctx-ops] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef TRACE
#undef KprintfT
#define KprintfT(fmt, ...) PrintPistorm("[xhci-ctx-ops] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

/* Handle -> device. Handles are slot ids; only context-mode devices resolve. */
static struct usb_device *ctx_device_for_handle(struct xhci_ctrl *ctrl, u32 handle)
{
    if (handle == 0 || handle >= MAX_HC_SLOTS)
        return NULL;

    struct usb_device *udev = ctrl->devices_by_slot_id[handle];
    return (udev && udev->ctx_mode) ? udev : NULL;
}

/* A shadow's embedded xfer carries the client and, for lifecycle ops, the op
 * param block and command are read straight from the client (no stash). */
static inline APTR ctx_op_data(const struct xhci_xfer *io)
{
    return ((struct IOStdReq *)((struct xhci_ctx_shadow *)io)->client)->io_Data;
}
static inline u16 ctx_op_cmd(const struct xhci_xfer *io)
{
    return ((struct xhci_ctx_shadow *)io)->client->io_Command;
}

/* The shadow's completion callback (installed as io.complete): copy the result
 * back into the client IOStdReq, retire the shadow, reply the client. */
static void ctx_shadow_complete(struct xhci_xfer *io)
{
    struct xhci_ctx_shadow *sh = (struct xhci_ctx_shadow *)io;
    struct IORequest *client = sh->client;

    client->io_Error = io->error;
    ((struct IOStdReq *)client)->io_Actual = io->actual;
    pool_free(io->ctrl->metaPool, sh);
    ReplyMsg((struct Message *)client);
}

static struct xhci_ctx_shadow *ctx_shadow_new(struct xhci_ctrl *ctrl, struct IORequest *client)
{
    struct xhci_ctx_shadow *sh = pool_zalloc(ctrl->metaPool, sizeof(*sh));
    if (!sh)
        return NULL;

    sh->client = client;
    sh->io.ctrl = ctrl;
    sh->io.complete = ctx_shadow_complete;
    return sh;
}

/* Root-hub handle -> the protocol view it addresses (NULL for a handle the
 * controller never handed out). */
static struct xhci_root_hub_view *ctx_roothub_view_for_handle(struct xhci_ctrl *ctrl, u32 handle)
{
    switch (handle)
    {
    case UHCD_HANDLE_ROOTHUB:
        return xhci_roothub_view(ctrl->root_hub, RH_VIEW_SS);
    case UHCD_HANDLE_ROOTHUB_USB2:
        return xhci_roothub_view(ctrl->root_hub, RH_VIEW_USB2);
    default:
        return NULL;
    }
}

static u32 ctx_reply_error(struct xhci_xfer *io, s8 err)
{
    io->error = err;
    return COMMAND_PROCESSED;
}

static s8 ctx_map_speed(u16 uhcd_speed, enum usb_device_speed *out)
{
    switch (uhcd_speed)
    {
    case UHCD_SPEED_LOW:
        *out = USB_SPEED_LOW;
        return UHIOERR_NO_ERROR;
    case UHCD_SPEED_FULL:
        *out = USB_SPEED_FULL;
        return UHIOERR_NO_ERROR;
    case UHCD_SPEED_HIGH:
        *out = USB_SPEED_HIGH;
        return UHIOERR_NO_ERROR;
    case UHCD_SPEED_SUPER:
        *out = USB_SPEED_SUPER;
        return UHIOERR_NO_ERROR;
    case UHCD_SPEED_SUPERPLUS:
        *out = USB_SPEED_SUPER_PLUS;
        return UHIOERR_NO_ERROR;
    default:
        return UHIOERR_BADPARAMS;
    }
}

static u32 ctx_op_create_device(struct xhci_ctrl *ctrl, struct xhci_xfer *io)
{
    struct UhcdCreateDevice *op = ctx_op_data(io);

    /* The root hubs: emulated, no hardware slot, reserved handles. A
     * controller with both protocol port groups exposes TWO protocol-pure
     * root hubs; cdo_Speed selects which one this create refers to. */
    if (op->cdo_ParentHandle == 0)
    {
        if (op->cdo_Speed >= UHCD_SPEED_SUPER && xhci_roothub_has_usb3_ports(ctrl->root_hub))
            op->cdo_DeviceHandle = UHCD_HANDLE_ROOTHUB;
        else if (xhci_roothub_has_usb2_ports(ctrl->root_hub))
            op->cdo_DeviceHandle = UHCD_HANDLE_ROOTHUB_USB2;
        else
            op->cdo_DeviceHandle = UHCD_HANDLE_ROOTHUB; /* USB3-only controller */
        op->cdo_Ep0Token = xhci_direct_roothub_token(
            (op->cdo_DeviceHandle == UHCD_HANDLE_ROOTHUB) ? RH_VIEW_SS : RH_VIEW_USB2, 0);
        io->error = UHIOERR_NO_ERROR;
        KprintfT("create: root hub (speed %lu) -> handle %08lx\n",
                 (ULONG)op->cdo_Speed, (ULONG)op->cdo_DeviceHandle);
        return COMMAND_PROCESSED;
    }

    struct usb_device *parent = NULL;
    u8 parent_port = (u8)op->cdo_HubPort;
    if (op->cdo_ParentHandle < UHCD_HANDLE_RESERVED)
    {
        parent = ctx_device_for_handle(ctrl, op->cdo_ParentHandle);
        if (!parent)
            return ctx_reply_error(io, UHIOERR_BADPARAMS);
    }
    else
    {
        /* a root-hub handle: root-port device, parent stays NULL; translate
         * the view-local port to the controller-global number (the slot
         * context's root-hub-port field and LPM take global numbers) */
        struct xhci_root_hub_view *v = ctx_roothub_view_for_handle(ctrl, op->cdo_ParentHandle);
        if (!v)
            return ctx_reply_error(io, UHIOERR_BADPARAMS);
        parent_port = xhci_roothub_view_global_port(v, (u8)op->cdo_HubPort);
        if (!parent_port)
            return ctx_reply_error(io, UHIOERR_BADPARAMS);
    }

    enum usb_device_speed speed;
    if (ctx_map_speed(op->cdo_Speed, &speed) != UHIOERR_NO_ERROR)
        return ctx_reply_error(io, UHIOERR_BADPARAMS);

    struct usb_device *udev = xhci_udev_alloc_ctx(ctrl);
    if (!udev)
        return ctx_reply_error(io, UHIOERR_OUTOFMEMORY);

    /* cdo_TTHubHandle is informative only: the TT facts are derived from the
     * parent chain, and a reserved (root-hub) TT handle means "no TT" — the
     * xHC does the split translation for root ports itself. */
    udev->parent = parent; /* NULL = root port; the TT walk uses the parent chain */
    udev->parent_port = parent_port;
    udev->speed = speed;

    KprintfT("create: parent=%08lx port=%lu speed=%lu\n",
             (ULONG)op->cdo_ParentHandle, (ULONG)op->cdo_HubPort, (ULONG)speed);

    /* Enable Slot -> Address Device; handle_address_device() fills the handle
     * and replies via xhci_ctxops_complete(). */
    io->priv_flags |= REQ_CTX_OP;
    xhci_address_device(udev, io);
    return COMMAND_SCHEDULED;
}

static u32 ctx_op_destroy_device(struct xhci_ctrl *ctrl, struct xhci_xfer *io)
{
    struct UhcdDestroyDevice *op = ctx_op_data(io);

    struct usb_device *udev = ctx_device_for_handle(ctrl, op->ddo_DeviceHandle);
    if (!udev)
        return ctx_reply_error(io, UHIOERR_BADPARAMS);

    /* Tear down rings and queue Disable Slot; the reply does not wait for the
     * hardware — the slot is unreachable from this point on and a fresh
     * create allocates a new slot. */
    xhci_udev_disconnect(udev, FALSE);
    io->error = UHIOERR_NO_ERROR;
    return COMMAND_PROCESSED;
}

/* The stack validates bMaxPacketSize0, but a value the wire protocol cannot
 * carry must never reach the EP0 context — a shrunken EP0 turns every longer
 * control read into a babble error.  FS allows the full legal set per USB
 * spec; every other speed has exactly its default. */
static BOOL ctx_ep0_mps_valid(const struct usb_device *udev, u16 mps)
{
    if (udev->speed == USB_SPEED_FULL)
        return mps == 8 || mps == 16 || mps == 32 || mps == 64;
    return mps != 0 && mps == xhci_ep0_default_mps(udev->speed);
}

static u32 ctx_op_update_ep0(struct xhci_ctrl *ctrl, struct xhci_xfer *io)
{
    struct UhcdUpdateEp0 *op = ctx_op_data(io);

    struct usb_device *udev = ctx_device_for_handle(ctrl, op->ueo_DeviceHandle);
    if (!udev)
        return ctx_reply_error(io, UHIOERR_BADPARAMS);

    if (!ctx_ep0_mps_valid(udev, op->ueo_Ep0MaxPkt))
    {
        Kprintf("update_ep0: rejecting mps %lu for speed %lu\n",
                (ULONG)op->ueo_Ep0MaxPkt, (ULONG)udev->speed);
        return ctx_reply_error(io, UHIOERR_BADPARAMS);
    }

    io->priv_flags |= REQ_CTX_OP;

    if (!xhci_update_maxpacket(udev, op->ueo_Ep0MaxPkt, io))
        return ctx_reply_error(io, UHIOERR_NO_ERROR); /* already correct */

    return COMMAND_SCHEDULED;
}

static u32 ctx_op_configure_endpoints(struct xhci_ctrl *ctrl, struct xhci_xfer *io)
{
    struct UhcdConfigureEndpoints *op = ctx_op_data(io);
    if ((op->ceo_NumAdd && !op->ceo_Add) || (op->ceo_NumDrop && !op->ceo_DropAddresses))
        return ctx_reply_error(io, UHIOERR_BADPARAMS);

    struct usb_device *udev = ctx_device_for_handle(ctrl, op->ceo_DeviceHandle);
    if (!udev)
        return ctx_reply_error(io, UHIOERR_BADPARAMS);

    io->priv_flags |= REQ_CTX_OP;

    s8 err = xhci_configure_endpoints_from_list(udev,
                                                op->ceo_Add, op->ceo_NumAdd,
                                                op->ceo_DropAddresses, op->ceo_NumDrop,
                                                io);
    if (err != UHIOERR_NO_ERROR)
        return ctx_reply_error(io, err);

    return COMMAND_SCHEDULED;
}

static u32 ctx_op_deconfigure(struct xhci_ctrl *ctrl, struct xhci_xfer *io)
{
    struct UhcdDeconfigure *op = ctx_op_data(io);

    struct usb_device *udev = ctx_device_for_handle(ctrl, op->dco_DeviceHandle);
    if (!udev)
        return ctx_reply_error(io, UHIOERR_BADPARAMS);

    io->priv_flags |= REQ_CTX_OP;
    xhci_deconfigure(udev, io); /* completion sets the slot back to Addressed */
    return COMMAND_SCHEDULED;
}

static u32 ctx_op_update_hub(struct xhci_ctrl *ctrl, struct xhci_xfer *io)
{
    struct UhcdUpdateHub *op = ctx_op_data(io);

    struct usb_device *udev = ctx_device_for_handle(ctrl, op->uho_DeviceHandle);
    if (!udev)
        return ctx_reply_error(io, UHIOERR_BADPARAMS);

    udev->is_hub = TRUE;
    udev->hub_num_ports = (u8)op->uho_NumPorts;
    udev->tt_think_time = (u8)(op->uho_TTThinkTime & 0x3U);
    udev->ctx_mtt = op->uho_MultiTT ? 2 : 1;
    udev->hub_hdr_dec_lat = op->uho_HdrDecLat; /* SS hubs: exit-latency math input */
    udev->hub_delay = op->uho_HubDelay;

    io->priv_flags |= REQ_CTX_OP;
    xhci_apply_hub_update(udev, io);
    return COMMAND_SCHEDULED;
}

/* NSCMD_USB_RESET_DEVICE: the stack has just port-reset the device (Default
 * state on the wire, hub class owns the port); execute xHCI 4.6.11 Reset
 * Device and chain Address Device (BSR=0) — on success the handle comes back
 * Addressed with a fresh EP0, every other endpoint context is dropped and
 * everything in flight failed IOERR_ABORTED.  The stack rebuilds with
 * SET_CONFIGURATION + CONFIGURE_ENDPOINTS (+ ALLOC_STREAMS).  On failure the
 * slot may already be disabled: an op error is device-lost to the caller.
 * Root-hub handles resolve NULL here — a root hub is not port-resettable. */
static u32 ctx_op_reset_device(struct xhci_ctrl *ctrl, struct xhci_xfer *io)
{
    struct UhcdResetDevice *op = ctx_op_data(io);

    struct usb_device *udev = ctx_device_for_handle(ctrl, op->rdo_DeviceHandle);
    if (!udev)
        return ctx_reply_error(io, UHIOERR_BADPARAMS);

    if (udev->slot_state < USB_DEV_SLOT_STATE_ADDRESSED)
        return ctx_reply_error(io, UHIOERR_BADPARAMS);

    io->priv_flags |= REQ_CTX_OP;
    xhci_reset_device(udev, io);
    return COMMAND_SCHEDULED;
}

/* NSCMD_USB_SET_SUSPEND: quiesce (or restart) every endpoint ring of the
 * device.  The port transition itself stays the stack's job — the hub class
 * drives the link on external hubs and root-hub views alike; this op only
 * provides the xHCI 4.15.1 "stop all endpoints before U3" half. */
static u32 ctx_op_set_suspend(struct xhci_ctrl *ctrl, struct xhci_xfer *io)
{
    struct UhcdSetSuspend *op = ctx_op_data(io);

    struct usb_device *udev = ctx_device_for_handle(ctrl, op->sso_DeviceHandle);
    if (!udev)
        return ctx_reply_error(io, UHIOERR_BADPARAMS);

    if (!op->sso_Suspend)
    {
        io->error = UHIOERR_NO_ERROR;
        xhci_udev_resume_device(udev);
        return COMMAND_PROCESSED;
    }

    /* A colliding suspend sequence would be indistinguishable from "nothing
     * to stop" below — reject it loudly instead. */
    if (xhci_udev_suspend_pending(udev))
    {
        Kprintf("set_suspend: suspend already in flight on handle %lu\n",
                (ULONG)op->sso_DeviceHandle);
        return ctx_reply_error(io, UHIOERR_HOSTERROR);
    }

    if (!xhci_udev_suspend_device(udev, 0, io))
    {
        /* nothing to stop — the rings are already quiet */
        io->error = UHIOERR_NO_ERROR;
        return COMMAND_PROCESSED;
    }

    return COMMAND_SCHEDULED; /* replied from xhci_udev_suspend_finish() */
}

/* NSCMD_USB_SET_LINK_POWER: stack-parsed BOS facts + U1/U2 policy in; the HCD
 * computes the wire parameters and programs its own state, writing the results
 * back into *op for the stack to issue (SET_SEL, SET_FEATURE(U1/U2/LTM_ENABLE),
 * port SetPortFeature).  When an SS U1/U2 state is enabled the op latches MEL
 * with an Evaluate Context and replies only from that command's completion, so
 * the controller state is in place before the stack arms the port timeouts. */
static u32 ctx_op_set_link_power(struct xhci_ctrl *ctrl, struct xhci_xfer *io)
{
    struct UhcdSetLinkPower *op = ctx_op_data(io);

    struct usb_device *udev = ctx_device_for_handle(ctrl, op->slo_DeviceHandle);
    if (!udev)
        return ctx_reply_error(io, UHIOERR_BADPARAMS);

    if (!xhci_lpm_set_link_power(udev, op))
    {
        /* USB2 hardware LPM / LTM-only / nothing to arm: OUT fields already
         * filled, no Evaluate Context needed — reply synchronously. */
        io->error = UHIOERR_NO_ERROR;
        return COMMAND_PROCESSED;
    }

    io->priv_flags |= REQ_CTX_OP;
    xhci_evaluate_mel(udev, io);
    return COMMAND_SCHEDULED; /* replied from handle_config_ep */
}

/* NSCMD_USB_ALLOC_STREAMS / NSCMD_USB_FREE_STREAMS: switch an SS bulk
 * endpoint between its default single ring and per-stream transfer rings
 * (UAS).  The software half (stream rings + linear stream context array) is
 * built or kept here; the hardware half is a Configure Endpoint patching
 * MaxPStreams/LSA/deq, and the op replies from its completion — which also
 * rolls back the software state on an alloc failure, and frees it on a
 * successful free (see ctx_done_streams). */
static s8 ctx_streams_resolve(struct xhci_ctrl *ctrl, struct xhci_xfer *io,
                              struct usb_device **udev_out,
                              struct ep_context **ep_out, u8 *ep_index_out)
{
    struct UhcdStreams *op = ctx_op_data(io);

    struct usb_device *udev = ctx_device_for_handle(ctrl, op->sto_DeviceHandle);
    if (!udev)
        return UHIOERR_BADPARAMS;

    const u8 ep_index = xhci_ep_index_from_address(op->sto_EpAddress);
    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
    if (!ep_ctx)
        return UHIOERR_BADPARAMS;

    *udev_out = udev;
    *ep_out = ep_ctx;
    *ep_index_out = ep_index;
    return UHIOERR_NO_ERROR;
}

static u32 ctx_op_free_streams(struct xhci_ctrl *ctrl, struct xhci_xfer *io)
{
    struct usb_device *udev;
    struct ep_context *ep_ctx;
    u8 ep_index;
    s8 err = ctx_streams_resolve(ctrl, io, &udev, &ep_ctx, &ep_index);
    if (err != UHIOERR_NO_ERROR)
        return ctx_reply_error(io, err);

    Kprintf("free_streams: slot %lu ep %lu\n", (ULONG)udev->slot_id, (ULONG)ep_index);
    if (!xhci_ep_streams_active(ep_ctx))
        return ctx_reply_error(io, UHIOERR_NO_ERROR); /* idempotent */
    if (xhci_ep_get_active_trb_count(ep_ctx) != 0)
    {
        Kprintf("free_streams: slot %lu ep %lu still has TDs in flight\n",
                (ULONG)udev->slot_id, (ULONG)ep_index);
        return ctx_reply_error(io, UHIOERR_HOSTERROR);
    }

    io->priv_flags |= REQ_CTX_OP;
    xhci_configure_ep_stream_mode(udev, ep_index, FALSE, io);
    return COMMAND_SCHEDULED;
}

static u32 ctx_op_alloc_streams(struct xhci_ctrl *ctrl, struct xhci_xfer *io)
{
    struct UhcdStreams *op = ctx_op_data(io);
    struct usb_device *udev;
    struct ep_context *ep_ctx;
    u8 ep_index;
    s8 err = ctx_streams_resolve(ctrl, io, &udev, &ep_ctx, &ep_index);
    if (err != UHIOERR_NO_ERROR)
        return ctx_reply_error(io, err);

    Kprintf("alloc_streams: slot %lu ep %lu num=%lu\n",
            (ULONG)udev->slot_id, (ULONG)ep_index, (ULONG)op->sto_NumStreams);
    const u8 psa_cap = HCC_MAX_PSA_SIZE(mmio_read32(&ctrl->hccr->cr_hccparams1));
    if (psa_cap == 0)
        return ctx_reply_error(io, IOERR_NOCMD); /* controller has no stream support (and the NSD list said so) */

    if (op->sto_NumStreams == 0 ||
        xhci_ep_get_max_streams(ep_ctx) == 0 ||
        op->sto_NumStreams > xhci_ep_get_max_streams(ep_ctx) ||
        xhci_ep_streams_active(ep_ctx) ||
        xhci_ep_type_for_index(udev, ep_index) != USB_ENDPOINT_XFER_BULK)
        return ctx_reply_error(io, UHIOERR_BADPARAMS);

    if (xhci_ep_get_state(ep_ctx) != USB_DEV_EP_STATE_IDLE ||
        xhci_ep_get_active_trb_count(ep_ctx) != 0)
    {
        Kprintf("alloc_streams: slot %lu ep %lu not idle\n",
                (ULONG)udev->slot_id, (ULONG)ep_index);
        return ctx_reply_error(io, UHIOERR_HOSTERROR);
    }

    /* bound MaxPStreams at 5 (63 streams) beyond the controller cap: recovery
     * issues one Set TR Deq per stream ring, so keep the fleet sane */
    const u8 cap = psa_cap < 5 ? psa_cap : 5;
    err = xhci_ep_streams_build(ep_ctx, op->sto_NumStreams, cap);
    if (err != UHIOERR_NO_ERROR)
        return ctx_reply_error(io, err);

    io->priv_flags |= REQ_CTX_OP;
    xhci_configure_ep_stream_mode(udev, ep_index, TRUE, io);
    return COMMAND_SCHEDULED;
}

/* The clock-driven iso hooks (NSCMD_USB_REGISTER/UNREGISTER_HOOKS,
 * START/STOP_STREAM — ABI doc §10.3): the continuous iso engine keyed on
 * {handle, endpoint} with a struct USBIsoHooks block.  The hook block and
 * direction are passed to the engine by typed parameter (not packed into the
 * xfer); a STOP with TDs still in flight defers, handing the shadow xfer to the
 * engine as the deferred reply token. */
static u32 ctx_op_iso_hooks(struct xhci_ctrl *ctrl, struct xhci_xfer *io)
{
    struct UhcdIsoHooks *op = ctx_op_data(io);

    struct usb_device *udev = ctx_device_for_handle(ctrl, op->uio_DeviceHandle);
    if (!udev || !op->uio_Hooks)
        return ctx_reply_error(io, UHIOERR_BADPARAMS);

    const u8 direction = (op->uio_EpAddress & USB_DIR_IN) ? XHCI_DIR_IN : XHCI_DIR_OUT;
    const u8 endpoint = op->uio_EpAddress & 0x0fU;

    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev,
        xhci_ep_index_from_parts(endpoint, direction));
    if (!ep_ctx)
        return ctx_reply_error(io, UHIOERR_BADPARAMS);

    s8 result;
    switch (ctx_op_cmd(io))
    {
    case NSCMD_USB_REGISTER_HOOKS:
        result = xhci_ep_rt_iso_add_handler(ep_ctx, op->uio_Hooks, direction);
        break;
    case NSCMD_USB_UNREGISTER_HOOKS:
        result = xhci_ep_rt_iso_rem_handler(ep_ctx, op->uio_Hooks);
        break;
    case NSCMD_USB_START_STREAM:
        result = xhci_ep_rt_iso_start(ep_ctx);
        break;
    default: /* NSCMD_USB_STOP_STREAM */
        result = xhci_ep_rt_iso_stop(ep_ctx, op->uio_Hooks, io);
        if (result == UHIOERR_NO_ERROR)
            return COMMAND_SCHEDULED; /* replied when the rings drain */
        break;
    }

    io->error = result;
    return COMMAND_PROCESSED;
}

/* ---- Command-completion epilogues (op OUT fields / state restoration) ----
 * Run from xhci_ctxops_complete before the reply.  udev may be NULL on
 * late/timeout error paths — only success paths and state restoration need
 * it. */

static void ctx_done_create(struct usb_device *udev, struct xhci_xfer *io, s8 err)
{
    if (err == UHIOERR_NO_ERROR && udev)
    {
        struct UhcdCreateDevice *op = ctx_op_data(io);
        op->cdo_DeviceHandle = udev->slot_id;
        op->cdo_Ep0Token = xhci_direct_device_token(udev, 0);
        KprintfT("created handle %lu\n", (ULONG)udev->slot_id);
    }
}

static void ctx_done_configure(struct usb_device *udev, struct xhci_xfer *io, s8 err)
{
    if (err != UHIOERR_NO_ERROR || !udev)
        return;

    udev->slot_state = USB_DEV_SLOT_STATE_CONFIGURED;

    struct UhcdConfigureEndpoints *op = ctx_op_data(io);

    /* the added endpoints' submit tokens (direct path) */
    for (u16 i = 0; i < op->ceo_NumAdd; ++i)
        op->ceo_Add[i].ed_Token =
            xhci_direct_device_token(udev, xhci_ep_index(&op->ceo_Add[i]));

    /* dropped endpoints retire their software contexts with the hardware
     * drop — unless the same endpoint was re-added in this op (alt-setting
     * switch): its context now belongs to the new alternate (xHCI processes
     * drops before adds). */
    for (u16 i = 0; i < op->ceo_NumDrop; ++i)
    {
        const u8 ep_index = xhci_ep_index_from_address(op->ceo_DropAddresses[i]);
        BOOL readded = FALSE;
        for (u16 j = 0; j < op->ceo_NumAdd; ++j)
        {
            if (xhci_ep_index(&op->ceo_Add[j]) == ep_index)
            {
                readded = TRUE;
                break;
            }
        }
        if (!readded)
            xhci_ep_destroy_context(udev, ep_index, UHIOERR_TIMEOUT);
    }
}

static void ctx_done_deconfigure(struct usb_device *udev, struct xhci_xfer *io, s8 err)
{
    (void)io;
    if (err == UHIOERR_NO_ERROR && udev)
        udev->slot_state = USB_DEV_SLOT_STATE_ADDRESSED;
}

/* Software stream state commits with the hardware: an alloc whose Configure
 * Endpoint failed rolls back to the default ring; a successful free retires
 * the rings the hardware no longer sees. */
static void ctx_done_streams(struct usb_device *udev, struct xhci_xfer *io, s8 err)
{
    struct UhcdStreams *op = ctx_op_data(io);

    if (!udev)
    {
        /* timeout paths arrive without the device — recover it so the
         * rollback still runs */
        udev = ctx_device_for_handle(io->ctrl, op->sto_DeviceHandle);
    }
    if (udev)
    {
        if (err == UHIOERR_NO_ERROR)
            udev->slot_state = USB_DEV_SLOT_STATE_CONFIGURED;

        const u8 ep_index = xhci_ep_index_from_address(op->sto_EpAddress);
        struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
        const BOOL keep_streams = (ctx_op_cmd(io) == NSCMD_USB_ALLOC_STREAMS) == (err == UHIOERR_NO_ERROR);
        if (ep_ctx && !keep_streams)
            xhci_ep_streams_destroy(ep_ctx, IOERR_ABORTED); /* lists are empty: the op gates on zero active TRBs */
    }
}

/* ---- The op-descriptor table -------------------------------------------
 * One row per context op, indexed by io_Command - NSCMD_USBHCD_BASE.
 * ROM-able: static const (rodata), function pointers resolve at link time.
 *
 * CTXOP_RH_NOOP: the emulated root hubs have no hardware slot, so an
 * implemented lifecycle op targeting a reserved handle is a successful no-op
 * (the stack drives the root devices through the same code path as any other
 * device).  The handle is the leading ULONG of all per-device op blocks;
 * CREATE_DEVICE keys on cdo_ParentHandle instead and keeps its own root-hub
 * rule, and the iso-hook/stream ops resolve the handle themselves (a reserved
 * handle fails with UHIOERR_BADPARAMS — header contract: the root hubs have
 * no iso or SS bulk endpoints).
 * CTXOP_RH_TOKENS: the root-hub no-op still delivers the view's endpoint
 * submit tokens (CONFIGURE_ENDPOINTS only).
 *
 * NSCMD_USB_ATTACH (0x0b) is handled before the table (synchronous, no
 * shadow).  NSCMD_USB_RESET_DEVICE carries no CTXOP_RH_NOOP: a root hub is
 * not port-resettable, so a reserved handle fails with UHIOERR_BADPARAMS. */

#define CTXOP_RH_NOOP   0x01u
#define CTXOP_RH_TOKENS 0x02u

struct ctx_op_desc
{
    u32 (*dispatch)(struct xhci_ctrl *ctrl, struct xhci_xfer *io); /* NULL = IOERR_NOCMD */
    void (*complete)(struct usb_device *udev, struct xhci_xfer *io, s8 err); /* NULL = none */
    u8 flags;
};

static const struct ctx_op_desc ctx_ops[0x10] = {
    [NSCMD_USB_CREATE_DEVICE - NSCMD_USBHCD_BASE] = {ctx_op_create_device, ctx_done_create, 0},
    [NSCMD_USB_DESTROY_DEVICE - NSCMD_USBHCD_BASE] = {ctx_op_destroy_device, NULL, CTXOP_RH_NOOP},
    [NSCMD_USB_UPDATE_EP0 - NSCMD_USBHCD_BASE] = {ctx_op_update_ep0, NULL, CTXOP_RH_NOOP},
    [NSCMD_USB_CONFIGURE_ENDPOINTS - NSCMD_USBHCD_BASE] = {ctx_op_configure_endpoints, ctx_done_configure, CTXOP_RH_NOOP | CTXOP_RH_TOKENS},
    [NSCMD_USB_DECONFIGURE - NSCMD_USBHCD_BASE] = {ctx_op_deconfigure, ctx_done_deconfigure, CTXOP_RH_NOOP},
    [NSCMD_USB_RESET_DEVICE - NSCMD_USBHCD_BASE] = {ctx_op_reset_device, NULL, 0},
    [NSCMD_USB_UPDATE_HUB - NSCMD_USBHCD_BASE] = {ctx_op_update_hub, NULL, CTXOP_RH_NOOP},
    [NSCMD_USB_SET_SUSPEND - NSCMD_USBHCD_BASE] = {ctx_op_set_suspend, NULL, CTXOP_RH_NOOP},
    [NSCMD_USB_SET_LINK_POWER - NSCMD_USBHCD_BASE] = {ctx_op_set_link_power, NULL, CTXOP_RH_NOOP},
    [NSCMD_USB_ALLOC_STREAMS - NSCMD_USBHCD_BASE] = {ctx_op_alloc_streams, ctx_done_streams, 0},
    [NSCMD_USB_FREE_STREAMS - NSCMD_USBHCD_BASE] = {ctx_op_free_streams, ctx_done_streams, 0},
    [NSCMD_USB_REGISTER_HOOKS - NSCMD_USBHCD_BASE] = {ctx_op_iso_hooks, NULL, 0},
    [NSCMD_USB_UNREGISTER_HOOKS - NSCMD_USBHCD_BASE] = {ctx_op_iso_hooks, NULL, 0},
    [NSCMD_USB_START_STREAM - NSCMD_USBHCD_BASE] = {ctx_op_iso_hooks, NULL, 0},
    [NSCMD_USB_STOP_STREAM - NSCMD_USBHCD_BASE] = {ctx_op_iso_hooks, NULL, 0},
};

static const struct ctx_op_desc *ctx_op_lookup(u16 cmd)
{
    const u32 idx = (u32)cmd - NSCMD_USBHCD_BASE;
    return (idx < sizeof(ctx_ops) / sizeof(ctx_ops[0])) ? &ctx_ops[idx] : NULL;
}

u32 xhci_ctxops_process(struct IOStdReq *client)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)client->io_Unit;
    struct xhci_ctrl *ctrl = unit ? unit->xhci_ctrl : NULL;
    APTR op = client->io_Data;

    if (!ctrl || !op)
    {
        client->io_Error = UHIOERR_BADPARAMS;
        return COMMAND_PROCESSED;
    }

    /* the attach handshake is synchronous with OUT fields in the client's
     * own op block — no shadow, no command-ring work */
    if (client->io_Command == NSCMD_USB_ATTACH)
        return xhci_direct_attach(client);

    const struct ctx_op_desc *desc = ctx_op_lookup(client->io_Command);
    if (!desc || !desc->dispatch)
    {
        /* reserved (RESET_DEVICE) and retired ops of the block — per-op
         * discovery is the NSD list, so an absent op is a clean IOERR_NOCMD */
        client->io_Error = IOERR_NOCMD;
        return COMMAND_PROCESSED;
    }

    if ((desc->flags & CTXOP_RH_NOOP) && *(u32 *)op >= UHCD_HANDLE_RESERVED)
    {
        const u32 handle = *(u32 *)op;
        if ((desc->flags & CTXOP_RH_TOKENS) &&
            ((handle == UHCD_HANDLE_ROOTHUB) || (handle == UHCD_HANDLE_ROOTHUB_USB2)))
        {
            struct UhcdConfigureEndpoints *ceo = op;
            const u8 view_id = (handle == UHCD_HANDLE_ROOTHUB) ? RH_VIEW_SS : RH_VIEW_USB2;
            for (u16 i = 0; i < ceo->ceo_NumAdd; ++i)
                ceo->ceo_Add[i].ed_Token =
                    xhci_direct_roothub_token(view_id, xhci_ep_index(&ceo->ceo_Add[i]));
        }
        KprintfT("cmd %lu on root-hub handle: no-op\n", (ULONG)client->io_Command);
        client->io_Error = UHIOERR_NO_ERROR;
        return COMMAND_PROCESSED;
    }

    struct xhci_ctx_shadow *sh = ctx_shadow_new(ctrl, (struct IORequest *)client);
    if (!sh)
    {
        client->io_Error = UHIOERR_OUTOFMEMORY;
        return COMMAND_PROCESSED;
    }

    u32 complete = desc->dispatch(ctrl, &sh->io);
    if (complete == COMMAND_PROCESSED)
    {
        /* synchronous outcome: propagate and retire the shadow; the unit-task
         * dispatcher replies the client */
        client->io_Error = sh->io.error;
        pool_free(ctrl->metaPool, sh);
    }
    return complete;
}

void xhci_ctxops_complete(struct usb_device *udev, struct xhci_xfer *io, s8 err)
{
    const struct ctx_op_desc *desc = ctx_op_lookup(ctx_op_cmd(io));
    if (desc && desc->complete)
        desc->complete(udev, io, err);

    io->error = err;
    xhci_xfer_reply(io);
}

