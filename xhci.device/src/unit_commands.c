// SPDX-License-Identifier: GPL-2.0-only
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#include <clib/utility_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME (*(struct ExecBase **)4UL)
#include <proto/exec.h>
#define UTILITY_BASE_NAME unit->device->utilityBase
#include <proto/utility.h>
#endif

#include <exec/errors.h>
#include <utility/tagitem.h>

#include <devices/newstyle.h>

#include <config.h>
#include <device.h>
#include <debug.h>
#include <memory.h>
#include <format.h>
#include <libraries/openpci.h>
#include <xhci/usb_defs.h>
#include <xhci/xhci.h>
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci-ctx-ops.h>
#include <xhci/xhci-direct.h>

/* The NSD list is the per-op discovery mechanism, so ONLY implemented ops may
 * appear in it.  The stream ops depend on controller support (HCCPARAMS1
 * MaxPSASize > 0), so two tables exist: the base list and the streams-capable
 * superset — NSCMD_DEVICEQUERY picks at query time. */
#define XHCI_NSD_COMMON_COMMANDS                                              \
    CMD_FLUSH,                                                                \
    UHCMD_QUERYDEVICE,                                                         \
    UHCMD_USBRESET,                                                         \
                                                                              \
    NSCMD_DEVICEQUERY,                                                        \
                                                                              \
    /* Context HCD ABI lifecycle ops (usbhcd_context.h) */                    \
    NSCMD_USB_CREATE_DEVICE,                                                  \
    NSCMD_USB_DESTROY_DEVICE,                                                 \
    NSCMD_USB_UPDATE_EP0,                                                     \
    NSCMD_USB_CONFIGURE_ENDPOINTS,                                            \
    NSCMD_USB_DECONFIGURE,                                                    \
    NSCMD_USB_RESET_DEVICE,                                                   \
    NSCMD_USB_UPDATE_HUB,                                                     \
    NSCMD_USB_SET_SUSPEND,                                                    \
    NSCMD_USB_SET_LINK_POWER,                                                 \
                                                                              \
    /* the direct transfer path's attach handshake (xhci-direct.c) +          \
     * clock-driven iso hooks (ABI doc §10) */                                \
    NSCMD_USB_ATTACH,                                                         \
    NSCMD_USB_REGISTER_HOOKS,                                                 \
    NSCMD_USB_UNREGISTER_HOOKS,                                               \
    NSCMD_USB_START_STREAM,                                                   \
    NSCMD_USB_STOP_STREAM

static const UWORD SupportedCommands[] = {
    XHCI_NSD_COMMON_COMMANDS,
    0};

static const UWORD SupportedCommandsStreams[] = {
    XHCI_NSD_COMMON_COMMANDS,
    /* SS bulk streams (UAS) — only when HCCPARAMS1 MaxPSASize > 0 */
    NSCMD_USB_ALLOC_STREAMS,
    NSCMD_USB_FREE_STREAMS,
    0};

#pragma pack(2)
/* Read-only overlay for the incoming UHCMD_QUERYDEVICE request.  The stack sends
 * the legacy struct IOUsbHWReq even to a context HCD; the driver reads only the
 * taglist at the frozen iouh_Data offset (52), so it needs no legacy struct. */
struct xhci_device_query
{
    struct IORequest dq_Req;    /* 0..31 */
    UWORD dq_Pad[6];            /* 32..43: iouh_Flags..iouh_MaxPktSize */
    ULONG dq_Actual;            /* 44: OUT filled-tag count (iouh_Actual) */
    ULONG dq_Length;            /* 48: iouh_Length */
    APTR  dq_TagList;           /* 52: iouh_Data -> query taglist */
};
#pragma pack()

static u32 Do_NSCMD_DEVICEQUERY(struct IOStdReq *io)
{
    KprintfT("[xhci] %s: NSCMD_DEVICEQUERY\n", __func__);
    struct NSDeviceQueryResult *dq = io->io_Data;
    struct XHCIUnit *unit = (struct XHCIUnit *)io->io_Unit;

    /* Fill out structure */
    dq->nsdqr_SizeAvailable = sizeof(struct NSDeviceQueryResult);
    if (io->io_Length < dq->nsdqr_SizeAvailable)
    {
        io->io_Error = IOERR_BADLENGTH;
        return COMMAND_PROCESSED;
    }
    dq->nsdqr_DeviceType = NSDEVTYPE_UNKNOWN;
    dq->nsdqr_DeviceSubType = 0;
    dq->nsdqr_SupportedCommands = (UWORD *)SupportedCommands;
    if (unit && unit->xhci_ctrl &&
        HCC_MAX_PSA_SIZE(mmio_read32(&unit->xhci_ctrl->hccr->cr_hccparams1)) > 0)
    {
        Kprintf("[xhci] %s: controller supports streams; returning stream-capable NSD\n", __func__);
        dq->nsdqr_SupportedCommands = (UWORD *)SupportedCommandsStreams;
    }
    io->io_Actual = dq->nsdqr_SizeAvailable;
    io->io_Error = 0;

    return COMMAND_PROCESSED;
}

static inline void flush_queued_unit_request(struct XHCIUnit *unit, struct IORequest *req)
{
    (void)unit;

    /* A driver-owned root-hub submit still queued: complete it through its
     * embedded xfer (which frees the message); nothing to reply. */
    if (req->io_Command == CMD_INTERNAL_RH_SUBMIT)
    {
        struct xhci_xfer *io = &((struct xhci_rh_submit_msg *)req)->rs_Xfer;
        io->error = IOERR_ABORTED;
        xhci_xfer_reply(io);
        return;
    }

    /* An unprocessed client request (never became a shadow): reply it directly. */
    req->io_Error = IOERR_ABORTED;
    ReplyMsg((struct Message *)req);
}

/*
 * Abort all transfer requests in progress or queued (context transfers and
 * the root-hub views' pending interrupt requests alike)
 */
static u32 Do_CMD_FLUSH(struct IORequest *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->io_Unit;
    KprintfT("[xhci] %s: CMD_FLUSH\n", __func__);

    struct IORequest *req;
    /* Flush and cancel all requests */
    while ((req = (struct IORequest *)GetMsg(&unit->unit.unit_MsgPort)))
        flush_queued_unit_request(unit, req);

    /* go through all devices and endpoints and flush their queues */
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;

    xhci_roothub_abort_int_request(ctrl->root_hub);

    /* Sweep the slot map: it holds every ring-bearing device, and CMD_FLUSH
     * must reply *everything*.  The root hub has no slot; its interrupt
     * request was aborted above. */
    for (u32 slot = 1; slot < MAX_HC_SLOTS; ++slot)
    {
        struct usb_device *udev = ctrl->devices_by_slot_id[slot];
        if (!udev)
            continue;

        /* A SET_SUSPEND op caught mid-sequence is a pending request too. */
        xhci_udev_suspend_cancel(udev, IOERR_ABORTED);

        for (u8 ep_index = 0; ep_index < USB_MAX_ENDPOINT_CONTEXTS; ++ep_index)
        {
            struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
            if (ep_ctx)
            {
                xhci_ep_flush(ep_ctx, IOERR_ABORTED);
                xhci_ep_request_stop(ep_ctx);
            }
        }
    }

    KprintfT("[xhci] %s: Flush completed\n", __func__);
    return COMMAND_PROCESSED;
}

static inline u32 Do_CMD_DEVICE_QUERY(struct IORequest *io)
{
    KprintfT("[xhci] %s: UHCMD_QUERYDEVICE\n", __func__);

    /* UHCMD_QUERYDEVICE arrives as a legacy IOUsbHWReq; the taglist rides its
     * iouh_Data slot (see struct xhci_device_query). */
    struct xhci_device_query *q = (struct xhci_device_query *)io;
    if (!q->dq_TagList)
    {
        io->io_Error = UHIOERR_BADPARAMS;
        return COMMAND_PROCESSED;
    }

    struct XHCIUnit *unit = (struct XHCIUnit *)io->io_Unit;

    struct TagItem *tag, *tagList = (struct TagItem *)q->dq_TagList;
    u32 filled = 0;
    KprintfT("[xhci] %s: Processing tag list at 0x%lx\n", __func__, tagList);
    while ((tag = NextTagItem(&tagList)))
    {
        if (!tag->ti_Data)
        {
            continue;
        }
        ULONG *out = (ULONG *)tag->ti_Data;
        switch (tag->ti_Tag)
        {
        case UHA_Manufacturer:
            if (!unit->xhci_ctrl->pci_dev)
                *out = (ULONG)(APTR) "Broadcom";
            else if (unit->xhci_ctrl->pci_dev->vendor == 0x1106)
                *out = (ULONG)(APTR) "VIA Labs";
            else
            {
                _SNPrintf((STRPTR)unit->vendor_str, sizeof(unit->vendor_str),
                          (CONST_STRPTR)"%04lx", (ULONG)unit->xhci_ctrl->pci_dev->vendor);
                *out = (ULONG)(APTR)unit->vendor_str;
            }
            filled++;
            break;
        case UHA_ProductName:
            if (!unit->xhci_ctrl->pci_dev)
                *out = (ULONG)(APTR) "BCM2711 xHCI";
            else if (unit->xhci_ctrl->pci_dev->device == 0x3483)
                *out = (ULONG)(APTR) "VL805 xHCI";
            else
            {
                _SNPrintf((STRPTR)unit->device_str, sizeof(unit->device_str),
                          (CONST_STRPTR)"%04lx", (ULONG)unit->xhci_ctrl->pci_dev->device);
                *out = (ULONG)(APTR)unit->device_str;
            }
            filled++;
            break;
        case UHA_Version:
            *out = DEVICE_VERSION;
            filled++;
            break;
        case UHA_Revision:
            *out = DEVICE_REVISION;
            filled++;
            break;
        case UHA_Description:
            *out = (ULONG)(APTR) "Generic xHCI USB Controller Driver";
            filled++;
            break;
        case UHA_Copyright:
            *out = (ULONG)(APTR) "GPLv2";
            filled++;
            break;
        case UHA_DriverVersion:
            // BCD of IO request structure version: support V2
            *out = 0x0200;
            filled++;
            break;
        case UHA_Capabilities:
            /* No QUICK_IO: BeginIO always defers to the unit task, so a
             * caller relying on synchronous IOF_QUICK completion would hang. */
            *out = UHCF_USB20 | UHCF_ISO | UHCF_RT_ISO |
                   UHCF_CONTEXT;  /* lifecycle-op ABI; optional ops via the NSD list */
            /* USB3 only when USB3-protocol root ports actually exist — the
             * stack's SuperSpeed root-hub attempt keys on this bit */
            if (unit->xhci_ctrl && xhci_roothub_has_usb3_ports(unit->xhci_ctrl->root_hub))
                *out |= UHCF_USB30;
            filled++;
            break;
        case UHA_NumRootHubs:
            /* context path: protocol-split root hubs (see usbhcd_context.h) */
            *out = 1;
            if (unit->xhci_ctrl &&
                xhci_roothub_has_usb3_ports(unit->xhci_ctrl->root_hub) &&
                xhci_roothub_has_usb2_ports(unit->xhci_ctrl->root_hub))
                *out = 2;
            filled++;
            break;
        case UHA_DMAAlignment:
            /* Cache-line granularity a data buffer must meet to be DMA'd
             * directly (else xhci_dma_span_map bounces it); lets the stack
             * place filesystem buffers to avoid the copy. */
            *out = DMA_ALIGN_MIN;
            filled++;
            break;
        default:
            // Unknown tag: leave untouched
            KprintfT("[xhci] %s: Unknown tag 0x%lx, skipping\n", __func__, tag->ti_Tag);
            break;
        }
        KprintfT("[xhci] %s: Processed tag 0x%lx\n", __func__, tag->ti_Tag);
    }

    KprintfT("[xhci] %s: Completed UHCMD_QUERYDEVICE\n", __func__);
    io->io_Error = UHIOERR_NO_ERROR;
    q->dq_Actual = filled; /* iouh_Actual: number of tags answered */
    return COMMAND_PROCESSED;
}

/*
 * Apply one hub-class port request to every root port, iterating the SS and
 * USB2 protocol views.  Driver-owned synthetic requests: view-local port
 * numbers, translated to controller-global at dispatch; the handlers
 * complete them in place.  Devices on the ports get the shared port
 * handlers' treatment — notably the xHCI 4.15.1 ring quiesce before a port
 * suspend and the ring restart on resume.
 */
static void roothub_all_ports_request(struct xhci_ctrl *ctrl, u8 bRequest, u16 feature)
{
    static const u8 view_ids[] = {RH_VIEW_SS, RH_VIEW_USB2};

    for (u8 i = 0; i < sizeof(view_ids); ++i)
    {
        struct xhci_root_hub_view *v = xhci_roothub_view(ctrl->root_hub, view_ids[i]);
        if (!v)
            continue;

        for (u8 p = 1; xhci_roothub_view_global_port(v, p) != 0; ++p)
        {
            struct xhci_xfer req;
            memset(&req, 0, sizeof(req));
            req.setup.usd_RequestType = USB_DIR_OUT | USB_RT_PORT; /* class=hub, recipient=other */
            req.setup.usd_Request = bRequest;
            req.setup.usd_Value = le16(feature);
            req.setup.usd_Index = le16(p);

            xhci_roothub_view_submit_ctrl_request(v, &req);
        }
    }
}

/*
 * reset USB bus
 */
static inline u32 Do_CMD_DEVICE_RESET(struct IORequest *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->io_Unit;
    KprintfT("[xhci] %s: UHCMD_USBRESET\n", __func__);

    /* Issue SET_FEATURE(RESET) on all root hub ports */
    roothub_all_ports_request(unit->xhci_ctrl, USB_REQ_SET_FEATURE, USB_PORT_FEAT_RESET);

    io->io_Error = UHIOERR_NO_ERROR;

    return COMMAND_PROCESSED;
}

void ProcessCommand(struct IORequest *io)
{
    u32 complete = COMMAND_SCHEDULED;

    /*
        Only NSCMD_DEVICEQUERY can use standard sized request. All other must be of
        size IORequest
    */
    if (io->io_Message.mn_Length < sizeof(struct IORequest) &&
        io->io_Command != NSCMD_DEVICEQUERY)
    {
        io->io_Error = IOERR_BADLENGTH;
        complete = COMMAND_PROCESSED;
    }
    else if (UHCD_IS_CTXCMD(io->io_Command))
    {
        /* The whole context-op block (NSCMD_USBHCD_BASE + 0x00..0x1f) routes
         * to the ctx-ops descriptor table; unimplemented slots (RESET_DEVICE,
         * reserved ops) reply IOERR_NOCMD there, matching their absence from
         * the NSD list. */
        io->io_Error = UHIOERR_NO_ERROR;
        complete = xhci_ctxops_process((struct IOStdReq *)io);
    }
    else
    {
        io->io_Error = UHIOERR_NO_ERROR;

        switch (io->io_Command)
        {
        case CMD_FLUSH:
            complete = Do_CMD_FLUSH(io);
            break;

        case UHCMD_QUERYDEVICE:
            complete = Do_CMD_DEVICE_QUERY(io);
            break;

        case UHCMD_USBRESET:
            complete = Do_CMD_DEVICE_RESET(io);
            break;

        case CMD_INTERNAL_RH_SUBMIT:
            complete = xhci_direct_rh_submit(io);
            break;

        case NSCMD_DEVICEQUERY:
            complete = Do_NSCMD_DEVICEQUERY((struct IOStdReq *)io);
            break;

        default:
            Kprintf("[xhci] %s: Unsupported command %lu\n", __func__, (ULONG)io->io_Command);
            io->io_Error = IOERR_NOCMD;
            complete = COMMAND_PROCESSED;
            break;
        }
    }

    /* Reply completed commands now.  IOF_QUICK never survives to this point:
     * beginIO strips it from every client request, and driver-internal
     * messages (CMD_INTERNAL_RH_SUBMIT) return COMMAND_SCHEDULED. */
    if (complete == COMMAND_PROCESSED)
        ReplyMsg((struct Message *)io);
}
