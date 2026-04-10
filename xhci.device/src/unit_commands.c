// SPDX-License-Identifier: GPL-2.0+
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
#include <devices/hcd_api.h>

#include <config.h>
#include <device.h>
#include <debug.h>
#include <emu_memory.h>
#include <xhci/usb_defs.h>
#include <xhci/xhci.h>
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-descriptors.h>
#include <xhci/xhci-td.h>
#include <xhci/xhci-udev.h>

static const UWORD SupportedCommands[] = {
    CMD_FLUSH,
    CMD_RESET,
    CMD_DEVICE_QUERY,
    CMD_DEVICE_RESET,
    CMD_DEVICE_RESUME,
    CMD_STOP,
    CMD_START,
    CMD_REQUEST_CONTROL,
    CMD_REQUEST_ISOCHRONOUS,
    CMD_REQUEST_INTERRUPT,
    CMD_REQUEST_BULK,

    NSCMD_DEVICEQUERY,
    0};

static int Do_NSCMD_DEVICEQUERY(struct IOStdReq *io)
{
    KprintfH("[xhci] %s: NSCMD_DEVICEQUERY\n", __func__);
    struct NSDeviceQueryResult *dq = io->io_Data;

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
    io->io_Actual = dq->nsdqr_SizeAvailable;
    io->io_Error = 0;

    return COMMAND_PROCESSED;
}

static inline void flush_queued_unit_request(struct XHCIUnit *unit, struct USBIORequest *req)
{
    if ((req->driver_private_flags & REQ_INTERNAL) && req->req.io_Command == CMD_INTERNAL_ABORT_REQUEST)
    {
        if (unit && unit->memoryPool)
            FreeVecPooled(unit->memoryPool, req);
        return;
    }

    req->req.io_Error = IOERR_ABORTED;
    ReplyMsg((struct Message *)req);
}

/*
 * Abort all UHCMD_CONTROLXFER, UHCMD_ISOXFER, UHCMD_INTXFER and UHCMD_BULKXFER requests in progress or queued
 */
static int Do_CMD_FLUSH(struct USBIORequest *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->req.io_Unit;
    KprintfH("[xhci] %s: CMD_FLUSH\n", __func__);

    struct USBIORequest *req;
    /* Flush and cancel all requests */
    while ((req = (struct USBIORequest *)GetMsg(&unit->unit.unit_MsgPort)))
        flush_queued_unit_request(unit, req);

    /* go through all devices and endpoints and flush their queues */
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;

    xhci_roothub_abort_int_request(ctrl->root_hub);

    for (unsigned int addr = 0; addr <= USB_MAX_ADDRESS; ++addr)
    {
        struct usb_device *udev = ctrl->devices_by_virtual_address[addr];
        if (!udev || addr == xhci_roothub_get_address(ctrl->root_hub))
            continue;

        for (unsigned int ep_index = 0; ep_index < USB_MAX_ENDPOINT_CONTEXTS; ++ep_index)
        {
            struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
            if (ep_ctx)
            {
                xhci_ep_flush(ep_ctx, IOERR_ABORTED);
                xhci_ep_request_stop(ep_ctx);
            }
        }
    }

    KprintfH("[xhci] %s: Flush completed\n", __func__);
    return COMMAND_PROCESSED;
}

static void uword_to_hex(UWORD value, UBYTE *buf)
{
    static const char hex[] = "0123456789abcdef";
    buf[0] = hex[(value >> 12) & 0xF];
    buf[1] = hex[(value >> 8) & 0xF];
    buf[2] = hex[(value >> 4) & 0xF];
    buf[3] = hex[value & 0xF];
    buf[4] = '\0';
}

static inline int Do_CMD_DEVICE_QUERY(struct USBIORequest *io)
{
    KprintfH("[xhci] %s: CMD_DEVICE_QUERY\n", __func__);

    if (!io->data_buffer)
    {
        io->req.io_Error = ERR_BAD_PARAMETERS;
        return COMMAND_PROCESSED;
    }

    struct XHCIUnit *unit = (struct XHCIUnit *)io->req.io_Unit;

    struct TagItem *tag, *tagList = (struct TagItem *)io->data_buffer;
    int filled = 0;
    KprintfH("[xhci] %s: Processing tag list at 0x%lx\n", __func__, tagList);
    while ((tag = NextTagItem(&tagList)))
    {
        if (!tag->ti_Data)
        {
            continue;
        }
        ULONG *out = (ULONG *)tag->ti_Data;
        switch (tag->ti_Tag)
        {
        case TAG_DRIVER_STATE:
            // TODO: derive from internal unit state
            *out = DRIVER_STATE_OPERATIONAL;
            io->state = DRIVER_STATE_OPERATIONAL;
            filled++;
            break;
        case TAG_DEVICE_VENDOR:
            if (unit->xhci_ctrl->pci_dev)
            {
                uword_to_hex(unit->xhci_ctrl->pci_dev->vendor, (UBYTE *)unit->vendor_str);
                *out = (ULONG)(APTR)unit->vendor_str;
            }
            else
                *out = (ULONG)(APTR) "Broadcom";
                
            filled++;
            break;
        case TAG_DEVICE_PRODUCT:
            if (unit->xhci_ctrl->pci_dev)
            {
                uword_to_hex(unit->xhci_ctrl->pci_dev->device, (UBYTE *)unit->device_str);
                *out = (ULONG)(APTR)unit->device_str;
            }
            else
                *out = (ULONG)(APTR) "OTG controller";

            filled++;
            break;
        case TAG_DEVICE_VERSION:
            *out = DEVICE_VERSION;
            filled++;
            break;
        case TAG_DEVICE_REVISION:
            *out = DEVICE_REVISION;
            filled++;
            break;
        case TAG_DRIVER_DESCRIPTION:
            *out = (ULONG)(APTR) "Generic xHCI USB Controller Driver";
            filled++;
            break;
        case TAG_DRIVER_LICENSE:
            *out = (ULONG)(APTR) "GPLv2";
            filled++;
            break;
        case TAG_DRIVER_VERSION:
            // BCD of IO request structure version: support V2
            *out = 0x0200;
            filled++;
            break;
        case TAG_DRIVER_FEATURES:
            *out = DRIVER_FEAT_USB2 | DRIVER_FEAT_USB3 | DRIVER_FEAT_ISOCHRONOUS | DRIVER_FEAT_ISOCHRONOUS_HOOKS; // | DRIVER_FEAT_QUICK_IO;
            filled++;
            break;
        default:
            // Unknown tag: leave untouched
            KprintfH("[xhci] %s: Unknown tag 0x%lx, skipping\n", __func__, tag->ti_Tag);
            break;
        }
        KprintfH("[xhci] %s: Processed tag 0x%lx\n", __func__, tag->ti_Tag);
    }

    KprintfH("[xhci] %s: Completed UHCMD_QUERYDEVICE\n", __func__);
    io->req.io_Error = ERR_NO_ERROR;
    io->actual_length = filled;
    return COMMAND_PROCESSED;
}

/*
 * reset USB bus
 */
static inline int Do_CMD_DEVICE_RESET(struct USBIORequest *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->req.io_Unit;
    Kprintf("[xhci] %s: CMD_DEVICE_RESET\n", __func__);

    /* Issue SET_FEATURE(RESET) on all root hub ports */
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    int maxp = xhci_roothub_get_num_ports(ctrl->root_hub);
    for (int p = 1; p <= maxp; ++p)
    {
        struct USBIORequest req;
        _memset(&req, 0, sizeof(req));
        req.setup.bmRequestType = USB_DIR_OUT | USB_RT_PORT; /* class=hub, recipient=other */
        req.setup.bRequest = USB_REQ_SET_FEATURE;
        req.setup.wValue = LE16(USB_PORT_FEAT_RESET);
        req.setup.wIndex = LE16(p);
        req.setup.wLength = LE16(0);
        req.virtual_address = xhci_roothub_get_address(ctrl->root_hub);

        xhci_roothub_submit_ctrl_request(ctrl->root_hub, &req);
    }

    io->req.io_Error = ERR_NO_ERROR;
    io->state = (io->req.io_Error == ERR_NO_ERROR) ? DRIVER_STATE_RESETING : 0;

    return COMMAND_PROCESSED;
}

static int Do_CMD_RESET(struct USBIORequest *io)
{
    // struct XHCIUnit *unit = (struct XHCIUnit *)io->req.io_Unit;
    Kprintf("[xhci] %s: CMD_RESET\n", __func__);
    // TODO should reset entire controller...
    return Do_CMD_DEVICE_RESET(io);
}

/*
 * resume from sleep mode
 */
static inline int Do_CMD_DEVICE_RESUME(struct USBIORequest *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->req.io_Unit;
    Kprintf("[xhci] %s: CMD_DEVICE_RESUME - resuming USB\n", __func__);

    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    int maxp = xhci_roothub_get_num_ports(ctrl->root_hub);
    for (int p = 1; p <= maxp; ++p)
    {
        struct USBIORequest req;
        _memset(&req, 0, sizeof(req));
        req.setup.bmRequestType = USB_DIR_OUT | USB_RT_PORT;
        req.setup.bRequest = USB_REQ_CLEAR_FEATURE;
        req.setup.wValue = LE16(USB_PORT_FEAT_SUSPEND);
        req.setup.wIndex = LE16(p);
        req.virtual_address = xhci_roothub_get_address(ctrl->root_hub);

        xhci_roothub_submit_ctrl_request(ctrl->root_hub, &req);
    }

    io->req.io_Error = ERR_NO_ERROR;
    io->state = DRIVER_STATE_OPERATIONAL;
    return COMMAND_PROCESSED;
}

/*
 * enter sleep mode
 */
static inline int Do_CMD_STOP(struct USBIORequest *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->req.io_Unit;
    Kprintf("[xhci] %s: CMD_STOP - suspending USB\n", __func__);

    // TODO check if there is a controller level suspend/resume
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    int maxp = xhci_roothub_get_num_ports(ctrl->root_hub);
    for (int p = 1; p <= maxp; ++p)
    {
        struct USBIORequest req;
        _memset(&req, 0, sizeof(req));
        req.setup.bmRequestType = USB_DIR_OUT | USB_RT_PORT;
        req.setup.bRequest = USB_REQ_SET_FEATURE;
        req.setup.wValue = LE16(USB_PORT_FEAT_SUSPEND);
        req.setup.wIndex = LE16(p);
        req.virtual_address = xhci_roothub_get_address(ctrl->root_hub);

        xhci_roothub_submit_ctrl_request(ctrl->root_hub, &req);
    }

    io->req.io_Error = ERR_NO_ERROR;
    io->state = DRIVER_STATE_SUSPENDED;
    return COMMAND_PROCESSED;
}

/*
 * enter operational state
 */
static inline int Do_CMD_START(struct USBIORequest *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->req.io_Unit;
    Kprintf("[xhci] %s: CMD_START - making USB operational\n", __func__);

    // TODO should likely also resume and perhaps reset the ports
    /* Ensure port power is on for all ports */
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    int maxp = xhci_roothub_get_num_ports(ctrl->root_hub);
    for (int p = 1; p <= maxp; ++p)
    {
        struct USBIORequest req;
        _memset(&req, 0, sizeof(req));
        req.setup.bmRequestType = USB_DIR_OUT | USB_RT_PORT;
        req.setup.bRequest = USB_REQ_SET_FEATURE;
        req.setup.wValue = LE16(USB_PORT_FEAT_POWER);
        req.setup.wIndex = LE16(p);
        req.virtual_address = xhci_roothub_get_address(ctrl->root_hub);

        xhci_roothub_submit_ctrl_request(ctrl->root_hub, &req);
    }

    io->req.io_Error = ERR_NO_ERROR;
    io->state = DRIVER_STATE_OPERATIONAL;
    return COMMAND_PROCESSED;
}

/*
 * start a generic transfer
 */
static inline int Do_CMD_XFER(struct USBIORequest *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->req.io_Unit;
    struct xhci_ctrl *ctrl = unit ? unit->xhci_ctrl : NULL;

    if (io->req.io_Command == CMD_REQUEST_INTERRUPT && ctrl && io->virtual_address <= USB_MAX_ADDRESS)
    {
        struct usb_device *udev = ctrl->devices_by_virtual_address[io->virtual_address];
        if (udev)
        {
            int ep_index = xhci_ep_index_from_parts(io->endpoint, io->direction);
            struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
            if (ep_ctx && xhci_ep_has_request(ep_ctx, io))
            {
                /* Work around the Poseidon hub resume path re-submitting the
                 * same interrupt IORequest while the original request is still
                 * active on this endpoint. Treat that second send as a safe
                 * no-op only when the exact same request object is still
                 * tracked here; a legitimately re-used request after ReplyMsg()
                 * must still be accepted as a fresh transfer.
                 *
                 * This avoids queueing the same IORequest twice or replying the
                 * same message twice. It does not fix Poseidon's pending-count
                 * accounting for the duplicate send. */
                KprintfH("[xhci] %s: ignoring duplicate hub resume re-send for tracked request %08lx state=%ld flags=%lx dflags=%lx\n",
                         __func__,
                         (ULONG)io,
                         (LONG)xhci_ep_get_state(ep_ctx),
                         (ULONG)io->req.io_Flags,
                         (ULONG)io->driver_private_flags);
                return COMMAND_SCHEDULED;
            }
        }
    }

    io->driver_private_flags = 0;
    io->driver_private_dma_address = NULL;

    int result = xhci_udev_send(io);
    if (result != ERR_NO_ERROR)
    {
        io->req.io_Error = result;
        return COMMAND_PROCESSED;
    }
    return COMMAND_SCHEDULED;
}

static inline int Do_CMD_INTERNAL_ABORT(struct USBIORequest *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->req.io_Unit;
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    struct USBIORequest *orig_req = (struct USBIORequest *)io->data_buffer;

    if (ctrl && orig_req)
        xhci_td_abort_req(orig_req);

    if (unit && unit->memoryPool)
        FreeVecPooled(unit->memoryPool, io);

    return COMMAND_PROCESSED;
}

static inline int Do_CMD_REGISTER_ISO_HANDLER(struct USBIORequest *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->req.io_Unit;
    KprintfH("[xhci] %s: CMD_REGISTER_ISO_HANDLER\n", __func__);

    // TODO check if state is operational
    if (!io->data_buffer)
        goto badparams;

    struct usb_device *udev = xhci_udev_get(unit, io->virtual_address);
    if (!udev)
        goto badparams;

    int ep_index = xhci_ep_index_from_parts(io->endpoint, io->direction);

    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
    if (!ep_ctx)
        goto badparams;

    BYTE result = xhci_ep_rt_iso_add_handler(ep_ctx, io);

    io->actual_length = 0;
    io->req.io_Error = result;
    return COMMAND_PROCESSED;

badparams:
    Kprintf("Bad params\n");
    io->req.io_Error = ERR_BAD_PARAMETERS;
    return COMMAND_PROCESSED;
}

static inline int Do_CMD_UNREGISTER_ISO_HANDLER(struct USBIORequest *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->req.io_Unit;
    KprintfH("[xhci] %s: CMD_UNREGISTER_ISO_HANDLER\n", __func__);

    if (!io->data_buffer)
        goto badparams;

    struct usb_device *udev = xhci_udev_get(unit, io->virtual_address);
    if (!udev)
        goto badparams;

    int ep_index = xhci_ep_index_from_parts(io->endpoint, io->direction);

    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
    if (!ep_ctx)
        goto badparams;

    BYTE result = xhci_ep_rt_iso_rem_handler(ep_ctx, io);
    io->req.io_Error = result;
    return COMMAND_PROCESSED;

badparams:
    Kprintf("Bad parameters while removing ISO handler\n");
    io->req.io_Error = ERR_BAD_PARAMETERS;
    return COMMAND_PROCESSED;
}

static inline int Do_CMD_STARTRTISO(struct USBIORequest *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->req.io_Unit;
    KprintfH("[xhci] %s: CMD_STOP_REALTIME_ISOCHRONOUS\n", __func__);

    KprintfH("RT ISO start addr=%ld ep=%ld dir=%s frame=%lu len=%lu\n",
             (LONG)io->virtual_address,
             (LONG)(io->endpoint & 0x0F),
             (io->direction == DIRECTION_IN) ? "IN" : "OUT",
             (ULONG)io->usb_frame,
             (ULONG)io->data_buffer_length);
    struct usb_device *udev = xhci_udev_get(unit, io->virtual_address);
    if (!udev)
        goto badparams;

    int ep_index = xhci_ep_index_from_parts(io->endpoint, io->direction);
    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
    if (!ep_ctx)
        goto badparams;

    BYTE result = xhci_ep_rt_iso_start(ep_ctx);

    io->actual_length = 0;
    io->req.io_Error = result;
    return COMMAND_PROCESSED;

badparams:
    Kprintf("Bad params\n");
    io->req.io_Error = ERR_BAD_PARAMETERS;
    return COMMAND_PROCESSED;
}

static inline int Do_CMD_STOPRTISO(struct USBIORequest *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->req.io_Unit;
    KprintfH("[xhci] %s: CMD_STOP_REALTIME_ISOCHRONOUS\n", __func__);

    KprintfH("RT ISO stop requested addr=%ld ep=%ld\n",
             (LONG)io->virtual_address, (LONG)(io->endpoint & 0x0F));
    struct usb_device *udev = xhci_udev_get(unit, io->virtual_address);
    if (!udev)
        goto badparams;

    int ep_index = xhci_ep_index_from_parts(io->endpoint, io->direction);
    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
    if (!ep_ctx)
        goto badparams;

    BYTE result = xhci_ep_rt_iso_stop(ep_ctx, io);
    io->req.io_Error = result;
    if (result != ERR_NO_ERROR)
    {
        io->actual_length = 0;
        return COMMAND_PROCESSED;
    }

    return COMMAND_SCHEDULED;

badparams:
    Kprintf("Bad params\n");
    io->req.io_Error = ERR_BAD_PARAMETERS;
    return COMMAND_PROCESSED;
}

void ProcessCommand(struct USBIORequest *io)
{
    ULONG complete = COMMAND_SCHEDULED;

    /*
        Only NSCMD_DEVICEQUERY can use standard sized request. All other must be of
        size IORequest
    */
    if (io->req.io_Message.mn_Length < sizeof(struct IORequest) &&
        io->req.io_Command != NSCMD_DEVICEQUERY)
    {
        io->req.io_Error = IOERR_BADLENGTH;
        complete = COMMAND_PROCESSED;
    }
    else
    {
        io->req.io_Error = ERR_NO_ERROR;

        switch (io->req.io_Command)
        {
        case CMD_FLUSH:
            complete = Do_CMD_FLUSH(io);
            break;

        case CMD_RESET:
            complete = Do_CMD_RESET(io);
            break;

        case CMD_DEVICE_QUERY:
            complete = Do_CMD_DEVICE_QUERY(io);
            break;

        case CMD_DEVICE_RESET:
            complete = Do_CMD_DEVICE_RESET(io);
            break;

        case CMD_DEVICE_RESUME:
            complete = Do_CMD_DEVICE_RESUME(io);
            break;

        case CMD_STOP:
            complete = Do_CMD_STOP(io);
            break;

        case CMD_START:
            complete = Do_CMD_START(io);
            break;

        case CMD_REQUEST_CONTROL:
        case CMD_REQUEST_ISOCHRONOUS:
        case CMD_REQUEST_INTERRUPT:
        case CMD_REQUEST_BULK:
            complete = Do_CMD_XFER(io);
            break;

        case CMD_INTERNAL_ABORT_REQUEST:
            complete = Do_CMD_INTERNAL_ABORT(io);
            break;

        case NSCMD_DEVICEQUERY:
            complete = Do_NSCMD_DEVICEQUERY((struct IOStdReq *)io);
            break;

        case CMD_REGISTER_ISOCHRONOUS_HOOKS:
            complete = Do_CMD_REGISTER_ISO_HANDLER(io);
            break;

        case CMD_UNREGISTER_ISOCHRONOUS_HOOKS:
            complete = Do_CMD_UNREGISTER_ISO_HANDLER(io);
            break;

        case CMD_START_REALTIME_ISOCHRONOUS:
            complete = Do_CMD_STARTRTISO(io);
            break;

        case CMD_STOP_REALTIME_ISOCHRONOUS:
            complete = Do_CMD_STOPRTISO(io);
            break;

        default:
            Kprintf("[xhci] %s: Unsupported command %ld\n", __func__, (LONG)io->req.io_Command);
            io->req.io_Error = IOERR_NOCMD;
            complete = COMMAND_PROCESSED;
            break;
        }
    }

    // If command is complete and not quick, reply it now
    if (complete == COMMAND_PROCESSED && !(io->req.io_Flags & IOF_QUICK))
    {
        ReplyMsg((struct Message *)io);
    }
}
