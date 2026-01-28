// SPDX-License-Identifier: GPL-2.0+
#ifdef __INTELLISENSE__
#include <clib/timer_protos.h>
#include <clib/exec_protos.h>
#include <clib/utility_protos.h>
#else
#include <proto/timer.h>
#include <proto/exec.h>
#include <proto/utility.h>
#endif

#include <utility/tagitem.h>

#include <devices/newstyle.h>
#include <devices/usbhardware.h>

#include <config.h>
#include <device.h>
#include <debug.h>
#include <compat.h>
#include <pci.h>
#include <xhci/xhci.h>
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-usb.h>

#include <usb_glue.h>

static const UWORD SupportedCommands[] = {
    CMD_FLUSH,
    CMD_RESET,
    UHCMD_QUERYDEVICE,
    UHCMD_USBRESET,
    UHCMD_USBRESUME,
    UHCMD_USBSUSPEND,
    UHCMD_USBOPER,
    UHCMD_CONTROLXFER,
    UHCMD_ISOXFER,
    UHCMD_INTXFER,
    UHCMD_BULKXFER,

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

/*
 * Abort all UHCMD_CONTROLXFER, UHCMD_ISOXFER, UHCMD_INTXFER and UHCMD_BULKXFER requests in progress or queued
 */
static int Do_CMD_FLUSH(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    KprintfH("[xhci] %s: CMD_FLUSH\n", __func__);

    struct IOUsbHWReq *req;
    /* Flush and cancel all requests */
    while ((req = (struct IOUsbHWReq *)GetMsg(&unit->unit.unit_MsgPort)))
    {
        req->iouh_Req.io_Error = IOERR_ABORTED;
        ReplyMsg((struct Message *)req);
    }

    /* go through all devices and endpoints and flush their queues */
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;

    for (unsigned int addr = 0; addr <= USB_MAX_ADDRESS; ++addr)
    {
        struct usb_device *udev = ctrl->devices_by_poseidon_address[addr];
        if (!udev || addr == xhci_roothub_get_address(ctrl->root_hub))
            continue;

        for (unsigned int ep = 0; ep < USB_MAXENDPOINTS; ++ep)
        {
            struct ep_context *ep_ctx = xhci_ep_get_context(udev, ep);
            xhci_ep_flush(ep_ctx, IOERR_ABORTED);

            u32 ep_out_index = xhci_ep_index_from_parts(ep, UHDIR_OUT);
            u32 ep_in_index = xhci_ep_index_from_parts(ep, UHDIR_IN);
            // TODO which one really exists?

            xhci_stop_endpoint(udev, ep_out_index);
            xhci_stop_endpoint(udev, ep_in_index);
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

static char vendor_str[5];
static char device_str[5];

static inline int Do_UHCMD_QUERYDEVICE(struct IOUsbHWReq *io)
{
    KprintfH("[xhci] %s: UHCMD_QUERYDEVICE\n", __func__);

    if (!io->iouh_Data)
    {
        io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
        return COMMAND_PROCESSED;
    }

    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;

    struct TagItem *tag, *tagList = (struct TagItem *)io->iouh_Data;
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
        case UHA_State:
            // TODO: derive from internal unit state
            *out = UHSF_OPERATIONAL;
            io->iouh_State = UHSF_OPERATIONAL;
            filled++;
            break;
        case UHA_Manufacturer:
            uword_to_hex(unit->xhci_ctrl->pci_dev->vendor, (UBYTE *)vendor_str);
            *out = (ULONG)(APTR)vendor_str;
            filled++;
            break;
        case UHA_ProductName:
            uword_to_hex(unit->xhci_ctrl->pci_dev->device, (UBYTE *)device_str);
            *out = (ULONG)(APTR)device_str;
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
            *out = UHCF_USB20 | UHCF_ISO | UHCF_RT_ISO | UHCF_USB30; // | UHCF_QUICKIO;
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
    io->iouh_Req.io_Error = UHIOERR_NO_ERROR;
    io->iouh_Actual = filled;
    return COMMAND_PROCESSED;
}

/*
 * reset USB bus
 */
static inline int Do_UHCMD_USBRESET(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    Kprintf("[xhci] %s: UHCMD_USBRESET\n", __func__);

    /* Issue SET_FEATURE(RESET) on all root hub ports */
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    int maxp = xhci_roothub_get_num_ports(ctrl->root_hub);
    for (int p = 1; p <= maxp; ++p)
    {
        struct IOUsbHWReq req;
        _memset(&req, 0, sizeof(req));
        req.iouh_SetupData.bmRequestType = USB_DIR_OUT | USB_RT_PORT; /* class=hub, recipient=other */
        req.iouh_SetupData.bRequest = USB_REQ_SET_FEATURE;
        req.iouh_SetupData.wValue = LE16(USB_PORT_FEAT_RESET);
        req.iouh_SetupData.wIndex = LE16(p);
        req.iouh_SetupData.wLength = LE16(0);
        req.iouh_DevAddr = xhci_roothub_get_address(ctrl->root_hub);

        xhci_roothub_submit_ctrl_request(ctrl->root_hub, &req);
    }

    io->iouh_Req.io_Error = UHIOERR_NO_ERROR;
    io->iouh_State = (io->iouh_Req.io_Error == UHIOERR_NO_ERROR) ? UHSF_RESET : 0;

    return COMMAND_PROCESSED;
}

static int Do_CMD_RESET(struct IOUsbHWReq *io)
{
    // struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    Kprintf("[xhci] %s: CMD_RESET\n", __func__);
    // TODO should reset entire controller...
    return Do_UHCMD_USBRESET(io);
}

/*
 * resume from sleep mode
 */
static inline int Do_UHCMD_USBRESUME(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    Kprintf("[xhci] %s: UHCMD_USBRESUME\n", __func__);

    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    int maxp = xhci_roothub_get_num_ports(ctrl->root_hub);
    for (int p = 1; p <= maxp; ++p)
    {
        struct IOUsbHWReq req;
        _memset(&req, 0, sizeof(req));
        req.iouh_SetupData.bmRequestType = USB_DIR_OUT | USB_RT_PORT;
        req.iouh_SetupData.bRequest = USB_REQ_CLEAR_FEATURE;
        req.iouh_SetupData.wValue = LE16(USB_PORT_FEAT_SUSPEND);
        req.iouh_SetupData.wIndex = LE16(p);
        req.iouh_DevAddr = xhci_roothub_get_address(ctrl->root_hub);

        xhci_roothub_submit_ctrl_request(ctrl->root_hub, &req);
    }

    io->iouh_Req.io_Error = UHIOERR_NO_ERROR;
    io->iouh_State = UHSF_OPERATIONAL;
    return COMMAND_PROCESSED;
}

/*
 * enter sleep mode
 */
static inline int Do_UHCMD_USBSUSPEND(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    Kprintf("[xhci] %s: UHCMD_USBSUSPEND\n", __func__);

    // TODO check if there is a controller level suspend/resume
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    int maxp = xhci_roothub_get_num_ports(ctrl->root_hub);
    for (int p = 1; p <= maxp; ++p)
    {
        struct IOUsbHWReq req;
        _memset(&req, 0, sizeof(req));
        req.iouh_SetupData.bmRequestType = USB_DIR_OUT | USB_RT_PORT;
        req.iouh_SetupData.bRequest = USB_REQ_SET_FEATURE;
        req.iouh_SetupData.wValue = LE16(USB_PORT_FEAT_SUSPEND);
        req.iouh_SetupData.wIndex = LE16(p);
        req.iouh_DevAddr = xhci_roothub_get_address(ctrl->root_hub);

        xhci_roothub_submit_ctrl_request(ctrl->root_hub, &req);
    }

    io->iouh_Req.io_Error = UHIOERR_NO_ERROR;
    io->iouh_State = UHSF_SUSPENDED;
    return COMMAND_PROCESSED;
}

/*
 * enter operational state
 */
static inline int Do_UHCMD_USBOPER(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    Kprintf("[xhci] %s: UHCMD_USBOPER\n", __func__);

    // TODO should likely also resume and perhaps reset the ports
    /* Ensure port power is on for all ports */
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    int maxp = xhci_roothub_get_num_ports(ctrl->root_hub);
    for (int p = 1; p <= maxp; ++p)
    {
        struct IOUsbHWReq req;
        _memset(&req, 0, sizeof(req));
        req.iouh_SetupData.bmRequestType = USB_DIR_OUT | USB_RT_PORT;
        req.iouh_SetupData.bRequest = USB_REQ_SET_FEATURE;
        req.iouh_SetupData.wValue = LE16(USB_PORT_FEAT_POWER);
        req.iouh_SetupData.wIndex = LE16(p);
        req.iouh_DevAddr = xhci_roothub_get_address(ctrl->root_hub);

        xhci_roothub_submit_ctrl_request(ctrl->root_hub, &req);
    }

    io->iouh_Req.io_Error = UHIOERR_NO_ERROR;
    io->iouh_State = UHSF_OPERATIONAL;
    return COMMAND_PROCESSED;
}

/*
 * start a control transfer
 */
static inline int Do_UHCMD_CONTROLXFER(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    KprintfH("[xhci] %s: UHCMD_CONTROLXFER\n", __func__);

    return usb_glue_ctrl(unit, io);
}

/*
 * start an isochronous transfer
 */
static inline int Do_UHCMD_ISOXFER(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    KprintfH("[xhci] %s: UHCMD_ISOXFER\n", __func__);

    return usb_glue_iso(unit, io);
}

/*
 * start an interrupt transfer
 */
static inline int Do_UHCMD_INTXFER(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    KprintfH("[xhci] %s: UHCMD_INTXFER\n", __func__);

    return usb_glue_int(unit, io);
}

/*
 * start a bulk transfer
 */
static inline int Do_UHCMD_BULKXFER(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    KprintfH("[xhci] %s: UHCMD_BULKXFER\n", __func__);

    return usb_glue_bulk(unit, io);
}

static inline int Do_UHCMD_ADDISOHANDLER(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    KprintfH("[xhci] %s: UHCMD_ADDISOHANDLER\n", __func__);

    // TODO check if UHSF_OPERATIONAL
    if (!io->iouh_Data)
        goto badparams;

    struct usb_device *udev = get_or_init_udev(unit, io->iouh_DevAddr);
    if (!udev)
        goto badparams;

    unsigned int endpoint = io->iouh_Endpoint & 0x0F;
    if (endpoint >= USB_MAXENDPOINTS)
        goto badparams;

    struct ep_context *ep_ctx = xhci_ep_get_context(udev, endpoint);
    BYTE result = xhci_ep_rt_iso_add_handler(ep_ctx, io);

    io->iouh_Actual = 0;
    io->iouh_Req.io_Error = result;
    return COMMAND_PROCESSED;

badparams:
    Kprintf("Bad params\n");
    io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
    return COMMAND_PROCESSED;
}

static inline int Do_UHCMD_REMISOHANDLER(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    KprintfH("[xhci] %s: UHCMD_REMISOHANDLER\n", __func__);

    if (!io->iouh_Data)
        goto badparams;

    struct usb_device *udev = get_or_init_udev(unit, io->iouh_DevAddr);
    if (!udev)
        goto badparams;

    unsigned int endpoint = io->iouh_Endpoint & 0x0F;
    if (endpoint >= USB_MAXENDPOINTS)
        goto badparams;

    struct ep_context *ep_ctx = xhci_ep_get_context(udev, endpoint);
    BYTE result = xhci_ep_rt_iso_rem_handler(ep_ctx, io);
    io->iouh_Req.io_Error = result;
    return COMMAND_PROCESSED;

badparams:
    Kprintf("Bad parameters while removing ISO handler\n");
    io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
    return COMMAND_PROCESSED;
}

static inline int Do_UHCMD_STARTRTISO(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    KprintfH("[xhci] %s: UHCMD_STARTRTISO\n", __func__);

    KprintfH("RT ISO start addr=%ld ep=%ld dir=%s frame=%lu interval=%lu len=%lu\n",
             (LONG)io->iouh_DevAddr,
             (LONG)(io->iouh_Endpoint & 0x0F),
             (io->iouh_Dir == UHDIR_IN) ? "IN" : "OUT",
             (ULONG)io->iouh_Frame,
             (ULONG)io->iouh_Interval,
             (ULONG)io->iouh_Length);
    struct usb_device *udev = get_or_init_udev(unit, io->iouh_DevAddr);
    if (!udev)
    {
        Kprintf("bad params\n");
        io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
        return COMMAND_PROCESSED;
    }

    int endpoint = io->iouh_Endpoint & 0x0F;
    struct ep_context *ep_ctx = xhci_ep_get_context(udev, endpoint);
    BYTE result = xhci_ep_rt_iso_start(ep_ctx);

    io->iouh_Actual = 0;
    io->iouh_Req.io_Error = result;
    return COMMAND_PROCESSED;
}

static inline int Do_UHCMD_STOPRTISO(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    KprintfH("[xhci] %s: UHCMD_STOPRTISO\n", __func__);

    KprintfH("RT ISO stop requested addr=%ld ep=%ld\n",
             (LONG)io->iouh_DevAddr, (LONG)(io->iouh_Endpoint & 0x0F));
    struct usb_device *udev = get_or_init_udev(unit, io->iouh_DevAddr);
    if (!udev)
    {
        Kprintf("bad params\n");
        io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
        return COMMAND_PROCESSED;
    }

    int endpoint = io->iouh_Endpoint & 0x0F;
    struct ep_context *ep_ctx = xhci_ep_get_context(udev, endpoint);
    BYTE result = xhci_ep_rt_iso_stop(ep_ctx, io);
    io->iouh_Req.io_Error = result;
    if (result != UHIOERR_NO_ERROR)
    {
        io->iouh_Actual = 0;
        return COMMAND_PROCESSED;
    }

    return COMMAND_SCHEDULED;
}

void ProcessCommand(struct IOUsbHWReq *io)
{
    ULONG complete = COMMAND_SCHEDULED;

    /*
        Only NSCMD_DEVICEQUERY can use standard sized request. All other must be of
        size IORequest
    */
    if (io->iouh_Req.io_Message.mn_Length < sizeof(struct IORequest) &&
        io->iouh_Req.io_Command != NSCMD_DEVICEQUERY)
    {
        io->iouh_Req.io_Error = IOERR_BADLENGTH;
        complete = COMMAND_PROCESSED;
    }
    else
    {
        io->iouh_Req.io_Error = UHIOERR_NO_ERROR;

        switch (io->iouh_Req.io_Command)
        {
        case CMD_FLUSH:
            complete = Do_CMD_FLUSH(io);
            break;

        case CMD_RESET:
            complete = Do_CMD_RESET(io);
            break;

        case UHCMD_QUERYDEVICE:
            complete = Do_UHCMD_QUERYDEVICE(io);
            break;

        case UHCMD_USBRESET:
            complete = Do_UHCMD_USBRESET(io);
            break;

        case UHCMD_USBRESUME:
            complete = Do_UHCMD_USBRESUME(io);
            break;

        case UHCMD_USBSUSPEND:
            complete = Do_UHCMD_USBSUSPEND(io);
            break;

        case UHCMD_USBOPER:
            complete = Do_UHCMD_USBOPER(io);
            break;

        case UHCMD_CONTROLXFER:
            complete = Do_UHCMD_CONTROLXFER(io);
            break;

        case UHCMD_ISOXFER:
            complete = Do_UHCMD_ISOXFER(io);
            break;

        case UHCMD_INTXFER:
            complete = Do_UHCMD_INTXFER(io);
            break;

        case UHCMD_BULKXFER:
            complete = Do_UHCMD_BULKXFER(io);
            break;

        case NSCMD_DEVICEQUERY:
            complete = Do_NSCMD_DEVICEQUERY((struct IOStdReq *)io);
            break;

        case UHCMD_ADDISOHANDLER:
            complete = Do_UHCMD_ADDISOHANDLER(io);
            break;

        case UHCMD_REMISOHANDLER:
            complete = Do_UHCMD_REMISOHANDLER(io);
            break;

        case UHCMD_STARTRTISO:
            complete = Do_UHCMD_STARTRTISO(io);
            break;

        case UHCMD_STOPRTISO:
            complete = Do_UHCMD_STOPRTISO(io);
            break;

        default:
            Kprintf("[xhci] %s: Unsupported command %ld\n", __func__, (LONG)io->iouh_Req.io_Command);
            io->iouh_Req.io_Error = IOERR_NOCMD;
            complete = COMMAND_PROCESSED;
            break;
        }
    }

    // If command is complete and not quick, reply it now
    if (complete == COMMAND_PROCESSED && !(io->iouh_Req.io_Flags & IOF_QUICK))
    {
        ReplyMsg((struct Message *)io);
    }
}
