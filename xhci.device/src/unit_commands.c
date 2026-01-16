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
    Kprintf("[xhci] %s: CMD_FLUSH\n", __func__);

    struct IOUsbHWReq *req;
    /* Flush and cancel all requests */
    while ((req = (struct IOUsbHWReq *)GetMsg(&unit->unit.unit_MsgPort)))
    {
        req->iouh_Req.io_Error = IOERR_ABORTED;
        ReplyMsg((struct Message *)req);
    }

    /* go through all devices and endpoints and flush their queues */
    usb_glue_flush_queues(unit);

    KprintfH("[xhci] %s: Flush completed\n", __func__);
    return COMMAND_PROCESSED;
}

static int Do_CMD_RESET(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    Kprintf("[xhci] %s: CMD_RESET\n", __func__);
    //TODO should reset entire controller...
    io->iouh_Req.io_Error = usb_glue_bus_reset(unit);
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
            KprintfH("[xhci] %s: Tag 0x%lx has NULL data ptr, skipping\n", __func__, tag->ti_Tag);
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
            *out = (ULONG)(APTR) vendor_str;
            filled++;
            break;
        case UHA_ProductName:
            uword_to_hex(unit->xhci_ctrl->pci_dev->device, (UBYTE *)device_str);
            *out = (ULONG)(APTR) device_str;
            filled++;
            break;
        case UHA_Version:
            *out = (DEVICE_VERSION << 8) | (DEVICE_REVISION & 0xFF);
            filled++;
            break;
        case UHA_Revision:
        {
            UBYTE rev;
            dm_pci_read_config8(unit->xhci_ctrl->pci_dev, PCI_REVISION_ID, &rev);
            *out = rev;
            filled++;
            break;
        }
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
            *out = UHCF_USB20 | UHCF_ISO | UHCF_RT_ISO | UHCF_USB30;// | UHCF_QUICKIO;
            filled++;
            break;
        default:
            // Unknown tag: leave untouched
            Kprintf("[xhci] %s: Unknown tag 0x%lx, skipping\n", __func__, tag->ti_Tag);
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
    io->iouh_Req.io_Error = usb_glue_bus_reset(unit);
    io->iouh_State = (io->iouh_Req.io_Error == UHIOERR_NO_ERROR) ? UHSF_RESET : 0;

    return COMMAND_PROCESSED;
}

/*
 * resume from sleep mode
 */
static inline int Do_UHCMD_USBRESUME(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    Kprintf("[xhci] %s: UHCMD_USBRESUME\n", __func__);

    io->iouh_Req.io_Error = usb_glue_bus_resume(unit);
    io->iouh_State = (io->iouh_Req.io_Error == UHIOERR_NO_ERROR) ? UHSF_OPERATIONAL : 0;
    return COMMAND_PROCESSED;
}

/*
 * enter sleep mode
 */
static inline int Do_UHCMD_USBSUSPEND(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    Kprintf("[xhci] %s: UHCMD_USBSUSPEND\n", __func__);

    io->iouh_Req.io_Error = usb_glue_bus_suspend(unit);
    io->iouh_State = (io->iouh_Req.io_Error == UHIOERR_NO_ERROR) ? UHSF_SUSPENDED : 0;
    return COMMAND_PROCESSED;
}

/*
 * enter operational state
 */
static inline int Do_UHCMD_USBOPER(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    Kprintf("[xhci] %s: UHCMD_USBOPER\n", __func__);

    io->iouh_Req.io_Error = usb_glue_bus_oper(unit);
    io->iouh_State = (io->iouh_Req.io_Error == UHIOERR_NO_ERROR) ? UHSF_OPERATIONAL : 0;
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
    Kprintf("[xhci] %s: UHCMD_ISOXFER\n", __func__);

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
    struct XHCIUnit *unit = (struct XHCIUnit *) io->iouh_Req.io_Unit;
    KprintfH("[xhci] %s: UHCMD_ADDISOHANDLER\n", __func__);   

    return usb_glue_add_iso_handler(unit, io);
}

static inline int Do_UHCMD_REMISOHANDLER(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *) io->iouh_Req.io_Unit;
    KprintfH("[xhci] %s: UHCMD_REMISOHANDLER\n", __func__);

    return usb_glue_rem_iso_handler(unit, io);
}

static inline int Do_UHCMD_STARTRTISO(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *) io->iouh_Req.io_Unit;
    KprintfH("[xhci] %s: UHCMD_STARTRTISO\n", __func__);

    return usb_glue_start_rt_iso(unit, io);
}

static inline int Do_UHCMD_STOPRTISO(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *) io->iouh_Req.io_Unit;
    KprintfH("[xhci] %s: UHCMD_STOPRTISO\n", __func__);
    return usb_glue_stop_rt_iso(unit, io);
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
