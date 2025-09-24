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

#include <device.h>
#include <debug.h>
#include <compat.h>

#include <usb_glue.h>

static const UWORD GENET_SupportedCommands[] = {
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
    dq->nsdqr_SupportedCommands = (UWORD *)GENET_SupportedCommands;
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

    KprintfH("[xhci] %s: Flush completed\n", __func__);
    return COMMAND_PROCESSED;
}

static int Do_CMD_RESET(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    Kprintf("[xhci] %s: CMD_RESET\n", __func__);
    io->iouh_Req.io_Error = usb_glue_bus_reset(unit);
    return COMMAND_PROCESSED;
}

static inline int Do_UHCMD_QUERYDEVICE(struct IOUsbHWReq *io)
{
    Kprintf("[xhci] %s: UHCMD_QUERYDEVICE\n", __func__);

    if(!io->iouh_Data) {
        io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
        return COMMAND_PROCESSED;
    }

    struct TagItem *tag, *tagList = (struct TagItem *)io->iouh_Data;
    int filled = 0;
    Kprintf("[xhci] %s: Processing tag list at 0x%lx\n", __func__, tagList);
    while((tag = NextTagItem(&tagList))) {
        if (!tag->ti_Data) {
            Kprintf("[xhci] %s: Tag 0x%lx has NULL data ptr, skipping\n", __func__, tag->ti_Tag);
            continue;
        }
        ULONG *out = (ULONG *)tag->ti_Data;
        switch(tag->ti_Tag) {
            case UHA_State:
                // TODO: derive from internal unit state
                *out = UHSF_OPERATIONAL;
                io->iouh_State = UHSF_OPERATIONAL;
                filled++;
                break;
            case UHA_Manufacturer:
                // TODO: get from PCI vendor
                *out = (ULONG)(APTR)"Generic";
                filled++;
                break;
            case UHA_ProductName:
                // TODO: get from PCI device
                *out = (ULONG)(APTR)"xHCI USB Controller";
                filled++;
                break;
            case UHA_Version:
                // TODO: get from PCI revision
                *out = 0x01;
                filled++;
                break;
            case UHA_Revision:
                // TODO: get from PCI revision
                *out = 0x01;
                filled++;
                break;
            case UHA_Description:
                *out = (ULONG)(APTR)"Generic xHCI USB Controller Driver";
                filled++;
                break;
            case UHA_Copyright:
                *out = (ULONG)(APTR)"GPLv2 or later";
                filled++;
                break;
            case UHA_DriverVersion:
                // BCD of IO request structure version: support V2
                *out = 0x0200;
                filled++;
                break;
            default:
                // Unknown tag: leave untouched
                break;
        }
        Kprintf("[xhci] %s: Processed tag 0x%lx\n", __func__, tag->ti_Tag);
    }

    Kprintf("[xhci] %s: Completed UHCMD_QUERYDEVICE\n", __func__);
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
    Kprintf("[xhci] %s: UHCMD_CONTROLXFER\n", __func__);

    return usb_glue_ctrl(unit, io);
}

/*
 * start an isochronous transfer
 */
static inline int Do_UHCMD_ISOXFER(struct IOUsbHWReq *io)
{
    // struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    Kprintf("[xhci] %s: UHCMD_ISOXFER\n", __func__);

    io->iouh_Req.io_Error = IOERR_NOCMD; /* not supported yet */
    return COMMAND_PROCESSED;
}

/*
 * start an interrupt transfer
 */
static inline int Do_UHCMD_INTXFER(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    Kprintf("[xhci] %s: UHCMD_INTXFER\n", __func__);

    return usb_glue_int(unit, io);
}

/*
 * start a bulk transfer
 */
static inline int Do_UHCMD_BULKXFER(struct IOUsbHWReq *io)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    Kprintf("[xhci] %s: UHCMD_BULKXFER\n", __func__);

    return usb_glue_bulk(unit, io);
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

        default:
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
