// SPDX-License-Identifier: GPL-2.0+
#include <proto/exec.h>

#include <devices/usb.h>
#include <devices/usbhardware.h>

#include <usb.h>
#include <xhci.h>

#include <device.h>
#include <debug.h>
#include <compat.h>

#include "usb_glue.h"

/* Minimal per-device context bound to a devaddr */
struct glue_devctx {
    struct usb_device udev;         /* driver-visible device */
    UBYTE used;
    UBYTE in_flight[USB_MAXENDPOINTS][2]; /* [ep][dir] boolean */
};

/* Keep a small table for devaddr 0..127 */
struct glue_state {
    struct glue_devctx devs[128];
};

static inline struct glue_state *gst(struct XHCIUnit *unit) {
    return (struct glue_state *)unit->xhci_ctrl->privptr; /* reuse privptr */
}

static struct usb_device *get_or_init_udev(struct XHCIUnit *unit, UWORD devaddr)
{
    if (devaddr > 127)
        return NULL;
    struct glue_state *st = gst(unit);
    struct glue_devctx *dc = &st->devs[devaddr];
    if (!dc->used) {
        _memset(dc, 0, sizeof(*dc));
        dc->used = 1;
        dc->udev.devnum = devaddr;
        dc->udev.controller = unit->xhci_ctrl;
        /* Default EP0 packet size for new address (8) */
        dc->udev.maxpacketsize = PACKET_SIZE_8;
        dc->udev.epmaxpacketin[0] = 8;
        dc->udev.epmaxpacketout[0] = 8;
        /* Default toggles cleared */
    }
    return &dc->udev;
}

static inline int map_status_to_uhio(int usb_status)
{
    switch (usb_status) {
    case 0:
        return UHIOERR_NO_ERROR;
    case USB_ST_STALLED:
        return UHIOERR_STALL;
    case USB_ST_NAK_REC:
        return UHIOERR_NAKTIMEOUT; /* best fit */
    case USB_ST_CRC_ERR:
        return UHIOERR_CRCERROR;
    case USB_ST_BABBLE_DET:
        return UHIOERR_OVERFLOW;
    case USB_ST_BIT_ERR:
        return UHIOERR_HOSTERROR;
    default:
        return UHIOERR_HOSTERROR;
    }
}

int usb_glue_init(struct XHCIUnit *unit)
{
    if (!unit || !unit->xhci_ctrl)
        return UHIOERR_BADPARAMS;
    if (!unit->xhci_ctrl->privptr) {
        struct glue_state *st = AllocMem(sizeof(struct glue_state), MEMF_PUBLIC|MEMF_CLEAR);
        if (!st)
            return UHIOERR_OUTOFMEMORY;
        unit->xhci_ctrl->privptr = st;
    }
    return UHIOERR_NO_ERROR;
}

void usb_glue_shutdown(struct XHCIUnit *unit)
{
    if (unit && unit->xhci_ctrl && unit->xhci_ctrl->privptr) {
        FreeMem(unit->xhci_ctrl->privptr, sizeof(struct glue_state));
        unit->xhci_ctrl->privptr = NULL;
    }
}

/* Per-endpoint serialization (simple guard). dir: 0=OUT,1=IN */
static BOOL ep_try_lock(struct glue_devctx *dc, UWORD ep, UWORD dir)
{
    if (ep >= USB_MAXENDPOINTS) return FALSE;
    if (dc->in_flight[ep][dir]) return FALSE;
    dc->in_flight[ep][dir] = 1;
    return TRUE;
}

static void ep_unlock(struct glue_devctx *dc, UWORD ep, UWORD dir)
{
    if (ep < USB_MAXENDPOINTS) dc->in_flight[ep][dir] = 0;
}

int usb_glue_ctrl(struct XHCIUnit *unit, struct IOUsbHWReq *io)
{
    struct usb_device *udev = get_or_init_udev(unit, io->iouh_DevAddr);
    if (!udev) { io->iouh_Req.io_Error = UHIOERR_BADPARAMS; return COMMAND_PROCESSED; }

    struct glue_devctx *dc = &gst(unit)->devs[io->iouh_DevAddr];
    if (!ep_try_lock(dc, 0, 1)) { io->iouh_Req.io_Error = UHIOERR_HOSTERROR; return COMMAND_PROCESSED; }

    /* Update EP0 MPS hint from IO if provided */
    if (io->iouh_MaxPktSize) {
        int mps = (int)io->iouh_MaxPktSize;
        if (mps == 8) udev->maxpacketsize = PACKET_SIZE_8;
        else if (mps == 16) udev->maxpacketsize = PACKET_SIZE_16;
        else if (mps == 32) udev->maxpacketsize = PACKET_SIZE_32;
        else udev->maxpacketsize = PACKET_SIZE_64;

        /* Cache explicit EP0 max packet for both directions */
        udev->epmaxpacketin[0] = mps;
        udev->epmaxpacketout[0] = mps;
    }

    unsigned long pipe = (io->iouh_SetupData.bmRequestType & URTF_IN)
               ? usb_rcvctrlpipe(udev, 0)
               : usb_sndctrlpipe(udev, 0);

    Kprintf("usb_glue_ctrl: dev=%ld addr=%ld pipe=%lx bmReqType=%02lx bReq=%02lx wValue=%04lx wIndex=%04lx wLength=%04lx flags=%lx timeout=%ld\n",
        (ULONG)udev, (ULONG)udev->devnum, pipe,
        (ULONG)io->iouh_SetupData.bmRequestType,
        (ULONG)io->iouh_SetupData.bRequest,
        (ULONG)io->iouh_SetupData.wValue,
        (ULONG)io->iouh_SetupData.wIndex,
        (ULONG)io->iouh_SetupData.wLength,
        (ULONG)io->iouh_Flags,
        (ULONG)io->iouh_NakTimeout);

    struct devrequest req;
    req.requesttype = io->iouh_SetupData.bmRequestType;
    req.request = io->iouh_SetupData.bRequest;
    req.value = io->iouh_SetupData.wValue;
    req.index = io->iouh_SetupData.wIndex;
    req.length = io->iouh_SetupData.wLength;

    void *buf = io->iouh_Data;
    int len = (int)io->iouh_Length;

    int ret;
    if ((io->iouh_Flags & UHFF_NAKTIMEOUT) && io->iouh_NakTimeout)
        ret = submit_control_msg_tmo(udev, pipe, buf, len, &req, (unsigned int)io->iouh_NakTimeout);
    else
        ret = submit_control_msg(udev, pipe, buf, len, &req);
    int uherr = map_status_to_uhio(udev->status);
    io->iouh_Actual = (ULONG)udev->act_len;
    io->iouh_Req.io_Error = (ret == 0) ? UHIOERR_NO_ERROR : uherr;

    /* If this was a successful GET_DESCRIPTOR(STRING) IN transfer and we got
     * an odd number of bytes, clamp to even to avoid a dangling half UTF-16
     * code unit, which Poseidon may display as a trailing '?'.
     */
    if (ret == 0 && (req.request == USB_REQ_GET_DESCRIPTOR) &&
        (req.requesttype & USB_DIR_IN) && (((LE16(req.value)) >> 8) == USB_DT_STRING)) {
        if ((udev->act_len & 1) != 0) {
            Kprintf("usb_glue_ctrl: clamping odd string length %ld to %ld\n",
                    (LONG)udev->act_len, (LONG)(udev->act_len - 1));
            udev->act_len -= 1;
            io->iouh_Actual = (ULONG)udev->act_len;
        }
    }

    Kprintf("usb_glue_ctrl: ret=%ld uherr=%ld udev->status=%ld act_len=%ld\n",
            (LONG)ret, (LONG)io->iouh_Req.io_Error, (LONG)udev->status, (LONG)udev->act_len);

    /* If this was a successful standard SET_ADDRESS, migrate the glue context
     * from old devaddr to the new one so subsequent transfers use the same slot.
     */
    if (ret == 0 &&
        (req.request == USB_REQ_SET_ADDRESS) &&
        ((req.requesttype & USB_TYPE_MASK) == USB_TYPE_STANDARD)) {
        UWORD old_addr = io->iouh_DevAddr & 0x7F;
        UWORD new_addr = (UWORD)(LE16(req.value) & 0x7F);
        if (new_addr != old_addr) {
            /* Unlock before moving so we don't copy a locked flag */
            ep_unlock(dc, 0, 1);

            struct glue_state *st = gst(unit);
            struct glue_devctx *from = &st->devs[old_addr];
            struct glue_devctx *to = &st->devs[new_addr];

            /* If destination was in use (shouldn't for enumeration), reset it */
            if (to->used) {
                _memset(to, 0, sizeof(*to));
            }

            /* Move the whole context, then fix up address fields */
            *to = *from;
            to->used = 1;
            to->udev.devnum = new_addr;
            /* Clear any in-flight guards after address change */
            _memset(to->in_flight, 0, sizeof(to->in_flight));

            /* Clear the old slot */
            _memset(from, 0, sizeof(*from));

            return COMMAND_PROCESSED;
        }
    }

    ep_unlock(dc, 0, 1);
    return COMMAND_PROCESSED;
}

int usb_glue_bulk(struct XHCIUnit *unit, struct IOUsbHWReq *io)
{
    struct usb_device *udev = get_or_init_udev(unit, io->iouh_DevAddr);
    if (!udev) { io->iouh_Req.io_Error = UHIOERR_BADPARAMS; return COMMAND_PROCESSED; }
    UWORD ep = io->iouh_Endpoint & 0x0F;
    UWORD dir = (io->iouh_Dir == UHDIR_IN) ? 1 : 0;
    struct glue_devctx *dc = &gst(unit)->devs[io->iouh_DevAddr];
    if (!ep_try_lock(dc, ep, dir)) { io->iouh_Req.io_Error = UHIOERR_HOSTERROR; return COMMAND_PROCESSED; }

    Kprintf("usb_glue_bulk: dev=%ld addr=%ld ep=%ld dir=%s len=%ld flags=%lx tmo=%ld\n",
        (ULONG)udev, (ULONG)udev->devnum, (LONG)ep, dir ? "IN" : "OUT",
        (LONG)io->iouh_Length, (ULONG)io->iouh_Flags, (LONG)io->iouh_NakTimeout);

    unsigned long pipe = (io->iouh_Dir == UHDIR_IN) ? usb_rcvbulkpipe(udev, ep)
                                                    : usb_sndbulkpipe(udev, ep);
    void *buf = io->iouh_Data;
    int len = (int)io->iouh_Length;
    int ret;
    int uherr;

    if (io->iouh_MaxPktSize) {
        int mps = (int)io->iouh_MaxPktSize;
        if (dir)
            udev->epmaxpacketin[ep] = mps;
        else
            udev->epmaxpacketout[ep] = mps;
    }

    if ((io->iouh_Flags & UHFF_NAKTIMEOUT) && io->iouh_NakTimeout)
        ret = xhci_bulk_tx_tmo(udev, pipe, len, buf, (unsigned int)io->iouh_NakTimeout);
    else
        ret = submit_bulk_msg(udev, pipe, buf, len);
    uherr = map_status_to_uhio(udev->status);
    io->iouh_Actual = (ULONG)udev->act_len;
    io->iouh_Req.io_Error = (ret == 0) ? UHIOERR_NO_ERROR : uherr;
    ep_unlock(dc, ep, dir);
    return COMMAND_PROCESSED;
}

int usb_glue_int(struct XHCIUnit *unit, struct IOUsbHWReq *io)
{
    struct usb_device *udev = get_or_init_udev(unit, io->iouh_DevAddr);
    if (!udev) { io->iouh_Req.io_Error = UHIOERR_BADPARAMS; return COMMAND_PROCESSED; }
    UWORD ep = io->iouh_Endpoint & 0x0F;
    UWORD dir = (io->iouh_Dir == UHDIR_IN) ? 1 : 0;
    struct glue_devctx *dc = &gst(unit)->devs[io->iouh_DevAddr];
    if (!ep_try_lock(dc, ep, dir)) { io->iouh_Req.io_Error = UHIOERR_HOSTERROR; return COMMAND_PROCESSED; }

    Kprintf("usb_glue_int: dev=%ld addr=%ld ep=%ld dir=%s len=%ld interval=%ld flags=%lx tmo=%ld\n",
        (ULONG)udev, (ULONG)udev->devnum, (LONG)ep, dir ? "IN" : "OUT",
        (LONG)io->iouh_Length, (LONG)io->iouh_Interval,
        (ULONG)io->iouh_Flags, (LONG)io->iouh_NakTimeout);

    /* Root hub doesn't have a real interrupt endpoint ring here; synthesize success */
    if (udev->devnum == unit->xhci_ctrl->rootdev) {
        io->iouh_Actual = 0;
        io->iouh_Req.io_Error = UHIOERR_NO_ERROR;
        ep_unlock(dc, ep, dir);
        return COMMAND_PROCESSED;
    }

    unsigned long pipe = (io->iouh_Dir == UHDIR_IN) ? usb_rcvintpipe(udev, ep)
                                                    : usb_sndintpipe(udev, ep);
    void *buf = io->iouh_Data;
    int len = (int)io->iouh_Length;
    int interval = (int)io->iouh_Interval; /* TODO: microframe vs ms */
    bool nonblock = false; /* synchronous for now */
    int ret;

    if (io->iouh_MaxPktSize) {
        int mps = (int)io->iouh_MaxPktSize;
        if (dir)
            udev->epmaxpacketin[ep] = mps;
        else
            udev->epmaxpacketout[ep] = mps;
    }
    if ((io->iouh_Flags & UHFF_NAKTIMEOUT) && io->iouh_NakTimeout)
        ret = xhci_bulk_tx_tmo(udev, pipe, len, buf, (unsigned int)io->iouh_NakTimeout);
    else
        ret = submit_int_msg(udev, pipe, buf, len, interval, nonblock);
    int uherr = map_status_to_uhio(udev->status);
    io->iouh_Actual = (ULONG)udev->act_len;
    io->iouh_Req.io_Error = (ret == 0) ? UHIOERR_NO_ERROR : uherr;
    ep_unlock(dc, ep, dir);
    return COMMAND_PROCESSED;
}

static inline int root_hub_max_ports(struct xhci_ctrl *ctrl)
{
    return HCS_MAX_PORTS(xhci_readl(&ctrl->hccr->cr_hcsparams1));
}

int usb_glue_bus_reset(struct XHCIUnit *unit)
{
    /* Issue SET_FEATURE(RESET) on all root hub ports */
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    struct usb_device *udev = get_or_init_udev(unit, ctrl->rootdev);
    if (!udev) return UHIOERR_HOSTERROR;
    int maxp = root_hub_max_ports(ctrl);
    unsigned long pipe = usb_sndctrlpipe(udev, 0);
    for (int p = 1; p <= maxp; ++p) {
        struct devrequest req;
        _memset(&req, 0, sizeof(req));
        req.requesttype = USB_DIR_OUT | USB_RT_PORT; /* class=hub, recipient=other */
        req.request = USB_REQ_SET_FEATURE;
        req.value = cpu_to_le16(USB_PORT_FEAT_RESET);
        req.index = cpu_to_le16(p);
        req.length = cpu_to_le16(0);
        if (submit_control_msg(udev, pipe, NULL, 0, &req) != 0)
            return UHIOERR_HOSTERROR;
    }
    return UHIOERR_NO_ERROR;
}

int usb_glue_bus_suspend(struct XHCIUnit *unit)
{
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    struct usb_device *udev = get_or_init_udev(unit, ctrl->rootdev);
    if (!udev) return UHIOERR_HOSTERROR;
    int maxp = root_hub_max_ports(ctrl);
    unsigned long pipe = usb_sndctrlpipe(udev, 0);
    for (int p = 1; p <= maxp; ++p) {
        struct devrequest req;
        _memset(&req, 0, sizeof(req));
        req.requesttype = USB_DIR_OUT | USB_RT_PORT;
        req.request = USB_REQ_SET_FEATURE;
        req.value = cpu_to_le16(USB_PORT_FEAT_SUSPEND);
        req.index = cpu_to_le16(p);
        if (submit_control_msg(udev, pipe, NULL, 0, &req) != 0)
            return UHIOERR_HOSTERROR;
    }
    return UHIOERR_NO_ERROR;
}

int usb_glue_bus_resume(struct XHCIUnit *unit)
{
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    struct usb_device *udev = get_or_init_udev(unit, ctrl->rootdev);
    if (!udev) return UHIOERR_HOSTERROR;
    int maxp = root_hub_max_ports(ctrl);
    unsigned long pipe = usb_sndctrlpipe(udev, 0);
    for (int p = 1; p <= maxp; ++p) {
        struct devrequest req;
        _memset(&req, 0, sizeof(req));
        req.requesttype = USB_DIR_OUT | USB_RT_PORT;
        req.request = USB_REQ_CLEAR_FEATURE;
        req.value = cpu_to_le16(USB_PORT_FEAT_SUSPEND);
        req.index = cpu_to_le16(p);
        if (submit_control_msg(udev, pipe, NULL, 0, &req) != 0)
            return UHIOERR_HOSTERROR;
    }
    return UHIOERR_NO_ERROR;
}

int usb_glue_bus_oper(struct XHCIUnit *unit)
{
    /* Ensure port power is on for all ports */
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    struct usb_device *udev = get_or_init_udev(unit, ctrl->rootdev);
    if (!udev) return UHIOERR_HOSTERROR;
    int maxp = root_hub_max_ports(ctrl);
    unsigned long pipe = usb_sndctrlpipe(udev, 0);
    for (int p = 1; p <= maxp; ++p) {
        struct devrequest req;
        _memset(&req, 0, sizeof(req));
        req.requesttype = USB_DIR_OUT | USB_RT_PORT;
        req.request = USB_REQ_SET_FEATURE;
        req.value = cpu_to_le16(USB_PORT_FEAT_POWER);
        req.index = cpu_to_le16(p);
        if (submit_control_msg(udev, pipe, NULL, 0, &req) != 0)
            return UHIOERR_HOSTERROR;
    }
    return UHIOERR_NO_ERROR;
}
