// SPDX-License-Identifier: GPL-2.0+
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#include <clib/utility_protos.h>
#else
#include <proto/exec.h>
#include <proto/utility.h>
#endif

#include <devices/usb.h>
#include <devices/usbhardware.h>

#include <usb.h>
#include <xhci.h>

#include <device.h>
#include <debug.h>
#include <compat.h>
#include <minlist.h>

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
        dc->udev.maxpacketsize0 = PACKET_SIZE_8;
        /* Default toggles cleared */

        _NewMinList(&dc->udev.configurations);
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
    //TODO clean up and remove usb_device structures
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

static void parse_config_descriptor(struct usb_device *udev, UBYTE *data, UWORD len)
{
    if (len < 2)
    {
        Kprintf("parse_config_descriptor: too short len=%ld\n", (LONG)len);
        return;
    }

    struct usb_config *conf = AllocVecPooled(udev->controller->memoryPool, sizeof(*conf));
    if (!conf)
    {
        Kprintf("parse_config_descriptor: AllocVecPooled failed\n");
        return;
    }
    _memset(conf, 0, sizeof(*conf));

    //TODO no data length validation
    struct usb_config_descriptor *desc = (struct usb_config_descriptor *)data;
    if (desc->bDescriptorType != USB_DT_CONFIG)
    {
        Kprintf("parse_config_descriptor: bad desc type %ld\n", (LONG)desc->bDescriptorType);
        goto error;
    }
    CopyMem(desc, &conf->desc, sizeof(*desc));
    data += desc->bLength;

    Kprintf("parse_config_descriptor: wTotalLength=%ld bNumInterfaces=%ld bConfigurationValue=%ld iConfiguration=%ld bmAttributes=0x%02lx bMaxPower=%ld\n",
        (LONG)LE16(desc->wTotalLength),
        (LONG)desc->bNumInterfaces,
        (LONG)desc->bConfigurationValue,
        (LONG)desc->iConfiguration,
        (LONG)desc->bmAttributes,
        (LONG)desc->bMaxPower);
    Kprintf("parse_config_descriptor: udev=%lx\n", udev);

    // TODO in 3.x there are association descriptors here

    conf->no_of_if = desc->bNumInterfaces;
    for(int i = 0; i < conf->no_of_if; i++)
    {
        struct usb_interface_descriptor *ifd = (struct usb_interface_descriptor *)data;
        if (ifd->bDescriptorType != USB_DT_INTERFACE)
        {
            Kprintf("parse_config_descriptor: bad if desc type %ld\n", (LONG)ifd->bDescriptorType);
            goto error;
        }
        CopyMem(ifd, &conf->if_desc[i].desc, sizeof(*ifd));
        data += ifd->bLength;

        conf->if_desc[i].no_of_ep = ifd->bNumEndpoints;
        conf->if_desc[i].num_altsetting = ifd->bAlternateSetting;
        Kprintf("parse_config_descriptor: interface %ld: bInterfaceNumber=%ld bAlternateSetting=%ld bNumEndpoints=%ld bInterfaceClass=0x%02lx bInterfaceSubClass=0x%02lx bInterfaceProtocol=0x%02lx iInterface=%ld\n",
            (LONG)i,
            (LONG)ifd->bInterfaceNumber,
            (LONG)ifd->bAlternateSetting,
            (LONG)ifd->bNumEndpoints,
            (LONG)ifd->bInterfaceClass,
            (LONG)ifd->bInterfaceSubClass,
            (LONG)ifd->bInterfaceProtocol,
            (LONG)ifd->iInterface);

        for(int j = 0; j < conf->if_desc[i].no_of_ep; j++)
        {
            struct usb_endpoint_descriptor *epd = (struct usb_endpoint_descriptor *)data;
            if (epd->bDescriptorType != USB_DT_ENDPOINT)
            {
                Kprintf("parse_config_descriptor: bad ep desc type %ld\n", (LONG)epd->bDescriptorType);
                goto error;
            }
            CopyMem(epd, &conf->if_desc[i].ep_desc[j], sizeof(*epd));
            data += epd->bLength;
            Kprintf("  parse_config_descriptor: endpoint %ld: bEndpointAddress=0x%02lx bmAttributes=0x%02lx wMaxPacketSize=%ld bInterval=%ld\n",
                (LONG)j,
                (LONG)epd->bEndpointAddress,
                (LONG)epd->bmAttributes,
                (LONG)LE16(epd->wMaxPacketSize),
                (LONG)epd->bInterval);
            // TODO copy usb_ss_ep_comp_descriptor associated with this endpoint
        }
        Kprintf("parse_config_descriptor: interface %ld has %ld endpoints\n", i, conf->if_desc[i].no_of_ep);
    }   
    Kprintf("parse_config_descriptor: parsed config with %ld interfaces\n", (LONG)conf->no_of_if);

    AddHeadMinList(&udev->configurations, (struct MinNode*)conf);
    //TODO why AddTailMinList doesn't work here?

    return;

error:
    FreeVecPooled(udev->controller->memoryPool, conf);

}

int usb_glue_ctrl(struct XHCIUnit *unit, struct IOUsbHWReq *io)
{
    struct usb_device *udev = get_or_init_udev(unit, io->iouh_DevAddr);
    if (!udev) { io->iouh_Req.io_Error = UHIOERR_BADPARAMS; return COMMAND_PROCESSED; }

    struct glue_devctx *dc = &gst(unit)->devs[io->iouh_DevAddr];
    if (!ep_try_lock(dc, 0, 1)) { io->iouh_Req.io_Error = UHIOERR_HOSTERROR; return COMMAND_PROCESSED; }

    /* Update EP0 MPS hint from IO if provided */
    unsigned int maxpkt_hint = (unsigned int)io->iouh_MaxPktSize;
    if (maxpkt_hint == 8) udev->maxpacketsize0 = PACKET_SIZE_8;
    else if (maxpkt_hint == 16) udev->maxpacketsize0 = PACKET_SIZE_16;
    else if (maxpkt_hint == 32) udev->maxpacketsize0 = PACKET_SIZE_32;
    else udev->maxpacketsize0 = PACKET_SIZE_64;

    unsigned long pipe = (io->iouh_SetupData.bmRequestType & URTF_IN)
               ? usb_rcvctrlpipe(udev, 0)
               : usb_sndctrlpipe(udev, 0);

    Kprintf("usb_glue_ctrl: dev=%lx addr=%ld pipe=%lx bmReqType=%02lx bReq=%02lx wValue=%04lx wIndex=%04lx wLength=%04lx flags=%lx timeout=%ld maxPktSize=%ld\n",
        (ULONG)udev, (ULONG)udev->devnum, pipe,
        (ULONG)io->iouh_SetupData.bmRequestType,
        (ULONG)io->iouh_SetupData.bRequest,
        (ULONG)io->iouh_SetupData.wValue,
        (ULONG)io->iouh_SetupData.wIndex,
        (ULONG)io->iouh_SetupData.wLength,
        (ULONG)io->iouh_Flags,
        (ULONG)io->iouh_NakTimeout,
        (ULONG)io->iouh_MaxPktSize);

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
        ret = submit_control_msg(udev, pipe, buf, len, &req, maxpkt_hint,
                                 (unsigned int)io->iouh_NakTimeout);
    else
        ret = submit_control_msg(udev, pipe, buf, len, &req, maxpkt_hint, XHCI_TIMEOUT);
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

    /* If this was a successful GET_DESCRIPTOR(CONFIGURATION),
     * cache the configuration descriptor for later use.
     */
    if (ret == 0 && (req.request == USB_REQ_GET_DESCRIPTOR) && (req.requesttype & USB_DIR_IN) && (((LE16(req.value)) >> 8) == USB_DT_CONFIG)) {
        parse_config_descriptor(udev, (UBYTE *)buf, (UWORD)udev->act_len);
    }

    /* If this was a successful standard SET_ADDRESS, migrate the glue context
     * from the old devaddr to the new one so subsequent transfers reuse the
     * same slot and endpoint state.
     */
    if (ret == 0 &&
        (req.request == USB_REQ_SET_ADDRESS) &&
        ((req.requesttype & USB_TYPE_MASK) == USB_TYPE_STANDARD)) {
        UWORD old_addr = io->iouh_DevAddr & 0x7F;
        UWORD new_addr = (UWORD)(LE16(req.value) & 0x7F);
        if (new_addr != old_addr) {
            struct glue_state *st = gst(unit);
            struct glue_devctx *from = &st->devs[old_addr];
            struct glue_devctx *to = &st->devs[new_addr];

            if (to->used) {
                Kprintf("usb_glue_ctrl: overwriting existing ctx for addr %ld\n", (LONG)new_addr);
                _memset(to, 0, sizeof(*to));
            }

            /* Copy the entire device context so slot/endpoint state follows the device. */
            *to = *from;
            to->used = 1;
            to->udev.devnum = new_addr;

            /* Ensure the EP0 IN guard stays locked until we drop it below. */
            to->in_flight[0][1] = from->in_flight[0][1];

            /* Clear the old address entry to avoid stale references. */
            _memset(from, 0, sizeof(*from));

            dc = to;
            udev = &dc->udev;
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

    unsigned int maxpkt = (unsigned int)io->iouh_MaxPktSize;

    if ((io->iouh_Flags & UHFF_NAKTIMEOUT) && io->iouh_NakTimeout)
        ret = submit_bulk_msg(udev, pipe, buf, len, maxpkt,
                              (unsigned int)io->iouh_NakTimeout);
    else
        ret = submit_bulk_msg(udev, pipe, buf, len, maxpkt, XHCI_TIMEOUT);
    uherr = map_status_to_uhio(udev->status);
    Kprintf("usb_glue_bulk: ret=%ld uherr=%ld udev->status=%ld act_len=%ld\n",
            (LONG)ret, (LONG)uherr, (LONG)udev->status, (LONG)udev->act_len);
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

    /* Root hub doesn't have a real interrupt endpoint ring here; synthesize success */
    if (udev->devnum == unit->xhci_ctrl->rootdev) {
        io->iouh_Actual = 0;
        io->iouh_Req.io_Error = UHIOERR_NO_ERROR;
        ep_unlock(dc, ep, dir);
        //TODO do we need to process it somehow?
        return COMMAND_PROCESSED;
    }

    Kprintf("usb_glue_int: dev=%lx addr=%ld ep=%ld dir=%s len=%ld interval=%ld flags=%lx timeout=%ld maxpkt=%ld\n",
        (ULONG)udev, (ULONG)udev->devnum, (LONG)ep, dir ? "IN" : "OUT",
        (LONG)io->iouh_Length, (LONG)io->iouh_Interval,
        (ULONG)io->iouh_Flags, (LONG)io->iouh_NakTimeout, (LONG)io->iouh_MaxPktSize);

    unsigned long pipe = (io->iouh_Dir == UHDIR_IN) ? usb_rcvintpipe(udev, ep)
                                                    : usb_sndintpipe(udev, ep);
    void *buf = io->iouh_Data;
    int len = (int)io->iouh_Length;
    int interval = (int)io->iouh_Interval; /* TODO: microframe vs ms */
    bool nonblock = false; /* synchronous for now */
    int ret;

    unsigned int maxpkt = (unsigned int)io->iouh_MaxPktSize;
    if ((io->iouh_Flags & UHFF_NAKTIMEOUT) && io->iouh_NakTimeout)
        ret = xhci_bulk_tx(udev, pipe, len, buf, maxpkt,
                           (unsigned int)io->iouh_NakTimeout);
    else
        ret = submit_int_msg(udev, pipe, buf, len, maxpkt, interval, nonblock);
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
    unsigned int ctrl_maxpkt = 8U << udev->maxpacketsize0;
    for (int p = 1; p <= maxp; ++p) {
        struct devrequest req;
        _memset(&req, 0, sizeof(req));
        req.requesttype = USB_DIR_OUT | USB_RT_PORT; /* class=hub, recipient=other */
        req.request = USB_REQ_SET_FEATURE;
        req.value = cpu_to_le16(USB_PORT_FEAT_RESET);
        req.index = cpu_to_le16(p);
        req.length = cpu_to_le16(0);
        if (submit_control_msg(udev, pipe, NULL, 0, &req, ctrl_maxpkt, XHCI_TIMEOUT) != 0)
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
    unsigned int ctrl_maxpkt = 8U << udev->maxpacketsize0;
    for (int p = 1; p <= maxp; ++p) {
        struct devrequest req;
        _memset(&req, 0, sizeof(req));
        req.requesttype = USB_DIR_OUT | USB_RT_PORT;
        req.request = USB_REQ_SET_FEATURE;
        req.value = cpu_to_le16(USB_PORT_FEAT_SUSPEND);
        req.index = cpu_to_le16(p);
        if (submit_control_msg(udev, pipe, NULL, 0, &req, ctrl_maxpkt, XHCI_TIMEOUT) != 0)
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
    unsigned int ctrl_maxpkt = 8U << udev->maxpacketsize0;
    for (int p = 1; p <= maxp; ++p) {
        struct devrequest req;
        _memset(&req, 0, sizeof(req));
        req.requesttype = USB_DIR_OUT | USB_RT_PORT;
        req.request = USB_REQ_CLEAR_FEATURE;
        req.value = cpu_to_le16(USB_PORT_FEAT_SUSPEND);
        req.index = cpu_to_le16(p);
        if (submit_control_msg(udev, pipe, NULL, 0, &req, ctrl_maxpkt, XHCI_TIMEOUT) != 0)
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
    unsigned int ctrl_maxpkt = 8U << udev->maxpacketsize0;
    for (int p = 1; p <= maxp; ++p) {
        struct devrequest req;
        _memset(&req, 0, sizeof(req));
        req.requesttype = USB_DIR_OUT | USB_RT_PORT;
        req.request = USB_REQ_SET_FEATURE;
        req.value = cpu_to_le16(USB_PORT_FEAT_POWER);
        req.index = cpu_to_le16(p);
        if (submit_control_msg(udev, pipe, NULL, 0, &req, ctrl_maxpkt, XHCI_TIMEOUT) != 0)
            return UHIOERR_HOSTERROR;
    }
    return UHIOERR_NO_ERROR;
}
