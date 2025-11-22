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

#include <xhci/usb.h>
#include <xhci/xhci.h>
#include <xhci/xhci-commands.h>

#include <device.h>
#include <debug.h>
#include <compat.h>
#include <minlist.h>

#include "usb_glue.h"

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[usb_glue] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[usb_glue] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

static unsigned int route_depth(unsigned int route)
{
    unsigned int depth = 0;
    while (route && depth < 5)
    {
        route >>= 4;
        depth++;
    }
    return depth;
}

static unsigned int build_route_string(struct usb_device *parent, unsigned int port)
{
    if (!parent)
        return 0;

    unsigned int depth = route_depth(parent->route);
    if (depth >= 5)
        return parent->route;

    unsigned int nibble = port & 0xF;
    if (nibble == 0)
        return parent->route;

    return parent->route | (nibble << (depth * 4));
}

static struct usb_device *resolve_parent(struct XHCIUnit *unit, UWORD addr)
{
    if (addr > 127)
        return NULL;

    struct usb_device *udev = &unit->xhci_ctrl->devices[addr];
    if (!udev->used)
        return NULL;

    return udev;
}

static void update_device_topology(struct XHCIUnit *unit, struct usb_device *udev,
                                   const struct IOUsbHWReq *io)
{
    if (io->iouh_Flags & UHFF_SPLITTRANS)
    {
        struct usb_device *parent = resolve_parent(unit, io->iouh_SplitHubAddr & 0x7F);
        if (parent)
        {
            unsigned int port = io->iouh_SplitHubPort & 0xFF;
            udev->parent = parent;
            udev->route = build_route_string(parent, port);
            udev->tt_slot = parent->slot_id;
            udev->tt_port = port;
            return;
        }
        /* Fall back to default routing if the parent isn't known yet */
        udev->parent = NULL;
        udev->route = 0;
        udev->tt_slot = 0;
        udev->tt_port = 0;
        return;
    }

    if (io->iouh_DevAddr == 0)
    {
        /* Default-address transfers without split data target the root hub path */
        udev->parent = NULL;
        udev->route = 0;
        udev->tt_slot = 0;
        udev->tt_port = 0;
    }
}

static struct usb_device *get_or_init_udev(struct XHCIUnit *unit, UWORD devaddr)
{
    if (devaddr > 127)
        return NULL;

    struct usb_device *udev = &unit->xhci_ctrl->devices[devaddr];
    if (!udev->used)
    {
        KprintfH("new device addr=%ld\n", (LONG)devaddr);
        _memset(udev, 0, sizeof(struct usb_device));
        udev->used = 1;
        udev->devnum = devaddr;
        udev->controller = unit->xhci_ctrl;
        /* Default EP0 packet size for new address (8) */
        udev->maxpacketsize0 = PACKET_SIZE_8;
        /* Default toggles cleared */

        _NewMinList(&udev->configurations);

        for (int i = 0; i < USB_MAXENDPOINTS; ++i)
            _NewMinList(&udev->ep_context[i].req_list);
    }
    else
    {
        KprintfH("existing device addr=%ld\n", (LONG)devaddr);
    }
    return udev;
}

static int queue_request_if_busy(struct usb_device *udev, struct IOUsbHWReq *io)
{
    unsigned int endpoint = io->iouh_Endpoint & 0x0F;
    if (endpoint >= USB_MAXENDPOINTS)
    {
        io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
        return -1;
    }

    struct ep_context *ep_ctx = &udev->ep_context[endpoint];

    if (ep_ctx->state == USB_DEV_EP_STATE_IDLE)
        return 0;

    AddTailMinList(&ep_ctx->req_list, (struct MinNode *)io);
    KprintfH("queued request cmd=%ld ep=%ld\n", (LONG)io->iouh_Req.io_Command, (LONG)endpoint);

    return 1;
}

static void dispatch_request(struct IOUsbHWReq *req)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)req->iouh_Req.io_Unit;
    if (!unit)
    {
        Kprintf("missing unit pointer for cmd=%ld\n", (LONG)req->iouh_Req.io_Command);
        io_reply_failed(req, UHIOERR_HOSTERROR);
        return;
    }

    switch (req->iouh_Req.io_Command)
    {
    case UHCMD_CONTROLXFER:
        usb_glue_ctrl(unit, req);
        break;
    case UHCMD_BULKXFER:
        usb_glue_bulk(unit, req);
        break;
    case UHCMD_INTXFER:
        usb_glue_int(unit, req);
        break;
    default:
        Kprintf("unsupported command %ld\n", (LONG)req->iouh_Req.io_Command);
        io_reply_failed(req, UHIOERR_BADPARAMS);
        return;
    }
}

static void parse_config_descriptor(struct usb_device *udev, UBYTE *data, UWORD len)
{
    if (len < 2)
    {
        KprintfH("too short len=%ld\n", (LONG)len);
        return;
    }

    struct usb_config *conf = AllocVecPooled(udev->controller->memoryPool, sizeof(*conf));
    if (!conf)
    {
        Kprintf("AllocVecPooled failed\n");
        return;
    }
    _memset(conf, 0, sizeof(*conf));

    struct usb_config_descriptor *desc = (struct usb_config_descriptor *)data;
    if (desc->bDescriptorType != USB_DT_CONFIG)
    {
        Kprintf("bad desc type %ld\n", (LONG)desc->bDescriptorType);
        goto error;
    }

    UWORD total_len = LE16(desc->wTotalLength);
    if ((UWORD)len < total_len)
    {
        KprintfH("short buffer len=%ld total_len=%ld\n", (LONG)len, (LONG)total_len);
        return;
    }

    UBYTE *cursor = data;
    UBYTE *end = data + total_len;
    if (cursor + desc->bLength > end)
    {
        Kprintf("bad desc length %ld\n", (LONG)desc->bLength);
        goto error;
    }

    CopyMem(desc, &conf->desc, sizeof(struct usb_config_descriptor));
    cursor += desc->bLength;

    KprintfH("wTotalLength=%ld bNumInterfaces=%ld bConfigurationValue=%ld iConfiguration=%ld bmAttributes=0x%02lx bMaxPower=%ld\n",
             (LONG)LE16(desc->wTotalLength),
             (LONG)desc->bNumInterfaces,
             (LONG)desc->bConfigurationValue,
             (LONG)desc->iConfiguration,
             (LONG)desc->bmAttributes,
             (LONG)desc->bMaxPower);

    // in 3.x there are association descriptors here

    int interface_map[USB_MAXINTERFACES];
    for (int i = 0; i < USB_MAXINTERFACES; ++i)
        interface_map[i] = -1;

    conf->no_of_if = 0;
    int if_index = 0;
    int current_alt_index = -1;
    struct usb_interface *current_if = NULL;
    struct usb_interface_altsetting *current_alt = NULL;

    while (cursor + 2 <= end)
    {
        UBYTE dlen = cursor[0];
        UBYTE dtype = cursor[1];
        if (dlen == 0)
        {
            Kprintf("zero length descriptor, aborting\n");
            break;
        }
        if (cursor + dlen > end)
        {
            Kprintf("descriptor overruns buffer (type=%ld len=%ld)\n", (LONG)dtype, (LONG)dlen);
            break;
        }

        switch (dtype)
        {
        case USB_DT_INTERFACE:
        {
            struct usb_interface_descriptor *ifd = (struct usb_interface_descriptor *)cursor;
            unsigned int iface_number = ifd->bInterfaceNumber;
            if (iface_number >= USB_MAXINTERFACES)
            {
                Kprintf("interface number %ld exceeds max %ld\n", (LONG)iface_number, (LONG)USB_MAXINTERFACES);
                current_if = NULL;
                current_alt = NULL;
                current_alt_index = -1;
                break;
            }

            if_index = interface_map[iface_number];
            if (if_index < 0)
            {
                if_index = conf->no_of_if;
                if (if_index >= USB_MAXINTERFACES)
                {
                    Kprintf("too many unique interfaces (%ld)\n", (LONG)if_index);
                    goto error;
                }
                interface_map[iface_number] = if_index;
                current_if = &conf->if_desc[if_index];
                _memset(current_if, 0, sizeof(struct usb_interface));
                current_if->interface_number = (u8)iface_number;
                current_if->num_altsetting = 0;
                current_if->active_altsetting = NULL;
                conf->no_of_if++;
            }
            else
            {
                current_if = &conf->if_desc[if_index];
            }

            current_if->interface_number = (u8)iface_number;

            if (current_if->num_altsetting >= USB_ALTSETTINGALLOC)
            {
                Kprintf("too many alternate settings (%ld) for interface %ld\n",
                        (LONG)current_if->num_altsetting, (LONG)iface_number);
                current_alt = NULL;
                current_alt_index = -1;
                break;
            }

            current_alt_index = current_if->num_altsetting++;
            current_alt = &current_if->altsetting[current_alt_index];
            _memset(current_alt, 0, sizeof(struct usb_interface_altsetting));

            CopyMem(ifd, &current_alt->desc, sizeof(struct usb_interface_descriptor));
            current_alt->no_of_ep = 0;

            KprintfH("interface %ld alt %ld: bInterfaceNumber=%ld bAlternateSetting=%ld bNumEndpoints=%ld bInterfaceClass=0x%02lx bInterfaceSubClass=0x%02lx bInterfaceProtocol=0x%02lx iInterface=%ld\n",
                     (LONG)if_index,
                     (LONG)current_alt_index,
                     (LONG)ifd->bInterfaceNumber,
                     (LONG)ifd->bAlternateSetting,
                     (LONG)ifd->bNumEndpoints,
                     (LONG)ifd->bInterfaceClass,
                     (LONG)ifd->bInterfaceSubClass,
                     (LONG)ifd->bInterfaceProtocol,
                     (LONG)ifd->iInterface);

            if (current_if->active_altsetting == NULL || current_alt->desc.bAlternateSetting == 0)
                current_if->active_altsetting = current_alt;
            break;
        }
        case USB_DT_ENDPOINT:
        {
            if (!current_if || !current_alt)
            {
                Kprintf("endpoint without interface or altsetting\n");
                break;
            }
            if (current_alt->no_of_ep >= USB_MAXENDPOINTS)
            {
                Kprintf("too many endpoints for interface %ld alt %ld\n",
                        (LONG)if_index, (LONG)current_alt_index);
                break;
            }

            struct usb_endpoint_descriptor *epd = (struct usb_endpoint_descriptor *)cursor;
            unsigned int ep_idx = current_alt->no_of_ep;
            CopyMem(epd, &current_alt->ep_desc[ep_idx], sizeof(struct usb_endpoint_descriptor));
            KprintfH("  endpoint %ld: bEndpointAddress=0x%02lx bmAttributes=0x%02lx wMaxPacketSize=%ld bInterval=%ld\n",
                     (LONG)ep_idx,
                     (LONG)epd->bEndpointAddress,
                     (LONG)epd->bmAttributes,
                     (LONG)LE16(epd->wMaxPacketSize),
                     (LONG)epd->bInterval);

            current_alt->no_of_ep++;
            break;
        }
        case USB_DT_SS_ENDPOINT_COMP:
        {
            KprintfH("found SS EP COMP descriptor\n");
            if (current_if && current_alt && current_alt->no_of_ep > 0)
            {
                struct usb_ss_ep_comp_descriptor *comp = (struct usb_ss_ep_comp_descriptor *)cursor;
                unsigned int ep_slot = current_alt->no_of_ep - 1;
                CopyMem(comp, &current_alt->ss_ep_comp_desc[ep_slot], sizeof(struct usb_ss_ep_comp_descriptor));
            }
            break;
        }
        default:
            // Skip class- or vendor-specific descriptors gracefully.
            KprintfH("found class/vendor-specific descriptor 0x%lx, len=%ld\n", (ULONG)dtype, (LONG)dlen);
            break;
        }

        cursor += dlen;
    }
    KprintfH("parsed config with %ld interfaces\n", (LONG)conf->no_of_if);

    if (conf->no_of_if != desc->bNumInterfaces)
    {
        Kprintf("interface count mismatch %ld != %ld\n",
                (LONG)conf->no_of_if, (LONG)desc->bNumInterfaces);
        goto error;
    }

    for (struct MinNode *n = udev->configurations.mlh_Head; n->mln_Succ; n = n->mln_Succ)
    {
        struct usb_config *oldconf = (struct usb_config *)n;
        if (oldconf->desc.bConfigurationValue == conf->desc.bConfigurationValue)
        {
            KprintfH("removing old config with value %ld\n", (LONG)oldconf->desc.bConfigurationValue);
            RemoveMinNode(n);
            FreeVecPooled(udev->controller->memoryPool, oldconf);
            break;
        }
    }
    AddHeadMinList(&udev->configurations, (struct MinNode *)conf);
    // TODO why AddTailMinList doesn't work here?

    return;

error:
    FreeVecPooled(udev->controller->memoryPool, conf);
}

int usb_glue_ctrl(struct XHCIUnit *unit, struct IOUsbHWReq *io)
{
    struct usb_device *udev = get_or_init_udev(unit, io->iouh_DevAddr);
    if (!udev)
    {
        io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
        return COMMAND_PROCESSED;
    }

    /* Update EP0 MPS hint from IO if provided */
    unsigned int maxpkt_hint = (unsigned int)io->iouh_MaxPktSize;
    if (maxpkt_hint == 8)
        udev->maxpacketsize0 = PACKET_SIZE_8;
    else if (maxpkt_hint == 16)
        udev->maxpacketsize0 = PACKET_SIZE_16;
    else if (maxpkt_hint == 32)
        udev->maxpacketsize0 = PACKET_SIZE_32;
    else
        udev->maxpacketsize0 = PACKET_SIZE_64;

    if (io->iouh_Flags & UHFF_HIGHSPEED)
        udev->speed = USB_SPEED_HIGH;
    else if (io->iouh_Flags & UHFF_LOWSPEED)
        udev->speed = USB_SPEED_LOW;
    else
        udev->speed = USB_SPEED_FULL;

    update_device_topology(unit, udev, io);

    {
        KprintfH("dev=%lx addr=%ld bmReqType=%02lx bReq=%02lx wValue=%04lx wIndex=%04lx wLength=%04lx flags=%lx timeout=%ld maxPktSize=%ld\n",
                 (ULONG)udev, (ULONG)udev->devnum,
                 (ULONG)io->iouh_SetupData.bmRequestType,
                 (ULONG)io->iouh_SetupData.bRequest,
                 (ULONG)io->iouh_SetupData.wValue,
                 (ULONG)io->iouh_SetupData.wIndex,
                 (ULONG)io->iouh_SetupData.wLength,
                 (ULONG)io->iouh_Flags,
                 (ULONG)io->iouh_NakTimeout,
                 (ULONG)io->iouh_MaxPktSize);

        KprintfH("split_hub_addr=%lx split_hub_port=%ld\n",
                 (ULONG)io->iouh_SplitHubAddr,
                 (ULONG)io->iouh_SplitHubPort);

        KprintfH("parent=%lx route=%ld tt_slot=%ld tt_port=%ld\n",
                 (ULONG)udev->parent,
                 (ULONG)udev->route,
                 (ULONG)udev->tt_slot,
                 (ULONG)udev->tt_port);
        KprintfH("maxpacketsize0=%lx speed=%ld\n",
                 (ULONG)udev->maxpacketsize0,
                 (LONG)udev->speed);
    }

    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    if (io->iouh_DevAddr == ctrl->rootdev)
    {
        xhci_submit_root(udev, io);
        return COMMAND_PROCESSED;
    }

    int queue_state = queue_request_if_busy(udev, io);
    if (queue_state > 0)
        return COMMAND_SCHEDULED;
    if (queue_state < 0)
        return COMMAND_PROCESSED;

    struct UsbSetupData *setup = &io->iouh_SetupData;

    if (setup->bRequest == USB_REQ_SET_ADDRESS && (setup->bmRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD)
    {
        xhci_address_device(udev, io);
        return COMMAND_SCHEDULED;
    }

    if (setup->bRequest == USB_REQ_SET_CONFIGURATION &&
        (setup->bmRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD)
    {
        int ret = xhci_set_configuration(udev, LE16(setup->wValue) & 0xff);
        if (ret != UHIOERR_NO_ERROR)
        {
            Kprintf("Failed to configure xHCI endpoint\n");
            io->iouh_Req.io_Error = ret;
            return COMMAND_PROCESSED;
        }

        // this will trigger a chain of commands and control xfer
        xhci_configure_endpoints(udev, FALSE, io);
        return COMMAND_SCHEDULED;
    }

    unsigned int timeout_ms = 0;
    if ((io->iouh_Flags & UHFF_NAKTIMEOUT))
        timeout_ms = io->iouh_NakTimeout;

    int ret = xhci_ctrl_tx(udev, io, timeout_ms);
    if (ret != UHIOERR_NO_ERROR)
    {
        io->iouh_Req.io_Error = ret;
        return COMMAND_PROCESSED;
    }

    return COMMAND_SCHEDULED;
}

int usb_glue_bulk(struct XHCIUnit *unit, struct IOUsbHWReq *io)
{
    struct usb_device *udev = get_or_init_udev(unit, io->iouh_DevAddr);
    if (!udev)
    {
        io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
        return COMMAND_PROCESSED;
    }

    KprintfH("dev=%ld addr=%ld ep=%ld dir=%s len=%ld flags=%lx tmo=%ld\n",
             (ULONG)udev, (ULONG)udev->devnum, io->iouh_Endpoint & 0x0F, (io->iouh_Dir == UHDIR_IN) ? "IN" : "OUT",
             (LONG)io->iouh_Length, (ULONG)io->iouh_Flags, (LONG)io->iouh_NakTimeout);

    int queue_state = queue_request_if_busy(udev, io);
    if (queue_state > 0)
        return COMMAND_SCHEDULED;
    if (queue_state < 0)
        return COMMAND_PROCESSED;

    unsigned int timeout_ms = 0;
    if ((io->iouh_Flags & UHFF_NAKTIMEOUT))
        timeout_ms = (unsigned int)io->iouh_NakTimeout;

    int ret = xhci_bulk_tx(udev, io, timeout_ms);
    if (ret != UHIOERR_NO_ERROR)
    {
        io->iouh_Req.io_Error = ret;
        return COMMAND_PROCESSED;
    }

    return COMMAND_SCHEDULED;
}

int usb_glue_int(struct XHCIUnit *unit, struct IOUsbHWReq *io)
{
    struct usb_device *udev = get_or_init_udev(unit, io->iouh_DevAddr);
    if (!udev)
    {
        io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
        return COMMAND_PROCESSED;
    }

    /* Root hub doesn't have a real interrupt endpoint ring here; synthesize success */
    if (udev->devnum == unit->xhci_ctrl->rootdev)
    {
        io->iouh_Actual = 0;
        io->iouh_Req.io_Error = UHIOERR_NO_ERROR;
        // TODO do we need to process it somehow? - yes, these are port state changes
        return COMMAND_PROCESSED;
    }

    KprintfH("dev=%lx addr=%ld ep=%ld dir=%s len=%ld interval=%ld flags=%lx timeout=%ld maxpkt=%ld\n",
             (ULONG)udev, (ULONG)udev->devnum, io->iouh_Endpoint & 0x0F, (io->iouh_Dir == UHDIR_IN) ? "IN" : "OUT",
             (LONG)io->iouh_Length, (LONG)io->iouh_Interval,
             (ULONG)io->iouh_Flags, (LONG)io->iouh_NakTimeout, (LONG)io->iouh_MaxPktSize);

    int queue_state = queue_request_if_busy(udev, io);
    if (queue_state > 0)
        return COMMAND_SCHEDULED;
    if (queue_state < 0)
        return COMMAND_PROCESSED;

    unsigned int timeout_ms = 0;
    if ((io->iouh_Flags & UHFF_NAKTIMEOUT))
        timeout_ms = (unsigned int)io->iouh_NakTimeout;

    int ret = xhci_bulk_tx(udev, io, timeout_ms);
    if (ret != UHIOERR_NO_ERROR)
    {
        io->iouh_Req.io_Error = ret;
        return COMMAND_PROCESSED;
    }

    return COMMAND_SCHEDULED;
}

void xhci_ep_schedule_next(struct usb_device *udev, int endpoint)
{
    if (endpoint < 0 || endpoint >= USB_MAXENDPOINTS)
        return;

    struct ep_context *ep_ctx = &udev->ep_context[endpoint];
    struct MinNode *node = RemHeadMinList(&ep_ctx->req_list);
    if (!node)
        return;

    struct IOUsbHWReq *req = (struct IOUsbHWReq *)node;

    KprintfH("starting queued request cmd=%ld ep=%ld\n", (LONG)req->iouh_Req.io_Command, (LONG)(req->iouh_Endpoint & 0x0F));
    dispatch_request(req);

    if (ep_ctx->state == USB_DEV_EP_STATE_IDLE)
        xhci_ep_schedule_next(udev, endpoint);
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
    if (!udev)
        return UHIOERR_HOSTERROR;
    int maxp = root_hub_max_ports(ctrl);
    for (int p = 1; p <= maxp; ++p)
    {
        struct IOUsbHWReq req;
        _memset(&req, 0, sizeof(req));
        req.iouh_SetupData.bmRequestType = USB_DIR_OUT | USB_RT_PORT; /* class=hub, recipient=other */
        req.iouh_SetupData.bRequest = USB_REQ_SET_FEATURE;
        req.iouh_SetupData.wValue = cpu_to_le16(USB_PORT_FEAT_RESET);
        req.iouh_SetupData.wIndex = cpu_to_le16(p);
        req.iouh_SetupData.wLength = cpu_to_le16(0);
        req.iouh_DevAddr = udev->devnum;
        req.iouh_MaxPktSize = 8U << udev->maxpacketsize0;

        xhci_submit_root(udev, &req);
    }
    return UHIOERR_NO_ERROR;
}

int usb_glue_bus_suspend(struct XHCIUnit *unit)
{
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    struct usb_device *udev = get_or_init_udev(unit, ctrl->rootdev);
    if (!udev)
        return UHIOERR_HOSTERROR;
    int maxp = root_hub_max_ports(ctrl);
    for (int p = 1; p <= maxp; ++p)
    {
        struct IOUsbHWReq req;
        _memset(&req, 0, sizeof(req));
        req.iouh_SetupData.bmRequestType = USB_DIR_OUT | USB_RT_PORT;
        req.iouh_SetupData.bRequest = USB_REQ_SET_FEATURE;
        req.iouh_SetupData.wValue = cpu_to_le16(USB_PORT_FEAT_SUSPEND);
        req.iouh_SetupData.wIndex = cpu_to_le16(p);
        req.iouh_DevAddr = udev->devnum;
        req.iouh_MaxPktSize = 8U << udev->maxpacketsize0;

        xhci_submit_root(udev, &req);
    }
    return UHIOERR_NO_ERROR;
}

int usb_glue_bus_resume(struct XHCIUnit *unit)
{
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    struct usb_device *udev = get_or_init_udev(unit, ctrl->rootdev);
    if (!udev)
        return UHIOERR_HOSTERROR;
    int maxp = root_hub_max_ports(ctrl);
    for (int p = 1; p <= maxp; ++p)
    {
        struct IOUsbHWReq req;
        _memset(&req, 0, sizeof(req));
        req.iouh_SetupData.bmRequestType = USB_DIR_OUT | USB_RT_PORT;
        req.iouh_SetupData.bRequest = USB_REQ_CLEAR_FEATURE;
        req.iouh_SetupData.wValue = cpu_to_le16(USB_PORT_FEAT_SUSPEND);
        req.iouh_SetupData.wIndex = cpu_to_le16(p);
        req.iouh_DevAddr = udev->devnum;
        req.iouh_MaxPktSize = 8U << udev->maxpacketsize0;

        xhci_submit_root(udev, &req);
    }
    return UHIOERR_NO_ERROR;
}

int usb_glue_bus_oper(struct XHCIUnit *unit)
{
    /* Ensure port power is on for all ports */
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    struct usb_device *udev = get_or_init_udev(unit, ctrl->rootdev);
    if (!udev)
        return UHIOERR_HOSTERROR;
    int maxp = root_hub_max_ports(ctrl);
    for (int p = 1; p <= maxp; ++p)
    {
        struct IOUsbHWReq req;
        _memset(&req, 0, sizeof(req));
        req.iouh_SetupData.bmRequestType = USB_DIR_OUT | USB_RT_PORT;
        req.iouh_SetupData.bRequest = USB_REQ_SET_FEATURE;
        req.iouh_SetupData.wValue = cpu_to_le16(USB_PORT_FEAT_POWER);
        req.iouh_SetupData.wIndex = cpu_to_le16(p);
        req.iouh_DevAddr = udev->devnum;
        req.iouh_MaxPktSize = 8U << udev->maxpacketsize0;

        xhci_submit_root(udev, &req);
    }
    return UHIOERR_NO_ERROR;
}

static void parse_control_msg(struct usb_device *udev, struct IOUsbHWReq *io)
{
    KprintfH("dev=%lx addr=%ld bmReqType=%02lx bReq=%02lx wValue=%04lx wIndex=%04lx wLength=%04lx actual=%ld\n",
             (ULONG)udev, (ULONG)udev->devnum,
             (ULONG)io->iouh_SetupData.bmRequestType,
             (ULONG)io->iouh_SetupData.bRequest,
             (ULONG)io->iouh_SetupData.wValue,
             (ULONG)io->iouh_SetupData.wIndex,
             (ULONG)io->iouh_SetupData.wLength,
             (LONG)io->iouh_Actual);

    /* If this was a successful GET_DESCRIPTOR(CONFIGURATION),
     * cache the configuration descriptor for later use.
     */
    if ((io->iouh_SetupData.bRequest == USB_REQ_GET_DESCRIPTOR) && (io->iouh_SetupData.bmRequestType & USB_DIR_IN) && (((LE16(io->iouh_SetupData.wValue)) >> 8) == USB_DT_CONFIG))
    {
        parse_config_descriptor(udev, (UBYTE *)io->iouh_Data, (UWORD)io->iouh_Actual);
    }

    /* Record TT think time from hub descriptors so child devices can be programmed correctly. */
    if ((io->iouh_SetupData.bRequest == USB_REQ_GET_DESCRIPTOR) &&
        (io->iouh_SetupData.bmRequestType == (USB_DIR_IN | USB_RT_HUB)))
    {
        UWORD dtype = (UWORD)((LE16(io->iouh_SetupData.wValue)) >> 8);
        if ((dtype == USB_DT_HUB || dtype == USB_DT_SS_HUB) && io->iouh_Actual >= 5)
        {
            struct usb_hub_descriptor *hub = (struct usb_hub_descriptor *)io->iouh_Data;
            UWORD characteristics = LE16(hub->wHubCharacteristics);
            u8 tt_think = (u8)((characteristics >> 5) & 0x3);
            if (udev->tt_think_time != tt_think)
            {
                udev->tt_think_time = tt_think;
                xhci_update_hub_tt(udev);
                KprintfH("hub TT think time code=%ld (bit-times=%ld)\n",
                         (LONG)tt_think, (LONG)((tt_think + 1) * 8));
            }
        }
    }

    /* If this was a successful standard SET_ADDRESS, migrate the glue context
     * from the old devaddr to the new one so subsequent transfers reuse the
     * same slot and endpoint state.
     */
    if ((io->iouh_SetupData.bRequest == USB_REQ_SET_ADDRESS) &&
        ((io->iouh_SetupData.bmRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD))
    {
        UWORD old_addr = io->iouh_DevAddr & 0x7F;
        UWORD new_addr = (UWORD)(LE16(io->iouh_SetupData.wValue) & 0x7F);
        if (new_addr != old_addr)
        {
            struct xhci_ctrl *ctrl = udev->controller;
            struct usb_device *from = &ctrl->devices[old_addr];
            struct usb_device *to = &ctrl->devices[new_addr];

            if (to->used)
            {
                Kprintf("overwriting existing ctx for addr %ld\n", (LONG)new_addr);
                _memset(to, 0, sizeof(*to));
            }

            /* Copy the entire device context so slot/endpoint state follows the device. */
            *to = *from;
            to->used = 1;
            to->devnum = new_addr;
            ctrl->devs[to->slot_id]->udev = to;

            /* Clear the old address entry to avoid stale references. */
            _memset(from, 0, sizeof(*from));
            Kprintf("migrated ctx from addr %ld to %ld\n", (LONG)old_addr, (LONG)new_addr);
        }
    }

    if ((io->iouh_SetupData.bRequest == USB_REQ_SET_INTERFACE) &&
        ((io->iouh_SetupData.bmRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD) &&
        ((io->iouh_SetupData.bmRequestType & 0x1F) == USB_RECIP_INTERFACE))
    {
        unsigned int iface = (unsigned int)(LE16(io->iouh_SetupData.wIndex) & 0xFF);
        unsigned int alt = (unsigned int)(LE16(io->iouh_SetupData.wValue) & 0xFF);
        int err = xhci_set_interface(udev, iface, alt);
        if (err != UHIOERR_NO_ERROR)
        {
            Kprintf("SET_INTERFACE iface=%ld alt=%ld failed err=%ld\n",
                    (LONG)iface, (LONG)alt, (LONG)err);
        }
    }
}

/* Hooks for responding to requests for lower layer */
void io_reply_failed(struct IOUsbHWReq *io, int err)
{
    if (io)
    {
        Kprintf("err=%ld\n", (LONG)err);
        io->iouh_Req.io_Error = err;
        ReplyMsg((struct Message *)io);
    }
}

void io_reply_data(struct usb_device *udev, struct IOUsbHWReq *io, int err, ULONG actual)
{
    if (!io)
        return;

    io->iouh_Actual = actual;
    io->iouh_Req.io_Error = err;

    if (io->iouh_Req.io_Command == UHCMD_CONTROLXFER && err == UHIOERR_NO_ERROR)
        parse_control_msg(udev, io);

    KprintfH("err=%ld actual=%ld\n", (LONG)err, (LONG)actual);
    ReplyMsg((struct Message *)io);
}

static void usb_glue_flush_udev(struct usb_device *udev)
{
    for (int ep = 0; ep < USB_MAXENDPOINTS; ++ep)
    {
        struct ep_context *ep_ctx = &udev->ep_context[ep];
        struct MinNode *node;
        while ((node = RemHeadMinList(&ep_ctx->req_list)) != NULL)
        {
            io_reply_failed((struct IOUsbHWReq *)node, IOERR_ABORTED);
        }
    }
}

void usb_glue_flush_queues(struct XHCIUnit *unit)
{
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    for (int addr = 0; addr <= 127; ++addr)
    {
        struct usb_device *udev = &ctrl->devices[addr];
        if (udev->used)
            usb_glue_flush_udev(udev);
    }
}