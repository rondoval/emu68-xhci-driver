// SPDX-License-Identifier: GPL-2.0+
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#include <clib/utility_protos.h>
#else
#include <proto/exec.h>
#include <proto/utility.h>
#endif

#include <devices/usbhardware.h>

#include <xhci/usb.h>
#include <xhci/xhci.h>
#include <xhci/xhci-events.h>
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

#define RT_ISO_IN_TARGET_TDS 16

static void parse_control_msg(struct usb_device *udev, struct IOUsbHWReq *io);
static void usb_glue_flush_udev(struct usb_device *udev);
static struct usb_device *usb_glue_alloc_udev(struct xhci_ctrl *ctrl, UWORD addr);
static UBYTE usb_glue_find_epaddr_by_num(struct usb_device *udev, UBYTE epnum);
static void usb_glue_patch_endpoint_address(struct usb_device *udev, struct IOUsbHWReq *io);
static struct usb_device *usb_glue_find_child_on_port(struct xhci_ctrl *ctrl, struct usb_device *hub, unsigned int port);

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

static struct usb_device *usb_glue_alloc_udev(struct xhci_ctrl *ctrl, UWORD addr)
{
    if (!ctrl || addr > 127)
        return NULL;

    if (ctrl->devices[addr])
        usb_glue_free_udev_slot(ctrl, addr);

    struct usb_device *udev = AllocVecPooled(ctrl->memoryPool, sizeof(*udev));
    if (!udev)
    {
        Kprintf("Failed to allocate usb_device for addr %ld\n", (LONG)addr);
        return NULL;
    }

    _memset(udev, 0, sizeof(*udev));
    udev->used = 1;
    udev->devnum = addr;
    udev->controller = ctrl;
    udev->maxpacketsize0 = PACKET_SIZE_8;

    _NewMinList(&udev->configurations);

    for (int i = 0; i < USB_MAXENDPOINTS; ++i)
    {
        _NewMinList(&udev->ep_context[i].pending_reqs);
        _NewMinList(&udev->ep_context[i].active_tds);
        InitSemaphore(&udev->ep_context[i].active_tds_lock);
    }

    ctrl->devices[addr] = udev;
    return udev;
}

static UBYTE usb_glue_find_epaddr_by_num(struct usb_device *udev, UBYTE epnum)
{
    if (!udev || !udev->active_config || epnum == 0)
        return 0;

    struct usb_config *cfg = udev->active_config;

    for (int i = 0; i < cfg->no_of_if; ++i)
    {
        struct usb_interface *iface = &cfg->if_desc[i];
        struct usb_interface_altsetting *alt = iface->active_altsetting;
        if (!alt)
            continue;

        for (int e = 0; e < alt->no_of_ep; ++e)
        {
            UBYTE addr = alt->ep_desc[e].bEndpointAddress;
            if ((addr & 0x0F) == epnum)
                return addr;
        }
    }

    return 0;
}

static void usb_glue_patch_endpoint_address(struct usb_device *udev, struct IOUsbHWReq *io)
{
    struct UsbSetupData *setup = &io->iouh_SetupData;

    /* Only patch class+endpoint recipient control requests. */
    if ((setup->bmRequestType & (USB_TYPE_MASK | USB_RECIP_MASK)) != (USB_TYPE_CLASS | USB_RECIP_ENDPOINT))
        return;

    /* If direction bit is already present, leave untouched. */
    UWORD wIndex = LE16(setup->wIndex);
    if (wIndex & 0x0080)
        return;

    UBYTE epnum = wIndex & 0x0F;
    if (epnum == 0)
        return;

    UBYTE fixed = usb_glue_find_epaddr_by_num(udev, epnum);
    if (!fixed || fixed == (UBYTE)wIndex)
        return;

    setup->wIndex = cpu_to_le16(fixed);

#ifdef DEBUG_HIGH
    KprintfH("patched endpoint address wIndex from %02lx to %02lx for epnum %ld\n",
             (ULONG)wIndex, (ULONG)fixed, (LONG)epnum);
#endif
}

/* Issue an internal CLEAR_FEATURE(ENDPOINT_HALT) to endpoint (by ep_index) on udev. Fire-and-forget. */
void usb_glue_clear_feature_halt_internal(struct usb_device *udev, u32 ep_index)
{
    if (!udev || !udev->controller)
        return;

    /* Convert ep_index (DCI-1) to USB endpoint address (number + direction bit). */
    UBYTE ep_addr = 0;
    if (ep_index != 0)
    {
        u32 ep = EP_INDEX_TO_ENDPOINT(ep_index);
        BOOL out = (ep_index & 0x1) != 0;
        ep_addr = (UBYTE)(ep | (out ? 0 : USB_DIR_IN));
    }

    struct xhci_ctrl *ctrl = udev->controller;
    struct IOUsbHWReq *io = AllocVecPooled(ctrl->memoryPool, sizeof(*io));
    if (!io)
        return;

    _memset(io, 0, sizeof(*io));
    io->iouh_Req.io_Command = UHCMD_CONTROLXFER;
    io->iouh_Req.io_Flags = IOF_QUICK; /* no reply port */
    io->iouh_DriverPrivate1 = (APTR)0xDEAD0001; /* magic tag to free on completion */
    io->iouh_DriverPrivate2 = (APTR)ctrl;

    io->iouh_SetupData.bmRequestType = USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_ENDPOINT;
    io->iouh_SetupData.bRequest = USB_REQ_CLEAR_FEATURE;
    io->iouh_SetupData.wValue = cpu_to_le16(USB_ENDPOINT_HALT);
    io->iouh_SetupData.wIndex = cpu_to_le16(ep_addr);
    io->iouh_SetupData.wLength = cpu_to_le16(0);

    io->iouh_DevAddr = udev->devnum;
    io->iouh_MaxPktSize = 8U << udev->maxpacketsize0;

    /* Split/hub routing if available from udev */
    io->iouh_SplitHubAddr = (udev->tt_slot & 0x7F);
    io->iouh_SplitHubPort = (udev->tt_port & 0xFF);
    if (udev->speed == USB_SPEED_LOW)
        io->iouh_Flags |= UHFF_LOWSPEED;
    else if (udev->speed == USB_SPEED_HIGH)
        io->iouh_Flags |= UHFF_HIGHSPEED;

    /* Queue it; on failure free immediately. */
    if (xhci_ctrl_tx(udev, io, 1000) != UHIOERR_NO_ERROR)
    {
        FreeVecPooled(ctrl->memoryPool, io);
    }
}

static void usb_glue_queue_clear_tt(struct usb_device *hub, u16 devinfo, u16 tt_port)
{
    if (!hub || !hub->controller)
        return;

    struct xhci_ctrl *ctrl = hub->controller;
    struct IOUsbHWReq *io = AllocVecPooled(ctrl->memoryPool, sizeof(*io));
    if (!io)
        return;

    _memset(io, 0, sizeof(*io));
    io->iouh_Req.io_Command = UHCMD_CONTROLXFER;
    io->iouh_Req.io_Flags = IOF_QUICK;
    io->iouh_DriverPrivate1 = (APTR)0xDEAD0001; /* magic tag to free on completion */
    io->iouh_DriverPrivate2 = (APTR)ctrl;

    io->iouh_SetupData.bmRequestType = USB_DIR_OUT | USB_RT_PORT;
    io->iouh_SetupData.bRequest = HUB_CLEAR_TT_BUFFER;
    io->iouh_SetupData.wValue = cpu_to_le16(devinfo);
    io->iouh_SetupData.wIndex = cpu_to_le16(tt_port);
    io->iouh_SetupData.wLength = cpu_to_le16(0);

    io->iouh_DevAddr = hub->devnum;
    io->iouh_MaxPktSize = 8U << hub->maxpacketsize0;

    if (hub->speed == USB_SPEED_LOW)
        io->iouh_Flags |= UHFF_LOWSPEED;
    else if (hub->speed == USB_SPEED_HIGH)
        io->iouh_Flags |= UHFF_HIGHSPEED;

    if (xhci_ctrl_tx(hub, io, 1000) != UHIOERR_NO_ERROR)
    {
        FreeVecPooled(ctrl->memoryPool, io);
    }
}

/* Issue an internal CLEAR_TT_BUFFER to the parent hub for control/bulk endpoints behind a TT. */
void usb_glue_clear_tt_buffer_internal(struct usb_device *udev, u32 ep_index, int ep_type)
{
    if (!udev || !udev->parent)
        return;

    /* Only applies to control or bulk endpoints behind a TT. */
    if (ep_type != USB_ENDPOINT_XFER_CONTROL && ep_type != USB_ENDPOINT_XFER_BULK)
        return;

    if (!udev->tt_port)
        return;

    struct usb_device *hub = udev->parent;

    u16 epnum = (u16)EP_INDEX_TO_ENDPOINT(ep_index);
    BOOL out = (ep_index & 0x1) != 0;

    u16 devinfo = epnum;
    devinfo |= ((u16)udev->devnum) << 4;
    devinfo |= ((u16)ep_type) << 11;
    if (!out)
        devinfo |= 1 << 15;

    usb_glue_queue_clear_tt(hub, devinfo, (u16)udev->tt_port);

    /* Control endpoints require clearing both directions. */
    if (ep_type == USB_ENDPOINT_XFER_CONTROL)
        usb_glue_queue_clear_tt(hub, devinfo ^ (1U << 15), (u16)udev->tt_port);
}

static void update_device_topology(struct XHCIUnit *unit, struct usb_device *udev,
                                   const struct IOUsbHWReq *io)
{
    if (io->iouh_Flags & UHFF_SPLITTRANS)
    {
        struct usb_device *parent = unit->xhci_ctrl->devices[io->iouh_SplitHubAddr & 0x7F];
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
    if (!unit || !unit->xhci_ctrl || devaddr > 127)
        return NULL;

    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    struct usb_device *udev = ctrl->devices[devaddr];
    if (!udev)
    {
        KprintfH("new device addr=%ld\n", (LONG)devaddr);
        udev = usb_glue_alloc_udev(ctrl, devaddr);
    }
    else
    {
        KprintfH("existing device addr=%ld\n", (LONG)devaddr);
    }

    return udev;
}

/* Sum bytes of in-flight RT ISO TDs to honor Poseidon OutPrefetch budget. */
static inline BOOL minlist_empty(const struct MinList *list)
{
    return list->mlh_Head == (struct MinNode *)&list->mlh_Tail;
}

static void enqueue_pending_request(struct ep_context *ep_ctx, struct IOUsbHWReq *io, const char *reason)
{
    io->iouh_DriverPrivate1 = ep_ctx;
    AddTailMinList(&ep_ctx->pending_reqs, (struct MinNode *)io);
    Kprintf("queued request cmd=%ld ep=%ld (%s)\n",
             (LONG)io->iouh_Req.io_Command,
             (LONG)(io->iouh_Endpoint & 0x0F),
             reason);
}

static void enqueue_pending_request_front(struct ep_context *ep_ctx, struct IOUsbHWReq *io, const char *reason)
{
    io->iouh_DriverPrivate1 = ep_ctx;
    AddHeadMinList(&ep_ctx->pending_reqs, (struct MinNode *)io);
    Kprintf("requeued request cmd=%ld ep=%ld (%s)\n",
             (LONG)io->iouh_Req.io_Command,
             (LONG)(io->iouh_Endpoint & 0x0F),
             reason);
}

static void ep_teardown(struct xhci_ctrl *ctrl, struct ep_context *ep_ctx)
{
    Kprintf("tearing down EP context\n");
    struct MinNode *node;

    while ((node = RemHeadMinList(&ep_ctx->pending_reqs)) != NULL)
    {
        struct IOUsbHWReq *req = (struct IOUsbHWReq *)node;
        req->iouh_DriverPrivate1 = NULL;
        req->iouh_DriverPrivate2 = NULL;
        io_reply_failed(req, IOERR_ABORTED);
    }

    ObtainSemaphore(&ep_ctx->active_tds_lock);
    while ((node = RemHeadMinList(&ep_ctx->active_tds)) != NULL)
    {
        struct xhci_td *td = (struct xhci_td *)node;
        xhci_td_release_trbs(td);
        if (td->req)
        {
            if (td->req->iouh_Data)
                xhci_dma_unmap(ctrl, (dma_addr_t)td->req->iouh_Data, td->length);
            if (td->rt_iso && td->req->iouh_Dir == UHDIR_IN && td->req->iouh_Data)
                FreeVecPooled(ctrl->memoryPool, td->req->iouh_Data);
            if (td->rt_iso)
                FreeVecPooled(ctrl->memoryPool, td->req);
            else
                io_reply_failed(td->req, IOERR_ABORTED);
        }
        FreeVecPooled(ctrl->memoryPool, td);
    }
    ReleaseSemaphore(&ep_ctx->active_tds_lock);

    /* Reset RT ISO inflight counters after teardown since all TDs are freed. */
    ep_ctx->rt_inflight_bytes = 0;
    ep_ctx->rt_inflight_tds = 0;

    if (ep_ctx->rt_template_req)
    {
        FreeVecPooled(ctrl->memoryPool, ep_ctx->rt_template_req);
        ep_ctx->rt_template_req = NULL;
    }
    if (ep_ctx->rt_stop_pending)
    {
        io_reply_failed(ep_ctx->rt_stop_pending, IOERR_ABORTED);
        ep_ctx->rt_stop_pending = NULL;
    }
    ep_ctx->rt_req = NULL;
}

static int handle_ring_busy(struct usb_device *udev, struct IOUsbHWReq *io)
{
    unsigned int endpoint = io->iouh_Endpoint & 0x0F;
    if (endpoint >= USB_MAXENDPOINTS)
    {
        io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
        return COMMAND_PROCESSED;
    }

    struct ep_context *ep_ctx = &udev->ep_context[endpoint];
    if (io->iouh_DriverPrivate1 == ep_ctx)
        enqueue_pending_request_front(ep_ctx, io, "ring-busy");
    else
        enqueue_pending_request(ep_ctx, io, "ring-busy");

    return COMMAND_SCHEDULED;
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
    case UHCMD_ISOXFER:
        usb_glue_iso(unit, req);
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

    //TODO check if could be removed or replaced with data from our copy of device descriptor
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

    update_device_topology(unit, udev, io); // TODO remove once discovery works OK, i.e. detects hub, port and root port correctly

    /* Work around class drivers that omit the direction bit in endpoint-recipient requests (e.g., UAC1 SET_CUR). */
    usb_glue_patch_endpoint_address(udev, io);

    {
        KprintfH("dev=%lx addr=%ld bmReqType=%02lx bReq=%02lx wValue=%04lx wIndex=%04lx wLength=%04lx flags=%lx timeout=%ld maxPktSize=%ld\n",
                 (ULONG)udev, (ULONG)udev->devnum,
                 (ULONG)io->iouh_SetupData.bmRequestType,
                 (ULONG)io->iouh_SetupData.bRequest,
                 LE16(io->iouh_SetupData.wValue),
                 LE16(io->iouh_SetupData.wIndex),
                 LE16(io->iouh_SetupData.wLength),
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
        parse_control_msg(udev, io);
        return COMMAND_PROCESSED;
    }

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
    
    /* If we don't have a slot yet, enable one and allocate Virt Dev */
    if (udev->slot_id == 0)
    {
        //TODO ugly hack, we should create the slot...
        io->iouh_Req.io_Error = UHIOERR_HOSTERROR;
        return COMMAND_PROCESSED;
    }

    int ret = xhci_ctrl_tx(udev, io, timeout_ms);
    if (ret == UHIOERR_RING_BUSY)
        return handle_ring_busy(udev, io);
    if (ret != UHIOERR_NO_ERROR)
    {
        io->iouh_Req.io_Error = ret;
        return COMMAND_PROCESSED;
    }

    io->iouh_DriverPrivate1 = NULL;
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

    unsigned int timeout_ms = 0;
    if ((io->iouh_Flags & UHFF_NAKTIMEOUT))
        timeout_ms = (unsigned int)io->iouh_NakTimeout;

    int ret = xhci_bulk_tx(udev, io, timeout_ms);
    if (ret == UHIOERR_RING_BUSY)
        return handle_ring_busy(udev, io);
    if (ret != UHIOERR_NO_ERROR)
    {
        io->iouh_Req.io_Error = ret;
        return COMMAND_PROCESSED;
    }

    io->iouh_DriverPrivate1 = NULL;
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

    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    if (udev->devnum == ctrl->rootdev)
    {
        if (ctrl->root_int_req)
        {
            KprintfH("root hub interrupt request already pending\n");
            io_reply_failed(io, UHIOERR_HOSTERROR);
            return COMMAND_PROCESSED;
        }

        ctrl->root_int_req = io;
        xhci_roothub_maybe_complete(ctrl);
        return COMMAND_SCHEDULED;
    }

    KprintfH("dev=%lx addr=%ld ep=%ld dir=%s len=%ld interval=%ld flags=%lx timeout=%ld maxpkt=%ld\n",
             (ULONG)udev, (ULONG)udev->devnum, io->iouh_Endpoint & 0x0F, (io->iouh_Dir == UHDIR_IN) ? "IN" : "OUT",
             (LONG)io->iouh_Length, (LONG)io->iouh_Interval,
             (ULONG)io->iouh_Flags, (LONG)io->iouh_NakTimeout, (LONG)io->iouh_MaxPktSize);

    unsigned int timeout_ms = 0;
    if ((io->iouh_Flags & UHFF_NAKTIMEOUT))
        timeout_ms = (unsigned int)io->iouh_NakTimeout;

    int ret = xhci_int_tx(udev, io, timeout_ms);
    if (ret == UHIOERR_RING_BUSY)
        return handle_ring_busy(udev, io);
    if (ret != UHIOERR_NO_ERROR)
    {
        io->iouh_Req.io_Error = ret;
        return COMMAND_PROCESSED;
    }

    io->iouh_DriverPrivate1 = NULL;
    return COMMAND_SCHEDULED;
}

int usb_glue_iso(struct XHCIUnit *unit, struct IOUsbHWReq *io)
{
    struct usb_device *udev = get_or_init_udev(unit, io->iouh_DevAddr);
    if (!udev)
    {
        io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
        return COMMAND_PROCESSED;
    }

    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    if (udev->devnum == ctrl->rootdev)
    {
        Kprintf("isochronous transfer on root hub are not supported\n");
        io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
        return COMMAND_PROCESSED;
    }

    KprintfH("dev=%lx addr=%ld ep=%ld dir=%s len=%ld interval=%ld flags=%lx maxpkt=%ld\n",
             (ULONG)udev, (ULONG)udev->devnum, io->iouh_Endpoint & 0x0F,
             (io->iouh_Dir == UHDIR_IN) ? "IN" : "OUT",
             (LONG)io->iouh_Length, (LONG)io->iouh_Interval,
             (ULONG)io->iouh_Flags, (LONG)io->iouh_MaxPktSize);

    unsigned int timeout_ms = 0;
    if ((io->iouh_Flags & UHFF_NAKTIMEOUT))
        timeout_ms = (unsigned int)io->iouh_NakTimeout;

    int ret = xhci_iso_tx(udev, io, timeout_ms);
    if (ret == UHIOERR_RING_BUSY)
        return handle_ring_busy(udev, io);
    if (ret != UHIOERR_NO_ERROR)
    {
        io->iouh_Req.io_Error = ret;
        return COMMAND_PROCESSED;
    }

    io->iouh_DriverPrivate1 = NULL;
    return COMMAND_SCHEDULED;
}

int usb_glue_add_iso_handler(struct XHCIUnit *unit, struct IOUsbHWReq *io)
{
    // TODO check if UHSF_OPERATIONAL
    // TODO cleanup, move to EP state machine
    if (!io->iouh_Data)
        goto badparams;

    struct usb_device *udev = get_or_init_udev(unit, io->iouh_DevAddr);
    if (!udev)
        goto badparams;

    unsigned int endpoint = io->iouh_Endpoint & 0x0F;
    if (endpoint >= USB_MAXENDPOINTS)
        goto badparams;

    struct ep_context *ep_ctx = &udev->ep_context[endpoint];
    if (ep_ctx->state != USB_DEV_EP_STATE_IDLE)
    {
        /* can only enable RT ISO if EP is idle */
        Kprintf("EP not idle. Current state: %ld\n", ep_ctx->state);
        io->iouh_Req.io_Error = UHIOERR_HOSTERROR;
        return COMMAND_PROCESSED;
    }

    ep_ctx->rt_req = (struct IOUsbHWRTIso *)io->iouh_Data;
    ep_ctx->state = USB_DEV_EP_STATE_RT_ISO_STOPPED;
    ep_ctx->rt_stop_pending = NULL;
    ep_ctx->rt_last_buffer = NULL;
    ep_ctx->rt_last_filled = 0;
    ep_ctx->rt_inflight_bytes = 0;
    ep_ctx->rt_inflight_tds = 0;

    ep_ctx->rt_template_req = AllocVecPooled(unit->xhci_ctrl->memoryPool, sizeof(struct IOUsbHWReq));
    if (!ep_ctx->rt_template_req)
    {
        Kprintf("Failed to allocate memory\n");
        io->iouh_Req.io_Error = UHIOERR_OUTOFMEMORY;
        return COMMAND_PROCESSED;
    }

    CopyMem(io, ep_ctx->rt_template_req, sizeof(struct IOUsbHWReq));

    struct IOUsbHWRTIso *rt = (struct IOUsbHWRTIso *)io->iouh_Data;
    if (io->iouh_Dir == UHDIR_IN)
    {
        Kprintf("Added ISO handler: EP %ld in req hook: %lx, in done hook: %lx, prefetch: %ld\n", endpoint, rt->urti_InReqHook, rt->urti_InDoneHook, rt->urti_OutPrefetch);
    }
    else
    {
        Kprintf("Added ISO handler: EP %ld out req hook: %lx, out done hook: %lx, prefetch: %ld\n", endpoint, rt->urti_OutReqHook, rt->urti_OutDoneHook, rt->urti_OutPrefetch);
    }

    io->iouh_Actual = 0;
    io->iouh_Req.io_Error = UHIOERR_NO_ERROR;
    return COMMAND_PROCESSED;

badparams:
    Kprintf("Bad params\n");
    io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
    return COMMAND_PROCESSED;
}

int usb_glue_rem_iso_handler(struct XHCIUnit *unit, struct IOUsbHWReq *io)
{
    // TODO cleanup, move to EP state machine
    if (!io->iouh_Data)
        goto badparams;

    struct usb_device *udev = get_or_init_udev(unit, io->iouh_DevAddr);
    if (!udev)
        goto badparams;

    unsigned int endpoint = io->iouh_Endpoint & 0x0F;
    if (endpoint >= USB_MAXENDPOINTS)
        goto badparams;

    struct ep_context *ep_ctx = &udev->ep_context[endpoint];

    if (ep_ctx->state != USB_DEV_EP_STATE_RT_ISO_STOPPED)
    {
        Kprintf("EP not in RT_ISO_STOPPED/IDLE state. Current state: %ld\n", ep_ctx->state);
        io->iouh_Req.io_Error = UHIOERR_HOSTERROR;
        return COMMAND_PROCESSED;
    }

    if (io->iouh_Data == ep_ctx->rt_req)
    {
        ep_ctx->rt_req = NULL;
        ep_ctx->rt_last_buffer = NULL;
        ep_ctx->rt_last_filled = 0;
        if (ep_ctx->rt_template_req)
        {
            FreeVecPooled(unit->xhci_ctrl->memoryPool, ep_ctx->rt_template_req);
            ep_ctx->rt_template_req = NULL;
        }
        xhci_ep_set_idle(udev, endpoint);
        Kprintf("Successfully removed ISO handler (state reset to IDLE)\n");
        io->iouh_Req.io_Error = UHIOERR_NO_ERROR;
        return COMMAND_PROCESSED;
    }

badparams:
    Kprintf("Bad parameters while removing ISO handler\n");
    io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
    return COMMAND_PROCESSED;
}

static void xhci_ep_schedule_rt_iso_out(struct usb_device *udev, struct ep_context *ep_ctx)
{
    struct xhci_ctrl *ctrl = udev->controller;
    struct IOUsbHWReq *template = ep_ctx->rt_template_req;
    ULONG prefetch_bytes = ep_ctx->rt_req->urti_OutPrefetch;

    struct xhci_giveback_info giveback = {0};
    BOOL have_giveback = FALSE;
    BOOL first_td = TRUE;

    while (ep_ctx->rt_inflight_bytes < prefetch_bytes)
    {
        ULONG frame = ep_ctx->rt_next_frame;
        struct IOUsbHWReq *rt_io = AllocVecPooled(ctrl->memoryPool, sizeof(struct IOUsbHWReq));
        if (!rt_io)
        {
            Kprintf("Failed to alloc RT ISO IO req\n");
            break;
        }
        CopyMem(template, rt_io, sizeof(struct IOUsbHWReq));

        struct IOUsbHWBufferReq rt_buffer_req;
        rt_buffer_req.ubr_Length = prefetch_bytes;
        rt_buffer_req.ubr_Buffer = ep_ctx->rt_last_buffer;
        rt_buffer_req.ubr_Flags = 0;
        rt_buffer_req.ubr_Frame = frame; /* monotonic frame counter to avoid jumps */

        KprintfH("RT ISO OUT sched frame=%lu len=%lu inflight_bytes=%lu inflight_tds=%lu\n",
             (ULONG)frame,
             (ULONG)rt_buffer_req.ubr_Length,
             (ULONG)ep_ctx->rt_inflight_bytes,
             (ULONG)ep_ctx->rt_inflight_tds);

        CallHookPkt(ep_ctx->rt_req->urti_OutReqHook, ep_ctx->rt_req, &rt_buffer_req);

        if (!rt_buffer_req.ubr_Buffer || rt_buffer_req.ubr_Length == 0)
        {
            Kprintf("RT ISO hook provided no buffer/length\n");
            FreeVecPooled(ctrl->memoryPool, rt_io);
            break;
        }

        ULONG offset = 0;
        if (rt_buffer_req.ubr_Buffer == ep_ctx->rt_last_buffer)
        {
            offset = ep_ctx->rt_last_filled;
        }
        else
        {
            ep_ctx->rt_last_buffer = rt_buffer_req.ubr_Buffer;
        }
        ep_ctx->rt_last_filled = offset + rt_buffer_req.ubr_Length;

        if (rt_buffer_req.ubr_Flags & UBFF_CONTBUFFER)
        {
            Kprintf("Continuous buffer not supported yet\n");
        }

        rt_io->iouh_Data = (APTR)((ULONG)rt_buffer_req.ubr_Buffer + offset);
        rt_io->iouh_Length = rt_buffer_req.ubr_Length;
        rt_io->iouh_Frame = (UWORD)frame;

        struct xhci_giveback_info local_giveback = {0};
        BOOL defer = TRUE; /* RT ISO defers doorbell to per-run giveback */
        int ret = xhci_rt_iso_tx(udev, rt_io, defer, first_td ? &local_giveback : NULL);
        if (ret == UHIOERR_RING_BUSY)
        {
            FreeVecPooled(ctrl->memoryPool, rt_io);
            KprintfH("RT ISO ring busy, deferring\n");
            break;
        }
        if (ret != UHIOERR_NO_ERROR)
        {
            FreeVecPooled(ctrl->memoryPool, rt_io);
            Kprintf("RT ISO submit failed %ld\n", (LONG)ret);
            break;
        }

        if (first_td && local_giveback.start_trb)
        {
            giveback = local_giveback;
            have_giveback = TRUE;
        }

        ep_ctx->rt_next_frame = (frame + 1) & 0xffff;
        ep_ctx->rt_inflight_bytes += rt_io->iouh_Length;
        ep_ctx->rt_inflight_tds++;
        KprintfH("RT ISO OUT queued frame=%lu len=%lu inflight_bytes=%lu inflight_tds=%lu\n",
                 (ULONG)frame,
                 (ULONG)rt_io->iouh_Length,
                 (ULONG)ep_ctx->rt_inflight_bytes,
                 (ULONG)ep_ctx->rt_inflight_tds);
        first_td = FALSE;
    }

    if (have_giveback)
        xhci_ring_giveback(udev, &giveback);
}

static void xhci_ep_schedule_rt_iso_in(struct usb_device *udev, struct ep_context *ep_ctx)
{
    struct xhci_ctrl *ctrl = udev->controller;
    struct IOUsbHWReq *template = ep_ctx->rt_template_req;

    struct xhci_giveback_info giveback = {0};
    BOOL have_giveback = FALSE;
    BOOL first_td = TRUE;

    while (ep_ctx->rt_inflight_tds < RT_ISO_IN_TARGET_TDS)
    {
        ULONG frame = ep_ctx->rt_next_frame;
        struct IOUsbHWReq *rt_io = AllocVecPooled(ctrl->memoryPool, sizeof(struct IOUsbHWReq));
        if (!rt_io)
        {
            Kprintf("Failed to alloc RT ISO IO req\n");
            break;
        }
        CopyMem(template, rt_io, sizeof(struct IOUsbHWReq));

        rt_io->iouh_Data = AllocVecPooled(ctrl->memoryPool, template->iouh_MaxPktSize);
        if (!rt_io->iouh_Data)
        {
            Kprintf("Failed to alloc RT ISO staging buffer\n");
            FreeVecPooled(ctrl->memoryPool, rt_io);
            break;
        }
        rt_io->iouh_Length = template->iouh_MaxPktSize;
        rt_io->iouh_Frame = (UWORD)frame;

        KprintfH("RT ISO IN sched frame=%lu maxpkt=%lu inflight_bytes=%lu inflight_tds=%lu\n",
                 (ULONG)frame,
                 (ULONG)rt_io->iouh_Length,
                 (ULONG)ep_ctx->rt_inflight_bytes,
                 (ULONG)ep_ctx->rt_inflight_tds);

        struct xhci_giveback_info local_giveback = {0};
        BOOL defer = TRUE; /* RT ISO defers doorbell to per-run giveback */
        int ret = xhci_rt_iso_tx(udev, rt_io, defer, first_td ? &local_giveback : NULL);
        if (ret == UHIOERR_RING_BUSY)
        {
            FreeVecPooled(ctrl->memoryPool, rt_io->iouh_Data);
            FreeVecPooled(ctrl->memoryPool, rt_io);
            KprintfH("RT ISO ring busy, deferring\n");
            break;
        }
        if (ret != UHIOERR_NO_ERROR)
        {
            FreeVecPooled(ctrl->memoryPool, rt_io->iouh_Data);
            FreeVecPooled(ctrl->memoryPool, rt_io);
            Kprintf("RT ISO submit failed %ld\n", (LONG)ret);
            break;
        }

        if (first_td && local_giveback.start_trb)
        {
            giveback = local_giveback;
            have_giveback = TRUE;
        }

        ep_ctx->rt_next_frame = (frame + 1) & 0xffff;
        ep_ctx->rt_inflight_bytes += rt_io->iouh_Length;
        ep_ctx->rt_inflight_tds++;
        KprintfH("RT ISO IN queued frame=%lu len=%lu inflight_bytes=%lu inflight_tds=%lu\n",
                 (ULONG)frame,
                 (ULONG)rt_io->iouh_Length,
                 (ULONG)ep_ctx->rt_inflight_bytes,
                 (ULONG)ep_ctx->rt_inflight_tds);
        first_td = FALSE;
    }

    if (have_giveback)
        xhci_ring_giveback(udev, &giveback);
}

void xhci_ep_schedule_rt_iso(struct usb_device *udev, int endpoint)
{
    if (endpoint < 0 || endpoint >= USB_MAXENDPOINTS)
    {
        Kprintf("Invalid endpoint %d\n", endpoint);
        return;
    }

    struct ep_context *ep_ctx = &udev->ep_context[endpoint];
    if (ep_ctx->state != USB_DEV_EP_STATE_RT_ISO_RUNNING)
    {
        Kprintf("EP not in RT_ISO_RUNNING\n");
        return;
    }

    struct IOUsbHWReq *template = ep_ctx->rt_template_req;
    if (!template)
    {
        Kprintf("No RT ISO template request\n");
        xhci_ep_set_failed(udev, endpoint);
        return;
    }

    if (template->iouh_Dir == UHDIR_IN)
        xhci_ep_schedule_rt_iso_in(udev, ep_ctx);
    else
        xhci_ep_schedule_rt_iso_out(udev, ep_ctx);
}

void usb_glue_notify_rt_iso_stopped(struct usb_device *udev, int endpoint)
{
    if (!udev || endpoint < 0 || endpoint >= USB_MAXENDPOINTS)
        return;

    struct ep_context *ep_ctx = &udev->ep_context[endpoint];
    struct IOUsbHWReq *stop_req = ep_ctx->rt_stop_pending;

    if (!stop_req)
        return;

    ep_ctx->rt_stop_pending = NULL;
    io_reply_data(udev, stop_req, UHIOERR_NO_ERROR, 0);
}

int usb_glue_start_rt_iso(struct XHCIUnit *unit, struct IOUsbHWReq *io)
{
    Kprintf("Starting RT ISO transfer\n");
    struct usb_device *udev = get_or_init_udev(unit, io->iouh_DevAddr);
    if (!udev)
        goto badparams;

    unsigned int endpoint = io->iouh_Endpoint & 0x0F;
    if (endpoint >= USB_MAXENDPOINTS)
        goto badparams;

    struct ep_context *ep_ctx = &udev->ep_context[endpoint];
    if (ep_ctx->state != USB_DEV_EP_STATE_RT_ISO_STOPPED)
    {
        Kprintf("EP not in RT_ISO_STOPPED\n");
        io->iouh_Req.io_Error = UHIOERR_HOSTERROR;
        return COMMAND_PROCESSED;
    }

    /* microframe_index is in 125us units; for FS frames use the frame number (divide by 8). */
    ep_ctx->rt_next_frame = (readl(unit->xhci_ctrl->run_regs->microframe_index) >> 3) & 0xffff;
    ep_ctx->state = USB_DEV_EP_STATE_RT_ISO_RUNNING;
    xhci_ep_schedule_rt_iso(udev, endpoint);

    io->iouh_Actual = 0;
    io->iouh_Req.io_Error = UHIOERR_NO_ERROR;
    return COMMAND_PROCESSED;

badparams:
    Kprintf("bad params\n");
    io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
    return COMMAND_PROCESSED;
}

int usb_glue_stop_rt_iso(struct XHCIUnit *unit, struct IOUsbHWReq *io)
{
    Kprintf("Stopping RT ISO transfer\n");
    struct usb_device *udev = get_or_init_udev(unit, io->iouh_DevAddr);
    if (!udev)
        goto badparams;

    unsigned int endpoint = io->iouh_Endpoint & 0x0F;
    if (endpoint >= USB_MAXENDPOINTS)
        goto badparams;

    struct ep_context *ep_ctx = &udev->ep_context[endpoint];
    if (ep_ctx->state != USB_DEV_EP_STATE_RT_ISO_RUNNING)
    {
        Kprintf("EP not in RT_ISO_RUNNING\n");
        io->iouh_Req.io_Error = UHIOERR_HOSTERROR;
        return COMMAND_PROCESSED;
    }

    if (ep_ctx->rt_req != io->iouh_Data)
        goto badparams;

    if (ep_ctx->rt_stop_pending)
    {
        Kprintf("STOPRTISO already pending\n");
        io->iouh_Req.io_Error = UHIOERR_HOSTERROR;
        return COMMAND_PROCESSED;
    }

    ep_ctx->rt_stop_pending = io;
    io->iouh_Req.io_Error = UHIOERR_NO_ERROR;

    BOOL no_inflight;
    ObtainSemaphore(&ep_ctx->active_tds_lock);
    no_inflight = minlist_empty(&ep_ctx->active_tds);
    ReleaseSemaphore(&ep_ctx->active_tds_lock);

    if (no_inflight)
    {
        ep_ctx->state = USB_DEV_EP_STATE_RT_ISO_STOPPED;
        usb_glue_notify_rt_iso_stopped(udev, endpoint);
    }
    else
    {
        ep_ctx->state = USB_DEV_EP_STATE_RT_ISO_STOPPING;
    }

    return COMMAND_SCHEDULED;

badparams:
    Kprintf("bad params\n");
    io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
    return COMMAND_PROCESSED;
}

void xhci_ep_schedule_next(struct usb_device *udev, int endpoint)
{
    if (endpoint < 0 || endpoint >= USB_MAXENDPOINTS)
        return;

    struct ep_context *ep_ctx = &udev->ep_context[endpoint];
    while (1)
    {
        struct MinNode *node = RemHeadMinList(&ep_ctx->pending_reqs);
        if (!node)
            return;

        struct IOUsbHWReq *req = (struct IOUsbHWReq *)node;
        req->iouh_DriverPrivate2 = NULL;

        KprintfH("starting queued request cmd=%ld ep=%ld\n",
             (LONG)req->iouh_Req.io_Command,
             (LONG)(req->iouh_Endpoint & 0x0F));
        dispatch_request(req);

        /* If the endpoint went idle immediately (e.g. zero-length or error), keep draining */
        if (ep_ctx->state == USB_DEV_EP_STATE_FAILED)
            return;

        if (req->iouh_DriverPrivate2 == XHCI_REQ_RING_BUSY)
            return;

        if (minlist_empty(&ep_ctx->pending_reqs))
            return;
    }
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
             LE16(io->iouh_SetupData.wValue),
             LE16(io->iouh_SetupData.wIndex),
             LE16(io->iouh_SetupData.wLength),
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

    /* Detect downstream port disconnects via hub GET_STATUS replies. */
    if ((io->iouh_SetupData.bRequest == USB_REQ_GET_STATUS) &&
        (io->iouh_SetupData.bmRequestType == (USB_DIR_IN | USB_RT_PORT)) &&
        io->iouh_Actual >= 4)
    {
        struct xhci_ctrl *ctrl = udev->controller;
        if (ctrl)
        {
            UWORD port = LE16(io->iouh_SetupData.wIndex);
            UWORD status = LE16(((UWORD *)io->iouh_Data)[0]);
            UWORD change = LE16(((UWORD *)io->iouh_Data)[1]);

            Kprintf("hub addr=%ld port=%ld status=%04lx change=%04lx\n", (LONG)udev->devnum, (LONG)port, (ULONG)status, (ULONG)change);

            /* Act only when the hub reports a connection change to avoid reacting to stale status. */
            if (change & USB_PORT_STAT_C_CONNECTION)
            {
                if (status & USB_PORT_STAT_CONNECTION)
                {
                    /* Remember parent/port for the next default-address attach without split info. */
                    Kprintf("hub addr=%ld port=%ld connected; remembering for pending attach\n", (LONG)udev->devnum, (LONG)port);
                    ctrl->pending_parent = udev;
                    ctrl->pending_parent_port = port;
                }

                if ((status & USB_PORT_STAT_CONNECTION) == 0)
                {
                    Kprintf("hub addr=%ld port=%ld disconnected; scanning children for match\n", (LONG)udev->devnum, (LONG)port);
                    struct usb_device *child = usb_glue_find_child_on_port(ctrl, udev, port);

                    if (child)
                    {
                        Kprintf("hub addr=%ld port=%ld disconnected, removing child addr=%ld slot=%ld\n",
                                (LONG)udev->devnum, (LONG)port, (LONG)child->devnum, (LONG)child->slot_id);
                        usb_glue_disconnect_device(ctrl, child, TRUE);
                    }
                    else
                    {
                        Kprintf("hub addr=%ld port=%ld disconnect: no child matched parent addr=%ld\n",
                                (LONG)udev->devnum, (LONG)port, (LONG)udev->devnum);
                    }
                }
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
            if (!ctrl)
                return;

            struct usb_device *current = ctrl->devices[old_addr];
            if (!current)
                current = udev;

            if (ctrl->devices[new_addr] && ctrl->devices[new_addr] != current)
            {
                Kprintf("overwriting existing ctx for addr %ld\n", (LONG)new_addr);
                usb_glue_free_udev_slot(ctrl, new_addr);
            }

            ctrl->devices[new_addr] = current;
            if (old_addr <= 127 && ctrl->devices[old_addr] == current)
                ctrl->devices[old_addr] = NULL;

            current->devnum = new_addr;
            current->used = 1;

            if (current->slot_id && ctrl->devs[current->slot_id])
                ctrl->devs[current->slot_id]->udev = current;

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

        /* Internal, reply-less requests (IOF_QUICK + magic tag) */
        if (io->iouh_DriverPrivate1 == (APTR)0xDEAD0001)
        {
            struct xhci_ctrl *ctrl = (struct xhci_ctrl *)io->iouh_DriverPrivate2;
            if (ctrl)
                FreeVecPooled(ctrl->memoryPool, io);
            return;
        }

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

    /* Internal, reply-less requests (IOF_QUICK + magic tag) */
    if (io->iouh_DriverPrivate1 == (APTR)0xDEAD0001)
    {
        struct xhci_ctrl *ctrl = (struct xhci_ctrl *)io->iouh_DriverPrivate2;
        if (ctrl)
            FreeVecPooled(ctrl->memoryPool, io);
        return;
    }

    ReplyMsg((struct Message *)io);
}

static void usb_glue_flush_udev(struct usb_device *udev)
{
    if (!udev)
        return;

    struct xhci_ctrl *ctrl = udev->controller;
    if (!ctrl)
        return;

    if (ctrl->root_int_req && ctrl->root_int_req->iouh_DevAddr == udev->devnum)
    {
        struct IOUsbHWReq *req = ctrl->root_int_req;
        ctrl->root_int_req = NULL;
        io_reply_failed(req, IOERR_ABORTED);
    }

    for (int ep = 0; ep < USB_MAXENDPOINTS; ++ep)
    {
        struct ep_context *ep_ctx = &udev->ep_context[ep];
        ep_teardown(ctrl, ep_ctx);

        if (ep_ctx->state != USB_DEV_EP_STATE_IDLE)
            xhci_ep_set_idle(udev, ep);
    }
}

void usb_glue_disconnect_device(struct xhci_ctrl *ctrl, struct usb_device *udev, BOOL recursive)
{
    if (!ctrl || !udev)
        return;

    if (!udev->slot_id)
        return;

    /* Disconnect downstream devices first so hubs drain their children before vanishing. */
    if (recursive)
    {
        for (int i = 0; i < MAX_HC_SLOTS; ++i)
        {
            struct usb_device *child = ctrl->devices[i];
            if (!child || child == udev)
                continue;

            if (child->parent == udev && child->used)
                usb_glue_disconnect_device(ctrl, child, TRUE);
        }
    }


    Kprintf("disconnect device addr=%ld slot=%ld port=%ld\n", (LONG)udev->devnum, (LONG)udev->slot_id, (LONG)udev->parent_port);

    xhci_disable_slot(udev);
}

static struct usb_device *usb_glue_find_child_on_port(struct xhci_ctrl *ctrl, struct usb_device *hub, unsigned int port)
{
    if (!ctrl || !hub)
        return NULL;

    for (int i = 0; i < MAX_HC_SLOTS; ++i)
    {
        struct usb_device *cand = ctrl->devices[i];
        if (!cand || cand == hub || cand->slot_id == 0)
            continue;

        if (cand->parent == hub && cand->parent_port == port)
            return cand;
    }

    return NULL;
}

void usb_glue_free_udev_slot(struct xhci_ctrl *ctrl, UWORD addr)
{
    if (!ctrl || addr > 127)
        return;

    struct usb_device *udev = ctrl->devices[addr];
    if (!udev)
        return;

    usb_glue_flush_udev(udev);

    struct MinNode *node;
    while ((node = RemHeadMinList(&udev->configurations)) != NULL)
    {
        struct usb_config *conf = (struct usb_config *)node;
        FreeVecPooled(ctrl->memoryPool, conf);
    }

    if (udev->slot_id && ctrl->devs[udev->slot_id])
        ctrl->devs[udev->slot_id]->udev = NULL;

    ctrl->devices[addr] = NULL;
    udev->used = 0;
    FreeVecPooled(ctrl->memoryPool, udev);
}

void usb_glue_flush_queues(struct XHCIUnit *unit)
{
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    if (!ctrl)
        return;
    for (int addr = 0; addr <= 127; ++addr)
    {
        struct usb_device *udev = ctrl->devices[addr];
        if (udev)
            usb_glue_flush_udev(udev);
    }
}