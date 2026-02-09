// SPDX-License-Identifier: GPL-2.0+
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#include <clib/utility_protos.h>
#else
#include <proto/exec.h>
#include <proto/utility.h>
#endif

#include <devices/usbhardware.h>

#include <xhci/xhci.h>
#include <xhci/ch9.h>
#include <xhci/usb_defs.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-descriptors.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci-ring.h>
#include <xhci/xhci-context.h>

#include <device.h>
#include <debug.h>
#include <compat.h>
#include <minlist.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-udev] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci-udev] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

static void xhci_udev_parse_control_message(struct usb_device *udev, struct IOUsbHWReq *io);

struct usb_device *xhci_udev_alloc(struct xhci_ctrl *ctrl, UWORD poseidon_address)
{
    if (!ctrl || poseidon_address > USB_MAX_ADDRESS)
        return NULL;

    if (ctrl->devices_by_poseidon_address[poseidon_address])
        xhci_udev_free(ctrl->devices_by_poseidon_address[poseidon_address]);

    struct usb_device *udev = AllocVecPooled(ctrl->memoryPool, sizeof(*udev));
    if (!udev)
    {
        Kprintf("Failed to allocate usb_device for addr %ld\n", (LONG)poseidon_address);
        goto nothing;
    }

    _memset(udev, 0, sizeof(*udev));
    udev->poseidon_address = poseidon_address;
    udev->controller = ctrl;
    udev->speed = ctrl->pending_parent_speed;

    _NewMinList(&udev->configurations);

    /* Allocate the (output) device context that will be used in the HC. */
    udev->out_ctx = xhci_alloc_container_ctx(ctrl, XHCI_CTX_TYPE_DEVICE);
    if (!udev->out_ctx)
    {
        Kprintf("Failed to allocate out context for addr %ld\n", (LONG)poseidon_address);
        goto free_udev;
    }
    KprintfH("out_ctx bytes=%lx size=%ld\n",
             (ULONG)udev->out_ctx->bytes, (ULONG)udev->out_ctx->size);

    /* Allocate the (input) device context for address device command */
    udev->in_ctx = xhci_alloc_container_ctx(ctrl, XHCI_CTX_TYPE_INPUT);
    if (!udev->in_ctx)
    {
        Kprintf("Failed to allocate in context for addr %ld\n", (LONG)poseidon_address);
        goto destroy_out_ctx;
    }
    KprintfH("in_ctx bytes=%lx size=%ld\n",
             (ULONG)udev->in_ctx->bytes, (ULONG)udev->in_ctx->size);

    ctrl->devices_by_poseidon_address[poseidon_address] = udev;
    return udev;

destroy_out_ctx:
    xhci_free_container_ctx(ctrl, udev->out_ctx);
free_udev:
    FreeVecPooled(ctrl->memoryPool, udev);
nothing:
    return NULL;
}

struct usb_device *xhci_udev_get(struct XHCIUnit *unit, UWORD poseidon_address)
{
    if (!unit || !unit->xhci_ctrl || poseidon_address > USB_MAX_ADDRESS)
        return NULL;

    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    struct usb_device *udev = ctrl->devices_by_poseidon_address[poseidon_address];
    if (!udev)
    {
        // We'll be only creating contexts for newly detected devices
        if (poseidon_address != 0 && poseidon_address != xhci_roothub_get_address(ctrl->root_hub))
            return NULL;
        KprintfH("new device addr=%ld\n", (LONG)poseidon_address);
        udev = xhci_udev_alloc(ctrl, poseidon_address);
    }

    return udev;
}

static void xhci_udev_flush(struct usb_device *udev, UBYTE reply_code)
{
    if (!udev)
        return;

    struct xhci_ctrl *ctrl = udev->controller;
    if (!ctrl)
        return;
    KprintfH("flushing device addr=%ld slot=%ld\n", (LONG)udev->poseidon_address, (LONG)udev->slot_id);

    if (ctrl->root_int_req && ctrl->root_int_req->iouh_DevAddr == udev->poseidon_address)
    {
        struct IOUsbHWReq *req = ctrl->root_int_req;
        ctrl->root_int_req = NULL;
        xhci_udev_io_reply_failed(req, reply_code);
    }

    xhci_ep_destroy_contexts(udev, reply_code);
}

void xhci_udev_free(struct usb_device *udev)
{
    if (!udev)
        return;

    struct xhci_ctrl *ctrl = udev->controller;
    if (!ctrl)
        return;

    xhci_udev_flush(udev, UHIOERR_TIMEOUT);

    struct MinNode *node;
    while ((node = RemHeadMinList(&udev->configurations)) != NULL)
    {
        struct usb_config *conf = (struct usb_config *)node;
        FreeVecPooled(ctrl->memoryPool, conf);
    }

    if (udev->in_ctx)
        xhci_free_container_ctx(udev->controller, udev->in_ctx);
    if (udev->out_ctx)
        xhci_free_container_ctx(udev->controller, udev->out_ctx);

    ctrl->dcbaa->dev_context_ptrs[udev->slot_id] = 0;
    ctrl->devices_by_poseidon_address[udev->poseidon_address] = NULL;
    ctrl->devices_by_slot_id[udev->slot_id] = NULL;
    FreeVecPooled(ctrl->memoryPool, udev);
}

static UBYTE xhci_udev_find_epaddr_by_num(struct usb_device *udev, UBYTE epnum)
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

static void xhci_udev_patch_endpoint_address(struct usb_device *udev, struct IOUsbHWReq *io)
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

    UBYTE fixed = xhci_udev_find_epaddr_by_num(udev, epnum);
    if (!fixed || fixed == (UBYTE)wIndex)
        return;

    setup->wIndex = cpu_to_le16(fixed);

    KprintfH("Patched endpoint address wIndex from %02lx to %02lx for epnum %ld\n",
             (ULONG)wIndex, (ULONG)fixed, (LONG)epnum);
}

int xhci_udev_send_ctrl(struct usb_device *udev, struct IOUsbHWReq *io)
{
    if (!udev || !io)
    {
        Kprintf("no IO request provided?\n");
        return UHIOERR_BADPARAMS;
    }

    unsigned int timeout_ms = 0;
    if ((io->iouh_Flags & UHFF_NAKTIMEOUT))
        timeout_ms = io->iouh_NakTimeout;

    int ret = xhci_ring_enqueue_td(udev, io, timeout_ms, FALSE);

    return ret;
}

static int xhci_udev_send_ctrl_first(struct usb_device *udev, struct IOUsbHWReq *io, unsigned int timeout_ms)
{
    /* Work around class drivers that omit the direction bit in endpoint-recipient requests (e.g., UAC1 SET_CUR). */
    xhci_udev_patch_endpoint_address(udev, io);

    KprintfH("bmReqType=%02lx bReq=%02lx wValue=%04lx wIndex=%04lx wLength=%04lx\n",
             (ULONG)io->iouh_SetupData.bmRequestType,
             (ULONG)io->iouh_SetupData.bRequest,
             LE16(io->iouh_SetupData.wValue),
             LE16(io->iouh_SetupData.wIndex),
             LE16(io->iouh_SetupData.wLength));

    struct xhci_ctrl *ctrl = udev->controller;
    if (io->iouh_DevAddr == xhci_roothub_get_address(ctrl->root_hub))
    {
        xhci_roothub_submit_ctrl_request(ctrl->root_hub, io);
        xhci_udev_parse_control_message(udev, io);
        if (io->iouh_Req.io_Error != UHIOERR_NO_ERROR)
            return io->iouh_Req.io_Error;
        if (!(io->iouh_Flags & IOF_QUICK))
            ReplyMsg((struct Message *)io);
        return UHIOERR_NO_ERROR;
    }

    struct UsbSetupData *setup = &io->iouh_SetupData;

    if (setup->bRequest == USB_REQ_SET_ADDRESS && (setup->bmRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD)
    {
        xhci_address_device(udev, io);
        return UHIOERR_NO_ERROR;
    }

    if (setup->bRequest == USB_REQ_SET_CONFIGURATION &&
        (setup->bmRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD)
    {
        int ret = xhci_set_configuration(udev, LE16(setup->wValue) & 0xff);
        if (ret != UHIOERR_NO_ERROR)
        {
            Kprintf("Failed to configure xHCI endpoint\n");
            return ret;
        }

        // this will trigger a chain of commands and control xfer
        xhci_configure_endpoints(udev, FALSE, io);
        return UHIOERR_NO_ERROR;
    }

    /* If we don't have a slot yet, enable one and allocate Virt Dev */
    if (udev->slot_id == 0 && udev->poseidon_address == 0)
    {
        // this will store the req and submit it once addressed
        xhci_address_device(udev, io);
        return UHIOERR_NO_ERROR;
    }

    int ret = xhci_ring_enqueue_td(udev, io, timeout_ms, FALSE);
    return ret;
}

int xhci_udev_send(struct IOUsbHWReq *req)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)req->iouh_Req.io_Unit;
    if (!unit)
    {
        Kprintf("missing unit pointer (cmd=%ld, req=%lx, devaddr=%ld)\n",
                (LONG)req->iouh_Req.io_Command, (ULONG)req, (LONG)req->iouh_DevAddr);
        return UHIOERR_BADPARAMS;
    }

    struct usb_device *udev = xhci_udev_get(unit, req->iouh_DevAddr);
    if (!udev)
    {
        KprintfH("Device does not exist for addr %ld\n", (LONG)req->iouh_DevAddr);
        return UHIOERR_TIMEOUT;
    }

    unsigned int timeout_ms = 0;
    if ((req->iouh_Flags & UHFF_NAKTIMEOUT))
        timeout_ms = (unsigned int)req->iouh_NakTimeout;

    KprintfH("dev=%lx addr=%ld slot=%ld ep=%ld dir=%s len=%ld flags=%lx interval=%ld tmo=%lu\n",
             (ULONG)udev, (ULONG)udev->poseidon_address, (LONG)udev->slot_id,
             req->iouh_Endpoint & 0x0F, (req->iouh_Dir == UHDIR_IN) ? "IN" : "OUT",
             (LONG)req->iouh_Length, (ULONG)req->iouh_Flags,
             (LONG)req->iouh_Interval, (ULONG)timeout_ms);

    switch (req->iouh_Req.io_Command)
    {
    case UHCMD_CONTROLXFER:
        return xhci_udev_send_ctrl_first(udev, req, timeout_ms);
    case UHCMD_ISOXFER:
    case UHCMD_BULKXFER:
        return xhci_ring_enqueue_td(udev, req, timeout_ms, FALSE);
    case UHCMD_INTXFER:
    {
        struct xhci_ctrl *ctrl = unit->xhci_ctrl;
        if (udev->poseidon_address == xhci_roothub_get_address(ctrl->root_hub))
        {
            int result = xhci_roothub_submit_int_request(ctrl->root_hub, req);
            return result;
        }

        return xhci_ring_enqueue_td(udev, req, timeout_ms, FALSE);
    }
    default:
        Kprintf("unsupported command %ld (req=%lx, devaddr=%ld, endpoint=%ld, flags=0x%lx)\n",
                (LONG)req->iouh_Req.io_Command,
                (ULONG)req,
                (LONG)req->iouh_DevAddr,
                (LONG)req->iouh_Endpoint,
                (ULONG)req->iouh_Flags);
        return UHIOERR_BADPARAMS;
    }
}

/* Hooks for responding to requests for lower layer */
void xhci_udev_io_reply_failed(struct IOUsbHWReq *io, int err)
{
    if (io)
    {
        io->iouh_Req.io_Error = err;

        /* Internal, reply-less requests (IOF_QUICK + magic tag) */
        if ((ULONG)io->iouh_DriverPrivate1 & REQ_INTERNAL)
        {
            struct xhci_ctrl *ctrl = (struct xhci_ctrl *)io->iouh_DriverPrivate2;
            if (ctrl)
                FreeVecPooled(ctrl->memoryPool, io);
            return;
        }

        KprintfH("addr %ld EP %ld err=%ld\n", (LONG)io->iouh_DevAddr, (LONG)io->iouh_Endpoint, (LONG)err);
        ReplyMsg((struct Message *)io);
    }
}

void xhci_udev_io_reply_data(struct usb_device *udev, struct IOUsbHWReq *io, int err, ULONG actual)
{
    if (!io || !udev)
        return;

    io->iouh_Actual = actual;
    io->iouh_Req.io_Error = err;

    if (io->iouh_Req.io_Command == UHCMD_CONTROLXFER && err == UHIOERR_NO_ERROR && io->iouh_Endpoint == 0)
    {
        xhci_udev_parse_control_message(udev, io);
    }

    KprintfH("err=%ld actual=%ld\n", (LONG)err, (LONG)actual);

    /* Internal, reply-less requests (IOF_QUICK + magic tag) */
    if ((ULONG)io->iouh_DriverPrivate1 & REQ_INTERNAL)
    {
        struct xhci_ctrl *ctrl = (struct xhci_ctrl *)io->iouh_DriverPrivate2;
        if (ctrl)
            FreeVecPooled(ctrl->memoryPool, io);
        return;
    }

    ReplyMsg((struct Message *)io);
}

static inline void xhci_udev_send_control_request(struct usb_device *udev, int ep_index,
                                                  UBYTE bmRequestType, UBYTE bRequest,
                                                  UWORD wValue, UWORD wIndex, UWORD wLength)
{
    if (!udev || !udev->controller)
        return;

    struct xhci_ctrl *ctrl = udev->controller;
    struct IOUsbHWReq *io = AllocVecPooled(ctrl->memoryPool, sizeof(*io));
    if (!io)
        return;

    _memset(io, 0, sizeof(*io));
    io->iouh_Req.io_Command = UHCMD_CONTROLXFER;
    io->iouh_Req.io_Flags = IOF_QUICK;                             /* no reply port */
    io->iouh_DriverPrivate1 = (APTR)(REQ_INTERNAL | REQ_ENQUEUED); /* magic tag to free on completion */
    io->iouh_DriverPrivate2 = (APTR)ctrl;

    io->iouh_SetupData.bmRequestType = bmRequestType;
    io->iouh_SetupData.bRequest = bRequest;
    io->iouh_SetupData.wValue = cpu_to_le16(wValue);
    io->iouh_SetupData.wIndex = cpu_to_le16(wIndex);
    io->iouh_SetupData.wLength = cpu_to_le16(wLength);

    io->iouh_DevAddr = udev->poseidon_address;

    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
    if (!ep_ctx)
    {
        Kprintf("No ep context for ep index %d\n", ep_index);
        FreeVecPooled(ctrl->memoryPool, io);
        return;
    }
    xhci_ep_enqueue(ep_ctx, io);
}

inline static UBYTE xhci_ep_index_to_address(u32 ep_index)
{
    if (ep_index == 0)
        return 0;
    return EP_INDEX_TO_ENDPOINT(ep_index) | ((ep_index & 0x1) ? USB_DIR_OUT : USB_DIR_IN);
}

/* Issue an internal CLEAR_FEATURE(ENDPOINT_HALT) to endpoint (by ep_index) on udev. Fire-and-forget. */
void xhci_udev_clear_feature_halt(struct usb_device *udev, ULONG ep_index)
{
    if (!udev || !udev->controller || ep_index == 0)
        return;

    /* Convert ep_index (DCI-1) to USB endpoint address (number + direction bit). */
    UBYTE addr = xhci_ep_index_to_address(ep_index);

    xhci_udev_send_control_request(udev, ep_index,
                                   USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_ENDPOINT,
                                   USB_REQ_CLEAR_FEATURE,
                                   USB_ENDPOINT_HALT,
                                   addr,
                                   0);
}

/* Issue an internal CLEAR_TT_BUFFER to the parent hub for control/bulk endpoints behind a TT. */
void xhci_udev_clear_tt_buffer(struct usb_device *udev, ULONG ep_index, int ep_type)
{
    if (!udev || !udev->parent)
        return;

    /* Only applies to control or bulk endpoints behind a TT. */
    if (ep_type != USB_ENDPOINT_XFER_CONTROL && ep_type != USB_ENDPOINT_XFER_BULK)
        return;

    struct usb_device *hub = udev->parent;

    u16 epnum = (u16)EP_INDEX_TO_ENDPOINT(ep_index);
    BOOL out = (ep_index & 0x1) != 0;

    u16 devinfo = epnum;
    devinfo |= ((u16)udev->xhci_address) << 4;
    devinfo |= ((u16)ep_type) << 11;
    if (!out)
        devinfo |= 1 << 15;

    xhci_udev_send_control_request(hub,
                                   0, /* ep_index 0 */
                                   USB_DIR_OUT | USB_RT_PORT,
                                   HUB_CLEAR_TT_BUFFER,
                                   devinfo,
                                   (u16)udev->parent_port,
                                   0);

    /* Control endpoints require clearing both directions. */
    if (ep_type == USB_ENDPOINT_XFER_CONTROL)
        xhci_udev_send_control_request(hub,
                                       0, /* ep_index 0 */
                                       USB_DIR_OUT | USB_RT_PORT,
                                       HUB_CLEAR_TT_BUFFER,
                                       devinfo ^ (1 << 15), /* toggle direction bit */
                                       (u16)udev->parent_port,
                                       0);
}

/*
 * Descriptor access
 */

int xhci_ep_type_for_index(struct usb_device *udev, u32 ep_index)
{
    if (!udev)
        return -1;

    if (ep_index == 0)
        return USB_ENDPOINT_XFER_CONTROL;

    struct usb_config *cfg = udev->active_config;
    if (!cfg)
        return -1;

    UBYTE addr = xhci_ep_index_to_address(ep_index);

    for (int i = 0; i < cfg->no_of_if; ++i)
    {
        struct usb_interface *iface = &cfg->if_desc[i];
        struct usb_interface_altsetting *alt = iface->active_altsetting;
        if (!alt)
            continue;

        for (int e = 0; e < alt->no_of_ep; ++e)
        {
            struct usb_endpoint_descriptor *desc = &alt->ep_desc[e];
            if (desc->bEndpointAddress == addr)
                return desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
        }
    }

    return -1;
}

static void parse_config_descriptor(struct usb_device *udev, UBYTE *data, UWORD len)
{
    if (len < 2)
    {
        KprintfH("too short, len=%ld\n", (LONG)len);
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

    return;

error:
    FreeVecPooled(udev->controller->memoryPool, conf);
}

static void xhci_filter_ss_ep_companion_desc(struct IOUsbHWReq *io)
{
    if (!io->iouh_Data || io->iouh_Actual < sizeof(struct usb_config_descriptor))
        return;

    struct usb_config_descriptor *desc = (struct usb_config_descriptor *)io->iouh_Data;
    if (desc->bDescriptorType != USB_DT_CONFIG)
        return;

    UWORD total_len = LE16(desc->wTotalLength);
    if (total_len > io->iouh_Actual)
        total_len = io->iouh_Actual;

    UBYTE *read = io->iouh_Data + desc->bLength;
    UBYTE *write = read;
    UBYTE *end = io->iouh_Data + total_len;

    while (read + 2 <= end)
    {
        UBYTE dlen = read[0];
        UBYTE dtype = read[1];
        if (dlen == 0 || read + dlen > end)
            break;

        if (dtype != USB_DT_SS_ENDPOINT_COMP)
        {
            if (write != read)
            {
                for (UBYTE i = 0; i < dlen; ++i)
                    write[i] = read[i];
            }
            write += dlen;
        }

        read += dlen;
    }

    UWORD new_total = (UWORD)(write - (UBYTE *)io->iouh_Data);
    if (new_total != total_len)
        desc->wTotalLength = LE16(new_total);

    io->iouh_Actual = new_total;
}

static BOOL xhci_udev_iface_has_active_rt_iso(struct usb_device *udev, unsigned int iface_number)
{
    if (!udev || !udev->active_config)
        return FALSE;

    struct usb_config *cfg = udev->active_config;
    for (int i = 0; i < cfg->no_of_if; ++i)
    {
        struct usb_interface *iface = &cfg->if_desc[i];
        if (iface->interface_number != (UBYTE)iface_number)
            continue;

        struct usb_interface_altsetting *alt = iface->active_altsetting;
        if (!alt)
            return FALSE;

        for (int e = 0; e < alt->no_of_ep; ++e)
        {
            int ep_index = xhci_address_to_ep_index(&alt->ep_desc[e]);

            struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
            if (!ep_ctx)
                continue;
            enum ep_state state = xhci_ep_get_state(ep_ctx);
            if (state == USB_DEV_EP_STATE_RT_ISO_RUNNING ||
                state == USB_DEV_EP_STATE_RT_ISO_STOPPING)
                return TRUE;
        }

        return FALSE;
    }

    return FALSE;
}

static void xhci_udev_disconnect(struct usb_device *udev, BOOL recursive)
{
    if (!udev || !udev->slot_id)
        return;

    /* Disconnect downstream devices first so hubs drain their children before vanishing. */
    if (recursive)
    {
        struct xhci_ctrl *ctrl = udev->controller;

        for (int i = 0; i <= USB_MAX_ADDRESS; ++i)
        {
            struct usb_device *child = ctrl->devices_by_poseidon_address[i];
            if (!child || child == udev)
                continue;

            if (child->parent == udev)
                xhci_udev_disconnect(child, TRUE);
        }
    }

    KprintfH("disconnect device addr=%ld slot=%ld port=%ld\n",
             (LONG)udev->poseidon_address,
             (LONG)udev->slot_id,
             (LONG)udev->parent_port);

    xhci_disable_slot(udev);
}

static enum usb_device_speed xhci_udev_speed_from_port_status(UWORD status)
{
    switch (status & USB_PORT_STAT_SPEED_MASK)
    {
    case USB_PORT_STAT_SUPER_SPEED:
        return USB_SPEED_SUPER;
    case USB_PORT_STAT_HIGH_SPEED:
        return USB_SPEED_HIGH;
    case USB_PORT_STAT_LOW_SPEED:
        return USB_SPEED_LOW;
    default:
        return USB_SPEED_FULL;
    }
}

static struct usb_device *xhci_udev_find_child_on_port(struct usb_device *hub, unsigned int port)
{
    if (!hub)
        return NULL;

    struct xhci_ctrl *ctrl = hub->controller;
    for (int i = 0; i <= USB_MAX_ADDRESS; ++i)
    {
        struct usb_device *cand = ctrl->devices_by_poseidon_address[i];
        if (!cand || cand == hub || cand->slot_id == 0)
            continue;

        if (cand->parent == hub && cand->parent_port == port)
            return cand;
    }

    return NULL;
}

static void handle_get_device_descriptor(struct usb_device *udev, struct IOUsbHWReq *io)
{
    // We don't need full descriptor... just the max packet size to detect changes that require endpoint reconfiguration.
    if (!io->iouh_Data || io->iouh_Actual < 8)
        return;

    struct usb_device_descriptor *dev_desc = (struct usb_device_descriptor *)io->iouh_Data;

    // For full speed devices, max packet size may change once we read the device descriptor
    if (udev->speed == USB_SPEED_FULL)
        xhci_update_maxpacket(udev, dev_desc->bMaxPacketSize0);

    if (udev->speed == USB_SPEED_SUPER && dev_desc->bMaxPacketSize0 != 64)
    {
        KprintfH("clamping SS bMaxPacketSize0 from %ld to 64 for Poseidon\n", (LONG)dev_desc->bMaxPacketSize0);
        dev_desc->bMaxPacketSize0 = 64;
    }

    if (io->iouh_Actual >= sizeof(struct usb_device_descriptor))
        udev->product_string_index = dev_desc->iProduct;
}

static void handle_get_hub_descriptor(struct usb_device *udev, struct IOUsbHWReq *io)
{
    if (!io->iouh_Data || io->iouh_Actual < 5)
        return;

    struct usb_hub_descriptor *hub = (struct usb_hub_descriptor *)io->iouh_Data;
    UWORD characteristics = LE16(hub->wHubCharacteristics);
    u8 tt_think = (u8)((characteristics >> 5) & 0x3);
    if (udev->tt_think_time != tt_think)
    {
        udev->tt_think_time = tt_think;
        xhci_update_hub_tt(udev);
        KprintfH("hub addr %ld TT think time code=%ld (bit-times=%ld)\n",
                 (LONG)udev->poseidon_address, (LONG)tt_think, (LONG)((tt_think + 1) * 8));
    }
}

static void xhci_trim_string_descriptor(struct IOUsbHWReq *io)
{
    if (!io || !io->iouh_Data || io->iouh_Actual < 2)
        return;

    struct usb_string_descriptor *str_desc = (struct usb_string_descriptor *)io->iouh_Data;
    if (str_desc->bDescriptorType != USB_DT_STRING || io->iouh_Actual < str_desc->bLength || str_desc->bLength < 2)
        return;

    const u8 length = str_desc->bLength - 2;
    for (u8 i = 0; i < length; i += 2)
    {
        if (str_desc->bString[i] == 0 && str_desc->bString[i + 1] == 0)
            str_desc->bString[i] = 0x20; // replace embedded nulls with space
    }
}

static void xhci_append_ss_suffix(struct usb_device *udev, struct IOUsbHWReq *io)
{
    if (!udev || !io || !io->iouh_Data || io->iouh_Actual < 2)
        return;

    if (udev->speed < USB_SPEED_SUPER)
        return;

    struct usb_string_descriptor *str_desc = (struct usb_string_descriptor *)io->iouh_Data;
    if (str_desc->bDescriptorType != USB_DT_STRING)
        return;

    static const char suffix[] = " \0(\0S\0S\0)\0";
    static const int suffix_len = sizeof(suffix) - 1;

    if (str_desc->bLength > io->iouh_Actual)
    {
        // the descriptor is actually larger than the buffer used to receive it, just mock the length
        str_desc->bLength += suffix_len;
        return;
    }

    int length = str_desc->bLength;
    if (length < 2)
        return;
    length -= 2;

    for (int i = 0; i < suffix_len && length + i + 2 < (int)io->iouh_Length; ++i)
        str_desc->bString[length + i] = (u8)suffix[i];

    str_desc->bLength += suffix_len;
    io->iouh_Actual = min(str_desc->bLength, io->iouh_Length);
}

static void handle_get_port_status(struct usb_device *udev, struct IOUsbHWReq *io)
{
    if (!io->iouh_Data || io->iouh_Actual < 4)
        return;

    struct xhci_ctrl *ctrl = udev->controller;
    if (!ctrl)
        return;
    const u16 port = LE16(io->iouh_SetupData.wIndex);
    const u16 status = LE16(((u16 *)io->iouh_Data)[0]);
    const u16 change = LE16(((u16 *)io->iouh_Data)[1]);

    const enum usb_device_speed speed = xhci_udev_speed_from_port_status(status);

    KprintfH("hub addr=%ld port=%ld status=%04lx change=%04lx\n", (LONG)udev->poseidon_address, (LONG)port, (ULONG)status, (ULONG)change);

    /* Drop children immediately if port power is off or disabled, regardless of change bits. */
    if ((status & USB_PORT_STAT_POWER) == 0 || (status & USB_PORT_STAT_ENABLE) == 0)
    {
        KprintfH("hub addr=%ld port=%ld lost power or disabled; removing child if any\n", (LONG)udev->poseidon_address, (LONG)port);
        struct usb_device *child = xhci_udev_find_child_on_port(udev, port);
        if (child)
        {
            KprintfH("hub addr=%ld port=%ld power-off or disabled, removing child addr=%ld slot=%ld\n",
                     (LONG)udev->poseidon_address, (LONG)port, (LONG)child->poseidon_address, (LONG)child->slot_id);
            xhci_udev_disconnect(child, TRUE);
        }
    }

    /* Act when the hub reports a connection change to avoid reacting to stale status. */
    if (change & USB_PORT_STAT_C_CONNECTION)
    {
        if (status & (USB_PORT_STAT_CONNECTION | USB_PORT_STAT_ENABLE))
        {
            /* Remember parent/port for the next default-address attach without split info. */
            KprintfH("hub addr=%ld port=%ld connected; remembering for pending attach (status=%04lx)\n",
                     (LONG)udev->poseidon_address, (LONG)port, (ULONG)status);
            ctrl->pending_parent = udev;
            ctrl->pending_parent_port = port;
            ctrl->pending_parent_speed = speed;
        }

        if ((status & USB_PORT_STAT_CONNECTION) == 0)
        {
            KprintfH("hub addr=%ld port=%ld disconnected; scanning children for match\n", (LONG)udev->poseidon_address, (LONG)port);
            struct usb_device *child = xhci_udev_find_child_on_port(udev, port);
            if (child)
            {
                KprintfH("hub addr=%ld port=%ld disconnected, removing child addr=%ld slot=%ld\n",
                         (LONG)udev->poseidon_address, (LONG)port, (LONG)child->poseidon_address, (LONG)child->slot_id);
                xhci_udev_disconnect(child, TRUE);
            }
        }
    }

    /* Also remember parent/port after a reset-complete change when the link is up, even if
     * no explicit connection-change bit was set (common on root hub resets).
     */
    if ((change & USB_PORT_STAT_C_RESET) && (status & (USB_PORT_STAT_CONNECTION | USB_PORT_STAT_ENABLE)))
    {
        KprintfH("hub addr=%ld port=%ld reset-complete; remembering for pending attach (status=%04lx)\n",
                 (LONG)udev->poseidon_address, (LONG)port, (ULONG)status);
        ctrl->pending_parent = udev;
        ctrl->pending_parent_port = port;
        ctrl->pending_parent_speed = speed;
    }

    /* Poseidon expects USB2 speed bits; keep SS internally but report HS to Poseidon. */
    if (speed == USB_SPEED_SUPER)
    {
        u16 new_status = status & ~USB_PORT_STAT_SUPER_SPEED;
        new_status |= USB_PORT_STAT_HIGH_SPEED;
        ((u16 *)io->iouh_Data)[0] = LE16(new_status);
    }
}

static void handle_clear_feature(struct usb_device *udev, struct IOUsbHWReq *io)
{
    u16 feat = LE16(io->iouh_SetupData.wValue);
    if (feat != USB_PORT_FEAT_POWER && feat != USB_PORT_FEAT_ENABLE)
        return;

    u16 port = LE16(io->iouh_SetupData.wIndex);
    struct usb_device *child = xhci_udev_find_child_on_port(udev, port);
    if (child)
    {
        KprintfH("hub addr=%ld port=%ld CLEAR_FEATURE(%s); removing child addr=%ld slot=%ld\n",
                 (LONG)udev->poseidon_address, (LONG)port,
                 (feat == USB_PORT_FEAT_POWER) ? "PORT_POWER" : "PORT_ENABLE",
                 (LONG)child->poseidon_address, (LONG)child->slot_id);
        xhci_udev_disconnect(child, TRUE);
    }
}

static void handle_set_address(struct usb_device *udev, struct IOUsbHWReq *io)
{
    UWORD old_addr = io->iouh_DevAddr & 0x7F;
    UWORD new_addr = (UWORD)(LE16(io->iouh_SetupData.wValue) & 0x7F);
    if (new_addr == old_addr)
        return;

    struct xhci_ctrl *ctrl = udev->controller;
    if (!ctrl)
        return;

    if (old_addr == xhci_roothub_get_address(ctrl->root_hub))
        udev->speed = USB_SPEED_SUPER;

    struct usb_device *current = ctrl->devices_by_poseidon_address[old_addr];
    if (!current)
        current = udev;

    if (ctrl->devices_by_poseidon_address[new_addr] && ctrl->devices_by_poseidon_address[new_addr] != current)
    {
        Kprintf("overwriting existing ctx for addr %ld\n", (LONG)new_addr);
        /* If we are replacing an existing device (e.g., hub power-cycle), disconnect it (and children) first. */
        xhci_udev_disconnect(ctrl->devices_by_poseidon_address[new_addr], TRUE);
    }

    ctrl->devices_by_poseidon_address[new_addr] = current;
    if (ctrl->devices_by_poseidon_address[old_addr] == current)
        ctrl->devices_by_poseidon_address[old_addr] = NULL;

    current->poseidon_address = new_addr;

    KprintfH("migrated ctx from addr %ld to %ld\n", (LONG)old_addr, (LONG)new_addr);
}

static void handle_set_interface(struct usb_device *udev, struct IOUsbHWReq *io)
{
    unsigned int iface = (unsigned int)(LE16(io->iouh_SetupData.wIndex) & 0xFF);
    unsigned int alt = (unsigned int)(LE16(io->iouh_SetupData.wValue) & 0xFF);
    /*
     * This is a workaround for Poseidon issue.
     * Poseidon issues SET_INTERFACE for all devices after connecting a new one.
     * Thing is, it first sets the alternate setting to 0, then to the desired setting.
     * This causes issues with active RT ISO endpoints, as they get disabled on alt=0.
     */
    if (!xhci_udev_iface_has_active_rt_iso(udev, iface))
    {
        int err = xhci_set_interface(udev, iface, alt);
        if (err != UHIOERR_NO_ERROR)
        {
            Kprintf("SET_INTERFACE iface=%ld alt=%ld failed err=%ld\n",
                    (LONG)iface, (LONG)alt, (LONG)err);
        }
    }
    else
    {
        KprintfH("SET_INTERFACE iface=%ld alt=%ld ignored (RT ISO active)\n",
                 (LONG)iface, (LONG)alt);
    }
}

static void xhci_udev_parse_control_message(struct usb_device *udev, struct IOUsbHWReq *io)
{
    KprintfH("dev=%lx addr=%ld bmReqType=%02lx bReq=%02lx wValue=%04lx wIndex=%04lx wLength=%04lx actual=%ld\n",
             (ULONG)udev, (ULONG)udev->poseidon_address,
             (ULONG)io->iouh_SetupData.bmRequestType,
             (ULONG)io->iouh_SetupData.bRequest,
             LE16(io->iouh_SetupData.wValue),
             LE16(io->iouh_SetupData.wIndex),
             LE16(io->iouh_SetupData.wLength),
             (LONG)io->iouh_Actual);

    const u8 descriptorType = (LE16(io->iouh_SetupData.wValue) >> 8) & 0xFF;
    const u16 typeReq = io->iouh_SetupData.bRequest | io->iouh_SetupData.bmRequestType << 8;

    switch (typeReq)
    {
    case (DeviceRequest | USB_REQ_GET_DESCRIPTOR):
        switch (descriptorType)
        {
        case USB_DT_CONFIG:
            /* If this was a successful GET_DESCRIPTOR(CONFIGURATION),
             * cache the configuration descriptor for later use.
             */
            parse_config_descriptor(udev, (UBYTE *)io->iouh_Data, (UWORD)io->iouh_Actual);

            /* Poseidon doesn't like seeing SS companion descriptors */
            xhci_filter_ss_ep_companion_desc(io);
            break;
        case USB_DT_DEVICE:
            /* Update FS control endpoint max packet size based on device descriptor. */
            handle_get_device_descriptor(udev, io);
            break;
        case USB_DT_STRING:
        {
            const u16 string_index = LE16(io->iouh_SetupData.wValue) & 0xFF;
            if (string_index && string_index == udev->product_string_index)
            {
                xhci_trim_string_descriptor(io);
                xhci_append_ss_suffix(udev, io);
            }
            break;
        }
        }
        break;
    case GetHubDescriptor:
        /* Record TT think time from hub descriptors so child devices can be programmed correctly. */
        if (descriptorType == USB_DT_HUB || descriptorType == USB_DT_SS_HUB)
            handle_get_hub_descriptor(udev, io);
        break;

    case GetPortStatus:
        /* Detect downstream port disconnects via hub GET_STATUS replies. */
        handle_get_port_status(udev, io);
        break;

    case ClearPortFeature:
        /* Proactively disconnect when the host powers or enables a hub port off. */
        handle_clear_feature(udev, io);
        break;

    case (DeviceOutRequest | USB_REQ_SET_ADDRESS):
        /* If this was a successful standard SET_ADDRESS, migrate the glue context
         * from the old devaddr to the new one so subsequent transfers reuse the
         * same slot and endpoint state.
         * Note that this containt the address Poseidon _thinks_ it set on the interface.
         * In reality, XHCI selects the address.
         * Hence poseidon_address is what Poseidon uses; xhci_address is the real one.
         */
        handle_set_address(udev, io);
        break;

    case (InterfaceOutRequest | USB_REQ_SET_INTERFACE):
        handle_set_interface(udev, io);
        break;
    }
}
