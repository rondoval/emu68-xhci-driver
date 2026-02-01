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
#include <xhci/xhci-commands.h>
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-usb.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-udev.h>

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
        return NULL;
    }

    _memset(udev, 0, sizeof(*udev));
    udev->poseidon_address = poseidon_address;
    udev->controller = ctrl;
    udev->speed = ctrl->pending_parent_speed;

    _NewMinList(&udev->configurations);
    xhci_ep_create_contexts(udev, ctrl->memoryPool);

    ctrl->devices_by_poseidon_address[poseidon_address] = udev;
    return udev;
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

    if (udev->slot_id && ctrl->devs[udev->slot_id])
        ctrl->devs[udev->slot_id]->udev = NULL;

    ctrl->devices_by_poseidon_address[udev->poseidon_address] = NULL;
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

    int ret = xhci_ctrl_tx(udev, io, timeout_ms);

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
        xhci_usb_parse_control_message(udev, io);
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

    int ret = xhci_ctrl_tx(udev, io, timeout_ms);
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
        return xhci_stream_tx(udev, req, timeout_ms,
                              TRB_TYPE(TRB_ISOC) | TRB_SIA, FALSE,
                              FALSE,
                              NULL);
    case UHCMD_BULKXFER:
        return xhci_stream_tx(udev, req, timeout_ms,
                              TRB_TYPE(TRB_NORMAL), TRUE,
                              FALSE,
                              NULL);
    case UHCMD_INTXFER:
    {
        struct xhci_ctrl *ctrl = unit->xhci_ctrl;
        if (udev->poseidon_address == xhci_roothub_get_address(ctrl->root_hub))
        {
            int result = xhci_roothub_submit_int_request(ctrl->root_hub, req);
            return result;
        }

        return xhci_stream_tx(udev, req, timeout_ms,
                              TRB_TYPE(TRB_NORMAL), TRUE,
                              FALSE,
                              NULL);
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

    if (io->iouh_Req.io_Command == UHCMD_CONTROLXFER && err == UHIOERR_NO_ERROR)
    {
        xhci_usb_parse_control_message(udev, io);
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
    io->iouh_Req.io_Flags = IOF_QUICK;          /* no reply port */
    io->iouh_DriverPrivate1 = (APTR)(REQ_INTERNAL | REQ_ENQUEUED); /* magic tag to free on completion */
    io->iouh_DriverPrivate2 = (APTR)ctrl;

    io->iouh_SetupData.bmRequestType = bmRequestType;
    io->iouh_SetupData.bRequest = bRequest;
    io->iouh_SetupData.wValue = cpu_to_le16(wValue);
    io->iouh_SetupData.wIndex = cpu_to_le16(wIndex);
    io->iouh_SetupData.wLength = cpu_to_le16(wLength);

    io->iouh_DevAddr = udev->poseidon_address;

    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
    xhci_ep_enqueue(ep_ctx, io);
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
