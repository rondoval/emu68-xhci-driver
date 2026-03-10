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
static void xhci_udev_translate_hub_descriptor_request(struct usb_device *udev, struct IOUsbHWReq *io);
static BOOL xhci_udev_fetch_hub_descriptor(struct usb_device *udev);

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
        xhci_udev_io_reply_failed(ctrl, req, reply_code);
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

static BOOL xhci_udev_filter_emulated_hub_ctrl_request(struct usb_device *udev, struct IOUsbHWReq *io)
{
    if (!udev || !io || !udev->ss_hub_emulation)
        return FALSE;

    struct UsbSetupData *setup = &io->iouh_SetupData;
    if (setup->bRequest != USB_REQ_CLEAR_FEATURE && setup->bRequest != USB_REQ_SET_FEATURE)
        return FALSE;

    if ((setup->bmRequestType & (USB_TYPE_MASK | USB_RECIP_MASK)) != (USB_TYPE_CLASS | USB_RECIP_OTHER))
        return FALSE;

    const u16 wValue = LE16(setup->wValue);
    const u8 portNo = LE16(setup->wIndex) & 0xFF;
    switch (wValue)
    {
    case USB_PORT_FEAT_SUSPEND:
        setup->wValue = LE16(USB_PORT_FEAT_LINK_STATE);
        const u8 link_state = (setup->bRequest == USB_REQ_CLEAR_FEATURE) ? XDEV_U0 : XDEV_U3;
        setup->wIndex = LE16(portNo | (link_state << 8));
        return FALSE;

    // these 3 are only for CLEAR_FEATURE
    case USB_PORT_FEAT_ENABLE:
        /* Can't disable USB 3.x port */
    case USB_PORT_FEAT_C_ENABLE: // this is only used for clear feature
        io->iouh_Actual = 0;
        io->iouh_Req.io_Error = UHIOERR_NO_ERROR;
        if (!(io->iouh_Flags & IOF_QUICK))
            ReplyMsg((struct Message *)io);
        return TRUE;

    case USB_PORT_FEAT_C_SUSPEND: // this is only used for clear feature
        setup->wValue = LE16(USB_SS_PORT_FEAT_C_LINK_STATE);
        return FALSE;

    default:
        return FALSE;
    }
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

    /* Translate USB 2.0 hub requests to SS format for SS hubs */
    xhci_udev_translate_hub_descriptor_request(udev, io);

    if (xhci_udev_filter_emulated_hub_ctrl_request(udev, io))
        return UHIOERR_NO_ERROR;

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
        /* For hubs, we need the hub descriptor BEFORE configuring so that
         * xhci_set_configuration() can program correct Number of Ports and
         * TT Think Time into the slot context.  EP0 is live (set up during
         * ADDRESS_DEVICE), so we fetch the hub descriptor first, then run
         * the full SET_CONFIGURATION + CONFIG_EP from the completion handler. */
        if (udev->is_hub)
        {
            udev->pending_set_config_req = io;
            if (xhci_udev_fetch_hub_descriptor(udev))
                return UHIOERR_NO_ERROR;
        }

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
void xhci_udev_io_reply_failed(struct xhci_ctrl *ctrl, struct IOUsbHWReq *io, int err)
{
    if (io)
    {
        io->iouh_Req.io_Error = err;

        /* Internal, reply-less requests (IOF_QUICK + magic tag) */
        if ((ULONG)io->iouh_DriverPrivate1 & REQ_INTERNAL)
        {
            if (ctrl)
                FreeVecPooled(ctrl->memoryPool, io);
            return;
        }

        KprintfH("addr %ld EP %ld err=%ld\n", (LONG)io->iouh_DevAddr, (LONG)io->iouh_Endpoint, (LONG)err);
        ReplyMsg((struct Message *)io);
    }
}

static void xhci_udev_handle_hub_prefetch(struct usb_device *udev, struct IOUsbHWReq *io)
{
    struct xhci_ctrl *ctrl = udev->controller;

    KprintfH("Hub descriptor pre-fetch done for addr=%ld (ports=%ld tt=%ld)\n",
             (LONG)udev->poseidon_address,
             (LONG)udev->ss_hub_desc.bNbrPorts,
             (LONG)udev->tt_think_time);

    if (ctrl && io->iouh_Data)
    {
        memalign_free(ctrl->memoryPool, io->iouh_Data);
        io->iouh_Data = NULL;
    }

    /* Retrieve the stashed Poseidon SET_CONFIGURATION IOReq */
    struct IOUsbHWReq *orig_req = udev->pending_set_config_req;
    udev->pending_set_config_req = NULL;

    /* Now run the deferred xhci_set_configuration — hub data is cached */
    UWORD config_value = LE16(orig_req->iouh_SetupData.wValue) & 0xff;
    int ret = xhci_set_configuration(udev, config_value);
    if (ret != UHIOERR_NO_ERROR)
    {
        Kprintf("Hub SET_CONFIGURATION failed after hub desc fetch\n");
        orig_req->iouh_Req.io_Error = ret;
        ReplyMsg((struct Message *)orig_req);
        return;
    }

    xhci_configure_endpoints(udev, FALSE, orig_req);
}

void xhci_udev_io_reply_data(struct usb_device *udev, struct IOUsbHWReq *io, int err, ULONG actual)
{
    if (!io || !udev)
        return;

    io->iouh_Actual = actual;
    io->iouh_Req.io_Error = err;

    if (io->iouh_Req.io_Command == UHCMD_CONTROLXFER && err == UHIOERR_NO_ERROR && io->iouh_Endpoint == 0)
        xhci_udev_parse_control_message(udev, io);

    KprintfH("err=%ld actual=%ld\n", (LONG)err, (LONG)actual);

    /* Internal, reply-less requests (IOF_QUICK + magic tag) */
    if ((ULONG)io->iouh_DriverPrivate1 & REQ_INTERNAL)
    {
        /* Hub descriptor pre-fetch completion: now run the full
         * SET_CONFIGURATION + CONFIG_EP with real hub data available.
         * xhci_udev_parse_control_message already ran above, so
         * handle_get_hub_descriptor cached ss_hub_desc + tt_think_time. */
        if ((ULONG)io->iouh_DriverPrivate1 & REQ_HUB_DESC_FETCH)
            xhci_udev_handle_hub_prefetch(udev, io);

        struct xhci_ctrl *ctrl = udev->controller;
        if (ctrl)
            FreeVecPooled(ctrl->memoryPool, io);
        return;
    }

    ReplyMsg((struct Message *)io);
}

/**
 * Issue an internal GET_DESCRIPTOR(hub) on EP0 before CONFIG_EP for hubs.
 * On completion, the hub descriptor data is cached and CONFIG_EP is issued
 * with the correct slot context fields.
 */
static BOOL xhci_udev_fetch_hub_descriptor(struct usb_device *udev)
{
    if (!udev || !udev->controller)
        return FALSE;

    struct xhci_ctrl *ctrl = udev->controller;

    /* Choose descriptor type based on device speed */
    const UBYTE desc_type = (udev->speed >= USB_SPEED_SUPER) ? USB_DT_SS_HUB : USB_DT_HUB;
    const UBYTE desc_len = (desc_type == USB_DT_SS_HUB) ? 12 : 9;

    const ULONG alloc_len = ALIGN(desc_len, ARCH_DMA_MINALIGN);
    UBYTE *buf = memalign(ctrl->memoryPool, ARCH_DMA_MINALIGN, alloc_len);
    struct IOUsbHWReq *io = AllocVecPooled(ctrl->memoryPool, sizeof(*io));
    if (!io || !buf)
    {
        Kprintf("xhci_udev_fetch_hub_descriptor: alloc failed, falling back to SET_CONFIGURATION without hub data\n");
        if (buf)
            memalign_free(ctrl->memoryPool, buf);
        if (io)
            FreeVecPooled(ctrl->memoryPool, io);
        udev->pending_set_config_req = NULL;
        return FALSE;
    }

    _memset(io, 0, sizeof(*io));
    io->iouh_Req.io_Command = UHCMD_CONTROLXFER;
    io->iouh_Req.io_Flags = IOF_QUICK;
    io->iouh_DriverPrivate1 = (APTR)(REQ_INTERNAL | REQ_ENQUEUED | REQ_HUB_DESC_FETCH);

    io->iouh_SetupData.bmRequestType = USB_DIR_IN | USB_RT_HUB;
    io->iouh_SetupData.bRequest = USB_REQ_GET_DESCRIPTOR;
    io->iouh_SetupData.wValue = cpu_to_le16((u16)(desc_type << 8));
    io->iouh_SetupData.wIndex = 0;
    io->iouh_SetupData.wLength = cpu_to_le16(desc_len);

    io->iouh_DevAddr = udev->poseidon_address;
    io->iouh_Data = buf;
    io->iouh_Length = desc_len;
    io->iouh_Dir = UHDIR_IN;

    KprintfH("Fetching hub descriptor (type=0x%02lx len=%ld) for addr=%ld before CONFIG_EP\n",
             (ULONG)desc_type, (LONG)desc_len, (LONG)udev->poseidon_address);

    /* Submit directly to the transfer ring — xhci_ep_enqueue only queues
     * for later and requires a completion event to drain, but EP0 may be
     * idle right now so nothing would ever kick the queue. */
    int ring_ret = xhci_ring_enqueue_td(udev, io, 1000, FALSE);
    if (ring_ret != UHIOERR_NO_ERROR)
    {
        Kprintf("xhci_udev_fetch_hub_descriptor: ring_enqueue_td failed (%ld), falling back\n", (LONG)ring_ret);
        memalign_free(ctrl->memoryPool, buf);
        FreeVecPooled(ctrl->memoryPool, io);
        udev->pending_set_config_req = NULL;
        return FALSE;
    }

    return TRUE;
}

static inline void xhci_udev_send_control_request(struct usb_device *udev, int ep_index,
                                                  UBYTE bmRequestType, UBYTE bRequest,
                                                  UWORD wValue, UWORD wIndex, UWORD wLength,
                                                  BOOL enqueue)
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
    if (enqueue)
        /* defer sending */
        xhci_ep_enqueue(ep_ctx, io);
    else
        xhci_ring_enqueue_td(udev, io, 1000, FALSE);
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
                                   USB_ENDPOINT_HALT /* wValue */,
                                   addr /* wIndex */,
                                   0 /* wLength */,
                                   TRUE /* enqueue */);
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
                                   0 /* wLength */,
                                   TRUE /* enqueue */);

    /* Control endpoints require clearing both directions. */
    if (ep_type == USB_ENDPOINT_XFER_CONTROL)
        xhci_udev_send_control_request(hub,
                                       0, /* ep_index 0 */
                                       USB_DIR_OUT | USB_RT_PORT,
                                       HUB_CLEAR_TT_BUFFER,
                                       devinfo ^ (1 << 15), /* toggle direction bit */
                                       (u16)udev->parent_port,
                                       0 /* wLength */,
                                       TRUE /* enqueue */);
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

    if (write < end)
        _memset(write, 0, end - write);

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
    case USB_PORT_STAT_HIGH_SPEED:
        return USB_SPEED_HIGH;
    case USB_PORT_STAT_LOW_SPEED:
        return USB_SPEED_LOW;
    default:
        return USB_SPEED_FULL;
    }
}

static enum usb_device_speed xhci_udev_speed_from_ss_port_status(u16 status)
{
    switch (status & USB_SS_PORT_STAT_SPEED)
    {
    case USB_SS_PORT_STAT_SPEED_LOW:
        return USB_SPEED_LOW;
    case USB_SS_PORT_STAT_SPEED_FULL:
        return USB_SPEED_FULL;
    case USB_SS_PORT_STAT_SPEED_HIGH:
        return USB_SPEED_HIGH;
    default:
        return USB_SPEED_SUPER;
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

static void xhci_udev_cache_ss_hub_descriptor(struct usb_device *udev, struct usb_hub_descriptor *hub, ULONG actual)
{
    if (!udev || !hub || actual < 4)
        return;

    UBYTE len = hub->bLength;
    if (len == 0 || len > actual)
        len = (UBYTE)min(actual, (ULONG)sizeof(struct usb_hub_descriptor));

    CopyMem(hub, &udev->ss_hub_desc, len);
    KprintfH("Cached SS hub descriptor for addr %ld with %ld ports\n",
             (LONG)udev->poseidon_address, (LONG)hub->bNbrPorts);
}

static void xhci_udev_set_ss_hub_depth(struct usb_device *udev)
{
    if (!udev || !udev->is_hub || !udev->ss_hub_emulation || udev->speed < USB_SPEED_SUPER)
        return;

    /* External hub only: root hub does not need this request. */
    if (!udev->parent)
        return;

    if (udev->ss_hub_depth_set)
        return;

    KprintfH("SS hub addr=%ld route=0x%lx -> SET_HUB_DEPTH depth=%ld\n",
             (LONG)udev->poseidon_address, (ULONG)udev->route, (LONG)udev->route_depth);

    xhci_udev_send_control_request(udev,
                                   0,
                                   (UBYTE)(USB_DIR_OUT | USB_RT_HUB),
                                   USB_REQ_SET_HUB_DEPTH,
                                   udev->route_depth /* wValue */,
                                   0 /* wIndex */,
                                   0 /* wLength */,
                                   FALSE /* enqueue */);

    udev->ss_hub_depth_set = TRUE;
}

static ULONG xhci_udev_build_usb2_hub_descriptor(struct usb_device *udev, UBYTE *buf, const ULONG max_len)
{
    if (!udev || !buf || max_len == 0)
        return 0;

    struct usb_hub_descriptor hub;
    _memset(&hub, 0, sizeof(hub));

    const UBYTE ports = udev->hub_num_ports;
    const UBYTE needed = min((ports + 1U + 7U) / 8U, sizeof(hub.u.hs.DeviceRemovable));

    hub.bLength = (UBYTE)(7U + 2U * needed);
    hub.bDescriptorType = USB_DT_HUB;
    hub.bNbrPorts = ports;

    hub.wHubCharacteristics = udev->ss_hub_desc.wHubCharacteristics;
    hub.bPwrOn2PwrGood = udev->ss_hub_desc.bPwrOn2PwrGood;
    hub.bHubContrCurrent = udev->ss_hub_desc.bHubContrCurrent;

    for (UBYTE i = 0; i < needed; ++i)
        hub.u.hs.PortPowerCtrlMask[i] = 0xFF;

    ULONG actual = min(max_len, (ULONG)hub.bLength);
    CopyMem(&hub, buf, actual);
    return actual;
}

/* Translate SS hub descriptor request: modify request to ask for SS descriptor,
 * it will be translated back to USB 2.0 in the parse handler */
static void xhci_udev_translate_hub_descriptor_request(struct usb_device *udev, struct IOUsbHWReq *io)
{
    if (!udev->ss_hub_emulation || !io || !io->iouh_Data || io->iouh_Length == 0)
        return;

    struct UsbSetupData *setup = &io->iouh_SetupData;
    const u8 descriptorType = (LE16(setup->wValue) >> 8) & 0xFF;
    const u16 typeReq = setup->bRequest | setup->bmRequestType << 8;

    /* Only translate GetHubDescriptor requests for USB_DT_HUB (Poseidon asking for USB 2.0) */
    if (typeReq != GetHubDescriptor || descriptorType != USB_DT_HUB)
        return;

    /* Modify the request to ask for SS hub descriptor instead */
    u16 old_value = LE16(setup->wValue);
    setup->wValue = cpu_to_le16((USB_DT_SS_HUB << 8) | (old_value & 0xFF));

    KprintfH("SS hub addr=%ld: modified wValue from 0x%04lx (USB_DT_HUB) to 0x%04lx (USB_DT_SS_HUB)\n",
             (LONG)udev->poseidon_address, (ULONG)old_value, (ULONG)LE16(setup->wValue));

    /* Return FALSE to let the request proceed normally - it will be translated back in parse */
    return;
}

static void xhci_udev_map_ss_port_status(u16 *wStatus, u16 *wChange, enum usb_device_speed speed)
{
    u16 wStatusNew = *wStatus & USB_SS_PORT_STAT_MASK;

    if ((*wStatus & PORT_PLS_MASK) == XDEV_U3)
    {
        KprintfH("SS hub: PLS=U3 detected, mapping to USB_PORT_STAT_SUSPEND\n");
        wStatusNew |= USB_PORT_STAT_SUSPEND;
    }
    if (*wStatus & USB_SS_PORT_STAT_POWER)
    {
        KprintfH("SS hub: POWER bit set, mapping to USB_PORT_STAT_POWER\n");
        wStatusNew |= USB_PORT_STAT_POWER;
    }

    switch (speed)
    {
    case USB_SPEED_LOW:
        KprintfH("SS hub: detected LowSpeed device, mapping to USB_PORT_STAT_LOW_SPEED\n");
        wStatusNew |= USB_PORT_STAT_LOW_SPEED;
        break;
    case USB_SPEED_FULL:
        KprintfH("SS hub: detected FullSpeed device\n");
        break;
    case USB_SPEED_HIGH:
        KprintfH("SS hub: detected HighSpeed device, mapping to USB_PORT_STAT_HIGH_SPEED\n");
        wStatusNew |= USB_PORT_STAT_HIGH_SPEED;
        break;
    default:
        KprintfH("SS hub: detected SuperSpeed device, mapping to USB_PORT_STAT_HIGH_SPEED for compatibility\n");
        wStatusNew |= USB_PORT_STAT_HIGH_SPEED;
        break;
    }

    KprintfH("SS hub: mapped status 0x%04lx -> 0x%04lx\n", (ULONG)*wStatus, (ULONG)wStatusNew);

    u16 wChangeNew = *wChange & (USB_PORT_STAT_C_CONNECTION | USB_PORT_STAT_C_OVERCURRENT | USB_PORT_STAT_C_RESET);

    /* Not needed, BH_RESET asserts C_RESET as well */
    // if (*wChange & USB_SS_PORT_STAT_C_BH_RESET)
    // {
    //     KprintfH("SS hub: C_BH_RESET detected, mapping to C_RESET\n");
    //     wChangeNew |= USB_PORT_STAT_C_RESET;
    // }

    if (*wChange & USB_SS_PORT_STAT_C_LINK_STATE && ((*wStatus & PORT_PLS_MASK) == XDEV_U0))
    {
        KprintfH("SS hub: C_LINK_STATE detected and PLS=U0, mapping to C_SUSPEND\n");
        wChangeNew |= USB_PORT_STAT_C_SUSPEND;
    }

    KprintfH("SS hub: mapped change 0x%04lx -> 0x%04lx\n", (ULONG)*wChange, (ULONG)wChangeNew);
    *wStatus = wStatusNew;
    *wChange = wChangeNew;
}

static void handle_get_device_descriptor(struct usb_device *udev, struct IOUsbHWReq *io)
{
    // We don't need full descriptor... just the max packet size to detect changes that require endpoint reconfiguration.
    if (!io->iouh_Data || io->iouh_Actual < 8)
        return;

    struct usb_device_descriptor *dev_desc = (struct usb_device_descriptor *)io->iouh_Data;
    KprintfH("Device Descriptor: bLength=%ld bDescriptorType=%ld bcdUSB=0x%04lx bDeviceClass=0x%02lx bDeviceSubClass=0x%02lx bDeviceProtocol=0x%02lx bMaxPacketSize0=%ld idVendor=0x%04lx idProduct=0x%04lx bcdDevice=0x%04lx iManufacturer=%ld iProduct=%ld iSerialNumber=%ld bNumConfigurations=%ld\n",
             (LONG)dev_desc->bLength,
             (LONG)dev_desc->bDescriptorType,
             (ULONG)LE16(dev_desc->bcdUSB),
             (LONG)dev_desc->bDeviceClass,
             (LONG)dev_desc->bDeviceSubClass,
             (LONG)dev_desc->bDeviceProtocol,
             (LONG)dev_desc->bMaxPacketSize0,
             (ULONG)LE16(dev_desc->idVendor),
             (ULONG)LE16(dev_desc->idProduct),
             (ULONG)LE16(dev_desc->bcdDevice),
             (LONG)dev_desc->iManufacturer,
             (LONG)dev_desc->iProduct,
             (LONG)dev_desc->iSerialNumber,
             (LONG)dev_desc->bNumConfigurations);

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

    if (dev_desc->bDeviceClass == USB_CLASS_HUB && (!udev->is_hub || !udev->ss_hub_emulation))
    {
        KprintfH("Device at addr=%ld is a hub\n", (LONG)udev->poseidon_address);
        udev->is_hub = TRUE;

        /* Only enable SS hub emulation for real external hubs, not the virtual root hub.
         * Root hub (parent == NULL) already provides port status in USB 2.0 format. */
        if (udev->speed >= USB_SPEED_SUPER && !udev->ss_hub_emulation)
        {
            KprintfH("Detected USB 3.0 hub at addr=%ld, enabling translation mode\n", (LONG)udev->poseidon_address);
            udev->ss_hub_emulation = TRUE;
        }
    }
}

static void handle_get_hub_descriptor(struct usb_device *udev, struct IOUsbHWReq *io, u8 descriptorType)
{
    if (!io->iouh_Data || io->iouh_Actual < 5)
        return;

    struct usb_hub_descriptor *hub = (struct usb_hub_descriptor *)io->iouh_Data;
    KprintfH("Hub Descriptor: bLength=%ld bDescriptorType=%ld bNbrPorts=%ld wHubCharacteristics=0x%04lx bPwrOn2PwrGood=%ld bHubContrCurrent=%ld\n",
             (LONG)hub->bLength,
             (LONG)hub->bDescriptorType,
             (LONG)hub->bNbrPorts,
             (ULONG)LE16(hub->wHubCharacteristics),
             (LONG)hub->bPwrOn2PwrGood,
             (LONG)hub->bHubContrCurrent);

    /* Update TT think time if changed */
    if (udev->parent)
    {
        const UWORD characteristics = LE16(hub->wHubCharacteristics);
        udev->tt_think_time = (u8)((characteristics >> 5) & 0x3);
        KprintfH("hub addr %ld TT think time code=%ld (bit-times=%ld)\n",
                 (LONG)udev->poseidon_address, (LONG)udev->tt_think_time, (LONG)((udev->tt_think_time + 1) * 8));
    }

    udev->hub_num_ports = hub->bNbrPorts;

    /* If this is an SS hub descriptor response, cache it */
    if (descriptorType == USB_DT_SS_HUB)
    {
        KprintfH("SS hub addr=%ld: caching USB3 hub descriptor (len=%ld)\n", (LONG)udev->poseidon_address, (LONG)io->iouh_Actual);
        xhci_udev_cache_ss_hub_descriptor(udev, hub, io->iouh_Actual);
        xhci_udev_set_ss_hub_depth(udev);

        /* If Poseidon requested USB 2.0 descriptor but we fetched SS, translate it */
        if (udev->ss_hub_emulation)
        {
            /* Build USB 2.0 descriptor from the SS descriptor we just cached */
            io->iouh_Actual = xhci_udev_build_usb2_hub_descriptor(udev, (UBYTE *)io->iouh_Data, io->iouh_Length);
            KprintfH("SS hub addr=%ld: translated USB3 descriptor to USB2 format. Size %ld bytes\n", (LONG)udev->poseidon_address, (LONG)io->iouh_Actual);
        }
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

    u16 wStatus = LE16(((u16 *)io->iouh_Data)[0]);
    u16 wChange = LE16(((u16 *)io->iouh_Data)[1]);
    /* Extract speed from the appropriate bit positions based on hub type */
    enum usb_device_speed speed = (udev->ss_hub_emulation) ? xhci_udev_speed_from_ss_port_status(wStatus) : xhci_udev_speed_from_port_status(wStatus);

    if (udev->ss_hub_emulation)
    {
        xhci_udev_map_ss_port_status(&wStatus, &wChange, speed);
        ((u16 *)io->iouh_Data)[0] = LE16(wStatus);
        ((u16 *)io->iouh_Data)[1] = LE16(wChange);
    }

    KprintfH("hub addr=%ld port=%ld status=%04lx change=%04lx\n", (LONG)udev->poseidon_address, (LONG)port, (ULONG)wStatus, (ULONG)wChange);

    /* Drop children immediately if port power is off or connection loss */
    if ((wStatus & USB_PORT_STAT_POWER) == 0 || ((wStatus & USB_PORT_STAT_CONNECTION) == 0))
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

    /* USB 3.0 port transitions to enabled automatically */
    if (speed == USB_SPEED_SUPER && (wChange & USB_PORT_STAT_C_CONNECTION) && (wStatus & USB_PORT_STAT_CONNECTION))
    {
        /* Remember parent/port for the next default-address attach without split info. */
        KprintfH("hub addr=%ld port=%ld speed=%ld connected; remembering for pending attach (status=%04lx)\n",
                 (LONG)udev->poseidon_address, (LONG)port, (LONG)speed, (ULONG)wStatus);
        ctrl->pending_parent = udev;
        ctrl->pending_parent_port = port;
        ctrl->pending_parent_speed = speed;
    }

    /* USB 2.0 enables device after reset completes */
    if ((wChange & USB_PORT_STAT_C_RESET) && (wStatus & (USB_PORT_STAT_CONNECTION | USB_PORT_STAT_ENABLE)))
    {
        KprintfH("hub addr=%ld port=%ld speed=%ld reset-complete; remembering for pending attach (status=%04lx)\n",
                 (LONG)udev->poseidon_address, (LONG)port, (LONG)speed, (ULONG)wStatus);
        ctrl->pending_parent = udev;
        ctrl->pending_parent_port = port;
        ctrl->pending_parent_speed = speed;
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
            handle_get_hub_descriptor(udev, io, descriptorType);
        break;

    case GetPortStatus:
        /* Detect downstream port disconnects via hub GET_STATUS replies. */
        handle_get_port_status(udev, io);
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
