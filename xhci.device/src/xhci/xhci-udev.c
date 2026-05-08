// SPDX-License-Identifier: GPL-2.0-only
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME (*(struct ExecBase **)4UL)
#include <proto/exec.h>
#endif

#include <devices/hcd_api.h>

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
#include <bits.h>
#include <byteorder.h>
#include <memory.h>
#include <minlist.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-udev] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci-udev] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

static void xhci_udev_parse_control_message(struct usb_device *udev, struct USBIORequest *io);
static void xhci_udev_translate_hub_descriptor_request(struct usb_device *udev, struct USBIORequest *io);
static BOOL xhci_udev_fetch_hub_descriptor(struct usb_device *udev);

struct usb_device *xhci_udev_alloc(struct xhci_ctrl *ctrl, u16 virtual_address)
{
    if (!ctrl || virtual_address > USB_MAX_ADDRESS)
        return NULL;

    if (ctrl->devices_by_virtual_address[virtual_address])
        xhci_udev_free(ctrl->devices_by_virtual_address[virtual_address]);

    struct usb_device *udev = pool_zalloc(ctrl->memoryPool, sizeof(*udev));
    if (!udev)
    {
        Kprintf("Failed to allocate usb_device for addr %lu\n", (ULONG)virtual_address);
        goto nothing;
    }

    udev->virtual_address = virtual_address;
    udev->controller = ctrl;
    udev->speed = ctrl->pending_parent_speed;

    _NewMinList(&udev->configurations);

    /* Allocate the (output) device context that will be used in the HC. */
    udev->out_ctx = xhci_alloc_container_ctx(ctrl, XHCI_CTX_TYPE_DEVICE);
    if (!udev->out_ctx)
    {
        Kprintf("Failed to allocate out context for addr %lu\n", (ULONG)virtual_address);
        goto free_udev;
    }
    KprintfH("out_ctx bytes=%lx size=%lu\n",
             (ULONG)udev->out_ctx->bytes, (ULONG)udev->out_ctx->size);

    /* Allocate the (input) device context for address device command */
    udev->in_ctx = xhci_alloc_container_ctx(ctrl, XHCI_CTX_TYPE_INPUT);
    if (!udev->in_ctx)
    {
        Kprintf("Failed to allocate in context for addr %lu\n", (ULONG)virtual_address);
        goto destroy_out_ctx;
    }
    KprintfH("in_ctx bytes=%lx size=%lu\n",
             (ULONG)udev->in_ctx->bytes, (ULONG)udev->in_ctx->size);

    ctrl->devices_by_virtual_address[virtual_address] = udev;
    return udev;

destroy_out_ctx:
    xhci_free_container_ctx(ctrl, udev->out_ctx);
free_udev:
    pool_free(ctrl->memoryPool, udev);
nothing:
    return NULL;
}

struct usb_device *xhci_udev_get(struct XHCIUnit *unit, u16 virtual_address)
{
    if (!unit || !unit->xhci_ctrl || virtual_address > USB_MAX_ADDRESS)
        return NULL;

    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    struct usb_device *udev = ctrl->devices_by_virtual_address[virtual_address];
    if (!udev)
    {
        // We'll be only creating contexts for newly detected devices
        if (virtual_address != 0 && virtual_address != xhci_roothub_get_address(ctrl->root_hub))
            return NULL;
        KprintfH("new device addr=%lu\n", (ULONG)virtual_address);
        udev = xhci_udev_alloc(ctrl, virtual_address);
    }

    return udev;
}

static void xhci_udev_flush(struct usb_device *udev, s8 reply_code)
{
    if (!udev)
        return;

    struct xhci_ctrl *ctrl = udev->controller;
    if (!ctrl)
        return;
    KprintfH("flushing device addr=%lu slot=%lu\n", (ULONG)udev->virtual_address, (ULONG)udev->slot_id);

    if (udev->virtual_address == xhci_roothub_get_address(ctrl->root_hub))
    {
        xhci_roothub_abort_int_request(ctrl->root_hub);
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

    xhci_udev_flush(udev, ERR_TIMEOUT);

    struct MinNode *node;
    while ((node = RemHeadMinList(&udev->configurations)) != NULL)
    {
        struct usb_config *conf = (struct usb_config *)node;
        pool_free(ctrl->memoryPool, conf);
    }

    if (udev->in_ctx)
        xhci_free_container_ctx(udev->controller, udev->in_ctx);
    if (udev->out_ctx)
        xhci_free_container_ctx(udev->controller, udev->out_ctx);

    ctrl->dcbaa->dev_context_ptrs[udev->slot_id] = 0;
    ctrl->devices_by_virtual_address[udev->virtual_address] = NULL;
    ctrl->devices_by_slot_id[udev->slot_id] = NULL;
    pool_free(ctrl->memoryPool, udev);
}

static u8 xhci_udev_find_epaddr_by_num(struct usb_device *udev, u8 epnum)
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
            u8 addr = alt->ep_desc[e].bEndpointAddress;
            if ((addr & 0x0F) == epnum)
                return addr;
        }
    }

    return 0;
}

static void xhci_udev_patch_endpoint_address(struct usb_device *udev, struct USBIORequest *io)
{
    struct USBSetupPacket *setup = &io->setup;

    /* Only patch class+endpoint recipient control requests. */
    if ((setup->bmRequestType & (USB_TYPE_MASK | USB_RECIP_MASK)) != (USB_TYPE_CLASS | USB_RECIP_ENDPOINT))
        return;

    /* If direction bit is already present, leave untouched. */
    u16 wIndex = le16(setup->wIndex);
    if (wIndex & 0x0080)
        return;

    u8 epnum = wIndex & 0x0F;
    if (epnum == 0)
        return;

    u8 fixed = xhci_udev_find_epaddr_by_num(udev, epnum);
    if (!fixed || fixed == (u8)wIndex)
        return;

    setup->wIndex = le16(fixed);

    KprintfH("Patched endpoint address wIndex from %02lx to %02lx for epnum %lu\n",
             (ULONG)wIndex, (ULONG)fixed, (ULONG)epnum);
}

static BOOL xhci_udev_filter_emulated_hub_ctrl_request(struct usb_device *udev, struct USBIORequest *io)
{
    if (!udev || !io || !udev->ss_hub_emulation)
        return FALSE;

    struct USBSetupPacket *setup = &io->setup;
    if (setup->bRequest != USB_REQ_CLEAR_FEATURE && setup->bRequest != USB_REQ_SET_FEATURE)
        return FALSE;

    if ((setup->bmRequestType & (USB_TYPE_MASK | USB_RECIP_MASK)) != (USB_TYPE_CLASS | USB_RECIP_OTHER))
        return FALSE;

    const u16 wValue = le16(setup->wValue);
    const u8 portNo = le16(setup->wIndex) & 0xFFU;
    switch (wValue)
    {
    case USB_PORT_FEAT_SUSPEND:
    {
        const u8 link_state = (setup->bRequest == USB_REQ_CLEAR_FEATURE) ? 0 : 3;
        setup->wValue = le16(USB_PORT_FEAT_LINK_STATE);
        setup->wIndex = le16(portNo | (link_state << 8));
        return FALSE;
    }

    // these 3 are only for CLEAR_FEATURE
    case USB_PORT_FEAT_ENABLE:
        /* Can't disable USB 3.x port */
    case USB_PORT_FEAT_C_ENABLE: // this is only used for clear feature
        io->actual_length = 0;
        io->req.io_Error = ERR_NO_ERROR;
        if (!(io->req.io_Flags & IOF_QUICK))
            ReplyMsg((struct Message *)io);
        return TRUE;

    case USB_PORT_FEAT_C_SUSPEND: // this is only used for clear feature
        setup->wValue = le16(USB_SS_PORT_FEAT_C_LINK_STATE);
        return FALSE;

    default:
        return FALSE;
    }
}

s8 xhci_udev_send_ctrl(struct usb_device *udev, struct USBIORequest *io)
{
    if (!udev || !io)
    {
        Kprintf("no IO request provided?\n");
        return ERR_BAD_PARAMETERS;
    }

    u32 timeout_ms = 0;
    if ((io->flags & DRIVER_FLAG_TIMEOUT_DEFINED))
        timeout_ms = io->timeout;

    s8 ret = xhci_ring_enqueue_td(udev, io, timeout_ms, FALSE);

    return ret;
}

static s8 xhci_udev_send_ctrl_first(struct usb_device *udev, struct USBIORequest *io, u32 timeout_ms)
{
    /* Work around class drivers that omit the direction bit in endpoint-recipient requests (e.g., UAC1 SET_CUR). */
    xhci_udev_patch_endpoint_address(udev, io);

    KprintfH("bmReqType=%02lx bReq=%02lx wValue=%04lx wIndex=%04lx wLength=%04lx\n",
             (ULONG)io->setup.bmRequestType,
             (ULONG)io->setup.bRequest,
             le16(io->setup.wValue),
             le16(io->setup.wIndex),
             le16(io->setup.wLength));

    /* Translate USB 2.0 hub requests to SS format for SS hubs */
    xhci_udev_translate_hub_descriptor_request(udev, io);

    if (xhci_udev_filter_emulated_hub_ctrl_request(udev, io))
        return ERR_NO_ERROR;

    struct xhci_ctrl *ctrl = udev->controller;
    if (io->virtual_address == xhci_roothub_get_address(ctrl->root_hub))
    {
        xhci_roothub_submit_ctrl_request(ctrl->root_hub, io);
        xhci_udev_parse_control_message(udev, io);
        if (io->req.io_Error != ERR_NO_ERROR)
            return io->req.io_Error;
        if (!(io->req.io_Flags & IOF_QUICK))
            ReplyMsg((struct Message *)io);
        return ERR_NO_ERROR;
    }

    struct USBSetupPacket *setup = &io->setup;
    if (setup->bRequest == USB_REQ_SET_ADDRESS && (setup->bmRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD)
    {
        xhci_address_device(udev, io);
        return ERR_NO_ERROR;
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
                return ERR_NO_ERROR;
        }

        s8 ret = xhci_set_configuration(udev, le16(setup->wValue) & 0xff);
        if (ret != ERR_NO_ERROR)
        {
            Kprintf("Failed to configure xHCI endpoint\n");
            return ret;
        }

        // this will trigger a chain of commands and control xfer
        xhci_configure_endpoints(udev, FALSE, io);
        return ERR_NO_ERROR;
    }

    /* If we don't have a slot yet, enable one and allocate Virt Dev */
    if (udev->slot_id == 0 && udev->virtual_address == 0)
    {
        // this will store the req and submit it once addressed
        xhci_address_device(udev, io);
        return ERR_NO_ERROR;
    }

    s8 ret = xhci_ring_enqueue_td(udev, io, timeout_ms, FALSE);
    return ret;
}

s8 xhci_udev_send(struct USBIORequest *req)
{
    struct XHCIUnit *unit = (struct XHCIUnit *)req->req.io_Unit;
    if (!unit)
    {
        Kprintf("missing unit pointer (cmd=%lu, req=%lx, devaddr=%lu)\n",
                (ULONG)req->req.io_Command, (ULONG)req, (ULONG)req->virtual_address);
        return ERR_BAD_PARAMETERS;
    }

    struct usb_device *udev = xhci_udev_get(unit, req->virtual_address);
    if (!udev)
    {
        KprintfH("Device does not exist for addr %lu\n", (ULONG)req->virtual_address);
        return ERR_TIMEOUT;
    }

    u32 timeout_ms = 0;
    if ((req->flags & DRIVER_FLAG_TIMEOUT_DEFINED))
        timeout_ms = req->timeout;

    KprintfH("dev=%lx addr=%lu slot=%lu ep=%lu dir=%s len=%lu flags=%lx tmo=%lu\n",
             (ULONG)udev, (ULONG)udev->virtual_address, (ULONG)udev->slot_id,
             (ULONG)(req->endpoint & 0x0F), (req->direction == DIRECTION_IN) ? "IN" : "OUT",
             (ULONG)req->data_buffer_length, (ULONG)req->flags,
             (ULONG)timeout_ms);

    switch (req->req.io_Command)
    {
    case CMD_REQUEST_CONTROL:
        return xhci_udev_send_ctrl_first(udev, req, timeout_ms);
    case CMD_REQUEST_ISOCHRONOUS:
    case CMD_REQUEST_BULK:
        return xhci_ring_enqueue_td(udev, req, timeout_ms, FALSE);
    case CMD_REQUEST_INTERRUPT:
    {
        struct xhci_ctrl *ctrl = unit->xhci_ctrl;
        if (udev->virtual_address == xhci_roothub_get_address(ctrl->root_hub))
        {
            s8 result = xhci_roothub_submit_int_request(ctrl->root_hub, req);
            return result;
        }

        return xhci_ring_enqueue_td(udev, req, timeout_ms, FALSE);
    }
    default:
        Kprintf("unsupported command %lu (req=%lx, devaddr=%lu, endpoint=%lu, flags=0x%lx)\n",
                (ULONG)req->req.io_Command,
                (ULONG)req,
                (ULONG)req->virtual_address,
                (ULONG)req->endpoint,
                (ULONG)req->flags);
        return ERR_BAD_PARAMETERS;
    }
}

/* Hooks for responding to requests for lower layer */
void xhci_udev_io_reply_failed(struct xhci_ctrl *ctrl, struct USBIORequest *io, s8 err)
{
    if (io)
    {
        io->req.io_Error = err;

        /* Internal, reply-less requests (IOF_QUICK + magic tag) */
        if (io->driver_private_flags & REQ_INTERNAL)
        {
            if (ctrl)
                pool_free(ctrl->memoryPool, io);
            return;
        }

        /* RT ISO clones were never sent as messages — never ReplyMsg.
         * IN clone staging buffers must be freed by callers (td_unmap_and_reply /
         * ep_handle_rt_iso / halted branch) before reaching here; data_buffer
         * should be NULL at this point. */
        if (io->driver_private_flags & REQ_RT_ISO_CLONE)
        {
            if (ctrl)
                slab_free(&ctrl->iso_clone_slab, io);

            return;
        }

        KprintfH("addr %lu EP %lu err=%ld\n", (ULONG)io->virtual_address, (ULONG)io->endpoint, (LONG)err);
        ReplyMsg((struct Message *)io);
    }
}

static void xhci_udev_handle_hub_prefetch(struct usb_device *udev, struct USBIORequest *io)
{
    struct xhci_ctrl *ctrl = udev->controller;

    KprintfH("Hub descriptor pre-fetch done for addr=%lu (ports=%lu tt=%lu)\n",
             (ULONG)udev->virtual_address,
             (ULONG)udev->ss_hub_desc.bNbrPorts,
             (ULONG)udev->tt_think_time);

    if (ctrl && io->data_buffer)
    {
        dma_free(ctrl->memoryPool, io->data_buffer);
        io->data_buffer = NULL;
    }

    /* Retrieve the stashed SET_CONFIGURATION IOReq */
    struct USBIORequest *orig_req = udev->pending_set_config_req;
    udev->pending_set_config_req = NULL;

    /* Now run the deferred xhci_set_configuration — hub data is cached */
    const u8 config_value = le16(orig_req->setup.wValue) & 0xffU;
    s8 ret = xhci_set_configuration(udev, config_value);
    if (ret != ERR_NO_ERROR)
    {
        Kprintf("Hub SET_CONFIGURATION failed after hub desc fetch\n");
        orig_req->req.io_Error = ret;
        ReplyMsg((struct Message *)orig_req);
        return;
    }

    xhci_configure_endpoints(udev, FALSE, orig_req);
}

void xhci_udev_io_reply_data(struct usb_device *udev, struct USBIORequest *io, s8 err, u32 actual)
{
    if (!io || !udev)
        return;

    io->actual_length = actual;
    io->req.io_Error = err;

    if (io->req.io_Command == CMD_REQUEST_CONTROL && err == ERR_NO_ERROR && io->endpoint == 0)
        xhci_udev_parse_control_message(udev, io);

    KprintfH("err=%ld actual=%lu\n", (LONG)err, (ULONG)actual);

    /* Internal, reply-less requests (IOF_QUICK + magic tag) */
    if (io->driver_private_flags & REQ_INTERNAL)
    {
        /* Hub descriptor pre-fetch completion: now run the full
         * SET_CONFIGURATION + CONFIG_EP with real hub data available.
         * xhci_udev_parse_control_message already ran above, so
         * handle_get_hub_descriptor cached ss_hub_desc + tt_think_time. */
        if (io->driver_private_flags & REQ_HUB_DESC_FETCH)
            xhci_udev_handle_hub_prefetch(udev, io);

        struct xhci_ctrl *ctrl = udev->controller;
        if (ctrl)
            pool_free(ctrl->memoryPool, io);
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
    const u8 desc_type = (udev->speed >= USB_SPEED_SUPER) ? USB_DT_SS_HUB : USB_DT_HUB;
    const u8 desc_len = (desc_type == USB_DT_SS_HUB) ? 12 : 9;

    const u32 alloc_len = ALIGN_UP((u32)desc_len, DMA_ALIGN_MIN);
    u8 *buf = dma_alloc(ctrl->memoryPool, DMA_ALIGN_MIN, alloc_len);
    struct USBIORequest *io = pool_zalloc(ctrl->memoryPool, sizeof(*io));
    if (!io || !buf)
    {
        Kprintf("xhci_udev_fetch_hub_descriptor: alloc failed, falling back to SET_CONFIGURATION without hub data\n");
        if (buf)
            dma_free(ctrl->memoryPool, buf);
        if (io)
            pool_free(ctrl->memoryPool, io);
        udev->pending_set_config_req = NULL;
        return FALSE;
    }

    io->req.io_Command = CMD_REQUEST_CONTROL;
    io->req.io_Flags = IOF_QUICK;
    io->driver_private_flags = REQ_INTERNAL | REQ_ENQUEUED | REQ_HUB_DESC_FETCH;

    io->setup.bmRequestType = USB_DIR_IN | USB_RT_HUB;
    io->setup.bRequest = USB_REQ_GET_DESCRIPTOR;
    io->setup.wValue = le16((u16)(desc_type << 8));
    io->setup.wIndex = 0;
    io->setup.wLength = le16(desc_len);

    io->virtual_address = udev->virtual_address;
    io->data_buffer = buf;
    io->data_buffer_length = desc_len;
    io->direction = DIRECTION_IN;

    KprintfH("Fetching hub descriptor (type=0x%02lx len=%lu) for addr=%lu before CONFIG_EP\n",
             (ULONG)desc_type, (ULONG)desc_len, (ULONG)udev->virtual_address);

    /* Submit directly to the transfer ring — xhci_ep_enqueue only queues
     * for later and requires a completion event to drain, but EP0 may be
     * idle right now so nothing would ever kick the queue. */
    s8 ring_ret = xhci_ring_enqueue_td(udev, io, 1000, FALSE);
    if (ring_ret != ERR_NO_ERROR)
    {
        Kprintf("xhci_udev_fetch_hub_descriptor: ring_enqueue_td failed (%ld), falling back\n", (LONG)ring_ret);
        dma_free(ctrl->memoryPool, buf);
        pool_free(ctrl->memoryPool, io);
        udev->pending_set_config_req = NULL;
        return FALSE;
    }

    return TRUE;
}

static inline void xhci_udev_send_control_request(struct usb_device *udev, u8 ep_index,
                                                  u8 bmRequestType, u8 bRequest,
                                                  u16 wValue, u16 wIndex, u16 wLength,
                                                  BOOL enqueue)
{
    if (!udev || !udev->controller)
        return;

    struct xhci_ctrl *ctrl = udev->controller;
    struct USBIORequest *io = pool_zalloc(ctrl->memoryPool, sizeof(*io));
    if (!io)
        return;

    io->req.io_Command = CMD_REQUEST_CONTROL;
    io->req.io_Flags = IOF_QUICK;                           /* no reply port */
    io->driver_private_flags = REQ_INTERNAL | REQ_ENQUEUED; /* magic tag to free on completion */

    io->setup.bmRequestType = bmRequestType;
    io->setup.bRequest = bRequest;
    io->setup.wValue = le16(wValue);
    io->setup.wIndex = le16(wIndex);
    io->setup.wLength = le16(wLength);

    io->virtual_address = udev->virtual_address;

    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
    if (!ep_ctx)
    {
        Kprintf("No ep context for ep index %d\n", ep_index);
        pool_free(ctrl->memoryPool, io);
        return;
    }
    if (enqueue)
        /* defer sending */
        xhci_ep_enqueue(ep_ctx, io);
    else
        xhci_ring_enqueue_td(udev, io, 1000, FALSE);
}

inline static u8 xhci_ep_index_to_address(u8 ep_index)
{
    if (ep_index == 0)
        return 0;
    return (u8)(EP_INDEX_TO_ENDPOINT(ep_index) | ((ep_index & 0x1U) ? USB_DIR_OUT : USB_DIR_IN));
}

/* Issue an internal CLEAR_FEATURE(ENDPOINT_HALT) to endpoint (by ep_index) on udev. Fire-and-forget. */
void xhci_udev_clear_feature_halt(struct usb_device *udev, u8 ep_index)
{
    if (!udev || !udev->controller || ep_index == 0)
        return;

    /* Convert ep_index (DCI-1) to USB endpoint address (number + direction bit). */
    u8 addr = xhci_ep_index_to_address(ep_index);

    xhci_udev_send_control_request(udev, ep_index,
                                   USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_ENDPOINT,
                                   USB_REQ_CLEAR_FEATURE,
                                   USB_ENDPOINT_HALT /* wValue */,
                                   addr /* wIndex */,
                                   0 /* wLength */,
                                   TRUE /* enqueue */);
}

/* Issue an internal CLEAR_TT_BUFFER to the parent hub for control/bulk endpoints behind a TT. */
void xhci_udev_clear_tt_buffer(struct usb_device *udev, u8 ep_index, int ep_type)
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
    devinfo |= (u16)((u16)udev->xhci_address << 4);
    devinfo |= (u16)((u16)ep_type << 11);
    if (!out)
        devinfo |= 1U << 15;

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

s32 xhci_ep_type_for_index(struct usb_device *udev, u8 ep_index)
{
    if (!udev)
        return -1;

    if (ep_index == 0)
        return USB_ENDPOINT_XFER_CONTROL;

    struct usb_config *cfg = udev->active_config;
    if (!cfg)
        return -1;

    u8 addr = xhci_ep_index_to_address(ep_index);

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

static void parse_config_descriptor(struct usb_device *udev, u8 *data, u16 len)
{
    if (len < 2)
    {
        KprintfH("too short, len=%lu\n", (ULONG)len);
        return;
    }

    struct usb_config *conf = pool_zalloc(udev->controller->memoryPool, sizeof(*conf));
    if (!conf)
    {
        Kprintf("pool_zalloc failed\n");
        return;
    }

    struct usb_config_descriptor *desc = (struct usb_config_descriptor *)data;
    if (desc->bDescriptorType != USB_DT_CONFIG)
    {
        Kprintf("bad desc type %lu\n", (ULONG)desc->bDescriptorType);
        goto error;
    }

    u16 total_len = le16(desc->wTotalLength);
    if (len < total_len)
    {
        KprintfH("short buffer len=%lu total_len=%lu\n", (ULONG)len, (ULONG)total_len);
        return;
    }

    u8 *cursor = data;
    u8 *end = data + total_len;
    if (cursor + desc->bLength > end)
    {
        Kprintf("bad desc length %lu\n", (ULONG)desc->bLength);
        goto error;
    }

    CopyMem(desc, &conf->desc, sizeof(struct usb_config_descriptor));
    cursor += desc->bLength;

    KprintfH("wTotalLength=%lu bNumInterfaces=%lu bConfigurationValue=%lu iConfiguration=%lu bmAttributes=0x%02lx bMaxPower=%lu\n",
             (ULONG)le16(desc->wTotalLength),
             (ULONG)desc->bNumInterfaces,
             (ULONG)desc->bConfigurationValue,
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
        u8 dlen = cursor[0];
        u8 dtype = cursor[1];
        if (dlen == 0)
        {
            Kprintf("zero length descriptor, aborting\n");
            break;
        }
        if (cursor + dlen > end)
        {
            Kprintf("descriptor overruns buffer (type=%lu len=%lu)\n", (ULONG)dtype, (ULONG)dlen);
            break;
        }

        switch (dtype)
        {
        case USB_DT_INTERFACE:
        {
            struct usb_interface_descriptor *ifd = (struct usb_interface_descriptor *)cursor;
            u32 iface_number = ifd->bInterfaceNumber;
            if (iface_number >= USB_MAXINTERFACES)
            {
                Kprintf("interface number %lu exceeds max %lu\n", (ULONG)iface_number, (ULONG)USB_MAXINTERFACES);
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
                    Kprintf("too many unique interfaces (%lu)\n", (ULONG)if_index);
                    goto error;
                }
                interface_map[iface_number] = if_index;
                current_if = &conf->if_desc[if_index];
                mem_zero(current_if, sizeof(struct usb_interface));
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
                Kprintf("too many alternate settings (%lu) for interface %lu\n",
                        (ULONG)current_if->num_altsetting, (ULONG)iface_number);
                current_alt = NULL;
                current_alt_index = -1;
                break;
            }

            current_alt_index = current_if->num_altsetting++;
            current_alt = &current_if->altsetting[current_alt_index];
            mem_zero(current_alt, sizeof(struct usb_interface_altsetting));

            CopyMem(ifd, &current_alt->desc, sizeof(struct usb_interface_descriptor));
            current_alt->no_of_ep = 0;

            KprintfH("interface %lu alt %lu: bInterfaceNumber=%lu bAlternateSetting=%lu bNumEndpoints=%lu bInterfaceClass=0x%02lx bInterfaceSubClass=0x%02lx bInterfaceProtocol=0x%02lx iInterface=%lu\n",
                     (ULONG)if_index,
                     (ULONG)current_alt_index,
                     (ULONG)ifd->bInterfaceNumber,
                     (ULONG)ifd->bAlternateSetting,
                     (ULONG)ifd->bNumEndpoints,
                     (ULONG)ifd->bInterfaceClass,
                     (ULONG)ifd->bInterfaceSubClass,
                     (ULONG)ifd->bInterfaceProtocol,
                     (ULONG)ifd->iInterface);

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
                Kprintf("too many endpoints for interface %lu alt %lu\n",
                        (ULONG)if_index, (ULONG)current_alt_index);
                break;
            }

            struct usb_endpoint_descriptor *epd = (struct usb_endpoint_descriptor *)cursor;
            u32 ep_idx = current_alt->no_of_ep;
            CopyMem(epd, &current_alt->ep_desc[ep_idx], sizeof(struct usb_endpoint_descriptor));
            KprintfH("  endpoint %lu: bEndpointAddress=0x%02lx bmAttributes=0x%02lx wMaxPacketSize=%lu bInterval=%lu\n",
                     (ULONG)ep_idx,
                     (ULONG)epd->bEndpointAddress,
                     (ULONG)epd->bmAttributes,
                     (ULONG)le16(epd->wMaxPacketSize),
                     (ULONG)epd->bInterval);

            current_alt->no_of_ep++;
            break;
        }
        case USB_DT_SS_ENDPOINT_COMP:
        {
            KprintfH("found SS EP COMP descriptor\n");
            if (current_if && current_alt && current_alt->no_of_ep > 0)
            {
                struct usb_ss_ep_comp_descriptor *comp = (struct usb_ss_ep_comp_descriptor *)cursor;
                u32 ep_slot = (u32)(current_alt->no_of_ep - 1U);
                CopyMem(comp, &current_alt->ss_ep_comp_desc[ep_slot], sizeof(struct usb_ss_ep_comp_descriptor));
            }
            break;
        }
        default:
            // Skip class- or vendor-specific descriptors gracefully.
            KprintfH("found class/vendor-specific descriptor 0x%lx, len=%lu\n", (ULONG)dtype, (ULONG)dlen);
            break;
        }

        cursor += dlen;
    }
    KprintfH("parsed config with %lu interfaces\n", (ULONG)conf->no_of_if);

    if (conf->no_of_if != desc->bNumInterfaces)
    {
        Kprintf("interface count mismatch %lu != %lu\n",
                (ULONG)conf->no_of_if, (ULONG)desc->bNumInterfaces);
        goto error;
    }

    for (struct MinNode *n = udev->configurations.mlh_Head; n->mln_Succ; n = n->mln_Succ)
    {
        struct usb_config *oldconf = (struct usb_config *)n;
        if (oldconf->desc.bConfigurationValue == conf->desc.bConfigurationValue)
        {
            KprintfH("removing old config with value %lu\n", (ULONG)oldconf->desc.bConfigurationValue);
            RemoveMinNode(n);
            pool_free(udev->controller->memoryPool, oldconf);
            break;
        }
    }
    AddHeadMinList(&udev->configurations, (struct MinNode *)conf);

    return;

error:
    pool_free(udev->controller->memoryPool, conf);
}

static void xhci_filter_ss_ep_companion_desc(struct USBIORequest *io)
{
    if (!io->data_buffer || io->actual_length < sizeof(struct usb_config_descriptor))
        return;

    struct usb_config_descriptor *desc = (struct usb_config_descriptor *)io->data_buffer;
    if (desc->bDescriptorType != USB_DT_CONFIG)
        return;

    u16 total_len = le16(desc->wTotalLength);
    if (total_len > io->actual_length)
        total_len = (u16)io->actual_length;

    u8 *read = io->data_buffer + desc->bLength;
    u8 *write = read;
    u8 *end = io->data_buffer + total_len;

    while (read + 2 <= end)
    {
        u8 dlen = read[0];
        u8 dtype = read[1];
        if (dlen == 0 || read + dlen > end)
            break;

        if (dtype != USB_DT_SS_ENDPOINT_COMP)
        {
            if (write != read)
            {
                for (u8 i = 0; i < dlen; ++i)
                    write[i] = read[i];
            }
            write += dlen;
        }

        read += dlen;
    }

    if (write < end)
        mem_zero(write, (ULONG)(end - write));

    u16 new_total = (u16)(write - (u8 *)io->data_buffer);
    if (new_total != total_len)
        desc->wTotalLength = le16(new_total);

    io->actual_length = new_total;
}

static BOOL xhci_udev_iface_has_active_rt_iso(struct usb_device *udev, u8 iface_number)
{
    if (!udev || !udev->active_config)
        return FALSE;

    struct usb_config *cfg = udev->active_config;
    for (int i = 0; i < cfg->no_of_if; ++i)
    {
        struct usb_interface *iface = &cfg->if_desc[i];
        if (iface->interface_number != iface_number)
            continue;

        struct usb_interface_altsetting *alt = iface->active_altsetting;
        if (!alt)
            return FALSE;

        for (int e = 0; e < alt->no_of_ep; ++e)
        {
            u8 ep_index = xhci_address_to_ep_index(&alt->ep_desc[e]);

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
            struct usb_device *child = ctrl->devices_by_virtual_address[i];
            if (!child || child == udev)
                continue;

            if (child->parent == udev)
                xhci_udev_disconnect(child, TRUE);
        }
    }

    KprintfH("disconnect device addr=%lu slot=%lu port=%lu\n",
             (ULONG)udev->virtual_address,
             (ULONG)udev->slot_id,
             (ULONG)udev->parent_port);

    xhci_disable_slot(udev);
}

static enum usb_device_speed xhci_udev_speed_from_port_status(u16 status)
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
    case USB_SS_PORT_STAT_SPEED_5GBPS:
        return USB_SPEED_SUPER;
    default:
        return USB_SPEED_UNKNOWN;
    }
}

static BOOL xhci_udev_ss_port_ready_for_attach(u16 status, enum usb_device_speed speed)
{
    return (status & USB_PORT_STAT_CONNECTION) != 0 &&
           (status & USB_PORT_STAT_ENABLE) != 0 &&
           (status & USB_PORT_STAT_RESET) == 0 &&
           speed != USB_SPEED_UNKNOWN;
}

static struct usb_device *xhci_udev_find_child_on_port(struct usb_device *hub, u32 port)
{
    if (!hub)
        return NULL;

    struct xhci_ctrl *ctrl = hub->controller;
    for (int i = 0; i <= USB_MAX_ADDRESS; ++i)
    {
        struct usb_device *cand = ctrl->devices_by_virtual_address[i];
        if (!cand || cand == hub || cand->slot_id == 0)
            continue;

        if (cand->parent == hub && cand->parent_port == port)
            return cand;
    }

    return NULL;
}

static void xhci_udev_cache_ss_hub_descriptor(struct usb_device *udev, struct usb_hub_descriptor *hub, u32 actual)
{
    if (!udev || !hub || actual < 4)
        return;

    u8 len = hub->bLength;
    if (len == 0 || len > actual)
        len = (u8)(actual < sizeof(struct usb_hub_descriptor) ? actual : sizeof(struct usb_hub_descriptor));

    CopyMem(hub, &udev->ss_hub_desc, len);
    KprintfH("Cached SS hub descriptor for addr %lu with %lu ports\n",
             (ULONG)udev->virtual_address, (ULONG)hub->bNbrPorts);
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

    KprintfH("SS hub addr=%lu route=0x%lx -> SET_HUB_DEPTH depth=%lu\n",
             (ULONG)udev->virtual_address, (ULONG)udev->route, (ULONG)udev->route_depth);

    xhci_udev_send_control_request(udev,
                                   0,
                                   USB_DIR_OUT | USB_RT_HUB,
                                   USB_REQ_SET_HUB_DEPTH,
                                   udev->route_depth /* wValue */,
                                   0 /* wIndex */,
                                   0 /* wLength */,
                                   FALSE /* enqueue */);

    udev->ss_hub_depth_set = TRUE;
}

static u32 xhci_udev_build_usb2_hub_descriptor(struct usb_device *udev, u8 *buf, const u32 max_len)
{
    if (!udev || !buf || max_len == 0)
        return 0;

    struct usb_hub_descriptor hub;
    mem_zero(&hub, sizeof(hub));

    const u8 ports = udev->hub_num_ports;
    u32 needed_words = ((u32)ports + 1U + 7U) / 8U;
    const u8 needed = (u8)(needed_words < sizeof(hub.u.hs.DeviceRemovable) ? needed_words : sizeof(hub.u.hs.DeviceRemovable));

    hub.bLength = (u8)(7U + 2U * needed);
    hub.bDescriptorType = USB_DT_HUB;
    hub.bNbrPorts = ports;

    hub.wHubCharacteristics = udev->ss_hub_desc.wHubCharacteristics;
    hub.bPwrOn2PwrGood = udev->ss_hub_desc.bPwrOn2PwrGood;
    hub.bHubContrCurrent = udev->ss_hub_desc.bHubContrCurrent;

    for (u8 i = 0; i < needed; ++i)
        hub.u.hs.PortPowerCtrlMask[i] = 0xFF;

    u32 actual = max_len < hub.bLength ? max_len : hub.bLength;
    CopyMem(&hub, buf, actual);
    return actual;
}

/* Translate SS hub descriptor request: modify request to ask for SS descriptor,
 * it will be translated back to USB 2.0 in the parse handler */
static void xhci_udev_translate_hub_descriptor_request(struct usb_device *udev, struct USBIORequest *io)
{
    if (!udev->ss_hub_emulation || !io || !io->data_buffer || io->data_buffer_length == 0)
        return;

    struct USBSetupPacket *setup = &io->setup;
    const u8 descriptorType = (le16(setup->wValue) >> 8) & 0xFFU;
    const u16 typeReq = (u16)(((u16)setup->bmRequestType << 8) | setup->bRequest);

    /* Only translate GetHubDescriptor requests for USB_DT_HUB */
    if (typeReq != GetHubDescriptor || descriptorType != USB_DT_HUB)
        return;

    /* Modify the request to ask for SS hub descriptor instead */
    u16 old_value = le16(setup->wValue);
    setup->wValue = le16((USB_DT_SS_HUB << 8) | (old_value & 0xFF));

    KprintfH("SS hub addr=%lu: modified wValue from 0x%04lx (USB_DT_HUB) to 0x%04lx (USB_DT_SS_HUB)\n",
             (ULONG)udev->virtual_address, (ULONG)old_value, (ULONG)le16(setup->wValue));

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

    if (*wChange & USB_SS_PORT_STAT_C_LINK_STATE && ((*wStatus & PORT_PLS_MASK) == XDEV_U0))
    {
        KprintfH("SS hub: C_LINK_STATE detected and PLS=U0, mapping to C_SUSPEND\n");
        wChangeNew |= USB_PORT_STAT_C_SUSPEND;
    }

    KprintfH("SS hub: mapped change 0x%04lx -> 0x%04lx\n", (ULONG)*wChange, (ULONG)wChangeNew);
    *wStatus = wStatusNew;
    *wChange = wChangeNew;
}

static void handle_get_device_descriptor(struct usb_device *udev, struct USBIORequest *io)
{
    // We don't need full descriptor... just the max packet size to detect changes that require endpoint reconfiguration.
    if (!io->data_buffer || io->actual_length < 8)
        return;

    struct usb_device_descriptor *dev_desc = (struct usb_device_descriptor *)io->data_buffer;
    KprintfH("Device Descriptor: bLength=%lu bDescriptorType=%lu bcdUSB=0x%04lx bDeviceClass=0x%02lx bDeviceSubClass=0x%02lx bDeviceProtocol=0x%02lx bMaxPacketSize0=%lu idVendor=0x%04lx idProduct=0x%04lx bcdDevice=0x%04lx iManufacturer=%lu iProduct=%lu iSerialNumber=%lu bNumConfigurations=%lu\n",
             (ULONG)dev_desc->bLength,
             (ULONG)dev_desc->bDescriptorType,
             (ULONG)le16(dev_desc->bcdUSB),
             (ULONG)dev_desc->bDeviceClass,
             (ULONG)dev_desc->bDeviceSubClass,
             (ULONG)dev_desc->bDeviceProtocol,
             (ULONG)dev_desc->bMaxPacketSize0,
             (ULONG)le16(dev_desc->idVendor),
             (ULONG)le16(dev_desc->idProduct),
             (ULONG)le16(dev_desc->bcdDevice),
             (ULONG)dev_desc->iManufacturer,
             (ULONG)dev_desc->iProduct,
             (ULONG)dev_desc->iSerialNumber,
             (ULONG)dev_desc->bNumConfigurations);

    // For full speed devices, max packet size may change once we read the device descriptor
    if (udev->speed == USB_SPEED_FULL)
        xhci_update_maxpacket(udev, dev_desc->bMaxPacketSize0);

    if (udev->speed == USB_SPEED_SUPER && dev_desc->bMaxPacketSize0 != 64)
    {
        KprintfH("clamping SS bMaxPacketSize0 from %lu to 64\n", (ULONG)dev_desc->bMaxPacketSize0);
        dev_desc->bMaxPacketSize0 = 64;
    }

    if (io->actual_length >= sizeof(struct usb_device_descriptor))
        udev->product_string_index = dev_desc->iProduct;

    if (dev_desc->bDeviceClass == USB_CLASS_HUB && (!udev->is_hub || !udev->ss_hub_emulation))
    {
        KprintfH("Device at addr=%lu is a hub\n", (ULONG)udev->virtual_address);
        udev->is_hub = TRUE;

        /* Only enable SS hub emulation for real external hubs, not the virtual root hub.
         * Root hub (parent == NULL) already provides port status in USB 2.0 format. */
        if (udev->speed >= USB_SPEED_SUPER && !udev->ss_hub_emulation)
        {
            KprintfH("Detected USB 3.0 hub at addr=%lu, enabling translation mode\n", (ULONG)udev->virtual_address);
            udev->ss_hub_emulation = TRUE;
        }
    }
}

static void handle_get_hub_descriptor(struct usb_device *udev, struct USBIORequest *io, u8 descriptorType)
{
    if (!io->data_buffer || io->actual_length < 5)
        return;

    struct usb_hub_descriptor *hub = (struct usb_hub_descriptor *)io->data_buffer;
    KprintfH("Hub Descriptor: bLength=%lu bDescriptorType=%lu bNbrPorts=%lu wHubCharacteristics=0x%04lx bPwrOn2PwrGood=%lu bHubContrCurrent=%lu\n",
             (ULONG)hub->bLength,
             (ULONG)hub->bDescriptorType,
             (ULONG)hub->bNbrPorts,
             (ULONG)le16(hub->wHubCharacteristics),
             (ULONG)hub->bPwrOn2PwrGood,
             (ULONG)hub->bHubContrCurrent);

    /* Update TT think time if changed */
    if (udev->parent)
    {
        const u16 characteristics = le16(hub->wHubCharacteristics);
        udev->tt_think_time = (u8)((characteristics >> 5) & 0x3);
        KprintfH("hub addr %lu TT think time code=%lu (bit-times=%lu)\n",
                 (ULONG)udev->virtual_address, (ULONG)udev->tt_think_time, (ULONG)((udev->tt_think_time + 1) * 8));
    }

    udev->hub_num_ports = hub->bNbrPorts;

    /* If this is an SS hub descriptor response, cache it */
    if (descriptorType == USB_DT_SS_HUB)
    {
        KprintfH("SS hub addr=%lu: caching USB3 hub descriptor (len=%lu)\n", (ULONG)udev->virtual_address, (ULONG)io->actual_length);
        xhci_udev_cache_ss_hub_descriptor(udev, hub, io->actual_length);
        xhci_udev_set_ss_hub_depth(udev);

        /* If the stack requested USB 2.0 descriptor but we fetched SS, translate it */
        if (udev->ss_hub_emulation)
        {
            /* Build USB 2.0 descriptor from the SS descriptor we just cached */
            io->actual_length = xhci_udev_build_usb2_hub_descriptor(udev, (u8 *)io->data_buffer, io->data_buffer_length);
            KprintfH("SS hub addr=%lu: translated USB3 descriptor to USB2 format. Size %lu bytes\n", (ULONG)udev->virtual_address, (ULONG)io->actual_length);
        }
    }
}

static void xhci_trim_string_descriptor(struct USBIORequest *io)
{
    if (!io || !io->data_buffer || io->actual_length < 2)
        return;

    struct usb_string_descriptor *str_desc = (struct usb_string_descriptor *)io->data_buffer;
    if (str_desc->bDescriptorType != USB_DT_STRING || io->actual_length < str_desc->bLength || str_desc->bLength < 2)
        return;

    const u32 length = str_desc->bLength - 2U;
    for (u32 i = 0; i < length; i += 2U)
    {
        if (str_desc->bString[i] == 0 && str_desc->bString[i + 1] == 0)
            str_desc->bString[i] = 0x20; // replace embedded nulls with space
    }
}

static void xhci_append_ss_suffix(struct usb_device *udev, struct USBIORequest *io)
{
    if (!udev || !io || !io->data_buffer || io->actual_length < 2)
        return;

    if (udev->speed < USB_SPEED_SUPER)
        return;

    struct usb_string_descriptor *str_desc = (struct usb_string_descriptor *)io->data_buffer;
    if (str_desc->bDescriptorType != USB_DT_STRING)
        return;

    static const char suffix[] = " \0(\0S\0S\0)\0";
    static const int suffix_len = sizeof(suffix) - 1;

    if (str_desc->bLength > io->actual_length)
    {
        // the descriptor is actually larger than the buffer used to receive it, just mock the length
        str_desc->bLength = (__le8)(str_desc->bLength + (u8)suffix_len);
        return;
    }

    int length = str_desc->bLength;
    if (length < 2)
        return;
    length -= 2;

    for (int i = 0; i < suffix_len && length + i + 2 < (int)io->data_buffer_length; ++i)
        str_desc->bString[length + i] = (u8)suffix[i];

    str_desc->bLength = (u8)(str_desc->bLength + (u8)suffix_len);
    io->actual_length = (u32)str_desc->bLength < io->data_buffer_length ? (u32)str_desc->bLength : io->data_buffer_length;
}

static void handle_get_port_status(struct usb_device *udev, struct USBIORequest *io)
{
    if (!io->data_buffer || io->actual_length < 4)
        return;

    struct xhci_ctrl *ctrl = udev->controller;
    if (!ctrl)
        return;
    const u8 port = le16(io->setup.wIndex) & 0xFFu;

    u16 wStatus = le16(((u16 *)io->data_buffer)[0]);
    u16 wChange = le16(((u16 *)io->data_buffer)[1]);
    const u16 rawStatus = wStatus;
    const u16 rawChange = wChange;
    /* Extract speed from the appropriate bit positions based on hub type */
    enum usb_device_speed speed = (udev->ss_hub_emulation) ? xhci_udev_speed_from_ss_port_status(rawStatus) : xhci_udev_speed_from_port_status(wStatus);

    if (udev->ss_hub_emulation)
    {
        xhci_udev_map_ss_port_status(&wStatus, &wChange, speed);
        ((u16 *)io->data_buffer)[0] = le16(wStatus);
        ((u16 *)io->data_buffer)[1] = le16(wChange);
    }

    KprintfH("hub addr=%lu port=%lu status=%04lx change=%04lx\n", (ULONG)udev->virtual_address, (ULONG)port, (ULONG)wStatus, (ULONG)wChange);

    /* Tear down any existing child as soon as the port is powered-but-disabled,
     * otherwise re-enumeration races the stale slot/context we still own. */
    const BOOL port_lost_child = ((wStatus & USB_PORT_STAT_POWER) == 0) ||
                                 ((wStatus & USB_PORT_STAT_CONNECTION) == 0) ||
                                 ((wStatus & USB_PORT_STAT_CONNECTION) != 0 &&
                                  (wStatus & USB_PORT_STAT_ENABLE) == 0 &&
                                  (wStatus & USB_PORT_STAT_RESET) == 0);

    if (port_lost_child)
    {
        KprintfH("hub addr=%lu port=%lu lost power, disconnected, or disabled; removing child if any\n",
                 (ULONG)udev->virtual_address, (ULONG)port);
        struct usb_device *child = xhci_udev_find_child_on_port(udev, port);
        if (child)
        {
            KprintfH("hub addr=%lu port=%lu tearing down child addr=%lu slot=%lu before re-enumeration\n",
                     (ULONG)udev->virtual_address, (ULONG)port, (ULONG)child->virtual_address, (ULONG)child->slot_id);
            xhci_udev_disconnect(child, TRUE);
        }
    }

    /* SS hub: use raw (pre-mapping) status to detect attach readiness.
     * USB 3.0 ports transition to enabled automatically after link training,
     * so we wait for connected + enabled + known speed before arming
     * pending_parent for the next SET_ADDRESS. */
    if (udev->ss_hub_emulation)
    {
        if (xhci_udev_ss_port_ready_for_attach(rawStatus, speed) &&
            (rawChange & (USB_PORT_STAT_C_CONNECTION |
                          USB_PORT_STAT_C_RESET |
                          USB_SS_PORT_STAT_C_BH_RESET |
                          USB_SS_PORT_STAT_C_LINK_STATE)))
        {
            KprintfH("hub addr=%lu port=%lu speed=%lu SS attach ready; remembering for pending attach (raw_status=%04lx)\n",
                     (ULONG)udev->virtual_address, (ULONG)port, (ULONG)speed, (ULONG)rawStatus);
            ctrl->pending_parent = udev;
            ctrl->pending_parent_port = port;
            ctrl->pending_parent_speed = speed;
        }
    }
    /* USB 2.0 enables device after reset completes */
    else if ((wChange & USB_PORT_STAT_C_RESET) && (wStatus & (USB_PORT_STAT_CONNECTION | USB_PORT_STAT_ENABLE)))
    {
        KprintfH("hub addr=%lu port=%lu speed=%lu reset-complete; remembering for pending attach (status=%04lx)\n",
                 (ULONG)udev->virtual_address, (ULONG)port, (ULONG)speed, (ULONG)wStatus);
        ctrl->pending_parent = udev;
        ctrl->pending_parent_port = port;
        ctrl->pending_parent_speed = speed;
    }
}

static void handle_set_address(struct usb_device *udev, struct USBIORequest *io)
{
    u16 old_addr = io->virtual_address & 0x7F;
    u16 new_addr = le16(io->setup.wValue) & 0x7F;
    if (new_addr == old_addr)
        return;

    struct xhci_ctrl *ctrl = udev->controller;
    if (!ctrl)
        return;

    struct usb_device *current = ctrl->devices_by_virtual_address[old_addr];
    if (!current)
        current = udev;

    if (ctrl->devices_by_virtual_address[new_addr] && ctrl->devices_by_virtual_address[new_addr] != current)
    {
        Kprintf("overwriting existing ctx for addr %lu\n", (ULONG)new_addr);
        /* If we are replacing an existing device (e.g., hub power-cycle), disconnect it (and children) first. */
        xhci_udev_disconnect(ctrl->devices_by_virtual_address[new_addr], TRUE);
    }

    ctrl->devices_by_virtual_address[new_addr] = current;
    if (ctrl->devices_by_virtual_address[old_addr] == current)
        ctrl->devices_by_virtual_address[old_addr] = NULL;

    current->virtual_address = new_addr;

    KprintfH("migrated ctx from addr %lu to %lu\n", (ULONG)old_addr, (ULONG)new_addr);
}

static void handle_set_interface(struct usb_device *udev, struct USBIORequest *io)
{
    u8 iface = le16(io->setup.wIndex) & 0xFFU;
    u8 alt = le16(io->setup.wValue) & 0xFFU;
    /*
     * This is a workaround for Poseidon issue.
     * Poseidon issues SET_INTERFACE for all devices after connecting a new one.
     * Thing is, it first sets the alternate setting to 0, then to the desired setting.
     * This causes issues with active RT ISO endpoints, as they get disabled on alt=0.
     */
    if (!xhci_udev_iface_has_active_rt_iso(udev, iface))
    {
        s8 err = xhci_set_interface(udev, iface, alt);
        if (err != ERR_NO_ERROR)
        {
            Kprintf("SET_INTERFACE iface=%lu alt=%lu failed err=%ld\n",
                    (ULONG)iface, (ULONG)alt, (LONG)err);
        }
    }
    else
    {
        KprintfH("SET_INTERFACE iface=%lu alt=%lu ignored (RT ISO active)\n",
                 (ULONG)iface, (ULONG)alt);
    }
}

static void xhci_udev_parse_control_message(struct usb_device *udev, struct USBIORequest *io)
{
    KprintfH("dev=%lx addr=%lu bmReqType=%02lx bReq=%02lx wValue=%04lx wIndex=%04lx wLength=%04lx actual=%lu\n",
             (ULONG)udev, (ULONG)udev->virtual_address,
             (ULONG)io->setup.bmRequestType,
             (ULONG)io->setup.bRequest,
             le16(io->setup.wValue),
             le16(io->setup.wIndex),
             le16(io->setup.wLength),
             (ULONG)io->actual_length);

    const u8 descriptorType = (le16(io->setup.wValue) >> 8) & 0xFFU;
    const u16 typeReq = (u16)(((u16)io->setup.bmRequestType << 8) | io->setup.bRequest);

    switch (typeReq)
    {
    case (DeviceRequest | USB_REQ_GET_DESCRIPTOR):
        switch (descriptorType)
        {
        case USB_DT_CONFIG:
            /* If this was a successful GET_DESCRIPTOR(CONFIGURATION),
             * cache the configuration descriptor for later use.
             */
            parse_config_descriptor(udev, (u8 *)io->data_buffer, (u16)io->actual_length);

            /* USB 2.0 stacks  don't like seeing SS companion descriptors */
            xhci_filter_ss_ep_companion_desc(io);
            break;
        case USB_DT_DEVICE:
            /* Update FS control endpoint max packet size based on device descriptor. */
            handle_get_device_descriptor(udev, io);
            break;
        case USB_DT_STRING:
        {
            const u16 string_index = le16(io->setup.wValue) & 0xFF;
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
         * Note that the API assumes it is the stack that assigns USB address.
         * In reality, XHCI selects the address.
         * Hence virtual_address is what the stack uses; xhci_address is the real one.
         */
        handle_set_address(udev, io);
        break;

    case (InterfaceOutRequest | USB_REQ_SET_INTERFACE):
        handle_set_interface(udev, io);
        break;
    }
}
