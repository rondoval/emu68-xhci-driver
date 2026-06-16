// SPDX-License-Identifier: GPL-2.0-only
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME (*(struct ExecBase **)4UL)
#include <proto/exec.h>
#endif

#include <exec/errors.h>
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
#include <xhci/xhci-lpm.h>
#include <xhci/xhci-hub.h>

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

/* Move udev to a new virtual address.  The sole writer of the migration, so
 * the invariant map[addr] == udev <=> udev->virtual_address == addr lives in
 * one place. */
static void xhci_udev_remap(struct xhci_ctrl *ctrl, struct usb_device *udev, u16 new_addr)
{
    if (ctrl->devices_by_virtual_address[udev->virtual_address] == udev)
        ctrl->devices_by_virtual_address[udev->virtual_address] = NULL;
    ctrl->devices_by_virtual_address[new_addr] = udev;
    udev->virtual_address = new_addr;
}

struct usb_device *xhci_udev_alloc(struct xhci_ctrl *ctrl, u16 virtual_address)
{
    if (!ctrl || virtual_address > USB_MAX_ADDRESS)
        return NULL;

    if (ctrl->devices_by_virtual_address[virtual_address])
        xhci_udev_free(ctrl->devices_by_virtual_address[virtual_address]);

    struct usb_device *udev = pool_zalloc(ctrl->metaPool, sizeof(*udev));
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
    pool_free(ctrl->metaPool, udev);
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
        pool_free(ctrl->metaPool, conf);
    }

    if (udev->in_ctx)
        xhci_free_container_ctx(udev->controller, udev->in_ctx);
    if (udev->out_ctx)
        xhci_free_container_ctx(udev->controller, udev->out_ctx);

    ctrl->dcbaa->dev_context_ptrs[udev->slot_id] = 0;
    ctrl->devices_by_virtual_address[udev->virtual_address] = NULL;
    ctrl->devices_by_slot_id[udev->slot_id] = NULL;
    pool_free(ctrl->metaPool, udev);
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

    u8 *buf = dma_alloc(ctrl->dmaPool, DMA_ALIGN_MIN, desc_len);
    struct USBIORequest *io = pool_zalloc(ctrl->metaPool, sizeof(*io));
    if (!io || !buf)
    {
        Kprintf("xhci_udev_fetch_hub_descriptor: alloc failed, falling back to SET_CONFIGURATION without hub data\n");
        if (buf)
            dma_free(ctrl->dmaPool, buf);
        if (io)
            pool_free(ctrl->metaPool, io);
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
        dma_free(ctrl->dmaPool, buf);
        pool_free(ctrl->metaPool, io);
        return FALSE;
    }

    return TRUE;
}

/**
 * Issue an internal GET_DESCRIPTOR(BOS, phase-1 header only) on EP0.
 * The caller stashes the request as UDEV_OP_CONFIGURE before calling; on failure
 * this leaves the stash intact so the caller can run the deferred config.
 * Returns TRUE if the fetch was submitted (SET_CONFIGURATION is deferred),
 * FALSE if the fetch could not be submitted (caller proceeds normally).
 */
static BOOL xhci_udev_fetch_bos(struct usb_device *udev)
{
    if (!udev || !udev->controller)
        return FALSE;

    struct xhci_ctrl *ctrl = udev->controller;

    u8 *buf = dma_alloc(ctrl->dmaPool, DMA_ALIGN_MIN, sizeof(struct usb_bos_descriptor));
    struct USBIORequest *io = pool_zalloc(ctrl->metaPool, sizeof(*io));
    if (!io || !buf)
    {
        Kprintf("xhci_udev_fetch_bos: alloc failed\n");
        if (buf)
            dma_free(ctrl->dmaPool, buf);
        if (io)
            pool_free(ctrl->metaPool, io);
        return FALSE;
    }

    io->req.io_Command = CMD_REQUEST_CONTROL;
    io->req.io_Flags = IOF_QUICK;
    io->driver_private_flags = REQ_INTERNAL | REQ_ENQUEUED | REQ_BOS_FETCH;

    io->setup.bmRequestType = USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE;
    io->setup.bRequest = USB_REQ_GET_DESCRIPTOR;
    io->setup.wValue = le16((u16)(USB_DT_BOS << 8));
    io->setup.wIndex = 0;
    io->setup.wLength = le16((u16)sizeof(struct usb_bos_descriptor));

    io->virtual_address = udev->virtual_address;
    io->data_buffer = buf;
    io->data_buffer_length = (u32)sizeof(struct usb_bos_descriptor);
    io->direction = DIRECTION_IN;

    KprintfH("Fetching BOS header for addr=%lu before CONFIG_EP\n",
             (ULONG)udev->virtual_address);

    s8 ring_ret = xhci_ring_enqueue_td(udev, io, 1000, FALSE);
    if (ring_ret != ERR_NO_ERROR)
    {
        Kprintf("xhci_udev_fetch_bos: ring_enqueue_td failed (%ld)\n", (LONG)ring_ret);
        dma_free(ctrl->dmaPool, buf);
        pool_free(ctrl->metaPool, io);
        return FALSE;
    }

    return TRUE;
}

/* Hooks for responding to requests for lower layer */
void xhci_udev_io_reply_failed(struct xhci_ctrl *ctrl, struct USBIORequest *io, s8 err)
{
    if (io)
    {
        io->req.io_Error = err;

        /* Internal, reply-less requests (IOF_QUICK + magic tag).  Free the
         * transfer's payload buffer and the request struct, but do NOT resume
         * any deferred work from here: a BOS/hub pre-fetch that STALLs leaves
         * EP0 halted, and resuming the stashed SET_CONFIGURATION before EP0 has
         * been recovered just re-STALLs the SET_CONFIGURATION transfer in an
         * endless loop.  the UDEV_OP_CONFIGURE stash stays put and is
         * resumed from handle_set_deq() once the EP0 reset/stop recovery
         * completes. */
        if (io->driver_private_flags & REQ_INTERNAL)
        {
            /* Internal EP0 requests are fire-and-forget; without this line a
             * device rejecting e.g. SET_FEATURE(U1/U2_ENABLE) or SET_SEL would
             * be invisible.  But "device gone" (ERR_TIMEOUT) and "transfer
             * cancelled" (IOERR_ABORTED) are the expected outcome when a hub or
             * device is torn down: in-flight clear-halt / CLEAR_TT_BUFFER
             * recovery races the physical removal and floods the log.  Keep
             * those at debug level; a genuine device-level reject (STALL /
             * Request Error) never reports timeout/aborted, so it stays loud. */
            if (err == ERR_TIMEOUT || err == IOERR_ABORTED)
                KprintfH("internal EP0 request retired (device gone): addr %lu bmReqType=%02lx bReq=%02lx wValue=%lu err=%ld\n",
                         (ULONG)io->virtual_address, (ULONG)io->setup.bmRequestType,
                         (ULONG)io->setup.bRequest, (ULONG)le16(io->setup.wValue), (LONG)err);
            else
                Kprintf("internal EP0 request failed: addr %lu bmReqType=%02lx bReq=%02lx wValue=%lu err=%ld\n",
                        (ULONG)io->virtual_address, (ULONG)io->setup.bmRequestType,
                        (ULONG)io->setup.bRequest, (ULONG)le16(io->setup.wValue), (LONG)err);

            /* A rejected SET_SEL ends the LPM sequence (no device-initiated
             * U1/U2); free the op slot. */
            if ((io->driver_private_flags & REQ_SET_SEL) && ctrl)
            {
                struct usb_device *udev = ctrl->devices_by_virtual_address[io->virtual_address];
                if (udev)
                    xhci_udev_op_cancel(udev, UDEV_OP_LPM_ENABLE, ERR_NO_ERROR);
            }
            if (ctrl && io->data_buffer)
                dma_free(ctrl->dmaPool, io->data_buffer);
            if (ctrl)
                pool_free(ctrl->metaPool, io);
            return;
        }

        KprintfH("addr %lu EP %lu err=%ld\n", (ULONG)io->virtual_address, (ULONG)io->endpoint, (LONG)err);
        ReplyMsg((struct Message *)io);
    }
}

static void xhci_udev_handle_hub_prefetch(struct usb_device *udev, struct USBIORequest *io)
{
    struct xhci_ctrl *ctrl = udev->controller;

    /* ss_hub_desc was cached by xhci_udev_parse_control_message before we got
     * here; mark it so a repeat SET_CONFIGURATION skips the re-fetch (and, for
     * SS hubs, the chained BOS fetch below). */
    udev->hub_desc_fetched = TRUE;

    KprintfH("Hub descriptor pre-fetch done for addr=%lu (ports=%lu tt=%lu)\n",
             (ULONG)udev->virtual_address,
             (ULONG)udev->ss_hub_desc.bNbrPorts,
             (ULONG)udev->tt_think_time);

    if (ctrl && io->data_buffer)
    {
        dma_free(ctrl->dmaPool, io->data_buffer);
        io->data_buffer = NULL;
    }

    /* SS hubs: chain a BOS fetch so the hub's own U1/U2 exit latencies are
     * known — they are needed to compute Max Exit Latency for any downstream
     * device.  the UDEV_OP_CONFIGURE stash stays put and the BOS completion
     * (or EP0-recovery) path runs the deferred SET_CONFIGURATION. */
    if (udev->speed >= USB_SPEED_SUPER && xhci_udev_fetch_bos(udev))
        return;

    /* Hub data is cached (or the BOS fetch wasn't submitted): run the deferred
     * SET_CONFIGURATION now. */
    xhci_udev_run_pending_set_config(udev);
}

/* Reset the per-device multi-step operation slot to idle. */
static void xhci_udev_op_clear(struct usb_device *udev)
{
    udev->op.op = UDEV_OP_NONE;
    udev->op.step = 0;
    udev->op.waits = 0;
    udev->op.arg = 0;
    udev->op.stash = NULL;
}

/* Program the configuration and issue CONFIG_EP for req (the stashed
 * SET_CONFIGURATION travels onward as the command's completion request). */
static void xhci_udev_do_set_config(struct usb_device *udev, struct USBIORequest *req)
{
    const u8 config_value = (u8)(le16(req->setup.wValue) & 0xffU);
    s8 ret = xhci_set_configuration(udev, config_value);
    if (ret != ERR_NO_ERROR)
    {
        Kprintf("SET_CONFIGURATION failed after pre-fetch\n");
        req->req.io_Error = ret;
        ReplyMsg((struct Message *)req);
        return;
    }

    xhci_configure_endpoints(udev, FALSE, req);
}

/**
 * Run the device SET_CONFIGURATION that was deferred behind a BOS/hub pre-fetch.
 * Consumes the UDEV_OP_CONFIGURE stash.  Must only be called once EP0 is idle
 * and its ring is clean — i.e. from the pre-fetch success path, or from
 * handle_set_deq() after an EP0 STALL/timeout recovery has completed
 * (UDEV_OP_EVENT_EP0_RECOVERED).
 */
void xhci_udev_run_pending_set_config(struct usb_device *udev)
{
    if (udev->op.op != UDEV_OP_CONFIGURE)
        return;

    struct USBIORequest *orig_req = udev->op.stash;
    xhci_udev_op_clear(udev);
    if (!orig_req)
        return;

    xhci_udev_do_set_config(udev, orig_req);
}

/* Phase 1 of the BOS pre-fetch: validate the 5-byte header and issue the
 * full-length BOS fetch.  Returns TRUE if phase 2 was submitted (the
 * UDEV_OP_CONFIGURE stash stays put; the caller must not run set-config yet);
 * FALSE if it should give up and run the deferred SET_CONFIGURATION now.  On
 * submit or a post-free give-up *pbuf is cleared so the caller does not double
 * free; a header-validation give-up leaves *pbuf for the caller to free. */
static BOOL xhci_udev_bos_prefetch_phase1(struct usb_device *udev, u8 **pbuf)
{
    struct xhci_ctrl *ctrl = udev->controller;
    u8 *buf = *pbuf;

    KprintfH("HUB descriptor pre-fetch phase 1: total length field is %lu\n",
             (ULONG)le16(((struct usb_bos_descriptor *)buf)->wTotalLength));
    const struct usb_bos_descriptor *hdr = (const struct usb_bos_descriptor *)buf;
    if (hdr->bDescriptorType != USB_DT_BOS || hdr->bNumDeviceCaps == 0)
        return FALSE;

    u16 total = le16(hdr->wTotalLength);
    if (total <= (u16)sizeof(struct usb_bos_descriptor) || total > 512u)
    {
        Kprintf("BOS wTotalLength %u out of range\n", (unsigned)total);
        return FALSE;
    }

    dma_free(ctrl->dmaPool, buf);
    *pbuf = NULL;

    u8 *buf2 = dma_alloc(ctrl->dmaPool, DMA_ALIGN_MIN, (u32)total);
    struct USBIORequest *io2 = pool_zalloc(ctrl->metaPool, sizeof(*io2));
    if (!io2 || !buf2)
    {
        Kprintf("xhci_udev_handle_bos_prefetch: phase-2 alloc failed\n");
        if (buf2)
            dma_free(ctrl->dmaPool, buf2);
        if (io2)
            pool_free(ctrl->metaPool, io2);
        return FALSE;
    }

    io2->req.io_Command = CMD_REQUEST_CONTROL;
    io2->req.io_Flags = IOF_QUICK;
    io2->driver_private_flags = REQ_INTERNAL | REQ_ENQUEUED | REQ_BOS_FETCH;

    io2->setup.bmRequestType = USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE;
    io2->setup.bRequest = USB_REQ_GET_DESCRIPTOR;
    io2->setup.wValue = le16((u16)(USB_DT_BOS << 8));
    io2->setup.wIndex = 0;
    io2->setup.wLength = le16(total);

    io2->virtual_address = udev->virtual_address;
    io2->data_buffer = buf2;
    io2->data_buffer_length = (u32)total;
    io2->direction = DIRECTION_IN;

    s8 ring_ret = xhci_ring_enqueue_td(udev, io2, 1000, FALSE);
    if (ring_ret != ERR_NO_ERROR)
    {
        Kprintf("xhci_udev_handle_bos_prefetch: phase-2 enqueue failed (%ld)\n", (LONG)ring_ret);
        dma_free(ctrl->dmaPool, buf2);
        pool_free(ctrl->metaPool, io2);
        return FALSE;
    }

    /* Phase 2 in flight; the UDEV_OP_CONFIGURE stash stays put */
    return TRUE;
}

static void xhci_udev_handle_bos_prefetch(struct usb_device *udev, struct USBIORequest *io)
{
    struct xhci_ctrl *ctrl = udev->controller;
    u8 *buf = io->data_buffer;
    KprintfH("BOS descriptor pre-fetch done for addr=%lu\n", (ULONG)udev->virtual_address);

    io->data_buffer = NULL;

    if (io->req.io_Error != ERR_NO_ERROR || !buf)
        goto run_set_config;

    if (io->data_buffer_length == (u32)sizeof(struct usb_bos_descriptor))
    {
        /* Phase 1: validate header and issue the full-BOS fetch. */
        if (xhci_udev_bos_prefetch_phase1(udev, &buf))
            return; /* phase 2 in flight */
    }
    else
    {
        /* Phase 2: hand the full BOS off to the LPM module, which parses the
         * Device Capability descriptors into udev's LPM fields and computes the
         * SEL/PEL/MEL parameters. */
        xhci_lpm_parse_bos_caps(udev, buf, io->data_buffer_length);
    }

run_set_config:
    /* BOS handling is terminal here (phase-2 parse, or a give-up path); cache it
     * so a repeat SET_CONFIGURATION reuses the LPM data instead of re-fetching.
     * The phase-1 success path returns earlier and does not reach this label. */
    udev->bos_fetched = TRUE;
    if (buf)
        dma_free(ctrl->dmaPool, buf);

    xhci_udev_run_pending_set_config(udev);
}

/**
 * Complete a successful internal (REQ_INTERNAL) request: run any deferred
 * pre-fetch completion handler, free transient payload buffers and release the
 * io struct.  Only used on the success path (xhci_udev_io_reply_data) where EP0
 * is healthy; the failure path frees the request directly and resumes the
 * deferred SET_CONFIGURATION from handle_set_deq() after EP0 recovery.
 */
static void xhci_udev_complete_internal(struct usb_device *udev, struct USBIORequest *io)
{
    struct xhci_ctrl *ctrl = udev->controller;

    /* Hub descriptor pre-fetch completion: now run the full
     * SET_CONFIGURATION + CONFIG_EP with real hub data available.
     * On the success path xhci_udev_parse_control_message already ran, so
     * xhci_hub_handle_get_descriptor cached ss_hub_desc + tt_think_time. */
    if (io->driver_private_flags & REQ_HUB_DESC_FETCH)
        xhci_udev_handle_hub_prefetch(udev, io);

    if (io->driver_private_flags & REQ_BOS_FETCH)
        xhci_udev_handle_bos_prefetch(udev, io);

    /* SET_SEL OUT transfer completed: free its 6-byte payload buffer and, now
     * that the device knows the exit latencies, enable device-initiated U1/U2
     * (UDEV_OP_LPM_ENABLE step 1).  A failed SET_SEL goes through
     * xhci_udev_io_reply_failed instead, which cancels the op. */
    if (io->driver_private_flags & REQ_SET_SEL)
    {
        if (ctrl && io->data_buffer)
        {
            dma_free(ctrl->dmaPool, io->data_buffer);
            io->data_buffer = NULL;
        }
        xhci_udev_op_advance(udev, UDEV_OP_EVENT_SET_SEL_DONE);
    }

#ifdef DEBUG_HIGH
    /* Confirm the device actually accepted the LPM feature enables (rejects
     * land in xhci_udev_io_reply_failed instead). */
    if (io->setup.bRequest == USB_REQ_SET_FEATURE &&
        io->setup.bmRequestType == (USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE) &&
        (le16(io->setup.wValue) == USB_DEVICE_U1_ENABLE ||
         le16(io->setup.wValue) == USB_DEVICE_U2_ENABLE))
        KprintfH("SET_FEATURE U%lu_ENABLE accepted by addr %lu\n",
                (ULONG)(le16(io->setup.wValue) - USB_DEVICE_U1_ENABLE + 1),
                (ULONG)io->virtual_address);
#endif

    if (ctrl)
        pool_free(ctrl->metaPool, io);
}

void xhci_udev_send_control_request(struct usb_device *udev, u8 ep_index,
                                    u8 bmRequestType, u8 bRequest,
                                    u16 wValue, u16 wIndex, u16 wLength,
                                    BOOL enqueue)
{
    if (!udev || !udev->controller)
        return;

    struct xhci_ctrl *ctrl = udev->controller;
    struct USBIORequest *io = pool_zalloc(ctrl->metaPool, sizeof(*io));
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
        pool_free(ctrl->metaPool, io);
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

/* ---- Multi-step device operations ------------------------------------------
 * One operation in flight per device (struct udev_operation).  Cross-module
 * completion sites feed xhci_udev_op_advance(); op-specific steps live in its
 * dispatch.  LPM enable is advisory and stashless, so a newly started
 * operation preempts it; everything else is mutually exclusive by
 * construction (and rejected with a log if that assumption ever breaks).
 */

/* Abort the in-flight operation (which == UDEV_OP_NONE matches any), replying
 * the stashed request so the stack isn't left waiting. */
void xhci_udev_op_cancel(struct usb_device *udev, enum udev_op which, s8 err)
{
    if (udev->op.op == UDEV_OP_NONE)
        return;
    if (which != UDEV_OP_NONE && udev->op.op != which)
        return;

    KprintfH("addr %lu: cancelling op %lu (step %lu)\n",
             (ULONG)udev->virtual_address, (ULONG)udev->op.op, (ULONG)udev->op.step);

    struct USBIORequest *stash = udev->op.stash;
    xhci_udev_op_clear(udev);
    if (stash)
        xhci_udev_io_reply_failed(udev->controller, stash, err);
}

BOOL xhci_udev_op_begin(struct usb_device *udev, enum udev_op op, struct USBIORequest *stash)
{
    if (udev->op.op == UDEV_OP_LPM_ENABLE && op != UDEV_OP_LPM_ENABLE)
        xhci_udev_op_cancel(udev, UDEV_OP_LPM_ENABLE, ERR_NO_ERROR);

    if (udev->op.op != UDEV_OP_NONE)
    {
        Kprintf("addr %lu: op %lu in flight, cannot start %lu\n",
                (ULONG)udev->virtual_address, (ULONG)udev->op.op, (ULONG)op);
        return FALSE;
    }

    /* Slot-state gates (xHCI 4.5.3): operations only make sense once the
     * device has reached the matching state. */
    switch (op)
    {
    case UDEV_OP_CONFIGURE:
    case UDEV_OP_SUSPEND:
        if (udev->slot_state < USB_DEV_SLOT_STATE_ADDRESSED)
        {
            Kprintf("addr %lu: op %lu needs an addressed slot (state %lu)\n",
                    (ULONG)udev->virtual_address, (ULONG)op, (ULONG)udev->slot_state);
            return FALSE;
        }
        break;
    case UDEV_OP_LPM_ENABLE:
        if (udev->slot_state != USB_DEV_SLOT_STATE_CONFIGURED)
            return FALSE;
        break;
    default:
        return FALSE;
    }

    udev->op.op = op;
    udev->op.step = 0;
    udev->op.waits = 0;
    udev->op.arg = 0;
    udev->op.stash = stash;
    return TRUE;
}

/* ---- Port suspend (U3) sequencing -----------------------------------------
 * xHCI 4.15.1: all of a device's endpoints shall be stopped before its port is
 * directed to U3.  Stop Endpoint completes asynchronously, so the suspend is
 * sequenced as UDEV_OP_SUSPEND: stop every endpoint ring (queued TDs stay on
 * the rings), count the completions, and only then suspend the port - a root
 * port by writing PLS=U3, a device behind an external hub by forwarding the
 * deferred SetPortFeature(SUSPEND) to that hub.  Mirrors Linux
 * xhci_stop_device()/xhci_ring_device().  Suspending a hub does not recurse
 * into its children: the stack suspends leaf devices first (as usbcore does).
 */

/* Tail of the suspend sequence, run when the last Stop Endpoint completes. */
static void xhci_udev_suspend_finish(struct usb_device *udev)
{
    u8 port = udev->op.arg;
    struct USBIORequest *req = udev->op.stash;
    xhci_udev_op_clear(udev);

    if (port != 0)
    {
        xhci_roothub_set_port_u3(udev->controller->root_hub, port);
        return;
    }

    if (req)
    {
        u32 timeout_ms = (req->flags & DRIVER_FLAG_TIMEOUT_DEFINED) ? req->timeout : 0;
        xhci_ring_enqueue_td(udev->parent, req, timeout_ms, FALSE);
    }
}

/* Stop all endpoint rings of udev ahead of a port suspend.  Returns TRUE if
 * Stop Endpoint commands are in flight and the port suspend is deferred to
 * xhci_udev_suspend_finish(); FALSE if there is nothing to wait for (caller
 * suspends the port synchronously). */
static BOOL xhci_udev_suspend_device(struct usb_device *udev, u8 root_port, struct USBIORequest *deferred_req)
{
    if (!xhci_udev_op_begin(udev, UDEV_OP_SUSPEND, deferred_req))
        return FALSE;

    u8 stops = 0;
    for (u8 i = 0; i < USB_MAX_ENDPOINT_CONTEXTS; ++i)
        if (udev->ep_context[i] && xhci_ep_request_suspend(udev->ep_context[i]))
            stops++;

    if (stops == 0)
    {
        xhci_udev_op_clear(udev); /* nothing to wait for; caller acts synchronously */
        return FALSE;
    }

    udev->op.waits = stops;
    udev->op.arg = root_port;
    KprintfH("suspending addr %lu: %lu endpoint stops pending\n",
             (ULONG)udev->virtual_address, (ULONG)stops);
    return TRUE;
}

/* Root-hub entry: stop the rings of the device on 1-based root port.  TRUE if
 * the U3 write is deferred until the stops complete. */
BOOL xhci_udev_suspend_port(struct usb_device *hub_udev, u8 port)
{
    struct usb_device *child = xhci_udev_find_child_on_port(hub_udev, port);
    if (!child)
        return FALSE;
    return xhci_udev_suspend_device(child, port, NULL);
}

/* Port resumed (root hub U0 write, or ClearPortFeature(SUSPEND) completion on
 * an external hub): restart the attached device's stopped endpoint rings. */
void xhci_udev_resume_port(struct usb_device *hub_udev, u8 port)
{
    struct usb_device *child = xhci_udev_find_child_on_port(hub_udev, port);
    if (!child)
        return;

    for (u8 i = 0; i < USB_MAX_ENDPOINT_CONTEXTS; ++i)
        if (child->ep_context[i])
            xhci_ep_resume(child->ep_context[i]);
}

/* Single entry point for async completions that advance the in-flight
 * operation.  (op, event) pairs that don't match are silently ignored - the
 * completion sites fire unconditionally and the op decides relevance. */
void xhci_udev_op_advance(struct usb_device *udev, enum udev_op_event event)
{
    struct udev_operation *op = &udev->op;

    switch (op->op)
    {
    case UDEV_OP_CONFIGURE:
        if (event == UDEV_OP_EVENT_EP0_RECOVERED)
            xhci_udev_run_pending_set_config(udev);
        return;

    case UDEV_OP_LPM_ENABLE:
        if (event == UDEV_OP_EVENT_MEL_EVAL_DONE && op->step == 0)
        {
            op->step = 1;
            /* No SET_SEL submitted: host-initiated only, sequence complete. */
            if (!xhci_lpm_enable_stage2(udev))
                xhci_udev_op_clear(udev);
        }
        else if (event == UDEV_OP_EVENT_SET_SEL_DONE && op->step == 1)
        {
            xhci_udev_op_clear(udev);
            xhci_lpm_devinit_enable(udev);
        }
        return;

    case UDEV_OP_SUSPEND:
        if (event == UDEV_OP_EVENT_STOP_DONE && op->waits != 0 && --op->waits == 0)
            xhci_udev_suspend_finish(udev);
        return;

    default:
        KprintfH("addr %lu: event %lu with no op in flight\n",
                 (ULONG)udev->virtual_address, (ULONG)event);
        return;
    }
}

/* Issue an internal CLEAR_FEATURE(ENDPOINT_HALT) to endpoint (by ep_index) on udev. Fire-and-forget. */
void xhci_udev_clear_feature_halt(struct usb_device *udev, u8 ep_index)
{
    if (!udev || !udev->controller || ep_index == 0)
        return;

    /* Convert ep_index (DCI-1) to USB endpoint address (number + direction bit). */
    u8 addr = xhci_ep_index_to_address(ep_index);

    /* The next stack-issued clear-halt for this endpoint is a duplicate. */
    xhci_ep_mark_halt_synced(xhci_ep_get_context_for_index(udev, ep_index));

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

void xhci_udev_disconnect(struct usb_device *udev, BOOL recursive)
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

    /* An operation in flight dies with the device: reply its stashed request
     * (deferred SetPortFeature(SUSPEND) or SET_CONFIGURATION) so the stack
     * isn't left waiting; late completions land on UDEV_OP_NONE and are
     * ignored. */
    xhci_udev_op_cancel(udev, UDEV_OP_NONE, ERR_TIMEOUT);

    /* Clear USB2 hardware LPM on the root port before the slot goes away so
     * PORTPMSC.L1DS no longer references this (about to be freed) slot. */
    xhci_lpm_disable(udev);

    xhci_disable_slot(udev);
}

struct usb_device *xhci_udev_find_child_on_port(struct usb_device *hub, u32 port)
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

    udev->device_protocol = dev_desc->bDeviceProtocol;

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

    xhci_udev_remap(ctrl, current, new_addr);

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
            xhci_parse_config_descriptor(udev, (u8 *)io->data_buffer, (u16)io->actual_length);

            /* USB 2.0 stacks  don't like seeing SS companion descriptors */
            xhci_hub_filter_ss_ep_companion_desc(io);
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
            xhci_hub_handle_get_descriptor(udev, io, descriptorType);
        break;

    case GetPortStatus:
        /* Detect downstream port disconnects via hub GET_STATUS replies. */
        xhci_hub_handle_get_port_status(udev, io);
        break;

    case ClearPortFeature:
        /* Port resume on an external hub completed: restart the child's
         * endpoint rings stopped for the suspend. */
        if (le16(io->setup.wValue) == USB_PORT_FEAT_SUSPEND)
            xhci_udev_resume_port(udev, le16(io->setup.wIndex) & 0xffU);
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

    case (DeviceOutRequest | USB_REQ_SET_CONFIGURATION):
        /* The device is now in the Configured state and accepts the LPM
         * feature requests (SET_FEATURE U1/U2_ENABLE is a Request Error in the
         * Address state, USB 3.2 9.4.9). */
        if (le16(io->setup.wValue) != 0)
            xhci_lpm_enable(udev);
        break;
    }
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
        xhci_udev_complete_internal(udev, io);
        return;
    }

    ReplyMsg((struct Message *)io);
}

/* ---- Request dispatch entry points -------------------------------------- */

/* SET_CONFIGURATION special case: stash the request as UDEV_OP_CONFIGURE, then
 * optionally pre-fetch a descriptor on EP0 before configuring; the prefetch
 * completion resumes the deferred SET_CONFIGURATION
 * (xhci_udev_run_pending_set_config).
 *   - Hubs: GET_DESCRIPTOR(hub) so xhci_set_configuration() can program correct
 *     Number of Ports / TT Think Time into the slot context.
 *   - HS/SS: GET_DESCRIPTOR(BOS) for U1/U2 exit latencies (USB3) or BESL (USB2
 *     LPM), needed for Max Exit Latency.  Non-fatal.
 * EP0 is live (set up during ADDRESS_DEVICE).  A prefetch that can't be
 * submitted leaves the stash intact, so we just run the config now.
 * Returns TRUE when this was a SET_CONFIGURATION (always fully handled). */
static BOOL xhci_udev_ctrl_set_config(struct usb_device *udev, struct USBIORequest *io)
{
    struct USBSetupPacket *setup = &io->setup;
    if (!(setup->bRequest == USB_REQ_SET_CONFIGURATION &&
          (setup->bmRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD))
        return FALSE;

    if (!xhci_udev_op_begin(udev, UDEV_OP_CONFIGURE, io))
    {
        /* op slot busy (unexpected): configure without the prefetch */
        xhci_udev_do_set_config(udev, io);
        return TRUE;
    }

    if (udev->is_hub)
    {
        if (!udev->hub_desc_fetched && xhci_udev_fetch_hub_descriptor(udev))
            return TRUE;
    }
    else if (udev->speed >= USB_SPEED_HIGH)
    {
        if (!udev->bos_fetched && xhci_udev_fetch_bos(udev))
            return TRUE;
    }

    /* LS/FS, or the prefetch wasn't submitted: configure immediately. */
    xhci_udev_run_pending_set_config(udev);
    return TRUE;
}

/* CLEAR_FEATURE(ENDPOINT_HALT) from the stack: keep the controller-side
 * endpoint state in sync with the device (mirror Linux xhci_endpoint_reset).
 * If our own STALL recovery already reset the endpoint and sent the
 * device-level clear-halt, answer without a duplicate wire request; if the
 * endpoint is still halted (the stack noticed before our event handling), run
 * our recovery - it ends with the internal clear-halt.  Returns TRUE only when
 * the request was answered here; an unsolicited clear on a healthy endpoint
 * (e.g. mass-storage reset recovery) returns FALSE and is forwarded as-is: the
 * device resets its data toggle/sequence, and a stale xHC sequence recovers
 * through the normal STALL path on the next transfer. */
static BOOL xhci_udev_ctrl_clear_ep_halt(struct usb_device *udev, struct USBIORequest *io)
{
    struct USBSetupPacket *setup = &io->setup;
    if (!(setup->bmRequestType == (USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_ENDPOINT) &&
          setup->bRequest == USB_REQ_CLEAR_FEATURE &&
          le16(setup->wValue) == USB_ENDPOINT_HALT &&
          udev->slot_id != 0))
        return FALSE;

    u8 ep_addr = (u8)(le16(setup->wIndex) & 0xffU);
    u8 halt_ep_index = xhci_ep_index_from_parts(ep_addr & 0x0fU,
                                                (ep_addr & USB_DIR_IN) ? DIRECTION_IN : DIRECTION_OUT);
    struct ep_context *halt_ep_ctx = (halt_ep_index != 0)
                                         ? xhci_ep_get_context_for_index(udev, halt_ep_index)
                                         : NULL;
    if (!halt_ep_ctx)
        return FALSE;

    BOOL synced = xhci_ep_consume_halt_synced(halt_ep_ctx);
    if (!synced && xhci_read_hw_ep_state(udev, halt_ep_index) == EP_STATE_HALTED)
    {
        KprintfH("stack clear-halt on halted EP %lu: running recovery\n", (ULONG)halt_ep_index);
        xhci_reset_ep(udev, halt_ep_index);
        synced = TRUE;
    }
    if (!synced)
        return FALSE;

    io->req.io_Error = ERR_NO_ERROR;
    io->actual_length = 0;
    if (!(io->req.io_Flags & IOF_QUICK))
        ReplyMsg((struct Message *)io);
    return TRUE;
}

/* SetPortFeature(SUSPEND) to an external hub: stop the attached child's
 * endpoint rings first (xHCI 4.15.1); the request is forwarded to the hub once
 * the stops complete (xhci_udev_suspend_finish).  Returns TRUE if the child's
 * stop sequence was started (request deferred), FALSE to forward immediately. */
static BOOL xhci_udev_ctrl_hub_suspend(struct usb_device *udev, struct USBIORequest *io)
{
    struct USBSetupPacket *setup = &io->setup;
    if (!(udev->is_hub &&
          setup->bmRequestType == (USB_DIR_OUT | USB_RT_PORT) &&
          setup->bRequest == USB_REQ_SET_FEATURE &&
          le16(setup->wValue) == USB_PORT_FEAT_SUSPEND))
        return FALSE;

    struct usb_device *child = xhci_udev_find_child_on_port(udev, le16(setup->wIndex) & 0xffU);
    return (child && xhci_udev_suspend_device(child, 0, io)) ? TRUE : FALSE;
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
    xhci_hub_translate_descriptor_request(udev, io);

    if (xhci_hub_filter_emulated_ctrl_request(udev, io))
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

    /* Standard-request special cases, each fully handled when it applies. */
    if (xhci_udev_ctrl_set_config(udev, io))
        return ERR_NO_ERROR;

    if (xhci_udev_ctrl_clear_ep_halt(udev, io))
        return ERR_NO_ERROR;

    if (xhci_udev_ctrl_hub_suspend(udev, io))
        return ERR_NO_ERROR;

    /* If we don't have a slot yet, enable one and allocate Virt Dev */
    if (udev->slot_id == 0 && udev->virtual_address == 0)
    {
        // this will store the req and submit it once addressed
        xhci_address_device(udev, io);
        return ERR_NO_ERROR;
    }

    return xhci_ring_enqueue_td(udev, io, timeout_ms, FALSE);
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
