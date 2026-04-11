// SPDX-License-Identifier: GPL-2.0+
/*
 * Based on xhci-ring.c from uboot
 * USB HOST XHCI Controller stack
 *
 * Based on xHCI host controller driver in linux-kernel
 * by Sarah Sharp.
 *
 * Copyright (C) 2008 Intel Corp.
 * Author: Sarah Sharp
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 * Authors: Vivek Gautam <gautam.vivek@samsung.com>
 *	    Vikas Sajjan <vikas.sajjan@samsung.com>
 */
#ifdef __INTELLISENSE__
#else
#endif

#include <debug.h>

#include <bits.h>
#include <byteorder.h>
#include <iomem.h>
#include <memory.h>

#include <devices/hcd_api.h>
#include <xhci/xhci.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-events.h>
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-descriptors.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci-ring.h>
#include <xhci/xhci-context.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-event] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci-event] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

typedef void (*ep_state_handler)(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event);

static void ep_handle_default(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event);
static void ep_handle_receiving_generic(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event);
static void ep_handle_rt_iso(struct USBIORequest *req, ULONG act_len, struct ep_context *ep_ctx, struct usb_device *udev);
static void ep_handle_receiving_control_short(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event);
static void ep_handle_aborting(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event);

static const ep_state_handler ep_state_dispatch[] = {
    [USB_DEV_EP_STATE_IDLE] = NULL, /* use default handler */
    [USB_DEV_EP_STATE_RECEIVING_CONTROL_SHORT] = ep_handle_receiving_control_short,
    [USB_DEV_EP_STATE_RECEIVING] = ep_handle_receiving_generic,
    [USB_DEV_EP_STATE_ABORTING] = ep_handle_aborting,
    [USB_DEV_EP_STATE_RESETTING] = NULL,
    [USB_DEV_EP_STATE_FAILED] = NULL,
    [USB_DEV_EP_STATE_RT_ISO_STOPPED] = NULL,
    [USB_DEV_EP_STATE_RT_ISO_RUNNING] = ep_handle_receiving_generic,
    [USB_DEV_EP_STATE_RT_ISO_STOPPING] = ep_handle_receiving_generic};

/* Endpoint event dispatcher
 * Called when a Transfer Event TRB is received
 */
static void dispatch_ep_event(struct usb_device *udev, union xhci_trb *event)
{
    UWORD ep_index = TRB_TO_EP_INDEX(le32(event->trans_event.flags));

    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
    if (!ep_ctx)
    {
        KprintfH("No ep context for addr %ld ep %ld\n", (LONG)udev->virtual_address, (LONG)ep_index);
        return;
    }
    enum ep_state state = xhci_ep_get_state(ep_ctx);

    if (ep_state_dispatch[state])
    {
        KprintfH("addr %ld EP %ld state %ld -> handling event\n", udev->virtual_address, (LONG)ep_index, (LONG)state);
        ep_state_handler handler = ep_state_dispatch[state];
        handler(udev, ep_ctx, event);
    }
    else
    {
        ep_handle_default(udev, ep_ctx, event);
    }
}

BOOL xhci_process_event_trb(struct xhci_ctrl *ctrl)
{
    BOOL activity = FALSE;
    union xhci_trb *event;
    while ((event = xhci_ring_get_event_trb(ctrl->event_ring)))
    {
        activity = TRUE;
        trb_type type = TRB_FIELD_TO_TYPE(le32(event->event_cmd.flags));
        xhci_ring_acknowledge_event(ctrl);

        switch (type)
        {
        case TRB_TRANSFER:
        {
            int slot = TRB_TO_SLOT_ID(le32(event->trans_event.flags));
            KprintfH("Transfer Event TRB detected: slot %ld ep %ld (%08lx %08lx %08lx %08lx)\n",
                     (LONG)slot,
                     (LONG)TRB_TO_EP_INDEX(le32(event->trans_event.flags)),
                     (LONG)le32(event->generic.field[0]),
                     (LONG)le32(event->generic.field[1]),
                     (LONG)le32(event->generic.field[2]),
                     (LONG)le32(event->generic.field[3]));

            struct usb_device *udev = ctrl->devices_by_slot_id[slot];
            if (!udev)
            {
                Kprintf("No usb_device for slot %ld\n", slot);
                break;
            }
            KprintfH("USB device addr %ld on slot %ld\n", (LONG)udev->virtual_address, (LONG)slot);

            dispatch_ep_event(udev, event);
        }
        break;

        case TRB_COMPLETION:
            xhci_dispatch_command_event(ctrl, event);
            break;

        case TRB_PORT_STATUS:
        {
#ifdef DEBUG_HIGH
            const ULONG port_field = (ULONG)le32(event->generic.field[0]);
            const ULONG flags = (ULONG)le32(event->generic.field[3]);
            const ULONG port_id = (ULONG)GET_PORT_ID(port_field);
            ULONG portsc = 0;

            if (port_id > 0 && port_id <= MAX_HC_PORTS)
                portsc = mmio_read32(&ctrl->hcor->portregs[port_id - 1].or_portsc);

            Kprintf("Port Status Change Event port=%lu portsc=%08lx usbsts=%08lx trb=(%08lx %08lx %08lx %08lx)\n",
                    port_id,
                    portsc,
                    (ULONG)mmio_read32(&ctrl->hcor->or_usbsts),
                    port_field,
                    (ULONG)le32(event->generic.field[1]),
                    (ULONG)le32(event->generic.field[2]),
                    flags);
#endif
            xhci_roothub_complete_int_request(ctrl->root_hub);
            break;
        }
        default:
            Kprintf("Unexpected XHCI event type %ld, skipping... (%08lx %08lx %08lx %08lx)\n",
                    (ULONG)type,
                    (ULONG)le32(event->generic.field[0]),
                    (ULONG)le32(event->generic.field[1]),
                    (ULONG)le32(event->generic.field[2]),
                    (ULONG)le32(event->generic.field[3]));
            break;
        }
    }
    return activity;
}

void xhci_process_event_timeouts(struct xhci_ctrl *ctrl)
{
    for (int i = 0; i < USB_MAX_ADDRESS; i++)
    {
        struct usb_device *udev = ctrl->devices_by_virtual_address[i];
        if (!udev)
            continue;

        for (int ep_index = 0; ep_index < USB_MAX_ENDPOINT_CONTEXTS; ep_index++)
        {
            struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
            if (ep_ctx && xhci_ep_is_expired(ep_ctx))
            {
                KprintfH("XHCI TD timeout on slot %ld ep %ld\n", (LONG)udev->slot_id, (LONG)ep_index);
                xhci_ep_request_timeout_recovery(ep_ctx);
            }
        }
    }
}

inline static ULONG translate_status(xhci_comp_code comp)
{
    ULONG status;

    switch (comp)
    {
    case COMP_SUCCESS:
        /* fallthrough */
    case COMP_SHORT_TX:
        status = ERR_NO_ERROR;
        break;
    case COMP_STALL:
        KprintfH("Device stalled\n");
        status = ERR_DEVICE_STALL;
        break;
    case COMP_TX_ERR:
        Kprintf("USB transaction error\n");
        status = ERR_TIMEOUT;
        break;
    case COMP_DB_ERR:
    case COMP_TRB_ERR:
        // Data Buffer Error or TRB Error
        Kprintf("TRB error\n");
        status = ERR_HCI_ERROR;
        break;
    case COMP_BABBLE:
        KprintfH("Babble detected\n");
        status = ERR_DEVICE_BABBLE;
        break;
        // TODO more codes, e.g. underrun/overrun
    case COMP_BUFF_OVER:
        KprintfH("Isoc buffer overrun\n");
        status = ERR_ISOC_OVERRUN;
        break;
    case COMP_BW_OVER:
        KprintfH("Bandwidth overrun\n");
        status = ERR_HCI_ERROR;
        break;
    case COMP_SPLIT_ERR:
        KprintfH("Split transaction error\n");
        status = ERR_TIMEOUT;
        break;
    default:
        Kprintf("Unhandled completion code %ld\n", (LONG)comp);
        status = ERR_HCI_ERROR;
    }

    return status;
}

/*
 * EP handlers
 */

/* Default endpoint event handler
 * Called when no specific handler is registered for the current endpoint state
 */
static void ep_handle_default(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{
    (void)event;
    enum ep_state state = xhci_ep_get_state(ep_ctx);
    int ep_index = xhci_ep_get_ep_index(ep_ctx);
    Kprintf("No handler for endpoint %ld state %ld addr %ld\n", ep_index, state, udev->virtual_address);
    KprintfH("Event TRB: (%08lx %08lx %08lx %08lx)\n",
             (ULONG)le32(event->generic.field[0]),
             (ULONG)le32(event->generic.field[1]),
             (ULONG)le32(event->generic.field[2]),
             (ULONG)le32(event->generic.field[3]));
}

static void ep_handle_receiving_generic(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{

    u32 flags = le32(event->trans_event.flags);
    int ep_index = TRB_TO_EP_INDEX(flags);
    u64 trb_addr = le64(event->trans_event.buffer);
    xhci_comp_code comp = GET_COMP_CODE(le32(event->trans_event.transfer_len));

#ifdef DEBUG_CONTEXT
    KprintfH("event flags=%08lx xfer_len=%08lx buf=%08lx%08lx\n",
             (ULONG)flags,
             (ULONG)le32(event->trans_event.transfer_len),
             (ULONG)u64_hi32(trb_addr),
             (ULONG)u64_lo32(trb_addr));
    xhci_dump_slot_ctx("[xhci-event] ep_handle_receiving_generic:", udev, FALSE);
    xhci_dump_ep_ctx("[xhci-event] ep_handle_receiving_generic:", udev, ep_index);
#endif

    /* Isoch OUT rings signal underrun/overrun with null TRB pointers. Do not treat those as lost TDs. */
    if ((comp == COMP_UNDERRUN || comp == COMP_OVERRUN) && trb_addr == 0)
    {
        enum ep_state state = xhci_ep_get_state(ep_ctx);
        Kprintf("Ring %s on addr %ld EP %ld state=%ld\n",
                (comp == COMP_UNDERRUN) ? "underrun" : "overrun",
                (LONG)udev->virtual_address,
                (LONG)ep_index,
                (LONG)state);

        if (state == USB_DEV_EP_STATE_RT_ISO_RUNNING)
            xhci_ep_schedule_rt_iso(ep_ctx);

        return;
    }

    struct USBIORequest *req = xhci_ep_get_by_trb(ep_ctx, trb_addr);
    if (!req)
    {
        Kprintf("No TD found for TRB %08lx%08lx  %08lx %08lx on EP %ld\n",
                (ULONG)u64_hi32(trb_addr), (ULONG)u64_lo32(trb_addr), (ULONG)flags, (ULONG)le32(event->trans_event.transfer_len), (LONG)ep_index);
        return;
    }

    ULONG act_len = req->data_buffer_length - EVENT_TRB_LEN(le32(event->trans_event.transfer_len));

    BOOL is_rt_iso = req->req.io_Command == CMD_REGISTER_ISOCHRONOUS_HOOKS;
    if (is_rt_iso)
    {
        KprintfH("RT ISO complete dir=%s act_len=%lu\n",
                 req->direction == DIRECTION_IN ? "IN" : "OUT",
                 (ULONG)act_len);
        ep_handle_rt_iso(req, act_len, ep_ctx, udev);
        return;
    }

    ULONG status = translate_status(comp);
    KprintfH("result status=%ld act_len=%ld comp=%ld\n", (LONG)status, (LONG)act_len, (LONG)comp);

    /* Flag short IN transfers as runts unless explicitly allowed or expected (control). */
    if (status == ERR_NO_ERROR && act_len < req->data_buffer_length &&
        req->direction == DIRECTION_IN && req->req.io_Command != CMD_REQUEST_CONTROL &&
        (req->flags & DRIVER_FLAG_IGNORE_SHORT_TRANSFER) == 0)
    {
        status = ERR_SHORT_TRANSFER;
    }

    BOOL halted = (comp == COMP_STALL || comp == COMP_BABBLE || comp == COMP_SPLIT_ERR || comp == COMP_TX_ERR);
    if (halted)
    {
        xhci_udev_io_reply_failed(udev->controller, req, status);
        xhci_reset_ep(udev, ep_index);
        return;
    }

    xhci_udev_io_reply_data(udev, req, status, act_len);
    if (req->req.io_Command == CMD_REQUEST_CONTROL && comp == COMP_SHORT_TX)
        xhci_ep_set_receiving_control_short(ep_ctx);
    else
        xhci_ep_set_idle(ep_ctx);
}

void ep_handle_rt_iso(struct USBIORequest *req, ULONG act_len, struct ep_context *ep_ctx, struct usb_device *udev)
{
    struct xhci_ctrl *ctrl = udev->controller;

    if (req->direction == DIRECTION_IN)
    {
        if (act_len > 0)
            xhci_ep_rt_iso_in(ep_ctx, req, act_len);

        pool_free(ctrl->memoryPool, req->data_buffer);
    }
    else
        xhci_ep_rt_iso_out(ep_ctx, req, act_len);

    /* RT ISO TDs clone IO requests; free them after completion to avoid leaks. */
    pool_free(ctrl->memoryPool, req);

    xhci_ep_schedule_rt_iso(ep_ctx);
}

static void ep_handle_receiving_control_short(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{
    (void)udev;
    (void)ep_ctx;
    (void)event;

    /* Short data stage, clear up additional status stage event */
    KprintfH("short-tx status event flags=%08lx status=%08lx\n",
             (ULONG)le32(event->generic.field[3]),
             (ULONG)le32(event->generic.field[2]));

    // no need to confirm slot and ep as this is done by event handler
    xhci_ep_set_idle(ep_ctx);
}

static void ep_handle_aborting(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{
    (void)ep_ctx;

    u32 flags = le32(event->trans_event.flags);
    if (TRB_TO_SLOT_ID(flags) != udev->slot_id)
    {
        Kprintf("Expected a TRB for slot %ld, got %ld\n", udev->slot_id, TRB_TO_SLOT_ID(flags));
        return;
    }
    if (GET_COMP_CODE(le32(event->trans_event.transfer_len)) != COMP_STOP)
    {
        Kprintf("Expected a TRB with STOP, got %ld\n", GET_COMP_CODE(le32(event->trans_event.transfer_len)));
    }

    /* no state change - that is done by handle_abort_stop_ring */
}
