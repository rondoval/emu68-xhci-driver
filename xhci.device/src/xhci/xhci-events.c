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
#include <clib/utility_protos.h>
#else
#include <proto/utility.h>
#endif

#include <compat.h>
#include <debug.h>

#include <devices/usbhardware.h>
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
static void ep_handle_rt_iso(struct IOUsbHWReq *req, ULONG act_len, struct ep_context *ep_ctx, struct usb_device *udev);
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
    UWORD ep_index = TRB_TO_EP_INDEX(LE32(event->trans_event.flags));

    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
    if (!ep_ctx)
    {
        KprintfH("No ep context for addr %ld ep %ld\n", (LONG)udev->poseidon_address, (LONG)ep_index);
        return;
    }
    enum ep_state state = xhci_ep_get_state(ep_ctx);

    if (ep_state_dispatch[state])
    {
        KprintfH("addr %ld EP %ld state %ld -> handling event\n", udev->poseidon_address, (LONG)ep_index, (LONG)state);
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
        trb_type type = TRB_FIELD_TO_TYPE(LE32(event->event_cmd.flags));
        xhci_ring_acknowledge_event(ctrl);

        switch (type)
        {
        case TRB_TRANSFER:
        {
            int slot = TRB_TO_SLOT_ID(LE32(event->trans_event.flags));
            KprintfH("Transfer Event TRB detected: slot %ld ep %ld (%08lx %08lx %08lx %08lx)\n",
                     (LONG)slot,
                     (LONG)TRB_TO_EP_INDEX(LE32(event->trans_event.flags)),
                     (LONG)LE32(event->generic.field[0]),
                     (LONG)LE32(event->generic.field[1]),
                     (LONG)LE32(event->generic.field[2]),
                     (LONG)LE32(event->generic.field[3]));

            struct usb_device *udev = ctrl->devices_by_slot_id[slot];
            if (!udev)
            {
                Kprintf("No usb_device for slot %ld\n", slot);
                break;
            }
            KprintfH("USB device addr %ld on slot %ld\n", (LONG)udev->poseidon_address, (LONG)slot);

            dispatch_ep_event(udev, event);
        }
        break;

        case TRB_COMPLETION:
            xhci_dispatch_command_event(ctrl, event);
            break;

        case TRB_PORT_STATUS:
            KprintfH("Port Status Change Event TRB: (%08lx %08lx %08lx %08lx)\n",
                     (ULONG)LE32(event->generic.field[0]),
                     (ULONG)LE32(event->generic.field[1]),
                     (ULONG)LE32(event->generic.field[2]),
                     (ULONG)LE32(event->generic.field[3]));
            xhci_roothub_complete_int_request(ctrl->root_hub);
            break;
        default:
            Kprintf("Unexpected XHCI event type %ld, skipping... (%08lx %08lx %08lx %08lx)\n",
                    (ULONG)type,
                    (ULONG)LE32(event->generic.field[0]),
                    (ULONG)LE32(event->generic.field[1]),
                    (ULONG)LE32(event->generic.field[2]),
                    (ULONG)LE32(event->generic.field[3]));
            break;
        }
    }
    return activity;
}

/*
 * This is supposed to implement NAK timeouts.
 * It scans all active TDs for expired timeouts,
 * if it finds one, it stops the endpoint, aborts all active TDs
 * and resets dequeue pointer.
 * I guess we could use something more sophisticated, like
 * stopping, removing the expired TD and restarting.
 */
void xhci_process_event_timeouts(struct xhci_ctrl *ctrl)
{
    for (int i = 0; i < USB_MAX_ADDRESS; i++)
    {
        struct usb_device *udev = ctrl->devices_by_poseidon_address[i];
        if (!udev)
            continue;

        for (int ep_index = 0; ep_index < USB_MAX_ENDPOINT_CONTEXTS; ep_index++)
        {
            struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
            if (ep_ctx && xhci_ep_is_expired(ep_ctx))
            {
                KprintfH("XHCI TD timeout on slot %ld ep %ld\n", (LONG)udev->slot_id, (LONG)ep_index);
                xhci_stop_endpoint(udev, ep_index);
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
        status = UHIOERR_NO_ERROR;
        break;
    case COMP_STALL:
        KprintfH("Device stalled\n");
        status = UHIOERR_STALL;
        break;
    case COMP_TX_ERR:
        Kprintf("USB transaction error\n");
        status = UHIOERR_TIMEOUT;
        break;
    case COMP_DB_ERR:
    case COMP_TRB_ERR:
        // Data Buffer Error or TRB Error
        Kprintf("TRB error\n");
        status = UHIOERR_HOSTERROR;
        break;
    case COMP_BABBLE:
        KprintfH("Babble detected\n");
        status = UHIOERR_BABBLE;
        break;
        // TODO more codes, e.g. underrun/overrun
    case COMP_BUFF_OVER:
        KprintfH("Isoc buffer overrun\n");
        status = UHIOERR_OVERFLOW;
        break;
    case COMP_BW_OVER:
        KprintfH("Bandwidth overrun\n");
        status = UHIOERR_HOSTERROR;
        break;
    case COMP_SPLIT_ERR:
        KprintfH("Split transaction error\n");
        status = UHIOERR_TIMEOUT;
        break;
    default:
        Kprintf("Unhandled completion code %ld\n", (LONG)comp);
        status = UHIOERR_HOSTERROR;
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
    Kprintf("No handler for endpoint %ld state %ld addr %ld\n", ep_index, state, udev->poseidon_address);
    KprintfH("Event TRB: (%08lx %08lx %08lx %08lx)\n",
             (ULONG)LE32(event->generic.field[0]),
             (ULONG)LE32(event->generic.field[1]),
             (ULONG)LE32(event->generic.field[2]),
             (ULONG)LE32(event->generic.field[3]));
}

static void ep_handle_receiving_generic(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{

    u32 flags = LE32(event->trans_event.flags);
    int ep_index = TRB_TO_EP_INDEX(flags);
    u64 trb_addr = LE64(event->trans_event.buffer);
    xhci_comp_code comp = GET_COMP_CODE(LE32(event->trans_event.transfer_len));

#ifdef DEBUG_HIGH
    KprintfH("event flags=%08lx xfer_len=%08lx buf=%08lx%08lx\n",
             (ULONG)flags,
             (ULONG)LE32(event->trans_event.transfer_len),
             (ULONG)upper_32_bits(trb_addr),
             (ULONG)lower_32_bits(trb_addr));
    xhci_dump_slot_ctx("[xhci-event] ep_handle_receiving_generic:", udev, FALSE);
    xhci_dump_ep_ctx("[xhci-event] ep_handle_receiving_generic:", udev, ep_index);
#endif

    /* Isoch OUT rings signal underrun/overrun with null TRB pointers. Do not treat those as lost TDs. */
    if ((comp == COMP_UNDERRUN || comp == COMP_OVERRUN) && trb_addr == 0)
    {
        enum ep_state state = xhci_ep_get_state(ep_ctx);
        Kprintf("Ring %s on addr %ld EP %ld state=%ld\n",
                (comp == COMP_UNDERRUN) ? "underrun" : "overrun",
                (LONG)udev->poseidon_address,
                (LONG)ep_index,
                (LONG)state);

        if (state == USB_DEV_EP_STATE_RT_ISO_RUNNING)
            xhci_ep_schedule_rt_iso(ep_ctx);

        return;
    }

    struct IOUsbHWReq *req = xhci_ep_get_by_trb(ep_ctx, trb_addr);
    if (!req)
    {
        Kprintf("No TD found for TRB %08lx%08lx  %08lx %08lx on EP %ld\n",
                (ULONG)upper_32_bits(trb_addr), (ULONG)lower_32_bits(trb_addr), (ULONG)flags, (ULONG)LE32(event->trans_event.transfer_len), (LONG)ep_index);
        return;
    }

    ULONG act_len = req->iouh_Length - EVENT_TRB_LEN(LE32(event->trans_event.transfer_len));

    BOOL is_rt_iso = req->iouh_Req.io_Command == UHCMD_ADDISOHANDLER;
    if (is_rt_iso)
    {
        KprintfH("RT ISO complete dir=%s act_len=%lu\n",
                 req->iouh_Dir == UHDIR_IN ? "IN" : "OUT",
                 (ULONG)act_len);
        ep_handle_rt_iso(req, act_len, ep_ctx, udev);
        return;
    }

    ULONG status = translate_status(comp);
    KprintfH("result status=%ld act_len=%ld comp=%ld\n", (LONG)status, (LONG)act_len, (LONG)comp);

    /* Flag short IN transfers as runts unless explicitly allowed or expected (control). */
    if (status == UHIOERR_NO_ERROR && act_len < req->iouh_Length &&
        req->iouh_Dir == UHDIR_IN && req->iouh_Req.io_Command != UHCMD_CONTROLXFER &&
        (req->iouh_Flags & UHFF_ALLOWRUNTPKTS) == 0)
    {
        status = UHIOERR_RUNTPACKET;
    }

    BOOL halted = (comp == COMP_STALL || comp == COMP_BABBLE || comp == COMP_SPLIT_ERR || comp == COMP_TX_ERR);
    if (halted)
    {
        xhci_udev_io_reply_failed(req, status);
        xhci_reset_ep(udev, ep_index);
        return;
    }

    xhci_udev_io_reply_data(udev, req, status, act_len);
    if (req->iouh_Req.io_Command == UHCMD_CONTROLXFER && comp == COMP_SHORT_TX)
        xhci_ep_set_receiving_control_short(ep_ctx);
    else
        xhci_ep_set_idle(ep_ctx);
}

void ep_handle_rt_iso(struct IOUsbHWReq *req, ULONG act_len, struct ep_context *ep_ctx, struct usb_device *udev)
{
    struct xhci_ctrl *ctrl = udev->controller;

    if (req->iouh_Dir == UHDIR_IN)
    {
        if (act_len > 0)
            xhci_ep_rt_iso_in(ep_ctx, req, act_len);

        FreeVecPooled(ctrl->memoryPool, req->iouh_Data);
    }
    else
        xhci_ep_rt_iso_out(ep_ctx, req, act_len);

    /* RT ISO TDs clone IO requests; free them after completion to avoid leaks. */
    FreeVecPooled(ctrl->memoryPool, req);

    xhci_ep_schedule_rt_iso(ep_ctx);
}

static void ep_handle_receiving_control_short(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{
    (void)udev;
    (void)ep_ctx;
    (void)event;

    /* Short data stage, clear up additional status stage event */
    KprintfH("short-tx status event flags=%08lx status=%08lx\n",
             (ULONG)LE32(event->generic.field[3]),
             (ULONG)LE32(event->generic.field[2]));

    // no need to confirm slot and ep as this is done by event handler
    xhci_ep_set_idle(ep_ctx);
}

static void ep_handle_aborting(struct usb_device *udev, struct ep_context *ep_ctx, union xhci_trb *event)
{
    (void)ep_ctx;

    u32 flags = LE32(event->trans_event.flags);
    if (TRB_TO_SLOT_ID(flags) != udev->slot_id)
    {
        Kprintf("Expected a TRB for slot %ld, got %ld\n", udev->slot_id, TRB_TO_SLOT_ID(flags));
        return;
    }
    if (GET_COMP_CODE(LE32(event->trans_event.transfer_len)) != COMP_STOP)
    {
        Kprintf("Expected a TRB with STOP, got %ld\n", GET_COMP_CODE(LE32(event->trans_event.transfer_len)));
    }

    /* no state change - that is done by handle_abort_stop_ring */
}
