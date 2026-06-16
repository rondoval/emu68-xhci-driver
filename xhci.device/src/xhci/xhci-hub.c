// SPDX-License-Identifier: GPL-2.0-only
/*
 * USB HOST XHCI Controller stack — SuperSpeed hub emulation
 *
 * Based on xHCI host controller driver in linux-kernel by Sarah Sharp.
 */

#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME (*(struct ExecBase **)4UL)
#include <proto/exec.h>
#endif

#include <devices/hcd_api.h>

#include <xhci/xhci.h>
#include <xhci/xhci-regs.h>
#include <xhci/ch9.h>
#include <xhci/usb_defs.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci-hub.h>

#include <device.h>
#include <debug.h>
#include <bits.h>
#include <byteorder.h>
#include <memory.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-hub] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci-hub] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

static enum usb_device_speed xhci_hub_speed_from_port_status(u16 status)
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

static enum usb_device_speed xhci_hub_speed_from_ss_port_status(u16 status)
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

static BOOL xhci_hub_ss_port_ready_for_attach(u16 status, enum usb_device_speed speed)
{
    return (status & USB_PORT_STAT_CONNECTION) != 0 &&
           (status & USB_PORT_STAT_ENABLE) != 0 &&
           (status & USB_PORT_STAT_RESET) == 0 &&
           speed != USB_SPEED_UNKNOWN;
}

static void xhci_hub_cache_ss_hub_descriptor(struct usb_device *udev, struct usb_hub_descriptor *hub, u32 actual)
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

static void xhci_hub_set_ss_hub_depth(struct usb_device *udev)
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

static u32 xhci_hub_build_usb2_hub_descriptor(struct usb_device *udev, u8 *buf, const u32 max_len)
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

static void xhci_hub_map_ss_port_status(u16 *wStatus, u16 *wChange, enum usb_device_speed speed)
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

void xhci_hub_filter_ss_ep_companion_desc(struct USBIORequest *io)
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

BOOL xhci_hub_filter_emulated_ctrl_request(struct usb_device *udev, struct USBIORequest *io)
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

/* Translate SS hub descriptor request: modify request to ask for SS descriptor,
 * it will be translated back to USB 2.0 in the parse handler */
void xhci_hub_translate_descriptor_request(struct usb_device *udev, struct USBIORequest *io)
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

void xhci_hub_handle_get_descriptor(struct usb_device *udev, struct USBIORequest *io, u8 descriptorType)
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
        xhci_hub_cache_ss_hub_descriptor(udev, hub, io->actual_length);
        xhci_hub_set_ss_hub_depth(udev);

        /* If the stack requested USB 2.0 descriptor but we fetched SS, translate it */
        if (udev->ss_hub_emulation)
        {
            /* Build USB 2.0 descriptor from the SS descriptor we just cached */
            io->actual_length = xhci_hub_build_usb2_hub_descriptor(udev, (u8 *)io->data_buffer, io->data_buffer_length);
            KprintfH("SS hub addr=%lu: translated USB3 descriptor to USB2 format. Size %lu bytes\n", (ULONG)udev->virtual_address, (ULONG)io->actual_length);
        }
    }
}

void xhci_hub_handle_get_port_status(struct usb_device *udev, struct USBIORequest *io)
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
    enum usb_device_speed speed = (udev->ss_hub_emulation) ? xhci_hub_speed_from_ss_port_status(rawStatus) : xhci_hub_speed_from_port_status(wStatus);

    if (udev->ss_hub_emulation)
    {
        xhci_hub_map_ss_port_status(&wStatus, &wChange, speed);
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
        if (xhci_hub_ss_port_ready_for_attach(rawStatus, speed) &&
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
