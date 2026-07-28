// SPDX-License-Identifier: GPL-2.0-only
/*
 * USB HOST XHCI Controller stack — Link Power Management (LPM)
 *
 * Based on xHCI host controller driver in linux-kernel by Sarah Sharp,
 * and usbcore LPM (usb_set_lpm_parameters / usb_enable_lpm).
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
#include <xhci/ch9.h>
#include <xhci/usb_defs.h>
#include <xhci/xhci-context.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci-ring.h>
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-lpm.h>

#include <device.h>
#include <debug.h>
#include <bits.h>
#include <byteorder.h>
#include <memory.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-lpm] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef TRACE
#undef KprintfT
#define KprintfT(fmt, ...) PrintPistorm("[xhci-lpm] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

/* BESL selector (0-15) to microseconds — USB 2.0 LPM ECN Table; identical to
 * Linux xhci_besl_encoding[]. */
static const u32 besl_encoding[16] = {
    125, 150, 200, 300, 400, 500, 1000, 2000,
    3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000};

static inline u32 u32max(u32 a, u32 b) { return a > b ? a : b; }

/* TRUE if any active periodic (int/isoc) endpoint has a service interval <= the
 * given MEL (ns), in which case that link state must not be enabled (xHCI
 * 4.23.5.2; mirrors xhci_calculate_u1/u2_timeout ESIT guard). */
static BOOL xhci_lpm_esit_blocks(struct usb_device *udev, u32 mel_ns)
{
    struct usb_config *cfg = udev->active_config;
    if (!cfg)
        return FALSE;

    for (u8 i = 0; i < cfg->no_of_if; i++)
    {
        struct usb_interface_altsetting *alt = cfg->if_desc[i].active_altsetting;
        if (!alt)
            continue;
        for (u8 j = 0; j < alt->no_of_ep; j++)
        {
            struct usb_endpoint_descriptor *ep = &alt->ep_desc[j];
            if (!usb_endpoint_xfer_int(ep) && !usb_endpoint_xfer_isoc(ep))
                continue;
            u8 bi = ep->bInterval;
            if (bi == 0)
                continue;
            if (bi > 16) /* SS bInterval is 1..16; clamp malformed values */
                bi = 16;
            /* SS service interval = 2^(bInterval-1) microframes * 125us.
             * Compare as uframes vs mel/125000 to avoid 64-bit math. */
            u32 uframes = 1u << (bi - 1);
            if (uframes <= mel_ns / 125000u)
                return TRUE;
        }
    }
    return FALSE;
}

/* Returns the hub-encoded U1/U2 timeout (generic, non-Intel host path: the
 * timeout equals SEL), or USB3_LPM_DISABLED if the state should not be enabled.
 * Mirrors xhci_calculate_u1_timeout / xhci_calculate_u2_timeout. */
static u16 xhci_usb3_state_timeout(struct usb_device *udev, BOOL u2)
{
    u32 dev_exit = u2 ? (u32)udev->u2_dev_exit_lat : (u32)udev->u1_dev_exit_lat;
    if (dev_exit == 0)
        return USB3_LPM_DISABLED; /* device doesn't implement this link state */

    u32 mel_ns = u2 ? udev->u2_mel : udev->u1_mel;
    if (xhci_lpm_esit_blocks(udev, mel_ns))
        return USB3_LPM_DISABLED;

    u32 sel_ns = u2 ? udev->u2_sel : udev->u1_sel;
    u32 timeout;
    if (u2)
    {
        timeout = (sel_ns + (256u * 1000u - 1u)) / (256u * 1000u); /* 256us units */
        if (timeout == 0)
            timeout = 1;
        if (timeout > USB3_LPM_U2_MAX_TIMEOUT)
            return USB3_LPM_DISABLED;
    }
    else
    {
        timeout = (sel_ns + 999u) / 1000u; /* us */
        if (timeout == 0)
            timeout = 1;
        if (timeout > USB3_LPM_U1_MAX_TIMEOUT)
            return USB3_LPM_DISABLED;
    }
    return (u16)timeout;
}

/* Device Max Exit Latency (us) for the slot context (mirror xhci_calculate_mel). */
u32 xhci_calculate_mel(struct usb_device *udev)
{
    if (udev->speed < USB_SPEED_HIGH)
        return 0;

    if (udev->speed == USB_SPEED_HIGH)
    {
        /* Only BESL-capable hosts program MEL for USB2 L1 (mirror
         * xhci_set_usb2_hardware_lpm: MEL = besl_encoding[baseline]). */
        if (!udev->lpm_capable || !udev->usb2_hw_lpm_capable ||
            !udev->usb2_hw_lpm_besl_capable)
            return 0;
        u8 besl = udev->besl_baseline_valid ? udev->besl_baseline : XHCI_DEFAULT_BESL;
        return besl_encoding[besl & 0xfU];
    }

    /* USB3: MEL is the max over the states that will actually be enabled. */
    u32 mel_ns = 0;
    if (xhci_usb3_state_timeout(udev, FALSE) != USB3_LPM_DISABLED)
        mel_ns = u32max(mel_ns, udev->u1_mel);
    if (xhci_usb3_state_timeout(udev, TRUE) != USB3_LPM_DISABLED)
        mel_ns = u32max(mel_ns, udev->u2_mel);

    u32 mel_us = (mel_ns + 999u) / 1000u;
    if (mel_us > 0xffffU)
        mel_us = 0xffffU;
    KprintfT("Calculated MEL for addr %lu: %lu us (u1_mel=%lu u2_mel=%lu ns)\n",
             (ULONG)udev->virtual_address, (ULONG)mel_us,
             (ULONG)udev->u1_mel, (ULONG)udev->u2_mel);
    return mel_us;
}

/* HIRD/BESL value for USB2 PORTPMSC (mirror xhci_calculate_hird_besl). */
static u8 xhci_calculate_hird_besl(struct usb_device *udev)
{
    struct xhci_ctrl *ctrl = udev->controller;
    u32 u2del = (u32)ctrl->u2_host_exit_lat;
    u32 besl_host = 0;
    u32 besl_device = 0;

    if (udev->besl_supported)
    {
        for (besl_host = 0; besl_host < 16; besl_host++)
            if (besl_encoding[besl_host] >= u2del)
                break;
        if (udev->besl_baseline_valid)
            besl_device = udev->besl_baseline;
        else if (udev->besl_deep_valid)
            besl_device = udev->besl_deep;
    }
    else
    {
        if (u2del <= 50)
            besl_host = 0;
        else
            besl_host = (u2del - 51) / 75 + 1;
    }

    u32 besl = besl_host + besl_device;
    if (besl > 15)
        besl = 15;
    return (u8)besl;
}

/* TRUE if device-initiated U1/U2 entry is allowed: every periodic endpoint's
 * service interval must absorb the system exit latency (mirror
 * usb_device_may_initiate_lpm: reject if sel + 125us > interval). */
static BOOL xhci_lpm_may_initiate(struct usb_device *udev, BOOL u2)
{
    u32 sel_us = ((u2 ? udev->u2_sel : udev->u1_sel) + 999u) / 1000u;

    struct usb_config *cfg = udev->active_config;
    if (!cfg)
        return FALSE;

    for (u8 i = 0; i < cfg->no_of_if; i++)
    {
        struct usb_interface_altsetting *alt = cfg->if_desc[i].active_altsetting;
        if (!alt)
            continue;
        for (u8 j = 0; j < alt->no_of_ep; j++)
        {
            struct usb_endpoint_descriptor *ep = &alt->ep_desc[j];
            if (!usb_endpoint_xfer_int(ep) && !usb_endpoint_xfer_isoc(ep))
                continue;
            u8 bi = ep->bInterval;
            if (bi == 0)
                continue;
            if (bi > 16)
                bi = 16;
            u32 interval_us = (1u << (bi - 1)) * 125u;
            if (sel_us + 125u > interval_us)
                return FALSE;
        }
    }
    return TRUE;
}

/* Compute USB3 SEL/PEL/MEL (USB 3.1 Appendix C) and detect USB2 hardware LPM
 * eligibility.  Mirrors usb_set_lpm_parameters() + xhci_update_device().  Run
 * once after the BOS descriptor has been parsed. */
static void xhci_set_lpm_parameters(struct usb_device *udev)
{
    if (!udev || !udev->controller || !udev->lpm_capable || !udev->parent)
        return;
    struct xhci_ctrl *ctrl = udev->controller;

    if (udev->speed == USB_SPEED_HIGH)
    {
        /* USB2 hardware LPM: non-hub device directly on a root-hub port that
         * advertises HLC in its Supported-Protocol cap. */
        if (!udev->is_hub && udev->parent->parent == NULL)
        {
            u32 root_port = xhci_find_root_port(udev);
            BOOL hw_lpm = FALSE, besl_lpm = FALSE;
            xhci_roothub_port_lpm_caps(ctrl->root_hub, root_port, &hw_lpm, &besl_lpm);
            if (hw_lpm)
            {
                udev->usb2_hw_lpm_capable = TRUE;
                udev->usb2_hw_lpm_besl_capable = besl_lpm;
            }
        }
        return;
    }

    if (udev->speed < USB_SPEED_SUPER)
        return;

    struct usb_device *parent = udev->parent;
    BOOL is_root = (parent->parent == NULL);

    u32 udev_u1 = (u32)udev->u1_dev_exit_lat;
    u32 udev_u2 = (u32)udev->u2_dev_exit_lat;
    u32 hub_u1 = is_root ? (u32)ctrl->u1_host_exit_lat : (u32)parent->u1_dev_exit_lat;
    u32 hub_u2 = is_root ? (u32)ctrl->u2_host_exit_lat : (u32)parent->u2_dev_exit_lat;
    u32 parent_u1_mel = is_root ? 0 : parent->u1_mel;
    u32 parent_u2_mel = is_root ? 0 : parent->u2_mel;
    u32 parent_u1_pel = is_root ? 0 : parent->u1_pel;
    u32 parent_u2_pel = is_root ? 0 : parent->u2_pel;
    u32 hub_hdr_dec = is_root ? 0 : (u32)parent->ss_hub_desc.u.ss.bHubHdrDecLat;
    u32 hub_delay = is_root ? 0 : (u32)le16(parent->ss_hub_desc.u.ss.wHubDelay);

    u32 common = hub_hdr_dec * 100u + (hub_delay + USB_TP_TRANSMISSION_DELAY) * 2u + (is_root ? (USB_PING_RESPONSE_TIME + 2100u) : 0u);

    /* MEL (ns) */
    udev->u1_mel = parent_u1_mel + u32max(udev_u1, hub_u1) * 1000u + common;
    udev->u2_mel = parent_u2_mel + u32max(udev_u2, hub_u2) * 1000u + common;

    /* PEL (ns) */
    u32 u1_first = u32max(udev_u1, hub_u1) * 1000u;
    udev->u1_pel = u32max(u1_first, 1u * 1000u + parent_u1_pel); /* p2p U1 = 1us */

    u32 p2p_u2 = (hub_u2 > hub_u1) ? (1u + hub_u2 - hub_u1) : (1u + hub_u1);
    u32 u2_first = u32max(udev_u2, hub_u2) * 1000u;
    udev->u2_pel = u32max(u2_first, p2p_u2 * 1000u + parent_u2_pel);

    /* SEL (ns) */
    u32 num_hubs = 0;
    for (struct usb_device *p = udev->parent; p->parent; p = p->parent)
        num_hubs++;
    u32 sel_extra = (num_hubs > 0 ? 2100u + 250u * (num_hubs - 1u) : 0u) + 250u * num_hubs;
    udev->u1_sel = udev->u1_pel + sel_extra;
    udev->u2_sel = udev->u2_pel + sel_extra;

    KprintfT("LPM params addr %lu: U1 sel=%lu pel=%lu mel=%lu | U2 sel=%lu pel=%lu mel=%lu (ns)\n",
             (ULONG)udev->virtual_address,
             (ULONG)udev->u1_sel, (ULONG)udev->u1_pel, (ULONG)udev->u1_mel,
             (ULONG)udev->u2_sel, (ULONG)udev->u2_pel, (ULONG)udev->u2_mel);
}

/* Parse a fully-fetched BOS descriptor into the device's LPM fields and compute
 * the SEL/PEL/MEL parameters.  Extracted from the SET_CONFIGURATION BOS
 * pre-fetch (phase 2). */
void xhci_lpm_parse_bos_caps(struct usb_device *udev, const u8 *buf, u32 len)
{
    const struct usb_bos_descriptor *hdr = (const struct usb_bos_descriptor *)buf;
    if (!buf || hdr->bDescriptorType != USB_DT_BOS)
        return;

    KprintfT("BOS descriptor total length is %lu, num device caps is %lu\n",
             (ULONG)le16(hdr->wTotalLength), (ULONG)hdr->bNumDeviceCaps);
    const u8 *cursor = buf + sizeof(struct usb_bos_descriptor);
    const u8 *end = buf + (u32)le16(hdr->wTotalLength);
    if (end > buf + len)
        end = buf + len;

    while (cursor + 3u <= end)
    {
        u8 caplen = cursor[0];
        u8 dtype = cursor[1];
        u8 captype = cursor[2];
        if (caplen < 3u || cursor + caplen > end)
            break;

        if (dtype == USB_DT_DEVICE_CAPABILITY)
        {
            if (captype == USB_CAP_DESC_USB20_EXTENSION &&
                caplen >= (u8)sizeof(struct usb_2_0_extension_capability_descriptor))
            {
                const struct usb_2_0_extension_capability_descriptor *ext =
                    (const struct usb_2_0_extension_capability_descriptor *)cursor;
                u32 att = le32(ext->bmAttributes);
                udev->lpm_capable = (att & USB_20_EXTENSION_ATT_LINK_POWER_MANAGEMENT) ? TRUE : FALSE;
                udev->besl_supported = (att & USB_20_EXTENSION_ATT_BESL_SUPPORTED) ? TRUE : FALSE;
                udev->besl_baseline_valid = (att & USB_20_EXTENSION_ATT_BESL_BASELINE_VALID) ? TRUE : FALSE;
                udev->besl_deep_valid = (att & USB_20_EXTENSION_ATT_BESL_DEEP_VALID) ? TRUE : FALSE;
                if (udev->besl_baseline_valid)
                    udev->besl_baseline = (u8)USB_20_EXTENSION_ATT_BESL_BASELINE(att);
                if (udev->besl_deep_valid)
                    udev->besl_deep = (u8)USB_20_EXTENSION_ATT_BESL_DEEP(att);
                KprintfT("USB2 Ext Cap: lpm=%ld besl=%ld baseline=%lu(v=%ld) deep=%lu(v=%ld)\n",
                         (LONG)udev->lpm_capable, (LONG)udev->besl_supported,
                         (ULONG)udev->besl_baseline, (LONG)udev->besl_baseline_valid,
                         (ULONG)udev->besl_deep, (LONG)udev->besl_deep_valid);
            }
            else if (captype == USB_CAP_DESC_SS_USB_DEVICE &&
                     caplen >= (u8)sizeof(struct usb_ss_device_capability_descriptor))
            {
                const struct usb_ss_device_capability_descriptor *ss =
                    (const struct usb_ss_device_capability_descriptor *)cursor;
                udev->u1_dev_exit_lat = ss->bU1DevExitLat;
                udev->u2_dev_exit_lat = le16(ss->wU2DevExitLat);
                udev->ltm_capable = (ss->bmAttributes & USB_SS_DEVICE_ATT_LATENCY_TOLERANCE_MESSAGES) != 0;
                KprintfT("SS Dev Cap: U1=%lu U2=%lu\n",
                         (ULONG)udev->u1_dev_exit_lat, (ULONG)udev->u2_dev_exit_lat);
            }
        }

        cursor += caplen;
    }
    KprintfT("Finished parsing BOS device capabilities for addr=%lu\n", (ULONG)udev->virtual_address);
    KprintfT("Device LPM capability: %ld, BESL support: %ld, BESL baseline: %lu\n",
             (LONG)udev->lpm_capable, (LONG)udev->besl_supported, (ULONG)udev->besl_baseline);
    KprintfT("Device U1 exit latency: %lu us, U2 exit latency: %lu us\n",
             (ULONG)udev->u1_dev_exit_lat, (ULONG)udev->u2_dev_exit_lat);

    /* For SS devices, LPM capability comes from the SS Device Cap exit
     * latencies (the USB2 Ext cap LPM bit only governs USB2 L1).  Mirror
     * usb_device_supports_lpm(): non-zero exit latency and an LPM-capable
     * path to the root hub.  (USB2 lpm_capable was set from the Ext cap.) */
    if (udev->speed >= USB_SPEED_SUPER)
        udev->lpm_capable = ((udev->u1_dev_exit_lat || udev->u2_dev_exit_lat) &&
                             (!udev->parent || udev->parent->parent == NULL ||
                              udev->parent->lpm_capable))
                                ? TRUE
                                : FALSE;

    /* Now that exit latencies are known, compute the USB3 SEL/PEL/MEL
     * parameters (no-op for non-SS devices). */
    xhci_set_lpm_parameters(udev);
}

/* Send SET_SEL (USB 3.2 9.4.12) to inform the device of host/path exit latencies.
 * Returns TRUE if the transfer was submitted (and device-initiated LPM may be
 * enabled), FALSE if SEL/PEL exceed the field range or submission failed. */
static BOOL xhci_udev_send_set_sel(struct usb_device *udev)
{
    if (!udev || !udev->controller || udev->speed < USB_SPEED_SUPER || !udev->lpm_capable)
        return FALSE;

    struct xhci_ctrl *ctrl = udev->controller;

    /* Convert ns -> us (round up) */
    u32 u1_sel = (udev->u1_sel + 999u) / 1000u;
    u32 u1_pel = (udev->u1_pel + 999u) / 1000u;
    u32 u2_sel = (udev->u2_sel + 999u) / 1000u;
    u32 u2_pel = (udev->u2_pel + 999u) / 1000u;

    if (u1_sel > USB3_LPM_MAX_U1_SEL_PEL || u1_pel > USB3_LPM_MAX_U1_SEL_PEL ||
        u2_sel > USB3_LPM_MAX_U2_SEL_PEL || u2_pel > USB3_LPM_MAX_U2_SEL_PEL)
    {
        Kprintf("SET_SEL skipped for addr %lu: SEL/PEL too large (u1 %lu/%lu u2 %lu/%lu us)\n",
                (ULONG)udev->virtual_address, (ULONG)u1_sel, (ULONG)u1_pel,
                (ULONG)u2_sel, (ULONG)u2_pel);
        return FALSE;
    }

    u8 *buf = dma_alloc(ctrl->dmaPool, DMA_ALIGN_MIN, sizeof(struct usb_set_sel_req));
    struct USBIORequest *io = pool_zalloc(ctrl->metaPool, sizeof(*io));
    if (!io || !buf)
    {
        Kprintf("xhci_udev_send_set_sel: alloc failed\n");
        if (buf)
            dma_free(ctrl->dmaPool, buf);
        if (io)
            pool_free(ctrl->metaPool, io);
        return FALSE;
    }

    struct usb_set_sel_req *sel = (struct usb_set_sel_req *)buf;
    sel->u1_sel = (u8)u1_sel;
    sel->u1_pel = (u8)u1_pel;
    sel->u2_sel = le16((u16)u2_sel);
    sel->u2_pel = le16((u16)u2_pel);

    io->req.io_Command = CMD_REQUEST_CONTROL;
    io->req.io_Flags = IOF_QUICK;
    io->driver_private_flags = REQ_INTERNAL | REQ_ENQUEUED | REQ_SET_SEL;

    io->setup.bmRequestType = USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE;
    io->setup.bRequest = USB_REQ_SET_SEL;
    io->setup.wValue = 0;
    io->setup.wIndex = 0;
    io->setup.wLength = le16((u16)sizeof(struct usb_set_sel_req));

    io->virtual_address = udev->virtual_address;
    io->data_buffer = buf;
    io->data_buffer_length = (u32)sizeof(struct usb_set_sel_req);
    io->direction = DIRECTION_OUT;

    s8 ring_ret = xhci_ring_enqueue_td(udev, io, 1000, FALSE);
    if (ring_ret != ERR_NO_ERROR)
    {
        Kprintf("xhci_udev_send_set_sel: enqueue failed (%ld)\n", (LONG)ring_ret);
        dma_free(ctrl->dmaPool, buf);
        pool_free(ctrl->metaPool, io);
        return FALSE;
    }

    KprintfT("SET_SEL addr %lu: u1 sel=%lu pel=%lu, u2 sel=%lu pel=%lu (us)\n",
             (ULONG)udev->virtual_address, (ULONG)u1_sel, (ULONG)u1_pel,
             (ULONG)u2_sel, (ULONG)u2_pel);
    return TRUE;
}

/* Enable device-initiated U1 or U2 transitions via SET_FEATURE.  Fire-and-forget. */
static void xhci_udev_set_device_lpm(struct usb_device *udev, BOOL u2)
{
    u8 feature = u2 ? USB_DEVICE_U2_ENABLE : USB_DEVICE_U1_ENABLE;
    xhci_udev_send_control_request(udev, 0,
                                   USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE,
                                   USB_REQ_SET_FEATURE,
                                   feature /* wValue */,
                                   0 /* wIndex */,
                                   0 /* wLength */,
                                   FALSE /* send now */);
    KprintfT("SET_FEATURE %s_ENABLE addr %lu\n", u2 ? "U2" : "U1", (ULONG)udev->virtual_address);
}

/* Enable device-initiated Latency Tolerance Messaging via SET_FEATURE.
 * Fire-and-forget; the xHC consumes the resulting LTM packets in hardware
 * (HCC_LTC) and uses the device's BELT for its U-state timing. */
static void xhci_udev_set_device_ltm(struct usb_device *udev)
{
    xhci_udev_send_control_request(udev, 0,
                                   USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE,
                                   USB_REQ_SET_FEATURE,
                                   USB_DEVICE_LTM_ENABLE /* wValue */,
                                   0 /* wIndex */,
                                   0 /* wLength */,
                                   FALSE /* send now */);
    Kprintf("SET_FEATURE LTM_ENABLE addr %lu\n", (ULONG)udev->virtual_address);
}

/* Program the U1/U2 inactivity timeout on the device's immediate parent-hub
 * downstream port.  A device directly on the root hub writes the root port's
 * PORTPMSC register; a device behind an external hub gets a
 * SetPortFeature(U1/U2_TIMEOUT) control transfer sent to that hub, which programs
 * its own downstream port (mirrors usb_set_lpm_timeout()).  timeout==0 disables
 * the state.  Fire-and-forget for the external-hub case. */
static void xhci_udev_set_port_lpm_timeout(struct usb_device *udev, BOOL u2, u16 timeout)
{
    if (!udev || !udev->controller)
        return;

    struct usb_device *parent = udev->parent;
    if (!parent)
        return;

    if (parent->parent == NULL)
    {
        /* Direct root-hub child: write the root port PORTPMSC directly. */
        xhci_roothub_set_usb3_port_timeout(udev->controller->root_hub,
                                           udev->parent_port, u2, timeout);
    }
    else
    {
        /* Behind an external hub: ask the hub to program its downstream port.
         * wValue = feature selector, wIndex = (timeout << 8) | port. */
        u8 feature = u2 ? USB_SS_PORT_FEAT_U2_TIMEOUT : USB_SS_PORT_FEAT_U1_TIMEOUT;
        u16 wIndex = (u16)(((u16)timeout << 8) | (u16)udev->parent_port);
        xhci_udev_send_control_request(parent, 0,
                                       USB_DIR_OUT | USB_RT_PORT,
                                       USB_REQ_SET_FEATURE,
                                       feature /* wValue */, wIndex,
                                       0 /* wLength */, FALSE /* send now */);
        Kprintf("SetPortFeature %s_TIMEOUT=%lu to hub addr %lu port %lu (for dev addr %lu)\n",
                u2 ? "U2" : "U1", (ULONG)timeout, (ULONG)parent->virtual_address,
                (ULONG)udev->parent_port, (ULONG)udev->virtual_address);
    }
}

/* Decide USB2 hardware-LPM (L1) policy for the device and program its root-hub
 * port.  Mirrors xhci_set_usb2_hardware_lpm(enable=1); the register writes live
 * in xhci_roothub_set_usb2_hw_lpm(). */
static void xhci_usb2_set_hw_lpm(struct usb_device *udev)
{
    struct xhci_ctrl *ctrl = udev->controller;
    if (udev->speed != USB_SPEED_HIGH || !udev->lpm_capable || !udev->usb2_hw_lpm_capable)
        return;
    if (udev->is_hub || !udev->parent || udev->parent->parent != NULL)
        return;

    BOOL besl_mode = udev->usb2_hw_lpm_besl_capable;
    u8 hird;
    u8 besld = 0;
    if (besl_mode)
    {
        hird = udev->besl_baseline_valid ? udev->besl_baseline : XHCI_DEFAULT_BESL;
        besld = udev->besl_deep_valid ? udev->besl_deep : 0;
    }
    else
    {
        hird = xhci_calculate_hird_besl(udev);
    }

    u32 root_port = xhci_find_root_port(udev);
    xhci_roothub_set_usb2_hw_lpm(ctrl->root_hub, root_port, hird, udev->slot_id,
                                 besl_mode, besld, XHCI_L1_TIMEOUT);

    KprintfT("USB2 HW LPM enabled: port %lu slot %lu hird=%lu besl_cap=%ld\n",
             (ULONG)root_port, (ULONG)udev->slot_id, (ULONG)hird, (LONG)besl_mode);
}

/* Orchestrate the LPM enable sequence (mirror usb_enable_lpm).  Run once, after
 * the SET_CONFIGURATION control transfer has completed on the wire: the device
 * must be in the Configured state or it rejects SET_FEATURE(U1/U2_ENABLE) with
 * a Request Error (USB 3.2 9.4.9).  The sequence continues in
 * xhci_lpm_enable_stage2() once the MEL Evaluate Context completes, and
 * device-initiated enable chains from the SET_SEL completion
 * (xhci_lpm_devinit_enable). */
void xhci_lpm_enable(struct usb_device *udev)
{
    if (!udev)
        return;

    /* LTM is independent of the U1/U2 policy (mirror usb_enable_ltm): enable
     * it for any configured SS device that advertises it, when the
     * controller consumes LTM packets (HCC_LTC). */
    if (!udev->ltm_setup_done && udev->speed >= USB_SPEED_SUPER &&
        udev->ltm_capable && udev->controller->ltc_supported)
    {
        udev->ltm_setup_done = TRUE;
        xhci_udev_set_device_ltm(udev);
    }

    if (!udev->lpm_capable || udev->lpm_setup_done)
        return;

    if (udev->speed == USB_SPEED_HIGH)
    {
        xhci_usb2_set_hw_lpm(udev);
        udev->lpm_setup_done = TRUE;
        return;
    }

    if (udev->speed < USB_SPEED_SUPER)
        return;

    udev->u1_timeout = xhci_usb3_state_timeout(udev, FALSE);
    udev->u2_timeout = xhci_usb3_state_timeout(udev, TRUE);
    udev->lpm_setup_done = TRUE;

    if (udev->u1_timeout == USB3_LPM_DISABLED && udev->u2_timeout == USB3_LPM_DISABLED)
        return;

    /* MEL must be latched by the controller before the port timeouts arm
     * (xHCI 4.23.5.2); handle_config_ep() resumes with stage 2 on success
     * (UDEV_OP_EVENT_MEL_EVAL_DONE). */
    if (!xhci_udev_op_begin(udev, UDEV_OP_LPM_ENABLE, NULL))
        return;
    xhci_evaluate_mel(udev);
}

/* Continue LPM enable after the MEL Evaluate Context succeeded: arm the parent
 * port's U1/U2 inactivity timeouts (root port PORTPMSC for a root-hub child,
 * SetPortFeature to the external hub otherwise) and send SET_SEL, whose
 * completion enables device-initiated entry.  Returns TRUE if SET_SEL was
 * submitted (the op stays alive for UDEV_OP_EVENT_SET_SEL_DONE). */
BOOL xhci_lpm_enable_stage2(struct usb_device *udev)
{
    if (!udev)
        return FALSE;

    if (udev->u1_timeout != USB3_LPM_DISABLED)
        xhci_udev_set_port_lpm_timeout(udev, FALSE, udev->u1_timeout);

    if (udev->u2_timeout != USB3_LPM_DISABLED)
        xhci_udev_set_port_lpm_timeout(udev, TRUE, udev->u2_timeout);

    return xhci_udev_send_set_sel(udev);
}

/* Enable device-initiated U1/U2 (SET_FEATURE) for the states whose port timeout
 * was enabled.  Called from the SET_SEL completion path, so the device is
 * configured and knows the exit latencies. */
void xhci_lpm_devinit_enable(struct usb_device *udev)
{
    if (!udev || udev->speed < USB_SPEED_SUPER || !udev->lpm_capable)
        return;

    if (udev->u1_timeout != USB3_LPM_DISABLED && xhci_lpm_may_initiate(udev, FALSE))
        xhci_udev_set_device_lpm(udev, FALSE);

    if (udev->u2_timeout != USB3_LPM_DISABLED && xhci_lpm_may_initiate(udev, TRUE))
        xhci_udev_set_device_lpm(udev, TRUE);
}

/* Tear down LPM when a device disconnects.  Mirrors usb_disable_device(): on a
 * physical disconnect Linux clears only USB2 hardware LPM (a direct root-port
 * register write via usb_disable_usb2_hardware_lpm()).  USB3 timeout teardown
 * short-circuits in usb_disable_lpm() because the device is already NOTATTACHED
 * (state < CONFIGURED), so we likewise leave USB3 PORTPMSC timeouts alone - they
 * are inert without a link and are overwritten on the next enumerate - and never
 * issue control transfers to a (possibly already-gone) external hub. */
void xhci_lpm_disable(struct usb_device *udev)
{
    if (!udev || !udev->lpm_setup_done)
        return;

    /* USB2 hardware LPM is only ever enabled for an HS device directly on a
     * root-hub port, so this clear is always a safe local register write. */
    if (udev->speed == USB_SPEED_HIGH && udev->usb2_hw_lpm_capable &&
        udev->parent && udev->parent->parent == NULL)
        xhci_roothub_clear_usb2_hw_lpm(udev->controller->root_hub, xhci_find_root_port(udev));

    udev->lpm_setup_done = FALSE;
}
