// SPDX-License-Identifier: GPL-2.0-only
/*
 * USB HOST XHCI Controller stack — Link Power Management (LPM)
 *
 * Controller-side LPM only: SEL/PEL/MEL/timeout computation, the MEL Evaluate
 * Context, and USB2 hardware LPM (L1) register programming.  The device/hub
 * control transfers (SET_SEL, SET_FEATURE(U1/U2/LTM_ENABLE), SetPortFeature
 * port timeouts) are issued by the stack — NSCMD_USB_SET_LINK_POWER returns the
 * computed wire parameters for it to use.
 */

#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME (*(struct ExecBase **)4UL)
#include <proto/exec.h>
#endif


#include <xhci/xhci.h>
#include <xhci/ch9.h>
#include <xhci/usb_defs.h>
#include <xhci/xhci-context.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-lpm.h>

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

/* The parent HUB device, or NULL when the device sits on a root port. */
static struct usb_device *xhci_lpm_parent_hub(struct usb_device *udev)
{
    return udev->parent;
}

/* Smallest service interval (in 125µs microframes) over the device's live
 * periodic endpoints, from the hardware endpoint contexts (Interval is the
 * normalized 2^n exponent there); 0 = no periodic endpoints. */
static u32 xhci_lpm_min_periodic_uframes(struct usb_device *udev)
{
    u32 min_uframes = 0;
    for (u8 ep_index = 1; ep_index < USB_MAX_ENDPOINT_CONTEXTS; ++ep_index)
    {
        if (!udev->ep_context[ep_index])
            continue;
        u32 usb_type = xhci_read_hw_ep_type(udev, ep_index) & USB_ENDPOINT_XFERTYPE_MASK;
        if (usb_type != USB_ENDPOINT_XFER_INT && usb_type != USB_ENDPOINT_XFER_ISOC)
            continue; /* also skips disabled contexts (hw type 0) */
        u32 exponent = xhci_read_hw_ep_interval(udev, ep_index);
        u32 uframes = 1u << (exponent > 15u ? 15u : exponent);
        if (min_uframes == 0 || uframes < min_uframes)
            min_uframes = uframes;
    }
    return min_uframes;
}

/* TRUE if any active periodic (int/isoc) endpoint has a service interval <= the
 * given MEL (ns), in which case that link state must not be enabled (xHCI
 * 4.23.5.2; mirrors xhci_calculate_u1/u2_timeout ESIT guard). */
static BOOL xhci_lpm_esit_blocks(struct usb_device *udev, u32 mel_ns)
{
    u32 min_uframes = xhci_lpm_min_periodic_uframes(udev);
    /* compare as uframes vs mel/125000 to avoid 64-bit math */
    return min_uframes != 0 && min_uframes <= mel_ns / 125000u;
}

/* Returns the hub-encoded U1/U2 timeout (generic, non-Intel host path: the
 * timeout equals SEL), or USB3_LPM_DISABLED if the state should not be enabled.
 * Mirrors xhci_calculate_u1_timeout / xhci_calculate_u2_timeout. */
static u16 xhci_usb3_state_timeout(struct usb_device *udev, BOOL u2)
{
    u32 dev_exit = u2 ? (u32)udev->lpm.u2_dev_exit_lat : (u32)udev->lpm.u1_dev_exit_lat;
    if (dev_exit == 0)
        return USB3_LPM_DISABLED; /* device doesn't implement this link state */

    u32 mel_ns = u2 ? udev->lpm.u2_mel : udev->lpm.u1_mel;
    if (xhci_lpm_esit_blocks(udev, mel_ns))
        return USB3_LPM_DISABLED;

    u32 sel_ns = u2 ? udev->lpm.u2_sel : udev->lpm.u1_sel;
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
static u32 xhci_calculate_mel(struct usb_device *udev)
{
    if (udev->lpm.mel_override_us) /* NSCMD_USB_SET_LINK_POWER stack override */
        return udev->lpm.mel_override_us;

    if (udev->speed < USB_SPEED_HIGH)
        return 0;

    if (udev->speed == USB_SPEED_HIGH)
    {
        /* Only BESL-capable hosts program MEL for USB2 L1 (mirror
         * xhci_set_usb2_hardware_lpm: MEL = besl_encoding[baseline]). */
        if (!udev->lpm.capable || !udev->lpm.usb2_hw_lpm_capable ||
            !udev->lpm.usb2_hw_lpm_besl_capable)
            return 0;
        u8 besl = udev->lpm.besl_baseline_valid ? udev->lpm.besl_baseline : XHCI_DEFAULT_BESL;
        return besl_encoding[besl & 0xfU];
    }

    /* USB3: MEL is the max over the states that will actually be enabled. */
    u32 mel_ns = 0;
    if (xhci_usb3_state_timeout(udev, FALSE) != USB3_LPM_DISABLED)
        mel_ns = u32max(mel_ns, udev->lpm.u1_mel);
    if (xhci_usb3_state_timeout(udev, TRUE) != USB3_LPM_DISABLED)
        mel_ns = u32max(mel_ns, udev->lpm.u2_mel);

    u32 mel_us = (mel_ns + 999u) / 1000u;
    if (mel_us > 0xffffU)
        mel_us = 0xffffU;
    KprintfT("Calculated MEL for slot %lu: %lu us (u1_mel=%lu u2_mel=%lu ns)\n",
             (ULONG)udev->slot_id, (ULONG)mel_us,
             (ULONG)udev->lpm.u1_mel, (ULONG)udev->lpm.u2_mel);
    return mel_us;
}

/* HIRD/BESL value for USB2 PORTPMSC (mirror xhci_calculate_hird_besl). */
static u8 xhci_calculate_hird_besl(struct usb_device *udev)
{
    struct xhci_ctrl *ctrl = udev->controller;
    u32 u2del = (u32)ctrl->u2_host_exit_lat;
    u32 besl_host = 0;
    u32 besl_device = 0;

    if (udev->lpm.besl_supported)
    {
        for (besl_host = 0; besl_host < 16; besl_host++)
            if (besl_encoding[besl_host] >= u2del)
                break;
        if (udev->lpm.besl_baseline_valid)
            besl_device = udev->lpm.besl_baseline;
        else if (udev->lpm.besl_deep_valid)
            besl_device = udev->lpm.besl_deep;
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
    u32 sel_us = ((u2 ? udev->lpm.u2_sel : udev->lpm.u1_sel) + 999u) / 1000u;

    u32 min_uframes = xhci_lpm_min_periodic_uframes(udev);
    if (min_uframes == 0)
        return TRUE; /* no periodic endpoints */

    return sel_us + 125u <= min_uframes * 125u;
}

/* Compute USB3 SEL/PEL/MEL (USB 3.1 Appendix C) and detect USB2 hardware LPM
 * eligibility.  Mirrors usb_set_lpm_parameters() + xhci_update_device().  Run
 * once after the BOS descriptor has been parsed. */
static void xhci_set_lpm_parameters(struct usb_device *udev)
{
    if (!udev || !udev->controller || !udev->lpm.capable)
        return;
    struct xhci_ctrl *ctrl = udev->controller;
    struct usb_device *hub = xhci_lpm_parent_hub(udev);

    if (udev->speed == USB_SPEED_HIGH)
    {
        /* USB2 hardware LPM: non-hub device directly on a root-hub port that
         * advertises HLC in its Supported-Protocol cap. */
        if (!udev->is_hub && hub == NULL)
        {
            u32 root_port = xhci_find_root_port(udev);
            BOOL hw_lpm = FALSE, besl_lpm = FALSE;
            xhci_roothub_port_lpm_caps(ctrl->root_hub, root_port, &hw_lpm, &besl_lpm);
            if (hw_lpm)
            {
                udev->lpm.usb2_hw_lpm_capable = TRUE;
                udev->lpm.usb2_hw_lpm_besl_capable = besl_lpm;
            }
        }
        return;
    }

    if (udev->speed < USB_SPEED_SUPER)
        return;

    BOOL is_root = (hub == NULL);

    u32 udev_u1 = (u32)udev->lpm.u1_dev_exit_lat;
    u32 udev_u2 = (u32)udev->lpm.u2_dev_exit_lat;
    u32 hub_u1 = is_root ? (u32)ctrl->u1_host_exit_lat : (u32)hub->lpm.u1_dev_exit_lat;
    u32 hub_u2 = is_root ? (u32)ctrl->u2_host_exit_lat : (u32)hub->lpm.u2_dev_exit_lat;
    u32 parent_u1_mel = is_root ? 0 : hub->lpm.u1_mel;
    u32 parent_u2_mel = is_root ? 0 : hub->lpm.u2_mel;
    u32 parent_u1_pel = is_root ? 0 : hub->lpm.u1_pel;
    u32 parent_u2_pel = is_root ? 0 : hub->lpm.u2_pel;
    u32 hub_hdr_dec = is_root ? 0 : (u32)hub->hub_hdr_dec_lat;
    u32 hub_delay = is_root ? 0 : (u32)hub->hub_delay;

    u32 common = hub_hdr_dec * 100u + (hub_delay + USB_TP_TRANSMISSION_DELAY) * 2u + (is_root ? (USB_PING_RESPONSE_TIME + 2100u) : 0u);

    /* MEL (ns) */
    udev->lpm.u1_mel = parent_u1_mel + u32max(udev_u1, hub_u1) * 1000u + common;
    udev->lpm.u2_mel = parent_u2_mel + u32max(udev_u2, hub_u2) * 1000u + common;

    /* PEL (ns) */
    u32 u1_first = u32max(udev_u1, hub_u1) * 1000u;
    udev->lpm.u1_pel = u32max(u1_first, 1u * 1000u + parent_u1_pel); /* p2p U1 = 1us */

    u32 p2p_u2 = (hub_u2 > hub_u1) ? (1u + hub_u2 - hub_u1) : (1u + hub_u1);
    u32 u2_first = u32max(udev_u2, hub_u2) * 1000u;
    udev->lpm.u2_pel = u32max(u2_first, p2p_u2 * 1000u + parent_u2_pel);

    /* SEL (ns) */
    u32 num_hubs = 0;
    for (struct usb_device *p = hub; p; p = xhci_lpm_parent_hub(p))
        num_hubs++;
    u32 sel_extra = (num_hubs > 0 ? 2100u + 250u * (num_hubs - 1u) : 0u) + 250u * num_hubs;
    udev->lpm.u1_sel = udev->lpm.u1_pel + sel_extra;
    udev->lpm.u2_sel = udev->lpm.u2_pel + sel_extra;

    KprintfT("LPM params slot %lu: U1 sel=%lu pel=%lu mel=%lu | U2 sel=%lu pel=%lu mel=%lu (ns)\n",
             (ULONG)udev->slot_id,
             (ULONG)udev->lpm.u1_sel, (ULONG)udev->lpm.u1_pel, (ULONG)udev->lpm.u1_mel,
             (ULONG)udev->lpm.u2_sel, (ULONG)udev->lpm.u2_pel, (ULONG)udev->lpm.u2_mel);
}

/* Decide USB2 hardware-LPM (L1) policy for the device and program its root-hub
 * port.  Mirrors xhci_set_usb2_hardware_lpm(enable=1); the register writes live
 * in xhci_roothub_set_usb2_hw_lpm(). */
static void xhci_usb2_set_hw_lpm(struct usb_device *udev)
{
    struct xhci_ctrl *ctrl = udev->controller;
    if (udev->speed != USB_SPEED_HIGH || !udev->lpm.capable || !udev->lpm.usb2_hw_lpm_capable)
        return;
    if (udev->is_hub || xhci_lpm_parent_hub(udev) != NULL)
        return;

    BOOL besl_mode = udev->lpm.usb2_hw_lpm_besl_capable;
    u8 hird;
    u8 besld = 0;
    if (besl_mode)
    {
        hird = udev->lpm.besl_baseline_valid ? udev->lpm.besl_baseline : XHCI_DEFAULT_BESL;
        besld = udev->lpm.besl_deep_valid ? udev->lpm.besl_deep : 0;
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

/* NSCMD_USB_SET_LINK_POWER entry: adopt the stack-parsed BOS facts + policy,
 * program the controller-side LPM state, and hand the computed wire parameters
 * back to the stack in *op (the stack issues SET_SEL / SET_FEATURE / port
 * SetPortFeature itself).  The stack calls this only after the wire
 * SET_CONFIGURATION completed; a disabled policy switch withholds the
 * capability fact so the state is never armed.  Safe to re-issue.
 *
 * Returns TRUE when a MEL Evaluate Context must be issued (an SS U1/U2 state is
 * enabled) — the caller issues it and replies the op from its completion, so
 * the controller state is latched before the stack arms the port timeouts
 * (xHCI 4.23.5.2).  Returns FALSE when the op can reply synchronously (USB2
 * hardware LPM, LTM-only, or nothing to arm). */
BOOL xhci_lpm_set_link_power(struct usb_device *udev, struct UhcdSetLinkPower *op)
{
    udev->lpm.u1_dev_exit_lat = op->slo_U1Enable ? (u8)op->slo_U1DevExitLat : 0;
    udev->lpm.u2_dev_exit_lat = op->slo_U2Enable ? op->slo_U2DevExitLat : 0;
    udev->lpm.ltm_capable = (op->slo_Flags & UHCD_LPF_LTM) ? TRUE : FALSE;
    udev->lpm.besl_supported = (op->slo_Flags & UHCD_LPF_BESL) ? TRUE : FALSE;
    udev->lpm.besl_baseline_valid = (op->slo_Flags & UHCD_LPF_BESL_BASELINE) ? TRUE : FALSE;
    udev->lpm.besl_deep_valid = (op->slo_Flags & UHCD_LPF_BESL_DEEP) ? TRUE : FALSE;
    udev->lpm.besl_baseline = op->slo_BeslBaseline & 0xfU;
    udev->lpm.besl_deep = op->slo_BeslDeep & 0xfU;
    udev->lpm.u1_timeout_override = op->slo_U1Timeout;
    udev->lpm.u2_timeout_override = op->slo_U2Timeout;
    udev->lpm.mel_override_us = op->slo_MaxExitLatency;

    if (udev->speed >= USB_SPEED_SUPER)
    {
        /* mirror usb_device_supports_lpm(): non-zero exit latency and an
         * LPM-capable path to the root (a hub's facts arrive before its
         * children configure, so the chain is populated top-down) */
        struct usb_device *hub = xhci_lpm_parent_hub(udev);
        udev->lpm.capable = ((udev->lpm.u1_dev_exit_lat || udev->lpm.u2_dev_exit_lat) &&
                             (hub == NULL || hub->lpm.capable))
                                ? TRUE
                                : FALSE;
    }
    else
    {
        udev->lpm.capable = (op->slo_Flags & UHCD_LPF_USB2_LPM) ? TRUE : FALSE;
    }

    /* Policy withheld (the stack zeroed the enables and dropped the capability
     * flags) or the device lost the capability: tear down whatever an earlier op
     * armed.  Without this the HS branch below early-returns and leaves
     * PORTPMSC.HLE set with a stale L1DS pointing at this slot.  Runs before
     * xhci_set_lpm_parameters(), which early-returns on !capable and so leaves
     * usb2_hw_lpm_capable - the fact xhci_lpm_disable() needs - intact. */
    if (!udev->lpm.capable && udev->lpm.setup_done)
        xhci_lpm_disable(udev);

    xhci_set_lpm_parameters(udev);

    /* Outputs default to "nothing to issue"; a repeated op re-arms cleanly. */
    op->slo_OutU1Timeout = 0;
    op->slo_OutU2Timeout = 0;
    op->slo_OutU1Sel = 0;
    op->slo_OutU1Pel = 0;
    op->slo_OutU2Sel = 0;
    op->slo_OutU2Pel = 0;
    op->slo_OutFlags = 0;

    /* LTM is independent of the U1/U2 policy (mirror usb_enable_ltm): enable it
     * for any configured SS device that advertises it, when the controller
     * consumes LTM packets (HCC_LTC).  Emitted as a device SET_FEATURE by the
     * stack. */
    if (udev->speed >= USB_SPEED_SUPER && udev->lpm.ltm_capable &&
        udev->controller->ltc_supported)
        op->slo_OutFlags |= UHCD_LPO_LTM;

    /* USB2 hardware LPM (L1): register-only, no control transfer, no MEL eval. */
    if (udev->speed == USB_SPEED_HIGH)
    {
        xhci_usb2_set_hw_lpm(udev);
        /* "armed", not "visited": a withheld policy must not leave setup_done
         * set, or the teardown above re-runs the register clear on every
         * subsequent op. */
        udev->lpm.setup_done = udev->lpm.capable;
        return FALSE;
    }

    if (udev->speed < USB_SPEED_SUPER || !udev->lpm.capable)
        return FALSE; /* LTM (if any) still replies synchronously */

    /* SS U1/U2: compute the effective inactivity timeouts and SEL/PEL. */
    u16 u1_to = udev->lpm.u1_timeout_override ? udev->lpm.u1_timeout_override
                                          : xhci_usb3_state_timeout(udev, FALSE);
    u16 u2_to = udev->lpm.u2_timeout_override ? udev->lpm.u2_timeout_override
                                          : xhci_usb3_state_timeout(udev, TRUE);
    udev->lpm.setup_done = TRUE;

    BOOL u1_on = (u1_to != USB3_LPM_DISABLED);
    BOOL u2_on = (u2_to != USB3_LPM_DISABLED);

    op->slo_OutU1Timeout = u1_on ? u1_to : 0;
    op->slo_OutU2Timeout = u2_on ? u2_to : 0;

    /* SEL/PEL for the SET_SEL payload (ns -> us, round up). */
    u32 u1_sel = (udev->lpm.u1_sel + 999u) / 1000u;
    u32 u1_pel = (udev->lpm.u1_pel + 999u) / 1000u;
    u32 u2_sel = (udev->lpm.u2_sel + 999u) / 1000u;
    u32 u2_pel = (udev->lpm.u2_pel + 999u) / 1000u;
    op->slo_OutU1Sel = (u16)u1_sel;
    op->slo_OutU1Pel = (u16)u1_pel;
    op->slo_OutU2Sel = (u16)u2_sel;
    op->slo_OutU2Pel = (u16)u2_pel;

    BOOL sel_in_range = (u1_sel <= USB3_LPM_MAX_U1_SEL_PEL &&
                         u1_pel <= USB3_LPM_MAX_U1_SEL_PEL &&
                         u2_sel <= USB3_LPM_MAX_U2_SEL_PEL &&
                         u2_pel <= USB3_LPM_MAX_U2_SEL_PEL);

    if ((u1_on || u2_on) && sel_in_range)
        op->slo_OutFlags |= UHCD_LPO_SET_SEL;
    if (u1_on && xhci_lpm_may_initiate(udev, FALSE))
        op->slo_OutFlags |= UHCD_LPO_U1_INIT;
    if (u2_on && xhci_lpm_may_initiate(udev, TRUE))
        op->slo_OutFlags |= UHCD_LPO_U2_INIT;

    KprintfT("LPM slot %lu: u1_to=%lu u2_to=%lu sel(u1 %lu/%lu u2 %lu/%lu us) flags=%04lx\n",
             (ULONG)udev->slot_id, (ULONG)u1_to, (ULONG)u2_to,
             (ULONG)u1_sel, (ULONG)u1_pel, (ULONG)u2_sel, (ULONG)u2_pel,
             (ULONG)op->slo_OutFlags);

    if (!u1_on && !u2_on)
        return FALSE; /* only LTM, if any — no MEL eval needed */

    /* MEL must be latched by the controller before the stack arms the port
     * timeouts (xHCI 4.23.5.2).  The caller issues the Evaluate Context and
     * replies the op from its completion. */
    udev->lpm.max_exit_latency_us = xhci_calculate_mel(udev);
    udev->lpm.mel_retry_count = 0;
    return TRUE;
}

/* COMP_MEL_ERR recovery (xHCI 4.23.5.2): the xHC rejected MAX_EXIT as too
 * large for the current schedule; eld (bits 23:0 of the completion status)
 * says by how much to reduce.  Shrinks the device MEL in the input context
 * and counts the retry; FALSE = retry budget exhausted, give up. */
BOOL xhci_lpm_handle_mel_err(struct usb_device *udev, u32 eld)
{
    if (udev->lpm.mel_retry_count >= 3)
        return FALSE;

    udev->lpm.max_exit_latency_us = (udev->lpm.max_exit_latency_us > eld)
                                    ? udev->lpm.max_exit_latency_us - eld
                                    : 0;
    udev->lpm.mel_retry_count++;
    KprintfT("COMP_MEL_ERR slot %lu: eld=%lu new_mel=%lu retry=%lu\n",
             (ULONG)udev->slot_id, (ULONG)eld,
             (ULONG)udev->lpm.max_exit_latency_us, (ULONG)udev->lpm.mel_retry_count);
    xhci_update_mel_in_input_ctx(udev);
    return TRUE;
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
    if (!udev || !udev->lpm.setup_done)
        return;

    /* USB2 hardware LPM is only ever enabled for an HS device directly on a
     * root-hub port, so this clear is always a safe local register write. */
    if (udev->speed == USB_SPEED_HIGH && udev->lpm.usb2_hw_lpm_capable &&
        xhci_lpm_parent_hub(udev) == NULL)
        xhci_roothub_clear_usb2_hw_lpm(udev->controller->root_hub, xhci_find_root_port(udev));

    udev->lpm.setup_done = FALSE;
}
