/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The clock-driven RT-ISO engine (NSCMD_USB_REGISTER/UNREGISTER_HOOKS,
 * START/STOP_STREAM — ABI doc §10.3): continuous iso streaming keyed on
 * {handle, endpoint} through a struct USBIsoHooks block.  Microframe/Frame-ID
 * scheduling, inflight accounting, the IN staging slab and the hook dispatch
 * live here; the endpoint lifecycle and state machine stay in
 * xhci-endpoint.c (shared internals in xhci-endpoint-priv.h).
 */

#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#include <clib/utility_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME SysBase /* a local in every function, from its context's sysBase */
#include <proto/exec.h>
#define UTILITY_BASE_NAME ep_ctx->udev->controller->utilityBase
#include <proto/utility.h>
#endif

#include <exec/errors.h>

#include <memory.h>
#include <debug.h>
#include <config.h>
#include <minlist.h>
#include <timing.h>

#include "xhci-endpoint-priv.h"
#include <xhci/xhci-submit.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci-ring.h>
#include <xhci/xhci.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-rtiso] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef TRACE
#undef KprintfT
#define KprintfT(fmt, ...) PrintPistorm("[xhci-rtiso] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

/* Hook object for every iso hook call: the registrant's uih_Object (Neptune
 * passes the classic IOUsbHWRTIso block so class hooks run unchanged), the
 * hooks block itself as fallback. */
#define RT_HOOK_OBJ(hooks) ((hooks)->uih_Object ? (hooks)->uih_Object : (APTR)(hooks))

/* uih_ReleaseHook: exactly once, only when the stream dies WITHOUT a client
 * STOP (endpoint failure, device teardown). */
void xhci_ep_rt_iso_fire_release(struct ep_context *ep_ctx)
{
    struct rt_iso_state *rt = ep_ctx->rt;
    if (!rt || rt->release_fired || !rt->hooks || !rt->hooks->uih_ReleaseHook)
        return;
    rt->release_fired = TRUE;
    CallHookPkt(rt->hooks->uih_ReleaseHook, RT_HOOK_OBJ(rt->hooks), NULL);
}

void xhci_ep_destroy_rt_staging_slab(struct ep_context *ep_ctx)
{
    if (!ep_ctx->rt || !ep_ctx->rt->in_staging_active)
        return;
    slab_cache_destroy(&ep_ctx->rt->in_staging_slab);
    ep_ctx->rt->in_staging_active = FALSE;
}

/*
 * RT ISO functions
 */
void xhci_ep_set_rt_service(struct ep_context *ep_ctx, u8 interval, u32 max_esit_payload)
{
    /* xHCI EP Context Interval is log2 of microframes-per-ESIT. */
    ep_ctx->rt_uframes_per_esit = (u16)(1U << interval);
    ep_ctx->rt_esit_payload = max_esit_payload;
}

#define RT_ISO_SCHED_OFFSET_UFRAMES 32U /* a stream's first TD: this far past the IST */
#define RT_ISO_RUNNING_LEAD_UFRAMES 8U  /* a running stream's TDs: at least this far past it */
#define RT_ISO_UFRAMES_PER_FRAME 8U
#define RT_ISO_IRQ_FRAMES_MAX 8U        /* the longest completion interrupt period */
#define RT_ISO_IRQ_SHARE 4U             /* the period: at most a quarter of the pipeline */
#define RT_ISO_FRAME_MASK 0x7ffU /* Frame ID modulus: 2048 frames */
#define RT_ISO_UF_MASK 0x3fffU   /* microframe modulus: 2048 frames * 8 = 16384 */
#define RT_UFRAME_TO_FRAME(uf) (((uf) >> 3) & RT_ISO_FRAME_MASK)

/* Earliest microframe a TD may be given, @lead past MFINDEX and the IST,
 * rounded UP to the next ESIT boundary. */
static u16 xhci_rt_iso_min_uf(struct ep_context *ep_ctx, u32 lead)
{
    u32 mfindex = mmio_read32(&ep_ctx->udev->controller->run_regs->microframe_index);
    u32 raw = mfindex + ep_ctx->rt->ist + lead;
    u32 ivf_mask = (u32)ep_ctx->rt_uframes_per_esit - 1U;
    return (u16)(((raw + ivf_mask) & ~ivf_mask) & RT_ISO_UF_MASK);
}

/* If rt_next_uframe has fallen behind the HW window, advance it by whole
 * ESITs to the next valid slot >= min. Updates rt_next_uframe in place
 * so the caller's post-submit advance keeps the stream contiguous.
 * Returns the 11-bit Frame ID for the resulting microframe.
 *
 * A running stream is late only when its next TD would fall inside the IST
 * and a frame's margin.  The first TD's larger offset, applied here too,
 * dropped microframes from every pipeline shorter than it: a 4 ms prefetch
 * of 192 kHz audio lost one in twenty-five. */
static u16 xhci_rt_iso_clamp_frame(struct ep_context *ep_ctx)
{
    u16 uf = ep_ctx->rt->next_uframe;
    u16 min_uf = xhci_rt_iso_min_uf(ep_ctx, RT_ISO_RUNNING_LEAD_UFRAMES);
    /* 14-bit signed delta to handle wraparound. */
    u16 raw_delta = (u16)(min_uf - uf) & RT_ISO_UF_MASK;
    s32 delta = (raw_delta >= 0x2000U) ? ((s32)raw_delta - 0x4000) : (s32)raw_delta;

    if (delta > 0)
    {
        u32 ivf_mask = (u32)ep_ctx->rt_uframes_per_esit - 1U;
        u32 advance = ((u32)delta + ivf_mask) & ~ivf_mask;
        uf = (u16)((uf + advance) & RT_ISO_UF_MASK);
        ep_ctx->rt->next_uframe = uf;
        ep_ctx->rt->skipped_uframes += advance;
        ep_ctx->rt->late_refills++;
    }
    return RT_UFRAME_TO_FRAME(uf);
}

inline static void xhci_ep_rt_iso_update_counters(struct ep_context *ep_ctx, u32 length)
{
    /* Maintain RT ISO inflight counters. */
    if (ep_ctx->rt->inflight_bytes >= length)
        ep_ctx->rt->inflight_bytes -= length;
}

inline static void xhci_ep_rt_iso_zero_counters(struct ep_context *ep_ctx)
{
    ep_ctx->rt->inflight_bytes = 0;
}

/* Free a staging buffer that was allocated from this endpoint's iso_in_staging_slab.
 * Falls back to pool_free if the slab is no longer active (edge-case teardown guard). */
void xhci_ep_free_rt_iso_buffer(struct ep_context *ep_ctx, APTR data_buffer)
{
    if (!data_buffer)
        return;
    if (ep_ctx && ep_ctx->rt && ep_ctx->rt->in_staging_active)
        slab_free(&ep_ctx->rt->in_staging_slab, data_buffer);
}

static void xhci_ep_set_rt_stopped(struct ep_context *ep_ctx)
{
    xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RT_ISO_STOPPED);
    if (ep_ctx->rt)
    {
        ep_ctx->rt->stop_pending = NULL;
        ep_ctx->rt->last_buffer = NULL;
        ep_ctx->rt->last_filled = 0;
        xhci_ep_rt_iso_zero_counters(ep_ctx);
    }
    xhci_ep_destroy_rt_staging_slab(ep_ctx);
}

s8 xhci_ep_rt_iso_add_handler(struct ep_context *ep_ctx, struct USBIsoHooks *hooks, u8 direction)
{
    if (!hooks || !ep_ctx)
        return UHIOERR_BADPARAMS;
    struct ExecBase *SysBase = ep_ctx->sysBase;

    if (ep_ctx->state != USB_DEV_EP_STATE_IDLE)
    {
        /* can only enable RT ISO if EP is idle */
        Kprintf("EP not idle. Current state: %lu\n", (ULONG)ep_ctx->state);
        return UHIOERR_HOSTERROR;
    }

    if (!ep_ctx->rt)
    {
        ep_ctx->rt = pool_zalloc(ep_ctx->udev->controller->metaPool, sizeof(struct rt_iso_state));
        if (!ep_ctx->rt)
        {
            Kprintf("Failed to allocate RT ISO state\n");
            return UHIOERR_OUTOFMEMORY;
        }
    }

    xhci_ep_set_rt_stopped(ep_ctx);

    ep_ctx->rt->hooks = hooks;
    ep_ctx->rt->direction = direction;
    ep_ctx->rt->release_fired = FALSE;

#ifdef TRACE
    struct USBIsoHooks *rt = hooks;
    if (direction == XHCI_DIR_IN)
    {
        KprintfT("Added ISO handler: EP %lu in req hook: %lx, in done hook: %lx, prefetch: %lu\n", (ULONG)ep_ctx->ep_index, (ULONG)rt->uih_InRequestHook, (ULONG)rt->uih_InDoneHook, (ULONG)rt->uih_MaxPrefetch);
    }
    else
    {
        KprintfT("Added ISO handler: EP %lu out req hook: %lx, out done hook: %lx, prefetch: %lu\n", (ULONG)ep_ctx->ep_index, (ULONG)rt->uih_OutRequestHook, (ULONG)rt->uih_OutDoneHook, (ULONG)rt->uih_MaxPrefetch);
    }
#endif
    return UHIOERR_NO_ERROR;
}

s8 xhci_ep_rt_iso_rem_handler(struct ep_context *ep_ctx, struct USBIsoHooks *hooks)
{
    if (!hooks || !ep_ctx)
    {
        Kprintf("Invalid parameters to remove RT ISO handler\n");
        return UHIOERR_BADPARAMS;
    }
    struct ExecBase *SysBase = ep_ctx->sysBase;

    if (ep_ctx->state != USB_DEV_EP_STATE_RT_ISO_STOPPED)
    {
        Kprintf("EP not in RT_ISO_STOPPED/IDLE state. Current state: %lu\n", (ULONG)ep_ctx->state);
        return UHIOERR_HOSTERROR;
    }

    if (!ep_ctx->rt || hooks != ep_ctx->rt->hooks)
    {
        Kprintf("Mismatched RT ISO handler removal request\n");
        return UHIOERR_BADPARAMS;
    }

    xhci_ep_destroy_rt_staging_slab(ep_ctx);
    pool_free(ep_ctx->udev->controller->metaPool, ep_ctx->rt);
    ep_ctx->rt = NULL;
    xhci_ep_set_idle(ep_ctx);
    KprintfT("Successfully removed ISO handler (state reset to IDLE)\n");
    return UHIOERR_NO_ERROR;
}

void xhci_ep_rt_iso_in(struct ep_context *ep_ctx, APTR buffer, u32 length, u32 act_len, u16 rt_frame, u16 ubr_flags)
{
    struct USBIsoHooks *hooks = ep_ctx->rt->hooks;

    /* An interval that brought nothing is routine on a fast endpoint and no
     * business of the class - unless it failed and the class asked to hear of
     * those (UHCD_IHF_IN_ERRORS). */
    if (act_len > 0 || (ubr_flags && (hooks->uih_Flags & UHCD_IHF_IN_ERRORS)))
    {
        /* Pull a destination buffer from the class, copy staged DMA into it, then signal completion. */
        struct USBBufferRequest rt_buffer_req;
        rt_buffer_req.frame = rt_frame;
        rt_buffer_req.flags = 0;
        rt_buffer_req.length = act_len;
        rt_buffer_req.data = NULL;

        CallHookPkt(hooks->uih_InRequestHook, RT_HOOK_OBJ(hooks), &rt_buffer_req);

        if (rt_buffer_req.data)
        {
            u32 copy_len = act_len < rt_buffer_req.length ? act_len : rt_buffer_req.length;
            memcpy(rt_buffer_req.data, buffer, copy_len);
            rt_buffer_req.length = copy_len;
            rt_buffer_req.flags = ubr_flags; /* done direction carries the wire status */

            CallHookPkt(hooks->uih_InDoneHook, RT_HOOK_OBJ(hooks), &rt_buffer_req);
        }
    }

    xhci_ep_rt_iso_update_counters(ep_ctx, length);
}

void xhci_ep_rt_iso_out(struct ep_context *ep_ctx, APTR buffer, u32 length, u32 act_len, u16 rt_frame, u16 ubr_flags)
{
    struct USBIsoHooks *hooks = ep_ctx->rt->hooks;
    (void)buffer;

    if (hooks->uih_OutDoneHook)
    {
        /* OUT path: completion hook only. OutReqHook is before transfer. */
        struct USBBufferRequest rt_buffer_req;
        rt_buffer_req.data = buffer;
        rt_buffer_req.frame = rt_frame;
        rt_buffer_req.length = act_len;
        rt_buffer_req.flags = ubr_flags; /* done direction carries the wire status */
        CallHookPkt(hooks->uih_OutDoneHook, RT_HOOK_OBJ(hooks), &rt_buffer_req);
    }

    xhci_ep_rt_iso_update_counters(ep_ctx, length);
}

/* Shared per-TD scheduling steps of the IN and OUT schedulers — the CFC
 * frame policy and the room/grow backpressure live in exactly one place. */
static BOOL rt_ensure_room(struct ep_context *ep_ctx)
{
    if (xhci_submit_has_room(ep_ctx, 2))
        return TRUE;
    return xhci_ring_grow(ep_ctx->udev->controller, xhci_ep_get_ring(ep_ctx), XHCI_SEGMENTS_PER_RING) &&
           xhci_submit_has_room(ep_ctx, 2);
}

static u16 rt_pick_frame(struct ep_context *ep_ctx)
{
    /* CFC controllers pin Frame ID to a slot; without CFC the HW chooses via
     * SIA, but the hooks still see a monotonic frame counter. */
    return ep_ctx->udev->controller->cfc_supported
               ? xhci_rt_iso_clamp_frame(ep_ctx)
               : RT_UFRAME_TO_FRAME(ep_ctx->rt->next_uframe);
}

/*
 * Completion interrupts.  Only some TDs raise one (the others carry BEI): an
 * endpoint served every microframe would otherwise interrupt 8000 times a
 * second.  The period is the largest power of two up to a quarter of the
 * pipeline, so three quarters of it remain while an interrupt waits to be
 * served and the class's hooks run; none below a frame (every TD interrupts
 * then), at most RT_ISO_IRQ_FRAMES_MAX frames.  Periods are aligned to frame
 * numbers: the TD ending a period interrupts on OUT, the TD starting one on
 * IN, whose packet arrives within its interval - so a stream's OUT and its
 * feedback IN interrupt together at the same edge.  A stream's end needs no
 * interrupt of its own: the controller reports the ring empty after the last
 * TD, and that event interrupts.
 */
static u32 rt_irq_period(const struct ep_context *ep_ctx, u32 depth_uframes)
{
    const u32 most = RT_ISO_IRQ_FRAMES_MAX * RT_ISO_UFRAMES_PER_FRAME;
    const u32 quarter = depth_uframes / RT_ISO_IRQ_SHARE;
    u32 period = RT_ISO_UFRAMES_PER_FRAME;

    if (ep_ctx->udev->controller->hci_version < 0x100 || quarter < period)
        return 0; /* BEI is xHCI 1.0's */
    while (period * 2U <= quarter && period < most)
        period *= 2U;
    return period;
}

static BOOL rt_silent(u32 uframe, u32 period)
{
    return period && (uframe & (period - 1U)) != 0;
}

/* An OUT TD's period comes from the pipeline its prefetch holds at this TD's
 * length. */
static BOOL rt_out_silent(const struct ep_context *ep_ctx, u32 prefetch_bytes, u32 length)
{
    const u32 depth_tds = prefetch_bytes / (length ? length : 1U);
    const u32 period = rt_irq_period(ep_ctx, depth_tds * ep_ctx->rt_uframes_per_esit);
    return rt_silent((u32)ep_ctx->rt->next_uframe + ep_ctx->rt_uframes_per_esit, period);
}

static void rt_advance(struct ep_context *ep_ctx, u32 bytes)
{
    ep_ctx->rt->next_uframe = (u16)(((u32)ep_ctx->rt->next_uframe + ep_ctx->rt_uframes_per_esit) & RT_ISO_UF_MASK);
    ep_ctx->rt->inflight_bytes += bytes;
}

static void xhci_ep_schedule_rt_iso_out(struct ep_context *ep_ctx)
{
    struct USBIsoHooks *hooks = ep_ctx->rt->hooks;
    u32 prefetch_bytes = hooks->uih_MaxPrefetch ? hooks->uih_MaxPrefetch : 2048;

    while (ep_ctx->rt->inflight_bytes < prefetch_bytes)
    {
        /* Don't enqueue if the ring can't take an RT TD (two TRBs at most: a
         * buffer crossing 64 KB); try to grow first */
        if (!rt_ensure_room(ep_ctx))
            break;

        struct USBBufferRequest rt_buffer_req;
        rt_buffer_req.length = prefetch_bytes;
        rt_buffer_req.data = ep_ctx->rt->last_buffer;
        rt_buffer_req.flags = 0;
        //TODO replace with timer perhaps, PCIe read is expensive on VL805
        rt_buffer_req.frame = rt_pick_frame(ep_ctx); /* monotonic frame counter to avoid jumps */

        KprintfT("RT ISO OUT sched frame=%lu len=%lu inflight_bytes=%lu inflight_tds=%lu\n",
                 (ULONG)rt_buffer_req.frame,
                 (ULONG)rt_buffer_req.length,
                 (ULONG)ep_ctx->rt->inflight_bytes,
                 (ULONG)xhci_ep_get_active_td_count(ep_ctx));

        CallHookPkt(hooks->uih_OutRequestHook, RT_HOOK_OBJ(hooks), &rt_buffer_req);

        if (!rt_buffer_req.data || rt_buffer_req.length == 0)
        {
            KprintfT("RT ISO hook provided no buffer/length\n");
            break;
        }

        u32 offset = 0;
        if (rt_buffer_req.data == ep_ctx->rt->last_buffer)
        {
            offset = ep_ctx->rt->last_filled;
        }
        else
        {
            ep_ctx->rt->last_buffer = rt_buffer_req.data;
        }
        ep_ctx->rt->last_filled = offset + rt_buffer_req.length;

        /* The hook may have run long (a class mixes its next pass from it): the
         * frame is picked again, and xhci_submit_rt_td() hands an OUT TD to the
         * controller at once, so no TD reaches it later than its frame allows. */
        u16 frame = rt_pick_frame(ep_ctx);
        u32 length = rt_buffer_req.length;
        s8 ret = xhci_submit_rt_td(ep_ctx->udev, ep_ctx,
                                   (APTR)((u8 *)rt_buffer_req.data + offset), length,
                                   frame, XHCI_DIR_OUT, rt_out_silent(ep_ctx, prefetch_bytes, length));
        if (ret != UHIOERR_NO_ERROR)
        {
            Kprintf("RT ISO submit failed %ld\n", (LONG)ret);
            break;
        }

        rt_advance(ep_ctx, length);
        KprintfT("RT ISO OUT queued frame=%lu len=%lu inflight_bytes=%lu inflight_tds=%lu\n",
                 (ULONG)frame,
                 (ULONG)length,
                 (ULONG)ep_ctx->rt->inflight_bytes,
                 (ULONG)xhci_ep_get_active_td_count(ep_ctx));
    }

    xhci_submit_giveback(ep_ctx->udev, ep_ctx);
}

/*
 * The pipeline, RT_ISO_IN_TARGET_FRAMES deep, is topped up an interrupt
 * period at a time (see rt_irq_period), so the hooks run at most a period
 * late.
 */
static void xhci_ep_schedule_rt_iso_in(struct ep_context *ep_ctx)
{
    const u32 period = ep_ctx->rt->in_irq_period;
    const u32 uframes_per_esit = ep_ctx->rt_uframes_per_esit;
    const u32 batch = (period > uframes_per_esit) ? period / uframes_per_esit : 1U;
    const u32 inflight = xhci_ep_get_active_td_count(ep_ctx);
    if (inflight + batch > ep_ctx->rt->inflight_tds_target)
        return;

    for (u32 remaining = ep_ctx->rt->inflight_tds_target - inflight; remaining; --remaining)
    {
        /* Same backpressure rule as the OUT path: bail before alloc if no room. */
        if (!rt_ensure_room(ep_ctx))
            break;

        u16 frame = rt_pick_frame(ep_ctx);
        /* One TD per service interval, sized for everything the interval can
         * carry: a high-bandwidth or SuperSpeed endpoint sends several
         * packets in it. */
        const u32 td_length = ep_ctx->rt_esit_payload;

        APTR staging = slab_alloc(&ep_ctx->rt->in_staging_slab);
        if (!staging)
        {
            Kprintf("Failed to alloc RT ISO staging buffer\n");
            break;
        }

        KprintfT("RT ISO IN sched frame=%lu len=%lu inflight_bytes=%lu inflight_tds=%lu\n",
                 (ULONG)frame,
                 (ULONG)td_length,
                 (ULONG)ep_ctx->rt->inflight_bytes,
                 (ULONG)xhci_ep_get_active_td_count(ep_ctx));

        s8 ret = xhci_submit_rt_td(ep_ctx->udev, ep_ctx,
                                   staging, td_length, frame, XHCI_DIR_IN,
                                   rt_silent(ep_ctx->rt->next_uframe, period));
        if (ret != UHIOERR_NO_ERROR)
        {
            slab_free(&ep_ctx->rt->in_staging_slab, staging);
            Kprintf("RT ISO submit failed %ld\n", (LONG)ret);
            break;
        }

        rt_advance(ep_ctx, td_length);
        KprintfT("RT ISO IN queued frame=%lu len=%lu inflight_bytes=%lu inflight_tds=%lu\n",
                 (ULONG)frame,
                 (ULONG)td_length,
                 (ULONG)ep_ctx->rt->inflight_bytes,
                 (ULONG)xhci_ep_get_active_td_count(ep_ctx));
    }

    xhci_submit_giveback(ep_ctx->udev, ep_ctx);
}

static void xhci_ep_notify_rt_iso_stopped(struct ep_context *ep_ctx)
{
    if (!ep_ctx || !ep_ctx->rt)
        return;

    struct xhci_xfer *stop_req = ep_ctx->rt->stop_pending;

    if (!stop_req)
        return;

    ep_ctx->rt->stop_pending = NULL;

    stop_req->error = UHIOERR_NO_ERROR;
    xhci_xfer_reply(stop_req); /* the STOP_STREAM shadow copies back to its client */
}

/* The controller reports the ring empty while the stream is being stopped.
 * A stop waits for the TDs in flight to complete - but TDs the controller
 * passed over only leave the list when a later TD's event names them, and
 * after the last one there is no later TD.  Whatever is still tracked now
 * will never complete: drop it, which lets the stop finish. */
void xhci_ep_rt_iso_ring_empty(struct ep_context *ep_ctx)
{
    xhci_td_fail_all(ep_default_tds(ep_ctx), IOERR_ABORTED);
    xhci_ep_schedule_rt_iso(ep_ctx); /* not running and nothing in flight: stopped */
}

void xhci_ep_schedule_rt_iso(struct ep_context *ep_ctx)
{
    if (ep_ctx->state != USB_DEV_EP_STATE_RT_ISO_RUNNING)
    {
        if (xhci_td_is_empty(ep_default_tds(ep_ctx)))
        {
            xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RT_ISO_STOPPED);
            xhci_ep_notify_rt_iso_stopped(ep_ctx);
        }
        return;
    }

    /* handle deferred queue - pending */
    xhci_ep_schedule_next(ep_ctx);

    if (!ep_ctx->rt)
    {
        Kprintf("No RT ISO state\n");
        xhci_ep_set_failed(ep_ctx);
        return;
    }

    if (ep_ctx->rt->direction == XHCI_DIR_IN)
        xhci_ep_schedule_rt_iso_in(ep_ctx);
    else
        xhci_ep_schedule_rt_iso_out(ep_ctx);
}

s8 xhci_ep_rt_iso_start(struct ep_context *ep_ctx)
{
    if (ep_ctx->state != USB_DEV_EP_STATE_RT_ISO_STOPPED)
    {
        Kprintf("EP not in RT_ISO_STOPPED\n");
        return UHIOERR_HOSTERROR;
    }

    /* HCSPARAMS2.IST is 4 bits (xHCI 5.3.6): bit 3 selects the unit of bits[2:0]
     * — 0 = microframes (0..7), 1 = frames (0..7, i.e. 0..56 microframes).
     * Normalize to microframes once so users can add to MFINDEX directly. */
    u32 ist_raw = HCS_IST(mmio_read32(&ep_ctx->udev->controller->hccr->cr_hcsparams2));
    ep_ctx->rt->ist = (ist_raw & 0x8U) ? ((ist_raw & 0x7U) << 3) : (ist_raw & 0x7U);
    ep_ctx->rt->next_uframe = xhci_rt_iso_min_uf(ep_ctx, RT_ISO_SCHED_OFFSET_UFRAMES);
    xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RT_ISO_RUNNING);

    const u32 uframes_per_esit = ep_ctx->rt_uframes_per_esit;
    const u32 target_uframes = RT_ISO_IN_TARGET_FRAMES * RT_ISO_UFRAMES_PER_FRAME;
    ep_ctx->rt->inflight_tds_target = (target_uframes + uframes_per_esit - 1U) / uframes_per_esit;

    ep_ctx->rt->in_irq_period = rt_irq_period(ep_ctx, ep_ctx->rt->inflight_tds_target * uframes_per_esit);
    ep_ctx->rt->skipped_uframes = 0;
    ep_ctx->rt->late_refills = 0;
    Kprintf("RT ISO start slot %lu ep %lu: IST %lu uframes\n",
            (ULONG)ep_ctx->udev->slot_id, (ULONG)ep_ctx->ep_index, (ULONG)ep_ctx->rt->ist);

    /* Build the per-endpoint IN staging slab now that we know both the TD length
     * and the target inflight depth.  OUT endpoints have no staging buffer.
     *
     * Each buffer is aligned to its own size rounded up to a power of two, so
     * none crosses a 64 KB boundary and every IN TD is exactly one TRB (the
     * payload of one interval never exceeds 48 KB).  That is what makes the
     * completion's length exact without ISP: a short packet can only end in
     * the TD's final TRB, whose event residue is the TD's. */
    if (ep_ctx->rt->direction == XHCI_DIR_IN && ep_ctx->rt_esit_payload > 0)
    {
        slab_cache_init(&ep_ctx->rt->in_staging_slab, ep_ctx->sysBase,
                        ep_ctx->udev->controller->metaPool,
                        ep_ctx->udev->controller->dmaPool,
                        ep_ctx->rt_esit_payload,
                        round_up_pow2_u32(ep_ctx->rt_esit_payload),
                        ep_ctx->rt->inflight_tds_target);
        ep_ctx->rt->in_staging_active = TRUE;
    }

    KprintfT("Starting RT ISO stream: IST=%lu uframes rt_next_uframe=%lu target_ms=%lu target_uframes=%lu uframes_per_esit=%lu target_tds=%lu\n",
             (ULONG)ep_ctx->rt->ist,
             (ULONG)ep_ctx->rt->next_uframe,
             (ULONG)RT_ISO_IN_TARGET_FRAMES,
             (ULONG)target_uframes,
             (ULONG)uframes_per_esit,
             (ULONG)ep_ctx->rt->inflight_tds_target);

    xhci_ep_schedule_rt_iso(ep_ctx);
    return UHIOERR_NO_ERROR;
}

s8 xhci_ep_rt_iso_stop(struct ep_context *ep_ctx, struct USBIsoHooks *hooks, struct xhci_xfer *stop_token)
{
    if (ep_ctx->state != USB_DEV_EP_STATE_RT_ISO_RUNNING)
    {
        Kprintf("EP not in RT_ISO_RUNNING\n");
        return UHIOERR_HOSTERROR;
    }

    if (ep_ctx->rt->hooks != hooks)
    {
        Kprintf("bad params\n");
        return UHIOERR_BADPARAMS;
    }

    if (ep_ctx->rt->stop_pending)
    {
        Kprintf("STOPRTISO already pending\n");
        return UHIOERR_HOSTERROR;
    }

    ep_ctx->rt->stop_pending = stop_token;
    Kprintf("RT ISO stop slot %lu ep %lu: %lu microframes skipped in %lu late refills\n",
            (ULONG)ep_ctx->udev->slot_id, (ULONG)ep_ctx->ep_index,
            (ULONG)ep_ctx->rt->skipped_uframes, (ULONG)ep_ctx->rt->late_refills);

    if (xhci_td_is_empty(ep_default_tds(ep_ctx)))
    {
        xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RT_ISO_STOPPED);
        xhci_ep_notify_rt_iso_stopped(ep_ctx);
    }
    else
    {
        KprintfT("RT ISO stopping slot=%lu ep=%lu inflight_tds=%lu inflight_bytes=%lu\n",
                 (ULONG)ep_ctx->udev->slot_id,
                 (ULONG)ep_ctx->ep_index,
                 (ULONG)xhci_ep_get_active_td_count(ep_ctx),
                 (ULONG)ep_ctx->rt->inflight_bytes);
        xhci_ep_transition(ep_ctx, USB_DEV_EP_STATE_RT_ISO_STOPPING);
    }
    return UHIOERR_NO_ERROR;
}