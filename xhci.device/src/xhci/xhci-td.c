/* SPDX-License-Identifier: GPL-2.0-only */

#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME SysBase /* a local in every function, from its context's sysBase */
#include <proto/exec.h>
#endif

#include <exec/errors.h>

#include <memory.h>
#include <timing.h>
#include <xhci/xhci-context.h> /* EP_CTX_CYCLE_MASK */
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-ring.h>
#include <xhci/xhci-submit.h>
#include <xhci/xhci-udev.h>
#include "xhci-td-priv.h"
#include <xhci/xhci.h>
#include <minlist.h>
#include <debug.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-td] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef TRACE
#undef KprintfT
#define KprintfT(fmt, ...) PrintPistorm("[xhci-td] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

struct xhci_td
{
    struct MinNode node; /* linkage in active TD list */
    BOOL is_rt_iso;      /* discriminates the payload union */
    union
    {
        struct xhci_xfer *req; /* owning request (!is_rt_iso) */
        struct
        {
            struct xhci_dma_span span; /* mapped buffer, owned by the TD */
            u16 frame;
            u16 dir;      /* XHCI_DIR_IN / XHCI_DIR_OUT */
            BOOL staging; /* IN buffer from the endpoint's staging slab */
        } rt;
    } u;
    dma_addr_t completion_trb; /* TRB address we expect a completion for */
    u32 length;                /* total length for completion accounting */
    u32 trb_count;             /* number of TRBs consumed by this TD */
    u32 trb_cap;               /* entries trb_addrs was allocated for (its size class) */
    dma_addr_t *trb_addrs;     /* DMA addresses for every TRB in this TD */

    /* Mid-TD short packet: exact transferred length recorded at the short
     * event; the TD completes on its final-TRB event with this value. */
    BOOL short_seen;
    u32 short_act_len;
};

/* A TD's NAK deadline is its request's; RT ISO TDs have none. */
static inline BOOL td_is_expired_at(const struct xhci_td *td, u32 now)
{
    return !td->is_rt_iso && xhci_xfer_expired(td->u.req, now);
}

struct TransferDescriptorList
{
    struct MinList list;
    struct xhci_ctrl *ctrl;
    struct ExecBase *sysBase; /* ctrl->sysBase, copied at create */
    u32 queued_tds;
    struct xhci_ring *ring;    /* the ring these TDs ride (one list per ring) */
    struct ep_context *ep_ctx; /* back-reference for per-endpoint resource cleanup */
};

void xhci_td_slab_init(struct xhci_ctrl *ctrl)
{
    slab_cache_init(&ctrl->td_slab, ctrl->sysBase, ctrl->metaPool, NULL, sizeof(struct xhci_td), DMA_ALIGN_MIN, 2048);
}

void xhci_td_slab_destroy(struct xhci_ctrl *ctrl)
{
    slab_cache_destroy(&ctrl->td_slab);
}

TransferDescriptorList *xhci_td_create_list(struct xhci_ctrl *ctrl, struct ep_context *ep_ctx,
                                            struct xhci_ring *ring)
{
    struct ExecBase *SysBase = ctrl->sysBase;
    TransferDescriptorList *td_list = pool_zalloc(ctrl->metaPool, sizeof(TransferDescriptorList));
    if (!td_list)
    {
        Kprintf("Failed to alloc TransferDescriptorList\n");
        return NULL;
    }

    _NewMinList(&td_list->list);
    td_list->ctrl = ctrl;
    td_list->sysBase = ctrl->sysBase;
    td_list->queued_tds = 0;
    td_list->ring = ring;
    td_list->ep_ctx = ep_ctx;

    xhci_ring_set_td_list(ring, td_list);
    return td_list;
}

void xhci_td_destroy_list(TransferDescriptorList *td_list, s8 error_code)
{
    if (!td_list)
        return;
    struct ExecBase *SysBase = td_list->sysBase;

    xhci_td_fail_all(td_list, error_code);

    xhci_ring_set_td_list(td_list->ring, NULL);
    pool_free(td_list->ctrl->metaPool, td_list);
}

BOOL xhci_td_is_empty(TransferDescriptorList *td_list)
{
    if (!td_list)
        return TRUE;
    return td_list->list.mlh_Head == (struct MinNode *)&td_list->list.mlh_Tail;
}

/**
 * xhci_td_any_expired() - Whether any TD on a list is past its NAK deadline.
 * @td_list: the ring's TD list.
 * @now: get_time() of the caller's tick.
 *
 * Return: TRUE at the first expired TD found.
 */
BOOL xhci_td_any_expired(TransferDescriptorList *td_list, u32 now)
{
    if (!td_list)
        return FALSE;

    for (struct MinNode *n = td_list->list.mlh_Head; n->mln_Succ; n = n->mln_Succ)
    {
        const struct xhci_td *td = (const struct xhci_td *)n;
        if (td_is_expired_at(td, now))
        {
            KprintfT("Found expired TD req=%lx deadline=%lu now=%lu\n",
                     td->u.req, (ULONG)td->u.req->deadline_us, (ULONG)now);
            return TRUE;
        }
    }
    return FALSE;
}

struct xhci_xfer *xhci_td_find_cookie_request(TransferDescriptorList *td_list, APTR cookie)
{
    if (!td_list)
        return NULL;

    for (struct MinNode *n = td_list->list.mlh_Head; n && n->mln_Succ; n = n->mln_Succ)
    {
        struct xhci_td *td = (struct xhci_td *)n;
        if (td->is_rt_iso || !td->u.req)
            continue;
        if ((td->u.req->priv_flags & REQ_DIRECT) && td->u.req->cookie == cookie)
            return td->u.req;
    }
    return NULL;
}

u32 xhci_td_get_queued_td_count(TransferDescriptorList *td_list)
{
    if (!td_list)
        return 0;

    return td_list->queued_tds;
}

static struct xhci_td *td_alloc_common(TransferDescriptorList *td_list,
                                       dma_addr_t *trb_addresses, u32 trb_count, u32 trb_cap)
{
    struct xhci_td *td = slab_zalloc(&td_list->ctrl->td_slab);
    if (!td)
    {
        Kprintf("Failed to alloc xhci_td\n");
        return NULL;
    }

    td->trb_count = trb_count;
    td->trb_cap = trb_cap;
    td->trb_addrs = trb_addresses;
    td->completion_trb = trb_addresses[trb_count - 1];
    return td;
}

static void td_append(TransferDescriptorList *td_list, struct xhci_td *td)
{
    struct ExecBase *SysBase = td_list->sysBase;
    td_list->queued_tds++;
    AddTailMinList(&td_list->list, (struct MinNode *)td);
}

BOOL xhci_td_add(TransferDescriptorList *td_list,
                 struct xhci_xfer *io_req,
                 dma_addr_t *trb_addresses,
                 u32 trb_count,
                 u32 trb_cap)
{
    if (!td_list)
        return FALSE;

    struct xhci_td *td = td_alloc_common(td_list, trb_addresses, trb_count, trb_cap);
    if (!td)
        return FALSE;

    td->u.req = io_req;
    io_req->priv_flags |= REQ_ON_RING;
    td->length = io_req->data_length;

    td_append(td_list, td);
    return TRUE;
}

BOOL xhci_td_add_rt(TransferDescriptorList *td_list,
                    const struct xhci_dma_span *span,
                    u16 frame, u16 dir, BOOL staging,
                    dma_addr_t *trb_addresses,
                    u32 trb_count)
{
    if (!td_list)
        return FALSE;

    /* never link-aligned: an RT TD writes the TRBs it reserved, so its count
     * is its cap */
    struct xhci_td *td = td_alloc_common(td_list, trb_addresses, trb_count, trb_count);
    if (!td)
        return FALSE;

    td->is_rt_iso = TRUE;
    td->u.rt.span = *span;
    td->u.rt.frame = frame;
    td->u.rt.dir = dir;
    td->u.rt.staging = staging;
    td->length = span->length;

    td_append(td_list, td);
    return TRUE;
}

/* Sum of data bytes carried by this TD's TRBs before index idx (a control
 * TD's setup TRB is not data).  TRB lengths are read back from the ring; the
 * controller never modifies them. */
static u32 td_sum_trb_lengths(const struct xhci_td *td, u32 idx)
{
    u32 first = (!td->is_rt_iso && td->u.req &&
                 td->u.req->type == UHCD_EPTYPE_CONTROL)
                    ? 1u
                    : 0u;
    u32 sum = 0;

    for (u32 i = first; i < idx; ++i)
    {
        const struct xhci_generic_trb *trb = (const struct xhci_generic_trb *)(uintptr_t)td->trb_addrs[i];
        sum += TRB_LEN(le32(trb->field[2]));
    }
    return sum;
}

/**
 * td_bytes_through() - Data bytes a TD moved up to and including one of its TRBs.
 * @td: the TD.
 * @idx: index of the TRB an event names.
 * @residue: the event's length field: bytes that TRB did not move.
 *
 * The TRBs before @idx moved all their bytes; @idx moved its length less
 * @residue.  A control TD's setup TRB moves no data (td_sum_trb_lengths()).
 *
 * Return: the byte count.
 */
static u32 td_bytes_through(const struct xhci_td *td, u32 idx, u32 residue)
{
    const u32 sum = td_sum_trb_lengths(td, idx + 1);
    return (sum > residue) ? sum - residue : 0;
}

static s32 td_find_trb_index(struct xhci_td *td, dma_addr_t trb_addr)
{
    for (u32 index = 0; index < td->trb_count; ++index)
    {
        if (td->trb_addrs[index] == trb_addr)
            return (s32)index;
    }

    return -1;
}

/* Locate the TD containing trb_addr and report the TRB's index within it.
 * Fast pass first: the final-TRB event (completion_trb) is the overwhelming
 * case, so the interior-TRB scan (mid-TD shorts, recovery stops) only runs
 * when nothing's final TRB matched. */
static struct xhci_td *find_td_by_trb(TransferDescriptorList *td_list, dma_addr_t trb_addr, u32 *idx_out)
{
    if (!td_list)
        return NULL;

    for (struct MinNode *n = td_list->list.mlh_Head; n && n->mln_Succ; n = n->mln_Succ)
    {
        struct xhci_td *td = (struct xhci_td *)n;
        if (td->completion_trb == trb_addr)
        {
            *idx_out = td->trb_count - 1;
            return td;
        }
    }

    for (struct MinNode *n = td_list->list.mlh_Head; n && n->mln_Succ; n = n->mln_Succ)
    {
        struct xhci_td *td = (struct xhci_td *)n;
        const s32 idx = td_find_trb_index(td, trb_addr);
        if (idx >= 0)
        {
            *idx_out = (u32)idx;
            return td;
        }
    }

    return NULL;
}

static void xhci_td_decrease_queued(TransferDescriptorList *td_list)
{
    if (td_list->queued_tds > 0)
        td_list->queued_tds--;
}

static void xhci_td_free(TransferDescriptorList *td_list, struct xhci_td *td)
{
    if (td->trb_addrs)
    {
        xhci_td_trb_addrs_free(td_list->ctrl, td->trb_addrs, td->trb_cap);
        td->trb_addrs = NULL;
    }

    slab_free(&td_list->ctrl->td_slab, td);
}

BOOL xhci_td_has_request(TransferDescriptorList *td_list, struct xhci_xfer *io_req)
{
    if (!td_list || !io_req)
        return FALSE;

    struct MinNode *n = td_list->list.mlh_Head;
    while (n && n->mln_Succ)
    {
        struct xhci_td *td = (struct xhci_td *)n;
        if (!td->is_rt_iso && td->u.req == io_req)
            return TRUE;
        n = n->mln_Succ;
    }

    return FALSE;
}


static void td_unmap_and_reply(TransferDescriptorList *td_list, struct xhci_td *td, BYTE error_code, u32 actual)
{
    /* RT ISO TDs have no request to reply: release the mapped span and the
     * IN staging buffer, drop the data. */
    if (td->is_rt_iso)
    {
        xhci_dma_span_unmap(td_list->ctrl, &td->u.rt.span, 0);
        if (td->u.rt.staging)
            xhci_ep_free_rt_iso_buffer(td_list->ep_ctx, td->u.rt.span.buffer);
        return;
    }

    struct xhci_xfer *req = td->u.req;
    if (!req)
        return;

    xhci_dma_span_unmap(td_list->ctrl, &req->data_span, actual);
    xhci_xfer_complete(NULL, req, error_code, actual);
}

/* Was an abort asked for td's request (abort_reqs)?  RT ISO TDs are stopped
 * through the hook API, never aborted. */
static BOOL td_is_aborted(const struct xhci_td *td, IOReqList *abort_reqs)
{
    if (td->is_rt_iso)
        return FALSE;

    for (struct MinNode *node = abort_reqs->mlh_Head; node->mln_Succ; node = node->mln_Succ)
    {
        if (((IOReqNode *)node)->req == td->u.req)
            return TRUE;
    }
    return FALSE;
}

/* Take a TD off its list and hand its payload to the completion path. */
static void td_consume(TransferDescriptorList *td_list, struct xhci_td *td, u32 act_len,
                       struct xhci_td_completion *out)
{
    struct ExecBase *SysBase = td_list->sysBase;

    out->rt = td->is_rt_iso;
    out->act_len = act_len;

    if (td->is_rt_iso)
    {
        out->req = NULL;
        out->rt_buffer = td->u.rt.span.buffer;
        out->rt_length = td->u.rt.span.length;
        out->rt_frame = td->u.rt.frame;
        out->rt_dir = td->u.rt.dir;
        /* The completion path consumes the data (and frees IN staging). */
        xhci_dma_span_unmap(td_list->ctrl, &td->u.rt.span, act_len);
    }
    else
    {
        struct xhci_xfer *req = td->u.req;
        out->req = req;
        out->rt_buffer = NULL;
        out->rt_length = 0;
        out->rt_frame = 0;
        out->rt_dir = 0;

        if (req)
            xhci_dma_span_unmap(td_list->ctrl, &req->data_span, act_len);
    }

    RemoveMinNode((struct MinNode *)td);
    xhci_td_decrease_queued(td_list);
    xhci_td_free(td_list, td);
}

/*
 * Complete - or defer - the TD containing trb_addr for a transfer event.
 *
 * A short packet on a non-final TRB records the exact transferred length
 * (bytes in the preceding data TRBs plus the short TRB's consumed bytes) and
 * keeps the TD: the controller always follows up with an event for the TD's
 * final TRB (xHCI 4.10.1.1), which consumes the TD with the recorded length.
 * *deferred tells this apart from "no TD found" (both return FALSE).
 *
 * On consumption out->act_len is the exact transferred byte count; the per-TRB
 * event residue is only trusted when no mid-TD short was seen.
 *
 * An iso ring retires its TDs in queue order, one event each.  An event that
 * names a TD behind the oldest one therefore says the controller passed over
 * the TDs before it without serving them, and without saying so: xHCI
 * 4.10.3.2 lets a controller without the Contiguous Frame ID capability skip
 * the Missed Service Error event of a missed interval, and any controller
 * drop those events while its event ring is full.  The passed-over TDs
 * come out first, oldest first and one a call, with out->missed set and
 * nothing transferred: the caller calls again until it has the event's own TD.
 */
BOOL xhci_td_complete_by_trb(TransferDescriptorList *td_list, dma_addr_t trb_addr,
                             u32 residue, BOOL short_packet,
                             struct xhci_td_completion *out, BOOL *deferred)
{
    *deferred = FALSE;

    if (!td_list)
        return FALSE;

    u32 idx;
    struct xhci_td *td = find_td_by_trb(td_list, trb_addr, &idx);
    if (!td)
        return FALSE;

    struct xhci_td *oldest = (struct xhci_td *)td_list->list.mlh_Head;
    out->missed = td->is_rt_iso && td != oldest;
    if (out->missed)
    {
        td_consume(td_list, oldest, 0, out);
        return TRUE;
    }

    u32 act_len;
    if (td->short_seen)
        act_len = td->short_act_len; /* fixed by a short packet on an earlier TRB */
    else if (idx + 1 == td->trb_count)
        /* the final TRB's residue is the TD's: td_bytes_through() without
         * reading the ring back, on every completion */
        act_len = (td->length > residue) ? td->length - residue : 0;
    else
    {
        act_len = td_bytes_through(td, idx, residue);
        if (short_packet)
        {
            /* the TD ends with its final TRB's event (xHCI 4.10.1.1) */
            td->short_act_len = act_len;
            td->short_seen = TRUE;
            *deferred = TRUE;
            KprintfT("mid-TD short at TRB %lu/%lu: act_len=%lu\n",
                     (ULONG)idx, (ULONG)td->trb_count, (ULONG)act_len);
            return FALSE;
        }
        /* an error (stall, babble, transaction error) ends the TD here */
    }

    td_consume(td_list, td, act_len, out);
    return TRUE;
}

/*
 * The one place a dequeue pointer for a transfer ring is made.
 *
 * A stopped endpoint keeps its own position, and a doorbell makes it carry on
 * from there.  The pointer only has to be moved when the TD the controller
 * stopped in is no longer on the list - it failed with a halt, or it was
 * retired.  Then the ring goes on at the oldest TD left, or at the software
 * enqueue if none is left.
 *
 * The cycle bit comes from the TRB itself, not from the endpoint context's
 * DCS: the VL805 writes that field back wrong (Linux XHCI_EP_CTX_BROKEN_DCS),
 * and the TRB's cycle is what the controller has to match anyway.
 */
dma_addr_t xhci_td_first_deq(TransferDescriptorList *td_list)
{
    struct xhci_td *oldest = (struct xhci_td *)td_list->list.mlh_Head;
    if (oldest->node.mln_Succ)
        return xhci_ring_get_deq_ptr_for_trb(td_list->ring, oldest->trb_addrs[0]);

    return xhci_ring_get_new_dequeue_ptr(td_list->ring);
}

/**
 * xhci_td_oldest_trb() - First TRB of the oldest TD on a ring.
 * @td_list: the ring's TD list.
 *
 * TDs sit on the list in ring order, so the head's first TRB is where the
 * controller's work on the ring begins: the room check keeps the enqueue out
 * of its segment (xhci_ring_td_fits()).
 *
 * Return: the TRB's ring address, 0 if the list is empty.
 */
dma_addr_t xhci_td_oldest_trb(TransferDescriptorList *td_list)
{
    struct xhci_td *oldest = (struct xhci_td *)td_list->list.mlh_Head;
    return oldest->node.mln_Succ ? oldest->trb_addrs[0] : 0;
}

void xhci_td_fail_all(TransferDescriptorList *td_list, s8 io_Error)
{
    if (!td_list)
        return;
    struct ExecBase *SysBase = td_list->sysBase;

    struct MinNode *n;
    while ((n = RemHeadMinList(&td_list->list)) != NULL)
    {
        struct xhci_td *td = (struct xhci_td *)n;
        td_unmap_and_reply(td_list, td, io_Error, 0);
        xhci_td_free(td_list, td);
    }

    td_list->queued_tds = 0;
}

/*
 * Retire TDs of a stopped ring, and say whether the ring has to be re-armed.
 *
 * The victims are the TDs of the listed requests and the TDs past their
 * deadline - or, with all, every TD.  A victim's TRBs become No-Ops, which
 * the controller steps over, and its request is answered: UHIOERR_NAKTIMEOUT
 * for an expired one (a deadline is a NAK timeout, not "device dead" - the
 * stack weighs UHIOERR_TIMEOUT three times worse), IOERR_ABORTED otherwise.
 * Either way with the bytes its fully consumed TRBs moved: Neptune's bulk
 * streams go on after a NAK timeout with a non-zero actual.  The TRB the
 * controller was working on counts as not moved.
 *
 * stopped_deq is where the controller stopped on this ring.  If that is still
 * inside a TD of the list, the controller carries on by itself and 0 is
 * returned; so it is when nothing was retired.  Otherwise the TD it stopped
 * in is gone and the return value is where to re-arm (xhci_td_first_deq).
 */
dma_addr_t xhci_td_retire(TransferDescriptorList *td_list, IOReqList *abort_reqs, u32 now_us,
                          dma_addr_t stopped_deq, BOOL all)
{
    if (!td_list)
        return 0;
    struct ExecBase *SysBase = td_list->sysBase;

    const dma_addr_t stopped_trb = stopped_deq & ~(dma_addr_t)EP_CTX_CYCLE_MASK;
    BOOL retired = FALSE;
    BOOL stopped_in_survivor = FALSE;

    struct MinNode *next;
    for (struct MinNode *node = td_list->list.mlh_Head; (next = node->mln_Succ) != NULL; node = next)
    {
        struct xhci_td *td = (struct xhci_td *)node;
        const s32 stopped_idx = td_find_trb_index(td, stopped_trb);
        const BOOL expired = td_is_expired_at(td, now_us);

        if (!all && !expired && !td_is_aborted(td, abort_reqs))
        {
            stopped_in_survivor |= stopped_idx >= 0;
            continue;
        }

        const u32 actual = (stopped_idx > 0) ? td_sum_trb_lengths(td, (u32)stopped_idx) : 0;

        /* stopped_idx < 0: the controller never reached this TD */
        Kprintf("retiring TD: first TRB %08lx, stopped at %08lx (index %ld), actual %lu, %s\n",
                (ULONG)td->trb_addrs[0], (ULONG)stopped_trb, (LONG)stopped_idx,
                actual, expired ? "expired" : "aborted");

        xhci_ring_patch_trbs_to_noop(td_list->ring, td->trb_addrs, td->trb_count, 0);
        RemoveMinNode(node);
        xhci_td_decrease_queued(td_list);
        td_unmap_and_reply(td_list, td, expired ? UHIOERR_NAKTIMEOUT : IOERR_ABORTED, actual);
        xhci_td_free(td_list, td);
        retired = TRUE;
    }

    if (!retired || stopped_in_survivor)
        return 0;

    return xhci_td_first_deq(td_list);
}
