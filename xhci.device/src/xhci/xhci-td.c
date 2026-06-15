/* SPDX-License-Identifier: GPL-2.0-only */

#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME (*(struct ExecBase **)4UL)
#include <proto/exec.h>
#endif

#include <exec/errors.h>

#include <device.h>
#include <memory.h>
#include <timing.h>
#include <xhci/xhci-descriptors.h>
#include <xhci/xhci-context.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-ring.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci-td.h>
#include <xhci/xhci.h>
#include <minlist.h>
#include <debug.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-td] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci-td] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

struct xhci_td
{
    struct MinNode node; /* linkage in active TD list */
    BOOL is_rt_iso;      /* discriminates the payload union */
    union
    {
        struct USBIORequest *req; /* owning request (!is_rt_iso) */
        struct
        {
            struct xhci_dma_span span; /* mapped buffer, owned by the TD */
            u16 frame;
            u16 dir;      /* DIRECTION_IN / DIRECTION_OUT */
            BOOL staging; /* IN buffer from the endpoint's staging slab */
        } rt;
    } u;
    dma_addr_t completion_trb; /* TRB address we expect a completion for */
    u32 length;                /* total length for completion accounting */
    BOOL deadline_active;      /* true if deadline_us is valid */
    u32 deadline_us;           /* absolute deadline in usec, 0 means no timeout */
    u32 trb_count;             /* number of TRBs consumed by this TD */
    dma_addr_t *trb_addrs;     /* DMA addresses for every TRB in this TD */

    /* Mid-TD short packet: exact transferred length recorded at the short
     * event; the TD completes on its final-TRB event with this value. */
    BOOL short_seen;
    u32 short_act_len;
};

struct TransferDescriptorList
{
    struct MinList list;
    struct xhci_ctrl *ctrl;
    u32 queued_trbs;
    u32 queued_tds;
    struct ep_context *ep_ctx; /* back-reference for per-endpoint resource cleanup */
};

void xhci_td_slab_init(struct xhci_ctrl *ctrl)
{
    slab_cache_init(&ctrl->td_slab, ctrl->metaPool, NULL, sizeof(struct xhci_td), DMA_ALIGN_MIN, 2048);
}

void xhci_td_slab_destroy(struct xhci_ctrl *ctrl)
{
    slab_cache_destroy(&ctrl->td_slab);
}

TransferDescriptorList *xhci_td_create_list(struct xhci_ctrl *ctrl, struct ep_context *ep_ctx)
{
    TransferDescriptorList *td_list = pool_zalloc(ctrl->metaPool, sizeof(TransferDescriptorList));
    if (!td_list)
    {
        Kprintf("Failed to alloc TransferDescriptorList\n");
        return NULL;
    }

    _NewMinList(&td_list->list);
    td_list->ctrl = ctrl;
    td_list->queued_trbs = 0;
    td_list->queued_tds = 0;
    td_list->ep_ctx = ep_ctx;

    return td_list;
}

void xhci_td_destroy_list(TransferDescriptorList *td_list, s8 error_code)
{
    if (!td_list)
        return;

    xhci_td_fail_all(td_list, error_code);

    pool_free(td_list->ctrl->metaPool, td_list);
}

BOOL xhci_td_is_empty(TransferDescriptorList *td_list)
{
    if (!td_list)
        return TRUE;
    return td_list->list.mlh_Head == (struct MinNode *)&td_list->list.mlh_Tail;
}

BOOL xhci_td_is_expired(TransferDescriptorList *td_list)
{
    if (!td_list)
        return FALSE;

    u32 now = get_time();

    struct MinNode *n = td_list->list.mlh_Head;
    while (n && n->mln_Succ)
    {
        struct xhci_td *td = (struct xhci_td *)n;
        if (td->deadline_active && (int32_t)(now - td->deadline_us) >= 0)
        {
            KprintfH("Found expired TD req=%lx deadline=%lu now=%lu\n",
                     td->is_rt_iso ? NULL : td->u.req,
                     (ULONG)td->deadline_us,
                     (ULONG)now);
            return TRUE;
        }
        n = n->mln_Succ;
    }

    return FALSE;
}

u32 xhci_td_get_queued_trb_count(TransferDescriptorList *td_list)
{
    if (!td_list)
        return 0;

    return td_list->queued_trbs;
}

u32 xhci_td_get_queued_td_count(TransferDescriptorList *td_list)
{
    if (!td_list)
        return 0;

    return td_list->queued_tds;
}

static struct xhci_td *td_alloc_common(TransferDescriptorList *td_list,
                                       dma_addr_t *trb_addresses, u32 trb_count)
{
    struct xhci_td *td = slab_zalloc(&td_list->ctrl->td_slab);
    if (!td)
    {
        Kprintf("Failed to alloc xhci_td\n");
        return NULL;
    }

    td->trb_count = trb_count;
    td->trb_addrs = trb_addresses;
    td->completion_trb = trb_addresses[trb_count - 1];
    return td;
}

static void td_append(TransferDescriptorList *td_list, struct xhci_td *td)
{
    td_list->queued_trbs += td->trb_count;
    td_list->queued_tds++;
    AddTailMinList(&td_list->list, (struct MinNode *)td);
}

BOOL xhci_td_add(TransferDescriptorList *td_list,
                 struct USBIORequest *io_req,
                 u32 timeout_ms,
                 dma_addr_t *trb_addresses,
                 u32 trb_count)
{
    if (!td_list)
        return FALSE;

    struct xhci_td *td = td_alloc_common(td_list, trb_addresses, trb_count);
    if (!td)
        return FALSE;

    td->u.req = io_req;
    io_req->driver_private_flags |= REQ_ON_RING;

    td->deadline_active = (timeout_ms != 0);
    td->deadline_us = (timeout_ms != 0) ? get_time() + timeout_ms * 1000UL : 0;
    td->length = io_req->data_buffer_length;

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

    struct xhci_td *td = td_alloc_common(td_list, trb_addresses, trb_count);
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
 * controller never modifies them.  Mirrors Linux sum_trb_lengths(). */
static u32 td_sum_trb_lengths(struct xhci_td *td, u32 idx)
{
    u32 first = (!td->is_rt_iso && td->u.req &&
                 td->u.req->req.io_Command == CMD_REQUEST_CONTROL)
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

static struct xhci_td *find_td_by_trb(TransferDescriptorList *td_list, dma_addr_t trb_addr)
{
    if (!td_list)
        return NULL;

    struct MinNode *n = td_list->list.mlh_Head;
    while (n && n->mln_Succ)
    {
        struct xhci_td *td = (struct xhci_td *)n;
        if (td->completion_trb == trb_addr)
            return td;

        if (td->trb_addrs)
        {
            for (u32 i = 0; i < td->trb_count; i++)
            {
                if (td->trb_addrs[i] == trb_addr)
                    return td;
            }
        }
        n = n->mln_Succ;
    }

    return NULL;
}

static void xhci_td_decrease_queued(TransferDescriptorList *td_list, struct xhci_td *td)
{
    if (td->trb_count > 0)
    {
        if (td_list->queued_trbs >= td->trb_count)
            td_list->queued_trbs -= td->trb_count;
        else
            td_list->queued_trbs = 0;
    }

    if (td_list->queued_tds > 0)
        td_list->queued_tds--;
}

static void xhci_td_free(TransferDescriptorList *td_list, struct xhci_td *td)
{
    if (td->trb_addrs)
    {
        if (td->trb_count <= XHCI_TD_SMALL_TRBS)
            slab_free(&td_list->ctrl->trb_addr_slab, td->trb_addrs);
        else
            pool_free(td_list->ctrl->metaPool, td->trb_addrs);
        td->trb_addrs = NULL;
    }

    slab_free(&td_list->ctrl->td_slab, td);
}

BOOL xhci_td_has_request(TransferDescriptorList *td_list, struct USBIORequest *io_req)
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

static inline BOOL td_is_expired_at(struct xhci_td *td, u32 now)
{
    return td && td->deadline_active && (int32_t)(now - td->deadline_us) >= 0;
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

static void td_unmap_and_reply(TransferDescriptorList *td_list, struct xhci_td *td, BYTE error_code, u32 actual)
{
    /* RT ISO TDs have no request to reply: release the mapped span and the
     * IN staging buffer, drop the data. */
    if (td->is_rt_iso)
    {
        xhci_dma_span_unmap(td_list->ctrl, &td->u.rt.span, FALSE);
        if (td->u.rt.staging)
            xhci_ep_free_rt_iso_buffer(td_list->ep_ctx, td->u.rt.span.cpu);
        return;
    }

    struct USBIORequest *req = td->u.req;
    if (!req)
        return;

    req->actual_length = actual;

    if (req->data_buffer_length > 0)
    {
        /* Copy bounce data back only when partial IN data is being reported */
        BOOL need_data = FALSE;
        if (actual != 0)
            need_data = (req->req.io_Command == CMD_REQUEST_CONTROL)
                            ? (req->setup.bmRequestType & USB_DIR_IN) != 0
                            : (req->direction == DIRECTION_IN);
        xhci_dma_unmap(td_list->ctrl, req, need_data);
    }

    xhci_udev_io_reply_failed(td_list->ctrl, req, error_code);
}

static inline BOOL td_req_is_recovery_abort(IOReqList *abort_reqs, struct USBIORequest *req)
{
    if (!abort_reqs || !req)
        return FALSE;

    struct MinNode *node = abort_reqs->mlh_Head;
    while (node && node->mln_Succ)
    {
        IOReqNode *abort_req_node = (IOReqNode *)node;
        if (abort_req_node->req == req)
            return TRUE;
        node = node->mln_Succ;
    }

    return FALSE;
}

static inline BOOL td_is_recovery_abort(struct xhci_td *td,
                                        IOReqList *abort_reqs,
                                        u32 now_us)
{
    if (td_is_expired_at(td, now_us))
        return TRUE;

    if (td->is_rt_iso)
        return FALSE;

    return td_req_is_recovery_abort(abort_reqs, td->u.req);
}

static void td_resolve_recovery_deq_ptr(TransferDescriptorList *td_list,
                                        struct xhci_ring *ring,
                                        IOReqList *abort_reqs,
                                        u32 now_us,
                                        dma_addr_t stopped_deq_ptr,
                                        dma_addr_t *resolved_deq_ptr)
{
    const dma_addr_t stopped_trb_addr = stopped_deq_ptr & ~(dma_addr_t)EP_CTX_CYCLE_MASK;
    *resolved_deq_ptr = 0;

    /* Locate the TD the hardware stopped in. */
    struct MinNode *node = td_list->list.mlh_Head;
    while (node && node->mln_Succ)
    {
        if (td_find_trb_index((struct xhci_td *)node, stopped_trb_addr) >= 0)
            break;
        node = node->mln_Succ;
    }

    if (!node || !node->mln_Succ)
    {
        Kprintf("Stopped TRB address %lx not found in any TD\n", (ULONG)stopped_trb_addr);
        return;
    }

    /* Re-arm at the first TD from the stop point onward that survives recovery:
     * the exact TRB the hardware halted on for the stopped TD, or a later
     * survivor's first TRB.  Take the cycle bit from the TRB itself, not the EP
     * context's DCS - the VL805 writes that field back wrong (Linux
     * XHCI_EP_CTX_BROKEN_DCS), and the TRB's cycle is what the consumer must
     * match anyway. */
    for (; node && node->mln_Succ; node = node->mln_Succ)
    {
        struct xhci_td *td = (struct xhci_td *)node;
        if (td_is_recovery_abort(td, abort_reqs, now_us))
            continue;

        dma_addr_t entry_trb = (td_find_trb_index(td, stopped_trb_addr) >= 0)
                                   ? stopped_trb_addr
                                   : td->trb_addrs[0];
        *resolved_deq_ptr = xhci_ring_get_deq_ptr_for_trb(entry_trb);
        return;
    }

    /* Everything from the stop point onward is being aborted: re-arm past it. */
    *resolved_deq_ptr = xhci_ring_get_new_dequeue_ptr(ring);
}

static void td_abort_recovery_requests(TransferDescriptorList *td_list,
                                       IOReqList *abort_reqs,
                                       u32 now_us,
                                       dma_addr_t stopped_trb_addr)
{
    struct MinNode *node = td_list->list.mlh_Head;

    while (node && node->mln_Succ)
    {
        struct xhci_td *td = (struct xhci_td *)node;
        struct MinNode *next = node->mln_Succ;
        BOOL recovery_abort = td_is_recovery_abort(td, abort_reqs, now_us);

        if (recovery_abort)
        {
            /* Bytes already moved by the TRBs the hardware fully consumed.
             * Poseidon's bulk streams continue on NAK_TIMEOUT with a non-zero
             * actual_length, so report the partial transfer rather than
             * discarding it (the in-progress TRB counts as untransferred -
             * conservative lower bound). */
            u32 actual = 0;
            s32 stopped_idx = td_find_trb_index(td, stopped_trb_addr);
            if (stopped_idx > 0)
                actual = td_sum_trb_lengths(td, (u32)stopped_idx);

            xhci_ring_patch_trbs_to_noop(td->trb_addrs, td->trb_count, 0);

            RemoveMinNode((struct MinNode *)td);
            xhci_td_decrease_queued(td_list, td);
            /* A deadline expiry is a NAK timeout, not "device dead": the
             * stack weighs ERR_TIMEOUT three times worse. */
            td_unmap_and_reply(td_list, td,
                               td_is_expired_at(td, now_us) ? ERR_NAK_TIMEOUT : IOERR_ABORTED,
                               actual);
            xhci_td_free(td_list, td);
        }

        node = next;
    }
}

void xhci_td_patch_recovery(TransferDescriptorList *td_list,
                            struct xhci_ring *ring,
                            IOReqList *abort_reqs,
                            dma_addr_t stopped_deq_ptr,
                            dma_addr_t *new_deq_ptr)
{
    if (!td_list || !ring || !new_deq_ptr || !stopped_deq_ptr)
        return;

    u32 now_us = get_time();

    td_resolve_recovery_deq_ptr(td_list, ring,
                                abort_reqs, now_us,
                                stopped_deq_ptr, new_deq_ptr);
    td_abort_recovery_requests(td_list, abort_reqs, now_us,
                               stopped_deq_ptr & ~(dma_addr_t)EP_CTX_CYCLE_MASK);
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
 */
BOOL xhci_td_complete_by_trb(TransferDescriptorList *td_list, dma_addr_t trb_addr,
                             u32 residue, BOOL short_packet,
                             struct xhci_td_completion *out, BOOL *deferred)
{
    *deferred = FALSE;

    if (!td_list)
        return FALSE;

    struct xhci_td *td = find_td_by_trb(td_list, trb_addr);
    if (!td)
        return FALSE;

    s32 idx = td_find_trb_index(td, trb_addr); /* >= 0: find_td_by_trb matched */

    if (short_packet && (u32)idx + 1 < td->trb_count)
    {
        const struct xhci_generic_trb *trb = (const struct xhci_generic_trb *)(uintptr_t)trb_addr;
        u32 trb_len = TRB_LEN(le32(trb->field[2]));
        td->short_act_len = td_sum_trb_lengths(td, (u32)idx) +
                            ((trb_len > residue) ? trb_len - residue : 0);
        td->short_seen = TRUE;
        *deferred = TRUE;
        KprintfH("mid-TD short at TRB %ld/%lu: act_len=%lu\n",
                 (LONG)idx, (ULONG)td->trb_count, (ULONG)td->short_act_len);
        return FALSE;
    }

    u32 act_len;
    if (td->short_seen)
        act_len = td->short_act_len;
    else if (td->length > residue)
        act_len = td->length - residue;
    else
        act_len = 0;

    out->rt = td->is_rt_iso;
    out->act_len = act_len;

    if (td->is_rt_iso)
    {
        out->req = NULL;
        out->rt_buffer = td->u.rt.span.cpu;
        out->rt_length = td->u.rt.span.length;
        out->rt_frame = td->u.rt.frame;
        out->rt_dir = td->u.rt.dir;
        /* The completion path consumes the data (and frees IN staging). */
        xhci_dma_span_unmap(td_list->ctrl, &td->u.rt.span,
                            td->u.rt.dir == DIRECTION_IN && act_len > 0);
    }
    else
    {
        struct USBIORequest *req = td->u.req;
        out->req = req;
        out->rt_buffer = NULL;
        out->rt_length = 0;
        out->rt_frame = 0;
        out->rt_dir = 0;

        if (req && req->data_buffer_length > 0)
        {
            BOOL need_data = (req->req.io_Command == CMD_REQUEST_CONTROL)
                                 ? (req->setup.bmRequestType & USB_DIR_IN) != 0
                                 : (req->direction == DIRECTION_IN);
            xhci_dma_unmap(td_list->ctrl, req, need_data);
        }
    }

    RemoveMinNode((struct MinNode *)td);
    xhci_td_decrease_queued(td_list, td);
    xhci_td_free(td_list, td);

    return TRUE;
}

void xhci_td_fail_all(TransferDescriptorList *td_list, s8 io_Error)
{
    if (!td_list)
        return;

    struct MinNode *n;
    while ((n = RemHeadMinList(&td_list->list)) != NULL)
    {
        struct xhci_td *td = (struct xhci_td *)n;
        td_unmap_and_reply(td_list, td, io_Error, 0);
        xhci_td_free(td_list, td);
    }

    td_list->queued_trbs = 0;
    td_list->queued_tds = 0;
}

/*
 * Abort the given IOUsbHWReq by removing it from any queues it may be on, and if it's already on the hardware ring, patching its TRBs to NOOP so that it won't complete.
 * The request is completed with IOERR_ABORTED if it was not yet completed, otherwise the abort is silently ignored.
 */
void xhci_td_abort_req(struct USBIORequest *io)
{
    if (!io || !io->req.io_Unit || io->virtual_address > USB_MAX_ADDRESS)
        return;

    if (io->req.io_Flags & IOF_QUICK || io->req.io_Message.mn_Node.ln_Type != NT_MESSAGE)
        return;

    struct XHCIUnit *unit = (struct XHCIUnit *)io->req.io_Unit;
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    struct usb_device *udev = ctrl->devices_by_virtual_address[io->virtual_address];
    if (!ctrl || !udev)
        return;

    u8 ep_index = xhci_ep_index_from_parts(io->endpoint, io->direction);
    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);

    if (io->driver_private_flags & REQ_ON_RING)
        xhci_ep_request_abort(ep_ctx, io);
    else
    {
        if (io->req.io_Command == CMD_REQUEST_INTERRUPT && io->virtual_address == xhci_roothub_get_address(ctrl->root_hub))
            xhci_roothub_abort_int_request(ctrl->root_hub);
        else
        {
            Remove(&io->req.io_Message.mn_Node);
            xhci_udev_io_reply_failed(ctrl, io, IOERR_ABORTED);
        }
    }
}
