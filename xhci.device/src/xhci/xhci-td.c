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

static inline void xhci_copy_from_bounce_buffer(CONST_APTR src, APTR dst, ULONG size)
{
    if ((((ULONG)src | (ULONG)dst | size) & (sizeof(ULONG) - 1)) == 0)
    {
        CopyMemQuick((ULONG *)src, (ULONG *)dst, size);
        return;
    }

    CopyMem(src, dst, size);
}

struct xhci_td
{
    struct MinNode node;       /* linkage in active TD list */
    struct USBIORequest *req;  /* owning request */
    dma_addr_t completion_trb; /* TRB address we expect a completion for */
    ULONG length;              /* total length for completion accounting */
    BOOL deadline_active;      /* true if deadline_us is valid */
    ULONG deadline_us;         /* absolute deadline in usec, 0 means no timeout */
    BOOL is_rt_iso;            /* true if this TD is part of RT ISO pipeline */
    UWORD trb_count;           /* number of TRBs consumed by this TD */
    dma_addr_t *trb_addrs;     /* DMA addresses for every TRB in this TD */
};

struct TransferDescriptorList
{
    struct MinList list;
    struct xhci_ctrl *ctrl;
    APTR memoryPool;
    ULONG queued_trbs;
    ULONG queued_tds;
};

TransferDescriptorList *xhci_td_create_list(struct xhci_ctrl *ctrl)
{
    TransferDescriptorList *td_list = pool_zalloc(ctrl->memoryPool, sizeof(TransferDescriptorList));
    if (!td_list)
    {
        Kprintf("Failed to alloc TransferDescriptorList\n");
        return NULL;
    }

    _NewMinList(&td_list->list);
    td_list->ctrl = ctrl;
    td_list->memoryPool = ctrl->memoryPool;
    td_list->queued_trbs = 0;
    td_list->queued_tds = 0;

    return td_list;
}

void xhci_td_destroy_list(TransferDescriptorList *td_list, UBYTE error_code)
{
    if (!td_list)
        return;

    xhci_td_fail_all(td_list, error_code);

    pool_free(td_list->memoryPool, td_list);
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

    ULONG now = get_time();

    struct MinNode *n = td_list->list.mlh_Head;
    while (n && n->mln_Succ)
    {
        struct xhci_td *td = (struct xhci_td *)n;
        if (td->deadline_active && (int32_t)(now - td->deadline_us) >= 0)
        {
            KprintfH("Found expired TD req=%lx deadline=%lu now=%lu\n",
                     td->req,
                     (ULONG)td->deadline_us,
                     (ULONG)now);
            return TRUE;
        }
        n = n->mln_Succ;
    }

    return FALSE;
}

ULONG xhci_td_get_queued_trb_count(TransferDescriptorList *td_list)
{
    if (!td_list)
        return 0;

    return td_list->queued_trbs;
}

ULONG xhci_td_get_queued_td_count(TransferDescriptorList *td_list)
{
    if (!td_list)
        return 0;

    return td_list->queued_tds;
}

static struct xhci_td *td_create(TransferDescriptorList *td_list,
                                 struct USBIORequest *io_req,
                                 ULONG timeout_ms,
                                 BOOL is_rt_iso,
                                 dma_addr_t *trb_addresses,
                                 ULONG trb_count)
{
    if (!td_list)
        return NULL;

    struct xhci_td *td = pool_zalloc(td_list->memoryPool, sizeof(struct xhci_td));
    if (!td)
    {
        Kprintf("Failed to alloc xhci_td\n");
        return NULL;
    }

    td->req = io_req;

    io_req->driver_private_flags |= REQ_ON_RING;

    td->deadline_active = (timeout_ms != 0);
    td->deadline_us = get_time() + timeout_ms * 1000UL;

    td->length = io_req->data_buffer_length;
    td->trb_count = trb_count;
    td->trb_addrs = trb_addresses;
    td->completion_trb = trb_addresses[trb_count - 1];

    td->is_rt_iso = is_rt_iso;

    return td;
}

BOOL xhci_td_add(TransferDescriptorList *td_list,
                 struct USBIORequest *io_req,
                 ULONG timeout_ms,
                 BOOL is_rt_iso,
                 dma_addr_t *trb_addresses,
                 ULONG trb_count)
{
    if (!td_list)
        return FALSE;

    struct xhci_td *td = td_create(td_list,
                                   io_req,
                                   timeout_ms,
                                   is_rt_iso,
                                   trb_addresses,
                                   trb_count);
    if (!td)
        return FALSE;

    td_list->queued_trbs += trb_count;
    td_list->queued_tds++;

    AddTailMinList(&td_list->list, (struct MinNode *)td);
    return TRUE;
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
            for (unsigned int i = 0; i < td->trb_count; i++)
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

inline static void xhci_dma_unmap(struct xhci_ctrl *ctrl, struct USBIORequest *req, BOOL copy)
{
    if (!ctrl || !req)
        return;

    APTR addr = req->data_buffer;
    ULONG size = req->data_buffer_length;
    if (!addr || size == 0)
        return;

    if (!(req->driver_private_flags & REQ_DMA_MAPPED))
    {
        if (copy)
            xhci_inval_cache(addr, size);
        return;
    }

    APTR bounce = (APTR)req->driver_private_dma_address;
    if (!bounce)
    {
        Kprintf("No bounce buffer found for unmap of %lx len=%ld\n", (ULONG)addr, (LONG)size);
        return;
    }

    if (copy)
    {
        xhci_inval_cache(bounce, size);
        xhci_copy_from_bounce_buffer(bounce, addr, size);
    }

    dma_free(ctrl->memoryPool, bounce);
    req->driver_private_flags &= ~REQ_DMA_MAPPED;
    req->driver_private_dma_address = NULL;
}

static void xhci_td_free(TransferDescriptorList *td_list, struct xhci_td *td)
{
    if (td->trb_addrs)
    {
        pool_free(td_list->memoryPool, td->trb_addrs);
        td->trb_addrs = NULL;
    }

    pool_free(td_list->memoryPool, td);
}

BOOL xhci_td_has_request(TransferDescriptorList *td_list, struct USBIORequest *io_req)
{
    if (!td_list || !io_req)
        return FALSE;

    struct MinNode *n = td_list->list.mlh_Head;
    while (n && n->mln_Succ)
    {
        struct xhci_td *td = (struct xhci_td *)n;
        if (td->req == io_req)
            return TRUE;
        n = n->mln_Succ;
    }

    return FALSE;
}

static inline BOOL td_is_expired_at(struct xhci_td *td, ULONG now)
{
    return td && td->deadline_active && (int32_t)(now - td->deadline_us) >= 0;
}

static WORD td_find_trb_index(struct xhci_td *td, dma_addr_t trb_addr)
{
    for (UWORD index = 0; index < td->trb_count; ++index)
    {
        if (td->trb_addrs[index] == trb_addr)
            return (WORD)index;
    }

    return -1;
}

static void td_unmap_and_reply(TransferDescriptorList *td_list, struct xhci_td *td, BYTE error_code)
{
    struct USBIORequest *req = td->req;

    if (req && req->data_buffer_length > 0)
        xhci_dma_unmap(td_list->ctrl, req, FALSE);

    if (req)
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
                                        ULONG now_us)
{
    if (td_is_expired_at(td, now_us))
        return TRUE;

    return td_req_is_recovery_abort(abort_reqs, td->req);
}

static void td_resolve_recovery_deq_ptr(TransferDescriptorList *td_list,
                                        struct xhci_ring *ring,
                                        IOReqList *abort_reqs,
                                        ULONG now_us,
                                        dma_addr_t stopped_deq_ptr,
                                        dma_addr_t *resolved_deq_ptr)
{
    const dma_addr_t stopped_trb_addr = stopped_deq_ptr & ~(dma_addr_t)EP_CTX_CYCLE_MASK;
    *resolved_deq_ptr = 0;

    BOOL passed_stopped_trb = FALSE;
    struct MinNode *node = td_list->list.mlh_Head;

    while (node && node->mln_Succ)
    {
        struct xhci_td *td = (struct xhci_td *)node;
        BOOL recovery_abort = td_is_recovery_abort(td, abort_reqs, now_us);

        if (!passed_stopped_trb && td_find_trb_index(td, stopped_trb_addr) >= 0)
        {
            passed_stopped_trb = TRUE;
            if (!recovery_abort)
            {
                *resolved_deq_ptr = stopped_deq_ptr;
                return;
            }
            node = node->mln_Succ;
            continue;
        }

        if (passed_stopped_trb && !recovery_abort)
        {
            *resolved_deq_ptr = xhci_ring_get_deq_ptr_for_trb(td->trb_addrs[0]);
            return;
        }

        node = node->mln_Succ;
    }

    if (!passed_stopped_trb)
    {
        Kprintf("Stopped TRB address %lx not found in any TD\n", (ULONG)stopped_trb_addr);
        return;
    }

    if (!*resolved_deq_ptr)
        *resolved_deq_ptr = xhci_ring_get_new_dequeue_ptr(ring);
}

static void td_abort_recovery_requests(TransferDescriptorList *td_list,
                                       IOReqList *abort_reqs,
                                       ULONG now_us)
{
    struct MinNode *node = td_list->list.mlh_Head;

    while (node && node->mln_Succ)
    {
        struct xhci_td *td = (struct xhci_td *)node;
        struct MinNode *next = node->mln_Succ;
        BOOL recovery_abort = td_is_recovery_abort(td, abort_reqs, now_us);

        if (recovery_abort)
        {
            xhci_ring_patch_trbs_to_noop(td->trb_addrs, td->trb_count, 0);

            RemoveMinNode((struct MinNode *)td);
            xhci_td_decrease_queued(td_list, td);
            td_unmap_and_reply(td_list, td, td_is_expired_at(td, now_us) ? ERR_TIMEOUT : IOERR_ABORTED);
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

    ULONG now_us = get_time();

    td_resolve_recovery_deq_ptr(td_list, ring,
                                abort_reqs, now_us,
                                stopped_deq_ptr, new_deq_ptr);
    td_abort_recovery_requests(td_list, abort_reqs, now_us);
}

/*
 * This is taking a Transfer Descriptor off the list.
 * If TD containing specified TRB address is found, it is removed from the list.
 * The TD is finalized and freed,
 * if it contains a IOUsbHWReq, the request is returned to the caller.
 */
struct USBIORequest *xhci_td_get_by_trb(TransferDescriptorList *td_list, dma_addr_t trb_addr)
{
    if (!td_list)
        return NULL;

    struct xhci_td *td = find_td_by_trb(td_list, trb_addr);
    if (!td)
        return NULL;

    struct USBIORequest *req = td->req;

    RemoveMinNode((struct MinNode *)td);
    xhci_td_decrease_queued(td_list, td);
    xhci_td_free(td_list, td);

    if (req && req->data_buffer_length > 0)
    {
        BOOL need_data;
        if (req->req.io_Command == CMD_REQUEST_CONTROL)
            need_data = (req->setup.bmRequestType & USB_DIR_IN) != 0;
        else
            need_data = (req->direction == DIRECTION_IN);

        xhci_dma_unmap(td_list->ctrl, req, need_data);
    }

    return req;
}

void xhci_td_fail_all(TransferDescriptorList *td_list, BYTE io_Error)
{
    if (!td_list)
        return;

    struct MinNode *n;
    while ((n = RemHeadMinList(&td_list->list)) != NULL)
    {
        struct xhci_td *td = (struct xhci_td *)n;
        if (td->req)
        {
            if (td->req->data_buffer)
                xhci_dma_unmap(td_list->ctrl, td->req, FALSE);
            if (td->is_rt_iso && td->req->direction == DIRECTION_IN && td->req->data_buffer)
                pool_free(td_list->memoryPool, td->req->data_buffer);
            if (td->is_rt_iso)
                pool_free(td_list->memoryPool, td->req);
            else
                xhci_udev_io_reply_failed(td_list->ctrl, td->req, io_Error);
        }
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

    int ep_index = xhci_ep_index_from_parts(io->endpoint, io->direction);
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
