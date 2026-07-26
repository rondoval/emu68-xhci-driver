/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * TD-list internals shared by exactly two translation units: xhci-td.c (the
 * tracker) and the endpoint layer (xhci-endpoint.c / xhci-ep-rtiso.c via
 * xhci-endpoint-priv.h).  Deliberately in src/, not include/ — nothing else
 * may track TDs.  Every function requires ctrl->xfer_lock held.
 */

#ifndef __XHCI_TD_PRIV_H
#define __XHCI_TD_PRIV_H

#include <minlist.h>
#include <xhci/xhci-td.h>
#include <xhci/xhci-xfer.h>

typedef struct MinList IOReqList;

typedef struct IOReqNode {
    struct MinNode node;
    struct xhci_xfer *req;
} IOReqNode;

typedef struct TransferDescriptorList TransferDescriptorList;
struct xhci_ring;
struct ep_context;
struct xhci_dma_span;

TransferDescriptorList* xhci_td_create_list(struct xhci_ctrl *ctrl, struct ep_context *ep_ctx);
void xhci_td_destroy_list(TransferDescriptorList *td_list, s8 error_code);

BOOL xhci_td_is_empty(TransferDescriptorList *td_list);
BOOL xhci_td_is_expired(TransferDescriptorList *td_list);
/* the on-ring direct transfer with this cookie, or NULL */
struct xhci_xfer *xhci_td_find_cookie_request(TransferDescriptorList *td_list, APTR cookie);
u32 xhci_td_get_queued_trb_count(TransferDescriptorList *td_list);
u32 xhci_td_get_queued_td_count(TransferDescriptorList *td_list);
BOOL xhci_td_has_request(TransferDescriptorList *td_list, struct xhci_xfer *io_req);
void xhci_td_patch_recovery(TransferDescriptorList *td_list,
    struct xhci_ring *ring,
    IOReqList *abort_reqs,
    dma_addr_t stopped_deq_ptr,
    dma_addr_t *new_deq_ptr);

/* Mark the streams owning an abort-listed or expired TD; TRUE if any. */
BOOL xhci_td_mark_recovery_streams(TransferDescriptorList *td_list,
    IOReqList *abort_reqs,
    u32 now_us,
    u32 *map, u16 num_streams);

/* Fail only the marked streams' TDs: UHIOERR_NAKTIMEOUT for expired TDs,
 * IOERR_ABORTED for their ring-mates, actual 0 either way. */
void xhci_td_fail_streams(TransferDescriptorList *td_list, const u32 *map, u32 now_us);

BOOL xhci_td_add(TransferDescriptorList *td_list,
    struct xhci_xfer *io_req,
    u32 timeout_ms,
    dma_addr_t *trb_addresses,
    u32 trb_count);

/* RT ISO TD: the TD itself owns the mapped span; staging marks an IN buffer
 * from the endpoint's staging slab (freed by the completion/teardown paths). */
BOOL xhci_td_add_rt(TransferDescriptorList *td_list,
    const struct xhci_dma_span *span,
    u16 frame, u16 dir, BOOL staging,
    dma_addr_t *trb_addresses,
    u32 trb_count);

/* Complete the TD containing trb_addr with an exact transferred length, or -
 * for a short packet on a non-final TRB - record the length and keep the TD
 * until its final-TRB event (*deferred = TRUE, returns FALSE). */
BOOL xhci_td_complete_by_trb(TransferDescriptorList *td_list, dma_addr_t trb_addr,
                             u32 residue, BOOL short_packet,
                             struct xhci_td_completion *out, BOOL *deferred);

void xhci_td_fail_all(TransferDescriptorList *td_list, s8 io_Error);

#endif /* __XHCI_TD_PRIV_H */
