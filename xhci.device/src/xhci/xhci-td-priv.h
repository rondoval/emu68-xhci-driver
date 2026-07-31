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

/* One TD list per transfer ring; creation registers the list on its ring
 * (xhci_ring_set_td_list), destruction clears it. */
TransferDescriptorList* xhci_td_create_list(struct xhci_ctrl *ctrl, struct ep_context *ep_ctx,
    struct xhci_ring *ring);
void xhci_td_destroy_list(TransferDescriptorList *td_list, s8 error_code);

BOOL xhci_td_is_empty(TransferDescriptorList *td_list);
BOOL xhci_td_is_expired(TransferDescriptorList *td_list);
/* the on-ring direct transfer with this cookie, or NULL */
struct xhci_xfer *xhci_td_find_cookie_request(TransferDescriptorList *td_list, APTR cookie);
u32 xhci_td_get_queued_td_count(TransferDescriptorList *td_list);
BOOL xhci_td_has_request(TransferDescriptorList *td_list, struct xhci_xfer *io_req);
/* Does this ring's list hold a recovery victim (an abort-listed or expired
 * TD)?  The per-ring pre-check that keeps recovery surgical: rings without
 * victims are skipped whole and their TDs keep running. */
BOOL xhci_td_has_recovery_victim(TransferDescriptorList *td_list,
    IOReqList *abort_reqs, u32 now_us);

/* Ring recovery, split so the caller can validate before mutating (both
 * halves take the same now_us so the victim set cannot shift between them).
 *
 * Resolve (non-destructive): the re-arm dequeue for this ring's victim set —
 * the stopped TRB itself when the hardware halted inside a survivor, a later
 * survivor's first TRB, or the software enqueue when everything from the
 * stop point on dies.  0 = the stopped dequeue lies outside every tracked TD
 * (anomaly; the caller falls back to coarse whole-endpoint recovery). */
dma_addr_t xhci_td_resolve_recovery(TransferDescriptorList *td_list,
    struct xhci_ring *ring,
    IOReqList *abort_reqs,
    u32 now_us,
    dma_addr_t stopped_deq_ptr);

/* Abort (destructive): No-Op every victim's TRBs and reply them —
 * UHIOERR_NAKTIMEOUT for expired TDs, IOERR_ABORTED otherwise, with the
 * partial actual recovered from the fully-consumed TRBs. */
void xhci_td_abort_recovery(TransferDescriptorList *td_list,
    IOReqList *abort_reqs,
    u32 now_us,
    dma_addr_t stopped_deq_ptr);

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
