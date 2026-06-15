/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __XHCI_TD_H
#define __XHCI_TD_H

#include <exec/types.h>
#include <devices/hcd_api.h>
#include <minlist.h>

typedef struct MinList IOReqList;

typedef struct IOReqNode {
    struct MinNode node;
    struct USBIORequest *req;
} IOReqNode;

typedef struct TransferDescriptorList TransferDescriptorList;
struct xhci_ctrl;
struct xhci_ring;
struct ep_context;
struct xhci_dma_span;

/* Result of completing a TD: either a request to reply (rt == FALSE) or the
 * payload of an RT ISO TD (no request object exists for those). */
struct xhci_td_completion
{
	BOOL rt;
	struct USBIORequest *req; /* !rt */
	APTR rt_buffer;           /* rt: CPU buffer (staging for IN, class buffer for OUT) */
	u32 rt_length;            /* rt: submitted length */
	u16 rt_frame;
	u16 rt_dir;               /* DIRECTION_IN / DIRECTION_OUT */
	u32 act_len;
};

void xhci_td_slab_init(struct xhci_ctrl *ctrl);
void xhci_td_slab_destroy(struct xhci_ctrl *ctrl);

TransferDescriptorList* xhci_td_create_list(struct xhci_ctrl *ctrl, struct ep_context *ep_ctx);
void xhci_td_destroy_list(TransferDescriptorList *td_list, s8 error_code);

BOOL xhci_td_is_empty(TransferDescriptorList *td_list);
BOOL xhci_td_is_expired(TransferDescriptorList *td_list);
u32 xhci_td_get_queued_trb_count(TransferDescriptorList *td_list);
u32 xhci_td_get_queued_td_count(TransferDescriptorList *td_list);
BOOL xhci_td_has_request(TransferDescriptorList *td_list, struct USBIORequest *io_req);
void xhci_td_patch_recovery(TransferDescriptorList *td_list,
    struct xhci_ring *ring,
    IOReqList *abort_reqs,
    dma_addr_t stopped_deq_ptr,
    dma_addr_t *new_deq_ptr);

BOOL xhci_td_add(TransferDescriptorList *td_list,
    struct USBIORequest *io_req,
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

void xhci_td_abort_req(struct USBIORequest *io);
void xhci_td_fail_all(TransferDescriptorList *td_list, s8 io_Error);

#endif