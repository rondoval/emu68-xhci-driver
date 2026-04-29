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

void xhci_td_slab_init(struct xhci_ctrl *ctrl);
void xhci_td_slab_destroy(struct xhci_ctrl *ctrl);

TransferDescriptorList* xhci_td_create_list(struct xhci_ctrl *ctrl);
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
    BOOL is_rt_iso,
    dma_addr_t *trb_addresses,
    u32 trb_count);

struct USBIORequest *xhci_td_get_by_trb(TransferDescriptorList *td_list, dma_addr_t trb_addr);

void xhci_td_abort_req(struct USBIORequest *io);
void xhci_td_fail_all(TransferDescriptorList *td_list, s8 io_Error);

#endif