#ifndef __XHCI_TD_H
#define __XHCI_TD_H

#include <exec/lists.h>
#include <exec/types.h>
#include <devices/usbhardware.h>
#include <compat.h>

typedef struct MinList IOReqList;

typedef struct {
    struct MinList list;
    struct SignalSemaphore semaphore;
    struct xhci_ctrl *ctrl;
    APTR memoryPool;
    ULONG queued_trbs;
} TransferDescriptorList;

TransferDescriptorList* xhci_td_create_list(struct xhci_ctrl *ctrl);
void xhci_td_destroy_list(TransferDescriptorList *td_list, UBYTE error_code);

BOOL xhci_td_is_empty(TransferDescriptorList *td_list);
BOOL xhci_td_is_expired(TransferDescriptorList *td_list);
ULONG xhci_td_get_queued_count(TransferDescriptorList *td_list);

BOOL xhci_td_add(TransferDescriptorList *td_list,
    struct IOUsbHWReq *io_req,
    ULONG timeout_ms,
    BOOL is_rt_iso,
    dma_addr_t *trb_addresses,
    ULONG trb_count);

struct IOUsbHWReq *xhci_td_get_by_trb(TransferDescriptorList *td_list, dma_addr_t trb_addr);

void xhci_td_abort_req(struct xhci_ctrl *ctrl, struct IOUsbHWReq *io);
void xhci_td_fail_all(TransferDescriptorList *td_list, BYTE io_Error);

#endif