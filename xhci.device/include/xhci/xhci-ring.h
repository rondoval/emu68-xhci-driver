#ifndef __XHCI_RING_H
#define __XHCI_RING_H

#include <exec/types.h>
#include <compat.h>
#include <xhci/xhci.h>

struct xhci_ring *xhci_ring_alloc(struct xhci_ctrl *ctrl, unsigned int num_segs,
								  BOOL link_trbs, BOOL is_event_ring, int ep_index);
void xhci_ring_free(struct xhci_ctrl *ctrl, struct xhci_ring *ring);								  

int xhci_ring_enqueue_td(struct usb_device *udev, struct IOUsbHWReq *io, unsigned int timeout_ms, BOOL defer_doorbell);
void xhci_ring_giveback(struct usb_device *udev, struct ep_context *ep_ctx);

void xhci_ring_acknowledge_event(struct xhci_ctrl *ctrl);
union xhci_trb *xhci_ring_get_event_trb(struct xhci_ring *ring);

u32 xhci_ring_get_new_dequeue_ptr(struct xhci_ring *ring);

dma_addr_t xhci_ring_enqueue_command(struct xhci_ring *ring, u64 address, u32 slot_id, u32 ep_index, trb_type cmd);
void xhci_ring_setup_erst(struct xhci_ring *ring, struct xhci_erst *erst, struct xhci_intr_reg *ir_set);

#endif /* __XHCI_RING_H */