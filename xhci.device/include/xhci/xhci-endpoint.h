#ifndef __XHCI_ENDPOINT_H__
#define __XHCI_ENDPOINT_H__

#include <devices/usbhardware.h>

enum ep_state
{
    USB_DEV_EP_STATE_IDLE = 0,
    USB_DEV_EP_STATE_RECEIVING,
    USB_DEV_EP_STATE_RECEIVING_CONTROL_SHORT,
    USB_DEV_EP_STATE_ABORTING,
    USB_DEV_EP_STATE_RESETTING,
    USB_DEV_EP_STATE_FAILED,
    USB_DEV_EP_STATE_RT_ISO_STOPPED,
    USB_DEV_EP_STATE_RT_ISO_RUNNING,
    USB_DEV_EP_STATE_RT_ISO_STOPPING
};

struct usb_device;
struct ep_context;

BOOL xhci_ep_create_context(struct usb_device *udev, int ep_index, APTR memoryPool);
void xhci_ep_destroy_contexts(struct usb_device *udev, BYTE reply_code);
struct ep_context *xhci_ep_get_context_for_index(struct usb_device *udev, int ep_index);

void xhci_ep_set_failed(struct ep_context *ep_ctx);
void xhci_ep_set_idle(struct ep_context *ep_ctx);
void xhci_ep_set_receiving(struct ep_context *ep_ctx, struct IOUsbHWReq *req, dma_addr_t *trb_addrs, ULONG timeout_ms, unsigned int trb_count);
void xhci_ep_set_receiving_control_short(struct ep_context *ep_ctx);
void xhci_ep_set_resetting(struct ep_context *ep_ctx);
void xhci_ep_set_aborting(struct ep_context *ep_ctx);

BOOL xhci_ep_is_expired(struct ep_context *ep_ctx);

enum ep_state xhci_ep_get_state(struct ep_context *ep_ctx);
int xhci_ep_get_ep_index(struct ep_context *ep_ctx);
int xhci_ep_get_active_trb_count(struct ep_context *ep_ctx);
struct xhci_ring *xhci_ep_get_ring(struct ep_context *ep_ctx);

struct IOUsbHWReq *xhci_ep_get_by_trb(struct ep_context *ep_ctx, dma_addr_t trb_addr);
void xhci_ep_enqueue(struct ep_context *ep_ctx, struct IOUsbHWReq *io);
void xhci_ep_flush(struct ep_context *ep_ctx, BYTE reply_code);

/* RT ISO functions */
BYTE xhci_ep_rt_iso_add_handler(struct ep_context *ep_ctx, struct IOUsbHWReq *req);
BYTE xhci_ep_rt_iso_rem_handler(struct ep_context *ep_ctx, struct IOUsbHWReq *req);

BYTE xhci_ep_rt_iso_start(struct ep_context *ep_ctx);
BYTE xhci_ep_rt_iso_stop(struct ep_context *ep_ctx, struct IOUsbHWReq *req);

void xhci_ep_rt_iso_in(struct ep_context *ep_ctx, struct IOUsbHWReq *req, ULONG act_len);
void xhci_ep_rt_iso_out(struct ep_context *ep_ctx, struct IOUsbHWReq *req, ULONG act_len);

void xhci_ep_schedule_rt_iso(struct ep_context *ep_ctx);


#endif /* __XHCI_ENDPOINT_H__ */