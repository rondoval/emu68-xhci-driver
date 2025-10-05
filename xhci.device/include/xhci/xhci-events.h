#ifndef _XHCI_EVENTS_H_
#define _XHCI_EVENTS_H_

#include <exec/lists.h>
#include <exec/types.h>
#include <compat.h>

enum ep_state
{
    USB_DEV_EP_STATE_IDLE = 0,
    USB_DEV_EP_STATE_RECEIVING_CONTROL,
    USB_DEV_EP_STATE_RECEIVING_CONTROL_SHORT,
    USB_DEV_EP_STATE_RECEIVING_BULK,
    USB_DEV_EP_STATE_RECEIVING_INT,
    USB_DEV_EP_STATE_RECEIVING_ISOC,
    USB_DEV_EP_STATE_ABORTING,
    USB_DEV_EP_STATE_RESETTING,
    USB_DEV_EP_STATE_FAILED
};

struct ep_context
{
    struct MinList req_list; /* list of pending requests */

    struct IOUsbHWReq *current_req; /* current request being processed */
    ULONG timeout_stamp;            /* deadline of the request */
    enum ep_state state;

    dma_addr_t trb_addr;
    ULONG trb_length;
    ULONG trb_available_length;
};

struct usb_device; /* forward declaration */
struct xhci_ctrl;  /* forward declaration */
void xhci_ep_set_failed(struct usb_device *udev, int ep_index);
void xhci_ep_set_idle(struct usb_device *udev, int ep_index);
void xhci_ep_set_receiving(struct usb_device *udev, struct IOUsbHWReq *req, enum ep_state state, dma_addr_t trb_addr, ULONG timeout_ms);
void xhci_ep_set_resetting(struct usb_device *udev, int ep_index);
void xhci_ep_set_aborting(struct usb_device *udev, int ep_index);

struct IOUsbHWReq *xhci_ep_get_next_request(struct usb_device *udev, int ep_index);

BOOL xhci_process_event_trb(struct xhci_ctrl *ctrl);
void xhci_process_event_timeouts(struct xhci_ctrl *ctrl);
void xhci_acknowledge_event(struct xhci_ctrl *ctrl);

#endif /* _XHCI_EVENTS_H_ */