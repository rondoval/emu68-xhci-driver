#ifndef _XHCI_EVENTS_H_
#define _XHCI_EVENTS_H_

#include <exec/lists.h>
#include <exec/types.h>
#include <compat.h>
#include <exec/semaphores.h>
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

struct xhci_ring; /* forward declaration */

struct xhci_td
{
    struct MinNode node;            /* linkage in active TD list */
    struct IOUsbHWReq *req;         /* owning request */
    dma_addr_t completion_trb;      /* TRB address we expect a completion for */
    ULONG length;                   /* total length for completion accounting */
    ULONG deadline_us;              /* absolute deadline in usec, 0 means no timeout */
    BOOL rt_iso;                    /* true if this TD is part of RT ISO pipeline */
    struct xhci_ring *ring;         /* ring that owns the queued TRBs */
    unsigned int trb_count;         /* number of TRBs consumed by this TD */
    dma_addr_t *trb_addrs;          /* DMA addresses for every TRB in this TD */
};

struct ep_context
{
    struct MinList pending_reqs; /* list of pending requests */
    struct MinList active_tds;   /* list of in-flight TDs */
    struct SignalSemaphore active_tds_lock; /* protects active_tds */

    enum ep_state state;

    /* Pending device-side CLEAR_FEATURE(HALT) request after recovery commands */
    BOOL clear_halt_pending;

    /* RT ISO data */
    /*
     * The idea here is that if this is filled, the event handlers will use
     * hooks to get more data.
     * As such, we will fail an attempt to enter RT ISO mode if there are requests ongoing.
     */
    struct IOUsbHWRTIso * rt_req;
    struct IOUsbHWReq *rt_template_req; /* template for cloning per TD */
    //TODO remove?

    /* Pending STOPRTISO command to reply once the pipe is fully stopped */
    struct IOUsbHWReq *rt_stop_pending;

    /* RT ISO frame tracking (monotonic frame number modulo 2^16) */
    ULONG rt_next_frame;
    /* Cache the last RT ISO buffer so hooks can omit repeating it */
    APTR rt_last_buffer;
    ULONG rt_last_filled;

    /* RT ISO inflight accounting */
    ULONG rt_inflight_bytes;
    ULONG rt_inflight_tds;
};

struct usb_device; /* forward declaration */
struct xhci_ctrl;  /* forward declaration */
void xhci_ep_set_failed(struct usb_device *udev, int endpoint);
void xhci_ep_set_idle(struct usb_device *udev, int endpoint);
void xhci_ep_set_receiving(struct usb_device *udev, struct IOUsbHWReq *req, enum ep_state state, dma_addr_t *trb_addrs, ULONG timeout_ms, struct xhci_ring *ring, unsigned int trb_count);
void xhci_ep_set_resetting(struct usb_device *udev, int endpoint);
void xhci_ep_set_aborting(struct usb_device *udev, int endpoint);

void xhci_td_release_trbs(struct xhci_td *td);
void xhci_td_free(struct xhci_ctrl *ctrl, struct xhci_td *td);

BOOL xhci_process_event_trb(struct xhci_ctrl *ctrl);
void xhci_process_event_timeouts(struct xhci_ctrl *ctrl);
void xhci_acknowledge_event(struct xhci_ctrl *ctrl);

#endif /* _XHCI_EVENTS_H_ */