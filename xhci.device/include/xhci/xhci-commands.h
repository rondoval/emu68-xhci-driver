#ifndef XHCI_COMMANDS_H
#define XHCI_COMMANDS_H

#include <exec/lists.h>
#include <compat.h>
#include <xhci/xhci.h>

struct pending_command; /* forward declaration */
typedef void (*command_handler)(struct xhci_ctrl *ctrl, struct pending_command *cmd, union xhci_trb *event);

struct pending_command
{
    struct MinNode node;
    dma_addr_t cmd_trb_dma;  /* value from queue_trb */
    struct usb_device *udev; /* for slot/endpoint checks */
    command_handler complete;
    struct IOUsbHWReq *req; /* to continue control transfers */
};

void xhci_dispatch_command_event(struct xhci_ctrl *ctrl, union xhci_trb *event);

void xhci_reset_ep(struct usb_device *udev, int ep_index);
void xhci_abort_td(struct usb_device *udev, unsigned int ep_index);
void xhci_configure_endpoints(struct usb_device *udev, BOOL ctx_change, struct IOUsbHWReq *req);
void xhci_address_device(struct usb_device *udev, struct IOUsbHWReq *req);

#endif /* XHCI_COMMANDS_H */