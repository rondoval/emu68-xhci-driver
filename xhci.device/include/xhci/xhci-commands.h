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
    u32 ep_index;             /* endpoint index encoded into the command */
    command_handler complete;
    struct IOUsbHWReq *req; /* to continue control transfers */
#ifdef DEBUG
    trb_type type;          /* command type for diagnostics */
#endif
};

void xhci_dispatch_command_event(struct xhci_ctrl *ctrl, union xhci_trb *event);

static inline u32 xhci_ep_index_from_parts(UBYTE iouh_endpoint, UBYTE iouh_dir)
{
    UBYTE ep = iouh_endpoint & 0x0F;
    BOOL dir_in = (iouh_dir == UHDIR_IN);
    if (ep == 0)
        return 0;
    return (ep << 1) - (dir_in ? 0 : 1);
}

void xhci_reset_ep(struct usb_device *udev, u32 ep_index);
void xhci_stop_endpoint(struct usb_device *udev, u32 ep_index);
void xhci_configure_endpoints(struct usb_device *udev, BOOL ctx_change, struct IOUsbHWReq *req);
void xhci_address_device(struct usb_device *udev, struct IOUsbHWReq *req);
void xhci_reset_device(struct usb_device *udev);
void xhci_set_deq_pointer(struct usb_device *udev, u32 ep_index);
void xhci_disable_slot(struct usb_device *udev);
void xhci_enable_slot(struct usb_device *udev, struct IOUsbHWReq *req);

#endif /* XHCI_COMMANDS_H */