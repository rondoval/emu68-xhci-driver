#ifndef XHCI_DEBUG_H
#define XHCI_DEBUG_H

struct xhci_ep_ctx;
struct xhci_slot_ctx;
struct IOUsbHWReq;

void xhci_dump_ep_ctx(const char *tag,
					  UBYTE ep_index,
					  struct xhci_ep_ctx *ep_ctx);
void xhci_dump_slot_ctx(const char *tag,
						struct xhci_slot_ctx *slot_ctx);

void xhci_dump_config(const char *tag, const struct usb_config *cfg, UBYTE addr);

void xhci_dump_request(const char *tag, const struct IOUsbHWReq *req);

#endif /* XHCI_DEBUG_H */