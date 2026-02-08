#ifndef _XHCI_EVENTS_H_
#define _XHCI_EVENTS_H_

#include <exec/types.h>

struct xhci_ctrl;  /* forward declaration */

BOOL xhci_process_event_trb(struct xhci_ctrl *ctrl);
void xhci_process_event_timeouts(struct xhci_ctrl *ctrl);

#endif /* _XHCI_EVENTS_H_ */