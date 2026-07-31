/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The public sliver of the TD tracker: slab lifecycle (controller init).
 * The TD-list API itself is private to the endpoint layer — see
 * src/xhci/xhci-td-priv.h.
 */

#ifndef __XHCI_TD_H
#define __XHCI_TD_H

#include <types.h>

struct xhci_ctrl;

void xhci_td_slab_init(struct xhci_ctrl *ctrl);
void xhci_td_slab_destroy(struct xhci_ctrl *ctrl);

#endif /* __XHCI_TD_H */
