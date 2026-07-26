/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The public sliver of the TD tracker: slab lifecycle (controller init) and
 * the streams-recovery bitmap utilities (shared with xhci-commands.c).  The
 * TD-list API itself is private to the endpoint layer — see
 * src/xhci/xhci-td-priv.h.
 */

#ifndef __XHCI_TD_H
#define __XHCI_TD_H

#include <types.h>

struct xhci_ctrl;

void xhci_td_slab_init(struct xhci_ctrl *ctrl);
void xhci_td_slab_destroy(struct xhci_ctrl *ctrl);

/* Streams recovery bitmap: bit id set = stream id targeted by the recovery.
 * Sized for ids 1..num_streams (bit 0 unused, like the rings array). */
#define XHCI_STREAM_MAP_WORDS(num_streams) ((((u32)(num_streams)) + 32) >> 5)

static inline void xhci_stream_map_set(u32 *map, u16 id)
{
    map[id >> 5] |= 1UL << (id & 31);
}

static inline BOOL xhci_stream_map_test(const u32 *map, u16 id)
{
    return (map[id >> 5] & (1UL << (id & 31))) != 0;
}

#endif /* __XHCI_TD_H */
