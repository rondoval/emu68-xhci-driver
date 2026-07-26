/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef __XHCI_XFER_H__
#define __XHCI_XFER_H__

/*
 * The driver-private transfer work-item.
 *
 * xhci.device is a context-ABI HCD; on the wire it speaks IOStdReq-shaped
 * lifecycle ops (usbhcd_context.h) while transfers arrive as direct calls
 * (xhci-direct.c).  Internally the ring/TD/event/endpoint machinery operates
 * on this lean descriptor, which every producer (direct path, context-op
 * shadow, internal EP0) fills and feeds to the core.
 *
 * Completion is carried as data: xf->complete(xf) is the single retire path, so
 * there is no central reply funnel switching on request kind.
 */

#include <types.h>
#include <bits.h>
#include <stddef.h>
#include <exec/io.h>
#include <exec/nodes.h>

#include <devices/usbhcd_context.h> /* UhcdSetupData, UHCD_EPTYPE_*, UHCD_XFF_*, USBIsoHooks */

struct xhci_ctrl;

#pragma pack(2)

/* Transfer direction — the numeric values feed the DCI math in
 * xhci_ep_index_from_parts() (xhci-descriptors.h). */
#define XHCI_DIR_OUT 1
#define XHCI_DIR_IN  2

/* xf->flags — the only two per-transfer behaviours the core reads. */
#define XHCI_XF_TIMEOUT   BIT(0) /* naktimeout is meaningful */
#define XHCI_XF_ALLOWRUNT BIT(1) /* a short IN is not an error */

/* xf->priv_flags — internal state markers, orthogonal to the completion route
 * (the completion route is xf->complete). */
#define REQ_ENQUEUED 0x2        /* linked on ep_ctx->pending_reqs; pre-set by a
                                 * producer it queue-jumps: xhci_ep_enqueue
                                 * AddHeads a request that carries the flag
                                 * (internal recovery EP0 traffic) and AddTails
                                 * everything else */
#define REQ_ON_RING  0x4        /* TRBs live on the hardware ring */
#define REQ_DMA_MAPPED 0x8      /* data buffer is DMA-mapped (dma_address valid) */
#define REQ_DIRECT   0x10       /* direct-path device transfer: cookie is the
                                 * abort demux key (xhci_ep_abort_cookie) */
#define REQ_DMA_DIRECT 0x20     /* mapping is the caller's buffer itself (no
                                 * bounce); dma_address == data */
#define REQ_CTX_OP   0x40       /* context-ABI lifecycle op: completion replies the op */

/* bounce class — which bounce slab the bounce buffer came from (0 = dma_alloc fallback) */
#define REQ_BOUNCE_CLASS_SHIFT 8
#define REQ_BOUNCE_CLASS_MASK  (0x7U << REQ_BOUNCE_CLASS_SHIFT)
#define REQ_BOUNCE_CLASS_NONE  0
#define REQ_BOUNCE_CLASS_SMALL 1
#define REQ_BOUNCE_CLASS_MED   2
#define REQ_BOUNCE_CLASS_LARGE 3

struct xhci_xfer
{
    struct MinNode node;        /* FIRST — ep_ctx->pending_reqs linkage; see assert below */
    void (*complete)(struct xhci_xfer *xf); /* the single retire path */
    struct xhci_ctrl *ctrl;

    /* what to transfer */
    u8   type;                  /* UHCD_EPTYPE_CONTROL/ISO/BULK/INTERRUPT */
    u8   direction;             /* XHCI_DIR_IN / XHCI_DIR_OUT */
    u8   endpoint;              /* endpoint number 0..15 */
    u8   flags;                 /* XHCI_XF_* */
    u16  stream_id;             /* SS bulk stream ring; 0 = default ring */
    u32  timeout_ms;            /* NAK timeout (valid when XHCI_XF_TIMEOUT) */
    APTR data;                  /* transfer buffer */
    u32  data_length;
    struct UhcdSetupData setup; /* control setup packet */

    /* OUT */
    s8   error;                 /* UHIOERR_/ERR_ result */
    u32  actual;                /* bytes transferred */

    /* driver bookkeeping */
    u32  priv_flags;            /* REQ_* */
    void *dma_address;          /* bounce pointer (valid when REQ_DMA_MAPPED) */
    APTR cookie;                /* direct-path demux key (the stack's pipe) */
    u8   owner_slot;            /* internal EP0 requests: owning slot id (device key) */
};

/* node must sit at offset 0: the pending-list links the xfer by &xf->node, and
 * the op wrapper (xhci_ctx_shadow) embeds a struct xhci_xfer first so the
 * completion callback can downcast an xfer pointer back to the wrapper. */
typedef char xhci_xfer_node_first_assert[offsetof(struct xhci_xfer, node) == 0 ? 1 : -1];

/* The single retire path: every producer sets xf->complete when it builds the
 * xfer (direct-path done hook, context-shadow copy-back, or internal free), so
 * completion needs no central switch on request kind. */
static inline void xhci_xfer_reply(struct xhci_xfer *xf) { xf->complete(xf); }

/* The classic 12-byte iso buffer block handed to the clock-driven iso hooks
 * (== Poseidon's struct IOUsbHWBufferReq).  The context ABI deliberately leaves
 * this type anonymous (usbhcd_context.h), so each side names it privately; its
 * ubr_Flags bits live in usbhcd_common.h. */
struct USBBufferRequest
{
    u8 *data;
    u32 length;
    u16 frame;
    u16 flags;
};

#pragma pack()

#endif /* __XHCI_XFER_H__ */
