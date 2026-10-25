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

#include <timing.h>

#include <devices/usbhcd_context.h> /* UhcdSetupData, UHCD_EPTYPE_*, UHCD_XFF_*, USBIsoHooks */
#include <xhci/xhci-dma.h>          /* struct xhci_dma_span */

struct xhci_ctrl;

#pragma pack(2)

/* Transfer direction — the numeric values feed the DCI math in
 * xhci_ep_index_from_parts() (xhci-descriptors.h). */
#define XHCI_DIR_OUT 1
#define XHCI_DIR_IN  2

/* xf->flags — the only two per-transfer behaviours the core reads. */
#define XHCI_XF_TIMEOUT   BIT(0) /* deadline_us is meaningful */
#define XHCI_XF_ALLOWRUNT BIT(1) /* a short IN is not an error */

/* xf->priv_flags — internal state markers, orthogonal to the completion route
 * (the completion route is xf->complete). */
#define REQ_ENQUEUED 0x2        /* linked on ep_ctx->pending_reqs; pre-set by a
                                 * producer it queue-jumps: xhci_ep_enqueue
                                 * AddHeads a request that carries the flag
                                 * (internal recovery EP0 traffic) and AddTails
                                 * everything else */
#define REQ_ON_RING  0x4        /* TRBs live on the hardware ring */
#define REQ_DIRECT   0x10       /* direct-path device transfer: cookie is the
                                 * abort demux key (xhci_ep_abort_cookie) */
#define REQ_CTX_OP   0x40       /* context-ABI lifecycle op: completion replies the op */

struct xhci_xfer
{
    struct MinNode node;        /* FIRST — ep_ctx->pending_reqs linkage; see assert below */
    void (*complete)(struct xhci_xfer *xf); /* the single retire path */
    struct xhci_ctrl *ctrl;
    struct ExecBase *sysBase;   /* ctrl->sysBase, set with ctrl */

    /* what to transfer */
    u8   type;                  /* UHCD_EPTYPE_CONTROL/ISO/BULK/INTERRUPT */
    u8   direction;             /* XHCI_DIR_IN / XHCI_DIR_OUT */
    u8   endpoint;              /* endpoint number 0..15 */
    u8   flags;                 /* XHCI_XF_* */
    u16  stream_id;             /* SS bulk stream ring; 0 = default ring */
    u32  deadline_us;           /* NAK timeout, as a get_time() deadline (valid when XHCI_XF_TIMEOUT) */
    APTR data;                  /* transfer buffer */
    u32  data_length;
    struct UhcdSetupData setup; /* control setup packet */

    /* OUT */
    s8   error;                 /* UHIOERR_/ERR_ result */
    u32  actual;                /* bytes transferred */

    /* driver bookkeeping */
    u32  priv_flags;            /* REQ_* */
    struct xhci_dma_span data_span; /* data's DMA mapping; XHCI_DMA_NONE = unmapped */
    APTR cookie;                /* direct-path demux key (the stack's pipe) */
    u8   owner_slot;            /* internal EP0 requests: owning slot id (device key) */
    u8   about_slot;            /* internal EP0 requests: the endpoint the request is */
    u8   about_ep;              /* about, told when it completes; slot 0 = none */
};

/* node must sit at offset 0: the pending-list links the xfer by &xf->node, and
 * the op wrapper (xhci_ctx_shadow) embeds a struct xhci_xfer first so the
 * completion callback can downcast an xfer pointer back to the wrapper. */
typedef char xhci_xfer_node_first_assert[offsetof(struct xhci_xfer, node) == 0 ? 1 : -1];

/* The single retire path: every producer sets xf->complete when it builds the
 * xfer (direct-path done hook, context-shadow copy-back, or internal free), so
 * completion needs no central switch on request kind. */
static inline void xhci_xfer_reply(struct xhci_xfer *xf) { xf->complete(xf); }

/**
 * xhci_xfer_set_timeout - start a transfer's NAK timeout
 * @xf: the transfer, as the driver accepts it
 * @timeout_ms: the timeout; 0 = none
 *
 * The timeout counts from acceptance, wherever the transfer waits from then
 * on: on a ring, or on the endpoint's pending queue behind a full ring, a
 * recovery or a port suspend.
 */
static inline void xhci_xfer_set_timeout(struct xhci_xfer *xf, u32 timeout_ms)
{
    if (!timeout_ms)
        return;
    xf->deadline_us = get_time() + timeout_ms * 1000UL;
    xf->flags |= XHCI_XF_TIMEOUT;
}

/**
 * xhci_xfer_expired - has a transfer's NAK timeout run out
 * @xf: the transfer
 * @now: get_time() of the caller's scan
 *
 * Return: TRUE if the transfer has a timeout and it has passed.
 */
static inline BOOL xhci_xfer_expired(const struct xhci_xfer *xf, u32 now)
{
    return (xf->flags & XHCI_XF_TIMEOUT) && time_deadline_passed(now, xf->deadline_us);
}

/* The classic 12-byte iso buffer block handed to the clock-driven iso hooks
 * (== Neptune's struct IOUsbHWBufferReq).  The context ABI deliberately leaves
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
