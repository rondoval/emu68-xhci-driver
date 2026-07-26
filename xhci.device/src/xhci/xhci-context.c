// SPDX-License-Identifier: GPL-2.0-only
/*
 * USB HOST XHCI Controller stack
 *
 * Based on xHCI host controller driver in linux-kernel
 * by Sarah Sharp.
 *
 * Copyright (C) 2008 Intel Corp.
 * Author: Sarah Sharp
 *
 * Copyright (C) 2013 Samsung Electronics Co.Ltd
 * Authors: Vivek Gautam <gautam.vivek@samsung.com>
 *	    Vikas Sajjan <vikas.sajjan@samsung.com>
 */

#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME (*(struct ExecBase **)4UL)
#include <proto/exec.h>
#endif

#include <debug.h>

#include <iomem.h>
#include <memory.h>

#include <xhci/xhci.h>
#include <xhci/ch9.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-ring.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci-context.h>
#include <xhci/xhci-descriptors.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-context] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef TRACE
#undef KprintfT
#define KprintfT(fmt, ...) PrintPistorm("[xhci-context] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

/**
 * Allocates the Container context
 *
 * @param ctrl	Host controller data structure
 * @param type type of XHCI Container Context
 * Return: NULL if failed else pointer to the context on success
 */
struct xhci_container_ctx *xhci_alloc_container_ctx(struct xhci_ctrl *ctrl, u32 type)
{
    struct xhci_container_ctx *ctx = pool_zalloc(ctrl->metaPool, sizeof(struct xhci_container_ctx));
    if (!ctx)
    {
        Kprintf("Failed to allocate container context\n");
        return NULL;
    }

    if ((type != XHCI_CTX_TYPE_DEVICE) && (type != XHCI_CTX_TYPE_INPUT))
    {
        Kprintf("Invalid context type\n");
        pool_free(ctrl->metaPool, ctx);
        return NULL;
    }

    ctx->type = type;
    ctx->size = (USB_MAX_ENDPOINT_CONTEXTS + 1) * (u32)ctrl->ctx_size;
    if (type == XHCI_CTX_TYPE_INPUT)
        ctx->size += ctrl->ctx_size;

    ctx->bytes = xhci_malloc_page_bounded(ctrl, ctx->size, XHCI_ALIGNMENT);
    return ctx;
}

/**
 * frees the "xhci_container_ctx" pointer passed
 *
 * @param ptr	pointer to "xhci_container_ctx" to be freed
 * Return: none
 */
void xhci_free_container_ctx(struct xhci_ctrl *ctrl, struct xhci_container_ctx *ctx)
{
    dma_free(ctrl->dmaPool, ctx->bytes);
    pool_free(ctrl->metaPool, ctx);
}

/**
 * Give the input control context for the passed container context
 *
 * @param ctx	pointer to the context
 * Return: pointer to the Input control context data
 */
static struct xhci_input_control_ctx *xhci_get_input_control_ctx(struct xhci_container_ctx *ctx)
{
    if (ctx->type != XHCI_CTX_TYPE_INPUT)
    {
        Kprintf("Invalid context type\n");
        return NULL;
    }
    return (struct xhci_input_control_ctx *)ctx->bytes;
}

/**
 * Give the slot context for the passed container context
 *
 * @param ctrl	Host controller data structure
 * @param ctx	pointer to the context
 * Return: pointer to the slot control context data
 */
static struct xhci_slot_ctx *xhci_get_slot_ctx(struct xhci_ctrl *ctrl, struct xhci_container_ctx *ctx)
{
    if (ctx->type == XHCI_CTX_TYPE_DEVICE)
        return (struct xhci_slot_ctx *)ctx->bytes;

    return (struct xhci_slot_ctx *)(ctx->bytes + ctrl->ctx_size);
}

/**
 * Gets the EP context from based on the ep_index
 *
 * @param ctrl	Host controller data structure
 * @param ctx	context container
 * @param ep_index	index of the endpoint
 * Return: pointer to the End point context
 */
static struct xhci_ep_ctx *xhci_get_ep_ctx(struct xhci_ctrl *ctrl, struct xhci_container_ctx *ctx, u8 ep_index)
{
    /* increment ep index by offset of start of ep ctx array */
    ep_index++;
    if (ctx->type == XHCI_CTX_TYPE_INPUT)
        ep_index++;

    return (struct xhci_ep_ctx *)(ctx->bytes + (ep_index * ctrl->ctx_size));
}

u32 xhci_get_hardware_address(struct usb_device *udev)
{
    struct xhci_slot_ctx *slot_ctx = xhci_get_slot_ctx(udev->controller, udev->out_ctx);
    cache_post_dma(slot_ctx, sizeof(struct xhci_slot_ctx), 0);
    return le32(slot_ctx->dev_state) & DEV_ADDR_MASK;
}

u64 xhci_get_endpoint_deq_ptr(struct usb_device *udev, u8 ep_index)
{
    struct xhci_ep_ctx *ep_ctx = xhci_get_ep_ctx(udev->controller, udev->out_ctx, ep_index);
    cache_post_dma(ep_ctx, sizeof(struct xhci_ep_ctx), 0);
    return le64(ep_ctx->deq);
}

/**
 * Copy output xhci_ep_ctx to the input xhci_ep_ctx copy.
 * Useful when you want to change one particular aspect of the endpoint
 * and then issue a configure endpoint command.
 *
 * @param ctrl	Host controller data structure
 * @param in_ctx contains the input context
 * @param out_ctx contains the input context
 * @param ep_index index of the end point
 * Return: none
 */
static void xhci_endpoint_copy(struct xhci_ctrl *ctrl,
                               struct xhci_container_ctx *in_ctx,
                               struct xhci_container_ctx *out_ctx,
                               u8 ep_index)
{
    struct xhci_ep_ctx *out_ep_ctx = xhci_get_ep_ctx(ctrl, out_ctx, ep_index);
    struct xhci_ep_ctx *in_ep_ctx = xhci_get_ep_ctx(ctrl, in_ctx, ep_index);

    in_ep_ctx->ep_info = out_ep_ctx->ep_info;
    in_ep_ctx->ep_info2 = out_ep_ctx->ep_info2;
    in_ep_ctx->deq = out_ep_ctx->deq;
    in_ep_ctx->tx_info = out_ep_ctx->tx_info;
}

/**
 * Copy output xhci_slot_ctx to the input xhci_slot_ctx.
 * Useful when you want to change one particular aspect of the endpoint
 * and then issue a configure endpoint command.
 * Only the context entries field matters, but
 * we'll copy the whole thing anyway.
 *
 * @param ctrl	Host controller data structure
 * @param in_ctx contains the inpout context
 * @param out_ctx contains the inpout context
 * Return: none
 */
static void xhci_slot_copy(struct xhci_ctrl *ctrl, struct xhci_container_ctx *in_ctx,
                           struct xhci_container_ctx *out_ctx)
{
    struct xhci_slot_ctx *in_slot_ctx = xhci_get_slot_ctx(ctrl, in_ctx);
    struct xhci_slot_ctx *out_slot_ctx = xhci_get_slot_ctx(ctrl, out_ctx);

    in_slot_ctx->dev_info = out_slot_ctx->dev_info;
    in_slot_ctx->dev_info2 = out_slot_ctx->dev_info2;
    in_slot_ctx->tt_info = out_slot_ctx->tt_info;
    in_slot_ctx->dev_state = out_slot_ctx->dev_state;
}

static void build_route_string(struct usb_device *udev)
{
    if (!udev)
        return;

    /* No routing for the root hub or a device on a root port (parent ==
     * NULL).  A parent with no parent is a first-tier hub whose port nibble
     * must still be appended. */
    struct usb_device *parent = udev->parent;
    if (!parent)
    {
        udev->route = 0;
        udev->route_depth = 0;
        return;
    }

    u8 nibble = (udev->parent_port > 15) ? 0xf : udev->parent_port;
    u32 route_depth = parent->route_depth + 1U;
    udev->route_depth = (u8)((route_depth <= 0xffU) ? route_depth : 0xffU);
    if (parent->route_depth > 5)
    {
        Kprintf("Route depth %lu exceeds xHCI max of 5, not adding to route string\n", (ULONG)parent->route_depth);
        udev->route = parent->route;
        return;
    }

    /*
     * xHCI route string packs hub port numbers in 4-bit nibbles where
     * the first hub tier below root occupies bits [3:0], second tier
     * occupies [7:4], etc.
     *
     * route_depth tracks tier count (1 for first tier), so the nibble
     * shift is based on parent depth.
     */
    u32 route_shift = (u32)parent->route_depth << 2;
    udev->route = parent->route | ((u32)nibble << route_shift);
}

u32 xhci_find_root_port(struct usb_device *udev)
{
    u32 root_port = udev->parent_port;
    if (udev->parent)
    {
        struct usb_device *ancestor = udev->parent;

        while (ancestor)
        {
            if (ancestor->parent_port)
                root_port = ancestor->parent_port;

            if (!ancestor->parent)
                break;

            ancestor = ancestor->parent;
        }
    }
    return root_port;
}

/* TRUE if hub is a HS multi-TT hub with its multi-TT interface selected:
 * bDeviceProtocol == 2 (capability) and the active altsetting of the hub
 * interface has bInterfaceProtocol == 2. */
static BOOL xhci_hub_multi_tt_enabled(struct usb_device *hub)
{
    /* The stack supplies the enabled MTT state explicitly
       (NSCMD_USB_UPDATE_HUB): 2 = multi-TT interface selected. */
    return hub->is_hub && hub->speed == USB_SPEED_HIGH && hub->ctx_mtt == 2;
}

/* Default EP0 max packet size per speed: SS(P) = 512, HS = 64, FS = 64 as a
 * first guess (corrected later by NSCMD_USB_UPDATE_EP0), LS = 8.
 * 0 = unknown speed. */
u16 xhci_ep0_default_mps(enum usb_device_speed speed)
{
    switch (speed)
    {
    case USB_SPEED_SUPER:
    case USB_SPEED_SUPER_PLUS:
        return 512;
    case USB_SPEED_HIGH:
    case USB_SPEED_FULL:
        return 64;
    case USB_SPEED_LOW:
        return 8;
    default:
        return 0;
    }
}

/* Low-level endpoint wiring shared by EP0 setup and the descriptor-driven
 * path: allocate the software endpoint context (+ transfer ring), then fill
 * the hardware input endpoint context.  ep_info carries the host-order
 * interval/mult/ESIT-hi bits (0 for control endpoints), tx_info the
 * ESIT-lo/average-TRB-length bits. */
static s8 xhci_wire_ep_ctx(struct usb_device *udev, u8 ep_index, u8 ep_type,
                           u16 max_packet_size, u8 max_burst, u8 err_count,
                           u32 ep_info, u32 tx_info)
{
    struct xhci_ctrl *ctrl = udev->controller;
    struct xhci_ep_ctx *epc = xhci_get_ep_ctx(ctrl, udev->in_ctx, ep_index);

    if (!xhci_ep_create_context(udev, ep_index, max_packet_size, max_burst))
        return UHIOERR_OUTOFMEMORY;
    xhci_ep_set_hw_type(xhci_ep_get_context_for_index(udev, ep_index), ep_type);

    epc->ep_info = le32(ep_info);
    epc->ep_info2 = le32(EP_TYPE(ep_type) | MAX_PACKET((u32)max_packet_size) |
                         MAX_BURST(max_burst) | ERROR_COUNT(err_count));

    struct ep_context *ep_context = xhci_ep_get_context_for_index(udev, ep_index);
    epc->deq = le64(xhci_ring_get_new_dequeue_ptr(xhci_ep_get_ring(ep_context)));
    epc->tx_info = le32(tx_info);

    return UHIOERR_NO_ERROR;
}

/* Route string, device speed and the initial Context Entries value (EP0 only)
 * into the input slot context's dev_info. */
static void ctx_pack_slot_speed_route(struct usb_device *udev, struct xhci_slot_ctx *slot_ctx)
{
    u32 dev_info = le32(slot_ctx->dev_info);
    dev_info &= ~(ROUTE_STRING_MASK | DEV_SPEED | DEV_MTT | LAST_CTX_MASK);
    dev_info |= (udev->route & ROUTE_STRING_MASK);
    /* Only the control endpoint is valid - one endpoint context */
    dev_info |= LAST_CTX(1);

    switch (udev->speed)
    {
    case USB_SPEED_SUPER:
    case USB_SPEED_SUPER_PLUS:
        dev_info |= SLOT_SPEED_SS;
        break;
    case USB_SPEED_HIGH:
        dev_info |= SLOT_SPEED_HS;
        break;
    case USB_SPEED_FULL:
        dev_info |= SLOT_SPEED_FS;
        break;
    case USB_SPEED_LOW:
        dev_info |= SLOT_SPEED_LS;
        break;
    default:
        /* Speed was set earlier, this shouldn't happen. */
        Kprintf("Unknown device speed %lu\n", (ULONG)udev->speed);
    }

    slot_ctx->dev_info = le32(dev_info);
}

/* Root-hub port into dev_info2; for LS/FS devices behind a high-speed hub,
 * the TT fields (and the DEV_MTT mirror of the hub's enabled multi-TT mode,
 * xHCI 6.2.2 — safe at address time: the hub class selects the TT mode
 * before powering ports). */
static void ctx_pack_tt_info(struct usb_device *udev, struct xhci_slot_ctx *slot_ctx)
{
    u32 root_port = xhci_find_root_port(udev);

    KprintfT("parent_slot=%lu port_num=%lu root_port_num=%lu speed=%lu route=%lx route_depth=%lu\n",
             (ULONG)(udev->parent ? udev->parent->slot_id : 0), /* devices on root ports have no parent udev */
             (ULONG)udev->parent_port, (ULONG)root_port, (ULONG)udev->speed, (ULONG)udev->route, (ULONG)udev->route_depth);

    u32 dev_info2 = le32(slot_ctx->dev_info2);
    dev_info2 &= ~(ROOT_HUB_PORT_MASK << ROOT_HUB_PORT_SHIFT);
    dev_info2 |= ROOT_HUB_PORT(root_port);
    slot_ctx->dev_info2 = le32(dev_info2);

    u32 tt_info = 0;
    if (udev->speed == USB_SPEED_LOW || udev->speed == USB_SPEED_FULL)
    {
        struct usb_device *tt_hub = udev->parent;
        u8 parent_port = udev->parent_port;

        while (tt_hub)
        {
            if (tt_hub->is_hub && tt_hub->speed >= USB_SPEED_HIGH)
            {
                tt_info = TT_SLOT(tt_hub->slot_id) | TT_PORT(parent_port);

                BOOL mtt = xhci_hub_multi_tt_enabled(tt_hub);
                if (mtt)
                    slot_ctx->dev_info |= le32(DEV_MTT);

                KprintfT("tt_slot=%lu tt_port=%lu tt_info=%08lx mtt=%ld\n",
                         (ULONG)tt_hub->slot_id, (ULONG)parent_port, (ULONG)tt_info,
                         (LONG)mtt);
                break;
            }
            parent_port = tt_hub->parent_port;
            tt_hub = tt_hub->parent;
        }

        if (!tt_hub)
            Kprintf("Low or full speed device slot %lu not behind a high-speed hub???\n", (ULONG)udev->slot_id);
    }

    /* SS/SSP devices carry no TT facts: the slot-context TT fields are defined
     * only for LS/FS devices behind a high-speed hub (xHCI 6.2.2). */
    slot_ctx->tt_info = le32(tt_info);
}

/* EP0 is a control endpoint (burst 0, CErr 3, Average TRB Length 8 per xHCI
 * 6.2.3) at the per-speed default max packet size. */
static void ctx_wire_ep0(struct usb_device *udev)
{
    u16 max_packet_size = xhci_ep0_default_mps(udev->speed);
    if (max_packet_size == 0)
        Kprintf("Unknown device speed %lu\n", (ULONG)udev->speed);
    KprintfT("SPEED=%lu MPS=%lu\n", (ULONG)udev->speed, (ULONG)max_packet_size);

    if (xhci_wire_ep_ctx(udev, 0, CTRL_EP, max_packet_size, /*max_burst*/ 0,
                         /*err_count*/ 3, /*ep_info*/ 0,
                         EP_AVG_TRB_LENGTH(8)) != UHIOERR_NO_ERROR)
        Kprintf("Failed to create EP0 context\n");
}

/**
 * Setup an xHCI virtual device for a Set Address command
 *
 * @param udev pointer to the Device Data Structure
 * Return: returns negative value on failure else 0 on success
 */
void xhci_setup_addressable_virt_dev(struct usb_device *udev)
{
    struct xhci_ctrl *ctrl = udev->controller;
    KprintfT("Setting up addressable virtual device slot=%lu parent_slot=%lu parent_port=%lu\n",
             (ULONG)udev->slot_id,
             (ULONG)(udev->parent ? udev->parent->slot_id : 0),
             (ULONG)udev->parent_port);
    build_route_string(udev);

    /* Extract the EP0 and Slot Ctrl */
    struct xhci_ep_ctx *ep0_ctx = xhci_get_ep_ctx(ctrl, udev->in_ctx, 0);
    struct xhci_slot_ctx *slot_ctx = xhci_get_slot_ctx(ctrl, udev->in_ctx);
    KprintfT("slot=%lu in_ctx=%lx out_ctx=%lx ep0_ctx=%lx slot_ctx=%lx\n",
             (ULONG)udev->slot_id, (ULONG)udev->in_ctx, (ULONG)udev->out_ctx,
             (ULONG)ep0_ctx, (ULONG)slot_ctx);

    ctx_pack_slot_speed_route(udev, slot_ctx);
    ctx_pack_tt_info(udev, slot_ctx);

    /* Step 4 - the EP0 ring is already allocated; step 5 wires its context.
     * Steps 7 and 8 were done in xhci_alloc_virt_device(). */
    ctx_wire_ep0(udev);

    cache_pre_dma(ep0_ctx, sizeof(struct xhci_ep_ctx), DMA_ReadFromRAM);
    cache_pre_dma(slot_ctx, sizeof(struct xhci_slot_ctx), DMA_ReadFromRAM);
    KprintfT("xhci_setup_addressable_virt_dev: ep0 deq=%lx tx_info=%08lx dev_info=%08lx dev_info2=%08lx\n",
             (ULONG)le64(ep0_ctx->deq), (ULONG)le32(ep0_ctx->tx_info),
             (ULONG)le32(slot_ctx->dev_info), (ULONG)le32(slot_ctx->dev_info2));

    struct xhci_input_control_ctx *ctrl_ctx = xhci_get_input_control_ctx(udev->in_ctx);
    ctrl_ctx->add_flags = le32(SLOT_FLAG | EP0_FLAG);
    ctrl_ctx->drop_flags = 0;

    cache_pre_dma(ctrl_ctx, sizeof(struct xhci_input_control_ctx), DMA_ReadFromRAM);
}

static void xhci_update_hub_tt(struct usb_device *udev, struct xhci_container_ctx *in_ctx)
{
    if (!udev || !udev->controller)
        return;

    if (udev->slot_id == 0)
        return;

    struct xhci_ctrl *ctrl = udev->controller;

    struct xhci_slot_ctx *slot_ctx = xhci_get_slot_ctx(ctrl, in_ctx);
    if (!slot_ctx)
        return;

    u32 dev_info = le32(slot_ctx->dev_info);
    u32 dev_info2 = le32(slot_ctx->dev_info2);
    u32 tt_info = le32(slot_ctx->tt_info);

    if (udev->is_hub)
    {
        dev_info |= DEV_HUB;
        dev_info2 &= ~(0xff << 24);
        dev_info2 |= XHCI_MAX_PORTS(udev->hub_num_ports);

        if (udev->speed == USB_SPEED_HIGH)
        {
            /* For high-speed hubs, TT think time is encoded in the hub descriptor */
            tt_info &= ~TT_THINK_TIME(0x03);
            tt_info |= TT_THINK_TIME(udev->tt_think_time);

            /* xHCI 6.2.2: MTT reflects the *enabled* multi-TT mode */
            if (xhci_hub_multi_tt_enabled(udev))
                dev_info |= DEV_MTT;
            else
                dev_info &= ~(u32)DEV_MTT;
        }
    }

    slot_ctx->dev_info = le32(dev_info);
    slot_ctx->dev_info2 = le32(dev_info2);
    slot_ctx->tt_info = le32(tt_info);
}

/* Refresh the input context from the hardware output context: invalidate the
 * output copy, carry the slot context over, optionally carry one endpoint
 * context (copy_ep >= 0), and re-apply the stack-supplied hub facts.
 * xhci_evaluate_mel deliberately does NOT use this: Evaluate Context must not
 * carry endpoint/hub state and zeroes dev_state itself. */
static void xhci_refresh_input_from_output(struct usb_device *udev, s16 copy_ep)
{
    struct xhci_ctrl *ctrl = udev->controller;

    cache_post_dma(udev->out_ctx->bytes, udev->out_ctx->size, 0);
    xhci_slot_copy(ctrl, udev->in_ctx, udev->out_ctx);
    if (copy_ep >= 0)
        xhci_endpoint_copy(ctrl, udev->in_ctx, udev->out_ctx, (u8)copy_ep);
    xhci_update_hub_tt(udev, udev->in_ctx);
}

/*
 * Full speed devices may have a max packet size greater than 8 bytes, but the
 * USB core doesn't know that until it reads the first 8 bytes of the
 * descriptor.  If the usb_device's max packet size changes after that point,
 * we need to issue an evaluate context command and wait on it.
 *
 * @param udev	pointer to the Device Data Structure
 * @param req	optional request to reply when the Evaluate Context completes
 * Return: TRUE when an Evaluate Context was issued (req is owned by the
 *         completion path), FALSE when the hardware value already matched.
 */
BOOL xhci_update_maxpacket(struct usb_device *udev, u16 max_packet_size, struct xhci_xfer *req)
{
    struct xhci_ctrl *ctrl = udev->controller;
    u8 ep_index = 0; /* control endpoint */

    cache_post_dma(udev->out_ctx->bytes, udev->out_ctx->size, 0);
    KprintfT("Checking max packet size for ep 0 of slot %lu\n", (ULONG)udev->slot_id);

    struct xhci_ep_ctx *ep_ctx = xhci_get_ep_ctx(ctrl, udev->out_ctx, ep_index);
    u16 hw_max_packet_size = (u16)MAX_PACKET_DECODED(le32(ep_ctx->ep_info2));

    if (hw_max_packet_size == max_packet_size)
    {
        KprintfT("Max Packet Size for ep 0 is already correct at %lu.\n", (ULONG)max_packet_size);
        return FALSE;
    }

    KprintfT("Max Packet Size for ep 0 changed to %lu.\n", (ULONG)max_packet_size);
    KprintfT("Max packet size in xHCI HW = %lu\n", (ULONG)hw_max_packet_size);

    // Update the EP context's max packet size as well
    struct ep_context *ep_context = xhci_ep_get_context_for_index(udev, ep_index);
    xhci_ep_set_max_packet_size(ep_context, max_packet_size);

    /* Set up the modified control endpoint 0 */
    xhci_endpoint_copy(ctrl, udev->in_ctx,
                       udev->out_ctx, ep_index);
    ep_ctx = xhci_get_ep_ctx(ctrl, udev->in_ctx, ep_index);
    ep_ctx->ep_info2 &= le32(~MAX_PACKET(MAX_PACKET_MASK));
    ep_ctx->ep_info2 |= le32(MAX_PACKET(max_packet_size));

    /*
     * Set up the input context flags for the command
     * FIXME: This won't work if a non-default control endpoint
     * changes max packet sizes.
     */
    struct xhci_input_control_ctx *ctrl_ctx = xhci_get_input_control_ctx(udev->in_ctx);
    ctrl_ctx->add_flags = le32(EP0_FLAG);
    ctrl_ctx->drop_flags = 0;

    xhci_configure_endpoints(udev, TRUE, req);
    return TRUE;
}

/* ---- Endpoint-descriptor -> xHCI context field derivation ----------------
 * Pure per-speed math over the stack-supplied UhcdEndpointDesc, consumed only
 * by xhci_init_one_ep_context below. */

static u8 xhci_microframes_to_exponent(u32 desc_interval,
                                       u32 min_exponent,
                                       u32 max_exponent)
{
    u32 interval = log2_floor_u64(desc_interval);
    interval = clamp_val(interval, min_exponent, max_exponent);
#ifdef TRACE
    if ((1U << interval) != desc_interval)
        KprintfT("rounding interval to %lu microframes, ep desc says %lu microframes\n",
                 (ULONG)(1U << interval), (ULONG)desc_interval);
#endif

    return (u8)interval;
}

static u8 xhci_parse_microframe_interval(const struct UhcdEndpointDesc *ed)
{
    if (ed->ed_Interval == 0)
        return 0;

    return xhci_microframes_to_exponent(ed->ed_Interval, 0, 15);
}

static u8 xhci_parse_frame_interval(const struct UhcdEndpointDesc *ed)
{
    return xhci_microframes_to_exponent((u32)ed->ed_Interval * 8U, 3, 10);
}

/*
 * Convert interval expressed as 2^(bInterval - 1) == interval into
 * straight exponent value 2^n == interval.
 */
static u8 xhci_parse_exponent_interval(struct usb_device *udev,
                                       const struct UhcdEndpointDesc *ed)
{
    u8 interval;

    interval = (u8)(clamp_val(ed->ed_Interval, 1, 16) - 1);
    if (interval != ed->ed_Interval - 1U)
        Kprintf("ep %#lx - rounding interval to %lu %sframes\n",
                (ULONG)ed->ed_Address, (ULONG)(1U << interval),
                udev->speed == USB_SPEED_FULL ? "" : "micro");

    if (udev->speed == USB_SPEED_FULL)
    {
        /*
         * Full speed isoc endpoints specify interval in frames,
         * not microframes. We are using microframes everywhere,
         * so adjust accordingly.
         */
        interval = (u8)(interval + 3u); /* 1 frame = 2^3 uframes */
    }

    return interval;
}

/*
 * Return the polling or NAK interval.
 *
 * The polling interval is expressed in "microframes". If xHCI's Interval field
 * is set to N, it will service the endpoint every 2^(Interval)*125us.
 *
 * The NAK interval is one NAK per 1 to 255 microframes, or no NAKs if interval
 * is set to 0.
 *
 * We re-encode from the raw bInterval (ed_Interval) per xHCI's per-speed rules
 * rather than reuse the stack's cooked interval: that cooked value is produced
 * for the legacy HCD API/GUI and is lossy for us (0 for control/bulk, a linear
 * 1<<(bInterval-1) in speed-dependent units, with the stack's own clamps, not
 * xHCI's exponent clamps).
 */
static u8 xhci_get_endpoint_interval(struct usb_device *udev, const struct UhcdEndpointDesc *ed)
{
    u8 interval = 0;

    switch (udev->speed)
    {
    case USB_SPEED_HIGH:
        /* Max NAK rate */
        if (ed_is_control(ed) || ed_is_bulk(ed))
        {
            interval = xhci_parse_microframe_interval(ed);
            break;
        }
        /* Fall through - SS and HS isoc/int have same decoding */
        __attribute__((fallthrough));

    case USB_SPEED_SUPER:
    case USB_SPEED_SUPER_PLUS:
        if (ed_is_int(ed) || ed_is_isoc(ed))
        {
            interval = xhci_parse_exponent_interval(udev, ed);
        }
        break;

    case USB_SPEED_FULL:
        if (ed_is_isoc(ed))
        {
            interval = xhci_parse_exponent_interval(udev, ed);
            break;
        }
        /*
         * Fall through for interrupt endpoint interval decoding
         * since it uses the same rules as low speed interrupt
         * endpoints.
         */
        __attribute__((fallthrough));
    case USB_SPEED_LOW:
        if (ed_is_int(ed) || ed_is_isoc(ed))
        {
            interval = xhci_parse_frame_interval(ed);
        }
        break;

    default:
        Kprintf("Unsupported USB speed: %lu\n", (ULONG)udev->speed);
    }

    return interval;
}

/*
 * The "Mult" field in the endpoint context is only set for SuperSpeed isoc eps.
 * High speed endpoint descriptors can define "the number of additional
 * transaction opportunities per microframe", but that goes in the Max Burst
 * endpoint context field.
 */
static u8 xhci_get_endpoint_mult(struct usb_device *udev, const struct UhcdEndpointDesc *ed)
{
    if (udev->speed < USB_SPEED_SUPER || !ed_is_isoc(ed))
        return 0;

    /* SS isoch Mult (companion bmAttributes bits 1:0) */
    return ed->ed_Mult & 0x3;
}

static u8 xhci_get_endpoint_max_burst(struct usb_device *udev, const struct UhcdEndpointDesc *ed)
{
    /* Super speed and Plus carry max burst in the ep companion descriptor */
    if (udev->speed >= USB_SPEED_SUPER)
        return ed->ed_MaxBurst;

    if (udev->speed == USB_SPEED_HIGH && (ed_is_isoc(ed) || ed_is_int(ed)))
        return (u8)(ed_maxp_mult(ed) - 1u);

    return 0;
}

/*
 * Return the maximum endpoint service interval time (ESIT) payload.
 * Basically, this is the maxpacket size, multiplied by the burst size
 * and mult size.
 */
static u32 xhci_get_max_esit_payload(struct usb_device *udev, const struct UhcdEndpointDesc *ed)
{
    /* Only applies for interrupt or isochronous endpoints */
    if (ed_is_control(ed) || ed_is_bulk(ed))
        return 0;

    /* SuperSpeed Isoc ep with less than 48k per esit */
    if (udev->speed >= USB_SPEED_SUPER)
        return ed->ed_BytesPerInterval;

    /* ed_maxp_mult() already returns the encoded multiplier + 1. */
    return (u32)ed_maxp(ed) * ed_maxp_mult(ed);
}

/**
 * Fill one endpoint context (+ allocate its transfer ring) from a stack-supplied
 * endpoint descriptor (driven by xhci_configure_endpoints_from_list).  The
 * ed_* fields are host-order USB facts; the xHCI-specific derivation lives in
 * the xhci_get_* helpers.
 *
 * @param udev  pointer to the USB device structure
 * @param ed    the context-ABI endpoint descriptor
 */
static s8 xhci_init_one_ep_context(struct usb_device *udev,
                                   const struct UhcdEndpointDesc *ed)
{
    struct xhci_ctrl *ctrl = udev->controller;
    u8 err_count = 0;

    /*
     * Get values to fill the endpoint context, mostly from the ep
     * descriptor. The average TRB buffer length for bulk endpoints
     * is unclear as we have no clue on scatter gather list entry
     * size. For Isoc and Int, set it to max available.
     * See xHCI 1.1 spec 4.14.1.1 for details.
     */
    u32 max_esit_payload = xhci_get_max_esit_payload(udev, ed);
    u8 interval = xhci_get_endpoint_interval(udev, ed);
    u8 mult = xhci_get_endpoint_mult(udev, ed);
    u8 max_burst = xhci_get_endpoint_max_burst(udev, ed);

    /* VL805 corrupts SS bulk OUT bursts for mass-storage devices behind a
     * hub (Linux XHCI_VLI_SS_BULK_OUT_BUG, xhci-mem.c). */
    if ((ctrl->quirks & XHCI_QUIRK_SS_BULK_OUT) && max_burst != 0 &&
        udev->speed >= USB_SPEED_SUPER && udev->route != 0 &&
        ed_is_bulk(ed) && !ed_dir_in(ed) &&
        ed->ed_IfClass == USB_CLASS_MASS_STORAGE)
    {
        Kprintf("VL805 quirk: max_burst %lu -> 0 for slot %lu ep 0x%02lx (SS bulk OUT, UMS behind hub)\n",
                (ULONG)max_burst, (ULONG)udev->slot_id,
                (ULONG)ed->ed_Address);
        max_burst = 0;
    }

    u32 avg_trb_len = max_esit_payload;

    u8 ep_index = xhci_ep_index(ed);
    u8 dir = ed_dir_in(ed) ? 1u : 0u;
    u8 ep_type = (u8)((ed_xfer_type(ed) | ((u32)dir << 2u)) & 0xffU);

    /* Allow 3 retries for everything but isoc, set CErr = 3 */
    if (!ed_is_isoc(ed))
        err_count = 3;

    /* xHCI spec 6.2.3: 'Average TRB Length' should be 8 for control endpoints. */
    if (ed_is_control(ed))
        avg_trb_len = 8;

    u16 max_packet_size = ed_maxp(ed);
    s8 err = xhci_wire_ep_ctx(udev, ep_index, ep_type, max_packet_size, max_burst, err_count,
                              EP_MAX_ESIT_PAYLOAD_HI(max_esit_payload) | EP_INTERVAL(interval) | EP_MULT(mult),
                              EP_MAX_ESIT_PAYLOAD_LO(max_esit_payload) | EP_AVG_TRB_LENGTH(avg_trb_len));
    if (err != UHIOERR_NO_ERROR)
        return err;

    xhci_ep_set_rt_interval(xhci_ep_get_context_for_index(udev, ep_index), interval);

    KprintfT("EP%lu %s: type=%lu maxp=%lu maxesit=%lu "
             "interval=%lu mult=%lu maxburst=%lu\n",
             (ULONG)ed_num(ed),
             dir ? "IN" : "OUT",
             (ULONG)ep_type,
             (ULONG)max_packet_size,
             (ULONG)max_esit_payload,
             (ULONG)interval,
             (ULONG)(mult + 1),
             (ULONG)(max_burst + 1));

    return UHIOERR_NO_ERROR;
}

static void xhci_update_slot_last_ctx(struct xhci_ctrl *ctrl,
                                      struct usb_device *udev,
                                      u32 max_ep_flag)
{
    if (!ctrl || !udev)
        return;

    struct xhci_slot_ctx *slot_ctx = xhci_get_slot_ctx(ctrl, udev->in_ctx);
    if (!slot_ctx)
        return;

    u32 dev_info = le32(slot_ctx->dev_info);
    dev_info &= ~LAST_CTX_MASK;
    dev_info |= LAST_CTX(max_ep_flag + 1);
    slot_ctx->dev_info = le32(dev_info);
}

/* Current hardware endpoint state (EP_STATE_*) from the output endpoint context. */
/* Reads the live hardware EP State (EP_STATE_*) from the HC's device context.
 * Distinct from xhci_ep_get_state(), which returns the driver's software
 * ep_state bookkeeping. */
u32 xhci_read_hw_ep_state(struct usb_device *udev, u8 ep_index)
{
    struct xhci_ep_ctx *ep_ctx = xhci_get_ep_ctx(udev->controller, udev->out_ctx, ep_index);
    cache_post_dma(ep_ctx, sizeof(*ep_ctx), 0);
    return le32(ep_ctx->ep_info) & EP_STATE_MASK;
}

/* Reads the xHCI EP Type field (xHCI §6.2.3) from the HC's device context.
 * 0 = context not valid (disabled endpoint). */
u32 xhci_read_hw_ep_type(struct usb_device *udev, u8 ep_index)
{
    struct xhci_ep_ctx *ep_ctx = xhci_get_ep_ctx(udev->controller, udev->out_ctx, ep_index);
    cache_post_dma(ep_ctx, sizeof(*ep_ctx), 0);
    return CTX_TO_EP_TYPE(le32(ep_ctx->ep_info2));
}

/* Reads the xHCI Interval exponent (service period = 2^n * 125µs) from the
 * HC's device context. */
u32 xhci_read_hw_ep_interval(struct usb_device *udev, u8 ep_index)
{
    struct xhci_ep_ctx *ep_ctx = xhci_get_ep_ctx(udev->controller, udev->out_ctx, ep_index);
    cache_post_dma(ep_ctx, sizeof(*ep_ctx), 0);
    return CTX_TO_EP_INTERVAL(le32(ep_ctx->ep_info));
}

/* Writes the current udev->lpm.max_exit_latency_us into MAX_EXIT in the input slot context. */
void xhci_update_mel_in_input_ctx(struct usb_device *udev)
{
    u32 mel = udev->lpm.max_exit_latency_us > 0xffffU ? 0xffffU : udev->lpm.max_exit_latency_us;
    struct xhci_slot_ctx *slot = xhci_get_slot_ctx(udev->controller, udev->in_ctx);
    u32 d2 = le32(slot->dev_info2);
    d2 = (d2 & ~MAX_EXIT) | mel;
    slot->dev_info2 = le32(d2);
}

/* ---- Context-ABI (NSCMD_USB_*) entry points ------------------------------
 * These build the input context straight from the stack-supplied endpoint
 * list — there is no descriptor model.  The issued command's completion
 * (handle_config_ep) replies the op request via the REQ_CTX_OP path. */

s8 xhci_configure_endpoints_from_list(struct usb_device *udev,
                                      const struct UhcdEndpointDesc *add, u16 num_add,
                                      const u8 *drop_addresses, u16 num_drop,
                                      struct xhci_xfer *req)
{
    struct xhci_ctrl *ctrl = udev->controller;

    xhci_refresh_input_from_output(udev, 0);

    u32 add_flags = SLOT_FLAG;
    u32 drop_flags = 0;

    for (u16 i = 0; i < num_add; ++i)
    {
        const struct UhcdEndpointDesc *ed = &add[i];

        s8 err = xhci_init_one_ep_context(udev, ed);
        if (err != UHIOERR_NO_ERROR)
            return err;

        u8 ep_index = xhci_ep_index(ed);
        /* SS bulk stream capability, consumed by NSCMD_USB_ALLOC_STREAMS */
        xhci_ep_set_max_streams(xhci_ep_get_context_for_index(udev, ep_index), ed->ed_MaxStreams);
        add_flags |= BIT(ep_index + 1);
    }

    for (u16 i = 0; i < num_drop; ++i)
    {
        /* Derive the index from the address alone; non-EP0 control endpoints
         * (which would need the control-endpoint indexing rule) do not occur
         * in practice. */
        drop_flags |= BIT(xhci_ep_index_from_address(drop_addresses[i]) + 1);
    }
    drop_flags &= ~(u32)(SLOT_FLAG | EP0_FLAG);

    struct xhci_input_control_ctx *ctrl_ctx = xhci_get_input_control_ctx(udev->in_ctx);
    ctrl_ctx->add_flags = le32(add_flags);
    ctrl_ctx->drop_flags = le32(drop_flags);

    /* Context Entries must name the highest valid DCI *after* this op, so
     * recompute it exactly from the hardware state plus this op's add/drop
     * set — an incremental max can only grow and goes stale on a shrinking
     * SET_INTERFACE.  xHCI processes drops before adds, so add wins on
     * overlap. */
    u32 last_ep_index = 0;
    for (u8 ep_index = 1; ep_index < USB_MAX_ENDPOINT_CONTEXTS; ++ep_index)
    {
        BOOL present;
        if (add_flags & BIT(ep_index + 1))
            present = TRUE;
        else if (drop_flags & BIT(ep_index + 1))
            present = FALSE;
        else
            present = xhci_read_hw_ep_state(udev, ep_index) != EP_STATE_DISABLED;
        if (present)
            last_ep_index = ep_index;
    }
    xhci_update_slot_last_ctx(ctrl, udev, last_ep_index);

    KprintfT("configure_endpoints_from_list: slot=%lu add=0x%lx drop=0x%lx last=%lu\n",
             (ULONG)udev->slot_id, (ULONG)add_flags, (ULONG)drop_flags, (ULONG)last_ep_index);

    xhci_configure_endpoints(udev, FALSE, req);
    return UHIOERR_NO_ERROR;
}

/* SET_CONFIGURATION 0: drop every endpoint but EP0, returning the slot to the
 * Addressed state (the DC-flag-free equivalent — an explicit drop-all). */
void xhci_deconfigure(struct usb_device *udev, struct xhci_xfer *req)
{
    struct xhci_ctrl *ctrl = udev->controller;

    xhci_refresh_input_from_output(udev, 0);

    u32 drop_flags = 0;
    for (u8 ep_index = 1; ep_index < USB_MAX_ENDPOINT_CONTEXTS; ++ep_index)
    {
        if (xhci_ep_get_context_for_index(udev, ep_index))
            drop_flags |= BIT(ep_index + 1);
    }

    struct xhci_input_control_ctx *ctrl_ctx = xhci_get_input_control_ctx(udev->in_ctx);
    ctrl_ctx->add_flags = le32(SLOT_FLAG);
    ctrl_ctx->drop_flags = le32(drop_flags);

    xhci_update_slot_last_ctx(ctrl, udev, 0); /* only EP0 remains */

    xhci_configure_endpoints(udev, FALSE, req);
}

/* Shared prologue of the two stream-mode switches: refresh the input context
 * from hardware, target one endpoint with an add flag, and return its input
 * endpoint context for patching. */
static struct xhci_ep_ctx *xhci_streams_input_ctx(struct usb_device *udev, u8 ep_index)
{
    struct xhci_ctrl *ctrl = udev->controller;

    xhci_refresh_input_from_output(udev, (s16)ep_index);

    struct xhci_input_control_ctx *ctrl_ctx = xhci_get_input_control_ctx(udev->in_ctx);
    ctrl_ctx->add_flags = le32(SLOT_FLAG | BIT(ep_index + 1));
    ctrl_ctx->drop_flags = 0;

    return xhci_get_ep_ctx(ctrl, udev->in_ctx, ep_index);
}

/* NSCMD_USB_ALLOC/FREE_STREAMS: switch the endpoint context between the
 * pre-built linear stream context array (enable: MaxPStreams + LSA; deq =
 * array base) and the single default ring (disable: the ring was idle when
 * streams were allocated and untouched since — its software enqueue position
 * is the correct restart dequeue). */
void xhci_configure_ep_stream_mode(struct usb_device *udev, u8 ep_index, BOOL enable, struct xhci_xfer *req)
{
    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
    struct xhci_ep_ctx *epc = xhci_streams_input_ctx(udev, ep_index);

    u32 info = le32(epc->ep_info);
    info &= ~(EP_MAXPSTREAMS_MASK | (u32)EP_HAS_LSA);
    if (enable)
        info |= EP_MAXPSTREAMS(xhci_ep_streams_max_pstreams(ep_ctx)) | EP_HAS_LSA;
    epc->ep_info = le32(info);
    epc->deq = enable ? le64((u64)xhci_ep_streams_array_dma(ep_ctx))
                      : le64((u64)xhci_ring_get_new_dequeue_ptr(xhci_ep_get_ring(ep_ctx)));

    xhci_configure_endpoints(udev, FALSE, req);
}

/* Apply stack-supplied hub facts (NSCMD_USB_UPDATE_HUB) to the slot context.
 * Hub fields are evaluated by Configure Endpoint (not Evaluate Context) —
 * legal in the Addressed state, where this normally runs. */
void xhci_apply_hub_update(struct usb_device *udev, struct xhci_xfer *req)
{
    xhci_refresh_input_from_output(udev, -1);

    struct xhci_input_control_ctx *ctrl_ctx = xhci_get_input_control_ctx(udev->in_ctx);
    ctrl_ctx->add_flags = le32(SLOT_FLAG);
    ctrl_ctx->drop_flags = 0;

    xhci_configure_endpoints(udev, FALSE, req);
}

#ifdef DEBUG /* name helpers below are used only by the context dumps */

static const char *slot_state_name(u32 state)
{
    switch (state)
    {
    case SLOT_STATE_DISABLED:
        return "disabled";
    case SLOT_STATE_DEFAULT:
        return "default";
    case SLOT_STATE_ADDRESSED:
        return "addressed";
    case SLOT_STATE_CONFIGURED:
        return "configured";
    default:
        return "reserved";
    }
}

static const char *slot_speed_name(u32 dev_info)
{
    switch (dev_info & DEV_SPEED)
    {
    case SLOT_SPEED_LS:
        return "low";
    case SLOT_SPEED_FS:
        return "full";
    case SLOT_SPEED_HS:
        return "high";
    case SLOT_SPEED_SS:
        return "super";
    default:
        return "reserved";
    }
}

static const char *ep_state_name(u32 state)
{
    switch (state & EP_STATE_MASK)
    {
    case EP_STATE_DISABLED:
        return "disabled";
    case EP_STATE_RUNNING:
        return "running";
    case EP_STATE_HALTED:
        return "halted";
    case EP_STATE_STOPPED:
        return "stopped";
    case EP_STATE_ERROR:
        return "error";
    default:
        return "reserved";
    }
}

static const char *ep_type_name(u32 type)
{
    switch (type & 0x7)
    {
    case ISOC_OUT_EP:
        return "isoc-out";
    case BULK_OUT_EP:
        return "bulk-out";
    case INT_OUT_EP:
        return "int-out";
    case CTRL_EP:
        return "control";
    case ISOC_IN_EP:
        return "isoc-in";
    case BULK_IN_EP:
        return "bulk-in";
    case INT_IN_EP:
        return "int-in";
    default:
        return "reserved";
    }
}

void xhci_dump_slot_ctx(const char *tag, struct usb_device *udev, BOOL in_ctx)
{
    struct xhci_container_ctx *ctx = in_ctx ? udev->in_ctx : udev->out_ctx;
    struct xhci_slot_ctx *slot_ctx = xhci_get_slot_ctx(udev->controller, ctx);
    cache_post_dma(slot_ctx, sizeof(struct xhci_slot_ctx), 0);
    const char *pfx = tag ? tag : "";

    if (!slot_ctx)
    {
        Kprintf("%s Slot context: (null)\n", pfx);
        return;
    }

    u32 dev_info = le32(slot_ctx->dev_info);
    u32 dev_info2 = le32(slot_ctx->dev_info2);
    u32 tt_info = le32(slot_ctx->tt_info);
    u32 dev_state = le32(slot_ctx->dev_state);

    u32 route = dev_info & ROUTE_STRING_MASK;
    u32 last_ctx = (dev_info & LAST_CTX_MASK) >> 27;
    s32 last_ep = (last_ctx == 0) ? -1 : ((s32)last_ctx - 1);

    u32 max_exit_latency = dev_info2 & MAX_EXIT;
    u32 root_port = DEVINFO_TO_ROOT_HUB_PORT(dev_info2);
    u32 max_ports = (dev_info2 >> 24) & 0xff;

    u32 tt_slot = tt_info & 0xff;
    u32 tt_port = (tt_info >> 8) & 0xff;
    u32 tt_think_code = (tt_info >> 16) & 0x3;
    u32 tt_think_bits = (tt_think_code + 1) * 8;

    u32 address = dev_state & DEV_ADDR_MASK;
    u32 slot_state = GET_SLOT_STATE(dev_state);
    Kprintf("%s Slot context %s @%lx\n", pfx, (in_ctx) ? "IN" : "OUT", (ULONG)slot_ctx);
    Kprintf("%s  dev_info=0x%08lx route=0x%05lx speed=%s hub=%ld mtt=%ld last_ctx=%lu (last_ep=%ld)\n",
            pfx,
            dev_info,
            route,
            slot_speed_name(dev_info),
            (LONG)((dev_info & DEV_HUB) != 0),
            (LONG)((dev_info & DEV_MTT) != 0),
            last_ctx,
            last_ep);
    Kprintf("%s  dev_info2=0x%08lx max_exit_latency=%lu root_port=%lu max_ports=%lu\n",
            pfx,
            dev_info2,
            max_exit_latency,
            root_port,
            max_ports);
    Kprintf("%s  tt_info=0x%08lx slot=%lu port=%lu think_time_code=%lu (%lu bit-times)\n",
            pfx,
            tt_info,
            tt_slot,
            tt_port,
            tt_think_code,
            tt_think_bits);
    Kprintf("%s  dev_state=0x%08lx xhci_address=%lu state=%s(%lu)\n",
            pfx,
            dev_state,
            address,
            slot_state_name(slot_state),
            slot_state);
}

#endif /* DEBUG (context-dump helpers + xhci_dump_slot_ctx) */

/* Issue an Evaluate Context carrying udev->lpm.max_exit_latency_us (mirror
 * xhci_change_max_exit_latency).  The xHC evaluates the slot's Max Exit
 * Latency only through Address Device / Evaluate Context (xHCI 6.2.2); the
 * value carried in a CONFIG_EP input context is ignored, and with an internal
 * MEL of 0 the controller will not take the link into U1/U2. */
void xhci_evaluate_mel(struct usb_device *udev, struct xhci_xfer *req)
{
    struct xhci_ctrl *ctrl = udev->controller;

    struct xhci_input_control_ctx *ctrl_ctx = xhci_get_input_control_ctx(udev->in_ctx);
    ctrl_ctx->add_flags = le32(SLOT_FLAG);
    ctrl_ctx->drop_flags = 0;

    cache_post_dma(udev->out_ctx->bytes, udev->out_ctx->size, 0);
    xhci_slot_copy(ctrl, udev->in_ctx, udev->out_ctx);
    xhci_update_mel_in_input_ctx(udev);

    struct xhci_slot_ctx *slot = xhci_get_slot_ctx(ctrl, udev->in_ctx);
    slot->dev_state = 0;

    KprintfT("Evaluate Context: MEL=%lu us for slot %lu\n",
             (ULONG)udev->lpm.max_exit_latency_us, (ULONG)udev->slot_id);
    xhci_configure_endpoints(udev, TRUE, req);
}

#ifdef DEBUG

void xhci_dump_ep_ctx(const char *tag, struct usb_device *udev, u8 ep_index)
{
    struct xhci_ep_ctx *ep_ctx = xhci_get_ep_ctx(udev->controller, udev->out_ctx, ep_index);
    cache_post_dma(ep_ctx, sizeof(struct xhci_ep_ctx), 0);
    const char *pfx = tag ? tag : "";

    if (!ep_ctx)
    {
        Kprintf("%s Endpoint context[%lu]: (null)\n", pfx, (ULONG)ep_index);
        return;
    }

    u32 ep_info = le32(ep_ctx->ep_info);
    u32 ep_info2 = le32(ep_ctx->ep_info2);
    u64 deq = le64(ep_ctx->deq);
    u32 tx_info = le32(ep_ctx->tx_info);

    u32 state = ep_info & EP_STATE_MASK;
    u32 mult = CTX_TO_EP_MULT(ep_info);
    u32 mult_count = mult + 1;
    u32 interval = CTX_TO_EP_INTERVAL(ep_info);
    u32 max_ps = (ep_info >> 10) & 0x1f;
    BOOL has_lsa = (ep_info & EP_HAS_LSA) != 0;

    u32 ep_type = CTX_TO_EP_TYPE(ep_info2);
    u32 error_count = (ep_info2 >> 1) & 0x3;
    u32 max_burst = CTX_TO_MAX_BURST(ep_info2);
    u32 max_packet = MAX_PACKET_DECODED(ep_info2);
    BOOL force_event = (ep_info2 & FORCE_EVENT) != 0;

    u32 esit_lo = (tx_info >> 16) & 0xffff;
    u32 esit_hi = (tx_info >> 24) & 0xff;
    u32 max_esit_payload = esit_lo | (esit_hi << 16);
    u32 avg_trb_len = EP_AVG_TRB_LENGTH(tx_info);

    u32 deq_low = (u32)(deq & 0xffffffffUL);
    u32 deq_high = (u32)((deq >> 32) & 0xffffffffUL);
    BOOL cycle_state = (deq & EP_CTX_CYCLE_MASK) != 0;

    Kprintf("%s Endpoint context[%lu] @%lx\n", pfx, ep_index, (ULONG)ep_ctx);
    Kprintf("%s  ep_info=0x%08lx state=%s(%lu) mult=%lu (%lu per uframe) interval=%lu streams=%lu lsa=%ld\n",
            pfx,
            ep_info,
            ep_state_name(state),
            state,
            mult,
            mult_count,
            interval,
            max_ps,
            has_lsa);
    Kprintf("%s  ep_info2=0x%08lx type=%s(%lu) max_packet=%lu max_burst=%lu error_count=%lu force_event=%ld\n",
            pfx,
            ep_info2,
            ep_type_name(ep_type),
            ep_type,
            max_packet,
            max_burst,
            error_count,
            force_event);
    if (deq_high)
        Kprintf("%s  deq=0x%08lx%08lx cycle=%ld\n",
                pfx,
                deq_high,
                deq_low,
                cycle_state);
    else
        Kprintf("%s  deq=0x%08lx cycle=%ld\n",
                pfx,
                deq_low,
                cycle_state);
    Kprintf("%s  tx_info=0x%08lx avg_trb_len=%lu max_esit_payload=%lu\n",
            pfx,
            tx_info,
            avg_trb_len,
            max_esit_payload);
}

#endif /* DEBUG (xhci_dump_ep_ctx) */
