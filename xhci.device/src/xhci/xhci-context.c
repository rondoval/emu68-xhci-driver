// SPDX-License-Identifier: GPL-2.0+
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
#include <proto/exec.h>
#endif

#include <compat.h>
#include <debug.h>

#include <xhci/xhci.h>
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

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci-context] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

/**
 * Allocates the Container context
 *
 * @param ctrl	Host controller data structure
 * @param type type of XHCI Container Context
 * Return: NULL if failed else pointer to the context on success
 */
struct xhci_container_ctx *xhci_alloc_container_ctx(struct xhci_ctrl *ctrl, int type)
{
    struct xhci_container_ctx *ctx = AllocVecPooled(ctrl->memoryPool, sizeof(struct xhci_container_ctx));
    if (!ctx)
    {
        Kprintf("Failed to allocate container context\n");
        return NULL;
    }

    if ((type != XHCI_CTX_TYPE_DEVICE) && (type != XHCI_CTX_TYPE_INPUT))
    {
        Kprintf("Invalid context type\n");
        FreeVecPooled(ctrl->memoryPool, ctx);
        return NULL;
    }

    ctx->type = type;
    ctx->size = (USB_MAX_ENDPOINT_CONTEXTS + 1) * CTX_SIZE(readl(&ctrl->hccr->cr_hccparams));
    if (type == XHCI_CTX_TYPE_INPUT)
        ctx->size += CTX_SIZE(readl(&ctrl->hccr->cr_hccparams));

    ctx->bytes = xhci_malloc(ctrl, ctx->size);
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
    memalign_free(ctrl->memoryPool, ctx->bytes);
    FreeVecPooled(ctrl->memoryPool, ctx);
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

    return (struct xhci_slot_ctx *)(ctx->bytes + CTX_SIZE(readl(&ctrl->hccr->cr_hccparams)));
}

/**
 * Gets the EP context from based on the ep_index
 *
 * @param ctrl	Host controller data structure
 * @param ctx	context container
 * @param ep_index	index of the endpoint
 * Return: pointer to the End point context
 */
static struct xhci_ep_ctx *xhci_get_ep_ctx(struct xhci_ctrl *ctrl, struct xhci_container_ctx *ctx, unsigned int ep_index)
{
    /* increment ep index by offset of start of ep ctx array */
    ep_index++;
    if (ctx->type == XHCI_CTX_TYPE_INPUT)
        ep_index++;

    return (struct xhci_ep_ctx *)(ctx->bytes + (ep_index * CTX_SIZE(readl(&ctrl->hccr->cr_hccparams))));
}

u32 xhci_get_hardware_address(struct usb_device *udev)
{
    struct xhci_slot_ctx *slot_ctx = xhci_get_slot_ctx(udev->controller, udev->out_ctx);
    xhci_inval_cache(slot_ctx, sizeof(struct xhci_slot_ctx));
    return LE32(slot_ctx->dev_state & DEV_ADDR_MASK);
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
                               unsigned int ep_index)
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

static unsigned int route_depth(unsigned int route)
{
    unsigned int depth = 0;
    while (route && depth < 5)
    {
        route >>= 4;
        depth++;
    }
    return depth;
}

static unsigned int build_route_string(struct usb_device *parent, unsigned int port)
{
    if (!parent)
        return 0;

    unsigned int depth = route_depth(parent->route);
    if (depth >= 5)
        return parent->route;

    unsigned int nibble = port & 0xF;
    if (nibble == 0)
        return parent->route;

    return parent->route | (nibble << (depth * 4));
}

/**
 * Setup an xHCI virtual device for a Set Address command
 *
 * @param udev pointer to the Device Data Structure
 * Return: returns negative value on failure else 0 on success
 */
void xhci_setup_addressable_virt_dev(struct xhci_ctrl *ctrl, struct usb_device *udev)
{
    udev->parent = ctrl->pending_parent;
    udev->parent_port = ctrl->pending_parent_port;
    udev->route = build_route_string(udev->parent, udev->parent_port);

    /* Extract the EP0 and Slot Ctrl */
    struct xhci_ep_ctx *ep0_ctx = xhci_get_ep_ctx(ctrl, udev->in_ctx, 0);
    struct xhci_slot_ctx *slot_ctx = xhci_get_slot_ctx(ctrl, udev->in_ctx);
    KprintfH("slot=%ld in_ctx=%lx out_ctx=%lx ep0_ctx=%lx slot_ctx=%lx\n",
             (ULONG)udev->slot_id, (ULONG)udev->in_ctx, (ULONG)udev->out_ctx,
             (ULONG)ep0_ctx, (ULONG)slot_ctx);

    /* Only the control endpoint is valid - one endpoint context */
    u32 dev_info = LE32(slot_ctx->dev_info);
    dev_info &= ~(ROUTE_STRING_MASK | DEV_SPEED | DEV_MTT | LAST_CTX_MASK);
    dev_info |= (udev->route & ROUTE_STRING_MASK);
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
        Kprintf("Unknown device speed %ld\n", (ULONG)udev->speed);
    }

    // Find root hub port number
    u32 root_port = ctrl->pending_parent_port;
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

    /* Low/full-speed devices behind a high-speed hub need TT info */
    BOOL needs_tt = (udev->speed == USB_SPEED_FULL || udev->speed == USB_SPEED_LOW) &&
                    udev->parent &&
                    (udev->parent->speed == USB_SPEED_HIGH ||
                     udev->parent->speed == USB_SPEED_SUPER ||
                     udev->parent->speed == USB_SPEED_SUPER_PLUS);

    slot_ctx->dev_info = LE32(dev_info);

    KprintfH("xhci_setup_addressable_virt_dev: parent_addr=%ld port_num=%ld root_port_num=%ld speed=%ld route=%lx\n",
             (ULONG)udev->parent->poseidon_address, udev->parent_port, root_port, (ULONG)udev->speed, (ULONG)udev->route);

    u32 dev_info2 = LE32(slot_ctx->dev_info2);
    dev_info2 &= ~((ROOT_HUB_PORT_MASK) << ROOT_HUB_PORT_SHIFT);
    dev_info2 |= ROOT_HUB_PORT(root_port);
    slot_ctx->dev_info2 = LE32(dev_info2);

    u32 tt_info = 0;
    if (needs_tt)
    {
        tt_info = TT_SLOT(udev->parent->slot_id) |
                  TT_PORT(udev->parent_port);
    }

    KprintfH("xhci_setup_addressable_virt_dev: needs_tt=%ld tt_slot=%ld tt_port=%ld tt_info=%08lx\n",
             (int)needs_tt,
             (int)udev->parent->slot_id, (int)udev->parent_port,
             (ULONG)tt_info);
    slot_ctx->tt_info = LE32(tt_info);

    /* Step 4 - ring already allocated */
    /* Step 5 */
    ep0_ctx->ep_info2 = LE32(EP_TYPE(CTRL_EP));
    KprintfH("xhci_setup_addressable_virt_dev: SPEED=%ld\n", (ULONG)udev->speed);

    int max_packet_size = 0;
    switch (udev->speed)
    {
    case USB_SPEED_SUPER:
        ep0_ctx->ep_info2 |= LE32(MAX_PACKET(512));
        max_packet_size = 512;
        KprintfH("xhci_setup_addressable_virt_dev: MPS=512\n");
        break;
    case USB_SPEED_HIGH:
    /* USB core guesses at a 64-byte max packet first for FS devices */
    case USB_SPEED_FULL:
        ep0_ctx->ep_info2 |= LE32(MAX_PACKET(64));
        max_packet_size = 64;
        KprintfH("xhci_setup_addressable_virt_dev: MPS=64\n");
        break;
    case USB_SPEED_LOW:
        ep0_ctx->ep_info2 |= LE32(MAX_PACKET(8));
        max_packet_size = 8;
        KprintfH("xhci_setup_addressable_virt_dev: MPS=8\n");
        break;
    default:
        /* New speed? */
        Kprintf("Unknown device speed %ld\n", (ULONG)udev->speed);
    }

    /* EP 0 can handle "burst" sizes of 1, so Max Burst Size field is 0 */
    ep0_ctx->ep_info2 |= LE32(MAX_BURST(0) | ERROR_COUNT(3));

    BOOL result = xhci_ep_create_context(udev, 0, max_packet_size, ctrl->memoryPool);
    if (!result)
        Kprintf("Failed to create EP0 context\n");

    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, 0);
    struct xhci_ring *ring = xhci_ep_get_ring(ep_ctx);
    ep0_ctx->deq = LE64(xhci_ring_get_new_dequeue_ptr(ring));

    /*
     * xHCI spec 6.2.3:
     * software shall set 'Average TRB Length' to 8 for control endpoints.
     */
    ep0_ctx->tx_info = LE32(EP_AVG_TRB_LENGTH(8));

    /* Steps 7 and 8 were done in xhci_alloc_virt_device() */

    xhci_flush_cache(ep0_ctx, sizeof(struct xhci_ep_ctx));
    xhci_flush_cache(slot_ctx, sizeof(struct xhci_slot_ctx));
    KprintfH("xhci_setup_addressable_virt_dev: ep0 deq=%lx tx_info=%08lx dev_info=%08lx dev_info2=%08lx\n",
             (ULONG)LE64(ep0_ctx->deq), (ULONG)LE32(ep0_ctx->tx_info),
             (ULONG)LE32(slot_ctx->dev_info), (ULONG)LE32(slot_ctx->dev_info2));

    struct xhci_input_control_ctx *ctrl_ctx = xhci_get_input_control_ctx(udev->in_ctx);
    ctrl_ctx->add_flags = LE32(SLOT_FLAG | EP0_FLAG);
    ctrl_ctx->drop_flags = 0;

    xhci_flush_cache(ctrl_ctx, sizeof(struct xhci_input_control_ctx));
}

void xhci_update_hub_tt(struct usb_device *udev)
{
    if (!udev || !udev->controller)
        return;

    if (udev->slot_id == 0)
        return;

    udev->route = build_route_string(udev->parent, udev->parent_port);

    struct xhci_ctrl *ctrl = udev->controller;

    xhci_inval_cache(udev->out_ctx->bytes, udev->out_ctx->size);

    /* Start from the controller's current view of the slot context */
    xhci_slot_copy(ctrl, udev->in_ctx, udev->out_ctx);

    struct xhci_slot_ctx *slot_ctx = xhci_get_slot_ctx(ctrl, udev->in_ctx);
    struct xhci_input_control_ctx *ctrl_ctx = xhci_get_input_control_ctx(udev->in_ctx);
    if (!slot_ctx || !ctrl_ctx)
        return;

    u32 dev_info = LE32(slot_ctx->dev_info);
    u32 tt_info = LE32(slot_ctx->tt_info);

    dev_info |= DEV_HUB;

    tt_info &= ~(TT_SLOT(0xff) | TT_PORT(0xff) | TT_THINK_TIME(0x3));
    tt_info = TT_SLOT(udev->parent->slot_id) |
              TT_PORT(udev->parent_port) |
              TT_THINK_TIME(udev->tt_think_time & 0x3);

    slot_ctx->dev_info = LE32(dev_info);
    slot_ctx->tt_info = LE32(tt_info);

    ctrl_ctx->add_flags = LE32(SLOT_FLAG);
    ctrl_ctx->drop_flags = 0;

    xhci_flush_cache(slot_ctx, sizeof(*slot_ctx));
    xhci_flush_cache(ctrl_ctx, sizeof(*ctrl_ctx));
    /* xhci_configure_endpoints will flush the full input context */

    KprintfH("xhci_update_hub_tt: addr=%ld tt_code=%ld\n",
             (ULONG)udev->poseidon_address, (LONG)udev->tt_think_time);

    xhci_configure_endpoints(udev, TRUE, NULL);
}

/*
 * Full speed devices may have a max packet size greater than 8 bytes, but the
 * USB core doesn't know that until it reads the first 8 bytes of the
 * descriptor.  If the usb_device's max packet size changes after that point,
 * we need to issue an evaluate context command and wait on it.
 *
 * @param udev	pointer to the Device Data Structure
 * Return: returns the status of the xhci_configure_endpoints
 */
void xhci_update_maxpacket(struct usb_device *udev, unsigned int max_packet_size)
{
    struct xhci_ctrl *ctrl = udev->controller;
    int ep_index = 0; /* control endpoint */
    unsigned int hw_max_packet_size;

    xhci_inval_cache(udev->out_ctx->bytes, udev->out_ctx->size);
    KprintfH("Checking max packet size for ep 0 of address %ld (slot %ld)\n", (LONG)udev->poseidon_address, (LONG)udev->slot_id);

    struct xhci_ep_ctx *ep_ctx = xhci_get_ep_ctx(ctrl, udev->out_ctx, ep_index);
    hw_max_packet_size = MAX_PACKET_DECODED(LE32(ep_ctx->ep_info2));

    if (hw_max_packet_size == max_packet_size)
    {
        KprintfH("Max Packet Size for ep 0 is already correct at %ld.\n", max_packet_size);
        return;
    }

    KprintfH("Max Packet Size for ep 0 changed to %ld.\n", max_packet_size);
    KprintfH("Max packet size in xHCI HW = %ld\n", hw_max_packet_size);

    // Update the EP context's max packet size as well
    struct ep_context *ep_context = xhci_ep_get_context_for_index(udev, ep_index);
    xhci_ep_set_max_packet_size(ep_context, max_packet_size);

    /* Set up the modified control endpoint 0 */
    xhci_endpoint_copy(ctrl, udev->in_ctx,
                       udev->out_ctx, ep_index);
    ep_ctx = xhci_get_ep_ctx(ctrl, udev->in_ctx, ep_index);
    ep_ctx->ep_info2 &= LE32(~MAX_PACKET(MAX_PACKET_MASK));
    ep_ctx->ep_info2 |= LE32(MAX_PACKET(max_packet_size));

    /*
     * Set up the input context flags for the command
     * FIXME: This won't work if a non-default control endpoint
     * changes max packet sizes.
     */
    struct xhci_input_control_ctx *ctrl_ctx = xhci_get_input_control_ctx(udev->in_ctx);
    ctrl_ctx->add_flags = LE32(EP0_FLAG);
    ctrl_ctx->drop_flags = 0;

    xhci_configure_endpoints(udev, TRUE, NULL);
}

/**
 * Fill endpoint contexts for interface descriptor ifdesc.
 *
 * @param udev		pointer to the USB device structure
 * @param ctrl		pointer to the xhci pravte device structure
 * @param virt_dev	pointer to the xhci virtual device structure
 * @param ifdesc	pointer to the USB interface config descriptor
 * Return: returns the status of xhci_init_ep_contexts_if
 */
static int xhci_init_ep_contexts_if(struct usb_device *udev,
                                    struct xhci_ctrl *ctrl,
                                    struct usb_interface *ifdesc)
{
    KprintfH("xhci_init_ep_contexts_if: enter\n");
    struct xhci_ep_ctx *ep_ctx[USB_MAX_ENDPOINT_CONTEXTS];
    int cur_ep;
    int ep_index;
    unsigned int dir;
    unsigned int ep_type;
    u32 max_esit_payload;
    unsigned int interval;
    unsigned int mult;
    unsigned int max_burst;
    unsigned int avg_trb_len;
    unsigned int err_count = 0;
    struct usb_interface_altsetting *active_alt = ifdesc->active_altsetting;
    if (!active_alt)
    {
        Kprintf("xhci_init_ep_contexts_if: no active altsetting for iface %ld\n",
                (ULONG)ifdesc->interface_number);
        return UHIOERR_BADPARAMS;
    }

    int num_of_ep = active_alt->no_of_ep;

    for (cur_ep = 0; cur_ep < num_of_ep; cur_ep++)
    {
        struct usb_endpoint_descriptor *endpt_desc = NULL;
        struct usb_ss_ep_comp_descriptor *ss_ep_comp_desc = NULL;

        endpt_desc = &active_alt->ep_desc[cur_ep];
        ss_ep_comp_desc = &active_alt->ss_ep_comp_desc[cur_ep];

        /*
         * Get values to fill the endpoint context, mostly from ep
         * descriptor. The average TRB buffer lengt for bulk endpoints
         * is unclear as we have no clue on scatter gather list entry
         * size. For Isoc and Int, set it to max available.
         * See xHCI 1.1 spec 4.14.1.1 for details.
         */
        max_esit_payload = xhci_get_max_esit_payload(udev, endpt_desc, ss_ep_comp_desc);
        interval = xhci_get_endpoint_interval(udev, endpt_desc);
        mult = xhci_get_endpoint_mult(udev, endpt_desc, ss_ep_comp_desc);
        max_burst = xhci_get_endpoint_max_burst(udev, endpt_desc, ss_ep_comp_desc);
        avg_trb_len = max_esit_payload;

        ep_index = xhci_get_ep_index(endpt_desc);
        ep_ctx[ep_index] = xhci_get_ep_ctx(ctrl, udev->in_ctx, ep_index);

        int max_packet_size = usb_endpoint_maxp(endpt_desc);
        /* Allocate the ep rings */
        BOOL result = xhci_ep_create_context(udev, ep_index, max_packet_size, ctrl->memoryPool);
        if (!result)
            return UHIOERR_OUTOFMEMORY;

        /*NOTE: ep_desc[0] actually represents EP1 and so on */
        dir = (((endpt_desc->bEndpointAddress) & (0x80)) >> 7);
        ep_type = (((endpt_desc->bmAttributes) & (0x3)) | (dir << 2));

        ep_ctx[ep_index]->ep_info = LE32(EP_MAX_ESIT_PAYLOAD_HI(max_esit_payload) | EP_INTERVAL(interval) | EP_MULT(mult));

        ep_ctx[ep_index]->ep_info2 = LE32(EP_TYPE(ep_type));
        ep_ctx[ep_index]->ep_info2 |= LE32(MAX_PACKET(max_packet_size));

        /* Allow 3 retries for everything but isoc, set CErr = 3 */
        if (!usb_endpoint_xfer_isoc(endpt_desc))
            err_count = 3;
        ep_ctx[ep_index]->ep_info2 |= LE32(MAX_BURST(max_burst) | ERROR_COUNT(err_count));

        struct ep_context *ep_context = xhci_ep_get_context_for_index(udev, ep_index);
        struct xhci_ring *ring = xhci_ep_get_ring(ep_context);
        ep_ctx[ep_index]->deq = LE64(xhci_ring_get_new_dequeue_ptr(ring));

        /*
         * xHCI spec 6.2.3:
         * 'Average TRB Length' should be 8 for control endpoints.
         */
        if (usb_endpoint_xfer_control(endpt_desc))
            avg_trb_len = 8;
        ep_ctx[ep_index]->tx_info = LE32(EP_MAX_ESIT_PAYLOAD_LO(max_esit_payload) | EP_AVG_TRB_LENGTH(avg_trb_len));

        KprintfH("EP%ld %s: type=%ld maxp=%ld maxesit=%ld "
                 "interval=%ld mult=%ld maxburst=%ld\n",
                 (ULONG)usb_endpoint_num(endpt_desc),
                 dir ? "IN" : "OUT",
                 (ULONG)ep_type,
                 (ULONG)usb_endpoint_maxp(endpt_desc),
                 (ULONG)max_esit_payload,
                 (ULONG)interval,
                 (ULONG)(mult + 1),
                 (ULONG)(max_burst + 1));
    }

    return UHIOERR_NO_ERROR;
}

static void xhci_update_slot_last_ctx(struct xhci_ctrl *ctrl,
                                      struct usb_device *udev,
                                      unsigned int max_ep_flag)
{
    if (!ctrl || !udev)
        return;

    struct xhci_slot_ctx *slot_ctx = xhci_get_slot_ctx(ctrl, udev->in_ctx);
    if (!slot_ctx)
        return;

    u32 dev_info = LE32(slot_ctx->dev_info);
    dev_info &= ~LAST_CTX_MASK;
    dev_info |= LAST_CTX(max_ep_flag + 1);
    slot_ctx->dev_info = LE32(dev_info);
}

/**
 * Configure the endpoint, programming the device contexts.
 *
 * @param udev	pointer to the USB device structure
 * Return: returns the status of the xhci_configure_endpoints
 */
int xhci_set_configuration(struct usb_device *udev, int config_value)
{
    KprintfH("xhci_set_configuration: config_val=%ld\n", (LONG)config_value);

    struct usb_config *cfg = xhci_find_config(udev, config_value);
    if (!cfg)
    {
        Kprintf("xhci_set_configuration: config_val=%ld not found!\n", (LONG)config_value);
        return UHIOERR_BADPARAMS;
    }

    udev->active_config = cfg;
    for (unsigned int i = 0; i < cfg->no_of_if; ++i)
        xhci_select_active_alt(&cfg->if_desc[i]);

#ifdef DEBUG_HIGH
    /* Dump entire cfg using kprintf (all fields and all interfaces and endpoints) */
    xhci_dump_config("[xhci] xhci_set_configuration:", cfg, (LONG)udev->poseidon_address);
#endif

    struct xhci_container_ctx *out_ctx;
    struct xhci_container_ctx *in_ctx;
    struct xhci_input_control_ctx *ctrl_ctx;
    struct xhci_ctrl *ctrl = udev->controller;
    unsigned int max_ifnum = cfg->no_of_if;
    unsigned int max_ep_flag = 0;

    out_ctx = udev->out_ctx;
    in_ctx = udev->in_ctx;

    ctrl_ctx = xhci_get_input_control_ctx(in_ctx);
    u32 add_flags = SLOT_FLAG;
    u32 mask = xhci_collect_config_masks(cfg, max_ifnum, &max_ep_flag);
    add_flags |= mask;
    ctrl_ctx->add_flags = LE32(add_flags);
    ctrl_ctx->drop_flags = 0;

    xhci_inval_cache(out_ctx->bytes, out_ctx->size);

    /* slot context */
    xhci_slot_copy(ctrl, in_ctx, out_ctx);
    xhci_update_slot_last_ctx(ctrl, udev, max_ep_flag);

    xhci_endpoint_copy(ctrl, in_ctx, out_ctx, 0);

    /* filling up ep contexts */
    for (unsigned int ifnum = 0; ifnum < max_ifnum; ++ifnum)
    {
        struct usb_interface *ifdesc = &cfg->if_desc[ifnum];
        int err = xhci_init_ep_contexts_if(udev, ctrl, ifdesc);
        if (err != UHIOERR_NO_ERROR)
        {
            return err;
        }
    }
    return UHIOERR_NO_ERROR;
}

int xhci_set_interface(struct usb_device *udev, unsigned int iface_number, unsigned int alt_setting)
{
    if (!udev || !udev->controller)
    {
        Kprintf("xhci_set_interface: invalid usb_device pointer\n");
        return UHIOERR_BADPARAMS;
    }

    struct usb_config *cfg = udev->active_config;
    if (!cfg)
    {
        Kprintf("xhci_set_interface: no active config for addr %ld\n", (LONG)udev->poseidon_address);
        return UHIOERR_BADPARAMS;
    }

    struct usb_interface *iface = xhci_find_interface(cfg, iface_number);
    if (!iface)
    {
        Kprintf("xhci_set_interface: interface %ld not found in config %ld\n",
                (LONG)iface_number, (LONG)cfg->desc.bConfigurationValue);
        return UHIOERR_BADPARAMS;
    }

    struct usb_interface_altsetting *current_alt = iface->active_altsetting;

    if (current_alt && current_alt->desc.bAlternateSetting == alt_setting)
    {
        KprintfH("xhci_set_interface: iface %ld already at alt %ld\n",
                 (LONG)iface_number, (LONG)alt_setting);
        return UHIOERR_NO_ERROR;
    }

    struct usb_interface_altsetting *new_alt = xhci_find_altsetting(iface, alt_setting);
    if (!new_alt)
    {
        Kprintf("xhci_set_interface: alt %ld missing for iface %ld\n",
                (LONG)alt_setting, (LONG)iface_number);
        return UHIOERR_BADPARAMS;
    }

    struct xhci_ctrl *ctrl = udev->controller;

    /* Poseidon stack guarantees endpoint queues are idle before switching. */
    u32 drop_mask = xhci_collect_ep_mask(current_alt, NULL);

    iface->active_altsetting = new_alt;

    u32 add_mask = xhci_collect_ep_mask(new_alt, NULL);
    xhci_inval_cache(udev->out_ctx->bytes, udev->out_ctx->size);
    xhci_slot_copy(ctrl, udev->in_ctx, udev->out_ctx);
    xhci_endpoint_copy(ctrl, udev->in_ctx, udev->out_ctx, 0);

    struct xhci_input_control_ctx *ctrl_ctx = xhci_get_input_control_ctx(udev->in_ctx);
    if (!ctrl_ctx)
    {
        Kprintf("xhci_set_interface: missing input control context\n");
        iface->active_altsetting = current_alt;
        return UHIOERR_HOSTERROR;
    }

    int err = UHIOERR_NO_ERROR;
    if (new_alt->no_of_ep > 0)
    {
        err = xhci_init_ep_contexts_if(udev, ctrl, iface);
        if (err != UHIOERR_NO_ERROR)
        {
            Kprintf("xhci_set_interface: failed to init ep contexts (err=%ld)\n", (LONG)err);
            iface->active_altsetting = current_alt;
            return err;
        }
    }

    u32 add_flags = SLOT_FLAG | add_mask;
    u32 drop_flags = drop_mask;
    ctrl_ctx->add_flags = LE32(add_flags);
    ctrl_ctx->drop_flags = LE32(drop_flags);

    unsigned int max_ep_flag = compute_max_ep_flag(cfg);
    xhci_update_slot_last_ctx(ctrl, udev, max_ep_flag);

    KprintfH("xhci_set_interface: updating device context for addr=%ld iface=%ld alt=%ld drop=0x%lx add=0x%lx\n",
             (LONG)udev->poseidon_address,
             (LONG)iface_number,
             (LONG)alt_setting,
             (ULONG)drop_mask,
             (ULONG)add_mask);

    xhci_configure_endpoints(udev, FALSE, NULL);

    return err;
}

static const char *slot_state_name(ULONG state)
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

static const char *slot_speed_name(ULONG dev_info)
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

static const char *ep_state_name(ULONG state)
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

static const char *ep_type_name(ULONG type)
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
    xhci_inval_cache(slot_ctx, sizeof(struct xhci_slot_ctx));
    const char *pfx = tag ? tag : "";

    if (!slot_ctx)
    {
        Kprintf("%s Slot context: (null)\n", pfx);
        return;
    }

    ULONG dev_info = LE32(slot_ctx->dev_info);
    ULONG dev_info2 = LE32(slot_ctx->dev_info2);
    ULONG tt_info = LE32(slot_ctx->tt_info);
    ULONG dev_state = LE32(slot_ctx->dev_state);

    ULONG route = dev_info & ROUTE_STRING_MASK;
    ULONG last_ctx = (dev_info & LAST_CTX_MASK) >> 27;
    LONG last_ep = (last_ctx == 0) ? -1 : ((LONG)last_ctx - 1);

    ULONG max_exit_latency = dev_info2 & MAX_EXIT;
    ULONG root_port = DEVINFO_TO_ROOT_HUB_PORT(dev_info2);
    ULONG max_ports = (dev_info2 >> 24) & 0xff;

    ULONG tt_slot = tt_info & 0xff;
    ULONG tt_port = (tt_info >> 8) & 0xff;
    ULONG tt_think_code = (tt_info >> 16) & 0x3;
    ULONG tt_think_bits = (tt_think_code + 1) * 8;

    ULONG address = dev_state & DEV_ADDR_MASK;
    ULONG slot_state = GET_SLOT_STATE(dev_state);
    Kprintf("%s Slot context @%lx\n", pfx, (ULONG)slot_ctx);
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

void xhci_dump_ep_ctx(const char *tag, struct usb_device *udev, UBYTE ep_index)
{
    struct xhci_ep_ctx *ep_ctx = xhci_get_ep_ctx(udev->controller, udev->out_ctx, ep_index);
    xhci_inval_cache(ep_ctx, sizeof(struct xhci_ep_ctx));
    const char *pfx = tag ? tag : "";

    if (!ep_ctx)
    {
        Kprintf("%s Endpoint context[%lu]: (null)\n", pfx, (ULONG)ep_index);
        return;
    }

    ULONG ep_info = LE32(ep_ctx->ep_info);
    ULONG ep_info2 = LE32(ep_ctx->ep_info2);
    u64 deq = LE64(ep_ctx->deq);
    ULONG tx_info = LE32(ep_ctx->tx_info);

    ULONG state = ep_info & EP_STATE_MASK;
    ULONG mult = CTX_TO_EP_MULT(ep_info);
    ULONG mult_count = mult + 1;
    ULONG interval = CTX_TO_EP_INTERVAL(ep_info);
    ULONG max_ps = (ep_info >> 10) & 0x1f;
    LONG has_lsa = (ep_info & EP_HAS_LSA) != 0;

    ULONG ep_type = CTX_TO_EP_TYPE(ep_info2);
    ULONG error_count = (ep_info2 >> 1) & 0x3;
    ULONG max_burst = CTX_TO_MAX_BURST(ep_info2);
    ULONG max_packet = MAX_PACKET_DECODED(ep_info2);
    LONG force_event = (ep_info2 & FORCE_EVENT) != 0;

    ULONG esit_lo = (tx_info >> 16) & 0xffff;
    ULONG esit_hi = (tx_info >> 24) & 0xff;
    ULONG max_esit_payload = esit_lo | (esit_hi << 16);
    ULONG avg_trb_len = EP_AVG_TRB_LENGTH(tx_info);

    ULONG deq_low = (ULONG)(deq & 0xffffffffUL);
    ULONG deq_high = (ULONG)((deq >> 32) & 0xffffffffUL);
    LONG cycle_state = (deq & EP_CTX_CYCLE_MASK) != 0;

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
