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
#include <xhci/xhci-root-hub.h>
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
    ctx->size = (USB_MAX_ENDPOINT_CONTEXTS + 1) * CTX_SIZE(mmio_read32(&ctrl->hccr->cr_hccparams1));
    if (type == XHCI_CTX_TYPE_INPUT)
        ctx->size += CTX_SIZE(mmio_read32(&ctrl->hccr->cr_hccparams1));

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

    return (struct xhci_slot_ctx *)(ctx->bytes + CTX_SIZE(mmio_read32(&ctrl->hccr->cr_hccparams1)));
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

    return (struct xhci_ep_ctx *)(ctx->bytes + (ep_index * CTX_SIZE(mmio_read32(&ctrl->hccr->cr_hccparams1))));
}

u32 xhci_get_hardware_address(struct usb_device *udev)
{
    struct xhci_slot_ctx *slot_ctx = xhci_get_slot_ctx(udev->controller, udev->out_ctx);
    xhci_inval_cache(slot_ctx, sizeof(struct xhci_slot_ctx));
    return le32(slot_ctx->dev_state) & DEV_ADDR_MASK;
}

u64 xhci_get_endpoint_deq_ptr(struct usb_device *udev, u8 ep_index)
{
    struct xhci_ep_ctx *ep_ctx = xhci_get_ep_ctx(udev->controller, udev->out_ctx, ep_index);
    xhci_inval_cache(ep_ctx, sizeof(struct xhci_ep_ctx));
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

    struct usb_device *parent = udev->parent;
    if (!parent || !parent->parent)
    {
        /* root hub and the first tier hub don't need routing */
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

static u32 find_root_port(struct usb_device *udev)
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
    if (!hub->is_hub || hub->speed != USB_SPEED_HIGH || hub->device_protocol != 2)
        return FALSE;

    struct usb_config *cfg = hub->active_config;
    if (!cfg)
        return FALSE;

    for (u8 i = 0; i < cfg->no_of_if; ++i)
    {
        struct usb_interface_altsetting *alt = cfg->if_desc[i].active_altsetting;
        if (alt && alt->desc.bInterfaceClass == USB_CLASS_HUB)
            return alt->desc.bInterfaceProtocol == 2;
    }
    return FALSE;
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
    KprintfH("Setting up addressable virtual device addr=%lu parent_addr=%lu parent_port=%lu\n",
             (ULONG)udev->virtual_address,
             (ULONG)(udev->parent ? udev->parent->virtual_address : 0),
             (ULONG)udev->parent_port);
    build_route_string(udev);

    /* Extract the EP0 and Slot Ctrl */
    struct xhci_ep_ctx *ep0_ctx = xhci_get_ep_ctx(ctrl, udev->in_ctx, 0);
    struct xhci_slot_ctx *slot_ctx = xhci_get_slot_ctx(ctrl, udev->in_ctx);
    KprintfH("slot=%lu in_ctx=%lx out_ctx=%lx ep0_ctx=%lx slot_ctx=%lx\n",
             (ULONG)udev->slot_id, (ULONG)udev->in_ctx, (ULONG)udev->out_ctx,
             (ULONG)ep0_ctx, (ULONG)slot_ctx);

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

    // Find root hub port number
    u32 root_port = find_root_port(udev);

    KprintfH("xhci_setup_addressable_virt_dev: parent_addr=%lu port_num=%lu root_port_num=%lu speed=%lu route=%lx route_depth=%lu\n",
             (ULONG)udev->parent->virtual_address, (ULONG)udev->parent_port, (ULONG)root_port, (ULONG)udev->speed, (ULONG)udev->route, (ULONG)udev->route_depth);

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

                /* xHCI 6.2.2: a LS/FS device behind a multi-TT hub mirrors
                 * the hub's enabled MTT mode.  Safe at address time: the hub
                 * class selects the TT mode before powering ports. */
                if (xhci_hub_multi_tt_enabled(tt_hub))
                {
                    dev_info |= DEV_MTT;
                    slot_ctx->dev_info = le32(dev_info);
                }

                KprintfH("xhci_setup_addressable_virt_dev: tt_slot=%lu tt_port=%lu tt_info=%08lx mtt=%ld\n",
                         (ULONG)tt_hub->slot_id, (ULONG)parent_port, (ULONG)tt_info,
                         (LONG)((dev_info & DEV_MTT) != 0));
                break;
            }
            parent_port = tt_hub->parent_port;
            tt_hub = tt_hub->parent;
        }

        if (!tt_hub)
            Kprintf("Low or full speed device addr %lu not behind a high-speed hub???\n", (ULONG)udev->virtual_address);
    }

    /* TODO for SS/SSP if connected by higher rank hub:
     * - TT_SLOT shoud contain slot id of the parent hub
     * - TT_PORT should be the port number on the parent hub that this device is connected to
     */

    slot_ctx->tt_info = le32(tt_info);

    /* Step 4 - ring already allocated */
    /* Step 5 */
    ep0_ctx->ep_info2 = le32(EP_TYPE(CTRL_EP));
    KprintfH("xhci_setup_addressable_virt_dev: SPEED=%lu\n", (ULONG)udev->speed);

    u32 max_packet_size = 0;
    switch (udev->speed)
    {
    case USB_SPEED_SUPER:
    case USB_SPEED_SUPER_PLUS:
        ep0_ctx->ep_info2 |= le32(MAX_PACKET(512));
        max_packet_size = 512;
        KprintfH("xhci_setup_addressable_virt_dev: MPS=512\n");
        break;
    case USB_SPEED_HIGH:
    /* USB core guesses at a 64-byte max packet first for FS devices */
    case USB_SPEED_FULL:
        ep0_ctx->ep_info2 |= le32(MAX_PACKET(64));
        max_packet_size = 64;
        KprintfH("xhci_setup_addressable_virt_dev: MPS=64\n");
        break;
    case USB_SPEED_LOW:
        ep0_ctx->ep_info2 |= le32(MAX_PACKET(8));
        max_packet_size = 8;
        KprintfH("xhci_setup_addressable_virt_dev: MPS=8\n");
        break;
    default:
        /* New speed? */
        Kprintf("Unknown device speed %lu\n", (ULONG)udev->speed);
    }

    /* EP 0 can handle "burst" sizes of 1, so Max Burst Size field is 0 */
    ep0_ctx->ep_info2 |= le32(MAX_BURST(0) | ERROR_COUNT(3));

    BOOL result = xhci_ep_create_context(udev, 0, max_packet_size, /*max_burst*/ 0);
    if (!result)
        Kprintf("Failed to create EP0 context\n");

    struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, 0);
    struct xhci_ring *ring = xhci_ep_get_ring(ep_ctx);
    ep0_ctx->deq = le64(xhci_ring_get_new_dequeue_ptr(ring));

    /*
     * xHCI spec 6.2.3:
     * software shall set 'Average TRB Length' to 8 for control endpoints.
     */
    ep0_ctx->tx_info = le32(EP_AVG_TRB_LENGTH(8));

    /* Steps 7 and 8 were done in xhci_alloc_virt_device() */

    xhci_flush_cache(ep0_ctx, sizeof(struct xhci_ep_ctx), 0);
    xhci_flush_cache(slot_ctx, sizeof(struct xhci_slot_ctx), 0);
    KprintfH("xhci_setup_addressable_virt_dev: ep0 deq=%lx tx_info=%08lx dev_info=%08lx dev_info2=%08lx\n",
             (ULONG)le64(ep0_ctx->deq), (ULONG)le32(ep0_ctx->tx_info),
             (ULONG)le32(slot_ctx->dev_info), (ULONG)le32(slot_ctx->dev_info2));

    struct xhci_input_control_ctx *ctrl_ctx = xhci_get_input_control_ctx(udev->in_ctx);
    ctrl_ctx->add_flags = le32(SLOT_FLAG | EP0_FLAG);
    ctrl_ctx->drop_flags = 0;

    xhci_flush_cache(ctrl_ctx, sizeof(struct xhci_input_control_ctx), 0);
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

/*
 * Full speed devices may have a max packet size greater than 8 bytes, but the
 * USB core doesn't know that until it reads the first 8 bytes of the
 * descriptor.  If the usb_device's max packet size changes after that point,
 * we need to issue an evaluate context command and wait on it.
 *
 * @param udev	pointer to the Device Data Structure
 * Return: returns the status of the xhci_configure_endpoints
 */
void xhci_update_maxpacket(struct usb_device *udev, u16 max_packet_size)
{
    struct xhci_ctrl *ctrl = udev->controller;
    u8 ep_index = 0; /* control endpoint */

    xhci_inval_cache(udev->out_ctx->bytes, udev->out_ctx->size);
    KprintfH("Checking max packet size for ep 0 of address %lu (slot %lu)\n", (ULONG)udev->virtual_address, (ULONG)udev->slot_id);

    struct xhci_ep_ctx *ep_ctx = xhci_get_ep_ctx(ctrl, udev->out_ctx, ep_index);
    u16 hw_max_packet_size = (u16)MAX_PACKET_DECODED(le32(ep_ctx->ep_info2));

    if (hw_max_packet_size == max_packet_size)
    {
        KprintfH("Max Packet Size for ep 0 is already correct at %lu.\n", (ULONG)max_packet_size);
        return;
    }

    KprintfH("Max Packet Size for ep 0 changed to %lu.\n", (ULONG)max_packet_size);
    KprintfH("Max packet size in xHCI HW = %lu\n", (ULONG)hw_max_packet_size);

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
static s8 xhci_init_ep_contexts_if(struct usb_device *udev, struct usb_interface *ifdesc)
{
    struct xhci_ctrl *ctrl = udev->controller;
    KprintfH("xhci_init_ep_contexts_if: enter\n");
    struct xhci_ep_ctx *ep_ctx[USB_MAX_ENDPOINT_CONTEXTS];
    u8 cur_ep;
    u8 ep_index;
    u8 dir;
    u8 ep_type;
    u32 max_esit_payload;
    u8 interval;
    u8 mult;
    u8 max_burst;
    u32 avg_trb_len;
    u8 err_count = 0;
    struct usb_interface_altsetting *active_alt = ifdesc->active_altsetting;
    if (!active_alt)
    {
        Kprintf("xhci_init_ep_contexts_if: no active altsetting for iface %lu\n",
                (ULONG)ifdesc->interface_number);
        return ERR_BAD_PARAMETERS;
    }

    u8 num_of_ep = active_alt->no_of_ep;

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

        /* VL805 corrupts SS bulk OUT bursts for mass-storage devices behind a
         * hub (Linux XHCI_VLI_SS_BULK_OUT_BUG, xhci-mem.c). */
        if ((ctrl->quirks & XHCI_QUIRK_SS_BULK_OUT) && max_burst != 0 &&
            udev->speed >= USB_SPEED_SUPER && udev->route != 0 &&
            usb_endpoint_xfer_bulk(endpt_desc) && usb_endpoint_dir_out(endpt_desc) &&
            ifdesc->altsetting[0].desc.bInterfaceClass == USB_CLASS_MASS_STORAGE)
        {
            Kprintf("VL805 quirk: max_burst %lu -> 0 for addr %lu ep 0x%02lx (SS bulk OUT, UMS behind hub)\n",
                    (ULONG)max_burst, (ULONG)udev->virtual_address,
                    (ULONG)endpt_desc->bEndpointAddress);
            max_burst = 0;
        }

        avg_trb_len = max_esit_payload;

        ep_index = xhci_get_ep_index(endpt_desc);
        ep_ctx[ep_index] = xhci_get_ep_ctx(ctrl, udev->in_ctx, ep_index);

        u16 max_packet_size = usb_endpoint_maxp(endpt_desc);
        /* Allocate the ep rings */
        BOOL result = xhci_ep_create_context(udev, ep_index, max_packet_size, max_burst);
        if (!result)
            return ERR_ALLOC_ERROR;

        /*NOTE: ep_desc[0] actually represents EP1 and so on */
        dir = (((endpt_desc->bEndpointAddress) & (0x80)) >> 7);
        ep_type = ((endpt_desc->bmAttributes & 0x3U) | ((u32)dir << 2u)) & 0xffU;

        ep_ctx[ep_index]->ep_info = le32(EP_MAX_ESIT_PAYLOAD_HI(max_esit_payload) | EP_INTERVAL(interval) | EP_MULT(mult));

        ep_ctx[ep_index]->ep_info2 = le32(EP_TYPE(ep_type));
        ep_ctx[ep_index]->ep_info2 |= le32(MAX_PACKET(max_packet_size));

        /* Allow 3 retries for everything but isoc, set CErr = 3 */
        if (!usb_endpoint_xfer_isoc(endpt_desc))
            err_count = 3;
        ep_ctx[ep_index]->ep_info2 |= le32(MAX_BURST(max_burst) | ERROR_COUNT(err_count));

        struct ep_context *ep_context = xhci_ep_get_context_for_index(udev, ep_index);
        struct xhci_ring *ring = xhci_ep_get_ring(ep_context);
        ep_ctx[ep_index]->deq = le64(xhci_ring_get_new_dequeue_ptr(ring));

        xhci_ep_set_rt_interval(ep_context, interval);

        /*
         * xHCI spec 6.2.3:
         * 'Average TRB Length' should be 8 for control endpoints.
         */
        if (usb_endpoint_xfer_control(endpt_desc))
            avg_trb_len = 8;
        ep_ctx[ep_index]->tx_info = le32(EP_MAX_ESIT_PAYLOAD_LO(max_esit_payload) | EP_AVG_TRB_LENGTH(avg_trb_len));

        KprintfH("EP%lu %s: type=%lu maxp=%lu maxesit=%lu "
                 "interval=%lu mult=%lu maxburst=%lu\n",
                 (ULONG)usb_endpoint_num(endpt_desc),
                 dir ? "IN" : "OUT",
                 (ULONG)ep_type,
                 (ULONG)usb_endpoint_maxp(endpt_desc),
                 (ULONG)max_esit_payload,
                 (ULONG)interval,
                 (ULONG)(mult + 1),
                 (ULONG)(max_burst + 1));
    }

    return ERR_NO_ERROR;
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
    xhci_inval_cache(ep_ctx, sizeof(*ep_ctx));
    return le32(ep_ctx->ep_info) & EP_STATE_MASK;
}

/* Writes the current udev->max_exit_latency_us into MAX_EXIT in the input slot context. */
void xhci_update_mel_in_input_ctx(struct usb_device *udev)
{
    u32 mel = udev->max_exit_latency_us > 0xffffU ? 0xffffU : udev->max_exit_latency_us;
    struct xhci_slot_ctx *slot = xhci_get_slot_ctx(udev->controller, udev->in_ctx);
    u32 d2 = le32(slot->dev_info2);
    d2 = (d2 & ~MAX_EXIT) | mel;
    slot->dev_info2 = le32(d2);
}

static u32 xhci_calculate_mel(struct usb_device *udev);

/* Compute the device's MEL and, when non-zero, write it into the input slot
 * context.  Shared by xhci_set_configuration() and xhci_set_interface(). */
static void xhci_compute_and_apply_mel(struct usb_device *udev)
{
    udev->max_exit_latency_us = xhci_calculate_mel(udev);
    if (udev->max_exit_latency_us > 0)
        xhci_update_mel_in_input_ctx(udev);
}

/**
 * Configure the endpoint, programming the device contexts.
 *
 * @param udev	pointer to the USB device structure
 * Return: returns the status of the xhci_configure_endpoints
 */
s8 xhci_set_configuration(struct usb_device *udev, u32 config_value)
{
    KprintfH("xhci_set_configuration: config_val=%lu\n", (ULONG)config_value);

    struct usb_config *cfg = xhci_find_config(udev, (int)config_value);
    if (!cfg)
    {
        Kprintf("xhci_set_configuration: config_val=%lu not found!\n", (ULONG)config_value);
        return ERR_BAD_PARAMETERS;
    }

    udev->active_config = cfg;
    for (u8 i = 0; i < cfg->no_of_if; ++i)
        xhci_select_active_alt(&cfg->if_desc[i]);

#ifdef DEBUG_HIGH
    /* Dump entire cfg using kprintf (all fields and all interfaces and endpoints) */
    xhci_dump_config("[xhci] xhci_set_configuration:", cfg, udev->virtual_address);
#endif

    struct xhci_ctrl *ctrl = udev->controller;
    u8 max_ifnum = cfg->no_of_if;
    u32 max_ep_flag = 0;

    struct xhci_container_ctx *out_ctx = udev->out_ctx;
    struct xhci_container_ctx *in_ctx = udev->in_ctx;

    struct xhci_input_control_ctx *ctrl_ctx = xhci_get_input_control_ctx(in_ctx);
    u32 add_flags = SLOT_FLAG;
    u32 mask = xhci_collect_config_masks(cfg, max_ifnum, &max_ep_flag);
    add_flags |= mask;
    ctrl_ctx->add_flags = le32(add_flags);
    ctrl_ctx->drop_flags = 0;

    xhci_inval_cache(out_ctx->bytes, out_ctx->size);

    /* slot context */
    xhci_slot_copy(ctrl, in_ctx, out_ctx);
    xhci_compute_and_apply_mel(udev);
    xhci_update_slot_last_ctx(ctrl, udev, max_ep_flag);

    xhci_endpoint_copy(ctrl, in_ctx, out_ctx, 0);

    /* update slot context hub stuff */
    xhci_update_hub_tt(udev, in_ctx);

    /* filling up ep contexts */
    for (u8 ifnum = 0; ifnum < max_ifnum; ++ifnum)
    {
        struct usb_interface *ifdesc = &cfg->if_desc[ifnum];
        s8 err = xhci_init_ep_contexts_if(udev, ifdesc);
        if (err != ERR_NO_ERROR)
        {
            return err;
        }
    }
    return ERR_NO_ERROR;
}

s8 xhci_set_interface(struct usb_device *udev, u8 iface_number, u8 alt_setting)
{
    if (!udev || !udev->controller)
    {
        Kprintf("xhci_set_interface: invalid usb_device pointer\n");
        return ERR_BAD_PARAMETERS;
    }

    struct usb_config *cfg = udev->active_config;
    if (!cfg)
    {
        Kprintf("xhci_set_interface: no active config for addr %lu\n", (ULONG)udev->virtual_address);
        return ERR_BAD_PARAMETERS;
    }

    struct usb_interface *iface = xhci_find_interface(cfg, iface_number);
    if (!iface)
    {
        Kprintf("xhci_set_interface: interface %lu not found in config %lu\n",
                (ULONG)iface_number, (ULONG)cfg->desc.bConfigurationValue);
        return ERR_BAD_PARAMETERS;
    }

    struct usb_interface_altsetting *current_alt = iface->active_altsetting;

    if (current_alt && current_alt->desc.bAlternateSetting == alt_setting)
    {
        KprintfH("xhci_set_interface: iface %lu already at alt %lu\n",
                 (ULONG)iface_number, (ULONG)alt_setting);
        return ERR_NO_ERROR;
    }

    struct usb_interface_altsetting *new_alt = xhci_find_altsetting(iface, alt_setting);
    if (!new_alt)
    {
        Kprintf("xhci_set_interface: alt %lu missing for iface %lu\n",
                (ULONG)alt_setting, (ULONG)iface_number);
        return ERR_BAD_PARAMETERS;
    }

    struct xhci_ctrl *ctrl = udev->controller;

    /* The stack must guarantee endpoint queues are idle before switching. */
    u32 drop_mask = xhci_collect_ep_mask(current_alt, NULL);

    iface->active_altsetting = new_alt;

    u32 add_mask = xhci_collect_ep_mask(new_alt, NULL);
    xhci_inval_cache(udev->out_ctx->bytes, udev->out_ctx->size);
    xhci_slot_copy(ctrl, udev->in_ctx, udev->out_ctx);
    xhci_compute_and_apply_mel(udev);
    /* Hub TT mode (MTT) is selected via SET_INTERFACE - refresh hub fields */
    xhci_update_hub_tt(udev, udev->in_ctx);
    xhci_endpoint_copy(ctrl, udev->in_ctx, udev->out_ctx, 0);

    struct xhci_input_control_ctx *ctrl_ctx = xhci_get_input_control_ctx(udev->in_ctx);
    if (!ctrl_ctx)
    {
        Kprintf("xhci_set_interface: missing input control context\n");
        iface->active_altsetting = current_alt;
        return ERR_HCI_ERROR;
    }

    s8 err = ERR_NO_ERROR;
    if (new_alt->no_of_ep > 0)
    {
        err = xhci_init_ep_contexts_if(udev, iface);
        if (err != ERR_NO_ERROR)
        {
            Kprintf("xhci_set_interface: failed to init ep contexts (err=%ld)\n", (LONG)err);
            iface->active_altsetting = current_alt;
            return err;
        }
    }

    u32 add_flags = SLOT_FLAG | add_mask;
    u32 drop_flags = drop_mask;
    ctrl_ctx->add_flags = le32(add_flags);
    ctrl_ctx->drop_flags = le32(drop_flags);

    u32 max_ep_flag = compute_max_ep_flag(cfg);
    xhci_update_slot_last_ctx(ctrl, udev, max_ep_flag);

    KprintfH("xhci_set_interface: updating device context for addr=%lu iface=%lu alt=%lu drop=0x%lx add=0x%lx\n",
             (ULONG)udev->virtual_address,
             (ULONG)iface_number,
             (ULONG)alt_setting,
             (ULONG)drop_mask,
             (ULONG)add_mask);

    xhci_configure_endpoints(udev, FALSE, NULL);

    return err;
}

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
    xhci_inval_cache(slot_ctx, sizeof(struct xhci_slot_ctx));
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

/* BESL selector (0-15) to microseconds — USB 2.0 LPM ECN Table; identical to
 * Linux xhci_besl_encoding[]. */
static const u32 besl_encoding[16] = {
    125, 150, 200, 300, 400, 500, 1000, 2000,
    3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000};

static inline u32 u32max(u32 a, u32 b) { return a > b ? a : b; }

/* TRUE if any active periodic (int/isoc) endpoint has a service interval <= the
 * given MEL (ns), in which case that link state must not be enabled (xHCI
 * 4.23.5.2; mirrors xhci_calculate_u1/u2_timeout ESIT guard). */
static BOOL xhci_lpm_esit_blocks(struct usb_device *udev, u32 mel_ns)
{
    struct usb_config *cfg = udev->active_config;
    if (!cfg)
        return FALSE;

    for (u8 i = 0; i < cfg->no_of_if; i++)
    {
        struct usb_interface_altsetting *alt = cfg->if_desc[i].active_altsetting;
        if (!alt)
            continue;
        for (u8 j = 0; j < alt->no_of_ep; j++)
        {
            struct usb_endpoint_descriptor *ep = &alt->ep_desc[j];
            if (!usb_endpoint_xfer_int(ep) && !usb_endpoint_xfer_isoc(ep))
                continue;
            u8 bi = ep->bInterval;
            if (bi == 0)
                continue;
            if (bi > 16) /* SS bInterval is 1..16; clamp malformed values */
                bi = 16;
            /* SS service interval = 2^(bInterval-1) microframes * 125us.
             * Compare as uframes vs mel/125000 to avoid 64-bit math. */
            u32 uframes = 1u << (bi - 1);
            if (uframes <= mel_ns / 125000u)
                return TRUE;
        }
    }
    return FALSE;
}

/* Returns the hub-encoded U1/U2 timeout (generic, non-Intel host path: the
 * timeout equals SEL), or USB3_LPM_DISABLED if the state should not be enabled.
 * Mirrors xhci_calculate_u1_timeout / xhci_calculate_u2_timeout. */
static u16 xhci_usb3_state_timeout(struct usb_device *udev, BOOL u2)
{
    u32 dev_exit = u2 ? (u32)udev->u2_dev_exit_lat : (u32)udev->u1_dev_exit_lat;
    if (dev_exit == 0)
        return USB3_LPM_DISABLED; /* device doesn't implement this link state */

    u32 mel_ns = u2 ? udev->u2_mel : udev->u1_mel;
    if (xhci_lpm_esit_blocks(udev, mel_ns))
        return USB3_LPM_DISABLED;

    u32 sel_ns = u2 ? udev->u2_sel : udev->u1_sel;
    u32 timeout;
    if (u2)
    {
        timeout = (sel_ns + (256u * 1000u - 1u)) / (256u * 1000u); /* 256us units */
        if (timeout == 0)
            timeout = 1;
        if (timeout > USB3_LPM_U2_MAX_TIMEOUT)
            return USB3_LPM_DISABLED;
    }
    else
    {
        timeout = (sel_ns + 999u) / 1000u; /* us */
        if (timeout == 0)
            timeout = 1;
        if (timeout > USB3_LPM_U1_MAX_TIMEOUT)
            return USB3_LPM_DISABLED;
    }
    return (u16)timeout;
}

/* Compute USB3 SEL/PEL/MEL (USB 3.1 Appendix C) and detect USB2 hardware LPM
 * eligibility.  Mirrors usb_set_lpm_parameters() + xhci_update_device().  Run
 * once after the BOS descriptor has been parsed. */
void xhci_set_lpm_parameters(struct usb_device *udev)
{
    if (!udev || !udev->controller || !udev->lpm_capable || !udev->parent)
        return;
    struct xhci_ctrl *ctrl = udev->controller;

    if (udev->speed == USB_SPEED_HIGH)
    {
        /* USB2 hardware LPM: non-hub device directly on a root-hub port that
         * advertises HLC in its Supported-Protocol cap. */
        if (!udev->is_hub && udev->parent->parent == NULL)
        {
            u32 root_port = find_root_port(udev);
            BOOL hw_lpm = FALSE, besl_lpm = FALSE;
            xhci_roothub_port_lpm_caps(ctrl->root_hub, root_port, &hw_lpm, &besl_lpm);
            if (hw_lpm)
            {
                udev->usb2_hw_lpm_capable = TRUE;
                udev->usb2_hw_lpm_besl_capable = besl_lpm;
            }
        }
        return;
    }

    if (udev->speed < USB_SPEED_SUPER)
        return;

    struct usb_device *parent = udev->parent;
    BOOL is_root = (parent->parent == NULL);

    u32 udev_u1 = (u32)udev->u1_dev_exit_lat;
    u32 udev_u2 = (u32)udev->u2_dev_exit_lat;
    u32 hub_u1 = is_root ? (u32)ctrl->u1_host_exit_lat : (u32)parent->u1_dev_exit_lat;
    u32 hub_u2 = is_root ? (u32)ctrl->u2_host_exit_lat : (u32)parent->u2_dev_exit_lat;
    u32 parent_u1_mel = is_root ? 0 : parent->u1_mel;
    u32 parent_u2_mel = is_root ? 0 : parent->u2_mel;
    u32 parent_u1_pel = is_root ? 0 : parent->u1_pel;
    u32 parent_u2_pel = is_root ? 0 : parent->u2_pel;
    u32 hub_hdr_dec = is_root ? 0 : (u32)parent->ss_hub_desc.u.ss.bHubHdrDecLat;
    u32 hub_delay = is_root ? 0 : (u32)le16(parent->ss_hub_desc.u.ss.wHubDelay);

    u32 common = hub_hdr_dec * 100u + (hub_delay + USB_TP_TRANSMISSION_DELAY) * 2u + (is_root ? (USB_PING_RESPONSE_TIME + 2100u) : 0u);

    /* MEL (ns) */
    udev->u1_mel = parent_u1_mel + u32max(udev_u1, hub_u1) * 1000u + common;
    udev->u2_mel = parent_u2_mel + u32max(udev_u2, hub_u2) * 1000u + common;

    /* PEL (ns) */
    u32 u1_first = u32max(udev_u1, hub_u1) * 1000u;
    udev->u1_pel = u32max(u1_first, 1u * 1000u + parent_u1_pel); /* p2p U1 = 1us */

    u32 p2p_u2 = (hub_u2 > hub_u1) ? (1u + hub_u2 - hub_u1) : (1u + hub_u1);
    u32 u2_first = u32max(udev_u2, hub_u2) * 1000u;
    udev->u2_pel = u32max(u2_first, p2p_u2 * 1000u + parent_u2_pel);

    /* SEL (ns) */
    u32 num_hubs = 0;
    for (struct usb_device *p = udev->parent; p->parent; p = p->parent)
        num_hubs++;
    u32 sel_extra = (num_hubs > 0 ? 2100u + 250u * (num_hubs - 1u) : 0u) + 250u * num_hubs;
    udev->u1_sel = udev->u1_pel + sel_extra;
    udev->u2_sel = udev->u2_pel + sel_extra;

    KprintfH("LPM params addr %lu: U1 sel=%lu pel=%lu mel=%lu | U2 sel=%lu pel=%lu mel=%lu (ns)\n",
             (ULONG)udev->virtual_address,
             (ULONG)udev->u1_sel, (ULONG)udev->u1_pel, (ULONG)udev->u1_mel,
             (ULONG)udev->u2_sel, (ULONG)udev->u2_pel, (ULONG)udev->u2_mel);
}

static u32 xhci_calculate_mel(struct usb_device *udev)
{
    if (udev->speed < USB_SPEED_HIGH)
        return 0;

    if (udev->speed == USB_SPEED_HIGH)
    {
        /* Only BESL-capable hosts program MEL for USB2 L1 (mirror
         * xhci_set_usb2_hardware_lpm: MEL = besl_encoding[baseline]). */
        if (!udev->lpm_capable || !udev->usb2_hw_lpm_capable ||
            !udev->usb2_hw_lpm_besl_capable)
            return 0;
        u8 besl = udev->besl_baseline_valid ? udev->besl_baseline : XHCI_DEFAULT_BESL;
        return besl_encoding[besl & 0xfU];
    }

    /* USB3: MEL is the max over the states that will actually be enabled. */
    u32 mel_ns = 0;
    if (xhci_usb3_state_timeout(udev, FALSE) != USB3_LPM_DISABLED)
        mel_ns = u32max(mel_ns, udev->u1_mel);
    if (xhci_usb3_state_timeout(udev, TRUE) != USB3_LPM_DISABLED)
        mel_ns = u32max(mel_ns, udev->u2_mel);

    u32 mel_us = (mel_ns + 999u) / 1000u;
    if (mel_us > 0xffffU)
        mel_us = 0xffffU;
    KprintfH("Calculated MEL for addr %lu: %lu us (u1_mel=%lu u2_mel=%lu ns)\n",
             (ULONG)udev->virtual_address, (ULONG)mel_us,
             (ULONG)udev->u1_mel, (ULONG)udev->u2_mel);
    return mel_us;
}

/* HIRD/BESL value for USB2 PORTPMSC (mirror xhci_calculate_hird_besl). */
static u8 xhci_calculate_hird_besl(struct usb_device *udev)
{
    struct xhci_ctrl *ctrl = udev->controller;
    u32 u2del = (u32)ctrl->u2_host_exit_lat;
    u32 besl_host = 0;
    u32 besl_device = 0;

    if (udev->besl_supported)
    {
        for (besl_host = 0; besl_host < 16; besl_host++)
            if (besl_encoding[besl_host] >= u2del)
                break;
        if (udev->besl_baseline_valid)
            besl_device = udev->besl_baseline;
        else if (udev->besl_deep_valid)
            besl_device = udev->besl_deep;
    }
    else
    {
        if (u2del <= 50)
            besl_host = 0;
        else
            besl_host = (u2del - 51) / 75 + 1;
    }

    u32 besl = besl_host + besl_device;
    if (besl > 15)
        besl = 15;
    return (u8)besl;
}

/* Decide USB2 hardware-LPM (L1) policy for the device and program its root-hub
 * port.  Mirrors xhci_set_usb2_hardware_lpm(enable=1); the register writes live
 * in xhci_roothub_set_usb2_hw_lpm(). */
static void xhci_usb2_set_hw_lpm(struct usb_device *udev)
{
    struct xhci_ctrl *ctrl = udev->controller;
    if (udev->speed != USB_SPEED_HIGH || !udev->lpm_capable || !udev->usb2_hw_lpm_capable)
        return;
    if (udev->is_hub || !udev->parent || udev->parent->parent != NULL)
        return;

    BOOL besl_mode = udev->usb2_hw_lpm_besl_capable;
    u8 hird;
    u8 besld = 0;
    if (besl_mode)
    {
        hird = udev->besl_baseline_valid ? udev->besl_baseline : XHCI_DEFAULT_BESL;
        besld = udev->besl_deep_valid ? udev->besl_deep : 0;
    }
    else
    {
        hird = xhci_calculate_hird_besl(udev);
    }

    u32 root_port = find_root_port(udev);
    xhci_roothub_set_usb2_hw_lpm(ctrl->root_hub, root_port, hird, udev->slot_id,
                                 besl_mode, besld, XHCI_L1_TIMEOUT);

    KprintfH("USB2 HW LPM enabled: port %lu slot %lu hird=%lu besl_cap=%ld\n",
             (ULONG)root_port, (ULONG)udev->slot_id, (ULONG)hird, (LONG)besl_mode);
}

/* TRUE if device-initiated U1/U2 entry is allowed: every periodic endpoint's
 * service interval must absorb the system exit latency (mirror
 * usb_device_may_initiate_lpm: reject if sel + 125us > interval). */
static BOOL xhci_lpm_may_initiate(struct usb_device *udev, BOOL u2)
{
    u32 sel_us = ((u2 ? udev->u2_sel : udev->u1_sel) + 999u) / 1000u;

    struct usb_config *cfg = udev->active_config;
    if (!cfg)
        return FALSE;

    for (u8 i = 0; i < cfg->no_of_if; i++)
    {
        struct usb_interface_altsetting *alt = cfg->if_desc[i].active_altsetting;
        if (!alt)
            continue;
        for (u8 j = 0; j < alt->no_of_ep; j++)
        {
            struct usb_endpoint_descriptor *ep = &alt->ep_desc[j];
            if (!usb_endpoint_xfer_int(ep) && !usb_endpoint_xfer_isoc(ep))
                continue;
            u8 bi = ep->bInterval;
            if (bi == 0)
                continue;
            if (bi > 16)
                bi = 16;
            u32 interval_us = (1u << (bi - 1)) * 125u;
            if (sel_us + 125u > interval_us)
                return FALSE;
        }
    }
    return TRUE;
}

/* Issue an Evaluate Context carrying udev->max_exit_latency_us (mirror
 * xhci_change_max_exit_latency).  The xHC evaluates the slot's Max Exit
 * Latency only through Address Device / Evaluate Context (xHCI 6.2.2); the
 * value carried in a CONFIG_EP input context is ignored, and with an internal
 * MEL of 0 the controller will not take the link into U1/U2. */
static void xhci_evaluate_mel(struct usb_device *udev)
{
    struct xhci_ctrl *ctrl = udev->controller;

    struct xhci_input_control_ctx *ctrl_ctx = xhci_get_input_control_ctx(udev->in_ctx);
    ctrl_ctx->add_flags = le32(SLOT_FLAG);
    ctrl_ctx->drop_flags = 0;

    xhci_inval_cache(udev->out_ctx->bytes, udev->out_ctx->size);
    xhci_slot_copy(ctrl, udev->in_ctx, udev->out_ctx);
    xhci_update_mel_in_input_ctx(udev);

    struct xhci_slot_ctx *slot = xhci_get_slot_ctx(ctrl, udev->in_ctx);
    slot->dev_state = 0;

    Kprintf("Evaluate Context: MEL=%lu us for addr %lu (slot %lu)\n",
            (ULONG)udev->max_exit_latency_us, (ULONG)udev->virtual_address,
            (ULONG)udev->slot_id);
    xhci_configure_endpoints(udev, TRUE, NULL);
}

/* Orchestrate the LPM enable sequence (mirror usb_enable_lpm).  Run once, after
 * the SET_CONFIGURATION control transfer has completed on the wire: the device
 * must be in the Configured state or it rejects SET_FEATURE(U1/U2_ENABLE) with
 * a Request Error (USB 3.2 9.4.9).  The sequence continues in
 * xhci_lpm_enable_stage2() once the MEL Evaluate Context completes, and
 * device-initiated enable chains from the SET_SEL completion
 * (xhci_lpm_devinit_enable). */
void xhci_lpm_enable(struct usb_device *udev)
{
    if (!udev)
        return;

    /* LTM is independent of the U1/U2 policy (mirror usb_enable_ltm): enable
     * it for any configured SS device that advertises it, when the
     * controller consumes LTM packets (HCC_LTC). */
    if (!udev->ltm_setup_done && udev->speed >= USB_SPEED_SUPER &&
        udev->ltm_capable && udev->controller->ltc_supported)
    {
        udev->ltm_setup_done = TRUE;
        xhci_udev_set_device_ltm(udev);
    }

    if (!udev->lpm_capable || udev->lpm_setup_done)
        return;

    if (udev->speed == USB_SPEED_HIGH)
    {
        xhci_usb2_set_hw_lpm(udev);
        udev->lpm_setup_done = TRUE;
        return;
    }

    if (udev->speed < USB_SPEED_SUPER)
        return;

    udev->u1_timeout = xhci_usb3_state_timeout(udev, FALSE);
    udev->u2_timeout = xhci_usb3_state_timeout(udev, TRUE);
    udev->lpm_setup_done = TRUE;

    if (udev->u1_timeout == USB3_LPM_DISABLED && udev->u2_timeout == USB3_LPM_DISABLED)
        return;

    /* MEL must be latched by the controller before the port timeouts arm
     * (xHCI 4.23.5.2); handle_config_ep() resumes with stage 2 on success
     * (UDEV_OP_EVENT_MEL_EVAL_DONE). */
    if (!xhci_udev_op_begin(udev, UDEV_OP_LPM_ENABLE, NULL))
        return;
    xhci_evaluate_mel(udev);
}

/* Continue LPM enable after the MEL Evaluate Context succeeded: arm the parent
 * port's U1/U2 inactivity timeouts (root port PORTPMSC for a root-hub child,
 * SetPortFeature to the external hub otherwise) and send SET_SEL, whose
 * completion enables device-initiated entry.  Returns TRUE if SET_SEL was
 * submitted (the op stays alive for UDEV_OP_EVENT_SET_SEL_DONE). */
BOOL xhci_lpm_enable_stage2(struct usb_device *udev)
{
    if (!udev)
        return FALSE;

    if (udev->u1_timeout != USB3_LPM_DISABLED)
        xhci_udev_set_port_lpm_timeout(udev, FALSE, udev->u1_timeout);

    if (udev->u2_timeout != USB3_LPM_DISABLED)
        xhci_udev_set_port_lpm_timeout(udev, TRUE, udev->u2_timeout);

    return xhci_udev_send_set_sel(udev);
}

/* Enable device-initiated U1/U2 (SET_FEATURE) for the states whose port timeout
 * was enabled.  Called from the SET_SEL completion path, so the device is
 * configured and knows the exit latencies. */
void xhci_lpm_devinit_enable(struct usb_device *udev)
{
    if (!udev || udev->speed < USB_SPEED_SUPER || !udev->lpm_capable)
        return;

    if (udev->u1_timeout != USB3_LPM_DISABLED && xhci_lpm_may_initiate(udev, FALSE))
        xhci_udev_set_device_lpm(udev, FALSE);

    if (udev->u2_timeout != USB3_LPM_DISABLED && xhci_lpm_may_initiate(udev, TRUE))
        xhci_udev_set_device_lpm(udev, TRUE);
}

/* Tear down LPM when a device disconnects.  Mirrors usb_disable_device(): on a
 * physical disconnect Linux clears only USB2 hardware LPM (a direct root-port
 * register write via usb_disable_usb2_hardware_lpm()).  USB3 timeout teardown
 * short-circuits in usb_disable_lpm() because the device is already NOTATTACHED
 * (state < CONFIGURED), so we likewise leave USB3 PORTPMSC timeouts alone - they
 * are inert without a link and are overwritten on the next enumerate - and never
 * issue control transfers to a (possibly already-gone) external hub. */
void xhci_lpm_disable(struct usb_device *udev)
{
    if (!udev || !udev->lpm_setup_done)
        return;

    /* USB2 hardware LPM is only ever enabled for an HS device directly on a
     * root-hub port, so this clear is always a safe local register write. */
    if (udev->speed == USB_SPEED_HIGH && udev->usb2_hw_lpm_capable &&
        udev->parent && udev->parent->parent == NULL)
        xhci_roothub_clear_usb2_hw_lpm(udev->controller->root_hub, find_root_port(udev));

    udev->lpm_setup_done = FALSE;
}

void xhci_dump_ep_ctx(const char *tag, struct usb_device *udev, u8 ep_index)
{
    struct xhci_ep_ctx *ep_ctx = xhci_get_ep_ctx(udev->controller, udev->out_ctx, ep_index);
    xhci_inval_cache(ep_ctx, sizeof(struct xhci_ep_ctx));
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
