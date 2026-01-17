// SPDX-License-Identifier: GPL-2.0+

#include <compat.h>
#include <debug.h>

#include <xhci/usb.h>
#include <xhci/xhci.h>
#include <xhci/xhci-debug.h>

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

static const char *prefix_or_empty(const char *tag)
{
    return tag ? tag : "";
}

void xhci_dump_slot_ctx(const char *tag, const struct xhci_slot_ctx *slot_ctx)
{
    xhci_inval_cache((uintptr_t)slot_ctx, sizeof(struct xhci_slot_ctx));
    const char *pfx = prefix_or_empty(tag);

    if (!slot_ctx)
    {
        Kprintf("%s Slot context: (null)\n", pfx);
        return;
    }

    u32 dev_info = LE32(slot_ctx->dev_info);
    u32 dev_info2 = LE32(slot_ctx->dev_info2);
    u32 tt_info = LE32(slot_ctx->tt_info);
    u32 dev_state = LE32(slot_ctx->dev_state);

    unsigned int route = dev_info & ROUTE_STRING_MASK;
    unsigned int last_ctx = (dev_info & LAST_CTX_MASK) >> 27;
    int last_ep = (last_ctx == 0) ? -1 : ((int)last_ctx - 1);

    unsigned int max_exit_latency = dev_info2 & MAX_EXIT;
    unsigned int root_port = DEVINFO_TO_ROOT_HUB_PORT(dev_info2);
    unsigned int max_ports = (dev_info2 >> 24) & 0xff;

    unsigned int tt_slot = tt_info & 0xff;
    unsigned int tt_port = (tt_info >> 8) & 0xff;
    unsigned int tt_think_code = (tt_info >> 16) & 0x3;
    unsigned int tt_think_bits = (tt_think_code + 1) * 8;

    unsigned int address = dev_state & DEV_ADDR_MASK;
    unsigned int slot_state = GET_SLOT_STATE(dev_state);

    Kprintf("%s Slot context @%lx\n", pfx, (unsigned long)slot_ctx);
    Kprintf("%s  dev_info=0x%08lx route=0x%05lx speed=%s hub=%ld mtt=%ld last_ctx=%ld (last_ep=%ld)\n",
            pfx,
            (unsigned long)dev_info,
            (unsigned long)route,
            slot_speed_name(dev_info),
            (long)((dev_info & DEV_HUB) != 0),
            (long)((dev_info & DEV_MTT) != 0),
            last_ctx,
            last_ep);
    Kprintf("%s  dev_info2=0x%08lx max_exit_latency=%lu root_port=%lu max_ports=%lu\n",
            pfx,
            (unsigned long)dev_info2,
            max_exit_latency,
            root_port,
            max_ports);
    Kprintf("%s  tt_info=0x%08lx slot=%lu port=%lu think_time_code=%lu (%lu bit-times)\n",
            pfx,
            (unsigned long)tt_info,
            tt_slot,
            tt_port,
            tt_think_code,
            tt_think_bits);
    Kprintf("%s  dev_state=0x%08lx xhci_address=%lu state=%s(%lu)\n",
            pfx,
            (unsigned long)dev_state,
            address,
            slot_state_name(slot_state),
            slot_state);
}

void xhci_dump_ep_ctx(const char *tag,
                      unsigned int ep_index,
                      const struct xhci_ep_ctx *ep_ctx)
{
    xhci_inval_cache((uintptr_t)ep_ctx, sizeof(struct xhci_ep_ctx));
    const char *pfx = prefix_or_empty(tag);

    if (!ep_ctx)
    {
        Kprintf("%s Endpoint context[%ld]: (null)\n", pfx, ep_index);
        return;
    }

    u32 ep_info = LE32(ep_ctx->ep_info);
    u32 ep_info2 = LE32(ep_ctx->ep_info2);
    u64 deq = LE64(ep_ctx->deq);
    u32 tx_info = LE32(ep_ctx->tx_info);

    unsigned int state = ep_info & EP_STATE_MASK;
    unsigned int mult = CTX_TO_EP_MULT(ep_info);
    unsigned int mult_count = mult + 1;
    unsigned int interval = CTX_TO_EP_INTERVAL(ep_info);
    unsigned int max_ps = (ep_info >> 10) & 0x1f;
    int has_lsa = (ep_info & EP_HAS_LSA) != 0;

    unsigned int ep_type = CTX_TO_EP_TYPE(ep_info2);
    unsigned int error_count = (ep_info2 >> 1) & 0x3;
    unsigned int max_burst = CTX_TO_MAX_BURST(ep_info2);
    unsigned int max_packet = MAX_PACKET_DECODED(ep_info2);
    int force_event = (ep_info2 & FORCE_EVENT) != 0;

    unsigned int esit_lo = (tx_info >> 16) & 0xffff;
    unsigned int esit_hi = (tx_info >> 24) & 0xff;
    unsigned int max_esit_payload = esit_lo | (esit_hi << 16);
    unsigned int avg_trb_len = EP_AVG_TRB_LENGTH(tx_info);

    unsigned long deq_low = (unsigned long)(deq & 0xffffffffUL);
    unsigned long deq_high = (unsigned long)((deq >> 32) & 0xffffffffUL);
    int cycle_state = (deq & EP_CTX_CYCLE_MASK) != 0;

    Kprintf("%s Endpoint context[%lu] @%lx\n", pfx, ep_index, (unsigned long)ep_ctx);
    Kprintf("%s  ep_info=0x%08lx state=%s(%lu) mult=%lu (%lu per uframe) interval=%lu streams=%lu lsa=%ld\n",
            pfx,
            (unsigned long)ep_info,
            ep_state_name(state),
            state,
            mult,
            mult_count,
            interval,
            max_ps,
            (long)has_lsa);
    Kprintf("%s  ep_info2=0x%08lx type=%s(%lu) max_packet=%lu max_burst=%lu error_count=%lu force_event=%ld\n",
            pfx,
            (unsigned long)ep_info2,
            ep_type_name(ep_type),
            ep_type,
            max_packet,
            max_burst,
            error_count,
            (long)force_event);
    if (deq_high)
        Kprintf("%s  deq=0x%08lx%08lx cycle=%ld\n",
                pfx,
                deq_high,
                deq_low,
                (long)cycle_state);
    else
        Kprintf("%s  deq=0x%08lx cycle=%ld\n",
                pfx,
                deq_low,
                (long)cycle_state);
    Kprintf("%s  tx_info=0x%08lx avg_trb_len=%lu max_esit_payload=%lu\n",
            pfx,
            (unsigned long)tx_info,
            avg_trb_len,
            max_esit_payload);
}
