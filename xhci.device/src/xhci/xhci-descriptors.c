/* SPDX-License-Identifier: GPL-2.0-only */

#include <exec/types.h>
#include <xhci/ch9.h>
#include <xhci/usb_defs.h>
#include <xhci/xhci.h>
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-descriptors.h>
#include <xhci/xhci-ring.h>
#include <xhci/xhci-udev.h>
#include <xhci/xhci-context.h>

#include <debug.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-descriptors] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci-descriptors] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

/**
 * Used for passing endpoint bitmasks between the core and HCDs.
 * Find the index for an endpoint given its descriptor.
 * Use the return value to right shift 1 for the bitmask.
 *
 * Index  = (epnum * 2) + direction - 1,
 * where direction = 0 for OUT, 1 for IN.
 * For control endpoints, the IN index is used (OUT index is unused), so
 * index = (epnum * 2) + direction - 1 = (epnum * 2) + 1 - 1 = (epnum * 2)
 *
 * @param desc	USB enpdoint Descriptor
 * Return: index of the Endpoint
 */
u8 xhci_get_ep_index(struct usb_endpoint_descriptor *desc)
{
    u8 index;

    if (usb_endpoint_xfer_control(desc))
        index = (u8)(usb_endpoint_num(desc) * 2);
    else
        index = (u8)((usb_endpoint_num(desc) * 2) -
                               (usb_endpoint_dir_in(desc) ? 0 : 1));

    return index;
}

struct usb_interface *xhci_find_interface(struct usb_config *cfg, u32 iface_number)
{
    if (!cfg)
        return NULL;

    for (u32 i = 0; i < cfg->no_of_if; ++i)
    {
        if (cfg->if_desc[i].interface_number == iface_number)
            return &cfg->if_desc[i];
    }

    return NULL;
}

struct usb_interface_altsetting *xhci_find_altsetting(struct usb_interface *iface, u8 alt_setting)
{
    if (!iface)
        return NULL;

    for (u32 idx = 0; idx < iface->num_altsetting; ++idx)
    {
        struct usb_interface_altsetting *candidate = &iface->altsetting[idx];
        if (candidate->desc.bAlternateSetting == alt_setting)
            return candidate;
    }

    return NULL;
}

u32 xhci_collect_ep_mask(const struct usb_interface_altsetting *alt, u32 *max_flag)
{
    if (!alt)
        return 0;

    u32 mask = 0;

    for (u32 i = 0; i < alt->no_of_ep; ++i)
    {
        const struct usb_endpoint_descriptor *epd = &alt->ep_desc[i];
        u8 ep_index = xhci_get_ep_index((struct usb_endpoint_descriptor *)epd);
        mask |= 1U << (ep_index + 1);
        if (max_flag && ep_index > *max_flag)
            *max_flag = ep_index;
    }

    return mask;
}

u32 xhci_collect_config_masks(const struct usb_config *cfg, u32 limit, u32 *max_flag)
{
    if (!cfg)
        return 0;

    u32 mask = 0;
    u32 max_if = limit < (u32)cfg->no_of_if ? limit : (u32)cfg->no_of_if;

    for (u32 ifnum = 0; ifnum < max_if; ++ifnum)
    {
        const struct usb_interface *iface = &cfg->if_desc[ifnum];
        const struct usb_interface_altsetting *active_alt = iface->active_altsetting;
        if (!active_alt)
            continue;

        KprintfH("Preparing iface=%lu alt=%lu\n", (ULONG)ifnum, (ULONG)active_alt->desc.bAlternateSetting);

        mask |= xhci_collect_ep_mask(active_alt, max_flag);
    }

    return mask;
}

struct usb_config *xhci_find_config(struct usb_device *udev, int config_value)
{
    if (!udev)
        return NULL;

    for (struct MinNode *node = udev->configurations.mlh_Head; node->mln_Succ != NULL; node = node->mln_Succ)
    {
        struct usb_config *cfg = (struct usb_config *)node;
        KprintfH("  found config: bConfigurationValue=%lu\n", (ULONG)cfg->desc.bConfigurationValue);
        if (cfg->desc.bConfigurationValue == config_value)
            return cfg;
    }

    return NULL;
}

struct usb_interface_altsetting *xhci_select_active_alt(struct usb_interface *iface)
{
    if (!iface)
        return NULL;

    if (iface->active_altsetting)
        return iface->active_altsetting;

    if (iface->num_altsetting == 0)
        return NULL;

    struct usb_interface_altsetting *fallback = NULL;
    for (u32 idx = 0; idx < iface->num_altsetting; ++idx)
    {
        struct usb_interface_altsetting *candidate = &iface->altsetting[idx];
        if (candidate->desc.bAlternateSetting == 0)
        {
            fallback = candidate;
            break;
        }
    }

    if (!fallback)
        fallback = &iface->altsetting[0];

    iface->active_altsetting = fallback;
    return fallback;
}

u32 compute_max_ep_flag(const struct usb_config *cfg)
{
    if (!cfg)
        return 0;

    u32 max_flag = 0;
    xhci_collect_config_masks(cfg, cfg->no_of_if, &max_flag);

    return max_flag;
}

/*
 * Convert bInterval expressed in microframes (in 1-255 range) to exponent of
 * microframes, rounded down to nearest power of 2.
 */
static u8 xhci_microframes_to_exponent(u32 desc_interval,
                                       u32 min_exponent,
                                       u32 max_exponent)
{
    u32 interval = log2_floor_u64(desc_interval);
    interval = clamp_val(interval, min_exponent, max_exponent);
#ifdef DEBUG_HIGH
    if ((1U << interval) != desc_interval)
        KprintfH("rounding interval to %lu microframes, ep desc says %lu microframes\n",
                 (ULONG)(1U << interval), (ULONG)desc_interval);
#endif

    return (u8)interval;
}

static u8 xhci_parse_microframe_interval(struct usb_endpoint_descriptor *endpt_desc)
{
    if (endpt_desc->bInterval == 0)
        return 0;

    return xhci_microframes_to_exponent(endpt_desc->bInterval, 0, 15);
}

static u8 xhci_parse_frame_interval(struct usb_endpoint_descriptor *endpt_desc)
{
    return xhci_microframes_to_exponent((u32)endpt_desc->bInterval * 8U, 3, 10);
}

/*
 * Convert interval expressed as 2^(bInterval - 1) == interval into
 * straight exponent value 2^n == interval.
 */
static u8 xhci_parse_exponent_interval(struct usb_device *udev,
                                       struct usb_endpoint_descriptor *endpt_desc)
{
    u8 interval;

    interval = (u8)(clamp_val(endpt_desc->bInterval, 1, 16) - 1);
    if (interval != endpt_desc->bInterval - 1U)
        Kprintf("ep %#lx - rounding interval to %lu %sframes\n",
                (ULONG)endpt_desc->bEndpointAddress, (ULONG)(1U << interval),
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
 */
u8 xhci_get_endpoint_interval(struct usb_device *udev, struct usb_endpoint_descriptor *endpt_desc)
{
    u8 interval = 0;

    switch (udev->speed)
    {
    case USB_SPEED_HIGH:
        /* Max NAK rate */
        if (usb_endpoint_xfer_control(endpt_desc) ||
            usb_endpoint_xfer_bulk(endpt_desc))
        {
            interval = xhci_parse_microframe_interval(endpt_desc);
            break;
        }
        /* Fall through - SS and HS isoc/int have same decoding */

    case USB_SPEED_SUPER:
        if (usb_endpoint_xfer_int(endpt_desc) ||
            usb_endpoint_xfer_isoc(endpt_desc))
        {
            interval = xhci_parse_exponent_interval(udev,
                                                    endpt_desc);
        }
        break;

    case USB_SPEED_FULL:
        if (usb_endpoint_xfer_isoc(endpt_desc))
        {
            interval = xhci_parse_exponent_interval(udev,
                                                    endpt_desc);
            break;
        }
        /*
         * Fall through for interrupt endpoint interval decoding
         * since it uses the same rules as low speed interrupt
         * endpoints.
         */
        /*fallthrough;*/
    case USB_SPEED_LOW:
        if (usb_endpoint_xfer_int(endpt_desc) ||
            usb_endpoint_xfer_isoc(endpt_desc))
        {
            interval = xhci_parse_frame_interval(endpt_desc);
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
u8 xhci_get_endpoint_mult(struct usb_device *udev,
                          struct usb_endpoint_descriptor *endpt_desc,
                          struct usb_ss_ep_comp_descriptor *ss_ep_comp_desc)
{
    if (udev->speed < USB_SPEED_SUPER || !usb_endpoint_xfer_isoc(endpt_desc))
        return 0;

    return ss_ep_comp_desc->bmAttributes;
}

u8 xhci_get_endpoint_max_burst(struct usb_device *udev,
                               struct usb_endpoint_descriptor *endpt_desc,
                               struct usb_ss_ep_comp_descriptor *ss_ep_comp_desc)
{
    /* Super speed and Plus have max burst in ep companion desc */
    if (udev->speed >= USB_SPEED_SUPER)
        return ss_ep_comp_desc->bMaxBurst;

    if (udev->speed == USB_SPEED_HIGH && (usb_endpoint_xfer_isoc(endpt_desc) || usb_endpoint_xfer_int(endpt_desc)))
        return (u8)(usb_endpoint_maxp_mult(endpt_desc) - 1u);

    return 0;
}

/*
 * Return the maximum endpoint service interval time (ESIT) payload.
 * Basically, this is the maxpacket size, multiplied by the burst size
 * and mult size.
 */
u32 xhci_get_max_esit_payload(struct usb_device *udev,
                              struct usb_endpoint_descriptor *endpt_desc,
                              struct usb_ss_ep_comp_descriptor *ss_ep_comp_desc)
{
    /* Only applies for interrupt or isochronous endpoints */
    if (usb_endpoint_xfer_control(endpt_desc) || usb_endpoint_xfer_bulk(endpt_desc))
        return 0;

    /* SuperSpeed Isoc ep with less than 48k per esit */
    if (udev->speed >= USB_SPEED_SUPER)
        return le16(ss_ep_comp_desc->wBytesPerInterval);

    u32 max_packet = usb_endpoint_maxp(endpt_desc);
    u8 max_burst = usb_endpoint_maxp_mult(endpt_desc);

    /* usb_endpoint_maxp_mult() already returns the encoded multiplier + 1. */
    return max_packet * max_burst;
}

static void xhci_dump_interface(const char *tag, u8 index, const struct usb_interface *iface)
{
    if (!iface)
        return;

    const char *pfx = tag ? tag : "";

    const struct usb_interface_altsetting *active_alt = iface->active_altsetting;
    const struct usb_interface_altsetting *desc_alt = active_alt;
    if (!desc_alt && iface->num_altsetting > 0)
        desc_alt = &iface->altsetting[0];

    Kprintf("%s  Interface %lu:\n", pfx, (ULONG)index);
    if (desc_alt)
    {
        Kprintf("%s    bLength=%lu bDescriptorType=%lu bInterfaceNumber=%lu bAlternateSetting=%lu\n", pfx,
                (ULONG)desc_alt->desc.bLength, (ULONG)desc_alt->desc.bDescriptorType,
                (ULONG)desc_alt->desc.bInterfaceNumber, (ULONG)desc_alt->desc.bAlternateSetting);
        Kprintf("%s    bNumEndpoints=%lu bInterfaceClass=%lu bInterfaceSubClass=%lu bInterfaceProtocol=%lu\n", pfx,
                (ULONG)desc_alt->desc.bNumEndpoints, (ULONG)desc_alt->desc.bInterfaceClass,
                (ULONG)desc_alt->desc.bInterfaceSubClass, (ULONG)desc_alt->desc.bInterfaceProtocol);
        Kprintf("%s    iInterface=%lu no_of_ep=%lu\n", pfx,
                (ULONG)desc_alt->desc.iInterface,
                (ULONG)(active_alt ? active_alt->no_of_ep : desc_alt->no_of_ep));
    }
    else
    {
        Kprintf("%s    (no descriptors captured for this interface)\n", pfx);
    }

    if (!active_alt)
    {
        Kprintf("%s    (no active alternate setting)\n", pfx);
        return;
    }

    for (u8 j = 0; j < active_alt->no_of_ep; ++j)
    {
        const struct usb_endpoint_descriptor *ep = &active_alt->ep_desc[j];
        const struct usb_ss_ep_comp_descriptor *ss_ep = &active_alt->ss_ep_comp_desc[j];
        Kprintf("%s    Endpoint %lu:\n", pfx, (ULONG)j);
        Kprintf("%s      bLength=%lu bDescriptorType=%lu bEndpointAddress=0x%02lx bmAttributes=0x%02lx\n",
                pfx, (ULONG)ep->bLength, (ULONG)ep->bDescriptorType,
                (ULONG)ep->bEndpointAddress, (ULONG)ep->bmAttributes);
        Kprintf("%s      wMaxPacketSize=%lu bInterval=%lu\n",
                pfx, (ULONG)le16(ep->wMaxPacketSize), (ULONG)ep->bInterval);
        Kprintf("%s      SS Companion: bLength=%lu bDescriptorType=%lu bMaxBurst=%lu bmAttributes=0x%02lx\n",
                pfx, (ULONG)ss_ep->bLength, (ULONG)ss_ep->bDescriptorType,
                (ULONG)ss_ep->bMaxBurst, (ULONG)ss_ep->bmAttributes);
        Kprintf("%s      SS Companion: wBytesPerInterval=%lu\n",
                pfx, (ULONG)le16(ss_ep->wBytesPerInterval));
    }
}

void xhci_dump_config(const char *tag, const struct usb_config *cfg, u16 addr)
{
    if (!cfg)
        return;

    const char *pfx = tag ? tag : "";

    Kprintf("%s Addr %lu configuration dump:\n", pfx, (ULONG)addr);
    Kprintf("%s  bLength=%lu bDescriptorType=%lu wTotalLength=%lu bNumInterfaces=%lu\n",
            pfx, (ULONG)cfg->desc.bLength, (ULONG)cfg->desc.bDescriptorType,
            (ULONG)le16(cfg->desc.wTotalLength), (ULONG)cfg->desc.bNumInterfaces);
    Kprintf("%s  bConfigurationValue=%lu iConfiguration=%lu bmAttributes=0x%02lx bMaxPower=%lu\n",
            pfx, (ULONG)cfg->desc.bConfigurationValue, (ULONG)cfg->desc.iConfiguration,
            (ULONG)cfg->desc.bmAttributes, (ULONG)cfg->desc.bMaxPower);
    Kprintf("%s  no_of_if=%lu\n", pfx, (ULONG)cfg->no_of_if);
    for (u8 i = 0; i < cfg->no_of_if; ++i)
        xhci_dump_interface(pfx, i, &cfg->if_desc[i]);
}

/* Running state while walking a configuration descriptor's sub-descriptors. */
struct cfg_parse_state
{
    struct usb_config *conf;
    int interface_map[USB_MAXINTERFACES];
    int if_index;
    int current_alt_index;
    struct usb_interface *current_if;
    struct usb_interface_altsetting *current_alt;
};

/* Handle an INTERFACE descriptor: select/allocate the interface and start a new
 * alternate setting.  Returns FALSE on a fatal error that must abort the parse
 * (too many unique interfaces); a recoverable problem just resets current_if/alt
 * and returns TRUE so the caller keeps scanning. */
static BOOL parse_interface_descriptor(struct cfg_parse_state *st, struct usb_interface_descriptor *ifd)
{
    u32 iface_number = ifd->bInterfaceNumber;
    if (iface_number >= USB_MAXINTERFACES)
    {
        Kprintf("interface number %lu exceeds max %lu\n", (ULONG)iface_number, (ULONG)USB_MAXINTERFACES);
        st->current_if = NULL;
        st->current_alt = NULL;
        st->current_alt_index = -1;
        return TRUE;
    }

    st->if_index = st->interface_map[iface_number];
    if (st->if_index < 0)
    {
        st->if_index = st->conf->no_of_if;
        if (st->if_index >= USB_MAXINTERFACES)
        {
            Kprintf("too many unique interfaces (%lu)\n", (ULONG)st->if_index);
            return FALSE;
        }
        st->interface_map[iface_number] = st->if_index;
        st->current_if = &st->conf->if_desc[st->if_index];
        mem_zero(st->current_if, sizeof(struct usb_interface));
        st->current_if->interface_number = (u8)iface_number;
        st->current_if->num_altsetting = 0;
        st->current_if->active_altsetting = NULL;
        st->conf->no_of_if++;
    }
    else
    {
        st->current_if = &st->conf->if_desc[st->if_index];
    }

    st->current_if->interface_number = (u8)iface_number;

    if (st->current_if->num_altsetting >= USB_ALTSETTINGALLOC)
    {
        Kprintf("too many alternate settings (%lu) for interface %lu\n",
                (ULONG)st->current_if->num_altsetting, (ULONG)iface_number);
        st->current_alt = NULL;
        st->current_alt_index = -1;
        return TRUE;
    }

    st->current_alt_index = st->current_if->num_altsetting++;
    st->current_alt = &st->current_if->altsetting[st->current_alt_index];
    mem_zero(st->current_alt, sizeof(struct usb_interface_altsetting));

    CopyMem(ifd, &st->current_alt->desc, sizeof(struct usb_interface_descriptor));
    st->current_alt->no_of_ep = 0;

    KprintfH("interface %lu alt %lu: bInterfaceNumber=%lu bAlternateSetting=%lu bNumEndpoints=%lu bInterfaceClass=0x%02lx bInterfaceSubClass=0x%02lx bInterfaceProtocol=0x%02lx iInterface=%lu\n",
             (ULONG)st->if_index,
             (ULONG)st->current_alt_index,
             (ULONG)ifd->bInterfaceNumber,
             (ULONG)ifd->bAlternateSetting,
             (ULONG)ifd->bNumEndpoints,
             (ULONG)ifd->bInterfaceClass,
             (ULONG)ifd->bInterfaceSubClass,
             (ULONG)ifd->bInterfaceProtocol,
             (ULONG)ifd->iInterface);

    if (st->current_if->active_altsetting == NULL || st->current_alt->desc.bAlternateSetting == 0)
        st->current_if->active_altsetting = st->current_alt;
    return TRUE;
}

/* Handle an ENDPOINT descriptor: append it to the current alternate setting. */
static void parse_endpoint_descriptor(struct cfg_parse_state *st, struct usb_endpoint_descriptor *epd)
{
    if (!st->current_if || !st->current_alt)
    {
        Kprintf("endpoint without interface or altsetting\n");
        return;
    }
    if (st->current_alt->no_of_ep >= USB_MAXENDPOINTS)
    {
        Kprintf("too many endpoints for interface %lu alt %lu\n",
                (ULONG)st->if_index, (ULONG)st->current_alt_index);
        return;
    }

    u32 ep_idx = st->current_alt->no_of_ep;
    CopyMem(epd, &st->current_alt->ep_desc[ep_idx], sizeof(struct usb_endpoint_descriptor));
    KprintfH("  endpoint %lu: bEndpointAddress=0x%02lx bmAttributes=0x%02lx wMaxPacketSize=%lu bInterval=%lu\n",
             (ULONG)ep_idx,
             (ULONG)epd->bEndpointAddress,
             (ULONG)epd->bmAttributes,
             (ULONG)le16(epd->wMaxPacketSize),
             (ULONG)epd->bInterval);

    st->current_alt->no_of_ep++;
}

/* Handle a SuperSpeed ENDPOINT COMPANION descriptor: attach it to the endpoint
 * it follows. */
static void parse_ss_ep_comp_descriptor(struct cfg_parse_state *st, struct usb_ss_ep_comp_descriptor *comp)
{
    KprintfH("found SS EP COMP descriptor\n");
    if (st->current_if && st->current_alt && st->current_alt->no_of_ep > 0)
    {
        u32 ep_slot = (u32)(st->current_alt->no_of_ep - 1U);
        CopyMem(comp, &st->current_alt->ss_ep_comp_desc[ep_slot], sizeof(struct usb_ss_ep_comp_descriptor));
    }
}

/* Parse a complete configuration descriptor (config + interface/altsetting/
 * endpoint/SS-companion sub-descriptors) into a fresh usb_config and store it on
 * the device, replacing any prior config with the same bConfigurationValue. */
void xhci_parse_config_descriptor(struct usb_device *udev, u8 *data, u16 len)
{
    if (len < 2)
    {
        KprintfH("too short, len=%lu\n", (ULONG)len);
        return;
    }

    struct usb_config *conf = pool_zalloc(udev->controller->metaPool, sizeof(*conf));
    if (!conf)
    {
        Kprintf("pool_zalloc failed\n");
        return;
    }

    struct usb_config_descriptor *desc = (struct usb_config_descriptor *)data;
    if (desc->bDescriptorType != USB_DT_CONFIG)
    {
        Kprintf("bad desc type %lu\n", (ULONG)desc->bDescriptorType);
        goto error;
    }

    u16 total_len = le16(desc->wTotalLength);
    if (len < total_len)
    {
        KprintfH("short buffer len=%lu total_len=%lu\n", (ULONG)len, (ULONG)total_len);
        return;
    }

    u8 *cursor = data;
    u8 *end = data + total_len;
    if (cursor + desc->bLength > end)
    {
        Kprintf("bad desc length %lu\n", (ULONG)desc->bLength);
        goto error;
    }

    CopyMem(desc, &conf->desc, sizeof(struct usb_config_descriptor));
    cursor += desc->bLength;

    KprintfH("wTotalLength=%lu bNumInterfaces=%lu bConfigurationValue=%lu iConfiguration=%lu bmAttributes=0x%02lx bMaxPower=%lu\n",
             (ULONG)le16(desc->wTotalLength),
             (ULONG)desc->bNumInterfaces,
             (ULONG)desc->bConfigurationValue,
             (LONG)desc->iConfiguration,
             (LONG)desc->bmAttributes,
             (LONG)desc->bMaxPower);

    // in 3.x there are association descriptors here

    struct cfg_parse_state st;
    st.conf = conf;
    for (int i = 0; i < USB_MAXINTERFACES; ++i)
        st.interface_map[i] = -1;
    conf->no_of_if = 0;
    st.if_index = 0;
    st.current_alt_index = -1;
    st.current_if = NULL;
    st.current_alt = NULL;

    while (cursor + 2 <= end)
    {
        u8 dlen = cursor[0];
        u8 dtype = cursor[1];
        if (dlen == 0)
        {
            Kprintf("zero length descriptor, aborting\n");
            break;
        }
        if (cursor + dlen > end)
        {
            Kprintf("descriptor overruns buffer (type=%lu len=%lu)\n", (ULONG)dtype, (ULONG)dlen);
            break;
        }

        switch (dtype)
        {
        case USB_DT_INTERFACE:
            if (!parse_interface_descriptor(&st, (struct usb_interface_descriptor *)cursor))
                goto error;
            break;
        case USB_DT_ENDPOINT:
            parse_endpoint_descriptor(&st, (struct usb_endpoint_descriptor *)cursor);
            break;
        case USB_DT_SS_ENDPOINT_COMP:
            parse_ss_ep_comp_descriptor(&st, (struct usb_ss_ep_comp_descriptor *)cursor);
            break;
        default:
            // Skip class- or vendor-specific descriptors gracefully.
            KprintfH("found class/vendor-specific descriptor 0x%lx, len=%lu\n", (ULONG)dtype, (ULONG)dlen);
            break;
        }

        cursor += dlen;
    }
    KprintfH("parsed config with %lu interfaces\n", (ULONG)conf->no_of_if);

    if (conf->no_of_if != desc->bNumInterfaces)
    {
        Kprintf("interface count mismatch %lu != %lu\n",
                (ULONG)conf->no_of_if, (ULONG)desc->bNumInterfaces);
        goto error;
    }

    for (struct MinNode *n = udev->configurations.mlh_Head; n->mln_Succ; n = n->mln_Succ)
    {
        struct usb_config *oldconf = (struct usb_config *)n;
        if (oldconf->desc.bConfigurationValue == conf->desc.bConfigurationValue)
        {
            KprintfH("removing old config with value %lu\n", (ULONG)oldconf->desc.bConfigurationValue);
            RemoveMinNode(n);
            pool_free(udev->controller->metaPool, oldconf);
            break;
        }
    }
    AddHeadMinList(&udev->configurations, (struct MinNode *)conf);

    return;

error:
    pool_free(udev->controller->metaPool, conf);
}
