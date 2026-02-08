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
unsigned int xhci_get_ep_index(struct usb_endpoint_descriptor *desc)
{
    unsigned int index;

    if (usb_endpoint_xfer_control(desc))
        index = (unsigned int)(usb_endpoint_num(desc) * 2);
    else
        index = (unsigned int)((usb_endpoint_num(desc) * 2) -
                               (usb_endpoint_dir_in(desc) ? 0 : 1));

    return index;
}

struct usb_interface *xhci_find_interface(struct usb_config *cfg, unsigned int iface_number)
{
    if (!cfg)
        return NULL;

    for (unsigned int i = 0; i < cfg->no_of_if; ++i)
    {
        if (cfg->if_desc[i].interface_number == iface_number)
            return &cfg->if_desc[i];
    }

    return NULL;
}

struct usb_interface_altsetting *xhci_find_altsetting(struct usb_interface *iface, unsigned int alt_setting)
{
    if (!iface)
        return NULL;

    for (unsigned int idx = 0; idx < iface->num_altsetting; ++idx)
    {
        struct usb_interface_altsetting *candidate = &iface->altsetting[idx];
        if (candidate->desc.bAlternateSetting == alt_setting)
            return candidate;
    }

    return NULL;
}

u32 xhci_collect_ep_mask(const struct usb_interface_altsetting *alt, unsigned int *max_flag)
{
    if (!alt)
        return 0;

    u32 mask = 0;

    for (unsigned int i = 0; i < alt->no_of_ep; ++i)
    {
        const struct usb_endpoint_descriptor *epd = &alt->ep_desc[i];
        unsigned int ep_index = xhci_get_ep_index((struct usb_endpoint_descriptor *)epd);
        mask |= 1U << (ep_index + 1);
        if (max_flag && ep_index > *max_flag)
            *max_flag = ep_index;
    }

    return mask;
}

u32 xhci_collect_config_masks(const struct usb_config *cfg, unsigned int limit, unsigned int *max_flag)
{
    if (!cfg)
        return 0;

    u32 mask = 0;
    unsigned int max_if = min(limit, cfg->no_of_if);

    for (unsigned int ifnum = 0; ifnum < max_if; ++ifnum)
    {
        const struct usb_interface *iface = &cfg->if_desc[ifnum];
        const struct usb_interface_altsetting *active_alt = iface->active_altsetting;
        if (!active_alt)
            continue;

        KprintfH("Preparing iface=%ld alt=%ld\n", (ULONG)ifnum, (LONG)active_alt->desc.bAlternateSetting);

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
        KprintfH("  found config: bConfigurationValue=%ld\n", (LONG)cfg->desc.bConfigurationValue);
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
    for (unsigned int idx = 0; idx < iface->num_altsetting; ++idx)
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

unsigned int compute_max_ep_flag(const struct usb_config *cfg)
{
    if (!cfg)
        return 0;

    unsigned int max_flag = 0;
    xhci_collect_config_masks(cfg, cfg->no_of_if, &max_flag);

    return max_flag;
}

/*
 * Convert bInterval expressed in microframes (in 1-255 range) to exponent of
 * microframes, rounded down to nearest power of 2.
 */
static unsigned int xhci_microframes_to_exponent(unsigned int desc_interval,
                                                 unsigned int min_exponent,
                                                 unsigned int max_exponent)
{
    unsigned int interval = fls(desc_interval) - 1;
    interval = clamp_val(interval, min_exponent, max_exponent);
#ifdef DEBUG_HIGH
    if ((1U << interval) != desc_interval)
        KprintfH("rounding interval to %ld microframes, ep desc says %ld microframes\n",
                 1U << interval, desc_interval);
#endif

    return interval;
}

static unsigned int xhci_parse_microframe_interval(struct usb_endpoint_descriptor *endpt_desc)
{
    if (endpt_desc->bInterval == 0)
        return 0;

    return xhci_microframes_to_exponent(endpt_desc->bInterval, 0, 15);
}

static unsigned int xhci_parse_frame_interval(struct usb_endpoint_descriptor *endpt_desc)
{
    return xhci_microframes_to_exponent(endpt_desc->bInterval * 8, 3, 10);
}

/*
 * Convert interval expressed as 2^(bInterval - 1) == interval into
 * straight exponent value 2^n == interval.
 */
static unsigned int xhci_parse_exponent_interval(struct usb_device *udev,
                                                 struct usb_endpoint_descriptor *endpt_desc)
{
    unsigned int interval;

    interval = clamp_val(endpt_desc->bInterval, 1, 16) - 1;
    if (interval != endpt_desc->bInterval - 1U)
        Kprintf("ep %#lx - rounding interval to %ld %sframes\n",
                endpt_desc->bEndpointAddress, 1 << interval,
                udev->speed == USB_SPEED_FULL ? "" : "micro");

    if (udev->speed == USB_SPEED_FULL)
    {
        /*
         * Full speed isoc endpoints specify interval in frames,
         * not microframes. We are using microframes everywhere,
         * so adjust accordingly.
         */
        interval += 3; /* 1 frame = 2^3 uframes */
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
unsigned int xhci_get_endpoint_interval(struct usb_device *udev, struct usb_endpoint_descriptor *endpt_desc)
{
    unsigned int interval = 0;

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
        Kprintf("Unsupported USB speed: %ld\n", udev->speed);
    }

    return interval;
}

/*
 * The "Mult" field in the endpoint context is only set for SuperSpeed isoc eps.
 * High speed endpoint descriptors can define "the number of additional
 * transaction opportunities per microframe", but that goes in the Max Burst
 * endpoint context field.
 */
u32 xhci_get_endpoint_mult(struct usb_device *udev,
                           struct usb_endpoint_descriptor *endpt_desc,
                           struct usb_ss_ep_comp_descriptor *ss_ep_comp_desc)
{
    if (udev->speed < USB_SPEED_SUPER || !usb_endpoint_xfer_isoc(endpt_desc))
        return 0;

    return ss_ep_comp_desc->bmAttributes;
}

u32 xhci_get_endpoint_max_burst(struct usb_device *udev,
                                struct usb_endpoint_descriptor *endpt_desc,
                                struct usb_ss_ep_comp_descriptor *ss_ep_comp_desc)
{
    /* Super speed and Plus have max burst in ep companion desc */
    if (udev->speed >= USB_SPEED_SUPER)
        return ss_ep_comp_desc->bMaxBurst;

    if (udev->speed == USB_SPEED_HIGH && (usb_endpoint_xfer_isoc(endpt_desc) || usb_endpoint_xfer_int(endpt_desc)))
        return usb_endpoint_maxp_mult(endpt_desc) - 1;

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
        return LE16(ss_ep_comp_desc->wBytesPerInterval);

    int max_packet = usb_endpoint_maxp(endpt_desc);
    int max_burst = usb_endpoint_maxp_mult(endpt_desc);

    /* A 0 in max burst means 1 transfer per ESIT */
    return max_packet * max_burst;
}

static void xhci_dump_interface(const char *tag, UBYTE index, const struct usb_interface *iface)
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

    for (UBYTE j = 0; j < active_alt->no_of_ep; ++j)
    {
        const struct usb_endpoint_descriptor *ep = &active_alt->ep_desc[j];
        const struct usb_ss_ep_comp_descriptor *ss_ep = &active_alt->ss_ep_comp_desc[j];
        Kprintf("%s    Endpoint %lu:\n", pfx, (ULONG)j);
        Kprintf("%s      bLength=%lu bDescriptorType=%lu bEndpointAddress=0x%02lx bmAttributes=0x%02lx\n",
                pfx, (ULONG)ep->bLength, (ULONG)ep->bDescriptorType,
                (ULONG)ep->bEndpointAddress, (ULONG)ep->bmAttributes);
        Kprintf("%s      wMaxPacketSize=%lu bInterval=%lu\n",
                pfx, (ULONG)LE16(ep->wMaxPacketSize), (ULONG)ep->bInterval);
        Kprintf("%s      SS Companion: bLength=%lu bDescriptorType=%lu bMaxBurst=%lu bmAttributes=0x%02lx\n",
                pfx, (ULONG)ss_ep->bLength, (ULONG)ss_ep->bDescriptorType,
                (ULONG)ss_ep->bMaxBurst, (ULONG)ss_ep->bmAttributes);
        Kprintf("%s      SS Companion: wBytesPerInterval=%lu\n",
                pfx, (ULONG)LE16(ss_ep->wBytesPerInterval));
    }
}

void xhci_dump_config(const char *tag, const struct usb_config *cfg, UBYTE addr)
{
    if (!cfg)
        return;

    const char *pfx = tag ? tag : "";

    Kprintf("%s Addr %ld configuration dump:\n", pfx, (ULONG)addr);
    Kprintf("%s  bLength=%lu bDescriptorType=%lu wTotalLength=%lu bNumInterfaces=%lu\n",
            pfx, (ULONG)cfg->desc.bLength, (ULONG)cfg->desc.bDescriptorType,
            (ULONG)LE16(cfg->desc.wTotalLength), (ULONG)cfg->desc.bNumInterfaces);
    Kprintf("%s  bConfigurationValue=%lu iConfiguration=%lu bmAttributes=0x%02lx bMaxPower=%lu\n",
            pfx, (ULONG)cfg->desc.bConfigurationValue, (ULONG)cfg->desc.iConfiguration,
            (ULONG)cfg->desc.bmAttributes, (ULONG)cfg->desc.bMaxPower);
    Kprintf("%s  no_of_if=%lu\n", pfx, (ULONG)cfg->no_of_if);
    for (UBYTE i = 0; i < cfg->no_of_if; ++i)
        xhci_dump_interface(pfx, i, &cfg->if_desc[i]);
}
