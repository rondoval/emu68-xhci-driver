#include <exec/types.h>
#include <xhci/ch9.h>
#include <xhci/xhci.h>
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-endpoint.h>
#include <xhci/xhci-debug.h>
#include <xhci/xhci-usb.h>

#include <debug.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-usb] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci-usb] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

int xhci_ep_type_for_index(struct usb_device *udev, u32 ep_index)
{
    if (!udev)
        return -1;

    if (ep_index == 0)
        return USB_ENDPOINT_XFER_CONTROL;

    struct usb_config *cfg = udev->active_config;
    if (!cfg)
        return -1;

    UBYTE addr = xhci_ep_index_to_address(ep_index);

    for (int i = 0; i < cfg->no_of_if; ++i)
    {
        struct usb_interface *iface = &cfg->if_desc[i];
        struct usb_interface_altsetting *alt = iface->active_altsetting;
        if (!alt)
            continue;

        for (int e = 0; e < alt->no_of_ep; ++e)
        {
            struct usb_endpoint_descriptor *desc = &alt->ep_desc[e];
            if (desc->bEndpointAddress == addr)
                return desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
        }
    }

    return -1;
}

static void parse_config_descriptor(struct usb_device *udev, UBYTE *data, UWORD len)
{
    if (len < 2)
    {
        KprintfH("too short, len=%ld\n", (LONG)len);
        return;
    }

    struct usb_config *conf = AllocVecPooled(udev->controller->memoryPool, sizeof(*conf));
    if (!conf)
    {
        Kprintf("AllocVecPooled failed\n");
        return;
    }
    _memset(conf, 0, sizeof(*conf));

    struct usb_config_descriptor *desc = (struct usb_config_descriptor *)data;
    if (desc->bDescriptorType != USB_DT_CONFIG)
    {
        Kprintf("bad desc type %ld\n", (LONG)desc->bDescriptorType);
        goto error;
    }

    UWORD total_len = LE16(desc->wTotalLength);
    if ((UWORD)len < total_len)
    {
        KprintfH("short buffer len=%ld total_len=%ld\n", (LONG)len, (LONG)total_len);
        return;
    }

    UBYTE *cursor = data;
    UBYTE *end = data + total_len;
    if (cursor + desc->bLength > end)
    {
        Kprintf("bad desc length %ld\n", (LONG)desc->bLength);
        goto error;
    }

    CopyMem(desc, &conf->desc, sizeof(struct usb_config_descriptor));
    cursor += desc->bLength;

    KprintfH("wTotalLength=%ld bNumInterfaces=%ld bConfigurationValue=%ld iConfiguration=%ld bmAttributes=0x%02lx bMaxPower=%ld\n",
             (LONG)LE16(desc->wTotalLength),
             (LONG)desc->bNumInterfaces,
             (LONG)desc->bConfigurationValue,
             (LONG)desc->iConfiguration,
             (LONG)desc->bmAttributes,
             (LONG)desc->bMaxPower);

    // in 3.x there are association descriptors here

    int interface_map[USB_MAXINTERFACES];
    for (int i = 0; i < USB_MAXINTERFACES; ++i)
        interface_map[i] = -1;

    conf->no_of_if = 0;
    int if_index = 0;
    int current_alt_index = -1;
    struct usb_interface *current_if = NULL;
    struct usb_interface_altsetting *current_alt = NULL;

    while (cursor + 2 <= end)
    {
        UBYTE dlen = cursor[0];
        UBYTE dtype = cursor[1];
        if (dlen == 0)
        {
            Kprintf("zero length descriptor, aborting\n");
            break;
        }
        if (cursor + dlen > end)
        {
            Kprintf("descriptor overruns buffer (type=%ld len=%ld)\n", (LONG)dtype, (LONG)dlen);
            break;
        }

        switch (dtype)
        {
        case USB_DT_INTERFACE:
        {
            struct usb_interface_descriptor *ifd = (struct usb_interface_descriptor *)cursor;
            unsigned int iface_number = ifd->bInterfaceNumber;
            if (iface_number >= USB_MAXINTERFACES)
            {
                Kprintf("interface number %ld exceeds max %ld\n", (LONG)iface_number, (LONG)USB_MAXINTERFACES);
                current_if = NULL;
                current_alt = NULL;
                current_alt_index = -1;
                break;
            }

            if_index = interface_map[iface_number];
            if (if_index < 0)
            {
                if_index = conf->no_of_if;
                if (if_index >= USB_MAXINTERFACES)
                {
                    Kprintf("too many unique interfaces (%ld)\n", (LONG)if_index);
                    goto error;
                }
                interface_map[iface_number] = if_index;
                current_if = &conf->if_desc[if_index];
                _memset(current_if, 0, sizeof(struct usb_interface));
                current_if->interface_number = (u8)iface_number;
                current_if->num_altsetting = 0;
                current_if->active_altsetting = NULL;
                conf->no_of_if++;
            }
            else
            {
                current_if = &conf->if_desc[if_index];
            }

            current_if->interface_number = (u8)iface_number;

            if (current_if->num_altsetting >= USB_ALTSETTINGALLOC)
            {
                Kprintf("too many alternate settings (%ld) for interface %ld\n",
                        (LONG)current_if->num_altsetting, (LONG)iface_number);
                current_alt = NULL;
                current_alt_index = -1;
                break;
            }

            current_alt_index = current_if->num_altsetting++;
            current_alt = &current_if->altsetting[current_alt_index];
            _memset(current_alt, 0, sizeof(struct usb_interface_altsetting));

            CopyMem(ifd, &current_alt->desc, sizeof(struct usb_interface_descriptor));
            current_alt->no_of_ep = 0;

            KprintfH("interface %ld alt %ld: bInterfaceNumber=%ld bAlternateSetting=%ld bNumEndpoints=%ld bInterfaceClass=0x%02lx bInterfaceSubClass=0x%02lx bInterfaceProtocol=0x%02lx iInterface=%ld\n",
                     (LONG)if_index,
                     (LONG)current_alt_index,
                     (LONG)ifd->bInterfaceNumber,
                     (LONG)ifd->bAlternateSetting,
                     (LONG)ifd->bNumEndpoints,
                     (LONG)ifd->bInterfaceClass,
                     (LONG)ifd->bInterfaceSubClass,
                     (LONG)ifd->bInterfaceProtocol,
                     (LONG)ifd->iInterface);

            if (current_if->active_altsetting == NULL || current_alt->desc.bAlternateSetting == 0)
                current_if->active_altsetting = current_alt;
            break;
        }
        case USB_DT_ENDPOINT:
        {
            if (!current_if || !current_alt)
            {
                Kprintf("endpoint without interface or altsetting\n");
                break;
            }
            if (current_alt->no_of_ep >= USB_MAXENDPOINTS)
            {
                Kprintf("too many endpoints for interface %ld alt %ld\n",
                        (LONG)if_index, (LONG)current_alt_index);
                break;
            }

            struct usb_endpoint_descriptor *epd = (struct usb_endpoint_descriptor *)cursor;
            unsigned int ep_idx = current_alt->no_of_ep;
            CopyMem(epd, &current_alt->ep_desc[ep_idx], sizeof(struct usb_endpoint_descriptor));
            KprintfH("  endpoint %ld: bEndpointAddress=0x%02lx bmAttributes=0x%02lx wMaxPacketSize=%ld bInterval=%ld\n",
                     (LONG)ep_idx,
                     (LONG)epd->bEndpointAddress,
                     (LONG)epd->bmAttributes,
                     (LONG)LE16(epd->wMaxPacketSize),
                     (LONG)epd->bInterval);

            current_alt->no_of_ep++;
            break;
        }
        case USB_DT_SS_ENDPOINT_COMP:
        {
            KprintfH("found SS EP COMP descriptor\n");
            if (current_if && current_alt && current_alt->no_of_ep > 0)
            {
                struct usb_ss_ep_comp_descriptor *comp = (struct usb_ss_ep_comp_descriptor *)cursor;
                unsigned int ep_slot = current_alt->no_of_ep - 1;
                CopyMem(comp, &current_alt->ss_ep_comp_desc[ep_slot], sizeof(struct usb_ss_ep_comp_descriptor));
            }
            break;
        }
        default:
            // Skip class- or vendor-specific descriptors gracefully.
            KprintfH("found class/vendor-specific descriptor 0x%lx, len=%ld\n", (ULONG)dtype, (LONG)dlen);
            break;
        }

        cursor += dlen;
    }
    KprintfH("parsed config with %ld interfaces\n", (LONG)conf->no_of_if);

    if (conf->no_of_if != desc->bNumInterfaces)
    {
        Kprintf("interface count mismatch %ld != %ld\n",
                (LONG)conf->no_of_if, (LONG)desc->bNumInterfaces);
        goto error;
    }

    for (struct MinNode *n = udev->configurations.mlh_Head; n->mln_Succ; n = n->mln_Succ)
    {
        struct usb_config *oldconf = (struct usb_config *)n;
        if (oldconf->desc.bConfigurationValue == conf->desc.bConfigurationValue)
        {
            KprintfH("removing old config with value %ld\n", (LONG)oldconf->desc.bConfigurationValue);
            RemoveMinNode(n);
            FreeVecPooled(udev->controller->memoryPool, oldconf);
            break;
        }
    }
    AddHeadMinList(&udev->configurations, (struct MinNode *)conf);

    return;

error:
    FreeVecPooled(udev->controller->memoryPool, conf);
}

static BOOL usb_glue_iface_has_active_rt_iso(struct usb_device *udev, unsigned int iface_number)
{
    if (!udev || !udev->active_config)
        return FALSE;

    struct usb_config *cfg = udev->active_config;
    for (int i = 0; i < cfg->no_of_if; ++i)
    {
        struct usb_interface *iface = &cfg->if_desc[i];
        if (iface->interface_number != (UBYTE)iface_number)
            continue;

        struct usb_interface_altsetting *alt = iface->active_altsetting;
        if (!alt)
            return FALSE;

        for (int e = 0; e < alt->no_of_ep; ++e)
        {
            int ep_index = xhci_address_to_ep_index(&alt->ep_desc[e]);

            struct ep_context *ep_ctx = xhci_ep_get_context_for_index(udev, ep_index);
            if(!ep_ctx)
                continue;
            enum ep_state state = xhci_ep_get_state(ep_ctx);
            if (state == USB_DEV_EP_STATE_RT_ISO_RUNNING ||
                state == USB_DEV_EP_STATE_RT_ISO_STOPPING)
                return TRUE;
        }

        return FALSE;
    }

    return FALSE;
}

static enum usb_device_speed usb_glue_speed_from_port_status(UWORD status)
{
    if (status & USB_PORT_STAT_SUPER_SPEED)
        return USB_SPEED_HIGH;
    // return USB_SPEED_SUPER; TODO capped speed... for now - fix context
    if (status & USB_PORT_STAT_HIGH_SPEED)
        return USB_SPEED_HIGH;
    if (status & USB_PORT_STAT_LOW_SPEED)
        return USB_SPEED_LOW;
    return USB_SPEED_FULL;
}

static struct usb_device *usb_glue_find_child_on_port(struct xhci_ctrl *ctrl, struct usb_device *hub, unsigned int port)
{
    if (!ctrl || !hub)
        return NULL;

    for (int i = 0; i <= USB_MAX_ADDRESS; ++i)
    {
        struct usb_device *cand = ctrl->devices_by_poseidon_address[i];
        if (!cand || cand == hub || cand->slot_id == 0)
            continue;

        if (cand->parent == hub && cand->parent_port == port)
            return cand;
    }

    return NULL;
}

static void usb_glue_disconnect_device(struct xhci_ctrl *ctrl, struct usb_device *udev, BOOL recursive)
{
    if (!ctrl || !udev)
        return;

    if (!udev->slot_id)
        return;

    /* Disconnect downstream devices first so hubs drain their children before vanishing. */
    if (recursive)
    {
        for (int i = 0; i <= USB_MAX_ADDRESS; ++i)
        {
            struct usb_device *child = ctrl->devices_by_poseidon_address[i];
            if (!child || child == udev)
                continue;

            if (child->parent == udev)
                usb_glue_disconnect_device(ctrl, child, TRUE);
        }
    }

    KprintfH("disconnect device addr=%ld slot=%ld port=%ld\n",
             (LONG)udev->poseidon_address,
             (LONG)udev->slot_id,
             (LONG)udev->parent_port);

    xhci_disable_slot(udev);
}

void xhci_usb_parse_control_message(struct usb_device *udev, struct IOUsbHWReq *io)
{
    KprintfH("dev=%lx addr=%ld bmReqType=%02lx bReq=%02lx wValue=%04lx wIndex=%04lx wLength=%04lx actual=%ld\n",
             (ULONG)udev, (ULONG)udev->poseidon_address,
             (ULONG)io->iouh_SetupData.bmRequestType,
             (ULONG)io->iouh_SetupData.bRequest,
             LE16(io->iouh_SetupData.wValue),
             LE16(io->iouh_SetupData.wIndex),
             LE16(io->iouh_SetupData.wLength),
             (LONG)io->iouh_Actual);

    /* If this was a successful GET_DESCRIPTOR(CONFIGURATION),
     * cache the configuration descriptor for later use.
     */
    if ((io->iouh_SetupData.bRequest == USB_REQ_GET_DESCRIPTOR) && (io->iouh_SetupData.bmRequestType & USB_DIR_IN) && (((LE16(io->iouh_SetupData.wValue)) >> 8) == USB_DT_CONFIG))
    {
        parse_config_descriptor(udev, (UBYTE *)io->iouh_Data, (UWORD)io->iouh_Actual);
    }

    /* Record TT think time from hub descriptors so child devices can be programmed correctly. */
    if ((io->iouh_SetupData.bRequest == USB_REQ_GET_DESCRIPTOR) &&
        (io->iouh_SetupData.bmRequestType == (USB_DIR_IN | USB_RT_HUB)))
    {
        UWORD dtype = (UWORD)((LE16(io->iouh_SetupData.wValue)) >> 8);
        if ((dtype == USB_DT_HUB || dtype == USB_DT_SS_HUB) && io->iouh_Actual >= 5)
        {
            struct usb_hub_descriptor *hub = (struct usb_hub_descriptor *)io->iouh_Data;
            UWORD characteristics = LE16(hub->wHubCharacteristics);
            u8 tt_think = (u8)((characteristics >> 5) & 0x3);
            if (udev->tt_think_time != tt_think)
            {
                udev->tt_think_time = tt_think;
                xhci_update_hub_tt(udev);
                KprintfH("hub addr %ld TT think time code=%ld (bit-times=%ld)\n",
                         (LONG)udev->poseidon_address, (LONG)tt_think, (LONG)((tt_think + 1) * 8));
            }
        }
    }

    /* Detect downstream port disconnects via hub GET_STATUS replies. */
    if ((io->iouh_SetupData.bRequest == USB_REQ_GET_STATUS) &&
        (io->iouh_SetupData.bmRequestType == (USB_DIR_IN | USB_RT_PORT)) &&
        io->iouh_Actual >= 4)
    {
        struct xhci_ctrl *ctrl = udev->controller;
        if (ctrl)
        {
            UWORD port = LE16(io->iouh_SetupData.wIndex);
            UWORD status = LE16(((UWORD *)io->iouh_Data)[0]);
            UWORD change = LE16(((UWORD *)io->iouh_Data)[1]);

            KprintfH("hub addr=%ld port=%ld status=%04lx change=%04lx\n", (LONG)udev->poseidon_address, (LONG)port, (ULONG)status, (ULONG)change);

            /* Drop children immediately if port power is off, regardless of change bits. */
            if ((status & USB_PORT_STAT_POWER) == 0)
            {
                KprintfH("hub addr=%ld port=%ld lost power; removing child if any\n", (LONG)udev->poseidon_address, (LONG)port);
                struct usb_device *child = usb_glue_find_child_on_port(ctrl, udev, port);
                if (child)
                {
                    KprintfH("hub addr=%ld port=%ld power-off, removing child addr=%ld slot=%ld\n",
                             (LONG)udev->poseidon_address, (LONG)port, (LONG)child->poseidon_address, (LONG)child->slot_id);
                    usb_glue_disconnect_device(ctrl, child, TRUE);
                }
            }

            /* If the port is disabled, tear down any attached child even if power is still on. */
            if ((status & USB_PORT_STAT_ENABLE) == 0)
            {
                KprintfH("hub addr=%ld port=%ld disabled; removing child if any\n", (LONG)udev->poseidon_address, (LONG)port);
                struct usb_device *child = usb_glue_find_child_on_port(ctrl, udev, port);
                if (child)
                {
                    KprintfH("hub addr=%ld port=%ld disabled, removing child addr=%ld slot=%ld\n",
                             (LONG)udev->poseidon_address, (LONG)port, (LONG)child->poseidon_address, (LONG)child->slot_id);
                    usb_glue_disconnect_device(ctrl, child, TRUE);
                }
            }

            /* Act when the hub reports a connection change to avoid reacting to stale status. */
            if (change & USB_PORT_STAT_C_CONNECTION)
            {
                if (status & USB_PORT_STAT_CONNECTION)
                {
                    /* Remember parent/port for the next default-address attach without split info. */
                    KprintfH("hub addr=%ld port=%ld connected; remembering for pending attach\n", (LONG)udev->poseidon_address, (LONG)port);
                    ctrl->pending_parent = udev;
                    ctrl->pending_parent_port = port;
                    ctrl->pending_parent_speed = usb_glue_speed_from_port_status(status);
                }

                if ((status & USB_PORT_STAT_CONNECTION) == 0)
                {
                    KprintfH("hub addr=%ld port=%ld disconnected; scanning children for match\n", (LONG)udev->poseidon_address, (LONG)port);
                    struct usb_device *child = usb_glue_find_child_on_port(ctrl, udev, port);

                    if (child)
                    {
                        KprintfH("hub addr=%ld port=%ld disconnected, removing child addr=%ld slot=%ld\n",
                                 (LONG)udev->poseidon_address, (LONG)port, (LONG)child->poseidon_address, (LONG)child->slot_id);
                        usb_glue_disconnect_device(ctrl, child, TRUE);
                    }
                    else
                    {
                        KprintfH("hub addr=%ld port=%ld disconnect: no child matched parent\n",
                                 (LONG)udev->poseidon_address, (LONG)port);
                    }
                }
            }

            /* Also remember parent/port after a reset-complete change when the link is up, even if
             * no explicit connection-change bit was set (common on root hub resets).
             */
            if ((change & USB_PORT_STAT_C_RESET) && (status & USB_PORT_STAT_CONNECTION))
            {
                KprintfH("hub addr=%ld port=%ld reset-complete; remembering for pending attach\n", (LONG)udev->poseidon_address, (LONG)port);
                ctrl->pending_parent = udev;
                ctrl->pending_parent_port = port;
                ctrl->pending_parent_speed = usb_glue_speed_from_port_status(status);
            }
        }
    }

    /* Proactively disconnect when the host powers or enables a hub port off. */
    if ((io->iouh_SetupData.bRequest == USB_REQ_CLEAR_FEATURE) &&
        (io->iouh_SetupData.bmRequestType == (USB_DIR_OUT | USB_RT_PORT)) &&
        (LE16(io->iouh_SetupData.wValue) == USB_PORT_FEAT_POWER ||
         LE16(io->iouh_SetupData.wValue) == USB_PORT_FEAT_ENABLE))
    {
        struct xhci_ctrl *ctrl = udev->controller;
        if (ctrl)
        {
            UWORD port = LE16(io->iouh_SetupData.wIndex);
            struct usb_device *child = usb_glue_find_child_on_port(ctrl, udev, port);
            if (child)
            {
                KprintfH("hub addr=%ld port=%ld CLEAR_FEATURE(%s); removing child addr=%ld slot=%ld\n",
                         (LONG)udev->poseidon_address, (LONG)port,
                         (LE16(io->iouh_SetupData.wValue) == USB_PORT_FEAT_POWER) ? "PORT_POWER" : "PORT_ENABLE",
                         (LONG)child->poseidon_address, (LONG)child->slot_id);
                usb_glue_disconnect_device(ctrl, child, TRUE);
            }
        }
    }

    /* If this was a successful standard SET_ADDRESS, migrate the glue context
     * from the old devaddr to the new one so subsequent transfers reuse the
     * same slot and endpoint state.
     * Note that this containt the address Poseidon _thinks_ it set on the interface.
     * In reality, XHCI selects the address.
     * Hence poseidon_address is what Poseidon uses; xhci_address is the real one.
     */
    if ((io->iouh_SetupData.bRequest == USB_REQ_SET_ADDRESS) &&
        ((io->iouh_SetupData.bmRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD))
    {
        UWORD old_addr = io->iouh_DevAddr & 0x7F;
        UWORD new_addr = (UWORD)(LE16(io->iouh_SetupData.wValue) & 0x7F);
        if (new_addr != old_addr)
        {
            struct xhci_ctrl *ctrl = udev->controller;
            if (!ctrl)
                return;

            if (old_addr == xhci_roothub_get_address(ctrl->root_hub))
                udev->speed = USB_SPEED_HIGH;

            struct usb_device *current = ctrl->devices_by_poseidon_address[old_addr];
            if (!current)
                current = udev;

            if (ctrl->devices_by_poseidon_address[new_addr] && ctrl->devices_by_poseidon_address[new_addr] != current)
            {
                Kprintf("overwriting existing ctx for addr %ld\n", (LONG)new_addr);
                /* If we are replacing an existing device (e.g., hub power-cycle), disconnect it (and children) first. */
                usb_glue_disconnect_device(ctrl, ctrl->devices_by_poseidon_address[new_addr], TRUE);
            }

            ctrl->devices_by_poseidon_address[new_addr] = current;
            if (ctrl->devices_by_poseidon_address[old_addr] == current)
                ctrl->devices_by_poseidon_address[old_addr] = NULL;

            current->poseidon_address = new_addr;

            if (current->slot_id && ctrl->devs[current->slot_id])
                ctrl->devs[current->slot_id]->udev = current;

            KprintfH("migrated ctx from addr %ld to %ld\n", (LONG)old_addr, (LONG)new_addr);
        }
    }

    if ((io->iouh_SetupData.bRequest == USB_REQ_SET_INTERFACE) &&
        ((io->iouh_SetupData.bmRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD) &&
        ((io->iouh_SetupData.bmRequestType & 0x1F) == USB_RECIP_INTERFACE))
    {
        unsigned int iface = (unsigned int)(LE16(io->iouh_SetupData.wIndex) & 0xFF);
        unsigned int alt = (unsigned int)(LE16(io->iouh_SetupData.wValue) & 0xFF);
        /*
         * This is a workaround for Poseidon issue.
         * Poseidon issues SET_INTERFACE for all devices after connecting a new one.
         * Thing is, it first sets the alternate setting to 0, then to the desired setting.
         * This causes issues with active RT ISO endpoints, as they get disabled on alt=0.
         */
        if (usb_glue_iface_has_active_rt_iso(udev, iface))
        {
            KprintfH("SET_INTERFACE iface=%ld alt=%ld ignored (RT ISO active)\n",
                     (LONG)iface, (LONG)alt);
        }
        else
        {
            int err = xhci_set_interface(udev, iface, alt);
            if (err != UHIOERR_NO_ERROR)
            {
                Kprintf("SET_INTERFACE iface=%ld alt=%ld failed err=%ld\n",
                        (LONG)iface, (LONG)alt, (LONG)err);
            }
        }
    }
}

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
static unsigned int xhci_get_ep_index(struct usb_endpoint_descriptor *desc)
{
    unsigned int index;

    if (usb_endpoint_xfer_control(desc))
        index = (unsigned int)(usb_endpoint_num(desc) * 2);
    else
        index = (unsigned int)((usb_endpoint_num(desc) * 2) -
                               (usb_endpoint_dir_in(desc) ? 0 : 1));

    return index;
}

static struct usb_interface *xhci_find_interface(struct usb_config *cfg, unsigned int iface_number)
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

static struct usb_interface_altsetting *xhci_find_altsetting(struct usb_interface *iface, unsigned int alt_setting)
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

static u32 xhci_collect_ep_mask(const struct usb_interface_altsetting *alt, unsigned int *max_flag)
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

static u32 xhci_collect_config_masks(const struct usb_config *cfg,
                                     unsigned int limit,
                                     unsigned int *max_flag,
                                     int log_prepare)
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

        if (log_prepare)
        {
            KprintfH("Preparing iface=%ld alt=%ld\n",
                     (ULONG)ifnum, (LONG)active_alt->desc.bAlternateSetting);
        }

        mask |= xhci_collect_ep_mask(active_alt, max_flag);
    }

    return mask;
}

static struct usb_config *xhci_find_config(struct usb_device *udev, int config_value)
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

static struct usb_interface_altsetting *xhci_select_active_alt(struct usb_interface *iface)
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

static int xhci_init_ep_contexts_if(struct usb_device *udev,
                                    struct xhci_ctrl *ctrl,
                                    struct xhci_virt_device *virt_dev,
                                    struct usb_interface *ifdesc);

static void xhci_update_slot_last_ctx(struct xhci_ctrl *ctrl,
                                      struct xhci_virt_device *virt_dev,
                                      unsigned int max_ep_flag)
{
    if (!ctrl || !virt_dev)
        return;

    struct xhci_slot_ctx *slot_ctx = xhci_get_slot_ctx(ctrl, virt_dev->in_ctx);
    if (!slot_ctx)
        return;

    u32 dev_info = LE32(slot_ctx->dev_info);
    dev_info &= ~LAST_CTX_MASK;
    dev_info |= LAST_CTX(max_ep_flag + 1);
    slot_ctx->dev_info = LE32(dev_info);
}

static unsigned int compute_max_ep_flag(const struct usb_config *cfg)
{
    if (!cfg)
        return 0;

    unsigned int max_flag = 0;
    xhci_collect_config_masks(cfg, cfg->no_of_if, &max_flag, 0);

    return max_flag;
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
    struct xhci_virt_device *virt_dev = ctrl->devs[udev->slot_id];
    if (!virt_dev || !virt_dev->in_ctx || !virt_dev->out_ctx)
    {
        Kprintf("xhci_set_interface: missing virtual device for slot %ld\n", (LONG)udev->slot_id);
        return UHIOERR_HOSTERROR;
    }

    /* Poseidon stack guarantees endpoint queues are idle before switching. */
    u32 drop_mask = xhci_collect_ep_mask(current_alt, NULL);

    iface->active_altsetting = new_alt;

    u32 add_mask = xhci_collect_ep_mask(new_alt, NULL);
    xhci_inval_cache(virt_dev->out_ctx->bytes, virt_dev->out_ctx->size);
    xhci_slot_copy(ctrl, virt_dev->in_ctx, virt_dev->out_ctx);
    xhci_endpoint_copy(ctrl, virt_dev->in_ctx, virt_dev->out_ctx, 0);

    struct xhci_input_control_ctx *ctrl_ctx = xhci_get_input_control_ctx(virt_dev->in_ctx);
    if (!ctrl_ctx)
    {
        Kprintf("xhci_set_interface: missing input control context\n");
        iface->active_altsetting = current_alt;
        return UHIOERR_HOSTERROR;
    }

    int err = UHIOERR_NO_ERROR;
    if (new_alt->no_of_ep > 0)
    {
        err = xhci_init_ep_contexts_if(udev, ctrl, virt_dev, iface);
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
    xhci_update_slot_last_ctx(ctrl, virt_dev, max_ep_flag);

    KprintfH("xhci_set_interface: updating device context for addr=%ld iface=%ld alt=%ld drop=0x%lx add=0x%lx\n",
             (LONG)udev->poseidon_address,
             (LONG)iface_number,
             (LONG)alt_setting,
             (ULONG)drop_mask,
             (ULONG)add_mask);

    xhci_configure_endpoints(udev, FALSE, NULL);

    return err;
}

/*
 * Convert bInterval expressed in microframes (in 1-255 range) to exponent of
 * microframes, rounded down to nearest power of 2.
 */
static unsigned int xhci_microframes_to_exponent(unsigned int desc_interval,
                                                 unsigned int min_exponent,
                                                 unsigned int max_exponent)
{
    unsigned int interval;

    interval = fls(desc_interval) - 1;
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
static unsigned int xhci_get_endpoint_interval(struct usb_device *udev,
                                               struct usb_endpoint_descriptor *endpt_desc)
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
static u32 xhci_get_endpoint_mult(struct usb_device *udev,
                                  struct usb_endpoint_descriptor *endpt_desc,
                                  struct usb_ss_ep_comp_descriptor *ss_ep_comp_desc)
{
    if (udev->speed < USB_SPEED_SUPER || !usb_endpoint_xfer_isoc(endpt_desc))
        return 0;

    return ss_ep_comp_desc->bmAttributes;
}

static u32 xhci_get_endpoint_max_burst(struct usb_device *udev,
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
static u32 xhci_get_max_esit_payload(struct usb_device *udev,
                                     struct usb_endpoint_descriptor *endpt_desc,
                                     struct usb_ss_ep_comp_descriptor *ss_ep_comp_desc)
{
    int max_burst;
    int max_packet;

    /* Only applies for interrupt or isochronous endpoints */
    if (usb_endpoint_xfer_control(endpt_desc) || usb_endpoint_xfer_bulk(endpt_desc))
        return 0;

    /* SuperSpeed Isoc ep with less than 48k per esit */
    if (udev->speed >= USB_SPEED_SUPER)
        return LE16(ss_ep_comp_desc->wBytesPerInterval);

    max_packet = usb_endpoint_maxp(endpt_desc);
    max_burst = usb_endpoint_maxp_mult(endpt_desc);

    /* A 0 in max burst means 1 transfer per ESIT */
    return max_packet * max_burst;
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
                                    struct xhci_virt_device *virt_dev,
                                    struct usb_interface *ifdesc)
{
    KprintfH("xhci_init_ep_contexts_if: enter\n");
    struct xhci_ep_ctx *ep_ctx[USB_MAX_ENDPOINT_CONTEXTS];
    int cur_ep;
    int ep_index;
    unsigned int dir;
    unsigned int ep_type;
    u64 trb_64 = 0;
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
        trb_64 = 0;

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
        ep_ctx[ep_index] = xhci_get_ep_ctx(ctrl, virt_dev->in_ctx, ep_index);

        /* Allocate the ep rings */
        BOOL result = xhci_ep_create_context(udev, ep_index, ctrl->memoryPool);
        if (!result)
            return UHIOERR_OUTOFMEMORY;

        /*NOTE: ep_desc[0] actually represents EP1 and so on */
        dir = (((endpt_desc->bEndpointAddress) & (0x80)) >> 7);
        ep_type = (((endpt_desc->bmAttributes) & (0x3)) | (dir << 2));

        ep_ctx[ep_index]->ep_info = LE32(EP_MAX_ESIT_PAYLOAD_HI(max_esit_payload) | EP_INTERVAL(interval) | EP_MULT(mult));

        ep_ctx[ep_index]->ep_info2 = LE32(EP_TYPE(ep_type));
        ep_ctx[ep_index]->ep_info2 |= LE32(MAX_PACKET(LE16(get_unaligned(&endpt_desc->wMaxPacketSize))));

        /* Allow 3 retries for everything but isoc, set CErr = 3 */
        if (!usb_endpoint_xfer_isoc(endpt_desc))
            err_count = 3;
        ep_ctx[ep_index]->ep_info2 |= LE32(MAX_BURST(max_burst) | ERROR_COUNT(err_count));

        struct ep_context *ep_context = xhci_ep_get_context_for_index(udev, ep_index);
        struct xhci_ring *ring = xhci_ep_get_ring(ep_context);
        trb_64 = (u64)(uintptr_t)ring->enqueue;
        ep_ctx[ep_index]->deq = LE64(trb_64 | ring->cycle_state);

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
    int slot_id = udev->slot_id;
    struct xhci_virt_device *virt_dev = ctrl->devs[slot_id];
    unsigned int max_ifnum = cfg->no_of_if;
    unsigned int max_ep_flag = 0;

    out_ctx = virt_dev->out_ctx;
    in_ctx = virt_dev->in_ctx;

    ctrl_ctx = xhci_get_input_control_ctx(in_ctx);
    u32 add_flags = SLOT_FLAG;
    u32 mask = xhci_collect_config_masks(cfg, max_ifnum, &max_ep_flag, 1);
    add_flags |= mask;
    ctrl_ctx->add_flags = LE32(add_flags);
    ctrl_ctx->drop_flags = 0;

    xhci_inval_cache(out_ctx->bytes, out_ctx->size);

    /* slot context */
    xhci_slot_copy(ctrl, in_ctx, out_ctx);
    xhci_update_slot_last_ctx(ctrl, virt_dev, max_ep_flag);

    xhci_endpoint_copy(ctrl, in_ctx, out_ctx, 0);

    /* filling up ep contexts */
    for (unsigned int ifnum = 0; ifnum < max_ifnum; ++ifnum)
    {
        struct usb_interface *ifdesc = &cfg->if_desc[ifnum];
        int err = xhci_init_ep_contexts_if(udev, ctrl, virt_dev, ifdesc);
        if (err != UHIOERR_NO_ERROR)
        {
            return err;
        }
    }
    return UHIOERR_NO_ERROR;
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
int xhci_check_maxpacket(struct usb_device *udev, unsigned int max_packet_size)
{
    struct xhci_ctrl *ctrl = udev->controller;
    unsigned int slot_id = udev->slot_id;
    int ep_index = 0; /* control endpoint */
    struct xhci_container_ctx *in_ctx;
    struct xhci_container_ctx *out_ctx;
    struct xhci_input_control_ctx *ctrl_ctx;
    struct xhci_ep_ctx *ep_ctx;
    unsigned int hw_max_packet_size;

    out_ctx = ctrl->devs[slot_id]->out_ctx;
    xhci_inval_cache(out_ctx->bytes, out_ctx->size);
    KprintfH("Checking max packet size for ep 0\n");

    ep_ctx = xhci_get_ep_ctx(ctrl, out_ctx, ep_index);
    hw_max_packet_size = MAX_PACKET_DECODED(LE32(ep_ctx->ep_info2));
    if (hw_max_packet_size != max_packet_size)
    {
        KprintfH("Max Packet Size for ep 0 changed to %ld.\n", max_packet_size);
        KprintfH("Max packet size in xHCI HW = %ld\n", hw_max_packet_size);
        KprintfH("Issuing evaluate context command.\n");

        /* Set up the modified control endpoint 0 */
        xhci_endpoint_copy(ctrl, ctrl->devs[slot_id]->in_ctx,
                           ctrl->devs[slot_id]->out_ctx, ep_index);
        in_ctx = ctrl->devs[slot_id]->in_ctx;
        ep_ctx = xhci_get_ep_ctx(ctrl, in_ctx, ep_index);
        ep_ctx->ep_info2 &= LE32(~MAX_PACKET(MAX_PACKET_MASK));
        ep_ctx->ep_info2 |= LE32(MAX_PACKET(max_packet_size));

        /*
         * Set up the input context flags for the command
         * FIXME: This won't work if a non-default control endpoint
         * changes max packet sizes.
         */
        ctrl_ctx = xhci_get_input_control_ctx(in_ctx);
        ctrl_ctx->add_flags = LE32(EP0_FLAG);
        ctrl_ctx->drop_flags = 0;

        return 1;
    }
    return 0;
}