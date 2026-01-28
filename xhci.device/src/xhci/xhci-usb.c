#include <exec/types.h>
#include <xhci/ch9.h>
#include <xhci/xhci.h>
#include <xhci/xhci-root-hub.h>
#include <xhci/xhci-commands.h>
#include <xhci/xhci-endpoint.h>

#include <debug.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-usb] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci-usb] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

u32 xhci_ep_index_from_parts(u8 iouh_endpoint, u8 iouh_dir)
{
    UBYTE ep = iouh_endpoint & 0x0F;
    BOOL dir_in = (iouh_dir == UHDIR_IN);
    if (ep == 0)
        return 0;
    return (ep << 1) - (dir_in ? 0 : 1);
}

int xhci_ep_type_for_index(struct usb_device *udev, u32 ep_index)
{
    if (!udev)
        return -1;

    if (ep_index == 0)
        return USB_ENDPOINT_XFER_CONTROL;

    struct usb_config *cfg = udev->active_config;
    if (!cfg)
        return -1;

    UBYTE ep_num = EP_INDEX_TO_ENDPOINT(ep_index);
    BOOL out = (ep_index & 0x1) != 0;
    UBYTE addr = ep_num | (out ? USB_DIR_OUT : USB_DIR_IN);

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
            UBYTE ep_addr = alt->ep_desc[e].bEndpointAddress;
            unsigned int epnum = ep_addr & 0x0F;
            if (epnum == 0 || epnum >= USB_MAXENDPOINTS)
                continue;

            struct ep_context *ep_ctx = xhci_ep_get_context(udev,epnum);
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

            if (child->parent == udev && child->used)
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
            current->used = 1;

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
