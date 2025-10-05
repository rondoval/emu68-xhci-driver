// SPDX-License-Identifier: GPL-2.0+
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#include <clib/utility_protos.h>
#else
#include <proto/exec.h>
#include <proto/utility.h>
#endif

#include <devices/usb.h>
#include <devices/usbhardware.h>

#include <xhci/usb.h>
#include <xhci/xhci.h>
#include <xhci/xhci-commands.h>

#include <device.h>
#include <debug.h>
#include <compat.h>
#include <minlist.h>

#include "usb_glue.h"

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

static struct usb_device *resolve_parent(struct XHCIUnit *unit, UWORD addr)
{
    if (addr > 127)
        return NULL;

    struct usb_device *udev = &unit->xhci_ctrl->devices[addr];
    if (!udev->used)
        return NULL;

    return udev;
}

static void update_device_topology(struct XHCIUnit *unit, struct usb_device *udev,
                                   const struct IOUsbHWReq *io)
{
    if (io->iouh_Flags & UHFF_SPLITTRANS)
    {
        struct usb_device *parent = resolve_parent(unit, io->iouh_SplitHubAddr & 0x7F);
        if (parent)
        {
            unsigned int port = io->iouh_SplitHubPort & 0xFF;
            udev->parent = parent;
            udev->portnr = port;
            udev->route = build_route_string(parent, port);
            udev->tt_slot = parent->slot_id;
            udev->tt_port = port;
            return;
        }
        /* Fall back to default routing if the parent isn't known yet */
        udev->parent = NULL;
        udev->portnr = 0;
        udev->route = 0;
        udev->tt_slot = 0;
        udev->tt_port = 0;
        return;
    }

    if (io->iouh_DevAddr == 0)
    {
        /* Default-address transfers without split data target the root hub path */
        udev->parent = NULL;
        udev->portnr = 0;
        udev->route = 0;
        udev->tt_slot = 0;
        udev->tt_port = 0;
    }
}

static struct usb_device *get_or_init_udev(struct XHCIUnit *unit, UWORD devaddr)
{
    if (devaddr > 127)
        return NULL;

    struct usb_device *udev = &unit->xhci_ctrl->devices[devaddr];
    if (!udev->used)
    {
        KprintfH("get_or_init_udev: new device addr=%ld\n", (LONG)devaddr);
        _memset(udev, 0, sizeof(struct usb_device));
        udev->used = 1;
        udev->devnum = devaddr;
        udev->controller = unit->xhci_ctrl;
        /* Default EP0 packet size for new address (8) */
        udev->maxpacketsize0 = PACKET_SIZE_8;
        /* Default toggles cleared */

        _NewMinList(&udev->configurations);

        for (int i = 0; i < USB_MAXENDPOINTS; ++i)
            _NewMinList(&udev->ep_context[i].req_list);
    }
    else
    {
        KprintfH("get_or_init_udev: existing device addr=%ld\n", (LONG)devaddr);
    }
    return udev;
}

static void parse_config_descriptor(struct usb_device *udev, UBYTE *data, UWORD len)
{
    if (len < 2)
    {
        Kprintf("parse_config_descriptor: too short len=%ld\n", (LONG)len);
        return;
    }

    struct usb_config *conf = AllocVecPooled(udev->controller->memoryPool, sizeof(*conf));
    if (!conf)
    {
        Kprintf("parse_config_descriptor: AllocVecPooled failed\n");
        return;
    }
    _memset(conf, 0, sizeof(*conf));

    struct usb_config_descriptor *desc = (struct usb_config_descriptor *)data;
    if (desc->bDescriptorType != USB_DT_CONFIG)
    {
        Kprintf("parse_config_descriptor: bad desc type %ld\n", (LONG)desc->bDescriptorType);
        goto error;
    }

    UWORD total_len = LE16(desc->wTotalLength);
    if ((UWORD)len < total_len)
    {
        Kprintf("parse_config_descriptor: short buffer len=%ld total_len=%ld\n", (LONG)len, (LONG)total_len);
        total_len = (UWORD)len;
    }

    UBYTE *cursor = data;
    UBYTE *end = data + total_len;
    if (cursor + desc->bLength > end)
    {
        Kprintf("parse_config_descriptor: bad desc length %ld\n", (LONG)desc->bLength);
        goto error;
    }

    CopyMem(desc, &conf->desc, sizeof(struct usb_config_descriptor));
    cursor += desc->bLength;

    KprintfH("parse_config_descriptor: wTotalLength=%ld bNumInterfaces=%ld bConfigurationValue=%ld iConfiguration=%ld bmAttributes=0x%02lx bMaxPower=%ld\n",
            (LONG)LE16(desc->wTotalLength),
            (LONG)desc->bNumInterfaces,
            (LONG)desc->bConfigurationValue,
            (LONG)desc->iConfiguration,
            (LONG)desc->bmAttributes,
            (LONG)desc->bMaxPower);

    // TODO in 3.x there are association descriptors here
    conf->no_of_if = desc->bNumInterfaces;
    int if_index = 0;
    int ep_index = 0;
    struct usb_interface *current_if = NULL;

    while (cursor + 2 <= end)
    {
        UBYTE dlen = cursor[0];
        UBYTE dtype = cursor[1];
        if (dlen == 0)
        {
            Kprintf("parse_config_descriptor: zero length descriptor, aborting\n");
            break;
        }
        if (cursor + dlen > end)
        {
            Kprintf("parse_config_descriptor: descriptor overruns buffer (type=%ld len=%ld)\n",
                    (LONG)dtype, (LONG)dlen);
            break;
        }

        switch (dtype)
        {
        case USB_DT_INTERFACE:
        {
            struct usb_interface_descriptor *ifd = (struct usb_interface_descriptor *)cursor;
            if (if_index >= USB_MAXINTERFACES)
            {
                Kprintf("parse_config_descriptor: too many interfaces (%ld)\n", (LONG)if_index);
                goto error;
            }

            CopyMem(ifd, &conf->if_desc[if_index].desc, sizeof(struct usb_interface_descriptor));
            conf->if_desc[if_index].no_of_ep = ifd->bNumEndpoints;
            conf->if_desc[if_index].num_altsetting = ifd->bAlternateSetting;
            KprintfH("parse_config_descriptor: interface %ld: bInterfaceNumber=%ld bAlternateSetting=%ld bNumEndpoints=%ld bInterfaceClass=0x%02lx bInterfaceSubClass=0x%02lx bInterfaceProtocol=0x%02lx iInterface=%ld\n",
                    (LONG)if_index,
                    (LONG)ifd->bInterfaceNumber,
                    (LONG)ifd->bAlternateSetting,
                    (LONG)ifd->bNumEndpoints,
                    (LONG)ifd->bInterfaceClass,
                    (LONG)ifd->bInterfaceSubClass,
                    (LONG)ifd->bInterfaceProtocol,
                    (LONG)ifd->iInterface);

            current_if = &conf->if_desc[if_index];
            if_index++;
            ep_index = 0; // reset endpoint index for new interface
            break;
        }
        case USB_DT_ENDPOINT:
        {
            if (!current_if)
            {
                Kprintf("parse_config_descriptor: endpoint without interface\n");
                break;
            }
            if (ep_index >= USB_MAXENDPOINTS)
            {
                Kprintf("parse_config_descriptor: too many endpoints for interface %ld\n", (LONG)(if_index - 1));
                break;
            }

            struct usb_endpoint_descriptor *epd = (struct usb_endpoint_descriptor *)cursor;
            CopyMem(epd, &current_if->ep_desc[ep_index], sizeof(struct usb_endpoint_descriptor));
            KprintfH("  parse_config_descriptor: endpoint %ld: bEndpointAddress=0x%02lx bmAttributes=0x%02lx wMaxPacketSize=%ld bInterval=%ld\n",
                    (LONG)ep_index,
                    (LONG)epd->bEndpointAddress,
                    (LONG)epd->bmAttributes,
                    (LONG)LE16(epd->wMaxPacketSize),
                    (LONG)epd->bInterval);

            ep_index++;
            break;
        }
        case USB_DT_SS_ENDPOINT_COMP:
        {
            KprintfH("parse_config_descriptor: found SS EP COMP descriptor\n");
            if (current_if && ep_index > 0)
            {
                struct usb_ss_ep_comp_descriptor *comp = (struct usb_ss_ep_comp_descriptor *)cursor;
                CopyMem(comp, &current_if->ss_ep_comp_desc[ep_index - 1], sizeof(struct usb_ss_ep_comp_descriptor));
            }
            break;
        }
        default:
            // Skip class- or vendor-specific descriptors gracefully.
            KprintfH("parse_config_descriptor: found class/vendor-specific descriptor 0x%lx, len=%ld\n", (ULONG)dtype, (LONG)dlen);
            break;
        }

        cursor += dlen;
    }
    KprintfH("parse_config_descriptor: parsed config with %ld interfaces\n", (LONG)conf->no_of_if);

    if (conf->no_of_if != if_index)
    {
        Kprintf("parse_config_descriptor: interface count mismatch %ld != %ld\n", (LONG)conf->no_of_if, (LONG)if_index);
        goto error;
    }

    for (struct MinNode *n = udev->configurations.mlh_Head; n->mln_Succ; n = n->mln_Succ)
    {
        struct usb_config *oldconf = (struct usb_config *)n;
        if (oldconf->desc.bConfigurationValue == conf->desc.bConfigurationValue)
        {
            Kprintf("parse_config_descriptor: removing old config with value %ld\n", (LONG)oldconf->desc.bConfigurationValue);
            RemoveMinNode(n);
            FreeVecPooled(udev->controller->memoryPool, oldconf);
            break;
        }
    }
    AddHeadMinList(&udev->configurations, (struct MinNode *)conf);
    // TODO why AddTailMinList doesn't work here?

    return;

error:
    FreeVecPooled(udev->controller->memoryPool, conf);
}

int usb_glue_ctrl(struct XHCIUnit *unit, struct IOUsbHWReq *io)
{
    struct usb_device *udev = get_or_init_udev(unit, io->iouh_DevAddr);
    if (!udev)
    {
        io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
        return COMMAND_PROCESSED;
    }

    /* Update EP0 MPS hint from IO if provided */
    unsigned int maxpkt_hint = (unsigned int)io->iouh_MaxPktSize;
    if (maxpkt_hint == 8)
        udev->maxpacketsize0 = PACKET_SIZE_8;
    else if (maxpkt_hint == 16)
        udev->maxpacketsize0 = PACKET_SIZE_16;
    else if (maxpkt_hint == 32)
        udev->maxpacketsize0 = PACKET_SIZE_32;
    else
        udev->maxpacketsize0 = PACKET_SIZE_64;

    if (io->iouh_Flags & UHFF_HIGHSPEED)
        udev->speed = USB_SPEED_HIGH;
    else if (io->iouh_Flags & UHFF_LOWSPEED)
        udev->speed = USB_SPEED_LOW;
    else
        udev->speed = USB_SPEED_FULL;

    update_device_topology(unit, udev, io);

    {
        Kprintf("usb_glue_ctrl: dev=%lx addr=%ld bmReqType=%02lx bReq=%02lx wValue=%04lx wIndex=%04lx wLength=%04lx flags=%lx timeout=%ld maxPktSize=%ld\n",
                (ULONG)udev, (ULONG)udev->devnum,
                (ULONG)io->iouh_SetupData.bmRequestType,
                (ULONG)io->iouh_SetupData.bRequest,
                (ULONG)io->iouh_SetupData.wValue,
                (ULONG)io->iouh_SetupData.wIndex,
                (ULONG)io->iouh_SetupData.wLength,
                (ULONG)io->iouh_Flags,
                (ULONG)io->iouh_NakTimeout,
                (ULONG)io->iouh_MaxPktSize);

        KprintfH("usb_glue_ctrl: split_hub_addr=%lx split_hub_port=%ld\n",
                (ULONG)io->iouh_SplitHubAddr,
                (ULONG)io->iouh_SplitHubPort);

        KprintfH("usb_glue_ctrl: parent=%lx portnr=%ld route=%ld tt_slot=%ld tt_port=%ld\n",
                (ULONG)udev->parent,
                (ULONG)udev->portnr,
                (ULONG)udev->route,
                (ULONG)udev->tt_slot,
                (ULONG)udev->tt_port);
        KprintfH("usb_glue_ctrl: maxpacketsize0=%lx speed=%ld\n",
                (ULONG)udev->maxpacketsize0,
                (LONG)udev->speed);
    }

	struct xhci_ctrl *ctrl = unit->xhci_ctrl;
	struct UsbSetupData *setup = &io->iouh_SetupData;

	if (io->iouh_DevAddr == ctrl->rootdev)
	{
		xhci_submit_root(udev, io);
		return COMMAND_PROCESSED;
	}

	if (setup->bRequest == USB_REQ_SET_ADDRESS && (setup->bmRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD)
	{
		xhci_address_device(udev, io);
		return COMMAND_SCHEDULED;
	}

	if (setup->bRequest == USB_REQ_SET_CONFIGURATION &&
		(setup->bmRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD)
	{
		int ret = xhci_set_configuration(udev, LE16(setup->wValue) & 0xff);
		if (ret != UHIOERR_NO_ERROR)
		{
			Kprintf("Failed to configure xHCI endpoint\n");
            io->iouh_Req.io_Error = ret;
			return COMMAND_PROCESSED;
		}

		// this will trigger a chain of commands and control xfer
		xhci_configure_endpoints(udev, FALSE, io);
		return COMMAND_SCHEDULED;
	}

	unsigned int timeout_ms = 0;
	if ((io->iouh_Flags & UHFF_NAKTIMEOUT))
		timeout_ms = io->iouh_NakTimeout;

	int ret = xhci_ctrl_tx(udev, io, timeout_ms);
    if (ret != UHIOERR_NO_ERROR)
    {
        io->iouh_Req.io_Error = ret;
        return COMMAND_PROCESSED;
    }

    return COMMAND_SCHEDULED;
}

int usb_glue_bulk(struct XHCIUnit *unit, struct IOUsbHWReq *io)
{
    struct usb_device *udev = get_or_init_udev(unit, io->iouh_DevAddr);
    if (!udev)
    {
        io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
        return COMMAND_PROCESSED;
    }
    UWORD ep = io->iouh_Endpoint & 0x0F;
    UWORD dir = (io->iouh_Dir == UHDIR_IN) ? 1 : 0;

    Kprintf("usb_glue_bulk: dev=%ld addr=%ld ep=%ld dir=%s len=%ld flags=%lx tmo=%ld\n",
            (ULONG)udev, (ULONG)udev->devnum, (LONG)ep, dir ? "IN" : "OUT",
            (LONG)io->iouh_Length, (ULONG)io->iouh_Flags, (LONG)io->iouh_NakTimeout);

    unsigned int timeout_ms = 0;
    if ((io->iouh_Flags & UHFF_NAKTIMEOUT))
        timeout_ms = (unsigned int)io->iouh_NakTimeout;

    int ret = xhci_bulk_tx(udev, io, timeout_ms);
    if (ret != UHIOERR_NO_ERROR)
    {
        io->iouh_Req.io_Error = ret;
        return COMMAND_PROCESSED;
    }

    return COMMAND_SCHEDULED;
}

int usb_glue_int(struct XHCIUnit *unit, struct IOUsbHWReq *io)
{
    struct usb_device *udev = get_or_init_udev(unit, io->iouh_DevAddr);
    if (!udev)
    {
        io->iouh_Req.io_Error = UHIOERR_BADPARAMS;
        return COMMAND_PROCESSED;
    }
    UWORD ep = io->iouh_Endpoint & 0x0F;
    UWORD dir = (io->iouh_Dir == UHDIR_IN) ? 1 : 0;

    /* Root hub doesn't have a real interrupt endpoint ring here; synthesize success */
    if (udev->devnum == unit->xhci_ctrl->rootdev)
    {
        io->iouh_Actual = 0;
        io->iouh_Req.io_Error = UHIOERR_NO_ERROR;
        // TODO do we need to process it somehow? - yes, these are port state changes
        return COMMAND_PROCESSED;
    }

    Kprintf("usb_glue_int: dev=%lx addr=%ld ep=%ld dir=%s len=%ld interval=%ld flags=%lx timeout=%ld maxpkt=%ld\n",
            (ULONG)udev, (ULONG)udev->devnum, (LONG)ep, dir ? "IN" : "OUT",
            (LONG)io->iouh_Length, (LONG)io->iouh_Interval,
            (ULONG)io->iouh_Flags, (LONG)io->iouh_NakTimeout, (LONG)io->iouh_MaxPktSize);

    unsigned int timeout_ms = 0;
    if ((io->iouh_Flags & UHFF_NAKTIMEOUT))
        timeout_ms = (unsigned int)io->iouh_NakTimeout;

    int ret = xhci_bulk_tx(udev, io, timeout_ms);
    if (ret != UHIOERR_NO_ERROR)
    {
        io->iouh_Req.io_Error = ret;
        return COMMAND_PROCESSED;
    }

    return COMMAND_SCHEDULED;
}

static inline int root_hub_max_ports(struct xhci_ctrl *ctrl)
{
    return HCS_MAX_PORTS(xhci_readl(&ctrl->hccr->cr_hcsparams1));
}

int usb_glue_bus_reset(struct XHCIUnit *unit)
{
    /* Issue SET_FEATURE(RESET) on all root hub ports */
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    struct usb_device *udev = get_or_init_udev(unit, ctrl->rootdev);
    if (!udev)
        return UHIOERR_HOSTERROR;
    int maxp = root_hub_max_ports(ctrl);
    for (int p = 1; p <= maxp; ++p)
    {
        struct IOUsbHWReq req;
        _memset(&req, 0, sizeof(req));
        req.iouh_SetupData.bmRequestType = USB_DIR_OUT | USB_RT_PORT; /* class=hub, recipient=other */
        req.iouh_SetupData.bRequest = USB_REQ_SET_FEATURE;
        req.iouh_SetupData.wValue = cpu_to_le16(USB_PORT_FEAT_RESET);
        req.iouh_SetupData.wIndex = cpu_to_le16(p);
        req.iouh_SetupData.wLength = cpu_to_le16(0);
        req.iouh_DevAddr = udev->devnum;
        req.iouh_MaxPktSize = 8U << udev->maxpacketsize0;

        xhci_submit_root(udev, &req);
    }
    return UHIOERR_NO_ERROR;
}

int usb_glue_bus_suspend(struct XHCIUnit *unit)
{
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    struct usb_device *udev = get_or_init_udev(unit, ctrl->rootdev);
    if (!udev)
        return UHIOERR_HOSTERROR;
    int maxp = root_hub_max_ports(ctrl);
    for (int p = 1; p <= maxp; ++p)
    {
        struct IOUsbHWReq req;
        _memset(&req, 0, sizeof(req));
        req.iouh_SetupData.bmRequestType = USB_DIR_OUT | USB_RT_PORT;
        req.iouh_SetupData.bRequest = USB_REQ_SET_FEATURE;
        req.iouh_SetupData.wValue = cpu_to_le16(USB_PORT_FEAT_SUSPEND);
        req.iouh_SetupData.wIndex = cpu_to_le16(p);
        req.iouh_DevAddr = udev->devnum;
        req.iouh_MaxPktSize = 8U << udev->maxpacketsize0;

        xhci_submit_root(udev, &req);
    }
    return UHIOERR_NO_ERROR;
}

int usb_glue_bus_resume(struct XHCIUnit *unit)
{
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    struct usb_device *udev = get_or_init_udev(unit, ctrl->rootdev);
    if (!udev)
        return UHIOERR_HOSTERROR;
    int maxp = root_hub_max_ports(ctrl);
    for (int p = 1; p <= maxp; ++p)
    {
        struct IOUsbHWReq req;
        _memset(&req, 0, sizeof(req));
        req.iouh_SetupData.bmRequestType = USB_DIR_OUT | USB_RT_PORT;
        req.iouh_SetupData.bRequest = USB_REQ_CLEAR_FEATURE;
        req.iouh_SetupData.wValue = cpu_to_le16(USB_PORT_FEAT_SUSPEND);
        req.iouh_SetupData.wIndex = cpu_to_le16(p);
        req.iouh_DevAddr = udev->devnum;
        req.iouh_MaxPktSize = 8U << udev->maxpacketsize0;

        xhci_submit_root(udev, &req);
    }
    return UHIOERR_NO_ERROR;
}

int usb_glue_bus_oper(struct XHCIUnit *unit)
{
    /* Ensure port power is on for all ports */
    struct xhci_ctrl *ctrl = unit->xhci_ctrl;
    struct usb_device *udev = get_or_init_udev(unit, ctrl->rootdev);
    if (!udev)
        return UHIOERR_HOSTERROR;
    int maxp = root_hub_max_ports(ctrl);
    for (int p = 1; p <= maxp; ++p)
    {
        struct IOUsbHWReq req;
        _memset(&req, 0, sizeof(req));
        req.iouh_SetupData.bmRequestType = USB_DIR_OUT | USB_RT_PORT;
        req.iouh_SetupData.bRequest = USB_REQ_SET_FEATURE;
        req.iouh_SetupData.wValue = cpu_to_le16(USB_PORT_FEAT_POWER);
        req.iouh_SetupData.wIndex = cpu_to_le16(p);
        req.iouh_DevAddr = udev->devnum;
        req.iouh_MaxPktSize = 8U << udev->maxpacketsize0;

        xhci_submit_root(udev, &req);
    }
    return UHIOERR_NO_ERROR;
}

static void parse_control_msg(struct usb_device *udev, struct IOUsbHWReq *io)
{
    KprintfH("parse_control_msg: dev=%lx addr=%ld bmReqType=%02lx bReq=%02lx wValue=%04lx wIndex=%04lx wLength=%04lx actual=%ld\n",
            (ULONG)udev, (ULONG)udev->devnum,
            (ULONG)io->iouh_SetupData.bmRequestType,
            (ULONG)io->iouh_SetupData.bRequest,
            (ULONG)io->iouh_SetupData.wValue,
            (ULONG)io->iouh_SetupData.wIndex,
            (ULONG)io->iouh_SetupData.wLength,
            (LONG)io->iouh_Actual);
    
    /* If this was a successful GET_DESCRIPTOR(STRING) IN transfer and we got
     * an odd number of bytes, clamp to even to avoid a dangling half UTF-16
     * code unit, which Poseidon may display as a trailing '?'.
     */
    // if ((io->iouh_SetupData.bRequest == USB_REQ_GET_DESCRIPTOR) &&
    //     (io->iouh_SetupData.bmRequestType & USB_DIR_IN) && (((LE16(io->iouh_SetupData.wValue)) >> 8) == USB_DT_STRING))
    // {
    //     if ((io->iouh_Actual & 1) != 0)
    //     {
    //         Kprintf("usb_glue_ctrl: clamping odd string length %ld to %ld\n",
    //                 (LONG)io->iouh_Actual, (LONG)(io->iouh_Actual - 1));
    //         io->iouh_Actual -= 1;
    //     }
    // }

    /* If this was a successful GET_DESCRIPTOR(CONFIGURATION),
     * cache the configuration descriptor for later use.
     */
    if ((io->iouh_SetupData.bRequest == USB_REQ_GET_DESCRIPTOR) && (io->iouh_SetupData.bmRequestType & USB_DIR_IN) && (((LE16(io->iouh_SetupData.wValue)) >> 8) == USB_DT_CONFIG))
    {
        parse_config_descriptor(udev, (UBYTE *)io->iouh_Data, (UWORD)io->iouh_Actual);
    }

    /* If this was a successful standard SET_ADDRESS, migrate the glue context
     * from the old devaddr to the new one so subsequent transfers reuse the
     * same slot and endpoint state.
     */
    if ((io->iouh_SetupData.bRequest == USB_REQ_SET_ADDRESS) &&
        ((io->iouh_SetupData.bmRequestType & USB_TYPE_MASK) == USB_TYPE_STANDARD))
    {
        UWORD old_addr = io->iouh_DevAddr & 0x7F;
        UWORD new_addr = (UWORD)(LE16(io->iouh_SetupData.wValue) & 0x7F);
        if (new_addr != old_addr)
        {
            struct xhci_ctrl *ctrl = udev->controller;
            struct usb_device *from = &ctrl->devices[old_addr];
            struct usb_device *to = &ctrl->devices[new_addr];

            if (to->used)
            {
                Kprintf("usb_glue_ctrl: overwriting existing ctx for addr %ld\n", (LONG)new_addr);
                _memset(to, 0, sizeof(*to));
            }

            /* Copy the entire device context so slot/endpoint state follows the device. */
            *to = *from;
            to->used = 1;
            to->devnum = new_addr;
            ctrl->devs[to->slot_id]->udev = to;

            /* Clear the old address entry to avoid stale references. */
            _memset(from, 0, sizeof(*from));
            Kprintf("usb_glue_ctrl: migrated ctx from addr %ld to %ld\n", (LONG)old_addr, (LONG)new_addr);
        }
    }
}

/* Hooks for responding to requests for lower layer */
void io_reply_failed(struct IOUsbHWReq *io, int err)
{
    if (io)
    {
        Kprintf("io_reply_failed: err=%ld\n", (LONG)err);
        io->iouh_Req.io_Error = err;
        ReplyMsg((struct Message *)io);
    }
}

void io_reply_data(struct usb_device *udev, struct IOUsbHWReq *io, int err, ULONG actual)
{
    if (!io)
        return;

    io->iouh_Actual = actual;
    io->iouh_Req.io_Error = err;

    if (io->iouh_Req.io_Command == UHCMD_CONTROLXFER && err == UHIOERR_NO_ERROR)
        parse_control_msg(udev, io);

    Kprintf("io_reply_data: err=%ld actual=%ld\n", (LONG)err, (LONG)actual);
    ReplyMsg((struct Message *)io);
}