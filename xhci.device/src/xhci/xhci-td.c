#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#else
#include <proto/exec.h>
#endif

#include <xhci/xhci-udev.h>
#include <xhci/xhci-td.h>
#include <xhci/xhci.h>
#include <debug.h>

#ifdef DEBUG
#undef Kprintf
#define Kprintf(fmt, ...) PrintPistorm("[xhci-td] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

#ifdef DEBUG_HIGH
#undef KprintfH
#define KprintfH(fmt, ...) PrintPistorm("[xhci-td] %s: " fmt, __func__, ##__VA_ARGS__)
#endif

struct xhci_td
{
    struct MinNode node;       /* linkage in active TD list */
    struct IOUsbHWReq *req;    /* owning request */
    dma_addr_t completion_trb; /* TRB address we expect a completion for */
    ULONG length;              /* total length for completion accounting */
    BOOL deadline_active;      /* true if deadline_us is valid */
    ULONG deadline_us;         /* absolute deadline in usec, 0 means no timeout */
    BOOL is_rt_iso;            /* true if this TD is part of RT ISO pipeline */
    UWORD trb_count;           /* number of TRBs consumed by this TD */
    dma_addr_t *trb_addrs;     /* DMA addresses for every TRB in this TD */
};

struct TransferDescriptorList {
    struct MinList list;
    struct xhci_ctrl *ctrl;
    APTR memoryPool;
    ULONG queued_trbs;
    ULONG queued_tds;
};

TransferDescriptorList *xhci_td_create_list(struct xhci_ctrl *ctrl)
{
    TransferDescriptorList *td_list = AllocVecPooled(ctrl->memoryPool, sizeof(TransferDescriptorList));
    if (!td_list)
    {
        Kprintf("Failed to alloc TransferDescriptorList\n");
        return NULL;
    }

    NewMinList(&td_list->list);
    td_list->ctrl = ctrl;
    td_list->memoryPool = ctrl->memoryPool;
    td_list->queued_trbs = 0;
    td_list->queued_tds = 0;

    return td_list;
}

void xhci_td_destroy_list(TransferDescriptorList *td_list, UBYTE error_code)
{
    if (!td_list)
        return;

    xhci_td_fail_all(td_list, error_code);

    FreeVecPooled(td_list->memoryPool, td_list);
}

BOOL xhci_td_is_empty(TransferDescriptorList *td_list)
{
    if (!td_list)
        return TRUE;

    return (td_list->list.mlh_TailPred == (struct MinNode *)&td_list->list);
}

BOOL xhci_td_is_expired(TransferDescriptorList *td_list)
{
    if (!td_list)
        return FALSE;

    ULONG now = get_time();

    struct MinNode *n = td_list->list.mlh_Head;
    while (n && n->mln_Succ)
    {
        struct xhci_td *td = (struct xhci_td *)n;
        if (td->deadline_active && (int32_t)(now - td->deadline_us) >= 0)
        {
            KprintfH("Found expired TD req=%lx deadline=%lu now=%lu\n",
                     td->req,
                     (ULONG)td->deadline_us,
                     (ULONG)now);
            return TRUE;
        }
        n = n->mln_Succ;
    }

    return FALSE;
}

ULONG xhci_td_get_queued_trb_count(TransferDescriptorList *td_list)
{
    if (!td_list)
        return 0;

    return td_list->queued_trbs;
}

ULONG xhci_td_get_queued_td_count(TransferDescriptorList *td_list)
{
    if (!td_list)
        return 0;

    return td_list->queued_tds;
}

static struct xhci_td *td_create(TransferDescriptorList *td_list,
                                 struct IOUsbHWReq *io_req,
                                 ULONG timeout_ms,
                                 BOOL is_rt_iso,
                                 dma_addr_t *trb_addresses,
                                 ULONG trb_count)
{
    if (!td_list)
        return NULL;

    struct xhci_td *td = AllocVecPooled(td_list->memoryPool, sizeof(struct xhci_td));
    if (!td)
    {
        Kprintf("Failed to alloc xhci_td\n");
        return NULL;
    }

    _memset(td, 0, sizeof(struct xhci_td));

    td->req = io_req;

    io_req->iouh_DriverPrivate1 = (APTR)((ULONG)io_req->iouh_DriverPrivate1 | REQ_ON_RING);

    td->deadline_active = (timeout_ms != 0);
    td->deadline_us = get_time() + timeout_ms * 1000UL;

    td->length = io_req->iouh_Length;
    td->trb_count = trb_count;
    td->trb_addrs = trb_addresses;
    td->completion_trb = trb_addresses[trb_count - 1];

    td->is_rt_iso = is_rt_iso;

    return td;
}

BOOL xhci_td_add(TransferDescriptorList *td_list,
                 struct IOUsbHWReq *io_req,
                 ULONG timeout_ms,
                 BOOL is_rt_iso,
                 dma_addr_t *trb_addresses,
                 ULONG trb_count)
{
    if (!td_list)
        return FALSE;

    struct xhci_td *td = td_create(td_list,
                                   io_req,
                                   timeout_ms,
                                   is_rt_iso,
                                   trb_addresses,
                                   trb_count);
    if (!td)
        return FALSE;

    td_list->queued_trbs += trb_count;
    td_list->queued_tds++;

    AddTailMinList(&td_list->list, (struct MinNode *)td);
    return TRUE;
}

static struct xhci_td *find_td_by_trb(TransferDescriptorList *td_list, dma_addr_t trb_addr)
{
    if (!td_list)
        return NULL;

    struct MinNode *n = td_list->list.mlh_Head;
    while (n && n->mln_Succ)
    {
        struct xhci_td *td = (struct xhci_td *)n;
        if (td->completion_trb == trb_addr)
            return td;

        if (td->trb_addrs)
        {
            for (unsigned int i = 0; i < td->trb_count; i++)
            {
                if (td->trb_addrs[i] == trb_addr)
                    return td;
            }
        }
        n = n->mln_Succ;
    }

    return NULL;
}

static void xhci_td_decrease_queued(TransferDescriptorList *td_list, struct xhci_td *td)
{
    if (td->trb_count > 0)
    {
        if (td_list->queued_trbs >= td->trb_count)
            td_list->queued_trbs -= td->trb_count;
        else
            td_list->queued_trbs = 0;
    }

    if (td_list->queued_tds > 0)
        td_list->queued_tds--;
}

static void xhci_td_free(TransferDescriptorList *td_list, struct xhci_td *td)
{
    if (td->trb_addrs)
    {
        FreeVecPooled(td_list->memoryPool, td->trb_addrs);
        td->trb_addrs = NULL;
    }

    FreeVecPooled(td_list->memoryPool, td);
}

inline static void xhci_dma_unmap(struct xhci_ctrl *ctrl, struct IOUsbHWReq *req, BOOL copy)
{
	if (!ctrl || !req)
		return;

	APTR addr = req->iouh_Data;
	ULONG size = req->iouh_Length;
	if (!addr || size == 0 || ((ULONG)req->iouh_DriverPrivate1 & REQ_INTERNAL))
		return;

	if (!((ULONG)req->iouh_DriverPrivate1 & REQ_DMA_MAPPED))
	{
		if (copy)
			xhci_inval_cache(addr, size);
		return;
	}

	APTR bounce = (APTR)req->iouh_DriverPrivate2;
	if (!bounce)
	{
		Kprintf("No bounce buffer found for unmap of %lx len=%ld\n", (ULONG)addr, (LONG)size);
		return;
	}

	if (copy)
	{
		xhci_inval_cache(bounce, size);
		CopyMem(bounce, addr, size);
	}

	memalign_free(ctrl->memoryPool, bounce);
	req->iouh_DriverPrivate1 = (APTR)((ULONG)req->iouh_DriverPrivate1 & ~REQ_DMA_MAPPED);
	req->iouh_DriverPrivate2 = NULL;
}

/*
 * This is taking a Transfer Descriptor off the list.
 * If TD containing specified TRB address is found, it is removed from the list.
 * The TD is finalized and freed,
 * if it contains a IOUsbHWReq, the request is returned to the caller.
 */
struct IOUsbHWReq *xhci_td_get_by_trb(TransferDescriptorList *td_list, dma_addr_t trb_addr)
{
    if (!td_list)
        return NULL;

    struct xhci_td *td = find_td_by_trb(td_list, trb_addr);
    if (!td)
        return NULL;

    struct IOUsbHWReq *req = td->req;

    RemoveMinNode((struct MinNode *)td);
    xhci_td_decrease_queued(td_list, td);
    xhci_td_free(td_list, td);

    if (req && req->iouh_Length > 0)
    {
        BOOL need_data;
        if (req->iouh_Req.io_Command == UHCMD_CONTROLXFER)
            need_data = (req->iouh_SetupData.bmRequestType & USB_DIR_IN) != 0;
        else
            need_data = (req->iouh_Dir == UHDIR_IN);

        xhci_dma_unmap(td_list->ctrl, req, need_data);
    }

    return req;
}

void xhci_td_fail_all(TransferDescriptorList *td_list, BYTE io_Error)
{
    if (!td_list)
        return;

    struct MinNode *n;
    while ((n = RemHeadMinList(&td_list->list)) != NULL)
    {
        struct xhci_td *td = (struct xhci_td *)n;
        if (td->req)
        {
            if (td->req->iouh_Data)
                xhci_dma_unmap(td_list->ctrl, td->req, FALSE);
            if (td->is_rt_iso && td->req->iouh_Dir == UHDIR_IN && td->req->iouh_Data)
                FreeVecPooled(td_list->memoryPool, td->req->iouh_Data);
            if (td->is_rt_iso)
                FreeVecPooled(td_list->memoryPool, td->req);
            else
                xhci_udev_io_reply_failed(td->req, io_Error);
        }
        xhci_td_free(td_list, td);
    }

    td_list->queued_trbs = 0;
    td_list->queued_tds = 0;
}

/*
 * Abort the given IOUsbHWReq by removing it from any queues.
 * If it's already on a hardware ring, we won't stop the ring and remove it,
 * well just null out the request so that the completion handler won't try
 * to reply to it.
 */
ULONG xhci_td_abort_req(struct IOUsbHWReq *io)
{
    ULONG aborted = -1;
    Forbid();

    /* If the IO was not quick and is of type message that is not yet on the hardware ring, abord it and remove from queue. */
    if ((io->iouh_Req.io_Flags & IOF_QUICK) == 0 && io->iouh_Req.io_Message.mn_Node.ln_Type == NT_MESSAGE)
    {
        if (!((ULONG)(io->iouh_DriverPrivate1) & REQ_ON_RING))
        {
            Remove(&io->iouh_Req.io_Message.mn_Node);
            xhci_udev_io_reply_failed(io, IOERR_ABORTED);
            aborted = 0;
        }
    }
    Permit();
    return aborted;
}
