// SPDX-License-Identifier: GPL-2.0+
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#else
#include <proto/exec.h>
#endif

#include <device.h>
#include <debug.h>
#include <devices/usbhardware.h>
#include <xhci/xhci-events.h>

LONG abortIO(struct IOUsbHWReq *io asm("a1"), struct XHCIDevice *base asm("a6") __attribute__((unused)))
{
    /* AbortIO is a *wish* call. Someone would like to abort current IORequest */
    KprintfH("[xhci] %s: Aborting IO request %lx\n", __func__, io);

    if (io->iouh_Req.io_Unit != NULL)
    {
        Forbid();
        /* If the IO was not quick and is of type message (not handled yet or in process), abord it and remove from queue. 
         * The TX task clears ln_Pred to indicate the request is already on TX ring and can't be cancelled. */
        if ((io->iouh_Req.io_Flags & IOF_QUICK) == 0 && io->iouh_Req.io_Message.mn_Node.ln_Type == NT_MESSAGE)
        {
            if (io->iouh_Req.io_Message.mn_Node.ln_Pred != NULL)
                Remove(&io->iouh_Req.io_Message.mn_Node);
            else if (io->iouh_DriverPrivate1 != NULL && io->iouh_DriverPrivate1 != (APTR)0xDEAD001)
            {
                struct xhci_td *td = (struct xhci_td *)io->iouh_DriverPrivate1;
                td->req = NULL; /* prevent completion */
            }
            io->iouh_Req.io_Error = IOERR_ABORTED;
            ReplyMsg(&io->iouh_Req.io_Message);
        }
        Permit();
    }
    return 0;
}
