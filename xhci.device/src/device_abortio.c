// SPDX-License-Identifier: GPL-2.0+
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#else
#include <proto/exec.h>
#endif

#include <device.h>
#include <debug.h>
#include <devices/hcd_api.h>
#include <xhci/xhci-td.h>

LONG abortIO(struct USBIORequest *io asm("a1"), struct XHCIDevice *base asm("a6") __attribute__((unused)))
{
    /* AbortIO is a *wish* call. Someone would like to abort current IORequest */
    KprintfH("[xhci] %s: Aborting IO request %lx\n", __func__, io);

    if (io->req.io_Unit != NULL)
    {
        return xhci_td_abort_req(io);
    }
    return -1;
}
