// SPDX-License-Identifier: GPL-2.0-only
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME (*(struct ExecBase **)4UL)
#include <proto/exec.h>
#endif

#include <device.h>
#include <debug.h>
#include <memory.h>
#include <devices/hcd_api.h>
#include <xhci/xhci-udev.h>

static LONG post_abort_request(struct XHCIUnit *unit, struct USBIORequest *io)
{
    if (!unit || !unit->memoryPool)
        return -1;

    struct USBIORequest *abort_req = pool_zalloc(unit->memoryPool, sizeof(*abort_req));
    if (!abort_req)
        return -1;

    abort_req->req.io_Message.mn_Length = sizeof(*abort_req);
    abort_req->req.io_Unit = io->req.io_Unit;
    abort_req->req.io_Command = CMD_INTERNAL_ABORT_REQUEST;
    abort_req->req.io_Flags = IOF_QUICK;
    abort_req->data_buffer = io;
    abort_req->driver_private_flags = REQ_INTERNAL;

    PutMsg(&unit->unit.unit_MsgPort, (struct Message *)abort_req);
    return 0;
}

LONG abortIO(struct USBIORequest *io asm("a1"), struct XHCIDevice *base asm("a6") __attribute__((unused)))
{
    /* AbortIO is a *wish* call. Someone would like to abort current IORequest */
    KprintfT("[xhci] %s: Aborting IO request %lx\n", __func__, io);
    if (!io)
        return -1;

    if (io->req.io_Unit != NULL)
        return post_abort_request((struct XHCIUnit *)io->req.io_Unit, io);

    return -1;
}
