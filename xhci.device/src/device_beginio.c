// SPDX-License-Identifier: GPL-2.0+
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME (*(struct ExecBase **)4UL)
#include <proto/exec.h>
#endif

#include <device.h>
#include <debug.h>
#include <devices/hcd_api.h>

void beginIO(struct USBIORequest *io asm("a1"), struct XHCIDevice *base asm("a6") __attribute__((unused)))
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->req.io_Unit;

    io->req.io_Error = ERR_NO_ERROR;
    io->req.io_Flags &= ~IOF_QUICK;
    PutMsg(&unit->unit.unit_MsgPort, (struct Message *)io);
}
