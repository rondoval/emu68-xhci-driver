// SPDX-License-Identifier: GPL-2.0+
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#else
#include <proto/exec.h>
#endif

#include <device.h>
#include <debug.h>
#include <devices/usbhardware.h>

void beginIO(struct IOUsbHWReq *io asm("a1"), struct XHCIDevice *base asm("a6") __attribute__((unused)))
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;

    KprintfH("[xhci] %s: Queuing %04lx\n", __func__, io->iouh_Req.io_Command);
    io->iouh_Req.io_Error = UHIOERR_NO_ERROR;
    io->iouh_Req.io_Flags &= ~IOF_QUICK;
    PutMsg(&unit->unit.unit_MsgPort, (struct Message *)io);
}
