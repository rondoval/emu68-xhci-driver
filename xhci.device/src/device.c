// SPDX-License-Identifier: GPL-2.0+
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#include <clib/utility_protos.h>
#else
#include <proto/exec.h>
#include <proto/utility.h>
#endif

#include <exec/types.h>
#include <exec/resident.h>
#include <exec/io.h>
#include <exec/devices.h>
#include <exec/errors.h>
#include <dos/dosextens.h>

#include <devices/usbhardware.h>

#include <device.h>
#include <config.h>
#include <minlist.h>
#include <debug.h>

/*
    Put the function at the very beginning of the file in order to avoid
    unexpected results when user executes the device by mistake
*/
int __attribute__((used, no_reorder)) doNotExecute()
{
    return -1;
}

/*
    Put this marker at the very end of your executable. It's not absolutely
    mandatory but will let rom tag scanner of exec.library work better/faster.
*/
extern const UBYTE endOfCode;

/*
    Const fields containing name of device and ID string. Note! It's not the
    version string as in case of executables (i.e. the $VER:), but rather
    "name version.revision (date)" string.
*/
static const char deviceName[] = DEVICE_NAME;
static const char deviceIdString[] = DEVICE_IDSTRING;
static const APTR initTable[4];

/*
    Resident structure describing the object. RTF_AUTOINIT means the rt_Init field
    points to the initializer table defined below. RTF_COLDSTART defines when the
    object will be initialized (coldstart means, before dos.library, after scheduler
    is started)
*/
static struct Resident const xhciDeviceResident __attribute__((used)) = {
    RTC_MATCHWORD,
    (struct Resident *)&xhciDeviceResident,
    (APTR)&endOfCode,
    RTF_AUTOINIT | RTF_AFTERDOS,
    DEVICE_VERSION,
    NT_DEVICE,
    DEVICE_PRIORITY,
    (APTR)&deviceName,
    (APTR)&deviceIdString,
    (APTR)&initTable};

/*
    Initializer table. First field is the size of structure describing the object,
    can be sizeof(struct Library), sizeof(struct Device) or any size necessary to
    store user defined object extending the Device structure.
*/
APTR initFunction(struct XHCIDevice *base asm("d0"), ULONG segList asm("a0"), struct XHCIDevice *dev_base asm("a6"));

static const APTR funcTable[];
static const APTR initTable[4] = {
    (APTR)sizeof(struct XHCIDevice),
    (APTR)funcTable,
    NULL,
    (APTR)initFunction};

void openLib(struct IOUsbHWReq *io asm("a1"), LONG unitNumber asm("d0"), ULONG flags asm("d1"), struct XHCIDevice *base asm("a6"));
ULONG closeLib(struct IOUsbHWReq *io asm("a1"), struct XHCIDevice *base asm("a6"));
ULONG expungeLib(struct XHCIDevice *base asm("a6"));
APTR extFunc(struct XHCIDevice *base asm("a6"));
void beginIO(struct IOUsbHWReq *io asm("a1"), struct XHCIDevice *base asm("a6"));
LONG abortIO(struct IOUsbHWReq *io asm("a1"), struct XHCIDevice *base asm("a6"));

static const APTR funcTable[] = {
    (APTR)openLib,
    (APTR)closeLib,
    (APTR)expungeLib,
    (APTR)extFunc,
    (APTR)beginIO,
    (APTR)abortIO,
    (APTR)-1};

struct ExecBase *SysBase;
struct Library *UtilityBase = NULL;

APTR initFunction(struct XHCIDevice *base asm("d0"), ULONG segList asm("a0"), struct XHCIDevice *dev_base asm("a6") __attribute__((unused)))
{
    SysBase = *((struct ExecBase **)4UL);
    Kprintf("[xhci] %s: Initializing device\n", __func__);
    base->segList = segList;
    base->device.dd_Library.lib_Revision = DEVICE_REVISION;
    _NewMinList(&base->units);

    UtilityBase = OpenLibrary((CONST_STRPTR)"utility.library", LIB_MIN_VERSION);
    if (UtilityBase == NULL)
    {
        Kprintf("[xhci] %s: Failed to open utility.library\n", __func__);
        expungeLib(base);
        return NULL;
    }

    return base;
}

void openLib(struct IOUsbHWReq *io asm("a1"), LONG unitNumber asm("d0"),
             ULONG flags asm("d1"), struct XHCIDevice *base asm("a6"))
{
    Kprintf("[xhci] %s: Opening device with unit number %ld and flags %lx\n", __func__, unitNumber, flags);
    if (unitNumber != 0)
    {
        Kprintf("[xhci] %s: Invalid unit number %ld\n", __func__, unitNumber);
        io->iouh_Req.io_Error = IOERR_OPENFAIL;
        return;
    }

    if (io->iouh_Req.io_Message.mn_Length < sizeof(struct IOStdReq))
    {
        Kprintf("[xhci] %s: Invalid request length %ld\n", __func__, io->iouh_Req.io_Message.mn_Length);
        io->iouh_Req.io_Error = IOERR_OPENFAIL;
        return;
    }

    // Seek through the list of units to find the one with the requested unit number
    struct XHCIUnit *unit = NULL;
    for(struct MinNode *node = base->units.mlh_Head; node->mln_Succ != NULL; node = node->mln_Succ)
    {
        struct XHCIUnit *currentUnit = (struct XHCIUnit *)node;
        if (currentUnit->unitNumber == unitNumber)
        {
            unit = currentUnit;
            break;
        }
    }

    if (unit == NULL)
    {
        Kprintf("[xhci] %s: Allocating unit structure\n", __func__);
        unit = AllocMem(sizeof(struct XHCIUnit), MEMF_FAST | MEMF_PUBLIC | MEMF_CLEAR);
        if (unit == NULL)
        {
            Kprintf("[xhci]%s: Failed to allocate unit\n", __func__);
            io->iouh_Req.io_Error = IOERR_OPENFAIL;
            return;
        }
        AddTailMinList(&base->units, (struct MinNode *)unit);
    }

    if (unit->unit.unit_OpenCnt > 0)
    {
        Kprintf("[xhci] %s: Unit is already open, we only support exclusive access\n", __func__);
        io->iouh_Req.io_Error = IOERR_UNITBUSY;
        return;
    }

    int result = UnitOpen(unit, unitNumber, flags);

    if (result == UHIOERR_NO_ERROR)
    {
        Kprintf("[xhci] %s: Unit opened successfully\n", __func__);
        io->iouh_Req.io_Unit = (struct Unit *)unit;
        base->device.dd_Library.lib_OpenCnt++;
        base->device.dd_Library.lib_Flags &= ~LIBF_DELEXP;
        io->iouh_Req.io_Message.mn_Node.ln_Type = NT_REPLYMSG;
    }
    else
    {
        Kprintf("[xhci] %s: Failed to open unit, error code %ld\n", __func__, result);
        io->iouh_Req.io_Error = IOERR_OPENFAIL;

        // Remove the failed unit from the list and free its memory
        RemoveMinNode((struct MinNode *)unit);
        FreeMem(unit, sizeof(struct XHCIUnit));
    }

    /* In contrast to normal library there is no need to return anything */
    return;
}

ULONG closeLib(struct IOUsbHWReq *io asm("a1"), struct XHCIDevice *base asm("a6"))
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->iouh_Req.io_Unit;
    Kprintf("[xhci] %s: Closing device\n", __func__);

    int result = UnitClose(unit);
    if (result == 0) // last user of Unit disappeared
    {
        Kprintf("[xhci] %s: Unit closed successfully, freeing resources\n", __func__);
        RemoveMinNode((struct MinNode *)unit);
        FreeMem(unit, sizeof(struct XHCIUnit));
    }

    base->device.dd_Library.lib_OpenCnt--;

    if (base->device.dd_Library.lib_OpenCnt == 0)
    {
        if (base->device.dd_Library.lib_Flags & LIBF_DELEXP)
        {
            return expungeLib(base);
        }
    }

    return 0;
}

ULONG expungeLib(struct XHCIDevice *base asm("a6"))
{
    Kprintf("[xhci] %s: Expunging device\n", __func__);
    if (base->device.dd_Library.lib_OpenCnt > 0)
    {
        Kprintf("[xhci] %s: Device is still open, cannot expunge\n", __func__);
        base->device.dd_Library.lib_Flags |= LIBF_DELEXP;
        return 0;
    }
    else
    {
        if (UtilityBase != NULL)
        {
            CloseLibrary(UtilityBase);
            UtilityBase = NULL;
        }

        ULONG segList = base->segList;

        /* Remove yourself from list of devices */
        Forbid();
        Remove((struct Node *)base);
        Permit();

        /* Calculate size of device base and deallocate memory */
        ULONG size = base->device.dd_Library.lib_NegSize + base->device.dd_Library.lib_PosSize;
        APTR pointer = (APTR)((ULONG)base - base->device.dd_Library.lib_NegSize);
        FreeMem(pointer, size);

        return segList;
    }
}

APTR extFunc(struct XHCIDevice *base asm("a6"))
{
    return base;
}
