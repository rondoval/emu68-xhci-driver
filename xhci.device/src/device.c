// SPDX-License-Identifier: GPL-2.0-only
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#include <clib/bcmpcie_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME (*(struct ExecBase **)4UL)
#include <proto/exec.h>
#define BCMPCIE_BASE_NAME pcielibBase
#include <proto/bcmpcie.h>
#endif

#include <exec/types.h>
#include <exec/resident.h>
#include <exec/io.h>
#include <exec/devices.h>
#include <exec/errors.h>
#include <dos/dosextens.h>

#include <devices/hcd_api.h>

#include <libraries/openpci.h>

#include <device.h>
#include <config.h>
#include <minlist.h>
#include <debug.h>

/*
    Put the function at the very beginning of the file in order to avoid
    unexpected results when user executes the device by mistake
*/
int doNotExecute(void);
int __attribute__((used, no_reorder)) doNotExecute(void)
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
APTR initFunction(struct XHCIDevice *base asm("d0"), ULONG segList asm("a0"), struct ExecBase *_SysBase asm("a6"));

static const APTR funcTable[];
static const APTR initTable[4] = {
    (APTR)sizeof(struct XHCIDevice),
    (APTR)funcTable,
    NULL,
    (APTR)initFunction};

void openLib(struct USBIORequest *io asm("a1"), LONG unitNumber asm("d0"), ULONG flags asm("d1"), struct XHCIDevice *base asm("a6"));
ULONG closeLib(struct USBIORequest *io asm("a1"), struct XHCIDevice *base asm("a6"));
ULONG expungeLib(struct XHCIDevice *base asm("a6"));
APTR extFunc(struct XHCIDevice *base asm("a6"));
void beginIO(struct USBIORequest *io asm("a1"), struct XHCIDevice *base asm("a6"));
LONG abortIO(struct USBIORequest *io asm("a1"), struct XHCIDevice *base asm("a6"));

static const APTR funcTable[] = {
    (APTR)openLib,
    (APTR)closeLib,
    (APTR)expungeLib,
    (APTR)extFunc,
    (APTR)beginIO,
    (APTR)abortIO,
    (APTR)-1};

s32 xhci_open_pcie_library(struct XHCIDevice *base)
{
    base->pcieBase = OpenLibrary((CONST_STRPTR) "bcmpcie.library", 1);
    if (base->pcieBase == NULL)
    {
        Kprintf("[xhci] %s: Failed to open %s\n", __func__, "bcmpcie.library");
        return -1;
    }

    struct Library *pcielibBase = base->pcieBase;
    UWORD flags = pci_bus();
    if (!(flags & BCM2711PCIeBus))
    {
        Kprintf("[xhci] %s: %s bus flags=0x%04lx, BCM2711PCIeBus not set\n", __func__, "bcmpcie.library", (ULONG)flags);
        CloseLibrary(pcielibBase);
        base->pcieBase = NULL;
        return -1;
    }

    return 0;
}

static void xhci_close_libraries(struct XHCIDevice *base)
{
    if (base->pcieBase != NULL)
    {
        CloseLibrary(base->pcieBase);
        base->pcieBase = NULL;
    }

    if (base->gic400Base != NULL)
    {
        CloseLibrary(base->gic400Base);
        base->gic400Base = NULL;
    }

    if (base->utilityBase != NULL)
    {
        CloseLibrary(base->utilityBase);
        base->utilityBase = NULL;
    }
}

static s32 xhci_open_libraries(struct XHCIDevice *base)
{
    if (base->utilityBase != NULL && base->gic400Base != NULL)
        return 0;

    xhci_close_libraries(base);

    base->utilityBase = OpenLibrary((CONST_STRPTR) "utility.library", LIB_MIN_VERSION);
    if (base->utilityBase == NULL)
    {
        Kprintf("[xhci] %s: Failed to open utility.library\n", __func__);
        return -1;
    }

    base->gic400Base = OpenLibrary((CONST_STRPTR) "gic400.library", 0);
    if (base->gic400Base == NULL)
    {
        Kprintf("[xhci] %s: Failed to open gic400.library\n", __func__);
        xhci_close_libraries(base);
        return -1;
    }

    return 0;
}

APTR initFunction(struct XHCIDevice *base asm("d0"), ULONG segList asm("a0"), struct ExecBase *_SysBase asm("a6"))
{
    (void)_SysBase;
    KprintfH("[xhci] %s: Initializing device\n", __func__);
    base->segList = segList;
    base->device.dd_Library.lib_Revision = DEVICE_REVISION;
    _NewMinList(&base->units);
    base->utilityBase = NULL;
    base->gic400Base = NULL;
    base->pcieBase = NULL;

    return base;
}

void openLib(struct USBIORequest *io asm("a1"), LONG unitNumber asm("d0"),
             ULONG flags asm("d1"), struct XHCIDevice *base asm("a6"))
{
    BOOL firstOpen = FALSE;
    BOOL createdUnit = FALSE;

    KprintfH("[xhci] %s: Opening device with unit number %ld and flags %lx\n", __func__, unitNumber, flags);

    if (io->req.io_Message.mn_Length < sizeof(struct IOStdReq))
    {
        Kprintf("[xhci] %s: Invalid request length %lu\n", __func__, (ULONG)io->req.io_Message.mn_Length);
        io->req.io_Error = IOERR_OPENFAIL;
        return;
    }

    // Seek through the list of units to find the one with the requested unit number
    struct XHCIUnit *unit = NULL;
    for (struct MinNode *node = base->units.mlh_Head; node->mln_Succ != NULL; node = node->mln_Succ)
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
        KprintfH("[xhci] %s: Allocating unit structure\n", __func__);
        unit = AllocMem(sizeof(struct XHCIUnit), MEMF_FAST | MEMF_PUBLIC | MEMF_CLEAR);
        if (unit == NULL)
        {
            Kprintf("[xhci]%s: Failed to allocate unit\n", __func__);
            io->req.io_Error = IOERR_OPENFAIL;
            return;
        }
        unit->device = base;
        AddTailMinList(&base->units, (struct MinNode *)unit);
        createdUnit = TRUE;
    }

    if (unit->unit.unit_OpenCnt > 0)
    {
        KprintfH("[xhci] %s: Unit is already open, we only support exclusive access\n", __func__);
        io->req.io_Error = IOERR_UNITBUSY;
        return;
    }

    firstOpen = (base->device.dd_Library.lib_OpenCnt == 0);
    if (firstOpen && xhci_open_libraries(base) != 0)
    {
        io->req.io_Error = IOERR_OPENFAIL;
        if (createdUnit)
        {
            RemoveMinNode((struct MinNode *)unit);
            FreeMem(unit, sizeof(struct XHCIUnit));
        }
        return;
    }

    int result = UnitOpen(unit, unitNumber, (LONG)flags);

    if (result == ERR_NO_ERROR)
    {
        KprintfH("[xhci] %s: Unit opened successfully\n", __func__);
        io->req.io_Unit = (struct Unit *)unit;
        base->device.dd_Library.lib_OpenCnt++;
        base->device.dd_Library.lib_Flags &= (UBYTE)~LIBF_DELEXP;
        io->req.io_Message.mn_Node.ln_Type = NT_REPLYMSG;
    }
    else
    {
        Kprintf("[xhci] %s: Failed to open unit, error code %ld\n", __func__, result);
        io->req.io_Error = IOERR_OPENFAIL;

        RemoveMinNode((struct MinNode *)unit);
        FreeMem(unit, sizeof(struct XHCIUnit));

        if (firstOpen)
            xhci_close_libraries(base);
    }

    /* In contrast to normal library there is no need to return anything */
    return;
}

ULONG closeLib(struct USBIORequest *io asm("a1"), struct XHCIDevice *base asm("a6"))
{
    struct XHCIUnit *unit = (struct XHCIUnit *)io->req.io_Unit;
    KprintfH("[xhci] %s: Closing device\n", __func__);

    int result = UnitClose(unit);
    if (result == 0) // last user of Unit disappeared
    {
        KprintfH("[xhci] %s: Unit closed successfully, freeing resources\n", __func__);
        RemoveMinNode((struct MinNode *)unit);
        FreeMem(unit, sizeof(struct XHCIUnit));
    }

    base->device.dd_Library.lib_OpenCnt--;

    if (base->device.dd_Library.lib_OpenCnt == 0)
    {
        xhci_close_libraries(base);
        if (base->device.dd_Library.lib_Flags & LIBF_DELEXP)
        {
            return expungeLib(base);
        }
    }

    return 0;
}

ULONG expungeLib(struct XHCIDevice *base asm("a6"))
{
    KprintfH("[xhci] %s: Expunging device\n", __func__);
    if (base->device.dd_Library.lib_OpenCnt > 0)
    {
        KprintfH("[xhci] %s: Device is still open, cannot expunge\n", __func__);
        base->device.dd_Library.lib_Flags |= LIBF_DELEXP;
        return 0;
    }
    else
    {
        ULONG segList = base->segList;

        /* Remove yourself from list of devices */
        Forbid();
        Remove((struct Node *)base);
        Permit();

        /* Calculate size of device base and deallocate memory */
        ULONG size = (ULONG)(base->device.dd_Library.lib_NegSize + base->device.dd_Library.lib_PosSize);
        APTR pointer = (APTR)((ULONG)base - base->device.dd_Library.lib_NegSize);
        FreeMem(pointer, size);

        return segList;
    }
}

APTR extFunc(struct XHCIDevice *base asm("a6"))
{
    return base;
}
