// SPDX-License-Identifier: GPL-2.0-only
#ifndef _GENET_DEVICE_H
#define _GENET_DEVICE_H

#if defined(__INTELLISENSE__)
#define asm(x)
#define __attribute__(x)
#endif

#include <exec/devices.h>
#include <exec/types.h>
#include <exec/semaphores.h>
#include <exec/interrupts.h>

#include <devices/hcd_api.h>

#define LIB_MIN_VERSION 39 /* we use memory pools */
#define DEVICE_PRIORITY 90

#define COMMAND_PROCESSED 1
#define COMMAND_SCHEDULED 0

#define CMD_INTERNAL_ABORT_REQUEST (CMD_NONSTD + 0x100)

struct XHCIDevice;

struct XHCIUnit
{
	struct Unit unit;
	APTR memoryPool;
	struct XHCIDevice *device;

	/* config */
	LONG unitNumber;
	LONG flags;

	/* state */
	struct Task *task;
	struct xhci_ctrl *xhci_ctrl;

	struct Interrupt irq_isr;
	u32 irq_line;
	BYTE irq_signal;
	char vendor_str[5];
	char device_str[5];
};

struct XHCIDevice
{
	struct Device device;
	ULONG segList;
	struct Library *utilityBase;
	struct Library *gic400Base;
	struct Library *pcieBase;    /* NULL until first PCIe unit opens */

	struct MinList units;
};

void beginIO(struct USBIORequest *io asm("a1"), struct XHCIDevice *base asm("a6"));
LONG abortIO(struct USBIORequest *io asm("a1"), struct XHCIDevice *base asm("a6"));

/* PCI library: lazy-open on first PCIe unit, tries bcmpcie.library then openpci.library */
s32 xhci_open_pcie_library(struct XHCIDevice *base);

/* Unit interface */
s32 UnitTaskStart(struct XHCIUnit *unit);
void UnitTaskStop(struct XHCIUnit *unit);

s32 UnitOpen(struct XHCIUnit *unit, LONG unitNumber, LONG flags);
s32 UnitClose(struct XHCIUnit *unit);

void ProcessCommand(struct USBIORequest *io);

s32 xhci_int_enable(struct XHCIUnit *unit);
void xhci_int_shutdown(struct XHCIUnit *unit);
void xhci_int_rearm(struct XHCIUnit *unit);

#endif