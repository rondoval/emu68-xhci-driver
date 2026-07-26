// SPDX-License-Identifier: GPL-2.0-only
#ifndef _XHCI_DEVICE_H
#define _XHCI_DEVICE_H

#if defined(__INTELLISENSE__)
#define asm(x)
#define __attribute__(x)
#endif

#include <exec/devices.h>
#include <exec/types.h>
#include <exec/semaphores.h>
#include <exec/interrupts.h>

#include <reset_guard.h>

#include <types.h>
#include <exec/io.h>

#define LIB_MIN_VERSION 39 /* we use memory pools */
#define DEVICE_PRIORITY 90

#define COMMAND_PROCESSED 1
#define COMMAND_SCHEDULED 0

/* Driver-private command: a root-hub transfer the direct path defers to the
 * unit task (struct xhci_rh_submit_msg, xhci-direct.h). */
#define CMD_INTERNAL_RH_SUBMIT (CMD_NONSTD + 0x100)

struct XHCIDevice;

struct XHCIUnit
{
	struct Unit unit;
	APTR memoryPool;
	struct XHCIDevice *device;

	/* config */
	LONG unitNumber;

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
	struct reset_guard resetGuard; /* pre-reset DMA quiesce hooks */

	struct MinList units;
};

void beginIO(struct IORequest *io asm("a1"), struct XHCIDevice *base asm("a6"));
LONG abortIO(struct IORequest *io asm("a1"), struct XHCIDevice *base asm("a6"));

/* Lazy-open bcmpcie.library (v2+) on first PCIe unit access */
s32 xhci_open_pcie_library(struct XHCIDevice *base);

/* Unit interface */
s32 UnitTaskStart(struct XHCIUnit *unit);
void UnitTaskStop(struct XHCIUnit *unit);

s32 UnitOpen(struct XHCIUnit *unit, LONG unitNumber);
s32 UnitClose(struct XHCIUnit *unit);

void ProcessCommand(struct IORequest *io);

s32 xhci_int_enable(struct XHCIUnit *unit);
void xhci_int_shutdown(struct XHCIUnit *unit);
void xhci_int_rearm(struct XHCIUnit *unit);

#endif