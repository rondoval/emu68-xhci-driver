// SPDX-License-Identifier: GPL-2.0+
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

#include <devices/usbhardware.h>

#define LIB_MIN_VERSION 39 /* we use memory pools */
#define DEVICE_PRIORITY 90

#define COMMAND_PROCESSED 1
#define COMMAND_SCHEDULED 0

struct XHCIDevice;

struct XHCIUnit
{
	struct Unit unit;
	APTR memoryPool;

	/* config */
	LONG unitNumber;
	LONG flags;

	struct Task *task;
	struct xhci_ctrl *xhci_ctrl;

	struct Interrupt irq_isr;
	LONG irq_line;
	BYTE irq_signal;
};

struct XHCIDevice
{
	struct Device device;
	ULONG segList;

	struct MinList units;
};

/* Unit interface */
int UnitTaskStart(struct XHCIUnit *unit);
void UnitTaskStop(struct XHCIUnit *unit);

int UnitOpen(struct XHCIUnit *unit, LONG unitNumber, LONG flags);
int UnitClose(struct XHCIUnit *unit);

void ProcessCommand(struct IOUsbHWReq *io);

int xhci_intx_enable(struct XHCIUnit *unit);
void xhci_intx_shutdown(struct XHCIUnit *unit);
void xhci_intx_handle(struct XHCIUnit *unit);

#endif