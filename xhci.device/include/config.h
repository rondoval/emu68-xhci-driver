/* SPDX-License-Identifier: GPL-2.0-only */

#ifndef XHCI_DEVICE_CONFIG_H
#define XHCI_DEVICE_CONFIG_H

#ifndef DEVICE_NAME
#define DEVICE_NAME "xhci.device"
#endif

#ifndef DEVICE_IDSTRING
#define DEVICE_IDSTRING "xhci USB 2.0/3.0 Host Controller Driver"
#endif

#ifndef DEVICE_VERSION
#define DEVICE_VERSION 1
#endif

#ifndef DEVICE_REVISION
#define DEVICE_REVISION 1
#endif

#ifndef DEVICE_USE_MSI
#define DEVICE_USE_MSI TRUE
#endif

/* Prefer MSI-X when the device and controller support it (falls back to MSI
 * then INTx).  Set FALSE to forbid MSI-X for this driver. */
#ifndef DEVICE_USE_MSIX
#define DEVICE_USE_MSIX TRUE
#endif

#define STACK_SIZE 65535
#define UNIT_TASK_PRIORITY 30
#define UNIT_TASK_POLL_DELAY_MS 100
/* Command ring timeout: 5 seconds */
#define CMD_TIMEOUT_MS 5000

/* Target RT ISO IN scheduling horizon in ms */
#define RT_ISO_IN_TARGET_FRAMES 16
#define XHCI_INITIAL_SEGMENTS_PER_RING 1
#define XHCI_SEGMENTS_PER_RING 4
#define XHCI_MAX_SEGMENTS_PER_RING 64 // Hard ceiling for dynamic transfer ring growth

/* Event ring sizing: fixed at startup */
#define XHCI_INITIAL_SEGS_PER_EVENT_RING 8

/* Minimum interval between interrupts (IMOD, in 250 ns units).  The interval
 * between interrupts is longer when the event ring is idle.  160 = 40 us,
 * matching Linux's default interrupt moderation. */
#define IRQ_INTERVAL 160

/* Perf report cadence: unit-task ticks per [xhci] perf_report (~2 s). */
#define XHCI_PROF_REPORT_TICKS 20

#endif