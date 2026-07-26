// SPDX-License-Identifier: GPL-2.0-only
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#include <clib/gic400_protos.h>
#include <clib/bcmpcie_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME (*(struct ExecBase **)4UL)
#include <proto/exec.h>
#define GIC400_BASE_NAME unit->device->gic400Base
#include <proto/gic400.h>
#define BCMPCIE_BASE_NAME pcielibBase
#include <proto/bcmpcie.h>
#endif

#include <iomem.h>
#include <config.h>
#include <debug.h>
#ifdef PROFILE
#include <timing.h>
#endif
#include <libraries/openpci.h>
#include <libraries/pci_constants.h> /* PCI_IRQ_* flags */
#include <libraries/pci_irq.h>
#include <xhci/xhci.h>
#include <device.h>

#define XHCI_IRQ_ACK_MASK (STS_EINT | STS_FATAL | STS_PORT)

/* IMAN writes use the RsvdP bits captured at xhci_irq_start (iman_base has
 * IP/IE cleared).  IP is W1C: writing 1 always is a no-op when not pending,
 * a clear when it is — so both writes need no prior PCIe read. */
static inline void xhci_irq_disable_runtime(struct xhci_ctrl *ctrl)
{
	mmio_write32(ctrl->iman_base | IMAN_IP, &ctrl->ir_set->irq_pending); /* IE=0, clear IP */
}

static inline void xhci_irq_enable_runtime(struct xhci_ctrl *ctrl)
{
	mmio_write32(ctrl->iman_base | IMAN_IP | IMAN_IE, &ctrl->ir_set->irq_pending); /* IE=1, clear IP */
}

static inline void xhci_irq_update_cmd(struct xhci_ctrl *ctrl, BOOL enable)
{
	KprintfT("[xhci] %s: %s CMD_EIE | CMD_HSEIE\n", __func__, enable ? "enabling" : "disabling");
	u32 cmd = mmio_read32(&ctrl->hcor->or_usbcmd);
	if (enable)
		cmd |= (CMD_EIE | CMD_HSEIE);
	else
		cmd &= ~(CMD_EIE | CMD_HSEIE);
	mmio_write32(cmd, &ctrl->hcor->or_usbcmd);
}

static ULONG xhci_int_isr(struct ExecBase *execBase asm("a6"), struct XHCIUnit *unit asm("a1"), ULONG vector asm("d0"))
{
	(void)execBase;
	(void)vector;

	struct xhci_ctrl *ctrl = unit->xhci_ctrl;
	ULONG status = mmio_read32(&ctrl->hcor->or_usbsts) & XHCI_IRQ_ACK_MASK;

	/* USBSTS.EINT is the "is this interrupt ours?" check for a shared INTx line:
	 * when it isn't set we return 0 and let the next interrupt server run. */
	if (!status)
		return 0;

	if (status & STS_FATAL)
	{
		Kprintf("[xhci] %s: fatal status interrupt (USBSTS=0x%08lx)\n", __func__, status);
	}

	/* Acking USBSTS.EINT and gating the interrupter (IMAN) deasserts the xHC's
	 * interrupt for MSI/MSI-X and INTx alike, so no PCIe-config
	 * PCI_COMMAND.INTX_DISABLE masking is needed. */
	mmio_write32(status & XHCI_IRQ_ACK_MASK, &ctrl->hcor->or_usbsts);
	xhci_irq_disable_runtime(ctrl);

#ifdef PROFILE
	/* XP_IRQ_TO_TASK start: overwritten (not accumulated) on coalesced IRQs,
	 * so the sample measures the LAST signal-to-pickup gap. */
	ctrl->irq_t0 = get_time();
#endif
	Signal(unit->task, 1UL << unit->irq_signal);

	return 1;
}

static inline void xhci_setup_isr(struct XHCIUnit *unit)
{
	unit->irq_isr.is_Node.ln_Type = NT_INTERRUPT;
	unit->irq_isr.is_Node.ln_Name = "xhci_isr";
	unit->irq_isr.is_Data = unit;
	unit->irq_isr.is_Code = (APTR)xhci_int_isr;
}

static inline void xhci_irq_start(struct xhci_ctrl *ctrl)
{
	ctrl->iman_base = mmio_read32(&ctrl->ir_set->irq_pending) & ~(IMAN_IP | IMAN_IE);
	xhci_irq_update_cmd(ctrl, TRUE);
	xhci_irq_enable_runtime(ctrl);
}

static inline void xhci_irq_stop(struct xhci_ctrl *ctrl)
{
	xhci_irq_disable_runtime(ctrl);
	xhci_irq_update_cmd(ctrl, FALSE);
}

void xhci_int_rearm(struct XHCIUnit *unit)
{
	/* Re-enabling the xHC interrupter (IMAN) rearms the source for MSI/MSI-X and
	 * INTx alike; events that arrived while masked re-raise the interrupt. */
	xhci_irq_enable_runtime(unit->xhci_ctrl);
}

static s32 xhci_pci_int_enable(struct XHCIUnit *unit)
{
	struct xhci_ctrl *ctrl = unit->xhci_ctrl;
	struct Library *pcielibBase = unit->device->pcieBase;

	ULONG flags = PCI_IRQ_INTX;
	if (DEVICE_USE_MSI)
		flags |= PCI_IRQ_MSI;
	if (DEVICE_USE_MSIX)
		flags |= PCI_IRQ_MSIX;

	ULONG itype = 0;
	LONG rc = pci_irq_attach(pcielibBase, ctrl->pci_dev, &unit->irq_isr, flags, &itype);
	if (rc != 0)
	{
		Kprintf("[xhci] %s: interrupt attach failed: %s (%ld)\n", __func__,
				pcie_strerror(rc), rc);
		return -1;
	}

	Kprintf("[xhci] %s: using %s\n", __func__,
			itype == PCI_IRQ_MSIX ? "MSI-X" : itype == PCI_IRQ_MSI ? "MSI"
																   : "INTx");

	return 0;
}

s32 xhci_int_enable(struct XHCIUnit *unit)
{
	xhci_setup_isr(unit);

	s32 result = 0;
	if (!unit->xhci_ctrl->pci_dev)
		result = AddIntServerEx((ULONG)unit->irq_line, 0, FALSE, &unit->irq_isr);
	else
		result = xhci_pci_int_enable(unit);

	xhci_irq_start(unit->xhci_ctrl);

	return result;
}

void xhci_int_shutdown(struct XHCIUnit *unit)
{
	if (!unit)
		return;

	struct Library *pcielibBase = unit->device->pcieBase;

	xhci_irq_stop(unit->xhci_ctrl);

	if (!unit->xhci_ctrl->pci_dev)
		RemIntServerEx((ULONG)unit->irq_line, &unit->irq_isr);
	else
		pci_irq_detach(pcielibBase, unit->xhci_ctrl->pci_dev, &unit->irq_isr);
}
