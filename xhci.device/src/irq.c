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
#include <libraries/openpci.h>
#include <libraries/pci_constants.h> /* PCI_IRQ_* flags */
#include <xhci/xhci.h>
#include <xhci/xhci-events.h>
#include <device.h>

#define XHCI_IRQ_ACK_MASK (STS_EINT | STS_FATAL | STS_PORT)

static inline void xhci_irq_disable_runtime(struct xhci_ctrl *ctrl)
{
	u32 iman = mmio_read32(&ctrl->ir_set->irq_pending);
	mmio_write32(ER_IRQ_DISABLE(iman) | ER_IRQ_PENDING(iman), &ctrl->ir_set->irq_pending);
}

static inline void xhci_irq_enable_runtime(struct xhci_ctrl *ctrl)
{
	u32 iman = mmio_read32(&ctrl->ir_set->irq_pending);
	mmio_write32(ER_IRQ_ENABLE(iman) | ER_IRQ_PENDING(iman), &ctrl->ir_set->irq_pending);
}

static inline void xhci_irq_update_cmd(struct xhci_ctrl *ctrl, BOOL enable)
{
	KprintfH("[xhci] %s: %s CMD_EIE | CMD_HSEIE\n", __func__, enable ? "enabling" : "disabling");
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
	struct Library *pcielibBase = unit->device->pcieBase;
	ULONG status = mmio_read32(&ctrl->hcor->or_usbsts) & XHCI_IRQ_ACK_MASK;

	if (!status)
		return 0;

	if (status & STS_FATAL)
	{
		Kprintf("[xhci] %s: fatal status interrupt (USBSTS=0x%08lx)\n", __func__, status);
	}

	mmio_write32(status & XHCI_IRQ_ACK_MASK, &ctrl->hcor->or_usbsts);
	xhci_irq_disable_runtime(ctrl);

	if (ctrl->pci_dev)
	{
		if (ctrl->msi_enabled)
		{
			MaskIntVector(ctrl->pci_dev, 0);
		}
		else if (!CheckSetINTxMask(ctrl->pci_dev, TRUE))
		{
			KprintfH("[xhci] %s: failed to mask INTx line\n", __func__);
		}
	}

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
	struct xhci_ctrl *ctrl = unit->xhci_ctrl;
	struct Library *pcielibBase = unit->device->pcieBase;

	if (ctrl->pci_dev && ctrl->msi_enabled)
	{
		UnmaskIntVector(ctrl->pci_dev, 0);
	}
	else if (ctrl->pci_dev && !CheckSetINTxMask(ctrl->pci_dev, FALSE))
	{
		Signal(unit->task, 1UL << unit->irq_signal);
		return;
	}

	xhci_irq_enable_runtime(ctrl);
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

	LONG nvec = AllocIntVectors(ctrl->pci_dev, 1, 1, flags);
	if (nvec < 1)
	{
		Kprintf("[xhci] %s: AllocIntVectors failed (%ld)\n", __func__, (LONG)nvec);
		return -1;
	}

	/* Message-signalled (MSI or MSI-X) vs INTx steers the ISR's masking path. */
	ULONG itype = GetIntVectorType(ctrl->pci_dev);
	ctrl->msi_enabled = (itype != PCI_IRQ_INTX);
	Kprintf("[xhci] %s: using %s\n", __func__,
			itype == PCI_IRQ_MSIX ? "MSI-X" : itype == PCI_IRQ_MSI ? "MSI" : "INTx");

	if (AddIntVectorServer(ctrl->pci_dev, 0, &unit->irq_isr) != 0)
	{
		Kprintf("[xhci] %s: AddIntVectorServer failed\n", __func__);
		FreeIntVectors(ctrl->pci_dev);
		return -1;
	}

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
	{
		RemIntVectorServer(unit->xhci_ctrl->pci_dev, 0, &unit->irq_isr);
		FreeIntVectors(unit->xhci_ctrl->pci_dev);
	}
}
