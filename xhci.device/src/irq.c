// SPDX-License-Identifier: GPL-2.0+
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#include <clib/gic400_protos.h>
#else
#include <proto/exec.h>
#include <proto/gic400.h>
#endif

#include <compat.h>
#include <debug.h>
#include <pci.h>
#include <xhci/xhci.h>
#include <xhci/xhci-events.h>
#include <device.h>

#define XHCI_IRQ_ACK_MASK (STS_EINT | STS_FATAL | STS_PORT)

static inline void xhci_irq_disable_runtime(struct xhci_ctrl *ctrl)
{
	KprintfH("[xhci] %s: disabling runtime IRQs\n", __func__);
	u32 iman = xhci_readl(&ctrl->ir_set->irq_pending);
	xhci_writel(&ctrl->ir_set->irq_pending,
		       ER_IRQ_DISABLE(iman) | ER_IRQ_PENDING(iman));
}

static inline void xhci_irq_enable_runtime(struct xhci_ctrl *ctrl)
{
	KprintfH("[xhci] %s: enabling runtime IRQs\n", __func__);
	u32 iman = xhci_readl(&ctrl->ir_set->irq_pending);
	xhci_writel(&ctrl->ir_set->irq_pending,
		       ER_IRQ_ENABLE(iman) | ER_IRQ_PENDING(iman));
}

static inline void xhci_irq_update_cmd(struct xhci_ctrl *ctrl, BOOL enable)
{
	KprintfH("[xhci] %s: %s CMD_EIE | CMD_HSEIE\n", __func__, enable ? "enabling" : "disabling");
	u32 cmd = xhci_readl(&ctrl->hcor->or_usbcmd);
	if (enable)
		cmd |= (CMD_EIE | CMD_HSEIE);
	else
		cmd &= ~(CMD_EIE | CMD_HSEIE);
	xhci_writel(&ctrl->hcor->or_usbcmd, cmd);
}

static ULONG xhci_intx_isr(struct ExecBase *SysBase asm("a6"), struct XHCIUnit *unit asm("a1"), ULONG vector asm("d0"))
{
	(void)SysBase;
	(void)vector;

	struct xhci_ctrl *ctrl = unit->xhci_ctrl;
	ULONG status = xhci_readl(&ctrl->hcor->or_usbsts) & XHCI_IRQ_ACK_MASK;

	if (!status)
		return 0;

	if (status & STS_FATAL)
	{
		Kprintf("[xhci] %s: fatal status interrupt (USBSTS=0x%08lx)\n", __func__, status);
	}

	xhci_writel(&ctrl->hcor->or_usbsts, status & XHCI_IRQ_ACK_MASK);
	xhci_irq_disable_runtime(ctrl);

	if(unit->xhci_ctrl->pci_dev->msi.enabled)
	{
		pci_msi_mask_irq(unit->xhci_ctrl->pci_dev, unit->xhci_ctrl->pci_dev->msi.irq);
	}
	else if (!pci_check_and_set_intx_mask(ctrl->pci_dev, TRUE))
	{
		KprintfH("[xhci] %s: failed to mask INTx line\n", __func__);
	}

	Signal(unit->task, 1UL << unit->irq_signal);

	return 1;
}

int xhci_intx_enable(struct XHCIUnit *unit)
{
	Kprintf("[xhci] %s: enabling INTx\n", __func__);
	// UBYTE irq_line_cfg;
	// dm_pci_read_config8(unit->xhci_ctrl->pci_dev, PCI_INTERRUPT_LINE, &irq_line_cfg);
	// if (irq_line_cfg == 0 || irq_line_cfg == 0xff)
	// {
	// 	Kprintf("[xhci] %s: controller reports no legacy INTx line (value=0x%02lx)\n", __func__, (ULONG)irq_line_cfg);
	// 	return -ENODEV;
	// }

	// unit->irq_line = irq_line_cfg + 32;
	unit->irq_line = unit->xhci_ctrl->pci_dev->irq + 32;

	unit->irq_isr.is_Node.ln_Type = NT_INTERRUPT;
	unit->irq_isr.is_Node.ln_Name = "xhci_intx_isr";
	unit->irq_isr.is_Data = unit;
	unit->irq_isr.is_Code = (APTR)xhci_intx_isr;

	int ret = AddIntServerEx((ULONG)unit->irq_line, 0, FALSE, &unit->irq_isr);
	if (ret < 0)
	{
		Kprintf("[xhci] %s: AddIntServerEx failed for IRQ %ld (ret=%ld)\n", __func__, unit->irq_line, (LONG)ret);
		return ret;
	}

	struct xhci_ctrl *ctrl = unit->xhci_ctrl;

	pci_intx(ctrl->pci_dev, TRUE);
	pci_check_and_set_intx_mask(ctrl->pci_dev, FALSE);

	xhci_irq_update_cmd(ctrl, TRUE);
	xhci_irq_enable_runtime(ctrl);

	Kprintf("[xhci] %s: INTx enabled on IRQ %ld\n", __func__, unit->irq_line);

	return 0;
}

int xhci_msi_enable(struct XHCIUnit *unit)
{
	Kprintf("[xhci] %s: enabling MSI\n", __func__);
	if(unit->xhci_ctrl->pci_dev->msi.enabled)
	{
		Kprintf("[xhci] %s: MSI already enabled\n", __func__);
		return 0;
	}

	if(pci_get_controller(unit->xhci_ctrl->pci_dev->bus)->msi.enabled == FALSE)
	{
		Kprintf("[xhci] %s: MSI not supported on this controller, falling back to INTx\n", __func__);
		return xhci_intx_enable(unit);
	}

	unit->irq_line = unit->xhci_ctrl->pci_dev->msi.irq + 32;

	unit->irq_isr.is_Node.ln_Type = NT_INTERRUPT;
	unit->irq_isr.is_Node.ln_Name = "xhci_msi_isr";
	unit->irq_isr.is_Data = unit;
	unit->irq_isr.is_Code = (APTR)xhci_intx_isr;

	int ret = add_int_server(unit->xhci_ctrl->pci_dev, &unit->irq_isr);
	if (ret < 0)
	{
		Kprintf("[xhci] %s: add_int_server failed (ret=%ld)\n", __func__, (LONG)ret);
		return ret;
	}

	struct xhci_ctrl *ctrl = unit->xhci_ctrl;

	xhci_irq_update_cmd(ctrl, TRUE);
	xhci_irq_enable_runtime(ctrl);

	return 0;
}

void xhci_msi_shutdown(struct XHCIUnit *unit)
{
	if (!unit)
		return;

	if(!unit->xhci_ctrl->pci_dev->msi.enabled)
	{
		Kprintf("[xhci] %s: MSI not enabled, shutting down INTx\n", __func__);
		xhci_intx_shutdown(unit);
		return;
	}

	struct xhci_ctrl *ctrl = unit->xhci_ctrl;
	xhci_irq_disable_runtime(ctrl);
	xhci_irq_update_cmd(ctrl, FALSE);

	rem_int_server(unit->xhci_ctrl->pci_dev);
}

void xhci_intx_shutdown(struct XHCIUnit *unit)
{
	if (!unit)
		return;

	struct xhci_ctrl *ctrl = unit->xhci_ctrl;
	xhci_irq_disable_runtime(ctrl);
	xhci_irq_update_cmd(ctrl, FALSE);

	if (!pci_check_and_set_intx_mask(ctrl->pci_dev, FALSE))
	{
		Kprintf("[xhci] %s: failed to unmask INTx line during shutdown\n", __func__);
	}

	RemIntServerEx((ULONG)unit->irq_line, &unit->irq_isr);
}

void xhci_intx_handle(struct XHCIUnit *unit)
{
	struct xhci_ctrl *ctrl = unit->xhci_ctrl;

	xhci_process_event_trb(ctrl);

	if(unit->xhci_ctrl->pci_dev->msi.enabled)
	{
		pci_msi_unmask_irq(unit->xhci_ctrl->pci_dev, unit->xhci_ctrl->pci_dev->msi.irq);
		xhci_irq_enable_runtime(ctrl);
		return;
	}

	if (!pci_check_and_set_intx_mask(ctrl->pci_dev, FALSE))
	{
		Signal(unit->task, 1UL << unit->irq_signal);
	}
	else
	{
		xhci_irq_enable_runtime(ctrl);
	}
}
