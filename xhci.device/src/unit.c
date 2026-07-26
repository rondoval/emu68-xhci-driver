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

#include <exec/execbase.h>
#include <exec/types.h>
#include <errors.h>
#include <iomem.h>
#include <strutil.h>
#include <timing.h>
#include <types.h>

#include <debug.h>
#include <device.h>
#include <minlist.h>

#define __NOLIBBASE__
#include <devtree.h>

#include <libraries/pci_constants.h>
#include <libraries/openpci.h>
#include <libraries/pcitags.h>
#include <utility/tagitem.h>
#include <xhci/xhci.h>
#include <xhci/xhci-direct.h>
#include <config.h>

static s32 unit_init_onboard_xhci(struct XHCIUnit *unit,
								  struct xhci_hccr **hccr,
								  struct xhci_hcor **hcor)
{
	APTR DeviceTreeBase = OpenResource((CONST_STRPTR) "devicetree.resource");
	if (DeviceTreeBase == NULL)
	{
		Kprintf("[bcm-xhci] %s: Failed to open devicetree.resource\n", __func__);
		return -1;
	}

	APTR key = DT_OpenKey((CONST_STRPTR) "/scb/xhci");
	if (key == NULL)
	{
		Kprintf("[bcm-xhci] %s: Failed to open key %s\n", __func__, "/scb/xhci");
		return -1;
	}

	CONST_STRPTR status = DT_GetPropValue(DT_FindProperty(key, (CONST_STRPTR) "status"));
	if (status != NULL && _Stricmp(status, (CONST_STRPTR) "disabled") == 0)
	{
		Kprintf("[bcm-xhci] %s: Node %s is disabled\n", __func__, "/scb/xhci");
		DT_CloseKey(key);
		return -1;
	}

#ifdef DEBUG
	CONST_STRPTR compatible = DT_GetPropValue(DT_FindProperty(key, (CONST_STRPTR) "compatible"));
#endif

	APTR base = DT_GetBaseAddressVirtual((CONST_STRPTR) "/scb/xhci");
	if (base == NULL)
	{
		Kprintf("[bcm-xhci] %s: Failed to get base address\n", __func__);
		DT_CloseKey(key);
		return -1;
	}

	Kprintf("[bcm-xhci] %s: compatible: %s\n", __func__, compatible);

	unit->irq_line = (u32)DT_GetInterrupt(key, 0);
	Kprintf("[bcm-xhci] %s: IRQ = %lu\n", __func__, (ULONG)unit->irq_line);

	// We're done with the device tree
	DT_CloseKey(key);

	*hccr = (struct xhci_hccr *)base;
	Kprintf("[bcm-xhci] %s: init mapped hccr %lx\n", __func__, *hccr);

	*hcor = (struct xhci_hcor *)((uintptr_t)*hccr + HC_LENGTH(mmio_read32(&(*hccr)->cr_capbase)));
	Kprintf("[bcm-xhci] %s: init hccr %lx and hcor %lx hc_length %lu\n",
			__func__, *hccr, *hcor, (ULONG)HC_LENGTH(mmio_read32(&(*hccr)->cr_capbase)));

	return 0;
}

static BOOL pcie_xhci_is_supported(struct Library *pcielibBase, struct pci_dev *pd)
{
#ifndef DEBUG
	(void)pcielibBase; /* only used by the debug PCI-config reads below */
#endif
	Kprintf("[xhci] %s: Device Info:\n", __func__);
	Kprintf("[xhci] %s:   Vendor:Device = 0x%04lx:%04lx\n", __func__,
			(ULONG)pd->vendor, (ULONG)pd->device);

	/* Check if device is responding */
	if (pd->vendor == 0xFFFFU && pd->device == 0xFFFFU)
	{
		Kprintf("[xhci] %s: Device not responding to config space reads!\n", __func__);
		return FALSE;
	}

#ifdef DEBUG
	UBYTE revision = pci_read_config_byte(PCI_REVISION_ID, pd);
	UBYTE prog_if = pci_read_config_byte(PCI_CLASS_PROG, pd);
	UBYTE subclass = pci_read_config_byte((UBYTE)PCI_CLASS_DEVICE, pd);
	UBYTE baseclass = pci_read_config_byte((UBYTE)(PCI_CLASS_DEVICE + 1), pd);
	ULONG mcu_firmware = pci_read_config_long((UBYTE)0x50, pd);

	Kprintf("[xhci] %s:   Class = %02lx:%02lx:%02lx (revision %02lx)\n",
			__func__, (ULONG)baseclass, (ULONG)subclass, (ULONG)prog_if, (ULONG)revision);
	Kprintf("[xhci] %s:   MCU Firmware Version: 0x%08lx\n", __func__, mcu_firmware);
#endif

	return TRUE;
}

#define PCI_VENDOR_ID_VIA 0x1106
#define PCI_DEVICE_ID_VIA_VL805 0x3483

/* Derive controller quirks from the PCI identity (mirrors the VL805 entries in
 * Linux xhci_pci_quirks).  pci_dev == NULL is the onboard BCM2711 controller,
 * which needs no quirks (Linux only applies system-PM quirks there). */
static void xhci_detect_quirks(struct Library *pcielibBase, struct xhci_ctrl *ctrl, struct pci_dev *pd)
{
	if (!pd)
		return;

	if (pd->vendor == PCI_VENDOR_ID_VIA && pd->device == PCI_DEVICE_ID_VIA_VL805)
	{
		ctrl->quirks |= XHCI_QUIRK_TRB_OVERFETCH | XHCI_QUIRK_SS_BULK_OUT;
		ctrl->vl805_fw_version = pci_read_config_long((UBYTE)0x50, pd);
		Kprintf("[xhci] %s: VL805 quirks enabled (0x%lx), fw 0x%08lx\n",
				__func__, (ULONG)ctrl->quirks, (ULONG)ctrl->vl805_fw_version);
	}
}

/*
 * Map BAR, get register pointers and enable bus mastering
 */
static s32 pcie_xhci_init(struct Library *pcielibBase, struct pci_dev *pd,
						  struct xhci_hccr **hccr, struct xhci_hcor **hcor)
{
	*hccr = (struct xhci_hccr *)pd->base_address[0];
	if (!*hccr)
	{
		Kprintf("[xhci] %s: BAR0 not mapped\n", __func__);
		return -EIO;
	}
	KprintfT("[xhci] %s: init mapped hccr %lx\n", __func__, *hccr);

	*hcor = (struct xhci_hcor *)((uintptr_t)*hccr +
								 HC_LENGTH(mmio_read32(&(*hccr)->cr_capbase)));

	KprintfT("[xhci] %s: init hccr %lx and hcor %lx hc_length %lu\n",
			__func__, *hccr, *hcor, (ULONG)HC_LENGTH(mmio_read32(&(*hccr)->cr_capbase)));

	pci_set_master(pd);
	return 0;
}

static s32 unit_init_pcie_xhci(LONG unitNumber, struct pci_dev **ret_pci_dev,
							   struct xhci_hccr **ret_hccr,
							   struct xhci_hcor **ret_hcor,
							   struct XHCIDevice *device)
{
	/* Lazily open the PCI library on first PCIe unit access */
	if (xhci_open_pcie_library(device) != 0)
	{
		Kprintf("[xhci] %s: No PCI library available for unit %ld\n", __func__, unitNumber);
		return UHIOERR_BADPARAMS;
	}

	struct Library *pcielibBase = device->pcieBase;

	/* Find the (unitNumber)th PCIe xHCI controller (unit 0 is always onboard;
	 * pcie.library handles bus init + VL805 firmware reload on first open). */
	struct pci_dev *pd = NULL;
	for (LONG i = 0; i < unitNumber; i++)
	{
		pd = pci_find_class(0x0C0330, pd);
		if (!pd)
			break;
	}
	if (!pd)
	{
		Kprintf("[xhci] %s: Failed to find XHCI PCI device (unit %ld)\n", __func__, unitNumber);
		return UHIOERR_BADPARAMS;
	}

	if (!pcie_xhci_is_supported(pcielibBase, pd))
	{
		Kprintf("[xhci] %s: Unsupported XHCI controller\n", __func__);
		return UHIOERR_BADPARAMS;
	}

	if (!SetBoardAttrs(pd, PRM_BoardOwner, (ULONG)FindTask(NULL), TAG_DONE))
	{
		Kprintf("[xhci] %s: PCI device already owned by another task\n", __func__);
		return UHIOERR_BADPARAMS;
	}

	s32 result = pcie_xhci_init(pcielibBase, pd, ret_hccr, ret_hcor);
	if (result != 0)
	{
		Kprintf("[xhci] %s: Failed to initialize XHCI PCI device: %ld\n", __func__, result);
		SetBoardAttrs(pd, PRM_BoardOwner, 0UL, TAG_DONE);
		return result;
	}

	*ret_pci_dev = pd;
	return 0;
}

static s32 unit_attach_xhci(struct XHCIUnit *unit, struct pci_dev *pci_dev,
							struct xhci_hccr *hccr, struct xhci_hcor *hcor)
{
	struct xhci_ctrl *xhci_ctrl = AllocMem(sizeof(struct xhci_ctrl), MEMF_CLEAR | MEMF_PUBLIC);
	if (!xhci_ctrl)
	{
		Kprintf("[xhci] %s: Failed to allocate memory for xhci_ctrl\n", __func__);
		return UHIOERR_OUTOFMEMORY;
	}

	xhci_ctrl->utilityBase = unit->device->utilityBase;
	xhci_ctrl->pci_dev = pci_dev;
	xhci_detect_quirks(unit->device->pcieBase, xhci_ctrl, pci_dev);

	s32 result = xhci_register(xhci_ctrl, hccr, hcor);
	if (result)
	{
		Kprintf("[xhci] %s: xhci_register failed: %ld\n", __func__, result);
		goto err_free_ctrl;
	}

	unit->xhci_ctrl = xhci_ctrl;

	result = UnitTaskStart(unit);
	if (result != UHIOERR_NO_ERROR)
	{
		Kprintf("[xhci] %s: Failed to start unit task: %ld\n", __func__, result);
		goto err_deregister;
	}

	result = xhci_int_enable(unit);
	if (result < 0)
	{
		Kprintf("[xhci] %s: Failed to enable interrupts (%ld)\n", __func__, (LONG)result);
		goto err_shutdown_irq;
	}

	return UHIOERR_NO_ERROR;

err_shutdown_irq:
	xhci_int_shutdown(unit);
err_deregister:
	xhci_deregister(xhci_ctrl);
	unit->xhci_ctrl = NULL;
err_free_ctrl:
	FreeMem(xhci_ctrl, sizeof(*xhci_ctrl));
	return result;
}

s32 UnitOpen(struct XHCIUnit *unit, LONG unitNumber)
{
	/* openLib enforces exclusive access; a unit arrives here only unopened. */
	KprintfT("[xhci] %s: Opening unit %ld\n", __func__, unitNumber);
	unit->unit.unit_OpenCnt = 1;
	unit->unitNumber = unitNumber;

	unit->memoryPool = CreatePool(MEMF_FAST | MEMF_PUBLIC, 16384, 8192);
	if (unit->memoryPool == NULL)
	{
		Kprintf("[xhci] %s: Failed to create memory pool\n", __func__);
		return UHIOERR_OUTOFMEMORY;
	}

	struct xhci_hccr *hccr;
	struct xhci_hcor *hcor;
	struct pci_dev *pci_dev = NULL;
	s32 result = (unitNumber == 0)
		? unit_init_onboard_xhci(unit, &hccr, &hcor)
		: unit_init_pcie_xhci(unitNumber, &pci_dev, &hccr, &hcor, unit->device);
	if (result != UHIOERR_NO_ERROR)
		goto err_del_pool;

	result = unit_attach_xhci(unit, pci_dev, hccr, hcor);
	if (result != UHIOERR_NO_ERROR)
		goto err_del_pool;

	return UHIOERR_NO_ERROR;

err_del_pool:
	DeletePool(unit->memoryPool);
	unit->memoryPool = NULL;
	return result;
}

s32 UnitClose(struct XHCIUnit *unit)
{
	Kprintf("[xhci] %s: Closing unit %ld\n", __func__, unit->unitNumber);

	unit->unit.unit_OpenCnt--;
	if (unit->unit.unit_OpenCnt == 0)
	{
		Kprintf("[xhci] %s: Last opener closed, cleaning up unit\n", __func__);
		struct Library *pcielibBase = unit->device->pcieBase;
		if (pcielibBase && unit->xhci_ctrl->pci_dev)
			SetBoardAttrs(unit->xhci_ctrl->pci_dev, PRM_BoardOwner, 0UL, TAG_DONE);
		UnitTaskStop(unit);
		xhci_direct_detach(unit); /* no direct call may land in a dying controller */
		xhci_int_shutdown(unit);
		xhci_deregister(unit->xhci_ctrl);
		FreeMem(unit->xhci_ctrl, sizeof(*unit->xhci_ctrl));
		DeletePool(unit->memoryPool);
		unit->memoryPool = NULL;
	}

	return unit->unit.unit_OpenCnt;
}
