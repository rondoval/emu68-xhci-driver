// SPDX-License-Identifier: GPL-2.0+
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#else
#define __NOLIBBASE__
#define EXEC_BASE_NAME (*(struct ExecBase **)4UL)
#include <proto/exec.h>
#endif

#include <exec/execbase.h>
#include <exec/types.h>
#include <emu_errors.h>
#include <emu_iomem.h>
#include <emu_string.h>
#include <emu_timing.h>
#include <emu_types.h>

#include <debug.h>
#include <device.h>
#include <minlist.h>

#define __NOLIBBASE__
#include <devtree.h>

#include <devices/hcd_api.h>
#include <pci_types.h>
#include <pci.h>
#include <xhci/xhci.h>
#include <config.h>

static int unit_init_onboard_xhci(struct XHCIUnit *unit,
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
	if (status != NULL && _Stricmp(status, (CONST_STRPTR)"disabled") == 0)
	{
		Kprintf("[bcm-xhci] %s: Node %s is disabled\n", __func__, "/scb/xhci");
		DT_CloseKey(key);
		return -1;
	}

	CONST_STRPTR compatible = DT_GetPropValue(DT_FindProperty(key, (CONST_STRPTR) "compatible"));

	APTR base = DT_GetBaseAddressVirtual((CONST_STRPTR) "/scb/xhci");
	if (base == NULL)
	{
		Kprintf("[bcm-xhci] %s: Failed to get base address\n", __func__);
		DT_CloseKey(key);
		return -1;
	}

	Kprintf("[bcm-xhci] %s: compatible: %s\n", __func__, compatible);

	unit->irq_line = DT_GetInterrupt(key, 0);
	Kprintf("[bcm-xhci] %s: IRQ = %ld\n", __func__, unit->irq_line);
	unit->irq_line += 32;

	// We're done with the device tree
	DT_CloseKey(key);

	*hccr = (struct xhci_hccr *)base;
	Kprintf("[bcm-xhci] %s: init mapped hccr %lx\n", __func__, *hccr);

	*hcor = (struct xhci_hcor *)((uintptr_t)*hccr + HC_LENGTH(readl(&(*hccr)->cr_capbase)));
	Kprintf("[bcm-xhci] %s: init hccr %lx and hcor %lx hc_length %ld\n",
			__func__, *hccr, *hcor, (u32)HC_LENGTH(readl(&(*hccr)->cr_capbase)));

	return 0;
}

/*
 * Initialize and enumerate the PCIe bus
 */
static int pcie_init(struct XHCIDevice *device)
{
	if (device->pcie != NULL)
		return 0;

	device->pcie = AllocMem(sizeof(struct pci_controller), MEMF_CLEAR | MEMF_PUBLIC);
	if (!device->pcie)
	{
		Kprintf("[pcie] %s: Failed to allocate memory for PCIe controller\n", __func__);
		return -ENOMEM;
	}
	_NewMinList(&device->pcie->buses);

	int ret = brcm_pcie_probe(device->pcie, /* bus number */ 0);
	if (ret < 0)
	{
		Kprintf("[pcie] %s: brcm_pcie_probe failed: %ld\n", __func__, ret);
		FreeMem(device->pcie, sizeof(*device->pcie));
		device->pcie = NULL;
		return -ENODEV;
	}
	Kprintf("[pcie] %s: brcm_pcie_probe succeeded\n", __func__);

	struct pci_bus *root_bus = AllocMem(sizeof(*root_bus), MEMF_CLEAR);
	if (!root_bus)
	{
		Kprintf("[pcie] %s: Failed to allocate memory for root bus\n", __func__);
		return -ENOMEM;
	}

	_NewMinList(&root_bus->devices);
	root_bus->controller = device->pcie;
	root_bus->parent = NULL;
	root_bus->pci_bridge = NULL;
	CopyMem((APTR)"pcie0", root_bus->name, sizeof("pcie0"));
	root_bus->bus_number = 0;
	root_bus->bus_number_last_sub = 0;
	AddTailMinList(&device->pcie->buses, (struct MinNode *)root_bus);

	ret = pci_bind_bus_devices(root_bus);
	if (ret)
	{
		Kprintf("[pcie] %s: pci_bind_bus_devices failed: %ld\n", __func__, ret);
		FreeMem(root_bus, sizeof(*root_bus));
		return -ENODEV;
	}

	ret = pci_auto_config_devices(root_bus);
	if (ret < 0)
	{
		Kprintf("[pcie] %s: pci_auto_config_devices failed: %ld\n", __func__, ret);
		FreeMem(root_bus, sizeof(*root_bus));
		return -ENODEV;
	}

	return 0;
}

static int vl805_init(void)
{
	int ret = bcm2711_reload_vl805_firmware();
	if (ret != 0)
	{
		Kprintf("[vl805] %s: Failed to load VL805 firmware: %ld\n", __func__, ret);
		return -ENODEV;
	}
	/* It seems to take a while for the VL805 to start responding */
	delay_us(1000);
	return 0;
}

static BOOL pcie_xhci_is_supported(struct pci_device *dev)
{
	// Check some basic PCI device info
	ULONG vendor_device;
	UBYTE revision, prog_if, subclass, baseclass;
	ULONG mcu_firmware;

	dm_pci_read_config32(dev, PCI_VENDOR_ID, &vendor_device);
	dm_pci_read_config8(dev, PCI_REVISION_ID, &revision);
	dm_pci_read_config8(dev, PCI_CLASS_PROG, &prog_if);
	dm_pci_read_config8(dev, PCI_CLASS_DEVICE, &subclass);
	dm_pci_read_config8(dev, PCI_CLASS_DEVICE + 1, &baseclass);
	dm_pci_read_config32(dev, 0x50, &mcu_firmware);

	Kprintf("[pcie] %s: Device Info:\n", __func__);
	Kprintf("[pcie] %s:   Vendor:Device = 0x%08lx\n", __func__, vendor_device);
	Kprintf("[pcie] %s:   Class = %02lx:%02lx:%02lx (revision %02lx)\n", __func__, baseclass, subclass, prog_if, revision);
	Kprintf("[pcie] %s:   MCU Firmware Version: 0x%08lx\n", __func__, mcu_firmware);

	// Check if device is responding to config space
	if (vendor_device == 0xFFFFFFFF)
	{
		Kprintf("[pcie] %s: Device not responding to config space reads!\n", __func__);
		return FALSE;
	}

	return TRUE;
}

/*
 * Map BAR, get register pointers and enable bus mastering
 */
static int pcie_xhci_init(struct pci_device *dev, struct xhci_hccr **hccr,
						  struct xhci_hcor **hcor)
{
	*hccr = (struct xhci_hccr *)dm_pci_map_bar(dev,
											   PCI_BASE_ADDRESS_0, 0, 0, PCI_REGION_TYPE,
											   PCI_REGION_MEM);
	if (!*hccr)
	{
		Kprintf("[xhci] %s: init cannot map PCI mem bar\n", __func__);
		return -EIO;
	}
	Kprintf("[xhci] %s: init mapped hccr %lx\n", __func__, *hccr);

	*hcor = (struct xhci_hcor *)((uintptr_t)*hccr +
								 HC_LENGTH(readl(&(*hccr)->cr_capbase)));

	Kprintf("[xhci] %s: init hccr %lx and hcor %lx hc_length %ld\n",
			__func__, *hccr, *hcor, (u32)HC_LENGTH(readl(&(*hccr)->cr_capbase)));

	/* enable busmaster */
	u32 cmd;
	dm_pci_read_config32(dev, PCI_COMMAND, &cmd);
	cmd |= PCI_COMMAND_MASTER;
	dm_pci_write_config32(dev, PCI_COMMAND, cmd);
	return 0;
}

static int unit_init_pcie_xhci(LONG unitNumber, struct pci_device **ret_xhci_dev,
							   struct xhci_hccr **ret_hccr,
							   struct xhci_hcor **ret_hcor,
							   struct XHCIDevice *device)
{
	struct pci_device *xhci_dev = NULL;
	int result = pcie_init(device);
	if (result != 0)
	{
		Kprintf("[xhci] %s: Failed to initialize PCIe: %ld\n", __func__, result);
		return result;
	}

	/* -1 because unit 0 is always the OTG port and dm_pci_find_class indexes from 0 */
	dm_pci_find_class(device->pcie, 0x0C0330, unitNumber - 1, &xhci_dev);
	if (xhci_dev == NULL)
	{
		Kprintf("[xhci] %s: Failed to find XHCI PCI device\n", __func__);
		return ERR_BAD_PARAMETERS;
	}

	if (xhci_dev->vendor == 0x1106 && xhci_dev->device == 0x3483)
	{
		Kprintf("[xhci] %s: Found VL805 XHCI controller, loading firmware\n", __func__);
		result = vl805_init();
		if (result != 0)
		{
			Kprintf("[xhci] %s: Failed to load VL805 firmware: %ld\n", __func__, result);
			/* continue, this may be other XHCI controller */
		}
	}

	if (!pcie_xhci_is_supported(xhci_dev))
	{
		Kprintf("[xhci] %s: Unsupported XHCI controller\n", __func__);
		return ERR_BAD_PARAMETERS;
	}

	result = pcie_xhci_init(xhci_dev, ret_hccr, ret_hcor);
	if (result != 0)
	{
		Kprintf("[xhci] %s: Failed to initialize XHCI PCI device: %ld\n", __func__, result);
		return result;
	}

	*ret_xhci_dev = xhci_dev;
	return 0;
}

static int unit_init_xhci_hw(struct XHCIUnit *unit, LONG unitNumber,
							 struct pci_device **ret_xhci_dev,
							 struct xhci_hccr **ret_hccr,
							 struct xhci_hcor **ret_hcor)
{
	*ret_xhci_dev = NULL;

	if (unitNumber == 0)
		return unit_init_onboard_xhci(unit, ret_hccr, ret_hcor);

	return unit_init_pcie_xhci(unitNumber, ret_xhci_dev, ret_hccr, ret_hcor, unit->device);
}

static int unit_attach_xhci(struct XHCIUnit *unit, struct pci_device *xhci_dev,
							struct xhci_hccr *hccr, struct xhci_hcor *hcor)
{
	struct xhci_ctrl *xhci_ctrl = AllocMem(sizeof(struct xhci_ctrl), MEMF_CLEAR | MEMF_PUBLIC);
	if (!xhci_ctrl)
	{
		Kprintf("[xhci] %s: Failed to allocate memory for xhci_ctrl\n", __func__);
		return ERR_ALLOC_ERROR;
	}

	xhci_ctrl->utilityBase = unit->device->utilityBase;
	xhci_ctrl->pci_dev = xhci_dev;

	int result = xhci_register(xhci_ctrl, hccr, hcor);
	if (result)
	{
		Kprintf("[xhci] %s: xhci_register failed: %ld\n", __func__, result);
		goto err_free_ctrl;
	}

	unit->xhci_ctrl = xhci_ctrl;

	result = UnitTaskStart(unit);
	if (result != ERR_NO_ERROR)
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

	return ERR_NO_ERROR;

err_shutdown_irq:
	xhci_int_shutdown(unit);
err_deregister:
	xhci_deregister(xhci_ctrl);
	unit->xhci_ctrl = NULL;
err_free_ctrl:
	FreeMem(xhci_ctrl, sizeof(*xhci_ctrl));
	return result;
}

int UnitOpen(struct XHCIUnit *unit, LONG unitNumber, LONG flags)
{
	Kprintf("[xhci] %s: Opening unit %ld with flags %lx\n", __func__, unitNumber, flags);
	if (unit->unit.unit_OpenCnt > 0)
	{
		unit->unit.unit_OpenCnt++;
		Kprintf("[xhci] %s: Unit opened successfully, current open count: %ld\n", __func__, unit->unit.unit_OpenCnt);
		return ERR_NO_ERROR;
	}

	unit->flags = flags;
	unit->unit.unit_OpenCnt = 1;
	unit->unitNumber = unitNumber;

	unit->memoryPool = CreatePool(MEMF_FAST | MEMF_PUBLIC, 16384, 8192);
	if (unit->memoryPool == NULL)
	{
		Kprintf("[xhci] %s: Failed to create memory pool\n", __func__);
		return ERR_ALLOC_ERROR;
	}

	struct xhci_hccr *hccr;
	struct xhci_hcor *hcor;
	struct pci_device *xhci_dev = NULL;
	int result = unit_init_xhci_hw(unit, unitNumber, &xhci_dev, &hccr, &hcor);
	if (result != ERR_NO_ERROR)
		goto err_del_pool;

	result = unit_attach_xhci(unit, xhci_dev, hccr, hcor);
	if (result != ERR_NO_ERROR)
		goto err_del_pool;

	return ERR_NO_ERROR;

err_del_pool:
	DeletePool(unit->memoryPool);
	unit->memoryPool = NULL;
	return result;
}

int UnitClose(struct XHCIUnit *unit)
{
	Kprintf("[xhci] %s: Closing unit %ld\n", __func__, unit->unitNumber);

	unit->unit.unit_OpenCnt--;
	if (unit->unit.unit_OpenCnt == 0)
	{
		Kprintf("[xhci] %s: Last opener closed, cleaning up unit\n", __func__);
		UnitTaskStop(unit);
		xhci_int_shutdown(unit);
		xhci_deregister(unit->xhci_ctrl);
		FreeMem(unit->xhci_ctrl, sizeof(*unit->xhci_ctrl));
		DeletePool(unit->memoryPool);
		unit->memoryPool = NULL;
	}

	return unit->unit.unit_OpenCnt;
}
