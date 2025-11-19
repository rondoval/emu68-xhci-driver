// SPDX-License-Identifier: GPL-2.0+
#ifdef __INTELLISENSE__
#include <clib/exec_protos.h>
#include <clib/dos_protos.h>
#include <clib/utility_protos.h>
#else
#include <proto/exec.h>
#include <proto/dos.h>
#include <proto/utility.h>
#endif

#include <exec/execbase.h>
#include <exec/types.h>
#include <stdarg.h>

#include <debug.h>
#include <device.h>
#include <compat.h>
#include <minlist.h>

#include <mbox.h>
#include <msg.h>
#include <devices/usbhardware.h>
#include <pci_types.h>
#include <pci.h>
#include <xhci/usb.h>
#include <xhci/xhci.h>
#include <usb_glue.h>


static struct pci_controller *pcie = NULL;
extern struct MinList pci_bus_list;

/*
 * Initialize and enumerate the PCIe bus
 */
static int pcie_init(void)
{
	if (pcie != NULL)
		return 0;

	pcie = AllocMem(sizeof(struct pci_controller), MEMF_CLEAR | MEMF_PUBLIC);
	if (!pcie)
	{
		Kprintf("[pcie] %s: Failed to allocate memory for PCIe controller\n", __func__);
		return -ENOMEM;
	}

	int ret = brcm_pcie_probe(pcie, /* bus number */ 0);
	if (ret < 0)
	{
		Kprintf("[pcie] %s: brcm_pcie_probe failed: %ld\n", __func__, ret);
		return -ENODEV;
	}
	Kprintf("[pcie] %s: brcm_pcie_probe succeeded\n", __func__);

	_NewMinList(&pci_bus_list);

	struct pci_bus *root_bus = AllocMem(sizeof(*root_bus), MEMF_CLEAR);
	if (!root_bus)
	{
		Kprintf("[pcie] %s: Failed to allocate memory for root bus\n", __func__);
		return -ENOMEM;
	}

	_NewMinList(&root_bus->devices);
	root_bus->controller = pcie;
	root_bus->parent = NULL;
	root_bus->pci_bridge = NULL;
	SNPrintf(root_bus->name, sizeof(root_bus->name), (CONST_STRPTR) "pcie0");
	root_bus->bus_number = 0;
	root_bus->bus_number_last_sub = 0;
	AddTailMinList(&pci_bus_list, (struct MinNode *)root_bus);

	ret = pci_bind_bus_devices(root_bus);
	if (ret)
	{
		Kprintf("[pcie] %s: pci_bind_bus_devices failed: %ld\n", __func__, ret);
		return -ENODEV;
	}

	ret = pci_auto_config_devices(root_bus);
	if (ret < 0)
	{
		Kprintf("[pcie] %s: pci_auto_config_devices failed: %ld\n", __func__, ret);
		return -ENODEV;
	}

	return 0;
}

static int vl805_init(void)
{
	int ret = mbox_parse_devtree();
	if (ret != 0)
	{
		Kprintf("[vl805] %s: mbox_parse_devtree failed: %ld\n", __func__, ret);
		return -ENODEV;
	}

	ret = bcm2711_notify_vl805_reset();
	if (ret != 0)
	{
		Kprintf("[vl805] %s: Failed to load VL805 firmware: %ld\n", __func__, ret);
		return -ENODEV;
	}
	/* It seems to take a while for the VL805 to start responding */
	delay_us(1000);
	return 0;
}

static BOOL is_supported(struct pci_device *dev)
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
static int xhci_pci_init(struct pci_device *dev, struct xhci_hccr **ret_hccr,
			 struct xhci_hcor **ret_hcor)
{
	struct xhci_hccr *hccr;
	struct xhci_hcor *hcor;
	u32 cmd;

	hccr = (struct xhci_hccr *)dm_pci_map_bar(dev,
			PCI_BASE_ADDRESS_0, 0, 0, PCI_REGION_TYPE,
			PCI_REGION_MEM);
	if (!hccr) {
		Kprintf("[xhci] %s: init cannot map PCI mem bar\n", __func__);
		return -EIO;
	}
	Kprintf("[xhci] %s: init mapped hccr %lx\n", __func__, hccr);

	hcor = (struct xhci_hcor *)((uintptr_t) hccr +
			HC_LENGTH(xhci_readl(&hccr->cr_capbase)));

	Kprintf("[xhci] %s: init hccr %lx and hcor %lx hc_length %ld\n",
	      __func__, hccr, hcor, (u32)HC_LENGTH(xhci_readl(&hccr->cr_capbase)));

	*ret_hccr = hccr;
	*ret_hcor = hcor;

	/* enable busmaster */
	dm_pci_read_config32(dev, PCI_COMMAND, &cmd);
	cmd |= PCI_COMMAND_MASTER;
	dm_pci_write_config32(dev, PCI_COMMAND, cmd);
	return 0;
}

int UnitOpen(struct XHCIUnit *unit, LONG unitNumber, LONG flags)
{
	Kprintf("[xhci] %s: Opening unit %ld with flags %lx\n", __func__, unitNumber, flags);
	if (unit->unit.unit_OpenCnt > 0)
	{
		Kprintf("[xhci] %s: Unit already running; using message to add opener\n", __func__);
		unit->unit.unit_OpenCnt++;
		Kprintf("[xhci] %s: Unit opened successfully, current open count: %ld\n", __func__, unit->unit.unit_OpenCnt);
		return UHIOERR_NO_ERROR;
	}

	unit->flags = flags;
	unit->unit.unit_OpenCnt = 1;
	unit->unitNumber = unitNumber;

	unit->memoryPool = CreatePool(MEMF_FAST | MEMF_PUBLIC, 16384, 8192);
	if (unit->memoryPool == NULL)
	{
		Kprintf("[xhci] %s: Failed to create memory pool\n", __func__);
		return UHIOERR_OUTOFMEMORY;
	}

	int result = pcie_init();
	if (result != 0)
	{
		Kprintf("[xhci] %s: Failed to initialize PCIe: %ld\n", __func__, result);
		goto err_del_pool;
	}

	struct pci_device *xhci_dev = NULL;
	dm_pci_find_class(0x0C0330, unitNumber, &xhci_dev);
	if (xhci_dev == NULL)
	{
		Kprintf("[xhci] %s: Failed to find XHCI PCI device\n", __func__);
		result = UHIOERR_BADPARAMS;
		goto err_del_pool;
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

	if(!is_supported(xhci_dev))
	{
		Kprintf("[xhci] %s: Unsupported XHCI controller\n", __func__);
		result = UHIOERR_BADPARAMS;
		goto err_del_pool;
	}

	struct xhci_hccr *hccr;
	struct xhci_hcor *hcor;

	result = xhci_pci_init(xhci_dev, &hccr, &hcor);
	if (result) {
		Kprintf("[xhci] %s: Failed to initialize XHCI PCI device: %ld\n", __func__, result);
		goto err_del_pool;
	}

	struct xhci_ctrl *xhci_ctrl = AllocMem(sizeof(struct xhci_ctrl), MEMF_CLEAR | MEMF_PUBLIC);
	if (!xhci_ctrl) {
		Kprintf("[xhci] %s: Failed to allocate memory for xhci_ctrl\n", __func__);
		result = UHIOERR_OUTOFMEMORY;
		goto err_del_pool;
	}

	xhci_ctrl->pci_dev = xhci_dev;
	result = xhci_register(xhci_ctrl, hccr, hcor);
	if (result) {
		Kprintf("[xhci] %s: xhci_register failed: %ld\n", __func__, result);
		goto err_del_ctrl;
	}

	unit->xhci_ctrl = xhci_ctrl;

	result = UnitTaskStart(unit);
	if (result != UHIOERR_NO_ERROR)
	{
		Kprintf("[xhci] %s: Failed to start unit task: %ld\n", __func__, result);
		goto err_dereg;
	}

	result = xhci_intx_enable(unit);
	if (result < 0)
	{
		Kprintf("[xhci] %s: Failed to enable INTx (%ld)\n", __func__, (LONG)result);
		goto err_int_shutdown;
	}
	return UHIOERR_NO_ERROR;

err_int_shutdown:
	xhci_intx_shutdown(unit);
err_dereg:	
	xhci_deregister(xhci_ctrl);
err_del_ctrl:
	FreeMem(xhci_ctrl, sizeof(*xhci_ctrl));

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
		xhci_intx_shutdown(unit);
		xhci_deregister(unit->xhci_ctrl);
		FreeMem(unit->xhci_ctrl, sizeof(*unit->xhci_ctrl));
		DeletePool(unit->memoryPool);
		unit->memoryPool = NULL;
	}

	return unit->unit.unit_OpenCnt;
}
