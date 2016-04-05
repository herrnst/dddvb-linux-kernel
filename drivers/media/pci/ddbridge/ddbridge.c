/*
 * ddbridge.c: Digital Devices PCIe bridge driver
 *
 * Copyright (C) 2010-2015 Digital Devices GmbH
 *                         Ralph Metzler <rjkm@metzlerbros.de>
 *                         Marcus Metzler <mocm@metzlerbros.de>
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 only, as published by the Free Software Foundation.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA
 * Or, point your browser to http://www.gnu.org/copyleft/gpl.html
 */

#undef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define DDB_USE_WORK

#include "ddbridge.h"
#include "ddbridge-regs.h"

static struct workqueue_struct *ddb_wq;

static int adapter_alloc;
module_param(adapter_alloc, int, 0444);
MODULE_PARM_DESC(adapter_alloc,
		 "0-one adapter per io, 1-one per tab with io, 2-one per tab, 3-one for all");

#ifdef CONFIG_PCI_MSI
#define DDB_USE_MSI_IRQHANDLERS
#ifdef CONFIG_DVB_DDBRIDGE_MSIENABLE
static int msi = 1;
#else
static int msi = 0;
#endif
module_param(msi, int, 0444);
MODULE_PARM_DESC(msi,
#ifdef CONFIG_DVB_DDBRIDGE_MSIENABLE
		 " Control MSI interrupts: 0-disable, 1-enable (default)");
#else
		 " Control MSI interrupts: 0-disable (default), 1-enable");
#endif
#endif

#include "ddbridge-core.c"

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static void ddb_unmap(struct ddb *dev)
{
	if (dev->regs)
		iounmap(dev->regs);
	vfree(dev);
}

static void ddb_irq_disable(struct ddb *dev)
{
	ddbwritel(dev, 0, INTERRUPT_ENABLE);
	ddbwritel(dev, 0, MSI1_ENABLE);
}

static void ddb_irq_exit(struct ddb *dev)
{
	ddb_irq_disable(dev);
	if (dev->msi == 2)
		free_irq(dev->pdev->irq + 1, dev);
	free_irq(dev->pdev->irq, dev);
#ifdef CONFIG_PCI_MSI
	if (dev->msi)
		pci_disable_msi(dev->pdev);
#endif
}

static void ddb_remove(struct pci_dev *pdev)
{
	struct ddb *dev = (struct ddb *) pci_get_drvdata(pdev);

	ddb_device_destroy(dev);
	ddb_ports_detach(dev);
	ddb_i2c_release(dev);

	ddb_irq_exit(dev);
	ddb_ports_release(dev);
	ddb_buffers_free(dev);

	ddb_unmap(dev);
	pci_set_drvdata(pdev, NULL);
	pci_disable_device(pdev);
}

#ifdef CONFIG_PCI_MSI
static void ddb_irq_msi(struct ddb *dev, int nr)
{
	int stat;

	if (msi && pci_msi_enabled()) {
		stat = pci_alloc_irq_vectors(dev->pdev, 1, nr, PCI_IRQ_MSI);
		if (stat >= 1) {
			dev->msi = stat;
			pr_info("using %d MSI interrupt(s)\n",
				dev->msi);
		} else
			pr_info("MSI not available.\n");
	}
}
#endif

static int ddb_irq_init(struct ddb *dev)
{
	int stat;
	int irq_flag = IRQF_SHARED;

	ddbwritel(dev, 0x00000000, INTERRUPT_ENABLE);
	ddbwritel(dev, 0x00000000, MSI1_ENABLE);
	ddbwritel(dev, 0x00000000, MSI2_ENABLE);
	ddbwritel(dev, 0x00000000, MSI3_ENABLE);
	ddbwritel(dev, 0x00000000, MSI4_ENABLE);
	ddbwritel(dev, 0x00000000, MSI5_ENABLE);
	ddbwritel(dev, 0x00000000, MSI6_ENABLE);
	ddbwritel(dev, 0x00000000, MSI7_ENABLE);

#ifdef CONFIG_PCI_MSI
	ddb_irq_msi(dev, 2);

	if (dev->msi)
		irq_flag = 0;
	if (dev->msi == 2) {
		stat = request_irq(dev->pdev->irq, irq_handler0,
				   irq_flag, "ddbridge", (void *) dev);
		if (stat < 0)
			return stat;
		stat = request_irq(dev->pdev->irq + 1, irq_handler1,
				   irq_flag, "ddbridge", (void *) dev);
		if (stat < 0) {
			free_irq(dev->pdev->irq, dev);
			return stat;
		}
	} else
#endif
	{
		stat = request_irq(dev->pdev->irq, irq_handler,
				   irq_flag, "ddbridge", (void *) dev);
		if (stat < 0)
			return stat;
	}
	if (dev->msi == 2) {
		ddbwritel(dev, 0x0fffff00, INTERRUPT_ENABLE);
		ddbwritel(dev, 0x0000000f, MSI1_ENABLE);
	} else {
		ddbwritel(dev, 0x0fffff0f, INTERRUPT_ENABLE);
		ddbwritel(dev, 0x00000000, MSI1_ENABLE);
	}
	return stat;
}

static int ddb_probe(struct pci_dev *pdev,
			       const struct pci_device_id *id)
{
	struct ddb *dev;
	int stat = 0;

	if (pci_enable_device(pdev) < 0)
		return -ENODEV;

	pci_set_master(pdev);

	if (pci_set_dma_mask(pdev, DMA_BIT_MASK(64)))
		if (pci_set_dma_mask(pdev, DMA_BIT_MASK(32)))
			return -ENODEV;

	dev = vzalloc(sizeof(struct ddb));
	if (dev == NULL)
		return -ENOMEM;

	mutex_init(&dev->mutex);
	dev->has_dma = 1;
	dev->pdev = pdev;
	dev->dev = &pdev->dev;
	pci_set_drvdata(pdev, dev);

	dev->link[0].ids.vendor = id->vendor;
	dev->link[0].ids.device = id->device;
	dev->link[0].ids.subvendor = id->subvendor;
	dev->link[0].ids.subdevice = id->subdevice;

	dev->link[0].dev = dev;
	dev->link[0].info = (struct ddb_info *) id->driver_data;
	pr_info("detected %s\n", dev->link[0].info->name);

	dev->regs_len = pci_resource_len(dev->pdev, 0);
	dev->regs = ioremap(pci_resource_start(dev->pdev, 0),
			    pci_resource_len(dev->pdev, 0));

	if (!dev->regs) {
		pr_err("not enough memory for register map\n");
		stat = -ENOMEM;
		goto fail;
	}
	if (ddbreadl(dev, 0) == 0xffffffff) {
		pr_err("cannot read registers\n");
		stat = -ENODEV;
		goto fail;
	}

	dev->link[0].ids.hwid = ddbreadl(dev, 0);
	dev->link[0].ids.regmapid = ddbreadl(dev, 4);

	pr_info("HW %08x REGMAP %08x\n",
		dev->link[0].ids.hwid, dev->link[0].ids.regmapid);

	ddbwritel(dev, 0, DMA_BASE_READ);
	ddbwritel(dev, 0, DMA_BASE_WRITE);

	stat = ddb_irq_init(dev);
	if (stat < 0)
		goto fail0;

	if (ddb_init(dev) == 0)
		return 0;

	ddb_irq_disable(dev);
fail0:
	pr_err("fail0\n");
	if (dev->msi)
		pci_disable_msi(dev->pdev);
fail:
	pr_err("fail\n");

	ddb_unmap(dev);
	pci_set_drvdata(pdev, NULL);
	pci_disable_device(pdev);
	return -1;
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static const struct ddb_info ddb_none = {
	.type     = DDB_NONE,
	.name     = "unknown Digital Devices PCIe card, install newer driver",
	.regmap   = &octopus_map,
};

static const struct ddb_info ddb_octopus = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus DVB adapter",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
};

static const struct ddb_info ddb_octopusv3 = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus V3 DVB adapter",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
};

static const struct ddb_info ddb_octopus_le = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus LE DVB adapter",
	.regmap   = &octopus_map,
	.port_num = 2,
	.i2c_mask = 0x03,
};

static const struct ddb_info ddb_octopus_oem = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus OEM",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
	.led_num  = 1,
	.fan_num  = 1,
	.temp_num = 1,
	.temp_bus = 0,
};

static const struct ddb_info ddb_octopus_mini = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus Mini",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
};

static const struct ddb_info ddb_v6 = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Cine S2 V6 DVB adapter",
	.regmap   = &octopus_map,
	.port_num = 3,
	.i2c_mask = 0x07,
};

static const struct ddb_info ddb_v6_5 = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Cine S2 V6.5 DVB adapter",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
};

static const struct ddb_info ddb_v7 = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Cine S2 V7 DVB adapter",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
	.board_control   = 2,
	.board_control_2 = 4,
	.ts_quirks = TS_QUIRK_REVERSED,
};

static const struct ddb_info ddb_v7a = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Cine S2 V7 Advanced DVB adapter",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
	.board_control   = 2,
	.board_control_2 = 4,
	.ts_quirks = TS_QUIRK_REVERSED,
};

static const struct ddb_info ddb_ctv7 = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Cine CT V7 DVB adapter",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
	.board_control   = 3,
	.board_control_2 = 4,
};

static const struct ddb_info ddb_satixS2v3 = {
	.type     = DDB_OCTOPUS,
	.name     = "Mystique SaTiX-S2 V3 DVB adapter",
	.regmap   = &octopus_map,
	.port_num = 3,
	.i2c_mask = 0x07,
};

static const struct ddb_info ddb_ci = {
	.type     = DDB_OCTOPUS_CI,
	.name     = "Digital Devices Octopus CI",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x03,
};

static const struct ddb_info ddb_cis = {
	.type     = DDB_OCTOPUS_CI,
	.name     = "Digital Devices Octopus CI single",
	.regmap   = &octopus_map,
	.port_num = 3,
	.i2c_mask = 0x03,
};

static const struct ddb_info ddb_ci_s2_pro = {
	.type     = DDB_OCTOPUS_CI,
	.name     = "Digital Devices Octopus CI S2 Pro",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x01,
	.board_control   = 2,
	.board_control_2 = 4,
};

static const struct ddb_info ddb_ci_s2_pro_a = {
	.type     = DDB_OCTOPUS_CI,
	.name     = "Digital Devices Octopus CI S2 Pro Advanced",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x01,
	.board_control   = 2,
	.board_control_2 = 4,
};

static const struct ddb_info ddb_dvbct = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices DVBCT V6.1 DVB adapter",
	.regmap   = &octopus_map,
	.port_num = 3,
	.i2c_mask = 0x07,
};

/****************************************************************************/

static struct ddb_info ddb_ct2_8 = {
	.type     = DDB_OCTOPUS_MAX_CT,
	.name     = "Digital Devices MAX A8 CT2",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
	.board_control   = 0x0ff,
	.board_control_2 = 0xf00,
	.ts_quirks = TS_QUIRK_SERIAL,
	.tempmon_irq = 24,
};

static struct ddb_info ddb_c2t2_8 = {
	.type     = DDB_OCTOPUS_MAX_CT,
	.name     = "Digital Devices MAX A8 C2T2",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
	.board_control   = 0x0ff,
	.board_control_2 = 0xf00,
	.ts_quirks = TS_QUIRK_SERIAL,
	.tempmon_irq = 24,
};

static struct ddb_info ddb_isdbt_8 = {
	.type     = DDB_OCTOPUS_MAX_CT,
	.name     = "Digital Devices MAX A8 ISDBT",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
	.board_control   = 0x0ff,
	.board_control_2 = 0xf00,
	.ts_quirks = TS_QUIRK_SERIAL,
	.tempmon_irq = 24,
};

static struct ddb_info ddb_c2t2i_v0_8 = {
	.type     = DDB_OCTOPUS_MAX_CT,
	.name     = "Digital Devices MAX A8 C2T2I V0",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
	.board_control   = 0x0ff,
	.board_control_2 = 0xf00,
	.ts_quirks = TS_QUIRK_SERIAL | TS_QUIRK_ALT_OSC,
	.tempmon_irq = 24,
};

static struct ddb_info ddb_c2t2i_8 = {
	.type     = DDB_OCTOPUS_MAX_CT,
	.name     = "Digital Devices MAX A8 C2T2I",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
	.board_control   = 0x0ff,
	.board_control_2 = 0xf00,
	.ts_quirks = TS_QUIRK_SERIAL,
	.tempmon_irq = 24,
};

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

#define DDVID 0xdd01 /* Digital Devices Vendor ID */

#define DDB_ID(_vend, _dev, _subvend, _subdev, _driverdata) { \
	.vendor      = _vend,    .device    = _dev, \
	.subvendor   = _subvend, .subdevice = _subdev, \
	.driver_data = (unsigned long)&_driverdata }

static const struct pci_device_id ddb_id_tbl[] = {
	DDB_ID(DDVID, 0x0002, DDVID, 0x0001, ddb_octopus),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0001, ddb_octopus),
	DDB_ID(DDVID, 0x0005, DDVID, 0x0004, ddb_octopusv3),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0002, ddb_octopus_le),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0003, ddb_octopus_oem),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0010, ddb_octopus_mini),
	DDB_ID(DDVID, 0x0005, DDVID, 0x0011, ddb_octopus_mini),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0020, ddb_v6),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0021, ddb_v6_5),
	DDB_ID(DDVID, 0x0006, DDVID, 0x0022, ddb_v7),
	DDB_ID(DDVID, 0x0006, DDVID, 0x0024, ddb_v7a),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0030, ddb_dvbct),
	DDB_ID(DDVID, 0x0003, DDVID, 0xdb03, ddb_satixS2v3),
	DDB_ID(DDVID, 0x0006, DDVID, 0x0031, ddb_ctv7),
	DDB_ID(DDVID, 0x0006, DDVID, 0x0032, ddb_ctv7),
	DDB_ID(DDVID, 0x0006, DDVID, 0x0033, ddb_ctv7),
	DDB_ID(DDVID, 0x0008, DDVID, 0x0034, ddb_ct2_8),
	DDB_ID(DDVID, 0x0008, DDVID, 0x0035, ddb_c2t2_8),
	DDB_ID(DDVID, 0x0008, DDVID, 0x0036, ddb_isdbt_8),
	DDB_ID(DDVID, 0x0008, DDVID, 0x0037, ddb_c2t2i_v0_8),
	DDB_ID(DDVID, 0x0008, DDVID, 0x0038, ddb_c2t2i_8),
	DDB_ID(DDVID, 0x0006, DDVID, 0x0039, ddb_ctv7),
	DDB_ID(DDVID, 0x0011, DDVID, 0x0040, ddb_ci),
	DDB_ID(DDVID, 0x0011, DDVID, 0x0041, ddb_cis),
	DDB_ID(DDVID, 0x0012, DDVID, 0x0042, ddb_ci),
	DDB_ID(DDVID, 0x0013, DDVID, 0x0043, ddb_ci_s2_pro),
	DDB_ID(DDVID, 0x0013, DDVID, 0x0044, ddb_ci_s2_pro_a),
	/* in case sub-ids got deleted in flash */
	DDB_ID(DDVID, 0x0003, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	DDB_ID(DDVID, 0x0005, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	DDB_ID(DDVID, 0x0006, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	DDB_ID(DDVID, 0x0007, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	DDB_ID(DDVID, 0x0008, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	DDB_ID(DDVID, 0x0011, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	DDB_ID(DDVID, 0x0013, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	DDB_ID(DDVID, 0x0201, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	DDB_ID(DDVID, 0x0320, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	{0}
};
MODULE_DEVICE_TABLE(pci, ddb_id_tbl);

static struct pci_driver ddb_pci_driver = {
	.name        = "ddbridge",
	.id_table    = ddb_id_tbl,
	.probe       = ddb_probe,
	.remove      = ddb_remove,
};

static __init int module_init_ddbridge(void)
{
	int stat = -1;

	pr_info("Digital Devices PCIE bridge driver "
		DDBRIDGE_VERSION
		", Copyright (C) 2010-16 Digital Devices GmbH\n");
	if (ddb_class_create() < 0)
		return -1;
	ddb_wq = create_workqueue("ddbridge");
	if (ddb_wq == NULL)
		goto exit1;
	stat = pci_register_driver(&ddb_pci_driver);
	if (stat < 0)
		goto exit2;
	return stat;
exit2:
	destroy_workqueue(ddb_wq);
exit1:
	ddb_class_destroy();
	return stat;
}

static __exit void module_exit_ddbridge(void)
{
	pci_unregister_driver(&ddb_pci_driver);
	destroy_workqueue(ddb_wq);
	ddb_class_destroy();
}

module_init(module_init_ddbridge);
module_exit(module_exit_ddbridge);

MODULE_DESCRIPTION("Digital Devices PCIe Bridge");
MODULE_AUTHOR("Ralph and Marcus Metzler, Metzler Brothers Systementwicklung GbR");
MODULE_LICENSE("GPL");
MODULE_VERSION(DDBRIDGE_VERSION);
