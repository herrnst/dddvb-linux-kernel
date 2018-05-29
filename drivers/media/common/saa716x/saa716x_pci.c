#include <linux/kmod.h>
#include <linux/vmalloc.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/delay.h>

#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/interrupt.h>

#include "saa716x_spi.h"
#include "saa716x_msi.h"
#include "saa716x_priv.h"

#define DRIVER_NAME				"SAA716x Core"

static int saa716x_enable_msi(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev = saa716x->pdev;
	int err;

	err = pci_enable_msi(pdev);
	if (err) {
		dprintk(SAA716x_ERROR, 1, "MSI enable failed <%d>", err);
		return err;
	}

	return err;
}

static int saa716x_request_irq(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev = saa716x->pdev;
	struct saa716x_config *config = saa716x->config;
	int ret = 0;

	if (saa716x->int_type == MODE_MSI || saa716x->int_type == MODE_MSI_X) {
		dprintk(SAA716x_DEBUG, 1, "Using MSI mode");
		saa716x->int_type = MODE_MSI;
		ret = saa716x_enable_msi(saa716x);
	}

	if (ret) {
		dprintk(SAA716x_ERROR, 1, "INT-A Mode");
		saa716x->int_type = MODE_INTA;
	}

	if (saa716x->int_type == MODE_MSI) {
		ret = request_irq(pdev->irq,
				  config->irq_handler,
				  0,
				  DRIVER_NAME,
				  saa716x);

		if (ret) {
			pci_disable_msi(pdev);
			dprintk(SAA716x_ERROR, 1, "MSI registration failed");
			ret = -EIO;
		}
	}

	if (saa716x->int_type == MODE_INTA) {
		ret = request_irq(pdev->irq,
				  config->irq_handler,
				  IRQF_SHARED,
				  DRIVER_NAME,
				  saa716x);
		if (ret < 0) {
			dprintk(SAA716x_ERROR, 1, "SAA716x IRQ registration failed <%d>", ret);
			ret = -ENODEV;
		}
	}

	return ret;
}

static void saa716x_free_irq(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev = saa716x->pdev;

	free_irq(pdev->irq, saa716x);
	if (saa716x->int_type == MODE_MSI)
		pci_disable_msi(pdev);

}

int saa716x_pci_init(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev = saa716x->pdev;
	int err = 0, ret = -ENODEV, i, use_dac, pm_cap;
	u32 msi_cap;
	u8 revision;

	dprintk(SAA716x_ERROR, 1, "found a %s PCIe card", saa716x->config->model_name);

	err = pci_enable_device(pdev);
	if (err != 0) {
		ret = -ENODEV;
		dprintk(SAA716x_ERROR, 1, "ERROR: PCI enable failed (%i)", err);
		goto fail0;
	}

	if (!pci_set_dma_mask(pdev, DMA_BIT_MASK(64))) {
		use_dac = 1;
		err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(64));
		if (err) {
			dprintk(SAA716x_ERROR, 1, "Unable to obtain 64bit DMA");
			goto fail1;
		}
	} else if ((err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32))) != 0) {
		dprintk(SAA716x_ERROR, 1, "Unable to obtain 32bit DMA");
		goto fail1;
	}

	pci_set_master(pdev);

	pm_cap = pci_find_capability(pdev, PCI_CAP_ID_PM);
	if (pm_cap == 0) {
		dprintk(SAA716x_ERROR, 1, "Cannot find Power Management Capability");
		err = -EIO;
		goto fail1;
	}

	if (!request_mem_region(pci_resource_start(pdev, 0),
				pci_resource_len(pdev, 0),
				DRIVER_NAME)) {

		dprintk(SAA716x_ERROR, 1, "BAR0 Request failed");
		ret = -ENODEV;
		goto fail1;
	}

	if (pci_resource_len(pdev, 0) < 0x30000) {
		dprintk(SAA716x_ERROR, 1, "wrong BAR0 length");
		ret = -ENODEV;
		goto fail2;
	}

	saa716x->mmio = ioremap_nocache(pci_resource_start(pdev, 0), 0x30000);
	if (!saa716x->mmio) {
		dprintk(SAA716x_ERROR, 1, "Mem 0 remap failed");
		ret = -ENODEV;
		goto fail2;
	}

	for (i = 0; i < SAA716x_MSI_MAX_VECTORS; i++)
		saa716x->msix_entries[i].entry = i;

	err = saa716x_request_irq(saa716x);
	if (err < 0) {
		dprintk(SAA716x_ERROR, 1, "SAA716x IRQ registration failed, err=%d", err);
		ret = -ENODEV;
		goto fail3;
	}

	pci_read_config_byte(pdev, PCI_CLASS_REVISION, &revision);
	pci_read_config_dword(pdev, 0x40, &msi_cap);

	saa716x->revision	= revision;

	dprintk(SAA716x_ERROR, 0, "    SAA%02x Rev %d [%04x:%04x], irq: %d, mmio: 0x%p",
		saa716x->pdev->device,
		revision,
		saa716x->pdev->subsystem_vendor,
		saa716x->pdev->subsystem_device,
		saa716x->pdev->irq,
		saa716x->mmio);

	dprintk(SAA716x_ERROR, 0, "    SAA%02x %sBit, MSI %s, %d/%d MSI vectors",
		saa716x->pdev->device,
		(((msi_cap >> 23) & 0x01) == 1 ? "64":"32"),
		(((msi_cap >> 16) & 0x01) == 1 ? "Enabled" : "Disabled"),
		(1 << ((msi_cap >> 20) & 0x07)),
		(1 << ((msi_cap >> 17) & 0x07)));

	pci_set_drvdata(pdev, saa716x);

	return 0;

fail3:
	dprintk(SAA716x_ERROR, 1, "Err: IO Unmap");
	if (saa716x->mmio)
		iounmap(saa716x->mmio);
fail2:
	dprintk(SAA716x_ERROR, 1, "Err: Release regions");
	release_mem_region(pci_resource_start(pdev, 0),
			   pci_resource_len(pdev, 0));

fail1:
	dprintk(SAA716x_ERROR, 1, "Err: Disabling device");
	pci_disable_device(pdev);

fail0:
	pci_set_drvdata(pdev, NULL);
	return ret;
}
EXPORT_SYMBOL_GPL(saa716x_pci_init);

void saa716x_pci_exit(struct saa716x_dev *saa716x)
{
	struct pci_dev *pdev = saa716x->pdev;

	saa716x_free_irq(saa716x);

	dprintk(SAA716x_NOTICE, 1, "SAA%02x mem0: 0x%p",
		saa716x->pdev->device,
		saa716x->mmio);

	if (saa716x->mmio) {
		iounmap(saa716x->mmio);
		release_mem_region(pci_resource_start(pdev, 0),
				   pci_resource_len(pdev, 0));
	}

	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
}
EXPORT_SYMBOL_GPL(saa716x_pci_exit);

MODULE_DESCRIPTION("SAA716x bridge driver");
MODULE_AUTHOR("Manu Abraham");
MODULE_LICENSE("GPL");
