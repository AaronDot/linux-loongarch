// SPDX-License-Identifier: GPL-2.0-only
/*
 * Loongson-2K Board Management Controller (BMC) MFD Core Driver.
 *
 * Copyright (C) 2024 Loongson Technology Corporation Limited.
 *
 * Originally written by Chong Qiao <qiaochong@loongson.cn>
 * Rewritten for mainline by Binbin Zhou <zhoubinbin@loongson.cn>
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/platform_data/simplefb.h>
#include <linux/platform_device.h>

#define LS2K_IPMI_RES_SIZE	0x1c
#define LS2K_IPMI0_RES_START	(SZ_16M + 0xf00000)
#define LS2K_IPMI1_RES_START	(LS2K_IPMI0_RES_START + LS2K_IPMI_RES_SIZE)
#define LS2K_IPMI2_RES_START	(LS2K_IPMI1_RES_START + LS2K_IPMI_RES_SIZE)
#define LS2K_IPMI3_RES_START	(LS2K_IPMI2_RES_START + LS2K_IPMI_RES_SIZE)
#define LS2K_IPMI4_RES_START	(LS2K_IPMI3_RES_START + LS2K_IPMI_RES_SIZE)

static struct resource ls2k_display_resources[] = {
	DEFINE_RES_MEM_NAMED(SZ_16M + SZ_2M, SZ_4M, "simpledrm-res"),
};

static struct resource ls2k_ipmi_resources[] = {
	DEFINE_RES_MEM_NAMED(LS2K_IPMI0_RES_START, LS2K_IPMI_RES_SIZE, "ipmi0-res"),
	DEFINE_RES_MEM_NAMED(LS2K_IPMI1_RES_START, LS2K_IPMI_RES_SIZE, "ipmi1-res"),
	DEFINE_RES_MEM_NAMED(LS2K_IPMI2_RES_START, LS2K_IPMI_RES_SIZE, "ipmi2-res"),
	DEFINE_RES_MEM_NAMED(LS2K_IPMI3_RES_START, LS2K_IPMI_RES_SIZE, "ipmi3-res"),
	DEFINE_RES_MEM_NAMED(LS2K_IPMI4_RES_START, LS2K_IPMI_RES_SIZE, "ipmi4-res"),
};

static struct mfd_cell ls2k_bmc_cells[] = {
	MFD_CELL_RES("simple-framebuffer", ls2k_display_resources),
	MFD_CELL_RES("ls2k-ipmi-si", ls2k_ipmi_resources),
};

/*
 * Currently the Loongson-2K0500 BMC hardware does not have an i2c interface to
 * adapt to the resolution.
 * We set the resolution by presetting "video=1280x1024-16@2M" to the bmc memory.
 */
static int ls2kbmc_get_video_mode(struct pci_dev *pdev, struct simplefb_platform_data *pd)
{
	char *mode;
	int depth, ret;

	/* The pci mem bar last 16M is used to store the string. */
	mode = devm_ioremap(&pdev->dev, pci_resource_start(pdev, 0) + SZ_16M, SZ_16M);
	if (!mode)
		return -ENOMEM;

	/*env at last 16M's beginning, first env is video */
	if (!strncmp(mode, "video=", 6))
		mode = mode + 6;

	ret = kstrtoint(strsep(&mode, "x"), 10, &pd->width);
	if (ret)
		return ret;

	ret = kstrtoint(strsep(&mode, "-"), 10, &pd->height);
	if (ret)
		return ret;

	ret = kstrtoint(strsep(&mode, "@"), 10, &depth);
	if (ret)
		return ret;

	pd->stride = pd->width * depth / 8;
	pd->format = depth == 32 ? "a8r8g8b8" : "r5g6b5";

	return 0;
}

static int ls2k_bmc_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int ret = 0;
	struct simplefb_platform_data pd;

	ret = pci_enable_device(dev);
	if (ret)
		return ret;

	ls2kbmc_get_video_mode(dev, &pd);

	ls2k_bmc_cells[0].platform_data = &pd;
	ls2k_bmc_cells[0].pdata_size = sizeof(pd);

	return devm_mfd_add_devices(&dev->dev, PLATFORM_DEVID_AUTO,
				    ls2k_bmc_cells, ARRAY_SIZE(ls2k_bmc_cells),
				    &dev->resource[0], 0, NULL);
}

static void ls2k_bmc_remove(struct pci_dev *dev)
{
	pci_disable_device(dev);
}

static struct pci_device_id ls2k_bmc_devices[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_LOONGSON, 0x1a05) },
	{ }
};
MODULE_DEVICE_TABLE(pci, ls2k_bmc_devices);

static struct pci_driver ls2k_bmc_driver = {
	.name = "ls2k-bmc",
	.id_table = ls2k_bmc_devices,
	.probe = ls2k_bmc_probe,
	.remove = ls2k_bmc_remove,
};

module_pci_driver(ls2k_bmc_driver);

MODULE_DESCRIPTION("Loongson-2K BMC driver");
MODULE_AUTHOR("Loongson Technology Corporation Limited");
MODULE_LICENSE("GPL");
