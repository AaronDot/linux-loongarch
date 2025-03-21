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
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/platform_data/simplefb.h>
#include <linux/platform_device.h>

#include <linux/delay.h>
#include <linux/minmax.h>
#include <linux/stop_machine.h>

#define LS2K_IPMI_RES_SIZE	0x1c
#define LS2K_IPMI0_RES_START	(SZ_16M + 0xf00000)
#define LS2K_IPMI1_RES_START	(LS2K_IPMI0_RES_START + LS2K_IPMI_RES_SIZE)
#define LS2K_IPMI2_RES_START	(LS2K_IPMI1_RES_START + LS2K_IPMI_RES_SIZE)
#define LS2K_IPMI3_RES_START	(LS2K_IPMI2_RES_START + LS2K_IPMI_RES_SIZE)
#define LS2K_IPMI4_RES_START	(LS2K_IPMI3_RES_START + LS2K_IPMI_RES_SIZE)

#define LS2K_BMC_RESET_DELAY	(60 * HZ)
#define LS2K_BMC_RESET_WAIT	10000

static const u32 index[] = { 0x4, 0x10, 0x14, 0x18, 0x1c, 0x20, 0x24,
			     0x30, 0x3c, 0x54, 0x58, 0x78, 0x7c, 0x80 };
static const u32 cindex[] = { 0x4, 0x10, 0x3c };


struct ls2kbmc_pci_data {
	u32 d80c;
	u32 d71c;
	u32 data[14];
	u32 cdata[3];
};

struct ls2kbmc_pdata {
	struct device *dev;
	struct platform_device *pdev;
	struct work_struct bmc_work;
	unsigned long reset_time;
	struct ls2kbmc_pci_data pci_data;
	struct simplefb_platform_data pd;
};

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
	MFD_CELL_RES("ls2k-ipmi-si", ls2k_ipmi_resources),
};

/*
 * Currently the Loongson-2K0500 BMC hardware does not have an i2c interface to
 * adapt to the resolution.
 * We set the resolution by presetting "video=1280x1024-16@2M" to the bmc memory.
 */
static int ls2kbmc_get_video_mode(struct pci_dev *pdev, struct ls2kbmc_pdata *priv)
{
	char *mode;
	int depth, ret;
	struct simplefb_platform_data *pd = &priv->pd;

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

static bool ls2kbmc_bar0_addr_is_set(struct pci_dev *ppdev)
{
	u32 addr;

	pci_read_config_dword(ppdev, PCI_BASE_ADDRESS_0, &addr);
	addr &= PCI_BASE_ADDRESS_MEM_MASK;

	return addr ? true : false;
}

static void ls2kbmc_save_pci_data(struct pci_dev *pdev, struct ls2kbmc_pdata *priv)
{
	struct pci_dev *parent = pdev->bus->self;
	int i;

	for (i = 0; i < ARRAY_SIZE(index); i++)
		pci_read_config_dword(parent, index[i], &priv->pci_data.data[i]);

	for (i = 0; i < ARRAY_SIZE(cindex); i++)
		pci_read_config_dword(pdev, cindex[i], &priv->pci_data.cdata[i]);

	pci_read_config_dword(parent, 0x80c, &priv->pci_data.d80c);
	priv->pci_data.d80c = (priv->pci_data.d80c & ~(3 << 17)) | (1 << 17);

	pci_read_config_dword(parent, 0x71c, &priv->pci_data.d71c);
	priv->pci_data.d71c |= 1 << 26;
}

static bool ls2kbmc_check_pcie_connected(struct pci_dev *parent)
{
	void __iomem *mmio;
	int sts, ret;

	mmio = pci_iomap(parent, 0, 0x100);
	if (!mmio)
		return false;

	writel(readl(mmio) | 0x8, mmio);

	ret = readl_poll_timeout(mmio + 0xc, sts, (sts & 0x11) == 0x11, 1, 1000);
	if (ret) {
		pr_err("pcie train failed status=0x%x\n", sts);
		return false;
	}

	pci_iounmap(parent, mmio);

	return true;
}

static int ls2kbmc_recove_pci_data(void *data)
{
	struct ls2kbmc_pdata *priv = data;
	struct pci_dev *pdev = to_pci_dev(priv->dev);
	struct pci_dev *parent = pdev->bus->self;
	u32 i, timeout, retry = 0;
	bool ready;

	pci_write_config_dword(parent, PCI_BASE_ADDRESS_2, 0);
	pci_write_config_dword(parent, PCI_BASE_ADDRESS_3, 0);
	pci_write_config_dword(parent, PCI_BASE_ADDRESS_4, 0);

	timeout = 10000;
	while (timeout) {
		ready = ls2kbmc_bar0_addr_is_set(parent);
		if (!ready)
			break;
		mdelay(1);
		timeout--;
	};

	if (!timeout)
		pr_warn("bar not clear 0\n");

retrain:
	for (i = 0; i < ARRAY_SIZE(index); i++)
		pci_write_config_dword(parent, index[i], priv->pci_data.data[i]);

	pci_write_config_dword(parent, 0x80c, priv->pci_data.d80c);
	pci_write_config_dword(parent, 0x71c, priv->pci_data.d71c);

	/* Check if the pcie is connected */
	ready = ls2kbmc_check_pcie_connected(parent);
	if (!ready)
		return ready;

	for (i = 0; i < ARRAY_SIZE(cindex); i++)
		pci_write_config_dword(pdev, cindex[i], priv->pci_data.cdata[i]);

	pr_info("pcie recovered done\n");

	if (!retry) {
		/*wait u-boot ddr config */
		mdelay(LS2K_BMC_RESET_WAIT);
		ready = ls2kbmc_bar0_addr_is_set(parent);
		if (!ready) {
			retry = 1;
			goto retrain;
		}
	}

	return 0;
}

static void ls2kbmc_events_fn(struct work_struct *work)
{
	struct ls2kbmc_pdata *priv = container_of(work, struct ls2kbmc_pdata, bmc_work);

	platform_device_unregister(priv->pdev);

	/*
	 * The pcie is lost when the BMC resets,
	 * at which point access to the pcie from other CPUs
	 * is suspended to prevent a crash.
	 */
	stop_machine(ls2kbmc_recove_pci_data, priv, NULL);

	pr_info("redraw console\n");

	/* We need to re-push the display due to previous pcie loss. */
	priv->pdev = platform_device_register_resndata(NULL, "simple-framebuffer", 0,
						       &ls2k_display_resources[0], 1,
						       &priv->pd, sizeof(priv->pd));
}

static irqreturn_t ls2kbmc_interrupt(int irq, void *arg)
{
	struct ls2kbmc_pdata *priv = arg;

	if (system_state != SYSTEM_RUNNING)
		return IRQ_HANDLED;

	/* skip interrupt in LS2K_BMC_RESET_DELAY */
	if (time_after(jiffies, priv->reset_time + LS2K_BMC_RESET_DELAY))
		schedule_work(&priv->bmc_work);

	priv->reset_time = jiffies;

	return IRQ_HANDLED;
}

#define BMC_RESET_GPIO			14
#define LOONGSON_GPIO_REG_BASE		0x1fe00500
#define LOONGSON_GPIO_REG_SIZE		0x18
#define LOONGSON_GPIO_OEN		0x0
#define LOONGSON_GPIO_FUNC		0x4
#define LOONGSON_GPIO_INTPOL		0x10
#define LOONGSON_GPIO_INTEN		0x14

/* The gpio interrupt is a watchdog interrupt that is triggered when the BMC resets. */
static int ls2kbmc_gpio_reset_handler(struct ls2kbmc_pdata *priv)
{
	int irq, ret = 0;
	int gsi = 16 + (BMC_RESET_GPIO & 7);
	void __iomem *gpio_base;

	/* Since Loongson-3A hardware does not support GPIO interrupt cascade,
	 * chip->gpio_to_irq() cannot be implemented,
	 * here acpi_register_gsi() is used to get gpio irq.
	 */
	irq = acpi_register_gsi(NULL, gsi, ACPI_EDGE_SENSITIVE, ACPI_ACTIVE_LOW);
	if (irq < 0)
		return irq;

	gpio_base = ioremap(LOONGSON_GPIO_REG_BASE, LOONGSON_GPIO_REG_SIZE);
	if (!gpio_base) {
		ret = PTR_ERR(gpio_base);
		goto acpi_failed;
	}

	writel(readl(gpio_base + LOONGSON_GPIO_OEN) | BIT(BMC_RESET_GPIO),
	       gpio_base + LOONGSON_GPIO_OEN);
	writel(readl(gpio_base + LOONGSON_GPIO_FUNC) & ~BIT(BMC_RESET_GPIO),
	       gpio_base + LOONGSON_GPIO_FUNC);
	writel(readl(gpio_base + LOONGSON_GPIO_INTPOL) & ~BIT(BMC_RESET_GPIO),
	       gpio_base + LOONGSON_GPIO_INTPOL);
	writel(readl(gpio_base + LOONGSON_GPIO_INTEN) | BIT(BMC_RESET_GPIO),
	       gpio_base + LOONGSON_GPIO_INTEN);

	ret = request_irq(irq, ls2kbmc_interrupt, IRQF_SHARED | IRQF_TRIGGER_FALLING,
			  "ls2kbmc gpio", priv);

	iounmap(gpio_base);

acpi_failed:
	acpi_unregister_gsi(gsi);

	return ret;
}

static int ls2kbmc_pdata_initial(struct pci_dev *pdev, struct ls2kbmc_pdata *priv)
{
	int ret;

	ls2kbmc_save_pci_data(pdev, priv);

	INIT_WORK(&priv->bmc_work, ls2kbmc_events_fn);

	ret = request_irq(pdev->irq, ls2kbmc_interrupt,
			  IRQF_SHARED | IRQF_TRIGGER_RISING, "ls2kbmc pcie", priv);
	if (ret) {
		pr_err("request_irq(%d) failed\n", pdev->irq);
		return ret;
	}

	return ls2kbmc_gpio_reset_handler(priv);
}

static int ls2k_bmc_register_simplefb(struct pci_dev *dev, struct ls2kbmc_pdata *priv)
{
	ls2kbmc_get_video_mode(dev, priv);

	priv->pdev = platform_device_register_resndata(NULL, "simple-framebuffer", 0,
						       &ls2k_display_resources[0], 1,
						       &priv->pd, sizeof(priv->pd));

	if (IS_ERR(priv->pdev))
		return -ENOMEM;

	return 0;
}

static int ls2k_bmc_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int ret = 0;
	struct ls2kbmc_pdata *priv;

	ret = pci_enable_device(dev);
	if (ret)
		return ret;

	priv = devm_kzalloc(&dev->dev, sizeof(*priv), GFP_KERNEL);
	if (IS_ERR(priv)) {
		ret = -ENOMEM;
		goto disable_pci;
	}

	priv->dev = &dev->dev;
	pci_set_drvdata(dev, priv);

	ret = ls2kbmc_pdata_initial(dev, priv);
	if (ret)
		goto disable_pci;

	ret = ls2k_bmc_register_simplefb(dev, priv);
	if (ret)
		goto disable_pci;

	return devm_mfd_add_devices(&dev->dev, PLATFORM_DEVID_AUTO,
				    ls2k_bmc_cells, ARRAY_SIZE(ls2k_bmc_cells),
				    &dev->resource[0], 0, NULL);

disable_pci:
       pci_disable_device(dev);
       return ret;
}

static void ls2k_bmc_remove(struct pci_dev *dev)
{
	struct ls2kbmc_pdata *priv = pci_get_drvdata(dev);

	pci_disable_device(dev);

	platform_device_unregister(priv->pdev);
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
