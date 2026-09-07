// SPDX-License-Identifier: GPL-2.0+
//
// Loongson SPI Controller driver
//
// Copyright (C) 2023-2026 Loongson Technology Corporation Limited

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#include "spi-loongson.h"

/* SPI controller register offsets */
#define LOONGSON_SPI_SPCR_REG		0x00
#define LOONGSON_SPI_SPSR_REG		0x01
#define LOONGSON_SPI_FIFO_REG		0x02
#define LOONGSON_SPI_SPER_REG		0x03
#define LOONGSON_SPI_PARA_REG		0x04
#define LOONGSON_SPI_SFCS_REG		0x05
#define LOONGSON_SPI_TIMI_REG		0x06

/* SPCR bits */
#define LOONGSON_SPI_SPCR_SPR		GENMASK(1, 0)
#define LOONGSON_SPI_SPCR_CPHA		BIT(2)
#define LOONGSON_SPI_SPCR_CPOL		BIT(3)
#define LOONGSON_SPI_SPCR_SPE		BIT(6)
#define LOONGSON_SPI_SPCR_SPIE		BIT(7)

/* SPSR bits */
#define LOONGSON_SPI_SPSR_RFEMPTY	BIT(0)
#define LOONGSON_SPI_SPSR_RFFULL	BIT(1)
#define LOONGSON_SPI_SPSR_WFEMPTY	BIT(2)
#define LOONGSON_SPI_SPSR_WFFULL	BIT(3)
#define LOONGSON_SPI_SPSR_WCOL		BIT(6)
#define LOONGSON_SPI_SPSR_SPIF		BIT(7)

/* SPER bits */
#define LOONGSON_SPI_SPER_SPRE		GENMASK(1, 0)

/* SFC_PARAM bits */
#define LOONGSON_SPI_PARA_MEM_EN	BIT(0)

struct loongson_spi_priv {
	struct spi_controller	*controller;
	void __iomem		*base;
	u8			spcr;
	u8			sper;
	u8			spsr;
	u8			para;
	u8			sfcs;
	u8			timi;
	u32			hz;
	u32			mode;
	u64			clk_rate;
};

static inline void loongson_spi_write_reg(struct loongson_spi_priv *priv, u32 reg, u8 data)
{
	writeb(data, priv->base + reg);
}

static inline char loongson_spi_read_reg(struct loongson_spi_priv *priv, u32 reg)
{
	return readb(priv->base + reg);
}

static void loongson_spi_set_cs(struct spi_device *spi, bool en)
{
	struct loongson_spi_priv *priv = spi_controller_get_devdata(spi->controller);
	u8 mask, val;
	int cs;

	mask = (BIT(4) | BIT(0)) << spi_get_chipselect(spi, 0);
	val = en ? mask : (BIT(0) << spi_get_chipselect(spi, 0));

	cs = loongson_spi_read_reg(priv, LOONGSON_SPI_SFCS_REG) & ~mask;
	loongson_spi_write_reg(priv, LOONGSON_SPI_SFCS_REG, val | cs);
}

static void loongson_spi_set_clk(struct loongson_spi_priv *priv, u32 hz)
{
	static const char rdiv[12] = {0, 1, 4, 2, 3, 5, 6, 7, 8, 9, 10, 11};
	u32 div, div_tmp;
	u8 val;

	div = clamp_val(DIV_ROUND_UP_ULL(priv->clk_rate, hz), 2, 4096);
	div_tmp = rdiv[fls(div - 1)];
	priv->spcr = (div_tmp & GENMASK(1, 0)) >> 0;
	priv->sper = (div_tmp & GENMASK(3, 2)) >> 2;

	val = loongson_spi_read_reg(priv, LOONGSON_SPI_SPCR_REG);
	val &= ~LOONGSON_SPI_SPCR_SPR;
	loongson_spi_write_reg(priv, LOONGSON_SPI_SPCR_REG, val | priv->spcr);

	val = loongson_spi_read_reg(priv, LOONGSON_SPI_SPER_REG);
	val &= ~LOONGSON_SPI_SPER_SPRE;
	loongson_spi_write_reg(priv, LOONGSON_SPI_SPER_REG, val | priv->sper);

	priv->hz = hz;
}

static void loongson_spi_set_mode(struct loongson_spi_priv *priv, struct spi_device *spi)
{
	u8 val;

	val = loongson_spi_read_reg(priv, LOONGSON_SPI_SPCR_REG);
	val &= ~(LOONGSON_SPI_SPCR_CPOL | LOONGSON_SPI_SPCR_CPHA);

	if (spi->mode & SPI_CPOL)
		val |= LOONGSON_SPI_SPCR_CPOL;

	if (spi->mode & SPI_CPHA)
		val |= LOONGSON_SPI_SPCR_CPHA;

	loongson_spi_write_reg(priv, LOONGSON_SPI_SPCR_REG, val);
	priv->mode |= spi->mode;
}

static int loongson_spi_update_state(struct loongson_spi_priv *priv, struct spi_device *spi,
				     struct spi_transfer *t)
{
	if (t && priv->hz != t->speed_hz)
		loongson_spi_set_clk(priv, t->speed_hz);

	if ((spi->mode ^ priv->mode) & SPI_MODE_X_MASK)
		loongson_spi_set_mode(priv, spi);

	return 0;
}

static int loongson_spi_setup(struct spi_device *spi)
{
	struct loongson_spi_priv *priv = spi_controller_get_devdata(spi->controller);

	if (spi->bits_per_word % 8)
		return -EINVAL;

	if (spi_get_chipselect(spi, 0) >= spi->controller->num_chipselect)
		return -EINVAL;

	priv->hz = 0;
	loongson_spi_set_cs(spi, true);

	return 0;
}

static int loongson_spi_write_read_8bit(struct spi_device *spi, const u8 **tx_buf,
					u8 **rx_buf, unsigned int num)
{
	struct loongson_spi_priv *priv = spi_controller_get_devdata(spi->controller);
	int ret;

	if (tx_buf && *tx_buf)
		loongson_spi_write_reg(priv, LOONGSON_SPI_FIFO_REG, *((*tx_buf)++));
	else
		loongson_spi_write_reg(priv, LOONGSON_SPI_FIFO_REG, 0);

	ret = readb_poll_timeout(priv->base + LOONGSON_SPI_SPSR_REG,
				 priv->spsr, (priv->spsr &
				 LOONGSON_SPI_SPSR_RFEMPTY) != LOONGSON_SPI_SPSR_RFEMPTY,
				 1, USEC_PER_MSEC);

	if (rx_buf && *rx_buf)
		*(*rx_buf)++ = loongson_spi_read_reg(priv, LOONGSON_SPI_FIFO_REG);
	else
		loongson_spi_read_reg(priv, LOONGSON_SPI_FIFO_REG);

	return ret;
}

static int loongson_spi_write_read(struct spi_device *spi, struct spi_transfer *xfer)
{
	const u8 *tx = xfer->tx_buf;
	u8 *rx = xfer->rx_buf;
	u32 count;
	int ret;

	count = xfer->len;
	do {
		ret = loongson_spi_write_read_8bit(spi, &tx, &rx, count);
		if (ret)
			break;
	} while (--count);

	return ret;
}

static int loongson_spi_prepare_message(struct spi_controller *host, struct spi_message *m)
{
	struct loongson_spi_priv *priv = spi_controller_get_devdata(host);

	priv->para = loongson_spi_read_reg(priv, LOONGSON_SPI_PARA_REG);
	loongson_spi_write_reg(priv, LOONGSON_SPI_PARA_REG, priv->para & ~LOONGSON_SPI_PARA_MEM_EN);

	return 0;
}

static int loongson_spi_transfer_one(struct spi_controller *host, struct spi_device *spi,
				     struct spi_transfer *xfer)
{
	struct loongson_spi_priv *priv = spi_controller_get_devdata(spi->controller);

	loongson_spi_update_state(priv, spi, xfer);
	if (xfer->len)
		return loongson_spi_write_read(spi, xfer);

	return 0;
}

static int loongson_spi_unprepare_message(struct spi_controller *host, struct spi_message *m)
{
	struct loongson_spi_priv *priv = spi_controller_get_devdata(host);

	loongson_spi_write_reg(priv, LOONGSON_SPI_PARA_REG, priv->para);

	return 0;
}

static void loongson_spi_reginit(struct loongson_spi_priv *priv)
{
	u8 val;

	val = loongson_spi_read_reg(priv, LOONGSON_SPI_SPCR_REG);
	val &= ~LOONGSON_SPI_SPCR_SPE;
	loongson_spi_write_reg(priv, LOONGSON_SPI_SPCR_REG, val);

	loongson_spi_write_reg(priv, LOONGSON_SPI_SPSR_REG,
			       (LOONGSON_SPI_SPSR_SPIF | LOONGSON_SPI_SPSR_WCOL));

	val = loongson_spi_read_reg(priv, LOONGSON_SPI_SPCR_REG);
	val |= LOONGSON_SPI_SPCR_SPE;
	loongson_spi_write_reg(priv, LOONGSON_SPI_SPCR_REG, val);
}

int loongson_spi_init_controller(struct device *dev, void __iomem *regs)
{
	struct loongson_spi_priv *priv;
	struct spi_controller *host;
	struct clk *clk;

	host = devm_spi_alloc_host(dev, sizeof(*priv));
	if (!host)
		return -ENOMEM;

	host->mode_bits = SPI_MODE_X_MASK | SPI_CS_HIGH;
	host->setup = loongson_spi_setup;
	host->prepare_message = loongson_spi_prepare_message;
	host->transfer_one = loongson_spi_transfer_one;
	host->unprepare_message = loongson_spi_unprepare_message;
	host->set_cs = loongson_spi_set_cs;
	host->num_chipselect = 4;
	dev_set_drvdata(dev, host);

	priv = spi_controller_get_devdata(host);
	priv->base = regs;
	priv->controller = host;

	clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "unable to get clock\n");

	priv->clk_rate = clk_get_rate(clk);
	loongson_spi_reginit(priv);

	priv->mode = 0;

	return devm_spi_register_controller(dev, host);
}
EXPORT_SYMBOL_NS_GPL(loongson_spi_init_controller, "SPI_LOONGSON_CORE");

static int loongson_spi_suspend(struct device *dev)
{
	struct spi_controller *host = dev_get_drvdata(dev);
	struct loongson_spi_priv *priv = spi_controller_get_devdata(host);

	spi_controller_suspend(host);

	priv->spcr = loongson_spi_read_reg(priv, LOONGSON_SPI_SPCR_REG);
	priv->sper = loongson_spi_read_reg(priv, LOONGSON_SPI_SPER_REG);
	priv->spsr = loongson_spi_read_reg(priv, LOONGSON_SPI_SPSR_REG);
	priv->para = loongson_spi_read_reg(priv, LOONGSON_SPI_PARA_REG);
	priv->sfcs = loongson_spi_read_reg(priv, LOONGSON_SPI_SFCS_REG);
	priv->timi = loongson_spi_read_reg(priv, LOONGSON_SPI_TIMI_REG);

	return 0;
}

static int loongson_spi_resume(struct device *dev)
{
	struct spi_controller *host = dev_get_drvdata(dev);
	struct loongson_spi_priv *priv = spi_controller_get_devdata(host);

	loongson_spi_write_reg(priv, LOONGSON_SPI_SPCR_REG, priv->spcr);
	loongson_spi_write_reg(priv, LOONGSON_SPI_SPER_REG, priv->sper);
	loongson_spi_write_reg(priv, LOONGSON_SPI_SPSR_REG, priv->spsr);
	loongson_spi_write_reg(priv, LOONGSON_SPI_PARA_REG, priv->para);
	loongson_spi_write_reg(priv, LOONGSON_SPI_SFCS_REG, priv->sfcs);
	loongson_spi_write_reg(priv, LOONGSON_SPI_TIMI_REG, priv->timi);

	spi_controller_resume(host);

	return 0;
}

DEFINE_SIMPLE_DEV_PM_OPS(loongson_spi_dev_pm_ops, loongson_spi_suspend, loongson_spi_resume);
EXPORT_SYMBOL_NS_GPL(loongson_spi_dev_pm_ops, "SPI_LOONGSON_CORE");

MODULE_DESCRIPTION("Loongson SPI core driver");
MODULE_LICENSE("GPL");
