// SPDX-License-Identifier: GPL-2.0+
//
// Loongson SPI Controller driver
//
// Copyright (C) 2023-2026 Loongson Technology Corporation Limited

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
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
	struct regmap		*regmap;
	u32			hz;
	u32			mode;
	u64			clk_rate;

	/* Current transfer context (IRQ mode) */
	bool			use_irq;
	const u8		*tx_ptr;
	u8			*rx_ptr;
	u32			count;
	u32			total;

	/* suspend/resume register storage */
	u32			spcr;
	u32			sper;
	u32			spsr;
	u32			para;
	u32			sfcs;
	u32			timi;
};

static const struct regmap_config loongson_spi_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.reg_stride	= 1,
	.max_register	= LOONGSON_SPI_TIMI_REG,
	.fast_io	= true,
};

static void loongson_spi_set_cs(struct spi_device *spi, bool en)
{
	struct loongson_spi_priv *priv = spi_controller_get_devdata(spi->controller);
	int cs = spi_get_chipselect(spi, 0);
	u8 mask = BIT(cs) | BIT(4 + cs);
	u8 val = en ? mask : BIT(cs);

	regmap_update_bits(priv->regmap, LOONGSON_SPI_SFCS_REG, mask, val);
}

static void loongson_spi_set_clk(struct loongson_spi_priv *priv, u32 hz)
{
	static const char rdiv[12] = {0, 1, 4, 2, 3, 5, 6, 7, 8, 9, 10, 11};
	u32 div, div_tmp;

	div = clamp_val(DIV_ROUND_UP_ULL(priv->clk_rate, hz), 2, 4096);
	div_tmp = rdiv[fls(div - 1)];
	priv->spcr = div_tmp & GENMASK(1, 0);
	priv->sper = (div_tmp & GENMASK(3, 2)) >> 2;

	regmap_update_bits(priv->regmap, LOONGSON_SPI_SPCR_REG, LOONGSON_SPI_SPCR_SPR, priv->spcr);
	regmap_update_bits(priv->regmap, LOONGSON_SPI_SPER_REG, LOONGSON_SPI_SPER_SPRE, priv->sper);

	priv->hz = hz;
}

static void loongson_spi_set_mode(struct loongson_spi_priv *priv, struct spi_device *spi)
{
	u8 val = 0;

	if (spi->mode & SPI_CPOL)
		val |= LOONGSON_SPI_SPCR_CPOL;

	if (spi->mode & SPI_CPHA)
		val |= LOONGSON_SPI_SPCR_CPHA;

	regmap_update_bits(priv->regmap, LOONGSON_SPI_SPCR_REG,
			   LOONGSON_SPI_SPCR_CPOL | LOONGSON_SPI_SPCR_CPHA, val);

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

/* ---------------- Polling mode ---------------------------------------- */

static int loongson_spi_write_read_8bit(struct spi_device *spi, const u8 **tx_buf, u8 **rx_buf)
{
	struct loongson_spi_priv *priv = spi_controller_get_devdata(spi->controller);
	u32 val = 0;
	int ret;

	if (tx_buf && *tx_buf)
		val = *((*tx_buf)++);

	regmap_write(priv->regmap, LOONGSON_SPI_FIFO_REG, val);

	/* Wait for RX FIFO not empty */
	ret = regmap_read_poll_timeout(priv->regmap, LOONGSON_SPI_SPSR_REG, priv->spsr,
				       !(priv->spsr & LOONGSON_SPI_SPSR_RFEMPTY),
				       1, USEC_PER_MSEC);
	if (ret)
		return ret;

	regmap_read(priv->regmap, LOONGSON_SPI_FIFO_REG, &val);
	if (rx_buf && *rx_buf)
		*(*rx_buf)++ = (u8)val;

	return ret;
}

static int loongson_spi_write_read(struct spi_device *spi, struct spi_transfer *xfer)
{
	const u8 *tx = xfer->tx_buf;
	u8 *rx = xfer->rx_buf;
	u32 count = xfer->len;
	int ret;

	do {
		ret = loongson_spi_write_read_8bit(spi, &tx, &rx);
		if (ret)
			break;
	} while (--count);

	return ret;
}

/* ---------------- Interrupt mode --------------------------------------- */

static irqreturn_t loongson_spi_irq(int irq, void *dev_id)
{
	struct loongson_spi_priv *priv = dev_id;
	u32 status, data = 0;
	int ret;

	ret = regmap_read(priv->regmap, LOONGSON_SPI_SPSR_REG, &status);
	if (ret)
		return IRQ_NONE;

	if (!(status & (LOONGSON_SPI_SPSR_SPIF | LOONGSON_SPI_SPSR_WCOL)))
		return IRQ_NONE;

	/* Clear WCOL if it happens to be set */
	if (status & LOONGSON_SPI_SPSR_WCOL)
		regmap_write(priv->regmap, LOONGSON_SPI_SPSR_REG,
			     LOONGSON_SPI_SPSR_WCOL);

	if (!(status & LOONGSON_SPI_SPSR_SPIF))
		return IRQ_HANDLED;

	/* Clear the SPIF flag (write 1 to clear) */
	regmap_write(priv->regmap, LOONGSON_SPI_SPSR_REG,
		     LOONGSON_SPI_SPSR_SPIF);

	/* Read out the byte that was clocked in */
	regmap_read(priv->regmap, LOONGSON_SPI_FIFO_REG, &data);
	if (priv->rx_ptr)
		priv->rx_ptr[priv->count] = (u8)data;

	priv->count++;

	if (priv->count >= priv->total) {
		/* Transfer complete: disable the byte interrupt and finalize */
		regmap_update_bits(priv->regmap, LOONGSON_SPI_SPCR_REG,
				   LOONGSON_SPI_SPCR_SPIE, 0);
		spi_finalize_current_transfer(priv->controller);
		return IRQ_HANDLED;
	}

	/* Kick off the next byte */
	if (priv->tx_ptr)
		data = priv->tx_ptr[priv->count];
	else
		data = 0;

	regmap_write(priv->regmap, LOONGSON_SPI_FIFO_REG, data);

	return IRQ_HANDLED;
}

static int loongson_spi_transfer_one_irq(struct spi_controller *host,
					 struct spi_device *spi,
					 struct spi_transfer *xfer)
{
	struct loongson_spi_priv *priv = spi_controller_get_devdata(host);
	u32 data = 0;

	priv->tx_ptr = xfer->tx_buf;
	priv->rx_ptr = xfer->rx_buf;
	priv->count = 0;
	priv->total = xfer->len;

	/* Clear any stale status before enabling the interrupt */
	regmap_write(priv->regmap, LOONGSON_SPI_SPSR_REG,
		     LOONGSON_SPI_SPSR_SPIF | LOONGSON_SPI_SPSR_WCOL);

	/* Enable the byte-completion interrupt */
	regmap_update_bits(priv->regmap, LOONGSON_SPI_SPCR_REG,
			   LOONGSON_SPI_SPCR_SPIE, LOONGSON_SPI_SPCR_SPIE);

	/* Send the first byte to start the transfer */
	if (priv->tx_ptr)
		data = priv->tx_ptr[0];
	regmap_write(priv->regmap, LOONGSON_SPI_FIFO_REG, data);

	/*
	 * Return 1 so the SPI core knows the transfer will be
	 * finalized asynchronously from the IRQ handler.
	 */
	return 1;
}

/* ---------------- Common message handling ------------------------------ */

static int loongson_spi_prepare_message(struct spi_controller *host, struct spi_message *m)
{
	struct loongson_spi_priv *priv = spi_controller_get_devdata(host);

	regmap_read(priv->regmap, LOONGSON_SPI_PARA_REG, &priv->para);
	regmap_update_bits(priv->regmap, LOONGSON_SPI_PARA_REG, LOONGSON_SPI_PARA_MEM_EN, 0);

	return 0;
}

static int loongson_spi_transfer_one(struct spi_controller *host, struct spi_device *spi,
				     struct spi_transfer *xfer)
{
	struct loongson_spi_priv *priv = spi_controller_get_devdata(spi->controller);

	loongson_spi_update_state(priv, spi, xfer);

	if (!xfer->len)
		return 0;

	if (priv->use_irq)
		return loongson_spi_transfer_one_irq(host, spi, xfer);

	return loongson_spi_write_read(spi, xfer);
}

static int loongson_spi_unprepare_message(struct spi_controller *host, struct spi_message *m)
{
	struct loongson_spi_priv *priv = spi_controller_get_devdata(host);

	regmap_write(priv->regmap, LOONGSON_SPI_PARA_REG, priv->para);

	return 0;
}

static void loongson_spi_reginit(struct loongson_spi_priv *priv)
{
	/* Disable SPI and its interrupt */
	regmap_update_bits(priv->regmap, LOONGSON_SPI_SPCR_REG,
			   LOONGSON_SPI_SPCR_SPE | LOONGSON_SPI_SPCR_SPIE, 0);

	/* Clear status flags (write 1 to clear) */
	regmap_write(priv->regmap, LOONGSON_SPI_SPSR_REG,
		     LOONGSON_SPI_SPSR_SPIF | LOONGSON_SPI_SPSR_WCOL);

	/* Enable SPI (interrupt remains disabled until a transfer starts) */
	regmap_update_bits(priv->regmap, LOONGSON_SPI_SPCR_REG,
			   LOONGSON_SPI_SPCR_SPE, LOONGSON_SPI_SPCR_SPE);
}

static void loongson_spi_irqinit(struct loongson_spi_priv *priv,
				 struct device *dev, int irq)
{
	int ret;

	if (irq <= 0) {
		priv->use_irq = false;
		return;
	}

	ret = devm_request_irq(dev, irq, loongson_spi_irq, 0, dev_name(dev), priv);
	if (ret) {
		priv->use_irq = false;
		dev_warn(dev, "failed to request IRQ %d (%d), using polling\n", irq, ret);
		return;
	}

	priv->use_irq = true;
}

int loongson_spi_init_controller(struct device *dev, void __iomem *regs, int irq)
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
	priv->controller = host;

	priv->regmap = devm_regmap_init_mmio(dev, regs, &loongson_spi_regmap_config);
	if (IS_ERR(priv->regmap))
		return dev_err_probe(dev, PTR_ERR(priv->regmap), "unable to init regmap\n");

	clk = devm_clk_get_optional(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "unable to get clock\n");

	priv->clk_rate = clk_get_rate(clk);
	loongson_spi_reginit(priv);

	priv->mode = 0;

	loongson_spi_irqinit(priv, dev, irq);

	return devm_spi_register_controller(dev, host);
}
EXPORT_SYMBOL_NS_GPL(loongson_spi_init_controller, "SPI_LOONGSON_CORE");

static int loongson_spi_suspend(struct device *dev)
{
	struct spi_controller *host = dev_get_drvdata(dev);
	struct loongson_spi_priv *priv = spi_controller_get_devdata(host);

	spi_controller_suspend(host);

	regmap_read(priv->regmap, LOONGSON_SPI_SPCR_REG, &priv->spcr);
	regmap_read(priv->regmap, LOONGSON_SPI_SPER_REG, &priv->sper);
	regmap_read(priv->regmap, LOONGSON_SPI_SPSR_REG, &priv->spsr);
	regmap_read(priv->regmap, LOONGSON_SPI_PARA_REG, &priv->para);
	regmap_read(priv->regmap, LOONGSON_SPI_SFCS_REG, &priv->sfcs);
	regmap_read(priv->regmap, LOONGSON_SPI_TIMI_REG, &priv->timi);

	return 0;
}

static int loongson_spi_resume(struct device *dev)
{
	struct spi_controller *host = dev_get_drvdata(dev);
	struct loongson_spi_priv *priv = spi_controller_get_devdata(host);

	regmap_write(priv->regmap, LOONGSON_SPI_SPCR_REG, priv->spcr);
	regmap_write(priv->regmap, LOONGSON_SPI_SPER_REG, priv->sper);
	regmap_write(priv->regmap, LOONGSON_SPI_SPSR_REG, priv->spsr);
	regmap_write(priv->regmap, LOONGSON_SPI_PARA_REG, priv->para);
	regmap_write(priv->regmap, LOONGSON_SPI_SFCS_REG, priv->sfcs);
	regmap_write(priv->regmap, LOONGSON_SPI_TIMI_REG, priv->timi);

	spi_controller_resume(host);

	return 0;
}

DEFINE_SIMPLE_DEV_PM_OPS(loongson_spi_dev_pm_ops, loongson_spi_suspend, loongson_spi_resume);
EXPORT_SYMBOL_NS_GPL(loongson_spi_dev_pm_ops, "SPI_LOONGSON_CORE");

MODULE_DESCRIPTION("Loongson SPI core driver");
MODULE_LICENSE("GPL");
