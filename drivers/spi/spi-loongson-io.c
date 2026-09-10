// SPDX-License-Identifier: GPL-2.0+
//
// Platform driver for Loongson-2K0300 SPI IO Support
//
// Copyright (C) 2017-2026 Loongson Technology Corporation Limited

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/scatterlist.h>

/* Loongson SPI-IO Controller registers */
#define LOONGSON_SPI_IO_CR1_REG			0x00
#define LOONGSON_SPI_IO_CR2_REG			0x04
#define LOONGSON_SPI_IO_CR3_REG			0x08
#define LOONGSON_SPI_IO_CR4_REG			0x0c
#define LOONGSON_SPI_IO_IER_REG			0x10
#define LOONGSON_SPI_IO_SR1_REG			0x14
#define LOONGSON_SPI_IO_SR2_REG			0x18
#define LOONGSON_SPI_IO_CFG1_REG		0x20
#define LOONGSON_SPI_IO_CFG2_REG		0x24
#define LOONGSON_SPI_IO_CFG3_REG		0x28
#define LOONGSON_SPI_IO_CRC1_REG		0x30
#define LOONGSON_SPI_IO_CRC2_REG		0x34
#define LOONGSON_SPI_IO_DR_REG			0x40

/* Bitfields of LOONGSON_SPI_IO_CR1_REG */
#define LOONGSON_SPI_IO_CR1_SPE			BIT(0)
#define LOONGSON_SPI_IO_CR1_CSTART		BIT(1)
#define LOONGSON_SPI_IO_CR1_AUTOSUS		BIT(2)
#define LOONGSON_SPI_IO_CR1_SSREV		BIT(8)

/* Bitfields of LOONGSON_SPI_IO_CR2_REG */
#define LOONGSON_SPI_IO_CR2_RXFTHLV		GENMASK(1, 0)
#define LOONGSON_SPI_IO_CR2_RXDMAEN		BIT(7)
#define LOONGSON_SPI_IO_CR2_TXFTHLV		GENMASK(9, 8)
#define LOONGSON_SPI_IO_CR2_TXDMAEN		BIT(15)

/* Bitfields of LOONGSON_SPI_IO_CR3_REG */
#define LOONGSON_SPI_IO_CR3_TSIZE		GENMASK(15, 0)

/* Bitfields of LOONGSON_SPI_IO_CR4_REG */
#define LOONGSON_SPI_IO_CR4_CTSIZE		GENMASK(15, 0)

/* Bitfields of LOONGSON_SPI_IO_IER_REG */
#define LOONGSON_SPI_IO_IER_RXAIE		BIT(0)
#define LOONGSON_SPI_IO_IER_TXAIE		BIT(1)
#define LOONGSON_SPI_IO_IER_DXAIE		BIT(2)
#define LOONGSON_SPI_IO_IER_RXEIE		BIT(4)
#define LOONGSON_SPI_IO_IER_TXEIE		BIT(5)
#define LOONGSON_SPI_IO_IER_SUSPIE		BIT(7)
#define LOONGSON_SPI_IO_IER_OVRIE		BIT(8)
#define LOONGSON_SPI_IO_IER_UDRIE		BIT(9)
#define LOONGSON_SPI_IO_IER_CRCEIE		BIT(10)
#define LOONGSON_SPI_IO_IER_MODFIE		BIT(11)
#define LOONGSON_SPI_IO_IER_EOTIE		BIT(15)

/* Bitfields of LOONGSON_SPI_IO_SR1_REG */
#define LOONGSON_SPI_IO_SR1_RXA			BIT(0)
#define LOONGSON_SPI_IO_SR1_TXA			BIT(1)
#define LOONGSON_SPI_IO_SR1_DXA			BIT(2)
#define LOONGSON_SPI_IO_SR1_RXE			BIT(4)
#define LOONGSON_SPI_IO_SR1_TXE			BIT(5)
#define LOONGSON_SPI_IO_SR1_SUSP		BIT(7)
#define LOONGSON_SPI_IO_SR1_OVR			BIT(8)
#define LOONGSON_SPI_IO_SR1_UDR			BIT(9)
#define LOONGSON_SPI_IO_SR1_CRCE		BIT(10)
#define LOONGSON_SPI_IO_SR1_MODF		BIT(11)
#define LOONGSON_SPI_IO_SR1_EOT			BIT(15)

/* Bitfields of LOONGSON_SPI_IO_SR2_REG */
#define LOONGSON_SPI_IO_SR2_RXFLV		GENMASK(2, 0)
#define LOONGSON_SPI_IO_SR2_TXFLV		GENMASK(10, 8)

/* Bitfields of LOONGSON_SPI_IO_CFG1_REG */
#define LOONGSON_SPI_IO_CFG1_CPOL		BIT(0)
#define LOONGSON_SPI_IO_CFG1_CPHA		BIT(1)
#define LOONGSON_SPI_IO_CFG1_LSBFRST		BIT(7)
#define LOONGSON_SPI_IO_CFG1_DSIZE		GENMASK(12, 8)

/* Bitfields of LOONGSON_SPI_IO_CFG2_REG */
#define LOONGSON_SPI_IO_CFG2_BRDEC		GENMASK(7, 2)
#define LOONGSON_SPI_IO_CFG2_BRINT		GENMASK(15, 8)

/* Bitfields of LOONGSON_SPI_IO_CFG3_REG */
#define LOONGSON_SPI_IO_CFG3_MSTR		BIT(0)
#define LOONGSON_SPI_IO_CFG3_DIOSWP		BIT(1)
#define LOONGSON_SPI_IO_CFG3_DIE		BIT(2)
#define LOONGSON_SPI_IO_CFG3_DOE		BIT(3)
#define LOONGSON_SPI_IO_CFG3_SSMODE		GENMASK(9, 8)

/* Bitfields of LOONGSON_SPI_IO_CRC1_REG */
#define LOONGSON_SPI_IO_CRC1_CRCEN		BIT(0)
#define LOONGSON_SPI_IO_CRC1_CRCPOLY		GENMASK(31, 1)

/* Bitfields of LOONGSON_SPI_IO_CRC2_REG */
#define LOONGSON_SPI_IO_CRC2_RCRCINI		BIT(0)
#define LOONGSON_SPI_IO_CRC2_TCRCINI		BIT(1)
#define LOONGSON_SPI_IO_CRC2_CRCSIZE		GENMASK(12, 8)

/**
 * enum xfer_mode - Loongson SPI transfer mode
 * @XFER_IDLE: no transfer is in progress
 * @XFER_INTERRUPT: interrupt-driven batch transfer
 * @XFER_DMA: DMA-driven transfer
 */
enum xfer_mode {
	XFER_IDLE,
	XFER_INTERRUPT,
	XFER_DMA,
};

/**
 * struct loongson_spi_priv - private data for the Loongson SPI controller
 * @controller: SPI controller instance
 * @dev: device backing the controller
 * @base: MMIO base address of the SPI controller
 * @regmap: register map (currently unused)
 * @hz: current SPI bus clock in Hz
 * @clk_rate: parent clock rate in Hz
 * @mode: current SPI mode bits
 * @irq: interrupt number
 * @tx_buf: current transmit buffer for interrupt mode
 * @rx_buf: current receive buffer for interrupt mode
 * @remaining: bytes left in the current interrupt transfer
 * @xfer_ret: result of the current transfer
 * @batch_size: number of bytes in the current interrupt batch
 * @xfer_completion: completion signaled when the current transfer finishes
 * @dma_tx: DMA channel for TX
 * @dma_rx: DMA channel for RX
 * @use_dma: true if both DMA channels are available
 * @phys_base: physical base address used for DMA slave configuration
 * @xfer_mode: current transfer mode
 */
struct loongson_spi_priv {
	struct spi_controller   *controller;
	struct device		*dev;
	void __iomem		*base;
	struct regmap           *regmap;
	u32                     hz;
	u64                     clk_rate;
	u8                      mode;

	int			irq;
	const u8		*tx_buf;
	u8			*rx_buf;
	u32			remaining;
	int			xfer_ret;
	u32			batch_size;
	struct completion	xfer_completion;

	struct dma_chan		*dma_tx;
	struct dma_chan		*dma_rx;
	bool			use_dma;
	phys_addr_t		phys_base;
	enum xfer_mode		xfer_mode;
};

/* Module parameter: DMA threshold (bytes), adjustable at runtime */
static unsigned int dma_threshold = 64;
module_param(dma_threshold, uint, 0644);
MODULE_PARM_DESC(dma_threshold, "Minimum transfer length to use DMA (default 64)");

static inline void loongson_spi_writel(struct loongson_spi_priv *spi, unsigned char reg, u32 data)
{
	writel(data, spi->base + reg);
}

static inline u32 loongson_spi_readl(struct loongson_spi_priv *spi, unsigned char reg)
{
	return readl(spi->base + reg);
}

static inline void loongson_spi_set_bits(struct loongson_spi_priv *spi, u32 offset, u32 bits)
{
	writel(readl(spi->base + offset) | bits, spi->base + offset);
}

static inline void loongson_spi_clr_bits(struct loongson_spi_priv *spi, u32 offset, u32 bits)
{
	writel(readl(spi->base + offset) & ~bits, spi->base + offset);
}

static void loongson_spi_io_set_cs(struct spi_device *spi, bool high)
{
	struct loongson_spi_priv *priv = spi_controller_get_devdata(spi->controller);

	if (high)
		loongson_spi_set_bits(priv, LOONGSON_SPI_IO_CR1_REG, LOONGSON_SPI_IO_CR1_SSREV);
	else
		loongson_spi_clr_bits(priv, LOONGSON_SPI_IO_CR1_REG, LOONGSON_SPI_IO_CR1_SSREV);
}

static int loongson_spi_io_set_clk(struct loongson_spi_priv *priv, unsigned int hz)
{
	u32 val = clamp_val(DIV_ROUND_UP_ULL(priv->clk_rate, hz), 2, 255);

	loongson_spi_writel(priv, LOONGSON_SPI_IO_CFG2_REG,
			    FIELD_PREP(LOONGSON_SPI_IO_CFG2_BRINT, val));
	priv->hz = hz;

	return 0;
}

static void loongson_spi_io_set_mode(struct loongson_spi_priv *priv, u32 mode)
{
	u32 val = 0;

	if (mode & SPI_CPOL)
		val |= LOONGSON_SPI_IO_CFG1_CPOL;

	if (mode & SPI_CPHA)
		val |= LOONGSON_SPI_IO_CFG1_CPHA;

	if (mode & SPI_LSB_FIRST)
		val |= LOONGSON_SPI_IO_CFG1_LSBFRST;

	val |= FIELD_PREP(LOONGSON_SPI_IO_CFG1_DSIZE, 7);
	loongson_spi_writel(priv, LOONGSON_SPI_IO_CFG1_REG, val);

	priv->mode = mode;
}

static int loongson_spi_io_update_state(struct spi_device *spi, struct spi_transfer *t)
{
	struct loongson_spi_priv *priv = spi_controller_get_devdata(spi->controller);

	if (t && priv->hz != t->speed_hz)
		loongson_spi_io_set_clk(priv, t->speed_hz);

	if ((spi->mode ^ priv->mode) & SPI_MODE_X_MASK)
		loongson_spi_io_set_mode(priv, spi->mode);

	return 0;
}

static int loongson_spi_io_setup(struct spi_device *spi)
{
	struct loongson_spi_priv *priv = spi_controller_get_devdata(spi->controller);

	if (spi->bits_per_word % 8)
		return -EINVAL;

	if (spi_get_chipselect(spi, 0) >= spi->controller->num_chipselect)
		return -EINVAL;

	priv->hz = 0;

	return 0;
}

/*
 * DMA eligibility callback.
 *
 * The SPI core calls this before mapping the transfer buffers to SG tables.
 * We allow DMA whenever there is at least one buffer direction and the length
 * exceeds the threshold. TX-only and RX-only transfers are also eligible:
 * the missing side gets a dummy buffer allocated and mapped inside
 * loongson_spi_dma_transfer(), because the Loongson controller always drives
 * both directions during a DMA transfer.
 */
static bool loongson_spi_can_dma(struct spi_controller *ctrl,
				 struct spi_device *spi,
				 struct spi_transfer *xfer)
{
	struct loongson_spi_priv *priv = spi_controller_get_devdata(ctrl);

	if (!priv->use_dma)
		return false;

	if (xfer->len < dma_threshold)
		return false;

	if (!xfer->tx_buf && !xfer->rx_buf)
		return false;

	return true;
}

/* Interrupt mode: batch transfer (up to 4 bytes per EOT) */
static int loongson_spi_io_start_batch(struct loongson_spi_priv *priv)
{
	u32 batch_size, i;
	u8 tx_data;

	if (priv->remaining == 0)
		return 0;

	batch_size = min_t(u32, priv->remaining, 4);
	priv->batch_size = batch_size;

	loongson_spi_writel(priv, LOONGSON_SPI_IO_CR3_REG, batch_size - 1);

	for (i = 0; i < batch_size; i++) {
		tx_data = 0;
		if (priv->tx_buf)
			tx_data = *priv->tx_buf++;
		writeb(tx_data, priv->base + LOONGSON_SPI_IO_DR_REG);
	}

	loongson_spi_writel(priv, LOONGSON_SPI_IO_SR1_REG, LOONGSON_SPI_IO_SR1_EOT);
	loongson_spi_set_bits(priv, LOONGSON_SPI_IO_CR1_REG, LOONGSON_SPI_IO_CR1_CSTART);
	loongson_spi_clr_bits(priv, LOONGSON_SPI_IO_CR1_REG, LOONGSON_SPI_IO_CR1_AUTOSUS);

	return batch_size;
}

/*
 * Prepare TX side of a DMA transfer.
 *
 * If xfer->tx_buf exists, SPI core has already mapped xfer->tx_sg for us.
 * Otherwise, we allocate a dummy TX buffer and map it manually so the
 * hardware has something to shift out while it clocks in RX.
 */
static int loongson_spi_dma_prep_tx(struct loongson_spi_priv *priv,
				    struct spi_transfer *xfer,
				    struct scatterlist *dummy_sg,
				    void **dummy_buf,
				    struct scatterlist **sg_out,
				    int *nents_out)
{
	struct device *dev = priv->dev;
	int nents;

	*dummy_buf = NULL;

	if (xfer->tx_buf) {
		*sg_out = xfer->tx_sg.sgl;
		*nents_out = xfer->tx_sg.nents;
		return 0;
	}

	*dummy_buf = kzalloc(xfer->len, GFP_KERNEL);
	if (!*dummy_buf)
		return -ENOMEM;

	sg_init_one(dummy_sg, *dummy_buf, xfer->len);
	nents = dma_map_sg(dev, dummy_sg, 1, DMA_TO_DEVICE);
	if (nents <= 0) {
		kfree(*dummy_buf);
		*dummy_buf = NULL;
		return -EIO;
	}

	*sg_out = dummy_sg;
	*nents_out = nents;
	return 0;
}

/*
 * Prepare RX side of a DMA transfer.
 *
 * Same logic as TX: use SPI core's mapping if rx_buf is present, otherwise
 * allocate a dummy RX buffer and map it manually.
 */
static int loongson_spi_dma_prep_rx(struct loongson_spi_priv *priv,
				    struct spi_transfer *xfer,
				    struct scatterlist *dummy_sg,
				    void **dummy_buf,
				    struct scatterlist **sg_out,
				    int *nents_out)
{
	struct device *dev = priv->dev;
	int nents;

	*dummy_buf = NULL;

	if (xfer->rx_buf) {
		*sg_out = xfer->rx_sg.sgl;
		*nents_out = xfer->rx_sg.nents;
		return 0;
	}

	*dummy_buf = kzalloc(xfer->len, GFP_KERNEL);
	if (!*dummy_buf)
		return -ENOMEM;

	sg_init_one(dummy_sg, *dummy_buf, xfer->len);
	nents = dma_map_sg(dev, dummy_sg, 1, DMA_FROM_DEVICE);
	if (nents <= 0) {
		kfree(*dummy_buf);
		*dummy_buf = NULL;
		return -EIO;
	}

	*sg_out = dummy_sg;
	*nents_out = nents;
	return 0;
}

static void loongson_spi_dma_cleanup_tx(struct loongson_spi_priv *priv,
					struct scatterlist *dummy_sg,
					void *dummy_buf)
{
	if (!dummy_buf)
		return;

	dma_unmap_sg(priv->dev, dummy_sg, 1, DMA_TO_DEVICE);
	kfree(dummy_buf);
}

static void loongson_spi_dma_cleanup_rx(struct loongson_spi_priv *priv,
					struct scatterlist *dummy_sg,
					void *dummy_buf)
{
	if (!dummy_buf)
		return;

	dma_unmap_sg(priv->dev, dummy_sg, 1, DMA_FROM_DEVICE);
	kfree(dummy_buf);
}

/*
 * Configure the DMA slave channels and submit the TX/RX SG descriptors.
 * The SPI core has already mapped the SG tables for real buffers.
 */
static int loongson_spi_dma_config_and_submit(struct loongson_spi_priv *priv,
					      struct scatterlist *tx_sg, int tx_nents,
					      struct scatterlist *rx_sg, int rx_nents)
{
	struct dma_async_tx_descriptor *tx_desc, *rx_desc;
	struct dma_slave_config tx_cfg = {0}, rx_cfg = {0};
	struct device *dev = priv->dev;
	int ret;

	tx_cfg.direction = DMA_MEM_TO_DEV;
	tx_cfg.dst_addr = priv->phys_base + LOONGSON_SPI_IO_DR_REG;
	tx_cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	tx_cfg.dst_maxburst = 1;

	ret = dmaengine_slave_config(priv->dma_tx, &tx_cfg);
	if (ret) {
		dev_err(dev, "TX DMA config failed: %d\n", ret);
		return ret;
	}

	tx_desc = dmaengine_prep_slave_sg(priv->dma_tx, tx_sg, tx_nents,
					  DMA_MEM_TO_DEV,
					  DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!tx_desc) {
		dev_err(dev, "TX DMA prep_slave_sg failed\n");
		return -EIO;
	}

	rx_cfg.direction = DMA_DEV_TO_MEM;
	rx_cfg.src_addr = priv->phys_base + LOONGSON_SPI_IO_DR_REG;
	rx_cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_1_BYTE;
	rx_cfg.src_maxburst = 1;

	ret = dmaengine_slave_config(priv->dma_rx, &rx_cfg);
	if (ret) {
		dev_err(dev, "RX DMA config failed: %d\n", ret);
		return ret;
	}

	rx_desc = dmaengine_prep_slave_sg(priv->dma_rx, rx_sg, rx_nents,
					  DMA_DEV_TO_MEM,
					  DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!rx_desc) {
		dev_err(dev, "RX DMA prep_slave_sg failed\n");
		return -EIO;
	}

	if (dma_submit_error(dmaengine_submit(tx_desc))) {
		dev_err(dev, "TX DMA submit failed\n");
		return -EIO;
	}
	if (dma_submit_error(dmaengine_submit(rx_desc))) {
		dev_err(dev, "RX DMA submit failed\n");
		return -EIO;
	}

	return 0;
}

/*
 * Wait for the EOT interrupt that terminates a DMA transfer. On timeout
 * terminate both DMA channels so the controller is left idle.
 */
static int loongson_spi_dma_wait_completion(struct loongson_spi_priv *priv,
					    struct spi_transfer *xfer)
{
	unsigned long timeout = msecs_to_jiffies(max(1000U, xfer->len * 100));
	int ret;

	priv->xfer_mode = XFER_DMA;
	ret = wait_for_completion_timeout(&priv->xfer_completion, timeout);
	priv->xfer_mode = XFER_IDLE;

	if (ret == 0) {
		dev_err(priv->dev, "DMA transfer timeout (no EOT)\n");
		dmaengine_terminate_sync(priv->dma_tx);
		dmaengine_terminate_sync(priv->dma_rx);
		return -ETIMEDOUT;
	}

	/* 0 on success, negative on error */
	return priv->xfer_ret;
}

/*
 * Execute one transfer with DMA. If a direction has no real buffer, a
 * dummy buffer is allocated because the controller clocks both directions
 * while DMA is enabled.
 */
static int loongson_spi_dma_transfer(struct loongson_spi_priv *priv, struct spi_transfer *xfer)
{
	struct scatterlist dummy_tx_sg, dummy_rx_sg;
	void *dummy_tx = NULL, *dummy_rx = NULL;
	struct scatterlist *tx_sg, *rx_sg;
	int tx_nents, rx_nents;
	int ret;

	ret = loongson_spi_dma_prep_tx(priv, xfer, &dummy_tx_sg, &dummy_tx,
				       &tx_sg, &tx_nents);
	if (ret)
		return ret;

	ret = loongson_spi_dma_prep_rx(priv, xfer, &dummy_rx_sg, &dummy_rx,
				       &rx_sg, &rx_nents);
	if (ret)
		goto cleanup_tx;

	ret = loongson_spi_dma_config_and_submit(priv, tx_sg, tx_nents,
						 rx_sg, rx_nents);
	if (ret)
		goto cleanup_both;

	/* Enable SPI DMA and start SPI engine */
	loongson_spi_set_bits(priv, LOONGSON_SPI_IO_CR2_REG,
			      LOONGSON_SPI_IO_CR2_TXDMAEN | LOONGSON_SPI_IO_CR2_RXDMAEN);
	loongson_spi_writel(priv, LOONGSON_SPI_IO_CR3_REG, xfer->len - 1);
	loongson_spi_writel(priv, LOONGSON_SPI_IO_SR1_REG, LOONGSON_SPI_IO_SR1_EOT);
	loongson_spi_set_bits(priv, LOONGSON_SPI_IO_CR1_REG, LOONGSON_SPI_IO_CR1_CSTART);
	loongson_spi_clr_bits(priv, LOONGSON_SPI_IO_CR1_REG, LOONGSON_SPI_IO_CR1_AUTOSUS);

	/* Start DMA */
	dma_async_issue_pending(priv->dma_tx);
	dma_async_issue_pending(priv->dma_rx);

	/* Wait for completion (EOT interrupt) */
	ret = loongson_spi_dma_wait_completion(priv, xfer);

	/* Disable SPI DMA */
	loongson_spi_clr_bits(priv, LOONGSON_SPI_IO_CR2_REG,
			      LOONGSON_SPI_IO_CR2_TXDMAEN | LOONGSON_SPI_IO_CR2_RXDMAEN);

cleanup_both:
	loongson_spi_dma_cleanup_rx(priv, &dummy_rx_sg, dummy_rx);
cleanup_tx:
	loongson_spi_dma_cleanup_tx(priv, &dummy_tx_sg, dummy_tx);
	return ret;
}

/*
 * Handle EOT and error interrupts. In interrupt mode the next batch is
 * queued until all bytes are done; in DMA mode EOT completes the transfer.
 */
static irqreturn_t loongson_spi_io_irq_handler(int irq, void *dev_id)
{
	struct loongson_spi_priv *priv = dev_id;
	u32 sr1, ier;
	u8 rx_data;
	u32 i;

	sr1 = loongson_spi_readl(priv, LOONGSON_SPI_IO_SR1_REG);

	if (sr1 & LOONGSON_SPI_IO_SR1_EOT) {
		loongson_spi_writel(priv, LOONGSON_SPI_IO_SR1_REG, LOONGSON_SPI_IO_SR1_EOT);

		if (priv->xfer_mode == XFER_DMA) {
			dmaengine_terminate_async(priv->dma_tx);
			dmaengine_terminate_async(priv->dma_rx);
			loongson_spi_clr_bits(priv, LOONGSON_SPI_IO_CR2_REG,
					      LOONGSON_SPI_IO_CR2_TXDMAEN |
					      LOONGSON_SPI_IO_CR2_RXDMAEN);
			priv->xfer_ret = 0;
			complete(&priv->xfer_completion);
		} else if (priv->xfer_mode == XFER_INTERRUPT) {
			for (i = 0; i < priv->batch_size; i++) {
				rx_data = readb(priv->base + LOONGSON_SPI_IO_DR_REG);
				if (priv->rx_buf)
					*priv->rx_buf++ = rx_data;
			}
			priv->remaining -= priv->batch_size;

			if (priv->remaining == 0) {
				ier = loongson_spi_readl(priv, LOONGSON_SPI_IO_IER_REG);
				ier &= ~LOONGSON_SPI_IO_IER_EOTIE;
				loongson_spi_writel(priv, LOONGSON_SPI_IO_IER_REG, ier);
				priv->xfer_ret = 0;
				complete(&priv->xfer_completion);
			} else {
				loongson_spi_io_start_batch(priv);
			}
		} else {
			dev_warn(&priv->controller->dev, "EOT with no active transfer\n");
		}
	} else {
		/* Error conditions */
		if (sr1 & (LOONGSON_SPI_IO_SR1_OVR | LOONGSON_SPI_IO_SR1_UDR |
		    LOONGSON_SPI_IO_SR1_MODF | LOONGSON_SPI_IO_SR1_CRCE)) {
			dev_err(&priv->controller->dev, "SPI error: SR1=0x%x\n", sr1);
			loongson_spi_writel(priv, LOONGSON_SPI_IO_IER_REG, 0);
			priv->xfer_ret = -EIO;
			if (priv->xfer_mode == XFER_DMA) {
				dmaengine_terminate_async(priv->dma_tx);
				dmaengine_terminate_async(priv->dma_rx);
				priv->xfer_mode = XFER_IDLE;
			}
			complete(&priv->xfer_completion);
		}
	}

	return IRQ_HANDLED;
}

static int loongson_spi_io_transfer_one(struct spi_controller *ctrl, struct spi_device *spi,
					struct spi_transfer *xfer)
{
	struct loongson_spi_priv *priv = spi_controller_get_devdata(spi->controller);
	unsigned long timeout;
	int ret;

	loongson_spi_io_update_state(spi, xfer);

	if (xfer->len == 0)
		return 0;

	/*
	 * Try DMA if eligible. SPI core has already mapped xfer->tx_sg /
	 * xfer->rx_sg for the directions that have real buffers.
	 */
	if (ctrl->can_dma && ctrl->can_dma(ctrl, spi, xfer)) {
		loongson_spi_set_bits(priv, LOONGSON_SPI_IO_IER_REG, LOONGSON_SPI_IO_IER_EOTIE);
		reinit_completion(&priv->xfer_completion);
		ret = loongson_spi_dma_transfer(priv, xfer);
		if (ret == 0)
			return 0;
		dev_dbg(&spi->dev, "DMA failed (%d), falling back to interrupt\n", ret);
	}

	/* Interrupt mode */
	priv->tx_buf = xfer->tx_buf;
	priv->rx_buf = xfer->rx_buf;
	priv->remaining = xfer->len;
	priv->xfer_ret = 0;

	reinit_completion(&priv->xfer_completion);
	priv->xfer_mode = XFER_INTERRUPT;

	loongson_spi_clr_bits(priv, LOONGSON_SPI_IO_CR1_REG, LOONGSON_SPI_IO_CR1_AUTOSUS);
	loongson_spi_set_bits(priv, LOONGSON_SPI_IO_IER_REG, LOONGSON_SPI_IO_IER_EOTIE);

	loongson_spi_io_start_batch(priv);

	timeout = msecs_to_jiffies(max(1000U, xfer->len * 100));
	ret = wait_for_completion_timeout(&priv->xfer_completion, timeout);
	priv->xfer_mode = XFER_IDLE;

	if (ret == 0) {
		loongson_spi_writel(priv, LOONGSON_SPI_IO_IER_REG, 0);
		dev_err(&spi->dev, "Interrupt transfer timeout\n");
		return -ETIMEDOUT;
	}

	return priv->xfer_ret;
}

/*
 * Initialize the controller as SPI master with data input/output enabled
 * and the SPI engine enabled.
 */
static void loongson_spi_io_reginit(struct loongson_spi_priv *priv)
{
	loongson_spi_writel(priv, LOONGSON_SPI_IO_CFG3_REG,
			    LOONGSON_SPI_IO_CFG3_MSTR | LOONGSON_SPI_IO_CFG3_DIE |
			    LOONGSON_SPI_IO_CFG3_DOE);

	loongson_spi_writel(priv, LOONGSON_SPI_IO_CR1_REG, LOONGSON_SPI_IO_CR1_SPE);
}

/*
 * Probe the Loongson SPI controller, map registers, acquire optional DMA
 * channels and register the SPI host.
 */
static int loongson_spi_io_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct loongson_spi_priv *priv;
	struct spi_controller *host;
	struct resource *res;
	struct clk *clk;
	int ret;

	host = devm_spi_alloc_host(dev, sizeof(*priv));
	if (!host)
		return -ENOMEM;

	host->mode_bits = SPI_MODE_X_MASK | SPI_LSB_FIRST;
	host->setup = loongson_spi_io_setup;
	host->transfer_one = loongson_spi_io_transfer_one;
	host->set_cs = loongson_spi_io_set_cs;
	host->num_chipselect = 1;
	dev_set_drvdata(dev, host);

	priv = spi_controller_get_devdata(host);

	priv->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->controller = host;
	priv->phys_base = res->start;
	priv->dev = dev;
	priv->xfer_mode = XFER_IDLE;
	priv->use_dma = false;
	priv->batch_size = 0;

	init_completion(&priv->xfer_completion);

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "unable to get clock\n");

	priv->clk_rate = clk_get_rate(clk);

	loongson_spi_io_reginit(priv);

	priv->dma_tx = dma_request_slave_channel(dev, "tx");
	priv->dma_rx = dma_request_slave_channel(dev, "rx");
	if (priv->dma_tx && priv->dma_rx) {
		priv->use_dma = true;
		host->dma_tx = priv->dma_tx;
		host->dma_rx = priv->dma_rx;
		host->can_dma = loongson_spi_can_dma;
		dev_info(dev, "DMA channels acquired\n");
	} else {
		if (priv->dma_tx)
			dma_release_channel(priv->dma_tx);
		if (priv->dma_rx)
			dma_release_channel(priv->dma_rx);
		priv->dma_tx = NULL;
		priv->dma_rx = NULL;
		dev_info(dev, "DMA unavailable, using interrupt mode only\n");
	}

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0)
		return dev_err_probe(dev, priv->irq, "no IRQ resource\n");

	ret = devm_request_irq(dev, priv->irq, loongson_spi_io_irq_handler,
			       IRQF_SHARED, dev_name(dev), priv);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	loongson_spi_writel(priv, LOONGSON_SPI_IO_IER_REG, 0);

	return devm_spi_register_controller(dev, host);
}

static const struct of_device_id loongson_spi_io_id_table[] = {
	{ .compatible = "loongson,ls2k0300-spi-io" },
	{ }
};
MODULE_DEVICE_TABLE(of, loongson_spi_io_id_table);

static struct platform_driver loongson_spi_io_plat_driver = {
	.probe = loongson_spi_io_probe,
	.driver	= {
		.name	= "loongson-spi-io",
		.bus = &platform_bus_type,
		.of_match_table = loongson_spi_io_id_table,
	},
};
module_platform_driver(loongson_spi_io_plat_driver);

MODULE_DESCRIPTION("Loongson spi io platform driver");
MODULE_LICENSE("GPL");
