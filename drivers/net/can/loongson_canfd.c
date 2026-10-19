// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Loongson 2K3000 CAN-FD controller driver
 *
 * Copyright (C) 2024-2026 Loongson Technology Corporation Limited
 *
 * Register definitions are taken from the LS2K3000/3B6000M user manual,
 * chapter 16 (CAN). The driver architecture follows the community
 * ctucanfd driver: a single NAPI poll routine, multi-buffer TX rotation
 * over eight hardware TX buffers, and error-frame generation driven by
 * FCSI/EWLI/ALI/BEI interrupts.
 */

#include <linux/acpi.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

/* -------------------------------------------------------------------------
 * Register offsets (manual Table 16-1)
 * ---------------------------------------------------------------------- */
#define LSCANFD_CR_ID		0x00
#define LSCANFD_CR_MODE		0x04
#define LSCANFD_CR_SET		0x08
#define LSCANFD_CR_STAT		0x0c
#define LSCANFD_CR_CMD		0x10
#define LSCANFD_INT_STAT	0x14
#define LSCANFD_INT_ENA		0x18
#define LSCANFD_INT_MASK	0x1c
#define LSCANFD_BTR_NORM	0x20
#define LSCANFD_BTR_FD		0x24
#define LSCANFD_ERL		0x28
#define LSCANFD_FSTAT		0x2c
#define LSCANFD_ERC		0x30
#define LSCANFD_ERR_CAPT	0x3c
#define LSCANFD_ALC		0x44
#define LSCANFD_SSP_CFG		0x4c
#define LSCANFD_RX_FR_CNT	0x50
#define LSCANFD_TS		0x5c
#define LSCANFD_FRC_DIV		0x64
#define LSCANFD_RX_MEM_INFO	0x8c
#define LSCANFD_RX_PRT		0x90
#define LSCANFD_RX_STAT		0x94
#define LSCANFD_RX_DATA		0x98
#define LSCANFD_TX_STAT		0x9c
#define LSCANFD_TX_CMD		0xa0
#define LSCANFD_TX_SEL		0xa4
#define LSCANFD_TX_DATA		0xb0	/* 0xb0 .. 0xf4, one TX buffer */

/* -------------------------------------------------------------------------
 * CR_MODE (Table 16-3)
 * ---------------------------------------------------------------------- */
#define LSCANFD_MODE_RST	BIT(0)
#define LSCANFD_MODE_BMM	BIT(1)
#define LSCANFD_MODE_FDE	BIT(4)
#define LSCANFD_MODE_ACF	BIT(7)
#define LSCANFD_MODE_TSTM	BIT(8)
#define LSCANFD_MODE_RXBAM	BIT(9)
#define LSCANFD_MODE_ITSM	BIT(10)
#define LSCANFD_MODE_RTSOP	BIT(12)
#define LSCANFD_MODE_BUFM	BIT(13)

/* -------------------------------------------------------------------------
 * CR_SET (Table 16-4)
 * ---------------------------------------------------------------------- */
#define LSCANFD_SET_RTRLE	BIT(0)
#define LSCANFD_SET_RTRTH	GENMASK(4, 1)
#define LSCANFD_SET_ILBP	BIT(5)
#define LSCANFD_SET_ENA		BIT(6)
#define LSCANFD_SET_NISOFD	BIT(7)

/* -------------------------------------------------------------------------
 * CR_STAT (Table 16-5)
 * ---------------------------------------------------------------------- */
#define LSCANFD_STAT_RXNE	BIT(0)
#define LSCANFD_STAT_DOR	BIT(1)
#define LSCANFD_STAT_RXS	BIT(4)
#define LSCANFD_STAT_TXS	BIT(5)
#define LSCANFD_STAT_EWL	BIT(6)
#define LSCANFD_STAT_IDLE	BIT(7)

/* -------------------------------------------------------------------------
 * CR_CMD (Table 16-6)
 * ---------------------------------------------------------------------- */
#define LSCANFD_CMD_RXRPMV	BIT(1)
#define LSCANFD_CMD_RRB		BIT(2)
#define LSCANFD_CMD_CDO		BIT(3)
#define LSCANFD_CMD_ERCRST	BIT(4)
#define LSCANFD_CMD_RXFCRST	BIT(5)
#define LSCANFD_CMD_TXFCRST	BIT(6)
#define LSCANFD_CMD_CPEXS	BIT(7)

/* -------------------------------------------------------------------------
 * INT_STAT / INT_ENA / INT_MASK (Tables 16-7, 16-8, 16-9)
 *
 * Both INT_ENA and INT_MASK follow the same layout:
 *   bits[12:0]  set enable / set mask (write 1)
 *   bits[28:16] clear enable / clear mask (write 1)
 * ---------------------------------------------------------------------- */
#define LSCANFD_INT_RXI		BIT(0)
#define LSCANFD_INT_TXI		BIT(1)
#define LSCANFD_INT_EWLI	BIT(2)
#define LSCANFD_INT_DOI		BIT(3)
#define LSCANFD_INT_FCSI	BIT(4)
#define LSCANFD_INT_ALI		BIT(5)
#define LSCANFD_INT_BEI		BIT(6)
#define LSCANFD_INT_RXFI	BIT(7)
#define LSCANFD_INT_BSI		BIT(8)
#define LSCANFD_INT_RBNEI	BIT(9)
#define LSCANFD_INT_TXBHCI	BIT(10)
#define LSCANFD_INT_OFI		BIT(11)
#define LSCANFD_INT_DMADI	BIT(12)

#define LSCANFD_INT_ERRORI	(LSCANFD_INT_EWLI | LSCANFD_INT_FCSI | \
				 LSCANFD_INT_ALI  | LSCANFD_INT_BEI)

/* -------------------------------------------------------------------------
 * BTR_NORM / BTR_FD (Tables 16-10, 16-11)
 * ---------------------------------------------------------------------- */
#define LSCANFD_BTR_PROP	GENMASK(6, 0)
#define LSCANFD_BTR_PH1		GENMASK(12, 7)
#define LSCANFD_BTR_PH2		GENMASK(18, 13)
#define LSCANFD_BTR_BRP		GENMASK(22, 19)
#define LSCANFD_BTR_SJW		GENMASK(31, 27)

#define LSCANFD_BTR_FD_PROP	GENMASK(6, 0)
#define LSCANFD_BTR_FD_PH1	GENMASK(12, 7)
#define LSCANFD_BTR_FD_PH2	GENMASK(18, 13)
#define LSCANFD_BTR_FD_BRP	GENMASK(26, 19)
#define LSCANFD_BTR_FD_SJW	GENMASK(31, 27)

/* -------------------------------------------------------------------------
 * FSTAT (Table 16-13), ERC (Table 16-14)
 * ---------------------------------------------------------------------- */
#define LSCANFD_FSTAT_ERA	BIT(0)
#define LSCANFD_FSTAT_ERP	BIT(1)
#define LSCANFD_FSTAT_BOF	BIT(2)
#define LSCANFD_FSTAT_MASK	GENMASK(2, 0)

#define LSCANFD_ERC_TEC		GENMASK(8, 0)
#define LSCANFD_ERC_REC		GENMASK(24, 16)

/* -------------------------------------------------------------------------
 * RX_STAT (Table 16-39)
 * ---------------------------------------------------------------------- */
#define LSCANFD_RX_STAT_RXE	BIT(0)
#define LSCANFD_RX_STAT_RXF	BIT(1)
#define LSCANFD_RX_STAT_RXFRC	GENMASK(14, 4)

/* -------------------------------------------------------------------------
 * TX_STAT (Table 16-41)
 * ---------------------------------------------------------------------- */
#define LSCANFD_TX_STAT_BRP	GENMASK(7, 0)
#define LSCANFD_TX_STAT_BS	GENMASK(31, 16)

/* -------------------------------------------------------------------------
 * TX_CMD (Table 16-42), TX_SEL (Table 16-43)
 * ---------------------------------------------------------------------- */
#define LSCANFD_TX_CMD_BAR	GENMASK(7, 0)
#define LSCANFD_TX_CMD_BCR	GENMASK(15, 8)
#define LSCANFD_TX_CMD_BSC	GENMASK(23, 16)

#define LSCANFD_TX_SEL_BUF_SEL	GENMASK(3, 0)
#define LSCANFD_TX_SEL_BUF_CNT	GENMASK(7, 4)

/* -------------------------------------------------------------------------
 * Frame format (Section 16.4)
 * ---------------------------------------------------------------------- */
#define LSCANFD_META0_ID_EXT	GENMASK(28, 0)
#define LSCANFD_META0_ID_BASE	GENMASK(28, 18)
#define LSCANFD_META0_RTR	BIT(29)
#define LSCANFD_META0_XDT	BIT(30)
#define LSCANFD_META0_ESI	BIT(31)

#define LSCANFD_META1_TS	GENMASK(15, 0)
#define LSCANFD_META1_DLC	GENMASK(19, 16)
#define LSCANFD_META1_BRS	BIT(20)
#define LSCANFD_META1_FDF	BIT(21)
#define LSCANFD_META1_RWCNT	GENMASK(28, 24)

#define LSCANFD_META0_OFF	0x0
#define LSCANFD_META1_OFF	0x4
#define LSCANFD_DATA_OFF	0x8

/* -------------------------------------------------------------------------
 * Driver constants
 * ---------------------------------------------------------------------- */
#define LSCANFD_TXBUF_NUM	8
#define LSCANFD_DRV_NAME	"loongson_canfd"
#define LSCANFD_DEVICE_ID	0xBABE

enum lscanfd_txtb_status {
	LSCANFD_TXT_IDLE	= 0x0,
	LSCANFD_TXT_VALID	= 0x1,
	LSCANFD_TXT_FAIL	= 0x2,
	LSCANFD_TXT_CANCEL	= 0x3,
};

enum lscanfd_txtb_cmd {
	LSCANFD_TXT_CMD_ADD	= 0x01,
	LSCANFD_TXT_CMD_CANCEL	= 0x02,
	LSCANFD_TXT_CMD_SR_CLEAN = 0x03,
};

struct lscanfd_priv {
	struct can_priv		can;		/* must be first */
	struct napi_struct	napi;
	void __iomem		*base;
	int			irq;
	u32			clk_rate;

	/* TX buffer rotation (ctucanfd style) */
	unsigned int		txb_head;
	unsigned int		txb_tail;
	spinlock_t		tx_lock;

	/* RX statistics helper */
	u32			rx_dropped;
};

static inline u32 lscanfd_read32(struct lscanfd_priv *priv, u32 reg)
{
	return ioread32(priv->base + reg);
}

static inline void lscanfd_write32(struct lscanfd_priv *priv, u32 reg, u32 val)
{
	iowrite32(val, priv->base + reg);
}

/* -------------------------------------------------------------------------
 * Bit-timing constants
 * ---------------------------------------------------------------------- */
static const struct can_bittiming_const lscanfd_bittiming_const = {
	.name		= LSCANFD_DRV_NAME,
	.tseg1_min	= 2,
	.tseg1_max	= 190,
	.tseg2_min	= 2,
	.tseg2_max	= 63,
	.sjw_max	= 31,
	.brp_min	= 1,
	.brp_max	= 15,
	.brp_inc	= 1,
};

static const struct can_bittiming_const lscanfd_data_bittiming_const = {
	.name		= LSCANFD_DRV_NAME,
	.tseg1_min	= 2,
	.tseg1_max	= 190,
	.tseg2_min	= 2,
	.tseg2_max	= 63,
	.sjw_max	= 31,
	.brp_min	= 1,
	.brp_max	= 255,
	.brp_inc	= 1,
};

static const struct can_tdc_const lscanfd_tdc_const = {
	.tdcv_min	= 0,
	.tdcv_max	= 0,
	.tdco_min	= 1,
	.tdco_max	= 127,
	.tdcf_min	= 0,
	.tdcf_max	= 0,
};

/* -------------------------------------------------------------------------
 * Low-level helpers
 * ---------------------------------------------------------------------- */
static void lscanfd_reset(struct lscanfd_priv *priv)
{
	lscanfd_write32(priv, LSCANFD_CR_MODE, LSCANFD_MODE_RST);
	lscanfd_write32(priv, LSCANFD_CR_MODE,
			LSCANFD_MODE_RXBAM | LSCANFD_MODE_BUFM);
}

static bool lscanfd_enabled(struct lscanfd_priv *priv)
{
	return !!(lscanfd_read32(priv, LSCANFD_CR_SET) & LSCANFD_SET_ENA);
}

/*
 * The hardware PROP field is wider than PH1.  The kernel splits tseg1 in
 * half between prop_seg and phase_seg1; if phase_seg1 overflows the PH1
 * field, push the surplus into prop_seg.
 */
static void lscanfd_set_btr(struct lscanfd_priv *priv,
			    struct can_bittiming *bt, bool nominal)
{
	u32 ph1_max = nominal ? FIELD_MAX(LSCANFD_BTR_PH1)
			      : FIELD_MAX(LSCANFD_BTR_FD_PH1);
	u32 prop = bt->prop_seg;
	u32 phase1 = bt->phase_seg1;
	u32 btr;

	if (phase1 > ph1_max) {
		prop += phase1 - ph1_max;
		phase1 = ph1_max;
	}

	if (nominal) {
		btr = FIELD_PREP(LSCANFD_BTR_PROP, prop) |
		      FIELD_PREP(LSCANFD_BTR_PH1, phase1) |
		      FIELD_PREP(LSCANFD_BTR_PH2, bt->phase_seg2) |
		      FIELD_PREP(LSCANFD_BTR_BRP, bt->brp) |
		      FIELD_PREP(LSCANFD_BTR_SJW, bt->sjw);
		lscanfd_write32(priv, LSCANFD_BTR_NORM, btr);
	} else {
		btr = FIELD_PREP(LSCANFD_BTR_FD_PROP, prop) |
		      FIELD_PREP(LSCANFD_BTR_FD_PH1, phase1) |
		      FIELD_PREP(LSCANFD_BTR_FD_PH2, bt->phase_seg2) |
		      FIELD_PREP(LSCANFD_BTR_FD_BRP, bt->brp) |
		      FIELD_PREP(LSCANFD_BTR_FD_SJW, bt->sjw);
		lscanfd_write32(priv, LSCANFD_BTR_FD, btr);
	}
}

static void lscanfd_set_ssp(struct lscanfd_priv *priv)
{
	u32 cfg = 0;

	/*
	 * SSP_CFG[SSP_SRC] = 0b01 disables SSP.  The kernel only uses TDC
	 * for data rates above 1 Mbps; otherwise leave SSP disabled.
	 */
	if (can_fd_tdc_is_enabled(&priv->can))
		cfg = FIELD_PREP(GENMASK(9, 8), 0x0) |
		      FIELD_PREP(GENMASK(7, 0), priv->can.fd.tdc.tdco);
	else
		cfg = FIELD_PREP(GENMASK(9, 8), 0x1);

	lscanfd_write32(priv, LSCANFD_SSP_CFG, cfg);
}

static void lscanfd_set_bittiming(struct net_device *ndev)
{
	struct lscanfd_priv *priv = netdev_priv(ndev);

	lscanfd_set_btr(priv, &priv->can.bittiming, true);
}

static void lscanfd_set_data_bittiming(struct net_device *ndev)
{
	struct lscanfd_priv *priv = netdev_priv(ndev);

	lscanfd_set_btr(priv, &priv->can.fd.data_bittiming, false);
}

/* -------------------------------------------------------------------------
 * Interrupt mask helpers.
 *
 * INT_ENA/INT_MASK use a "write-1-set" on bits[12:0] and "write-1-clear" on
 * bits[28:16].  The helpers below always clear then set, so they are safe
 * to call from any context.
 * ---------------------------------------------------------------------- */
static void lscanfd_int_enable(struct lscanfd_priv *priv, u32 mask)
{
	lscanfd_write32(priv, LSCANFD_INT_ENA, mask);
}

static void lscanfd_int_disable(struct lscanfd_priv *priv, u32 mask)
{
	lscanfd_write32(priv, LSCANFD_INT_ENA, mask << 16);
}

static void lscanfd_int_mask(struct lscanfd_priv *priv, u32 mask)
{
	lscanfd_write32(priv, LSCANFD_INT_MASK, mask);
}

static void lscanfd_int_unmask(struct lscanfd_priv *priv, u32 mask)
{
	lscanfd_write32(priv, LSCANFD_INT_MASK, mask << 16);
}

static void lscanfd_disable_interrupts(struct lscanfd_priv *priv)
{
	lscanfd_write32(priv, LSCANFD_INT_ENA, 0x1fff << 16);
	lscanfd_write32(priv, LSCANFD_INT_MASK, 0x1fff);
}

/* -------------------------------------------------------------------------
 * Chip start / stop
 * ---------------------------------------------------------------------- */
static int lscanfd_chip_start(struct net_device *ndev)
{
	struct lscanfd_priv *priv = netdev_priv(ndev);
	u32 mode = 0, set = 0, int_ena;

	lscanfd_set_bittiming(ndev);
	lscanfd_set_data_bittiming(ndev);
	lscanfd_set_ssp(priv);

	/* CR_MODE */
	mode |= LSCANFD_MODE_ITSM | LSCANFD_MODE_RXBAM | LSCANFD_MODE_BUFM;
	if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
		mode |= LSCANFD_MODE_BMM;
	if (priv->can.ctrlmode & CAN_CTRLMODE_FD)
		mode |= LSCANFD_MODE_FDE;
	if (priv->can.ctrlmode & CAN_CTRLMODE_PRESUME_ACK)
		mode |= LSCANFD_MODE_ACF;
	lscanfd_write32(priv, LSCANFD_CR_MODE, mode);

	/* CR_SET */
	set |= LSCANFD_SET_RTRLE;
	if (priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT)
		set |= FIELD_PREP(LSCANFD_SET_RTRTH, 0xf);
	if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
		set |= LSCANFD_SET_ILBP;
	if (priv->can.ctrlmode & CAN_CTRLMODE_FD_NON_ISO)
		set |= LSCANFD_SET_NISOFD;
	lscanfd_write32(priv, LSCANFD_CR_SET, set);

	/* Interrupts */
	int_ena = LSCANFD_INT_TXBHCI | LSCANFD_INT_EWLI |
		  LSCANFD_INT_FCSI  | LSCANFD_INT_RBNEI;
	if (priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING)
		int_ena |= LSCANFD_INT_ALI | LSCANFD_INT_BEI;

	lscanfd_write32(priv, LSCANFD_INT_MASK, 0x1fff);	/* mask all */
	lscanfd_write32(priv, LSCANFD_INT_ENA, 0x1fff << 16);	/* disable all */
	lscanfd_write32(priv, LSCANFD_INT_MASK, int_ena << 16);	/* unmask used */
	lscanfd_write32(priv, LSCANFD_INT_ENA, int_ena);	/* enable used */

	priv->can.state = CAN_STATE_ERROR_ACTIVE;

	/* Enable controller last */
	lscanfd_write32(priv, LSCANFD_CR_SET, set | LSCANFD_SET_ENA);

	return 0;
}

static void lscanfd_chip_stop(struct net_device *ndev)
{
	struct lscanfd_priv *priv = netdev_priv(ndev);
	u32 set;

	lscanfd_disable_interrupts(priv);

	set = lscanfd_read32(priv, LSCANFD_CR_SET);
	set &= ~LSCANFD_SET_ENA;
	lscanfd_write32(priv, LSCANFD_CR_SET, set);

	priv->can.state = CAN_STATE_STOPPED;
}

/* -------------------------------------------------------------------------
 * Error handling
 * ---------------------------------------------------------------------- */
static void lscanfd_get_berr_counter(struct lscanfd_priv *priv,
				     struct can_berr_counter *bec)
{
	u32 erc = lscanfd_read32(priv, LSCANFD_ERC);

	bec->txerr = FIELD_GET(LSCANFD_ERC_TEC, erc);
	bec->rxerr = FIELD_GET(LSCANFD_ERC_REC, erc);
}

static enum can_state lscanfd_get_state(struct lscanfd_priv *priv)
{
	struct can_berr_counter bec;
	u32 fstat;

	fstat = lscanfd_read32(priv, LSCANFD_FSTAT);
	lscanfd_get_berr_counter(priv, &bec);

	switch (fstat & LSCANFD_FSTAT_MASK) {
	case LSCANFD_FSTAT_ERA:
		if (bec.txerr >= CAN_ERROR_WARNING_THRESHOLD ||
		    bec.rxerr >= CAN_ERROR_WARNING_THRESHOLD)
			return CAN_STATE_ERROR_WARNING;
		return CAN_STATE_ERROR_ACTIVE;
	case LSCANFD_FSTAT_ERP:
		return CAN_STATE_ERROR_PASSIVE;
	case LSCANFD_FSTAT_BOF:
		return CAN_STATE_BUS_OFF;
	default:
		return priv->can.state;
	}
}

static void lscanfd_err_interrupt(struct net_device *ndev, u32 isr)
{
	struct lscanfd_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct can_berr_counter bec;
	enum can_state new_state;
	struct can_frame *cf;
	struct sk_buff *skb;

	lscanfd_get_berr_counter(priv, &bec);
	new_state = lscanfd_get_state(priv);

	if ((isr & (LSCANFD_INT_FCSI | LSCANFD_INT_EWLI)) &&
	    new_state != priv->can.state) {
		enum can_state old_state = priv->can.state;

		priv->can.state = new_state;
		skb = alloc_can_err_skb(ndev, &cf);
		if (skb) {
			switch (new_state) {
			case CAN_STATE_BUS_OFF:
				priv->can.can_stats.bus_off++;
				cf->can_id |= CAN_ERR_BUSOFF;
				can_bus_off(ndev);
				break;
			case CAN_STATE_ERROR_PASSIVE:
				priv->can.can_stats.error_passive++;
				cf->can_id |= CAN_ERR_CRTL | CAN_ERR_CNT;
				cf->data[1] = (bec.rxerr > bec.txerr) ?
					      CAN_ERR_CRTL_RX_PASSIVE :
					      CAN_ERR_CRTL_TX_PASSIVE;
				cf->data[6] = bec.txerr;
				cf->data[7] = bec.rxerr;
				break;
			case CAN_STATE_ERROR_WARNING:
				priv->can.can_stats.error_warning++;
				cf->can_id |= CAN_ERR_CRTL | CAN_ERR_CNT;
				cf->data[1] = (bec.txerr > bec.rxerr) ?
					      CAN_ERR_CRTL_TX_WARNING :
					      CAN_ERR_CRTL_RX_WARNING;
				cf->data[6] = bec.txerr;
				cf->data[7] = bec.rxerr;
				break;
			case CAN_STATE_ERROR_ACTIVE:
				cf->can_id |= CAN_ERR_CNT;
				cf->data[1] = CAN_ERR_CRTL_ACTIVE;
				cf->data[6] = bec.txerr;
				cf->data[7] = bec.rxerr;
				break;
			default:
				break;
			}
			netif_receive_skb(skb);
		}
		netdev_info(ndev, "CAN state %s -> %s\n",
			    can_get_state_str(old_state),
			    can_get_state_str(new_state));
	}

	if (isr & LSCANFD_INT_ALI) {
		priv->can.can_stats.arbitration_lost++;
		skb = alloc_can_err_skb(ndev, &cf);
		if (skb) {
			cf->can_id |= CAN_ERR_LOSTARB;
			cf->data[0] = CAN_ERR_LOSTARB_UNSPEC;
			stats->rx_packets++;
			stats->rx_bytes += cf->len;
			netif_receive_skb(skb);
		}
	}

	if (isr & LSCANFD_INT_BEI) {
		priv->can.can_stats.bus_error++;
		stats->rx_errors++;
		skb = alloc_can_err_skb(ndev, &cf);
		if (skb) {
			cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;
			cf->data[2] = CAN_ERR_PROT_UNSPEC;
			cf->data[3] = CAN_ERR_PROT_LOC_UNSPEC;
			stats->rx_packets++;
			stats->rx_bytes += cf->len;
			netif_receive_skb(skb);
		}
	}
}

/* -------------------------------------------------------------------------
 * RX path
 * ---------------------------------------------------------------------- */
static int lscanfd_rx_one(struct net_device *ndev)
{
	struct lscanfd_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct canfd_frame *cf;
	struct sk_buff *skb;
	u32 meta0, meta1, rwcnt, dlc;
	bool is_fd;
	u32 i;

	meta0 = lscanfd_read32(priv, LSCANFD_RX_DATA);
	meta1 = lscanfd_read32(priv, LSCANFD_RX_DATA);

	rwcnt = FIELD_GET(LSCANFD_META1_RWCNT, meta1);
	if (rwcnt < 2 || rwcnt > 18) {
		/* Malformed header: drain and drop */
		for (i = 2; i < rwcnt && i < 18; i++)
			lscanfd_read32(priv, LSCANFD_RX_DATA);
		stats->rx_dropped++;
		return -EINVAL;
	}

	is_fd = !!(meta1 & LSCANFD_META1_FDF);
	dlc = FIELD_GET(LSCANFD_META1_DLC, meta1);

	if (is_fd)
		skb = alloc_canfd_skb(ndev, &cf);
	else
		skb = alloc_can_skb(ndev, (struct can_frame **)&cf);
	if (unlikely(!skb)) {
		for (i = 2; i < rwcnt; i++)
			lscanfd_read32(priv, LSCANFD_RX_DATA);
		stats->rx_dropped++;
		return -ENOMEM;
	}

	if (meta0 & LSCANFD_META0_XDT)
		cf->can_id = (meta0 & CAN_EFF_MASK) | CAN_EFF_FLAG;
	else
		cf->can_id = FIELD_GET(LSCANFD_META0_ID_BASE, meta0) &
			     CAN_SFF_MASK;

	cf->flags = 0;
	if (is_fd) {
		if (meta1 & LSCANFD_META1_BRS)
			cf->flags |= CANFD_BRS;
		if (meta0 & LSCANFD_META0_ESI)
			cf->flags |= CANFD_ESI;
	} else if (meta0 & LSCANFD_META0_RTR) {
		cf->can_id |= CAN_RTR_FLAG;
	}

	if (is_fd)
		cf->len = can_fd_dlc2len(dlc);
	else
		can_frame_set_cc_len((struct can_frame *)cf, dlc,
				     priv->can.ctrlmode);

	for (i = 2; i < rwcnt; i++) {
		u32 data = lscanfd_read32(priv, LSCANFD_RX_DATA);

		put_unaligned_le32(data, cf->data + (i - 2) * 4);
	}

	if (!(cf->can_id & CAN_RTR_FLAG))
		stats->rx_bytes += cf->len;
	stats->rx_packets++;
	netif_receive_skb(skb);
	return 0;
}

static int lscanfd_rx_poll(struct napi_struct *napi, int quota)
{
	struct net_device *ndev = napi->dev;
	struct lscanfd_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	int work_done = 0;
	u32 rx_stat, framecnt;

	while (work_done < quota) {
		rx_stat = lscanfd_read32(priv, LSCANFD_RX_STAT);
		framecnt = FIELD_GET(LSCANFD_RX_STAT_RXFRC, rx_stat);

		if (!framecnt)
			break;

		if (lscanfd_rx_one(ndev) < 0)
			break;

		work_done++;
	}

	/* Overrun: clear DOR, then release RX buffer if needed */
	rx_stat = lscanfd_read32(priv, LSCANFD_CR_STAT);
	if (rx_stat & LSCANFD_STAT_DOR) {
		struct can_frame *cf;
		struct sk_buff *skb;

		lscanfd_write32(priv, LSCANFD_CR_CMD, LSCANFD_CMD_CDO);
		stats->rx_over_errors++;
		stats->rx_errors++;

		skb = alloc_can_err_skb(ndev, &cf);
		if (skb) {
			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] = CAN_ERR_CRTL_RX_OVERFLOW;
			stats->rx_packets++;
			stats->rx_bytes += cf->len;
			netif_rx(skb);
		}
	}

	if (work_done < quota) {
		if (napi_complete_done(napi, work_done)) {
			/* Level-triggered; safe to unmask now */
			lscanfd_int_mask(priv, LSCANFD_INT_RBNEI);
			lscanfd_int_unmask(priv, LSCANFD_INT_RBNEI);
		}
	}

	return work_done;
}

/* -------------------------------------------------------------------------
 * TX path
 * ---------------------------------------------------------------------- */
static bool lscanfd_txbuf_writable(struct lscanfd_priv *priv, u8 buf)
{
	u32 tx_stat = lscanfd_read32(priv, LSCANFD_TX_STAT);

	/* BRP bit set => buffer pending or not yet added */
	if (FIELD_GET(LSCANFD_TX_STAT_BRP, tx_stat) & BIT(buf))
		return false;

	/* BS state must be IDLE */
	if ((FIELD_GET(LSCANFD_TX_STAT_BS, tx_stat) >> (buf * 2)) & 0x3)
		return false;

	return true;
}

static bool lscanfd_insert_frame(struct lscanfd_priv *priv,
				 const struct canfd_frame *cf, u8 buf, bool is_fd)
{
	u32 meta0 = 0, meta1 = 0;
	u32 i;

	if (buf >= LSCANFD_TXBUF_NUM)
		return false;

	if (!lscanfd_txbuf_writable(priv, buf))
		return false;

	if (cf->can_id & CAN_EFF_FLAG) {
		meta0 = cf->can_id & CAN_EFF_MASK;
		meta0 |= LSCANFD_META0_XDT;
	} else {
		meta0 = FIELD_PREP(LSCANFD_META0_ID_BASE,
				   cf->can_id & CAN_SFF_MASK);
	}
	if (cf->can_id & CAN_RTR_FLAG)
		meta0 |= LSCANFD_META0_RTR;

	if (is_fd) {
		meta1 |= LSCANFD_META1_FDF;
		if (cf->flags & CANFD_BRS)
			meta1 |= LSCANFD_META1_BRS;
	}
	meta1 |= FIELD_PREP(LSCANFD_META1_DLC, can_fd_len2dlc(cf->len));

	lscanfd_write32(priv, LSCANFD_TX_SEL, buf);
	lscanfd_write32(priv, LSCANFD_TX_DATA + LSCANFD_META0_OFF, meta0);
	lscanfd_write32(priv, LSCANFD_TX_DATA + LSCANFD_META1_OFF, meta1);

	if (!(cf->can_id & CAN_RTR_FLAG)) {
		for (i = 0; i < cf->len; i += 4) {
			u32 word = get_unaligned_le32(cf->data + i);

			lscanfd_write32(priv,
					LSCANFD_TX_DATA + LSCANFD_DATA_OFF + i,
					word);
		}
	}
	return true;
}

static void lscanfd_tx_add(struct lscanfd_priv *priv, u8 buf)
{
	lscanfd_write32(priv, LSCANFD_TX_CMD,
			FIELD_PREP(LSCANFD_TX_CMD_BAR, BIT(buf)));
}

static void lscanfd_tx_clean(struct lscanfd_priv *priv, u8 buf)
{
	lscanfd_write32(priv, LSCANFD_TX_CMD,
			FIELD_PREP(LSCANFD_TX_CMD_BSC, BIT(buf)));
}

static netdev_tx_t lscanfd_start_xmit(struct sk_buff *skb,
				      struct net_device *ndev)
{
	struct lscanfd_priv *priv = netdev_priv(ndev);
	struct canfd_frame *cf = (struct canfd_frame *)skb->data;
	unsigned long flags;
	u8 buf, candidate;

	if (can_dropped_invalid_skb(ndev, skb))
		return NETDEV_TX_OK;

	spin_lock_irqsave(&priv->tx_lock, flags);

	for (buf = 0; buf < LSCANFD_TXBUF_NUM; buf++) {
		candidate = (priv->txb_head + buf) % LSCANFD_TXBUF_NUM;
		if (lscanfd_txbuf_writable(priv, candidate))
			break;
	}
	if (buf == LSCANFD_TXBUF_NUM) {
		netif_stop_queue(ndev);
		spin_unlock_irqrestore(&priv->tx_lock, flags);
		return NETDEV_TX_BUSY;
	}
	buf = candidate;

	if (!lscanfd_insert_frame(priv, cf, buf, can_is_canfd_skb(skb))) {
		spin_unlock_irqrestore(&priv->tx_lock, flags);
		dev_kfree_skb_any(skb);
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	can_put_echo_skb(skb, ndev, buf, 0);
	priv->txb_head = (buf + 1) % LSCANFD_TXBUF_NUM;

	lscanfd_tx_add(priv, buf);

	/* If every buffer is now pending, stop the queue */
	if (FIELD_GET(LSCANFD_TX_STAT_BRP,
		      lscanfd_read32(priv, LSCANFD_TX_STAT)) ==
	    GENMASK(LSCANFD_TXBUF_NUM - 1, 0))
		netif_stop_queue(ndev);

	spin_unlock_irqrestore(&priv->tx_lock, flags);
	return NETDEV_TX_OK;
}

static void lscanfd_tx_interrupt(struct net_device *ndev)
{
	struct lscanfd_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	unsigned long flags;
	bool processed;

	spin_lock_irqsave(&priv->tx_lock, flags);

	do {
		u8 buf = priv->txb_tail;
		u32 tx_stat;
		u8 sts;

		processed = false;
		tx_stat = lscanfd_read32(priv, LSCANFD_TX_STAT);
		sts = (FIELD_GET(LSCANFD_TX_STAT_BS, tx_stat) >> (buf * 2)) & 0x3;

		if (sts == LSCANFD_TXT_IDLE)
			break;

		switch (sts) {
		case LSCANFD_TXT_VALID:
			stats->tx_bytes += can_get_echo_skb(ndev, buf, NULL);
			stats->tx_packets++;
			break;
		case LSCANFD_TXT_FAIL:
		case LSCANFD_TXT_CANCEL:
			can_free_echo_skb(ndev, buf, NULL);
			stats->tx_dropped++;
			break;
		default:
			break;
		}

		lscanfd_tx_clean(priv, buf);
		priv->txb_tail = (buf + 1) % LSCANFD_TXBUF_NUM;
		processed = true;
	} while (processed);

	spin_unlock_irqrestore(&priv->tx_lock, flags);

	if (netif_queue_stopped(ndev))
		netif_wake_queue(ndev);
}

/* -------------------------------------------------------------------------
 * IRQ handler
 * ---------------------------------------------------------------------- */
static irqreturn_t lscanfd_interrupt(int irq, void *dev_id)
{
	struct net_device *ndev = dev_id;
	struct lscanfd_priv *priv = netdev_priv(ndev);
	int loops;

	for (loops = 0; loops < 10000; loops++) {
		u32 isr = lscanfd_read32(priv, LSCANFD_INT_STAT);

		if (!isr)
			return loops ? IRQ_HANDLED : IRQ_NONE;

		if (isr & LSCANFD_INT_TXBHCI) {
			lscanfd_write32(priv, LSCANFD_INT_STAT,
					LSCANFD_INT_TXBHCI);
			lscanfd_tx_interrupt(ndev);
		}

		if (isr & (LSCANFD_INT_RBNEI | LSCANFD_INT_DMADI)) {
			u32 rx_irq = isr & (LSCANFD_INT_RBNEI |
					    LSCANFD_INT_DMADI);

			lscanfd_int_mask(priv, rx_irq);
			lscanfd_write32(priv, LSCANFD_INT_STAT, rx_irq);
			napi_schedule(&priv->napi);
		}

		if (isr & LSCANFD_INT_ERRORI) {
			u32 err_irq = isr & LSCANFD_INT_ERRORI;

			lscanfd_int_mask(priv, err_irq);
			lscanfd_write32(priv, LSCANFD_INT_STAT, err_irq);
			lscanfd_err_interrupt(ndev, isr);
			lscanfd_int_unmask(priv, err_irq);
		}

		/* RXI/TXI/BSI/RXFI/OFI/DOI are not used by this driver */
	}

	netdev_err(ndev, "interrupt loop timeout, ISR=0x%08x\n", 
		   lscanfd_read32(priv, LSCANFD_INT_STAT));
	lscanfd_disable_interrupts(priv);
	return IRQ_HANDLED;
}

/* -------------------------------------------------------------------------
 * CAN priv callbacks
 * ---------------------------------------------------------------------- */
static int lscanfd_do_set_bittiming(struct net_device *ndev)
{
	struct lscanfd_priv *priv = netdev_priv(ndev);

	if (lscanfd_enabled(priv))
		return -EBUSY;

	lscanfd_set_bittiming(ndev);
	return 0;
}

static int lscanfd_do_set_data_bittiming(struct net_device *ndev)
{
	struct lscanfd_priv *priv = netdev_priv(ndev);

	if (lscanfd_enabled(priv))
		return -EBUSY;

	lscanfd_set_data_bittiming(ndev);
	lscanfd_set_ssp(priv);
	return 0;
}

static int lscanfd_do_set_mode(struct net_device *ndev, enum can_mode mode)
{
	struct lscanfd_priv *priv = netdev_priv(ndev);

	switch (mode) {
	case CAN_MODE_START:
		lscanfd_reset(priv);
		lscanfd_chip_start(ndev);
		netif_wake_queue(ndev);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int lscanfd_do_get_berr_counter(const struct net_device *ndev,
				       struct can_berr_counter *bec)
{
	struct lscanfd_priv *priv = netdev_priv(ndev);

	lscanfd_get_berr_counter(priv, bec);
	return 0;
}

/* -------------------------------------------------------------------------
 * net_device ops
 * ---------------------------------------------------------------------- */
static int lscanfd_open(struct net_device *ndev)
{
	struct lscanfd_priv *priv = netdev_priv(ndev);
	int ret;

	lscanfd_reset(priv);

	ret = open_candev(ndev);
	if (ret) {
		netdev_err(ndev, "open_candev failed\n");
		return ret;
	}

	ret = request_irq(priv->irq, lscanfd_interrupt, IRQF_SHARED,
			  ndev->name, ndev);
	if (ret) {
		netdev_err(ndev, "request_irq failed\n");
		goto err_close;
	}

	napi_enable(&priv->napi);

	ret = lscanfd_chip_start(ndev);
	if (ret) {
		netdev_err(ndev, "chip_start failed\n");
		goto err_napi;
	}

	netif_start_queue(ndev);
	netdev_info(ndev, "Loongson CANFD device registered\n");
	return 0;

err_napi:
	napi_disable(&priv->napi);
	free_irq(priv->irq, ndev);
err_close:
	close_candev(ndev);
	return ret;
}

static int lscanfd_close(struct net_device *ndev)
{
	struct lscanfd_priv *priv = netdev_priv(ndev);

	netif_stop_queue(ndev);
	lscanfd_chip_stop(ndev);
	napi_disable(&priv->napi);
	free_irq(priv->irq, ndev);
	close_candev(ndev);
	return 0;
}

static const struct net_device_ops lscanfd_netdev_ops = {
	.ndo_open	= lscanfd_open,
	.ndo_stop	= lscanfd_close,
	.ndo_start_xmit	= lscanfd_start_xmit,
};

/* -------------------------------------------------------------------------
 * Probe / remove
 * ---------------------------------------------------------------------- */
static int lscanfd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lscanfd_priv *priv;
	struct net_device *ndev;
	struct resource *res;
	void __iomem *base;
	int ret, irq;
	u32 clk_rate;

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	ret = device_property_read_u32(dev, "clock-frequency", &clk_rate);
	if (ret)
		return dev_err_probe(dev, ret, "missing clock-frequency\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return dev_err_probe(dev, irq, "failed to get IRQ\n");

	ndev = alloc_candev(sizeof(*priv), LSCANFD_TXBUF_NUM);
	if (!ndev)
		return -ENOMEM;

	priv = netdev_priv(ndev);
	priv->base = base;
	priv->irq = irq;
	priv->clk_rate = clk_rate;
	spin_lock_init(&priv->tx_lock);

	priv->can.clock.freq = clk_rate;
	priv->can.bittiming_const = &lscanfd_bittiming_const;
	priv->can.fd.data_bittiming_const = &lscanfd_data_bittiming_const;
	priv->can.fd.tdc_const = &lscanfd_tdc_const;
	priv->can.do_set_bittiming = lscanfd_do_set_bittiming;
	priv->can.fd.do_set_data_bittiming = lscanfd_do_set_data_bittiming;
	priv->can.do_set_mode = lscanfd_do_set_mode;
	priv->can.do_get_berr_counter = lscanfd_do_get_berr_counter;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
					CAN_CTRLMODE_LISTENONLY |
					CAN_CTRLMODE_ONE_SHOT |
					CAN_CTRLMODE_BERR_REPORTING |
					CAN_CTRLMODE_FD |
					CAN_CTRLMODE_PRESUME_ACK |
					CAN_CTRLMODE_FD_NON_ISO |
					CAN_CTRLMODE_TDC_AUTO |
					CAN_CTRLMODE_CC_LEN8_DLC;

	ndev->irq = irq;
	ndev->flags |= IFF_ECHO;
	ndev->netdev_ops = &lscanfd_netdev_ops;
	SET_NETDEV_DEV(ndev, dev);

	platform_set_drvdata(pdev, ndev);

	/* Check the controller ID */
	if ((lscanfd_read32(priv, LSCANFD_CR_ID) & 0xffff) !=
	    LSCANFD_DEVICE_ID) {
		netdev_err(ndev, "LSCANFD signature not found\n");
		ret = -ENODEV;
		goto err_free;
	}

	lscanfd_reset(priv);

	/* RX/TX queues */
	priv->txb_head = 0;
	priv->txb_tail = 0;

	netif_napi_add(ndev, &priv->napi, lscanfd_rx_poll);

	ret = register_candev(ndev);
	if (ret) {
		netdev_err(ndev, "register_candev failed (%d)\n", ret);
		goto err_napi;
	}

	return 0;

err_napi:
	netif_napi_del(&priv->napi);
err_free:
	free_candev(ndev);
	return ret;
}

static void lscanfd_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct lscanfd_priv *priv = netdev_priv(ndev);

	unregister_candev(ndev);
	netif_napi_del(&priv->napi);
	free_candev(ndev);
}

/* -------------------------------------------------------------------------
 * Module boilerplate
 * ---------------------------------------------------------------------- */
static const struct acpi_device_id lscanfd_acpi_ids[] = {
	{ "LOON0015", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, lscanfd_acpi_ids);

static struct platform_driver lscanfd_driver = {
	.probe		= lscanfd_probe,
	.remove		= lscanfd_remove,
	.driver		= {
		.name		= LSCANFD_DRV_NAME,
		.acpi_match_table = lscanfd_acpi_ids,
	},
};
module_platform_driver(lscanfd_driver);

MODULE_AUTHOR("Loongson Technology Corporation Limited");
MODULE_DESCRIPTION("Loongson 2K3000 CAN-FD controller driver");
MODULE_LICENSE("GPL");
