// SPDX-License-Identifier: GPL-2.0+
/*
 * LOONGSON CANFD controller driver
 *
 * Copyright (C) 2024-2026 Loongson Technology Corporation Limited
 */

#include <linux/acpi.h>
#include <linux/acpi_dma.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/cleanup.h>
#include <linux/dmaengine.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/skbuff.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#define LOONGSON_CANFD_DEVICE_ID	0x0	/* CANFD controller ID Register */
#define LOONGSON_CANFD_MODE		0x4	/* Mode Configuration Register */
#define LOONGSON_CANFD_CONF		0x8	/* Configure Register */
#define LOONGSON_CANFD_STAT		0xc	/* Status Register */
#define LOONGSON_CANFD_CMD		0x10	/* Command Register */
#define LOONGSON_CANFD_INT_STAT		0x14	/* Interrupt Status Register */
#define LOONGSON_CANFD_INT_ENA		0x18	/* Interrupt Enable Register */
#define LOONGSON_CANFD_INT_MASK		0x1c	/* Interrupt Mask Register */
#define LOONGSON_CANFD_BTR_NORM		0x20	/* Normal Rate Configuration Register */
#define LOONGSON_CANFD_BTR_FD		0x24	/* FD Data Rate Configuration Register */
#define LOONGSON_CANFD_ERL		0x28	/* Error Threshold Configuration Register */
#define LOONGSON_CANFD_FSTAT		0x2c	/* Error Status Register */
#define LOONGSON_CANFD_ERC		0x30	/* Error Count Register */
#define LOONGSON_CANFD_BRE		0x34	/* Rate Error Count Register */
#define LOONGSON_CANFD_CTR_PRES		0x38	/* Error Count Debug Register */
#define LOONGSON_CANFD_ERR_CAPT		0x3c	/* Error Capture Status Register */
#define LOONGSON_CANFD_RETX_CNT		0x40	/* Retransmission Count Register */
#define LOONGSON_CANFD_ALC		0x44	/* Lost Arbitration Capture Register */
#define LOONGSON_CANFD_TRV_DLY		0x48	/* Transmission Delay Measurement Register */
#define LOONGSON_CANFD_SSP_CFG		0x4c	/* Second Sampling Point Configuration Register */
#define LOONGSON_CANFD_RX_FR_CNT	0x50	/* Receive Message Count Register */
#define LOONGSON_CANFD_TX_FR_CNT	0x54	/* Transmit Message Count Register */
#define LOONGSON_CANFD_DEBUG		0x58	/* Debug Register */
#define LOONGSON_CANFD_TS		0x5c	/* Timestamp Register */
#define LOONGSON_CANFD_TX_FRM_TST	0x60	/* Transmit Message Debug Register */
#define LOONGSON_CANFD_FRC_DIV		0x64	/* Fractional Divider Ratio Register */
#define LOONGSON_CANFD_FLT_A_MASK	0x68	/* Filter A Mask Register */
#define LOONGSON_CANFD_FLT_A_VAL	0x6c	/* Filter A value Register */
#define LOONGSON_CANFD_FLT_B_MASK	0x70	/* Filter B Mask Register */
#define LOONGSON_CANFD_FLT_B_VAL	0x74	/* Filter B value Register */
#define LOONGSON_CANFD_FLT_C_MASK	0x78	/* Filter C Mask Register */
#define LOONGSON_CANFD_FLT_C_VAL	0x7c	/* Filter C value Register */
#define LOONGSON_CANFD_FLT_R_LOW	0x80	/* Range Filter Low Threshold Register */
#define LOONGSON_CANFD_FLT_R_HI		0x84	/* Range Filter High Threshold Register */
#define LOONGSON_CANFD_FLT_CTRL		0x88	/* Filter Control Register */
#define LOONGSON_CANFD_RX_MEM_INFO	0x8c	/* Receive Buffer Information Register */
#define LOONGSON_CANFD_RX_PRT		0x90	/* Receive Buffer Pointer Register */
#define LOONGSON_CANFD_RX_STAT		0x94	/* Receive Buffer Status Register */
#define LOONGSON_CANFD_RX_DATA		0x98	/* Receive Data Register */
#define LOONGSON_CANFD_TX_STAT		0x9c	/* Transmit Buffer Status Register */
#define LOONGSON_CANFD_TX_CMD		0xa0	/* Transmit Command Register */
#define LOONGSON_CANFD_TX_SEL		0xa4	/* Transmit Buffer Selection Register */
#define LOONGSON_CANFD_TX_DATA_1	0xb0
#define LOONGSON_CANFD_TX_DATA_18	0xf4

/* Bitfields of CANFD controller ID register */
#define REG_ID_MASK			GENMASK(15, 0)
#define REG_ID_VER_MIN			GENMASK(23, 16)
#define REG_ID_VER_MAJ			GENMASK(31, 24)

/* Bitfields of Mode Configuration register */
#define REG_MODE_RST			BIT(0)	/* Soft Reset */
#define REG_MODE_BMM			BIT(1)	/* Bus Monitoring Mode */
#define REG_MODE_STM			BIT(2)	/* Self-Test Mode */
#define REG_MODE_AFM			BIT(3)	/* Receive Filter Mode */
#define REG_MODE_FDE			BIT(4)	/* CANFD Enabled */
#define REG_MODE_TTTM			BIT(5)	/* Timed Transmission Mode */
#define REG_MODE_ROM			BIT(6)	/* Restricted Operation Mode */
#define REG_MODE_ACF			BIT(7)	/* No ACK Mode*/
#define REG_MODE_TSTM			BIT(8)	/* Test Mode */
#define REG_MODE_RXBAM			BIT(9)	/* Receive Buffer Auto Mode */
#define REG_MODE_ITSM			BIT(10)	/* Internal Timestamp Mode */
#define REG_MODE_RTSOP			BIT(12)	/* Receive Buffer Timestamp Addition Mode */
#define REG_MODE_BUFM			BIT(13)	/* Transmit Buffer Mode */

/* Bitfields of Configure register */
#define REG_CONF_RTRLE			BIT(0)	/* Retransmission Threshold Enable */
#define REG_CONF_RTRTH			GENMASK(4, 1)	/* Retransmission Threshold */
#define REG_CONF_ILBP			BIT(5)	/* Internal Loopback Mode */
#define REG_CONF_ENA			BIT(6)	/* Controller Enable */
#define REG_CONF_NISOFD			BIT(7)	/* NON-ISO CANFD Protocol Enable */
#define REG_CONF_PEX			BIT(8)	/* Protocol Exception Handling */
#define REG_CONF_FDRF			BIT(10)	/* Ignore Remote Frame Enable */

/* Bitfields of Status register */
#define REG_STAT_RXNE			BIT(0)	/* Receive Buffer Not Empty Status */
#define REG_STAT_DOR			BIT(1)	/* Receive Data Overflow Status */
#define REG_STAT_EFT			BIT(3)	/* Error Frame Flag */
#define REG_STAT_RXS			BIT(4)	/* Receive Status Flag */
#define REG_STAT_TXS			BIT(5)	/* Transmission Status Flag */
#define REG_STAT_EWL			BIT(6)	/* Transmission Error Threshold Flag */
#define REG_STAT_IDLE			BIT(7)	/* Bus Idle Flag */
#define REG_STAT_PEXS			BIT(8)	/* Protocol Exception Status */
#define REG_STAT_STCNT			BIT(16)	/* Transmission Count Support */

/* Bitfields of Command register */
#define REG_CMD_RXRPMV			BIT(1)	/* Receive Buffer Read-to-Move */
#define REG_CMD_RRB			BIT(2)	/* Receive Buffer Release */
#define REG_CMD_CDO			BIT(3)	/* Received Data Overflow Status Clear */
#define REG_CMD_ERCRST			BIT(4)	/* Error Count Clear */
#define REG_CMD_RXFCRST			BIT(5)	/* Receive Frame Count Clear */
#define REG_CMD_TXFCRST			BIT(6)	/* Transmit Frame Count Clear */
#define REG_CMD_CPEXS			BIT(7)	/* Exception Status Clear */

/* Bitfields of Interrupt Status register */
#define REG_INT_STAT_RXI		BIT(0)	/* Message Reception Interrupt */
#define REG_INT_STAT_TXI		BIT(1)	/* Message Transmission Interrupt */
#define REG_INT_STAT_EWLI		BIT(2)	/* Error Warning Threshold Interrupt */
#define REG_INT_STAT_DOI		BIT(3)	/* Receive Data Overflow Interrupt */
#define REG_INT_STAT_FCSI		BIT(4)	/* Fault State Change Interrupt */
#define REG_INT_STAT_ALI		BIT(5)	/* Arbitration Loss Interrupt */
#define REG_INT_STAT_BEI		BIT(6)	/* Bus Error Interrupt */
#define REG_INT_STAT_RXFI		BIT(7)	/* Receive Buffer Full Interrupt */
#define REG_INT_STAT_BSI		BIT(8)	/* Rate Switching Interrupt */
#define REG_INT_STAT_RBNEI		BIT(9)	/* Receive Buffer Not Empty Interrupt */
#define REG_INT_STAT_TXBHCI		BIT(10)	/* Transmit Buffer Hardware Command Interrupt */
#define REG_INT_STAT_OFI		BIT(11)	/* Overload Frame Transmission Interrupt */
#define REG_INT_STAT_DMADI		BIT(12)	/* Message DMA Transfer Complete Interrupt */

#define REG_INT_STAT_ERRORI		(REG_INT_STAT_EWLI | REG_INT_STAT_FCSI |\
					 REG_INT_STAT_ALI | REG_INT_STAT_BEI)

/* Bitfields of Interrupt Enable register */
#define REG_INT_ENA_SET			GENMASK(12, 0)
#define REG_INT_ENA_CLR			GENMASK(28, 16)

/* Bitfields of Interrupt Mask register */
#define REG_INT_MASK_SET		GENMASK(12, 0)
#define REG_INT_MASK_CLR		GENMASK(28, 16)

/* Bitfields of Normal Rate Configuration register */
#define REG_BTR_PROP			GENMASK(6, 0)
#define REG_BTR_PH1			GENMASK(12, 7)
#define REG_BTR_PH2			GENMASK(18, 13)
#define REG_BTR_BRP			GENMASK(22, 19)
#define REG_BTR_SJW			GENMASK(31, 27)

/* Bitfields of FD Data Rate Configuration register */
#define REG_BTR_FD_PROP			GENMASK(6, 0)
#define REG_BTR_FD_PH1			GENMASK(12, 7)
#define REG_BTR_FD_PH2			GENMASK(18, 13)
#define REG_BTR_FD_BRP			GENMASK(26, 19)
#define REG_BTR_FD_SJW			GENMASK(31, 27)

/* Bitfields of Error Threshold Configuration register */
#define REG_ERL_ERP			GENMASK(7, 0)
#define REG_ERL_EW			GENMASK(23, 16)

/* Bitfields of Error Status register */
#define REG_FSTAT_ERA			BIT(0)
#define REG_FSTAT_ERP			BIT(1)
#define REG_FSTAT_BOF			BIT(2)

#define REG_FSTAT_MASK			GENMASK(2, 0)

/* Bitfields of Error Count register */
#define REG_ERC_TEC			GENMASK(8, 0)
#define REG_ERC_REC			GENMASK(24, 16)

/* Bitfields of Rate Error Count register */
#define REG_BRE_NORM			GENMASK(15, 0)
#define REG_BRE_FD_DATA			GENMASK(31, 16)

/* Bitfields of Error Count Debug register */
#define REG_CTR_PRES_CTPV		GENMASK(8, 0)
#define REG_CTR_PRES_PTX		BIT(9)
#define REG_CTR_PRES_PRX		BIT(10)

/* Bitfields of Error Capture Status register */
#define REG_ERR_CAPT_POS		GENMASK(4, 0)
#define REG_ERR_CAPT_TYPE		GENMASK(7, 5)

/* Bitfields of Retransmission Count register */
#define REG_RETX_CNT_VAL		GENMASK(3, 0)

/* Bitfields of Lost Arbitration Capture register */
#define REG_ALC_BIT_POS			GENMASK(4, 0)
#define REG_ALC_ID_FIELD		GENMASK(7, 5)

/* Bitfields of Transmission Delay Measurement register */
#define REG_TRV_DLY_VAL			GENMASK(6, 0)

/* Bitfields of Second Sampling Point Configuration register */
#define REG_SSP_CFG_OFF			GENMASK(7, 0)
#define REG_SSP_CFG_SRC			GENMASK(9, 8)
#define REG_SSP_CFG_SAT			BIT(10)

/* Bitfields of Receive Message Count register */
#define REG_RX_FR_CNT_VAL		GENMASK(31, 0)

/* Bitfields of Transmit Message Count register */
#define REG_TX_FR_CNT_VAL		GENMASK(31, 0)

/* Bitfields of Debug register */
#define REG_DEBUG_STF_CNT		GENMASK(2, 0)
#define REG_DEBUG_DSTF_CNT		GENMASK(5, 3)
#define REG_DEBUG_PC_ARB		BIT(6)
#define REG_DEBUG_PC_CON		BIT(7)
#define REG_DEBUG_PC_DAT		BIT(8)
#define REG_DEBUG_PC_STC		BIT(9)
#define REG_DEBUG_PC_CRC		BIT(10)
#define REG_DEBUG_PC_CRCD		BIT(11)
#define REG_DEBUG_PC_ACK		BIT(12)
#define REG_DEBUG_PC_ACKD		BIT(13)
#define REG_DEBUG_PC_EOF		BIT(14)
#define REG_DEBUG_PC_INT		BIT(15)
#define REG_DEBUG_PC_SUSP		BIT(16)
#define REG_DEBUG_PC_OVR		BIT(17)
#define REG_DEBUG_PC_SOF		BIT(18)

/* Bitfields of Timestamp register */
#define REG_TS_TIMESTAMP		GENMASK(15, 0)
#define REG_TS_PSC			GENMASK(24, 16)

/* Bitfields of Fractional Divider Ratio register */
#define REG_FRC_FRC_DBT			GENMASK(15, 8)
#define REG_FRC_FRC_NBT			GENMASK(7, 0)

/* Bitfields of Filter A Mask register */
#define REG_FIL_A_MASK			GENMASK(28, 0)

/* Bitfields of Filter A value register */
#define REG_FIL_A_VAL			GENMASK(28, 0)

/* Bitfields of Filter B Mask register */
#define REG_FIL_B_MASK			GENMASK(28, 0)

/* Bitfields of Filter B value register */
#define REG_FIL_B_VAL			GENMASK(28, 0)

/* Bitfields of Filter C Mask register */
#define REG_FIL_C_MASK			GENMASK(28, 0)

/* Bitfields of Filter C value register */
#define REG_FIL_C_VAL			GENMASK(28, 0)

/* Bitfields of Range Filter Low Threshold register */
#define REG_FIL_R_LOW_VAL		GENMASK(28, 0)

/* Bitfields of Range Filter High Threshold register */
#define REG_FIL_R_HI_VAL		GENMASK(28, 0)

/* Bitfields of Filter Control register */
#define REG_FIL_CTRL_FANB		BIT(0)
#define REG_FIL_CTRL_FANE		BIT(1)
#define REG_FIL_CTRL_FAFB		BIT(2)
#define REG_FIL_CTRL_FAFE		BIT(3)
#define REG_FIL_CTRL_FBNB		BIT(4)
#define REG_FIL_CTRL_FBNE		BIT(5)
#define REG_FIL_CTRL_FBFB		BIT(6)
#define REG_FIL_CTRL_FBFE		BIT(7)
#define REG_FIL_CTRL_FCNB		BIT(8)
#define REG_FIL_CTRL_FCNE		BIT(9)
#define REG_FIL_CTRL_FCFB		BIT(10)
#define REG_FIL_CTRL_FCFE		BIT(11)
#define REG_FIL_CTRL_FRNB		BIT(12)
#define REG_FIL_CTRL_FRNE		BIT(13)
#define REG_FIL_CTRL_FRFB		BIT(14)
#define REG_FIL_CTRL_FRFE		BIT(15)
#define REG_FIL_CTRL_SFA		BIT(16)
#define REG_FIL_CTRL_SFB		BIT(17)
#define REG_FIL_CTRL_SFC		BIT(18)
#define REG_FIL_CTRL_SFR		BIT(19)

/* Bitfields of Receive Buffer Information register */
#define REG_RX_MEM_INFO_BUFF_SIZE	GENMASK(12, 0)
#define REG_RX_MEM_INFO_MEM_FREE	GENMASK(28, 16)

/* Bitfields of Receive Buffer Pointer register */
#define REG_RX_PTR_WPP			GENMASK(11, 0)
#define REG_RX_PTR_RPP			GENMASK(27, 16)

/* Bitfields of Receive Buffer Status register */
#define REG_RX_STAT_RXE			BIT(0)
#define REG_RX_STAT_RXF			BIT(1)
#define REG_RX_STAT_RXMOF		BIT(2)
#define REG_RX_STAT_RXFRC		GENMASK(14, 4)
#define REG_RX_STAT_RTSOP		BIT(16)

/* Bitfields of Receive Data register */
#define REG_RX_DATA_VAL			GENMASK(31, 0)

/* Bitfields of Transmit Buffer Status register */
#define REG_TX_STAT_BRP			GENMASK(7, 0)
#define REG_TX_STAT_TXS			GENMASK(10, 8)
#define REG_TX_STAT_BS			GENMASK(31, 16)

#define REG_TX_STAT_BS_TX_MASK_BASE	GENMASK(17, 16)

/* Bitfields of Transmit Command register */
#define REG_TX_CMD_BAR			GENMASK(7, 0)
#define REG_TX_CMD_BCR			GENMASK(15, 8)
#define REG_TX_CMD_BSC			GENMASK(23, 16)

/* Bitfields of Transmit Buffer Selection register */
#define REG_TX_SEL_BUF_SEL		GENMASK(3, 0)
#define REG_TX_SEL_BUF_CNT		GENMASK(7, 4)

/* Loongson CANFD Frame format */
#define LOONGSON_CANFD_FRAME_META0	0x0
#define LOONGSON_CANFD_FRAME_META1	0x4
#define LOONGSON_CANFD_FRAME_DB_1	0x8

/* Bitfields of FRAME META0 */
#define REG_FRAME_META0_ID_EXT		GENMASK(17, 0)
#define REG_FRAME_META0_ID_BASE		GENMASK(28, 18)
#define REG_FRAME_META0_RTR		BIT(29)	/* Remote Transmission Request, only in CAN2.0 */
/* Extended Identifier Type. 1: 29 bits ID, 0: 11 bits ID. */
#define REG_FRAME_META0_XDT		BIT(30)
#define REG_FRAME_META0_ESI		BIT(31)	/* Error State Indicator, only in CAN-FD */

/* Bitfields of FRAME META1 */
#define REG_FRAME_META1_TIMESTAMP	GENMASK(15, 0)
#define REG_FRAME_META1_DLC		GENMASK(19, 16)	/* Data Length Code */
#define REG_FRAME_META1_BRS		BIT(20)	/* Bit Rate Shift, Only in CANFD */
/* Flexible Data-rate Format. 1: CANFD, 0: CAN2.0 */
#define REG_FRAME_META1_FDF		BIT(21)
#define REG_FRAME_META1_RWCNT		GENMASK(28, 24)	/* Read Word Counter */

/* Bitfields of FRAME TEST */
#define REG_FRAME_TEST_FSTC		BIT(0)
#define REG_FRAME_TEST_FCRC		BIT(1)
#define REG_FRAME_TEST_SDLC		BIT(2)
#define REG_FRAME_TEST_TPRM		GENMASK(12, 8)

#define DEV_NAME			"loongson_canfd"
#define LOONGSON_CANFD_ID		0xBABE
#define LOONGSON_CANFD_DW_BYTE		4
#define LOONGSON_CANFD_RXBUF_SZ		SZ_1K
#define LOONGSON_CANFD_TXBUF_NUM	8
#define LOONGSON_CANFD_MAX_RTXTH	0xf
#define LOONGSON_CANFD_RXDMA_NUM	(LOONGSON_CANFD_RXBUF_SZ / DMA_SLAVE_BUSWIDTH_4_BYTES)

/**
 * struct loongson_canfd_priv - Driver private data
 * @can: CAN private data structure (must be first member)
 * @napi: NAPI structure
 * @regmap: Regmap of the CAN device
 * @res: Pointer to the CAN device resource (MMIO region)
 * @tx_lock: Lock for synchronizing TX interrupt handling
 * @stats_lock: Lock for protecting netdev statistics
 * @rx_ch: DMA receive channel
 * @rx_cookie: DMA receive cookie
 * @rx_dma_buf: DMA RX buffer bus address
 * @rx_buf: DMA RX buffer CPU address
 * @rx_rd_ptr_dist: Read pointer distance from buffer end (in 32-bit words)
 * @get_rx_data: Callback for reading RX data (DMA or poll mode)
 * @get_rxbuf_empty: Callback for checking RX buffer empty state
 */
struct loongson_canfd_priv {
	struct can_priv		can;
	struct napi_struct	napi;
	struct regmap		*regmap;
	struct resource		*res;
	spinlock_t		tx_lock;	/* protect the sending queue */
	spinlock_t		stats_lock;	/* protect the ndev->stats */
	struct dma_chan		*rx_ch;
	dma_cookie_t		rx_cookie;
	dma_addr_t		rx_dma_buf;
	unsigned int		*rx_buf;
	u16			rx_rd_ptr_dist;
	u32 (*get_rx_data)(struct loongson_canfd_priv *priv);
	bool (*get_rxbuf_empty)(struct loongson_canfd_priv *priv);
};

/**
 * enum loongson_canfd_txbuf_sts - Status of TX buffer
 * @TX_BS_IDLE: No transmission or processed.
 * @TX_BS_VALID: Transmission successful.
 * @TX_BS_FAIL: Transmission failed (retry limit exceeded).
 * @TX_BS_CANCEL: Transmission cancelled.
 */
enum loongson_canfd_txbuf_sts {
	TX_BS_IDLE	= 0x0,
	TX_BS_VALID	= 0x1,
	TX_BS_FAIL	= 0x2,
	TX_BS_CANCEL	= 0x3
};

/**
 * enum loongson_canfd_txbuf_cmd - Commands for TX buffer
 * @TXT_CMD_ADD: Add buffer request (start transmission)
 * @TXT_CMD_CANCEL: Cancel buffer request
 * @TXT_CMD_SR_CLEAN: Clear send completion record
 */
enum loongson_canfd_txbuf_cmd {
	TXT_CMD_ADD		= 0x01,
	TXT_CMD_CANCEL		= 0x02,
	TXT_CMD_SR_CLEAN	= 0x03
};

static const struct can_bittiming_const loongson_canfd_bit_timing = {
	.name		= DEV_NAME,
	.tseg1_min	= 2,
	.tseg1_max	= 190,
	.tseg2_min	= 2,
	.tseg2_max	= 63,
	.sjw_max	= 31,
	.brp_min	= 1,
	.brp_max	= 15,
	.brp_inc	= 1,
};

static const struct can_bittiming_const loongson_canfd_bit_timing_data = {
	.name		= DEV_NAME,
	.tseg1_min	= 2,
	.tseg1_max	= 190,
	.tseg2_min	= 2,
	.tseg2_max	= 63,
	.sjw_max	= 31,
	.brp_min	= 1,
	.brp_max	= 255,
	.brp_inc	= 1,
};

static const struct can_tdc_const loongson_canfd_tdc_data = {
	.tdcv_min	= 0,
	.tdcv_max	= 0,
	.tdco_min	= 1,
	.tdco_max	= 127,
	.tdcf_min	= 0,
	.tdcf_max	= 0,
};

/**
 * loongson_canfd_enabled() - Check if controller is enabled
 * @priv: Driver private data
 *
 * Return: true if enabled, false otherwise.
 */
static bool loongson_canfd_enabled(struct loongson_canfd_priv *priv)
{
	return !!regmap_test_bits(priv->regmap, LOONGSON_CANFD_CONF, REG_CONF_ENA);
}

/**
 * loongson_canfd_txbuf_freed() - Check if at least one TX buffer is free
 * @priv: Driver private data
 *
 * Examines the TX buffer pending status (BRP bits). If any of the 8 buffers
 * is not pending, it indicates a free buffer.
 *
 * Return: true if at least one TX buffer is free, false if all are pending.
 */
static bool loongson_canfd_txbuf_freed(struct loongson_canfd_priv *priv)
{
	return !regmap_test_bits(priv->regmap, LOONGSON_CANFD_TX_STAT, REG_TX_STAT_BRP);
}

/**
 * loongson_canfd_disable_interrupts() - Disable all interrupts
 * @priv: Driver private data
 */
static void loongson_canfd_disable_interrupts(struct loongson_canfd_priv *priv)
{
	regmap_write(priv->regmap, LOONGSON_CANFD_INT_ENA, FIELD_MAX(REG_INT_ENA_CLR) << 16);
	regmap_write(priv->regmap, LOONGSON_CANFD_INT_MASK, FIELD_MAX(REG_INT_MASK_SET));
}

/**
 * loongson_canfd_get_txbuf_sts() - Get status of a TX buffer
 * @priv: Driver private data
 * @buf_id: Buffer index (0-based)
 *
 * Return: Status of the specified TX buffer.
 */
static enum loongson_canfd_txbuf_sts
loongson_canfd_get_txbuf_sts(struct loongson_canfd_priv *priv, u8 buf_id)
{
	u32 sts, mask;

	mask = REG_TX_STAT_BS_TX_MASK_BASE << (2 * buf_id);
	regmap_read(priv->regmap, LOONGSON_CANFD_TX_STAT, &sts);

	return field_get(mask, sts);
}

/**
 * loongson_canfd_txbuf_is_writable() - Check if a TX buffer is writable
 * @priv: Driver private data
 * @buf_id: Buffer index (0-based)
 *
 * Return: true if the buffer is in a writable state, false otherwise.
 */
static bool loongson_canfd_txbuf_is_writable(struct loongson_canfd_priv *priv, u8 buf_id)
{
	enum loongson_canfd_txbuf_sts bs;

	bs = loongson_canfd_get_txbuf_sts(priv, buf_id);
	if (bs)
		return false;

	return !regmap_test_bits(priv->regmap, LOONGSON_CANFD_TX_STAT, BIT(buf_id));
}

/**
 * loongson_canfd_set_txbuf_cmd() - Apply command on TX buffer
 * @ndev: Network device
 * @cmd: Command to apply
 * @buf_id: Buffer index (0-based)
 */
static void loongson_canfd_set_txbuf_cmd(struct net_device *ndev,
					 enum loongson_canfd_txbuf_cmd cmd, u8 buf_id)
{
	struct loongson_canfd_priv *priv = netdev_priv(ndev);
	u32 mask;

	switch (cmd) {
	case TXT_CMD_ADD:
		mask = REG_TX_CMD_BAR;
		break;
	case TXT_CMD_CANCEL:
		mask = REG_TX_CMD_BCR;
		break;
	case TXT_CMD_SR_CLEAN:
		mask = REG_TX_CMD_BSC;
		break;
	default:
		netdev_err(ndev, "Unknown command id: %x\n", cmd);
		return;
	}

	regmap_write(priv->regmap, LOONGSON_CANFD_TX_CMD, field_prep(mask, BIT(buf_id)));
}

/**
 * loongson_canfd_rxdata_dma() - Read RX data in DMA mode
 * @priv: Driver private data
 *
 * Reads the next 32-bit word from the DMA RX buffer, advancing the read pointer.
 *
 * Return: The next 32-bit data word.
 */
static u32 loongson_canfd_rxdata_dma(struct loongson_canfd_priv *priv)
{
	u32 data = 0;

	data = priv->rx_buf[LOONGSON_CANFD_RXDMA_NUM - priv->rx_rd_ptr_dist--];
	if (!priv->rx_rd_ptr_dist)
		priv->rx_rd_ptr_dist = LOONGSON_CANFD_RXDMA_NUM;

	return data;
}

/**
 * loongson_canfd_rxbuf_empty_dma() - Check if RX buffer is empty in DMA mode
 * @priv: Driver private data
 *
 * Uses DMA transfer status to compare read and write pointers.
 * If the write pointer has wrapped past the read pointer, data loss is assumed,
 * and the read pointer is advanced to the write position.
 *
 * Return: true if buffer is empty, false if data is pending.
 */
static bool loongson_canfd_rxbuf_empty_dma(struct loongson_canfd_priv *priv)
{
	struct dma_tx_state state;
	enum dma_status status;
	u32 wptr_dist;

	status = dmaengine_tx_status(priv->rx_ch, priv->rx_cookie, &state);
	if (status != DMA_IN_PROGRESS)
		return true;

	/* Convert residue from bytes to number of 32-bit words */
	wptr_dist = state.residue / DMA_SLAVE_BUSWIDTH_4_BYTES;

	/*
	 * wptr_dist is the write pointer's distance from the buffer end.
	 * It starts at BUFFER_SIZE (when write pointer is at start) and
	 * decreases to 0 (when write pointer reaches end), then wraps back.
	 * priv->rx_rd_ptr_dist is the read pointer's distance from the end.
	 * If write pointer wraps past read pointer, wptr_dist becomes larger,
	 * indicating data loss, so we advance read pointer to write position.
	 */
	if (priv->rx_rd_ptr_dist < wptr_dist)
		priv->rx_rd_ptr_dist = wptr_dist;

	return priv->rx_rd_ptr_dist == wptr_dist;
}

static void loongson_canfd_rxdma_free(struct loongson_canfd_priv *priv, struct device *dev)
{
	if (!priv->rx_buf)
		return;

	dma_free_coherent(dev, LOONGSON_CANFD_RXBUF_SZ, priv->rx_buf, priv->rx_dma_buf);
	priv->rx_buf = NULL;
}

static void loongson_canfd_rxdma_remove(struct loongson_canfd_priv *priv, struct device *dev)
{
	if (!priv->rx_ch)
		return;

	dmaengine_terminate_sync(priv->rx_ch);
	loongson_canfd_rxdma_free(priv, dev);
	dma_release_channel(priv->rx_ch);
	priv->rx_ch = NULL;
}

/**
 * loongson_canfd_rxdma_init() - Initialize RX DMA
 * @ndev: Network device
 *
 * Allocates DMA buffer and configures cyclic DMA transfer.
 *
 * Return: 0 on success, negative error code on failure.
 */
static int loongson_canfd_rxdma_init(struct net_device *ndev)
{
	struct loongson_canfd_priv *priv = netdev_priv(ndev);
	struct dma_async_tx_descriptor *desc = NULL;
	struct device *dev = ndev->dev.parent;
	struct dma_slave_config config;
	int ret;

	if (!priv->rx_ch)
		return -EINVAL;

	priv->rx_buf = dma_alloc_coherent(dev, LOONGSON_CANFD_RXBUF_SZ,
					  &priv->rx_dma_buf, GFP_KERNEL);
	if (!priv->rx_buf) {
		dma_release_channel(priv->rx_ch);
		priv->rx_ch = NULL;
		return -ENOMEM;
	}

	memset(&config, 0, sizeof(config));
	config.src_addr = priv->res->start + LOONGSON_CANFD_RX_DATA;
	config.src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;

	ret = dmaengine_slave_config(priv->rx_ch, &config);
	if (ret < 0) {
		netdev_err(ndev, "RX DMA channel config failed\n");
		goto err_config;
	}

	desc = dmaengine_prep_dma_cyclic(priv->rx_ch, priv->rx_dma_buf,
					 LOONGSON_CANFD_RXBUF_SZ, LOONGSON_CANFD_RXBUF_SZ,
					 DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);
	if (!desc) {
		netdev_err(ndev, "RX DMA cyclic transaction preparation failed\n");
		ret = -EBUSY;
		goto err_config;
	}

	priv->rx_cookie = dmaengine_submit(desc);
	ret = dma_submit_error(priv->rx_cookie);
	if (ret) {
		dmaengine_terminate_sync(priv->rx_ch);
		goto err_config;
	}

	dma_async_issue_pending(priv->rx_ch);

	return 0;

err_config:
	loongson_canfd_rxdma_free(priv, dev);
	dma_release_channel(priv->rx_ch);
	priv->rx_ch = NULL;
	return ret;
}

/**
 * loongson_canfd_rxdata_poll() - Read RX data in poll mode
 * @priv: Driver private data
 *
 * Return: The next 32-bit data word from the RX data register.
 */
static u32 loongson_canfd_rxdata_poll(struct loongson_canfd_priv *priv)
{
	u32 data;

	regmap_read(priv->regmap, LOONGSON_CANFD_RX_DATA, &data);
	return data;
}

/**
 * loongson_canfd_rxbuf_empty_poll() - Check if RX buffer is empty in poll mode
 * @priv: Driver private data
 *
 * Return: true if buffer is empty, false otherwise.
 */
static bool loongson_canfd_rxbuf_empty_poll(struct loongson_canfd_priv *priv)
{
	return !!regmap_test_bits(priv->regmap, LOONGSON_CANFD_RX_STAT, REG_RX_STAT_RXE);
}

/**
 * loongson_canfd_reset() - Issue soft reset to controller
 * @ndev: Network device
 */
static void loongson_canfd_reset(struct net_device *ndev)
{
	struct loongson_canfd_priv *priv = netdev_priv(ndev);

	regmap_write(priv->regmap, LOONGSON_CANFD_MODE, REG_MODE_RST);
	regmap_write(priv->regmap, LOONGSON_CANFD_MODE, REG_MODE_RXBAM | REG_MODE_BUFM);
}

/**
 * loongson_canfd_set_btr() - Set CAN bit timing registers
 * @ndev: Network device
 * @bt: Bit timing structure
 * @nominal: true for nominal timing, false for data timing
 *
 * The hardware has different field widths for PROP (7 bits) and PH1 (6 bits).
 * This function redistributes tseg1 values to fit the hardware constraints.
 */
static void loongson_canfd_set_btr(struct net_device *ndev, struct can_bittiming *bt, bool nominal)
{
	struct loongson_canfd_priv *priv = netdev_priv(ndev);
	u32 ph1_max = FIELD_MAX(REG_BTR_PH1);
	u32 prop = bt->prop_seg, phase1 = bt->phase_seg1;
	u32 btr = 0;

	/* Hardware PROP is 7 bits, PH1 is 6 bits; redistribute if PH1 exceeds max */
	if (phase1 > ph1_max) {
		prop += phase1 - ph1_max;
		phase1 = ph1_max;
	}

	if (nominal) {
		btr = FIELD_PREP(REG_BTR_PROP, prop) |
		      FIELD_PREP(REG_BTR_PH1, phase1) |
		      FIELD_PREP(REG_BTR_PH2, bt->phase_seg2) |
		      FIELD_PREP(REG_BTR_BRP, bt->brp) |
		      FIELD_PREP(REG_BTR_SJW, bt->sjw);

		regmap_write(priv->regmap, LOONGSON_CANFD_BTR_NORM, btr);
	} else {
		btr = FIELD_PREP(REG_BTR_FD_PROP, prop) |
		      FIELD_PREP(REG_BTR_FD_PH1, phase1) |
		      FIELD_PREP(REG_BTR_FD_PH2, bt->phase_seg2) |
		      FIELD_PREP(REG_BTR_FD_BRP, bt->brp) |
		      FIELD_PREP(REG_BTR_FD_SJW, bt->sjw);

		regmap_write(priv->regmap, LOONGSON_CANFD_BTR_FD, btr);
	}
}

/**
 * loongson_canfd_set_bittiming() - Set nominal bit timing
 * @ndev: Network device
 *
 * Return: 0 on success, negative on error.
 */
static int loongson_canfd_set_bittiming(struct net_device *ndev)
{
	struct loongson_canfd_priv *priv = netdev_priv(ndev);
	struct can_bittiming *bt = &priv->can.bittiming;

	loongson_canfd_set_btr(ndev, bt, true);
	return 0;
}

/**
 * loongson_canfd_set_data_bittiming() - Set data bit timing
 * @ndev: Network device
 *
 * Return: 0 on success, negative on error.
 */
static int loongson_canfd_set_data_bittiming(struct net_device *ndev)
{
	struct loongson_canfd_priv *priv = netdev_priv(ndev);
	struct can_bittiming *dbt = &priv->can.fd.data_bittiming;

	loongson_canfd_set_btr(ndev, dbt, false);
	return 0;
}

/**
 * loongson_canfd_get_auto_tdcv() - Get transmitter delay compensation value
 * @ndev: Network device
 * @tdcv: Pointer to store TDCV value
 *
 * Return: 0 on success.
 */
static int loongson_canfd_get_auto_tdcv(const struct net_device *ndev, u32 *tdcv)
{
	struct loongson_canfd_priv *priv = netdev_priv(ndev);
	u32 val;

	regmap_read(priv->regmap, LOONGSON_CANFD_TRV_DLY, &val);
	*tdcv = FIELD_GET(REG_TRV_DLY_VAL, val);

	return 0;
}

/**
 * loongson_canfd_set_secondary_sample_point() - Configure secondary sample point
 * @ndev: Network device
 *
 * Sets the SSP offset based on TDC enablement.
 *
 * Return: 0 on success, -EPERM if controller is enabled.
 */
static int loongson_canfd_set_secondary_sample_point(struct net_device *ndev)
{
	struct loongson_canfd_priv *priv = netdev_priv(ndev);
	u32 ssp_cfg;

	if (loongson_canfd_enabled(priv)) {
		netdev_err(ndev, "Cannot set SSP while CAN is enabled\n");
		return -EPERM;
	}

	if (can_fd_tdc_is_enabled(&priv->can))
		ssp_cfg = FIELD_PREP(REG_SSP_CFG_OFF, priv->can.fd.tdc.tdco) |
			  FIELD_PREP(REG_SSP_CFG_SRC, 0x0);
	else
		ssp_cfg = FIELD_PREP(REG_SSP_CFG_SRC, 0x1);

	regmap_write(priv->regmap, LOONGSON_CANFD_SSP_CFG, ssp_cfg);

	return 0;
}

/**
 * loongson_canfd_set_conf_mode() - Set mode and configuration registers
 * @priv: Driver private data
 */
static void loongson_canfd_set_conf_mode(struct loongson_canfd_priv *priv)
{
	u32 ctrlmode = priv->can.ctrlmode;
	u32 mode, conf;

	regmap_read(priv->regmap, LOONGSON_CANFD_MODE, &mode);

	if (ctrlmode & CAN_CTRLMODE_LISTENONLY)
		mode |= REG_MODE_BMM;
	else
		mode &= ~REG_MODE_BMM;

	if (ctrlmode & CAN_CTRLMODE_FD)
		mode |= REG_MODE_FDE;
	else
		mode &= ~REG_MODE_FDE;

	if (ctrlmode & CAN_CTRLMODE_PRESUME_ACK)
		mode |= REG_MODE_ACF;
	else
		mode &= ~REG_MODE_ACF;

	/* TSTM must be off during normal operation */
	mode &= ~REG_MODE_TSTM;
	regmap_write(priv->regmap, LOONGSON_CANFD_MODE, mode);

	regmap_read(priv->regmap, LOONGSON_CANFD_CONF, &conf);

	if (ctrlmode & CAN_CTRLMODE_LOOPBACK)
		conf |= REG_CONF_ILBP;
	else
		conf &= ~REG_CONF_ILBP;

	if (ctrlmode & CAN_CTRLMODE_FD_NON_ISO)
		conf |= REG_CONF_NISOFD;
	else
		conf &= ~REG_CONF_NISOFD;

	conf &= ~FIELD_PREP(REG_CONF_RTRTH, LOONGSON_CANFD_MAX_RTXTH);

	if (ctrlmode & CAN_CTRLMODE_ONE_SHOT) {
		conf |= REG_CONF_RTRLE;
		conf &= ~FIELD_PREP(REG_CONF_RTRTH, LOONGSON_CANFD_MAX_RTXTH);
	} else {
		conf |= REG_CONF_RTRLE |
			FIELD_PREP(REG_CONF_RTRTH, LOONGSON_CANFD_MAX_RTXTH);
	}

	regmap_write(priv->regmap, LOONGSON_CANFD_CONF, conf);
}

/**
 * loongson_canfd_chip_start() - Start the CAN controller
 * @ndev: Network device
 *
 * Configures bit timing, SSP, interrupt enables, and enables the controller.
 *
 * Return: 0 on success, negative error on failure.
 */
static int loongson_canfd_chip_start(struct net_device *ndev)
{
	struct loongson_canfd_priv *priv = netdev_priv(ndev);
	u16 int_ena, int_msk;
	int ret;

	ret = loongson_canfd_set_bittiming(ndev);
	if (ret < 0)
		return ret;

	ret = loongson_canfd_set_data_bittiming(ndev);
	if (ret < 0)
		return ret;

	ret = loongson_canfd_set_secondary_sample_point(ndev);
	if (ret < 0)
		return ret;

	loongson_canfd_set_conf_mode(priv);

	/* Configure interrupts */
	int_ena = REG_INT_STAT_TXBHCI | REG_INT_STAT_EWLI | REG_INT_STAT_FCSI;
	if (priv->rx_ch)
		int_ena |= REG_INT_STAT_DMADI;
	else
		int_ena |= REG_INT_STAT_RBNEI;

	if (priv->can.ctrlmode & CAN_CTRLMODE_BERR_REPORTING)
		int_ena |= REG_INT_STAT_ALI | REG_INT_STAT_BEI;

	int_msk = FIELD_GET(REG_INT_MASK_SET, ~int_ena);

	regmap_write(priv->regmap, LOONGSON_CANFD_INT_MASK, int_msk);
	regmap_write(priv->regmap, LOONGSON_CANFD_INT_ENA, int_ena);

	priv->can.state = CAN_STATE_STOPPED;

	regmap_update_bits(priv->regmap, LOONGSON_CANFD_CONF, REG_CONF_ENA, REG_CONF_ENA);

	return 0;
}

/**
 * loongson_canfd_do_set_mode() - Change operation mode
 * @ndev: Network device
 * @mode: Mode to set
 *
 * Return: 0 on success, negative error on failure.
 */
static int loongson_canfd_do_set_mode(struct net_device *ndev, enum can_mode mode)
{
	int ret;

	switch (mode) {
	case CAN_MODE_START:
		loongson_canfd_reset(ndev);

		ret = loongson_canfd_chip_start(ndev);
		if (ret < 0) {
			netdev_err(ndev, "Failed to start chip\n");
			return ret;
		}

		netif_wake_queue(ndev);
		break;
	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

/**
 * loongson_canfd_insert_frame() - Insert a frame into TX buffer
 * @priv: Driver private data
 * @cf: CAN frame to transmit
 * @buf: Buffer index (0-based)
 * @isfdf: true for CAN FD frame, false for CAN 2.0
 *
 * Return: true if frame inserted successfully, false otherwise.
 */
static bool loongson_canfd_insert_frame(struct loongson_canfd_priv *priv,
					const struct canfd_frame *cf, u8 buf, bool isfdf)
{
	u32 frame_hdr0, frame_hdr1 = 0;

	if (buf >= LOONGSON_CANFD_TXBUF_NUM)
		return false;

	if (!loongson_canfd_txbuf_is_writable(priv, buf))
		return false;

	if (cf->can_id & CAN_EFF_FLAG) {
		frame_hdr0 = cf->can_id & CAN_EFF_MASK;
		frame_hdr0 |= REG_FRAME_META0_XDT;
	} else {
		frame_hdr0 = FIELD_PREP(REG_FRAME_META0_ID_BASE, cf->can_id & CAN_SFF_MASK);
	}

	if (cf->can_id & CAN_RTR_FLAG)
		frame_hdr0 |= REG_FRAME_META0_RTR;

	if (isfdf) {
		frame_hdr1 = REG_FRAME_META1_FDF;
		if (cf->flags & CANFD_BRS)
			frame_hdr1 |= REG_FRAME_META1_BRS;
	}

	frame_hdr1 |= FIELD_PREP(REG_FRAME_META1_DLC, can_fd_len2dlc(cf->len));

	regmap_write(priv->regmap, LOONGSON_CANFD_TX_SEL, buf);

	regmap_write(priv->regmap, LOONGSON_CANFD_TX_DATA_1 + LOONGSON_CANFD_FRAME_META0,
		     frame_hdr0);
	regmap_write(priv->regmap, LOONGSON_CANFD_TX_DATA_1 + LOONGSON_CANFD_FRAME_META1,
		     frame_hdr1);

	if (cf->can_id & CAN_RTR_FLAG)
		return true;

	for (unsigned int i = 0; i < cf->len; i += LOONGSON_CANFD_DW_BYTE) {
		u32 word = get_unaligned_le32(cf->data + i);

		regmap_write(priv->regmap,
			     LOONGSON_CANFD_TX_DATA_1 + LOONGSON_CANFD_FRAME_DB_1 + i, word);
	}

	return true;
}

/**
 * loongson_canfd_start_xmit() - Transmit a CAN frame
 * @skb: Socket buffer
 * @ndev: Network device
 *
 * Return: NETDEV_TX_OK on success, NETDEV_TX_BUSY if queue full.
 */
static netdev_tx_t loongson_canfd_start_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct canfd_frame *cf = (struct canfd_frame *)skb->data;
	struct loongson_canfd_priv *priv = netdev_priv(ndev);
	u32 buf_id = LOONGSON_CANFD_TXBUF_NUM;
	unsigned long tx_brp;
	u32 tx_stat, i = 0;
	u16 tx_bs;

	if (can_dropped_invalid_skb(ndev, skb))
		return NETDEV_TX_OK;

	guard(spinlock_irqsave)(&priv->tx_lock);

	if (unlikely(!loongson_canfd_txbuf_freed(priv))) {
		netif_stop_queue(ndev);
		netdev_err(ndev, "No free TX buffer while queue was awake!\n");
		return NETDEV_TX_BUSY;
	}

	regmap_read(priv->regmap, LOONGSON_CANFD_TX_STAT, &tx_stat);
	tx_brp = FIELD_GET(REG_TX_STAT_BRP, tx_stat);
	tx_bs = FIELD_GET(REG_TX_STAT_BS, tx_stat);

	for_each_clear_bit(i, &tx_brp, LOONGSON_CANFD_TXBUF_NUM) {
		u32 transmission_complete_mask = 0x3 << (i * 2);

		if (!(tx_bs & transmission_complete_mask)) {
			buf_id = i;
			break;
		}
	}

	if (buf_id == LOONGSON_CANFD_TXBUF_NUM) {
		netif_stop_queue(ndev);
		netdev_err(ndev, "No writable TX buffer, stopping queue\n");
		return NETDEV_TX_BUSY;
	}

	if (!loongson_canfd_insert_frame(priv, cf, buf_id, can_is_canfd_skb(skb))) {
		netdev_err(ndev, "Failed to insert frame into TX buffer\n");
		dev_kfree_skb_any(skb);
		ndev->stats.tx_dropped++;
		return NETDEV_TX_OK;
	}

	can_put_echo_skb(skb, ndev, buf_id, 0);

	loongson_canfd_set_txbuf_cmd(ndev, TXT_CMD_ADD, buf_id);

	if (!loongson_canfd_txbuf_freed(priv))
		netif_stop_queue(ndev);

	return NETDEV_TX_OK;
}

/**
 * loongson_canfd_rx() - Process a received CAN frame
 * @ndev: Network device
 *
 * Reads frame metadata and data from RX buffer, constructs an skb and passes it up.
 *
 * Return: 1 if frame processed, -EAGAIN if no complete frame available.
 */
static int loongson_canfd_rx(struct net_device *ndev)
{
	struct loongson_canfd_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	u32 frame_hdr0, frame_hdr1, dlc, rwcnt, dbcnt, i;
	struct canfd_frame *cfd;
	struct can_frame *ccf;
	struct sk_buff *skb;
	bool is_can_fd;

	frame_hdr0 = priv->get_rx_data(priv);
	frame_hdr1 = priv->get_rx_data(priv);

	rwcnt = FIELD_GET(REG_FRAME_META1_RWCNT, frame_hdr1);
	if (rwcnt < 2)
		return -EAGAIN;

	dbcnt = (rwcnt - 2) * LOONGSON_CANFD_DW_BYTE;
	if (dbcnt > CANFD_MAX_DLEN)
		return -EAGAIN;

	is_can_fd = !!(frame_hdr1 & REG_FRAME_META1_FDF);
	if (is_can_fd) {
		skb = alloc_canfd_skb(ndev, &cfd);
	} else {
		skb = alloc_can_skb(ndev, &ccf);
		cfd = (struct canfd_frame *)ccf;
	}

	if (unlikely(!skb)) {
		for (i = 0; i < dbcnt; i += LOONGSON_CANFD_DW_BYTE)
			priv->get_rx_data(priv);
		stats->rx_dropped++;
		return 1;
	}

	if (frame_hdr0 & REG_FRAME_META0_XDT)
		cfd->can_id = (frame_hdr0 & CAN_EFF_MASK) | CAN_EFF_FLAG;
	else
		cfd->can_id = FIELD_GET(REG_FRAME_META0_ID_BASE, frame_hdr0) & CAN_SFF_MASK;

	cfd->flags = 0;

	if (is_can_fd) {
		if (frame_hdr1 & REG_FRAME_META1_BRS)
			cfd->flags |= CANFD_BRS;
		if (frame_hdr0 & REG_FRAME_META0_ESI)
			cfd->flags |= CANFD_ESI;
	} else if (frame_hdr0 & REG_FRAME_META0_RTR) {
		cfd->can_id |= CAN_RTR_FLAG;
	}

	dlc = FIELD_GET(REG_FRAME_META1_DLC, frame_hdr1);
	if (is_can_fd)
		cfd->len = can_fd_dlc2len(dlc);
	else
		can_frame_set_cc_len(ccf, dlc, priv->can.ctrlmode);

	for (i = 0; i < dbcnt; i += LOONGSON_CANFD_DW_BYTE)
		put_unaligned_le32(priv->get_rx_data(priv), cfd->data + i);

	while (unlikely(i < dbcnt)) {
		priv->get_rx_data(priv);
		i += LOONGSON_CANFD_DW_BYTE;
	}

	if (!(cfd->can_id & CAN_RTR_FLAG))
		stats->rx_bytes += cfd->len;
	stats->rx_packets++;

	netif_receive_skb(skb);
	return 1;
}

/**
 * loongson_canfd_get_fault_state() - Read fault confinement state
 * @ndev: Network device
 *
 * Return: Current CAN state (ACTIVE, WARNING, PASSIVE, BUS_OFF).
 */
static enum can_state loongson_canfd_get_fault_state(struct net_device *ndev)
{
	struct loongson_canfd_priv *priv = netdev_priv(ndev);
	enum can_state sts = CAN_STATE_ERROR_PASSIVE;
	u32 fstat, ewl, erl, rec_tec, max_tec;

	regmap_read(priv->regmap, LOONGSON_CANFD_FSTAT, &fstat);

	switch (FIELD_GET(REG_FSTAT_MASK, fstat)) {
	case REG_FSTAT_ERA:
		regmap_read(priv->regmap, LOONGSON_CANFD_ERL, &erl);
		regmap_read(priv->regmap, LOONGSON_CANFD_ERC, &rec_tec);

		ewl = FIELD_GET(REG_ERL_EW, erl);
		max_tec = max(FIELD_GET(REG_ERC_REC, rec_tec), FIELD_GET(REG_ERC_TEC, rec_tec));

		if (ewl > max_tec)
			sts = CAN_STATE_ERROR_ACTIVE;
		else
			sts = CAN_STATE_ERROR_WARNING;
		break;
	case REG_FSTAT_ERP:
		sts = CAN_STATE_ERROR_PASSIVE;
		break;
	case REG_FSTAT_BOF:
		sts = CAN_STATE_BUS_OFF;
		break;
	default:
		netdev_err(ndev, "Invalid error state\n");
		break;
	}

	return sts;
}

/**
 * loongson_canfd_get_bec() - Read error counters
 * @priv: Driver private data
 * @bec: Structure to fill with TX/RX error counts
 */
static void loongson_canfd_get_bec(struct loongson_canfd_priv *priv, struct can_berr_counter *bec)
{
	u32 erc;

	regmap_read(priv->regmap, LOONGSON_CANFD_ERC, &erc);
	bec->rxerr = FIELD_GET(REG_ERC_REC, erc);
	bec->txerr = FIELD_GET(REG_ERC_TEC, erc);
}

/**
 * loongson_canfd_get_berr_counter() - Get error counter (callback)
 * @ndev: Network device
 * @bec: Structure to fill
 *
 * Return: 0 always.
 */
static int loongson_canfd_get_berr_counter(const struct net_device *ndev,
					   struct can_berr_counter *bec)
{
	struct loongson_canfd_priv *priv = netdev_priv(ndev);

	loongson_canfd_get_bec(priv, bec);
	return 0;
}

/**
 * loongson_canfd_err_interrupt() - Handle error interrupts
 * @ndev: Network device
 * @isr: Interrupt status register value
 *
 * Processes error events and generates appropriate CAN error frames.
 */
static void loongson_canfd_err_interrupt(struct net_device *ndev, u32 isr)
{
	struct loongson_canfd_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	struct can_berr_counter bec;
	u32 capt, alc, mask = 0;
	enum can_state state;
	struct can_frame *cf;
	struct sk_buff *skb;

	loongson_canfd_get_bec(priv, &bec);

	state = loongson_canfd_get_fault_state(ndev);
	regmap_read(priv->regmap, LOONGSON_CANFD_ERR_CAPT, &capt);
	regmap_read(priv->regmap, LOONGSON_CANFD_ALC, &alc);

	if (net_ratelimit())
		netdev_info(ndev, "%s: ISR 0x%08x, rxerr %d, txerr %d, error type %lu, pos %lu, ALC id_field %lu, bit %lu\n",
			    __func__, isr, bec.rxerr, bec.txerr,
			    FIELD_GET(REG_ERR_CAPT_TYPE, capt), FIELD_GET(REG_ERR_CAPT_POS, capt),
			    FIELD_GET(REG_ALC_ID_FIELD, alc), FIELD_GET(REG_ALC_BIT_POS, alc));

	skb = alloc_can_err_skb(ndev, &cf);

	if ((isr & REG_INT_STAT_FCSI) || (isr & REG_INT_STAT_EWLI)) {
		netdev_info(ndev, "state changes from %s to %s\n",
			    can_get_state_str(priv->can.state), can_get_state_str(state));

		if (priv->can.state == state)
			netdev_warn(ndev, "Current and previous states are same (missed interrupt?)\n");

		mask |= REG_INT_STAT_FCSI | REG_INT_STAT_EWLI;
		priv->can.state = state;
		switch (state) {
		case CAN_STATE_BUS_OFF:
			priv->can.can_stats.bus_off++;
			can_bus_off(ndev);
			if (skb)
				cf->can_id |= CAN_ERR_BUSOFF;
			break;
		case CAN_STATE_ERROR_PASSIVE:
			priv->can.can_stats.error_passive++;
			if (skb) {
				cf->can_id |= CAN_ERR_CRTL | CAN_ERR_CNT;
				if (bec.rxerr >= CAN_ERROR_PASSIVE_THRESHOLD)
					cf->data[1] = CAN_ERR_CRTL_RX_PASSIVE;
				else
					cf->data[1] = CAN_ERR_CRTL_TX_PASSIVE;
				cf->data[6] = bec.txerr;
				cf->data[7] = bec.rxerr;
			}
			break;
		case CAN_STATE_ERROR_WARNING:
			priv->can.can_stats.error_warning++;
			if (skb) {
				cf->can_id |= CAN_ERR_CRTL | CAN_ERR_CNT;
				if (bec.txerr > bec.rxerr)
					cf->data[1] |= CAN_ERR_CRTL_TX_WARNING;
				else
					cf->data[1] |= CAN_ERR_CRTL_RX_WARNING;
				cf->data[6] = bec.txerr;
				cf->data[7] = bec.rxerr;
			}
			break;
		case CAN_STATE_ERROR_ACTIVE:
			if (skb) {
				cf->can_id |= CAN_ERR_CNT;
				cf->data[1] = CAN_ERR_CRTL_ACTIVE;
				cf->data[6] = bec.txerr;
				cf->data[7] = bec.rxerr;
			}
			break;
		default:
			netdev_err(ndev, "Unexpected state: %d\n", state);
			break;
		}
	}

	if (isr & REG_INT_STAT_ALI) {
		mask |= REG_INT_STAT_ALI;
		netdev_err(ndev, "Arbitration lost interrupt\n");
		priv->can.can_stats.arbitration_lost++;
		if (skb) {
			cf->can_id |= CAN_ERR_LOSTARB;
			cf->data[0] = CAN_ERR_LOSTARB_UNSPEC;
		}
	}

	if (isr & REG_INT_STAT_BEI) {
		mask |= REG_INT_STAT_BEI;
		netdev_err(ndev, "Bus error interrupt\n");
		priv->can.can_stats.bus_error++;
		scoped_guard(spinlock_irqsave, &priv->stats_lock) {
			stats->rx_errors++;
		}
		if (skb) {
			cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;
			cf->data[2] = CAN_ERR_PROT_UNSPEC;
			cf->data[3] = CAN_ERR_PROT_LOC_UNSPEC;
		}
	}

	if (skb)
		netif_rx(skb);

	regmap_write(priv->regmap, LOONGSON_CANFD_INT_STAT, mask);
	regmap_write(priv->regmap, LOONGSON_CANFD_INT_MASK, mask << 16);
}

/**
 * loongson_canfd_rx_napi() - NAPI poll routine for RX
 * @napi: NAPI structure
 * @quota: Maximum number of packets to process
 *
 * Return: Number of packets processed.
 */
static int loongson_canfd_rx_napi(struct napi_struct *napi, int quota)
{
	struct net_device *ndev = napi->dev;
	struct loongson_canfd_priv *priv = netdev_priv(ndev);
	bool no_more_data = false;
	int work_done = 0, ret;
	u32 sts;

	while (work_done < quota) {
		if (priv->get_rxbuf_empty(priv)) {
			no_more_data = true;
			break;
		}

		ret = loongson_canfd_rx(ndev);
		if (ret == -EAGAIN) {
			no_more_data = true;
			break;
		}

		work_done++;
	}

	regmap_read(priv->regmap, LOONGSON_CANFD_STAT, &sts);
	if (sts & REG_STAT_DOR) {
		struct net_device_stats *stats = &ndev->stats;
		struct can_frame *cf;
		struct sk_buff *skb;

		netdev_info(ndev, "RX overflow\n");
		stats->rx_over_errors++;
		scoped_guard(spinlock_irqsave, &priv->stats_lock) {
			stats->rx_errors++;
		}

		skb = alloc_can_err_skb(ndev, &cf);
		if (skb) {
			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] |= CAN_ERR_CRTL_RX_OVERFLOW;
			netif_rx(skb);
		}

		regmap_write(priv->regmap, LOONGSON_CANFD_CMD, REG_CMD_CDO);
	}

	if (no_more_data) {
		if (napi_complete_done(napi, work_done)) {
			int int_ena;

			if (priv->rx_ch)
				int_ena = REG_INT_STAT_DMADI;
			else
				int_ena = REG_INT_STAT_RBNEI;

			regmap_write(priv->regmap, LOONGSON_CANFD_INT_STAT, int_ena);
			regmap_write(priv->regmap, LOONGSON_CANFD_INT_MASK, (int_ena << 16));
		}
	}

	return work_done;
}

/**
 * loongson_canfd_tx_interrupt() - Handle TX completion interrupt
 * @ndev: Network device
 */
static void loongson_canfd_tx_interrupt(struct net_device *ndev)
{
	struct loongson_canfd_priv *priv = netdev_priv(ndev);
	struct net_device_stats *stats = &ndev->stats;
	enum loongson_canfd_txbuf_sts sts;
	bool processed = false;
	u32 tx_frame_cnt;

	guard(spinlock_irqsave)(&priv->tx_lock);

	for (unsigned int buf_id = 0; buf_id < LOONGSON_CANFD_TXBUF_NUM; buf_id++) {
		sts = loongson_canfd_get_txbuf_sts(priv, buf_id);
		if (sts == TX_BS_IDLE)
			continue;

		processed = true;
		switch (sts) {
		case TX_BS_VALID:
			stats->tx_bytes += can_get_echo_skb(ndev, buf_id, NULL);
			stats->tx_packets++;
			break;
		case TX_BS_FAIL:
			regmap_read(priv->regmap, LOONGSON_CANFD_TX_FR_CNT, &tx_frame_cnt);
			netdev_warn(ndev, "TX buffer in FAIL state, TX frame count: %d\n",
				    tx_frame_cnt);
			can_free_echo_skb(ndev, buf_id, NULL);
			stats->tx_dropped++;
			break;
		case TX_BS_CANCEL:
			netdev_warn(ndev, "TX buffer in CANCEL state\n");
			can_free_echo_skb(ndev, buf_id, NULL);
			stats->tx_dropped++;
			break;
		default:
			netdev_warn(ndev, "Unexpected TX status: %d\n", sts);
			break;
		}

		loongson_canfd_set_txbuf_cmd(ndev, TXT_CMD_SR_CLEAN, buf_id);
	}

	regmap_write(priv->regmap, LOONGSON_CANFD_INT_STAT, REG_INT_STAT_TXBHCI);

	if (!processed)
		netdev_warn_once(ndev, "Spurious TX interrupt\n");

	if (loongson_canfd_txbuf_freed(priv))
		netif_wake_queue(ndev);
}

/**
 * loongson_canfd_interrupt() - Main interrupt handler
 * @irq: IRQ number
 * @dev_id: Device identifier (net_device)
 *
 * Return: IRQ_HANDLED if interrupt was handled, IRQ_NONE otherwise.
 */
static irqreturn_t loongson_canfd_interrupt(int irq, void *dev_id)
{
	struct net_device *ndev = (struct net_device *)dev_id;
	struct loongson_canfd_priv *priv = netdev_priv(ndev);
	u32 isr, imask;

	for (unsigned int irq_loops = 0; irq_loops < 10000; irq_loops++) {
		regmap_read(priv->regmap, LOONGSON_CANFD_INT_STAT, &isr);
		if (!isr) {
			if (irq_loops)
				return IRQ_HANDLED;
			else
				return IRQ_NONE;
		}

		imask = priv->rx_ch ? REG_INT_STAT_DMADI : REG_INT_STAT_RBNEI;
		if (isr & imask) {
			regmap_write(priv->regmap, LOONGSON_CANFD_INT_MASK, imask);
			regmap_write(priv->regmap, LOONGSON_CANFD_INT_STAT, imask);
			napi_schedule(&priv->napi);
		}

		if (isr & REG_INT_STAT_TXBHCI)
			loongson_canfd_tx_interrupt(ndev);

		imask = isr & REG_INT_STAT_ERRORI;
		if (imask) {
			regmap_write(priv->regmap, LOONGSON_CANFD_INT_MASK, imask);
			regmap_write(priv->regmap, LOONGSON_CANFD_INT_STAT, imask);
			loongson_canfd_err_interrupt(ndev, isr);
		}
	}

	netdev_err(ndev, "Interrupt loop timeout, state: 0x%x\n", isr);

	if (isr & REG_INT_STAT_TXBHCI) {
		for (unsigned int i = 0; i < LOONGSON_CANFD_TXBUF_NUM; i++) {
			u32 sts = loongson_canfd_get_txbuf_sts(priv, i);

			netdev_err(ndev, "TX buf %d status: 0x%x\n", i, sts);
		}
	}

	loongson_canfd_disable_interrupts(priv);

	return IRQ_HANDLED;
}

/**
 * loongson_canfd_chip_stop() - Stop the controller
 * @ndev: Network device
 */
static void loongson_canfd_chip_stop(struct net_device *ndev)
{
	struct loongson_canfd_priv *priv = netdev_priv(ndev);

	loongson_canfd_disable_interrupts(priv);
	regmap_update_bits(priv->regmap, LOONGSON_CANFD_CONF, REG_CONF_ENA, 0);

	priv->can.state = CAN_STATE_STOPPED;
}

/**
 * loongson_canfd_open() - Open the network device
 * @ndev: Network device
 *
 * Return: 0 on success, negative error on failure.
 */
static int loongson_canfd_open(struct net_device *ndev)
{
	struct loongson_canfd_priv *priv = netdev_priv(ndev);
	int ret;

	loongson_canfd_reset(ndev);

	ret = open_candev(ndev);
	if (ret) {
		netdev_warn(ndev, "open_candev failed\n");
		return ret;
	}

	napi_enable(&priv->napi);
	ret = request_irq(ndev->irq, loongson_canfd_interrupt, IRQF_SHARED, ndev->name, ndev);
	if (ret < 0) {
		netdev_err(ndev, "IRQ request failed\n");
		goto exit_napi_disable;
	}

	ret = loongson_canfd_chip_start(ndev);
	if (ret < 0) {
		netdev_err(ndev, "Chip start failed\n");
		goto exit_free_irq;
	}

	netdev_info(ndev, "Loongson CANFD device registered\n");
	netif_start_queue(ndev);

	return 0;

exit_free_irq:
	free_irq(ndev->irq, ndev);
exit_napi_disable:
	napi_disable(&priv->napi);
	close_candev(ndev);
	return ret;
}

/**
 * loongson_canfd_close() - Close the network device
 * @ndev: Network device
 *
 * Return: 0 always.
 */
static int loongson_canfd_close(struct net_device *ndev)
{
	struct loongson_canfd_priv *priv = netdev_priv(ndev);

	netif_stop_queue(ndev);
	napi_disable(&priv->napi);
	loongson_canfd_chip_stop(ndev);
	free_irq(ndev->irq, ndev);
	close_candev(ndev);

	return 0;
}

static const struct net_device_ops loongson_canfd_netdev_ops = {
	.ndo_open       = loongson_canfd_open,
	.ndo_stop       = loongson_canfd_close,
	.ndo_start_xmit = loongson_canfd_start_xmit,
};

static const struct ethtool_ops loongson_canfd_ethtool_ops = {
	.get_ts_info = ethtool_op_get_ts_info,
};

static const struct regmap_range loongson_canfd_reg_table_wr_range[] = {
	regmap_reg_range(LOONGSON_CANFD_DEVICE_ID, LOONGSON_CANFD_CONF),
	regmap_reg_range(LOONGSON_CANFD_CMD, LOONGSON_CANFD_CMD),
	regmap_reg_range(LOONGSON_CANFD_INT_STAT, LOONGSON_CANFD_ERL),
	regmap_reg_range(LOONGSON_CANFD_CTR_PRES, LOONGSON_CANFD_CTR_PRES),
	regmap_reg_range(LOONGSON_CANFD_SSP_CFG, LOONGSON_CANFD_SSP_CFG),
	regmap_reg_range(LOONGSON_CANFD_TS, LOONGSON_CANFD_FLT_CTRL),
	regmap_reg_range(LOONGSON_CANFD_TX_CMD, LOONGSON_CANFD_TX_DATA_18),
};

static const struct regmap_range loongson_canfd_reg_table_rd_range[] = {
	regmap_reg_range(LOONGSON_CANFD_DEVICE_ID, LOONGSON_CANFD_STAT),
	regmap_reg_range(LOONGSON_CANFD_INT_STAT, LOONGSON_CANFD_BRE),
	regmap_reg_range(LOONGSON_CANFD_ERR_CAPT, LOONGSON_CANFD_TX_STAT),
	regmap_reg_range(LOONGSON_CANFD_TX_SEL, LOONGSON_CANFD_TX_DATA_18),
};

static const struct regmap_access_table loongson_canfd_reg_table_wr = {
	.yes_ranges = loongson_canfd_reg_table_wr_range,
	.n_yes_ranges = ARRAY_SIZE(loongson_canfd_reg_table_wr_range),
};

static const struct regmap_access_table loongson_canfd_reg_table_rd = {
	.yes_ranges = loongson_canfd_reg_table_rd_range,
	.n_yes_ranges = ARRAY_SIZE(loongson_canfd_reg_table_rd_range),
};

static bool loongson_canfd_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case LOONGSON_CANFD_MODE:
	case LOONGSON_CANFD_CONF:
	case LOONGSON_CANFD_STAT:
	case LOONGSON_CANFD_INT_STAT:
	case LOONGSON_CANFD_INT_ENA:
	case LOONGSON_CANFD_INT_MASK:
	case LOONGSON_CANFD_ERL:
	case LOONGSON_CANFD_FSTAT:
	case LOONGSON_CANFD_ERC:
	case LOONGSON_CANFD_ERR_CAPT:
	case LOONGSON_CANFD_ALC:
	case LOONGSON_CANFD_TRV_DLY:
	case LOONGSON_CANFD_TX_FR_CNT:
	case LOONGSON_CANFD_RX_STAT:
	case LOONGSON_CANFD_RX_DATA:
	case LOONGSON_CANFD_TX_STAT:
	case LOONGSON_CANFD_TX_SEL:
		return true;
	default:
		return false;
	};
}

static const struct regmap_config loongson_cangfd_regmap = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.wr_table	= &loongson_canfd_reg_table_wr,
	.rd_table	= &loongson_canfd_reg_table_rd,
	.volatile_reg	= loongson_canfd_volatile_reg,
	.max_register	= LOONGSON_CANFD_TX_DATA_18,
	.cache_type	= REGCACHE_MAPLE,
};

static int loongson_canfd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct loongson_canfd_priv *priv;
	struct net_device *ndev;
	struct regmap *regmap;
	struct resource *res;
	void __iomem *base;
	int ret = 0, irq;
	u32 clk_rate;

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	regmap = devm_regmap_init_mmio(dev, base, &loongson_cangfd_regmap);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	ret = device_property_read_u32(dev, "clock-frequency", &clk_rate);
	if (ret)
		return dev_err_probe(dev, ret, "Missing clock-frequency property\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ndev = alloc_candev(sizeof(*priv), LOONGSON_CANFD_TXBUF_NUM);
	if (!ndev)
		return -ENOMEM;

	priv = netdev_priv(ndev);
	spin_lock_init(&priv->tx_lock);
	spin_lock_init(&priv->stats_lock);
	priv->regmap = regmap;
	priv->res = res;

	priv->can.clock.freq = clk_rate;
	priv->can.bittiming_const = &loongson_canfd_bit_timing;
	priv->can.fd.data_bittiming_const = &loongson_canfd_bit_timing_data;
	priv->can.fd.tdc_const = &loongson_canfd_tdc_data;

	priv->can.fd.do_set_data_bittiming = loongson_canfd_set_data_bittiming;
	priv->can.fd.do_get_auto_tdcv = loongson_canfd_get_auto_tdcv;
	priv->can.do_set_mode = loongson_canfd_do_set_mode;
	priv->can.do_set_bittiming = loongson_canfd_set_bittiming;
	priv->can.do_get_berr_counter = loongson_canfd_get_berr_counter;

	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK | CAN_CTRLMODE_LISTENONLY |
				       CAN_CTRLMODE_ONE_SHOT | CAN_CTRLMODE_BERR_REPORTING |
				       CAN_CTRLMODE_FD | CAN_CTRLMODE_PRESUME_ACK |
				       CAN_CTRLMODE_FD_NON_ISO | CAN_CTRLMODE_TDC_AUTO |
				       CAN_CTRLMODE_CC_LEN8_DLC;

	ndev->irq = irq;
	ndev->flags |= IFF_ECHO;
	platform_set_drvdata(pdev, ndev);
	ndev->netdev_ops = &loongson_canfd_netdev_ops;
	ndev->ethtool_ops = &loongson_canfd_ethtool_ops;
	SET_NETDEV_DEV(ndev, dev);

	priv->get_rx_data = loongson_canfd_rxdata_poll;
	priv->get_rxbuf_empty = loongson_canfd_rxbuf_empty_poll;

	priv->rx_ch = dma_request_chan(dev, "rx");
	if (PTR_ERR(priv->rx_ch) == -EPROBE_DEFER) {
		ret = -EPROBE_DEFER;
		goto err_candev_free;
	}

	if (IS_ERR(priv->rx_ch)) {
		dev_warn(dev, "DMA not available, falling back to poll mode\n");
		priv->rx_ch = NULL;
	}

	ret = loongson_canfd_rxdma_init(ndev);
	if (!ret) {
		priv->get_rx_data = loongson_canfd_rxdata_dma;
		priv->get_rxbuf_empty = loongson_canfd_rxbuf_empty_dma;
		priv->rx_rd_ptr_dist = LOONGSON_CANFD_RXDMA_NUM;
	}
	netif_napi_add(ndev, &priv->napi, loongson_canfd_rx_napi);

	ret = register_candev(ndev);
	if (ret) {
		dev_err(dev, "register_candev failed with %d\n", ret);
		goto err_napi_del;
	}

	return 0;

err_napi_del:
	netif_napi_del(&priv->napi);
	loongson_canfd_rxdma_remove(priv, &pdev->dev);
err_candev_free:
	free_candev(ndev);
	return ret;
}

/**
 * loongson_canfd_remove() - Remove the device
 * @pdev: Platform device
 */
static void loongson_canfd_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct loongson_canfd_priv *priv = netdev_priv(ndev);

	unregister_candev(ndev);
	netif_napi_del(&priv->napi);
	loongson_canfd_rxdma_remove(priv, &pdev->dev);
	free_candev(ndev);
}

static const struct acpi_device_id loongson_canfd_acpi_match[] = {
	{ .id = "LOON0015" },
	{ }
};
MODULE_DEVICE_TABLE(acpi, loongson_canfd_acpi_match);

static struct platform_driver loongson_canfd_driver = {
	.probe  = loongson_canfd_probe,
	.remove = loongson_canfd_remove,
	.driver = {
		.name = DEV_NAME,
		.acpi_match_table = loongson_canfd_acpi_match,
	},
};
module_platform_driver(loongson_canfd_driver);

MODULE_SOFTDEP("pre: loongson2-apb-cmc-dma");
MODULE_AUTHOR("Loongson Technology Corporation Limited");
MODULE_DESCRIPTION("Loongson CAN-FD Controller driver");
MODULE_LICENSE("GPL");
