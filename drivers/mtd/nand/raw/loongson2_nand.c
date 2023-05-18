// SPDX-License-Identifier: GPL-2.0

#include <linux/module.h>
#include <linux/init.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/partitions.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

#include <asm/dma.h>

#define ALIGN_DMA(x)	((x + 3)/4)
#define REG(reg)	(info->mmio_base + reg)

#define MAX_BUFF_SIZE		0x10000
#define STATUS_TIME_LOOP_R	300
#define STATUS_TIME_LOOP_WS	100
#define STATUS_TIME_LOOP_WM	60
#define STATUS_TIME_LOOP_E	10000

/* Register offset */
#define NAND_CMD_REG		0x00
#define NAND_ADDRC_REG		0x04
#define NAND_ADDRR_REG		0x08
#define NAND_TIM_REG		0x0c
#define NAND_IDL_REG		0x10
#define NAND_IDH_REG		0x14
#define NAND_STA_REG		0x14
#define NAND_PARAM_REG		0x18
#define NAND_OP_NUM_REG		0x1c
#define NAND_CS_RDY_REG		0x20
#define NAND_DMA_ADDR_REG	0x40

/* NAND_CMD_REG */
#define CMD_VALID		(1 << 0)	/* command valid */
#define CMD_RD_OP		(1 << 1)	/* read operation */
#define CMD_WR_OP		(1 << 2)	/* write operation */
#define CMD_ER_OP		(1 << 3)	/* erase operation */
#define CMD_BER_OP		(1 << 4)	/* blocks erase operation */
#define CMD_RD_ID		(1 << 5)	/* read id */
#define CMD_RESET		(1 << 6)	/* reset */
#define CMD_RD_STATUS		(1 << 7)	/* read status */
#define CMD_MAIN		(1 << 8)	/* operation in main region */
#define CMD_SPARE		(1 << 9)	/* operation in spare region */
#define CMD_DONE		(1 << 10)	/* operation done */
#define CMD_RS_RD		(1 << 11)	/* ecc is enable when reading */
#define CMD_RS_WR		(1 << 12)	/* ecc is enable when writing */
#define CMD_INT_EN		(1 << 13)	/* interrupt enable */
#define CMD_WAIT_RS		(1 << 14)	/* waiting ecc read done */
#define CMD_ECC_DMA_REQ		(1 << 30)	/* dma request in ecc mode */
#define CMD_DMA_REQ		(1 << 31)	/* dma request in normal mode */
#define CMD_RDY_SHIF		16		/* four bits for chip ready */
#define CMD_CE_SHIF		20		/* four bits for chip enable */

/* NAND_PARAM_REG */
#define CHIP_CAP_SHIFT		8
#define ID_NUM_SHIFT		12
#define OP_SCOPE_SHIFT		16
/* DMA COMMAND REG */
#define DMA_INT_MASK		(1 << 0)	/* dma interrupt mask */
#define DMA_INT			(1 << 1)	/* dma interrupt */
#define DMA_SIN_TR_OVER		(1 << 2)	/* a single dma transfer over */
#define DMA_TR_OVER		(1 << 3)	/* all dma transfer over */
#define DMA_RD_WR		(1 << 12)	/* dma operation type */
#define DMA_RD_STU_SHIF		4		/* dma read data status */
#define DMA_WR_STU_SHIF		8		/* dma write data status */

int parse_mtd_partitions(struct mtd_info *master, const char *const *types,
			 struct mtd_partition **pparts,
			 struct mtd_part_parser_data *data);
/* DMA Descripter */
struct loongson2_nand_dma_desc {
	uint32_t orderad;
	uint32_t saddr;
	uint32_t daddr;
	uint32_t length;
	uint32_t step_length;
	uint32_t step_times;
	uint32_t cmd;
	uint32_t dummy;
	uint32_t order_addr_hi;
	uint32_t saddr_hi;
};

struct loongson2_nand_plat_data {
	int	enable_arbiter;
	struct	mtd_partition *parts;
	int	cs;
	u32	csrdy;
	int	chip_cap;
};

struct loongson2_nand_info {
	struct nand_chip	nand_chip;
	struct platform_device	*pdev;
	spinlock_t		nand_lock;

	/* MTD data control */
	unsigned int		buf_start;
	unsigned int		buf_count;

	void __iomem		*mmio_base;
	unsigned int		irq;
	unsigned int		cmd;

	/* DMA information */
	u64			dma_order_reg;	/* dma controller register */
	unsigned int		apb_data_addr;	/* dma access this address */
	u64			desc_addr;	/* dma descriptor address */
	dma_addr_t		desc_addr_phys;
	size_t			desc_size;
	u64			dma_ask;
	dma_addr_t		dma_ask_phy;

	struct dma_chan *dmac;

	unsigned char		*data_buff;	/* dma data buffer */
	dma_addr_t		data_buff_phys;
	size_t			data_buff_size;

	struct timer_list	test_timer;

	size_t			data_size;	/* data size in FIFO */
	struct completion	cmd_complete;
	unsigned int		seqin_column;
	unsigned int		seqin_page_addr;
	u32			chip_version;
	int			cs, cs0;
	u32			csrdy;
	int			chip_cap;
};
static unsigned int bch = 4;
module_param(bch,	     uint, 0400);
MODULE_PARM_DESC(bch,		 "Enable BCH ecc and set how many bits should "
				 "be correctable in 512-byte blocks");

static void wait_nand_done(struct loongson2_nand_info *info, int timeout);
static int loongson2_nand_init_buff(struct loongson2_nand_info *info)
{
	struct platform_device *pdev = info->pdev;

	info->data_buff = dma_alloc_coherent(&pdev->dev, MAX_BUFF_SIZE,
					     &info->data_buff_phys, GFP_KERNEL);

	pr_info("%s, Binbin: info->data_buff= 0x%px, info->data_buff_phys = 0x%llx\n", __func__, info->data_buff, info->data_buff_phys);
	if (info->data_buff == NULL) {
		dev_err(&pdev->dev, "failed to allocate dma buffer\n");
		return -ENOMEM;
	}

	info->data_buff_size = MAX_BUFF_SIZE;
	return 0;
}

static int loongson2_nand_ecc_calculate(struct nand_chip *chip,
				const uint8_t *dat, uint8_t *ecc_code)
{
	return 0;
}

static int loongson2_nand_ecc_correct(struct nand_chip *chip,
				uint8_t *dat, uint8_t *read_ecc,
				uint8_t *calc_ecc)
{
	/*
	 * Any error include ERR_SEND_CMD, ERR_DBERR, ERR_BUSERR, we
	 * consider it as a ecc error which will tell the caller the
	 * read fail We have distinguish all the errors, but the
	 * nand_read_ecc only check this function return value
	 */
	return 0;
}

static void loongson2_nand_ecc_hwctl(struct nand_chip *chip, int mode)
{

}

static int loongson2_nand_get_ready(struct mtd_info *mtd)
{
	struct loongson2_nand_info *info = mtd->priv;
	unsigned char status;

	writel(CMD_RD_STATUS | CMD_VALID, REG(NAND_CMD_REG));
	wait_nand_done(info, STATUS_TIME_LOOP_R);
	status = readl(REG(NAND_IDH_REG))>>16;
	return status;
}

static int loongson2_nand_waitfunc(struct nand_chip *chip)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct loongson2_nand_info *info = mtd->priv;
	unsigned char status;

	status = readl(REG(NAND_IDH_REG))>>16;
	return status;
}

static void loongson2_nand_select_chip(struct nand_chip *chip, int cs)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct loongson2_nand_info *info = mtd->priv;

	info->cs = (cs == -1) ? info->cs0 : info->cs0 + cs;
}

static int loongson2_nand_dev_ready(struct nand_chip *chip)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct loongson2_nand_info *info = mtd->priv;

	return	!!(readl(REG(NAND_CMD_REG)) & (1<<(info->cs+16)));
}

static const char cap2cs[16] = {[0] = 16, [1]  = 17, [2]  = 18, [3] = 19,
				[4] = 19, [5]  = 19, [6]  = 20, [7] = 21,
				[9] = 14, [10] = 15, [11] = 16, [12] = 17,
				[13] = 18};

static void nand_setup(struct loongson2_nand_info *info,
		int cmd, int addr_c, int addr_r, int param, int op_num)
{
	unsigned int addr_cs = info->cs*(1UL<<cap2cs[(param>>CHIP_CAP_SHIFT)&0xf]);

	writel(param, REG(NAND_PARAM_REG));
	writel(op_num, REG(NAND_OP_NUM_REG));
	writel(addr_c, REG(NAND_ADDRC_REG));
	writel(addr_r|addr_cs, REG(NAND_ADDRR_REG));
	writel(0, REG(NAND_CMD_REG));
	writel(0, REG(NAND_CMD_REG));
	writel(cmd, REG(NAND_CMD_REG));
}

static void wait_nand_done(struct loongson2_nand_info *info, int timeout)
{
	int t, status_times = timeout;

	do {
		t = readl(REG(NAND_CMD_REG)) & CMD_DONE;
		if (!(status_times--)) {
			writel(0x0, REG(NAND_CMD_REG));
			writel(0x0, REG(NAND_CMD_REG));
			writel(CMD_RESET | CMD_VALID, REG(NAND_CMD_REG));
			break;
		}
		udelay(50);
	} while (t == 0);

	writel(0x0, REG(NAND_CMD_REG));
}

struct scatterlist sg;

static void loongson2_nand_dma_init(struct loongson2_nand_info *info, struct device *dev)
{
	struct dma_slave_config cfg = {0};
	int ret;

	info->dmac = dma_request_chan(dev, "nand_rw");
	if (IS_ERR(info->dmac))
		return;

	cfg.src_addr = info->apb_data_addr;
	cfg.dst_addr = info->apb_data_addr;
	cfg.src_addr_width = DMA_SLAVE_BUSWIDTH_8_BYTES;
	cfg.dst_addr_width = DMA_SLAVE_BUSWIDTH_8_BYTES;
	cfg.src_maxburst = 4;
	cfg.dst_maxburst = 4;

	ret = dmaengine_slave_config(info->dmac, &cfg);
	if (ret < 0) {
		pr_info("slave config error\n");
		return ;
	}
}

void dma_desc_init(struct loongson2_nand_info *info)
{
	volatile struct loongson2_nand_dma_desc *dma_base =
		(volatile struct loongson2_nand_dma_desc *)(info->desc_addr);

	dma_base->orderad = 0;
	dma_base->saddr = info->data_buff_phys;
	dma_base->daddr = info->apb_data_addr;
	dma_base->step_length = 0;
	dma_base->step_times = 0x1;
	dma_base->length = 0;
	dma_base->cmd = 0;
	dma_base->order_addr_hi = 0;
	dma_base->saddr_hi = ((info->data_buff_phys) >> 32);

}

static void ls2x_dma_complete_func(void *completion)
{
	struct loongson2_nand_info *info = completion;

	//dmaengine_terminate_async(info->dmac);
	complete(&info->cmd_complete);
}

static void dma_setup(struct loongson2_nand_info *info, int dma_cmd, int dma_cnt)
{
	int ret;
	enum dma_transfer_direction tdir;
	enum dma_data_direction dir;
	struct dma_async_tx_descriptor *dmad;
	dma_cookie_t dmat;
	struct device *dev = &info->pdev->dev;

	if (dma_cmd & DMA_RD_WR) {
		dir = DMA_TO_DEVICE;
		tdir = DMA_MEM_TO_DEV;
	} else {
		dir = DMA_FROM_DEVICE;
		tdir = DMA_DEV_TO_MEM;
	}

	sg_init_one(&sg, info->data_buff, dma_cnt);
	ret = dma_map_sg(dev, &sg, 1, dir);
	if (!ret) {
		pr_info("%s, dma_map_sg failed\n", __func__);
		return;
	}

	dmad = dmaengine_prep_slave_sg(info->dmac, &sg, 1, tdir,
				       DMA_PREP_INTERRUPT);

	dmad->callback = ls2x_dma_complete_func;
	dmad->callback_param = info;

	dmat = dmaengine_submit(dmad);

	ret = dma_submit_error(dmat);
	if (ret)
		goto err_unmap_buf;

	init_completion(&info->cmd_complete);
	dma_async_issue_pending(info->dmac);

	wait_nand_done(info, STATUS_TIME_LOOP_R);

	return;

err_unmap_buf:
	dma_unmap_sg(dev, &sg, 1, dir);
	return;
}

static int get_chip_capa_num(uint64_t  chipsize, int pagesize)
{
	int size_mb = chipsize >> 20;

	if (pagesize == 4096)
		return 4;
	else if (pagesize == 2048)
		switch (size_mb) {
		case (1 << 7):		/* 1Gb */
			return 0;
		case (1 << 8):		/* 2Gb */
			return 1;
		case (1 << 9):		/* 4Gb */
			return 2;
		case (1 << 10):		/* 8Gb */
		default:
			return 3;
		}
	else if (pagesize == 8192)

		switch (size_mb) {
		case (1 << 12):		/* 32Gb */
			return 5;
		case (1 << 13):		/* 64Gb */
			return 6;
		case (1 << 14):		/* 128Gb */
		default:
			return 7;
		}
	else if (pagesize == 512)

		switch (size_mb) {
		case (1 << 3):		/* 64Mb */
			return 9;
		case (1 << 4):		/* 128Mb */
			return 0xa;
		case (1 << 5):		/* 256Mb */
			return 0xb;
		case (1 << 6):		/* 512Mb */
		default:
			return 0xc;
		}
	else
		return 0;
}

static void ls_read_id(struct loongson2_nand_info *info)
{
	unsigned int id_l, id_h;
	unsigned char *data = (unsigned char *)(info->data_buff);
	unsigned int addr_cs, chip_cap;
	struct mtd_info *mtd = nand_to_mtd(&info->nand_chip);

	if (mtd->writesize) {
		chip_cap = get_chip_capa_num(nanddev_target_size(&info->nand_chip.base), mtd->writesize);
		addr_cs = info->cs*(1UL<<cap2cs[chip_cap]);
		info->chip_cap  = chip_cap;
	} else  {
		addr_cs = info->cs*(1UL<<cap2cs[info->chip_cap]);
	}

	writel((6 << ID_NUM_SHIFT) | (info->chip_cap << CHIP_CAP_SHIFT), REG(NAND_PARAM_REG));
	writel(addr_cs, REG(NAND_ADDRR_REG));
	writel((CMD_RD_ID | CMD_VALID), REG(NAND_CMD_REG));
	wait_nand_done(info, 100);
	id_l = readl(REG(NAND_IDL_REG));
	id_h = readl(REG(NAND_IDH_REG));
	pr_debug("id_l: %08x, id_h:%08x\n", id_l, id_h);
	data[0] = ((id_h >> 8) & 0xff);
	data[1] = (id_h & 0xff);
	data[2] = (id_l >> 24) & 0xff;
	data[3] = (id_l >> 16) & 0xff;
	data[4] = (id_l >> 8) & 0xff;
	data[5] = id_l & 0xff;
	data[6] = 0;
	data[7] = 0;
}

static void loongson2_nand_cmdfunc(struct nand_chip *chip, unsigned command,
			    int column, int page_addr)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct loongson2_nand_info *info = mtd->priv;
	int chip_cap, oobsize, pagesize;
	int cmd, addrc, addrr, op_num, param;
	int dma_cmd, dma_cnt;
	unsigned long flags;

	info->cmd = command;
	oobsize = mtd->oobsize;
	pagesize = mtd->writesize;
	chip_cap = get_chip_capa_num(nanddev_target_size(&info->nand_chip.base), pagesize);
	spin_lock_irqsave(&info->nand_lock, flags);
	switch (command) {
	case NAND_CMD_READOOB:
		info->buf_count = oobsize;
		if (info->buf_count <= 0)
			break;
		info->buf_start = 0;
		addrc = pagesize;
		addrr = page_addr;
		param = (oobsize << OP_SCOPE_SHIFT)
			| (chip_cap << CHIP_CAP_SHIFT);
		op_num = oobsize;
		cmd = CMD_VALID | CMD_SPARE | CMD_RD_OP;
		nand_setup(info, cmd, addrc, addrr, param, op_num);

		dma_cnt = op_num;
		dma_cmd = DMA_INT_MASK;
		dma_setup(info, dma_cmd, dma_cnt);
		break;
	case NAND_CMD_READ0:
		addrc = 0;
		addrr = page_addr;
		op_num = oobsize + pagesize;
		param = (op_num << OP_SCOPE_SHIFT) | (chip_cap << CHIP_CAP_SHIFT);
		cmd = CMD_VALID | CMD_SPARE | CMD_RD_OP | CMD_MAIN;
		info->buf_count = op_num;
		info->buf_start = column;
		nand_setup(info, cmd, addrc, addrr, param, op_num);

		dma_cnt = op_num;
		dma_cmd = DMA_INT_MASK;
		dma_setup(info, dma_cmd, dma_cnt);
		break;
	case NAND_CMD_SEQIN:
		info->buf_count = oobsize + pagesize - column;
		info->buf_start = 0;
		info->seqin_column = column;
		info->seqin_page_addr = page_addr;
		break;
	case NAND_CMD_PAGEPROG:
		addrc = info->seqin_column;
		addrr = info->seqin_page_addr;
		op_num = info->buf_start;
		param = ((pagesize + oobsize) << OP_SCOPE_SHIFT)
			| (chip_cap << CHIP_CAP_SHIFT);
		cmd = CMD_VALID | CMD_SPARE | CMD_WR_OP;
		if (addrc < pagesize)
			cmd |= CMD_MAIN;
		nand_setup(info, cmd, addrc, addrr, param, op_num);

		dma_cnt = op_num;
		dma_cmd = DMA_INT_MASK | DMA_RD_WR;
		dma_setup(info, dma_cmd, dma_cnt);
		break;
	case NAND_CMD_RESET:
		nand_setup(info, (CMD_RESET | CMD_VALID), 0, 0, 0, 0);
		wait_nand_done(info, STATUS_TIME_LOOP_R);
		break;
	case NAND_CMD_ERASE1:
		addrc = 0;
		addrr = page_addr;
		op_num = 1;
		param = ((pagesize + oobsize) << OP_SCOPE_SHIFT)
			| (chip_cap << CHIP_CAP_SHIFT);
		cmd = CMD_ER_OP | CMD_VALID;
		nand_setup(info, cmd, addrc, addrr, param, op_num);
		wait_nand_done(info, STATUS_TIME_LOOP_E);
		break;
	case NAND_CMD_STATUS:
		info->buf_count = 0x1;
		info->buf_start = 0x0;
		*(unsigned char *)info->data_buff =
			loongson2_nand_get_ready(mtd);
		break;
	case NAND_CMD_READID:
		info->buf_count = 0x6;
		info->buf_start = 0;
		ls_read_id(info);
		break;
	case NAND_CMD_ERASE2:
	case NAND_CMD_READ1:
		break;
	case NAND_CMD_RNDOUT:
		info->buf_start = column;
		break;
	default:
		printk(KERN_ERR "non-supported command.\n");
		break;
	}

	spin_unlock_irqrestore(&info->nand_lock, flags);
}

static uint8_t loongson2_nand_read_byte(struct nand_chip *chip)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct loongson2_nand_info *info = mtd->priv;
	unsigned long flags;
	char retval = 0xFF;

	spin_lock_irqsave(&info->nand_lock, flags);

	if (info->buf_start < info->buf_count)
		retval = info->data_buff[(info->buf_start)++];

	spin_unlock_irqrestore(&info->nand_lock, flags);
	return retval;
}

static void loongson2_nand_read_buf(struct nand_chip *chip, u8 *buf, int len)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct loongson2_nand_info *info = mtd->priv;
	int real_len = min_t(size_t, len, info->buf_count - info->buf_start);
	unsigned long flags;

	spin_lock_irqsave(&info->nand_lock, flags);

	memcpy(buf, info->data_buff + info->buf_start, real_len);

	info->buf_start += real_len;
	spin_unlock_irqrestore(&info->nand_lock, flags);
}

static void loongson2_nand_write_buf(struct nand_chip *chip, const u8 *buf, int len)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct loongson2_nand_info *info = mtd->priv;
	int real_len = min_t(size_t, len, info->buf_count - info->buf_start);
	unsigned long flags;

	spin_lock_irqsave(&info->nand_lock, flags);

	memcpy(info->data_buff + info->buf_start, buf, real_len);
	info->buf_start += real_len;

	spin_unlock_irqrestore(&info->nand_lock, flags);
}

static void loongson2_nand_init_mtd(struct mtd_info *mtd,
			       struct loongson2_nand_info *info)
{
	struct nand_chip *this = &info->nand_chip;

	this->options		= 8;
	this->legacy.select_chip	= loongson2_nand_select_chip;
	this->legacy.waitfunc		= loongson2_nand_waitfunc;
	this->legacy.dev_ready		= loongson2_nand_dev_ready;
	this->legacy.cmdfunc		= loongson2_nand_cmdfunc;
	this->legacy.read_byte		= loongson2_nand_read_byte;
	this->legacy.read_buf		= loongson2_nand_read_buf;
	this->legacy.write_buf		= loongson2_nand_write_buf;

	this->ecc.engine_type = NAND_ECC_ENGINE_TYPE_SOFT;
	this->ecc.algo		= NAND_ECC_ALGO_BCH;
	this->ecc.hwctl		= loongson2_nand_ecc_hwctl;
	this->ecc.calculate	= loongson2_nand_ecc_calculate;
	this->ecc.correct	= loongson2_nand_ecc_correct;
	this->ecc.size		= 256;
	this->ecc.bytes		= 3;
	mtd->owner = THIS_MODULE;
}

#if 0
static void test_handler(struct timer_list *t)
{
	u64 dma_order, val;
	struct loongson2_nand_info *info = from_timer(info, t, test_timer);

	mod_timer(&info->test_timer, jiffies + 1);
	val = (info->dma_ask_phy & ~0x1fUL) | 0x4;

	dma_order = (readq((void *)info->dma_order_reg) & 0x1fUL) | val;
	writeq(dma_order, (void *)info->dma_order_reg);
	udelay(1000);
}
#endif

static void loongson2_nand_init_info(struct loongson2_nand_info *info)
{
	info->buf_start = 0;
	info->buf_count = 0;
	info->seqin_column = 0;
	info->seqin_page_addr = 0;
	spin_lock_init(&info->nand_lock);
	writel(0x412, REG(NAND_TIM_REG));
	writel(info->csrdy, REG(NAND_CS_RDY_REG));

	info->test_timer.expires = jiffies + 10;
	//timer_setup(&info->test_timer, test_handler, 0);
}

#if 0
static irqreturn_t ls2x_nand_irq(int irq, void *dev)
{
	struct loongson2_nand_info *info = dev;

	writel(CMD_INT_EN, REG(NAND_CMD_REG));

	return IRQ_HANDLED;
}
#endif

static int ls2x_apbdma_config(struct device *dev)
{
	u32 conf_args[2];
	struct regmap *regmap;

	regmap = syscon_regmap_lookup_by_phandle_args(dev->of_node,
						      "loongson,apbdma-conf", 2,
						      conf_args);
	if (IS_ERR(regmap))
		return IS_ERR(regmap);

	return regmap_update_bits(regmap, 0, conf_args[0], conf_args[1]);
}

static int loongson2_nand_probe(struct platform_device *pdev)
{
	struct loongson2_nand_plat_data *pdata;
	struct loongson2_nand_info *info;
	struct nand_chip *this;
	struct mtd_info *mtd;
	struct resource *r;
	int ret = 0;
	int data;
#ifdef CONFIG_MTD_CMDLINE_PARTS
	const char *part_probes[] = { "cmdlinepart", NULL };
	struct mtd_partition *partitions = NULL;
	int num_partitions = 0;
#endif

	__be32 *of_property = NULL;

	/* get DMA parameters from controller */
	ret = ls2x_apbdma_config(&pdev->dev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "ls2x_dma_parse_of failed.\n");

	dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));

	pdata = devm_kzalloc(&pdev->dev, sizeof(struct loongson2_nand_plat_data), GFP_KERNEL);
	if (!pdata) {
		dev_err(&pdev->dev, "fialed to allocate memory\n");
		return -ENODEV;
	}

	pdata->cs = 2;
	pdata->csrdy = 0x88442200;
	pdata->enable_arbiter = 1;

	if (pdev->dev.of_node) {
		if (!of_property_read_u32(pdev->dev.of_node, "nand-cs", &data))
			pdata->cs = data;

		if (!of_property_read_u32(pdev->dev.of_node, "chip_cap", &data))
			pdata->chip_cap  = data;
	}

	info = devm_kzalloc(&pdev->dev, sizeof(struct loongson2_nand_info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "fialed to allocate memory\n");
		return -ENOMEM;
	}

	info->pdev = pdev;
	info->cs0 = info->cs = pdata->cs;
	info->csrdy = pdata->csrdy;
	info->chip_cap = pdata->chip_cap;

	this = &info->nand_chip;
	mtd = nand_to_mtd(&info->nand_chip);
	mtd->priv = info;

	info->mmio_base = devm_platform_get_and_ioremap_resource(pdev, 0, &r);
	if (info->mmio_base == NULL) {
		dev_err(&pdev->dev, "ioremap() failed\n");
		ret = -ENODEV;
		goto fail_free_res;
	}

	info->apb_data_addr = r->start;
	pr_info("%s, Binbin: info->apb_data_addr= %x\n", __func__, info->apb_data_addr);

	//chan = dma_request_chan(&pdev->dev, "nand_rw");
	//if (chan == NULL) {
	//	dev_err(&pdev->dev, "no nand APBDMA resource defined\n");
	//	return -ENODEV;
	//}

	//ret = of_property_read_u32(pdev->dev.of_node, "#address-cells", &data);
	//if (ret) {
	//	dev_err(&pdev->dev, "missing #address-cells property\n");
	//	data = 1;
	//}

	//if (data == 2) {
	//	of_property = (__be32 *)of_get_property(chan->device->dev->of_node, "reg", NULL);
	//	if (of_property != 0)
	//		r->start = of_read_number(of_property, 2);
	//} else {
	//	of_property_read_u32(chan->device->dev->of_node, "reg", &data);
	//	r->start = data;
	//}

	//info->dma_order_reg = (u64)ioremap(r->start, 8);
	//pr_info("%s, Binbin: info->dma_order_reg = %llx, r->start = %llx\n", __func__, info->dma_order_reg, r->start);


	ret = loongson2_nand_init_buff(info);
	if (ret)
		goto fail_free_io;

	//irq = platform_get_irq(pdev, 0);
	//if (irq < 0) {
	//	dev_err(&pdev->dev, "no IRQ resource defined\n");
	//	ret = -ENXIO;
	//	goto fail_free_io;
	//}
	//info->irq = irq;

	//ret = devm_request_irq(&pdev->dev, info->irq, ls2x_nand_irq, IRQF_TRIGGER_RISING, "ls2x nand", info);
	//if (ret)
	//	return ret;

	loongson2_nand_init_mtd(mtd, info);

	loongson2_nand_init_info(info);
	//dma_desc_init(info);
	loongson2_nand_dma_init(info, &pdev->dev);
	platform_set_drvdata(pdev, mtd);

	if (bch) {
		this->ecc.engine_type = NAND_ECC_ENGINE_TYPE_SOFT;
		this->ecc.algo = NAND_ECC_ALGO_BCH;
		this->ecc.size = 512;
		this->ecc.strength = bch;
		pr_info("using %u-bit/%u bytes BCH ECC\n", bch, this->ecc.size);
	}

	if (nand_scan(this, 4)) {
		dev_err(&pdev->dev, "failed to scan nand\n");
		ret = -ENXIO;
		goto fail_free_io;
	}

#ifdef CONFIG_MTD_CMDLINE_PARTS
	mtd->name = "nand-flash";
	num_partitions = parse_mtd_partitions(mtd, part_probes, &partitions, 0);
#endif
	ret = mtd_device_parse_register(mtd, NULL, NULL, NULL, 0);

	return ret;

fail_free_io:
fail_free_res:
fail_free_buf:
	dma_free_coherent(&pdev->dev, info->data_buff_size,
			  info->data_buff, info->data_buff_phys);
	return ret;
}

static void loongson2_nand_remove(struct platform_device *pdev)
{
	struct mtd_info *mtd = platform_get_drvdata(pdev);
	struct loongson2_nand_info *info = mtd->priv;

	platform_set_drvdata(pdev, NULL);

	mtd_device_unregister(mtd);
	kfree((void *)info->desc_addr);
}

static const struct of_device_id loongson2_nand_of_ids[] = {
	{ .compatible = "loongson,ls2k1000-nand", },
	{},
};
MODULE_DEVICE_TABLE(of, loongson2_nand_of_ids);

static struct platform_driver loongson2_nand_driver = {
	.driver	= {
		.name = "loongson2-nand",
		.of_match_table = loongson2_nand_of_ids,
	},
	.probe	= loongson2_nand_probe,
	.remove	= loongson2_nand_remove,
};

module_platform_driver(loongson2_nand_driver);

MODULE_DESCRIPTION("Loongson-2K NAND controller driver");
MODULE_AUTHOR("Loongson Technology Corporation Limited");
MODULE_LICENSE("GPL");
