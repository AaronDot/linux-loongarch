// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for Loongson-2K BMC IPMI
 *
 * Copyright (C) 2024 Loongson Technology Corporation Limited.
 *
 * Originally written by Chong Qiao <qiaochong@loongson.cn>
 * Rewritten for mainline by Binbin Zhou <zhoubinbin@loongson.cn>
 */

#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/types.h>

#include "ipmi_si.h"

#define LS2K_KCS_STS_OBF	BIT(0)
#define LS2K_KCS_STS_IBF	BIT(1)
#define LS2K_KCS_STS_SMS_ATN	BIT(2)
#define LS2K_KCS_STS_CMD	BIT(3)

#define LS2K_KCS_DATA_MASK	(LS2K_KCS_STS_OBF | LS2K_KCS_STS_IBF | LS2K_KCS_STS_CMD)

/* Read and write fifo pointers for data consistency. */
struct ls2k_fifo_flag {
	u8 ibfh;
	u8 ibft;
	u8 obfh;
	u8 obft;
};

struct ls2k_kcs_reg {
	u8 status;
	u8 data_out;
	s16 data_in;
	s16 cmd;
};

struct ls2k_kcs_data {
	struct ls2k_fifo_flag fifo;
	struct ls2k_kcs_reg reg;
	u8 cmd_data;
	u8 version;
	u32 write_req;
	u32 write_ack;
	u32 reserved[2];
};

static void ls2k_set_obf(struct ls2k_kcs_data *ik, u8 sts)
{
	ik->reg.status = (ik->reg.status & ~LS2K_KCS_STS_OBF) | (sts & BIT(0));
}

static void ls2k_set_ibf(struct ls2k_kcs_data *ik, u8 sts)
{
	ik->reg.status = (ik->reg.status & ~LS2K_KCS_STS_IBF) | ((sts & BIT(0)) << 1);
}

static u8 ls2k_get_ibf(struct ls2k_kcs_data *ik)
{
	return (ik->reg.status >> 1) & BIT(0);
}

static unsigned char intf_sim_inb_v0(struct ls2k_kcs_data *ik,
				     unsigned int offset)
{
	u32 inb = 0;

	switch (offset & BIT(0)) {
	case 0:
		inb = ik->reg.data_out;
		ls2k_set_obf(ik, 0);
		break;
	case 1:
		inb = ik->reg.status;
		break;
	}

	return inb;
}

static unsigned char intf_sim_inb_v1(struct ls2k_kcs_data *ik,
				     unsigned int offset)
{
	u32 inb = 0;
	int cmd;
	bool obf, ibf;

	obf = ik->fifo.obfh != ik->fifo.obft;
	ibf = ik->fifo.ibfh != ik->fifo.ibft;
	cmd = ik->cmd_data;

	switch (offset & BIT(0)) {
	case 0:
		inb = ik->reg.data_out;
		ik->fifo.obft = ik->fifo.obfh;
		break;
	case 1:
		inb = ik->reg.status & ~LS2K_KCS_DATA_MASK;
		inb |= obf | (ibf << 1) | (cmd << 3);
		break;
	}

	return inb;
}

static unsigned char ls2k_mem_inb(const struct si_sm_io *io,
				  unsigned int offset)
{
	struct ls2k_kcs_data *ik = io->addr;
	int inb = 0;

	if (ik->version == 0)
		inb = intf_sim_inb_v0(ik, offset);
	else if (ik->version == 1)
		inb = intf_sim_inb_v1(ik, offset);

	return inb;
}

static void intf_sim_outb_v0(struct ls2k_kcs_data *ik, unsigned int offset,
			     unsigned char val)
{
	if (ls2k_get_ibf(ik))
		return;

	switch (offset & BIT(0)) {
	case 0:
		ik->reg.data_in = val;
		ik->reg.status &= ~LS2K_KCS_STS_CMD;
		break;

	case 1:
		ik->reg.cmd = val;
		ik->reg.status |= LS2K_KCS_STS_CMD;
		break;
	}

	ls2k_set_ibf(ik, 1);
	ik->write_req++;
}

static void intf_sim_outb_v1(struct ls2k_kcs_data *ik, unsigned int offset,
			     unsigned char val)
{
	if (ik->fifo.ibfh != ik->fifo.ibft)
		return;

	switch (offset & BIT(0)) {
	case 0:
		ik->reg.data_in = val;
		ik->cmd_data = 0;
		break;

	case 1:
		ik->reg.cmd = val;
		ik->cmd_data = 1;
		break;
	}

	ik->fifo.ibfh = !ik->fifo.ibft;
	ik->write_req++;
}

static void ls2k_mem_outb(const struct si_sm_io *io, unsigned int offset,
			  unsigned char val)
{
	struct ls2k_kcs_data *ik = io->addr;

	if (ik->version == 0)
		intf_sim_outb_v0(ik, offset, val);
	else if (ik->version == 1)
		intf_sim_outb_v1(ik, offset, val);
}

static void ls2k_mem_cleanup(struct si_sm_io *io)
{
	if (io->addr)
		iounmap(io->addr);
}

static int ipmi_ls2k_sim_setup(struct si_sm_io *io)
{
	io->addr = ioremap(io->addr_data, io->regspacing);
	if (!io->addr)
		return -EIO;

	io->inputb = ls2k_mem_inb;
	io->outputb = ls2k_mem_outb;
	io->io_cleanup = ls2k_mem_cleanup;

	return 0;
}

static int ipmi_ls2k_probe(struct platform_device *pdev)
{
	struct si_sm_io io;

	dev_info(&pdev->dev, "probing via ls2k platform");
	memset(&io, 0, sizeof(io));

	io.addr_source	= SI_PLATFORM;
	io.si_type	= SI_KCS;
	io.addr_space	= IPMI_MEM_ADDR_SPACE;
	io.io_setup	= ipmi_ls2k_sim_setup;
	io.addr_data	= pdev->resource[0].start;
	io.regspacing	= pdev->resource[0].end - pdev->resource[0].start + 1;
	io.regsize	= DEFAULT_REGSIZE;
	io.regshift	= 0;
	io.dev		= &pdev->dev;
	io.irq		= 0;
	if (io.irq)
		io.irq_setup = ipmi_std_irq_setup;

	dev_info(&pdev->dev, "%pR regsize %d spacing %d irq %d\n",
		 &pdev->resource[0], io.regsize, io.regspacing, io.irq);

	return ipmi_si_add_smi(&io);
}

static void ipmi_ls2k_remove(struct platform_device *pdev)
{
	ipmi_si_remove_by_dev(&pdev->dev);
}

struct platform_driver ipmi_ls2k_platform_driver = {
	.driver = {
		.name = "ls2k-ipmi-si",
	},
	.probe	= ipmi_ls2k_probe,
	.remove	= ipmi_ls2k_remove,
};

static bool platform_registered;
void ipmi_si_ls2k_init(void)
{
	int rv;

	rv = platform_driver_register(&ipmi_ls2k_platform_driver);
	if (rv)
		pr_err("Unable to register driver: %d\n", rv);
	else
		platform_registered = true;
}

void ipmi_si_ls2k_shutdown(void)
{
	if (platform_registered)
		platform_driver_unregister(&ipmi_ls2k_platform_driver);
}
