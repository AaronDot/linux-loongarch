/* SPDX-License-Identifier: GPL-2.0+ */
//
// Loongson Flash SPI Support - common header
//
// Copyright (C) 2023-2026 Loongson Technology Corporation Limited

#ifndef __LINUX_SPI_LOONGSON_FLASH_H
#define __LINUX_SPI_LOONGSON_FLASH_H

int loongson_spi_init_controller(struct device *dev, void __iomem *reg, int irq);
extern const struct dev_pm_ops loongson_spi_dev_pm_ops;

#endif /* __LINUX_SPI_LOONGSON_FLASH_H */
