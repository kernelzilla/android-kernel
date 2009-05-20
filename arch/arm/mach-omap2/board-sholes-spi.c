/*
 * arch/arm/mach-omap2/board-sholes-spi.c
 *
 * Copyright (C) 2007-2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/spi/cpcap.h>
#include <linux/spi/spi.h>
#include <linux/irq.h>
#include <mach/mcspi.h>
#include <mach/gpio.h>
#include <mach/mux.h>

struct cpcap_spi_init_data sholes_cpcap_spi_init[] = {
	{CPCAP_REG_ASSIGN1,   0x0101},
	{CPCAP_REG_ASSIGN2,   0x0000},
	{CPCAP_REG_ASSIGN3,   0x0000},
	{CPCAP_REG_ASSIGN4,   0x0000},
	{CPCAP_REG_ASSIGN5,   0x0000},
	{CPCAP_REG_ASSIGN6,   0x0000},
	{CPCAP_REG_MIM2,      0x0000},
	{CPCAP_REG_PC1,       0x010A},
	{CPCAP_REG_PC2,       0x0150},
	{CPCAP_REG_PGC,       0x0000},
	{CPCAP_REG_SDVSPLL,   0xDB04},
	{CPCAP_REG_SI2CC1,    0x0201},
	{CPCAP_REG_Si2CC2,    0x00C4},
	{CPCAP_REG_S1C1,      0x6434},
	{CPCAP_REG_S1C2,      0x3C14},
	{CPCAP_REG_S2C1,      0x6434},
	{CPCAP_REG_S2C2,      0x3C14},
	{CPCAP_REG_S3C,       0x0521},
	{CPCAP_REG_S4C1,      0x4434},
	{CPCAP_REG_S4C2,      0x3434},
	{CPCAP_REG_S6C,       0x0000},
	{CPCAP_REG_VRF1C,     0x002C},
	{CPCAP_REG_VRF2C,     0x000B},
	{CPCAP_REG_VRFREFC,   0x000B},
	{CPCAP_REG_VUSBINT1C, 0x0029},
	{CPCAP_REG_VUSBINT2C, 0x0029},
	{CPCAP_REG_USBC3,     0x3DFF},
	{CPCAP_REG_UIER2,     0x001F},
	{CPCAP_REG_UIEF2,     0x001F},
	{CPCAP_REG_OWDC,      0x0002},
	{CPCAP_REG_GPIO0,     0x0000},
	{CPCAP_REG_GPIO1,     0x0000},
	{CPCAP_REG_GPIO2,     0x0000},
	{CPCAP_REG_GPIO3,     0x0000},
	{CPCAP_REG_GPIO4,     0x0000},
	{CPCAP_REG_GPIO5,     0x0000},
	{CPCAP_REG_GPIO6,     0x0000},
};

#define CPCAP_GPIO 0

static struct cpcap_platform_data sholes_cpcap_data = {
	.init = sholes_cpcap_spi_init,
	.init_len = ARRAY_SIZE(sholes_cpcap_spi_init),
};

static struct omap2_mcspi_device_config tsc2005_mcspi_config = {
	.turbo_mode = 0,
	.single_channel = 1,
};

static struct spi_board_info sholes_spi_board_info[] __initdata = {
	{
		.modalias = "cpcap",
		.bus_num = 1,
		.chip_select = 0,
		.max_speed_hz = 20000000,
		.controller_data = &sholes_cpcap_data,
		.mode = SPI_CS_HIGH,
	},
	{
		.modalias = "tsc2005",
		.bus_num = 3,
		.chip_select = 1,
		.max_speed_hz = 1500000,
		.controller_data = &tsc2005_mcspi_config,
	}
};

void __init sholes_spi_init(void)
{
	int irq;
	int ret;

	ret = gpio_request(CPCAP_GPIO, "cpcap-irq");
	if (ret)
		return;
	ret = gpio_direction_input(CPCAP_GPIO);
	if (ret) {
		gpio_free(CPCAP_GPIO);
		return;
	}

	irq = gpio_to_irq(CPCAP_GPIO);
	set_irq_type(irq, IRQ_TYPE_EDGE_RISING);
	omap_cfg_reg(AF26_34XX_GPIO0);

	sholes_spi_board_info[0].irq = irq;
	spi_register_board_info(sholes_spi_board_info,
			       ARRAY_SIZE(sholes_spi_board_info));
}
