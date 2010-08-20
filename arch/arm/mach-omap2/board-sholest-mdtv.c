/*
 * linux/arch/arm/mach-omap2/board-MAPPHONE-sensors.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

#include <mach/mux.h>
#include <mach/gpio.h>
#include <mach/keypad.h>

#include "linux/i2c/lp3907_i2c.h"

#define SHOLEST_MDTV_INT_GPIO			38
#define SHOLEST_MDTV_PWDN_GPIO			53
#define SHOLEST_MDTV_RESET_N_GPIO		54
#define SHOLEST_MDTV_REG_EN_GPIO		21

static int sholest_lp3907_init(void)
{
	printk(KERN_INFO "sholest_lp3907_init()");
	return 0;
}

static void sholest_lp3907_exit(void)
{
	/*regulator_put(sholest_akm8973_regulator);*/
}

static int sholest_lp3907_power_on(void)
{
	/* SPI pin control */
	omap_cfg_reg(F1_34XX_MDTV_INT_ON);

	/* EN_T is high */
	gpio_set_value(SHOLEST_MDTV_REG_EN_GPIO, 1);
	/*mdelay(6);*/	/* stable time */
	msleep(6);

	return 0;
}

static int sholest_lp3907_power_off(void)
{
	/* SPI pin control */
	omap_cfg_reg(F1_34XX_MDTV_INT_OFF);

	/* EN_T is low */
	gpio_set_value(SHOLEST_MDTV_REG_EN_GPIO, 0);
	/*mdelay(6);*/ /* stable time */
	msleep(6);

	return 0;
}

struct lp3907_platform_data sholest_lp3907_data = {
	.init = sholest_lp3907_init,
	.exit = sholest_lp3907_exit,
	.power_on = sholest_lp3907_power_on,
	.power_off = sholest_lp3907_power_off,
};

/*
*	TDMB module initialize.
*/
void __init sholest_mdtv_init(void)
{
	/* MTV_INT pin */
	gpio_request(SHOLEST_MDTV_INT_GPIO, "sms1130 int");
	gpio_direction_input(SHOLEST_MDTV_INT_GPIO);
	omap_cfg_reg(T3_34XX_GPIO38);

	/* MTV_PWDN pin - low */
	gpio_request(SHOLEST_MDTV_PWDN_GPIO, "sms1130 pwdn");
	gpio_direction_output(SHOLEST_MDTV_PWDN_GPIO, 0);
	omap_cfg_reg(V8_34XX_GPIO53_OUT);

	/* MTV_RST_N pin - low */
	gpio_request(SHOLEST_MDTV_RESET_N_GPIO, "sms1130 reset");
	gpio_direction_output(SHOLEST_MDTV_RESET_N_GPIO, 0);
	omap_cfg_reg(U8_34XX_GPIO54_OUT);

	/* MTV_REG_EN pin - low */
	gpio_request(SHOLEST_MDTV_REG_EN_GPIO, "lp3907 en");
	gpio_direction_output(SHOLEST_MDTV_REG_EN_GPIO, 0);
	omap_cfg_reg(V8_34XX_GPIO53_OUT);

	printk(KERN_INFO "[TDMB] sholest_mdtv_init()\n");
}
