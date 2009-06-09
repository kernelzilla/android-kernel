/*
 * linux/arch/arm/mach-omap2/board-sholes-sensors.c
 *
 * Copyright (C) 2009 Google, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/sfh7743.h>
#include <linux/bu52014hfv.h>
#include <linux/vib-omap-pwm.h>
#include <linux/lis331dlh.h>
#include <linux/akm8973.h>
#include <linux/delay.h>

#include <mach/mux.h>
#include <mach/gpio.h>
#include <mach/keypad.h>

#define SHOLES_PROX_INT_GPIO		180
#define SHOLES_HF_NORTH_GPIO		10
#define SHOLES_HF_SOUTH_GPIO		111
#define SHOLES_AKM8973_INT_GPIO		175
#define SHOLES_AKM8973_RESET_GPIO	28

static struct sfh7743_platform_data omap3430_proximity_data = {
	.gpio_prox_int = SHOLES_PROX_INT_GPIO,
	.regulator = "vsdio",
};

static struct bu52014hfv_platform_data bu52014hfv_platform_data = {
	.docked_north_gpio = SHOLES_HF_NORTH_GPIO,
	.docked_south_gpio = SHOLES_HF_SOUTH_GPIO,
	.north_is_desk = 1,
};

static struct regulator *sholes_lis331dlh_regulator;
static int sholes_lis331dlh_init(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vhvio");
	if (IS_ERR(reg)) {
		return PTR_ERR(reg);
	}
	sholes_lis331dlh_regulator = reg;
	return 0;
}

static void sholes_lis331dlh_exit(void)
{
	regulator_put(sholes_lis331dlh_regulator);
}

static int sholes_lis331dlh_power_on(void)
{
	return regulator_enable(sholes_lis331dlh_regulator);
}

static int sholes_lis331dlh_power_off(void)
{
	if (sholes_lis331dlh_regulator)
		return regulator_disable(sholes_lis331dlh_regulator);
	return 0;
}

struct lis331dlh_platform_data sholes_lis331dlh_data = {
	.init = sholes_lis331dlh_init,
	.exit = sholes_lis331dlh_exit,
	.power_on = sholes_lis331dlh_power_on,
	.power_off = sholes_lis331dlh_power_off,
	.min_interval   = 1,
	.g_range        = 48,
	.fuzz           = 4,
	.flat           = 4,
	.interval       = 200,
	.axis_map_x     = 0,
	.axis_map_y     = 1,
	.axis_map_z     = 2,
	.negate_x       = 0,
	.negate_y       = 0,
	.negate_z       = 0,
};

static struct regulator *sholes_akm8973_regulator;
static int sholes_akm8973_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vhvio");
	if (IS_ERR(reg)) {
		return PTR_ERR(reg);
	}
	sholes_akm8973_regulator = reg;
	return 0;
}

static void sholes_akm8973_exit(void)
{
	regulator_put(sholes_akm8973_regulator);
}

static int sholes_akm8973_power_on(void)
{
	int ret;

	ret = regulator_enable(sholes_akm8973_regulator);
	gpio_set_value(SHOLES_AKM8973_RESET_GPIO, 0);
	udelay(10);
	gpio_set_value(SHOLES_AKM8973_RESET_GPIO, 1);
	return ret;
}

static int sholes_akm8973_power_off(void)
{
	if (sholes_akm8973_regulator)
		return regulator_disable(sholes_akm8973_regulator);
	return 0;
}

struct akm8973_platform_data sholes_akm8973_data = {
	.init = sholes_akm8973_initialization,
	.exit = sholes_akm8973_exit,
	.power_on = sholes_akm8973_power_on,
	.power_off = sholes_akm8973_power_off,
	.poll_interval = 200,
	.i2c_retries = 5,
	.i2c_retry_delay = 5,

	.cal_min_threshold = 8,
	.cal_max_threshold = 247,

	.hxda = 0x02,
	.hyda = 0x86,
	.hzda = 0x07,

	.orientation = 270,
	.xy_swap = 1,
	.z_flip = 1,
};

static void __init sholes_akm8973_init(void)
{
	gpio_request(SHOLES_AKM8973_RESET_GPIO, "akm8973 reset");
	gpio_direction_output(SHOLES_AKM8973_RESET_GPIO, 1);
	omap_cfg_reg(AB10_34XX_GPIO28_OUT);

	gpio_request(SHOLES_AKM8973_INT_GPIO, "akm8973 irq");
	gpio_direction_input(SHOLES_AKM8973_INT_GPIO);
	omap_cfg_reg(AC3_34XX_GPIO175);
}

struct platform_device sfh7743_platform_device = {
	.name = SFH7743_MODULE_NAME,
	.id = -1,
	.dev = {
		.platform_data = &omap3430_proximity_data,
	},
};

static struct platform_device omap3430_hall_effect_dock = {
	.name	= BU52014HFV_MODULE_NAME,
	.id	= -1,
	.dev	= {
		.platform_data  = &bu52014hfv_platform_data,
	},
};

static struct platform_device omap3430_master_sensor = {
	.name		= "master_sensor",
	.id		= -1,
	.dev		= {
		.platform_data  = NULL,
	},
};

static void sholes_proximity_init(void)
{
	gpio_request(SHOLES_PROX_INT_GPIO, "Sholes proximity sensor");
	gpio_direction_input(SHOLES_PROX_INT_GPIO);
	omap_cfg_reg(Y3_34XX_GPIO180);
}

static void sholes_vibrator_init(void)
{
	omap_cfg_reg(Y4_34XX_GPIO181);
}

static struct platform_device *sholes_sensors[] __initdata = {
	&omap3430_master_sensor,
	&sfh7743_platform_device,
	&omap3430_hall_effect_dock,
};

static void sholes_hall_effect_init(void)
{
	gpio_request(SHOLES_HF_NORTH_GPIO, "sholes dock north");
	gpio_direction_input(SHOLES_HF_NORTH_GPIO);
	omap_cfg_reg(AG25_34XX_GPIO10);

	gpio_request(SHOLES_HF_SOUTH_GPIO, "sholes dock south");
	gpio_direction_input(SHOLES_HF_SOUTH_GPIO);
	omap_cfg_reg(B26_34XX_GPIO111);
}

void __init sholes_sensors_init(void)
{
	sholes_proximity_init();
	sholes_hall_effect_init();
	sholes_vibrator_init();
	/* vibrate for 500ms at startup */
	vibrator_omap_pwm_init(500);
	sholes_akm8973_init();
	platform_add_devices(sholes_sensors, ARRAY_SIZE(sholes_sensors));
}
