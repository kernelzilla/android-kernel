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
#include <mach/mux.h>

#define SHOLES_PROX_INT_GPIO		180
#define SHOLES_HF_NORTH_GPIO		10
#define SHOLES_HF_SOUTH_GPIO		111
#define SHOLES_AKM8973_INT_GPIO		175
#define SHOLES_AKM8973_RESET_GPIO	28

int sholesp0b_keymap[] = {
	0x0000000a, 0x01000013, 0x03000072, 0x05000073, 0x060000d9, 0x07000020,
	0x10000008, 0x11000032, 0x12000026, 0x13000025, 0x14000031, 0x1500002e,
	0x1600002c, 0x20000002, 0x21000015, 0x22000017, 0x2300006b, 0x240000e5,
	0x25000034, 0x26000022, 0x27000012, 0x31000007, 0x32000004, 0x3300006c,
	0x34000067, 0x35000069, 0x3600006a, 0x3700001c, 0x40000006, 0x41000024,
	0x42000030, 0x430000d4, 0x44000014, 0x4500000b, 0x46000096, 0x4700002d,
	0x50000009, 0x51000039, 0x52000064, 0x5300009e, 0x540000e4, 0x550000e7,
	0x5600000e, 0x5700001e, 0x60000003, 0x6100000b, 0x62000021, 0x63000036,
	0x6400001c, 0x65000018, 0x66000023, 0x67000010, 0x70000005, 0x7100002f,
	0x7200001f, 0x73000019, 0x7400009e, 0x76000016, 0x77000011
};

static struct omap_kp_switchmap sholesp1_switchmap[] = {
	{.gpio = 100, .key = SW_HEADPHONE_INSERT},  /* silence */
	{.gpio = 177, .key = SW_LID}                /* slider */
};

int sholesp1_row_gpios[] = { 34, 35, 36, 37, 38, 39, 40, 41, };
int sholesp1_col_gpios[] = { 43, 53, 54, 55, 56, 57, 58, 63, };

static struct omap_kp_platform_data omap3430_kp_data = {
	.rows		= 8,
	.cols		= 8,
	.keymap		= sholesp0b_keymap,
	.keymapsize	= 59,
	.switchmap	= sholesp1_switchmap,
	.switchmapsize  = 2,
	.delay		= 200,
	.rep		= 0,
	.row_gpios	= sholesp1_row_gpios,
	.col_gpios	= sholesp1_col_gpios,
	.input_name	= "sholes-keypad",
	.input_phys_device = "sholes-keypad/input0"
};

static struct sfh7743_platform_data omap3430_proximity_data = {
	.gpio_prox_int = SHOLES_PROX_INT_GPIO,
	.regulator = "vsdio",
};

static struct bu52014hfv_platform_data bu52014hfv_platform_data = {
	.docked_north_gpio = SHOLES_HF_NORTH_GPIO,
	.docked_south_gpio = SHOLES_HF_SOUTH_GPIO,
	.north_is_desk = 1,
};

static struct platform_device omap3430_kp_device = {
	.name		= "omap-keypad",
	.id		= -1,
	.dev		= {
		.platform_data  = &omap3430_kp_data,
	},
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
	.poll_interval = 27,
	.i2c_retries = 5,
	.i2c_retry_delay = 5,

	.cal_min_threshold = 45,
	.cal_max_threshold = 210,

	.hxda = 0x81,
	.hyda = 0x82,
	.hzda = 0x01,

	.orientation = 180,
	.xy_swap = 180,
	.z_flip = 0,
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
	omap_cfg_reg(Y3_3430_GPIO180);
}

static void sholes_vibrator_init(void)
{
	omap_cfg_reg(Y4_3430_GPIO181);
}

static struct platform_device *sholes_sensors[] __initdata = {
	&omap3430_kp_device,
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
	/* keypad rows */
	omap_cfg_reg(N4_34XX_GPIO34);
	omap_cfg_reg(M4_34XX_GPIO35);
	omap_cfg_reg(L4_34XX_GPIO36);
	omap_cfg_reg(K4_34XX_GPIO37);
	omap_cfg_reg(T3_34XX_GPIO38);
	omap_cfg_reg(R3_34XX_GPIO39);
	omap_cfg_reg(N3_34XX_GPIO40);
	omap_cfg_reg(M3_34XX_GPIO41);

	/* keypad columns */
	omap_cfg_reg(K3_34XX_GPIO43_OUT);
	omap_cfg_reg(V8_34XX_GPIO53_OUT);
	omap_cfg_reg(U8_34XX_GPIO54_OUT);
	omap_cfg_reg(T8_34XX_GPIO55_OUT);
	omap_cfg_reg(R8_34XX_GPIO56_OUT);
	omap_cfg_reg(P8_34XX_GPIO57_OUT);
	omap_cfg_reg(N8_34XX_GPIO58_OUT);
	omap_cfg_reg(L8_34XX_GPIO63_OUT);

	/* switches */
	omap_cfg_reg(AB2_34XX_GPIO177);
	omap_cfg_reg(AH17_34XX_GPIO100);

	sholes_proximity_init();
	sholes_hall_effect_init();
	sholes_vibrator_init();
	/* vibrate for 500ms at startup */
	vibrator_omap_pwm_init(500);
	sholes_akm8973_init();
	platform_add_devices(sholes_sensors, ARRAY_SIZE(sholes_sensors));
}
