/*
 * linux/arch/arm/mach-omap2/board-sholest-sensors.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/sfh7743.h>
#include <linux/bu52014hfv.h>
#include <linux/lis331dlh.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/vib-gpio.h>
#include <linux/vib-pwm.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

#include <mach/mux.h>
#include <mach/gpio.h>
#include <mach/keypad.h>
#include <mach/dmtimer.h>

#define SHOLEST_PROX_INT_GPIO		180
#define SHOLEST_HF_NORTH_GPIO		10
#define SHOLEST_HF_SOUTH_GPIO		111
#define SHOLEST_AKM8973_INT_GPIO	175
#define SHOLEST_AKM8973_RESET_GPIO	28
#define SHOLEST_VIBRATOR_GPIO		181
#ifdef CONFIG_VIB_PWM
#define SHOLEST_VIBRATOR_EN_GPIO	9
#endif
#define SHOLEST_LVIBRATOR_PERIOD	5714
#define SHOLEST_LVIBRATOR_DUTY		2857

static struct regulator *sholest_vibrator_regulator;
static int sholest_vibrator_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vvib");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	sholest_vibrator_regulator = reg;
	return 0;
}

static void sholest_vibrator_exit(void)
{
	regulator_put(sholest_vibrator_regulator);
}

static int sholest_vibrator_power_on(void)
{
	regulator_set_voltage(sholest_vibrator_regulator, 3000000, 3000000);
	return regulator_enable(sholest_vibrator_regulator);
}

static int sholest_vibrator_power_off(void)
{
	if (sholest_vibrator_regulator)
		return regulator_disable(sholest_vibrator_regulator);
	return 0;
}

static struct vib_gpio_platform_data sholest_vib_gpio_data = {
	.gpio = SHOLEST_VIBRATOR_GPIO,
	.max_timeout = 15000,
	.active_low = 0,
	.initial_vibrate = 0,

	.init = sholest_vibrator_initialization,
	.exit = sholest_vibrator_exit,
	.power_on = sholest_vibrator_power_on,
	.power_off = sholest_vibrator_power_off,
};

static struct platform_device sholest_vib_gpio = {
	.name           = VIB_GPIO_NAME,
	.id             = -1,
	.dev            = {
		.platform_data  = &sholest_vib_gpio_data,
    },
};

static struct omap_dm_timer *vib_pwm_timer;
static int sholest_lvibrator_initialization(void)
{
	unsigned long load_reg, cmp_reg;
	uint32_t timer_rate = 0;
	int ret = 0;
	vib_pwm_timer = omap_dm_timer_request_specific(11);
	if (vib_pwm_timer == NULL)
		ret = -ENODEV;
	timer_rate = clk_get_rate(omap_dm_timer_get_fclk(vib_pwm_timer));
	load_reg = timer_rate * SHOLEST_LVIBRATOR_PERIOD / 1000000;
	cmp_reg = timer_rate * (SHOLEST_LVIBRATOR_PERIOD -
				SHOLEST_LVIBRATOR_DUTY) / 1000000;
	omap_dm_timer_enable(vib_pwm_timer);
	omap_dm_timer_set_source(vib_pwm_timer, OMAP_TIMER_SRC_32_KHZ);
	omap_dm_timer_set_load(vib_pwm_timer, 1, -load_reg);
	omap_dm_timer_set_match(vib_pwm_timer, 1, -cmp_reg);
	omap_dm_timer_set_pwm(vib_pwm_timer, 0, 1,
			      OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
	omap_dm_timer_write_counter(vib_pwm_timer, -2);
	return 0;
}

static void sholest_lvibrator_exit(void)
{
	omap_dm_timer_stop(vib_pwm_timer);
}

static void sholest_lvibrator_power_on(void)
{
#ifdef CONFIG_VIB_PWM
	gpio_set_value(SHOLEST_VIBRATOR_EN_GPIO, 1);
#endif
	omap_dm_timer_start(vib_pwm_timer);
}

static void sholest_lvibrator_power_off(void)
{
#ifdef CONFIG_VIB_PWM
	gpio_set_value(SHOLEST_VIBRATOR_EN_GPIO, 0);
#endif
	omap_dm_timer_stop(vib_pwm_timer);
	omap_dm_timer_disable(vib_pwm_timer);
}

static struct vib_pwm_platform_data sholest_vib_pwm_data = {
	.initial_vibrate = 500,
	.init = sholest_lvibrator_initialization,
	.exit = sholest_lvibrator_exit,
	.power_on = sholest_lvibrator_power_on,
	.power_off = sholest_lvibrator_power_off,
};

static struct platform_device sholest_vib_pwm = {
	.name = VIB_PWM_NAME,
	.id = -1,
	.dev = {
		.platform_data = &sholest_vib_pwm_data,
	},
};

static struct regulator *sholest_sfh7743_regulator;
static int sholest_sfh7743_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vsdio");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	sholest_sfh7743_regulator = reg;
	return 0;
}

static void sholest_sfh7743_exit(void)
{
	regulator_put(sholest_sfh7743_regulator);
}

static int sholest_sfh7743_power_on(void)
{
	return regulator_enable(sholest_sfh7743_regulator);
}

static int sholest_sfh7743_power_off(void)
{
	if (sholest_sfh7743_regulator)
		return regulator_disable(sholest_sfh7743_regulator);
	return 0;
}

static struct sfh7743_platform_data sholest_sfh7743_data = {
	.init = sholest_sfh7743_initialization,
	.exit = sholest_sfh7743_exit,
	.power_on = sholest_sfh7743_power_on,
	.power_off = sholest_sfh7743_power_off,

	.gpio = SHOLEST_PROX_INT_GPIO,
};

static void __init sholest_sfh7743_init(void)
{
	gpio_request(SHOLEST_PROX_INT_GPIO, "sfh7743 proximity int");
	gpio_direction_input(SHOLEST_PROX_INT_GPIO);
	omap_cfg_reg(Y3_34XX_GPIO180);
}

static struct bu52014hfv_platform_data bu52014hfv_platform_data = {
	.docked_north_gpio = SHOLEST_HF_NORTH_GPIO,
	.docked_south_gpio = SHOLEST_HF_SOUTH_GPIO,
	.north_is_desk = 1,
};

struct lis331dlh_platform_data sholest_lis331dlh_data;
static void __init sholest_lis331dlh_init(void)
{
#ifdef CONFIG_ARM_OF
	struct device_node *lis331dlh_node;
	const void *lis331dlh_prop;
	int len = 0;

	lis331dlh_node = of_find_node_by_path(DT_PATH_LIS331DLH);
	if (lis331dlh_node) {
		lis331dlh_prop = of_get_property(lis331dlh_node, \
						"axis_map_x", &len);
		if (lis331dlh_prop && len)
			sholest_lis331dlh_data.axis_map_x = \
						*(u8 *)lis331dlh_prop;
		lis331dlh_prop = of_get_property(lis331dlh_node, \
						"axis_map_y", &len);
		if (lis331dlh_prop && len)
			sholest_lis331dlh_data.axis_map_y = \
						*(u8 *)lis331dlh_prop;
		lis331dlh_prop = of_get_property(lis331dlh_node, \
						"axis_map_z", &len);
		if (lis331dlh_prop && len)
			sholest_lis331dlh_data.axis_map_z = \
						*(u8 *)lis331dlh_prop;
		lis331dlh_prop = of_get_property(lis331dlh_node, \
						"negate_x", &len);
		if (lis331dlh_prop && len)
			sholest_lis331dlh_data.negate_x = \
						*(u8 *)lis331dlh_prop;
		lis331dlh_prop = of_get_property(lis331dlh_node, \
						"negate_x", &len);
		if (lis331dlh_prop && len)
			sholest_lis331dlh_data.negate_x = \
						*(u8 *)lis331dlh_prop;
		lis331dlh_prop = of_get_property(lis331dlh_node, \
						"negate_z", &len);
		if (lis331dlh_prop && len)
			sholest_lis331dlh_data.negate_z = \
						*(u8 *)lis331dlh_prop;
		of_node_put(lis331dlh_node);
	}
#endif
}

static struct regulator *sholest_lis331dlh_regulator;
static int sholest_lis331dlh_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vhvio");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	sholest_lis331dlh_regulator = reg;
	return 0;
}

static void sholest_lis331dlh_exit(void)
{
	regulator_put(sholest_lis331dlh_regulator);
}

static int sholest_lis331dlh_power_on(void)
{
	return regulator_enable(sholest_lis331dlh_regulator);
}

static int sholest_lis331dlh_power_off(void)
{
	if (sholest_lis331dlh_regulator)
		return regulator_disable(sholest_lis331dlh_regulator);
	return 0;
}

struct lis331dlh_platform_data sholest_lis331dlh_data = {
	.init = sholest_lis331dlh_initialization,
	.exit = sholest_lis331dlh_exit,
	.power_on = sholest_lis331dlh_power_on,
	.power_off = sholest_lis331dlh_power_off,

	.min_interval	= 1,
	.poll_interval	= 200,

	.g_range	= LIS331DLH_G_8G,

	.axis_map_x	= 0,
	.axis_map_y	= 1,
	.axis_map_z	= 2,

	.negate_x	= 0,
	.negate_y	= 1,
	.negate_z	= 1,
};

static void __init sholest_akm8973_init(void)
{
	gpio_request(SHOLEST_AKM8973_RESET_GPIO, "akm8973 reset");
	gpio_direction_output(SHOLEST_AKM8973_RESET_GPIO, 1);
	omap_cfg_reg(AB10_34XX_GPIO28_OUT);

	gpio_request(SHOLEST_AKM8973_INT_GPIO, "akm8973 irq");
	gpio_direction_input(SHOLEST_AKM8973_INT_GPIO);
	omap_cfg_reg(AC3_34XX_GPIO175);
}

struct platform_device sfh7743_platform_device = {
	.name = "sfh7743",
	.id = -1,
	.dev = {
		.platform_data = &sholest_sfh7743_data,
	},
};

static struct platform_device omap3430_hall_effect_dock = {
	.name	= BU52014HFV_MODULE_NAME,
	.id	= -1,
	.dev	= {
		.platform_data  = &bu52014hfv_platform_data,
	},
};

static void sholest_vibrator_init(void)
{
	gpio_request(SHOLEST_VIBRATOR_GPIO, "vibrator");
	gpio_direction_output(SHOLEST_VIBRATOR_GPIO, 0);
	omap_cfg_reg(Y4_34XX_GPIO181);

#ifdef CONFIG_VIB_PWM
	gpio_request(SHOLEST_VIBRATOR_EN_GPIO, "vibrator en");
	gpio_direction_output(SHOLEST_VIBRATOR_EN_GPIO, 0);
	omap_cfg_reg(AF22_34XX_GPIO9_OUT);
#endif
}

static struct platform_device *sholest_sensors[] __initdata = {
	&sfh7743_platform_device,
	&omap3430_hall_effect_dock,
	&sholest_vib_gpio,
	&sholest_vib_pwm,
};

static void sholest_hall_effect_init(void)
{
	gpio_request(SHOLEST_HF_NORTH_GPIO, "mapphone dock north");
	gpio_direction_input(SHOLEST_HF_NORTH_GPIO);
	omap_cfg_reg(AG25_34XX_GPIO10);

	gpio_request(SHOLEST_HF_SOUTH_GPIO, "mapphone dock south");
	gpio_direction_input(SHOLEST_HF_SOUTH_GPIO);
	omap_cfg_reg(B26_34XX_GPIO111);
}

void __init sholest_sensors_init(void)
{
	sholest_sfh7743_init();
	sholest_hall_effect_init();
	sholest_vibrator_init();
	sholest_akm8973_init();
	sholest_lis331dlh_init();
	platform_add_devices(sholest_sensors, ARRAY_SIZE(sholest_sensors));
}
