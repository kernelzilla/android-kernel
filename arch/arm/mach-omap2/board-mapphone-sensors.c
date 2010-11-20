/*
 * linux/arch/arm/mach-omap2/board-mapphone-sensors.c
 *
 * Copyright (C) 2009 Google, Inc.
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Revision history (newest first):
 *
 * Date         CR          Author    Description
 * 10/14/2009  IKMAP-1275   A19746    Fix for Ruth Rotator issue
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/sfh7743.h>
#include <linux/bu52014hfv.h>
#include <linux/lis331dlh.h>
#include <linux/airc.h>
#include <linux/akm8973_akmd.h>
#include <linux/kxtf9.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/vib-gpio.h>
#include <linux/vib-pwm.h>

#ifdef CONFIG_ARM_OF
#include <asm/prom.h>
#include <mach/dt_path.h>
#endif

#include <mach/mux.h>
#include <mach/gpio.h>
#include <mach/keypad.h>
#include <mach/dmtimer.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

#define MAPPHONE_PROX_INT_GPIO		180
#define MAPPHONE_HF_NORTH_GPIO		10
#define MAPPHONE_HF_SOUTH_GPIO		111
#define MAPPHONE_AKM8973_INT_GPIO	175
#define MAPPHONE_AKM8973_RESET_GPIO	28
#define MAPPHONE_VIBRATOR_GPIO		181
#define MAPPHONE_KXTF9_INT_GPIO		22

static int vib_pwm_period;
static int vib_pwm_duty;
static int vib_pwm_enable_gpio;

static struct regulator *mapphone_vibrator_regulator;
static int mapphone_vibrator_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vvib");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	mapphone_vibrator_regulator = reg;
	return 0;
}

static void mapphone_vibrator_exit(void)
{
	regulator_put(mapphone_vibrator_regulator);
}

static int mapphone_vibrator_power_on(void)
{
	regulator_set_voltage(mapphone_vibrator_regulator, 3000000, 3000000);
	return regulator_enable(mapphone_vibrator_regulator);
}

static int mapphone_vibrator_power_off(void)
{
	if (mapphone_vibrator_regulator)
		return regulator_disable(mapphone_vibrator_regulator);
	return 0;
}

static struct vib_gpio_platform_data mapphone_vib_gpio_data = {
	.gpio = MAPPHONE_VIBRATOR_GPIO,
	.max_timeout = 15000,
	.active_low = 0,
	.initial_vibrate = 0,

	.init = mapphone_vibrator_initialization,
	.exit = mapphone_vibrator_exit,
	.power_on = mapphone_vibrator_power_on,
	.power_off = mapphone_vibrator_power_off,
};

static struct platform_device mapphone_vib_gpio = {
	.name           = "vib-gpio",
	.id             = -1,
	.dev            = {
		.platform_data  = &mapphone_vib_gpio_data,
	},
};

static struct omap_dm_timer *vib_pwm_timer;
static int mapphone_lvibrator_initialization(void)
{
	unsigned long load_reg, cmp_reg;
	uint32_t timer_rate = 0;
	int ret = 0;
	vib_pwm_timer = omap_dm_timer_request_specific(11);
	if (vib_pwm_timer == NULL)
		ret = -ENODEV;
	timer_rate = clk_get_rate(omap_dm_timer_get_fclk(vib_pwm_timer));
	load_reg = timer_rate * vib_pwm_period / 1000000;
	cmp_reg = timer_rate * (vib_pwm_period -
				vib_pwm_duty) / 1000000;

	omap_dm_timer_set_source(vib_pwm_timer, OMAP_TIMER_SRC_32_KHZ);
	omap_dm_timer_set_load(vib_pwm_timer, 1, -load_reg);
	omap_dm_timer_set_match(vib_pwm_timer, 1, -cmp_reg);
	omap_dm_timer_set_pwm(vib_pwm_timer, 0, 1,
		       OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
	omap_dm_timer_write_counter(vib_pwm_timer, -2);

	ret = gpio_request(vib_pwm_enable_gpio, "glohap_en_omap");
	if (ret) {
		printk(KERN_ERR "Vibrator GPIO request error %d\n", ret);
		return ret;
	}
	gpio_direction_output(vib_pwm_enable_gpio, 1);

	return 0;
}

static void mapphone_lvibrator_exit(void)
{
	omap_dm_timer_stop(vib_pwm_timer);
	omap_dm_timer_disable(vib_pwm_timer);
	gpio_set_value(vib_pwm_enable_gpio, 0);
}
static void mapphone_lvibrator_power_on(void)
{
	gpio_set_value(vib_pwm_enable_gpio, 1);
	omap_dm_timer_enable(vib_pwm_timer);
	omap_dm_timer_start(vib_pwm_timer);
}

static void mapphone_lvibrator_power_off(void)
{
	omap_dm_timer_stop(vib_pwm_timer);
	omap_dm_timer_disable(vib_pwm_timer);
	gpio_set_value(vib_pwm_enable_gpio, 0);
}

static struct vib_pwm_platform_data vib_pwm_data = {
	.initial_vibrate = 500,
	.init = mapphone_lvibrator_initialization,
	.exit = mapphone_lvibrator_exit,
	.power_on = mapphone_lvibrator_power_on,
	.power_off = mapphone_lvibrator_power_off,
};

static struct platform_device mapphone_vib_pwm = {
	.name = VIB_PWM_NAME,
	.id = -1,
	.dev = {
		.platform_data = &vib_pwm_data,
	},
};


static struct regulator *mapphone_sfh7743_regulator;
static int mapphone_sfh7743_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vsdio");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	mapphone_sfh7743_regulator = reg;
	return 0;
}

static void mapphone_sfh7743_exit(void)
{
	regulator_put(mapphone_sfh7743_regulator);
}

static int mapphone_sfh7743_power_on(void)
{
	return regulator_enable(mapphone_sfh7743_regulator);
}

static int mapphone_sfh7743_power_off(void)
{
	if (mapphone_sfh7743_regulator)
		return regulator_disable(mapphone_sfh7743_regulator);
	return 0;
}

static struct sfh7743_platform_data mapphone_sfh7743_data = {
	.init = mapphone_sfh7743_initialization,
	.exit = mapphone_sfh7743_exit,
	.power_on = mapphone_sfh7743_power_on,
	.power_off = mapphone_sfh7743_power_off,

	.gpio = MAPPHONE_PROX_INT_GPIO,
};

static void __init mapphone_sfh7743_init(void)
{
	gpio_request(MAPPHONE_PROX_INT_GPIO, "sfh7743 proximity int");
	gpio_direction_input(MAPPHONE_PROX_INT_GPIO);
	omap_cfg_reg(Y3_34XX_GPIO180);
}

static struct regulator *mapphone_airc_regulator;
static int mapphone_airc_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vsdio");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	mapphone_airc_regulator = reg;
	return 0;
}

static void mapphone_airc_exit(void)
{
	regulator_put(mapphone_airc_regulator);
}

static int mapphone_airc_power_on(void)
{
	return regulator_enable(mapphone_airc_regulator);
}

static int mapphone_airc_power_off(void)
{
	if (mapphone_airc_regulator)
		return regulator_disable(mapphone_airc_regulator);
	return 0;
}

struct airc_platform_data mapphone_airc_data = {
	.init = mapphone_airc_initialization,
	.exit = mapphone_airc_exit,
	.power_on = mapphone_airc_power_on,
	.power_off = mapphone_airc_power_off,

	.gpio = MAPPHONE_PROX_INT_GPIO,
};

static void __init mapphone_airc_init(void)
{
	gpio_request(MAPPHONE_PROX_INT_GPIO, "airc proximity int");
	gpio_direction_input(MAPPHONE_PROX_INT_GPIO);
	omap_cfg_reg(Y3_34XX_GPIO180);
}

static struct bu52014hfv_platform_data bu52014hfv_platform_data = {
	.docked_north_gpio = MAPPHONE_HF_NORTH_GPIO,
	.docked_south_gpio = MAPPHONE_HF_SOUTH_GPIO,
	.north_is_desk = 1,
};

static struct regulator *mapphone_lis331dlh_regulator;
static int mapphone_lis331dlh_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vhvio");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	mapphone_lis331dlh_regulator = reg;
	return 0;
}

static void mapphone_lis331dlh_exit(void)
{
	regulator_put(mapphone_lis331dlh_regulator);
}

static int mapphone_lis331dlh_power_on(void)
{
	return regulator_enable(mapphone_lis331dlh_regulator);
}

static int mapphone_lis331dlh_power_off(void)
{
	if (mapphone_lis331dlh_regulator)
		return regulator_disable(mapphone_lis331dlh_regulator);
	return 0;
}

struct lis331dlh_platform_data mapphone_lis331dlh_data = {
	.init = mapphone_lis331dlh_initialization,
	.exit = mapphone_lis331dlh_exit,
	.power_on = mapphone_lis331dlh_power_on,
	.power_off = mapphone_lis331dlh_power_off,

	.min_interval	= 1,
	.poll_interval	= 200,

	.g_range	= LIS331DLH_G_8G,

	.axis_map_x	= 0,
	.axis_map_y	= 1,
	.axis_map_z	= 2,

	.negate_x	= 0,
	.negate_y	= 0,
	.negate_z	= 0,
};

static struct regulator *mapphone_kxtf9_regulator;
static int mapphone_kxtf9_initialization(void)
{
	struct regulator *reg;
	reg = regulator_get(NULL, "vhvio");
	if (IS_ERR(reg))
		return PTR_ERR(reg);
	mapphone_kxtf9_regulator = reg;
	return 0;
}

static void mapphone_kxtf9_exit(void)
{
	regulator_put(mapphone_kxtf9_regulator);
}

static int mapphone_kxtf9_power_on(void)
{
	return regulator_enable(mapphone_kxtf9_regulator);
}

static int mapphone_kxtf9_power_off(void)
{
	if (mapphone_kxtf9_regulator)
		return regulator_disable(mapphone_kxtf9_regulator);
	return 0;
}

struct kxtf9_platform_data mapphone_kxtf9_data = {
	.init = mapphone_kxtf9_initialization,
	.exit = mapphone_kxtf9_exit,
	.power_on = mapphone_kxtf9_power_on,
	.power_off = mapphone_kxtf9_power_off,

	.min_interval	= 2,
	.poll_interval	= 200,

	.g_range	= KXTF9_G_8G,

	.axis_map_x	= 1,
	.axis_map_y	= 0,
	.axis_map_z	= 2,

	.negate_x	= 0,
	.negate_y	= 0,
	.negate_z	= 1,

	.data_odr_init		= ODR12_5,
	.ctrl_reg1_init		= RES_12BIT | KXTF9_G_2G | TPE | WUFE | TDTE,
	.int_ctrl_init		= IEA | IEN,
	.tilt_timer_init	= 0x03,
	.engine_odr_init	= OTP12_5 | OWUF50 | OTDT400,
	.wuf_timer_init		= 0x0A,
	.wuf_thresh_init	= 0x20,
	.tdt_timer_init		= 0x78,
	.tdt_h_thresh_init	= 0xB6,
	.tdt_l_thresh_init	= 0x1A,
	.tdt_tap_timer_init	= 0xA2,
	.tdt_total_timer_init	= 0x24,
	.tdt_latency_timer_init	= 0x28,
	.tdt_window_timer_init	= 0xA0,

	.gpio = MAPPHONE_KXTF9_INT_GPIO,
	.gesture = 0,
	.sensitivity_low = {
		  0x50, 0xFF, 0xB8, 0x32, 0x09, 0x0A, 0xA0,
	},
	.sensitivity_medium = {
		  0x50, 0xFF, 0x68, 0x32, 0x09, 0x0A, 0xA0,
	},
	.sensitivity_high = {
		  0x78, 0xB6, 0x1A, 0xA2, 0x24, 0x28, 0xA0,
	},
};

static void __init mapphone_kxtf9_init(void)
{
#ifdef CONFIG_ARM_OF
	struct device_node *node;
	const void *prop;
	int len = 0;

	node = of_find_node_by_path(DT_PATH_ACCELEROMETER);
	if (node) {
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_AXIS_MAP_X, &len);
		if (prop && len)
			mapphone_kxtf9_data.axis_map_x = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_AXIS_MAP_Y, &len);
		if (prop && len)
			mapphone_kxtf9_data.axis_map_y = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_AXIS_MAP_Z, &len);
		if (prop && len)
			mapphone_kxtf9_data.axis_map_z = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_NEGATE_X, &len);
		if (prop && len)
			mapphone_kxtf9_data.negate_x = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_NEGATE_Y, &len);
		if (prop && len)
			mapphone_kxtf9_data.negate_y = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_NEGATE_Z, &len);
		if (prop && len)
			mapphone_kxtf9_data.negate_z = *(u8 *)prop;
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_SENS_LOW, &len);
		if (prop && len)
				memcpy(mapphone_kxtf9_data.sensitivity_low,
						(u8 *)prop, len);
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_SENS_MEDIUM, &len);
		if (prop && len)
				memcpy(mapphone_kxtf9_data.sensitivity_medium,
						(u8 *)prop, len);
		prop = of_get_property(node,
				DT_PROP_ACCELEROMETER_SENS_HIGH, &len);
		if (prop && len)
				memcpy(mapphone_kxtf9_data.sensitivity_high,
						(u8 *)prop, len);
		of_node_put(node);
	}
#endif
	gpio_request(MAPPHONE_KXTF9_INT_GPIO, "kxtf9 accelerometer int");
	gpio_direction_input(MAPPHONE_KXTF9_INT_GPIO);
	omap_cfg_reg(AF9_34XX_GPIO22_DOWN);
}

static void __init mapphone_akm8973_init(void)
{
	gpio_request(MAPPHONE_AKM8973_RESET_GPIO, "akm8973 reset");
	gpio_direction_output(MAPPHONE_AKM8973_RESET_GPIO, 1);
	omap_cfg_reg(AB10_34XX_GPIO28_OUT);

	gpio_request(MAPPHONE_AKM8973_INT_GPIO, "akm8973 irq");
	gpio_direction_input(MAPPHONE_AKM8973_INT_GPIO);
	omap_cfg_reg(AC3_34XX_GPIO175);
}

struct platform_device kxtf9_platform_device = {
	.name = "kxtf9",
	.id = -1,
	.dev = {
		.platform_data = &mapphone_kxtf9_data,
	},
};

struct platform_device sfh7743_platform_device = {
	.name = "sfh7743",
	.id = -1,
	.dev = {
		.platform_data = &mapphone_sfh7743_data,
	},
};

static struct platform_device omap3430_hall_effect_dock = {
	.name	= BU52014HFV_MODULE_NAME,
	.id	= -1,
	.dev	= {
		.platform_data  = &bu52014hfv_platform_data,
	},
};

static void mapphone_vibrator_init(void)
{
	gpio_direction_output(MAPPHONE_VIBRATOR_GPIO, 0);
	omap_cfg_reg(Y4_34XX_GPIO181);
}

static struct platform_device *mapphone_sensors[] __initdata = {
	&kxtf9_platform_device,
	&sfh7743_platform_device,
	&omap3430_hall_effect_dock,
	&mapphone_vib_gpio,
};

static int mapphone_lvibrator_devtree(void)
{
#ifdef CONFIG_ARM_OF
	struct device_node *vib_pwm_node;
	const void *vib_pwm_prop = NULL;
	int len = 0;
#endif
	u8 lvibrator_in_device = 0;

	vib_pwm_enable_gpio = 9;
	vib_pwm_period = 5714;
	vib_pwm_duty = 2857;

#ifdef CONFIG_ARM_OF
	vib_pwm_node = of_find_node_by_path("/System@0/LinearVibrator@0");
	if (vib_pwm_node != NULL) {
		lvibrator_in_device = 1;

		vib_pwm_prop = of_get_property(vib_pwm_node, "supported", &len);
		if (vib_pwm_prop != NULL)
			lvibrator_in_device = *((u8 *)vib_pwm_prop);

		if (lvibrator_in_device != 0) {
			vib_pwm_prop = of_get_property(vib_pwm_node,
					"period", &len);
			if (vib_pwm_prop != NULL)
				vib_pwm_period = *((int *)vib_pwm_prop);

			vib_pwm_prop = of_get_property(vib_pwm_node,
					"cycle", &len);
			if (vib_pwm_prop != NULL)
				vib_pwm_duty = *((int *)vib_pwm_prop);

			vib_pwm_prop = of_get_property(vib_pwm_node,
					"gpio", &len);
			if (vib_pwm_prop != NULL)
				vib_pwm_enable_gpio = *((int *)vib_pwm_prop);
		}

		of_node_put(vib_pwm_node);
	}
#endif
	return lvibrator_in_device;
}

static void mapphone_hall_effect_init(void)
{
	gpio_request(MAPPHONE_HF_NORTH_GPIO, "mapphone dock north");
	gpio_direction_input(MAPPHONE_HF_NORTH_GPIO);
	omap_cfg_reg(AG25_34XX_GPIO10);

	gpio_request(MAPPHONE_HF_SOUTH_GPIO, "mapphone dock south");
	gpio_direction_input(MAPPHONE_HF_SOUTH_GPIO);
	omap_cfg_reg(B26_34XX_GPIO111);
}

void __init mapphone_sensors_init(void)
{
	mapphone_kxtf9_init();
	mapphone_sfh7743_init();
	mapphone_hall_effect_init();
	mapphone_vibrator_init();
	mapphone_akm8973_init();
#ifdef CONFIG_SENSORS_AIRC
	mapphone_airc_init();
#endif
	platform_add_devices(mapphone_sensors, ARRAY_SIZE(mapphone_sensors));
	if (mapphone_lvibrator_devtree())
		platform_device_register(&mapphone_vib_pwm);
}
