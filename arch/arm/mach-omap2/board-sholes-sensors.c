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

#include <mach/mux.h>
#include <mach/keypad.h>

int sholesp0b_keymap[] = {
	0x0000000a, 0x01000013, 0x03000072, 0x05000073, 0x060000d9, 0x07000020, 
	0x10000008, 0x11000032, 0x12000026, 0x13000025, 0x14000031, 0x1500002e,
	0x1600002c, 0x20000002, 0x21000015, 0x22000017, 0x2300006b, 0x240000e5,
	0x25000034, 0x26000022, 0x27000012, 0x31000007, 0x32000004, 0x3300006c,
	0x34000067, 0x35000069, 0x3600006a, 0x3700001c, 0x40000006, 0x41000024,
	0x42000030, 0x430000d4, 0x44000014, 0x4500000b, 0x46000096, 0x4700002d,
	0x50000009, 0x51000039, 0x520000e3, 0x5300009e, 0x540000e4, 0x550000e7,
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
};

static struct platform_device omap3430_kp_device = {
	.name		= "omap-keypad",
	.id		= -1,
	.dev		= {
		.platform_data  = &omap3430_kp_data,
	},
};

static struct platform_device omap3430_master_sensor= {
	.name		= "master_sensor",
	.id		= -1,
	.dev		= {
		.platform_data  = NULL,
	},
};

static struct platform_device *sholes_sensors[] __initdata = {
	&omap3430_kp_device,
	&omap3430_master_sensor,
};

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

	platform_add_devices(sholes_sensors, ARRAY_SIZE(sholes_sensors));
}
