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

#include <mach/keypad.h>

int sholesp0b_keymap[] = {
	0x0000000a, 0x01000013, 0x03000072, 0x05000073, 0x060000d9, 0x07000020, 
	0x10000008, 0x11000032, 0x12000026, 0x13000025, 0x14000031, 0x1500002e,
	0x1600002c, 0x20000002, 0x21000015, 0x22000017, 0x2300006b, 0x240000e5,
	0x25000034, 0x26000022, 0x27000012, 0x31000038, 0x32000004, 0x3300006a,
	0x34000069, 0x3500006c, 0x36000067, 0x3700001c, 0x40000006, 0x41000024,
	0x42000030, 0x430000d4, 0x44000014, 0x4500000b, 0x46000096, 0x4700002d,
	0x50000009, 0x51000039, 0x520000e3, 0x5300009e, 0x540000e4, 0x550000e7,
	0x5600000e, 0x5700001e, 0x60000003, 0x6100000b, 0x62000021, 0x63000036,
	0x6400001c, 0x65000018, 0x66000023, 0x67000010, 0x70000005, 0x7100002f,
	0x7200001f, 0x73000019, 0x7400009e, 0x76000016, 0x77000011
};

int sholesp0b_row_gpios[] = { 34, 35, 36, 37, 38, 39, 40, 41, };
int sholesp0b_col_gpios[] = { 43, 53, 54, 55, 56, 57, 58, 63, };

static struct omap_kp_platform_data omap3430_kp_data = {
	.rows		= 8,
	.cols		= 8,
	.keymap		= sholesp0b_keymap,
	.keymapsize	= 59,
	.rep		= 0,
	.row_gpios	= sholesp0b_row_gpios,
	.col_gpios	= sholesp0b_col_gpios,
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
	platform_add_devices(sholes_sensors, ARRAY_SIZE(sholes_sensors));
}
EXPORT_SYMBOL(sholes_sensors_init);
