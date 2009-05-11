/*
 * linux/arch/arm/mach-omap2/board-sholes.c
 *
 * Copyright (C) 2007-2009 Motorola, Inc.
 *
 * Modified from mach-omap3/board-3430sdp.c
 *
 * Copyright (C) 2007 Texas Instruments
 *
 * Modified from mach-omap2/board-generic.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/mm.h>
#include <linux/qtouch_obp_ts.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>

#include <mach/board-sholes.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/board.h>
#include <mach/common.h>
#include <mach/gpmc.h>
#include <mach/usb.h>
#include <asm/delay.h>
#include <mach/control.h>

#define SHOLES_TOUCH_RESET_N_GPIO	164
#define SHOLES_TOUCH_INT_GPIO		99

static void __init sholes_init_irq(void)
{
	omap2_init_common_hw(NULL, NULL, NULL, NULL);
	omap_init_irq();
#ifdef CONFIG_OMAP3_PM
	scm_clk_init();
#endif
	omap_gpio_init();
}

static struct omap_uart_config sholes_uart_config __initdata = {
	.enabled_uarts = ((1 << 0) | (1 << 1) | (1 << 2)),
};

static struct omap_board_config_kernel sholes_config[] __initdata = {
	{OMAP_TAG_UART,		&sholes_uart_config },
};

static int sholes_touch_reset(void)
{
	gpio_direction_output(SHOLES_TOUCH_RESET_N_GPIO, 1);
	msleep(1);
	gpio_set_value(SHOLES_TOUCH_RESET_N_GPIO, 0);
	msleep(20);
	gpio_set_value(SHOLES_TOUCH_RESET_N_GPIO, 1);
	msleep(20);

	return 0;
}

static void sholes_touch_init(void)
{
	gpio_request(SHOLES_TOUCH_RESET_N_GPIO, "sholes touch reset");
	gpio_direction_output(SHOLES_TOUCH_RESET_N_GPIO, 1);
	omap_cfg_reg(H19_34XX_GPIO164_OUT);

	gpio_request(SHOLES_TOUCH_INT_GPIO, "sholes touch irq");
	gpio_direction_input(SHOLES_TOUCH_INT_GPIO);
	omap_cfg_reg(AG17_34XX_GPIO99);
}

static struct qtouch_key sholes_touch_key_list[] = {
	{
		.channel	= 0,
		.code		= KEY_MENU,
	},
	{
		.channel	= 3,
		.code		= KEY_HOME,
	},
	{
		.channel	= 6,
		.code		= KEY_BACK,
	},
};

static struct qtouch_ts_platform_data sholes_ts_platform_data = {
	.irqflags	= IRQF_TRIGGER_LOW,
	.flags		= (QTOUCH_SWAP_XY |
			   QTOUCH_USE_MULTITOUCH |
			   QTOUCH_USE_KEYARRAY |
			   QTOUCH_CFG_BACKUPNV),
	.abs_min_x	= 0,
	.abs_max_x	= 1024,
	.abs_min_y	= 0,
	.abs_max_y	= 1024,
	.abs_min_p	= 0,
	.abs_max_p	= 255,
	.abs_min_w	= 0,
	.abs_max_w	= 15,
	.fuzz_x		= 0,
	.fuzz_y		= 0,
	.fuzz_p		= 2,
	.fuzz_w		= 2,
	.hw_reset	= sholes_touch_reset,
	.power_cfg	= {
		.idle_acq_int	= 1,
		.active_acq_int	= 16,
		.active_idle_to	= 25,
	},
	.acquire_cfg	= {
		.charge_time	= 10,
		.atouch_drift	= 5,
		.touch_drift	= 20,
		.drift_susp	= 20,
		.touch_autocal	= 0,
		.sync		= 0,
	},
	.multi_touch_cfg	= {
		.x_origin	= 0,
		.y_origin	= 0,
		.x_size		= 10,
		.y_size		= 7,
		.aks_cfg	= 1,
		.burst_len	= 0x50,
		.tch_det_thr	= 0xf,
		.tch_det_int	= 0x2,
		.mov_hyst_init	= 5,
		.mov_hyst_next	= 5,
		.mov_filter	= 0,
		.num_touch	= 4,
		.merge_hyst	= 0,
		.merge_thresh	= 3,
	},
	.key_array		= {
		.keys		= sholes_touch_key_list,
		.num_keys	= ARRAY_SIZE(sholes_touch_key_list),
		.cfg		= {
			.x_origin	= 11,
			.y_origin	= 0,
			.x_size		= 1,
			.y_size		= 7,
			.aks_cfg	= 1,
			.burst_len	= 0x50,
			.tch_det_thr	= 0xf,
			.tch_det_int	= 0x2,
		},
	},
};

static struct i2c_board_info __initdata sholes_i2c_bus1_board_info[] = {
	{
		I2C_BOARD_INFO(QTOUCH_TS_NAME, 0x11),
		.platform_data = &sholes_ts_platform_data,
		.irq = OMAP_GPIO_IRQ(SHOLES_TOUCH_INT_GPIO),
	},
};

static int __init sholes_i2c_init(void)
{
	omap_register_i2c_bus(1, 400, sholes_i2c_bus1_board_info,
			      ARRAY_SIZE(sholes_i2c_bus1_board_info));
	return 0;
}

arch_initcall(sholes_i2c_init);

extern void __init sholes_spi_init(void);
extern void __init sholes_flash_init(void);
extern void __init sholes_gpio_iomux_init(void);


static void __init sholes_init(void)
{
	omap_board_config = sholes_config;
	omap_board_config_size = ARRAY_SIZE(sholes_config);
	sholes_spi_init();
	sholes_flash_init();
	omap_serial_init();
	sholes_panel_init();
	sholes_sensors_init();
	sholes_touch_init();
	usb_musb_init();
}


static void __init sholes_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(SHOLES, "sholes")
	/* Maintainer: Motorola, Inc. */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
#ifdef CONFIG_MACH_SHOLES_UMTS
	.boot_params	= 0x80C00100,
#else
	.boot_params	= 0x80000100,
#endif
	.map_io		= sholes_map_io,
	.init_irq	= sholes_init_irq,
	.init_machine	= sholes_init,
	.timer		= &omap_timer,
MACHINE_END
