/*
 * linux/arch/arm/mach-omap2/board-omap3evm.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp.c
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
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/input.h>
#include <linux/leds.h>

#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/i2c/twl4030.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/mux.h>
#include <mach/usb.h>
#include <mach/common.h>
#include <mach/mcspi.h>
#include <mach/display.h>
#include <mach/omap-pm.h>
#include <mach/clock.h>

#include "sdram-micron-mt46h32m32lf-6.h"
#include "twl4030-generic-scripts.h"
#include "mmc-twl4030.h"
#include "pm.h"
#include "omap3-opp.h"

static struct resource omap3evm_smc911x_resources[] = {
	[0] =	{
		.start  = OMAP3EVM_ETHR_START,
		.end    = (OMAP3EVM_ETHR_START + OMAP3EVM_ETHR_SIZE - 1),
		.flags  = IORESOURCE_MEM,
	},
	[1] =	{
		.start  = OMAP_GPIO_IRQ(OMAP3EVM_ETHR_GPIO_IRQ),
		.end    = OMAP_GPIO_IRQ(OMAP3EVM_ETHR_GPIO_IRQ),
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device omap3evm_smc911x_device = {
	.name		= "smc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(omap3evm_smc911x_resources),
	.resource	= &omap3evm_smc911x_resources [0],
};

static inline void __init omap3evm_init_smc911x(void)
{
	int eth_cs;
	struct clk *l3ck;
	unsigned int rate;

	eth_cs = OMAP3EVM_SMC911X_CS;

	l3ck = clk_get(NULL, "l3_ck");
	if (IS_ERR(l3ck))
		rate = 100000000;
	else
		rate = clk_get_rate(l3ck);

	if (gpio_request(OMAP3EVM_ETHR_GPIO_IRQ, "SMC911x irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smc911x IRQ\n",
			OMAP3EVM_ETHR_GPIO_IRQ);
		return;
	}

	gpio_direction_input(OMAP3EVM_ETHR_GPIO_IRQ);
}

static struct omap_uart_config omap3_evm_uart_config __initdata = {
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
};

static struct twl4030_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.wires		= 4,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= 63,
	},
	{}	/* Terminator */
};

static struct gpio_led gpio_leds[] = {
	{
		.name			= "omap3evm::ledb",
		/* normally not visible (board underside) */
		.default_trigger	= "default-on",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= true,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};


static int omap3evm_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	omap_cfg_reg(L8_34XX_GPIO63);
	mmc[0].gpio_cd = gpio + 0;
	twl4030_mmc_init(mmc);

	/* Most GPIOs are for USB OTG.  Some are mostly sent to
	 * the P2 connector; notably LEDA for the LCD backlight.
	 */

	/* TWL4030_GPIO_MAX + 1 == ledB (out, active low LED) */
	gpio_leds[2].gpio = gpio + TWL4030_GPIO_MAX + 1;

	platform_device_register(&leds_gpio);

	return 0;
}

static struct twl4030_gpio_platform_data omap3evm_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.setup		= omap3evm_twl_gpio_setup,
};

static struct twl4030_usb_data omap3evm_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static int omap3evm_keymap[] = {
	KEY(0, 0, KEY_LEFT),
	KEY(0, 1, KEY_RIGHT),
	KEY(0, 2, KEY_A),
	KEY(0, 3, KEY_B),
	KEY(1, 0, KEY_DOWN),
	KEY(1, 1, KEY_UP),
	KEY(1, 2, KEY_E),
	KEY(1, 3, KEY_F),
	KEY(2, 0, KEY_ENTER),
	KEY(2, 1, KEY_I),
	KEY(2, 2, KEY_J),
	KEY(2, 3, KEY_K),
	KEY(3, 0, KEY_M),
	KEY(3, 1, KEY_N),
	KEY(3, 2, KEY_O),
	KEY(3, 3, KEY_P)
};

static struct twl4030_keypad_data omap3evm_kp_data = {
	.rows		= 4,
	.cols		= 4,
	.keymap		= omap3evm_keymap,
	.keymapsize	= ARRAY_SIZE(omap3evm_keymap),
	.rep		= 1,
};

static struct twl4030_madc_platform_data omap3evm_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_platform_data omap3evm_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.keypad		= &omap3evm_kp_data,
	.madc		= &omap3evm_madc_data,
	.usb		= &omap3evm_usb_data,
	.power		= GENERIC3430_T2SCRIPTS_DATA,
	.gpio		= &omap3evm_gpio_data,
};

static struct i2c_board_info __initdata omap3evm_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &omap3evm_twldata,
	},
};

static int __init omap3_evm_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, omap3evm_i2c_boardinfo,
			ARRAY_SIZE(omap3evm_i2c_boardinfo));
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}

#define LCD_PANEL_LR		2
#define LCD_PANEL_UD		3
#define LCD_PANEL_INI		152
#define LCD_PANEL_ENABLE_GPIO	153
#define LCD_PANEL_QVGA		154
#define LCD_PANEL_RESB		155

#define ENABLE_VDAC_DEDICATED	0x03
#define ENABLE_VDAC_DEV_GRP	0x20
#define ENABLE_VPLL2_DEDICATED	0x05
#define ENABLE_VPLL2_DEV_GRP	0xE0

#define TWL4030_GPIODATA_IN3	0x03
#define TWL4030_GPIODATA_DIR3	0x06
#define TWL4030_VPLL2_DEV_GRP	0x33
#define TWL4030_VPLL2_DEDICATED	0x36

static int lcd_enabled;
static int dvi_enabled;

static void enable_vpll2(int enable)
{
	u8 ded_val, grp_val;

	if (enable) {
		ded_val = ENABLE_VPLL2_DEDICATED;
		grp_val = ENABLE_VPLL2_DEV_GRP;
	} else {
		ded_val = 0;
		grp_val = 0;
	}

	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ded_val, TWL4030_VPLL2_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			grp_val, TWL4030_VPLL2_DEV_GRP);
}

static int omap3evm_dsi_power_up(void)
{
	if (omap_rev() > OMAP3430_REV_ES1_0)
		enable_vpll2(1);
	return 0;
}

static void omap3evm_dsi_power_down(void)
{
	if (omap_rev() > OMAP3430_REV_ES1_0)
		enable_vpll2(0);
}


static void __init omap3_evm_display_init(void)
{
	int r;
	r = gpio_request(LCD_PANEL_LR, "lcd_panel_lr");
	if (r) {
		printk(KERN_ERR "failed to get LCD_PANEL_LR\n");
		return;
	}
	r = gpio_request(LCD_PANEL_UD, "lcd_panel_ud");
	if (r) {
		printk(KERN_ERR "failed to get LCD_PANEL_UD\n");
		goto err_1;
	}

	r = gpio_request(LCD_PANEL_INI, "lcd_panel_ini");
	if (r) {
		printk(KERN_ERR "failed to get LCD_PANEL_INI\n");
		goto err_2;
	}
	r = gpio_request(LCD_PANEL_RESB, "lcd_panel_resb");
	if (r) {
		printk(KERN_ERR "failed to get LCD_PANEL_RESB\n");
		goto err_3;
	}
	r = gpio_request(LCD_PANEL_QVGA, "lcd_panel_qvga");
	if (r) {
		printk(KERN_ERR "failed to get LCD_PANEL_QVGA\n");
		goto err_4;
	}

	gpio_direction_output(LCD_PANEL_LR, 0);
	gpio_direction_output(LCD_PANEL_UD, 0);
	gpio_direction_output(LCD_PANEL_INI, 0);
	gpio_direction_output(LCD_PANEL_RESB, 0);
	gpio_direction_output(LCD_PANEL_QVGA, 0);

#define TWL_LED_LEDEN           0x00
#define TWL_PWMA_PWMAON         0x00
#define TWL_PWMA_PWMAOFF        0x01

	twl4030_i2c_write_u8(TWL4030_MODULE_LED, 0x11, TWL_LED_LEDEN);
	twl4030_i2c_write_u8(TWL4030_MODULE_PWMA, 0x01, TWL_PWMA_PWMAON);
	twl4030_i2c_write_u8(TWL4030_MODULE_PWMA, 0x02, TWL_PWMA_PWMAOFF);

	gpio_direction_output(LCD_PANEL_RESB, 1);
	gpio_direction_output(LCD_PANEL_INI, 1);
	gpio_direction_output(LCD_PANEL_QVGA, 0);
	gpio_direction_output(LCD_PANEL_LR, 1);
	gpio_direction_output(LCD_PANEL_UD, 1);

	return;

err_4:
	gpio_free(LCD_PANEL_RESB);
err_3:
	gpio_free(LCD_PANEL_INI);
err_2:
	gpio_free(LCD_PANEL_UD);
err_1:
	gpio_free(LCD_PANEL_LR);

}

static int omap3_evm_panel_enable_lcd(struct omap_display *display)
{
	if (dvi_enabled) {
		printk(KERN_ERR "cannot enable LCD, DVI is enabled\n");
		return -EINVAL;
	}
	omap3evm_dsi_power_up();
	gpio_direction_output(LCD_PANEL_ENABLE_GPIO, 0);
	lcd_enabled = 1;
	return 0;
}

static void omap3_evm_panel_disable_lcd(struct omap_display *display)
{
	omap3evm_dsi_power_down();
	gpio_direction_output(LCD_PANEL_ENABLE_GPIO, 1);
	lcd_enabled = 0;
}

static struct omap_dss_display_config omap3_evm_display_data = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "lcd",
	.panel_name = "sharp-ls037v7dw01",
	.u.dpi.data_lines = 18,
	.panel_enable = omap3_evm_panel_enable_lcd,
	.panel_disable = omap3_evm_panel_disable_lcd,
};

static int omap3_evm_panel_enable_tv(struct omap_display *display)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEDICATED, TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
			ENABLE_VDAC_DEV_GRP, TWL4030_VDAC_DEV_GRP);
	return 0;
}

static void omap3_evm_panel_disable_tv(struct omap_display *display)
{
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEDICATED);
	twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, 0x00,
			TWL4030_VDAC_DEV_GRP);
}

static struct omap_dss_display_config omap3_evm_display_data_tv = {
	.type = OMAP_DISPLAY_TYPE_VENC,
	.name = "tv",
	.u.venc.type = OMAP_DSS_VENC_TYPE_SVIDEO,
	.panel_enable = omap3_evm_panel_enable_tv,
	.panel_disable = omap3_evm_panel_disable_tv,
};


static int omap3_evm_panel_enable_dvi(struct omap_display *display)
{
	if (lcd_enabled) {
		printk(KERN_ERR "cannot enable DVI, LCD is enabled\n");
		return -EINVAL;
	}

	omap3evm_dsi_power_up();
	twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0x80,
			TWL4030_GPIODATA_IN3);
	twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0x80,
			TWL4030_GPIODATA_DIR3);
	dvi_enabled = 1;

	return 0;
}

static void omap3_evm_panel_disable_dvi(struct omap_display *display)
{

	omap3evm_dsi_power_down();
	twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0x00,
			TWL4030_GPIODATA_IN3);
	twl4030_i2c_write_u8(TWL4030_MODULE_GPIO, 0x00,
			TWL4030_GPIODATA_DIR3);
	dvi_enabled = 0;
}


static struct omap_dss_display_config omap3_evm_display_data_dvi = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "dvi",
	.panel_name = "panel-generic",
	.u.dpi.data_lines = 24,
	.panel_enable = omap3_evm_panel_enable_dvi,
	.panel_disable = omap3_evm_panel_disable_dvi,
};

static struct omap_dss_board_info omap3_evm_dss_data = {
	.dsi_power_up = omap3evm_dsi_power_up,
	.dsi_power_down = omap3evm_dsi_power_down,
	.num_displays = 3,
	.displays = {
		&omap3_evm_display_data,
		&omap3_evm_display_data_dvi,
		&omap3_evm_display_data_tv,
	}
};
static struct platform_device omap3_evm_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev            = {
		.platform_data = &omap3_evm_dss_data,
	},
};

static void ads7846_dev_init(void)
{
	if (gpio_request(OMAP3_EVM_TS_GPIO, "ADS7846 pendown") < 0)
		printk(KERN_ERR "can't get ads7846 pen down GPIO\n");

	gpio_direction_input(OMAP3_EVM_TS_GPIO);

	omap_set_gpio_debounce(OMAP3_EVM_TS_GPIO, 1);
	omap_set_gpio_debounce_time(OMAP3_EVM_TS_GPIO, 0xa);
}

static int ads7846_get_pendown_state(void)
{
	return !gpio_get_value(OMAP3_EVM_TS_GPIO);
}

struct ads7846_platform_data ads7846_config = {
	.x_max                  = 0x0fff,
	.y_max                  = 0x0fff,
	.x_plate_ohms           = 180,
	.pressure_max           = 255,
	.debounce_max           = 10,
	.debounce_tol           = 3,
	.debounce_rep           = 1,
	.get_pendown_state	= ads7846_get_pendown_state,
	.keep_vref_on		= 1,
	.settle_delay_usecs     = 150,
};

static struct omap2_mcspi_device_config ads7846_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,  /* 0: slave, 1: master */
};

struct spi_board_info omap3evm_spi_board_info[] = {
	[0] = {
		.modalias		= "ads7846",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &ads7846_mcspi_config,
		.irq			= OMAP_GPIO_IRQ(OMAP3_EVM_TS_GPIO),
		.platform_data		= &ads7846_config,
	},
};

static void __init omap3_evm_init_irq(void)
{
	omap2_init_common_hw(mt46h32m32lf6_sdrc_params, omap3_mpu_rate_table,
	                     omap3_dsp_rate_table, omap3_l3_rate_table);
	omap_init_irq();
	omap_gpio_init();
	omap3evm_init_smc911x();
}

static struct omap_board_config_kernel omap3_evm_config[] __initdata = {
	{ OMAP_TAG_UART,	&omap3_evm_uart_config },
};

static struct platform_device *omap3_evm_devices[] __initdata = {
	&omap3_evm_dss_device,
	&omap3evm_smc911x_device,
};

static void __init omap3_evm_init(void)
{
	omap3_evm_i2c_init();

	platform_add_devices(omap3_evm_devices, ARRAY_SIZE(omap3_evm_devices));
	omap_board_config = omap3_evm_config;
	omap_board_config_size = ARRAY_SIZE(omap3_evm_config);

	spi_register_board_info(omap3evm_spi_board_info,
				ARRAY_SIZE(omap3evm_spi_board_info));

	omap_serial_init();
	usb_musb_init();
	usb_ehci_init();
	omap3evm_flash_init();
	ads7846_dev_init();
	omap3_evm_display_init();
}

static void __init omap3_evm_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(OMAP3EVM, "OMAP3 EVM")
	/* Maintainer: Syed Mohammed Khasim - Texas Instruments */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap3_evm_map_io,
	.init_irq	= omap3_evm_init_irq,
	.init_machine	= omap3_evm_init,
	.timer		= &omap_timer,
MACHINE_END
