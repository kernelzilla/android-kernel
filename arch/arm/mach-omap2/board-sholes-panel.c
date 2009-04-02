/*
 * linux/arch/arm/mach-omap2/board-sholes-panel.c
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
#include <linux/delay.h>

#include <mach/display.h>
#include <mach/gpio.h>
#include <mach/mux.h>


#define SHOLES_DISPLAY_RESET_GPIO	136

static void sholes_dsi_enable(int enable)
{
	/* XXX power managment to enable dsi block */
#if 0
	u8 ded_val, grp_val;

	
	if (omap_rev() <= OMAP3430_REV_ES1_0)
		return;

	if (enable) {
		ded_val = ENABLE_VPLL2_DEDICATED;
		grp_val = ENABLE_VPLL2_DEV_GRP;
	} else {
		ded_val = 0;
                grp_val = 0;
	}

	twl4030_i2c_write_u8(PM_RECEIVER, ded_val, TWL4030_VPLL2_DEDICATED);
	twl4030_i2c_write_u8(PM_RECEIVER, grp_val, TWL4030_VPLL2_DEV_GRP);
#endif 
}

static int sholes_dsi_power_up(void)
{
	sholes_dsi_enable(1);
	return 0;
}

static void sholes_dsi_power_down(void)
{
	sholes_dsi_enable(0);
}

static int sholes_panel_enable_lcd(struct omap_display *display)
{
	/* backlight enable? */
	return 0;
}

static void sholes_panel_disable_lcd(struct omap_display *display)
{
#if 0
	/* backlight disable? */

#endif
}

static int sholes_edisco_ctrl_enable(struct omap_display *display)
{
	gpio_request(SHOLES_DISPLAY_RESET_GPIO, "display reset");
	gpio_direction_output(SHOLES_DISPLAY_RESET_GPIO, 1);
	msleep(5);
	gpio_set_value(SHOLES_DISPLAY_RESET_GPIO, 0);
	msleep(5);
	gpio_set_value(SHOLES_DISPLAY_RESET_GPIO, 1);
	msleep(10);

	return 0;
}

static void sholes_edisco_ctrl_disable(struct omap_display *display)
{
	gpio_direction_output(SHOLES_DISPLAY_RESET_GPIO, 1);
	gpio_set_value(SHOLES_DISPLAY_RESET_GPIO, 0);

	return;
}

static struct omap_dss_display_config sholes_display_data_lcd = {
	.type = OMAP_DISPLAY_TYPE_DSI,
	.name = "lcd",
	.ctrl_name = "ctrl-edisco",
	.panel_name = "panel-sholes",
	.u.dsi.clk_lane = 1,
	.u.dsi.clk_pol = 0,
	.u.dsi.data1_lane = 2,
	.u.dsi.data1_pol = 0,
	.u.dsi.data2_lane = 3,
	.u.dsi.data2_pol = 0,
	.u.dsi.ddr_clk_hz = 150000000,
	.panel_enable = sholes_panel_enable_lcd,
	.panel_disable = sholes_panel_disable_lcd,
	.ctrl_enable = sholes_edisco_ctrl_enable,
	.ctrl_disable = sholes_edisco_ctrl_disable,
};

static struct omap_dss_board_info sholes_dss_data = {
	.dsi_power_up = sholes_dsi_power_up,
	.dsi_power_down = sholes_dsi_power_down,
	.num_displays = 1,
	.displays = {
		&sholes_display_data_lcd,
	}
};

static struct platform_device sholes_dss_device = {
        .name          = "omapdss",
        .id            = -1,
        .dev            = {
                .platform_data = &sholes_dss_data,
        },
};

void __init sholes_panel_init(void)
{
	int ret;

	omap_cfg_reg(AG22_34XX_DSI_DX0);
	omap_cfg_reg(AH22_34XX_DSI_DY0);
	omap_cfg_reg(AG23_34XX_DSI_DX1);
	omap_cfg_reg(AH23_34XX_DSI_DY1);
	omap_cfg_reg(AG24_34XX_DSI_DX2);
	omap_cfg_reg(AH24_34XX_DSI_DY2);
	/* disp reset b */
	omap_cfg_reg(AE4_34XX_GPIO136_OUT);

	ret = gpio_request(SHOLES_DISPLAY_RESET_GPIO, "display reset");
	if (ret) {
		printk(KERN_ERR "failed to get display reset gpio\n");
		goto error;
	}

	platform_device_register(&sholes_dss_device);
	return;

error:
	gpio_free(SHOLES_DISPLAY_RESET_GPIO);
}
EXPORT_SYMBOL(sholes_panel_init);
