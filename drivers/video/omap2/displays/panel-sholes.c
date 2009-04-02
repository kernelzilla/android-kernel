/*
 * Generic panel support
 *
 * Copyright (C) 2008 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>

#include <mach/display.h>

static int sholes_panel_init(struct omap_display *display)
{
	return 0;
}

static int sholes_panel_enable(struct omap_display *display)
{
	int r = 0;

	if (display->hw_config.panel_enable)
		r = display->hw_config.panel_enable(display);

	return r;
}

static void sholes_panel_disable(struct omap_display *display)
{
	if (display->hw_config.panel_disable)
		display->hw_config.panel_disable(display);
}

static int sholes_panel_suspend(struct omap_display *display)
{
	sholes_panel_disable(display);
	return 0;
}

static int sholes_panel_resume(struct omap_display *display)
{
	return sholes_panel_enable(display);
}

static struct omap_panel sholes_panel = {
	.owner		= THIS_MODULE,
	.name		= "panel-sholes",
	.init		= sholes_panel_init,
	.enable		= sholes_panel_enable,
	.disable	= sholes_panel_disable,
	.suspend	= sholes_panel_suspend,
	.resume		= sholes_panel_resume,

	.timings = {
		.x_res		= 480,
		.y_res		= 854,
		/*.pixel_clock	= 25000,*/
		.hfp		= 44,
		.hsw		= 2,
		.hbp		= 38,
		.vfp		= 1,
		.vsw		= 1,
		.vbp		= 1,
	},

	.config		= OMAP_DSS_LCD_TFT,
};


static int __init sholes_panel_drv_init(void)
{
	omap_dss_register_panel(&sholes_panel);
	return 0;
}

static void __exit sholes_panel_drv_exit(void)
{
	omap_dss_unregister_panel(&sholes_panel);
}

module_init(sholes_panel_drv_init);
module_exit(sholes_panel_drv_exit);
MODULE_LICENSE("GPL");
