/*
 * linux/drivers/video/omap2/dss/sdi.c
 *
 * Copyright (C) 2009 Nokia Corporation
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

#define DSS_SUBSYS_NAME "SDI"

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>

#include <mach/board.h>
#include <mach/display.h>
#include "dss.h"

#define CONTROL_PADCONF_BASE	0x48002000

#define OMAP_SDI_PAD_DIS(pe,pu)	((7 << 0)		| /* MODE 7 = safe */ \
				 (((pe) ? 1 : 0) << 3)	| /* PULL_ENA */      \
				 (((pu) ? 1 : 0) << 4)	| /* PULL_UP  */      \
				 (1 << 8))		  /* INPUT_EN */

#define OMAP_SDI_PAD_EN		 (1 << 0)		  /* MODE 1 = SDI_xx */

#define OMAP_SDI_PAD_MASK	OMAP_SDI_PAD_DIS(1, 1)

static struct {
	bool skip_init;
	bool update_enabled;
} sdi;

/* CONTROL_PADCONF_DSS_DATAXX */
static const u16 sdi_pads[] =
{
	0x0f0,		/* 10[ 7..0]:SDI_DAT1N */
	0x0f2,		/* 10[15..0]:SDI_DAT1P */
	0x0f4,		/* 12[ 7..0]:SDI_DAT2N */
	0x0f6,		/* 12[15..0]:SDI_DAT2P */
	0x0f8,		/* 14[ 7..0]:SDI_DAT3N */
	0x0fa,		/* 14[15..0]:SDI_DAT3P */
	0x108,		/* 22[ 7..0]:SDI_CLKN */
	0x10a,		/* 22[15..0]:SDI_CLKP */
};

/*
 * Check if bootloader / platform code has configured the SDI pads properly.
 * This means it either configured all required pads for SDI mode, or that it
 * left all the required pads unconfigured.
 */
static int sdi_pad_init(struct omap_dss_device *dssdev)
{
	unsigned req_map;
	bool configured = false;
	bool unconfigured = false;
	int data_pairs;
	int i;

	data_pairs = dssdev->phy.sdi.datapairs;
	req_map = (1 << (data_pairs * 2)) - 1;		/* data lanes */
	req_map |= 3 << 6;				/* clk lane */
	for (i = 0; i < ARRAY_SIZE(sdi_pads); i++) {
		u32 reg;
		u32 val;

		if (!((1 << i) & req_map))
			/* Ignore unneded pads. */
			continue;
		reg = CONTROL_PADCONF_BASE + sdi_pads[i];
		val = omap_readw(reg);
		switch (val & 0x07) {	/* pad mode */
		case 1:
			if (unconfigured)
				break;
			/* Is the pull configuration ok for SDI mode? */
			if ((val & OMAP_SDI_PAD_MASK) != OMAP_SDI_PAD_EN)
				break;
			configured = true;
			break;
		case 0:
		case 7:
			if (configured)
				break;
			unconfigured = true;
			break;
		default:
			break;
		}
	}
	if (i != ARRAY_SIZE(sdi_pads)) {
		DSSERR("SDI: invalid pad configuration\n");
		return -1;
	}

	return 0;
}

static void sdi_pad_config(struct omap_dss_device *dssdev, bool enable)
{
	int data_pairs;
	bool pad_off_pe, pad_off_pu;
	unsigned req_map;
	int i;

	data_pairs = dssdev->phy.sdi.datapairs;
	pad_off_pe = dssdev->phy.sdi.pad_off_pe;
	pad_off_pu = dssdev->phy.sdi.pad_off_pu;
	req_map = (1 << (data_pairs * 2)) - 1;		/* data lanes */
	req_map |= 3 << 6;				/* clk lane */
	for (i = 0; i < ARRAY_SIZE(sdi_pads); i++) {
		u32 reg;
		u16 val;

		if (!((1 << i) & req_map))
			continue;
		if (enable)
			val = OMAP_SDI_PAD_EN;
		else
			val = OMAP_SDI_PAD_DIS(pad_off_pe, pad_off_pu);
		reg = CONTROL_PADCONF_BASE + sdi_pads[i];
		omap_writew(val, reg);
	}
}

static void sdi_basic_init(void)
{
	dispc_set_parallel_interface_mode(OMAP_DSS_PARALLELMODE_BYPASS);

	dispc_set_lcd_display_type(OMAP_DSS_LCD_DISPLAY_TFT);
	dispc_set_tft_data_lines(24);
	dispc_lcd_enable_signal_polarity(1);
}

static int sdi_display_enable(struct omap_dss_device *dssdev)
{
	struct omap_video_timings *t = &dssdev->panel.timings;
	struct dispc_clock_info cinfo;
	u16 lck_div, pck_div;
	unsigned long fck;
	unsigned long pck;
	int r;

	r = omap_dss_start_device(dssdev);
	if (r) {
		DSSERR("failed to start device\n");
		goto err0;
	}

	if (dssdev->state != OMAP_DSS_DISPLAY_DISABLED) {
		DSSERR("dssdev already enabled\n");
		r = -EINVAL;
		goto err1;
	}

	sdi_pad_config(dssdev, 1);

	/* In case of skip_init sdi_init has already enabled the clocks */
	if (!sdi.skip_init)
		dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);

	sdi_basic_init();

	/* 15.5.9.1.2 */
	dssdev->panel.config |= OMAP_DSS_LCD_RF | OMAP_DSS_LCD_ONOFF;

	dispc_set_pol_freq(dssdev->panel.config, dssdev->panel.acbi,
			dssdev->panel.acb);

	if (!sdi.skip_init)
		r = dispc_calc_clock_div(1, t->pixel_clock * 1000,
				&cinfo);
	else
		r = dispc_get_clock_div(&cinfo);

	if (r)
		goto err2;

	fck = cinfo.fck;
	lck_div = cinfo.lck_div;
	pck_div = cinfo.pck_div;

	pck = fck / lck_div / pck_div / 1000;

	if (pck != t->pixel_clock) {
		DSSWARN("Could not find exact pixel clock. Requested %d kHz, "
				"got %lu kHz\n",
				t->pixel_clock, pck);

		t->pixel_clock = pck;
	}


	dispc_set_lcd_timings(t);

	r = dispc_set_clock_div(&cinfo);
	if (r)
		goto err2;

	if (!sdi.skip_init) {
		dss_sdi_init(dssdev->phy.sdi.datapairs);
		dss_sdi_enable();
		mdelay(2);
	}

	dispc_enable_lcd_out(1);

	if (dssdev->driver->enable) {
		r = dssdev->driver->enable(dssdev);
		if (r)
			goto err3;
	}

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	sdi.skip_init = 0;

	return 0;
err3:
	dispc_enable_lcd_out(0);
err2:
	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);
err1:
	omap_dss_stop_device(dssdev);
err0:
	return r;
}

static int sdi_display_resume(struct omap_dss_device *dssdev);

static void sdi_display_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->state == OMAP_DSS_DISPLAY_DISABLED)
		return;

	if (dssdev->state == OMAP_DSS_DISPLAY_SUSPENDED)
		sdi_display_resume(dssdev);

	if (dssdev->driver->disable)
		dssdev->driver->disable(dssdev);

	dispc_enable_lcd_out(0);

	dss_sdi_disable();

	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);
	sdi_pad_config(dssdev, 0);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;

	omap_dss_stop_device(dssdev);
}

static int sdi_display_suspend(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return -EINVAL;

	if (dssdev->driver->suspend)
		dssdev->driver->suspend(dssdev);

	dispc_enable_lcd_out(0);

	dss_sdi_disable();

	dss_clk_disable(DSS_CLK_ICK | DSS_CLK_FCK1);
	sdi_pad_config(dssdev, 0);

	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;

	return 0;
}

static int sdi_display_resume(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_SUSPENDED)
		return -EINVAL;

	sdi_pad_config(dssdev, 1);
	dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);

	dss_sdi_enable();
	mdelay(2);

	dispc_enable_lcd_out(1);

	if (dssdev->driver->resume)
		dssdev->driver->resume(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static int sdi_display_set_update_mode(struct omap_dss_device *dssdev,
		enum omap_dss_update_mode mode)
{
	if (mode == OMAP_DSS_UPDATE_MANUAL)
		return -EINVAL;

	if (mode == OMAP_DSS_UPDATE_DISABLED) {
		dispc_enable_lcd_out(0);
		sdi.update_enabled = 0;
	} else {
		dispc_enable_lcd_out(1);
		sdi.update_enabled = 1;
	}

	return 0;
}

static enum omap_dss_update_mode sdi_display_get_update_mode(
		struct omap_dss_device *dssdev)
{
	return sdi.update_enabled ? OMAP_DSS_UPDATE_AUTO :
		OMAP_DSS_UPDATE_DISABLED;
}

static void sdi_get_timings(struct omap_dss_device *dssdev,
			struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

int sdi_init_display(struct omap_dss_device *dssdev)
{
	DSSDBG("SDI init\n");

	dssdev->enable = sdi_display_enable;
	dssdev->disable = sdi_display_disable;
	dssdev->suspend = sdi_display_suspend;
	dssdev->resume = sdi_display_resume;
	dssdev->set_update_mode = sdi_display_set_update_mode;
	dssdev->get_update_mode = sdi_display_get_update_mode;
	dssdev->get_timings = sdi_get_timings;

	return sdi_pad_init(dssdev);
}

int sdi_init(bool skip_init)
{
	/* we store this for first display enable, then clear it */
	sdi.skip_init = skip_init;

	/*
	 * Enable clocks already here, otherwise there would be a toggle
	 * of them until sdi_display_enable is called.
	 */
	if (skip_init)
		dss_clk_enable(DSS_CLK_ICK | DSS_CLK_FCK1);
	return 0;
}

void sdi_exit(void)
{
}
