/*
 * linux/arch/arm/mach-omap2/board-mapphone-camera.c
 *
 * Copyright (C) 2009 Motorola, Inc.
 *
 * Derived from mach-omap3/board-3430sdp.c
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

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <mach/mux.h>
#include <mach/board-sholes.h>
#include <mach/omap-pm.h>

#ifdef CONFIG_VIDEO_OLDOMAP3
#include <media/v4l2-int-device.h>
#include <../drivers/media/video/oldomap34xxcam.h>
#include <../drivers/media/video/oldisp/ispreg.h>
#include <../drivers/media/video/oldisp/isp.h>
#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
#include <media/mt9p012.h>
#endif
#endif

#ifdef CONFIG_VIDEO_OMAP3_HPLENS
#include <../drivers/media/video/hplens.h>
#endif

static void sholes_camera_lines_safe_mode(void);
static void sholes_camera_lines_func_mode(void);

#ifdef CONFIG_VIDEO_OMAP3_HPLENS
static int hplens_power_set(enum v4l2_power power)
{
	(void)power;

	return 0;
}

static int hplens_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_LENS;

	return 0;
}

struct hplens_platform_data sholes_hplens_platform_data = {
	.power_set = hplens_power_set,
	.priv_data_set = hplens_set_prv_data,
};
#endif

#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
static struct omap34xxcam_sensor_config mt9p012_cam_hwc = {
	.sensor_isp = 0,
	.xclk = OMAP34XXCAM_XCLK_A,
	.capture_mem = PAGE_ALIGN(2592 * 1944 * 2) * 4,
};

static int mt9p012_sensor_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->u.sensor.xclk = mt9p012_cam_hwc.xclk;
	hwc->u.sensor.sensor_isp = mt9p012_cam_hwc.sensor_isp;
	hwc->u.sensor.capture_mem = mt9p012_cam_hwc.capture_mem;
	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;
	hwc->interface_type = ISP_PARLL;
	return 0;
}

static struct isp_interface_config mt9p012_if_config = {
	.ccdc_par_ser = ISP_PARLL,
	.dataline_shift = 0x1,
	.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE,
	.vdint0_timing = 0x0,
	.vdint1_timing = 0x0,
	.strobe = 0x0,
	.prestrobe = 0x0,
	.shutter = 0x0,
	.wenlog = ISPCCDC_CFG_WENLOG_OR,
	.dcsub = 42,
	.raw_fmt_in = ISPCCDC_INPUT_FMT_GR_BG,
	.wbal.coef0 = 0x23,
	.wbal.coef1 = 0x20,
	.wbal.coef2 = 0x20,
	.wbal.coef3 = 0x30,
	.u.par.par_bridge = 0x0,
	.u.par.par_clk_pol = 0x0,
};

static int mt9p012_sensor_power_set(struct device* dev, enum v4l2_power power)
{
	static enum v4l2_power previous_power = V4L2_POWER_OFF;
	static struct regulator *regulator;

	switch (power) {
	case V4L2_POWER_OFF:
		/* Power Down Sequence */
		gpio_free(GPIO_MT9P012_RESET);

		/* Turn off power */
		if (regulator != NULL) {
			regulator_disable(regulator);
			regulator_put(regulator);
			regulator = NULL;
		} else {
			sholes_camera_lines_safe_mode();
			pr_err("%s: Regulator for vcam is not "\
					"initialized\n", __func__);
			return -EIO;
		}

		/* Release pm constraints */
		omap_pm_set_min_bus_tput(dev, OCP_INITIATOR_AGENT, 0);
		sholes_camera_lines_safe_mode();
	break;
	case V4L2_POWER_ON:
		if (previous_power == V4L2_POWER_OFF) {
			/* Power Up Sequence */
			sholes_camera_lines_func_mode();

			/* Set min throughput to:
			 *  2592 x 1944 x 2bpp x 30fps x 3 L3 accesses */
			omap_pm_set_min_bus_tput(dev, OCP_INITIATOR_AGENT, 885735);

			/* Configure pixel clock divider (here?) */
			omap_writel(0x2, 0x48004f40);
			isp_configure_interface(&mt9p012_if_config);

			/* Request and configure gpio pins */
			if (gpio_request(GPIO_MT9P012_RESET,
						"mt9p012 camera reset") != 0)
				return -EIO;

			/* set to output mode */
			gpio_direction_output(GPIO_MT9P012_RESET, 0);

			/* nRESET is active LOW. set HIGH to release reset */
			gpio_set_value(GPIO_MT9P012_RESET, 1);

			/* turn on digital power */
			if (regulator != NULL) {
				pr_warning("%s: Already have "\
						"regulator\n", __func__);
			} else {
				regulator = regulator_get(NULL, "vcam");
				if (IS_ERR(regulator)) {
					pr_err("%s: Cannot get vcam "\
						"regulator, err=%ld\n",
						__func__, PTR_ERR(regulator));
					return PTR_ERR(regulator);
				}
			}

			if (regulator_enable(regulator) != 0) {
				pr_err("%s: Cannot enable vcam regulator\n",
						__func__);
				return -EIO;
			}
		}

		udelay(1000);

		if (previous_power == V4L2_POWER_OFF) {
			/* trigger reset */
			gpio_direction_output(GPIO_MT9P012_RESET, 0);

			udelay(1500);

			/* nRESET is active LOW. set HIGH to release reset */
			gpio_set_value(GPIO_MT9P012_RESET, 1);

			/* give sensor sometime to get out of the reset.
			 * Datasheet says 2400 xclks. At 6 MHz, 400 usec is
			 * enough
			 */
			udelay(300);
		}
		break;
	case V4L2_POWER_STANDBY:
		/* Stand By Sequence */
		break;
	}
	/* Save powerstate to know what was before calling POWER_ON. */
	previous_power = power;
	return 0;
}

u32 mt9p012_set_xclk(u32 xclkfreq)
{
	return isp_set_xclk(xclkfreq, OMAP34XXCAM_XCLK_A);
}


struct mt9p012_platform_data sholes_mt9p012_platform_data = {
	.power_set      = mt9p012_sensor_power_set,
	.set_xclk	= mt9p012_set_xclk,
	.priv_data_set  = mt9p012_sensor_set_prv_data,
};

#endif /* #ifdef CONFIG_VIDEO_MT9P012 || CONFIG_VIDEO_MT9P012_MODULE */

/* We can't change the IOMUX config after bootup
 * with the current pad configuration architecture,
 * the next two functions are hack to configure the
 * camera pads at runtime to save power in standby */

void sholes_camera_lines_safe_mode(void)
{
	omap_writew(0x0007, 0x4800210c);
	omap_writew(0x0007, 0x4800210e);
	omap_writew(0x0007, 0x48002110);
	omap_writew(0x0007, 0x48002112);
	omap_writew(0x0007, 0x48002114);
	omap_writew(0x011F, 0x4800211a);
	omap_writew(0x011F, 0x4800211c);
	omap_writew(0x011F, 0x4800211e);
	omap_writew(0x011F, 0x48002120);
	omap_writew(0x001F, 0x48002122);
	omap_writew(0x001F, 0x48002124);
	omap_writew(0x001F, 0x48002126);
	omap_writew(0x001F, 0x48002128);
	omap_writew(0x011F, 0x4800212a);
	omap_writew(0x011F, 0x4800212c);
}

void sholes_camera_lines_func_mode(void)
{
	omap_writew(0x0118, 0x4800210c);
	omap_writew(0x0118, 0x4800210e);
	omap_writew(0x0000, 0x48002110);
	omap_writew(0x0118, 0x48002112);
	omap_writew(0x0004, 0x48002114);
	omap_writew(0x0100, 0x4800211a);
	omap_writew(0x0100, 0x4800211c);
	omap_writew(0x0100, 0x4800211e);
	omap_writew(0x0100, 0x48002120);
	omap_writew(0x0100, 0x48002122);
	omap_writew(0x0100, 0x48002124);
	omap_writew(0x0100, 0x48002126);
	omap_writew(0x0100, 0x48002128);
	omap_writew(0x0100, 0x4800212a);
	omap_writew(0x0100, 0x4800212c);
}

void __init sholes_camera_init(void)
{
	omap_cfg_reg(A24_34XX_CAM_HS);
	omap_cfg_reg(A23_34XX_CAM_VS);
	omap_cfg_reg(C25_34XX_CAM_XCLKA);
	omap_cfg_reg(C27_34XX_CAM_PCLK);
	omap_cfg_reg(C23_34XX_CAM_FLD);
	omap_cfg_reg(AG17_34XX_CAM_D0);
	omap_cfg_reg(AH17_34XX_CAM_D1);
	omap_cfg_reg(B24_34XX_CAM_D2);
	omap_cfg_reg(C24_34XX_CAM_D3);
	omap_cfg_reg(D24_34XX_CAM_D4);
	omap_cfg_reg(A25_34XX_CAM_D5);
	omap_cfg_reg(K28_34XX_CAM_D6);
	omap_cfg_reg(L28_34XX_CAM_D7);
	omap_cfg_reg(K27_34XX_CAM_D8);
	omap_cfg_reg(L27_34XX_CAM_D9);
	omap_cfg_reg(B25_34XX_CAM_D10);
	omap_cfg_reg(C26_34XX_CAM_D11);
	omap_cfg_reg(B23_34XX_CAM_WEN);
	omap_cfg_reg(D25_34XX_CAM_STROBE);
	omap_cfg_reg(K8_34XX_GPMC_WAIT2);

	sholes_camera_lines_safe_mode();
}

