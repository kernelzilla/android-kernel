/*
 * linux/arch/arm/mach-omap2/board-zoom2-camera.c
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

#ifdef CONFIG_TWL4030_CORE

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mm.h>

#include <linux/i2c/twl4030.h>

#include <asm/io.h>

#include <mach/gpio.h>

static int cam_inited;
#include <media/v4l2-int-device.h>
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>
#define DEBUG_BASE		0x08000000

#define REG_SDP3430_FPGA_GPIO_2 (0x50)
#define FPGA_SPR_GPIO1_3v3	(0x1 << 14)
#define FPGA_GPIO6_DIR_CTRL	(0x1 << 6)

#define VAUX_2_8_V		0x09
#define VAUX_1_8_V		0x05
#define VAUX_DEV_GRP_P1		0x20
#define VAUX_DEV_GRP_NONE	0x00

/* Sensor specific GPIO signals */
#define IMX046_RESET_GPIO  	98
#define IMX046_STANDBY_GPIO	58

#if defined(CONFIG_VIDEO_IMX046) || defined(CONFIG_VIDEO_IMX046_MODULE)
#include <media/imx046.h>
#include <../drivers/media/video/isp/ispcsi2.h>
#define IMX046_CSI2_CLOCK_POLARITY	0	/* +/- pin order */
#define IMX046_CSI2_DATA0_POLARITY	0	/* +/- pin order */
#define IMX046_CSI2_DATA1_POLARITY	0	/* +/- pin order */
#define IMX046_CSI2_CLOCK_LANE		1	 /* Clock lane position: 1 */
#define IMX046_CSI2_DATA0_LANE		2	 /* Data0 lane position: 2 */
#define IMX046_CSI2_DATA1_LANE		3	 /* Data1 lane position: 3 */
#define IMX046_CSI2_PHY_THS_TERM	2
#define IMX046_CSI2_PHY_THS_SETTLE	23
#define IMX046_CSI2_PHY_TCLK_TERM	0
#define IMX046_CSI2_PHY_TCLK_MISS	1
#define IMX046_CSI2_PHY_TCLK_SETTLE	14
#define IMX046_BIGGEST_FRAME_BYTE_SIZE	PAGE_ALIGN(3280 * 2464 * 2)
#endif

#ifdef CONFIG_VIDEO_LV8093
#include <media/lv8093.h>
#define LV8093_PS_GPIO			7
/* GPIO7 is connected to lens PS pin through inverter */
#define LV8093_PWR_OFF			1
#define LV8093_PWR_ON			(!LV8093_PWR_OFF)
#endif


#ifdef CONFIG_VIDEO_LV8093
static int lv8093_lens_power_set(enum v4l2_power power)
{
	static enum v4l2_power previous_pwr = V4L2_POWER_OFF;

	switch (power) {
	case V4L2_POWER_ON:
		printk(KERN_DEBUG "lv8093_lens_power_set(ON)\n");
		if (previous_pwr == V4L2_POWER_OFF) {
			if (gpio_request(LV8093_PS_GPIO, "lv8093_ps") != 0) {
				printk(KERN_WARNING "Could not request GPIO %d"
					" for LV8093\n", LV8093_PS_GPIO);
				return -EIO;
			}

			gpio_set_value(LV8093_PS_GPIO, LV8093_PWR_OFF);
			gpio_direction_output(LV8093_PS_GPIO, true);
		}
		gpio_set_value(LV8093_PS_GPIO, LV8093_PWR_ON);
		break;
	case V4L2_POWER_OFF:
		printk(KERN_DEBUG "lv8093_lens_power_set(OFF)\n");
		gpio_free(LV8093_PS_GPIO);
		break;
	case V4L2_POWER_STANDBY:
		printk(KERN_DEBUG "lv8093_lens_power_set(STANDBY)\n");
		gpio_set_value(LV8093_PS_GPIO, LV8093_PWR_OFF);
		break;
	}
	previous_pwr = power;
	return 0;
}

static int lv8093_lens_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->dev_index = 2;
	hwc->dev_minor = 5;
	hwc->dev_type = OMAP34XXCAM_SLAVE_LENS;
	return 0;
}

struct lv8093_platform_data zoom2_lv8093_platform_data = {
	.power_set      = lv8093_lens_power_set,
	.priv_data_set  = lv8093_lens_set_prv_data,
};
#endif

#if defined(CONFIG_VIDEO_IMX046) || defined(CONFIG_VIDEO_IMX046_MODULE)

static struct omap34xxcam_sensor_config imx046_hwc = {
	.sensor_isp  = 0,
	.xclk        = OMAP34XXCAM_XCLK_B,
	.capture_mem = IMX046_BIGGEST_FRAME_BYTE_SIZE * 2,
	.ival_default	= { 1, 10 },
};

static int imx046_sensor_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->u.sensor.xclk	= imx046_hwc.xclk;
	hwc->u.sensor.sensor_isp = imx046_hwc.sensor_isp;
	hwc->dev_index		= 2;
	hwc->dev_minor		= 5;
	hwc->dev_type		= OMAP34XXCAM_SLAVE_SENSOR;

	return 0;
}

static struct isp_interface_config imx046_if_config = {
	.ccdc_par_ser 		= ISP_CSIA,
	.dataline_shift 	= 0x0,
	.hsvs_syncdetect 	= ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe 		= 0x0,
	.prestrobe 		= 0x0,
	.shutter 		= 0x0,
	.wenlog 		= ISPCCDC_CFG_WENLOG_AND,
	.wait_hs_vs		= 2,
	.raw_fmt_in		= ISPCCDC_INPUT_FMT_GR_BG,
	.u.csi.crc 		= 0x0,
	.u.csi.mode 		= 0x0,
	.u.csi.edge 		= 0x0,
	.u.csi.signalling 	= 0x0,
	.u.csi.strobe_clock_inv = 0x0,
	.u.csi.vs_edge 		= 0x0,
	.u.csi.channel 		= 0x0,
	.u.csi.vpclk 		= 0x2,
	.u.csi.data_start 	= 0x0,
	.u.csi.data_size 	= 0x0,
	.u.csi.format 		= V4L2_PIX_FMT_SGRBG10,
};


static int imx046_sensor_power_set(enum v4l2_power power)
{
	struct isp_csi2_lanes_cfg lanecfg;
	struct isp_csi2_phy_cfg phyconfig;
	static enum v4l2_power previous_power = V4L2_POWER_OFF;
	int err = 0;

	switch (power) {
	case V4L2_POWER_ON:
		/* Power Up Sequence */
		printk(KERN_DEBUG "imx046_sensor_power_set(ON)\n");
		isp_csi2_reset();

		lanecfg.clk.pol = IMX046_CSI2_CLOCK_POLARITY;
		lanecfg.clk.pos = IMX046_CSI2_CLOCK_LANE;
		lanecfg.data[0].pol = IMX046_CSI2_DATA0_POLARITY;
		lanecfg.data[0].pos = IMX046_CSI2_DATA0_LANE;
		lanecfg.data[1].pol = IMX046_CSI2_DATA1_POLARITY;
		lanecfg.data[1].pos = IMX046_CSI2_DATA1_LANE;
		lanecfg.data[2].pol = 0;
		lanecfg.data[2].pos = 0;
		lanecfg.data[3].pol = 0;
		lanecfg.data[3].pos = 0;
		isp_csi2_complexio_lanes_config(&lanecfg);
		isp_csi2_complexio_lanes_update(true);

		isp_csi2_ctrl_config_ecc_enable(true);

		phyconfig.ths_term = IMX046_CSI2_PHY_THS_TERM;
		phyconfig.ths_settle = IMX046_CSI2_PHY_THS_SETTLE;
		phyconfig.tclk_term = IMX046_CSI2_PHY_TCLK_TERM;
		phyconfig.tclk_miss = IMX046_CSI2_PHY_TCLK_MISS;
		phyconfig.tclk_settle = IMX046_CSI2_PHY_TCLK_SETTLE;
		isp_csi2_phy_config(&phyconfig);
		isp_csi2_phy_update(true);

		isp_configure_interface(&imx046_if_config);

		if (previous_power == V4L2_POWER_OFF) {
			/* Request and configure gpio pins */
			if (gpio_request(IMX046_RESET_GPIO, "imx046_rst") != 0)
				return -EIO;

			/* nRESET is active LOW. set HIGH to release reset */
			gpio_set_value(IMX046_RESET_GPIO, 1);

			/* set to output mode */
			gpio_direction_output(IMX046_RESET_GPIO, true);

			/* turn on analog power */
			twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_2_8_V, TWL4030_VAUX2_DEDICATED);
			twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_DEV_GRP_P1, TWL4030_VAUX2_DEV_GRP);

			twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_1_8_V, TWL4030_VAUX4_DEDICATED);
			twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_DEV_GRP_P1, TWL4030_VAUX4_DEV_GRP);
			udelay(100);

			/* have to put sensor to reset to guarantee detection */
			gpio_set_value(IMX046_RESET_GPIO, 0);
			udelay(1500);

			/* nRESET is active LOW. set HIGH to release reset */
			gpio_set_value(IMX046_RESET_GPIO, 1);
			udelay(300);
		}
		break;
	case V4L2_POWER_OFF:
		printk(KERN_DEBUG "imx046_sensor_power_set(OFF)\n");
		/* Power Down Sequence */
		isp_csi2_complexio_power(ISP_CSI2_POWER_OFF);

		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				VAUX_DEV_GRP_NONE, TWL4030_VAUX4_DEV_GRP);
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				VAUX_DEV_GRP_NONE, TWL4030_VAUX2_DEV_GRP);
		gpio_free(IMX046_RESET_GPIO);
		break;
	case V4L2_POWER_STANDBY:
		printk(KERN_DEBUG "imx046_sensor_power_set(STANDBY)\n");
		/*TODO*/
		break;
	}

	/* Save powerstate to know what was before calling POWER_ON. */
	previous_power = power;
	return err;
}

struct imx046_platform_data zoom2_imx046_platform_data = {
	.power_set            = imx046_sensor_power_set,
	.priv_data_set        = imx046_sensor_set_prv_data,
	.set_xclk             = isp_set_xclk,
	.csi2_lane_count      = isp_csi2_complexio_lanes_count,
	.csi2_cfg_vp_out_ctrl = isp_csi2_ctrl_config_vp_out_ctrl,
	.csi2_ctrl_update     = isp_csi2_ctrl_update,
	.csi2_cfg_virtual_id  = isp_csi2_ctx_config_virtual_id,
	.csi2_ctx_update      = isp_csi2_ctx_update,
	.csi2_calc_phy_cfg0   = isp_csi2_calc_phy_cfg0,
};
#endif

void __init zoom2_cam_init(void)
{
	cam_inited = 0;
	/* Request and configure gpio pins */
	if (gpio_request(IMX046_STANDBY_GPIO, "ov3640_standby_gpio") != 0) {
		printk(KERN_ERR "Could not request GPIO %d",
					IMX046_STANDBY_GPIO);
		return;
	}

	/* set to output mode */
	gpio_direction_output(IMX046_STANDBY_GPIO, true);

	cam_inited = 1;
}
#else
void __init zoom2_cam_init(void)
{
}
#endif
