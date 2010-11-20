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
#include <mach/board-mapphone.h>
#include <mach/omap-pm.h>
#include <mach/control.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

#ifdef CONFIG_VIDEO_OLDOMAP3
#include <media/v4l2-int-device.h>
#include <../drivers/media/video/oldomap34xxcam.h>
#include <../drivers/media/video/oldisp/ispreg.h>
#include <../drivers/media/video/oldisp/isp.h>
#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
#include <media/mt9p012.h>
#endif
#endif
#if defined(CONFIG_VIDEO_OV8810) || defined(CONFIG_VIDEO_OV8810_MODULE)
#include <media/ov8810.h>
#include <../drivers/media/video/oldisp/ispcsi2.h>
#if defined(CONFIG_LEDS_FLASH_RESET)
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#endif
#define OV8810_CSI2_CLOCK_POLARITY	0	/* +/- pin order */
#define OV8810_CSI2_DATA0_POLARITY	0	/* +/- pin order */
#define OV8810_CSI2_DATA1_POLARITY	0	/* +/- pin order */
#define OV8810_CSI2_CLOCK_LANE		1	 /* Clock lane position: 1 */
#define OV8810_CSI2_DATA0_LANE		2	 /* Data0 lane position: 2 */
#define OV8810_CSI2_DATA1_LANE		3	 /* Data1 lane position: 3 */
#define OV8810_CSI2_PHY_THS_TERM	1  /* GVH */
#define OV8810_CSI2_PHY_THS_SETTLE	21  /* GVH */
#define OV8810_CSI2_PHY_TCLK_TERM	0
#define OV8810_CSI2_PHY_TCLK_MISS	1
#define OV8810_CSI2_PHY_TCLK_SETTLE	14
#endif

#ifdef CONFIG_VIDEO_OMAP3_HPLENS
#include <../drivers/media/video/hplens.h>
#endif

#define CAM_IOMUX_SAFE_MODE (OMAP343X_PADCONF_PULL_UP | \
				OMAP343X_PADCONF_PUD_ENABLED | \
				OMAP343X_PADCONF_MUXMODE7)
#define CAM_IOMUX_SAFE_MODE_INPUT (OMAP343X_PADCONF_INPUT_ENABLED | \
				OMAP343X_PADCONF_PULL_UP | \
				OMAP343X_PADCONF_PUD_ENABLED | \
				OMAP343X_PADCONF_MUXMODE7)
#define CAM_IOMUX_FUNC_MODE (OMAP343X_PADCONF_INPUT_ENABLED | \
				OMAP343X_PADCONF_MUXMODE0)

static void mapphone_camera_lines_safe_mode(void);
static void mapphone_camera_lines_func_mode(void);
static void mapphone_tablet_camera_lines_safe_mode(void);
static void mapphone_tablet_camera_lines_func_mode(void);

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

struct hplens_platform_data mapphone_hplens_platform_data = {
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
			mapphone_camera_lines_safe_mode();
			pr_err("%s: Regulator for vcam is not "\
					"initialized\n", __func__);
			return -EIO;
		}

		/* Release pm constraints */
		omap_pm_set_min_bus_tput(dev, OCP_INITIATOR_AGENT, 0);
		mapphone_camera_lines_safe_mode();
	break;
	case V4L2_POWER_ON:
		if (previous_power == V4L2_POWER_OFF) {
			/* Power Up Sequence */
			mapphone_camera_lines_func_mode();
			/* Set min throughput to:
			 *  2592 x 1944 x 2bpp x 30fps x 3 L3 accesses */
			omap_pm_set_min_bus_tput(dev, OCP_INITIATOR_AGENT, 885735);

			/* Configure pixel clock divider (here?) */
			omap_writel(OMAP_MCAM_SRC_DIV, 0x48004f40);
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


struct mt9p012_platform_data mapphone_mt9p012_platform_data = {
	.power_set      = mt9p012_sensor_power_set,
	.set_xclk	= mt9p012_set_xclk,
	.priv_data_set  = mt9p012_sensor_set_prv_data,
};

#endif /* #ifdef CONFIG_VIDEO_MT9P012 || CONFIG_VIDEO_MT9P012_MODULE */


#if defined(CONFIG_VIDEO_OV8810)
static struct omap34xxcam_sensor_config ov8810_cam_hwc = {
	.sensor_isp = 0,
	.xclk = OMAP34XXCAM_XCLK_A,
	.capture_mem = PAGE_ALIGN(3264 * 2448 * 2 * 2) * 4,
};

static int ov8810_sensor_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->u.sensor.xclk = ov8810_cam_hwc.xclk;
	hwc->u.sensor.sensor_isp = ov8810_cam_hwc.sensor_isp;
	hwc->u.sensor.capture_mem = ov8810_cam_hwc.capture_mem;
	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;
	hwc->interface_type = ISP_CSIA;

	hwc->csi2.hw_csi2.lanes.clock.polarity = OV8810_CSI2_CLOCK_POLARITY;
	hwc->csi2.hw_csi2.lanes.clock.position = OV8810_CSI2_CLOCK_LANE;
	hwc->csi2.hw_csi2.lanes.data[0].polarity = OV8810_CSI2_DATA0_POLARITY;
	hwc->csi2.hw_csi2.lanes.data[0].position = OV8810_CSI2_DATA0_LANE;
	hwc->csi2.hw_csi2.lanes.data[1].polarity = OV8810_CSI2_DATA1_POLARITY;
	hwc->csi2.hw_csi2.lanes.data[1].position = OV8810_CSI2_DATA1_LANE;
	hwc->csi2.hw_csi2.phy.ths_term = OV8810_CSI2_PHY_THS_TERM;
	hwc->csi2.hw_csi2.phy.ths_settle = OV8810_CSI2_PHY_THS_SETTLE;
	hwc->csi2.hw_csi2.phy.tclk_term = OV8810_CSI2_PHY_TCLK_TERM;
	hwc->csi2.hw_csi2.phy.tclk_miss = OV8810_CSI2_PHY_TCLK_MISS;
	hwc->csi2.hw_csi2.phy.tclk_settle = OV8810_CSI2_PHY_TCLK_SETTLE;

	return 0;
}

static struct isp_interface_config ov8810_if_config = {
	.ccdc_par_ser = ISP_CSIA,
	.dataline_shift = 0x0,
	.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE,
	.vdint0_timing = 0x0,
	.vdint1_timing = 0x0,
	.strobe = 0x0,
	.prestrobe = 0x0,
	.shutter = 0x0,
	.wenlog = ISPCCDC_CFG_WENLOG_OR,
	.dcsub = OV8810_BLACK_LEVEL_10BIT,
	.raw_fmt_in = ISPCCDC_INPUT_FMT_BG_GR,
	.wbal.coef0		= 0x23,
	.wbal.coef1		= 0x20,
	.wbal.coef2		= 0x20,
	.wbal.coef3		= 0x39,
	.u.csi.crc = 0x0,
	.u.csi.mode = 0x0,
	.u.csi.edge = 0x0,
	.u.csi.signalling = 0x0,
	.u.csi.strobe_clock_inv = 0x0,
	.u.csi.vs_edge = 0x0,
	.u.csi.channel = 0x1,
	.u.csi.vpclk = 0x1,
	.u.csi.data_start = 0x0,
	.u.csi.data_size = 0x0,
	.u.csi.format = V4L2_PIX_FMT_SGRBG10,
};

static int ov8810_sensor_power_set(struct device *dev, enum v4l2_power power)
{

#if defined(CONFIG_VIDEO_MIPI_INTERFACE)
	struct isp_csi2_lanes_cfg lanecfg;
	struct isp_csi2_phy_cfg phyconfig;
#endif

	static enum v4l2_power previous_power = V4L2_POWER_OFF;
	static struct regulator *regulator_vcam;
	static struct regulator *regulator_vwlan1;
	/*Basic turn on operation is will be first one time executed.*/
	static bool regulator_poweron;

#if defined(CONFIG_LEDS_FLASH_RESET)
	static enum flash_control_type {
		FLASH_UNKNOWN = 0,
		FLASH_NOT_DETECTED,
		FLASH_GPIO0_DC_EN,
		FLASH_GPIO0_RESET
	} flash_control_mode = FLASH_UNKNOWN;
#endif

	switch (power) {
	case V4L2_POWER_OFF:
		printk(KERN_DEBUG "%s: power off\n", __func__);
#if defined(CONFIG_VIDEO_MIPI_INTERFACE)
		/* Power Down Sequence */
		isp_csi2_complexio_power(ISP_CSI2_POWER_OFF);
#endif

		/* Release pm constraints */
		omap_pm_set_min_bus_tput(dev, OCP_INITIATOR_AGENT, 0);

		/* Turn off power */
		if (regulator_vcam != NULL) {
			regulator_disable(regulator_vcam);
			regulator_put(regulator_vcam);
			regulator_vcam = NULL;
		} else {
			mapphone_tablet_camera_lines_safe_mode();
			pr_err("%s: Regulator for vcam is not initialized\n",
					__func__);
			return -EIO;
		}

		/* Delay 6 msec for vcam to drop (4.7uF to 10uF change) */
		msleep(6);

		/* Turn off power */
		if (regulator_vwlan1 != NULL) {
			regulator_disable(regulator_vwlan1);
			regulator_put(regulator_vwlan1);
			regulator_vwlan1 = NULL;
		} else {
			mapphone_tablet_camera_lines_safe_mode();
			pr_err("%s: Regulator for vwlan1 is not initialized\n",
					__func__);
			return -EIO;
		}
		gpio_set_value(GPIO_OV8810_RESET, 0);
		gpio_set_value(GPIO_OV8810_STANDBY, 0);

		regulator_poweron = 0;

#if defined(CONFIG_LEDS_FLASH_RESET)
		/* If CPCAP GPIO0 is connected to FLASH_DC_EN (CHGDIS), set it high
		   when the camera is shut down to disable the DCDC converter. */
		/* FIXME: really needed? */
		if (flash_control_mode == FLASH_GPIO0_DC_EN ||
		    flash_control_mode == FLASH_GPIO0_RESET)
			cpcap_direct_misc_write(CPCAP_REG_GPIO0,\
				CPCAP_BIT_GPIO0DRV, CPCAP_BIT_GPIO0DRV);
		/* If CPCAP GPIO0 is connected to FLASH_RESET_N (NRST), set it low
		   when the camera is shut down. */
		if (flash_control_mode == FLASH_GPIO0_DC_EN ||
		    flash_control_mode == FLASH_GPIO0_RESET)
			cpcap_direct_misc_write(CPCAP_REG_GPIO0,\
				0, CPCAP_BIT_GPIO0DRV);
#endif

		gpio_free(GPIO_OV8810_RESET);
		gpio_free(GPIO_OV8810_STANDBY);

		mapphone_tablet_camera_lines_safe_mode();
	break;
	case V4L2_POWER_ON:
		printk(KERN_DEBUG "%s: power on\n", __func__);

		mapphone_tablet_camera_lines_func_mode();

		/* Set min throughput to:
		*  2592 x 1944 x 2bpp x 30fps x 3 L3 accesses */
		omap_pm_set_min_bus_tput(dev, OCP_INITIATOR_AGENT, 885735);

#if defined(CONFIG_VIDEO_MIPI_INTERFACE)
		/* why always reset? if (previous_power == V4L2_POWER_OFF) */
			isp_csi2_reset();

		lanecfg.clk.pol = OV8810_CSI2_CLOCK_POLARITY;
		lanecfg.clk.pos = OV8810_CSI2_CLOCK_LANE;
		lanecfg.data[0].pol = OV8810_CSI2_DATA0_POLARITY;
		lanecfg.data[0].pos = OV8810_CSI2_DATA0_LANE;
		lanecfg.data[1].pol = OV8810_CSI2_DATA1_POLARITY;
		lanecfg.data[1].pos = OV8810_CSI2_DATA1_LANE;
		lanecfg.data[2].pol = 0;
		lanecfg.data[2].pos = 0;
		lanecfg.data[3].pol = 0;
		lanecfg.data[3].pos = 0;
		isp_csi2_complexio_lanes_config(&lanecfg);
		isp_csi2_complexio_lanes_update(true);

		phyconfig.ths_term = OV8810_CSI2_PHY_THS_TERM;
		phyconfig.ths_settle = OV8810_CSI2_PHY_THS_SETTLE;
		phyconfig.tclk_term = OV8810_CSI2_PHY_TCLK_TERM;
		phyconfig.tclk_miss = OV8810_CSI2_PHY_TCLK_MISS;
		phyconfig.tclk_settle = OV8810_CSI2_PHY_TCLK_SETTLE;
		isp_csi2_phy_config(&phyconfig);
		isp_csi2_phy_update(true);

		isp_configure_interface(&ov8810_if_config);
#endif

		if ((previous_power == V4L2_POWER_OFF) &&
				(regulator_poweron == 0)) {
			/* Power Up Sequence */

			/* Configure pixel clock divider (here?) */
			omap_writel(OMAP_MCAM_SRC_DIV_MIPI, 0x48004f40);

			/* turn on VWLAN1 power */
			if (regulator_vwlan1 != NULL) {
				pr_warning("%s: Already have regulator_vwlan1 \n",
						__func__);
			} else {
				regulator_vwlan1 = regulator_get(NULL, "vwlan1");
				if (IS_ERR(regulator_vwlan1)) {
					pr_err("%s: Cannot get vwlan1 regulator_vwlan1, err=%ld\n",
							__func__, PTR_ERR(regulator_vwlan1));
					return PTR_ERR(regulator_vwlan1);
				}
			}

			if (regulator_enable(regulator_vwlan1) != 0) {
				pr_err("%s: Cannot enable vcam regulator_vwlan1\n",
						__func__);
				return -EIO;
			}

			/* turn on VCAM power */
			if (regulator_vcam != NULL) {
				pr_warning("%s: Already have regulator_vcam\n",
						__func__);
			} else {
				regulator_vcam = regulator_get(NULL, "vcam");
				if (IS_ERR(regulator_vcam)) {
					pr_err("%s: Cannot get vcam regulator_vcam, err=%ld\n",
							__func__, PTR_ERR(regulator_vcam));
					return PTR_ERR(regulator_vcam);
				}
			}

			if (regulator_enable(regulator_vcam) != 0) {
				pr_err("%s: Cannot enable vcam regulator_vcam\n",
						__func__);
				return -EIO;
			}

			/* Let power supplies settle.  Some hardware have large filter
			   caps on the VCAM rail. */
			mdelay(10);

			/* Don't try to turn them on again if they are already on (why do
			   we need this additional flag?) */
			regulator_poweron = 1;
		}

		if (previous_power == V4L2_POWER_OFF) {
			/* Request and configure gpio pins */
			if (gpio_request(GPIO_OV8810_STANDBY,
						"ov8810 camera standby") != 0) {
				pr_err("%s: Failed to request GPIO_OV8810_STANDBY\n",
						__func__);
				return -EIO;
			}

			/* Bring camera out of standby */
			gpio_direction_output(GPIO_OV8810_STANDBY, 0);

			if (gpio_request(GPIO_OV8810_RESET,
						"ov8810 camera reset") != 0) {
				pr_err("%s: Failed to request GPIO_OV8810_RESET\n",
						__func__);
				return -EIO;
			}

			/* Assert reset */
			gpio_direction_output(GPIO_OV8810_RESET, 0);

			/* Release reset */
			gpio_set_value(GPIO_OV8810_RESET, 1);

			/* Give sensor some time to get out of the reset.
			 * Datasheet says 2400 xclks. At 6 MHz, 400 usec should
			 * be enough.  SKT waits 10ms here.  Why?
			 */
			mdelay(1);

#if defined(CONFIG_LEDS_FLASH_RESET)
			/* Elaborate algorithm to detect flash module wiring */
			/* FIXME: shouldn't this mess be in the bd7885 driver?  Too 
			   intertwined with the camera reset stuff? */
			if (flash_control_mode == FLASH_UNKNOWN) {
				/* Start with CPCAP GPIO0 low */				
				cpcap_direct_misc_write(CPCAP_REG_GPIO0,
					CPCAP_BIT_GPIO0DIR,	CPCAP_BIT_GPIO0DIR);
				cpcap_direct_misc_write(CPCAP_REG_GPIO0,
					0, CPCAP_BIT_GPIO0DRV);
				
				/* If we can detect the BD7885, then FLASH_RESET_N (NRST) must
				   be attached to the camera reset line and not GPIO0.  That
				   means GPIO0 is controlling FLASH_DC_EN (CHGDIS). */
				if (bd7885_device_detection())
					flash_control_mode = FLASH_GPIO0_DC_EN;
				
				/* Now set GPIO0 high, which will take the BD7885 out of reset
				   or enable the charging circuit. */
				cpcap_direct_misc_write(CPCAP_REG_GPIO0,
					CPCAP_BIT_GPIO0DRV, CPCAP_BIT_GPIO0DRV);
				
				/* If we didn't detect it before, try again now.  If we see it,
				   then we know that CPCAP GPIO is controlling reset. */
				if (flash_control_mode == FLASH_UNKNOWN &&
				    bd7885_device_detection())
					flash_control_mode = FLASH_GPIO0_RESET;
					
				/* If we still can't see it, then it's probably not there. */
				if (flash_control_mode == FLASH_UNKNOWN) {
					flash_control_mode = FLASH_NOT_DETECTED;
					printk(KERN_INFO "%s: flash module not detected\n",
				    	   __func__);
				} else {
					printk(KERN_INFO "%s: flash module detected in mode %d\n",
				    	   __func__, flash_control_mode);
				}
			}
			
			/* Prepare the BD7885 */
			if (flash_control_mode == FLASH_GPIO0_DC_EN) {
				/* If CPCAP GPIO0 is connected to FLASH_DC_EN (CHGDIS), set
				   it low when the camera is powered up. */
				cpcap_direct_misc_write(CPCAP_REG_GPIO0,
					0, CPCAP_BIT_GPIO0DRV);
			} else if (flash_control_mode == FLASH_GPIO0_RESET) {
				/* If CPCAP GPIO0 is connected to FLASH_RESET_N (NRST), set
				   it high when the camera is powered up. */
				cpcap_direct_misc_write(CPCAP_REG_GPIO0,
					CPCAP_BIT_GPIO0DRV, CPCAP_BIT_GPIO0DRV);
			}
#endif
		}
		break;
	case V4L2_POWER_STANDBY:
		/* stand by */
		break;
	}
	/* Save powerstate to know what was before calling POWER_ON. */
	previous_power = power;
	return 0;
}

struct ov8810_platform_data mapphone_ov8810_platform_data = {
	.power_set      = ov8810_sensor_power_set,
	.priv_data_set  = ov8810_sensor_set_prv_data,
	.default_regs   = NULL,
};

#endif  /* #ifdef CONFIG_VIDEO_OV8810*/


/* We can't change the IOMUX config after bootup
 * with the current pad configuration architecture,
 * the next two functions are hack to configure the
 * camera pads at runtime to save power in standby.
 * For phones don't have MIPI camera support, like
 * Ruth, Tablet P2,P3 */

void mapphone_camera_lines_safe_mode(void)
{
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE_INPUT, 0x011a);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE_INPUT, 0x011c);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE_INPUT, 0x011e);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE_INPUT, 0x0120);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE, 0x0122);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE, 0x0124);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE, 0x0126);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE, 0x0128);
}

void mapphone_camera_lines_func_mode(void)
{
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x011a);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x011c);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x011e);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0120);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0122);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0124);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0126);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0128);
}

/* the next two functions are for Phones have MIPI
 * camera support, like Tablet P2A */

void mapphone_tablet_camera_lines_safe_mode(void)
{
#if defined(CONFIG_VIDEO_MIPI_INTERFACE)
	omap_writew(0x0704, 0x4800207C);	/* CONTROL_PADCONF_GPMC_A2 */
	omap_writew(0x0704, 0x480020D0);	/* CONTROL_PADCONF_GPMC_WAIT2 */
#endif
#if CONFIG_VIDEO_MIPI_INTERFACE
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE_INPUT, 0x0116);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE_INPUT, 0x0118);
	omap_ctrl_writew(OMAP343X_PADCONF_MUXMODE7, 0x0134);
	omap_ctrl_writew(OMAP343X_PADCONF_MUXMODE7, 0x0136);
	omap_ctrl_writew(OMAP343X_PADCONF_MUXMODE7, 0x0138);
	omap_ctrl_writew(OMAP343X_PADCONF_MUXMODE7, 0x013a);
#endif
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE, 0x0122);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE, 0x0124);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE, 0x0126);
	omap_ctrl_writew(CAM_IOMUX_SAFE_MODE, 0x0128);
}

void mapphone_tablet_camera_lines_func_mode(void)
{
#if defined(CONFIG_VIDEO_MIPI_INTERFACE)
	omap_writew(0x0704, 0x4800207C);	/* CONTROL_PADCONF_GPMC_A2 */
	omap_writew(0x061C, 0x480020D0);	/* CONTROL_PADCONF_GPMC_WAIT2 */
#endif
#if CONFIG_VIDEO_MIPI_INTERFACE
	omap_ctrl_writew(OMAP343X_PADCONF_MUXMODE2 |
			OMAP343X_PADCONF_INPUT_ENABLED, 0x0116);
	omap_ctrl_writew(OMAP343X_PADCONF_MUXMODE2 |
			OMAP343X_PADCONF_INPUT_ENABLED, 0x0118);
	omap_ctrl_writew(OMAP343X_PADCONF_MUXMODE0 |
			OMAP343X_PADCONF_INPUT_ENABLED, 0x0134);
	omap_ctrl_writew(OMAP343X_PADCONF_MUXMODE0 |
			OMAP343X_PADCONF_INPUT_ENABLED, 0x0136);
	omap_ctrl_writew(OMAP343X_PADCONF_MUXMODE0 |
			OMAP343X_PADCONF_INPUT_ENABLED, 0x0138);
	omap_ctrl_writew(OMAP343X_PADCONF_MUXMODE0 |
			OMAP343X_PADCONF_INPUT_ENABLED, 0x013a);
#endif
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0122);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0124);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0126);
	omap_ctrl_writew(CAM_IOMUX_FUNC_MODE, 0x0128);
}

void __init mapphone_camera_init(void)
{
	struct device_node *feat_node;
	const void *feat_prop;
	unsigned int is_mipi_cam = 0;
	unsigned int is_smart_cam = 0;

	/* Check sensor Type */
	feat_node = of_find_node_by_path(DT_HIGH_LEVEL_FEATURE);
	if (NULL != feat_node) {
		feat_prop = of_get_property(feat_node,
					"feature_mipi_cam", NULL);
		if (NULL != feat_prop) {
			is_mipi_cam = *(u8 *)feat_prop;
			printk(KERN_INFO "feature_mipi_cam %d\n", is_mipi_cam) ;
		}
		
		feat_prop = of_get_property(feat_node,
					"feature_smart_cam", NULL);
		if (NULL != feat_prop) {
			is_smart_cam = *(u8 *)feat_prop;
			printk(KERN_INFO "feature_smart_cam %d\n", is_smart_cam) ;
		}
		
	}
	

	if (is_mipi_cam) {
	    printk(KERN_INFO "mapphone_camera_init: MIPI camera\n");
	    omap_cfg_reg(AD17_34XX_CSI2_DX0);
	    omap_cfg_reg(AE18_34XX_CSI2_DY0);
	    omap_cfg_reg(AD16_34XX_CSI2_DX1);
	    omap_cfg_reg(AE17_34XX_CSI2_DY1);
	    omap_cfg_reg(C25_34XX_CAM_XCLKA);
	    omap_cfg_reg(C23_34XX_CAM_FLD);
	    omap_cfg_reg(AG17_34XX_CAM_D0_ST);
	    omap_cfg_reg(AH17_34XX_CAM_D1_ST);
	    omap_cfg_reg(H2_34XX_GPMC_A3);

	    /*Initialize F_RDY_N pin for Xenon flash control.*/
	    if (gpio_request(GPIO_FLASH_READY, "xenon flash ready pin") != 0)
			pr_err("%s: Xenon flash ready pin control failure.\n",__func__);

	    gpio_direction_input(GPIO_FLASH_READY);
	    mapphone_tablet_camera_lines_safe_mode();
	} else if (is_smart_cam) {
	    printk(KERN_INFO "mapphone_camera_init: smart camera\n");
		omap_cfg_reg(A24_34XX_CAM_HS);
	    omap_cfg_reg(A23_34XX_CAM_VS);
	    omap_cfg_reg(C27_34XX_CAM_PCLK);
	    omap_cfg_reg(B24_34XX_CAM_D2);
	    omap_cfg_reg(C24_34XX_CAM_D3);
	    omap_cfg_reg(D24_34XX_CAM_D4);
	    omap_cfg_reg(A25_34XX_CAM_D5);
	    omap_cfg_reg(K28_34XX_CAM_D6);
	    omap_cfg_reg(L28_34XX_CAM_D7);
	    omap_cfg_reg(K27_34XX_CAM_D8);
	    omap_cfg_reg(L27_34XX_CAM_D9);
		omap_cfg_reg(C25_34XX_CAM_XCLKA);
		omap_cfg_reg(K8_34XX_GPMC_WAIT2);
		omap_cfg_reg(C23_34XX_CAM_FLD);
	
	} else {
	    printk(KERN_INFO "mapphone_camera_init: conventional camera\n");
	    omap_cfg_reg(A24_34XX_CAM_HS);
	    omap_cfg_reg(A23_34XX_CAM_VS);
	    omap_cfg_reg(C27_34XX_CAM_PCLK);
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
	    omap_cfg_reg(C25_34XX_CAM_XCLKA);
	    omap_cfg_reg(C23_34XX_CAM_FLD);
	    omap_cfg_reg(AG17_34XX_CAM_D0_S);
	    omap_cfg_reg(AH17_34XX_CAM_D1_S);
	    mapphone_camera_lines_safe_mode();
	}
}
