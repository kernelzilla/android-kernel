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
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/spi/spi.h>
#include <linux/mm.h>
#include <linux/i2c/twl4030.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>

#include <mach/board-sholes.h>
#include <mach/hardware.h>
#include <mach/mcspi.h>
#include <mach/gpio.h>
#include <mach/mux.h>
#include <mach/board.h>
#include <mach/common.h>
#include <mach/keypad.h>
#include <mach/gpmc.h>

#if 0
#ifdef CONFIG_VIDEO_OMAP3
#include <media/v4l2-int-device.h>
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>
#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
#include <../drivers/media/video/mt9p012.h>
#endif
#endif

#ifdef CONFIG_VIDEO_DW9710
#include <../drivers/media/video/dw9710.h>
#endif

#ifdef CONFIG_OMAP3_PM
#include "prcm-regs.h"
#include <mach/prcm_34xx.h>
#endif
#endif

#include <asm/io.h>
#include <asm/delay.h>
#include <mach/control.h>

#if 0
#if defined(CONFIG_MOT_FEAT_GPIO_API) || defined(CONFIG_MOT_FEAT_MEM_TIMING_API)
#include <mach/mot-gpio-omap.h>
#include "board-sholes-timing.h"
#endif

#if defined(CONFIG_MOT_FEAT_DEVICE_TREE)
#include <asm/mothwcfg.h>
#endif

#ifdef CONFIG_OMAP3_PM
#define CONTROL_SYSC_SMARTIDLE  (0x2 << 3)
#define CONTROL_SYSC_AUTOIDLE   (0x1)

#define PRCM_INTERRUPT_MASK     (1 << 11)
#define UART1_INTERRUPT_MASK    (1 << 8)
#define UART2_INTERRUPT_MASK    (1 << 9)
#define UART3_INTERRUPT_MASK    (1 << 10)
int console_detect(char *str);
unsigned int uart_interrupt_mask_value;
#endif

#ifdef CONFIG_OMAP3_PM
/*
 * Board DDR timings used during frequency changes
 */
struct dvfs_config omap3_vdd2_config[PRCM_NO_VDD2_OPPS] = {
#ifdef CONFIG_OMAP3_CORE_166MHZ
	{
	/* SDRC CS0/CS1 values 83MHZ*/
	/* not optimized at 1/2 speed except for RFR */
	{{0x00025801, 0x629db4c6, 0x00012214},   /* cs 0 */
	 {0x00025801, 0x629db4c6, 0x00012214} }, /* cs 1*/
	},

	/* SDRC CS0/CS1 values 166MHZ*/
	{
	{{0x0004e201, 0x629db4c6, 0x00012214},
	 {0x0004e201, 0x629db4c6, 0x00012214} },
	},
#endif
};

#endif /* CONFIG_OMAP3_PM */
#endif
#define PM_RECEIVER             TWL4030_MODULE_PM_RECEIVER


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

static struct omap2_mcspi_device_config cpcap_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel = 1,  /* 0: slave, 1: master */
};

static struct omap2_mcspi_device_config tsc2005_mcspi_config = {
	.turbo_mode     = 0,
	.single_channel = 1,
};

static struct spi_board_info sholes_spi_board_info[] __initdata = {
	[0] = {
		.modalias	= "cpcap",
		.bus_num	= 1,
		.chip_select	= 0,
		.max_speed_hz	= 20000000,


		.controller_data= &cpcap_mcspi_config,
		.mode           = SPI_CS_HIGH,
	},
	{
		.modalias	= "tsc2005",
		.bus_num	= 3,
		.chip_select	= 1,
		.max_speed_hz	= 1500000,
		.controller_data= &tsc2005_mcspi_config,
	}



};

#if 0
#ifdef CONFIG_VIDEO_DW9710
static int dw9710_lens_power_set(enum v4l2_power power)
{

	return 0;
}

static int dw9710_lens_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_LENS;

	return 0;
}

static struct dw9710_platform_data sdp3430_dw9710_platform_data = {
	.power_set      = dw9710_lens_power_set,
	.priv_data_set  = dw9710_lens_set_prv_data,
};
#endif

#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
static struct omap34xxcam_sensor_config cam_hwc = {
	.sensor_isp = 0,
	.xclk = OMAP34XXCAM_XCLK_A,
	.capture_mem = PAGE_ALIGN(2592 * 1944 * 2) * 4,
};

static int mt9p012_sensor_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->u.sensor.xclk = cam_hwc.xclk;
	hwc->u.sensor.sensor_isp = cam_hwc.sensor_isp;
	hwc->u.sensor.capture_mem = cam_hwc.capture_mem;
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
	.prev_sph = 2,
	.prev_slv = 0,
	.wenlog = ISPCCDC_CFG_WENLOG_OR,
	.u.par.par_bridge = 0x0,
	.u.par.par_clk_pol = 0x0,
};

static int mt9p012_sensor_power_set(enum v4l2_power power)
{
	static enum v4l2_power previous_power = V4L2_POWER_OFF;
	switch (power) {
	case V4L2_POWER_OFF:
		/* Power Down Sequence */
		omap_free_gpio(MT9P012_RESET_GPIO);
#if defined(CONFIG_MOT_FEAT_STANDBY_CONN)
		omap_free_gpio(MT9P012_STANDBY_GPIO);
#endif
		break;
	case V4L2_POWER_ON:
		if (previous_power == V4L2_POWER_OFF) {
			/* Power Up Sequence */

                        /* Configure pixel clock divider (here?) */
                        omap_writel(0x4, 0x48004f40);

			isp_configure_interface(&mt9p012_if_config);

#if defined(CONFIG_MOT_FEAT_STANDBY_CONN)
			/* Request and configure gpio pins */
			if (omap_request_gpio(MT9P012_STANDBY_GPIO) != 0) {
				printk(KERN_WARNING "Could not request GPIO %d"
							" for MT9P012\n",
							MT9P012_STANDBY_GPIO);
				return -EIO;
			}
#endif
			/* Request and configure gpio pins */
			if (omap_request_gpio(MT9P012_RESET_GPIO) != 0)
				return -EIO;

#if defined(CONFIG_MOT_FEAT_STANDBY_CONN)
			/* set to output mode */
			omap_set_gpio_direction(MT9P012_STANDBY_GPIO, 0);
#endif
			/* set to output mode */
			omap_set_gpio_direction(MT9P012_RESET_GPIO, 0);
#if defined(CONFIG_MOT_FEAT_STANDBY_CONN)
			/* STANDBY_GPIO is active HIGH for set LOW to release */
			omap_set_gpio_dataout(MT9P012_STANDBY_GPIO, 1);
#endif

			/* nRESET is active LOW. set HIGH to release reset */
			omap_set_gpio_dataout(MT9P012_RESET_GPIO, 1);

			/* turn on digital power */
                        cpcap_spi_write(396, 0x11);
		}

#if defined(CONFIG_MOT_FEAT_STANDBY_CONN)
		/* out of standby */
		omap_set_gpio_dataout(MT9P012_STANDBY_GPIO, 0);
#endif
		udelay(1000);

		if (previous_power == V4L2_POWER_OFF) {
			/* have to put sensor to reset to guarantee detection */
			omap_set_gpio_dataout(MT9P012_RESET_GPIO, 0);

			udelay(1500);

			/* nRESET is active LOW. set HIGH to release reset */
			omap_set_gpio_dataout(MT9P012_RESET_GPIO, 1);
			/* give sensor sometime to get out of the reset.
			 * Datasheet says 2400 xclks. At 6 MHz, 400 usec is
			 * enough
			 */
			udelay(300);
		}
		break;
	case V4L2_POWER_STANDBY:
		/* stand by */
#if defined(CONFIG_MOT_FEAT_STANDBY_CONN)
		omap_set_gpio_dataout(MT9P012_STANDBY_GPIO, 1);
#endif
		break;
	}
	/* Save powerstate to know what was before calling POWER_ON. */
	previous_power = power;
	return 0;
}

static struct mt9p012_platform_data sdp3430_mt9p012_platform_data = {
	.power_set      = mt9p012_sensor_power_set,
	.priv_data_set  = mt9p012_sensor_set_prv_data,
	.default_regs   = NULL,
};

#endif
static struct i2c_board_info __initdata sholes_i2c_bus2_board_info[] = {
        {
                I2C_BOARD_INFO("akm8973", 0x1C),
        },
};

#if 0
static struct i2c_board_info __initdata sholes_i2c_bus3_board_info[] = {
#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
	{
#if defined(CONFIG_MOT_FEAT_MT9P012_FIXES)
		/* FIXME: Workaround for HW address conflict */
                I2C_BOARD_INFO("mt9p012", 0x10),
#else
                I2C_BOARD_INFO("mt9p012", MT9P012_I2C_ADDR),
#endif /* CONFIG_MOT_FEAT_MT9P012_FIXES */
		.platform_data = &sdp3430_mt9p012_platform_data,
	},
#ifdef CONFIG_VIDEO_DW9710
	{
		I2C_BOARD_INFO(DW9710_NAME,  DW9710_AF_I2C_ADDR),
		.platform_data = &sdp3430_dw9710_platform_data,
	},
#endif
#endif
};
#endif
#endif

/*
static struct platform_device sholes_lcd_device = {
	.name		= "sholes_lcd",
	.id		= -1,
};
*/
#if 0
#if defined(CONFIG_MOT_FEAT_CS8900A_ETH)

#define CS8900A_CS		3
#define CS8900A_IRQ_GPIO        GPIO_SIGNAL_ETHERNET_INT
#define CS8900A_IRQ             OMAP_GPIO_IRQ(CS8900A_IRQ_GPIO)

extern unsigned int netcard_portlist[2];
extern unsigned int cs8900_irq_map[4];
#if defined(CONFIG_MOT_FEAT_MEM_TIMING_API)
extern unsigned int ethernet_is_available(void);
#else
unsigned int ethernet_is_available(void) {return 0;}
#endif

void __init phone3430_init_cs8900a(void)
{
	int eth_cs;
	unsigned long cs_mem_base;
#if defined(CONFIG_MOT_FEAT_MEM_TIMING_API)
	unsigned int init_opp;
#endif

	if (!ethernet_is_available()) { 
		printk(KERN_ERR"Skip Ethernet initialization!\n");
		return;
	}

	eth_cs	= CS8900A_CS;

 	/* Configure IRQ for CS 8900A */
     	if (omap_request_gpio(CS8900A_IRQ_GPIO) < 0) {
       		printk(KERN_ERR "Failed to request GPIO%d for CS8900A IRQ\n", CS8900A_IRQ_GPIO);
                return;
   	}
        omap_set_gpio_direction(CS8900A_IRQ_GPIO, 1);

#if defined(CONFIG_MOT_FEAT_MEM_TIMING_API)
        init_opp = get_mot_init_opp(2);
	if(is_mot_gpmc_timing_valid(init_opp,eth_cs)) {
            gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG1,
			      get_mot_gpmc_config(init_opp,eth_cs, 1));
            gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG2,
			      get_mot_gpmc_config(init_opp,eth_cs,2));
            gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG3,get_mot_gpmc_config(init_opp,eth_cs,3));
            gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG4,get_mot_gpmc_config(init_opp,eth_cs,4));
            gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG5,get_mot_gpmc_config(init_opp,eth_cs,5));
            gpmc_cs_write_reg(eth_cs, GPMC_CS_CONFIG6,get_mot_gpmc_config(init_opp,eth_cs,6));
            printk(KERN_INFO "Initializing Ethernet controller GPMC done @ L3 = %d Hz!\n",read_mot_l3_freq(init_opp));
        }
        else {
            printk(KERN_ERR "Ethernet controller GPMC setting (cs%d) is invalid in device tree\n",eth_cs);
        }
#endif /* CONFIG_MOT_FEAT_MEM_TIMING_API */

	/* 
	 * gpmc_cs_request() shall be called at last since it enables the CS
	 * All parameters shall be setup before it.
	 *
	 */
	if (gpmc_cs_request(eth_cs, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for CS8900A\n");
		omap_free_gpio(CS8900A_IRQ_GPIO);
		return;
	}

	netcard_portlist[0] = cs_mem_base + 0x300;
	cs8900_irq_map[0] = CS8900A_IRQ;
	printk("Phy addr 0x%lx allocated for nCS3(CS8900A)\n", cs_mem_base);
} 

#endif /* CONFIG_MOT_FEAT_CS8900A_ETH */
#endif

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
	.enabled_uarts	= ((1 << 0) | (1 << 1) | (1 << 2)),
};

/*static struct omap_mmc_config sholes_mmc_config __initdata = {
	.mmc [0] = {
		.enabled	= 1,
		.wire4		= 1,
	},
};*/

static struct omap_board_config_kernel sholes_config[] __initdata = {
	{OMAP_TAG_UART,		&sholes_uart_config },
	/* {OMAP_TAG_MMC,		&sholes_mmc_config },*/
};


/*
static struct i2c_board_info __initdata sholes_i2c_bus3_board_info[] = {
	{
		I2C_BOARD_INFO("sholes_panel_ctrl", 0x57),
		.platform_data = &panel_ctrl_data,
	},
};
*/

static int __init sholes_i2c_init(void)
{
	omap_register_i2c_bus(1, 400, NULL, 0);
/*
	omap_register_i2c_bus(2, 100, sholes_i2c_bus2_board_info,
			      ARRAY_SIZE(sholes_i2c_bus2_board_info));
	omap_register_i2c_bus(3, 100, sholes_i2c_bus3_board_info,
			      ARRAY_SIZE(sholes_i2c_bus3_board_info));
*/
	return 0;
}
arch_initcall(sholes_i2c_init);

extern void __init sholes_flash_init(void);
extern void __init sholes_gpio_iomux_init(void);

static struct platform_device *sholes_devices[] __initdata = {
	&omap3430_kp_device,
	&omap3430_master_sensor,
};

static void __init sholes_init(void)
{
	omap_board_config = sholes_config;
	omap_board_config_size = ARRAY_SIZE(sholes_config);
	spi_register_board_info(sholes_spi_board_info,
				ARRAY_SIZE(sholes_spi_board_info));
        //sholes_memory_timing_init(); 
	//prcm_init();
	sholes_flash_init();
	omap_serial_init();
	sholes_panel_init();
	platform_add_devices(sholes_devices, ARRAY_SIZE(sholes_devices));
	//usb_musb_init();
	//usb_ehci_init();
	//hsmmc_init();

}

static void __init sholes_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(OMAP_SHOLES, "Motorola Product - Sholes Phone")
	/* Maintainer: Motorola, Inc. */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
#if 1
	.boot_params	= 0x80C00100,
#else
	.boot_params	= 0x80000100,
#endif
	.map_io		= sholes_map_io,
	.init_irq	= sholes_init_irq,
	.init_machine	= sholes_init,
	.timer		= &omap_timer,
MACHINE_END
