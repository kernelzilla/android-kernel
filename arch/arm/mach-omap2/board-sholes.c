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
#include <linux/usb/omap.h>

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

#define SHOLES_IPC_USB_SUSP_GPIO	142
#define SHOLES_AP_TO_BP_FLASH_EN_GPIO	157
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



#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)

static int sholes_usb_port_startup(struct platform_device *dev, int port)
{
	int r;

	if (port == 2) {
		r = gpio_request(SHOLES_IPC_USB_SUSP_GPIO, "ipc_usb_susp");
		if (r < 0) {
			printk(KERN_WARNING "Could not request GPIO %d"
			       " for IPC_USB_SUSP\n",
			       SHOLES_IPC_USB_SUSP_GPIO);
			return r;
		}
		gpio_direction_output(SHOLES_IPC_USB_SUSP_GPIO, 0);
	} else {
		return -EINVAL;
	}
	return 0;
}

static void sholes_usb_port_shutdown(struct platform_device *dev, int port)
{
	if (port == 2)
		gpio_free(SHOLES_IPC_USB_SUSP_GPIO);
}


static void sholes_usb_port_suspend(struct platform_device *dev,
				    int port, int suspend)
{
	if (port == 2)
		gpio_set_value(SHOLES_IPC_USB_SUSP_GPIO, suspend);
}


static struct omap_usb_port_data usb_port_data[] = {
	[0] = { .flags = 0x0, }, /* disabled */
	[1] = { .flags = 0x0, }, /* disabled */
	[2] = {
		.flags = OMAP_USB_PORT_FLAG_ENABLED |
			OMAP_USB_PORT_FLAG_NOBITSTUFF,
		.mode = OMAP_USB_PORT_MODE_UTMI_PHY_4PIN,
		.startup = sholes_usb_port_startup,
		.shutdown = sholes_usb_port_shutdown,
		.suspend = sholes_usb_port_suspend,
	},
};

static struct omap_usb_platform_data usb_platform_data = {
	.port_data = usb_port_data,
	.num_ports = ARRAY_SIZE(usb_port_data),
};

static struct resource ehci_resources[] = {
	[0] = {
		.start	= OMAP34XX_HSUSB_HOST_BASE + 0x800,
		.end	= OMAP34XX_HSUSB_HOST_BASE + 0x800 + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {         /* general IRQ */
		.start	= INT_34XX_EHCI_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

static u64 ehci_dmamask = ~(u32)0;
static struct platform_device ehci_device = {
	.name		= "ehci-omap",
	.id		= 0,
	.dev = {
		.dma_mask		= &ehci_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data		= &usb_platform_data,
	},
	.num_resources	= ARRAY_SIZE(ehci_resources),
	.resource	= ehci_resources,
};
#endif

#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
static struct resource ohci_resources[] = {
	[0] = {
		.start	= OMAP34XX_HSUSB_HOST_BASE + 0x400,
		.end	= OMAP34XX_HSUSB_HOST_BASE + 0x400 + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {         /* general IRQ */
		.start	= INT_34XX_OHCI_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

static u64 ohci_dmamask = ~(u32)0;

static struct omap_usb_config dummy_usb_config = {
};

static struct platform_device ohci_device = {
	.name		= "ohci",
	.id		= 0,
	.dev = {
		.dma_mask		= &ohci_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data	= &dummy_usb_config,
	},
	.num_resources	= ARRAY_SIZE(ohci_resources),
	.resource	= ohci_resources,
};
#endif /* OHCI specific data */


static void __init sholes_ehci_init(void)
{

	omap_cfg_reg(AF5_34XX_GPIO142);		/*  IPC_USB_SUSP      */
	omap_cfg_reg(AA21_34XX_GPIO157);	/*  AP_TO_BP_FLASH_EN */
	omap_cfg_reg(AD1_3430_USB3FS_PHY_MM3_RXRCV);
	omap_cfg_reg(AD2_3430_USB3FS_PHY_MM3_TXDAT);
	omap_cfg_reg(AC1_3430_USB3FS_PHY_MM3_TXEN_N);
	omap_cfg_reg(AE1_3430_USB3FS_PHY_MM3_TXSE0);

	if (gpio_request(SHOLES_AP_TO_BP_FLASH_EN_GPIO,
			 "ap_to_bp_flash_en") != 0) {
		printk(KERN_WARNING "Could not request GPIO %d"
		       " for IPC_USB_SUSP\n",
		       SHOLES_IPC_USB_SUSP_GPIO);
		return;
	}
	gpio_direction_output(SHOLES_AP_TO_BP_FLASH_EN_GPIO, 0);

#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
	platform_device_register(&ehci_device);
#endif
#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
	platform_device_register(&ohci_device);
#endif
}


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
	sholes_ehci_init();
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
