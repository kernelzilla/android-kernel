/* linux/arch/arm/mach-msm/board-mot-7x01a.c
 *
 * Copyright (C) 2008-2009 Motorola, Inc.
 * Author: Igor Kovalenko <igor.kovalenko@motorola.com>
 * 
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 *
 * Copyright (c) 2008-2009 QUALCOMM USA, INC.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/i2c.h>
#include <linux/android_pmem.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/sfh7743.h>
#include <linux/lis331dlh.h>
#include <linux/mot_battery_info.h>
#include <linux/proc_fs.h>
#include <linux/power_supply.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/setup.h>

#include <asm/mach/mmc.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/irqs.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/msm_iomap.h>
#include <mach/msm_serial_hs.h>
#include <mach/irqs-7xxx.h>
#include <mach/msm_hsusb.h>
#include <mach/msm_battery.h>
#include <mach/vreg.h>
#include "proc_comm.h"
#include <mach/msm_rpcrouter.h>
#include <mach/msm_fb.h>
#include <mach/camera.h>

#ifdef CONFIG_USB_FUNCTION
#include <linux/usb/mass_storage_function.h>
#endif
#include "devices.h"
#include "clock.h"
#include "socinfo.h"
#include "pm.h"
#include "board-mot.h"
#include "timed_vibrator.h"
#include "smd_private.h"

/* Defaults. Actual values passed via ATAGs */
unsigned smi_64m = 0;
unsigned ddr_mono = 0;

unsigned mot_hw_rev = 0x20;
char     keypad[12] = "qwerty";

static smem_mot_vendor1_type *vendor1;
static int secure_hw;
int is_secure_hw(void);

static int model_set;
char *board_model;
char *board_model_name(void);
static int i2c_speed_set;
static int mfg_mode;

static struct vreg *vreg_slide;

static void msm_fb_mddi_power_save(int on)
{
	if(on)
	{
		vreg_enable(vreg_slide);
		/* TPO_NV requires 10ms delay after reset */
		msleep(10);
		gpio_set_value(LCD_RST_N_SIGNAL, on);
	} else {
		gpio_set_value(LCD_RST_N_SIGNAL, on);
		/* slight delay after deassert */
		msleep(1);
		vreg_disable(vreg_slide);
	}
}

#define PM_VID_EN_CONFIG_PROC		24
#define PM_VID_EN_API_PROG		0x30000061
#define PM_VID_EN_API_VERS		0x00010001

#ifndef CONFIG_MACH_MOT
static struct msm_rpc_endpoint *pm_vid_en_ep;

static int msm_fb_pm_vid_en(int on)
{
	int rc = 0;
	struct msm_fb_pm_vid_en_req {
		struct rpc_request_hdr hdr;
		uint32_t on;
	} req;

	pm_vid_en_ep = msm_rpc_connect(PM_VID_EN_API_PROG,
			 PM_VID_EN_API_VERS, 0);

	if(IS_ERR(pm_vid_en_ep)) {
		printk(KERN_ERR "%s: msm_rpc_connect failed ! rc = %ld\n",
				__func__, PTR_ERR(pm_vid_en_ep));
		return -EINVAL;
	}
	req.on = cpu_to_be32(on);
	rc = msm_rpc_call(pm_vid_en_ep,
			PM_VID_EN_CONFIG_PROC,
			&req, sizeof(req),
			5 * HZ);
	if(rc)
		printk(KERN_ERR "%s: msm_rpc_call failed! rc = %d\n",
				__func__, rc);
	msm_rpc_close(pm_vid_en_ep);
	return rc;
}

static struct tvenc_platform_data tvenc_pdata = {
	.pm_vid_en = msm_fb_pm_vid_en,
};
#endif

static struct mddi_platform_data mddi_pdata = {
	.mddi_power_save = msm_fb_mddi_power_save,
};

static struct resource msm_fb_resources[] = {
	{
		.flags		= IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	int ret;

	if(!strcmp(name, "mddi_tpo_nv"))
		ret = 0;
	else
		ret = -ENODEV;

	return ret;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.mddi_prescan = 1,
};

static struct platform_device msm_fb_device = {
	.name = "msm_fb",
	.id = 0,
	.num_resources = ARRAY_SIZE(msm_fb_resources),
	.resource = msm_fb_resources,
	.dev = {
		.platform_data = &msm_fb_pdata,
	}
};

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 97,
};

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("pmdh", &mddi_pdata);
}



/* Initializers for i2c device parameters */
static struct resource stmicro_resources[] = {
    {
	.start    = LIS331DLH_INT1,
	.end    = LIS331DLH_INT2,
	.flags    = IORESOURCE_IRQ,
    },
};

static struct resource key08_resources[]  =  {
      {
	.start    = KEY08_I2C_ADDR,
	.end      = KEY08_BL_I2C_ADDR,
	.flags    = IORESOURCE_MEM,
      },
};

static struct resource minipad_resources[]  =  {
    {
            .start  = MINIPAD_I2C_ADDR,
            .end    = MINIPAD_BL_I2C_ADDR,
            .flags  = IORESOURCE_MEM,
    },
};

static struct platform_device mot_adp5588_device = {
	.name        = "adp5588_morrison",
	.id         = 0,
};
static struct platform_device mot_key08_device = {
	.name        = "key08",
	.id            = 0,
	.num_resources    = ARRAY_SIZE(key08_resources),
	.resource    = key08_resources,
};

static struct platform_device mot_qtm_obp_device = {
    .name        = "qtm_obp",
    .id            = 0,
    .num_resources    = ARRAY_SIZE(key08_resources),
    .resource    = key08_resources,
};

static struct platform_device mot_minipad_device = {
    .name           = "minipad",
    .id             = 0,
    .num_resources  = ARRAY_SIZE(minipad_resources),
    .resource       = minipad_resources,
};

static struct platform_device mot_stmicro_device = {
    .name        = "stmicro",
    .id         = 0,
    .num_resources    = ARRAY_SIZE(stmicro_resources),
    .resource    = stmicro_resources,
};

static int lis331dlh_initialization(void)
{
	return 0;
}

static void lis331dlh_exit(void)
{
}

static int lis331dlh_power_on(void)
{
	return 0;
}

static int lis331dlh_power_off(void)
{
	return 0;
}

struct lis331dlh_platform_data lis331dlh_data = {
	.init = lis331dlh_initialization,
	.exit = lis331dlh_exit,
	.power_on = lis331dlh_power_on,
	.power_off = lis331dlh_power_off,

	.min_interval	= 1,
	.poll_interval	= 200,

	.g_range	= LIS331DLH_G_4G,

	.axis_map_x	= 0,
	.axis_map_y	= 1,
	.axis_map_z	= 2,

	.negate_x	= 1,
	.negate_y	= 1,
	.negate_z	= 0,
};

#ifdef CONFIG_ANDROID_RAM_CONSOLE
static struct resource ramconsole_resource[] = {
	{
		.start    = MSM_RAMCONSOLE_BASE,
		.end    = MSM_RAMCONSOLE_BASE + MSM_RAMCONSOLE_SIZE - 1,
		.flags    = IORESOURCE_MEM,
	},
};

static struct platform_device ramconsole_device = {
	.name	= "ram_console",
	.id	= 0,
	.num_resources	= ARRAY_SIZE(ramconsole_resource),
	.resource	= ramconsole_resource,
};
#endif

#define MSM_GPIO_INT_BASE  64
#define MSM_GPIO_TO_IRQ(x)  (MSM_GPIO_INT_BASE + x)

/* 
    List of common i2c devices to use by init_machine.
 */
static struct i2c_board_info i2c_devices[] = {
    {    /* accelerometer [xcvr] */
	I2C_BOARD_INFO("lis331dlh", LIS331DLH_I2C_ADDR),
	.irq = 0,    /* Morrison-specific parameter */
	.platform_data = &mot_stmicro_device,
    },
    {    /* compass [xcvr] */
	I2C_BOARD_INFO("akm8973", 0x1C),
	/* .platform_data = &compass_platform_data, */
	.irq = MSM_GPIO_TO_IRQ(58),
    },
	{	/* couloumb counter [xcvr] */
		I2C_BOARD_INFO("bq27505", BQ27505_I2C_ADDR),
		.irq = MSM_GPIO_TO_IRQ(BQ27505_INT_N_SIGNAL)
	},
};

/*
    Board-specific extra i2c devices. 
	Those are all off the main XCVR board. We do not add them if bare board is detected.
 */
static struct i2c_board_info i2c_morrison_devices[] = {
      {     /* keypad [kpd/interconnect pcb] */
	I2C_BOARD_INFO("adp5588_keypad", ADP5588_I2C_ADDR),
	.irq = MSM_GPIO_TO_IRQ(QWERTY_INT_N_SIGNAL),
	.platform_data = &mot_adp5588_device
      },
      {    /* backlight [slider pcb] */
	I2C_BOARD_INFO("lm3535", LM3535_I2C_ADDR),
	.irq = MSM_GPIO_TO_IRQ(LM3535_INT_N_SIGNAL)
      },

      {    /* touchscreen [slider pcb] */
	I2C_BOARD_INFO("key08", KEY08_I2C_ADDR),
	.irq = MSM_GPIO_TO_IRQ(KEY08_INT_N_SIGNAL),
	.platform_data = &mot_key08_device
      },
      {
	/* camera [keypad/interconnect pcb] */
	I2C_BOARD_INFO("mt9p012", 0x6c >> 1),
      },
};

static struct i2c_board_info i2c_motus_devices[] = {
    {     /* keypad [kpd/interconnect pcb] */
        I2C_BOARD_INFO("adp5588_keypad", ADP5588_I2C_ADDR),
        .irq = MSM_GPIO_TO_IRQ(QWERTY_INT_N_SIGNAL),
        .platform_data = &mot_adp5588_device
    },
    {    /* backlight */
        I2C_BOARD_INFO("lm3535", LM3535_I2C_ADDR),
        .irq = MSM_GPIO_TO_IRQ(LM3535_INT_N_SIGNAL)
    },
    {    /* touchscreen */
        I2C_BOARD_INFO("key08", KEY08_I2C_ADDR),
        .irq = MSM_GPIO_TO_IRQ(KEY08_INT_N_SIGNAL),
        .platform_data = &mot_key08_device
    },
    {    /* motus minipad */
        I2C_BOARD_INFO("minipad", MINIPAD_I2C_ADDR),
        .irq = MSM_GPIO_TO_IRQ(MINIPAD_INT_N_SIGNAL),
        .platform_data = &mot_minipad_device
    },
    {    /* camera */
        I2C_BOARD_INFO("mt9p012", 0x6C >> 1),
    },
};

static struct i2c_board_info i2c_zeppelin_devices[] = {
    {     /* keypad [kpd/interconnect pcb] */
        I2C_BOARD_INFO("adp5588_keypad", ADP5588_I2C_ADDR),
        .irq = MSM_GPIO_TO_IRQ(QWERTY_INT_N_SIGNAL),
        .platform_data = &mot_adp5588_device
    },
    {   /* backlight */
        I2C_BOARD_INFO("lm3535", LM3535_I2C_ADDR),
        .irq = MSM_GPIO_TO_IRQ(LM3535_INT_N_SIGNAL)
    },
    {    /* touchscreen */
        I2C_BOARD_INFO("qtm_obp", KEY08_I2C_ADDR),
        .irq = MSM_GPIO_TO_IRQ(KEY08_INT_N_SIGNAL),
        .platform_data = &mot_qtm_obp_device
    },
    {    /* zeppelin minipad */
        I2C_BOARD_INFO("minipad", MINIPAD_I2C_ADDR),
        .irq = MSM_GPIO_TO_IRQ(MINIPAD_INT_N_SIGNAL),
        .platform_data = &mot_minipad_device
    },
    {    /* camera */
        I2C_BOARD_INFO("mt9p012", 0x6C >> 1),
    },
};

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 16000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 20000,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 12000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].residency = 20000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled = 1,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 2000,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 0,
};




static void msm_i2c_gpio_config(int iface, int config_type)
{
	int gpio_scl;
	int gpio_sda;
	if (iface) {
		return;
	} else {
		gpio_scl = 60;
		gpio_sda = 61;
	}
	if (config_type) {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 1, GPIO_INPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 1, GPIO_INPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
	} else {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 0, GPIO_OUTPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 0, GPIO_OUTPUT,
					GPIO_NO_PULL, GPIO_16MA), GPIO_ENABLE);
	}
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 100000,
	/* SMEM_I2C_MUTEX */
	/* We don't need mutex, because AMSS doesn't
	* use i2c because AMSS doesn't use i2c
	*/
	/*.lock_val = 0,*/
	.rmutex = NULL,
	.pri_clk = 60,
	.pri_dat = 61,
	.aux_clk = 60,	/* we use the same pins for */
	.aux_dat = 61,   /* second adapter, just in different mode */
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (gpio_request(60, "i2c_pri_clk"))
		pr_err("failed to request gpio i2c_pri_clk\n");
	if (gpio_request(61, "i2c_pri_dat"))
		pr_err("failed to request gpio i2c_pri_dat\n");

	msm_i2c_pdata.pm_lat =
	msm_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency;

	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.start = MSM_PMEM_MDP_BASE,
	.size = MSM_PMEM_MDP_SIZE,
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.no_allocator = 1,
	.cached = 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.start = MSM_PMEM_ADSP_BASE,
	.size = MSM_PMEM_ADSP_SIZE,
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct resource resources_hw3d[] = {
	{
		.start	= 0xA0000000,
		.end	= 0xA00fffff,
		.flags	= IORESOURCE_MEM,
		.name	= "regs",
	},
	{
		.flags	= IORESOURCE_MEM,
		.start	= MSM_PMEM_GPU0_BASE,
		.end	= MSM_PMEM_GPU0_BASE + MSM_PMEM_GPU0_SIZE - 1,
		.name	= "smi",
	},
	{
		.flags	= IORESOURCE_MEM,
		.start	= 0,
		.name	= "ebi",
	},
	{
		.start	= INT_GRAPHICS,
		.end	= INT_GRAPHICS,
		.flags	= IORESOURCE_IRQ,
		.name	= "gfx",
	},
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device hw3d_device = {
	.name		= "msm_hw3d",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(resources_hw3d),
	.resource	= resources_hw3d,
};

static struct vibrator_platform_data vibrator_data = {
	.name = "vibrator",
	.gpio_en = HAPTICS_AMP_EN,
	.gpio_pwm = HAPTICS_MSM_PWM,
	.max_timeout = 15000,
	.active_low  = 0,
};

static struct platform_device mot_vibrator = {
	.name   = "vibrator",
	.id     = -1,
	.dev    = {
		.platform_data    = &vibrator_data,
	},
};

static struct sfh7743_platform_data sfh7743_data = {
    .name = "sfh7743",
    .gpio_en = SFH7743_GPIO_EN,
    .gpio_intr = SFH7743_GPIO_INTR,
};

static struct platform_device msm_sfh7743_device = {
    .name           = "sfh7743",
    .id             = -2,
    .dev            = {
	.platform_data = &sfh7743_data,
    },
};

#define SND(desc, num) { .name = #desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
    SND(HANDSET, 16),  		/* Redefine HANDSET to IN_S_SADC_OUT_HANDSET for dial mic */

    SND(HEADSET, 3),    	/* Stereo headset */
    SND(HEADPHONE, 4),    	/* Stereo headphone, no mic */
    SND(HEADSET_MOS, 5),    /* Stereo headset MOS */
    SND(SPEAKER, 25),       /* Redefine SPEAKER  to IN_S_SADC_OUT_SPEAKER_PHONE for dial mic */
    SND(DUALMIC_SPEAKER, 6),
    SND(TTY_HEADSET, 8),
    SND(TTY_VCO, 9),
    SND(TTY_HCO, 10),
    SND(BT,      12),
    SND(IN_S_SADC_OUT_HANDSET, 16),
    SND(IN_S_SADC_OUT_SPEAKER_PHONE, 25),
    SND(FM_HEADSET , 22),
    SND(FM_SPEAKER , 23),
    SND(CURRENT, 27),
};
#undef SND

static struct msm_snd_endpoints mot_snd_endpoints = {
    .endpoints = snd_endpoints_list,
    .num = sizeof(snd_endpoints_list) / sizeof(struct snd_endpoint)
};

static struct platform_device mot_snd = {
    .name = "msm_snd",
    .id = -1,
    .dev    = {
        .platform_data = &mot_snd_endpoints
    },
};

static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
  .wakeup_irq = MSM_GPIO_TO_INT(45),
  .inject_rx_on_wakeup = 0, /* 1; We have BT wakeup signal */
  .rx_to_inject = 0x32,
};

/* 
    USB stuff
    ---------------------------------------------------------------- 
 */
#ifdef CONFIG_USB_FUNCTION
static void hsusb_gpio_init(void)
{
	if (gpio_request(111, "ulpi_data_0"))
		pr_err("failed to request gpio ulpi_data_0\n");
	if (gpio_request(112, "ulpi_data_1"))
		pr_err("failed to request gpio ulpi_data_1\n");
	if (gpio_request(113, "ulpi_data_2"))
		pr_err("failed to request gpio ulpi_data_2\n");
	if (gpio_request(114, "ulpi_data_3"))
		pr_err("failed to request gpio ulpi_data_3\n");
	if (gpio_request(115, "ulpi_data_4"))
		pr_err("failed to request gpio ulpi_data_4\n");
	if (gpio_request(116, "ulpi_data_5"))
		pr_err("failed to request gpio ulpi_data_5\n");
	if (gpio_request(117, "ulpi_data_6"))
		pr_err("failed to request gpio ulpi_data_6\n");
	if (gpio_request(118, "ulpi_data_7"))
		pr_err("failed to request gpio ulpi_data_7\n");
	if (gpio_request(119, "ulpi_dir"))
		pr_err("failed to request gpio ulpi_dir\n");
	if (gpio_request(120, "ulpi_next"))
		pr_err("failed to request gpio ulpi_next\n");
	if (gpio_request(121, "ulpi_stop"))
		pr_err("failed to request gpio ulpi_stop\n");
}

static unsigned usb_gpio_lpm_config[] = {
	GPIO_CFG(111, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),	/* DATA 0 */
	GPIO_CFG(112, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),	/* DATA 1 */
	GPIO_CFG(113, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),	/* DATA 2 */
	GPIO_CFG(114, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),	/* DATA 3 */
	GPIO_CFG(115, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),	/* DATA 4 */
	GPIO_CFG(116, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),	/* DATA 5 */
	GPIO_CFG(117, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),	/* DATA 6 */
	GPIO_CFG(118, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),	/* DATA 7 */
	GPIO_CFG(119, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),	/* DIR */
	GPIO_CFG(120, 1, GPIO_INPUT,  GPIO_NO_PULL, GPIO_8MA),	/* NEXT */
	GPIO_CFG(121, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),	/* STOP */
};

static unsigned usb_gpio_lpm_unconfig[] = {
	GPIO_CFG(111, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_8MA), /* DATA 0 */
	GPIO_CFG(112, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_8MA), /* DATA 1 */
	GPIO_CFG(113, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_8MA), /* DATA 2 */
	GPIO_CFG(114, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_8MA), /* DATA 3 */
	GPIO_CFG(115, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_8MA), /* DATA 4 */
	GPIO_CFG(116, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_8MA), /* DATA 5 */
	GPIO_CFG(117, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_8MA), /* DATA 6 */
	GPIO_CFG(118, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_8MA), /* DATA 7 */
	GPIO_CFG(119, 1, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_8MA), /* DIR */
	GPIO_CFG(120, 1, GPIO_INPUT,  GPIO_PULL_DOWN, GPIO_8MA), /* NEXT */
	GPIO_CFG(121, 1, GPIO_OUTPUT, GPIO_PULL_UP,   GPIO_8MA), /* STOP */
};

static int usb_config_gpio(int config)
{
	int pin, rc;

	if (config) {
		for (pin = 0; pin < ARRAY_SIZE(usb_gpio_lpm_config); pin++) {
			rc = gpio_tlmm_config(usb_gpio_lpm_config[pin],
								GPIO_ENABLE);
			if (rc) {
				printk(KERN_ERR
					"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, usb_gpio_lpm_config[pin], rc);
				return -EIO;
			}
		}
	} else {
		for (pin = 0; pin < ARRAY_SIZE(usb_gpio_lpm_unconfig); pin++) {
			rc = gpio_tlmm_config(usb_gpio_lpm_unconfig[pin],
								GPIO_ENABLE);
			if (rc) {
				printk(KERN_ERR
					"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, usb_gpio_lpm_config[pin], rc);
				return -EIO;
			}
		}
	}

	return 0;
}

/* bit shifts for USB functions */
static struct usb_function_map usb_functions_map[] = {
	{"diag",		0},
	{"adb",			1},
	{"ethernet",		2},
	{"modem",		3},
	{"usb_mass_storage",	4},
#ifdef CONFIG_EXPOSE_NMEA
	{"nmea",		5},
#endif
};

/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
#ifdef CONFIG_EXPOSE_NMEA
    {
        .product_id     = 0x2d61,
        .functions        = 0x3F,     /* 00111111 nmea/ums/modem/ether/adb/diag */
        .config         = "Motorola Android Composite Device",

    },
#else
    {
        .product_id     = 0x2d61,
        .functions        = 0x1F,        /* 00011111 ums/modem/ether/adb/diag */
        .config         = "Motorola Config 38",

    },
#endif
    {
        .product_id     = 0x2d63,
        .functions      = 0x1D,     /* 00011101 ums/modem/ether/diag */
        .config         = "Motorola Config 39",
    },
    {
        .product_id     = 0x2d64,
        .functions      = 0x0F,     /* 00001111 modem/ether/adb/diag */
        .config         = "Motorola Config 40",
    },
    {
        .product_id     = 0x2d65,
        .functions      = 0x0D,     /* 00001101 modem/ether/diag */
        .config         = "Motorola Config 41",
    },
    {
        .product_id     = 0x2d66,
        .functions      = 0x12,     /* 00010010 ums/adb */
        .config         = "Motorola Config 42",
    },
    {
        .product_id     = 0x2d67,
        .functions      = 0x10,     /* 00010000 ums */
        .config         = "Motorola Config 14",
    },
    {
        .product_id     = 0x2d68,
        .functions      = 0x07,     /* 00000111 ether/adb/diag */
        .config         = "Motorola Config 48",
    },
    {
        .product_id     = 0x2d69,
        .functions      = 0x05,     /* 00000101 ether/diag */
        .config         = "Motorola Config 47",
    },
};
#endif /* CONFIG_USB_FUNCTION */

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
#ifdef CONFIG_USB_FUNCTION
	.version	= 0x0100,
	.phy_info	= USB_PHY_EXTERNAL,
	.vendor_id	= 0x22b8,
	.product_id	= 0x2d61,
	.product_name	= "Morrison",
	.serial_number	= "1234567890ABCDEF",
	.manufacturer_name	= "Motorola ",
	.compositions		= usb_func_composition,
	.num_compositions	= ARRAY_SIZE(usb_func_composition),
	.function_map		= usb_functions_map,
	.num_functions		= ARRAY_SIZE(usb_functions_map),
	.ulpi_data_1_pin	= 112,
	.ulpi_data_3_pin	= 114,
	.config_gpio		= usb_config_gpio,
#endif
};

#ifdef CONFIG_USB_FUNCTION_MASS_STORAGE
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns = 1,
	.buf_size = 16384,
	.vendor = "Motorola ",
	.product = "Morrison",
	.release = 0x0010,
};

static struct platform_device usb_mass_storage_device = {
	.name = "usb_mass_storage",
	.id = -1,
	.dev = {
		.platform_data = &mass_storage_pdata,
	},
};
#endif


/* 
    SDCC stuff 
    ---------------------------------------------------------------- 
 */
static void sdcc_gpio_init(void)
{
    int rc = 0;

    if (gpio_request(SD_DETECT_N_SIGNAL, "sdc1_status_irq"))
		pr_err("failed to request gpio sdc1_status_irq\n");
    rc = gpio_tlmm_config(GPIO_CFG(SD_DETECT_N_SIGNAL, 0, GPIO_INPUT, GPIO_NO_PULL,
		GPIO_2MA), GPIO_ENABLE);
    if (rc)
	printk(KERN_ERR "%s: Failed to configure GPIO %d\n",
		__func__, rc);

	/* SDC1 GPIOs */
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	if (gpio_request(51, "sdc1_data_3"))
		pr_err("failed to request gpio sdc1_data_3\n");
	if (gpio_request(52, "sdc1_data_2"))
		pr_err("failed to request gpio sdc1_data_2\n");
	if (gpio_request(53, "sdc1_data_1"))
		pr_err("failed to request gpio sdc1_data_1\n");
	if (gpio_request(54, "sdc1_data_0"))
		pr_err("failed to request gpio sdc1_data_0\n");
	if (gpio_request(55, "sdc1_cmd"))
		pr_err("failed to request gpio sdc1_cmd\n");
	if (gpio_request(56, "sdc1_clk"))
		pr_err("failed to request gpio sdc1_clk\n");
#endif

	/* SDC2 GPIOs */
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	if (gpio_request(62, "sdc2_clk"))
		pr_err("failed to request gpio sdc2_clk\n");
	if (gpio_request(63, "sdc2_cmd"))
		pr_err("failed to request gpio sdc2_cmd\n");
	if (gpio_request(64, "sdc2_data_3"))
		pr_err("failed to request gpio sdc2_data_3\n");
	if (gpio_request(65, "sdc2_data_2"))
		pr_err("failed to request gpio sdc2_data_2\n");
	if (gpio_request(66, "sdc2_data_1"))
		pr_err("failed to request gpio sdc2_data_1\n");
	if (gpio_request(67, "sdc2_data_0"))
		pr_err("failed to request gpio sdc2_data_0\n");
	if (gpio_request(WLAN_HOST_IRQ, "host_wlan_irq "))
		pr_err("failed to request gpio host_wlan_irq\n");
#endif
}
/* WiFi static items */
static unsigned int wlan_vreg_enabled = 0;
static void (*wlan_status_notify)(int card_present, void *dev_id) = NULL;
static unsigned long *wlan_host_ptr = NULL;

static unsigned sdcc_cfg_data[][7] = {
    /* SDCC_CMD should be set to 8mA (7627 GPIO 55 and 63)*/
	/* SDC1 configs */
	{
	GPIO_CFG(51, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	GPIO_CFG(52, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	GPIO_CFG(53, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	GPIO_CFG(54, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	GPIO_CFG(55, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA),
	GPIO_CFG(56, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA),
	GPIO_CFG(56, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), /* 7th item in array */
	},
	/* SDC2 configs */
	{
	GPIO_CFG(62, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(63, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(64, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(65, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(66, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(67, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA),
	GPIO_CFG(WLAN_HOST_IRQ , 0, GPIO_INPUT,  GPIO_PULL_UP, GPIO_2MA),
	},
};

static unsigned long vreg_sdcc1_sts, vreg_sdcc2_sts, gpio_sts;

static void msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int i, rc;

	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return;

	if (enable)
		set_bit(dev_id, &gpio_sts);
	else
		clear_bit(dev_id, &gpio_sts);

	for (i = 0; i < ARRAY_SIZE(sdcc_cfg_data[dev_id - 1]); i++) {
		rc = gpio_tlmm_config(sdcc_cfg_data[dev_id - 1][i],
			enable ? GPIO_ENABLE : GPIO_DISABLE);
		if (rc)
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, sdcc_cfg_data[dev_id - 1][i], rc);
	}
}

uint32_t msm_sdcc1_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;
	struct vreg *vreg;

	pdev = container_of(dv, struct platform_device, dev);

	if (pdev->id == 1) /* SD */
		vreg = vreg_get(0, "wlan");
	else {
		printk(KERN_ERR "%s: unsupported sdcc port (%d) selected!\n", __func__, pdev->id);
		return -1;
	}

	if (vdd == 0) {
	/* Reconfigure GPIO's before disabling VREG */
		msm_sdcc_setup_gpio(pdev->id, !!vdd);

		if (!vreg_sdcc1_sts)
			return 0;

		clear_bit(pdev->id, &vreg_sdcc1_sts);

		if (!vreg_sdcc1_sts) {
			if ((rc = vreg_disable(vreg)))
				printk(KERN_ERR "%s: vreg(wlan) for MMC disable failed (%d)\n", __func__, rc);
		msleep(7); /* SanDisk requirement to allow VDD to get to 0.3v for 1 msec */
		}
		return 0;
	}

	if (!vreg_sdcc1_sts) {
		rc = vreg_set_level(vreg, 2850);
		if (rc) {
			printk(KERN_ERR "%s: vreg set level failed (%d)\n",
				 __func__, rc);
			return -1;
		}

		if ((rc = vreg_enable(vreg)))
			printk(KERN_ERR "%s: vreg(wlan) for MMC enable failed (%d)\n", __func__, rc);
	}
	set_bit(pdev->id, &vreg_sdcc1_sts);
    /* Reconfigure GPIO's only after enabling VREG */
    msm_sdcc_setup_gpio(pdev->id, !!vdd);

	return 0;
}

uint32_t msm_sdcc2_setup_power(struct device *dv, unsigned int vdd)
{
#ifndef CONFIG_MACH_MOT
	int rc = 0;
	/* struct platform_device *pdev; */
	struct vreg *vreg;
#endif
	/* pdev = container_of(dv, struct platform_device, dev);*/
	/* msm_sdcc_setup_gpio(pdev->id, !!vdd);*/
	msm_sdcc_setup_gpio(2, !!vdd);

	if ((vdd && wlan_vreg_enabled) || (!vdd && !wlan_vreg_enabled))
		return 0;

	if (vdd == 0) {
		if (!vreg_sdcc2_sts)
			return 0;

		/* clear_bit(pdev->id, &vreg_sdcc2_sts); */
		clear_bit(2, &vreg_sdcc2_sts);

		if (!vreg_sdcc2_sts) {
			/* WiFi DEBUG INFO:*/
			printk(KERN_ERR "msm_sdcc_setup_power - DISABLING...WIFI\n");
			gpio_request(WLAN_RST_N, "wlan_reset_n"); /* put the WLAN in reset */
			gpio_direction_output(WLAN_RST_N, 0);
			msleep_interruptible(100);              /* do we really need this much? */
			gpio_request(WLAN_REG_ON_SIGNAL, "wlan_reg_on"); /* turn the WLAN internal VREG off */
			gpio_direction_output(WLAN_REG_ON_SIGNAL, 0);
			msleep_interruptible(100);  /* do we really need this much? */
			wlan_vreg_enabled = 0;
		}
		return 0;
	}

	if (!vreg_sdcc2_sts) {
		/* WiFi DEBUG INFO: */
		printk(KERN_ERR "msm_sdcc_setup_power - ENABLING...WIFI\n");
		gpio_request(WLAN_REG_ON_SIGNAL, "wlan_reg_on");	/* turn on WLAN internal VREG */
		gpio_direction_output(WLAN_REG_ON_SIGNAL, 1);
		msleep_interruptible(100);				/* BCM4325 powerup requirement */
		gpio_request(WLAN_RST_N, "wlan_reset_n");		/* take WLAN out of reset */
		gpio_direction_output(WLAN_RST_N, 1);
		msleep_interruptible(100);				/* BCM4325 powerup requirement */
		wlan_vreg_enabled = 1;
	}
	/* set_bit(pdev->id, &vreg_sdcc2_sts); */
	set_bit(2, &vreg_sdcc2_sts);
	return 0;
}

/*
 * Mot HW uses two SD slots - the SD card and BT/WIFI interface
 */

#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
static unsigned int mot_7x01_sdcc_slot1_status(struct device *dev)
{
	unsigned val;
	/*
	 * SD_DETECT is GPIO 17 on Calgary
	 */
	val = gpio_get_value(SD_DETECT_N_SIGNAL);
	return (1 - val);
}

static unsigned int mot_7x01_sdcc_slot2_status(struct device *dev)
{
	/*
	 * This is hooked up to BT/WIFI chip.
	 */
	return wlan_vreg_enabled;
}
#endif
/* WiFi */
static int mot_sdcc_wifi_status_notify_register(void (*notify_callback)(int card_present, void *dev_id), void *dev)
{
    if ((NULL != notify_callback) && (NULL == wlan_status_notify)) {
	wlan_status_notify = notify_callback;
	wlan_host_ptr = dev;
	printk(KERN_INFO "%s:  status_notify callback registered dev_ptr %p\n", __FUNCTION__, dev);
	} else {
	printk(KERN_INFO "%s:  status_notify callback registration skipped dev_ptr %p\n", __FUNCTION__, dev);
    }
    return 0;
}

static struct mmc_platform_data mot_7x01_sdcc1_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc1_setup_power,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status         = mot_7x01_sdcc_slot1_status,
#endif

};

static struct mmc_platform_data mot_7x01_sdcc2_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd  = msm_sdcc2_setup_power,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status         = mot_7x01_sdcc_slot2_status,
	.register_status_notify    = mot_sdcc_wifi_status_notify_register,
#endif
};

static void __init mot_7x01_init_mmc(void)
{
    unsigned on_off = 1;  /* Turn on pulldown when VREG is disabled */
    unsigned id = 15; /*vreg_mmc->id*/;
	msm_add_sdcc(1, &mot_7x01_sdcc1_data, MSM_GPIO_TO_INT(SD_DETECT_N_SIGNAL), IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING); /* SD slot */

    /* Setup VREG to pulldown when off */
	msm_proc_comm(PCOM_VREG_PULLDOWN, &on_off, &id);

	set_irq_wake(MSM_GPIO_TO_INT(SD_DETECT_N_SIGNAL), 1);
}

static void __init mot_7x01_init_wlan(void)
{
	msm_add_sdcc(2, &mot_7x01_sdcc2_data, 0, 0); /* BRCM 4325 */
}

/*
    Camera stuff
    ----------------------------------------------------------------
*/
static uint32_t camera_off_gpio_table[] = {
    /* GPIO input mode, lowest drain */
    GPIO_CFG(2,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT2 */
    GPIO_CFG(3,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT3 */
    GPIO_CFG(4,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT4 */
    GPIO_CFG(5,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT5 */
    GPIO_CFG(6,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT6 */
    GPIO_CFG(7,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT7 */
    GPIO_CFG(8,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT8 */
    GPIO_CFG(9,  0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT9 */
    GPIO_CFG(10, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT10 */
    GPIO_CFG(11, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* DAT11 */
    GPIO_CFG(12, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* PCLK */
    GPIO_CFG(13, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* HSYNC_IN */
    GPIO_CFG(14, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), /* VSYNC_IN */
    GPIO_CFG(15, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),  /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
   /* parallel CAMERA interfaces, functional mode 1 */
   GPIO_CFG(2,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),  /* DAT2 */
   GPIO_CFG(3,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),  /* DAT3 */
   GPIO_CFG(4,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),  /* DAT4 */
   GPIO_CFG(5,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),  /* DAT5 */
   GPIO_CFG(6,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),  /* DAT6 */
   GPIO_CFG(7,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),  /* DAT7 */
   GPIO_CFG(8,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),  /* DAT8 */
   GPIO_CFG(9,  1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),  /* DAT9 */
   GPIO_CFG(10, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),  /* DAT10 */
   GPIO_CFG(11, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),  /* DAT11 */
   GPIO_CFG(12, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_16MA), /* PCLK */
   GPIO_CFG(13, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),  /* HSYNC_IN */
   GPIO_CFG(14, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),  /* VSYNC_IN */
   GPIO_CFG(15, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA),/* MCLK */
};

static void config_gpio_table(uint32_t *table, int len)
{
    int n, rc;
    for (n = 0; n < len; n++) {
        rc = gpio_tlmm_config(table[n], GPIO_ENABLE);
        if (rc) {
            printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
                __func__, table[n], rc);
            break;
        }
    }
}

static void config_camera_on_gpios(void)
{
    config_gpio_table(camera_on_gpio_table,
        ARRAY_SIZE(camera_on_gpio_table));
}

static void config_camera_off_gpios(void)
{
    config_gpio_table(camera_off_gpio_table,
        ARRAY_SIZE(camera_off_gpio_table));
}

#define MSM_PROBE_INIT(name) name##_probe_init
static struct msm_camera_sensor_info msm_camera_sensor[] = {
    {
        .sensor_reset	= CAM_RESET_N_SIGNAL,
        .sensor_pwd	= CAM_PWD_SIGNAL,
        .vcm_pwd	= 0, /* w16959 - AF doesn't have separate pwr */
        .sensor_name	= "mt9p012",
        .flash_type	= MSM_CAMERA_FLASH_LED,
	.sensor_probe   = MSM_PROBE_INIT(mt9p012),

    },
/* Always have the MOT_CAMERA for the ISE towards the last of the table. */
    {
        .sensor_reset	= CAM_RESET_N_SIGNAL,
        .sensor_pwd	= CAM_PWD_SIGNAL,
        .vcm_pwd	= 0,
        .sensor_name	= "mot_camera",
        .flash_type	= MSM_CAMERA_FLASH_NONE,
	.sensor_probe = MSM_PROBE_INIT(mot_camera),
    },

};
#undef MSM_PROBE_INIT

static struct msm_camera_device_platform_data msm_camera_device_data = {
    .camera_gpio_on  = config_camera_on_gpios,
    .camera_gpio_off = config_camera_off_gpios,
    .snum = ARRAY_SIZE(msm_camera_sensor),
    .sinfo = &msm_camera_sensor[0],
    .ioext.mdcphy = MSM_MDC_PHYS,
    .ioext.mdcsz  = MSM_MDC_SIZE,
    .ioext.appphy = MSM_CLK_CTL_PHYS,
    .ioext.appsz  = MSM_CLK_CTL_SIZE,
};

static struct platform_device msm_camera_sensor_mot_mt9p012 = {
	.name = "msm_camera",
	.dev = {
		.platform_data = &msm_camera_device_data,
	},
};

/*
	Bluetooth stuff
----------------------------------------------------------------
*/
#ifdef CONFIG_BT

static struct platform_device msm_bt_power_device = {
    .name = "bt_power",
};

enum {
	BT_WAKE,
	BT_RFR,
	BT_CTS,
	BT_RX,
	BT_TX,
	BT_PCM_DOUT,
	BT_PCM_DIN,
	BT_PCM_SYNC,
	BT_PCM_CLK,
	BT_HOST_WAKE,
};

static unsigned bt_config_power_on[] = {
/* GPIO_CFG(91, 0, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_2MA), */ /* WAKE */
    GPIO_CFG(91, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),    /* WAKE */
    GPIO_CFG(43, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),    /* RFR */
    GPIO_CFG(44, 2, GPIO_INPUT,  GPIO_PULL_UP, GPIO_2MA),    /* CTS */
    GPIO_CFG(45, 2, GPIO_INPUT,  GPIO_PULL_UP, GPIO_2MA),    /* Rx */
    GPIO_CFG(46, 3, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA),    /* Tx */
    GPIO_CFG(68, 1, GPIO_OUTPUT, GPIO_KEEPER, GPIO_2MA),    /* PCM_DOUT */
    GPIO_CFG(69, 1, GPIO_INPUT,  GPIO_KEEPER, GPIO_2MA),    /* PCM_DIN */
    GPIO_CFG(70, 2, GPIO_OUTPUT, GPIO_KEEPER, GPIO_2MA),    /* PCM_SYNC */
    GPIO_CFG(71, 2, GPIO_OUTPUT, GPIO_KEEPER, GPIO_2MA),    /* PCM_CLK */
    GPIO_CFG(90, 0, GPIO_INPUT,  GPIO_PULL_UP, GPIO_2MA),    /* HOST_WAKE */
};
static unsigned bt_config_power_off[] = {
/*    GPIO_CFG(91, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),*/  /* WAKE */
    GPIO_CFG(91, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),    /* WAKE */
    GPIO_CFG(43, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),    /* RFR */
    GPIO_CFG(44, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),    /* CTS */
    GPIO_CFG(45, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),    /* Rx */
    GPIO_CFG(46, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),    /* Tx */
    GPIO_CFG(68, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),    /* PCM_DOUT */
    GPIO_CFG(69, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),    /* PCM_DIN */
    GPIO_CFG(70, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),    /* PCM_SYNC */
    GPIO_CFG(71, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),    /* PCM_CLK */
    GPIO_CFG(90, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA),    /* HOST_WAKE */
};

static int bluetooth_power(int on)
{
    struct vreg *vreg_bt;
    int pin, rc;

    printk(KERN_INFO "BLUETOOTH: bluetooth_power(%d) (board-mot init)\n", on);

    printk(KERN_DEBUG "%s\n", __func__);

    /* VREG_GP3 is supplying power to BT/WiFi */
    vreg_bt = vreg_get(0, "gp3");

    if (!vreg_bt) {
	printk(KERN_ERR "%s: vreg get failed\n", __func__);
	return -EIO;
    }

    if (on) {
	for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on); pin++) {
		rc = gpio_request(0x3ff & (bt_config_power_on[pin] >> 4),
			"BT PIN");
		if (rc) {
		printk(KERN_ERR
			"%s.%d: gpio_request()=%d [gpio #%d, array idx=%d]\n",
			__func__, __LINE__, rc,
			0x3ff & (bt_config_power_on[pin] >> 4), pin);
		}

	rc = gpio_tlmm_config(bt_config_power_on[pin], GPIO_ENABLE);

	if (rc) {
		printk(KERN_ERR
			"%s.%d: gpio_tlmm_config(%#x)=%d [gpio #%d, array idx=%d]\n",
			__func__, __LINE__, bt_config_power_on[pin], rc,
			0x3ff & (bt_config_power_on[pin] >> 4), pin);
		/* return -EIO;*/
		}
	}

	/* units of mV, steps of 50 mV */
	rc = vreg_set_level(vreg_bt, 2600);
	if (rc) {
		printk(KERN_ERR "%s.%d: vreg set level failed (%d)\n",
			__func__, __LINE__, rc);
		return -EIO;
	}
	rc = vreg_enable(vreg_bt);
	if (rc) {
		printk(KERN_ERR "%s.%d: vreg enable failed (%d)\n",
			__func__, __LINE__, rc);
		return -EIO;
	}

	/* turn on internal BT VREG*/
	gpio_request(BT_REG_ON_SIGNAL, "bt_reg_on");
	gpio_direction_output(BT_REG_ON_SIGNAL, 1);

	/* BCM4325 powerup requirement*/
	msleep_interruptible(100);

	/* take BT out of reset*/
	gpio_request(BT_RESET_N_SIGNAL, "bt_reset_n");
	gpio_direction_output(BT_RESET_N_SIGNAL, 1);

	} else {

	/* turn off internal BT VREG */
	gpio_request(BT_REG_ON_SIGNAL, "bt_reg_on");
	gpio_direction_output(BT_REG_ON_SIGNAL, 0);

	rc = vreg_disable(vreg_bt);
	if (rc) {
		printk(KERN_ERR "%s.%d: vreg disable failed (%d)\n",
			__func__, __LINE__, rc);
		return -EIO;
	}
	for (pin = 0; pin < ARRAY_SIZE(bt_config_power_off); pin++) {
		rc = gpio_tlmm_config(bt_config_power_off[pin],
						GPIO_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s.%d: gpio_tlmm_config(%#x)=%d\n",
				__func__, __LINE__, bt_config_power_off[pin], rc);
			return -EIO;
			}
		}
	}
	return 0;
}

void __init bt_power_init(void)
{
	msm_bt_power_device.dev.platform_data = &bluetooth_power;
}

static struct resource bluesleep_resources[] = {
	{
		.name    = "gpio_host_wake",
		.start    = BT_HOST_WAKE_SIGNAL,
		.end    = BT_HOST_WAKE_SIGNAL,
		.flags    = IORESOURCE_IO,
	},
	{
		.name    = "gpio_ext_wake",
		.start    = BT_EXT_WAKE_SIGNAL,
		.end    = BT_EXT_WAKE_SIGNAL,
		.flags    = IORESOURCE_IO,
	},
	{
		.name    = "host_wake",
		.start    = MSM_GPIO_TO_INT(BT_HOST_WAKE_SIGNAL),
		.end    = MSM_GPIO_TO_INT(BT_HOST_WAKE_SIGNAL),
		.flags    = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_bluesleep_device = {
	.name = "bluesleep",
	.id        = -1,
	.num_resources    = ARRAY_SIZE(bluesleep_resources),
	.resource    = bluesleep_resources,
};

#else
#define bt_power_init(x) do {} while (0)
#endif /* CONFIG_BT */

/* 
    List of platform devices to use by init_machine 
 */
static struct platform_device *devices[] __initdata = {
	&msm_device_uart3,
	&msm_device_uart_dm1,
	&msm_device_smd,
	&msm_device_nand,
	&android_pmem_device,
	&android_pmem_adsp_device,
	&hw3d_device,
	&msm_device_i2c_gpio,
	&msm_device_i2c,
#if defined(CONFIG_USB_FUNCTION)
	&msm_device_hsusb_peripheral,
#endif
#ifdef CONFIG_USB_FUNCTION_MASS_STORAGE
	&usb_mass_storage_device,
#endif
#ifdef CONFIG_BT
	&msm_bt_power_device,
	&msm_bluesleep_device,
#endif
	&mot_snd,
	&msm_fb_device,
	&msm_sfh7743_device,     /* Proximity sensor */
#ifdef CONFIG_ANDROID_RAM_CONSOLE
	&ramconsole_device,
#endif
	&mot_vibrator,          /* Vibrator */
};

static struct msm_acpu_clock_platform_data mot_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 256000,
	.vdd_switch_time_us = 62,
	.power_collapse_khz = 19200000,
	.wait_for_irq_khz = 128000000,
	.max_axi_khz = 128000,
};

/* Called by init_machine */
void msm_serial_debug_init(unsigned int base, int irq,
		struct device *clk_device, int signal_irq);

void morrison_setup(void)
{
    unsigned size;
    
    printk(KERN_INFO "%s, hw_rev: %x ...\n", __func__, mot_hw_rev);

    if (mot_hw_rev < 0x20) /* legacy HW */
    {
        stmicro_resources[0].start = LIS331DLH_INT1_LEGACY;
        i2c_morrison_devices[0].irq = MSM_GPIO_TO_IRQ(QWERTY_INT_N_SIGNAL_LEGACY);
        mot_adp5588_device.name = "adp5588_keypad";

        /* Re-assign GPIOs and cnange pulls */
        gpio_request(18, "HAPTICS_PWM");
        gpio_request(21, "SEND_END_DET");
        gpio_request(27, "QWERTY_INT");
        gpio_request(28, "SLIDER_INT_N");
        gpio_request(41, "EL_PWM");
        gpio_request(84, "HEAD_SEL_N");

        gpio_tlmm_config(GPIO_CFG(18, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);    /* HAPTICS_PWM */
        gpio_tlmm_config(GPIO_CFG(21, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);     /* SEND_END_DET */
        gpio_tlmm_config(GPIO_CFG(27, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);     /* QWERTY_INT */
        gpio_tlmm_config(GPIO_CFG(28, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE);   /* ACCEL_INT */
        gpio_tlmm_config(GPIO_CFG(41, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);     /* SLIDER_INT_N */
        gpio_tlmm_config(GPIO_CFG(77, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);    /* EL_PWM */
        gpio_tlmm_config(GPIO_CFG(84, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);    /* HEAD_SEL_N */
    }
    else if (mot_hw_rev >= 0x2D)    /* P2B */
    {
        gpio_request(100, "fairchild_led_blinker");
        gpio_tlmm_config(GPIO_CFG(100, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);    /* HEAD_SEL_N */
        gpio_direction_output(100, 0);
    }

    if (mot_hw_rev >= 0x20  && mot_hw_rev < 0x30)
    {
        mot_adp5588_device.name = "adp5588_morrison_P2";
    }

    if (mot_hw_rev >= 0x20)
    {
        gpio_tlmm_config(GPIO_CFG(SFH7743_GPIO_INTR, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);     /* proximity */
    }

    if (mot_hw_rev >= 0x30) /* P3A runs spurious interrupts without pullup */
    {
        gpio_tlmm_config(GPIO_CFG(SFH7743_GPIO_INTR, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);     /* proximity */
    }

    if(!model_set) {
        msm_hsusb_pdata.product_name = "MB200";
#ifdef CONFIG_USB_FUNCTION_MASS_STORAGE
        mass_storage_pdata.product = "MB200";
#endif
    }

    gpio_request(COMPASS_RST_N_SIGNAL, "ecompass_rst_n");
    gpio_direction_output(COMPASS_RST_N_SIGNAL, 1);      		/* enable ecompass */

    msm_camera_sensor[0].flash_type = MSM_CAMERA_FLASH_NONE;    /* no flash */

	/* don't even bother with those devices on bare board */
	if (!vendor1->bare_board)	
	{
	    gpio_request(QWERTY_RST_N_SIGNAL, "qwerty_rst_n");
	    gpio_direction_output(QWERTY_RST_N_SIGNAL, 1);          /* enable keypad */

	    gpio_request(TOUCH_RST_N_SIGNAL, "touch_rst_n");
	    gpio_direction_output(TOUCH_RST_N_SIGNAL, 1);        	/* enable touchpad */
	    
	    size = ARRAY_SIZE(i2c_morrison_devices);
	    if (mot_hw_rev < 0x2D ) size--;                         /* chop off camera on anything older than P2B */
	    if (mot_hw_rev < 0x20 ) size -= 2;                      /* chop off e-compass & couloumb counter on anything older than P2 */
	    i2c_register_board_info(0, i2c_morrison_devices, size);
	}
}

void motus_setup(void)
{
    printk(KERN_INFO "%s, hw_rev: %x ...\n", __func__, mot_hw_rev);

    mot_adp5588_device.id = 1;    /*  keypad lock & no ringer switch & no light control ) */
    mot_stmicro_device.id = 3;    /* tells accelerometer driver to swap axes */

	if (mot_hw_rev < 0x20) 		/* P1 keyboard layout */
	{
		mot_adp5588_device.name = "adp5588_motus_P1";
	}
	else if (mot_hw_rev < 0x30) 	/* P2 keyboard layout */
	{
		mot_adp5588_device.name = "adp5588_motus_P2";
	}	
	else if(mot_hw_rev < 0x0400) /* P3 keypad layout */
	{
		mot_adp5588_device.name = "adp5588_motus_P3";
	} else
	{
		mot_adp5588_device.name = "adp5588_motus";
    }

    if(!model_set) {
        msm_hsusb_pdata.product_name = "MB300";
#ifdef CONFIG_USB_FUNCTION_MASS_STORAGE
        mass_storage_pdata.product = "MB300";
#endif
    }

    /* Need to change pulls on couple GPIOs */
    gpio_tlmm_config(GPIO_CFG(20, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);    /* FLIP_OPEN_B */
    gpio_tlmm_config(GPIO_CFG(83, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);    /* FLASH_CAM_KEY */

    gpio_request(COMPASS_RST_N_SIGNAL, "ecompass_rst_n");
    gpio_direction_output(COMPASS_RST_N_SIGNAL, 1);            	/* enable ecompass */

	/* don't even bother with those devices on bare board */
	if (!vendor1->bare_board)	
	{
	    gpio_request(QWERTY_RST_N_SIGNAL, "qwerty_rst_n");
	    gpio_direction_output(QWERTY_RST_N_SIGNAL, 1);          /* enable keypad */

	    gpio_request(TOUCH_RST_N_SIGNAL, "touch_rst_n");
	    gpio_direction_output(TOUCH_RST_N_SIGNAL, 1);           /* enable touchpad */
	    
	    i2c_register_board_info(0, i2c_motus_devices, ARRAY_SIZE(i2c_motus_devices));
	}
}

void zeppelin_setup(void)
{
    printk(KERN_INFO "%s, hw_rev: %x ...\n", __func__, mot_hw_rev);

    /* Set unused GPIOs to input/pulldown */
    gpio_request(20, "SEND_CENTER_KEY");
    gpio_request(28, "EL_PWM");
    gpio_request(42, "SLIDE_DETECT");
    gpio_request(94, "SLIDE_BACK_KEY");

    gpio_tlmm_config(GPIO_CFG(20, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE);
    gpio_tlmm_config(GPIO_CFG(28, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE);
    gpio_tlmm_config(GPIO_CFG(42, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE);
    gpio_tlmm_config(GPIO_CFG(94, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA), GPIO_ENABLE);
    gpio_tlmm_config(GPIO_CFG(83, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);		/* Minipad interupt */
	
    mot_adp5588_device.name = "adp5588_zeppelin";

    if(!model_set) {
        msm_hsusb_pdata.product_name = "Zeppelin";    
#ifdef CONFIG_USB_FUNCTION_MASS_STORAGE
        mass_storage_pdata.product = "Zeppelin";
#endif
    }

    mot_minipad_device.name = "minipad_zeppelin";
    
    gpio_request(COMPASS_RST_N_SIGNAL, "ecompass_rst_n");
    gpio_direction_output(COMPASS_RST_N_SIGNAL, 1);            	/* enable ecompass */

    gpio_request(100, "fairchild_led_blinker");
    gpio_tlmm_config(GPIO_CFG(100, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA), GPIO_ENABLE);
    gpio_direction_output(100, 0);
    
	/* don't even bother with those devices on bare board */
	if (!vendor1->bare_board)	
	{
	    gpio_request(TOUCH_RST_N_SIGNAL, "touch_rst_n");
	    gpio_direction_output(TOUCH_RST_N_SIGNAL, 1);           /*  enable touchpad */

	    i2c_register_board_info(0, i2c_zeppelin_devices, ARRAY_SIZE(i2c_zeppelin_devices));
	}
}

/* 
 * Called by map_io 
 * If buffers have no pre-polulated physical address, allocate them dynamically.
 */

void __init msm_mot_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;
	size = MSM_PMEM_MDP_SIZE;
	if(!android_pmem_pdata.start)
	{
		addr = alloc_bootmem(size);
		android_pmem_pdata.start = __pa(addr);
		android_pmem_pdata.size = size;
		printk(KERN_INFO "Allocating %lu bytes at %p (%lx physical)"
			" for pmem\n", size, addr, __pa(addr));
	}

	size = MSM_PMEM_ADSP_SIZE;
	if (machine_is_motus()) {
		size += MSM_PMEM_ADSP_FLIP_SIZE; /* need more ADSP space to perform image flip & rotate */
		// It would be nice if there was a more elegant solution, but for now Motus will
		// have to burn an extra 20MB of RAM for this.  Morrison/Zeppelin will keep ADSP in SMI
		// because the 12 MB ADSP fits in SMI.
		android_pmem_adsp_pdata.start = 0;
	}
	if(!android_pmem_adsp_pdata.start)
	{
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		printk(KERN_INFO "Allocating %lu bytes at %p (%lx physical)"
			" for adsp pmem\n", size, addr, __pa(addr));
	}

	size = MSM_PMEM_GPU1_SIZE;
	if(!resources_hw3d[2].start)
	{
		addr = alloc_bootmem_aligned(size, 0x100000);
		resources_hw3d[2].start = __pa(addr);
		resources_hw3d[2].end = resources_hw3d[2].start + size -1;
		printk(KERN_INFO "Allocating %lu bytes at %p (%lx physical)"
			" for gpu1 pmem\n", size, addr, __pa(addr));
	}

	if(!msm_fb_resources[0].start)
	{
		size = MSM_FB_SIZE;
		addr = alloc_bootmem(size);
		msm_fb_resources[0].start = __pa(addr);
		msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
		printk(KERN_INFO "Allocating %lu bytes at %p (%lx physical)"
			" for fb\n", size, addr, __pa(addr));
	}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
    if (!ramconsole_resource[0].start)
    {
        size = MSM_RAMCONSOLE_SIZE;
        addr = alloc_bootmem(size);
        ramconsole_resource[0].start = __pa(addr);
        ramconsole_resource[0].end = ramconsole_resource[0].start + size - 1;
        printk(KERN_INFO "allocating %lu bytes at %p (%lx physical) for ram console\n",
            size, addr, __pa(addr));
    }
#endif

}

/*
    Entry points needed by machine descriptors:
        .fixup             - memory configuration
        .map_io            - map peripherals
        .init_irq        - machine-type interrupt setup
        .init_machine    - board-specific config
 */

static int mfg_mode;

int hwrev_readproc(char* page, char** start, off_t offset, int count, int *eof, void* data)
{
	int len;

	for(len = 0; len < count && len < 10; len++)
	{
		page[len] = vendor1->fti[offset+len];
	}
	return len;
}

static void __init mot_init(void)
{
    static char   keypad_driver_name[64]; 
	mot_battery_info batt_info;

	if (socinfo_init() < 0)
		BUG();
    /* All 7x01 2.0 based boards are expected to have RAM chips capable
     * of 160 MHz. */
	if (cpu_is_msm7x01() &&
		 SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 2)
			mot_clock_data.max_axi_khz = 160000;

#if defined(CONFIG_MSM_SERIAL_DEBUGGER)
    msm_serial_debug_init(MSM_UART3_PHYS, INT_UART3,
                  &msm_device_uart3.dev, 1);
#endif

	vendor1 = smem_alloc(SMEM_ID_VENDOR1, sizeof(smem_mot_vendor1_type));
	secure_hw = vendor1 ? vendor1->security_on : 1;

	msm_hsusb_pdata.soc_version = socinfo_get_version();
	msm_acpu_clock_init(&mot_clock_data);
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;

	/* Get vreg & reset for the display */
	vreg_slide = vreg_get(0, "gp2");
	if(IS_ERR(vreg_slide)) {
		printk(KERN_ERR "%s: vreg_get failed (%ld)\n",
				__func__, PTR_ERR(vreg_slide));
		return ;
	}
	gpio_request(LCD_RST_N_SIGNAL, "lcd_rst_n");

	msm_device_i2c_init();

	/* Register common i2c devices */
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

    /* Adjust the GPIOs, register machine-specific I2C addresses and USB strings */
    if (machine_is_morrison())
    {
        morrison_setup();
    } 
    else if (machine_is_motus())
    {
        motus_setup();
    } 
    else if (machine_is_zeppelin())
    {
        zeppelin_setup();
    }
    else
    {
        printk(KERN_INFO "%s: could not determine machine type (%d)\n", __func__, __machine_arch_type);
    }

    if( ( strcmp( keypad,"QWERTY")!= 0)  && ( strcmp( keypad,"qwerty") != 0) )
    { /* is not qwerty keypad , add name extension from cmdline */
        sprintf( keypad_driver_name,"%s_%s",mot_adp5588_device.name,keypad);
    	mot_adp5588_device.name = keypad_driver_name;
    }

	create_proc_read_entry("mot_hw", 0, NULL, hwrev_readproc, NULL);

	platform_add_devices(devices, ARRAY_SIZE(devices));

	msm_fb_add_devices();

    config_camera_off_gpios();
	sdcc_gpio_init();
    /* Keep these next two init functions in this order, to make sure that
     * mmc0 is transflash card, and mmc1 is wlan.
     */
	mot_7x01_init_mmc();
	mot_7x01_init_wlan();
#ifdef CONFIG_USB_FUNCTION
	hsusb_gpio_init();
#endif
	bt_power_init();

	platform_device_register(&msm_camera_sensor_mot_mt9p012);

	msm_pm_set_platform_data(msm_pm_data);

	get_mot_battery_info(&batt_info);
#ifdef CONFIG_USB_FUNCTION
	if (batt_info.factory_cable || mfg_mode) {
        if(machine_is_morrison())
            msm_hsusb_pdata.product_id = 0x2d65;
        else
			msm_hsusb_pdata.product_id = 0x2d69;
	} else {
		if (is_secure_hw())
			msm_hsusb_pdata.product_id = 0x2d67;
		else
			msm_hsusb_pdata.product_id = 0x2d63;
	}
#endif
}

static void __init mot_fixup(struct machine_desc *desc, struct tag *tags,
			char **cmdline, struct meminfo *mi)
{

	 struct tag *t;
    /* 
        Parse tags to tell HW flavor (Px and/or discrete vs mono memory).
        If none are given, assume dual die memory, until monolithic is prevaling HW
        Also adjust for the SMI size. Real cmdline is also in the tags.
    */

	for (t = tags; t->hdr.size; t = tag_next(t)) {
		if (t->hdr.tag == ATAG_CMDLINE) {
			printk(KERN_INFO "%s: cmdline=\"%s\"\n", __func__,
				t->u.cmdline.cmdline);
	}
		if (t->hdr.tag == ATAG_REVISION) {
			mot_hw_rev = t->u.revision.rev;
			printk(KERN_INFO "%s: mot_hw_rev=%x\n",
				__func__, mot_hw_rev);
	}
		if (t->hdr.tag == ATAG_MEM) {
			smi_64m = t->u.mem.start;
			ddr_mono = t->u.mem.size;
			printk(KERN_INFO "%s: smi_64m=%d, ddr_mono=%d\n",
				__func__, smi_64m, ddr_mono);
	}
		if (t->hdr.tag == ATAG_KEYPAD) {

			printk(KERN_INFO "%s: keypad=\"%s\"\n", __func__,
				t->u.keypad.name);
			strncpy(keypad, t->u.keypad.name, 8);
	}

    }

    /*
     * Assume monolithic package 1x256MB, and 64Mb SMI package
     * Linux got all of it. Note, all of this can be blown away
     * by command line mem=XXX, so DO NOT pass it if this code is enabled.
     */

	mi->nr_banks = 1;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].node = PHYS_TO_NID(PHYS_OFFSET);
	mi->bank[0].size = 256 * 1024 * 1024;

	mi->bank[1].start = 0x20000000;
	mi->bank[1].size = 128 * 1024 * 1024;

	msm_fb_resources[0].start = MSM_FB_BASE;
	msm_fb_resources[0].end = msm_fb_resources[0].start + MSM_FB_SIZE - 1;

#ifdef CONFIG_ANDROID_RAM_CONSOLE
	ramconsole_resource[0].start = msm_fb_resources[0].end + 1;
	ramconsole_resource[0].end = ramconsole_resource[0].start +
						MSM_RAMCONSOLE_SIZE - 1;
#endif
//	android_pmem_adsp_pdata.start = 0;
	resources_hw3d[2].start = 0;

    printk("%s: bank[0]=%lx@%lx\n", __func__, mi->bank[0].size, mi->bank[0].start);
    printk("%s: bank[1]=%lx@%lx\n", __func__, mi->bank[1].size, mi->bank[1].start);
#ifdef CONFIG_ANDROID_RAM_CONSOLE
    printk("%s: ramconsole_resource[0]=%x..%x\n", __func__, ramconsole_resource[0].start, ramconsole_resource[0].end + 1);
#endif
    printk("%s: msm_fb_resources[0]=%x..%x\n", __func__, msm_fb_resources[0].start, msm_fb_resources[0].end + 1);
    printk("%s: android_pmem_pdata.start=%lx\n", __func__, android_pmem_pdata.start);
    printk("%s: android_pmem_adsp_pdata.start=%lx\n", __func__, android_pmem_adsp_pdata.start);
    printk("%s: resources_hw3d[2].start=%lx\n", __func__, resources_hw3d[2].start);
}

static void __init mot_map_io(void)
{
	msm_shared_ram_phys = 0x01F00000;
	msm_map_common_io();
	msm_clock_init(msm_clocks_7x01a, msm_num_clocks_7x01a);
	msm_mot_allocate_memory_regions();
}

static void __init mot_init_irq(void)
{
	msm_init_irq();
}

extern struct sys_timer msm_timer;

/*
    Exported functions
 */
void bcm_wlan_power_off(unsigned power_mode)
{
     struct device *dv = NULL;

     printk(KERN_INFO "%s: power_mode %d\n", __FUNCTION__, power_mode);

	switch (power_mode) {
	case 1:
		/* Unload driver */
		msm_sdcc2_setup_power(dv, 0);
		wlan_status_notify(0, wlan_host_ptr);
		msleep_interruptible(100);  /* do we really need this much? */
		break;
	case 2:
		/* Stop driver */
		msm_sdcc2_setup_power(dv, 0);
		break;
	default:
		printk(KERN_INFO "%s: ERROR unsupported power_mode %d\n", __FUNCTION__, power_mode);
		break;
     }
}
EXPORT_SYMBOL(bcm_wlan_power_off);

void bcm_wlan_power_on(unsigned power_mode)
{
     struct device *dv = NULL;

     printk(KERN_INFO"%s: power_mode %d\n", __FUNCTION__, power_mode);

	switch (power_mode) {
	case 1:
		/* Load driver */
		msm_sdcc2_setup_power(dv, 1);
		wlan_status_notify(1, wlan_host_ptr);
		msleep_interruptible(100);
		break;
	case 2:
		/* Start driver */
		msm_sdcc2_setup_power(dv, 1);
		break;
	default:
		printk(KERN_INFO"%s: ERROR unsupported power_mode %d\n", __FUNCTION__, power_mode);
		break;
	}
}
EXPORT_SYMBOL(bcm_wlan_power_on);
/* END  WLAN POWER MGMT */


void msm_blink_charging_led (uint16_t onoff, uint16_t level,
    uint16_t on_ms, uint16_t off_ms)
{
    uint32_t param1, param2;
    int ret;

    param1 = (level << 16) | onoff;
    param2 = (on_ms << 16) | off_ms;
    ret = msm_proc_comm (PCOM_CUSTOMER_CMD1, &param1, &param2);
    if (ret) {
        printk (KERN_ERR "%s: msm_proc_comm error: %d\n",
            __FUNCTION__, ret);
    }
}
EXPORT_SYMBOL(msm_blink_charging_led);

void msm_config_gp_mn (uint32_t freq, uint32_t duty)
{
    uint32_t param1;
    uint32_t N = 480, M = freq/10;
    int ret;

    pr_debug ("%s: N = %d, M = %d, duty = %d\n",
        __FUNCTION__, N, M, duty);
    param1 = (M << 16) | N;
    ret = msm_proc_comm (PCOM_CUSTOMER_CMD2, &param1, &duty);
    if (ret) {
        printk (KERN_ERR "%s: msm_proc_comm error: %d\n",
            __FUNCTION__, ret);
    }
}
EXPORT_SYMBOL(msm_config_gp_mn);

int is_secure_hw(void)
{
	return secure_hw;
}
EXPORT_SYMBOL(is_secure_hw);

char *board_model_name (void)
{
    return board_model;
}
EXPORT_SYMBOL(board_model_name);

int __init board_mfg_mode_init(char *s)
{
	if (!strcmp(s, "factory") || !strcmp(s, "factory2"))
		mfg_mode = 1;
	else
		mfg_mode = 0;

	return 1;
}
__setup("androidboot.mode=", board_mfg_mode_init);

static int __init board_serialno_setup(char *serialno)
{
	char *str = NULL;

	if(strlen(serialno))
		str = serialno;

#ifdef CONFIG_USB_FUNCTION
	msm_hsusb_pdata.serial_number = str;
#endif

	return 0;
}
__setup("androidboot.serialno=", board_serialno_setup);

static int __init board_model_setup(char *model)
{
	char *str = NULL;

	if(strlen(model))
		str = model;
	else
		return -1;

	msm_hsusb_pdata.product_name = str;
#ifdef CONFIG_USB_FUNCTION_MASS_STORAGE
	mass_storage_pdata.product = str;
#endif

	board_model = model;
	model_set = 1;
	printk(KERN_DEBUG "Set model to %s\n", str);
	return 0;
}
__setup("model=", board_model_setup);



static int __init board_i2c_setup(char *speed)
{
	char *str = NULL;

	if (strlen(speed))
		str = speed;
	else
		return 0;

	msm_i2c_pdata.clk_freq = simple_strtol(speed, &str, 10);
	if (msm_i2c_pdata.clk_freq > 0 && msm_i2c_pdata.clk_freq <= 400000) {
		i2c_speed_set = 1;
		printk(KERN_DEBUG "Set i2c speed to %s\n", speed);
	}
	return 0;
}
__setup("i2c.speed=", board_i2c_setup);

static int __init board_keypad_setup(char *keypad_type)
{
	/* static char   keypad_driver_name[64]; */
	unsigned keypad_type_len;

	keypad_type_len = strlen(keypad_type);
	if (keypad_type_len && (keypad_type_len < 8))
		strncpy(keypad, keypad_type, keypad_type_len);
	else
		return 0;

	printk(KERN_DEBUG "Set keypad type to %s\n", keypad_type);
	return 0;
}
__setup("keypad=", board_keypad_setup);

/*
    Machine descriptors
 */

MACHINE_START(MORRISON, "Morrison")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io = MSM_DEBUG_UART_PHYS,
	.io_pg_offst = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params = 0x10000100,
	.fixup = mot_fixup,
	.map_io = mot_map_io,
	.init_irq = mot_init_irq,
	.init_machine = mot_init,
	.timer = &msm_timer,
MACHINE_END

MACHINE_START(MOTUS, "Motus")
#ifdef CONFIG_MSM_DEBUG_UART
    .phys_io        = MSM_DEBUG_UART_PHYS,
    .io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
    .boot_params    = 0x10000100,
    .fixup            = mot_fixup,
    .map_io            = mot_map_io,
    .init_irq        = mot_init_irq,
    .init_machine    = mot_init,
    .timer            = &msm_timer,
MACHINE_END

MACHINE_START(ZEPPELIN, "Zeppelin")
#ifdef CONFIG_MSM_DEBUG_UART
    .phys_io        = MSM_DEBUG_UART_PHYS,
    .io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
    .boot_params    = 0x10000100,
    .fixup            = mot_fixup,
    .map_io            = mot_map_io,
    .init_irq        = mot_init_irq,
    .init_machine    = mot_init,
    .timer            = &msm_timer,
MACHINE_END
