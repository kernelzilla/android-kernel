/* linux/arch/arm/mach-msm/panel-mot.c
** Author: Brian Swetland <swetland@google.com>
*/

#include <linux/bootmem.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/clk.h>
#include <linux/err.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <mach/vreg.h>

#include <asm/mach-types.h>

#include <mach/msm_fb.h>
#include <mach/msm_iomap.h>

#include "board-mot.h"
#include "devices.h"

#define MDDI_PMDH_CLK_RATE 122880000 // 81920000 -- slow down the clock to save some power

struct msm_fb_data fb_data = {
		.xres = 320,
		.yres = 480,
		.output_format = 0
};

static void mot_mddi_power_client(struct msm_mddi_client_data *client_data,
				    int on)
{
	struct vreg *vreg;
	int rc;

	printk("%s: %d\n", __FUNCTION__, on);

	// Turn on VREG_SLIDE for display
	vreg = vreg_get(0, "gp6");
	if ((rc = on ? vreg_enable(vreg) : vreg_disable(vreg)))
		printk(KERN_ERR "%s: vreg(gp6) %s failed (%d)\n", __func__, on ? "enable" : "disable", rc);

	/* 
	 * LCD_RST_N must be high here to be able to talk to client,
	 * otherwise we won't be able to read caps. 
	 */
	// if (on) /* hack to avoid wakeup problems. need to re-init MDDI if we reset it. */
	gpio_set_value(LCD_RST_N_SIGNAL, on);
}


static struct resource resources_mddi0[] = {
	{
		.start	= MSM_PMDH_PHYS,
		.end	= MSM_PMDH_PHYS + MSM_PMDH_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_MDDI_PRI,
		.end	= INT_MDDI_PRI,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource resources_mddi1[] = {
	{
		.start	= MSM_EMDH_PHYS,
		.end	= MSM_EMDH_PHYS + MSM_EMDH_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_MDDI_EXT,
		.end	= INT_MDDI_EXT,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_mddi0 = {
	.name = "msm_mddi",
	.id = 0,
	.num_resources = ARRAY_SIZE(resources_mddi0),
	.resource = resources_mddi0,
	.dev            = {
		.coherent_dma_mask      = 0xffffffff,
	},
};

struct platform_device msm_device_mddi1 = {
	.name = "msm_mddi",
	.id = 1,
	.num_resources = ARRAY_SIZE(resources_mddi1),
	.resource = resources_mddi1,
	.dev            = {
		.coherent_dma_mask      = 0xffffffff,
	}
};

static struct resource resources_mdp[] = {
	{
		.start	= MSM_MDP_PHYS,
		.end	= MSM_MDP_PHYS + MSM_MDP_SIZE - 1,
		.name	= "mdp",
		.flags	= IORESOURCE_MEM
	},
	{
		.start	= INT_MDP,
		.end	= INT_MDP,
		.flags	= IORESOURCE_IRQ,
	},
};

struct platform_device msm_device_mdp = {
	.name = "msm_mdp",
	.id = 0,
	.num_resources = ARRAY_SIZE(resources_mdp),
	.resource = resources_mdp,
};

static struct resource resources_msm_fb[] = {
	{
		.start = MSM_FB_BASE,
		.end = MSM_FB_BASE + MSM_FB_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

struct msm_mddi_platform_data mddi_pdata = {
	.clk_rate = MDDI_PMDH_CLK_RATE,
	.power_client = mot_mddi_power_client,
	.fb_resource = resources_msm_fb,
	.num_clients = 1,
	.client_platform_data = {
		{
			.product_id = (0 << 16 | 0),
			.name = "mddi_c_0000_0000", 	// wildcard
			.id = 0, 			// MSM_MDDI_PMDH_INTERFACE
			.client_data = &fb_data,
			.clk_rate = 81920000,
		},
	},
};

int __init mot_init_panel(void)
{
	int rc;
	unsigned	size, addr;

	size = MSM_FB_SIZE;
	if (resources_msm_fb[0].start == 0)
	{
		addr = alloc_bootmem(size);
		resources_msm_fb[0].start = __pa(addr);
		printk(KERN_INFO "allocating %lu bytes at %p (%lx physical) for fb\n",
		  size, addr, __pa(addr));
		resources_msm_fb[0].end = resources_msm_fb[0].start + size - 1;
	}

	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	msm_device_mddi0.dev.platform_data = &mddi_pdata;
	return platform_device_register(&msm_device_mddi0);
}

device_initcall(mot_init_panel);
