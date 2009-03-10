/*
 * linux/arch/arm/mach-omap2/board-ldp-flash.c
 *
 * Copyright (C) 2008 Texas Instruments Inc.
 *
 * Modified from mach-omap2/board-2430sdp-flash.c
 * Author: Rohit Choraria <rohitkc@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/types.h>
#include <linux/io.h>

#include <asm/mach/flash.h>
#include <mach/board.h>
#include <mach/gpmc.h>
#include <mach/nand.h>

#ifdef CONFIG_MACH_OMAP_ZOOM2
#include <mach/board-zoom2.h>
#else
#include <mach/board-ldp.h>
#endif

#define GPMC_CS0_BASE	0x60
#define GPMC_CS_SIZE	0x30

#ifdef CONFIG_OMAP3_PM
/*
 * Number of frequencies supported by gpmc
 */
#define NO_GPMC_FREQ_SUPPORTED		2
#define SUPPORTED_FREQ1			83
#define SUPPORTED_FREQ2			166

/*
 * TBD: Get optimized NAND setting for 83MHz
 *      Get 133/66MHz timings.
 */

struct gpmc_cs_config pdc_nand_gpmc_setting[] = {
	{0x1800, 0x00030300, 0x00030200, 0x03000400, 0x00040505, 0x030001C0},
	{0x1800, 0x00141400, 0x00141400, 0x0f010f01, 0x010c1414, 0x1f0f0a80}
};

/* ethernet goes crazy if differt times are used */
struct gpmc_cs_config enet_gpmc_setting[] = {
	{0x611200, 0x001F1F01, 0x00080803, 0x1D091D09, 0x041D1F1F, 0x1D0904C4},
	{0x611200, 0x001F1F01, 0x00080803, 0x1D091D09, 0x041D1F1F, 0x1D0904C4}
};

/*
 * Structure containing the gpmc cs values at different frequencies
 * This structure will be populated run time depending on the
 * values read from FPGA registers..On 3430 SDP FPGA is always on CS3
 */
struct gpmc_freq_config freq_config[NO_GPMC_FREQ_SUPPORTED];
#endif

static struct mtd_partition ldp_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader-NAND",
		.offset		= 0,
		.size		= 4 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 4 * (64 * 2048),
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "Boot Env-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x100000 */
		.size		= 2 * (64 * 2048),
	},
	{
		.name		= "Kernel-NAND",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x140000 */
		.size		= 32 * (64 * 2048),
	},
	{
		.name		= "system",
 		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x540000 */
		.size		= 512 * (64 * 2048),    /* 64M */
	},
	{
		.name		= "userdata",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x4540000 */
		.size		= 512 * (64 * 2048),    /* 64M */
 	},
	{
		.name		= "cache",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x8540000 */
		.size		= 512 * (64 * 2048),    /* 64M */
 	},
};

/* NAND chip access: 16 bit */
static struct omap_nand_platform_data ldp_nand_data = {
	.parts		= ldp_nand_partitions,
	.nr_parts	= ARRAY_SIZE(ldp_nand_partitions),
	.nand_setup	= NULL,
	.dma_channel	= -1,		/* disable DMA in OMAP NAND driver */
	.dev_ready	= NULL,
};

static struct resource ldp_nand_resource = {
	.flags		= IORESOURCE_MEM,
};

static struct platform_device ldp_nand_device = {
	.name		= "omap2-nand",
	.id		= 0,
	.dev		= {
	.platform_data	= &ldp_nand_data,
	},
	.num_resources	= 1,
	.resource	= &ldp_nand_resource,
};

/**
 * ldp430_flash_init - Identify devices connected to GPMC and register.
 *
 * @return - void.
 */
void __init ldp_flash_init(void)
{
	u8 nandcs = GPMC_CS_NUM + 1;
	u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;

#ifdef CONFIG_OMAP3_PM
	freq_config[0].freq = SUPPORTED_FREQ1;
	freq_config[1].freq = SUPPORTED_FREQ2;

	/* smc9211 debug ether */
	freq_config[0].gpmc_cfg[LDP_SMC911X_CS] = enet_gpmc_setting[0];
	freq_config[1].gpmc_cfg[LDP_SMC911X_CS] = enet_gpmc_setting[1];
#endif
	/* pop nand part */
	nandcs = LDP3430_NAND_CS;

	ldp_nand_data.cs = nandcs;
	ldp_nand_data.gpmc_cs_baseaddr = (void *)(gpmc_base_add +
					GPMC_CS0_BASE + nandcs * GPMC_CS_SIZE);
	ldp_nand_data.gpmc_baseaddr = (void *) (gpmc_base_add);

	if (platform_device_register(&ldp_nand_device) < 0)
		printk(KERN_ERR "Unable to register NAND device\n");
#ifdef CONFIG_OMAP3_PM
	/*
	 * Setting up gpmc_freq_cfg so tat gpmc module is aware of the
	 * frequencies supported and the various config values for cs
	 */
	gpmc_freq_cfg.total_no_of_freq = NO_GPMC_FREQ_SUPPORTED;
	gpmc_freq_cfg.freq_cfg = freq_config;
#endif
}

