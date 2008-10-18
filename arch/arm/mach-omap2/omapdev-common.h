/*
 * OMAP on-chip devices present on OMAP2/3
 *
 * Copyright (C) 2007-2008 Texas Instruments, Inc.
 * Copyright (C) 2007-2008 Nokia Corporation
 * Paul Walmsley
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#ifndef ARCH_ARM_MACH_OMAP2_OMAPDEV_COMMON_H
#define ARCH_ARM_MACH_OMAP2_OMAPDEV_COMMON_H

#include <mach/omapdev.h>

#include "omapdev242x.h"
#include "omapdev243x.h"
#include "omapdev3xxx.h"

static struct omapdev *omapdevs[] = {

#ifdef CONFIG_ARCH_OMAP2420
	&mpu_242x_omapdev,
	&iva_242x_omapdev,
	&gfx_242x_omapdev,
	&prcm_242x_omapdev,
	&l3_242x_omapdev,
	&l4_core_242x_omapdev,
	&dsp_242x_omapdev,
	&dsp_mmu_242x_omapdev,
	&control_242x_omapdev,
	&tap_242x_omapdev,
	&gpio2_242x_omapdev,
	&gpio3_242x_omapdev,
	&gpio4_242x_omapdev,
	&gptimer12_242x_omapdev,
	&uart3_242x_omapdev,
	&mcbsp2_242x_omapdev,
	&wdtimer4_242x_omapdev,
	&gptimer2_242x_omapdev,
	&gptimer3_242x_omapdev,
	&gptimer4_242x_omapdev,
	&gptimer5_242x_omapdev,
	&gptimer6_242x_omapdev,
	&gptimer7_242x_omapdev,
	&gptimer8_242x_omapdev,
	&gptimer9_242x_omapdev,
	&etb_242x_omapdev,
	&cwt_242x_omapdev,
	&xti_242x_omapdev,
	&dap_242x_omapdev,
	&dsi_242x_omapdev,
	&dsi_pll_242x_omapdev,
	&dss_242x_omapdev,
	&dispc_242x_omapdev,
	&rfbi_242x_omapdev,
	&venc_242x_omapdev,
	&fac_242x_omapdev,
	&cam_242x_omapdev,
	&cam_core_242x_omapdev,
	&cam_dma_242x_omapdev,
	&cam_mmu_242x_omapdev,
	&mpu_intc_242x_omapdev,
	&sms_242x_omapdev,
	&gpmc_242x_omapdev,
	&sdrc_242x_omapdev,
	&ocm_ram_242x_omapdev,
	&ocm_rom_242x_omapdev,
	&ssi_242x_omapdev,
	&ohci_242x_omapdev,
	&otg_242x_omapdev,
	&sdma_242x_omapdev,
	&i2c1_242x_omapdev,
	&i2c2_242x_omapdev,
	&uart1_242x_omapdev,
	&uart2_242x_omapdev,
	&mcbsp1_242x_omapdev,
	&gptimer10_242x_omapdev,
	&gptimer11_242x_omapdev,
	&mailbox_242x_omapdev,
	&mcspi1_242x_omapdev,
	&mcspi2_242x_omapdev,
	&mg_242x_omapdev,
	&hdq_242x_omapdev,
	&mspro_242x_omapdev,
	&wdtimer3_242x_omapdev,
	&vlynq_242x_omapdev,
	&eac_242x_omapdev,
	&mmc_242x_omapdev,
	&gptimer1_242x_omapdev,
	&omap_32ksynct_242x_omapdev,
	&gpio1_242x_omapdev,
	&wdtimer2_242x_omapdev,
	&wdtimer1_242x_omapdev,
	&rng_242x_omapdev,
	&sha1md5_242x_omapdev,
	&des_242x_omapdev,
	&aes_242x_omapdev,
	&pka_242x_omapdev,
#endif

	NULL,
};

#endif
