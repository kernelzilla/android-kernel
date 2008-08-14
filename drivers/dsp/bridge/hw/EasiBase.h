/*
 * linux/drivers/dsp/bridge/hw/omap3/inc/EasiBase.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef __EASIBASE_H
#define __EASIBASE_H

/* ----------------------------------------------------------------------------
* DEFINE:        ****_BASE_ID
*
* DESCRIPTION:  These are registser BASE IDs that will be used to identify
*               errors when doing the EASI-Checker register tests
*
* NOTE:         The values of these defines will be defined at a later stage.
* 		TBD
*
* -----------------------------------------------------------------------------
*/

#define OSPL1_BASE_ID 0
#define D3D1_BASE_ID 0
#define MBSP1_BASE_ID 0
#define MBSP2_BASE_ID 0
#define MSDI1_BASE_ID 0
#define RNG1_BASE_ID 0
#define SHAM1_BASE_ID 0
#define RFBI1_BASE_ID 0
#define DISC1_BASE_ID 0
#define DSS1_BASE_ID 0
#define MLB1_BASE_ID 0
#define IPI1_BASE_ID 0
#define PDMA1_BASE_ID 0
#define PRCM_BASE_ID 0
#define SDMA1_BASE_ID 0
#define SDRC1_BASE_ID 0
#define ST1_BASE_ID 0
#define SMS1_BASE_ID 0
#define WDT1_BASE_ID 0
#define WDT2_BASE_ID 0
#define WDT3_BASE_ID 0
#define WDT4_BASE_ID 0
#define INTC1_BASE_ID 0
#define INTC2_BASE_ID 0
#define MMU1_BASE_ID 0
#define GPMC1_BASE_ID 0
#define GPT1_BASE_ID 0
#define GPT2_BASE_ID 0
#define GPT3_BASE_ID 0
#define GPT4_BASE_ID 0
#define GPT5_BASE_ID 0
#define GPT6_BASE_ID 0
#define GPT7_BASE_ID 0
#define GPT8_BASE_ID 0
#define GPT9_BASE_ID 0
#define GPT10_BASE_ID 0
#define GPT11_BASE_ID 0
#define GPT12_BASE_ID 0
#define WTR1_BASE_ID 0
#define WTR2_BASE_ID 0
#define WTR3_BASE_ID 0
#define WTR4_BASE_ID 0
#define I2C1_BASE_ID 0
#define I2C2_BASE_ID 0
#define T32K1_BASE_ID 0
#define PRCM1_BASE_ID 0

#define AES1_BASE_ID 0
#define C2CF1_BASE_ID 0
#define DSPF1_BASE_ID 0
#define FAC1_BASE_ID 0
#define GPMF1_BASE_ID 0
#define GPIO1_BASE_ID 0
#define GPIO2_BASE_ID 0
#define GPIO3_BASE_ID 0
#define GPIO4_BASE_ID 0
#define HDQW1_BASE_ID 0
#define PKA1_BASE_ID 0

#define IM1_BASE_ID 0
#define IM2_BASE_ID 0
#define IM3_BASE_ID 0
#define IM4_BASE_ID 0
#define IM5_BASE_ID 0
#define IM6_BASE_ID 0
#define IM7_BASE_ID 0
#define IM8_BASE_ID 0
#define IMA1_BASE_ID 0
#define IMTM1_BASE_ID 0
#define IVAF1_BASE_ID 0
#define LRCR1_BASE_ID 0
#define LRCR2_BASE_ID 0
#define LRCS1_BASE_ID 0
#define LRCS2_BASE_ID 0
#define RAMF1_BASE_ID 0
#define ROMF1_BASE_ID 0
#define TM1_BASE_ID 0
#define TML1_BASE_ID 0
#define TML2_BASE_ID 0
#define TML3_BASE_ID 0
#define TML4_BASE_ID 0
#define TML5_BASE_ID 0
#define TML6_BASE_ID 0



/* ----------------------------------------------------------------------------
* DEFINE: ***_BASE_EASIL1
*
* DESCRIPTION:  These are registser BASE EASIl1 numbers that can be used to
*               identify what EASI C functions have been called.
*
* NOTE:         The values of these defines will be defined at a later stage.
* 		TBD
*
* -----------------------------------------------------------------------------
*/

#define OSPL1_BASE_EASIL1 0
#define D3D_BASE_EASIL1 0
#define MBSP_BASE_EASIL1 0
#define MSDI_BASE_EASIL1 0
#define RNG_BASE_EASIL1 0
#define SHAM_BASE_EASIL1 0
#define RFBI_BASE_EASIL1 0
#define DISC_BASE_EASIL1 0
#define DSS_BASE_EASIL1 0
#define MLB_BASE_EASIL1 0
#define IPI_BASE_EASIL1 0
#define PDMA_BASE_EASIL1 0
#define SDMA_BASE_EASIL1 0
#define SDRC_BASE_EASIL1 0
#define ST_BASE_EASIL1 0
#define SMS_BASE_EASIL1 0
#define WDT1_BASE_EASIL1 0
#define INTC1_BASE_EASIL1 0
#define INTC2_BASE_EASIL1 0
#define MMU1_BASE_EASIL1 0
#define GPMC_BASE_EASIL1 0
#define GPT_BASE_EASIL1 0
#define WTR_BASE_EASIL1 0
#define MBSP2_BASE_EASIL1 0
#define I2C1_BASE_EASIL1 0
#define I2C2_BASE_EASIL1 0
#define T32K1_BASE_EASIL1 0
#define PRCM1_BASE_EASIL1 0

#define AES1_BASE_EASIL1 0
#define C2CF1_BASE_EASIL1 0
#define DSPF1_BASE_EASIL1 0
#define FAC1_BASE_EASIL1 0
#define GPMF1_BASE_EASIL1 0
#define GPIO1_BASE_EASIL1 0
#define HDQW1_BASE_EASIL1 0
#define PKA1_BASE_EASIL1 0

#define IMA_BASE_EASIL1  0
#define IM_BASE_EASIL1  0
#define IMTM_BASE_EASIL1  0
#define IVAF_BASE_EASIL1  0
#define LRCR_BASE_EASIL1  0
#define LRCS_BASE_EASIL1  0
#define RAMF_BASE_EASIL1  0
#define ROMF_BASE_EASIL1  0
#define TML_BASE_EASIL1  0
#define TM_BASE_EASIL1  0

#endif	/* __EASIBASE_H */
