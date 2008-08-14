/*
 * linux/drivers/dsp/bridge/wmd/linux/omap/common/_tiomap_api.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */


/*
 *  ======== _tiomap_api.h ========
 *  Description:
 *      Definitions, types and function prototypes for the API module.
 *
 *! Revision History
 *! ================
 *! 27-Mar-2003 vp:  Updated for Power Management
 *!		     - Added setAPI function
 *!		     - Added API_DSP_BOOT_IDLE enumeration
 *!		     - Modified setDSPBootMode function to take boot mode option
 *! 19-Feb-2003 vp:  Ported to Linux platform.
 *! 08-Oct-2002 rr:  Created.
 */

#ifndef _TIOMAP_API_
#define _TIOMAP_API_

/* I/O ARM Memory Mapping and ARM Peripheral mapping */
#define ARM_API_START                   0xfffec900
#define ARM_API_LENGTH                  0xff

/* API I/F Registers */
#define API_CTRL_OFFSET                 0x0000	/* 32 bit control register   */
#define API_STATUS_OFFSET               0x0010	/* 16 bit status register(RO) */
#define API_DSP_STATUS_OFFSET           0x0014	/* 16 bit DSP status register */
#define API_DSP_BOOT_OFFSET             0x0018	/* 16 bit Boot configuration
						 * register
						 */
#define API_SIZE                        0x001c
#define API_DSP_API_OFFSET              0x0020	/* 16 bit API Configuration
						 * register
						 */

/* API_DSP_STATUS_REG related */
#define API_DSP_STATUS_HOM_MODE         0x0c00	/* HOM Mode bits - set if HOM */

/* SAM/HOM bit in API_DSP_STATUS register value */
#define SAM_BIT                         0x0400

/*  Define position bit fields */
#define API_HIGH_FREQ_POSBIT            0
#define API_TIMEOUT_EN_POSBIT           1
#define API_API_ERR_EN_POSBIT           3
#define API_ACCESS_FACTOR_POSBIT        4
#define API_TIMEOUT_POSBIT              8
#define API_ENDIANISM_POSBIT            16
#define API_ACCESS_PRIORITY_POSBIT      18

typedef enum {
	API_HIGH_FREQ_LOW = 0,
	API_HIGH_FREQ_HIGH = 1
} API_HighFreq_t;

typedef enum {
	API_TIMEOUT_DIS = 0,
	API_TIMEOUT_EN = 1
} API_TimeoutEn_t;

typedef enum {
	API_API_ERR_DIS = 0,
	API_API_ERR_EN = 1
} API_ApiErrEn_t;

typedef enum {
	API_ACCESS_FACTOR_0 = 0,
	API_ACCESS_FACTOR_1,
	API_ACCESS_FACTOR_2,
	API_ACCESS_FACTOR_3,
	API_ACCESS_FACTOR_4,
	API_ACCESS_FACTOR_5,
	API_ACCESS_FACTOR_6,
	API_ACCESS_FACTOR_7,
	API_ACCESS_FACTOR_8,
	API_ACCESS_FACTOR_9,
	API_ACCESS_FACTOR_10,
	API_ACCESS_FACTOR_11,
	API_ACCESS_FACTOR_12,
	API_ACCESS_FACTOR_13,
	API_ACCESS_FACTOR_14,
	API_ACCESS_FACTOR_15
} API_AccessFactor_t;

typedef enum {
	API_TIMEOUT_MIN = 0,
	API_TIMEOUT_MAX = 255
} API_Timeout_t;

typedef enum {
	API_ENDIANISM_NO_CONVERT = 0,
	API_ENDIANISM_CONVERT_ALL_ACCESS = 2,
	API_ENDIANISM_CONVERT_API_MEM_ONLY = 3
} API_Endianism_t;

typedef enum {
	API_ACCESS_PRIORITY_ARM_DMA_HSAB = 0,
	API_ACCESS_PRIORITY_ARM_HSAB_DMA,
	API_ACCESS_PRIORITY_DMA_ARM_HSAB,
	API_ACCESS_PRIORITY_HSAB_ARM_DMA,
	API_ACCESS_PRIORITY_DMA_HSAB_ARM,
	API_ACCESS_PRIORITY_HSAB_DMA_ARM
} API_AccessPriority_t;

typedef enum {
	API_DSP_BOOT_INTERNAL = 5,
	API_DSP_BOOT_EXTERNAL = 4,
	API_DSP_BOOT_EMIF16 = 3,
	API_DSP_BOOT_IDLE = 2,
	API_DSP_BOOT_PSEUDO_EXT = 1,
	API_DSP_BOOT_MPNMC = 0
} API_DSPBootMode_t;

/* Function prototypes */

/*
 *  ======== setAPIsize ========
 *  Configures the SARAM blocks which can be accessed in the HOM mode
 *  Register 0xfffe:c900 offset 0x1c.
 */
extern void setAPIsize(struct WMD_DEV_CONTEXT *pDevContext, u16 size);

/*
 *  ======== setDspBootModeAPI ========
 *  Sets up the DSP boot mode
 *  Register 0xfffe:c900 offset 18.
 *  Boot mode is set API_DSP_BOOT_INTERNAL; DSP will start executing from
 *  SARAM location 0x10000 byte address.
 */
extern void setDspBootModeAPI(struct WMD_DEV_CONTEXT *pDevContext,
			      API_DSPBootMode_t dsp_boot_mode);

/*
 *  ======== setAPI ========
 *  Configures the API interface.
 *  Register 0xfffe:c900 offset 0x0.
 *      -   Set the API access priority to ARM-DMA-HSAB
 *      -   Sets the no byte swap
 *      -   Time out is set to MAX
 *      -   Access factor is 4
 *      -   Enable the time out
 *      -   Set it to high frequency mode
 */
extern void setAPI(struct WMD_DEV_CONTEXT *pDevContext,
		   API_HighFreq_t high_freq,
		   API_TimeoutEn_t timeout_en,
		   API_ApiErrEn_t api_err_en,
		   API_AccessFactor_t access_factor,
		   API_Timeout_t timeout,
		   API_Endianism_t endianism,
		   API_AccessPriority_t access_priority);

/*
 *  ======== setupAPI ========
 *  Configures the API interface.
 *  Register 0xfffe:c900 offset 0x0.
 *      -   Set the API access priority to ARM-DMA-HSAB
 *      -   Sets the no byte swap
 *      -   Time out is set to MAX
 *      -   Access factor is 4
 *      -   Enable the time out
 *      -   Set it to high frequency mode
 */
extern void setupAPI(struct WMD_DEV_CONTEXT *pDevContext);

#endif				/* _TIOMAP_API_ */
