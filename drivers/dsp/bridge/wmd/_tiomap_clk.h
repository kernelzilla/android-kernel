/*
 * linux/drivers/dsp/bridge/wmd/linux/omap/common/_tiomap_clk.h
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
 *  ======== _tiomap_clk.h ========
 *  Description:
 *      Definitions and types for the CLOCK and RESET modules. This
 *  includes enabling/disabling peripheral clocks,reseting and releasing
 *  the DSP.
 *! Revision History
 *! ================
 *! 08-Oct-2002 rr:  Created.
 */

#ifndef _TIOMAP_CLK_
#define _TIOMAP_CLK_

#include "_tiomap.h"

/* ARM CLK Module */
#define ARM_CLKM_START                  0xfffece00
#define ARM_CLKM_LENGTH                 0xff
#define ARM_SYSST_OFFSET                0x18
#define IDLE_DSP_MASK                   0x40

/* DSP CLK Module */
#define DSP_CLKM2_BASE                  0xe1008000
#define DSP_CLKM2_LENGTH                0xff
#define DSP_IDLECT2_OFFSET              0x8	/* Clock domain control */

/* CLK related defines */
/*  ARM_CKCTL */
#define ARM_CKCTL_OFFEST                0x0

#define EN_DSPCK_POS                    13
#define EN_DSPCK_NUMB                   1

/*  ARM_RSTCT1 */
#define ARM_RSTCT1_OFFSET               0x10

#define DSP_RST_POS                     2
#define DSP_RST_NUMB                    1
#define DSP_EN_POS                      1
#define DSP_EN_NUMB                     1

/*  ARM_IDLECT2 */
#define ARM_IDLECT2_OFFSET              0x8

#define EN_APICK_POS                    6
#define EN_APICK_NUMB                   1

/* Function prototypes */
/*
 *  ======== dspPeripheralClockDisable ========
 *  Disables the clock for the DSP external periperal clock (DSPPER_CK)
 *  Register 0xe100:8000 offset 0x8 bit position 1 is set to zero.
 */
extern void dspPeripheralClockDisable(struct WMD_DEV_CONTEXT *pDevContext);

/*
 *  ======== dspPeripheralClockEnable ========
 * Enables the clock for DSP external refernce clock, periperal clock, USART clk
 *  GPIO clk and GPIO peripheral clock.
 *  Register 0xe100:8000 offset 0x8 bit position 0,1,2,3,4 and 5 are set to 1.
 */
extern void dspPeripheralClockEnable(struct WMD_DEV_CONTEXT *pDevContext);

/*
 *  ======== releaseDSP ========
 *  Releases the DSP from reset. Th DSP starts running.from whatever
 *  boot mode is set to. If it is set to internal boot mode, the DSP
 *  starts executing from location 0x10000 byte address.
 *  0xfffe:ce00 bit postion 1 is set to one.
 */
extern DSP_STATUS releaseDSP(struct WMD_DEV_CONTEXT *pDevContext,
			     BOOL fCheckForSAM);

/*
 *  ======== releaseDspInterFace ========
 *  Enables the pritoiry registers, EMIF configuration registers and
 *  the API control logic in the DSP subsystem so that they can be programmed.
 *  0xfffe:ce00 bit postion 2 is set to one.
 */
extern void releaseDSPInterface(struct WMD_DEV_CONTEXT *pDevContext);

/*
 *  ======== resetDSP ========
 *  Resets the DSP. This stops the DSP from running.
 *  0xfffe:ce00 bit postion 1 is set to zero
 */
extern void resetDSP(struct WMD_DEV_CONTEXT *pDevContext);

/*
 *  ======== resetDSPInterface ========
 *  Resets the pritoiry registers, EMIF configuration registers and
 *  the API control logic in the DSP subsystem. This is required to configure
 *  the API and MMU.
 *  0xfffe:ce00 bit postion 2 is set to zero.
 */
extern void resetDSPInterface(struct WMD_DEV_CONTEXT *pDevContext);

 /*
  *  ======== setTCEndianism =========
  *  Sets the endianism register.
  */
extern void setTCEndianism(struct WMD_DEV_CONTEXT *pDevContext, u32 value);

#endif				/* _TIOMAP_CLK_ */

