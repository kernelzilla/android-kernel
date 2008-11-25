/*
 * tiomap_sm.c
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
 *  ======== tiomap_sm.c ========
 *  Description:
 *      Implements lower edge channel class library functions.
 *
 *! Revision History:
 *! ================
 *! 05-Jan-2004 vp    Updated for the new HW library for 24xx platform.
 *! 12-Feb-2004 hp    use 'CFG_GetHostResources' to fetch virtual addresses of
 *!           PRCM, dMMU components.
 *! 08-Oct-2002 rr    Renamed to tiomap1510_sm.c
 *! 15-Feb-2002 ag    Remove #include <pkfuncs.h> & util.h.
 *! 07-Jan-2001 ag    Set DSP MBX val (to DSP) contained in DEV context.
 *! 05-Nov-2001 kc:   Modified CHNLSM_ISR to read mailbox1 interrupt values
 *! 26-Sep-2001 rr:   InterruptDSP does not spin forever for retail build.
 *! 29-Aug-2001 rr:   Cleaned up the non referenced variables.
 *! 26-Jul-2001 jeh   Enable interrupt to DSP.
 *! 28-Jun-2001 ag    Disable INTR gen to DSP in CHNLSM_InterruptDSP().
 *! 26-Jun-2001 ag    Added INTR support.
 *! 17-May-2000 ag    Initial. No INTR support.
 */

/*  ----------------------------------- Host OS */
#include <dspbridge/host_os.h>
/*  ----------------------------------- DSP/BIOS Bridge */
#include <dspbridge/std.h>
#include <dspbridge/dbdefs.h>
#include <dspbridge/errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <dspbridge/dbg.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <dspbridge/cfg.h>
#include <dspbridge/drv.h>
#include <dspbridge/util.h>

/*  ----------------------------------- Mini Driver */
#include <dspbridge/wmd.h>

/*  ----------------------------------- Platform Manager */
#include <dspbridge/wcd.h>
#include <dspbridge/dev.h>

/* ------------------------------------ Hardware Abstraction Layer */
#include <hw_defs.h>
#include <hw_mbox.h>

/*  ----------------------------------- This */
#include "_tiomap.h"
#include <dspbridge/chnl_sm.h>
#include "_tiomap_pwr.h"

#if (defined CONFIG_PM) && (defined CONFIG_BRIDGE_DVFS)
extern struct platform_device omap_dspbridge_dev;
#endif

/*  ----------------------------------- Defines, Data Structures, Typedefs */
#ifndef DEBUG
#define TIHELEN_INT_TIMEOUT     1
#define LOOP_COUNT              1000000
#endif

extern struct MAILBOX_CONTEXT mboxsetting;
extern DSP_STATUS DSP_PeripheralClocks_Enable(struct WMD_DEV_CONTEXT
					     *pDevContext, IN void *pArgs);
/*
 *  ======== CHNLSM_EnableInterrupt ========
 *      Enables interrupts from DSP.
 */
DSP_STATUS CHNLSM_EnableInterrupt(struct WMD_DEV_CONTEXT *hDevContext)
{
	DSP_STATUS status = DSP_SOK;
	HW_STATUS hwStatus;
	struct WMD_DEV_CONTEXT *pDevContext = hDevContext;
	u32 numMbxMsg;
	u32 mbxValue;
	struct CFG_HOSTRES resources;
	u32 devType;
	struct IO_MGR *hIOMgr;

	DBG_Trace(DBG_ENTER, "CHNLSM_EnableInterrupt(0x%x)\n", pDevContext);

	/* Read the messages in the mailbox until the message queue is empty */

	status = CFG_GetHostResources(
			(struct CFG_DEVNODE *)DRV_GetFirstDevExtension(),
			&resources);
	status = DEV_GetDevType(pDevContext->hDevObject, &devType);
	status = DEV_GetIOMgr(pDevContext->hDevObject, &hIOMgr);
	if (devType == DSP_UNIT) {
		hwStatus = HW_MBOX_NumMsgGet(resources.dwMboxBase,
					       MBOX_DSP2ARM, &numMbxMsg);
		while (numMbxMsg != 0) {
			hwStatus = HW_MBOX_MsgRead(resources.dwMboxBase,
						     MBOX_DSP2ARM,
						     &mbxValue);
			numMbxMsg--;
		}
		/* clear the DSP mailbox as well...*/
		hwStatus = HW_MBOX_NumMsgGet(resources.dwMboxBase,
					       MBOX_ARM2DSP, &numMbxMsg);
		while (numMbxMsg != 0) {
			hwStatus = HW_MBOX_MsgRead(resources.dwMboxBase,
						    MBOX_ARM2DSP, &mbxValue);
			numMbxMsg--;
			UTIL_Wait(10);

			HW_MBOX_EventAck(resources.dwMboxBase, MBOX_ARM2DSP,
					  HW_MBOX_U1_DSP1,
					  HW_MBOX_INT_NEW_MSG);
		}
		/* Enable the new message events on this IRQ line */
		hwStatus = HW_MBOX_EventEnable(resources.dwMboxBase,
						 MBOX_DSP2ARM,
						 MBOX_ARM,
						 HW_MBOX_INT_NEW_MSG);
	}

	return status;
}

/*
 *  ======== CHNLSM_DisableInterrupt ========
 *      Disables interrupts from DSP.
 */
DSP_STATUS CHNLSM_DisableInterrupt(struct WMD_DEV_CONTEXT *hDevContext)
{
	DSP_STATUS status = DSP_SOK;
	HW_STATUS hwStatus;
	struct CFG_HOSTRES resources;

	DBG_Trace(DBG_ENTER, "CHNLSM_DisableInterrupt(0x%x)\n", hDevContext);

	status = CFG_GetHostResources(
			(struct CFG_DEVNODE *)DRV_GetFirstDevExtension(),
			&resources);
	hwStatus = HW_MBOX_EventDisable(resources.dwMboxBase, MBOX_DSP2ARM,
					  MBOX_ARM, HW_MBOX_INT_NEW_MSG);
	return status;
}

/*
 *  ======== CHNLSM_InterruptDSP ========
 *      Send an interrupt to the DSP processor(s).
 */
DSP_STATUS CHNLSM_InterruptDSP(struct WMD_DEV_CONTEXT *hDevContext)
{
	DSP_STATUS status = DSP_SOK;
	struct WMD_DEV_CONTEXT *pDevContext = hDevContext;

#if (defined CONFIG_PM) && (defined CONFIG_BRIDGE_DVFS)
	struct dspbridge_platform_data *pdata =
				omap_dspbridge_dev.dev.platform_data;
	u32 opplevel = 0;
#endif
	HW_STATUS hwStatus;
	u32 mbxFull;
	struct CFG_HOSTRES resources;
	u16 cnt = 10;
	u32 temp;
	/* We are waiting indefinitely here. This needs to be fixed in the
	 * second phase */
	status = CFG_GetHostResources(
			(struct CFG_DEVNODE *)DRV_GetFirstDevExtension(),
			&resources);
#if (defined CONFIG_PM) && (defined CONFIG_BRIDGE_DVFS)
		if (pdata->dsp_get_opp)
			opplevel = (*pdata->dsp_get_opp)();

		/* If OPP is at minimum level, increase it before waking up
		* the DSP */
		if (opplevel == 1) {
			if (pdata->dsp_set_min_opp) {
				(*pdata->dsp_set_min_opp)(opplevel+1);
				DBG_Trace(DBG_LEVEL7, "CHNLSM_InterruptDSP: "
					"Setting the vdd1 opp level to %d "
					"before waking DSP \n",
					(opplevel + 1));
			}
		}

#endif

	if  (pDevContext->dwBrdState == BRD_DSP_HIBERNATION ||
	    pDevContext->dwBrdState == BRD_HIBERNATION) {

		temp = (u32) *((REG_UWORD32 *) ((u32)
		       (resources.dwDmmuBase) + 0x10));

		/* Restore mailbox settings */
		status = HW_MBOX_restoreSettings(resources.dwMboxBase);

                /* Restart the peripheral clocks that were disabled only
                 * in DSP initiated Hibernation case.*/
		if (pDevContext->dwBrdState == BRD_DSP_HIBERNATION)
			DSP_PeripheralClocks_Enable(hDevContext, NULL);

		pDevContext->dwBrdState = BRD_RUNNING;
	}
	while (--cnt) {
		hwStatus = HW_MBOX_IsFull(resources.dwMboxBase,
					   MBOX_ARM2DSP, &mbxFull);
		if (mbxFull)
			UTIL_Wait(1000);	/* wait for 1 ms)      */
		else
			break;
	}
	if (!cnt) {
		DBG_Trace(DBG_LEVEL7, "Timed out waiting for DSP mailbox \n");
		status = WMD_E_TIMEOUT;
		return status;
	}
	DBG_Trace(DBG_LEVEL3, "writing %x to Mailbox\n",
		 pDevContext->wIntrVal2Dsp);

	hwStatus = HW_MBOX_MsgWrite(resources.dwMboxBase, MBOX_ARM2DSP,
				     pDevContext->wIntrVal2Dsp);
	/* set the Mailbox interrupt to default value */
	pDevContext->wIntrVal2Dsp = MBX_PCPY_CLASS;
	return status;
}

/*
 *  ======== CHNLSM_InterruptDSP2 ========
 *      Set MBX value & send an interrupt to the DSP processor(s).
 */
DSP_STATUS CHNLSM_InterruptDSP2(struct WMD_DEV_CONTEXT *hDevContext,
				u16 wMbVal)
{
	struct WMD_DEV_CONTEXT *pDevContext = hDevContext;

	pDevContext->wIntrVal2Dsp = wMbVal;

	return CHNLSM_InterruptDSP(hDevContext);
}

/*
 *  ======== CHNLSM_DPC ========
 */
void CHNLSM_DPC(struct WMD_DEV_CONTEXT *hDevContext)
{
	DBG_Trace(DBG_ENTER, "CHNLSM_DPC(0x%x)\n", hDevContext);
}

/*
 *  ======== CHNLSM_ISR ========
 */
bool CHNLSM_ISR(struct WMD_DEV_CONTEXT *hDevContext, OUT bool *pfSchedDPC,
		OUT u16 *pwIntrVal)
{
	bool fMyInterrupt = true;	/*
					 * We own the mbx and
					 * we're not sharing it
					 */
	struct CFG_HOSTRES resources;
	u32 numMbxMsg;
	u32 mbxValue;

	DBG_Trace(DBG_ENTER, "CHNLSM_ISR(0x%x)\n", hDevContext);

	CFG_GetHostResources(
		(struct CFG_DEVNODE *)DRV_GetFirstDevExtension(), &resources);

	HW_MBOX_NumMsgGet(resources.dwMboxBase, MBOX_DSP2ARM, &numMbxMsg);

	if (numMbxMsg > 0) {
		HW_MBOX_MsgRead(resources.dwMboxBase, MBOX_DSP2ARM, &mbxValue);

		HW_MBOX_EventAck(resources.dwMboxBase, MBOX_DSP2ARM,
				 HW_MBOX_U0_ARM, HW_MBOX_INT_NEW_MSG);

		DBG_Trace(DBG_LEVEL3, "Read %x from Mailbox\n", mbxValue);
		*pwIntrVal = (u16) mbxValue;
	}
	/* Set *pfSchedDPC to true; */
	*pfSchedDPC = true;
	return fMyInterrupt;
}

