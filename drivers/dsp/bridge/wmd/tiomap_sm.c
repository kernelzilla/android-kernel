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

#include <dspbridge/dbdefs.h>
#include <dspbridge/errbase.h>

#include <dspbridge/cfg.h>
#include <dspbridge/drv.h>
#include <dspbridge/dev.h>

#include <dspbridge/dbg.h>

#include "_tiomap.h"
#include "_tiomap_pwr.h"

#ifdef CONFIG_BRIDGE_DVFS
extern struct platform_device omap_dspbridge_dev;
#endif

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

DSP_STATUS CHNLSM_InterruptDSP(struct WMD_DEV_CONTEXT *hDevContext)
{
	DSP_STATUS status = DSP_SOK;
	struct WMD_DEV_CONTEXT *pDevContext = hDevContext;

#ifdef CONFIG_BRIDGE_DVFS
	struct dspbridge_platform_data *pdata =
				omap_dspbridge_dev.dev.platform_data;
	u32 opplevel = 0;
#endif
	HW_STATUS hwStatus;
	u32 mbxFull;
	struct CFG_HOSTRES resources;
	u16 cnt = 10;
	u32 temp;

	status = CFG_GetHostResources(
			(struct CFG_DEVNODE *)DRV_GetFirstDevExtension(),
			&resources);
       if (DSP_FAILED(status))
               return DSP_EFAIL;
#ifdef CONFIG_BRIDGE_DVFS
       if (pDevContext->dwBrdState == BRD_DSP_HIBERNATION ||
           pDevContext->dwBrdState == BRD_HIBERNATION) {
		if (pdata->dsp_get_opp)
			opplevel = (*pdata->dsp_get_opp)();
		if (opplevel == 1) {
                       if (pdata->dsp_set_min_opp)
				(*pdata->dsp_set_min_opp)(opplevel+1);
		}
       }
#endif

       if (pDevContext->dwBrdState == BRD_DSP_HIBERNATION ||
	    pDevContext->dwBrdState == BRD_HIBERNATION) {
		/* Restore mailbox settings */
               /* Restart the peripheral clocks that were disabled only
               * in DSP initiated Hibernation case.*/
               if (pDevContext->dwBrdState == BRD_DSP_HIBERNATION) {
                       DSP_PeripheralClocks_Enable(hDevContext, NULL);
                /* Enabling Dpll in lock mode*/
                       temp = (u32) *((REG_UWORD32 *)
                                       ((u32) (resources.dwCmBase) + 0x34));
                       temp = (temp & 0xFFFFFFFE) | 0x1;
                       *((REG_UWORD32 *) ((u32) (resources.dwCmBase) + 0x34)) =
                                       (u32) temp;
                       temp = (u32) *((REG_UWORD32 *)
                                       ((u32) (resources.dwCmBase) + 0x4));
                       temp = (temp & 0xFFFFFC8) | 0x37;

                       *((REG_UWORD32 *) ((u32) (resources.dwCmBase) + 0x4)) =
                                       (u32) temp;
               }
		status = HW_MBOX_restoreSettings(resources.dwMboxBase);

               /*  Access MMU SYS CONFIG register to generate a short wakeup */
               temp = (u32) *((REG_UWORD32 *) ((u32)
                       (resources.dwDmmuBase) + 0x10));

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

DSP_STATUS CHNLSM_InterruptDSP2(struct WMD_DEV_CONTEXT *hDevContext,
				u16 wMbVal)
{
	struct WMD_DEV_CONTEXT *pDevContext = hDevContext;

	pDevContext->wIntrVal2Dsp = wMbVal;

	return CHNLSM_InterruptDSP(hDevContext);
}

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
