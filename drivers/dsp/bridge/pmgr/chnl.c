/*
 * linux/drivers/dsp/bridge/pmgr/chnl.c
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
 *  ======== chnl.c ========
 *  Description:
 *      WCD channel interface: multiplexes data streams through the single
 *      physical link managed by a 'Bridge mini-driver.
 *
 *  Public Functions:
 *      CHNL_Close
 *      CHNL_CloseOrphans
 *      CHNL_Create
 *      CHNL_Destroy
 *      CHNL_Exit
 *      CHNL_GetHandle
 *      CHNL_GetProcessHandle
 *      CHNL_Init
 *      CHNL_Open
 *
 *  Notes:
 *      This interface is basically a pass through to the WMD CHNL functions,
 *      except for the CHNL_Get() accessor functions which call
 *      WMD_CHNL_GetInfo().
 *
 *! Revision History:
 *! ================
 *! 24-Feb-2003 swa PMGR Code review comments incorporated.
 *! 07-Jan-2002 ag  CHNL_CloseOrphans() now closes supported # of channels.
 *! 17-Nov-2000 jeh Removed IRQ, shared memory stuff from CHNL_Create.
 *! 28-Feb-2000 rr: New GT USage Implementation
 *! 03-Feb-2000 rr: GT and Module init/exit Changes.(Done up front from
 *!		    SERVICES)
 *! 21-Jan-2000 ag: Added code review comments.
 *! 13-Jan-2000 rr: CFG_Get/SetPrivateDword renamed to CFG_Get/SetDevObject.
 *! 08-Dec-1999 ag: CHNL_[Alloc|Free]Buffer bufs taken from client process heap.
 *! 02-Dec-1999 ag: Implemented CHNL_GetEventHandle().
 *! 17-Nov-1999 ag: CHNL_AllocBuffer() allocs extra word for process mapping.
 *! 28-Oct-1999 ag: WinCE port. Search for "WinCE" for changes(TBR).
 *! 07-Jan-1998 gp: CHNL_[Alloc|Free]Buffer now call MEM_UMB functions.
 *! 22-Oct-1997 gp: Removed requirement in CHNL_Open that hReserved1 != NULL.
 *! 30-Aug-1997 cr: Renamed cfg.h wbwcd.h b/c of WINNT file name collision.
 *! 10-Mar-1997 gp: Added GT trace.
 *! 14-Jan-1997 gp: Updated based on code review feedback.
 *! 03-Jan-1997 gp: Moved CHNL_AllocBuffer/CHNL_FreeBuffer code from udspsys.
 *! 14-Dec-1996 gp: Added uChnlId parameter to CHNL_Open().
 *! 09-Sep-1996 gp: Added CHNL_GetProcessHandle().
 *! 15-Jul-1996 gp: Created.
 */

/*  ----------------------------------- Host OS */
#include <dspbridge/host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <dspbridge/std.h>
#include <dspbridge/dbdefs.h>
#include <dspbridge/errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <dspbridge/dbc.h>
#include <dspbridge/gt.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <dspbridge/cfg.h>
#include <dspbridge/csl.h>
#include <dspbridge/dpc.h>
#include <dspbridge/list.h>
#include <dspbridge/mem.h>
#include <dspbridge/sync.h>

/*  ----------------------------------- Platform Manager */
#include <dspbridge/proc.h>
#include <dspbridge/dev.h>

/*  ----------------------------------- Others */
#include <dspbridge/chnlpriv.h>
#include <chnlobj.h>

/*  ----------------------------------- This */
#include <dspbridge/chnl.h>

/*  ----------------------------------- Globals */
static u32 cRefs;
#if GT_TRACE
static struct GT_Mask CHNL_DebugMask = { NULL, NULL };	/* WCD CHNL Mask */
#endif

/*  ----------------------------------- Function Prototypes */
static DSP_STATUS GetNumOpenChannels(struct CHNL_MGR *hChnlMgr,
				    OUT u32 *pcOpenChannels);

static DSP_STATUS GetNumChannels(struct CHNL_MGR *hChnlMgr,
				 OUT u32 *pcChannels);

/*
 *  ======== CHNL_Close ========
 *  Purpose:
 *      Ensures all pending I/O on this channel is cancelled, discards all
 *      queued I/O completion notifications, then frees the resources
 *      allocated for this channel, and makes the corresponding logical
 *      channel id available for subsequent use.
 */
DSP_STATUS CHNL_Close(struct CHNL_OBJECT *hChnl)
{
	DSP_STATUS status;
	struct CHNL_OBJECT_ *pChnl = (struct CHNL_OBJECT_ *)hChnl;
	struct WMD_DRV_INTERFACE *pIntfFxns;

	DBC_Require(cRefs > 0);

	GT_1trace(CHNL_DebugMask, GT_ENTER,
		  "Entered CHNL_Close:hChnl: 0x%x\n",
		  hChnl);

	if (CHNL_IsValidChnl(pChnl)) {
		pIntfFxns = pChnl->pChnlMgr->pIntfFxns;
		status = (*pIntfFxns->pfnChnlClose) (hChnl);
	} else {
		GT_0trace(CHNL_DebugMask, GT_7CLASS,
			  "CHNL_Close:Invalid Handle\n");
		status = DSP_EHANDLE;
	}
	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Exiting CHNL_Close:hChnl: 0x%x, status:"
		  " 0x%x\n", hChnl, status);
	return status;
}

/*
 *  ======== CHNL_CloseOrphans ========
 *  Purpose:
 *      Close open channels orphaned by a closing process.
 */
DSP_STATUS CHNL_CloseOrphans(struct CHNL_MGR *hChnlMgr, HANDLE hProcess)
{
	u32 uChnlID;
	DSP_STATUS status = DSP_SFALSE;
	HANDLE hProc;
	u32 cOpenChannels;
	u32 cTotalChnls;
	struct CHNL_OBJECT *hChnl;

	DBC_Require(cRefs > 0);

	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Enter CHNL_CloseOrphans hChnlMgr "
		  "0x%x\t\nhProcess: 0x%x\n", hChnlMgr, hProcess);
	if (!CHNL_IsValidMgr((struct CHNL_MGR_ *)hChnlMgr)) {
		status = DSP_EHANDLE;
		goto func_end;
	}
	if (DSP_SUCCEEDED(GetNumOpenChannels(hChnlMgr, &cOpenChannels)) &&
			 (cOpenChannels > 0)) {
		if (DSP_FAILED(GetNumChannels(hChnlMgr, &cTotalChnls)))
			goto func_end;

		/* For each channel (except for RMS), get process handle: */
		for (uChnlID = 2; uChnlID < cTotalChnls; uChnlID++) {
			if (DSP_FAILED(CHNL_GetHandle(hChnlMgr, uChnlID,
			    &hChnl))) {
				continue;
			}
			if (DSP_FAILED(CHNL_GetProcessHandle(hChnl,
			    &hProc))) {
				continue;
			}
			/* See if channel owned by this process: */
			if (hProc == hProcess) {
				/* If so, close it now. */
				CHNL_Close(hChnl);
				status = DSP_SOK;
			}
		}
	}
func_end:
	GT_1trace(CHNL_DebugMask, GT_ENTER, "CHNL_CloseOrphans status 0x%x\n",
		  status);

	return status;
}

/*
 *  ======== CHNL_Create ========
 *  Purpose:
 *      Create a channel manager object, responsible for opening new channels
 *      and closing old ones for a given 'Bridge board.
 */
DSP_STATUS CHNL_Create(OUT struct CHNL_MGR **phChnlMgr,
		       struct DEV_OBJECT *hDevObject,
		       IN CONST struct CHNL_MGRATTRS *pMgrAttrs)
{
	DSP_STATUS status;
	struct CHNL_MGR *hChnlMgr;
	struct CHNL_MGR_ *pChnlMgr = NULL;

	DBC_Require(cRefs > 0);
	DBC_Require(phChnlMgr != NULL);
	DBC_Require(pMgrAttrs != NULL);

	GT_3trace(CHNL_DebugMask, GT_ENTER,
		  "Entered CHNL_Create: phChnlMgr: 0x%x\t"
		  "hDevObject: 0x%x\tpMgrAttrs:0x%x\n",
		  phChnlMgr, hDevObject, pMgrAttrs);

	*phChnlMgr = NULL;

	/* Validate args: */
	if ((0 < pMgrAttrs->cChannels) &&
	   (pMgrAttrs->cChannels <= CHNL_MAXCHANNELS)) {
		status = DSP_SOK;
	} else if (pMgrAttrs->cChannels == 0) {
		status = DSP_EINVALIDARG;
		GT_0trace(CHNL_DebugMask, GT_7CLASS,
			  "CHNL_Create:Invalid Args\n");
	} else {
		status = CHNL_E_MAXCHANNELS;
		GT_0trace(CHNL_DebugMask, GT_7CLASS,
			  "CHNL_Create:Error Max Channels\n");
	}
	if (pMgrAttrs->uWordSize == 0) {
		status = CHNL_E_INVALIDWORDSIZE;
		GT_0trace(CHNL_DebugMask, GT_7CLASS,
			  "CHNL_Create:Invalid Word size\n");
	}
	if (DSP_SUCCEEDED(status)) {
		status = DEV_GetChnlMgr(hDevObject, &hChnlMgr);
		if (DSP_SUCCEEDED(status) && hChnlMgr != NULL)
			status = CHNL_E_MGREXISTS;

	}

	if (DSP_SUCCEEDED(status)) {
		struct WMD_DRV_INTERFACE *pIntfFxns;
		DEV_GetIntfFxns(hDevObject, &pIntfFxns);
		/* Let WMD channel module finish the create: */
		status = (*pIntfFxns->pfnChnlCreate)(&hChnlMgr, hDevObject,
			  pMgrAttrs);
		if (DSP_SUCCEEDED(status)) {
			/* Fill in WCD channel module's fields of the
			 * CHNL_MGR structure */
			pChnlMgr = (struct CHNL_MGR_ *)hChnlMgr;
			pChnlMgr->pIntfFxns = pIntfFxns;
			/* Finally, return the new channel manager handle: */
			*phChnlMgr = hChnlMgr;
			GT_1trace(CHNL_DebugMask, GT_1CLASS,
				  "CHNL_Create: Success pChnlMgr:"
				  "0x%x\n", pChnlMgr);
		}
	}

	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Exiting CHNL_Create: pChnlMgr: 0x%x,"
		  "status: 0x%x\n", pChnlMgr, status);
	DBC_Ensure(DSP_FAILED(status) || CHNL_IsValidMgr(pChnlMgr));

	return status;
}

/*
 *  ======== CHNL_Destroy ========
 *  Purpose:
 *      Close all open channels, and destroy the channel manager.
 */
DSP_STATUS CHNL_Destroy(struct CHNL_MGR *hChnlMgr)
{
	struct CHNL_MGR_ *pChnlMgr = (struct CHNL_MGR_ *)hChnlMgr;
	struct WMD_DRV_INTERFACE *pIntfFxns;
	DSP_STATUS status;

	DBC_Require(cRefs > 0);

	GT_1trace(CHNL_DebugMask, GT_ENTER,
		  "Entered CHNL_Destroy: hChnlMgr: 0x%x\n", hChnlMgr);
	if (CHNL_IsValidMgr(pChnlMgr)) {
		pIntfFxns = pChnlMgr->pIntfFxns;
		/* Let WMD channel module destroy the CHNL_MGR: */
		status = (*pIntfFxns->pfnChnlDestroy)(hChnlMgr);
	} else {
		GT_0trace(CHNL_DebugMask, GT_7CLASS,
			  "CHNL_Destroy:Invalid Handle\n");
		status = DSP_EHANDLE;
	}

	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Exiting CHNL_Destroy: pChnlMgr: 0x%x,"
		  " status:0x%x\n", pChnlMgr, status);
	DBC_Ensure(DSP_FAILED(status) || !CHNL_IsValidMgr(pChnlMgr));

	return status;
}

/*
 *  ======== CHNL_Exit ========
 *  Purpose:
 *      Discontinue usage of the CHNL module.
 */
void CHNL_Exit(void)
{
	DBC_Require(cRefs > 0);

	cRefs--;

	GT_1trace(CHNL_DebugMask, GT_5CLASS,
		  "Entered CHNL_Exit, ref count: 0x%x\n", cRefs);

	DBC_Ensure(cRefs >= 0);
}

/*
 *  ======== CHNL_GetHandle ========
 *  Purpose:
 *      Retrieve the channel handle given the logical ID and channel manager.
 */
DSP_STATUS CHNL_GetHandle(struct CHNL_MGR *hChnlMgr, u32 uChnlID,
			  OUT struct CHNL_OBJECT **phChnl)
{
	DSP_STATUS status = DSP_SOK;
	struct CHNL_MGR_ *pChnlMgr = (struct CHNL_MGR_ *)hChnlMgr;
	struct WMD_DRV_INTERFACE *pIntfFxns;
	struct CHNL_MGRINFO chnlMgrInfo;

	DBC_Require(cRefs > 0);

	GT_3trace(CHNL_DebugMask, GT_ENTER, "Entered CHNL_GetHandle: hChnlMgr: "
		  "0x%x\tuChnlID: 0x%x\t\nphChnl: 0x%x\n", hChnlMgr, uChnlID,
		  phChnl);
	if (phChnl) {
		*phChnl = NULL;
		if (CHNL_IsValidMgr(pChnlMgr)) {
			pIntfFxns = pChnlMgr->pIntfFxns;
			status = (*pIntfFxns->pfnChnlGetMgrInfo)(hChnlMgr,
				  uChnlID, &chnlMgrInfo);
			if (DSP_SUCCEEDED(status))
				*phChnl = chnlMgrInfo.hChnl;

		} else {
			status = DSP_EHANDLE;
			GT_0trace(CHNL_DebugMask, GT_7CLASS,
				  "CHNL_GetHandle:Invalid Handle\n");
		}
	} else {
		status = DSP_EPOINTER;
	}
	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Exit CHNL_GetHandle: status: 0x%x\t\n"
		  "hChnl: 0x%x\n", status, *phChnl);
	return status;
}

/*
 *  ======== CHNL_GetProcessHandle ========
 *  Purpose:
 *      Retrieve the handle of the process owning this channel.
 */
DSP_STATUS CHNL_GetProcessHandle(struct CHNL_OBJECT *hChnl,
				 OUT HANDLE *phProcess)
{
	DSP_STATUS status = DSP_SOK;
	struct CHNL_OBJECT_ *pChnl = (struct CHNL_OBJECT_ *)hChnl;
	struct WMD_DRV_INTERFACE *pIntfFxns;
	struct CHNL_INFO chnlInfo;

	DBC_Require(cRefs > 0);

	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Enter CHNL_GetProcessHandle: hChnl: "
		  "0x%x\t\n phProcess: 0x%x\n", hChnl, phProcess);
	if (phProcess) {
		*phProcess = NULL;
		if (CHNL_IsValidChnl(pChnl)) {
			pIntfFxns = pChnl->pChnlMgr->pIntfFxns;
			status = (*pIntfFxns->pfnChnlGetInfo)(hChnl, &chnlInfo);
			if (DSP_SUCCEEDED(status))
				*phProcess = chnlInfo.hProcess;

		} else {
			status = DSP_EHANDLE;
		}
	} else {
		status = DSP_EPOINTER;
	}
	GT_2trace(CHNL_DebugMask, GT_ENTER,
		  "Exit CHNL_GetProcessHandle: status: "
		  "0x%x\t\n phProcess: 0x%x\n", status, *phProcess);
	return status;
}

/*
 *  ======== CHNL_Init ========
 *  Purpose:
 *      Initialize the CHNL module's private state.
 */
bool CHNL_Init(void)
{
	bool fRetval = true;

	DBC_Require(cRefs >= 0);

	if (cRefs == 0) {
		DBC_Assert(!CHNL_DebugMask.flags);
		GT_create(&CHNL_DebugMask, "CH");   /* "CH" for CHannel */
	}

	if (fRetval)
		cRefs++;

	GT_1trace(CHNL_DebugMask, GT_5CLASS,
		  "Entered CHNL_Init, ref count: 0x%x\n",
		  cRefs);

	DBC_Ensure((fRetval && (cRefs > 0)) || (!fRetval && (cRefs >= 0)));

	return fRetval;
}

/*
 *  ======== GetNumOpenChannels ========
 *  Purpose:
 *      Retrieve number of open channels
 *  Parameters:
 *      hChnlMgr:       Handle to a valid channel manager, or NULL.
 *      pcOpenChannels: Location to store number of open channels.
 *  Returns:
 *      DSP_SOK:        Success;
 *      DSP_EHANDLE:    Invalid hChnlMgr.
 *      E_POINTER:      pcOpenChannels == NULL.
 *  Requires:
 *  Ensures:
 *      DSP_SOK:        *pcOpenChannels points to a valid number
 *                      if pcOpenChannels != NULL.
 */
static DSP_STATUS GetNumOpenChannels(struct CHNL_MGR *hChnlMgr,
				     OUT u32 *pcOpenChannels)
{
	DSP_STATUS status = DSP_SOK;
	struct CHNL_MGR_ *pChnlMgr = (struct CHNL_MGR_ *)hChnlMgr;
	struct WMD_DRV_INTERFACE *pIntfFxns;
	struct CHNL_MGRINFO chnlMgrInfo;

	DBC_Require(cRefs > 0);
	if (pcOpenChannels) {
		*pcOpenChannels = 0;
		if (CHNL_IsValidMgr(pChnlMgr)) {
			pIntfFxns = pChnlMgr->pIntfFxns;
			status = (*pIntfFxns->pfnChnlGetMgrInfo)(hChnlMgr, 0,
				 &chnlMgrInfo);
			if (DSP_SUCCEEDED(status))
				*pcOpenChannels = chnlMgrInfo.cOpenChannels;

		} else {
			status = DSP_EHANDLE;
		}
	} else {
		status = DSP_EPOINTER;
	}
	return status;
}

/*
 *  ======== GetNumOpenChannels ========
 *  Purpose:
 *      Retrieve number of total channels supported.
 *  Parameters:
 *      hChnlMgr:       Handle to a valid channel manager, or NULL.
 *      pcChannels:     Location to store number of channels.
 *  Returns:
 *      DSP_SOK:        Success;
 *      DSP_EHANDLE:    Invalid hChnlMgr.
 *      E_POINTER:      pcOpenChannels == NULL.
 *  Requires:
 *  Ensures:
 *      DSP_SOK:        *pcChannels points to a valid number
 *                      if pcOpenChannels != NULL.
 */
static DSP_STATUS GetNumChannels(struct CHNL_MGR *hChnlMgr,
				 OUT u32 *pcChannels)
{
	DSP_STATUS status = DSP_SOK;
	struct CHNL_MGR_ *pChnlMgr = (struct CHNL_MGR_ *)hChnlMgr;
	struct WMD_DRV_INTERFACE *pIntfFxns;
	struct CHNL_MGRINFO chnlMgrInfo;

	DBC_Require(cRefs > 0);

	if (pcChannels) {
		*pcChannels = 0;
		if (CHNL_IsValidMgr(pChnlMgr)) {
			pIntfFxns = pChnlMgr->pIntfFxns;
			status = (*pIntfFxns->pfnChnlGetMgrInfo)(hChnlMgr, 0,
				 &chnlMgrInfo);
			if (DSP_SUCCEEDED(status))
				*pcChannels = chnlMgrInfo.cChannels;

		} else {
			status = DSP_EHANDLE;
		}
	} else {
		status = DSP_EPOINTER;
	}
	return status;
}

