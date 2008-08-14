/*
 * linux/drivers/dsp/bridge/services/prcs.c
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
 *  ======== procce.c ========
 *  Purpose:
 *      Provide information about processes and threads.
 *
 *  Public Functions:
 *      PRCS_Exit
 *      PRCS_GetCurrentHandle
 *      PRCS_Init
 *
 *! Revision History:
 *! ================
 *! 18-Dec-2000 rr: PRCS_GetCurrentProcesshandle's DBC_Ensure class
 *!                 removed. See the foot node.
 *! 06-Jul-2000 rr: Prefix changed to PRCS to accomodate RM PROC.
 *! 03-Feb-2000 rr: Module init/exit is handled by SERVICES Init/Exit.
 *!		 GT Changes.
 *! 22-Nov-1999 kc: Added changes from code review.
 *! 22-Sep-1999 kc: Modified from procnt.c.
 *! 26-Aug-1997 cr: Implemented.
 *! 16-Aug-1997 cr: Stubbed from proc95.c
 */

/*  ----------------------------------- Host OS */
#include <host_os.h>

/*  ----------------------------------- DSP/BIOS Bridge */
#include <std.h>
#include <dbdefs.h>
#include <errbase.h>

/*  ----------------------------------- Trace & Debug */
#include <dbc.h>
#include <gt.h>

/*  ----------------------------------- This */
#include <prcs.h>

/*  ----------------------------------- Globals & Defines */
#if GT_TRACE
static struct GT_Mask PRCS_debugMask = { NULL, NULL };	/* GT trace var. */
#endif

/*
 *  ======== PRCS_Exit ========
 *  Purpose:
 *      Discontinue usage of the PRCS module.
 */
void PRCS_Exit(void)
{
	GT_0trace(PRCS_debugMask, GT_5CLASS, "PRCS_Exit\n");
}

/*
 *  ======== PRCS_GetCurrentHandle ========
 *  Purpose:
 *      This functions returns the process handle of the
 *      caller process.
 */
DSP_STATUS PRCS_GetCurrentHandle(OUT HANDLE *phProcess)
{
	DSP_STATUS status;

	DBC_Require(phProcess != NULL);

	GT_1trace(PRCS_debugMask, GT_ENTER,
		  "PRCS_GetCurrentHandle: phProcess 0x%x\n",
		  phProcess);
	if (phProcess) {
		if (!in_interrupt()) {
			/* Return PID instead of process handle */
			*phProcess = (HANDLE)current->pid;
			status = DSP_SOK;
		} else {
			*phProcess = NULL;
			status = DSP_EFAIL;
		}
	} else {
		*phProcess = NULL;
		status = DSP_EPOINTER;
		GT_0trace(PRCS_debugMask, GT_6CLASS,
			  "PRCS_GetCurrentHandle: invalid "
			  "handle\n");
	}
	return status;
}

/*
 *  ======== PRCS_Init ========
 *  Purpose:
 *      Initialize the PRCS module's private state.
 */
bool PRCS_Init(void)
{

	GT_create(&PRCS_debugMask, "PS");	/* PS for ProcesS */

	GT_0trace(PRCS_debugMask, GT_5CLASS, "PRCS_Init\n");

	return true;
}

