/*
 * linux/drivers/dsp/bridge/services/isr.c
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
 *  ======== isrce.c ========
 *  Purpose:
 *      Interrupt services.
 *
 *  Public Functions:
 *      ISR_Exit
 *      ISR_Init
 *      ISR_Install
 *      ISR_Uninstall
 *
 *
 *! Revision History:
 *! ================
 *! 06-Feb-2003 kc: Renamed DSP_MAILBOX1 to ISR_MAILBOX1.
 *! 14-Mar-2002 rr: Added HELEN1_V1 flag while installing the interrupt.
 *! 05-Nov-2001 kc: Updated ISR_Install to support multiple HW interrupts.
 *! 27-Jul-2001 rr: Interrupt Id is based on x86 or ARM define.
 *! 24-Apr-2001 ag: Replaced nkintr.h with hw.h.
 *! 10-Oct-2000 rr: CeSetThreadPriority used instead of SetThreadPriority.
 *! 11-Aug-2000 ag: Removed #include <stdwin.h>
 *! 10-Aug-2000 rr: InterruptInitialize happens before the IST creation.
 *! 15-Feb-2000 rr: InterruptInitialize return value checked.
 *! 03-Feb-2000 rr: Module init/exit is handled by SERVICES Init/Exit.
 *!		 GT Changes.
 *! 31-Jan-2000 rr: Changes after code review.Terminate thread,handle
 *!                 modified.ISR_UnInstall frees the ISR_Object only on
 *!                 Successful termination of the thread and the handle.
 *! 19-Jan-2000 rr: Code Cleaned up after code review.
 *! 06-Jan-2000 rr: Bus type included in the IRQ object. It is checked
 *!                 during the install and uninstall.
 *! 29-Dec-1999 rr: WaitForSingleObject removed during ISR_UnInstall
 *! 22-Nov-1999 rr: Event gets created before CardRequestIRQ
 *! 05-Nov-1999 rr: ISR_Install does not intialize the interrupt for PCMCIA
 *!                 For other bus type this will happen. IST function return
 *!                 value not checked as anyway we have to say InterruptDone
 *! 29-Oct-1999 rr: Hardware IST is created here.
 *! 25-Oct-1999 rr: New Isr design.
 *! 13-Sep-1999 a0216266: Stubbed from isrnt.c.
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

/*  ----------------------------------- OS Adaptation Layer */
#include <list.h>
#include <mem.h>

/*  ----------------------------------- This */
#include <isr.h>

/*  ----------------------------------- Defines, Data Structures, Typedefs */
#define SIGNATURE       0x5f525349	/* "ISR_" (in reverse). */

/* The IRQ object, passed to our hardware and virtual interrupt routines: */
struct ISR_IRQ {
	u32 dwSignature;	/* Used for object validation.   */
	void *pRefData;		/* Argument for client's ISR.    */
	ISR_PROC pfnISR;	/* Client's ISR.                 */

	u32 dwIntrID;		/* hardware intertupt identifier */
};

/*  ----------------------------------- Globals & Defines */
#if GT_TRACE
static struct GT_Mask ISR_DebugMask = { NULL, NULL };	/* ISR Debug Mask */
#endif

/*  ----------------------------------- Function Prototypes */
static irqreturn_t HardwareIST(int irq, void *hIRQ);

/*
 *  ======== ISR_Exit ========
 *  Purpose:
 *      Discontinue usage of the ISR module.
 */
void ISR_Exit(void)
{
	GT_0trace(ISR_DebugMask, GT_ENTER, "Entered ISR_Exit\n");
}

/*
 *  ======== ISR_Init ========
 *  Purpose:
 *      Initialize the ISR module's private state.
 */
bool ISR_Init(void)
{
	GT_create(&ISR_DebugMask, "IS");

	GT_0trace(ISR_DebugMask, GT_5CLASS, "Entered ISR_Init\n");

	return true;
}

/*
 *  ======== ISR_Install ========
 *  Purpose:
 *      Register an ISR for a given IRQ with the system's interrupt manager.
 */
DSP_STATUS ISR_Install(OUT struct ISR_IRQ **phIRQ,
		       IN CONST struct CFG_HOSTRES *pHostConfig,
		       ISR_PROC pfnISR, u32 dwIntrType, void *pRefData)
{
	DSP_STATUS status = DSP_SOK;
	struct ISR_IRQ *pIRQObject = NULL;

	DBC_Require(pHostConfig);
	DBC_Require(pRefData != NULL);

	GT_5trace(ISR_DebugMask, GT_ENTER,
		  "Entered ISR_Install, args:" "\n\thIRQ:"
		  "0x%x\n\tpHostConfig: 0x%x\n\tpfnISR: 0x%x\n" "\tpRefData:"
		  "0x%x\n \tdwIntrType 0x%x\n", phIRQ, pHostConfig, pfnISR,
		  pRefData, dwIntrType);

	if (phIRQ != NULL) {
		*phIRQ = NULL;
		/*
		 *  Allocate an IRQ object to store information allowing our
		 *  interrupt handler to dispatch to the client's interrupt
		 *  routines.
		 */
		MEM_AllocObject(pIRQObject, struct ISR_IRQ, SIGNATURE);
		if (pIRQObject != NULL) {
			/* Fill out the Object: */
			pIRQObject->pRefData = pRefData;
			pIRQObject->pfnISR = pfnISR;

			/* Install different HW interrupts based on interrupt
			 * type. */
				switch (dwIntrType) {
				case ISR_MAILBOX1:
				pIRQObject->dwIntrID = INT_MAIL_MPU_IRQ;

				GT_0trace(ISR_DebugMask, GT_1CLASS,
					  "Setting intr id for "
					  "ISR_MAILBOX1\n");
				break;

				case ISR_MAILBOX2:
				pIRQObject->dwIntrID = MAIL_U3_MPU_IRQ;
				GT_0trace(ISR_DebugMask, GT_1CLASS,
					  "Setting intr id for "
					  "ISR_MAILBOX2\n");

				break;

				case DSP_MMUFAULT:
				pIRQObject->dwIntrID = INT_DSP_MMU_IRQ;
				GT_0trace(ISR_DebugMask, GT_1CLASS,
					  "Setting intr id for "
					  "DSP_MMUFAULT\n");
				break;

				default:
				pIRQObject->dwIntrID = (u32) 0x00;
				GT_0trace(ISR_DebugMask, GT_1CLASS,
					  "Setting intr id to NULL\n");
				break;
			}

			if (pIRQObject->dwIntrID != 0) {
				status = request_irq(pIRQObject->dwIntrID,
						     HardwareIST, 0,
						    "DspBridge", pIRQObject);
			} else {
				status = DSP_EINVALIDARG;
			}
		}
		if (DSP_SUCCEEDED(status)) {
			*phIRQ = pIRQObject;
			GT_1trace(ISR_DebugMask, GT_1CLASS,
				  "ISR:IRQ Object 0x%x \n",
				  pIRQObject);
		} else {
			MEM_FreeObject(pIRQObject);
		}
	}
	DBC_Ensure(DSP_SUCCEEDED(status) || (!phIRQ || *phIRQ == NULL));
	return status;
}

/*
 *  ======== ISR_Uninstall ========
 *  Purpose:
 *      Deregister the ISR previously installed by ISR_Install().
 *      if it fails we do not delete the IRQ object.
 */
DSP_STATUS ISR_Uninstall(struct ISR_IRQ *hIRQ)
{
	DSP_STATUS status = DSP_SOK;
	struct ISR_IRQ *pIRQObject = (struct ISR_IRQ *)hIRQ;

	DBC_Require(hIRQ > 0);

	GT_1trace(ISR_DebugMask, GT_ENTER,
		  "Entered ISR_Uninstall, hIRQ: 0x%x\n", hIRQ);

	if (MEM_IsValidHandle(hIRQ, SIGNATURE)) {
		/* Linux function to uninstall ISR */
		free_irq(pIRQObject->dwIntrID, pIRQObject);
		pIRQObject->dwIntrID = (u32) -1;
	} else {
		status = DSP_EHANDLE;
	}

	/* Free our IRQ object: */
	if (DSP_SUCCEEDED(status)) {
		MEM_FreeObject(pIRQObject);
		pIRQObject = NULL;
	}

	DBC_Ensure((DSP_SUCCEEDED(status) && pIRQObject == NULL) ||
		   DSP_FAILED(status));

	return status;
}

/*
 *  ======== HardwareIST ========
 *  Purpose:
 *      Linux calls this IRQ handler on interrupt.
 */
static irqreturn_t HardwareIST(int irq, void *hIRQ)
{
	struct ISR_IRQ *pIRQObject = (struct ISR_IRQ *)hIRQ;

	DBC_Require(irq == pIRQObject->dwIntrID);

	/* Call the function registered in ISR_Install */
	(*(pIRQObject->pfnISR)) (pIRQObject->pRefData);

	return IRQ_HANDLED;
}
