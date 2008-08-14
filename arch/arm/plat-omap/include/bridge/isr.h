/*
 * bridge/inc/isr.h
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
 *  ======== isr.h ========
 *  Purpose:
 *      Interrupt services.
 *
 *  Public Functions:
 *      ISR_Disable
 *      ISR_Exit
 *      ISR_GetStatus
 *      ISR_Init
 *      ISR_Install
 *      ISR_Restore
 *      ISR_SimulateInt
 *      ISR_Uninstall
 *
 *  Notes:
 *
 *! Revision History:
 *! ================
 *! 01-Mar-2004 vp:  Added IVA related functions.
 *! 06-Feb-2003 kc:  Added ISR_MAILBOX1 (renamed from DSP_MAILBOX1).
 *! 05-Nov-2001 kc:  Added interrupt type param to ISR_Install.
 *! 31-Jan-2000 rr:  Comments modified after code review.
 *! 29-Oct-1999 kc:  Moved header files to within ifdefed 'extern "C"'.
 *! 17-Sep-1997 gp:  Added CFG_HOSTRES struct as argument to ISR_Install, thus
 *!                  breaking backward compatibility with Ver. 1.0 mini drivers.
 *! 18-Aug-1997 cr:  Added explicit CDECL identifiers.
 *! 03-Feb-1996 gp:  Changed behaviour of ISR_SimulateInt.
 *! 24-Jul-1996 gp:  Created.
 */

#ifndef ISR_
#define ISR_
#include <host_os.h>
#include <cfg.h>

/* Interrupt Object handle: */
	struct ISR_IRQ;

/* ISR install type (private) */
#define ISR_MAILBOX1            0x00000080	/* Arbitrary value */
#define ISR_MAILBOX2			0x00000081

/* Temporary until the baseport defines it */
#define MAIL_U3_MPU_IRQ	34
/*
 *  ======== ISR_PROC ========
 *  Purpose:
 *      Routine to service an interrupt.
 *  Parameters:
 *      pRefData:   Ptr to user data: passed in via ISR_Install.
 *  Returns:
 *      TRUE if the interrupt was handled; FALSE otherwise.
 *  Requires:
 *      ISR code must be in locked memory.
 *      All data touched must be locked.
 *      No resources should be acquired within the ISR.
 *      May only call asynchrounous services.
 *  Ensures:
 *      This routine must not affect the state of the physical PIC.
 *      (i.e.; don't send an EOI).
 */
	typedef bool (CDECL *ISR_PROC) (void *pRefData);

/*
 *  ======== ISR_Disable ========
 *  Purpose:
 *      Turns off interrupts to begin a critical section of code.
 *      Not implemented in CE
 *  Parameters:
 *      pFlags: Location to store flags.
 *  Returns:
 *  Requires:
 *  Ensures:
 */
	extern void CDECL ISR_Disable(OUT u32 *pFlags);

/*
 *  ======== ISR_Exit ========
 *  Purpose:
 *      Discontinue usage of the ISR module.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      ISR_Init() was previously called.
 *  Ensures:
 *      Resources acquired in ISR_Init() are freed.
 */
	extern void CDECL ISR_Exit();

/*
 *  ======== ISR_GetStatus ========
 *  Purpose:
 *      Return platform specific status flags containing information about
 *      a virtualized IRQ.  Used by clients for debugging only.
 *      Not implemented.
 *  Parameters:
 *      hIRQ:       Interrupt object handle as returned by ISR_Install().
 *      pdwFlags:   Location to store status flags on output.
 *  Returns:
 *      DSP_SOK:        Success;
 *      DSP_ENOTIMPL:   Not implemented
 *  Requires:
 *      pdwFlags != NULL.
 *  Ensures:
 */
	extern DSP_STATUS CDECL ISR_GetStatus(IN struct ISR_IRQ *hIRQ,
					      OUT u32 *pdwFlags);

/*
 *  ======== ISR_Init ========
 *  Purpose:
 *      Initialize the ISR module's private state.
 *  Parameters:
 *  Returns:
 *      TRUE if initialized; FALSE if error occured.
 *  Requires:
 *  Ensures:
 *      A requirement for each of the other public ISR functions.
 */
	extern bool CDECL ISR_Init();

/*
 *  ======== ISR_Install ========
 *  Purpose:
 *      Register an ISR for a given IRQ with the system's interrupt manager.
 *  Parameters:
 *      phIRQ:          Ptr to store a handle to an interrupt object.
 *      pHostConfig:    Ptr to a host resource structure, containing both
 *                      interrupt and bus info.
 *      pfnISR:         ISR function.  See definition of ISR_PROC.
 *      dwIntrType:     Type of interrupt.
 *      pRefData:       Pointer to user-defined reference data.
 *  Returns:
 *      DSP_SOK:        ISR installed.
 *      DSP_EPOINTER:   phIRQ == NULL.
 *      DSP_EMEMORY:    Insufficient memory.
 *      DSP_EFAIL:      Error registering the ISR for the IRQ.
 *  Requires:
 *      pHostConfig is a valid pointer to assigned host resources;
 *      iIRQ number is valid for this (host) processor.
 *      See requirements for ISR_PROC.
 *  Ensures:
 *      DSP_SOK:    ISR is installed, and the IRQ is unmasked. The ISR can be
 *                  called at any time until ISR_Uninstall() is called for this
 *                  IRQ.
 *      else:       *phIRQ is set to NULL.
 */
	extern DSP_STATUS CDECL ISR_Install(OUT struct ISR_IRQ **phIRQ,
					    IN CONST struct CFG_HOSTRES
					    *pHostConfig,
					    IN ISR_PROC pfnISR,
					    IN u32 dwIntrType,
					    IN void *pRefData);

/*
 *  ======== ISR_Restore ========
 *  Purpose:
 *      In CE, the client should use ISR_Install to restore the interrupt.
 *  Parameters:
 *      saveFlags: To save or not to save.
 *  Returns:
 *  Requires:
 *  Ensures:
 */
	extern void CDECL ISR_Restore(IN u32 saveFlags);

/*
 *  ======== ISR_SimulateInt ========
 *  Purpose:
 *      Simulate a hardware interrupt.
 *  Parameters:
 *      hIRQ:       Interrupt object handle as returned by ISR_Install().
 *  Returns:
 *      DSP_SOK:        Success;
 *      DSP_ENOTIMPL:   Not yet implemented.
 *      DSP_EHANDLE:    Invalid hIRQ.
 *  Requires:
 *      DSP_SOK:    An interrupt handler must have been previously installed
 *                  with ISR_Install().
 *  Ensures:
 *      DSP_SOK:    The ISR installed by ISR_Install() will be called, before
 *                  or after this function returns.
 */
	extern DSP_STATUS CDECL ISR_SimulateInt(IN struct ISR_IRQ *hIRQ);

/*
 *  ======== ISR_Uninstall ========
 *  Purpose:
 *      Deregister the ISR previously installed by ISR_Install().
 *  Parameters:
 *      hIRQ:           Handle to an IRQ object returned from ISR_Install().
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EHANDLE:    Invalid hIRQ.
 *  Requires:
 *  Ensures:
 *      (SUCCESS && hDPC is NULL) or DSP_EFAILED status
 */
	extern DSP_STATUS CDECL ISR_Uninstall(IN struct ISR_IRQ *hIRQ);

#endif				/* ISR_ */
