/*
 * bridge/inc/prcs.h
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
 *  ======== prcs.h ========
 *  Purpose:
 *      Return process and thread information.
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
 *! 06-Jul-2000 rr: Name changed to PRCS to accomodate RM PROC.
 *! 29-Oct-1999 kc: Cleaned up for code review.
 *! 22-Oct-1996 gp: Created.
 */

#ifndef _PRCS_H
#define _PRCS_H

/*
 *  ======== PRCS_Exit ========
 *  Purpose:
 *      Discontinue usage of module; free resources when reference count
 *      reaches 0.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      PRCS initialized.
 *  Ensures:
 *      Resources used by module are freed when cRef reaches zero.
 */
	extern void PRCS_Exit();

/*
 *  ======== PRCS_GetCurrentHandle ========
 *  Purpose:
 *      Retrieve the handle of the calling process.
 *  Parameters:
 *      phProcess:      Location to store the current process handle.
 *  Returns:
 *      DSP_SOK:        Success.
 *      DSP_EPOINTER:   Invalid argument.
 *  Requires:
 *      - PRCS initialized.
 *      - phProcess != NULL.
 *  Ensures:
 *
 */
	extern DSP_STATUS PRCS_GetCurrentHandle(OUT HANDLE *phProcess);



/*
 *  ======== PRCS_Init ========
 *  Purpose:
 *      Initializes private state of PROC module.
 *  Parameters:
 *  Returns:
 *      TRUE if initialized; FALSE if error occured.
 *  Requires:
 *  Ensures:
 *      PRCS initialized.
 */
	extern bool PRCS_Init();

#endif				/* _PRCS_H */
