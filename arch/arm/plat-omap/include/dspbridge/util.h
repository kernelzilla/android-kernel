/*
 * util.h
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
 *  ======== util.h ========
 *  Purpose:
 *      Provide general purpose utility functions.
 *
 *  Public Functions:
 *      UTIL_CDTestDll
 *      UTIL_CmdLineToArgs
 *      UTIL_Exit
 *      UTIL_GetSysInfo
 *      UTIL_Init
 *      UTIL_Wait
 *
 *  Private Functions:
 *      isspace
 *
 *  Notes:
 *
 *! Revision History
 *! ================
 *! 12-Aug-2000 ag: Added UTIL_GetSysInfo().
 *! 31-Jan-2000 rr: Comments modifed for UTIL_Wait fxn.
 *! 17-Jan-2000 rr: UTIL_Wait fxn added. Busy waits x number of micro seconds.
 *! 12-Nov-1999 kc: Modified UTIL_CmdLineToArgs for use by different tests.
 *! 02-Nov-1999 kc: Removed parameter from UTIL_CDTestDll.
 *! 29-Oct-1999 kc: Cleaned up for code review.
 *! 08-Oct-1999 kc: Renamed UTIL_TestDllExec to UTIL_CDTestDll; added
 *!                 UTIL_CmdLineToArgs(); added isspace private func.
 *! 26-Dec-1996 cr: Created from misc pieces of wcd95.c, ktest.c.
 *
 */

#ifndef _UTIL_H
#define _UTIL_H

#include <linux/delay.h>
#include <linux/sched.h>

#include <dspbridge/utildefs.h>

/*
 *  ======== UTIL_CDTestDll ========
 *  Purpose:
 *      Provides test entry point in class driver context.
 *  Parameters:
 *      cArgc:  test module command line input count.
 *      ppArgv: test module command line args.
 *  Returns:
 *      0 if successful, a negative value otherwise.
 *  Requires:
 *      UTIL initialized.
 *  Ensures:
 */
	extern u32 UTIL_CDTestDll(IN s32 cArgc, IN char **ppArgv);

/*
 *  ======== UTIL_CmdLineToArgs ========
 *  Purpose:
 *      This function re-creates C-style cmd line argc & argv from WinMain()
 *      cmd line args.
 *  Parameters:
 *      s8 *pszProgName   - The name of the program currently being executed.
 *      s8 *argv[]        - The argument vector.
 *      s8 *pCmdLine      - The pointer to the command line.
 *      bool fHasProgName   - Indicats whether a program name is supplied.
 *  Returns:
 *      Returns the number of arguments found.
 *  Requires:
 *      UTIL initialized.
 *  Ensures:
 */
	extern s32 UTIL_CmdLineToArgs(IN char *pszProgName,
				      IN char *argv[UTIL_MAXARGVS],
				      IN char *pCmdLine, IN bool fHasProgName);

/*
 *  ======== UTIL_Exit ========
 *  Purpose:
 *      Discontinue usage of module; free resources when reference count
 *      reaches 0.
 *  Parameters:
 *  Returns:
 *  Requires:
 *      UTIL initialized.
 *  Ensures:
 *      Resources used by module are freed when cRef reaches zero.
 */
	extern inline void UTIL_Exit(void)
	{
	}
/*
 *  ======== UTIL_GetSysInfo ========
 *  Purpose:
 *      This function return platform specific system information.
 *
 *  Parameters:
 *      pSysInfo  - address to store the system information.
 *  Returns:
 *      DSP_SOK
 *      S_FAIL
 *  Requires:
 *      UTIL initialized.
 *      pSysInfo != NULL
 *  Ensures:
 */
	extern DSP_STATUS UTIL_GetSysInfo(OUT struct UTIL_SYSINFO *pSysInfo);

/*
 *  ======== UTIL_Init ========
 *  Purpose:
 *      Initializes private state of UTIL module.
 *  Parameters:
 *  Returns:
 *      TRUE if success, else FALSE.
 *  Requires:
 *  Ensures:
 *      UTIL initialized.
 */
	extern inline bool UTIL_Init(void)
	{
		return true;
	}

/*
 *  ======== UTIL_Wait ========
 *  Purpose:
 *      This fucntion busy waits given a certain number of micro seconds
 *      independent of the processor speed. This is acheived by the
 *      OEM functions QueryPerformanceCounter and QueryPerformanceFrequency.
 *  Parameters:
 *      u32   Number of Micro seconds to busy wait
 *  Returns:
 *          Nothing
 *  Requires:
 *          UTIL_Initalized.
 *  Ensures:
 *      Busy waits x Micro seconds or Sleeps X milli seconds; in that
 *      case, it is a blocking call there will be a context switching
 *      hence it may not represent the absolute busy wait time.
 */
	extern inline void UTIL_Wait(IN u32 dwMicroSeconds)
	{
		if (dwMicroSeconds / 1000 <= 1) {
			/* <= 1 millisecond delay */
			udelay(dwMicroSeconds);
		} else if (dwMicroSeconds * HZ / 1000000 <= 1) {
			/* <= 10 millisecond delay */
			mdelay(dwMicroSeconds / 1000);
		} else {
			/* > 10 millisecond delay */
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(dwMicroSeconds * HZ / 1000000);
		}
	}

#endif				/* _UTIL_H */
