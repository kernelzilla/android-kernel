/*
 * bridge/inc/dspapi.h
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2008 Texas Instruments, Inc.
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
 *  ======== dspapi.h ========
 *  Purpose:
 *      Defines function type modifiers used in all DSPSYS public header
 *      files.
 *
 *  Notes:
 *      Provides __stdcall (required by VB 4.0) and __declspec(dllimport)
 *      function modifiers for fast dyna-linking.
 *
 *! Revision History:
 *! =================
 *! 23-Dec-1997 cr: Added WBKERNEL_API definition.
 *! 11-Oct-1996 gp: Created.
 */

#ifndef DSPAPI_
#define DSPAPI_

/* Define API decoration for direct importing of DLL references. */
#if !defined(_DSPSYSDLL32_)
#define DSPAPIDLL __declspec(dllimport)
#else
#define DSPAPIDLL
#endif

/* Full export modifier: */
#define DSPAPI DSPAPIDLL DSP_STATUS WINAPI

/* Explicitly define class driver calling conventions */
#define WBKERNEL_API CDECL

#endif				/* DSPAPI_ */
