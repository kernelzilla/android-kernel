/*
 * bridge/inc/dldr.h
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
 *  ======== dldr.h ========
 *
 *  Description:
 *      DSP/BIOS Bridge dynamic loader interface. See the file dldrdefs.h
 *  for a description of these functions.
 *
 *  Public Functions:
 *      DLDR_Allocate
 *      DLDR_Create
 *      DLDR_Delete
 *      DLDR_Exit
 *      DLDR_Free
 *      DLDR_GetFxnAddr
 *      DLDR_Init
 *      DLDR_Load
 *      DLDR_Unload
 *
 *  Notes:
 *
 *! Revision History
 *! ================
 *! 31-Jul-2002 jeh     Removed function header comments.
 *! 17-Apr-2002 jeh     Created.
 */

#include <dbdefs.h>
#include <dbdcddef.h>
#include <dldrdefs.h>

#ifndef DLDR_
#define DLDR_

	extern DSP_STATUS DLDR_Allocate(struct DLDR_OBJECT *hDldr,
					void *pPrivRef,
					IN CONST struct DCD_NODEPROPS
					*pNodeProps,
					OUT struct DLDR_NODEOBJECT
					**phDldrNode);

	extern DSP_STATUS DLDR_Create(OUT struct DLDR_OBJECT **phDldr,
				      struct DEV_OBJECT *hDevObject,
				      IN CONST struct DLDR_ATTRS *pAttrs);

	extern void DLDR_Delete(struct DLDR_OBJECT *hDldr);
	extern void DLDR_Exit();
	extern void DLDR_Free(struct DLDR_NODEOBJECT *hDldrNode);

	extern DSP_STATUS DLDR_GetFxnAddr(struct DLDR_NODEOBJECT *hDldrNode,
					  char *pstrFxn, u32 *pulAddr);

	extern bool DLDR_Init();
	extern DSP_STATUS DLDR_Load(struct DLDR_NODEOBJECT *hDldrNode,
				    enum DLDR_PHASE phase);
	extern DSP_STATUS DLDR_Unload(struct DLDR_NODEOBJECT *hDldrNode,
				    enum DLDR_PHASE phase);

#endif				/* DLDR_ */
