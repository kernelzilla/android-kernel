/*
 * dspbridge/mp_driver/inc/memry.h
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
 *  ======== memry.h ========
 *  Purpose:
 *      Functional interface for the memory manager, exported by the DSP
 *      system API DLL.  This interface is not publicly documented.
 *
 *  Public Functions:
 *      MEMRY_Alloc
 *      MEMRY_BindMem
 *      MEMRY_Calloc
 *      MEMRY_Free
 *      MEMRY_FreeVM
 *      MEMRY_LinearAddress
 *      MEMRY_ReserveVM
 *      MEMRY_PageLock
 *      MEMRY_PageUnlock
 *      MEMRY_UnMapLinearAddress
 *
 *! Revision History:
 *! ================
 *! 01-Sep-2001 ag: Added MEMRY_[UnMap]LinearAddress.
 *! 11-Oct-2000 ag: Added MEMRY_Reserve[Free]VM() & MEMRY_BindMem().
 *! 12-Nov-1999 kc: Updated for WinCE.
 *!
 */

#ifndef MEMRY_
#define MEMRY_

#include <dspapi.h>

#include <memdefs.h>

/*
 *  MEMRY_[GET]SET]VIRTUALSEGID is used by Node & Strm to access virtual
 *  address space in the correct client process context. The virtual to
 *  physical mapping is done in the client process context.
 */
#define MEMRY_SETVIRTUALSEGID   MEM_SETVIRTUALSEGID
#define MEMRY_GETVIRTUALSEGID   MEM_GETVIRTUALSEGID
#define MEMRY_MASKVIRTUALSEGID  MEM_MASKVIRTUALSEGID

/*
 *  ======== MEMRY_LinearAddress ========
 *  Purpose:
 *      Get the linear address corresponding to the given physical address.
 *  Parameters:
 *      pPhysAddr:      Physical address to be mapped.
 *      cBytes:         Number of bytes in physical range to map.
 *  Returns:
 *      The corresponding linear address, or NULL if unsuccessful.
 *  Requires:
 *     PhysAddr != 0
 *  Ensures:
 *  Notes:
 *      If valid linear address is returned, be sure to call
 *      MEMRY_UnMapLinearAddress().
 */
	extern inline void *MEMRY_LinearAddress(void *pPhyAddr, u32 cBytes)
	{
		return pPhyAddr;
	}

/*
 *  ======== MEMRY_UnMapLinearAddress ========
 *  Purpose:
 *      Unmap the linear address mapped in MEMRY_LinearAddress.
 *  Parameters:
 *      pBaseAddr:  Ptr to mapped memory (as returned by MEMRY_LinearAddress()).
 *  Returns:
 *  Requires:
 *      - pBaseAddr is a valid linear address mapped in MEMRY_LinearAddress.
 *  Ensures:
 *      - pBaseAddr no longer points to a valid linear address.
 */
	extern inline void MEMRY_UnMapLinearAddress(void *pBaseAddr)
	{
	}

#endif				/* MEMRY_ */
