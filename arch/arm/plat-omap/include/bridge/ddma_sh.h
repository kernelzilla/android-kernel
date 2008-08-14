/*
 * arch/arm/plat-omap/include/bridge/ddma_sh.h
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
 *  ======== ddma_sh.h ========
 *
 *  DSP/BIOS Bridge DSP-DMA(DDMA) shared definitions.
 *  (used on both GPP and DSP sides).
 *
 *
 *! Revision History
 *! ================
 *! 12-Jan-2001 ag  Added Zero-copy channel descriptor
 *!                    and #ifdef CHNL_DDMA for GPP-side build.
 *! 05-Jan-2002 ag  Made chnl buf desc ndxs private.
 *! 07-Dec-2001 ag  DDMA_DSPDESCDONE value changed.
 *! 24-Jan-2001 ag  Initial.
 */

#ifndef DDMA_SH_
#define DDMA_SH_

#include <ddmatypes.h>

/* Max # of DDSP DMA channels allowed */
#define DDMA_MAXZCPYCHNLS   16
#define DDMA_MAXDDMACHNLS   16

/* descriptor status */
#define DDMA_SOK            0x0	/* transfer ok */
#define DDMA_SCANCELED      0x1	/* I/O request canceled */
#define DDMA_SBUFMISMATCH   0x2	/* Src Buf larger than dst. Data clipped */

/* descriptor errors */
#define DDMA_ERRDMA         0x8000  /* DMA error(s) occured */
#define DDMA_ERRILLDESC	    0x800   /* Illegal desc format */
#define DDMA_ERRTIMEOUT	    0x400   /* DMA timeout occured on this descriptor */
#define DDMA_ERRALIGN       0x200   /* Bad SM buffer alignment */

/* cmd codes (workReq field)*/
#define DDMA_WORKREQ        1	/* Channel work requested by Gpp */
#define DDMA_WORKREQCLEAR   2	/* Work requested Cleared by Dsp */
#define DDMA_DSPDESCDONE    2	/* DSP completed DDMA descr proc'ing */

/*
 * DSP-DMA GPP<->DSP structures
 */

/* SM buffer descriptor structure */
struct DDMA_BUFDESC {
	DDMA_DWORD workReq;     /* Requesting work(DSP-DMA) to be performed */
	DDMA_DWORD status;	/* Desc completion status  */
	DDMA_DWORD bufPaDsp;	/* DSP address of SM buffer. */
	DDMA_DWORD xferSize;	/* Size of transfer request in Gpp bytes */
	DDMA_DWORD bufArg;	/* Optional argument associated with buffer */
} ;

/* bridge channel descriptor ctrl */
struct DDMA_CHNLDESC {
	DDMA_WORD numBufDesc;	/* # of SM buf descriptors for this channel */
	DDMA_DWORD descPaDsp;	/* DSP address of base SM descriptor. */
} ;

/*
 * Zero-copy channel descriptor(SM buffer swap)
 */
struct DDMA_ZCPYCHNLDESC {
	/* DSP-Side */
	DDMA_DWORD inBuf;	/* address of input buffer */
	DDMA_DWORD inDataSize;	/* Data size of inBuf  (DSP MAUs) */
	DDMA_DWORD inBufSize;	/* Actual buffer size of inBuf  (DSP MAUs) */
	DDMA_DWORD inUserArg;	/* Optional in user argument */

	DDMA_DWORD outBuf;	/* address of output buffer  */
	DDMA_DWORD outDataSize;	/* Data size of outBuf  (DSP MAUs) */
	DDMA_DWORD outBufSize;	/* Actual buffer size of outBuf  (DSP MAUs) */
	DDMA_DWORD outUserArg;	/* Optional out user argument */
	} ;

#endif				/* DDMA_SH_ */

