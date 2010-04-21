/****************************************************************************
**+-----------------------------------------------------------------------+**
**|                                                                       |**
**| Copyright(c) 1998 - 2008 Texas Instruments. All rights reserved.      |**
**| All rights reserved.                                                  |**
**|                                                                       |**
**| Redistribution and use in source and binary forms, with or without    |**
**| modification, are permitted provided that the following conditions    |**
**| are met:                                                              |**
**|                                                                       |**
**|  * Redistributions of source code must retain the above copyright     |**
**|    notice, this list of conditions and the following disclaimer.      |**
**|  * Redistributions in binary form must reproduce the above copyright  |**
**|    notice, this list of conditions and the following disclaimer in    |**
**|    the documentation and/or other materials provided with the         |**
**|    distribution.                                                      |**
**|  * Neither the name Texas Instruments nor the names of its            |**
**|    contributors may be used to endorse or promote products derived    |**
**|    from this software without specific prior written permission.      |**
**|                                                                       |**
**| THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   |**
**| "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     |**
**| LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR |**
**| A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  |**
**| OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, |**
**| SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      |**
**| LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, |**
**| DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY |**
**| THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   |**
**| (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE |**
**| OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  |**
**|                                                                       |**
**+-----------------------------------------------------------------------+**
****************************************************************************/

/****************************************************************************
 *
 *   MODULE:  whalTrace.h
 *   PURPOSE: Trace on Rx/Tx packets
 * 
 ****************************************************************************/

#ifndef _WHAL_TRACE_H
#define _WHAL_TRACE_H

#include "whalCommon.h"
#include "whalHwDefs.h"


#ifndef TIWLN_WINCE30
#define TRACER_MAX_EVENTS		2000
#else
#define TRACER_MAX_EVENTS		10
#endif

#define TRACER_MAX_ACT_LEN		5
#define TRACER_MAX_OBJ_LEN		3
#define TRACER_HDR_LEN			2
#define TRACER_MAX_STR_LEN		70



typedef enum
{
	NUM_FREE_DESCRIPTOR,
	DESCRIPTOR_OCCUPIED,
	CLEAR_DESCRIPTORS,
	FREE_BLOCKS,
} varSetType_e;

typedef struct
{
	UINT32			time;
	UINT16			MpduLen;
	UINT8			Ctl;
	UINT16			Rate;
	UINT8			Status[4];
	dot11_header_t	FrameHeader;
	int				FrameData;
} TRACER_DATA;

typedef struct
{
	char			String[TRACER_MAX_STR_LEN];
	int				IntValue;
} TRACER_CTRL;

typedef struct
{
	UINT32			TimStamp;
	char			Action[TRACER_MAX_ACT_LEN];
	char			Object[TRACER_MAX_OBJ_LEN];

	union
	{
		TRACER_DATA		TrcData;
		TRACER_CTRL		TrcCtrl;
	}Info;

} TRACER_EVENT;

typedef struct _WhalTrace_T
{
	int Enable;
	int Idx;
	int Num;
#ifdef TNETW_MASTER_MODE
	TRACER_EVENT Evt[TRACER_MAX_EVENTS];
#endif
	void *hProtect;
	short traceMask;

	UINT8			MinNumDescriptorFree;
	UINT8			MacOccupiedDescriptor;
	UINT8			MaxClearDescriptor;
	UINT8			MaxFreeBlks;
	UINT8			BugCounter;
	UINT8			reBug;


	TI_HANDLE		hTNETWIF;

	TI_HANDLE hOs;
	TI_HANDLE hReport;
} WhalTrace_T;

extern WhalTrace_T *whal_traceCreate(TI_HANDLE hOs);
extern int whal_traceDestroy(WhalTrace_T *pTrc);
extern int whal_traceConfig(WhalTrace_T *pTrc, TI_HANDLE hTNETWIF, TI_HANDLE hReport);
#ifdef TNETW_MASTER_MODE
extern int  whal_traceAddTx(WhalTrace_T *pTrc, HwTxDesc_T *pHwTxDesc, char *Action);
#endif
extern int  whal_traceIsEnable(WhalTrace_T *pTrc);
extern void whal_traceEnable(WhalTrace_T *pTrc, int val);

#endif
