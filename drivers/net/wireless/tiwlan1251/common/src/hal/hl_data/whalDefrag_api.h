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

#ifndef _WHAL_DEFRAG_API_H
#define _WHAL_DEFRAG_API_H

#include "whalCommon.h"
#include "whalCtrl_api.h"
#include "whalCtrl.h"
#include "memMngrEx.h"
#include "whalEndpntEnt_api.h"

#define MAX_SITES_FRAGMENT_COLLECTION		10

/* CLASS WHAL_DEFRAG*/
typedef struct
{
   UINT8 srcMac[MAC_ADDR_LEN];
   BOOL  inUse;
} whalDefrag_entMngr_t;

typedef struct _WHAL_DEFRAG
{
	UINT32	duplicateCnt;
	UINT32	mpduCnt;
	UINT32	msduCnt;
	UINT8   numCollectEntry;
	
	whalDefrag_entMngr_t endpntMngr[MAX_SITES_FRAGMENT_COLLECTION];
	WHAL_ENDPNT* pWhalEndpntEnt[MAX_SITES_FRAGMENT_COLLECTION];     /* Pointer to the HL_HAL endpoint entry module*/

	WHAL_CTRL* hWhalCtrl;       /* Pointer to the HL_HAL control module*/ 
	
	TI_HANDLE hOs;
	TI_HANDLE hReport;
	TI_HANDLE hMemMngr;
} WHAL_DEFRAG;

typedef struct 
{
	TI_HANDLE hReport; /* handle to the reporter module*/
	TI_HANDLE hMemMngr; /* handle to the memory manager module*/ 
} whalDefrag_config_t;

typedef struct 
{
	UINT32	duplicateCnt;
	UINT32	mpduCnt;
	UINT32	msduCnt;
} whalDefrag_counters_t;



/* WHAL DEFRAG Class API*/			    
TI_HANDLE whalDefrag_Create (TI_HANDLE hWhalCtrl, TI_HANDLE hOs, UINT8 numCollectEntries);

int whalDefrag_Config (TI_HANDLE hWhalDefrag, whalDefrag_config_t* pWhalDefragCfg);

collectStatus_e whalDefrag_MpduCollect (TI_HANDLE hWhalDefrag, mem_MSDU_T* pMpdu, mem_MSDU_T** pMsdu);

int whalDefrag_CountersGet (TI_HANDLE hWhalDefrag, whalDefrag_counters_t* pWhalDefragCntr);

int whalDefrag_Destroy (TI_HANDLE hWhalDefrag);
#endif /* _WHAL_DEFRAG_API_H*/
