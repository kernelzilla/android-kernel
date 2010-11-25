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

#ifndef _WHAL_ENDPNT_H
#define _WHAL_ENDPNT_H

#include "whalCommon.h"
#include "whalCtrl_api.h"
#include "whalCtrl.h"
#include "memMngrEx.h"

/* CLASS WHAL_ENDPNT*/
typedef enum 
{
    MSDU_READY = 0,
    MSDU_IN_PROGRESS,
    MPDU_DROP,
    MPDU_DUP_DROP,
	MSDU_DROP
} collectStatus_e;

typedef struct 
{
    UINT32          seqNum;    /* The sequence number of the last MSDU received from the station*/ 
    UINT32          fragNum;   /* The fragment number of the last fragment received from the station*/ 
	mem_MSDU_T*     msduPtr;   /* Pointer to the MSDU structure*/ 
	mem_BD_T*		lastBdPtr; /* Pointer to the last fragment in the collection*/ 
	UINT32 			timeStamp; /* A timestamp contains the receive time of the first fragment in the collection,
							      if any*/ 
	BOOL	   		collect;  	/* designates if the station is in a middle of a fragment collection*/
} collectEntry_t;

typedef struct _WHAL_ENDPNT
{
	collectEntry_t collectEntry;

	WHAL_CTRL *pWhalCtrl;       /* Pointer to the HL_HAL control module*/ 

	TI_HANDLE hOs;
	TI_HANDLE hReport;
	TI_HANDLE hMemMngr;

} WHAL_ENDPNT;

typedef struct 
{
	TI_HANDLE hReport; /* handle to the reporter module*/
	TI_HANDLE hMemMngr; /* handle to the memory manager module*/ 
} whalEndpnt_config_t;


/* WHAL ENDPOINT Class API*/			    
TI_HANDLE whalEndpnt_Create (TI_HANDLE hWhalCtrl, TI_HANDLE hOs);

int whalEndpnt_Config (TI_HANDLE hWhalEndpnt, whalEndpnt_config_t* pWhalEndpntCfg);

collectStatus_e	whalEndpnt_FragCollect (TI_HANDLE hWhalEndpnt, mem_MSDU_T* pMpdu, mem_MSDU_T** pMsdu);

BOOL whalEndpnt_IsCollect (TI_HANDLE hWhalEndpnt);

int whalEndpnt_Destroy (TI_HANDLE hWhalEnpnt);
#endif /* _WHAL_ENDPNT_H*/
