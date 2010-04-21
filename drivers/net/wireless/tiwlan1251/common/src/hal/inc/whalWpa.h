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

#ifndef _WHAL_WPA_H
#define _WHAL_WPA_H

#include "whalCommon.h"

#define WPA_MIC_FIELD_LEN 	8
#define WPA_TX_ICV_FIELD_SIZE 12 /* 4+8 bytes*/

typedef struct
{
	TI_HANDLE hReport; /* handle to the reporter module*/
	TI_HANDLE hMemMgr; /* handle to the memory manager module*/ 
} whalWpa_config_t;

/* CLASS WHAL_WPA*/
typedef struct _WHAL_WPA
{
	UINT32 wepFailCounter;

	WHAL_CTRL 		*pWhalCtrl;		/* Pointer to the HL_HAL control module*/ 

	TI_HANDLE hOs;
	TI_HANDLE hReport;
	TI_HANDLE hMemMgr;

	keyType_e currTxKeyType;  /* key type of the current configured unicast key */ 
} WHAL_WPA;

/* WHAL WPA Class API*/			    
TI_HANDLE whalWpa_Create (TI_HANDLE hOs, TI_HANDLE hWhalCtrl);

int whalWpa_Config (TI_HANDLE hWhalWep, whalWpa_config_t* pWhalWpaCfg);

int whalWpa_MpduListFieldsAdd (TI_HANDLE hWhalWpa, mem_MSDU_T* pMpduList);  	

int whalWpa_MpduFieldsHandle (TI_HANDLE hWhalWpa, mem_MSDU_T* pMpdu);  	

int whalWpa_KeyAdd (TI_HANDLE hWhalWpa, securityKeys_t* pKey, void *CB_Func, TI_HANDLE CB_handle);

int whalWpa_KeyRemove (TI_HANDLE hWhalWpa, securityKeys_t* pKey, void *CB_Func, TI_HANDLE CB_handle);

int whalWpa_DefaultKeyIdSet (TI_HANDLE hWhalWpa, UINT8 aKeyId, void *CB_Func, TI_HANDLE CB_handle);

int whalWpa_Destroy (TI_HANDLE hWhalWpa);
#endif /* _WHAL_WPA_H*/
