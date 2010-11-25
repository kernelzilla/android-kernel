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

#ifndef _WHAL_TX_API_H
#define _WHAL_TX_API_H

#include "memMngrEx.h"
#include "whalCtrl_api.h"
#include "whalBus_Defs.h"

typedef struct
{
	/* call-backs */
	TxCompleteStatusCB_t TxCmplt_CB;

	SendPacketTranferCB_t PacketTranfer_CB;
	
	TI_HANDLE hReport; /* handle to the reporter module*/
	TI_HANDLE hMemMgr; /* handle to the memory manager module*/ 
	TI_HANDLE hWhalCtrl;
} whalTx_config_t;

TI_HANDLE whalTx_Create (TI_HANDLE hOs, TI_HANDLE hWhalCtrl);
TI_STATUS whalTx_Config (TI_HANDLE hWhalTx, whalTx_config_t* pWhalTxCfg);
UINT8 whalTx_GetAcUsedHwBlks(TI_HANDLE hWhalTx, UINT8 AcID);
void whalTx_resetQosCounters(TI_HANDLE hWhalTx);
void whalTx_Register_CB(TI_HANDLE hWhalTx,tiUINT32 CallBackID,void *CBFunc,TI_HANDLE CBObj);






/************************************************************************************************
*****************************   NEW HAL FUNCTIONS AND STRUCTS    ********************************
************************************************************************************************/


TI_STATUS whalTx_SendPacket (TI_HANDLE hWhalTx,
							 const void * aFrame,
							 UINT32       aLength,
							 UINT32       aMaxTransmitRate,
							 UINT32       aQueueId,
							 UINT32       aPowerLevel,
							 BOOL 		  aMore,
							 UINT32       aPacketId,
							 UINT8        aTxRateClassId,
                             UINT32       aExpiryTime,
							 void         *aReserved);




int whalTx_Destroy (TI_HANDLE hWhalTx);

#endif /* _WHAL_TX_API_H*/
