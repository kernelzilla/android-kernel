/** \file Core_AdaptTx.h
 *  \brief CORE Adaptation Ctrl include file
 *
 *  \see Core_AdaptTx.c
 */
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

/***************************************************************************/
/*                                                                         */
/*    MODULE:	Core_AdaptTx.h					           */
/*    PURPOSE:	CORE Adaptation TX include file                            */
/*                                                                         */
/***************************************************************************/
#ifndef __CORE_ADAPT_TX_H__
#define __CORE_ADAPT_TX_H__


#include "paramOut.h"
#include "whalBus_Defs.h"
#include "MsduList.h"
#include "DataCtrl_Api.h"

extern TI_HANDLE		CORE_AdaptTx_handle;

/* Callback for tx sendPacketTranfer */
typedef void (* CoreAdapt_SendPacketTranferCB_t)(TI_HANDLE        hTxData,
												 UINT32           pPacketIdAttr);

/* Callback for tx compleate */
typedef void (*CoreAdapt_TxCompleteStatusCB_t)( TI_HANDLE        hCtrlData,           
												txCompleteAttr_t *pTxCompleteAttr);


typedef void (*CoreAdapt_QueueFreeEventCB_t)(TI_HANDLE hTxData,UINT32 *pTxDataHwFreeBuffersPerQ);



typedef struct
{
	TI_HANDLE hReport;
	TI_HANDLE hMemMgr;
	TI_HANDLE hTxData;
	TI_HANDLE hTnetwDrv;
	TI_HANDLE hCtrlData;


} Core_AdaptTx_config_t;


/* Typedefs */
typedef struct
{
	/* handles handled */
	TI_HANDLE hOs;
	TI_HANDLE hReport;
	TI_HANDLE hMemMgr;
	TI_HANDLE hTnetwDrv;
	TI_HANDLE hTxData;
	TI_HANDLE hCtrlData;
} CORE_AdaptTx_t;





/* External data definitions */

/* External functions definitions */

/* Function prototypes */
TI_HANDLE CORE_AdaptTx_Create(TI_HANDLE hOs);
int CORE_AdaptTx_Config(TI_HANDLE CORE_AdaptTx_handle, Core_AdaptTx_config_t *pCore_AdaptTx_config,txDataInitParams_t *txDataInitParams);
int CORE_AdaptTx_Destroy(TI_HANDLE hCore_AdaptTx);
TI_STATUS CORE_AdaptTx_SendPacket(TI_HANDLE hCore_AdaptTx,
							 UINT8 TxQid,
							 mem_MSDU_T* pMsdu,
							 txData_attr_t* pTxAttr,
                             UINT32 packetId,
							 UINT32 msduTimeToExpiry );

/*  callbacks */
void SendPacketComplete (TI_HANDLE hUser,
                         systemStatus_e aStatus,
                        UINT32 aPacketId,
                        UINT32 aRate,
                        UINT8 aAckFailures,
                        UINT32 actualDurationInAir,
                        UINT32 fwHandlingTime,
						UINT32 mediumDelay);

void SendPacketTransfer (TI_HANDLE hUser, UINT32 aPacketId);
void SendPacketDebug    (TI_HANDLE hUser, UINT32 aPacketId, UINT32 uDebugInfo);


#endif /* __CORE_ADAPT_TX_H__*/
