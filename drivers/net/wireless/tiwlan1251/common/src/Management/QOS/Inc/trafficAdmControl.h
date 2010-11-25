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
/*																		   */
/*		MODULE:	admCtrl.c												   */
/*    PURPOSE:	WSM/WME admission Control							   */
/*																	 	   */
/***************************************************************************/

#ifndef _ADM_CTRL_SM_H
#define _ADM_CTRL_SM_H

#include "fsm.h"
#include "qosMngr_API.h"
#include "mlmeApi.h"

#define INITIAL_DIALOG_TOKEN	1

/* state machine states */
typedef enum 
{
	TRAFFIC_ADM_CTRL_SM_STATE_IDLE			= 0,
	TRAFFIC_ADM_CTRL_SM_STATE_WAIT   		= 1,
} trafficAdmCtrl_smState_t;

/* State machine inputs */
typedef enum 
{
	TRAFFIC_ADM_CTRL_SM_EVENT_START			= 0,
	TRAFFIC_ADM_CTRL_SM_EVENT_STOP			= 1,
	TRAFFIC_ADM_CTRL_SM_EVENT_ACCEPT    	= 2,
    TRAFFIC_ADM_CTRL_SM_EVENT_REJECT		= 3,
   	TRAFFIC_ADM_CTRL_SM_EVENT_TIMEOUT		= 4
} trafficAdmCtrl_smEvents_t;


typedef struct
{
	UINT8					tsInfo[3];
	trafficAdmCtrl_smState_t	rafficAdmCtrlSmCurrState;
}tidInfo_t;


typedef struct
{
	TI_HANDLE			        hTxData;
	TI_HANDLE			        hReport;
	TI_HANDLE			        hOs;
	TI_HANDLE			        hQosMngr;
	TI_HANDLE					hCtrlData;
	TI_HANDLE					hMemMgr;
	TI_HANDLE					hExcMgr;

	TI_HANDLE			        timer[MAX_NUM_OF_AC];
	UINT8				        currentState[MAX_NUM_OF_AC];
	UINT8						dialogToken[MAX_NUM_OF_AC];

	UINT8						dialogTokenCounter;


	fsm_stateMachine_t	        *pTrafficAdmCtrlSm;

	UINT32				        timeout;
    BOOL                        useFixedMsduSize;


} trafficAdmCtrl_t;




TI_HANDLE trafficAdmCtrl_create(TI_HANDLE pOs);

TI_STATUS trafficAdmCtrl_unload(TI_HANDLE hTrafficAdmCtrl);

TI_STATUS trafficAdmCtrl_config(TI_HANDLE hTrafficAdmCtrl,
						TI_HANDLE hTxData,
						TI_HANDLE hReport,
						TI_HANDLE hOs,
						TI_HANDLE hQosMngr,
						TI_HANDLE hCtrlData,
						TI_HANDLE hMemMgr,
						TI_HANDLE hExcMgr,
						trafficAdmCtrlInitParams_t	*pTrafficAdmCtrlInitParams);

TI_STATUS trafficAdmCtrl_setParam(TI_HANDLE hTrafficAdmCtrl, paramInfo_t	*pParam);

TI_STATUS trafficAdmCtrl_getParam(TI_HANDLE hTrafficAdmCtrl, paramInfo_t	*pParam);

TI_STATUS trafficAdmCtrl_startAdmRequest(TI_HANDLE	hTrafficAdmCtrl, tspecInfo_t *pTSpecInfo);

TI_STATUS trafficAdmCtrl_stopAdmRequest(TI_HANDLE hTrafficAdmCtrl, UINT8 tid);

TI_STATUS trafficAdmCtrl_stop(TI_HANDLE hTrafficAdmCtrl);

TI_STATUS trafficAdmCtrl_recv(TI_HANDLE hTrafficAdmCtrl, UINT8* pData, UINT8 action);

TI_STATUS trafficAdmCtrl_sendDeltsFrame(TI_HANDLE hTrafficAdmCtrl, tspecInfo_t *pTSpecInfo, UINT8 reasonCode);

void trafficAdmCtrl_print(trafficAdmCtrl_t *pTrafficAdmCtr);

void trafficAdmCtrl_buildTSPec(trafficAdmCtrl_t	*pTrafficAdmCtrl, 
							   tspecInfo_t		*pTSpecInfo, 
							   UINT8				*pDataBuff,
							   UINT32			*len);
void trafficAdmCtrl_parseTspecIE(trafficAdmCtrl_t	*pTrafficAdmCtrl, 
								 tspecInfo_t		*tsInfo, 
								 dot11_WME_TSPEC_IE_t	*dot11_WME_TSPEC_IE);
#endif

