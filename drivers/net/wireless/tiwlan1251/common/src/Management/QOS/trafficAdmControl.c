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
/*	  MODULE:	admCtrlQos.c											   */
/*    PURPOSE:	WSM/WME admission Control							       */
/*																	 	   */
/***************************************************************************/

#include "osApi.h"

#include "paramOut.h"

#include "utils.h"
#include "fsm.h"
#include "report.h"

#include "DataCtrl_Api.h"

#include "trafficAdmControl.h"
#include "qosMngr_API.h"
#include "TNETW_Driver_types.h"
#ifdef EXC_MODULE_INCLUDED
#include "excMngr.h"
#endif
/* Constants */

/** number of states in the state machine */
#define	TRAFFIC_ADM_CTRL_SM_NUM_STATES		2

/** number of events in the state machine */
#define	TRAFFIC_ADM_CTRL_SM_NUM_EVENTS			5

extern int WMEQosTagToACTable[MAX_NUM_OF_802_1d_TAGS];

PACKED_STRUCT( dot11_local_WME_TSPEC_IE_t,

    UINT16	nominalMSDUSize;
	UINT16	maximumMSDUSize;
	UINT32	minimumServiceInterval;
	UINT32	maximumServiceInterval;
	UINT32	inactivityInterval;
	UINT32	suspensionInterval;
	UINT32	serviceStartTime;
	UINT32	minimumDataRate;
	UINT32	meanDataRate;
	UINT32	peakDataRate;
	UINT32	maximumBurstSize;
	UINT32	delayBound;
	UINT32	minimumPHYRate;
	UINT16	surplusBandwidthAllowance;
	UINT16	mediumTime;
);

typedef struct 
{
	TI_HANDLE hTrafficAdmCtrl;
	tspecInfo_t *pTSpecInfo;
	UINT8		acID;

}fsmTSpecInfo_t;


/* Timer functions */
void trafficAdmCtrl_timeoutAcBE(TI_HANDLE hTrafficAdmCtrl);
void trafficAdmCtrl_timeoutAcBK(TI_HANDLE hTrafficAdmCtrl);
void trafficAdmCtrl_timeoutAcVI(TI_HANDLE hTrafficAdmCtrl);
void trafficAdmCtrl_timeoutAcVO(TI_HANDLE hTrafficAdmCtrl);


/* SM Functions */
TI_STATUS trafficAdmCtrl_smEvent(trafficAdmCtrl_t *pAdmCtrlQos, UINT8 event, void *pData);

TI_STATUS trafficAdmCtrl_smActionUnexpectedTspecResponse(fsmTSpecInfo_t *fsmTSpecInfo);	/*unxcepted*/
TI_STATUS trafficAdmCtrl_smActionUnexpected(fsmTSpecInfo_t *fsmTSpecInfo);	/*unxcepted*/
TI_STATUS trafficAdmCtrl_smActionNop(fsmTSpecInfo_t *fsmTSpecInfo);			/*NOP*/
TI_STATUS trafficAdmCtrl_smStart(fsmTSpecInfo_t *fsmTSpecInfo);				/*EVENT_START*/
TI_STATUS trafficAdmCtrl_smWaitStop(fsmTSpecInfo_t *fsmTSpecInfo);			/*EVENT_STOP*/
TI_STATUS trafficAdmCtrl_smWaitAccept(fsmTSpecInfo_t *fsmTSpecInfo);		/*EVENT_ACCEPT*/
TI_STATUS trafficAdmCtrl_smWaitReject(fsmTSpecInfo_t *fsmTSpecInfo);		/*EVENT_REJECT*/
TI_STATUS trafficAdmCtrl_smWaitTimeout(fsmTSpecInfo_t *fsmTSpecInfo);		/*EVENT_TIMEOUT*/



TI_STATUS trafficAdmCtrl_sendAdmissionReq(TI_HANDLE hTrafficAdmCtrl, tspecInfo_t *pTSpecInfo);
TI_STATUS trafficAdmCtrl_startTimer(trafficAdmCtrl_t* pTrafficAdmCtrl, UINT8 acID);
TI_STATUS trafficAdmCtrl_stopTimer(trafficAdmCtrl_t* pTrafficAdmCtrl, UINT8 acID);


TI_STATUS trafficAdmCtrl_buildFrameHeader(trafficAdmCtrl_t *hprafficAdmCtrl, mem_MSDU_T	*pMsdu);

static TI_STATUS trafficAdmCtrl_tokenToAc (TI_HANDLE hTrafficAdmCtrl, UINT8 token, UINT8 *acID);

/********************************************************************************
 *							trafficAdmCtrl_create								*
 ********************************************************************************
DESCRIPTION: trafficAdmCtrl module creation function

  INPUT:      hOs -			Handle to OS		


OUTPUT:		

RETURN:     Handle to the trafficAdmCtrl module on success, NULL otherwise

************************************************************************/

TI_HANDLE trafficAdmCtrl_create(TI_HANDLE hOs)
{
	trafficAdmCtrl_t 		*pTrafficAdmCtrl;
	TI_STATUS			status;

	/* allocate admission control context memory */
	pTrafficAdmCtrl = (trafficAdmCtrl_t*)os_memoryAlloc(hOs, sizeof(trafficAdmCtrl_t));
	if (pTrafficAdmCtrl == NULL)
	{
		return NULL;
	}

	os_memoryZero(hOs, pTrafficAdmCtrl, sizeof(trafficAdmCtrl_t));

	pTrafficAdmCtrl->hOs = hOs;

	/* allocate memory for admCtrlQos state machine */
	status = fsm_Create(hOs, &pTrafficAdmCtrl->pTrafficAdmCtrlSm, TRAFFIC_ADM_CTRL_SM_NUM_STATES, TRAFFIC_ADM_CTRL_SM_NUM_EVENTS);
	if (status != OK)
	{
		os_memoryFree(hOs, pTrafficAdmCtrl, sizeof(trafficAdmCtrl_t));
		return NULL;
	}

	/* allocate OS timer AC BE */
	pTrafficAdmCtrl->timer[QOS_AC_BE] = os_timerCreate(hOs, trafficAdmCtrl_timeoutAcBE, pTrafficAdmCtrl);
	if (pTrafficAdmCtrl->timer[QOS_AC_BE] == NULL)
	{
		fsm_Unload(hOs, pTrafficAdmCtrl->pTrafficAdmCtrlSm);
		os_memoryFree(hOs, pTrafficAdmCtrl, sizeof(trafficAdmCtrl_t));		
		return NULL;
	}
	/* allocate OS timer AC BK */
	pTrafficAdmCtrl->timer[QOS_AC_BK] = os_timerCreate(hOs, trafficAdmCtrl_timeoutAcBK, pTrafficAdmCtrl);
	if (pTrafficAdmCtrl->timer[QOS_AC_BK] == NULL)
	{
		fsm_Unload(hOs, pTrafficAdmCtrl->pTrafficAdmCtrlSm);
		os_timerDestroy(pTrafficAdmCtrl->hOs, pTrafficAdmCtrl->timer[QOS_AC_BE]);
		os_memoryFree(hOs, pTrafficAdmCtrl, sizeof(trafficAdmCtrl_t));		
		return NULL;
	}
	/* allocate OS timer AC VI */
	pTrafficAdmCtrl->timer[QOS_AC_VI] = os_timerCreate(hOs, trafficAdmCtrl_timeoutAcVI, pTrafficAdmCtrl);
	if (pTrafficAdmCtrl->timer[QOS_AC_VI] == NULL)
	{
		fsm_Unload(hOs, pTrafficAdmCtrl->pTrafficAdmCtrlSm);
		os_timerDestroy(pTrafficAdmCtrl->hOs, pTrafficAdmCtrl->timer[QOS_AC_BE]);
		os_timerDestroy(pTrafficAdmCtrl->hOs, pTrafficAdmCtrl->timer[QOS_AC_BK]);
		os_memoryFree(hOs, pTrafficAdmCtrl, sizeof(trafficAdmCtrl_t));		
		return NULL;
	}
	/* allocate OS timer AC VO   */
	pTrafficAdmCtrl->timer[QOS_AC_VO] = os_timerCreate(hOs, trafficAdmCtrl_timeoutAcVO, pTrafficAdmCtrl);
	if (pTrafficAdmCtrl->timer[QOS_AC_VO] == NULL)
	{
		fsm_Unload(hOs, pTrafficAdmCtrl->pTrafficAdmCtrlSm);
		os_timerDestroy(pTrafficAdmCtrl->hOs, pTrafficAdmCtrl->timer[QOS_AC_BE]);
		os_timerDestroy(pTrafficAdmCtrl->hOs, pTrafficAdmCtrl->timer[QOS_AC_BK]);
		os_timerDestroy(pTrafficAdmCtrl->hOs, pTrafficAdmCtrl->timer[QOS_AC_VI]);
		os_memoryFree(hOs, pTrafficAdmCtrl, sizeof(trafficAdmCtrl_t));		
		return NULL;
	}

	return pTrafficAdmCtrl;
}
/************************************************************************
 *                        trafficAdmCtrl_unload						    *
 ************************************************************************
DESCRIPTION: trafficAdmCtrl module destroy function, 
				-	Free all memory alocated by the module
				
INPUT:      hTrafficAdmCtrl	-	trafficAdmCtrl handle.		

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS trafficAdmCtrl_unload(TI_HANDLE hTrafficAdmCtrl)
{
    TI_STATUS 				status;
	trafficAdmCtrl_t		*pTrafficAdmCtrl;

	pTrafficAdmCtrl = (trafficAdmCtrl_t*)hTrafficAdmCtrl;

	status = fsm_Unload(pTrafficAdmCtrl->hOs, pTrafficAdmCtrl->pTrafficAdmCtrlSm);
    if (status != OK)
	{
		/* report failure but don't stop... */
		WLAN_REPORT_ERROR(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
				  ("ADM_CTRL_SM: Error releasing FSM memory \n"));
	}
	
	/* free timers */
	os_timerDestroy(pTrafficAdmCtrl->hOs, pTrafficAdmCtrl->timer[QOS_AC_BE]);
	os_timerDestroy(pTrafficAdmCtrl->hOs, pTrafficAdmCtrl->timer[QOS_AC_BK]);
	os_timerDestroy(pTrafficAdmCtrl->hOs, pTrafficAdmCtrl->timer[QOS_AC_VI]);
	os_timerDestroy(pTrafficAdmCtrl->hOs, pTrafficAdmCtrl->timer[QOS_AC_VO]);
	
	os_memoryFree(pTrafficAdmCtrl->hOs, pTrafficAdmCtrl, sizeof(trafficAdmCtrl_t));

	return OK;
}

/************************************************************************
 *                        trafficAdmCtrl_config							*
 ************************************************************************
DESCRIPTION: trafficAdmCtrl module configuration function, 
				performs the following:
				-	Reset & initiailzes local variables
				-	Init the handles to be used by the module
                                                                                                   
INPUT:      hTrafficAdmCtrl	         -	trafficAdmCtrl handle.
		    List of handles to be used by the module
			pTrafficAdmCtrlInitParams	-	init parameters.		
			
OUTPUT:		

RETURN:     OK on success, NOK otherwise
************************************************************************/

TI_STATUS trafficAdmCtrl_config(TI_HANDLE hTrafficAdmCtrl,
							TI_HANDLE hTxData,
							TI_HANDLE hReport,
							TI_HANDLE hOs,
							TI_HANDLE hQosMngr,
							TI_HANDLE hCtrlData,
							TI_HANDLE hMemMgr,
							TI_HANDLE hExcMgr,
							trafficAdmCtrlInitParams_t	*pTrafficAdmCtrlInitParams)
{
	trafficAdmCtrl_t	*pTrafficAdmCtrl;
	TI_STATUS			status;
	UINT8       		idx;

	fsm_actionCell_t	trafficAdmCtrl_smMatrix[TRAFFIC_ADM_CTRL_SM_NUM_STATES][TRAFFIC_ADM_CTRL_SM_NUM_EVENTS] =
	{
		/* next state and actions for IDLE state */
		{{TRAFFIC_ADM_CTRL_SM_STATE_WAIT, (fsm_Action_t)trafficAdmCtrl_smStart},			/*EVENT_START*/
		 {TRAFFIC_ADM_CTRL_SM_STATE_IDLE, (fsm_Action_t)trafficAdmCtrl_smActionNop},		/*EVENT_STOP*/
		 {TRAFFIC_ADM_CTRL_SM_STATE_IDLE, (fsm_Action_t)trafficAdmCtrl_smActionUnexpectedTspecResponse}, /*EVENT_ACCEPT*/
		 {TRAFFIC_ADM_CTRL_SM_STATE_IDLE, (fsm_Action_t)trafficAdmCtrl_smActionUnexpectedTspecResponse}, /*EVENT_REJECT*/
		 {TRAFFIC_ADM_CTRL_SM_STATE_IDLE, (fsm_Action_t)trafficAdmCtrl_smActionUnexpected}, /*EVENT_TIMEOUT*/
		},
		/* next state and actions for WAIT state */
		{{TRAFFIC_ADM_CTRL_SM_STATE_WAIT, (fsm_Action_t)trafficAdmCtrl_smActionUnexpected},	/*EVENT_START*/
		 {TRAFFIC_ADM_CTRL_SM_STATE_IDLE, (fsm_Action_t)trafficAdmCtrl_smWaitStop},			/*EVENT_STOP*/
		 {TRAFFIC_ADM_CTRL_SM_STATE_IDLE, (fsm_Action_t)trafficAdmCtrl_smWaitAccept},		/*EVENT_ACCEPT*/
		 {TRAFFIC_ADM_CTRL_SM_STATE_IDLE, (fsm_Action_t)trafficAdmCtrl_smWaitReject},		/*EVENT_REJECT*/
		 {TRAFFIC_ADM_CTRL_SM_STATE_IDLE, (fsm_Action_t)trafficAdmCtrl_smWaitTimeout},		/*EVENT_TIMEOUT*/
		},
	};
	
	pTrafficAdmCtrl = (trafficAdmCtrl_t*)hTrafficAdmCtrl;

	pTrafficAdmCtrl->hTxData	= hTxData;
	pTrafficAdmCtrl->hReport	= hReport;
	pTrafficAdmCtrl->hOs		= hOs;
	pTrafficAdmCtrl->hQosMngr	= hQosMngr;
	pTrafficAdmCtrl->hCtrlData	= hCtrlData;
	pTrafficAdmCtrl->hMemMgr	= hMemMgr;
	pTrafficAdmCtrl->hExcMgr	= hExcMgr;

	
	/* configure state machine */
	status = fsm_Config(pTrafficAdmCtrl->pTrafficAdmCtrlSm, &trafficAdmCtrl_smMatrix[0][0], 
						TRAFFIC_ADM_CTRL_SM_NUM_STATES, TRAFFIC_ADM_CTRL_SM_NUM_EVENTS, NULL, hOs);
	if (status != OK)
	{
		WLAN_REPORT_ERROR(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
				  ("TRAFFIC_ADM_CTRL_SM: fsm_Config - Error  \n"));

		return NOK;
	}

	pTrafficAdmCtrl->timeout =  pTrafficAdmCtrlInitParams->trafficAdmCtrlResponseTimeout;
    pTrafficAdmCtrl->useFixedMsduSize = pTrafficAdmCtrlInitParams->trafficAdmCtrlUseFixedMsduSize;

	pTrafficAdmCtrl->dialogTokenCounter = INITIAL_DIALOG_TOKEN;
	

	for (idx=0; idx<MAX_NUM_OF_AC; idx++)
		pTrafficAdmCtrl->dialogToken[idx] = 0;
		

	WLAN_REPORT_INFORMATION(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
				  ("TRAFFIC ADM CTRL -  configuration completed ..... \n"));

	return OK;
}

/************************************************************************
 *                        FOR SM PRINTINGS								*
 ************************************************************************/

#ifdef REPORT_LOG

static char *admCtrlQosSMStateDesc[TRAFFIC_ADM_CTRL_SM_NUM_STATES] = {	
		"TRAFFIC_ADM_CTRL_STATE_IDLE",
		"TRAFFIC_ADM_CTRL_STATE_WAIT",
	};
	
/* State machine inputs */
static char *admCtrlQosSMEventDesc[TRAFFIC_ADM_CTRL_SM_NUM_EVENTS] = {
		"TRAFFIC_ADM_CTRL_EVENT_START",
		"TRAFFIC_ADM_CTRL_EVENT_STOP",
		"TRAFFIC_ADM_CTRL_EVENT_ACCEPT",
		"TRAFFIC_ADM_CTRL_EVENT_REJECT",
		"TRAFFIC_ADM_CTRL_EVENT_TIMEOUT",
	};

#endif

/************************************************************************
 *                        trafficAdmCtrl_smEvent						*
 ************************************************************************
DESCRIPTION: trafficAdmCtrl SM general function
                                                                                                   
INPUT:      pTrafficAdmCtrl	    -	trafficAdmCtr handle.
			event				-	the event to the SM.
			pData				-	handle to passing parameter			
OUTPUT:		

RETURN:     OK on success, NOK otherwise
************************************************************************/

TI_STATUS trafficAdmCtrl_smEvent(trafficAdmCtrl_t *pTrafficAdmCtrl, UINT8 event, void *pData)
{
	TI_STATUS 		status;
	UINT8			nextState;
	fsmTSpecInfo_t	*fsmTSpecInfo = (fsmTSpecInfo_t*)pData;
	UINT8			acID = fsmTSpecInfo->acID;

	status = fsm_GetNextState(pTrafficAdmCtrl->pTrafficAdmCtrlSm, pTrafficAdmCtrl->currentState[acID], event, &nextState);
	if (status != OK)
	{
		WLAN_REPORT_ERROR(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
						  ("ADM_CTRL: ERROR - failed getting next state \n"));

		return(NOK);
	}

	WLAN_REPORT_SM(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
					 ("ADM_CTRL acID = %d: <%s, %s> --> %s\n",acID,
					  admCtrlQosSMStateDesc[pTrafficAdmCtrl->currentState[acID] ],
					  admCtrlQosSMEventDesc[event],
					  admCtrlQosSMStateDesc[nextState]));

	status = fsm_Event(pTrafficAdmCtrl->pTrafficAdmCtrlSm, &pTrafficAdmCtrl->currentState[acID], event, pData);

	return(status);
}


/************************************************************************
*							state machine functions						*
************************************************************************/
/************************************************************************
 *                        trafficAdmCtrl_smStart						*
 ************************************************************************
DESCRIPTION: the action function when event start ocuured on idle state 
				performs the following:
				-	send admision requestReset 
				-	start timer for the response.
                                                                                                   
INPUT:      fsmTSpecInfo - parameters for the request		
			
OUTPUT:		

RETURN:     OK on success, NOK otherwise
************************************************************************/

TI_STATUS trafficAdmCtrl_smStart(fsmTSpecInfo_t *fsmTSpecInfo)		
{
	TI_STATUS				status;
	trafficAdmCtrl_t		*pTrafficAdmCtrl;
	tspecInfo_t				*pTSpecInfo;

	pTrafficAdmCtrl = (trafficAdmCtrl_t*)(fsmTSpecInfo->hTrafficAdmCtrl);
	pTSpecInfo = fsmTSpecInfo->pTSpecInfo;

	/* send adm request frame */
	status = trafficAdmCtrl_sendAdmissionReq(pTrafficAdmCtrl, pTSpecInfo);
	if(status != OK)
		return status;

	/* init timer */
	trafficAdmCtrl_startTimer(pTrafficAdmCtrl, pTSpecInfo->AC);

	return OK;
}
/************************************************************************
 *                        trafficAdmCtrl_smWaitStop						*
 ************************************************************************
DESCRIPTION: the action function when event stop ocuured on wait state 
				performs the following:
				-	stop timer.
                                                                                                   
INPUT:      fsmTSpecInfo - parameters of the request		
			
OUTPUT:		

RETURN:     OK on success, NOK otherwise
************************************************************************/

TI_STATUS trafficAdmCtrl_smWaitStop(fsmTSpecInfo_t *fsmTSpecInfo)
{
	trafficAdmCtrl_t		*pTrafficAdmCtrl;
	tspecInfo_t				*pTSpecInfo;

	pTrafficAdmCtrl = (trafficAdmCtrl_t*)(fsmTSpecInfo->hTrafficAdmCtrl);
	pTSpecInfo = fsmTSpecInfo->pTSpecInfo;

	/* stop timer */
	trafficAdmCtrl_stopTimer(pTrafficAdmCtrl,fsmTSpecInfo->pTSpecInfo->AC);

	WLAN_REPORT_INFORMATION(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
				  ("TRAFFIC ADM CTRL -  AC = %d,    Stoped ..... \n", pTSpecInfo->AC));

	
	return OK;
}
/************************************************************************
 *                        trafficAdmCtrl_smWaitAccept					*
 ************************************************************************
DESCRIPTION: the action function when event accept ocuured on wait state 
				performs the following:
				-	update the Qos Mngr of the status and the parameters
                                                                                                   
INPUT:      fsmTSpecInfo - parameters of the response		
			
OUTPUT:		

RETURN:     OK on success, NOK otherwise
************************************************************************/

TI_STATUS trafficAdmCtrl_smWaitAccept(fsmTSpecInfo_t *fsmTSpecInfo)
{
	trafficAdmCtrl_t		*pTrafficAdmCtrl;
	tspecInfo_t				*pTSpecInfo;

	pTrafficAdmCtrl = (trafficAdmCtrl_t*)(fsmTSpecInfo->hTrafficAdmCtrl);
	pTSpecInfo = fsmTSpecInfo->pTSpecInfo;

	/* update the QosMngr */
	qosMngr_setAdmissionInfo(pTrafficAdmCtrl->hQosMngr, pTSpecInfo->AC, pTSpecInfo, STATUS_TRAFFIC_ADM_REQUEST_ACCEPT);

	return OK;
}
/************************************************************************
 *                        trafficAdmCtrl_smWaitReject					*
 ************************************************************************
DESCRIPTION: the action function when event reject ocuured on wait state 
				performs the following:
				-	update the Qos Mngr of the status and the parameters
                                                                                                   
INPUT:      fsmTSpecInfo - parameters of the response		
			
OUTPUT:		

RETURN:     OK on success, NOK otherwise
************************************************************************/
TI_STATUS trafficAdmCtrl_smWaitReject(fsmTSpecInfo_t *fsmTSpecInfo)
{
	trafficAdmCtrl_t		*pTrafficAdmCtrl;
	tspecInfo_t				*pTSpecInfo;

	pTrafficAdmCtrl = (trafficAdmCtrl_t*)(fsmTSpecInfo->hTrafficAdmCtrl);
	pTSpecInfo = fsmTSpecInfo->pTSpecInfo;

	/* update the QosMngr */
	qosMngr_setAdmissionInfo(pTrafficAdmCtrl->hQosMngr, pTSpecInfo->AC, pTSpecInfo,	STATUS_TRAFFIC_ADM_REQUEST_REJECT);

	return OK;
}
/************************************************************************
 *                        trafficAdmCtrl_smWaitTimeout					*
 ************************************************************************
DESCRIPTION: the action function when event timeout ocuured on wait state 
				performs the following:
				-	update the Qos Mngr of the status and the parameters
                                                                                                   
INPUT:      fsmTSpecInfo - parameters of the request		
			
OUTPUT:		

RETURN:     OK on success, NOK otherwise
************************************************************************/

TI_STATUS trafficAdmCtrl_smWaitTimeout(fsmTSpecInfo_t *fsmTSpecInfo)
{
	trafficAdmCtrl_t		*pTrafficAdmCtrl;

	pTrafficAdmCtrl = (trafficAdmCtrl_t*)(fsmTSpecInfo->hTrafficAdmCtrl);

	/* update the QosMngr */
	qosMngr_setAdmissionInfo(pTrafficAdmCtrl->hQosMngr, fsmTSpecInfo->acID, NULL, STATUS_TRAFFIC_ADM_REQUEST_TIMEOUT);

	return OK;
}
/************************************************************************
 *               trafficAdmCtrl_smActionUnexpected						*
 ************************************************************************
DESCRIPTION:                                                 
INPUT:      fsmTSpecInfo - tspec parameters 	
			
OUTPUT:		

RETURN:     OK on success, NOK otherwise
************************************************************************/

TI_STATUS trafficAdmCtrl_smActionUnexpected(fsmTSpecInfo_t *fsmTSpecInfo)
{
	trafficAdmCtrl_t		*pTrafficAdmCtrl;

	pTrafficAdmCtrl = (trafficAdmCtrl_t*)(fsmTSpecInfo->hTrafficAdmCtrl);

	WLAN_REPORT_ERROR(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
				  ("TRAFFIC ADM CTRL -  AC = %d,    ActionUnexpected ..... \n", fsmTSpecInfo->acID));

	return OK;
}

/************************************************************************
 *               trafficAdmCtrl_smActionUnexpectedTspecResponse			*
 ************************************************************************
DESCRIPTION:                                                 
INPUT:      fsmTSpecInfo - tspec parameters 	
OUTPUT:		
RETURN:     OK on success, NOK otherwise
************************************************************************/

TI_STATUS trafficAdmCtrl_smActionUnexpectedTspecResponse(fsmTSpecInfo_t *fsmTSpecInfo)
{
	trafficAdmCtrl_t		*pTrafficAdmCtrl;
	tspecInfo_t				*pTSpecInfo;

	pTrafficAdmCtrl = (trafficAdmCtrl_t*)(fsmTSpecInfo->hTrafficAdmCtrl);
	pTSpecInfo = fsmTSpecInfo->pTSpecInfo;

	/* Send event to user application - how come TSPEC response arrives without request ? */
	qosMngr_sendUnexpectedTSPECResponseEvent (pTrafficAdmCtrl->hQosMngr,pTSpecInfo);

	WLAN_REPORT_WARNING(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
				  ("TRAFFIC ADM CTRL -  AC = %d,    ActionUnexpected ..... \n", fsmTSpecInfo->acID));

	return OK;
}


/************************************************************************
 *                        trafficAdmCtrl_smActionNop					*
 ************************************************************************
DESCRIPTION:                                                 
INPUT:      fsmTSpecInfo - tspec parameters 	
			
OUTPUT:		

RETURN:     OK on success, NOK otherwise
************************************************************************/

TI_STATUS trafficAdmCtrl_smActionNop(fsmTSpecInfo_t *fsmTSpecInfo)
{
	trafficAdmCtrl_t		*pTrafficAdmCtrl;
	tspecInfo_t				*pTSpecInfo;

	pTrafficAdmCtrl = (trafficAdmCtrl_t*)(fsmTSpecInfo->hTrafficAdmCtrl);
	pTSpecInfo = fsmTSpecInfo->pTSpecInfo;

	WLAN_REPORT_INFORMATION(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
				  ("TRAFFIC ADM CTRL -  AC = %d,    Action NOP..... \n", pTSpecInfo->AC));

	return OK;
}
/************************************************************************
 *							API FUNCTIONS						        *
 ************************************************************************
 ************************************************************************/

/************************************************************************
 *                    trafficAdmCtrl_startAdmRequest                    *
 ************************************************************************
DESCRIPTION: start TSPEC signaling
                                                                                                   
INPUT:      pTrafficAdmCtrl	    -	trafficAdmCtr handle.
			pTSpecInfo			-	the TSPEC parameters
	
OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

TI_STATUS trafficAdmCtrl_startAdmRequest(TI_HANDLE	hTrafficAdmCtrl, tspecInfo_t *pTSpecInfo)
{
	TI_STATUS			status;
	fsmTSpecInfo_t		fsmTSpecInfo;

	trafficAdmCtrl_t *pTrafficAdmCtrl = (trafficAdmCtrl_t*)hTrafficAdmCtrl;

	if (pTrafficAdmCtrl == NULL)
		return NOK;

	fsmTSpecInfo.hTrafficAdmCtrl = hTrafficAdmCtrl;
	fsmTSpecInfo.pTSpecInfo = pTSpecInfo;
	fsmTSpecInfo.acID = pTSpecInfo->AC;

	/* send event START to SM */
	status = trafficAdmCtrl_smEvent(pTrafficAdmCtrl, TRAFFIC_ADM_CTRL_SM_EVENT_START, &fsmTSpecInfo);

	return status;

}
/************************************************************************
 *                    trafficAdmCtrl_stopAdmRequest                     *
 ************************************************************************
DESCRIPTION: stop specific tspec signaling
                                                                                                   
INPUT:      pTrafficAdmCtrl	    -	trafficAdmCtr handle.
			acID				-	the AC of the tspec to stop
	
OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

TI_STATUS trafficAdmCtrl_stopAdmRequest(TI_HANDLE hTrafficAdmCtrl, UINT8 acID)
{
	TI_STATUS			status;
	trafficAdmCtrl_t	*pTrafficAdmCtrl = (trafficAdmCtrl_t*)hTrafficAdmCtrl;
	
	tspecInfo_t			pTSpecInfo;
	fsmTSpecInfo_t		fsmTSpecInfo;

	fsmTSpecInfo.hTrafficAdmCtrl = hTrafficAdmCtrl;
	fsmTSpecInfo.pTSpecInfo = &pTSpecInfo;

	fsmTSpecInfo.pTSpecInfo->AC = (acTrfcType_e)acID;
	fsmTSpecInfo.acID = acID;

	/* send event STOP to SM */
	status = trafficAdmCtrl_smEvent(pTrafficAdmCtrl, TRAFFIC_ADM_CTRL_SM_EVENT_STOP, &fsmTSpecInfo);
	
	return status;

}
/************************************************************************
 *                    trafficAdmCtrl_stop			                     *
 ************************************************************************
DESCRIPTION: stop all tspecs and reset SM  
			called on disconnect
                                                                                                 
INPUT:      pTrafficAdmCtrl	    -	trafficAdmCtr handle.
	
OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS trafficAdmCtrl_stop(TI_HANDLE	hTrafficAdmCtrl)
{
	UINT8				acID;
	UINT8				idx;

	trafficAdmCtrl_t	*pTrafficAdmCtrl = (trafficAdmCtrl_t*)hTrafficAdmCtrl;
	
	tspecInfo_t			pTSpecInfo;
	fsmTSpecInfo_t		fsmTSpecInfo;

	fsmTSpecInfo.hTrafficAdmCtrl = hTrafficAdmCtrl;
	fsmTSpecInfo.pTSpecInfo = &pTSpecInfo;

	/* clean all AC SM  */
	for (acID = 0 ; acID < MAX_NUM_OF_AC ; acID++)
	{
		fsmTSpecInfo.pTSpecInfo->AC = (acTrfcType_e)acID;
		fsmTSpecInfo.acID = acID;
		trafficAdmCtrl_smEvent(pTrafficAdmCtrl, TRAFFIC_ADM_CTRL_SM_EVENT_STOP, &fsmTSpecInfo);
	}

	pTrafficAdmCtrl->dialogTokenCounter = INITIAL_DIALOG_TOKEN;
	
	for (idx=0; idx<MAX_NUM_OF_AC; idx++)
		pTrafficAdmCtrl->dialogToken[idx] = 0;

	return OK;
}

/************************************************************************
 *                    trafficAdmCtrl_recv			                     *
 ************************************************************************
DESCRIPTION: 
                                                                                                 
INPUT:      pTrafficAdmCtrl	    -	trafficAdmCtr handle.
	
OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

TI_STATUS trafficAdmCtrl_recv(TI_HANDLE hTrafficAdmCtrl, UINT8* pData, UINT8 action)
{
	TI_STATUS 			status = OK;
	UINT8				statusCode;
	UINT8				dialogToken;
	UINT8				tacID;
	tspecInfo_t			tspecInfo;
	fsmTSpecInfo_t		fsmTSpecInfo;

	trafficAdmCtrl_t *pTrafficAdmCtrl = (trafficAdmCtrl_t*)hTrafficAdmCtrl;

	if (action == ADDTS_RESPONSE_ACTION) 
	{
		WLAN_REPORT_INFORMATION(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
								("action = 1 - ADDTS RESPONSE ACTION........!! \n"));

		/* parsing the dialog token */
		dialogToken = *pData;
		pData++;
		
		/* in WME status code is 1 byte, in WSM is 2 bytes */
		statusCode = *pData;
		pData++;

		tspecInfo.statusCode = statusCode;

		WLAN_REPORT_INFORMATION(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
					("dialogToken = %d ,  statusCode = %d \n",dialogToken, statusCode));

		trafficAdmCtrl_parseTspecIE(pTrafficAdmCtrl, &tspecInfo, (dot11_WME_TSPEC_IE_t *)pData);

		if (trafficAdmCtrl_tokenToAc (pTrafficAdmCtrl, dialogToken, &tacID) == NOK)
		{
			WLAN_REPORT_WARNING(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
					("dialog token Not found,  dialogToken = %d , \n",dialogToken));
			
			qosMngr_sendUnexpectedTSPECResponseEvent(pTrafficAdmCtrl->hQosMngr, &tspecInfo);
		
			return NOK;
		}
		
		/* validate dialog token matching */
		if(pTrafficAdmCtrl->dialogToken[tspecInfo.AC] != dialogToken)
		{
			WLAN_REPORT_WARNING(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
					("dialog token mismatch,  dialogToken = %d ,  acID = %d \n",dialogToken, tspecInfo.AC));
			
			qosMngr_sendUnexpectedTSPECResponseEvent(pTrafficAdmCtrl->hQosMngr, &tspecInfo);
		
			return NOK;
		}

		/* Stop the relevant Timer */
		trafficAdmCtrl_stopTimer(pTrafficAdmCtrl, tspecInfo.AC);

		fsmTSpecInfo.hTrafficAdmCtrl = hTrafficAdmCtrl;
		fsmTSpecInfo.pTSpecInfo = &tspecInfo;

		fsmTSpecInfo.acID = tspecInfo.AC;
		
		if(statusCode != ADDTS_STATUS_CODE_SUCCESS)
		{
			/* admission reject */
			/********************/
			WLAN_REPORT_INFORMATION(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
					("***** admCtrlQos_recv: admission reject [ statusCode = %d ]\n",statusCode));
			
			
			WLAN_REPORT_INFORMATION(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
					("ADDTS Response (reject) userPriority = %d , \n", tspecInfo.userPriority));
			
			trafficAdmCtrl_smEvent(pTrafficAdmCtrl, TRAFFIC_ADM_CTRL_SM_EVENT_REJECT, &fsmTSpecInfo);
			
		}
		else
		{
			/* admission accept */
			/********************/
			
			WLAN_REPORT_INFORMATION(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
				("***** admCtrlQos_recv: admission accept [ statusCode = %d ]\n",statusCode));
			
			
			WLAN_REPORT_INFORMATION(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
						("ADDTS Response (accepted) userPriority = %d ,  \n", tspecInfo.userPriority));
	
			WLAN_REPORT_INFORMATION(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
						("mediumTime = %d ,  surplusBandwidthAllowance = %d \n", ((dot11_WME_TSPEC_IE_t*)pData)->mediumTime, ((dot11_WME_TSPEC_IE_t*)pData)->surplusBandwidthAllowance));
			
			trafficAdmCtrl_smEvent(pTrafficAdmCtrl, TRAFFIC_ADM_CTRL_SM_EVENT_ACCEPT, &fsmTSpecInfo);
		}
	}
	else
	{
		status = NOK;
		WLAN_REPORT_INFORMATION(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
			("trafficAdmCtrl_recv: unknown action code = %d ,  \n",action));

	}
	return status;
}
/************************************************************************
 *                    trafficAdmCtrl_recv			                     *
 ************************************************************************
DESCRIPTION: 
                                                                                                 
INPUT:      pTrafficAdmCtrl	    -	trafficAdmCtr handle.
	
OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/



TI_STATUS trafficAdmCtrl_sendDeltsFrame(TI_HANDLE hTrafficAdmCtrl, tspecInfo_t *pTSpecInfo, UINT8 reasonCode)
{
	TI_STATUS			status = OK;
	mem_MSDU_T		    *pMsdu;
	char				*pDataBuf;
	UINT32				totalLen = 0;
	tsInfo_t			tsInfo;
        UINT32 tspecLen;


	trafficAdmCtrl_t *pTrafficAdmCtrl = (trafficAdmCtrl_t*)hTrafficAdmCtrl;

	WLAN_REPORT_INFORMATION(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
								("admCtrlQos_sendDeltsFrame: Enter....!! \n"));

	/* GET NEW MSDU !!! */
	status = wlan_memMngrAllocMSDU(pTrafficAdmCtrl->hMemMgr, &pMsdu, 2000 +TX_TOTAL_OFFSET_BEFORE_DATA, ADM_CTRL_QOS_MODULE);
	if (status != OK)
	{
		return NOK;
	}

    /* 
     * Set data offset before header builder, because it assumes it's already set
     */
    memMgr_BufOffset(pMsdu->firstBDPtr) = TX_TOTAL_OFFSET_BEFORE_DATA; 

	status = trafficAdmCtrl_buildFrameHeader(pTrafficAdmCtrl, pMsdu);
	if (status != OK)
	{
		wlan_memMngrFreeMSDU(pTrafficAdmCtrl->hMemMgr, pMsdu->handle);
		return NOK;
	}
	pDataBuf = (memMgr_BufData(pMsdu->firstBDPtr) + memMgr_BufOffset(pMsdu->firstBDPtr) + WLAN_HDR_LEN );


	*pDataBuf = WME_CATAGORY_QOS;				/* CATEGORY_QOS in WME = 17*/
	pDataBuf++ ;
	totalLen++;

	*pDataBuf = DELTS_ACTION;					/* DELTS ACTION */
	pDataBuf++ ;
	totalLen++;

       /* according to WMM Specification v1.1, section 2.2.10, dialog token = 0 in DELTS */
	*pDataBuf = 0;		/* DIALOG_TOKEN */

	pDataBuf++ ;
	totalLen++;

	/* according to WMM Specification v1.1, section 2.2.10, status = 0 in DELTS */
	*pDataBuf = 0;					/* STATUS CODE = REASON CODE */
	pDataBuf++ ;
	totalLen++;
	

    /*
     * WMM specification v1.1 specifie that DELTS must include at least
     * a full TSPEC IE.  The format used by TI is following 802.11e format
     * which does not include TSPEC IE but only TS info field
     */
    trafficAdmCtrl_buildTSPec(pTrafficAdmCtrl, pTSpecInfo, pDataBuf, &tspecLen);
    totalLen += tspecLen;

	tsInfo.tsInfoArr[0] = 0;
	tsInfo.tsInfoArr[1] = 0;
	tsInfo.tsInfoArr[2] = 0;

	/* Build tsInfo  */
    tsInfo.tsInfoArr[0] |=		( (pTSpecInfo->userPriority) << TSID_SHIFT);

	tsInfo.tsInfoArr[0] |=		(BI_DIRECTIONAL << DIRECTION_SHIFT);		/* bidirectional */
	tsInfo.tsInfoArr[0] |=		(TS_INFO_0_ACCESS_POLICY_EDCA << ACCESS_POLICY_SHIFT);	/* EDCA */
	
	tsInfo.tsInfoArr[1] |=		(0 << AGGREGATION_SHIFT);
	
	tsInfo.tsInfoArr[1] |=		(pTSpecInfo->UPSDFlag << APSD_SHIFT);
	
	tsInfo.tsInfoArr[1] |=		(pTSpecInfo->userPriority << USER_PRIORITY_SHIFT);
	tsInfo.tsInfoArr[1] |=		(NORMAL_ACKNOWLEDGEMENT << TSINFO_ACK_POLICY_SHIFT);
	
	tsInfo.tsInfoArr[2] |=		(NO_SCHEDULE << SCHEDULE_SHIFT);

#if 0 /* Need to send TSPEC IE in DELTS or only the tsinfo*/

	/* only tsinfo*/
	
	
	*pDataBuf = tsInfo.tsInfoArr[0];
	pDataBuf++;
	totalLen++;

	*pDataBuf = tsInfo.tsInfoArr[1];
	pDataBuf++;
	totalLen++;

	*pDataBuf = tsInfo.tsInfoArr[2];
	pDataBuf++;
	totalLen++; 
	
#else

	/*  send TSpec IE in DELTS*/

	{
		dot11_WME_TSPEC_IE_t*	 dot11_WME_TSPEC_IE = (dot11_WME_TSPEC_IE_t*)pDataBuf;


		dot11_WME_TSPEC_IE->tHdr.hdr.eleId =		WME_TSPEC_IE_ID;
		dot11_WME_TSPEC_IE->tHdr.hdr.eleLen =	WME_TSPEC_IE_TSINFO_LEN;

		dot11_WME_TSPEC_IE->tHdr.OUI[0] =		0x00;
		dot11_WME_TSPEC_IE->tHdr.OUI[1] =		0x50;
		dot11_WME_TSPEC_IE->tHdr.OUI[2] =		0xf2;
		dot11_WME_TSPEC_IE->tHdr.oui_type =		WME_TSPEC_IE_OUI_TYPE;
		dot11_WME_TSPEC_IE->tHdr.oui_subtype =	WME_TSPEC_IE_OUI_SUB_TYPE;
		dot11_WME_TSPEC_IE->tHdr.version =		WME_TSPEC_IE_VERSION;
		
		dot11_WME_TSPEC_IE->tHdr.tsInfoField.tsInfoArr[0] =	tsInfo.tsInfoArr[0];
		dot11_WME_TSPEC_IE->tHdr.tsInfoField.tsInfoArr[1] =	tsInfo.tsInfoArr[1];
		dot11_WME_TSPEC_IE->tHdr.tsInfoField.tsInfoArr[2] =	tsInfo.tsInfoArr[2];

		totalLen+=sizeof(dot11_WME_TSPEC_IE_hdr_t);

	}

#endif


	/*date MSDU parameters */
	pMsdu->headerLen = WLAN_HDR_LEN;
	pMsdu->dataLen += totalLen;
	pMsdu->firstBDPtr->length = pMsdu->dataLen+TX_TOTAL_OFFSET_BEFORE_DATA;

	
	
	/* send the packet to the TX */
	pMsdu->qosTag = 0;
	pMsdu->txFlags |= (TX_DATA_MGMT_MSDU);
	status = txData_txSendMsdu(pTrafficAdmCtrl->hTxData, pMsdu);



	return OK;
}


/************************************************************************
 *							INTERNAL FUNCTIONS					        *
 ************************************************************************/
/************************************************************************
 *                    trafficAdmCtrl_startTimer		                    *
 ************************************************************************
DESCRIPTION: start a specific ac timer
                                                                                                   
INPUT:      pTrafficAdmCtrl	    -	trafficAdmCtr handle.
			acID				-	the AC of the timer
	
OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

TI_STATUS trafficAdmCtrl_startTimer(trafficAdmCtrl_t* pTrafficAdmCtrl, UINT8 acID)
{
	if (pTrafficAdmCtrl == NULL)
		return NOK;
	
	os_timerStart(pTrafficAdmCtrl->hOs, pTrafficAdmCtrl->timer[acID], pTrafficAdmCtrl->timeout, FALSE);

	return OK;
}
/************************************************************************
 *                    trafficAdmCtrl_stopTimer		                    *
 ************************************************************************
DESCRIPTION: stop a specific ac timer
                                                                                                   
INPUT:      pTrafficAdmCtrl	    -	trafficAdmCtr handle.
			acID				-	the AC of the timer
	
OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

TI_STATUS trafficAdmCtrl_stopTimer(trafficAdmCtrl_t* pTrafficAdmCtrl, UINT8 acID)
{
	if (pTrafficAdmCtrl == NULL)
		return NOK;
	
	os_timerStop(pTrafficAdmCtrl->hOs, pTrafficAdmCtrl->timer[acID]);

	return OK;
}

/************************************************************************
 *						  AC timers functionc		                    *
 ************************************************************************/

/* QOS_AC_BE */
/*********/
void trafficAdmCtrl_timeoutAcBE(TI_HANDLE hTrafficAdmCtrl)
{
	fsmTSpecInfo_t	fsmTSpecInfo;
	trafficAdmCtrl_t *pTrafficAdmCtrl = (trafficAdmCtrl_t*)hTrafficAdmCtrl;

	
	/* FSM Tspec Info Structure */
	fsmTSpecInfo.acID = QOS_AC_BE;
	fsmTSpecInfo.hTrafficAdmCtrl = hTrafficAdmCtrl;
	fsmTSpecInfo.pTSpecInfo = NULL;

	trafficAdmCtrl_smEvent(pTrafficAdmCtrl, TRAFFIC_ADM_CTRL_SM_EVENT_TIMEOUT, &fsmTSpecInfo);
}

/* QOS_AC_BK */
/*********/
void trafficAdmCtrl_timeoutAcBK(TI_HANDLE hTrafficAdmCtrl)
{
	fsmTSpecInfo_t	fsmTSpecInfo;
	trafficAdmCtrl_t *pTrafficAdmCtrl = (trafficAdmCtrl_t*)hTrafficAdmCtrl;

	
	/* FSM Tspec Info Structure */
	fsmTSpecInfo.acID = QOS_AC_BK;
	fsmTSpecInfo.hTrafficAdmCtrl = hTrafficAdmCtrl;
	fsmTSpecInfo.pTSpecInfo = NULL;

	trafficAdmCtrl_smEvent(pTrafficAdmCtrl, TRAFFIC_ADM_CTRL_SM_EVENT_TIMEOUT, &fsmTSpecInfo);

}
/* QOS_AC_VI */
/*********/
void trafficAdmCtrl_timeoutAcVI(TI_HANDLE hTrafficAdmCtrl)
{
	fsmTSpecInfo_t	fsmTSpecInfo;
	trafficAdmCtrl_t *pTrafficAdmCtrl = (trafficAdmCtrl_t*)hTrafficAdmCtrl;

	
	/* FSM Tspec Info Structure */
	fsmTSpecInfo.acID = QOS_AC_VI;
	fsmTSpecInfo.hTrafficAdmCtrl = hTrafficAdmCtrl;
	fsmTSpecInfo.pTSpecInfo = NULL;

	trafficAdmCtrl_smEvent(pTrafficAdmCtrl, TRAFFIC_ADM_CTRL_SM_EVENT_TIMEOUT, &fsmTSpecInfo);

}
/* QOS_AC_VO */
/*********/
void trafficAdmCtrl_timeoutAcVO(TI_HANDLE hTrafficAdmCtrl)
{
	fsmTSpecInfo_t	fsmTSpecInfo;
	trafficAdmCtrl_t *pTrafficAdmCtrl = (trafficAdmCtrl_t*)hTrafficAdmCtrl;

	
	/* FSM Tspec Info Structure */
	fsmTSpecInfo.acID = QOS_AC_VO;
	fsmTSpecInfo.hTrafficAdmCtrl = hTrafficAdmCtrl;
	fsmTSpecInfo.pTSpecInfo = NULL;

	trafficAdmCtrl_smEvent(pTrafficAdmCtrl, TRAFFIC_ADM_CTRL_SM_EVENT_TIMEOUT, &fsmTSpecInfo);

}


static TI_STATUS trafficAdmCtrl_tokenToAc (TI_HANDLE hTrafficAdmCtrl, UINT8 token, UINT8 *acID)
{
	UINT8 idx;
	trafficAdmCtrl_t *pTrafficAdmCtrl = (trafficAdmCtrl_t*)hTrafficAdmCtrl;

	for (idx=0; idx<MAX_NUM_OF_AC; idx++)
	{
		if (pTrafficAdmCtrl->dialogToken[idx] == token)
		{
			*acID = idx;
			return (OK);
		}
	}

	return (NOK);

}



/************************************************************************
 *              trafficAdmCtrl_buildFrameHeader							*
 ************************************************************************
DESCRIPTION: build frame header 
                                                                                                   
INPUT:    	
			
OUTPUT:		

RETURN:     OK on success, NOK otherwise
************************************************************************/

TI_STATUS trafficAdmCtrl_buildFrameHeader(trafficAdmCtrl_t *pTrafficAdmCtrl, mem_MSDU_T	*pMsdu)
{
   	TI_STATUS			status = OK;
	paramInfo_t		    daParam, saParam;
	dot11_mgmtHeader_t *pdot11Header;
	bssType_e			currBssType;
	macAddress_t		currBssId;

	pdot11Header = (dot11_mgmtHeader_t*)(memMgr_BufData(pMsdu->firstBDPtr)+ memMgr_BufOffset(pMsdu->firstBDPtr));
	
    /* Get the Destination MAC address */
	daParam.paramType = CTRL_DATA_CURRENT_BSSID_PARAM;
	status = ctrlData_getParam(pTrafficAdmCtrl->hCtrlData, &daParam);
	if (status != OK)
	{
		return NOK;
	}

    /* Get the Source MAC address */
	saParam.paramType = CTRL_DATA_MAC_ADDRESS;
	status = ctrlData_getParam(pTrafficAdmCtrl->hCtrlData, &saParam);
	if (status != OK)
	{
		return NOK;
	}

	/* receive BssId and Bss Type from control module */
	ctrlData_getCurrBssTypeAndCurrBssId(pTrafficAdmCtrl->hCtrlData, &currBssId, &currBssType);
	if (currBssType != BSS_INFRASTRUCTURE)
    {
 		/* report failure but don't stop... */
		WLAN_REPORT_ERROR(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
				  ("trafficAdmCtrl_buildFrameHeader: Error !! currBssType = BSS_INFRASTRUCTURE \n"));

		return NOK;
    }
	/* infrastructure BSS */

	/* copy BSSID */
	MAC_COPY(pTrafficAdmCtrl->hOs,(&pdot11Header->BSSID),(&currBssId));
	/* copy source mac address */
	MAC_COPY(pTrafficAdmCtrl->hOs,(&pdot11Header->SA),(&saParam.content.ctrlDataCurrentBSSID));
	/* copy destination mac address */
	MAC_COPY(pTrafficAdmCtrl->hOs,(&pdot11Header->DA),(&daParam.content.ctrlDataCurrentBSSID));

	/* set frame ctrl to mgmt action frame an to DS */
	pdot11Header->fc = DOT11_FC_ACTION;

	/* Update MSDU parameters */
	pMsdu->headerLen = WLAN_HDR_LEN;
	pMsdu->dataLen = WLAN_HDR_LEN;
	pMsdu->firstBDPtr->length = WLAN_HDR_LEN;

	return OK;
}

/************************************************************************
 *                  trafficAdmCtrl_sendAdmissionReq						*
 ************************************************************************
DESCRIPTION: send admision request frame
                                                                                                   
INPUT:      hTrafficAdmCtrl	         -	Qos Manager handle.
		    pTSpecInfo				 -  tspec parameters
			
OUTPUT:		

RETURN:     OK on success, NOK otherwise
************************************************************************/

TI_STATUS trafficAdmCtrl_sendAdmissionReq(TI_HANDLE hTrafficAdmCtrl, tspecInfo_t *pTSpecInfo)
{
	TI_STATUS			status = OK;
	mem_MSDU_T		    *pMsdu;
	char				*pDataBuf;
	UINT32				len;
	UINT32				totalLen = 0;

	trafficAdmCtrl_t *pTrafficAdmCtrl = (trafficAdmCtrl_t*)hTrafficAdmCtrl;

	WLAN_REPORT_INFORMATION(pTrafficAdmCtrl->hReport, TRAFFIC_ADM_CTRL_MODULE_LOG,
								("admCtrlQos_smAdmissionReq: Enter....!! \n"));

	/* GET NEW MSDU !!! */
	status = wlan_memMngrAllocMSDU(pTrafficAdmCtrl->hMemMgr, &pMsdu, 2000 + TX_TOTAL_OFFSET_BEFORE_DATA, ADM_CTRL_QOS_MODULE);
	if (status != OK)
	{
		return NOK;
	}

    /* 
     * Set data offset before header builder, because it assumes it's already set
     */
	memMgr_BufOffset(pMsdu->firstBDPtr) = TX_TOTAL_OFFSET_BEFORE_DATA;	

	status = trafficAdmCtrl_buildFrameHeader(pTrafficAdmCtrl, pMsdu);
	if (status != OK)
	{
		wlan_memMngrFreeMSDU(pTrafficAdmCtrl->hMemMgr, pMsdu->handle);
		return NOK;
	}

	pDataBuf = (memMgr_BufData(pMsdu->firstBDPtr) + memMgr_BufOffset(pMsdu->firstBDPtr) + WLAN_HDR_LEN);

	*pDataBuf = WME_CATAGORY_QOS;			/* CATEGORY_QOS WME = 17*/
	pDataBuf++ ;
	totalLen++;

	*pDataBuf = ADDTS_REQUEST_ACTION;		/* ADDTS request ACTION */
	pDataBuf++ ;
	totalLen++;

	/* storing the dialog token for response validation */
	pTrafficAdmCtrl->dialogToken[pTSpecInfo->AC] = pTrafficAdmCtrl->dialogTokenCounter++;	/* DIALOG_TOKEN */
	*pDataBuf = pTrafficAdmCtrl->dialogToken[pTSpecInfo->AC];
	pDataBuf++ ;
	totalLen++;

	/* in WME there is a staus code in ADDTS Request ??*/
	*pDataBuf = ADDTS_STATUS_CODE_SUCCESS;	/* STATUS CODE */
	pDataBuf++ ;
	totalLen++;

	trafficAdmCtrl_buildTSPec(pTrafficAdmCtrl, pTSpecInfo, (UINT8 *)pDataBuf, (UINT32*)&len);
	pDataBuf+=len;
	totalLen+=len;
#ifdef EXC_MODULE_INCLUDED
	excMngr_buildExcTS_IEs(pTrafficAdmCtrl->hExcMgr, (UINT8 *)pDataBuf, (UINT32*)&len, pTSpecInfo->userPriority);
	pDataBuf+=len;
	totalLen+=len;
#endif
	/* Update MSDU parameters */
	pMsdu->headerLen = WLAN_HDR_LEN;
	pMsdu->dataLen += totalLen;
	pMsdu->firstBDPtr->length = pMsdu->dataLen + TX_TOTAL_OFFSET_BEFORE_DATA;
	
	/* send the packet to the TX */
	pMsdu->qosTag = 0; 
	pMsdu->txFlags |= (TX_DATA_MGMT_MSDU);

	status = txData_txSendMsdu(pTrafficAdmCtrl->hTxData, pMsdu);

	return OK;
}


/************************************************************************
 *                        trafficAdmCtrl_buildTSPec							*
 ************************************************************************
DESCRIPTION: build a tspec according to the tspec parameters
                                                                                                   
INPUT:      hTrafficAdmCtrl	         -	Qos Manager handle.
		    pTSpecInfo				 -  tspec parameters

OUTPUT:		len						 - the tspec frame len				

RETURN:     OK on success, NOK otherwise
************************************************************************/

void trafficAdmCtrl_buildTSPec(trafficAdmCtrl_t	*pTrafficAdmCtrl, 
							   tspecInfo_t		*pTSpecInfo, 
							   UINT8			*pDataBuff,
							   UINT32			*len)
{
	tsInfo_t			tsInfo;
	dot11_WME_TSPEC_IE_t*	 dot11_WME_TSPEC_IE = (dot11_WME_TSPEC_IE_t*)pDataBuff;
    dot11_local_WME_TSPEC_IE_t   myLocal;

	dot11_WME_TSPEC_IE->tHdr.hdr.eleId =	WME_TSPEC_IE_ID;
	dot11_WME_TSPEC_IE->tHdr.hdr.eleLen =	WME_TSPEC_IE_LEN;

	dot11_WME_TSPEC_IE->tHdr.OUI[0] =		0x00;
	dot11_WME_TSPEC_IE->tHdr.OUI[1] =		0x50;
	dot11_WME_TSPEC_IE->tHdr.OUI[2] =		0xf2;
	dot11_WME_TSPEC_IE->tHdr.oui_type =		WME_TSPEC_IE_OUI_TYPE;
	dot11_WME_TSPEC_IE->tHdr.oui_subtype =	WME_TSPEC_IE_OUI_SUB_TYPE;
	dot11_WME_TSPEC_IE->tHdr.version =		WME_TSPEC_IE_VERSION;


	tsInfo.tsInfoArr[0] = 0;
	tsInfo.tsInfoArr[1] = 0;
	tsInfo.tsInfoArr[2] = 0;

	tsInfo.tsInfoArr[0] |=		( (pTSpecInfo->userPriority) << TSID_SHIFT);
	tsInfo.tsInfoArr[0] |=		(pTSpecInfo->streamDirection << DIRECTION_SHIFT);		/* bidirectional */

	tsInfo.tsInfoArr[0] |=		(TS_INFO_0_ACCESS_POLICY_EDCA << ACCESS_POLICY_SHIFT);	/* EDCA */

	tsInfo.tsInfoArr[1] |=		(0 << AGGREGATION_SHIFT);

	tsInfo.tsInfoArr[1] |=		(pTSpecInfo->UPSDFlag << APSD_SHIFT);

	tsInfo.tsInfoArr[1] |=		(pTSpecInfo->userPriority << USER_PRIORITY_SHIFT);
	tsInfo.tsInfoArr[1] |=		(NORMAL_ACKNOWLEDGEMENT << TSINFO_ACK_POLICY_SHIFT);
	
	tsInfo.tsInfoArr[2] |=		(NO_SCHEDULE << SCHEDULE_SHIFT);

	dot11_WME_TSPEC_IE->tHdr.tsInfoField.tsInfoArr[0] =	tsInfo.tsInfoArr[0];
	dot11_WME_TSPEC_IE->tHdr.tsInfoField.tsInfoArr[1] =	tsInfo.tsInfoArr[1];
	dot11_WME_TSPEC_IE->tHdr.tsInfoField.tsInfoArr[2] =	tsInfo.tsInfoArr[2];
    

    myLocal.nominalMSDUSize =			pTSpecInfo->nominalMsduSize;
	myLocal.maximumMSDUSize =			(pTSpecInfo->nominalMsduSize & 0x7fff);


	myLocal.minimumServiceInterval =	0;
	myLocal.maximumServiceInterval =	0;
	myLocal.inactivityInterval =		0;
	myLocal.suspensionInterval =		0xFFFFFFFF; /*disable*/
	myLocal.serviceStartTime =			0;
	myLocal.minimumDataRate =			pTSpecInfo->meanDataRate;
	myLocal.meanDataRate =				pTSpecInfo->meanDataRate;
	myLocal.peakDataRate =				pTSpecInfo->meanDataRate;
	myLocal.maximumBurstSize =			0;
	myLocal.delayBound =				0;
	myLocal.minimumPHYRate =			pTSpecInfo->minimumPHYRate;
	myLocal.surplusBandwidthAllowance = pTSpecInfo->surplausBwAllowance;
	myLocal.mediumTime =				0;

    os_memoryCopy (pTrafficAdmCtrl->hOs,(void *)&dot11_WME_TSPEC_IE->nominalMSDUSize,(void *)&myLocal.nominalMSDUSize,sizeof(dot11_WME_TSPEC_IE_t));

	*len = sizeof(dot11_WME_TSPEC_IE_t);   
}



/************************************************************************
 *                        trafficAdmCtrl_parseTspecIE					*
 ************************************************************************
DESCRIPTION: parses a tspec IE according to the tspec parameters
                                                                                                   
INPUT:      hTrafficAdmCtrl	         -	Qos Manager handle.
		    pTSpecInfo				 -  tspec parameters

OUTPUT:		len						 - the tspec frame len				

RETURN:     OK on success, NOK otherwise
************************************************************************/

void trafficAdmCtrl_parseTspecIE(trafficAdmCtrl_t	*pTrafficAdmCtrl, 
								 tspecInfo_t		*pTSpecInfo, 
								 dot11_WME_TSPEC_IE_t	*dot11_WME_TSPEC_IE)
{
	tsInfo_t			tsInfo;
	UINT8				userPriority;
	UINT8				acID;
	UINT8				tid;
	UINT8				direction;
	UINT8				APSDbit;

	tsInfo.tsInfoArr[0] = dot11_WME_TSPEC_IE->tHdr.tsInfoField.tsInfoArr[0];
	tsInfo.tsInfoArr[1] = dot11_WME_TSPEC_IE->tHdr.tsInfoField.tsInfoArr[1];
	tsInfo.tsInfoArr[2] = dot11_WME_TSPEC_IE->tHdr.tsInfoField.tsInfoArr[2];

	userPriority = (((tsInfo.tsInfoArr[1]) & TS_INFO_1_USER_PRIORITY_MASK) >> USER_PRIORITY_SHIFT);

	acID = WMEQosTagToACTable[userPriority];

	tid = 	(((tsInfo.tsInfoArr[0]) & TS_INFO_0_TSID_MASK) >> TSID_SHIFT);
	APSDbit = (((tsInfo.tsInfoArr[1]) & TS_INFO_1_APSD_MASK) >> APSD_SHIFT);
	direction = (((tsInfo.tsInfoArr[0]) & TS_INFO_0_DIRECTION_MASK) >> DIRECTION_SHIFT);

	pTSpecInfo->AC = (acTrfcType_e)acID;
	pTSpecInfo->userPriority = userPriority;
	pTSpecInfo->nominalMsduSize = dot11_WME_TSPEC_IE->nominalMSDUSize;
	pTSpecInfo->meanDataRate = dot11_WME_TSPEC_IE->meanDataRate;
	
	/* The surplus is transmitted in the 3 MSB of UINT16 */
        pTSpecInfo->surplausBwAllowance = dot11_WME_TSPEC_IE->surplusBandwidthAllowance;

	pTSpecInfo->minimumPHYRate = dot11_WME_TSPEC_IE->minimumPHYRate;
	pTSpecInfo->mediumTime = dot11_WME_TSPEC_IE->mediumTime;
	pTSpecInfo->UPSDFlag = APSDbit;
	pTSpecInfo->streamDirection = (streamDirection_e)direction;
	pTSpecInfo->tid = tid;
}


/*************************************************************************
 *																		 *
 *							DEBUG FUNCTIONS								 *
 *																		 *
 *************************************************************************/
void trafficAdmCtrl_print(trafficAdmCtrl_t *pTrafficAdmCtr)
{
	UINT32 acID;

	WLAN_OS_REPORT(("     traffic Adm Ctrl  \n"));
	WLAN_OS_REPORT(("-----------------------------------\n\n"));
	WLAN_OS_REPORT(("timeout                   = %d\n",pTrafficAdmCtr->timeout));
	WLAN_OS_REPORT(("dialogTokenCounter        = %d\n",pTrafficAdmCtr->dialogTokenCounter));

	for (acID = 0 ; acID < MAX_NUM_OF_AC ; acID++)
	{
			WLAN_OS_REPORT(("     AC = %d  \n",acID));
			WLAN_OS_REPORT(("----------------------\n"));
			WLAN_OS_REPORT(("currentState   = %s \n",admCtrlQosSMStateDesc[pTrafficAdmCtr->currentState[acID] ]));
			WLAN_OS_REPORT(("dialogToken    = %d \n",pTrafficAdmCtr->dialogToken[acID]));
	}
}
	
