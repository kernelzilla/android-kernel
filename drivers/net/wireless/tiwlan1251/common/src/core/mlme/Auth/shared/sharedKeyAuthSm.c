/** \file sharedKeyAuthSm.c
 *  \brief shared key 802.11 authentication SM source
 *
 *  \see sharedKeyAuthSm.h 
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
/*																		   */
/*		MODULE:	sharedKeyAuthSm.c										   */
/*    PURPOSE:	shared key 802.11 authentication SM source				   */
/*																	 	   */
/***************************************************************************/

#include "osApi.h"

#include "paramOut.h"
#include "paramIn.h"

#include "utils.h"
#include "fsm.h"
#include "report.h"
#include "mlmeApi.h"
#include "../authSm.h"
#include "sharedKeyAuthSm.h"

/* Constants */

/** number of states in the state machine */
#define	SHARED_KEY_AUTH_SM_NUM_STATES		4

/** number of events in the state machine */
#define	SHARED_KEY_AUTH_SM_NUM_EVENTS		8

/* Enumerations */

/* Typedefs */

/* Structures */

/* External data definitions */

/* External functions definitions */

/* Global variables */

/* Local function prototypes */

/* functions */

/**
*
* sharedKeyAuth_smConfig - configure a new authentication SM
*
* \b Description: 
*
* Configure a new authentication SM.
*
* \b ARGS:
*
*  I   - hAuth - Association SM context  \n
*  I   - hMlme - MLME SM context  \n
*  I   - hSiteMgr - Site manager context  \n
*  I   - hCtrlData - Control data context  \n
*  I   - hTxData - TX data context  \n
*  I   - hHalCtrl - Hal control context  \n
*  I   - hReport - Report context  \n
*  I   - hOs - OS context  \n
*  I   - authTimeout - Association SM timeout \n
*  I   - authMaxCount - Max number of authentication requests to send  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa sharedKeyAuth_Create, sharedKeyAuth_Unload
*/
TI_STATUS sharedKeyAuth_Config(TI_HANDLE hAuth, TI_HANDLE hOs)
{
	auth_t		*pHandle;
	TI_STATUS		status;
	/** Main 802.1X State Machine matrix */
	fsm_actionCell_t	sharedKeyAuth_smMatrix[SHARED_KEY_AUTH_SM_NUM_STATES][SHARED_KEY_AUTH_SM_NUM_EVENTS] =
	{
		/* next state and actions for IDLE state */
		{{SHARED_KEY_AUTH_SM_STATE_WAIT_1, (fsm_Action_t)sharedKeyAuth_smStartIdle},
		 {SHARED_KEY_AUTH_SM_STATE_IDLE, (fsm_Action_t)sharedKeyAuth_smActionUnexpected},
		 {SHARED_KEY_AUTH_SM_STATE_IDLE, (fsm_Action_t)sharedKeyAuth_smActionUnexpected},
		 {SHARED_KEY_AUTH_SM_STATE_IDLE, (fsm_Action_t)sharedKeyAuth_smActionUnexpected},
		 {SHARED_KEY_AUTH_SM_STATE_IDLE, (fsm_Action_t)sharedKeyAuth_smActionUnexpected},
		 {SHARED_KEY_AUTH_SM_STATE_IDLE, (fsm_Action_t)sharedKeyAuth_smActionUnexpected},
		 {SHARED_KEY_AUTH_SM_STATE_IDLE, (fsm_Action_t)sharedKeyAuth_smActionUnexpected},
		 {SHARED_KEY_AUTH_SM_STATE_IDLE, (fsm_Action_t)sharedKeyAuth_smActionUnexpected}
		},
		/* next state and actions for WAIT_1 state */
		{{SHARED_KEY_AUTH_SM_STATE_WAIT_1, (fsm_Action_t)sharedKeyAuth_smActionUnexpected},
		 {SHARED_KEY_AUTH_SM_STATE_IDLE, (fsm_Action_t)sharedKeyAuth_smStopWait},
		 {SHARED_KEY_AUTH_SM_STATE_WAIT_2, (fsm_Action_t)sharedKeyAuth_smSuccess1Wait1},
		 {SHARED_KEY_AUTH_SM_STATE_IDLE, (fsm_Action_t)sharedKeyAuth_smFailure1Wait1},
		 {SHARED_KEY_AUTH_SM_STATE_WAIT_1, (fsm_Action_t)sharedKeyAuth_smActionUnexpected},
		 {SHARED_KEY_AUTH_SM_STATE_WAIT_1, (fsm_Action_t)sharedKeyAuth_smActionUnexpected},
		 {SHARED_KEY_AUTH_SM_STATE_WAIT_1, (fsm_Action_t)sharedKeyAuth_smTimeoutWait1},
		 {SHARED_KEY_AUTH_SM_STATE_IDLE, (fsm_Action_t)sharedKeyAuth_smMaxRetryWait}
		},
		/* next state and actions for WAIT_2 state */
		{{SHARED_KEY_AUTH_SM_STATE_WAIT_2, (fsm_Action_t)sharedKeyAuth_smActionUnexpected},
		 {SHARED_KEY_AUTH_SM_STATE_IDLE, (fsm_Action_t)sharedKeyAuth_smStopWait},
		 {SHARED_KEY_AUTH_SM_STATE_WAIT_2, (fsm_Action_t)sharedKeyAuth_smActionUnexpected},
		 {SHARED_KEY_AUTH_SM_STATE_WAIT_2, (fsm_Action_t)sharedKeyAuth_smActionUnexpected},
		 {SHARED_KEY_AUTH_SM_STATE_AUTH, (fsm_Action_t)sharedKeyAuth_smSuccess2Wait2},
		 {SHARED_KEY_AUTH_SM_STATE_IDLE, (fsm_Action_t)sharedKeyAuth_smFailure2Wait2},
		 {SHARED_KEY_AUTH_SM_STATE_WAIT_2, (fsm_Action_t)sharedKeyAuth_smTimeoutWait2},
		 {SHARED_KEY_AUTH_SM_STATE_IDLE, (fsm_Action_t)sharedKeyAuth_smMaxRetryWait}
		},
		/* next state and actions for AUTH state */
		{{SHARED_KEY_AUTH_SM_STATE_AUTH, (fsm_Action_t)sharedKeyAuth_smActionUnexpected},
		 {SHARED_KEY_AUTH_SM_STATE_IDLE, (fsm_Action_t)sharedKeyAuth_smStopAuth},
		 {SHARED_KEY_AUTH_SM_STATE_AUTH, (fsm_Action_t)sharedKeyAuth_smActionUnexpected},
		 {SHARED_KEY_AUTH_SM_STATE_AUTH, (fsm_Action_t)sharedKeyAuth_smActionUnexpected},
		 {SHARED_KEY_AUTH_SM_STATE_AUTH, (fsm_Action_t)sharedKeyAuth_smActionUnexpected},
		 {SHARED_KEY_AUTH_SM_STATE_AUTH, (fsm_Action_t)sharedKeyAuth_smActionUnexpected},
		 {SHARED_KEY_AUTH_SM_STATE_AUTH, (fsm_Action_t)sharedKeyAuth_smActionUnexpected},
		 {SHARED_KEY_AUTH_SM_STATE_AUTH, (fsm_Action_t)sharedKeyAuth_smActionUnexpected}
		}};
	

	if (hAuth == NULL)
	{
		return NOK;
	}

	pHandle = (auth_t*)hAuth;
	
	status = fsm_Config(pHandle->pAuthSm, &sharedKeyAuth_smMatrix[0][0], 
						SHARED_KEY_AUTH_SM_NUM_STATES, SHARED_KEY_AUTH_SM_NUM_EVENTS, auth_skSMEvent, hOs);
	if (status != OK)
	{
		return NOK;
	}

	pHandle->currentState = SHARED_KEY_AUTH_SM_STATE_IDLE;
	
	return OK;
}


#ifdef REPORT_LOG

static char *stateDesc[SHARED_KEY_AUTH_SM_NUM_STATES] = 
	{
		"SHARED_KEY_AUTH_SM_STATE_IDLE",
		"SHARED_KEY_AUTH_SM_STATE_WAIT_1",
		"SHARED_KEY_AUTH_SM_STATE_WAIT_2",
		"SHARED_KEY_AUTH_SM_STATE_AUTH",
	};

static char *eventDesc[SHARED_KEY_AUTH_SM_NUM_EVENTS] = 
	{
		"SHARED_KEY_AUTH_SM_EVENT_START",
		"SHARED_KEY_AUTH_SM_EVENT_STOP",
		"SHARED_KEY_AUTH_SM_EVENT_SUCCESS_1",
		"SHARED_KEY_AUTH_SM_EVENT_FAIL_1",
		"SHARED_KEY_AUTH_SM_EVENT_SUCCESS_2",
		"SHARED_KEY_AUTH_SM_EVENT_FAIL_2",
		"SHARED_KEY_AUTH_SM_EVENT_TIMEOUT",
		"SHARED_KEY_AUTH_SM_EVENT_MAX_RETRY",
	};

#endif


TI_STATUS auth_skSMEvent(UINT8 *currentState, UINT8 event, TI_HANDLE hAuth)
{
   auth_t *pAuth = (auth_t *)hAuth;
	TI_STATUS 		status;
	UINT8		nextState;

	status = fsm_GetNextState(pAuth->pAuthSm, *currentState, event, &nextState);
	if (status != OK)
	{
		WLAN_REPORT_SM(pAuth->hReport, AUTH_MODULE_LOG, ("State machine error, failed getting next state\n"));
		return(NOK);
	}

	WLAN_REPORT_SM(pAuth->hReport, AUTH_MODULE_LOG, 
							  ("<%s, %s> --> %s\n\n",
							   stateDesc[*currentState],
							   eventDesc[event],
							   stateDesc[nextState]));

	status = fsm_Event(pAuth->pAuthSm, currentState, event, (void *)pAuth);

	return status;
}


/**
*
* sharedKeyAuth_Recv - Recive a message from the AP
*
* \b Description: 
*
* Parse a message form the AP and perform the appropriate event.
*
* \b ARGS:
*
*  I   - hAuth - Association SM context  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa sharedKeyAuth_Start, sharedKeyAuth_Stop
*/
TI_STATUS sharedKeyAuth_Recv(TI_HANDLE hAuth, mlmeFrameInfo_t *pFrame)
{
	TI_STATUS 			status = NOK;
	auth_t			*pHandle;
	UINT16			authAlgo;
	UINT16			rspSeq;

	pHandle = (auth_t*)hAuth;

	if (pHandle == NULL)
	{
		return NOK;
	}
	
	/* check response status */
	authAlgo = pFrame->content.auth.authAlgo;
	if (authAlgo != AUTH_LEGACY_SHARED_KEY)
	{
		WLAN_REPORT_SM(pHandle->hReport, AUTH_MODULE_LOG,
				  ("SHARED_KEY_AUTH_SM: DEBUG recieved authentication message with wrong algorithm \n"));
		return NOK;
	}
    
	/* check response status */
	rspSeq  = pFrame->content.auth.seqNum;
	
    pHandle->authData.status = pFrame->content.auth.status;
    pHandle->authData.pChalange = (char *)(pFrame->content.auth.pChallenge->text);
    pHandle->authData.challangeLen = pFrame->content.auth.pChallenge->hdr.eleLen;

	if (pHandle->authData.status == STATUS_SUCCESSFUL)
	{
		switch (rspSeq)
		{
		case 2:
			WLAN_REPORT_SM(pHandle->hReport, AUTH_MODULE_LOG,
					  ("SHARED_KEY_AUTH_SM: DEBUG Success authenticating to AP stage 1\n"));

			if (pFrame->content.auth.pChallenge->hdr.eleId != CHALLANGE_TEXT_IE_ID)
			{
				WLAN_REPORT_ERROR(pHandle->hReport, AUTH_MODULE_LOG,
						  ("SHARED_KEY_AUTH_SM: Wrong element ID for challange \n"));
				status = NOK;
				break;
			}

			status = auth_skSMEvent(&pHandle->currentState, SHARED_KEY_AUTH_SM_EVENT_SUCCESS_1, hAuth);
			break;

		case 4:
			WLAN_REPORT_SM(pHandle->hReport, AUTH_MODULE_LOG,
					  ("SHARED_KEY_AUTH_SM: DEBUG Success authenticating to AP stage 2\n"));
			
			status = auth_skSMEvent(&pHandle->currentState, SHARED_KEY_AUTH_SM_EVENT_SUCCESS_2, hAuth);
			break;

		default:
			WLAN_REPORT_ERROR(pHandle->hReport, AUTH_MODULE_LOG,
					  ("SHARED_KEY_AUTH_SM: Wrong sequence number \n"));
			status = NOK;
			break;
		}
	} 
	
	else 
	{
		switch (rspSeq)
		{
		case 2:
			status = auth_skSMEvent(&pHandle->currentState, SHARED_KEY_AUTH_SM_EVENT_FAIL_1, hAuth);
			break;

		case 4:
			status = auth_skSMEvent(&pHandle->currentState, SHARED_KEY_AUTH_SM_EVENT_FAIL_2, hAuth);
			break;
	
		default:
			status = NOK;
			break;
		}
	}

	return status;
}

/* state machine functions */

TI_STATUS sharedKeyAuth_smStartIdle(auth_t *hAuth)
{
	TI_STATUS		status;

	status = sharedKeyAuth_smResetRetry(hAuth);
	status = sharedKeyAuth_smSendAuth1(hAuth);
	status = sharedKeyAuth_smStartTimer(hAuth);
	status = sharedKeyAuth_smIncRetry(hAuth);

	return status;
}

TI_STATUS sharedKeyAuth_smStopWait(auth_t *hAuth)
{
	TI_STATUS		status;

	status = sharedKeyAuth_smStopTimer(hAuth);

	return status;
}

TI_STATUS sharedKeyAuth_smSuccess1Wait1(auth_t *hAuth)
{
	TI_STATUS		status;

	status = sharedKeyAuth_smResetRetry(hAuth);
	if (status != OK)
		return status;
	status = sharedKeyAuth_smStopTimer(hAuth);
	if (status != OK)
		return status;
	status = sharedKeyAuth_smSendAuth2(hAuth);
	if (status != OK)
		return status;
	status = sharedKeyAuth_smStartTimer(hAuth);
	if (status != OK)
		return status;
	status = sharedKeyAuth_smIncRetry(hAuth);

	return status;
}

TI_STATUS sharedKeyAuth_smFailure1Wait1(auth_t *hAuth)
{
	TI_STATUS		status;

	status = sharedKeyAuth_smStopTimer(hAuth);
	status = sharedKeyAuth_smReportFailure(hAuth);

	return status;
}

TI_STATUS sharedKeyAuth_smTimeoutWait1(auth_t *hAuth)
{
	TI_STATUS		status;

	status = sharedKeyAuth_smSendAuth1(hAuth);
	status = sharedKeyAuth_smStartTimer(hAuth);
	status = sharedKeyAuth_smIncRetry(hAuth);

	return status;
}

TI_STATUS sharedKeyAuth_smMaxRetryWait(auth_t *hAuth)
{
	TI_STATUS		status;

	status = sharedKeyAuth_smReportFailure(hAuth);

	return status;
}

TI_STATUS sharedKeyAuth_smSuccess2Wait2(auth_t *hAuth)
{
	TI_STATUS		status;

	status = sharedKeyAuth_smStopTimer(hAuth);
	status = sharedKeyAuth_smReportSuccess(hAuth);

	return status;
}

TI_STATUS sharedKeyAuth_smFailure2Wait2(auth_t *hAuth)
{
	TI_STATUS		status;

	status = sharedKeyAuth_smStopTimer(hAuth);
	status = sharedKeyAuth_smReportFailure(hAuth);

	return status;
}

TI_STATUS sharedKeyAuth_smTimeoutWait2(auth_t *hAuth)
{
	TI_STATUS		status;

	status = sharedKeyAuth_smSendAuth2(hAuth);
	status = sharedKeyAuth_smStartTimer(hAuth);
	status = sharedKeyAuth_smIncRetry(hAuth);

	return status;
}

/* action routines for authentication SM */

TI_STATUS sharedKeyAuth_smSendAuth1(auth_t *hAuth)
{
	TI_STATUS		status;

	status = auth_smMsgBuild(hAuth, 1, 0, NULL, 0);

	return status;
}

TI_STATUS sharedKeyAuth_smSendAuth2(auth_t *hAuth)
{
	TI_STATUS		status;
	
	/* GET SECRET  */

	/* ENCRYPT CHALLANGE WITH SECRET */

	status = auth_smMsgBuild(hAuth, 3, 0, (UINT8 *)(hAuth->authData.pChalange), hAuth->authData.challangeLen);

	return status;
}

TI_STATUS sharedKeyAuth_smStopAuth(auth_t *hAuth)
{
	return OK;
}

TI_STATUS sharedKeyAuth_smActionUnexpected(auth_t *hAuth)
{
	return OK;
}

/* local functions */


TI_STATUS sharedKeyAuth_smResetRetry(auth_t *hAuth)
{
	if (hAuth == NULL)
	{
		return NOK;
	}
	
	hAuth->retryCount = 0;
	
	return OK;
}

TI_STATUS sharedKeyAuth_smIncRetry(auth_t *hAuth)
{
	if (hAuth == NULL)
	{
		return NOK;
	}
	
	hAuth->retryCount++;
	
	return OK;
} 

TI_STATUS sharedKeyAuth_smReportSuccess(auth_t *hAuth)
{
	TI_STATUS 		status;

	if (hAuth == NULL)
	{
		return NOK;
	}
	
	status = mlme_reportAuthStatus(hAuth->hMlme, hAuth->authData.status);

	return status;
}

TI_STATUS sharedKeyAuth_smReportFailure(auth_t *hAuth)
{
	TI_STATUS 		status;

	if (hAuth == NULL)
	{
		return NOK;
	}
	
	status = mlme_reportAuthStatus(hAuth->hMlme, hAuth->authData.status);

	return status;
}

TI_STATUS sharedKeyAuth_smStartTimer(auth_t *hAuth)
{
	if (hAuth == NULL)
	{
		return NOK;
	}
	
	os_timerStart(hAuth->hOs, hAuth->timer, hAuth->timeout, FALSE);

	return OK;
}

TI_STATUS sharedKeyAuth_smStopTimer(auth_t *hAuth)
{
	if (hAuth == NULL)
	{
		return NOK;
	}
	
	os_timerStop(hAuth->hOs, hAuth->timer);

	return OK;
}

TI_STATUS sharedKey_Timeout(auth_t *pAuth)
{
	if (pAuth->retryCount >= pAuth->maxCount)
	{
		pAuth->authData.status = STATUS_PACKET_REJ_TIMEOUT;
		return auth_skSMEvent(&pAuth->currentState, SHARED_KEY_AUTH_SM_EVENT_MAX_RETRY, pAuth);
	}

	return auth_skSMEvent(&pAuth->currentState, SHARED_KEY_AUTH_SM_EVENT_TIMEOUT, pAuth);
}


