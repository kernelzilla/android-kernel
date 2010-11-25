/** \file authSM.c
 *  \brief 802.11 authentication SM source
 *
 *  \see authSM.h
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
/*		MODULE:	authSM.c												   */
/*    PURPOSE:	802.11 authentication SM source							   */
/*																	 	   */
/***************************************************************************/

#include "osApi.h"

#include "paramOut.h"
#include "paramIn.h"

#include "utils.h"
#include "fsm.h"
#include "report.h"
#include "mlmeApi.h"

#include "mlmeBuilder.h"
#include "authSm.h"
#include "openAuthSm.h"
#include "sharedKeyAuthSm.h"

/* Constants */ 

/** number of states in the state machine */
#define	AUTH_SM_MAX_NUM_STATES		4

/** number of events in the state machine */
#define	AUTH_SM_MAX_NUM_EVENTS		8

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
* auth_create - allocate memory for authentication SM
*
* \b Description: 
*
* Allocate memory for authentication SM. \n
* 		Allocates memory for Association context. \n
* 		Allocates memory for authentication timer. \n
* 		Allocates memory for authentication SM matrix. \n
*
* \b ARGS:
*
*  I   - hOs - OS context  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa rsn_mainSecSmKeysOnlyStop()
*/
TI_HANDLE auth_create(TI_HANDLE hOs)
{
	auth_t 	*pHandle;
	TI_STATUS		status;

	/* allocate authentication context memory */
	pHandle = (auth_t*)os_memoryAlloc(hOs, sizeof(auth_t));
	if (pHandle == NULL)
	{
		return NULL;
	}
	
	os_memoryZero(hOs, pHandle, sizeof(auth_t));

	pHandle->hOs = hOs;
    
	/* allocate memory for authentication state machine */
	status = fsm_Create(hOs, &pHandle->pAuthSm, AUTH_SM_MAX_NUM_STATES, AUTH_SM_MAX_NUM_EVENTS);
	if (status != OK)
	{
		os_memoryFree(hOs, pHandle, sizeof(auth_t));
		return NULL;
	}

	/* allocate OS timer memory */
	pHandle->timer = os_timerCreate(hOs, auth_smTimeout, pHandle);
	if (pHandle->timer == NULL)
	{
		fsm_Unload(hOs, pHandle->pAuthSm);
		os_memoryFree(hOs, pHandle, sizeof(auth_t));		
		return NULL;
	}

	return pHandle;
}


/**
*
* auth_unload - unload authentication SM from memory
*
* \b Description: 
*
* Unload authentication SM from memory
*
* \b ARGS:
*
*  I   - hAuth - Authentication SM context  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa rsn_mainSecSmKeysOnlyStop()
*/
TI_STATUS auth_unload(TI_HANDLE hAuth)
{
    TI_STATUS 		status;
	auth_t		*pHandle;

	pHandle = (auth_t*)hAuth;

	status = fsm_Unload(pHandle->hOs, pHandle->pAuthSm);
    if (status != OK)
	{
		/* report failure but don't stop... */
		WLAN_REPORT_ERROR(pHandle->hReport, AUTH_MODULE_LOG,
				  ("AUTH_SM: Error releasing FSM memory \n"));
	}
	
	os_timerDestroy(pHandle->hOs, pHandle->timer);
	
	os_memoryFree(pHandle->hOs, pHandle, sizeof(auth_t));

	return OK;
}

/**
*
* auth_config - configure a new authentication SM
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
* \sa auth_Create, auth_Unload
*/
TI_STATUS auth_config(TI_HANDLE 		hAuth,
				   TI_HANDLE 		hMlme,
				   TI_HANDLE		hRsn,
				   TI_HANDLE 		hReport,
				   TI_HANDLE 		hOs,
				   authInitParams_t		*pAuthInitParams)
{
	auth_t		*pHandle;
	
	if (hAuth == NULL)
	{
		return NOK;
	}

	pHandle = (auth_t*)hAuth;
	
	pHandle->hMlme = hMlme;
	pHandle->hRsn = hRsn;
	pHandle->hReport = hReport;
	pHandle->hOs = hOs;

	pHandle->timeout = pAuthInitParams->authResponseTimeout;
	pHandle->maxCount = pAuthInitParams->authMaxRetryCount;
	
	pHandle->retryCount = 0;
	pHandle->authRejectCount = 0;
	pHandle->authTimeoutCount = 0;

	pHandle->authType = AUTH_LEGACY_NONE;

	return OK;
}


/**
*
* auth_start - Start event for the authentication SM
*
* \b Description: 
*
* Start event for the authentication SM
*
* \b ARGS:
*
*  I   - hAuth - Authentication SM context  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa auth_Stop, auth_Recv
*/
TI_STATUS auth_start(TI_HANDLE hAuth)
{
	auth_t		*pHandle;

	pHandle = (auth_t*)hAuth;

	if (pHandle == NULL)
		return NOK;

	if (pHandle->authType == AUTH_LEGACY_NONE)
		return NOK;

	switch (pHandle->authType)
	{
    case AUTH_LEGACY_RESERVED1: 
	case AUTH_LEGACY_OPEN_SYSTEM:
		return auth_osSMEvent(&pHandle->currentState, OPEN_AUTH_SM_EVENT_START, pHandle);

	case AUTH_LEGACY_SHARED_KEY:
		return auth_skSMEvent(&pHandle->currentState, SHARED_KEY_AUTH_SM_EVENT_START, pHandle);

	default:
		return NOK;
	}
}

/**
*
* auth_stop - Stop event for the authentication SM
*
* \b Description: 
*
* Stop event for the authentication SM
*
* \b ARGS:
*
*  I   - hAuth - Authentication SM context  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa auth_Start, auth_Recv
*/
TI_STATUS auth_stop(TI_HANDLE hAuth, BOOL sendDeAuth, mgmtStatus_e reason )
{
	auth_t		*pHandle;

	pHandle = (auth_t*)hAuth;

	if (pHandle == NULL)
		return NOK;
	
	if (pHandle->authType == AUTH_LEGACY_NONE)
		return NOK;

	if( sendDeAuth == TRUE )
	{
		deAuth_t	deAuth;
		deAuth.reason = ENDIAN_HANDLE_WORD(reason);
		mlmeBuilder_sendFrame(pHandle->hMlme, DE_AUTH, (UINT8*)&deAuth, sizeof(deAuth_t), 0);
	}

	switch (pHandle->authType)
	{
    case AUTH_LEGACY_RESERVED1:
	case AUTH_LEGACY_OPEN_SYSTEM:
		return auth_osSMEvent(&pHandle->currentState, OPEN_AUTH_SM_EVENT_STOP, pHandle);

	case AUTH_LEGACY_SHARED_KEY:
		return auth_skSMEvent(&pHandle->currentState, SHARED_KEY_AUTH_SM_EVENT_STOP, pHandle);

	default:
		return NOK;
	}
}

/**
*
* auth_recv - Recive a message from the AP
*
* \b Description: 
*
* Parse a message form the AP and perform the appropriate event.
*
* \b ARGS:
*
*  I   - hAuth - Authentication SM context  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa auth_Start, auth_Stop
*/
TI_STATUS auth_recv(TI_HANDLE hAuth, mlmeFrameInfo_t *pFrame)
{
	auth_t			*pHandle;

	pHandle = (auth_t*)hAuth;

	if (pHandle == NULL)
		return NOK;
	
	if (pFrame->subType != AUTH)
		return NOK;

	if (pHandle->authType == AUTH_LEGACY_NONE)
		return NOK;

	if (pFrame->content.auth.status != STATUS_SUCCESSFUL)
		pHandle->authRejectCount++;

	switch (pHandle->authType)
	{
	case AUTH_LEGACY_RESERVED1:
	case AUTH_LEGACY_OPEN_SYSTEM:
		return openAuth_Recv(hAuth, pFrame);

	case AUTH_LEGACY_SHARED_KEY:
		return sharedKeyAuth_Recv(hAuth, pFrame);

	default:
		return OK;
	}
}

/**
*
* auth_getParam - Get a specific parameter from the authentication SM
*
* \b Description: 
*
* Get a specific parameter from the authentication SM.
*
* \b ARGS:
*
*  I   - hAuth - Authentication SM context  \n
*  I/O - pParam - Parameter \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa auth_Start, auth_Stop
*/
TI_STATUS auth_getParam(TI_HANDLE hAuth, paramInfo_t *pParam)
{
	auth_t		*pHandle;

	pHandle = (auth_t*)hAuth;

	if ((pHandle == NULL) || (pParam == NULL))
	{
		return NOK;
	}

	switch (pParam->paramType)
	{
	case AUTH_RESPONSE_TIMEOUT_PARAM:
		pParam->content.authResponseTimeout = pHandle->timeout;
		break;

	case AUTH_COUNTERS_PARAM:
		pParam->content.siteMgrTiWlanCounters.AuthRejects = pHandle->authRejectCount;
		pParam->content.siteMgrTiWlanCounters.AuthTimeouts = pHandle->authTimeoutCount;
		break;

	case AUTH_LEGACY_TYPE_PARAM:
		pParam->content.authLegacyAuthType = pHandle->authType;
		break;

	default:
		return NOK;
	}

	return OK;
}

/**
*
* auth_setParam - Set a specific parameter to the authentication SM
*
* \b Description: 
*
* Set a specific parameter to the authentication SM.
*
* \b ARGS:
*
*  I   - hAuth - Authentication SM context  \n
*  I/O - pParam - Parameter \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa auth_Start, auth_Stop
*/
TI_STATUS auth_setParam(TI_HANDLE hAuth, paramInfo_t *pParam)
{
	auth_t		*pHandle;

	pHandle = (auth_t*)hAuth;

	if ((pHandle == NULL) || (pParam == NULL))
	{
		return NOK;
	}

	switch (pParam->paramType)
	{
	case AUTH_LEGACY_TYPE_PARAM:
		pHandle->authType = pParam->content.authLegacyAuthType;

		switch (pHandle->authType)
		{
        case AUTH_LEGACY_RESERVED1:
		case AUTH_LEGACY_OPEN_SYSTEM:
			openAuth_Config(hAuth, pHandle->hOs);
			break;

		case AUTH_LEGACY_SHARED_KEY:
			sharedKeyAuth_Config(hAuth, pHandle->hOs);
			break;

		default:
			return NOK;
		}
		break;

	case AUTH_RESPONSE_TIMEOUT_PARAM:
		if ((pParam->content.authResponseTimeout >= AUTH_RESPONSE_TIMEOUT_MIN) &&
			(pParam->content.authResponseTimeout <= AUTH_RESPONSE_TIMEOUT_MAX))
		{
			pHandle->timeout = pParam->content.authResponseTimeout;
		} 
		else 
		{
			return NOK;
		}
		break;

	default:
		return NOK;
	}

	return OK;
}

/**
*
* auth_smTimeout - Set a specific parameter to the authentication SM
*
* \b Description: 
*
* Set a specific parameter to the authentication SM.
*
* \b ARGS:
*
*  I   - hAuth - authentication SM context  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa auth_Start, auth_Stop
*/
void auth_smTimeout(TI_HANDLE hAuth)
{
	auth_t		*pHandle;

	pHandle = (auth_t*)hAuth;

	if (pHandle == NULL)
		return;
	
	if (pHandle->authType == AUTH_LEGACY_NONE)
		return;

	pHandle->authTimeoutCount++;

	switch (pHandle->authType)
	{
    case AUTH_LEGACY_RESERVED1:
	case AUTH_LEGACY_OPEN_SYSTEM:
		openAuth_Timeout(pHandle);
		break;

	case AUTH_LEGACY_SHARED_KEY:
		sharedKey_Timeout(pHandle);
		break;

	default:
		break;
	}
}

/*****************************************************************************
**
** Authentication messages builder/Parser
**
*****************************************************************************/

/**
*
* auth_smMsgBuild - Build an authentication message and send it to the mlme builder
*
* \b Description: 
*
* Build an authentication message and send it to the mlme builder.
*
* \b ARGS:
*
*  I   - pAssoc - Association SM context  \n
*  I/O - pParam - Parameter \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa auth_Start, auth_Stop
*/
TI_STATUS auth_smMsgBuild(auth_t *pCtx, UINT16 seq, UINT16 statusCode, UINT8* pChallange, UINT8 challangeLen)
{
	TI_STATUS				status;
	UINT8				len;
	UINT8				authMsg[MAX_AUTH_MSG_LEN];
	authMsg_t			*pAuthMsg;
	dot11_CHALLENGE_t	*pDot11Challenge;
	UINT8				wepOpt;

	wepOpt = 0;

	pAuthMsg = (authMsg_t*)authMsg;
	
	/* insert algorithm */
	pAuthMsg->authAlgo = ENDIAN_HANDLE_WORD((UINT16)pCtx->authType);

	/* insert sequense */
	pAuthMsg->seqNum = ENDIAN_HANDLE_WORD(seq);

	/* insert status code */
	pAuthMsg->status = ENDIAN_HANDLE_WORD(statusCode);

	len = sizeof(authMsg_t) - sizeof(pAuthMsg->pChallenge);

	if (pChallange != NULL)
	{
		pDot11Challenge = (dot11_CHALLENGE_t*)&authMsg[len];

		pDot11Challenge->hdr.eleId = CHALLANGE_TEXT_IE_ID;
		pDot11Challenge->hdr.eleLen = challangeLen;

		os_memoryCopy(pCtx->hOs, (void *)pDot11Challenge->text, pChallange, challangeLen);
		len += challangeLen + 2;

		wepOpt = 1;
	}

	status =  mlmeBuilder_sendFrame(pCtx->hMlme, AUTH, authMsg, len, wepOpt);

	return status;
}







