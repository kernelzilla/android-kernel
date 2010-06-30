/** \file assocSM.c
 *  \brief 802.11 association SM source
 *
 *  \see assocSM.h
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
/*		MODULE:	assocSM.c												   */
/*    PURPOSE:	802.11 association SM source							   */
/*																	 	   */
/***************************************************************************/

#include "osApi.h"

#include "paramOut.h"
#include "paramIn.h"

#include "utils.h"
#include "fsm.h"
#include "report.h"

#include "DataCtrl_Api.h"
#include "siteMgrApi.h"
#include "rsnApi.h"
#include "regulatoryDomainApi.h"

#include "mlmeBuilder.h"

#include "mlmeApi.h"

#include "AssocSM.h"
#include "qosMngr_API.h"
#ifdef EXC_MODULE_INCLUDED
#include "excRMMngr.h"
#include "excMngr.h"
#endif
#include "apConn.h"

/* Constants */

/** number of states in the state machine */
#define	ASSOC_SM_NUM_STATES		3

/** number of events in the state machine */
#define	ASSOC_SM_NUM_EVENTS		6

/* Enumerations */

/* Typedefs */

/* Structures */

/* External data definitions */

/* External functions definitions */

/* Global variables */

/* Local function prototypes */

/* functions */


/* state machine functions */


TI_STATUS assoc_smEvent(assoc_t *pAssoc, UINT8 event, void *pData);

void assoc_smTimeout(TI_HANDLE hAssoc);

TI_STATUS assoc_smStartIdle(assoc_t *pAssoc);
TI_STATUS assoc_smStopWait(assoc_t *pAssoc);
TI_STATUS assoc_smSuccessWait(assoc_t *pAssoc);
TI_STATUS assoc_smFailureWait(assoc_t *pAssoc);
TI_STATUS assoc_smTimeoutWait(assoc_t *pAssoc);
TI_STATUS assoc_smMaxRetryWait(assoc_t *pAssoc);
TI_STATUS assoc_smStopAssoc(assoc_t *pAssoc);
TI_STATUS assoc_smActionUnexpected(assoc_t *pAssoc);

TI_STATUS assoc_smResetRetry(assoc_t *pAssoc);
TI_STATUS assoc_smIncRetry(assoc_t *pAssoc);
TI_STATUS assoc_smReportSuccess(assoc_t *pAssoc);
TI_STATUS assoc_smReportFailure(assoc_t *pAssoc, UINT16 uStatusCode);
TI_STATUS assoc_smSendAssocReq(assoc_t *pAssoc);
TI_STATUS assoc_smStartTimer(assoc_t *pAssoc);
TI_STATUS assoc_smStopTimer(assoc_t *pAssoc);

TI_STATUS assoc_smCapBuild(assoc_t *pCtx, UINT16 *cap);
TI_STATUS assoc_smSSIDBuild(assoc_t *pCtx, UINT8 *pSSID, UINT32 *ssidLen);
TI_STATUS assoc_smRatesBuild(assoc_t *pCtx, UINT8 *pRates, UINT32 *ratesLen);
TI_STATUS assoc_smRequestBuild(assoc_t *pCtx, UINT8* reqBuf, UINT32* reqLen);
#ifdef SUPPORT_4X
TI_STATUS assoc_4xBuild(assoc_t *pCtx, UINT8 *fourX, UINT32 *fourXLen);
#endif

TI_STATUS assoc_saveAssocReqMessage(assoc_t *pAssocSm, UINT8 *pAssocBuffer, UINT32 length);
TI_STATUS assoc_sendDisAssoc(assoc_t *pAssocSm, mgmtStatus_e reason);

/**
*
* assoc_create - allocate memory for association SM
*
* \b Description: 
*
* Allocate memory for association SM. \n
* 		Allocates memory for Association context. \n
* 		Allocates memory for association timer. \n
* 		Allocates memory for association SM matrix. \n
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
TI_HANDLE assoc_create(TI_HANDLE hOs)
{
	assoc_t 	*pHandle;
	TI_STATUS		status;

	/* allocate association context memory */
	pHandle = (assoc_t*)os_memoryAlloc(hOs, sizeof(assoc_t));
	if (pHandle == NULL)
	{
		return NULL;
	}

	os_memoryZero(hOs, pHandle, sizeof(assoc_t));

	pHandle->hOs = hOs;

	/* allocate memory for association state machine */
	status = fsm_Create(hOs, &pHandle->pAssocSm, ASSOC_SM_NUM_STATES, ASSOC_SM_NUM_EVENTS);
	if (status != OK)
	{
		os_memoryFree(hOs, pHandle, sizeof(assoc_t));
		return NULL;
	}

	/* allocate OS timer memory */
	pHandle->timer = os_timerCreate(hOs, assoc_smTimeout, pHandle);
	if (pHandle->timer == NULL)
	{
		fsm_Unload(hOs, pHandle->pAssocSm);
		os_memoryFree(hOs, pHandle, sizeof(assoc_t));		
		return NULL;
	}

	return pHandle;
}


/**
*
* assocunload - unload association SM from memory
*
* \b Description: 
*
* Unload association SM from memory
*
* \b ARGS:
*
*  I   - hAssoc - association SM context  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa rsn_mainSecSmKeysOnlyStop()
*/
TI_STATUS assoc_unload(TI_HANDLE hAssoc)
{
    TI_STATUS 		status;
	assoc_t		*pHandle;

	pHandle = (assoc_t*)hAssoc;

	status = fsm_Unload(pHandle->hOs, pHandle->pAssocSm);
    if (status != OK)
	{
		/* report failure but don't stop... */
		WLAN_REPORT_ERROR(pHandle->hReport, ASSOC_MODULE_LOG,
				  ("ASSOC_SM: Error releasing FSM memory \n"));
	}
	
	os_timerDestroy(pHandle->hOs, pHandle->timer);
	
	os_memoryFree(pHandle->hOs, hAssoc, sizeof(assoc_t));

	return OK;
}

/**
*
* assoc_config - configure a new association SM
*
* \b Description: 
*
* Configure a new association SM.
*
* \b ARGS:
*
*  I   - hAssoc - Association SM context  \n
*  I   - hMlme - MLME SM context  \n
*  I   - hSiteMgr - Site manager context  \n
*  I   - hCtrlData - Control data context  \n
*  I   - hTxData - TX data context  \n
*  I   - hHalCtrl - Hal control context  \n
*  I   - hReport - Report context  \n
*  I   - hOs - OS context  \n
*  I   - assocTimeout - Association SM timeout \n
*  I   - assocMaxCount - Max number of association requests to send  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa assoc_Create, assoc_Unload
*/
TI_STATUS assoc_config(TI_HANDLE hAssoc,
					TI_HANDLE hMlme,
					TI_HANDLE hRegulatoryDomain,
					TI_HANDLE hSiteMgr,
					TI_HANDLE hCtrlData,
					TI_HANDLE hTxData,
					TI_HANDLE hHalCtrl,
					TI_HANDLE hRsn,
					TI_HANDLE hReport,
					TI_HANDLE hOs,
					TI_HANDLE hExcMngr,
					TI_HANDLE hQosMngr,
					TI_HANDLE hMeasurementMgr,
					TI_HANDLE hApConn,
					assocInitParams_t	*pAssocInitParams)
{
	assoc_t		*pHandle;
	TI_STATUS		status;
	/** Main 802.1X State Machine matrix */
	fsm_actionCell_t	assoc_smMatrix[ASSOC_SM_NUM_STATES][ASSOC_SM_NUM_EVENTS] =
	{
		/* next state and actions for IDLE state */
		{{ASSOC_SM_STATE_WAIT, (fsm_Action_t)assoc_smStartIdle},
		 {ASSOC_SM_STATE_IDLE, (fsm_Action_t)assoc_smActionUnexpected},
		 {ASSOC_SM_STATE_IDLE, (fsm_Action_t)assoc_smActionUnexpected},
		 {ASSOC_SM_STATE_IDLE, (fsm_Action_t)assoc_smActionUnexpected},
		 {ASSOC_SM_STATE_IDLE, (fsm_Action_t)assoc_smActionUnexpected},
		 {ASSOC_SM_STATE_IDLE, (fsm_Action_t)assoc_smActionUnexpected}
		},
		/* next state and actions for WAIT state */
		{{ASSOC_SM_STATE_WAIT, (fsm_Action_t)assoc_smActionUnexpected},
		 {ASSOC_SM_STATE_IDLE, (fsm_Action_t)assoc_smStopWait},
		 {ASSOC_SM_STATE_ASSOC, (fsm_Action_t)assoc_smSuccessWait},
		 {ASSOC_SM_STATE_IDLE, (fsm_Action_t)assoc_smFailureWait},
		 {ASSOC_SM_STATE_WAIT, (fsm_Action_t)assoc_smTimeoutWait},
		 {ASSOC_SM_STATE_IDLE, (fsm_Action_t)assoc_smMaxRetryWait}
		},
		/* next state and actions for ASSOC state */
		{{ASSOC_SM_STATE_ASSOC, (fsm_Action_t)assoc_smActionUnexpected},
		 {ASSOC_SM_STATE_IDLE, (fsm_Action_t)assoc_smStopAssoc},
		 {ASSOC_SM_STATE_ASSOC, (fsm_Action_t)assoc_smActionUnexpected},
		 {ASSOC_SM_STATE_ASSOC, (fsm_Action_t)assoc_smActionUnexpected},
		 {ASSOC_SM_STATE_ASSOC, (fsm_Action_t)assoc_smActionUnexpected},
		 {ASSOC_SM_STATE_ASSOC, (fsm_Action_t)assoc_smActionUnexpected}
		}};
	

	if (hAssoc == NULL)
	{
		return NOK;
	}

	pHandle = (assoc_t*)hAssoc;
	
	/* configure state machine */
	status = fsm_Config(pHandle->pAssocSm, &assoc_smMatrix[0][0], 
						ASSOC_SM_NUM_STATES, ASSOC_SM_NUM_EVENTS, NULL, hOs);
	if (status != OK)
	{
		return NOK;
	}

	pHandle->assocRejectCount = 0;
	pHandle->assocTimeoutCount = 0;

	pHandle->currentState = ASSOC_SM_STATE_IDLE;
	
	pHandle->hMlme = hMlme;
	pHandle->hRegulatoryDomain = hRegulatoryDomain;
	pHandle->hSiteMgr = hSiteMgr;
	pHandle->hCtrlData = hCtrlData;
	pHandle->hTxData = hTxData;
	pHandle->hHalCtrl = hHalCtrl;
	pHandle->hRsn = hRsn;
	pHandle->hReport = hReport;
	pHandle->hOs = hOs;
	pHandle->hExcMngr = hExcMngr;
	pHandle->hQosMngr = hQosMngr;
    pHandle->hMeasurementMgr = hMeasurementMgr;
	pHandle->hApConn = hApConn;

	pHandle->timeout = pAssocInitParams->assocResponseTimeout;
	pHandle->maxCount = pAssocInitParams->assocMaxRetryCount;

	return OK;
}


/**
*
* assoc_start - Start event for the association SM
*
* \b Description: 
*
* Start event for the association SM
*
* \b ARGS:
*
*  I   - hAssoc - Association SM context  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa assoc_Stop, assoc_Recv
*/
TI_STATUS assoc_start(TI_HANDLE hAssoc)
{
	TI_STATUS 		status;
	assoc_t		*pHandle;

	pHandle = (assoc_t*)hAssoc;

	if (pHandle == NULL)
	{
		return NOK;
	}

	pHandle->reAssoc = FALSE;

	pHandle->disAssoc = FALSE;

	status = assoc_smEvent(pHandle, ASSOC_SM_EVENT_START, hAssoc);

	return status;
}


/**
*
* assoc_start - Start event for the association SM
*
* \b Description: 
*
* Start event for the association SM - for Re-assoc request 
*
* \b ARGS:
*
*  I   - hAssoc - Association SM context  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa assoc_Stop, assoc_Recv
*/
TI_STATUS reassoc_start(TI_HANDLE hAssoc)
{
	TI_STATUS 		status;
	assoc_t		*pHandle;

	pHandle = (assoc_t*)hAssoc;

	if (pHandle == NULL)
	{
		return NOK;
	}
	pHandle->reAssoc = TRUE;

	status = assoc_smEvent(pHandle, ASSOC_SM_EVENT_START, hAssoc);

	return status;
}

/**
*
* assoc_stop - Stop event for the association SM
*
* \b Description: 
*
* Stop event for the association SM
*
* \b ARGS:
*
*  I   - hAssoc - Association SM context  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa assoc_Start, assoc_Recv
*/
TI_STATUS assoc_stop(TI_HANDLE hAssoc)
{
	TI_STATUS 		status;
	assoc_t		*pHandle;

	pHandle = (assoc_t*)hAssoc;

	if (pHandle == NULL)
	{
		return NOK;
	}
	
	status = assoc_smEvent(pHandle, ASSOC_SM_EVENT_STOP, hAssoc);
	
	return status;
}


TI_STATUS assoc_setDisAssocFlag(TI_HANDLE hAssoc, BOOL disAsoccFlag)
{
	assoc_t		*pHandle;
	pHandle = (assoc_t*)hAssoc;	

	pHandle->disAssoc = disAsoccFlag;

	return OK;
}



/**
*
* assoc_recv - Recive a message from the AP
*
* \b Description: 
*
* Parse a message form the AP and perform the appropriate event.
*
* \b ARGS:
*
*  I   - hAssoc - Association SM context  \n
*  I   - pFrame - Frame recieved  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa assoc_Start, assoc_Stop
*/
TI_STATUS assoc_recv(TI_HANDLE hAssoc, mlmeFrameInfo_t *pFrame)
{
	TI_STATUS 		status;
	assoc_t			*pHandle;
	whalParamInfo_t		whalParam;
	UINT16			rspStatus;

	pHandle = (assoc_t*)hAssoc;

	/* ensure that the SM is waiting for assoc response */
	if(pHandle->currentState != ASSOC_SM_STATE_WAIT)
		return OK;

	if (pHandle == NULL)
	{
		return NOK;
	}
	
	if ((pFrame->subType != ASSOC_RESPONSE) && (pFrame->subType != RE_ASSOC_RESPONSE))
	{
		return NOK;
	}

    /* check response status */
	rspStatus  = pFrame->content.assocRsp.status;
	
	if (rspStatus == 0)
	{
		rsnData_t	rsnData;
        dot11_RSN_t *pRsnIe;
        UINT8       curRsnData[255];
        UINT8       rsnAssocIeLen;
        UINT16      length=0;


        WLAN_REPORT_SM(pHandle->hReport, ASSOC_MODULE_LOG,
				  ("ASSOC_SM: DEBUG Success associating to AP \n"));
		
		/* set AID to HAL */
		whalParam.paramType = HAL_CTRL_AID_PARAM;
		whalParam.content.halCtrlAid  = pFrame->content.assocRsp.aid;
		whalCtrl_SetParam(pHandle->hHalCtrl, &whalParam);
        

        /* Get the RSN IE data */
        pRsnIe = pFrame->content.assocRsp.pRsnIe;
        while ((length < pFrame->content.assocRsp.rsnIeLen) && (pFrame->content.assocRsp.rsnIeLen < 255))
        {
            if ((pRsnIe->hdr.eleLen + length + 2) > 255) { /* Dm: Security fix */
                WLAN_REPORT_ERROR(pHandle->hReport, ASSOC_MODULE_LOG, 
                                  ("%s - Security Error: %u > 255\n", __FUNCTION__,(pRsnIe->hdr.eleLen + length + 2)));
                break;
            }
            curRsnData[0+length] = pRsnIe->hdr.eleId;
            curRsnData[1+length] = pRsnIe->hdr.eleLen;
            os_memoryCopy(pHandle->hOs, &curRsnData[2+length], (void *)pRsnIe->rsnIeData, pRsnIe->hdr.eleLen); 
            length += pRsnIe->hdr.eleLen+2;
            pRsnIe += 1;
        }
        
		if (pFrame->content.assocRsp.rsnIeLen != 0)
		{
			rsnData.pIe = curRsnData;
			rsnData.ieLen = pFrame->content.assocRsp.rsnIeLen;
			rsnData.privacy =  ((pFrame->content.assocRsp.capabilities >> CAP_PRIVACY_SHIFT) & CAP_PRIVACY_MASK) ? TRUE : FALSE;
			rsn_setSite(pHandle->hRsn, &rsnData, NULL, &rsnAssocIeLen);
		}

		/* update siteMgr with capabilities and whether we are connected to Cisco AP */
		siteMgr_assocReport(pHandle->hSiteMgr,
							pFrame->content.assocRsp.capabilities, pFrame->content.assocRsp.ciscoIEPresent);

		/* update 4x info element */
		ctrlData_setSite(pHandle->hCtrlData, pFrame->content.assocRsp.fourXParams);

        /* update QoS Manager - it the QOS active protocol is NONE, or no WME IE present, it will return OK */
		/* if configured by AP, update MSDU lifetime */
        status = qosMngr_setSite(pHandle->hQosMngr, &pFrame->content.assocRsp);

        if(status != OK)
        {
			WLAN_REPORT_ERROR(pHandle->hReport, ASSOC_MODULE_LOG,
					  ("ASSOC_SM: DEBUG - Association failed : qosMngr_setSite error \n"));
			/* in case we wanted to work with qosAP and failed to connect to qos AP we want to reassociated again 
			   to another one */  
			status = assoc_smEvent(pHandle, ASSOC_SM_EVENT_FAIL, hAssoc);
        }
		else
		{
			status = assoc_smEvent(pHandle, ASSOC_SM_EVENT_SUCCESS, hAssoc);
		}
	} 
	else 
	{
		pHandle->assocRejectCount++;
		
		/* If there was attempt to renegotiate voice settings, update QoS Manager */
		qosMngr_checkTspecRenegResults(pHandle->hQosMngr, &pFrame->content.assocRsp);

		/* check failure reason */
		switch (rspStatus)
		{
		case 0:
			break;
		case 1:
			/* print debug message */
			WLAN_REPORT_SM(pHandle->hReport, ASSOC_MODULE_LOG,
							  ("ASSOC_SM: DEBUG - Association denied: Unspecified error \n"));
			break;
		case 10:
			/* print debug message */
			WLAN_REPORT_SM(pHandle->hReport, ASSOC_MODULE_LOG,
							  ("ASSOC_SM: DEBUG - Association denied: Cannot support all requested capabilities in the Capability Information field \n"));
			break;
		case 11:
			/* print debug message */
			WLAN_REPORT_SM(pHandle->hReport, ASSOC_MODULE_LOG,
							  ("ASSOC_SM: DEBUG - Association denied: Reassociation denied due to inability to confirm that association exists \n"));
			break;
		case 12:
			/* print debug message */
			WLAN_REPORT_SM(pHandle->hReport, ASSOC_MODULE_LOG,
							  ("ASSOC_SM: DEBUG - Association denied: Association denied due to reason outside the scope of this standard \n"));
			rsn_reportAuthFailure(pHandle->hRsn, RSN_AUTH_STATUS_INVALID_TYPE);
            break;
        case 13:
			WLAN_REPORT_SM(pHandle->hReport, ASSOC_MODULE_LOG,
							  ("ASSOC_SM: DEBUG - Association denied: Association denied due to wrong authentication algorithm \n"));
			rsn_reportAuthFailure(pHandle->hRsn, RSN_AUTH_STATUS_INVALID_TYPE);
            break;
		case 17:
			/* print debug message */
			WLAN_REPORT_SM(pHandle->hReport, ASSOC_MODULE_LOG,
							  ("ASSOC_SM: DEBUG - Association denied: Association denied because AP is unable to handle additional associated stations \n"));
			break;
		case 18:
			/* print debug message */
			WLAN_REPORT_SM(pHandle->hReport, ASSOC_MODULE_LOG,
							  ("ASSOC_SM: DEBUG - Association denied: Association denied due to requesting station not supporting all of the data rates in the BSSBasicRateSet parameter \n"));
			break;
		default:
			/* print error message on wrong error code for association response */
			WLAN_REPORT_ERROR(pHandle->hReport, ASSOC_MODULE_LOG,
							  ("ASSOC_SM: ERROR - Association denied: error code (%d) irrelevant \n", rspStatus));
			break;
		}

		status = assoc_smEvent(pHandle, ASSOC_SM_EVENT_FAIL, hAssoc);
	}

	return status;
}

/**
*
* assoc_getParam - Get a specific parameter from the association SM
*
* \b Description: 
*
* Get a specific parameter from the association SM.
*
* \b ARGS:
*
*  I   - hAssoc - Association SM context  \n
*  I/O - pParam - Parameter \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa assoc_Start, assoc_Stop
*/
TI_STATUS assoc_getParam(TI_HANDLE hAssoc, paramInfo_t *pParam)
{
	assoc_t		*pHandle;

	pHandle = (assoc_t*)hAssoc;

	if ((pHandle == NULL) || (pParam == NULL))
	{
		return NOK;
	}

	/* serch parameter type */
	switch (pParam->paramType)
	{
	case ASSOC_RESPONSE_TIMEOUT_PARAM:
		pParam->content.assocResponseTimeout = pHandle->timeout;
		break;

	case ASSOC_COUNTERS_PARAM:
		pParam->content.siteMgrTiWlanCounters.AssocRejects = pHandle->assocRejectCount;
		pParam->content.siteMgrTiWlanCounters.AssocTimeouts = pHandle->assocTimeoutCount;
		break;

	case ASSOC_ASSOCIATION_RESP_PARAM:
		pParam->content.applicationConfigBuffer.buffer = pHandle->assocRespBuffer;
		pParam->content.applicationConfigBuffer.bufferSize = pHandle->assocRespLen;
		break;

    case ASSOC_ASSOCIATION_INFORMATION_PARAM:
       {
           UINT8  reqBuffIEOffset, respBuffIEOffset;
           UINT32 RequestIELength = 0;
           UINT32 ResponseIELength = 0;
		   paramInfo_t	param;

		   WLAN_REPORT_SM(pHandle->hReport, ASSOC_MODULE_LOG,
                              ("ASSOC_SM: DEBUG - Association Information Get:  \n"));

		   /* Assoc exists only in Infrastructure */
		   param.paramType = CTRL_DATA_CURRENT_BSS_TYPE_PARAM;
		   ctrlData_getParam(pHandle->hCtrlData, &param);
		   if (param.content.ctrlDataCurrentBssType != BSS_INFRASTRUCTURE)
		   {
			   WLAN_REPORT_ERROR(pHandle->hReport, ASSOC_MODULE_LOG, 
									 ("Not in Infrastructure BSS, No ASSOC Info for GET ASSOC_ASSOCIATION_INFORMATION_PARAM\n"));
			   return NOK;
		   }

           /* Init the result buffer to 0 */
           os_memoryZero(pHandle->hOs ,&pParam->content, sizeof(OS_802_11_ASSOCIATION_INFORMATION));

           reqBuffIEOffset  = 4;  /* In Assoc request frame IEs are located from byte 4 */
           respBuffIEOffset = 6;  /* In Assoc response frame the IEs are located from byte 6 */

            /* If the last associate was re-associciation, the current AP MAC address */
            /* is placed before the IEs. Copy it to the result parameters.            */
            if (pHandle->reAssoc)
    	    {
                os_memoryCopy(pHandle->hOs,
                              (void *)pParam->content.assocAssociationInformation.RequestFixedIEs.CurrentAPAddress,
                              &pHandle->assocReqBuffer[reqBuffIEOffset], MAC_ADDR_LEN);
                reqBuffIEOffset += MAC_ADDR_LEN;
            }

            /* Calculate length of Info elements in assoc request and response frames */
            if(pHandle->assocReqLen > reqBuffIEOffset)
                RequestIELength = pHandle->assocReqLen - reqBuffIEOffset;

            if(pHandle->assocRespLen > respBuffIEOffset)
                ResponseIELength = pHandle->assocRespLen - respBuffIEOffset;

            /* Copy the association request information */
            pParam->content.assocAssociationInformation.Length = sizeof(OS_802_11_ASSOCIATION_INFORMATION);
            pParam->content.assocAssociationInformation.AvailableRequestFixedIEs = OS_802_11_AI_REQFI_CAPABILITIES | OS_802_11_AI_REQFI_LISTENINTERVAL;
            pParam->content.assocAssociationInformation.RequestFixedIEs.Capabilities = *(UINT16*)&(pHandle->assocReqBuffer[0]);
            pParam->content.assocAssociationInformation.RequestFixedIEs.ListenInterval = *(UINT16*)(&pHandle->assocReqBuffer[2]);

            pParam->content.assocAssociationInformation.RequestIELength = RequestIELength; 
            pParam->content.assocAssociationInformation.OffsetRequestIEs = 0;
            if (RequestIELength > 0)
            {
                pParam->content.assocAssociationInformation.OffsetRequestIEs = (UINT32)&pHandle->assocReqBuffer[reqBuffIEOffset];
            }
            /* Copy the association response information */
            pParam->content.assocAssociationInformation.AvailableResponseFixedIEs = 
                OS_802_11_AI_RESFI_CAPABILITIES | OS_802_11_AI_RESFI_STATUSCODE | OS_802_11_AI_RESFI_ASSOCIATIONID;
            pParam->content.assocAssociationInformation.ResponseFixedIEs.Capabilities = *(UINT16*)&(pHandle->assocRespBuffer[0]);
            pParam->content.assocAssociationInformation.ResponseFixedIEs.StatusCode = *(UINT16*)&(pHandle->assocRespBuffer[2]);
            pParam->content.assocAssociationInformation.ResponseFixedIEs.AssociationId = *(UINT16*)&(pHandle->assocRespBuffer[4]);
            pParam->content.assocAssociationInformation.ResponseIELength = ResponseIELength;
            pParam->content.assocAssociationInformation.OffsetResponseIEs = 0;
            if (ResponseIELength > 0)
            {
                pParam->content.assocAssociationInformation.OffsetResponseIEs = (UINT32)&pHandle->assocRespBuffer[respBuffIEOffset];
            }

       }
        break;
	default:
		return NOK;
	}

	return OK;
}

/**
*
* assoc_getParamPartial - Get a specific parameter from the association SM
*
* \b Description: 
*
* Get a specific parameter from the association SM.
*
* \b ARGS:
*
*  I   - hAssoc - Association SM context  \n
*  I/O - pParam - Parameter \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa assoc_Start, assoc_Stop
*/
/* note: assoc_getParamPartial() is part of assoc_getParam() it was implemented to reduce Stack usage */
TI_STATUS assoc_getParamPartial(TI_HANDLE hAssoc, paramInfoPartial_t *pParam)
{
	assoc_t		*pHandle;

	pHandle = (assoc_t*)hAssoc;

	if ((pHandle == NULL) || (pParam == NULL))
	{
		return NOK;
	}

	/* serch parameter type */
	switch (pParam->paramType)
	{
    case ASSOC_ASSOCIATION_RESP_PARAM:
		pParam->content.applicationConfigBuffer.buffer = pHandle->assocRespBuffer;
		pParam->content.applicationConfigBuffer.bufferSize = pHandle->assocRespLen;
		break;

	default:
		WLAN_REPORT_ERROR(pHandle->hReport, ASSOC_MODULE_LOG, 
							  ("assoc_getParamPartial no such entry %d\n",pParam->paramType));
        return NOK;
	}

	return OK;
}


/**
*
* assoc_setParam - Set a specific parameter to the association SM
*
* \b Description: 
*
* Set a specific parameter to the association SM.
*
* \b ARGS:
*
*  I   - hAssoc - Association SM context  \n
*  I/O - pParam - Parameter \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa assoc_Start, assoc_Stop
*/
TI_STATUS assoc_setParam(TI_HANDLE hAssoc, paramInfo_t *pParam)
{
	assoc_t		*pHandle;

	pHandle = (assoc_t*)hAssoc;

	if ((pHandle == NULL) || (pParam == NULL))
	{
		return NOK;
	}

	switch (pParam->paramType)
	{
	case ASSOC_RESPONSE_TIMEOUT_PARAM:
		/* check bounds */
		if ((pParam->content.assocResponseTimeout >= ASSOC_RESPONSE_TIMEOUT_MIN) &&
			(pParam->content.assocResponseTimeout <= ASSOC_RESPONSE_TIMEOUT_MAX))
		{
			pHandle->timeout = pParam->content.assocResponseTimeout;
		} else {
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
* assoc_smTimeout - Time out event activation function
*
* \b Description: 
*
* Time out event activation function.
*
* \b ARGS:
*
*  I   - hAssoc - Association SM context  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa assoc_Start, assoc_Stop
*/
void assoc_smTimeout(TI_HANDLE hAssoc)
{
	assoc_t		*pHandle;

	pHandle = (assoc_t*)hAssoc;


	if (pHandle == NULL)
	{
		return;
	}
	
	pHandle->assocTimeoutCount++;

	assoc_smEvent(pHandle, ASSOC_SM_EVENT_TIMEOUT, hAssoc);
}

/**
*
* assoc_smEvent - Perform an event on the association SM
*
* \b Description: 
*
* Perform an event on the association SM.
*
* \b ARGS:
*
*  I   - pAssoc - Association SM context  \n
*  I   - event - Current event \n
*  I   - pData - event related data
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa 
*/

#ifdef REPORT_LOG

static char *assocSMStateDesc[ASSOC_SM_NUM_STATES] = {	
		"ASSOC_SM_STATE_IDLE",
		"ASSOC_SM_STATE_WAIT",
		"ASSOC_SM_STATE_ASSOC",
	};
	
/* State machine inputs */
static char *assocSMEventDesc[ASSOC_SM_NUM_EVENTS] = {
		"ASSOC_SM_EVENT_START",
		"ASSOC_SM_EVENT_STOP",
		"ASSOC_SM_EVENT_SUCCESS",
		"ASSOC_SM_EVENT_FAILURE",
		"ASSOC_SM_EVENT_TIME_OUT",
		"ASSOC_SM_EVENT_MAX_RETRY"
	};

#endif

TI_STATUS assoc_smEvent(assoc_t *pAssoc, UINT8 event, void *pData)
{
	TI_STATUS 		status;
	UINT8		nextState;

	status = fsm_GetNextState(pAssoc->pAssocSm, pAssoc->currentState, event, &nextState);
	if (status != OK)
	{
		WLAN_REPORT_ERROR(pAssoc->hReport, ASSOC_MODULE_LOG,
						  ("ASSOC_SM: ERROR - failed getting next state \n"));

		return(NOK);
	}

	WLAN_REPORT_SM(pAssoc->hReport, ASSOC_MODULE_LOG,
					 ("ASSOC_SM: <%s, %s> --> %s\n",
					  assocSMStateDesc[pAssoc->currentState],
					  assocSMEventDesc[event],
					  assocSMStateDesc[nextState]));

	status = fsm_Event(pAssoc->pAssocSm, &pAssoc->currentState, event, pData);

	return(status);
}

/* state machine functions */

TI_STATUS assoc_smStartIdle(assoc_t *pAssoc)
{
	TI_STATUS		status;

	status = assoc_smResetRetry(pAssoc);
	status = assoc_smSendAssocReq(pAssoc);
	status = assoc_smStartTimer(pAssoc);
	status = assoc_smIncRetry(pAssoc);

	return status;
}

TI_STATUS assoc_smStopWait(assoc_t *pAssoc)
{
	TI_STATUS		status;

	status = assoc_smStopTimer(pAssoc);

	return status;
}

TI_STATUS assoc_smSuccessWait(assoc_t *pAssoc)
{
	TI_STATUS		status;

	status = assoc_smStopTimer(pAssoc);
	status = assoc_smReportSuccess(pAssoc);

	return status;
}

TI_STATUS assoc_smFailureWait(assoc_t *pAssoc)
{
	TI_STATUS		status;
	UINT16			uRspStatus = *(UINT16*)&(pAssoc->assocRespBuffer[2]);

	status = assoc_smStopTimer(pAssoc);

	/* Sanity check. If the Response status is indeed not 0 */
	if (uRspStatus)	
	{
		status = assoc_smReportFailure(pAssoc, uRspStatus);
	}
	else	/* (uRspStatus == 0) how did we get here ? */ 
	{
		WLAN_REPORT_ERROR(pAssoc->hReport, ASSOC_MODULE_LOG,
			("assoc_smFailureWait: while Response status is OK (0) !!! \n"));

		status = assoc_smReportFailure(pAssoc, (UINT16)NOK);
	}
	return status;
}

TI_STATUS assoc_smTimeoutWait(assoc_t *pAssoc)
{
	TI_STATUS		status;

	status = assoc_smSendAssocReq(pAssoc);
	status = assoc_smStartTimer(pAssoc);
	status = assoc_smIncRetry(pAssoc);

	return status;
}

TI_STATUS assoc_smMaxRetryWait(assoc_t *pAssoc)
{
	TI_STATUS		status;

	status = assoc_smStopTimer(pAssoc);
	status = assoc_smReportFailure(pAssoc, STATUS_PACKET_REJ_TIMEOUT);

	return status;
}

TI_STATUS assoc_smSendAssocReq(assoc_t *pAssoc)
{
	UINT8 				assocMsg[MAX_ASSOC_MSG_LENGTH];
	UINT32				msgLen;
	TI_STATUS			status;
	dot11MgmtSubType_e	assocType=ASSOC_REQUEST;

	if (pAssoc->reAssoc)
	{
		assocType = RE_ASSOC_REQUEST;
	}
	status = assoc_smRequestBuild(pAssoc, assocMsg, &msgLen);
	if (status != OK)
		return status;

    /* Save the association request message */
	assoc_saveAssocReqMessage(pAssoc, assocMsg, msgLen);
	status = mlmeBuilder_sendFrame(pAssoc->hMlme, assocType, assocMsg, msgLen, 0);
	
	return status;
}

TI_STATUS assoc_smStopAssoc(assoc_t *pAssoc)
{
	if (pAssoc->disAssoc) {
		assoc_sendDisAssoc(pAssoc, STATUS_UNSPECIFIED);
	}
	return OK;
}

TI_STATUS assoc_smActionUnexpected(assoc_t *pAssoc)
{
	return OK;
}

/* local functions */


TI_STATUS assoc_smResetRetry(assoc_t *pAssoc)
{
	if (pAssoc == NULL)
	{
		return NOK;
	}
	
	pAssoc->retryCount = 0;
	
	return OK;
}

TI_STATUS assoc_smIncRetry(assoc_t *pAssoc)
{
	TI_STATUS		status;

	if (pAssoc == NULL)
	{
		return NOK;
	}
	
	pAssoc->retryCount++;
	
	if (pAssoc->retryCount > pAssoc->maxCount)
	{
		status = assoc_smEvent(pAssoc, ASSOC_SM_EVENT_MAX_RETRY, pAssoc);

		return status;
	}

	return OK;
}

TI_STATUS assoc_smReportSuccess(assoc_t *pAssoc)
{
	TI_STATUS 		status;

	if (pAssoc == NULL)
	{
		return NOK;
	}
	status = mlme_reportAssocStatus(pAssoc->hMlme, (UINT16)OK);

	return status;
}

TI_STATUS assoc_smReportFailure(assoc_t *pAssoc, UINT16 uStatusCode)
{
	TI_STATUS 		status;

	if (pAssoc == NULL)
	{
		return NOK;
	}
	
	status = mlme_reportAssocStatus(pAssoc->hMlme, uStatusCode);

	return status;
}

TI_STATUS assoc_smStartTimer(assoc_t *pAssoc)
{
	if (pAssoc == NULL)
	{
		return NOK;
	}
	
	os_timerStart(pAssoc->hOs, pAssoc->timer, pAssoc->timeout, FALSE);

	return OK;
}

TI_STATUS assoc_smStopTimer(assoc_t *pAssoc)
{
	if (pAssoc == NULL)
	{
		return NOK;
	}
	
	os_timerStop(pAssoc->hOs, pAssoc->timer);

	return OK;
}

/*****************************************************************************
**
** Association messages builder/Parser
**
*****************************************************************************/

TI_STATUS assoc_smCapBuild(assoc_t *pCtx, UINT16 *cap)
{
	paramInfo_t			param;
	TI_STATUS			status;
    dot11mode_e         mode;
	UINT32				rateSuppMask, rateBasicMask;
	UINT8				ratesBuf[MAX_SUPPORTED_RATES];
	UINT32				len = 0, ofdmIndex = 0;

	*cap = 0;

	/* Bss type */
    param.paramType = CTRL_DATA_CURRENT_BSS_TYPE_PARAM;
	status =  ctrlData_getParam(pCtx->hCtrlData, &param);
	if (status == OK)
	{
		if (param.content.ctrlDataCurrentBssType == BSS_INFRASTRUCTURE)
		{
			*cap |= DOT11_CAPS_ESS;
		} else {
			*cap |= DOT11_CAPS_IBSS;
		}
	} else {
		return NOK;
	}

	/* Privacy */
    param.paramType = RSN_ENCRYPTION_STATUS_PARAM;
	status =  rsn_getParam(pCtx->hRsn, &param);
	if (status == OK)
	{
		if (param.content.rsnEncryptionStatus != RSN_CIPHER_NONE)
		{
			*cap |= DOT11_CAPS_PRIVACY;
		}
	} else {
		return NOK;
	}

	/* Preamble */
    param.paramType = SITE_MGR_DESIRED_PREAMBLE_TYPE_PARAM;
	status =  siteMgr_getParam(pCtx->hSiteMgr, &param);
	if (status == OK)
	{
		if (param.content.siteMgrCurrentPreambleType == PREAMBLE_SHORT)
			*cap |= DOT11_CAPS_SHORT_PREAMBLE;
	} else {
		return NOK;
	}

	/* Pbcc */
    param.paramType = SITE_MGR_CURRENT_RATE_PAIR_PARAM;
	status =  siteMgr_getParam(pCtx->hSiteMgr, &param);
	if (status == OK)
	{
		if(param.content.siteMgrCurrentRateMask.supportedRateMask & DRV_RATE_MASK_22_PBCC)
			*cap |= DOT11_CAPS_PBCC;
	} else {
		return NOK;
	}

	
	/* Checking if the station supports Spectrum Management (802.11h) */
	param.paramType = REGULATORY_DOMAIN_MANAGEMENT_CAPABILITY_ENABLED_PARAM;
	status =  regulatoryDomain_getParam(pCtx->hRegulatoryDomain, &param);
	if (status == OK )
	{
		if( param.content.spectrumManagementEnabled)
			*cap |= DOT11_SPECTRUM_MANAGEMENT;
	} 
	else
	{
		return NOK;
	}
	
	/* slot time */
    param.paramType = SITE_MGR_OPERATIONAL_MODE_PARAM;
    status = siteMgr_getParam(pCtx->hSiteMgr, &param);
    if(status == OK)
    {
        mode = param.content.siteMgrDot11OperationalMode;
    }
    else
        return NOK;

    if(mode == DOT11_G_MODE)
    {
		/* new requirement: the short slot time should be set only
		   if the AP's modulation is OFDM (highest rate) */
		
		/* get Rates */
		param.paramType = SITE_MGR_CURRENT_RATE_PAIR_PARAM;
		status =  siteMgr_getParam(pCtx->hSiteMgr, &param);
		if (status == OK)
		{
			rateBasicMask = param.content.siteMgrCurrentRateMask.basicRateMask;
			rateSuppMask  = param.content.siteMgrCurrentRateMask.supportedRateMask;
		} else {
			return NOK;
		}
		
		/* convert the bit map to the rates array */
		bitMapToNetworkStringRates(rateSuppMask, rateBasicMask,
								   ratesBuf, &len, &ofdmIndex);

		if(ofdmIndex < len)
			*cap |= DOT11_CAPS_SHORT_SLOT_TIME;

/*		
		param.paramType = SITE_MGR_CURRENT_MODULATION_TYPE_PARAM;
		status = siteMgr_getParam(pCtx->hSiteMgr, &param);
		if(param.content.siteMgrCurrentModulationType == DRV_MODULATION_OFDM)
			*cap |= DOT11_CAPS_SHORT_SLOT_TIME;
*/
    }

    /* Qos Support - (is WME on?)*/
    param.paramType = QOS_MNGR_ACTIVE_PROTOCOL;
    status = qosMngr_getParams(pCtx->hQosMngr, &param);
    if (status == OK)
    {
      if (param.content.qosSiteProtocol !=  NONE_QOS)
      {
         // *cap |= DOT11_CAPS_QOS_SUPPORTED;  /* deleted due to MCS00035798 */
      }
    }
    else 
      {
	     return NOK;
	  }

	return OK;
}

#ifdef SUPPORT_4X
TI_STATUS assoc_4xBuild(assoc_t *pCtx, UINT8 *fourX, UINT32 *fourXLen)
{

	TI_STATUS		status;
	dot11_4X_t		*pDot11_4X;
	paramInfo_t		param;
	BOOL			sts, fourXen;

	pDot11_4X = (dot11_4X_t*)fourX;

	param.paramType = SITE_MGR_4X_PARAM;
	sts = siteMgr_getParam(pCtx->hSiteMgr, &param);
	fourXen = param.content.siteMgrFourxParam;
	if(sts != OK || fourXen == FALSE)
	{
		pDot11_4X->hdr.eleId = 0;
		pDot11_4X->hdr.eleLen = 0;
		*fourXLen = 0;
		return OK;
	}

	status = ctrlData_get4xInfoElemnt(pCtx->hCtrlData, pDot11_4X);
	if(status != OK)
	{
		pDot11_4X->hdr.eleId = 0;
		pDot11_4X->hdr.eleLen = 0;
		*fourXLen = 0;
		return OK;
	}

	*fourXLen = pDot11_4X->hdr.eleLen + sizeof(dot11_eleHdr_t);
	
	return OK;
}
#endif
TI_STATUS assoc_smSSIDBuild(assoc_t *pCtx, UINT8 *pSSID, UINT32 *ssidLen)
{
	paramInfo_t			param;
	TI_STATUS				status;
	dot11_SSID_t		*pDot11Ssid;

	pDot11Ssid = (dot11_SSID_t*)pSSID;
	/* set SSID element id */
	pDot11Ssid->hdr.eleId = SSID_IE_ID;

	/* get SSID */
    param.paramType = SITE_MGR_DESIRED_SSID_PARAM;
	status =  siteMgr_getParam(pCtx->hSiteMgr, &param);
	if (status != OK)
	{
		return status;
    }
    
    /* check for ANY ssid */
    if (param.content.siteMgrDesiredSSID.len != 0)
    {
        pDot11Ssid->hdr.eleLen = param.content.siteMgrDesiredSSID.len;
        os_memoryCopy(pCtx->hOs, 
                      (void *)pDot11Ssid->serviceSetId, 
                      (void *)param.content.siteMgrDesiredSSID.ssidString, 
                      param.content.siteMgrDesiredSSID.len);

    } else {
        /* if ANY ssid is configured, use the current SSID */
        param.paramType = SITE_MGR_CURRENT_SSID_PARAM;
        status =  siteMgr_getParam(pCtx->hSiteMgr, &param);
        if (status != OK)
        {
            return status;
        }
        pDot11Ssid->hdr.eleLen = param.content.siteMgrCurrentSSID.len;
        os_memoryCopy(pCtx->hOs, 
                      (void *)pDot11Ssid->serviceSetId, 
                      (void *)param.content.siteMgrCurrentSSID.ssidString, 
                      param.content.siteMgrCurrentSSID.len);

    }

	*ssidLen = pDot11Ssid->hdr.eleLen + sizeof(dot11_eleHdr_t);
	
	return OK;
}

TI_STATUS assoc_smRatesBuild(assoc_t *pCtx, UINT8 *pRates, UINT32 *ratesLen)
{
	paramInfo_t			param;
	TI_STATUS			status;
	UINT32				rateSuppMask, rateBasicMask;
	dot11_RATES_t		*pDot11Rates;
	UINT32				len = 0, ofdmIndex = 0;
	UINT8				ratesBuf[MAX_SUPPORTED_RATES];
	BOOL				useESRie;
	dot11mode_e			mode;
	UINT32				suppRatesLen, extSuppRatesLen, i;
	pDot11Rates = (dot11_RATES_t*)pRates;


	/* get Rates */
    param.paramType = SITE_MGR_CURRENT_RATE_PAIR_PARAM;
	status =  siteMgr_getParam(pCtx->hSiteMgr, &param);
	if (status == OK)
	{
		rateBasicMask = param.content.siteMgrCurrentRateMask.basicRateMask;
		rateSuppMask  = param.content.siteMgrCurrentRateMask.supportedRateMask;
	} else {
		return NOK;
	}

	/* get operational mode */
	param.paramType = SITE_MGR_OPERATIONAL_MODE_PARAM;
	status = siteMgr_getParam(pCtx->hSiteMgr, &param);
	if(status == OK)
		mode = param.content.siteMgrDot11OperationalMode;
	else
		return NOK;

	/* get param indicating whether the ESR IE should be used */
	param.paramType = SITE_MGR_USE_DRAFT_NUM_PARAM;
	status = siteMgr_getParam(pCtx->hSiteMgr, &param);
	if(status == OK)
	{
		if(param.content.siteMgrUseDraftNum == DRAFT_6_AND_LATER)
			useESRie = TRUE;
		else
			useESRie = FALSE;
	}
	else
		return NOK;

	/* convert the bit map to the rates array */
	bitMapToNetworkStringRates(rateSuppMask, rateBasicMask,
							   ratesBuf, &len, &ofdmIndex);

	if(mode != DOT11_G_MODE || ofdmIndex == len || useESRie == FALSE)
	{
		pDot11Rates->hdr.eleId = SUPPORTED_RATES_IE_ID;
		pDot11Rates->hdr.eleLen = len;
		os_memoryCopy(NULL, (void *)pDot11Rates->rates, ratesBuf, len);
		*ratesLen = pDot11Rates->hdr.eleLen + sizeof(dot11_eleHdr_t);
	}
	else 
	{
		/* fill in the supported rates */
		pDot11Rates->hdr.eleId = SUPPORTED_RATES_IE_ID;
		pDot11Rates->hdr.eleLen = ofdmIndex;
		os_memoryCopy(NULL, (void *)pDot11Rates->rates, ratesBuf, pDot11Rates->hdr.eleLen);
		suppRatesLen = pDot11Rates->hdr.eleLen + sizeof(dot11_eleHdr_t);
		/* fill in the extended supported rates */
		pDot11Rates = (dot11_RATES_t*)(pRates + suppRatesLen);
		pDot11Rates->hdr.eleId = EXT_SUPPORTED_RATES_IE_ID;
		pDot11Rates->hdr.eleLen = len - ofdmIndex;
		os_memoryCopy(NULL, (void *)pDot11Rates->rates, &ratesBuf[ofdmIndex], pDot11Rates->hdr.eleLen);
		extSuppRatesLen = pDot11Rates->hdr.eleLen + sizeof(dot11_eleHdr_t);
		*ratesLen = suppRatesLen + extSuppRatesLen;
	}

	WLAN_REPORT_INFORMATION(pCtx->hReport, ASSOC_MODULE_LOG,
							("ASSOC_SM: ASSOC_REQ - bitmapSupp= 0x%X,bitMapBasic = 0x%X, len = %d\n",						   
							rateSuppMask,rateBasicMask,len));
	for(i=0; i<len; i++)
	{
		WLAN_REPORT_INFORMATION(pCtx->hReport, ASSOC_MODULE_LOG,("ASSOC_SM: ASSOC_REQ - ratesBuf[%d] = 0x%X\n",							  
								i, ratesBuf[i]));
	}

	return OK;
}

TI_STATUS assoc_powerCapabilityBuild(assoc_t *pCtx, UINT8 *pPowerCapability, UINT32 *powerCapabilityLen)
{
	paramInfo_t			param;
	TI_STATUS				status;
	dot11_CAPABILITY_t		*pDot11PowerCapability;

	pDot11PowerCapability = (dot11_CAPABILITY_t*)pPowerCapability;
	
	/* set Power Capability element id */
	pDot11PowerCapability->hdr.eleId = DOT11_CAPABILITY_ELE_ID;
	pDot11PowerCapability->hdr.eleLen = DOT11_CAPABILITY_ELE_LEN;

	/* get power capability */
    param.paramType = REGULATORY_DOMAIN_POWER_CAPABILITY_PARAM;
	status =  regulatoryDomain_getParam(pCtx->hRegulatoryDomain, &param);

	if (status == OK)
	{
		pDot11PowerCapability->minTxPower = param.content.powerCapability.minTxPower;
		pDot11PowerCapability->maxTxPower = param.content.powerCapability.maxTxPower;
		*powerCapabilityLen = pDot11PowerCapability->hdr.eleLen + sizeof(dot11_eleHdr_t);
	}
	else
		*powerCapabilityLen = 0;

	return OK;
}
#if 0
	/* Supported Channels IE is not required */

TI_STATUS assoc_supportedChannelBuild(assoc_t *pCtx, UINT8 *pSupportedChannels, UINT32 *supportedChannelsLen)
{
	paramInfo_t						param;
	TI_STATUS						status;
	dot11_CHANNEL_SUPPORTED_t		*pDot11SupportedChannels;

	pDot11SupportedChannels = (dot11_CHANNEL_SUPPORTED_t*)pSupportedChannels;
	
	/* set Supported Channels element id */
	pDot11SupportedChannels->hdr.eleId = DOT11_CHANNEL_SUPPORTED_ELE_ID;
	pDot11SupportedChannels->hdr.eleLen = 0;

    /* get Num of Supported Channels */
    param.paramType = REGULATORY_DOMAIN_SUPPORTED_CHANNEL_PAIRS_NUM_PARAM;
	status =  regulatoryDomain_getParam(pCtx->hRegulatoryDomain, &param);

    if(status == OK)
    {
        pDot11SupportedChannels->hdr.eleLen = param.content.numOfSupportedChannelPairs * 2;

	/* get Supported Channels */
    param.paramType = REGULATORY_DOMAIN_SUPPORTED_CHANNEL_PARAM;
	status =  regulatoryDomain_getParam(pCtx->hRegulatoryDomain, &param);

	if (status == OK)
	{
            
            os_memoryCopy(pCtx->hOs,&(pDot11SupportedChannels->supportedChannel[0]),param.content.pSupportedChannel,pDot11SupportedChannels->hdr.eleLen * sizeof(UINT8));
		*supportedChannelsLen = pDot11SupportedChannels->hdr.eleLen + sizeof(dot11_eleHdr_t);
	}
	else
		*supportedChannelsLen = 0;
    }
    else
        *supportedChannelsLen = 0;

	return OK;
}
#endif


TI_STATUS assoc_smRequestBuild(assoc_t *pCtx, UINT8* reqBuf, UINT32* reqLen)
{
	TI_STATUS		status;
	UINT8			*pRequest;
	UINT32			len;
	paramInfo_t		param;
    whalParamInfo_t whalParam;
	UINT16			capabilities;
	
	pRequest = reqBuf;
	*reqLen = 0;

	/* insert capabilities */
	status = assoc_smCapBuild(pCtx, &capabilities);
	if (status == OK)
	{
		*(UINT16*)pRequest = ENDIAN_HANDLE_WORD(capabilities);
	}
	else
		return NOK;

	pRequest += 2;
	*reqLen += 2;

	/* insert listen interval */
    whalParam.paramType = HAL_CTRL_LISTEN_INTERVAL_PARAM;
	status =  whalCtrl_GetParam(pCtx->hHalCtrl, &whalParam);
	if (status == OK)
	{
		*(UINT16*)pRequest = ENDIAN_HANDLE_WORD((UINT16)whalParam.content.halCtrlListenInterval);
	} else {
		return NOK;
	}
	
	pRequest += 2;
	*reqLen += 2;
	if (pCtx->reAssoc)
	{	/* Insert currentAPAddress element only in reassoc request*/
		param.paramType = SITE_MGR_PREV_SITE_BSSID_PARAM;
		status = siteMgr_getParam(pCtx->hSiteMgr, &param);
		if (status == OK)
		{
			os_memoryCopy(pCtx->hOs, pRequest, (void *)param.content.siteMgrDesiredBSSID.addr, MAC_ADDR_LEN);
			WLAN_REPORT_INFORMATION(pCtx->hReport, ASSOC_MODULE_LOG,
									("ASSOC_SM: ASSOC_REQ - prev AP = %x-%x-%x-%x-%x-%x\n",						   
									param.content.siteMgrDesiredBSSID.addr[0], param.content.siteMgrDesiredBSSID.addr[1],
									param.content.siteMgrDesiredBSSID.addr[2], param.content.siteMgrDesiredBSSID.addr[3],
									param.content.siteMgrDesiredBSSID.addr[4], param.content.siteMgrDesiredBSSID.addr[5]));


			pRequest += MAC_ADDR_LEN;
			*reqLen += MAC_ADDR_LEN;
		}
		else
		{
			WLAN_REPORT_ERROR(pCtx->hReport, ASSOC_MODULE_LOG,
									("ASSOC_SM: ASSOC_REQ - No prev AP \n"));
			return status;

		}
	}

	/* insert SSID element */
	status = assoc_smSSIDBuild(pCtx, pRequest, &len);
	if (status != OK)
	{
		return NOK;
	}

	pRequest += len;
	*reqLen += len;

	/* insert Rates element */
	status = assoc_smRatesBuild(pCtx, pRequest, &len);
	if (status != OK)
	{
		return NOK;
	}
	pRequest += len;
	*reqLen += len;

	/* Checking if the station supports Spectrum Management (802.11h) */
	param.paramType = REGULATORY_DOMAIN_MANAGEMENT_CAPABILITY_ENABLED_PARAM;
	status = regulatoryDomain_getParam(pCtx->hRegulatoryDomain,&param);
	if( (status == OK) && param.content.spectrumManagementEnabled)
	{
		/* Checking the selected AP capablities */
		param.paramType = SITE_MGR_SITE_CAPABILITY_PARAM;
		status =  siteMgr_getParam(pCtx->hSiteMgr,&param);
		if(status == OK && ((param.content.siteMgrSiteCapability & DOT11_SPECTRUM_MANAGEMENT) != 0)) 
		{
			/* insert Power capability element */
			status = assoc_powerCapabilityBuild(pCtx, pRequest, &len);
			if (status != OK)
			{
				return NOK;
			}
			pRequest += len;
			*reqLen += len;
#if 0
			/* insert Supported Channels element */
			status = assoc_supportedChannelBuild(pCtx, pRequest, &len);
			if (status != OK)
			{
				return NOK;
			}
			pRequest += len;
			*reqLen += len;
#endif
		}


	}

	status = qosMngr_getQosCapabiltyInfeElement(pCtx->hQosMngr,pRequest,(UINT8*)&len);
	if (status != OK)
	{
		return NOK;
	}
	pRequest += len;
	*reqLen += len;


#ifdef EXC_MODULE_INCLUDED
	status = rsn_getExcExtendedInfoElement(pCtx->hRsn, pRequest, (UINT8*)&len);
	if (status != OK)
	{
		return NOK;
	}
	pRequest += len;
	*reqLen += len;

	if (pCtx->reAssoc)
	{	/* insert CCKM information element only in reassoc */
		status = excMngr_getCckmInfoElement(pCtx->hExcMngr, pRequest, (UINT8*)&len);
		
		if (status != OK)
		{
			return NOK;
		}
		pRequest += len;
		*reqLen += len;
	}
	status = excMngr_getEXCVersionInfoElement(pCtx->hExcMngr, pRequest, (UINT8*)&len);
	if (status != OK)
	{
		return NOK;
	}
	pRequest += len;
	*reqLen += len;

    /* Insert Radio Mngt Capability IE */
    status = measurementMgr_radioMngtCapabilityBuild(pCtx->hMeasurementMgr, pRequest, (UINT8*)&len);
    if (status != OK)
	{
		return NOK;
	}
	pRequest += len;
	*reqLen += len;
#endif

#ifdef SUPPORT_4X
	/* insert 4X element */
	status = assoc_4xBuild(pCtx, pRequest, &len);
	if (status != OK)
	{
		return NOK;
	}
	pRequest += len;
	*reqLen += len;
#endif

    /* insert RSN information elements */
    status = rsn_getInfoElement(pCtx->hRsn, pRequest, (UINT8*)&len);
	
	if (status != OK)
	{
		return NOK;
	}
	pRequest += len;
	*reqLen += len;

	status = qosMngr_assocReqBuild(pCtx->hQosMngr,pRequest,(UINT8*)&len);
	if (status != OK)
	{
		return NOK;
	}
	pRequest += len;
	*reqLen += len;

	status = apConn_getVendorSpecificIE(pCtx->hApConn, pRequest, &len);
	if (status != OK)
	{
		return NOK;
	}
	pRequest += len;
	*reqLen += len;
 
	if (*reqLen>=MAX_ASSOC_MSG_LENGTH)
	{
		return NOK;
	}

	return OK;
}



TI_STATUS assoc_saveAssocRespMessage(assoc_t *pAssocSm, UINT8 *pAssocBuffer, UINT32 length)
{
    if ((pAssocSm==NULL) || (pAssocBuffer==NULL) || (length>=MAX_ASSOC_MSG_LENGTH))
    {
        return NOK;
    }
    os_memoryCopy(pAssocSm->hOs, pAssocSm->assocRespBuffer, pAssocBuffer, length);  
    pAssocSm->assocRespLen = length;
    
    WLAN_REPORT_INFORMATION(pAssocSm->hReport, ASSOC_MODULE_LOG,
                      ("assoc_saveAssocRespMessage: length=%ld \n",length));
    return OK;
}

TI_STATUS assoc_saveAssocReqMessage(assoc_t *pAssocSm, UINT8 *pAssocBuffer, UINT32 length)
{

    if ((pAssocSm==NULL) || (pAssocBuffer==NULL) || (length>=MAX_ASSOC_MSG_LENGTH))
    {
        return NOK;
    }

    os_memoryCopy(pAssocSm->hOs, pAssocSm->assocReqBuffer, pAssocBuffer, length);  
    pAssocSm->assocReqLen = length;
    
    WLAN_REPORT_INFORMATION(pAssocSm->hReport, ASSOC_MODULE_LOG,
                      ("assoc_saveAssocReqMessage: length=%ld \n",length));
    return OK;
}


TI_STATUS assoc_sendDisAssoc(assoc_t *pAssocSm, mgmtStatus_e reason)
{
	TI_STATUS		status;
	disAssoc_t		disAssoc;

	if (reason == STATUS_SUCCESSFUL)
	{
		disAssoc.reason = ENDIAN_HANDLE_WORD(STATUS_UNSPECIFIED);
	} else {
		disAssoc.reason = ENDIAN_HANDLE_WORD(reason);
	}

	status = mlmeBuilder_sendFrame(pAssocSm->hMlme, DIS_ASSOC, (UINT8*)&disAssoc, sizeof(disAssoc_t), 0);

	return status;
}


