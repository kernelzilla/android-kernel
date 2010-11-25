/** \file conn.c
 *  \brief connection module interface
 *
 *  \see conn.h
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

/****************************************************************************************************/
/*																									*/
/*		MODULE:		conn.c																			*/
/*		PURPOSE:	Connection module interface. The connection	itself is implemented in the files	*/
/*					connInfra, connIbss & connSelf. This file distributes the events received to 	*/
/*					one of the modules based on the current connection type.						*/																 	
/*																									*/
/****************************************************************************************************/


#include "report.h"
#include "osTIType.h"
#include "osApi.h"
#include "conn.h"
#include "connApi.h"
#include "connIbss.h"
#include "connInfra.h"
#include "802_11Defs.h"
#include "utils.h"
#include "smeApi.h"
#include "paramOut.h"
#include "paramIn.h"
#include "siteMgrApi.h"
#include "smeSmApi.h"
#include "scrApi.h"
#include "healthMonitor.h"
#include "qosMngr_API.h"

#define CONN_INIT_BIT					1
#define TIMER_INIT_BIT					2
#define	IBSS_SM_INIT_BIT				3
#define	INFRA_SM_INIT_BIT				4
#define INFRA_DISASSOC_TIMER_INIT_BIT	6

/* Local functions prototypes */

static void conn_timeout(TI_HANDLE hConn);

static void release_module(conn_t *pConn, UINT32 initVec);

/* Interface functions Implementation */

/************************************************************************
 *                        conn_create								*
 ************************************************************************
DESCRIPTION: Connection module creation function, called by the config mgr in creation phase 
				performs the following:
				-	Allocate the connection handle
				-	Create the connection timer
				-	Create the connection state machine
                                                                                                   
INPUT:      hOs -			Handle to OS		


OUTPUT:		

RETURN:     Handle to the connection module on success, NULL otherwise

************************************************************************/
TI_HANDLE conn_create(TI_HANDLE hOs)
{
	conn_t			*pConn;
	UINT32			initVec;
	fsm_stateMachine_t *pFsm;
	TI_STATUS status;

	initVec = 0;

	pConn = os_memoryAlloc(hOs, sizeof(conn_t));
	if (pConn == NULL)
		return NULL;

	initVec |= (1<<CONN_INIT_BIT);

    /* Rename to connSelfIbssTimer */
	pConn->pTimer = os_timerCreate(hOs, conn_timeout, pConn);
	if (pConn->pTimer == NULL)
	{
		release_module(pConn, initVec);
		return NULL;
	}
	
	initVec |= (1<<TIMER_INIT_BIT);
	
	
	/* Creating connection Ibss SM */
    status = fsm_Create(hOs, &pFsm, CONN_IBSS_NUM_STATES, CONN_IBSS_NUM_EVENTS);
	if (status != OK)
	{
		release_module(pConn, initVec);
		return NULL;
	}
	pConn->ibss_pFsm = pFsm;

    initVec |= (1<<IBSS_SM_INIT_BIT);

    /* Creating connection Infra SM */
   	status = fsm_Create(hOs, &pFsm, CONN_INFRA_NUM_STATES, CONN_INFRA_NUM_EVENTS);
	if (status != OK)
	{
		release_module(pConn, initVec);
		return NULL;
	}
	pConn->infra_pFsm = pFsm;

    initVec |= (1<<INFRA_SM_INIT_BIT);


    /* ------- */

	pConn->hOs = hOs;

	return(pConn);
}


/************************************************************************
 *                        conn_config									*
 ************************************************************************
DESCRIPTION: Connection module configuration function, called by the config mgr in configuration phase
				performs the following:
				-	Reset & initiailzes local variables
				-	Init the handles to be used by the module
                                                                                                   
INPUT:      hConn	-	Connection handle
			List of handles to be used by the module
			pConnInitParams	-	Init table of the module.		


OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS conn_config(TI_HANDLE 	hConn, 
				   TI_HANDLE 	hSiteMgr, 
				   TI_HANDLE	hSmeSm, 
				   TI_HANDLE	hMlmeSm, 
				   TI_HANDLE	hRsn,
				   TI_HANDLE	hRxData,
				   TI_HANDLE	hTxData,
				   TI_HANDLE 	hReport,
				   TI_HANDLE 	hOs,
				   TI_HANDLE	hPwrMngr,
				   TI_HANDLE    hCtrlData,
				   TI_HANDLE	hMeasurementMgr,
				   TI_HANDLE	hTrafficMonitor,
				   TI_HANDLE	hScr,
				   TI_HANDLE	hExcMngr,
				   TI_HANDLE	hQosMngr,
				   TI_HANDLE	hHalCtrl,
				   TI_HANDLE	hScanCnc,
				   TI_HANDLE	hCurrBss,
				   TI_HANDLE	hSwitchChannel,
				   TI_HANDLE	hEvHandler,
				   TI_HANDLE	hHealthMonitor,
				   TI_HANDLE	hMacServices,
                   TI_HANDLE    hRegulatoryDomain,
  				   TI_HANDLE    hSoftGemini,
				   connInitParams_t		*pConnInitParams)
{
	conn_t *pConn = (conn_t *)hConn;

	pConn->state = 0;
	os_memoryZero(hOs, &(pConn->smContext), sizeof(connSmContext_t)); 
	pConn->timeout			= pConnInitParams->connSelfTimeout;
	pConn->hSiteMgr			= hSiteMgr;
	pConn->hSmeSm			= hSmeSm;
	pConn->hMlmeSm			= hMlmeSm;
	pConn->hRsn				= hRsn;
	pConn->hRxData			= hRxData;
	pConn->hTxData			= hTxData;
	pConn->hReport			= hReport;
	pConn->hOs				= hOs;
	pConn->hPwrMngr			= hPwrMngr;
	pConn->hCtrlData		= hCtrlData;
	pConn->hMeasurementMgr	= hMeasurementMgr;
	pConn->hTrafficMonitor  = hTrafficMonitor;
	pConn->hScr				= hScr;
	pConn->hExcMngr			= hExcMngr;
	pConn->hQosMngr			= hQosMngr;
	pConn->hHalCtrl			= hHalCtrl;
	pConn->hScanCnc			= hScanCnc;
	pConn->hCurrBss			= hCurrBss;
	pConn->hSwitchChannel	= hSwitchChannel;
	pConn->hEvHandler		= hEvHandler;
	pConn->hHealthMonitor	= hHealthMonitor;
	pConn->hMacServices 	= hMacServices;
	pConn->hSoftGemini		= hSoftGemini;
    pConn->hRegulatoryDomain = hRegulatoryDomain;
	
	pConn->connType			 = CONN_TYPE_FIRST_CONN;
    pConn->ibssDisconnectCount = 0;

	whalCtrl_EventMbox_RegisterForEvent(pConn->hHalCtrl,  HAL_EVENT_JOIN_CMPLT, 
									(void *)connInfra_JoinCmpltNotification, pConn);

	whalCtrl_EventMbox_Enable(pConn->hHalCtrl, HAL_EVENT_JOIN_CMPLT);
	
	WLAN_REPORT_INIT(hReport, CONN_MODULE_LOG,  (".....Connection configured successfully\n"));

	return OK;
}

/************************************************************************
 *                        conn_unLoad									*
 ************************************************************************
DESCRIPTION: Connection module unload function, called by the config mgr in the unlod phase 
				performs the following:
				-	Free all memory aloocated by the module
                                                                                                   
INPUT:      hConn	-	Connection handle.		


OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS conn_unLoad(TI_HANDLE hConn)
{
	UINT32			initVec;
	conn_t			*pConn = (conn_t *)hConn;

	if (!pConn)
		return OK;

	initVec = 0xFFFF;
	release_module(pConn, initVec);

	return OK;
}

/***********************************************************************
 *                        conn_setParam									
 ***********************************************************************
DESCRIPTION: Connection set param function, called by the following:
				-	config mgr in order to set a parameter from the OS abstraction layer.
				-	Form inside the driver
				In this fuction, the site manager configures the connection type in the select phase.
				The connection type is used to distribute the connection events to the corresponding connection SM	
                                                                                                   
INPUT:      hConn	-	Connection handle.
			pParam	-	Pointer to the parameter		

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS conn_setParam(TI_HANDLE		hConn, 
					 paramInfo_t	*pParam)
{
	conn_t *pConn = (conn_t *)hConn;

	switch(pParam->paramType)
	{
	case CONN_TYPE_PARAM:
		pConn->currentConnType = pParam->content.connType;
		switch (pParam->content.connType)
		{
		case CONNECTION_IBSS:
		case CONNECTION_SELF:
			return conn_ibssConfig(pConn);

		case CONNECTION_INFRA:
			return conn_infraConfig(pConn);

		default:
			WLAN_REPORT_ERROR(pConn->hReport, CONN_MODULE_LOG, ("Set connection type, type is not valid, %d\n\n", pParam->content.connType));
			return PARAM_VALUE_NOT_VALID;
		}

	case CONN_SELF_TIMEOUT_PARAM:
		if ((pParam->content.connSelfTimeout < CONN_SELF_TIMEOUT_MIN) || (pParam->content.connSelfTimeout > CONN_SELF_TIMEOUT_MAX))
			return PARAM_VALUE_NOT_VALID;
		pConn->timeout = pParam->content.connSelfTimeout;
		break;

	default:
		WLAN_REPORT_ERROR(pConn->hReport, CONN_MODULE_LOG, ("Set param, Params is not supported, %d\n\n", pParam->paramType));
		return PARAM_NOT_SUPPORTED;
	}

	return OK;
}

/***********************************************************************
 *                        conn_getParam									
 ***********************************************************************
DESCRIPTION: Connection get param function, called by the following:
			-	config mgr in order to get a parameter from the OS abstraction layer.
			-	Fomr inside the dirver	
                                                                                                   
INPUT:      hConn	-	Connection handle.
			pParam	-	Pointer to the parameter		

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS conn_getParam(TI_HANDLE		hConn, 
					 paramInfo_t	*pParam)
{
	conn_t *pConn = (conn_t *)hConn;

	switch(pParam->paramType)
	{
	case CONN_TYPE_PARAM:
		pParam->content.connType = pConn->currentConnType;
		break;

	case CONN_SELF_TIMEOUT_PARAM:
		pParam->content.connSelfTimeout = pConn->timeout;
		break;
	
	default:
		WLAN_REPORT_ERROR(pConn->hReport, CONN_MODULE_LOG, ("Get param, Params is not supported, %d\n\n", pParam->paramType));
		return PARAM_NOT_SUPPORTED;
	}

	return OK;
}

/***********************************************************************
 *                        conn_start									
 ***********************************************************************
DESCRIPTION: Called by the SME SM in order to start the connection SM
			 This function start the current connection SM	
                                                                                                   
INPUT:      hConn	-	Connection handle.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS conn_start(TI_HANDLE hConn, connType_e connType,
					 	conn_status_callback_t  pConnStatusCB,
						TI_HANDLE connStatCbObj,
						BOOL disConEraseKeys,
						BOOL reNegotiateTspec)
{
	conn_t *pConn = (conn_t *)hConn;
	paramInfoPartial_t param;

	pConn->pConnStatusCB = pConnStatusCB;
	pConn->connStatCbObj = connStatCbObj;

	pConn->connType = connType;
	pConn->disConEraseKeys = disConEraseKeys;
	
	/* Initialize the DISASSOCIATE event parameters to default */ 
	pConn->smContext.disAssocEventReason = STATUS_UNSPECIFIED;
	pConn->smContext.disAssocEventStatusCode  = 0;

	/* If requested, re-negotiate voice TSPEC */
	param.paramType = QOS_MNGR_VOICE_RE_NEGOTIATE_TSPEC;
	param.content.TspecConfigure.voiceTspecConfigure = reNegotiateTspec; 
	param.content.TspecConfigure.videoTspecConfigure = reNegotiateTspec; 
	qosMngr_setParamsPartial(pConn->hQosMngr, &param);

	switch(pConn->currentConnType)
	{
	case CONNECTION_IBSS:
		return conn_ibssSMEvent(&pConn->state, CONN_IBSS_CONNECT, (TI_HANDLE) pConn);

	case CONNECTION_SELF:
		return conn_ibssSMEvent(&pConn->state, CONN_IBSS_CREATE, (TI_HANDLE) pConn);

	case CONNECTION_INFRA:
		return conn_infraSMEvent(&pConn->state, CONN_INFRA_CONNECT, (TI_HANDLE) pConn);

    default:
		WLAN_REPORT_ERROR(pConn->hReport, CONN_MODULE_LOG, ("Start connection, invalid type %d\n\n", pConn->currentConnType));
		return NOK;

	}
}

/***********************************************************************
 *                        conn_stop									
 ***********************************************************************
DESCRIPTION: Called by the SME SM in order to stop the connection SM
			 This function stop the current connection SM.	
                                                                                                   
INPUT:      hConn	-	Connection handle.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS conn_stop(TI_HANDLE 				hConn, 
					disConnType_e 			disConnType, 
					mgmtStatus_e 			reason,
					BOOL					disConEraseKeys,
					conn_status_callback_t  pConnStatusCB,
					TI_HANDLE 				connStatCbObj  )
{
	conn_t *pConn = (conn_t *)hConn;

	pConn->pConnStatusCB = pConnStatusCB;
	pConn->connStatCbObj = connStatCbObj;

	pConn->disConnType 		 = disConnType;
	pConn->disConnReasonToAP = reason;
	pConn->disConEraseKeys	 = disConEraseKeys;

	/* 
	 * Mark the disconnection reason as unspecified to indicate that conn module has no information regarding the DISASSOCIATE event to be raised
	 * by the SME
	 */
	pConn->smContext.disAssocEventReason = STATUS_UNSPECIFIED;
	pConn->smContext.disAssocEventStatusCode  = 0;

    WLAN_REPORT_INFORMATION(pConn->hReport, CONN_MODULE_LOG, ("conn_stop, disConnType %d, reason=%d, disConEraseKeys=%d\n\n", 
                  disConnType, reason, disConEraseKeys));

	switch(pConn->currentConnType)
	{
	case CONNECTION_IBSS:
    case CONNECTION_SELF:
        pConn->ibssDisconnectCount++;
		return conn_ibssSMEvent(&pConn->state, CONN_IBSS_DISCONNECT, (TI_HANDLE) pConn);

	case CONNECTION_INFRA:
		return conn_infraSMEvent(&pConn->state, CONN_INFRA_DISCONNECT, (TI_HANDLE) pConn);


	default:
		WLAN_REPORT_ERROR(pConn->hReport, CONN_MODULE_LOG, ("Stop connection, invalid type %d\n\n", pConn->currentConnType));
		return NOK;
	}
}


/***********************************************************************
 *                        conn_reportMlmeStatus									
 ***********************************************************************
DESCRIPTION:	Called by the MLME SM when MLME status changed. 
				Valid only in the case that the current connection type is infrastructure
				The function calls the connection infra SM with MLME success or MLME failure 
				according to the status
                                                                                                   
INPUT:      hConn	-	Connection handle.
			status	-	MLME status

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS conn_reportMlmeStatus(TI_HANDLE			hConn, 
								mgmtStatus_e		status, UINT16 uStatusCode)
{
	conn_t *pConn = (conn_t *)hConn;

	/* Save the reason for the use of the SME when triggering DISASSOCIATE event */ 
	pConn->smContext.disAssocEventReason = status;
	pConn->smContext.disAssocEventStatusCode = uStatusCode;

	if (status == STATUS_SUCCESSFUL)
	{
		conn_infraSMEvent(&pConn->state, CONN_INFRA_MLME_SUCC, pConn);
	}
	else
	{
		WLAN_OS_REPORT(("-------------------------------------\n"));
		WLAN_OS_REPORT(("               CONN LOST             \n"));
		WLAN_OS_REPORT(("-------------------------------------\n"));

		if( pConn->connType == CONN_TYPE_ROAM )
			pConn->disConnType = DISCONN_TYPE_IMMEDIATE;
		else /* connType == CONN_TYPE_ESS */
			pConn->disConnType = DISCONN_TYPE_DEAUTH;

        WLAN_REPORT_INFORMATION(pConn->hReport, CONN_MODULE_LOG, 
			("conn_reportMlmeStatus, disAssocEventReason %d, disAssocEventStatusCode = %d, connType=%d, disConnType=%d \n", 
            pConn->smContext.disAssocEventReason, pConn->smContext.disAssocEventStatusCode, pConn->connType, pConn->disConnType));

		conn_infraSMEvent(&pConn->state, CONN_INFRA_DISCONNECT, pConn);
	}

	return OK;
}

/***********************************************************************
 *                        conn_reportRsnStatus									
 ***********************************************************************
DESCRIPTION:	Called by the RSN SM when RSN status changed. 
				This function calls the current connection SM with RSN success or RSN failure based on the status	
                                                                                                   
INPUT:      hConn	-	Connection handle.
			status	-	RSN status

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS conn_reportRsnStatus(TI_HANDLE			hConn, 
							mgmtStatus_e		status)
{
	conn_t *pConn = (conn_t *)hConn;

	/* Save the reason for the use of the SME when triggering DISASSOCIATE event. For now we just have STATUS_SECURITY_FAILURE */ 
	pConn->smContext.disAssocEventReason = status;
	pConn->smContext.disAssocEventStatusCode = 0; /* For now we don't use this parameter in RSN */

	switch(pConn->currentConnType)
	{
	case CONNECTION_IBSS:
	case CONNECTION_SELF:
		if (status == STATUS_SUCCESSFUL)
			return conn_ibssSMEvent(&pConn->state, CONN_IBSS_RSN_SUCC, (TI_HANDLE) pConn);
		else
			return conn_ibssSMEvent(&pConn->state, CONN_IBSS_DISCONNECT, (TI_HANDLE) pConn);



	case CONNECTION_INFRA:
		if (status == STATUS_SUCCESSFUL)
			return conn_infraSMEvent(&pConn->state, CONN_INFRA_RSN_SUCC, (TI_HANDLE) pConn);
		
		else{ /* status == STATUS_SECURITY_FAILURE */
			/*
			 * In infrastructure - if the connection is standard 802.11 connection (ESS) then
			 * need to disassociate. In roaming mode, the connection is stopped without sending
			 * the reassociation frame.
			 */
			if( pConn->connType == CONN_TYPE_ROAM )
				pConn->disConnType = DISCONN_TYPE_IMMEDIATE;
			else /* connType == CONN_TYPE_ESS */
				pConn->disConnType = DISCONN_TYPE_DISASSOC;

            WLAN_REPORT_INFORMATION(pConn->hReport, CONN_MODULE_LOG, ("conn_reportRsnStatus, disAssocEventReason %d, connType=%d, disConnType=%d \n\n", 
                          pConn->smContext.disAssocEventReason, pConn->connType, pConn->disConnType));

			return conn_infraSMEvent(&pConn->state, CONN_INFRA_DISCONNECT, (TI_HANDLE) pConn);
		}
	case CONNECTION_NONE:
		break;
	}
	
	return OK;
}

/***********************************************************************
 *                        conn_timeout									
 ***********************************************************************
DESCRIPTION:	Called by the OS abstraction layer when the self timer expired 
				Valid only if the current connection type is self
				This function calls the self connection SM with timeout event
                                                                                                   
INPUT:      hConn	-	Connection handle.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
static void conn_timeout(void *pContext)
{
	conn_t *pConn = (conn_t *)pContext;

	switch(pConn->currentConnType)
	{
	case CONNECTION_IBSS:
	case CONNECTION_SELF:
		conn_ibssSMEvent(&pConn->state, CONN_IBSS_DISCONNECT, pConn);
		break;

	case CONNECTION_INFRA:
		conn_infraSMEvent(&pConn->state, CONN_INFRA_DISCONN_COMPLETE, (TI_HANDLE) pConn);
		healthMonitor_sendFailureEvent(pConn->hHealthMonitor, DISCONNECT_TIMEOUT);
		break;

	case CONNECTION_NONE:
		break;
	}

	return;
}


/***********************************************************************
 *                        conn_join									
 ***********************************************************************
DESCRIPTION:	Called by the site manager when detecting that another station joined our own created IBSS 
				Valid only if the current connection type is self
				This function calls the self connection SM with join event
                                                                                                   
INPUT:      hConn	-	Connection handle.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS conn_ibssStaJoined(TI_HANDLE hConn)
{
	conn_t *pConn = (conn_t *)hConn;
	conn_ibssSMEvent(&pConn->state, CONN_IBSS_STA_JOINED, pConn);
	return OK;
}

/***********************************************************************
 *                        release_module									
 ***********************************************************************
DESCRIPTION:	Called by the un load function
				Go over the vector, for each bit that is set, release the corresponding module.
                                                                                                   
INPUT:      hConn	-	Connection handle.
			initVec	-	Vector that contains a bit set for each module thah had been initiualized

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
static void release_module(conn_t *pConn, UINT32 initVec)
{
	if (initVec & (1 << IBSS_SM_INIT_BIT))
		fsm_Unload(pConn->hOs, pConn->ibss_pFsm);

    if (initVec & (1 << INFRA_SM_INIT_BIT))
		fsm_Unload(pConn->hOs, pConn->infra_pFsm);

	if (initVec & (1 << TIMER_INIT_BIT))
		utils_nullTimerDestroy(pConn->hOs, pConn->pTimer);

    if (initVec & (1 << CONN_INIT_BIT))
		utils_nullMemoryFree(pConn->hOs, pConn, sizeof(conn_t));

	initVec = 0;
}


/***********************************************************************
 *                        conn_waitToDisassoc									
 ***********************************************************************
DESCRIPTION:                 

  INPUT:      hConn	-	Connection handle.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
void conn_disConnFrameSentCBFunc(TI_HANDLE hConn)
{
	conn_t *pConn = (conn_t *)hConn;
	
	conn_infraSMEvent(&pConn->state, CONN_INFRA_DISCONN_COMPLETE, (TI_HANDLE) pConn);
}

/**
*
* conn_ibssPrintStatistics
*
* \b Description: 
*
* Called by Site Manager when request to print statistics is requested from CLI  
*
* \b ARGS: Connection handle
*
* \b RETURNS:
*
*  None.
*
* \sa 
*/
void conn_ibssPrintStatistics(TI_HANDLE hConn)
{
#ifdef REPORT_LOG
    conn_t *pConn = (conn_t *)hConn;

    WLAN_OS_REPORT(("- IBSS Disconnect = %d\n", pConn->ibssDisconnectCount));
    WLAN_OS_REPORT(("\n"));
#endif
}

