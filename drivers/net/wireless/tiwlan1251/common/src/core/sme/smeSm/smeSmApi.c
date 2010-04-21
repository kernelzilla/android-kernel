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

/** \file smeSmApi.c
 *  \brief SME SM API implementation
 * 
 *  The state machine itself is implemented in the file	smeSm.c.				
 *
 *  \see smeSmApi.h
 */

#include "report.h"
#include "osTIType.h"
#include "osApi.h"
#include "smeSm.h"
#include "smeApi.h"
#include "smeSmApi.h"
#include "utils.h"
#include "802_11Defs.h"
#include "regulatoryDomainApi.h"
#include "siteMgrApi.h"
#include "connApi.h"
#include "EvHandler.h"
#include "TI_IPC_Api.h"


#define WLAN_INTER_SCAN_DELTA 10

/* State machine definitions */
#define SME_INIT_BIT			1
#define SM_INIT_BIT				2
#define TIMER_INIT_BIT			3

/* Local functions prototypes */

static void release_module(smeSm_t *pSmeSm, UINT32 initVec);

void smeSm_InterScanTimeoutCB(TI_HANDLE     hSmeSm);


/* Interface functions Implementation */

/************************************************************************
 *                        smeSm_create								*
 ************************************************************************
DESCRIPTION: SME SM module creation function, called by the config mgr in creation phase 
				performs the following:
				-	Allocate the SME SM handle
				-	Create the SME state machine
                                                                                                   
INPUT:      hOs -			Handle to OS		


OUTPUT:		

RETURN:     Handle to the SME SM module on success, NULL otherwise

************************************************************************/
TI_HANDLE smeSm_create(TI_HANDLE hOs)
{
	smeSm_t			*pSmeSm;
	UINT32			initVec;

	initVec = 0;

	pSmeSm = os_memoryAlloc(hOs, sizeof(smeSm_t));
	if (pSmeSm == NULL)
		return NULL;
	os_memoryZero(hOs, pSmeSm, sizeof(smeSm_t)); /* Dm: Fix */
	
	initVec |= (1 << SME_INIT_BIT);

	pSmeSm->pFsm = smeSm_smCreate(hOs);
	if (pSmeSm->pFsm == NULL)
	{
		release_module(pSmeSm, initVec);
		return NULL;
	}
	
	initVec |= (1 << SM_INIT_BIT);

	pSmeSm->hOs	= hOs;

	pSmeSm->interScanTimeoutTimer = os_timerCreate(hOs, smeSm_InterScanTimeoutCB, pSmeSm);
	if(pSmeSm->interScanTimeoutTimer == NULL)
	{
		release_module(pSmeSm, initVec);
	    WLAN_OS_REPORT(("FATAL ERROR: smeSm_create(): Error Creating smeSm - Aborting\n"));
		return NULL;
	}
	initVec |= (1 << TIMER_INIT_BIT);

	return(pSmeSm);
}

/************************************************************************
 *                        smeSm_config									*
 ************************************************************************
DESCRIPTION: SME SM module configuration function, called by the config mgr in configuration phase
				performs the following:
				-	Reset & initiailzes local variables
				-	Init the handles to be used by the module
                                                                                                   
INPUT:      hSmeSm	-	SME SM handle
			List of handles to be used by the module

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS smeSm_config(TI_HANDLE		hSmeSm, 
					TI_HANDLE		    hConn,
					TI_HANDLE		    hScanCncn,
					TI_HANDLE		    hSiteMgr,
					TI_HANDLE		    hHalCtrl,
					TI_HANDLE		    hReport,
					TI_HANDLE 		    hOs,
                    TI_HANDLE		    hEvHandler,
					TI_HANDLE		    hScr,
					TI_HANDLE		    hApConn,
					TI_HANDLE		    hCurrBss,
					TI_HANDLE		    hPowerMgr,
                    TI_HANDLE           hRegulatoryDomain,
					smeInitParams_t*	smeInitParams)

{
	TI_STATUS status;
	int index;
	
	smeSm_t *pSmeSm = (smeSm_t *)hSmeSm;

	pSmeSm->state = SME_SM_STATE_IDLE;
	pSmeSm->hConn			= hConn;
	pSmeSm->hScanCncn		= hScanCncn;
	pSmeSm->hSiteMgr		= hSiteMgr;
	pSmeSm->hHalCtrl		= hHalCtrl;
	pSmeSm->hReport			= hReport;
	pSmeSm->hOs				= hOs;
    pSmeSm->hEvHandler  	= hEvHandler;
	pSmeSm->hScr			= hScr;
	pSmeSm->hApConn			= hApConn;
	pSmeSm->hCurrBss		= hCurrBss;
	pSmeSm->hPowerMgr		= hPowerMgr;
    pSmeSm->hRegulatoryDomain = hRegulatoryDomain;

	/* interscan timeout values */
	pSmeSm->scanEnabled				= (scanEnabledOptions_e)smeInitParams->EnableFirstConnScan; 
    pSmeSm->interScanTimeoutMin 	= smeInitParams->InterScanIntervalMin;
    pSmeSm->interScanTimeoutMax 	= smeInitParams->InterScanIntervalMax;
    pSmeSm->interScanTimeoutDelta 	= smeInitParams->InterScanIntervalDelta;
    pSmeSm->shutDownStatus = 0;


	/* 
	 *    Setting scan parameters for band 2.4Ghtz
	 */
	os_memoryCopy(hOs, &(pSmeSm->scanParamsBG), &(smeInitParams->scanParamsBG), sizeof(sme_scan_Params_t)); 
	/* The channel list is represented as char string terminate in zeros. */
	
	for( index = 0; 
		 ((index < MAX_NUMBER_OF_CHANNELS_PER_SCAN )&&(pSmeSm->scanParamsBG.channelsList[index] != 0)); 
		 index++ );

	pSmeSm->scanParamsBG.numOfChannels = index;

	/* 
	 *    Setting scan parameters for band 5.0Ghtz
	 */
	os_memoryCopy(hOs, &(pSmeSm->scanParamsA), &(smeInitParams->scanParamsA), sizeof(sme_scan_Params_t));
	
	for( index = 0; 
		 ((index < MAX_NUMBER_OF_CHANNELS_PER_SCAN )&&(pSmeSm->scanParamsA.channelsList[index] != 0)); 
		 index++ );

	pSmeSm->scanParamsA.numOfChannels = index;


    
    /* register to scan result callback */
    scanConcentrator_registerScanResultCB( pSmeSm->hScanCncn, SCAN_SCC_DRIVER, smeSm_scanComplete, hSmeSm );

	status = smeSm_smConfig(pSmeSm);

	if (status != OK)
		WLAN_REPORT_INIT(hReport, SME_SM_MODULE_LOG,  (".....Sme state machine configuration Failure\n"));
	else
		WLAN_REPORT_INIT(hReport, SME_SM_MODULE_LOG,  (".....Sme state machine configuration Success\n"));

	return status;
}

/************************************************************************
 *                        smeSm_getDriverShutdownStatus									*
 ************************************************************************
DESCRIPTION: Return shutdown status of driver.
                                                                                                   
INPUT:      hSmeSm	-	SME SM handle.		

OUTPUT:		

RETURN:     shutdown status of driver (SME/HAL)

************************************************************************/
UINT8 smeSm_getDriverShutdownStatus (TI_HANDLE		hSmeSm)
{
	smeSm_t			*pSmeSm = (smeSm_t *)hSmeSm;
    return (pSmeSm->shutDownStatus);
}


/************************************************************************
 *                        smeSm_unLoad									*
 ************************************************************************
DESCRIPTION: SME SM module unload function, called by the config mgr in the unlod phase 
				performs the following:
				-	Free all memory allocated by the module
                                                                                                   
INPUT:      hSmeSm	-	SME SM handle.		


OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS smeSm_unLoad(TI_HANDLE		hSmeSm)
{
	UINT32			initVec;
	smeSm_t			*pSmeSm = (smeSm_t *)hSmeSm;

	if (!pSmeSm)
		return OK;

	initVec = 0xFFFF;
	release_module(pSmeSm, initVec);

	return OK;
}

/***********************************************************************
 *                        smeSm_start									
 ***********************************************************************
DESCRIPTION: Called by the configuration module in order to start the driver
			 Calls the SME SM with a start event
                                                                                                   
INPUT:      hSmeSm	-	SME SM handle.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS smeSm_start(TI_HANDLE		hSmeSm)
{
	smeSm_t *pSmeSm = (smeSm_t *)hSmeSm;

	pSmeSm->radioOn = TRUE;
    pSmeSm->immediateShutdownRequired = FALSE;

	return smeSm_SMEvent(&pSmeSm->state, SME_SM_EVENT_START, pSmeSm);
}



/***********************************************************************
 *                        smeSm_restart									
 ***********************************************************************
DESCRIPTION: Called by the configuration module in order to start the driver
			 Calls the SME SM with a start event
                                                                                                   
INPUT:      hSmeSm	-	SME SM handle.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS smeSm_reselect(TI_HANDLE		hSmeSm)
{
	smeSm_t *pSmeSm = (smeSm_t *)hSmeSm;
	paramInfo_t			param;

	/* For new SSID reset interScanTimeout */
	pSmeSm->interScanTimeout = pSmeSm->interScanTimeoutMin;

	/* 
		Junk SSID is used for disabling connection attempts, if it is 
	   set the driver will be stopped at "inter scan" state.
	*/    

	param.paramType = SITE_MGR_DESIRED_SSID_PARAM;
	siteMgr_getParam(pSmeSm->hSiteMgr, &param);

	if (utils_isJunkSSID(&param.content.siteMgrDesiredSSID))
	{
		pSmeSm->connectEnabled    = FALSE;

		WLAN_REPORT_INFORMATION(pSmeSm->hReport, SME_SM_MODULE_LOG,  
					("Sme Set JUNK SSID\n"));
		
		if( pSmeSm->state == SME_SM_STATE_SCANNING )
			/* If in scanning stop the scan, the disconnect event will
			   be sent by the scan complete function. */
			scanConcentrator_stopScan( pSmeSm->hScanCncn, SCAN_SCC_DRIVER );
		else
			smeSm_SMEvent(&pSmeSm->state, SME_SM_EVENT_DISCONNECT, pSmeSm);
	}
	else 
	{
		pSmeSm->connectEnabled    = TRUE;
		smeSm_SMEvent(&pSmeSm->state, SME_SM_EVENT_RESELECT, pSmeSm);
	}

	return OK;
}


/***********************************************************************
 *                        smeSm_stop									
 ***********************************************************************
DESCRIPTION: Called by the configuration module in order to stop the driver
			 Calls the SME SM with a stop event
                                                                                                   
INPUT:      hSmeSm	-	SME SM handle.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS smeSm_stop(TI_HANDLE		hSmeSm)
{
	smeSm_t *pSmeSm = (smeSm_t *)hSmeSm;

	pSmeSm->radioOn = FALSE;
    pSmeSm->immediateShutdownRequired = FALSE;    

	return smeSm_SMEvent(&pSmeSm->state, SME_SM_EVENT_STOP, pSmeSm);
}

/***********************************************************************
 *                        smeSm_stopAndShutdown									
 ***********************************************************************
DESCRIPTION: Called by the configuration module in order to stop the driver
			 Calls the SME SM with a stop event
                                                                                                   
INPUT:      hSmeSm	-	SME SM handle.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
void smeSm_stopAndShutdown(TI_HANDLE		hSmeSm)
{
	smeSm_t *pSmeSm = (smeSm_t *)hSmeSm;

	pSmeSm->radioOn = FALSE;
    pSmeSm->immediateShutdownRequired = TRUE;

	smeSm_SMEvent(&pSmeSm->state, SME_SM_EVENT_STOP, pSmeSm);
}

/***********************************************************************
 *                        smeSm_scanComplete									
 ***********************************************************************
DESCRIPTION: Called by the site manager When scan is completed
			 Calls the SME SM with a scan complete event
                                                                                                   
INPUT:      hSmeSm	-	SME SM handle.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
void smeSm_scanComplete( TI_HANDLE hSmeSm, scan_cncnResultStatus_e status,
                         scan_frameInfo_t *frameInfo, UINT16 SPSStatus )
{
	smeSm_t *pSmeSm = (smeSm_t *)hSmeSm;

    /* if this call is due to a scan result received, simply store it in the site mngr */
	if ( SCAN_CRS_RECEIVED_FRAME == status )
    {
		siteMgr_updateSite( pSmeSm->hSiteMgr, frameInfo->bssId, frameInfo->parsedIEs, frameInfo->channel, frameInfo->band, FALSE);
        if ( BEACON == frameInfo->parsedIEs->subType )
        {
            siteMgr_saveBeaconBuffer( pSmeSm->hSiteMgr, frameInfo->bssId, frameInfo->buffer, frameInfo->bufferLength );
        }
        else
        {
            siteMgr_saveProbeRespBuffer( pSmeSm->hSiteMgr, frameInfo->bssId, frameInfo->buffer, frameInfo->bufferLength );
        }
#ifdef TI_DBG
		/* update statistics - count one more result that was received */
		pSmeSm->smeStats.currentNumberOfScanResults++;
#endif
        return;
    }

#ifdef TI_DBG
	/* update statistics - update scan results histogram */
	if ( SCAN_RESULT_HISTOGRAM_SIZE <= pSmeSm->smeStats.currentNumberOfScanResults )
	{
		pSmeSm->smeStats.scanResulCountHistogram[ SCAN_RESULT_HISTOGRAM_SIZE -1 ]++;
	}
	else
	{
		pSmeSm->smeStats.scanResulCountHistogram[ pSmeSm->smeStats.currentNumberOfScanResults ]++;
	}
	pSmeSm->smeStats.currentNumberOfScanResults = 0;
#endif

	WLAN_REPORT_INFORMATION(pSmeSm->hReport, SME_SM_MODULE_LOG,  
				("smeSm_scanComplete\n"));

    siteMgr_removeNotReceivedSites(pSmeSm->hSiteMgr);

	if ( pSmeSm->connectEnabled ) 
    {
	    /* check for rescan and perform scan when it is on */
		if ( TRUE == pSmeSm->reScanFlag )
        {
		    WLAN_REPORT_INFORMATION( pSmeSm->hReport, SME_SM_MODULE_LOG,
                                     ("SME_SM: doing additional scan due to reScanFlag = ON\n") );
		    pSmeSm->reScanFlag = FALSE;
			sme_startScan(pSmeSm);
		} 
        /* check for dual band rescan */
		else if ( TRUE == pSmeSm->dualBandReScanFlag )
        {
		    WLAN_REPORT_INFORMATION( pSmeSm->hReport, SME_SM_MODULE_LOG,
                                     ("SME_SM: doing additional scan due to dualBandReScanFlag = ON\n") );
			sme_startScan(pSmeSm);
        }
        else
        {
			smeSm_SMEvent(&pSmeSm->state, SME_SM_EVENT_SCAN_COMPLETE, pSmeSm);
		}
	} 
	else
    {
		/* If connection is disabled then send disconnect event, the SM will
		 * move into inter scan state
		 */
		pSmeSm->reScanFlag = FALSE;	/* (Just to make sure) */
        pSmeSm->dualBandReScanFlag = FALSE;
		smeSm_SMEvent(&pSmeSm->state, SME_SM_EVENT_DISCONNECT, pSmeSm);
	}
}


/***********************************************************************
 *                        smeSm_reportConnStatus					
 ***********************************************************************
DESCRIPTION: Called by the connection module when connection status changes
			 Calls the SME SM with a connection suceess or connection failure based on the status
                                                                                                   
INPUT:      hSmeSm		-	SME SM handle.
			statusType	-	Connection status
			uStatusCode -	extra information to statusType (usually status code of the packet)

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS smeSm_reportConnStatus(TI_HANDLE	hSmeSm, mgmtStatus_e statusType, UINT32 uStatusCode)							 
{
	smeSm_t *pSmeSm = (smeSm_t *)hSmeSm;

	WLAN_REPORT_INFORMATION(pSmeSm->hReport, SME_SM_MODULE_LOG,  
		("%s statusType = %d, uStatusCode = %d \n",__FUNCTION__, statusType, uStatusCode));

	switch(statusType)
	{
		case STATUS_SUCCESSFUL:
			return smeSm_SMEvent(&pSmeSm->state, SME_SM_EVENT_CONN_SUCCESS, pSmeSm);

		/*
		*  The next section handles connection failures, all cases are sending same event to SM. 
		*/              
		case		STATUS_AUTH_REJECT:										
		case		STATUS_ASSOC_REJECT:										
		case		STATUS_SECURITY_FAILURE:                                   
		case		STATUS_AP_DEAUTHENTICATE:	
		case		STATUS_AP_DISASSOCIATE:
		case		STATUS_ROAMING_TRIGGER:		
			pSmeSm->DisAssoc.mgmtStatus  = statusType;
			pSmeSm->DisAssoc.uStatusCode = uStatusCode;
			/* Note that in case of unspecified status we won't update the status. This is done since this function could be called twice */
			/* for example: apConn called this function and than SME called conn_stop and this function is called again					  */
		case		STATUS_UNSPECIFIED:                                        

			return smeSm_SMEvent(&pSmeSm->state, SME_SM_EVENT_CONN_FAILURE, pSmeSm);

	default:
		WLAN_REPORT_WARNING(pSmeSm->hReport, SME_SM_MODULE_LOG,  
			("%s unknown statusType = %d\n",__FUNCTION__, statusType));

		break;
	}

	return OK;
}

/***********************************************************************
 *                        smeSm_reportSelectStatus									
 ***********************************************************************
DESCRIPTION: Called by the selection function
			 Calls the SME SM with a selection suceess or selection failure based on the status
                                                                                                   
INPUT:      hSmeSm	-	SME SM handle.
			status	-	selection status

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS smeSm_reportSelectStatus(TI_HANDLE		hSmeSm, 
								mgmtStatus_e	status)
{
	smeSm_t *pSmeSm = (smeSm_t *)hSmeSm;
	
	if (status == SELECT_STATUS_SUCCESS)
		return smeSm_SMEvent(&pSmeSm->state, SME_SM_EVENT_SELECT_SUCCESS, pSmeSm);
	else
		return smeSm_SMEvent(&pSmeSm->state, SME_SM_EVENT_SELECT_FAILURE, pSmeSm);
}


/***********************************************************************
 *                        smeSm_startScan									
 ***********************************************************************
DESCRIPTION: Timer callback, on expiration of which, scan started
                                                                                                   
INPUT:      hSmeSm	-	SME SM handle.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
void smeSm_InterScanTimeoutCB(TI_HANDLE     hSmeSm)
{
	smeSm_t *pSmeSm = (smeSm_t *)hSmeSm;

	if ( pSmeSm->connectEnabled )
	{
		pSmeSm->interScanTimeout += pSmeSm->interScanTimeoutDelta;

		if( pSmeSm->interScanTimeout > pSmeSm->interScanTimeoutMax )
			pSmeSm->interScanTimeout = pSmeSm->interScanTimeoutMax;

		smeSm_SMEvent(&pSmeSm->state, SME_SM_EVENT_RESELECT, pSmeSm);
	}
}


/***********************************************************************
 *                        release_module									
 ***********************************************************************
DESCRIPTION:	Called by the un load function
				Go over the vector, for each bit that is set, release the corresponding module.
                                                                                                   
INPUT:      hConn	-	SME SM handle.
			pSmeSm	-	Vector that contains a bit set for each module thah had been initiualized

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
static void release_module(smeSm_t *pSmeSm, UINT32 initVec)
{

	if (initVec & (1 << SM_INIT_BIT))
		smeSm_smUnLoad(pSmeSm->hOs, pSmeSm->pFsm);

	if (initVec & (1 << TIMER_INIT_BIT))
	{
		os_timerStop(pSmeSm->hOs, pSmeSm->interScanTimeoutTimer);
		utils_nullTimerDestroy(pSmeSm->hOs, pSmeSm->interScanTimeoutTimer);
	}

	if (initVec & (1 << SME_INIT_BIT))
		utils_nullMemoryFree(pSmeSm->hOs, pSmeSm, sizeof(smeSm_t));


	initVec = 0;
}

/***********************************************************************
 *                        smeSm_setParam									
 ***********************************************************************
DESCRIPTION: SME SM set param function, called by the following:
				-	config mgr in order to set a parameter from the OS abstraction layer.
				-	Form inside the driver
                                                                                                   
INPUT:      hSmeSm	-	SME SM handle.
			pParam	-	Pointer to the parameter		

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS smeSm_setParam(TI_HANDLE			hSmeSm,
						paramInfo_t		*pParam)
{
	smeSm_t *pSmeSm = (smeSm_t *)hSmeSm;

	switch(pParam->paramType)
	{
	case SME_SCAN_ENABLED_PARAM:
		if (pSmeSm->scanEnabled != pParam->content.smeSMScanEnabled)
		{
			if ((pParam->content.smeSMScanEnabled == SCAN_ENABLED) &&
				(pSmeSm->scanEnabled == SKIP_NEXT_SCAN))
			{
				/* Requested to st scanEnable to TRUE; 
				  if we are about to skip the nextcoming scan, ignore the request */
				break;
			}
			if ((pParam->content.smeSMScanEnabled == SKIP_NEXT_SCAN) &&
				(pSmeSm->scanEnabled == SCAN_DISABLED))
			{
				/* Requested to st scanEnable to SKIP_NEXT_SCAN 
				   while it is currently set to FALSE - error, ignore the request */
				WLAN_REPORT_ERROR( pSmeSm->hReport, SME_SM_MODULE_LOG, 
								   ("Set param, error changing scan enabled param from %d to %d\n",
									pSmeSm->scanEnabled, pParam->content.smeSMScanEnabled));
				break;
			}
			pSmeSm->scanEnabled = pParam->content.smeSMScanEnabled;
		}
		break;

	default:
		WLAN_REPORT_ERROR( pSmeSm->hReport, SME_SM_MODULE_LOG, 
						   ("Set param, Params is not supported, %d\n\n", pParam->paramType));
		return PARAM_NOT_SUPPORTED;
	}

	return OK;
}

/***********************************************************************
 *                        smeSm_getParam									
 ***********************************************************************
DESCRIPTION: SME SM get param function, called by the following:
			-	config mgr in order to get a parameter from the OS abstraction layer.
			-	Fomr inside the dirver	
                                                                                                   
INPUT:      hSmeSm	-	SME SM handle.
			pParam	-	Pointer to the parameter		

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS smeSm_getParam(TI_HANDLE			hSmeSm, 
						paramInfo_t		*pParam)
{
	smeSm_t *pSmeSm = (smeSm_t *)hSmeSm;

	switch(pParam->paramType)
	{
	case SITE_MGR_CONNECTION_STATUS_PARAM:
		switch (pSmeSm->state)
		{
		case SME_SM_STATE_IDLE:
		case SME_SM_STATE_INTER_SCAN:
			pParam->content.smeSmConnectionStatus = eDot11Idle;
			break;
		case SME_SM_STATE_SCANNING:
			pParam->content.smeSmConnectionStatus = eDot11Scaning;
			break;
		case SME_SM_STATE_CONNECTING:
			pParam->content.smeSmConnectionStatus = eDot11Connecting;
			break;
		default: 
			pParam->content.smeSmConnectionStatus = eDot11Associated;
			break;
		}
		break;

	case SME_SM_STATE_PARAM:
		pParam->content.smeSmState = pSmeSm->state;
		break;

	case SME_SCAN_ENABLED_PARAM:
		pParam->content.smeSMScanEnabled = pSmeSm->scanEnabled;
		break;

	default:
		WLAN_REPORT_ERROR(pSmeSm->hReport, SME_SM_MODULE_LOG, ("Get param, Params is not supported, %d\n\n", pParam->paramType));
		return PARAM_NOT_SUPPORTED;
	}

	return OK;
}
