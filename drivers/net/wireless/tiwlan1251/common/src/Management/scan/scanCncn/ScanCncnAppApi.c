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

/** \file ScanCncnAppApi.c
 *  \brief This file include implementation of application scan requests adapter.\n
 *  \author Ronen Kalish
 *  \date 30-Jan-2005
 */

#include "ScanCncnAppApi.h"
#include "ScanCncn.h"
#include "EvHandler.h"
#include "report.h"
#include "smeApi.h"
#include "siteMgrApi.h"
#include "ScanCncnOidSM.h"

/**
 * \author Ronen Kalish\n
 * \date 30-Jan-2005\n
 * \brief Parses and executes a set param command.\n
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param pParam - the param to set.\n
 * \return OK if the scan started successfully, NOK otherwise.\n
 */
TI_STATUS scanConcentrator_setParam( TI_HANDLE hScanCncn, paramInfo_t *pParam )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t *)hScanCncn;
	scan_Params_t* pScanParam; 
	UINT32 currentTimeStamp;

    switch ( pParam->paramType )
    {
	case SCAN_CNCN_START_APP_SCAN:
		pScanParam = pParam->content.pScanParams;
		/*
		 * Prepare scan complete's aging, by increasing the scanned sites 
		 * scan attemps counter. The counter will be checked upon scan complete,  
		 * and the sites with no update scan results will be dropped.   
		 */
		siteMgr_setNotReceivedParameter( pScanConcentrator->hSiteManager, &(pScanParam->desiredSsid), pScanParam->band );
        
		if ( SCAN_CRS_SCAN_RUNNING != 
                scanConcentrator_scan( hScanCncn, SCAN_SCC_APP, pScanParam ) )
        {
            /* Scan was not started successfully, send a scan complete event to the user */
            EvHandlerSendEvent( pScanConcentrator->hEventSrv, IPC_EVENT_SCAN_COMPLETE, NULL, 0 );
            return NOK;
        }
        break;

    case SCAN_CNCN_STOP_APP_SCAN:
        scanConcentrator_stopScan( hScanCncn, SCAN_SCC_APP );
        break;
	case SCAN_CNCN_BSSID_LIST_SCAN_PARAM:
		/* check if OID scans are enabled in the registry */
		if ( 0 == pScanConcentrator->initParams.minimumDurationBetweenOidScans )
		{
			WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
									 ("Received OID scan request when OID scans are disabled, quitting...\n") );
			return OK; /* TODO ronen: return correct Windows value */
		}

		/* check if the last OID scan didn't start at a shorter duration than the configured minimum */
		currentTimeStamp = os_timeStampMs( pScanConcentrator->hOS );
		if ( (currentTimeStamp - pScanConcentrator->oidScanLastTimeStamp) < 
			 (pScanConcentrator->initParams.minimumDurationBetweenOidScans * 1000) ) /*converted to ms */
		{
			WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
									 ("Last OID scan performed at:%d, now is:%d, min duration is %d, too early for another scan!\n", \
									  pScanConcentrator->oidScanLastTimeStamp, currentTimeStamp, pScanConcentrator->initParams.minimumDurationBetweenOidScans) );
			return OK; /* TODO ronen: return correct Windows value */
		}

		/* mark that an OID scan process has started */
		pScanConcentrator->bOidScanRunning = TRUE;
		pScanConcentrator->oidScanLastTimeStamp = currentTimeStamp;
		WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
   								 ("Starting OID scan process...\n") );

		/* and actually start the scan */
		scanConcentratorOidSM_SMEvent( hScanCncn, (scan_oidSMStates_e*)&(pScanConcentrator->oidSMState), OID_SCAN_EVENT_START_SCAN );
		break;
        
    default:
	    WLAN_REPORT_ERROR( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG, 
                           ("Set param, Params is not supported:%d\n\n", pParam->paramType) );
	    return PARAM_NOT_SUPPORTED;
    }

    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 30-Jan-2005\n
 * \brief Parses and executes a get param command.\n
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param pParam - the param to get.\n
 * \return always PARAM_NOT_SUPPORTED (not supposed to be called).\n
 */
TI_STATUS scanConcentrator_getParam( TI_HANDLE hScanCncn, paramInfo_t *pParam )
{
    return PARAM_NOT_SUPPORTED;
}

/**
 * \author Ronen Kalish\n
 * \date 30-Jan-2005\n
 * \brief Scan result callback for application scan.\n
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param status - the scan result status (scan complete, result received etc.).\n
 * \param frameInfo - a pointer to the structure holding all frame related info (in case a frame was received).\n
 * \prama SPSStatus - a bitmap indicating on which channels scan was attempted (valid for SPS scan only!).\n
 */
void scanConcentrator_appScanResultCB( TI_HANDLE hScanCncn, scan_cncnResultStatus_e status,
                                       scan_frameInfo_t* frameInfo, UINT16 SPSStatus )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    switch ( status )
	{
	case SCAN_CRS_RECEIVED_FRAME:
        /* forward scan results to site manager, like in the good old days... */
        siteMgr_updateSite( pScanConcentrator->hSiteManager, frameInfo->bssId, 
                            frameInfo->parsedIEs, frameInfo->channel, frameInfo->band, FALSE );
        if ( BEACON == frameInfo->parsedIEs->subType )
        {
            siteMgr_saveBeaconBuffer( pScanConcentrator->hSiteManager, frameInfo->bssId, 
                                      frameInfo->buffer, frameInfo->bufferLength );
        }
        else
        {
            siteMgr_saveProbeRespBuffer( pScanConcentrator->hSiteManager, frameInfo->bssId,
                                         frameInfo->buffer, frameInfo->bufferLength );
        }		
		break;

	case SCAN_CRS_SCAN_COMPLETE_OK:
		siteMgr_removeNotReceivedSites( pScanConcentrator->hSiteManager );

	/* There's no break on purpose! */
		/* if the current running app scan is an OID scan, send a scan complete event to its state machine */
		if ( TRUE == pScanConcentrator->bOidScanRunning )
		{
			scanConcentratorOidSM_SMEvent(hScanCncn, (scan_oidSMStates_e*)&(pScanConcentrator->oidSMState), OID_SCAN_EVENT_SCAN_COMPLETE );
		}
		else
		{
			/* send a scan complete event to the user */
			EvHandlerSendEvent( pScanConcentrator->hEventSrv, IPC_EVENT_SCAN_COMPLETE, NULL, 0 );
		}
		break;
	case SCAN_CRS_TSF_ERROR:
	case SCAN_CRS_SCAN_STOPPED:
	case SCAN_CRS_SCAN_RUNNING:
	case SCAN_CRS_SCAN_FAILED:
	case SCAN_CRS_SCAN_ABORTED_HIGHER_PRIORITY:
	case SCAN_CRS_SCAN_ABORTED_FW_RESET:
	case SCAN_CRS_NUM_OF_RES_STATUS:
	default:
        /* The scan was finished, send a scan complete event to the user
		   (regardless of why the scan was completed) */
		if ( TRUE == pScanConcentrator->bOidScanRunning )
		{
			scanConcentratorOidSM_SMEvent(hScanCncn, (scan_oidSMStates_e*)&(pScanConcentrator->oidSMState), OID_SCAN_EVENT_SCAN_FAILED );
		}
		else
		{
        EvHandlerSendEvent( pScanConcentrator->hEventSrv, IPC_EVENT_SCAN_COMPLETE, NULL, 0 );
		}
		break;
	}
}

