/** \file ScanCncn.c
 *  \brief This file include the scan concentrator module implementation
 *  \author Ronen Kalish
 *  \date 03-Jan-2005
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

#include "ScanCncn.h"
#include "ScanCncnAppSM.h"
#include "ScanCncnDrvSM.h"
#include "ScanCncnRoamingContSM.h"
#include "ScanCncnRoamingImmedSM.h"
#include "ScanCncnAppApi.h"
#include "ScanCncnOidSM.h"  
#include "report.h"
#include "fsm.h"
#include "scrApi.h"
#include "paramIn.h"
#include "regulatoryDomainApi.h"
#include "siteMgrApi.h"
#include "siteHash.h"
#include "utils.h"
#include "healthMonitor.h"

/* static functions */
static void scanConcentrator_SGupdateScanParams( TI_HANDLE hScanCncn, scan_Params_t* pScanParams );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Creates the scan concentrator object
 *
 * Function Scope \e Public.\n
 * \param hOS - handle to the OS object.\n
 * \return a handle to the scan SRV object, NULL if an error occurred.\n
 */
TI_HANDLE scanConcentrator_create( TI_HANDLE hOS )
{
    UINT16   initVector = 0;

    /* allocate the scan concentrator object */
    scanConcentrator_t *pScanConcentrator = os_memoryAlloc( hOS, sizeof(scanConcentrator_t) );
    if ( NULL == pScanConcentrator )
    {
        WLAN_OS_REPORT( ("ERROR: Failed to create scan concnetrator module\n") );
        return NULL;
    }
    initVector |= (1 << SCAN_ALLOC_OBJECT);

    /* store the OS handle */
    pScanConcentrator->hOS = hOS;

    /* create state machines */
    if ( OK != fsm_Create( hOS, &(pScanConcentrator->clientSM[ SCAN_SCC_APP ]),
                           APP_SCAN_NUM_OF_STATES, APP_SCAN_NUM_OF_EVENTS ) )
    {
        WLAN_OS_REPORT( ("ERROR: Failed to allocate application scan state machine\n") );
        scanConcentrator_freeMem( pScanConcentrator, initVector );
        return NULL;
    }
    initVector |= (1 << SCAN_ALLOC_APP_SM);

    if ( OK != fsm_Create( hOS, &(pScanConcentrator->clientSM[ SCAN_SCC_DRIVER ]),
                           DRV_SCAN_NUM_OF_STATES, DRV_SCAN_NUM_OF_EVENTS ) )
    {
        WLAN_OS_REPORT( ("ERROR: Failed to allocate driver scan state machine\n") );
        scanConcentrator_freeMem( pScanConcentrator, initVector );
        return NULL;
    }
    initVector |= (1 << SCAN_ALLOC_DRV_SM);

    if ( OK != fsm_Create( hOS, &(pScanConcentrator->clientSM[ SCAN_SCC_ROAMING_CONT ]),
                           CONT_SCAN_NUM_OF_STATES, CONT_SCAN_NUM_OF_EVENTS ) )
    {
        WLAN_OS_REPORT( ("ERROR: Failed to allocate continuous scan for roaming state machine\n") );
        scanConcentrator_freeMem( pScanConcentrator, initVector );
        return NULL;
    }
    initVector |= (1 << SCAN_ALLOC_CONT_SM);

    if ( OK != fsm_Create( hOS, &(pScanConcentrator->clientSM[ SCAN_SCC_ROAMING_IMMED ]),
                           IMMED_SCAN_NUM_OF_STATES, IMMED_SCAN_NUM_OF_EVENTS ) )
    {
        WLAN_OS_REPORT( ("ERROR: Failed to allocate immediate scan for roaming state machine\n") );
        scanConcentrator_freeMem( pScanConcentrator, initVector );
        return NULL;
    }
    initVector |= (1 << SCAN_ALLOC_IMMED_SM);
	if ( OK != fsm_Create( hOS, &(pScanConcentrator->hOidSM), OID_SCAN_NUM_OF_STATES, OID_SCAN_NUM_OF_EVENTS) )
	{
        WLAN_OS_REPORT( ("ERROR: Failed to allocate OID scan state machine\n") );
        scanConcentrator_freeMem( pScanConcentrator, initVector );
        return NULL;
	}
    return pScanConcentrator;
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Finalizes the scan concentrator object (freeing system resources)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 */
void scanConcentrator_release( TI_HANDLE hScanCncn )
{
    scanConcentrator_freeMem( hScanCncn, 0xffff );
}

/**
 * \author Ronen Kalish\n
 * \date 05-Jan-2005\n
 * \brief Frees the scan concentrator memory, according to the init vector.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param initVec - a vector holding on bits for allocated components and off bits for non allocated components
 */
void scanConcentrator_freeMem( TI_HANDLE hScanCncn, UINT16 initVec )
{
    scanConcentrator_t *pScanConcentrator= (scanConcentrator_t*)hScanCncn;
    
    /* free state machines */
    if ( initVec & (1<<SCAN_ALLOC_APP_SM) )
    {
        fsm_Unload( pScanConcentrator->hOS, pScanConcentrator->clientSM[ SCAN_SCC_APP ] );
    }
    if ( initVec & (1 << SCAN_ALLOC_DRV_SM) )
    {
        fsm_Unload( pScanConcentrator->hOS, pScanConcentrator->clientSM[ SCAN_SCC_DRIVER ] );
    }
    if ( initVec & (1 << SCAN_ALLOC_CONT_SM) )
    {
        fsm_Unload( pScanConcentrator->hOS, pScanConcentrator->clientSM[ SCAN_SCC_ROAMING_CONT ] );
    }
    if ( initVec & (1 << SCAN_ALLOC_IMMED_SM) )
    {
        fsm_Unload( pScanConcentrator->hOS, pScanConcentrator->clientSM[ SCAN_SCC_ROAMING_IMMED ] );
    }
	if ( initVec & (1 << SCAN_ALLOC_OID_SM) )
	{
		fsm_Unload( pScanConcentrator->hOS, pScanConcentrator->hOidSM );
	}
    /* free scan concentrator object */
    if ( initVec & (1 << SCAN_ALLOC_OBJECT) )
    {
        os_memoryFree( pScanConcentrator->hOS, pScanConcentrator, sizeof(scanConcentrator_t) );
    }
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Initialize the scan concentrator object.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param hHalCtrl - handle to the Hal Ctrl object.\n
 * \param hReport - handle to the Report object.\n
 * \param hRegDomain - handle to the regulatory domain object.\n
 * \param hSiteMngr - handle to the site manager object.\n
 * \param hSCR - handle to the SCR object.\n
 * \param hMacServices - handle to the MacServices object.\n
 * \param hAPConn - handle to the AP connection object.\n
 * \param hEventSRV - handle to the event SRV object.\n
 * \param hMlme - handle to the MLME object.\n
 * \param hHealthMonitor - handle to the health monitor object.\n
 * \param pScanConcentratorInitParams - pointer to the init parameters structure.\n
 */
void scanConcentrator_init( TI_HANDLE hScanCncn,
                            TI_HANDLE hHalCtrl,
                            TI_HANDLE hReport,
                            TI_HANDLE hRegDomain,
                            TI_HANDLE hSiteMngr,
                            TI_HANDLE hSCR,
                            TI_HANDLE hMacServices,
                            TI_HANDLE hAPConn,
                            TI_HANDLE hEventSRV,
                            TI_HANDLE hMlme,
                            TI_HANDLE hCtrlData,
                            TI_HANDLE hHealthMonitor,
                            scanConcentratorInitParams_t* pScanConcentratorInitParams )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    /* copy handles */
    pScanConcentrator->hReport = hReport;
    pScanConcentrator->hHalCtrl = hHalCtrl;
    pScanConcentrator->hRegulatoryDomain = hRegDomain;
    pScanConcentrator->hSiteManager = hSiteMngr;
    pScanConcentrator->hSCR = hSCR;
    pScanConcentrator->hMacServices = hMacServices;
    pScanConcentrator->hAPConn = hAPConn;
    pScanConcentrator->hEventSrv = hEventSRV;
    pScanConcentrator->hMlme = hMlme;
    pScanConcentrator->hCtrlData = hCtrlData;
    pScanConcentrator->hHealthMonitor = hHealthMonitor;

    /* copy registry values */
    os_memoryCopy( pScanConcentrator->hOS, &(pScanConcentrator->initParams), pScanConcentratorInitParams,
                   sizeof( scanConcentratorInitParams_t ) );

    /* initialize state machines */
    if ( OK != scanConcentratorAppSM_init( hScanCncn ) )
    {
        WLAN_REPORT_ERROR( hReport, SCAN_CNCN_MODULE_LOG, ("Error: application SM initialization failed.\n") );
        return;
    }

    if ( OK != scanConcentratorDrvSM_init( hScanCncn ) )
    {
        WLAN_REPORT_ERROR( hReport, SCAN_CNCN_MODULE_LOG, ("Error: driver SM initialization failed.\n") );
        return;
    }

    if ( OK != scanConcentratorRoamingContSM_init( hScanCncn ) )
    {
        WLAN_REPORT_ERROR( hReport, SCAN_CNCN_MODULE_LOG, ("Error: continuous scan for roaming SM initialization failed.\n") );
        return;
    }

    if ( OK != scanConcentratorRoamingImmedSM_init( hScanCncn ) )
    {
        WLAN_REPORT_ERROR( hReport, SCAN_CNCN_MODULE_LOG, ("Error: immediate scan for roaming SM initialization failed.\n") );
        return;
    }
	if ( OK != scanConcentratorOidSM_init( hScanCncn ) )
	{
        WLAN_REPORT_ERROR( hReport, SCAN_CNCN_MODULE_LOG, ("Error: OID scan SM initialization failed.\n") );
        return;
	}
    /* register SCR callbacks */
    scr_registerClientCB( pScanConcentrator->hSCR, SCR_CID_APP_SCAN, scanConcentrator_scrAppCB, hScanCncn );
    scr_registerClientCB( pScanConcentrator->hSCR, SCR_CID_DRIVER_FG_SCAN, scanConcentrator_scrDriverCB, hScanCncn );
    scr_registerClientCB( pScanConcentrator->hSCR, SCR_CID_CONT_SCAN, scanConcentrator_scrRoamingContCB, hScanCncn );
    scr_registerClientCB( pScanConcentrator->hSCR, SCR_CID_IMMED_SCAN, scanConcentrator_scrRoamingImmedCB, hScanCncn );

    /* register scan SRV scan complete CB */
    MacServices_scanSRV_registerScanCompleteCB( hMacServices, scanConcentrator_scanCompleteNotificationCB, hScanCncn );

    /* nullify other parameters */
    pScanConcentrator->currentRunningScanClient = SCAN_SCC_NO_CLIENT;
    pScanConcentrator->connectionStatus = STA_NOT_CONNECTED;

    /* bUseSGParams is TRUE only when SG module is enabled */
    pScanConcentrator->bUseSGParams = FALSE;
    
    /* "register" the application scan result callback */
    scanConcentrator_registerScanResultCB( hScanCncn, SCAN_SCC_APP, scanConcentrator_appScanResultCB, hScanCncn );
    
    WLAN_REPORT_INIT( hReport, SCAN_CNCN_MODULE_LOG,  (".....Scan concentrator configured successfully.\n"));
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief switch the connection mode to connected (infrastructure BSS)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 */
void scanConcentrator_switchToConnected( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG, ("Switching to connected state.\n") );
    
    /* change connection status to connected */
    pScanConcentrator->connectionStatus = STA_CONNECTED;

    /* Any running scans in other modes will be aborted (if needed) by the SCR (or have already been) */
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief switch the connection mode to not connected
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 */
void scanConcentrator_switchToNotConnected( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG, ("Switching to not connected state.\n") );

    /* change connection status to connected */
    pScanConcentrator->connectionStatus = STA_NOT_CONNECTED;

    /* Any running scans in other modes will be aborted (if needed) by the SCR (or have already been) */
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief switch the connection mode to IBSS participation
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 */
void scanConcentrator_switchToIBSS( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG, ("Switching to IBSS state.\n") );

    /* change connection status to connected */
    pScanConcentrator->connectionStatus = STA_IBSS;

    /* Any running scans in other modes will be aborted (if needed) by the SCR (or have already been) */
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Called by a client to request a scan.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param client - the client ID.\n
 * \return scan operation detailed result status.\n
 * \retval SCAN_CRS_SCAN_RUNNING - scan started successfully and is now running.\n
 * \retval SCAN_CRS_SCAN_FAILED - scan failed to start due to an unexpected error.\n
 */
scan_cncnResultStatus_e scanConcentrator_scan( TI_HANDLE hScanCncn, 
                                               scan_CncnClient_e client, 
                                               scan_Params_t* pScanParams )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;
    whalCtrl_setTemplate_t  templateStruct;
    probeReqTemplate_t      probeReqTemplate;
    paramInfo_t             param;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("Received scan request from client %d\n", client) );

    /* sanity check verify that this is a known client */
    if ( client >= SCAN_SCC_NUM_OF_CLIENTS)
    {
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG, 
                             ("Scan request with illegal client %d, aborting request.\n", client) );
        return SCAN_CRS_SCAN_FAILED;
    }

    /* scan requests with junk SSID (other than application scans) are rejected */
    if ( (client == SCAN_SCC_DRIVER) &&
         utils_isJunkSSID( &(pScanParams->desiredSsid) ) )
    {
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("Scan request with junk SSID from client %d.\n", client) );
        return SCAN_CRS_SCAN_FAILED;
    }

#ifndef TI_DBG
    /* in release mode, no channel expansion is done */
    if ( 0 == pScanParams->numOfChannels )
    {
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("Scan request with 0 channels from client:%d\n", client) );
        return SCAN_CRS_SCAN_FAILED;
    }
#endif

	/* Validate dwell times */
	if ( pScanParams->scanType != SCAN_TYPE_SPS )
	{
		int channel;

		for (channel = 0; channel < pScanParams->numOfChannels; ++channel)
		{
			if (pScanParams->channelEntry[channel].normalChannelEntry.maxChannelDwellTime < 
				pScanParams->channelEntry[channel].normalChannelEntry.minChannelDwellTime)
			{
				WLAN_REPORT_ERROR( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
					("%s: MaxChannelDwellTime must not be smaller than MinChannelDwellTime on channel %d!\n", __FUNCTION__, channel));

				return SCAN_CRS_SCAN_FAILED;
			}
		}
	}

    /* Check if country code should be reset in RegDomain. This is done here in order to detect if country should be updated 
       before the next scan. RegDomain will decide if country code should be deleted or remain the same. */
    param.paramType = REGULATORY_DOMAIN_CHECK_COUNTRY_PARAM;
    regulatoryDomain_setParam(pScanConcentrator->hRegulatoryDomain,&param);

	WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,("SG=%d, scan band = %d \n",
																			   pScanConcentrator->bUseSGParams,
																			   pScanParams->band));
    /* act according to client */
    switch ( client )
    {
    case SCAN_SCC_APP:
        /* check that the application state machine is in IDLE state 
           (no more than one scan at a time per client is allowed). */
        if ( APP_SCAN_STATE_IDLE != pScanConcentrator->clientSMState[ SCAN_SCC_APP ] )
        {
            return SCAN_CRS_SCAN_FAILED;
        }
        else
        {
            /* copy scan parameters to local buffer */
            os_memoryCopy( pScanConcentrator->hOS, &(pScanConcentrator->clientScanParams[ SCAN_SCC_APP ]), 
                           pScanParams, sizeof(scan_Params_t) );

            /* ask the reg domain which channels are allowed for the requested scan type */
            scanConcentrator_verifyChannelsWithRegDomain( hScanCncn, &(pScanConcentrator->clientScanParams[ SCAN_SCC_APP ]) );
 
            /* if no channels are available for scan, return negative result */
            if ( 0 == pScanConcentrator->clientScanParams[ SCAN_SCC_APP ].numOfChannels )
            {
                return SCAN_CRS_SCAN_FAILED;
            }
            
            if ((pScanConcentrator->bUseSGParams) && (pScanConcentrator->clientScanParams[ SCAN_SCC_APP ].band == RADIO_BAND_2_4_GHZ))
            {
				WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,("SG is pn && RADIO_BAND_2_4_GHZ \n"));
                scanConcentrator_SGupdateScanParams(hScanCncn,&(pScanConcentrator->clientScanParams[ SCAN_SCC_APP ]));
            }
            
            /* mark that a scan request is in progress (to avoid client re-entrance if the scan fail) */
            pScanConcentrator->bInRequest = TRUE;

            /* mark the scan result as OK (until other status will replace it) */
            pScanConcentrator->scanResult[ SCAN_SCC_APP ] = SCAN_CRS_SCAN_COMPLETE_OK;

            /* send probe request template */
            if ( (SCAN_TYPE_NORMAL_ACTIVE == pScanParams->scanType) || 
                 (SCAN_TYPE_TRIGGERED_ACTIVE == pScanParams->scanType) )
            {

                templateStruct.pTemplate = (UINT8 *)&probeReqTemplate;
                templateStruct.templateType = PROBE_REQUEST_TEMPLATE;
                buildProbeReqTemplate( pScanConcentrator->hSiteManager, &templateStruct,  
					&(pScanConcentrator->clientScanParams[ SCAN_SCC_APP ].desiredSsid),
					  pScanConcentrator->clientScanParams[ SCAN_SCC_APP ].band);
                whalCtrl_SetTemplate( pScanConcentrator->hHalCtrl, &templateStruct);
            }           

            /* send a start scan event to the SM */
            scanConcentratorAppSM_SMEvent( hScanCncn, 
                                           (scan_appSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_APP ]),
                                           APP_SCAN_EVENT_START_SCAN );

            /* mark that the scan request is no longer in progress */
            pScanConcentrator->bInRequest = FALSE;

            /* return scan result */
            if ( SCAN_CRS_SCAN_COMPLETE_OK == pScanConcentrator->scanResult[ SCAN_SCC_APP ] )
            {
                return SCAN_CRS_SCAN_RUNNING;
            }
            else 
            {
                return pScanConcentrator->scanResult[ SCAN_SCC_APP ];
            }
        }
/*        break; - unreachable */

    case SCAN_SCC_DRIVER:
        /* check that the driver state machine is in IDLE state 
           (no more than one scan at a time per client is allowed)*/
        if ( DRV_SCAN_STATE_IDLE != pScanConcentrator->clientSMState[ SCAN_SCC_DRIVER ] )
        {
            return SCAN_CRS_SCAN_FAILED;
        }
        else
        {
            /* copy scan parameters to local buffer */
            os_memoryCopy( pScanConcentrator->hOS, &(pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ]), 
                           pScanParams, sizeof(scan_Params_t) );

            if ((pScanConcentrator->bUseSGParams) && (pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].band == RADIO_BAND_2_4_GHZ))
            {
				WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,("SG is pn && RADIO_BAND_2_4_GHZ \n"));
                scanConcentrator_SGupdateScanParams(hScanCncn,&(pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ]));
            }

            /* mark that a scan request is in progress (to avoid client re-entrance if the scan fail) */
            pScanConcentrator->bInRequest = TRUE;

            /* mark the scan result as OK (until other status will replace it) */
            pScanConcentrator->scanResult[ SCAN_SCC_DRIVER ] = SCAN_CRS_SCAN_COMPLETE_OK;

            /* send probe request template */
            if ( (SCAN_TYPE_NORMAL_ACTIVE == pScanParams->scanType) || 
                 (SCAN_TYPE_TRIGGERED_ACTIVE == pScanParams->scanType) )
            {

                templateStruct.pTemplate = (UINT8 *)&probeReqTemplate;
                templateStruct.templateType = PROBE_REQUEST_TEMPLATE;
                buildProbeReqTemplate( pScanConcentrator->hSiteManager, &templateStruct,  
					&(pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].desiredSsid),
					  pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].band);
                whalCtrl_SetTemplate( pScanConcentrator->hHalCtrl, &templateStruct);
            }
            /* send a start scan event to the SM */
            scanConcentratorDrvSM_SMEvent( hScanCncn, 
                                           (scan_drvSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_DRIVER ]),
                                           DRV_SCAN_EVENT_START_SCAN );

            /* mark that the scan request is no longer in progress */
            pScanConcentrator->bInRequest = FALSE;

            /* return scan result */
            if ( SCAN_CRS_SCAN_COMPLETE_OK == pScanConcentrator->scanResult[ SCAN_SCC_DRIVER ] )
            {
                return SCAN_CRS_SCAN_RUNNING;
            }
            else
            {
                return pScanConcentrator->scanResult[ SCAN_SCC_DRIVER ];
            }
        }
/*        break; - unreachable */

    case SCAN_SCC_ROAMING_CONT:
        /* check that the continuous roaming state machine is in IDLE state 
           (no more than one scan at a time per client is allowed)*/
        if ( CONT_SCAN_STATE_IDLE != pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_CONT ] )
        {
            return SCAN_CRS_SCAN_FAILED;
        }
        else
        {
            /* copy scan parameters to local buffer */
            os_memoryCopy( pScanConcentrator->hOS, &(pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_CONT ]), 
                           pScanParams, sizeof(scan_Params_t) );
            
            if ((pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_CONT ].desiredSsid.len!=0) &&
                ((pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_CONT ].scanType == SCAN_TYPE_NORMAL_ACTIVE) ||
                 (pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_CONT ].scanType == SCAN_TYPE_TRIGGERED_ACTIVE)))
            {
                /* set the SSID of the current AP */
                param.paramType = SITE_MGR_DESIRED_SSID_PARAM;
                siteMgr_getParam( pScanConcentrator->hSiteManager, &param );
                os_memoryCopy( pScanConcentrator->hOS, 
                               &(pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_CONT ].desiredSsid),
                               &(param.content.siteMgrDesiredSSID),
                               sizeof(ssid_t) );

            }
            WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                                     ("CONT Scan: The scan SSID is %s\n", pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_CONT ].desiredSsid.ssidString) );

            /* ask the reg domain which channels are allowed for the requested scan type */
            scanConcentrator_verifyChannelsWithRegDomain( hScanCncn, &(pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_CONT ]) );
 
            /* if no channels are available for scan, return negative result */
            if ( 0 == pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_CONT ].numOfChannels )
            {
                return SCAN_CRS_SCAN_FAILED;
            }

            if ((pScanConcentrator->bUseSGParams) && (pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_CONT ].band == RADIO_BAND_2_4_GHZ))
            {
				WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,("SG is pn && RADIO_BAND_2_4_GHZ \n"));
                scanConcentrator_SGupdateScanParams(hScanCncn,&(pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_CONT ]));
            }

            /* mark that a scan request is in progress (to avoid client re-entrance if the scan fail) */
            pScanConcentrator->bInRequest = TRUE;

            /* mark the scan result as OK (until other status will replace it) */
            pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_CONT ] = SCAN_CRS_SCAN_COMPLETE_OK;

            /* send probe request template */
            if ( (SCAN_TYPE_NORMAL_ACTIVE == pScanParams->scanType) || 
                 (SCAN_TYPE_TRIGGERED_ACTIVE == pScanParams->scanType) )
            {

                templateStruct.pTemplate = (UINT8 *)&probeReqTemplate;
                templateStruct.templateType = PROBE_REQUEST_TEMPLATE;
                buildProbeReqTemplate( pScanConcentrator->hSiteManager, &templateStruct,  
					&(pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_CONT ].desiredSsid),
					  pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_CONT ].band);
                whalCtrl_SetTemplate( pScanConcentrator->hHalCtrl, &templateStruct);
            }

            /* send a start scan event to the SM */
            scanConcentratorRoamingContSM_SMEvent( hScanCncn, 
                                                   (scan_contSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_CONT ]),
                                                   CONT_SCAN_EVENT_START_SCAN );

            /* mark that the scan request is no longer in progress */
            pScanConcentrator->bInRequest = FALSE;

            /* return scan result */
            if ( SCAN_CRS_SCAN_COMPLETE_OK == pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_CONT ] )
            {
                return SCAN_CRS_SCAN_RUNNING;
            }
            else
            {
                return pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_CONT ];
            }
        }
/*        break; - unreachable */

    case SCAN_SCC_ROAMING_IMMED:
        /* check that the immediate roaming state machine is in IDLE state 
           (no more than one scan at a time per client is allowed)*/
        if ( IMMED_SCAN_STATE_IDLE != pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_IMMED ] )
        {
            return SCAN_CRS_SCAN_FAILED;
        }
        else
        {
            
            /* copy scan parameters to local buffer */
            os_memoryCopy( pScanConcentrator->hOS, &(pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_IMMED ]), 
                     pScanParams, sizeof(scan_Params_t) );

            /* set the SSID of the current AP */
            param.paramType = SITE_MGR_DESIRED_SSID_PARAM;
            siteMgr_getParam( pScanConcentrator->hSiteManager, &param );
            os_memoryCopy( pScanConcentrator->hOS, 
                   &(pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_IMMED ].desiredSsid),
                   &(param.content.siteMgrDesiredSSID),
                   sizeof(ssid_t) );
            
            /* ask the reg domain which channels are allowed for the requested scan type */
            scanConcentrator_verifyChannelsWithRegDomain( hScanCncn, &(pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_IMMED ]) );
 
            /* if no channels are available for scan, return negative result */
            if ( 0 == pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_IMMED ].numOfChannels )
            {
                return SCAN_CRS_SCAN_FAILED;
            }

            if ((pScanConcentrator->bUseSGParams) && (pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_IMMED ].band == RADIO_BAND_2_4_GHZ))
            {
				WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,("SG is pn && RADIO_BAND_2_4_GHZ \n"));
                scanConcentrator_SGupdateScanParams(hScanCncn,&(pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_IMMED ]));
            }
            
            /* mark that a scan request is in progress (to avoid client re-entrance if the scan fail) */
            pScanConcentrator->bInRequest = TRUE;

            /* mark the scan result as OK (until other status will replace it) */
            pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_IMMED ] = SCAN_CRS_SCAN_COMPLETE_OK;

            /* send probe request template */
            if ( (SCAN_TYPE_NORMAL_ACTIVE == pScanParams->scanType) || 
                 (SCAN_TYPE_TRIGGERED_ACTIVE == pScanParams->scanType) )
            {

                templateStruct.pTemplate = (UINT8 *)&probeReqTemplate;
                templateStruct.templateType = PROBE_REQUEST_TEMPLATE;
                buildProbeReqTemplate( pScanConcentrator->hSiteManager, &templateStruct,  
					&(pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_IMMED ].desiredSsid),
					  pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_IMMED ].band);
                whalCtrl_SetTemplate( pScanConcentrator->hHalCtrl, &templateStruct);
            }
            
            /* send a start scan event to the SM */
            scanConcentratorRoamingImmedSM_SMEvent( hScanCncn, 
                                                    (scan_immedSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_IMMED ]),
                                                    IMMED_SCAN_EVENT_START_SCAN );

            /* mark that the scan request is no longer in progress */
            pScanConcentrator->bInRequest = FALSE;

            /* return scan result */
            if ( SCAN_CRS_SCAN_COMPLETE_OK == pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_IMMED ] )
            {
                return SCAN_CRS_SCAN_RUNNING;
            }
            else
            {
                return pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_IMMED ];
            }
        }
/*        break; - unreachable */

    default:
        WLAN_REPORT_ERROR( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("Scan request from unknown client:%d\n", client) );
        return SCAN_CRS_SCAN_FAILED;
/*        break; - unreachable */
        
    }

/*    return SCAN_CRS_SCAN_RUNNING; - unreachable */
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Called by a client to stop a scan request in process. A client should ONLY stop its own request!
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param client - the client ID.\n
 */
void scanConcentrator_stopScan( TI_HANDLE hScanCncn, scan_CncnClient_e client )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("Received stop scan request from client %d\n", client) );

    /* act according to client */
    switch ( client )
    {
    case SCAN_SCC_APP:
        /* if no previous error has occurred, change the state to stopped */
        if ( SCAN_CRS_SCAN_COMPLETE_OK == pScanConcentrator->scanResult[ SCAN_SCC_APP ] )
        {
            pScanConcentrator->scanResult[ SCAN_SCC_APP ] = SCAN_CRS_SCAN_STOPPED;
            /* set the abort or stop flag (to identify when to exit driver mode (stop) and
            when not (abort due to higher priority client) */
            pScanConcentrator->bAbortOrStop = SCAN_CNCN_STOP;
        }
        else if (SCAN_CRS_SCAN_ABORTED_HIGHER_PRIORITY != pScanConcentrator->scanResult[ SCAN_SCC_APP ])
        {   /* In all scan failures, besides abort, indicate the flag as STOP */
            /* set the abort or stop flag (to identify when to exit driver mode (stop) and
            when not (abort due to higher priority client) */
            pScanConcentrator->bAbortOrStop = SCAN_CNCN_STOP;
        }

        /* send a stop scan event to the SM */
        scanConcentratorAppSM_SMEvent( hScanCncn, 
                                       (scan_appSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_APP ]),
                                       APP_SCAN_EVENT_STOP_SCAN );
        break;

    case SCAN_SCC_DRIVER:
        /* if no previous error has occurred, change the state to stopped */
        if ( SCAN_CRS_SCAN_COMPLETE_OK == pScanConcentrator->scanResult[ SCAN_SCC_DRIVER ] )
        {
            pScanConcentrator->scanResult[ SCAN_SCC_DRIVER ] = SCAN_CRS_SCAN_STOPPED;
        }

        /* send a stop scan event to the SM */
        scanConcentratorDrvSM_SMEvent( hScanCncn, 
                                       (scan_drvSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_DRIVER ]),
                                       DRV_SCAN_EVENT_STOP_SCAN );
        break;

    case SCAN_SCC_ROAMING_CONT:
        /* if no previous error has occurred, change the state to stopped */
        if ( SCAN_CRS_SCAN_COMPLETE_OK == pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_CONT ] )
        {
            pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_CONT ] = SCAN_CRS_SCAN_STOPPED;

            /* set the abort or stop flag (to identify when to exit driver mode (stop) and
            when not (abort due to higher priority client) */
            pScanConcentrator->bAbortOrStop = SCAN_CNCN_STOP;
        }
        else if (SCAN_CRS_SCAN_ABORTED_HIGHER_PRIORITY != pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_CONT ])
        {   /* In all scan failures, besides abort, indicate the flag as STOP */
            /* set the abort or stop flag (to identify when to exit driver mode (stop) and
            when not (abort due to higher priority client) */
            pScanConcentrator->bAbortOrStop = SCAN_CNCN_STOP;
        }

        /* send a stop scan event to the SM */
        scanConcentratorRoamingContSM_SMEvent( hScanCncn, 
                                               (scan_contSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_CONT ]),
                                               CONT_SCAN_EVENT_STOP_SCAN );
        break;

    case SCAN_SCC_ROAMING_IMMED:
        /* if no previous error has occurred, change the state to stopped */
        if ( SCAN_CRS_SCAN_COMPLETE_OK == pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_IMMED ] )
        {
            pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_IMMED ] = SCAN_CRS_SCAN_STOPPED;

            /* set the abort or stop flag (to identify when to exit driver mode (stop) and
            when not (abort due to higher priority client) */
            pScanConcentrator->bAbortOrStop = SCAN_CNCN_STOP;
        }
        else if (SCAN_CRS_SCAN_ABORTED_HIGHER_PRIORITY != pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_IMMED ])
        {   /* In all scan failures, besides abort, indicate the flag as STOP */
            /* set the abort or stop flag (to identify when to exit driver mode (stop) and
            when not (abort due to higher priority client) */
            pScanConcentrator->bAbortOrStop = SCAN_CNCN_STOP;
        }

        /* send a stop scan event to the SM */
        scanConcentratorRoamingImmedSM_SMEvent( hScanCncn, 
                                                (scan_immedSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_IMMED ]),
                                                IMMED_SCAN_EVENT_STOP_SCAN );
        break;

    default:
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG, 
                             ("Stop scan request with illegal client %d, aborting request.\n", client) );
        break;
    }
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Registers a scan result function for a specific client.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param client - the client ID.\n
 * \param scanResultCBFunc - the function to use.\n
 * \param scanresultCBObj - the object to pass to the scan result CB function.\n
 */
void scanConcentrator_registerScanResultCB( TI_HANDLE hScanCncn, scan_CncnClient_e client,
                                            scan_resultCB_t scanResultCBFunc, TI_HANDLE scanResultCBObj )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    /* save the function and object pointers */
    pScanConcentrator->scanResultCB[ client ] = scanResultCBFunc;
    pScanConcentrator->scanResultCBObj[ client ] = scanResultCBObj;
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Called by the scan SRV (after registration) to notify of a scan complete event.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param SPSStatus - which channels were attempted (if SPS scan).\n
 * \param bTSFError - whether a TSF error occurred (if SPS scan).\n
 * \param  ScanStatus - return the status of the scan . \n
 */
void scanConcentrator_scanCompleteNotificationCB( TI_HANDLE hScanCncn, UINT16 SPSStatus, BOOLEAN bTSFError , TI_STATUS ScanStatus , TI_STATUS PSMode )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

     /* start built-in test timer */
    healthMonitor_resumePeriodicTest( pScanConcentrator->hHealthMonitor );

#ifdef TI_DBG
    /* check that current running client is valid */
    if ( SCAN_SCC_NO_CLIENT == pScanConcentrator->currentRunningScanClient ) 
    {
        WLAN_REPORT_ERROR( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                           ("Scan complete called when current running client is invalid: %d\n", 
                            pScanConcentrator->currentRunningScanClient) );
        return;
    }
#endif
    /* send a scan complete event to the running client (according to its type) */
    switch ( pScanConcentrator->currentRunningScanClient )
    {
    case SCAN_SCC_APP:
        scanConcentratorAppSM_SMEvent( hScanCncn, 
                                       (scan_appSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_APP ]),
                                       APP_SCAN_EVENT_SCAN_COMPLETE );
        break;

    case SCAN_SCC_DRIVER:
        scanConcentratorDrvSM_SMEvent( hScanCncn, 
                                       (scan_drvSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_DRIVER ]),
                                       scanConcentrator_getNextDriverEvent( hScanCncn ) );
        break;

    case SCAN_SCC_ROAMING_CONT:
        /* copy the SPS scan result (in case this was an SPS scan) */
        pScanConcentrator->SPSScanResult = SPSStatus;

        /* if A TSF error occurred (for non-SPS scans this value is always FALSE!), and no previous error occurred,
           mark this error */
        if ( (SCAN_CRS_SCAN_COMPLETE_OK == 
                pScanConcentrator->scanResult[ pScanConcentrator->currentRunningScanClient ]) &&
              (TRUE == bTSFError) )
        {
            pScanConcentrator->scanResult[ pScanConcentrator->currentRunningScanClient ] = SCAN_CRS_TSF_ERROR;
        }
        scanConcentratorRoamingContSM_SMEvent( hScanCncn, 
                                               (scan_contSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_CONT ]),
                                               CONT_SCAN_EVENT_SCAN_COMPLETE );
        break;

    case SCAN_SCC_ROAMING_IMMED:
        scanConcentratorRoamingImmedSM_SMEvent( hScanCncn, 
                                                (scan_immedSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_IMMED ]),
                                                IMMED_SCAN_EVENT_SCAN_COMPLETE );
        break;

    default:
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG, 
                             ("Scan complete notification with illegal client %d, aborting request.\n", pScanConcentrator->currentRunningScanClient) );
        break;
    }
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Called by the MLME parser to pass information received on a beacon or probe response.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param bssid - a pointer to the address of the AP sending this frame.\n
 * \param frameInfo - the IE in the frame.\n
 * \param pRxAttr - a pointer to TNET RX attributes struct.\n
 * \param buffer - a pointer to the frame body.\n
 * \param byfferLength - the frame body length.\n
 */
void scanConcentrator_mlmeResultCB( TI_HANDLE hScanCncn, macAddress_t* bssid, mlmeFrameInfo_t* frameInfo, 
                                    Rx_attr_t* pRxAttr, UINT8* buffer, UINT16 bufferLength )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;
    paramInfo_t param;
    scan_frameInfo_t scanFrameInfo;

    /* verify that there is a running client (in case the result arrived after the scan complete event) */
    switch (pScanConcentrator->currentRunningScanClient)
    {
    case SCAN_SCC_APP:
        if ( APP_SCAN_STATE_IDLE == pScanConcentrator->clientSMState[ SCAN_SCC_APP ] )
        {
            return;
        }
        break;

    case SCAN_SCC_DRIVER:
        if ( DRV_SCAN_STATE_IDLE == pScanConcentrator->clientSMState[ SCAN_SCC_DRIVER ] )
        {
            return;
        }
        break;

    case SCAN_SCC_ROAMING_CONT:
        if ( CONT_SCAN_STATE_IDLE == pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_CONT ] )
        {
            return;
        }
        /* check that the SSID IE exists in the frame received */
        if (frameInfo->content.iePacket.pSsid==NULL) 
        {
            /* Discard sites with no SID IE */
            return;
        }
        /* check that scan results are not from current BSS */
        param.paramType = CTRL_DATA_CURRENT_BSSID_PARAM;
        ctrlData_getParam( pScanConcentrator->hCtrlData, &param );
        if (( 0 == os_memoryCompare( pScanConcentrator->hOS, 
                                    (void *)&(bssid->addr[ 0 ]), 
                                    (void *)&(param.content.ctrlDataCurrentBSSID.addr[ 0 ]), 
                                    MAC_ADDR_LEN ) ) ||
            (( os_memoryCompare( pScanConcentrator->hOS, 
                                (PUINT8)frameInfo->content.iePacket.pSsid->serviceSetId, 
                                (PUINT8)pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_CONT ].desiredSsid.ssidString, 
                                    pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_CONT ].desiredSsid.len )) &&
             pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_CONT ].scanType != SCAN_TYPE_SPS) )
        {
            /* reply from current site - discard it (by returning w/o notifying the scan manager */
            /* Also discard sites with different SSID than the desired */
            WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                                     ("Cont scan: discarding frame from SSID: %s, BSSID: %2x:%2x:%2x:%2x:%2x:%2x, because SSID different from desired or from current AP!\n",
                                      frameInfo->content.iePacket.pSsid->serviceSetId, bssid->addr[ 0 ],
                                      bssid->addr[ 1 ], bssid->addr[ 2 ], bssid->addr[ 3 ],
                                      bssid->addr[ 4 ], bssid->addr[ 5 ]) );
            return;
        }

        break;

    case SCAN_SCC_ROAMING_IMMED:
        if ( IMMED_SCAN_STATE_IDLE == pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_IMMED ] )
        {
            return;
        }
        /* check that the SSID IE exists in the frame received */
        if (frameInfo->content.iePacket.pSsid==NULL) 
        {
            /* Discard sites with no SID IE */
            return;
        }
        /* check that scan results are not from current BSS */
        param.paramType = CTRL_DATA_CURRENT_BSSID_PARAM;
        ctrlData_getParam( pScanConcentrator->hCtrlData, &param );
        if (( 0 == os_memoryCompare( pScanConcentrator->hOS, 
                                    (PUINT8)&(bssid->addr[ 0 ]), 
                                    (PUINT8)&(param.content.ctrlDataCurrentBSSID.addr[ 0 ]), 
                                    MAC_ADDR_LEN ) ) ||
            ( os_memoryCompare( pScanConcentrator->hOS, 
                                (PUINT8)frameInfo->content.iePacket.pSsid->serviceSetId, 
                                (PUINT8)pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_IMMED ].desiredSsid.ssidString, 
                                    pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_IMMED ].desiredSsid.len )))

        {
            /* reply from current site - discard it (by returning w/o notifying the scan manager */
            /* Also discard sites with different SSID than the desired */
            WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                                     ("Immed scan: discarding frame from SSID: %s, BSSID: %2x:%2x:%2x:%2x:%2x:%2x, because SSID different from desired or from current AP!\n",
                                      frameInfo->content.iePacket.pSsid->serviceSetId, bssid->addr[ 0 ],
                                      bssid->addr[ 1 ], bssid->addr[ 2 ], bssid->addr[ 3 ],
                                      bssid->addr[ 4 ], bssid->addr[ 5 ]) );
            return;
        }
        break;

    default:
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG, 
                             ("Scan result with illegal client %d, aborting request.\n", pScanConcentrator->currentRunningScanClient) );
        return;
/*        break; - unreachable */
    }

    /* build the scan frame info object */
    scanFrameInfo.bssId = bssid;
    scanFrameInfo.band = (radioBand_e)pRxAttr->band;
    scanFrameInfo.channel = pRxAttr->channel;
    scanFrameInfo.parsedIEs = frameInfo;
    scanFrameInfo.rate = pRxAttr->Rate;
    scanFrameInfo.rssi = pRxAttr->Rssi;
    scanFrameInfo.staTSF = pRxAttr->TimeStamp;
    scanFrameInfo.buffer = buffer;
    scanFrameInfo.bufferLength = bufferLength;

    /* call the client result CB, according to the running client type */
    if ( NULL != pScanConcentrator->scanResultCB[ pScanConcentrator->currentRunningScanClient ] )
    {
        pScanConcentrator->scanResultCB[ pScanConcentrator->currentRunningScanClient ]( 
            pScanConcentrator->scanResultCBObj[ pScanConcentrator->currentRunningScanClient ],
            SCAN_CRS_RECEIVED_FRAME,
            &scanFrameInfo,
            0xffff ); /* SPS status is only valid on SPS scan complete */
    }
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Called by the SCR (after registration) to notify the scan concentrator of a status change
 * \brief for the immediate scan for roaming client.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param requestStatus - the continuous scan for roaming client status.\n
 */
void scanConcentrator_scrRoamingImmedCB( TI_HANDLE hScanCncn, scr_clientRequestStatus_e requestStatus,
                                         scr_pendReason_e pendReason )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("Immed. romaing CB called by SCR. Status is: %d, pend reason: %d.\n", 
                              requestStatus, pendReason) );

    /* act according to the request staus */
    switch ( requestStatus )
    {
    case SCR_CRS_RUN:
        /* send an SCR run event to the SM */
        scanConcentratorRoamingImmedSM_SMEvent( hScanCncn, 
                                                (scan_immedSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_IMMED ]),
                                                IMMED_SCAN_EVENT_SCR_RUN );
        break;

    case SCR_CRS_PEND:
        /* if pending reason has changed to different group - send a reject event 
           (should only happen when pending) */
        if ( SCR_PR_DIFFERENT_GROUP_RUNNING == pendReason )
        {
            /* send an SCR reject event to the SM - would not scan when not performing roaming */
            pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_IMMED ] = SCAN_CRS_SCAN_FAILED;
            scanConcentratorRoamingImmedSM_SMEvent( hScanCncn, 
                                                    (scan_immedSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_IMMED ]),
                                                    IMMED_SCAN_EVENT_SCR_REJECT );            
        }
        else
        {
            /* send an SCR pend event to the SM */
            scanConcentratorRoamingImmedSM_SMEvent( hScanCncn, 
                                                    (scan_immedSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_IMMED ]),
                                                    IMMED_SCAN_EVENT_SCR_PEND );
        }
        break;

    case SCR_CRS_FW_RESET:
        /* if no previous error has occurred, change the state to FW reset */
        if ( SCAN_CRS_SCAN_COMPLETE_OK == pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_IMMED ] )
        {
            pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_IMMED ] = SCAN_CRS_SCAN_ABORTED_FW_RESET;
        }

         /* start built-in test timer */
        healthMonitor_resumePeriodicTest( pScanConcentrator->hHealthMonitor );

        /* send a FW reset event to the SM */
        scanConcentratorRoamingImmedSM_SMEvent( hScanCncn, 
                                                (scan_immedSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_IMMED ]),
                                                IMMED_SCAN_EVENT_FW_RESET );
        break;

    case SCR_CRS_ABORT:
        /* This should never happen, report error */
    default:
        WLAN_REPORT_ERROR( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("Illegal SCR request status: %d.\n", requestStatus) );
        break;
    }
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Called by the SCR (after registration) to notify the scan concentrator of a status change
 * \brief for the continuous scan for roaming client.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param requestStatus - the continuous scan for roaming client status.\n
 */
void scanConcentrator_scrRoamingContCB( TI_HANDLE hScanCncn, scr_clientRequestStatus_e requestStatus,
                                        scr_pendReason_e pendReason )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("Cont. romaing CB called by SCR. Status is: %d, pend reason: %d.\n", 
                              requestStatus, pendReason) );

    /* act according to the request staus */
    switch ( requestStatus )
    {
    case SCR_CRS_RUN:
        /* send an SCR run event to the SM */
        scanConcentratorRoamingContSM_SMEvent( hScanCncn, 
                                               (scan_contSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_CONT ]),
                                               CONT_SCAN_EVENT_SCR_RUN );
        break;

    case SCR_CRS_PEND:
        /* if pending reason has changed to different group - send a reject event 
           (should only happen when pending) */
        if ( SCR_PR_DIFFERENT_GROUP_RUNNING == pendReason )
        {
            pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_CONT ] = SCAN_CRS_SCAN_FAILED;
            /* send an SCR reject event to the SM - would not scan when not connected */
        scanConcentratorRoamingContSM_SMEvent( hScanCncn, 
                                               (scan_contSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_CONT ]),
                                               CONT_SCAN_EVENT_SCR_REJECT );
        }
        else
        {
            /* send an SCR pend event to the SM */
            scanConcentratorRoamingContSM_SMEvent( hScanCncn, 
                                                   (scan_contSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_CONT ]),
                                                   CONT_SCAN_EVENT_SCR_PEND );
        }
        break;

    case SCR_CRS_FW_RESET:
        /* if no previous error has occurred, change the state to FW reset */
        if ( SCAN_CRS_SCAN_COMPLETE_OK == pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_CONT ] )
        {
            pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_CONT ] = SCAN_CRS_SCAN_ABORTED_FW_RESET;
        }

         /* start built-in test timer */
        healthMonitor_resumePeriodicTest( pScanConcentrator->hHealthMonitor );

        /* send a FW reset event to the SM */
        scanConcentratorRoamingContSM_SMEvent( hScanCncn, 
                                               (scan_contSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_CONT ]),
                                               CONT_SCAN_EVENT_FW_RESET );
        break;

    case SCR_CRS_ABORT:
        /* set the abort or stop flag (to identify when to exit driver mode (stop) and
           when not (abort due to higher priority client) */

        pScanConcentrator->bAbortOrStop = SCAN_CNCN_ABORT;

        /* if no previous error has occurred, change the state to abort (according to reason) */
        if ( SCAN_CRS_SCAN_COMPLETE_OK == pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_CONT ] )
        {
            pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_CONT ] = SCAN_CRS_SCAN_ABORTED_HIGHER_PRIORITY;
        }

        /* send an abort scan event to the SM */
        scanConcentratorRoamingContSM_SMEvent( hScanCncn, 
                                               (scan_contSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_CONT ]),
                                               CONT_SCAN_EVENT_ABORT_SCAN );
        break;

    default:
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("Illegal SCR request status: %d.\n", requestStatus) );
        break;
    }
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Called by the SCR (after registration) to notify the scan concentrator of a status change
 * \brief for the Application scan client.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param requestStatus - the application scan status.\n
 * \param pendReason - the reason for pend status, if the status is pend.\n
 */
void scanConcentrator_scrAppCB( TI_HANDLE hScanCncn, scr_clientRequestStatus_e requestStatus,
                                scr_pendReason_e pendReason )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("App. CB called by SCR. Status is: %d, pend reason: %d.\n", 
                              requestStatus, pendReason) );
    
    /* act according to the request staus */
    switch ( requestStatus )
    {
    /* Note: pend is not handled because application scan cancel its scan request when it receives pend
       as the SCR request result, and thus it is assumed that the application scan request will never be
       pending */

    case SCR_CRS_RUN:
        /* send an SCR run event to the SM */
        scanConcentratorAppSM_SMEvent( hScanCncn, 
                                       (scan_appSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_APP ]),
                                       APP_SCAN_EVENT_SCR_RUN );
        break;

    case SCR_CRS_FW_RESET:
        /* if no previous error has occurred, change the state to FW reset */
        if ( SCAN_CRS_SCAN_COMPLETE_OK == pScanConcentrator->scanResult[ SCAN_SCC_APP ] )
        {
            pScanConcentrator->scanResult[ SCAN_SCC_APP ] = SCAN_CRS_SCAN_ABORTED_FW_RESET;
        }

         /* start built-in test timer */
        healthMonitor_resumePeriodicTest( pScanConcentrator->hHealthMonitor );

        /* send a FW reset event to the SM */
        scanConcentratorAppSM_SMEvent( hScanCncn, 
                                       (scan_appSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_APP ]),
                                       APP_SCAN_EVENT_FW_RESET );
        break;

    case SCR_CRS_ABORT:  
        /* set the abort or stop flag (to identify when to exit driver mode (stop) and
           when not (abort due to higher priority client) */

        pScanConcentrator->bAbortOrStop = SCAN_CNCN_ABORT;

        /* if no previous error has occurred, change the state to abort (according to reason) */
        if ( SCAN_CRS_SCAN_COMPLETE_OK == pScanConcentrator->scanResult[ SCAN_SCC_APP ] )
        {
            pScanConcentrator->scanResult[ SCAN_SCC_APP ] = SCAN_CRS_SCAN_ABORTED_HIGHER_PRIORITY;
        }

        /* send an abort scan event to the SM */
        scanConcentratorAppSM_SMEvent( hScanCncn, 
                                       (scan_appSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_APP ]),
                                       APP_SCAN_EVENT_ABORT_SCAN );
        break;

    default:
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("Illegal SCR request status: %d.\n", requestStatus) );
        break;
    }
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Called by the SCR (after registration) to notify the scan concentrator of a status change
 * \brief for the driver scan client.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param requestStatus - the driver scan status.\n
 * \param pendReason - the reason for pend status, if the status is pend.\n
 */
void scanConcentrator_scrDriverCB( TI_HANDLE hScanCncn, scr_clientRequestStatus_e requestStatus,
                                   scr_pendReason_e pendReason )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("Driver CB called by SCR. Status is: %d, pend reason: %d.\n",
                              requestStatus, pendReason) );

    /* act according to the request staus */
    switch ( requestStatus )
    {
    case SCR_CRS_RUN:
        /* send the next event to the SM */
        scanConcentratorDrvSM_SMEvent( hScanCncn, 
                                       (scan_drvSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_DRIVER ]),
                                       scanConcentrator_getNextDriverEvent( hScanCncn ) );
        break;

    case SCR_CRS_PEND:
        /* a pend event should only be sent when the SM is waiting for the SCR */
        if ( pendReason == SCR_PR_DIFFERENT_GROUP_RUNNING )
        {
            /* send a reject event - should not perform scan if not in connecting mode */
            pScanConcentrator->scanResult[ SCAN_SCC_DRIVER ] = SCAN_CRS_SCAN_FAILED;
        scanConcentratorDrvSM_SMEvent( hScanCncn, 
                                       (scan_drvSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_DRIVER ]),
                                       DRV_SCAN_EVENT_SCR_REJECT );
        }
        else
        {
            /* send a pend event */
            scanConcentratorDrvSM_SMEvent( hScanCncn, 
                                           (scan_drvSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_DRIVER ]),
                                           DRV_SCAN_EVENT_SCR_PEND );            
        }
        break;

    case SCR_CRS_FW_RESET:
        /* if no previous error has occurred, change the state to FW reset */
        if ( SCAN_CRS_SCAN_COMPLETE_OK == pScanConcentrator->scanResult[ SCAN_SCC_DRIVER ] )
        {
            pScanConcentrator->scanResult[ SCAN_SCC_DRIVER ] = SCAN_CRS_SCAN_ABORTED_FW_RESET;
        }

        /* start built-in test timer */
        healthMonitor_resumePeriodicTest( pScanConcentrator->hHealthMonitor );

        /* send a FW reset event to the SM */
        scanConcentratorDrvSM_SMEvent( hScanCncn, 
                                       (scan_drvSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_DRIVER ]),
                                       DRV_SCAN_EVENT_FW_RESET );
        break;
    
    case SCR_CRS_ABORT:
    /* This should never happen, report error */
    default:
        WLAN_REPORT_ERROR( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("Illegal SCR request status: %d.\n", requestStatus) );
        break;
    }
}

/**
 * \author Ronen Kalish\n
 * \date 09-Jan-2005\n
 * \brief Verifies that specified channels are allowed for the specified scan type, and removes those that are not.\n
 *
 * Function Scope \e Private.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param pScanParams - a pointer to the structure holding the scan params (inc. channels and scan type).\n
 */
void scanConcentrator_verifyChannelsWithRegDomain( TI_HANDLE hScanCncn, scan_Params_t* pScanParams )
{
    paramInfo_t     param;
    UINT8           i,j,k;
    UINT8           tempChannelList[ SCAN_MAX_NUM_OF_NORMAL_CHANNELS_PER_COMMAND ];
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    /* if no channels were given by the user, get all channels from reg domain */
    if ( 0 == pScanParams->numOfChannels )
    {
        /* SPS cannot be performed when the user does not define the channels! */
        if ( SCAN_TYPE_SPS != pScanParams->scanType )
        {
            UINT validChannelsCnt=0;
            
            /* extract the channel list (channel numbers) from the bitmap */
            i = SCAN_MAX_NUM_OF_NORMAL_CHANNELS_PER_COMMAND;
            /*scanConcentrator_ExtractBitmapToList( pScanParams->band, &(tmpChannelBitmap[0]), &(tempChannelList[0]), &i );*/

            param.paramType = REGULATORY_DOMAIN_ALL_SUPPORTED_CHANNELS;
            param.content.siteMgrRadioBand = pScanParams->band;
            /*param.content.supportedChannels.listOfChannels = tempChannelList;*/
            regulatoryDomain_getParam( pScanConcentrator->hRegulatoryDomain, &param );
            i = param.content.supportedChannels.sizeOfList;
            if (i > SCAN_MAX_NUM_OF_NORMAL_CHANNELS_PER_COMMAND)
            {
                i = SCAN_MAX_NUM_OF_NORMAL_CHANNELS_PER_COMMAND;
            }
            os_memoryCopy(pScanConcentrator->hOS, tempChannelList, param.content.supportedChannels.listOfChannels, i);
            /* add default values to channels extracted from the bitmap */
            for ( j = 0; j < i; j++ )
            {

                param.paramType = REGULATORY_DOMAIN_GET_SCAN_CAPABILITIES;
                param.content.channelCapabilityReq.band = pScanParams->band;
                if ( (pScanParams->scanType == SCAN_TYPE_NORMAL_PASSIVE) ||
                     (pScanParams->scanType == SCAN_TYPE_TRIGGERED_PASSIVE) ||
                     (pScanParams->scanType == SCAN_TYPE_SPS) )
                {
                    param.content.channelCapabilityReq.scanOption = PASSIVE_SCANNING;
                }
                else
                {
                    param.content.channelCapabilityReq.scanOption = ACTIVE_SCANNING;
                }
                param.content.channelCapabilityReq.channelNum = tempChannelList[ j ];
                regulatoryDomain_getParam( pScanConcentrator->hRegulatoryDomain, &param );
                if (param.content.channelCapabilityRet.channelValidity == TRUE)
                {
                    /* add the channel ID */
                    pScanParams->channelEntry[ j ].normalChannelEntry.channel = tempChannelList[ j ];
                    /* add other default parameters */
                    pScanParams->channelEntry[ j ].normalChannelEntry.minChannelDwellTime = 
                        SCAN_DEFAULT_MIN_CHANNEL_DWELL_TIME;
                    pScanParams->channelEntry[ j ].normalChannelEntry.maxChannelDwellTime = 
                        SCAN_DEFAULT_MAX_CHANNEL_DWELL_TIME;
                    pScanParams->channelEntry[ j ].normalChannelEntry.earlyTerminationEvent =  
                        SCAN_DEFAULT_EARLY_TERMINATION_EVENT;
                    pScanParams->channelEntry[ j ].normalChannelEntry.ETMaxNumOfAPframes = 
                        SCAN_DEFAULT_EARLY_TERMINATION_NUM_OF_FRAMES;
                    pScanParams->channelEntry[ j ].normalChannelEntry.txPowerDbm = param.content.channelCapabilityRet.maxTxPowerDbm;

                    /* Fill broadcast BSSID */
                    for ( k = 0; k < 6; k++ )
                    {
                        pScanParams->channelEntry[ j ].normalChannelEntry.bssId.addr[ k ] = 0xff;
                    }

                    validChannelsCnt++;
                }
            }
            pScanParams->numOfChannels = validChannelsCnt;
        }
    }
    /* channels were supplied by user - must verify that all of them are allowed */
    else
    {
        /* check channels */
        for ( i = 0; i < pScanParams->numOfChannels; )
        { /* Note that i is only increased when channel is valid - if channel is invalid, another 
             channel is copied in its place, and thus the same index should be checked again. However,
             since the number of channels is decreased, the loop end condition is getting nearer! */
            
            param.paramType = REGULATORY_DOMAIN_GET_SCAN_CAPABILITIES;
            param.content.channelCapabilityReq.band = pScanParams->band;
            if ( (pScanParams->scanType == SCAN_TYPE_NORMAL_PASSIVE) ||
                 (pScanParams->scanType == SCAN_TYPE_TRIGGERED_PASSIVE) ||
                 (pScanParams->scanType == SCAN_TYPE_SPS) )
            {
                param.content.channelCapabilityReq.scanOption = PASSIVE_SCANNING;
            }
            else
            {
                param.content.channelCapabilityReq.scanOption = ACTIVE_SCANNING;
            }
            
            /* SPS scan */
            if ( SCAN_TYPE_SPS == pScanParams->scanType )
            {
                param.content.channelCapabilityReq.channelNum = pScanParams->channelEntry[ i ].SPSChannelEntry.channel;
                regulatoryDomain_getParam( pScanConcentrator->hRegulatoryDomain, &param );
                if (!param.content.channelCapabilityRet.channelValidity)
                {   /* channel not allowed - copy the rest of the channel in its place */
                    os_memoryCopy( pScanConcentrator->hOS, &(pScanParams->channelEntry[ i ]),
                                   &(pScanParams->channelEntry[ i + 1 ]), sizeof(scan_SPSChannelEntry_t) * 
                                                                          (pScanParams->numOfChannels - i - 1) );
                    pScanParams->numOfChannels--;
                }
                else
                {
                    i += 1;
                }
                
            }
            /* all other scan types */
            else
            {
                param.content.channelCapabilityReq.channelNum = pScanParams->channelEntry[ i ].normalChannelEntry.channel;
                regulatoryDomain_getParam( pScanConcentrator->hRegulatoryDomain, &param );
                if (!param.content.channelCapabilityRet.channelValidity)
                {   /* channel not allowed - copy the rest of the channel in its place */
                    os_memoryCopy( pScanConcentrator->hOS, &(pScanParams->channelEntry[ i ]),
                                   &(pScanParams->channelEntry[ i + 1 ]), sizeof(scan_normalChannelEntry_t) * 
                                                                          (pScanParams->numOfChannels - i - 1) );
                    pScanParams->numOfChannels--;
                }
                else
                {
                    pScanParams->channelEntry[i].normalChannelEntry.txPowerDbm = 
							MIN( param.content.channelCapabilityRet.maxTxPowerDbm, 
								pScanParams->channelEntry[i].normalChannelEntry.txPowerDbm ); 

                    i += 1;
                }
            }
        }
    }
}

/**
 * \author Yuval Adler\n
 * \date 26-Jan-2006\n
 * \brief enable/disable the use of SG parameters , and update parameters of next scans 
 *      requests for minDwellTime,MaxDwellTime,numProbeReq  .\n
 *          this function is called when SG is enabled or disabled from the SoftGemini module
 *          The compensation is needed since BT Activity holds the antenna and over-ride Scan activity
 *
 * Function Scope \e Private.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param bUseSGParams - whether to use the new parameters (TRUE when SG is enabled)
 * \param probeReqNumber - 
 * \param SGcompensationMaxTime - max value from which we won't increase dwelling time
 * \param SGcompensationPercent - increasing dwell time in that percentage
 */
void scanConcentrator_SGconfigureScanParams( TI_HANDLE hScanCncn, BOOL bUseSGParams ,
                                             UINT8 probeReqNumber , UINT32 SGcompensationMaxTime, 
                                             UINT32 SGcompensationPercent) 
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    pScanConcentrator->bUseSGParams            = bUseSGParams;
    pScanConcentrator->SGnumOfProbeRequest     = probeReqNumber;
    pScanConcentrator->SGcompensationMaxTime   = SGcompensationMaxTime;
    pScanConcentrator->SGcompensationPercent   = SGcompensationPercent;

    WLAN_REPORT_INFORMATION(pScanConcentrator->hReport,SCAN_CNCN_MODULE_LOG,
            ("%s: bUseSGParams = %d, numOfProbeRequest = %d, compensationMaxTime = %d, SGcompensationPercent = %d\n "
            ,__FUNCTION__,pScanConcentrator->bUseSGParams,pScanConcentrator->SGnumOfProbeRequest,
            pScanConcentrator->SGcompensationMaxTime,pScanConcentrator->SGcompensationPercent));
}

/**
 * \author Yuval Adler\n
 * \date 26-Jan-2006\n
 * \brief update minDwellTime,MaxDwellTime,numProbeReq according to SG module request .\n
 *          this function is called when SG is enabled.
 * Function Scope \e Private.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param pScanParams - a pointer to the structure holding the scan params .\n
 */
void scanConcentrator_SGupdateScanParams( TI_HANDLE hScanCncn, scan_Params_t* pScanParams ) 
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;
    UINT8 i;
    UINT32 tempTime;

    /* for each channel increase the min and max dwell time */
    for ( i = 0 ; i < pScanParams->numOfChannels ; i++)
    {       
        if (pScanConcentrator->SGcompensationMaxTime >
            pScanParams->channelEntry[i].normalChannelEntry.minChannelDwellTime)
        {
            tempTime = ((pScanParams->channelEntry[i].normalChannelEntry.minChannelDwellTime) * 
                ( 100 + pScanConcentrator->SGcompensationPercent)) / 100 ;

            if (tempTime > pScanConcentrator->SGcompensationMaxTime)
            {
                tempTime = pScanConcentrator->SGcompensationMaxTime;
            }
            pScanParams->channelEntry[i].normalChannelEntry.minChannelDwellTime = tempTime;
        }

        if (pScanConcentrator->SGcompensationMaxTime > 
                pScanParams->channelEntry[i].normalChannelEntry.maxChannelDwellTime)
        {
            tempTime = ((pScanParams->channelEntry[i].normalChannelEntry.maxChannelDwellTime) * 
                ( 100 + pScanConcentrator->SGcompensationPercent)) / 100 ;
                    
            if (tempTime > pScanConcentrator->SGcompensationMaxTime)
            {
                tempTime = pScanConcentrator->SGcompensationMaxTime;
            }
            pScanParams->channelEntry[i].normalChannelEntry.maxChannelDwellTime = tempTime;
        }
        WLAN_REPORT_INFORMATION(pScanConcentrator->hReport,SCAN_CNCN_MODULE_LOG,
                ("%s new parmas : channel = %d  MaxDwellTime = %d MinDwellTime = %d\n"
                ,__FUNCTION__,i,pScanParams->channelEntry[i].normalChannelEntry.maxChannelDwellTime,
                pScanParams->channelEntry[i].normalChannelEntry.minChannelDwellTime));
    }

    /* update ProbeReqNumber only if it is larger than 0 and smaller than the new value */
    if ((pScanParams->probeReqNumber > 0) && 
        (pScanConcentrator->SGnumOfProbeRequest > pScanParams->probeReqNumber))
    {
        pScanParams->probeReqNumber = pScanConcentrator->SGnumOfProbeRequest;
    }

    WLAN_REPORT_INFORMATION(pScanConcentrator->hReport,SCAN_CNCN_MODULE_LOG,
        ("%s number of Probe requests = %d\n",__FUNCTION__,pScanParams->probeReqNumber));
}


