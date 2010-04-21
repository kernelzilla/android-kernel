/** \file ScanCncnDrvSM.c
 *  \brief This file include the scan concentrator driver state machine module implementation
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

#include "ScanCncnDrvSM.h"
#include "MacServices_api.h" 
#include "report.h"
#include "siteMgrApi.h"
#include "utils.h"
#include "regulatoryDomainApi.h"
#include "healthMonitor.h"


static TI_STATUS actionUnexpected( TI_HANDLE hScanCncn );
static TI_STATUS actionNop( TI_HANDLE hScanCncn );


/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Initialize the scan concentrator driver SM.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorDrvSM_init( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    fsm_actionCell_t    smMatrix[ DRV_SCAN_NUM_OF_STATES ][ DRV_SCAN_NUM_OF_EVENTS ] =
	{
		/* next state and actions for IDLE state */
		{	
            {DRV_SCAN_STATE_SCR_REQUEST, scanConcentratorDrvSM_requestSCR},              /*"START_SCAN",*/
			{DRV_SCAN_STATE_IDLE, actionUnexpected},                                     /*"SCR_PEND",  */
			{DRV_SCAN_STATE_IDLE, actionUnexpected},                                     /*"SCR_REJECT",*/
            {DRV_SCAN_STATE_IDLE, actionUnexpected},                                     /*"PASSIVE_SCAN"*/
            {DRV_SCAN_STATE_IDLE, actionUnexpected},                                     /*"ACTIVE_SCAN"*/
            {DRV_SCAN_STATE_IDLE, actionUnexpected},                                     /*"ABORT_SCAN"*/
            {DRV_SCAN_STATE_IDLE, actionUnexpected},                                     /*"FW_RESET"*/
            {DRV_SCAN_STATE_IDLE, actionNop},                                            /*"STOP_SCAN"*/
            {DRV_SCAN_STATE_IDLE, actionUnexpected}                                      /*"SCAN_COMPLETE"*/
		},

		/* next state and actions for SCR_REQUEST state */
		{	
            {DRV_SCAN_STATE_SCR_REQUEST, actionUnexpected},                              /*"START_SCAN",*/
			{DRV_SCAN_STATE_SCR_WAIT, actionNop},                                        /*"SCR_PEND",  */
			{DRV_SCAN_STATE_IDLE, scanConcentratorDrvSM_scanRejected},                   /*"SCR_REJECT",*/
            {DRV_SCAN_STATE_PASSIVE_SCANNING, scanConcentratorDrvSM_passiveScan},        /*"PASSIVE_SCAN"*/
            {DRV_SCAN_STATE_ACTIVE_SCANNING, scanConcentratorDrvSM_activeScan},          /*"ACTIVE_SCAN"*/
            {DRV_SCAN_STATE_SCR_REQUEST, actionUnexpected},                              /*"ABORT_SCAN"*/
            {DRV_SCAN_STATE_SCR_REQUEST, actionUnexpected},                              /*"FW_RESET"*/
            {DRV_SCAN_STATE_SCR_REQUEST, actionUnexpected},                              /*"STOP_SCAN"*/
            {DRV_SCAN_STATE_SCR_REQUEST, actionUnexpected}                               /*"SCAN_COMPLETE"*/
		},

		/* next state and actions for SCR_WAIT state */
		{	
            {DRV_SCAN_STATE_SCR_WAIT, actionUnexpected},                                 /*"START_SCAN",*/
			{DRV_SCAN_STATE_SCR_WAIT, actionNop},                                        /*"SCR_PEND",  */
			{DRV_SCAN_STATE_IDLE, scanConcentratorDrvSM_scanRejected},                   /*"SCR_REJECT",*/
            {DRV_SCAN_STATE_PASSIVE_SCANNING, scanConcentratorDrvSM_passiveScan},        /*"PASSIVE_SCAN"*/
            {DRV_SCAN_STATE_ACTIVE_SCANNING, scanConcentratorDrvSM_activeScan},          /*"ACTIVE_SCAN"*/
            {DRV_SCAN_STATE_SCR_WAIT, actionUnexpected},                                 /*"ABORT_SCAN"*/
            {DRV_SCAN_STATE_SCR_WAIT, actionUnexpected},                                 /*"FW_RESET"*/
            {DRV_SCAN_STATE_IDLE, scanConcentratorDrvSM_scanRejected},                   /*"STOP_SCAN"*/
            {DRV_SCAN_STATE_SCR_WAIT, actionUnexpected}                                  /*"SCAN_COMPLETE"*/
		},

        /* next state and actions for PASSIVE_SCANNING state */
		{	
            {DRV_SCAN_STATE_PASSIVE_SCANNING, actionUnexpected},                               /*"START_SCAN",*/
			{DRV_SCAN_STATE_PASSIVE_SCANNING, actionUnexpected},                               /*"SCR_PEND",  */
			{DRV_SCAN_STATE_PASSIVE_SCANNING, actionUnexpected},                               /*"SCR_REJECT",*/
            {DRV_SCAN_STATE_PASSIVE_SCANNING, actionUnexpected},                               /*"PASSIVE_SCAN"*/
            {DRV_SCAN_STATE_ACTIVE_SCANNING, scanConcentratorDrvSM_activeScan},          /*"ACTIVE_SCAN"*/
            {DRV_SCAN_STATE_STOPPING, scanConcentratorDrvSM_abortScan},                  /*"ABORT_SCAN"*/
            {DRV_SCAN_STATE_PASSIVE_SCANNING, scanConcentratorDrvSM_recoveryDuringScan}, /*"FW_RESET"*/
            {DRV_SCAN_STATE_STOPPING, scanConcentratorDrvSM_abortScan},                  /*"STOP_SCAN"*/
            {DRV_SCAN_STATE_IDLE, scanConcentratorDrvSM_scanComplete}                    /*"SCAN_COMPLETE"*/
		},

        /* next state and actions for ACTIVE_SCANNING state */
		{	
            {DRV_SCAN_STATE_ACTIVE_SCANNING, actionUnexpected},                                     /*"START_SCAN",*/
			{DRV_SCAN_STATE_ACTIVE_SCANNING, actionUnexpected},                                     /*"SCR_PEND",  */
			{DRV_SCAN_STATE_ACTIVE_SCANNING, actionUnexpected},                                     /*"SCR_REJECT",*/
            {DRV_SCAN_STATE_ACTIVE_SCANNING, actionUnexpected},                                     /*"PASSIVE_SCAN"*/
            {DRV_SCAN_STATE_ACTIVE_SCANNING, actionUnexpected},                                     /*"ACTIVE_SCAN"*/
            {DRV_SCAN_STATE_STOPPING, scanConcentratorDrvSM_abortScan},                  /*"ABORT_SCAN"*/
            {DRV_SCAN_STATE_ACTIVE_SCANNING, scanConcentratorDrvSM_recoveryDuringScan},  /*"FW_RESET"*/
            {DRV_SCAN_STATE_STOPPING, scanConcentratorDrvSM_abortScan},                  /*"STOP_SCAN"*/
            {DRV_SCAN_STATE_IDLE, scanConcentratorDrvSM_scanComplete}                    /*"SCAN_COMPLETE"*/

		},

        /* next state and actions for STOPPING state */
		{	
            {DRV_SCAN_STATE_STOPPING, actionUnexpected},                                     /*"START_SCAN",*/
			{DRV_SCAN_STATE_STOPPING, actionUnexpected},                                     /*"SCR_PEND",  */
			{DRV_SCAN_STATE_STOPPING, actionUnexpected},                                     /*"SCR_REJECT",*/
            {DRV_SCAN_STATE_STOPPING, actionUnexpected},                                     /*"PASSIVE_SCAN"*/
            {DRV_SCAN_STATE_STOPPING, actionUnexpected},                                     /*"ACTIVE_SCAN"*/
            {DRV_SCAN_STATE_STOPPING, actionNop},                                        /*"ABORT_SCAN"*/
            {DRV_SCAN_STATE_STOPPING, scanConcentratorDrvSM_recoveryDuringScan},         /*"FW_RESET"*/
            {DRV_SCAN_STATE_STOPPING, actionNop},                                        /*"STOP_SCAN"*/
            {DRV_SCAN_STATE_IDLE, scanConcentratorDrvSM_scanComplete}                    /*"SCAN_COMPLETE"*/
		}
    };

    /* initialize current state */
    pScanConcentrator->clientSMState[ SCAN_SCC_DRIVER ] = DRV_SCAN_STATE_IDLE;

    /* configure the state machine */
	return fsm_Config( pScanConcentrator->clientSM[ SCAN_SCC_DRIVER ], (fsm_Matrix_t)smMatrix, 
                       DRV_SCAN_NUM_OF_STATES, DRV_SCAN_NUM_OF_EVENTS, 
                       (fsm_eventActivation_t)scanConcentratorDrvSM_SMEvent, pScanConcentrator->hOS );
}


#ifdef REPORT_LOG

/* state descriptions, for state machine logging */
static char stateDesc[ DRV_SCAN_NUM_OF_STATES ][ MAX_DESC_STRING_LEN ] = 
{
    "STATE_IDLE",
    "STATE_SCR_REQUEST",
    "STATE_SCR_WAIT",
    "STATE_PASSIVE_SCANNING",
    "STATE_ACTIVE_SCANNING",
    "STATE_STOPPING"
};

/* event descriptions, for state machine logging */
static char eventDesc[ DRV_SCAN_NUM_OF_EVENTS ][ MAX_DESC_STRING_LEN ] = 
{
    "EVENT_START_SCAN",
    "EVENT_SCR_PEND",
    "EVENT_SCR_REJECT",
    "EVENT_PASSIVE_SCAN",
    "EVENT_ACTIVE_SCAN",
    "EVENT_ABORT_SCAN",
    "EVENT_FW_RESET",
    "EVENT_STOP_SCAN",
    "EVENT_SCAN_COMPLETE"
};

#endif

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Processes an event.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param currentState - the current driver SM state.\n
 * \param event - the event to handle.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorDrvSM_SMEvent( TI_HANDLE hScanCncn, scan_drvSMStates_e* currentState, 
                                         scan_drvSMEvents_e event )
{
    scanConcentrator_t *pScanConcentrator = (scanConcentrator_t *)hScanCncn;
	TI_STATUS status = OK;
	UINT8 nextState;

    /* obtain the next state */
	status = fsm_GetNextState( pScanConcentrator->clientSM[ SCAN_SCC_DRIVER ], *(UINT8*)currentState,
                               (UINT8)event, &nextState );
	if ( status != OK )
	{
		WLAN_REPORT_SM( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG, ("Failed getting driver scan next state.\n") );
		return NOK;
	}

    /* report the move */
    WLAN_REPORT_SM( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG, 
                    ("DRIVER SCAN: <%s, %s> --> %s\n\n",
                    stateDesc[*((UINT8*)currentState)],
                    eventDesc[(UINT8)event],
                    stateDesc[nextState]) );

    /* move */
    return fsm_Event( pScanConcentrator->clientSM[ SCAN_SCC_DRIVER ], (UINT8*)currentState, (UINT8)event, hScanCncn );
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief SM action - handles a start scan event (by requesting the SCR)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorDrvSM_requestSCR( TI_HANDLE hScanCncn )
{
    scr_clientRequestStatus_e scrReplyStatus;
    scr_pendReason_e scrPendReason;
	scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("DRV SM: Requesting SCR.\n") );

    /* request the SCR as driver client, and act according to return status */
    switch (  scrReplyStatus = scr_clientRequest( pScanConcentrator->hSCR, SCR_CID_DRIVER_FG_SCAN, &scrPendReason ) )
    {
    case SCR_CRS_PEND:
        WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                                 ("DRV SM: SCR pending, pend reason: %d.\n", scrPendReason) );
        
        /* check the pending reason */
        if ( SCR_PR_DIFFERENT_GROUP_RUNNING == scrPendReason )
        {
            /* send a reject event to the SM - would not scan if not in connecting mode! */
            pScanConcentrator->scanResult[ SCAN_SCC_DRIVER ] = SCAN_CRS_SCAN_FAILED; 
            return scanConcentratorDrvSM_SMEvent( hScanCncn, 
                                                  (scan_drvSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_DRIVER ]),
                                                  DRV_SCAN_EVENT_SCR_REJECT );
        }
        else
        {
            /* send a pend event to the SM */
            return scanConcentratorDrvSM_SMEvent( hScanCncn, 
                                                  (scan_drvSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_DRIVER ]),
                                                   DRV_SCAN_EVENT_SCR_PEND );
        }
 /*       break; - unreachable */

    case SCR_CRS_RUN:
        /* send an event to the SM */
        WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                                 ("DRV SM: SCR acquired.\n") );
        return scanConcentratorDrvSM_SMEvent( hScanCncn, 
                                              (scan_drvSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_DRIVER ]),
                                              scanConcentrator_getNextDriverEvent( hScanCncn ) );
/*        break; - unreachable */

    default:
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("DRV SM: SCR returned unrecognized status: %d.\n", scrReplyStatus) );
		pScanConcentrator->scanResult[ SCAN_SCC_DRIVER ] = SCAN_CRS_SCAN_FAILED;
        scanConcentratorDrvSM_SMEvent( hScanCncn, 
                                       (scan_drvSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_DRIVER ]),
                                       DRV_SCAN_EVENT_SCAN_COMPLETE );
        return NOK;
/*        break; - unreachable */
    }

 /*   return OK; - unreachable */
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief SM action - handles a FW reset event (by calling the complete CB)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorDrvSM_callCompleteCB( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("DRV SM: calling complete CB.\n") );

    /* mark that no scan is currently running */
    pScanConcentrator->currentRunningScanClient = SCAN_SCC_NO_CLIENT;

    /* notify scan complete to scan mngr */
    if ( FALSE == pScanConcentrator->bInRequest )
    {
        pScanConcentrator->scanResultCB[ SCAN_SCC_DRIVER ]( pScanConcentrator->scanResultCBObj[ SCAN_SCC_DRIVER ],
                                                        pScanConcentrator->scanResult[ SCAN_SCC_DRIVER ],
                                                        NULL, 0xffff );
    }

    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief SM action - handles a passive scan event (by starting a passive scan)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorDrvSM_passiveScan( TI_HANDLE hScanCncn )
{
    TI_STATUS status;
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;
    int i;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("DRV SM: Sending passive scan command to scan SRV.\n") );

    /* mark that this scan is currently running */
    pScanConcentrator->currentRunningScanClient = SCAN_SCC_DRIVER;

    /* if the requested scan type is not passive in the first place, change it to passive */
    pScanConcentrator->drvScanRequestType = pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].scanType;
    if ( (SCAN_TYPE_NORMAL_ACTIVE == pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].scanType) ||
         (SCAN_TYPE_TRIGGERED_ACTIVE == pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].scanType) )
    {
        pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].scanType = SCAN_TYPE_NORMAL_PASSIVE;
        
        /* save requested SSID, and write broadcast SSID instead. This is done to find ANY beacon for .11d
           and .11h, in case the desired SSID is not being broadcast in beacons */
        os_memoryCopy( pScanConcentrator->hOS, &(pScanConcentrator->drvScanSsid), 
                       &(pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].desiredSsid), sizeof(ssid_t) );
        pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].desiredSsid.len = 0;

        /* save max and min dwell time for all channels, and replace them with default values */
        for ( i = 0; i < pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].numOfChannels; i++ )
        {
            pScanConcentrator->drvScanMaxDwellTime[ i ] = 
                pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].channelEntry[ i ].normalChannelEntry.maxChannelDwellTime;
            pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].channelEntry[ i ].normalChannelEntry.maxChannelDwellTime = 
                pScanConcentrator->initParams.passiveScanDwellTime;
            pScanConcentrator->drvScanMinDwellTime[ i ] = 
                pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].channelEntry[ i ].normalChannelEntry.minChannelDwellTime;
            pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].channelEntry[ i ].normalChannelEntry.minChannelDwellTime = 
                pScanConcentrator->initParams.passiveScanDwellTime;
			if (pScanConcentrator->bUseSGParams)
			{
				/* increasing dwell time in case BT is active to compensate loss of dwelling time */
				pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].channelEntry[ i ].normalChannelEntry.minChannelDwellTime =
					(pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].channelEntry[ i ].normalChannelEntry.minChannelDwellTime * 
					(100 + pScanConcentrator->SGcompensationPercent)) / 100;
				pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].channelEntry[ i ].normalChannelEntry.maxChannelDwellTime =
					(pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].channelEntry[ i ].normalChannelEntry.maxChannelDwellTime * 
					(100 + pScanConcentrator->SGcompensationPercent)) / 100;
				WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("SoftGemini compensation time for channel %d : min = %d, max = %d  .\n",
							 i,pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].channelEntry[ i ].normalChannelEntry.minChannelDwellTime,
							 pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].channelEntry[ i ].normalChannelEntry.maxChannelDwellTime) );
		
			}
        }
    }

    /* ask the reg domain which channels are allowed for the requested scan type */
    scanConcentrator_verifyChannelsWithRegDomain( hScanCncn, &(pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ]) );

    /* if no channels are available for scan, return negative result */
    if ( 0 == pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].numOfChannels )
    {
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("DRV SM: No cahnnels available for passive scan, quitting.\n") );
        scanConcentratorDrvSM_handleScanError( hScanCncn );
        return NOK;
    }

    /* register for scan results with the MLME parser */
    if ( OK != (status = mlmeParser_registerForBeaconAndProbeResp( pScanConcentrator->hMlme, 
                                                                   scanConcentrator_mlmeResultCB, 
                                                                   hScanCncn )) )
    {
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("DRV SM: MLME result registration failed.\n") );
        scanConcentratorDrvSM_handleScanError( hScanCncn );
        return NOK;
    }

	 /* stop built-in test timer, to avoid TX stuck due to heavy traffic and unknown scan result time originally at scanSrv*/
	healthMonitor_suspendPeriodicTest( pScanConcentrator->hHealthMonitor );

    /* call the scan SRV start scan */
    if ( OK != (status = MacServices_scanSRV_scan( pScanConcentrator->hMacServices, 
                                              &(pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ]),
                                              FALSE,
                                              FALSE,
                                              FALSE,
                                              POWER_SAVE_KEEP_CURRENT,
                                              FALSE ,
											  NULL,NULL)) )
    {
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("DRV SM: scan SRV returned status %d, quitting driver passive scan.\n", status) );

        /* unregister at the MLME for scan result frames */
        mlmeParser_unregisterForBeaconAndProbeResp( pScanConcentrator->hMlme );
        scanConcentratorDrvSM_handleScanError( hScanCncn );
        return NOK;
    }

    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief SM action - handles an active scan event (by starting an active scan)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorDrvSM_activeScan( TI_HANDLE hScanCncn )
{
    TI_STATUS status;
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("DRV SM: Sending active scan command to scan SRV.\n") );

    /* mark that this scan is currently running */
    pScanConcentrator->currentRunningScanClient = SCAN_SCC_DRIVER;

    /* ask the reg domain which channels are allowed for the requested scan type */
    scanConcentrator_verifyChannelsWithRegDomain( hScanCncn, &(pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ]) );

    /* if no channels are available for scan, return negative result */
    if ( 0 == pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].numOfChannels )
    {
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("DRV SM: No cahnnels available for active scan, quitting.\n") );
        scanConcentratorDrvSM_handleScanError( hScanCncn );
        return NOK;
    }


    /* register for scan results with the MLME parser */
    if ( OK != mlmeParser_registerForBeaconAndProbeResp( pScanConcentrator->hMlme, 
                                                         scanConcentrator_mlmeResultCB, 
                                                         hScanCncn ) )
    {
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("DRV SM: MLME result registration failed.\n") );
        scanConcentratorDrvSM_handleScanError( hScanCncn );
        return NOK;
    }

	/* stop built-in test timer, to avoid TX stuck due to heavy traffic and unknown scan result time originally at scanSrv*/
	healthMonitor_suspendPeriodicTest( pScanConcentrator->hHealthMonitor );

    if ( OK != (status = MacServices_scanSRV_scan( pScanConcentrator->hMacServices, 
                                       &(pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ]),
                                       FALSE,
                                       FALSE,
                                       FALSE,
                                       POWER_SAVE_KEEP_CURRENT,
                                       FALSE ,
									   NULL,NULL)) )
    {
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("DRV SM: scan SRV returned status %d, quitting driver active scan.\n", status) );
	    /* unregister at the MLME for scan result frames */
        mlmeParser_unregisterForBeaconAndProbeResp( pScanConcentrator->hMlme );
        scanConcentratorDrvSM_handleScanError( hScanCncn );
        return NOK;
    }

    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief SM action - handles an abort scan or stop scan event (by stopping the actual scan)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorDrvSM_abortScan( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("DRV SM: aborting scan.\n") );

    /* call the scan SRV stop scan (don't exit driver mode, as it wasn't entered for driver scan */
    MacServices_scanSRV_stopScan( pScanConcentrator->hMacServices, FALSE , NULL , NULL );

    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 10-July-2005\n
 * \brief SM action - handles a recovery event (calls the scan SRV abort on FW reset and than finishes scan)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorDrvSM_recoveryDuringScan( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("DRV SM: Recovery occured!.\n") );
    
    /* reset the scan SRV */
    MacServices_scanSRV_stopOnFWReset( pScanConcentrator->hMacServices );

    /* send a scan complete event to the SM */
    return scanConcentratorDrvSM_SMEvent( hScanCncn, 
                                          (scan_drvSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_DRIVER ]),
                                          DRV_SCAN_EVENT_SCAN_COMPLETE );
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief SM action - handles a scan complete event (by releasing the SCR and calling the scan complete CB)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorDrvSM_scanComplete( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("DRV SM: Scan is complete.\n") );

	/* unregister at the MLME for scan result frames */
    mlmeParser_unregisterForBeaconAndProbeResp( pScanConcentrator->hMlme );

    /* mark that this scan is no longer running */
    pScanConcentrator->currentRunningScanClient = SCAN_SCC_NO_CLIENT;

    /* release the SCR */
    scr_clientComplete( pScanConcentrator->hSCR, SCR_CID_DRIVER_FG_SCAN );

    /* notify the scan complete to the scan mngr */
    if ( FALSE == pScanConcentrator->bInRequest )
    {
        pScanConcentrator->scanResultCB[ SCAN_SCC_DRIVER ]( pScanConcentrator->scanResultCBObj[ SCAN_SCC_DRIVER ],
                                                            pScanConcentrator->scanResult[ SCAN_SCC_DRIVER ],
                                                            NULL, 0xffff );    
    }

    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief SM action - handles a scan reject event (abort scan before scan actually started)\n
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorDrvSM_scanRejected( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("DRV SM: Scan is Rejected.\n") );

    /* release the SCR */
    scr_clientComplete( pScanConcentrator->hSCR, SCR_CID_DRIVER_FG_SCAN );

    /* notify the scan complete to the scan mngr */
    if ( FALSE == pScanConcentrator->bInRequest )
    {
        pScanConcentrator->scanResultCB[ SCAN_SCC_DRIVER ]( pScanConcentrator->scanResultCBObj[ SCAN_SCC_DRIVER ],
                                                            pScanConcentrator->scanResult[ SCAN_SCC_DRIVER ],
                                                            NULL, 0xffff );    
    }

    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 11-Jan-2005\n
 * \brief Handles an unexpected event.\n
 *
 * Function Scope \e Private.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return always OK.\n
 */
TI_STATUS actionUnexpected( TI_HANDLE hScanCncn ) 
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_ERROR( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG, ("Driver scan state machine error, unexpected Event, state=%d\n\n",
																	  pScanConcentrator->clientSMState[SCAN_SCC_DRIVER] ) );
	
    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 10-Jan-2005\n
 * \brief Handles an event that doesn't require any action.\n
 *
 * Function Scope \e Private.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return always OK.\n
 */
TI_STATUS actionNop( TI_HANDLE hScanCncn )
{
    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 09-Jan-2005\n
 * \brief Determines the next event to send to the driver SM (when a scan can be run)\n
 *
 * Function Scope \e Private.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return the next event to use with the driver SM.\n
 */
scan_drvSMEvents_e scanConcentrator_getNextDriverEvent( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;
    paramInfo_t         param;
    BOOL                is_801_11d_enabled, is_801_11h_enabled;
    int					i;

    /* getting regulatory Domain status - is 802.11d enabled ? */
    param.paramType = REGULATORY_DOMAIN_ENABLED_PARAM;
    regulatoryDomain_getParam( pScanConcentrator->hRegulatoryDomain,&param );
    is_801_11d_enabled = param.content.regulatoryDomainEnabled;

    param.paramType = REGULATORY_DOMAIN_MANAGEMENT_CAPABILITY_ENABLED_PARAM;
    regulatoryDomain_getParam( pScanConcentrator->hRegulatoryDomain,&param );
	is_801_11h_enabled = param.content.spectrumManagementEnabled;

    /* The next event (or rather, scan type) is determined according to the driver SM state.
       The order is passive-active, when either the active or passive scan can be eliminated (but the order is kept).
     */
    switch (pScanConcentrator->clientSMState[ SCAN_SCC_DRIVER ])
    {
    /* active scan already performed, send a scan complete event */
    case DRV_SCAN_STATE_ACTIVE_SCANNING:
    /* Stop or abort is in progress, also send a scan complete event */
    case DRV_SCAN_STATE_STOPPING:
        return DRV_SCAN_EVENT_SCAN_COMPLETE;
/*        break; - unreachable */

    /* passive scan already performed, check if active scan is needed. */
    case DRV_SCAN_STATE_PASSIVE_SCANNING:
        /* first, restore the user requested scan type, */
        pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].scanType = pScanConcentrator->drvScanRequestType;
        /* the user desired SSID */
        os_memoryCopy( pScanConcentrator->hOS, &(pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].desiredSsid),
                       &(pScanConcentrator->drvScanSsid), sizeof(ssid_t) );
        /* and dwell times for all channels */
        for ( i = 0; i < pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].numOfChannels; i++ )
        {
            pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].channelEntry[ i ].normalChannelEntry.maxChannelDwellTime = 
                pScanConcentrator->drvScanMaxDwellTime[ i ];
            pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].channelEntry[ i ].normalChannelEntry.minChannelDwellTime = 
                pScanConcentrator->drvScanMinDwellTime[ i ];
        }

        /* send scan complete event if:
           1. The requested scan type is passive
           2. .11d is enabled and no country IE is available
         */

		/* Get country code status */
		param.paramType			 = REGULATORY_DOMAIN_IS_COUNTRY_FOUND;
		param.content.eRadioBand = pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].band;
		regulatoryDomain_getParam(pScanConcentrator->hRegulatoryDomain,&param);
		
        if ( (pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].scanType == SCAN_TYPE_NORMAL_PASSIVE) ||
             (pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].scanType == SCAN_TYPE_TRIGGERED_PASSIVE) ||
             (pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].scanType == SCAN_TYPE_SPS) ||
             ((TRUE == is_801_11d_enabled) && 
              ( !param.content.bIsCountryFound )) )
        {
            return DRV_SCAN_EVENT_SCAN_COMPLETE;
        }
        /* otherwise, send active scan event */
        else
        {
            /* Very ugly - but nowhere better to do it...
               If we continue to active scan after passive scan, we are already registered
               in the MLME parser. We MUST unregister here, or else the registration at
               the active scan call will fail */
            mlmeParser_unregisterForBeaconAndProbeResp( pScanConcentrator->hMlme );
            return DRV_SCAN_EVENT_ACTIVE_SCAN;
        }
/*        break; - unreachable */

    /* no scan already performed, any scan can be issued */
    case DRV_SCAN_STATE_SCR_WAIT:
    case DRV_SCAN_STATE_SCR_REQUEST:
        /* do a passive scan if:
           1. it was requested
           2. .11d is enabled and country is unknown
           3. .11h is enabled and the scan is on A band (always perform passive before active, to validate channels)
         */

		/* Get country code status */
		param.paramType			 = REGULATORY_DOMAIN_IS_COUNTRY_FOUND;
		param.content.eRadioBand = pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].band ;
		regulatoryDomain_getParam(pScanConcentrator->hRegulatoryDomain,&param);

        if ( (pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].scanType == SCAN_TYPE_NORMAL_PASSIVE) ||
             (pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].scanType == SCAN_TYPE_TRIGGERED_PASSIVE) ||
             (pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].scanType == SCAN_TYPE_SPS) ||
             ((TRUE == is_801_11d_enabled) && 
              (!param.content.bIsCountryFound)) ||
             ((TRUE == is_801_11h_enabled) && 
              (RADIO_BAND_5_0_GHZ == pScanConcentrator->clientScanParams[ SCAN_SCC_DRIVER ].band)) )
        {
            return DRV_SCAN_EVENT_PASSIVE_SCAN;
        }
        /* else, do an active scan if
         */
        else
        {
            return DRV_SCAN_EVENT_ACTIVE_SCAN;
        }
/*        break; - unreachable */

    default:
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("DRV SM: State %d is invalid to obtain next event", pScanConcentrator->clientSMState[ SCAN_SCC_DRIVER ]) );
        break;
    }

    return DRV_SCAN_EVENT_SCAN_COMPLETE;
}

/**
 * \author Ronen Kalish\n
 * \date 07-Feb-2005\n
 * \brief Handles an error during scan operation
 *
 * Function Scope \e Private.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 */
void scanConcentratorDrvSM_handleScanError( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;
    
    /* mark the return status */
    pScanConcentrator->scanResult[ SCAN_SCC_DRIVER ] = SCAN_CRS_SCAN_FAILED;

    /* send a scan complete event */
    scanConcentratorDrvSM_SMEvent( hScanCncn, 
                                   (scan_drvSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_DRIVER ]),
                                   DRV_SCAN_EVENT_SCAN_COMPLETE );
}
