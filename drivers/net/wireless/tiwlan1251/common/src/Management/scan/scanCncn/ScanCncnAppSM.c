/** \file ScanCncnAppSM.c
 *  \brief This file include the scan concentrator application SM module implementation
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

#include "ScanCncnAppSM.h"
#include "MacServices_api.h" 
#include "report.h"
#include "healthMonitor.h"

static TI_STATUS actionUnexpected( TI_HANDLE hScanCncn );
static TI_STATUS actionNop( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Initialize the scan concentrator application SM.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorAppSM_init( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    fsm_actionCell_t    smMatrix[ APP_SCAN_NUM_OF_STATES ][ APP_SCAN_NUM_OF_EVENTS ] =
	{
		/* next state and actions for IDLE state */
		{	
            {APP_SCAN_STATE_SCR_REQUEST, scanConcentratorAppSM_requestSCR},              /*"START_SCAN",*/
			{APP_SCAN_STATE_IDLE, actionUnexpected},                                     /*"SCR_RUN",*/
			{APP_SCAN_STATE_IDLE, actionUnexpected},                                     /*"SCR_PEND",  */
            {APP_SCAN_STATE_IDLE, actionNop},                                            /*"STOP_SCAN"*/
            {APP_SCAN_STATE_IDLE, actionUnexpected},                                     /*"ABORT_SCAN"*/
            {APP_SCAN_STATE_IDLE, actionUnexpected},                                     /*"FW_RESET"*/
            {APP_SCAN_STATE_IDLE, actionUnexpected}                                      /*"SCAN_COMPLETE"*/
		},

		/* next state and actions for SCR_REQUEST state */
		{	
            {APP_SCAN_STATE_SCR_REQUEST, actionUnexpected},                                     /*"START_SCAN",*/
			{APP_SCAN_STATE_SCANNING, scanConcentratorAppSM_startScan},                  /*"SCR_RUN",*/
			{APP_SCAN_STATE_IDLE, scanConcentratorAppSM_scanRejected},                   /*"SCR_PEND",  */
            {APP_SCAN_STATE_SCR_REQUEST, actionUnexpected},                                     /*"STOP_SCAN"*/
            {APP_SCAN_STATE_SCR_REQUEST, actionUnexpected},                                     /*"ABORT_SCAN"*/
            {APP_SCAN_STATE_SCR_REQUEST, actionUnexpected},                                     /*"FW_RESET"*/
            {APP_SCAN_STATE_SCR_REQUEST, actionUnexpected}                                      /*"SCAN_COMPLETE"*/
		},

        /* next state and actions for SCANNING state */
		{	
            {APP_SCAN_STATE_SCANNING, actionUnexpected},                                    /*"START_SCAN",*/
			{APP_SCAN_STATE_SCANNING, actionUnexpected},                                    /*"SCR_RUN",*/
			{APP_SCAN_STATE_SCANNING, actionUnexpected},                                    /*"SCR_PEND",  */
            {APP_SCAN_STATE_STOPPING, scanConcentratorAppSM_abortScan},                  /*"STOP_SCAN"*/
            {APP_SCAN_STATE_STOPPING, scanConcentratorAppSM_abortScan},                  /*"ABORT_SCAN"*/
            {APP_SCAN_STATE_SCANNING, scanConcentratorAppSM_recoveryDuringScan},         /*"FW_RESET"*/
            {APP_SCAN_STATE_IDLE, scanConcentratorAppSM_scanComplete}                    /*"SCAN_COMPLETE"*/
		},

        /* next state and actions for STOPPING state */
		{	
            {APP_SCAN_STATE_STOPPING, actionUnexpected},                                    /*"START_SCAN",*/
			{APP_SCAN_STATE_STOPPING, actionUnexpected},                                    /*"SCR_RUN",*/
			{APP_SCAN_STATE_STOPPING, actionUnexpected},                                    /*"SCR_PEND",  */
            {APP_SCAN_STATE_STOPPING, actionNop},                                        /*"STOP_SCAN"*/
            {APP_SCAN_STATE_STOPPING, actionNop},                                        /*"ABORT_SCAN"*/
            {APP_SCAN_STATE_STOPPING, scanConcentratorAppSM_recoveryDuringScan},         /*"FW_RESET"*/
            {APP_SCAN_STATE_IDLE, scanConcentratorAppSM_scanComplete}                    /*"SCAN_COMPLETE"*/
		}
    };

    /* initialize current state */
    pScanConcentrator->clientSMState[ SCAN_SCC_APP ] = APP_SCAN_STATE_IDLE;

    /* configure the state machine */
	return fsm_Config( pScanConcentrator->clientSM[ SCAN_SCC_APP ], (fsm_Matrix_t)smMatrix, 
                       APP_SCAN_NUM_OF_STATES, APP_SCAN_NUM_OF_EVENTS, 
                       (fsm_eventActivation_t)scanConcentratorAppSM_SMEvent, pScanConcentrator->hOS );
}


#ifdef REPORT_LOG

/* state descriptions, for state machine logging */
static char stateDesc[ APP_SCAN_NUM_OF_STATES ][ MAX_DESC_STRING_LEN ] = 
{
    "STATE_IDLE",
	"STATE_SCR_REQUEST",
	"STATE_SCANNING",
	"STATE_STOPPING"
};

/* event descriptions, for state machine logging */
static char eventDesc[ APP_SCAN_NUM_OF_EVENTS ][ MAX_DESC_STRING_LEN ] = 
{
    "EVENT_START_SCAN",
    "EVENT_SCR_RUN",
    "EVENT_SCR_PEND",
    "EVENT_STOP_SCAN",
    "EVENT_ABORT_SCAN",
    "EVENT_FW_RESET",
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
 * \param currentState - the current App SM state.\n
 * \param event - the event to handle.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorAppSM_SMEvent( TI_HANDLE hScanCncn, scan_appSMStates_e* currentState, 
                                         scan_appSMEvents_e event )
{
    scanConcentrator_t *pScanConcentrator = (scanConcentrator_t *)hScanCncn;
	TI_STATUS status = OK;
	UINT8 nextState;

    /* obtain the next state */
	status = fsm_GetNextState( pScanConcentrator->clientSM[ SCAN_SCC_APP ], *(UINT8*)currentState,
                                (UINT8)event, &nextState );
	if ( status != OK )
	{
		WLAN_REPORT_SM( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG, ("Failed getting application scan next state.\n") );
		return NOK;
	}

    /* report the move */
    WLAN_REPORT_SM( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG, 
                    ("APP SCAN: <%s, %s> --> %s\n\n",
                    stateDesc[*((UINT8*)currentState)],
                    eventDesc[(UINT8)event],
                    stateDesc[nextState]) );

    /* move */
    return fsm_Event( pScanConcentrator->clientSM[ SCAN_SCC_APP ], (UINT8*)currentState, (UINT8)event, hScanCncn );
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
TI_STATUS scanConcentratorAppSM_requestSCR( TI_HANDLE hScanCncn )
{
    scr_clientRequestStatus_e scrReplyStatus;
	scr_pendReason_e scrPendReason;
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("APP SM: Requesting SCR.\n") );
    
    /* request the SCR as application scan client, and act according to return status */
    switch ( scrReplyStatus = scr_clientRequest( pScanConcentrator->hSCR, SCR_CID_APP_SCAN, &scrPendReason ) )
    {
    case SCR_CRS_PEND:
        /* send a pend event to the SM */
        WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                                 ("APP SM: SCR pending, pend reason: %d.\n", scrPendReason) );
		pScanConcentrator->scanResult[ SCAN_SCC_APP ] = SCAN_CRS_SCAN_FAILED;
        return scanConcentratorAppSM_SMEvent( hScanCncn, 
                                              (scan_appSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_APP ]),
                                              APP_SCAN_EVENT_SCR_PEND );
/*        break; - unreachable */

    case SCR_CRS_RUN:
        /* send a run event to the SM */
        WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                                 ("APP SM: SCR acquired.\n") );
        return scanConcentratorAppSM_SMEvent( hScanCncn, 
                                              (scan_appSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_APP ]),
                                              APP_SCAN_EVENT_SCR_RUN );
/*        break; - unreachable */

    default:
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("APP SM: SCR returned unrecognized status: %d.\n", scrReplyStatus) );
        /* Notify scan complete to recover from this error */
		pScanConcentrator->scanResult[ SCAN_SCC_APP ] = SCAN_CRS_SCAN_FAILED;
        scanConcentratorAppSM_SMEvent( hScanCncn,
                                       (scan_appSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_APP ]),
                                       APP_SCAN_EVENT_SCAN_COMPLETE );
        
        return NOK;
/*        break; - unreachable */
    }

 /*   return OK; - unreachable */
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief SM action - handles a SCR run event (starts the actual scan)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorAppSM_startScan( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;
    TI_STATUS status;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("APP SM: Sending scan command to scan SRV.\n") );

    /* mark that this scan is currently running */
    pScanConcentrator->currentRunningScanClient = SCAN_SCC_APP;

    /* register for scan results with the MLME parser */
    if ( OK != (status = mlmeParser_registerForBeaconAndProbeResp( pScanConcentrator->hMlme, 
                                                                   scanConcentrator_mlmeResultCB, 
                                                                   hScanCncn )) )
    {
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("APP SM: MLME result registration failed.\n") );

        /* mark the return status */
        pScanConcentrator->scanResult[ SCAN_SCC_APP ] = SCAN_CRS_SCAN_FAILED;

        /* could not start scan, send a scan complete event */
        scanConcentratorAppSM_SMEvent( hScanCncn, 
                                       (scan_appSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_APP ]),
                                       APP_SCAN_EVENT_SCAN_COMPLETE );

        return NOK;
    }
	
	 /* stop built-in test timer, to avoid TX stuck due to heavy traffic and unknown scan result time originally at scanSrv*/
	healthMonitor_suspendPeriodicTest( pScanConcentrator->hHealthMonitor );

    /* call the scan SRV start scan - enter driver mode (PS) only if station is connected */
    if ( OK != (status = MacServices_scanSRV_scan( pScanConcentrator->hMacServices, 
                                              &(pScanConcentrator->clientScanParams[ SCAN_SCC_APP ]),
                                              FALSE,
                                              (STA_CONNECTED == pScanConcentrator->connectionStatus ? TRUE : FALSE),
                                              FALSE,
                                              POWER_SAVE_ON, /* this parameter is used only when driver mode requested */
                                              (STA_CONNECTED == pScanConcentrator->connectionStatus ? TRUE : FALSE),
											  NULL,NULL)))
    {
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("APP SM: scan SRV returned status %d, quitting app scan.\n", status) );

        /* mark the return status */
        pScanConcentrator->scanResult[ SCAN_SCC_APP ] = SCAN_CRS_SCAN_FAILED;

	    /* unregister at the MLME for scan result frames */
        mlmeParser_unregisterForBeaconAndProbeResp( pScanConcentrator->hMlme );

        /* could not start scan, send a scan complete event */
        scanConcentratorAppSM_SMEvent( hScanCncn, 
                                       (scan_appSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_APP ]),
                                       APP_SCAN_EVENT_SCAN_COMPLETE );

        return NOK;
    }

    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief SM action - handles an abort scan event (call the scan SRV stop)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorAppSM_abortScan( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("APP SM: aborting scan.\n") );
    
    /* call the scan SRV stop scan. Whether to exit driver mode depends on the connection status
       (scan when not connected or in IBSS does not request PS) and on the flag set by the
       scan concentrator API functions (abort or stop)*/
    if ( pScanConcentrator->connectionStatus != STA_CONNECTED )
    {
        MacServices_scanSRV_stopScan( pScanConcentrator->hMacServices, FALSE , NULL , NULL );
    }
    else
    {
        BOOLEAN  bSendNullData = (pScanConcentrator->bAbortOrStop == SCAN_CNCN_STOP) ? TRUE : FALSE;
        MacServices_scanSRV_stopScan( pScanConcentrator->hMacServices,  bSendNullData, NULL , NULL );
    }

    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief SM action - handles a scan complete event (releases the SCR and call the scan complete CB)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorAppSM_scanComplete( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("APP SM: Scan is complete.\n") );

	/* unregister at the MLME for scan result frames */
    mlmeParser_unregisterForBeaconAndProbeResp( pScanConcentrator->hMlme );

    /* mark that this scan is no longer running */
    pScanConcentrator->currentRunningScanClient = SCAN_SCC_NO_CLIENT;

    /* release the SCR with the client ID used for request */
    scr_clientComplete( pScanConcentrator->hSCR, SCR_CID_APP_SCAN );

    /* notify the scan complete to the scan mngr */
    if ( FALSE == pScanConcentrator->bInRequest )
    {
        pScanConcentrator->scanResultCB[ SCAN_SCC_APP ]( pScanConcentrator->scanResultCBObj[ SCAN_SCC_APP ],
                                                         pScanConcentrator->scanResult[ SCAN_SCC_APP ],
                                                         NULL, 0xffff );    
    }

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
TI_STATUS scanConcentratorAppSM_recoveryDuringScan( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("APP SM: Recovery occured!.\n") );
    
    /* reset the scan SRV */
    MacServices_scanSRV_stopOnFWReset( pScanConcentrator->hMacServices );

    /* send a scan complete event to the SM */
    return scanConcentratorAppSM_SMEvent( hScanCncn, 
                                          (scan_appSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_APP ]),
                                          APP_SCAN_EVENT_SCAN_COMPLETE );    
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
TI_STATUS scanConcentratorAppSM_scanRejected( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("APP SM: Scan is rejected.\n") );

    /* release the SCR with the client ID used for request */
    scr_clientComplete( pScanConcentrator->hSCR, SCR_CID_APP_SCAN );

    /* notify the scan complete to the scan mngr */
    if ( FALSE == pScanConcentrator->bInRequest )
    {
        pScanConcentrator->scanResultCB[ SCAN_SCC_APP ]( pScanConcentrator->scanResultCBObj[ SCAN_SCC_APP ],
                                                         pScanConcentrator->scanResult[ SCAN_SCC_APP ],
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

    WLAN_REPORT_ERROR( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG, 
					("Application scan state machine error, unexpected Event, state=%d\n\n",
					 pScanConcentrator->clientSMState[SCAN_SCC_APP]) );
	
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

