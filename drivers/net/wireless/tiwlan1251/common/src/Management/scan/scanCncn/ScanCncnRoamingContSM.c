/** \file ScanCncnRoamingContSM.c
 *  \brief This file include the scan concentrator continuous scan for roaming module implementation
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

#include "ScanCncnRoamingContSM.h"
#include "MacServices_api.h" 
#include "report.h"
#include "siteMgrApi.h"
#include "healthMonitor.h"

static TI_STATUS actionUnexpected( TI_HANDLE hScanCncn );
static TI_STATUS actionNop( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Initialize the scan concentrator continuous scan for roaming SM.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorRoamingContSM_init( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    fsm_actionCell_t    smMatrix[ CONT_SCAN_NUM_OF_STATES ][ CONT_SCAN_NUM_OF_EVENTS ] =
	{
		/* next state and actions for IDLE state */
		{	
            {CONT_SCAN_STATE_SCR_REQUEST, scanConcentratorRoamingContSM_requestSCR},      /*"START_SCAN",*/
			{CONT_SCAN_STATE_IDLE, actionUnexpected},                                     /*"SCR_RUN",*/
			{CONT_SCAN_STATE_IDLE, actionUnexpected},                                     /*"SCR_PEND",  */
			{CONT_SCAN_STATE_IDLE, actionUnexpected},                                     /*"SCR_REJECT",*/
            {CONT_SCAN_STATE_IDLE, actionUnexpected},                                     /*"ABORT_SCAN"*/
            {CONT_SCAN_STATE_IDLE, actionUnexpected},                                     /*"FW_RESET"*/
            {CONT_SCAN_STATE_IDLE, actionNop},                                            /*"STOP_SCAN"*/
            {CONT_SCAN_STATE_IDLE, actionUnexpected}                                      /*"SCAN_COMPLETE"*/
            
		},

		/* next state and actions for SCR_REQUEST state */
		{	
            {CONT_SCAN_STATE_SCR_REQUEST, actionUnexpected},                                     /*"START_SCAN",*/
            {CONT_SCAN_STATE_SCANNING, scanConcentratorRoamingContSM_startScan},          /*"SCR_RUN",*/
			{CONT_SCAN_STATE_SCR_WAIT, actionNop},                                        /*"SCR_PEND",  */
			{CONT_SCAN_STATE_IDLE, scanConcentratorRoamingContSM_scanRejected},           /*"SCR_REJECT",*/
            {CONT_SCAN_STATE_SCR_REQUEST, actionUnexpected},                                     /*"ABORT_SCAN"*/
            {CONT_SCAN_STATE_SCR_REQUEST, actionUnexpected},                                     /*"FW_RESET"*/
            {CONT_SCAN_STATE_SCR_REQUEST, actionUnexpected},                                     /*"STOP_SCAN"*/
            {CONT_SCAN_STATE_SCR_REQUEST, actionUnexpected}                                      /*"SCAN_COMPLETE"*/
		},

		/* next state and actions for SCR_WAIT state */
		{	
            {CONT_SCAN_STATE_SCR_WAIT, actionUnexpected},                                     /*"START_SCAN",*/
			{CONT_SCAN_STATE_SCANNING, scanConcentratorRoamingContSM_startScan},          /*"SCR_RUN",*/
			{CONT_SCAN_STATE_SCR_WAIT, actionNop},                                        /*"SCR_PEND",  */
			{CONT_SCAN_STATE_IDLE, scanConcentratorRoamingContSM_scanRejected},           /*"SCR_REJECT",*/
            {CONT_SCAN_STATE_SCR_WAIT, actionUnexpected},                                     /*"ABORT_SCAN"*/
            {CONT_SCAN_STATE_SCR_WAIT, actionUnexpected},                                     /*"FW_RESET"*/
            {CONT_SCAN_STATE_IDLE, scanConcentratorRoamingContSM_scanRejected},           /*"STOP_SCAN"*/
            {CONT_SCAN_STATE_SCR_WAIT, actionUnexpected}                                      /*"SCAN_COMPLETE"*/
		},

        /* next state and actions for SCANNING state */
        {	 
            {CONT_SCAN_STATE_SCANNING, actionUnexpected},                                     /*"START_SCAN",*/
			{CONT_SCAN_STATE_SCANNING, actionUnexpected},                                     /*"SCR_RUN",*/
			{CONT_SCAN_STATE_SCANNING, actionUnexpected},                                     /*"SCR_PEND",  */
			{CONT_SCAN_STATE_SCANNING, actionUnexpected},                                     /*"SCR_REJECT",*/
            {CONT_SCAN_STATE_STOPPING, scanConcentratorRoamingContSM_abortScan},          /*"ABORT_SCAN"*/
            {CONT_SCAN_STATE_SCANNING, scanConcentratorRoamingContSM_recoveryDuringScan}, /*"FW_RESET"*/
            {CONT_SCAN_STATE_STOPPING, scanConcentratorRoamingContSM_abortScan},          /*"STOP_SCAN"*/
            {CONT_SCAN_STATE_IDLE, scanConcentratorRoamingContSM_scanComplete}            /*"SCAN_COMPLETE"*/
		},

        /* next state and actions for STOPPING state */
		{	
            {CONT_SCAN_STATE_STOPPING, actionUnexpected},                                     /*"START_SCAN",*/
			{CONT_SCAN_STATE_STOPPING, actionUnexpected},                                     /*"SCR_RUN",*/
			{CONT_SCAN_STATE_STOPPING, actionUnexpected},                                     /*"SCR_PEND",  */
			{CONT_SCAN_STATE_STOPPING, actionUnexpected},                                     /*"SCR_REJECT",*/
            {CONT_SCAN_STATE_STOPPING, actionNop},                                        /*"ABORT_SCAN"*/
            {CONT_SCAN_STATE_STOPPING, scanConcentratorRoamingContSM_recoveryDuringScan}, /*"FW_RESET"*/
            {CONT_SCAN_STATE_STOPPING, actionNop},                                        /*"STOP_SCAN"*/
            {CONT_SCAN_STATE_IDLE, scanConcentratorRoamingContSM_scanComplete}            /*"SCAN_COMPLETE"*/
		}
    };

    /* initialize current state */
    pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_CONT ] = CONT_SCAN_STATE_IDLE;

    /* configure the state machine */
	return fsm_Config( pScanConcentrator->clientSM[ SCAN_SCC_ROAMING_CONT ], (fsm_Matrix_t)smMatrix, 
                       CONT_SCAN_NUM_OF_STATES, CONT_SCAN_NUM_OF_EVENTS, 
                       (fsm_eventActivation_t)scanConcentratorRoamingContSM_SMEvent, pScanConcentrator->hOS );
}


#ifdef REPORT_LOG

/* state descriptions, for state machine logging */
static char stateDesc[ CONT_SCAN_NUM_OF_STATES ][ MAX_DESC_STRING_LEN ] = 
{
    "STATE_IDLE",
    "STATE_SCR_REQUEST",
    "STATE_SCR_WAIT",
    "STATE_SCANNING",
    "STATE_STOPPING"
};

/* event descriptions, for state machine logging */
static char eventDesc[ CONT_SCAN_NUM_OF_EVENTS ][ MAX_DESC_STRING_LEN ] = 
{
    "EVENT_START_SCAN",
    "EVENT_SCR_RUN",
    "EVENT_SCR_PEND",
    "EVENT_SCR_REJECT",
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
 * \param currentState - the current continuous scan for roaming SM state.\n
 * \param event - the event to handle.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorRoamingContSM_SMEvent( TI_HANDLE hScanCncn, scan_contSMStates_e* currentState, 
                                                 scan_contSMEvents_e event )
{
    scanConcentrator_t *pScanConcentrator = (scanConcentrator_t *)hScanCncn;
	TI_STATUS status = OK;
	UINT8 nextState;

    /* obtain the next state */
	status = fsm_GetNextState( pScanConcentrator->clientSM[ SCAN_SCC_ROAMING_CONT ], *(UINT8*)currentState, (UINT8)event, &nextState );
	if ( status != OK )
	{
		WLAN_REPORT_SM( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG, ("Failed getting continuous scan for roaming next state.\n") );
		return NOK;
	}

    /* report the move */
    WLAN_REPORT_SM( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG, 
                    ("CONT ROAMING SCAN: <%s, %s> --> %s\n\n",
                    stateDesc[(UINT8)*currentState],
                    eventDesc[(UINT8)event],
                    stateDesc[nextState]) );

    /* move */
    return fsm_Event( pScanConcentrator->clientSM[ SCAN_SCC_ROAMING_CONT ], (UINT8*)currentState, 
                      (UINT8)event, hScanCncn );
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
TI_STATUS scanConcentratorRoamingContSM_requestSCR( TI_HANDLE hScanCncn )
{
    scr_clientRequestStatus_e scrReplyStatus;
	scr_pendReason_e scrPendReason;
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("CONT SM: Requesting SCR.\n") );

    /* request the SCR as continuous roaming client, and act according to return status */
    switch ( scrReplyStatus = scr_clientRequest( pScanConcentrator->hSCR, SCR_CID_CONT_SCAN, &scrPendReason ) )
    {
    case SCR_CRS_PEND:
        WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                                 ("CONT SM: SCR pending, pend Reason: %d.\n", scrPendReason) );
        
        if ( SCR_PR_DIFFERENT_GROUP_RUNNING == scrPendReason )
        {
            /* send a reject event to the SM - will not scan if not in connected group */
			pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_CONT ] = SCAN_CRS_SCAN_FAILED;
			return scanConcentratorRoamingContSM_SMEvent( hScanCncn, 
                                                          (scan_contSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_CONT ]),
                                                          CONT_SCAN_EVENT_SCR_REJECT );
        }
        else
        {
        /* send a pend event to the SM */
        return scanConcentratorRoamingContSM_SMEvent( hScanCncn, 
                                                      (scan_contSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_CONT ]),
                                                      CONT_SCAN_EVENT_SCR_PEND );
        }
/*        break; - unreachable */

    case SCR_CRS_RUN:
        /* send a run event to the SM */
        WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                                 ("CONT SM: SCR acquired.\n") );
        return scanConcentratorRoamingContSM_SMEvent( hScanCncn, 
                                                      (scan_contSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_CONT ]),
                                                      CONT_SCAN_EVENT_SCR_RUN );
/*        break; - unreachable */

    default:
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("CONT SM: SCR returned unrecognized status: %d.\n", scrReplyStatus) );
        /* Notify scan complete to recover from this error */
		pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_CONT ] = SCAN_CRS_SCAN_FAILED;
        scanConcentratorRoamingContSM_SMEvent( hScanCncn,
                                               (scan_contSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_CONT ]),
                                               CONT_SCAN_EVENT_SCAN_COMPLETE );
        return NOK;
/*        break; - unreachable */
    }

 /*   return OK; - unreachable */
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief SM action - handles a SCR run event (by starting the actual scan)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorRoamingContSM_startScan( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;
    TI_STATUS status;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("CONT SM: Sending scan command to scan SRV.\n") );

    /* mark that this scan is currently running */
    pScanConcentrator->currentRunningScanClient = SCAN_SCC_ROAMING_CONT;

    /* register for scan results with the MLME parser */
    if ( OK != mlmeParser_registerForBeaconAndProbeResp( pScanConcentrator->hMlme, 
                                                         scanConcentrator_mlmeResultCB, 
                                                         hScanCncn ) )
    {
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("CONT SM: MLME result registration failed.\n") );

        /* mark the return status */
        pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_CONT ] = SCAN_CRS_SCAN_FAILED;

        /* could not start scan, send a scan complete event */
        scanConcentratorRoamingContSM_SMEvent( hScanCncn, 
                                               (scan_contSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_CONT ]),
                                               CONT_SCAN_EVENT_SCAN_COMPLETE );

        return NOK;
    }

	 /* stop built-in test timer, to avoid TX stuck due to heavy traffic and unknown scan result time originally at scanSrv*/
	healthMonitor_suspendPeriodicTest( pScanConcentrator->hHealthMonitor );

    /* call the scan SRV start scan */
    if ( OK != (status = MacServices_scanSRV_scan( pScanConcentrator->hMacServices, 
                                       &(pScanConcentrator->clientScanParams[ SCAN_SCC_ROAMING_CONT ]),
                                       FALSE,
                                       TRUE,
                                       FALSE,
                                       POWER_SAVE_ON,
                                       TRUE ,
									   NULL,NULL)) )
    {
        WLAN_REPORT_WARNING( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("CONT SM: scan SRV returned status %d, quitting continuous scan.\n", status) );

        /* mark the return status */
        pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_CONT ] = SCAN_CRS_SCAN_FAILED;

	    /* unregister at the MLME for scan result frames */
        mlmeParser_unregisterForBeaconAndProbeResp( pScanConcentrator->hMlme );

        /* could not start scan, send a scan complete event */
        scanConcentratorRoamingContSM_SMEvent( hScanCncn, 
                                               (scan_contSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_CONT ]),
                                               CONT_SCAN_EVENT_SCAN_COMPLETE );
        
        return NOK;
    }

    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief SM action - handles a stop scan or abort scan event (by stopping the actual scan)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorRoamingContSM_abortScan( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;
    BOOLEAN  bSendNullData;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("CONT SM: aborting scan.\n") );
    bSendNullData = (pScanConcentrator->bAbortOrStop == SCAN_CNCN_STOP) ? TRUE : FALSE;

    /* call the scan SRV stop scan */
    MacServices_scanSRV_stopScan( pScanConcentrator->hMacServices, bSendNullData, NULL , NULL  );

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
TI_STATUS scanConcentratorRoamingContSM_recoveryDuringScan( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("CONT SM: Recovery occured!.\n") );
    
    /* reset the scan SRV */
    MacServices_scanSRV_stopOnFWReset( pScanConcentrator->hMacServices );

    /* send a scan complete event to the SM */
    return scanConcentratorRoamingContSM_SMEvent( hScanCncn, 
                                                  (scan_contSMStates_e*)&(pScanConcentrator->clientSMState[ SCAN_SCC_ROAMING_CONT ]),
                                                  CONT_SCAN_EVENT_SCAN_COMPLETE );
}

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief SM action - handles a scan complete event (by releasing the SCR and calling the scan complete CB).
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorRoamingContSM_scanComplete( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("CONT SM: Scan is complete.\n") );

	/* unregister at the MLME for scan result frames */
    mlmeParser_unregisterForBeaconAndProbeResp( pScanConcentrator->hMlme );

    /* mark that this scan is no longer running */
    pScanConcentrator->currentRunningScanClient = SCAN_SCC_NO_CLIENT;

    /* release the SCR */
    scr_clientComplete( pScanConcentrator->hSCR, SCR_CID_CONT_SCAN );

    /* notify the scan complete to the scan mngr */
    if ( FALSE == pScanConcentrator->bInRequest )
    {
        pScanConcentrator->scanResultCB[ SCAN_SCC_ROAMING_CONT ]( pScanConcentrator->scanResultCBObj[ SCAN_SCC_ROAMING_CONT ],
                                                                  pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_CONT ],
                                                                  NULL, pScanConcentrator->SPSScanResult );
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
TI_STATUS scanConcentratorRoamingContSM_scanRejected( TI_HANDLE hScanCncn )
{
    scanConcentrator_t* pScanConcentrator = (scanConcentrator_t*)hScanCncn;

    WLAN_REPORT_INFORMATION( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG,
                             ("CONT SM: Scan is rejected.\n") );

    /* release the SCR */
    scr_clientComplete( pScanConcentrator->hSCR, SCR_CID_CONT_SCAN );

    /* notify the scan complete to the scan mngr */
    if ( FALSE == pScanConcentrator->bInRequest )
    {
        pScanConcentrator->scanResultCB[ SCAN_SCC_ROAMING_CONT ]( pScanConcentrator->scanResultCBObj[ SCAN_SCC_ROAMING_CONT ],
                                                                  pScanConcentrator->scanResult[ SCAN_SCC_ROAMING_CONT ],
                                                                  NULL, pScanConcentrator->SPSScanResult );
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

    WLAN_REPORT_ERROR( pScanConcentrator->hReport, SCAN_CNCN_MODULE_LOG, ("Continuous scan for roaming state machine error, unexpected Event, state=%d\n\n",
																		  pScanConcentrator->clientSMState[SCAN_SCC_ROAMING_CONT]) );
	
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

