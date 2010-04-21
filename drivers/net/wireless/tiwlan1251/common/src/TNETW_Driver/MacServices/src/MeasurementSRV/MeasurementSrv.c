/** \file measurementSrv.c
 *  \brief This file include the measurement SRV interface functions implementation.
 *  \author Ronen Kalish
 *  \date 09-November-2005
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

#include "measurementTypes.h"
#include "MeasurementSrv.h"
#include "MeasurementSrvSM.h"
#include "report.h"
#include "whalCtrl_api.h"
#include "osApi.h"
#include "MacServices.h"
#include "paramIn.h"
#include "public_event_mbox.h"
#include "measurementSrvDbgPrint.h"

/**
 * \author Ronen Kalish\n
 * \date 08-November-2005\n
 * \brief Creates the measurement SRV object
 *
 * Function Scope \e Public.\n
 * \param hOS - handle to the OS object.\n
 * \return a handle to the measurement SRV object, NULL if an error occurred.\n
 */
TI_HANDLE MacServices_measurementSRV_create( TI_HANDLE hOS )
{
    measurementSRV_t* pMeasurementSRV;
    int i;

    /* allocate the measurement SRV object */
    pMeasurementSRV = os_memoryAlloc( hOS, sizeof(measurementSRV_t) );
    if ( NULL == pMeasurementSRV )
    {
        WLAN_OS_REPORT( ("ERROR: Failed to create measurement SRV object.") );
        return NULL;
    }
    
    /* nullify the object */
    os_memoryZero( hOS, pMeasurementSRV, sizeof(measurementSRV_t) );

    /* store OS handle */
    pMeasurementSRV->hOS = hOS;

    /* allocate the SM */
    if ( OK != fsm_Create( hOS, &(pMeasurementSRV->SM), MSR_SRV_NUM_OF_STATES, MSR_SRV_NUM_OF_EVENTS ) )
    {
        pMeasurementSRV->SM = NULL;
        WLAN_OS_REPORT(("Failed to create measurement SRV state machine.\n"));
        MacServices_measurementSRV_destroy( pMeasurementSRV );
        return NULL;
    }

    /* allocate the timers */
    pMeasurementSRV->hStartStopTimer = 
        os_timerCreate( hOS, MacServices_measurementSRV_startStopTimerExpired, (TI_HANDLE)pMeasurementSRV );
    if ( NULL == pMeasurementSRV->hStartStopTimer )
    {
        WLAN_OS_REPORT(("Failed to create measurement SRV FW guard timer.\n"));
        MacServices_measurementSRV_destroy( pMeasurementSRV );
        return NULL;
    }

    for ( i = 0; i < MAX_NUM_OF_MSR_TYPES_IN_PARALLEL; i++ )
    {
        pMeasurementSRV->hRequestTimer[ i ]
            = os_timerCreate( hOS, MacServices_measurementSRV_requestTimerExpired, (TI_HANDLE)pMeasurementSRV );
        if ( NULL == pMeasurementSRV->hRequestTimer[ i ] )
        {
            WLAN_OS_REPORT(("Failed to create measurement SRV request timers.\n"));
            MacServices_measurementSRV_destroy( pMeasurementSRV );
            return NULL;
        }
    }

    return (TI_HANDLE)pMeasurementSRV;
}

/**
 * \author Ronen Kalish\n
 * \date 08-November-2005\n
 * \brief Initializes the measurement SRV object
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSRV - handle to the measurement SRV object.\n
 * \param hReport - handle to the report object.\n
 * \param hHALCtrl - handle to the HAL CTRL object.\n
 * \param hPowerSaveSRV - handle to the power save SRV object.\n
 */
void MacServices_measurementSRV_init( TI_HANDLE hMeasurementSRV, TI_HANDLE hReport, TI_HANDLE hHALCtrl,
                                      TI_HANDLE hPowerSaveSRV )
{
    measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
	int i;

    /* store handles */
    pMeasurementSRV->hReport = hReport;
    pMeasurementSRV->hHalCtrl = hHALCtrl;
    pMeasurementSRV->hPowerSaveSRV = hPowerSaveSRV;

    /* Initialize the state machine */
    measurementSRVSM_init( hMeasurementSRV );

    /* mark that all timers are not running */
    pMeasurementSRV->bStartStopTimerRunning = FALSE;
    for ( i = 0; i < MAX_NUM_OF_MSR_TYPES_IN_PARALLEL; i++ )
    {
        pMeasurementSRV->bRequestTimerRunning[ i ] = FALSE;
    }

    /* register HAL callbacks */
    /* Register and Enable the Measure Start event in HAL */
    whalCtrl_EventMbox_RegisterForEvent( pMeasurementSRV->hHalCtrl, 
                                         HAL_EVENT_MEASUREMENT_START,
                                         (void *)MacServices_measurementSRV_measureStartCB, 
                                         hMeasurementSRV );
    whalCtrl_EventMbox_Enable( pMeasurementSRV->hHalCtrl, HAL_EVENT_MEASUREMENT_START );

    /* Register and Enable the Measurement Complete event in HAL.
    This event will be received when the Measurement duration expired,
    or after Stop Measure command. */
    whalCtrl_EventMbox_RegisterForEvent( pMeasurementSRV->hHalCtrl,
                                         HAL_EVENT_MEASUREMENT_COMPLETE,
                                         (void *)MacServices_measurementSRV_measureCompleteCB,
                                         hMeasurementSRV );
    whalCtrl_EventMbox_Enable( pMeasurementSRV->hHalCtrl, HAL_EVENT_MEASUREMENT_COMPLETE );

    /* Register and Enable the AP Discovery Complete event in HAL */
    whalCtrl_EventMbox_RegisterForEvent( pMeasurementSRV->hHalCtrl, 
                                         HAL_EVENT_AP_DISCOVERY_COMPLETE, 
                                         (void *)MacServices_measurementSRV_apDiscoveryCompleteCB, 
                                         hMeasurementSRV );
    whalCtrl_EventMbox_Enable( pMeasurementSRV->hHalCtrl, HAL_EVENT_AP_DISCOVERY_COMPLETE);


    WLAN_REPORT_INIT( hReport, MEASUREMENT_SRV_MODULE_LOG,  (".....Measurement SRV configured successfully.\n") );
}

/**
 * \brief Restart the measurement SRV object upon recovery.
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSRV - handle to the measurement SRV object.\n
 */
void measurementSRV_restart( TI_HANDLE hMeasurementSRV)
{
    measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
	int i;

	/* if a timer is running, stop it */
	if ( TRUE == pMeasurementSRV->bStartStopTimerRunning )
	{
		os_timerStop( pMeasurementSRV->hOS, pMeasurementSRV->hStartStopTimer );
		pMeasurementSRV->bStartStopTimerRunning = FALSE;
	}
	for ( i = 0; i < MAX_NUM_OF_MSR_TYPES_IN_PARALLEL; i++ )
	{
		if ( TRUE == pMeasurementSRV->bRequestTimerRunning[ i ] )
		{
			os_timerStop( pMeasurementSRV->hOS, pMeasurementSRV->hRequestTimer[ i ] );
			pMeasurementSRV->bRequestTimerRunning[ i ] = FALSE;
		}
	}


    /* Initialize the state machine */
	/* initialize current state */
	pMeasurementSRV->SMState = MSR_SRV_STATE_IDLE;

    /* mark that all timers are not running */
    pMeasurementSRV->bStartStopTimerRunning = FALSE;
    for ( i = 0; i < MAX_NUM_OF_MSR_TYPES_IN_PARALLEL; i++ )
    {
        pMeasurementSRV->bRequestTimerRunning[ i ] = FALSE;
    }

}

/**
 * \author Ronen Kalish\n
 * \date 08-November-2005\n
 * \brief Destroys the measurement SRV object
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSRV - handle to the measurement SRV object.\n
 */
void MacServices_measurementSRV_destroy( TI_HANDLE hMeasurementSRV )
{
    measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
    int i;

    /* sanity cehcking */
    if ( NULL == hMeasurementSRV )
    {
        return;
    }

    /* release state machine */
    if ( NULL != pMeasurementSRV->SM )
    {
        fsm_Unload( pMeasurementSRV->hOS, pMeasurementSRV->SM );
    }

    /* release timers */
    for ( i = 0; i < MAX_NUM_OF_MSR_TYPES_IN_PARALLEL; i++ )
    {
        if ( NULL != pMeasurementSRV->hRequestTimer[ i ] )
        {
            if ( TRUE == pMeasurementSRV->bRequestTimerRunning[ i ] )
            {
                os_timerStop( pMeasurementSRV->hOS, pMeasurementSRV->hRequestTimer[ i ] );
            }
            os_timerDestroy( pMeasurementSRV->hOS, pMeasurementSRV->hRequestTimer[ i ] );
        }
    }
    if ( NULL != pMeasurementSRV->hStartStopTimer )
    {
        if ( TRUE == pMeasurementSRV->bStartStopTimerRunning )
        {
            os_timerStop( pMeasurementSRV->hOS, pMeasurementSRV->hStartStopTimer );
        }
        os_timerDestroy( pMeasurementSRV->hOS, pMeasurementSRV->hStartStopTimer );
    }

    /* release object space */
    os_memoryFree( pMeasurementSRV->hOS, (TI_HANDLE)pMeasurementSRV, sizeof(measurementSRV_t) );
}

/**
 * \author Ronen Kalish\n
 * \date 09-November-2005\n
 * \brief Starts a measurement operation.\n
 *
 * Function Scope \e Public.\n
 * \param hMacServices - handle to the MacServices object.\n
 * \param pMsrRequest - a structure containing measurement parameters.\n
 * \param timeToRequestexpiryMs - the time (in milliseconds) the measurement SRV has to start the request.\n
 * \param cmdResponseCBFunc - callback function to used for command response.\n
 * \param cmdResponseCBObj - handle to pass to command response CB.\n
 * \param cmdCompleteCBFunc - callback function to be used for command complete.\n
 * \param cmdCompleteCBObj - handle to pass to command complete CB.\n
 * \return OK if successful (various, TBD codes if not).\n
 */ 
TI_STATUS MacServices_measurementSRV_startMeasurement( TI_HANDLE hMacServices, 
                                                       measurement_request_t* pMsrRequest,
													   UINT32 timeToRequestExpiryMs,
                                                       CmdResponseCB_t cmdResponseCBFunc,
                                                       TI_HANDLE cmdResponseCBObj,
                                                       measurement_srvCompleteCB_t cmdCompleteCBFunc,
                                                       TI_HANDLE cmdCompleteCBObj )
{
    measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)((MacServices_t*)hMacServices)->hMeasurementSRV;
	int i;

#ifdef TI_DBG
	WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
							 ("%s: Received measurement request.\n", __FUNCTION__) );
	measurementSRVPrintRequest( (TI_HANDLE)pMeasurementSRV, pMsrRequest );
	WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
							 ("time to expiry: %d ms, cmd response CB: 0x%x, cmd response handle: 0x%x\n",
							  timeToRequestExpiryMs,
							  cmdResponseCBFunc,
							  cmdResponseCBObj) );
	WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
							 ("cmd complete CB: 0x%x, cmd complete handle: 0x%x\n",
							  cmdCompleteCBFunc,
							  cmdCompleteCBObj) );
#endif

	/* mark that request is in progress */
    pMeasurementSRV->bInRequest = TRUE;

	/* mark to send NULL data when exiting driver mode (can be changed to FALSE
	   only when explictly stopping the measurement */
	pMeasurementSRV->bSendNullDataWhenExitPs = TRUE;

    /* Nullify return status */
    pMeasurementSRV->returnStatus = OK;

    /* copy request parameters */
    os_memoryCopy( pMeasurementSRV->hOS, (void *)&(pMeasurementSRV->msrRequest), (void *)pMsrRequest, 
                   sizeof(measurement_request_t) );

	/* Mark the current time stamp and the duration to start to cehck expiry later */
	pMeasurementSRV->requestRecptionTimeStampMs = os_timeStampMs( pMeasurementSRV->hOS );
	pMeasurementSRV->timeToRequestExpiryMs = timeToRequestExpiryMs;

	/* copy callbacks */
    pMeasurementSRV->commandResponseCBFunc = cmdResponseCBFunc;
    pMeasurementSRV->commandResponseCBObj = cmdResponseCBObj;
    pMeasurementSRV->measurmentCompleteCBFunc = cmdCompleteCBFunc;
    pMeasurementSRV->measurementCompleteCBObj = cmdCompleteCBObj;

	/* initialize reply */
	pMeasurementSRV->msrReply.numberOfTypes = pMsrRequest->numberOfTypes;
	for ( i = 0; i < pMsrRequest->numberOfTypes; i++ )
	{
		pMeasurementSRV->msrReply.msrTypes[ i ].msrType = pMeasurementSRV->msrRequest.msrTypes[ i ].msrType;
		pMeasurementSRV->msrReply.msrTypes[ i ].status = OK;
	}

	/* nullify the pending CBs bitmap */
	pMeasurementSRV->pendingParamCBs = 0;
    
    /* send a start measurement event to the SM */
    measurementSRVSM_SMEvent( (TI_HANDLE)pMeasurementSRV, &(pMeasurementSRV->SMState), 
                              MSR_SRV_EVENT_MEASURE_START_REQUEST );

    /* mark that request has been sent */
    pMeasurementSRV->bInRequest = FALSE;

    return pMeasurementSRV->returnStatus;
}

/**
 * \author Ronen Kalish\n
 * \date 09-November-2005\n
 * \brief Stops a measurement operation in progress.\n
 *
 * Function Scope \e Public.\n
 * \param hMacServices - handle to the MacServices object.\n
 * \param bSendNullData - whether to send NULL data when exiting driver mode.\n
 * \param cmdResponseCBFunc - callback function to used for command response.\n
 * \param cmdResponseCBObj - handle to pass to command response CB.\n
 * \return OK if successful (various, TBD codes if not).\n
 */
TI_STATUS MacServices_measurementSRV_stopMeasurement( TI_HANDLE hMacServices,
													  BOOLEAN bSendNullData,
                                                      CmdResponseCB_t cmdResponseCBFunc,
                                                      TI_HANDLE cmdResponseCBObj )
{
    measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)((MacServices_t*)hMacServices)->hMeasurementSRV;
    
	WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
							 ("%s: Received measurement stop request.\n", __FUNCTION__) );
	WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
							 ("Send null data:%s, cmd response CB: 0x%x, cmd response handle: 0x%x\n",
							  (TRUE == bSendNullData ? "Yes" : "No"),
							  cmdResponseCBFunc,
							  cmdResponseCBObj) );

	/* store callbacks */
    pMeasurementSRV->commandResponseCBFunc = cmdResponseCBFunc;
    pMeasurementSRV->commandResponseCBObj = cmdResponseCBObj;

	/* store NULL data indication */
	pMeasurementSRV->bSendNullDataWhenExitPs = bSendNullData;

    /* mark that current return status is OK */
    pMeasurementSRV->returnStatus = OK;

	/* mark that a stop request is in progress */
	pMeasurementSRV->bInRequest = TRUE;

    /* send a stop event to the SM */
    measurementSRVSM_SMEvent( (TI_HANDLE)pMeasurementSRV, &(pMeasurementSRV->SMState),
                              MSR_SRV_EVENT_MEASURE_STOP_REQUEST );

	/*mark that stop request has completed */
	pMeasurementSRV->bInRequest = FALSE;

    return pMeasurementSRV->returnStatus;
}

/**
 * \author Ronen Kalish\n
 * \date 09-November-2005\n
 * \brief Notifies the measurement SRV of a FW reset (recovery).\n
 *
 * Function Scope \e Public.\n
 * \param hMacServices - handle to the MacServices object.\n
 */
void MacServices_measurementSRV_FWReset( TI_HANDLE hMacServices )
{
    measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)((MacServices_t*)hMacServices)->hMeasurementSRV;
    int i;

	WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
							 ("%s: Received FW reset indication.\n", __FUNCTION__) );

	/* if a timer is running, stop it */
    if ( TRUE == pMeasurementSRV->bStartStopTimerRunning )
    {
        os_timerStop( pMeasurementSRV->hOS, pMeasurementSRV->hStartStopTimer );
        pMeasurementSRV->bStartStopTimerRunning = FALSE;
    }
    for ( i = 0; i < MAX_NUM_OF_MSR_TYPES_IN_PARALLEL; i++ )
    {
        if ( TRUE == pMeasurementSRV->bRequestTimerRunning[ i ] )
        {
            os_timerStop( pMeasurementSRV->hOS, pMeasurementSRV->hRequestTimer[ i ] );
            pMeasurementSRV->bRequestTimerRunning[ i ] = FALSE;
        }
    }

    /* change SM state to idle */
    pMeasurementSRV->SMState = MSR_SRV_STATE_IDLE;
}

/** 
 * \author Ronen Kalish\n
 * \date 09-November-2005\n
 * \brief callback function used by the power manager to notify driver mode result
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSRV - handle to the measurement SRV object.\n
 * \param PSMode - the power save mode the STA is currently in.\n
 * \param psStatus - the power save request status.\n
 */
void MacServices_measurementSRV_powerSaveCB( TI_HANDLE hMeasurementSRV, UINT8 PSMode, UINT8 psStatus )
{
    measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;

	WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
							 ("%s: Power save SRV CB called. PS mode:%d status: %d\n", 
							  __FUNCTION__, PSMode, psStatus) );

	/* if driver mode entry succeedded */
    if ( ENTER_POWER_SAVE_SUCCESS == psStatus )
    {
        WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
								 ("%s: PS successful.\n", __FUNCTION__) );

        /* send a RIVER_MODE_SUCCESS event */
        measurementSRVSM_SMEvent( (TI_HANDLE)pMeasurementSRV, &(pMeasurementSRV->SMState), 
                                  MSR_SRV_EVENT_DRIVER_MODE_SUCCESS );
    }
    /* driver mode entry failed */
    else
    {
        WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
								 ("%s: PS failed, status %d.\n", __FUNCTION__, psStatus) );

        /* Set the return status to NOK */
        pMeasurementSRV->returnStatus = (TI_STATUS)psStatus;

        /* send a DRIVER_MODE_FAILURE event */
        measurementSRVSM_SMEvent( (TI_HANDLE)pMeasurementSRV, &(pMeasurementSRV->SMState), 
								  MSR_SRV_EVENT_DRIVER_MODE_FAILURE );
    }
}

/** 
 * \author Ronen Kalish\n
 * \date 14-November-2005\n
 * \brief callback function used by the HAL for measure start event (sent when the FW 
 * has started measurement operation, i.e. switched channel and changed RX filters).\n
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSRV - handle to the measurement SRV object.\n
 */
void MacServices_measurementSRV_measureStartCB( TI_HANDLE hMeasurementSRV )
{
    measurementSRV_t *pMeasurementSRV  = (measurementSRV_t*)hMeasurementSRV;

    WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
        ("%s: measure start CB called.\n", __FUNCTION__) ); 
	
	/* stop the FW guard timer */
	os_timerStop( pMeasurementSRV->hOS, pMeasurementSRV->hStartStopTimer );
	pMeasurementSRV->bStartStopTimerRunning = FALSE;

	/* clear the CB function, so that it won't be called on stop as well! */
	pMeasurementSRV->commandResponseCBFunc = NULL;
	pMeasurementSRV->commandResponseCBObj = NULL;

    measurementSRVSM_SMEvent( hMeasurementSRV, &(pMeasurementSRV->SMState), 
							  MSR_SRV_EVENT_START_SUCCESS );
}

/** 
 * \author Ronen Kalish\n
 * \date 14-November-2005\n
 * \brief callback function used by the HAL for measure stop event (sent when the FW 
 * has finished measurement operation, i.e. switched channel to serving channel and changed back RX filters).\n
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSRV - handle to the measurement SRV object.\n
 */
void MacServices_measurementSRV_measureCompleteCB( TI_HANDLE hMeasurementSRV )
{
    measurementSRV_t *pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;

    WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                             ("%s: measure complete CB called.\n", __FUNCTION__));
	
	/* stop the FW guard timer */
	os_timerStop( pMeasurementSRV->hOS, pMeasurementSRV->hStartStopTimer );
	pMeasurementSRV->bStartStopTimerRunning = FALSE;

    measurementSRVSM_SMEvent( hMeasurementSRV, &(pMeasurementSRV->SMState), 
							  MSR_SRV_EVENT_STOP_COMPLETE );
}

/** 
 * \author Ronen Kalish\n
 * \date 14-November-2005\n
 * \brief callback function used by the HAL for AP discovery stop event (sent when the FW 
 * has finished AP discovery operation).\n
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSRV - handle to the measurement SRV object.\n
 */
void MacServices_measurementSRV_apDiscoveryCompleteCB( TI_HANDLE hMeasurementSRV )
{
#ifdef TI_DBG
    measurementSRV_t *pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;

    WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
                             ("%s: AP Discovery complete CB called.\n", __FUNCTION__) ); 
#endif /* TI_DBG */
}

/**
 * \author Ronen Kalish\n
 * \date 14-November-2005\n
 * \brief called when a measurement FW guard timer expires.
 *
 * Function Scope \e Public.\n
 * \param hMeasuremntSRV - handle to the measurement SRV object.\n
 */
void MacServices_measurementSRV_startStopTimerExpired( TI_HANDLE hMeasurementSRV )
{
    measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
    int i;

    WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                       ("%s: FW guard timer expired.\n", __FUNCTION__) );

    /* mark that the FW guard timer is not running */
    pMeasurementSRV->bStartStopTimerRunning = FALSE;

    /* if any other timer is running - stop it */
    for ( i = 0; i < MAX_NUM_OF_MSR_TYPES_IN_PARALLEL; i++ )
    {
        if ( TRUE == pMeasurementSRV->bRequestTimerRunning[ i ] )
        {
            os_timerStop( pMeasurementSRV->hOS, pMeasurementSRV->hRequestTimer[ i ] );
            pMeasurementSRV->bRequestTimerRunning[ i ] = FALSE;
        }
    }

    /* change SM state to idle */
    pMeasurementSRV->SMState = MSR_SRV_STATE_IDLE;

	/*Error Reporting - call the centeral error function in the health monitor if a request for measurement was faield*/
	pMeasurementSRV->failureEventFunc(pMeasurementSRV->failureEventObj ,MEASUREMENT_FAILURE);
}

/**
 * \author Ronen Kalish\n
 * \date 15-November-2005\n
 * \brief called when a measurement type timer expires.\n
 *
 * Function Scope \e Public.\n
 * \param hMeasuremntSRV - handle to the measurement SRV object.\n
 */
void MacServices_measurementSRV_requestTimerExpired( TI_HANDLE hMeasurementSRV )
{
    measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
    int requestIndex;

	/* find the expired measurement type */
    requestIndex = measurementSRVFindMinDuration( hMeasurementSRV );
	if ( -1 == requestIndex )
	{
		/* indicates we can't find the request in the requets array. Shouldn't happen, but nothing to do */
		WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
						   ("%s: Request timer expired and request index from findMinDuration is -1?!?", __FUNCTION__) );
		return;
	}

	WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
							 ("%s: request timer expired, request index: %d.\n", __FUNCTION__, requestIndex) );

	/* mark that the timer is not running and that this request has completed */
    pMeasurementSRV->bRequestTimerRunning[ requestIndex ] = FALSE;

    /* collect results and send stop command if necessary */
    switch (pMeasurementSRV->msrRequest.msrTypes[ requestIndex ].msrType)
    {
    case MSR_TYPE_BEACON_MEASUREMENT:
        measurementSRVHandleBeaconMsrComplete( hMeasurementSRV, requestIndex );
        break;

    case MSR_TYPE_CCA_LOAD_MEASUREMENT:
        measurementSRVHandleChannelLoadComplete( hMeasurementSRV, requestIndex );
        break;

    case MSR_TYPE_NOISE_HISTOGRAM_MEASUREMENT:
        measurementSRVHandleNoiseHistogramComplete( hMeasurementSRV, requestIndex );
        break;

	/* used here to avoid compilation warning only, does nothing */
	case MSR_TYPE_BASIC_MEASUREMENT:
	case MSR_TYPE_FRAME_MEASUREMENT:
	case MSR_TYPE_MAX_NUM_OF_MEASURE_TYPES:
	default:
		WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
						   ("%s: measure type %d not supported for request %d\n", 
							__FUNCTION__, 
							pMeasurementSRV->msrRequest.msrTypes[ requestIndex ].msrType,
							requestIndex) );
		break;
    }

	/* if no measurement are running and no CBs are pending, send ALL TYPES COMPLETE event */
	if ( TRUE == measurementSRVIsMeasurementComplete( hMeasurementSRV ) )
	{
		/* send the event */
		measurementSRVSM_SMEvent( hMeasurementSRV, &(pMeasurementSRV->SMState), 
								  MSR_SRV_EVENT_ALL_TYPES_COMPLETE );
	}
}

/** 
 * \author Ronen Kalish\n
 * \date 13-November-2005\n
 * \brief Checks whether a beacon measurement is part of current measurement request
 *
 * Function Scope \e Private.\n
 * \param hMeasurementSRV - handle to the measurement SRV object.\n
 * \return TRUE if a beacon measurement is part of current request, FALSE otherwise.\n
 */
BOOLEAN measurementSRVIsBeaconMeasureIncluded( TI_HANDLE hMeasurementSRV )
{
    measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
    int i;

    for ( i = 0; i < MAX_NUM_OF_MSR_TYPES_IN_PARALLEL; i++ )
    {
        if ( MSR_TYPE_BEACON_MEASUREMENT == pMeasurementSRV->msrRequest.msrTypes[ i ].msrType )
        {
            return TRUE;
        }
    }
    return FALSE;
}

/** 
 * \author Ronen Kalish\n
 * \date 15-November-2005\n
 * \brief Finds the index for the measurement request with the shortest period 
 * (the one that has now completed).\n
 *
 * Function Scope \e Private.\n
 * \param hMeasurementSRV - handle to the measurement SRV object.\n
 * \return index of the measurement request with the shortest duration.\n
 */
int measurementSRVFindMinDuration( TI_HANDLE hMeasurementSRV )
{
    measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
    int i, minIndex;
	UINT32 minValue;


    minIndex = minValue = 0; /* minIndex is initialized only to avoid compilation warning! */

    /* find the index with the minimum duration */
    for ( i = 0; i < pMeasurementSRV->msrRequest.numberOfTypes; i++ )
    {
        if ( TRUE == pMeasurementSRV->bRequestTimerRunning[ i ] )
        {
			if ( (0 == minValue) ||
				 (pMeasurementSRV->msrRequest.msrTypes[ i ].duration < minValue) )
			{
				minValue = pMeasurementSRV->msrRequest.msrTypes[ i ].duration;
				minIndex = i;
			}
        }
    }

    /* if no entry with positive duration exists, return -1 */
    if ( 0 == minValue )
    {
        return -1;
    }
    else
    { /* otherwise, return the index of the type with the shortest duration */
        return minIndex;
    }
}

/** 
 * \author Ronen Kalish\n
 * \date 15-November-2005\n
 * \brief Handles an AP discovery timer expiry, by setting necessary values in the
 * reply struct.\n
 *
 * Function Scope \e Private.\n
 * \param hMeasurementSRV - handle to the measurement SRV object.\n
 * \param requestIndex - index of the beacon request in the request structure.\n
 */
void measurementSRVHandleBeaconMsrComplete( TI_HANDLE hMeasurementSRV, int requestIndex )
{
	measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
	int status;


	WLAN_REPORT_INFORMATION(pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
			("%s: Sending AP Discovery Stop to the HAL...", __FUNCTION__) );

	/* send stop AP discovery command */
	status = whalCtrl_ApDiscoveryStop( pMeasurementSRV->hHalCtrl );
	if ( OK != status )
	{
		WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
				("%s: status %d received from whalCtrl_ApDiscoveryStop\n", __FUNCTION__, status) );
	}
}

/** 
 * \author Ronen Kalish\n
 * \date 15-November-2005\n
 * \brief Handles a channel load timer expiry, by requesting channel load 
 * results from the FW.\n
 *
 * Function Scope \e Private.\n
 * \param hMeasurementSRV - handle to the measurement SRV object.\n
 * \param requestIndex - index of the channel load request in the request structure.\n
 */
void measurementSRVHandleChannelLoadComplete( TI_HANDLE hMeasurementSRV, int requestIndex )
{
	measurementSRV_t*		pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
	whalParamInfo_t			whalParam;
	TI_STATUS               status;

    /* Getting the Medium Occupancy Register */
	whalParam.paramType = HAL_CTRL_MEDIUM_OCCUPANCY_PARAM;
    whalParam.content.interogateCmdCBParams.CB_Func = (void *)MacServices_measurementSRV_channelLoadParamCB;
    whalParam.content.interogateCmdCBParams.CB_handle = hMeasurementSRV;
    whalParam.content.interogateCmdCBParams.CB_buf = (UINT8*)(&(pMeasurementSRV->mediumOccupancyResults));
	status = whalCtrl_GetParam( pMeasurementSRV->hHalCtrl, &whalParam );

	if ( status != OK )
    {
		WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
						   ("%s: whalCtrl_GetParam returned status %d\n", __FUNCTION__, status) );

		/* mark that the specific measurment type has failed */
		pMeasurementSRV->msrReply.msrTypes[ requestIndex ].status = NOK;

		/* if all measurement types has finished, an event will be send by request timer expired */
    }
    else
    {
		/* mark that channel load param CB is pending */
		pMeasurementSRV->pendingParamCBs |= MSR_SRV_WAITING_CHANNEL_LOAD_RESULTS;
	}
}

/** 
 * \author Ronen Kalish\n
 * \date 15-November-2005\n
 * \brief Handles a noise histogram timer expiry, by requesting noise histogram
 * reaults from the FW.\n
 *
 * Function Scope \e Private.\n
 * \param hMeasurementSRV - handle to the measurement SRV object.\n
 * \param requestIndex - index of the beacon request in the request structure.\n
 */
void measurementSRVHandleNoiseHistogramComplete( TI_HANDLE hMeasurementSRV, int requestIndex )
{
    measurementSRV_t		    *pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
	whalParamInfo_t	            whalParam;
    whalCtrl_noiseHistogram_t   pNoiseHistParams;
	TI_STATUS	                status;
	
    /* Set Noise Histogram Cmd Params */
    pNoiseHistParams.cmd = STOP_NOISE_HIST;
    pNoiseHistParams.sampleInterval = 0;
    os_memoryZero( pMeasurementSRV->hOS, &(pNoiseHistParams.ranges[0]), MEASUREMENT_NOISE_HISTOGRAM_NUM_OF_RANGES );
    
    /* Send a Stop command to the FW */
    status = (TI_STATUS)whalCtrl_NoiseHistogramCmd( pMeasurementSRV->hHalCtrl, &pNoiseHistParams );
    
    if ( OK != status )
	{
		WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
						   ("%s: whalCtrl_NoiseHistogramCmd returned status %d\n", __FUNCTION__, status) );

		/* mark that the specific measurment type has failed */
		pMeasurementSRV->msrReply.msrTypes[ requestIndex ].status = NOK;

		/* if all measurement types has finished, an event will be send by request timer expired */
	}

   	/* Get measurement results */
	whalParam.paramType = HAL_CTRL_NOISE_HISTOGRAM_PARAM;
    whalParam.content.interogateCmdCBParams.CB_Func = (void *)MacServices_measurementSRV_noiseHistCallBack;
    whalParam.content.interogateCmdCBParams.CB_handle = hMeasurementSRV;
    whalParam.content.interogateCmdCBParams.CB_buf = (UINT8*)(&(pMeasurementSRV->noiseHistogramResults));
	status = whalCtrl_GetParam( pMeasurementSRV->hHalCtrl, &whalParam );

    if ( OK == status )
    {
		/* setting On the Waitng for Noise Histogram Results Bit */
		pMeasurementSRV->pendingParamCBs |= MSR_SRV_WAITING_NOISE_HIST_RESULTS;

		WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
								 ("%s: sent noise histogram stop command.\n",
								  __FUNCTION__) );
	}
    else
    {
		WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
						   ("%s: whalCtrl_GetParam returned status %d\n", __FUNCTION__, status) );

		/* mark that the specific measurment type has failed */
		pMeasurementSRV->msrReply.msrTypes[ requestIndex ].status = NOK;

		/* if all measurement types has finished, an event will be send by request timer expired */
    }
}

/** 
 * \author Ronen Kalish\n
 * \date 16-November-2005\n
 * \brief Callback for channel load get param call.\n
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSRV - handle to the measurement SRV object.\n
 * \param status - the get_param call status.\n
 * \param CB_buf - pointer to the results buffer (already on the measurement SRV object)
 */
void MacServices_measurementSRV_channelLoadParamCB( TI_HANDLE hMeasurementSRV, TI_STATUS status, 
													UINT8* CB_buf )
{
    measurementSRV_t	    *pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
    UINT32                  mediumUsageInMs, periodInMs;
	int 					requestIndex;

	/* when this CB is called as a result of the nulify call at the measurement beginning,
	   the handle will be NULL. In this case, nothing needs to be done. */
	if ( NULL == hMeasurementSRV )
	{
		return;
	}

	WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
							 ("%s: Channel load CB called, status:%d\n", __FUNCTION__, status) );
	WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
							 ("result address (reported): 0x%x, result address (assumed): 0x%x, results (reported):\n",
							  CB_buf, &(pMeasurementSRV->mediumOccupancyResults)) );
	WLAN_REPORT_HEX_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, CB_buf, sizeof(mediumOccupancy_t) );

	/* setting Off the Waitng for Channel Load Results Bit */
	pMeasurementSRV->pendingParamCBs &= ~MSR_SRV_WAITING_CHANNEL_LOAD_RESULTS;

	/* find the request index */
	requestIndex = measurementSRVFindIndexByType( hMeasurementSRV, MSR_TYPE_CCA_LOAD_MEASUREMENT );
	if ( -1 == requestIndex )
	{
		/* indicates we can't find the request in the requets array. Shouldn't happen, but nothing to do */
		WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
						   ("%s: request index from measurementSRVFindIndexByType is -1?!?", __FUNCTION__) );
		return;
	}

	if ( (OK == status) && (0 != pMeasurementSRV->mediumOccupancyResults.Period) )
	{		
		/* calculate results */
		mediumUsageInMs = pMeasurementSRV->mediumOccupancyResults.MediumUsage / 1000;
		periodInMs      = pMeasurementSRV->mediumOccupancyResults.Period / 1000;

		WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
								 ("%s: MediumUsage = %d Period = %d\n", 
								  __FUNCTION__,mediumUsageInMs, periodInMs) );
		
		if ( periodInMs <= pMeasurementSRV->msrRequest.msrTypes[ requestIndex ].duration )
		{
			pMeasurementSRV->msrReply.msrTypes[ requestIndex ].replyValue.CCABusyFraction = 
				( 255 * pMeasurementSRV->mediumOccupancyResults.MediumUsage ) / 
					pMeasurementSRV->mediumOccupancyResults.Period;
		}
		else
		{       
			pMeasurementSRV->msrReply.msrTypes[ requestIndex ].replyValue.CCABusyFraction = 
				( 255 * pMeasurementSRV->mediumOccupancyResults.MediumUsage ) / 
					(pMeasurementSRV->msrRequest.msrTypes[ requestIndex ].duration * 1000);
		}		
	}
	else
	{
		WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
						   ("%s: channel load failed. Status=%d, period=%d\n",
							__FUNCTION__,
							status,
							pMeasurementSRV->mediumOccupancyResults.Period) );

		/* mark result status */
		pMeasurementSRV->msrReply.msrTypes[ requestIndex ].status = NOK;
	}
	
	/* if no measurement are running and no CBs are pending, 
	   send ALL TYPES COMPLETE event */
	if ( TRUE == measurementSRVIsMeasurementComplete( hMeasurementSRV ) )
	{
		/* send the event */
		measurementSRVSM_SMEvent( hMeasurementSRV, &(pMeasurementSRV->SMState), 
								  MSR_SRV_EVENT_ALL_TYPES_COMPLETE );
	}
}

/** 
 * \date 03-January-2005\n
 * \brief Dummy callback for channel load get param call. Used to clear the channel load tracker.\n
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSRV - handle to the measurement SRV object.\n
 * \param status - the get_param call status.\n
 * \param CB_buf - pointer to the results buffer (already on the measurement SRV object)
 */
void MacServices_measurementSRV_dummyChannelLoadParamCB( TI_HANDLE hMeasurementSRV, TI_STATUS status, 
													UINT8* CB_buf )
{
#ifdef TI_DBG
    measurementSRV_t *pMeasurementSRV = (measurementSRV_t*) hMeasurementSRV;

	WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
			("%s: Dummy Channel Load callback called (status = %d)\n", __FUNCTION__, status));
#endif /* TI_DBG */
}

/** 
 * \author Ronen Kalish\n
 * \date 16-November-2005\n
 * \brief Callback for noise histogram get param call.\n
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSRV - handle to the measurement SRV object.\n
 * \param status - the get_param call status.\n
 * \param CB_buf - pointer to the results buffer (already on the measurement SRV object)
 */
void MacServices_measurementSRV_noiseHistCallBack( TI_HANDLE hMeasurementSRV, TI_STATUS status, 
												   UINT8* CB_buf )
{
    measurementSRV_t		    *pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
	UINT8		                index;
    UINT32                      sumOfSamples;
    int							requestIndex;

	WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
   							 ("%s: noise histogram CB called, status: %d\n", __FUNCTION__, status) );
	WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
							 ("result address (reported): 0x%x, result address (assumed): 0x%x, results (reported):\n",
							  CB_buf, &(pMeasurementSRV->noiseHistogramResults)) );
	WLAN_REPORT_HEX_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, CB_buf, sizeof(noiseHistogramResults_t) );

	/* setting Off the Waitng for noise histogram Results Bit */
	pMeasurementSRV->pendingParamCBs &= ~MSR_SRV_WAITING_NOISE_HIST_RESULTS;

	/* find the request index */
	requestIndex = measurementSRVFindIndexByType( hMeasurementSRV, MSR_TYPE_NOISE_HISTOGRAM_MEASUREMENT );
	if ( -1 == requestIndex )
	{
		/* indicates we can't find the request in the requets array. Shouldn't happen, but nothing to do */
		WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
						   ("%s: request index from measurementSRVFindIndexByType is -1?!?", __FUNCTION__) );
		return;
	}

	if ( OK == status )
	{
		sumOfSamples = pMeasurementSRV->noiseHistogramResults.numOfLostCycles;
		
		/* Print For Debug */
		WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
			("%s: numOfLostCycles = %d "
			 "numOfTxHwGenLostCycles = %d numOfRxLostCycles = %d numOfExceedLastThresholdLostCycles = %d\n", 
			__FUNCTION__,
			pMeasurementSRV->noiseHistogramResults.numOfLostCycles, 
			pMeasurementSRV->noiseHistogramResults.numOfTxHwGenLostCycles,
			pMeasurementSRV->noiseHistogramResults.numOfRxLostCycles,
			pMeasurementSRV->noiseHistogramResults.numOfLostCycles - 
			 (pMeasurementSRV->noiseHistogramResults.numOfTxHwGenLostCycles + 
			  pMeasurementSRV->noiseHistogramResults.numOfRxLostCycles)) );
		
		for ( index = 0; index < NUM_OF_NOISE_HISTOGRAM_COUNTERS; index++ )
		{
			sumOfSamples += pMeasurementSRV->noiseHistogramResults.counters[ index ];
			
			/* Print For Debug */
			WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
				("%s: Counter #%d = %x\n", 
				__FUNCTION__, index, pMeasurementSRV->noiseHistogramResults.counters[index]));
		}
		
		/* If there weren't enough samples --> Reject the Request */
		if ( (sumOfSamples - pMeasurementSRV->noiseHistogramResults.numOfLostCycles) < 
				NOISE_HISTOGRAM_THRESHOLD )
		{
			WLAN_REPORT_WARNING( pMeasurementSRV->hReport,  MEASUREMENT_SRV_MODULE_LOG,
								 ("%s: noise histogram CB, rejecting request because %d samples received.\n",
								  __FUNCTION__,
								  sumOfSamples - pMeasurementSRV->noiseHistogramResults.numOfLostCycles) );

			/* set negative result status */
			pMeasurementSRV->msrReply.msrTypes[ requestIndex ].status = NOK;
		}
		else
		{   
 			for (index = 0; index < NUM_OF_NOISE_HISTOGRAM_COUNTERS; index++)
			{
				pMeasurementSRV->msrReply.msrTypes[ requestIndex ].replyValue.RPIDensity[ index ] = 
					( 255 * pMeasurementSRV->noiseHistogramResults.counters[ index ]) / sumOfSamples;
			}

			WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
									 ("%s: Valid noise histogram reply. RPIDensity: %d %d %d %d %d %d %d %d\n",
									  __FUNCTION__,
									  pMeasurementSRV->msrReply.msrTypes[ requestIndex ].replyValue.RPIDensity[ 0 ],
									  pMeasurementSRV->msrReply.msrTypes[ requestIndex ].replyValue.RPIDensity[ 1 ],
									  pMeasurementSRV->msrReply.msrTypes[ requestIndex ].replyValue.RPIDensity[ 2 ],
									  pMeasurementSRV->msrReply.msrTypes[ requestIndex ].replyValue.RPIDensity[ 3 ],
									  pMeasurementSRV->msrReply.msrTypes[ requestIndex ].replyValue.RPIDensity[ 4 ],
									  pMeasurementSRV->msrReply.msrTypes[ requestIndex ].replyValue.RPIDensity[ 5 ],
									  pMeasurementSRV->msrReply.msrTypes[ requestIndex ].replyValue.RPIDensity[ 6 ],
									  pMeasurementSRV->msrReply.msrTypes[ requestIndex ].replyValue.RPIDensity[ 7 ]) );
		}
	}
	else
	{
		WLAN_REPORT_WARNING( pMeasurementSRV->hReport,  MEASUREMENT_SRV_MODULE_LOG,
							 ("%s: noise histogram CB with status: %d, rejecting request.\n",
							  __FUNCTION__, status) );
		/* set negative result status */
		pMeasurementSRV->msrReply.msrTypes[ requestIndex ].status = NOK;
	}
	
	/* if no measurement are running and no CBs are pending, 
	   send ALL TYPES COMPLETE event */
	if ( TRUE == measurementSRVIsMeasurementComplete( hMeasurementSRV ) )
	{
		/* send the event */
		measurementSRVSM_SMEvent( hMeasurementSRV, &(pMeasurementSRV->SMState), 
								  MSR_SRV_EVENT_ALL_TYPES_COMPLETE );
	}
}

/** 
 * \author Ronen Kalish\n
 * \date 16-November-2005\n
 * \brief Checks whether all measuremtn types had completed and all param CBs had been called.\n
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSRV - handle to the measurement SRV object.\n
 * \param status - the get_param call status.\n
 * \param CB_buf - pointer to the results buffer (already on the measurement SRV object)
 */
BOOLEAN measurementSRVIsMeasurementComplete( TI_HANDLE hMeasurementSRV )
{
	measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
	int i;

	/* verify that no request is currently running */
	for ( i = 0; i < pMeasurementSRV->msrRequest.numberOfTypes; i++ )
	{
		if ( TRUE == pMeasurementSRV->bRequestTimerRunning[ i ] )
		{
			return FALSE;
		}
	}

	/* verify that no CBs are pending */
	if ( 0 != (pMeasurementSRV->pendingParamCBs & 
			   (MSR_SRV_WAITING_CHANNEL_LOAD_RESULTS | MSR_SRV_WAITING_NOISE_HIST_RESULTS)) )
	{
		return FALSE;
	}

	return TRUE;
}

/** 
 * \author Ronen Kalish\n
 * \date 17-November-2005\n
 * \brief Finds a measure type index in the measure request array.\n
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSRV - handle to the measurement SRV object.\n
 * \param type - the measure type to look for.\n
 * \return the type index, -1 if not found.\n
 */
int measurementSRVFindIndexByType( TI_HANDLE hMeasurementSRV, measurement_type_e type )
{
	measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
	int i;

	for ( i = 0; i < pMeasurementSRV->msrRequest.numberOfTypes; i++ )
	{
		if ( type == pMeasurementSRV->msrRequest.msrTypes[ i ].msrType )
		{
			return i;
		}
	}
	return -1;
}



/****************************************************************************************
 *                        measurementSRVRegisterFailureEventCB													*
 ****************************************************************************************
DESCRIPTION: Registers a failure event callback for scan error notifications.
			    
				                                                                                                   
INPUT:     	- hMeasurementSRV	- handle to the Measurement SRV object.		
			- failureEventCB 		- the failure event callback function.\n
			- hFailureEventObj 	- handle to the object passed to the failure event callback function.

OUTPUT:	
RETURN:    void.
****************************************************************************************/

void measurementSRVRegisterFailureEventCB( TI_HANDLE hMeasurementSRV, 
                                     void * failureEventCB, TI_HANDLE hFailureEventObj )
{
    measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;

    pMeasurementSRV->failureEventFunc	= (failureEventCB_t)failureEventCB;
    pMeasurementSRV->failureEventObj	= hFailureEventObj;
}


