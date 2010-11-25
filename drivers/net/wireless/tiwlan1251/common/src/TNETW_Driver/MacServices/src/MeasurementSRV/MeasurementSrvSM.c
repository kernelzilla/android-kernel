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

/** \file measurementSrv.c
 *  \brief This file include the measurement SRV state machine implementation.
 *  \author Ronen Kalish
 *  \date 13-November-2005
 */

#include "MeasurementSrvSM.h"
#include "MeasurementSrv.h"
#include "PowerSrv_API.h"
#include "osApi.h"
#include "report.h"
#include "commonTypes.h"
#include "fsm.h"
#include "tnetwCommon.h"
#include "paramIn.h"
#include "public_commands.h"

TI_STATUS actionUnexpected( TI_HANDLE hMeasurementSrv );
TI_STATUS actionNop( TI_HANDLE hMeasurementSrv );

/**
 * \author Ronen Kalish\n
 * \date 08-November-2005\n
 * \brief Initialize the measurement SRV SM.\n
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSrv - handle to the Measurement SRV object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS measurementSRVSM_init( TI_HANDLE hMeasurementSRV )
{
   measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;

    fsm_actionCell_t    smMatrix[ MSR_SRV_NUM_OF_STATES ][ MSR_SRV_NUM_OF_EVENTS ] =
    {
        /* next state and actions for IDLE state */
        {   
            {MSR_SRV_STATE_WAIT_FOR_DRIVER_MODE, measurementSRVSM_requestDriverMode},     /*"MESSURE_START_REQUEST"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"DRIVER_MODE_SUCCESS"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"DRIVER_MODE_FAILURE"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"START_SUCCESS"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"START_FAILURE"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"ALL_TYPES_COMPLETE"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"STOP_COMPLETE"*/
            {MSR_SRV_STATE_IDLE, measurementSRVSRVSM_dummyStop}                           /*"MEASURE_STOP_REQUEST"*/
        },


        /* next state and actions for WAIT_FOR_DRIVER_MODE state */
        {   
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"MESSURE_START_REQUEST"*/
            {MSR_SRV_STATE_WAIT_FOR_MEASURE_START, measurementSRVSM_requestMeasureStart}, /*"DRIVER_MODE_SUCCESS"*/
            {MSR_SRV_STATE_IDLE, measurementSRVSM_DriverModeFailure},                     /*"DRIVER_MODE_FAILURE"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"START_SUCCESS"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"START_FAILURE"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"ALL_TYPES_COMPLETE"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"STOP_COMPLETE"*/
            {MSR_SRV_STATE_IDLE, measurementSRVSM_stopFromWaitForDriverMode}              /*"MEASURE_STOP_REQUEST"*/
        },

        /* next state and actions for WAIT_FOR_MEASURE_START state */
        {    
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"MESSURE_START_REQUEST"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"DRIVER_MODE_SUCCESS"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"DRIVER_MODE_FAILURE"*/
            {MSR_SRV_STATE_MEASURE_IN_PROGRESS, measurementSRVSM_startMeasureTypes},      /*"START_SUCCESS"*/
            {MSR_SRV_STATE_IDLE, measurementSRVSM_measureStartFailure},                   /*"START_FAILURE"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"ALL_TYPES_COMPLETE"*/
            {MSR_SRV_STATE_IDLE, measurementSRVSM_completeMeasure},                       /*"STOP_COMPLETE"*/
            {MSR_SRV_STATE_WAIT_FOR_MEASURE_STOP, measurementSRVSM_stopFromWaitForMeasureStart}
                                                                                          /*"MEASURE_STOP_REQUEST"*/
        },

        /* next state and actions for MEASURE_IN_PROGRESS state */
        {   
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"MESSURE_START_REQUEST"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"DRIVER_MODE_SUCCESS"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"DRIVER_MODE_FAILURE"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"START_SUCCESS"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"START_FAILURE"*/
            {MSR_SRV_STATE_WAIT_FOR_MEASURE_STOP, measurementSRVSM_requestMeasureStop},   /*"ALL_TYPES_COMPLETE"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"STOP_COMPLETE"*/
            {MSR_SRV_STATE_WAIT_FOR_MEASURE_STOP, measurementSRVSM_stopFromMeasureInProgress}
                                                                                          /*"MEASURE_STOP_REQUEST"*/
        },

        /* next state and actions for WAIT_FOR_MEASURE_STOP state */
        {   
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"MESSURE_START_REQUEST"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"DRIVER_MODE_SUCCESS"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"DRIVER_MODE_FAILURE"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"START_SUCCESS"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"START_FAILURE"*/
            {MSR_SRV_STATE_IDLE, actionUnexpected},                                       /*"ALL_TYPES_COMPLETE"*/
            {MSR_SRV_STATE_IDLE, measurementSRVSM_completeMeasure},                       /*"STOP_COMPLETE"*/
            {MSR_SRV_STATE_WAIT_FOR_MEASURE_STOP, measurementSRVSRVSM_dummyStop}          /*"MEASURE_STOP_REQUEST"*/
        }
    };

    /* initialize current state */
    pMeasurementSRV->SMState = MSR_SRV_STATE_IDLE;

    /* configure the state machine */
    return fsm_Config( pMeasurementSRV->SM, (fsm_Matrix_t)smMatrix, 
                       (UINT8)MSR_SRV_NUM_OF_STATES, (UINT8)MSR_SRV_NUM_OF_EVENTS, 
                       (fsm_eventActivation_t)measurementSRVSM_SMEvent, pMeasurementSRV->hOS );
}

#ifdef TI_DBG
/* state descriptions, for state machine logging */
static char stateDesc[ MSR_SRV_NUM_OF_STATES ][ MAX_DESC_STRING_LEN ] = 
{
    "STATE_IDLE\0",
    "STATE_WAIT_FOR_DRIVER_MODE\0",
    "STATE_WAIT_FOR_MEASURE_START\0",
    "STATE_MEASURE_IN_PROGRESS\0",
    "STATE_WAIT_FOR_MEASURE_STOP\0"
};

/* event descriptions, for state machine logging */
static char eventDesc[ MSR_SRV_NUM_OF_EVENTS ][ MAX_DESC_STRING_LEN ] = 
{
    "EVENT_MEASURE_START_REQUEST\0",
    "EVENT_DRIVER_MODE_SUCCESS\0",
    "EVENT_DRIVER_MODE_FAILURE\0",
    "EVENT_START_SUCCESS\0",
    "EVENT_START_FAILURE\0",
    "EVENT_ALL_TYPES_COMPLETE\0",
    "EVENT_STOP_COMPLETE\0",
    "EVENT_STOP_REQUEST\0",
};
#endif /* TI_DBG */

/**
 * \author Ronen Kalish\n
 * \date 08-November-2005\n
 * \brief Processes an event.\n
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSrv - handle to the measurement SRV object.\n
 * \param currentState - the current scan SRV SM state.\n
 * \param event - the event to handle.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS measurementSRVSM_SMEvent( TI_HANDLE hMeasurementSrv, measurements_SRVSMStates_e* currentState, 
                                    measurement_SRVSMEvents_e event )
{
    measurementSRV_t *pMeasurementSRV = (measurementSRV_t *)hMeasurementSrv;
    TI_STATUS status = OK;
    UINT8 nextState;

    /* obtain the next state */
    status = fsm_GetNextState( pMeasurementSRV->SM, (UINT8)*currentState, (UINT8)event, &nextState );
    if ( status != OK )
    {
        WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
                ("%s: State machine error, failed getting next state\n", __FUNCTION__) );
        return NOK;
    }

#ifdef TI_DBG
    /* report the move */
    WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
            ("%s: <%s, %s> --> %s\n", __FUNCTION__,
                    stateDesc[(UINT8)*currentState],
                    eventDesc[(UINT8)event],
                    stateDesc[nextState]) );
#endif

    /* move */
    return fsm_Event( pMeasurementSRV->SM, (UINT8*)currentState, (UINT8)event, hMeasurementSrv );
}

/**
 * \author Ronen Kalish\n
 * \date 08-November-2005\n
 * \brief Handle a MEASURE_START_REQUEST event by requesting driver mode.\n
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSrv - handle to the Measurement SRV object.\n
 * \return always OK.\n
 */
TI_STATUS measurementSRVSM_requestDriverMode( TI_HANDLE hMeasurementSRV )
{
    measurementSRV_t    *pMeasurementSRV  = (measurementSRV_t*)hMeasurementSRV;
    TI_STATUS           PSStatus;
    whalParamInfo_t     paramInfo;

    /* get the current channel */
    paramInfo.paramType = HAL_CTRL_CURRENT_CHANNEL;
    whalCtrl_GetParam( pMeasurementSRV->hHalCtrl, &paramInfo );
    
    /* check if the request is on the serving channel */
    if ( paramInfo.content.halCtrlCurrentChannel == pMeasurementSRV->msrRequest.channel )
    {
        /* Switch Power Save SRV to driver mode w/o changing power save mode*/
        PSStatus = powerSrv_ReservePS( pMeasurementSRV->hPowerSaveSRV, POWER_SAVE_KEEP_CURRENT,
                                       TRUE, hMeasurementSRV, MacServices_measurementSRV_powerSaveCB );
    }
    else
    {
        /* Switch Power Save SRV to driver mode with PS mode */      
        PSStatus = powerSrv_ReservePS( pMeasurementSRV->hPowerSaveSRV, POWER_SAVE_ON,
                                       TRUE, hMeasurementSRV, MacServices_measurementSRV_powerSaveCB );
    }

    switch (PSStatus)
    {
        case POWER_SAVE_802_11_IS_CURRENT:
            WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
                                     ("%s: Driver mode entered successfully\n",__FUNCTION__) );
            /* send a power save success event */
            return measurementSRVSM_SMEvent( hMeasurementSRV, &(pMeasurementSRV->SMState),
                                             MSR_SRV_EVENT_DRIVER_MODE_SUCCESS );
/*            break; */
    
        case PENDING:
        case OK:
            WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                                     ("%s: Driver mode pending\n", __FUNCTION__) );
            break;
        
        default: /* Error */
            WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
                               ("%s: Error %d when requesting driver mode\n",__FUNCTION__,PSStatus) );

            /* Set the return status to NOK */
            pMeasurementSRV->returnStatus = PSStatus;

            /* send a power save failure event */
            measurementSRVSM_SMEvent( hMeasurementSRV, &(pMeasurementSRV->SMState),
                                      MSR_SRV_EVENT_DRIVER_MODE_FAILURE );
            break;
    }
    
    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 08-November-2005\n
 * \brief Handle a DRIVER_MODE_SUCCESS event by sending start measure command to the FW.\n
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSrv - handle to the Measurement SRV object.\n
 * \return always OK.\n
 */
TI_STATUS measurementSRVSM_requestMeasureStart( TI_HANDLE hMeasurementSRV )
{
    measurementSRV_t                    *pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
    whalCtrl_MeasurementParameters_t    pMeasurementCmd;
    TI_STATUS                           status;
    UINT32                              currentTime = os_timeStampMs( pMeasurementSRV->hOS );

    /* check if request time has expired (note: timer wrap-around is also handled)*/
    if ( (pMeasurementSRV->requestRecptionTimeStampMs + pMeasurementSRV->timeToRequestExpiryMs)
                    < currentTime )
    {
        int i;

        WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                           ("%s: request time has expired, request expiry time:%d, current time:%d\n",
                            __FUNCTION__,
                            pMeasurementSRV->requestRecptionTimeStampMs + pMeasurementSRV->timeToRequestExpiryMs, 
                            currentTime) );

        /* mark that all measurement types has failed */
        for ( i = 0; i < pMeasurementSRV->msrRequest.numberOfTypes; i++ )
        {
            pMeasurementSRV->msrReply.msrTypes[ i ].status = NOK;
        }

        /* send a measurement complete event */
        measurementSRVSM_SMEvent( hMeasurementSRV, &(pMeasurementSRV->SMState),
                                  MSR_SRV_EVENT_STOP_COMPLETE );
        
        return OK;
    }

    pMeasurementCmd.channel = pMeasurementSRV->msrRequest.channel;
    pMeasurementCmd.band = pMeasurementSRV->msrRequest.band;
    pMeasurementCmd.duration = 0; /* Infinite */

    if ( measurementSRVIsBeaconMeasureIncluded( hMeasurementSRV ) )
    {  /* Beacon Measurement is one of the types */

		/* get the current channel */
		whalParamInfo_t    					paramInfo;

		paramInfo.paramType = HAL_CTRL_CURRENT_CHANNEL;
		whalCtrl_GetParam( pMeasurementSRV->hHalCtrl, &paramInfo );

		pMeasurementCmd.ConfigOptions = RX_CONFIG_OPTION_FOR_MEASUREMENT; 

		/* check if the request is on the serving channel */
		if ( paramInfo.content.halCtrlCurrentChannel == pMeasurementSRV->msrRequest.channel )
		{
			/* Set the RX Filter to the join one, so that any packets will 
            be received on the serving channel - beacons and probe requests for
			the measurmenet, and also data (for normal operation) */
            pMeasurementCmd.FilterOptions = RX_FILTER_OPTION_JOIN;
		}
		else
		{
			/* not on the serving channle - only beacons and rpobe responses are required */
			pMeasurementCmd.FilterOptions = RX_FILTER_OPTION_DEF_PRSP_BCN; 
		}
    }
    else
    {  /* No beacon measurement - use the current RX Filter */
        pMeasurementCmd.ConfigOptions = 0xffffffff;
        pMeasurementCmd.FilterOptions = 0xffffffff;
    }

    /* Send start measurement command */
    status = (TI_STATUS)whalCtrl_measurementCmd( pMeasurementSRV->hHalCtrl, &pMeasurementCmd,
                                      (void *)measurementSRVSM_requestMeasureStartResponseCB, 
                                      pMeasurementSRV );
    
    if ( OK != status )
    {
        WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                           ("%s: Failed to send measurement start command, statud=%d,\n",
                            __FUNCTION__, status) );

        /* keep the faulty return status */
        pMeasurementSRV->returnStatus = status;

        /* send a measurement start fail event */
        return measurementSRVSM_SMEvent( hMeasurementSRV, &(pMeasurementSRV->SMState),
                                         MSR_SRV_EVENT_START_FAILURE );
    }

    WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
                             ("%s: measure start command sent. Params:\n channel=%d, band=%d, duration=%d, \n configOptions=0x%x, filterOptions=0x%x, status=%d, \n",
                             __FUNCTION__,
                             pMeasurementCmd.channel,
                             pMeasurementCmd.band,
                             pMeasurementCmd.duration,
                             pMeasurementCmd.ConfigOptions,
                             pMeasurementCmd.FilterOptions,
                             status) );

    /* start the FW guard timer */
    pMeasurementSRV->bStartStopTimerRunning = TRUE;
    os_timerStart( pMeasurementSRV->hOS, pMeasurementSRV->hStartStopTimer, MSR_FW_GUARD_TIME, FALSE );
  
    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 08-November-2005\n
 * \brief Handle a START_SUCCESS event by starting different measure types and setting timers.\n
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSrv - handle to the Measurement SRV object.\n
 * \return always OK.\n
 */
TI_STATUS measurementSRVSM_startMeasureTypes( TI_HANDLE hMeasurementSRV )
{
    measurementSRV_t                    *pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
    UINT8                               requestIndex, rangeIndex;
    INT8                                rangeUpperBound;
    whalParamInfo_t                     whalParam;
    TI_STATUS                           status;
    whalCtrl_noiseHistogram_t           pNoiseHistParams;
    whalCtrl_ApDiscoveryParameters_t    pApDiscoveryParams;
    UINT32                              currentTime = os_timeStampMs( pMeasurementSRV->hOS );

    /* check if request time has expired (note: timer wrap-around is also handled)*/
    if ( (pMeasurementSRV->requestRecptionTimeStampMs + pMeasurementSRV->timeToRequestExpiryMs)
                    < currentTime )
    {
        int i;

        WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                           ("%s: request time has expired, request expiry time:%d, current time:%d\n",
                            __FUNCTION__,
                            pMeasurementSRV->requestRecptionTimeStampMs + pMeasurementSRV->timeToRequestExpiryMs, 
                            currentTime) );

        /* mark that all measurement types has failed */
        for ( i = 0; i < pMeasurementSRV->msrRequest.numberOfTypes; i++ )
        {
            pMeasurementSRV->msrReply.msrTypes[ i ].status = MSR_REJECT_MAX_DELAY_PASSED;
        }

        /* send a measurement complete event */
        measurementSRVSM_SMEvent( hMeasurementSRV, &(pMeasurementSRV->SMState),
                                  MSR_SRV_EVENT_ALL_TYPES_COMPLETE );
        
        return OK;
    }

    /* Going over all request types that should be executed in parallel 
    to start their timers and execute the measurement */
    for ( requestIndex = 0; requestIndex < pMeasurementSRV->msrRequest.numberOfTypes ; requestIndex++ )
    {
        switch (pMeasurementSRV->msrRequest.msrTypes[ requestIndex ].msrType)
        {
        case MSR_TYPE_CCA_LOAD_MEASUREMENT:    
            /* Clearing the Medium Occupancy Register */
            whalParam.paramType = HAL_CTRL_MEDIUM_OCCUPANCY_PARAM;
            whalParam.content.interogateCmdCBParams.CB_Func = (void *)MacServices_measurementSRV_dummyChannelLoadParamCB;
            whalParam.content.interogateCmdCBParams.CB_handle = hMeasurementSRV;
            whalParam.content.interogateCmdCBParams.CB_buf = 
                    (UINT8*)(&(pMeasurementSRV->mediumOccupancyResults));
            status = whalCtrl_GetParam( pMeasurementSRV->hHalCtrl, &whalParam );
            if( OK == status  )
            {
                WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
                                         ("%s: Medium Usage has been nullified, starting timer.\n", __FUNCTION__) );

                /* Start Timer */
                os_timerStart( pMeasurementSRV->hOS, pMeasurementSRV->hRequestTimer[ requestIndex ],
                               pMeasurementSRV->msrRequest.msrTypes[ requestIndex ].duration, FALSE );
                pMeasurementSRV->bRequestTimerRunning[ requestIndex ] = TRUE;
            }
            else
            {
                WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                                   ("%s: whalCtrl_GetParam (for channel load) returned status %d\n", __FUNCTION__, status) );
            }

            break;
        
        case MSR_TYPE_NOISE_HISTOGRAM_MEASUREMENT:
            /* Set Noise Histogram Cmd Params */
            pNoiseHistParams.cmd = START_NOISE_HIST;
            pNoiseHistParams.sampleInterval = DEF_SAMPLE_INTERVAL;
            os_memoryZero( pMeasurementSRV->hOS, &(pNoiseHistParams.ranges[0]), MEASUREMENT_NOISE_HISTOGRAM_NUM_OF_RANGES );
        
            /* Set Ranges */
            /* (-87) - First Range's Upper Bound */
            rangeUpperBound = -87;

            /* Previously we converted from RxLevel to dBm - now this isn't necessary */
            /* rangeUpperBound = whalCtrl_convertRSSIToRxLevel( pMeasurementSRV->hHalCtrl, -87); */

            for(rangeIndex = 0; rangeIndex < MEASUREMENT_NOISE_HISTOGRAM_NUM_OF_RANGES -1; rangeIndex++)
            {
                if(rangeUpperBound > 0)
                {
                    pNoiseHistParams.ranges[rangeIndex] = 0;
                }
                else
                {               
                    pNoiseHistParams.ranges[rangeIndex] = rangeUpperBound;
                }         
                rangeUpperBound += 5; 
            }
            pNoiseHistParams.ranges[rangeIndex] = 0xFE;

            /* Print for Debug */
            WLAN_REPORT_INFORMATION(pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
                    ("%s:Noise histogram Measurement Ranges:\n%d %d %d %d %d %d %d %d\n", __FUNCTION__,
                            (INT8) pNoiseHistParams.ranges[0], (INT8) pNoiseHistParams.ranges[1],
                            (INT8) pNoiseHistParams.ranges[2], (INT8) pNoiseHistParams.ranges[3],
                            (INT8) pNoiseHistParams.ranges[4], (INT8) pNoiseHistParams.ranges[5],
                            (INT8) pNoiseHistParams.ranges[6], (INT8) pNoiseHistParams.ranges[7]));

            /* Send a Start command to the FW */
            status = (TI_STATUS)whalCtrl_NoiseHistogramCmd( pMeasurementSRV->hHalCtrl, &pNoiseHistParams );

            if ( OK == status )
            {
                /* Print for Debug */
                WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
                                         ("%s: Sent noise histogram command. Starting timer\n",
                                          __FUNCTION__) );

                /* Start Timer */
                os_timerStart( pMeasurementSRV->hOS, pMeasurementSRV->hRequestTimer[ requestIndex ], 
                               pMeasurementSRV->msrRequest.msrTypes[ requestIndex ].duration, FALSE );
                pMeasurementSRV->bRequestTimerRunning[ requestIndex ] = TRUE;
            }
            else
            {
                WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                                   ("%s: whalCtrl_NoiseHistogramCmd returned status %d\n", __FUNCTION__, status) );
            }
            break;
        
        case MSR_TYPE_BEACON_MEASUREMENT:
            /* set all parameters in the AP discovery command */
            pApDiscoveryParams.scanDuration = pMeasurementSRV->msrRequest.msrTypes[ requestIndex ].duration * 1000; /* TODO change this to an infinite value (was 0) */
            pApDiscoveryParams.numOfProbRqst = 1;
            pApDiscoveryParams.txdRateSet = HW_BIT_RATE_1MBPS;
            pApDiscoveryParams.ConfigOptions = RX_CONFIG_OPTION_FOR_MEASUREMENT;
            pApDiscoveryParams.FilterOptions = RX_FILTER_OPTION_DEF_PRSP_BCN;
            pApDiscoveryParams.txPowerDbm = pMeasurementSRV->msrRequest.txPowerDbm;
            pApDiscoveryParams.scanOptions = SCAN_ACTIVE; /* both scan type and band are 0 for active and */
                                                          /* 2.4 GHz, respectively, but 2.4 is not defined */

            /* band and scan mode go into the scan option field */
            if ( RADIO_BAND_5_0_GHZ == pMeasurementSRV->msrRequest.band )
            {
                pApDiscoveryParams.scanOptions |= SCAN_5GHZ_BAND;
            }

            if ( MSR_SCAN_MODE_PASSIVE == pMeasurementSRV->msrRequest.msrTypes[ requestIndex ].scanMode )
            {
                pApDiscoveryParams.scanOptions |= SCAN_PASSIVE;
            }

            /* Send AP Discovery command */
            status = (TI_STATUS)whalCtrl_ApDiscoveryCmd( pMeasurementSRV->hHalCtrl, &pApDiscoveryParams );

            if ( OK == status )
            {
                WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
                          ("%s: AP discovery command sent. Params:\n scanDuration=%d, scanOptions=%d,"
                           "numOfProbRqst=%d, txdRateSet=%d, txPowerDbm=%d, configOptions=%d, filterOptions=%d\n"
                           "Starting timer...\n", 
                          __FUNCTION__,
                          pApDiscoveryParams.scanDuration, pApDiscoveryParams.scanOptions,
                          pApDiscoveryParams.numOfProbRqst, pApDiscoveryParams.txdRateSet,
                          pApDiscoveryParams.txPowerDbm, pApDiscoveryParams.ConfigOptions,
                          pApDiscoveryParams.FilterOptions) );
        
                /* Start Timer */
                os_timerStart( pMeasurementSRV->hOS, pMeasurementSRV->hRequestTimer[ requestIndex ], 
                               pMeasurementSRV->msrRequest.msrTypes[ requestIndex ].duration, FALSE);
                pMeasurementSRV->bRequestTimerRunning[ requestIndex ] = TRUE;
            }
            else
            {
                WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                                   ("%s: whalCtrl_ApDiscoveryCmd returned status %d\n", __FUNCTION__, status) );
            }
            break;

        case MSR_TYPE_BASIC_MEASUREMENT: /* not supported in current implemntation */
        case MSR_TYPE_FRAME_MEASUREMENT: /* not supported in current implemntation */
        default:
            WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
                               ("%s: Measurement type %d is not supported\n", 
                               __FUNCTION__, pMeasurementSRV->msrRequest.msrTypes[ requestIndex ].msrType) );
            break;
        }        
    }

    /* if no measurement types are running, sen al types complete event.
       This can happen if all types failed to start */
    if ( TRUE == measurementSRVIsMeasurementComplete( hMeasurementSRV ) )
    {
        /* send the event */
        measurementSRVSM_SMEvent( hMeasurementSRV, &(pMeasurementSRV->SMState), 
                                  MSR_SRV_EVENT_ALL_TYPES_COMPLETE );
    }

    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 08-November-2005\n
 * \brief Handle an ALL_TYPE_COMPLETE event by sending a stop measure command to the FW.\n
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSrv - handle to the Measurement SRV object.\n
 * \return always OK.\n
 */
TI_STATUS measurementSRVSM_requestMeasureStop( TI_HANDLE hMeasurementSRV )
{
    measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
    TI_STATUS status;

    /* since this function may also be called when stop is requested and start complete event
       has not yet been received from the FW, we may need to stop the FW guard timer */
    if ( TRUE == pMeasurementSRV->bStartStopTimerRunning )
    {
		WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,  ("***** STOP TIMER 6 *****\n") );
        os_timerStop( pMeasurementSRV->hOS, pMeasurementSRV->hStartStopTimer );
        pMeasurementSRV->bStartStopTimerRunning = FALSE;
    }

    /* Send Measurement Stop command to the FW */
    status = (TI_STATUS)whalCtrl_measurementStop( pMeasurementSRV->hHalCtrl, 
                                       (void *) pMeasurementSRV->commandResponseCBFunc,
                                       pMeasurementSRV->commandResponseCBObj );

    pMeasurementSRV->commandResponseCBFunc = NULL;
    pMeasurementSRV->commandResponseCBObj = NULL;
    
    if ( OK != status )
    {
        WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                           ("%s: Failed to send measurement stop command, statud=%d,\n",
                            __FUNCTION__, status) );

        /* send a measurement complete event - since it can't be stopped */
        measurementSRVSM_SMEvent( hMeasurementSRV, &(pMeasurementSRV->SMState), MSR_SRV_EVENT_STOP_COMPLETE );
        return OK;
    }

    WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
                             ("%s: measure stop command sent.\n", __FUNCTION__) );

    /* start the FW guard timer */
    pMeasurementSRV->bStartStopTimerRunning = TRUE;
    os_timerStart( pMeasurementSRV->hOS, pMeasurementSRV->hStartStopTimer, MSR_FW_GUARD_TIME, FALSE );

    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 08-November-2005\n
 * \brief Handle a STOP_COMPLETE event by exiting driver mode and calling the complete CB.\n
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSrv - handle to the Measurement SRV object.\n
 * \return always OK.\n
 */
TI_STATUS measurementSRVSM_completeMeasure( TI_HANDLE hMeasurementSRV )
{
    measurementSRV_t *pMeasurementSRV = (measurementSRV_t *)hMeasurementSRV;

    /* Switch Power Save SRV back to user mode */
    powerSrv_ReleasePS( pMeasurementSRV->hPowerSaveSRV, pMeasurementSRV->bSendNullDataWhenExitPs, NULL, NULL );

    /* if the response CB is still pending, call it (when requestExpiryTimeStamp was reached) */
    if ( NULL != pMeasurementSRV->commandResponseCBFunc )
    {
        pMeasurementSRV->commandResponseCBFunc( pMeasurementSRV->commandResponseCBObj, OK );
    }

    /* call the complete CB */
    if ( NULL != pMeasurementSRV->measurmentCompleteCBFunc )
    {
        pMeasurementSRV->measurmentCompleteCBFunc( pMeasurementSRV->measurementCompleteCBObj, 
                                                   &(pMeasurementSRV->msrReply) );
    }

    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 08-November-2005\n
 * \brief Handle a STOP_REQUEST event when in WAIT_FOR_DRIVER_MODE state by exiting driver mode.
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSrv - handle to the Measurement SRV object.\n
 * \return always OK.\n
 */
TI_STATUS measurementSRVSM_stopFromWaitForDriverMode( TI_HANDLE hMeasurementSRV )
{
    measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;

    /* Switch Power Save SRV back to user mode */
    powerSrv_ReleasePS( pMeasurementSRV->hPowerSaveSRV, pMeasurementSRV->bSendNullDataWhenExitPs, NULL, NULL );

    /* if we are not running within a stop request context (shouldn't happen), call the CBs */
    if ( FALSE == pMeasurementSRV->bInRequest )
    {
        WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
                           ("%s: stop from wait for driver mode: not within a request context?!? \n", __FUNCTION__) );

        /* call the response CB - this shouldn't happen, as only GWSI has response CB, and it shouldn't call
           stop before driver */
        if ( NULL != pMeasurementSRV->commandResponseCBFunc )
        {
            WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
                               ("%s: stop from wait for driver mode: command response CB is not NULL?!? \n", __FUNCTION__) );
            pMeasurementSRV->commandResponseCBFunc( pMeasurementSRV->commandResponseCBObj, OK );

            pMeasurementSRV->commandResponseCBFunc = NULL;
            pMeasurementSRV->commandResponseCBObj = NULL;
        }
        /* call the complete CB */
        if ( NULL != pMeasurementSRV->measurmentCompleteCBFunc )
        {
            /* mark that all types has failed */
            int i;
            for ( i = 0; i < MAX_NUM_OF_MSR_TYPES_IN_PARALLEL; i++ )
            {
                pMeasurementSRV->msrReply.msrTypes[ i ].status = NOK;
            }
            /* call the complete CB */
            pMeasurementSRV->measurmentCompleteCBFunc( pMeasurementSRV->measurementCompleteCBObj, 
                                                       &(pMeasurementSRV->msrReply) );
        }
        else
        {
            WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                               ("%s: stop from wait for driver mode and response CB is NULL!!!\n",
                                __FUNCTION__) );
        }
    }
    /* we are within a stop request context */
    else
    {
        /* if the command response Cb is valid, send a measure stop command to the FW - 
           although it is not necessary, we need it to get a different context for the command response.
           This shouldn't happen, as only GWSI has command response, and it shouldn't call stop measure
           before it got the commadn response for start measure */
        if ( NULL != pMeasurementSRV->commandResponseCBFunc )
        {
            /* shouldn't happen - a command response is valid (GWSI) and stop measure called 
               before measure start response was received (driver) */
            WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                               ("%s: stop from wait for driver mode - within request context and command response is not NULL?!?\n", __FUNCTION__) );

            whalCtrl_measurementStop( pMeasurementSRV->hHalCtrl, (void *)pMeasurementSRV->commandResponseCBFunc,
                                      pMeasurementSRV->commandResponseCBObj );

            pMeasurementSRV->commandResponseCBFunc = NULL;
            pMeasurementSRV->commandResponseCBObj = NULL;
        }
        if ( NULL != pMeasurementSRV->measurmentCompleteCBFunc )
        {
            /* Note: this is being called from request context, but there's npthing else that can be done */
            /* mark that all types has failed */
            int i;
            for ( i = 0; i < MAX_NUM_OF_MSR_TYPES_IN_PARALLEL; i++ )
            {
                pMeasurementSRV->msrReply.msrTypes[ i ].status = NOK;
            }
            /* call the complete CB */
            pMeasurementSRV->measurmentCompleteCBFunc( pMeasurementSRV->measurementCompleteCBObj, 
                                                       &(pMeasurementSRV->msrReply) );
        }
    }

    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 27-November-2005\n
 * \brief handle a STOP_REQUEST event when in WAIT_FOR_DRIVER_MODE by marking negative result status
 * \brief and calling the ordinary stop function
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSrv - handle to the Measurement SRV object.\n
 * \return always OK.\n
 */
TI_STATUS measurementSRVSM_stopFromWaitForMeasureStart( TI_HANDLE hMeasurementSRV )
{
    measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
    int i;

    /* mark that all types has failed */
    for ( i = 0; i < pMeasurementSRV->msrRequest.numberOfTypes; i++ )
    {
        pMeasurementSRV->msrReply.msrTypes[ i ].status = NOK;
    }

    /* call the ordinary stop function (will send a measure stop command to FW) */
    measurementSRVSM_requestMeasureStop( hMeasurementSRV );

    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 08-November-2005\n
 * \brief handle a STOP_REQUEST event when in MEASURE_IN_PROGRESS by stopping all measure types and
 * \brief requesting measure stop from the FW.\n
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSrv - handle to the Measurement SRV object.\n
 * \return always OK.\n
 */
TI_STATUS measurementSRVSM_stopFromMeasureInProgress( TI_HANDLE hMeasurementSRV )
{
    measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
    whalCtrl_noiseHistogram_t   pNoiseHistParams;
    TI_STATUS                   status;
    int                         i;

    /* stop all running measure types */
    for ( i = 0; i < pMeasurementSRV->msrRequest.numberOfTypes; i++ )
    {
        if ( TRUE == pMeasurementSRV->bRequestTimerRunning[ i ] )
        {
            /* stop timer */
            os_timerStop( pMeasurementSRV->hOS, pMeasurementSRV->hRequestTimer[ i ] );
            pMeasurementSRV->bRequestTimerRunning[ i ] = FALSE;

            /* if necessary, stop measurement type */
            switch ( pMeasurementSRV->msrRequest.msrTypes[ i ].msrType )
            {
            case MSR_TYPE_BEACON_MEASUREMENT:
                /* send stop AP discovery command */
                status = (TI_STATUS)whalCtrl_ApDiscoveryStop( pMeasurementSRV->hHalCtrl );
                if ( OK != status )
                {
                    WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                                       ("%s: whalCtrl_ApDiscoveryStop returned status %d\n", __FUNCTION__, status) );
                }
                break;

            case MSR_TYPE_NOISE_HISTOGRAM_MEASUREMENT:
                /* Set Noise Histogram Cmd Params */
                pNoiseHistParams.cmd = STOP_NOISE_HIST;
                pNoiseHistParams.sampleInterval = 0;
                os_memoryZero( pMeasurementSRV->hOS, &(pNoiseHistParams.ranges[ 0 ]), MEASUREMENT_NOISE_HISTOGRAM_NUM_OF_RANGES );

                /* Send a Stop command to the FW */
                status = (TI_STATUS)whalCtrl_NoiseHistogramCmd( pMeasurementSRV->hHalCtrl, &pNoiseHistParams );

                if ( OK != status )
                {
                    WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                                       ("%s: whalCtrl_NoiseHistogramCmd returned status %d\n", __FUNCTION__, status) );
                }
                break;

            /* These are just to avoid compilation warnings, nothing is actualy done here! */
            case MSR_TYPE_BASIC_MEASUREMENT:
            case MSR_TYPE_CCA_LOAD_MEASUREMENT:
            case MSR_TYPE_FRAME_MEASUREMENT:
            case MSR_TYPE_MAX_NUM_OF_MEASURE_TYPES:
            default:
                WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                                   ("%s: unsupported measurement type: %d\n",
                                    __FUNCTION__,
                                    pMeasurementSRV->msrRequest.msrTypes[ i ].msrType) );
                break;
            }

            /* mark that measurement has failed */
            pMeasurementSRV->msrReply.msrTypes[ i ].status = NOK;
        }
    }

    /* Send Measurement Stop command to the FW */
    status = (TI_STATUS)whalCtrl_measurementStop( pMeasurementSRV->hHalCtrl,
                                       (void *)pMeasurementSRV->commandResponseCBFunc,
                                       pMeasurementSRV->commandResponseCBObj );

    pMeasurementSRV->commandResponseCBFunc = NULL;
    pMeasurementSRV->commandResponseCBObj = NULL;
    
    if ( OK != status )
    {
        WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                           ("%s: Failed to send measurement stop command, statud=%d,\n",
                            __FUNCTION__, status) );

        /* send a measurement complete event - since it can't be stopped */
        measurementSRVSM_SMEvent( hMeasurementSRV, &(pMeasurementSRV->SMState),
                                  MSR_SRV_EVENT_STOP_COMPLETE );
        return OK;
    }

    WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
                             ("%s: measure stop command sent.\n") );

    /* start the FW guard timer */
    pMeasurementSRV->bStartStopTimerRunning = TRUE;
    os_timerStart( pMeasurementSRV->hOS, pMeasurementSRV->hStartStopTimer, MSR_FW_GUARD_TIME, FALSE );

    return OK; 
}

/**
 * \author Ronen Kalish\n
 * \date 08-November-2005\n
 * \brief handle a DRIVER_MODE_FAILURE event by calling the response and complete CBs.\n
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSrv - handle to the Measurement SRV object.\n
 * \return always OK.\n
 */
TI_STATUS measurementSRVSM_DriverModeFailure( TI_HANDLE hMeasurementSRV )
{
    measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;

    /* this function can be called from within a request (when the power save SRV returned an immediate error),
       or in a different context, when power save entry failed. The latter is a valid status, whereas the former
       indicates a severe error. However, as there is nothing to do with the former (other than debug it), the same
       failure indication is used for both of them, which will make the upper layer (Palau driver or TI measurement
       manager) to return to idle state. Still, for the former the error is returned as the return status from the
       measurement start API call whereas for the latter the error is indicated both by the command response and
       measurement complete CBs status */

    /* if we are running within a request context, don't call the CBs! The startMeasurement function
       will return an invalid status instead */
    if ( FALSE == pMeasurementSRV->bInRequest )
    {
        /* if a response CB is available (GWSI) call it */
        if ( NULL != pMeasurementSRV->commandResponseCBFunc )
        {
            pMeasurementSRV->commandResponseCBFunc( pMeasurementSRV->commandResponseCBObj, NOK );
        }

        /* if a complete CB is available (both GWSI and TI driver), call it */
        if ( NULL != pMeasurementSRV->measurmentCompleteCBFunc )
        {
            /* mark that all types has failed */
            int i;
            for ( i = 0; i < MAX_NUM_OF_MSR_TYPES_IN_PARALLEL; i++ )
            {
                pMeasurementSRV->msrReply.msrTypes[ i ].status = NOK;
            }
            /* call the complete CB */
            pMeasurementSRV->measurmentCompleteCBFunc( pMeasurementSRV->measurementCompleteCBObj, 
                                                       &(pMeasurementSRV->msrReply) );
        }
        else
        {
            WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                               ("%s: driver mode failure and complete CB is NULL!!!\n",
                                __FUNCTION__) );
        }
    }

    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 08-November-2005\n
 * \brief handle a START_FAILURE event by exiting driver mode and calling the complete CB.\n
 *
 * Function Scope \e Public.\n
 * \param hMeasurementSrv - handle to the Measurement SRV object.\n
 * \return always OK.\n
 */
TI_STATUS measurementSRVSM_measureStartFailure( TI_HANDLE hMeasurementSRV )
{
    measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;

    /* This function can be called from within a request context (if the driver mode entry process
       was immediate), or from the driver mode CB context. Regardless of teh context in which it runs,
       The error indicates that something is wrong in the HAL. There is no way to solve this (other than debug it).
       The error is either indicating by the measurement start API return status (if still in the request context),
       or by calling the response (if available, only in GWSI) and complete CBs with invalid status */

    /* Switch Power save SRV back to user mode */
    powerSrv_ReleasePS( pMeasurementSRV->hPowerSaveSRV, pMeasurementSRV->bSendNullDataWhenExitPs, NULL, NULL );

    /* if we are running within a request context, don't call the CB! The startMeasurement function
       will return an invalid status instead */
    if ( FALSE == pMeasurementSRV->bInRequest )
    {
        /* if a response CB is available (GWSI) call it */
        if ( NULL != pMeasurementSRV->commandResponseCBFunc )
        {
            pMeasurementSRV->commandResponseCBFunc( pMeasurementSRV->commandResponseCBObj, NOK );
        }

        /* if a complete CB is available (both GWSI and TI driver), call it */
        if ( NULL != pMeasurementSRV->measurmentCompleteCBFunc )
        {
            /* mark that all types has failed */
            int i;
            for ( i = 0; i < MAX_NUM_OF_MSR_TYPES_IN_PARALLEL; i++ )
            {
                pMeasurementSRV->msrReply.msrTypes[ i ].status = NOK;
            }
            /* call the complete CB */
            pMeasurementSRV->measurmentCompleteCBFunc( pMeasurementSRV->measurementCompleteCBObj, 
                                                       &(pMeasurementSRV->msrReply) );
        }
        else
        {
            WLAN_REPORT_ERROR( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                               ("%s: Start measurement failure and response and complete CBs are NULL!!!\n",
                                __FUNCTION__) );
        }
    }

    return OK;
}



void measurementSRVSM_requestMeasureStartResponseCB(TI_HANDLE hMeasurementSRV, UINT16 uMboxStatus)
{
	measurementSRV_t* pMeasurementSRV = (measurementSRV_t*)hMeasurementSRV;
	int i;

	WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                               ("%s: FW has responded with CMD_STATUS = %d\n",
                                __FUNCTION__, uMboxStatus) );

	if (uMboxStatus == OK) 
	{
		WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                               ("%s: FW has responded with CMD_STATUS_SUCCESS!\n",
                                __FUNCTION__) );

		if ( NULL != pMeasurementSRV->commandResponseCBFunc )
        {
            pMeasurementSRV->commandResponseCBFunc( pMeasurementSRV->commandResponseCBObj, OK );
        }
	}
	else
	{
		if (uMboxStatus == SG_REJECT_MEAS_SG_ACTIVE) 
		{
			WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                               ("%s: FW has responded with CMD_STATUS_REJECT_MEAS_SG_ACTIVE!\n",
                                __FUNCTION__) );
		}

		WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                               ("%s: FW has responded with CMD_STATUS NOK!!!\n",
                                __FUNCTION__) );


		/* if a timer is running, stop it */
		if ( TRUE == pMeasurementSRV->bStartStopTimerRunning )
		{
			WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,  ("***** STOP TIMER 8 *****\n") );
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
		
		measurementSRVSM_SMEvent( hMeasurementSRV, &(pMeasurementSRV->SMState),
											 MSR_SRV_EVENT_START_FAILURE );
	}
}


/**
 * \author Ronen Kalish\n
 * \date 23-December-2005\n
 * \brief Handles a stop request when no stop is needed (SM is either idle or already send stop command to FW.\n
 *
 * Function Scope \e Private.\n
 * \param hMeasurementSrv - handle to the measurement SRV object.\n
 * \return always OK.\n
 */
TI_STATUS measurementSRVSRVSM_dummyStop( TI_HANDLE hMeasurementSrv )
{
    measurementSRV_t *pMeasurementSRV = (measurementSRV_t*)hMeasurementSrv;

    WLAN_REPORT_WARNING( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,
                         ("%s: sending unnecessary stop measurement command to FW...\n", __FUNCTION__) );

    /* send a stop command to FW, to obtain a different context in ehich to cal the command response CB */
    whalCtrl_measurementStop( pMeasurementSRV->hHalCtrl, (void *)pMeasurementSRV->commandResponseCBFunc,
                              pMeasurementSRV->commandResponseCBObj );

    pMeasurementSRV->commandResponseCBFunc = NULL;
    pMeasurementSRV->commandResponseCBObj = NULL;

    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 17-November-2005\n
 * \brief Handles an unexpected event.\n
 *
 * Function Scope \e Private.\n
 * \param hMeasurementSrv - handle to the measurement SRV object.\n
 * \return always OK.\n
 */
TI_STATUS actionUnexpected( TI_HANDLE hMeasurementSrv ) 
{
    measurementSRV_t *pMeasurementSRV = (measurementSRV_t*)hMeasurementSrv;
    int i;

    WLAN_REPORT_SM( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG, 
                    ("%s: measurement SRV state machine error, unexpected Event\n", __FUNCTION__) );

    if ( TRUE == pMeasurementSRV->bStartStopTimerRunning )
    {
		WLAN_REPORT_INFORMATION( pMeasurementSRV->hReport, MEASUREMENT_SRV_MODULE_LOG,  ("***** STOP TIMER 7 *****\n") );
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

    /* we must clean the old command response CB since they are no longer relevant 
      since the state machine may be corrupted */
    pMeasurementSRV->commandResponseCBFunc = NULL;
    pMeasurementSRV->commandResponseCBObj = NULL;

    /* indicate the unexpected event in the return status */
    pMeasurementSRV->returnStatus = NOK;
    
    return OK;
}

/**
 * \author Ronen Kalish\n
 * \date 10-Jan-2005\n
 * \brief Handles an event that doesn't require any action.\n
 *
 * Function Scope \e Private.\n
 * \param hMeasurementSrv - handle to the measurement SRV object.\n
 * \return always OK.\n
 */
TI_STATUS actionNop( TI_HANDLE hMeasurementSrv )
{   
    return OK;
}

