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

/**************************************************************************/
/*																		  */
/*		MODULE:		measurementMgrSM.c									  */
/*		PURPOSE:	Measurement Manager State Machine module interface.   */
/*																		  */
/**************************************************************************/



#include "measurementMgrSM.h"
#include "measurementMgr.h"
#include "802_11Defs.h"
#ifdef EXC_MODULE_INCLUDED
 #include "excMngr.h"
 #include "excRMMngr.h"
#endif
#include "spectrumMngmntMgr.h"
#include "siteMgrApi.h"
#include "MacServices_api.h"
#include "healthMonitor.h"
#include "regulatoryDomainApi.h"
#include "whalCtrl_api.h"





char * measurementMgr_stateDesc[MEASUREMENTMGR_NUM_STATES] =
{  
	"STATE_IDLE",
    "STATE_PROCESSING_REQUEST",
    "STATE_WAITING_FOR_SCR",
    "STATE_MEASURING"
};

	
char * measurementMgr_eventDesc[MEASUREMENTMGR_NUM_EVENTS] =
{
	"EVENT_CONNECTED",
	"EVENT_DISCONNECTED",
	"EVENT_ENABLE",
	"EVENT_DISABLE",
    "EVENT_FRAME_RECV",
    "EVENT_SEND_REPORT",
	"EVENT_REQUEST_SCR",
    "EVENT_SCR_WAIT",
    "EVENT_SCR_RUN",
    "EVENT_ABORT",
    "EVENT_COMPLETE",
    "EVENT_FW_RESET"
};




/********************************************************************************/
/*						MeasurementMgr SM Action Prototypes						*/
/********************************************************************************/

static TI_STATUS measurementMgrSM_acUnexpected(void * pData);

static TI_STATUS measurementMgrSM_acNop(void * pData);


static TI_STATUS measurementMgrSM_acConnected(void * pData);

static TI_STATUS measurementMgrSM_acDisconnected_fromIdle(void * pData);

static TI_STATUS measurementMgrSM_acEnable(void * pData);

static TI_STATUS measurementMgrSM_acDisable_fromIdle(void * pData);

static TI_STATUS measurementMgrSM_acFrameReceived_fromIdle(void * pData);

static TI_STATUS measurementMgrSM_acSendReportAndCleanObj(void * pData);


static TI_STATUS measurementMgrSM_acDisconnected_fromProcessingRequest(void * pData);

static TI_STATUS measurementMgrSM_acDisable_fromProcessingRequest(void * pData);

static TI_STATUS measurementMgrSM_acFrameReceived_fromProcessingRequest(void * pData);

static TI_STATUS measurementMgrSM_acAbort_fromProcessingRequest(void * pData);

static TI_STATUS measurementMgrSM_acRequestSCR(void * pData);


static TI_STATUS measurementMgrSM_acDisconnected_fromWaitForSCR(void * pData);

static TI_STATUS measurementMgrSM_acDisable_fromWaitForSCR(void * pData);

static TI_STATUS measurementMgrSM_acFrameReceived_fromWaitForSCR(void * pData);

static TI_STATUS measurementMgrSM_acAbort_fromWaitForSCR(void * pData);

static TI_STATUS measurementMgrSM_acStartMeasurement(void * pData);


static TI_STATUS measurementMgrSM_acDisconnected_fromMeasuring(void * pData);

static TI_STATUS measurementMgrSM_acDisable_fromMeasuring(void * pData);

static TI_STATUS measurementMgrSM_acFrameReceived_fromMeasuring(void * pData);

static TI_STATUS measurementMgrSM_acAbort_fromMeasuring(void * pData);

static TI_STATUS measurementMgrSM_acMeasurementComplete(void * pData);

static TI_STATUS measurementMgrSM_acFirmwareReset(void * pData);








/********************************************************************************/
/*						Internal Functions Prototypes							*/
/********************************************************************************/

static void measurementMgrSM_resetParams(measurementMgr_t * pMeasurementMgr);








/********************************************************************************/
/*						MeasurementMgr SM General Use Functions					*/
/********************************************************************************/


/**
 * Configures the Measurement Manager State Machine.
 * 
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 * 
 * @date 01-Jan-2006
 */
TI_STATUS measurementMgrSM_config(TI_HANDLE hMeasurementMgr)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;
    TI_STATUS status;

    /* MeasurementMgr State Machine matrix */
	fsm_actionCell_t measurementMgr_matrix[MEASUREMENTMGR_NUM_STATES][MEASUREMENTMGR_NUM_EVENTS] =
	{
		/* next state and actions for STATE_IDLE state */    
		{
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acConnected},				/* CONNECTED         */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acDisconnected_fromIdle},	/* DISCONNECTED      */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acEnable},					/* ENABLE            */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acDisable_fromIdle},		/* DISABLE           */
			{MEASUREMENTMGR_STATE_PROCESSING_REQUEST, measurementMgrSM_acFrameReceived_fromIdle},	/* FRAME_RECV        */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acSendReportAndCleanObj},	/* SEND_REPORT       */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acUnexpected},				/* REQUEST_SCR       */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acUnexpected},				/* SCR_WAIT          */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acUnexpected},				/* SCR_RUN           */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acUnexpected},				/* ABORT             */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acUnexpected},				/* COMPLETE          */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acUnexpected}				/* FW_RESET          */
		},

		/* next state and actions for STATE_PROCESSING_REQUEST state */    
		{
			{MEASUREMENTMGR_STATE_PROCESSING_REQUEST, measurementMgrSM_acUnexpected},			/* CONNECTED         */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acDisconnected_fromProcessingRequest},	/* DISCONNECTED      */
			{MEASUREMENTMGR_STATE_PROCESSING_REQUEST, measurementMgrSM_acNop},					/* ENABLE            */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acDisable_fromProcessingRequest},		/* DISABLE           */
			{MEASUREMENTMGR_STATE_PROCESSING_REQUEST, measurementMgrSM_acFrameReceived_fromProcessingRequest},	/* FRAME_RECV        */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acSendReportAndCleanObj},				/* SEND_REPORT       */
			{MEASUREMENTMGR_STATE_WAITING_FOR_SCR, measurementMgrSM_acRequestSCR},				/* REQUEST_SCR       */
			{MEASUREMENTMGR_STATE_PROCESSING_REQUEST, measurementMgrSM_acUnexpected},			/* SCR_WAIT          */
			{MEASUREMENTMGR_STATE_PROCESSING_REQUEST, measurementMgrSM_acUnexpected},			/* SCR_RUN           */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acAbort_fromProcessingRequest},		/* ABORT             */
			{MEASUREMENTMGR_STATE_PROCESSING_REQUEST, measurementMgrSM_acUnexpected},			/* COMPLETE          */
			{MEASUREMENTMGR_STATE_PROCESSING_REQUEST, measurementMgrSM_acUnexpected}			/* FW_RESET          */
		},

		/* next state and actions for STATE_WAITING_FOR_SCR state */    
		{
			{MEASUREMENTMGR_STATE_WAITING_FOR_SCR, measurementMgrSM_acUnexpected},						/* CONNECTED         */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acDisconnected_fromWaitForSCR},				/* DISCONNECTED      */
			{MEASUREMENTMGR_STATE_WAITING_FOR_SCR, measurementMgrSM_acNop},								/* ENABLE            */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acDisable_fromWaitForSCR},						/* DISABLE           */
			{MEASUREMENTMGR_STATE_PROCESSING_REQUEST, measurementMgrSM_acFrameReceived_fromWaitForSCR},	/* FRAME_RECV        */
			{MEASUREMENTMGR_STATE_WAITING_FOR_SCR, measurementMgrSM_acUnexpected},						/* SEND_REPORT       */
			{MEASUREMENTMGR_STATE_WAITING_FOR_SCR, measurementMgrSM_acUnexpected},						/* REQUEST_SCR       */
			{MEASUREMENTMGR_STATE_WAITING_FOR_SCR, measurementMgrSM_acNop},								/* SCR_WAIT          */
			{MEASUREMENTMGR_STATE_MEASURING, measurementMgrSM_acStartMeasurement},						/* SCR_RUN           */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acAbort_fromWaitForSCR},						/* ABORT             */
			{MEASUREMENTMGR_STATE_WAITING_FOR_SCR, measurementMgrSM_acUnexpected},						/* COMPLETE          */
			{MEASUREMENTMGR_STATE_WAITING_FOR_SCR, measurementMgrSM_acUnexpected}						/* FW_RESET          */
		},

		/* next state and actions for STATE_MEASURING state */    
		{
			{MEASUREMENTMGR_STATE_MEASURING, measurementMgrSM_acUnexpected},					/* CONNECTED         */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acDisconnected_fromMeasuring},			/* DISCONNECTED      */
			{MEASUREMENTMGR_STATE_MEASURING, measurementMgrSM_acNop},							/* ENABLE            */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acDisable_fromMeasuring},				/* DISABLE           */
			{MEASUREMENTMGR_STATE_PROCESSING_REQUEST, measurementMgrSM_acFrameReceived_fromMeasuring},	/* FRAME_RECV        */
			{MEASUREMENTMGR_STATE_MEASURING, measurementMgrSM_acUnexpected},					/* SEND_REPORT       */
			{MEASUREMENTMGR_STATE_MEASURING, measurementMgrSM_acUnexpected},					/* REQUEST_SCR       */
			{MEASUREMENTMGR_STATE_MEASURING, measurementMgrSM_acUnexpected},					/* SCR_WAIT          */
			{MEASUREMENTMGR_STATE_MEASURING, measurementMgrSM_acUnexpected},					/* SCR_RUN           */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acAbort_fromMeasuring},				/* ABORT             */
			{MEASUREMENTMGR_STATE_PROCESSING_REQUEST, measurementMgrSM_acMeasurementComplete},	/* COMPLETE          */
			{MEASUREMENTMGR_STATE_IDLE, measurementMgrSM_acFirmwareReset}						/* FW_RESET          */
		}

	};

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG,
			("%s: Configured MeasurementMgr state machine\n", __FUNCTION__));
	
	status = fsm_Config(pMeasurementMgr->pMeasurementMgrSm, 
						&measurementMgr_matrix[0][0], 
						MEASUREMENTMGR_NUM_STATES, 
						MEASUREMENTMGR_NUM_EVENTS, 
						measurementMgrSM_event, pMeasurementMgr->hOs);

	return status;
}



/**
 * Raises a State Machine event in the Measurement Manager SM.
 * 
 * @param currentState A point to the member holding the SM's current state.
 * @param event The event we want to raise.
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 * 
 * @date 05-Jan-2006
 */
TI_STATUS measurementMgrSM_event(UINT8 * currentState, UINT8 event, TI_HANDLE hMeasurementMgr)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;
	TI_STATUS status;
	UINT8 nextState;

	status = fsm_GetNextState(pMeasurementMgr->pMeasurementMgrSm, 
								*currentState, event, &nextState);

	if (status != OK)
	{
		WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
				("%s: State machine error, failed getting next state\n", __FUNCTION__));

		return(NOK);
	}

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: <%s, %s> --> %s\n", __FUNCTION__,
				   measurementMgr_stateDesc[*currentState],
				   measurementMgr_eventDesc[event],
				   measurementMgr_stateDesc[nextState]));

	status = fsm_Event(pMeasurementMgr->pMeasurementMgrSm,
						currentState, event, (void *) pMeasurementMgr);

	return status;
}







/********************************************************************************/
/*					MeasurementMgr SM Action Functions							*/
/********************************************************************************/


/********************************************************************************/
/*                            IDLE State Actions                                */
/********************************************************************************/

/**
 * Performs the required action when the Measurement Manager module has
 * been advised that the station has connected to an AP.
 * 
 * @date 05-Jan-2006
 */
static TI_STATUS measurementMgrSM_acConnected(void * pData)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;
	paramInfo_t param;

#ifdef EXC_MODULE_INCLUDED
	iappParsingRegistrationTable_t iappParsingRegistration;
#endif

	/* do nothing if we're already in connected mode */
	if (pMeasurementMgr->Connected)
	{
		WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG,
				("%s: Connected flag already set\n", __FUNCTION__));

		return OK;
	}

	pMeasurementMgr->Connected = TRUE;

    WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Connected flag has been set\n", __FUNCTION__));

	/* get the current serving channel */
	param.paramType = SITE_MGR_CURRENT_CHANNEL_PARAM;
	siteMgr_getParam(pMeasurementMgr->hSiteMgr, &param);
	pMeasurementMgr->servingChannelID = param.content.siteMgrCurrentChannel;
	    
#ifdef EXC_MODULE_INCLUDED
	if(pMeasurementMgr->Mode == MSR_MODE_EXC)
	{
		WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
				("%s: MeasurementMgr set to EXC mode\n", __FUNCTION__));

        if(pMeasurementMgr->isModuleRegistered == FALSE)
        {
            /* Registering to the excMngr */
            iappParsingRegistration.handler = pMeasurementMgr;
            iappParsingRegistration.iappParsingRegistrationProcedure = measurementMgr_excParse;

            if (excMngr_registerForRecvIappPacket(pMeasurementMgr->hExcMngr,
				iappParsingRegistration, IAPP_RADIO_MEASUREMENT) != OK)
            {
				WLAN_REPORT_WARNING(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
						("%s: Could not register to receive IAPP packets\n", __FUNCTION__));

                return NOK;
            }

            pMeasurementMgr->isModuleRegistered = TRUE;
        }
        
        pMeasurementMgr->parserFrameReq = measurementMgr_excParseFrameReq;
        pMeasurementMgr->isTypeValid = measurementMgr_excIsTypeValid;
		pMeasurementMgr->buildReport = measurementMgr_excBuildReport;
		pMeasurementMgr->buildRejectReport = measurementMgr_excBuildRejectReport;
		pMeasurementMgr->sendReportAndCleanObj = measurementMgr_excSendReportAndCleanObject;
        requestHandler_setRequestParserFunction(pMeasurementMgr->hRequestH, 
                                                measurementMgr_excParseRequestIEHdr);
	}
	else
#endif
	{
		if(pMeasurementMgr->Mode == MSR_MODE_SPECTRUM_MANAGEMENT)
		{
			WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
					("%s: MeasurementMgr set to Spectrum Management mode\n", __FUNCTION__));

            /* NOTE: These 5 functions need to be corrected to fit the 802.11h standered */
            pMeasurementMgr->parserFrameReq = measurementMgr_dot11hParseFrameReq;
            pMeasurementMgr->isTypeValid = measurementMgr_dot11hIsTypeValid;
			pMeasurementMgr->buildReport = measurementMgr_dot11hBuildReport;
			pMeasurementMgr->buildRejectReport = measurementMgr_dot11hBuildRejectReport;
			pMeasurementMgr->sendReportAndCleanObj = measurementMgr_dot11hSendReportAndCleanObject;
            requestHandler_setRequestParserFunction(pMeasurementMgr->hRequestH, 
                                                    measurementMgr_dot11hParseRequestIEHdr);

		}
	}

	return OK;
}



/**
 * Called when the Measurement Manager has been advised that the station
 * has disconnected from the AP.
 * 
 * @date 05-Jan-2006
 */
static TI_STATUS measurementMgrSM_acDisconnected_fromIdle(void * pData)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Connected flag unset\n", __FUNCTION__));

	pMeasurementMgr->Connected = FALSE;

	return OK;
}



/**
 * Called when the Measurement Manager is enabled.
 * 
 * @date 05-Jan-2006
 */
static TI_STATUS measurementMgrSM_acEnable(void * pData)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Measurement Manager has been enabled\n", __FUNCTION__));

	pMeasurementMgr->Enabled = TRUE;

	return OK;
}



/**
 * Called when the Measurement Manager is disabled.
 * 
 * @date 05-Jan-2006
 */
static TI_STATUS measurementMgrSM_acDisable_fromIdle(void * pData)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Measurement Manager has been disabled\n", __FUNCTION__));

	pMeasurementMgr->Enabled = FALSE;

	return OK;
}



/**
 * Called when the SM is in an idle state and we receive a new measurement frame.
 * 
 * @date 05-Jan-2006
 */
static TI_STATUS measurementMgrSM_acFrameReceived_fromIdle(void * pData)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;
    UINT16 activationDelay;
    TI_STATUS status;
    paramInfo_t param;
    UINT16 tbtt;

	/* handle frame request only if we're connected and measurement is enabled */
	if (pMeasurementMgr->Connected == FALSE ||
		pMeasurementMgr->Enabled == FALSE)
	{
        WLAN_REPORT_ERROR(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Frame received while SM is in disconnected/disabled state\n", __FUNCTION__));

        return measurementMgrSM_event((UINT8 *) &(pMeasurementMgr->currentState), 
                               MEASUREMENTMGR_EVENT_ABORT, pMeasurementMgr);
	}

	/* Setting the frame Type */
	pMeasurementMgr->currentFrameType = pMeasurementMgr->newFrameRequest.frameType;

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Frame Type = %d\n", __FUNCTION__, pMeasurementMgr->currentFrameType));

    /* Getting the Beacon Interval from the Site Mgr */
    param.paramType = SITE_MGR_BEACON_INTERVAL_PARAM;
    status = siteMgr_getParam(pMeasurementMgr->hSiteMgr, &param);
    if (status != OK)
    {
        WLAN_REPORT_ERROR(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Failed to retrieve beacon interval - not connected?\n", __FUNCTION__));

        return measurementMgrSM_event((UINT8 *) &(pMeasurementMgr->currentState), 
                               MEASUREMENTMGR_EVENT_ABORT, pMeasurementMgr);
    }

	/* converting beacon interval to msec */
    tbtt = (param.content.beaconInterval * 1024) / 1000;	/* from TU to msec */   

	/* Initializing Activation Delay Time */
	activationDelay	= pMeasurementMgr->newFrameRequest.hdr->activatioDelay;
	activationDelay	*= tbtt;    
    /* Adding the Measurement Offset to the activation delay */
	activationDelay	+= pMeasurementMgr->newFrameRequest.hdr->measurementOffset;

    /* Inserting all received measurement requests into the queue */
	status = requestHandler_insertRequests(pMeasurementMgr->hRequestH, 
                                           pMeasurementMgr->Mode, 
								           pMeasurementMgr->newFrameRequest);

    /* Clean New Frame Params */
    os_memoryZero(pMeasurementMgr->hOs, &pMeasurementMgr->newFrameRequest, 
                      sizeof(measurement_frameRequest_t));

    if (status != OK)
    {
        pMeasurementMgr->currentFrameType = MSR_FRAME_TYPE_NO_ACTIVE;

		WLAN_REPORT_ERROR(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
				("%s: Could not insert request into the queue\n", __FUNCTION__));

        return measurementMgrSM_event((UINT8 *) &(pMeasurementMgr->currentState), 
                               MEASUREMENTMGR_EVENT_ABORT, pMeasurementMgr);
    }

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: New frame has been inserted into the queue\n", __FUNCTION__));

	/* If frame type isn't Unicast add to Activation Delay a random delay */
	if ((pMeasurementMgr->currentFrameType != MSR_FRAME_TYPE_UNICAST) && (activationDelay > 0))
	{
		activationDelay	+= ((os_timeStampMs(pMeasurementMgr->hOs) % MSR_ACTIVATION_DELAY_RANDOM)
								+ MSR_ACTIVATION_DELAY_OFFSET);
	}

    WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Activation Delay in ms = %d\n", __FUNCTION__, activationDelay));

	if (activationDelay > 0)
	{
		WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
				("%s: Going to wait for activation delay timer callback\n", __FUNCTION__));

		/* Starting the Activation Delay Timer */
		os_timerStart(pMeasurementMgr->hOs, pMeasurementMgr->pActivationDelayTimer, 
					  activationDelay, FALSE);

		return OK;
	}
	else
	{
		WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
				("%s: Activating the next request immediately without waiting for callback\n", __FUNCTION__));

		/* Calling to schedule the first waiting request */
		return measurementMgr_activateNextRequest(pData);
	}
}





/********************************************************************************/
/*                      PROCESSING_REQUEST State Actions                        */
/********************************************************************************/

/**
 * Called when the station disconnects from the AP while processing
 * a measurement request.
 * 
 * @date 05-Jan-2006
 */
static TI_STATUS measurementMgrSM_acDisconnected_fromProcessingRequest(void * pData)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;

    /* Stopping the activationDelay Timer */
    os_timerStop(pMeasurementMgr->hOs, pMeasurementMgr->pActivationDelayTimer);

    /* Clear Measurement fields */
    measurementMgrSM_resetParams(pMeasurementMgr);
           	
	pMeasurementMgr->Connected = FALSE;

	return OK;
}



/**
 * Called when the Measurement Manager module has been disable while
 * processing a measurement request.
 * 
 * @date 05-Jan-2006
 */
static TI_STATUS measurementMgrSM_acDisable_fromProcessingRequest(void * pData)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;

    /* Stopping the activationDelay Timer */
    os_timerStop(pMeasurementMgr->hOs, pMeasurementMgr->pActivationDelayTimer);

    /* Clear Measurement fields  */
    measurementMgrSM_resetParams(pMeasurementMgr);

	pMeasurementMgr->Enabled = FALSE;

    return OK;
}



/**
 * Called when a frame has been received while we are processing another frame.
 * In this case the older frame is discarded and the new frame is processed.
 * 
 * @date 05-Jan-2006
 */
static TI_STATUS measurementMgrSM_acFrameReceived_fromProcessingRequest(void * pData)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;

    /* Stopping the activationDelay Timer */
    os_timerStop(pMeasurementMgr->hOs, pMeasurementMgr->pActivationDelayTimer);

    /* Clear Measurement fields */
    measurementMgrSM_resetParams(pMeasurementMgr);
           	
	/* Process New Frame */
	return measurementMgrSM_acFrameReceived_fromIdle(pData);
}



/**
 * Sends measurement reports to the AP and cleans up the module.
 * 
 * @date 05-Jan-2006
 */
static TI_STATUS measurementMgrSM_acSendReportAndCleanObj(void * pData)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Sending pending reports and cleaning up...\n", __FUNCTION__));

    return pMeasurementMgr->sendReportAndCleanObj(pData);
}



/**
 * Called when for some reason we abort while processing a request.
 * 
 * @date 05-Jan-2006
 */
static TI_STATUS measurementMgrSM_acAbort_fromProcessingRequest(void * pData)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;

    WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Entered\n", __FUNCTION__));

    /* Stopping the activationDelay Timer */
    os_timerStop(pMeasurementMgr->hOs, pMeasurementMgr->pActivationDelayTimer);

    /* Clear Measurement fields */
    measurementMgrSM_resetParams(pMeasurementMgr);
    
    return OK;
}



/**
 * Called when we finished processing a request and want to request the SCR.
 * 
 * @date 05-Jan-2006
 */
static TI_STATUS measurementMgrSM_acRequestSCR(void * pData)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;
	scr_clientRequestStatus_e scrStatus;
	scr_pendReason_e scrPendReason;

	/* Request the channel */
    scrStatus = scr_clientRequest(pMeasurementMgr->hScr, SCR_CID_EXC_MEASURE, &scrPendReason);
	
    if (scrStatus == SCR_CRS_RUN)
    {	
		WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
				("%s: Received RUN response from SCR\n", __FUNCTION__));

		/* The channel is allocated for the measurement */
        return measurementMgrSM_event((UINT8 *) &(pMeasurementMgr->currentState), 
				MEASUREMENTMGR_EVENT_SCR_RUN, pMeasurementMgr);    
    }
    else if ((scrStatus == SCR_CRS_PEND) && (scrPendReason == SCR_PR_DIFFERENT_GROUP_RUNNING))
    {	
		WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
				("%s: Received PEND/DIFFGROUP response from SCR\n", __FUNCTION__));

		/* No need to wait for the channel allocation */
        return measurementMgrSM_event((UINT8 *) &(pMeasurementMgr->currentState), 
				MEASUREMENTMGR_EVENT_ABORT, pMeasurementMgr);  
    }

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Going to wait for SCR callback...\n", __FUNCTION__));

	/* In all other cases wait for the callback function to be called */
    return OK;
}





/********************************************************************************/
/*                        WAIT_FOR_SCR State Actions                            */
/********************************************************************************/


/**
 * Called if the station disconnects from the AP while waiting for a
 * response from the SCR.
 * 
 * @date 05-Jan-2006
 */
static TI_STATUS measurementMgrSM_acDisconnected_fromWaitForSCR(void * pData)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;

	/* Release the SCR */
	scr_clientComplete(pMeasurementMgr->hScr, SCR_CID_EXC_MEASURE);

    /* Clear Measurement fields */
    measurementMgrSM_resetParams(pMeasurementMgr);

	pMeasurementMgr->Connected = FALSE;

    return OK;
}



/**
 * Called if the Measurement Manager module is disabled while we are
 * waiting for a response from the SCR.
 * 
 * @date 05-Jan-2006
 */
static TI_STATUS measurementMgrSM_acDisable_fromWaitForSCR(void * pData)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;

	/* Release the SCR */
	scr_clientComplete(pMeasurementMgr->hScr, SCR_CID_EXC_MEASURE);

    /* Clear Measurement fields */
    measurementMgrSM_resetParams(pMeasurementMgr);

	pMeasurementMgr->Enabled = FALSE;

    return OK;
}



/**
 * Called if a frame is received after we requested the SCR for another frame. 
 * In this case the older frame is discarded and the new frame is processed.
 * 
 * @date 05-Jan-2006
 */
static TI_STATUS measurementMgrSM_acFrameReceived_fromWaitForSCR(void * pData)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;

	/* Release the SCR */
	scr_clientComplete(pMeasurementMgr->hScr, SCR_CID_EXC_MEASURE);

    /* Clear Measurement fields */
    measurementMgrSM_resetParams(pMeasurementMgr);

	/* Process New Frame */
    return measurementMgrSM_acFrameReceived_fromIdle(pData);
}



/**
 * Called if the SCR callbacked with a response other than RUN.
 * 
 * @date 05-Jan-2006
 */
static TI_STATUS measurementMgrSM_acAbort_fromWaitForSCR(void * pData)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;

	/* Release the SCR */
	scr_clientComplete(pMeasurementMgr->hScr, SCR_CID_EXC_MEASURE);

	/* Build a reject report */
	measurementMgr_rejectPendingRequests(pMeasurementMgr, MSR_REJECT_SCR_UNAVAILABLE);

	/* Clear Measurement fields */
    pMeasurementMgr->sendReportAndCleanObj(pMeasurementMgr);

    return OK;
}



/**
 * Called when the SCR callbacks with a RUN response or if the SCR
 * returned a RUN response when we requested it.
 * 
 * @date 05-Jan-2006
 */
static TI_STATUS measurementMgrSM_acStartMeasurement(void * pData)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;

	/* Cryptic: the first struct is the requestHandler request while */
	/* the second one is the measurementSRV request */
    MeasurementRequest_t * pRequestArr[MAX_NUM_REQ];
	measurement_request_t request;

    paramInfo_t	param;
    UINT8 numOfRequestsInParallel;
    UINT8 requestIndex;
	UINT32 timePassed;
	BOOL requestedBeaconMeasurement= FALSE;
	TI_STATUS status;

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG,
			("%s: Starting Measurement operation\n", __FUNCTION__));

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG,
			("%s: Suspending health monitor\n", __FUNCTION__));

    /* Suspend periodic health test */
    healthMonitor_suspendPeriodicTest(pMeasurementMgr->hHealthMonitor);

	request.channel = pMeasurementMgr->measuredChannelID;
	request.startTime = 0;	/* ignored by MeasurementSRV for now - for .11k */
	request.numberOfTypes = 0;

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG,
			("%s: Measured Channel = %d\n", __FUNCTION__, pMeasurementMgr->measuredChannelID));

	param.paramType = REGULATORY_DOMAIN_GET_SCAN_CAPABILITIES;
	param.content.channelCapabilityReq.channelNum = pMeasurementMgr->measuredChannelID;
	param.content.channelCapabilityReq.scanOption = ACTIVE_SCANNING;

	if (pMeasurementMgr->measuredChannelID <= MAX_CHANNEL_IN_BAND_2_4)
	{
		request.band = RADIO_BAND_2_4_GHZ;
		param.content.channelCapabilityReq.band = RADIO_BAND_2_4_GHZ;
	}
	else
	{
		request.band = RADIO_BAND_5_0_GHZ;
		param.content.channelCapabilityReq.band = RADIO_BAND_5_0_GHZ;
	}

	regulatoryDomain_getParam(pMeasurementMgr->hRegulatoryDomain, &param);
	
    request.txPowerDbm = param.content.channelCapabilityRet.maxTxPowerDbm;

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG,
			("%s: Querying Request Handler for the next request in the queue\n", __FUNCTION__));

    /* Getting the next request/requests from the request handler */
    status = requestHandler_getNextReq(pMeasurementMgr->hRequestH, TRUE, pRequestArr,
        &numOfRequestsInParallel);

	if (status != OK)
	{	
		WLAN_REPORT_ERROR(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG,
				("%s: Failed getting next request from Request Handler\n", __FUNCTION__));

        return measurementMgrSM_event((UINT8 *) &(pMeasurementMgr->currentState), 
				MEASUREMENTMGR_EVENT_COMPLETE, pMeasurementMgr);  
	}
	
	/* Save the number of requests in parallel so that once the */
	/* measurement operation ends we can get rid of this amount of requests */
	/* from the requestHandler */
	pMeasurementMgr->currentNumOfRequestsInParallel = numOfRequestsInParallel;

	for (requestIndex = 0; requestIndex < numOfRequestsInParallel; requestIndex++)
	{
        if (pRequestArr[requestIndex]->Type == MSR_TYPE_BEACON_MEASUREMENT)
        {
			requestedBeaconMeasurement = TRUE;

			if (pRequestArr[requestIndex]->ScanMode == MSR_SCAN_MODE_BEACON_TABLE)
			{
				WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG,
						("%s: Beacon Table request encountered, building report now\n", __FUNCTION__));

				/* building Report for beacon table request */
				pMeasurementMgr->buildReport(pMeasurementMgr, *pRequestArr[requestIndex], NULL);

				continue;
			}
        }

        /* save the request so we can reference it when results arrive */
        pMeasurementMgr->currentRequest[request.numberOfTypes] = pRequestArr[requestIndex];

        /* add the measurement type to the request's list */
		request.msrTypes[request.numberOfTypes].duration = pRequestArr[requestIndex]->DurationTime;
		request.msrTypes[request.numberOfTypes].scanMode = pRequestArr[requestIndex]->ScanMode;
		request.msrTypes[request.numberOfTypes].msrType = pRequestArr[requestIndex]->Type;

		WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG,
				("%s:\n\nMeasurement Request #%d Details: Type = %d, Duration = %d\n\n", __FUNCTION__,
						request.numberOfTypes+1,
						request.msrTypes[request.numberOfTypes].msrType,
						request.msrTypes[request.numberOfTypes].duration));

		request.numberOfTypes++;
	}

	if (requestedBeaconMeasurement == TRUE)
	{
        /* build a probe request template and send it to the HAL */
        whalCtrl_setTemplate_t templateStruct;
		probeReqTemplate_t probeReqTemplate;
		ssid_t broadcastSSID;

		/* register to MLME for Beacons/Probe Resp  */
		WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG,
				("%s: Registering at the MLME\n", __FUNCTION__));

		mlmeParser_registerForBeaconAndProbeResp(pMeasurementMgr->hMlme, 
												 measurementMgr_mlmeResultCB, 
												 pMeasurementMgr);

 		templateStruct.pTemplate = (UINT8 *) &probeReqTemplate;
		templateStruct.templateType = PROBE_REQUEST_TEMPLATE;
		broadcastSSID.len = 0;

		WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG,
				("%s: Sending probe request template...\n", __FUNCTION__));

        buildProbeReqTemplate(pMeasurementMgr->hSiteMgr, &templateStruct, &broadcastSSID, request.band);
		whalCtrl_SetTemplate(pMeasurementMgr->hHalCtrl, &templateStruct);
	}

	/* Check if the maximum time to wait for the measurement request to */
	/* finish has already passed */
	timePassed = os_timeStampMs(pMeasurementMgr->hOs) - pMeasurementMgr->currentRequestStartTime;
	if (timePassed > MSR_START_MAX_DELAY)
	{
		WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG,
				("%s: Maximum delay to perform measurement operation has passed (%d / %d)\n", __FUNCTION__,
						MSR_START_MAX_DELAY, (os_timeStampMs(pMeasurementMgr->hOs) - pMeasurementMgr->currentRequestStartTime)));

		pMeasurementMgr->buildRejectReport(pMeasurementMgr, pRequestArr, numOfRequestsInParallel, MSR_REJECT_MAX_DELAY_PASSED);

        return measurementMgrSM_event((UINT8 *) &(pMeasurementMgr->currentState), 
				MEASUREMENTMGR_EVENT_COMPLETE, pMeasurementMgr);  
	}

	/* Yalla, start measuring */
	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG,
			("%s: Finished preparing request. Handing over to MeasurementSRV...\n", __FUNCTION__));

	MacServices_measurementSRV_startMeasurement(pMeasurementMgr->hMacServices,
												&request, 
												MSR_START_MAX_DELAY - timePassed,
												NULL, NULL,
												measurementMgr_MeasurementCompleteCB, pMeasurementMgr);

	return OK;
}






/********************************************************************************/
/*                          MEASURING State Actions                             */
/********************************************************************************/


static TI_STATUS measurementMgrSM_acDisconnected_fromMeasuring(void * pData)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;

    /* resume the self test */
    healthMonitor_resumePeriodicTest(pMeasurementMgr->hHealthMonitor);

    /* stop receiving from the MLME */
    mlmeParser_unregisterForBeaconAndProbeResp(pMeasurementMgr->hMlme);

    /* release the SCR */
	scr_clientComplete(pMeasurementMgr->hScr, SCR_CID_EXC_MEASURE);

    /* Clear Measurement fields */
    measurementMgrSM_resetParams(pMeasurementMgr);
		
	pMeasurementMgr->Connected = FALSE;

	return OK;
}



static TI_STATUS measurementMgrSM_acDisable_fromMeasuring(void * pData)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;

    /* resume the self test */
    healthMonitor_resumePeriodicTest(pMeasurementMgr->hHealthMonitor);

    /* stop receiving from the MLME */
    mlmeParser_unregisterForBeaconAndProbeResp(pMeasurementMgr->hMlme);

    /* release the SCR */
	scr_clientComplete(pMeasurementMgr->hScr, SCR_CID_EXC_MEASURE);

    /* Clear Measurement fields */
    measurementMgrSM_resetParams(pMeasurementMgr);
		
	pMeasurementMgr->Enabled = FALSE;

    return OK;    
}



static TI_STATUS measurementMgrSM_acFrameReceived_fromMeasuring(void * pData)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;

    /* resume the self test */
    healthMonitor_resumePeriodicTest(pMeasurementMgr->hHealthMonitor);

    /* stop receiving from the MLME */
    mlmeParser_unregisterForBeaconAndProbeResp(pMeasurementMgr->hMlme);

    /* release the SCR */
	scr_clientComplete(pMeasurementMgr->hScr, SCR_CID_EXC_MEASURE);

    /* Clear Measurement fields */
    measurementMgrSM_resetParams(pMeasurementMgr);
           	
	/* Process New Frame */
	return measurementMgrSM_acFrameReceived_fromIdle(pData);
}



static TI_STATUS measurementMgrSM_acAbort_fromMeasuring(void * pData)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;

    /* resume the self test */
    healthMonitor_resumePeriodicTest(pMeasurementMgr->hHealthMonitor);

    /* stop receiving from the MLME */
    mlmeParser_unregisterForBeaconAndProbeResp(pMeasurementMgr->hMlme);

    /* release the SCR */
	scr_clientComplete(pMeasurementMgr->hScr, SCR_CID_EXC_MEASURE);

    /* Clear Measurement fields */
    measurementMgrSM_resetParams(pMeasurementMgr);
		
    return OK;
}



/**
 * Called when we finished a measurement request.
 * 
 * @date 05-Jan-2006
 */
static TI_STATUS measurementMgrSM_acMeasurementComplete(void * pData)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;
	requestHandler_t * pRequestH = (requestHandler_t *) pMeasurementMgr->hRequestH;

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG,
			("%s: Completing measurement operation and resuming normal behavior\n", __FUNCTION__));

	/* advance the activeRequestID variable to get rid of the */
	/* measurement requests we've already executed */
	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG,
			("%s: Increasing activeRequestID from %d to %d.\n", __FUNCTION__, pRequestH->activeRequestID, pRequestH->activeRequestID + pMeasurementMgr->currentNumOfRequestsInParallel));

	pRequestH->activeRequestID += pMeasurementMgr->currentNumOfRequestsInParallel;

    /* resume the self test */
    healthMonitor_resumePeriodicTest(pMeasurementMgr->hHealthMonitor);

    /* stop receiving from the MLME */
    mlmeParser_unregisterForBeaconAndProbeResp(pMeasurementMgr->hMlme);

    /* release the SCR */
	scr_clientComplete(pMeasurementMgr->hScr, SCR_CID_EXC_MEASURE);

	/* Process New Frame */
	return measurementMgr_activateNextRequest(pData);
}



/**
 * Called when a firmware reset has been detected.
 * 
 * @date 05-Jan-2006
 */
static TI_STATUS measurementMgrSM_acFirmwareReset(void * pData)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;

    WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
                ("%s: Firmware Reset!!\n", __FUNCTION__));

	/* TODO */
	
    return OK;
}







/********************************************************************************/
/*						Miscellaneous State Actions								*/
/********************************************************************************/

/**
 * Called when an unexpected event has been triggered.
 * 
 * @date 05-Jan-2006
 */
static TI_STATUS measurementMgrSM_acUnexpected(void * pData)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;

    WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
                ("%s: Entered when state is %s\n", __FUNCTION__, measurementMgr_stateDesc[pMeasurementMgr->currentState]));

	return OK;
}

/**
 * A do nothing action.
 * 
 * @date 05-Jan-2006
 */
static TI_STATUS measurementMgrSM_acNop(void * pData)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) pData;

    WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
                ("%s: Entered when state is %s\n", __FUNCTION__, measurementMgr_stateDesc[pMeasurementMgr->currentState]));

	return OK;
}








/********************************************************************************/
/*						Internal Functions Prototypes							*/
/********************************************************************************/



static void measurementMgrSM_resetParams(measurementMgr_t *pMeasurementMgr)
{  
	/* clear the waiting requests */
	requestHandler_clearRequests(pMeasurementMgr->hRequestH);	

	/* clearing reports data base */
#ifdef EXC_MODULE_INCLUDED
	os_memoryZero(pMeasurementMgr->hOs,&(pMeasurementMgr->excFrameReport),
			sizeof(RM_report_frame_t));
#endif
    os_memoryZero(pMeasurementMgr->hOs,&(pMeasurementMgr->dot11hFrameReport),
			sizeof(MeasurementReportFrame_t));

	pMeasurementMgr->frameLength = 0;
	pMeasurementMgr->nextEmptySpaceInReport = 0;
	pMeasurementMgr->measuredChannelID = 0;
	pMeasurementMgr->currentFrameType = MSR_FRAME_TYPE_NO_ACTIVE;
}
