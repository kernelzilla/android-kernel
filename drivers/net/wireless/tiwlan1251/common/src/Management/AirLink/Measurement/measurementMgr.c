
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

/***************************************************************************/
/*																		   */
/*	  MODULE:	measurementMgr.c										   */
/*    PURPOSE:	measurement Manager module file							   */
/*																		   */
/***************************************************************************/




#include "measurementMgr.h"
#include "regulatoryDomainApi.h"
#include "healthMonitor.h"
#include "siteMgrApi.h"
#include "utils.h"
#include "TrafficMonitorAPI.h"
#include "smeApi.h"
#ifdef EXC_MODULE_INCLUDED
 #include "excTSMngr.h"
#endif



/* allocation vector */
#define MEASUREMENT_INIT_BIT								(1)
#define MEASUREMENT_ACTIVATION_DELAY_TIMER_INIT_BIT			(2)
#define MEASUREMENT_SM_INIT_BIT                             (3)
#define MEASUREMENT_REQUEST_HANDLER_SUB_MODULE_INIT_BIT		(4)


/* default measurement parameters */
#define MEASUREMENT_CAPABILITIES_NONE					0x00
#define MEASUREMENT_CAPABILITIES_DOT11H					0x01
#define MEASUREMENT_CAPABILITIES_EXC_RM					0x02


#define MEASUREMENT_BEACON_INTERVAL_IN_MICRO_SEC		1024
#define MEASUREMENT_MSEC_IN_MICRO                       1000






/********************************************************************************/
/*						Internal functions prototypes.							*/
/********************************************************************************/

static void measurementMgr_releaseModule(measurementMgr_t * pMeasurementMgr, UINT32 initVec);

static BOOL measurementMgr_isTrafficIntensityHigherThanThreshold(measurementMgr_t * pMeasurementMgr);

static BOOL	measurementMgr_isRequestValid(TI_HANDLE hMeasurementMgr, MeasurementRequest_t *pRequestArr[], UINT8 numOfRequest);

static void	measurementMgr_uponActivationDelayTimeout(TI_HANDLE Context);







/********************************************************************************/
/*						Interface functions Implementation.						*/
/********************************************************************************/


/**
 * Creates the Measurement Manager moodule.
 * 
 * @param hOs A handle to the OS object.
 *
 * @date 16-Dec-2005
 */
TI_HANDLE measurementMgr_create(TI_HANDLE hOs)
{
	measurementMgr_t * pMeasurementMgr = NULL;
	UINT32 initVec = 0;
    TI_STATUS status;

	/* allocating the MeasurementMgr object */
	pMeasurementMgr = os_memoryAlloc(hOs, sizeof(measurementMgr_t));

    if (pMeasurementMgr == NULL)
		return NULL;

    os_memoryZero(hOs, pMeasurementMgr, sizeof(measurementMgr_t));
    pMeasurementMgr->hOs = hOs;

	initVec |= (1 << MEASUREMENT_INIT_BIT);

	
	/* allocating the measurement Activation Delay timer */
	pMeasurementMgr->pActivationDelayTimer = os_timerCreate(hOs, measurementMgr_uponActivationDelayTimeout, 
                                                         pMeasurementMgr);
	if (pMeasurementMgr->pActivationDelayTimer == NULL)
	{
		measurementMgr_releaseModule(pMeasurementMgr, initVec);
		return NULL;
	}

	initVec |= (1<<MEASUREMENT_ACTIVATION_DELAY_TIMER_INIT_BIT);

#ifdef EXC_MODULE_INCLUDED	
	if ((pMeasurementMgr->pTsMetricsReportTimer[QOS_AC_BE] = 
		os_timerCreate(hOs, measurementMgr_sendTSMReport_AC_BE, pMeasurementMgr)) == NULL)
	{
		measurementMgr_releaseModule(pMeasurementMgr, initVec);
		return NULL;
	}

	if ((pMeasurementMgr->pTsMetricsReportTimer[QOS_AC_BK] = 
		os_timerCreate(hOs, measurementMgr_sendTSMReport_AC_BK, pMeasurementMgr)) == NULL)
	{
		measurementMgr_releaseModule(pMeasurementMgr, initVec);
		return NULL;
	}
	if ((pMeasurementMgr->pTsMetricsReportTimer[QOS_AC_VI] = 
		os_timerCreate(hOs, measurementMgr_sendTSMReport_AC_VI, pMeasurementMgr)) == NULL)
	{
		measurementMgr_releaseModule(pMeasurementMgr, initVec);
		return NULL;
	}
	if ((pMeasurementMgr->pTsMetricsReportTimer[QOS_AC_VO] = 
		os_timerCreate(hOs, measurementMgr_sendTSMReport_AC_VO, pMeasurementMgr)) == NULL)
	{
		measurementMgr_releaseModule(pMeasurementMgr, initVec);
		return NULL;
	}	
#endif

    /* creating the Measurement SM */
    status = fsm_Create(pMeasurementMgr->hOs, &(pMeasurementMgr->pMeasurementMgrSm), 
                        MEASUREMENTMGR_NUM_STATES , MEASUREMENTMGR_NUM_EVENTS);
	if(status != OK)
	{
		measurementMgr_releaseModule(pMeasurementMgr, initVec);
		return NULL;
	}

	initVec |= (1<<MEASUREMENT_SM_INIT_BIT);
   
	

	/* creating the sub modules of measurement module */
	
	/* creating Request Handler sub module */
	if( (pMeasurementMgr->hRequestH = requestHandler_create(hOs)) == NULL)
	{
		measurementMgr_releaseModule(pMeasurementMgr, initVec);
		return NULL;
	}
	
	initVec |= (1<<MEASUREMENT_REQUEST_HANDLER_SUB_MODULE_INIT_BIT);


    return(pMeasurementMgr);
}





/**
 * Configures the Measurement Manager module.
 * 
 * @param hXXXX Handles to other modules the Measurement Manager needs.
 * @param pMeasurementInitParams A pointer to the measurement init parameters.
 * 
 * @date 16-Dec-2005
 */
TI_STATUS measurementMgr_config(TI_HANDLE 	hMeasurementMgr,
								TI_HANDLE	hMacServices,
								TI_HANDLE	hRegulatoryDomain,
								TI_HANDLE	hExcMngr,
								TI_HANDLE	hSiteMgr,
								TI_HANDLE	hHalCtrl,
								TI_HANDLE	hMlme,
                                TI_HANDLE	hTrafficMonitor,
								TI_HANDLE	hReport,
								TI_HANDLE	hOs,
                                TI_HANDLE	hScr,
                                TI_HANDLE	hHealthMonitor,
								TI_HANDLE	hApConn,
								TI_HANDLE	hTx,
								measurementInitParams_t * pMeasurementInitParams)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;
	paramInfo_t param;
    TI_STATUS status;  
#ifdef EXC_MODULE_INCLUDED
	UINT32 currAC;
#endif
	
	/* Init Handlers */
	pMeasurementMgr->hMacServices		= hMacServices;
	pMeasurementMgr->hRegulatoryDomain	= hRegulatoryDomain;
	pMeasurementMgr->hExcMngr			= hExcMngr;
	pMeasurementMgr->hSiteMgr			= hSiteMgr;
	pMeasurementMgr->hHalCtrl			= hHalCtrl;
	pMeasurementMgr->hMlme				= hMlme;
    pMeasurementMgr->hTrafficMonitor	= hTrafficMonitor;
	pMeasurementMgr->hReport			= hReport;
	pMeasurementMgr->hOs				= hOs;
    pMeasurementMgr->hScr				= hScr;
    pMeasurementMgr->hHealthMonitor		= hHealthMonitor;
	pMeasurementMgr->hApConn			= hApConn;
	pMeasurementMgr->hTx				= hTx;

    /* initialize variables to default values */
    pMeasurementMgr->Enabled = TRUE;
	pMeasurementMgr->Connected = FALSE;
	pMeasurementMgr->Capabilities = MEASUREMENT_CAPABILITIES_NONE;
	pMeasurementMgr->Mode = MSR_MODE_NONE;

    /* Getting management capability status */
	param.paramType = REGULATORY_DOMAIN_MANAGEMENT_CAPABILITY_ENABLED_PARAM;
	regulatoryDomain_getParam(pMeasurementMgr->hRegulatoryDomain, &param);
	if (param.content.spectrumManagementEnabled)
	{
		pMeasurementMgr->Capabilities |= MEASUREMENT_CAPABILITIES_DOT11H;
	}
    
#ifdef EXC_MODULE_INCLUDED
    /* Check in the Registry if the station supports EXC RM */
    if (pMeasurementInitParams->excEnabled == EXC_MODE_ENABLED)
    {
        pMeasurementMgr->Capabilities |= MEASUREMENT_CAPABILITIES_EXC_RM;
    }
#endif

    /* Init Functions */
    pMeasurementMgr->parserFrameReq = NULL;
    pMeasurementMgr->isTypeValid = NULL;
    pMeasurementMgr->buildReport = NULL;
    pMeasurementMgr->buildRejectReport = NULL;
    pMeasurementMgr->sendReportAndCleanObj = NULL;
    
    /* initialize variables */	
    pMeasurementMgr->currentState = MEASUREMENTMGR_STATE_IDLE;
    pMeasurementMgr->isModuleRegistered = FALSE;
	pMeasurementMgr->currentFrameType = MSR_FRAME_TYPE_NO_ACTIVE;
    pMeasurementMgr->measuredChannelID = 0;
	pMeasurementMgr->currentNumOfRequestsInParallel = 0;
	
    pMeasurementMgr->trafficIntensityThreshold = pMeasurementInitParams->trafficIntensityThreshold;
    pMeasurementMgr->maxDurationOnNonServingChannel = pMeasurementInitParams->maxDurationOnNonServingChannel;

    /* config sub modules */
	RequestHandler_config(pMeasurementMgr->hRequestH, hReport, hOs);

    /* Register to the SCR module */
    scr_registerClientCB(pMeasurementMgr->hScr, SCR_CID_EXC_MEASURE, measurementMgr_scrResponseCB, pMeasurementMgr);

#ifdef EXC_MODULE_INCLUDED
	for (currAC = 0; currAC < MAX_NUM_OF_AC; currAC++)
	{
		pMeasurementMgr->isTsMetricsEnabled[currAC] = FALSE;
		os_timerStop(pMeasurementMgr->hOs, pMeasurementMgr->pTsMetricsReportTimer[currAC]);
	}
#endif

	status = measurementMgrSM_config(hMeasurementMgr);   

	if(status == OK)
    {
        WLAN_REPORT_INIT(hReport, MEASUREMENT_MNGR_MODULE_LOG,  
				("%s: Measurement Manager configured successfully\n", __FUNCTION__));
    }
    else
    {
        WLAN_REPORT_ERROR(hReport, MEASUREMENT_MNGR_MODULE_LOG,  
				("%s: Measurement Manager configuration failed\n", __FUNCTION__));
    }

	return status;
}





/**
 * Sets the specified Measurement Manager parameter.
 *
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 * @param pParam The parameter to set.
 * 
 * @date 16-Dec-2005
 */
TI_STATUS measurementMgr_setParam(TI_HANDLE hMeasurementMgr, paramInfo_t * pParam)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;

	switch (pParam->paramType)
	{
		case MEASUREMENT_ENABLE_DISABLE_PARAM:
		{
			WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
					("%s: MEASUREMENT_ENABLE_DISABLE_PARAM <- %d\n", __FUNCTION__, pParam->content.measurementEnableDisableStatus));

			if (pParam->content.measurementEnableDisableStatus)
			{
				measurementMgr_enable(pMeasurementMgr);
			}
			else
			{
				measurementMgr_disable(pMeasurementMgr);
			}

			break;
		}

		case MEASUREMENT_TRAFFIC_THRESHOLD_PARAM:
		{
			if ((pParam->content.measurementTrafficThreshold >= MEASUREMENT_TRAFFIC_THRSHLD_MIN) &&
				(pParam->content.measurementTrafficThreshold <= MEASUREMENT_TRAFFIC_THRSHLD_MAX))
			{
				WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
						("%s: MEASUREMENT_TRAFFIC_THRESHOLD_PARAM <- %d\n", __FUNCTION__, pParam->content.measurementTrafficThreshold));

				pMeasurementMgr->trafficIntensityThreshold = pParam->content.measurementTrafficThreshold;
			}
			else
			{
				WLAN_REPORT_ERROR(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
						("%s: Invalid value for MEASUREMENT_TRAFFIC_THRESHOLD_PARAM (%d)\n", __FUNCTION__, pParam->content.measurementTrafficThreshold));
			}
        
			break;
		}

		
		case MEASUREMENT_MAX_DURATION_PARAM:
		{
			WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
					("%s: MEASUREMENT_MAX_DURATION_PARAM <- %d\n", __FUNCTION__, pParam->content.measurementMaxDuration));

			pMeasurementMgr->maxDurationOnNonServingChannel = pParam->content.measurementMaxDuration;

			break;
		}
        

		default:
		{
			WLAN_REPORT_ERROR(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
					("%s: Specified parameter is not supported (%d)\n", __FUNCTION__, pParam->paramType));

			return PARAM_NOT_SUPPORTED;
		}

	}
	
	return OK;
}





/**
 * Gets the specified parameter from the Measurement Manager.
 *
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 * @param pParam The parameter to get.
 * 
 * @date 16-Dec-2005
 */
TI_STATUS measurementMgr_getParam(TI_HANDLE hMeasurementMgr, paramInfo_t * pParam)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;

	switch(pParam->paramType)
	{

		case MEASUREMENT_GET_STATUS_PARAM:
		{
			WLAN_OS_REPORT(("%s: \n\n", __FUNCTION__));
			WLAN_OS_REPORT(("MeasurementMgr Status Report:\n\n"));

			WLAN_OS_REPORT(("Current State: %d\n\n", pMeasurementMgr->currentState));

			WLAN_OS_REPORT(("Connected: %d\n", pMeasurementMgr->Connected));
			WLAN_OS_REPORT(("Enabled: %d\n\n", pMeasurementMgr->Enabled));

			WLAN_OS_REPORT(("Mode: %d\n", pMeasurementMgr->Mode));
			WLAN_OS_REPORT(("Capabilities: %d\n\n", pMeasurementMgr->Capabilities));

			WLAN_OS_REPORT(("current Frame Type: %d\n", pMeasurementMgr->currentFrameType));
			WLAN_OS_REPORT(("Measured Channel: %d\n", pMeasurementMgr->measuredChannelID));
			WLAN_OS_REPORT(("Serving Channel: %d\n", pMeasurementMgr->servingChannelID));
			WLAN_OS_REPORT(("Traffic Intensity Threshold: %d\n", pMeasurementMgr->trafficIntensityThreshold));
			WLAN_OS_REPORT(("Max Duration on Nonserving Channel: %d\n", pMeasurementMgr->maxDurationOnNonServingChannel));

			break;
		}
		

		default:
		{
			WLAN_REPORT_ERROR(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
					("%s: Specified parameter is not supported (%d)\n", __FUNCTION__, pParam->paramType));

			return PARAM_NOT_SUPPORTED;
		}

	}

	return OK;
}






/**
 * Signals the Measurement Manager that the STA is connected.
 *
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 * 
 * @date 16-Dec-2005
 */
TI_STATUS measurementMgr_connected(TI_HANDLE hMeasurementMgr)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;

    /* checking if measurement is enabled */
	if (pMeasurementMgr->Mode == MSR_MODE_NONE)
		return OK;

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: MeasurementMgr set to connected.\n", __FUNCTION__));

    return measurementMgrSM_event((UINT8 *) &(pMeasurementMgr->currentState), 
                               MEASUREMENTMGR_EVENT_CONNECTED, pMeasurementMgr);
}





/**
 * Signals the Measurement Manager that the STA is disconnected.
 *
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 * 
 * @date 16-Dec-2005
 */
TI_STATUS measurementMgr_disconnected(TI_HANDLE hMeasurementMgr)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: MeasurementMgr set to disconnected.\n", __FUNCTION__));

	return measurementMgrSM_event((UINT8 *) &(pMeasurementMgr->currentState), 
								MEASUREMENTMGR_EVENT_DISCONNECTED, pMeasurementMgr);
}




/**
 * Enables the Measurement Manager module.
 * 
 * @date 10-Jan-2006
 */
TI_STATUS measurementMgr_enable(TI_HANDLE hMeasurementMgr)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: MeasurementMgr set to enabled.\n", __FUNCTION__));

	return measurementMgrSM_event((UINT8 *) &(pMeasurementMgr->currentState), 
								MEASUREMENTMGR_EVENT_ENABLE, pMeasurementMgr);
}





/**
 * Disables the Measurement Manager module.
 * 
 * @date 10-Jan-2006
 */
TI_STATUS measurementMgr_disable(TI_HANDLE hMeasurementMgr)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: MeasurementMgr set to disabled.\n", __FUNCTION__));

	return measurementMgrSM_event((UINT8 *) &(pMeasurementMgr->currentState), 
								MEASUREMENTMGR_EVENT_DISABLE, pMeasurementMgr);
}





/**
 * Destroys the Measurement Manager module.
 * 
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 * 
 * @date 16-Dec-2005
 */
TI_STATUS measurementMgr_destroy(TI_HANDLE hMeasurementMgr)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;
	UINT32 initVec;

	if (pMeasurementMgr == NULL)
		return OK;

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: MeasurementMgr is being destroyed\n", __FUNCTION__));

	initVec = 0xFFFF;	/* release everything */

	measurementMgr_releaseModule(pMeasurementMgr, initVec);

	return OK;
}






/**
 * Sets the Measurement Mode.
 * 
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 * @param capabilities The AP capabilities.
 * @param pIeBuffer Pointer to the list of IEs.
 * @param length Length of the IE list.
 * 
 * @date 16-Dec-2005
 */
TI_STATUS measurementMgr_setMeasurementMode(TI_HANDLE hMeasurementMgr, UINT16 capabilities, 
                                         UINT8 * pIeBuffer, UINT16 length)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;

	/*
	 * 11h Measurement is not supported in the current version.
	 */
/*	if( (pMeasurementMgr->Capabilities & MEASUREMENT_CAPABILITIES_DOT11H) &&
        (capabilities & DOT11_SPECTRUM_MANAGEMENT) )
    {
        pMeasurementMgr->Mode = MSR_MODE_SPECTRUM_MANAGEMENT;
    }
    else
    {
*/
#ifdef EXC_MODULE_INCLUDED

        if(pMeasurementMgr->Capabilities & MEASUREMENT_CAPABILITIES_EXC_RM)
        {
                    pMeasurementMgr->Mode = MSR_MODE_EXC;
        }
        else
#endif
        {
            pMeasurementMgr->Mode = MSR_MODE_NONE;
        }


	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: MeasurementMgr mode changed to: %d\n", __FUNCTION__, pMeasurementMgr->Mode));
    
	return OK;
}






/**
 * Called when a frame with type measurement request is received.
 * 
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 * @param frameType The frame type.
 * @param dataLen The length of the frame.
 * @param pData A pointer to the frame's content.
 * 
 * @date 16-Dec-2005
 */
TI_STATUS measurementMgr_receiveFrameRequest(TI_HANDLE hMeasurementMgr,
											measurement_frameType_e frameType,
											INT32 dataLen,
											UINT8 * pData)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;

    measurement_frameRequest_t * frame = &(pMeasurementMgr->newFrameRequest);    
	UINT16 currentFrameToken;
	
	/* checking if measurement is enabled */
	if (pMeasurementMgr->Mode == MSR_MODE_NONE)
		return NOK;

	/* ignore broadcast/multicast request if unicast request is active */
	if (frameType != MSR_FRAME_TYPE_UNICAST && pMeasurementMgr->currentFrameType == MSR_FRAME_TYPE_UNICAST)
    {
        WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
	           ("%s: Broadcast/Multicast measurement frame has been ignored\n", __FUNCTION__));

		return MEASUREMENT_REQUEST_IGNORED;
    }

	/* ignore broadcast request if multicast request is active */
	if (frameType == MSR_FRAME_TYPE_BROADCAST && pMeasurementMgr->currentFrameType == MSR_FRAME_TYPE_MULTICAST)
    {
        WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
		        ("%s: Broadcast measurement frame has been ignored\n", __FUNCTION__));
        
		return MEASUREMENT_REQUEST_IGNORED;
    }

    WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Measurement frame received\n", __FUNCTION__));

    /* Parsing the Frame Request Header */
    pMeasurementMgr->parserFrameReq(hMeasurementMgr, pData, dataLen, 
										frame);

    frame->frameType = frameType;

	/* checking if the received token frame is the same as the one that is being processed */
	if ((requestHandler_getFrameToken(pMeasurementMgr->hRequestH, &currentFrameToken) == OK)
		&& (currentFrameToken == frame->hdr->dialogToken))
	{
		os_memoryZero(pMeasurementMgr->hOs, &pMeasurementMgr->newFrameRequest, 
                      sizeof(measurement_frameRequest_t));

		WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG,
				("%s: Measurement frame token %d is identical to current frame token - ignoring frame\n", __FUNCTION__, currentFrameToken));

		return MEASUREMENT_REQUEST_IGNORED;
	}

    WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Measurement frame token is %d\n", __FUNCTION__, frame->hdr->dialogToken));

    /* Frame is Received for processing */
    return measurementMgrSM_event((UINT8 *) &(pMeasurementMgr->currentState), 
                               MEASUREMENTMGR_EVENT_FRAME_RECV, pMeasurementMgr);
}





/**
 * Activates the next measurement request.
 * 
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 * 
 * @date 16-Dec-2005
 */
TI_STATUS measurementMgr_activateNextRequest(TI_HANDLE hMeasurementMgr)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;
	requestHandler_t * pRequestH = (requestHandler_t *) pMeasurementMgr->hRequestH;
	MeasurementRequest_t * pRequestArr[MAX_NUM_REQ];
	UINT8 numOfRequestsInParallel = 0;
	BOOL valid;
	UINT8 index;

	/* Keep note of the time we started processing the request. this will be used */
	/* to give the measurementSRV a time frame to perform the measurement operation */
	pMeasurementMgr->currentRequestStartTime = os_timeStampMs(pMeasurementMgr->hOs);

    WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Timer started at %d, we have 20ms to begin measurement...\n", __FUNCTION__, 
					pMeasurementMgr->currentRequestStartTime));

    WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Looking for a valid request\n", __FUNCTION__));

	do
	{
		TI_STATUS status;

		if (numOfRequestsInParallel != 0)
		{
			WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
					("%s: Changing activeRequestID from %d to %d, and numOfWaitingRequests from %d to %d.\n", __FUNCTION__,
							pRequestH->activeRequestID, pRequestH->activeRequestID + numOfRequestsInParallel, pRequestH->numOfWaitingRequests, pRequestH->numOfWaitingRequests - numOfRequestsInParallel));
		}

		pRequestH->activeRequestID += numOfRequestsInParallel;
        pRequestH->numOfWaitingRequests -= numOfRequestsInParallel;

        for (index = 0; index < MAX_NUM_REQ; index++)
        {
            pRequestArr[index] = NULL;
        }
		numOfRequestsInParallel = 0;

		/* Getting the next request/requests from the request handler */
		status = requestHandler_getNextReq(pMeasurementMgr->hRequestH, FALSE, pRequestArr, 
										   &numOfRequestsInParallel);
		
		/* Checking if there are no waiting requests */
		if (status != OK)
		{
            WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
				    ("%s: There are no waiting requests in the queue\n", __FUNCTION__));

			return measurementMgrSM_event((UINT8 *) &(pMeasurementMgr->currentState), 
                               MEASUREMENTMGR_EVENT_SEND_REPORT, pMeasurementMgr);
		}

        /* Checking validity of request/s */
		valid = measurementMgr_isRequestValid(pMeasurementMgr, pRequestArr, 
								numOfRequestsInParallel);

        /* Checking if the current request is Beacon Table */
        if( (numOfRequestsInParallel == 1) && 
            (pRequestArr[0]->Type == MSR_TYPE_BEACON_MEASUREMENT) &&
            (pRequestArr[0]->ScanMode == MSR_SCAN_MODE_BEACON_TABLE) )
        {
            WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
					("%s: Received Beacon Table request, building a report for it and continuing\n", __FUNCTION__));

            pMeasurementMgr->buildReport(hMeasurementMgr, *(pRequestArr[0]), NULL);
            valid = FALSE; /* In order to get the next request/s*/
        }
		
	} while (valid == FALSE);
    
	
	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Request(s) for activation:\n", __FUNCTION__));

	for (index = 0; index < numOfRequestsInParallel; index++)
	{
		WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
				("%s: \n\nRequest #%d:\n Type: %d\n Measured Channel: %d (Serving Channel: %d)\n Scan Mode: %d\n Duration: %d\n\n", __FUNCTION__, 
						index+1, pRequestArr[index]->Type, pRequestArr[index]->channelNumber,
						pMeasurementMgr->servingChannelID, pRequestArr[index]->ScanMode, pRequestArr[index]->DurationTime));
	}

	/* Ignore requests if traffic intensity is high */
	if (measurementMgr_isTrafficIntensityHigherThanThreshold(pMeasurementMgr) == TRUE)
	{
        WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
				("%s: Traffic intensity too high, giving up...\n", __FUNCTION__));

		measurementMgr_rejectPendingRequests(pMeasurementMgr, MSR_REJECT_TRAFFIC_INTENSITY_TOO_HIGH);

		return measurementMgrSM_event((UINT8 *) &(pMeasurementMgr->currentState), 
                               MEASUREMENTMGR_EVENT_SEND_REPORT, pMeasurementMgr);
	}

    WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Request is Valid, about to start\n", __FUNCTION__)); 
    
    pMeasurementMgr->measuredChannelID = pRequestArr[0]->channelNumber;
  
    /* Request resource from the SCR */
    return measurementMgrSM_event((UINT8 *) &(pMeasurementMgr->currentState), 
        MEASUREMENTMGR_EVENT_REQUEST_SCR, pMeasurementMgr);    
}	



void measurementMgr_rejectPendingRequests(TI_HANDLE hMeasurementMgr, measurement_rejectReason_e rejectReason)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;
	requestHandler_t * pRequestH = (requestHandler_t *) pMeasurementMgr->hRequestH;
    MeasurementRequest_t * pRequestArr[MAX_NUM_REQ];
	UINT8 numOfRequestsInParallel;

	/* reject all pending measurement requests */
	while (requestHandler_getNextReq(pMeasurementMgr->hRequestH, TRUE, 
				pRequestArr, &numOfRequestsInParallel) == OK)
	{
		WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
					("%s: Rejecting pending request (activeRequestID = %d)...\n", __FUNCTION__, pRequestH->activeRequestID));

        pMeasurementMgr->buildRejectReport(pMeasurementMgr, pRequestArr,
				numOfRequestsInParallel, rejectReason);

		pRequestH->activeRequestID += numOfRequestsInParallel;
	}
}





/********************************************************************************/
/*						Callback functions Implementation.						*/
/********************************************************************************/


/**
 * The callback called by the MeasurementSRV module when then
 * measurement operation has ended.
 * 
 * @param clientObj A handle to the Measurement Manager module.
 * @param msrReply An array of replies sent by the MeasurementSRV module,
 * where each reply contains the result of a single measurement request.
 * 
 * @date 01-Jan-2006
 */
void measurementMgr_MeasurementCompleteCB(TI_HANDLE clientObj, measurement_reply_t * msrReply)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) clientObj;
    UINT8 index;

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG,
			("%s: Building reports for measurement requests\n", __FUNCTION__));

    /* build a report for each measurement request/reply pair */
	for (index = 0; index < msrReply->numberOfTypes; index++)
    {
        pMeasurementMgr->buildReport(pMeasurementMgr, *(pMeasurementMgr->currentRequest[index]), &msrReply->msrTypes[index]);
    }

    measurementMgrSM_event((UINT8 *) &(pMeasurementMgr->currentState), 
			MEASUREMENTMGR_EVENT_COMPLETE, pMeasurementMgr);
}


/**
 * The callback called when the SCR responds to the SCR request.
 * 
 * @param hClient A handle to the Measurement Manager module.
 * @param requestStatus The request's status
 * @param pendReason The reason of a PEND status.
 * 
 * @date 01-Jan-2006
 */
void measurementMgr_scrResponseCB(TI_HANDLE hClient, scr_clientRequestStatus_e requestStatus,
							   scr_pendReason_e pendReason )
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hClient;
	measurementMgrSM_Events event;

    WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: SCR callback entered\n", __FUNCTION__));	

    /* If the SM is in a state where it waits for the CB, status of RUN */
	/* results in the SM asking the measurementSRV to start measurement; */
	/* otherwise we got an ABORT or a PEND reason worse than the one we */
	/* got when calling the SCR, so the SM aborts the measurement */
    if (pMeasurementMgr->currentState == MEASUREMENTMGR_STATE_WAITING_FOR_SCR)
    {
		if (requestStatus == SCR_CRS_RUN)
		{
			WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
					("%s: Received SCR status RUN, running...\n", __FUNCTION__));

			event = MEASUREMENTMGR_EVENT_SCR_RUN;
		}
		else
		{
			if (requestStatus == SCR_CRS_PEND)
			{
				WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
						("%s: Received SCR status PEND with reason %d, aborting...\n", __FUNCTION__, pendReason));
			}
			else
			{
				WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
						("%s: Received SCR status ABORT/FW_RESET, aborting...\n", __FUNCTION__));
			}

			event = MEASUREMENTMGR_EVENT_ABORT;
		}
	}
	else
	{	
		/* This can only occur if FW reset occurs or when higher priority */
		/* client is running in all these cases indicate ABORT event */

		WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
				("%s: MeasurementMgrSM current state is %d (which isn't WAITING_FOR_SCR), aborting...\n", __FUNCTION__, pMeasurementMgr->currentState));

		event = MEASUREMENTMGR_EVENT_ABORT;
	}

	measurementMgrSM_event((UINT8 *) &(pMeasurementMgr->currentState), 
		event, pMeasurementMgr);
}






/**
 * The callback called by the MLME.
 * 
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 * 
 * @date 01-Jan-2006
 */
void measurementMgr_mlmeResultCB(TI_HANDLE hMeasurementMgr, macAddress_t * bssid, mlmeFrameInfo_t * frameInfo, 
								 Rx_attr_t * pRxAttr, UINT8 * buffer, UINT16 bufferLength)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;

	/* check whether the packet was received while the firmware was performing a beacon measurement */

	BOOL receivedWhileMeasuring = ((pRxAttr->packetInfo & RX_PACKET_FLAGS_MEASURMENT) == RX_PACKET_FLAGS_MEASURMENT);

    if (pMeasurementMgr == NULL || pRxAttr == NULL)
    {
        WLAN_REPORT_ERROR(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
				("%s: MLME callback called with NULL object\n", __FUNCTION__));	

        return;
    }

    WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: MLME callback entered\n", __FUNCTION__));

    /* save scan results in site manager */
    siteMgr_updateSite(pMeasurementMgr->hSiteMgr, bssid, frameInfo, pRxAttr->channel, (radioBand_e)pRxAttr->band, receivedWhileMeasuring);

    if (frameInfo->subType == PROBE_RESPONSE)
    {
        siteMgr_saveProbeRespBuffer(pMeasurementMgr->hSiteMgr, bssid, buffer, bufferLength);
    }
    else if (frameInfo->subType == BEACON)
    {
        siteMgr_saveBeaconBuffer(pMeasurementMgr->hSiteMgr, bssid, buffer, bufferLength);
    }
    else
    {
        WLAN_REPORT_ERROR(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
				("%s: Unknown frame subtype (%d)\n", __FUNCTION__, frameInfo->subType));	
    }

    WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: MLME Frame: rcvWhileMeasuring = %d, Subtype = %d, MAC = %x-%x-%x-%x-%x-%x, RSSI = %d\n", 
					__FUNCTION__, receivedWhileMeasuring, frameInfo->subType,
					bssid->addr[0], bssid->addr[1], bssid->addr[2],
					bssid->addr[3], bssid->addr[4], bssid->addr[5],
					pRxAttr->Rssi));
}








/********************************************************************************/
/*						Internal functions Implementation.						*/
/********************************************************************************/


/**
 * Releases the module's allocated objects according to the given init vector.
 * 
 * @param pMeasurementMgr A handle to the Measurement Manager module.
 * @param initVec The init vector with a bit set for each allocated object.
 * 
 * @date 01-Jan-2006
 */
static void measurementMgr_releaseModule(measurementMgr_t * pMeasurementMgr, UINT32 initVec)
{
#ifdef EXC_MODULE_INCLUDED
	UINT32 currAC;
#endif

    WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: InitVec = %d\n", __FUNCTION__, initVec));

	if (initVec & (1 << MEASUREMENT_ACTIVATION_DELAY_TIMER_INIT_BIT))
	{
		os_timerStop(pMeasurementMgr->hOs, pMeasurementMgr->pActivationDelayTimer);
		utils_nullTimerDestroy(pMeasurementMgr->hOs, pMeasurementMgr->pActivationDelayTimer);
#ifdef EXC_MODULE_INCLUDED
		for (currAC = 0; currAC < MAX_NUM_OF_AC; currAC++)
		{
			if (pMeasurementMgr->pTsMetricsReportTimer[currAC] != NULL)
			{
				os_timerStop(pMeasurementMgr->hOs, pMeasurementMgr->pTsMetricsReportTimer[currAC]);
				utils_nullTimerDestroy(pMeasurementMgr->hOs, pMeasurementMgr->pTsMetricsReportTimer[currAC]);
			}
		}
#endif
	}

    if (initVec & (1 << MEASUREMENT_SM_INIT_BIT))
		fsm_Unload(pMeasurementMgr->hOs, pMeasurementMgr->pMeasurementMgrSm);
    

	if (initVec & (1 << MEASUREMENT_REQUEST_HANDLER_SUB_MODULE_INIT_BIT))
		requestHandler_destroy(pMeasurementMgr->hRequestH);
	

	if (initVec & (1 << MEASUREMENT_INIT_BIT))
		utils_nullMemoryFree(pMeasurementMgr->hOs, pMeasurementMgr, sizeof(measurementMgr_t));
}





/**
 * Checks whether the traffic intensity, i.e. number of packets per seconds, is higher
 * than the preconfigured threshold.
 * 
 * @param pMeasurementMgr A handle to the Measurement Manager module.
 * 
 * @return True iff the traffic intensity is high
 * 
 * @date 01-Jan-2006
 */
static BOOL measurementMgr_isTrafficIntensityHigherThanThreshold(measurementMgr_t * pMeasurementMgr)
{
    BOOL trafficIntensityHigh = FALSE;
    int pcksPerSec;

    pcksPerSec = TrafficMonitor_GetFrameBandwidth(pMeasurementMgr->hTrafficMonitor);

    WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
	        ("%s: pcksPerSec = %d\n", __FUNCTION__, pcksPerSec));	
    
    if (pcksPerSec >= pMeasurementMgr->trafficIntensityThreshold)
        trafficIntensityHigh = TRUE;
    
    return trafficIntensityHigh;
}




/**
 * Checks whether the given measurement request is valid.
 * 
 * @param hMeasurementMgr A handle to the Measurement Manager module.
 * @param pRequestArr The measurement request.
 * @param numOfRequest Number of type requests
 * 
 * @return True iff the request is valid
 * 
 * @date 01-Jan-2006
 */
static BOOL	measurementMgr_isRequestValid(TI_HANDLE hMeasurementMgr, MeasurementRequest_t *pRequestArr[],
						   UINT8 numOfRequest)
{
    measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) hMeasurementMgr;
	UINT8			requestIndex;
	paramInfo_t		param;

	/* Checking validity of the measured channel number */
	param.content.channel = pRequestArr[0]->channelNumber;
	param.paramType = REGULATORY_DOMAIN_IS_CHANNEL_SUPPORTED;
	regulatoryDomain_getParam(pMeasurementMgr->hRegulatoryDomain, &param);
	if ( !param.content.bIsChannelSupprted  )
    {
        WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
	            ("%s: Request rejected due to invalid channel\n", __FUNCTION__));

        if (pMeasurementMgr->currentFrameType == MSR_FRAME_TYPE_UNICAST)
            pMeasurementMgr->buildRejectReport(pMeasurementMgr, pRequestArr, numOfRequest, 
									MSR_REJECT_INVALID_CHANNEL);

        return FALSE;
    }
    else
    {
        WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
	            ("%s: Request channel is Valid\n", __FUNCTION__));
    }
	
    WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
	        ("%s: Starting to check each request:\n", __FUNCTION__));

    /* Check Validity of each request */
	for (requestIndex = 0; requestIndex < numOfRequest; requestIndex++)
	{
	    WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
		        ("%s: Checking request #%d:\n", __FUNCTION__, requestIndex+1));

		/* Checking validity of the Request Type */
        if (pMeasurementMgr->isTypeValid(hMeasurementMgr, pRequestArr[requestIndex]->Type, 
				pRequestArr[requestIndex]->ScanMode) == FALSE)
        {
            WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
					("%s: Request rejected due to invalid measurement type of request #%d (type = %d)\n", __FUNCTION__, requestIndex+1, pRequestArr[requestIndex]->Type));
            
            if(pMeasurementMgr->currentFrameType == MSR_FRAME_TYPE_UNICAST)
                pMeasurementMgr->buildRejectReport(pMeasurementMgr, pRequestArr, numOfRequest, 
										MSR_REJECT_INVALID_MEASUREMENT_TYPE);

            return FALSE;
        }
        else
        {
            WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
					("%s: Measurement type of request #%d is supported (type = %d)\n", __FUNCTION__, requestIndex+1, pRequestArr[requestIndex]->Type));
        }

        /* For measurement types different than Beacon Table */
        if ((pRequestArr[requestIndex]->Type != MSR_TYPE_BEACON_MEASUREMENT) || 
            (pRequestArr[requestIndex]->ScanMode != MSR_SCAN_MODE_BEACON_TABLE))
		{
    		/* Checking Measurement request's duration only when request is on a non-serving channel */
            if (pMeasurementMgr->servingChannelID != pRequestArr[requestIndex]->channelNumber)
			{
				UINT8 dtimPeriod;
				UINT32 beaconInterval;
				UINT32 dtimDuration;


				/* Checking duration doesn't exceed given max duration */
				if (pRequestArr[requestIndex]->DurationTime > pMeasurementMgr->maxDurationOnNonServingChannel)
				{
					WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
							("%s: Request #%d rejected because duration exceeds maximum duration\n", __FUNCTION__, requestIndex+1));

					WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG,
							("%s: Duration = %d, MaxDurationOnNonServingChannel = %d\n", __FUNCTION__, 
									pRequestArr[requestIndex]->DurationTime,
									pMeasurementMgr->maxDurationOnNonServingChannel));
                
					if (pMeasurementMgr->currentFrameType == MSR_FRAME_TYPE_UNICAST)
						pMeasurementMgr->buildRejectReport(pMeasurementMgr, pRequestArr, numOfRequest, 
								MSR_REJECT_DURATION_EXCEED_MAX_DURATION);
                
					return FALSE;
				}
				else
				{
					WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
							("%s: Duration of request #%d doesn't exceed max duration\n", __FUNCTION__, requestIndex+1));
				}


				/* Checking DTIM */

				/* Getting the DTIM count */
				param.paramType = SITE_MGR_DTIM_PERIOD_PARAM;
				siteMgr_getParam(pMeasurementMgr->hSiteMgr, &param);
				dtimPeriod = param.content.siteMgrDtimPeriod;

				/* Getting the beacon Interval */
				param.paramType = SITE_MGR_BEACON_INTERVAL_PARAM;
				siteMgr_getParam(pMeasurementMgr->hSiteMgr, &param);
				beaconInterval = param.content.beaconInterval;

				dtimDuration = beaconInterval * MEASUREMENT_BEACON_INTERVAL_IN_MICRO_SEC/MEASUREMENT_MSEC_IN_MICRO*dtimPeriod;
				if (pRequestArr[requestIndex]->DurationTime > dtimDuration)
				{
					WLAN_REPORT_WARNING(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
							("%s: Request rejected due to DTIM overlap of request #%d\n", __FUNCTION__, requestIndex+1));
									
					WLAN_REPORT_WARNING(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
							("%s: Duration = %d, DTIM Duration = %d\n", __FUNCTION__, pRequestArr[requestIndex]->DurationTime, dtimDuration));
        
					if (pMeasurementMgr->currentFrameType == MSR_FRAME_TYPE_UNICAST)
						pMeasurementMgr->buildRejectReport(pMeasurementMgr, pRequestArr, numOfRequest, 
												MSR_REJECT_DTIM_OVERLAP);

					return FALSE;
				}
				else
				{
					WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
							("%s: DTIM of request #%d doesn't overlap\n", __FUNCTION__, requestIndex+1));
				}
	        }
		}
	}

	return TRUE;
}






/**
 * The callback called when the activation delay timer has ended.
 * 
 * @param Context A handle to the Measurement Manager module.
 * 
 * @date 01-Jan-2006
 */
static void	measurementMgr_uponActivationDelayTimeout(TI_HANDLE Context)
{
	measurementMgr_t * pMeasurementMgr = (measurementMgr_t *) Context;

	WLAN_REPORT_INFORMATION(pMeasurementMgr->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Activation delay timeout callback entered\n", __FUNCTION__));

    measurementMgr_activateNextRequest(Context);
}
