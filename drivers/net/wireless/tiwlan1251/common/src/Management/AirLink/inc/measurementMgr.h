
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
/*	  MODULE:	measurementMgr.h										   */
/*    PURPOSE:	measurement Manager module header file					   */
/*																		   */
/***************************************************************************/




#ifndef __MEASUREMENTMGR_H__
#define __MEASUREMENTMGR_H__


#include "report.h"
#include "paramOut.h"
#include "requestHandler.h"
#include "measurementMgrSM.h"
#ifdef EXC_MODULE_INCLUDED
 #include "excRMMngrParam.h"
#endif


/* Functions Pointers Definitions */
typedef TI_STATUS (*parserFrameReq_t)   (TI_HANDLE hMeasurementMgr, 
                                         UINT8 *pData, INT32 dataLen,
                                         measurement_frameRequest_t *frameReq);

typedef BOOL (*isTypeValid_t)           (TI_HANDLE hMeasurementMgr, 
                                         measurement_type_e type, 
                                         measurement_scanMode_e scanMode);

typedef TI_STATUS (*buildRejectReport_t) (TI_HANDLE hMeasurementMgr,
										  MeasurementRequest_t *pRequestArr[],
										  UINT8	numOfRequestsInParallel,
										  measurement_rejectReason_e rejectReason);

typedef TI_STATUS (*buildReport_t)		(TI_HANDLE hMeasurementMgr,
										 MeasurementRequest_t request,
										 measurement_typeReply_t * reply);

typedef TI_STATUS (*sendReportAndCleanObj_t)(TI_HANDLE hMeasurementMgr);



typedef struct 
{

    /* Timers */
	void *						pActivationDelayTimer;


    /* Methods */
    parserFrameReq_t            parserFrameReq;
    isTypeValid_t               isTypeValid;
    buildRejectReport_t			buildRejectReport;
	buildReport_t				buildReport;
	sendReportAndCleanObj_t		sendReportAndCleanObj;


    /* Data */
    BOOL						Enabled;
	BOOL						Connected;

	UINT8						servingChannelID;
	UINT8						measuredChannelID;

	measurement_mode_e			Mode;	
	UINT8						Capabilities;
	BOOL                        isModuleRegistered;

	UINT16						trafficIntensityThreshold;
    UINT16                      maxDurationOnNonServingChannel;


    /* State Machine Params */
    fsm_stateMachine_t *		pMeasurementMgrSm;
    measurementMgrSM_States		currentState;

	
    /* Report Frame Params */
#ifdef EXC_MODULE_INCLUDED
	RM_report_frame_t			excFrameReport;
#endif
	MeasurementReportFrame_t	dot11hFrameReport;
	UINT16						nextEmptySpaceInReport;
	UINT16						frameLength;


    /* Request Frame Params */
    MeasurementRequest_t *      currentRequest[MAX_NUM_REQ];
	UINT8						currentNumOfRequestsInParallel;
	measurement_frameType_e		currentFrameType;
	UINT32						currentRequestStartTime;
    measurement_frameRequest_t	newFrameRequest;


	/* EXC Traffic Stream Metrics measurement parameters */
	void 						*pTsMetricsReportTimer[MAX_NUM_OF_AC];
	BOOL 						isTsMetricsEnabled[MAX_NUM_OF_AC];

	/* Handles to other modules */
	TI_HANDLE					hRequestH;
	TI_HANDLE					hMacServices;
	TI_HANDLE					hRegulatoryDomain;
	TI_HANDLE					hExcMngr;
	TI_HANDLE					hSiteMgr;
	TI_HANDLE					hHalCtrl;
	TI_HANDLE					hMlme;
    TI_HANDLE                   hTrafficMonitor;
	TI_HANDLE					hReport;
	TI_HANDLE					hOs;
    TI_HANDLE                   hScr;
    TI_HANDLE                   hHealthMonitor;
    TI_HANDLE                   hApConn;
    TI_HANDLE                   hTx;
} measurementMgr_t;




TI_STATUS measurementMgr_activateNextRequest(TI_HANDLE pContext);




#endif /* __MEASUREMENTMGR_H__*/

