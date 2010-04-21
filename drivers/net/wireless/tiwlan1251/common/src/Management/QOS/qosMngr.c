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


/** \file qosMngr.c
 *  \brief QOS module interface
 *
 *  \see qosMngr.h
 */

/****************************************************************************************************/
/*																									*/
/*		MODULE:		qosMGr.c																	    */
/*		PURPOSE:	QOS module interface.												            */
/*                  This module handles the QOS manager configuration.	 							*/
/*																						 			*/
/****************************************************************************************************/
#include "report.h"
#include "osApi.h"
#include "paramOut.h"
#include "paramIn.h"
#include "utils.h"
#include "siteMgrApi.h"
#include "public_infoele.h"
#include "whalCtrl_api.h"
#include "qosMngr.h"
#include "qosMngr_API.h"
#include "qosMngr_QueueMapping.h"
#include "smeSmApi.h"
#include "EvHandler.h"
#ifdef EXC_MODULE_INCLUDED
#include "excMngr.h"
#include "excTSMngr.h"
#endif

static int NonQosTagToACTable[MAX_NUM_OF_802_1d_TAGS] = { NonQosTag0ToAC, NonQosTag1ToAC, NonQosTag2ToAC, NonQosTag3ToAC, NonQosTag4ToAC, NonQosTag5ToAC, NonQosTag6ToAC, NonQosTag7ToAC };

/* This structure is now shared between the qosMngr and TxData to allow PS_POLL voice delivery mode for NON_QOS AP */
int WMEQosTagToACTable[MAX_NUM_OF_802_1d_TAGS] = { WMEQosTag0ToAC, WMEQosTag1ToAC, WMEQosTag2ToAC, WMEQosTag3ToAC, WMEQosTag4ToAC, WMEQosTag5ToAC, WMEQosTag6ToAC, WMEQosTag7ToAC };

/* This array is now shared between qosMngr and TxData */
UINT8 wmeAcToUpIndex[MAX_NUM_OF_AC] = {0,1,4,6};

/* Used to indicate no user priority is assigned for AC */
#define INACTIVE_USER_PRIORITY 0xFF

/********************************************************************************/
/*						Internal functions prototypes.							*/
/********************************************************************************/
static void release_module(qosMngr_t *pQosMngr, UINT32 initVec);
static TI_STATUS verifyAndConfigTrafficParams(qosMngr_t *pQosMngr, queueTrafficParams_t *pQtrafficParams);
static TI_STATUS verifyAndConfigQosParams(qosMngr_t *pQosMngr,acQosParams_t *pAcQosParams);
static TI_STATUS getWMEInfoElement(qosMngr_t *pQosMngr,UINT8 *pWMEie,UINT8 *pLen);
static TI_STATUS verifyWmeIeParams(qosMngr_t *pQosMngr,UINT8 *pQosIeParams);
static TI_STATUS updateACParams(qosMngr_t *pQosMngr,ACParameters_t *pAcParams);
static void      updateTagToAcTable(qosMngr_t *pQosMngr,acTrfcType_e *pProtocolTagToACTable);
static TI_STATUS setWmeSiteParams(qosMngr_t *pQosMngr, UINT8 *pQosIeParams);
void qosMngr_resetAdmCtrlParameters(TI_HANDLE hQosMngr);
static TI_STATUS qosMngr_getCurrAcStatus(TI_HANDLE hQosMngr, OS_802_11_AC_UPSD_STATUS_PARAMS *pAcStatusParams);
static void deleteTspecConfiguration(qosMngr_t *pQosMngr, UINT8 acID);
static void setNonQosAdmissionState(qosMngr_t *pQosMngr, UINT8 acID);
static void qosMngr_storeTspecCandidateParams (tspecInfo_t *pCandidateParams, OS_802_11_QOS_TSPEC_PARAMS *pTSPECParams, UINT8 ac);

/********************************************************************************
 *							qosMngr_create										*
 ********************************************************************************
DESCRIPTION: QOS module creation function, called by the config mgr in creation phase. 
				performs the following:
				- Allocate the QOS MNGR handle.				                                                                                                   
INPUT:      hOs -			Handle to OS		


OUTPUT:		

RETURN:     Handle to the QOS MNGR module on success, NULL otherwise

************************************************************************/
TI_HANDLE qosMngr_create(TI_HANDLE hOs)
{
	qosMngr_t		*pQosMngr = NULL;
	UINT32			initVec = 0;

	if(!hOs)
		return NULL;
	
	/* allocating the WME object */
	pQosMngr = os_memoryAlloc(hOs,sizeof(qosMngr_t));

	if (pQosMngr == NULL)
		return NULL;

	initVec |= (1 << QOS_MNGR_INIT_BIT_LOCAL_VECTOR);
    
	/* create admission control object */
	pQosMngr->pTrafficAdmCtrl = trafficAdmCtrl_create(hOs);

	if (pQosMngr->pTrafficAdmCtrl == NULL)
	{
		qosMngr_destroy(pQosMngr);
		return NULL;
	}

	initVec |= (1 << QOS_MNGR_INIT_BIT_ADM_CTRL);

	return(pQosMngr);
}

/************************************************************************
 *                        qosMngr_destroy							    *
 ************************************************************************
DESCRIPTION: QOS MNGR module destroy function, called by the config mgr in
				 the destroy phase 
				 performs the following:
				-	Free all memory alocated by the module
				
INPUT:      hQosMngr	-	QOS Manager handle.		

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS qosMngr_destroy(TI_HANDLE hQosMngr)
{
	UINT32				   initVec;
	qosMngr_t *pQosMngr = (qosMngr_t *)hQosMngr;

	if (pQosMngr == NULL)
		return OK;

	initVec = 0xFFFF; 
    release_module(pQosMngr, initVec);

	return OK;
}

/***********************************************************************
 *                        release_module									
 ***********************************************************************
DESCRIPTION:	Called by the destroy function or by the create function (on failure)
				Go over the vector, for each bit that is set, release the corresponding module.
                                                                                                   
INPUT:      pQosMngr  -  QOS Mngr pointer.
			initVec	  -	 Vector that contains a bit set for each module thah had been initiualized

OUTPUT:		

RETURN:     OK on success, NOK otherwise
************************************************************************/
static void release_module(qosMngr_t *pQosMngr, UINT32 initVec)
{
	
	if (initVec & (1 << QOS_MNGR_INIT_BIT_ADM_CTRL))
		trafficAdmCtrl_unload(pQosMngr->pTrafficAdmCtrl);

	if (initVec & (1 << QOS_MNGR_INIT_BIT_LOCAL_VECTOR))
		utils_nullMemoryFree(pQosMngr->hOs, pQosMngr, sizeof(qosMngr_t));
		
	initVec = 0;
}

/************************************************************************
 *                        qosMngr_config								*
 ************************************************************************
DESCRIPTION: QOS Manager module configuration function, called by the config 
				mgr in configuration phase
				performs the following:
				-	Reset & initiailzes local variables
				-	Init the handles to be used by the module
                                                                                                   
INPUT:      hQosMngr	         -	Qos Manager handle.
		    List of handles to be used by the module
			pQosMngrInitParams	-	Init table of the module.		


OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS   qosMngr_config(TI_HANDLE     	        hQosMngr,
						   TI_HANDLE		        hHalCtrl,
						   TI_HANDLE		        hSiteMgr,
						   TI_HANDLE		        hReport,
						   TI_HANDLE		        hOs,
                           TI_HANDLE                hTxData,
                           TI_HANDLE                hMeasurementMngr,
                           TI_HANDLE                hSmeSm,
						   TI_HANDLE				hMemMgr,
						   TI_HANDLE				hCtrlData,
						   TI_HANDLE				hEvHandler,
						   TI_HANDLE				hExcMgr,
						   QosMngrInitParams_t		*pQosMngrInitParams)
{

    qosMngr_t *pQosMngr = (qosMngr_t *)hQosMngr;
	paramInfo_t			  param;
  	UINT8                 acID;
	TI_STATUS             status;

    /* init handles */
    pQosMngr->hOs       = hOs;
    pQosMngr->hReport   = hReport;
    pQosMngr->hSiteMgr  = hSiteMgr;
    pQosMngr->hHalCtrl  = hHalCtrl;
    pQosMngr->hTxData   = hTxData;
    pQosMngr->hMeasurementMngr = hMeasurementMngr;
    pQosMngr->hSmeSm = hSmeSm;
    pQosMngr->hMemMgr   = hMemMgr;
    pQosMngr->hCtrlData = hCtrlData;
	pQosMngr->hEvHandler = hEvHandler;
	pQosMngr->hExcMgr = hExcMgr;

    pQosMngr->isConnected = FALSE;

    /* init params */
    pQosMngr->WMEEnable = pQosMngrInitParams->wmeEnable;
    pQosMngr->trafficAdmCtrlEnable = pQosMngrInitParams->trafficAdmCtrlEnable;
    pQosMngr->tagZeroConverHeader = pQosMngrInitParams->qosTagZeroConverHeader;
	pQosMngr->qosPacketBurstEnable = pQosMngrInitParams->PacketBurstEnable;
	pQosMngr->qosPacketBurstTxOpLimit = pQosMngrInitParams->PacketBurstTxOpLimit;
	pQosMngr->desiredPsMode = pQosMngrInitParams->desiredPsMode;

    pQosMngr->activeProtocol    = NONE_QOS;
    pQosMngr->WMESiteSupport    = FALSE;

	pQosMngr->desiredMaxSpLen	= pQosMngrInitParams->desiredMaxSpLen;
	pQosMngr->desiredVoiceDeliveryProtocol = PS_SCHEME_LEGACY_PSPOLL;

    pQosMngr->voiceTspecConfigured = FALSE;
	pQosMngr->videoTspecConfigured = FALSE;
    pQosMngr->performTSPECRenegotiation = FALSE;
	pQosMngr->TSPECNegotiationResultCallb = NULL;
	pQosMngr->TSPECNegotiationResultModule = NULL;

    /* No template has been set for UPSD */
    pQosMngr->QosNullDataTemplateUserPriority = 0xFF;

	/* configure admission control parameters */
	qosMngr_resetAdmCtrlParameters(pQosMngr);

	status = trafficAdmCtrl_config(pQosMngr->pTrafficAdmCtrl,
								pQosMngr->hTxData,
								pQosMngr->hReport,
								pQosMngr->hOs,
								pQosMngr,
								pQosMngr->hCtrlData,
								pQosMngr->hMemMgr,
								pQosMngr->hExcMgr,
								&pQosMngrInitParams->trafficAdmCtrlInitParams);
	if(status != OK)
		return NOK;

	/* 
	 * configure per AC traffic parameters
	 */
    for(acID = FIRST_AC_INDEX;acID < MAX_NUM_OF_AC; acID++)
	{
		/*
		 * setting ac traffic params for TrafficCategoryCfg (TNET configuration 
		 * The parameters can be changed in run-time, so they are saved in "init params"
		 * for 'disconnecting' .
		 * the parameters being set in setSite; "select" phase.
         */
		pQosMngr->acParams[acID].QtrafficParams.queueID       = acID;
		pQosMngr->acParams[acID].QtrafficParams.channelType   = CHANNEL_TYPE_EDCF;
		pQosMngr->acParams[acID].QtrafficParams.tsid          = acID;
		pQosMngr->acParams[acID].QtrafficParams.dot11EDCATableMSDULifeTime = 0;
		pQosMngr->acParams[acID].QtrafficParams.psScheme      = PS_SCHEME_LEGACY;
		pQosMngr->acParams[acID].QtrafficParams.ackPolicy     = pQosMngrInitParams->acAckPolicy[acID];
		pQosMngr->acParams[acID].QtrafficParams.APSDConf[0]   = 0;
		pQosMngr->acParams[acID].QtrafficParams.APSDConf[1]   = 0;


		/* 
		 * Update the qTrafficInitParams as well 
		 */
		os_memoryCopy(hOs,&(pQosMngr->acParams[acID].QTrafficInitParams),&(pQosMngr->acParams[acID].QtrafficParams),sizeof(queueTrafficParams_t));

		/* will be config only after select */
		verifyAndConfigTrafficParams(pQosMngr,&(pQosMngr->acParams[acID].QtrafficParams));
		
		/*
		 * setting ac QoS params for acQosParams (TNET configuration) 
		 * The parameters can be changed in run-time, so they are saved in "init params"
		 * for 'disconnecting'.
		 * the parameters being set in setSite; "select" phase.
         */
		pQosMngr->acParams[acID].acQosParams.ac        = acID;
		pQosMngr->acParams[acID].acQosParams.aifsn     = AIFS_DEF;
		pQosMngr->acParams[acID].acQosParams.cwMax     = CW_MAX_DEF;
		pQosMngr->acParams[acID].acQosParams.cwMin     = CW_MIN_MAX;
		pQosMngr->acParams[acID].acQosParams.txopLimit = QOS_TX_OP_LIMIT_DEF;
		pQosMngr->acParams[acID].msduLifeTimeParam     = pQosMngrInitParams->MsduLifeTime[acID];

		/* The protocol is NONE_QOS. If Packet Burst is Enable,            */
		/* then, the BE queue is configured to the TxOP Limit of Packet burst */
		/* (that is, 3 ms) and the txopContinuation is set to  qosPacketBurstEnable  */
		/* The protocol is NONE_QOS. If Packet Burst is Enable,            */
		/* then, the BE queue is configured to the TxOP Limit of Packet burst */
		/* (that is, 3 ms) and the txopContinuation is set to  qosPacketBurstEnable  */

		if (acID == QOS_AC_BE)	
		{
			if (pQosMngr->qosPacketBurstEnable==TRUE)
			{
				pQosMngr->acParams[QOS_AC_BE].acQosParams.txopLimit = pQosMngr->qosPacketBurstTxOpLimit;
			}
			else 
			{
				pQosMngr->acParams[QOS_AC_BE].acQosParams.txopLimit = QOS_TX_OP_LIMIT_DEF;
			}
		}

		/* 
		 * Update the acQosInitParams as well 
		 */
		os_memoryCopy(hOs,&(pQosMngr->acParams[acID].acQosInitParams),&(pQosMngr->acParams[acID].acQosParams),sizeof(acQosParams_t));

		/* will be config only after select */
		verifyAndConfigQosParams(hQosMngr,&(pQosMngr->acParams[acID].acQosParams));
		
		


		/*
		 * setting ps mode per ac for protocol specific configuration.
		 */

        /* validity check - allow to set the desired Ps mode per-AC to UPSD ONLY IF the station supports UPSD */
        if ((pQosMngrInitParams->desiredPsMode == PS_SCHEME_UPSD_TRIGGER) && (pQosMngrInitParams->desiredWmeAcPsMode[acID] == PS_SCHEME_UPSD_TRIGGER))
        {
		        pQosMngr->acParams[acID].desiredWmeAcPsMode = PS_SCHEME_UPSD_TRIGGER;
        }
        else
        {
               pQosMngr->acParams[acID].desiredWmeAcPsMode = PS_SCHEME_LEGACY_PSPOLL;
        }

		pQosMngr->acParams[acID].currentWmeAcPsMode  = PS_SCHEME_LEGACY_PSPOLL; /* default configuration is legacy PS  for all queues */

		/*
		 * configure TxData per AC params
		 */
		param.content.txDataQosParams.acID                      = acID;
		param.content.txDataQosParams.acTrfcCtrl.PsMode         = PS_SCHEME_LEGACY;
		param.content.txDataQosParams.acTrfcCtrl.QueueIndex     = acID; 
        param.content.txDataQosParams.acTrfcCtrl.TxQueueSize    = pQosMngrInitParams->TxQueueSize[acID];
		param.content.txDataQosParams.acTrfcCtrl.QueueOvFlowPolicy = pQosMngrInitParams->QueueOvFlowPolicy[acID];			              
		param.content.txDataQosParams.acTrfcCtrl.MsduLifeTime 	= pQosMngrInitParams->MsduLifeTime[acID];
		param.content.txDataQosParams.acTrfcCtrl.ackPolicy    	= ACK_POLICY_LEGACY; /* working with Non - Qos method */

		param.paramType = TX_DATA_PS_MODE_PARAM;
		txData_setParam(pQosMngr->hTxData,&param);

		param.paramType = TX_DATA_CONFIG_TX_QUEUE_SIZE;
		txData_setParam(pQosMngr->hTxData,&param);

		param.paramType = TX_DATA_CONFIG_TX_QUEUE_OVFLOW_POLICY;
		txData_setParam(pQosMngr->hTxData,&param);

		param.paramType = TX_DATA_CONFIG_AC_MSDU_LIFE_TIME;
		txData_setParam(pQosMngr->hTxData,&param);

		param.paramType = TX_DATA_CONFIG_AC_ACK_POLICY;
		txData_setParam(pQosMngr->hTxData,&param);


		/* setting wme Ack Policy */
		pQosMngr->acParams[acID].wmeAcAckPolicy = pQosMngrInitParams->acAckPolicy[acID];

		/* Set admission state per AC for non-QoS and update the Tx module. */
		setNonQosAdmissionState(pQosMngr, acID);
	}

	pQosMngr->headerConvetMode = LEGACY_CONVERT;
	param.content.txDataQosParams.qosParams.headerConverMode          = LEGACY_CONVERT;

	/* Update the Tx tag to AC tables for legacy mode. */
	updateTagToAcTable(pQosMngr, (acTrfcType_e *)NonQosTagToACTable);

	
	param.paramType = TX_DATA_CONVERT_HEADER_MODE;
	txData_setParam(pQosMngr->hTxData,&param);

	param.paramType = TX_DATA_CONVERT_TAG_ZERO_HEADER_MODE;
    param.content.txDataQosParams.qosParams.convertTagZeroFrames = pQosMngrInitParams->qosTagZeroConverHeader;
	txData_setParam(pQosMngr->hTxData,&param);

	WLAN_REPORT_INFORMATION(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
		  ("qosMngr_config : QoS configuration complete!")); 
	

    return OK;
}

/************************************************************************
 *                    qosMngr_resetAdmCtrlParameters	                *
 ************************************************************************
DESCRIPTION: reset the admCtrl parameters
                                                                                                   
INPUT:      hQosMngr	         -	Qos Manager handle.

OUTPUT:		

RETURN:     

************************************************************************/

void qosMngr_resetAdmCtrlParameters(TI_HANDLE hQosMngr)
{
	UINT8 acID;
    qosMngr_t *pQosMngr = (qosMngr_t *)hQosMngr;

	/* reset admission control parameters */
	for(acID = FIRST_AC_INDEX ; acID < MAX_NUM_OF_AC ; acID++)
	{
		pQosMngr->resourceMgmtTable.currentTspecInfo[acID].AC = (acTrfcType_e)acID;
		pQosMngr->resourceMgmtTable.currentTspecInfo[acID].userPriority = INACTIVE_USER_PRIORITY; /* Setting invalid user Priority to prevent GET_TSPEC or DELETE */
		pQosMngr->resourceMgmtTable.currentTspecInfo[acID].nominalMsduSize = 0;
        pQosMngr->resourceMgmtTable.currentTspecInfo[acID].minimumPHYRate = 0;
		pQosMngr->resourceMgmtTable.currentTspecInfo[acID].meanDataRate = 0;
		pQosMngr->resourceMgmtTable.currentTspecInfo[acID].surplausBwAllowance = 0;
		pQosMngr->resourceMgmtTable.currentTspecInfo[acID].mediumTime = 0;
		pQosMngr->resourceMgmtTable.currentTspecInfo[acID].UPSDFlag = 0;
		pQosMngr->resourceMgmtTable.currentTspecInfo[acID].streamDirection = BI_DIRECTIONAL;
		pQosMngr->resourceMgmtTable.currentTspecInfo[acID].trafficAdmState = AC_NOT_ADMITTED;

		pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].AC = (acTrfcType_e)acID;
		pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].userPriority = INACTIVE_USER_PRIORITY; /* Setting invalid user Priority to prevent GET_TSPEC or DELETE */
		pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].nominalMsduSize = 0;
        pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].minimumPHYRate = 0;
		pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].meanDataRate = 0;
		pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].surplausBwAllowance = 0;
		pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].mediumTime = 0;
		pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].UPSDFlag = 0;
		pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].streamDirection = BI_DIRECTIONAL;
		pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].trafficAdmState = AC_NOT_ADMITTED;

		pQosMngr->resourceMgmtTable.totalAllocatedMediumTime = 0;
	}
}


/************************************************************************
 *                        qosMngr_disconnect   			                *
 ************************************************************************
DESCRIPTION: the function is called upon driver disconnecting to reset all
             QOS parameters to init values.
                                                                                                   
INPUT:      hQosMngr	         -	Qos Manager handle.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

TI_STATUS qosMngr_disconnect(TI_HANDLE hQosMngr)
{
    qosMngr_t *pQosMngr = (qosMngr_t *)hQosMngr;
	paramInfo_t			  param;
	UINT8                 acID;
	TI_STATUS             status;

	if(hQosMngr == NULL)
		return OK;

	pQosMngr->activeProtocol    = NONE_QOS;
    pQosMngr->WMESiteSupport    = FALSE;

	/* clear admission control params */
	qosMngr_resetAdmCtrlParameters(pQosMngr);
	
	trafficAdmCtrl_stop(pQosMngr->pTrafficAdmCtrl);

	for(acID = FIRST_AC_INDEX;acID < MAX_NUM_OF_AC; acID++)
	{

		/* disable event in rate adaptation */
		ctrlData_setTspecsRateEvent(pQosMngr->hCtrlData, acID, FALSE);

        /* Disable medium time events in TX */
		txData_setAdmisionCtrlParams(pQosMngr->hTxData, acID, 0 , 0, FALSE);

		/* The protocol after disconnect is NONE_QOS. If Packet Burst is Enabled, the BE queue InitParams 
		    is configured to the TxOP Limit of Packet burst  (that is, 3 ms) and the 
		    txopContinuation is set to qosPacketBurstEnable. */
		
		if (acID == QOS_AC_BE)
		{
			if (pQosMngr->qosPacketBurstEnable==TRUE)
			{
				pQosMngr->acParams[QOS_AC_BE].acQosInitParams.txopLimit = pQosMngr->qosPacketBurstTxOpLimit;
			}
			else 
			{
				pQosMngr->acParams[QOS_AC_BE].acQosInitParams.txopLimit = QOS_TX_OP_LIMIT_DEF;
			}
		}
		
		/* Copy init traffic params (non-QoS defaults) to current traffic params, and config to HAL and TNET. */
       os_memoryCopy(pQosMngr->hOs,&(pQosMngr->acParams[acID].acQosParams),&(pQosMngr->acParams[acID].acQosInitParams),sizeof(acQosParams_t));		

		/* 
		 * Update the qTrafficInitParams as well 
		 */
	   os_memoryCopy(pQosMngr->hOs,&(pQosMngr->acParams[acID].QtrafficParams),&(pQosMngr->acParams[acID].QTrafficInitParams),sizeof(queueTrafficParams_t));



	   pQosMngr->acParams[acID].currentWmeAcPsMode  = PS_SCHEME_LEGACY_PSPOLL; /* default configuration is legacy PS  for all queues */

	    /*
	    * setting TxData params, only PS , Txoplimit and Admission State and AC status
	    * can be changed and reset to defaults.
	    */
	   param.content.txDataQosParams.acID                      = acID;
	   param.content.txDataQosParams.acTrfcCtrl.PsMode         = PS_SCHEME_LEGACY;
	   param.content.txDataQosParams.acTrfcCtrl.ackPolicy      = ACK_POLICY_LEGACY; /* working with Non - Qos method */
	   param.content.txDataQosParams.acTrfcCtrl.MsduLifeTime   = pQosMngr->acParams[acID].msduLifeTimeParam;

	   param.paramType = TX_DATA_CONFIG_AC_MSDU_LIFE_TIME;
	   txData_setParam(pQosMngr->hTxData, &param);

	   param.paramType = TX_DATA_PS_MODE_PARAM;
	   txData_setParam(pQosMngr->hTxData,&param);

	   param.paramType = TX_DATA_CONFIG_AC_ACK_POLICY;
	   txData_setParam(pQosMngr->hTxData,&param);

	   /* Set admission state per AC for non-QoS and update the Tx module. */
	   setNonQosAdmissionState(pQosMngr, acID);
	}

	/* 
	 * configure only BE AC 
	 * NOTE : this is done after "disconnect" or Init phase so those are defaults BE params 
	 */

	/*
	 * configureQueue
	 */
	status = verifyAndConfigTrafficParams(hQosMngr,&(pQosMngr->acParams[QOS_AC_BE].QtrafficParams));
	if (status != OK)
	{
			WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
							("qosMngr_setSite:failed to init NON_QOS Queue Traffic parameters!!!\n\n")); 
			return status;
	}

	/*
	 * configureAC
	 */

	status = verifyAndConfigQosParams(hQosMngr,&(pQosMngr->acParams[QOS_AC_BE].acQosParams));
	if (status != OK)
	{
			WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
							("qosMngr_setSite:failed to init NON_QOS AC QoS parameters!!!\n\n")); 
			return status;
	}


	/*
	 * reset TxData globals parameters:  header converting to legacy and Tag to Ac table without
	 * QOS 
	 */
	pQosMngr->headerConvetMode = LEGACY_CONVERT;
	param.content.txDataQosParams.qosParams.headerConverMode          = LEGACY_CONVERT;



	/* Update the Tx and HAL tag to AC tables for legacy mode. */
	updateTagToAcTable(pQosMngr, (acTrfcType_e *)NonQosTagToACTable);


	param.paramType = TX_DATA_CONVERT_HEADER_MODE;
	txData_setParam(pQosMngr->hTxData,&param);


    /* Update our internal state */
    pQosMngr->isConnected = FALSE;

    /* Mark that no Qos Null template is currently set into firmware */
    pQosMngr->QosNullDataTemplateUserPriority = 0xFF;

    pQosMngr->voiceTspecConfigured = FALSE;

	pQosMngr->videoTspecConfigured = FALSE;

    /* Mark that no Qos Null template is currently set into firmware */
    pQosMngr->QosNullDataTemplateUserPriority = 0xFF;

	WLAN_REPORT_INFORMATION(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
		  ("qosMngr_disconnect : QoS disconnect complete!")); 

#ifdef EXC_MODULE_INCLUDED
	measurementMgr_stopTsMetrics(pQosMngr->hMeasurementMngr);
#endif

	return OK;
}


/************************************************************************
 *                        qosMngr_connect   			                *
 ************************************************************************
DESCRIPTION: the function is called upon driver connection to inform all 
             the other modules about the voice mode
                                                                                                   
INPUT:      hQosMngr	         -	Qos Manager handle.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

TI_STATUS qosMngr_connect(TI_HANDLE hQosMngr)
{
    qosMngr_t *pQosMngr = (qosMngr_t *)hQosMngr;
    psPollTemplate_t        psPollTemplate;
    whalCtrl_setTemplate_t  templateStruct;
    QosNullDataTemplate_t  QosNullDataTemplate;
    UINT8   acID,UPSDCnt=0;

   if (pQosMngr->isConnected == TRUE)
   {
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, ("qosMngr_connect : Already connected !!!\n")); 
     return OK;
   }

    /* Send PsPoll template to HAL */
    
    templateStruct.pTemplate = (UINT8 *)&psPollTemplate;
    templateStruct.templateType = PS_POLL_TEMPLATE;
    buildPsPollTemplate(pQosMngr->hSiteMgr, &templateStruct);
    whalCtrl_SetTemplate(pQosMngr->hHalCtrl, &templateStruct);
    	
    /* Update our internal state */
    pQosMngr->isConnected = TRUE;

    /* Set Qos-Null Data template into firmware */
	for(acID = FIRST_AC_INDEX;acID < MAX_NUM_OF_AC; acID++)
	{
        /* Set QOS Null data template into the firmware (only if at least one AC is configured as UPSD )*/
        if (pQosMngr->acParams[acID].currentWmeAcPsMode == PS_SCHEME_UPSD_TRIGGER)
        {
           UPSDCnt++;
           if ( pQosMngr->acParams[acID].apInitAdmissionState != ADMISSION_REQUIRED )
           {
            pQosMngr->QosNullDataTemplateUserPriority = wmeAcToUpIndex[acID];

            templateStruct.pTemplate = (UINT8 *)&QosNullDataTemplate;
            templateStruct.templateType = QOS_NULL_DATA_TEMPLATE;
            buildQosNullDataTemplate(pQosMngr->hSiteMgr, &templateStruct,pQosMngr->QosNullDataTemplateUserPriority);
            whalCtrl_SetTemplate(pQosMngr->hHalCtrl, &templateStruct);

            WLAN_REPORT_INFORMATION(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
               ("setWmeSiteParams: Setting QOS Null data for UserPriority %d (belongs to AC %d)\n",wmeAcToUpIndex[acID],acID));

            break; /* Only need to set ONE template, so after setting it, we can exit the loop */
           }
        }

    }

    /* If MAX_NUM_OF_AC (4) ACs were found as UPSD, but we still haven't configured any UP in the Qos Null data template, it must mean all ACs require admission - not valid*/
    if ((pQosMngr->QosNullDataTemplateUserPriority == 0xFF) && (UPSDCnt == MAX_NUM_OF_AC))
    {
      WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, ("qosMngr_connect : QOS Null Data template not set since all ACs require admission !!!\n")); 
	}

	return OK;
}

/************************************************************************
 *                        qosMngr_evalSite					            *
 ************************************************************************
DESCRIPTION: Evaluate the site for the selction algorithm
			 In case the station is configure to work in UPSD mode
			 prefer a site that support UPSD and return 1.
			 All other case return 0.
                                                                                                   
INPUT:      siteAPSDSupport - the UPSD capabilit of the site

OUTPUT:		 

RETURN:     1 - evaluation is good...
			0 - evaluation can be better....

************************************************************************/
UINT8 qosMngr_evalSite(TI_HANDLE hQosMngr, BOOL siteAPSDSupport)
{
   qosMngr_t *pQosMngr = (qosMngr_t *)hQosMngr;

   if (pQosMngr->desiredPsMode == PS_SCHEME_UPSD_TRIGGER && siteAPSDSupport)
   {
		return 1;
   }

   return 0;
}



/************************************************************************
 *                        qosMngr_getACparams           			    *
 ************************************************************************
DESCRIPTION: The function is an API for external modules to qet qos parameters
                                                                                                   
INPUT:      hQosMngr	         -	Qos Manager handle.
            pParamInfo           -  qos parameters information.


OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

TI_STATUS qosMngr_getParams(TI_HANDLE  hQosMngr,paramInfo_t *pParamInfo)
{
	acTrfcType_e           acID;
	qosMngr_t *pQosMngr = (qosMngr_t *)hQosMngr;

	if(pQosMngr == NULL)
		return NOK;

	WLAN_REPORT_INFORMATION(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, ("qosMngr_getParams: %x\n", pParamInfo->paramType)); 

	switch(pParamInfo->paramType)
	{
	case QOS_PACKET_BURST_ENABLE:
		pParamInfo->content.qosPacketBurstEnb = pQosMngr->qosPacketBurstEnable;
		break;
	case QOS_MNGR_CURRENT_PS_MODE:
		pParamInfo->content.currentPsMode = pQosMngr->currentPsMode;
		break;

    case QOS_MNGR_ACTIVE_PROTOCOL:
       pParamInfo->content.qosSiteProtocol = pQosMngr->activeProtocol;
       break;

    case QOS_MNGR_GET_DESIRED_PS_MODE:
        pParamInfo->content.qosDesiredPsMode.uDesiredPsMode = pQosMngr->desiredPsMode;
		for(acID = FIRST_AC_INDEX; acID < MAX_NUM_OF_AC ; acID++ )
			pParamInfo->content.qosDesiredPsMode.uDesiredWmeAcPsMode[acID] = pQosMngr->acParams[acID].desiredWmeAcPsMode;
        break;
	   
	case QOS_MNGR_VOICE_RE_NEGOTIATE_TSPEC:
	/* Check if voice call present. If so, store current TSPEC configuration */
		pParamInfo->content.TspecConfigure.voiceTspecConfigure = (UINT8)pQosMngr->voiceTspecConfigured;
		pParamInfo->content.TspecConfigure.videoTspecConfigure = (UINT8)pQosMngr->videoTspecConfigured;

		WLAN_REPORT_INFORMATION(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, ("qosMngr_getParams: QOS_MNGR_VOICE_RE_NEGOTIATE_TSPEC=%d\n", pQosMngr->voiceTspecConfigured)); 

		if (pQosMngr->voiceTspecConfigured == TRUE) 
		{
			OS_802_11_QOS_TSPEC_PARAMS *pTspecParams;
			tspecInfo_t *pConfiguredParams;

			/* Store voice TSPEC params - must be configured */
			pTspecParams = &pQosMngr->tspecRenegotiationParams[USER_PRIORITY_6];
			pConfiguredParams = &pQosMngr->resourceMgmtTable.candidateTspecInfo[USER_PRIORITY_6];

			pTspecParams->uUserPriority = pConfiguredParams->userPriority;
			pTspecParams->uNominalMSDUsize = pConfiguredParams->nominalMsduSize;
			pTspecParams->uMeanDataRate = pConfiguredParams->meanDataRate;
			pTspecParams->uMinimumPHYRate = pConfiguredParams->minimumPHYRate;
			pTspecParams->uSurplusBandwidthAllowance = pConfiguredParams->surplausBwAllowance;
			pTspecParams->uAPSDFlag = pConfiguredParams->UPSDFlag;
			pTspecParams->uMediumTime = pConfiguredParams->mediumTime;
		}
		else
		{
			pQosMngr->tspecRenegotiationParams[USER_PRIORITY_6].uUserPriority = MAX_USER_PRIORITY;
		}

		if (pQosMngr->videoTspecConfigured == TRUE) 
		{
			OS_802_11_QOS_TSPEC_PARAMS *pTspecParams;
			tspecInfo_t *pConfiguredParams;

			/* Store signalling TSPEC params if configured in user priority 4 */
			pTspecParams = &pQosMngr->tspecRenegotiationParams[USER_PRIORITY_4];
			pConfiguredParams = &pQosMngr->resourceMgmtTable.candidateTspecInfo[USER_PRIORITY_4];

			pTspecParams->uUserPriority = pConfiguredParams->userPriority;
   			pTspecParams->uNominalMSDUsize = pConfiguredParams->nominalMsduSize;
   			pTspecParams->uMeanDataRate = pConfiguredParams->meanDataRate;
   			pTspecParams->uMinimumPHYRate = pConfiguredParams->minimumPHYRate;
   			pTspecParams->uSurplusBandwidthAllowance = pConfiguredParams->surplausBwAllowance;
   			pTspecParams->uAPSDFlag = pConfiguredParams->UPSDFlag;
   			pTspecParams->uMediumTime = pConfiguredParams->mediumTime;
   		}
		else
		{
			pQosMngr->tspecRenegotiationParams[USER_PRIORITY_4].uUserPriority = MAX_USER_PRIORITY;
		}
		break;

	default:
           WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
				("qosMngr_getParams Error: unknown paramType 0x%x!\n",pParamInfo->paramType)); 
			return (PARAM_NOT_SUPPORTED);
/*		break; - unreachable */

	}
	return OK;
}

/************************************************************************
 *                        qosMngr_setParams              			    *
 ************************************************************************
DESCRIPTION: The function is an API for external modules to set qos parameters
                                                                                                   
INPUT:      hQosMngr	         -	Qos Manager handle.
            pParamInfo           -  qos parameters information.


OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

TI_STATUS qosMngr_setParams(TI_HANDLE  hQosMngr,paramInfo_t *pParamInfo)
{
    qosMngr_t *pQosMngr = (qosMngr_t *)hQosMngr;
	paramInfo_t			   param;
	whalParamInfo_t		   whalParam;		
	acTrfcType_e           acID;
	TI_STATUS              status;

	if(pQosMngr == NULL)
		return NOK;

	if(pParamInfo == NULL)
	{
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
			("qosMngr_setParams :Error trying to set NULL params!\n")); 
		return NOK;
	}

	WLAN_REPORT_INFORMATION(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, ("qosMngr_setParams: %x\n", pParamInfo->paramType)); 

	switch(pParamInfo->paramType)
	{

		case QOS_PACKET_BURST_ENABLE:

			if (pParamInfo->content.qosPacketBurstEnb > QOS_PACKET_BURST_ENABLE_MAX)
				return (PARAM_VALUE_NOT_VALID);
			
			/* No change */
			if (pParamInfo->content.qosPacketBurstEnb == pQosMngr->qosPacketBurstEnable)
				return OK;

			/* Update the qosPacketBurstEnable parameter */
			pQosMngr->qosPacketBurstEnable = pParamInfo->content.qosPacketBurstEnb;

			/* Packet burst enable changed from F to T */ 
			if (pParamInfo->content.qosPacketBurstEnb == TRUE)
			{
				/* Update the acTrafficInitParams of BE to the packet burst def*/
				pQosMngr->acParams[QOS_AC_BE].acQosInitParams.txopLimit = pQosMngr->qosPacketBurstTxOpLimit;
			
				/* Update the acTrafficParams of BE and the hal to the packet burst def*/
				if (pQosMngr->activeProtocol == NONE_QOS)
				{
					pQosMngr->acParams[QOS_AC_BE].acQosParams.txopLimit = pQosMngr->qosPacketBurstTxOpLimit;
					/* verify the parameters and update the hal */
					status = verifyAndConfigQosParams(hQosMngr,&(pQosMngr->acParams[QOS_AC_BE].acQosParams));
					if(status != OK)
						return status;
				}
			}
				
			/* Packet burst enable changed from T to F*/ 
			else 
			{
				/* Update the acTrafficInitParams of BE to the AC def*/
				pQosMngr->acParams[QOS_AC_BE].acQosInitParams.txopLimit = QOS_TX_OP_LIMIT_DEF;
			
				/* Update the acTrafficParams of BE  and the hal to the AC def*/
				if (pQosMngr->activeProtocol == NONE_QOS)
				{
					pQosMngr->acParams[QOS_AC_BE].acQosParams.txopLimit = QOS_TX_OP_LIMIT_DEF;
					/* verify the parameters and update the hal */
					status = verifyAndConfigQosParams(hQosMngr,&(pQosMngr->acParams[QOS_AC_BE].acQosParams));
					if(status != OK)
						return status;
				}
			}
			break;

		case QOS_MNGR_SET_SITE_PROTOCOL:
			if(pParamInfo->content.qosSiteProtocol == WME)
				pQosMngr->WMESiteSupport = TRUE;
			else
				pQosMngr->WMESiteSupport = FALSE;
		break;

	case QOS_MNGR_SET_OS_PARAMS:
		if((acTrfcType_e)pParamInfo->content.qosOsParams.acID > QOS_HIGHEST_AC_INDEX)
		{
			WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
					  ("qosMngr_setParams :Error  trying to set invalid acId: %d param",pParamInfo->content.qosOsParams.acID)); 

			return (PARAM_VALUE_NOT_VALID);
		}

		if((pParamInfo->content.qosOsParams.VoiceDeliveryProtocol != TRUE) && (pParamInfo->content.qosOsParams.VoiceDeliveryProtocol != FALSE))
		{
			WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
				("qosMngr_setParams :Error  trying to set invalid VoiceDeliveryProtocol: %d param",pParamInfo->content.qosOsParams.VoiceDeliveryProtocol)); 
			
			return (PARAM_VALUE_NOT_VALID);
		}
		
		if((pParamInfo->content.qosOsParams.VoiceDeliveryProtocol == TRUE) && ((PSScheme_e)pParamInfo->content.qosOsParams.PSDeliveryProtocol == PS_SCHEME_UPSD_TRIGGER))
		{
			WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
				("qosMngr_setParams Error: trying to set invalid VoiceDeliveryProtocol == PS_POLL and PSDeliveryProtocol = UPSD\n")); 
			
			return (PARAM_VALUE_NOT_VALID);
		}

		if((pParamInfo->content.qosOsParams.VoiceDeliveryProtocol == TRUE) && ((acTrfcType_e)pParamInfo->content.qosOsParams.acID !=  QOS_AC_VO))
		{
			WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
				("qosMngr_setParams Error: trying to set invalid VoiceDeliveryProtocol == PS_POLL and AC != VOICE \n")); 
			
			return (PARAM_VALUE_NOT_VALID);
		}

		if(((PSScheme_e)pParamInfo->content.qosOsParams.PSDeliveryProtocol != PS_SCHEME_LEGACY_PSPOLL) && ((PSScheme_e)pParamInfo->content.qosOsParams.PSDeliveryProtocol != PS_SCHEME_UPSD_TRIGGER))
		{
			WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
				("qosMngr_setParams :Error trying to set invalid PSDeliveryProtocol: %d param\n",pParamInfo->content.qosOsParams.PSDeliveryProtocol)); 

			return (PARAM_VALUE_NOT_VALID);
		}

		/* config tidConf */
		acID = (acTrfcType_e)pParamInfo->content.qosOsParams.acID;

        /* Convert TRUE/FALSE setting (from Utility Adapter) to PS_POLL or PS_NONE settings */
        if (pParamInfo->content.qosOsParams.VoiceDeliveryProtocol == FALSE)
        {
           pParamInfo->content.qosOsParams.VoiceDeliveryProtocol = PS_SCHEME_LEGACY;
        }
        if (pParamInfo->content.qosOsParams.VoiceDeliveryProtocol == TRUE)
        {
           pParamInfo->content.qosOsParams.VoiceDeliveryProtocol = PS_SCHEME_LEGACY_PSPOLL;
        }
           

		if( (pParamInfo->content.qosOsParams.PSDeliveryProtocol != pQosMngr->acParams[acID].desiredWmeAcPsMode) && 
			(pQosMngr->isConnected == TRUE) )
		{
			WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
				("qosMngr_setParams :Error  trying to set new PS protocol while connected")); 
			
			return (PARAM_VALUE_NOT_VALID);
		}


		/* UPSD_FW open in upsd integration */
		/* set the current PS mode. In not connected state it is always Legacy since the currentPsMode only 
		 update after connection */
		pQosMngr->acParams[acID].QtrafficParams.psScheme = pQosMngr->acParams[acID].currentWmeAcPsMode;
		pQosMngr->acParams[acID].msduLifeTimeParam = pParamInfo->content.qosOsParams.MaxLifeTime;

		status = verifyAndConfigTrafficParams(pQosMngr,&(pQosMngr->acParams[acID].QtrafficParams));
		if(status != OK)
			return status;
		
		/* set txData Voice Delivery Protocol mode */
		if(acID == QOS_AC_VO)
			pQosMngr->desiredVoiceDeliveryProtocol = pParamInfo->content.qosOsParams.VoiceDeliveryProtocol;

		param.content.txDataQosParams.acID = acID;
		param.content.txDataQosParams.acTrfcCtrl.PsMode = pParamInfo->content.qosOsParams.VoiceDeliveryProtocol;
		param.paramType = TX_DATA_PS_MODE_PARAM;
		txData_setParam(pQosMngr->hTxData,&param);

		param.content.txDataQosParams.acTrfcCtrl.MsduLifeTime = pParamInfo->content.qosOsParams.MaxLifeTime;
		param.paramType = TX_DATA_CONFIG_AC_MSDU_LIFE_TIME;
		txData_setParam(pQosMngr->hTxData,&param);

		/* synch psPoll mode with qosMngr */
		/* Update the PsMode parameter */
		pQosMngr->acParams[acID].desiredWmeAcPsMode = pParamInfo->content.qosOsParams.PSDeliveryProtocol;
		break;

	case QOS_MNGR_CURRENT_PS_MODE:
		if( (pQosMngr->activeProtocol == WME) && (pQosMngr->desiredPsMode == PS_SCHEME_UPSD_TRIGGER) && (pParamInfo->content.currentPsMode == PS_SCHEME_UPSD_TRIGGER) )
			pQosMngr->currentPsMode = PS_SCHEME_UPSD_TRIGGER;
		else
			pQosMngr->currentPsMode = PS_SCHEME_LEGACY_PSPOLL;
		break;
		
    case QOS_MNGR_ADD_TSPEC_REQUEST:
		pQosMngr->TSPECNegotiationResultCallb = NULL;
		pQosMngr->TSPECNegotiationResultModule = NULL;
		return (qosMngr_requestAdmission(hQosMngr,  &pParamInfo->content.qosAddTspecRequest));
/*		break; - unreachable */
		
	case QOS_MNGR_RESEND_TSPEC_REQUEST:
		WLAN_REPORT_INFORMATION(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
								("qosMngr_setParams: QOS_MNGR_RESEND_TSPEC_REQUEST\n")); 
		pQosMngr->TSPECNegotiationResultCallb = (qosMngrCallb_t)pParamInfo->content.qosRenegotiateTspecRequest.callback;
		pQosMngr->TSPECNegotiationResultModule = pParamInfo->content.qosRenegotiateTspecRequest.handler; 
		status = qosMngr_requestAdmission(hQosMngr,  &pQosMngr->tspecRenegotiationParams[USER_PRIORITY_6]);

		if ((status == OK) && (pQosMngr->tspecRenegotiationParams[USER_PRIORITY_4].uUserPriority != MAX_USER_PRIORITY))
		{
			status = qosMngr_requestAdmission(hQosMngr,  &pQosMngr->tspecRenegotiationParams[USER_PRIORITY_4]);
		}
		return (status);
/*		break;  - unreachable */
		
    case QOS_MNGR_DEL_TSPEC_REQUEST:
		return (qosMngr_deleteAdmission(hQosMngr, &pParamInfo->content.qosDelTspecRequest));
/*		break; - unreachable */
		
    case QOS_MNGR_AC_STATUS:
		return (qosMngr_getCurrAcStatus (hQosMngr,&pParamInfo->content.qosCurrentAcStatus));
/*		break; - unreachable */

    case QOS_MNGR_AP_QOS_PARAMETERS:  /* API GetAPQosParameters */
		acID = (acTrfcType_e) pParamInfo->content.qosApQosParams.uAC;

		if(acID > QOS_HIGHEST_AC_INDEX)
		{
			WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
					  ("qosMngr_setParams :Error  trying to set invalid acId: %d param\n",pParamInfo->content.qosApQosParams.uAC)); 

			return (PARAM_VALUE_NOT_VALID);
		}
		if(pQosMngr->isConnected == FALSE)
		{
			return NOT_CONNECTED;
		}
		if(pQosMngr->activeProtocol == NONE_QOS)
		{
			return NO_QOS_AP;
		}
		
		pParamInfo->content.qosApQosParams.uAssocAdmissionCtrlFlag = pQosMngr->acParams[acID].apInitAdmissionState; /* admission flag */
		pParamInfo->content.qosApQosParams.uAIFS = pQosMngr->acParams[acID].acQosParams.aifsn;
		pParamInfo->content.qosApQosParams.uCwMin = (1 << pQosMngr->acParams[acID].acQosParams.cwMin)-1;
		pParamInfo->content.qosApQosParams.uCwMax = (1 << pQosMngr->acParams[acID].acQosParams.cwMax)-1;
		pParamInfo->content.qosApQosParams.uTXOPLimit = pQosMngr->acParams[acID].acQosParams.txopLimit << 5;

		break;

	case QOS_MNGR_OS_TSPEC_PARAMS: 

		if( pParamInfo->content.qosTspecParameters.uUserPriority > MAX_USER_PRIORITY )
		{	
			WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
               ("qosMngr_getTspecParams: userPriority > 7 -> Ignore !!!\n"));
			return NOK;
		}

		if(pQosMngr->isConnected == FALSE)
		{	
			WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
               ("qosMngr_getTspecParams: Not connected - Ignoring request !!!\n"));
			return NOT_CONNECTED;
		}

		if(pQosMngr->activeProtocol == NONE_QOS)
		{	
			WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
               ("qosMngr_getTspecParams: Not connected to QOS AP - Ignoring reqeust !!!\n"));
			return NO_QOS_AP;
		}
		
		acID = (acTrfcType_e)WMEQosTagToACTable[pParamInfo->content.qosTspecParameters.uUserPriority];

		/* check if signaling is already in process*/
		if(pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].trafficAdmState == AC_WAIT_ADMISSION)
		{
			WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,("qosMngr_requestAdmission: AC = %d , TSPEC Signaling is in progress -> Ignoring request !!!\n",acID));
			return TRAFIC_ADM_PENDING;
		}

	   /* check if UP is admitted or not */
	   if(pQosMngr->resourceMgmtTable.currentTspecInfo[acID].userPriority != pParamInfo->content.qosTspecParameters.uUserPriority)
       {	
		 WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
            ("qosMngr_getTspecParams: user priority is not admitted. -> Ignore !!!\n"));
		 return USER_PRIORITY_NOT_ADMITTED;
		}

		pParamInfo->content.qosTspecParameters.uMeanDataRate = pQosMngr->resourceMgmtTable.currentTspecInfo[acID].meanDataRate;
		pParamInfo->content.qosTspecParameters.uNominalMSDUsize = pQosMngr->resourceMgmtTable.currentTspecInfo[acID].nominalMsduSize;
		pParamInfo->content.qosTspecParameters.uAPSDFlag  = pQosMngr->resourceMgmtTable.currentTspecInfo[acID].UPSDFlag;
		pParamInfo->content.qosTspecParameters.uMinimumPHYRate  = pQosMngr->resourceMgmtTable.currentTspecInfo[acID].minimumPHYRate;
		pParamInfo->content.qosTspecParameters.uSurplusBandwidthAllowance  = pQosMngr->resourceMgmtTable.currentTspecInfo[acID].surplausBwAllowance;
		pParamInfo->content.qosTspecParameters.uMediumTime = pQosMngr->resourceMgmtTable.currentTspecInfo[acID].mediumTime;
		break;

	case QOS_SET_RATE_THRESHOLD: 

		ctrlData_setTspecsRateThresholds(pQosMngr->hCtrlData,
											(UINT8)pParamInfo->content.QOSRateThreshold.uAC,
											(UINT8)pParamInfo->content.QOSRateThreshold.uHighThreshold,
											(UINT8)pParamInfo->content.QOSRateThreshold.uLowThreshold);

		
		/* If currently connected to a QOS AP , need to enable the TSPEc event upon the setting of the thresholds */
		if(pQosMngr->activeProtocol == WME)
		{
			ctrlData_setTspecsRateEvent(pQosMngr->hCtrlData, (UINT8)pParamInfo->content.QOSRateThreshold.uAC, TRUE);
		}

		break;

	case QOS_GET_RATE_THRESHOLD:

		/* SET operation is performed, but actually this is only for AC parameter transfer from Utility Adapter to driver, since copy
	      of user supplied block of data (and vice versa) is only performed in SetParam calls, the driver can also modify the supplied
		  structure and thus return it to user mode */
		ctrlData_getTspecsRateThresholds(pQosMngr->hCtrlData,
									(UINT8)pParamInfo->content.QOSRateThreshold.uAC,
									(UINT32*)&pParamInfo->content.QOSRateThreshold.uHighThreshold,
									(UINT32*)&pParamInfo->content.QOSRateThreshold.uLowThreshold);

		break;

	case QOS_SET_RX_TIME_OUT:
		if (pParamInfo->content.rxTimeOut.UPSD == 0)
		{
				WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
								  ("%s :Error trying to set invalid zero timeout for UPSD \n",__FUNCTION__)); 
				return PARAM_VALUE_NOT_VALID;
				
		}
		pQosMngr->rxTimeOut.psPoll = (UINT16)pParamInfo->content.rxTimeOut.psPoll;
		pQosMngr->rxTimeOut.UPSD = (UINT16)pParamInfo->content.rxTimeOut.UPSD;


		/* set RxTimeOut to FW */
		whalParam.paramType	= HAL_CTRL_RX_TIME_OUT_PARAM;
		whalParam.content.halCtrlRxTimeOut = pQosMngr->rxTimeOut;
		whalCtrl_SetParam(pQosMngr->hHalCtrl,&whalParam); 
		break;


	case QOS_MNGR_VOICE_RE_NEGOTIATE_TSPEC:
		if( pParamInfo->content.TspecConfigure.voiceTspecConfigure || pParamInfo->content.TspecConfigure.videoTspecConfigure)
            pQosMngr->performTSPECRenegotiation = TRUE;
		else
			pQosMngr->performTSPECRenegotiation = FALSE;

	   WLAN_REPORT_INFORMATION(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
							   ("qosMngr_setParams: QOS_MNGR_VOICE_RE_NEGOTIATE_TSPEC=%d\n", pQosMngr->performTSPECRenegotiation)); 
	   break;

	default:
         WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
				("qosMngr_getParams Error: unknown paramType 0x%x!\n",pParamInfo->paramType)); 
			return (PARAM_NOT_SUPPORTED);
/*		break; - unreachable */
	}

	return OK;


}

/************************************************************************
 *                        verifyAndConfigTrafficParams  			    *
 ************************************************************************
DESCRIPTION: The function verifies the parameters set by qosMngr to 
             the queue traffic params in whalCtrl to be configured to TNET. 
                                                                                                   
INPUT:      hQosMngr	         -	Qos Manager handle.
            pAcTrafficParams     -  pointer to ac parameters.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

static TI_STATUS verifyAndConfigTrafficParams(qosMngr_t *pQosMngr, queueTrafficParams_t *pQtrafficParams)
{
    whalParamInfo_t		   whalParam;    
    queueTrafficParams_t   queueTrafficParams;

	if(pQtrafficParams->queueID >  MAX_NUM_OF_TX_QUEUES - 1)
    {
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
						  ("verifyAndConfigTrafficParams :Error  trying to set invalid queueID: %d param",pQtrafficParams->queueID)); 

		return (PARAM_VALUE_NOT_VALID);
	}


	if(pQtrafficParams->channelType > MAX_CHANNEL_TYPE)
	{
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
						  ("verifyAndConfigTrafficParams :Error  trying to set invalid channelType: %d param",pQtrafficParams->channelType)); 

		return (PARAM_VALUE_NOT_VALID);

	}

    /* TBD */
	if(pQtrafficParams->tsid > AC_PARAMS_MAX_TSID)
	{
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
						  ("verifyAndConfigTrafficParams :Error  trying to set invalid tsid: %d param",pQtrafficParams->tsid)); 

		return (PARAM_VALUE_NOT_VALID);

	}


	if(pQtrafficParams->psScheme > MAX_PS_SCHEME)
	{
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
						  ("verifyAndConfigTrafficParams :Error  trying to set invalid psScheme: %d param",pQtrafficParams->psScheme)); 

		return (PARAM_VALUE_NOT_VALID);
	}

    /* TBD */
#if 0
	if(pQtrafficParams->APSDConf > MAX_APSD_CONF)
	{
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
						  ("verifyAndConfigTrafficParams :Error  trying to set invalid APSDConf: %d param",pQtrafficParams->APSDConf)); 

		return (PARAM_VALUE_NOT_VALID);
	}
#endif

	if(pQtrafficParams->ackPolicy > MAX_ACK_POLICY)
	{
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
						  ("verifyAndConfigTrafficParams :Error  trying to set invalid ackPolicy: %d param",pQtrafficParams->ackPolicy)); 

		return (PARAM_VALUE_NOT_VALID);
	}

    /*
     * Convert the QOS PS-scheme enum into the WHAL enum.
     * NOTE: need to copy for not to change the QoS configuration
     */
    queueTrafficParams = *pQtrafficParams;

    switch (queueTrafficParams.psScheme)
    {
    case PS_SCHEME_LEGACY:
        queueTrafficParams.psScheme = PS_SCHEME_LEGACY;
        break;
    case PS_SCHEME_UPSD_TRIGGER:
        queueTrafficParams.psScheme = PS_SCHEME_UPSD_TRIGGER;
        break;
    case PS_SCHEME_LEGACY_PSPOLL:
        queueTrafficParams.psScheme = PS_SCHEME_LEGACY;
        break;
    case PS_SCHEME_SAPSD:
        queueTrafficParams.psScheme = PS_SCHEME_SAPSD;
        break;
    }

	whalParam.paramType = HAL_CTRL_QUEUES_PARAMS;
	/* set parameters */
	whalParam.content.pQueueTrafficParams = &queueTrafficParams;

	return(whalCtrl_SetParam(pQosMngr->hHalCtrl, &whalParam));
}

/************************************************************************
 *                        verifyAndConfigQosParams          		    *
 ************************************************************************
DESCRIPTION: The function verifies the parameters set by qosMngr to 
             the AC Qos params in whalCtrl to be configured to TNET. 
                                                                                                   
INPUT:      hQosMngr	         -	Qos Manager handle.
            pAcTrafficParams     -  pointer to ac parameters.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

static TI_STATUS  verifyAndConfigQosParams(qosMngr_t *pQosMngr,acQosParams_t *pAcQosParams)
{
    whalParamInfo_t		   whalParam;
	acQosParams_t          acQosParams;

	if(pAcQosParams->ac >  MAX_NUM_OF_AC - 1 )
    {
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
						  ("verifyAndConfigQosParams :Error  trying to set invalid ac : %d param",pAcQosParams->ac)); 

		return (PARAM_VALUE_NOT_VALID);
	}
 /*  verification is unnecessary due to limited range of pAcQosParams->aifsn data type (UINT8)
	if(pAcQosParams->aifsn >  QOS_AIFS_MAX )
    {
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
						  ("verifyAndConfigQosParams :Error  trying to set invalid aifsn : %d param",pAcQosParams->aifsn)); 

		return (PARAM_VALUE_NOT_VALID);
	}
 */
	if(pAcQosParams->cwMax >  QOS_CWMAX_MAX )
    {
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
						  ("verifyAndConfigQosParams :Error  trying to set invalid cwMax : %d param",pAcQosParams->cwMax)); 

		return (PARAM_VALUE_NOT_VALID);
	}

	if(pAcQosParams->cwMin >  QOS_CWMIN_MAX )
    {
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
						  ("verifyAndConfigQosParams :Error  trying to set invalid cwMax : %d param",pAcQosParams->cwMax)); 

		return (PARAM_VALUE_NOT_VALID);
	}

	if(pAcQosParams->txopLimit >  QOS_TX_OP_LIMIT_MAX )
    {
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
						  ("verifyAndConfigQosParams :Error  trying to set invalid txopLimit : %d param",pAcQosParams->txopLimit)); 

		return (PARAM_VALUE_NOT_VALID);
	}


	whalParam.paramType = HAL_CTRL_AC_PARAMS; 

	acQosParams.ac = pAcQosParams->ac;
	acQosParams.aifsn =  pAcQosParams->aifsn;

	/* convert to TNET units */
	acQosParams.cwMax =  (1 << pAcQosParams->cwMax) - 1; /* CwMax = 2^CwMax - 1*/
	acQosParams.cwMin =  (1 << pAcQosParams->cwMin) - 1; /* CwMin = 2^CwMin - 1*/
	acQosParams.txopLimit =  pAcQosParams->txopLimit << 5; /* in us */



	/* set parameters */
	whalParam.content.configureCmdCBParams.CB_buf = (UINT8*)&acQosParams;
	whalParam.content.configureCmdCBParams.CB_Func = NULL;
	whalParam.content.configureCmdCBParams.CB_handle = NULL;

	return whalCtrl_SetParam(pQosMngr->hHalCtrl, &whalParam);
}

/************************************************************************
 *                        qosMngr_selectActiveProtocol    			            *
 ************************************************************************
DESCRIPTION: The function is called in order to set the active protocol in
             the qosMngr according to site capabilities and desired mode.
             called from SystemConfig.
                                                                                                   
INPUT:      

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS qosMngr_selectActiveProtocol(TI_HANDLE  hQosMngr)
{
   	qosMngr_t *pQosMngr = (qosMngr_t *)hQosMngr;

	/* decide qos protocol */
	/* NOTE: if both exc qnd wme supported wme is chosen */
	if(pQosMngr->WMESiteSupport && pQosMngr->WMEEnable)
	{
		pQosMngr->activeProtocol = WME;
	}
	else
    {
		pQosMngr->activeProtocol = NONE_QOS;
	}
	WLAN_REPORT_DEBUG_TX(pQosMngr->hReport,
	 (" qosMngr_selectActiveProtocol() : pQosMngr->activeProtocol %d\n",pQosMngr->activeProtocol));

    return OK;
}

/************************************************************************
 *                        qosMngr_setAcPsDeliveryMode    			            *
 ************************************************************************
DESCRIPTION: The function is called in order to set the upsd/ps_poll according 
             to the desired and current upsd mode (per AC as well).
             called from systemConfig.
                                                                                                   
INPUT:      

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS qosMngr_setAcPsDeliveryMode(TI_HANDLE  hQosMngr)
{
   UINT8 acID;
   qosMngr_t *pQosMngr = (qosMngr_t *)hQosMngr;

	/* in case the current PS mode is not UPSD  - the IE is empty */
	if(pQosMngr->currentPsMode == PS_SCHEME_UPSD_TRIGGER)
	{
		for(acID = FIRST_AC_INDEX;acID < MAX_NUM_OF_AC; acID++)
		{
			if(pQosMngr->acParams[acID].desiredWmeAcPsMode == PS_SCHEME_UPSD_TRIGGER)
			{
				pQosMngr->acParams[acID].currentWmeAcPsMode = PS_SCHEME_UPSD_TRIGGER;
			}
		}
    }
	else {
        paramInfo_t param;

        // Setting params for PS Poll mode
        // This was done to enable voice delivery protocol when the AP doesn't
        // support UASPD.
        // Note these values have been taken from the default value table in 
        // TiCtrl.c -- see TiUAPSDConfig()
        
        os_memoryZero( pQosMngr->hOs, &param, sizeof(paramInfo_t) );
        param.paramType = QOS_MNGR_SET_OS_PARAMS;

        // Setting AC Best Effort settings
        param.content.qosOsParams.acID = QOS_AC_BE;
        param.content.qosOsParams.MaxLifeTime = 512;
        param.content.qosOsParams.VoiceDeliveryProtocol = FALSE;
        param.content.qosOsParams.PSDeliveryProtocol = PS_SCHEME_LEGACY_PSPOLL;
        qosMngr_setParams( pQosMngr, &param );


        // Setting AC Background settings
        param.content.qosOsParams.acID = QOS_AC_BK;
        param.content.qosOsParams.MaxLifeTime = 100;
        param.content.qosOsParams.VoiceDeliveryProtocol = FALSE;
        param.content.qosOsParams.PSDeliveryProtocol = PS_SCHEME_LEGACY_PSPOLL;
        qosMngr_setParams( pQosMngr, &param );


        // Setting AC Video settings
        param.content.qosOsParams.acID = QOS_AC_VI;
        param.content.qosOsParams.MaxLifeTime = 100;
        param.content.qosOsParams.VoiceDeliveryProtocol = FALSE;
        param.content.qosOsParams.PSDeliveryProtocol = PS_SCHEME_LEGACY_PSPOLL;
        qosMngr_setParams( pQosMngr, &param );

        // Setting AC Voice settings
        param.content.qosOsParams.acID = QOS_AC_VO;
        param.content.qosOsParams.MaxLifeTime = 60;
        param.content.qosOsParams.VoiceDeliveryProtocol = TRUE;
        param.content.qosOsParams.PSDeliveryProtocol = PS_SCHEME_LEGACY_PSPOLL;
        qosMngr_setParams( pQosMngr, &param );
    }
	
	return OK;

}


/************************************************************************
 *                        qosMngr_getQosCapabiltyInfeElement    			            *
 ************************************************************************
DESCRIPTION: The function is called in order to build the Qos Capability
			 IE for the associatiomn request.
                                                                                                   
INPUT:      

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS qosMngr_getQosCapabiltyInfeElement(TI_HANDLE  hQosMngr, UINT8 *pQosIe, UINT8 *pLen)
{    
	qosMngr_t *pQosMngr = (qosMngr_t *)hQosMngr;
	dot11_QOS_CAPABILITY_IE_t *dot11_QOS_CAPABILITY_IE = (dot11_QOS_CAPABILITY_IE_t *)pQosIe;
	TI_STATUS status = OK;
	UINT8	extraIeLen = 0;
	
	if(pQosMngr->activeProtocol == WME)
	{
		dot11_QOS_CAPABILITY_IE->hdr.eleId    = DOT11_QOS_CAPABILITY_ELE_ID;
		dot11_QOS_CAPABILITY_IE->hdr.eleLen   = DOT11_QOS_CAPABILITY_ELE_LEN;
		
		/* The default configuration of QoS info Field is legacy PS for all ACs */
		dot11_QOS_CAPABILITY_IE->QosInfoField = 0;
		
		/* in case the current PS mode is not UPSD  - the IE is empty */
		if(pQosMngr->currentPsMode == PS_SCHEME_UPSD_TRIGGER)
		{
			if(pQosMngr->acParams[QOS_AC_VO].currentWmeAcPsMode == PS_SCHEME_UPSD_TRIGGER)
			{
				dot11_QOS_CAPABILITY_IE->QosInfoField |= (1 << AC_VO_APSD_FLAGS_SHIFT);
			}
			if(pQosMngr->acParams[QOS_AC_VI].currentWmeAcPsMode == PS_SCHEME_UPSD_TRIGGER)
			{
				dot11_QOS_CAPABILITY_IE->QosInfoField |= (1 << AC_VI_APSD_FLAGS_SHIFT);
			}

			if(pQosMngr->acParams[QOS_AC_BK].currentWmeAcPsMode == PS_SCHEME_UPSD_TRIGGER)
			{
				dot11_QOS_CAPABILITY_IE->QosInfoField |= (1 << AC_BK_APSD_FLAGS_SHIFT);
			}

			if(pQosMngr->acParams[QOS_AC_BE].currentWmeAcPsMode == PS_SCHEME_UPSD_TRIGGER)
			{
				dot11_QOS_CAPABILITY_IE->QosInfoField |= (1 << AC_BE_APSD_FLAGS_SHIFT);
			}

			dot11_QOS_CAPABILITY_IE->QosInfoField |= (((pQosMngr->desiredMaxSpLen) & MAX_SP_LENGTH_MASK) << MAX_SP_LENGTH_SHIFT);

            WLAN_REPORT_INFORMATION(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,
               ("dot11_QOS_CAPABILITY_IE->QosInfoField = 0x%x\n",dot11_QOS_CAPABILITY_IE->QosInfoField));
		}

		*pLen = dot11_QOS_CAPABILITY_IE->hdr.eleLen + sizeof(dot11_eleHdr_t);
		
#ifdef EXC_MODULE_INCLUDED
		/* If required, add exc info-elements to the association request packets */
		if (pQosMngr->performTSPECRenegotiation == TRUE)
		{
            WLAN_REPORT_INFORMATION(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,
               ("qosMngr_getQosCapabiltyInfeElement: performing TSPEC renegotiation\n"));

			status = excMngr_getEXCQosIElements(pQosMngr->hExcMgr, (pQosIe+(*pLen)), &extraIeLen);
		}
#endif
		*pLen += extraIeLen;
	}
	else
	{
		*pLen = 0;
	}

	return status;

}
/************************************************************************
 *                        qosMngr_assocReqBuild    			            *
 ************************************************************************
DESCRIPTION: The function is called in order to build the assocReq IE for
             the current site QOS protocol.
                                                                                                   
INPUT:      hQosMngr	         -	Qos Manager handle.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

TI_STATUS qosMngr_assocReqBuild(TI_HANDLE  hQosMngr, UINT8 *pQosIe, UINT8 *pLen)
{
    qosMngr_t *pQosMngr = (qosMngr_t *)hQosMngr;
	TI_STATUS		status;

	
	if(pQosMngr == NULL)
	{
		*pLen = 0;
		return OK;
	}	

	/* building assocReq frame */
	switch(pQosMngr->activeProtocol)
	{
	case WME:
		status = getWMEInfoElement(pQosMngr,pQosIe,pLen);
		if (status !=OK) 
		{
			*pLen = 0;
		}
		break;

	case NONE_QOS:
			*pLen = 0;
			return OK;
/*		break; - unreachable */

	default:
			*pLen = 0;
		break;
	}
	
	return OK;
}

/************************************************************************
 *                        getWMEInfoElement     			            *
 ************************************************************************
DESCRIPTION: building WME IE.
                                                                                                   
INPUT:      hQosMngr	         -	Qos Manager handle.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

static TI_STATUS getWMEInfoElement(qosMngr_t *pQosMngr,UINT8 *pWMEie,UINT8 *pLen)
{
	dot11_WME_IE_t *pDot11_WME_IE = (dot11_WME_IE_t *)pWMEie;

	pDot11_WME_IE->hdr.eleId         = DOT11_WME_ELE_ID;
	pDot11_WME_IE->hdr.eleLen        = DOT11_WME_ELE_LEN;
	pDot11_WME_IE->OUI[0]            = 0x00;
	pDot11_WME_IE->OUI[1]            = 0x50;
	pDot11_WME_IE->OUI[2]            = 0xf2;
	pDot11_WME_IE->OUIType           = dot11_WME_OUI_TYPE;
	pDot11_WME_IE->OUISubType        = dot11_WME_OUI_SUB_TYPE_IE;
	pDot11_WME_IE->version           = dot11_WME_VERSION;
	pDot11_WME_IE->ACInfoField       = 0;

	if(pQosMngr->currentPsMode == PS_SCHEME_UPSD_TRIGGER)
	{
		if(pQosMngr->acParams[QOS_AC_VO].currentWmeAcPsMode == PS_SCHEME_UPSD_TRIGGER)
		{
			pDot11_WME_IE->ACInfoField |= (1 << AC_VO_APSD_FLAGS_SHIFT);
		}
		if(pQosMngr->acParams[QOS_AC_VI].currentWmeAcPsMode == PS_SCHEME_UPSD_TRIGGER)
		{
			pDot11_WME_IE->ACInfoField |= (1 << AC_VI_APSD_FLAGS_SHIFT);
		}

		if(pQosMngr->acParams[QOS_AC_BK].currentWmeAcPsMode == PS_SCHEME_UPSD_TRIGGER)
		{
			pDot11_WME_IE->ACInfoField |= (1 << AC_BK_APSD_FLAGS_SHIFT);
		}

		if(pQosMngr->acParams[QOS_AC_BE].currentWmeAcPsMode == PS_SCHEME_UPSD_TRIGGER)
		{
			pDot11_WME_IE->ACInfoField |= (1 << AC_BE_APSD_FLAGS_SHIFT);
		}

		pDot11_WME_IE->ACInfoField |= (((pQosMngr->desiredMaxSpLen) & MAX_SP_LENGTH_MASK) << MAX_SP_LENGTH_SHIFT);
	}

	*pLen = pDot11_WME_IE->hdr.eleLen + sizeof(dot11_eleHdr_t);

	return OK;

}

/************************************************************************
 *                        qosMngr_checkTspecRenegResults		        *
 ************************************************************************
DESCRIPTION: The function is called upon association response to check 
            Tspec renegotiation results
                                                                                                   
INPUT:      hQosMngr	  -	Qos Manager handle.
            assocRsp      -  pointer to received IE parameters received 
			                 in association response. 
OUTPUT:		

RETURN:     -

************************************************************************/
void qosMngr_checkTspecRenegResults(TI_HANDLE hQosMngr, assocRsp_t *assocRsp)
{
	tspecInfo_t	tspecInfo;
	qosMngr_t *pQosMngr = (qosMngr_t *)hQosMngr;
#ifdef EXC_MODULE_INCLUDED
	UINT32 acCount;
#endif

	WLAN_REPORT_INFORMATION(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,
	   ("qosMngr_checkTspecRenegResults: performTSPECRenegotiation = %d, tspecParams received= %x\n",
		pQosMngr->performTSPECRenegotiation, assocRsp->tspecVoiceParameters));

	if (pQosMngr->performTSPECRenegotiation != TRUE)
	{
		/* If no re-negotiation was requested, no parsing shall be done */
#ifdef EXC_MODULE_INCLUDED
		measurementMgr_disableTsMetrics(pQosMngr->hMeasurementMngr, MAX_NUM_OF_AC);
#endif
		return;
	}

	if (assocRsp->tspecVoiceParameters == NULL && assocRsp->tspecSignalParameters == NULL)
	{
		/* The renegotiation request was ignored - update QoS Manager database */
		qosMngr_setAdmissionInfo(pQosMngr, USER_PRIORITY_6, 
								 &pQosMngr->resourceMgmtTable.candidateTspecInfo[USER_PRIORITY_6], 
								 STATUS_TRAFFIC_ADM_REQUEST_REJECT);

		if (pQosMngr->tspecRenegotiationParams[USER_PRIORITY_4].uUserPriority != MAX_USER_PRIORITY)
		{
			qosMngr_setAdmissionInfo(pQosMngr, USER_PRIORITY_4, 
									 &pQosMngr->resourceMgmtTable.candidateTspecInfo[USER_PRIORITY_4], 
									 STATUS_TRAFFIC_ADM_REQUEST_REJECT);
		}
#ifdef EXC_MODULE_INCLUDED
        measurementMgr_disableTsMetrics(pQosMngr->hMeasurementMngr, MAX_NUM_OF_AC);
#endif
		return;
	}

	if (assocRsp->tspecVoiceParameters != NULL)
	{
	/* The renogitaion was performed - update QoS Manager database */
	pQosMngr->voiceTspecConfigured = TRUE;
	trafficAdmCtrl_parseTspecIE(pQosMngr->pTrafficAdmCtrl, &tspecInfo, assocRsp->tspecVoiceParameters);

	qosMngr_setAdmissionInfo(pQosMngr, tspecInfo.AC, &tspecInfo, STATUS_TRAFFIC_ADM_REQUEST_ACCEPT);
	}

	if (assocRsp->tspecSignalParameters != NULL)
	{
		/* Signal TSPEC was re-negotiated as well */
		pQosMngr->videoTspecConfigured = TRUE;
		trafficAdmCtrl_parseTspecIE(pQosMngr->pTrafficAdmCtrl, &tspecInfo, assocRsp->tspecSignalParameters);
		qosMngr_setAdmissionInfo(pQosMngr, tspecInfo.AC, &tspecInfo, STATUS_TRAFFIC_ADM_REQUEST_ACCEPT);
	}
	else if (pQosMngr->tspecRenegotiationParams[USER_PRIORITY_4].uUserPriority != MAX_USER_PRIORITY) 
	{
		/* Signal TSPEC was not re-negotiated although requested to - ERROR */
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,
				  ("qosMngr_setSite: Signal TSPEC was not re-negotiated while voice was \n"));
		qosMngr_setAdmissionInfo(pQosMngr, USER_PRIORITY_4, 
								 &pQosMngr->resourceMgmtTable.candidateTspecInfo[USER_PRIORITY_4], 
								 STATUS_TRAFFIC_ADM_REQUEST_REJECT);
	}

#ifdef EXC_MODULE_INCLUDED
	/* If EXC IEs are present for one or more ACs, update other modules with received parameters */
	for (acCount = 0; acCount < MAX_NUM_OF_AC; acCount++)
	{
		excMngr_setEXCQoSParams(pQosMngr->hExcMgr, &assocRsp->excIEs[acCount], acCount);
	}
#endif
}


/************************************************************************
 *                        qosMngr_setSite        			            *
 ************************************************************************
DESCRIPTION: The function is called upon association response to set site 
             parameters.
                                                                                                   
INPUT:      hQosMngr	  -	Qos Manager handle.
            assocRsp      -  pointer to received IE parameters received 
			                 in association response. 
OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS qosMngr_setSite(TI_HANDLE hQosMngr, assocRsp_t *assocRsp)
{
	qosMngr_t *pQosMngr = (qosMngr_t *)hQosMngr;
	TI_STATUS  status;
    paramInfo_t	param;

	if(hQosMngr == NULL)
        return NOK;

	/* checking active protocol */
	switch(pQosMngr->activeProtocol)
	{
		case WME:
			/* verify QOS protocol received in association response */
			status = verifyWmeIeParams(pQosMngr, (UINT8 *)assocRsp->WMEParams);
			if(status != OK)
			{
                pQosMngr->activeProtocol = NONE_QOS;
                WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,
                          ("qosMngr_setSite: setting active protocol WME params with non WME IE params frame, setting active protocol back to NONE \n"));
                return NOK;
			}

            status = setWmeSiteParams(pQosMngr, (UINT8 *)assocRsp->WMEParams);
			if (status != OK)
			{
                pQosMngr->activeProtocol = NONE_QOS;
                WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,
                          ("Warning: qosMngr_setSite-> failed to set AC WME parameters!!! , setting active protocol back to NONE\n"));
                return NOK;
			}
			/* update siteMgr with recevied params */
			status = siteMgr_setWMEParamsSite(pQosMngr->hSiteMgr, assocRsp->WMEParams);
			if (status != OK)
			{
                pQosMngr->activeProtocol = NONE_QOS;
                WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,
                                    ("qosMngr_setSite:failed to init WME parameters!!! , setting active protocol back to NONE\n\n"));
                return NOK;
			}

			break;

	case NONE_QOS:

			/* Check if the packet burst is enable, if it is, 
			should update the BE parames and the hal to the packet burst def */
			if (pQosMngr->qosPacketBurstEnable == TRUE)
			{
				/* Update the acTrafficInitParams of BE to the packet burst def*/
				pQosMngr->acParams[QOS_AC_BE].acQosInitParams.txopLimit = pQosMngr->qosPacketBurstTxOpLimit;
				/* Update the acTrafficParams of BE to the packet burst def*/
				pQosMngr->acParams[QOS_AC_BE].acQosInitParams.txopLimit = pQosMngr->qosPacketBurstTxOpLimit;
				/* verify the parameters and update the hal */
				status = verifyAndConfigQosParams(hQosMngr,&(pQosMngr->acParams[QOS_AC_BE].acQosParams));
				if (status != OK)
				{
					WLAN_REPORT_WARNING(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
									("qosMngr_setSite:failed to init NON_QOS parameters!!!\n\n")); 
					return NOK;
				}
			}

			/* By setting the PS_MODE of QOS_AC_VO to desiredVoiceDeliveryProtocol, we enable sending PS_POLL packets by the txData module 
			   Even though all user priorities are mapped to QOS_AC_BE, when txData checks if Ps_Poll is needed for AC_VO */
			param.content.txDataQosParams.acID = QOS_AC_VO;
			param.content.txDataQosParams.acTrfcCtrl.PsMode = pQosMngr->desiredVoiceDeliveryProtocol;
			param.paramType = TX_DATA_PS_MODE_PARAM;
			txData_setParam(pQosMngr->hTxData,&param);

		break;

	default:
		WLAN_REPORT_WARNING(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
							("Warning: qosMngr_setSite NO active protocls To set \n")); 
		break;
	}

	/* Check if TSPEC re-negotiation was performed, if so - look for results */
	qosMngr_checkTspecRenegResults(pQosMngr, assocRsp);
 
    return OK;

}

/************************************************************************
 *                        qosMngr_updateIEinfo     			            *
 ************************************************************************
DESCRIPTION: The function is called upon run-time update of AC parameters
                                                                                                   
INPUT:      hQosMngr	         -	Qos Manager handle.
            pQosIeParams         -  pointer to received IE parameters received 
			                        in beacon or probe response. 
OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

void qosMngr_updateIEinfo(TI_HANDLE hQosMngr, UINT8 *pQosIeParams,qosProtocols_e qosSetProtocol)
{
	qosMngr_t *pQosMngr = (qosMngr_t *)hQosMngr;
	TI_STATUS  status;
	dot11_WME_PARAM_t		*pWMEparams;



	if(pQosMngr == NULL)
		return;

	/* checking active protocol */
	switch(pQosMngr->activeProtocol)
	{
	case WME:
		if(qosSetProtocol != WME)
			return;

		if(pQosIeParams == NULL)
		{
			WLAN_REPORT_WARNING(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,("Warning: updateIEinfo -> trying to update WME parameters with NULL site parameters!!!\n"));
			return ;
		}

		/* verify QOS protocol received in update IE */
		status = verifyWmeIeParams(pQosMngr,pQosIeParams);
		if(status != OK)
		{
			WLAN_REPORT_WARNING(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,("Warning: updateIEinfo ->upadting active protocl WME params with non WME IE params frame !!!\n"));
			return ;
		}
		pWMEparams = (dot11_WME_PARAM_t *)pQosIeParams;

		/* update AC params */
		status = updateACParams(pQosMngr,&(pWMEparams->WME_ACParameteres));
		if(status != OK)
		{
			WLAN_REPORT_WARNING(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,("updateIEinfo-> failed to update AC WME parameters!!!\n\n"));
			return ;
		}
		break;


	default:
		WLAN_REPORT_WARNING(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,("updateIEinfo-> trying to update qos paramters without active protocol !!!"));
		break;
	}
}

/************************************************************************
 *                        qosMngr_buildTSPec       			            *
 ************************************************************************/
UINT32 qosMngr_buildTSPec(TI_HANDLE hQosMngr, UINT32 user_priority, UINT8 *pQosIe)
{
	qosMngr_t *pQosMngr = (qosMngr_t *)hQosMngr;
	OS_802_11_QOS_TSPEC_PARAMS *pPreservedParams;
	tspecInfo_t *pCandidateParams;
	UINT32 ieLen;

	pPreservedParams = &pQosMngr->tspecRenegotiationParams[user_priority];
	pCandidateParams = &pQosMngr->resourceMgmtTable.candidateTspecInfo[user_priority];

	if (pPreservedParams->uUserPriority != MAX_USER_PRIORITY)
	{
		qosMngr_storeTspecCandidateParams (pCandidateParams, pPreservedParams, user_priority);
		pCandidateParams->trafficAdmState = AC_WAIT_ADMISSION;

		trafficAdmCtrl_buildTSPec(pQosMngr->pTrafficAdmCtrl, pCandidateParams, pQosIe, &ieLen);
		return ieLen;
	}
	else
	{
		return 0;
	}
}

/************************************************************************
 *                        setWmeSiteParams        			            *
 ************************************************************************
DESCRIPTION: The function is called upon association response to set WME site 
             parameters.
                                                                                                   
INPUT:      hQosMngr	         -	Qos Manager handle.
            pQosIeParams         -  pointer to received IE parameters received 
			                        in association response. 
OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

static TI_STATUS setWmeSiteParams(qosMngr_t *pQosMngr, UINT8 *pQosIeParams)
{
	dot11_WME_PARAM_t  *pWMEparams = (dot11_WME_PARAM_t *)pQosIeParams;
	TI_STATUS           status;         
    paramInfo_t			param;
	UINT8               acID;

	if (pQosIeParams == NULL)
	{
		WLAN_REPORT_WARNING(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,
			("setWmeSiteParams: pQosIeParams is NULL !"));
		return NOK;
	}
	
	for(acID = FIRST_AC_INDEX;acID < MAX_NUM_OF_AC; acID++)
	{
		/* configure queue */
		status = verifyAndConfigTrafficParams(pQosMngr,&(pQosMngr->acParams[acID].QtrafficParams));
		if(status != OK)
			return status;



		/* Enable event in rate adaptation */
		ctrlData_setTspecsRateEvent(pQosMngr->hCtrlData, acID, TRUE);

		/* ac powerSave protocol mode */
		param.content.txDataQosParams.acID              = acID;
		param.content.txDataQosParams.acTrfcCtrl.PsMode = pQosMngr->acParams[acID].currentWmeAcPsMode;
		param.content.txDataQosParams.acTrfcCtrl.ackPolicy = pQosMngr->acParams[acID].wmeAcAckPolicy; /* working with Non - Qos method */
		
        /* In case QOS_AC_VO is not configured for UPSD, set it according to the desired voice delivery protocol, which can be either PS_POLL or PS_NONE */
        if ((acID == QOS_AC_VO) && (pQosMngr->acParams[acID].currentWmeAcPsMode != PS_SCHEME_UPSD_TRIGGER))
        {
           param.content.txDataQosParams.acTrfcCtrl.PsMode = pQosMngr->desiredVoiceDeliveryProtocol;
        }
		
		param.paramType = TX_DATA_PS_MODE_PARAM;
		txData_setParam(pQosMngr->hTxData,&param);

		param.paramType = TX_DATA_CONFIG_AC_ACK_POLICY;
		txData_setParam(pQosMngr->hTxData,&param);

	}

	/* update AC params */
	status = updateACParams(pQosMngr,&(pWMEparams->WME_ACParameteres));
	if(status != OK)
		return status;


	/* update per protocol params */

	/* update header convert mode */
	pQosMngr->headerConvetMode = QOS_CONVERT;
	param.content.txDataQosParams.qosParams.headerConverMode = QOS_CONVERT;
	param.paramType = TX_DATA_CONVERT_HEADER_MODE;
	txData_setParam(pQosMngr->hTxData,&param);

	return OK;
}


/************************************************************************
 *                        updateACParams     			                *
 ************************************************************************
DESCRIPTION: the function is called upon QOS protocol updates paramters 
             to TNET and TxData object 
                                                                                                   
INPUT:      hQosMngr	         -	Qos Manager handle.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

static TI_STATUS updateACParams(qosMngr_t *pQosMngr,ACParameters_t *pAcParams)
{
	UINT8               acID,i;
	TI_STATUS           status;
    QOS_AC_IE_ParametersRecord_t     *pACParameteresRecord;
	paramInfo_t			    param;

	/*
	 * For WME: setting ac traffic params (edcf etc')  
	 * in this order BE, BK , VI, VO .
	 */

    pACParameteresRecord = (QOS_AC_IE_ParametersRecord_t *)pAcParams;

	for(i = FIRST_AC_INDEX; i < MAX_NUM_OF_AC; i++, pACParameteresRecord++)
	{
		/* getting ac from ACI */
		acID = (pACParameteresRecord->ACI_AIFSN & AC_PARAMS_ACI_MASK) >> 5;

		/* edcf params */

		pQosMngr->acParams[acID].acQosParams.ac        				   = acID;
		pQosMngr->acParams[acID].acQosParams.aifsn                     = pACParameteresRecord->ACI_AIFSN & AC_PARAMS_AIFSN_MASK;
		pQosMngr->acParams[acID].acQosParams.cwMin                     = pACParameteresRecord->ECWmin_ECWmax & AC_PARAMS_CWMIN_MASK;
		pQosMngr->acParams[acID].acQosParams.cwMax                     = (pACParameteresRecord->ECWmin_ECWmax & AC_PARAMS_CWMAX_MASK) >> 4;
		pQosMngr->acParams[acID].acQosParams.txopLimit                 = pACParameteresRecord->TXOPLimit;

		status = verifyAndConfigQosParams(pQosMngr,&(pQosMngr->acParams[acID].acQosParams));
		if(status != OK)
			return status;


		/* UPSD configuration */
		pQosMngr->acParams[acID].QtrafficParams.psScheme = pQosMngr->acParams[acID].currentWmeAcPsMode;
		status = verifyAndConfigTrafficParams(pQosMngr,&(pQosMngr->acParams[acID].QtrafficParams));
		if(status != OK)
			return status;


		/* update admission state */
		if(pACParameteresRecord->ACI_AIFSN & AC_PARAMS_ACM_MASK)
		{
			pQosMngr->acParams[acID].apInitAdmissionState = ADMISSION_REQUIRED;
			param.content.txDataQosParams.qosParams.admissionRequired = ADMISSION_REQUIRED; 

			pQosMngr->resourceMgmtTable.currentTspecInfo[acID].trafficAdmState = AC_NOT_ADMITTED;
			param.content.txDataQosParams.qosParams.admissionState = AC_NOT_ADMITTED;
		}
		else
		{
			pQosMngr->acParams[acID].apInitAdmissionState = ADMISSION_NOT_REQUIRED;
			param.content.txDataQosParams.qosParams.admissionRequired = ADMISSION_NOT_REQUIRED; 

			pQosMngr->resourceMgmtTable.currentTspecInfo[acID].trafficAdmState = AC_ADMITTED;
			param.content.txDataQosParams.qosParams.admissionState = AC_ADMITTED;
		}

		param.content.txDataQosParams.acID = acID;
		param.paramType = TX_DATA_AC_ADMISSION_STATE;
		txData_setParam(pQosMngr->hTxData,&param);
	}

	/* Update the Tx and HAL tag to AC tables for WME mode. */
	updateTagToAcTable(pQosMngr, (acTrfcType_e *)WMEQosTagToACTable);

	return OK;
}

/************************************************************************
 *                             updateTagToAcTable  	                    *
 ************************************************************************
DESCRIPTION: the function updates tag to AC index according to the delivered
			 protocol tag to AC table (WME or legacy).
                                                                                                   
INPUT:      hQosMngr	           -	Qos Manager handle.
			pProtocolTagToACTable  -  protocol tag to AC table.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

void updateTagToAcTable(qosMngr_t *pQosMngr,acTrfcType_e *pProtocolTagToACTable)
{
	paramInfo_t		param;

	/* 
	 *  Update the Tx and HAL Tag to AC tables.
	 */
	os_memoryCopy(pQosMngr->hOs,(param.content.txDataQosParams.qosParams.tag_ToAcClsfrTable),
		pProtocolTagToACTable, sizeof(acTrfcType_e) * MAX_NUM_OF_802_1d_TAGS);

	param.paramType = TX_DATA_TAG_TO_AC_CLASSIFIER_TABLE;
	txData_setParam(pQosMngr->hTxData,&param);

}


/************************************************************************
 *                        verifyWmeIeParams     			            *
 ************************************************************************
DESCRIPTION: verify WME IE.
                                                                                                   
INPUT:      hQosMngr	         -	Qos Manager handle.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

static TI_STATUS verifyWmeIeParams(qosMngr_t *pQosMngr,UINT8 *pQosIeParams)
{
	dot11_WME_IE_t  WMEie;
	UINT8           Len;
	dot11_WME_IE_t  *pWMERecvIe = (dot11_WME_IE_t  *)pQosIeParams;

	if(pQosIeParams == NULL)
	{
		WLAN_REPORT_WARNING(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,("verifyWmeIeParams: FAILED"));
		return NOK;
	}


	/* get WME IE */
	getWMEInfoElement(pQosMngr,(UINT8 *)&WMEie,(UINT8 *)&Len);

	if((WMEie.hdr.eleId != pWMERecvIe->hdr.eleId ) ||
	   (WMEie.OUI[0] != pWMERecvIe->OUI[0]) ||
	   (WMEie.OUI[1] != pWMERecvIe->OUI[1]) ||
	   (WMEie.OUI[2] != pWMERecvIe->OUI[2]) ||
	   (WMEie.OUIType != pWMERecvIe->OUIType))
	{
		WLAN_REPORT_WARNING(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,("verifyWmeIeParams: FAILED"));
		return NOK;
	}


    if(WMEie.version != pWMERecvIe->version)
	   WLAN_REPORT_WARNING(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,("WARNING: verifyWmeIeParams: Driver WME version: %d, Site WME version: %d",WMEie.version,pWMERecvIe->version));

	/* compare assocRsp IE to WME IE (without acInfo field */
	return OK;
}


/************************************************************************
 *                    Admission Control Functions     		            *
 ************************************************************************/
/************************************************************************
 *                        qosMngr_requestAdmission     			        *
 ************************************************************************
DESCRIPTION: This function is API function for TSPEC request.

INPUT:      hQosMngr	         -	Qos Manager handle.
			addTspecParams		 -  The Tspec Parameters
OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

TI_STATUS qosMngr_requestAdmission(TI_HANDLE			hQosMngr, 
                                   OS_802_11_QOS_TSPEC_PARAMS *addTspecParams)
{

    qosMngr_t *pQosMngr = (qosMngr_t *)hQosMngr;
	TI_STATUS	status;
	UINT8		acID;
		

	/* check if STA is already connected to AP */
	if(pQosMngr->isConnected == FALSE)
	{
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,("qosMngr_requestAdmission: Not connected - Ignoring request !!!\n"));
		return NOT_CONNECTED;
	}

	/* check if AP support WME */
	if(pQosMngr->activeProtocol != WME)
	{
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,("qosMngr_requestAdmission: Not connected to a QOS AP - Ignoring request !!!\n"));
		return NO_QOS_AP;
	}

	/* check if Traffic Admission Control is enable */
	if(pQosMngr->trafficAdmCtrlEnable == FALSE)
	{
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,("qosMngr_requestAdmission: Admission-Control is disabled - Ignoring request !!!\n"));
		return ADM_CTRL_DISABLE;
	}

	/* check UP validity */
	if( addTspecParams->uUserPriority > MAX_USER_PRIORITY)
	{
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,("uUserPriority = %d > 7 !!!\n",addTspecParams->uUserPriority));
		return NOK;
	}

	/* find acID from the user priority */
	acID = WMEQosTagToACTable[addTspecParams->uUserPriority];

	/* check if signaling is already in process for this AC */
	if(pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].trafficAdmState == AC_WAIT_ADMISSION)
	{
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,("qosMngr_requestAdmission: AC = %d , signaling is in process -> Ignore Request !!!\n",acID));
		return TRAFIC_ADM_PENDING;
	}
	
	/* check if AC is already admitted with other UP */
	if( (pQosMngr->resourceMgmtTable.currentTspecInfo[acID].trafficAdmState == AC_ADMITTED) &&
		(pQosMngr->resourceMgmtTable.currentTspecInfo[acID].userPriority <= MAX_USER_PRIORITY) &&
		(pQosMngr->resourceMgmtTable.currentTspecInfo[acID].userPriority != addTspecParams->uUserPriority) )
	{
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,
			("qosMngr_requestAdmission: AC = %d , another UP (%d) on same AC is already admited -> Ignoring request !!!\n",
			acID, pQosMngr->resourceMgmtTable.currentTspecInfo[acID].userPriority));
		return AC_ALREADY_IN_USE;
	}

	/* check msdu size validity */

	if( (addTspecParams->uNominalMSDUsize & !FIX_MSDU_SIZE) > MAX_DATA_BODY_LENGTH)
	{
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,("uNominalMSDUsize = %d > 2312, !!!\n",addTspecParams->uNominalMSDUsize));
		return NOK;
	}
	
	/* check PS mode validity */
	if( (addTspecParams->uAPSDFlag == PS_SCHEME_UPSD_TRIGGER) && (pQosMngr->currentPsMode != PS_SCHEME_UPSD_TRIGGER) )
	{
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,
			("The STA's current status does not support UPSD -> Ignoring TSPEC request that has UPSD on !!!\n"));
		return NOK;
	}

	WLAN_REPORT_INFORMATION(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,("qosMngr_requestAdmission: UP = %d , acID = %d\n",addTspecParams->uUserPriority, acID));

    /* The following code was removed since TX module is updated on UPSD/PS-POLL configuration of AC, and therefore will NOT
       send PS-POLLs when QOS_AC_VO is configured for UPSD */
#if 0
	/* check PS mode validity for the voice queue*/
    /* Do not allow CHANGING QOS_AC_VO currentPsMode to UPSD while the desiredVoiceDeliveryProtocol is enabled */
	if( (acID == QOS_AC_VO) && 
       ((addTspecParams->uAPSDFlag == UPSD) && (pQosMngr->acParams[acID].currentWmeAcPsMode != UPSD)) &&
       (pQosMngr->desiredVoiceDeliveryProtocol == PS_POLL) )
	{
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,
			("The STA's current QOS_AC_VO configured to VoiceDeliveryProtocol,\n This configuration can not be in parallel with UPSD.\n In order to change the AC_VO to UPSD, user must first disable the VoiceDeliveryProtocol!!!\n"));
		return NOK;
	}
#endif

	/* set tspec parameters in candidateTspecInfo table */
	qosMngr_storeTspecCandidateParams (&(pQosMngr->resourceMgmtTable.candidateTspecInfo[acID]), 
										addTspecParams, (UINT8)acID);

	/* Perhaps this should be done only if the request was successfully sent */
	if (acID == QOS_AC_VO) 
	{
		pQosMngr->voiceTspecConfigured = TRUE;
	}
	
	if (acID == QOS_AC_VI) 
	{
		pQosMngr->videoTspecConfigured = TRUE;
	}

	/* call TrafficAdmCtrl API function for the signaling proccess */
	status = trafficAdmCtrl_startAdmRequest(pQosMngr->pTrafficAdmCtrl, &(pQosMngr->resourceMgmtTable.candidateTspecInfo[acID]));

	if(status == OK)
	{
		/* request transmitted OK */
		pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].trafficAdmState = AC_WAIT_ADMISSION;
		WLAN_REPORT_INFORMATION(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,("qosMngr_requestAdmission: UP = %d , request OK !!!\n",addTspecParams->uUserPriority));
	}
	else
	{
		/* reaquest not transmitted OK */
		pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].trafficAdmState = AC_NOT_ADMITTED;
		WLAN_REPORT_WARNING(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,("qosMngr_requestAdmission: UP = %d , request  NOT OK status=%d!!!\n",addTspecParams->uUserPriority, status));
		return NOK;
	}

	return status;
}

/************************************************************************
 *                        qosMngr_deleteAdmission     		            *
 ************************************************************************
DESCRIPTION: This function is API fuunction for tspec delete.

INPUT:      hQosMngr	         -	Qos Manager handle.
			delAdmissionParams	 -  
OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

TI_STATUS qosMngr_deleteAdmission(TI_HANDLE hQosMngr, OS_802_11_QOS_DELETE_TSPEC_PARAMS *delAdmissionParams)
{

    qosMngr_t *pQosMngr = (qosMngr_t *)hQosMngr;
	UINT8		acID;
	
	/* check UP validity */
	if( delAdmissionParams->uUserPriority > MAX_USER_PRIORITY )
	{	
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
           ("qosMngr_deleteAdmission: userPriority > 7 -> Ignore !!!"));
		return NOK;
	}

	/* check if STA is already connected to AP */
	if(pQosMngr->isConnected == FALSE)
	{	
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
           ("qosMngr_deleteAdmission: pQosMngr->connected == FALSE -> Ignore !!!"));
		return NOT_CONNECTED;
	}

	/* check if AP support WME */
	if(pQosMngr->activeProtocol != WME)
	{	
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
           ("qosMngr_deleteAdmission: activeProtocol != WME -> Ignore !!!"));
		return NO_QOS_AP;
	}

	/* find acID from the user priority */
	acID = WMEQosTagToACTable[delAdmissionParams->uUserPriority];

	/* check if tspec is already addmited for this AC */
	if(pQosMngr->resourceMgmtTable.currentTspecInfo[acID].trafficAdmState != AC_ADMITTED)
	{	
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
           ("qosMngr_deleteAdmission: AC is not ADMITED -> Ignore !!!"));
		return NOK;
	}

	/* check if AC is already admited with the same UP */
	if(pQosMngr->resourceMgmtTable.currentTspecInfo[acID].userPriority != delAdmissionParams->uUserPriority)
	{	
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
           ("qosMngr_deleteAdmission: user priority is invalid. -> Ignore !!!\n"));
		return USER_PRIORITY_NOT_ADMITTED;
	}

	/* check if signaling is already in procces for this AC */
	if(pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].trafficAdmState == AC_WAIT_ADMISSION)
	{	
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
           ("qosMngr_deleteAdmission: AC is under negotiation -> Ignore !!!"));
		return TRAFIC_ADM_PENDING;
	}


	
	/* call TrafficAdmCtrl API function for the delete tspec */
	trafficAdmCtrl_sendDeltsFrame(pQosMngr->pTrafficAdmCtrl, &(pQosMngr->resourceMgmtTable.currentTspecInfo[acID]), 
										(UINT8)delAdmissionParams->uReasonCode );

	
	deleteTspecConfiguration(pQosMngr, acID);
	
	return OK;

}
/************************************************************************
 *                        deleteTspecConfiguration     		            *
 ************************************************************************
DESCRIPTION: configure the driver and FW to default configuration after
			 tspec deletion.

INPUT:      hQosMngr	             - Qos Manager handle.
			acID					 - the AC of the Tspec to delete
OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

static void deleteTspecConfiguration(qosMngr_t *pQosMngr, UINT8 acID)
{
   paramInfo_t param;

    /* Zero Tspec parameters */
    pQosMngr->resourceMgmtTable.currentTspecInfo[acID].nominalMsduSize = 0;
    pQosMngr->resourceMgmtTable.currentTspecInfo[acID].minimumPHYRate = 0;
    pQosMngr->resourceMgmtTable.currentTspecInfo[acID].meanDataRate = 0;
    pQosMngr->resourceMgmtTable.currentTspecInfo[acID].surplausBwAllowance = 0;
    pQosMngr->resourceMgmtTable.currentTspecInfo[acID].mediumTime = 0;
    pQosMngr->resourceMgmtTable.currentTspecInfo[acID].UPSDFlag = 0;
    pQosMngr->resourceMgmtTable.currentTspecInfo[acID].streamDirection = BI_DIRECTIONAL;
	
	/* update total medium time */
	pQosMngr->resourceMgmtTable.totalAllocatedMediumTime -= pQosMngr->resourceMgmtTable.currentTspecInfo[acID].mediumTime;
	
	
	/* set params to TX */
	/*------------------*/
	/* disable TSRS for this ac */
	param.content.txDataQosParams.acID = acID;
	param.content.txDataQosParams.tsrsArrLen = 0;
	param.paramType = CTRL_DATA_TSRS_PARAM;
	ctrlData_setParam(pQosMngr->hCtrlData, &param);

	/* restore default MSDU lifetime value */
	param.content.txDataQosParams.acID = acID;
	param.content.txDataQosParams.acTrfcCtrl.MsduLifeTime = pQosMngr->acParams[acID].msduLifeTimeParam;
	param.paramType = TX_DATA_CONFIG_AC_MSDU_LIFE_TIME;
	txData_setParam(pQosMngr->hTxData, &param);

	/* stop TS metrix for this ac */
#ifdef EXC_MODULE_INCLUDED
	measurementMgr_disableTsMetrics(pQosMngr->hMeasurementMngr, acID);
#endif

	/* update medium time and rate adaptation event only when init admission bit was 0 */
	if( pQosMngr->acParams[acID].apInitAdmissionState == ADMISSION_REQUIRED )
	{
		/* update currentTspecInfo parameters */
		pQosMngr->resourceMgmtTable.currentTspecInfo[acID].trafficAdmState = AC_NOT_ADMITTED;
		
		/* set params to TX */
		txData_setAdmisionCtrlParams(pQosMngr->hTxData,
									acID,
									pQosMngr->resourceMgmtTable.currentTspecInfo[acID].mediumTime , 
									pQosMngr->resourceMgmtTable.currentTspecInfo[acID].minimumPHYRate, FALSE);
	}
	else
		pQosMngr->resourceMgmtTable.currentTspecInfo[acID].trafficAdmState = AC_ADMITTED;

    /* After we have updated the TxData with the new status of the UP, we can zero the userPriority field */
    pQosMngr->resourceMgmtTable.currentTspecInfo[acID].userPriority = INACTIVE_USER_PRIORITY;

	/* set PS mode according to the PS mode from the association */
    /* restore the current Ps mode per AC to UPSD ONLY IF both the station and AP support UPSD */
    if ((pQosMngr->currentPsMode == PS_SCHEME_UPSD_TRIGGER) && (pQosMngr->acParams[acID].desiredWmeAcPsMode == PS_SCHEME_UPSD_TRIGGER))
    {
	  pQosMngr->acParams[acID].currentWmeAcPsMode = PS_SCHEME_UPSD_TRIGGER;
    }
    else
    {
	  pQosMngr->acParams[acID].currentWmeAcPsMode = PS_SCHEME_LEGACY_PSPOLL;
    }

	if(acID == QOS_AC_VO)
	{
		/* update TX with "new" PS Voice Delivery mode */
		txData_setPsVoiceDeliveryMode(pQosMngr->hTxData,pQosMngr->acParams[QOS_AC_VO].currentWmeAcPsMode);
		pQosMngr->voiceTspecConfigured = FALSE;
	}
	
	if (acID == QOS_AC_VI) 
	{
		pQosMngr->videoTspecConfigured = FALSE;
	}

	/* UPSD_FW - open comment in UPSD FW integration */
	 
	/* UPSD configuration */
	pQosMngr->acParams[acID].QtrafficParams.psScheme = pQosMngr->acParams[acID].currentWmeAcPsMode;
	verifyAndConfigTrafficParams(pQosMngr,&(pQosMngr->acParams[acID].QtrafficParams));


}

/*-----------------------------------------------------------------------------
Routine Name: qosMngr_sendUnexpectedTSPECResponse
Routine Description: send event to user application, informing of unexpected TSPEC response
					 which might imply loss of UPSD mode synch between AP and STA
Arguments: pTspecInfo - contains unexpected TSPEC response information 
Return Value:
-----------------------------------------------------------------------------*/
TI_STATUS qosMngr_sendUnexpectedTSPECResponseEvent(TI_HANDLE	hQosMngr,
								   tspecInfo_t	*pTspecInfo)
{
	OS_802_11_QOS_TSPEC_PARAMS addtsReasonCode;
    qosMngr_t *pQosMngr =	(qosMngr_t *)hQosMngr;

	/* set the event params */
	addtsReasonCode.uAPSDFlag = pTspecInfo->UPSDFlag;
	addtsReasonCode.uUserPriority = pTspecInfo->userPriority;
	addtsReasonCode.uNominalMSDUsize = pTspecInfo->nominalMsduSize;
	addtsReasonCode.uMeanDataRate = pTspecInfo->meanDataRate;	
	addtsReasonCode.uMinimumPHYRate = pTspecInfo->minimumPHYRate;
	addtsReasonCode.uSurplusBandwidthAllowance = pTspecInfo->surplausBwAllowance;
	addtsReasonCode.uMediumTime = pTspecInfo->mediumTime;

    addtsReasonCode.uReasonCode = pTspecInfo->statusCode + TSPEC_RESPONSE_UNEXPECTED;
		
	/* send event */
	EvHandlerSendEvent(pQosMngr->hEvHandler, IPC_EVENT_TSPEC_STATUS, (UINT8*)(&addtsReasonCode), sizeof(OS_802_11_QOS_TSPEC_PARAMS));

	return OK;
}

/************************************************************************
 *                        qosMngr_setAdmissionInfo                      *
 ************************************************************************
DESCRIPTION: This function is API function.
            the trafficAdmCtrl object calls this function in
            order to update the QOSMngr on TSPEC request status

INPUT:      hQosMngr                 - Qos Manager handle.
            pTspecInfo               - The TSPEC Parameters
            trafficAdmRequestStatus  - the status of the request
OUTPUT:     

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS qosMngr_setAdmissionInfo(TI_HANDLE    hQosMngr, 
                                   UINT8        acID,
                                   tspecInfo_t  *pTspecInfo,
                                   trafficAdmRequestStatus_e trafficAdmRequestStatus)
{
    UINT32                 actualMediumTime;
    OS_802_11_QOS_TSPEC_PARAMS addtsReasonCode;
    qosMngr_t *pQosMngr =  (qosMngr_t *)hQosMngr;
    whalCtrl_setTemplate_t templateStruct;
    QosNullDataTemplate_t  QosNullDataTemplate;

    /* Check if the updated AC is in WAIT state */
    if(pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].trafficAdmState != AC_WAIT_ADMISSION)
    {
        WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
             ("qosMngr_setAdmissionInfo: acID = %d, trafficAdmState != WAIT. IGNORE !!!\n", acID)); 
        
        return NOK;
    }

    if (pQosMngr->TSPECNegotiationResultCallb != NULL)
    {
        pQosMngr->TSPECNegotiationResultCallb (pQosMngr->TSPECNegotiationResultModule, trafficAdmRequestStatus);
        pQosMngr->TSPECNegotiationResultCallb = NULL;
        pQosMngr->TSPECNegotiationResultModule = NULL;
    }

    switch(trafficAdmRequestStatus)
    {
    case STATUS_TRAFFIC_ADM_REQUEST_ACCEPT:
        /* Received admission response with status accept */

        WLAN_REPORT_INFORMATION(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
                ("qosMngr_setAdmissionInfo: admCtrl status =  REQUEST_ACCEPT [ acID = %d, mediumTime = %d, minimumPHYRate = %d ]\n",
                acID, pTspecInfo->mediumTime, pTspecInfo->minimumPHYRate)); 
        
        /* Set the event params */
        addtsReasonCode.uAPSDFlag = pTspecInfo->UPSDFlag;
        addtsReasonCode.uUserPriority = pTspecInfo->userPriority;
        addtsReasonCode.uNominalMSDUsize = pTspecInfo->nominalMsduSize;
        addtsReasonCode.uMeanDataRate = pTspecInfo->meanDataRate;   
        addtsReasonCode.uMinimumPHYRate = pTspecInfo->minimumPHYRate;
        addtsReasonCode.uSurplusBandwidthAllowance = pTspecInfo->surplausBwAllowance;
        addtsReasonCode.uMediumTime = pTspecInfo->mediumTime;

        /* Free the candidate parameters */
        pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].trafficAdmState = AC_NOT_ADMITTED;

        /* Validate tid matching */
        if (pTspecInfo->tid == pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].tid)
        {
            addtsReasonCode.uReasonCode = ADDTS_RESPONSE_ACCEPT;
            
            /* Send event */
            EvHandlerSendEvent (pQosMngr->hEvHandler, 
                                IPC_EVENT_TSPEC_STATUS, 
                                (UINT8*)&addtsReasonCode, 
                                sizeof(OS_802_11_QOS_TSPEC_PARAMS));
        }
        else
        {
            addtsReasonCode.uReasonCode = ADDTS_RESPONSE_AP_PARAM_INVALID;
            
            /* Send event */
            EvHandlerSendEvent (pQosMngr->hEvHandler, 
                                IPC_EVENT_TSPEC_STATUS, 
                                (UINT8*)&addtsReasonCode, 
                                sizeof(OS_802_11_QOS_TSPEC_PARAMS));
            return OK;
        }

        /* Update the current TSPEC parameters from the received TSPEC */
        os_memoryCopy (pQosMngr->hOs, 
                       &pQosMngr->resourceMgmtTable.currentTspecInfo[acID], 
                       pTspecInfo, 
                       sizeof(tspecInfo_t));

        /* Set the TSPEC to admitted */
        pQosMngr->resourceMgmtTable.currentTspecInfo[acID].trafficAdmState = AC_ADMITTED;
        
        /* Update total medium time */
        pQosMngr->resourceMgmtTable.totalAllocatedMediumTime += pTspecInfo->mediumTime;

        /*
         * Set QOS Null-data template into the firmware. 
         * When a new TSPEC with UPSD is "accepted" by the AP, 
         * we set the user priority of it into the firmware. 
         * Since this AC is already ADMITTED (we are processing the successful response), 
         * it is OK to set the qos null data template with this UP 
         */
        if (addtsReasonCode.uAPSDFlag == PS_SCHEME_UPSD_TRIGGER && 
            pQosMngr->QosNullDataTemplateUserPriority == 0xFF)
        {
            /* Remember the user priority which we have set */
            pQosMngr->QosNullDataTemplateUserPriority = (UINT8)addtsReasonCode.uUserPriority;

            templateStruct.pTemplate = (UINT8 *)&QosNullDataTemplate;
            templateStruct.templateType = QOS_NULL_DATA_TEMPLATE;
            buildQosNullDataTemplate (pQosMngr->hSiteMgr, &templateStruct, pQosMngr->QosNullDataTemplateUserPriority);
            whalCtrl_SetTemplate (pQosMngr->hHalCtrl, &templateStruct);

            WLAN_REPORT_INFORMATION(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
                ("qosMngr_setAdmissionInfo: Setting QOS null data for UserPriority=%d (due to TSPEC ACCEPT response)\n",
                addtsReasonCode.uUserPriority));
        }

        /* Set params to TX */
        /*------------------*/

        /* Update medium time and rate adaptation event only when init admission bit was 0 */
        if (pQosMngr->acParams[acID].apInitAdmissionState == ADMISSION_REQUIRED)
        {
            /* mediumTime is in units of 32uSec and we work in mSec */
            actualMediumTime = (pTspecInfo->mediumTime * 32) / 1000;

            /* Set TX params */
            txData_setAdmisionCtrlParams (pQosMngr->hTxData,
                                          acID,
                                          actualMediumTime, 
                                          pTspecInfo->minimumPHYRate, 
                                          TRUE);
        }
        
        {
            PSScheme_e psMode = pTspecInfo->UPSDFlag ? PS_SCHEME_UPSD_TRIGGER 
                                                     : PS_SCHEME_LEGACY_PSPOLL; 
       
            if (pQosMngr->acParams[acID].currentWmeAcPsMode != psMode)
            {
                TI_STATUS status;

                pQosMngr->acParams[acID].currentWmeAcPsMode = psMode;

                /* UPSD_FW - open comment in UPSD FW integration */
                pQosMngr->acParams[acID].QtrafficParams.psScheme = pQosMngr->acParams[acID].currentWmeAcPsMode;
                status = verifyAndConfigTrafficParams (pQosMngr, &pQosMngr->acParams[acID].QtrafficParams);
                if (status != OK)
                    return status;

                if (acID == QOS_AC_VO)
                {
                    /* Update TX with "new" PS Voice Delivery mode */
                    txData_setPsVoiceDeliveryMode (pQosMngr->hTxData, pQosMngr->acParams[QOS_AC_VO].currentWmeAcPsMode);
                }
            }
        }   
        break;

    case STATUS_TRAFFIC_ADM_REQUEST_REJECT:
        /* Received admission response with status reject */

        WLAN_REPORT_INFORMATION(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
             ("qosMngr_setAdmissionInfo: admCtrl status = REQUEST_REJECT [ acID = %d ]\n", acID)); 
        
        /* Validate tid matching */
        if (pTspecInfo->tid == pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].tid)
        {
            addtsReasonCode.uReasonCode = pTspecInfo->statusCode;
        }
        else
        {
            addtsReasonCode.uReasonCode = ADDTS_RESPONSE_AP_PARAM_INVALID;
        }

        // put the candidateTspec back to a previously accepted TSPEC if
        // one exist.  This will prevent TI from sending same rejected
        // TSPEC when roaming to another AP that has the same configuration
        // as our current AP (under which case our the new AP will likely reject
        // the TSPEC as well and we will not be able to roam
        os_memoryCopy (pQosMngr->hOs, 
                       &pQosMngr->resourceMgmtTable.candidateTspecInfo[acID], 
                       &pQosMngr->resourceMgmtTable.currentTspecInfo[acID], 
                       sizeof(tspecInfo_t));
        if( pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].userPriority == INACTIVE_USER_PRIORITY ) {
            if( acID == QOS_AC_VO ) pQosMngr->voiceTspecConfigured = FALSE;
            else if( acID == QOS_AC_VI ) pQosMngr->voiceTspecConfigured = FALSE;
        }


        /* Send event to application */
        addtsReasonCode.uAPSDFlag = pTspecInfo->UPSDFlag;
        addtsReasonCode.uUserPriority = pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].userPriority;
        addtsReasonCode.uNominalMSDUsize = pTspecInfo->nominalMsduSize;
        addtsReasonCode.uMeanDataRate = pTspecInfo->meanDataRate;   
        addtsReasonCode.uMinimumPHYRate = pTspecInfo->minimumPHYRate;
        addtsReasonCode.uSurplusBandwidthAllowance = pTspecInfo->surplausBwAllowance;
        addtsReasonCode.uMediumTime = pTspecInfo->mediumTime;
    
        EvHandlerSendEvent (pQosMngr->hEvHandler, 
                            IPC_EVENT_TSPEC_STATUS, 
                            (UINT8*)&addtsReasonCode, 
                            sizeof(OS_802_11_QOS_TSPEC_PARAMS));       
        break;

    case STATUS_TRAFFIC_ADM_REQUEST_TIMEOUT:
        WLAN_REPORT_INFORMATION(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
             ("qosMngr_setAdmissionInfo: admCtrl status = REQUEST_TIMEOUT [ acID = %d ]\n", acID)); 
        
        /* Free the candidate parameters */
        pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].trafficAdmState = AC_NOT_ADMITTED;

        /* Send event to application */
        addtsReasonCode.uUserPriority = pQosMngr->resourceMgmtTable.candidateTspecInfo[acID].userPriority;
        addtsReasonCode.uReasonCode = ADDTS_RESPONSE_TIMEOUT;
        addtsReasonCode.uAPSDFlag = 0;
        addtsReasonCode.uNominalMSDUsize = 0;
        addtsReasonCode.uMeanDataRate = 0;  
        addtsReasonCode.uMinimumPHYRate = 0;
        addtsReasonCode.uSurplusBandwidthAllowance = 0;
        addtsReasonCode.uMediumTime = 0;

        EvHandlerSendEvent (pQosMngr->hEvHandler, 
                            IPC_EVENT_TSPEC_STATUS, 
                            (UINT8*)&addtsReasonCode, 
                            sizeof(OS_802_11_QOS_TSPEC_PARAMS));       
        break;
       
    default:
        WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
            ("qosMngr_setAdmissionInfo: receive state from admCtrl = unknown !!! \n")); 
        break;
    }

    return OK;  
}

/************************************************************************
 *                    QosMngr_receiveActionFrames                       *
 ************************************************************************
DESCRIPTION: 
                                                                                                 
RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS QosMngr_receiveActionFrames(TI_HANDLE hQosMngr, UINT8* pData, UINT8 action, UINT32 bodyLen)
{
	UINT8					acID;
	tsInfo_t				tsInfo;
	UINT8					userPriority;
	dot11_WME_TSPEC_IE_t*	dot11_WME_TSPEC_IE;
    OS_802_11_QOS_TSPEC_PARAMS addtsReasonCode;


    qosMngr_t *pQosMngr = (qosMngr_t *)hQosMngr;

	/* check if STA is already connected to AP */
	if( (pQosMngr->isConnected == FALSE) || 
		(pQosMngr->activeProtocol != WME) || 
		(pQosMngr->trafficAdmCtrlEnable == FALSE) )
	{
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG,("QosMngr_receiveActionFrames:  Ignore  !!!"));
		return NOK;
	}

	/* check DELTS action code */
	if (action == DELTS_ACTION) 
	{
		/* parse the frame */
		pData++;	/* DIALOG TOKEN  */
		pData++;	/* STATUS CODE */

#if 0 /*  Need to send TSPEC IE in DELTS or only the tsinfo ??*/
		/*  only tsinfo   */
		tsInfo.tsInfoArr[0] = *pData;
		pData++;
		tsInfo.tsInfoArr[1] = *pData;
		pData++;
		tsInfo.tsInfoArr[2] = *pData;
		pData++;
#else
		/*  TSpec IE in DELTS*/
		dot11_WME_TSPEC_IE = (dot11_WME_TSPEC_IE_t*)pData;
		tsInfo.tsInfoArr[0] = dot11_WME_TSPEC_IE->tHdr.tsInfoField.tsInfoArr[0];
		pData++;
		bodyLen--;
		tsInfo.tsInfoArr[1] = dot11_WME_TSPEC_IE->tHdr.tsInfoField.tsInfoArr[1];
		pData++;
		bodyLen--;
		tsInfo.tsInfoArr[2] = dot11_WME_TSPEC_IE->tHdr.tsInfoField.tsInfoArr[2];
#endif
		
        userPriority = (((tsInfo.tsInfoArr[1]) & TS_INFO_1_USER_PRIORITY_MASK) >> USER_PRIORITY_SHIFT);
		
		acID = WMEQosTagToACTable[userPriority];
		

		WLAN_REPORT_INFORMATION(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
				("QosMngr_receiveActionFrames: DELTS [ acID = %d ] \n", acID)); 


		/* check if this AC is admitted with the correct userPriority */
		if( (pQosMngr->resourceMgmtTable.currentTspecInfo[acID].trafficAdmState == AC_ADMITTED) &&
			( pQosMngr->resourceMgmtTable.currentTspecInfo[acID].userPriority == userPriority) )
		{
			deleteTspecConfiguration(pQosMngr, acID);

            /* Send event to notify DEL_TS */
            addtsReasonCode.uAPSDFlag = 0;
		    addtsReasonCode.uUserPriority = userPriority;
            addtsReasonCode.uReasonCode = TSPEC_DELETED_BY_AP;
		    addtsReasonCode.uNominalMSDUsize = 0;
		    addtsReasonCode.uMeanDataRate = 0;	
		    addtsReasonCode.uMinimumPHYRate = 0;
		    addtsReasonCode.uSurplusBandwidthAllowance = 0;
		    addtsReasonCode.uMediumTime = 0;

            EvHandlerSendEvent(pQosMngr->hEvHandler, IPC_EVENT_TSPEC_STATUS, (UINT8*)(&addtsReasonCode), sizeof(OS_802_11_QOS_TSPEC_PARAMS));
		}
		else
		{
			WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
				("QosMngr_receiveActionFrames: DELTS [ acID = %d userPriority = %d  currentUserPriority = %d] Current State in not ADMITED !! \n", acID, userPriority,pQosMngr->resourceMgmtTable.currentTspecInfo[acID].userPriority)); 
			
		}
	}
	/* if action code is ADDTS call trafficAdmCtrl object API function */
	else if (action == ADDTS_RESPONSE_ACTION) 
	{
		if (trafficAdmCtrl_recv(pQosMngr->pTrafficAdmCtrl, pData, action) == OK)
		{
#ifdef EXC_MODULE_INCLUDED
			/* Check if EXC IEs present, if so, parse them and update relevant modules; 
               skip the TSPEC IE;
               do not forget 2 bytes of status and dialog code that must be skipped as well */
			EXCv4IEs_t			excIE;
			UINT32 				readLen;

			excIE.edcaLifetimeParameter = NULL;
			excIE.trafficStreamParameter = NULL;
			excIE.tsMetrixParameter = NULL;

			userPriority = ((((dot11_WME_TSPEC_IE_t *)(pData+2))->tHdr.tsInfoField.tsInfoArr[1] & TS_INFO_1_USER_PRIORITY_MASK) >> USER_PRIORITY_SHIFT);
			acID = WMEQosTagToACTable[userPriority];

			readLen = ((dot11_eleHdr_t *)(pData+2))->eleLen;

			/* 4 stands for 1 byte of token+1 byte of status+1 byte of EID+1 byte of len */
			bodyLen = bodyLen - 4 - readLen; 
			pData = pData + 4 + readLen;

			while (bodyLen) 
			{
				mlmeParser_readExcOui(pData, bodyLen, &readLen, &excIE);
				bodyLen -= readLen;
				pData += readLen;
			}

			excMngr_setEXCQoSParams(pQosMngr->hExcMgr, &excIE, acID);
#endif
		}
	}
	else
	{
		WLAN_REPORT_WARNING(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
           ("QosMngr_receiveActionFrames: Receive unknown action code = %d  -> Ignore !! \n",action));
	}
	
	return OK;
}

/************************************************************************
 *                        qosMngr_getCurrAcStatus     		            *
 ************************************************************************
DESCRIPTION: This function is API fuunction for getting tha AC status .

INPUT:      hQosMngr	             - Qos Manager handle.
			pAcStatusParams

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
static TI_STATUS qosMngr_getCurrAcStatus(TI_HANDLE hQosMngr, OS_802_11_AC_UPSD_STATUS_PARAMS *pAcStatusParams)
{
    qosMngr_t *pQosMngr = (qosMngr_t *)hQosMngr;

	/* check AC validity */
	if( pAcStatusParams->uAC > MAX_NUM_OF_AC )
	{	
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
           ("qosMngr_getCurrAcStatus: acID > 3 -> Ignore !!!"));
		return NOK;
	}

	/* check if sta is connected to AP */
	if(pQosMngr->isConnected == FALSE)
	{	
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
           ("qosMngr_getCurrAcStatus: pQosMngr->connected == FALSE -> Ignore !!!"));
		return NOT_CONNECTED;
	}
	
	 /* check if AP support WME */
	if(pQosMngr->activeProtocol != WME)
	{	
		WLAN_REPORT_ERROR(pQosMngr->hReport, QOS_MANAGER_MODULE_LOG, 
           ("qosMngr_getCurrAcStatus: activeProtocol != WME -> Ignore !!!"));
		return NO_QOS_AP;
	}

	pAcStatusParams->uCurrentUAPSDStatus = pQosMngr->acParams[pAcStatusParams->uAC].currentWmeAcPsMode;
	pAcStatusParams->pCurrentAdmissionStatus = pQosMngr->resourceMgmtTable.currentTspecInfo[pAcStatusParams->uAC].trafficAdmState;

	return OK;
}



/************************************************************************
 *                        setNonQosAdmissionState  		                *
 ************************************************************************
DESCRIPTION: This function resets the admission state variables as required
				for non-QoS mode and configures the Tx module.

INPUT:      pQosMngr	- Qos Manager pointer.
			acId		- the AC to update.

OUTPUT:		

RETURN:     

************************************************************************/

static void setNonQosAdmissionState(qosMngr_t *pQosMngr, UINT8 acID)
{
	paramInfo_t	param;

	if(acID == QOS_AC_BE)
	{
		pQosMngr->acParams[acID].apInitAdmissionState = ADMISSION_NOT_REQUIRED;
		param.content.txDataQosParams.qosParams.admissionRequired = ADMISSION_NOT_REQUIRED; 
		

		pQosMngr->resourceMgmtTable.currentTspecInfo[acID].trafficAdmState = AC_ADMITTED;
		param.content.txDataQosParams.qosParams.admissionState = AC_ADMITTED; 
	}
	else
	{
		pQosMngr->acParams[acID].apInitAdmissionState = ADMISSION_REQUIRED;
		param.content.txDataQosParams.qosParams.admissionRequired = ADMISSION_REQUIRED; 
		

		pQosMngr->resourceMgmtTable.currentTspecInfo[acID].trafficAdmState = AC_NOT_ADMITTED;
		param.content.txDataQosParams.qosParams.admissionState = AC_NOT_ADMITTED; 
	}

	param.content.txDataQosParams.acID = acID;
	param.paramType = TX_DATA_AC_ADMISSION_STATE;
	
	txData_setParam(pQosMngr->hTxData,&param);
}

static void qosMngr_storeTspecCandidateParams (tspecInfo_t *pCandidateParams, OS_802_11_QOS_TSPEC_PARAMS *pTSPECParams, UINT8 ac)
{
	pCandidateParams->AC = (acTrfcType_e)ac;
	pCandidateParams->tid = (UINT8)pTSPECParams->uUserPriority;
	pCandidateParams->userPriority = (UINT8)pTSPECParams->uUserPriority;
	pCandidateParams->meanDataRate = pTSPECParams->uMeanDataRate;
	pCandidateParams->nominalMsduSize = (UINT16)pTSPECParams->uNominalMSDUsize;
	pCandidateParams->UPSDFlag = (BOOL)pTSPECParams->uAPSDFlag;
	pCandidateParams->surplausBwAllowance = (UINT16)pTSPECParams->uSurplusBandwidthAllowance;
	pCandidateParams->minimumPHYRate = pTSPECParams->uMinimumPHYRate;
	pCandidateParams->streamDirection = BI_DIRECTIONAL;
	pCandidateParams->mediumTime = 0;
}
