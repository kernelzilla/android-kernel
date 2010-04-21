/** \file qosMngr_API.h
 *  \brief QOS manager module external header file
 *
 *  \see qosMngr.c
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

/***************************************************************************/
/*																		   */
/*	  MODULE:	qosMgr_API.h						   	  				   */
/*    PURPOSE:	QOS manager module external header file	   				   */
/*																		   */
/***************************************************************************/
#ifndef __QOS_MNGR_API_H__
#define __QOS_MNGR_API_H__

#include "802_11Defs.h"

#define MAX_NUM_OF_TID						16 
#define FIRST_TID							8
 
#define USER_PRIORITY_4 (WMEQosTagToACTable[4])
#define USER_PRIORITY_6 (WMEQosTagToACTable[6])


typedef struct 
{
	acTrfcType_e		AC;
	UINT8				tid;
	UINT8				userPriority;
	UINT16				nominalMsduSize; /* in bytes */
	UINT32				meanDataRate;	 /* bits per second */
	UINT16				surplausBwAllowance;
	INT32				minimumPHYRate;
	UINT16				mediumTime;
	BOOL				UPSDFlag;
	streamDirection_e	streamDirection;
	trafficAdmState_e	trafficAdmState;
	UINT8				statusCode;
}tspecInfo_t;

typedef enum 
{
	STATUS_TRAFFIC_ADM_REQUEST_ACCEPT    	= 0,
    STATUS_TRAFFIC_ADM_REQUEST_REJECT		= 1,
   	STATUS_TRAFFIC_ADM_REQUEST_TIMEOUT		= 2,
} trafficAdmRequestStatus_e;


TI_HANDLE qosMngr_create(TI_HANDLE hOs);

TI_STATUS qosMngr_destroy(TI_HANDLE hQosMngr);

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
						   QosMngrInitParams_t		*pQosMngrInitParams);

TI_STATUS qosMngr_disconnect(TI_HANDLE hQosMngr);

TI_STATUS qosMngr_connect(TI_HANDLE hQosMngr);

TI_STATUS qosMngr_setParams(TI_HANDLE  hQosMngr,paramInfo_t *pParamInfo);

TI_STATUS qosMngr_getParams(TI_HANDLE  hQosMngr,paramInfo_t *pParamInfo);

TI_STATUS qosMngr_assocReqBuild(TI_HANDLE  hQosMngr, UINT8 *pQosIe, UINT8 *pLen);

TI_STATUS qosMngr_setSite(TI_HANDLE hQosMngr, assocRsp_t *assocRsp);

void      qosMngr_updateIEinfo(TI_HANDLE hQosMngr, UINT8 *pQosIeParams,qosProtocols_e qosSetProtocol);

UINT8 qosMngr_evalSite(TI_HANDLE hQosMngr, BOOL siteAPSDSupport);

TI_STATUS qosMngr_getQosCapabiltyInfeElement(TI_HANDLE  hQosMngr, UINT8 *pQosIe, UINT8 *pLen);

TI_STATUS qosMngr_requestAdmission(TI_HANDLE			hQosMngr, 
                                   OS_802_11_QOS_TSPEC_PARAMS *addTspecParams);

TI_STATUS qosMngr_deleteAdmission(TI_HANDLE hQosMngr, OS_802_11_QOS_DELETE_TSPEC_PARAMS *delAdmissionParams);

TI_STATUS qosMngr_selectActiveProtocol(TI_HANDLE  hQosMngr);

TI_STATUS qosMngr_setAcPsDeliveryMode(TI_HANDLE  hQosMngr);

TI_STATUS qosMngr_sendUnexpectedTSPECResponseEvent(TI_HANDLE	hQosMngr,
								   tspecInfo_t	*pTspecInfo);

TI_STATUS qosMngr_setAdmissionInfo(TI_HANDLE	hQosMngr, 
								   UINT8		acID,
								   tspecInfo_t	*pTspecInfo,
								   trafficAdmRequestStatus_e trafficAdmRequestStatus);



TI_STATUS QosMngr_receiveActionFrames(TI_HANDLE hQosMngr, UINT8* pData, UINT8 action, UINT32 bodyLen);

void qosMngr_checkTspecRenegResults(TI_HANDLE hQosMngr, assocRsp_t *assocRsp);

UINT32 qosMngr_buildTSPec(TI_HANDLE hQosMngr, UINT32 user_priority, UINT8 *pQosIe);

#endif /* __QOS_MNGR_API_H__ */

