/** \file mlmeApi.h
 *  \brief MLME API
 *
 *  \see mlmeSm.c
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
/*																			*/
/*	  MODULE:	mlmeApi.h													*/
/*    PURPOSE:	MLME API			 										*/
/*																			*/
/***************************************************************************/
#ifndef __MLME_API_H__
#define __MLME_API_H__

#include "osTIType.h"

#include "paramOut.h"

#include "802_11Defs.h"
#include "memMngrEx.h"

#include "whalBus_Defs.h"

/* Constants */

/* Enumerations */

typedef enum
{
	MSG_BROADCAST,
	MSG_MULTICAST,
	MSG_UNICAST
} mlmeMsgDestType_t;

/*
 * Disconnecting type enum
 */
typedef enum{
	DISCONN_TYPE_IMMEDIATE     = 0,  /* Disconnect without disassociation. */
	DISCONN_TYPE_DISASSOC,           /* Send disassocciation frame upon disconnecting */
	DISCONN_TYPE_DEAUTH              /* Send disassocciation notification upon disconnecting */
}disConnType_e;

/* Typedefs */


typedef struct 
{
	dot11MgmtSubType_e subType;

	union 
	{
		beacon_probeRsp_t iePacket;
		disAssoc_t	disAssoc;
		assocRsp_t 	assocRsp;
		authMsg_t	auth;
		deAuth_t	deAuth;
		action_t	action;
	} content;

	union
	{
		mlmeMsgDestType_t	destType;
	} extesion;

} mlmeFrameInfo_t;

typedef struct 
{
	dot11_SSID_t 		ssid;
	macAddress_t        bssid;
    dot11_CHALLENGE_t   challenge;
    dot11_RATES_t 		rates;
    dot11_RATES_t 		extRates;
    dot11_FH_PARAMS_t 	fhParams;
    dot11_CF_PARAMS_t 	cfParams;
	dot11_DS_PARAMS_t 	dsParams;
    dot11_IBSS_PARAMS_t ibssParams;
    dot11_COUNTRY_t 	country;
    dot11_WME_PARAM_t 	WMEParams;
    dot11_POWER_CONSTRAINT_t powerConstraint;
    dot11_4X_t 			fourXParams;
    dot11_CHANNEL_SWITCH_t channelSwitch;
    dot11_QUIET_t 		quiet;
    dot11_TPC_REPORT_t 	TPCReport;
#ifdef EXC_MODULE_INCLUDED
	dot11_CELL_TP_t		cellTP;
#endif
    dot11_RSN_t 		rsnIe[3];
	dot11_TIM_t 		tim;
	dot11_QOS_CAPABILITY_IE_t	QosCapParams;
	UINT8 				rxChannel;
	UINT8 				band;
	BOOL 				myBssid;
	BOOL				myDst;
	BOOL				mySa;
	BOOL 				recvChannelSwitchAnnoncIE;
	mlmeFrameInfo_t		frame;
}mlmeIEParsingParams_t;

typedef void (*mlme_resultCB_t)( TI_HANDLE hObj, macAddress_t* bssid, mlmeFrameInfo_t* pFrameInfo,
                                 Rx_attr_t* pRxAttr, UINT8* frame, UINT16 frameLength );

/* External data definitions */

/* External functions definitions */

/* Function prototypes */

/* MLME SM API */

TI_HANDLE mlme_create(TI_HANDLE hOs);

TI_STATUS mlme_unload(TI_HANDLE hMlme);

TI_STATUS mlme_config(TI_HANDLE hMlme,
				   TI_HANDLE hAuth,
				   TI_HANDLE hAssoc,
				   TI_HANDLE hSiteMgr,
				   TI_HANDLE hCtrlData,
				   TI_HANDLE hConn,
				   TI_HANDLE hTxData,
				   TI_HANDLE hHalCtrl,
				   TI_HANDLE hMemMgr,
				   TI_HANDLE hMeasurementMgr,
				   TI_HANDLE hSwitchChannel,
				   TI_HANDLE hRegulatoryDomain,
				   TI_HANDLE hReport,
				   TI_HANDLE hOs,
				   TI_HANDLE hCurrBss,
				   TI_HANDLE hApConn,
				   TI_HANDLE hScanCncn,
				   TI_HANDLE hQosMngr,
                   TI_HANDLE hConfigMgr);

TI_STATUS mlme_setParam(TI_HANDLE			hMlmeSm,
						paramInfo_t			*pParam);

TI_STATUS mlme_getParam(TI_HANDLE			hMlmeSm, 
						paramInfo_t			*pParam);

TI_STATUS mlme_start(TI_HANDLE hMlme);

TI_STATUS mlme_stop(TI_HANDLE hMlme, disConnType_e disConnType, mgmtStatus_e reason);

TI_STATUS mlme_reportAuthStatus(TI_HANDLE hMlme, UINT16 status);

TI_STATUS mlme_reportAssocStatus(TI_HANDLE hMlme, UINT16 status);

/* MLME parser API */

TI_STATUS mlmeParser_recv(TI_HANDLE hMlme, mem_MSDU_T *pMSDU, Rx_attr_t* pRxAttr);

TI_STATUS mlmeParser_registerForBeaconAndProbeResp( TI_HANDLE hMlme, 
                                                    mlme_resultCB_t resultCBFunc, 
                                                    TI_HANDLE resultCBObj );

void mlmeParser_unregisterForBeaconAndProbeResp( TI_HANDLE hMlme );

TI_STATUS mlmeParser_parseIEs(TI_HANDLE *hMlme, 
							  UINT8 *pData,
							  INT32 bodyDataLen,
							  mlmeIEParsingParams_t *params);

#ifdef EXC_MODULE_INCLUDED
void mlmeParser_readExcOui (UINT8 *pData, 
							UINT32 dataLen, 
							UINT32 *pReadLen, 
							EXCv4IEs_t *excIEs);
#endif

mlmeIEParsingParams_t *mlmeParser_getParseIEsBuffer(TI_HANDLE *hMlme);

/* Association SM API */

TI_HANDLE assoc_create(TI_HANDLE pOs);

TI_STATUS assoc_unload(TI_HANDLE pAssoc);

TI_STATUS assoc_config(TI_HANDLE hAssoc,
					TI_HANDLE hMlme,
					TI_HANDLE hRegulatoryDomain,
					TI_HANDLE hSiteMgr,
					TI_HANDLE hCtrlData,
					TI_HANDLE hTxData,
					TI_HANDLE hHalCtrl,
					TI_HANDLE hRsn,
					TI_HANDLE hReport,
					TI_HANDLE hOs,
					TI_HANDLE hExcMngr,
					TI_HANDLE hQosMngr,
                    TI_HANDLE hMeasurementMgr,
					TI_HANDLE hApConn,
					assocInitParams_t	*pAssocInitParams);

TI_STATUS assoc_setParam(TI_HANDLE hCtrlData, paramInfo_t	*pParam);

TI_STATUS assoc_getParam(TI_HANDLE hCtrlData, paramInfo_t	*pParam);

/* Authentication SM API */

TI_HANDLE auth_create(TI_HANDLE hOs);

TI_STATUS auth_unload(TI_HANDLE hAuth);

TI_STATUS auth_config(TI_HANDLE	 		hAuth,
                   TI_HANDLE	 		hMlme,
				   TI_HANDLE			hRsn,
				   TI_HANDLE	 		hReport,
				   TI_HANDLE	 		hOs,
				   authInitParams_t		*pAuthInitParams);


TI_STATUS auth_setParam(TI_HANDLE hCtrlData, paramInfo_t	*pParam);

TI_STATUS auth_getParam(TI_HANDLE hCtrlData, paramInfo_t	*pParam);

#endif /* __MLME_API_H__*/
