/** \file connApi.h
 *  \brief connection module API
 *
 *  \see conn.c
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
/*																									*/
/*	  MODULE:	connApi.h																*/
/*    PURPOSE:	connection module API			 								*/
/*																									*/
/***************************************************************************/
#ifndef __CONN_API_H__
#define __CONN_API_H__

#include "osTIType.h"
#include "paramOut.h"
#include "paramIn.h"
#include "802_11Defs.h"
#include "mlmeApi.h"

/*
 * Connection type enum
 */
typedef enum{
	CONN_TYPE_FIRST_CONN    = 0,  /* Standart 802.11 association */
	CONN_TYPE_ROAM           /* Perform roaming connection. (Re Association) */ 
}connType_e;


/* 
	Prototype for connection status announcment, this function is called upon connection
   lost or connection establishment.
*/
typedef TI_STATUS (*conn_status_callback_t)( TI_HANDLE hObj, mgmtStatus_e status, UINT32 uStatusCode);


/* Connection interface functions prototypes */

TI_HANDLE conn_create(TI_HANDLE hOs);

TI_STATUS conn_config(TI_HANDLE 	hConn, 
				   TI_HANDLE 	hSiteMgr, 
				   TI_HANDLE	hSmeSm, 
				   TI_HANDLE	hMlmeSm, 
				   TI_HANDLE	hRsn,
				   TI_HANDLE	hRxData,
				   TI_HANDLE	hTxData,
				   TI_HANDLE 	hReport,
				   TI_HANDLE 	hOs,
				   TI_HANDLE	hPwrMngr,
				   TI_HANDLE    hCtrlData,
				   TI_HANDLE	hMeasurementMgr,
				   TI_HANDLE	hTrafficMonitor,
				   TI_HANDLE	hScr,
				   TI_HANDLE	hExcMngr,
				   TI_HANDLE	hQosMngr,
				   TI_HANDLE	hHalCtrl,
				   TI_HANDLE	hScanCnc,
				   TI_HANDLE	hCurrBss,
				   TI_HANDLE	hSwitchChannel,
				   TI_HANDLE	hEvHandler,
				   TI_HANDLE	hHealthMonitor,
				   TI_HANDLE	hMacServices,
                   TI_HANDLE    hRegulatoryDomain,
				   TI_HANDLE    hSoftGemini,
				   connInitParams_t		*pConnInitParams);

TI_STATUS conn_unLoad(TI_HANDLE hConn);

TI_STATUS conn_setParam(TI_HANDLE		hConn, 
					 paramInfo_t	*pParam);

TI_STATUS conn_getParam(TI_HANDLE		hConn, 
					 paramInfo_t	*pParam);

TI_STATUS conn_start(TI_HANDLE hConn ,connType_e connType,
					 	conn_status_callback_t  pConnStatusCB,
						TI_HANDLE connStatCbObj,
						BOOL disConEraseKeys,
						BOOL reNegotiateTspec);
void connInfraJoinCmdCmpltNotification(TI_HANDLE CB_handle);

TI_STATUS conn_stop(TI_HANDLE hConn, disConnType_e disConnType,
					mgmtStatus_e 			reason,
					BOOL					disConEraseKeys,
					conn_status_callback_t  pConnStatusCB,
					TI_HANDLE				connStatCbObj );


TI_STATUS conn_ibssStaJoined(TI_HANDLE hConn);

void conn_disConnFrameSentCBFunc(TI_HANDLE hConn);

void conn_ibssPrintStatistics(TI_HANDLE hConn);

#endif /* __CONN_API_H__ */
