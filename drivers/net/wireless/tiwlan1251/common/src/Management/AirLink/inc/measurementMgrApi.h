
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
/*	  MODULE:	measurementMgrApi.h										   */
/*    PURPOSE:	Measurement Manager module interface header file		   */
/*																		   */
/***************************************************************************/





#ifndef __MEASUREMENTMGR_API_H__
#define __MEASUREMENTMGR_API_H__




#include "measurementTypes.h"
#include "scrApi.h"
#include "mlmeApi.h"







TI_HANDLE measurementMgr_create(TI_HANDLE hOs);

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
								measurementInitParams_t * pMeasurementInitParams);

TI_STATUS measurementMgr_destroy(TI_HANDLE hMeasurementMgr);

TI_STATUS measurementMgr_setParam(TI_HANDLE hMeasurementMgr, paramInfo_t * pParam);

TI_STATUS measurementMgr_getParam(TI_HANDLE hMeasurementMgr, paramInfo_t * pParam);

TI_STATUS measurementMgr_connected(TI_HANDLE hMeasurementMgr);

TI_STATUS measurementMgr_disconnected(TI_HANDLE hMeasurementMgr);

TI_STATUS measurementMgr_enable(TI_HANDLE hMeasurementMgr);

TI_STATUS measurementMgr_disable(TI_HANDLE hMeasurementMgr);

TI_STATUS measurementMgr_setMeasurementMode(TI_HANDLE hMeasurementMgr, UINT16 capabilities, 
											UINT8 * pIeBuffer, UINT16 length);

TI_STATUS measurementMgr_receiveFrameRequest(TI_HANDLE hMeasurementMgr, measurement_frameType_e frameType,
											INT32 dataLen, UINT8 * pData);

void measurementMgr_rejectPendingRequests(TI_HANDLE hMeasurementMgr, measurement_rejectReason_e rejectReason);

void measurementMgr_MeasurementCompleteCB(TI_HANDLE clientObj, measurement_reply_t * msrReply);

void measurementMgr_scrResponseCB(TI_HANDLE hClient, scr_clientRequestStatus_e requestStatus,
								scr_pendReason_e pendReason);

void measurementMgr_mlmeResultCB(TI_HANDLE hMeasurementMgr, macAddress_t * bssid, mlmeFrameInfo_t * frameInfo, 
								 Rx_attr_t * pRxAttr, UINT8 * buffer, UINT16 bufferLength);


#endif	/* __MEASUREMENTMGR_API_H__ */
