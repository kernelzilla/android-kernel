/** \file smeSmApi.h
 *  \brief SME SM module API
 *
 *  \see smeSmApi.c
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
/*	  MODULE:	smeSmApi.h																*/
/*    PURPOSE:	SME SM API			 								*/
/*																									*/
/***************************************************************************/
#ifndef __SME_SM_API_H__
#define __SME_SM_API_H__

#include "osTIType.h"
#include "paramOut.h"
#include "paramIn.h"
#include "ScanCncnApi.h"

/* SME SM status definitions */

typedef enum
{
	CONN_STATUS_SUCCESS 	 = 0,
	CONN_STATUS_FAILURE 	 = 1,
	CONN_STATUS_RECONNECTING = 2,
} connStatus_e;

typedef enum
{
	SELECT_STATUS_SUCCESS 	= 0,
	SELECT_STATUS_FAILURE 	= 1,
} selectStatus_t;

/* SME SM interface functions prototypes */

/************************************************************
  Function: smeSm_create
  Description: 
 ************************************************************/ 
TI_HANDLE smeSm_create(TI_HANDLE hOs);

/************************************************************
  Function: smeSm_config
  Description: 
 ************************************************************/ 
TI_STATUS smeSm_config(TI_HANDLE		hSmeSm, 
					TI_HANDLE		hConn,
					TI_HANDLE		hScanCncn,
					TI_HANDLE		hSiteMgr,
					TI_HANDLE		hHalCtrl,
					TI_HANDLE		hReport,
					TI_HANDLE 		hOs,
                    TI_HANDLE		hEvHandler,
					TI_HANDLE		hScr,
					TI_HANDLE		hApConn,
					TI_HANDLE		hCurrBss,
					TI_HANDLE		hPowerMgr,
                    TI_HANDLE       hRegulatoryDomain,
					smeInitParams_t*	smeInitParams);

/************************************************************
  Function: smeSm_unLoad
  Description: 
 ************************************************************/ 
TI_STATUS smeSm_unLoad(TI_HANDLE		hSmeSm);

/************************************************************
  Function: smeSm_start
  Description: Called by configMgr_start
 ************************************************************/ 
TI_STATUS smeSm_start(TI_HANDLE		hSmeSm);


/************************************************************
  Function: smeSm_reselect
  Description: 
 ************************************************************/ 
TI_STATUS smeSm_reselect(TI_HANDLE		hSmeSm);

/************************************************************
  Function: smeSm_stop
  Description: 
 ************************************************************/ 
TI_STATUS smeSm_stop(TI_HANDLE		hSmeSm);

/************************************************************
  Function: smeSm_stopAndShutdown
  Description: 
 ************************************************************/ 
void smeSm_stopAndShutdown(TI_HANDLE		hSmeSm);

/************************************************************
  Function: smeSm_getDriverShutdownStatus
  Description: 
 ************************************************************/ 
UINT8 smeSm_getDriverShutdownStatus (TI_HANDLE		hSmeSm);

/************************************************************
  Function: smeSm_scanComplete
  Description: 
 ************************************************************/ 
void smeSm_scanComplete( TI_HANDLE hSmeSm, scan_cncnResultStatus_e status,
                         scan_frameInfo_t *frameInfo, UINT16 SPSStatus );


/************************************************************
  Function: smeSm_reportConnStatus
  Description: 
 ************************************************************/ 
TI_STATUS smeSm_reportConnStatus(TI_HANDLE		hSmeSm, 
							  mgmtStatus_e	status, 
							  UINT32 uStatusCode);

/************************************************************
  Function: smeSm_reportSelectStatus
  Description: 
 ************************************************************/ 
TI_STATUS smeSm_reportSelectStatus(TI_HANDLE		hSmeSm, 
								mgmtStatus_e	status);


/************************************************************
  Function: smeSm_setParam
  Description: 
 ************************************************************/ 
TI_STATUS smeSm_setParam(TI_HANDLE			hSmeSm,
						paramInfo_t		*pParam);


/************************************************************
  Function: smeSm_getParam
  Description: 
 ************************************************************/ 
TI_STATUS smeSm_getParam(TI_HANDLE			hSmeSm, 
						paramInfo_t		*pParam);


#ifdef TI_DBG
/************************************************************
  Function: smeSm_dbgPrintObject
  Description: 
 ************************************************************/ 
void smeSm_dbgPrintObject( TI_HANDLE hSmeSm );

/************************************************************
  Function: smeSm_resetStats
  Description: 
 ************************************************************/ 
void smeSm_resetStats( TI_HANDLE hSmeSm );

/************************************************************
  Function: smeSm_printStats
  Description: 
 ************************************************************/ 
void smeSm_printStats( TI_HANDLE hSmeSm );
#endif /* TI_DBG */

#endif /* __SME_SM_API_H__ */
