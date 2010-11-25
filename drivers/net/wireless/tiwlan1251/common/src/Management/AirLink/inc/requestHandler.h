/** \file requestHandler.h
 *  \brief Request Handler module interface header file
 *
 *  \see requestHandler.c
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
/*	  MODULE:	requestHandler.h											*/
/*    PURPOSE:	Request Handler module interface header file				*/
/*																			*/
/***************************************************************************/
#ifndef __REQUEST_HANDLER_H__
#define __REQUEST_HANDLER_H__

#include "paramOut.h"
#include "802_11Defs.h"
#include "measurementMgrApi.h"

PACKED_STRUCT( MeasurementRequest_t	,

	measurement_type_e		Type;
	BOOL					isParallel;	
	UINT16					frameToken;	
	UINT16					measurementToken;
	UINT8					channelNumber;
	UINT16					DurationTime;
	UINT8					ActualDurationTime;
	measurement_scanMode_e	ScanMode;
);


/* Functions Pointers Definitions */
typedef TI_STATUS (*parserRequestIEHdr_t)   (UINT8 *pData, UINT16 *reqestLen,
                                             UINT16 *measurementToken);

typedef struct 
{
    /* Function to the Pointer */
    parserRequestIEHdr_t   parserRequestIEHdr;

    /* General Params */
	MeasurementRequest_t    reqArr[MAX_NUM_REQ];
	UINT8				numOfWaitingRequests;	
	INT8				activeRequestID;

    /* Handlers */
	TI_HANDLE			hReport;
	TI_HANDLE			hOs;
} requestHandler_t;



TI_HANDLE requestHandler_create(TI_HANDLE hOs);

TI_STATUS RequestHandler_config(TI_HANDLE 	hRequestHandler,
						        TI_HANDLE		hReport,
						        TI_HANDLE		hOs);

TI_STATUS requestHandler_setParam(TI_HANDLE	hRequestHandler,
								  paramInfo_t	*pParam);

TI_STATUS requestHandler_getParam(TI_HANDLE		hRequestHandler,
											paramInfo_t	*pParam);

TI_STATUS requestHandler_destroy(TI_HANDLE hRequestHandler);

TI_STATUS requestHandler_insertRequests(TI_HANDLE hRequestHandler,
										measurement_mode_e measurementMode,
										measurement_frameRequest_t measurementFrameReq);

TI_STATUS requestHandler_getNextReq(TI_HANDLE hRequestHandler,
									BOOL	  isForActivation,
									MeasurementRequest_t *pRequest[],
									UINT8*	  numOfRequests);

TI_STATUS requestHandler_getCurrentExpiredReq(TI_HANDLE hRequestHandler,
											  UINT8 requestIndex,
											  MeasurementRequest_t **pRequest);

TI_STATUS requestHandler_clearRequests(TI_HANDLE hRequestHandler);

TI_STATUS requestHandler_getFrameToken(TI_HANDLE hRequestHandler,UINT16 *frameToken );

TI_STATUS requestHandler_setRequestParserFunction(TI_HANDLE hRequestHandler, 
                                                  parserRequestIEHdr_t parserRequestIEHdr);


#endif /* __REQUEST_HANDLER_H__*/
