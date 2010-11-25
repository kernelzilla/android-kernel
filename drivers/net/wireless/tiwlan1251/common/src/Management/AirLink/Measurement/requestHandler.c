/** \file RequestHandler.c
 *  \brief RequestHandler module interface
 *
 *  \see RequestHandler.h
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

/****************************************************************************************************/
/*																									*/
/*		MODULE:		RequestHandler.c																*/
/*		PURPOSE:	RequestHandler module interface.												*/
/*                  This module handle the incoming measurement requests. The object handle			*/
/*					data base that stores all measurement requests from the last incoming.			*/
/*					This module export interface function for sceduling the next requests to be		*/
/*					executed and stores all relevent fields for constructing a measurement report.	*/
/*																									*/
/****************************************************************************************************/
#include "report.h"
#include "osApi.h"
#include "paramOut.h"
#include "paramIn.h"
#include "utils.h"
#include "requestHandler.h"

#ifdef EXC_MODULE_INCLUDED
#include "excRMMngrParam.h"
#endif

/* allocation vector */
#define REQUEST_HANDLER_INIT_BIT		(1)

#define DOT11_MEASUREMENT_REQUEST_ELE_ID (38)

/********************************************************************************/
/*						Internal functions prototypes.							*/
/********************************************************************************/
static void release_module(requestHandler_t *pRequestHandler, UINT32 initVec);

static TI_STATUS insertMeasurementIEToQueue(TI_HANDLE           hRequestHandler,
											UINT16				frameToken,
											measurement_mode_e	measurementMode,
											UINT8				*pData,
                                            UINT8               *singelRequestLen);

/********************************************************************************/
/*						Interface functions Implementation.						*/
/********************************************************************************/


/********************************************************************************
 *                        requestHandler_create									*
 ********************************************************************************
DESCRIPTION: RequestHandler module creation function, called by the measurement in 
				creation phase. performs the following:

				-	Allocate the RequestHandler handle
				                                                                                                   
INPUT:      hOs -	Handle to OS		

OUTPUT:		

RETURN:     Handle to the RequestHandler module on success, NULL otherwise
************************************************************************/
TI_HANDLE requestHandler_create(TI_HANDLE hOs)
{
	requestHandler_t			*pRequestHandler = NULL;
	UINT32			initVec = 0;
	
	
	/* allocating the RequestHandler object */
	pRequestHandler = os_memoryAlloc(hOs,sizeof(requestHandler_t));

	if (pRequestHandler == NULL)
		return NULL;

	initVec |= (1 << REQUEST_HANDLER_INIT_BIT);

	return(pRequestHandler);
}

/************************************************************************
 *                        requestHandler_config		    				*
 ************************************************************************
DESCRIPTION: RequestHandler module configuration function, called by the measurement
			  in configuration phase. performs the following:
				-	Reset & initiailzes local variables
				-	Init the handles to be used by the module
                                                                                                   
INPUT:      hRequestHandler	-	RequestHandler handle.
			List of handles to be used by the module
			
OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS RequestHandler_config(TI_HANDLE 	hRequestHandler,
						TI_HANDLE		hReport,
						TI_HANDLE		hOs)
{
	requestHandler_t *pRequestHandler = (requestHandler_t *)hRequestHandler;
	

	/* init variables */
    pRequestHandler->parserRequestIEHdr = NULL;
	pRequestHandler->numOfWaitingRequests = 0;	/*	indicating empty data base	*/
	pRequestHandler->activeRequestID = -1;		/*			   					*/
	pRequestHandler->hReport	= hReport;
	pRequestHandler->hOs		= hOs;

	/* Clearing the Request Array , mostly due to parallel bit */
	os_memoryZero(pRequestHandler->hOs, pRequestHandler->reqArr, MAX_NUM_REQ * sizeof(MeasurementRequest_t));

	WLAN_REPORT_INIT(pRequestHandler->hReport, MEASUREMENT_MNGR_MODULE_LOG, ("%s: RequestHandler configured successfully\n", __FUNCTION__));

	return OK;
}

/***********************************************************************
 *                        requestHandler_setParam									
 ***********************************************************************
DESCRIPTION: RequestHandler set param function, called by the following:
			-	config mgr in order to set a parameter receiving from 
				the OS abstraction layer.
			-	From inside the dirver	
                                                                                                   
INPUT:      hRequestHandler	-	RequestHandler handle.
			pParam			-	Pointer to the parameter		

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS requestHandler_setParam(TI_HANDLE	hRequestHandler,
								  paramInfo_t	*pParam)
{
	requestHandler_t *pRequestHandler = (requestHandler_t *)hRequestHandler;
	
	switch(pParam->paramType)
	{
/*	case RequestHandler_PARAM_TYPE:*/
		
	/*	break;*/
		
	default:
		WLAN_REPORT_ERROR(pRequestHandler->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Set param, Params is not supported, %d\n\n", __FUNCTION__, pParam->paramType));
		return PARAM_NOT_SUPPORTED;
	}

/*	return OK; - unreachable */
}

/***********************************************************************
 *                        requestHandler_getParam									
 ***********************************************************************
DESCRIPTION: RequestHandler get param function, called by the following:
			-	config mgr in order to get a parameter from the OS a
				bstraction layer.
			-	From inside the dirver	
                                                                                                   
INPUT:      hRequestHandler	-	RequestHandler handle.
			pParam			-	Pointer to the parameter		

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS requestHandler_getParam(TI_HANDLE		hRequestHandler,
								  paramInfo_t	*pParam)
{
	requestHandler_t *pRequestHandler = (requestHandler_t *)hRequestHandler;
/*	TI_STATUS			status;*/
	
	switch(pParam->paramType)
	{
	/*case RequestHandler_PARAM:*/
		
		
		/*return status;*/
	
	default:
		WLAN_REPORT_ERROR(pRequestHandler->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Get param, Params is not supported, %d\n\n", __FUNCTION__, pParam->paramType));
		return PARAM_NOT_SUPPORTED;
	}

/*	return OK; - unreachable */
}

/************************************************************************
 *                        RequestHandler_destroy						*
 ************************************************************************
DESCRIPTION: RequestHandler module destroy function, called by the config 
			 mgr in the destroy phase 
			 performs the following:
			 -	Free all memory aloocated by the module
                                                                                                   
INPUT:      hRequestHandler	-	RequestHandler handle.		

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS requestHandler_destroy(TI_HANDLE hRequestHandler)
{
	requestHandler_t * pRequestHandler = (requestHandler_t *)hRequestHandler;
	UINT32 initVec;

	if (pRequestHandler == NULL)
		return OK;

	initVec = 0xFFFF;
	release_module(pRequestHandler, initVec);

	return OK;
}

/************************************************************************
 *                  requestHandler_insertRequests						*
 ************************************************************************
DESCRIPTION: RequestHandler module parsing function, called by the 
			  measurement object when measuremnt request frame is received.
				performs the following:
				-	Parsers the measurement request frame.
				-	Inserts all requests into the queue.
				-	Initializes each request according to the its frame 
					token, measurement token, measurement type, parallel,
					channel number, duration time and scan mode.
				-	The function updates the numOfWaitingRequests variable
					and set to zero the activeReqId.

			 Note:  The activeReqId contains the index for the first request
					that should be executed or to the current active request.

INPUT:      hRequestHandler	    -	RequestHandler handle.
			measurementMode     -	The MEasurement Object Mode.
			measurementFrameReq -   The New Frame request that was received.

OUTPUT:		

RETURN:     OK on success, NOK otherwise
************************************************************************/
TI_STATUS requestHandler_insertRequests(TI_HANDLE hRequestHandler,
										measurement_mode_e measurementMode,
										measurement_frameRequest_t measurementFrameReq)
{
	requestHandler_t	*pRequestHandler = (requestHandler_t *)hRequestHandler;
    INT32               requestsLen = measurementFrameReq.requestsLen;
    UINT8               singelRequestLen = 0;
    UINT8               *requests = measurementFrameReq.requests;
		    
	if (requestsLen < 2)
    {
        WLAN_REPORT_ERROR(pRequestHandler->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
				("%s: Invalid length of the data.\n", __FUNCTION__));

        return NOK;
    }

	/* Inserting all measurement request into the queues */
	while (requestsLen > 0)
	{
		if(insertMeasurementIEToQueue(hRequestHandler, 
                                       measurementFrameReq.hdr->dialogToken, 
                                       measurementMode, 
                                       requests,
                                       &singelRequestLen) != OK )
        {
            requestHandler_clearRequests(hRequestHandler);
			return NOK;
        }
  
		requestsLen -= singelRequestLen;
		requests += singelRequestLen;
      
	}

	pRequestHandler->activeRequestID = 0;
	
	WLAN_REPORT_INFORMATION(pRequestHandler->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Inserted into queue: activeRequestID = %d, numOfWaitingRequests = %d\n", __FUNCTION__,
					pRequestHandler->activeRequestID, pRequestHandler->numOfWaitingRequests));

	return OK;
}

/************************************************************************
 *                  requestHandler_getNextReq							*
 ************************************************************************
DESCRIPTION: RequestHandler module function for retrieving the requests that
				should be executed.
				performs the following:
				-	returns pointers to one request/several requests that
					should be performed in parallel.
				Note: The function updates the numOfWaitingRequests internal
				varaible ONLY IF the returned request/s are going to be 
				executed immediatly (isForActivation = TRUE).

INPUT:      hRequestHandler	-	RequestHandler handle.
			
  			isForActivation -	A flag that indicates if the returned 
								request/s are going to be executed immediatly

OUTPUT:		pRequest		-	pointer contains the address in which the 
								next requests for activation should be inserted.
		
			numOfRequests	-	indicates how many requests should be activated
								in parallel.
			
RETURN:     OK on success, NOK otherwise
************************************************************************/
TI_STATUS requestHandler_getNextReq(TI_HANDLE hRequestHandler,
									BOOL	  isForActivation,
									MeasurementRequest_t *pRequest[],
									UINT8*	  numOfRequests)
{
	requestHandler_t	*pRequestHandler = (requestHandler_t *)hRequestHandler;
	UINT8				requestIndex = pRequestHandler->activeRequestID;
	UINT8				loopIndex = 0;
	
	WLAN_REPORT_INFORMATION(pRequestHandler->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Looking for requests. activeRequestID = %d, numOfWaitingRequests = %d\n", __FUNCTION__,
					pRequestHandler->activeRequestID, pRequestHandler->numOfWaitingRequests));

	if(pRequestHandler->numOfWaitingRequests <= 0)
		return NOK;

	do{
		pRequest[loopIndex] = &(pRequestHandler->reqArr[requestIndex]);
		requestIndex++;
		loopIndex++;
	}
	while ( (loopIndex < pRequestHandler->numOfWaitingRequests) && 
            (pRequestHandler->reqArr[requestIndex].isParallel) );

	*numOfRequests = loopIndex;

	WLAN_REPORT_INFORMATION(pRequestHandler->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Found %d requests to execute in parallel.\n", __FUNCTION__, loopIndex));

	if(isForActivation == TRUE)
	{
		pRequestHandler->numOfWaitingRequests -= loopIndex;

		WLAN_REPORT_INFORMATION(pRequestHandler->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
				("%s: Requests were queried for activation so decreasing numOfWaitingRequests to %d\n", __FUNCTION__, pRequestHandler->numOfWaitingRequests));
	}

	return OK;
}

/************************************************************************
 *                  requestHandler_getCurrentExpiredReq					*
 ************************************************************************
DESCRIPTION: RequestHandler module function for retrieving the request that
				finished its execution.
				performs the following:
				-	returns pointers to the request that
					finished its execution in.

INPUT:      hRequestHandler	-	RequestHandler handle.
			requestIndex	-	Index of request in the queue
			
OUTPUT:		pRequest		-	pointer contains the addresse of the 
								request that finished its execution.

RETURN:     OK on success, NOK otherwise
************************************************************************/
TI_STATUS requestHandler_getCurrentExpiredReq(TI_HANDLE hRequestHandler,
											  UINT8 requestIndex,
											  MeasurementRequest_t **pRequest)
{
	requestHandler_t	*pRequestHandler = (requestHandler_t *)hRequestHandler;
	
	requestIndex += pRequestHandler->activeRequestID;

	*pRequest = &(pRequestHandler->reqArr[requestIndex]);

	return OK;
}


/************************************************************************
 *                  requestHandler_clearRequests						*
 ************************************************************************
DESCRIPTION: RequestHandler module function for cleaning the data base.
				performs the following:
				-	Clears all requests from the queue by setting
					the activeReqId and numOfWaitingRequests variables.
			Note:	The function does not actually zero all queue 
					variables or destroy the object.

INPUT:      hRequestHandler	-	RequestHandler handle.
			
OUTPUT:		None

RETURN:     OK on success, NOK otherwise
************************************************************************/
TI_STATUS requestHandler_clearRequests(TI_HANDLE hRequestHandler)
{
	requestHandler_t	*pRequestHandler = (requestHandler_t *)hRequestHandler;

	pRequestHandler->numOfWaitingRequests = 0;
	pRequestHandler->activeRequestID = -1;
	
	/* Clearing the Request Array , mostly due to parallel bit */
	os_memoryZero(pRequestHandler->hOs,pRequestHandler->reqArr,
				  MAX_NUM_REQ * sizeof(MeasurementRequest_t));	
	
	WLAN_REPORT_INFORMATION(pRequestHandler->hReport, MEASUREMENT_MNGR_MODULE_LOG, 
			("%s: Request queue has been cleared. activeRequestID = %d, numOfWaitingRequests = %d\n", __FUNCTION__,
					pRequestHandler->activeRequestID, pRequestHandler->numOfWaitingRequests));

	return OK;
}


	
/************************************************************************
 *                  requestHandler_getFrameToken						*
 ************************************************************************
DESCRIPTION: RequestHandler module function for getting the token of the 
				frame that is now being processed.

INPUT:      hRequestHandler	-	RequestHandler handle.
			
			
OUTPUT:		frameToken

RETURN:     OK on success, NOK otherwise
************************************************************************/
TI_STATUS requestHandler_getFrameToken(TI_HANDLE hRequestHandler,UINT16 *frameToken )
{
	requestHandler_t	*pRequestHandler = (requestHandler_t *)hRequestHandler;

	if(pRequestHandler->activeRequestID == -1)
		return NOK;

	*frameToken = pRequestHandler->reqArr[0].frameToken;
	
	return OK;
}

/************************************************************************
 *              requestHandler_setRequestParserFunction					*
 ************************************************************************
DESCRIPTION: RequestHandler module function for setting the function that
             parasers a request IE.

INPUT:      hRequestHandler	-	RequestHandler handle.
            parserRequestIE -   A pointer to the function.
			
			
OUTPUT:		

RETURN:     OK on success, NOK otherwise
************************************************************************/	
TI_STATUS requestHandler_setRequestParserFunction(TI_HANDLE hRequestHandler, 
                                                  parserRequestIEHdr_t parserRequestIEHdr)
{
	requestHandler_t	*pRequestHandler = (requestHandler_t *)hRequestHandler;

    pRequestHandler->parserRequestIEHdr = parserRequestIEHdr;

    return OK;
}

/********************************************************************************/
/*						Internal functions Implementation.						*/
/********************************************************************************/

/************************************************************************
 *                  insertMeasurementIEToQueue							*
 ************************************************************************
DESCRIPTION: The function inserts measurement request of one received 
				measurement request information element.

INPUT:      hRequestHandler	-	A Handler to the Request Handler Object.
			frameToken		-	Frame token of the received frame in which
								This current measurement request IE is included.
            measurementObjMode - EXC or SPECTRUM_MNGMNT
			dataLen			-	pointer to the data length that is left.
			pData			-	pointer to the data.
			
OUTPUT:		singelRequestLen - The Length of the request that was inserted 
                               to the queue.

RETURN:     OK on success, NOK otherwise
************************************************************************/
static TI_STATUS insertMeasurementIEToQueue(TI_HANDLE           hRequestHandler,
											UINT16				frameToken,
											measurement_mode_e	measurementObjMode,
											UINT8				*pData,
                                            UINT8               *singelRequestLen)
{
   	requestHandler_t	*pRequestHandler = (requestHandler_t *)hRequestHandler;

	UINT16		HeaderLen;
	UINT8		measurementMode;
	UINT8		parallelBit;
	UINT8		enableBit;
	UINT16		durationTime;
    UINT16      measurementToken;
	
	MeasurementRequest_t	*pCurrRequest = &(pRequestHandler->reqArr[pRequestHandler->numOfWaitingRequests]);

    if (pRequestHandler->parserRequestIEHdr(pData, &HeaderLen, &measurementToken) != OK)
    {
        return NOK;
    }

	pCurrRequest->frameToken = frameToken;	
	pCurrRequest->measurementToken = measurementToken;

    pData += HeaderLen;

    /*** Getting the Measurement Mode ***/
	measurementMode		= *pData++;

	/* getting parallel bit */
	parallelBit = measurementMode & 0x1;
	
    /* getting Enable bit */
	enableBit = (measurementMode & 0x2)>>1;
	
    /* checking enable bit, the current implementation does not support 
		enable bit which set to one, so there is no need to check request/report bits	*/
	if(enableBit == 1)
		return OK;
    
    pCurrRequest->isParallel = parallelBit;


    /* Getting the Measurement Mode */
   	pCurrRequest->Type = (measurement_type_e)(*pData++);

	/* Inserting the request that is included in the current measurement request IE. */
	pCurrRequest->channelNumber = *pData++;
    
	pCurrRequest->ScanMode = (measurement_scanMode_e)(*pData++); /* IN dot11h - Spare = 0 */

    os_memoryCopy(pRequestHandler->hOs, &durationTime, pData, 2);
    durationTime = ENDIAN_HANDLE_WORD(durationTime);
	pCurrRequest->DurationTime = durationTime;
	
	*singelRequestLen = HeaderLen + 6;

	pRequestHandler->numOfWaitingRequests ++;
	
	return OK;
}


/***********************************************************************
 *                        release_module									
 ***********************************************************************
DESCRIPTION:	Called by the destroy function or by the create function 
				(on failure). Go over the vector, for each bit that is 
				set, release the corresponding module.
                                                                                                   
INPUT:      pRequestHandler	-	RequestHandler pointer.
			initVec			-	Vector that contains a bit set for each 
								module thah had been initiualized

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
static void release_module(requestHandler_t *pRequestHandler, UINT32 initVec)
{
	
	if ( initVec & (1 << REQUEST_HANDLER_INIT_BIT) )
		utils_nullMemoryFree(pRequestHandler->hOs, pRequestHandler, 
		sizeof(requestHandler_t));
		
	initVec = 0;
}






