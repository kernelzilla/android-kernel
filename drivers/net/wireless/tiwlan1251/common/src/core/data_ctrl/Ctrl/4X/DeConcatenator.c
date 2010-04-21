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
/*		MODULE:													   */
/*		PURPOSE:		 						   */
/*																		   */
/***************************************************************************/
#include "DeConcatenator.h"
#include "report.h"
#include "osApi.h"
#include "utils.h"
#include "802_11Defs.h"

#define MAX_DECONACT_LEN  1600 /* Maximum length of real ip packet inside concatination packet */   

/*************************************************************************
*                        concat_create                                 *
**************************************************************************
* DESCRIPTION:	This function initializes the Ctrl data module.                 
*                                                      
* INPUT:		hOs - handle to Os Abstraction Layer
*				
* OUTPUT:		TxCmplt_CB - call back function that return to configMngr
*				in order to register in the Hal
*
* RETURN:		Handle to the allocated Ctrl data control block
************************************************************************/

deConcatenator_t* deConcat_create(TI_HANDLE hOs)
{
	deConcatenator_t* pDeConcatenator;

	if( hOs  == NULL )
	{
	    WLAN_OS_REPORT(("FATAL ERROR: deConcat_create(): OS handle Error - Aborting\n"));
		return NULL;
	}

	/* alocate concatenator block */
	pDeConcatenator = os_memoryAlloc(hOs, (sizeof(deConcatenator_t)));


	if (!pDeConcatenator)
	{
		utils_nullMemoryFree(hOs, pDeConcatenator, sizeof(deConcatenator_t));
	    WLAN_OS_REPORT(("FATAL ERROR: deConcat_create(): Error Creating DeConcatenator module- Aborting\n"));
		return(NULL);
	}

	/* reset control module control block */
	os_memoryZero(hOs, pDeConcatenator, (sizeof(deConcatenator_t)));

	pDeConcatenator->hOs = hOs;

	return(pDeConcatenator);
}

/***************************************************************************
*							ctrlData_config				                   *
****************************************************************************
* DESCRIPTION:	This function configures the Ctrl Data module		
* 
* INPUTS:		hCtrlData - The object
*				hOs - Handle to the Os Abstraction Layer
*				hReport - Handle to the Report object
* 				ctrlDataInitParams - pointer to Ctrl module init parameters
* OUTPUT:		
* 
* RETURNS:		OK - Configuration succesfull
*				NOK - Configuration unsuccesfull
***************************************************************************/
TI_STATUS deConcat_config(deConcatenator_t*	pDeConcatenator,
						  TI_HANDLE			hOs,
						  TI_HANDLE			hReport,
						  TI_HANDLE			hMemMngr)
{
	/* check parameters validity */
	if( pDeConcatenator == NULL || hOs == NULL || 
		hReport == NULL || hMemMngr == NULL)
	{
	    WLAN_OS_REPORT(("FATAL ERROR: deConcat_config(): Parameters Error - Aborting\n"));
		return NOK;
	}

	/* set objects handles */
	pDeConcatenator->hOs = hOs;
	pDeConcatenator->hReport = hReport;
	pDeConcatenator->hMemMngr = hMemMngr;

	WLAN_REPORT_INIT(pDeConcatenator->hReport, DE_CONCATENATOR_MODULE_LOG, 
		(".....DeConcatenator configured successfully\n"));

	return OK;
}

/***************************************************************************
*							ctrlData_unLoad				                   *
****************************************************************************
* DESCRIPTION:	This function unload the Ctrl data module. 
* 
* INPUTS:		hCtrlData - the object
*		
* OUTPUT:		
* 
* RETURNS:		OK - Unload succesfull
*				NOK - Unload unsuccesfull
***************************************************************************/

TI_STATUS deConcat_destroy(deConcatenator_t*	pDeConcatenator)
{
	/* free control module controll block */
	os_memoryFree(pDeConcatenator->hOs, pDeConcatenator, sizeof(deConcatenator_t));

	return OK;
}


/*************************************************************************
 *                        wdrv_rxDeConcatMsdu                            *
 *************************************************************************
DESCRIPTION: This function de concatenates number of MSDUs froma single 
             MPDU.                
                                                                                                   
INPUT:       msduPtr     - Pointer to the first MSDU.
             
             
OUTPUT:     

RETURN:      OK : Deconcatenation OK.
             NOK: Deconcatenation Faild.                                                                                                                                             
************************************************************************/
TI_STATUS deConcat_deConcatMsdu(deConcatenator_t*	pDeConcatenator,
								mem_MSDU_T**		MsduPtr)
{
	
    Wdrv4xConcatHeader_t	*concatHdrPtr;
    dot114xMsdu_t			*pdot11fourXHeader;
	mem_MSDU_T				*firstMsduPtr = NULL;
    UINT8					*currPtr;
    UINT8					*newDot11HeaderPtr;
	UINT16					currentLength;
    UINT32					concatLength;
    UINT32					rc = OK;
    UINT32					parsedLen;
	UINT32					msduNum = 0;
	BOOL					firsdMsduInList = TRUE;
	mem_MSDU_T				*NextMsduPtr;
    mem_MSDU_T				*PrevMsduPtr = NULL;

	/*print_MsduDataHeader(pDeConcatenator->hMemMngr, *MsduPtr);*/

	
   /*
	* The 802.11 header is located in the beggining of the concat frame.
	*/
    pdot11fourXHeader = (dot114xMsdu_t*)memMgr_BufData((*MsduPtr)->firstBDPtr);
	
    concatLength =  (*MsduPtr)->dataLen;
	
   /*
	* The parsed size is currently the wlan snap and 4x headers.
	*/ 
    parsedLen =  sizeof( dot114xMsdu_t );
    currPtr =  (UINT8*) pdot11fourXHeader + sizeof(dot114xMsdu_t);
	
    /*
	* Start parsing the concatenated MPDU and create an MSDUs.
	*/

	if(parsedLen + sizeof(Wdrv4xConcatHeader_t) >= concatLength)
	{
		WLAN_REPORT_ERROR(pDeConcatenator->hReport, DE_CONCATENATOR_MODULE_LOG, 
				("deConcat_deConcatMsdu: Error Parsing Deconcat Packet: concatLength = %d ",concatLength));
		firstMsduPtr = NULL;
		rc = NOK;
	}

    while ( parsedLen + sizeof(Wdrv4xConcatHeader_t) < concatLength ) 
	{
		
		UINT8		numOfPadBytes;
		mem_MSDU_T* CurrentMsduPtr;
		
		msduNum++;
		
		concatHdrPtr = (Wdrv4xConcatHeader_t*)currPtr;
		
		currentLength = wlan_htons(concatHdrPtr->len);

		if( currentLength > MAX_DECONACT_LEN)
		{
			WLAN_REPORT_ERROR(pDeConcatenator->hReport, DE_CONCATENATOR_MODULE_LOG, 
				("deConcat_deConcatMsdu: Error Parsing Deconcat Packet Len  %d ",currentLength));
            rc = NOK;
			break;
		}
		
		currPtr += sizeof(Wdrv4xConcatHeader_t);
		
       /*
		*  Zero Padding checking.
		*/
		numOfPadBytes = (currentLength+ 2) % 4;
		
		if( numOfPadBytes ) 
			numOfPadBytes = 4 - numOfPadBytes;
		
	   /*
		* Create MSDU with one buffer for the wlan header.
		* and copy the 802.11 header from the MSDU to it. 
		*/
#if 1
		/*******************************************************************/
		/*     Deconcatenation with COPY Solution                          */
		/*******************************************************************/         
		
		if( wlan_memMngrAllocMSDU(pDeConcatenator->hMemMngr, 
								  &CurrentMsduPtr, 
								  WLAN_HDR_LEN + currentLength - sizeof(macAddress_t),
								  DE_CONCAT_MODULE ) != OK)
		{
			WLAN_REPORT_ERROR(pDeConcatenator->hReport, DE_CONCATENATOR_MODULE_LOG,
				("fail to allocate msdu \n"));
			rc = NOK;
			break; /* fail to allocate - abort deconcatenation */
		}
		
		if(firsdMsduInList == TRUE)
		{
			firstMsduPtr = CurrentMsduPtr;
			firstMsduPtr->prevMSDUinList = NULL;
			firsdMsduInList = FALSE;
		}
		else
		{
			CurrentMsduPtr->prevMSDUinList = PrevMsduPtr;
			PrevMsduPtr->nextMSDUinList = CurrentMsduPtr;
		}

		
		CurrentMsduPtr->headerLen = WLAN_HDR_LEN;
		newDot11HeaderPtr =  (UINT8 *)memMgr_BufData(CurrentMsduPtr->firstBDPtr);
		
		os_memoryCopy(NULL, newDot11HeaderPtr, (char*)pdot11fourXHeader, WLAN_HDR_LEN );
		
	   /*
		*  Copy the SA from the concatenated MSDU to the header.
		*/ 
		os_memoryCopy(NULL, (void*)&(((dot11_header_t*)newDot11HeaderPtr)->address3), 
			(void*)&(concatHdrPtr->SaDa),sizeof(macAddress_t) );
		
	   /*
		* Reading the concatenation More bit, if it is exists, set it in each MSDU's
		* order bit. (Used by the Ack emulation)
		*/
		if( pdot11fourXHeader->header4x.txFlags == wlan_htons(WLAN_4X_CONCAT_MORE_BIT) )
			((dot11_header_t*)newDot11HeaderPtr)->fc |= DOT11_FC_ORDER; 
		else
            ((dot11_header_t*)newDot11HeaderPtr)->fc &= ~DOT11_FC_ORDER;       

	   /*
        *  Copy the SNAP + Payload length to the new data buffer.
        *  ( 2 copies solution ). 
		*/
		
		os_memoryCopy(NULL, (void*)((UINT32)(newDot11HeaderPtr)+WLAN_HDR_LEN ), 
			currPtr, currentLength - sizeof(macAddress_t)  );
		
		
		currPtr+= currentLength - sizeof(macAddress_t) + numOfPadBytes;
		
        CurrentMsduPtr->firstBDPtr->length = currentLength - sizeof(macAddress_t) + WLAN_HDR_LEN ;
		CurrentMsduPtr->dataLen =  CurrentMsduPtr->firstBDPtr->length;
		CurrentMsduPtr->firstBDPtr->dataOffset = 0;
		CurrentMsduPtr->firstBDPtr->nextBDPtr = NULL;
		
		
#else
		/*******************************************************************/
		/*     Deconcatenation with NO COPY Solution                       */
		/*******************************************************************/ 
		if( wlan_memMngrAllocMSDU(pDeConcatenator->hMemMngr, &buildMsduPtr,
			sizeof(dot11_header_t),DE_CONCAT_MODULE )==NOK)
		{
			WLAN_REPORT_ERROR(pDeConcatenator->hReport, DE_CONCATENATOR_MODULE_LOG,
				("fail to allocate msdu \n"));
			return NOK;
		}
		
	   /*
		*  Copy the 802.11 header to the first buffer. (BssBridge needs the 802.11
		*  and the SNAP in the same buffer for Ethernet header translation. 
		*/
		newDot11HeaderPtr =  memMgr_BufData(buildMsduPtr->firstBDPtr);
		os_memoryCopy(NULL, newDot11HeaderPtr, (char*)pdot11Header, sizeof(dot11_header_t));
		
	   /*
		*  Copy the SA from the concatenated MSDU to the header.
		*/ 
		os_memoryCopy(NULL, (void*)&(((dot11_header_t*)newDot11HeaderPtr)->address3), 
			(void*)&(concatHdrPtr->SaDa),sizeof(macAddress_t) );
		
			/*
			* Reading the concatenation More bit, if it is exists, set it in each MSDU's
			* order bit. (Used by the Ack emulation)
		*/
		if( pdot11Header->header4x.txFlags == WLAN_4X_CONCAT_MORE_BIT )
			((dot11_header_t*)newDot11HeaderPtr)->fc |= DOT11_FC_ORDER;        
		else
            ((dot11_header_t*)newDot11HeaderPtr)->fc &= ~DOT11_FC_ORDER;  
		
	   /*
		*  Copy the SNAP header to the first buffer. (BssBridge needs the 802.11
		*  and the SNAP in the same buffer for Ethernet header translation. 
		*/
		os_memoryCopy(NULL, (void*)((UINT32)(newDot11HeaderPtr)+sizeof(dot11_header_t)), 
			currPtr, sizeof(Wlan_LlcHeader_T) );
		
		currPtr+= sizeof(Wlan_LlcHeader_T);
		
		buildMsduPtr->firstBDPtr->length = sizeof( dot11_header_t ) + sizeof(Wlan_LlcHeader_T);
		
		/*
		* Create a new BD, link it concatenated to the data buffer's SNAP header.
		*/
		if( wlan_memMngrAllocBDs(pDeConcatenator->hMemMngr, 1, &payloadBdPtr) == NOK)
		{
			WLAN_REPORT_ERROR(pDeConcatenator->hReport, DE_CONCATENATOR_MODULE_LOG,
				("fail to allocate BD \n"));
			return NOK;
		}
		payloadBdPtr->dataBuf = MsduPtr->firstBDPtr->dataBuf;
		payloadBdPtr->data = currPtr;
		payloadBdPtr->length = currentLength - sizeof(macAddress_t) - sizeof(Wlan_LlcHeader_T);
		payloadBdPtr->dataOffset = 0;
		payloadBdPtr->nextBDPtr = NULL;
		
		/*
		*  Link the MSDU first BD (wlan header) to the second one (snap+payload)
		*  creating a complete MSDU.
		*/
		buildMsduPtr->firstBDPtr->nextBDPtr =  payloadBdPtr;
		payloadBdPtr->dataBuf->refCount++;
		buildMsduPtr->dataLen =  buildMsduPtr->firstBDPtr->length + payloadBdPtr->length;
		
		currPtr += currentLength - sizeof(macAddress_t) - sizeof(Wlan_LlcHeader_T) + numOfPadBytes;;          
#endif
		
		PrevMsduPtr = CurrentMsduPtr;
		parsedLen += currentLength + 2 + numOfPadBytes;
		
    } /* while ( parsedLen < concatLength ) */
	
	  /*
	  *  Free the c MSDU.
	  */
	if(rc == OK)
	{
		wlan_memMngrFreeMSDU(pDeConcatenator->hMemMngr, (*MsduPtr)->handle);
		/* update the return msdu */
		*MsduPtr = firstMsduPtr;
	}
	else /* free all allocated MSDUs in case of failure */
	{
		while(firstMsduPtr)
		{
			/* Save the next msdu in the list before free the current msdu */
			NextMsduPtr = firstMsduPtr->nextMSDUinList;
			wlan_memMngrFreeMSDU(pDeConcatenator->hMemMngr, firstMsduPtr->handle);
			firstMsduPtr = NextMsduPtr;
		}
	}
  
	return (TI_STATUS)rc;
}

