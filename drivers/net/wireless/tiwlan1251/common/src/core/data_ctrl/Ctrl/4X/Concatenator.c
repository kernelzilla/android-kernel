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
#include "Concatenator.h"
#include "report.h"
#include "osApi.h"
#include "utils.h"
#include "802_11Defs.h"
#include "whalBus_Defs.h"
#include "TNETW_Driver_api.h"


static TI_STATUS concat_replaceWlanHeader(concatenator_t* pConcatenator,
										  mem_MSDU_T *msduPtr);


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

concatenator_t* concat_create(TI_HANDLE hOs)
{
	concatenator_t* pConcatenator;

	if( hOs  == NULL )
	{
	    WLAN_OS_REPORT(("FATAL ERROR: concat_create(): OS handle Error - Aborting\n"));
		return NULL;
	}

	/* alocate concatenator block */
	pConcatenator = os_memoryAlloc(hOs, (sizeof(concatenator_t)));


	if (!pConcatenator)
	{
		utils_nullMemoryFree(hOs, pConcatenator, sizeof(concatenator_t));
	    WLAN_OS_REPORT(("FATAL ERROR: concat_create(): Error Creating Concatenator module- Aborting\n"));
		return(NULL);
	}

	/* reset control module control block */
	os_memoryZero(hOs, pConcatenator, (sizeof(concatenator_t)));

	pConcatenator->hOs = hOs;

	return(pConcatenator);
}

/***************************************************************************
*							concat_config				                   *
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
TI_STATUS concat_config(concatenator_t*		pConcatenator,
						TI_HANDLE			hOs,
						TI_HANDLE			hReport,
						TI_HANDLE			hMemMngr
						/*concatInitParams_t* concatInitParams*/)
{
	/* check parameters validity */
	if( pConcatenator == NULL || hOs == NULL || 
		hReport == NULL || hMemMngr == NULL /*||  concatInitParams == NULL*/)
	{
	    WLAN_OS_REPORT(("FATAL ERROR: concat_config(): Parameters Error - Aborting\n"));
		return NOK;
	}

	/* set objects handles */
	pConcatenator->hOs = hOs;
	pConcatenator->hReport = hReport;
	pConcatenator->hMemMngr = hMemMngr;

	WLAN_REPORT_INIT(pConcatenator->hReport, CONCATENATOR_MODULE_LOG, 
		(".....Concatenator configured successfully\n"));

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

TI_STATUS concat_destroy(concatenator_t* pConcatenator)
{
	/* free control module controll block */
	os_memoryFree(pConcatenator->hOs, pConcatenator, sizeof(concatenator_t));

	return OK;
}


/*************************************************************************
 *                        wdrv_txConcatMsduList                          *
 *************************************************************************
DESCRIPTION: This function get a list of MSDUs (The first MSDU points 
             on the others) and concatenate them into one MPDU by linking the
             buffers BDs. In return only the first MSDU struct is in use, 
             and it contains the entire concatenated MPDU.                
                                                                                                   
INPUT:       msduPtr     - Pointer to the first MSDU.
             concatFlags - Concatenation flags.
             
             
OUTPUT:      By reference to the msduPtr.

RETURN:      OK : Initiation successful.
             NOK: Initiation unsuccessful.                                                                                                                                             
************************************************************************/
TI_STATUS concat_concatMsduList(concatenator_t*	pConcatenator,
								mem_MSDU_T*		pFirstMsduPtr,
								mem_MSDU_T**	pReturnMsduPtr,
								UINT16			concatFlags)
{
	mem_MSDU_T     *currMsduPtr;
	mem_MSDU_T     *prevMsduPtr;
	/*mem_MSDU_T     *buildMsduPtr; */
	UINT8          *srcDataPtr;
	UINT8          *buildDataPtr;
    Wdrv4xHeader_t *w4xHeaderPtr;
	UINT8			tiSnapDataArray[8] = {0xAA,0xAA,0x03,0x08,0x00,0x28,0x60,0xD0};

	/* Allocate MSDU and a BD for the concatenatetion header. */
	if(wlan_memMngrAllocMSDU(pConcatenator->hMemMngr, pReturnMsduPtr, 
							  WLAN_HDR_LEN + WLAN_SNAP_HDR_LEN +WLAN_4X_CONCAT_HDR_LEN + sizeof(DbTescriptor),
							  CONCAT_MODULE) == NOK)
	{
		WLAN_REPORT_ERROR(pConcatenator->hReport, CONCATENATOR_MODULE_LOG, 
			("concat_concatMsduList: no MemMngre resources, free the MsduList to concat\n"));
			
		wlan_memMngrFreeListOfMSDU(pConcatenator->hMemMngr, memMgr_MsduHandle(pFirstMsduPtr));
		return NOK;
	}

	(*pReturnMsduPtr)->txFlags = pFirstMsduPtr->txFlags;
	
	/* Create the first BD.(contains 802.11 header, TI wlan SNAP & Concat headr) */
	srcDataPtr = (UINT8 *)memMgr_MsduHdrAddr(pFirstMsduPtr);
	buildDataPtr = (UINT8 *)memMgr_MsduHdrAddr(*pReturnMsduPtr);

	/* Copy the 802.11 header. */
	os_memoryCopy(pConcatenator->hOs, buildDataPtr, srcDataPtr, WLAN_HDR_LEN );

	/* We send the frame from the STA so the AP */
	os_memoryCopy(pConcatenator->hOs, buildDataPtr+WLAN_DA_FIELD_OFFSET, srcDataPtr+WLAN_BSSID_FIELD_OFFSET, 6 );
   
	buildDataPtr += WLAN_HDR_LEN;
	
	/* create a TI WLAN SNAP */
	os_memoryCopy(pConcatenator->hOs, buildDataPtr ,tiSnapDataArray, WLAN_SNAP_HDR_LEN );
	buildDataPtr += WLAN_SNAP_HDR_LEN;
	
	/* create 4X header.   */
    w4xHeaderPtr = (Wdrv4xHeader_t*)buildDataPtr;
    w4xHeaderPtr->type = WLAN_HEADER_TYPE_CONCATENATION;
    w4xHeaderPtr->headerLen = WLAN_CONCAT_HEADER_LEN;
    w4xHeaderPtr->txFlags = wlan_htons(concatFlags);
	
	(*pReturnMsduPtr)->firstBDPtr->length = WLAN_HDR_LEN + WLAN_SNAP_HDR_LEN + 
										WLAN_4X_CONCAT_HDR_LEN;

    (*pReturnMsduPtr)->dataLen =  WLAN_HDR_LEN + WLAN_SNAP_HDR_LEN + 
										WLAN_4X_CONCAT_HDR_LEN;


    (*pReturnMsduPtr)->headerLen = WLAN_HDR_LEN;


    /*buildMsduPtr->nextMSDUinList = (*pMsduPtr);*/

	
	/* Link the new MSDU to the first MSDU to imitate the MSDU list format.     */
	(*pReturnMsduPtr)->firstBDPtr->nextBDPtr = pFirstMsduPtr->firstBDPtr;
	
	/* Start Concatenating the MSDUs, payload is copied from the SNAP to       */
	/* payload  end.                                                           */        
	currMsduPtr = pFirstMsduPtr;
	while( currMsduPtr != NULL )
	{
        concat_replaceWlanHeader(pConcatenator, currMsduPtr );

        /* Update the size of the concatenated MSDU. */		
        (*pReturnMsduPtr)->dataLen += currMsduPtr->dataLen;

		
        if(  currMsduPtr->nextMSDUinList != NULL )
        {
            /* Link last BD of the current MSDU to the first BD of the next MSDU.*/
             currMsduPtr->lastBDPtr->nextBDPtr = currMsduPtr->nextMSDUinList->firstBDPtr;
		
            /* Jump for the next MSDU */
            prevMsduPtr = currMsduPtr;
			currMsduPtr = currMsduPtr->nextMSDUinList;
			prevMsduPtr->firstBDPtr = NULL;
			prevMsduPtr->nextMSDUinList = NULL;
			wlan_memMngrFreeMSDU(pConcatenator->hMemMngr, memMgr_MsduHandle(prevMsduPtr));

		
        }
        else 
		{
            /* Last MSDU */
            prevMsduPtr = currMsduPtr;
			prevMsduPtr->firstBDPtr = NULL;
			prevMsduPtr->nextMSDUinList = NULL;
			wlan_memMngrFreeMSDU(pConcatenator->hMemMngr, memMgr_MsduHandle(prevMsduPtr));

            currMsduPtr = NULL;

        }

		
	} /* While( currMsduPtr != NULL ) */

	
	return OK;
}


/*************************************************************************
 *                        wdrv_txReplaceWlanHeader                       *
 *************************************************************************
DESCRIPTION: This function replaces the 802.11 header with length + SA as
             prefix to the concatenated MSDU.                
                                                                                                   
INPUT:       msduPtr     - Pointer to the first MSDU.
             
             

RETURN:      OK : Initiation succesfull.
             NOK: Initiation unsuccesfull.                                                                                                                                             
************************************************************************/
static TI_STATUS concat_replaceWlanHeader(concatenator_t* pConcatenator,
										  mem_MSDU_T *msduPtr)
{
    UINT8   *firstDataBuf;
    UINT8    numOfPadBytes;
    UINT8   *tmpPtr;
	UINT8	tmpMacAddr[WLAN_DA_FIELD_LEN];
    int i;
	
	
	/* Replace the 802.11 header with 2 length bytes and 6 DA . */
	firstDataBuf = (UINT8 *)(msduPtr->firstBDPtr->data + 
							msduPtr->firstBDPtr->dataOffset);

	/*
	 * Use temporary buffer to prevent overwrite on the same data
	 */
	os_memoryCopy(pConcatenator->hOs,
				  tmpMacAddr,
				  firstDataBuf+WLAN_DA_FIELD_OFFSET, WLAN_DA_FIELD_LEN);
	os_memoryCopy(pConcatenator->hOs,
				  firstDataBuf+ WLAN_HDR_LEN - WLAN_DA_FIELD_LEN,
				  tmpMacAddr, WLAN_DA_FIELD_LEN);
	
	msduPtr->firstBDPtr->dataOffset += WLAN_CONCAT_HDR_OFFSET;
	msduPtr->firstBDPtr->length -= WLAN_CONCAT_HDR_OFFSET;
	msduPtr->dataLen -= WLAN_CONCAT_HDR_OFFSET;
	
	/* Fill the length bytes.   */
	(*(UINT16*)(firstDataBuf+ WLAN_CONCAT_HDR_OFFSET)) = 
			wlan_htons((UINT16)(msduPtr->dataLen - WLAN_4X_LEN_FIELD_LEN));
	
	/* Padding the last buffer with zeros.*/
	numOfPadBytes = msduPtr->dataLen % 4;
	
	if( numOfPadBytes > 0 )
	{ 
#if 1
	/*
	* fixing the alignment bug. 
	       */
		numOfPadBytes = 4 - numOfPadBytes;
#endif	      
		tmpPtr = (UINT8 *) ((UINT32)msduPtr->lastBDPtr->data + msduPtr->lastBDPtr->length +
			msduPtr->lastBDPtr->dataOffset);
		for( i=0; i<numOfPadBytes; i++)
			tmpPtr[i] = 0x00;
		
		msduPtr->lastBDPtr->length += numOfPadBytes;
		msduPtr->dataLen += numOfPadBytes;
	}

	return OK; 
}
