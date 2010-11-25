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


#include "whalEndpntEnt_api.h"

                    
/* debug counters - temporary to remove*/
UINT32		duplicateMpdu = 0;
UINT32		msduFreeNotInOrder = 0;
UINT32		mpduValidFrames = 0;
UINT32		duplicateMsdu = 0;


/*
 * ----------------------------------------------------------------------------
 * Function : collectEntryFree
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
static void collectEntryFree (collectEntry_t* pCollectEnt)
{
  	pCollectEnt->seqNum		= 0; 
	pCollectEnt->fragNum	= 0;
  	pCollectEnt->msduPtr 	= NULL;
  	pCollectEnt->lastBdPtr 	= NULL;
  	pCollectEnt->timeStamp	= 0;
  	pCollectEnt->collect 	= FALSE;
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalEndpnt_Create
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
TI_HANDLE whalEndpnt_Create (TI_HANDLE hWhalCtrl, TI_HANDLE hOs)
{
	WHAL_ENDPNT* pWhalEndpnt;

	pWhalEndpnt = (WHAL_ENDPNT *)os_memoryAlloc (hOs, sizeof(WHAL_ENDPNT));
	if (pWhalEndpnt == NULL)
		return NULL;

	os_memoryZero (hOs, pWhalEndpnt, sizeof(WHAL_ENDPNT));

	pWhalEndpnt->pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
	pWhalEndpnt->hOs = hOs;

	return((TI_HANDLE)pWhalEndpnt);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalEndpnt_Config
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalEndpnt_Config (TI_HANDLE hWhalEndpnt, whalEndpnt_config_t* pWhalEndpntCfg)
{
	WHAL_ENDPNT* pWhalEndpnt = (WHAL_ENDPNT *)hWhalEndpnt;
	
	/* Save configuration parameters */
	pWhalEndpnt->hReport = pWhalEndpntCfg->hReport;
	pWhalEndpnt->hMemMngr = pWhalEndpntCfg->hMemMngr;
	
    return (OK);
}			    
						    
/*
 * ----------------------------------------------------------------------------
 * Function : whalEndpnt_IsCollect
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
BOOL whalEndpnt_IsCollect (TI_HANDLE hWhalEndpnt)
{
	WHAL_ENDPNT* pWhalEndpnt = (WHAL_ENDPNT *)hWhalEndpnt;
	
	return (pWhalEndpnt->collectEntry.collect);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalEndpnt_FragCollect
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
collectStatus_e	whalEndpnt_FragCollect (TI_HANDLE hWhalEndpnt, mem_MSDU_T* pMpdu, mem_MSDU_T** pMsdu)
{
    dot11_header_t*	pHeader;
    UINT32          fragNumber;
    UINT32          seqNumber;
    UINT32          moreFrag;
	WHAL_ENDPNT* 	pWhalEndpnt = (WHAL_ENDPNT *)hWhalEndpnt;
	collectEntry_t* collectEnt 	= &(pWhalEndpnt->collectEntry);
    
	
	/* initial checking on the received MPDU*/
	if ((pMpdu == NULL) || (pMpdu->firstBDPtr == NULL) || (pMpdu->lastBDPtr == NULL))
		WLAN_REPORT_FATAL_ERROR(pWhalEndpnt->hReport, HAL_RX_MODULE_LOG, 
								(" whalEndpnt_FragCollect: original Mpdu pointers with NULL !\n"));
	
	*pMsdu = NULL;
	pHeader = (dot11_header_t*)memMgr_MsduHdrAddr(pMpdu);

	fragNumber = (pHeader->seqCtrl & 0x000F);
    seqNumber  = (pHeader->seqCtrl & 0xFFF0) >> 4;
    moreFrag = (pHeader->fc & DOT11_FC_MORE_FRAG);

	if ((seqNumber == collectEnt->seqNum) && (collectEnt->collect == TRUE)) 
	{
		/* This MPDU belongs to the current collection of the MSDU */
        if (fragNumber == (collectEnt->fragNum+1)) 
		{
            /* This is the next MPDU of the current MSDU */
            /* Update the new MPDU */
            collectEnt->fragNum++;
											  /* (!!!YV) to change WLAN_HDR_LEN to pMpdu->headerLen*/ 
			collectEnt->msduPtr->dataLen += (pMpdu->dataLen - WLAN_HDR_LEN); /* Update MSDU length */
            pMpdu->firstBDPtr->dataOffset += WLAN_HDR_LEN; /* Point to the start of the Data */
            pMpdu->firstBDPtr->length -= WLAN_HDR_LEN;
			
			/* internal structure checking*/
			if (collectEnt->lastBdPtr == NULL)
            {
				WLAN_REPORT_ERROR(pWhalEndpnt->hReport, HAL_RX_MODULE_LOG, 
								  (" whalEndpnt_FragCollect: collectEnt->lastBdPtr is NULL !!!!!!!!!!\n"));
				wlan_memMngrFreeMSDU(pWhalEndpnt->hMemMngr, pMpdu->handle);
				msduFreeNotInOrder++;
				return MPDU_DUP_DROP;
            }
			collectEnt->lastBdPtr->nextBDPtr = pMpdu->firstBDPtr; /* Update the BD list */
	        collectEnt->lastBdPtr = pMpdu->lastBDPtr;
			collectEnt->msduPtr->lastBDPtr = pMpdu->lastBDPtr; 

			/* Recycle the recieved MPDU header only*/
			pMpdu->firstBDPtr = NULL;
			wlan_memMngrFreeMSDU(pWhalEndpnt->hMemMngr, pMpdu->handle);

            mpduValidFrames++;
        }
        else if (fragNumber <= collectEnt->fragNum) 
			 {
				/* Order ERROR or Duplication- NEED to Drop the recieved MPDU */
				wlan_memMngrFreeMSDU(pWhalEndpnt->hMemMngr, pMpdu->handle);
				duplicateMpdu++;
				return MPDU_DUP_DROP;
	 		 }
	 		 else
			 {
				/* We lost one of the Fragments fragNumber > (collectEnt->fragNum+1)
				   Need to free all the Fragments of the current MSDU */
				wlan_memMngrFreeMSDU(pWhalEndpnt->hMemMngr, pMpdu->handle);
				wlan_memMngrFreeMSDU(pWhalEndpnt->hMemMngr, collectEnt->msduPtr->handle);
				collectEntryFree (collectEnt);
				msduFreeNotInOrder++;
				return MSDU_DROP;
			 }
    }
    else 
	{  
		/* This MPDU is part of a new MSDU */
        if (collectEnt->collect == TRUE) 
		{
            /* Still in the middle of collecting the previous MSDU
               Need to drop the previous MSDU */
            wlan_memMngrFreeMSDU(pWhalEndpnt->hMemMngr, collectEnt->msduPtr->handle);
            collectEntryFree (collectEnt);
            msduFreeNotInOrder++;
        }
        
        if (fragNumber != 0) 
		{
            /* MPDU not in order. This is should first MPDU and must be 0
			   Need to drop the MPDU */
            wlan_memMngrFreeMSDU(pWhalEndpnt->hMemMngr, pMpdu->handle);
            msduFreeNotInOrder++;
			WLAN_REPORT_INFORMATION(pWhalEndpnt->hReport, HAL_RX_MODULE_LOG, 
							  (" whalEndpnt_FragCollect:  MPDU not in order. fragNumber = %d seqNumber = %d",
							  fragNumber, seqNumber));
	      	
			return MPDU_DROP;
        }
        
	/* New MSDU */
        mpduValidFrames++;
        collectEnt->msduPtr = pMpdu;
	    collectEnt->lastBdPtr = pMpdu->lastBDPtr;
        collectEnt->collect = TRUE;
        collectEnt->seqNum = seqNumber;


    }

    /* Check if the MSDU collection finished */
    if (!moreFrag) 
	{
        /* MSDU collection terminated */
        *pMsdu = collectEnt->msduPtr;
        collectEntryFree (collectEnt);
        return MSDU_READY;
    }


    return MSDU_IN_PROGRESS;
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalEndpnt_Destroy
 *
 * Input    : 
 * Output   :									 
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalEndpnt_Destroy (TI_HANDLE hWhalEndpnt)
{
	WHAL_ENDPNT* pWhalEndpnt = (WHAL_ENDPNT *)hWhalEndpnt;

	if (pWhalEndpnt)
		os_memoryFree (pWhalEndpnt->hOs, pWhalEndpnt, sizeof(WHAL_ENDPNT));
	
	return (OK);
}




