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
#include "fourX.h"
#include "report.h"
#include "osApi.h"
#include "utils.h"
#include "802_11Defs.h"


static Wlan4XType_t fourX_parseRxFrame(fourX_t* pFourX, mem_MSDU_T* msduPtr);

static TI_STATUS fourX_MakeConcatDecision(fourX_t*				pFourX,
										   MsduList_t*			pMsduList,
										   hwTxInformation_t*	pHwTxInformation,
										   UINT32					numOfReadyMsdu,
										   UINT16*				concatFlags,
                                           UINT32*              numOfMsduToConcat);

static TI_STATUS fourX_prepareMsduListToConcat(fourX_t*		pFourX,
											   mem_MSDU_T**	returnMsduPtr,
											   MsduList_t*  pMsduList,
                                               UINT32       maxNumMsduToConcat);


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

fourX_t* fourX_create(TI_HANDLE hOs)
{
	fourX_t*			pFourX;
	deConcatenator_t*	pDeConcatenator;
	concatenator_t*		pConcatenator;
	ackEmul_t*			pAckEmul;


	if( hOs  == NULL )
	{
	    WLAN_OS_REPORT(("FATAL ERROR: fourX_create(): OS handle Error - Aborting\n"));
		return NULL;
	}

	/* alocate concatenator block */
	pFourX = os_memoryAlloc(hOs, (sizeof(fourX_t)));

	/* create 4x sub moduls */
	pConcatenator = concat_create(hOs);
	pDeConcatenator = deConcat_create(hOs);
	pAckEmul = ackEmul_create(hOs);


	if ( (!pFourX) || (!pConcatenator) || (!pDeConcatenator) || (!pAckEmul))
	{
		utils_nullMemoryFree(hOs, pDeConcatenator, sizeof(deConcatenator_t));
		utils_nullMemoryFree(hOs, pConcatenator, sizeof(concatenator_t));
		utils_nullMemoryFree(hOs, pAckEmul, sizeof(ackEmul_t));
		utils_nullMemoryFree(hOs, pFourX, sizeof(fourX_t));
	    WLAN_OS_REPORT(("FATAL ERROR: fourX_create(): Error Creating fourX module- Aborting\n"));
		return(NULL);
	}

	/* reset control module control block */
	os_memoryZero(hOs, pFourX, (sizeof(fourX_t)));

	pFourX->pConcatenator = pConcatenator;
	pFourX->pDeConcatenator = pDeConcatenator;
	pFourX->pAckEmul = pAckEmul;

	pFourX->hOs = hOs;

	return(pFourX);
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
TI_STATUS fourX_config(fourX_t*				pFourX,
					   TI_HANDLE			hOs,
					   TI_HANDLE			hReport,
					   TI_HANDLE			hMemMngr,
					   TI_HANDLE			hWhalCtrl,
					   TI_HANDLE			hTxData,
					   fourXInitParams_t*	fourXInitParams)
{
	/* check parameters validity */
	if( (pFourX == NULL) || (hOs == NULL) || (hReport == NULL) || 
		(hMemMngr == NULL) || (hWhalCtrl == NULL) || (hTxData == NULL))
	{
	    WLAN_OS_REPORT(("FATAL ERROR: fourX_config(): Parameters Error - Aborting\n"));
		return NOK;
	}

	/* set objects handles */
	pFourX->hOs = hOs;
	pFourX->hReport = hReport;
	pFourX->hMemMngr = hMemMngr;
	pFourX->hWhalCtrl = hWhalCtrl;
	pFourX->hTxData = hTxData;

	/* configure 4x parameters - TODO USE fourXInitParams */
	pFourX->desiredConcatenationEnable = DESIRED_CONCATENATION_ENABLE_DEF;
	pFourX->desiredCWMinEnable = DESIRED_CWMIN_ENABLE_DEF;
	pFourX->desiredCWComboEnable = DESIRED_CWCOMBO_ENABLE_DEF;
	pFourX->desiredAckEmulationEnable = DESIRED_ACKEMULATION_ENABLE_DEF;
	pFourX->desiredERP_ProtectionEnable = DESIRED_ERP_PROTECTION_ENABLE_DEF;
	pFourX->desiredMaxConcatSize = MAX_CONCAT_SIZE_DEF;
	pFourX->desiredCWMin = CW_MIN_DEF;
	pFourX->desiredCWMax = CW_MAX_DEF;


	/* configure 4x sub modules */
	concat_config(pFourX->pConcatenator, hOs, hReport, hMemMngr);
	deConcat_config(pFourX->pDeConcatenator, hOs, hReport, hMemMngr);
	ackEmul_config(pFourX->pAckEmul,hWhalCtrl,hOs,hReport,hMemMngr);


	WLAN_REPORT_INIT(pFourX->hReport, FOUR_X_MODULE_LOG, 
		(".....fouorX configured successfully\n"));

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

TI_STATUS fourX_destroy(fourX_t* pFourX)
{
	concat_destroy(pFourX->pConcatenator);
	deConcat_destroy(pFourX->pDeConcatenator);
	ackEmul_destroy(pFourX->pAckEmul);

	/* free control module controll block */
	os_memoryFree(pFourX->hOs, pFourX, sizeof(fourX_t));

	return OK;
}


static Wlan4XType_t fourX_parseRxFrame(fourX_t* pFourX, mem_MSDU_T* msduPtr)
{
    dot114xMsdu_t*	pdot114xHeader;
	UINT8			tiSnapDataArray[8] = {0xAA,0xAA,0x03,0x08,0x00,0x28,0x60,0xD0};

	/* Check frame len validity */
	if(msduPtr->dataLen < sizeof(dot114xMsdu_t))
		return NOT_4X_MSDU;

    pdot114xHeader = (dot114xMsdu_t*)memMgr_BufData(msduPtr->firstBDPtr);

    /* 
       Verify a TI SNAP header. 
    */
    if( os_memoryCompare(pFourX->hOs,
						 (void*)&(pdot114xHeader->msduHeader.snapHeader), 
						 tiSnapDataArray, 
						 sizeof(Wlan_LlcHeader_T)) != 0)
	{
		return NOT_4X_MSDU;
	}

    switch (pdot114xHeader->header4x.type) 
	{
		
    case CONCATENATION : 
        return CONCATENATION;
/*		break; - unreachable*/
		
    case MANAGMENT_4X :
        return MANAGMENT_4X;
/*		break; - unreachable*/
		
    default: 
		return NOT_4X_MSDU;
/*		break; - unreachable*/
		
    }
}

TI_STATUS fourX_rxMsdu(fourX_t*	pFourX, mem_MSDU_T**	rxMsduPtr)
{
    Wlan4XType_t	type4xMsdu; 
	mem_MSDU_T*		currMsduPtr;

	type4xMsdu = fourX_parseRxFrame(pFourX, *rxMsduPtr);

	switch (type4xMsdu) 
	{
		case CONCATENATION :
		   /*
			* Deconcatenate Msdu
			*/
            WLAN_REPORT_INFORMATION(pFourX->hReport, FOUR_X_MODULE_LOG, 
			        (" Received Concat msdu \n"));
			if(deConcat_deConcatMsdu(pFourX->pDeConcatenator, rxMsduPtr) != OK)
			{
		        WLAN_REPORT_ERROR(pFourX->hReport, FOUR_X_MODULE_LOG, 
			        (" Failed to deconcat packet \n"));
			
				return NOK;
			}
			break;
			
		case MANAGMENT_4X :
			break;
			
		default: 
			break;
			
    }

	if(pFourX->ackEmulationEnable)
	{
		/* call ack emulation for each packet */
		currMsduPtr = *rxMsduPtr;
		while(currMsduPtr)
		{
			wdrv_ackEmulationRxPacket(pFourX->pAckEmul, currMsduPtr);
			currMsduPtr = currMsduPtr->nextMSDUinList;
		}
	}
	return OK;

}

#define NUM_MSDU_TO_COPY 8

/***************************************************************************
*							fourX_CopyReplace
****************************************************************************
* DESCRIPTION:	This function copy OS data blocks to local data blocks 
*				in the MSDU in order to release OS resources and to enable
*				high stream of data from the OS
* 
* INPUTS:		
*		
* OUTPUT:		
* 
* RETURNS:		
***************************************************************************/
TI_STATUS fourX_CopyReplace(fourX_t* pFourX, mem_MSDU_T **pMsdu, MsduList_t *pMsduList, mem_MSDU_T **pNewMsdu)
{
	mem_MSDU_T		*pOrigMsdu;

	pOrigMsdu = (*pMsdu);

	/* 
	 * The copy is to new msdu pointed by pMsdu 
	 */
	if(txData_copyPacketToMsdu(pFourX->hTxData, pMsdu, 0 /* dont FreeOldMsdu */) != OK)
	{
		/* no msdu to transmit */
		WLAN_REPORT_INFORMATION(pFourX->hReport, FOUR_X_MODULE_LOG,  
			(" fourX_CopyReplace() : txData_copyPacketToMsdu FAILED  \n"));

		return NOK;
	}

	/* 
	 * Still use the old Msdu with its pointers, so swap the BDs of the new and the orig Msdu 
	 */
	wlan_memMngrSwapMsdu(pFourX->hMemMngr, pOrigMsdu, (*pMsdu));

	/* return the new msdu to free later */
	(*pNewMsdu) = (*pMsdu);
	/* use the orig Msdu */
	(*pMsdu) = pOrigMsdu;

	return OK;
}


/***************************************************************************
*							fourX_CopyOsData
****************************************************************************
* DESCRIPTION:	This function scan the Msdu list and copy OS data blocks to 
*				local data blocks .
*				in the MSDU in order to release OS resources and to enable
*				high stream of data from the OS
* 
* INPUTS:		
*		
* OUTPUT:		
* 
* RETURNS:		
***************************************************************************/
UINT32 fourX_CopyOsData(fourX_t* pFourX, MsduList_t *pMsduList)
{
	int NumMsduToCopy = 0;
	mem_MSDU_T *pMsdu;
	mem_MSDU_T *pNewMsdu=NULL;
	mem_MSDU_T *pMsduFreeList=NULL;
	mem_MSDU_T *pMsduFreeCurr=NULL;
	mem_MSDU_T *pKeepNext=NULL;
	int i;
	int NumOfCopiedPackets=0;

	os_protectLock(pMsduList->hOs, pMsduList->hCriticalSectionProtect); /* START OF CRITICAL SECTION */

	if (pMsduList->CurrNumOfMsdu == 0)
	{
		pFourX->counters.count6 = 0;
		pFourX->counters.count7 = 0;
		os_protectUnlock(pMsduList->hOs, pMsduList->hCriticalSectionProtect); /* END OF CRITICAL SECTION */
		return 0;
	}

	/* 
	 * Find how many msdu to copy 
	 */
	if (pMsduList->CurrNumOfMsdu <= NUM_MSDU_TO_COPY)
		NumMsduToCopy = pMsduList->CurrNumOfMsdu;
	else
		NumMsduToCopy = NUM_MSDU_TO_COPY;

	pMsdu=pMsduList->first;
		
	/* 
	 * Start the copy 
	 */
	for (i=0; (i<NumMsduToCopy) ; i++)
	{
		if (pMsdu == NULL)
		{
			WLAN_REPORT_ERROR(pFourX->hReport, FOUR_X_MODULE_LOG,  
				(" fourX_CopyMsdu() : fourX_CopyReplace FAILED, pMsdu is NULL i=%d, Num=%d (Actual=%d) \n", i, NumMsduToCopy, pMsduList->CurrNumOfMsdu));

			break;
		}
		/* 
		 * Already copied - skip it 
		 */
		if (pMsdu->freeFunc == NULL)
		{
			pMsdu=pMsdu->nextMSDUinList;
			NumOfCopiedPackets++;
			continue;
		}

		WLAN_REPORT_INFORMATION(pFourX->hReport, FOUR_X_MODULE_LOG,  
			(" fourX_CopyMsdu() : i=%d, CopyReplace 0x%x\n", i , pMsdu));
		
		/* 
		 * Copy and replace in the list 
		 */
		if(fourX_CopyReplace(pFourX, &pMsdu, pMsduList, &pNewMsdu) != OK)
		{
			/* no msdu to transmit */
			WLAN_REPORT_ERROR(pFourX->hReport, FOUR_X_MODULE_LOG,  
				(" fourX_CopyMsdu() : fourX_CopyReplace FAILED  \n"));

			break;
		}

		/*
		 * Enter the new Msdu to the free list 
		 */
		if (pMsduFreeList == NULL)
		{
			pMsduFreeList = pNewMsdu;
		}
		else
		{
			pMsduFreeCurr = pMsduFreeList;
			while (pMsduFreeCurr->nextMSDUinList != NULL)
				pMsduFreeCurr = pMsduFreeCurr->nextMSDUinList;
			pMsduFreeCurr->nextMSDUinList = pNewMsdu;
		}
		pNewMsdu->nextMSDUinList = NULL;

		/* copy the next msdu */
		pMsdu=pMsdu->nextMSDUinList;
		NumOfCopiedPackets++;
	}

	pFourX->counters.count6 = pMsduList->CurrNumOfMsdu;
	pFourX->counters.count7 = i;

	/* !!!! This is the right place for the unlock */
	os_protectUnlock(pMsduList->hOs, pMsduList->hCriticalSectionProtect); /* END OF CRITICAL SECTION */

	/* 
	 * free loop, do not call the free inside lock !!!
	 */
	pMsduFreeCurr = pMsduFreeList;
	while (pMsduFreeCurr != NULL)
	{
		pKeepNext = pMsduFreeCurr->nextMSDUinList;

		wlan_memMngrFreeMSDU(pFourX->hMemMngr, memMgr_MsduHandle(pMsduFreeCurr));

		pMsduFreeCurr=pKeepNext;
	}

	return NumOfCopiedPackets;
}
	


TI_STATUS fourX_txMsduDeQueue(fourX_t*				pFourX,
							  mem_MSDU_T**			returnMsduPtr,
							  MsduList_t*			pMsduList,
							  hwTxInformation_t*	pHwTxInformation)
{
	TI_STATUS		status;
	mem_MSDU_T*		firstMsduPtr;
	UINT16			concatFlags = 0;
    UINT32          numOfMsduToConcat;
	UINT32			numMsdu = 0;

	*returnMsduPtr = NULL;


#ifdef NO_COPY_NDIS_BUFFERS	
	/*
	 * Scan the Msdu list and copy OS data blocks to local data blocks .
	 * in the MSDU in order to release OS resources and to enable
	 * high stream of data from the OS.
	 */
	numMsdu = fourX_CopyOsData(pFourX, pMsduList);
	/* This function copied up to 8 or numOfMsdu in list packets from OS to Shared memory
	 * As there is NDISfreeFunc after the copy, New un-copied packets can enter the msduList, 
	 * but the scheduler was blocked from entering the send again.
	 */
#else

	if (pMsduList->CurrNumOfMsdu < NUM_MSDU_TO_COPY )
		numMsdu = pMsduList->CurrNumOfMsdu; 	
	else 
		numMsdu = NUM_MSDU_TO_COPY;
#endif
	
	/*if(pFourX->concatenationEnable != TRUE)*/
	status = fourX_MakeConcatDecision(pFourX,pMsduList,pHwTxInformation,numMsdu, 
                                       &concatFlags,&numOfMsduToConcat);


	switch(status)
	{
	case MAKE_CONCATENATION:
	   
	   /*
		* Make Concatenation
		*/
	
		fourX_prepareMsduListToConcat(pFourX, 
									  &firstMsduPtr, 
									  pMsduList,
                                      numOfMsduToConcat);

		if(firstMsduPtr == NULL)
			return NOK;

		
		return concat_concatMsduList(pFourX->pConcatenator, 
							  firstMsduPtr,
							  returnMsduPtr,
							  concatFlags);


	case SEND_ONE_MSDU:

	   /*
		* Send Only One Msdu
		*/

		if((msduList_GetFirst( pMsduList, returnMsduPtr )) != OK)
		{
			/* msdu list is empty - should never reach here */
			WLAN_REPORT_ERROR(pFourX->hReport, FOUR_X_MODULE_LOG,  
				(" fourX_txMsduDeQueue:msduList_GetFirst() : No Msdu to get in list \n"));
			
			return NOK;
		}

		return OK;

	case DO_NOT_SEND_MSDU:

	   /*
		* Don't Send any Msdu
		*/

		return NOK;

	default:
		break;

	}
	return OK;
}

TI_STATUS fourX_txMsduBeforInsertToQueue(fourX_t* pFourX, 
										 mem_MSDU_T** msduPtr)
{
/* no ACK emulation !!!!!!!!!!!!!!!!!! */
#if 0
	UINT32 discardPacket;
	if(pFourX->ackEmulationEnable)
	{
		wdrv_ackEmulationTxPacket(pFourX->pAckEmul, *msduPtr, &discardPacket);
		if (discardPacket == TRUE)
		{
			wlan_memMngrFreeMSDU(pFourX->hMemMngr, (*msduPtr)->handle);
			(*msduPtr) = NULL;
			return OK;
		}
	}
#endif
	return OK;
}

static TI_STATUS fourX_prepareMsduListToConcat(fourX_t*		pFourX,
											   mem_MSDU_T**	returnMsduPtr,
											   MsduList_t*	pMsduList,
                                               UINT32       maxNumMsduToConcat)
{
	mem_MSDU_T*		currentMsduPtr;
	mem_MSDU_T*		PrevMsduPtr=NULL;
	UINT32			totalLen = 0;
	BOOL			firsdMsduInList = TRUE;
	UINT32			numOfMsdu = 0;

	
	*returnMsduPtr = NULL;

	while(maxNumMsduToConcat--)
	{
		/* prepare a link list of msdu in the concatenator format */
		if((msduList_WatchFirst( pMsduList, &currentMsduPtr )) != OK)
		{
			/* msdu list is empty - can return now */
			WLAN_REPORT_INFORMATION(pFourX->hReport, FOUR_X_MODULE_LOG,  
				(" fourX_txMsduDeQueue: msduList_WatchFirst() : No Msdu to watch \n"));
			break;
		}
		totalLen += currentMsduPtr->dataLen;
		
		if(totalLen > 4032/*pFourX->currentMaxConcatSize*/)
			break;
		
		if((msduList_GetFirst( pMsduList, &currentMsduPtr )) != OK)
		{
			/* msdu list is empty - should never reach here */
			WLAN_REPORT_ERROR(pFourX->hReport, FOUR_X_MODULE_LOG,  
				(" fourX_txMsduDeQueue:msduList_GetFirst() : No Msdu to get in list \n"));
			break;
		}

		/* In this phase, the free func should be NULL !!!!!!!!!!! */
		if (currentMsduPtr->freeFunc)
		{
			WLAN_REPORT_ERROR(pFourX->hReport, FOUR_X_MODULE_LOG,  
				(" fourX_txMsduDeQueue:msduList_GetFirst() : fourX_prepareMsduListToConcat, free funct is not NULL !!!!!!!\n"));
		}

		numOfMsdu++;

		if(firsdMsduInList == TRUE)
		{
			*returnMsduPtr = currentMsduPtr;
			firsdMsduInList = FALSE;
		}
		else
		{
			PrevMsduPtr->nextMSDUinList = currentMsduPtr;
		}
		
		PrevMsduPtr = currentMsduPtr;
	}

	return OK;
}

static TI_STATUS fourX_MakeConcatDecision(fourX_t*				pFourX,
										   MsduList_t*			pMsduList,
										   hwTxInformation_t*	pHwTxInformation,
										   UINT32					numOfReadyMsdu,
										   UINT16*				concatFlags,
                                           UINT32*              numOfMsduToConcat)
{
	memMgrResources_t memMgrResources;

#ifdef TNETW_MASTER_MODE
	UINT32 limitResourcees;
#endif

	*numOfMsduToConcat = 0;
	
	/* get MemoryMgr resources for concatenation */
	wlan_memMngrGetMemMgrResources(pFourX->hMemMngr, &memMgrResources);

#ifdef TNETW_MASTER_MODE
	limitResourcees = MIN(memMgrResources.numOfFreeBufPool2 ,(MIN(memMgrResources.numOfFreeBufPool1-1 , pHwTxInformation->hwNumOfFreeBDs)));



	/* No free space in HW to send */
    if( (pHwTxInformation->hwNumOfFreeMsdu == 0) || (limitResourcees < 2) )
    {
        pFourX->counters.count1++;
        return DO_NOT_SEND_MSDU;
	}

    /*
	 * Phase 1: At least 3 waiting msdu for concat (and enough space in HW)
	 * --------------------------------------------------------------------
	 * In case there are 2 msdu the decision will be later
     * We have resources to send, decide if to concat, send single or wait.
     */ 
    if(  numOfReadyMsdu >= 3 )
    {
        pFourX->counters.count2++;
        /* We have enough msdus to concat. */
		*concatFlags = *concatFlags | WLAN_4X_CONCAT_MORE_BIT;

		/* not enough free bd to send or concat - need at least 2 msdu and 2 bd for each + concat header bd */
		if (limitResourcees < 5)
			return DO_NOT_SEND_MSDU;

		/* minimum 2 bd for each msdu and one for cocat header */        
		*numOfMsduToConcat = MIN( (limitResourcees - 1)>>1, numOfReadyMsdu);

		/* minimum 2 msdu to concat */        
		if (*numOfMsduToConcat < 2)
			return SEND_ONE_MSDU;

		return MAKE_CONCATENATION;
    }


	/*
	 * Phase 2: Less than 3 waiting msdu, and the HW already has Msdu to send
	 * ----------------------------------------------------------------------
	 * It is allowed to delay the msdu and continue queueing more MSDU 
	 */
	if(pHwTxInformation->hwNumOfBusyMsdu > 1)
	{
        pFourX->counters.count3++;
	   /*
		* ACX has enough packets to send, delay the current sending.
		*/
		return DO_NOT_SEND_MSDU;
	}
 
	/*
	 * Phase 3: Less than 3 waiting msdu, and the HW is free to send
	 * -------------------------------------------------------------
	 * It is NOT allowed to delay the msdu, so send it 
	 */
    else 
    {
	   /*
		* Phase 4: There are 2 msdu to concat and the HW is free to send
		* --------------------------------------------------------------
        */
        if(  numOfReadyMsdu > 1 ) 
		{
            pFourX->counters.count4++;
            /* We have enough msdus to concat. */
            *concatFlags = *concatFlags | WLAN_4X_CONCAT_MORE_BIT;

			/* not enough free bd to send or concat - need at least 2 msdu and 2 bd for each + concat header bd */
			if (limitResourcees < 5)
				return DO_NOT_SEND_MSDU;

			/* minimum 2 bd for each msdu and one for cocat header */        
			*numOfMsduToConcat = MIN( (limitResourcees - 1)>>1, numOfReadyMsdu);

			/* minimum 2 msdu to concat */        
			if (*numOfMsduToConcat < 2)
				return SEND_ONE_MSDU;

	         return MAKE_CONCATENATION;
        }
	   /*
		* Phase 5: There are only one msdu to send, send it (no concatination)
		* --------------------------------------------------------------------
		*/
        else 
        {
            pFourX->counters.count5++;
			/* 
			 * There space in HW and only one MSDU to send, send it as single.
			 */
            return SEND_ONE_MSDU;
        }
    }


#else /* SLAVE_MODE */
		if((pHwTxInformation->hwTotalAvailMem > 4095) && (numOfReadyMsdu > 1))
			{
				*concatFlags = *concatFlags | WLAN_4X_CONCAT_MORE_BIT;
				return MAKE_CONCATENATION;
			}
		else
        {
			if(pHwTxInformation->hwTotalAvailMem > pMsduList->first->dataLen)
				return SEND_ONE_MSDU;
			else
				return DO_NOT_SEND_MSDU;
        }

#endif /* MASTER/SALVE modes */



}

	
/* debug functions */
void fourX_printParams(fourX_t* pFourX)
{
	
	WLAN_OS_REPORT(("          FOUR X Parameters          \n"));
	WLAN_OS_REPORT(("-------------------------------------\n"));

	WLAN_OS_REPORT(("hOs                           = 0x%X\n",pFourX->hOs  ));
	WLAN_OS_REPORT(("hReport                       = 0x%X\n",pFourX->hReport  ));
	WLAN_OS_REPORT(("hMemMngr                      = 0x%X\n\n",pFourX->hMemMngr  ));

	WLAN_OS_REPORT(("concatenationEnable           = %d\n",pFourX->concatenationEnable  ));
	WLAN_OS_REPORT(("CWMinEnable                   = %d\n",pFourX->CWMinEnable  ));
	WLAN_OS_REPORT(("CWComboEnable                 = %d\n",pFourX->CWComboEnable  ));
	WLAN_OS_REPORT(("ackEmulationEnable            = %d\n",pFourX->ackEmulationEnable  ));
	WLAN_OS_REPORT(("ERP_ProtectionEnable          = %d\n\n",pFourX->ERP_ProtectionEnable  ));

	WLAN_OS_REPORT(("desiredConcatenationEnable    = %d\n",pFourX->desiredConcatenationEnable  ));
	WLAN_OS_REPORT(("desiredCWMinEnable            = %d\n",pFourX->desiredCWMinEnable  ));
	WLAN_OS_REPORT(("desiredCWComboEnable          = %d\n",pFourX->desiredCWComboEnable  ));
	WLAN_OS_REPORT(("desiredAckEmulationEnable     = %d\n",pFourX->desiredAckEmulationEnable  ));
	WLAN_OS_REPORT(("desiredERP_ProtectionEnable   = %d\n\n",pFourX->desiredERP_ProtectionEnable  ));

	WLAN_OS_REPORT(("desiredMaxConcatSize          = %d\n",pFourX->desiredMaxConcatSize  ));
	WLAN_OS_REPORT(("desiredCWMin                  = %d\n",pFourX->desiredCWMin  ));
	WLAN_OS_REPORT(("desiredCWMax                  = %d\n\n",pFourX->desiredCWMax  ));


	/* AP supported features */


	/* 4x parameters */
	WLAN_OS_REPORT(("currentMaxConcatSize          = %d\n",pFourX->currentMaxConcatSize  ));
	WLAN_OS_REPORT(("currentCWMin                  = %d\n",pFourX->currentCWMin  ));
	WLAN_OS_REPORT(("currentCWMax                  = %d\n\n",pFourX->currentCWMax  ));


	WLAN_OS_REPORT(("ApFourX_Capabilities.fourXProtocolVersion        = %d\n", pFourX->ApFourX_Capabilities.fourXProtocolVersion));
	WLAN_OS_REPORT(("-------------------------------------------------\n"));

	WLAN_OS_REPORT(("AP_Cap.concatenationParams.enableDisable          = %d\n", pFourX->ApFourX_Capabilities.concatenationParams.enableDisable));
	WLAN_OS_REPORT(("AP_Cap.concatenationParams.concatenationSize      = %d\n", pFourX->ApFourX_Capabilities.concatenationParams.concatenationSize));

	WLAN_OS_REPORT(("AP_Cap.contentionWindowParams.enableDisable       = %d\n", pFourX->ApFourX_Capabilities.contentionWindowParams.enableDisable));
	WLAN_OS_REPORT(("AP_Cap.contentionWindowParams.CWMin               = %d\n", pFourX->ApFourX_Capabilities.contentionWindowParams.CWMin));
	WLAN_OS_REPORT(("AP_Cap.contentionWindowParams.CWMax               = %d\n", pFourX->ApFourX_Capabilities.contentionWindowParams.CWMax));

	WLAN_OS_REPORT(("AP_Cap.CWCombParams.enableDisable                 = %d\n", pFourX->ApFourX_Capabilities.CWCombParams.enableDisable));
	WLAN_OS_REPORT(("AP_Cap.CWCombParams.DIFS                          = %d\n", pFourX->ApFourX_Capabilities.CWCombParams.DIFS));
	WLAN_OS_REPORT(("AP_Cap.CWCombParams.SLOT                          = %d\n", pFourX->ApFourX_Capabilities.CWCombParams.SLOT));
	WLAN_OS_REPORT(("AP_Cap.CWCombParams.CWMin                         = %d\n", pFourX->ApFourX_Capabilities.CWCombParams.CWMin));
	
	WLAN_OS_REPORT(("AP_Cap.ackEmulationParams.enableDisable           = %d\n", pFourX->ApFourX_Capabilities.ackEmulationParams.enableDisable));
	
	WLAN_OS_REPORT(("AP_Cap.ERP_ProtectionParams.enableDisable         = %d\n", pFourX->ApFourX_Capabilities.ERP_ProtectionParams.enableDisable));

   	WLAN_OS_REPORT(("No Free Msdu in ACX   = %d\n", pFourX->counters.count1));
   	WLAN_OS_REPORT(("Concat more than 2    = %d\n", pFourX->counters.count2));
   	WLAN_OS_REPORT(("Delay sending         = %d\n", pFourX->counters.count3));
   	WLAN_OS_REPORT(("Concat less than 3    = %d\n", pFourX->counters.count4));
   	WLAN_OS_REPORT(("send one msdu         = %d\n", pFourX->counters.count5));

	WLAN_OS_REPORT(("Msdu in Queue Total   = %d\n", pFourX->counters.count6));
	WLAN_OS_REPORT(("Msdu in Queue Copied  = %d\n", pFourX->counters.count7));

}
