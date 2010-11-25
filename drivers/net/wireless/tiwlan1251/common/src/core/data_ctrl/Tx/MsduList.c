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
/*		MODULE:	envList.c											       */
/*    PURPOSE:	Envelope list implementation        					   */
/*																		   */
/***************************************************************************/
 
#include "MsduList.h"
#include "osTIType.h"
#include "paramIn.h"
#include "osApi.h"

/*************************************************************************
 *                        msduList_CreateNewMsduList                     *
 *************************************************************************
DESCRIPTION: This function creates new Msdu list. 
                
INPUT:       maxNumOfElements : The maximum number of elements allowd 
			 pOs			  : Pointer to os abstraction layer

OUTPUT:      

RETURN:      MsduList_T                                                 
*************************************************************************/
MsduList_t* msduList_CreateNewMsduList(  TI_HANDLE hOs )
{
	
	MsduList_t* msduList = (MsduList_t*) os_memoryAlloc(hOs,sizeof(MsduList_t));
	if( msduList == NULL )
        return NULL;

	if(( msduList->hCriticalSectionProtect = os_protectCreate(hOs)) == NULL)
	{
	    WLAN_OS_REPORT(("FATAL ERROR: Could not Create Critical Section Protection for Msdu List - Aborting\n"));
	   
		/* free Msdu List Control Block */
		os_memoryFree(hOs, msduList, sizeof(MsduList_t));

		return NULL;
	}

    msduList->hOs = hOs;
	
	msduList->first = NULL;
	msduList->last = NULL;
	msduList->maxNumOfMsdu = 0;
	msduList->CurrNumOfMsdu = 0;
	msduList->ovFlowPolicy = DROP_NEW_PACKET;
	msduList->numOfOverFlow = 0;
	msduList->maxCurrOfMsdu =0;

	return msduList;
}

/*************************************************************************
 *                        msduList_ConfigMsduList                        *
 *************************************************************************
DESCRIPTION: This function configure the Msdu list. 
                
INPUT:       *this	:			A pointer to the list to configure
			 msduListConfig	:	Pointer to the Msdu list configuration
								structure.

OUTPUT:      

RETURN:      TI_STATUS: OK/NOK                                                 
*************************************************************************/
TI_STATUS msduList_ConfigMsduList( MsduList_t* this,	TI_HANDLE hMemMgr,
							 TI_HANDLE hReport, TI_HANDLE hOs,INT16 maxNumOfElements)
{
	if( hOs == NULL || hReport == NULL || 
		this == NULL || hMemMgr == NULL)
		return NOK;

	this->hReport = hReport;
	this->hMemMgr = hMemMgr;
	this->hOs = hOs;
	this->maxNumOfMsdu = maxNumOfElements;

	this->useAdmissionAlgo = FALSE;
	this->credit = 0;
	this->enableTransmissionTime = 0;
	this->lastTimeStamp = 0;
	this->mediumTime = 0;
	this->totalUsedTime = 0;
	
	this->highMediumUsageThreshold = 0;
	this->lowMediumUsageThreshold = 0;


	return OK;

}

/*************************************************************************
 *                        msduList_SetMsduListNumOfElements              *
 *************************************************************************
DESCRIPTION: This function configure the Msdu list max num of elements. 
                
INPUT:       *this	:			A pointer to the list to configure
			 maxNumOfElements:  max num of elements.
OUTPUT:      

RETURN:      TI_STATUS: OK/NOK                                                 
*************************************************************************/

TI_STATUS	msduList_SetMsduListNumOfElements( MsduList_t* this, UINT16 maxNumOfElements)
{
	if(this == NULL)
		return NOK;

	this->maxNumOfMsdu = maxNumOfElements;

	return OK;

}

/*************************************************************************
 *                        msduList_SetMsduListOverFlowPolicy             *
 *************************************************************************
DESCRIPTION: This function configure the Msdu list policy in case of over flow . 
                
INPUT:       *this	:			A pointer to the list to configure
			 QueueOvFlowPolicy:  over flow polict - new packet drop or old packet drop.
OUTPUT:      

RETURN:      TI_STATUS: OK/NOK                                                 
*************************************************************************/

TI_STATUS	msduList_SetMsduListOverFlowPolicy( MsduList_t* this, qOvFlowPolicy_e  QueueOvFlowPolicy)
{
	if(this == NULL)
		return NOK;

	this->ovFlowPolicy = QueueOvFlowPolicy;

	return OK;

}


/*************************************************************************
 *                        msduList_FreeMsduList                          *
 *************************************************************************
DESCRIPTION: This function free the Msdu list. 
                
INPUT:       *this	:	A pointer to the list to free
			 pOs	:	Pointer to os abstraction layer

OUTPUT:      

RETURN:      TI_STATUS: OK/NOK
*************************************************************************/
TI_STATUS msduList_FreeMsduList( MsduList_t* this)
{
	if( this->CurrNumOfMsdu != 0 )
	{
		if( msduList_EmptyMsduList( this ) != OK )
		{
			WLAN_REPORT_ERROR(this->hReport, TX_DATA_MODULE_LOG, 
				(" msduList_FreeMsduList() : failed \n"));
			return NOK;
		}
	}

	/* free protection */
	os_protectDestroy(this->hOs,this->hCriticalSectionProtect);

	/* free msdu control block */
	os_memoryFree(this->hOs, this, sizeof(MsduList_t));
	
	return OK;			

}

/*************************************************************************
 *                        msduList_EmptyMsduList						 *
 *************************************************************************
DESCRIPTION: This function free all the MSDUs from the Msdu list.
                
INPUT:       *this	:	A pointer to the list to empty
			 pOs	:	Pointer to os abstraction layer

OUTPUT:      

RETURN:      TI_STATUS: OK/NOK
*************************************************************************/
TI_STATUS msduList_EmptyMsduList( MsduList_t* this)
{
	UINT32 count;
	mem_MSDU_T* pTempMsdu;

	os_protectLock(this->hOs, this->hCriticalSectionProtect); /* START OF CRITICAL SECTION */
	if( this->CurrNumOfMsdu == 0 )
	{
		WLAN_REPORT_INFORMATION(this->hReport, TX_DATA_MODULE_LOG,
			(" msduList_EmptyMsduList() : List is empty \n"));
		os_protectUnlock(this->hOs, this->hCriticalSectionProtect); /* END OF CRITICAL SECTION */
		return OK;
	}
	
	/* update Tx status to NOK for all msdu in list */
	pTempMsdu = this->first;
	for(count = 0 ; count < this->CurrNumOfMsdu ; count++)
	{
		memMgr_MsduFreeArg2Get(pTempMsdu) = NOK;
		pTempMsdu = pTempMsdu->nextMSDUinList;
	}

	os_protectUnlock(this->hOs, this->hCriticalSectionProtect);
	/* free all msdu back to the memMngr */
	if ((wlan_memMngrFreeListOfMSDU(this->hMemMgr, memMgr_MsduHandle(	this->first))) != OK)
	{
		WLAN_REPORT_ERROR(this->hReport, TX_DATA_MODULE_LOG,  
			(" msduList_EmptyMsduList() : Msdu free failed \n"));
	}
	
	os_protectLock(this->hOs, this->hCriticalSectionProtect); /* START OF CRITICAL SECTION */
	this->first = NULL;
	this->last = NULL;
	this->CurrNumOfMsdu = 0;
	this->numOfOverFlow = 0;
	this->maxCurrOfMsdu = 0;

	this->useAdmissionAlgo = FALSE;
	this->credit = 0;
	this->enableTransmissionTime = 0;
	this->lastTimeStamp = 0;
	this->mediumTime = 0;
	this->totalUsedTime = 0;

	os_protectUnlock(this->hOs, this->hCriticalSectionProtect); /* END OF CRITICAL SECTION */
	
	return OK;
}

/*************************************************************************
 *                        msduList_MsduListIns                           *
 *************************************************************************
DESCRIPTION: This function insert MSDU to the list pointed by this.                 
                                                                                                   
INPUT:       *this :	A pointer to the list to insert. 
             *pMsdu :	Pointer to the MSDU to insert, the MSDU
                        is inserted to be the last in the list. If the 
                        list is full the first MSDU will be taken 
                        out and will be returned in this pointer.

OUTPUT:      *pMsdu :	In case the list is full,this poiter will hold 
						the dropt MSDU's. 

RETURN:      OK :		The MSDU has been inserted, there was enough 
						place in the list.
             NOK:		The list was full, the first MSDU is dropt, and 
						returned by *pMsdu.                                                                                                                                                  
************************************************************************/
TI_STATUS msduList_Insert( MsduList_t* this , mem_MSDU_T  **pMsdu )
{
	
	os_protectLock(this->hOs, this->hCriticalSectionProtect); /* START OF CRITICAL SECTION */
	
	if( this->CurrNumOfMsdu == 0 )
	{
		this->last = *pMsdu;
		this->first = *pMsdu;
		(*pMsdu)->nextMSDUinList = NULL;
		(*pMsdu)->prevMSDUinList = NULL;
		this->CurrNumOfMsdu++;
		os_protectUnlock(this->hOs, this->hCriticalSectionProtect); /* END OF CRITICAL SECTION */
		
		/* for debug */
		if( this->CurrNumOfMsdu > this->maxCurrOfMsdu )
			this->maxCurrOfMsdu = this->CurrNumOfMsdu;
		
		return OK;
	}
	else
	{
		if(  this->CurrNumOfMsdu == this->maxNumOfMsdu ) 
		{ 
			this->numOfOverFlow++;
			
			if(this->ovFlowPolicy == DROP_NEW_PACKET)
			{
				/* The list is full, remove the new coming msdu*/
				WLAN_REPORT_INFORMATION(this->hReport, TX_DATA_MODULE_LOG,  
					(" msduList_MsduListIns() : New Msdu has to be removed \n"));
				os_protectUnlock(this->hOs, this->hCriticalSectionProtect); /* END OF CRITICAL SECTION */
				
				/* for debug */
				if( this->CurrNumOfMsdu > this->maxCurrOfMsdu )
					this->maxCurrOfMsdu = this->CurrNumOfMsdu;

				return NOK;
				
			}else
			{
				/* The list is full, insert the new msdu and remove the first msdu*/
				this->last->nextMSDUinList = *pMsdu;
				(*pMsdu)->prevMSDUinList = this->last;
				(*pMsdu)->nextMSDUinList = NULL;
				this->last = *pMsdu;
				
				/* remove the first msdu from list */
				(*pMsdu) = this->first;
				this->first = this->first->nextMSDUinList;
				this->first->prevMSDUinList = NULL;
				(*pMsdu)->nextMSDUinList = NULL;
				
				WLAN_REPORT_INFORMATION(this->hReport, TX_DATA_MODULE_LOG,  
					(" msduList_MsduListIns() : First Msdu was removed \n"));
				os_protectUnlock(this->hOs, this->hCriticalSectionProtect); /* END OF CRITICAL SECTION */
				/* indicate that the first msdu has removed */
				
				/* for debug */
				if( this->CurrNumOfMsdu > this->maxCurrOfMsdu )
					this->maxCurrOfMsdu = this->CurrNumOfMsdu;

				return NOK;
				
			}
		}
		else			
		{ /* insert the MSDU to be the last. */
			this->last->nextMSDUinList = *pMsdu;
			(*pMsdu)->prevMSDUinList = this->last;
			(*pMsdu)->nextMSDUinList = NULL;
			this->last = *pMsdu;
		}   
		
		this->CurrNumOfMsdu++;
		
		os_protectUnlock(this->hOs, this->hCriticalSectionProtect); /* END OF CRITICAL SECTION */
	
		/* for debug */
		if( this->CurrNumOfMsdu > this->maxCurrOfMsdu )
			this->maxCurrOfMsdu = this->CurrNumOfMsdu;
		
		return OK;
	}
}


/*************************************************************************
 *                        msduList_MsduListGetFirst                        *
 *************************************************************************
DESCRIPTION: This function get MSDU to the list pointed by this.                 
                                                                                                   
INPUT:       *this : A pointer to the list to get. 

OUTPUT:      *pMsdu : A pointer to the first MSDU in the list. 

RETURN:      OK : There was an MSDU in the list, and it is assigned 
                  to the *pMsdu.
             NOK: The list was empty, *pMsdu is trush.     
************************************************************************/
TI_STATUS msduList_GetFirst( MsduList_t *this, mem_MSDU_T  **pMsdu)
{
	
	os_protectLock(this->hOs, this->hCriticalSectionProtect); /* START OF CRITICAL SECTION */
	
	if( this->CurrNumOfMsdu == 0 )
	{
		os_protectUnlock(this->hOs, this->hCriticalSectionProtect); /* END OF CRITICAL SECTION */
		return NOK;
	}
	
	*pMsdu = this->first;
	this->first = this->first->nextMSDUinList;
	if (this->first != NULL)
		this->first->prevMSDUinList = NULL;
	this->CurrNumOfMsdu--;
	(*pMsdu)->nextMSDUinList = NULL;
	
	os_protectUnlock(this->hOs, this->hCriticalSectionProtect); /* END OF CRITICAL SECTION */
	
	return OK;
}

/*************************************************************************
 *                        msduList_MsduListWatchFirst                       *
 *************************************************************************
DESCRIPTION: This function watch at the first SDU in the list. The 
             MSDU is not removed yet from the list.                 
                                                                                                   
INPUT:       *this : A pointer to the list to watch. 

OUTPUT:      *pMsdu : A pointer to the first MSDU in the list. 

RETURN:      OK : There was an MSDU in the list, and it is assigned 
                  to the *pMsdu.
             NOK: The list was empty, *pMsdu is trush.     
************************************************************************/
TI_STATUS msduList_WatchFirst( MsduList_t *this, mem_MSDU_T  **pMsdu)
{
	os_protectLock(this->hOs, this->hCriticalSectionProtect); /* START OF CRITICAL SECTION */
	
	if( this->CurrNumOfMsdu == 0 )
	{
		os_protectUnlock(this->hOs, this->hCriticalSectionProtect); /* END OF CRITICAL SECTION */
		return NOK;
	}

	*pMsdu = this->first;
	
	os_protectUnlock(this->hOs, this->hCriticalSectionProtect); /* END OF CRITICAL SECTION */
	
    return OK;
}

UINT32 msduList_getCurrNumOfMsdu(MsduList_t *this)
{
	return this->CurrNumOfMsdu;
}


/* Debug functions */
/*-----------------*/
void printMsduList(MsduList_t *this)
{
	WLAN_OS_REPORT(("Msdu List : \n"));

	WLAN_OS_REPORT(("first         = %X\n", this->first));
	WLAN_OS_REPORT(("last          = %X\n", this->last));
	WLAN_OS_REPORT(("maxNumOfMsdu  = %d\n", this->maxNumOfMsdu));
	WLAN_OS_REPORT(("CurrNumOfMsdu = %d\n", this->CurrNumOfMsdu));
	WLAN_OS_REPORT(("numOfOverFlow = %d\n", this->numOfOverFlow));
	WLAN_OS_REPORT(("maxCurrOfMsdu = %d\n", this->maxCurrOfMsdu));

	WLAN_OS_REPORT(("useAdmissionAlgo = %d\n", this->useAdmissionAlgo));
	WLAN_OS_REPORT(("credit = %d\n", this->credit));
	WLAN_OS_REPORT(("enableTransmissionTime  = %d\n", this->enableTransmissionTime));
	WLAN_OS_REPORT(("lastTimeStamp = %d\n", this->lastTimeStamp));
	WLAN_OS_REPORT(("mediumTime = %d\n", this->mediumTime));
	WLAN_OS_REPORT(("totalUsedTime = %d\n", this->totalUsedTime));

}

void printFullMsduList(MsduList_t *this)
{
	mem_MSDU_T* 	tmpMSDU;
	UINT32			i=0;

	printMsduList(this);

    tmpMSDU = this->first;
    while (++i, tmpMSDU != NULL) 
	{
        WLAN_OS_REPORT(("tmpMSDU %d = %X handle=%d tmpMSDU->nextMSDU=%X\n", i, tmpMSDU, tmpMSDU->handle, tmpMSDU->nextMSDUinList));
        tmpMSDU = tmpMSDU->nextMSDUinList;
    }

}
