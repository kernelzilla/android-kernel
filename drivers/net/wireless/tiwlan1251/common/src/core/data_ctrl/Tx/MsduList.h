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
/*															           	   */
/*		MODULE:	MsduList.c									           	   */
/*    PURPOSE:	MSDU list implementation        				       	   */
/*															               */
/***************************************************************************/

#ifndef _MSDU_LIST_H_
#define _MSDU_LIST_H_

#include "memMngrEx.h"
#include "report.h" 


/************************************************************************/
/* MSDU link list definition.											*/
/************************************************************************/
typedef struct 
{
	TI_HANDLE	hMemMgr;		/* handle to memory manger */
	TI_HANDLE	hReport;		/* handle to Report object */
	TI_HANDLE	hOs;			/* handle to Os Abstraction object */

	TI_HANDLE	hCriticalSectionProtect;

	mem_MSDU_T *first;			/* Points to the first Msdu's in the list. */
	mem_MSDU_T *last;			/* Points to the last Msdu's is the list. */
	UINT16   maxNumOfMsdu;		/* The maximum number of Msdu's allowed to be in the queue. */
	UINT16   CurrNumOfMsdu;		/* The current number of Msdu's.         */
	qOvFlowPolicy_e  ovFlowPolicy; /* tx over flow policy */

	UINT16	numOfOverFlow ;
	UINT16	maxCurrOfMsdu ;

	/* admission ctrl params */
	UINT8	acId;	/* the AC used for this Tx queue. */
	BOOL	useAdmissionAlgo;
	INT32 	credit;
	UINT32	enableTransmissionTime ;
	UINT32	lastTimeStamp;
	UINT32	mediumTime;
	
	UINT32 totalUsedTime;

	trafficAdmState_e  admissionState;
	admissionState_e   admissionRequired;


	INT32 highMediumUsageThreshold;
	INT32 lowMediumUsageThreshold;
	UINT8 selectionHistoryCounter; /* When selected by Tx scheduler this counter is preset, and is decremented
									  every scheduler activation, so we have some indication on selections history. */

}MsduList_t;


MsduList_t* msduList_CreateNewMsduList(TI_HANDLE hOs );

TI_STATUS	msduList_ConfigMsduList( MsduList_t* this,	TI_HANDLE hMemMgr,
							 TI_HANDLE hReport, TI_HANDLE hOs,INT16 maxNumOfElements );

TI_STATUS	msduList_SetMsduListNumOfElements( MsduList_t* this, UINT16 maxNumOfElements);

TI_STATUS	msduList_SetMsduListOverFlowPolicy( MsduList_t* this, qOvFlowPolicy_e  QueueOvFlowPolicy);

TI_STATUS msduList_FreeMsduList( MsduList_t* this);

TI_STATUS msduList_EmptyMsduList( MsduList_t* this );

TI_STATUS msduList_Insert( MsduList_t* this , mem_MSDU_T  **pMsdu );

TI_STATUS msduList_WatchFirst( MsduList_t *this, mem_MSDU_T  **pMsdu );

TI_STATUS msduList_GetFirst( MsduList_t *this, mem_MSDU_T  **pMsdu );

UINT32 msduList_getCurrNumOfMsdu(MsduList_t *this);


/* Test Functions */
/*----------------*/
void printFullMsduList(MsduList_t *this);
void printMsduList(MsduList_t *this);


#endif /* _MSDU_LIST_H_ */
