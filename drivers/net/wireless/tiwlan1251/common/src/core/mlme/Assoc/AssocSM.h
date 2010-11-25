
/** \file AssocSM.h
 *  \brief 802.11 Association SM
 *
 *  \see assocSM.c
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
/*																		   */
/*		MODULE:	AssocSM.h												   */
/*    PURPOSE:	802.11 Association SM									   */
/*																	 	   */
/***************************************************************************/

#ifndef _ASSOC_SM_H
#define _ASSOC_SM_H

#include "fsm.h"
#include "mlmeApi.h"

/* Constants */

#define MAX_ASSOC_MSG_LENGTH			512

#define ASSOC_SM_CAP_ESS_MODE			0x0001
#define ASSOC_SM_CAP_IBSS_MODE			0x0002
#define ASSOC_SM_CAP_CF_POLLABLE		0x0004
#define ASSOC_SM_CAP_CF_POLL_REQ		0x0008
#define ASSOC_SM_CAP_PIVACY				0x0010
#define ASSOC_SM_CAP_SHORT_PREAMBLE		0x0020
#define ASSOC_SM_CAP_PBCC				0x0040
#define ASSOC_SM_CAP_CH_AGILITY			0x0080

/* Enumerations */

/* state machine states */
typedef enum 
{
	ASSOC_SM_STATE_IDLE		= 0,
	ASSOC_SM_STATE_WAIT    	= 1,
	ASSOC_SM_STATE_ASSOC	= 2
} assoc_smStates_t;

/* State machine inputs */
typedef enum 
{
	ASSOC_SM_EVENT_START		= 0,
	ASSOC_SM_EVENT_STOP			= 1,
	ASSOC_SM_EVENT_SUCCESS    	= 2,
    ASSOC_SM_EVENT_FAIL			= 3,
   	ASSOC_SM_EVENT_TIMEOUT		= 4,
   	ASSOC_SM_EVENT_MAX_RETRY	= 5
} assoc_smEvents_t;



/* Typedefs */

typedef struct
{
	fsm_stateMachine_t	        *pAssocSm;
	UINT32				        timeout;
	UINT8				        currentState;
	UINT32				        maxCount;
	UINT8				        retryCount;
	UINT32				        assocRejectCount;
	UINT32				        assocTimeoutCount;
	char				        *pChalange;
    UINT8                       assocRespBuffer[MAX_ASSOC_MSG_LENGTH];
    UINT32                      assocRespLen;
    UINT8                       assocReqBuffer[MAX_ASSOC_MSG_LENGTH];
    UINT32                      assocReqLen;

	BOOL						reAssoc;
	BOOL 						disAssoc; /* When set dissasociation frame will be sent. */

	TI_HANDLE			        timer;
	TI_HANDLE			        hMlme;
	TI_HANDLE			        hSiteMgr;
	TI_HANDLE			        hCtrlData;
	TI_HANDLE			        hTxData;
	TI_HANDLE			        hHalCtrl;
	TI_HANDLE			        hRsn;
	TI_HANDLE			        hReport;
	TI_HANDLE			        hOs;
	TI_HANDLE					hRegulatoryDomain;
	TI_HANDLE			        hExcMngr;
	TI_HANDLE			        hQosMngr;
    TI_HANDLE			        hMeasurementMgr;
	TI_HANDLE			        hApConn;
} assoc_t;

/* Structures */

/* External data definitions */

/* External functions definitions */

/* Function prototypes */

TI_STATUS assoc_start(TI_HANDLE hAssoc);

TI_STATUS reassoc_start(TI_HANDLE hAssoc);

TI_STATUS assoc_stop(TI_HANDLE hAssoc);

TI_STATUS assoc_recv(TI_HANDLE hAssoc, mlmeFrameInfo_t *pFrame);

TI_STATUS assoc_setDisAssocFlag(TI_HANDLE hAssoc, BOOL disAsoccFlag);

TI_STATUS assoc_smCapBuild(assoc_t *pCtx, UINT16 *cap);


/* local functions */

TI_STATUS assoc_saveAssocRespMessage(assoc_t *pAssocSm, UINT8 *pAssocBuffer, UINT32 length);


#endif

