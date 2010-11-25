/** \file	smeSm.h
 *  \brief SME SM internal header file
 *
 *  \see smeSm.c
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
/*	  MODULE:	smeSm.h														*/
/*    PURPOSE:	SME SM internal header file									*/
/*																			*/
/***************************************************************************/
#ifndef __SME_SM_H__
#define __SME_SM_H__

#include "osTIType.h"
#include "paramOut.h"
#include "paramIn.h"
#include "fsm.h"
#include "802_11Defs.h"
#include "ScanCncnApi.h"

#define IBSS_INTER_SCAN_PERIOD			20
#define SCAN_ATTAMEPTS_HISTOGRAM_SIZE	8
#define SCAN_RESULT_HISTOGRAM_SIZE		16

/* SME SM definitions */
typedef enum
{
	SME_SM_STATE_IDLE 				     = 0,
	SME_SM_STATE_SCANNING			     = 1,
	SME_SM_STATE_SELECTING			     = 2,
	SME_SM_STATE_CONNECTING 		     = 3,
	SME_SM_STATE_CONNECTED 		  	     = 4,
	SME_SM_STATE_DISCONNECTING		     = 5,
	SME_SM_STATE_INTER_SCAN		         = 6,
	SME_SM_NUM_STATES                    = 7
} sme_sm_states_e;


typedef enum 
{
	SME_SM_EVENT_START						= 0,
	SME_SM_EVENT_STOP						= 1,
	SME_SM_EVENT_SCAN_COMPLETE			    = 2,
	SME_SM_EVENT_SELECT_SUCCESS				= 3,
	SME_SM_EVENT_SELECT_FAILURE				= 4,
	SME_SM_EVENT_CONN_SUCCESS				= 5,
	SME_SM_EVENT_CONN_FAILURE				= 6,
	SME_SM_EVENT_RESELECT					= 7,
	SME_SM_EVENT_DISCONNECT					= 8,
	SME_SM_NUM_EVENTS                       = 9
} sme_sm_events_e;


typedef struct
{
	mgmtStatus_e			mgmtStatus;		/* Contains the last DisAssociation reason towards upper layer					*/
	UINT32					uStatusCode;	/* Extra information to the reason. i.e. packet status code or roaming trigger  */
} DisAssocReason_t;

typedef struct
{
	BOOL				RadioStandByEnable;
	radioStandByState_t	lastRadioStandByState; 
	UINT16				radioStandByBitNumber;
	BOOL				GPIO13_debug;
} sme_radioDisableParameters_t;

#ifdef TI_DBG
typedef struct
{
	UINT8	currentNumberOfScanResults;
	UINT32	scanResulCountHistogram[ SCAN_RESULT_HISTOGRAM_SIZE ];
	UINT8	currentNumberOfScanAttempts;
	UINT32	scanAttemptsForConnectionHistogram[ SCAN_ATTAMEPTS_HISTOGRAM_SIZE ];
} smeSmStats_t;
#endif

/* SME SM handle */
typedef struct 
{
	UINT8				state;	
	DisAssocReason_t	DisAssoc;
	fsm_stateMachine_t	*pFsm;
	scanEnabledOptions_e scanEnabled;
	BOOL				reScanFlag;
	BOOL				dualBandReScanFlag;
	radioBand_e     	currBand;
	BOOL				bSendDisAssocEvent;
	TI_HANDLE			hConn;
	TI_HANDLE			hScanCncn;
	TI_HANDLE			hSiteMgr;
	TI_HANDLE			hHalCtrl;
	TI_HANDLE			hRegulatoryDomain;
    TI_HANDLE           hPowerMgr;
	TI_HANDLE			hReport;
	TI_HANDLE			hOs;
	TI_HANDLE			hEvHandler;
	TI_HANDLE			hScr;
	TI_HANDLE			hApConn;
	TI_HANDLE			hCurrBss;

	void				*interScanTimeoutTimer;
	UINT32				interScanTimeout;
	UINT32				interScanTimeoutMin;
	UINT32				interScanTimeoutMax;
	UINT32				interScanTimeoutDelta;
    sme_scan_Params_t   scanParamsBG;
    sme_scan_Params_t   scanParamsA;
	BOOL 				radioOn;
    BOOL                immediateShutdownRequired; /* Flag to indicate whether DISASSOC packet should be sent to AP or not - used to instruct AP-Conn when stopping */
	BOOL 				connectEnabled;
	scan_Params_t 		scanParams; /* 	Though this is a temporary command, we keep it as 
										class member to avoid stack overflow. */
    UINT8               shutDownStatus; /* Internal shutdown status - used to indicate SME move to IDLE during unload process */
#ifdef TI_DBG
	smeSmStats_t		smeStats;
#endif
} smeSm_t;


fsm_stateMachine_t *smeSm_smCreate(TI_HANDLE hOs);

TI_STATUS smeSm_smConfig(smeSm_t *pSmeSm);

TI_STATUS smeSm_smUnLoad(TI_HANDLE hOs, fsm_stateMachine_t *pFsm);

TI_STATUS smeSm_SMEvent(UINT8 *currentState, UINT8 event, TI_HANDLE hSmeSm);

TI_STATUS sme_startScan(void *pData);

#endif /* __SME_SM_H__ */


