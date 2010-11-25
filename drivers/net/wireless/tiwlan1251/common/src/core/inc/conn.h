/** \file conn.h
 *  \brief connection module internal header file
 *
 *  \see conn.c
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
/*	  MODULE:	conn.h														*/
/*    PURPOSE:	connection module internal header file						*/
/*																			*/
/***************************************************************************/
#ifndef __CONN_H__
#define __CONN_H__

#include "osTIType.h"
#include "paramOut.h"
#include "paramIn.h"
#include "fsm.h"
#include "802_11Defs.h"
#include "connApi.h"

typedef struct
{
	mgmtStatus_e			disAssocEventReason;		/* Disassoc reason to be used for upper layer	*/
	UINT32					disAssocEventStatusCode;	/* Extra information for disConnEventReason		*/
} connSmContext_t;


/*
 *	ibssRandomSsidGen - in IBSS, that flag indicates if the ibss must be randommaly changed. 
 *	FALSE - do not generate random ibss.
 *	TRUE -	generate random ibss.
 *	1.	at Start up that flag is FALSE
 *  2.	after Connection fail in connIbss or connSelf SM, set the flag to TRUE in order to 
 *		generate a new random ibss.
 */

/* Connection handle */
typedef struct 
{
	UINT8					state;	
	connSmContext_t			smContext;
	connectionType_e		currentConnType;
	UINT32					timeout;
	void					*pTimer; /* This timer is used both by IBSS and BSS */
	fsm_stateMachine_t		*ibss_pFsm;
    fsm_stateMachine_t		*infra_pFsm;
	connType_e				connType;
	disConnType_e 			disConnType;
	mgmtStatus_e			disConnReasonToAP;			/* Status code for Deauth packet to AP */
	BOOL					disConEraseKeys;

	conn_status_callback_t  pConnStatusCB;
	TI_HANDLE				connStatCbObj;

	BOOL 					scrRequested; 
    UINT32                  ibssDisconnectCount;

	TI_HANDLE				hSiteMgr;
	TI_HANDLE				hSmeSm;
	TI_HANDLE				hMlmeSm;
	TI_HANDLE				hRsn;
	TI_HANDLE				hReport;
	TI_HANDLE				hOs;
	TI_HANDLE				hRxData;
	TI_HANDLE				hTxData;
	TI_HANDLE 				hPwrMngr;
	TI_HANDLE				hCtrlData;
	TI_HANDLE 				hQosMngr;
	TI_HANDLE				hHalCtrl;
	TI_HANDLE				hMeasurementMgr;
	TI_HANDLE				hScr;
	TI_HANDLE				hTrafficMonitor;
	TI_HANDLE				hExcMngr;
	TI_HANDLE				hScanCnc;
	TI_HANDLE				hCurrBss;
	TI_HANDLE				hSwitchChannel;
	TI_HANDLE				hEvHandler;
	TI_HANDLE				hHealthMonitor;
	TI_HANDLE				hMacServices;
    TI_HANDLE               hRegulatoryDomain;
	TI_HANDLE    			hSoftGemini;
} conn_t;

#endif /* __CONN_H__*/
