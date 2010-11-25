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
/*	  MODULE:	recoveryMgr.h										       */
/*    PURPOSE:	RecoveryMgr module Header file                             */
/*																		   */
/***************************************************************************/
#ifndef _RECOVERY_CTRL_H_
#define _RECOVERY_CTRL_H_

#include "osTIType.h"


/* State-Machine States */
typedef enum
{
	REC_CTRL_STATE_IDLE,				/*  */
	REC_CTRL_STATE_WAIT_END_CURR_TXN,	/*  */
	REC_CTRL_STATE_INIT_CMPLT,			/*  */
	REC_CTRL_STATE_END_RECONFIG
} recoveryCtrlSmState_e;

/* Callback function definition for EndOfRecovery */
typedef void (* EndOfRecoveryCB_t)(TI_HANDLE CBObj);

/* The module object. */
typedef struct 
{
	/* Handles */
	TI_HANDLE hOs;
	TI_HANDLE hReport;
	TI_HANDLE hTNETWIF;
	TI_HANDLE hTxXfer;
	TI_HANDLE hRxXfer;
	TI_HANDLE hTxResult;
	TI_HANDLE hMacServices;
	TI_HANDLE hTxCtrlBlk;
	TI_HANDLE hTxHwQueue;
	TI_HANDLE hHalCtrl;
	TI_HANDLE hHalRx;	
	TI_HANDLE hHwIntr;
	TI_HANDLE hWhalParams;		
	TI_HANDLE hCmdQueue;
	TI_HANDLE hFwEvent;
	TI_HANDLE hCmdMBox;
	TI_HANDLE hHwInit;

	TI_HANDLE hTNETWArb;//hBusArbiter;
	TI_HANDLE hBusTxn;
	TI_HANDLE hELPCtrl;

	TI_HANDLE hScanSRV;
	TI_HANDLE hMeasurementSRV;
	TI_HANDLE hPowerSrv;
	TI_HANDLE hPowerAutho;

	TI_HANDLE hRecoveryMgr;
	EndOfRecoveryCB_t endOfRecoveryCB;		/* EndOfRecovery callback */
		
	recoveryCtrlSmState_e smState;			/* The current state of the SM. */	

} recoveryCtrl_t;


#endif /* _RECOVERY_CTRL_H_ */
