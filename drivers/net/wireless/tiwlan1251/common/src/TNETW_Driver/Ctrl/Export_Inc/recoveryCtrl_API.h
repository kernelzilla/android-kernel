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
/*	  MODULE:	recoveryCtrl.h										       */
/*    PURPOSE:	recoveryCtrl module Header file                             */
/*																		   */
/***************************************************************************/
#ifndef _RECOVERY_CTRL_API_H_
#define _RECOVERY_CTRL_API_H_

#include "osTIType.h"

TI_HANDLE recoveryCtrl_create(TI_HANDLE hOs);
TI_STATUS recoveryCtrl_config(TI_HANDLE hRecoveryCtrl, 
							  TI_HANDLE hReport,
							  TI_HANDLE hTNETWIF,
							  TI_HANDLE hTxXfer,     
							  TI_HANDLE hRxXfer,     
							  TI_HANDLE hTxResult,   
							  TI_HANDLE hMacServices,
							  TI_HANDLE hTxCtrlBlk,  
							  TI_HANDLE hTxHwQueue,  
							  TI_HANDLE hHalCtrl,    
							  TI_HANDLE hHwIntr,     
							  TI_HANDLE hWhalParams, 
							  TI_HANDLE hCmdQueue,   
							  TI_HANDLE hFwEvent,    
							  TI_HANDLE hCmdMBox,
							  TI_HANDLE hHwInit);

TI_STATUS recoveryCtrl_destroy(TI_HANDLE hRecoverCtrl);
TI_STATUS recoveryCtrl_restartTWD(TI_HANDLE hRecoverCtrl,void *endOfRecoveryCB, TI_HANDLE hRecoveryMgr);
TI_STATUS recoveryCtrl_CB(TI_HANDLE hRecoveryCtrl);



#endif /* _RECOVERY_CTRL_API_H_ */
