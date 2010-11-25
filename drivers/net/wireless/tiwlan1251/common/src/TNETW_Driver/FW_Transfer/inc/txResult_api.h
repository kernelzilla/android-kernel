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

/****************************************************************************
 *
 *   MODULE:  txResult_api.h
 *
 *   PURPOSE: Tx result module API.
 * 
 ****************************************************************************/

#ifndef _TX_RESULT_API_H
#define _TX_RESULT_API_H


#include "public_infoele.h"


#define FW_TX_CMPLT_BLOCK_SIZE		16	/* Number of rows in the FW Tx-Result table */


/* Public Function Definitions */

TI_HANDLE	txResult_Create(TI_HANDLE hOs);
TI_STATUS	txResult_Destroy(TI_HANDLE hTxResult);
TI_STATUS	txResult_init(TI_HANDLE hTxResult, TI_HANDLE hReport, TI_HANDLE hTNETWIF, TI_HANDLE hFwEvent);
TI_STATUS	txResult_restart(TI_HANDLE hTxResult);
void		txResult_setHwInfo(TI_HANDLE hTxResult, ACXDataPathParamsResp_t *pDataPathParams);
void		txResult_RegisterCB(TI_HANDLE hTxResult, tiUINT32 CallBackID, void *CBFunc, TI_HANDLE CBObj);
void		txResult_printInfo(TI_HANDLE hTxResult);
void		txResult_clearInfo(TI_HANDLE hTxResult);
TI_STATUS   txResult_TxCmpltIntrCB (TI_HANDLE hTxResult);


#endif /* _TX_RESULT_API_H */

