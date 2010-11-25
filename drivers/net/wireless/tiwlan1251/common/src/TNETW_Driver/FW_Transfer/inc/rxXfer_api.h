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
 *   MODULE:  rxXfer_api.h
 *
 *   PURPOSE: Rx Xfer module API.
 * 
 ****************************************************************************/

#ifndef _RX_XFER_API_H
#define _RX_XFER_API_H


#include "osTIType.h"
#include "commonTypes.h"
#include "public_infoele.h"

/* Public Function Definitions */


TI_HANDLE			rxXfer_Create(TI_HANDLE hOs);

void				rxXfer_Destroy(TI_HANDLE hRxXfer);

void				rxXfer_Config( TI_HANDLE hRxXfer,
								   TI_HANDLE hFwEvent, 
								   TI_HANDLE hMemMgr,
								   TI_HANDLE hReport,
								   TI_HANDLE hTNETWIF);

TI_STATUS			rxXfer_RxEvent(TI_HANDLE hRxXfer);

void				rxXfer_Register_CB(TI_HANDLE hRxXfer,tiUINT32 CallBackID,void *CBFunc,TI_HANDLE CBObj);

void				rxXfer_SetDoubleBufferAddr(TI_HANDLE hRxXfer, ACXDataPathParamsResp_t *pDataPathParams);

#ifdef TI_DBG

void rxXfer_ClearStats( TI_HANDLE hRxXfer );

void rxXfer_PrintStats( TI_HANDLE hRxXfer );


#endif /* TI_DBG */
VOID RxXfer_ReStart(TI_HANDLE hRxXfer);


#endif /* _RX_XFER_API_H */




