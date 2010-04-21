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
 *   MODULE:  FwEvent_api.h
 *
 *   PURPOSE: Firmware Event module API.
 * 
 ****************************************************************************/

#ifndef _FW_EVENT_API_H
#define _FW_EVENT_API_H


#include "osTIType.h"
#include "commonTypes.h"
#include "public_infoele.h"

/* Public Function Definitions */

TI_HANDLE       FwEvent_Create(TI_HANDLE hOs);
TI_STATUS       FwEvent_Destroy(TI_HANDLE hFwEvent);
void            FwEvent_BusReadyCB(TI_HANDLE hFwEvent,UINT8 module_id, TI_STATUS status);
void            FwEvent_EventComplete(TI_HANDLE hFwEvent, systemStatus_e rc);
TI_STATUS       FwEvent(TI_HANDLE hFwEvent);
void            FwEvent_Config(TI_HANDLE hFwEvent, TI_HANDLE hTnetwDrv);
void            FwEvent_Enable(TI_HANDLE hFwEvent, UINT32 uEventMask);
void            FwEvent_Disable(TI_HANDLE hFwEvent, UINT32 uEventMask);
UINT32          FwEvent_GetEnabled(TI_HANDLE hFwEvent);
void			FwEvent_SetHwInfo(TI_HANDLE hFwEvent, ACXDataPathParamsResp_t *pDataPathParams);

/****************************************************************
*
*   Functions originally located at whalHwIntr.c 
*
*****************************************************************/
void            FwEvent_EnableInterrupts (TI_HANDLE hFwEvent);
void            FwEvent_DisableInterrupts(TI_HANDLE hFwEvent); 
UINT32          FwEvent_CheckInterrupts  (TI_HANDLE hFwEvent);
void            FwEvent_ChangeState      (TI_HANDLE hFwEvent, int State);
void            FwEvent_StateChanged     (TI_HANDLE hFwEvent);
void            FwEvent_Stop             (TI_HANDLE hFwEvent);

#endif /* _FW_EVENT_API_H */







