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

#ifndef _EVHANDLER_H_
#define _EVHANDLER_H_


#include "TI_IPC_Api.h"


typedef struct 
{
    IPC_EV_DATA Array[MAX_SEND_EVENTS];
    UINT32       TailIndex; /* Points to the next free node*/
    UINT32       HeadIndex; /* Points to the first occupied node*/
    UINT32       Counter;
}EV_CYCL_ARRAY, *PEV_CYCL_ARRAY;

typedef struct _EV_HANDLER_STRUCT_T {
	
    TI_HANDLE		   hOs;
    IPC_EVENT_PARAMS   RegistrationArray[IPC_EVENT_MAX][MAX_REGISTERED_MODULES];
    EV_CYCL_ARRAY      SendEventArray;
    UINT32             LastUMEventType;

}EV_HANDLER_STRUCT_T, *PEV_HANDLER_STRUCT_T;

/* Upper Interface*/
TI_HANDLE EvHandlerInit         (TI_HANDLE hOs);

UINT32 EvHandlerUnload          (TI_HANDLE hEvHandler);

UINT32 EvHandlerRegisterEvent   (TI_HANDLE hEvHandler, PUCHAR pData,   ULONG Length);

UINT32 EvHandlerUnRegisterEvent (TI_HANDLE hEvHandler, TI_HANDLE uEventID);

UINT32 EvHandlerMaskEvent       (TI_HANDLE hEvHandler, UINT32 uEventID);

UINT32 EvHandlerUnMaskEvent     (TI_HANDLE hEvHandler, UINT32 uEventID);

UINT32 EvHandlerGetEventData    (TI_HANDLE hEvHandler, PUCHAR pData, ULONG* pLength);

/* Bottom Interface*/

UINT32 EvHandlerSendEvent       (TI_HANDLE hEvHandler, UINT32 EvType, UINT8* pData, UINT32 Length);


#endif
