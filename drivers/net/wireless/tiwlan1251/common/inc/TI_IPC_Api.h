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

#ifndef _TI_IPC_API_H
#define _TI_IPC_API_H

#include "osTIType.h"
#include "TI_Results.h"


#define MAX_REGISTERED_MODULES 5
#define MAX_EVENT_DATA_SIZE 2048
#define MAX_SEND_EVENTS 4

#ifdef _WINDOWS
#endif 

#ifdef  __cplusplus
extern "C" {
#endif

/*******************Defines*********************/

/* WARNING! DON'T CHANGE THE ORDER OF EVENTS! */
/* OS EVENTS MUST COME FIRST!*/

enum
{
    IPC_EVENT_ASSOCIATED = 0,
    IPC_EVENT_DISASSOCIATED,
    IPC_EVENT_LINK_SPEED,
    IPC_EVENT_AUTH_SUCC,
    IPC_EVENT_SCAN_COMPLETE,
    IPC_EVENT_TIMEOUT,
    IPC_EVENT_CCKM_START,
    IPC_EVENT_MEDIA_SPECIFIC,
    IPC_EVENT_MAX_OS_EVENT = IPC_EVENT_MEDIA_SPECIFIC,
    IPC_EVENT_EAPOL,
    IPC_EVENT_BOUND,
    IPC_EVENT_UNBOUND,
    IPC_EVENT_PREAUTH_EAPOL,
    IPC_EVENT_RESERVED2,
    IPC_EVENT_LOW_SNR,
    IPC_EVENT_LOW_RSSI,
    IPC_EVENT_TSPEC_STATUS,
    IPC_EVENT_TSPEC_RATE_STATUS,
    IPC_EVENT_MEDIUM_TIME_CROSS,
    IPC_EVENT_ROAMING_COMPLETE,
    IPC_EVENT_EAP_AUTH_FAILURE,
    IPC_EVENT_WPA2_PREAUTHENTICATION,
    IPC_EVENT_TRAFFIC_INTENSITY_THRESHOLD_CROSSED,
    IPC_EVENT_GWSI,
    IPC_EVENT_BT_COEX_MODE,
    IPC_EVENT_MAX
};

enum
{
    DELIVERY_PUSH =0,
    DELIVERY_GET_DATA
};

/************************* IOCTLs Functions *******************************/

TI_HANDLE   IPC_Init(void);

tiINT32     IPC_DeInit(void);

TI_HANDLE   IPC_DeviceOpen(tiVOID* AdapterName); /* get hDevice Handle*/

tiINT32     IPC_DeviceClose(TI_HANDLE hDevice);

tiINT32     IPC_DeviceIoControl(TI_HANDLE   hDevice,
                            tiUINT32    IoControlCode, 
                            tiVOID*     pInBuffer,
                            tiUINT32    InBufferSize,
                            tiVOID*     pOutBuffer,
                            tiUINT32    pOutBufferSize,
                            tiUINT32*   pBytesReturned);

/************************* Events Functions *******************************/

typedef struct _IPC_EV_DATA * PIPC_EV_DATA;  

typedef tiINT32 (*TI_EVENT_CALLBACK) (PIPC_EV_DATA  pData);

typedef struct _IPC_EVENT_PARAMS
{
    tiUINT32            uEventType;
    TI_HANDLE           uEventID;
    tiUINT32            uProcessID;
    tiUINT32            uDeliveryType;
    TI_HANDLE           hUserParam;            /* Handle to back reference*/
    TI_EVENT_CALLBACK   pfEventCallback;
}IPC_EVENT_PARAMS;

/* EvParams are assumed to be the first field. Any addtions shoild be made 
    afterwards
 */
typedef struct _IPC_EV_DATA
{
    IPC_EVENT_PARAMS    EvParams;
    tiUINT32            uBufferSize;
    tiUINT8             uBuffer[MAX_EVENT_DATA_SIZE];
}IPC_EV_DATA;


/*this function will also enable event and pass all the parameters about it*/
/* returns unique ID of registered event, to be passed later for unregister*/
tiINT32 IPC_RegisterEvent(TI_HANDLE             hDevice,    /* Driver Handle*/
                          IPC_EVENT_PARAMS*     pEvParams);  /* size of the structure + size of the params*/

tiINT32 IPC_UnRegisterEvent(TI_HANDLE   hDevice,
                            IPC_EVENT_PARAMS*   pEvParams); /* returned by IPC_RegisterEvent*/

/***************************************************************************/

#ifdef  __cplusplus
}
#endif

#endif /*_IPC_UTIL_H*/

