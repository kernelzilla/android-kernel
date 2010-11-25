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

#ifndef _WHAL_CTRL_H
#define _WHAL_CTRL_H

#include "whalCommon.h"
#include "whalParams.h"
#include "whalHwCtrl.h"

#include "rxXfer_api.h"


/* CLASS WHAL_CTRL*/
typedef struct _WHAL_CTRL
{
    WhalParams_T*  pWhalParams;

    ReadWriteCommand_t printRegsBuf;    
    
    scanCompleteCB_t ScanCmplt_CB;
    void*            ScanCmplt_CB_handle;

    MacStatusCB_t    MacStatusCB_CB;
    void*            MacStatusCB_CB_handle;

    HealthReportCB_t HealthReportCB_CB;
    void*            HealthReportCB_handle;

    AciIndicationCB_t AciIndicationCB_CB;
    void*             AciIndicationCB_handle;

    failureEventCB_t FailureEvent_CB;
    void*            FailureEvent_CB_handle;
    

    BOOL EncDecEnableFlag; /* flag to designates whether the Hw encrypt and decrypt
                              facility on WEP options IE already enabled*/

    TI_HANDLE hOs;
    TI_HANDLE hReport;
    TI_HANDLE hTNETW_Driver;

    TI_HANDLE hFwEvent;
    TI_HANDLE hTxXfer;
    TI_HANDLE hTxResult;
    TI_HANDLE hTxHwQueue;
    TI_HANDLE hRxXfer;
    TI_HANDLE hEventMbox;
    struct _WHAL_SECURITY* pWhalSecurity;
        
    struct _HwCtrl_T *pHwCtrl; /* Pointer to the LL_HAL acxCtrl module*/
    TI_HANDLE   hWhalBus; /* Pointer to the LL_HAL acxCtrl module*/
    TI_HANDLE   hCmdQueue;

    /* Init stage callback function pointer and handle */ 
    void     *fCb;
    TI_HANDLE hCb;
    
    ACXMisc_t misc;

#ifdef TI_DBG
    TI_HANDLE hDebugTrace;
#endif /* TI_DBG */

} WHAL_CTRL;


WhalParams_T *whalCtrl_GetWhalParamsHandle(WHAL_CTRL *pWhalCtrl);
void whalCtrl_MacStatus_CB(TI_HANDLE hWhalCtrl, char* MacStatus , UINT32 strLen);
void whalCtrl_HealthReoprt_CB(TI_HANDLE hWhalCtrl, char* MacStatus , UINT32 strLen);
void whalCtrl_registerDump(TI_HANDLE hWhalCtrl);
int  whalCtrl_RegisterCmdCompleteGenericCB(TI_HANDLE hWhalCtrl, void *CbFunc ,void *CbObj);
int  whalCtrl_RegisterErrorsCallbacks(TI_HANDLE hWhalCtrl);
int  whalCtrl_ReJoinBss (TI_HANDLE hWhalCtrl);
TI_STATUS whalCtrl_getRadioNumber(TI_HANDLE hWhalCtrl, UINT32 *outRadioType, UINT32 *outRadioNumber);
TI_STATUS whalCtrl_FinalizeDownload(TI_HANDLE hWhalCtrl);
/* Used for Memory or Registers reading/writing*/
typedef enum
{
   TNETW_INTERNAL_RAM = 0,
   TNETW_MAC_REGISTERS = 1,
   TNETW_PHY_REGISTERS = 2,
} readWrite_MemoryType_e;

void    whalCtrl_Print_Mem_Regs (TI_HANDLE hWhalCtrl, UINT32 addr, UINT32 len, readWrite_MemoryType_e memType);
int     whalCtrl_Set_Mem_Regs (TI_HANDLE hWhalCtrl, UINT32 address, UINT32 len, UINT32 aWriteVal, readWrite_MemoryType_e memType);
#endif /* _WHAL_CTRL_H*/
