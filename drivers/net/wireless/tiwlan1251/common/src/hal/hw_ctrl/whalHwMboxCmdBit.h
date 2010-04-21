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
 *   MODULE:  whalHwMboxCmdBit.h
 *   PURPOSE: wlan hardware BIT(Built-In Test) commands header file
 * 
 ****************************************************************************/

#ifndef _WHAL_HW_MBOX_CMD_BIT_H
#define _WHAL_HW_MBOX_CMD_BIT_H

#include "whalCommon.h"
#include "CmdQueue_api.h"

/* Function prototypes */

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    void*                   CB_Func;              /* Pointer to callback function to be called with the results, if working in Async mode (GWSI).*/
    TI_HANDLE               CB_Handle;
    PLT_RxPerCmd_e          CB_RxPerCmd;          /* Enumerator of the invoked PLT command*/
    PltRxPer_t              PltRxPer;             /* The accumulated FCS error and total packets counts */ 
    ACXErrorCounters_t      ACXErrCountTable;     /* Last F.W. counters results */
} HwMboxCmdBit_RxPer_t;

typedef struct
{
    void*                   CB_Func;              /* Pointer to callback function to be called with the results, if working in Async mode (GWSI).*/
    TI_HANDLE               CB_Handle;
    TI_STATUS              lastStatus;
    
} HwMboxCmdBit_RxTxCal_t;

typedef union
{
    HwMboxCmdBit_RxPer_t   RxPer;
    HwMboxCmdBit_RxTxCal_t RxTxCal;
}HwMboxCmdBit_u;

typedef struct _HwMboxCmdBit_T
{
    HwMboxCmdBit_u           PltData;
    TI_HANDLE                hCmdQueue;
    TI_HANDLE                hOs;
    TI_HANDLE                hReport;
    TI_HANDLE                hWhalCtr;
} HwMboxCmdBit_T;


extern HwMboxCmdBit_T *whal_hwMboxCmdBit_Create(TI_HANDLE hOs);
extern int whal_hwMboxCmdBit_Destroy(HwMboxCmdBit_T *this);
extern int whal_hwMboxCmdBit_Config(TI_HANDLE hWhalCtr, HwMboxCmdBit_T *pHwMboxCmdBit, TI_HANDLE hCmdQueue, TI_HANDLE hReport);

int whal_hwCmdBit_ReadRegister(HwMboxCmdBit_T *pHwMboxCmdBit, TI_HANDLE CB_Handle, void *CB_Func, void *CB_Buf);
int whal_hwCmdBit_WriteRegister(HwMboxCmdBit_T *pHwMboxCmdBit, TI_HANDLE CB_Handle, void *CB_Func, void *Command_Buf);

int whal_hwCmdBit_RxPER(HwMboxCmdBit_T *pHwMboxCmdBit, PLT_RxPerCmd_e eRxPerCmd, TI_HANDLE CB_Handle, void *CB_Func);

int whal_hwCmdBit_TestCmd(HwMboxCmdBit_T *pHwMboxCmdBit, void *CB_Func, TI_HANDLE CB_handle, TestCmd_t* pTestCmd_Buf);

void whal_hwCmdBit_GetPltRxCalibrationStatus( TI_HANDLE objectHandle, TI_STATUS* pLastStatus  );

int whal_hwCmdBit_Fcc(HwMboxCmdBit_T *pHwMboxCmdBit,
                      int chID, int rate, int preamble,int bandID,
                      int InterPacketDelay, int TestMode, uint32 numFrames,
                      uint32 seqNumMode, uint32 frameBodySize, uint8 *PeerMacAddr,
                      void *CB_Func, TI_HANDLE CB_handle, void *CB_Buf);

int whal_hwCmdBit_Telec(HwMboxCmdBit_T *pHwMboxCmdBit, int chID, int bandID, void *CB_Func, TI_HANDLE CB_handle, void *CB_Buf);

int whal_hwCmdBit_perTxStop(HwMboxCmdBit_T *pHwMboxCmdBit, void *CB_Func, TI_HANDLE CB_handle, void *CB_Buf);

#ifdef __cplusplus
}
#endif

#endif   /* _WHAL_HW_MBOX_CMD_BIT_H */

