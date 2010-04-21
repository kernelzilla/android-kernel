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
 *   MODULE:  whalBus_Api.h
 *   PURPOSE: Bus access component API
 *
 ****************************************************************************/

#ifndef _WHAL_BUS_API_H
#define _WHAL_BUS_API_H

#include "whalCtrl_api.h"
#include "whalCommon.h"
#include "whalHwDefs.h" 
#include "whalBus_Defs.h"
#include "memMngrEx.h"
#include "commonTypes.h"

/*
 * --------------------------------------------------------------
 *                  Creation/Configuration API
 * --------------------------------------------------------------
 */                  
TI_HANDLE   whalBus_Create  (TI_HANDLE hOs);
int         whalBus_Destroy (TI_HANDLE hWhalBus);
TI_STATUS   whalBus_Config  (TI_HANDLE hWhalBus, 
                             TI_HANDLE hWhalCtrl,
                             UINT8     AccessMode, 
                             UINT32    RegBaseAddr,
                             UINT32    MemBaseAddr,
                             TI_HANDLE hReport, 
                             TI_HANDLE hMemMgr,
                             fnotify_t fCb,
                             TI_HANDLE hCb);
TI_HANDLE   whalBus_GetTnentwifHandle (TI_HANDLE hWhalBus);
int         whalBus_ExitFromInitMode  (TI_HANDLE hWhalBus);
int         whalBus_ReConfig          (TI_HANDLE hWhalBus);

/*
 * --------------------------------------------------------------
 *                  Registers/Memory access API
 * --------------------------------------------------------------
 */                  
UINT32  whalBus_MacRegRead       (TI_HANDLE hWhalBus, UINT32 RegAddr);
void    whalBus_MacRegWrite      (TI_HANDLE hWhalBus, UINT32 RegAddr, UINT32 Val);
void    whalBus_PhyRegWrite      (TI_HANDLE hWhalBus, UINT32 PhyRegAddr, UINT32 DataVal);
UINT32  whalBus_PhyRegRead       (TI_HANDLE hWhalBus, UINT32 PhyRegAddr);
void    whalBus_MemWrite         (TI_HANDLE hWhalBus, UINT32 Addr, char *data, int Len);
void    whalBus_MemRead          (TI_HANDLE hWhalBus, UINT32 Addr, char *data, int Len);

/*
 * --------------------------------------------------------------
 *                  Firmware control API
 * --------------------------------------------------------------
 */                  
typedef struct
{
    UINT32  radioType;
    UINT8   MacClock;
    UINT8   ArmClock;
    BOOL    FirmwareDebug;
    UINT32  minorE2Ver;
    UINT32  majorE2Ver;
    UINT32  bugfixE2Ver;
} BootAttr_T;


TI_STATUS  whalBus_FwCtrl_Boot     (TI_HANDLE hWhalBus, TI_HANDLE hHwCtrl, BootAttr_T *pBootAttr);
int        whalBus_FwCtrl_Reset    (TI_HANDLE hWhalBus);
int        whalBus_FwCtrl_isCardIn (TI_HANDLE hWhalBus);
void       whalBus_FwCtrl_Halt     (TI_HANDLE hWhalBus);
UINT32     whalBus_FwCtrl_GetRadioStandByState
                                   (TI_HANDLE hWhalBus);

/*
 * --------------------------------------------------------------
 *                  Interrupt handler API
 * --------------------------------------------------------------
 */                  

/*  states */
typedef enum
{
    STATE_INIT          = 0,
    STATE_DPC              ,    
    STATE_OPERATIONAL      ,
    STATE_WAIT_FOR_DPC      ,
}whalHwIntr_states;

/* Callback template */
typedef void (*whal_hwIntrCB)(void *pObj);

/*Power Ctrl Callback template */
#define PWRCTRL_CB_TYPE_INTR_WAS_ISSUE  0
#define PWRCTRL_CB_TYPE_INTR_STARTED    1
typedef void (*whal_PwrCtrlCB)(TI_HANDLE pObj, int CbType);


void whalBus_TNETWIF_HandleBusTxn_Complete  (TI_HANDLE hWhalBus);

/*
 * --------------------------------------------------------------
 *                  TNETWIF API
 * --------------------------------------------------------------
 */
int whalBus_TNETWIF_ElpCtrl_SetMode(TI_HANDLE hWhalBus, elpCtrl_Mode_e mode);

/*
 * --------------------------------------------------------------
 *                  Debug API
 * -------------------------------------------------------------- 
 */                  
void    whalBus_performHealthMonitorTest(TI_HANDLE hWhalBus, UINT32 TEST);
/*
 * General
 */

#define BUS_PRINT_ALL           1

/* 
 * Control
 */
#define BUS_PRINT_MBOX_INFO     10
#define BUS_PRINT_INTR_INFO     11
#define BUS_PRINT_ACCESS_INFO   12
#define BUS_PRINT_EEPROM        13
#define BUS_PRINT_TRACE_A_B     14
#define BUS_PRINT_ARBITER       15

/*
 * Tx
 */
#define BUS_PRINT_TX_INFO       20
#define BUS_PRINT_TX            21
#define BUS_PRINT_TX_QUEUE      22

/*
 * Rx
 */
#define BUS_PRINT_RX_INFO       30
#define BUS_CLEAR_RX_INFO       31

/*
 * Debug
 */
#define BUS_PRINT_REG_DUMP      40
#define BUS_PRINT_RX_REGS       41
#define BUS_PRINT_TX_REGS       42
#define BUS_PRINT_SCR_PAD_REGS  43
#define BUS_PRINT_LIST_REGS     44
#define BUS_PRINT_MEM           45

void    whalBus_PrintInfo(TI_HANDLE hWhalBus, UINT32 funcType, void *pParam);
/***************************************************************************************************
************************************* New Hal ******************************************************
***************************************************************************************************/

#endif /* _WHAL_BUS_API_H */
