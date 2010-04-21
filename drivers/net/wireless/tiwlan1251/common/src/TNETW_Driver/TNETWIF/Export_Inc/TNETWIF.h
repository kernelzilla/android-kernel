/** \file GWSI_Synchronizer.h
 *  \brief GWSI Synchronizer include file
 *
 *  \see GWSI_Synchronizer.c 
 */
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
/*                                                                         */
/*    MODULE:   TNETWIF.h                                                  */
/*    PURPOSE:  TNETWIF api file                                            */
/*                                                                         */
/***************************************************************************/

#ifndef TNETWIF_H
#define TNETWIF_H

#include "commonTypes.h"
#include "TNETW_Driver_types.h"


/* Modules ID's */
enum 
{
    DEFAULT_MODULE_ID = 0,
    TX_XFER_MODULE_ID, 
    HAL_RX_MODULE_ID,  
    HAL_INT_MODULE_ID,  
    HAL_CMD_MODULE_ID,  
    FW_EVENT_MODULE_ID,
    HAL_INIT_MODULE_ID,

    NUM_OF_TNETWIF_MODULES,
    TNETWIF_LAST_MODULE_ID = NUM_OF_TNETWIF_MODULES - 1
};


typedef enum
{
    HW_ACCESS_DOWNLOAD,
    HW_ACCESS_WORKING,

} TNETIF_HwAccess_SetPartition_mode_e;


/* 
 * Defines a TNETWIF read/write field with padding.
 * A low level driver may use this padding internally
 */
#if defined(HW_ACCESS_WSPI)
#define PADDING(field) \
    UINT8 padding [TNETWIF_READ_OFFSET_BYTES]; \
    field;
#else
#define PADDING(field) field;
#endif
/*
 * A pointer to the read offset of the padded field
 */
#define PADREAD(field_ptr) \
   (((UINT8*)field_ptr) - TNETWIF_READ_OFFSET_BYTES)

/*
 * A pointer to the write offset of the padded field
 */
#define PADWRITE(field_ptr) \
   (((UINT8*)field_ptr) - TNETWIF_WRITE_OFFSET_BYTES)


typedef void (*TNETWIF_callback_t)(TI_HANDLE CB_Handle, UINT8 module_id, TI_STATUS status);

/**************************** TNETWIF_CB ********************************/
/****************************************************************************/
typedef struct T_TNETWIF_CB
{
    TI_HANDLE           hOs;
    TI_HANDLE           hReport;
    TI_HANDLE           hTNETWArb;
    TI_HANDLE           hHwAccess;
    TI_HANDLE           hELPCtrl;
    UINT32              uInitStage;
    UINT32              uRegBaseAddr;
    UINT32              uMemBaseAddr;
    TNETWIF_callback_t  fCb;
    TI_HANDLE           hCb;
    TI_STATUS           status;
} TNETWIF_t;


/* API for external module in charge of creating the gwsi_synchronizer initialization */
TI_HANDLE TNETWIF_Create              (TI_HANDLE hOs);
TI_STATUS TNETWIF_Config              (TI_HANDLE hTNETWIF, TI_HANDLE hReport, UINT32 uRegBaseAddr, UINT32 uMemBaseAddr, TNETWIF_callback_t fCb, TI_HANDLE hCb);
TI_STATUS TNETWIF_Destroy             (TI_HANDLE hTNETWIF);

/************************************************************************
 API for the recovery 
************************************************************************/
TI_STATUS TNETWIF_ReConfig            (TI_HANDLE hTNETWIF);

/************************************************************************
 API for the OS (PALAU) to handle the running module's Callbacks.
************************************************************************/
void      TNETWIF_BusTxn_Complete     (TI_HANDLE hTNETWIF);

/************************************************************************
 API for the Client to send an event to the Synchronizer : It can be :
            EV_REG_ID (PERFORM_IMMEDIATE or start time for future use)
            EV_FINISH_ID (To be called by the client when if finishes its State Machine
************************************************************************/
TI_STATUS TNETWIF_Start               (TI_HANDLE hTNETWIF, UINT8 module_id, TI_HANDLE CB_Handle,TNETWIF_callback_t CallBack_Func);
TI_STATUS TNETWIF_Restart             (TI_HANDLE hTNETWIF, UINT8 module_id, TI_HANDLE CB_Handle,TNETWIF_callback_t CallBack_Func);
TI_STATUS TNETWIF_Finish              (TI_HANDLE hTNETWIF, UINT8 module_id, TI_HANDLE CB_Handle,TNETWIF_callback_t CallBack_Func);

/************************************************************************
 API for the Client to use the ElpCtrl
************************************************************************/
int TNETWIF_ElpCtrl_Mode              (TI_HANDLE hTNETWIF, elpCtrl_Mode_e mode);
int TNETWIF_ElpCtrl_HostIF_required   (TI_HANDLE hTNETWIF, int flag);
TI_STATUS TNETWIF_UnMux (TI_HANDLE hTNETWIF);

/************************************************************************
 API for the Client to do I/O operations write/read Memory/Register 
************************************************************************/
TI_STATUS   TNETWIF_SetPartitionsOpt  (TI_HANDLE hTNETWIF, TNETIF_HwAccess_SetPartition_mode_e partitionMode, UINT32 partition_start, UINT8 module_id, TNETWIF_callback_t CBFunc, TI_HANDLE CB_Handle);
void TNETWIF_RegisterFailureEventCB   (TI_HANDLE hTNETWIF, void *failureEventCB, TI_HANDLE hFailureEventObj );
void TNETWIF_RegisterBusFailureEventCB(TI_HANDLE hTNETWIF, void *failureEventCB, TI_HANDLE hFailureEventObj );
void TNETWIF_RegisterFailureEventCB   (TI_HANDLE hTNETWIF, void *failureEventCB, TI_HANDLE hFailureEventObj);

/************************************************************************
 Optimized IO mode : In this mode the SDIO/SPI Driver will decide with its inner thresholds if to make a DMA or not
************************************************************************/
TI_STATUS   TNETWIF_ReadMemOpt        (TI_HANDLE hTNETWIF, UINT32 addr, UINT8* data, UINT32 len, UINT8 module_id, TNETWIF_callback_t CBFunc, TI_HANDLE CB_Handle);
TI_STATUS   TNETWIF_WriteMemOpt       (TI_HANDLE hTNETWIF, UINT32 addr, UINT8* data, UINT32 len, UINT8 module_id, TNETWIF_callback_t CBFunc, TI_HANDLE CB_Handle);
TI_STATUS   TNETWIF_ReadRegOpt        (TI_HANDLE hTNETWIF, UINT32 addr, UINT32* data,            UINT8 module_id, TNETWIF_callback_t CBFunc, TI_HANDLE CB_Handle);
TI_STATUS   TNETWIF_WriteRegOpt       (TI_HANDLE hTNETWIF, UINT32 addr, UINT32 data,             UINT8 module_id, TNETWIF_callback_t CBFunc, TI_HANDLE CB_Handle);
TI_STATUS   TNETWIF_WriteELPOpt       (TI_HANDLE hTNETWIF,              UINT32 data,             UINT8 module_id, TNETWIF_callback_t CBFunc, TI_HANDLE CB_Handle, BOOL bMore);
TI_STATUS   TNETWIF_ReadELPOpt        (TI_HANDLE hTNETWIF,              UINT8 *data, UINT8 module_id, TNETWIF_callback_t fCb, TI_HANDLE hCb, BOOL bMore);

#ifdef USE_SYNC_API

TI_STATUS   TNETWIF_SetPartitions     (TI_HANDLE hTNETWIF, TNETIF_HwAccess_SetPartition_mode_e partitionMode, UINT32 partition_start);

/************************************************************************
 Synchronous IO mode 
************************************************************************/
TI_STATUS   TNETWIF_ReadMemSync       (TI_HANDLE hTNETWIF, UINT32 addr, UINT8* data, UINT32 len);
TI_STATUS   TNETWIF_WriteMemSync      (TI_HANDLE hTNETWIF, UINT32 addr, UINT8* data, UINT32 len);
TI_STATUS   TNETWIF_ReadRegSync       (TI_HANDLE hTNETWIF, UINT32 addr, UINT32* data);
TI_STATUS   TNETWIF_WriteRegSync      (TI_HANDLE hTNETWIF, UINT32 addr, UINT32 data);
TI_STATUS   TNETWIF_WriteELPSync      (TI_HANDLE hTNETWIF,              UINT32 data);

/*
 * Hardware memory API
 */
UINT8       TNETWIF_GetU08            (TI_HANDLE hTNETWIF, UINT32 addr);
void        TNETWIF_SetU08            (TI_HANDLE hTNETWIF, UINT32 addr, UINT8  Val);
void        TNETWIF_SetU08_Bits       (TI_HANDLE hTNETWIF, UINT32 addr, UINT8  BitsVal);
void        TNETWIF_ResetU08_Bits     (TI_HANDLE hTNETWIF, UINT32 addr, UINT8  BitsVal);
UINT16      TNETWIF_GetU16            (TI_HANDLE hTNETWIF, UINT32 addr);
void        TNETWIF_SetU16            (TI_HANDLE hTNETWIF, UINT32 addr, UINT16 Val);
void        TNETWIF_SetU16_Bits       (TI_HANDLE hTNETWIF, UINT32 addr, UINT16 BitsVal);
void        TNETWIF_ResetU16_Bits     (TI_HANDLE hTNETWIF, UINT32 addr, UINT16 BitsVal);
UINT32      TNETWIF_GetU32            (TI_HANDLE hTNETWIF, UINT32 addr);
void        TNETWIF_SetU32            (TI_HANDLE hTNETWIF, UINT32 addr, UINT32 Val);
void        TNETWIF_SetU32_Bits       (TI_HANDLE hTNETWIF, UINT32 addr, UINT32 BitsVal);
void        TNETWIF_ResetU32_Bits     (TI_HANDLE hTNETWIF, UINT32 addr, UINT32 BitsVal);

/*
 * Hardware Registers API
 */
void        TNETWIF_RegSetBitVal      (TI_HANDLE hTNETWIF, UINT32 addr, UINT32 BitVal);
void        TNETWIF_RegResetBitVal    (TI_HANDLE hTNETWIF, UINT32 addr, UINT32 BitVal);
int         TNETWIF_RegIsBitSet       (TI_HANDLE hTNETWIF, UINT32 addr, UINT32 BitVal);

#endif /* USE_SYNC_API */

char* TNETWIF_ModuleIdToString(UINT32 module_id);
void  TNETWIF_printErrorLog(void);

#endif /* TNETWIF_H */

