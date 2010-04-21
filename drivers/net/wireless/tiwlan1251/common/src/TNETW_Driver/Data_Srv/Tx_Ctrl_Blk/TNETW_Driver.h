/** \file TNETW_Driver.h
 *  \brief TNETW Driver include file
 *
 *  \see TNETW_Driver.c
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
/*                                                                          */
/*    MODULE:   TNETW_Driver.h                                              */
/*    PURPOSE:  TNETW Driver include file                                   */
/*                                                                          */
/***************************************************************************/


#ifndef TNETW_DRIVER_H
#define TNETW_DRIVER_H


#include "TNETW_Driver_types.h"
#include "whalCtrl_api.h"



#define TNETW_DRIVER_CB_MODULE_OWNER_MASK   0xff00
#define TNETW_DRIVER_CB_TYPE_MASK           0x00ff

#define SHIFT_BETWEEN_TU_AND_USEC  10  /* Shift factor to conver between TU (1024 uSec) and uSec. */


/* TNETW-Driver object */
typedef struct T_TnetwDrv_t
{
    TI_HANDLE           hOs;
    TI_HANDLE           hUser;
    TI_HANDLE           hReport;
    TI_HANDLE           hMemMgr;
    TI_HANDLE           hMacServices;
    TI_HANDLE           hHalCtrl;
    TI_HANDLE           hTxCtrlBlk;
    TI_HANDLE           hTxHwQueue;
    TI_HANDLE           hTNETWIF;
    TI_HANDLE           hHwIntr;
    TI_HANDLE           hWhalParams;

    /* 
     * Ctrl modules:
     */
    TI_HANDLE           hCmdQueue;
#if !defined(GWSI_DRIVER) && !defined(GWSI_LIB)
    TI_HANDLE           hRecoveryCtrl;
#endif

    /* 
     * FW-Transfer modules:
     */
    TI_HANDLE           hTxXfer;
    TI_HANDLE           hTxResult;
    TI_HANDLE           hRxXfer;
    TI_HANDLE           hFwEvent;
#if !defined(GWSI_DRIVER) && !defined(GWSI_LIB)
    TI_HANDLE           hHwInit;
    BOOL                bRecoveryFlag;/*if true it means that we are in recovery process*/
#endif
    TI_HANDLE           hCmdMBox;
    TI_HANDLE           hEventMbox;

    void               *pInitTableCopy;

    /* Init success flag */
    BOOL                bInitSuccess;

    /* User application configuration callback */
    TnetDrv_callback_t  fUserConf;

    /* CB at the end of TnetwDrv_Configure(). not called if no registration was done */
    TnetDrv_callback_t  fConfigureCmplteteCB;
    TI_HANDLE           hConfigureCompleteOBj;  

    /* CB at the end of TnetwDrv_Configure(). not called if no registration was done */
    TnetDrv_callback_t      fConfigureEndCB;
    TI_HANDLE           fConfigureEndObj;   

#ifdef TI_DBG  /* Just for debug. */
    TI_HANDLE           hDebugTrace;

    UINT32 dbgPktSeqNum[MAX_NUM_OF_TX_QUEUES];          /* Packets sequence counter per queue. */
    /* Tx counters per queue:*/
    UINT32 dbgCountSentPackets[MAX_NUM_OF_TX_QUEUES];   /* Count packets sent from upper driver. */
    UINT32 dbgCountQueueAvailable[MAX_NUM_OF_TX_QUEUES];/* Count packets sent and queue not busy. */
    UINT32 dbgCountXferDone[MAX_NUM_OF_TX_QUEUES];      /* Count XferDone return values from Xfer. */
    UINT32 dbgCountXferSuccess[MAX_NUM_OF_TX_QUEUES];   /* Count Success return values from Xfer. */
    UINT32 dbgCountXferPending[MAX_NUM_OF_TX_QUEUES];   /* Count Pending return value from Xfer. */
    UINT32 dbgCountXferError[MAX_NUM_OF_TX_QUEUES];     /* Count Error return value from Xfer. */
    UINT32 dbgCountXferDoneCB[MAX_NUM_OF_TX_QUEUES];    /* Count XferDone callback calls. */
    UINT32 dbgCountTxCompleteCB[MAX_NUM_OF_TX_QUEUES];  /* Count TxComplete callback calls. */
#endif
    
} TnetwDrv_t;


void TnetwDrv_TxXferDone(TI_HANDLE hTnetwDrv, txCtrlBlkEntry_t *pPktCtrlBlk);
void TnetwDrv_TxComplete(TI_HANDLE hTnetwDrv, TxResultDescriptor_t *pTxResultInfo);
void TnetwDrv_TxXferDebug (TI_HANDLE hTnetwDrv, txCtrlBlkEntry_t *pPktCtrlBlk, UINT32 uDebugInfo);
void TnetwDrv_RecoveryCtrlBlk(TI_HANDLE hTnetwDrv);


/* External Functions Prototypes */

void  SendPacketTransfer (TI_HANDLE hUser, UINT32 aPacketId);
void  SendPacketDebug (TI_HANDLE hUser, UINT32 aPacketId, UINT32 uDebugInfo);
void  SendPacketComplete (TI_HANDLE hUser, systemStatus_e aStatus, UINT32 aPacketId, UINT32 aRate, 
                                 UINT8 aAckFailures, UINT32 durationInAir, UINT32 fwHandlingTime, UINT32 mediumDelay);




#endif /* TNETW_DRIVER_H */

