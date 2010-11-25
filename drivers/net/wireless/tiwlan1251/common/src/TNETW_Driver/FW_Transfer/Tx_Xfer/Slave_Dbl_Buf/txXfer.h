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
/*    MODULE:   txXfer.h                                                   */
/*    PURPOSE:  Tx-Xfer module Header file - slave-mode, double-buffer     */
/*                                                                         */
/***************************************************************************/
#ifndef _TX_XFER_H_
#define _TX_XFER_H_


#include "TNETW_Driver_types.h"


#define MAX_XFER_BUFFERS                2       /* The number of frames that can be handled in 
                                                    the XFER module at a time. */
#define TX_STATUS_DATA_OUT_COUNT_MASK   0x000F  /* Tx status register mask of the FW counter of
                                                    packets copied from its Tx double buffer. */
#define ALIGN_32BIT_MASK                0x3     /* Masked LS bits for 32 bit aligned addresses or lengths.*/


typedef enum
{
    TX_XFER_STATE_IDLE,
    TX_XFER_STATE_WAIT_BUS,
    TX_XFER_STATE_WAIT_HW_BUFFER,
    TX_XFER_STATE_WAIT_XFER_DONE,
    TX_XFER_STATE_WAIT_TRIGGER_DONE
} TxXferState_e;


/* Callback function definition for Tx sendPacketTranfer */
typedef void (* SendPacketTranferCB_t)(TI_HANDLE CBObj, txCtrlBlkEntry_t *pPktCtrlBlk);
typedef void (* SendPacketDebugCB_t)  (TI_HANDLE CBObj, txCtrlBlkEntry_t *pPktCtrlBlk, UINT32 uDebugInfo);


/* The TxXfer module object. */
typedef struct 
{
    TI_HANDLE  hOs;
    TI_HANDLE  hReport;
    TI_HANDLE  hTNETWIF;
    TI_HANDLE  hTxResult;

    UINT32 numBufferedPkts; /* The number of packets buffered in the Xfer module (0, 1 or 2). */
    BOOL xferDonePostponed; /* Indicates if postponed the last Xfer-Done event to the upper driver. */
    BOOL syncXferIndication; /* Cleared once the sendPkt sequence is broken to Async mode. */
    TxXferState_e txXferState; /* The current state of the Xfer state machine. */
    txCtrlBlkEntry_t *pPktCtrlBlk[MAX_XFER_BUFFERS]; /* The pointers to the transfered packets ctrl-blk. */
    UINT32 dataInCount;  /* The number of packets transfered to FW modulu 16. */
    SendPacketTranferCB_t sendPacketTransferCB; /* Xfer-Done callback */
    TI_HANDLE sendPacketTransferHandle;         /* Xfer-Done callback handle */
    SendPacketDebugCB_t sendPacketDebugCB;      /* Xfer-Debug callback */
    TI_HANDLE sendPacketDebugHandle;            /* Xfer-Debug callback handle */
    UINT32 dblBufAddr[DP_TX_PACKET_RING_CHUNK_NUM];  /* The HW Tx double buffer. */
    UINT32 txPathStatusAddr;  /* The HW Tx status register address. */

    /* use a struct to read buffers from the bus - used for extra bytes reserving */
    PADDING (UINT32 hwTxPathStatusRead) /* Saves the last read from the HW Tx-Path-Status. */

    /*Error handling*/
    failureEventCB_t        failureEventFunc;   /* upper layer Failure Event CB.                                            * called when the scan command has been Timer Expiry*/
    TI_HANDLE           failureEventObj;    /* object parameter passed to the failureEventFunc when it is called */

    UINT32 hwStatusReadLoopCount; /* Count loops of HW status polling to detect endless loop. */
    UINT32 timeToTxStuckMs;       /* The time (in msec) from first attempt to get a HW buffer status
                                    until recovery is triggered */
    UINT32 TimeStampFirstHwBufferRead;
                                    /* Time stamp at the first attempt to read the HW buffer status */
    BOOL   bRecovery;             /* Recovery indicator */  

#ifdef TI_DBG  /* Debug Counters */
    UINT32 hwBufferReadCount;     /* Count total number of HW-Tx-Status Reads. */
    UINT32 hwBufferFullCount;     /* Count total number of status reads where the HW Xfer buffers were full. */
    UINT32 sendPacketCount;       /* Count number of SendPacket calls from upper driver. */
    UINT32 busStartSyncCount;     /* Count number of calls to arbiter bus Start request returned Complete. */
    UINT32 busStartAsyncCount;    /* Count number of calls to arbiter bus Start request returned Pending. */
    UINT32 pktTransferSyncCount;  /* Count number of transfered packets in sync mode. */
    UINT32 pktTransferAsyncCount; /* Count number of transfered packets in sync mode. */
    UINT32 busRestartCount;       /* Count number of calls to arbiter bus Restart request. */
    UINT32 xferDonePostponeCount; /* Count number of postponed XferDone calls. */
    UINT32 xferDoneSyncCount;     /* Count number of XferDone synchronous event. */
    UINT32 xferDoneCallCBCount;   /* Count number of XferDone synchronous event. */
#endif

} txXferObj_t;



/* Local function definitions */
static void         xferStateMachine(TI_HANDLE hTxXfer, UINT8 module_id, TI_STATUS status);
static UINT32       hwBuffersOccupied(txXferObj_t *pTxXfer);
static TI_STATUS    transferPacket(txXferObj_t *pTxXfer);



#endif  /* _TX_XFER_H_ */
        


