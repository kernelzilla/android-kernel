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
 *   MODULE:  txXfer.c
 *   
 *   PURPOSE: Handle Tx frame transfer to the firmware in slave double-buffer scheme. 
 * 
 *   DESCRIPTION:  
 *   ============
 *      This module gets the upper driver's Tx packets after FW resources were
 *        allocated for it, and handles its transfer to the FW double-buffer.
 *      It can handle two packets at a time, thus providing a pipe-line behavior.
 *      It is planned to start an asynchronous copy of one packet to the FW,
 *        and immediately indicate the upper layer to start handling the next
 *        packet transmission in parallel.
 *
 ****************************************************************************/

#include "osTIType.h"
#include "paramIn.h"
#include "commonTypes.h"
#include "TNETWIF.h"
#include "whalCommon.h"
#include "whalHwDefs.h"
#include "txXfer_api.h"
#include "txXfer.h"  /* Local definitions */


/****************************************************************************
 *                      txXfer_Create()
 ****************************************************************************
 * DESCRIPTION: Create the Xfer module object 
 * 
 * INPUTS:  None
 * 
 * OUTPUT:  None
 * 
 * RETURNS: The Created object
 ****************************************************************************/
TI_HANDLE txXfer_Create(TI_HANDLE hOs)
{
    txXferObj_t *pTxXfer;

    pTxXfer = os_memoryAlloc(hOs, sizeof(txXferObj_t));
    if (pTxXfer == NULL)
    {
        return NULL;
    }

    os_memoryZero(hOs, pTxXfer, sizeof(txXferObj_t));

    pTxXfer->hOs = hOs;

    return (TI_HANDLE)pTxXfer;
}


/****************************************************************************
 *                      txXfer_Destroy()
 ****************************************************************************
 * DESCRIPTION: Destroy the Xfer module object 
 * 
 * INPUTS:  hTxXfer - The object to free
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
TI_STATUS txXfer_Destroy(TI_HANDLE hTxXfer)
{
    txXferObj_t *pTxXfer = (txXferObj_t *)hTxXfer;

    if (pTxXfer)
    {
        os_memoryFree(pTxXfer->hOs, pTxXfer, sizeof(txXferObj_t));
    }

    return OK;
}


/****************************************************************************
 *               txXfer_init()
 ****************************************************************************
   DESCRIPTION:  
   ============
     Initialize the Xfer module.
 ****************************************************************************/
TI_STATUS txXfer_init(TI_HANDLE hTxXfer, TI_HANDLE hReport, TI_HANDLE hTNETWIF, TI_HANDLE hTxResult)
{
    txXferObj_t *pTxXfer = (txXferObj_t *)hTxXfer;

    pTxXfer->hReport = hReport;
    pTxXfer->hTNETWIF = hTNETWIF;
    pTxXfer->hTxResult = hTxResult;
    pTxXfer->sendPacketTransferCB = NULL;
    pTxXfer->sendPacketDebugCB = NULL;

    return txXfer_restart(pTxXfer);
}

/****************************************************************************
 *               txXfer_config()
 ****************************************************************************
 * DESCRIPTION:  
 *      Configures the TX XFER module with initialization parameters.
 *          
 * INPUTS:  
 *      hTxXfer             The object
 *      pInitParams         initialization parameters values
 * 
 * OUTPUT:  None
 * 
 * RETURNS: None 
 ****************************************************************************/
void txXfer_config(TI_HANDLE hTxXfer, TnetwDrv_InitParams_t *pInitParams)
{
    txXferObj_t *pTxXfer = (txXferObj_t *)hTxXfer;

    pTxXfer->timeToTxStuckMs = pInitParams->txXferInitParams.timeToTxStuckMs;
}


/****************************************************************************
 *                      txXfer_setHwInfo()
 ****************************************************************************
 * DESCRIPTION:  
 *      Called after the HW configuration upon init or recovery.
 *      Store the HW addresses of the Double Buffer and the Tx-status.
 *          
 * INPUTS:  
 *      hTxXfer             The object
 *      pDataPathParams     Pointer to the Double Buffer Address
 * 
 * OUTPUT:  None
 * 
 * RETURNS: None 
 ****************************************************************************/
void  txXfer_setHwInfo(TI_HANDLE hTxXfer, ACXDataPathParamsResp_t *pDataPathParams)
{
    txXferObj_t *pTxXfer = (txXferObj_t *)hTxXfer;

    pTxXfer->dblBufAddr[0] = pDataPathParams->txPacketRingAddr;
    pTxXfer->dblBufAddr[1] = pDataPathParams->txPacketRingAddr + pDataPathParams->txPacketRingChunkSize;
    pTxXfer->txPathStatusAddr = pDataPathParams->txControlAddr;
    
    /* Print of the Tx double buffer address */
    WLAN_REPORT_INFORMATION(pTxXfer->hReport, TNETW_XFER_MODULE_LOG, 
        ("TX DOUBLE BUFFER Addresses: Buf_0 = 0x%x,   Buf_1 = 0x%x\n", 
        pTxXfer->dblBufAddr[0], pTxXfer->dblBufAddr[1]));
} 




    
/****************************************************************************
 *               txXfer_restart()
 ****************************************************************************
   DESCRIPTION:  
   ============
     Restarts the Xfer module.
     Should be called upon init and recovery!!
 ****************************************************************************/
TI_STATUS txXfer_restart(TI_HANDLE hTxXfer)
{
    txXferObj_t *pTxXfer = (txXferObj_t *)hTxXfer;

    /* Initialize module variables. */
    pTxXfer->txXferState = TX_XFER_STATE_IDLE;
    pTxXfer->numBufferedPkts = 0;
    pTxXfer->xferDonePostponed = FALSE;
    pTxXfer->bRecovery = FALSE;
    pTxXfer->dataInCount = 0;
    pTxXfer->hwStatusReadLoopCount = 0;

    return OK;
}




/****************************************************************************
 *                  txXfer_sendPacket()
 ****************************************************************************
 * DESCRIPTION: 
   ============
    Handle sent packet according to the number of packets already in the Xfer buffers:
      If no buffered pkts:
        If in IDLE state, update state to WAIT_BUS and request the bus.
        Return SEND_PACKET_SUCCESS.
      If one buffered pkt, just buffer the request and return SEND_PACKET_PENDING (can't get more).
      If two buffered pkts, return SEND_PACKET_ERROR (not expected to get more packets).
 ****************************************************************************/
systemStatus_e txXfer_sendPacket(TI_HANDLE hTxXfer, txCtrlBlkEntry_t *pPktCtrlBlk)
{
    txXferObj_t *pTxXfer = (txXferObj_t *)hTxXfer;
    TI_STATUS tnetwifStatus;

#ifdef TI_DBG
    pTxXfer->sendPacketCount++;
#endif

    /* Handle sent packet according to the number of packets already in the Xfer buffers. */
    switch(pTxXfer->numBufferedPkts) 
    {

        /* No buffered pkts. */ 
        case 0:

            pTxXfer->numBufferedPkts = 1;    
            pTxXfer->pPktCtrlBlk[0] = pPktCtrlBlk;    /* Save the first pkt Ctrl-Blk pointer. */
            
            /* If in IDLE state, update state to WAIT_BUS and request the bus. */
            if (pTxXfer->txXferState == TX_XFER_STATE_IDLE)
            {
                WLAN_REPORT_INFORMATION(pTxXfer->hReport, TNETW_XFER_MODULE_LOG,  
                    ("txXfer_sendPacket(): First Pkt, IDLE-STATE, Start Bus-Wait\n"));

                /* Note: Update the Xfer-SM state before calling the TNETWIF because it will call the
                         SM immediately if the bus is available. */
                pTxXfer->txXferState = TX_XFER_STATE_WAIT_BUS;    
                
                /* Set to detect if the Xfer proces is completed in this context (returns XFER_DONE). */
                pTxXfer->syncXferIndication = TRUE; 
                
                tnetwifStatus = TNETWIF_Start(pTxXfer->hTNETWIF, TX_XFER_MODULE_ID, pTxXfer, xferStateMachine);

                if (pTxXfer->bRecovery)
                    return SEND_PACKET_RECOVERY;

#ifdef TI_DBG
                if (tnetwifStatus == TNETWIF_COMPLETE)
                {
                    pTxXfer->busStartSyncCount++;
                }
                else if (tnetwifStatus == TNETWIF_PENDING)
                {
                    pTxXfer->busStartAsyncCount++;
                } 
                else
                {
                    WLAN_REPORT_ERROR(pTxXfer->hReport, TNETW_XFER_MODULE_LOG,  
                                      ("txXfer_sendPacket(): TNETWIF_Start returned error=%d\n", tnetwifStatus));
                }
#endif

                /* If the Xfer was completed in this context (Synchronous), return XFER_DONE. */
                /* Note: In this case xferDone callback will not be called!  */
                if (pTxXfer->syncXferIndication)
                {
                    pTxXfer->syncXferIndication = FALSE;

                    if (tnetwifStatus == TNETWIF_COMPLETE) /* The SM was called from the Start function. */
                    {
#ifdef TI_DBG
                        pTxXfer->xferDoneSyncCount++;
                        WLAN_REPORT_INFORMATION(pTxXfer->hReport, TNETW_XFER_MODULE_LOG,  
                            ("txXfer_sendPacket(): Xfer Completed in upper driver context\n"));
#endif
                        return SEND_PACKET_XFER_DONE;  /* Xfer can get another packet. */
                    }
                }
            }
            else
            {
                WLAN_REPORT_INFORMATION(pTxXfer->hReport, TNETW_XFER_MODULE_LOG,  
                    ("txXfer_sendPacket(): First Pkt, not IDLE-STATE, XferState=%d\n", pTxXfer->txXferState));
            }

            /* If the Xfer wasn't completed in this context (Asynchronous), return SUCCESS
                (xferDone callback will be called). */
            return SEND_PACKET_SUCCESS;  /* Xfer can get another packet. */



        /* If one buffered pkt, just buffer the request and return SEND_PACKET_PENDING (can't get more). */
        case 1:

            if (pTxXfer->bRecovery)
            {
                return SEND_PACKET_RECOVERY;
            }

            else
            {
                WLAN_REPORT_INFORMATION (pTxXfer->hReport, TNETW_XFER_MODULE_LOG,  
                    ("txXfer_sendPacket(): Second Pkt Buffered, XferState=%d\n", pTxXfer->txXferState));

                pTxXfer->numBufferedPkts = 2;  /* We now have two packets handled in the Xfer. */
                pTxXfer->pPktCtrlBlk[1] = pPktCtrlBlk;  /* Save the second pkt Ctrl-Blk pointer. */

                return SEND_PACKET_PENDING;  /* Xfer can't get further packets. */
            }


        /* If two buffered pkts, return SEND_PACKET_ERROR (not expected to get more packets). */
        default:
            
            WLAN_REPORT_ERROR(pTxXfer->hReport, TNETW_XFER_MODULE_LOG,  
                ("txXfer_sendPacket(): Two Pkts Allready Buffered, XferState=%d, xferDonePostponed=%d, numPkts=%d\n",
                pTxXfer->txXferState, pTxXfer->xferDonePostponed, pTxXfer->numBufferedPkts));

            return SEND_PACKET_ERROR;
    }
}


#define MAX_RECOVERY_LOOPS 1000


/****************************************************************************
 *                  xferStateMachine()
 ****************************************************************************
 * DESCRIPTION:  
   ============
    This is the Xfer process state machine.
    It handles the transfer of packets sent by the upper driver to the HW via TNETWIF.
   
    The SM supports both Sync and Async accesses to the HW.
    It loops and progresses from state to state as long as the HW is accessed synchronously.
    Once the access is Asynchronous (TNETWIF_PENDING), it exits and is called later
      by the TNETWIF when the HW is ready.
    That's why it uses only unspecified-mode accesses (e.g. TNETWIF_ReadMemOpt) which
      selects either Sync or Async automatically according to the platform.

    When the SM is active (not in IDLE state), it has either one or two packets buffered.
    This enables (in Async access) pipeline behavior, where one packet is transfered to 
      the HW, and the other is being processed from the upper driver to the Xfer in parallel.


    The SM steps are:
    =================

Event:     Send-Pkt       Bus-Ready         HW-Buffer-Ready         Xfer-Done             Trigger-Done
               |              |                    |                    |                       |
               V              V                    V                    V                       V
State:  IDLE ----> WAIT_BUS ----> WAIT_HW_BUFFER ----> WAIT_XFER_DONE ----> WAIT_TRIGGER_DONE ----> IDLE
                       |                                                             |  
                       |                                                             |  
                        <---------<---------<---------<---------<---------<----------
                                             Pending-Packet (*)

  (*) When a packet transfer is finished but another packet is already waiting in the Xfer 
        for processing, the Xfer will postpone the Xfer-Done indication of the first packet 
        until it has started the second one's transfer to the HW.
        It will request "Bus Restart" in order to prevent nesting and enable bus allocation
          to other tasks.

 ****************************************************************************/
static void xferStateMachine(TI_HANDLE hTxXfer, UINT8 module_id, TI_STATUS status)
{
    txXferObj_t *pTxXfer = (txXferObj_t *)hTxXfer;
    TI_STATUS tnetwifStatus;

#ifdef TI_DBG
    if (hTxXfer == NULL)
    {
        WLAN_REPORT_ERROR(pTxXfer->hReport, TNETW_XFER_MODULE_LOG,  
            ("xferStateMachine(): ****  Called with NULL handle!!  ****\n"));
        return;
    }
#endif
    
    /* 
     * Loop through the states sequence as long as the process is synchronous.
     * Exit when finished or if an Asynchronous process is required. In this case
     *   the SM process will be resumed later (called back by TNETWIF). 
     */
    while (1)
    {
        switch (pTxXfer->txXferState)
        {
            
            case TX_XFER_STATE_WAIT_BUS:

                /* Call debug callback */
                if (pTxXfer->sendPacketDebugCB)
                { 
                    pTxXfer->sendPacketDebugCB (pTxXfer->sendPacketDebugHandle, 
                                                pTxXfer->pPktCtrlBlk[0], 
                                                pTxXfer->txXferState);
                }

                /* We now own the bus, so request to read HW-Tx-Status and go to WAIT_HW_BUFFER state. */
                pTxXfer->txXferState = TX_XFER_STATE_WAIT_HW_BUFFER;

                /* also mark the time at which the first attemot is done */
                pTxXfer->TimeStampFirstHwBufferRead = os_timeStampMs( pTxXfer->hOs );

                tnetwifStatus = TNETWIF_ReadMemOpt (pTxXfer->hTNETWIF, 
                                                    pTxXfer->txPathStatusAddr, 
                                                    PADREAD (&pTxXfer->hwTxPathStatusRead), 
                                                    sizeof(UINT32), 
                                                    TX_XFER_MODULE_ID,
                                                    xferStateMachine, 
                                                    pTxXfer);
                
                break;


                
            case TX_XFER_STATE_WAIT_HW_BUFFER:

                /* Call debug callback */
                if (pTxXfer->sendPacketDebugCB)
                {
                    pTxXfer->sendPacketDebugCB (pTxXfer->sendPacketDebugHandle, 
                                                pTxXfer->pPktCtrlBlk[0], 
                                                pTxXfer->txXferState);
                }
                
                /* We now have the HW-Tx-Status read, so check if there are HW buffers available. */
                if (hwBuffersOccupied(pTxXfer) < DP_TX_PACKET_RING_CHUNK_NUM)
                {
                    /* Handle actual packet transfer to HW buffer. */
                    tnetwifStatus = transferPacket(pTxXfer);

                    pTxXfer->txXferState = TX_XFER_STATE_WAIT_XFER_DONE; 

                    /* 
                     * If we've postponed the Xfer-Done callback of the previous transfered 
                     *   packet, call it now in parallel with the Xfer of the current packet.
                     * Note that all variables are updated before calling the XferDone, since
                     *   it may be used to send another packet.
                     * The current transfered packet pointer is moved to the first buffer.
                     */
                    if (pTxXfer->xferDonePostponed)
                    {
                        txCtrlBlkEntry_t *pPostponedPktCtrlBlk = pTxXfer->pPktCtrlBlk[0];
                        pTxXfer->numBufferedPkts = 1;
                        pTxXfer->pPktCtrlBlk[0] = pTxXfer->pPktCtrlBlk[1];
                        pTxXfer->xferDonePostponed = FALSE;
                        pTxXfer->sendPacketTransferCB(pTxXfer->sendPacketTransferHandle, pPostponedPktCtrlBlk);
#ifdef TI_DBG
                        pTxXfer->xferDoneCallCBCount++;
#endif
                    }
                    pTxXfer->hwStatusReadLoopCount = 0;
#ifdef TI_DBG
                    pTxXfer->hwBufferReadCount++;
#endif
                }

                /* If HW buffer isn't available, try reading the status again (loop on same state). */
                else
                {
                    tnetwifStatus = TNETWIF_ReadMemOpt (pTxXfer->hTNETWIF, 
                                                        pTxXfer->txPathStatusAddr, 
                                                        PADREAD (&pTxXfer->hwTxPathStatusRead), 
                                                        sizeof(UINT32), 
                                                        TX_XFER_MODULE_ID,
                                                        xferStateMachine, 
                                                        pTxXfer);

#ifdef TI_DBG
                    /* For Debug:  Update counters */
                    pTxXfer->hwBufferFullCount++;
                    pTxXfer->hwBufferReadCount++;
#endif
                    /* Detect endless loop and perform recovery if needed */
                    pTxXfer->hwStatusReadLoopCount++;
                    if (os_timeStampMs (pTxXfer->hOs) - pTxXfer->TimeStampFirstHwBufferRead > 
                        pTxXfer->timeToTxStuckMs)
                    {
                        WLAN_REPORT_ERROR(pTxXfer->hReport, TNETW_XFER_MODULE_LOG,  
                            ("xferStateMachine(): Looping too long for Tx-Status, LastTxStatus=%d\n",   
                            pTxXfer->hwTxPathStatusRead));
                        WLAN_REPORT_ERROR(pTxXfer->hReport, TNETW_XFER_MODULE_LOG,
                                          ("Loop count=%d, First time stamp:%d, current time stamp:%d\n",
                                           pTxXfer->hwStatusReadLoopCount, pTxXfer->TimeStampFirstHwBufferRead,
                                           os_timeStampMs( pTxXfer->hOs )) );
                            
                      #ifdef USE_RECOVERY     
                        pTxXfer->bRecovery = TRUE;         
                        /* Error Reporting - if after a configurable interval we could not 
                           transfer data a recovery will be called */
                        if (pTxXfer->failureEventFunc)
                        {
                            pTxXfer->failureEventFunc(pTxXfer->failureEventObj, TX_STUCK);
                        }

                        return;
                      #endif
                    }         
                }
                
                break;

            case TX_XFER_STATE_WAIT_XFER_DONE:

                /* Call debug callback */
                if (pTxXfer->sendPacketDebugCB)
                {
                    pTxXfer->sendPacketDebugCB (pTxXfer->sendPacketDebugHandle, 
                                                pTxXfer->pPktCtrlBlk[0], 
                                                pTxXfer->txXferState);
                }
                
                /* Now the last packet transfer to the HW is finished, so we issue a trigger to the FW. */

                {
                    UINT32 txInterruptRegAddress;
                    UINT32 txInterruptRegData;

                    /* Set the Tx-interrupt address and value according to the
                         HW buffer used for the last transfer. */
                    if (pTxXfer->dataInCount & 0x1)
                    {
                        txInterruptRegAddress = ACX_REG_INTERRUPT_TRIG_H;
                        txInterruptRegData = INTR_TRIG_TX_PROC1;
                    }
                    else
                    {
                        txInterruptRegAddress = ACX_REG_INTERRUPT_TRIG;
                        txInterruptRegData = INTR_TRIG_TX_PROC0;
                    }                   

                    /* Call debug callback */
                    if (pTxXfer->sendPacketDebugCB)
                    {
                        pTxXfer->sendPacketDebugCB (pTxXfer->sendPacketDebugHandle, 
                                                    pTxXfer->pPktCtrlBlk[0], 
                                                    0);
                    }

                    /* Issue the Tx interrupt trigger to the FW. */   
                    tnetwifStatus = TNETWIF_WriteRegOpt(pTxXfer->hTNETWIF, 
                                                        txInterruptRegAddress, 
                                                        txInterruptRegData, 
                                                        TX_XFER_MODULE_ID,
                                                        xferStateMachine, 
                                                        pTxXfer);

                    /* Increment the transfered packets counter modulo 16 (as FW data-out counter). */ 
                    pTxXfer->dataInCount = (pTxXfer->dataInCount + 1) & TX_STATUS_DATA_OUT_COUNT_MASK;

                    pTxXfer->txXferState = TX_XFER_STATE_WAIT_TRIGGER_DONE; 
                }   
                
                break;
               
            case TX_XFER_STATE_WAIT_TRIGGER_DONE:

                /* Call debug callback */
                if (pTxXfer->sendPacketDebugCB)
                {
                    pTxXfer->sendPacketDebugCB (pTxXfer->sendPacketDebugHandle, 
                                                pTxXfer->pPktCtrlBlk[0], 
                                                pTxXfer->txXferState);
                }

                /* Now the HW Tx trigger is done so we can continue to the next packet if waiting. */

                /* If we don't have another packet pending for transfer. */
                if (pTxXfer->numBufferedPkts == 1)
                {
                    pTxXfer->numBufferedPkts = 0;

                    /*
                     * Call the XferDone callback, but only if we are not in the original 
                     *   SendPacket context (i.e. completely synchronous). 
                     * This is to avoid nesting, since the callback may start another SendPacket.
                     */
                    if (!pTxXfer->syncXferIndication)
                    {
#ifdef TI_DBG
                        pTxXfer->xferDoneCallCBCount++;
                        WLAN_REPORT_INFORMATION(pTxXfer->hReport, TNETW_XFER_MODULE_LOG,  
                            ("sendPacketTransferCB: CB_Func=0x%x, CB_Handle=0x%x\n", 
                            pTxXfer->sendPacketTransferCB, pTxXfer->sendPacketTransferHandle));
#endif
                        pTxXfer->sendPacketTransferCB(pTxXfer->sendPacketTransferHandle, pTxXfer->pPktCtrlBlk[0]);
                    }

                    /* If still no packet was sent, release bus (Finish), set IDLE state and exit. */ 
                    if (pTxXfer->numBufferedPkts == 0)
                    {
                        pTxXfer->txXferState = TX_XFER_STATE_IDLE; 
                        TNETWIF_Finish (pTxXfer->hTNETWIF, TX_XFER_MODULE_ID, pTxXfer, NULL);
                        
                        WLAN_REPORT_INFORMATION(pTxXfer->hReport, TNETW_XFER_MODULE_LOG,  
                            ("txXferSM Finished -> IDLE: NumPkts=%d, XferDonePostponed=%d, SyncIndication=%d\n", 
                            pTxXfer->numBufferedPkts, pTxXfer->xferDonePostponed, pTxXfer->syncXferIndication));

                        return;   /************    Exit State Machine (back to IDLE)   ************/
                    }

                    /* 
                     * A new packet was sent (in XferDone callback), so request the bus again using Restart.
                     * This will call the SM later, and start the process again from WAIT_BUS state.
                     */
                    else
                    {
                        pTxXfer->txXferState = TX_XFER_STATE_WAIT_BUS; 
                        tnetwifStatus = TNETWIF_Restart(pTxXfer->hTNETWIF, TX_XFER_MODULE_ID, pTxXfer, xferStateMachine);
#ifdef TI_DBG
                        pTxXfer->busRestartCount++;
#endif
                    }
                }

                /*
                 * We have another packet pending.
                 * So to enable parallel processing, we postpone the XferDone callback (just set flag).
                 * Thus, we'll start first the new packet transfer and only than call the postponed
                 *   XferDone (see WAIT_HW_BUFFER state), which may start another SendPacket in 
                 *   parallel to the HW transfer.
                 * Note that we request the bus again using Restart (to avoid nesting or bus starvation).
                 * This will call the SM later, and start the process again from WAIT_BUS state.
                 */
                else
                {
                    pTxXfer->xferDonePostponed = TRUE;
                    pTxXfer->txXferState = TX_XFER_STATE_WAIT_BUS; 
                    tnetwifStatus = TNETWIF_Restart(pTxXfer->hTNETWIF, TX_XFER_MODULE_ID, pTxXfer, xferStateMachine);
#ifdef TI_DBG
                    pTxXfer->busRestartCount++;
                    pTxXfer->xferDonePostponeCount++;
#endif
                }

                break;

                
                
            default:
                    WLAN_REPORT_ERROR(pTxXfer->hReport, TNETW_XFER_MODULE_LOG,  
                        ("xferStateMachine(): Unexpected state, txXferState=%d, NumPkts=%d\n",  
                        pTxXfer->txXferState, pTxXfer->numBufferedPkts));
                    
                return;

        }  /* switch (pTxXfer->txXferState) */

        WLAN_REPORT_INFORMATION(pTxXfer->hReport, TNETW_XFER_MODULE_LOG,  
            ("txXferSM(): SmState=%d, NumPkts=%d, XferDonePostponed=%d, TnetwIfStatus=%d, SyncIndication=%d, HwTxStatus=0x%x, DataInCount=0x%x\n", 
            pTxXfer->txXferState, pTxXfer->numBufferedPkts, pTxXfer->xferDonePostponed, 
            tnetwifStatus, pTxXfer->syncXferIndication, pTxXfer->hwTxPathStatusRead, pTxXfer->dataInCount));

        /* 
         * If the last HW access request was pended, exit the SM (Asynchronous process).
         * The SM will be called back when the HW access is done.
         * Also reset the Sync flag to notify that the Xfer wasn't completed in the SendPacket context.
         */
        if (tnetwifStatus == TNETWIF_PENDING)
        {
            pTxXfer->syncXferIndication = FALSE;

            return;  /**********    Exit State Machine (to be called back by TNETWIF)   **********/
        }

#ifdef TI_DBG
        else if (tnetwifStatus == TNETWIF_ERROR)
        {
            WLAN_REPORT_ERROR(pTxXfer->hReport, TNETW_XFER_MODULE_LOG,  
                ("txXferSM TNETWIF_ERROR: SmState=%d, NumPkts=%d, XferDonePostponed=%d, TnetwIfStatus=%d, SyncIndication=%d, HwTxStatus=0x%x, DataInCount=0x%x\n", 
                pTxXfer->txXferState, pTxXfer->numBufferedPkts, pTxXfer->xferDonePostponed, 
                tnetwifStatus, pTxXfer->syncXferIndication, pTxXfer->hwTxPathStatusRead, pTxXfer->dataInCount));
            return;
        }
#endif

    }  /* while (1) */

}




/****************************************************************************
 *                  hwBuffersOccupied()
 ****************************************************************************
 * DESCRIPTION: 
   ============
   Return the number of occupied buffers in the HW Tx double buffer, based on
     the last read of the HW Tx path status.
 ****************************************************************************/
static UINT32 hwBuffersOccupied(txXferObj_t *pTxXfer)
{
    UINT32 dataOutCount = pTxXfer->hwTxPathStatusRead & TX_STATUS_DATA_OUT_COUNT_MASK;

    /* Return the difference between the packets transfered to the double buffer (by host)
        and the packets copied from it (by FW). The else is for counter wrap around. */
    
    if (pTxXfer->dataInCount >= dataOutCount)
    {
        return pTxXfer->dataInCount - dataOutCount;
    }
    else
    {
        return pTxXfer->dataInCount + TX_STATUS_DATA_OUT_COUNT_MASK + 1 - dataOutCount;
    }
}




/****************************************************************************
 *                  transferPacket()
 ****************************************************************************
 * DESCRIPTION: 
   ============
   Handle the current packet transfer to the HW Tx double buffer.
   Return the transfer status:
     TNETWIF_COMPLETE - if completed, i.e. Synchronous mode.
     TNETWIF_PENDING  - if pending, i.e. Asynchronous mode (callback function will be called). 

    
   If the packet was pending during disconnect, notify the TxResult to issue Tx-Complete,
     and return TNETWIF_COMPLETE, since we don't transfer it to the FW.

 ****************************************************************************/
static TI_STATUS transferPacket(txXferObj_t *pTxXfer)
{
    UINT16 XferLength;  /* The actual length of the transfer. */
    txCtrlBlkEntry_t *pPktCtrlBlk;  /* The packet control block pointer. */
    TI_STATUS status; 
    
    /* Get the current packet control-block pointer. If we've postponed the last packet Xfer-Done (i.e.
        was transfered but still buffered) than our current packet is the second one. */
    pPktCtrlBlk = pTxXfer->pPktCtrlBlk[ ((pTxXfer->xferDonePostponed) ? 1 : 0) ];

    /* Get the packet length, add descriptor length and align upward to 32 bit. */
    XferLength = (pPktCtrlBlk->txDescriptor.length + sizeof(DbTescriptor) + ALIGN_32BIT_MASK) & ~ALIGN_32BIT_MASK; 

    /* Initiate the packet transfer. The status indicates if Sync or Async mode was used!! */
    status = TNETWIF_WriteMemOpt (pTxXfer->hTNETWIF,  
                                  pTxXfer->dblBufAddr[pTxXfer->dataInCount & 0x1], 
                                  (UINT8 *)(pPktCtrlBlk->txPktParams.pFrame), 
                                  XferLength, 
                                  TX_XFER_MODULE_ID,
                                  xferStateMachine, 
                                  pTxXfer);

#ifdef TI_DBG
    WLAN_REPORT_INFORMATION(pTxXfer->hReport, TNETW_XFER_MODULE_LOG, 
        ("transferPacket(): Status=%d, XferLength=%d, DataInCount=0x%x, pPktCtrlBlk=0x%x, dbgPktSeqNum=%d, Expiry=%d\n", 
        status, XferLength, pTxXfer->dataInCount, pPktCtrlBlk, pPktCtrlBlk->txPktParams.dbgPktSeqNum,
        pPktCtrlBlk->txDescriptor.expiryTime));

    if (status == TNETWIF_COMPLETE)
    {
        pTxXfer->pktTransferSyncCount++;
    }
    else if (status == TNETWIF_PENDING)
    {
        pTxXfer->pktTransferAsyncCount++;
    }
    else
    {
        WLAN_REPORT_ERROR(pTxXfer->hReport, TNETW_XFER_MODULE_LOG, 
            ("transferPacket - TNETWIF_ERROR: Status=%d, XferLength=%d, DataInCount=0x%x, pPktCtrlBlk=0x%x, dbgPktSeqNum=%d, Expiry=%d\n", 
            status, XferLength, pTxXfer->dataInCount, pPktCtrlBlk, pPktCtrlBlk->txPktParams.dbgPktSeqNum,
            pPktCtrlBlk->txDescriptor.expiryTime));
    }
#endif

    /* Return the transfer status:
         TNETWIF_COMPLETE - if completed, i.e. Synchronous mode.
         TNETWIF_PENDING  - if pending, i.e. Asynchronous mode (callback function will be called). 
    */
    return status;
}




/****************************************************************************
 *                      txXfer_RegisterCB()
 ****************************************************************************
 * DESCRIPTION:  Register the upper driver Xfer callback functions.
 ****************************************************************************/
void txXfer_RegisterCB(TI_HANDLE hTxXfer, tiUINT32 CallBackID, void *CBFunc, TI_HANDLE CBObj)
{
    txXferObj_t* pTxXfer = (txXferObj_t*)hTxXfer;

    WLAN_REPORT_INFORMATION(pTxXfer->hReport, TNETW_XFER_MODULE_LOG, 
        ("txXfer_RegisterCB(): CallBackID=%d, CBFunc=0x%x, CBObj=0x%x\n", CallBackID, CBFunc, CBObj));

    switch(CallBackID)
    {
        /* Set Transfer-Done callback */
        case TX_XFER_SEND_PKT_TRANSFER:
            pTxXfer->sendPacketTransferCB = (SendPacketTranferCB_t)CBFunc;
            pTxXfer->sendPacketTransferHandle = CBObj;
            break;

        case TX_XFER_SEND_PKT_DEBUG:
            pTxXfer->sendPacketDebugCB = (SendPacketDebugCB_t)CBFunc;
            pTxXfer->sendPacketDebugHandle = CBObj;
            break;

        default:
            WLAN_REPORT_ERROR(pTxXfer->hReport, TNETW_XFER_MODULE_LOG, ("txXfer_RegisterCB - Illegal value\n"));
            break;
    }
}



/****************************************************************************************
 *                        txXfer_RegisterFailureEventCB                                                 *
 ****************************************************************************************
DESCRIPTION: Registers a failure event callback for scan error notifications.
                
                                                                                                                   
INPUT:      - hTxXfer       - handle to the xfer object.        
            - failureEventCB    - the failure event callback function.\n
            - hFailureEventObj - handle to the object passed to the failure event callback function.

OUTPUT: 
RETURN:    void.
****************************************************************************************/

void txXfer_RegisterFailureEventCB( TI_HANDLE hTxXfer, 
                                     void * failureEventCB, TI_HANDLE hFailureEventObj )
{
    txXferObj_t *pTxXfer = (txXferObj_t *)hTxXfer;

    pTxXfer->failureEventFunc   = (failureEventCB_t)failureEventCB;
    pTxXfer->failureEventObj    = hFailureEventObj;
}

/****************************************************************************
 *                      txXfer_printInfo()
 ****************************************************************************
 * DESCRIPTION:  Print the txXfer object main fields.
 ****************************************************************************/
void txXfer_printInfo(TI_HANDLE hTxXfer)
{
#ifdef TI_DBG
    txXferObj_t *pTxXfer = (txXferObj_t *)hTxXfer;

    WLAN_OS_REPORT(("Tx-Xfer Module Information:\n"));
    WLAN_OS_REPORT(("===========================\n"));

    switch (pTxXfer->txXferState)
    {
        case TX_XFER_STATE_IDLE:                
            WLAN_OS_REPORT(("State = IDLE\n")); 
            break;

        case TX_XFER_STATE_WAIT_BUS:
            WLAN_OS_REPORT(("State = WAIT_BUS\n")); 
        break;

        case TX_XFER_STATE_WAIT_HW_BUFFER:
            WLAN_OS_REPORT(("State = WAIT_HW_BUFFER\n"));   
        break;

        case TX_XFER_STATE_WAIT_XFER_DONE:
            WLAN_OS_REPORT(("State = WAIT_XFER_DONE\n"));   
        break;

        case TX_XFER_STATE_WAIT_TRIGGER_DONE:
            WLAN_OS_REPORT(("State = WAIT_TRIGGER_DONE\n"));    
        break;

        default:
            WLAN_OS_REPORT(("State = UNKNOWN !!\n"));
        break;
    }
    
    WLAN_OS_REPORT(("numBufferedPkts    = %d\n", pTxXfer->numBufferedPkts));
    WLAN_OS_REPORT(("xferDonePostponed  = %d\n", pTxXfer->xferDonePostponed));
    WLAN_OS_REPORT(("syncXferIndication = %d\n", pTxXfer->syncXferIndication));
    WLAN_OS_REPORT(("pPktCtrlBlk[0]     = 0x%x\n", pTxXfer->pPktCtrlBlk[0]));
    WLAN_OS_REPORT(("pPktCtrlBlk[1]     = 0x%x\n", pTxXfer->pPktCtrlBlk[1]));
    WLAN_OS_REPORT(("hwTxPathStatusRead = 0x%x\n", pTxXfer->hwTxPathStatusRead));
    WLAN_OS_REPORT(("dataInCount        = 0x%x\n", pTxXfer->dataInCount));
    WLAN_OS_REPORT(("txPathStatusAddr   = 0x%x\n", pTxXfer->txPathStatusAddr));
    WLAN_OS_REPORT(("dblBufAddr[0]      = 0x%x\n", pTxXfer->dblBufAddr[0]));
    WLAN_OS_REPORT(("dblBufAddr[1]      = 0x%x\n\n", pTxXfer->dblBufAddr[1]));
    
    WLAN_OS_REPORT(("hwBufferReadCount      = %d\n", pTxXfer->hwBufferReadCount));
    WLAN_OS_REPORT(("hwBufferFullCount      = %d\n", pTxXfer->hwBufferFullCount));
    WLAN_OS_REPORT(("sendPacketCount        = %d\n", pTxXfer->sendPacketCount));
    WLAN_OS_REPORT(("busStartSyncCount      = %d\n", pTxXfer->busStartSyncCount));
    WLAN_OS_REPORT(("busStartAsyncCount     = %d\n", pTxXfer->busStartAsyncCount));
    WLAN_OS_REPORT(("pktTransferSyncCount   = %d\n", pTxXfer->pktTransferSyncCount));
    WLAN_OS_REPORT(("pktTransferAsyncCount  = %d\n", pTxXfer->pktTransferAsyncCount));
    WLAN_OS_REPORT(("busRestartCount        = %d\n", pTxXfer->busRestartCount));
    WLAN_OS_REPORT(("xferDonePostponeCount  = %d\n", pTxXfer->xferDonePostponeCount));
    WLAN_OS_REPORT(("xferDoneSyncCount      = %d\n", pTxXfer->xferDoneSyncCount));
    WLAN_OS_REPORT(("xferDoneCallCBCount    = %d\n", pTxXfer->xferDoneCallCBCount));
#endif /* TI_DBG */
}
