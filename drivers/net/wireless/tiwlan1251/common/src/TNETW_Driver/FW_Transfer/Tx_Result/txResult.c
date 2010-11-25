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
 *   MODULE:  txResult.c
 *   
 *   PURPOSE:  Handle packets Tx results upon Tx-complete from the FW. 
 * 
 *   DESCRIPTION:  
 *   ============
 *      This module is called upon Tx-complete from FW. 
 *      It retrieves the transmitted packets results from the FW TxResult table and
 *        calls the upper layer callback function for each packet with its results.
 *
 ****************************************************************************/

#include "osTIType.h"
#include "whalCommon.h"
#include "TNETWIF.h"
#include "whalHwDefs.h"
#include "txResult_api.h"
#include "TNETW_Driver_types.h"
#include "FwEvent_api.h"
#include "txResult.h"  /* Local definitions */
 
/****************** static function decleration *****************************************/
static void txResult_handleNewEntries(txResultObj_t *pTxResult);
static void txResult_StateMachine(TI_HANDLE hTxResult,UINT8 module_id ,TI_STATUS status);
static TI_STATUS txResult_writeNewEntries(txResultObj_t *pTxResult,UINT8 currBuffer);


/****************************************************************************
 *                      txResult_Create()
 ****************************************************************************
 * DESCRIPTION: Create the Tx-Result object 
 * 
 * INPUTS:  hOs
 * 
 * OUTPUT:  None
 * 
 * RETURNS: The Created object
 ****************************************************************************/
TI_HANDLE txResult_Create(TI_HANDLE hOs)
{
    txResultObj_t *pTxResult;

    pTxResult = os_memoryAlloc(hOs, sizeof(txResultObj_t));
    if (pTxResult == NULL)
        return NULL;

    os_memoryZero(hOs, pTxResult, sizeof(txResultObj_t));

    pTxResult->hOs = hOs;

    return( (TI_HANDLE)pTxResult );
}


/****************************************************************************
 *                      txResult_Destroy()
 ****************************************************************************
 * DESCRIPTION: Destroy the Tx-Result object 
 * 
 * INPUTS:  hTxResult - The object to free
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
TI_STATUS txResult_Destroy(TI_HANDLE hTxResult)
{
    txResultObj_t *pTxResult = (txResultObj_t *)hTxResult;

    if (pTxResult)
        os_memoryFree(pTxResult->hOs, pTxResult, sizeof(txResultObj_t));

    return OK;
}


/****************************************************************************
 *               txResult_init()
 ****************************************************************************
   DESCRIPTION:  
   ============
     Initialize the txResult module.
 ****************************************************************************/
TI_STATUS txResult_init(TI_HANDLE hTxResult, TI_HANDLE hReport, TI_HANDLE hTNETWIF, TI_HANDLE hFwEvent)
{
    txResultObj_t *pTxResult = (txResultObj_t *)hTxResult;

    pTxResult->hReport = hReport;
    pTxResult->hTNETWIF = hTNETWIF;
    pTxResult->hFwEvent = hFwEvent;

    txResult_restart(pTxResult);

#ifdef TI_DBG
    os_memoryZero( pTxResult->hOs, &(pTxResult->txCompleteDepthHistogram), sizeof(UINT32) * FW_TX_CMPLT_BLOCK_SIZE );
#endif

    FwEvent_Enable(pTxResult->hFwEvent, ACX_INTR_TX_RESULT);

    return OK;
}


/****************************************************************************
 *               txResult_restart()
 ****************************************************************************
   DESCRIPTION:  
   ============
     Restarts the Tx-Result module.
     Should be called upon init and recovery!!
     Shouldn't be called upon disconnect, since the FW provides Tx-Complete
       for all pending packets in FW!!
 ****************************************************************************/
TI_STATUS txResult_restart(TI_HANDLE hTxResult)
{
    txResultObj_t *pTxResult = (txResultObj_t *)hTxResult;

    pTxResult->TxCmpltStartPointIterator = 0;
    
    return OK;
}


/****************************************************************************
 *                      txResult_setHwInfo()
 ****************************************************************************
 * DESCRIPTION:  
 *      Called after the HW configuration upon init or recovery.
 *      Store the Tx-result table HW address.
 *          
 * INPUTS:  
 *      hTxXfer             The object
 *      pDataPathParams     Pointer to the HW Addresses
 * 
 * OUTPUT:  None
 * 
 * RETURNS: None 
 ****************************************************************************/
void  txResult_setHwInfo(TI_HANDLE hTxResult, ACXDataPathParamsResp_t *pDataPathParams)
{
    txResultObj_t *pTxResult = (txResultObj_t *)hTxResult;

    pTxResult->txResultTableAddr = pDataPathParams->txCompleteAddr;

    /* Print of the Tx Result Table address */
    WLAN_REPORT_INFORMATION(pTxResult->hReport, TX_RESULT_MODULE_LOG, 
         ("Get Tx-Result-Table HW-Addr:  0x%x\n", pTxResult->txResultTableAddr));
} 


/****************************************************************************
 *                      txResult_TxCmpltIntrCB()
 ****************************************************************************
 * DESCRIPTION:   
 * ============
 *  Called upon Tx-complete interrupt from the FW.
 * 
 * INPUTS:  TI_HANDLE  hTxResult - the txResult object handle.
 *  
 * OUTPUT:  None
 * 
 * RETURNS: TNETWIF_OK in Synch mode or TNETWIF_PENDING on Asynch mode.                     
 ***************************************************************************/

#if FW_TX_CMPLT_BLOCK_SIZE & (FW_TX_CMPLT_BLOCK_SIZE - 1)
  #error "FW_TX_CMPLT_BLOCK_SIZE must be power of 2"
#endif

TI_STATUS txResult_TxCmpltIntrCB (TI_HANDLE hTxResult)
{
    txResultObj_t *pTxResult = (txResultObj_t *)hTxResult;

    if (TX_RESULT_STATE_IDLE != pTxResult->state)
    {
        WLAN_REPORT_ERROR(pTxResult->hReport,TX_RESULT_MODULE_LOG,
            ("rxXfer_RxEvent called in state %d !!!\n",pTxResult->state));
        return TNETWIF_ERROR;
    }
    
    /* assume that we are in synch bus until otherwise is proven */
    pTxResult->bSync = TRUE;

    /*                              0,OK has no meaning */
    txResult_StateMachine(hTxResult,0,OK);
    
    return pTxResult->returnValue;
}


/****************************************************************************
 *                      txResult_StateMachine()
 ****************************************************************************
 * DESCRIPTION:     main SM of the module.called in IDLE state by txResult_TxCmpltIntrCB() on 
 *                  Tx Complete interrupt from the FW.
 *                  Reads all Tx-Result cyclic table from the FW.
 *                  Goes over the valid entries (containing unread Tx-results) and calls the 
 *                  upper layer callback function for each packet with its results.
 *                  At the end - writes all new results back to the FW
 *                  The flow of the SM is by that order:
 *                  IDLE -> READING -> WRITING1 -> WRITING2 -> EXIT
 *                  On synch mode - each state is called in the same context in the while loop.
 *                  On Asynch mode - each state returns TNETWIF_PENDING and exits the SM.The CB of
 *                  each Asynch is the SM, that will continue the handling.
 *
 *  
 *          
 * INPUTS:  module_id   - not used (for prototype only).
 *          status      - not used (for prototype only).
 *
 * OUTPUT:  returnValue     - This parameter is used to indicate the FwEvent mosule about the status of
 *                              the client.if (returnValue == TNETWIF_OK) than client finished working (synch mode)
 *                              if (returnValue == TNETWIF_PENDING) than FwEvent module is waiting to be notified
 *                              that client finished the handling (Asynch mode)
 * 
 * RETURNS: None 
 ****************************************************************************/
static void txResult_StateMachine(TI_HANDLE hTxResult,UINT8 module_id ,TI_STATUS status)
{
    txResultObj_t *pTxResult = (txResultObj_t *)hTxResult;

    pTxResult->returnValue = OK;
 
     /* this while loop will continue till the exit or when waiting for the CB due to
        memory transfer operation pending for DMA to complete   */
    while (TNETWIF_PENDING != pTxResult->returnValue)
    {
        WLAN_REPORT_INFORMATION(pTxResult->hReport,TX_RESULT_MODULE_LOG,
            ("txResult SM: state = %d, rc = %d, Buffers = %d\n",
            pTxResult->state,pTxResult->returnValue,pTxResult->numOfBuffers));

        switch(pTxResult->state) 
        {
        case TX_RESULT_STATE_IDLE:
             /* Read all Tx-Result table from the FW (Synch or Asynch) */
            pTxResult->returnValue = TNETWIF_ReadMemOpt (pTxResult->hTNETWIF, 
                                                         pTxResult->txResultTableAddr, 
                                                         PADREAD (pTxResult->TxCmpltAttr), 
                                                         FW_TX_CMPLT_BLOCK_SIZE * sizeof(TxResultDescriptor_t),
                                                         FW_EVENT_MODULE_ID,
                                                         txResult_StateMachine,hTxResult);

            pTxResult->state = TX_RESULT_STATE_READING;
            break;

        case TX_RESULT_STATE_READING:
            /* process the new table and call the upper layers to handle results. 
               Also update numOfBuffers & entry (from, to) */
            txResult_handleNewEntries(pTxResult);

            if (TX_RESULT_NO_BUFFER == pTxResult->numOfBuffers) 
            {   /* no need to write to FW - exit SM */
                pTxResult->state = TX_RESULT_STATE_EXIT;
            }
            else
            {   
                pTxResult->state = TX_RESULT_STATE_WRITING1;
            }
            break;
        case TX_RESULT_STATE_WRITING1:

            pTxResult->returnValue = txResult_writeNewEntries(pTxResult,0);
            if (TX_RESULT_ONE_BUFFER == pTxResult->numOfBuffers) 
            {   /* only one write was needed - exit SM */
                pTxResult->state = TX_RESULT_STATE_EXIT;
            }
            else
            {
                pTxResult->state = TX_RESULT_STATE_WRITING2;
            }
            break;

        case TX_RESULT_STATE_WRITING2:

            pTxResult->returnValue = txResult_writeNewEntries(pTxResult,1);

            pTxResult->state = TX_RESULT_STATE_EXIT;
            break;

        case TX_RESULT_STATE_EXIT:

            if (FALSE == pTxResult->bSync)
            {   /* Async bus - call FwEvent for notifying the completion */
                FwEvent_EventComplete(pTxResult->hFwEvent, TNETWIF_OK);
            }
            else    /* This is the synch case - we should return TNETWIF_OK */
            {
                pTxResult->returnValue = TNETWIF_OK;
            }
            pTxResult->state = TX_RESULT_STATE_IDLE;

            return;

        default:
            WLAN_REPORT_ERROR(pTxResult->hReport,HAL_TX_MODULE_LOG,("rxXfer_StateMachine Unknown state = %d\n",
                pTxResult->state));
        }
    }

    /* if we are here - we got TNETWIF_PENDING, so we are in Async mode */
    pTxResult->bSync = FALSE;

    if (TNETWIF_ERROR == pTxResult->returnValue)
    {   
        WLAN_REPORT_ERROR(pTxResult->hReport,TX_RESULT_MODULE_LOG,
            ("txResult_StateMachine returning TNETWIF_ERROR in state %d !!!\n",pTxResult->state));
    }
}


/****************************************************************************
 *                      txResult_handleNewEntries()
 ****************************************************************************
 * DESCRIPTION:   
 * ============
 *      Goes over the valid entries (containing unread Tx-results) and calls the 
 *      upper layer callback function for each packet with its results.
 *
 * INPUTS:  TI_HANDLE  hTxResult - the txResult object handle.
 *  
 * OUTPUT:  1) number of buffers to write on .  (The case of 2 buffers can happen
 *          in case of wrap around - and we need 2 different writes to FW)
 *
 *          2) start-end address of the buffers to be written
 * 
 * RETURNS: 
 ***************************************************************************/
static void txResult_handleNewEntries(txResultObj_t *pTxResult)
{   
    TxResultDescriptor_t *pCurrentEntry; /* Points to the current table entry */                       
    UINT32 uIndex;                       /* The current table entry */
    UINT32 uNumOfTxComplete;             /* Counts contiguous valid entries (i.e. waiting for host read) */
    UINT32 uLoopCount;                   /* Used only to prevent endless loop */

    uIndex = pTxResult->TxCmpltStartPointIterator;
    uNumOfTxComplete = 0;

    /* Begin the loop only from the point where the last Tx Complete was received before */
    for (uLoopCount = uNumOfTxComplete = 0; uLoopCount < FW_TX_CMPLT_BLOCK_SIZE; uLoopCount ++)
    {
        /*
         * Update current entry.
         * Take into account that uIndex may be 16, so make & 0xf
         */
        pCurrentEntry = &pTxResult->TxCmpltAttr[uIndex & (FW_TX_CMPLT_BLOCK_SIZE - 1)];

        WLAN_REPORT_INFORMATION(pTxResult->hReport, TX_RESULT_MODULE_LOG,  
            ("Tx Result Entry %d: Done1/2=%d/%d, DescID=%d, Status=%d, Rate=%d, Duration=%d, Retries=%d, HandleTime=%d, SecurNum=%d\n", 
            uIndex, pCurrentEntry->done1, pCurrentEntry->done2, pCurrentEntry->descID, pCurrentEntry->status, 
            pCurrentEntry->actualRate, pCurrentEntry->mediumUsage, pCurrentEntry->ackFailures, 
            pCurrentEntry->fwHandlingTime, pCurrentEntry->lsbSecuritySequenceNumber));

        /* If the current entry contains fresh Tx-result information */
        if (pCurrentEntry->done1 == 1 && pCurrentEntry->done2 == 1)
        {
            /* Call GWSI Tx-complete callback with current entry pointer. */
            /* It is assumed that the entry is only accessed in this context and only for reading. */
            pTxResult->sendPacketCompleteCB (pTxResult->sendPacketCompleteHandle, pCurrentEntry);
            
            /* Clear entry */
            pCurrentEntry->done1 = 0;
            pCurrentEntry->done2 = 0;

            /* Increment the index to point to next entry (wrap around is handled below) */
            if (uIndex >= FW_TX_CMPLT_BLOCK_SIZE) 
                uIndex = 1;
            else
                uIndex ++;

            uNumOfTxComplete ++;
        }
        else
            break;
    }

#ifdef TI_DBG
    /* Update the TX result depth histogram */
    pTxResult->txCompleteDepthHistogram [uNumOfTxComplete] ++;
#endif
    
    /* Copy the handled entries back to FW to clear them */
    if (uNumOfTxComplete)
    {
        /* No wrap. Make only 1 write */
        if (uIndex > pTxResult->TxCmpltStartPointIterator)
        {
            pTxResult->entry[0].from = pTxResult->TxCmpltStartPointIterator;
            pTxResult->entry[0].to = uIndex;
            pTxResult->numOfBuffers = TX_RESULT_ONE_BUFFER;
        }
        /* Wrap. Make 2 writes */
        else if (uIndex < pTxResult->TxCmpltStartPointIterator)
        {
            pTxResult->entry[0].from = pTxResult->TxCmpltStartPointIterator;
            pTxResult->entry[0].to = FW_TX_CMPLT_BLOCK_SIZE;
            pTxResult->entry[1].from = 0; 
            pTxResult->entry[1].to = uIndex; 
            pTxResult->numOfBuffers = TX_RESULT_TWO_BUFFERS;
        }
        /* Wrap, all 16 descriptors are filled. Make 1 write from index 0 */
        else
        {        
            pTxResult->entry[0].from = 0;
            pTxResult->entry[0].to = FW_TX_CMPLT_BLOCK_SIZE;
            pTxResult->numOfBuffers = TX_RESULT_ONE_BUFFER;
        }

    }
    else /* no new entry - no need to write buffers */
    {
        pTxResult->numOfBuffers = TX_RESULT_NO_BUFFER;
    }
    /*
     * Update start point iterator.
     * Take into account that uIndex may be 16, so make & 0xf
     */
    pTxResult->TxCmpltStartPointIterator = uIndex & (FW_TX_CMPLT_BLOCK_SIZE - 1);
}


/****************************************************************************
 *                      txResult_writeNewEntries()
 ****************************************************************************
 * DESCRIPTION:   
 * ============
 *              Clears the read entries in the FW. 
 *              
 *
 * INPUTS:  pTxResult - the txResult object handle.
 *          currBuffer - number of buffer to write on (0 or 1)
 *  
 * OUTPUT:  None
 * 
 * RETURNS: TNETWIF_OK in case of Synch call, TNETWIF_PENDING in case of Asynch call
 *
 * NOTE: please note that we are running over the last WORD before the first entry
 *          since we need to save this place for the bus handling
 ***************************************************************************/
static TI_STATUS txResult_writeNewEntries(txResultObj_t *pTxResult,UINT8 currBuffer)
{
    /* 
     * Write to firmware - NOTE: that we are running over the last WORD before the first entry
     * since we need to save this place for the bus handling
     */
    return TNETWIF_WriteMemOpt (pTxResult->hTNETWIF,
                                pTxResult->txResultTableAddr + pTxResult->entry[currBuffer].from * sizeof(TxResultDescriptor_t),
                                PADWRITE (&pTxResult->TxCmpltAttr[pTxResult->entry[currBuffer].from]),
                                (pTxResult->entry[currBuffer].to - pTxResult->entry[currBuffer].from) * sizeof(TxResultDescriptor_t),
                                FW_EVENT_MODULE_ID,
                                txResult_StateMachine,
                                (TI_HANDLE)pTxResult);

}

/****************************************************************************
 *                      txResult_RegisterCB()
 ****************************************************************************
 * DESCRIPTION:  Register the upper driver Tx-Result callback functions.
 ****************************************************************************/
void txResult_RegisterCB(TI_HANDLE hTxResult, tiUINT32 CallBackID, void *CBFunc, TI_HANDLE CBObj)
{
    txResultObj_t* pTxResult = (txResultObj_t*)hTxResult;

    switch(CallBackID)
    {
        /* Set Tx-Complete callback */
        case TX_RESULT_SEND_PKT_COMPLETE:
            pTxResult->sendPacketCompleteCB = (SendPacketCompleteCB_t)CBFunc;
            pTxResult->sendPacketCompleteHandle = CBObj;
            break;

        default:
            WLAN_REPORT_ERROR(pTxResult->hReport, TX_RESULT_MODULE_LOG, ("txResult_RegisterCB - Illegal value\n"));
            return;
    }
}


/****************************************************************************
 *                      txResult_RegisterCB()
 ****************************************************************************
 * DESCRIPTION:  Prints TX result debig information.
 ****************************************************************************/
void txResult_printInfo(TI_HANDLE hTxResult)
{
#ifdef TI_DBG
    txResultObj_t* pTxResult = (txResultObj_t*)hTxResult;

    WLAN_OS_REPORT(("Tx-Result Module Information:\n"));
    WLAN_OS_REPORT(("=============================\n\n"));

    WLAN_OS_REPORT(("        FW result array depth histogram:\n"));
    WLAN_OS_REPORT(("        --------------------------------\n\n"));
    WLAN_OS_REPORT((" depth: %8d %8d %8d %8d %8d %8d\n", 0, 1, 2, 3, 4, 5));
    WLAN_OS_REPORT((" INTR:  %8d %8d %8d %8d %8d %8d\n\n",
                    pTxResult->txCompleteDepthHistogram[ 0 ],
                    pTxResult->txCompleteDepthHistogram[ 1 ],
                    pTxResult->txCompleteDepthHistogram[ 2 ],
                    pTxResult->txCompleteDepthHistogram[ 3 ],
                    pTxResult->txCompleteDepthHistogram[ 4 ],
                    pTxResult->txCompleteDepthHistogram[ 5 ]));
    WLAN_OS_REPORT((" depth: %8d %8d %8d %8d %8d %8d\n", 6, 7, 8, 9, 10, 11));
    WLAN_OS_REPORT((" INTR:  %8d %8d %8d %8d %8d %8d\n\n",
                    pTxResult->txCompleteDepthHistogram[ 6 ],
                    pTxResult->txCompleteDepthHistogram[ 7 ],
                    pTxResult->txCompleteDepthHistogram[ 8 ],
                    pTxResult->txCompleteDepthHistogram[ 9 ],
                    pTxResult->txCompleteDepthHistogram[ 10 ],
                    pTxResult->txCompleteDepthHistogram[ 11 ]));
    WLAN_OS_REPORT((" depth: %8d %8d %8d %8d %8d\n", 12, 13, 14, 15, 16));
    WLAN_OS_REPORT((" INTR:  %8d %8d %8d %8d %8d\n\n",
                    pTxResult->txCompleteDepthHistogram[ 12 ],
                    pTxResult->txCompleteDepthHistogram[ 13 ],
                    pTxResult->txCompleteDepthHistogram[ 14 ],
                    pTxResult->txCompleteDepthHistogram[ 15 ],
                    pTxResult->txCompleteDepthHistogram[ 16 ]));
#endif
}

/****************************************************************************
 *                      txResult_RegisterCB()
 ****************************************************************************
 * DESCRIPTION:  Prints TX result debig information.
 ****************************************************************************/
void txResult_clearInfo(TI_HANDLE hTxResult)
{
#ifdef TI_DBG
    txResultObj_t* pTxResult = (txResultObj_t*)hTxResult;

    os_memoryZero( pTxResult->hOs, pTxResult->txCompleteDepthHistogram, sizeof(UINT32) * FW_TX_CMPLT_BLOCK_SIZE );
#endif
}

