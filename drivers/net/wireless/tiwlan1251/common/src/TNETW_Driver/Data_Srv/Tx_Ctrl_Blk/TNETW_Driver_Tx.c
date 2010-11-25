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
 *   MODULE:	TNETW_Driver_Tx.c
 *
 *   PURPOSE:	TNETW_Driver Tx API functions needed externally to the driver.
 *
 ****************************************************************************/

#include "whalParams.h"
#include "report.h"
#include "TNETW_Driver_api.h"
#include "txCtrlBlk_api.h"
#include "txHwQueue_api.h"
#include "txXfer_api.h"
#include "txResult_api.h"
#include "TNETW_Driver.h"


static systemStatus_e ConvertTxResultStatus(TxDescStatus_enum txResultStatus);


/****************************************************************************
 *					Tx Control Block API functions							*
 ****************************************************************************/

txCtrlBlkEntry_t *TnetwDrv_txCtrlBlk_alloc(TI_HANDLE hTnetwDrv)
{
	TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;

	return txCtrlBlk_alloc(pTnetwDrv->hTxCtrlBlk);
}


void TnetwDrv_txCtrlBlk_free(TI_HANDLE hTnetwDrv, txCtrlBlkEntry_t *pCurrentEntry)
{
	TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;

	txCtrlBlk_free(pTnetwDrv->hTxCtrlBlk, pCurrentEntry);
}


txCtrlBlkEntry_t *TnetwDrv_txCtrlBlk_GetPointer(TI_HANDLE hTnetwDrv, UINT8 descId)
{
	TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;

	return txCtrlBlk_GetPointer(pTnetwDrv->hTxCtrlBlk, descId);
}



/****************************************************************************
 *						Tx HW Queue API functions							*
 ****************************************************************************/

TI_STATUS  TnetwDrv_txHwQueue_alloc(TI_HANDLE hTnetwDrv, txCtrlBlkEntry_t *pPktCtrlBlk)
{
	TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;

	return txHwQueue_alloc(pTnetwDrv->hTxHwQueue, pPktCtrlBlk);
}


TI_STATUS  TnetwDrv_txHwQueue_free(TI_HANDLE hTnetwDrv, txCtrlBlkEntry_t *pPktCtrlBlk)
{
	TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;

	return txHwQueue_free(pTnetwDrv->hTxHwQueue, pPktCtrlBlk);
}


UINT8  TnetwDrv_txHwQueue_GetUsedHwBlks(TI_HANDLE hTnetwDrv, int TxQid)
{
	TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;

	return txHwQueue_GetUsedHwBlks(pTnetwDrv->hTxHwQueue, TxQid);
}



/****************************************************************************
 *							Tx Xfer API functions							*
 ****************************************************************************/

systemStatus_e	TnetwDrv_txXfer_sendPacket(TI_HANDLE hTnetwDrv, 
				const void	*aFrame,		/* Pointer to the packet content. points to */
                                            /* the place that the actual packet begins. */
                                            /* a size of TX_TOTAL_OFFSET_BEFORE_DATA    */
                                            /* must be saved before that pointer        */
				UINT16		aLength,		/* MSDU length from first byte of MAC		*/
											/*   header to last byteof frame body.		*/
				UINT8		aQueueId,		/* Tx queue as defined in ConfigureQueue.	*/
				UINT8		aTxRateClassId,	/* Tx rate class ID	defined in txRatePolicy.*/
				UINT16		aMaxTransmitRate,/* A bit mask that specifies the initial	*/
											/*     (highest) rate to use.				*/
				BOOL		aMore,			/* Tells if there is another packet	coming	*/
											/*   shortly after this one.				*/
				UINT32		aPacketId,		/* Packet identifier used as a context by	*/
											/*   the host driver.					*/
				UINT8	       aPowerLevel,	/* Transmission power level.				*/
				UINT32		aExpiryTime,	/* Time left for this MSDU to live.			*/
				void	   	     *aReserved)  	/* Optional parameters pointer.				*/

{
	TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;
	TxDescCtrl_t txAttr, *pTxAttr = &txAttr;  /* A union for setting the txAttr bit fields. */
    dot11_header_t      *pDot11Hdr;
	systemStatus_e      status;
	txCtrlBlkEntry_t    *pPktCtrlBlk;
    WhalParams_T *pWhalParams = (WhalParams_T *)(pTnetwDrv->hWhalParams);
    BOOL                bIsMultiCastAndIBSS; /* used for the Ack policy */
	
#ifdef TI_DBG
	/* If queue number is invalid return ERROR. */
	if (aQueueId >= MAX_NUM_OF_TX_QUEUES)
	{
		WLAN_REPORT_ERROR(pTnetwDrv->hReport, TNETW_DRV_MODULE_LOG,
			("TnetwDrv_txXfer_sendPacket(): Invalid aQueueId = %d !!!\n", aQueueId));
		return SEND_PACKET_ERROR;
	}
#endif

	/*****************************************************************************************/
	/*  1) Allocates a Control-Block for the packet Tx parameters and descriptor.            */
	/*****************************************************************************************/

	pPktCtrlBlk = TnetwDrv_txCtrlBlk_alloc(pTnetwDrv);

#ifdef TI_DBG
	pTnetwDrv->dbgCountSentPackets[aQueueId]++;	/* Count packets sent from upper driver. */
#endif	
	/* If null entry (not expected to happen) return ERROR. */
	if (!pPktCtrlBlk)
	{
#ifdef TI_DBG
		WLAN_REPORT_ERROR(pTnetwDrv->hReport, TNETW_DRV_MODULE_LOG,
			("TnetwDrv_txXfer_sendPacket(): Tx Ctrl-Blk allocation failed!!!\n"));
#endif
		return SEND_PACKET_ERROR;
	}
	
	/*****************************************************************************************/
	/*  2) Set the packet control block parameters (including descriptor structure).         */
	/*****************************************************************************************/
	
	/* Note that the following params are currently not used:	aMore, aPowerLevel, aReserved. */
	
	/* Note that the following descriptor params are for future use, so they are left zeroed:
		 pktType and xferPadding fields in txAttr, and tid. */

	/* Note that the other params are set later in the Tx process (e.g. fragThresh and numMemBlks). */

    /* aFrame points to the start of the data, but we need to reserve place for descriptor + TNETWIF_WRITE_OFFSET_BYTES */
	pPktCtrlBlk->txPktParams.pFrame = (void*)((UINT8 *)aFrame - TX_TOTAL_OFFSET_BEFORE_DATA);

	pPktCtrlBlk->txDescriptor.length = aLength;
	pPktCtrlBlk->txDescriptor.xmitQueue = aQueueId;
	pPktCtrlBlk->txDescriptor.rate = aMaxTransmitRate;
	pPktCtrlBlk->txPktParams.packetId = aPacketId;
	pPktCtrlBlk->txDescriptor.expiryTime = aExpiryTime << SHIFT_BETWEEN_TU_AND_USEC;  /* Convert TUs to microseconds. */

	pDot11Hdr = (dot11_header_t*)(aFrame); /* aFrame points to the start of the data */
	pPktCtrlBlk->txPktParams.headerFrameCtrl = pDot11Hdr->fc;  /* Save frame-control field from MAC header. */

	/* Set descriptor control bit-mask fields and write the whole word to the descriptor. */
	*(uint16 *)pTxAttr = 0; /* Clear temporary union. */
	txAttr.ratePolicy = aTxRateClassId;

    /* Configure the Ack policy */
    bIsMultiCastAndIBSS = ((BSS_INDEPENDENT == (bssType_e)(whal_ParamsGetReqBssType(pWhalParams)))
                            && (MAC_MULTICAST(GET_DA_FROM_DOT11_HEADER_T(pDot11Hdr))));
    txAttr.ackPolicy = TnetwDrv_txGetAckPolicy(pTnetwDrv, aQueueId , bIsMultiCastAndIBSS);

	if (IS_QOS_FRAME(pDot11Hdr->fc))  /* Check if frame is QoS-Data or QoS-Null. */
		txAttr.qosFrame = 1;

	/* if this is a management frame, request immediate TX complete indication */
	if ( (pDot11Hdr->fc & DOT11_FC_TYPE_MASK) == DOT11_FC_TYPE_MGMT )
	{
		txAttr.txCmpltRequired = 1;
	}

	pPktCtrlBlk->txDescriptor.txAttr = *(uint16 *)pTxAttr; 

	pPktCtrlBlk->txPktParams.flags = 0; 


	/************************************************************************************************/
	/*  3) Call HwQueue for Hw resources allocation. If not available free CtrlBlk and return BUSY. */
	/************************************************************************************************/

	/* Note that the HwQueue calls first the fragThreshold and numMemBlks calculation. */

	if ( TnetwDrv_txHwQueue_alloc(pTnetwDrv, pPktCtrlBlk) != OK )
	{
		TnetwDrv_txCtrlBlk_free(pTnetwDrv, pPktCtrlBlk);
		return SEND_PACKET_BUSY;
	}

#ifdef TI_DBG  
	/* Just for debug, write per queue sequence number to packet ctrl-blk (after allocation success!). */
	pTnetwDrv->dbgPktSeqNum[aQueueId]++;
	pPktCtrlBlk->txPktParams.dbgPktSeqNum = pTnetwDrv->dbgPktSeqNum[aQueueId];

	pTnetwDrv->dbgCountQueueAvailable[aQueueId]++;	/* Count packets sent and queue not busy. */
#endif
	

	/*****************************************************************************************/
	/*  4) Copy the descriptor to the frame .                      */
	/*****************************************************************************************/

    os_memoryCopy(pTnetwDrv->hOs, (void *)((UINT8*)aFrame - sizeof(DbTescriptor)), &(pPktCtrlBlk->txDescriptor), sizeof(DbTescriptor));


	/*****************************************************************************************/
	/*  5) Call the Tx-Xfer to start packet transfer to the FW and return its result.        */
	/*****************************************************************************************/
	
	status = txXfer_sendPacket(pTnetwDrv->hTxXfer, pPktCtrlBlk);
	
	/* If the packet was transfered in this context and Tx-complete already occured, free the ctrl-blk. */
	if (status == SEND_PACKET_XFER_DONE)
	{
		if (pPktCtrlBlk->txPktParams.flags & TX_CTRL_BLK_FLAGS_TX_COMPLETE_ISSUED)
			TnetwDrv_txCtrlBlk_free(pTnetwDrv, pPktCtrlBlk);
		else
			pPktCtrlBlk->txPktParams.flags |= TX_CTRL_BLK_FLAGS_XFER_DONE_ISSUED;
	}

#ifdef TI_DBG  
	if (status == SEND_PACKET_XFER_DONE)
		pTnetwDrv->dbgCountXferDone[aQueueId]++;	
	else if (status == SEND_PACKET_SUCCESS)
		pTnetwDrv->dbgCountXferSuccess[aQueueId]++;	
	else if (status == SEND_PACKET_PENDING)
		pTnetwDrv->dbgCountXferPending[aQueueId]++;	
	else
		pTnetwDrv->dbgCountXferError[aQueueId]++;	
#endif
	

	return status;
}




/****************************************************************************
 *				Tx API functions needed for GWSI interface					*
 ****************************************************************************/

UINT8  TnetwDrv_txGetAckPolicy(TI_HANDLE hTnetwDrv, int TxQid ,  BOOL bIsMultiCastAndIBSS)
{
	TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;
	WhalParams_T *pWhalParams = (WhalParams_T *)(pTnetwDrv->hWhalParams);

    /* If we are in IBSS and we are transmitting a Multicast/Broadcast frame -> we don't expect an Ack packet */
    if (bIsMultiCastAndIBSS)
    {
        return ACK_POLICY_NO_ACK;
    }

	return (pWhalParams->QueuesParams.queues[TxQid].ackPolicy);
}

/*************************************************************************
*                        TnetwDrv_TxXferDone                                 *
**************************************************************************
* DESCRIPTION:  
  ============
	Called  upon Xfer-Done of transmitted packet.
	Calls the upper driver's Xfer-Done handler.

*
* INPUT:     hDummyHandle - Just to support the CB API.
*			 pPktCtrlBlk -  The packet's control block pointer.  
*              
* OUTPUT:
*
* RETURN:       

*************************************************************************/
void TnetwDrv_TxXferDone(TI_HANDLE hTnetwDrv, txCtrlBlkEntry_t *pPktCtrlBlk)
{
	/* Make a working copy of TNETDriver Handle */
	TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;


#ifdef TI_DBG
	pTnetwDrv->dbgCountXferDoneCB[pPktCtrlBlk->txDescriptor.xmitQueue]++;
#endif	
	/* If the pointed entry is already free, print error and exit (not expected to happen). */
	if (pPktCtrlBlk->pNextFreeEntry != NULL)
	{
#ifdef TI_DBG	
		WLAN_REPORT_ERROR(pTnetwDrv->hReport, TNETW_DRV_MODULE_LOG,
			("TnetwDrv_TxXferDone(): Pkt already free!!, DescID=%d, dbgPktSeqNum=%d, flags=%d, packetId=0x%x, Queue=%d\n", 
			pPktCtrlBlk->txDescriptor.descID, pPktCtrlBlk->txPktParams.dbgPktSeqNum, pPktCtrlBlk->txPktParams.flags, 
			pPktCtrlBlk->txPktParams.packetId, pPktCtrlBlk->txDescriptor.xmitQueue));
#endif			
		return;
	}

	/* If Tx-complete already occurred, free the ctrl-blk. */
	/* Note that this may happen when the Xfer-SM delays the Xfer-Done (for pipeline sequence). */
	if (pPktCtrlBlk->txPktParams.flags & TX_CTRL_BLK_FLAGS_TX_COMPLETE_ISSUED)
		TnetwDrv_txCtrlBlk_free(pTnetwDrv, pPktCtrlBlk);
	else
		pPktCtrlBlk->txPktParams.flags |= TX_CTRL_BLK_FLAGS_XFER_DONE_ISSUED;
	
	/* Call the upper driver's Xfer-Done handler with the packet-ID. */
	/* Note that depending on the type of compilation, the upper layers vary: */
	/* It may be either CoreAdaptTx or GWSIAdaptTx */
	/* Both functions are called the same */
	SendPacketTransfer (pTnetwDrv->hUser, pPktCtrlBlk->txPktParams.packetId);
}



/*************************************************************************
*                        TnetwDrv_TxComplete                                 *
**************************************************************************
* DESCRIPTION:  
  ============
	Called upon Tx-complete of transmitted packet.
	Handles it as follows:
	1) Update the HwQueue to free queue resources.
	2) Call the upper driver's tx-complete handler.
	3) Free the packet's Control-Block if Xfer-Done already occured.

*
* INPUT:   hDummyHandle -  Just to support the CB API.
*		   pTxResultInfo - The packet's Tx result information.    
*              
* OUTPUT:
*
* RETURN:       

*************************************************************************/
void TnetwDrv_TxComplete(TI_HANDLE hTnetwDrv, TxResultDescriptor_t *pTxResultInfo)
{
	txCtrlBlkEntry_t *pPktCtrlBlk;
	/* Make a working copy of TNETDriver Handle */
	TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;

	/* Get the packet's control block pointer by the descId index. */
	pPktCtrlBlk = TnetwDrv_txCtrlBlk_GetPointer(pTnetwDrv, pTxResultInfo->descID);

#ifdef TI_DBG
	WLAN_REPORT_INFORMATION(pTnetwDrv->hReport, TNETW_DRV_MODULE_LOG,
		("TnetwDrv_TxComplete(): DescID=%d, dbgPktSeqNum=%d, flags=%d, packetId=0x%x, Queue=%d\n", 
		pTxResultInfo->descID, pPktCtrlBlk->txPktParams.dbgPktSeqNum, pPktCtrlBlk->txPktParams.flags, 
		pPktCtrlBlk->txPktParams.packetId, pPktCtrlBlk->txDescriptor.xmitQueue));

	pTnetwDrv->dbgCountTxCompleteCB[pPktCtrlBlk->txDescriptor.xmitQueue]++;
#endif	
	/* If the pointed entry is already free, print error and exit (not expected to happen). */
	if (pPktCtrlBlk->pNextFreeEntry != NULL)
	{
#ifdef TI_DBG	
		WLAN_REPORT_ERROR(pTnetwDrv->hReport, TNETW_DRV_MODULE_LOG,
			("TnetwDrv_TxComplete(): Pkt already free!!, DescID=%d, dbgPktSeqNum=%d, flags=%d, packetId=0x%x, Queue=%d\n", 
			pTxResultInfo->descID, pPktCtrlBlk->txPktParams.dbgPktSeqNum, pPktCtrlBlk->txPktParams.flags, 
			pPktCtrlBlk->txPktParams.packetId, pPktCtrlBlk->txDescriptor.xmitQueue));
#endif			
		return;
	}

	/* Update the HwQueue to free queue resources. */
	TnetwDrv_txHwQueue_free(pTnetwDrv, pPktCtrlBlk);

	/* @@@ Note: Add Security Sequence Number handling. */
	/* Update the TKIP/AES sequence-number according to the Tx data packet security-seq-num. */
	/* Note: The FW always provides the last used seq-num so no need to check if the current 
			 packet is data and WEP is on. */
	whalCtrl_updateSecuritySeqNum(pTnetwDrv->hHalCtrl, pTxResultInfo->lsbSecuritySequenceNumber);

	/* Call the upper driver's tx-complete handler. */
	/* Note that depending on the type of compilation, the upper layeres varry: */
	/* It may be either CoreAdaptTx or GWSIAdaptTx */
	/* Both functions are called the same */
	SendPacketComplete (pTnetwDrv->hUser,
                        ConvertTxResultStatus((TxDescStatus_enum)pTxResultInfo->status), 
					    pPktCtrlBlk->txPktParams.packetId, 
					    pTxResultInfo->actualRate,
					    pTxResultInfo->ackFailures,
					    (UINT32)pTxResultInfo->mediumUsage,
					    pTxResultInfo->fwHandlingTime,
					    pTxResultInfo->mediumDelay);

	/* If Xfer-Done already occured, free the ctrl-blk (otherwise Xfer-Done will do it). */
	/* Note that the Xfer-SM may delay the Xfer-Done (for pipeline sequence). */
	if (pPktCtrlBlk->txPktParams.flags & TX_CTRL_BLK_FLAGS_XFER_DONE_ISSUED)
		TnetwDrv_txCtrlBlk_free(pTnetwDrv, pPktCtrlBlk);
	else
		pPktCtrlBlk->txPktParams.flags |= TX_CTRL_BLK_FLAGS_TX_COMPLETE_ISSUED;
}


/*************************************************************************
*                        TnetwDrv_RecoveryCtrlBlk                        *
**************************************************************************
* DESCRIPTION:  
  ============
	Called upon recovery.
	Handles it as follows:
	1) Update the HwQueue to free queue resources.
	3) Free the packet's Control-Block if Xfer-Done already occured.

*
* INPUT:   hDummyHandle -  Just to support the CB API.
*              
* OUTPUT:
*
* RETURN:       

*************************************************************************/
void TnetwDrv_RecoveryCtrlBlk(TI_HANDLE hTnetwDrv)
{
	txCtrlBlkEntry_t *pPktCtrlBlk;
	/* Make a working copy of TNETDriver Handle */
	TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;
	UINT32 entry;
    const UINT32 MAX_CTRL_BLK_ENTRY = 64;

	for (entry = 0; entry < MAX_CTRL_BLK_ENTRY-1; entry++)
	{
		/* Get the packet's control block pointer by the descId index. */
		pPktCtrlBlk = TnetwDrv_txCtrlBlk_GetPointer(pTnetwDrv, entry);
		if (pPktCtrlBlk->pNextFreeEntry == NULL)
		{
			TnetwDrv_txHwQueue_free(pTnetwDrv, pPktCtrlBlk);
			TnetwDrv_txCtrlBlk_free(pTnetwDrv, pPktCtrlBlk);
		}
	}
}


/*************************************************************************
*                        TnetwDrv_TxXferDebug                                *
**************************************************************************
* DESCRIPTION:  
  ============
    Called upon issuing interrupt to firmware.
    Calls the upper driver's Xfer-Debug handler.

*
* INPUT:     hDummyHandle - Just to support the CB API.
*            pPktCtrlBlk  - The packet's control block pointer.  
             uDebugInfo   - Additional debug info
*              
* OUTPUT:
*
* RETURN:       

*************************************************************************/
#ifdef TI_DBG
void TnetwDrv_TxXferDebug (TI_HANDLE hTnetwDrv, txCtrlBlkEntry_t *pPktCtrlBlk, UINT32 uDebugInfo)
{   
    TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;
       /* Call the upper driver's Xfer-Done handler with the packet-ID. */
	/* Note that depending on the type of compilation, the upper layeres varry: */
	/* It may be either CoreAdaptTx or GWSIAdaptTx */
	/* Both functions are called the same */
    SendPacketDebug (pTnetwDrv->hUser, pPktCtrlBlk->txPktParams.packetId, uDebugInfo);
}
#endif


/****************************************************************************
 *                      ConvertTxResultStatus
 ****************************************************************************
 * DESCRIPTION: Convert the status bit field in the TxDone descriptor, indexed
 *				by the given index, to a driver status bit field
 *			
 * INPUTS:	txResultStatus - Status value received from the FW
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	The converted value 
 ****************************************************************************/
systemStatus_e ConvertTxResultStatus(TxDescStatus_enum txResultStatus)
{
	/* Convert Tx-Result from FW to GWSI values. */
	/* Note: only 1 bit in the entire status field should be set */
	switch (txResultStatus)
	{
		case TX_SUCCESS:
			return SEND_COMPLETE_SUCCESS;

		case TX_RETRY_EXCEEDED:
			return SEND_COMPLETE_RETRY_EXCEEDED;

		case TX_TIMEOUT:
			return SEND_COMPLETE_LIFETIME_EXCEEDED;

		default:
			return SEND_COMPLETE_NO_LINK;
	}
}


/****************************************************************************
 *                      TnetwDrv_printInfo()
 ****************************************************************************
 * DESCRIPTION:	 Print the txXfer object main fields.
 ****************************************************************************/
void TnetwDrv_printInfo(TI_HANDLE hTnetwDrv)
{
#ifdef TI_DBG
    TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;
    int qIndex;

    WLAN_OS_REPORT(("TNETW Driver  Tx Counters per Queue:\n"));
    WLAN_OS_REPORT(("===========================\n"));

    WLAN_OS_REPORT(("-------------- packets sent from upper driver ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTnetwDrv->dbgCountSentPackets[qIndex]));

    WLAN_OS_REPORT(("-------------- packets sent and queue not busy ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTnetwDrv->dbgCountQueueAvailable[qIndex]));

    WLAN_OS_REPORT(("-------------- XferDone value returned from Xfer ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTnetwDrv->dbgCountXferDone[qIndex]));

    WLAN_OS_REPORT(("-------------- Success value returned from Xfer ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTnetwDrv->dbgCountXferSuccess[qIndex]));

    WLAN_OS_REPORT(("-------------- Pending value returned from Xfer ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTnetwDrv->dbgCountXferPending[qIndex]));

    WLAN_OS_REPORT(("-------------- Error value returned from Xfer ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTnetwDrv->dbgCountXferError[qIndex]));

    WLAN_OS_REPORT(("-------------- XferDone callback calls ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTnetwDrv->dbgCountXferDoneCB[qIndex]));

    WLAN_OS_REPORT(("-------------- TxComplete callback calls ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTnetwDrv->dbgCountTxCompleteCB[qIndex]));
#endif /* TI_DBG */
}
