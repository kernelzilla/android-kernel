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
 *   MODULE:  txHwQueue.c
 *   
 *   PURPOSE: manage the wlan hardware Tx memory blocks allocation per queue. 
 * 
 *	 DESCRIPTION:  
 *   ============
 *		This module is responsible for the HW Tx resources allocation (except
 *		  the HW double buffer).
 *		The HW Tx resources are allocated and freed in the driver by pure
 *		  calculations without accessing the FW. This is done by tracking the 
 *		resources allocation and freeing, and checking against thresholds before 
 *		  each allocation.
 ****************************************************************************/

#ifdef _WINDOWS
#endif 

#include "whalCommon.h"
#include "whalHwDefs.h"
#include "whalCtrl_api.h"
#include "whalParams.h"
#include "txCtrlBlk_api.h"
#include "txHwQueue_api.h"

#include "txHwQueue.h"  /* Local definitions */





/****************************************************************************
 *                      txHwQueue_Create()
 ****************************************************************************
 * DESCRIPTION:	Create the Tx buffers pool object 
 * 
 * INPUTS:	None
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	The Created object
 ****************************************************************************/
TI_HANDLE txHwQueue_Create(TI_HANDLE hOs)
{
	TxHwQueueObj_t *pTxHwQueue;

	pTxHwQueue = os_memoryAlloc(hOs, sizeof(TxHwQueueObj_t));
	if (pTxHwQueue == NULL)
		return NULL;

	os_memoryZero(hOs, pTxHwQueue, sizeof(TxHwQueueObj_t));

	pTxHwQueue->hOs = hOs;

	return( (TI_HANDLE)pTxHwQueue );
}

/****************************************************************************
 *                      txHwQueue_Destroy()
 ****************************************************************************
 * DESCRIPTION:	Destroy the Tx buffers pool object 
 * 
 * INPUTS:	hTxHwQueue - The object to free
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
TI_STATUS txHwQueue_Destroy(TI_HANDLE hTxHwQueue)
{
	TxHwQueueObj_t *pTxHwQueue = (TxHwQueueObj_t *)hTxHwQueue;

	if (pTxHwQueue)
		os_memoryFree(pTxHwQueue->hOs, pTxHwQueue, sizeof(TxHwQueueObj_t));

	return OK;
}




/****************************************************************************
 *               txHwQueue_init()
 ****************************************************************************

  DESCRIPTION:	Initialize module handles.

 ****************************************************************************/
TI_STATUS txHwQueue_init(TI_HANDLE hTxHwQueue, TI_HANDLE hReport, TI_HANDLE hWhalParams)
{
	TxHwQueueObj_t *pTxHwQueue = (TxHwQueueObj_t *)hTxHwQueue;
	
	pTxHwQueue->hReport = hReport;
	pTxHwQueue->pWhalParams = (WhalParams_T *)hWhalParams;

	return OK;
}

/****************************************************************************
 *                      txHwQueue_Config()
 ****************************************************************************
 * DESCRIPTION:	Configure the Tx buffers pool object 
 * 
 * INPUTS:	None
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	
 ****************************************************************************/
TI_STATUS txHwQueue_Config(TI_HANDLE hTxHwQueue, TnetwDrv_InitParams_t *pInitParams)
{
	UINT8			acID;
	
	/* Configure queue parameters to Tx-HW queue module */
	for(acID = 0 ; acID < MAX_NUM_OF_AC ; acID++)
	{
		txHwQueue_configQueue(	hTxHwQueue, 
								acID, 
								pInitParams->whalCtrl_init.TxBlocksLowPercentPerAc[acID],
								pInitParams->whalCtrl_init.TxBlocksHighPercentPerAc[acID]);
	}
	
	return OK;
}


/****************************************************************************
 *					txHwQueue_setHwInfo()
 ****************************************************************************

  DESCRIPTION:	
  
	Called after the HW configuration in the driver init or recovery process.
	Configure Tx HW information, including Tx-HW-blocks number, and per queue
	  Tx-descriptors number. Than, restart the module variables.

 ****************************************************************************/
TI_STATUS txHwQueue_setHwInfo(TI_HANDLE hTxHwQueue, DmaParams_T *pDmaParams) 
{
	TxHwQueueObj_t *pTxHwQueue = (TxHwQueueObj_t *)hTxHwQueue;
	int TxQid;
	
	pTxHwQueue->NumBlocks = pDmaParams->NumTxBlocks - 1; /* One block must be always free for FW use. */
	
	/* Get the Tx descriptors number per queue. */
	for(TxQid = 0; TxQid < MAX_NUM_OF_TX_QUEUES ; TxQid++)
		pTxHwQueue->TxHwQueueInfo[TxQid].numDescriptors = pDmaParams->TxNumDesc[TxQid];

	/* Restart the module variables. */
	txHwQueue_restart(hTxHwQueue);
	
	return OK;
}




/****************************************************************************
 *               txHwQueue_configQueue()
 ****************************************************************************
   DESCRIPTION:	
   
   Configure Tx HW queue blocks accounting parameters used in the allocate and free 
     procedures in this module.

	Two thresholds are defined per queue:
	a)	TxBlocksLowPercentPerQueue[queue] - The lower threshold is the minimal number of 
		Tx blocks guaranteed for each queue.
		The sum of all low thresholds should be less than 100%.
	b)	TxBlocksHighPercentPerQueue[queue] - The higher threshold is the maximal number of
		Tx blocks that may be allocated to the queue.
		The extra blocks above the low threshold can be allocated when needed only 
		if they are currently available and are not needed in order to guarantee
		the other queues low threshold.
		The sum of all high thresholds should be more than 100%.
		
 ****************************************************************************/
TI_STATUS txHwQueue_configQueue(TI_HANDLE hTxHwQueue, UINT8 TxQid,
								UINT16 percentOfBlockLowThreshold, UINT16 percentOfBlockHighThreshold)
{
	TxHwQueueObj_t *pTxHwQueue = (TxHwQueueObj_t *)hTxHwQueue;
	txHwQueueInfo_t *pQueueInfo = &(pTxHwQueue->TxHwQueueInfo[TxQid]);
	
	/* Calculate queue's blocks high threshold:  maximum number that may be allocated to it. */
	pQueueInfo->numBlocksHighThreshold = percentOfBlockHighThreshold * pTxHwQueue->NumBlocks / 100;

	/* Calculate queue's blocks low threshold:  minimum number that must be reserved for it. */
	pQueueInfo->numBlocksLowThreshold = percentOfBlockLowThreshold * pTxHwQueue->NumBlocks / 100;

	/* Set the threshold for low block resources:  when the next packet may not have enough blocks. */
	if (pQueueInfo->numBlocksLowThreshold > MAX_BLKS_PER_PKT)
		pQueueInfo->lowResourceThresh = pQueueInfo->numBlocksLowThreshold - MAX_BLKS_PER_PKT;
	else
		pQueueInfo->lowResourceThresh = 0;

	pQueueInfo->numBlocksUsed = 0;
	pQueueInfo->numPackets = 0;

	/* Since no blocks are used yet, reserved blocks number equals to the low threshold. */
	pQueueInfo->numBlocksReserved = pQueueInfo->numBlocksLowThreshold;

	/* Accumulate total reserved blocks. */
	pTxHwQueue->TotalBlocksReserved += pQueueInfo->numBlocksReserved;
	
	WLAN_REPORT_INIT(pTxHwQueue->hReport, TX_HW_QUEUE_MODULE_LOG,
		("txHwQueue_configQueue(): HighThresh=%d, LowThresh=%d, LowRsrcThresh=%d, TotalReserved=%d\n", 
		pQueueInfo->numBlocksHighThreshold, pQueueInfo->numBlocksLowThreshold, 
		pQueueInfo->lowResourceThresh, pTxHwQueue->TotalBlocksReserved));

	return OK;
}




/****************************************************************************
 *               txHwQueue_restart()
 ****************************************************************************
   DESCRIPTION:	 
   ============
	 Restarts the Tx-HW-Queue module.
	 Should be called upon disconnect and recovery!!
 ****************************************************************************/
TI_STATUS txHwQueue_restart(TI_HANDLE hTxHwQueue)
{
	TxHwQueueObj_t *pTxHwQueue = (TxHwQueueObj_t *)hTxHwQueue;
	txHwQueueInfo_t *pQueueInfo;
	int TxQid;
	
	/* All blocks are free at restart.
		Note that free means all blocks that are currently not in use, 
		while reserved are a part of the free blocks that are the summary of all queues reserved blocks.
		Each queue may take from the reserved part only up to its own reservation (according to
		its low threshold). */

	pTxHwQueue->NumFree = pTxHwQueue->NumBlocks;
	pTxHwQueue->TotalBlocksReserved = 0;

	for(TxQid = 0; TxQid < MAX_NUM_OF_TX_QUEUES ; TxQid++)
	{
		pQueueInfo = &(pTxHwQueue->TxHwQueueInfo[TxQid]);
		pQueueInfo->numBlocksUsed = 0;
		pQueueInfo->numPackets = 0;

		/* Since no blocks are used yet, reserved blocks number equals to the low threshold. */
		pQueueInfo->numBlocksReserved = pQueueInfo->numBlocksLowThreshold;

		/* Accumulate total reserved blocks. */
		pTxHwQueue->TotalBlocksReserved += pQueueInfo->numBlocksReserved;
	}

	return OK;
}




/****************************************************************************
 *					txHwQueue_alloc()
 ****************************************************************************
 * DESCRIPTION:	
   ============
   If the required blocks are available for the queue and there is an available
     descriptor, update the blocks and descriptor allocation and return OK.
   Else, return NOK.
   If the queue's reaources (blocks) are low, indicate in the descriptor to get
     Tx-complete from FW immediately.
 ****************************************************************************/
TI_STATUS txHwQueue_alloc(TI_HANDLE hTxHwQueue, txCtrlBlkEntry_t *pPktCtrlBlk)
{
	TxHwQueueObj_t *pTxHwQueue = (TxHwQueueObj_t *)hTxHwQueue;
	UINT8 numBlksToAlloc; /* The number of blocks required for the current packet. */
	UINT8 maxAllowed; /* Max blocks that may be currently allocated to this Queue to obey the high threshold.*/
	UINT8 maxAvailable; /* Max blocks that are currently free and not reserved for other Queues. */
	UINT8 reserved; /* How many blocks are reserved to this Queue before this allocation. */
	txHwQueueInfo_t *pQueueInfo = &(pTxHwQueue->TxHwQueueInfo[pPktCtrlBlk->txDescriptor.xmitQueue]);


	/***********************************************************************/
	/*  Calculate packet fragmentation threshold and required HW blocks.   */
	/***********************************************************************/

	txHwQueueCalc_BlocksNum(hTxHwQueue, pPktCtrlBlk);


	/***********************************************************************/
	/*            Check if the required resources are available            */
	/***********************************************************************/

	/* If all queue's descriptors are occupied, return BUSY. */
	if (pQueueInfo->numPackets == pQueueInfo->numDescriptors)
	{
		WLAN_REPORT_INFORMATION(pTxHwQueue->hReport, TX_HW_QUEUE_MODULE_LOG,  
			("txHwQueue_alloc(): No Descriptors, Queue=%d, Descriptors=%d, Packets=%d\n", 
			pPktCtrlBlk->txDescriptor.xmitQueue, pQueueInfo->numDescriptors, pQueueInfo->numPackets));
		return NOK;
	}

	reserved = pQueueInfo->numBlocksReserved;
	numBlksToAlloc = pPktCtrlBlk->txDescriptor.numMemBlks;

	/* Calculate how far we are from this Queue's high threshold limit (allowed = highThresh - used). */
	maxAllowed = pQueueInfo->numBlocksHighThreshold - pQueueInfo->numBlocksUsed;
	
	/* Calculate how many buffers are available for this Queue: the total free buffers minus the buffers
	     that are reserved for other Queues (all reserved minus this Queue's reserved). */
	maxAvailable = pTxHwQueue->NumFree - (pTxHwQueue->TotalBlocksReserved - reserved);

	/* If we need more blocks than are allowed or available, return BUSY. */
	if (numBlksToAlloc > min(maxAllowed, maxAvailable))
	{
		WLAN_REPORT_INFORMATION(pTxHwQueue->hReport, TX_HW_QUEUE_MODULE_LOG,  
			("txHwQueue_alloc(): No Hw-Blocks, Queue=%d, Req-blks=%d , Free=%d, Used=%d, available=%d\n",
			  pPktCtrlBlk->txDescriptor.xmitQueue, numBlksToAlloc, pTxHwQueue->NumFree, pQueueInfo->numBlocksUsed, maxAvailable));
		return NOK;
	}

	/***********************************************************************/
	/*                    Allocate required resources                      */
	/***********************************************************************/

	/* Update number of packets in FW (for descriptors allocation check). */
	pQueueInfo->numPackets++;

	
	/* If we are currently using less than the low threshold (i.e. we have some reserved blocks), 
		blocks allocation should reduce the reserved blocks number as follows:
	*/
	if (reserved)
	{

		/* If adding the allocated blocks to the used blocks will pass the low-threshold,
			only the part up to the low-threshold is subtracted from the reserved blocks.
			This is because blocks are reserved for the Queue only up to its low-threshold. 
			
		      0   old used                    low      new used       high
			  |######|                         |          |            |
			  |######|                         |          |            |
			          <------------ allocated ----------->
			          <----- old reserved ---->
					         new reserved = 0     (we passed the low threshold)
		*/
		if (numBlksToAlloc > reserved)
		{
			pQueueInfo->numBlocksReserved = 0;
			pTxHwQueue->TotalBlocksReserved -= reserved; /* reduce change from total reserved.*/
		}


		/* Else, if allocating less than reserved,
			the allocated blocks are subtracted from the reserved blocks:
			
		      0   old used       new used               low      high
			  |######|               |                   |        |
			  |######|               |                   |        |
			          <- allocated ->
			          <--------- old reserved ---------->
			                         <-- new reserved -->
		*/
		else
		{
			pQueueInfo->numBlocksReserved -= numBlksToAlloc;
			pTxHwQueue->TotalBlocksReserved -= numBlksToAlloc; /* reduce change from total reserved.*/
		}
	}


	/* Update total free blocks and Queue used blocks with the allocated blocks number. */
	pTxHwQueue->NumFree -= numBlksToAlloc;
	pQueueInfo->numBlocksUsed += numBlksToAlloc;

	/* If this queue has low resources (blocks or descriptors), set descriptor flag to get Tx-Complete from FW. */
	if ( (pQueueInfo->numBlocksUsed > pQueueInfo->lowResourceThresh) ||
		 (pQueueInfo->numPackets == pQueueInfo->numDescriptors - 1) )
	{
		#ifdef _WINDOWS
		#else
		  pPktCtrlBlk->txDescriptor.txAttr |= TX_COMPLETE_REQUIRED_BIT;  
		#endif
	}

	WLAN_REPORT_INFORMATION(pTxHwQueue->hReport, TX_HW_QUEUE_MODULE_LOG,  
		("txHwQueue_alloc(): SUCCESS,  Queue=%d, Req-blks=%d , Free=%d, Used=%d, LowResources=%d\n",
		  pPktCtrlBlk->txDescriptor.xmitQueue, numBlksToAlloc, pTxHwQueue->NumFree, 
		  pQueueInfo->numBlocksUsed, (pQueueInfo->numBlocksUsed > pQueueInfo->lowResourceThresh)));

	return OK;
}




/****************************************************************************
 *					txHwQueue_free()
 ****************************************************************************
 * DESCRIPTION:	Decrement the number of used descriptors and used data blks 
				for the specific queue.
 ****************************************************************************/
TI_STATUS txHwQueue_free(TI_HANDLE hTxHwQueue, txCtrlBlkEntry_t *pPktCtrlBlk)
{
	TxHwQueueObj_t *pTxHwQueue = (TxHwQueueObj_t *)hTxHwQueue;
	UINT8 numBlksToFree; /* The number of blocks freed by the current packet. */
	UINT8 lowThreshold;  /* Minimum blocks that are guaranteed for this Queue. */
	UINT8 newUsed; /* Blocks that are used by this Queue after freeing these blocks. */
	UINT8 newReserved; /* How many blocks are reserved to this Queue after freeing. */
	txHwQueueInfo_t *pQueueInfo = &(pTxHwQueue->TxHwQueueInfo[pPktCtrlBlk->txDescriptor.xmitQueue]);
	
	numBlksToFree = pPktCtrlBlk->txDescriptor.numMemBlks;

	/* Update number of packets in FW (for descriptors allocation check). */
	pQueueInfo->numPackets--;

#ifdef TI_DBG  /* Debug Counters */
	/* Sanity check: make sure we don't free more blocks than are in use. */
	if (numBlksToFree > pQueueInfo->numBlocksUsed)
	{
		WLAN_REPORT_ERROR(pTxHwQueue->hReport, TX_HW_QUEUE_MODULE_LOG,
		("txHwQueue_free():  Try to free more blks than used: Queue %d, free %d, used %d\n",
			pPktCtrlBlk->txDescriptor.xmitQueue, numBlksToFree, pQueueInfo->numBlocksUsed));
		return NOK;
	}
#endif
	
	/* Update total free blocks and Queue used blocks with the freed blocks number. */
	pTxHwQueue->NumFree += numBlksToFree;
	pQueueInfo->numBlocksUsed -= numBlksToFree;


	lowThreshold = pQueueInfo->numBlocksLowThreshold;
	newUsed = pQueueInfo->numBlocksUsed;

	
	/* If after freeing the blocks we are using less than the low threshold, 
		update total reserved blocks number as follows:
	   (note: if we are above the low threshold after freeing the blocks we still have no reservation.)
	*/
	if (newUsed < lowThreshold)
	{
		newReserved = lowThreshold - newUsed;
		pQueueInfo->numBlocksReserved = newReserved;

		
		/* If freeing the blocks reduces the used blocks from above to below the low-threshold,
			only the part from the low-threshold to the new used number is added to the 
			reserved blocks (because blocks are reserved for the Queue only up to its low-threshold):
			
		      0        new used               low            old used         high
			  |###########|####################|################|             |
			  |###########|####################|################|             |
			               <-------------- freed -------------->
			               <-- new reserved -->
					         old reserved = 0
		*/
		if (numBlksToFree > newReserved)
			pTxHwQueue->TotalBlocksReserved += newReserved; /* Add change to total reserved.*/


		/* Else, if we were under the low-threshold before freeing these blocks,
			all freed blocks are added to the reserved blocks: 
			
		      0             new used          old used             low               high
			  |################|#################|                  |                  |
			  |################|#################|                  |                  |
			                    <---- freed ---->
			                                      <- old reserved ->
			                    <---------- new reserved ---------->
		*/
		else
			pTxHwQueue->TotalBlocksReserved += numBlksToFree; /* Add change to total reserved.*/
	}
	
	return OK;
}




/****************************************************************************
 *					txHwQueue_GetUsedHwBlks()
 ****************************************************************************
 * DESCRIPTION:	 Provide the number of used HW Tx blocks of the given Queue.
 ****************************************************************************/
UINT8 txHwQueue_GetUsedHwBlks(TI_HANDLE hTxHwQueue, int TxQid)
{
	TxHwQueueObj_t *pTxHwQueue = (TxHwQueueObj_t *)hTxHwQueue;
	return (pTxHwQueue->TxHwQueueInfo[TxQid].numBlocksUsed);
}




/****************************************************************************
 *                      txHwQueue_printInfo()
 ****************************************************************************
 * DESCRIPTION:	Print the Hw Queue current information
 ****************************************************************************/
void txHwQueue_printInfo(TI_HANDLE hTxHwQueue)
{
#ifdef TI_DBG    
	TxHwQueueObj_t *pTxHwQueue = (TxHwQueueObj_t *)hTxHwQueue;
	int TxQid;

	/* Print the Tx-HW-Queue information: */
    WLAN_OS_REPORT(("Hw-Queues Information:\n"));
    WLAN_OS_REPORT(("======================\n"));
    WLAN_OS_REPORT(("Total Blocks:			%d\n", pTxHwQueue->NumBlocks));
    WLAN_OS_REPORT(("Total Free Blocks:		%d\n", pTxHwQueue->NumFree));
    WLAN_OS_REPORT(("Total Reserved Blocks: %d\n\n", pTxHwQueue->TotalBlocksReserved));

	for(TxQid = 0; TxQid < MAX_NUM_OF_TX_QUEUES; TxQid++)
	{
		WLAN_OS_REPORT(("Queue %d: Used=%d, Reserved=%d, LowThresh=%d, HighThresh=%d, LowRsrcThresh=%d, NumDesc=%d, UsedDesc=%d\n", 
			TxQid,
			pTxHwQueue->TxHwQueueInfo[TxQid].numBlocksUsed,
			pTxHwQueue->TxHwQueueInfo[TxQid].numBlocksReserved,
			pTxHwQueue->TxHwQueueInfo[TxQid].numBlocksLowThreshold,
			pTxHwQueue->TxHwQueueInfo[TxQid].numBlocksHighThreshold,
			pTxHwQueue->TxHwQueueInfo[TxQid].lowResourceThresh,
			pTxHwQueue->TxHwQueueInfo[TxQid].numDescriptors,
			pTxHwQueue->TxHwQueueInfo[TxQid].numPackets));
	}
#endif /* TI_DBG */
}

