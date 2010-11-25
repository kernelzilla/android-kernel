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
 *   MODULE:  txHwQueueCalc.c
 *   
 *   PURPOSE: 
 *		Calculates the fragmentation threshold and number of HW memory blocks
 *		  required for the transmitted packet.
 * 
 *	 DESCRIPTION:  
 *   ============
 *		This is a sub-module of the Tx-HW-Queue module.
 *		When the Tx-HW-Queue is requested to allocate HW resources for a Tx 
 *		  packet (see txHwQueue_alloc()), it first calls this module to get
 *		  the number of HW memory blocks required for the packet.
 *		First the fragmentation threshold is calculated, and then the number
 *		  blocks. Both values are written in the provided control block of
 *		  the packet (in the descriptor structure to be copied to the FW).
 *
 ****************************************************************************/

#include "public_types.h"
#include "802_11Defs.h"
#include "ratesTypes.h"
#include "whalCommon.h"
#include "whalParams.h"
#include "whalCtrl_api.h"
#include "txHwQueue_api.h"  
#include "utils.h"
#include "txHwQueue.h"		/* Local definitions */
#include "txHwQueueCalc.h"  /* Local definitions */




/****************************************************************************
 *                      txHwQueueCalc_BlocksNum()  
 ****************************************************************************
 * DESCRIPTION:	
 * ============
 *	Calculate the fragmentation threshold and the number of HW blocks
 *	  required for the whole FW Tx processing of the packet.
 *
 ****************************************************************************/
void txHwQueueCalc_BlocksNum(TI_HANDLE hTxHwQueue, txCtrlBlkEntry_t *pPktCtrlBlk)
{
	TxHwQueueObj_t *pTxHwQueue = (TxHwQueueObj_t *)hTxHwQueue;
    UINT16 memBlocks;
    UINT16 numMpdus;
    UINT16 numMemBlocksPerFullFrag;
    UINT16 payloadDataLen;
    UINT16 fragThreshold;

	/* Calculate the fragmentation threshold. */
	fragThreshold = CalcFragThreshold(pTxHwQueue, pPktCtrlBlk);

    payloadDataLen  = pPktCtrlBlk->txDescriptor.length + MAX_MSDU_SECURITY_LENGTH; 

    if (payloadDataLen > fragThreshold)
	{
#ifdef TI_DBG
        if (fragThreshold == 0)
        {
            WLAN_OS_REPORT(("ERROR !!!!!!!!! fragThreshold==0 !!!!!!"));
            return;
        }
#endif

		numMemBlocksPerFullFrag = ((fragThreshold + MAX_MPDU_HEADER_AND_SECURITY) / HW_BLOCK_SIZE) + 1;
        numMpdus = payloadDataLen / fragThreshold;
        memBlocks = numMpdus * numMemBlocksPerFullFrag;
        payloadDataLen -= numMpdus * fragThreshold;
		numMpdus++;
	}
	else
	{
		numMemBlocksPerFullFrag = 0;
		memBlocks = 0;
		numMpdus = 1;
	}

    memBlocks += (payloadDataLen / HW_BLOCK_SIZE) + 1;

	/* If fragmentation needed, add spare blocks for FW internal copy purposes. */
    if (numMpdus > 1)
        memBlocks += min(numMpdus, numMemBlocksPerFullFrag); 

  #ifdef TI_DBG
    if (memBlocks > 255)
        WLAN_REPORT_ERROR (pTxHwQueue->hReport, TX_HW_QUEUE_MODULE_LOG,  
            ("txHwQueueCalc_BlocksNum(): number of required blocks is bigger than 255 = %d\n", memBlocks));
  #endif

	/* Copy the frag-threshold and HW blocks number to the descriptor. */
    pPktCtrlBlk->txDescriptor.numMemBlks = (UINT8)memBlocks;
	pPktCtrlBlk->txDescriptor.fragThreshold = fragThreshold;

	WLAN_REPORT_INFORMATION(pTxHwQueue->hReport, TX_HW_QUEUE_MODULE_LOG,  
		("txHwQueueCalc_BlocksNum(): FragThresh=%d, NumBlks=%d, NumMpdus=%d, DataLen=%d\n",
		  fragThreshold, memBlocks, numMpdus, payloadDataLen));
}




/****************************************************************************
 *               CalcFragThreshold()
 ****************************************************************************
   DESCRIPTION:  Calculates the frag threshold per frame according to the frag threshold 
                  defined by the user and the TxOp fragmentation (if WME is used).

   PARAMETERS:   pTxHwQueue - The module object.
				 pPktCtrlBlk	- The current packet control block (including descriptor).
   
   RETURNS:      The final fragmentation threshold.
 ****************************************************************************/
static UINT16 CalcFragThreshold(TxHwQueueObj_t *pTxHwQueue, txCtrlBlkEntry_t *pPktCtrlBlk)
{
    UINT32 txOpLimit;
    UINT16 fragThreshold, uMaxHdrLen;
    WlanParams_T *pWlanParams = &(pTxHwQueue->pWhalParams->WlanParams);
    dot11_header_t *pDot11Hdr = (dot11_header_t*)((UINT8 *)pPktCtrlBlk->txPktParams.pFrame + TX_TOTAL_OFFSET_BEFORE_DATA);

    /* 
     *       It is prohibited by the standard to fragment multicast/broadcast 
     *       frames both in Infrastructure and Independent BSS types
     */
    
    /* check for multicast packet in the destination address (address1 or address3)*/
    if (MAC_MULTICAST(GET_DA_FROM_DOT11_HEADER_T(pDot11Hdr))) 
            return MAX_FRAG_THRESHOLD;

    /* For 4X don't fragment (use max frag threshold). */
    if (pWlanParams->Enable4x)
        return MAX_FRAG_THRESHOLD;

    /* Non-QOS mode */
    if (IS_LEGACY_DATA (pPktCtrlBlk->txPktParams.headerFrameCtrl))
    {
        /* use "legacy" mode for the WLAN header length */ 
        uMaxHdrLen = WLAN_HDR_LEN;
        /* Use "legacy" fragmentation */
        fragThreshold = pWlanParams->FragmentThreshold;
    } 

    /* QOS mode */
    else
    {
        uMaxHdrLen = WLAN_QOS_HDR_LEN;

        txOpLimit = pTxHwQueue->pWhalParams->AcParams.ac[pPktCtrlBlk->txDescriptor.xmitQueue].txopLimit;

        if (txOpLimit == 0)
        {
            /* 
             * If working in WME and TXOP limit is not set for this AC - 
             * Use "legacy" fragmentation and substract the over head of the QoS header
             */
            /* TODO yuval - check why 2 is needed */
            fragThreshold = pWlanParams->FragmentThreshold - DIFF_HEADER_LENGTH_LEGACY_TO_QOS;
        }
        else

        {
            /*
             * If TXOP-limit value is set (may require fragmentation for the time limit) -
             * calculate the fragmentation threshold for the given TXOP-limit
             */
            fragThreshold = GetTxOpFragThreshold (pTxHwQueue, pPktCtrlBlk, txOpLimit, pWlanParams);
        }
    }

    /* If the frag threshold is below minimal frag, use minimal frag threshold */
    if (fragThreshold < MIN_FRAG_THRESH)
        fragThreshold = MIN_FRAG_THRESH;

    /* Subtract header length and CRC length */
    fragThreshold -= uMaxHdrLen + FCS_LENGTH;

    /* Return the frag threshold. */
    /* Note that security overheads are excluded as they are allowed to exceed the time limit. */
    return fragThreshold;
}





/****************************************************************************
 *                      GetTxOpFragThreshold()
 ****************************************************************************
   DESCRIPTION:  Calculates the fragmentation threshold caused by the TxOpLimit.

   PARAMETERS:   pTxHwQueue - The module object.
				 pPktCtrlBlk	- The current packet control block (including descriptor structure).
   
   RETURNS:      The fragmentation threshold calculated for the TXOP limit.
 ****************************************************************************/
static UINT16 GetTxOpFragThreshold(TxHwQueueObj_t *pTxHwQueue, txCtrlBlkEntry_t *pPktCtrlBlk, 
								   UINT16 txOpLimit, WlanParams_T *pWlanParams)
{
    UINT16  fragDataTime; 
	UINT16  txOpFragThresh; 
	UINT16	plcpHdrTimeBRate;
	UINT16	plcpHdrTime;
	UINT16  durationOverhead;
	UINT16  rateMbps; 
    rate_e	initialRate;
	BOOL	rtsSet;
	rate_e  ctrlFrameRate = pTxHwQueue->pWhalParams->BssInfoParams.txCtrlFrmRateDriverFormat;
	uint8   ackPolicy;

	initialRate = ConvertRateTnetToDriver(pPktCtrlBlk->txDescriptor.rate);

#ifdef TI_DBG
	if (initialRate == DRV_RATE_INVALID)
	{
		WLAN_REPORT_ERROR(pTxHwQueue->hReport, TX_HW_QUEUE_MODULE_LOG,  
			("GetTxOpFragThreshold(): Unexpected Tx-Rate = %d\n", initialRate));
	}
#endif

    rateMbps = TxMemCalcRateValueTable[initialRate]; /* Convert from driver enum to Mbps value */

    /* Set the PLCP header time for B rates according to the preamble type. */
	if ( (pWlanParams->preamble == PREAMBLE_LONG) || (initialRate == DRV_RATE_1M) )
        plcpHdrTimeBRate = LONG_PREAMBLE_MICROSECONDS;
    else
        plcpHdrTimeBRate = SHORT_PREAMBLE_MICROSECONDS;

	/* Get PLCP header duration overhead. */
	if (initialRate >= DRV_RATE_6M)	/* If it's an OFDM rate. */
		plcpHdrTime = OFDM_PLCP_HDR_MICROSECONDS;
    else
        plcpHdrTime = plcpHdrTimeBRate;

	durationOverhead = plcpHdrTime;
	
	/* Add ACK overhead if not using No-ACK. */
	{    
        TxDescCtrl_t tmpTxDesc;
        COPY_UNALIGNED_LONG(&tmpTxDesc, &(pPktCtrlBlk->txDescriptor.txAttr));
        ackPolicy = tmpTxDesc.ackPolicy;
    }
    if ( !ackPolicy )
	{
        durationOverhead += TxMemCalcAckDurationTable[initialRate] + plcpHdrTime; 
		if (initialRate >= DRV_RATE_6M)
		    durationOverhead += OFDM_SIGNAL_EXT_MICROSECONDS; /* If OFDM add SIFS extra 6 uSec. */
    }

	/* If packet length bigger than RTS threshold, add RTS time to the duration overhead. */
    if (pPktCtrlBlk->txDescriptor.length > pWlanParams->RtsThreshold)
	{
        durationOverhead += RTS_FRAG_DATA_TIME;
		rtsSet = TRUE;
	}
	else
		rtsSet = FALSE;

	/* If protection CTS required for OFDM packet or RTS needed, add CTS time to duration overhead. */
    if ( (pWlanParams->CtsToSelf && (initialRate >= DRV_RATE_6M))  ||  rtsSet )
        durationOverhead += TxMemCalcAckDurationTable[ctrlFrameRate] + plcpHdrTimeBRate;
	
	/* If the TXOP time is longer than the packet overheads, get the delta (fragment body time). */
    if (txOpLimit > durationOverhead)
        fragDataTime = txOpLimit - durationOverhead;
	
	/* Else, Can't get into the TXOP limit time. The minimal frag threshold (256) will be used. */
    else
		fragDataTime = 0;  

	/* Calculate the fragmentation threshold in data bytes from the required duration and rate. */
    txOpFragThresh = fragDataTime * rateMbps / BIT_TO_BYTE_FACTOR;

    /* If rate is 5.5M, a value of 55 is used so compensate for the 10 times factor. */
    if (rateMbps == 55) 
        txOpFragThresh = txOpFragThresh / 10;

    /* Firmware requires the fragmentation threshold to be an EVEN number */
    txOpFragThresh &= ~1;

	WLAN_REPORT_INFORMATION(pTxHwQueue->hReport, TX_HW_QUEUE_MODULE_LOG,  
		("GetTxOpFragThreshold(): FragThresh=%d, Rate=%d, TXOP=%d, Overhead=%d, NoACK=%d, CTS=%d, RTS=%d\n",
		  txOpFragThresh, initialRate, txOpLimit, durationOverhead, ackPolicy,
		  pWlanParams->CtsToSelf, rtsSet));
    
    return (txOpFragThresh);
}




/****************************************************************************
 *                      ConvertRateTnetToDriver
 ****************************************************************************
 * DESCRIPTION: Convert the given rate from TNET format (Tx-descriptor) to driver format.
 *			
 * INPUTS:	txDescRate - Rate value in Tx-descriptor format
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	The converted rate in driver format. 
 ****************************************************************************/
static rate_e ConvertRateTnetToDriver (UINT16 tnetRate)
{
	switch (tnetRate)
	{
		case HW_BIT_RATE_1MBPS:		return DRV_RATE_1M;
		case HW_BIT_RATE_2MBPS:		return DRV_RATE_2M;
		case HW_BIT_RATE_5_5MBPS:	return DRV_RATE_5_5M;
		case HW_BIT_RATE_6MBPS:		return DRV_RATE_6M;
		case HW_BIT_RATE_9MBPS:		return DRV_RATE_9M;
		case HW_BIT_RATE_11MBPS:	return DRV_RATE_11M;
		case HW_BIT_RATE_12MBPS:	return DRV_RATE_12M;
		case HW_BIT_RATE_18MBPS:	return DRV_RATE_18M;
		case HW_BIT_RATE_22MBPS:	return DRV_RATE_22M;
		case HW_BIT_RATE_24MBPS:	return DRV_RATE_24M;
		case HW_BIT_RATE_36MBPS:	return DRV_RATE_36M;
		case HW_BIT_RATE_48MBPS:	return DRV_RATE_48M;
		case HW_BIT_RATE_54MBPS:	return DRV_RATE_54M;

		default:					return DRV_RATE_INVALID;
	}
}




