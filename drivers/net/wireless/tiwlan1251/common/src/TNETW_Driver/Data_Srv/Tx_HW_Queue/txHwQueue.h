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
 *   MODULE:  txHwQueue.h
 *
 *   PURPOSE: hardware Tx queues management object
 * 
 ****************************************************************************/

#ifndef _TX_HW_QUEUE_H
#define _TX_HW_QUEUE_H


#include "whalParams.h"
#include "TNETW_Driver_types.h"
#include "txHwQueue_api.h"


#define MAX_BLKS_PER_PKT		16		/* Max number of Tx HW blocks that may be required for one packet. */
#define MAX_FRAG_THRESHOLD		4096


/* Per Queue HW blocks accounting data: */
typedef struct
{
    UINT8  	numBlocksHighThreshold;	/* The maximum HW blocks that can be allocated for this Queue. */
    UINT8   numBlocksLowThreshold;	/* The minimum HW blocks that must be reserved for this Queue. */
    UINT8   numBlocksUsed;			/* The number of HW blocks that are currently allocated for this Queue. */
    UINT8   numBlocksReserved;		/* The number of HW blocks currently reserved for this Queue (to guarentee the low threshold). */
	UINT8	lowResourceThresh;		/* Above this blocks number we need to request tx-complete from FW. */
	UINT8	numDescriptors;			/* The number of descriptors in the FW per this Queue. */
	UINT8	numPackets;				/* The current number of packets in the FW per this Queue. */
}txHwQueueInfo_t; 

typedef struct
{
	TI_HANDLE  hOs;
	TI_HANDLE  hReport;
	WhalParams_T *pWhalParams;
	
	UINT32  NumBlocks;		/* The total number of Tx blocks		*/
	UINT32  NumFree;		/* Total number of free HW blocks		*/    
	UINT32  TotalBlocksReserved; /* Total number of free but reserved HW blocks */
	
	txHwQueueInfo_t  TxHwQueueInfo[MAX_NUM_OF_TX_QUEUES]; 

} TxHwQueueObj_t;


/* Queue-Calc public functions: */
extern  void  txHwQueueCalc_BlocksNum(TI_HANDLE hTxHwQueue, txCtrlBlkEntry_t *pPktCtrlBlk);




#endif  /* _TX_HW_QUEUE_H */
