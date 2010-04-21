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
 *   MODULE:  txCtrlBlk.c
 *   
 *   PURPOSE: Maintains active packets Tx attributes table (including descriptor). 
 * 
 *	 DESCRIPTION:  
 *   ============
 *		This module allocates and frees table entry for each packet in the Tx
 *		process (from sendPkt by upper driver until Tx-complete).
 *
 ****************************************************************************/

#include "osTIType.h"
#include "whalCommon.h"
#include "whalHwDefs.h"
#include "txCtrlBlk_api.h"

#include "txCtrlBlk.h"  /* Local definitions */



/****************************************************************************
 *                      txCtrlBlk_Create()
 ****************************************************************************
 * DESCRIPTION:	Create the Tx control block table object 
 * 
 * INPUTS:	hOs
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	The Created object
 ****************************************************************************/
TI_HANDLE txCtrlBlk_Create(TI_HANDLE hOs)
{
	txCtrlBlkObj_t *pTxCtrlBlk;

	pTxCtrlBlk = os_memoryAlloc(hOs, sizeof(txCtrlBlkObj_t));
	if (pTxCtrlBlk == NULL)
		return NULL;

	os_memoryZero(hOs, pTxCtrlBlk, sizeof(txCtrlBlkObj_t));

	pTxCtrlBlk->hOs = hOs;

	return( (TI_HANDLE)pTxCtrlBlk );
}




/****************************************************************************
 *                      txCtrlBlk_Destroy()
 ****************************************************************************
 * DESCRIPTION:	Destroy the Tx control block table object 
 * 
 * INPUTS:	hTxCtrlBlk - The object to free
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
TI_STATUS txCtrlBlk_Destroy(TI_HANDLE hTxCtrlBlk)
{
	txCtrlBlkObj_t *pTxCtrlBlk = (txCtrlBlkObj_t *)hTxCtrlBlk;

	if (pTxCtrlBlk)
		os_memoryFree(pTxCtrlBlk->hOs, pTxCtrlBlk, sizeof(txCtrlBlkObj_t));

	return OK;
}




/****************************************************************************
 *               txCtrlBlk_init()
 ****************************************************************************
   DESCRIPTION:	 Initialize the Tx control block module.
 ****************************************************************************/
TI_STATUS txCtrlBlk_init(TI_HANDLE hTxCtrlBlk, TI_HANDLE hReport)
{
	txCtrlBlkObj_t *pTxCtrlBlk = (txCtrlBlkObj_t *)hTxCtrlBlk;
	
	pTxCtrlBlk->hReport = hReport;
	
	txCtrlBlk_restart(hTxCtrlBlk);

	return OK;
}




/****************************************************************************
 *               txCtrlBlk_restart()
 ****************************************************************************
   DESCRIPTION:	 
   ============
	 Restarts the Tx-control-block table.
	 Should be called upon init, disconnect and recovery!!
 ****************************************************************************/
TI_STATUS txCtrlBlk_restart(TI_HANDLE hTxCtrlBlk)
{
	UINT8 entry;
	txCtrlBlkObj_t *pTxCtrlBlk = (txCtrlBlkObj_t *)hTxCtrlBlk;

	/* For all entries, write the entry index in the descriptor and the next entry address
		 in the next free entery pointer. */
	for(entry = 0; entry < CTRL_BLK_ENTRIES_NUM; entry++)
	{
		pTxCtrlBlk->TxCtrlBlkTbl[entry].txDescriptor.descID = entry;
		pTxCtrlBlk->TxCtrlBlkTbl[entry].pNextFreeEntry = &(pTxCtrlBlk->TxCtrlBlkTbl[entry + 1]);
	}

	/* Write null in the next-free index of the last entry. */
	pTxCtrlBlk->TxCtrlBlkTbl[CTRL_BLK_ENTRIES_NUM - 1].pNextFreeEntry = NULL;

#ifdef TI_DBG
	pTxCtrlBlk->numUsedEntries = 0;
#endif

	return OK;
}





/****************************************************************************
 *					txCtrlBlk_alloc()
 ****************************************************************************
 * DESCRIPTION:	 
	Allocate a free control-block entry for the current Tx packet's parameters
	  (including the descriptor structure).
	Note that entry 0 in the list is never allocated and points to the
	  first free entry.
 ****************************************************************************/
txCtrlBlkEntry_t *txCtrlBlk_alloc(TI_HANDLE hTxCtrlBlk)
{
	txCtrlBlkObj_t   *pTxCtrlBlk = (txCtrlBlkObj_t *)hTxCtrlBlk;
	txCtrlBlkEntry_t *pCurrentEntry; /* The pointer of the new entry allocated for the packet. */
	txCtrlBlkEntry_t *pFirstFreeEntry; /* The first entry just points to the first free entry. */ 

	pFirstFreeEntry = &(pTxCtrlBlk->TxCtrlBlkTbl[0]); 
	pCurrentEntry = pFirstFreeEntry->pNextFreeEntry; /* Get free entry. */

	/* If no free entries, print error (not expected to happen) and return NULL. */
	if (pCurrentEntry == NULL)
	{
#ifdef TI_DBG
		WLAN_REPORT_ERROR(pTxCtrlBlk->hReport, TX_CTRL_BLK_MODULE_LOG,
			("txCtrlBlk_alloc():  No free entry,  UsedEntries=%d\n", pTxCtrlBlk->numUsedEntries));
#endif
		return NULL;
	}
#ifdef TI_DBG
	pTxCtrlBlk->numUsedEntries++;
#endif

	/* Link the first entry to the next free entry. */
	pFirstFreeEntry->pNextFreeEntry = pCurrentEntry->pNextFreeEntry;
	
	/* Clear the next-free-entry index just as an indication that our entry is not free. */
	pCurrentEntry->pNextFreeEntry = 0;

	return pCurrentEntry;
}




/****************************************************************************
 *					txCtrlBlk_free()
 ****************************************************************************
 * DESCRIPTION:	
	Link the freed entry after entry 0, so now it is the first free entry to
	  be allocated.
 ****************************************************************************/
void txCtrlBlk_free(TI_HANDLE hTxCtrlBlk, txCtrlBlkEntry_t *pCurrentEntry)
{
	txCtrlBlkObj_t   *pTxCtrlBlk = (txCtrlBlkObj_t *)hTxCtrlBlk;
	txCtrlBlkEntry_t *pFirstFreeEntry = &(pTxCtrlBlk->TxCtrlBlkTbl[0]);

	/* If the pointed entry is already free, print error and exit (not expected to happen). */
	if (pCurrentEntry->pNextFreeEntry != 0)
	{
#ifdef TI_DBG	
		WLAN_REPORT_ERROR(pTxCtrlBlk->hReport, TX_CTRL_BLK_MODULE_LOG,
			("txCtrlBlk_free(): Entry %d alredy free, UsedEntries=%d\n", 
			pCurrentEntry->txDescriptor.descID, pTxCtrlBlk->numUsedEntries));
#endif			
		return;
	}
#ifdef TI_DBG	
	pTxCtrlBlk->numUsedEntries--;
#endif

	/* Link the freed entry between entry 0 and the next free entry. */
	pCurrentEntry->pNextFreeEntry   = pFirstFreeEntry->pNextFreeEntry;
	pFirstFreeEntry->pNextFreeEntry = pCurrentEntry;
}




/****************************************************************************
 *					txCtrlBlk_GetPointer()
 ****************************************************************************
 * DESCRIPTION:	 
	Return a pointer to the control block entry of the requested packet.
	Used upon tx-complete to retrieve info after getting the descId from the FW.
 ****************************************************************************/
txCtrlBlkEntry_t *txCtrlBlk_GetPointer(TI_HANDLE hTxCtrlBlk, UINT8 descId)
{
	txCtrlBlkObj_t *pTxCtrlBlk = (txCtrlBlkObj_t *)hTxCtrlBlk;
	return ( &(pTxCtrlBlk->TxCtrlBlkTbl[descId]) );
}




/****************************************************************************
 *                      txCtrlBlk_printTable()
 ****************************************************************************
 * DESCRIPTION:	 Print the txCtrlBlk table main fields.
 ****************************************************************************/
void txCtrlBlk_printTable(TI_HANDLE hTxCtrlBlk)
{
#ifdef TI_DBG    
	txCtrlBlkObj_t *pTxCtrlBlk = (txCtrlBlkObj_t *)hTxCtrlBlk;
	UINT8 entry;

	WLAN_OS_REPORT((" Tx-Control-Block Information,  UsedEntries=%d\n", pTxCtrlBlk->numUsedEntries));
	WLAN_OS_REPORT(("==============================================\n"));
	
	for(entry = 0; entry < CTRL_BLK_ENTRIES_NUM; entry++)
	{
		WLAN_OS_REPORT(("Entry %d: DescID=%d, NextEntry=0x%x, PktID=0x%x, PktLen=%d, FragThresh=%d, NumBlks=%d, FC=0x%x, Flags=0x%x\n", 
			entry, 
			pTxCtrlBlk->TxCtrlBlkTbl[entry].txDescriptor.descID,
			pTxCtrlBlk->TxCtrlBlkTbl[entry].pNextFreeEntry,
			pTxCtrlBlk->TxCtrlBlkTbl[entry].txPktParams.packetId,
			pTxCtrlBlk->TxCtrlBlkTbl[entry].txDescriptor.length,
			pTxCtrlBlk->TxCtrlBlkTbl[entry].txDescriptor.fragThreshold,
			pTxCtrlBlk->TxCtrlBlkTbl[entry].txDescriptor.numMemBlks,
			pTxCtrlBlk->TxCtrlBlkTbl[entry].txPktParams.headerFrameCtrl,
			pTxCtrlBlk->TxCtrlBlkTbl[entry].txPktParams.flags));
	}
#endif /* TI_DBG */
}
