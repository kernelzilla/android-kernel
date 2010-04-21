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

#include "whalDefrag_api.h"

/*
 * ----------------------------------------------------------------------------
 * Function : findAndInsertEndpointIndex
 *
 * Input    : 
 * Output   :									 
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
static int findAndInsertEndpointIndex (TI_HANDLE hWhalDefrag, UINT8* mac, UINT8* idx)
{
	UINT8 i;
	WHAL_DEFRAG *pWhalDefrag = (WHAL_DEFRAG *)hWhalDefrag;
	
	/* first search for the endpoint*/
	for (i=0; i<pWhalDefrag->numCollectEntry; i++)
	{
		if ((pWhalDefrag->endpntMngr[i].inUse == TRUE) && 
			(os_memoryCompare (pWhalDefrag->hOs, pWhalDefrag->endpntMngr[i].srcMac, mac, MAC_ADDR_LEN) == 0))
		{
			*idx = i;
			return OK;
		}
	}
						
	/* endpoint not found, now try to locate unused entry and insert the endpoint*/
	for (i=0; i<pWhalDefrag->numCollectEntry; i++)
	{
		if (pWhalDefrag->endpntMngr[i].inUse == FALSE)
		{
			pWhalDefrag->endpntMngr[i].inUse = TRUE;
            os_memoryCopy (pWhalDefrag->hOs, pWhalDefrag->endpntMngr[i].srcMac, mac, MAC_ADDR_LEN);
			*idx = i;
			return OK;
		}
	}
	
	/* no free entry found*/
	return NOK;
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalDefrag_Create
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
TI_HANDLE whalDefrag_Create (TI_HANDLE hWhalCtrl, TI_HANDLE hOs, UINT8 numCollectEntries)
{
	UINT8 i;
	WHAL_DEFRAG* pWhalDefrag;
	
	pWhalDefrag = (WHAL_DEFRAG *)os_memoryAlloc (hOs, sizeof(WHAL_DEFRAG));
	if (pWhalDefrag == NULL)
		return NULL;

	os_memoryZero (hOs, pWhalDefrag, sizeof(WHAL_DEFRAG));

	pWhalDefrag->hWhalCtrl = (WHAL_CTRL *)hWhalCtrl; 
	pWhalDefrag->hOs = hOs;

	/* get the number of fragment collection entries from HAL DB*/ 
	pWhalDefrag->numCollectEntry = numCollectEntries;
	
	/* Create EndPoint objects*/
	for (i=0; i<pWhalDefrag->numCollectEntry; i++)
	{
		pWhalDefrag->pWhalEndpntEnt[i] = whalEndpnt_Create (hWhalCtrl, hOs);
		if (pWhalDefrag->pWhalEndpntEnt[i] == NULL)
		{
			whalDefrag_Destroy(pWhalDefrag);
			return NULL;
		}
	}

	return((TI_HANDLE)pWhalDefrag);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalDefrag_Config
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalDefrag_Config (TI_HANDLE hWhalDefrag, whalDefrag_config_t* pWhalDefragCfg)
{
	UINT8 i;
	whalEndpnt_config_t whalEndpntCfg;
	whalEndpnt_config_t* pWhalEndpntCfg	= &whalEndpntCfg;
	WHAL_DEFRAG *pWhalDefrag = (WHAL_DEFRAG *)hWhalDefrag;
	
	/* Save configuration parameters */
	pWhalDefrag->hReport = pWhalDefragCfg->hReport;
	pWhalDefrag->hMemMngr = pWhalDefragCfg->hMemMngr;

	/* Config EndPoint object */
	pWhalEndpntCfg->hReport = pWhalDefrag->hReport;
	pWhalEndpntCfg->hMemMngr = pWhalDefrag->hMemMngr;

	for (i=0; i<pWhalDefrag->numCollectEntry; i++)
		 whalEndpnt_Config (pWhalDefrag->pWhalEndpntEnt[i], pWhalEndpntCfg);

	return (OK);
}			    
    

/*
 * ----------------------------------------------------------------------------
 * Function : whalDefrag_MpduCollect
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
collectStatus_e whalDefrag_MpduCollect (TI_HANDLE hWhalDefrag, mem_MSDU_T* pMpdu, mem_MSDU_T** pMsdu)
{
    UINT8 index;
	dot11_header_t*	pHeader;
	UINT32          moreFrag;
	collectStatus_e rc;
	WHAL_DEFRAG *pWhalDefrag = (WHAL_DEFRAG *)hWhalDefrag;
	UINT8 *bssid = whal_ParamsGetBssId(pWhalDefrag->hWhalCtrl->pWhalParams);
	
	if (pMpdu == NULL)
		return (MPDU_DROP);

	/*
	 * if the received bssid isn't match the current bssid we wan't to filter the those fragments,
	 * we don't expect to get fragments without the current bssid.
	 */
	pHeader = (dot11_header_t*)((pMpdu->firstBDPtr->data) + memMgr_BufOffset(pMpdu->firstBDPtr));
    moreFrag = (pHeader->fc & DOT11_FC_MORE_FRAG);
	if(moreFrag && (os_memoryCompare(pWhalDefrag->hOs,(void *)bssid,(void *)pHeader->address2.addr,MAC_ADDR_LEN)!=0))
	{
		if (wlan_memMngrFreeMSDU(pWhalDefrag->hMemMngr, pMpdu->handle) != OK)
			WLAN_REPORT_ERROR(pWhalDefrag->hReport, HAL_RX_MODULE_LOG,
					  ("whalDefrag_MpduCollect: drop mpdu because moreFrag = %d && bssid = 0x%x   \n",moreFrag,pHeader->address2.addr[0],pHeader->address2.addr[1],pHeader->address2.addr[2],pHeader->address2.addr[3],pHeader->address2.addr[4],pHeader->address2.addr[5]));
			return (MPDU_DROP);
	}

	if (findAndInsertEndpointIndex (hWhalDefrag, (UINT8 *)pHeader->address2.addr, &index) == OK)
	{
		rc = whalEndpnt_FragCollect (pWhalDefrag->pWhalEndpntEnt[index], pMpdu, pMsdu);
		
        if (whalEndpnt_IsCollect (pWhalDefrag->pWhalEndpntEnt[index]) == FALSE)
			pWhalDefrag->endpntMngr[index].inUse = FALSE;
		
		return (rc);
	}
	else
	{
		WLAN_REPORT_ERROR(pWhalDefrag->hReport, HAL_RX_MODULE_LOG, 
						   (" whalDefrag_MpduCollect: findAndInsertEndpointIndex failure, MPDU drop\n"));
		if (wlan_memMngrFreeMSDU(pWhalDefrag->hMemMngr, pMpdu->handle) != OK)
			WLAN_REPORT_ERROR(pWhalDefrag->hReport, HAL_RX_MODULE_LOG,
							  ("whalDefrag_MpduCollect: ERROR wlan_memMngrFreeMSDU failure \n"));
		return (MPDU_DROP);
	}
}                   

 
/*
 * ----------------------------------------------------------------------------
 * Function : whalDefrag_Destroy
 *
 * Input    : 
 * Output   :									 
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalDefrag_Destroy (TI_HANDLE hWhalDefrag)
{
	UINT8 i;
	WHAL_DEFRAG *pWhalDefrag = (WHAL_DEFRAG *)hWhalDefrag;

	if (pWhalDefrag == NULL)
		return OK;

	for (i=0; i<pWhalDefrag->numCollectEntry; i++)
	{
		/* Destroy EndPoint object */
		if (whalEndpnt_Destroy (pWhalDefrag->pWhalEndpntEnt[i]) != OK)
			WLAN_REPORT_ERROR(pWhalDefrag->hReport, HAL_RX_MODULE_LOG, 
							   (" whalDefrag_Destroy: whalEndpnt_Destroy failure \n"));
	}
							 
	os_memoryFree (pWhalDefrag->hOs, pWhalDefrag, sizeof(WHAL_DEFRAG));
	
	return (OK);
}


