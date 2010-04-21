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

#include "802_11Defs.h"
#include "Ethernet.h"
#include "whalCommon.h"
#include "whalCtrl_api.h"
#include "whalCtrl.h"
#include "whalSecurity.h"
#include "whalWpa.h"



#undef WHAL_WPA_MODULE_LOG_PRINT_ENABLE

static void	whal_MsduContentDump (TI_HANDLE hWhalWpa, mem_MSDU_T* pMsdu, char* str);

/*
 * ----------------------------------------------------------------------------
 * Function : whal_MsduContentDump
 *
 * Input    : 
 * Output   :									 
 * Process  : 
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
static void	whal_MsduContentDump (TI_HANDLE hWhalWpa, mem_MSDU_T* pMsdu, char *str)
{	
#ifdef WHAL_WPA_MODULE_LOG_PRINT_ENABLE
    UINT32 msduLen;
	mem_BD_T* pCurrBd;
	WHAL_WPA* pWhalWpa = (WHAL_WPA *)hWhalWpa;
    
	WLAN_REPORT_INFORMATION(pWhalWpa->hReport, HAL_SECURITY_MODULE_LOG,  
					("%s totalLen=%d^^^^^ \n", str, pMsdu->dataLen));
	
	msduLen = pMsdu->dataLen;
	pCurrBd = pMsdu->firstBDPtr;
	
	while ((msduLen >= 0)&&(pCurrBd!=NULL))
	{
		WLAN_REPORT_INFORMATION(pWhalWpa->hReport, HAL_SECURITY_MODULE_LOG,  
			("\nBdLen = %d\n", pCurrBd->length));
		
		HexDumpData((UINT8*)(pCurrBd->data+pCurrBd->dataOffset), pCurrBd->length);
	
		msduLen -=  pCurrBd->length;
		pCurrBd =  pCurrBd->nextBDPtr;
	}

#endif /*WHAL_WPA_MODULE_LOG_PRINT_ENABLE*/
}
/*
 * ----------------------------------------------------------------------------
 * Function : whalWpa_Create
 *
 * Input    : 
 * Output   :
 * Process  : Create the WPA - security sub-module.
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
TI_HANDLE whalWpa_Create (TI_HANDLE hOs, TI_HANDLE hWhalCtrl)
{
	WHAL_WPA* pWhalWpa;

	pWhalWpa = (WHAL_WPA *)os_memoryAlloc (hOs, sizeof(WHAL_WPA));
	if (pWhalWpa == NULL)
		return NULL;

	os_memoryZero (hOs, (void *)pWhalWpa, sizeof(WHAL_WPA));

	pWhalWpa->pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
	pWhalWpa->hOs = hOs;
	
	return((TI_HANDLE)pWhalWpa);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalWpa_Config
 *
 * Input    : 
 * Output   :
 * Process  : Configure the WPA - security sub-module.
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalWpa_Config (TI_HANDLE hWhalWpa, whalWpa_config_t* pWhalWpaCfg)
{
	WHAL_WPA* pWhalWpa = (WHAL_WPA *)hWhalWpa;

	pWhalWpa->hReport = pWhalWpaCfg->hReport;
	pWhalWpa->hMemMgr = pWhalWpaCfg->hMemMgr;

	return (OK);
}			    
						    
/*
 * ----------------------------------------------------------------------------
 * Function : whalWpa_MpduListFieldsAdd
 *
 * Input    : Transmitted MSDU
 * Output   : MSDU after padding for TKIP EIV or AES RSN fields.
 * Process  : Shift LLC+SNAP apart from WLAN header to add padding for EIV/RSN field. 
 *			  	
 * Note(s)  : An LLC+SNAP is located just after the 802.11 header	
 *			  	 on the first MPDU header only.
 * -----------------------------------------------------------------------------
 */
int whalWpa_MpduListFieldsAdd (TI_HANDLE hWhalWpa, mem_MSDU_T* pMpduList)
{
	WHAL_WPA* pWhalWpa = (WHAL_WPA *)hWhalWpa;
	UINT32 AddFieldSize;
	UINT8 tempbuff[WLAN_SNAP_HDR_LEN + WLAN_4X_CONCAT_HDR_LEN];

	/* 
	 * Check TX key type
	 * In case of TKIP we need to add bytes for IV
	 * For AES  we need to add 8 bytes
	 * For unified AES - always add 8 bytes
	 */
	if(pWhalWpa->currTxKeyType == TKIP_KEY)
	{
		AddFieldSize = IV_FIELD_SIZE;
	}
	else
	if(pWhalWpa->currTxKeyType == AES_KEY)
	{
		AddFieldSize = AES_AFTER_HEADER_FIELD_SIZE;
	}
	else
		return OK;

	/* Check if LLC+SNAP exist in the buffer of the header. */
	if ( (memMgr_MsduFirstLen(pMpduList) == pMpduList->headerLen+WLAN_SNAP_HDR_LEN) || 
		 (memMgr_MsduFirstLen(pMpduList) == pMpduList->headerLen+WLAN_SNAP_HDR_LEN + WLAN_4X_CONCAT_HDR_LEN ) )
	{
		UINT8 *pHeaderEnd =	(UINT8*)( (UINT32)memMgr_BufData(pMpduList->firstBDPtr) +
			memMgr_BufOffset(pMpduList->firstBDPtr) + pMpduList->headerLen );

		/* Copy LLC+SNAP to temp buffer. */
		os_memoryCopy (pWhalWpa->hOs, (void *)tempbuff, (void *)pHeaderEnd, WLAN_SNAP_HDR_LEN + WLAN_4X_CONCAT_HDR_LEN);

		/* Copy LLC+SNAP from temp buffer back to BD with required padding. */
		os_memoryCopy (pWhalWpa->hOs, (void *)(pHeaderEnd + AddFieldSize), (void *)tempbuff, WLAN_SNAP_HDR_LEN + WLAN_4X_CONCAT_HDR_LEN);
	  
		/* Clear padding field. */
		os_memoryZero (pWhalWpa->hOs, (void *)pHeaderEnd, AddFieldSize);
	}
	else
	{
		WLAN_REPORT_WARNING(pWhalWpa->hReport, HAL_SECURITY_MODULE_LOG,  
		      ("whalWpa_MpduListFieldsAdd:  ERROR: First Buffer Length Is Illegal !!!!!! \n"));
	}

	/* Update BD and MSDU sizes to include the padding. */
	memMgr_MsduFirstLen(pMpduList) += AddFieldSize;
	memMgr_MsduDataSize(pMpduList) += AddFieldSize;
		
	whal_MsduContentDump (pWhalWpa, pMpduList, "whalWpa_MpduListFieldsAdd: Exit (Tx Path) ");
	return (OK);			  
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalWpa_KeyAdd
 *
 * Input    : 
 * Output   :									 
 * Process  : Add key to the HW.
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalWpa_KeyAdd (TI_HANDLE hWhalWpa, securityKeys_t* pKey, void *CB_Func, TI_HANDLE CB_handle)
{
	WHAL_WPA* 		pWhalWpa = (WHAL_WPA *)hWhalWpa;
 
	/* Only WEP, TKIP, AES keys are handled*/
	switch (pKey->keyType)
	{
		case WEP_KEY:
			/* Configure the encKeys to the HW - default keys cache*/
			return (whal_hwCtrl_WepDefaultKeyAdd (pWhalWpa->pWhalCtrl->pHwCtrl, pKey, CB_Func, CB_handle));
		
		case TKIP_KEY:
			/* Set the REAL TKIP key into the TKIP key cache*/
			if (whal_hwCtrl_TkipMicMappingKeyAdd (pWhalWpa->pWhalCtrl->pHwCtrl, pKey, CB_Func, CB_handle) != OK)
				return (NOK);

			break;
		
		case AES_KEY:
			if (whal_hwCtrl_AesMappingKeyAdd (pWhalWpa->pWhalCtrl->pHwCtrl, pKey, CB_Func, CB_handle) != OK)
				return (NOK);
			break;
		
		default:
			return (NOK);
	}
	
	/* AES or TKIP key has been successfully added. Store the current */
	/* key type of the unicast (i.e. transmit !) key                  */
	if(!MAC_BROADCAST((&pKey->macAddress)))
	{
		pWhalWpa->currTxKeyType = pKey->keyType;
	}

	return (OK);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalWpa_KeyRemove
 *
 * Input    : 
 * Output   :									 
 * Process  : Add key to the HW.
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalWpa_KeyRemove (TI_HANDLE hWhalWpa, securityKeys_t* pKey, void *CB_Func, TI_HANDLE CB_handle)
{
	WHAL_WPA* 		pWhalWpa = (WHAL_WPA *)hWhalWpa;

	/* Only WEP, TKIP, AES keys are handled*/
	switch (pKey->keyType)
	{
		case WEP_KEY:
			/* Configure the encKeys to the HW - default keys cache*/
			return (whal_hwCtrl_WepDefaultKeyRemove (pWhalWpa->pWhalCtrl->pHwCtrl, pKey, CB_Func, CB_handle));
		
		case TKIP_KEY:
			/* Configure the encKeys to the HW - mapping keys cache*/
			/* configure through SET_KEYS command */

			/* remove the TKIP key from the TKIP key cache*/
			if (whal_hwCtrl_TkipMicMappingKeyRemove (pWhalWpa->pWhalCtrl->pHwCtrl, pKey, CB_Func, CB_handle) != OK)
				return (NOK);

			break;
		
		case AES_KEY:
			if (whal_hwCtrl_AesMappingKeyRemove (pWhalWpa->pWhalCtrl->pHwCtrl, pKey, CB_Func, CB_handle) != OK)
				return (NOK);
			break;

		
		default:
			return (NOK);
	}
	
	return (OK);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalWpa_DefaultKeyIdSet
 *
 * Input    : 
 * Output   :									 
 * Process  : Configure the default key Id to the HW.
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalWpa_DefaultKeyIdSet (TI_HANDLE hWhalWpa, UINT8 aKeyId, void *CB_Func, TI_HANDLE CB_handle)
{
	/* Do not configure the default key id to the HW, return OK to the caller*/
	return (OK);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalWpa_Destroy
 *
 * Input    : 
 * Output   :									 
 * Process  : Destroy the WPA - security sub-module.
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalWpa_Destroy (TI_HANDLE hWhalWpa)
{
	WHAL_WPA *pWhalWpa = (WHAL_WPA *)hWhalWpa;

	if (pWhalWpa)
		os_memoryFree (pWhalWpa->hOs, pWhalWpa, sizeof(WHAL_WPA));
	
	return (OK);
}

