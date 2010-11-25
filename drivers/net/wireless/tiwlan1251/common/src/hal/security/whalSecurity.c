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

#include "whalCommon.h"
#include "whalCtrl_api.h"
#include "whalCtrl.h"
#include "whalSecurity.h"


/*
 * ----------------------------------------------------------------------------
 * Function : whalSecur_Create
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :  
 * -----------------------------------------------------------------------------
 */
TI_HANDLE whalSecur_Create (TI_HANDLE hOs, TI_HANDLE hWhalCtrl, UINT16 numOfStations)
{
	WHAL_SECURITY* pWhalSecur;

	pWhalSecur = (WHAL_SECURITY *)os_memoryAlloc (hOs, sizeof(WHAL_SECURITY));
	if (pWhalSecur == NULL)
		return NULL;

	os_memoryZero (hOs, (void *)pWhalSecur, sizeof(WHAL_SECURITY));

	pWhalSecur->pWhalCtrl = (WHAL_CTRL*)hWhalCtrl; 
	pWhalSecur->hOs = hOs;
	pWhalSecur->securityMode = RSN_CIPHER_NONE;

	pWhalSecur->pWhalWep = (WHAL_WEP*)whalWep_Create (hOs, hWhalCtrl);
	if (pWhalSecur->pWhalWep == NULL)
	{
		whalSecur_Destroy(pWhalSecur, numOfStations);
		return NULL;
	}
	pWhalSecur->pWhalWpa = (WHAL_WPA*)whalWpa_Create (hOs, hWhalCtrl);
	if (pWhalSecur->pWhalWpa == NULL)
	{
		whalSecur_Destroy(pWhalSecur, numOfStations);
		return NULL;
	}
#ifdef CKIP_ENABLED
	pWhalSecur->pWhalPrivacy = (privacy_t*)privacy_create (hOs);
	pWhalSecur->pWhalExc = (WHAL_EXC*)whalExc_Create(hOs, hWhalCtrl);
	if (pWhalSecur->pWhalExc == NULL)
	{
		whalSecur_Destroy(pWhalSecur, numOfStations);
		return NULL;
	}
#endif /* CKIP_ENABLED */
	pWhalSecur->numOfStations = numOfStations;
	pWhalSecur->reconfData.reconfKeys = (securityKeys_t*)os_memoryAlloc (hOs, 
										(sizeof(securityKeys_t))*(numOfStations*NO_OF_RECONF_SECUR_KEYS_PER_STATION+NO_OF_EXTRA_RECONF_SECUR_KEYS));
	os_memoryZero (hOs, (void *)pWhalSecur->reconfData.reconfKeys, 
										(sizeof(securityKeys_t))*(numOfStations*NO_OF_RECONF_SECUR_KEYS_PER_STATION+NO_OF_EXTRA_RECONF_SECUR_KEYS));
	
	return ((TI_HANDLE)pWhalSecur);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalSecur_Config
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalSecur_Config (TI_HANDLE hWhalSecur, whalSecur_config_t* pWhalSecurCfg)
{
	UINT32 index;
	whalWep_config_t wepCfg;
	whalWpa_config_t wpaCfg;
#ifdef CKIP_ENABLED
	whalExc_config_t excCfg;
#endif /* CKIP_ENABLED*/

	WHAL_SECURITY* pWhalSecur = (TI_HANDLE)hWhalSecur;

	/* Reset all reconfig valid fields*/
	pWhalSecur->reconfData.isHwEncDecrEnableValid = FALSE;
	pWhalSecur->reconfData.isDefaultKeyIdValid = FALSE;  
	for (index=0; index < ((pWhalSecur->numOfStations)*NO_OF_RECONF_SECUR_KEYS_PER_STATION+NO_OF_EXTRA_RECONF_SECUR_KEYS); index++)
			(pWhalSecur->reconfData.reconfKeys+index)->keyType = NULL_KEY;

	/* Save configuration parameters */
	pWhalSecur->hMemMgr = pWhalSecurCfg->hMemMgr;
	pWhalSecur->hReport = pWhalSecurCfg->hReport;
	

	/* Config the wep sub-module*/
	wepCfg.hMemMgr = pWhalSecur->hMemMgr;
	wepCfg.hReport = pWhalSecur->hReport;
	if (whalWep_Config (pWhalSecur->pWhalWep, &wepCfg) != OK)
		return (NOK);											  

	/* Config the wpa sub-module*/
	wpaCfg.hMemMgr = pWhalSecur->hMemMgr; 
	wpaCfg.hReport = pWhalSecur->hReport; 
	if (whalWpa_Config (pWhalSecur->pWhalWpa, &wpaCfg) != OK)
		return (NOK);
	
#ifdef CKIP_ENABLED
    wpaCfg.pWhalPrivacy = pWhalSecur->pWhalPrivacy;
	/* Config the privacy sub-module*/
	if (privacy_config (pWhalSecur->pWhalPrivacy, 
						pWhalSecur->hReport, pWhalSecur->hOs) != OK)
		return (NOK);
	excCfg.hMemMgr = pWhalSecur->hMemMgr;   
	excCfg.hReport = pWhalSecur->hReport;  
	excCfg.pWhalPrivacy = pWhalSecur->pWhalPrivacy;  
	if (whalExc_Config (pWhalSecur->pWhalExc, &excCfg) != OK)
		return (NOK);
#endif /*CKIP_ENABLED */

	return (OK);
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalSecur_KeyAdd
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalSecur_KeyAdd (TI_HANDLE hWhalSecur, securityKeys_t* pKey, BOOL reconfFlag, void *CB_Func, TI_HANDLE CB_handle)
{
	WHAL_SECURITY* pWhalSecur = (TI_HANDLE)hWhalSecur;
	UINT8          keyIdx     = (UINT8)pKey->keyIndex;

	/* store the security key for reconfigure time (FW reload)*/
	if (reconfFlag != TRUE)
	{
	  	if (keyIdx >= ((pWhalSecur->numOfStations)*NO_OF_RECONF_SECUR_KEYS_PER_STATION+NO_OF_EXTRA_RECONF_SECUR_KEYS))
		{
			WLAN_REPORT_ERROR(pWhalSecur->hReport, HAL_SECURITY_MODULE_LOG,  
				("whalSecur_KeyAdd: ERROR Key keyIndex field out of range =%d, range is (0 to %d)\n",
				 pKey->keyIndex, (pWhalSecur->numOfStations)*NO_OF_RECONF_SECUR_KEYS_PER_STATION+NO_OF_EXTRA_RECONF_SECUR_KEYS-1));
			
			return (NOK);
		}

		if (pKey->keyType == NULL_KEY)
		{
			WLAN_REPORT_ERROR(pWhalSecur->hReport, HAL_SECURITY_MODULE_LOG,  
								("whalSecur_KeyAdd: ERROR KeyType is NULL_KEY\n"));
			
			return (NOK);
		}

		os_memoryCopy (pWhalSecur->hOs, 
				(void *)(pWhalSecur->reconfData.reconfKeys + keyIdx),
				(void *)pKey, sizeof(securityKeys_t));
	}
	
	switch (pWhalSecur->securityMode)
	{
		case RSN_CIPHER_WEP:
		case RSN_CIPHER_WEP104:
			return (whalWep_KeyAdd (pWhalSecur->pWhalWep, pKey, CB_Func, CB_handle));
	
		case RSN_CIPHER_TKIP:
		case RSN_CIPHER_AES_CCMP:
			return (whalWpa_KeyAdd (pWhalSecur->pWhalWpa, pKey, CB_Func, CB_handle));

		default:
			return (NOK);
	}

}

/*
 * ----------------------------------------------------------------------------
 * Function : whalSecur_KeyRemove
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalSecur_KeyRemove (TI_HANDLE hWhalSecur, securityKeys_t* pKey, BOOL reconfFlag, void *CB_Func, TI_HANDLE CB_handle)
{
	WHAL_SECURITY* pWhalSecur = (TI_HANDLE)hWhalSecur;

	switch (pWhalSecur->securityMode)
	{
		case RSN_CIPHER_WEP:
		case RSN_CIPHER_WEP104:
			return (whalWep_KeyRemove (pWhalSecur->pWhalWep, pKey, CB_Func, CB_handle));
	
		case RSN_CIPHER_TKIP:
		case RSN_CIPHER_AES_CCMP:
			return (whalWpa_KeyRemove (pWhalSecur->pWhalWpa, pKey, CB_Func, CB_handle));
		
		default:
			return (NOK);
	}

}

/*
 * ----------------------------------------------------------------------------
 * Function : whalSecur_DefaultKeyIdSet
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalSecur_DefaultKeyIdSet (TI_HANDLE hWhalSecur, UINT8 aKeyId, void *CB_Func, TI_HANDLE CB_handle)
{
	WHAL_SECURITY* pWhalSecur = (TI_HANDLE)hWhalSecur;

	/* store the default key ID value for reconfigure time (FW reload)*/
	pWhalSecur->reconfData.reconfDefaultKeyId = aKeyId;
	
	switch (pWhalSecur->securityMode)
	{
		case RSN_CIPHER_WEP:
		case RSN_CIPHER_WEP104:
			pWhalSecur->reconfData.isDefaultKeyIdValid = TRUE;
			return (whalWep_DefaultKeyIdSet (pWhalSecur->pWhalWep, aKeyId, CB_Func, CB_handle));
	
		case RSN_CIPHER_TKIP:
		case RSN_CIPHER_AES_CCMP:
			pWhalSecur->reconfData.isDefaultKeyIdValid = TRUE;
			return (whalWpa_DefaultKeyIdSet (pWhalSecur->pWhalWpa, aKeyId, CB_Func, CB_handle));

		default:
			return (NOK);
	}

}

/*
 * ----------------------------------------------------------------------------
 * Function : whalSecur_HwEncDecrEnable
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalSecur_HwEncDecrEnable (TI_HANDLE hWhalSecur, BOOL aHwEncEnable)
{
	WHAL_SECURITY* pWhalSecur = (TI_HANDLE)hWhalSecur;
	
	/* store the HW encryption Enable flag for reconfigure time (FW reload)*/
	pWhalSecur->reconfData.reconfHwEncEnable = aHwEncEnable;

	pWhalSecur->reconfData.isHwEncDecrEnableValid = TRUE;

    return (whal_hwCtrl_EncDecrSet (pWhalSecur->pWhalCtrl->pHwCtrl, aHwEncEnable, aHwEncEnable));
}

#ifdef CKIP_ENABLED
/*
 * ----------------------------------------------------------------------------
 * Function : whalSecur_SwEncEnable
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalSecur_SwEncEnable (TI_HANDLE hWhalSecur, BOOL aSwEncEnable)
{
	WHAL_SECURITY* pWhalSecur = (TI_HANDLE)hWhalSecur;
	
	return (whalExc_swEncEnable (pWhalSecur->pWhalExc, aSwEncEnable));
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalSecur_MicFieldEnable
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalSecur_MicFieldEnable (TI_HANDLE hWhalSecur, BOOL aMicFieldEnable)
{
	WHAL_SECURITY* pWhalSecur = (TI_HANDLE)hWhalSecur;

	return (whalExc_micFieldEnable (pWhalSecur->pWhalExc, aMicFieldEnable));
}
#endif /*CKIP_ENABLED*/

/*
 * ----------------------------------------------------------------------------
 * Function : whalSecur_SecurModeSet
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalSecur_SecurModeSet (TI_HANDLE hWhalSecur, cipherSuite_e aSecurMode)
{
	UINT32 index;
	WHAL_SECURITY* pWhalSecur = (TI_HANDLE)hWhalSecur;
	if (aSecurMode<=RSN_CIPHER_CKIP)
	{
		WLAN_REPORT_INFORMATION (pWhalSecur->hReport, HAL_SECURITY_MODULE_LOG,
								("whalSecur_SecurModeSet: change security mode from %d --> %d\n",
							     pWhalSecur->securityMode, aSecurMode));
		/* check if security mode is equal to previous one*/
		if (pWhalSecur->securityMode == aSecurMode)
			return (OK);

		/* Reset all reconfig valid fields*/
		pWhalSecur->reconfData.isHwEncDecrEnableValid = FALSE;
		pWhalSecur->reconfData.isDefaultKeyIdValid = FALSE;  
		for (index=0; index < ((pWhalSecur->numOfStations)*NO_OF_RECONF_SECUR_KEYS_PER_STATION+NO_OF_EXTRA_RECONF_SECUR_KEYS); index++)
				(pWhalSecur->reconfData.reconfKeys+index)->keyType = NULL_KEY;
		
        /* set the new security mode*/
		pWhalSecur->securityMode = aSecurMode;

#ifdef CKIP_ENABLED
		/* Upon entering to RSN_PRIVACY_EXC mode, disable the excSwEnc and excMicField flags, 
		   reset the privacy Enc sub-module */
		if (aSecurMode == RSN_CIPHER_CKIP)
		{
			if (whalExc_swEncEnable (pWhalSecur->pWhalExc, FALSE) != OK)
				return (NOK);

			if (whalExc_micFieldEnable (pWhalSecur->pWhalExc, FALSE) != OK)
				return (NOK);

			if (privacy_resetEnc (pWhalSecur->pWhalPrivacy) != OK)
				return (NOK);
		}

		privacy_setPrivacyMode(pWhalSecur->pWhalPrivacy, aSecurMode);
#endif /* CKIP_ENABLED*/

		/* disable defrag, duplicate detection on TNETW+EXC on chip level*/
		if (pWhalSecur->securityMode==RSN_CIPHER_CKIP)
			/* YV- to add fragmentation control (if there is- artur ?)*/
			return (whal_hwCtrl_RxMsduFormatSet (pWhalSecur->pWhalCtrl->pHwCtrl, FALSE));
		else
			/* YV- to add fragmentation control (if there is- artur ?)*/
			return (whal_hwCtrl_RxMsduFormatSet (pWhalSecur->pWhalCtrl->pHwCtrl, TRUE));
		

	}
	else
		return (NOK);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalSecur_SecurModeGet
 *
 * Input    : 
 * Output   :	security mode
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
cipherSuite_e whalSecur_SecurModeGet (TI_HANDLE hWhalSecur)
{
	WHAL_SECURITY* pWhalSecur = (TI_HANDLE)hWhalSecur;

	return pWhalSecur->securityMode;
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalSecur_KeysReconfig
 *
 * Input    : 
 * Output   :
 * Process  : Reconfig security keys, default key Id and encryption/decryption 
 *			  control to the FW
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalSecur_KeysReconfig (TI_HANDLE hWhalSecur)
{
	UINT32 index;
	WHAL_SECURITY* pWhalSecur = (WHAL_SECURITY *)hWhalSecur;


	if (pWhalSecur->securityMode != RSN_CIPHER_NONE)
	{

		/* set the keys to the HW*/
		for (index=0; index < ((pWhalSecur->numOfStations)*NO_OF_RECONF_SECUR_KEYS_PER_STATION+NO_OF_EXTRA_RECONF_SECUR_KEYS); index++)
		{
			if ((pWhalSecur->reconfData.reconfKeys+index)->keyType != NULL_KEY)
			{
				if (whalSecur_KeyAdd (pWhalSecur, pWhalSecur->reconfData.reconfKeys+index, TRUE, NULL, NULL) != OK)
				{
					WLAN_REPORT_ERROR(pWhalSecur->hReport, HAL_SECURITY_MODULE_LOG,  
						("whalSecur_KeysReconfig: ERROR whalSecur_KeyAdd failure index=%d\n", index));
					return (NOK);
				}   
			}
		}
	
		if (pWhalSecur->reconfData.isDefaultKeyIdValid == TRUE)
		{
			/* set the deafult key ID to the HW*/
			if (whalSecur_DefaultKeyIdSet (pWhalSecur, pWhalSecur->reconfData.reconfDefaultKeyId, NULL, NULL) != OK)
			{
				WLAN_REPORT_ERROR(pWhalSecur->hReport, HAL_SECURITY_MODULE_LOG,  
					("whalSecur_KeysReconfig: ERROR whalSecur_DefaultKeyIdSet failure \n"));
				return (NOK);
			}   
		}
	} /* pWhalSecur->securityMode != RSN_CIPHER_NONE */


	/* set the encryption/decryption control on the HW*/   
	if (whalSecur_HwEncDecrEnable (pWhalSecur, pWhalSecur->reconfData.reconfHwEncEnable) != OK)
	{
		WLAN_REPORT_ERROR(pWhalSecur->hReport, HAL_SECURITY_MODULE_LOG,  
			("whalSecur_KeysReconfig: ERROR whalSecur_HwEncDecrEnable failure \n"));
		return (NOK);
	}   
	
	return (OK);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalSecur_Destroy
 *
 * Input    : 
 * Output   :									 
 * Process  : Unload the HAL security module
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalSecur_Destroy (TI_HANDLE hWhalSecur, UINT16 numOfStations)
{
	WHAL_SECURITY* pWhalSecur = (WHAL_SECURITY *)hWhalSecur;

	if (!pWhalSecur)
		return OK;
#ifdef CKIP_ENABLED
	whalExc_Destroy (pWhalSecur->pWhalExc);
	if (privacy_unload(pWhalSecur->pWhalPrivacy) != OK)
		WLAN_REPORT_ERROR(pWhalSecur->hReport, HAL_CTRL_MODULE_LOG,  (" whalSecur_Destroy: privacy_unload failure \n"));
#endif /* CKIP_ENABLED*/

	if (whalWpa_Destroy (pWhalSecur->pWhalWpa) != OK)
		WLAN_REPORT_ERROR(pWhalSecur->hReport, HAL_CTRL_MODULE_LOG,  (" whalSecur_Destroy: whalWpa_Destroy failure \n"));
	if (whalWep_Destroy (pWhalSecur->pWhalWep) != OK)
		WLAN_REPORT_ERROR(pWhalSecur->hReport, HAL_CTRL_MODULE_LOG,  (" whalSecur_Destroy: whalWep_Destroy failure \n"));

	if (pWhalSecur->reconfData.reconfKeys)
		os_memoryFree (pWhalSecur->hOs, pWhalSecur->reconfData.reconfKeys, 
				   (sizeof(securityKeys_t))*(numOfStations*NO_OF_RECONF_SECUR_KEYS_PER_STATION+NO_OF_EXTRA_RECONF_SECUR_KEYS));
	
	os_memoryFree (pWhalSecur->hOs, pWhalSecur, sizeof(WHAL_SECURITY));
	
	return (OK);
}

