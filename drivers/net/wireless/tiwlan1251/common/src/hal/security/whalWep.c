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
 * Function : whalWep_Create
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
TI_HANDLE whalWep_Create (TI_HANDLE hOs, TI_HANDLE hWhalCtrl)
{
	WHAL_WEP* pWhalWep;

	pWhalWep = (WHAL_WEP *)os_memoryAlloc (hOs, sizeof(WHAL_WEP));
	if (pWhalWep == NULL)
		return NULL;

	os_memoryZero (hOs, (void *)pWhalWep, sizeof(WHAL_WEP));

	pWhalWep->pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
	pWhalWep->hOs = hOs;
	
	return((TI_HANDLE)pWhalWep);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalWep_Config
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalWep_Config (TI_HANDLE hWhalWep, whalWep_config_t* pWhalWepCfg)
{
	WHAL_WEP* pWhalWep = (WHAL_WEP *)hWhalWep;

	pWhalWep->hMemMgr = pWhalWepCfg->hMemMgr;
	pWhalWep->hReport = pWhalWepCfg->hReport;

	/* YV - need to move here, the settings of ACXWEPOptions IE configuration!!*/
	/* which set the number of keys for which to reseve space in the WEP cache*/
	
	return (OK);
}			    
						    
/*
 * ----------------------------------------------------------------------------
 * Function : whalWep_KeyAdd
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalWep_KeyAdd (TI_HANDLE hWhalWep, securityKeys_t* pKey, void *CB_Func, TI_HANDLE CB_handle)
{
	WHAL_WEP* pWhalWep = (WHAL_WEP *)hWhalWep;
    
	/* Non WEP keys are trashed*/
	if (pKey->keyType != WEP_KEY)
		return (NOK);

    /* Check for mapping key or default key */
    if ( MAC_NULL(&pKey->macAddress) )
    {
        /* Configure the encKeys to the HW - default keys cache*/
        return (whal_hwCtrl_WepDefaultKeyAdd (pWhalWep->pWhalCtrl->pHwCtrl, pKey, CB_Func, CB_handle));
    } 
    else /* Use key mapping */
    {
        return (whal_hwCtrl_WepMappingKeyAdd (pWhalWep->pWhalCtrl->pHwCtrl, pKey, CB_Func, CB_handle));
    }
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalWep_KeyRemove
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalWep_KeyRemove (TI_HANDLE hWhalWep, securityKeys_t* pKey, void *CB_Func, TI_HANDLE CB_handle)
{
	WHAL_WEP* pWhalWep = (WHAL_WEP *)hWhalWep;

	/* Non WEP keys are trashed*/
	if (pKey->keyType != WEP_KEY)
		return (NOK);

    /* Check for mapping key or default key */
    if ( MAC_NULL(&pKey->macAddress) )
    {
	    return (whal_hwCtrl_WepDefaultKeyRemove (pWhalWep->pWhalCtrl->pHwCtrl, pKey, CB_Func, CB_handle));
    }
    else
    {
        return (whal_hwCtrl_WepMappingKeyRemove (pWhalWep->pWhalCtrl->pHwCtrl, pKey, CB_Func, CB_handle));
    }
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalWep_DefaultKeyIdSet
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalWep_DefaultKeyIdSet (TI_HANDLE hWhalWep, UINT8 aKeyId, void *CB_Func, TI_HANDLE CB_handle)
{
	WHAL_WEP *pWhalWep = (WHAL_WEP *)hWhalWep;

	/* Configure the default key id to the HW*/
	return (whal_hwCtrl_DefaultKeyIdSet (pWhalWep->pWhalCtrl->pHwCtrl, aKeyId, CB_Func, CB_handle));
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalWep_Destroy
 *
 * Input    : 
 * Output   :									 
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalWep_Destroy (TI_HANDLE hWhalWep)
{
	WHAL_WEP *pWhalWep = (WHAL_WEP *)hWhalWep;

	if (pWhalWep)
		os_memoryFree (pWhalWep->hOs, pWhalWep, sizeof(WHAL_WEP));
	
	return (OK);
}

