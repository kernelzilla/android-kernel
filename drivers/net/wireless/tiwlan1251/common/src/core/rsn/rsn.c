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


/** \file rsn.c
 *  \brief 802.11 rsniation SM source
 *
 *  \see rsnSM.h
 */

#include "osApi.h"
#include "paramOut.h"
#include "paramIn.h"
#include "utils.h"
#include "report.h"
#include "Ethernet.h"
#include "whalCtrl_api.h"
#include "whalCtrl_prm.h"
#include "DataCtrl_Api.h"
#include "memMngrEx.h"
#include "siteMgrApi.h"
#include "smeApi.h"
#include "mainSecSm.h"
#include "admCtrl.h"
#include "rsnApi.h"
#include "rsn.h"
#include "keyParser.h"
#include "EvHandler.h"
#include "TI_IPC_Api.h"
#include "smeSmApi.h"
#include "apConn.h"
#include "802_11Defs.h"
#include "public_infoele.h"

#ifdef EXC_MODULE_INCLUDED
#include "admCtrlWpa.h"
#include "excMngr.h"
#include "admCtrlExc.h"
#endif

/* Constants */

/* Enumerations */

/* Typedefs */

/* Structures */

/* External data definitions */

/* External functions definitions */

/* Global variables */

/* Local function prototypes */
TI_STATUS rsn_sendKeysNotSet(rsn_t *pRsn);
void rsn_groupReKeyTimeout(TI_HANDLE hRsn);
void rsn_micFailureReportTimeout(TI_HANDLE hRsn);
static rsn_siteBanEntry_t * findEntryForInsert(TI_HANDLE hRsn);
static rsn_siteBanEntry_t * findBannedSiteAndCleanup(TI_HANDLE hRsn, macAddress_t siteBssid);
/* Comment out the call to clearBannedSiteList due to fail in WiFi mic attack test */
/*static void clearBannedSiteList(TI_HANDLE hRsn);*/



/* functions */

/**
*
* rsn_Create - allocate memory for rsniation SM
*
* \b Description: 
*
* Allocate memory for rsniation SM. \n
*       Allocates memory for Rsniation context. \n
*       Allocates memory for rsniation timer. \n
*       Allocates memory for rsniation SM matrix. \n
*
* \b ARGS:
*
*  I   - hOs - OS context  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa rsn_mainSecKeysOnlyStop()
*/
TI_HANDLE rsn_create(TI_HANDLE hOs)
{
    rsn_t  *pRsn;

    /* allocate rsniation context memory */
    pRsn = (rsn_t*)os_memoryAlloc (hOs, sizeof(rsn_t));
    if (pRsn == NULL)
    {
        return NULL;
    }

    os_memoryZero (hOs, pRsn, sizeof(rsn_t));
    
    /* create admission control */
    pRsn->pAdmCtrl = admCtrl_create (hOs);
    if (pRsn->pAdmCtrl == NULL)
    {
        os_memoryFree (hOs, pRsn, sizeof(rsn_t));
        return NULL;
    }

    /* create main security SM */
    pRsn->pMainSecSm = mainSec_create (hOs);
    if (pRsn->pMainSecSm == NULL)
    {
        admCtrl_unload (pRsn->pAdmCtrl);
        os_memoryFree (hOs, pRsn, sizeof(rsn_t));
        return NULL;
    }

    pRsn->pKeyParser = pRsn->pMainSecSm->pKeyParser;
    
    pRsn->micFailureReportWaitTimer = os_timerCreate (hOs, rsn_micFailureReportTimeout, pRsn);
    if (pRsn->micFailureReportWaitTimer == NULL)
    {
        mainSec_unload (pRsn->pMainSecSm);
        admCtrl_unload (pRsn->pAdmCtrl);
        os_memoryFree (hOs, pRsn, sizeof(rsn_t));
        return NULL;
    }
    
    pRsn->micFailureReKeyTimer = os_timerCreate (hOs, rsn_groupReKeyTimeout, pRsn);
    if (pRsn->micFailureReKeyTimer == NULL)
    {
        os_timerDestroy (hOs, pRsn->micFailureReportWaitTimer);
        mainSec_unload (pRsn->pMainSecSm);
        admCtrl_unload (pRsn->pAdmCtrl);
        os_memoryFree (hOs, pRsn, sizeof(rsn_t));
        return NULL;
    }
    
    pRsn->hOs = hOs;
    
    return pRsn;
}


/**
*
* rsn_Unload - unload rsniation SM from memory
*
* \b Description: 
*
* Unload rsniation SM from memory
*
* \b ARGS:
*
*  I   - hRsn - rsniation SM context  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa rsn_mainSecKeysOnlyStop() 
*/
TI_STATUS rsn_unload (TI_HANDLE hRsn)
{
    rsn_t           *pRsn;
    TI_STATUS       status;

    if (hRsn == NULL)
    {
        return NOK;
    }

    pRsn = (rsn_t*)hRsn;

    os_timerStop (pRsn->hOs, pRsn->micFailureReportWaitTimer);
    os_timerDestroy (pRsn->hOs, pRsn->micFailureReportWaitTimer);    
    os_timerStop (pRsn->hOs, pRsn->micFailureReKeyTimer);
    os_timerDestroy (pRsn->hOs, pRsn->micFailureReKeyTimer);
    
    status = admCtrl_unload (pRsn->pAdmCtrl);
    status = mainSec_unload (pRsn->pMainSecSm);
    
    os_memoryFree (pRsn->hOs, hRsn, sizeof(rsn_t));

    return status;
}


/**
*
* rsn_smConfig - configure a new rsniation SM
*
* \b Description: 
*
* Configure a new rsniation SM.
*
* \b ARGS:
*
*  I   - hRsn - Rsniation SM context  \n
*  I   - hMlme - MLME SM context  \n
*  I   - hSiteMgr - Site manager context  \n
*  I   - hCtrlData - Control data context  \n
*  I   - hTxData - TX data context  \n
*  I   - hHalCtrl - Hal control context  \n
*  I   - hReport - Report context  \n
*  I   - hOs - OS context  \n
*  I   - rsnTimeout - Rsniation SM timeout \n
*  I   - rsnMaxCount - Max number of rsniation requests to send  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa rsn_Create, rsn_Unload
*/
TI_STATUS rsn_config (TI_HANDLE         hRsn,
                      TI_HANDLE         hTx,
                      TI_HANDLE         hRx,
                      TI_HANDLE         hConn,
                      TI_HANDLE         hMlme,
                      TI_HANDLE         hCtrlData,
                      TI_HANDLE         hWhalCtrl,
                      TI_HANDLE         hMemMgr,
                      TI_HANDLE         hSiteMgr,
                      TI_HANDLE         hReport,
                      TI_HANDLE         hOs,
                      TI_HANDLE         hExcMngr,
                      TI_HANDLE         hPowerMgr,
                      TI_HANDLE         hEvHandler,
                      TI_HANDLE         hSmeSm,
                      TI_HANDLE         hAPConn,
                      rsnInitParams_t   *pInitParam)
{
    rsn_t       *pRsn;
    TI_STATUS    status;
    UINT8        keyIndex;

    if (hRsn == NULL)
    {
        return NOK;
    } 

    pRsn = (rsn_t*)hRsn;

    pRsn->groupKeyUpdate = GROUP_KEY_UPDATE_FALSE;
    pRsn->PrivacyOptionImplemented = TRUE;
    
    pRsn->hTx = hTx;
    pRsn->hRx = hRx;
    pRsn->hConn = hConn;
    pRsn->hWhalCtrl = hWhalCtrl;
    pRsn->hCtrlData = hCtrlData;
    pRsn->hMemMgr = hMemMgr;
    pRsn->hSiteMgr= hSiteMgr;
    pRsn->hReport = hReport;
    pRsn->hOs = hOs;
    pRsn->hExcMngr = hExcMngr;
    pRsn->hEvHandler = hEvHandler;
    pRsn->hSmeSm = hSmeSm;
    pRsn->hAPConn = hAPConn;

    pRsn->setPaeConfig = rsn_setPaeConfig;
    pRsn->getNetworkMode = rsn_getNetworkMode;
    pRsn->setKey = rsn_setKey;
    pRsn->removeKey = rsn_removeKey;
    pRsn->reportStatus = rsn_reportStatus;
    pRsn->setDefaultKeyId = rsn_setDefaultKeyId;
    pRsn->defaultKeysOn = TRUE;
    pRsn->eapType = OS_EAP_TYPE_NONE;
    pRsn->numOfBannedSites = 0;

    /* config the admission control with the authentication suite selected.
       Admission control will configure the main security SM. */
    status = admCtrl_config (pRsn->pAdmCtrl, hMlme, hRx, hReport, hOs, pRsn, hExcMngr, hPowerMgr, hEvHandler, pInitParam);
    if (status != OK)
    {
        return status;
    }            

    /* Configure keys from registry */
    if (pInitParam->privacyOn)
    {
        pRsn->wepStaticKey = TRUE;
    }

    pRsn->defaultKeyId = pInitParam->defaultKeyId;
    for (keyIndex = 0; keyIndex < MAX_KEYS_NUM; keyIndex++)
    {
        os_memoryCopy (hOs, &pRsn->keys[keyIndex], &pInitParam->keys[keyIndex], sizeof(securityKeys_t));
        if (pRsn->keys[keyIndex].keyType != NULL_KEY)
        {
            pRsn->wepDefaultKeys[keyIndex] = TRUE;
        }
        pRsn->keys_en [keyIndex] = FALSE;
    }

    return status;
}


/**
*
* rsn_reconfig - re-configure a rsniation
*
* \b Description: 
*
* Re-configure rsniation 
*
* \b ARGS:
*
*  I   - hRsn - Rsniation SM context  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa rsn_Create, rsn_Unload
*/
TI_STATUS rsn_reconfig (TI_HANDLE hRsn)
{
    rsn_t  *pRsn = (rsn_t *)hRsn;
    UINT8   keyIndex;

    /* Mark all keys as removed */
    for (keyIndex = 0; keyIndex < MAX_KEYS_NUM; keyIndex++)
        pRsn->keys_en [keyIndex] = FALSE;       

    return OK;
}


/** 
*
* rsn_setDefaultKeys - 
*
* \b Description: 
*
* 
*
* \b ARGS:
*
*  I   - hRsn - Rsn SM context  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa rsn_Stop, rsn_Recv
*/
TI_STATUS rsn_setDefaultKeys(rsn_t *pRsn)
{
    TI_STATUS       status = OK;
    whalParamInfo_t whalParam;
    UINT8           keyIndex;

    for (keyIndex = 0; keyIndex < MAX_KEYS_NUM; keyIndex++)
    {
        /* Set the WEP key to the HAL */
        if (pRsn->wepDefaultKeys[keyIndex] /*pRsn->keys[keyIndex].encLen>0*/)
        {
            /* Change key type to WEP-key before setting*/
            pRsn->keys[keyIndex].keyType = WEP_KEY;

            status = pRsn->pMainSecSm->setKey (pRsn->pMainSecSm, &pRsn->keys[keyIndex]);

            if (status != OK)
            {
                WLAN_REPORT_ERROR(pRsn->hReport, RSN_MODULE_LOG,
                                  ("RSN: Setting key #%d failed \n", keyIndex));
                return status;
            }
        }
    }

    /* Now we configure default key ID to the HAL */
    if (pRsn->defaultKeyId < MAX_KEYS_NUM)
    {
        whalParam.paramType = HAL_CTRL_RSN_DEFAULT_KEY_ID_PARAM;
        whalParam.content.configureCmdCBParams.CB_buf = &pRsn->defaultKeyId;
        whalParam.content.configureCmdCBParams.CB_Func = NULL;
        whalParam.content.configureCmdCBParams.CB_handle = NULL;
        status = whalCtrl_SetParam (pRsn->hWhalCtrl, &whalParam); 

        WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG,
                           ("RSN: default key ID =%d \n", pRsn->defaultKeyId));
    }

    return status;
}


/** 
*
* rsn_Start - Start event for the rsniation SM
*
* \b Description: 
*
* Start event for the rsniation SM
*
* \b ARGS:
*
*  I   - hRsn - Rsniation SM context  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa rsn_Stop, rsn_Recv
*/
TI_STATUS rsn_start(TI_HANDLE hRsn)
{
    TI_STATUS           status;
    rsn_t               *pRsn;
    cipherSuite_e       suite;
    externalAuthMode_e  extAuthMode;
    whalParamInfo_t     whalParam;

    pRsn = (rsn_t*)hRsn;

    if (pRsn == NULL)
    {
        return NOK;
    }

    WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, ("rsn_start ...\n"));

    pRsn->rsnStartedTs = os_timeStampMs (pRsn->hOs);

    status = pRsn->pMainSecSm->start (pRsn->pMainSecSm);
    /* Set keys that need to be set */
    pRsn->defaultKeysOn = FALSE;
    pRsn->pAdmCtrl->getCipherSuite (pRsn->pAdmCtrl, &suite);
    pRsn->pAdmCtrl->getExtAuthMode (pRsn->pAdmCtrl, &extAuthMode);

    if (pRsn->wepStaticKey && ((suite == RSN_CIPHER_WEP) || (suite == RSN_CIPHER_CKIP)))
    {   /* set default WEP keys */
        status = rsn_sendKeysNotSet (pRsn);
        pRsn->eapType = OS_EAP_TYPE_NONE;
    }
    else if (suite == RSN_CIPHER_NONE && extAuthMode != RSN_EXT_AUTH_MODE_OPEN)
    {   /* remove previously WEP key for SHARED */
        pRsn->wepStaticKey = FALSE;
        status = rsn_removedDefKeys (pRsn);

        /* Set None to HAL */
        whalParam.paramType = HAL_CTRL_RSN_SECURITY_MODE_PARAM;
        whalParam.content.rsnEncryptionStatus = (halCtrl_CipherSuite_e)RSN_CIPHER_NONE;
        status = whalCtrl_SetParam (pRsn->hWhalCtrl, &whalParam);

    }
    else if (suite==RSN_CIPHER_NONE)
    {
        pRsn->eapType = OS_EAP_TYPE_NONE;
    }

    return status;
}


TI_STATUS rsn_sendKeysNotSet(rsn_t *pRsn)
{
    UINT8           keyIndex;
    OS_802_11_KEY   rsnOsKey;
    TI_STATUS       status = OK;
    
    for (keyIndex = 0; keyIndex < MAX_KEYS_NUM; keyIndex++)
    {
        if (pRsn->wepDefaultKeys[keyIndex])
        {
            rsnOsKey.KeyIndex  = pRsn->keys[keyIndex].keyIndex;
            rsnOsKey.KeyLength = pRsn->keys[keyIndex].encLen;
            rsnOsKey.Length    = sizeof(rsnOsKey);

            /* Change key type to WEP-key before setting*/
            pRsn->keys[keyIndex].keyType = WEP_KEY;

            os_memoryCopy (pRsn->hOs, rsnOsKey.BSSID, 
                           (void *)pRsn->keys[keyIndex].macAddress.addr, 
                           MAC_ADDR_LEN);
            os_memoryCopy (pRsn->hOs, &rsnOsKey.KeyRSC, 
                           (void *)pRsn->keys[keyIndex].keyRsc, 
                           KEY_RSC_LEN);
            os_memoryCopy (pRsn->hOs, rsnOsKey.KeyMaterial, 
                           (void *)pRsn->keys[keyIndex].encKey, 
                           MAX_KEY_LEN /*pRsn->keys[keyIndex].encLen*/);
           
            /* Set WEP transmit key mask on the default key */
            if (keyIndex == pRsn->defaultKeyId)
            {
                rsnOsKey.KeyIndex |= 0x80000000;
            }

            status = pRsn->pKeyParser->recv (pRsn->pKeyParser, (UINT8*)&rsnOsKey, sizeof(rsnOsKey));
        }
    }

    return status;
}


TI_STATUS rsn_removedDefKeys (TI_HANDLE hRsn)
{
    UINT8  keyIndex;
    rsn_t  *pRsn = (rsn_t*)hRsn;
    
    WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                            ("rsn_removedDefKeys Enter \n"));
    
    for (keyIndex = 0; keyIndex < MAX_KEYS_NUM; keyIndex++)
    {
        securityKeys_t   key;

        WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                                ("rsn_removedDefKeys, Remove keyId=%d\n", keyIndex));
       
		pRsn->wepDefaultKeys[keyIndex] = FALSE;
        os_memoryCopy (pRsn->hOs, &key, &pRsn->keys[keyIndex], sizeof(securityKeys_t));
        pRsn->removeKey (pRsn, &key);
       
        /* Set WEP transmit key mask on the default key */
        if (keyIndex == pRsn->defaultKeyId)
        {
            pRsn->defaultKeyId = 0;
        }
    }
	/* Clear the band site list */
    /* Comment out the call to clearBannedSiteList due to fail in WiFi mic attack test */
    /*clearBannedSiteList(hRsn);*/

    return OK;
}


/**
*
* rsn_Stop - Stop event for the rsniation SM
*
* \b Description: 
*
* Stop event for the rsniation SM
*
* \b ARGS:
*
*  I   - hRsn - Rsniation SM context  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa rsn_Start, rsn_Recv
*/
TI_STATUS rsn_stop (TI_HANDLE hRsn, BOOL removeKeys)
{
    TI_STATUS        status;
    rsn_t           *pRsn;
    UINT8            keyIndex;
    securityKeys_t   key;

    pRsn = (rsn_t*)hRsn;

    if (pRsn == NULL)
    {
        return NOK;
    }
    
    WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                            ("RSN: calling STOP... removeKeys=%d\n", removeKeys));

    for (keyIndex = 0; keyIndex < MAX_KEYS_NUM; keyIndex++)
    {
        os_memoryCopy (pRsn->hOs, &key, &pRsn->keys[keyIndex], sizeof(securityKeys_t));

        if (!pRsn->wepDefaultKeys[keyIndex])
        {	/* Remove only dynamic keys. Default keys are removed by calling: rsn_removedDefKeys() */
            WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                                    ("rsn_stop, Remove keyIndex=%d, key.keyIndex=%d\n",keyIndex, key.keyIndex));
            
            WLAN_REPORT_HEX_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, (UINT8 *)key.macAddress.addr, 6);

            pRsn->removeKey (pRsn, &key);
        }

    }

    os_timerStop (pRsn->hOs, pRsn->micFailureReportWaitTimer);

    /* Stop the pre-authentication timer in case we are disconnecting */
    os_timerStop (pRsn->hOs, pRsn->pAdmCtrl->preAuthTimerWpa2);

    status = pRsn->pMainSecSm->stop (pRsn->pMainSecSm);

    pRsn->groupKeyUpdate = GROUP_KEY_UPDATE_FALSE;
    pRsn->defaultKeysOn = TRUE;

    if (removeKeys)
    {   /* reset PMKID list if exist */
        pRsn->pAdmCtrl->resetPmkidList (pRsn->pAdmCtrl);
    }

    return status;
}


/**
*
* rsn_GetParam - Get a specific parameter from the rsniation SM
*
* \b Description: 
*
* Get a specific parameter from the rsniation SM.
*
* \b ARGS:
*
*  I   - hRsn - Rsniation SM context  \n
*  I/O - pParam - Parameter \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa rsn_Start, rsn_Stop
*/
TI_STATUS rsn_getParam(TI_HANDLE hRsn, paramInfo_t *pParam)
{
    rsn_t      *pRsn;
    TI_STATUS   status = OK;

    pRsn = (rsn_t*)hRsn;

    if ((pRsn == NULL) || (pParam == NULL))
    {
        return NOK;
    }

    switch (pParam->paramType)
    {
    case RSN_PRIVACY_OPTION_IMPLEMENTED_PARAM:
        pParam->content.rsnPrivacyOptionImplemented = TRUE;
        break;

    case RSN_KEY_PARAM:
        pParam->content.pRsnKey = &pRsn->keys[pParam->content.pRsnKey->keyIndex];
        if (pParam->content.pRsnKey->keyIndex == pRsn->defaultKeyId)
        {
            pParam->content.pRsnKey->keyIndex |= 0x80000000;
            WLAN_REPORT_WARNING(pRsn->hReport, RSN_MODULE_LOG,  ("default Key: %d\n", pRsn->defaultKeyId));
        }
        break;

    case RSN_SECURITY_STATE_PARAM:
        status = pRsn->pMainSecSm->getAuthState (pRsn->pMainSecSm, (TIWLN_SECURITY_STATE*)&(pParam->content.rsnAuthState));
        break;

    case RSN_ENCRYPTION_STATUS_PARAM: 
        status = pRsn->pAdmCtrl->getCipherSuite (pRsn->pAdmCtrl, &pParam->content.rsnEncryptionStatus);
        break;

    case RSN_EXT_AUTHENTICATION_MODE:
        status = pRsn->pAdmCtrl->getExtAuthMode (pRsn->pAdmCtrl, &pParam->content.rsnExtAuthneticationMode);
        break;

    case RSN_MIXED_MODE:
        status = pRsn->pAdmCtrl->getMixedMode (pRsn->pAdmCtrl, &pParam->content.rsnMixedMode);
        break;

    case RSN_AUTH_ENCR_CAPABILITY:
        status = pRsn->pAdmCtrl->getAuthEncrCap(pRsn->pAdmCtrl, pParam->content.pRsnAuthEncrCapability);
        break;

    case RSN_PMKID_LIST:
        pParam->content.rsnPMKIDList.Length = pParam->paramLength;
        status = pRsn->pAdmCtrl->getPmkidList (pRsn->pAdmCtrl, &pParam->content.rsnPMKIDList);
        pParam->paramLength = pParam->content.rsnPMKIDList.Length + 2 * sizeof(UINT32);
        break;

    case RSN_PRE_AUTH_STATUS:
        {
            UINT8 cacheIndex;
        
            pParam->content.rsnPreAuthStatus = pRsn->pAdmCtrl->getPreAuthStatus (pRsn->pAdmCtrl, &pParam->content.rsnApMac, &cacheIndex);
        }
        break;

    case  RSN_WPA_PROMOTE_AVAILABLE_OPTIONS:
        status = pRsn->pAdmCtrl->getWPAMixedModeSupport (pRsn->pAdmCtrl, &pParam->content.rsnWPAMixedModeSupport);
        WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                               ("RSN: Get WPA Mixed MODE support  %d \n",pParam->content.rsnWPAMixedModeSupport));
        break;

    case RSN_WPA_PROMOTE_OPTIONS:
        status = pRsn->pAdmCtrl->getPromoteFlags (pRsn->pAdmCtrl, 
                                                  &pParam->content.rsnWPAPromoteFlags);
                WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                               ("RSN: Get WPA promote flags  %d \n",pParam->content.rsnWPAPromoteFlags));
        
        break;

#ifdef EXC_MODULE_INCLUDED
    case RSN_EXC_NETWORK_EAP:
        status = pRsn->pAdmCtrl->getNetworkEap (pRsn->pAdmCtrl, &pParam->content.networkEap);
        break;
#endif
    case RSN_EAP_TYPE:
        pParam->content.eapType = pRsn->eapType;
        WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                          ("RSN: Get RSN_EAP_TYPE eapType  %d \n", 
                          pParam->content.eapType));  
        break;

    case WPA_801_1X_AKM_EXISTS:

        status = pRsn->pAdmCtrl->get802_1x_AkmExists(pRsn->pAdmCtrl, &pParam->content.wpa_802_1x_AkmExists);
        WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                          ("RSN: Get WPA_801_1X_AKM_EXISTS  %d \n", 
                          pParam->content.wpa_802_1x_AkmExists));  
        break;

    case RSN_DEFAULT_KEY_ID:
        pParam->content.rsnDefaultKeyID = pRsn->defaultKeyId;
        WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
            ("RSN: Get RSN_DEFAULT_KEY_ID  %d \n", 
            pParam->content.rsnDefaultKeyID));
        break;

    default:
        return NOK;
    }
    
    return status;
}


/**
*
* rsn_SetParam - Set a specific parameter to the rsniation SM
*
* \b Description: 
*
* Set a specific parameter to the rsniation SM.
*
* \b ARGS:
*
*  I   - hRsn - Rsniation SM context  \n
*  I/O - pParam - Parameter \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa rsn_Start, rsn_Stop
*/
TI_STATUS rsn_setParam (TI_HANDLE hRsn, paramInfo_t *pParam)
{
    rsn_t               *pRsn;
    TI_STATUS           status=OK;
    whalParamInfo_t     whalParam;

    pRsn = (rsn_t*)hRsn;

    if ((pRsn == NULL) || (pParam == NULL))
    {
        return NOK;
    }

    WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                          ("RSN: Set rsn_setParam   %X \n", 
                          pParam->paramType));

    switch (pParam->paramType)
    {

    case RSN_DEFAULT_KEY_ID:
    {
        UINT8  defKeyId, i;

        defKeyId = pParam->content.rsnDefaultKeyID;
        
        if(defKeyId >= MAX_KEYS_NUM)
        {
            WLAN_REPORT_ERROR(pRsn->hReport, RSN_MODULE_LOG, 
                    ("RSN: Error - the value of the default Key Id  is incorrect \n"));
            return NOK;
        }

        /* Clean transmit flag (1 in the bit31) in the previous default key */
        for(i = 0; i < MAX_KEYS_NUM; i++)
        {
            pRsn->keys[i].keyIndex &= 0x7FFFFFFF;
        }

        /* Set the default key ID value in the RSN data structure */ 
        pRsn->defaultKeyId = defKeyId;

        /* Set the default key ID in the HAL */
        whalParam.paramType = HAL_CTRL_RSN_DEFAULT_KEY_ID_PARAM;
        whalParam.content.configureCmdCBParams.CB_buf = &pRsn->defaultKeyId;
        whalParam.content.configureCmdCBParams.CB_Func = NULL;
        whalParam.content.configureCmdCBParams.CB_handle = NULL;
        status = whalCtrl_SetParam (pRsn->hWhalCtrl, &whalParam); 

        WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG,
                        ("RSN: default key ID =%d \n", pRsn->defaultKeyId));


        status = RE_SCAN_NEEDED;
        break;
    }

    case RSN_ADD_KEY_PARAM:
    {
        UINT8           keyIndex, i = 0;
        cipherSuite_e   cipherSuite;

        status = pRsn->pAdmCtrl->getCipherSuite (pRsn->pAdmCtrl, &cipherSuite);
        if (status !=OK)
        {
            return status;
        }
        
        WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                      ("RSN: Set RSN_ADD_KEY_PARAM KeyIndex  %x , keyLength=%d\n", 
                      pParam->content.rsnOsKey.KeyIndex,pParam->content.rsnOsKey.KeyLength));
        keyIndex = (UINT8)pParam->content.rsnOsKey.KeyIndex;
        if (keyIndex >= MAX_KEYS_NUM)
        {
            return NOK;
        }
       
        status = pRsn->pKeyParser->recv (pRsn->pKeyParser, (UINT8*)&pParam->content.rsnOsKey, sizeof(pParam->content.rsnOsKey));

        if (status==STATUS_BAD_KEY_PARAM)
        {
            return NOK;
        }
        /* If the Key is not BAD, it may be that WEP key is sent before WEP status is set, 
            save the key, and set it later at rsn_start */
         pRsn->keys[keyIndex].keyIndex = pParam->content.rsnOsKey.KeyIndex;
         pRsn->keys[keyIndex].encLen = pParam->content.rsnOsKey.KeyLength;
         os_memoryCopy (pRsn->hOs, (void *)pRsn->keys[keyIndex].macAddress.addr, pParam->content.rsnOsKey.BSSID, MAC_ADDR_LEN);
         os_memoryCopy (pRsn->hOs, (void *)pRsn->keys[keyIndex].keyRsc, (UINT8*)&(pParam->content.rsnOsKey.KeyRSC), KEY_RSC_LEN);
         os_memoryCopy (pRsn->hOs, (void *)pRsn->keys[keyIndex].encKey, pParam->content.rsnOsKey.KeyMaterial, MAX_KEY_LEN);
           
        /* Process the transmit flag (31-st bit of keyIndex).        */
        /* If the added key has the TX bit set to TRUE (i.e. the key */
        /* is the new transmit key (default key), update             */
        /* RSN data def.key Id and clean this bit in all other keys  */
        if (pParam->content.rsnOsKey.KeyIndex & 0x80000000)
        {
            pRsn->defaultKeyId = keyIndex;
            
            for (i = 0; i < MAX_KEYS_NUM; i ++)
            {
                if (i != keyIndex)
                {
                    pRsn->keys[i].keyIndex &= 0x7FFFFFFF;
                }
            }
        }
        
        if (pRsn->defaultKeysOn)
        {   /* This is a WEP default key */
            WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                         ("RSN_ADD_KEY_PARAM, Default key configured - keyIndex=%d-TRUE\n", keyIndex));

            pRsn->wepDefaultKeys[keyIndex] = TRUE;
            pRsn->wepStaticKey = TRUE;
            status = OK;
        }
        break;
    }
    case RSN_REMOVE_KEY_PARAM:
    {
        UINT8           keyIndex;
        cipherSuite_e   cipherSuite;

        status = pRsn->pAdmCtrl->getCipherSuite (pRsn->pAdmCtrl, &cipherSuite);
        if (status !=OK)
        {
            return status;
        }
        /*if (cipherSuite == RSN_CIPHER_NONE)
        {
            WLAN_REPORT_ERROR(pRsn->hReport, RSN_MODULE_LOG, 
                          ("RSN: Error Remove Wep/Key when no encryption \n"));
            return NOK;
        }*/

        WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                      ("RSN: Set RSN_REMOVE_KEY_PARAM KeyIndex  %x \n", 
                      pParam->content.rsnOsKey.KeyIndex));
        keyIndex = (UINT8)pParam->content.rsnOsKey.KeyIndex;
        if (keyIndex >= MAX_KEYS_NUM)
        {
            return NOK;
        }
        
        status = pRsn->pKeyParser->remove (pRsn->pKeyParser, 
                                           (UINT8*)&pParam->content.rsnOsKey, 
                                           sizeof(pParam->content.rsnOsKey));

        if (status == OK)
        {
            pRsn->keys[keyIndex].keyType = NULL_KEY;
            pRsn->keys[keyIndex].keyIndex &= 0x000000FF;
        }

        break;
    }
    
    case RSN_ENCRYPTION_STATUS_PARAM: 
        {
            cipherSuite_e   cipherSuite;

            WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                              ("RSN: Set RSN_ENCRYPTION_STATUS_PARAM rsnEncryptionStatus  %d \n", 
                              pParam->content.rsnEncryptionStatus));

            pRsn->pAdmCtrl->getCipherSuite (pRsn->pAdmCtrl, &cipherSuite);
            if (cipherSuite != pParam->content.rsnEncryptionStatus)
            {
                status = pRsn->pAdmCtrl->setUcastSuite (pRsn->pAdmCtrl, pParam->content.rsnEncryptionStatus);
                status = pRsn->pAdmCtrl->setBcastSuite (pRsn->pAdmCtrl, pParam->content.rsnEncryptionStatus);
                WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, (" status = %d \n", status));
            }
            pRsn->defaultKeysOn = TRUE;
        }
        break;

    case RSN_EXT_AUTHENTICATION_MODE:
        {
            externalAuthMode_e  extAuthMode;

            pRsn->pAdmCtrl->getExtAuthMode (pRsn->pAdmCtrl, &extAuthMode);
            if (pParam->content.rsnExtAuthneticationMode!=extAuthMode)
            {
                WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                                  ("RSN: Set RSN_EXT_AUTHENTICATION_MODE rsnExtAuthneticationMode  %d \n", 
                                  pParam->content.rsnExtAuthneticationMode));  
                
                /*WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                                        ("RSN: remove all Keys\n"));

                for (keyIndex=0; keyIndex<MAX_KEYS_NUM; keyIndex++)
                {
                    os_memoryCopy(pRsn->hOs, &key, &pRsn->keys[keyIndex], sizeof(securityKeys_t));
                    pRsn->removeKey(pRsn, &key);

                }*/

                status = pRsn->pAdmCtrl->setExtAuthMode (pRsn->pAdmCtrl, pParam->content.rsnExtAuthneticationMode);
            }
            pRsn->defaultKeysOn = TRUE;
        }
        break;

#ifdef EXC_MODULE_INCLUDED
    case RSN_EXC_NETWORK_EAP:
        {
            OS_EXC_NETWORK_EAP      networkEap;

            pRsn->pAdmCtrl->getNetworkEap (pRsn->pAdmCtrl, &networkEap);
            if (networkEap != pParam->content.networkEap)
            {
                WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                                      ("RSN: Set RSN_EXC_NETWORK_EAP networkEap  %d \n", 
                                      pParam->content.networkEap));  
                
                status = pRsn->pAdmCtrl->setNetworkEap (pRsn->pAdmCtrl, pParam->content.networkEap);
                if (status == OK) 
                {
                    /*status = RE_SCAN_NEEDED;*/
                }
            }
        }
        break;
#endif
    case RSN_MIXED_MODE:
        {
            BOOL mixedMode;
        
            pRsn->pAdmCtrl->getMixedMode (pRsn->pAdmCtrl, &mixedMode);
            if (mixedMode!=pParam->content.rsnMixedMode)
            {
                status = pRsn->pAdmCtrl->setMixedMode (pRsn->pAdmCtrl, pParam->content.rsnMixedMode);
                
                WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                                  ("RSN: Set RSN_MIXED_MODE mixedMode  %d, status=%d \n",
                                   pParam->content.rsnMixedMode, status));
            }
            break;
        }

    case RSN_PMKID_LIST:
        WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                               ("RSN: Set RSN_PMKID_LIST \n"));

        WLAN_REPORT_HEX_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                               (UINT8*)&pParam->content.rsnPMKIDList ,pParam->content.rsnPMKIDList.Length);
         status = pRsn->pAdmCtrl->setPmkidList (pRsn->pAdmCtrl, 
                                                &pParam->content.rsnPMKIDList);
         if(status == OK)
         {
            WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                                  ("RSN: Set RSN_PMKID_LIST:   %d PMKID entries has been added to the cache.\n",
                                   pParam->content.rsnPMKIDList.BSSIDInfoCount));
         }
         else
         {
            WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                                  ("RSN: Set RSN_PMKID_LIST failure"));
         }
        break;

    case RSN_WPA_PROMOTE_OPTIONS:
         status = pRsn->pAdmCtrl->setPromoteFlags (pRsn->pAdmCtrl, 
                                                   pParam->content.rsnWPAPromoteFlags);
         if(status == OK)
         {
            WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
             ("RSN: Set WPA promote options:  %d \n", pParam->content.rsnWPAPromoteFlags));
         }
         else
         {
            WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                                  ("RSN: Set WPA promote options failure"));
         }
        break;

    case RSN_EAP_TYPE:
        WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                          ("RSN: Set RSN_EAP_TYPE eapType  %d \n", 
                          pParam->content.eapType));  

        pRsn->eapType = pParam->content.eapType;
		pRsn->defaultKeysOn = TRUE;
        break;

    default:
        return NOK;
    }

    return status;
}


/**
*
* rsn_eventRecv - Set a specific parameter to the rsniation SM
*
* \b Description: 
*
* Set a specific parameter to the rsniation SM.
*
* \b ARGS:
*
*  I   - hRsn - Rsniation SM context  \n
*  I/O - pParam - Parameter \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa rsn_Start, rsn_Stop
*/
TI_STATUS rsn_reportStatus (rsn_t *pRsn, TI_STATUS rsnStatus)
{
    TI_STATUS           status = OK;
    paramInfo_t         param;
    externalAuthMode_e  extAuthMode;

    if (pRsn == NULL)
    {
        return NOK;
    }
    
    if (rsnStatus == OK)
    {
        /* set EAPOL encryption status according to authentication protocol */
        pRsn->rsnCompletedTs = os_timeStampMs (pRsn->hOs);
        
        status = pRsn->pAdmCtrl->getExtAuthMode (pRsn->pAdmCtrl, &extAuthMode);
        if (status != OK)
        {
            return status;
        }

        if (extAuthMode >= RSN_EXT_AUTH_MODE_WPA)
        {
            param.content.txDataEapolEncryptionStatus = TRUE;
        } else {
            param.content.txDataEapolEncryptionStatus = FALSE;
        }

        param.paramType = TX_DATA_EAPOL_ENCRYPTION_STATUS_PARAM;
        txData_setParam (pRsn->hTx, &param);
        
        /* set WEP invoked mode according to cipher suite */
        switch (pRsn->paeConfig.unicastSuite)
        {
        case RSN_CIPHER_NONE:
            param.content.txDataCurrentPrivacyInvokedMode = FALSE;
            break;
        
        default:
            param.content.txDataCurrentPrivacyInvokedMode = TRUE;
            break;
        }

        param.paramType = TX_DATA_CURRENT_PRIVACY_INVOKE_MODE_PARAM;
        txData_setParam (pRsn->hTx, &param);
        /* The value of exclude unencrypted should be as privacy invoked */
        param.paramType = RX_DATA_EXCLUDE_UNENCRYPTED_PARAM;
        rxData_setParam (pRsn->hRx, &param);
        
        param.paramType = RX_DATA_EXCLUDE_BROADCAST_UNENCRYPTED_PARAM;
        if (pRsn->pAdmCtrl->mixedMode)
        {   /* do not exclude Broadcast packets */
            param.content.txDataCurrentPrivacyInvokedMode = FALSE;
        }
        rxData_setParam (pRsn->hRx, &param);
    } 

    else 
        rsnStatus = (TI_STATUS)STATUS_SECURITY_FAILURE;

    status = conn_reportRsnStatus (pRsn->hConn, (mgmtStatus_e)rsnStatus);

    if (status!=OK)
    {
        return status;
    }
    
    if (rsnStatus == OK)
    {
        EvHandlerSendEvent (pRsn->hEvHandler, IPC_EVENT_AUTH_SUCC, NULL, 0);
    }

    WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                          ("RSN: rsn_reportStatus \n"));

    return OK;
}


/**
*
* rsn_eventRecv - Set a specific parameter to the rsniation SM
*
* \b Description: 
*
* Set a specific parameter to the rsniation SM.
*
* \b ARGS:
*
*  I   - hRsn - Rsniation SM context  \n
*  I/O - pParam - Parameter \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa rsn_Start, rsn_Stop
*/
TI_STATUS rsn_setPaeConfig(rsn_t *pRsn, rsn_paeConfig_t *pPaeConfig)
{
    TI_STATUS           status;
    mainSecInitData_t   initData;

    if ((pRsn == NULL) || (pPaeConfig == NULL))
    {
        return NOK;
    }

    WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                            ("RSN: Calling set PAE config..., unicastSuite = %d, broadcastSuite = %d \n", 
                             pPaeConfig->unicastSuite, pPaeConfig->broadcastSuite));
    
    os_memoryCopy(pRsn->hOs, &pRsn->paeConfig, pPaeConfig, sizeof(rsn_paeConfig_t));

    initData.pPaeConfig = &pRsn->paeConfig;

    status = mainSec_config (pRsn->pMainSecSm, 
                             &initData, 
                             pRsn, 
                             pRsn->hReport, 
                             pRsn->hOs, 
                             pRsn->hCtrlData,
                             pRsn->hEvHandler, 
                             pRsn->hConn, 
                             pRsn->hWhalCtrl);

    return status;
}


/**
*
* rsn_eventRecv - Set a specific parameter to the rsniation SM
*
* \b Description: 
*
* Set a specific parameter to the rsniation SM.
*
* \b ARGS:
*
*  I   - hRsn - Rsniation SM context  \n
*  I/O - pParam - Parameter \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa rsn_Start, rsn_Stop
*/
TI_STATUS rsn_getNetworkMode(rsn_t *pRsn, rsn_networkMode_t *pNetMode)
{
    paramInfo_t     param;
    TI_STATUS       status;

    param.paramType = CTRL_DATA_CURRENT_BSS_TYPE_PARAM;

    status =  ctrlData_getParam (pRsn->hCtrlData, &param);

    if (status == OK)
    {
        if (param.content.ctrlDataCurrentBssType == BSS_INFRASTRUCTURE)
        {
            *pNetMode = RSN_INFRASTRUCTURE;
        } 
        else 
        {
            *pNetMode = RSN_IBSS;
        }
    }
    else 
    {
        return NOK;
    }

    return OK;
}


/**
*
* rsn_eventRecv - Set a specific parameter to the rsniation SM
*
* \b Description: 
*
* Set a specific parameter to the rsniation SM.
*
* \b ARGS:
*
*  I   - hRsn - Rsniation SM context  \n
*  I/O - pParam - Parameter \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa rsn_Start, rsn_Stop
*/
TI_STATUS rsn_evalSite(TI_HANDLE hRsn, rsnData_t *pRsnData, bssType_e bssType, macAddress_t bssid, UINT32 *pMetric)
{
    rsn_t       *pRsn;
    TI_STATUS       status;

    if ((pRsnData == NULL) || (hRsn == NULL))
    {
        *pMetric = 0;
        return NOK;
    }

    pRsn = (rsn_t*)hRsn;

    if (rsn_isSiteBanned(hRsn, bssid) == TRUE)
    {
        *pMetric = 0;
        WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, ("%s: Site is banned!\n", __FUNCTION__));
        return NOK;
    }

    status = pRsn->pAdmCtrl->evalSite (pRsn->pAdmCtrl, pRsnData, bssType, pMetric);

    WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, ("%s: pMetric=%d status=%d\n", __FUNCTION__, *pMetric, status));

    return status;
}


/**
*
* rsn_getInfoElement - 
*
* \b Description: 
*
* Get the RSN information element.
*
* \b ARGS:
*
*  I   - hRsn - Rsn SM context  \n
*  I/O - pRsnIe - Pointer to the return information element \n
*  I/O - pRsnIeLen - Pointer to the returned IE's length \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa 
*/
TI_STATUS rsn_getInfoElement(TI_HANDLE hRsn, UINT8 *pRsnIe, UINT8 *pRsnIeLen)
{
    rsn_t       *pRsn;
    TI_STATUS   status;

    if ((hRsn == NULL) || (pRsnIe == NULL) || (pRsnIeLen == NULL))
    {
        return NOK;
    }

    pRsn = (rsn_t*)hRsn;

    status = pRsn->pAdmCtrl->getInfoElement (pRsn->pAdmCtrl, pRsnIe, pRsnIeLen);

    WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, ("rsn_getInfoElement pRsnIeLen= %d\n",*pRsnIeLen));

    return status;   
}


#ifdef EXC_MODULE_INCLUDED
/**
*
* rsn_getExcExtendedInfoElement - 
*
* \b Description: 
*
* Get the Aironet information element.
*
* \b ARGS:
*
*  I   - hRsn - Rsn SM context  \n
*  I/O - pRsnIe - Pointer to the return information element \n
*  I/O - pRsnIeLen - Pointer to the returned IE's length \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa 
*/
TI_STATUS rsn_getExcExtendedInfoElement(TI_HANDLE hRsn, UINT8 *pRsnIe, UINT8 *pRsnIeLen)
{
    rsn_t       *pRsn;
    TI_STATUS   status;

    if ((hRsn == NULL) || (pRsnIe == NULL) || (pRsnIeLen == NULL))
    {
        return NOK;
    }

    pRsn = (rsn_t*)hRsn;

    status = admCtrlExc_getInfoElement (pRsn->pAdmCtrl, pRsnIe, pRsnIeLen);

    WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, ("rsn_getExcExtendedInfoElement pRsnIeLen= %d\n",*pRsnIeLen));

    return status;    
}
#endif


/**
*
* rsn_eventRecv - Set a specific parameter to the rsniation SM
*
* \b Description: 
*
* Set a specific parameter to the rsniation SM.
*
* \b ARGS:
*
*  I   - hRsn - Rsniation SM context  \n
*  I/O - pParam - Parameter \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa rsn_Start, rsn_Stop
*/
TI_STATUS rsn_setSite(TI_HANDLE hRsn, rsnData_t *pRsnData, UINT8 *pAssocIe, UINT8 *pAssocIeLen)
{
    rsn_t      *pRsn;
    TI_STATUS   status;

    if ((pRsnData == NULL) || (hRsn == NULL))
    {
        *pAssocIeLen = 0;
        return NOK;
    }

    pRsn = (rsn_t*)hRsn;

    status = pRsn->pAdmCtrl->setSite (pRsn->pAdmCtrl, pRsnData, pAssocIe, pAssocIeLen);

    WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, ("rsn_setSite ieLen= %d\n",pRsnData->ieLen));
    return status;
}


TI_STATUS rsn_setKey (rsn_t *pRsn, securityKeys_t *pKey)
{
    TI_STATUS           status = OK;
    whalParamInfo_t     whalParam;
    paramInfo_t         param;
    UINT8               keyIndex;
	BOOL				macIsBroadcast = FALSE;

    keyIndex = (UINT8)pKey->keyIndex;
    if ((pRsn == NULL) || (pKey == NULL) || (keyIndex >= MAX_KEYS_NUM))
    {
        return NOK;
    }

    /* 
     * In full driver we use only WEP default keys. That's why we make sure that the macAddress is cleared.
     * In GWSI we use WEP mapping key if the macAddress is not NULL.
     */
    if (pKey->keyType == WEP_KEY)
    {
        os_memoryZero(pRsn->hOs,(void*)pKey->macAddress.addr,
           sizeof(macAddress_t)); 
    }

    if (pKey->keyType != NULL_KEY)
    {
        /* set the size to reserve for encryption to the tx */
        /* update this parameter only in accordance with pairwise key setting */
        if (!MAC_BROADCAST((&pKey->macAddress)))
        {
            param.paramType = TX_DATA_ENCRYPTION_FIELD_SIZE;
            switch (pKey->keyType)
            {
                case TKIP_KEY:
                    param.content.txDataEncryptionFieldSize = IV_FIELD_SIZE;
                    break;
                case AES_KEY:
                    param.content.txDataEncryptionFieldSize = AES_AFTER_HEADER_FIELD_SIZE;
                    break;
                case NULL_KEY:
                case WEP_KEY:
                case EXC_KEY:
                default:
                    param.content.txDataEncryptionFieldSize = 0;
                    break;
            }

            txData_setParam (pRsn->hTx, &param);
        }
		macIsBroadcast = MAC_BROADCAST((&pKey->macAddress));
		if ((pRsn->keys[keyIndex].keyType != NULL_KEY )&&
			macIsBroadcast && !MAC_BROADCAST((&pRsn->keys[keyIndex].macAddress)))
		{	/* In case a new Group key is set instead of a Unicast key, 
				first remove the UNIcast key from FW */
			rsn_removeKey(pRsn, &pRsn->keys[keyIndex]);
		}
        pRsn->keys[keyIndex].keyType = pKey->keyType;
		pRsn->keys[keyIndex].keyIndex = keyIndex;
        whalParam.paramType = HAL_CTRL_RSN_KEY_ADD_PARAM;
        whalParam.content.configureCmdCBParams.CB_buf = (UINT8*) pKey;
        whalParam.content.configureCmdCBParams.CB_Func = NULL;
        whalParam.content.configureCmdCBParams.CB_handle = NULL;

        if (macIsBroadcast)
        {
            WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                                    ("RSN: rsn_setKey, Group ReKey timer started\n"));
            os_timerStop (pRsn->hOs, pRsn->micFailureReKeyTimer);
            os_timerStart (pRsn->hOs, pRsn->micFailureReKeyTimer, RSN_MIC_FAILURE_RE_KEY, FALSE);
            pRsn->groupKeyUpdate = GROUP_KEY_UPDATE_TRUE;
        }
 
        /* Mark key as added */
        pRsn->keys_en [keyIndex] = TRUE;

        status = whalCtrl_SetParam (pRsn->hWhalCtrl, &whalParam);
    }
    
    WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                            ("RSN: rsn_setKey, KeyType=%d, KeyId = 0x%lx,encLen=0x%x\n",
                             pKey->keyType,pKey->keyIndex, pKey->encLen));

    WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, ("\nEncKey = "));

    WLAN_REPORT_HEX_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, (UINT8 *)pKey->encKey, pKey->encLen);

    if (pKey->keyType != WEP_KEY)
    { 
        WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, ("\nMac address = "));
        WLAN_REPORT_HEX_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, (UINT8 *)pKey->macAddress.addr, MAC_ADDR_LEN);
        WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, ("\nRSC = "));
        WLAN_REPORT_HEX_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, (UINT8 *)pKey->keyRsc, KEY_RSC_LEN);
        WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, ("\nMic RX = "));
        WLAN_REPORT_HEX_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, (UINT8 *)pKey->micRxKey, MAX_KEY_LEN);
        WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, ("\nMic TX = "));
        WLAN_REPORT_HEX_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, (UINT8 *)pKey->micTxKey, MAX_KEY_LEN);
    }

    return status; 
}


TI_STATUS rsn_removeKey (rsn_t *pRsn, securityKeys_t *pKey)
{
    TI_STATUS           status = OK;
    whalParamInfo_t     whalParam;
    UINT8               keyIndex;

    keyIndex = (UINT8)pKey->keyIndex;
    if ((pRsn == NULL) || (pKey == NULL) || (keyIndex >= MAX_KEYS_NUM))
    {
        return NOK;
    }

    WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                            ("rsn_removeKey Entry, keyType=%d, keyIndex=0x%lx\n",pKey->keyType, keyIndex));

    /* Now set to the RSN structure. */
    if (pKey->keyType != NULL_KEY && pRsn->keys_en[keyIndex])
    {
        whalParam.paramType = HAL_CTRL_RSN_KEY_REMOVE_PARAM;
        /*os_memoryCopy(pRsn->hOs, &whalParam.content.rsnKey, pKey, sizeof(securityKeys_t));*/
        whalParam.content.configureCmdCBParams.CB_buf = (UINT8*) pKey;
        whalParam.content.configureCmdCBParams.CB_Func = NULL;
        whalParam.content.configureCmdCBParams.CB_handle = NULL;

        /* If keyType is TKIP or AES, set the encLen to the KEY enc len - 16 */
        if (pKey->keyType == TKIP_KEY || pKey->keyType == AES_KEY)
        {
            pKey->encLen = 16;
            if (keyIndex != 0)
            {   
                const UINT8 broadcast[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
                /* 
                 * if keyType is TKIP or AES, and the key index is broadcast, overwrite the MAC address as broadcast 
                 * for removing the Broadcast key from the FW 
                 */
                os_memoryCopy (pRsn->hOs, (void *)&pKey->macAddress.addr[0], (void*)broadcast, MAC_ADDR_LEN);
            }
        }
		else if (pKey->keyType == WEP_KEY)
		{
			/* In full driver we use only WEP default keys. To remove it we make sure that the MAC address is NULL */
			os_memoryZero(pRsn->hOs,(void*)pKey->macAddress.addr,sizeof(macAddress_t)); 
		}
       
        /* Mark key as deleted */
        pRsn->keys_en[keyIndex] = FALSE;

        status = whalCtrl_SetParam (pRsn->hWhalCtrl, &whalParam);
        
        WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                      ("rsn_removeKey in whal, status =%d\n", status));

        /* clean the key flags*/
        pRsn->keys[keyIndex].keyIndex &= 0x000000FF;
        pRsn->keys[keyIndex].keyType   = NULL_KEY;
        pRsn->keys[keyIndex].encLen    = 0;
        pRsn->wepDefaultKeys[keyIndex] = FALSE;        
    }

    return status; 
}


TI_STATUS rsn_setDefaultKeyId(rsn_t *pRsn, UINT8 keyId)
{
    TI_STATUS               status = OK;
    whalParamInfo_t         whalParam;

    if (pRsn == NULL)
    {
        return NOK;
    }
    pRsn->defaultKeyId = keyId;
    /* Now we configure default key ID to the HAL */
    whalParam.paramType = HAL_CTRL_RSN_DEFAULT_KEY_ID_PARAM;
    whalParam.content.configureCmdCBParams.CB_buf = &keyId;
    whalParam.content.configureCmdCBParams.CB_Func = NULL;
    whalParam.content.configureCmdCBParams.CB_handle = NULL;
    status = whalCtrl_SetParam(pRsn->hWhalCtrl, &whalParam); 
    
    WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                        ("RSN: rsn_setDefaultKeyId, KeyId = 0x%lx\n",
                         keyId));
    return status;
}


TI_STATUS rsn_reportAuthFailure(TI_HANDLE hRsn, authStatus_e authStatus) 
{
    TI_STATUS    status = OK;
    rsn_t       *pRsn;
    paramInfo_t param;

    if (hRsn==NULL)
    {
        return NOK;
    }

    pRsn = (rsn_t*)hRsn;

    /* Remove AP from candidate list for a specified amount of time */
	param.paramType = CTRL_DATA_CURRENT_BSSID_PARAM;
	status = ctrlData_getParam(pRsn->hCtrlData, &param);
	if (status != OK)
	{
		WLAN_REPORT_ERROR(pRsn->hReport, RSN_MODULE_LOG, 
          ("rsn_reportAuthFailure, unable to retrieve BSSID \n"));
	}
    else
    {
        WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG,
             ("current station is banned from the roaming candidates list for %d Ms\n",
              RSN_AUTH_FAILURE_TIMEOUT));

        rsn_banSite(hRsn, param.content.ctrlDataCurrentBSSID, RSN_SITE_BAN_LEVEL_FULL, RSN_AUTH_FAILURE_TIMEOUT);
    }

	
#ifdef EXC_MODULE_INCLUDED
	WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                        ("CALLING rougeAP, status= %d \n",authStatus));
    status = excMngr_rogueApDetected (pRsn->hExcMngr, authStatus);
#endif
    UNUSED(pRsn);
    return status;
}


/******
This is the CB function for mic failure event from the FW 
*******/
TI_STATUS rsn_reportMicFailure(TI_HANDLE hRsn, UINT8 *pType, UINT32 Length)
{
    rsn_t                               *pRsn = (rsn_t *) hRsn;
    rsn_siteBanLevel_e                  banLevel;
    OS_802_11_AUTHENTICATION_REQUEST    *request;
    UINT8 AuthBuf[sizeof(UINT32) + sizeof(OS_802_11_AUTHENTICATION_REQUEST)];
    paramInfo_t                         param;
    UINT8                               failureType;

    failureType = *pType;

    if (((pRsn->paeConfig.unicastSuite == RSN_CIPHER_TKIP) && (failureType == KEY_TKIP_MIC_PAIRWISE)) ||
        ((pRsn->paeConfig.broadcastSuite == RSN_CIPHER_TKIP) && (failureType == KEY_TKIP_MIC_GROUP)))
    {
        /* check if the MIC failure is group and group key update */
        /* was performed during the last 3 seconds */
        if ((failureType == KEY_TKIP_MIC_GROUP) &&
            (pRsn->groupKeyUpdate == GROUP_KEY_UPDATE_TRUE))
        {
            WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                    ("%s: Group MIC failure ignored, key update was performed within the last 3 seconds.\n", __FUNCTION__));
            return OK;
        }

        /* Prepare the Authentication Request */
        request = (OS_802_11_AUTHENTICATION_REQUEST *)(AuthBuf + sizeof(UINT32));
        request->Length = sizeof(OS_802_11_AUTHENTICATION_REQUEST);

        param.paramType = CTRL_DATA_CURRENT_BSSID_PARAM;
        if (ctrlData_getParam (pRsn->hCtrlData, &param) != OK)
        {
            return NOK;
        }
        
        /* Generate 802 Media specific indication event */
        *(UINT32*)AuthBuf = os802_11StatusType_Authentication;

        os_memoryCopy (pRsn->hOs, request->BSSID, (void *)param.content.ctrlDataCurrentBSSID.addr, MAC_ADDR_LEN);

        if (failureType == KEY_TKIP_MIC_PAIRWISE)
        {
            request->Flags = OS_802_11_REQUEST_PAIRWISE_ERROR;
        }
        else
        {
            request->Flags = OS_802_11_REQUEST_GROUP_ERROR;
        }

        EvHandlerSendEvent (pRsn->hEvHandler, 
                            IPC_EVENT_MEDIA_SPECIFIC, 
                            (UINT8*)AuthBuf,
                            sizeof(UINT32) + sizeof(OS_802_11_AUTHENTICATION_REQUEST));

        /* Update and check the ban level to decide what actions need to take place */
        banLevel = rsn_banSite (hRsn, param.content.ctrlDataCurrentBSSID, RSN_SITE_BAN_LEVEL_HALF, RSN_MIC_FAILURE_TIMEOUT);
        if (banLevel == RSN_SITE_BAN_LEVEL_FULL)
        {
            /* Site is banned so prepare to disconnect */
            WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                    ("%s: Second MIC failure, closing Rx port...\n", __FUNCTION__));

            param.paramType = RX_DATA_PORT_STATUS_PARAM;
            param.content.rxDataPortStatus = CLOSE;
            rxData_setParam(pRsn->hRx, &param);

            /* stop the mic failure Report timer and start a new one for 0.5 seconds */
            os_timerStop(pRsn->hOs, pRsn->micFailureReportWaitTimer);
            os_timerStart(pRsn->hOs, pRsn->micFailureReportWaitTimer, RSN_MIC_FAILURE_REPORT_WAIT, FALSE);
        }
        else
        {
            /* Site is only half banned so nothing needs to be done for now */
            WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
                    ("%s: First MIC failure, business as usual for now...\n", __FUNCTION__));
        }
    }

    return OK;
}


void rsn_groupReKeyTimeout(TI_HANDLE hRsn)
{
    rsn_t *pRsn;

    pRsn = (rsn_t*)hRsn;

    if (pRsn == NULL)
    {
        return;
    }

    pRsn->groupKeyUpdate = GROUP_KEY_UPDATE_FALSE;
}


void rsn_micFailureReportTimeout(TI_HANDLE hRsn)
{
    rsn_t *pRsn;

    pRsn = (rsn_t*)hRsn;

    if (pRsn == NULL)
    {
        return;
    }

    WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, 
            ("%s: MIC failure reported, disassociating...\n", __FUNCTION__));

    apConn_reportRoamingEvent (pRsn->hAPConn, ROAMING_TRIGGER_SECURITY_ATTACK, NULL);
}


/**
*
* rsn_resetPMKIDList - 
*
* \b Description: 
*   Cleans up the PMKID cache.
*   Called when SSID is being changed.
*
* \b ARGS:
*
*  I   - hRsn - Rsniation SM context  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*/

TI_STATUS rsn_resetPMKIDList(TI_HANDLE hRsn)
{
    rsn_t  *pRsn = (rsn_t*)hRsn;

    if (!pRsn)
        return NOK;

    return (pRsn->pAdmCtrl->resetPmkidList (pRsn->pAdmCtrl));
}


void rsn_debugFunc(TI_HANDLE hRsn)
{
    rsn_t *pRsn;

    if (hRsn == NULL)
    {
        return;
    }
    pRsn = (rsn_t*)hRsn;

    WLAN_OS_REPORT(("rsnStartedTs, ts = %d\n", pRsn->rsnStartedTs));
    WLAN_OS_REPORT(("rsnCompletedTs, ts = %d\n", pRsn->rsnCompletedTs));  
}


/**
*
* rsn_startPreAuth - 
*
* \b Description: 
*
* Start pre-authentication on a list of given BSSIDs.
*
* \b ARGS:
*
*  I   - hRsn - Rsniation SM context  \n
*  I/O - pBssidList - list of BSSIDs that require Pre-Auth \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa 
*/
TI_STATUS rsn_startPreAuth(TI_HANDLE hRsn, bssidList4PreAuth_t *pBssidList)
{
    rsn_t       *pRsn;
    TI_STATUS    status;

    if (hRsn == NULL || pBssidList == NULL)
    {
        return NOK;
    }

    pRsn = (rsn_t*)hRsn;

    status = pRsn->pAdmCtrl->startPreAuth (pRsn->pAdmCtrl, pBssidList);

    WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, ("rsn_startPreAuth \n"));

    return status;
}


/**
 *
 * isSiteBanned - 
 *
 * \b Description: 
 *
 * Returns whether or not the site with the specified Bssid is banned or not. 
 *
 * \b ARGS:
 *
 *  I   - hRsn - RSN module context \n
 *  I   - siteBssid - The desired site's bssid \n
 *
 * \b RETURNS:
 *
 *  NOK iff site is banned.
 *
 */
BOOL rsn_isSiteBanned(TI_HANDLE hRsn, macAddress_t siteBssid)
{
    rsn_t * pRsn = (rsn_t *) hRsn;
    rsn_siteBanEntry_t * entry;

    /* Check if site is in the list */
    if ((entry = findBannedSiteAndCleanup(hRsn, siteBssid)) == NULL)
    {
        return FALSE;
    }

    WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, ("%s: Site %02X-%02X-%02X-%02X-%02X-%02X found with ban level %d...\n", __FUNCTION__, siteBssid.addr[0], siteBssid.addr[1], siteBssid.addr[2], siteBssid.addr[3], siteBssid.addr[4], siteBssid.addr[5], entry->banLevel));

    return (entry->banLevel == RSN_SITE_BAN_LEVEL_FULL);
}


/**
 *
 * rsn_banSite - 
 *
 * \b Description: 
 *
 * Bans the specified site from being associated to for the specified duration.
 * If a ban level of WARNING is given and no previous ban was in effect the
 * warning is marked down but other than that nothing happens. In case a previous
 * warning (or ban of course) is still in effect
 *
 * \b ARGS:
 *
 *  I   - hRsn - RSN module context \n
 *  I   - siteBssid - The desired site's bssid \n
 *  I   - banLevel - The desired level of ban (Warning / Ban)
 *  I   - durationMs - The duration of ban in milliseconds
 *
 * \b RETURNS:
 *
 *  The level of ban (warning / banned).
 *
 */
rsn_siteBanLevel_e rsn_banSite(TI_HANDLE hRsn, macAddress_t siteBssid, rsn_siteBanLevel_e banLevel, UINT32 durationMs)
{
    rsn_t * pRsn = (rsn_t *) hRsn;
    rsn_siteBanEntry_t * entry;

    /* Try finding the site in the list */
    if ((entry = findBannedSiteAndCleanup(hRsn, siteBssid)) != NULL)
    {
        /* Site found so a previous ban is still in effect */ 
        WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, ("%s: Site %02X-%02X-%02X-%02X-%02X-%02X found and has been set to ban level full!\n", __FUNCTION__, siteBssid.addr[0], siteBssid.addr[1], siteBssid.addr[2], siteBssid.addr[3], siteBssid.addr[4], siteBssid.addr[5]));

        entry->banLevel = RSN_SITE_BAN_LEVEL_FULL;
    }
    else
    {
        /* Site doesn't appear in the list, so find a place to insert it */
        WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, ("%s: Site %02X-%02X-%02X-%02X-%02X-%02X added with ban level %d!\n", __FUNCTION__, siteBssid.addr[0], siteBssid.addr[1], siteBssid.addr[2], siteBssid.addr[3], siteBssid.addr[4], siteBssid.addr[5], banLevel));

        entry = findEntryForInsert (hRsn);

        entry->siteBssid = siteBssid;
        entry->banLevel = banLevel;

        pRsn->numOfBannedSites++;
    }

    entry->banStartedMs = os_timeStampMs (pRsn->hOs);
    entry->banDurationMs = durationMs;

    return entry->banLevel;
}


/**
 *
 * findEntryForInsert - 
 *
 * \b Description: 
 *
 * Returns a place to insert a new banned site. 
 *
 * \b ARGS:
 *
 *  I   - hRsn - RSN module context \n
 *
 * \b RETURNS:
 *
 *  A pointer to a suitable site entry.
 *
 */
static rsn_siteBanEntry_t * findEntryForInsert(TI_HANDLE hRsn)
{
    rsn_t * pRsn = (rsn_t *) hRsn;

    /* In the extreme case that the list is full we overwrite an old entry */
    if (pRsn->numOfBannedSites == RSN_MAX_NUMBER_OF_BANNED_SITES)
    {
        WLAN_REPORT_ERROR(pRsn->hReport, RSN_MODULE_LOG,
                ("%s: No room left to insert new banned site, overwriting old one!\n", __FUNCTION__));

        return &(pRsn->bannedSites[0]);
    }

    return &(pRsn->bannedSites[pRsn->numOfBannedSites]);
}


/**
 *
 * findBannedSiteAndCleanup - 
 *
 * \b Description: 
 *
 * Searches the banned sites list for the desired site while cleaning up
 * expired sites found along the way.
 * 
 * Note that this function might change the structure of the banned sites 
 * list so old iterators into the list might be invalidated.
 *
 * \b ARGS:
 *
 *  I   - hRsn - RSN module context \n
 *  I   - siteBssid - The desired site's bssid \n
 *
 * \b RETURNS:
 *
 *  A pointer to the desired site's entry if found,
 *  NULL otherwise.
 *
 */
static rsn_siteBanEntry_t * findBannedSiteAndCleanup(TI_HANDLE hRsn, macAddress_t siteBssid)
{
    rsn_t * pRsn = (rsn_t *) hRsn;
    int iter;

    for (iter = 0; iter < pRsn->numOfBannedSites; iter++)
    {
        /* If this entry has expired we'd like to clean it up */
        if (os_timeStampMs(pRsn->hOs) - pRsn->bannedSites[iter].banStartedMs >= pRsn->bannedSites[iter].banDurationMs)
        {
            WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, ("%s: Found expired entry at index %d, cleaning it up...\n", __FUNCTION__, iter));

            /* Replace this entry with the last one */
            pRsn->bannedSites[iter] = pRsn->bannedSites[pRsn->numOfBannedSites - 1];
            pRsn->numOfBannedSites--;

            /* we now repeat the iteration on this entry */
            iter--;

            continue;
        }

        /* Is this the entry for the site we're looking for? */
        if (MAC_EQUAL (&siteBssid, &pRsn->bannedSites[iter].siteBssid))
        {
            WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, ("%s: Site %02X-%02X-%02X-%02X-%02X-%02X found at index %d!\n", __FUNCTION__, siteBssid.addr[0], siteBssid.addr[1], siteBssid.addr[2], siteBssid.addr[3], siteBssid.addr[4], siteBssid.addr[5], iter));

            return &pRsn->bannedSites[iter];
        } 
    }

    /* Entry not found... */
    WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, ("%s: Site %02X-%02X-%02X-%02X-%02X-%02X not found...\n", __FUNCTION__, siteBssid.addr[0], siteBssid.addr[1], siteBssid.addr[2], siteBssid.addr[3], siteBssid.addr[4], siteBssid.addr[5], iter));

    return NULL;
}

/**
 *
 * clearBannedSiteList - 
 *
 * \b Description: 
 *
 * Clears the banned sites list.
 * 
 *
 * \b ARGS:
 *
 *  I   - hRsn - RSN module context \n
 *
 * \b RETURNS:
 *
 *
 */
/* Comment out the call to clearBannedSiteList due to fail in WiFi mic attack test */
/*
static void clearBannedSiteList(TI_HANDLE hRsn)
{
    rsn_t * pRsn = (rsn_t *) hRsn;

	pRsn->numOfBannedSites = 0;
    WLAN_REPORT_INFORMATION(pRsn->hReport, RSN_MODULE_LOG, ("%s\n", __FUNCTION__));

}
*/


#ifdef RSN_NOT_USED

static INT16 convertAscii2Unicode(INT8* userPwd, INT16 len)
{
    INT16 i;
    INT8 unsiiPwd[MAX_PASSWD_LEN];
    

    for (i=0; i<len; i++)
    {
        unsiiPwd[i] = userPwd[i];
    }
    for (i=0; i<len; i++)
    {
        userPwd[i*2] = unsiiPwd[i];
        userPwd[i*2+1] = 0;
    }
    return (INT16)(len*2);     
}

#endif
