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

/** \file admCtrl.c
 *  \brief Admission control API implimentation
 *
 *  \see admCtrl.h
 */

#include "osApi.h"

#include "paramOut.h"
#include "paramIn.h"

#include "utils.h"
#include "fsm.h"
#include "report.h"
#include "mlmeApi.h"
#include "DataCtrl_Api.h"

#include "TI_IPC_Api.h"

#include "rsn.h"
#include "admCtrl.h"

#include "admCtrlWpa.h"
#include "admCtrlWpa2.h"
#include "admCtrlNone.h"
#include "admCtrlWep.h"
#include "EvHandler.h"
 
/* Constants */

/* Enumerations */

/* Typedefs */

/* Structures */

/* External data definitions */

/* Local functions definitions */

/* Global variables */

/* Function prototypes */

TI_STATUS admCtrl_setAuthSuite(admCtrl_t *pAdmCtrl, authSuite_e authSuite);

TI_STATUS admCtrl_getAuthSuite(admCtrl_t *pAdmCtrl, authSuite_e *pSuite);

TI_STATUS admCtrl_setNetworkMode(admCtrl_t *pAdmCtrl, rsn_networkMode_t mode);

TI_STATUS admCtrl_setExtAuthMode(admCtrl_t *pAdmCtrl, externalAuthMode_e extAuthMode);

TI_STATUS admCtrl_getExtAuthMode(admCtrl_t *pAdmCtrl, externalAuthMode_e *pExtAuthMode);

TI_STATUS admCtrl_setUcastSuite(admCtrl_t *pAdmCtrl, cipherSuite_e suite);

TI_STATUS admCtrl_setBcastSuite(admCtrl_t *pAdmCtrl, cipherSuite_e suite);

TI_STATUS admCtrl_getCipherSuite(admCtrl_t *pAdmCtrl, cipherSuite_e *pSuite);

TI_STATUS admCtrl_setKeyMngSuite(admCtrl_t *pAdmCtrl, rsn_keyMngSuite_e suite);

TI_STATUS admCtrl_getMixedMode(admCtrl_t *pAdmCtrl, BOOL *pMixedMode);

TI_STATUS admCtrl_setMixedMode(admCtrl_t *pAdmCtrl, BOOL mixedMode);

TI_STATUS admCtrl_getAuthEncrCapability(admCtrl_t *pAdmCtrl, 
                   rsnAuthEncrCapability_t   *authEncrCapability);

TI_STATUS admCtrl_getPromoteFlags(admCtrl_t *pAdmCtrl, UINT32 *WPAPromoteFlags);

TI_STATUS admCtrl_setPromoteFlags(admCtrl_t *pAdmCtrl, UINT32 WPAPromoteFlags);

TI_STATUS admCtrl_getWPAMixedModeSupport(admCtrl_t *pAdmCtrl, UINT32 *support);

TI_STATUS admCtrl_checkSetSuite(admCtrl_t *pAdmCtrl, cipherSuite_e suite, BOOL Broadcast);

#ifdef EXC_MODULE_INCLUDED
TI_STATUS admCtrl_setNetworkEap(admCtrl_t *pAdmCtrl, OS_EXC_NETWORK_EAP networkEap);

TI_STATUS admCtrl_getNetworkEap(admCtrl_t *pAdmCtrl, OS_EXC_NETWORK_EAP *networkEap);
#endif

/**
*
* admCtrl_create
*
* \b Description: 
*
* Create the admission control context.
*
* \b ARGS:
*
*  I   - role - admission cotrol role (AP or Station)  \n
*  I   - authSuite - authentication suite to work with \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
admCtrl_t* admCtrl_create(TI_HANDLE hOs)
{
	admCtrl_t 		*pHandle;

	/* allocate rsniation context memory */
	pHandle = (admCtrl_t*)os_memoryAlloc(hOs, sizeof(admCtrl_t));
	if (pHandle == NULL)
	{
		return NULL;
	}

	os_memoryZero(hOs, pHandle, sizeof(admCtrl_t));

    pHandle->preAuthTimerWpa2 = os_timerCreate(hOs, admCtrlWpa2_preAuthTimerExpire, pHandle);

    /* If we failed to create the timer - exit with error */
    if (pHandle->preAuthTimerWpa2 == NULL)
    {
       os_memoryFree(pHandle->hOs, pHandle, sizeof(admCtrl_t));
       return NULL;
    }
	pHandle->hOs = hOs;

	return pHandle;
}

/**
*
* admCtrl_unload
*
* \b Description: 
*
* Unload admission control module from memory
*
* \b ARGS:
*
*  I   - hAdmCtrl - Admossion control context  \n
*
* \b RETURNS:
*
*  OK if successful, NOK otherwise.
*
* \sa admCtrl_create
*/
TI_STATUS admCtrl_unload(admCtrl_t *pAdmCtrl)
{
	if (pAdmCtrl == NULL)
	{
		return NOK;
	}
	
    /* Stop and destroy the wpa2 pre-authentication timer */
   os_timerStop(pAdmCtrl->hOs, pAdmCtrl->preAuthTimerWpa2);
   utils_nullTimerDestroy(pAdmCtrl->hOs, pAdmCtrl->preAuthTimerWpa2);
	os_memoryFree(pAdmCtrl->hOs, pAdmCtrl, sizeof(admCtrl_t));

	return OK;
}

/**
*
* admCtrl_config
*
* \b Description: 
*
* Configure the admission control module.
*
* \b ARGS:
*
*  I   - role - admission cotrol role (AP or Station)  \n
*  I   - authSuite - authentication suite to work with \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS admCtrl_config(TI_HANDLE hAdmCtrl,
					  TI_HANDLE hMlme,
					  TI_HANDLE hRx,
					  TI_HANDLE hReport,
					  TI_HANDLE hOs,
					  struct _rsn_t *pRsn,
					  TI_HANDLE hExcMngr,
					  TI_HANDLE hPowerMgr,
	                  TI_HANDLE	hEvHandler,
					  rsnInitParams_t *pInitParam)
{
	admCtrl_t		*pAdmCtrl;
	TI_STATUS			status;

	if (hAdmCtrl == NULL)
	{
		return NOK;
	}
	
	pAdmCtrl = (admCtrl_t*)hAdmCtrl;

	pAdmCtrl->pRsn = pRsn;
	pAdmCtrl->hMlme = hMlme;
	pAdmCtrl->hRx = hRx;
	pAdmCtrl->hReport = hReport;
	pAdmCtrl->hOs = hOs;
	pAdmCtrl->hExcMngr = hExcMngr;
	pAdmCtrl->hPowerMgr = hPowerMgr;
    pAdmCtrl->hEvHandler = hEvHandler;

	/* Initialize admission control parameters */
	pAdmCtrl->role = RSN_PAE_SUPP;
	pAdmCtrl->networkMode = RSN_INFRASTRUCTURE;
	pAdmCtrl->authSuite	= pInitParam->authSuite;
	pAdmCtrl->externalAuthMode = pInitParam->externalAuthMode;
    pAdmCtrl->mixedMode = pInitParam->mixedMode;
	
    if (pInitParam->privacyOn)
    {
        pAdmCtrl->broadcastSuite = RSN_CIPHER_WEP;
        pAdmCtrl->unicastSuite = RSN_CIPHER_WEP;
    } else {
        pAdmCtrl->broadcastSuite = RSN_CIPHER_NONE;
        pAdmCtrl->unicastSuite = RSN_CIPHER_NONE;
    }

	pAdmCtrl->preAuthSupport     = pInitParam->preAuthSupport;
    pAdmCtrl->preAuthTimeout     = pInitParam->preAuthTimeout;
	pAdmCtrl->WPAMixedModeEnable = pInitParam->WPAMixedModeEnable;
	/*pAdmCtrl->PMKIDCandListDelay = pInitParam->PMKIDCandListDelay;*/
	pAdmCtrl->MaxNumOfPMKIDs     = PMKID_MAX_NUMBER;

	/* Initialize admission control member functions */
	pAdmCtrl->setAuthSuite = admCtrl_setAuthSuite;
	pAdmCtrl->setNetworkMode = admCtrl_setNetworkMode;
	pAdmCtrl->getAuthSuite = admCtrl_getAuthSuite;
	pAdmCtrl->setExtAuthMode = admCtrl_setExtAuthMode;
	pAdmCtrl->getExtAuthMode = admCtrl_getExtAuthMode;
	pAdmCtrl->setUcastSuite	= admCtrl_setUcastSuite;
	pAdmCtrl->setBcastSuite = admCtrl_setBcastSuite;
	pAdmCtrl->getCipherSuite = admCtrl_getCipherSuite;
	pAdmCtrl->setKeyMngSuite = admCtrl_setKeyMngSuite;
    pAdmCtrl->getMixedMode = admCtrl_getMixedMode;
    pAdmCtrl->setMixedMode = admCtrl_setMixedMode;
    pAdmCtrl->getAuthEncrCap = admCtrl_getAuthEncrCapability;
    pAdmCtrl->getPmkidList   = admCtrl_nullGetPMKIDlist;
    pAdmCtrl->setPmkidList   = admCtrl_nullSetPMKIDlist;
    pAdmCtrl->resetPmkidList = admCtrl_resetPMKIDlist;
    pAdmCtrl->getPromoteFlags = admCtrl_getPromoteFlags;
    pAdmCtrl->setPromoteFlags = admCtrl_setPromoteFlags;
    pAdmCtrl->getWPAMixedModeSupport = admCtrl_getWPAMixedModeSupport;
#ifdef EXC_MODULE_INCLUDED
	pAdmCtrl->setNetworkEap = admCtrl_setNetworkEap;
	pAdmCtrl->getNetworkEap = admCtrl_getNetworkEap;
    pAdmCtrl->networkEapMode = OS_EXC_NETWORK_EAP_OFF;
#endif

	pAdmCtrl->getPreAuthStatus = admCtrl_nullGetPreAuthStatus;
	pAdmCtrl->startPreAuth	= admCtrl_nullStartPreAuth;
    pAdmCtrl->get802_1x_AkmExists = admCtrl_nullGet802_1x_AkmExists;
    /* Zero number of sent wpa2 preauthentication candidates */
    pAdmCtrl->numberOfPreAuthCandidates = 0;

	status = admCtrl_subConfig(pAdmCtrl);

	return status;
}

/**
*
* admCtrl_subConfig
*
* \b Description: 
*
* Configure the admission control module according to the Privacy Mode.
*
* \b ARGS:
*
*  I   - pAdmCtrl - pointer to admission cotrol context  \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS admCtrl_subConfig(TI_HANDLE hAdmCtrl)

{
    admCtrl_t* pAdmCtrl = (admCtrl_t*)hAdmCtrl;
	TI_STATUS status;

	switch(pAdmCtrl->externalAuthMode)
	{
	case RSN_EXT_AUTH_MODE_WPA:
	case RSN_EXT_AUTH_MODE_WPAPSK:
	case RSN_EXT_AUTH_MODE_WPANONE:
		status = admCtrlWpa_config(pAdmCtrl);
		break;

	case RSN_EXT_AUTH_MODE_WPA2:
	case RSN_EXT_AUTH_MODE_WPA2PSK:
		status = admCtrlWpa2_config(pAdmCtrl);
		break;

	default:
		if(pAdmCtrl->unicastSuite==RSN_CIPHER_NONE)
		{
			status = admCtrlNone_config(pAdmCtrl);
		}
		else
		{
			status = admCtrlWep_config(pAdmCtrl);
		}
		break;

	}
	
	return status;

}

/**
*
* admCtrl_setNetworkMode - Change current network mode.
*
* \b Description: 
*
* Change current network mode.
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  I   - mode - network association mode (Infustrucure/IBSS) \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS admCtrl_setNetworkMode(admCtrl_t *pAdmCtrl, rsn_networkMode_t mode)
{
    pAdmCtrl->networkMode = mode;

    return OK;
}

/**
*
* admCtrl_setAuthSuite - Change current authentication suite.
*
* \b Description: 
*
* Change current authentication suite.
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  I   - authSuite - authentication suite to work with \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS admCtrl_setAuthSuite(admCtrl_t *pAdmCtrl, authSuite_e authSuite)
{
	TI_STATUS 		status = NOK;

	if (pAdmCtrl == NULL)
	{
		return NOK;
	}

	if (pAdmCtrl->authSuite == authSuite)
	{
		return OK;
	}

	if (pAdmCtrl->authSuite > RSN_AUTH_AUTO_SWITCH)
	{
		return NOK;
	}
	pAdmCtrl->externalAuthMode = (externalAuthMode_e)authSuite;
	pAdmCtrl->authSuite = authSuite;
	status = admCtrl_subConfig(pAdmCtrl);
	return status;
}


/**
*
* admCtrl_getAuthSuite  - Get current authentication suite.
*
* \b Description: 
*
* Get current authentication suite.
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  O   - suite - key management suite to work with \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS admCtrl_getAuthSuite(admCtrl_t *pAdmCtrl, authSuite_e *pSuite)
{
	if (pAdmCtrl == NULL)
	{
		return NOK;
	}

	*pSuite = pAdmCtrl->authSuite;

	return OK;
}

/**
*
* admCtrl_setExtAuthMode  - Set current External authentication Mode Status.
*
* \b Description: 
*
* Set current External authentication Mode Status.
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  I   - extAuthMode - External authentication Mode \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS admCtrl_setExtAuthMode(admCtrl_t *pAdmCtrl, externalAuthMode_e extAuthMode)
{

	if (extAuthMode >= RSN_EXT_AUTH_MODEMAX)
	{
		return NOK;
	}


	if (pAdmCtrl->externalAuthMode == extAuthMode)
	{
		return OK;
	}
	pAdmCtrl->externalAuthMode = extAuthMode;
	if (extAuthMode <= RSN_EXT_AUTH_MODE_AUTO_SWITCH)
	{
		pAdmCtrl->authSuite = (authSuite_e)extAuthMode;
	}
	else
	{
		pAdmCtrl->authSuite = RSN_AUTH_OPEN;
	}
	
	return (admCtrl_subConfig(pAdmCtrl));
}

/**
*
* admCtrl_getExtAuthMode  - Get current External authentication Mode Status.
*
* \b Description: 
*
* Get current External Mode Status.
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  I   - pExtAuthMode - EXC External Mode Status \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS admCtrl_getExtAuthMode(admCtrl_t *pAdmCtrl, externalAuthMode_e *pExtAuthMode)
{
	*pExtAuthMode = pAdmCtrl->externalAuthMode;

	return OK;
}


/**
*
* admCtrl_checkSetSuite -
*
* \b Description: 
*
* Check the validity/support of the cipher suite according to
* the admission control parameters
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  I   - suite - cipher suite to check \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS admCtrl_checkSetSuite(admCtrl_t *pAdmCtrl, cipherSuite_e suite, BOOL Broadcast)
{
	if (pAdmCtrl->externalAuthMode<=RSN_EXT_AUTH_MODE_AUTO_SWITCH)
	{   
		if ((suite==RSN_CIPHER_NONE) || (suite==RSN_CIPHER_WEP) || (suite==RSN_CIPHER_WEP104))
		{
			return OK;
		}
	}
	else
	{
		if ((suite==RSN_CIPHER_TKIP) || (suite==RSN_CIPHER_WEP) || 
			(suite==RSN_CIPHER_WEP104) || (suite==RSN_CIPHER_AES_CCMP))
		{
			return OK;
		}
		else if (!Broadcast && (suite==RSN_CIPHER_NONE))
		{
			return OK;
		}
	}
	return NOK;
}

/**
*
* admCtrl_setUcastSuite  - Set current unicast cipher suite support.
*
* \b Description: 
*
* Set current unicast cipher suite support.
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  I   - suite - cipher suite to work with \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS admCtrl_setUcastSuite(admCtrl_t *pAdmCtrl, cipherSuite_e suite)
{
	TI_STATUS status;

	if (suite == pAdmCtrl->unicastSuite)
	{
		return OK;
	}
	status = admCtrl_checkSetSuite(pAdmCtrl, suite, FALSE);
	if (status == OK)
	{
		pAdmCtrl->unicastSuite = suite;
		status = admCtrl_subConfig(pAdmCtrl);
	}

	return status;
}

/**
*
* admCtrl_setBcastSuite  - Set current broadcast cipher suite support.
*
* \b Description: 
*
* Set current broadcast cipher suite support.
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  I   - suite - cipher suite to work with \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS admCtrl_setBcastSuite(admCtrl_t *pAdmCtrl, cipherSuite_e suite)
{
	TI_STATUS status;

	if (suite == pAdmCtrl->broadcastSuite)
	{
		return OK;
	}

	status = admCtrl_checkSetSuite(pAdmCtrl, suite, TRUE);
	if (status == OK)
	{
		pAdmCtrl->broadcastSuite = suite;
		status = admCtrl_subConfig(pAdmCtrl);
	}
	return status;

}

/**
*
* admCtrl_getCipherSuite  - Set current broadcast cipher suite support.
*
* \b Description: 
*
* Set current broadcast cipher suite support.
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  O   - suite - cipher suite to work with \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS admCtrl_getCipherSuite(admCtrl_t *pAdmCtrl, cipherSuite_e *pSuite)
{
	if (pAdmCtrl == NULL)
	{
		return NOK;
	}

	*pSuite = (pAdmCtrl->broadcastSuite > pAdmCtrl->unicastSuite) ? pAdmCtrl->broadcastSuite :pAdmCtrl->unicastSuite;
	 
	return OK;
}
	
/**
*
* admCtrl_setKeyMngSuite  - Set current key management suite support.
*
* \b Description: 
*
* Set current key management suite support.
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  I   - suite - key management suite to work with \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS admCtrl_setKeyMngSuite(admCtrl_t *pAdmCtrl, rsn_keyMngSuite_e suite)
{
    pAdmCtrl->keyMngSuite = suite;

	return OK;
}


/**
*
* admCtrl_parseIe  - Parse a required information element.
*
* \b Description: 
*
* Parse an Aironet information element. 
* Builds a structure of all the capabilities described in the Aironet IE.
* We look at Flags field only to determine KP and MIC bits value
*
* \b ARGS:
*
*  I   - pAdmCtrl - pointer to admCtrl context
*  I   - pAironetIe - pointer to Aironet IE buffer  \n
*  O   - pAironetData - capabilities structure
*  
*  
* \b RETURNS:
*
* OK on success, NOK on failure. 
*
* \sa 
*/
TI_STATUS admCtrl_parseIe(admCtrl_t *pAdmCtrl, rsnData_t *pRsnData, UINT8 **pIe, UINT8 IeId)
{

	dot11_eleHdr_t   *eleHdr;
	INT16			 length;
	UINT8            *pCurIe;


	*pIe = NULL;

	if ((pRsnData == NULL) || (pRsnData->ieLen==0))
	{
	   return OK;	
	}

	pCurIe = pRsnData->pIe;
	
	length = pRsnData->ieLen;
	while (length>0)
	{
		eleHdr = (dot11_eleHdr_t*)pCurIe;
		
		if (length<(eleHdr->eleLen+2))
		{
			WLAN_REPORT_INFORMATION(pAdmCtrl->hReport, RSN_MODULE_LOG, 
								("admCtrl_parseIe ERROR: pRsnData->ieLen=%d, length=%d\n\n", pRsnData->ieLen,length));
			return OK;
		}
		
		if (eleHdr->eleId == IeId)
		{
            *pIe = (UINT8*)eleHdr;
            break;
		}
		length -= eleHdr->eleLen+2;
		pCurIe += eleHdr->eleLen+2;
	}
	return OK;
}

/**
*
* admCtrl_setMixedMode  - Set current mixed Mode Status.
*
* \b Description: 
*
* Set current mixed Mode Status.
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  I   - authMode - mixed Mode \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS admCtrl_setMixedMode(admCtrl_t *pAdmCtrl, BOOL mixedMode)
{

	if (pAdmCtrl->mixedMode == mixedMode)
	{
		return OK;
	}
	pAdmCtrl->mixedMode = mixedMode;

	return OK;
}

/**
*
* admCtrl_getMixedMode  - Get current mixed Mode Status.
*
* \b Description: 
*
* Get current mixed Mode Status.
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  I   - pAuthMode - mixed Mode Status \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS admCtrl_getMixedMode(admCtrl_t *pAdmCtrl, BOOL *pMixedMode)
{
	*pMixedMode = pAdmCtrl->mixedMode;

	return OK;
}



/* This table presents supported pairs of auth.mode/cipher type */
static 	authEncrPairList_t  supportedAuthEncrPairs[MAX_AUTH_ENCR_PAIR] = 
{
	{RSN_EXT_AUTH_MODE_OPEN,       RSN_CIPHER_NONE},
	{RSN_EXT_AUTH_MODE_OPEN,       RSN_CIPHER_WEP},
	{RSN_EXT_AUTH_MODE_SHARED_KEY, RSN_CIPHER_NONE},
	{RSN_EXT_AUTH_MODE_SHARED_KEY, RSN_CIPHER_WEP},
	{RSN_EXT_AUTH_MODE_WPA,        RSN_CIPHER_TKIP},
	{RSN_EXT_AUTH_MODE_WPA,        RSN_CIPHER_AES_CCMP},
	{RSN_EXT_AUTH_MODE_WPAPSK,     RSN_CIPHER_TKIP},
	{RSN_EXT_AUTH_MODE_WPAPSK,     RSN_CIPHER_AES_CCMP},
	{RSN_EXT_AUTH_MODE_WPANONE,    RSN_CIPHER_NONE},    /* No encryption in IBSS mode */
	{RSN_EXT_AUTH_MODE_WPA2,       RSN_CIPHER_TKIP},
	{RSN_EXT_AUTH_MODE_WPA2,       RSN_CIPHER_AES_CCMP},
	{RSN_EXT_AUTH_MODE_WPA2PSK,    RSN_CIPHER_TKIP},
	{RSN_EXT_AUTH_MODE_WPA2PSK,    RSN_CIPHER_AES_CCMP}
};

/**
*
* admCtrl_getAuthEncrCapability  - Get all supported pais of
*                                  authenticationmode/cipher suite
*
* \b Description: 
*
*    Returns all supported pais of authenticationmode/cipher suite
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  I   - authEncrCapability - ptr to list of auth.mode/cipher pairs \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/

TI_STATUS admCtrl_getAuthEncrCapability(admCtrl_t *pAdmCtrl, 
                                        rsnAuthEncrCapability_t   *authEncrCapability)
{
    int i = 0;

    if(!authEncrCapability)
        return NOK;

    /* The current driver code version  uses the above hardcoded list */
    /* of auth/encr pairs */

    authEncrCapability->NoOfAuthEncrPairSupported = MAX_AUTH_ENCR_PAIR;
    authEncrCapability->NoOfPMKIDs                = PMKID_MAX_NUMBER;

    WLAN_REPORT_INFORMATION(pAdmCtrl->hReport, RSN_MODULE_LOG, 
    ("admCtrl get AuthEncr capability:  No. of auth/encr pairs = %d, No of PMKIDs = %d \n", 
        authEncrCapability->NoOfAuthEncrPairSupported,  authEncrCapability->NoOfPMKIDs));

    /* Copy the hardcoded table of the auth.mode/cipher type */
    for (i = 0; i < MAX_AUTH_ENCR_PAIR; i++)
    {
        authEncrCapability->authEncrPairs[i].authenticationMode = 
            supportedAuthEncrPairs[i].authenticationMode;
        authEncrCapability->authEncrPairs[i].cipherSuite        = 
            supportedAuthEncrPairs[i].cipherSuite;

        WLAN_REPORT_INFORMATION(pAdmCtrl->hReport, RSN_MODULE_LOG, 
            ("admCtrl get AuthEncr pair list: i = %d, auth mode = %d , cipher suite = %d \n", i,
                       authEncrCapability->authEncrPairs[i].authenticationMode,
                       authEncrCapability->authEncrPairs[i].cipherSuite));
    }
    
    return OK;
}


TI_STATUS admCtrl_nullSetPMKIDlist(admCtrl_t *pAdmCtrl, OS_802_11_PMKID  *pmkIdList)
{

    return CONFIGURATION_NOT_VALID;
}

TI_STATUS admCtrl_nullGetPMKIDlist(admCtrl_t *pAdmCtrl, OS_802_11_PMKID  *pmkIdList)
{

    return CONFIGURATION_NOT_VALID;
}


TI_STATUS admCtrl_resetPMKIDlist(admCtrl_t *pAdmCtrl)
{

    os_memoryZero(pAdmCtrl->hOs, (PVOID)&pAdmCtrl->pmkid_cache, sizeof(pmkid_cache_t));
    return OK;
}

TI_STATUS admCtrl_getWPAMixedModeSupport(admCtrl_t *pAdmCtrl, UINT32 *support)
{
    
    if(pAdmCtrl->WPAMixedModeEnable)
       *support = ADMCTRL_WPA_OPTION_MAXVALUE;
    else
       *support = 0;

    return OK;
}

TI_STATUS admCtrl_getPromoteFlags(admCtrl_t *pAdmCtrl, UINT32 *WPAPromoteFlags)
{
    *WPAPromoteFlags = pAdmCtrl->WPAPromoteFlags;
    return OK;
}

TI_STATUS admCtrl_setPromoteFlags(admCtrl_t *pAdmCtrl, UINT32 WPAPromoteFlags)
{
    if(WPAPromoteFlags > ADMCTRL_WPA_OPTION_MAXVALUE)
        return NOK;

    if(!pAdmCtrl->WPAMixedModeEnable)
       return NOK;

    pAdmCtrl->WPAPromoteFlags = WPAPromoteFlags;
    return OK;
}

BOOL admCtrl_nullGetPreAuthStatus(admCtrl_t *pAdmCtrl, macAddress_t *givenAP, UINT8 *cacheIndex)
{
    return FALSE;
}


TI_STATUS admCtrl_nullStartPreAuth(admCtrl_t *pAdmCtrl, bssidList4PreAuth_t *pBssidList)
{
	return OK;
}

TI_STATUS admCtrl_nullGet802_1x_AkmExists (admCtrl_t *pAdmCtrl, BOOL *wpa_802_1x_AkmExists)
{
    *wpa_802_1x_AkmExists = FALSE;
    return OK;
}

/*-----------------------------------------------------------------------------
Routine Name: admCtrl_notifyPreAuthStatus
Routine Description: This routine is used to notify higher level application of the pre-authentication status
Arguments: newStatus - pre authentication status
Return Value:
-----------------------------------------------------------------------------*/
void admCtrl_notifyPreAuthStatus (admCtrl_t *pAdmCtrl, preAuthStatusEvent_e newStatus)
{
    UINT32 memBuff;

    memBuff = (UINT32) newStatus;

    EvHandlerSendEvent(pAdmCtrl->hEvHandler, IPC_EVENT_WPA2_PREAUTHENTICATION,
                            (UINT8*)&memBuff, sizeof(UINT32));

}

#ifdef EXC_MODULE_INCLUDED

/**
*
* admCtrl_setNetworkEap  - Set current Network EAP Mode Status.
*
* \b Description: 
*
* Set current Network EAP Mode Status..
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  I   - networkEap - Network EAP Mode \n
*
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS admCtrl_setNetworkEap(admCtrl_t *pAdmCtrl, OS_EXC_NETWORK_EAP networkEap)
{
	if (pAdmCtrl==NULL)
		return NOK;

	if (pAdmCtrl->networkEapMode == networkEap)
	{
		return OK;
	}
	pAdmCtrl->networkEapMode = networkEap;
	
	return OK;
}

/**
*
* admCtrl_getNetworkEap  - Get current Network EAP Mode Status.
*
* \b Description: 
*
* Get current Network EAP Mode Status.
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  I   - networkEap - Network EAP Mode \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS admCtrl_getNetworkEap(admCtrl_t *pAdmCtrl, OS_EXC_NETWORK_EAP *networkEap)
{
	
	if (pAdmCtrl==NULL)
	{
		return NOK;
	}

	switch (pAdmCtrl->networkEapMode)
	{
	case OS_EXC_NETWORK_EAP_OFF: 
		*networkEap = OS_EXC_NETWORK_EAP_OFF;
		break; 
	case OS_EXC_NETWORK_EAP_ON:      
	case OS_EXC_NETWORK_EAP_ALLOWED:
	case OS_EXC_NETWORK_EAP_PREFERRED:
		*networkEap = OS_EXC_NETWORK_EAP_ON;
		break;
	default:
		return NOK;
/*		break; - unreachable */
	}

	return OK;
}
#endif /* EXC_MODULE_INCLUDED*/

