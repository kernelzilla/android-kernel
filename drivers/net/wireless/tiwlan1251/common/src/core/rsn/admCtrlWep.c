/** \file admCtrlWep.c
 *  \brief Admission control API implimentation
 *
 *  \see admCtrl.h
 */
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
 *                                                                          *
 *   MODULE:  Admission Control	    		                                *
 *   PURPOSE: Admission Control Module API                              	*
 *                                                                          *
 ****************************************************************************/

#include "osApi.h"

#include "paramOut.h"
#include "paramIn.h"

#include "utils.h"
#include "fsm.h"
#include "report.h"
#include "mlmeApi.h"
#include "DataCtrl_Api.h"

#include "rsnApi.h"
#include "admCtrl.h"

#include "rsn.h"
#include "admCtrl.h"
#ifdef EXC_MODULE_INCLUDED
#include "admCtrlWpa.h"
#include "admCtrlExc.h"
#endif


/* Constants */

/* Enumerations */

/* Typedefs */

/* Structures */

/* External data definitions */

/* Local functions definitions */

/* Global variables */

/* Function prototypes */
TI_STATUS admCtrlWep_getInfoElement(admCtrl_t *pAdmCtrl, UINT8 *pIe, UINT8 *pLength);

TI_STATUS admCtrlWep_setSite(admCtrl_t *pAdmCtrl, rsnData_t *pRsnData, UINT8 *pAssocIe, UINT8 *pAssocIeLen);

TI_STATUS admCtrlWep_evalSite(admCtrl_t *pAdmCtrl, rsnData_t *pRsnData, bssType_e bssType, UINT32 *pEvaluation);


/**
*
* admCtrlWep_config  - Configure EXC admission control.
*
* \b Description: 
*
* Configure EXC admission control.
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS admCtrlWep_config(admCtrl_t *pAdmCtrl)
{
	TI_STATUS			status;
	rsn_paeConfig_t 	paeConfig;

	
	/* set admission control parameters */
	pAdmCtrl->externalAuthMode = (externalAuthMode_e)pAdmCtrl->authSuite;	   
	if ((pAdmCtrl->unicastSuite != RSN_CIPHER_WEP) && 
		(pAdmCtrl->unicastSuite != RSN_CIPHER_CKIP))
	{
		pAdmCtrl->unicastSuite = RSN_CIPHER_WEP;
	}
	
	if ((pAdmCtrl->broadcastSuite != RSN_CIPHER_WEP) && 
		(pAdmCtrl->broadcastSuite != RSN_CIPHER_CKIP))
	{
		pAdmCtrl->broadcastSuite = RSN_CIPHER_WEP;
	}

	/* set callback functions (API) */
	pAdmCtrl->getInfoElement = admCtrlWep_getInfoElement;
	pAdmCtrl->setSite = admCtrlWep_setSite;
	pAdmCtrl->evalSite = admCtrlWep_evalSite;
	pAdmCtrl->getPreAuthStatus = admCtrl_nullGetPreAuthStatus;
	pAdmCtrl->startPreAuth	= admCtrl_nullStartPreAuth;
    pAdmCtrl->get802_1x_AkmExists = admCtrl_nullGet802_1x_AkmExists;


		
	pAdmCtrl->keyMngSuite = RSN_KEY_MNG_802_1X;

	/* set PAE parametrs */
	paeConfig.authProtocol = pAdmCtrl->externalAuthMode;
	paeConfig.unicastSuite = pAdmCtrl->unicastSuite;
	paeConfig.broadcastSuite = pAdmCtrl->broadcastSuite;
	paeConfig.keyExchangeProtocol = pAdmCtrl->keyMngSuite;
	/* set default PAE configuration */
	status = pAdmCtrl->pRsn->setPaeConfig(pAdmCtrl->pRsn, &paeConfig);

	return status;
}

 
/**
*
* admCtrlWep_getInfoElement - Build the current information element.
*
* \b Description: 
*
* Build the current information element.
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  I   - pIe - IE buffer \n
*  I   - pLength - length of IE \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*							  
* \sa 
*/
TI_STATUS admCtrlWep_getInfoElement(admCtrl_t *pAdmCtrl, UINT8 *pIe, UINT8 *pLength)
{
	 
	if ((pAdmCtrl==NULL) || (pLength==NULL)) 
	{
		return NOK;
	}
	*pLength = 0;
	return OK;

}
/**
*
* admCtrlWep_setSite  - Set current primary site parameters for registration.
*
* \b Description: 
*
* Set current primary site parameters for registration.
*
* \b ARGS:
*
*  I   - pAdmCtrl - context \n
*  I   - pRsnData - site's RSN data \n
*  O   - pAssocIe - result IE of evaluation \n
*  O   - pAssocIeLen - length of result IE of evaluation \n
*  
* \b RETURNS:
*
*  OK on site is aproved, NOK on site is rejected.
*
* \sa 
*/
TI_STATUS admCtrlWep_setSite(admCtrl_t *pAdmCtrl, rsnData_t *pRsnData, UINT8 *pAssocIe, UINT8 *pAssocIeLen)
{
	TI_STATUS			status;
	paramInfo_t			param;
    whalParamInfo_t     whalParam;
	authSuite_e			authSuite;

#ifdef EXC_MODULE_INCLUDED
	if (parseIeBuffer(pAdmCtrl->hOs, pRsnData->pIe, pRsnData->ieLen, EXC_EXT_1_IE_ID, NULL, NULL, 0))
	{
		pAdmCtrl->excSupport = TRUE;
		return (admCtrlExc_setSite(pAdmCtrl, pRsnData, pAssocIe, pAssocIeLen));
	}
#endif


	pAdmCtrl->excSupport = FALSE;
	pAdmCtrl->unicastSuite = RSN_CIPHER_WEP;
	pAdmCtrl->broadcastSuite = RSN_CIPHER_WEP;

	admCtrlWep_config(pAdmCtrl);

	authSuite = pAdmCtrl->authSuite;

  /* Config the default keys */
	if ((authSuite == RSN_AUTH_SHARED_KEY) || (authSuite == RSN_AUTH_AUTO_SWITCH))
	{	/* Configure Security status in HAL */
		whalParam.paramType = HAL_CTRL_RSN_SECURITY_MODE_PARAM;
		whalParam.content.rsnEncryptionStatus = (halCtrl_CipherSuite_e)RSN_CIPHER_WEP;
		status = whalCtrl_SetParam(pAdmCtrl->pRsn->hWhalCtrl, &whalParam);
		/* Configure the keys in HAL */
		rsn_setDefaultKeys(pAdmCtrl->pRsn);
	}


	/* Now we configure the MLME module with the 802.11 legacy authentication suite, 
		THe MLME will configure later the authentication module */
	param.paramType = MLME_LEGACY_TYPE_PARAM;
#ifdef EXC_MODULE_INCLUDED	
	if (pAdmCtrl->networkEapMode!=OS_EXC_NETWORK_EAP_OFF)
    {
        param.content.mlmeLegacyAuthType = AUTH_LEGACY_RESERVED1;
    }
    else
#endif
    {
        switch (authSuite)
        {
        case RSN_AUTH_OPEN:
            param.content.mlmeLegacyAuthType = AUTH_LEGACY_OPEN_SYSTEM;
            break;

        case RSN_AUTH_SHARED_KEY: 
            param.content.mlmeLegacyAuthType = AUTH_LEGACY_SHARED_KEY;
            break;

        case RSN_AUTH_AUTO_SWITCH:
            param.content.mlmeLegacyAuthType = AUTH_LEGACY_AUTO_SWITCH;
            WLAN_REPORT_INFORMATION(pAdmCtrl->hReport , RSN_MODULE_LOG, 
                          ("WEP admCtrl mlme_setParam, RSN_AUTH_AUTO_SWITCH\n"));
            break;

        default:
            return NOK;
        }
    }
	
	status = mlme_setParam(pAdmCtrl->hMlme, &param);
	if (status != OK)
	{
		return status;
	}

	param.paramType = RX_DATA_EAPOL_DESTINATION_PARAM;
	param.content.rxDataEapolDestination = OS_ABS_LAYER;
	status = rxData_setParam(pAdmCtrl->hRx, &param);
	if (status != OK)
	{
		return status;
	}

	/* Configure Security status in HAL */
    whalParam.paramType = HAL_CTRL_RSN_SECURITY_MODE_PARAM;
    whalParam.content.rsnEncryptionStatus = (halCtrl_CipherSuite_e)RSN_CIPHER_WEP;
    status = whalCtrl_SetParam(pAdmCtrl->pRsn->hWhalCtrl, &whalParam);

	return status;

}

/**
*
* admCtrlWep_evalSite  - Evaluate site for registration.
*
* \b Description: 
*
* evaluate site RSN capabilities against the station's cap.
* If the BSS type is infrastructure, the station matches the site only if it's WEP status is same as the site
* In IBSS, it does not matter
*
* \b ARGS:
*
*  I   - pAdmCtrl - Context \n
*  I   - pRsnData - site's RSN data \n
*  O   - pEvaluation - Result of evaluation \n
*  
* \b RETURNS:
*
*  OK 
*
* \sa 
*/
TI_STATUS admCtrlWep_evalSite(admCtrl_t *pAdmCtrl, rsnData_t *pRsnData, bssType_e bssType, UINT32 *pEvaluation)
{
	
	*pEvaluation = 0;
	
	if (pRsnData==NULL)
	{
		return NOK;
	}
    pAdmCtrl->setSiteFirst = FALSE;

#ifdef EXC_MODULE_INCLUDED
	if (admCtrlExc_evalSite(pAdmCtrl, pRsnData, bssType, pEvaluation, &pAdmCtrl->excSupport) != OK)
    {
        return NOK;
    }
#else
	pAdmCtrl->excSupport = FALSE;
#endif /*EXC_MODULE_INCLUDED*/
	if (!pAdmCtrl->excSupport)
	{	/* WEP only */
		*pEvaluation = 1;
	}

	/* Check privacy bit if not in mixed mode */
	if (!pAdmCtrl->mixedMode)
	{	/* There's no mixed mode, so make sure that the privacy Bit matches the privacy mode*/
		if (!pRsnData->privacy)
			{
				*pEvaluation = 0;
                return NOK;
			}
	}

	return OK;
}




