/** \file reportReplvl.c
 *  \brief Report level implementation
 *
 *  \see reportReplvl.h
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

/***************************************************************************/
/*																			*/
/*		MODULE:	reportReplvl.c																*/
/*    PURPOSE:	Report level implementation	 										*/
/*																			*/
/***************************************************************************/
#include "report.h"
#include "osTIType.h"
#include "osApi.h"
#include "siteHash.h"
#include "utils.h"
#include "rsnApi.h"
#include "regulatoryDomainApi.h"
#include "siteMgrApi.h"

/********************************************/
/*		Functions Implementations			*/
/********************************************/

/************************************************************************
 *                        buildNullTemplate								*
 ************************************************************************
DESCRIPTION: This function build a NULL data template to set to the HAL 
				when joining an infrastructure network
				performs the following:
				-	Build a template & set the template len, the template type is set in the site mgr
                                                                                                   
INPUT:      pSiteMgr	-	Handle to site manager	
			pTemplate	-	Pointer to the template structure		


OUTPUT:		


RETURN:     OK

************************************************************************/
TI_STATUS buildNullTemplate(siteMgr_t *pSiteMgr, whalCtrl_setTemplate_t *pTemplate)
{
	paramInfo_t			param;
	UINT32				size;
	nullDataTemplate_t	*pBuffer = (nullDataTemplate_t	*)pTemplate->pTemplate;
	siteEntry_t			*pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;
	UINT16				fc;

	os_memoryZero(pSiteMgr->hOs, pBuffer, sizeof(nullDataTemplate_t));

	/*
	 * Header First
	 */
	/* Set destination address */
	os_memoryCopy(pSiteMgr->hOs, &(pBuffer->hdr.DA.addr), &(pPrimarySite->bssid), MAC_ADDR_LEN);  

	/* Set BSSID address */
	os_memoryCopy(pSiteMgr->hOs, &(pBuffer->hdr.BSSID.addr), &(pPrimarySite->bssid), MAC_ADDR_LEN);  

	/* Build Source address */
	param.paramType = CTRL_DATA_MAC_ADDRESS;
	ctrlData_getParam(pSiteMgr->hCtrlData, &param);
	os_memoryCopy(pSiteMgr->hOs, &(pBuffer->hdr.SA.addr), &(param.content.ctrlDataDeviceMacAddress), MAC_ADDR_LEN);  
	
	fc = DOT11_FC_DATA_NULL_FUNCTION;
	fc |= (TRUE << DOT11_FC_TO_DS_SHIFT);

	pBuffer->hdr.fc = ENDIAN_HANDLE_WORD(fc);

	size = sizeof(dot11_mgmtHeader_t);

	pTemplate->templateLen = size;
	
	return OK;
}

/************************************************************************
 *                        buildProbeReqTemplate							*
 ************************************************************************
DESCRIPTION: This function build a probe request template to set to the HAL in the scan process.
				performs the following:
				-	Build a template & set the template len, the template type is set in the site mgr
                                                                                                   
INPUT:      pSiteMgr	-	Handle to site manager	
			pTemplate	-	Pointer to the template structure		
			pSsid		-	Desired SSID


OUTPUT:		


RETURN:     OK

************************************************************************/
TI_STATUS buildProbeReqTemplate(siteMgr_t *pSiteMgr, whalCtrl_setTemplate_t *pTemplate, ssid_t *pSsid, radioBand_e radioBand)
{
	paramInfo_t			param;
	char				*pBuf;
	int i;
	probeReqTemplate_t	*pBuffer = (probeReqTemplate_t	*)pTemplate->pTemplate;
	UINT32				size;
	dot11_RATES_t		*pDot11Rates;	
	UINT32				len = 0, ofdmIndex = 0;
	UINT32				suppRatesLen, extSuppRatesLen;
	UINT8				ratesBuf[MAX_SUPPORTED_RATES];	
	UINT32				supportedRateMask,basicRateMask;	

	os_memoryZero(pSiteMgr->hOs, pBuffer, sizeof(probeReqTemplate_t));

	/*
	 * Header First
	 */
	/* Set destination address */
	for (i = 0; i < MAC_ADDR_LEN; i++)
		pBuffer->hdr.DA.addr[i] = 0xFF;

	/* Set BSSID address */

	for (i = 0; i < MAC_ADDR_LEN; i++)
		pBuffer->hdr.BSSID.addr[i] = 0xFF;
 

	/* Build Source address */
	param.paramType = CTRL_DATA_MAC_ADDRESS;
	ctrlData_getParam(pSiteMgr->hCtrlData, &param);
	os_memoryCopy(pSiteMgr->hOs, &(pBuffer->hdr.SA.addr), &(param.content.ctrlDataDeviceMacAddress), MAC_ADDR_LEN);  
	
	pBuffer->hdr.fc = ENDIAN_HANDLE_WORD(DOT11_FC_PROBE_REQ);

	size = sizeof(dot11_mgmtHeader_t);
	pBuf = (char *)&(pBuffer->infoElements);
	
   /*
	* Informataion elements
	*/
	/* SSID */
	((dot11_SSID_t *)(pBuf))->hdr.eleId = DOT11_SSID_ELE_ID;
	((dot11_SSID_t *)(pBuf))->hdr.eleLen = pSsid->len;
	os_memoryCopy(pSiteMgr->hOs, pBuf + sizeof(dot11_eleHdr_t), (void *)pSsid->ssidString, pSsid->len);
	size += sizeof(dot11_eleHdr_t) + pSsid->len;
	pBuf += sizeof(dot11_eleHdr_t) + pSsid->len;

	/* Rates */
	pDot11Rates = (dot11_RATES_t *) pBuf;

    /* 
     * Supported rates in probe request will always use the default rates for BG or A bands,
     * regardless of the STA desired rates.
     */
    if (radioBand == RADIO_BAND_2_4_GHZ)
    {
        /* Basic rates: 1,2,5.5,11 */  
		basicRateMask = translateBasicRateValueToMask(pSiteMgr->pDesiredParams->siteMgrRegstryBasicRate[DOT11_G_MODE], FALSE);
        /* Extended: 6,9,12,18,24,36,48,54 */
        supportedRateMask = translateSupportedRateValueToMask(pSiteMgr->pDesiredParams->siteMgrRegstrySuppRate[DOT11_G_MODE], FALSE);
    }
    else if (radioBand == RADIO_BAND_5_0_GHZ)
    {   /* Basic rates: 6,12,24 */
        basicRateMask = translateBasicRateValueToMask(pSiteMgr->pDesiredParams->siteMgrRegstryBasicRate[DOT11_A_MODE], TRUE);
         /* Extended: 9,18,24,36,48,54 */
        supportedRateMask = translateSupportedRateValueToMask(pSiteMgr->pDesiredParams->siteMgrRegstrySuppRate[DOT11_A_MODE], TRUE);
	}
	else
	{
        WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  
            ("buildProbeReqTemplate, radioBand =%d ???\n",radioBand));
        /* Use default and pray for the best */
        /* Basic rates: 1,2,5.5,11 */  
        basicRateMask = translateBasicRateValueToMask(BASIC_RATE_SET_1_2_5_5_11, FALSE);
        /* Extended: 6,9,12,18,24,36,48,54 */
        supportedRateMask = translateSupportedRateValueToMask(SUPPORTED_RATE_SET_UP_TO_54, FALSE);
	}
	
	bitMapToNetworkStringRates(supportedRateMask,
							   basicRateMask,
                           ratesBuf, &len, &ofdmIndex);

	WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  
                            ("buildProbeReqTemplate, supportedRateMask=0x%x, basicRateMask=0x%x, len=%d, ofdmIndex=%d, radioBand =%d\n",
                             supportedRateMask,basicRateMask,len, ofdmIndex, radioBand));

    
    if(radioBand == RADIO_BAND_5_0_GHZ ||
       pSiteMgr->pDesiredParams->siteMgrUseDraftNum == DRAFT_5_AND_EARLIER ||
	   ofdmIndex == len)
	{
		pDot11Rates->hdr.eleId = DOT11_SUPPORTED_RATES_ELE_ID;
		pDot11Rates->hdr.eleLen = len;
		os_memoryCopy(pSiteMgr->hOs, (void *)pDot11Rates->rates, ratesBuf, pDot11Rates->hdr.eleLen);
		size += pDot11Rates->hdr.eleLen + sizeof(dot11_eleHdr_t);
		pBuf += pDot11Rates->hdr.eleLen + sizeof(dot11_eleHdr_t);		
	}
	else
	{
		pDot11Rates->hdr.eleId = DOT11_SUPPORTED_RATES_ELE_ID;
		pDot11Rates->hdr.eleLen = ofdmIndex;
		os_memoryCopy(pSiteMgr->hOs, (void *)pDot11Rates->rates, ratesBuf, pDot11Rates->hdr.eleLen);
		suppRatesLen = pDot11Rates->hdr.eleLen + sizeof(dot11_eleHdr_t);
		pDot11Rates = (dot11_RATES_t *) (pBuf + suppRatesLen); 
		pDot11Rates->hdr.eleId = DOT11_EXT_SUPPORTED_RATES_ELE_ID;
		pDot11Rates->hdr.eleLen = len - ofdmIndex;
		os_memoryCopy(pSiteMgr->hOs, (void *)pDot11Rates->rates, &ratesBuf[ofdmIndex], pDot11Rates->hdr.eleLen);
		extSuppRatesLen = pDot11Rates->hdr.eleLen + sizeof(dot11_eleHdr_t);
		size += suppRatesLen + extSuppRatesLen;
		pBuf += suppRatesLen + extSuppRatesLen;		
	}

	pTemplate->templateLen = size;
	
	return OK;
}

/************************************************************************
 *                        buildProbeRspTemplate							*
 ************************************************************************
DESCRIPTION: This function build a probe response template to set to the HAL 
				when joining an IBSS network.
				performs the following:
				-	Build a template & set the template len, the template type is set in the site mgr
				-	The template is built based on the chosen site attributes

			NOTE: This function is used to build beacon template too.
			The site manager set the template type (after thos function returns) to beacon or probe response accordingly.
                                                                                                   
INPUT:      pSiteMgr	-	Handle to site manager	
			pTemplate	-	Pointer to the template structure		


OUTPUT:		


RETURN:     OK

************************************************************************/
TI_STATUS buildProbeRspTemplate(siteMgr_t *pSiteMgr, whalCtrl_setTemplate_t *pTemplate)
{
	paramInfo_t			param;
	UINT8				*pBuf;
	probeRspTemplate_t	*pBuffer = (probeRspTemplate_t	*)pTemplate->pTemplate;
	siteEntry_t			*pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;
	int i;
	UINT32				size;
	dot11_RATES_t		*pDot11Rates;
	dot11_ERP_t         *pdot11Erp;
	UINT32				len = 0, ofdmIndex = 0;
	BOOL				extRates = FALSE;
	BOOL                useProtection,NonErpPresent,barkerPreambleType;
	UINT8				ratesBuf[MAX_SUPPORTED_RATES];
	UINT32				supportedRateMask,basicRateMask;

	os_memoryZero(pSiteMgr->hOs, pBuffer, sizeof(probeRspTemplate_t));

	/*
	 * Header First
	 */
	/* Set destination address */
	for (i = 0; i < MAC_ADDR_LEN; i++)
		pBuffer->hdr.DA.addr[i] = 0xFF;

	/* Set BSSID address */
	os_memoryCopy(pSiteMgr->hOs, &(pBuffer->hdr.BSSID.addr), &(pPrimarySite->bssid), MAC_ADDR_LEN);  

	/* Build Source address */
	param.paramType = CTRL_DATA_MAC_ADDRESS;
	ctrlData_getParam(pSiteMgr->hCtrlData, &param);
	os_memoryCopy(pSiteMgr->hOs, &(pBuffer->hdr.SA.addr), &(param.content.ctrlDataDeviceMacAddress), MAC_ADDR_LEN);  
	
	pBuffer->hdr.fc = ENDIAN_HANDLE_WORD(DOT11_FC_PROBE_RESP);

	size = sizeof(dot11_mgmtHeader_t);
	pBuf = (UINT8 *)pBuffer->timeStamp;
   /*
	* Fixed Fields
	*/
	/* we skip the timestamp field */
	size += TIME_STAMP_LEN;
	pBuf += TIME_STAMP_LEN;

	/* Beacon interval */
	*((UINT16 *)pBuf) = pPrimarySite->beaconInterval;
	size += FIX_FIELD_LEN;
	pBuf += FIX_FIELD_LEN;

	/* capabilities */
	*((UINT16 *)pBuf) = pPrimarySite->capabilities;
	size += FIX_FIELD_LEN;
	pBuf += FIX_FIELD_LEN;

	/*
	* Informataion elements
	*/
	/* SSID */
	((dot11_SSID_t *)(pBuf))->hdr.eleId = DOT11_SSID_ELE_ID;
	((dot11_SSID_t *)(pBuf))->hdr.eleLen = pPrimarySite->ssid.len;
	os_memoryCopy(pSiteMgr->hOs, pBuf + sizeof(dot11_eleHdr_t), (void *)pPrimarySite->ssid.ssidString, pPrimarySite->ssid.len);
	size += sizeof(dot11_eleHdr_t) + pPrimarySite->ssid.len;
	pBuf += sizeof(dot11_eleHdr_t) + pPrimarySite->ssid.len;

	/* Rates */

	pDot11Rates = (dot11_RATES_t *) pBuf;

	if (pPrimarySite->channel == SPECIAL_BG_CHANNEL) 
	{
		supportedRateMask = getSupportedRateMaskForSpecialBGchannel() ;
		basicRateMask	  = getBasicRateMaskForSpecialBGchannel();
	}
	else
	{
		supportedRateMask = pSiteMgr->pDesiredParams->siteMgrMatchedSuppRateMask;
		basicRateMask     = pSiteMgr->pDesiredParams->siteMgrMatchedBasicRateMask;
	}
	
	bitMapToNetworkStringRates(supportedRateMask,
							   basicRateMask,
							   ratesBuf, &len, &ofdmIndex);

    if(pSiteMgr->siteMgrOperationalMode != DOT11_G_MODE ||
       pSiteMgr->pDesiredParams->siteMgrUseDraftNum == DRAFT_5_AND_EARLIER ||
	   ofdmIndex == len)
	{
		pDot11Rates->hdr.eleId = DOT11_SUPPORTED_RATES_ELE_ID;
		pDot11Rates->hdr.eleLen = len;
		os_memoryCopy(pSiteMgr->hOs, (void *)pDot11Rates->rates, ratesBuf, pDot11Rates->hdr.eleLen);
		size += pDot11Rates->hdr.eleLen + sizeof(dot11_eleHdr_t);
		pBuf += pDot11Rates->hdr.eleLen + sizeof(dot11_eleHdr_t);		
	}
	else
	{
		pDot11Rates->hdr.eleId = DOT11_SUPPORTED_RATES_ELE_ID;
		pDot11Rates->hdr.eleLen = ofdmIndex;
		os_memoryCopy(pSiteMgr->hOs, (void *)pDot11Rates->rates, ratesBuf, pDot11Rates->hdr.eleLen);
		size += pDot11Rates->hdr.eleLen + sizeof(dot11_eleHdr_t);
		pBuf += pDot11Rates->hdr.eleLen + sizeof(dot11_eleHdr_t);
		extRates = TRUE;
	}

	/* DS */
	((dot11_DS_PARAMS_t *)(pBuf))->hdr.eleId = DOT11_DS_PARAMS_ELE_ID;
	((dot11_DS_PARAMS_t *)(pBuf))->hdr.eleLen = DOT11_DS_PARAMS_ELE_LEN;
	((dot11_DS_PARAMS_t *)(pBuf))->currChannel = pPrimarySite->channel;
	size += sizeof(dot11_eleHdr_t) + DOT11_DS_PARAMS_ELE_LEN;
	pBuf += sizeof(dot11_eleHdr_t) + DOT11_DS_PARAMS_ELE_LEN;

	/* IBSS */
	((dot11_IBSS_PARAMS_t *)(pBuf))->hdr.eleId = DOT11_IBSS_PARAMS_ELE_ID;
	((dot11_IBSS_PARAMS_t *)(pBuf))->hdr.eleLen = DOT11_IBSS_PARAMS_ELE_LEN;
#if 1
	((UINT8 *)&((dot11_IBSS_PARAMS_t *)(pBuf))->atimWindow)[0] = ((UINT8 *)&pPrimarySite->atimWindow)[0];
	((UINT8 *)&((dot11_IBSS_PARAMS_t *)(pBuf))->atimWindow)[1] = ((UINT8 *)&pPrimarySite->atimWindow)[1];
#else /* fix for WinCE */
	COPY_UNALIGNED_WORD(((UINT8 *)&((dot11_IBSS_PARAMS_t *)(pBuf))->atimWindow), &pPrimarySite->atimWindow);
#endif
	pPrimarySite->atimWindow = ENDIAN_HANDLE_WORD(pPrimarySite->atimWindow);

	size += sizeof(dot11_eleHdr_t) + DOT11_IBSS_PARAMS_ELE_LEN;
	pBuf += sizeof(dot11_eleHdr_t) + DOT11_IBSS_PARAMS_ELE_LEN;

	/* Add country Information Element */
	param.paramType = REGULATORY_DOMAIN_ENABLED_PARAM;
	regulatoryDomain_getParam(pSiteMgr->hRegulatoryDomain,&param);

	if(	param.content.regulatoryDomainEnabled == TRUE )
	{
        /* get country IE */
        param.paramType = REGULATORY_DOMAIN_COUNTRY_PARAM;
		regulatoryDomain_getParam(pSiteMgr->hRegulatoryDomain, &param);

        /* Check if a country IE was found */
		if(param.content.pCountry != NULL)
		{
			*pBuf = DOT11_COUNTRY_ELE_ID;
			pBuf += 1;
			size += 1;
			*pBuf = (UINT8)param.content.pCountry->len;
			pBuf += 1;
			size += 1;
			os_memoryCopy(pSiteMgr->hOs, pBuf , &param.content.pCountry->countryIE, param.content.pCountry->len);
			pBuf += param.content.pCountry->len;
			size += param.content.pCountry->len;
		}
	}
	 
	/*ERP IE*/
	siteMgr_IsERP_Needed(pSiteMgr,&useProtection,&NonErpPresent,&barkerPreambleType);
	if (useProtection || NonErpPresent || barkerPreambleType)
	{
		pdot11Erp = (dot11_ERP_t *) pBuf;
		pdot11Erp->hdr.eleId = DOT11_ERP_IE_ID;
		pdot11Erp->hdr.eleLen = 1;
		if (NonErpPresent)
			pdot11Erp->ctrl |= ERP_IE_NON_ERP_PRESENT_MASK;
		if (useProtection)
			pdot11Erp->ctrl |= ERP_IE_USE_PROTECTION_MASK;
		if (barkerPreambleType)
			pdot11Erp->ctrl |= ERP_IE_BARKER_PREAMBLE_MODE_MASK;
		size += pdot11Erp->hdr.eleLen + sizeof(dot11_eleHdr_t);
		pBuf += pdot11Erp->hdr.eleLen + sizeof(dot11_eleHdr_t);
		
	}


	/* Extended supported rates IE */
	if(extRates)
	{
		pDot11Rates = (dot11_RATES_t *) pBuf;
		pDot11Rates->hdr.eleId = DOT11_EXT_SUPPORTED_RATES_ELE_ID;
		pDot11Rates->hdr.eleLen = len - ofdmIndex;
		os_memoryCopy(pSiteMgr->hOs, (void *)pDot11Rates->rates, &ratesBuf[ofdmIndex], pDot11Rates->hdr.eleLen);
		size += pDot11Rates->hdr.eleLen + sizeof(dot11_eleHdr_t);
		pBuf += pDot11Rates->hdr.eleLen + sizeof(dot11_eleHdr_t);	
	}

    /* no need to insert RSN information elements */
		 
	pTemplate->templateLen = size;
	WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("Probe response template len = %d\n",size));
	
	return OK;
}

/************************************************************************
 *                        buildPsPollTemplate							*
 ************************************************************************
DESCRIPTION: This function build a ps poll template
				performs the following:
				-	Build a template & set the template len, the template type is set in the site mgr
                                                                                                   
INPUT:      pSiteMgr	-	Handle to site manager	
			pTemplate	-	Pointer to the template structure		
			pSsid		-	Desired SSID

OUTPUT:		

RETURN:     OK
************************************************************************/
TI_STATUS buildPsPollTemplate(siteMgr_t *pSiteMgr, whalCtrl_setTemplate_t *pTemplate)
{
    paramInfo_t			param;
    whalParamInfo_t     whalParam;
	UINT32				size;
	psPollTemplate_t	*pBuffer = (psPollTemplate_t *)pTemplate->pTemplate;
	siteEntry_t			*pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;
	UINT16				fc;

	os_memoryZero(pSiteMgr->hOs, pBuffer, sizeof(psPollTemplate_t));

	/*
	 * Header First
	 */
	
	/* Set BSSID address */
	os_memoryCopy(pSiteMgr->hOs, &(pBuffer->hdr.BSSID.addr), &(pPrimarySite->bssid), MAC_ADDR_LEN);  

	/* Build Source address */
	param.paramType = CTRL_DATA_MAC_ADDRESS;
	ctrlData_getParam(pSiteMgr->hCtrlData, &param);
	os_memoryCopy(pSiteMgr->hOs, &(pBuffer->hdr.TA.addr), &(param.content.ctrlDataDeviceMacAddress), MAC_ADDR_LEN);  
	
    /*
    **   Building the Frame Control word (16 bits)
    ** ---------------------------------------------
    ** Type = Control
    ** SubType = Power Save (PS) POLL,  */
    fc = DOT11_FC_PS_POLL;
    /*
    ** setting the Power Management bit in the Frame control field
    ** to be "Power Save mode"
    */
    fc |= (0x1 << DOT11_FC_PWR_MGMT_SHIFT);

    pBuffer->hdr.fc = ENDIAN_HANDLE_WORD(fc);

    /*
    **   Association ID
    ** -----------------
    */
    whalParam.paramType = HAL_CTRL_AID_PARAM;
    whalCtrl_GetParam (pSiteMgr->hHalCtrl, &whalParam) ;

    /* AID should have its two MSB bit Set to "1"*/
    pBuffer->hdr.AID = whalParam.content.halCtrlAid | 0xC000;

	size = sizeof(dot11_PsPollFrameHeader_t);

	pTemplate->templateLen = size;
	


	return OK;
}


/************************************************************************
 *                        buildQosNullDataTemplate							*
 ************************************************************************
DESCRIPTION: This function build a qos null data template
				performs the following:
				-	Build a template & set the template len, the template type is set in the site mgr
                                                                                                   
INPUT:      pSiteMgr	-	Handle to site manager	
			pTemplate	-	Pointer to the template structure		
			pSsid		-	Desired SSID

OUTPUT:		

RETURN:     OK
************************************************************************/
TI_STATUS buildQosNullDataTemplate(siteMgr_t *pSiteMgr, whalCtrl_setTemplate_t *pTemplate, UINT8 userPriority)
{
	paramInfo_t			param;
	UINT32				size;
	QosNullDataTemplate_t	*pBuffer = (QosNullDataTemplate_t	*)pTemplate->pTemplate;
	siteEntry_t			*pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;
	UINT16				fc;

	os_memoryZero(pSiteMgr->hOs, pBuffer, sizeof(QosNullDataTemplate_t));

	/*
	 * Header First
	 */
	/* Set destination address */
    if (pPrimarySite)
    {
	  os_memoryCopy(pSiteMgr->hOs, &(pBuffer->hdr.address1.addr), &(pPrimarySite->bssid), MAC_ADDR_LEN);  

	  /* Set BSSID address */
	  os_memoryCopy(pSiteMgr->hOs, &(pBuffer->hdr.address3.addr), &(pPrimarySite->bssid), MAC_ADDR_LEN);  
    }
    else
    {
	  WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("No Primary site so cannot fill QosNullData template\n"));
    }

	/* Build Source address */
	param.paramType = CTRL_DATA_MAC_ADDRESS;
	ctrlData_getParam(pSiteMgr->hCtrlData, &param);
	os_memoryCopy(pSiteMgr->hOs, &(pBuffer->hdr.address2.addr), &(param.content.ctrlDataDeviceMacAddress), MAC_ADDR_LEN);  

	fc = DOT11_FC_DATA_NULL_QOS;
	fc |= (TRUE << DOT11_FC_TO_DS_SHIFT);

    pBuffer->hdr.qosControl = (userPriority << QOS_CONTROL_UP_SHIFT);

	pBuffer->hdr.fc = ENDIAN_HANDLE_WORD(fc);

	size = sizeof(dot11_header_t);

	pTemplate->templateLen = size;
	
	return OK;    
}
   
   
   
   
   
