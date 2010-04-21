/** \file regulatoryDomain.c
 *  \brief regulatoryDomain module interface
 *
 *  \see regulatoryDomain.h
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

/************************************************************************************************/
/*  												*/
/*		MODULE:		regulatoryDomain.c					        */
/*		PURPOSE:	regulatoryDomain module interface.			        */
/*                  This module calculated the channel that should be scanned and that are      */
/*                   supported. Moreover, he set the transmit power level according to the      */
/*                   regulatory domain requirements and the supported channel.                  */
/*								 			        */
/************************************************************************************************/
#include "report.h"
#include "osApi.h"
#include "paramOut.h"
#include "utils.h"
#include "regulatoryDomain.h"
#include "regulatoryDomainApi.h"
#include "whalCtrl_api.h"
#include "siteMgrApi.h"
#include "whalHwCtrl.h"
#include "SwitchChannelApi.h"

/* Mask for retrieving the TxPower from the Scan Control Table */
#define MASK_TX_POWER					(0x1f) /* bits 0-4 indicates MaxTxPower */ 
#define MASK_ACTIVE_ALLOWED 			(0x40) /* bit 6 indiactes the channel is allowed for Active scan */
#define MASK_FREQ_ALLOWED 				(0x80) /* bit 7 indicates the cahnnel is allowed*/

#define CHANNEL_VALIDITY_TS_THRESHOLD   10000 /* 10 sec */

#define INVALID_CHANNEL_165 165

/********************************************************************************/
/*						Internal functions prototypes.							*/
/********************************************************************************/
static TI_STATUS regulatoryDomain_updateCurrTxPower(regulatoryDomain_t	*pRegulatoryDomain);

static void regulatoryDomain_setChannelValidity(regulatoryDomain_t *pRegulatoryDomain, 
												UINT16 channelNum, BOOL channelValidity);

static TI_STATUS setSupportedChannelsAccording2CountryIe(regulatoryDomain_t *pRegulatoryDomain, country_t*	pCountry, BOOL band_2_4);

static void setSupportedChannelsAccording2ScanControlTable(regulatoryDomain_t  *pRegulatoryDomain);

static TI_STATUS regulatoryDomain_getChannelCapability(regulatoryDomain_t *pRegulatoryDomain, 
													   channelCapabilityReq_t channelCapabilityReq, 
													   channelCapabilityRet_t *channelCapabilityRet);

static void regulatoryDomain_updateChannelsTs(regulatoryDomain_t *pRegulatoryDomain, UINT8 channel);

static void regulatoryDomain_buildDefaultListOfChannelsPerBand(regulatoryDomain_t *pRegulatoryDomain, radioBand_e band, UINT8 *listSize);

static void regulatoryDomain_checkCountryCodeExpiry(regulatoryDomain_t *pRegulatoryDomain);

static BOOL regulatoryDomain_isChannelSupprted(regulatoryDomain_t *pRegulatoryDomain, UINT8 channel);

static BOOL regulatoryDomain_isCountryFound(regulatoryDomain_t *pRegulatoryDomain, radioBand_e radioBand);

static void regulatoryDomain_getPowerLevelTableCB( TI_HANDLE hRegulatoryDomain, TI_STATUS status, 
												   UINT8* CB_buf );

static UINT8 regulatoryDomain_getMaxPowerAllowed(regulatoryDomain_t	*pRegulatoryDomain,
												 UINT8				uChannel,
												 radioBand_e		eBand,
												 BOOL				bServingChannel);

/********************************************************************************/
/*						Interface functions Implementation.						*/
/********************************************************************************/


/************************************************************************
 *                        regulatoryDomain_create									*
 ************************************************************************
DESCRIPTION: regulatoryDomain module creation function, called by the config mgr in creation phase 
				performs the following:
				-	Allocate the regulatoryDomain handle
				                                                                                                   
INPUT:      hOs -			Handle to OS		


OUTPUT:		

RETURN:     Handle to the regulatoryDomain module on success, NULL otherwise

************************************************************************/
TI_HANDLE regulatoryDomain_create(TI_HANDLE hOs)
{
	regulatoryDomain_t			*pRegulatoryDomain = NULL;
	
	/* allocating the regulatoryDomain object */
	pRegulatoryDomain = os_memoryAlloc(hOs,sizeof(regulatoryDomain_t));

	if (pRegulatoryDomain == NULL)
		return NULL;

	return(pRegulatoryDomain);
}

/************************************************************************
 *                        regulatoryDomain_config						*
 ************************************************************************
DESCRIPTION: regulatoryDomain module configuration function, called by the config mgr in configuration phase
				performs the following:
				-	Reset & initializes local variables
				-	Init the handles to be used by the module
                                                                                                   
INPUT:      hRegulatoryDomain	-	regulatoryDomain handle
			List of handles to be used by the module
			pRegulatoryDomainInitParams	-	Init table of the module.		


OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS regulatoryDomain_config(TI_HANDLE 	hRegulatoryDomain,
								TI_HANDLE		hSiteMgr,
					  			TI_HANDLE		hHalCtrl,
								TI_HANDLE		hReport,
								TI_HANDLE		hOs,
                                TI_HANDLE		hSwitchChannel,
								regulatoryDomainInitParams_t *pRegulatoryDomainInitParams)
{
	regulatoryDomain_t *pRegulatoryDomain = (regulatoryDomain_t *)hRegulatoryDomain;
	whalParamInfo_t	   tWhalParam;

	/* 
	 *init variables - must be the 1st thing that is init!
	 */
	pRegulatoryDomain->country_2_4_WasFound		= FALSE;
	pRegulatoryDomain->country_5_WasFound		= FALSE;
	pRegulatoryDomain->uExternTxPowerPreferred	= MAX_TX_POWER;	/* i.e. no restriction */
	pRegulatoryDomain->uPowerConstraint			= MIN_TX_POWER;	/* i.e. no restriction */
	 
	/* Init handlers */
	pRegulatoryDomain->hSiteMgr = hSiteMgr;
	pRegulatoryDomain->hHalCtrl	= hHalCtrl;
	pRegulatoryDomain->hReport	= hReport;
	pRegulatoryDomain->hOs	= hOs;
    pRegulatoryDomain->hSwitchChannel = hSwitchChannel;

	/* User max Tx power for all channels */
	pRegulatoryDomain->uUserMaxTxPower	  = pRegulatoryDomainInitParams->desiredTxPower; 
	/* Temporary Tx Power control to be used */
	pRegulatoryDomain->uDesiredTemporaryTxPower = pRegulatoryDomainInitParams->uTemporaryTxPower;
    pRegulatoryDomain->uTemporaryTxPower  = pRegulatoryDomainInitParams->uTemporaryTxPower;

    /* 
	 * Indicate the time in which the STA didn't receive any country code and was not connected, and therefore
     * will delete its current country code 
	 */
    pRegulatoryDomain->uTimeOutToResetCountryMs = pRegulatoryDomainInitParams->uTimeOutToResetCountryMs;
	pRegulatoryDomain->uLastCountryReceivedTS = 0;

	/* Get Power Translation Table from NVS. This table is retrieved only once */
	tWhalParam.paramType = HAL_CTRL_POWER_LEVEL_TABLE_PARAM;
	tWhalParam.content.interogateCmdCBParams.CB_Func = (void *)regulatoryDomain_getPowerLevelTableCB;
	tWhalParam.content.interogateCmdCBParams.CB_handle = hRegulatoryDomain;
	tWhalParam.content.interogateCmdCBParams.CB_buf = (UINT8*)(&(pRegulatoryDomain->tPowerLevelTableInterrogate));
	whalCtrl_GetParam( pRegulatoryDomain->hHalCtrl, &tWhalParam );
	

	pRegulatoryDomain->regulatoryDomainEnabled = pRegulatoryDomainInitParams->multiRegulatoryDomainEnabled;
	pRegulatoryDomain->spectrumManagementEnabled = pRegulatoryDomainInitParams->spectrumManagementEnabled;
	if (pRegulatoryDomain->spectrumManagementEnabled == TRUE)
	{
		pRegulatoryDomain->regulatoryDomainEnabled = TRUE;
	}
		
	/* Getting the desired Control Table contents for 2.4 Ghz*/
	os_memoryCopy(pRegulatoryDomain->hOs,
				  (void *)pRegulatoryDomain->scanControlTable.ScanControlTable24.tableString,
				  (void *)pRegulatoryDomainInitParams->desiredScanControlTable.ScanControlTable24.tableString,
					NUM_OF_CHANNELS_24 * sizeof(INT8));

	/* Getting the desired Control Table contents for 5 Ghz*/
	os_memoryCopy(pRegulatoryDomain->hOs,
				  (void *)pRegulatoryDomain->scanControlTable.ScanControlTable5.tableString,
				  (void *)pRegulatoryDomainInitParams->desiredScanControlTable.ScanControlTable5.tableString,
					A_5G_BAND_NUM_CHANNELS * sizeof(INT8));

	setSupportedChannelsAccording2ScanControlTable(pRegulatoryDomain);

    pRegulatoryDomain->minDFS_channelNum = A_5G_BAND_MIN_MIDDLE_BAND_DFS_CHANNEL;
    pRegulatoryDomain->maxDFS_channelNum = A_5G_BAND_MAX_UPPER_BAND_DFS_CHANNEL;

	WLAN_REPORT_INIT(hReport, REGULATORY_DOMAIN_MODULE_LOG,  (".....Regulatory domain configured successfully\n"));

	return OK;
}


/***********************************************************************
 *                        regulatoryDomain_setParam									
 ***********************************************************************
DESCRIPTION: Regulatory Domain set param function, called by the following:
			-	config mgr in order to set a parameter receiving from the OS abstraction layer.
			-	From inside the driver	
                                                                                                   
INPUT:      hRegulatoryDomain	-	Regulatory Domain handle.
			pParam	-	Pointer to the parameter		

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS regulatoryDomain_setParam(TI_HANDLE		hRegulatoryDomain,
											paramInfo_t	*pParam)
{
	regulatoryDomain_t *pRegulatoryDomain = (regulatoryDomain_t *)hRegulatoryDomain;
    
			
	switch(pParam->paramType)
	{
    case REGULATORY_DOMAIN_COUNTRY_PARAM:
        {
            BOOL        bBand_2_4;

            /* Sanity check */
            if (NULL == pParam->content.pCountry)
            {   
                WLAN_REPORT_ERROR(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
                  ("regulatoryDomain_setParam, REGULATORY_DOMAIN_COUNTRY_PARAM is set with NULL pointer"));

                return NOK;
            }
            else /* Update country code and supported channels */
            {         
                bBand_2_4 = siteMgr_isCurrentBand24(pRegulatoryDomain->hSiteMgr);

			    /* Setting the CountryIE for every Band */
			    setSupportedChannelsAccording2CountryIe(pRegulatoryDomain, pParam->content.pCountry, bBand_2_4);
            }
        }
		break;

    case REGULATORY_DOMAIN_CHECK_COUNTRY_PARAM:
        
        /* Check if Country code should be updated */
        regulatoryDomain_checkCountryCodeExpiry(pRegulatoryDomain);

        break;

	case REGULATORY_DOMAIN_SET_POWER_CONSTRAINT_PARAM:

        /* Update only if 11h enabled */
        if (pRegulatoryDomain->spectrumManagementEnabled)
		{	
            /* Convert to RegDomain units */
            UINT8 uNewPowerConstraint = DBM2DBMDIV10(pParam->content.powerConstraint);

			WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
							  ("SET_POWER_CONSTRAINT Old= %d New = %d (Only if bigger...)\n", 
							  pRegulatoryDomain->uPowerConstraint, uNewPowerConstraint));

			/* Update powerConstraint */
			if ( pRegulatoryDomain->uPowerConstraint != uNewPowerConstraint )
			{
				pRegulatoryDomain->uPowerConstraint = uNewPowerConstraint;
				/* Set new Tx power to Hal - only if needed ! */
				regulatoryDomain_updateCurrTxPower(pRegulatoryDomain);
			}
        }
		break;	
		
	case REGULATORY_DOMAIN_EXTERN_TX_POWER_PREFERRED:
		/* ExternTxPowerPreferred is the TX Power Control (TPC) */
		{
			/* Convert to RegDomain units */
			UINT8 uNewTPC = DBM2DBMDIV10(pParam->content.ExternTxPowerPreferred);

			WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
				("REGULATORY_DOMAIN_EXTERN_TX_POWER_PREFERRED Old= %d New = %d\n", 
				pRegulatoryDomain->uExternTxPowerPreferred, uNewTPC));

			if ( uNewTPC != pRegulatoryDomain->uExternTxPowerPreferred )
			{
				pRegulatoryDomain->uExternTxPowerPreferred = uNewTPC;
				/* Set new Tx power to Hal - only if needed ! */
				regulatoryDomain_updateCurrTxPower(pRegulatoryDomain);
			}
		}
		break;	
	
	case REGULATORY_DOMAIN_SET_CHANNEL_VALIDITY:
		/* Set channel as Valid or Invalid for Active SCAN only.
			Mainly used by DFS when Switch Channel is active */
		regulatoryDomain_setChannelValidity(pRegulatoryDomain, pParam->content.channelValidity.channelNum, 
															   pParam->content.channelValidity.channelValidity);
		break;
	
	case REGULATORY_DOMAIN_CURRENT_TX_POWER_IN_DBM_PARAM:
		/* This case is called when the desired Tx Power Level in Dbm is changed by the user */
        if(pRegulatoryDomain->uUserMaxTxPower != pParam->content.desiredTxPower)
        {
            pRegulatoryDomain->uUserMaxTxPower = pParam->content.desiredTxPower;			
			/* Set new Tx power to Hal - only if needed ! */
			regulatoryDomain_updateCurrTxPower(pRegulatoryDomain);
        }

		break;

	case REGULATORY_DOMAIN_TX_POWER_AFTER_SELECTION_PARAM:
		/* Called after joining BSS, set Tx power to Hal */

        WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
                          ("regulatoryDomain_setParam, REGULATORY_DOMAIN_TX_POWER_AFTER_SELECTION_PARAM \n"));

	   /* setting the Tx Power according to the selected channel */
        regulatoryDomain_updateCurrTxPower(pRegulatoryDomain);
        
		break;

    case REGULATORY_DOMAIN_DISCONNECT_PARAM:
        WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
                          ("regulatoryDomain_setParam, REGULATORY_DOMAIN_DISCONNECT_PARAM\n"));

        pRegulatoryDomain->uExternTxPowerPreferred = MAX_TX_POWER;	/* i.e. no restriction */
        pRegulatoryDomain->uPowerConstraint		   = MIN_TX_POWER;	/* i.e. no restriction */

        /* Update the last time a country code was used. 
        After uTimeOutToResetCountryMs the country code will be deleted     */
        if (pRegulatoryDomain->country_2_4_WasFound || pRegulatoryDomain->country_5_WasFound)
        {
            pRegulatoryDomain->uLastCountryReceivedTS = os_timeStampMs(pRegulatoryDomain->hOs);
        }
        break;

	case REGULATORY_DOMAIN_UPDATE_CHANNEL_VALIDITY:
		regulatoryDomain_updateChannelsTs(pRegulatoryDomain, pParam->content.channel);
		break;

    case REGULATORY_DOMAIN_TEMPORARY_TX_ATTENUATION_PARAM:
		/* Temporary Tx Power control */
		WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
						  (" temporary fix = %d, \n", pParam->content.bActivateTempPowerFix));

		/* Check if Temporary Tx Power control is enabled or disabled */
		if ( pParam->content.bActivateTempPowerFix )
        {   /* setting the Temporary Tx Power directly to Hal */
            whalParamInfo_t     whalParam;

            whalParam.content.halCtrlTxPowerDbm = pRegulatoryDomain->uTemporaryTxPower;
            whalParam.paramType = HAL_CTRL_TX_POWER_PARAM;
			WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
							  (" Setting actual temporary TX power = %d,Desired =%d \n", 
							   pRegulatoryDomain->uTemporaryTxPower,
							   pRegulatoryDomain->uDesiredTemporaryTxPower));
            whalCtrl_SetParam(pRegulatoryDomain->hHalCtrl, &whalParam);
        }    
		else	
		{	/* Exit from Temporary Tx Power control- return to normal Tx Power */
			regulatoryDomain_updateCurrTxPower(pRegulatoryDomain);
        }        

        break;

    case REGULATORY_DOMAIN_ENABLE_DISABLE_802_11D:
        WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
                          ("regulatoryDomain_setParam, REGULATORY_DOMAIN_ENABLE_DISABLE_802_11D = %d, \n", pParam->content.enableDisable_802_11d));

        if ((pRegulatoryDomain->regulatoryDomainEnabled != pParam->content.enableDisable_802_11d) &&
            !pParam->content.enableDisable_802_11d && pRegulatoryDomain->spectrumManagementEnabled)
        {   /* Disable of 802_11d, is not allowed when 802_11h is enabled */
            WLAN_REPORT_ERROR(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
                              ("regulatoryDomain_setParam, Disable of 802_11d, is not allowed when 802_11h is enabled  \n"));
            return NOK;
            
        }
        pRegulatoryDomain->regulatoryDomainEnabled = pParam->content.enableDisable_802_11d;

		/* Mark that no country was found - applies for both enabling and disabling of 11d */
		pRegulatoryDomain->country_2_4_WasFound = FALSE;
		pRegulatoryDomain->country_5_WasFound = FALSE;

        if (!pRegulatoryDomain->regulatoryDomainEnabled)
        {   /* Set regulatory Domain according to scan control table */
            setSupportedChannelsAccording2ScanControlTable(pRegulatoryDomain);
        }

		break;

    case REGULATORY_DOMAIN_ENABLE_DISABLE_802_11H:
        WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
                          ("regulatoryDomain_setParam, REGULATORY_DOMAIN_ENABLE_DISABLE_802_11H = %d, \n", pParam->content.enableDisable_802_11h));

        pRegulatoryDomain->spectrumManagementEnabled = pParam->content.enableDisable_802_11h;
        if (pParam->content.enableDisable_802_11h)
        {   /* If 802_11h is enabled, enable 802_11d as well */
            pRegulatoryDomain->regulatoryDomainEnabled = TRUE;
        }
        switchChannel_enableDisableSpectrumMngmt(pRegulatoryDomain->hSwitchChannel, pRegulatoryDomain->spectrumManagementEnabled);
		break;

	case REGULATORY_DOMAIN_COUNTRY_2_4_PARAM:
        /* NOTE !!! use this feature carefully. */
        WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
                          ("regulatoryDomain_setParam, REGULATORY_DOMAIN_COUNTRY_2_4_PARAM Len = %d, \n", pParam->paramLength));

        WLAN_REPORT_HEX_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, (UINT8*)pParam->content.pCountry, sizeof(country_t));

        return setSupportedChannelsAccording2CountryIe(pRegulatoryDomain, pParam->content.pCountry, TRUE);

	case REGULATORY_DOMAIN_COUNTRY_5_PARAM:
        /* NOTE !!! use this feature carefully */
        return setSupportedChannelsAccording2CountryIe(pRegulatoryDomain, pParam->content.pCountry, FALSE);


    case REGULATORY_DOMAIN_DFS_CHANNELS_RANGE:
        WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
                          ("regulatoryDomain_setParam, DFS_CHANNELS_RANGE, min = %d, max = %d, \n", 
                           pParam->content.DFS_ChannelRange.minDFS_channelNum,
                           pParam->content.DFS_ChannelRange.maxDFS_channelNum));
        if ((pParam->content.DFS_ChannelRange.minDFS_channelNum<A_5G_BAND_MIN_CHANNEL) ||
            (pParam->content.DFS_ChannelRange.maxDFS_channelNum>A_5G_BAND_MAX_CHANNEL) ||
            pParam->content.DFS_ChannelRange.minDFS_channelNum > pParam->content.DFS_ChannelRange.maxDFS_channelNum)
        {
            WLAN_REPORT_ERROR(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
                              ("regulatoryDomain_setParam, Bad DFS_CHANNELS_RANGE, min = %d, max = %d, \n", 
                               pParam->content.DFS_ChannelRange.minDFS_channelNum,
                               pParam->content.DFS_ChannelRange.maxDFS_channelNum));
            return NOK;
        }
        pRegulatoryDomain->minDFS_channelNum = (UINT8)pParam->content.DFS_ChannelRange.minDFS_channelNum;
        pRegulatoryDomain->maxDFS_channelNum = (UINT8)pParam->content.DFS_ChannelRange.maxDFS_channelNum;

        break;

	default:
		WLAN_REPORT_ERROR(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, ("Set param, Params is not supported, %d\n\n", pParam->paramType));
		return PARAM_NOT_SUPPORTED;
	}

	return OK;
}

/***********************************************************************
 *                        regulatoryDomain_getParam									
 ***********************************************************************
DESCRIPTION: Regulatory Domain get param function, called by the following:
			-	config mgr in order to get a parameter from the OS abstraction layer.
			-	From inside the driver	
                                                                                                   
INPUT:      hRegulatoryDomain	-	Regulatory Domain handle.
			pParam	-	Pointer to the parameter		

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS regulatoryDomain_getParam(TI_HANDLE		hRegulatoryDomain,
											paramInfo_t	*pParam)
{
	regulatoryDomain_t	*pRegulatoryDomain = (regulatoryDomain_t *)hRegulatoryDomain;

	switch(pParam->paramType)
	{

	case REGULATORY_DOMAIN_TX_POWER_LEVEL_TABLE_PARAM:
        /* Copy power translation table */
		os_memoryCopy(pRegulatoryDomain->hOs, (void*)&pParam->content.powerLevelTable,
			(void*)&(pRegulatoryDomain->tPowerLevelTableInterrogate.tTable), sizeof(powerLevelTable_t));

        break;

	case REGULATORY_DOMAIN_MANAGEMENT_CAPABILITY_ENABLED_PARAM:
		pParam->content.spectrumManagementEnabled = pRegulatoryDomain->spectrumManagementEnabled;
		break;
		
	case REGULATORY_DOMAIN_ENABLED_PARAM:
		pParam->content.regulatoryDomainEnabled = pRegulatoryDomain->regulatoryDomainEnabled;
		break;

	case REGULATORY_DOMAIN_GET_SCAN_CAPABILITIES:
		{
			channelCapabilityReq_t	channelCapabilityReq;

			channelCapabilityReq.band = pParam->content.channelCapabilityReq.band;
			channelCapabilityReq.channelNum = pParam->content.channelCapabilityReq.channelNum;
			channelCapabilityReq.scanOption = pParam->content.channelCapabilityReq.scanOption;

			regulatoryDomain_getChannelCapability(pRegulatoryDomain, channelCapabilityReq, &pParam->content.channelCapabilityRet);
		}
		break;

	case REGULATORY_DOMAIN_POWER_CAPABILITY_PARAM:
		/* power capability is only applicable when spectrum management is active (802.11h) */ 
		if(pRegulatoryDomain->spectrumManagementEnabled)
		{
			pParam->content.powerCapability.minTxPower = pRegulatoryDomain->uMinPowerDbm;
			pParam->content.powerCapability.maxTxPower = pRegulatoryDomain->uMaxPowerDbm;
		}
		else
		{
			return NOK;
		}
		break;

	case REGULATORY_DOMAIN_IS_CHANNEL_SUPPORTED:
		/* checking if the channel is supported */
		pParam->content.bIsChannelSupprted  = 
			regulatoryDomain_isChannelSupprted(pRegulatoryDomain, pParam->content.channel);
			
		break;

	case REGULATORY_DOMAIN_ALL_SUPPORTED_CHANNELS:
		{
			radioBand_e	band = pParam->content.siteMgrRadioBand;
			regulatoryDomain_buildDefaultListOfChannelsPerBand(pRegulatoryDomain, band, &pParam->content.supportedChannels.sizeOfList);
		    pParam->content.supportedChannels.listOfChannels = pRegulatoryDomain->pDefaultChannels;
		}
		break;

	case REGULATORY_DOMAIN_CURRENT_TX_POWER_IN_DBM_PARAM:

            {
			whalParamInfo_t		tWhalParam;
			/* Get last configured Tx power from Hal */
			tWhalParam.paramType = HAL_CTRL_TX_POWER_PARAM;
			whalCtrl_GetParam(pRegulatoryDomain->hHalCtrl, &tWhalParam);

			pParam->content.desiredTxPower = tWhalParam.content.halCtrlTxPowerDbm;

			WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
							  ("regulatoryDomain_getParam, CURRENT_TX_POWER_IN_DBM  = %d\n", 
							   pParam->content.desiredTxPower));
            }

        break;

    case REGULATORY_DOMAIN_COUNTRY_PARAM:
        {   
            /* This case is used as an inner function of the driver to retrieve the full IE of the country */

            BOOL bBand_2_4 = siteMgr_isCurrentBand24(pRegulatoryDomain->hSiteMgr);

            /* Check if country code is still valid */
            regulatoryDomain_checkCountryCodeExpiry(pRegulatoryDomain);
                
            if (bBand_2_4)
            {
                if (pRegulatoryDomain->country_2_4_WasFound)
                {
                    pParam->content.pCountry = &pRegulatoryDomain->country24;
                }
                else    /* Do not use the Inforamtion */
                {
                    pParam->content.pCountry = NULL;
                }
            }   /* band 5.0 */
            else
            { 
                if (pRegulatoryDomain->country_5_WasFound)
                {
                   pParam->content.pCountry = &pRegulatoryDomain->country5;
                }
                else    /* Do not use the Inforamtion */
                {
                    pParam->content.pCountry = NULL;
                }
            }
        }
        break;
        
	case REGULATORY_DOMAIN_COUNTRY_2_4_PARAM:
		/* Getting only country string */

        /* Check if country code is still valid */
        regulatoryDomain_checkCountryCodeExpiry(pRegulatoryDomain);

        if (pRegulatoryDomain->country_2_4_WasFound)
        {
            os_memoryCopy(pRegulatoryDomain->hOs, (void*)pParam->content.pCountryString, (void*)pRegulatoryDomain->country24.countryIE.CountryString, COUNTRY_STRING_LEN);
        }
        else
        {
            pParam->content.pCountryString[0] = '\0';
        }
 		break;

	case REGULATORY_DOMAIN_COUNTRY_5_PARAM:
		/* Getting only country string */

        /* Check if country code is still valid */
        regulatoryDomain_checkCountryCodeExpiry(pRegulatoryDomain);

        if (pRegulatoryDomain->country_5_WasFound)
        {
            os_memoryCopy(pRegulatoryDomain->hOs, (void*)pParam->content.pCountryString, (void*)pRegulatoryDomain->country5.countryIE.CountryString, COUNTRY_STRING_LEN);
        }
        else
        {
            pParam->content.pCountryString[0] = '\0';
        }
		break;

    case REGULATORY_DOMAIN_DFS_CHANNELS_RANGE:
        WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
                          ("regulatoryDomain_getParam, DFS_CHANNELS_RANGE, min = %d, max = %d, \n", 
                           pRegulatoryDomain->minDFS_channelNum,
                           pRegulatoryDomain->maxDFS_channelNum));
        pParam->content.DFS_ChannelRange.minDFS_channelNum = pRegulatoryDomain->minDFS_channelNum;
        pParam->content.DFS_ChannelRange.maxDFS_channelNum = pRegulatoryDomain->maxDFS_channelNum;

        break;

	case REGULATORY_DOMAIN_IS_COUNTRY_FOUND:

		pParam->content.bIsCountryFound = 
			 regulatoryDomain_isCountryFound(pRegulatoryDomain, pParam->content.eRadioBand);
		
		break;

	default:
		WLAN_REPORT_ERROR(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, ("Get param, Params is not supported, %d\n\n", pParam->paramType));
		return PARAM_NOT_SUPPORTED;
	}

	return OK;
}

/************************************************************************
 *                        regulatoryDomain_destroy						*
 ************************************************************************
DESCRIPTION: regulatoryDomain module destroy function, called by the config mgr in the destroy phase 
				performs the following:
				-	Free all memory allocated by the module
                                                                                                   
INPUT:      hRegulatoryDomain	-	regulatoryDomain handle.		


OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS regulatoryDomain_destroy(TI_HANDLE hRegulatoryDomain)
{
	regulatoryDomain_t	*pRegulatoryDomain = (regulatoryDomain_t *)hRegulatoryDomain;

	if (pRegulatoryDomain == NULL)
		return OK;

    utils_nullMemoryFree(pRegulatoryDomain->hOs, pRegulatoryDomain, sizeof(regulatoryDomain_t));

	return OK;
}

/************************************************************************
 *                        regulatoryDomain_isCountryFound						*
 ************************************************************************
DESCRIPTION: This function returns the validity of Country according to band
                                                                                                   
INPUT:      hRegulatoryDomain	-	regulatoryDomain handle.   
            radioBand           - the desired band 	


OUTPUT:		

RETURN:     TRUE - if country IE was found according to the band.
            FALSE - otherwise.

************************************************************************/
BOOL regulatoryDomain_isCountryFound(regulatoryDomain_t  *pRegulatoryDomain, radioBand_e radioBand)
{

    if(radioBand == RADIO_BAND_2_4_GHZ)
    {
            return pRegulatoryDomain->country_2_4_WasFound;
    }
    else
    {
        return pRegulatoryDomain->country_5_WasFound;
    }

}

/***********************************************************************
 *                       setSupportedChannelsAccording2CountryIe									
 ***********************************************************************
DESCRIPTION:	Called when beacon/Probe Response with Country IE
				is found.
				The function sets the local countryIE per band with the CountryIE
				 that was detected in the last passive scan.
				 It is assumed that only one Country IE per band is allowed.
				 If Country is changed when the TNET is loaded, it should
				 be re-loaded in order to re-config the new Country domain.
                                                                                                   
INPUT:      hRegulatoryDomain	-	RegulatoryDomain handle.
			pCountry	-	pointer to the detected country IE.

OUTPUT:		

RETURN:     OK - New country code was set (or the same one was already configured)
            NOK - The new country code could not be set

************************************************************************/
static TI_STATUS setSupportedChannelsAccording2CountryIe(regulatoryDomain_t *pRegulatoryDomain, country_t* pCountry, BOOL band_2_4)
{
	channelCapability_t *pSupportedChannels;
	UINT8				channelIndex;
	UINT8				tripletChannelIndex, tripletChannelCnt;
	UINT8				channelStep, numberOfChannels, minChannelNumber, maxChannelNumber;

	
	if (!pRegulatoryDomain->regulatoryDomainEnabled)
	{  /* Ignore the Country IE if 802.11d is disabled */
		return NOK;
	}

    /* Check if the country code should be reset */
    regulatoryDomain_checkCountryCodeExpiry(pRegulatoryDomain);

	if( band_2_4 == TRUE )
	{
		if (pRegulatoryDomain->country_2_4_WasFound)
		{	/* Do not update new Country IE */
			if (os_memoryCompare(pRegulatoryDomain->hOs, (void *)&pCountry->countryIE, 
				(void *)&pRegulatoryDomain->country24.countryIE, sizeof(countryIE_t)))
			{
				WLAN_REPORT_WARNING(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
									("setSupportedChannelsAccording2CountryIe different Country, cur=%s, new=%s\n", 
									pRegulatoryDomain->country24.countryIE.CountryString, pCountry->countryIE.CountryString));
            	return NOK;
            }
            else    /* Same IE - just mark the TS and return OK */
            {
                /* Mark the time of the received country IE */                
                pRegulatoryDomain->uLastCountryReceivedTS = os_timeStampMs(pRegulatoryDomain->hOs);
                return OK;
            }
		}
		pRegulatoryDomain->country_2_4_WasFound = TRUE;
		pSupportedChannels = pRegulatoryDomain->supportedChannels_band_2_4;
		channelStep = BG_24G_BAND_CHANNEL_HOPS;
		maxChannelNumber = NUM_OF_CHANNELS_24;
		minChannelNumber = BG_24G_BAND_MIN_CHANNEL;
		numberOfChannels = NUM_OF_CHANNELS_24;
		/* save the country IE */
		os_memoryCopy(pRegulatoryDomain->hOs, (void*)&pRegulatoryDomain->country24, (void *)pCountry, sizeof(country_t));

        WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
                                ("Country 2.4 =%c%c%c\n",pRegulatoryDomain->country24.countryIE.CountryString[0],
                                 pRegulatoryDomain->country24.countryIE.CountryString[1], 
								 pRegulatoryDomain->country24.countryIE.CountryString[2]));

	}
	else    /* band 5.0 */
	{
		if (pRegulatoryDomain->country_5_WasFound)
		{	/* Do not update new Country IE if the IE is the same*/
			if (os_memoryCompare(pRegulatoryDomain->hOs, (void *)&pCountry->countryIE, 
				(void *)&pRegulatoryDomain->country5.countryIE, sizeof(countryIE_t)))
			{
				WLAN_REPORT_WARNING(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
									("setSupportedChannelsAccording2CountryIe different Country, cur=%s, new=%s\n", 
									pRegulatoryDomain->country5.countryIE.CountryString, pCountry->countryIE.CountryString));
            	return NOK;
            }
            else    /* Same IE - just mark the TS and return OK */
            {
                /* Mark the time of the received country IE */                
                pRegulatoryDomain->uLastCountryReceivedTS = os_timeStampMs(pRegulatoryDomain->hOs);
                return OK;
            }
		}
		pRegulatoryDomain->country_5_WasFound = TRUE;
		pSupportedChannels = pRegulatoryDomain->supportedChannels_band_5;
		channelStep = A_5G_BAND_CHANNEL_HOPS;
		maxChannelNumber = A_5G_BAND_MAX_CHANNEL;
		minChannelNumber = A_5G_BAND_MIN_CHANNEL;
		numberOfChannels = A_5G_BAND_NUM_CHANNELS;
		/* save the country IE */
		os_memoryCopy(pRegulatoryDomain->hOs, (void*)&pRegulatoryDomain->country5, (void*)pCountry, sizeof(country_t));

        WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
                                ("Country 5 =%c%c%c\n",pRegulatoryDomain->country5.countryIE.CountryString[0],
                                 pRegulatoryDomain->country5.countryIE.CountryString[1], 
								 pRegulatoryDomain->country5.countryIE.CountryString[2]));
	}

    /*
     * New Country IE was saved. Now - update the last received TS and ScanControlTable
     */

    /* Mark the time of the received country IE */                
    pRegulatoryDomain->uLastCountryReceivedTS = os_timeStampMs(pRegulatoryDomain->hOs);

	/* First clear the validity of all channels 
		Overwrite the ScanControlTable */
	for (channelIndex=0; channelIndex<numberOfChannels; channelIndex++)
	{
		pSupportedChannels[channelIndex].channelValidityActive = FALSE;
		pSupportedChannels[channelIndex].channelValidityPassive = FALSE;
		pSupportedChannels[channelIndex].bChanneInCountryIe = FALSE;
		pSupportedChannels[channelIndex].uMaxTxPowerDomain = MIN_TX_POWER; 	
	}
    
	tripletChannelCnt = (pCountry->len - COUNTRY_STRING_LEN) / 3;
	/* set validity of the channels according to the band (2.4 or 5) */
	for( tripletChannelIndex = 0; tripletChannelIndex < tripletChannelCnt ; tripletChannelIndex++)
	{
		UINT8	firstChannelNumInTriplet;
		
		firstChannelNumInTriplet = pCountry->countryIE.tripletChannels[tripletChannelIndex].firstChannelNumber;
		WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
                                ("firstChannelNumInTriplet=%d,channelStep=%d\n", firstChannelNumInTriplet, channelStep));
		for (channelIndex=0; channelIndex<pCountry->countryIE.tripletChannels[tripletChannelIndex].numberOfChannels; channelIndex++)
		{
			UINT16	channelNumber;

			channelNumber = firstChannelNumInTriplet+(channelIndex*channelStep);
			WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
					("setSupportedChannelsAccording2CountryIe of channel=%d\n", channelNumber));
			
			if ((channelNumber <= maxChannelNumber)&& (channelNumber !=INVALID_CHANNEL_165))
			{
				UINT8 	channelIndex4Band;

				channelIndex4Band = (channelNumber-minChannelNumber);
				pSupportedChannels[channelIndex4Band].bChanneInCountryIe = TRUE;
				pSupportedChannels[channelIndex4Band].channelValidityPassive = TRUE;
				pSupportedChannels[channelIndex4Band].channelValidityActive = TRUE;

				/* set the TX power in DBM/10 units */
			    pSupportedChannels[channelIndex4Band].uMaxTxPowerDomain = 
					DBM2DBMDIV10(pCountry->countryIE.tripletChannels[tripletChannelIndex].maxTxPowerLevel);

				WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
                                        ("channel = %d uMaxTxPowerDomain=%d\n", 
										channelNumber, pSupportedChannels[channelIndex4Band].uMaxTxPowerDomain));
			}
		}
    }

	return OK;
}


/***********************************************************************
 *                        regulatoryDomain_isChannelSupprted									
 ***********************************************************************
DESCRIPTION:	The function checks if the input channel is supported.
                                                                                                   
INPUT:      pRegulatoryDomain	-	RegulatoryDomain pointer.
			channel				-	Channel number.
			

OUTPUT:		

RETURN:     OK if channel is supported, NOK otherwise.

************************************************************************/
static BOOL regulatoryDomain_isChannelSupprted(regulatoryDomain_t *pRegulatoryDomain, UINT8 channel)
{
	UINT8				channelIndex;
	channelCapability_t *pSupportedChannels;

	if (pRegulatoryDomain==NULL)
	{
		return FALSE;
	}

	if ((channel<BG_24G_BAND_MIN_CHANNEL) || (channel>A_5G_BAND_MAX_CHANNEL))
	{
		return FALSE;
	}
	if (channel>=A_5G_BAND_MIN_CHANNEL)
	{
		channelIndex = (channel-A_5G_BAND_MIN_CHANNEL);
		pSupportedChannels = pRegulatoryDomain->supportedChannels_band_5;
	}
	else
	{
		channelIndex = (channel-BG_24G_BAND_MIN_CHANNEL);
		pSupportedChannels = pRegulatoryDomain->supportedChannels_band_2_4;
	}
	if (pRegulatoryDomain->spectrumManagementEnabled 
		&& (channel >= pRegulatoryDomain->minDFS_channelNum) 
        && (channel <= pRegulatoryDomain->maxDFS_channelNum)
		&& ((os_timeStampMs(pRegulatoryDomain->hOs)-pSupportedChannels[channelIndex].timestamp) >=CHANNEL_VALIDITY_TS_THRESHOLD ))
	{	/* If 802.11h is enabled, a DFS channel is valid only for 10 sec
			from the last Beacon/ProbeResponse */
        pSupportedChannels[channelIndex].channelValidityActive = FALSE;
        WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, 
                                ("regulatoryDomain_isChannelSupprted(): CHANNEL_VALIDITY_TS_THRESHOLD !! Disable channel no %d, DFS channel\n", channel ));

	}

	return (pSupportedChannels[channelIndex].channelValidityActive);

}

/************************************************************************
 *                        regulatoryDomain_setChannelValidity			*
 ************************************************************************/
/*
*
*
* \b Description: 
*
* This function sets a channel as invalid or valid in the internal Regulatory Domain
 * database. 
*
* \b ARGS:
*
*  I   - pData - pointer to the regDoamin SM context  \n
*  I   - channelNum - the invalid/valid channel number
*  I   - channelValidity - TRUE if channel is valid, FALSE channel is invalid
*
* \b RETURNS:
*
*  None.
*
* 
*************************************************************************/
static void regulatoryDomain_setChannelValidity(regulatoryDomain_t *pRegulatoryDomain, 
												UINT16 channelNum, BOOL channelValidity)
{
	channelCapability_t		*pSupportedChannels;
	UINT8					channelIndex;


	if (pRegulatoryDomain == NULL)
	{
		return;
	}
	if ((channelNum==0 ) || (channelNum>A_5G_BAND_MAX_CHANNEL))
	{
		WLAN_REPORT_ERROR(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, 
								("regulatoryDomain_setChannelValidity, invalid channelNum=%d \n", channelNum));
		return;
	}
	
	if (channelNum <= NUM_OF_CHANNELS_24)
	{
		pSupportedChannels = pRegulatoryDomain->supportedChannels_band_2_4;
		channelIndex = (channelNum-BG_24G_BAND_MIN_CHANNEL);
	}
	else 
	{
		pSupportedChannels = pRegulatoryDomain->supportedChannels_band_5;
		channelIndex = (channelNum - A_5G_BAND_MIN_CHANNEL);
	}
	
	if(channelValidity == TRUE)
		if((pSupportedChannels[channelIndex].bChanneInCountryIe == FALSE) && (pRegulatoryDomain->regulatoryDomainEnabled == TRUE))
		{
			WLAN_REPORT_WARNING(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, 
									("regulatoryDomain_setChannelValidity: channelNum = %d isn't supported at the Country. wll not set to active!\n", channelNum));
			return;
		}

    WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, 
							("regulatoryDomain_setChannelValidity: channelNum=%d, validity=%d \n", channelNum, channelValidity));


	pSupportedChannels[channelIndex].channelValidityActive = channelValidity;
}


/************************************************************************
 *      setSupportedChannelsAccording2ScanControlTable 					*
 ************************************************************************/
/**
*
*
* \b Description: 
*
* This function is called in config and sets the supported channels according to
* the scan control table read from registry and reg domain read from the chip.
*
* \b ARGS:
*
*  I   - pRegulatoryDomain - pointer to the regDoamin SM context  \n
*
* \b RETURNS:
*
*  None.
*
* 
*************************************************************************/
static void setSupportedChannelsAccording2ScanControlTable(regulatoryDomain_t  *pRegulatoryDomain)
{
	UINT8 	channelIndex;
	UINT8	channelMask;

	if (pRegulatoryDomain==NULL)
	{
		return;
	}

	WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, 
							("setSupportedChannelsAccording2ScanControlTable \n"));

	for (channelIndex=0; channelIndex<NUM_OF_CHANNELS_24; channelIndex++)
	{
		channelMask = pRegulatoryDomain->scanControlTable.ScanControlTable24.tableString[channelIndex];
		pRegulatoryDomain->supportedChannels_band_2_4[channelIndex].bChanneInCountryIe = FALSE;

		/* Calculate Domain Tx Power - channelMask units are in Dbm. */
		pRegulatoryDomain->supportedChannels_band_2_4[channelIndex].uMaxTxPowerDomain = 
						DBM2DBMDIV10(channelMask & MASK_TX_POWER);
		if (channelMask & (MASK_ACTIVE_ALLOWED | MASK_FREQ_ALLOWED))
		{	/* The channel is allowed for Active & Passive scans */
			if (pRegulatoryDomain->regulatoryDomainEnabled)
			{	/* All channels should be invalid for Active scan */
				pRegulatoryDomain->supportedChannels_band_2_4[channelIndex].channelValidityActive = FALSE;
				WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, 
					("channelIndex=%d is invalid for Active \n", channelIndex+1));
			}
			else
			{
				pRegulatoryDomain->supportedChannels_band_2_4[channelIndex].channelValidityActive = TRUE;
				WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, 
					("channelIndex=%d is Active valid \n", channelIndex+1));
			}
			
		}
		
		if (channelMask & MASK_FREQ_ALLOWED)
		{	/* The channel is allowed for Passive scan */
			pRegulatoryDomain->supportedChannels_band_2_4[channelIndex].channelValidityPassive = TRUE;
			WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, 
									("channelIndex=%d is Passive valid \n", channelIndex+1));
		}
		else
		{	/* The channel is not allowed */
			pRegulatoryDomain->supportedChannels_band_2_4[channelIndex].channelValidityPassive = FALSE;
			pRegulatoryDomain->supportedChannels_band_2_4[channelIndex].channelValidityActive = FALSE;
		}
	}

	for (channelIndex=A_5G_BAND_MIN_CHANNEL; channelIndex<=A_5G_BAND_MAX_CHANNEL; channelIndex++)
	{	
		UINT8	channelIndexInBand5;

		channelIndexInBand5 = (channelIndex-A_5G_BAND_MIN_CHANNEL);
		channelMask = pRegulatoryDomain->scanControlTable.ScanControlTable5.tableString[channelIndexInBand5];
		WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
								("channelIndex=%d, channelIndexInBand5=%d channelMask=%d\n", channelIndex, channelIndexInBand5, channelMask));

		/* Calculate Domain Tx Power - channelMask units are in Dbm. */
		pRegulatoryDomain->supportedChannels_band_5[channelIndexInBand5].uMaxTxPowerDomain = 
			DBM2DBMDIV10(channelMask & MASK_TX_POWER);

		pRegulatoryDomain->supportedChannels_band_5[channelIndexInBand5].bChanneInCountryIe = FALSE;
		if (channelMask & (MASK_ACTIVE_ALLOWED | MASK_FREQ_ALLOWED))
		{	 /* The channel is allowed for Active & Passive scans */
			if (pRegulatoryDomain->regulatoryDomainEnabled)
			{	/* All channels should be invalid for Active scan */
				pRegulatoryDomain->supportedChannels_band_5[channelIndexInBand5].channelValidityActive = FALSE;
				WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, 
					("channelIndex=%d is invalid for Active \n", channelIndex));
			}
			else
			{
				pRegulatoryDomain->supportedChannels_band_5[channelIndexInBand5].channelValidityActive = TRUE;
				WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, 
				  ("channelIndex=%d, channelIndexInBand5=%d, is Active valid \n", channelIndex, channelIndexInBand5));
			}   		
		}
		
		if (channelMask & MASK_FREQ_ALLOWED)
		{	/* The channel is allowed for Passive scan */
			pRegulatoryDomain->supportedChannels_band_5[channelIndexInBand5].channelValidityPassive = TRUE;
			WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, 
			   ("channelIndex=%d, channelIndexInBand5=%d, is Passive valid \n", channelIndex, channelIndexInBand5));
		}
		else
		{	/* The channel is not allowed */
			pRegulatoryDomain->supportedChannels_band_5[channelIndexInBand5].channelValidityPassive = FALSE;
			pRegulatoryDomain->supportedChannels_band_5[channelIndexInBand5].channelValidityActive = FALSE;
		}

	}
}


/***********************************************************************
*                        regulatoryDomain_getChannelCapability									
***********************************************************************
DESCRIPTION:	This function returns the channel capability information

INPUT:      pRegulatoryDomain		-	RegulatoryDomain pointer.
			channelCapabilityReq	-	Channels parameters


OUTPUT:		channelCapabilityRet	-   Channel capability information

RETURN:     OK if information was retrieved, NOK otherwise.

************************************************************************/
static TI_STATUS regulatoryDomain_getChannelCapability(regulatoryDomain_t *pRegulatoryDomain, 
													   channelCapabilityReq_t channelCapabilityReq, 
													   channelCapabilityRet_t *channelCapabilityRet)
{
	channelCapability_t		*pSupportedChannels;
	UINT8					channelIndex;
	BOOL					bCountryWasFound, bServingChannel;
	paramInfo_t				tParamInfo;

	if ((pRegulatoryDomain == NULL) || (channelCapabilityRet == NULL))
	{
		return NOK;
	}
	
	channelCapabilityRet->channelValidity = FALSE;
	channelCapabilityRet->maxTxPowerDbm = 0;
	if ((channelCapabilityReq.channelNum==0 ) || (channelCapabilityReq.channelNum>A_5G_BAND_MAX_CHANNEL))
	{
		WLAN_REPORT_ERROR(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, 
								("regulatoryDomain_getChannelCapability, invalid channelNum=%d \n", channelCapabilityReq.channelNum));
		return NOK;
	}
	
	if (channelCapabilityReq.band==RADIO_BAND_2_4_GHZ)
	{
		pSupportedChannels = pRegulatoryDomain->supportedChannels_band_2_4;
		channelIndex = (channelCapabilityReq.channelNum-BG_24G_BAND_MIN_CHANNEL);
		bCountryWasFound = pRegulatoryDomain->country_2_4_WasFound;
	}
	else if (channelCapabilityReq.band==RADIO_BAND_5_0_GHZ)
	{
		pSupportedChannels = pRegulatoryDomain->supportedChannels_band_5;
		channelIndex = (channelCapabilityReq.channelNum - A_5G_BAND_MIN_CHANNEL);
		bCountryWasFound = pRegulatoryDomain->country_5_WasFound;
	}
	else
	{
		WLAN_REPORT_ERROR(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, 
								("regulatoryDomain_getChannelCapability, invalid band=%d \n", channelCapabilityReq.band));
		return NOK;
	}


	/* 
	 * Set channelValidity according to ScanTable and whether 11d is enabled 
	 */
	if (channelCapabilityReq.scanOption == ACTIVE_SCANNING)
	{
		if ( ( pRegulatoryDomain->regulatoryDomainEnabled ) && ( !bCountryWasFound ) )
		{	/* 11d enabled and no country IE was found - set channel to invalid */
			channelCapabilityRet->channelValidity = FALSE;
		}
		else
		{
        channelCapabilityRet->channelValidity = pSupportedChannels[channelIndex].channelValidityActive;
			/*
			 * Set Maximum Tx power for the channel - only for active scanning
			 */ 

			/* Get current channel and check if we are using the same one */
			tParamInfo.paramType = SITE_MGR_CURRENT_CHANNEL_PARAM;
			siteMgr_getParam(pRegulatoryDomain->hSiteMgr, &tParamInfo);

			bServingChannel = ( tParamInfo.content.siteMgrCurrentChannel == channelCapabilityReq.channelNum ?
								TRUE : FALSE );

			channelCapabilityRet->maxTxPowerDbm = regulatoryDomain_getMaxPowerAllowed(pRegulatoryDomain, 
				channelCapabilityReq.channelNum, 
				channelCapabilityReq.band, 
				bServingChannel);
		}
	}
	else	/* Passive scanning */
	{
		if ( ( pRegulatoryDomain->regulatoryDomainEnabled ) && ( !bCountryWasFound ) )
		{	/* 11d enabled and no country IE was found - set channel to valid for passive scan */
			channelCapabilityRet->channelValidity = TRUE;
		}
	else
	{
		channelCapabilityRet->channelValidity = pSupportedChannels[channelIndex].channelValidityPassive;
	}
	}
	
	if (pRegulatoryDomain->spectrumManagementEnabled 
		&& (channelCapabilityReq.scanOption == ACTIVE_SCANNING)
        && (channelCapabilityReq.channelNum >= pRegulatoryDomain->minDFS_channelNum) 
        && (channelCapabilityReq.channelNum <= pRegulatoryDomain->maxDFS_channelNum)
		&& ((os_timeStampMs(pRegulatoryDomain->hOs)-pSupportedChannels[channelIndex].timestamp) >=CHANNEL_VALIDITY_TS_THRESHOLD ))
	{	/* If 802.11h is enabled, a DFS channel is valid only for 10 sec
			from the last Beacon/ProbeResponse */
        pSupportedChannels[channelIndex].channelValidityActive = FALSE;
        channelCapabilityRet->channelValidity = FALSE;
        WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, 
                                ("regulatoryDomain_getChannelCapability(): CHANNEL_VALIDITY_TS_THRESHOLD !!! Disable channel no %d, DFS channel\n", channelCapabilityReq.channelNum  ));
    }

	WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, 
			(" Channel num= %d, scan option=%d validity = %d, TX power = %d \n", 
					channelCapabilityReq.channelNum, 
					channelCapabilityReq.scanOption,
					channelCapabilityRet->channelValidity,
					channelCapabilityRet->maxTxPowerDbm));
	return OK;

}


static void regulatoryDomain_updateChannelsTs(regulatoryDomain_t *pRegulatoryDomain, UINT8 channel)
{
	UINT8				channelIndex;
	channelCapability_t *pSupportedChannels;

	if (pRegulatoryDomain==NULL)
	{
		return;
	}

	if ((channel<BG_24G_BAND_MIN_CHANNEL) || (channel>A_5G_BAND_MAX_CHANNEL))
	{
		return;
	}

	if (channel>=A_5G_BAND_MIN_CHANNEL)
	{
		channelIndex = (channel-A_5G_BAND_MIN_CHANNEL);
		pSupportedChannels = pRegulatoryDomain->supportedChannels_band_5;
	}
	else
	{
		channelIndex = (channel-BG_24G_BAND_MIN_CHANNEL);
		pSupportedChannels = pRegulatoryDomain->supportedChannels_band_2_4;
	}
	
	if((pSupportedChannels[channelIndex].bChanneInCountryIe == FALSE) && (pRegulatoryDomain->regulatoryDomainEnabled == TRUE))
  	{
  		WLAN_REPORT_WARNING(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, 
  								("regulatoryDomain_updateChannelsTs: channelNum = %d isn't supported at the Country. wll not set to active!\n", channel));
  		return;
  	}

	pSupportedChannels[channelIndex].timestamp = os_timeStampMs(pRegulatoryDomain->hOs);
	pSupportedChannels[channelIndex].channelValidityActive = TRUE;

}

/***********************************************************************
 *              regulatoryDomain_updateCurrTxPower								
 ***********************************************************************
DESCRIPTION: Called when new Tx power should be calculated and configured.
			 Check if we are already joined to BSS/IBSS, calculate 
			 new Tx power and configure it to Hal.
				
INPUT:		pRegulatoryDomain	- regulatoryDomain pointer.
			
RETURN:     OK - New value was configured to Hal, NOK - Can't configure value
			TX_POWER_SET_SAME_VALUE - Same value was already configured.

************************************************************************/
static TI_STATUS regulatoryDomain_updateCurrTxPower(regulatoryDomain_t	*pRegulatoryDomain)
{
	UINT8				uCurrChannel, uNewTxPower;
	whalParamInfo_t		whalParam;
	paramInfo_t			tParamInfo;
	TI_STATUS			eStatus;

	/* Get the current channel, and update HAL with the changed */
	tParamInfo.paramType = SITE_MGR_CURRENT_CHANNEL_PARAM;
	eStatus = siteMgr_getParam(pRegulatoryDomain->hSiteMgr, &tParamInfo);

	if ( eStatus != OK )
	{
		/* We are not joined yet - no meaning for new Tx power */
		WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, 
			("regulatoryDomain_updateCurrTxPower, No site selected yet\n"));

		return NOK;
	}
	/* Save current channel */
	uCurrChannel = tParamInfo.content.siteMgrCurrentChannel;

	/* Get the current channel, and update HAL with the changed */
	tParamInfo.paramType = 	SITE_MGR_RADIO_BAND_PARAM;
	siteMgr_getParam(pRegulatoryDomain->hSiteMgr, &tParamInfo);

	/* Calculate maximum Tx power for the serving channel */
	uNewTxPower = regulatoryDomain_getMaxPowerAllowed(pRegulatoryDomain, uCurrChannel, 
													  tParamInfo.content.siteMgrRadioBand, TRUE);
	/* Verify that the Temporary TX Power Control doesn't violate the TX Power Constraint */
	pRegulatoryDomain->uTemporaryTxPower = MIN(pRegulatoryDomain->uDesiredTemporaryTxPower, uNewTxPower);

	WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG, 
					("regulatoryDomain_updateCurrTxPower, Write to Hal = %d \n", uNewTxPower));

    whalParam.paramType = HAL_CTRL_TX_POWER_PARAM;
	whalParam.content.halCtrlTxPowerDbm = uNewTxPower;
	return whalCtrl_SetParam(pRegulatoryDomain->hHalCtrl, &whalParam);
}

/***********************************************************************
 *                        regulatoryDomain_checkCountryCodeExpiry									
 ***********************************************************************
DESCRIPTION: Check & Reset the country code that was detected earlier.
                Reseting country code will be done when the STA was not connected for 
                a certain amount of time, and no country code was received in that period (from the same country).
                This scenario might indicate that the STA has moved to a different country.
                                                                                                   
INPUT:      pRegulatoryDomain	-	Regulatory Domain handle.

OUTPUT:		updating country code if necessary.

RETURN:     

************************************************************************/
void regulatoryDomain_checkCountryCodeExpiry(regulatoryDomain_t *pRegulatoryDomain)
{
    paramInfo_t param;
    TI_STATUS   connStatus;
    UINT32      uCurrentTS = os_timeStampMs(pRegulatoryDomain->hOs);

    if ((pRegulatoryDomain->country_2_4_WasFound) || (pRegulatoryDomain->country_5_WasFound))
    {
        /* Get connection status */
        param.paramType = SITE_MGR_CURRENT_SSID_PARAM;
        connStatus      = siteMgr_getParam(pRegulatoryDomain->hSiteMgr, &param);    

         /* If (uTimeOutToResetCountryMs has elapsed && we are not connected)
                 delete the last country code received */
        if (((uCurrentTS - pRegulatoryDomain->uLastCountryReceivedTS) > pRegulatoryDomain->uTimeOutToResetCountryMs) &&
            (connStatus == NO_SITE_SELECTED_YET))
        {
            WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
                 ("%s, Reset country code after %d Ms\n",
                 __FUNCTION__,(uCurrentTS - pRegulatoryDomain->uLastCountryReceivedTS)));

            /* Reset country codes */
            pRegulatoryDomain->country_2_4_WasFound = FALSE;
            pRegulatoryDomain->country_5_WasFound = FALSE;
            
            /* Restore default values of the scan control table */
            setSupportedChannelsAccording2ScanControlTable(pRegulatoryDomain); 
        } 
    }
}

/***********************************************************************
*              regulatoryDomain_getMaxPowerAllowed								
***********************************************************************
DESCRIPTION: Get the maximum tx power allowed for the given channel.
				The final value is constructed by:
				1) User max value
				2) Domain restriction - 11d country code IE
				3) 11h power constraint - only on serving channel
				4) EXC TPC - only on serving channel

RETURN:     Max power in Dbm/10 for the given channel

************************************************************************/
static UINT8 regulatoryDomain_getMaxPowerAllowed(regulatoryDomain_t	*pRegulatoryDomain,
												 UINT8				uChannel,
												 radioBand_e		eBand,
												 BOOL				bServingChannel)
{
	channelCapability_t	*pSupportedChannels;
	UINT8				 uChannelIndex, uTxPower;

	if( eBand == RADIO_BAND_2_4_GHZ)
	{
		pSupportedChannels = pRegulatoryDomain->supportedChannels_band_2_4;
		uChannelIndex = uChannel - BG_24G_BAND_MIN_CHANNEL;
	}
	else
	{
		pSupportedChannels = pRegulatoryDomain->supportedChannels_band_5;
		uChannelIndex = uChannel - A_5G_BAND_MIN_CHANNEL;
	}

	/* We'll start with the "Domain restriction - 11d country code IE" */
	uTxPower = pSupportedChannels[uChannelIndex].uMaxTxPowerDomain;

	if ( bServingChannel)
	{
		if (pRegulatoryDomain->uPowerConstraint < uTxPower)
		{
			/* When 802.11h is disabled, uPowerConstraint is 0 anyway */
			uTxPower -= pRegulatoryDomain->uPowerConstraint;
		}
        
        /* Take EXC limitation too */
        uTxPower = MIN(uTxPower, pRegulatoryDomain->uExternTxPowerPreferred);

	}

	/* Now make sure we are not exceeding the user maximum */
	uTxPower = MIN(uTxPower, pRegulatoryDomain->uUserMaxTxPower);

	WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
		("%s uChannel = %d bServingChannel = %d uTxPower = %d \n", 
		__FUNCTION__, uChannel, bServingChannel, uTxPower));

	return uTxPower;
}


static void regulatoryDomain_buildDefaultListOfChannelsPerBand(regulatoryDomain_t *pRegulatoryDomain, radioBand_e band, UINT8 *listSize)
{
	UINT8				channelIndex;
	UINT8				numberOfChannels, minChannelNumber;
	channelCapability_t	*pSupportedChannels;
	UINT8				maxSupportedChannels=0;

	if ( (pRegulatoryDomain==NULL) || (listSize==NULL))
	{
		return;
	}

	if( band == RADIO_BAND_2_4_GHZ)
	{
		minChannelNumber = BG_24G_BAND_MIN_CHANNEL;
		numberOfChannels = NUM_OF_CHANNELS_24;
		pSupportedChannels = pRegulatoryDomain->supportedChannels_band_2_4;
	}
	else
	{
		minChannelNumber = A_5G_BAND_MIN_CHANNEL;
		numberOfChannels = A_5G_BAND_NUM_CHANNELS;
		pSupportedChannels = pRegulatoryDomain->supportedChannels_band_5;
	}


	for (channelIndex=0; channelIndex<numberOfChannels; channelIndex++)
	{
		if (pSupportedChannels[channelIndex].channelValidityPassive)
		{
			pRegulatoryDomain->pDefaultChannels[maxSupportedChannels] = channelIndex+minChannelNumber;
			WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
									("Channel num %d is supported \n", pRegulatoryDomain->pDefaultChannels[maxSupportedChannels]));
			maxSupportedChannels++;
		}
	}
 
	*listSize = maxSupportedChannels;
    
}

/***********************************************************************
*              regulatoryDomain_getPowerLevelTableCB								
***********************************************************************
DESCRIPTION: CB for retrieving power level table from FW (NVS).
			 The table is made of 4 power levels and 5 bands/sub-bands. For
			 each power level there's a maximum value of Dbm to be used.

RETURN:     void

************************************************************************/
static void regulatoryDomain_getPowerLevelTableCB( TI_HANDLE hRegulatoryDomain, TI_STATUS status, 
												  UINT8* CB_buf )
{
	regulatoryDomain_t  *pRegulatoryDomain = (regulatoryDomain_t *)hRegulatoryDomain;
	UINT8	i;

	/* Calculate Min and Max values of the table */
	pRegulatoryDomain->uMinPowerDbm = MAX_TX_POWER;
	pRegulatoryDomain->uMaxPowerDbm = MIN_TX_POWER; 
	for ( i = 0 ; i < NUM_SUB_BANDS_FOR_POWER_TABLE ; i++ )
	{
		WLAN_REPORT_INFORMATION(pRegulatoryDomain->hReport, REGULATORY_DOMAIN_MODULE_LOG,
			("PowerTable sub-band %i : %d %d %d %d\n", i,  
			pRegulatoryDomain->tPowerLevelTableInterrogate.tTable.uDbm[i][0],
			pRegulatoryDomain->tPowerLevelTableInterrogate.tTable.uDbm[i][1],
			pRegulatoryDomain->tPowerLevelTableInterrogate.tTable.uDbm[i][2],
			pRegulatoryDomain->tPowerLevelTableInterrogate.tTable.uDbm[i][3]));

		pRegulatoryDomain->uMinPowerDbm = MIN(pRegulatoryDomain->uMinPowerDbm, 
											  pRegulatoryDomain->tPowerLevelTableInterrogate.tTable.uDbm[i][MIN_POWER_LEVEL]);
		pRegulatoryDomain->uMaxPowerDbm = MAX(pRegulatoryDomain->uMaxPowerDbm, 
			pRegulatoryDomain->tPowerLevelTableInterrogate.tTable.uDbm[i][MAX_POWER_LEVEL]);

	}
}

/* for debug */
void regDomainPrintValidTables(TI_HANDLE hRegulatoryDomain)
{
	regulatoryDomain_t  *pRegulatoryDomain = (regulatoryDomain_t *)hRegulatoryDomain;
	UINT16 channelIndex;

	for (channelIndex=0; channelIndex<NUM_OF_CHANNELS_24; channelIndex++)
	{
		if (pRegulatoryDomain->supportedChannels_band_2_4[channelIndex].channelValidityPassive)
			WLAN_OS_REPORT(("channel num =%d is valid for passive \n", channelIndex+1));
		if (pRegulatoryDomain->supportedChannels_band_2_4[channelIndex].channelValidityActive)
		{
			WLAN_OS_REPORT(("channel =%d is valid for active TX power=%d\n", 
				channelIndex+1, pRegulatoryDomain->supportedChannels_band_2_4[channelIndex].uMaxTxPowerDomain));
		}
	}

	for (channelIndex=0; channelIndex<A_5G_BAND_NUM_CHANNELS; channelIndex++)
	{
		UINT8	channelNum;
		channelNum = channelIndex+A_5G_BAND_MIN_CHANNEL;
		if (pRegulatoryDomain->supportedChannels_band_5[channelIndex].channelValidityPassive)
			WLAN_OS_REPORT(("channel =%d is valid for passive \n", channelNum));
		if (pRegulatoryDomain->supportedChannels_band_5[channelIndex].channelValidityActive)
		{
			WLAN_OS_REPORT(("channel =%d is valid for active TX power=%d\n", 
				channelNum,pRegulatoryDomain->supportedChannels_band_5[channelIndex].uMaxTxPowerDomain));
		}
		}

	WLAN_OS_REPORT(("11h PowerConstraint = %d, EXC TPC = %d, User  = %d\n", 
		pRegulatoryDomain->uPowerConstraint, pRegulatoryDomain->uExternTxPowerPreferred,
		pRegulatoryDomain->uUserMaxTxPower));

}
