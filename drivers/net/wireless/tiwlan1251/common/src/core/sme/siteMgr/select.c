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
/*																									*/
/*		MODULE:	reportReplvl.c																*/
/*    PURPOSE:	Report level implementation	 										*/
/*																									*/
/***************************************************************************/
#include "osTIType.h"
#include "osApi.h"
#include "siteHash.h"
#include "smeSmApi.h"
#include "utils.h"
#include "smeApi.h"
#include "rsnApi.h"
#include "report.h"
#include "whalCtrl_api.h"
#include "connApi.h"
#include "DataCtrl_Api.h"  
#include "siteMgrApi.h"  
#include "regulatoryDomainApi.h"
#include "measurementMgrApi.h"
#ifdef EXC_MODULE_INCLUDED
#include "excMngr.h"
#include "TransmitPowerExc.h"
#include "excRMMngr.h"
#endif

#include "qosMngr_API.h"


/****************************************************************************
								MATRIC ISSUE							
	Each function in the select process returns a MATCH, NO_MATCH value in order to 	
	skip non relevant sites. In addition, some of the functions also measures a matching level of a site.
	The matching level is returned as a bit map. The select function 'OR's those bit maps in order to 
	select the site that has the biggest matching level. If a function returns a NO_MATCH value, the matching level of the
	site is reset.
	Following is the site matching level bit map structure.
	Please notice, that if all the match functions returned MATCH for a site, its matric must be different than 0, 
	because of the rates bits.
	

	    31 - 24           23 - 20           20 - 16             15 - 10       9 - 8         7         6           5         4 - 0
	+---------------+---------------+-----------------------+-------------+------------+----------+---------+-----------+-----------+
	| Rx Level      | Privacy       | Attempts              |Rates        | Modulation |Preamble  | Channel | Spectrum  | Reserved  |
	|		        |	    		|		                | 		      |			   |		  |		    | management|		    |	
	+---------------+---------------+-----------------------+-------------+------------+----------+---------+-----------+-----------+
****************************************************************************/

/* Matric bit map definition */
typedef enum 
{
	/* Rx level */
	METRIC_RX_LEVEL_MASK			= 0xFF,
	METRIC_RX_LEVEL_SHIFT			= 24,

	/* Privacy */
	METRIC_PRIVACY_MASK				= 0x0F,
	METRIC_PRIVACY_SHIFT			= 20,

	/* Number of attempts */
	METRIC_ATTEMPTS_NUMBER_MASK		= 0x0F,
	METRIC_ATTEMPTS_NUMBER_SHIFT	= 16,
	
	
	/* Rates */
	METRIC_RATES_MASK				= 0x3F,
	METRIC_RATES_SHIFT				= 10,

	/* PBCC */
	METRIC_MODULATION_MASK			= 0x03,
	METRIC_MODULATION_SHIFT			= 8,
	
	/* Preamble*/
	METRIC_PREAMBLE_MASK			= 0x01,
	METRIC_PREAMBLE_SHIFT			= 7,

	/* Channel */
	METRIC_CHANNEL_MASK				= 0x01,
	METRIC_CHANNEL_SHIFT			= 6,

	/* Spectrum management Capability */
	METRIC_SPECTRUM_MANAGEMENT_MASK	= 0x01,
	METRIC_SPECTRUM_MANAGEMENT_SHIFT= 5,

	/* Priority Site */
	METRIC_PRIORITY_SITE_MASK		= 0x01,
	METRIC_PRIORITY_SITE_SHIFT		= 4,

} matric_e;

#define MAX_GB_MODE_CHANEL		14

/* RSSI values boundaries and metric values for best, good, etc  signals */
#define SELECT_RSSI_BEST_LEVEL      (-22)
#define SELECT_RSSI_GOOD_LEVEL      (-38)
#define SELECT_RSSI_NORMAL_LEVEL    (-56)
#define SELECT_RSSI_POOR_LEVEL      (-72)
#define SELECT_RSSI_BAD_LEVEL       (-82)


#define  RSSI_METRIC_BEST      6
#define  RSSI_METRIC_GOOD      5
#define  RSSI_METRIC_NORMAL    4
#define  RSSI_METRIC_POOR      3
#define  RSSI_METRIC_BAD       2
#define  RSSI_METRIC_NOSIGNAL  1

/* Local functions prototypes */

static match_e ssidMatchingLevel(siteMgr_t *pSiteMgr, siteEntry_t *pSite);


static match_e bssidMatchingLevel(siteMgr_t *pSiteMgr, siteEntry_t *pSite);

static match_e bssTypeMatchingLevel(siteMgr_t *pSiteMgr, siteEntry_t *pSite);

static match_e ratesMatchingLevel(siteMgr_t *pSiteMgr, siteEntry_t *pSite, UINT32 *matchingLevel);

static match_e modulationTypeMatchingLevel(siteMgr_t *pSiteMgr, siteEntry_t *pSite, UINT32 *matchingLevel);

static match_e preambleTypeMatchingLevel(siteMgr_t *pSiteMgr, siteEntry_t *pSite, UINT32 *matchingLevel);

static match_e channelMatchingLevel(siteMgr_t *pSiteMgr, siteEntry_t *pSite, UINT32 *matchingLevel);

static match_e spectrumManagementMatchingLevel(siteMgr_t *pSiteMgr, UINT16 siteCapability, UINT32 *matchingLevel);

static match_e rxLevelMatchingLevel(siteMgr_t *pSiteMgr, siteEntry_t *pSite, UINT32 *matchingLevel);

static match_e attemptsNumberMatchingLevel(siteMgr_t *pSiteMgr, siteEntry_t *pSite, UINT32 *matchingLevel);

static match_e prioritySiteMatchingLevel(siteMgr_t *pSiteMgr, siteEntry_t *pSite, UINT32 *matchingLevel);

static siteEntry_t *addSelfSite(siteMgr_t *pSiteMgr);

static TI_STATUS sendProbeResponse(siteMgr_t *pSiteMgr, macAddress_t *pBssid);

/* Interface functions Implementation */


/***********************************************************************
 *                        siteMgr_disSelectSite									
 ***********************************************************************
DESCRIPTION: Called by the SME SM in order to dis select the primary site.
			The function set the primary site pointer to NULL and set its type to type regular	
                                                                                                   
INPUT:      hSiteMgr	-	site mgr handle.

OUTPUT:		

RETURN:     OK 

************************************************************************/
TI_STATUS siteMgr_disSelectSite(TI_HANDLE	hSiteMgr)
{
	siteMgr_t	*pSiteMgr = (siteMgr_t *)hSiteMgr;

	/* This protection is because in the case that the BSS was LOST the primary site was removed already. */
	if (pSiteMgr->pSitesMgmtParams->pPrimarySite != NULL)
	{
        WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("siteMgr_disSelectSite REMOVE Primary ssid=%s, bssid= 0x%x-0x%x-0x%x-0x%x-0x%x-0x%x\n\n",
                   pSiteMgr->pSitesMgmtParams->pPrimarySite->ssid.ssidString,
                   pSiteMgr->pSitesMgmtParams->pPrimarySite->bssid.addr[0], pSiteMgr->pSitesMgmtParams->pPrimarySite->bssid.addr[1], pSiteMgr->pSitesMgmtParams->pPrimarySite->bssid.addr[2],
                   pSiteMgr->pSitesMgmtParams->pPrimarySite->bssid.addr[3], pSiteMgr->pSitesMgmtParams->pPrimarySite->bssid.addr[4], pSiteMgr->pSitesMgmtParams->pPrimarySite->bssid.addr[5] ));
		
        pSiteMgr->pSitesMgmtParams->pPrimarySite->siteType = SITE_REGULAR;
		pSiteMgr->pSitesMgmtParams->pPrevPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;
		pSiteMgr->pSitesMgmtParams->pPrimarySite = NULL;
	}

	return OK;
}

/***********************************************************************
 *                        siteMgr_selectSite									
 ***********************************************************************
DESCRIPTION: Site selection function. Called by the SME SM in order to select 
			the best site for teh station. 
			The function go over the site table, and for each site it calls the match functions
			If one fo the functions returns NO_MATCH from some reason, it skips the current site
			and move to the next one.
			The site that has the biggest bit map is chosen.
				-	If a site is chosen, the function calls the 'systemConfig()' function in order 
					to configure the station with the chosen site attribute. Than it reports 
					a select status success to the SME SM.
				-	If no site is chosen & the desired BSS type is infrastructure, it reports 
					a select status failure to the SME SM.
				-	If no site is chosen but the desired BSS type is IBSS, we create a self site by adding an entry
					to the site table, than we configure the system and reports a select status success to the SME SM.

			NOTE: if the reScanFlag is set, means we received a scan command from the utility while in the
					previous scanning, we report a select status failure to the SME SM because a re scan is needed
					and we don't perform a selection.
					
                                                                                                   
INPUT:      hSiteMgr	-	site mgr handle.

OUTPUT:		

RETURN:     OK 

************************************************************************/
siteEntry_t* siteMgr_selectSiteFromTable(TI_HANDLE	hSiteMgr)
{
	UINT32		metric;
	UINT32		prevMatchingLevel = 0;		
	UINT8		siteIndex, tableIndex, numberOfSites = 0;
	siteMgr_t	*pSiteMgr = (siteMgr_t *)hSiteMgr;
	siteEntry_t *pSiteEntry, *pLastMatchSite = NULL;	
	rsnData_t	rsnData;
    dot11_RSN_t *pRsnIe;
    UINT8       rsnIECount=0;
    UINT8       curRsnData[255];
    UINT8       length=0;
	paramInfo_t	param;
    radioBand_e radioBand;
	BOOL		bRegulatoryDomainEnabled;

	siteTablesParams_t* currTable = pSiteMgr->pSitesMgmtParams->pCurrentSiteTable;

	WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  
							("SITE MATCH , Desired ssid (%s) len= (%d)\n\n", 
							 pSiteMgr->pDesiredParams->siteMgrDesiredSSID.ssidString, 
							 pSiteMgr->pDesiredParams->siteMgrDesiredSSID.len));
	

    for (tableIndex = 0; tableIndex < NUM_OF_SITE_TABLE ; tableIndex++)
	{
		/* If regulatoryDomains is enable, it should be checked if Country IE was detected */
		param.paramType = REGULATORY_DOMAIN_ENABLED_PARAM;
		regulatoryDomain_getParam(pSiteMgr->hRegulatoryDomain,&param);
		bRegulatoryDomainEnabled = param.content.regulatoryDomainEnabled;

		if(currTable == &pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables)
			radioBand = RADIO_BAND_2_4_GHZ;
		else
			radioBand = RADIO_BAND_5_0_GHZ;

		/* Check if country code was received */
		param.paramType			 = REGULATORY_DOMAIN_IS_COUNTRY_FOUND;
		param.content.eRadioBand = radioBand;
		regulatoryDomain_getParam(pSiteMgr->hRegulatoryDomain,&param);

		if( ( bRegulatoryDomainEnabled == TRUE) && ( !param.content.bIsCountryFound ) )
		{
			if( radioBand == RADIO_BAND_2_4_GHZ)
				WLAN_REPORT_WARNING(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("There is no valid Country IE for 2.4G band\n"));
			else
				WLAN_REPORT_WARNING(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("There is no valid Country IE for 5G band\n"));

			if(pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode == DOT11_DUAL_MODE)
			{
				/* change site table */
				if(currTable == &pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables)
                {
                    currTable = (siteTablesParams_t *)&pSiteMgr->pSitesMgmtParams->dot11A_sitesTables;
/*                    siteMgr_updateRates(pSiteMgr, TRUE, TRUE);*/
                }
				else
                {
                    currTable = &pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables;
/*                    siteMgr_updateRates(pSiteMgr, FALSE, TRUE);*/
                }
			}
			continue;
		}
				
		for (siteIndex = 0; siteIndex < currTable->maxNumOfSites; siteIndex++)
		{
			pSiteEntry = &(currTable->siteTable[siteIndex]);
			
			length = 0;
			metric = 0;
			pSiteEntry->matchingLevel = 0;


			if(pSiteEntry->siteType == SITE_PRIMARY)
			{
				WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SITE PRIMARY, ssid (%s len= %d ),  %X-%X-%X-%X-%X-%X\n\n", pSiteEntry->ssid.ssidString, pSiteEntry->ssid.len, pSiteEntry->bssid.addr[0], pSiteEntry->bssid.addr[1], pSiteEntry->bssid.addr[2], pSiteEntry->bssid.addr[3], pSiteEntry->bssid.addr[4], pSiteEntry->bssid.addr[5]));
			}

			
			if (pSiteEntry->siteType != SITE_REGULAR)
				continue;
			
			numberOfSites++;
			
			pSiteEntry->ssid.ssidString[pSiteEntry->ssid.len] = 0;
			if (ssidMatchingLevel(pSiteMgr, pSiteEntry) != MATCH)
			{		
				WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SITE MATCH FAILURE, ssid (%s len= %d ),  %X-%X-%X-%X-%X-%X\n\n", pSiteEntry->ssid.ssidString, pSiteEntry->ssid.len, pSiteEntry->bssid.addr[0], pSiteEntry->bssid.addr[1], pSiteEntry->bssid.addr[2], pSiteEntry->bssid.addr[3], pSiteEntry->bssid.addr[4], pSiteEntry->bssid.addr[5]));
				continue;
			}

			if (bssidMatchingLevel(pSiteMgr, pSiteEntry) != MATCH)
			{
				WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SITE MATCH FAILURE, bssid,  %X-%X-%X-%X-%X-%X\n\n", pSiteEntry->bssid.addr[0], pSiteEntry->bssid.addr[1], pSiteEntry->bssid.addr[2], pSiteEntry->bssid.addr[3], pSiteEntry->bssid.addr[4], pSiteEntry->bssid.addr[5]));
				continue;
			}

			if (bssTypeMatchingLevel(pSiteMgr, pSiteEntry) != MATCH)
			{
				WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SITE MATCH FAILURE, bss Type,  %X-%X-%X-%X-%X-%X\n", pSiteEntry->bssid.addr[0], pSiteEntry->bssid.addr[1], pSiteEntry->bssid.addr[2], pSiteEntry->bssid.addr[3], pSiteEntry->bssid.addr[4], pSiteEntry->bssid.addr[5]));
				WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SITE MATCH FAILURE, bss Type,  siteMgrDesiredBSSType=%d, bssType=%d\n",pSiteMgr->pDesiredParams->siteMgrDesiredBSSType, pSiteEntry->bssType));
				continue;
			}
			
			if (ratesMatchingLevel(pSiteMgr, pSiteEntry, &metric) != MATCH)
			{
				pSiteEntry->matchingLevel = 0;
				WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SITE MATCH FAILURE, rates,  %X-%X-%X-%X-%X-%X\n\n", pSiteEntry->bssid.addr[0], pSiteEntry->bssid.addr[1], pSiteEntry->bssid.addr[2], pSiteEntry->bssid.addr[3], pSiteEntry->bssid.addr[4], pSiteEntry->bssid.addr[5]));
				continue;
			} 
			pSiteEntry->matchingLevel |= metric << METRIC_RATES_SHIFT;
			
			
			/* Get the RSN IE data */
			pRsnIe = pSiteEntry->pRsnIe;
            rsnIECount = 0;
			while ((length < pSiteEntry->rsnIeLen) && (rsnIECount < MAX_RSN_IE))
			{
				curRsnData[0+length] = pRsnIe->hdr.eleId;
				curRsnData[1+length] = pRsnIe->hdr.eleLen;
				os_memoryCopy(pSiteMgr->hOs, &curRsnData[2+length], (void *)pRsnIe->rsnIeData, pRsnIe->hdr.eleLen); 
				length += pRsnIe->hdr.eleLen+2;
				pRsnIe += 1;
                rsnIECount++;
			}
            if (length<pSiteEntry->rsnIeLen) 
            {
                WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  
                                  ("siteMgr_selectSiteFromTable, RSN IE is too long: rsnIeLen=%d, MAX_RSN_IE=%d\n",
                                    pSiteEntry->rsnIeLen, MAX_RSN_IE));
			}
			
			rsnData.pIe = (pSiteEntry->rsnIeLen==0) ? NULL :curRsnData;
			rsnData.ieLen = pSiteEntry->rsnIeLen;
			rsnData.privacy = pSiteEntry->privacy;
			
			if (rsn_evalSite(pSiteMgr->hRsn, &rsnData, pSiteEntry->bssType, pSiteEntry->bssid, &metric) != OK)
			{
				pSiteEntry->matchingLevel = 0;
				WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SITE MATCH FAILURE, RSN,  %X-%X-%X-%X-%X-%X\n\n", pSiteEntry->bssid.addr[0], pSiteEntry->bssid.addr[1], pSiteEntry->bssid.addr[2], pSiteEntry->bssid.addr[3], pSiteEntry->bssid.addr[4], pSiteEntry->bssid.addr[5]));
				continue;
			}
			
			pSiteEntry->matchingLevel |= metric << METRIC_PRIVACY_SHIFT;



			
#if 0 /* TODO - Define 4x evaluation */	
			if (ctrlData_evalSite(pSiteMgr->hCtrlData, &pSiteEntry->fourXParams, &metric) != OK)
			{
				pSiteEntry->matchingLevel = 0;
				WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SITE MATCH FAILURE, RSN,  %X-%X-%X-%X-%X-%X\n\n", pSiteEntry->bssid.addr[0], pSiteEntry->bssid.addr[1], pSiteEntry->bssid.addr[2], pSiteEntry->bssid.addr[3], pSiteEntry->bssid.addr[4], pSiteEntry->bssid.addr[5]));
				continue;
			}
			pSiteEntry->matchingLevel |= metric << METRIC_4X_SHIFT;
#endif	
			
			if (modulationTypeMatchingLevel(pSiteMgr, pSiteEntry, &metric) != MATCH)
			{
				pSiteEntry->matchingLevel = 0;
				WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SITE MATCH FAILURE, modulation Type,  %X-%X-%X-%X-%X-%X\n\n", pSiteEntry->bssid.addr[0], pSiteEntry->bssid.addr[1], pSiteEntry->bssid.addr[2], pSiteEntry->bssid.addr[3], pSiteEntry->bssid.addr[4], pSiteEntry->bssid.addr[5]));
				continue;
			}
			pSiteEntry->matchingLevel |= metric << METRIC_MODULATION_SHIFT;
			
			if (preambleTypeMatchingLevel(pSiteMgr, pSiteEntry, &metric) != MATCH)
			{
				pSiteEntry->matchingLevel = 0;
				WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SITE MATCH FAILURE, preamble,  %X-%X-%X-%X-%X-%X\n\n", pSiteEntry->bssid.addr[0], pSiteEntry->bssid.addr[1], pSiteEntry->bssid.addr[2], pSiteEntry->bssid.addr[3], pSiteEntry->bssid.addr[4], pSiteEntry->bssid.addr[5]));
				continue;
			}
			pSiteEntry->matchingLevel |= metric << METRIC_PREAMBLE_SHIFT;
			
			if (channelMatchingLevel(pSiteMgr, pSiteEntry, &metric) != MATCH)
			{
				pSiteEntry->matchingLevel = 0;
				WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SITE MATCH FAILURE, channel,  %X-%X-%X-%X-%X-%X\n\n", pSiteEntry->bssid.addr[0], pSiteEntry->bssid.addr[1], pSiteEntry->bssid.addr[2], pSiteEntry->bssid.addr[3], pSiteEntry->bssid.addr[4], pSiteEntry->bssid.addr[5]));
				continue;
			}
			pSiteEntry->matchingLevel |= metric << METRIC_CHANNEL_SHIFT;
			
			if (spectrumManagementMatchingLevel(pSiteMgr,pSiteEntry->capabilities,&metric) != MATCH)
			{
				pSiteEntry->matchingLevel = 0;
				WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SITE MATCH FAILURE, spectrum management.\n\n"));
				continue;
			}
			pSiteEntry->matchingLevel |= metric << METRIC_SPECTRUM_MANAGEMENT_SHIFT;
			
			if (rxLevelMatchingLevel(pSiteMgr, pSiteEntry, &metric) != MATCH)
			{
				pSiteEntry->matchingLevel = 0;
				WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SITE MATCH FAILURE, Rx level,  %X-%X-%X-%X-%X-%X\n\n", pSiteEntry->bssid.addr[0], pSiteEntry->bssid.addr[1], pSiteEntry->bssid.addr[2], pSiteEntry->bssid.addr[3], pSiteEntry->bssid.addr[4], pSiteEntry->bssid.addr[5]));
				continue; 
			}
			pSiteEntry->matchingLevel |= metric << METRIC_RX_LEVEL_SHIFT;
			
			if (attemptsNumberMatchingLevel(pSiteMgr, pSiteEntry, &metric) != MATCH)
			{
				pSiteEntry->matchingLevel = 0;
				WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SITE MATCH FAILURE, Number of attempts,  %X-%X-%X-%X-%X-%X\n\n", pSiteEntry->bssid.addr[0], pSiteEntry->bssid.addr[1], pSiteEntry->bssid.addr[2], pSiteEntry->bssid.addr[3], pSiteEntry->bssid.addr[4], pSiteEntry->bssid.addr[5]));
				continue;
			}
			pSiteEntry->matchingLevel |= metric << METRIC_ATTEMPTS_NUMBER_SHIFT;

			if (prioritySiteMatchingLevel(pSiteMgr, pSiteEntry, &metric) != MATCH)
			{
				pSiteEntry->matchingLevel = 0;
				WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SITE MATCH FAILURE, priority Site,  %X-%X-%X-%X-%X-%X\n\n", pSiteEntry->bssid.addr[0], pSiteEntry->bssid.addr[1], pSiteEntry->bssid.addr[2], pSiteEntry->bssid.addr[3], pSiteEntry->bssid.addr[4], pSiteEntry->bssid.addr[5]));
				continue;
			}
			pSiteEntry->matchingLevel |= metric << METRIC_PRIORITY_SITE_SHIFT;
			
			if(pSiteEntry->matchingLevel > prevMatchingLevel)
			{
				prevMatchingLevel = pSiteEntry->matchingLevel;
				pLastMatchSite = pSiteEntry;
			}
		}
		if(pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode == DOT11_DUAL_MODE)
		{
            /* change site table */
            if(currTable == &pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables)
            {
                currTable = (siteTablesParams_t *)&pSiteMgr->pSitesMgmtParams->dot11A_sitesTables;
/*                siteMgr_updateRates(pSiteMgr, TRUE, TRUE);*/
            }
            else
            {
                currTable = &pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables;
 /*               siteMgr_updateRates(pSiteMgr, FALSE, TRUE);*/
            }
		}
		else
			break;
		
	}
	WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("				NUMBER OF SITES:	%d\n\n", numberOfSites));
	
	return pLastMatchSite;
}

TI_STATUS siteMgr_selectSite(TI_HANDLE	hSiteMgr)
{
	radioBand_e radioBand;
	paramInfo_t	param;
	siteMgr_t	*pSiteMgr = (siteMgr_t *)hSiteMgr;

	siteEntry_t* pLastMatchSite = siteMgr_selectSiteFromTable(hSiteMgr);

	if (pLastMatchSite != NULL)
	{
		pSiteMgr->pSitesMgmtParams->pPrimarySite = pLastMatchSite;
		pLastMatchSite->siteType = SITE_PRIMARY;	
		
		WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SELECT SUCCESS FROM TABLE, bssid: %X-%X-%X-%X-%X-%X\n\n", pLastMatchSite->bssid.addr[0], pLastMatchSite->bssid.addr[1], pLastMatchSite->bssid.addr[2], pLastMatchSite->bssid.addr[3], pLastMatchSite->bssid.addr[4], pLastMatchSite->bssid.addr[5]));		
		

		/***************** Config Connection *************************/
		param.paramType = CONN_TYPE_PARAM;	
		if (pLastMatchSite->bssType == BSS_INDEPENDENT)
			param.content.connType = CONNECTION_IBSS;
		else
			param.content.connType = CONNECTION_INFRA;
		conn_setParam(pSiteMgr->hConn, &param);

		return smeSm_reportSelectStatus(pSiteMgr->hSmeSm, (mgmtStatus_e)SELECT_STATUS_SUCCESS);
	}

	if ((pSiteMgr->pDesiredParams->siteMgrDesiredBSSType == BSS_ANY) || 
		(pSiteMgr->pDesiredParams->siteMgrDesiredBSSType == BSS_INDEPENDENT))	/* Means we can start our own BSS */
	{
		
		if (pSiteMgr->pDesiredParams->siteMgrDesiredChannel >= SITE_MGR_CHANNEL_A_MIN)
		{
			radioBand = RADIO_BAND_5_0_GHZ;
		} 
		else {
			radioBand = RADIO_BAND_2_4_GHZ;
		}

        /*
        update the regulatory domain with the selected band
        */
		/* Check if the selected channel is valid according to regDomain */
		param.paramType = REGULATORY_DOMAIN_GET_SCAN_CAPABILITIES;
		param.content.channelCapabilityReq.band = radioBand;
		param.content.channelCapabilityReq.scanOption = ACTIVE_SCANNING;
		param.content.channelCapabilityReq.channelNum = pSiteMgr->pDesiredParams->siteMgrDesiredChannel;
		regulatoryDomain_getParam(pSiteMgr->hRegulatoryDomain,&param);
		if (!param.content.channelCapabilityRet.channelValidity)
		{
			WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("IBSS SELECT FAILURE  - No channel !!!\n\n"));
			return smeSm_reportSelectStatus(pSiteMgr->hSmeSm, (mgmtStatus_e)SELECT_STATUS_FAILURE);
		}

		pLastMatchSite = addSelfSite(pSiteMgr);
		if (pLastMatchSite == NULL)
		{
			WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("IBSS SELECT FAILURE  - could not open self site !!!\n\n"));
			return smeSm_reportSelectStatus(pSiteMgr->hSmeSm, (mgmtStatus_e)SELECT_STATUS_FAILURE);
		}
		
		pSiteMgr->pSitesMgmtParams->pPrimarySite = pLastMatchSite;
		pLastMatchSite->siteType = SITE_SELF;
	
		WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("%%%%%%%%%%%%%%	SELF SELECT SUCCESS, bssid: %X-%X-%X-%X-%X-%X	%%%%%%%%%%%%%%\n\n", pLastMatchSite->bssid.addr[0], pLastMatchSite->bssid.addr[1], pLastMatchSite->bssid.addr[2], pLastMatchSite->bssid.addr[3], pLastMatchSite->bssid.addr[4], pLastMatchSite->bssid.addr[5]));

		/***************** Config Connection *************************/

		param.paramType = CONN_TYPE_PARAM;											/* Connection Type*/
		param.content.connType = CONNECTION_SELF;
		conn_setParam(pSiteMgr->hConn, &param);

		return smeSm_reportSelectStatus(pSiteMgr->hSmeSm, (mgmtStatus_e)SELECT_STATUS_SUCCESS);
	}
	
	WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SELECT FAILURE \n\n"));

	return smeSm_reportSelectStatus(pSiteMgr->hSmeSm, (mgmtStatus_e)SELECT_STATUS_FAILURE);
}


void siteMgr_setNotReceivedParameter(TI_HANDLE	hSiteMgr, ssid_t* ssid , radioBand_e band)
{
	UINT8		siteIndex;
	siteEntry_t *pSiteEntry;
	siteMgr_t	*pSiteMgr = (siteMgr_t *)hSiteMgr;
	siteTablesParams_t* currTable=NULL;


	/*
	 * Set the propiate site table.
	 */
	switch (band) {
	case  RADIO_BAND_2_4_GHZ  :
		currTable = &pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables;
		break;

	case RADIO_BAND_5_0_GHZ :
		currTable = (siteTablesParams_t*)&pSiteMgr->pSitesMgmtParams->dot11A_sitesTables;
		break;

	case RADIO_BAND_DUAL:
	default:
		WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("siteMgr_setNotReceivedParameter() invalid band.\n"));

	}


	/*
	 *  Increase the scanned sites "Not_Received" counter, this counter is used by the 
	 *  aging logic to clean old sites.
	 */
	for (siteIndex = 0; siteIndex < currTable->maxNumOfSites; siteIndex++)
	{
		pSiteEntry = &(currTable->siteTable[siteIndex]);

		/* Self site & null site are never aged out. */
		if ((pSiteEntry->siteType == SITE_SELF) || (pSiteEntry->siteType == SITE_NULL) || (pSiteEntry->siteType == SITE_PRIMARY))
			continue;
		
		/* If scan for any ssid all sites are expected to be found, and all should be marked. */
		if( utils_isAnySSID(ssid) )
		{
			pSiteEntry->Not_Received++;
		}
		/* otherwise, the scan was a unicast scan, and thus only sites that match the desired SSID are marked*/
		else 
		{
            if(os_memoryCompare(pSiteMgr->hOs, (UINT8 *)ssid->ssidString, (UINT8 *)pSiteEntry->ssid.ssidString, pSiteEntry->ssid.len) == 0)
			{
				pSiteEntry->Not_Received++;
			}
		}   
	} /* for (... all sites in table ) */
}



void siteMgr_resetAttemptsNumberParameter(TI_HANDLE	hSiteMgr)
{
	UINT8		siteIndex, tableIndex;
	siteEntry_t *pSiteEntry;
	siteMgr_t	*pSiteMgr = (siteMgr_t *)hSiteMgr;

	siteTablesParams_t* currTable = pSiteMgr->pSitesMgmtParams->pCurrentSiteTable;
	
	for (tableIndex = 0; tableIndex < NUM_OF_SITE_TABLE ; tableIndex++)
	{
		for (siteIndex = 0; siteIndex < currTable->maxNumOfSites; siteIndex++)
		{
			pSiteEntry = &(currTable->siteTable[siteIndex]);

			/* Self site & null site are never aged out. */
			if ((pSiteEntry->siteType == SITE_SELF) || (pSiteEntry->siteType == SITE_NULL))
				continue;
			
			pSiteEntry->attemptsNumber = 0;
		}
		if(pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode == DOT11_DUAL_MODE)
		{
			/* change site table */
			if(currTable == &pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables)
				currTable = (siteTablesParams_t *)&pSiteMgr->pSitesMgmtParams->dot11A_sitesTables;
			else
				currTable = &pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables;
		}
		else
			break;

	}
}

/* Local functions Implementation */

/************************************************************************************************************/
/*		Each functions of the following measures the matching level of a site for the specific attribute, 
		for example: BSSId, SSID, etc...
		The input is the site manager handle which contains the desired attributes, and a pointer 
		to the site in the site table.
		The function returns NO_MATCH if it is not possible to work with this site and MATCH otherwise.
		Some of the functions, in a case of a MATCH, compute also the site matching level and returns it too.
		This used later in the function 'siteMgr_selectSite()'.
	*/
/************************************************************************************************************/

static match_e ssidMatchingLevel(siteMgr_t *pSiteMgr, siteEntry_t *pSite)
{
	if (pSiteMgr->pDesiredParams->siteMgrDesiredSSID.len == 0)
	{	/* match any site that is not hidden */
        if (pSite->ssid.ssidString[0]=='\0') /* hidden ssid */
        {
            WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("Null SSID and hidden ssid \n\n"));
            return NO_MATCH;
        }
        else
        { 
            WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("Null SSID and not hidden ssid \n\n"));
            return MATCH;
        }
    }
	if ((pSiteMgr->pDesiredParams->siteMgrDesiredSSID.len == pSite->ssid.len) && 
            (os_memoryCompare(pSiteMgr->hOs, (UINT8 *)pSiteMgr->pDesiredParams->siteMgrDesiredSSID.ssidString, (UINT8 *)pSite->ssid.ssidString, pSite->ssid.len) == 0))
		return MATCH;

	else
		return NO_MATCH;
}


static match_e bssidMatchingLevel(siteMgr_t *pSiteMgr, siteEntry_t *pSite)
{
	/* If BSSID is NULL, return NO_MATCH */
	if MAC_NULL((&(pSite->bssid)))
		return NO_MATCH;

	if MAC_BROADCAST((&(pSiteMgr->pDesiredParams->siteMgrDesiredBSSID)))
		return MATCH;

	if MAC_EQUAL((&(pSite->bssid)), (&(pSiteMgr->pDesiredParams->siteMgrDesiredBSSID)))
		return MATCH;

	return NO_MATCH;
}

static match_e bssTypeMatchingLevel(siteMgr_t *pSiteMgr, siteEntry_t *pSite)
{
	if (pSiteMgr->pDesiredParams->siteMgrDesiredBSSType == BSS_ANY)
		return MATCH;

	if (pSiteMgr->pDesiredParams->siteMgrDesiredBSSType == pSite->bssType)
		return MATCH;

	return NO_MATCH;
}

static match_e ratesMatchingLevel(siteMgr_t *pSiteMgr, siteEntry_t *pSite, UINT32 *matchingLevel)
{
    UINT32 MatchedBasicRateMask;
    UINT32 MatchedSupportedRateMask;
    UINT32 MatchedMaxBasicRate;
    UINT32 MatchedMaxActiveRate;
	UINT32 StaTotalRates;
	UINT32 SiteTotalRates;

	/* If the basic or active rate are invalid (0), return NO_MATCH. */
	if ((pSite->maxBasicRate == DRV_RATE_INVALID) || (pSite->maxActiveRate == DRV_RATE_INVALID)) 
	{
		WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SITE MATCH FAILURE, 1.maxBasic=%d,maxActive=%d \n", pSite->maxBasicRate,pSite->maxActiveRate));
		return NO_MATCH;
	}
	
	if (DRV_RATE_MAX < pSite->maxBasicRate)
	{
		WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SITE MATCH FAILURE, 1.maxBasic=%d,maxActive=%d \n", pSite->maxBasicRate,pSite->maxActiveRate));
		return NO_MATCH;
	}

	if(pSite->channel <= 14)
		siteMgr_updateRates(pSiteMgr, FALSE, TRUE);
	else
		siteMgr_updateRates(pSiteMgr, TRUE, TRUE);

	StaTotalRates = pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.basicRateMask
				| pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.supportedRateMask;

	SiteTotalRates = pSite->rateMask.basicRateMask | pSite->rateMask.supportedRateMask;
    
	MatchedBasicRateMask = SiteTotalRates 
				& pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.basicRateMask;
                                                            

    MatchedSupportedRateMask = SiteTotalRates &
				pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.supportedRateMask;

    if ((StaTotalRates & pSite->rateMask.basicRateMask) != pSite->rateMask.basicRateMask)
    {
        WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SITE MATCH FAILURE, Basic or Supported Rates Doesn't Match \n"));
        return NO_MATCH;

    }

    
    MatchedMaxBasicRate = getMaxRatefromBitmap(MatchedBasicRateMask);
    MatchedMaxActiveRate = getMaxRatefromBitmap(MatchedSupportedRateMask);

    MatchedMaxActiveRate = MAX(MatchedMaxBasicRate,MatchedMaxActiveRate);

	*matchingLevel = MatchedMaxActiveRate;
	return MATCH;
}

static match_e modulationTypeMatchingLevel(siteMgr_t *pSiteMgr, siteEntry_t *pSite, UINT32 *matchingLevel)
{
	/* If the desired modulation is CCK, we set the modulation bits off. */
	if (pSiteMgr->pDesiredParams->siteMgrDesiredModulationType == DRV_MODULATION_CCK)
	{
		*matchingLevel = 0;
		return MATCH;
	}

	/* Now, the desired modulation is PBCC */
	if (pSite->beaconModulation == DRV_MODULATION_PBCC)		/* if the site current modulation is PBCC */
	{	 
		*matchingLevel = 2;
		return MATCH;
	}
	
	if (pSite->probeModulation == DRV_MODULATION_PBCC)		/* if the site potential modulation is PBCC */
	{	 
		*matchingLevel = 1;
		return MATCH;
	}
	else												/* the current modulation is CCK */
	{ 
		*matchingLevel = 0;
		return MATCH;
	}
}

static match_e preambleTypeMatchingLevel(siteMgr_t *pSiteMgr, siteEntry_t *pSite, UINT32 *matchingLevel)
{
	/* NOw, the chip preamble is SHORT */
	/* If the desired preamble is LONG, we set the preamble bits off. */
	if (pSiteMgr->pDesiredParams->siteMgrDesiredPreambleType == PREAMBLE_LONG)
	{
		*matchingLevel = 0;
		return MATCH;
	}

	/* Now, the desired preamble is SHORT */
	if (pSite->currentPreambleType == PREAMBLE_SHORT)	/* site preamble is SHORT */
	{	 
		*matchingLevel = 1;
		return MATCH;
	}
	else										/* site preamble is LONG */
	{ 
		*matchingLevel = 0;
		return MATCH;
	}
}

static match_e channelMatchingLevel(siteMgr_t *pSiteMgr, siteEntry_t *pSite, UINT32 *matchingLevel)
{
	paramInfo_t     tParam;
	/*	when 802.11d is enabled, 
	channels that are not valid for Active, will not be mach.*/
	tParam.content.channel = pSite->channel;
	tParam.paramType = REGULATORY_DOMAIN_IS_CHANNEL_SUPPORTED;
	regulatoryDomain_getParam(pSiteMgr->hRegulatoryDomain, &tParam);
	if ( !tParam.content.bIsChannelSupprted )
	{
        WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
			("channelMatchingLevel: channel =%d isn't valid for Active and will not be matched.\n",
			pSite->channel));
		return NO_MATCH;
	}
	
	/* If the site channel is equal to the desired channel, we set the channel bit on. */
	if (pSite->channel == pSiteMgr->pDesiredParams->siteMgrDesiredChannel)
	{
		*matchingLevel = 1;
		return MATCH;
	}
	else										
	{ 
		*matchingLevel = 0;
		return MATCH;
	}
/*	return NO_MATCH; - unreachable*/

}

static match_e rxLevelMatchingLevel(siteMgr_t *pSiteMgr, siteEntry_t *pSite, UINT32 *matchingLevel)
{

	if(pSite->rssi > SELECT_RSSI_BEST_LEVEL)
		*matchingLevel = (UINT8)RSSI_METRIC_BEST;
	else if (pSite->rssi > SELECT_RSSI_GOOD_LEVEL)
		*matchingLevel = (UINT8)RSSI_METRIC_GOOD;
	else if(pSite->rssi > SELECT_RSSI_NORMAL_LEVEL)
		*matchingLevel = (UINT8)RSSI_METRIC_NORMAL;
	else if (pSite->rssi > SELECT_RSSI_POOR_LEVEL)
		*matchingLevel = (UINT8)RSSI_METRIC_POOR;
	else if(pSite->rssi > SELECT_RSSI_BAD_LEVEL)
		*matchingLevel = (UINT8)RSSI_METRIC_BAD;
	else 
		*matchingLevel = (UINT8)RSSI_METRIC_NOSIGNAL;

	return MATCH;

}

static match_e attemptsNumberMatchingLevel(siteMgr_t *pSiteMgr, siteEntry_t *pSite, UINT32 *matchingLevel)
{
	if(pSite->attemptsNumber > 0)
	{
		return NO_MATCH;
	}
	else
	{
		*matchingLevel = 0x0F - pSite->attemptsNumber;
		return MATCH;
	}
}

static match_e spectrumManagementMatchingLevel(siteMgr_t *pSiteMgr, UINT16 siteCapability, UINT32 *matchingLevel)
{
	paramInfo_t		param;
	TI_STATUS		status;
	/* If the site has spectrum management capabilty and the station 
	   spectrumManagementCapabilty is enabled, we set the spectrum management bit on. */
	param.paramType = REGULATORY_DOMAIN_MANAGEMENT_CAPABILITY_ENABLED_PARAM;
	status = regulatoryDomain_getParam(pSiteMgr->hRegulatoryDomain,&param);
	if(status == OK && param.content.spectrumManagementEnabled)
	{
		if (siteCapability & DOT11_SPECTRUM_MANAGEMENT)
			*matchingLevel = 1;
		else										
			*matchingLevel = 0;
	}

	return MATCH;
}

static match_e prioritySiteMatchingLevel(siteMgr_t *pSiteMgr, siteEntry_t *pSite, UINT32 *matchingLevel)
{
	/* If the site channel is equal to the desired channel, we set the channel bit on. */
	if (pSite->prioritySite)
	{
		*matchingLevel = 1;
		return MATCH;
	}
	else										
	{ 
		*matchingLevel = 0;
		return MATCH;
	}
/*	return NO_MATCH; - unreachable */
}

/***********************************************************************
 *                        addSelfSite									
 ***********************************************************************
DESCRIPTION: This function is called if the selection fails and desired BSS type is IBSS
			That means we creating our own network and wait for other stations to join us.
			the best site for teh station. 
			Performs the following:
				-	If the desired BSSID is broadcast, we generate a random BSSId, otherwise we use the desired one.
				-	If the site table is full we remove the most old site
				-	We send a probe response with our oiwn desired attributes in order to add the site to the site table
                                                                                                   
INPUT:      pSiteMgr	-	site mgr handle.

OUTPUT:		

RETURN:     Pointer to rthe self site entry in the site table

************************************************************************/
siteEntry_t *addSelfSite(siteMgr_t *pSiteMgr)
{
	siteEntry_t			*pSite;
	macAddress_t		bssid; 

	
	if (utils_isJunkSSID(&(pSiteMgr->pDesiredParams->siteMgrDesiredSSID)) == TRUE)
		return NULL ;

	if MAC_BROADCAST((&(pSiteMgr->pDesiredParams->siteMgrDesiredBSSID)))
	{
		os_memoryCopy(pSiteMgr->hOs, (void *)bssid.addr, (void *)pSiteMgr->ibssBssid.addr, sizeof(macAddress_t));
	}
	else
	{
		os_memoryCopy(pSiteMgr->hOs, (void *)bssid.addr, (void *)pSiteMgr->pDesiredParams->siteMgrDesiredBSSID.addr, sizeof(macAddress_t));  
	}

	if(pSiteMgr->pDesiredParams->siteMgrDesiredChannel <= 14)
	{
		pSiteMgr->pSitesMgmtParams->pCurrentSiteTable = &pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables;
        pSiteMgr->siteMgrOperationalMode = DOT11_G_MODE;
	}
	else
	{
		pSiteMgr->pSitesMgmtParams->pCurrentSiteTable = (siteTablesParams_t *)&pSiteMgr->pSitesMgmtParams->dot11A_sitesTables;
		pSiteMgr->siteMgrOperationalMode = DOT11_A_MODE;
	}

    siteMgr_ConfigRate(pSiteMgr);

	/* First make sure that there is a place in the site table, if not, reomve the eldest site. */
	if (pSiteMgr->pSitesMgmtParams->pCurrentSiteTable->numOfSites == pSiteMgr->pSitesMgmtParams->pCurrentSiteTable->maxNumOfSites)
		removeEldestSite(pSiteMgr);

	sendProbeResponse(pSiteMgr, &bssid);

	/* Now find the site in the site table. */
	pSite = findSiteEntry(pSiteMgr, &bssid);
	if (pSite == NULL)
	{
		return NULL;
	}
	pSite->beaconModulation = pSite->probeModulation;
	pSite->barkerPreambleType = PREAMBLE_UNSPECIFIED;

	return pSite;
	
}

/***********************************************************************
 *                        sendProbeResponse									
 ***********************************************************************
DESCRIPTION: This function is called by the function 'addSelfSite()' in order to send a probe response
			to the site mgr. This will cause the site manager to add a new entry to the site table, the self site entry.

INPUT:      pSiteMgr	-	site mgr handle.
			pBssid		-	Received BSSID

OUTPUT:		

RETURN:     OK

************************************************************************/
static TI_STATUS sendProbeResponse(siteMgr_t *pSiteMgr, macAddress_t *pBssid)
{
	mlmeFrameInfo_t		frame;
	paramInfo_t			param;
	dot11_SSID_t 		ssid;	   
	dot11_RATES_t 		rates;	   
	dot11_FH_PARAMS_t 	FHParamsSet;	   
	dot11_DS_PARAMS_t 	DSParamsSet;	   
	dot11_CF_PARAMS_t 	CFParamsSet;	   
	dot11_IBSS_PARAMS_t IBSSParamsSet;
	UINT32				len = 0, ofdmIndex = 0;
    radioBand_e         band;
	dot11_RATES_t 		extRates;
	UINT8				ratesBuf[MAX_SUPPORTED_RATES];	
	BOOL				extRatesInd = FALSE;
	
	/* The easiest way to add a site to the site table is to simulate a probe frame. */
	frame.subType = PROBE_RESPONSE;
	os_memoryZero(pSiteMgr->hOs, &frame, sizeof(mlmeFrameInfo_t));
		/* Initialize the frame fields */
	frame.subType = PROBE_RESPONSE;
	os_memoryZero(pSiteMgr->hOs, (void *)frame.content.iePacket.timestamp, TIME_STAMP_LEN);

	/* Build  Beacon interval  */
	frame.content.iePacket.beaconInerval = pSiteMgr->pDesiredParams->siteMgrDesiredBeaconInterval;

	/* Build  capability field */
	frame.content.iePacket.capabilities = 0;
	frame.content.iePacket.capabilities |= (TRUE << CAP_IBSS_SHIFT); /* Bss type must be independent */

	if ((pSiteMgr->pDesiredParams->siteMgrDesiredPreambleType == PREAMBLE_SHORT))
		frame.content.iePacket.capabilities |= (TRUE << CAP_PREAMBLE_SHIFT);

	/* call RSN to get the privacy desired */
	param.paramType = RSN_ENCRYPTION_STATUS_PARAM;
	rsn_getParam(pSiteMgr->hRsn, &param);
	if (param.content.rsnEncryptionStatus == RSN_CIPHER_NONE)
	{
		frame.content.iePacket.capabilities |= (FALSE << CAP_PRIVACY_SHIFT);
	} else {
		frame.content.iePacket.capabilities |= (TRUE << CAP_PRIVACY_SHIFT);
	}
	
	if (pSiteMgr->pDesiredParams->siteMgrDesiredModulationType == DRV_MODULATION_PBCC)
		frame.content.iePacket.capabilities |= (TRUE << CAP_PBCC_SHIFT);
	
    if (pSiteMgr->siteMgrOperationalMode == DOT11_G_MODE)
    {
        if(pSiteMgr->pDesiredParams->siteMgrDesiredSlotTime == PHY_SLOT_TIME_SHORT)
            frame.content.iePacket.capabilities |= (TRUE << CAP_SLOT_TIME_SHIFT);
    }
	
	/* Build ssid */
	os_memoryZero(pSiteMgr->hOs, (void *)ssid.serviceSetId, MAX_SSID_LEN);

	if (pSiteMgr->pDesiredParams->siteMgrDesiredSSID.len == 0)
		ssid.hdr.eleLen = 0;
	else
	{
		os_memoryCopy(pSiteMgr->hOs, (void *)ssid.serviceSetId, (void *)pSiteMgr->pDesiredParams->siteMgrDesiredSSID.ssidString, pSiteMgr->pDesiredParams->siteMgrDesiredSSID.len);
		ssid.hdr.eleLen = pSiteMgr->pDesiredParams->siteMgrDesiredSSID.len;
	}
	
	if(pSiteMgr->pDesiredParams->siteMgrDesiredChannel <= MAX_GB_MODE_CHANEL)
		siteMgr_updateRates(pSiteMgr, FALSE, TRUE);
	else
		siteMgr_updateRates(pSiteMgr, TRUE, TRUE);

	/* Build Rates */
	bitMapToNetworkStringRates(pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.supportedRateMask,
							   pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.basicRateMask,
							   ratesBuf, &len, &ofdmIndex);

	if(pSiteMgr->siteMgrOperationalMode != DOT11_G_MODE ||
       pSiteMgr->pDesiredParams->siteMgrUseDraftNum == DRAFT_5_AND_EARLIER ||
	   ofdmIndex == len)
	{
		rates.hdr.eleId = DOT11_SUPPORTED_RATES_ELE_ID;
		rates.hdr.eleLen = len;
		os_memoryCopy(pSiteMgr->hOs, (void *)rates.rates, ratesBuf, rates.hdr.eleLen);
	}
	else
	{
		rates.hdr.eleId = DOT11_SUPPORTED_RATES_ELE_ID;
		rates.hdr.eleLen = ofdmIndex;
		os_memoryCopy(pSiteMgr->hOs, (void *)rates.rates, ratesBuf, rates.hdr.eleLen);

		extRates.hdr.eleId = DOT11_EXT_SUPPORTED_RATES_ELE_ID;
		extRates.hdr.eleLen = len - ofdmIndex;
		os_memoryCopy(pSiteMgr->hOs, (void *)extRates.rates, &ratesBuf[ofdmIndex], extRates.hdr.eleLen);
		extRatesInd = TRUE;
	}


	/* Build FH */
	os_memoryZero(pSiteMgr->hOs, &FHParamsSet, sizeof(dot11_FH_PARAMS_t));

	/* Build DS */
	DSParamsSet.hdr.eleLen = 1;
	DSParamsSet.currChannel = pSiteMgr->pDesiredParams->siteMgrDesiredChannel;

	/* Build CF */
	os_memoryZero(pSiteMgr->hOs, &CFParamsSet, sizeof(dot11_CF_PARAMS_t));

	/* Build IBSS */
	os_memoryZero(pSiteMgr->hOs, &IBSSParamsSet, sizeof(dot11_IBSS_PARAMS_t));
	IBSSParamsSet.hdr.eleLen = 2;
	IBSSParamsSet.atimWindow = pSiteMgr->pDesiredParams->siteMgrDesiredAtimWindow;

	frame.content.iePacket.pSsid = &ssid;
	frame.content.iePacket.pRates = &rates;

	if(extRatesInd)
		frame.content.iePacket.pExtRates = &extRates;
	else
		frame.content.iePacket.pExtRates = NULL;

	frame.content.iePacket.pFHParamsSet = &FHParamsSet;
	frame.content.iePacket.pDSParamsSet = &DSParamsSet;
	frame.content.iePacket.pCFParamsSet = &CFParamsSet;
	frame.content.iePacket.pIBSSParamsSet = &IBSSParamsSet;

    band = ( MAX_GB_MODE_CHANEL >= pSiteMgr->pDesiredParams->siteMgrDesiredChannel ? RADIO_BAND_2_4_GHZ : RADIO_BAND_5_0_GHZ );
	/* Update site */
	siteMgr_updateSite(pSiteMgr, pBssid, &frame ,pSiteMgr->pDesiredParams->siteMgrDesiredChannel, band, FALSE);
	
	return OK;
}

/***********************************************************************
 *                        systemConfig									
 ***********************************************************************
DESCRIPTION: This function is called by the function 'siteMgr_selectSite()' in order to configure
			the system with the chosen site attribute.

INPUT:      pSiteMgr	-	site mgr handle.

OUTPUT:		

RETURN:     OK

************************************************************************/
TI_STATUS systemConfig(siteMgr_t *pSiteMgr)
{ 
	paramInfo_t param;
	siteEntry_t *pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;
	rsnData_t	rsnData;
	UINT8		rsnAssocIeLen;
    dot11_RSN_t *pRsnIe;
    UINT8       rsnIECount=0;
    UINT8       curRsnData[255];
    UINT16      length;
    UINT16      capabilities;
    UINT16      PktLength=0;
    UINT8		*pIeBuffer=NULL;

#ifdef EXC_MODULE_INCLUDED
    UINT8        ExternTxPower;
#endif
	TI_STATUS	status;
	slotTime_e	slotTime;
	UINT32		StaTotalRates;
	ACParameters_t *p_ACParametersDummy = NULL;


	if (pPrimarySite->probeRecv)
	{
		pIeBuffer = pPrimarySite->probeRespBuffer;
		PktLength = pPrimarySite->probeRespLength;
	}
    else if (pPrimarySite->beaconRecv)
	{
		pIeBuffer = pPrimarySite->beaconBuffer;
		PktLength = pPrimarySite->beaconLength;
	}
		
	pSiteMgr->prevRadioBand = pSiteMgr->radioBand;
	
	WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
		("%s: Capabilities, Slot Time Bit = %d (capabilities = %d)\n", __FUNCTION__, (pPrimarySite->capabilities >> CAP_SLOT_TIME_SHIFT) & 1, pPrimarySite->capabilities));
	
	if(pPrimarySite->channel <= MAX_GB_MODE_CHANEL)
	{
		if(pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode == DOT11_B_MODE)
		{
			pSiteMgr->siteMgrOperationalMode = DOT11_B_MODE;
			slotTime = PHY_SLOT_TIME_LONG;

			WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
				("%s: 11b Mode, Slot Time = %d\n", __FUNCTION__, (UINT8)slotTime));
		}
		else
		{
			pSiteMgr->siteMgrOperationalMode = DOT11_G_MODE;

			if (((pPrimarySite->capabilities >> CAP_SLOT_TIME_SHIFT) & CAP_SLOT_TIME_MASK) == PHY_SLOT_TIME_SHORT)
			{
			slotTime = pSiteMgr->pDesiredParams->siteMgrDesiredSlotTime;

				WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
					("%s: 11g Mode, Slot Time = %d (desired)\n", __FUNCTION__, (UINT8)slotTime));
			}
			else
			{
				slotTime = PHY_SLOT_TIME_LONG;

				WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
					("%s: 11g Mode, Slot Time = %d\n", __FUNCTION__, (UINT8) slotTime));
			}
		}

		pSiteMgr->radioBand = RADIO_BAND_2_4_GHZ;
		pSiteMgr->pSitesMgmtParams->pCurrentSiteTable = &pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables;
	}
	else
	{
		pSiteMgr->siteMgrOperationalMode = DOT11_A_MODE;
		pSiteMgr->radioBand = RADIO_BAND_5_0_GHZ;
		slotTime = PHY_SLOT_TIME_SHORT;

		WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
			("%s: 11a Mode, Slot Time = %d\n", __FUNCTION__, (UINT8)slotTime));

		pSiteMgr->pSitesMgmtParams->pCurrentSiteTable = (siteTablesParams_t *)&pSiteMgr->pSitesMgmtParams->dot11A_sitesTables;
	}

	/* since we are moving to the different band, the siteMgr should be reconfigured */
	if(pSiteMgr->prevRadioBand != pSiteMgr->radioBand)
		siteMgr_bandParamsConfig(pSiteMgr, TRUE);

	if(pPrimarySite->channel <= MAX_GB_MODE_CHANEL)
		siteMgr_updateRates(pSiteMgr, FALSE, TRUE);
	else
		siteMgr_updateRates(pSiteMgr, TRUE, TRUE);

	/* configure hal with common core-hal parameters */
	whalCtrl_SetRadioBand(pSiteMgr->hHalCtrl, pSiteMgr->radioBand);

	pPrimarySite->currentSlotTime = slotTime;
	whalCtrl_SetSlotTime(pSiteMgr->hHalCtrl, slotTime);

	/***************** Config HAL *************************/
	/* Current Beacon Interval */
	whalCtrl_SetBeaconInterval( pSiteMgr->hHalCtrl , pPrimarySite->beaconInterval);

	/***************** Config Site Manager *************************/
	/* L.M. Should be fixed, should take into account the AP's rates */ 
	if(pSiteMgr->pDesiredParams->siteMgrDesiredModulationType == DRV_MODULATION_CCK)
		pSiteMgr->chosenModulation = DRV_MODULATION_CCK;
	else if(pSiteMgr->pDesiredParams->siteMgrDesiredModulationType == DRV_MODULATION_PBCC)
	{
		if(pPrimarySite->probeModulation != DRV_MODULATION_NONE)
			pSiteMgr->chosenModulation = pPrimarySite->probeModulation;
		else
			pSiteMgr->chosenModulation = pPrimarySite->beaconModulation;
	}
	else
		pSiteMgr->chosenModulation = DRV_MODULATION_OFDM;
	
	/* We use this variable in order tp perform the PBCC algorithm. */
	pSiteMgr->currentDataModulation = pSiteMgr->chosenModulation;
	/***************** Config Data CTRL *************************/
	
	param.paramType = CTRL_DATA_CURRENT_BSSID_PARAM;							/* Current BSSID */
	os_memoryCopy(pSiteMgr->hOs, &(param.content.ctrlDataCurrentBSSID), &(pPrimarySite->bssid), sizeof(macAddress_t));
	ctrlData_setParam(pSiteMgr->hCtrlData, &param);

	param.paramType = CTRL_DATA_CURRENT_BSS_TYPE_PARAM;							/* Current BSS Type */
	param.content.ctrlDataCurrentBssType = pPrimarySite->bssType;
	ctrlData_setParam(pSiteMgr->hCtrlData, &param);

	param.paramType = CTRL_DATA_CURRENT_PREAMBLE_TYPE_PARAM;					/* Current Preamble Type */
	if ((pSiteMgr->pDesiredParams->siteMgrDesiredPreambleType == PREAMBLE_SHORT) &&
		(pPrimarySite->currentPreambleType == PREAMBLE_SHORT))
		param.content.ctrlDataCurrentPreambleType = PREAMBLE_SHORT;
	else
		param.content.ctrlDataCurrentPreambleType = PREAMBLE_LONG;
	ctrlData_setParam(pSiteMgr->hCtrlData, &param);

    /* Mutual Rates Matching */
	StaTotalRates = pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.basicRateMask |
					pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.supportedRateMask;


    pSiteMgr->pDesiredParams->siteMgrMatchedSuppRateMask = StaTotalRates & 
														   pPrimarySite->rateMask.supportedRateMask;
	
	pSiteMgr->pDesiredParams->siteMgrMatchedBasicRateMask = StaTotalRates &
															pPrimarySite->rateMask.basicRateMask;
	if (pSiteMgr->pDesiredParams->siteMgrMatchedBasicRateMask == 0)
	{
		pSiteMgr->pDesiredParams->siteMgrMatchedBasicRateMask = 
			pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.basicRateMask;
	}

    pSiteMgr->pDesiredParams->siteMgrMatchedMaxBasicRate = getMaxRatefromBitmap(pSiteMgr->pDesiredParams->siteMgrMatchedBasicRateMask);

    pSiteMgr->pDesiredParams->siteMgrMatchedMaxActiveRate = getMaxRatefromBitmap(pSiteMgr->pDesiredParams->siteMgrMatchedSuppRateMask);

    pSiteMgr->pDesiredParams->siteMgrMatchedMaxActiveRate = MAX(pSiteMgr->pDesiredParams->siteMgrMatchedMaxBasicRate,pSiteMgr->pDesiredParams->siteMgrMatchedMaxActiveRate);

	param.paramType = CTRL_DATA_CURRENT_BASIC_RATE_PARAM;						/* Current Basic Rate */
	param.content.ctrlDataCurrentBasicRate = (rate_e)pSiteMgr->pDesiredParams->siteMgrMatchedMaxBasicRate;
	ctrlData_setParam(pSiteMgr->hCtrlData, &param);

	param.paramType = CTRL_DATA_CURRENT_BASIC_RATE_MASK_PARAM;
	param.content.ctrlDataBasicRateBitMask = pSiteMgr->pDesiredParams->siteMgrMatchedBasicRateMask;
	ctrlData_setParam(pSiteMgr->hCtrlData, &param);

	param.paramType = CTRL_DATA_CURRENT_BASIC_MODULATION_PARAM;						/* Current Mgmt Rate */
	if ((pPrimarySite->maxBasicRate == DRV_RATE_1M) || (pPrimarySite->maxBasicRate == DRV_RATE_2M))
		param.content.ctrlDataCurrentBasicModulationType = DRV_MODULATION_QPSK;
	else if (pPrimarySite->maxBasicRate == DRV_RATE_22M)
		param.content.ctrlDataCurrentBasicModulationType = DRV_MODULATION_PBCC;
	else if (pPrimarySite->maxBasicRate < DRV_RATE_22M)
		param.content.ctrlDataCurrentBasicModulationType = DRV_MODULATION_CCK;
	else
		param.content.ctrlDataCurrentBasicModulationType = DRV_MODULATION_OFDM;

	ctrlData_setParam(pSiteMgr->hCtrlData, &param);

    param.paramType = CTRL_DATA_CURRENT_PROTECTION_STATUS_PARAM;
    param.content.ctrlDataProtectionEnabled = pPrimarySite->useProtection;
    ctrlData_setParam(pSiteMgr->hCtrlData, &param);

	ctrlData_setSite(pSiteMgr->hCtrlData, &pPrimarySite->fourXParams);

	pbccAlgorithm(pSiteMgr);

	/********** Set Site QOS protocol support *************/

	/* Set WME Params */
	 status = siteMgr_getWMEParamsSite(pSiteMgr,&p_ACParametersDummy);
	 if(status == OK)
	 {
		 param.content.qosSiteProtocol = WME;
	 }
	 else
	 {
			 param.content.qosSiteProtocol = NONE_QOS;
	 }

	WLAN_REPORT_DEBUG_TX(pSiteMgr->hReport,
	 (" systemConfigt() : param.content.qosSiteProtoco %d\n", param.content.qosSiteProtocol));

	 param.paramType = QOS_MNGR_SET_SITE_PROTOCOL;
	 qosMngr_setParams(pSiteMgr->hQosMngr,&param);
	 
     /* Set active protocol in qosMngr according to station desired mode and site capabilities 
	Must be called BEFORE setting the "CURRENT_PS_MODE" into the QosMngr */
     qosMngr_selectActiveProtocol(pSiteMgr->hQosMngr);

	 /* set PS capability parameter */
	 param.paramType = QOS_MNGR_CURRENT_PS_MODE;
	 if(pPrimarySite->APSDSupport == TRUE)
		 param.content.currentPsMode = PS_SCHEME_UPSD_TRIGGER;
	 else
		 param.content.currentPsMode = PS_SCHEME_LEGACY_PSPOLL;
      qosMngr_setParams(pSiteMgr->hQosMngr,&param);

     /* Set upsd/ps_poll configuration */
     /* Must be done AFTER setting the active Protocol */
     qosMngr_setAcPsDeliveryMode (pSiteMgr->hQosMngr);
	
	/***************** Config RSN *************************/
    /* Get the RSN IE data */
    pRsnIe = pPrimarySite->pRsnIe;
	length=0;
    rsnIECount = 0;
    while ((length < pPrimarySite->rsnIeLen) && (pPrimarySite->rsnIeLen < 255) 
           && (rsnIECount < MAX_RSN_IE))
    {
        curRsnData[0+length] = pRsnIe->hdr.eleId;
        curRsnData[1+length] = pRsnIe->hdr.eleLen;
        os_memoryCopy(pSiteMgr->hOs, &curRsnData[2+length], (void *)pRsnIe->rsnIeData, pRsnIe->hdr.eleLen); 
        length += pRsnIe->hdr.eleLen+2;
        pRsnIe += 1;
        rsnIECount++;
    }
    if (length<pPrimarySite->rsnIeLen) 
    {
        WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  
                          ("siteMgr_selectSiteFromTable, RSN IE is too long: rsnIeLen=%d, MAX_RSN_IE=%d\n",
                            pPrimarySite->rsnIeLen, MAX_RSN_IE));
    }

	rsnData.pIe = (pPrimarySite->rsnIeLen==0) ? NULL :curRsnData;
	rsnData.ieLen = pPrimarySite->rsnIeLen;
    rsnData.privacy = pPrimarySite->privacy; 
    
    rsn_setSite(pSiteMgr->hRsn, &rsnData, NULL, &rsnAssocIeLen);

	/***************** Config RegulatoryDomain **************************/
	
#ifdef EXC_MODULE_INCLUDED
	/* set EXC TPC if present */
	if(Exc_ParseClientTP(pSiteMgr->hOs,pPrimarySite,&ExternTxPower,pIeBuffer,PktLength) == OK)
    {
        WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
			("Select Exc_ParseClientTP == OK: Dbm = %d\n",ExternTxPower));
		param.paramType = REGULATORY_DOMAIN_EXTERN_TX_POWER_PREFERRED;
		param.content.ExternTxPowerPreferred = ExternTxPower;
		regulatoryDomain_setParam(pSiteMgr->hRegulatoryDomain,&param);
    }
	/* Parse and save the EXC Version Number if exists */
	excMngr_parseExcVer(pSiteMgr->hExcMngr, pIeBuffer, PktLength);

#endif

	/* Note: TX Power Control adjustment is now done through siteMgr_assocReport() */
	if (pPrimarySite->powerConstraint>0)
	{	/* setting power constraint */
		param.paramType = REGULATORY_DOMAIN_SET_POWER_CONSTRAINT_PARAM;
		param.content.powerConstraint = pPrimarySite->powerConstraint;
		regulatoryDomain_setParam(pSiteMgr->hRegulatoryDomain,&param);
	}


	/***************** Config MeasurementMgr object **************************/
    capabilities = pPrimarySite->capabilities;

    /* Updating the Measurement Module Mode */
    measurementMgr_setMeasurementMode(pSiteMgr->hMeasurementMgr, capabilities, 
									pIeBuffer, PktLength);

    
	return OK;
}

