/** \file siteMgrApi.h
 *  \brief site manager module API
 *
 *  \see siteMgr.c
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
/*                                                                                                  */
/*    MODULE:   siteMgrApi.h                                                                */
/*    PURPOSE:  site manager module API                                         */
/*                                                                                                  */
/***************************************************************************/
#ifndef __SITE_MGR_API_H__
#define __SITE_MGR_API_H__

#include "osTIType.h"
#include "paramOut.h"
#include "paramIn.h"
#include "802_11Defs.h"
#include "mlmeApi.h"
#include "siteHash.h"
#include "ScanCncnApi.h"
#include "bssTypes.h"


/* Site manager interface functions prototypes */

TI_HANDLE siteMgr_create(TI_HANDLE hOs);

TI_STATUS siteMgr_config(   TI_HANDLE       hSiteMgr,
                        TI_HANDLE       hConn,
                        TI_HANDLE       hSmeSm,
                        TI_HANDLE       hCtrlData,
                        TI_HANDLE       hRxData,
                        TI_HANDLE       hTxData,
                        TI_HANDLE       hRsn,
                        TI_HANDLE       hAuth,
                        TI_HANDLE       hAssoc,
                        TI_HANDLE       hHalCtrl,
                        TI_HANDLE       hMlmeSm,
                        TI_HANDLE       hRegulatoryDomain,
                        TI_HANDLE       hMeasurementMgr,
                        TI_HANDLE       hApConn,
                        TI_HANDLE       hCurrBss,
                        TI_HANDLE       hReport,
                        TI_HANDLE       hOs ,
                        TI_HANDLE       hExcMngr,
                        TI_HANDLE       hQosMngr,
                        TI_HANDLE       thePowerMgrHandle,
                        TI_HANDLE       hScr,
                        TI_HANDLE       hEvHandler,
                        TI_HANDLE       hMacServices,
                        siteMgrInitParams_t     *pSiteMgrInitParams);

TI_STATUS siteMgr_unLoad(TI_HANDLE hSiteMgr);

TI_STATUS siteMgr_setParam(TI_HANDLE        hSiteMgr,
                        paramInfo_t     *pParam);

TI_STATUS siteMgr_getParam(TI_HANDLE        hSiteMgr, 
                        paramInfo_t     *pParam);

TI_STATUS siteMgr_join(TI_HANDLE    hSiteMgr);

TI_STATUS siteMgr_forceInfraJoin(TI_HANDLE    hSiteMgr);
TI_STATUS siteMgr_disJoin(TI_HANDLE hSiteMgr);

TI_STATUS siteMgr_removeSelfSite(TI_HANDLE  hSiteMgr);

TI_STATUS siteMgr_disSelectSite(TI_HANDLE   hSiteMgr);

TI_STATUS systemConfig(siteMgr_t *pSiteMgr);

TI_STATUS siteMgr_start(TI_HANDLE   hSiteMgr);

TI_STATUS siteMgr_stop(TI_HANDLE    hSiteMgr);

TI_STATUS siteMgr_resetSiteTable(TI_HANDLE  hSiteMgr, siteTablesParams_t*   pSiteTableParams);

TI_STATUS siteMgr_removeNotReceivedSites(TI_HANDLE  hSiteMgr);

TI_STATUS siteMgr_updatePrimarySiteFailStatus(TI_HANDLE hSiteMgr, 
                                           BOOL bRemoveSite);

TI_STATUS siteMgr_resetPrimarySiteAttemptsNumber(TI_HANDLE  hSiteMgr);

TI_STATUS siteMgr_resetPrevPrimarySiteRssi(TI_HANDLE    hSiteMgr);

TI_STATUS siteMgr_selectSite(TI_HANDLE  hSiteMgr);

siteEntry_t* siteMgr_selectSiteFromTable(TI_HANDLE  hSiteMgr);

void siteMgr_setNotReceivedParameter(TI_HANDLE  hSiteMgr, ssid_t* ssid , radioBand_e band);

void siteMgr_resetAttemptsNumberParameter(TI_HANDLE hSiteMgr);

void siteMgrControlLnaOperation(TI_HANDLE hSiteMgr, BOOL NextState);

BOOL siteMgr_isCurrentBand24(TI_HANDLE  hSiteMgr);

BOOL siteMgr_isThereCountryIEforCurrentBand(TI_HANDLE   hSiteMgr);

BOOL siteMgr_isThereValidSSID (TI_HANDLE hSiteMgr);

TI_STATUS pbccAlgorithm(TI_HANDLE hSiteMgr);

TI_STATUS siteMgr_assocReport(TI_HANDLE hSiteMgr, UINT16 capabilities, BOOL bCiscoAP);

void siteMgr_setCurrentTable(TI_HANDLE hSiteMgr, radioBand_e radioBand);

void siteMgr_updateRates(TI_HANDLE hSiteMgr, BOOL dot11a, BOOL updateToOS);

void siteMgr_bandParamsConfig(TI_HANDLE hSiteMgr, BOOL updateToOS);

void siteMgr_ConfigRate(TI_HANDLE hSiteMgr);

TI_STATUS siteMgr_getWMEParamsSite(TI_HANDLE hSiteMgr,ACParameters_t **pWME_ACParameters_t);

TI_STATUS siteMgr_setWMEParamsSite(TI_HANDLE hSiteMgr,dot11_WME_PARAM_t *pDot11_WME_PARAM);

void siteMgr_resetChannelList(TI_HANDLE hSiteMgr);

siteEntry_t *siteMgr_findSiteEntry(TI_HANDLE hSiteMgrm, macAddress_t *bssid);

TI_STATUS siteMgr_resetEventStatisticsHistory(TI_HANDLE hSiteMgr);

void siteMgr_IsERP_Needed(TI_HANDLE hSiteMgr,BOOL *useProtection,BOOL *NonErpPresent,BOOL *barkerPreambleType);

TI_STATUS siteMgr_updateNewApList (TI_HANDLE hSiteMgr);

TI_STATUS siteMgr_overwritePrimarySite(TI_HANDLE hSiteMgr, bssEntry_t *newAP, BOOL requiredToStorePrevSite);

void siteMgr_clearFirstBcnFlag(TI_HANDLE hSiteMgr);

void siteMgr_setFirstBcnFlag(TI_HANDLE hSiteMgr);

void siteMgr_checkTxPower(TI_HANDLE hSiteMgr);

void siteMgr_printPrimarySiteDesc(TI_HANDLE hSiteMgr );

void siteMgr_keepAliveSendNullDataTimer(TI_HANDLE hSiteMgr);

#endif /* __SITE_MGR_API_H__ */
