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

/** \file siteMgr.c
 *  \brief Site Manager implementation
 *
 *  \see siteMgr.h
 */

#include "report.h"
#include "osTIType.h"
#include "osApi.h"
#include "siteMgrApi.h"
#include "siteHash.h"
#include "smeApi.h"
#include "utils.h"
#include "connApi.h"
#include "mlmeSm.h"
#include "smeSmApi.h"
#include "DataCtrl_Api.h"
#include "regulatoryDomainApi.h"
#include "rsnApi.h"
#include "measurementMgrApi.h"
#include "qosMngr_API.h"
#include "PowerMgr_API.h"
#include "EvHandler.h"
#include "TI_IPC_Api.h"
#include "MacServices_api.h" 
#include "whalHwDefs.h"
#include "apConn.h"
#include "currBss.h"
#include "PowerMgr.h"

#ifdef EXC_MODULE_INCLUDED
#include "excMngr.h"
#endif
#include "configMgr.h"

/* definitions */

#define JOIN_RATE_MASK_1M   0x01
#define JOIN_RATE_MASK_2M   0x02
#define JOIN_RATE_MASK_5_5M 0x04
#define JOIN_RATE_MASK_11M  0x08
#define JOIN_RATE_MASK_22M  0x10


#define SITE_MGR_INIT_BIT           1
#define TIMER_INIT_BIT              2
#define DESIRED_PARAMS_INIT_BIT     3
#define MGMT_PARAMS_INIT_BIT        4

#define BUILT_IN_TEST_PERIOD 500

#define KEEP_ALIVE_SEND_NULL_DATA_PERIOD  10000

#define SITE_MGR_IBSS_AGING_TIMEOUT_DEF   10 * 1000        /* 10 seconds */

#define DEAFULT_BEACON_FILTERING_NUM        (10)

/* Reconfig constants */
#define SCAN_FAIL_THRESHOLD_FOR_RECONFIG        4  /* After 4 times we reset the 580 register and still no AP found - make recovery */
#define SCAN_FAIL_THRESHOLD_FOR_RESET_REG_580   90  /* After 90 times (45 seconds) and  no AP found - reset the 580 register */
#define SCAN_FAIL_RECONFIG_ENABLED              TRUE
#define SCAN_FAIL_RECONFIG_DISABLED             FALSE

/* Local Macros */

#define UPDATE_BEACON_INTERVAL(pSite, pFrameInfo)       pSite->beaconInterval = pFrameInfo->content.iePacket.beaconInerval

#define UPDATE_CAPABILITIES(pSite, pFrameInfo)          pSite->capabilities = pFrameInfo->content.iePacket.capabilities

#define UPDATE_PRIVACY(pSite, pFrameInfo)               pSite->privacy = ((pFrameInfo->content.iePacket.capabilities >> CAP_PRIVACY_SHIFT) & CAP_PRIVACY_MASK) ? TRUE : FALSE

#define UPDATE_AGILITY(pSite, pFrameInfo)               pSite->agility = ((pFrameInfo->content.iePacket.capabilities >> CAP_AGILE_SHIFT) & CAP_AGILE_MASK) ? TRUE : FALSE

#define UPDATE_SLOT_TIME(pSite, pFrameInfo)             pSite->newSlotTime = ((pFrameInfo->content.iePacket.capabilities >> CAP_SLOT_TIME_SHIFT) & CAP_SLOT_TIME_MASK) ? PHY_SLOT_TIME_SHORT : PHY_SLOT_TIME_LONG
#define UPDATE_PROTECTION(pSite, pFrameInfo)            pSite->useProtection = (pFrameInfo->content.iePacket.useProtection)

#define UPDATE_SSID(pSite, pFrameInfo)                  if (pFrameInfo->content.iePacket.pSsid != NULL) { \
                                                        pSite->ssid.len = pFrameInfo->content.iePacket.pSsid->hdr.eleLen; \
        os_memoryCopy(pSiteMgr->hOs, (void *)pSite->ssid.ssidString, (void *)pFrameInfo->content.iePacket.pSsid->serviceSetId, pFrameInfo->content.iePacket.pSsid->hdr.eleLen) ;}

#define UPDATE_CHANNEL(pSite, pFrameInfo, rxChannel)    if (pFrameInfo->content.iePacket.pDSParamsSet == NULL) \
                                                            pSite->channel = rxChannel; \
                                                        else \
                                                            pSite->channel = pFrameInfo->content.iePacket.pDSParamsSet->currChannel;



#define UPDATE_DTIM_PERIOD(pSite, pFrameInfo)           if (pFrameInfo->content.iePacket.pTIM != NULL) \
                                                        pSite->dtimPeriod = pFrameInfo->content.iePacket.pTIM->dtimPeriod

#define UPDATE_ATIM_WINDOW(pSite, pFrameInfo)           if (pFrameInfo->content.iePacket.pIBSSParamsSet != NULL) \
                                                        pSite->atimWindow = pFrameInfo->content.iePacket.pIBSSParamsSet->atimWindow

#define UPDATE_BEACON_AP_TX_POWER(pSite, pFrameInfo)    if (pFrameInfo->content.iePacket.TPCReport != NULL) \
                                                        pSite->APTxPower = pFrameInfo->content.iePacket.TPCReport->transmitPower

#define UPDATE_PROBE_AP_TX_POWER(pSite, pFrameInfo)     if (pFrameInfo->content.iePacket.TPCReport != NULL) \
                                                        pSite->APTxPower = pFrameInfo->content.iePacket.TPCReport->transmitPower

#define UPDATE_BSS_TYPE(pSite, pFrameInfo)              pSite->bssType = ((pFrameInfo->content.iePacket.capabilities >> CAP_ESS_SHIFT) & CAP_ESS_MASK) ? BSS_INFRASTRUCTURE : BSS_INDEPENDENT

#define UPDATE_LOCAL_TIME_STAMP(pSiteMgr, pSite, pFrameInfo)        pSite->localTimeStamp = os_timeStampMs(pSiteMgr->hOs)

#define UPDATE_DTIM_TIME(pSiteMgr, pSite, pFrameInfo)       pSite->dtimTimeStamp = os_timeStampMs(pSiteMgr->hOs)

/* Updated from beacons */
#define UPDATE_BEACON_MODULATION(pSite, pFrameInfo)     pSite->beaconModulation = ((pFrameInfo->content.iePacket.capabilities >> CAP_PBCC_SHIFT) & CAP_PBCC_MASK) ? DRV_MODULATION_PBCC : DRV_MODULATION_CCK

/* Updated from probes */
#define UPDATE_PROBE_MODULATION(pSite, pFrameInfo)          pSite->probeModulation = ((pFrameInfo->content.iePacket.capabilities >> CAP_PBCC_SHIFT) & CAP_PBCC_MASK) ? DRV_MODULATION_PBCC : DRV_MODULATION_CCK

#define UPDATE_BEACON_RECV(pSite)                       pSite->beaconRecv = TRUE

#define UPDATE_PROBE_RECV(pSite)                        pSite->probeRecv = TRUE


#define UPDATE_RSN_IE(pSite, pRsnIe, rsnIeLen)              if (pRsnIe != NULL) { \
                                                            UINT8 length=0, index=0;\
                                                            pSite->rsnIeLen = rsnIeLen;\
                                                            while ((length < pSite->rsnIeLen) && (index<MAX_RSN_IE)){\
                                                                pSite->pRsnIe[index].hdr = pRsnIe->hdr;\
                                                                os_memoryCopy(pSiteMgr->hOs, (void *)pSite->pRsnIe[index].rsnIeData, (void *)pRsnIe->rsnIeData, pRsnIe->hdr.eleLen);\
                                                                length += (pRsnIe->hdr.eleLen+2); \
                                                                pRsnIe += 1; \
                                                                index++;}\
                                                        } \
                                                        else {pSite->rsnIeLen = 0;}

#define UPDATE_BEACON_TIMESTAMP(pSiteMgr, pSite, pFrameInfo)    os_memoryCopy(pSiteMgr->hOs, pSite->tsfTimeStamp, (void *)pFrameInfo->content.iePacket.timestamp, TIME_STAMP_LEN)

#define SET_ENTRY_FLAG_IN_SITE_TABLE(pSite)             pSite->Not_Received = 0




/* Local  functions definitions*/

static void update_apsd(siteEntry_t *pSite, mlmeFrameInfo_t *pFrameInfo);

static void release_module(siteMgr_t *pSiteMgr, UINT32 initVec);

static void updateSiteInfo(siteMgr_t *pSiteMgr, mlmeFrameInfo_t *pFrameInfo, siteEntry_t    *pSite, UINT8 rxChannel);

static void updateRates(siteMgr_t *pSiteMgr, siteEntry_t *pSite, mlmeFrameInfo_t *pFrameInfo);

static void updateBeaconQosParams(siteMgr_t *pSiteMgr, siteEntry_t *pSite, mlmeFrameInfo_t *pFrameInfo);

static void updateProbeQosParams(siteMgr_t *pSiteMgr, siteEntry_t *pSite, mlmeFrameInfo_t *pFrameInfo);

static void updatePreamble(siteMgr_t *pSiteMgr, siteEntry_t *pSite, mlmeFrameInfo_t *pFrameInfo);

static void updateFourX(siteMgr_t *pSiteMgr, siteEntry_t *pSite, mlmeFrameInfo_t *pFrameInfo);

static TI_STATUS getBssidList(siteMgr_t *pSiteMgr, OS_802_11_BSSID_LIST_EX *bssidList, UINT32* pLength, BOOL allVarIes);

static void getPrimarySiteDesc(siteMgr_t *pSiteMgr, OS_802_11_BSSID *pPrimarySiteDesc, BOOL supplyExtendedInfo);

static TI_STATUS getPrimaryBssid(siteMgr_t *pSiteMgr, OS_802_11_BSSID_EX *primaryBssid, UINT32 *pLength);

static rate_e translateRateMaskToValue(siteMgr_t *pSiteMgr, UINT32 rateMask);

static void getSupportedRateSet(siteMgr_t *pSiteMgr, rates_t *pRatesSet);

static TI_STATUS setSupportedRateSet(siteMgr_t *pSiteMgr, rates_t *pRatesSet);

static void validateDesiredTxRate(rate_e desiredTxRate,modulationType_e desiredModulation,UINT32 suppRates,UINT32 *bitMap,BOOL *txDesiredRateSupported);

static TI_STATUS calculateBssidListSize(siteMgr_t *pSiteMgr, UINT32 *pLength, BOOL allVarIes);

static void siteMgr_externalConfigurationParametersSet(TI_HANDLE hSiteMgr);


void siteMgr_gotFirstBcn(TI_HANDLE hSiteMgr);

/**************************************************************/
/* DEBUG CLI CRASH                                            */
/**************************************************************/
static    whalCtrl_joinBss_t      joinParams;
static    whalCtrl_setTemplate_t  templateStruct;
static    probeRspTemplate_t      probeRspTemplate;
static    nullDataTemplate_t      nullDataTemplate;
static    psPollTemplate_t        psPollTemplate;
static    QosNullDataTemplate_t   QosNullDataTemplate;
/**************************************************************/

#define CHAN_FREQ_TABLE_SIZE        (sizeof(ChanFreq) / sizeof(struct CHAN_FREQ))

struct CHAN_FREQ {
    UINT8       chan;
    UINT32      freq;
} ChanFreq[] = {
    {1,2412000}, {2,2417000}, {3,2422000}, {4,2427000},
    {5,2432000}, {6,2437000}, {7,2442000}, {8,2447000},
    {9,2452000},
    {10,2457000}, {11,2462000}, {12,2467000}, {13,2472000},
    {14,2484000}, {36,5180000}, {40,5200000}, {44,5220000},
    {48,5240000}, {52,5260000}, {56,5280000}, {60,5300000},
    {64,5320000},
    {100,5500000}, {104,5520000}, {108,5540000}, {112,5560000},
    {116,5580000}, {120,5600000}, {124,5620000}, {128,5640000},
    {132,5660000}, {136,5680000}, {140,5700000}, {149,5745000},
    {153,5765000}, {157,5785000}, {161,5805000} };



static UINT8 Freq2Chan(UINT32 freq)
{
    UINT32 i;

    for(i=0; i<CHAN_FREQ_TABLE_SIZE; i++)
        if(ChanFreq[i].freq == freq) return ChanFreq[i].chan;

    return 0;
}


static UINT32 Chan2Freq(UINT8 chan)
{
    UINT32 i;

    for(i=0; i<CHAN_FREQ_TABLE_SIZE; i++)
        if(ChanFreq[i].chan == chan) return ChanFreq[i].freq;

    return 0;
}

/************************************************************************
*                        siteMgr_setTemporaryTxPower                    *
*************************************************************************
DESCRIPTION:	This function is used to start the Tx Power Control adjust mechanism
				in regulatoryDomain.

INPUT:      bActivateTempFix             -   Whether the power should be adjusted 
************************************************************************/
void siteMgr_setTemporaryTxPower(siteMgr_t* pSiteMgr, BOOL bActivateTempFix)
{
	paramInfo_t         param;

	WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG, 
		("siteMgr_setTemporaryTxPower is = %s \n", (bActivateTempFix ? "ON" : "OFF")));

	/* Set the temporary Power Level via the Regulatory Domain*/
	param.paramType = REGULATORY_DOMAIN_TEMPORARY_TX_ATTENUATION_PARAM;
	param.content.bActivateTempPowerFix = bActivateTempFix;
	regulatoryDomain_setParam(pSiteMgr->hRegulatoryDomain,&param);
}

/* Interface functions Implementation */


/*static void UPDATE_RSN_IE (siteMgr_t* pSiteMgr, siteEntry_t   *pSite, dot11_RSN_t *pRsnIe, UINT8 rsnIeLen)
{

    if (pRsnIe != NULL) {
            UINT8 length=0, index=0;
            pSite->rsnIeLen = rsnIeLen;
            while ((length < pSite->rsnIeLen) && (index<MAX_RSN_IE)){
                pSite->pRsnIe[index].hdr = pRsnIe->hdr;
                os_memoryCopy(pSiteMgr->hOs, pSite->pRsnIe[index].rsnIeData, pRsnIe->rsnIeData, pRsnIe->hdr.eleLen);
                length += (pRsnIe->hdr.eleLen+2);
                pRsnIe += 1;
                index++;}
        }
        else {pSite->rsnIeLen = 0;}
}*/
/************************************************************************
 *                        siteMgr_create                                *
 ************************************************************************
DESCRIPTION: Site manager module creation function, called by the config mgr in creation phase
                performs the following:
                -   Allocate the site manager handle
                -   Allocate the desired & mgmt params structure

INPUT:      hOs             -   Handle to OS


OUTPUT:


RETURN:     Handle to the site manager module on success, NULL otherwise
************************************************************************/
TI_HANDLE siteMgr_create(TI_HANDLE hOs)
{
    siteMgr_t       *pSiteMgr;
    UINT32          initVec;

    initVec = 0;

    pSiteMgr = os_memoryAlloc(hOs, sizeof(siteMgr_t));
    if (pSiteMgr == NULL)
        return NULL;

    os_memoryZero(hOs, pSiteMgr, sizeof(siteMgr_t));

    initVec |= (1 << SITE_MGR_INIT_BIT);

    pSiteMgr->pDesiredParams = os_memoryAlloc(hOs, sizeof(siteMgrInitParams_t));
    if (pSiteMgr->pDesiredParams == NULL)
    {
        release_module(pSiteMgr, initVec);
        return NULL;
    }

    initVec |= (1 << DESIRED_PARAMS_INIT_BIT);

    pSiteMgr->pSitesMgmtParams = os_memoryAlloc(hOs, sizeof(sitesMgmtParams_t));
    if (pSiteMgr->pSitesMgmtParams == NULL)
    {
        release_module(pSiteMgr, initVec);
        return NULL;
    }

    initVec |= (1 << MGMT_PARAMS_INIT_BIT);

    pSiteMgr->hOs = hOs;

    return(pSiteMgr);
}



/************************************************************************
 *                        siteMgr_config                                    *
 ************************************************************************
DESCRIPTION: Site manager module configuration function, called by the config mgr in configuration phase
                performs the following:
                -   Reset & initiailzes local variables
                -   Init the handles to be used by the module

INPUT:      hSiteMgr    -   site manager handle
            List of handles to be used by the module
            pSiteMgrInitParams  -   Init table of the module, contains the following:
                -   Parameters read from registry
                -   Chip parameters


OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
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
                        siteMgrInitParams_t     *pSiteMgrInitParams)
{ 
    siteMgr_t       *pSiteMgr = (siteMgr_t *)hSiteMgr;
    UINT32          timestamp;
    slotTime_e      slotTime;
    paramInfo_t     saParam;
    TI_STATUS       status;

    /* Init handles */
    pSiteMgr->hConn                 = hConn;
    pSiteMgr->hSmeSm                = hSmeSm;
    pSiteMgr->hHalCtrl              = hHalCtrl;
    pSiteMgr->hCtrlData             = hCtrlData;
    pSiteMgr->hRxData               = hRxData;
    pSiteMgr->hTxData               = hTxData;
    pSiteMgr->hRsn                  = hRsn;
    pSiteMgr->hAuth                 = hAuth;
    pSiteMgr->hAssoc                = hAssoc;
    pSiteMgr->hRegulatoryDomain     = hRegulatoryDomain;
    pSiteMgr->hMeasurementMgr       = hMeasurementMgr;
    pSiteMgr->hReport               = hReport;
    pSiteMgr->hOs                   = hOs;
    pSiteMgr->hMlmeSm               = hMlmeSm;
    pSiteMgr->hAssoc                = hAssoc;
    pSiteMgr->hReport               = hReport;
    pSiteMgr->hExcMngr              = hExcMngr;
    pSiteMgr->hApConn               = hApConn;
    pSiteMgr->hCurrBss              = hCurrBss;
    pSiteMgr->hQosMngr              = hQosMngr;
    pSiteMgr->hPowerMgr             = thePowerMgrHandle;
    pSiteMgr->hScr                  = hScr;
    pSiteMgr->hEvHandler            = hEvHandler;
    pSiteMgr->hMacServices          = hMacServices;

	/* Reset counter for Tx Power Control adjustment */
    pSiteMgr->siteMgrTxPowerCheckTime   = 0;

    /* Init desired parameters */
    os_memoryCopy(hOs, pSiteMgr->pDesiredParams, pSiteMgrInitParams, sizeof(siteMgrInitParams_t));

    /* Init Beacon Filter Desired State */
    pSiteMgr->beaconFilterParams.desiredState = pSiteMgrInitParams->beaconFilterParams.desiredState;
    /* Init Beacon Filter numOfStored parameter */
    pSiteMgr->beaconFilterParams.numOfStored = pSiteMgrInitParams->beaconFilterParams.numOfStored;

    /* Init management params */
    pSiteMgr->pSitesMgmtParams->dot11A_sitesTables.maxNumOfSites = MAX_SITES_A_BAND;
    siteMgr_resetSiteTable(pSiteMgr,(siteTablesParams_t *)&pSiteMgr->pSitesMgmtParams->dot11A_sitesTables);
    pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables.maxNumOfSites = MAX_SITES_BG_BAND;
    siteMgr_resetSiteTable(pSiteMgr,&pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables);

    /* calculate random BSSID for usage in IBSS */
    timestamp = os_timeStampMs(pSiteMgr->hOs);
    os_memoryCopy(pSiteMgr->hOs, (void *)&(pSiteMgr->ibssBssid.addr[0]), &timestamp, sizeof(UINT32));
    pSiteMgr->ibssBssid.addr[0] = 0x00;
    timestamp = os_timeStampMs(pSiteMgr->hOs);
    os_memoryCopy(pSiteMgr->hOs, (void *)&(pSiteMgr->ibssBssid.addr[2]), &timestamp, sizeof(UINT32));

    /* Get the Source MAC address in order to use it for AD-Hoc BSSID, solving Conexant ST issue for WiFi test */
    saParam.paramType = CTRL_DATA_MAC_ADDRESS;
    status = ctrlData_getParam(hCtrlData, &saParam);
    if (status != OK)
    {
        WLAN_OS_REPORT(("\n ERROR !!! : siteMgr_config - Error in getting MAC address\n" ));
        return NOK;
    }
    pSiteMgr->ibssBssid.addr[0] = 0x02;
    pSiteMgr->ibssBssid.addr[1] = saParam.content.ctrlDataDeviceMacAddress.addr[1];
    pSiteMgr->ibssBssid.addr[2] = saParam.content.ctrlDataDeviceMacAddress.addr[2];

    pSiteMgr->keepAliveEnable = pSiteMgrInitParams->siteMgrDesiredkeepAliveEnable;
    pSiteMgr->numOfBeaconFiltering = DEAFULT_BEACON_FILTERING_NUM;

    switch(pSiteMgr->pDesiredParams->siteMgrRadioValues.siteMgr_radioType)
    {
    case MAXIM:
    pSiteMgr->pDesiredParams->siteMgrRadioValues.pSiteMgr_selectedRadioValues =
        &pSiteMgr->pDesiredParams->siteMgrRadioValues.siteMgr_maximRadioValues;
    pSiteMgr->pDesiredParams->siteMgrSupportedBand = RADIO_BAND_2_4_GHZ;
       WLAN_REPORT_INIT(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("MAXIM  !!! \n"));
    
        break;

    case RFMD:
        pSiteMgr->pDesiredParams->siteMgrRadioValues.pSiteMgr_selectedRadioValues =
            &pSiteMgr->pDesiredParams->siteMgrRadioValues.siteMgr_rfmdRadioValues;
        pSiteMgr->pDesiredParams->siteMgrSupportedBand = RADIO_BAND_2_4_GHZ;
        WLAN_REPORT_INIT(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("RFMD  !!! \n"));
        break;

    case RADIA_BG:
    pSiteMgr->pDesiredParams->siteMgrRadioValues.pSiteMgr_selectedRadioValues =
        &pSiteMgr->pDesiredParams->siteMgrRadioValues.siteMgr_radiaRadioValues;
    pSiteMgr->pDesiredParams->siteMgrSupportedBand = RADIO_BAND_2_4_GHZ;
       WLAN_REPORT_INIT(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("RADIA bg  !!! \n"));
       break;

    case RADIA_ABG:
    pSiteMgr->pDesiredParams->siteMgrRadioValues.pSiteMgr_selectedRadioValues =
        &pSiteMgr->pDesiredParams->siteMgrRadioValues.siteMgr_radiaRadioValues;
    pSiteMgr->pDesiredParams->siteMgrSupportedBand = RADIO_BAND_DUAL;
    WLAN_REPORT_INIT(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("RADIA abg  !!! \n"));

        break;

    default:
    pSiteMgr->pDesiredParams->siteMgrRadioValues.pSiteMgr_selectedRadioValues = NULL;
    WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                    ("!!!.....UnKnown Radio Type !!!\n"));          
    }

    pSiteMgr->beaconSentCount = 0;
    pSiteMgr->pDesiredParams->siteMgrDesiredAtimWindow = 0;

    if(pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode == DOT11_DUAL_MODE)
    {
       if(pSiteMgr->pDesiredParams->siteMgrSupportedBand == RADIO_BAND_DUAL)
       {
           pSiteMgr->siteMgrOperationalMode = DOT11_G_MODE;
           pSiteMgr->radioBand = RADIO_BAND_2_4_GHZ;
           slotTime = pSiteMgr->pDesiredParams->siteMgrDesiredSlotTime;
           pSiteMgr->pSitesMgmtParams->pCurrentSiteTable = &pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables;
       }
       else if(pSiteMgr->pDesiredParams->siteMgrSupportedBand == RADIO_BAND_2_4_GHZ)
       {
           pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode = DOT11_G_MODE;
           pSiteMgr->siteMgrOperationalMode = DOT11_G_MODE;
           pSiteMgr->radioBand = RADIO_BAND_2_4_GHZ;
           slotTime = pSiteMgr->pDesiredParams->siteMgrDesiredSlotTime;
           pSiteMgr->pSitesMgmtParams->pCurrentSiteTable = &pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables;
       }
       else
       {
           pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode = DOT11_A_MODE;
           pSiteMgr->siteMgrOperationalMode = DOT11_A_MODE;
           pSiteMgr->radioBand = RADIO_BAND_5_0_GHZ;
           slotTime = PHY_SLOT_TIME_SHORT;
           pSiteMgr->pSitesMgmtParams->pCurrentSiteTable = (siteTablesParams_t *)&pSiteMgr->pSitesMgmtParams->dot11A_sitesTables;
       }
    }
    else if(pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode == DOT11_G_MODE)
    {
        slotTime = pSiteMgr->pDesiredParams->siteMgrDesiredSlotTime;
        pSiteMgr->radioBand = RADIO_BAND_2_4_GHZ;
        if((pSiteMgr->pDesiredParams->siteMgrSupportedBand == RADIO_BAND_DUAL) ||
           (pSiteMgr->pDesiredParams->siteMgrSupportedBand == RADIO_BAND_2_4_GHZ))
        {
            pSiteMgr->pSitesMgmtParams->pCurrentSiteTable = &pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables;
            pSiteMgr->siteMgrOperationalMode = pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode;

        }
        else
        {
            WLAN_OS_REPORT(("\nERROR !!!.....The radio doesn't support the desired dot11 mode !!! \n"));
            return NOK;
        }
    }
    else if(pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode == DOT11_B_MODE)
    {
        slotTime = PHY_SLOT_TIME_LONG;
        pSiteMgr->radioBand = RADIO_BAND_2_4_GHZ;
        if((pSiteMgr->pDesiredParams->siteMgrSupportedBand == RADIO_BAND_DUAL) ||
           (pSiteMgr->pDesiredParams->siteMgrSupportedBand == RADIO_BAND_2_4_GHZ))
        {
            pSiteMgr->pSitesMgmtParams->pCurrentSiteTable = &pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables;
            pSiteMgr->siteMgrOperationalMode = pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode;
        }
        else
        {
            WLAN_OS_REPORT(("\nERROR !!!.....The radio doesn't support the desired dot11 mode !!! \n"));
            return NOK;
        }
    }
    else
    {
        slotTime = PHY_SLOT_TIME_SHORT;
        pSiteMgr->radioBand = RADIO_BAND_5_0_GHZ;
        if((pSiteMgr->pDesiredParams->siteMgrSupportedBand == RADIO_BAND_DUAL) ||
           (pSiteMgr->pDesiredParams->siteMgrSupportedBand == RADIO_BAND_5_0_GHZ))
        {
            pSiteMgr->siteMgrOperationalMode = pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode;
            pSiteMgr->pSitesMgmtParams->pCurrentSiteTable = (siteTablesParams_t *)&pSiteMgr->pSitesMgmtParams->dot11A_sitesTables;
        }
        else
        {
            WLAN_OS_REPORT(("\nERROR !!!.....The radio doesn't support the desired dot11 mode !!! \n"));
            return NOK;
        }
    }


    /* configure hal with common core-hal parameters */
    whalCtrl_SetRadioBand(pSiteMgr->hHalCtrl, pSiteMgr->radioBand);
    whalCtrl_SetSlotTime(pSiteMgr->hHalCtrl, slotTime);
    siteMgr_ConfigRate(hSiteMgr);

    WLAN_REPORT_INIT(hReport, SITE_MGR_MODULE_LOG,
        (" SiteMgr - numOfElements = %d IETableSize = %d\n" , pSiteMgrInitParams->beaconFilterParams.numOfElements, pSiteMgrInitParams->beaconFilterParams.IETableSize)) ;
    /*send the table regardless to the state*/
    whalCtrl_SetBeaconFilterIETable(pSiteMgr->hHalCtrl ,(UINT8 *)&(pSiteMgrInitParams->beaconFilterParams.numOfElements), (UINT8 *)&(pSiteMgrInitParams->beaconFilterParams.IETable[0]) , (UINT8 *)&(pSiteMgrInitParams->beaconFilterParams.IETableSize)) ;

    /*  At start-up Set the Beacon Filter state as the User required */
    whalCtrl_SetBeaconFiltering(pSiteMgr->hHalCtrl, pSiteMgrInitParams->beaconFilterParams.desiredState, pSiteMgr->beaconFilterParams.numOfStored);
    
    pSiteMgr->pSitesMgmtParams->pPrevPrimarySite = NULL;

    pSiteMgr->powerSaveLdMode = FALSE;

    WLAN_REPORT_INIT(hReport, SITE_MGR_MODULE_LOG,  (".....Site manager configured successfully\n"));

    return OK;
}


/************************************************************************
 *                        siteMgr_unLoad                                    *
 ************************************************************************
DESCRIPTION: site manager module unload function, called by the config mgr in the unlod phase
                performs the following:
                -   Free all memory aloocated by the module

INPUT:      hSiteMgr    -   site mgr handle.


OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS siteMgr_unLoad(TI_HANDLE hSiteMgr)
{
    UINT32          initVec;
    siteMgr_t       *pSiteMgr = (siteMgr_t *)hSiteMgr;

    if (!pSiteMgr)
        return OK;

    initVec = 0xFFFF;
    release_module(pSiteMgr, initVec);

    return OK;
}

/***********************************************************************
 *                        siteMgr_setParam
 ***********************************************************************
DESCRIPTION: site mgr set param function, called by the following:
                -   config mgr in order to set a parameter from the OS abstraction layer.
                In this fuction, the site manager OS abstraction layer configures the site manager to the desired params.
                Sometimes it requires a re scan, depending in the parameter type

INPUT:      hSiteMgr    -   Connection handle.
            pParam  -   Pointer to the parameter

OUTPUT:

RETURN:     RE_SCAN_NEEDED if re scan needed, OK on success, NOK on failure

************************************************************************/

TI_STATUS siteMgr_setParam(TI_HANDLE        hSiteMgr,
                        paramInfo_t     *pParam)
{
    siteMgr_t *pSiteMgr = (siteMgr_t *)hSiteMgr;
    siteEntry_t *pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;
    OS_802_11_CONFIGURATION *pConfig;
    UINT32      channel;
    slotTime_e  slotTime;

    switch(pParam->paramType)
    {
    case SITE_MGR_CONFIGURATION_PARAM:
        pConfig = pParam->content.pSiteMgrConfiguration;

/*      for(channel = 0; channel < SITE_MGR_CHANNEL_MAX+1; channel++)
        {
            if(pConfig->channel == pSiteMgr->pDesiredParams->siteMgrFreq2ChannelTable[channel])
                break;
        }*/

        channel = Freq2Chan(pConfig->Union.channel);

        if(channel == 0 || channel > SITE_MGR_CHANNEL_MAX)
            return PARAM_VALUE_NOT_VALID;
        else
            pConfig->Union.channel = channel;

        if((pSiteMgr->pDesiredParams->siteMgrDesiredChannel != pConfig->Union.channel) ||
           (pSiteMgr->pDesiredParams->siteMgrDesiredChannel != pConfig->Union.channel) ||
           (pSiteMgr->pDesiredParams->siteMgrDesiredAtimWindow != pConfig->ATIMWindow))
        {
            pSiteMgr->pDesiredParams->siteMgrDesiredChannel = (UINT8)pConfig->Union.channel;
            pSiteMgr->pDesiredParams->siteMgrDesiredBeaconInterval = (UINT16)pConfig->BeaconPeriod;
            pSiteMgr->pDesiredParams->siteMgrDesiredAtimWindow = pConfig->ATIMWindow;
        }

        return OK;

    case SITE_MGR_DESIRED_CHANNEL_PARAM:
        if (pParam->content.siteMgrDesiredChannel > SITE_MGR_CHANNEL_MAX)
            return PARAM_VALUE_NOT_VALID;

        if (pSiteMgr->pDesiredParams->siteMgrDesiredChannel != pParam->content.siteMgrDesiredChannel)
            pSiteMgr->pDesiredParams->siteMgrDesiredChannel = (UINT8)pParam->content.siteMgrDesiredChannel;
        return OK;

    case SITE_MGR_DESIRED_BSSID_PARAM:
            os_memoryCopy(pSiteMgr->hOs, (void *)pSiteMgr->pDesiredParams->siteMgrDesiredBSSID.addr, (void *)pParam->content.siteMgrDesiredBSSID.addr, sizeof(macAddress_t));
#if 0
            WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("Set BSSID = 0x%02x:0x%02x:0x%02x:0x%02x:0x%02x:0x%02x \n", 
                                                                              pSiteMgr->pDesiredParams->siteMgrDesiredBSSID.addr[0],
                                                                              pSiteMgr->pDesiredParams->siteMgrDesiredBSSID.addr[1],
                                                                              pSiteMgr->pDesiredParams->siteMgrDesiredBSSID.addr[2],
                                                                              pSiteMgr->pDesiredParams->siteMgrDesiredBSSID.addr[3],
                                                                              pSiteMgr->pDesiredParams->siteMgrDesiredBSSID.addr[4],
                                                                              pSiteMgr->pDesiredParams->siteMgrDesiredBSSID.addr[5]));
#endif
           return OK;
                


    case SITE_MGR_DESIRED_SSID_PARAM: 

        WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                ("\nSet new SSID=%s (len=%d)  \n",
                                 pParam->content.siteMgrDesiredSSID.ssidString,
                                 pParam->content.siteMgrDesiredSSID.len));

        if (pParam->content.siteMgrDesiredSSID.len > MAX_SSID_LEN)
            return PARAM_VALUE_NOT_VALID;

        os_memoryCopy(pSiteMgr->hOs, &pSiteMgr->pDesiredParams->siteMgrDesiredSSID, &pParam->content.siteMgrDesiredSSID, sizeof(ssid_t));
        /* only add null at the end of the string if the string length is less than 32 bytes and so we have one char left
           TODO: another byte must be added, and the alignment change MUST be tested (esp. in CLI commands with ssid_t */
        if ( MAX_SSID_LEN > pSiteMgr->pDesiredParams->siteMgrDesiredSSID.len )
        {
            pSiteMgr->pDesiredParams->siteMgrDesiredSSID.ssidString[pSiteMgr->pDesiredParams->siteMgrDesiredSSID.len] = '\0';
        }

        /* increase the random IBSS BSSID calculated during init */
        pSiteMgr->ibssBssid.addr[MAC_ADDR_LEN - 1] ++;

        if (utils_isJunkSSID(&pSiteMgr->pDesiredParams->siteMgrDesiredSSID))
        {
            rsn_removedDefKeys(pSiteMgr->hRsn);
        }
        return RE_SCAN_NEEDED;


    case SITE_MGR_DESIRED_BSS_TYPE_PARAM:
         WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                ("\nSet BssType = %d\n", pParam->content.siteMgrDesiredBSSType));
        if (pParam->content.siteMgrDesiredBSSType > BSS_ANY)
            return PARAM_VALUE_NOT_VALID;

        if (pSiteMgr->pDesiredParams->siteMgrDesiredBSSType != pParam->content.siteMgrDesiredBSSType)
        {
            pSiteMgr->pDesiredParams->siteMgrDesiredBSSType = pParam->content.siteMgrDesiredBSSType;

			/* If the new BSS type is NOT Ad_Hoc, We make sure that the rate masks are set to G */
			 if(pSiteMgr->pDesiredParams->siteMgrDesiredBSSType != BSS_INDEPENDENT)

            {
				 pSiteMgr->siteMgrOperationalMode = DOT11_G_MODE;
                 siteMgr_ConfigRate(pSiteMgr);
            }

            /* If the new BSS type is Ad_Hoc, increase the random BSSID calculated during init */
            if(pSiteMgr->pDesiredParams->siteMgrDesiredBSSType == BSS_INDEPENDENT)
            {
                pSiteMgr->ibssBssid.addr[MAC_ADDR_LEN - 1] ++;
            }

            /* go to B_ONLY Mode only if WiFI bit is Set*/
            if (pSiteMgr->pDesiredParams->siteMgrWiFiAdhoc == TRUE)
            {   /* Configuration For AdHoc when using external configuration */
                if(pSiteMgr->pDesiredParams->siteMgrExternalConfiguration == FALSE)
                {
                    siteMgr_externalConfigurationParametersSet(hSiteMgr);
                }
            }
        }

        return OK;

    case SITE_MGR_DESIRED_MODULATION_TYPE_PARAM:
        if ((pParam->content.siteMgrDesiredModulationType < DRV_MODULATION_CCK) ||
            (pParam->content.siteMgrDesiredModulationType > DRV_MODULATION_OFDM))
            return PARAM_VALUE_NOT_VALID;

        if (pSiteMgr->pDesiredParams->siteMgrDesiredModulationType != pParam->content.siteMgrDesiredModulationType)
        {
            pSiteMgr->pDesiredParams->siteMgrDesiredModulationType = pParam->content.siteMgrDesiredModulationType;
            /* means that we are moving from non-pbcc network to pbcc */
            if (pParam->content.siteMgrDesiredModulationType == DRV_MODULATION_PBCC)
                return RE_SCAN_NEEDED;
            return OK;
        }
        return OK;

    case SITE_MGR_BEACON_RECV:
        if (!pPrimarySite)
        {
            return NO_SITE_SELECTED_YET;
        }
        pPrimarySite->beaconRecv = pParam->content.siteMgrBeaconRecv;
        return OK;


    case SITE_MGR_DESIRED_BEACON_INTERVAL_PARAM:
        if (pParam->content.siteMgrDesiredBeaconInterval < SITE_MGR_BEACON_INTERVAL_MIN)
            return PARAM_VALUE_NOT_VALID;

        if (pSiteMgr->pDesiredParams->siteMgrDesiredBeaconInterval != pParam->content.siteMgrDesiredBeaconInterval)
            pSiteMgr->pDesiredParams->siteMgrDesiredBeaconInterval = pParam->content.siteMgrDesiredBeaconInterval;
        return OK;

    case SITE_MGR_DESIRED_PREAMBLE_TYPE_PARAM:
        if ((pParam->content.siteMgrDesiredPreambleType != PREAMBLE_LONG) &&
            (pParam->content.siteMgrDesiredPreambleType != PREAMBLE_SHORT))
            return PARAM_VALUE_NOT_VALID;

        if (pSiteMgr->pDesiredParams->siteMgrDesiredPreambleType != pParam->content.siteMgrDesiredPreambleType)
        {
            pSiteMgr->pDesiredParams->siteMgrDesiredPreambleType = pParam->content.siteMgrDesiredPreambleType;
        }
        return OK;

    case SITE_MGR_DESIRED_SUPPORTED_RATE_SET_PARAM:
        return setSupportedRateSet(pSiteMgr, &(pParam->content.siteMgrDesiredSupportedRateSet));

    case SITE_MGR_DESIRED_DOT11_MODE_PARAM:
        if(pParam->content.siteMgrDot11Mode > DOT11_MAX_MODE)
            return PARAM_VALUE_NOT_VALID;

        if(pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode != pParam->content.siteMgrDot11Mode)
        {
            pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode = pParam->content.siteMgrDot11Mode;

            /* since the dot11ABAmode changed, the STA operational mode should be changed */
            if(pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode == DOT11_DUAL_MODE)
            {
                if(pSiteMgr->pDesiredParams->siteMgrSupportedBand == RADIO_BAND_DUAL)
                {
                    pSiteMgr->siteMgrOperationalMode = DOT11_G_MODE;
                }
                else if(pSiteMgr->pDesiredParams->siteMgrSupportedBand == RADIO_BAND_2_4_GHZ)
                {
                    pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode = DOT11_G_MODE;
                    pSiteMgr->siteMgrOperationalMode = DOT11_G_MODE;
                }
                else
                {
                    pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode = DOT11_G_MODE;
                    pSiteMgr->siteMgrOperationalMode = DOT11_A_MODE;
                }

            }
            else
                pSiteMgr->siteMgrOperationalMode = pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode;

            /* configure HAL with new parameters update rates and select site table */
            pSiteMgr->prevRadioBand = pSiteMgr->radioBand;
            if(pSiteMgr->siteMgrOperationalMode == DOT11_A_MODE)
            {
                pSiteMgr->pSitesMgmtParams->pCurrentSiteTable = (siteTablesParams_t *)&pSiteMgr->pSitesMgmtParams->dot11A_sitesTables;
                pSiteMgr->radioBand = RADIO_BAND_5_0_GHZ;
                slotTime = PHY_SLOT_TIME_SHORT;
            }
            else if(pSiteMgr->siteMgrOperationalMode == DOT11_G_MODE)
            {
                pSiteMgr->pSitesMgmtParams->pCurrentSiteTable = &pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables;
                pSiteMgr->radioBand = RADIO_BAND_2_4_GHZ;
                slotTime = pSiteMgr->pDesiredParams->siteMgrDesiredSlotTime;
            }
            else
            {
                pSiteMgr->pSitesMgmtParams->pCurrentSiteTable = &pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables;
                pSiteMgr->radioBand = RADIO_BAND_2_4_GHZ;
                slotTime = PHY_SLOT_TIME_LONG;
            }

            if(pSiteMgr->prevRadioBand != pSiteMgr->radioBand)
                siteMgr_bandParamsConfig(pSiteMgr, TRUE);

            /* configure HAL */
            whalCtrl_SetRadioBand(pSiteMgr->hHalCtrl, pSiteMgr->radioBand);
            whalCtrl_SetSlotTime(pSiteMgr->hHalCtrl, slotTime);

            /* If the BSS type is Ad_Hoc, increase the random BSSID calculated during init */
            if(pSiteMgr->pDesiredParams->siteMgrDesiredBSSType == BSS_INDEPENDENT)
            {
                pSiteMgr->ibssBssid.addr[MAC_ADDR_LEN - 1] ++;
            }

            /*siteMgr_resetAllSiteTables(pSiteMgr); */
            return RE_SCAN_NEEDED;
        }
        return OK;

    case SITE_MGR_OPERATIONAL_MODE_PARAM:

        if(pParam->content.siteMgrDot11OperationalMode < DOT11_B_MODE ||
            pParam->content.siteMgrDot11OperationalMode > DOT11_G_MODE )
            return PARAM_VALUE_NOT_VALID;

        pSiteMgr->siteMgrOperationalMode = pParam->content.siteMgrDot11OperationalMode;
        break;


    case SITE_MGR_USE_DRAFT_NUM_PARAM:
        if(pParam->content.siteMgrUseDraftNum != DRAFT_5_AND_EARLIER &&
           pParam->content.siteMgrUseDraftNum != DRAFT_6_AND_LATER)
            return PARAM_VALUE_NOT_VALID;

        if(pSiteMgr->pDesiredParams->siteMgrUseDraftNum != pParam->content.siteMgrUseDraftNum)
        {
            pSiteMgr->pDesiredParams->siteMgrUseDraftNum = pParam->content.siteMgrUseDraftNum;
            return RE_SCAN_NEEDED;
        }
        return OK;

    case SITE_MGR_RADIO_BAND_PARAM:
        if((INT8)pParam->content.siteMgrRadioBand < RADIO_BAND_2_4_GHZ ||
           (INT8)pParam->content.siteMgrRadioBand > RADIO_BAND_DUAL )
            return PARAM_VALUE_NOT_VALID;

        pSiteMgr->prevRadioBand = pSiteMgr->radioBand;
        pSiteMgr->radioBand = pParam->content.siteMgrRadioBand;
        if(pSiteMgr->prevRadioBand != pSiteMgr->radioBand)
            siteMgr_bandParamsConfig(pSiteMgr, FALSE);

        break;

    case SITE_MGR_DESIRED_SLOT_TIME_PARAM:
        if(pParam->content.siteMgrSlotTime != PHY_SLOT_TIME_LONG &&
           pParam->content.siteMgrSlotTime != PHY_SLOT_TIME_SHORT)
            return PARAM_VALUE_NOT_VALID;

        if(pSiteMgr->pDesiredParams->siteMgrDesiredSlotTime != pParam->content.siteMgrSlotTime)
        {
            if(pSiteMgr->siteMgrOperationalMode == DOT11_G_MODE)
            {
                pSiteMgr->pDesiredParams->siteMgrDesiredSlotTime = pParam->content.siteMgrSlotTime;
                if(!pPrimarySite)
                    whalCtrl_SetSlotTime(pSiteMgr->hHalCtrl, pSiteMgr->pDesiredParams->siteMgrDesiredSlotTime);
                else if(pPrimarySite->bssType != BSS_INFRASTRUCTURE)
                    whalCtrl_SetSlotTime(pSiteMgr->hHalCtrl, pSiteMgr->pDesiredParams->siteMgrDesiredSlotTime);
            }

        }
        return OK;

    case SITE_MGR_BEACON_FILTER_DESIRED_STATE_PARAM:
        {
            int desiredStateBeforeChange = pSiteMgr->beaconFilterParams.desiredState;

            /* Set the New Desired User request of Beacon Filter */
            pSiteMgr->beaconFilterParams.desiredState = pParam->content.siteMgrDesiredBeaconFilterState;
                
            /* Check if the Desired  mode has changed - If not no need to send the MIB to the FW */
            if ( pSiteMgr->beaconFilterParams.desiredState == desiredStateBeforeChange )
            {
                WLAN_REPORT_DEBUG_CONTROL(pSiteMgr->hReport,
                    ("Beacon Filter already %s" , (TRUE == desiredStateBeforeChange)? "ENABLED":"DISABLED" ) );
                break;
            }

            WLAN_REPORT_DEBUG_CONTROL(pSiteMgr->hReport,
                ("\n New   Beacon Filter Desired State is : %s pSiteMgr->beaconFilterParams.currentState %s\n" , ( TRUE == (pSiteMgr->beaconFilterParams.desiredState) )?"ENABLED":"DISABLED",( TRUE == (pSiteMgr->beaconFilterParams.currentState) )?"ENABLED":"DISABLED"));


            /* Check if the new Desired state is TRUE then Also check the Current State and then if FALSEdo not send the MIB to FW*/
            if ( (TRUE == pSiteMgr->beaconFilterParams.desiredState ) && (FALSE == pSiteMgr->beaconFilterParams.currentState ))
            {
                WLAN_REPORT_DEBUG_CONTROL(pSiteMgr->hReport,
                    ("\n New   Beacon Filter Desired State is TRUE But Current State is DISABLED So the MIBwill be sent Later !!!!"));

            }
            /* In any other cases the User required Beacon Filter Configuration will be sent to FW immediately */
            else
            {
                WLAN_REPORT_DEBUG_CONTROL(pSiteMgr->hReport,
                    ("\n New   Sending  Beacon Filter Desired State To FW !!!!"));

                whalCtrl_SetBeaconFiltering(pSiteMgr->hHalCtrl, pSiteMgr->beaconFilterParams.desiredState, pSiteMgr->beaconFilterParams.numOfStored);
            }


        }

        break;

    case SITE_MGR_DISASSOCIATE_PARAM:
    case SITE_MGR_DEAUTHENTICATE_PARAM:
        if(!pPrimarySite)
            return OK;

        else { /* Set Junk SSID */
            pSiteMgr->pDesiredParams->siteMgrDesiredSSID.len = 4;
            pSiteMgr->pDesiredParams->siteMgrDesiredSSID.ssidString[0]=1;
            pSiteMgr->pDesiredParams->siteMgrDesiredSSID.ssidString[1]=1;
            pSiteMgr->pDesiredParams->siteMgrDesiredSSID.ssidString[2]=1;
            pSiteMgr->pDesiredParams->siteMgrDesiredSSID.ssidString[3]=1;
            return RE_SCAN_NEEDED;
        }


    case SITE_MGR_BSSID_LIST_SCAN_PARAM:
#if 0
        /* TODO - merge fix from WinCE version (and generalize it if time permits) */
        /* Must return NOK in each case the scan is not actually performed */
        if(!pPrimarySite)
        {
            WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                ("Not connected to Network => do the BSSID_LIST_SCAN command\n"));
            smeSm_startScan(pSiteMgr->hSmeSm);
        } 

        param.paramType = RX_DATA_COUNTERS_PARAM;
        rxData_getParam(pSiteMgr->hRxData, &param);

        /* get current received data frames counter */
        currRxPacketsCount = param.content.siteMgrTiWlanCounters.DirectedFramesRecv;

        if((pSiteMgr->rxPacketsCount + 1) < currRxPacketsCount)
        {
            WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                ("Traffic is active now => ignoring the BSSID_LIST_SCAN command\n"));
            return NOK;
        }
        else
        {
            WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                ("Traffic is not active now => do the BSSID_LIST_SCAN command\n"));
            smeSm_startScan(pSiteMgr->hSmeSm);

        }
    }
#endif
        return OK;

    case SITE_MGR_LAST_RX_RATE_PARAM:
        if (pPrimarySite != NULL)
        {
            pPrimarySite->rxRate = pParam->content.ctrlDataCurrentBasicRate;
        }
        break;

    case SITE_MGR_CURRENT_CHANNEL_PARAM:
        if (!pPrimarySite)
        {
            return NO_SITE_SELECTED_YET;
        }
        pPrimarySite->channel = pParam->content.siteMgrCurrentChannel;
        break;

    case SITE_MGR_CURRENT_SIGNAL_PARAM:
        if (!pPrimarySite)
        {
            return NO_SITE_SELECTED_YET;
        }

        pPrimarySite->rssi = pParam->content.siteMgrCurrentSignal.rssi;
        break;

    default:
        WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG, ("Set param, Params is not supported, %d\n", pParam->paramType));
        return PARAM_NOT_SUPPORTED;
    }

    return OK;
}

/***********************************************************************
 *                        siteMgr_getParam
 ***********************************************************************
DESCRIPTION: Site mgr get param function, called by the following:
            -   config mgr in order to get a parameter from the OS abstraction layer.
            -   From inside the dirver

INPUT:      hSiteMgr    -   site mgr handle.
            pParam  -   Pointer to the parameter

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS siteMgr_getParam(TI_HANDLE        hSiteMgr,
                        paramInfo_t     *pParam)
{
    siteMgr_t       *pSiteMgr = (siteMgr_t *)hSiteMgr;
    siteEntry_t     *pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;
    TI_STATUS       status = OK;
    UINT8           siteEntryIndex;
    UINT32          dtimInterval;
    UINT32          delta;
    whalParamInfo_t whalParam;

    switch(pParam->paramType)
    {

    case SITE_MGR_CONFIGURATION_PARAM:
        pParam->content.pSiteMgrConfiguration->Length = sizeof(OS_802_11_CONFIGURATION);
        pParam->content.pSiteMgrConfiguration->ATIMWindow = pSiteMgr->pDesiredParams->siteMgrDesiredAtimWindow;
        pParam->content.pSiteMgrConfiguration->BeaconPeriod = pSiteMgr->pDesiredParams->siteMgrDesiredBeaconInterval;
        pParam->content.pSiteMgrConfiguration->Union.channel =
            Chan2Freq(pSiteMgr->pDesiredParams->siteMgrDesiredChannel);
            /*pSiteMgr->pDesiredParams->siteMgrFreq2ChannelTable[pSiteMgr->pDesiredParams->siteMgrDesiredChannel];*/

        if(pPrimarySite) {
            pParam->content.pSiteMgrConfiguration->FHConfig.DwellTime = pPrimarySite->FHParams.dwellTime;
            pParam->content.pSiteMgrConfiguration->FHConfig.HopPattern = pPrimarySite->FHParams.hopPattern;
            pParam->content.pSiteMgrConfiguration->FHConfig.HopSet = pPrimarySite->FHParams.hopSet;
        }
        else {
            pParam->content.pSiteMgrConfiguration->FHConfig.DwellTime = 0;
            pParam->content.pSiteMgrConfiguration->FHConfig.HopPattern = 0;
            pParam->content.pSiteMgrConfiguration->FHConfig.HopSet = 0;
        }

        pParam->content.pSiteMgrConfiguration->FHConfig.Length = sizeof(OS_802_11_CONFIGURATION_FH);
        break;

    case SITE_MGR_DESIRED_CHANNEL_PARAM:
        pParam->content.siteMgrDesiredChannel = pSiteMgr->pDesiredParams->siteMgrDesiredChannel;
        break;

    case SITE_MGR_DESIRED_BSSID_PARAM:
        os_memoryCopy(pSiteMgr->hOs, &pParam->content.siteMgrDesiredBSSID, &pSiteMgr->pDesiredParams->siteMgrDesiredBSSID, sizeof(macAddress_t));
        break;

    case SITE_MGR_DESIRED_SSID_PARAM:
        os_memoryCopy(pSiteMgr->hOs, &pParam->content.siteMgrDesiredSSID, &pSiteMgr->pDesiredParams->siteMgrDesiredSSID, sizeof(ssid_t));
        break;

    case SITE_MGR_DESIRED_BSS_TYPE_PARAM:
        pParam->content.siteMgrDesiredBSSType = pSiteMgr->pDesiredParams->siteMgrDesiredBSSType;
        break;

    case SITE_MGR_DESIRED_SUPPORTED_RATE_SET_PARAM:
        getSupportedRateSet(pSiteMgr, &(pParam->content.siteMgrDesiredSupportedRateSet));
        break;

    case SITE_MGR_DESIRED_MODULATION_TYPE_PARAM:
        pParam->content.siteMgrDesiredModulationType = pSiteMgr->pDesiredParams->siteMgrDesiredModulationType;
        break;

    case SITE_MGR_DESIRED_BEACON_INTERVAL_PARAM:
        pParam->content.siteMgrDesiredBeaconInterval = pSiteMgr->pDesiredParams->siteMgrDesiredBeaconInterval;
        break;

    case SITE_MGR_DESIRED_PREAMBLE_TYPE_PARAM:
        pParam->content.siteMgrDesiredPreambleType = pSiteMgr->pDesiredParams->siteMgrDesiredPreambleType;
        break;

    case SITE_MGR_CURRENT_RADIO_TYPE_PARAM:
        pParam->content.siteMgrRadioType = pSiteMgr->pDesiredParams->siteMgrRadioValues.siteMgr_radioType;
        break;

    case SITE_MGR_CURRENT_SIGNAL_PARAM:
        if (!pPrimarySite)
        {
            pParam->content.siteMgrCurrentSignal.rssi = 0;
            pParam->content.siteMgrCurrentSignal.snr = 0;
            return NO_SITE_SELECTED_YET;
        }

        pParam->content.siteMgrCurrentSignal.rssi = pPrimarySite->rssi;
        pParam->content.siteMgrCurrentSignal.snr = pPrimarySite->snr;
        break;

    case SITE_MGR_POWER_CONSTRAINT_PARAM:
        if (!pPrimarySite)
        {
            pParam->content.powerConstraint = 0;
            return NO_SITE_SELECTED_YET;
        }
        pParam->content.powerConstraint = pPrimarySite->powerConstraint;
        break;


    case SITE_MGR_DTIM_PERIOD_PARAM:
        if (!pPrimarySite)
        {
            pParam->content.siteMgrDtimPeriod = 0;
            return NO_SITE_SELECTED_YET;
        }
        pParam->content.siteMgrDtimPeriod = pPrimarySite->dtimPeriod;
        break;

    case SITE_MGR_BEACON_RECV:
        if (!pPrimarySite)
        {
            pParam->content.siteMgrBeaconRecv = FALSE;
            return NO_SITE_SELECTED_YET;
        }
        pParam->content.siteMgrBeaconRecv = pPrimarySite->beaconRecv;
        break;


    case SITE_MGR_BEACON_INTERVAL_PARAM:
        if (!pPrimarySite)
        {
            pParam->content.beaconInterval = 0;
            return NO_SITE_SELECTED_YET;
        }
        pParam->content.beaconInterval = pPrimarySite->beaconInterval;
        break;

    case SITE_MGR_AP_TX_POWER_PARAM:
        if (!pPrimarySite)
        {
            pParam->content.APTxPower = 0;
            return NO_SITE_SELECTED_YET;
        }
        pParam->content.APTxPower = pPrimarySite->APTxPower;
        break;

    case SITE_MGR_SITE_CAPABILITY_PARAM:
        if (!pPrimarySite)
        {
            pParam->content.siteMgrSiteCapability = 0;
            return NO_SITE_SELECTED_YET;
        }
        pParam->content.siteMgrSiteCapability = pPrimarySite->capabilities;
        break;

    case SITE_MGR_4X_PARAM:
        if(!pPrimarySite)
            return NO_SITE_SELECTED_YET;

        pParam->content.siteMgrFourxParam = pPrimarySite->fourXsupported;
        break;

    case SITE_MGR_CURRENT_CHANNEL_PARAM:
        if (!pPrimarySite)
        {
            pParam->content.siteMgrCurrentChannel = 0;
            return NO_SITE_SELECTED_YET;
        }
        pParam->content.siteMgrCurrentChannel = pPrimarySite->channel;
        break;

    case SITE_MGR_CURRENT_SSID_PARAM:
        if (!pPrimarySite)
        {
            os_memoryZero(pSiteMgr->hOs, (void *)pParam->content.siteMgrCurrentSSID.ssidString, MAX_SSID_LEN);
            pParam->content.siteMgrCurrentSSID.len = 0;
            return NO_SITE_SELECTED_YET;
        }
        if(pPrimarySite->ssid.len == 0)
            WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG, ("siteMgr_getParam: ssid length is zero, while primarySite is selected \n"));
        os_memoryCopy(pSiteMgr->hOs, &pParam->content.siteMgrCurrentSSID, &pPrimarySite->ssid, sizeof(ssid_t));
        break;


    case SITE_MGR_CURRENT_BSS_TYPE_PARAM:
        if (!pPrimarySite)
        {
            pParam->content.siteMgrCurrentBSSType = pSiteMgr->pDesiredParams->siteMgrDesiredBSSType;
            WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG, ("Trying to get current BSS Type while no site is selected\n"));
            
        }
        else{
            pParam->content.siteMgrCurrentBSSType = pPrimarySite->bssType;
        }

        break;


    case SITE_MGR_CURRENT_RATE_PAIR_PARAM:
        if (!pPrimarySite)
        {
            pParam->content.siteMgrCurrentRateMask.basicRateMask = 0;
            pParam->content.siteMgrCurrentRateMask.supportedRateMask = 0;
            return NO_SITE_SELECTED_YET;
        }
        pParam->content.siteMgrCurrentRateMask.basicRateMask = pSiteMgr->pDesiredParams->siteMgrMatchedBasicRateMask;
        pParam->content.siteMgrCurrentRateMask.supportedRateMask = pSiteMgr->pDesiredParams->siteMgrMatchedSuppRateMask;
        WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                ("SITE_MGR: bitmapBasicPrimary= 0x%X,bitMapBasicDesired = 0x%X,bitMapSuppPrimary = 0x%X, bitMapSuppDesired = 0x%X\n",
                                pPrimarySite->rateMask.basicRateMask,pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.basicRateMask,
                                pPrimarySite->rateMask.supportedRateMask,pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.supportedRateMask));

        break;

    case SITE_MGR_CURRENT_MODULATION_TYPE_PARAM:
        if (!pPrimarySite)
        {
            pParam->content.siteMgrCurrentModulationType = DRV_MODULATION_NONE;
            return NO_SITE_SELECTED_YET;
        }
        pParam->content.siteMgrCurrentModulationType = pSiteMgr->chosenModulation;
        break;

    case SITE_MGR_NEXT_DTIM_TIME_STAMP_PARAM:
        if (!pPrimarySite)
        {
            pParam->content.siteMgrNextDtimTimeStamp = 0;
            return NO_SITE_SELECTED_YET;
        }
        /* Convert TBTT to msec (*1024/1000) */
        dtimInterval = (UINT32)pPrimarySite->beaconInterval * 1024 *
             (UINT32)pPrimarySite->dtimPeriod / 1000 ;
        delta = os_timeStampMs(pSiteMgr->hOs) - pPrimarySite->dtimTimeStamp;
        if(delta < dtimInterval)
            delta = 0;
        pParam->content.siteMgrNextDtimTimeStamp = pPrimarySite->dtimTimeStamp + dtimInterval + delta;
        WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                ("Get DTIM - dtimTimeStamp=%d, dtimInterval=%d, delta=%d\n",
                        pPrimarySite->dtimTimeStamp, dtimInterval, delta));
        break;

    case SITE_MGR_DESIRED_SLOT_TIME_PARAM:
        pParam->content.siteMgrSlotTime = pSiteMgr->pDesiredParams->siteMgrDesiredSlotTime;
        break;

    case SITE_MGR_CURRENT_SLOT_TIME_PARAM:

        if(pSiteMgr->siteMgrOperationalMode == DOT11_G_MODE)
        {
            if(!pPrimarySite)
                pParam->content.siteMgrSlotTime = pSiteMgr->pDesiredParams->siteMgrDesiredSlotTime;
            else
                pParam->content.siteMgrSlotTime = pPrimarySite->currentSlotTime;
        }
        else if(pSiteMgr->siteMgrOperationalMode == DOT11_A_MODE)
            pParam->content.siteMgrSlotTime = PHY_SLOT_TIME_SHORT;
        else
            pParam->content.siteMgrSlotTime = PHY_SLOT_TIME_LONG;

        break;

    case SITE_MGR_BSSID_LIST_PARAM:
        if (pParam->paramLength == 0)
        {

            calculateBssidListSize(pSiteMgr, &pParam->paramLength, FALSE);

            return NOK;
        } 
        else 
        {
            status = getBssidList(pSiteMgr, pParam->content.pSiteMgrBssidList, &pParam->paramLength, FALSE);
        }
        break;

    case SITE_MGR_BSSID_FULL_LIST_PARAM:
        if (pParam->paramLength == 0)
        {

            calculateBssidListSize(pSiteMgr, &pParam->paramLength, TRUE);

            return NOK;
        } 
        else 
        {
            status = getBssidList(pSiteMgr, pParam->content.pSiteMgrBssidList, &pParam->paramLength, TRUE);
        }
        break;

    case SITE_MGR_LAST_BEACON_BUF_PARAM:
        if (pPrimarySite != NULL)
        {
            if (pPrimarySite->probeRecv)
            {
                pParam->content.siteMgrLastBeacon.isBeacon = FALSE;
                pParam->content.siteMgrLastBeacon.bufLength = pPrimarySite->probeRespLength;
                pParam->content.siteMgrLastBeacon.buffer = pPrimarySite->probeRespBuffer;
            }
            else
            {
                pParam->content.siteMgrLastBeacon.isBeacon = TRUE;
                pParam->content.siteMgrLastBeacon.bufLength = pPrimarySite->beaconLength;
                pParam->content.siteMgrLastBeacon.buffer = pPrimarySite->beaconBuffer;
            }
        }
        break;

    case SITE_MGR_BEACON_FILTER_DESIRED_STATE_PARAM:
        {
            if ( NULL != pSiteMgr )
            {
                pParam->content.siteMgrDesiredBeaconFilterState = pSiteMgr->beaconFilterParams.desiredState;
            }
            else
            {
                WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG ,("pSite = NULL ! No info available"));
            }
        }
        break;

    case SITE_MGR_GET_SELECTED_BSSID_INFO:
        getPrimarySiteDesc(pSiteMgr, pParam->content.pSiteMgrPrimarySiteDesc, FALSE);
        break;

    case SITE_MGR_PRIMARY_SITE_PARAM:
       status = getPrimaryBssid(pSiteMgr, (OS_802_11_BSSID_EX *)pParam->content.pSiteMgrSelectedSiteInfo, &pParam->paramLength);
       break;


    case SITE_MGR_TI_WLAN_COUNTERS_PARAM:
        pParam->paramType = RX_DATA_COUNTERS_PARAM;
        rxData_getParam(pSiteMgr->hRxData, pParam);

        whalParam.paramType = HAL_CTRL_COUNTERS_PARAM;
        whalCtrl_GetParam(pSiteMgr->hHalCtrl, &whalParam);
        pParam->content.siteMgrTiWlanCounters.RecvNoBuffer = whalParam.content.halCtrlCounters.RecvNoBuffer;
        pParam->content.siteMgrTiWlanCounters.FragmentsRecv = whalParam.content.halCtrlCounters.FragmentsRecv;
        pParam->content.siteMgrTiWlanCounters.FrameDuplicates = whalParam.content.halCtrlCounters.FrameDuplicates;
        pParam->content.siteMgrTiWlanCounters.FcsErrors = whalParam.content.halCtrlCounters.FcsErrors;
        pParam->content.siteMgrTiWlanCounters.RecvError = whalParam.content.halCtrlCounters.RecvError;

        pParam->paramType = AUTH_COUNTERS_PARAM;
        auth_getParam(pSiteMgr->hAuth, pParam);
        
        pParam->paramType = ASSOC_COUNTERS_PARAM;
        assoc_getParam(pSiteMgr->hAssoc, pParam);        
        pParam->content.siteMgrTiWlanCounters.BeaconsXmit = pSiteMgr->beaconSentCount;
        break;
    
    case SITE_MGR_EEPROM_VERSION_PARAM:
        os_memoryCopy(pSiteMgr->hOs, &(pParam->content.siteMgrEEpromVersion), &(pSiteMgr->pDesiredParams->siteMgrEEpromVersion), sizeof(e2Version_t));
        break;

    case SITE_MGR_FIRMWARE_VERSION_PARAM:
        os_memoryCopy(pSiteMgr->hOs, &(pParam->content.siteMgrFwVersion), &(pSiteMgr->pDesiredParams->siteMgrFwVersion), FW_VERSION_LEN);
        break;

    case SITE_MGR_DESIRED_TX_RATE_PARAM:
        pParam->content.siteMgrDesiredTxRate = hostToUtilityRate(pSiteMgr->pDesiredParams->siteMgrCurrentDesiredTxRate);
        break;

    case SITE_MGR_CURRENT_TX_RATE_PARAM:
        /* Collect txAttr from control */
        {   txData_attr_t txAttr;
            
            ctrlData_getTxAttributes(pSiteMgr->hCtrlData, TX_DATA_DATA_MSDU, &txAttr, QOS_AC_BE);

            pParam->content.siteMgrCurrentTxRate = hostToUtilityRate( txAttr.Rate );
        }
        break;

    case SITE_MGR_DESIRED_DOT11_MODE_PARAM:
        pParam->content.siteMgrDot11Mode = pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode;
        break;

	case SITE_MGR_NETWORK_TYPE_IN_USE:
		if (pPrimarySite)
		{ /* Connected - return the current mode */
			pParam->content.siteMgrDot11Mode = pSiteMgr->siteMgrOperationalMode;
		}
		else
		{ /* Disconnected - return the desired mode */
			pParam->content.siteMgrDot11Mode = pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode;
		}
        break;


    case SITE_MGR_OPERATIONAL_MODE_PARAM:
        pParam->content.siteMgrDot11OperationalMode = pSiteMgr->siteMgrOperationalMode;
        break;

    case SITE_MGR_USE_DRAFT_NUM_PARAM:
        pParam->content.siteMgrUseDraftNum = pSiteMgr->pDesiredParams->siteMgrUseDraftNum;
        break;

    case SITE_MGR_RADIO_BAND_PARAM:
        pParam->content.siteMgrRadioBand = pSiteMgr->radioBand;
        break;

    case SITE_MGR_CURRENT_PREAMBLE_TYPE_PARAM:
        if (!pPrimarySite)
            return NO_SITE_SELECTED_YET;

        pParam->content.siteMgrCurrentPreambleType = pPrimarySite->currentPreambleType;
        break;

    case SITE_MGR_CURRENT_BSSID_PARAM:
        if (pPrimarySite != NULL)
        {
            os_memoryCopy(pSiteMgr->hOs, (void *)pParam->content.siteMgrDesiredBSSID.addr, (void *)pPrimarySite->bssid.addr, MAC_ADDR_LEN);
        }
        break;

    case SITE_MGR_LAST_RX_RATE_PARAM:
        if (pPrimarySite != NULL)
        {
            pParam->content.ctrlDataCurrentBasicRate = pPrimarySite->rxRate;
        }
        break;

    case SITE_MGR_PREV_SITE_BSSID_PARAM:
        if (pSiteMgr->pSitesMgmtParams->pPrevPrimarySite==NULL)
        {
            return NOK;
        }
        os_memoryCopy(pSiteMgr->hOs, (void *)pParam->content.siteMgrDesiredBSSID.addr, (void *)pSiteMgr->pSitesMgmtParams->pPrevPrimarySite->bssid.addr, MAC_ADDR_LEN);
        break;

    case SITE_MGR_PREV_SITE_SSID_PARAM:
        if (pSiteMgr->pSitesMgmtParams->pPrevPrimarySite==NULL)
        {
            return NOK;
        }
        pParam->content.siteMgrDesiredSSID.len = pSiteMgr->pSitesMgmtParams->pPrevPrimarySite->ssid.len;
        os_memoryCopy(pSiteMgr->hOs, (void *)pParam->content.siteMgrDesiredSSID.ssidString, (void *)pSiteMgr->pSitesMgmtParams->pPrevPrimarySite->ssid.ssidString, pSiteMgr->pSitesMgmtParams->pPrevPrimarySite->ssid.len);
        break;

    case SITE_MGR_PREV_SITE_CHANNEL_PARAM:
        if (pSiteMgr->pSitesMgmtParams->pPrevPrimarySite==NULL)
        {
            return NOK;
        }
        pParam->content.siteMgrDesiredChannel = pSiteMgr->pSitesMgmtParams->pPrevPrimarySite->channel;
        break;

    case SITE_MGR_SITE_ENTRY_BY_INDEX:
        siteEntryIndex = pParam->content.siteMgrIndexOfDesiredSiteEntry;
        if(siteEntryIndex >= MAX_SITES_BG_BAND)
        {
            return NOK;
        }
        pParam->content.pSiteMgrDesiredSiteEntry =
            (UINT8*)(&(pSiteMgr->pSitesMgmtParams->pCurrentSiteTable->siteTable[siteEntryIndex]));
        break;

    case SITE_MGR_CUR_NUM_OF_SITES:
        pParam->content.siteMgrNumberOfSites = pSiteMgr->pSitesMgmtParams->pCurrentSiteTable->numOfSites;
        break;

    case SITE_MGR_CURRENT_TSF_TIME_STAMP:
        os_memoryCopy(pSiteMgr->hOs, pParam->content.siteMgrCurrentTsfTimeStamp,
                      pSiteMgr->pSitesMgmtParams->pPrimarySite->tsfTimeStamp,
                      TIME_STAMP_LEN);
        break;

    case SITE_MGR_GET_AP_QOS_CAPABILITIES:
       if (!pPrimarySite)
        {
            pParam->content.qosApCapabilities.uQOSFlag = 0;
            pParam->content.qosApCapabilities.uAPSDFlag = 0;
            return NOT_CONNECTED;
        }
       pParam->content.qosApCapabilities.uQOSFlag = pPrimarySite->WMESupported;
       pParam->content.qosApCapabilities.uAPSDFlag = pPrimarySite->APSDSupport;
        
        break;

    default:
        {
            UINT8* ptr = NULL;
            WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG, ("Get param, Params is not supported, %d\n", pParam->paramType));
            *ptr=3;
        }

        return PARAM_NOT_SUPPORTED;
    }

    return status;
}


/***********************************************************************
 *                        calculateHwGenTxRate
 ***********************************************************************
DESCRIPTION: Service routine, which calculates the HW generated frames rate
             to be used.

INPUT:      basicBitMap - bit map of basic rates
            mode        - operating mode

OUTPUT:

RETURN:     rate

************************************************************************/
void calculateHwGenTxRate(siteMgr_t *pSiteMgr, rate_e *rate)
{

    UINT32 rateBitMap = 0;
    modulationType_e modulation = DRV_MODULATION_NONE; /* Not used */
    mgmtCtrlTxRateOption_e mgmtCtrlTxRateOption = pSiteMgr->pDesiredParams->siteMgrRegstryDesiredMgmtCtrlTxRateOption;
    UINT32 mgmtCtrlDesiredTxRateBitMap = ((0x01) << (pSiteMgr->pDesiredParams->siteMgrRegstryDesiredMgmtCtrlTxRate -1));

    
    /* ratebitmap is the matched basic rate from the STA and AP */
    if (pSiteMgr->pSitesMgmtParams->pPrimarySite->channel == SPECIAL_BG_CHANNEL)
    {
        rateBitMap = (UINT16)getBasicRateMaskForSpecialBGchannel() ;
    }
    else
    {
        rateBitMap = pSiteMgr->pDesiredParams->siteMgrMatchedBasicRateMask;
    }

    /* Check if the user request specific Rate */
    if(mgmtCtrlTxRateOption == SPECIFIC_TX_RATE)
    {
        /* If the required rate is included in the matched basic rate, this specific rate is the only one to use */
        if(rateBitMap & mgmtCtrlDesiredTxRateBitMap)
        {
            rateBitMap = mgmtCtrlDesiredTxRateBitMap;
        }
        else
        {
            /* If the specific required Tx Rate is not in the basic rate => Use Max basic rate */
            mgmtCtrlTxRateOption = MAX_BASIC_TX_RATE;
        }
    }

    /* Now calculate HW Tx rate according to the rate bitmap */
    if(mgmtCtrlTxRateOption == MAX_BASIC_TX_RATE)
        getMaxRate(rateBitMap,rate,&modulation,pSiteMgr->siteMgrOperationalMode);
    else
    {
        /* If we are here , means: The mgmtCtrlTxRateOption is either MIN_BASIC_TX_RATE or SPECIFIC_TX_RATE. */
        getMinRate(rateBitMap,rate,&modulation,pSiteMgr->siteMgrOperationalMode);

    }
}




/***********************************************************************
 *                        siteMgr_join
 ***********************************************************************
DESCRIPTION: Called by the connection state machine in order to join a BSS.
                -   If the BSS is infrastructure, sets a NULL data template to the HAL
                -   If the BSS is IBSS, sets a probe response & beacon template to the HAL
            Call the HAL with the join parameters


INPUT:      hSiteMgr    -   site mgr handle.
            JoinCompleteCB - join command complete callback function ptr
            CB_handle - handle to pass to callback function

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS siteMgr_join(TI_HANDLE    hSiteMgr)
{
    siteMgr_t               *pSiteMgr = (siteMgr_t *)hSiteMgr;
    siteEntry_t             *pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;
    rate_e                  rate;
	paramInfoPartial_t      param;

    if (pPrimarySite == NULL)
    {
        WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG, ("Join BSS, Primary Site is NULL\n"));
        return OK;
    }

    /* Configure the system according to parameters of Primary Site */
    systemConfig(pSiteMgr);

    joinParams.bssType = pPrimarySite->bssType;
    joinParams.beaconInterval = pPrimarySite->beaconInterval;
    joinParams.dtimInterval = pPrimarySite->dtimPeriod;
    joinParams.pBSSID = (UINT8 *)&(pPrimarySite->bssid.addr);
    joinParams.pSSID = (UINT8 *)&(pPrimarySite->ssid.ssidString);
    joinParams.ssidLength = pPrimarySite->ssid.len;

    joinParams.channel = pPrimarySite->channel;

        if (joinParams.channel == SPECIAL_BG_CHANNEL)
    {
         joinParams.basicRateSet     = (UINT16)getBasicRateMaskForSpecialBGchannel() ;
         joinParams.supportedRateSet = (UINT16)getSupportedRateMaskForSpecialBGchannel() ;
    }
    else /* != SPECIAL_BG_CHANNEL */
    {
        joinParams.basicRateSet = (UINT16)pSiteMgr->pDesiredParams->siteMgrMatchedBasicRateMask;
        joinParams.supportedRateSet = (UINT16)pSiteMgr->pDesiredParams->siteMgrMatchedSuppRateMask;
    }
    calculateHwGenTxRate(pSiteMgr, &rate);
    joinParams.hwGenCtrlTxRate = rate;

    param.paramType = CTRL_DATA_CURRENT_PREAMBLE_TYPE_PARAM;
    ctrlData_getParamPartial(pSiteMgr->hCtrlData, &param);
    joinParams.preamble = param.content.ctrlDataCurrentPreambleType;
     /*set the preamble before the join*/
    whalCtrl_SetPreamble(pSiteMgr->hHalCtrl, joinParams.preamble);
    
    

/*  
 * Set the radio band and the HW management Tx rate according to operational mode.
 * The HW management frames includes Beacon and Probe-Response (in IBSS). 
 */
    if(pSiteMgr->siteMgrOperationalMode == DOT11_A_MODE)
    {
        joinParams.radioBand = RADIO_BAND_5_0_GHZ;
        joinParams.hwGenMgmtTxRate = DRV_RATE_6M;           
    }
    else
    {
        joinParams.radioBand = RADIO_BAND_2_4_GHZ;
        joinParams.hwGenMgmtTxRate = DRV_RATE_2M;           
    }

    /* Now, Set templates to the HAL */
    if (pPrimarySite->bssType == BSS_INDEPENDENT)
    {
        templateStruct.pTemplate = (UINT8 *)&probeRspTemplate;
        templateStruct.templateType = PROBE_RESPONSE_TEMPLATE;
        buildProbeRspTemplate(pSiteMgr, &templateStruct);
        whalCtrl_SetTemplate(pSiteMgr->hHalCtrl, &templateStruct);

        /* We don't have to build a beacon template, because it is equal to probe response,
        we only have to change the frame sup type */
        probeRspTemplate.hdr.fc = ENDIAN_HANDLE_WORD(DOT11_FC_BEACON);
        templateStruct.templateType = BEACON_TEMPLATE;
        whalCtrl_SetTemplate(pSiteMgr->hHalCtrl, &templateStruct);
    }
    else
    {
        templateStruct.pTemplate = (UINT8 *)&nullDataTemplate;
        templateStruct.templateType = NULL_DATA_TEMPLATE;
        buildNullTemplate(pSiteMgr, &templateStruct);
        whalCtrl_SetTemplate(pSiteMgr->hHalCtrl, &templateStruct);

        /* Send PsPoll template to HAL */
        templateStruct.pTemplate = (UINT8 *)&psPollTemplate;
        templateStruct.templateType = PS_POLL_TEMPLATE;
        buildPsPollTemplate(pSiteMgr, &templateStruct);
        whalCtrl_SetTemplate(pSiteMgr->hHalCtrl, &templateStruct);
            
        /* Set QOS Null data template to the firmware.
            Note:  the AC to use with this template may change in QoS-manager. */
        templateStruct.pTemplate = (UINT8 *)&QosNullDataTemplate;
        templateStruct.templateType = QOS_NULL_DATA_TEMPLATE;
        buildQosNullDataTemplate(pSiteMgr, &templateStruct, 0);
        whalCtrl_SetTemplate(pSiteMgr->hHalCtrl, &templateStruct);
    }

    /* Reset the Tx Power Control adjustment in RegulatoryDomain */
    siteMgr_setTemporaryTxPower(pSiteMgr, FALSE);     
    
    /*
    clear the beacon receive flag to indicates when receive a beacon after the join command.
    need for PowerMgr_startPS() - must be called only after join command and then only after
    receiving beacon of the primary site!
    */
    pPrimarySite->beaconReceiveAfterJoin = FALSE;
     
    /* Stop Tx till geting Join Event complete */
    txData_disableTransmission(pSiteMgr->hTxData, DISABLE_IMMEDIATELY);
    
    /* This is the forbidden period to enable the current state from the cli, even if desired state is TRUE */
    /* Also This is set to FALSE since the First Beaqcin shall be received for the Power Manager */
    pSiteMgr->beaconFilterParams.currentState = FALSE;
    whalCtrl_SetBeaconFiltering(pSiteMgr->hHalCtrl, FALSE, pSiteMgr->beaconFilterParams.numOfStored);
    
    WLAN_REPORT_INFORMATION(pSiteMgr->hReport , SITE_MGR_MODULE_LOG , ("--          Beacon Filter Active Mode  Disable Beacon Filtering !!!  --\n"));

    return (TI_STATUS)whalCtrl_JoinBss (((siteMgr_t *)hSiteMgr)->hHalCtrl, &joinParams);
}

/***********************************************************************
 *                        siteMgr_forceInfraJoin
 ***********************************************************************
DESCRIPTION: Called by the connection state machine in order to join a BSS.
                -   If the BSS is infrastructure, sets a NULL data template to the HAL
                -   If the BSS is IBSS, sets a probe response & beacon template to the HAL
            Call the HAL with the join parameters


INPUT:      hSiteMgr    -   site mgr handle.
            JoinCompleteCB - join command complete callback function ptr
            CB_handle - handle to pass to callback function

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS siteMgr_forceInfraJoin(TI_HANDLE    hSiteMgr)
{
    siteMgr_t               *pSiteMgr = (siteMgr_t *)hSiteMgr;
    whalCtrl_joinBss_t      joinParams;
    UINT8                   junkBSSID[MAC_ADDR_LEN] = {0xAA,0xAA,0xAA,0xAA,0xAA,0xAA};
    UINT8                   junkSSID[1] = {0x01};

    WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG, ("Force INFRA Join\n"));

    joinParams.bssType = BSS_INFRASTRUCTURE;
    joinParams.beaconInterval = 200;
    joinParams.dtimInterval = 3;
    joinParams.pBSSID = (UINT8 *)junkBSSID;
    joinParams.pSSID = (UINT8 *)junkSSID;
    joinParams.ssidLength = 1;

    joinParams.channel = 11;

    joinParams.basicRateSet = (UINT16)pSiteMgr->pDesiredParams->siteMgrMatchedBasicRateMask;
    joinParams.supportedRateSet = (UINT16)pSiteMgr->pDesiredParams->siteMgrMatchedSuppRateMask;
    
    joinParams.hwGenCtrlTxRate = DRV_RATE_2M;
    joinParams.hwGenMgmtTxRate = DRV_RATE_2M;        
    joinParams.preamble = PREAMBLE_LONG;

    if(pSiteMgr->siteMgrOperationalMode == DOT11_A_MODE)
        joinParams.radioBand = RADIO_BAND_5_0_GHZ;
    else
        joinParams.radioBand = RADIO_BAND_2_4_GHZ;

    /* Stop Tx till geting Join Event complete */
    txData_disableTransmission(pSiteMgr->hTxData, DISABLE_IMMEDIATELY);
    return (TI_STATUS)whalCtrl_JoinBss (((siteMgr_t *)hSiteMgr)->hHalCtrl, &joinParams);
}

/***********************************************************************
 *                        siteMgr_disJoin
 ***********************************************************************
DESCRIPTION: Called by the connection state machine in order to dis join a BSS.


INPUT:      hSiteMgr    -   site mgr handle.

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS siteMgr_disJoin(TI_HANDLE hSiteMgr)
{
#if 0
    siteMgr_t               *pSiteMgr = (siteMgr_t *)hSiteMgr;
    whalCtrl_joinBss_t      joinParams;
    macAddress_t            dummyMac = {0xDE, 0xAD, 0xBE, 0xEF, 0xDE, 0xAD};
    ssid_t                  dummySsid;

    joinParams.bssType = BSS_INFRASTRUCTURE;
    joinParams.beaconInterval = SITE_MGR_BEACON_INTERVAL_DEF;
    joinParams.dtimInterval = 1;
    joinParams.channel = SITE_MGR_CHANNEL_DEF;

    joinParams.pBSSID = (UINT8 *)&(dummyMac.addr);

    dummySsid.len = 2;
    os_memoryCopy(pSiteMgr->hOs, dummySsid.ssidString, "TI_WLAN", 7);
    joinParams.pSSID = (UINT8 *)&(dummySsid.ssidString);
    joinParams.ssidLength = dummySsid.len;

    joinParams.basicRateSet = buildRatesBitMap(pSiteMgr, SITE_MGR_DEF_RATE_SET_MAX_BASIC_DEF);
    joinParams.supportedRateSet = buildRatesBitMap(pSiteMgr, SITE_MGR_DEF_RATE_SET_MAX_ACTIVE_DEF);

    return whalCtrl_JoinBss (((siteMgr_t *)hSiteMgr)->hHalCtrl, &joinParams);
#else
    return OK;
#endif
}

/***********************************************************************
 *                        siteMgr_removeSelfSite
 ***********************************************************************
DESCRIPTION: Called by the Self connection state machine in order to remove the self site from the site table.
                Remove the site entry form the table and reset the primary site pointer


INPUT:      hSiteMgr    -   site mgr handle.

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS siteMgr_removeSelfSite(TI_HANDLE  hSiteMgr)
{
    siteMgr_t           *pSiteMgr  = (siteMgr_t *)hSiteMgr;
    siteTablesParams_t  *currTable = pSiteMgr->pSitesMgmtParams->pCurrentSiteTable;

    if(pSiteMgr->pSitesMgmtParams->pPrimarySite == NULL)
    {
        WLAN_REPORT_WARNING(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("Remove self site Failure, pointer is NULL\n\n"));
        return OK;
    }

    if(pSiteMgr->pSitesMgmtParams->pPrimarySite->siteType != SITE_SELF)
    {
        WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("Remove self site Failure, site is not self\n\n"));
        return OK;
    }

    removeSiteEntry(pSiteMgr, currTable, pSiteMgr->pSitesMgmtParams->pPrimarySite);
    pSiteMgr->pSitesMgmtParams->pPrimarySite = NULL;

    return OK;
}

/***********************************************************************
 *                        siteMgr_updateSite
 ***********************************************************************
DESCRIPTION: Called by the MLME parser upon receiving a beacon or probe response.
            Performs the following:
                -   Insert the site entry into the site hash table
                -   Update the site information in the site table
                -   If the site is the primary site, it handles the PBCC algorithm if needed
                -   If the site is NULL (means it is the first frame received from this site)
                    we update the site type to be regular
                -   If the site type is self, we inform the self connection SM
                    that another station joined the network we created


INPUT:      hSiteMgr    -   site mgr handle.
            bssid       -   BSSID received
            pFrameInfo  -   Frame content after the parsing
            rxChannel   -   The channel on which frame was received
            band        -   Band on which frame was received
            measuring   -   Determines whether the beacon or probe response
                            has been received while a beacon measurement
                            took place

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS siteMgr_updateSite(TI_HANDLE          hSiteMgr,
                          macAddress_t      *bssid,
                          mlmeFrameInfo_t   *pFrameInfo,
                          UINT8             rxChannel,
                          radioBand_e       band,
                          BOOL              measuring)
{
    siteEntry_t *pSite;
    siteMgr_t   *pSiteMgr = (siteMgr_t *)hSiteMgr;
    paramInfo_t param;


    /* The following is not required, since the scanCncn is responsible to check
        the channels validity before scanning.
        The problem it caused was that when 802.11d is enabled, 
        channels that are valid for Passive only, will not be updated.*/
    /*if (isChannelSupprted(pSiteMgr->hRegulatoryDomain , rxChannel) == FALSE)
    {
        WLAN_REPORT_WARNING(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
            ("Channel ERROR - try to register a site that its channel (=%d) isn't in the regulatory domain.\n\
            registration ABORTED!!!",
            rxChannel));
        return NOK;
    }*/


    pSite = findAndInsertSiteEntry(pSiteMgr, bssid, band);



    if (pSite == NULL)
    {
        WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("Site Update failure, table is full, bssid: %X-%X-%X-%X-%X-%X\n", bssid->addr[0], bssid->addr[1], bssid->addr[2], bssid->addr[3], bssid->addr[4], bssid->addr[5]));
        return OK;
    }

    updateSiteInfo(pSiteMgr, pFrameInfo, pSite, rxChannel);

    if (measuring != FALSE)
    {
        pSite->detectedWhileMeasuring = TRUE;
    }

    switch(pSite->siteType)
    {
    case SITE_PRIMARY:
        WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_UPDATE_MODULE_LOG,  ("PRIMARY site updated, bssid: %X-%X-%X-%X-%X-%X\n\n", bssid->addr[0], bssid->addr[1], bssid->addr[2], bssid->addr[3], bssid->addr[4], bssid->addr[5]));
        if (pSiteMgr->pSitesMgmtParams->pPrimarySite == NULL)
        {
            WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG, ("siteMgr_updateSite: Primary Site Is NULL\n"));
            pSite->siteType = SITE_REGULAR;
            break;
        }
        /* Now, if the following is TRUE we perform the PBCC algorithm: */
        /* If the BSS type is infrastructure, &&
            The chosen modulation is PBCC &&
            The beacon modulation is not NONE &&
            The current data modulation is different than the beacon modulation. */
        if ((pSite->bssType == BSS_INFRASTRUCTURE) &&
            (pSiteMgr->chosenModulation == DRV_MODULATION_PBCC) &&
            (pSite->beaconModulation != DRV_MODULATION_NONE) &&
            (pSiteMgr->currentDataModulation != pSite->beaconModulation))
        {
            pSiteMgr->currentDataModulation = pSite->beaconModulation;
            pbccAlgorithm(pSiteMgr);
        }

        /* Now handle the slot time, first check if the slot time changed since the last
           setting to the HAL ,and if yes set the new value */
        if((pSiteMgr->siteMgrOperationalMode == DOT11_G_MODE) &&
           (pSite->bssType == BSS_INFRASTRUCTURE))
        {
            if (pSite->currentSlotTime != pSite->newSlotTime)
            {
                pSite->currentSlotTime = pSite->newSlotTime;
                whalCtrl_SetSlotTime(pSiteMgr->hHalCtrl, pSite->currentSlotTime);
            }
        }

        /* Now handle the current protection status */
        if((pSiteMgr->siteMgrOperationalMode == DOT11_G_MODE) &&
           (pSite->bssType == BSS_INFRASTRUCTURE))
        {
            param.paramType = CTRL_DATA_CURRENT_PROTECTION_STATUS_PARAM;
            param.content.ctrlDataProtectionEnabled = pSite->useProtection;
            ctrlData_setParam(pSiteMgr->hCtrlData, &param);
        }

        /* Now handle the current preamble type,
           if desired preamble type is long, the ctrl data param should not be changed */
        if((pSiteMgr->siteMgrOperationalMode == DOT11_G_MODE) &&
           (pSite->bssType == BSS_INFRASTRUCTURE) &&
           (pSiteMgr->pDesiredParams->siteMgrDesiredPreambleType != PREAMBLE_LONG))
        {
            param.paramType = CTRL_DATA_CURRENT_PREAMBLE_TYPE_PARAM;
            if((pSite->preambleAssRspCap == PREAMBLE_LONG) ||
               (pSite->barkerPreambleType == PREAMBLE_LONG))
                  {
                param.content.ctrlDataCurrentPreambleType = PREAMBLE_LONG;
            }
            else
                param.content.ctrlDataCurrentPreambleType = PREAMBLE_SHORT;

            ctrlData_setParam(pSiteMgr->hCtrlData, &param);
             
        }

        /*
         * NOTE: THIS MUST BE AFTER SETTING PREAMBLE TYPE 
         */
        if (pSiteMgr->chosenModulation != DRV_MODULATION_PBCC)
        {            
            if (pSiteMgr->pDesiredParams->siteMgrCurrentDesiredTxRate == DRV_RATE_AUTO)
            {
                param.paramType = CTRL_DATA_RATE_CONTROL_ENABLE_PARAM;
                param.content.ctrlDataRateControlEnable = TRUE;
                ctrlData_setParam(pSiteMgr->hCtrlData, &param);
            }
            else
            {
                param.paramType = CTRL_DATA_RATE_CONTROL_ENABLE_PARAM;
                param.content.ctrlDataRateControlEnable = FALSE;
                ctrlData_setParam(pSiteMgr->hCtrlData, &param);
            }

        }
            break;

    case SITE_NULL:
        pSite->siteType = SITE_REGULAR;
        WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_UPDATE_MODULE_LOG,  ("REGULAR site added, bssid: %X-%X-%X-%X-%X-%X\n\n", bssid->addr[0], bssid->addr[1], bssid->addr[2], bssid->addr[3], bssid->addr[4], bssid->addr[5]));
        break;

    case SITE_SELF:
        pSite->siteType = SITE_PRIMARY;
        WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("SELF ----> PRIMARY site , bssid: %X-%X-%X-%X-%X-%X\n\n", bssid->addr[0], bssid->addr[1], bssid->addr[2], bssid->addr[3], bssid->addr[4], bssid->addr[5]));
        conn_ibssStaJoined(pSiteMgr->hConn);
        break;

    case SITE_REGULAR:
        WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_UPDATE_MODULE_LOG,  ("REGULAR site updated, bssid: %X-%X-%X-%X-%X-%X\n\n", bssid->addr[0], bssid->addr[1], bssid->addr[2], bssid->addr[3], bssid->addr[4], bssid->addr[5]));
        break;

    default:
        WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("Setting site type failure, bssid: %X-%X-%X-%X-%X-%X\n\n", bssid->addr[0], bssid->addr[1], bssid->addr[2], bssid->addr[3], bssid->addr[4], bssid->addr[5]));
        break;
    }

    return OK;
}

/***********************************************************************
 *                        siteMgr_start
 ***********************************************************************
DESCRIPTION: Called by the SME SM in order to start the aging timer


INPUT:      hSiteMgr    -   site mgr handle.

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS siteMgr_start(TI_HANDLE   hSiteMgr)
{
    siteMgr_t       *pSiteMgr = (siteMgr_t *)hSiteMgr;

    /* update timestamp each time aging started (needed for quiet scan) */
    if(pSiteMgr->pSitesMgmtParams->pPrimarySite)
        pSiteMgr->pSitesMgmtParams->pPrimarySite->localTimeStamp = os_timeStampMs(pSiteMgr->hOs);

    return OK;
}


/***********************************************************************
 *                        siteMgr_stop
 ***********************************************************************
DESCRIPTION: Called by the SME SM in order to stop site mgr timers


INPUT:      hSiteMgr    -   site mgr handle.

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS siteMgr_stop(TI_HANDLE    hSiteMgr)
{

    return OK;
}

/***********************************************************************
 *                        siteMgr_CheckRxSignalValidity
 ***********************************************************************
DESCRIPTION: Called by the mlme_parserRecv when receiving managment frame 
                Find the ste in the site table and validate that the 
                RSSI of that site signal is not lower then -80DB + not lower
                then the exising site RSSI


INPUT:      hSiteMgr    -   site mgr handle.
            rxLevel     -   Rx level the frame received in
            bssid       -   BSSID of the frame

OUTPUT:

RETURN:     OK / NOK

************************************************************************/
TI_STATUS siteMgr_CheckRxSignalValidity(TI_HANDLE  hSiteMgr,
                         INT8               rxLevel,
                         UINT8              channel,
                         macAddress_t       *bssid)
{
    siteEntry_t *pSite;
    siteMgr_t   *pSiteMgr = (siteMgr_t *)hSiteMgr;

    pSite = NULL;
    pSite = findSiteEntry(pSiteMgr, bssid);

    if (pSite != NULL)
    {
        if ((channel != pSite->channel) &&
            (rxLevel < pSite->rssi))
        {   /* Ignore wrong packets with lower RSSI that were detect as
            ripples from different channels */
            WLAN_REPORT_WARNING(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                              ("siteMgr_CheckRxSignalValidity:Rx RSSI =%d, on channel=%d, is lower then given RSSI =%d on channel=%d, dropping it.\n", 
                               rxLevel,  channel, pSite->rssi, pSite->channel));
            return NOK;
        }
    }
    return OK;
}

/***********************************************************************
 *                        siteMgr_updateRxSignal
 ***********************************************************************
DESCRIPTION: Called by the Rx data when receiving a frae in the RX
                Find the ste in the site table and updates the SNR and RSSI


INPUT:      hSiteMgr    -   site mgr handle.
            snr         -   SNR the frame received in
            rxLevel     -   Rx level the frame received in
            bssid       -   BSSID of the frame

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS siteMgr_updateRxSignal(TI_HANDLE  hSiteMgr,
                         UINT8              snr,
                         INT8               rxLevel,
                         rate_e             rate,
                         macAddress_t       *bssid,
                         BOOL               dataMsdu)
{
    siteEntry_t *pSite;
    siteMgr_t   *pSiteMgr = (siteMgr_t *)hSiteMgr;

    pSite = findSiteEntry(pSiteMgr, bssid);

    if (pSite != NULL)
    {
        pSite->localTimeStamp = os_timeStampMs(pSiteMgr->hOs);

        pSite->snr = snr;

    /* We check the rate just for finding error case */
    switch(rate)
        {
        case DRV_RATE_1M:
        case DRV_RATE_2M:
        case DRV_RATE_5_5M:
        case DRV_RATE_11M:
        case DRV_RATE_22M:
        case DRV_RATE_6M:
        case DRV_RATE_9M:
        case DRV_RATE_12M:
        case DRV_RATE_18M:
        case DRV_RATE_24M:
        case DRV_RATE_36M:
        case DRV_RATE_48M:
        case DRV_RATE_54M:
            pSite->rxRate = rate;
            break;
        default:
            pSite->rxRate = (rate_e)0;
            WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                ("siteMgr_updateRxSignal:Rx Rate is GARBAGE!\nRx Rate = %d\n", rate));
            break;
        }
        /* Is this is not a primary site, update rssi;
           For the Primary site, we store average RSSI that is counted and updated from Current BSS module */
        if (pSite->siteType != SITE_PRIMARY)
        {
            pSite->rssi = (INT8)rxLevel;
        }
    }

    return OK;
}


/***********************************************************************
 *                        siteMgr_updatePrimarySiteFailStatus
 ***********************************************************************
DESCRIPTION: Called by the SME SM when the connection with the primary site fails
                If the primary site is NULL, return.


INPUT:      hSiteMgr    -   site mgr handle.
            bRemoveSite -   Whether to remove the site

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS siteMgr_updatePrimarySiteFailStatus(TI_HANDLE hSiteMgr,
                                           BOOL bRemoveSite)
{
    siteMgr_t           *pSiteMgr = (siteMgr_t *)hSiteMgr;
    siteTablesParams_t  *currTable = pSiteMgr->pSitesMgmtParams->pCurrentSiteTable;

    if (pSiteMgr->pSitesMgmtParams->pPrimarySite == NULL)
        return OK;

    WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
    (" SITE MGR: bRemoveSite = %d \n", bRemoveSite));

    if (bRemoveSite) 
    {
        removeSiteEntry(pSiteMgr, currTable, pSiteMgr->pSitesMgmtParams->pPrimarySite);
        pSiteMgr->pSitesMgmtParams->pPrimarySite = NULL;
    }
    else	/* Currently never used */
    {
        pSiteMgr->pSitesMgmtParams->pPrimarySite->failStatus = STATUS_UNSPECIFIED;
        pSiteMgr->pSitesMgmtParams->pPrimarySite->attemptsNumber++;
        pSiteMgr->pSitesMgmtParams->pPrimarySite->attemptsNumber &= 0x0F;
    }

    return OK;
}



/***********************************************************************
 *          siteMgr_resetEventStatisticsHistory
 ***********************************************************************
DESCRIPTION: Called by the SME SM when successing in establishing a connection with the primary site
                If thr primary site is NULL, return.
                Otherwise, reset the site rssi level

INPUT:      hSiteMgr    -   site mgr handle.

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS siteMgr_resetEventStatisticsHistory(TI_HANDLE hSiteMgr)
{
    siteMgr_t   *pSiteMgr = (siteMgr_t *)hSiteMgr;

    if (pSiteMgr->pSitesMgmtParams->pPrimarySite == NULL)
    {
        WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG, ("siteMgr_resetPrimarySiteRssiPerLevelBeaconMissed: Primary Site Is NULL\n"));
        return OK;
    }

    return OK;
}


/***********************************************************************
 *                        siteMgr_resetPrimarySiteAttemptsNumber
 ***********************************************************************
DESCRIPTION: Called by the SME SM when successing in establishing a connection with the primary site
                If thr primary site is NULL, return.
                Otherwise, reset the site attemptsNumber counter.


INPUT:      hSiteMgr    -   site mgr handle.

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS siteMgr_resetPrimarySiteAttemptsNumber(TI_HANDLE  hSiteMgr)
{
    siteMgr_t   *pSiteMgr = (siteMgr_t *)hSiteMgr;

    if (pSiteMgr->pSitesMgmtParams->pPrimarySite == NULL)
    {
        WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG, ("siteMgr_resetPrimarySiteAttemptsNumber: Primary Site Is NULL\n"));
        return OK;
    }
    pSiteMgr->pSitesMgmtParams->pPrimarySite->attemptsNumber = 0;

    return OK;
}

/***********************************************************************
 *                        siteMgr_resetPrevPrimarySiteRssi
 ***********************************************************************
DESCRIPTION: Called by the SME SM when successing in establishing a connection with the primary site
                If thr primary site is NULL, return.
                Otherwise, reset the site attemptsNumber counter.


INPUT:      hSiteMgr    -   site mgr handle.

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS siteMgr_resetPrevPrimarySiteRssi(TI_HANDLE    hSiteMgr)
{
    siteMgr_t   *pSiteMgr = (siteMgr_t *)hSiteMgr;

    if (pSiteMgr->pSitesMgmtParams->pPrevPrimarySite == NULL)
    {
        WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                ("Previous Primary Site Is NULL\n"));
        return OK;
    }

    WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                            ("Resetting previous primary site RSSI\n"));
    /*
    pSiteMgr->pSitesMgmtParams->pPrevPrimarySite->rssi = 0;
    pSiteMgr->pSitesMgmtParams->pPrevPrimarySite->attemptsNumber = 0;
    */

    return OK;
}

/***********************************************************************
 *                        siteMgr_isCurrentBand24
 ***********************************************************************
DESCRIPTION: The function checks the current operational mode and
                returns if the current band is 2.4Ghz or 5Ghz.

INPUT:      hSiteMgr    -   site mgr handle.

OUTPUT:

RETURN:     TRUE if current band is 2.4Ghz, FALSE otherwise.

************************************************************************/
BOOL siteMgr_isCurrentBand24(TI_HANDLE  hSiteMgr)
{
    siteMgr_t   *pSiteMgr =     (siteMgr_t *)hSiteMgr;

    if(pSiteMgr->siteMgrOperationalMode == DOT11_A_MODE)
        return FALSE;

    return TRUE; /* 802.11b supports onlty 2.4G band */

}


/***********************************************************************
 *                        siteMgr_isThereValidSSID
 ***********************************************************************
DESCRIPTION: checks if a valid SSID exists

INPUT:      hSiteMgr    -   site mgr handle.

OUTPUT:

RETURN:     TRUE if valid SSID found, FALSE if not

************************************************************************/
BOOL siteMgr_isThereValidSSID (TI_HANDLE hSiteMgr)
{
   paramInfo_t param;

   param.paramType = SITE_MGR_DESIRED_SSID_PARAM;
   siteMgr_getParam(hSiteMgr,&param);

    if((utils_isAnySSID(&param.content.siteMgrDesiredSSID)) || (utils_isJunkSSID(&param.content.siteMgrDesiredSSID)))
      return (FALSE);
    else
       return (TRUE);
}

/***********************************************************************
 *                        removeEldestSite
 ***********************************************************************
DESCRIPTION: Called by the select when trying to create an IBSS and site table is full
                Remove the eldest site from the table

INPUT:      hSiteMgr    -   site mgr handle.

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS removeEldestSite(siteMgr_t *pSiteMgr)
{
    int             i;
    siteEntry_t     *pEldestSite = NULL, *pSiteTable = pSiteMgr->pSitesMgmtParams->pCurrentSiteTable->siteTable;
    siteTablesParams_t  *currTable = pSiteMgr->pSitesMgmtParams->pCurrentSiteTable;
    UINT32          currentTimsStamp = os_timeStampMs(pSiteMgr->hOs);
    UINT32          biggestGap = 0;

    for (i = 0; i < currTable->maxNumOfSites; i++)
    {
        if (biggestGap < ((UINT32)(currentTimsStamp - pSiteTable[i].localTimeStamp)))
        {
            biggestGap = ((UINT32)(currentTimsStamp - pSiteTable[i].localTimeStamp));
            pEldestSite = &(pSiteTable[i]);
        }
    }

    removeSiteEntry(pSiteMgr, currTable, pEldestSite);

    return OK;
}


/***********************************************************************
 *                        update_apsd
 ***********************************************************************
DESCRIPTION:    Sets the site APSD support flag according to the
                beacon's capabilities vector and the WME-params IE if exists.

INPUT:      pSite       -   Pointer to the site entry in the site table
            pFrameInfo  -   Frame information after the parsing
            
OUTPUT:     pSite->APSDSupport flag

RETURN:     

************************************************************************/
static void update_apsd(siteEntry_t *pSite, mlmeFrameInfo_t *pFrameInfo)
{
    /* If WME-Params IE is not included in the beacon, set the APSD-Support flag
         only by the beacons capabilities bit map. */
    if (pFrameInfo->content.iePacket.WMEParams == NULL)
        pSite->APSDSupport = (((pFrameInfo->content.iePacket.capabilities >> CAP_APSD_SHIFT) & CAP_APSD_MASK) ? TRUE : FALSE);

    /* Else, set the APSD-Support flag if either the capabilities APSD bit or the 
         WME-Params APSD bit indicate so. */
    else
        pSite->APSDSupport = ((((pFrameInfo->content.iePacket.capabilities >> CAP_APSD_SHIFT) & CAP_APSD_MASK) ? TRUE : FALSE) ||
        (((pFrameInfo->content.iePacket.WMEParams->ACInfoField >> AP_QOS_INFO_UAPSD_SHIFT) & AP_QOS_INFO_UAPSD_MASK) ? TRUE : FALSE));
}


/***********************************************************************
 *                        release_module
 ***********************************************************************
DESCRIPTION:    Called by the un load function
                Go over the vector, for each bit that is set, release the corresponding module.

INPUT:      pSiteMgr    -   site mgr handle.
            initVec -   Vector that contains a bit set for each module thah had been initiualized

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
static void release_module(siteMgr_t *pSiteMgr, UINT32 initVec)
{
    if (initVec & (1 << MGMT_PARAMS_INIT_BIT))
        utils_nullMemoryFree(pSiteMgr->hOs, pSiteMgr->pSitesMgmtParams, sizeof(sitesMgmtParams_t));

    if (initVec & (1 << DESIRED_PARAMS_INIT_BIT))
        utils_nullMemoryFree(pSiteMgr->hOs, pSiteMgr->pDesiredParams, sizeof(siteMgrInitParams_t));

    if (initVec & (1 << SITE_MGR_INIT_BIT))
        utils_nullMemoryFree(pSiteMgr->hOs, pSiteMgr, sizeof(siteMgr_t));

    initVec = 0;
}


/***********************************************************************
 *                        updateSiteInfo
 ***********************************************************************
DESCRIPTION:    Called upon receiving a beacon or probe response
                Go over the vector, for each bit that is set, release the corresponding module.
                Update theaite entry in the site table with the information received in the frame

INPUT:      pSiteMgr    -   site mgr handle.
            pFrameInfo  -   Frame information after the parsing
            pSite       -   Pointer to the site entry in the site table

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
static void updateSiteInfo(siteMgr_t *pSiteMgr, mlmeFrameInfo_t *pFrameInfo, siteEntry_t    *pSite, UINT8 rxChannel)
{
    paramInfo_t param;
    BOOL        ssidUpdated;


    SET_ENTRY_FLAG_IN_SITE_TABLE(pSite);
    ssidUpdated = FALSE;

    switch (pFrameInfo->subType)
    {
    case BEACON:

        UPDATE_BEACON_INTERVAL(pSite, pFrameInfo);
        
        UPDATE_CAPABILITIES(pSite, pFrameInfo);
        
		if (utils_isIESSID_Broadcast(pFrameInfo->content.iePacket.pSsid) == FALSE)
        {   /* And the SSID is not Broadcast */
                ssidUpdated = TRUE;
                UPDATE_SSID(pSite, pFrameInfo);
        }

        if (ssidUpdated)
        {
            UPDATE_PRIVACY(pSite, pFrameInfo);
        }

        update_apsd(pSite, pFrameInfo);

        updatePreamble(pSiteMgr, pSite, pFrameInfo);

        UPDATE_AGILITY(pSite, pFrameInfo);


        if(pSiteMgr->siteMgrOperationalMode == DOT11_G_MODE)
        {
            UPDATE_SLOT_TIME(pSite, pFrameInfo);
            UPDATE_PROTECTION(pSite, pFrameInfo);
        }

        updateRates(pSiteMgr, pSite, pFrameInfo);

        if ((pFrameInfo->content.iePacket.pDSParamsSet != NULL)  &&
            (pFrameInfo->content.iePacket.pDSParamsSet->currChannel!=rxChannel))
        {
            WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG, 
                           ("updateSiteInfo, wrong CHANNELS:rxChannel=%d,currChannel=%d\n",
                            rxChannel, pFrameInfo->content.iePacket.pDSParamsSet->currChannel));
        }
        else
            UPDATE_CHANNEL(pSite, pFrameInfo , rxChannel);


        UPDATE_BSS_TYPE(pSite, pFrameInfo);

        if (pSite->bssType == BSS_INFRASTRUCTURE)
            UPDATE_DTIM_PERIOD(pSite, pFrameInfo);

        UPDATE_ATIM_WINDOW(pSite, pFrameInfo);

        UPDATE_BEACON_AP_TX_POWER(pSite, pFrameInfo);

        /* Updating QoS params */
        updateBeaconQosParams(pSiteMgr, pSite, pFrameInfo);

        /* updating CountryIE  */
        if ((pFrameInfo->content.iePacket.country  != NULL) && 
            (pFrameInfo->content.iePacket.country->hdr.eleLen != 0))
        {
            /* set the country info in the regulatory domain - If a different code was detected earlier
               the regDomain will ignore it */
            param.paramType = REGULATORY_DOMAIN_COUNTRY_PARAM;
            param.content.pCountry = (country_t *)pFrameInfo->content.iePacket.country;
            regulatoryDomain_setParam(pSiteMgr->hRegulatoryDomain,&param);
        }

        /* Checking if this beacon is a DTIM i.e. DTIM count is zero */
        if( (pFrameInfo->content.iePacket.pTIM != NULL) && (pFrameInfo->content.iePacket.pTIM->dtimCount == 0))
        {
            /* Update the DTIM beacon time */
            UPDATE_DTIM_TIME(pSiteMgr, pSite, pFrameInfo);
        }

        UPDATE_LOCAL_TIME_STAMP(pSiteMgr, pSite, pFrameInfo);

        UPDATE_BEACON_MODULATION(pSite, pFrameInfo);

        /* If the BSS type is independent, the beacon & probe modulation are equal,
            It is important to update this field here for dynamic PBCC algorithm compatibility */
        if (pSite->bssType == BSS_INDEPENDENT)
            UPDATE_PROBE_MODULATION(pSite, pFrameInfo);


        if (pSite->siteType == SITE_PRIMARY)
        {
           
            if (pSiteMgr->pSitesMgmtParams->pPrimarySite == NULL)
            {
                WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG, ("updateSiteInfo: Primary Site Is NULL\n"));
                pSite->siteType = SITE_REGULAR;
            }
            else
            {
                /*  If the site that we got the beacon on is the primary site - which means we are either trying */
                /*  to connect to it or we are already connected - send the EVENT_GOT_BEACON to the conn module (through the SME module) */
                /*  so the conn module will be aware of the beacon status of the site it's trying to connect to */
                
#ifdef EXC_MODULE_INCLUDED
                UINT8 ExternTxPower;

                if (pFrameInfo->content.iePacket.cellTP != NULL)
                {
                    ExternTxPower = pFrameInfo->content.iePacket.cellTP->power;
                }
                else	/* Set to maximum possible. Note that we add +1 so that Dbm = 26 and not 25 */
                {
                    ExternTxPower = MAX_TX_POWER / DBM_TO_TX_POWER_FACTOR + 1;
                }

                param.paramType = REGULATORY_DOMAIN_EXTERN_TX_POWER_PREFERRED;
                param.content.ExternTxPowerPreferred = ExternTxPower;
                regulatoryDomain_setParam(pSiteMgr->hRegulatoryDomain, &param);
#endif

				/* Updating the Tx Power according to the received Power Constraint  */
				if(pFrameInfo->content.iePacket.powerConstraint  != NULL)
				{   /* Checking if the recieved constraint is different from the one that is already known  */
					if( pFrameInfo->content.iePacket.powerConstraint->powerConstraint != pSite->powerConstraint)
					{   /* check if Spectrum Management is enabled */
						param.paramType = REGULATORY_DOMAIN_MANAGEMENT_CAPABILITY_ENABLED_PARAM;
						regulatoryDomain_getParam(pSiteMgr->hRegulatoryDomain,&param);
						if(param.content.spectrumManagementEnabled)
						{   /* setting power constraint */
							pSite->powerConstraint = pFrameInfo->content.iePacket.powerConstraint->powerConstraint;
							param.paramType = REGULATORY_DOMAIN_SET_POWER_CONSTRAINT_PARAM;
							param.content.powerConstraint = pSite->powerConstraint;
							regulatoryDomain_setParam(pSiteMgr->hRegulatoryDomain,&param);

						}
					}
				}

                if (pSite->beaconReceiveAfterJoin == FALSE)
                {
                    /*
                    use for indicate to that it safe to call to PowerMgr_startPS() that can be call only
                    after join command followed by beacon receive later.
                    every join command the beaconReceiveAfterJoin flag is cleared therefor it will
                    be true after the join command.

                    the right follow is: join --> receive primary site beacon --> PowerMgr_startPS().

                    Note: the beaconReceiveAfterJoin flag is in use for the SME path!
                    */
                    siteMgr_gotFirstBcn(pSiteMgr);
                    /* set Hw not available now that the first beacon arrived */
                    MacServices_powerAutho_AwakeRequiredUpdate(pSiteMgr->hMacServices, POWERAUTHO_AWAKE_NOT_REQUIRED, POWERAUTHO_AWAKE_REASON_FIRST_BEACON);

                    pSite->beaconReceiveAfterJoin = TRUE;

                    WLAN_REPORT_INFORMATION(pSiteMgr->hReport,
                                            SITE_MGR_MODULE_LOG,
                                            ("1st beacon after join command RECEIVED!!!\n"));
        }

            }
        }

        UPDATE_BEACON_RECV(pSite);
        
        if (ssidUpdated)
        {
            dot11_RSN_t *pRsnIe = pFrameInfo->content.iePacket.pRsnIe;
            UINT8       rsnIeLen = pFrameInfo->content.iePacket.rsnIeLen;
            UPDATE_RSN_IE(pSite, pRsnIe, rsnIeLen);
        }


        UPDATE_BEACON_TIMESTAMP(pSiteMgr, pSite, pFrameInfo);

        break;


    case PROBE_RESPONSE:

        UPDATE_BEACON_INTERVAL(pSite, pFrameInfo);

        UPDATE_CAPABILITIES(pSite, pFrameInfo);

                ssidUpdated = TRUE;
        if (pSite->siteType == SITE_PRIMARY)
        {   /* Primary SITE */
            if ((pFrameInfo->content.iePacket.pSsid != NULL) && 
                (pSiteMgr->pDesiredParams->siteMgrDesiredSSID.len>0))
            {   /* There's a desired SSID*/
                if (os_memoryCompare(pSiteMgr->hOs, (UINT8*)(pSiteMgr->pDesiredParams->siteMgrDesiredSSID.ssidString), 
                                     (UINT8*)(pFrameInfo->content.iePacket.pSsid->serviceSetId), pFrameInfo->content.iePacket.pSsid->hdr.eleLen)!=0)
                {   /* Do not overwrite the primary site's SSID with a different than the desired SSID*/
                    ssidUpdated = FALSE;
                }
                
            }
            else if (pFrameInfo->content.iePacket.pSsid == NULL)
            {
                WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                  ("updateSiteInfo PROBE_RESP, pSsid=NULL\n"));
            }
        }

        if (ssidUpdated)
        {
            UPDATE_SSID(pSite, pFrameInfo);
            UPDATE_PRIVACY(pSite, pFrameInfo);
        }

        update_apsd(pSite, pFrameInfo);


        if(pSiteMgr->siteMgrOperationalMode == DOT11_G_MODE)
        {
            UPDATE_PROTECTION(pSite, pFrameInfo);
        }

        updateFourX(pSiteMgr, pSite, pFrameInfo);

        updatePreamble(pSiteMgr, pSite, pFrameInfo);

        UPDATE_AGILITY(pSite, pFrameInfo);

        updateRates(pSiteMgr, pSite, pFrameInfo);

        if ((pFrameInfo->content.iePacket.pDSParamsSet != NULL)  &&
            (pFrameInfo->content.iePacket.pDSParamsSet->currChannel!=rxChannel))
        {
            WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG, 
                           ("updateSiteInfo, wrong CHANNELS:rxChannel=%d,currChannel=%d\n",
                            rxChannel, pFrameInfo->content.iePacket.pDSParamsSet->currChannel));
        }
        else
            UPDATE_CHANNEL(pSite, pFrameInfo, rxChannel);


        UPDATE_BSS_TYPE(pSite, pFrameInfo);

        UPDATE_ATIM_WINDOW(pSite, pFrameInfo);

        UPDATE_PROBE_AP_TX_POWER(pSite, pFrameInfo);

        /* Updating WME params */
        updateProbeQosParams(pSiteMgr, pSite, pFrameInfo);


        /* updating CountryIE  */
        if ((pFrameInfo->content.iePacket.country  != NULL) && 
            (pFrameInfo->content.iePacket.country->hdr.eleLen != 0))
        {
            /* set the country info in the regulatory domain - If a different code was detected earlier
               the regDomain will ignore it */
            param.paramType = REGULATORY_DOMAIN_COUNTRY_PARAM;
            param.content.pCountry = (country_t *)pFrameInfo->content.iePacket.country;
            regulatoryDomain_setParam(pSiteMgr->hRegulatoryDomain,&param);
		}

        UPDATE_LOCAL_TIME_STAMP(pSiteMgr, pSite, pFrameInfo);

        UPDATE_PROBE_MODULATION(pSite, pFrameInfo);

        /* If the BSS type is independent, the beacon & probe modulation are equal,
            It is important to update this field here for dynamic PBCC algorithm compatibility */
        if (pSite->bssType == BSS_INDEPENDENT)
            UPDATE_BEACON_MODULATION(pSite, pFrameInfo);

        UPDATE_PROBE_RECV(pSite);

        if (ssidUpdated)
        {
            dot11_RSN_t *pRsnIe = pFrameInfo->content.iePacket.pRsnIe;
            UINT8       rsnIeLen = pFrameInfo->content.iePacket.rsnIeLen;
            UPDATE_RSN_IE(pSite, pRsnIe, rsnIeLen);

        }

        UPDATE_BEACON_TIMESTAMP(pSiteMgr, pSite, pFrameInfo);

        break;

    default:
        WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("Site Update failure, un known frame sub type %d\n\n", pFrameInfo->subType));
        break;
    }
}


/***********************************************************************
 *                        updateFourX
 ***********************************************************************
DESCRIPTION:    Called by the function 'updateSiteInfo()'

INPUT:      pSiteMgr    -   site mgr handle.
            pFrameInfo  -   Frame information after the parsing
            pSite       -   Pointer to the site entry in the site table

OUTPUT:

RETURN:

************************************************************************/
static void updateFourX(siteMgr_t *pSiteMgr, siteEntry_t *pSite, mlmeFrameInfo_t *pFrameInfo)
{
    UINT8 ti_oui[] = TI_OUI;

    /* It is done to overcome interop issues. We don't send AssocReq with 4x IE
       if ProbeReq without 4x IE was received previously */
    if (pFrameInfo->content.iePacket.fourXParams != NULL)
    {
        os_memoryCopy(pSiteMgr->hOs, &pSite->fourXParams, pFrameInfo->content.iePacket.fourXParams, pFrameInfo->content.iePacket.fourXParams->hdr.eleLen+sizeof( dot11_eleHdr_t));

        if( (pSite->fourXParams.hdr.eleId != DOT11_4X_ELE_ID) ||
            (pSite->fourXParams.hdr.eleLen > DOT11_4X_MAX_LEN) ||
            (os_memoryCompare(pSiteMgr->hOs ,ti_oui, (PUINT8)pSite->fourXParams.fourXCapabilities, DOT11_OUI_LEN) != 0) )
        {
            pSite->fourXsupported = FALSE;
        }
    }
    else
        pSite->fourXsupported = FALSE;
}

/***********************************************************************
 *                        updatePreamble
 ***********************************************************************
DESCRIPTION:    Called by the function 'updateSiteInfo()'

INPUT:      pSiteMgr    -   site mgr handle.
            pFrameInfo  -   Frame information after the parsing
            pSite       -   Pointer to the site entry in the site table

OUTPUT:

RETURN:

************************************************************************/
static void updatePreamble(siteMgr_t *pSiteMgr, siteEntry_t *pSite, mlmeFrameInfo_t *pFrameInfo)
{
    pSite->currentPreambleType = ((pFrameInfo->content.iePacket.capabilities >> CAP_PREAMBLE_SHIFT) & CAP_PREAMBLE_MASK) ? PREAMBLE_SHORT : PREAMBLE_LONG;

    pSite->barkerPreambleType = pFrameInfo->content.iePacket.barkerPreambleMode;
}

/***********************************************************************
 *                        updateBeaconQosParams
 ***********************************************************************
DESCRIPTION:    Called by the function 'updateSiteInfo()'

INPUT:      pSiteMgr    -   site mgr handle.
            pFrameInfo  -   Frame information after the parsing
            pSite       -   Pointer to the site entry in the site table

OUTPUT:

RETURN:

************************************************************************/

static void updateBeaconQosParams(siteMgr_t *pSiteMgr, siteEntry_t *pSite, mlmeFrameInfo_t *pFrameInfo)
{
    /* Updating WME params */
    if (pFrameInfo->content.iePacket.WMEParams  != NULL)
    {
        /* Checking if this is IE includes new WME Parameters */
        if(( ((pFrameInfo->content.iePacket.WMEParams->ACInfoField) & dot11_WME_ACINFO_MASK ) != pSite->lastWMEParameterCnt) ||
            (!pSite->WMESupported) )
        {
            pSite->WMESupported = TRUE;

            /* Checking if this IE is information only or is a paremeters IE */
            if(pFrameInfo->content.iePacket.WMEParams->OUISubType == dot11_WME_OUI_SUB_TYPE_PARAMS_IE)
            {
                if(pSite->siteType == SITE_PRIMARY)
                {
                    qosMngr_updateIEinfo(pSiteMgr->hQosMngr,(UINT8 *)(pFrameInfo->content.iePacket.WMEParams),WME);
                }
                /* updating the WME paraeters into the site table. */
                os_memoryCopy(pSiteMgr->hOs, &pSite->WMEParameters, &(pFrameInfo->content.iePacket.WMEParams->WME_ACParameteres), sizeof( ACParameters_t));
                pSite->lastWMEParameterCnt = (pFrameInfo->content.iePacket.WMEParams->ACInfoField) & dot11_WME_ACINFO_MASK;
                WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,("$$$$$$ WME parameters were updates according to beacon, cntSeq = %d\n",pSite->lastWMEParameterCnt));
            }
        }
    }else
    {
        pSite->WMESupported = FALSE;
        }

}

/***********************************************************************
 *                        updateProbeQosParams
 ***********************************************************************
DESCRIPTION:    Called by the function 'updateSiteInfo()'

INPUT:      pSiteMgr    -   site mgr handle.
            pFrameInfo  -   Frame information after the parsing
            pSite       -   Pointer to the site entry in the site table

OUTPUT:

RETURN:

************************************************************************/
static void updateProbeQosParams(siteMgr_t *pSiteMgr, siteEntry_t *pSite, mlmeFrameInfo_t *pFrameInfo)
{
    /* Updating WME params */
    if (pFrameInfo->content.iePacket.WMEParams  != NULL)
    {
        /* Checking if this is IE includes new WME Parameters */
        if(( ((pFrameInfo->content.iePacket.WMEParams->ACInfoField) & dot11_WME_ACINFO_MASK ) != pSite->lastWMEParameterCnt) ||
            (!pSite->WMESupported) )
        {
            pSite->WMESupported = TRUE;

            /* Checking if this IE is information only or is a paremeters IE */
            if(pFrameInfo->content.iePacket.WMEParams->OUISubType == dot11_WME_OUI_SUB_TYPE_PARAMS_IE)
            {
                if(pSite->siteType == SITE_PRIMARY)
                {
                    qosMngr_updateIEinfo(pSiteMgr->hQosMngr,(UINT8 *)(pFrameInfo->content.iePacket.WMEParams),WME);
                }
                /* updating the WME paraeters into the site table. */
                os_memoryCopy(pSiteMgr->hOs, &pSite->WMEParameters, &(pFrameInfo->content.iePacket.WMEParams->WME_ACParameteres), sizeof( ACParameters_t));
                pSite->lastWMEParameterCnt = (pFrameInfo->content.iePacket.WMEParams->ACInfoField) & dot11_WME_ACINFO_MASK;
                WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,("$$$$$$ WME parameters were updates according to  probe response, cntSeq = %d\n",pSite->lastWMEParameterCnt));
            }
        }
    }else
    {
        pSite->WMESupported = FALSE;
        }

}

/***********************************************************************
 *                        updateRates
 ***********************************************************************
DESCRIPTION:    Called by the function 'updateSiteInfo()' in order to translate the rates received
                in the beacon or probe response to rate used by the driver. Perfoms the following:
                    -   Check the rates. validity. If rates are invalid, return
                    -   Get the max active rate & max basic rate, if invalid, return
                    -   Translate the max active rate and max basic rate from network rates to host rates.
                        The max active & max basic rate are used by the driver from now on in all the processes:
                        (selection, join, transmission, etc....)

INPUT:      pSiteMgr    -   site mgr handle.
            pFrameInfo  -   Frame information after the parsing
            pSite       -   Pointer to the site entry in the site table

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
static void updateRates(siteMgr_t *pSiteMgr, siteEntry_t *pSite, mlmeFrameInfo_t *pFrameInfo)
{
    UINT8   maxBasicRate = 0, maxActiveRate = 0;
    UINT32  bitMapExtSupp = 0;
    paramInfo_t param;

    if (pFrameInfo->content.iePacket.pRates == NULL)
    {
        WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("updateRates, pRates=NULL, beacon & probeResp are: \n"));
        WLAN_REPORT_HEX_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG, 
                          (UINT8*)pFrameInfo->content.iePacket.pRates, pFrameInfo->content.iePacket.pRates->hdr.eleLen+2);
        WLAN_REPORT_HEX_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG, 
                          (UINT8*)pFrameInfo->content.iePacket.pRates, pFrameInfo->content.iePacket.pRates->hdr.eleLen+2);
        return;
    }

    /* Update the rate elements */
    maxBasicRate = getMaxBasicRatefromString((UINT8 *)pFrameInfo->content.iePacket.pRates->rates,pFrameInfo->content.iePacket.pRates->hdr.eleLen, maxBasicRate);
    maxActiveRate = getMaxActiveRatefromString((UINT8 *)pFrameInfo->content.iePacket.pRates->rates,pFrameInfo->content.iePacket.pRates->hdr.eleLen, maxActiveRate);
    
    if(pFrameInfo->content.iePacket.pExtRates)
    {
        maxBasicRate = getMaxBasicRatefromString((UINT8 *)pFrameInfo->content.iePacket.pExtRates->rates,pFrameInfo->content.iePacket.pExtRates->hdr.eleLen, maxBasicRate);
        maxActiveRate = getMaxActiveRatefromString((UINT8 *)pFrameInfo->content.iePacket.pExtRates->rates,pFrameInfo->content.iePacket.pExtRates->hdr.eleLen, maxActiveRate);
    }


    /*WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("1- maxBasicRate = 0x%X, maxActiveRate = 0x%X \n", maxBasicRate,maxActiveRate));*/

    if (maxActiveRate == 0)
        maxActiveRate = maxBasicRate;

    /* Now update it from network to host rates */
    pSite->maxBasicRate = networkToHostRate(maxBasicRate);

    pSite->maxActiveRate = networkToHostRate(maxActiveRate);
    if (pSite->maxActiveRate == DRV_RATE_INVALID)
            WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("Network To Host Rate failure, no active network rate\n"));

    if (pSite->maxBasicRate != DRV_RATE_INVALID)
    {
        if (pSite->maxActiveRate != DRV_RATE_INVALID)
        {
            pSite->maxActiveRate = MAX(pSite->maxActiveRate,pSite->maxBasicRate);
        }
    } else { /* in case some vendors don't specify basic rates */
        WLAN_REPORT_WARNING(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("Network To Host Rate failure, no basic network rate"));
        pSite->maxBasicRate = pSite->maxActiveRate;
    }

    /* build rates bit map */
    networkStringToBitMapSuppRates((UINT32 *)&pSite->rateMask.supportedRateMask,
                                   (UINT8 *)pFrameInfo->content.iePacket.pRates->rates,
                                   pFrameInfo->content.iePacket.pRates->hdr.eleLen);
    networkStringToBitMapBasicRates((UINT32 *)&pSite->rateMask.basicRateMask,
                                    (UINT8 *)pFrameInfo->content.iePacket.pRates->rates,
                                    pFrameInfo->content.iePacket.pRates->hdr.eleLen);

    if(pFrameInfo->content.iePacket.pExtRates)
    {
        networkStringToBitMapSuppRates(&bitMapExtSupp, (UINT8 *)pFrameInfo->content.iePacket.pExtRates->rates,
                                       pFrameInfo->content.iePacket.pExtRates->hdr.eleLen);

        pSite->rateMask.supportedRateMask |= bitMapExtSupp;

        networkStringToBitMapBasicRates(&bitMapExtSupp, (UINT8 *)pFrameInfo->content.iePacket.pExtRates->rates,
                                        pFrameInfo->content.iePacket.pExtRates->hdr.eleLen);

        pSite->rateMask.basicRateMask |= bitMapExtSupp;
    }
    
    
    param.paramType = CTRL_DATA_CURRENT_SUPPORTED_RATE_MASK_PARAM;
    param.content.ctrlDataCurrentRateMask = pSite->rateMask.supportedRateMask;
    /* clear the 22Mbps bit in case the PBCC is not allowed */
    if(pSiteMgr->currentDataModulation != DRV_MODULATION_PBCC && pSiteMgr->currentDataModulation != DRV_MODULATION_OFDM)
    {
      param.content.ctrlDataCurrentRateMask &= ~DRV_RATE_MASK_22_PBCC;
    }
    ctrlData_setParam(pSiteMgr->hCtrlData, &param);

    param.paramType = CTRL_DATA_CURRENT_BASIC_RATE_MASK_PARAM;
    param.content.ctrlDataBasicRateBitMask = pSite->rateMask.basicRateMask;
    ctrlData_setParam(pSiteMgr->hCtrlData, &param);


    /*WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("2- pSite->maxActiveRate = 0x%X, pSite->maxBasicRate = 0x%X \n", pSite->maxActiveRate,pSite->maxBasicRate));*/

}

/***********************************************************************
 *                        getBssidList
 ***********************************************************************
DESCRIPTION:    Called by the OS abstraction layer in order to get the BSSID list from the site table

INPUT:      pSiteMgr    -   site mgr handle.

OUTPUT:     bssidList   -   BSSID list pointer

RETURN:

************************************************************************/
static TI_STATUS getBssidList(siteMgr_t *pSiteMgr, OS_802_11_BSSID_LIST_EX *bssidList, UINT32 *pLength, BOOL allVarIes)
{
    UINT8                   siteTableIndex, tableIndex, i, index, rsnIeLength;
    siteEntry_t             *pSiteEntry;
    OS_802_11_BSSID_EX      *pBssid;
    OS_802_11_FIXED_IEs     *pFixedIes;
    OS_802_11_VARIABLE_IEs  *pVarIes;
    
    UINT32                  length;
    UINT8                   *pData;
    UINT32                  len, firstOFDMloc = 0;
    siteTablesParams_t      *currTable;
    short                   numOfTables;

    UINT8		    *pDesiredIe;
    UINT8                   *pSiteEntryIeBuffer;
    UINT16                  ieLength;
	
    UINT8                   wpsOuiBuffer[DOT11_WPS_OUI_LEN+1]=DOT11_WPS_OUI;

#ifdef DBG_BSSIS_NAME_PRINT
    UINT8                   tempName[33];
#endif

    numOfTables = NUM_OF_SITE_TABLE;


    bssidList->NumberOfItems = 0;

    calculateBssidListSize(pSiteMgr, &length, allVarIes);

    if (length > *pLength)
    {
        *pLength = length;
        return NOK;
    }

    length = sizeof(OS_802_11_BSSID_LIST_EX) - sizeof(OS_802_11_BSSID_EX);

    pData = (UINT8*)&bssidList->Bssid[0];

    WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("Entering getBssidList, length = %d, pData = 0x%X\n", *pLength, (UINT32)pData));

    currTable = pSiteMgr->pSitesMgmtParams->pCurrentSiteTable;

    for (tableIndex = 0; tableIndex < numOfTables ; tableIndex++)
    {
        for (siteTableIndex = 0, i = 0; siteTableIndex < currTable->maxNumOfSites; siteTableIndex++)
        {
            pBssid = (OS_802_11_BSSID_EX*)pData;

            pSiteEntry = &(currTable->siteTable[siteTableIndex]);

            if (pSiteEntry->siteType == SITE_NULL)
                continue;


            /* MacAddress */
            os_memoryCopy(pSiteMgr->hOs, (void *)pBssid->MacAddress, (void *)&(pSiteEntry->bssid.addr), MAC_ADDR_LEN);

            
            /* Capabilities */
            pBssid->Union.Capabilities = pSiteEntry->capabilities;

            /* SSID */
            /* This line is in order to prevent presantation problems at the utility. */
            os_memoryZero(pSiteMgr->hOs, &(pBssid->Ssid.Ssid), MAX_SSID_LEN);
            if (pSiteEntry->ssid.len > MAX_SSID_LEN)
            {
                pSiteEntry->ssid.len = MAX_SSID_LEN;
            }
            os_memoryCopy(pSiteMgr->hOs, (void *)pBssid->Ssid.Ssid, &(pSiteEntry->ssid.ssidString), pSiteEntry->ssid.len);
            pBssid->Ssid.SsidLength = pSiteEntry->ssid.len;

            /* privacy */
            pBssid->Privacy = pSiteEntry->privacy;

            /* RSSI */

            pBssid->Rssi = (pSiteEntry->rssi);

            pBssid->Configuration.Length = sizeof(OS_802_11_CONFIGURATION);
            pBssid->Configuration.BeaconPeriod = pSiteEntry->beaconInterval;
            pBssid->Configuration.ATIMWindow = pSiteEntry->atimWindow;
            pBssid->Configuration.Union.channel = Chan2Freq(pSiteEntry->channel);

            pBssid->Configuration.FHConfig.Length = sizeof(OS_802_11_CONFIGURATION_FH);
            pBssid->Configuration.FHConfig.DwellTime = pSiteEntry->FHParams.dwellTime;
            pBssid->Configuration.FHConfig.HopPattern = pSiteEntry->FHParams.hopPattern;
            pBssid->Configuration.FHConfig.HopSet = pSiteEntry->FHParams.hopSet;

            if  (pSiteEntry->bssType == BSS_INDEPENDENT)
                pBssid->InfrastructureMode = os802_11IBSS;
            else
                pBssid->InfrastructureMode = os802_11Infrastructure;

            /* SupportedRates */
            os_memoryZero(pSiteMgr->hOs, (void *)pBssid->SupportedRates, sizeof(OS_802_11_RATES_EX));

            bitMapToNetworkStringRates(pSiteEntry->rateMask.supportedRateMask,
                                       pSiteEntry->rateMask.basicRateMask,
                                       (UINT8 *)pBssid->SupportedRates,
                                       &len, &firstOFDMloc);

            /* set network type acording to band and rates */
            if (currTable == &pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables)
            {
                if (firstOFDMloc == len)
                {
                    pBssid->NetworkTypeInUse = os802_11DS;
                } else {
                    pBssid->NetworkTypeInUse = os802_11OFDM24;
                }
            } else {
                pBssid->NetworkTypeInUse = os802_11OFDM5;
            }

            pBssid->IELength = 0;

            /* copy fixed IEs from site entry */
            pFixedIes = (OS_802_11_FIXED_IEs*)&pBssid->IEs[pBssid->IELength];
            os_memoryCopy(pSiteMgr->hOs, (void *)pFixedIes->TimeStamp, &pSiteEntry->localTimeStamp, sizeof(pSiteEntry->localTimeStamp));
            pFixedIes->BeaconInterval = pSiteEntry->beaconInterval;
            pFixedIes->Capabilities = pSiteEntry->capabilities;
            pBssid->IELength += sizeof(OS_802_11_FIXED_IEs);
            pVarIes = (OS_802_11_VARIABLE_IEs*)&pBssid->IEs[pBssid->IELength];

            if (!allVarIes)
            {   /* copy only several variable IEs */

                /* copy SSID */
                pVarIes->ElementID = SSID_IE_ID;
                pVarIes->Length = pSiteEntry->ssid.len;
                os_memoryCopy(pSiteMgr->hOs, (void *)pVarIes->data, (void *)pSiteEntry->ssid.ssidString, pSiteEntry->ssid.len);
                pBssid->IELength += (pVarIes->Length + 2);
    
                /* copy RATES */
                pVarIes = (OS_802_11_VARIABLE_IEs*)&pBssid->IEs[pBssid->IELength];
                pVarIes->ElementID = SUPPORTED_RATES_IE_ID;
                bitMapToNetworkStringRates(pSiteEntry->rateMask.supportedRateMask, pSiteEntry->rateMask.basicRateMask,
                                           (UINT8 *)pVarIes->data, &len, &firstOFDMloc);
                pVarIes->Length = len;
                pBssid->IELength += (pVarIes->Length + 2);
    
                /* copy FH */
                pVarIes = (OS_802_11_VARIABLE_IEs*)&pBssid->IEs[pBssid->IELength];
                pVarIes->ElementID = FH_PARAMETER_SET_IE_ID;
                pVarIes->Length = DOT11_FH_PARAMS_ELE_LEN;
                os_memoryCopy(pSiteMgr->hOs, (void *)pVarIes->data, &pSiteEntry->FHParams.dwellTime, sizeof(pSiteEntry->FHParams.dwellTime));
                os_memoryCopy(pSiteMgr->hOs, (void *)&pVarIes->data[2], &pSiteEntry->FHParams.hopSet, sizeof(pSiteEntry->FHParams.hopSet));
                os_memoryCopy(pSiteMgr->hOs, (void *)&pVarIes->data[3], &pSiteEntry->FHParams.hopPattern, sizeof(pSiteEntry->FHParams.hopPattern));
                pVarIes->data[4] = 1;
                pBssid->IELength += (pVarIes->Length + 2);
    
                /* copy DS */
                pVarIes = (OS_802_11_VARIABLE_IEs*)&pBssid->IEs[pBssid->IELength];
                pVarIes->ElementID = DS_PARAMETER_SET_IE_ID;
                pVarIes->Length = DOT11_DS_PARAMS_ELE_LEN;
                os_memoryCopy(pSiteMgr->hOs, (void *)pVarIes->data, &pSiteEntry->channel, DOT11_DS_PARAMS_ELE_LEN);
                pBssid->IELength += (pVarIes->Length + 2);
    
                /* copy RSN information elements */
                rsnIeLength = 0;
                for (index=0; index<MAX_RSN_IE && pSiteEntry->pRsnIe[index].hdr.eleLen>0; index++)
                {
                    pVarIes =  (OS_802_11_VARIABLE_IEs*)&pBssid->IEs[pBssid->IELength+rsnIeLength];
                    pVarIes->ElementID = pSiteEntry->pRsnIe[index].hdr.eleId;
                    pVarIes->Length = pSiteEntry->pRsnIe[index].hdr.eleLen;
                    os_memoryCopy(pSiteMgr->hOs, (void *)pVarIes->data, (void *)pSiteEntry->pRsnIe[index].rsnIeData, pSiteEntry->pRsnIe[index].hdr.eleLen);
                    rsnIeLength += pSiteEntry->pRsnIe[index].hdr.eleLen+2;
                    WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                            ("RSN IE ID=%d, Length=%x\n", pVarIes->ElementID, pVarIes->Length));
    
                    WLAN_REPORT_HEX_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG, (UINT8 *)pVarIes->data,pVarIes->Length);
                }
    
                pBssid->IELength += pSiteEntry->rsnIeLen;
    
                /* create information element for the WME/EXC.
                    Note that the existence of these IEs should be considered as a fact that the site supports this feature.
                    by alcel
                */
                if (pSiteEntry->WMESupported)
                {
                    /* oui */
                    UINT8 ouiWME[3] = { 0x50,0xf2,0x01};
                    dot11_WME_PARAM_t *pWMEParams; 
        
    
                    /* fill in the general element  parameters */
    
                    pVarIes =  (OS_802_11_VARIABLE_IEs*)&pBssid->IEs[pBssid->IELength];
                    pVarIes->ElementID =DOT11_WME_ELE_ID;
                    pVarIes->Length = DOT11_WME_PARAM_ELE_LEN;
    
                    WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                            ("WMESupported ID=%d, Length=%x\n", pVarIes->ElementID, pVarIes->Length));
    
                    /* fill in the specific element  parameters */
                    pWMEParams = (dot11_WME_PARAM_t *) pVarIes;
                    os_memoryCopy(pSiteMgr->hOs,(void *)pWMEParams->OUI,ouiWME,3);
                    pWMEParams->OUIType = dot11_WME_OUI_TYPE;
                    pWMEParams->OUISubType = dot11_WME_OUI_SUB_TYPE_PARAMS_IE;
                    pWMEParams->version = dot11_WME_VERSION;
                    pWMEParams->ACInfoField = dot11_WME_ACINFO_MASK & pSiteEntry->lastWMEParameterCnt;
    
                    /* fill in the data  */
    
                    os_memoryCopy(pSiteMgr->hOs,&(pWMEParams->WME_ACParameteres),&(pSiteEntry->WMEParameters),sizeof(ACParameters_t));
    
    
                    /* update the general length */
    
                    pBssid->IELength += (pVarIes->Length + 2);
                }

				/* copy WPS information elements */
                
				if ((pSiteEntry->probeRecv) &&
					(pSiteEntry->siteType != SITE_PRIMARY))
				{
					pSiteEntryIeBuffer = pSiteEntry->probeRespBuffer;
					ieLength = pSiteEntry->probeRespLength;
				}
				else
				{
					pSiteEntryIeBuffer = pSiteEntry->beaconBuffer;
					ieLength = pSiteEntry->beaconLength;
				}
                
				parseIeBuffer(pSiteMgr->hOs, pSiteEntryIeBuffer, ieLength, DOT11_WPS_ELE_ID, &pDesiredIe, wpsOuiBuffer, DOT11_WPS_OUI_LEN);

				if (pDesiredIe)
				{
					pVarIes = (OS_802_11_VARIABLE_IEs*)&pBssid->IEs[pBssid->IELength];
					pVarIes->ElementID = DOT11_WPS_ELE_ID;
					pVarIes->Length = *((UINT8*)(pDesiredIe + 1));
					os_memoryCopy(pSiteMgr->hOs, (void *)pVarIes->data, (UINT8*)(pDesiredIe+2), pVarIes->Length);

					pBssid->IELength += (pVarIes->Length + 2);
				}
            }
            else
            {   /* Copy all variable IEs */
                
                if ((pSiteEntry->probeRecv) &&
                    (pSiteEntry->siteType != SITE_PRIMARY))
                {
                    pSiteEntryIeBuffer = pSiteEntry->probeRespBuffer;
                    ieLength = pSiteEntry->probeRespLength;
                }
                else
                {
                    pSiteEntryIeBuffer = pSiteEntry->beaconBuffer;
                    ieLength = pSiteEntry->beaconLength;
                }
                WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                        ("Copy all variable IEs Length=%x\n", ieLength));
                WLAN_REPORT_HEX_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                            pSiteEntryIeBuffer, ieLength);

                os_memoryCopy(pSiteMgr->hOs, pVarIes, pSiteEntryIeBuffer, ieLength);

                pBssid->IELength += ieLength;
            }

            pBssid->Length = sizeof(OS_802_11_BSSID_EX) + pBssid->IELength - 1;

            WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                    ("BSSID MAC = %x-%x-%x-%x-%x-%x\n",
                                     pBssid->MacAddress[0], pBssid->MacAddress[1], pBssid->MacAddress[2],
                                     pBssid->MacAddress[3], pBssid->MacAddress[4], pBssid->MacAddress[5]));

#ifdef DBG_BSSIS_NAME_PRINT
            os_memoryCopy(pSiteMgr->hOs, tempName, &pBssid->Ssid.Ssid[0], pBssid->Ssid.SsidLength);
            tempName[pBssid->Ssid.SsidLength] ='\0';

            WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                    ("BSSID NAME = %s\n",tempName)) ;
#endif


            /* make sure length is 4 bytes aligned */
            if (pBssid->Length % 4)
            {
                pBssid->Length += (4 - (pBssid->Length % 4));
            }

            WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                    ("BSSID length after alignment = %d\n", pBssid->Length));

            pData += pBssid->Length;
            length += pBssid->Length;


            i++;
        }

        bssidList->NumberOfItems += i;

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


    WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                      ("Exiting getBssidList, site count = %i, length = %d, pData = 0x%X\n",
                      bssidList->NumberOfItems, length, (UINT32)pData));

    *pLength = length;

    return OK;
}

/***********************************************************************
 *                        getPrimaryBssid
 ***********************************************************************
DESCRIPTION:    Called by the OS abstraction layer in order to get the BSSID list from the site table

INPUT:      pSiteMgr    -   site mgr handle.

OUTPUT:     bssidList   -   BSSID list pointer

RETURN:

************************************************************************/
static TI_STATUS getPrimaryBssid(siteMgr_t *pSiteMgr, OS_802_11_BSSID_EX *primaryBssid, UINT32 *pLength)
{
    siteEntry_t *pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;
    UINT32                  len, firstOFDMloc = 0;
    OS_802_11_FIXED_IEs     *pFixedIes;
    OS_802_11_VARIABLE_IEs  *pVarIes;
    UINT32                  length;


    if ((primaryBssid==NULL) || (pLength==NULL))
    {
        *pLength = 0;
        WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                          ("getPrimaryBssid, one of the ptr is NULL, primaryBssid=%p,pLength=%p  \n",
                           primaryBssid, pLength));
        return NOK;

    }

    if (pPrimarySite==NULL)
    {
        *pLength = 0;
        WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                          ("getPrimaryBssid, pPrimarySite is NULL \n"));
        return NOK;

    }
    length = pPrimarySite->beaconLength + sizeof(OS_802_11_BSSID_EX) + sizeof(OS_802_11_FIXED_IEs);
    if (length > *pLength)
    {
        WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                          ("getPrimaryBssid, insufficient length,  required length=%d, pLength=%d \n",
                           length, *pLength));
        *pLength = length;

        return NOK;
    }

    WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("Entering getPrimaryBssid, length = %d\n", *pLength));

    primaryBssid->Length = length; 
    /* MacAddress */
    os_memoryCopy(pSiteMgr->hOs, (void *)primaryBssid->MacAddress, (void *)&(pPrimarySite->bssid.addr), MAC_ADDR_LEN);
        
    /* Capabilities */
    primaryBssid->Union.Capabilities = pPrimarySite->capabilities;

    /* SSID */
    os_memoryZero(pSiteMgr->hOs, &(primaryBssid->Ssid.Ssid), MAX_SSID_LEN);
    if (pPrimarySite->ssid.len > MAX_SSID_LEN)
    {
        pPrimarySite->ssid.len = MAX_SSID_LEN;
    }
    os_memoryCopy(pSiteMgr->hOs, (void *)primaryBssid->Ssid.Ssid, (void *)&(pPrimarySite->ssid.ssidString), pPrimarySite->ssid.len);
    primaryBssid->Ssid.SsidLength = pPrimarySite->ssid.len;

    /* privacy */
    primaryBssid->Privacy = pPrimarySite->privacy;

    /* RSSI */
    primaryBssid->Rssi = pPrimarySite->rssi;

    /* NetworkTypeInUse & SupportedRates */
    /* SupportedRates */
    os_memoryZero(pSiteMgr->hOs, (void *)primaryBssid->SupportedRates, sizeof(OS_802_11_RATES_EX));

    bitMapToNetworkStringRates(pPrimarySite->rateMask.supportedRateMask,
                               pPrimarySite->rateMask.basicRateMask,
                               (UINT8 *)primaryBssid->SupportedRates,
                               &len, &firstOFDMloc);

    /* set network type acording to band and rates */
    if (pPrimarySite->channel <= SITE_MGR_CHANNEL_B_G_MAX)
    {
        if (firstOFDMloc == len)
        {
            primaryBssid->NetworkTypeInUse = os802_11DS;
        } else {
            primaryBssid->NetworkTypeInUse = os802_11OFDM24;
        }
    } else {
        primaryBssid->NetworkTypeInUse = os802_11OFDM5;
    }

    /* Configuration */
    primaryBssid->Configuration.Length = sizeof(OS_802_11_CONFIGURATION);
    primaryBssid->Configuration.BeaconPeriod = pPrimarySite->beaconInterval;
    primaryBssid->Configuration.ATIMWindow = pPrimarySite->atimWindow;
    primaryBssid->Configuration.Union.channel = Chan2Freq(pPrimarySite->channel);

    primaryBssid->Configuration.FHConfig.Length = sizeof(OS_802_11_CONFIGURATION_FH);
    primaryBssid->Configuration.FHConfig.DwellTime = pPrimarySite->FHParams.dwellTime;
    primaryBssid->Configuration.FHConfig.HopPattern = pPrimarySite->FHParams.hopPattern;
    primaryBssid->Configuration.FHConfig.HopSet = pPrimarySite->FHParams.hopSet;

    /* InfrastructureMode */
    if  (pPrimarySite->bssType == BSS_INDEPENDENT)
        primaryBssid->InfrastructureMode = os802_11IBSS;
    else
        primaryBssid->InfrastructureMode = os802_11Infrastructure;



    primaryBssid->IELength = 0;

    /* copy fixed IEs from site entry */
    pFixedIes = (OS_802_11_FIXED_IEs*)&primaryBssid->IEs[primaryBssid->IELength];
    os_memoryCopy(pSiteMgr->hOs, (void *)pFixedIes->TimeStamp, (void *)&pPrimarySite->localTimeStamp, sizeof(pPrimarySite->localTimeStamp));
    pFixedIes->BeaconInterval = pPrimarySite->beaconInterval;
    pFixedIes->Capabilities = pPrimarySite->capabilities;
    primaryBssid->IELength += sizeof(OS_802_11_FIXED_IEs);
    pVarIes = (OS_802_11_VARIABLE_IEs*)&primaryBssid->IEs[primaryBssid->IELength];

    /* Coppy all variable IEs */
    WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                            ("Coppy all variable beaconLength=%d, IELength=%d\n", 
                             pPrimarySite->beaconLength, primaryBssid->IELength));
    WLAN_REPORT_HEX_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                pPrimarySite->beaconBuffer, pPrimarySite->beaconLength);

    os_memoryCopy(pSiteMgr->hOs, pVarIes, pPrimarySite->beaconBuffer, pPrimarySite->beaconLength);

    primaryBssid->IELength += pPrimarySite->beaconLength;


    primaryBssid->Length = sizeof(OS_802_11_BSSID_EX) + primaryBssid->IELength - 1;

    WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                            ("BSSID MAC = %x-%x-%x-%x-%x-%x\n",
                             primaryBssid->MacAddress[0], primaryBssid->MacAddress[1], primaryBssid->MacAddress[2],
                             primaryBssid->MacAddress[3], primaryBssid->MacAddress[4], primaryBssid->MacAddress[5]));


    WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                            ("primaryBssid is\n"));
    WLAN_REPORT_HEX_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                (UINT8*)primaryBssid, primaryBssid->Length);


    WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                      ("Exiting getBssidList, length =%d, IELength=%d \n", 
                       primaryBssid->Length,
                       primaryBssid->IELength));

    *pLength = primaryBssid->Length;

    return OK;
}

/***********************************************************************
 *                        siteMgr_printPrimarySiteDesc
 ***********************************************************************
DESCRIPTION:    Called by the OS abstraction layer in order to get the primary site description

INPUT:      pSiteMgr            -   site mgr handle.
            supplyExtendedInfo  - If OS_802_11_BSSID_EX structure should be used (extended info)
                                  (Assuming that if this function is called with TRUE, enough memory was allocated to hold the extended info)

OUTPUT:     pPrimarySiteDesc    -   Primary site description pointer

RETURN:

************************************************************************/
void siteMgr_printPrimarySiteDesc(TI_HANDLE hSiteMgr )
{
    siteMgr_t *pSiteMgr = (siteMgr_t*) hSiteMgr;
#ifdef REPORT_LOG
    siteEntry_t *pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;
    macAddress_t *bssid = &pPrimarySite->bssid;
#endif

    WLAN_REPORT_INFORMATION( pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                             ("-- SSID  = %s \n",pPrimarySite->ssid.ssidString) );
    WLAN_REPORT_INFORMATION( pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                             ("-- BSSID = %02X-%02X-%02X-%02X-%02X-%02X\n", 
                              bssid->addr[0], bssid->addr[1], bssid->addr[2], 
                              bssid->addr[3], bssid->addr[4], bssid->addr[5]) );
}

/***********************************************************************
 *                        getPrimarySiteDesc
 ***********************************************************************
DESCRIPTION:    Called by the OS abstraction layer in order to get the primary site description

INPUT:      pSiteMgr            -   site mgr handle.
            supplyExtendedInfo  - If OS_802_11_BSSID_EX structure should be used (extended info)
                                  (Assuming that if this function is called with TRUE, enough memory was allocated to hold the extended info)

OUTPUT:     pPrimarySiteDesc    -   Primary site description pointer

RETURN:

************************************************************************/
static void getPrimarySiteDesc(siteMgr_t *pSiteMgr, OS_802_11_BSSID *pPrimarySiteDesc, BOOL supplyExtendedInfo)
{
    siteEntry_t *pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;
    OS_802_11_BSSID_EX *pExPrimarySiteDesc = (OS_802_11_BSSID_EX *) pPrimarySiteDesc;
    UINT32  len, firstOFDMloc = 0;
    OS_802_11_FIXED_IEs     *pFixedIes;
    OS_802_11_VARIABLE_IEs  *pVarIes;
    UINT8 rsnIeLength,index;
    OS_802_11_RATES_EX  SupportedRates;


    if (pPrimarySiteDesc == NULL)
    {
        return;
    }
    if (pPrimarySite == NULL)
    {
        os_memoryZero(pSiteMgr->hOs, pPrimarySiteDesc, sizeof(OS_802_11_BSSID));
        return;
    }

    WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG, 
                            ("getPrimarySiteDesc - enter\n"));

    
    /* If an "extended" request has been made - update the length accordingly */
   if (supplyExtendedInfo == FALSE)
    pPrimarySiteDesc->Length = sizeof(OS_802_11_BSSID);
   else
    pPrimarySiteDesc->Length = sizeof(OS_802_11_BSSID_EX);

    /* MacAddress */
    os_memoryCopy(pSiteMgr->hOs, (void *)pPrimarySiteDesc->MacAddress, (void *)&(pPrimarySite->bssid.addr), MAC_ADDR_LEN);

    /* Capabilities */
    pPrimarySiteDesc->Union.Capabilities = pPrimarySite->capabilities;

    /* SSID */
    os_memoryCopy(pSiteMgr->hOs, (void *)&(pPrimarySiteDesc->Ssid.Ssid[0]), (void *)&(pPrimarySite->ssid.ssidString[0]), pPrimarySite->ssid.len);
    pPrimarySiteDesc->Ssid.SsidLength = pPrimarySite->ssid.len;

    /* privacy */
    pPrimarySiteDesc->Privacy = pPrimarySite->privacy;

    /* RSSI */

    pPrimarySiteDesc->Rssi = pPrimarySite->rssi;

	bitMapToNetworkStringRates(pPrimarySite->rateMask.supportedRateMask,
							   pPrimarySite->rateMask.basicRateMask,
							   &SupportedRates[0],
							   &len, &firstOFDMloc);

    /* set network type acording to band and rates */
    if (pPrimarySite->channel <= SITE_MGR_CHANNEL_B_G_MAX)
    {
        if (firstOFDMloc == len)
        {
            pPrimarySiteDesc->NetworkTypeInUse = os802_11DS;
        } else {
            pPrimarySiteDesc->NetworkTypeInUse = os802_11OFDM24;
        }
    } else {
        pPrimarySiteDesc->NetworkTypeInUse = os802_11OFDM5;
    }


    pPrimarySiteDesc->Configuration.Length = sizeof(OS_802_11_CONFIGURATION);
    pPrimarySiteDesc->Configuration.BeaconPeriod = pPrimarySite->beaconInterval;
    pPrimarySiteDesc->Configuration.ATIMWindow = pPrimarySite->atimWindow;
    pPrimarySiteDesc->Configuration.Union.channel = pPrimarySite->channel;

    pPrimarySiteDesc->Configuration.FHConfig.Length = sizeof(OS_802_11_CONFIGURATION_FH);
    pPrimarySiteDesc->Configuration.FHConfig.DwellTime = pPrimarySite->FHParams.dwellTime;
    pPrimarySiteDesc->Configuration.FHConfig.HopPattern = pPrimarySite->FHParams.hopPattern;
    pPrimarySiteDesc->Configuration.FHConfig.HopSet = pPrimarySite->FHParams.hopSet;

    if  (pPrimarySite->bssType == BSS_INDEPENDENT)
        pPrimarySiteDesc->InfrastructureMode = os802_11IBSS;
    else
        pPrimarySiteDesc->InfrastructureMode = os802_11Infrastructure;

   /* SupportedRates */
   if (supplyExtendedInfo == FALSE)
      os_memoryZero(pSiteMgr->hOs, (void *)pPrimarySiteDesc->SupportedRates, sizeof(OS_802_11_RATES));
   else
      os_memoryZero(pSiteMgr->hOs, (void *)pPrimarySiteDesc->SupportedRates, sizeof(OS_802_11_RATES_EX));


   if (supplyExtendedInfo == FALSE)
       os_memoryCopy(pSiteMgr->hOs, (void *)pPrimarySiteDesc->SupportedRates, (void *)SupportedRates, sizeof(OS_802_11_RATES));
   else
       os_memoryCopy(pSiteMgr->hOs, (void *)pPrimarySiteDesc->SupportedRates, (void *)SupportedRates, sizeof(OS_802_11_RATES_EX));
   

   if (supplyExtendedInfo == TRUE)
   {
      pExPrimarySiteDesc->IELength = 0;

     /* copy fixed IEs from site entry */
     pFixedIes = (OS_802_11_FIXED_IEs*)&pExPrimarySiteDesc->IEs[pExPrimarySiteDesc->IELength];
     os_memoryCopy(pSiteMgr->hOs, (void *)pFixedIes->TimeStamp, (void *)&pPrimarySite->localTimeStamp, sizeof(pPrimarySite->localTimeStamp));
     pFixedIes->BeaconInterval = pPrimarySite->beaconInterval;
     pFixedIes->Capabilities = pPrimarySite->capabilities;
     pExPrimarySiteDesc->IELength += sizeof(OS_802_11_FIXED_IEs);

     /* copy variable IEs */
     /* copy SSID */
     pVarIes = (OS_802_11_VARIABLE_IEs*)&pExPrimarySiteDesc->IEs[pExPrimarySiteDesc->IELength];
     pVarIes->ElementID = SSID_IE_ID;
     pVarIes->Length = pPrimarySite->ssid.len;
     os_memoryCopy(pSiteMgr->hOs, (void *)pVarIes->data, (void *)pPrimarySite->ssid.ssidString, pPrimarySite->ssid.len);
     pExPrimarySiteDesc->IELength += (pVarIes->Length + 2);

     /* copy RATES */
     
     pVarIes = (OS_802_11_VARIABLE_IEs*)&pExPrimarySiteDesc->IEs[pExPrimarySiteDesc->IELength];
     pVarIes->ElementID = SUPPORTED_RATES_IE_ID;
     bitMapToNetworkStringRates(pPrimarySite->rateMask.supportedRateMask, pPrimarySite->rateMask.basicRateMask,
                                (UINT8 *)pVarIes->data, &len, &firstOFDMloc);
     pVarIes->Length = len;
     pExPrimarySiteDesc->IELength += (pVarIes->Length + 2);

     /* copy FH */
     pVarIes = (OS_802_11_VARIABLE_IEs*)&pExPrimarySiteDesc->IEs[pExPrimarySiteDesc->IELength];
     pVarIes->ElementID = FH_PARAMETER_SET_IE_ID;
     pVarIes->Length = DOT11_FH_PARAMS_ELE_LEN;
     os_memoryCopy(pSiteMgr->hOs, (void *)pVarIes->data, (void *)&pPrimarySite->FHParams.dwellTime, sizeof(pPrimarySite->FHParams.dwellTime));
     os_memoryCopy(pSiteMgr->hOs, (void *)&pVarIes->data[2], (void *)&pPrimarySite->FHParams.hopSet, sizeof(pPrimarySite->FHParams.hopSet));
     os_memoryCopy(pSiteMgr->hOs, (void *)&pVarIes->data[3], (void *)&pPrimarySite->FHParams.hopPattern, sizeof(pPrimarySite->FHParams.hopPattern));
     pVarIes->data[4] = 1;
     pExPrimarySiteDesc->IELength += (pVarIes->Length + 2);

     /* copy DS */
     pVarIes = (OS_802_11_VARIABLE_IEs*)&pExPrimarySiteDesc->IEs[pExPrimarySiteDesc->IELength];
     pVarIes->ElementID = DS_PARAMETER_SET_IE_ID;
     pVarIes->Length = DOT11_DS_PARAMS_ELE_LEN;
     os_memoryCopy(pSiteMgr->hOs, (void *)pVarIes->data, (void *)&pPrimarySite->channel, DOT11_DS_PARAMS_ELE_LEN);
     pExPrimarySiteDesc->IELength += (pVarIes->Length + 2);

     /* copy RSN information elements */
     rsnIeLength = 0;
     for (index=0; index<MAX_RSN_IE && pPrimarySite->pRsnIe[index].hdr.eleLen>0; index++)
       {
            pVarIes =  (OS_802_11_VARIABLE_IEs*)&pExPrimarySiteDesc->IEs[pExPrimarySiteDesc->IELength+rsnIeLength];
         pVarIes->ElementID = pPrimarySite->pRsnIe[index].hdr.eleId;
             pVarIes->Length = pPrimarySite->pRsnIe[index].hdr.eleLen;
             os_memoryCopy(pSiteMgr->hOs, (void *)pVarIes->data, (void *)pPrimarySite->pRsnIe[index].rsnIeData, pPrimarySite->pRsnIe[index].hdr.eleLen);
             rsnIeLength += pPrimarySite->pRsnIe[index].hdr.eleLen+2;
            WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                    ("RSN IE ID=%d, Length=%x\n", pVarIes->ElementID, pVarIes->Length));

            WLAN_REPORT_HEX_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG, (UINT8 *)pVarIes->data,pVarIes->Length);
         }

         pExPrimarySiteDesc->IELength += pPrimarySite->rsnIeLen;

         pExPrimarySiteDesc->Length = sizeof(OS_802_11_BSSID_EX) + pExPrimarySiteDesc->IELength - 1;

         WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                 ("BSSID MAC = %x-%x-%x-%x-%x-%x\n",
                                  pExPrimarySiteDesc->MacAddress[0], pExPrimarySiteDesc->MacAddress[1], pExPrimarySiteDesc->MacAddress[2],
                                  pExPrimarySiteDesc->MacAddress[3], pExPrimarySiteDesc->MacAddress[4], pExPrimarySiteDesc->MacAddress[5]));

         WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                 ("pExPrimarySiteDesc length before alignment = %d\n", pExPrimarySiteDesc->Length));

         /* make sure length is 4 bytes aligned */
         if (pExPrimarySiteDesc->Length % 4)
         {
            pExPrimarySiteDesc->Length += (4 - (pExPrimarySiteDesc->Length % 4));
         }

         WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                 ("pExPrimarySiteDesc length after alignment = %d\n", pExPrimarySiteDesc->Length));

        }

}


/***********************************************************************
 *                        translateRateMaskToValue
 ***********************************************************************
DESCRIPTION:    Called by the function 'siteMgr_config()' in order to translate the rate mask read
                from registry to rate value

INPUT:      pSiteMgr            -   site mgr handle.
            rateMask            -   Rate mask

OUTPUT:     The rate after the translation

RETURN:

************************************************************************/
static rate_e translateRateMaskToValue(siteMgr_t *pSiteMgr, UINT32 rateMask)
{
    if (rateMask & DRV_RATE_MASK_54_OFDM)
        return DRV_RATE_54M;
    if (rateMask & DRV_RATE_MASK_48_OFDM)
        return DRV_RATE_48M;
    if (rateMask & DRV_RATE_MASK_36_OFDM)
        return DRV_RATE_36M;
    if (rateMask & DRV_RATE_MASK_24_OFDM)
        return DRV_RATE_24M;
    if (rateMask & DRV_RATE_MASK_22_PBCC)
        return DRV_RATE_22M;
    if (rateMask & DRV_RATE_MASK_18_OFDM)
        return DRV_RATE_18M;
    if (rateMask & DRV_RATE_MASK_12_OFDM)
        return DRV_RATE_12M;
    if (rateMask & DRV_RATE_MASK_11_CCK)
        return DRV_RATE_11M;
    if (rateMask & DRV_RATE_MASK_9_OFDM)
        return DRV_RATE_9M;
    if (rateMask & DRV_RATE_MASK_6_OFDM)
        return DRV_RATE_6M;
    if (rateMask & DRV_RATE_MASK_5_5_CCK)
        return DRV_RATE_5_5M;
    if (rateMask & DRV_RATE_MASK_2_BARKER)
        return DRV_RATE_2M;
    if (rateMask & DRV_RATE_MASK_1_BARKER)
        return DRV_RATE_1M;

    WLAN_REPORT_WARNING(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("Translate rate mask to value, mask is 0x%X\n", rateMask));
    if(pSiteMgr->siteMgrOperationalMode != DOT11_A_MODE)
        return DRV_RATE_1M;
    else
        return DRV_RATE_6M;
}


/***********************************************************************
 *                        getSupportedRateSet
 ***********************************************************************
DESCRIPTION:    Called by the function 'siteMgr_getParam()' in order to get the supported rate set
                Build an array of network rates based on the max active & max basic rates

INPUT:      pSiteMgr            -   site mgr handle.

OUTPUT:     pRatesSet           -   The array built

RETURN:

************************************************************************/
static void getSupportedRateSet(siteMgr_t *pSiteMgr, rates_t *pRatesSet)
{
    UINT32 dontCareParam;
    UINT32 len = 0;

    bitMapToNetworkStringRates(pSiteMgr->pDesiredParams->siteMgrRegstrySuppRateMask,
                               pSiteMgr->pDesiredParams->siteMgrRegstryBasicRateMask,
                               (UINT8 *)pRatesSet->ratesString, &len, &dontCareParam);

    pRatesSet->len = (UINT8) len;
}

/***********************************************************************
 *                        setSupportedRateSet
 ***********************************************************************
DESCRIPTION:    Called by the function 'siteMgr_setParam()' in order to set the supported rate set
                Go over the input array and set the max active & max basic rates. (after translation from network
                rates to host rates ofcourse)
                If max basic rate is bigger than the mac active one, print an error.
                If the basic or active rate are different than the ones already stored by the site manager,
                it returns RE_SCAN_NEEDED

INPUT:      pSiteMgr            -   site mgr handle.
            pRatesSet           -   The rates array received

OUTPUT:

RETURN:     RE_SCAN_NEEDED if re scan is needed, OK on success

************************************************************************/
static TI_STATUS setSupportedRateSet(siteMgr_t *pSiteMgr, rates_t *pRatesSet)
{
    UINT8  i,j, drvRate;
    rate_e maxActiveRate = (rate_e)0, maxBasicRate = (rate_e)0;
    UINT32 suppBitMap, basicBitMap;
    static  rate_e basicRates_G[]  = {DRV_RATE_1M,DRV_RATE_2M,DRV_RATE_5_5M,DRV_RATE_11M};
    static rate_e basicRates_A[]  = {DRV_RATE_6M,DRV_RATE_12M,DRV_RATE_24M};
    rate_e* currentBasicRates;
    UINT32 currentBasicRatesLength;

#ifndef NET_BASIC_MASK
#define NET_BASIC_MASK      0x80    /* defined in common/src/utils/utils.c */
#endif

    if(pSiteMgr->siteMgrOperationalMode == DOT11_A_MODE)
    {
        currentBasicRates = &basicRates_A[0];
        currentBasicRatesLength = sizeof(basicRates_A) / sizeof(basicRates_A[0]);
    }
    else
    {
        currentBasicRates = &basicRates_G[0];
        currentBasicRatesLength = sizeof(basicRates_G) / sizeof(basicRates_G[0]);
    }
    
        /* Basic rates must be supported. If one of 1M,2M,5.5M,11M is not supported fail.*/

    for (j=0;j < currentBasicRatesLength;j++) 
    {
        for (i = 0; i < pRatesSet->len; i++)
        {
            drvRate = utilityToHostRate(pRatesSet->ratesString[i]);
            if ((drvRate & ( NET_BASIC_MASK-1)) == currentBasicRates[j])
                break;
        }
        /* not all the basic rates are supported! Failure*/
        if (i == pRatesSet->len) 
        {
            WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("Rates set must contain the basic set! Failing\n"));
            return PARAM_VALUE_NOT_VALID;
        }
    }
    
    for (i = 0; i < pRatesSet->len; i++)
    {
        drvRate = utilityToHostRate(pRatesSet->ratesString[i]);
        if(pSiteMgr->siteMgrOperationalMode == DOT11_A_MODE)
        {
            if(drvRate < DRV_RATE_6M)
            {
                WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("Notice, the driver configured in 11a mode, but CCK rate appears\n"));
                return PARAM_VALUE_NOT_VALID;
            }
        }
        else if(pSiteMgr->siteMgrOperationalMode == DOT11_B_MODE)
        {
            if(drvRate >= DRV_RATE_6M)
            {
                WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("Notice, the driver configured in 11b mode, but OFDM rate appears\n"));
                return PARAM_VALUE_NOT_VALID;
            }
        }
        maxActiveRate = MAX((rate_e)drvRate, maxActiveRate);
    }

    for (i = 0; i < pRatesSet->len; i++)
    {
        if (IS_BASIC_RATE(pRatesSet->ratesString[i]))
            maxBasicRate = MAX(utilityToHostRate(pRatesSet->ratesString[i]), maxBasicRate);
    }

    /* If the basic rate is bigger than the supported one, we print an error */
    if (pSiteMgr->pDesiredParams->siteMgrDesiredRatePair.maxBasic > pSiteMgr->pDesiredParams->siteMgrDesiredRatePair.maxActive)
    {
        WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("Notice, the rates configuration is invalid, basic rate is bigger than supported, Max Basic: %d Max Supported: %d\n", pSiteMgr->pDesiredParams->siteMgrDesiredRatePair.maxBasic, pSiteMgr->pDesiredParams->siteMgrDesiredRatePair.maxActive));
        return PARAM_VALUE_NOT_VALID;
    }

    pSiteMgr->pDesiredParams->siteMgrDesiredRatePair.maxActive = maxActiveRate;
    pSiteMgr->pDesiredParams->siteMgrDesiredRatePair.maxBasic = maxBasicRate;

    
    /* configure desired modulation */
    if(maxActiveRate == DRV_RATE_22M)
        pSiteMgr->pDesiredParams->siteMgrDesiredModulationType = DRV_MODULATION_PBCC;
    else if(maxActiveRate < DRV_RATE_22M)
        pSiteMgr->pDesiredParams->siteMgrDesiredModulationType = DRV_MODULATION_CCK;
    else
        pSiteMgr->pDesiredParams->siteMgrDesiredModulationType = DRV_MODULATION_OFDM;


    networkStringToBitMapSuppRates(&suppBitMap,
                                   (UINT8 *)pRatesSet->ratesString,
                                   pRatesSet->len);

    networkStringToBitMapBasicRates(&basicBitMap,
                                    (UINT8 *)pRatesSet->ratesString,
                                    pRatesSet->len);

    if((pSiteMgr->pDesiredParams->siteMgrRegstryBasicRateMask != basicBitMap) ||
       (pSiteMgr->pDesiredParams->siteMgrRegstrySuppRateMask != suppBitMap))
    {
        pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.supportedRateMask = suppBitMap;
        pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.basicRateMask = basicBitMap;
        pSiteMgr->pDesiredParams->siteMgrRegstryBasicRateMask = basicBitMap;
        pSiteMgr->pDesiredParams->siteMgrRegstrySuppRateMask = suppBitMap;
        /* Initialize Mutual Rates Matching */
        pSiteMgr->pDesiredParams->siteMgrMatchedBasicRateMask = pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.basicRateMask;
        pSiteMgr->pDesiredParams->siteMgrMatchedSuppRateMask = pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.supportedRateMask;
        pSiteMgr->pDesiredParams->siteMgrMatchedMaxBasicRate = getMaxRatefromBitmap(pSiteMgr->pDesiredParams->siteMgrMatchedBasicRateMask);
        pSiteMgr->pDesiredParams->siteMgrMatchedMaxActiveRate = getMaxRatefromBitmap(pSiteMgr->pDesiredParams->siteMgrMatchedSuppRateMask);

        return RE_SCAN_NEEDED;
    }

    return OK;
}

/***********************************************************************
 *                        pbccAlgorithm
 ***********************************************************************
DESCRIPTION:    Called by the following functions:
                -   systemConfig(), in the system configuration phase after the selection
                -   siteMgr_updateSite(), in a case of a primary site update & if a PBCC algorithm
                    is needed to be performed
                Performs the PBCC algorithm


INPUT:      hSiteMgr            -   site mgr handle.

OUTPUT:

RETURN:     OK on always

************************************************************************/
TI_STATUS pbccAlgorithm(TI_HANDLE hSiteMgr)
{
    siteMgr_t   *pSiteMgr = (siteMgr_t *)hSiteMgr;
    paramInfo_t param;
    siteEntry_t *pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;
    UINT32      bitMap = 0;
    BOOL        desiredTxRateSupported = FALSE;
    UINT32      supportedRateMask ;


    /* First set the data modulation. */
    param.paramType = CTRL_DATA_CURRENT_MODULATION_TYPE_PARAM;                  /* Current Modulation Type */
    param.content.ctrlDataCurrentModulationType = pSiteMgr->currentDataModulation;
    ctrlData_setParam(pSiteMgr->hCtrlData, &param);

    if (pPrimarySite->channel == SPECIAL_BG_CHANNEL)
        supportedRateMask = (getSupportedRateMaskForSpecialBGchannel() & pSiteMgr->pDesiredParams->siteMgrMatchedSuppRateMask) ;
    else
        supportedRateMask = pSiteMgr->pDesiredParams->siteMgrMatchedSuppRateMask ;

    validateDesiredTxRate(pSiteMgr->pDesiredParams->siteMgrCurrentDesiredTxRate,
                          pSiteMgr->currentDataModulation,
                          supportedRateMask,
                          &bitMap,
                          &desiredTxRateSupported);

    if(desiredTxRateSupported == FALSE)
    {
        WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("Notice, the rates configuration is invalid, desired tx rate is not supported => switched to auto"));
        pSiteMgr->pDesiredParams->siteMgrCurrentDesiredTxRate = DRV_RATE_AUTO;
    }

    param.paramType = CTRL_DATA_CURRENT_SUPPORTED_RATE_MASK_PARAM;
    param.content.ctrlDataCurrentRateMask = supportedRateMask;
    /* clear the 22Mbps bit in case the PBCC is not allowed */
    if(pSiteMgr->currentDataModulation != DRV_MODULATION_PBCC &&
       pSiteMgr->currentDataModulation != DRV_MODULATION_OFDM)
            param.content.ctrlDataCurrentRateMask &= ~DRV_RATE_MASK_22_PBCC;
    ctrlData_setParam(pSiteMgr->hCtrlData, &param);

    param.paramType = CTRL_DATA_CURRENT_ACTIVE_RATE_PARAM;

    param.content.ctrlDataCurrentActiveRate = pSiteMgr->pDesiredParams->siteMgrCurrentDesiredTxRate;
    ctrlData_setParam(pSiteMgr->hCtrlData, &param);

    /*
     * NOTE: THIS MUST BE AFTER SETTING PREAMBLE TYPE 
     */
    
    if (pSiteMgr->pDesiredParams->siteMgrCurrentDesiredTxRate == DRV_RATE_AUTO)
    {
        param.paramType = CTRL_DATA_RATE_CONTROL_ENABLE_PARAM;
        param.content.ctrlDataRateControlEnable = TRUE;
        ctrlData_setParam(pSiteMgr->hCtrlData, &param);
    }
    else
    {
        param.paramType = CTRL_DATA_RATE_CONTROL_ENABLE_PARAM;
        param.content.ctrlDataRateControlEnable = FALSE;
        ctrlData_setParam(pSiteMgr->hCtrlData, &param);
    }

    return OK;
}

/***********************************************************************
 *                        validateDesiredTxRate
 ***********************************************************************
DESCRIPTION:    Called by the following functions:
                -   pbccAlgorithm()


INPUT:      desiredTxRate           -   configured desired tx rate.
            desiredModulation       -   configured desired tx modulation.
            suppRates               -   rates, supported by both STA and primary site.

OUTPUT:     bitMap                  -   rates bit map to be set to the ctrlData module
            txDesiredRateSupported  -   indication whether the desired tx rate supported
                                        by both STA and the primary site.
RETURN:     OK on always

************************************************************************/
void validateDesiredTxRate(rate_e desiredTxRate, modulationType_e desiredModulation,
                           UINT32 suppRates, UINT32 *bitMap, BOOL *txDesiredRateSupported)
{

    UINT32  maskTable[DRV_RATE_MAX+1] =
                                        {DRV_RATE_MASK_AUTO,
                                         DRV_RATE_MASK_1_BARKER,
                                         DRV_RATE_MASK_2_BARKER,
                                         DRV_RATE_MASK_5_5_CCK,
                                         DRV_RATE_MASK_11_CCK,
                                         DRV_RATE_MASK_22_PBCC
                                         ,DRV_RATE_MASK_6_OFDM,
                                         DRV_RATE_MASK_9_OFDM,
                                         DRV_RATE_MASK_12_OFDM,
                                         DRV_RATE_MASK_18_OFDM,
                                         DRV_RATE_MASK_24_OFDM,
                                         DRV_RATE_MASK_36_OFDM,
                                         DRV_RATE_MASK_48_OFDM,
                                         DRV_RATE_MASK_54_OFDM
                                        };

    if(desiredModulation != DRV_MODULATION_PBCC)
    {
        if(desiredTxRate == DRV_RATE_22M)
            *bitMap = DRV_RATE_MASK_11_CCK;
        else
            *bitMap = maskTable[desiredTxRate];
    }
    else
        *bitMap = maskTable[desiredTxRate];

    if((suppRates & maskTable[desiredTxRate]) || (desiredTxRate == DRV_RATE_AUTO))
        *txDesiredRateSupported = TRUE;
    else
        *txDesiredRateSupported = FALSE;
}

/***********************************************************************
 *                        siteMgr_assocReport
 ***********************************************************************
DESCRIPTION:    Called by the following functions:
                -   assoc_recv()


INPUT:      hSiteMgr            -   siteMgr handle.
            capabilities        -   assoc rsp capabilities field.
			bCiscoAP			-   whether we are connected to a Cisco AP. Used for Tx Power Control adjustment
OUTPUT:

RETURN:     OK on always

************************************************************************/
TI_STATUS siteMgr_assocReport(TI_HANDLE hSiteMgr, UINT16 capabilities, BOOL bCiscoAP)
{
    siteMgr_t   *pSiteMgr = (siteMgr_t *)hSiteMgr;
    siteEntry_t *pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;

    /* handle AP's preamble capability */
    pPrimarySite->preambleAssRspCap = ((capabilities >> CAP_PREAMBLE_SHIFT) & CAP_PREAMBLE_MASK)  ? PREAMBLE_SHORT : PREAMBLE_LONG;

	/*
	 * Enable/Disable the ATx Power Control adjustment.
	 * When we are connected to Cisco AP - TX Power Control adjustment is disabled.
	 */
	pSiteMgr->bTempTxPowerEnabled = ( !bCiscoAP ) && ( pSiteMgr->pDesiredParams->TxPowerControlOn );

	WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
		("%s: Tx Power Control adjustment is %d\n",__FUNCTION__, pSiteMgr->bTempTxPowerEnabled));

    return OK;
}

/***********************************************************************
 *                        siteMgr_setWMEParamsSite
 ***********************************************************************
DESCRIPTION:    Set the WME params as received from the associated
                AP. The function is called by the QoS Mgr
                after receving association response succefully.

INPUT:      hSiteMgr    -   SiteMgr handle.

OUTPUT:     pDot11_WME_PARAM_t - pointer to the WME Param IE.

RETURN:     OK on always

************************************************************************/
TI_STATUS siteMgr_setWMEParamsSite(TI_HANDLE hSiteMgr,dot11_WME_PARAM_t *pDot11_WME_PARAM)
{
    siteMgr_t   *pSiteMgr = (siteMgr_t *)hSiteMgr;
    siteEntry_t *pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;

    if( pPrimarySite == NULL )
    {
        return OK;
    }

    if( pDot11_WME_PARAM == NULL )
    {
        pPrimarySite->WMESupported = FALSE;
        return OK;
    }

    /* Update the WME params */
    os_memoryCopy(pSiteMgr->hOs,&pPrimarySite->WMEParameters,&pDot11_WME_PARAM->WME_ACParameteres,sizeof(ACParameters_t));
    pPrimarySite->lastWMEParameterCnt = (pDot11_WME_PARAM->ACInfoField & dot11_WME_ACINFO_MASK);
    WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,("$$$$$$ WME parameters were updates according to association response, cntSeq = %d\n",pPrimarySite->lastWMEParameterCnt));

    return OK;
}


/***********************************************************************
 *                        siteMgr_getWMEParamsSite
 ***********************************************************************
DESCRIPTION:    Get the WME params as recieved from the associated
                AP. The function is called by the Qos Mgr in order
                to set all WME parameters to the core and Hal

INPUT:      hSiteMgr    -   SiteMgr handle.

OUTPUT:     pWME_ACParameters_t - pointer to the WME Param.

RETURN:     OK if there are valid WME parameters , NOK otherwise.

************************************************************************/
TI_STATUS siteMgr_getWMEParamsSite(TI_HANDLE hSiteMgr,ACParameters_t **pWME_ACParameters_t)
{
    siteMgr_t   *pSiteMgr = (siteMgr_t *)hSiteMgr;
    siteEntry_t *pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;

    if(pPrimarySite->WMESupported == TRUE)
    {
        *pWME_ACParameters_t = &pPrimarySite->WMEParameters;
        return OK;
    }
    else
    {
        *pWME_ACParameters_t = NULL;
        return NOK;
    }

}



/***********************************************************************
 *                        calculateBssidListSize
 ***********************************************************************
DESCRIPTION:    Calculate the size of memory buffer required for BSSID
                List.


INPUT:      hSiteMgr            -   site mgr handle.

OUTPUT:

RETURN:     OK on always

************************************************************************/
static TI_STATUS calculateBssidListSize(siteMgr_t *pSiteMgr, UINT32 *pLength, BOOL allVarIes)
{
    UINT32 siteTableIndex, tableIndex;
    UINT32 length;
    siteEntry_t* pSiteEntry;
    siteTablesParams_t* currTable;

    *pLength = sizeof(OS_802_11_BSSID_LIST_EX) - sizeof(OS_802_11_BSSID_EX);

    currTable = pSiteMgr->pSitesMgmtParams->pCurrentSiteTable;

    for (tableIndex = 0; tableIndex < NUM_OF_SITE_TABLE ; tableIndex++)
    {
        for (siteTableIndex = 0; siteTableIndex < currTable->maxNumOfSites; siteTableIndex++)
        {
            pSiteEntry = (siteEntry_t*)&currTable->siteTable[siteTableIndex];

            if (pSiteEntry->siteType != SITE_NULL)
            {
                if (allVarIes)
                {
                    length = sizeof(OS_802_11_BSSID_EX) + sizeof(OS_802_11_FIXED_IEs);
                    if (pSiteEntry->probeRecv)
                    {
                        length += pSiteEntry->probeRespLength;
                    }
                    else
                    {
                        length += pSiteEntry->beaconLength;
                    }
                             
                }
                else
                {
                    length = (sizeof(OS_802_11_BSSID_EX) + sizeof(OS_802_11_FIXED_IEs) +
                              (pSiteEntry->ssid.len + 2) + (MAX_SUPPORTED_RATES + 2) +
                              (DOT11_FH_PARAMS_ELE_LEN + 2) + (DOT11_DS_PARAMS_ELE_LEN +2) +
                              pSiteEntry->rsnIeLen);
                    /* WME information element by alcel*/
                    if (pSiteEntry->WMESupported)
                    {
                        /* length of element + header*/
                        length += (DOT11_WME_ELE_LEN + 2);
                    }
                }

                /* make sure length is 4 bytes aligned */
                if (length % 4)
                {
                    length += (4 - (length % 4));
                }

                *pLength += length;

                WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                        ("BSSID length =%d on table index %d \n", length, tableIndex));

            }
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

    return OK;
}








/***********************************************************************
 *                        siteMgr_setCurrentTable
 ***********************************************************************
DESCRIPTION:

INPUT:      hSiteMgr    -   SiteMgr handle.

OUTPUT:

RETURN:

************************************************************************/
void siteMgr_setCurrentTable(TI_HANDLE hSiteMgr, radioBand_e radioBand)
{
    siteMgr_t   *pSiteMgr = (siteMgr_t *)hSiteMgr;

    if(radioBand == RADIO_BAND_2_4_GHZ)
        pSiteMgr->pSitesMgmtParams->pCurrentSiteTable = &pSiteMgr->pSitesMgmtParams->dot11BG_sitesTables;
    else
        pSiteMgr->pSitesMgmtParams->pCurrentSiteTable = (siteTablesParams_t *)&pSiteMgr->pSitesMgmtParams->dot11A_sitesTables;
}



/***********************************************************************
 *                        siteMgr_updateRates
 ***********************************************************************
DESCRIPTION:

INPUT:      hSiteMgr    -   SiteMgr handle.

OUTPUT:

RETURN:

************************************************************************/

void siteMgr_updateRates(TI_HANDLE hSiteMgr, BOOL dot11a, BOOL updateToOS)
{
    UINT32                  statusData;
    rate_e                  txDesiredRate;
    UINT32                  localSuppRateMask, localBasicRateMask;

    siteMgr_t   *pSiteMgr = (siteMgr_t *)hSiteMgr;

    localSuppRateMask = pSiteMgr->pDesiredParams->siteMgrRegstrySuppRateMask;
    localBasicRateMask = pSiteMgr->pDesiredParams->siteMgrRegstryBasicRateMask;


    validateRatesVsBand(&localSuppRateMask, &localBasicRateMask, dot11a);

    pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.basicRateMask = localBasicRateMask;
    pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.supportedRateMask = localSuppRateMask;

     /* Initialize Mutual Rates Matching */
    pSiteMgr->pDesiredParams->siteMgrMatchedBasicRateMask = pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.basicRateMask;
    pSiteMgr->pDesiredParams->siteMgrMatchedSuppRateMask = pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.supportedRateMask;
    pSiteMgr->pDesiredParams->siteMgrMatchedMaxBasicRate = getMaxRatefromBitmap(pSiteMgr->pDesiredParams->siteMgrMatchedBasicRateMask);
    pSiteMgr->pDesiredParams->siteMgrMatchedMaxActiveRate = getMaxRatefromBitmap(pSiteMgr->pDesiredParams->siteMgrMatchedSuppRateMask);


    /*If we are in dual mode and we are only scanning A band we don't have to set the siteMgrCurrentDesiredTxRate.*/
    if (updateToOS == TRUE)
    {
        /* Validate that the masks and tx rate are OK */
        txDesiredRate = pSiteMgr->pDesiredParams->siteMgrRegstryDesiredTxRate;

        validateRates((UINT32 *)&pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.basicRateMask,
                      (UINT32 *)&pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.supportedRateMask,
                          (UINT32 *)&txDesiredRate,
                          &pSiteMgr->pDesiredParams->siteMgrDesiredModulationType, dot11a);

        pSiteMgr->pDesiredParams->siteMgrCurrentDesiredTxRate = txDesiredRate;
    }
    pSiteMgr->pDesiredParams->siteMgrDesiredRatePair.maxBasic = translateRateMaskToValue(pSiteMgr, pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.basicRateMask);
    pSiteMgr->pDesiredParams->siteMgrDesiredRatePair.maxActive = translateRateMaskToValue(pSiteMgr, pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.supportedRateMask);

    if (updateToOS == TRUE) {
        /* report the desired rate to OS */
        statusData = hostToUtilityRate(pSiteMgr->pDesiredParams->siteMgrDesiredRatePair.maxActive);

        EvHandlerSendEvent(pSiteMgr->hEvHandler, IPC_EVENT_LINK_SPEED, (UINT8 *)&statusData,sizeof(UINT32));
    }

}

/***********************************************************************
 *                        siteMgr_bandParamsConfig
 ***********************************************************************
DESCRIPTION:

INPUT:      hSiteMgr    -   SiteMgr handle.

OUTPUT:

RETURN:

************************************************************************/
void siteMgr_bandParamsConfig(TI_HANDLE hSiteMgr,  BOOL updateToOS)
{
    siteMgr_t   *pSiteMgr = (siteMgr_t *)hSiteMgr;

    /* reconfig rates */
    if(pSiteMgr->siteMgrOperationalMode == DOT11_A_MODE)
        siteMgr_updateRates(hSiteMgr, TRUE, updateToOS);
    else
        siteMgr_updateRates(hSiteMgr, FALSE, updateToOS);

    /* go to B_ONLY Mode only if WiFI bit is Set*/
    if (pSiteMgr->pDesiredParams->siteMgrWiFiAdhoc == TRUE)
    {   /* Configuration For AdHoc when using external configuration */
        if (pSiteMgr->pDesiredParams->siteMgrExternalConfiguration == FALSE)
        {
            siteMgr_externalConfigurationParametersSet(hSiteMgr);
        }
    }

}

void siteMgr_keepAliveSendNullDataTimer(TI_HANDLE hSiteMgr)
{
    paramInfo_t param;
    siteMgr_t   *pSiteMgr = (siteMgr_t *)hSiteMgr;
    UINT32      txPacketsCount = 0;
    UINT32      TxQid;

    if ((pSiteMgr->pSitesMgmtParams->pPrimarySite != NULL) &&
        (pSiteMgr->pSitesMgmtParams->pPrimarySite->bssType != BSS_INFRASTRUCTURE))
    {   /* No need for kepp alive when not in Infra */
        return;
    }
    param.paramType = TX_DATA_COUNTERS_PARAM;
    txData_getParam(pSiteMgr->hTxData, &param);
    /* get current tx data frames counter */
    for (TxQid = 0; TxQid < MAX_NUM_OF_TX_QUEUES; TxQid++)
        txPacketsCount += param.content.pTxDataCounters[TxQid].XmitOk;

    if (pSiteMgr->txPacketsCount==txPacketsCount)
    {  /* send NULL data */
        WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,("siteMgr_keepAliveSendNullDataTimer\n"));

        /* sending null frame with power save bit set to off (if keepAliveEnable in the registry)
          Note: the Hardware modify the PS bit according to the current PS mode. */
        if(pSiteMgr->keepAliveEnable == TRUE)
        {
            txData_sendNullFrame(pSiteMgr->hTxData, FALSE, SITE_MGR_MODULE);
        }
        txPacketsCount++;

    }
    pSiteMgr->txPacketsCount=txPacketsCount;

}

void siteMgr_ConfigRate(TI_HANDLE hSiteMgr)
{
    siteMgr_t   *pSiteMgr = (siteMgr_t *)hSiteMgr;
    BOOL        dot11a;
    dot11mode_e OperationMode ;

    OperationMode = pSiteMgr->siteMgrOperationalMode;

    /* reconfig rates */
    if(OperationMode == DOT11_A_MODE)
        dot11a = TRUE;
    else
        dot11a = FALSE;

    /*
    ** Specific change to ch 14, that channel is only used in Japan, and is limited
    ** to rates 1,2,5.5,11
    */
    if(pSiteMgr->pDesiredParams->siteMgrDesiredChannel == SPECIAL_BG_CHANNEL)
    {
        if(pSiteMgr->pDesiredParams->siteMgrRegstryBasicRate[OperationMode] > BASIC_RATE_SET_1_2_5_5_11)
            pSiteMgr->pDesiredParams->siteMgrRegstryBasicRate[OperationMode] = BASIC_RATE_SET_1_2_5_5_11;


        if(pSiteMgr->pDesiredParams->siteMgrRegstrySuppRate[OperationMode] > SUPPORTED_RATE_SET_1_2_5_5_11)
            pSiteMgr->pDesiredParams->siteMgrRegstrySuppRate[OperationMode] = SUPPORTED_RATE_SET_1_2_5_5_11;
    }

    pSiteMgr->pDesiredParams->siteMgrRegstryBasicRateMask =
        translateBasicRateValueToMask(pSiteMgr->pDesiredParams->siteMgrRegstryBasicRate[OperationMode], dot11a);

    pSiteMgr->pDesiredParams->siteMgrRegstrySuppRateMask =
        translateSupportedRateValueToMask(pSiteMgr->pDesiredParams->siteMgrRegstrySuppRate[OperationMode], dot11a);

    siteMgr_updateRates(pSiteMgr, dot11a, TRUE);

        /* go to B_ONLY Mode only if WiFI bit is Set*/
    if (pSiteMgr->pDesiredParams->siteMgrWiFiAdhoc == TRUE)
    {   /* Configuration For AdHoc when using external configuration */
        if (pSiteMgr->pDesiredParams->siteMgrExternalConfiguration == FALSE)
        {
            siteMgr_externalConfigurationParametersSet(hSiteMgr);
        }
    }
}

static void siteMgr_externalConfigurationParametersSet(TI_HANDLE hSiteMgr)
{
    siteMgr_t   *pSiteMgr = (siteMgr_t *)hSiteMgr;

    /* Overwrite the parameters for AdHoc with External Configuration */

    if( ((pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode == DOT11_A_MODE) ||
        (pSiteMgr->pDesiredParams->siteMgrDesiredDot11Mode == DOT11_DUAL_MODE)) &&
        !pSiteMgr->pDesiredParams->siteMgrWiFiAdhoc && pSiteMgr->pDesiredParams->siteMgrDesiredBSSType == BSS_INDEPENDENT)
        return;


    if(pSiteMgr->pDesiredParams->siteMgrDesiredBSSType == BSS_INDEPENDENT)
    {
        pSiteMgr->siteMgrOperationalMode = DOT11_B_MODE;
        pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.basicRateMask = translateBasicRateValueToMask(BASIC_RATE_SET_1_2_5_5_11, FALSE);
        pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.supportedRateMask = translateSupportedRateValueToMask(SUPPORTED_RATE_SET_1_2_5_5_11, FALSE);
        pSiteMgr->pDesiredParams->siteMgrCurrentDesiredTxRate = DRV_RATE_AUTO;
        pSiteMgr->pDesiredParams->siteMgrRegstryBasicRateMask = translateBasicRateValueToMask(BASIC_RATE_SET_1_2_5_5_11, FALSE);
        pSiteMgr->pDesiredParams->siteMgrRegstrySuppRateMask = translateSupportedRateValueToMask(SUPPORTED_RATE_SET_1_2_5_5_11, FALSE);
        pSiteMgr->pDesiredParams->siteMgrRegstryDesiredTxRate = DRV_RATE_AUTO;
        pSiteMgr->pDesiredParams->siteMgrDesiredSlotTime = PHY_SLOT_TIME_LONG;

        whalCtrl_SetRadioBand(pSiteMgr->hHalCtrl, RADIO_BAND_2_4_GHZ);
        whalCtrl_SetSlotTime(pSiteMgr->hHalCtrl, PHY_SLOT_TIME_LONG);

    }
    else
    {
        if(pSiteMgr->radioBand == RADIO_BAND_2_4_GHZ)
            pSiteMgr->siteMgrOperationalMode = DOT11_G_MODE;
        else
            pSiteMgr->siteMgrOperationalMode = DOT11_A_MODE;
        pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.basicRateMask = translateBasicRateValueToMask(BASIC_RATE_SET_1_2_5_5_11, FALSE);
        pSiteMgr->pDesiredParams->siteMgrCurrentDesiredRateMask.supportedRateMask = translateSupportedRateValueToMask(SUPPORTED_RATE_SET_ALL, FALSE);
        pSiteMgr->pDesiredParams->siteMgrCurrentDesiredTxRate = DRV_RATE_AUTO;
        pSiteMgr->pDesiredParams->siteMgrRegstryBasicRateMask = translateBasicRateValueToMask(BASIC_RATE_SET_1_2_5_5_11, FALSE);
        pSiteMgr->pDesiredParams->siteMgrRegstrySuppRateMask = translateSupportedRateValueToMask(SUPPORTED_RATE_SET_ALL, FALSE);
        pSiteMgr->pDesiredParams->siteMgrRegstryDesiredTxRate = DRV_RATE_AUTO;

        pSiteMgr->pDesiredParams->siteMgrDesiredSlotTime = PHY_SLOT_TIME_LONG;

        whalCtrl_SetSlotTime(pSiteMgr->hHalCtrl, PHY_SLOT_TIME_LONG);
    }
}

void siteMgr_checkTxPower(TI_HANDLE hSiteMgr)
{
    siteMgr_t           *pSiteMgr = (siteMgr_t *)hSiteMgr;

    if ( (pSiteMgr->pSitesMgmtParams->pPrimarySite) && (pSiteMgr->bTempTxPowerEnabled)
        && (++pSiteMgr->siteMgrTxPowerCheckTime >= pSiteMgr->pDesiredParams->TxPowerCheckTime) )
        {
            pSiteMgr->siteMgrTxPowerCheckTime = 0; /* reset counter for next check */

            WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                ("%s: RSSI = %d TxPowerRssiThresh = %d TxPowerRssiRestoreThresh = %D\n", 
				__FUNCTION__, pSiteMgr->pSitesMgmtParams->pPrimarySite->rssi,
				pSiteMgr->pDesiredParams->TxPowerRssiThresh,pSiteMgr->pDesiredParams->TxPowerRssiRestoreThresh));

        if ((pSiteMgr->pSitesMgmtParams->pPrimarySite->rssi) >= (INT32) (0 - pSiteMgr->pDesiredParams->TxPowerRssiThresh))
        {
            /* activate Tx Power Control adjustment */
			siteMgr_setTemporaryTxPower(pSiteMgr, TRUE);  
        }
        else if (pSiteMgr->pSitesMgmtParams->pPrimarySite->rssi <= (INT32) (0 - pSiteMgr->pDesiredParams->TxPowerRssiRestoreThresh))
        {
			/* deactivate Tx Power Control adjustment */
			siteMgr_setTemporaryTxPower(pSiteMgr, FALSE);  
        }
    }
}

TI_STATUS siteMgr_saveProbeRespBuffer(TI_HANDLE hSiteMgr, macAddress_t  *bssid, UINT8 *pProbeRespBuffer, UINT32 length)
{
    siteEntry_t *pSite;
    siteMgr_t   *pSiteMgr = (siteMgr_t *)hSiteMgr;

    if ((pSiteMgr==NULL) || (pProbeRespBuffer==NULL) || (length>=MAX_MGMT_BODY_LENGTH))
    {
        return NOK;
    }

    pSite = findSiteEntry(pSiteMgr, bssid);
    if (pSite==NULL)
    {
        /*WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                          ("siteMgr_saveProbeRespBuffer: site doesn't exist\n"));*/
        return NOK;

    }

    os_memoryCopy(pSiteMgr->hOs, pSite->probeRespBuffer, pProbeRespBuffer, length);
    pSite->osTimeStamp = os_timeStampMs(pSiteMgr->hOs);
    pSite->probeRespLength = length;

    WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                          ("siteMgr_saveProbeRespBuffer: BSSID=%x-%x-%x-%x-%x-%x, TSF=%x-%x-%x-%x-%x-%x-%x-%x, \n ts=%d, rssi=%d\n channel = %d \n",
                           pSite->bssid.addr[0], pSite->bssid.addr[1], pSite->bssid.addr[2],
                           pSite->bssid.addr[3], pSite->bssid.addr[4], pSite->bssid.addr[5],
                           pSite->tsfTimeStamp[0], pSite->tsfTimeStamp[1], pSite->tsfTimeStamp[2], pSite->tsfTimeStamp[3],
                           pSite->tsfTimeStamp[4], pSite->tsfTimeStamp[5], pSite->tsfTimeStamp[6], pSite->tsfTimeStamp[7],
                           pSite->osTimeStamp,  pSite->rssi, pSite->channel));

    return OK;
}

TI_STATUS siteMgr_saveBeaconBuffer(TI_HANDLE hSiteMgr, macAddress_t *bssid, UINT8 *pBeaconBuffer, UINT32 length)
{
    siteEntry_t *pSite;
    siteMgr_t   *pSiteMgr = (siteMgr_t *)hSiteMgr;

    if ((pSiteMgr==NULL) || (pBeaconBuffer==NULL) || (length>=MAX_MGMT_BODY_LENGTH))
    {
        return NOK;
    }

    pSite = findSiteEntry(pSiteMgr, bssid);
    if (pSite==NULL)
    {
        /*WLAN_REPORT_ERROR(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                          ("siteMgr_saveBeaconBuffer: site doesn't exist\n"));*/
        return NOK;

    }

    os_memoryCopy(pSiteMgr->hOs, pSite->beaconBuffer, pBeaconBuffer, length);
    pSite->osTimeStamp = os_timeStampMs(pSiteMgr->hOs);
    pSite->beaconLength = length;

    /*if (pSiteMgr->pSitesMgmtParams->pPrimarySite!=NULL)
    {
        if (!os_memoryCompare(pSiteMgr->hOs, pSiteMgr->pSitesMgmtParams->pPrimarySite->ssid.ssidString, pSite->ssid.ssidString, pSiteMgr->pSitesMgmtParams->pPrimarySite->ssid.len))
        {
            WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                                  ("siteMgr_saveBeaconBuffer: BSSID=%x-%x-%x-%x-%x-%x, TSF=%x-%x-%x-%x-%x-%x-%x-%x, \n ts=%d, rssi=%d \n",
                                   pSite->bssid.addr[0], pSite->bssid.addr[1], pSite->bssid.addr[2],
                                   pSite->bssid.addr[3], pSite->bssid.addr[4], pSite->bssid.addr[5],
                                   pSite->tsfTimeStamp[0], pSite->tsfTimeStamp[1], pSite->tsfTimeStamp[2], pSite->tsfTimeStamp[3],
                                   pSite->tsfTimeStamp[4], pSite->tsfTimeStamp[5], pSite->tsfTimeStamp[6], pSite->tsfTimeStamp[7],
                                   pSite->osTimeStamp,  pSite->rssi));
        }
    }*/
    pSite->ssid.ssidString[pSite->ssid.len] = '\0';

    /*WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,
                          ("siteMgr_saveBeaconBuffer: BSSID=%x-%x-%x-%x-%x-%x, SSID=%s, \n ts=%d, rssi=%d\n channel = %d \n",
                           pSite->bssid.addr[0], pSite->bssid.addr[1], pSite->bssid.addr[2],
                           pSite->bssid.addr[3], pSite->bssid.addr[4], pSite->bssid.addr[5],
                           pSite->ssid.ssidString, pSite->osTimeStamp,  pSite->rssi, pSite->channel));

    */
    return OK;
}


void siteMgr_resetChannelList(TI_HANDLE hSiteMgr)
{
    UINT8       index;
    siteMgr_t   *pSiteMgr = (siteMgr_t*)hSiteMgr;

    if (hSiteMgr==NULL)
    {
        return;
    }

    for (index=0; index < pSiteMgr->pSitesMgmtParams->pCurrentSiteTable->maxNumOfSites; index++)
    {
        if (pSiteMgr->pSitesMgmtParams->pCurrentSiteTable->siteTable[index].siteType != SITE_NULL)
        {
            pSiteMgr->pSitesMgmtParams->pCurrentSiteTable->siteTable[index].prioritySite = FALSE;
        }
    }

}


siteEntry_t *siteMgr_findSiteEntry(TI_HANDLE hSiteMgrm, macAddress_t *bssid)
{
    return (findSiteEntry(hSiteMgrm, bssid));
}



void siteMgr_IsERP_Needed(TI_HANDLE hSiteMgr,BOOL *useProtection,BOOL *NonErpPresent,BOOL *barkerPreambleType)
{
    siteMgr_t       *pSiteMgr = (siteMgr_t*)hSiteMgr;
    paramInfo_t     param;
    siteEntry_t     *pPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;

    *useProtection = FALSE;
    *NonErpPresent = FALSE;
    *barkerPreambleType = FALSE;

    param.paramType = CTRL_DATA_CURRENT_IBSS_PROTECTION_PARAM;
    ctrlData_getParam(pSiteMgr->hCtrlData, &param);

    /* On WifiAdhoc (for band B) The STa should not include in the beacon an ERP IE (see WiFi B  clause 2.2, 5.8.2) */
    if (pSiteMgr->pDesiredParams->siteMgrWiFiAdhoc == TRUE)
    {
        /* Return the default => ERP is not needed */
        return;
    }

    /* check if STA is connected */
    if (pPrimarySite)
    {
        if(pSiteMgr->siteMgrOperationalMode == DOT11_G_MODE || pSiteMgr->siteMgrOperationalMode == DOT11_DUAL_MODE)
        {
            if(param.content.ctrlDataIbssProtecionType == ERP_PROTECTION_STANDARD)
            {
                if(pPrimarySite->siteType == SITE_SELF)
                {
                    if(pPrimarySite->channel <= SITE_MGR_CHANNEL_B_G_MAX) /* if channel B&G*/
                    {
                            *useProtection = TRUE;
                            *NonErpPresent = TRUE;
                            *barkerPreambleType = TRUE;
                    }
                }
                else if(pPrimarySite->bssType == BSS_INDEPENDENT)
                {
                    if(pPrimarySite->useProtection == TRUE)
                        *useProtection = TRUE;
                    if(pPrimarySite->NonErpPresent == TRUE)
                        *NonErpPresent = TRUE;
                    if(pPrimarySite->barkerPreambleType == PREAMBLE_SHORT)
                        *barkerPreambleType = TRUE;
                }
            }
        }
    }
}




void siteMgr_gotFirstBcn(TI_HANDLE hSiteMgr)
{
        siteMgr_t   *pSiteMgr = (siteMgr_t*)hSiteMgr;
        int status ;

    WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  
                                ("siteMgr_gotFirstBcn: dtimPeriod=%d, beaconInterval=%d\n",
                                pSiteMgr->pSitesMgmtParams->pPrimarySite->dtimPeriod,
                                pSiteMgr->pSitesMgmtParams->pPrimarySite->beaconInterval));

    /* Check if the Beacon Filter Desired State is TRUE */
    pSiteMgr->beaconFilterParams.currentState = TRUE;
    /* If the Beacon Filter was TRUE then send it to FW if not do not do nothing */
    if( TRUE == pSiteMgr->beaconFilterParams.desiredState )
    {
        if ( (status = whalCtrl_SetBeaconFiltering(pSiteMgr->hHalCtrl, pSiteMgr->beaconFilterParams.desiredState, pSiteMgr->beaconFilterParams.numOfStored)) != OK)
        {
            WLAN_REPORT_INFORMATION(pSiteMgr->hReport,
                              POWER_MANAGER_MODULE_LOG,
                              ("%s(%d) - Error N.%d in configuring beacon filtering !\n",
                              __FILE__,__LINE__,status));
        }
        else
        {           
            WLAN_REPORT_INFORMATION(pSiteMgr->hReport,SITE_MGR_MODULE_LOG , ("--          Beacon Filter Enable Beacon Filtering MIB Was Sent to FW !!!  --\n"));
        }
    }
    else
    {
            WLAN_REPORT_INFORMATION(pSiteMgr->hReport,SITE_MGR_MODULE_LOG , ("--          Beacon Filter Disable Beacon Filtering - Do not send to FW Already Disabled !!!  --\n"));
    }

    whalCtrl_setDtimPeriod(pSiteMgr->hHalCtrl, pSiteMgr->pSitesMgmtParams->pPrimarySite->dtimPeriod,
                            pSiteMgr->pSitesMgmtParams->pPrimarySite->beaconInterval);

    /* The power mode configuration gets overridden by the above DTIM period */
    /* configuration. We use this call to send it again to the firmware. */
    PowerMgr_reloadPowerMode(pSiteMgr->hPowerMgr);
}

void siteMgr_clearFirstBcnFlag(TI_HANDLE hSiteMgr)
{
    siteMgr_t   *pSiteMgr = (siteMgr_t*)hSiteMgr;

    /* This is set to FALSE since the First Beaqcin should be received for the first beacon mechanism */
	pSiteMgr->beaconFilterParams.currentState = FALSE;
	whalCtrl_SetBeaconFiltering(pSiteMgr->hHalCtrl, FALSE, pSiteMgr->beaconFilterParams.numOfStored);
	

    /* set Hw available until we get the first beacon */
    MacServices_powerAutho_AwakeRequiredUpdate(pSiteMgr->hMacServices, POWERAUTHO_AWAKE_REQUIRED, POWERAUTHO_AWAKE_REASON_FIRST_BEACON);

    if(pSiteMgr->pSitesMgmtParams->pPrimarySite != NULL)
        pSiteMgr->pSitesMgmtParams->pPrimarySite->beaconReceiveAfterJoin = FALSE;
}

void siteMgr_setFirstBcnFlag(TI_HANDLE hSiteMgr)
{
    siteMgr_t   *pSiteMgr = (siteMgr_t*)hSiteMgr;

    /* set Hw not available until now that the connection was closed */
    MacServices_powerAutho_AwakeRequiredUpdate(pSiteMgr->hMacServices, POWERAUTHO_AWAKE_NOT_REQUIRED, POWERAUTHO_AWAKE_REASON_FIRST_BEACON);
    
    if(pSiteMgr->pSitesMgmtParams->pPrimarySite != NULL)
        pSiteMgr->pSitesMgmtParams->pPrimarySite->beaconReceiveAfterJoin = TRUE;
}

/**
*
* siteMgr_overwritePrimarySite
*
* \b Description: 
*
* This function sets new AP as a primary site and, if requested, stores previous
* AP's info; called during roaming
*
* \b ARGS:
*
*  I   - pCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS siteMgr_overwritePrimarySite(TI_HANDLE hSiteMgr, bssEntry_t *newAP, BOOL requiredToStorePrevSite)
{
    siteMgr_t   *pSiteMgr = (siteMgr_t *)hSiteMgr;
    siteEntry_t *newApEntry;
    mlmeIEParsingParams_t *ieListParseParams = mlmeParser_getParseIEsBuffer(pSiteMgr->hMlmeSm);

    WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("siteMgr_overwritePrimarySite: new site bssid= 0x%x-0x%x-0x%x-0x%x-0x%x-0x%x\n\n",
               newAP->BSSID.addr[0], newAP->BSSID.addr[1], newAP->BSSID.addr[2],
               newAP->BSSID.addr[3], newAP->BSSID.addr[4], newAP->BSSID.addr[5]));

    /* If previous primary site present, and requested to save it - store it */
    if (requiredToStorePrevSite)
    {
        WLAN_REPORT_INFORMATION(pSiteMgr->hReport,
                                SITE_MGR_MODULE_LOG,
                                ("siteMgr_overwritePrimarySite: required to store prev prim site \n"));
        /* Store latest primary site, make ite a regular site */
        pSiteMgr->pSitesMgmtParams->pPrevPrimarySite = pSiteMgr->pSitesMgmtParams->pPrimarySite;
        pSiteMgr->pSitesMgmtParams->pPrevPrimarySite->siteType = SITE_REGULAR;
    }
    else
    {
        WLAN_REPORT_INFORMATION(pSiteMgr->hReport,
                                SITE_MGR_MODULE_LOG,
                                ("siteMgr_overwritePrimarySite: not required to store prev prim site \n"));
        if (pSiteMgr->pSitesMgmtParams->pPrimarySite != NULL)
        {
            WLAN_REPORT_INFORMATION(pSiteMgr->hReport, SITE_MGR_MODULE_LOG,  ("Removing Primary ssid=%s, bssid= 0x%x-0x%x-0x%x-0x%x-0x%x-0x%x\n\n",
            pSiteMgr->pSitesMgmtParams->pPrimarySite->ssid.ssidString,
            pSiteMgr->pSitesMgmtParams->pPrimarySite->bssid.addr[0], pSiteMgr->pSitesMgmtParams->pPrimarySite->bssid.addr[1], pSiteMgr->pSitesMgmtParams->pPrimarySite->bssid.addr[2],
            pSiteMgr->pSitesMgmtParams->pPrimarySite->bssid.addr[3], pSiteMgr->pSitesMgmtParams->pPrimarySite->bssid.addr[4], pSiteMgr->pSitesMgmtParams->pPrimarySite->bssid.addr[5] ));

            pSiteMgr->pSitesMgmtParams->pPrimarySite->siteType  = SITE_REGULAR;
            pSiteMgr->pSitesMgmtParams->pPrimarySite->beaconRecv = FALSE;
            pSiteMgr->pSitesMgmtParams->pPrimarySite->beaconReceiveAfterJoin = TRUE;

            pSiteMgr->pSitesMgmtParams->pPrimarySite = NULL;
        }
        else
        {
            WLAN_REPORT_ERROR(pSiteMgr->hReport,
                                    SITE_MGR_MODULE_LOG,
                                    ("siteMgr_overwritePrimarySite: primary site is NULL \n"));
        }

    }

    /* Find not occupied entry in site table, and store new AP BSSID in */
    /* If pPrimarySite is not set to NULL, store it in pPrevSite before updating */
    newApEntry = findAndInsertSiteEntry(pSiteMgr, &(newAP->BSSID), newAP->band);

    if (newApEntry != NULL)
    {
        /* Zero frame content */
        os_memoryZero(pSiteMgr->hOs, ieListParseParams, sizeof(mlmeIEParsingParams_t));

        /* Update parameters of new AP */
        newApEntry->rssi            = newAP->RSSI;
        newApEntry->bssType         = BSS_INFRASTRUCTURE;
        newApEntry->dtimPeriod      = 1;
        newApEntry->rxRate          = (rate_e)newAP->rxRate;
        /* Mark the site as regular in order to prevent from calling Power manager during beacon parsing */
        newApEntry->siteType        = SITE_REGULAR; 

        os_memoryCopy(pSiteMgr->hOs, &newApEntry->ssid, &pSiteMgr->pDesiredParams->siteMgrDesiredSSID, sizeof(ssid_t));

        if (newAP->resultType == SCAN_RFT_PROBE_RESPONSE)
        {
            ieListParseParams->frame.subType = PROBE_RESPONSE;
            siteMgr_saveProbeRespBuffer(hSiteMgr, &(newAP->BSSID), newAP->pBuffer, newAP->bufferLength);
        }
        else
        {
            ieListParseParams->frame.subType = BEACON;
            siteMgr_saveBeaconBuffer(hSiteMgr, &(newAP->BSSID), newAP->pBuffer, newAP->bufferLength);
        }
        ieListParseParams->band = newAP->band;
        ieListParseParams->rxChannel = newAP->channel;
        ieListParseParams->myBssid = FALSE;

        ieListParseParams->frame.content.iePacket.pRsnIe = NULL;
        ieListParseParams->frame.content.iePacket.rsnIeLen = 0;
        ieListParseParams->frame.content.iePacket.barkerPreambleMode = PREAMBLE_UNSPECIFIED;
        os_memoryCopy(pSiteMgr->hOs, (void *)ieListParseParams->frame.content.iePacket.timestamp, (void *)&newAP->lastRxTSF, TIME_STAMP_LEN);
        ieListParseParams->frame.content.iePacket.beaconInerval     = newAP->beaconInterval;
        ieListParseParams->frame.content.iePacket.capabilities  = newAP->capabilities;

        if (mlmeParser_parseIEs(pSiteMgr->hMlmeSm, newAP->pBuffer, newAP->bufferLength, ieListParseParams) != OK)
        {
            /* Error in parsing Probe response packet - exit */
            return NOK;
        }

        siteMgr_updateSite(hSiteMgr, &(newAP->BSSID), &ieListParseParams->frame, newAP->channel, newAP->band, FALSE);

        /* Select the entry as primary site */
        newApEntry->siteType = SITE_PRIMARY;
        pSiteMgr->pSitesMgmtParams->pPrimarySite = newApEntry;
        return OK;
    }
    else
    {
        return NOK;
    }
}

#if 0
/***********************************************************************
                siteMgr_updateLowPriorityTimeout
 ***********************************************************************
DESCRIPTION: Call to the PowerCtrl with the new claculated low priority
             timeout that is depend on the:
             - short/long doze.
             - beacon filtering number.
             - beacon interval.

INPUT:      hSiteMgr    -   site mgr handle.

OUTPUT:

RETURN:     UINT32 - the timeout in mSec.

************************************************************************/
static void siteMgr_updateLowPriorityTimeout(TI_HANDLE hSiteMgr)
{
    siteMgr_t *pSiteMgr = (siteMgr_t*)hSiteMgr;
    UINT32 lowPriorityTimeout = 0;

    if(pSiteMgr->pDesiredParams->siteMgrDesiredBSSType == BSS_INDEPENDENT)
    {
        lowPriorityTimeout = SITE_MGR_IBSS_AGING_TIMEOUT_DEF;
    }
    else
    {
        /*
        If the pPrimarySite is NULL then we cannot update the aging timeout
        */
        if (pSiteMgr->pSitesMgmtParams->pPrimarySite == NULL)
        {
            WLAN_REPORT_ERROR(pSiteMgr->hReport,
                              SITE_MGR_MODULE_LOG,
                              ("there is NO primary site! pPrimarySite is NULL\n"));
            return;
        }
        else
        {
            /*
            in shortDoze and active mode the system behaviour is the same as regarding to
            beacon evnets. On each beacon the TNET is awake and should received the beacon
            */
            if(pSiteMgr->powerSaveLdMode == FALSE)
            {
                lowPriorityTimeout = pSiteMgr->pSitesMgmtParams->pPrimarySite->beaconInterval;
            }
            else
            /*
            In longDoze, the interval time should based on  numOfBeaconFiltering * Dtim interval
            */
            {
                /*
                if beacon filtering disable.
                */
                if ( 0 == pSiteMgr->numOfBeaconFiltering )
                {
                    lowPriorityTimeout = pSiteMgr->pSitesMgmtParams->pPrimarySite->dtimPeriod * pSiteMgr->pSitesMgmtParams->pPrimarySite->beaconInterval;
                }
                else
                {
                    lowPriorityTimeout = pSiteMgr->numOfBeaconFiltering * pSiteMgr->pSitesMgmtParams->pPrimarySite->dtimPeriod * pSiteMgr->pSitesMgmtParams->pPrimarySite->beaconInterval;
                }
            }

            WLAN_REPORT_INFORMATION(pSiteMgr->hReport,
                                    SITE_MGR_MODULE_LOG,
                                    ("siteMgr_calcLowPriorityTimeout: lowPriorityTimeout = %d, BeaconInterval = %d, DtimPeriod = %d,powerSaveLdMode = %d, numOfBeaconFiltering = %d\n",
                                    lowPriorityTimeout,
                                    pSiteMgr->pSitesMgmtParams->pPrimarySite->beaconInterval,
                                    pSiteMgr->pSitesMgmtParams->pPrimarySite->dtimPeriod,
                                    pSiteMgr->powerSaveLdMode,
                                    pSiteMgr->numOfBeaconFiltering));
        }
    }

    if (lowPriorityTimeout != 0)
    {
#ifdef NO_HAL_VOB
        whalCtrl_setLowPriorityTimeout(pSiteMgr->hHalCtrl,
                                       lowPriorityTimeout);
#endif
    }
    else
    {
        WLAN_REPORT_ERROR(pSiteMgr->hReport,
                          SITE_MGR_MODULE_LOG,
                          ("illegal lowPriorityTimeout (=%d), configuration ABORTED!\n",lowPriorityTimeout));
    }
}

#endif
