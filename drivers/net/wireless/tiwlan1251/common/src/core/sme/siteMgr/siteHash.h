/** \file siteHash.h
 *  \brief Hash & site table internal header file
 *
 *  \see siteHash.c
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
/*                                                                          */
/*    MODULE:   siteHash.h                                                  */
/*    PURPOSE:  Hash & site table internal header file                      */
/*                                                                          */
/***************************************************************************/
#ifndef __SITE_MGR_H__
#define __SITE_MGR_H__

#include "osTIType.h"
#include "paramOut.h"
#include "paramIn.h"
#include "802_11Defs.h"
#include "DataCtrl_Api.h"
#include "whalCtrl_api.h"

#define MAX_RSN_IE          3

/* site types */
typedef enum
{
    SITE_PRIMARY        = 0,
    SITE_SELF           = 1,
    SITE_REGULAR        = 2,
    SITE_NULL           = 3,
} siteType_e;

typedef struct
{
    UINT8   hopPattern;
    UINT8   hopSet;
    UINT16  dwellTime;
} FHParams_t;

/* A site entry contains all the site attribute received in beacon and probes
    and data used to manage the site table and hash table */
typedef struct siteEntry_t siteEntry_t;
struct siteEntry_t
{
    /* The following fields, in addition with the BSSID is used for entry management */
    UINT8                   index;
    siteType_e              siteType;
    UINT32                  localTimeStamp;
    UINT32                  dtimTimeStamp;
    UINT8                   tsfTimeStamp[TIME_STAMP_LEN];
    UINT32                  osTimeStamp;



    /* The following fields are used for the selection */
    BOOL                    probeRecv;
    BOOL                    beaconRecv;
    BOOL                    beaconReceiveAfterJoin;
    macAddress_t            bssid;
    ssid_t                  ssid;
    bssType_e               bssType;
    rateMask_t              rateMask;
    rate_e                  maxBasicRate;
    rate_e                  maxActiveRate;
    modulationType_e        beaconModulation;
    modulationType_e        probeModulation;
    preamble_e              currentPreambleType;
    preamble_e              preambleAssRspCap;
    preamble_e              barkerPreambleType;
    slotTime_e              currentSlotTime;
    slotTime_e              newSlotTime;
    BOOL                    useProtection;
    BOOL                    NonErpPresent;
    UINT8                   channel;
    UINT8                   attemptsNumber;
    UINT8                   Not_Received;
    UINT32                  matchingLevel;

    BOOL                    privacy;
    BOOL                    agility;
    UINT16                  capabilities;
    UINT16                  beaconInterval;
    UINT8                   dtimPeriod;
    UINT8                   snr;
    rate_e                  rxRate;
    INT32                   rssi;
    dot11_4X_t              fourXParams;
    BOOL                    fourXsupported;

    /* Power Constraint */
    UINT8                   powerConstraint;

    /* AP Tx Power obtained from TPC Report */
    UINT8                   APTxPower;

    /* QOS */
    BOOL                    WMESupported;
    ACParameters_t          WMEParameters;
    UINT8                   lastWMEParameterCnt;

    /* UPSD */
    BOOL                    APSDSupport;

    /* The following fields are never updated */
    UINT16                  atimWindow;
    FHParams_t              FHParams;
    dot11_RSN_t             pRsnIe[MAX_RSN_IE];
    UINT8                   rsnIeLen;

    mgmtStatus_e            failStatus;
    BOOL                    prioritySite;
    UINT8                   probeRespBuffer[MAX_BEACON_BODY_LENGTH];
    UINT16                  probeRespLength;
    UINT8                   beaconBuffer[MAX_BEACON_BODY_LENGTH];
    UINT16                  beaconLength;

    BOOL                    detectedWhileMeasuring;
};

typedef struct
{
    UINT8           numOfSites;
    UINT8           maxNumOfSites;
    siteEntry_t     siteTable[MAX_SITES_BG_BAND];
}siteTablesParams_t;

/* This struct is seperated from the above struct (siteTablesParams_t) for memory optimizations */
typedef struct
{
    UINT8           numOfSites;
    UINT8           maxNumOfSites;
    siteEntry_t     siteTable[MAX_SITES_A_BAND];
}siteTablesParamsBandA_t;

/* Ths following structure is used to manage the sites */
typedef struct
{
    siteTablesParamsBandA_t  dot11A_sitesTables;
    siteTablesParams_t        dot11BG_sitesTables;
    siteTablesParams_t        *pCurrentSiteTable;
    siteEntry_t               *pPrimarySite;
    siteEntry_t               *pPrevPrimarySite;
} sitesMgmtParams_t;


/* Site manager handle */
typedef struct
{
    siteMgrInitParams_t *pDesiredParams;
    sitesMgmtParams_t   *pSitesMgmtParams;

    TI_HANDLE           hConn;
    TI_HANDLE           hSmeSm;
    TI_HANDLE           hCtrlData;
    TI_HANDLE           hRxData;
    TI_HANDLE           hTxData;
    TI_HANDLE           hRsn;
    TI_HANDLE           hAuth;
    TI_HANDLE           hAssoc;
    TI_HANDLE           hRegulatoryDomain;
    TI_HANDLE           hMeasurementMgr;
    TI_HANDLE           hHalCtrl;
    TI_HANDLE           hMlmeSm;
    TI_HANDLE           hMemMgr;
    TI_HANDLE           hReport;
    TI_HANDLE           hOs;
    TI_HANDLE           hExcMngr;
    TI_HANDLE           hApConn;
    TI_HANDLE           hCurrBss;
    TI_HANDLE           hQosMngr;
    TI_HANDLE           hPowerMgr;
    TI_HANDLE           hEvHandler;
    TI_HANDLE           hMacServices;
    TI_HANDLE           hScr;

    UINT32              beaconSentCount;
    UINT32              rxPacketsCount;
    UINT32              txPacketsCount;

    modulationType_e    chosenModulation;
    modulationType_e    currentDataModulation;
    dot11mode_e         siteMgrOperationalMode;
    radioBand_e         radioBand;
    radioBand_e         prevRadioBand;
    
    macAddress_t        ibssBssid;
    BOOLEAN             bPostponedDisconnectInProgress;
    BOOL                isAgingEnable;

    /* TX Power Adjust */
    UINT32              siteMgrTxPowerCheckTime;
    BOOL                bTempTxPowerEnabled;

    /* Scans procedures */
    UINT8               numOfBeaconFiltering;
    BOOL                keepAliveEnable;
    /*UINT8             siteMgrDesiredBeaconFilterState;*/
    beaconFilterParams_t    beaconFilterParams; /*contains the desired state*/


    /*HW Request from Power Ctrl */
    UINT32              DriverTestId;

    BOOL                powerSaveLdMode;
} siteMgr_t;



siteEntry_t *findAndInsertSiteEntry(siteMgr_t       *pSiteMgr,
                                    macAddress_t    *bssid,
                                    radioBand_e     band);

siteEntry_t *findSiteEntry(siteMgr_t        *pSiteMgr,
                           macAddress_t     *bssid);

void removeSiteEntry(siteMgr_t *pSiteMgr, siteTablesParams_t *pCurrSiteTblParams,
                     siteEntry_t  *hashPtr);

TI_STATUS removeEldestSite(siteMgr_t *pSiteMgr);

TI_STATUS buildProbeReqTemplate(siteMgr_t *pSiteMgr, whalCtrl_setTemplate_t *pTemplate, ssid_t *pSsid, 
								radioBand_e radioBand);

TI_STATUS buildProbeRspTemplate(siteMgr_t *pSiteMgr, whalCtrl_setTemplate_t *pTemplate);

TI_STATUS buildNullTemplate(siteMgr_t *pSiteMgr, whalCtrl_setTemplate_t *pTemplate);

TI_STATUS buildPsPollTemplate(siteMgr_t *pSiteMgr, whalCtrl_setTemplate_t *pTemplate);

TI_STATUS buildQosNullDataTemplate(siteMgr_t *pSiteMgr, whalCtrl_setTemplate_t *pTemplate, UINT8 userPriority);

#endif /* __SITE_MGR_H__ */
