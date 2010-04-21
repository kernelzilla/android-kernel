/** \file currBss.c
 *  \brief Current BSS info
 *
 *  \see currBss.h
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
 *   MODULE:  Current BSS                                                   *
 *   PURPOSE:                                                               *
 *   Roaming ability of eSTA is implemented by Roaming Manager Component and 
 *   described in "Roaming Manager module LLD" document, and by 
 *   AP Connection module. AP Connection module implemented as two sub-modules.
 *   The major one is AP Connection, that is responsible for: 
 *   - providing Roaming Manager with access to other parts of WLAN Driver, 
 *   - implementing low levels of roaming mechanism.
 *   Current BSS sub-module takes care of:
 *   - maintaining database of current AP info,
 *   - providing access to database of current AP info.
 *   The Current BSS represents the BSS we are currently connected to. 
 *   Among other parameters, it holds the capabilities of the current AP, 
 *   its ID and its quality.
 *   When FW indicates 'Out of Sync' event, Current BSS module is responsible
 *   for awaking the device, sending unicast Probe request, waiting for
 *   response and - in case FW comes to the conclusion that there was 
 *   no response - for triggering "Beacon missed" to AP Connection module. 
 *   In eSTA5.0 FW updates and checks the quality of the connection with
 *   current AP. Current BSS module is responsible to handle event of type
 *   'Low RSSI' from FW. Third type of roaming event reported by FW is
 *   'Consecutive no ack on TX", and it is handled as well by Current
 *   BSS module.Upon reception of any roaming event from FW, Current BSS
 *   module is transferring this event to the AP Connection module in case
 *   of BSS connection, or to SME module in case of IBSS connection.
 *   When WLAN driver is working in IBSS mode, Current BSS module is holding
 *   the parameters of BSS (channel, band, SSID etc.).
 *                                                                          *
 ****************************************************************************/

#include "currBss.h"
#include "osApi.h"
#include "paramIn.h"
#include "report.h"
#include "802_11Defs.h"
#include "utils.h"
#include "memMngrEx.h"
#include "DataCtrl_Api.h"
#include "qosMngr_API.h"
#include "regulatoryDomainApi.h"
#include "apConn.h"
#include "scanMngrApi.h" 
#include "MacServices_api.h"
#include "smeApi.h"
#include "smeSmApi.h"
#include "TNETW_Driver_types.h"

/* Constants */

#define NUM_PACKETS_4_CONN_LIVENESS 2

/* Enumerations */


/* Typedefs */

typedef UINT8 (*currBSS_beaconRxCallb_t) (TI_HANDLE hModule, UINT64 staTSF, UINT8 dtimCount);


/* Structures */

/**
* Current BSS control block 
* Following structure defines parameters that can be configured externally,
* internal variables, and handlers of other modules used by Current BSS module
*/
typedef struct _currBSS_t
{
    /* Internal variables and configurable parameters */
    bssType_e   type;                   /**< Set by SME module; EBSS, IBSS or none */
    radioBand_e band;                   /**< Set by SME module */
    UINT8       channel;                /**< Set by AP Connection, SME and Switch Channel modules */
    UINT8       numOfPktsInRevivalTest; /**< Set by AP Connection; nmber of Probe Request Packets sent when 'Out of sync' suspected */
    BOOLEAN     fwIsOutOfSync;          /**< TRUE if 'Out of Sync' event received */
    BOOLEAN     isConnected;            /**< Default: not connected */
    bssEntry_t  currAPInfo;             /**< Set by SME upon request from AP Connection */
    UINT8       snr;                    /**< Value of SNR of last management packet received form the current AP */
    UINT8        snrFilterWeight;            /**< The weigh for average SNR value of management packets received form the current AP */
    BOOLEAN     rssiBelowThrReported;   /**< Set to TRUE whem low RSSI threshold crossed */
    BOOLEAN     rssiAboveThrReported;   /**< Set to TRUE whem high RSSI threshold crossed */
    INT8        averageRssi;            /**< Average value of RSSI of management packets received form the current AP */
    INT8        lowRssiThreshold;       /**< Indicator used to increase the background scan period when quality is low */
    INT8        highRssiThreshold;      /**< Indicator used to reduce the background scan period when quality is normal */
    UINT8        rssiFilterWeight;            /**< The weigh for average RSSI value of management packets received form the current AP */
    BOOLEAN     bUseSGParams;           /**< Whether to use the Soft Gemini compensation on the roaming triggers (currently: BSS Loss) */
                                        /**< This compensation is needed since BT Activity might over-run beacons                       */
    UINT8       numExpectedTbttForBSSLoss; /**< last configured value without Soft Gemini compensation                                 */
    UINT32      SGcompensationPercent;  /*< the percentage of increasing the TbttForBSSLoss value when SG is enabled */

    /* Handlers of other modules used by AP Connection */
    TI_HANDLE   hOs;
    TI_HANDLE   hPowerMngr;
    TI_HANDLE   hAPConn;
    TI_HANDLE   hSme;
    TI_HANDLE   hHal;
    TI_HANDLE   hMlme;
    TI_HANDLE   hReport;
    TI_HANDLE   hRegulatoryDomain;
    TI_HANDLE   hMemMgr;
    TI_HANDLE   hTxData;
    TI_HANDLE   hSiteMgr;
    TI_HANDLE   hScanMngr;
    TI_HANDLE   hMacServices;
} currBSS_t;

 
/* Internal functions prototypes */

static void currBSS_lowRssiThrCrossed(currBSS_t *hCurrBSS, UINT8 *data, UINT8 dataLength);
static void currBSS_lowSnrThrCrossed(currBSS_t *hCurrBSS, UINT8 *data, UINT8 dataLength);
static void currBSS_consecTxErrors(currBSS_t *hCurrBSS, UINT8 *data, UINT8 dataLength);
static void currBSS_outOfSync(currBSS_t *hCurrBSS, UINT8 *data, UINT8 dataLength);
static void currBSS_reviveConnection(currBSS_t *hCurrBSS);
static void currBSS_terminateOutOfSyncMode(currBSS_t *pCurrBSS);
static void currBSS_beaconMissed(currBSS_t *hCurrBSS, UINT8 *data, UINT8 dataLength);
static void currBSS_sendUnicastProbeRequest(currBSS_t *pCurrBSS);
static void currBSS_reportRoamingEvent(currBSS_t *hCurrBSS, apConn_roamingTrigger_e roamingEventType, roamingEventData_u *pRoamingEventData);
static void currBSS_updateBSSLoss(currBSS_t *pCurrBSS);

/* Public functions implementation */


/**
*
* currBSS_create
*
* \b Description: 
*
* Create the Current BSS context: allocate memory for internal variables
*
* \b ARGS:
*
*  I   - hOS - the handle to the OS object
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_HANDLE currBSS_create(TI_HANDLE hOs)
{
    currBSS_t   *pCurrBss;

    if ((pCurrBss = os_memoryAlloc(hOs, sizeof(currBSS_t))) != NULL)
    {
        pCurrBss->hOs = hOs;
    
        return pCurrBss;
    }
    else /* Failed to allocate control block */
    {
        WLAN_OS_REPORT(("FATAL ERROR: currBSS_create(): Error allocating cb - aborting\n"));
        return NULL;
    }
}


/**
*
* currBSS_unload
*
* \b Description: 
*
* Finish Current BSS module work.
*
* \b ARGS:
*
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS currBSS_unload(TI_HANDLE hCurrBSS)
{
    currBSS_t   *pCurrBSS;
    
    if (hCurrBSS != NULL)
    {
        pCurrBSS = (currBSS_t *)hCurrBSS;

        /* Free pre-allocated control block */
        utils_nullMemoryFree(pCurrBSS->hOs, pCurrBSS, sizeof(currBSS_t));
    }
    return OK;
}

/**
*
* currBSS_init
*
* \b Description: 
*
* Prepare Current BSS module to work
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  I   - hMlme  \n
*  I   - hScanMng  \n
*  I   - hPowerMgr  \n
*  I   - hAPConnection  \n
*  I   - hSME  \n
*  I   - hHal  \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS currBSS_init(TI_HANDLE hCurrBSS,
                       TI_HANDLE hMlme,
                       TI_HANDLE hPowerMgr,
                       TI_HANDLE hAPConnection,
                       TI_HANDLE hSME,
                       TI_HANDLE hHal,
                       TI_HANDLE hReport,
                       TI_HANDLE hMemMgr,
                       TI_HANDLE hTxData,
                       TI_HANDLE hSiteMngr,
                       TI_HANDLE hScanMngr,
                       TI_HANDLE hMacServices)
{
    currBSS_t   *pCurrBSS;
    whalCtrl_roamingTriggerCmd_t params;
    
    if (hCurrBSS != NULL)
    {
        pCurrBSS = (currBSS_t *)hCurrBSS;
        
        /* Registration succeeded, continue with init procedure */
        pCurrBSS->band = RADIO_BAND_2_4_GHZ;
        pCurrBSS->channel = 0;
        pCurrBSS->isConnected = FALSE;
        pCurrBSS->fwIsOutOfSync = FALSE;
        pCurrBSS->rssiAboveThrReported = FALSE;
        pCurrBSS->rssiBelowThrReported = FALSE;
        pCurrBSS->type = BSS_ANY;
        pCurrBSS->numOfPktsInRevivalTest = NUM_PACKETS_4_CONN_LIVENESS;
        pCurrBSS->snr = 0;
        pCurrBSS->snrFilterWeight = SNR_DEFAULT_WEIGHT;
        pCurrBSS->averageRssi = 0;
        pCurrBSS->rssiFilterWeight = RSSI_DEFAULT_WEIGHT;
        pCurrBSS->currAPInfo.RSSI = 0;
        pCurrBSS->highRssiThreshold = RSSI_DEFAULT_THRESHOLD;
        pCurrBSS->lowRssiThreshold = RSSI_DEFAULT_THRESHOLD;
        pCurrBSS->bUseSGParams = FALSE;
        
        pCurrBSS->hAPConn = hAPConnection;
        pCurrBSS->hTxData = hTxData;
        pCurrBSS->hHal = hHal;
        pCurrBSS->hMlme = hMlme;
        pCurrBSS->hPowerMngr = hPowerMgr;
        pCurrBSS->hSme = hSME;
        pCurrBSS->hMemMgr = hMemMgr;
        pCurrBSS->hSiteMgr = hSiteMngr;
        pCurrBSS->hReport = hReport;
        pCurrBSS->hScanMngr = hScanMngr;
        pCurrBSS->hMacServices = hMacServices;
        
        /* Configure and enable the Low RSSI, the Low SNR and the Missed beacon events */
        whalCtrl_EventMbox_RegisterForEvent(pCurrBSS->hHal, HAL_EVENT_RSSI_LEVEL, (void *)currBSS_lowRssiThrCrossed, pCurrBSS); 
        whalCtrl_EventMbox_Enable(pCurrBSS->hHal, HAL_EVENT_RSSI_LEVEL);

        whalCtrl_EventMbox_RegisterForEvent(pCurrBSS->hHal, HAL_EVENT_LOW_SNR, (void *)currBSS_lowSnrThrCrossed, pCurrBSS); 
        whalCtrl_EventMbox_Enable(pCurrBSS->hHal, HAL_EVENT_LOW_SNR);

        whalCtrl_EventMbox_RegisterForEvent(pCurrBSS->hHal, HAL_EVENT_BSS_REGAIN, (void *)currBSS_Bss_Regain_CallB , pCurrBSS); 
        whalCtrl_EventMbox_Enable(pCurrBSS->hHal, HAL_EVENT_BSS_REGAIN);

        params.rssiFilterDepth = RSSI_DEFAULT_DEPTH;
        params.rssiFilterWeight = RSSI_DEFAULT_WEIGHT;
        params.rssiThreshold =  RSSI_DEFAULT_THRESHOLD;
        params.lowRSSIEventType = LOW_RSSI_EVENT_LEVEL;
        whalCtrl_SetRSSIParamsCmd(pCurrBSS->hHal, &params);
        
        params.snrFilterDepth = SNR_DEFAULT_DEPTH;
        params.snrFilterWeight = SNR_DEFAULT_WEIGHT;
        params.snrThreshold =  SNR_DEFAULT_THRESHOLD;
        params.lowSNREventType = LOW_SNR_EVENT_LEVEL;
        whalCtrl_SetSNRParamsCmd(pCurrBSS->hHal, &params);
        
        /* Register for 'Out of Sync' and 'No Beacon trigger' events */
        whalCtrl_EventMbox_RegisterForEvent(pCurrBSS->hHal, HAL_EVENT_BSS_LOSE, (void *)currBSS_beaconMissed, pCurrBSS);
        whalCtrl_EventMbox_Enable(pCurrBSS->hHal, HAL_EVENT_BSS_LOSE);
        whalCtrl_EventMbox_RegisterForEvent(pCurrBSS->hHal, HAL_EVENT_SYNCHRONIZATION_TIMEOUT, (void *)currBSS_outOfSync, pCurrBSS);
        whalCtrl_EventMbox_Enable(pCurrBSS->hHal, HAL_EVENT_SYNCHRONIZATION_TIMEOUT);
            /* save last configured value for handling Soft Gemini changes */ 
        pCurrBSS->numExpectedTbttForBSSLoss = OUT_OF_SYNC_DEFAULT_THRESHOLD;
        params.TsfMissThreshold = OUT_OF_SYNC_DEFAULT_THRESHOLD;
        params.BssLossTimeout = NO_BEACON_DEFAULT_TIMEOUT;
        whalCtrl_SetBssLossTsfThresholdParamsCmd(pCurrBSS->hHal, &params);

        /* Register for BSS_RESET event' */        
        whalCtrl_EventMbox_RegisterForEvent(pCurrBSS->hHal, HAL_EVENT_BSS_RESET, (void *)currBSS_Bss_Reset_CallB , pCurrBSS);	
        /* Register for 'Consec. Tx error' */
        whalCtrl_EventMbox_RegisterForEvent(pCurrBSS->hHal, HAL_EVENT_MAX_TX_RETRY, (void *)currBSS_consecTxErrors, pCurrBSS);
        whalCtrl_EventMbox_Enable(pCurrBSS->hHal, HAL_EVENT_MAX_TX_RETRY);
        params.maxTxRetry = NO_ACK_DEFAULT_THRESHOLD;
        whalCtrl_SetMaxTxRetryParamsCmd(pCurrBSS->hHal, &params);
        
        return OK;
    }
    else /* Current BSS handle is NULL */
    {
        WLAN_OS_REPORT(("FATAL ERROR: Current BSS context is not initiated\n"));
        return NOK;
    }
}


/**
*
* currBSS_updateRoamingTriggers
*
* \b Description: 
*
* Configure parameter of Current BSS
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  I   - params - pointer to datablock of roaming threshols \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS currBSS_updateRoamingTriggers(TI_HANDLE hCurrBSS,
                                        roamingMngrThresholdsConfig_t *params)
{
    currBSS_t   *pCurrBSS = (currBSS_t *)hCurrBSS;
    whalCtrl_roamingTriggerCmd_t roamingTriggersParams;
    
    /* Configure HAL with RSSI parameters */
    roamingTriggersParams.rssiFilterDepth = RSSI_DEFAULT_DEPTH;
    roamingTriggersParams.rssiFilterWeight = params->rssiFilterWeight /* RSSI_DEFAULT_WEIGHT */ ;
    roamingTriggersParams.rssiThreshold = params->lowRssiThreshold;
    roamingTriggersParams.lowRSSIEventType = LOW_RSSI_EVENT_LEVEL;
    whalCtrl_SetRSSIParamsCmd(pCurrBSS->hHal, &roamingTriggersParams);
    
    /* Configure HAL with SNR parameters */
    roamingTriggersParams.snrFilterDepth = SNR_DEFAULT_DEPTH;
    roamingTriggersParams.snrFilterWeight = params->snrFilterWeight /* SNR_DEFAULT_WEIGHT */ ;
    roamingTriggersParams.snrThreshold = params->lowSnrThreshold;
    roamingTriggersParams.lowSNREventType = LOW_SNR_EVENT_LEVEL;
    whalCtrl_SetSNRParamsCmd(pCurrBSS->hHal, &roamingTriggersParams);
    
    /* save last configured value for handling Soft Gemini changes */ 
    pCurrBSS->numExpectedTbttForBSSLoss = params->numExpectedTbttForBSSLoss;
    /* Configure HAL with 'No BSS' thresholds (Same as the other parameters but in  a special
        function for the Soft Gemini module consideration) */
    currBSS_updateBSSLoss(pCurrBSS);
    
    /* Configure HAL with 'Consecutive NACK' thresholds */
    roamingTriggersParams.maxTxRetry = params->dataRetryThreshold;
    whalCtrl_SetMaxTxRetryParamsCmd(pCurrBSS->hHal, &roamingTriggersParams);
    
    pCurrBSS->lowRssiThreshold = params->lowQualityForBackgroungScanCondition;
    
    pCurrBSS->highRssiThreshold = params->normalQualityForBackgroungScanCondition;
    
    pCurrBSS->rssiFilterWeight = params->rssiFilterWeight;

    pCurrBSS->snrFilterWeight = params->snrFilterWeight;

    
    return OK;
}

/**
*
* currBSS_getRoamingParams
*
* \b Description: 
*
* Retrieves the roaming triggers stored in the CurrBSS module.
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  O   - aNumExpectedTbttForBSSLoss - Current BSS handle \n
*  O   - aLowQualityForBackgroungScanCondition - Current BSS handle \n
*  O   - aNormalQualityForBackgroungScanCondition - Current BSS handle \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS currBSS_getRoamingParams(TI_HANDLE hCurrBSS,
                                   UINT8 * aNumExpectedTbttForBSSLoss,
                                   INT8 * aLowQualityForBackgroungScanCondition,
                                   INT8 * aNormalQualityForBackgroungScanCondition,
                                   UINT8 * rssiFilterWeight,
                                   UINT8 * snrFilterWeight)
{
    currBSS_t * pCurrBSS = (currBSS_t *) hCurrBSS;

    *aNumExpectedTbttForBSSLoss = pCurrBSS->numExpectedTbttForBSSLoss;
    *aLowQualityForBackgroungScanCondition = pCurrBSS->lowRssiThreshold;
    *aNormalQualityForBackgroungScanCondition = pCurrBSS->highRssiThreshold;
    *rssiFilterWeight = pCurrBSS->rssiFilterWeight;
    *snrFilterWeight = pCurrBSS->snrFilterWeight;
    
    return OK;
}

/**
*
* currBSS_SGconfigureBSSLoss
*
* \b Description: 
*
*   This function is called by the Soft Gemini module in order to enable/disable the use of
*   the compensation value for the BSSLoss count , and the percent of increasing that value
*   It also set the new parameter to the FW (with another generic function)
*   The compensation is needed since BT activity might over-run recieved beacons
*    
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*        SGcompensationPercent - percent of increasing the BSSLoss value to the FW \n
*        bUseSGParams - whether to use the SG compensation
*
* \b RETURNS:
*
*  -
*
* \sa 
*/

void currBSS_SGconfigureBSSLoss(TI_HANDLE hCurrBSS,
                                        UINT32 SGcompensationPercent , BOOLEAN bUseSGParams)
{
    currBSS_t   *pCurrBSS = (currBSS_t *)hCurrBSS;
	 
    pCurrBSS->bUseSGParams = bUseSGParams;
    pCurrBSS->SGcompensationPercent = SGcompensationPercent;

	WLAN_REPORT_INFORMATION(pCurrBSS->hReport, ROAMING_MANAGER_MODULE_LOG,("CurrBSS_SGConf: SG =%d\n",
																		   pCurrBSS->bUseSGParams));

    /* update the change of BSSLoss in the FW */
    currBSS_updateBSSLoss(pCurrBSS);
}

/**
*
* currBSS_updateBSSLoss
*
* \b Description: 
*
*   This function updates only BSS Loss parameter , we need it to be able to consider the
*   Soft Gemini status , and change the parameter according to it 
*
* \b ARGS:
*
*  I   - pCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  -
*
* \sa 
*/
void currBSS_updateBSSLoss(currBSS_t   *pCurrBSS)
{
    whalCtrl_roamingTriggerCmd_t roamingTriggersParams;

    /* In Ad-Hoc we use default parameter */
    if (pCurrBSS->type == BSS_INDEPENDENT)
    {
       roamingTriggersParams.TsfMissThreshold = OUT_OF_SYNC_IBSS_THRESHOLD; 
    }
    else /* In Infra we use the saved parameter */
    {
        roamingTriggersParams.TsfMissThreshold = pCurrBSS->numExpectedTbttForBSSLoss;
    }
    
    roamingTriggersParams.BssLossTimeout = NO_BEACON_DEFAULT_TIMEOUT;

	WLAN_REPORT_INFORMATION(pCurrBSS->hReport, ROAMING_MANAGER_MODULE_LOG,("CurrBSS: SG=%d, Band=%d\n",
																		   pCurrBSS->bUseSGParams,
																		   pCurrBSS->currAPInfo.band));
    /* if Soft Gemini is enabled - increase the BSSLoss value (because BT activity might over-run beacons) */
    if ((pCurrBSS->bUseSGParams) && (pCurrBSS->currAPInfo.band == RADIO_BAND_2_4_GHZ))
    {
        roamingTriggersParams.TsfMissThreshold = (roamingTriggersParams.TsfMissThreshold * 
            (100 + pCurrBSS->SGcompensationPercent)) / 100;

        WLAN_REPORT_INFORMATION(pCurrBSS->hReport, ROAMING_MANAGER_MODULE_LOG,
            ("%s: old value = %d, new value (for SG compensation) = %d\n", __FUNCTION__, 
            pCurrBSS->numExpectedTbttForBSSLoss,roamingTriggersParams.TsfMissThreshold));   
    }
    whalCtrl_SetBssLossTsfThresholdParamsCmd(pCurrBSS->hHal, &roamingTriggersParams);
}

/**
*
* currBSS_swChFinished
*
* \b Description: 
*
* Called when switch channel process is complete in order to reset RSSI calculations
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  -
*
* \sa 
*/
void currBSS_restartRssiCounting(TI_HANDLE hCurrBSS)
{
    currBSS_t   *pCurrBSS = (currBSS_t *)hCurrBSS;

    pCurrBSS->averageRssi = 0;
    pCurrBSS->currAPInfo.RSSI = 0;
}

/**
*
* currBSS_getBssInfo
*
* \b Description: 
*
* Get parameter of Current BSS
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  pointer to current BSS info block.
*
* \sa 
*/
bssEntry_t *currBSS_getBssInfo(TI_HANDLE hCurrBSS)
{
    currBSS_t   *pCurrBSS = (currBSS_t *)hCurrBSS;

    /* Return pointer to current AP info */
    return &(pCurrBSS->currAPInfo);
}


/**
*
* currBSS_probRespReceivedCallb
*
* \b Description: 
*
* Callback function, provided to MLME module. Called each time Probe response received.
* This function verifies that the Probe response was sent by current AP, and then
* updates current AP database.
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS currBSS_probRespReceivedCallb(TI_HANDLE hCurrBSS,
                                        Rx_attr_t *pRxAttr,
                                        macAddress_t *bssid,
                                        mlmeFrameInfo_t *pFrameInfo,
                                        char *dataBuffer,
                                        UINT16 bufLength)
{
    currBSS_t   *pCurrBSS = (currBSS_t *)hCurrBSS;
    paramInfo_t param;
    
    param.paramType = SITE_MGR_CURRENT_BSSID_PARAM;
    siteMgr_getParam(pCurrBSS->hSiteMgr, &param);    

    if ((pCurrBSS->isConnected) && (MAC_EQUAL((&(param.content.siteMgrDesiredBSSID)), (bssid))))
    {
            /* If this is first probe response since FW reported 'Out of sync' situation, 
               then we were trying to revive connection, now stop sending probe requests */
        currBSS_terminateOutOfSyncMode(pCurrBSS);

        siteMgr_updateSite(pCurrBSS->hSiteMgr, bssid, pFrameInfo, pRxAttr->channel, (radioBand_e)pRxAttr->band, FALSE);
        /* Save the IE part of the Probe Response buffer in the site table */
        siteMgr_saveProbeRespBuffer(pCurrBSS->hSiteMgr, bssid, (UINT8 *)dataBuffer, bufLength);
    }
    return OK;
}



/**
*
* currBSS_beaconReceivedCallb
*
* \b Description: 
*
* Callback function, provided to MLME module. Called each time Beacon received.
* This function verifies that the Probe response was sent by current AP, and then
* updates current AP database.
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS currBSS_beaconReceivedCallb(TI_HANDLE hCurrBSS,
                                      Rx_attr_t *pRxAttr,
                                      macAddress_t *bssid,
                                      mlmeFrameInfo_t *pFrameInfo,
                                      char *dataBuffer,
                                      UINT16 bufLength)
{
    currBSS_t   *pCurrBSS = (currBSS_t *)hCurrBSS;
    paramInfo_t param;
    param.paramType = SITE_MGR_CURRENT_BSSID_PARAM;
    siteMgr_getParam(pCurrBSS->hSiteMgr, &param);    

    if ((pCurrBSS->isConnected) && (MAC_EQUAL((&(param.content.siteMgrDesiredBSSID)), (bssid))))
    {
        /* If this is first beacon since FW reported 'Out of sync' situation, 
           then we were trying to revive connection, now stop sending probe requests */
        currBSS_terminateOutOfSyncMode(pCurrBSS);

        siteMgr_updateSite(pCurrBSS->hSiteMgr, bssid, pFrameInfo, pRxAttr->channel, (radioBand_e)pRxAttr->band, FALSE);
        /* Save the IE part of the beacon buffer in the site table */
        siteMgr_saveBeaconBuffer(pCurrBSS->hSiteMgr, bssid, (UINT8 *)dataBuffer, bufLength);

        currBSS_updateRxSignal(hCurrBSS, pRxAttr->SNR, pRxAttr->Rssi, FALSE);
    }

    return OK;
}



/**
*
* currBSS_performRecovery
*
* \b Description: 
*
* This function is called when FW recovery performed.
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
TI_STATUS currBSS_performRecovery(TI_HANDLE hCurrBSS)
{
    currBSS_terminateOutOfSyncMode(hCurrBSS);
    return OK;
}


/**
*
* currBSS_updateConnectedState
*
* \b Description: 
*
* This function is called when FW recovery performed.
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  I   - isConnected - TRUE or FALSE \n
*  I   - type - IBSS or EBSS \n
*  
* \b RETURNS:
*
*  -
*
* \sa 
*/
void currBSS_updateConnectedState(TI_HANDLE hCurrBSS, BOOLEAN isConnected, bssType_e type)
{
    currBSS_t   *pCurrBSS = (currBSS_t *)hCurrBSS;
    paramInfo_t param;
    pCurrBSS->type = type;
    pCurrBSS->isConnected = isConnected;

    /* If new connection established or roaming started, as well as
       in case of disconnect, the event 'Out of sync' from FW is 
       no more relevant, do not proceed with it's handling */
    currBSS_terminateOutOfSyncMode(pCurrBSS);
    pCurrBSS->averageRssi = pCurrBSS->currAPInfo.RSSI;

    if (isConnected) 
    {
        /*** Store the info of current AP ***/

        /* BSSID */
        param.paramType = SITE_MGR_CURRENT_BSSID_PARAM;
        siteMgr_getParam(pCurrBSS->hSiteMgr, &param);    
        os_memoryCopy(pCurrBSS->hOs, (void *)pCurrBSS->currAPInfo.BSSID.addr, (void *)param.content.siteMgrDesiredBSSID.addr, MAC_ADDR_LEN);

        /* Rx rate */
        param.paramType = SITE_MGR_LAST_RX_RATE_PARAM;
        siteMgr_getParam(pCurrBSS->hSiteMgr, &param);               
        pCurrBSS->currAPInfo.rxRate = param.content.ctrlDataCurrentBasicRate;

        /* Band */
        param.paramType = SITE_MGR_RADIO_BAND_PARAM;
        siteMgr_getParam(pCurrBSS->hSiteMgr, &param);               
        pCurrBSS->currAPInfo.band = param.content.siteMgrRadioBand;

        /* Channel */
        param.paramType = SITE_MGR_CURRENT_CHANNEL_PARAM;
        siteMgr_getParam(pCurrBSS->hSiteMgr, &param);               
        pCurrBSS->currAPInfo.channel = param.content.siteMgrCurrentChannel;

        /* Last Rx Tsf */
        param.paramType = SITE_MGR_CURRENT_TSF_TIME_STAMP;
        siteMgr_getParam(pCurrBSS->hSiteMgr, &param);               
        os_memoryCopy(pCurrBSS->hOs, &pCurrBSS->currAPInfo.lastRxTSF, 
                      param.content.siteMgrCurrentTsfTimeStamp, sizeof(pCurrBSS->currAPInfo.lastRxTSF));

        /* Beacon interval */
        param.paramType = SITE_MGR_BEACON_INTERVAL_PARAM;
        siteMgr_getParam(pCurrBSS->hSiteMgr, &param);               
        pCurrBSS->currAPInfo.beaconInterval = param.content.beaconInterval;

        /* Capability */
        param.paramType = SITE_MGR_SITE_CAPABILITY_PARAM;
        siteMgr_getParam(pCurrBSS->hSiteMgr,&param);
        pCurrBSS->currAPInfo.capabilities = param.content.siteMgrSiteCapability;
        param.paramType = SITE_MGR_CURRENT_TSF_TIME_STAMP;
        siteMgr_getParam(pCurrBSS->hSiteMgr, &param);    

        /* pCurrBSS->currAPInfo.lastRxHostTimestamp = *((UINT64 *)(pIEs->TimeStamp));*/ /* TBD*/
        os_memoryCopy(pCurrBSS->hOs, &pCurrBSS->currAPInfo.lastRxHostTimestamp, param.content.siteMgrCurrentTsfTimeStamp, sizeof(UINT32));

        param.paramType = SITE_MGR_LAST_BEACON_BUF_PARAM;
        siteMgr_getParam(pCurrBSS->hSiteMgr, &param);               
        pCurrBSS->currAPInfo.pBuffer = param.content.siteMgrLastBeacon.buffer;
        pCurrBSS->currAPInfo.bufferLength = param.content.siteMgrLastBeacon.bufLength;
        pCurrBSS->currAPInfo.resultType = (param.content.siteMgrLastBeacon.isBeacon) ? SCAN_RFT_BEACON : SCAN_RFT_PROBE_RESPONSE;

        /* Set BSS Loss to Fw - note that it depends on the Connection type - (Infa/IBSS) */
        currBSS_updateBSSLoss(pCurrBSS);

        /* 
        this section is for the first beacon. in here we set the flag for waiting for 
        the first beacon and during all this time we must keep the Hw awake 
        this things are done only for INFRA connection.
        */
        if(type == BSS_INFRASTRUCTURE)
        {
                siteMgr_clearFirstBcnFlag(pCurrBSS->hSiteMgr);
        }        
    }
    else
    {
        /* 
        this section is for the first beacon. in here we cancel the setting of the flag 
        and the Hw available in case that the connection was closed.
        this things are done only for INFRA connection.
        */
        if(type == BSS_INFRASTRUCTURE)
            siteMgr_setFirstBcnFlag(pCurrBSS->hSiteMgr);
        
    }
}

/* Internal functions implementation */


/**
*
* currBSS_outOfSync
*
* \b Description: 
*
* Callback function, provided to HAL module.
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
static void currBSS_outOfSync(currBSS_t *hCurrBSS,
                              UINT8     *data,
                              UINT8     dataLength)
{
    currBSS_t   *pCurrBSS = (currBSS_t *)hCurrBSS;
    /*while syn with current BSS is lost, we need beacons to resynchronize*/
        
    /* If connected, and this kind of event was not received yet, handle it */ 
    if (pCurrBSS->isConnected && 
        pCurrBSS->fwIsOutOfSync == FALSE && 
        pCurrBSS->type != BSS_INDEPENDENT)
    {
        
        /* TBD IBSS configuration; no roaming enabled, report SME */
        
        pCurrBSS->fwIsOutOfSync = TRUE;

        /* force TNETW to active */
        MacServices_powerAutho_AwakeRequiredUpdate(pCurrBSS->hMacServices, POWERAUTHO_AWAKE_REQUIRED, POWERAUTHO_AWAKE_REASON_OUT_OS_SYNC);

        /* try to send Unicast Probe requests */
        currBSS_reviveConnection(pCurrBSS);

    }
    else
    {
    /*  If no connection, 
        or in the middle of roaming,
        or this kind of event was already received, - do nothing */ 
    }
}



/**
*
* currBSS_reviveConnection
*
* \b Description: 
*
* Called when FW indicates "Beacon missed" problem ('Out of sync' event).
* This function makes sure that FW is not asleep (the radio is awake), and
* sends Unicast Probe Request packet in order to provoke AP to answer with 
* Probe response.
* If HW interface is available (normal case), called immediately upon reception
* of the event from FW. Otherwise, this function is a function, provided to Power control.
* Called by Power control module in several cases:
* - when Power control exits pending and informs registered clients about HW availability; 
* - when FW recovery occurs - timeout waiting for FW to wakeup
*
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
static void currBSS_reviveConnection(currBSS_t *pCurrBSS)
{
    int pktsCount;
   
    /* TBD Turn ON LNA if the current state is OFF */

#if 0
    /* Perform MAC RX module reset. */
    /* For the moment we don't think the Rx Reset is nessasery */
    whalCtrl_resetMacRx(pCurrBSS->hHal);
#endif

    WLAN_REPORT_INFORMATION(pCurrBSS->hReport, ROAMING_MANAGER_MODULE_LOG,("currBSS_reviveConnection: sending %d Probe requests\n", pCurrBSS->numOfPktsInRevivalTest)); 

    /* Test connection with Unicast Probe request */
    for (pktsCount = pCurrBSS->numOfPktsInRevivalTest; pktsCount > 0; pktsCount--) 
    {
        currBSS_sendUnicastProbeRequest(pCurrBSS);
    }
}


/**
*
* currBSS_sendUnicastProbeRequest
*
* \b Description: 
*
* This function creaes and sends Unicast Probe request packet to current AP
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
static void currBSS_sendUnicastProbeRequest(currBSS_t *pCurrBSS)
{
    paramInfo_t             param;
    mem_MSDU_T              *pMsdu;
    probeReqTemplate_t      *pProbeReqTemplate;
    whalCtrl_setTemplate_t  templateStruct;
    TI_STATUS               status;
	radioBand_e				eRadioBand;
    /* 1. Allocate packet */ 
    status = wlan_memMngrAllocMSDU(pCurrBSS->hMemMgr, &pMsdu, sizeof(probeReqTemplate_t)+TX_TOTAL_OFFSET_BEFORE_DATA, CURRENT_BSS_MODULE);
    if (status != OK)
    {
        return;
    }

    /* 2. Build a probe request template */
    pProbeReqTemplate = (probeReqTemplate_t *)((char *)(memMgr_BufData(pMsdu->firstBDPtr))+ TX_TOTAL_OFFSET_BEFORE_DATA);
    templateStruct.pTemplate = (UINT8 *)pProbeReqTemplate;
    templateStruct.templateType = PROBE_REQUEST_TEMPLATE;

	param.paramType = SITE_MGR_RADIO_BAND_PARAM;
	siteMgr_getParam(pCurrBSS->hSiteMgr, &param);    
	eRadioBand = param.content.siteMgrRadioBand;

    param.paramType = SITE_MGR_CURRENT_SSID_PARAM;
    siteMgr_getParam(pCurrBSS->hSiteMgr, &param);    
    buildProbeReqTemplate(pCurrBSS->hSiteMgr, &templateStruct, &param.content.siteMgrCurrentSSID, eRadioBand);

    /* 3. Update BSSID to current BSSID */
    param.paramType = SITE_MGR_CURRENT_BSSID_PARAM;
    siteMgr_getParam(pCurrBSS->hSiteMgr, &param);    
    os_memoryCopy(pCurrBSS->hOs, &(pProbeReqTemplate->hdr.DA.addr), (void *)param.content.siteMgrDesiredBSSID.addr, MAC_ADDR_LEN);
    os_memoryCopy(pCurrBSS->hOs, &(pProbeReqTemplate->hdr.BSSID.addr), (void *)param.content.siteMgrDesiredBSSID.addr, MAC_ADDR_LEN);
    os_memoryCopy(pCurrBSS->hOs, &(pProbeReqTemplate->hdr.DA.addr), (void *)param.content.siteMgrDesiredBSSID.addr, MAC_ADDR_LEN);
    /* 4. Update MSDU parameters */
    pMsdu->headerLen = sizeof(dot11_mgmtHeader_t) + TX_TOTAL_OFFSET_BEFORE_DATA;
    pMsdu->dataLen = templateStruct.templateLen;
    memMgr_BufOffset(pMsdu->firstBDPtr) = TX_TOTAL_OFFSET_BEFORE_DATA;
    /*
     * Fix length according to TX_DESCRIPTOR_SIZE and bus txn reserved place
     */
    pMsdu->firstBDPtr->length = pMsdu->dataLen + TX_TOTAL_OFFSET_BEFORE_DATA;

    /* 5. Send the packet to the TX */
    pMsdu->txFlags |= TX_DATA_MGMT_MSDU;
    status = txData_txSendMsdu(pCurrBSS->hTxData, pMsdu);
}


/**
*
* currBSS_terminateOutOfSyncMode
*
* \b Description: 
*
* Beacon/probe response received, or connected state is changes - 
* if was in the middle of handling 'Out os sync' event, give it up
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
static void currBSS_terminateOutOfSyncMode(currBSS_t *pCurrBSS)
{
    if (pCurrBSS->fwIsOutOfSync) 
    {
        pCurrBSS->fwIsOutOfSync = FALSE;

        /* undo forcing TNETW to active */
        MacServices_powerAutho_AwakeRequiredUpdate(pCurrBSS->hMacServices, POWERAUTHO_AWAKE_NOT_REQUIRED, POWERAUTHO_AWAKE_REASON_OUT_OS_SYNC);     
    }
}

/**
*
* currBSS_beaconMissed
*
* \b Description: 
*
* Callback function, provided to HAL module.
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
static void currBSS_beaconMissed(currBSS_t *hCurrBSS,
                                      UINT8     *data,
                                      UINT8     dataLength)
{
    currBSS_terminateOutOfSyncMode(hCurrBSS);
    currBSS_reportRoamingEvent(hCurrBSS, ROAMING_TRIGGER_BSS_LOSS, NULL);
}


/**
*
* currBSS_consecTxErrors
*
* \b Description: 
*
* Callback function, provided to HAL module.
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
static void currBSS_consecTxErrors(currBSS_t *hCurrBSS,
                                   UINT8     *data,
                                   UINT8     dataLength)
{
    currBSS_reportRoamingEvent(hCurrBSS, ROAMING_TRIGGER_MAX_TX_RETRIES, NULL);
}


/**
*
* currBSS_lowRssiThrCrossed
*
* \b Description: 
*
* Callback function, provided to HAL module.
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
static void currBSS_lowRssiThrCrossed(currBSS_t *hCurrBSS,
                                      UINT8     *data,
                                      UINT8     dataLength)
{
    currBSS_reportRoamingEvent(hCurrBSS, ROAMING_TRIGGER_LOW_QUALITY, NULL);
}


/**
*
* currBSS_lowSnrThrCrossed
*
* \b Description: 
*
* Callback function, provided to HAL module.
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
static void currBSS_lowSnrThrCrossed(currBSS_t *hCurrBSS,
                                      UINT8     *data,
                                      UINT8     dataLength)
{
    currBSS_reportRoamingEvent(hCurrBSS, ROAMING_TRIGGER_LOW_SNR, NULL);
}

/**
*
* currBSS_reportRoamingEvent
*
* \b Description: 
*
* This function checks the mode of Current BSS module. 
* If connected to EBSS, it reports roaming event to AP Connection.
*
* \b ARGS:
*
*  I   - pCurrBSS - Current BSS handle \n
*  I   - roamingEventType - Roaming trigger to report \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/
static void currBSS_reportRoamingEvent(currBSS_t *pCurrBSS, 
                                       apConn_roamingTrigger_e roamingEventType,
									   roamingEventData_u *pRoamingEventData)
{
    WLAN_REPORT_INFORMATION(pCurrBSS->hReport, ROAMING_MANAGER_MODULE_LOG,("currBSS_reportRoamingEvent: trigger %d\n", roamingEventType));  

    if (pCurrBSS->isConnected)
    {
        if (pCurrBSS->type == BSS_INFRASTRUCTURE) 
        {
            apConn_reportRoamingEvent(pCurrBSS->hAPConn, roamingEventType, pRoamingEventData);
        }
        else /* IBSS */
        { 
            if( roamingEventType == ROAMING_TRIGGER_BSS_LOSS )
            {
                /* If in IBSS call the SME reselect function, this logic issues a DISCONNECT 
                 * event and tries to connect to other STA or establish self connection.
                 */
                smeSm_reselect(pCurrBSS->hSme);
            }
        }
    }
}

/***********************************************************************
 *                        currBSS_updateRxSignal
 ***********************************************************************
DESCRIPTION: Called when receiving beacon or Probe response from current AP
             updates SNR and RSSI in siteMgr and calls apConn in case 
             roaming trigger for BG scan occurred.

INPUT:      hCurrBSS    -   currBss handle.
            uSNR        
            iRSSI
            bAveragedData - indicate whether the given SNR,RSSI are already averaged.
                            This is done since this values can be given directly from the 
                            Fw, which average those values.
OUTPUT:

RETURN:     

************************************************************************/
void currBSS_updateRxSignal(TI_HANDLE hCurrBSS, UINT8 uSNR, INT8 iRSSI, BOOL bAveragedData)
{
    currBSS_t   *pCurrBSS = (currBSS_t *)hCurrBSS;
    INT8 tmpRssiAvg;
    INT8 rssiPrevVal;
    INT8 rssiLatestVal;
    paramInfo_t param;

    /* Update SNR */
    pCurrBSS->snr = uSNR;

    /* Update Rx rate of primary site is done in Site Manager */

    /* Update RSSI: */
    /* Count average RSSI */
    rssiPrevVal = pCurrBSS->averageRssi;
    rssiLatestVal = pCurrBSS->currAPInfo.RSSI = iRSSI;

    /* if the data is already averaged, or this is the first measured RSSI */
    if ((bAveragedData) || (rssiPrevVal == 0))  
    {
        pCurrBSS->averageRssi = rssiLatestVal;
    }
    else /* average the RSSI given */
    {
        tmpRssiAvg = ((rssiLatestVal*pCurrBSS->rssiFilterWeight) + (rssiPrevVal*(100-pCurrBSS->rssiFilterWeight)))/100;
    
        /* for faster convergence on RSSI changes use rounding error calculation with latest sample and not */
        /* on latest average */
        if (rssiLatestVal > tmpRssiAvg)
            tmpRssiAvg++;
        else
            if (rssiLatestVal < tmpRssiAvg)
                tmpRssiAvg--;

        pCurrBSS->averageRssi = tmpRssiAvg;
    }

    /* Report to AP Connection about reaching RSSI low or normal threshold */
    if ((pCurrBSS->rssiBelowThrReported == FALSE) && (pCurrBSS->averageRssi < pCurrBSS->lowRssiThreshold))
    {
        pCurrBSS->rssiAboveThrReported = FALSE;
        pCurrBSS->rssiBelowThrReported = TRUE;
        apConn_reportRoamingEvent(pCurrBSS->hAPConn, ROAMING_TRIGGER_LOW_QUALITY_FOR_BG_SCAN, NULL);
    }
    if ((pCurrBSS->rssiAboveThrReported == FALSE) && (pCurrBSS->averageRssi > pCurrBSS->highRssiThreshold))
    {
        pCurrBSS->rssiAboveThrReported = TRUE;
        pCurrBSS->rssiBelowThrReported = FALSE;
        apConn_reportRoamingEvent(pCurrBSS->hAPConn, ROAMING_TRIGGER_NORMAL_QUALITY_FOR_BG_SCAN, NULL); 
    }

    /* Update Site Table in order to represent the RSSI of current AP correctly in the utility */
    param.paramType = SITE_MGR_CURRENT_SIGNAL_PARAM;
    param.content.siteMgrCurrentSignal.rssi = pCurrBSS->averageRssi;
    siteMgr_setParam(pCurrBSS->hSiteMgr, &param);
}

/**
*
* currBSS_Bss_Regain_CallB
*
* \b Description: 
*   Used by site manager in case of BSS regain
* 
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/

TI_STATUS currBSS_Bss_Regain_CallB(TI_HANDLE hCurrBSS)

{
    currBSS_t *pCurrBSS = (currBSS_t *)hCurrBSS;
    paramInfo_t param;

    if ( NULL == hCurrBSS )
    {
        WLAN_REPORT_ERROR(pCurrBSS->hReport, CURR_BSS_MODULE_LOG,("Pointer to Curr BSS NULL, abort regain !")) ;
        return NOK;
    }
    
    param.paramType = SITE_MGR_CURRENT_BSSID_PARAM;
    siteMgr_getParam(pCurrBSS->hSiteMgr, &param);           

    if (pCurrBSS->isConnected) 
    {
        WLAN_REPORT_INFORMATION(pCurrBSS->hReport, CURR_BSS_MODULE_LOG,("Regain Current Connected BSS" ));  

        /* If we got the REGAIN_BSS Event from FW since it reported 'Out of sync' situation*/
        /* then we were trying to revive connection, now stop sending probe requests */
        currBSS_terminateOutOfSyncMode(pCurrBSS);

    }
    else
    {
        WLAN_REPORT_INFORMATION(pCurrBSS->hReport, CURR_BSS_MODULE_LOG,("Can't Regain BSS not connected !" ));    
    }
    return OK;

}

/**
*
* currBSS_Bss_Reset_CallB
*
* \b Description: 
*	Used by site manager in case of BSS Reset - if tsf was changed FW will signal with this event
* 
* \b ARGS:
*
*  I   - hCurrBSS - Current BSS handle \n
*  
* \b RETURNS:
*
*  OK on success, NOK on failure.
*
* \sa 
*/

TI_STATUS currBSS_Bss_Reset_CallB(TI_HANDLE hCurrBSS)

{
	currBSS_t *pCurrBSS = (currBSS_t *)hCurrBSS;
    
	if ( NULL == hCurrBSS )
	{
		WLAN_REPORT_ERROR(pCurrBSS->hReport, CURR_BSS_MODULE_LOG,("Pointer to Curr BSS NULL, abort Reset !")) ;
		return NOK;
	}
    
    
	if (pCurrBSS->isConnected) 
		{
			WLAN_REPORT_INFORMATION(pCurrBSS->hReport, CURR_BSS_MODULE_LOG,("Reset Current Connected BSS" ));	

           /* If we got the Reset_BSS Event from FW since TSF was changed may indicate BSS reset in the AP
           requier to initiate fist beacon mechanism and also test connectivity with the AP (NULL packet)*/
           siteMgr_clearFirstBcnFlag(pCurrBSS->hSiteMgr);

           

        }
	else
	{
		WLAN_REPORT_ERROR(pCurrBSS->hReport, CURR_BSS_MODULE_LOG,("Can't Reset BSS not connected !" ));	
	}
    return OK;

}

