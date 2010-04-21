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
/*                                                                         */
/*      MODULE:     Ctrl.c                                                 */
/*      PURPOSE:    Control module functions                               */
/*                                                                         */
/***************************************************************************/
#include "Ctrl.h"
#include "802_11Defs.h"
#include "DataCtrl_Api.h"
#include "osApi.h"
#include "report.h" 
#include "utils.h"
#include "smeApi.h"
#include "siteMgrApi.h"
#include "Ethernet.h"
#include "tx.h"
#include "TrafficMonitorAPI.h"
#include "TI_IPC_Api.h"
#include "EvHandler.h"
#include "apConn.h"
#include "TNETW_Driver_api.h"
#include "Core_AdaptTx.h"
#include "whalCtrl_api.h"

static void selectRateTable(TI_HANDLE hCtrlData, UINT32 rateMask);
static void ctrlData_setTxRatePolicies(ctrlData_t *pCtrlData);
static void ctrlData_resetCounters(TI_HANDLE hCtrlData);

static void ctrlData_resetCounters(TI_HANDLE hCtrlData);

static void ctrlData_UnregisterTrafficIntensityEvents (TI_HANDLE hCtrlData);
static void ctrlData_RegisterTrafficIntensityEvents (TI_HANDLE hCtrlData);

static UINT32 ctrlData_buildHwBitMapFromArray(policyClassRatesArray_t *pArray);
static void ctrlData_storeTSRateSet(ctrlData_t *pCtrlData, txDataQosParams_t *tsrsParams);
static rate_e ctrlData_getClosestTSRate(ctrlData_t *pCtrlData, UINT32 ac, rate_e givenRate);

void ctrlData_TrafficThresholdCrossed(TI_HANDLE Context,UINT32 Cookie);


/* definitions for medium usage calculations - in uSec units*/
#define AVERAGE_ACK_TIME   10
#define AVERAGE_CTS_TIME   20
#define B_SIFS             10

#define SHORT_PREAMBLE_TIME   96
#define LONG_PREAMBLE_TIME    192

#define OFDM_PREAMBLE      12
#define OFDM_SIGNAL_EXT    6
#define OFDM_PLCP_HDR      24

#define OFDM_DURATION           (B_SIFS + OFDM_PLCP_HDR + OFDM_SIGNAL_EXT)
#define NONOFDM_SHORT_DURATION  (B_SIFS + SHORT_PREAMBLE_TIME)
#define NONOFDM_LONG_DURATION   (B_SIFS + LONG_PREAMBLE_TIME)

/*************************************************************************
*                        ctrlData_create                                 *
**************************************************************************
* DESCRIPTION:  This function initializes the Ctrl data module.                 
*                                                      
* INPUT:        hOs - handle to Os Abstraction Layer
*               
* OUTPUT:       TxCmplt_CB - call back function that return to configMngr
*               in order to register in the Hal
*
* RETURN:       Handle to the allocated Ctrl data control block
************************************************************************/
TI_HANDLE ctrlData_create(TI_HANDLE hOs)
{
    ctrlData_t*         hCtrlData;
    rateAdaptation_t*   pRateAdaptation;
#ifdef SUPPORT_4X
    fourX_t*            pFourX;
#endif
    classifier_t*       pClsfr;

    if( hOs  == NULL )
    {
        WLAN_OS_REPORT(("FATAL ERROR: ctrlData_create(): OS handle Error - Aborting\n"));
        return NULL;
    }

    /* alocate Control module control block */
    hCtrlData = os_memoryAlloc(hOs, (sizeof(ctrlData_t)));
    if(!hCtrlData)
        return NULL;

    /* create rate adaptation module */
    pRateAdaptation = rateAdaptation_create(hOs);
#ifdef SUPPORT_4X
    /* create 4X module */
    pFourX = fourX_create(hOs);
#endif
    /* create the classifier module */
    pClsfr = Classifier_create(hOs);
    
    if ( (!pRateAdaptation) 
#ifdef SUPPORT_4X
         || (!pFourX)
#endif
       )
    {
        utils_nullMemoryFree(hOs, pRateAdaptation, sizeof(rateAdaptation_t));
#ifdef SUPPORT_4X
        utils_nullMemoryFree(hOs, pFourX, sizeof(fourX_t));
#endif
        utils_nullMemoryFree(hOs, hCtrlData, sizeof(ctrlData_t));

        WLAN_OS_REPORT(("FATAL ERROR: ctrlData_create(): Error Creating Ctrl Module - Aborting\n"));
        return(NULL);
    }

    /* reset control module control block */
    os_memoryZero(hOs, hCtrlData, (sizeof(ctrlData_t)));

    hCtrlData->pRateAdaptation = pRateAdaptation;
#ifdef SUPPORT_4X
    hCtrlData->pFourX = pFourX;
#endif
    hCtrlData->pClsfr = pClsfr;

    hCtrlData->hOs = hOs;

    return(hCtrlData);
}

/***************************************************************************
*                           ctrlData_config                                *
****************************************************************************
* DESCRIPTION:  This function configures the Ctrl Data module       
* 
* INPUTS:       hCtrlData - The object
*               hWhalCtrl - Handle to the Whal Ctrl object
*               hSiteMgrHandle - Handle to the Site Mngr object
*               hTxData - Handle to the Tx Data object
*               hRxData - Handle to the Rx Data object
*               hOs - Handle to the Os Abstraction Layer
*               hReport - Handle to the Report object
*               ctrlDataInitParams - pointer to Ctrl module init parameters
* OUTPUT:       
* 
* RETURNS:      OK - Configuration succesfull
*               NOK - Configuration unsuccesfull
***************************************************************************/
TI_STATUS ctrlData_config(TI_HANDLE         hCtrlData,
                       TI_HANDLE            hWhalCtrl, 
                       TI_HANDLE            hSiteMgrHandle, 
                       TI_HANDLE            hTxData, 
                       TI_HANDLE            hRxData, 
                       TI_HANDLE            hOs, 
                       TI_HANDLE            hReport, 
                       TI_HANDLE            hMemMngr, 
                       TI_HANDLE            hEvHandler,
                       TI_HANDLE            hAPConnection,
                       TI_HANDLE            hTrafficMonitor,                       
                       disassocSentCB_t     disassocSentCBFunc,
                       TI_HANDLE            disassocSentCBObj,  
                       ctrlDataInitParams_t *ctrlDataInitParams)
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;
    TI_STATUS Status = OK;
    rateClassClients_e clientIDindex;
    txRateClassId_e  TxRateIndex;
    UINT32 ac;

    /* check parameters validity */
    if( hCtrlData == NULL || hWhalCtrl == NULL || hSiteMgrHandle == NULL || hRxData == NULL ||
        hTxData == NULL || hOs == NULL || hReport == NULL || hMemMngr == NULL || ctrlDataInitParams == NULL)
    {
        WLAN_OS_REPORT(("FATAL ERROR: ctrlData_config(): Parameters Error - Aborting\n"));
        return NOK;
    }

    /* set objects handles */
    pCtrlData->hWhalCtrl =  hWhalCtrl;  
    pCtrlData->hSiteMgr =   hSiteMgrHandle;
    pCtrlData->hTxData =    hTxData;
    pCtrlData->hRxData =    hRxData;
    pCtrlData->hOs =        hOs;
    pCtrlData->hReport =    hReport;
    pCtrlData->hAPConn = hAPConnection;
    pCtrlData->hEvHandler = hEvHandler;
    pCtrlData->hTrafficMonitor  = hTrafficMonitor;

    pCtrlData->disassocSentCBObj = disassocSentCBObj;
    pCtrlData->disassocSentCBFunc = disassocSentCBFunc;


    /*  set Control module parameters */
    pCtrlData->ctrlDataRateControlEnable = ctrlDataInitParams->ctrlDataRateControlEnable;
    pCtrlData->ctrlDataIbssProtectionType = ctrlDataInitParams->ctrlDataDesiredIbssProtection;
    pCtrlData->ctrlDataRtsCtsStatus = ctrlDataInitParams->ctrlDataDesiredCtsRtsStatus;

#ifdef SUPPORT_4X
    pCtrlData->ctrlDataFourXEnable = ctrlDataInitParams->ctrlDataFourXEnable;
    pCtrlData->ctrlDataCerruentFourXstate = ctrlDataInitParams->ctrlDataFourXEnable;
#else
    pCtrlData->ctrlDataFourXEnable = FALSE;
    pCtrlData->ctrlDataCerruentFourXstate = FALSE;
#endif
    
    MAC_COPY(pCtrlData->hOs, (&pCtrlData->ctrlDataDeviceMacAddress), 
            (&ctrlDataInitParams->ctrlDataDeviceMacAddress));

    pCtrlData->ctrlDataStartStoplinkControlAlg = DEF_START_STOP_LINK_CTRL_ALG;
    pCtrlData->ctrlDataCurrentBasicRate = DEF_BASIC_RATE;
    pCtrlData->ctrlDataBasicRateBitMask = DEF_BASIC_RATE_MASK;
    pCtrlData->ctrlDataCurrentBasicModulationType = DRV_MODULATION_QPSK;
    pCtrlData->ctrlDataCurrentModulationType = DEF_CURRENT_MUDULATION_TYPE;
    pCtrlData->ctrlDataCurrentBssType = BSS_INFRASTRUCTURE;
    os_memoryCopy(pCtrlData->hOs, &pCtrlData->ctrlDataRateTables,
                                  &ctrlDataInitParams->rateTable,
                                  sizeof(rateTables_t) );

    for (clientIDindex = (rateClassClients_e)0 ; clientIDindex < NUM_OF_RATE_CLASS_CLIENTS ; clientIDindex++) 
    {
        pCtrlData->bIsClassAvailable[clientIDindex] = TRUE;
        /* by default all clients use all available rates */ 
        pCtrlData->currClientRateMask[clientIDindex] = ALL_RATES_AVAILABLE;
        pCtrlData->nextClientRateMask[clientIDindex] = ALL_RATES_AVAILABLE;

        /* Init Params are initialized for USER & SG policies only */
        /* Set short/long retry for all ACs */
        for (ac=0; ac < MAX_NUM_OF_AC; ac++) 
        {
            pCtrlData->ctrlDataTxRatePolicy.rateClass[clientIDindex*MAX_NUM_OF_AC+ac].longRetryLimit  = ctrlDataInitParams->ctrlDataTxRatePolicy[clientIDindex].longRetryLimit;
            pCtrlData->ctrlDataTxRatePolicy.rateClass[clientIDindex*MAX_NUM_OF_AC+ac].shortRetryLimit = ctrlDataInitParams->ctrlDataTxRatePolicy[clientIDindex].shortRetryLimit;
        }

        for (TxRateIndex = txPolicy54; TxRateIndex < MAX_NUM_OF_TX_RATES_IN_CLASS; TxRateIndex++)
        {
            pCtrlData->policyClassRatesArrayCck[clientIDindex].txRate[TxRateIndex] = ctrlDataInitParams->policyClassRatesArrayCck[clientIDindex].txRate[TxRateIndex];
            pCtrlData->policyClassRatesArrayPbcc[clientIDindex].txRate[TxRateIndex] = ctrlDataInitParams->policyClassRatesArrayPbcc[clientIDindex].txRate[TxRateIndex]; 
            pCtrlData->policyClassRatesArrayOfdm[clientIDindex].txRate[TxRateIndex] = ctrlDataInitParams->policyClassRatesArrayOfdm[clientIDindex].txRate[TxRateIndex]; 
            pCtrlData->policyClassRatesArrayOfdmA[clientIDindex].txRate[TxRateIndex] = ctrlDataInitParams->policyClassRatesArrayOfdmA[clientIDindex].txRate[TxRateIndex]; 
        }
    }
    /* By default use USER_RATE_CLASS */
    pCtrlData->currClientRateID = USER_RATE_CLASS;
    pCtrlData->configuredClientRateID = USER_RATE_CLASS;

    pCtrlData->ctrlDataCurrentRateTable = &pCtrlData->ctrlDataRateTables.ctrlDataCckRateTable;

    /* reset Counters */
    ctrlData_resetCounters(pCtrlData);
    
    /* Configure Rate Adaptation Module */
    rateAdaptation_config(pCtrlData->pRateAdaptation, hOs, hReport, pCtrlData, hEvHandler,
                          hAPConnection, &ctrlDataInitParams->rateAdaptationInitParam); 

#ifdef SUPPORT_4X
    /* configure fourX Module */
    fourX_config(pCtrlData->pFourX, hOs, hReport, hMemMngr, hWhalCtrl, hTxData,
                            &ctrlDataInitParams->fourXInitParams);
#endif

    /* configure the classifier Module */
    Status = Classifier_config(pCtrlData->pClsfr, hOs, hReport, &ctrlDataInitParams->ClsfrInitParam);
    if (Status != OK)
    {
        WLAN_OS_REPORT(("FATAL ERROR: ctrlData_config(): Classifier_config() failed - Aborting\n"));
        return Status;
    }

    /* Initialize traffic intensity threshold parameters */
    pCtrlData->ctrlDataTrafficIntensityEventsEnabled = ctrlDataInitParams->ctrlDataTrafficThresholdEnabled;
    pCtrlData->ctrlDataTrafficIntensityThresholds.uHighThreshold = ctrlDataInitParams->ctrlDataTrafficThreshold.uHighThreshold;
    pCtrlData->ctrlDataTrafficIntensityThresholds.uLowThreshold = ctrlDataInitParams->ctrlDataTrafficThreshold.uLowThreshold;
    pCtrlData->ctrlDataTrafficIntensityThresholds.TestInterval = ctrlDataInitParams->ctrlDataTrafficThreshold.TestInterval;

    WLAN_REPORT_INFORMATION(pCtrlData->hReport, CTRL_DATA_MODULE_LOG,
       ("\nTraffic Intensity parameters:\nEvents enabled = %d\nuHighThreshold = %d\nuLowThreshold = %d\nTestInterval = %d\n\n",
       pCtrlData->ctrlDataTrafficIntensityEventsEnabled,
       pCtrlData->ctrlDataTrafficIntensityThresholds.uHighThreshold,
       pCtrlData->ctrlDataTrafficIntensityThresholds.uLowThreshold,
       pCtrlData->ctrlDataTrafficIntensityThresholds.TestInterval));

    /* Register the traffic intensity events with the traffic monitor */
    ctrlData_RegisterTrafficIntensityEvents (pCtrlData);

    /* If the events are enabled, start notification, if disabled - then do nothing */
    ctrlData_ToggleTrafficIntensityNotification (pCtrlData, pCtrlData->ctrlDataTrafficIntensityEventsEnabled);

    WLAN_REPORT_INIT(pCtrlData->hReport, CTRL_DATA_MODULE_LOG, 
        (".....Ctrl Data configured successfully ...\n"));

    return OK;
}

/***************************************************************************
*                           ctrlData_unLoad                                *
****************************************************************************
* DESCRIPTION:  This function unload the Ctrl data module. 
* 
* INPUTS:       hCtrlData - the object
*       
* OUTPUT:       
* 
* RETURNS:      OK - Unload succesfull
*               NOK - Unload unsuccesfull
***************************************************************************/

TI_STATUS ctrlData_unLoad(TI_HANDLE hCtrlData)  
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;

    /* check parameters validity */
    if( pCtrlData == NULL )
    {
        WLAN_REPORT_ERROR(pCtrlData->hReport, CTRL_DATA_MODULE_LOG,
            (" ctrlData_unLoad() : parametrs value error \n"));
        return NOK;
    }


    rateAdaptation_destroy(pCtrlData->pRateAdaptation);
#ifdef SUPPORT_4X
    fourX_destroy(pCtrlData->pFourX);
#endif
    Classifier_destroy(pCtrlData->pClsfr);

    /* free timer */
    /* free control module controll block */
    os_memoryFree(pCtrlData->hOs, hCtrlData, sizeof(ctrlData_t));

    return OK;
}
/***************************************************************************
*                           ctrlData_getParam                              *
****************************************************************************
* DESCRIPTION:  get a specific parameter
* 
* INPUTS:       hCtrlData - the object
*               
*       
* OUTPUT:       pParamInfo - structure which include the value of 
*               the requested parameter
* 
* RETURNS:      OK
*               NOK
***************************************************************************/

TI_STATUS ctrlData_getParam(TI_HANDLE hCtrlData, paramInfo_t *pParamInfo)   
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;
    
    /* WLAN_REPORT_INFORMATION(pCtrlData->hReport, CTRL_DATA_MODULE_LOG, 
        ("ctrlData_getParam() : param=0x%x \n", pParamInfo->paramType)); */

    switch (pParamInfo->paramType)
    {
    case CTRL_DATA_RATE_CONTROL_ENABLE_PARAM: 
        pParamInfo->content.ctrlDataRateControlEnable = pCtrlData->ctrlDataRateControlEnable;
        break; 
        
    case CTRL_DATA_FOUR_X_ENABLE_PARAM: 
        pParamInfo->content.ctrlDataFourXEnable = pCtrlData->ctrlDataFourXEnable;
        break; 

    case CTRL_DATA_FOUR_X_CURRRENT_STATUS_PARAM:
        if (pCtrlData->ctrlDataCurrentBssType == BSS_INFRASTRUCTURE)
        {
            pParamInfo->content.ctrlDataCerruentFourXstate = pCtrlData->ctrlDataCerruentFourXstate;
        } else {
            pParamInfo->content.ctrlDataCerruentFourXstate = pCtrlData->ctrlDataFourXEnable;
        }

        break; 

    case CTRL_DATA_CURRENT_BSSID_PARAM:          
        MAC_COPY(pCtrlData->hOs, (&pParamInfo->content.ctrlDataCurrentBSSID), 
                (&pCtrlData->ctrlDataCurrentBSSID));
        break; 

    case CTRL_DATA_CURRENT_BSS_TYPE_PARAM:  
        pParamInfo->content.ctrlDataCurrentBssType = pCtrlData->ctrlDataCurrentBssType;        
        break; 

    case CTRL_DATA_CURRENT_MODULATION_TYPE_PARAM: 
        pParamInfo->content.ctrlDataCurrentModulationType = pCtrlData->ctrlDataCurrentModulationType;      
        break; 

    case CTRL_DATA_CURRENT_PREAMBLE_TYPE_PARAM: 
        pParamInfo->content.ctrlDataCurrentPreambleType = pCtrlData->ctrlDataCurrentPreambleType;        
        break; 

    case CTRL_DATA_MAC_ADDRESS:    
        MAC_COPY(pCtrlData->hOs, (&pParamInfo->content.ctrlDataDeviceMacAddress), (&pCtrlData->ctrlDataDeviceMacAddress)); 
        break;

    case CTRL_DATA_CURRENT_BASIC_RATE_PARAM:             
        pParamInfo->content.ctrlDataCurrentBasicRate = pCtrlData->ctrlDataCurrentBasicRate;               
        break;

    case CTRL_DATA_CURRENT_BASIC_RATE_MASK_PARAM: 
        pParamInfo->content.ctrlDataBasicRateBitMask = pCtrlData->ctrlDataBasicRateBitMask;      
        break; 

    case CTRL_DATA_CURRENT_BASIC_MODULATION_PARAM: 
        pParamInfo->content.ctrlDataCurrentBasicModulationType = pCtrlData->ctrlDataCurrentBasicModulationType;      
        break; 

    case CTRL_DATA_COUNTERS_PARAM:
        os_memoryCopy(pCtrlData->hOs,&pParamInfo->content.ctrlDataCounters,
            &pCtrlData->ctrlDataCounters, sizeof(ctrlDataCounters_t));
        break; 

    case CTRL_DATA_CURRENT_SUPPORTED_RATE_MASK_PARAM: 
        pParamInfo->content.ctrlDataCurrentRateMask = pCtrlData->ctrlDataCurrentRateMask;
        break; 

    case CTRL_DATA_CURRENT_PROTECTION_STATUS_PARAM:
        pParamInfo->content.ctrlDataProtectionEnabled = pCtrlData->ctrlDataProtectionEnabled;
        break;

    case CTRL_DATA_CURRENT_IBSS_PROTECTION_PARAM:
        pParamInfo->content.ctrlDataIbssProtecionType = pCtrlData->ctrlDataIbssProtectionType;
        break;

    case CTRL_DATA_CURRENT_RTS_CTS_STATUS_PARAM:
        pParamInfo->content.ctrlDataRtsCtsStatus = pCtrlData->ctrlDataRtsCtsStatus;
        break;

    case CTRL_DATA_CLSFR_TYPE:
        Classifier_getClsfrType (pCtrlData->pClsfr,&pParamInfo->content.ctrlDataClsfrType);
        break;
        /*
         * NOTE: currently supporting only USER_RATE_CLASS!!!!!!!!!
         */
    case CTRL_DATA_SHORT_RETRY_LIMIT_PARAM:
        pParamInfo->content.TxRatePolicy.rateClass[USER_RATE_CLASS].shortRetryLimit = pCtrlData->ctrlDataTxRatePolicy.rateClass[USER_RATE_CLASS].shortRetryLimit;
        break;

        /*
         * NOTE: currently supporting only ONE txRatePolicy!!!!!!!!!
         */
    case CTRL_DATA_LONG_RETRY_LIMIT_PARAM:
        pParamInfo->content.TxRatePolicy.rateClass[USER_RATE_CLASS].longRetryLimit = pCtrlData->ctrlDataTxRatePolicy.rateClass[USER_RATE_CLASS].longRetryLimit;
        break;


    case CTRL_DATA_TRAFFIC_INTENSITY_THRESHOLD:
        pParamInfo->content.ctrlDataTrafficIntensityThresholds.uHighThreshold = pCtrlData->ctrlDataTrafficIntensityThresholds.uHighThreshold;
        pParamInfo->content.ctrlDataTrafficIntensityThresholds.uLowThreshold = pCtrlData->ctrlDataTrafficIntensityThresholds.uLowThreshold;
        pParamInfo->content.ctrlDataTrafficIntensityThresholds.TestInterval = pCtrlData->ctrlDataTrafficIntensityThresholds.TestInterval;
        break;

    default:
        return (PARAM_NOT_SUPPORTED);
/*        WLAN_REPORT_ERROR(pCtrlData->hReport, TX_DATA_MODULE_LOG, 
            (" ctrlData_getParam() : PARAMETER NOT SUPPORTED \n"));
        return NOK;
        break; - unreachable */
    }

    return (OK);
}

/***************************************************************************
*                           ctrlData_buildSupportedHwRates                 *
****************************************************************************
* DESCRIPTION:  builds HwRatesBitMap (supported rates) for txRatePolicy using 4 elements :
*               1) AP support
*               2) Driver support
*               3) Client support (using currClientRateMask[clientIDindex] )
*               4) Policy rates   (retries per client)
* 
* OUTPUT:       
* 
* RETURNS:      OK
*               NOK
***************************************************************************/
static UINT32 ctrlData_buildSupportedHwRates(ctrlData_rateAdapt_t *pDriverTable,
                                             UINT32             APsupport,
                                             UINT32             clientSupport,
                                             UINT32             policySupport)
{
    UINT16  AppRateBitMap = 0;
    UINT32  HwRatesBitMap, DriverTableBitMap = 0;
    UINT32 i = 0;

    /* 1. Convert Rates table into bit mask */
    while (i <= pDriverTable->len)
    {
        DriverTableBitMap |= (1 << (pDriverTable->rateAdaptRatesTable[i++] - 1));
    }

    /* 2. AND with other masks */
    AppRateBitMap = (UINT16)(DriverTableBitMap & APsupport & clientSupport & policySupport);

    /* In case there are no mutual rates, try to ignore policy settings */
    if (AppRateBitMap == 0)
    {
        AppRateBitMap = (UINT16)(DriverTableBitMap & APsupport);
    }

    /* 3. Set total supported rates bit map for txRatePolicy */
    ConvertAppRatesToBitmap(AppRateBitMap, &HwRatesBitMap);

    return HwRatesBitMap;
}


/***************************************************************************
*                           ctrlData_setTxRatePolicies                     *
****************************************************************************
* DESCRIPTION:  This function sets rate fallback policies to be configured to FW
*               If TSRS is defined to specific AC, the policy is derived from it,
*               otherwise it is derived from pre-defined map
* 
* INPUTS:       pCtrlData - the object
*       
* RETURNS:      -
*               
***************************************************************************/
static void ctrlData_setTxRatePolicies(ctrlData_t *pCtrlData)
{
    UINT32 ac, policyClassRateMask, supportedHwRatesBitMap, clientIDindex, hwRateIndex;
    UINT32 fwPolicyID;
    UINT8 shortRetryLimit, longRetryLimit;
    txRateClassId_e  txRateIndex;
    whalParamInfo_t param;

    pCtrlData->currClientRateID = pCtrlData->configuredClientRateID;

    for (clientIDindex = 0, fwPolicyID = 0; clientIDindex < NUM_OF_RATE_CLASS_CLIENTS; clientIDindex++) 
    {
        /* Retrieve retry limits stored in first AC of current class */ 
        shortRetryLimit = pCtrlData->ctrlDataTxRatePolicy.rateClass[fwPolicyID].shortRetryLimit;
        longRetryLimit = pCtrlData->ctrlDataTxRatePolicy.rateClass[fwPolicyID].longRetryLimit;

        /* By default assume that this class can be enabled */
        pCtrlData->bIsClassAvailable[clientIDindex] = TRUE;

        /* Update the rate mask from nextClientRateMask, where it was stored untill now */
        pCtrlData->currClientRateMask[clientIDindex] = pCtrlData->nextClientRateMask[clientIDindex];

        for (ac = 0; ac < MAX_NUM_OF_AC; ac++)
        {
            /* 1. Check if there is special rate set defined for this access category, */
            /*    then verify that at least one rate is mutual between TSRS and class policy, */
            /*    otherwise use default settings for this class */
            if (pCtrlData->tsrsParameters[ac].supportedRatesMask[clientIDindex] != 0)
            {
                /* Check if at least one rate is mutual between TSRS and current class policy, */
                /* otherwise use default for this class */
                policyClassRateMask = pCtrlData->tsrsParameters[ac].policyClassRateMask[clientIDindex];
            }
            else
            {
                policyClassRateMask = ctrlData_buildHwBitMapFromArray(&(pCtrlData->pCurrPolicyClassRatesArray[clientIDindex]));
            }

            /* 2. Build a bitMap for the supported rates */
            supportedHwRatesBitMap = ctrlData_buildSupportedHwRates(
                                pCtrlData->ctrlDataCurrentRateTable,            /* according to radio mode */
                                pCtrlData->ctrlDataCurrentRateMask,             /* AP supported rates */
                                pCtrlData->currClientRateMask[clientIDindex],   /* STA supported rates */
                                policyClassRateMask);                           /* requested by class policy rates */

            if (supportedHwRatesBitMap == 0)
            {
                WLAN_REPORT_ERROR(pCtrlData->hReport,CTRL_DATA_MODULE_LOG,
                                  ("%s No supported rates for client %d, ac %d \n", __FUNCTION__, clientIDindex, ac));
                pCtrlData->bIsClassAvailable[clientIDindex] = FALSE;
                pCtrlData->currClientRateID = USER_RATE_CLASS;
            }

            /* 3. Configure retransmission for the rates */
            for (hwRateIndex = HW_BIT_RATE_54MBPS, txRateIndex = txPolicy54; 
                 txRateIndex < MAX_NUM_OF_TX_RATES_IN_CLASS; 
                 hwRateIndex >>= 1, txRateIndex++)
            {
                /* if supported rate */
                if (supportedHwRatesBitMap & hwRateIndex)
                {
                    /* if rate fall back is enabled */
                    if (pCtrlData->ctrlDataRateControlEnable)
                    {
                        /* Set retries as they were configured in ini file for this class; */
                        /* make sure at least one retransmission is defined, */
                        /* to take care of cases in which we ignored pre-defined policy */ 
                        pCtrlData->ctrlDataTxRatePolicy.rateClass[fwPolicyID].txRate[txRateIndex] = 
                            (pCtrlData->pCurrPolicyClassRatesArray[clientIDindex].txRate[txRateIndex] > 1) ? 
                            pCtrlData->pCurrPolicyClassRatesArray[clientIDindex].txRate[txRateIndex] : 1;
                    }
                    else /* no rate fallback */
                    {
                        /* set max reties because no fall back is implemented */
                        pCtrlData->ctrlDataTxRatePolicy.rateClass[fwPolicyID].txRate[txRateIndex] = 
                            (shortRetryLimit > longRetryLimit) ? shortRetryLimit : longRetryLimit;
                        pCtrlData->ctrlDataTxRatePolicy.rateClass[fwPolicyID].flags |= TX_POLICY_FLAGS_TRUNCATE;
                    }
                }
                else
                {
                    pCtrlData->ctrlDataTxRatePolicy.rateClass[fwPolicyID].txRate[txRateIndex] = 0;
                }

                WLAN_REPORT_INFORMATION(pCtrlData->hReport,CTRL_DATA_MODULE_LOG,
                    ("%s: AC %d, class %d, rate 0x%x[%d]\n", __FUNCTION__, ac, clientIDindex, hwRateIndex, pCtrlData->ctrlDataTxRatePolicy.rateClass[fwPolicyID].txRate[txRateIndex]));
            }

            /* Note that Long/Short retries are pre-set during configuration stage */

            /* 4. Finally, increase total number of policies */
            pCtrlData->tsrsParameters[ac].fwPolicyID[clientIDindex] = fwPolicyID++;
        }
    }

    /* Download policies to the FW. Num of policies is 8 - one for each AC for every class */
    WLAN_REPORT_INFORMATION(pCtrlData->hReport, CTRL_DATA_MODULE_LOG,
        ("%s: num of Rate policies: %d\n", __FUNCTION__, fwPolicyID));

    pCtrlData->ctrlDataTxRatePolicy.numOfRateClasses = fwPolicyID;
    param.paramType = HAL_CTRL_TX_RATE_CLASS_PARAMS;
    param.content.pTxRatePlicy = &pCtrlData->ctrlDataTxRatePolicy;

    whalCtrl_SetParam(pCtrlData->hWhalCtrl, &param);
}


/***************************************************************************
*                           ctrlData_setParam                              *
****************************************************************************
* DESCRIPTION:  set a specific parameter
* 
* INPUTS:       hCtrlData - the object
*               pParamInfo - structure which include the value to set for 
*               the requested parameter
*       
* OUTPUT:       
* 
* RETURNS:      OK
*               NOK
***************************************************************************/

TI_STATUS ctrlData_setParam(TI_HANDLE hCtrlData, paramInfo_t *pParamInfo)   
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;
    whalParamInfo_t param;
    rateClassClients_e clientID;

    /* WLAN_REPORT_INFORMATION(pCtrlData->hReport, CTRL_DATA_MODULE_LOG, 
        ("ctrlData_setParam() : param=0x%x \n", pParamInfo->paramType)); */

    switch (pParamInfo->paramType)
    {
    case CTRL_DATA_RATE_CONTROL_ENABLE_PARAM:
        pCtrlData->ctrlDataRateControlEnable = pParamInfo->content.ctrlDataRateControlEnable;
        selectRateTable(pCtrlData, pCtrlData->ctrlDataCurrentRateMask);

        ctrlData_setTxRatePolicies(pCtrlData);

        rateAdaptation_buildRateMapTable(pCtrlData->pRateAdaptation,
                                                 pCtrlData->ctrlDataCurrentRateTable,
                                                 pCtrlData->ctrlDataCurrentRateMask,
                                                 pCtrlData->currClientRateMask[pCtrlData->currClientRateID],
                                                 pCtrlData->ctrlDataCurrentModulationType,
                                                 pCtrlData->ctrlDataCerruentFourXstate,
                                          pCtrlData->ctrlDataCurrentBssType);
    


    

             if(pCtrlData->ctrlDataRateControlEnable == TRUE )
             {
                 /* start rate adaptation algorithm */

                if( pCtrlData->ctrlDataStartStoplinkControlAlg == TRUE)
                {   

                rateAdaptation_start(pCtrlData->pRateAdaptation);
            }
            else 
            {    
                /* stop rate adaptation algorithm */
                rateAdaptation_stop(pCtrlData->pRateAdaptation);
            }
        }
        else 
        {
            rateAdaptation_stopTimer(pCtrlData->pRateAdaptation);
        }
        break; 
        
    case CTRL_DATA_FOUR_X_ENABLE_PARAM: 
#ifdef SUPPORT_4X
        pCtrlData->ctrlDataFourXEnable = pParamInfo->content.ctrlDataFourXEnable;
        if(pCtrlData->ctrlDataStartStoplinkControlAlg == TRUE)
        {
            if(pCtrlData->ctrlDataFourXEnable == TRUE)
            {   
                pCtrlData->ctrlDataCerruentFourXstate = TRUE;
            }
            else 
            {   
                pCtrlData->ctrlDataCerruentFourXstate = FALSE;
            }
        }
        rateAdaptation_update4xEnable(pCtrlData->pRateAdaptation,
                                      pCtrlData->ctrlDataCerruentFourXstate,
                                      pCtrlData->ctrlDataCurrentBssType );

#else 
        pCtrlData->ctrlDataFourXEnable = FALSE;
        pCtrlData->ctrlDataCerruentFourXstate = FALSE;
#endif
        break; 

    case CTRL_DATA_CURRENT_RATE_CLASS_CLIENT:
        /* set a new rate class client to be used on data packets */
        clientID = pParamInfo->content.ctrlDataRateClassID;
        /* always save the wanted configuration , even when not enabled */
        pCtrlData->configuredClientRateID = clientID;
        
        if (clientID == pCtrlData->currClientRateID)
        {       
            /* Do nothing - already configured */
            break;
        }
        else 
        {
            if (TRUE == pCtrlData->bIsClassAvailable[clientID])
            {       
                /* use the new clientID + reset rateAdaptation tables */
                pCtrlData->currClientRateID = clientID;
                selectRateTable(pCtrlData, pCtrlData->ctrlDataCurrentRateMask);
                rateAdaptation_buildRateMapTable(pCtrlData->pRateAdaptation,
                                                     pCtrlData->ctrlDataCurrentRateTable,
                                                     pCtrlData->ctrlDataCurrentRateMask,
                                                     pCtrlData->currClientRateMask[pCtrlData->currClientRateID],
                                                     pCtrlData->ctrlDataCurrentModulationType,
                                                     pCtrlData->ctrlDataCerruentFourXstate,
                                                     pCtrlData->ctrlDataCurrentBssType);
                break;

            }
            else
            {   
                /* The class could not be configured due to no rate support - don't use it */
                WLAN_REPORT_ERROR(pCtrlData->hReport,CTRL_DATA_MODULE_LOG,
                    ("%s: Can't enable rate class ID %d\n",__FUNCTION__,clientID));
                break;
            } 
        }

    case CTRL_DATA_NEXT_RATE_MASK_FOR_CLIENT:
        /* configure the next rate mask to be used for a specific client on the next connection */
        /* NOTE : changing USER_RATE_CLASS configuration is not advisable */
        pCtrlData->nextClientRateMask[pParamInfo->content.ctrlDataRateClassMask.clientID] = 
            pParamInfo->content.ctrlDataRateClassMask.clientRateMask;
        break;
        
    case CTRL_DATA_FOUR_X_CURRRENT_STATUS_PARAM: 
#ifdef SUPPORT_4X
        if((pCtrlData->ctrlDataFourXEnable == TRUE) &&
           (pCtrlData->ctrlDataCurrentBssType == BSS_INFRASTRUCTURE))
            pCtrlData->ctrlDataCerruentFourXstate = pParamInfo->content.ctrlDataCerruentFourXstate;
        else
            pCtrlData->ctrlDataCerruentFourXstate = FALSE;
#else 
        pCtrlData->ctrlDataCerruentFourXstate = FALSE;
#endif
        break; 

    case CTRL_DATA_CURRENT_BSSID_PARAM:          
        MAC_COPY(pCtrlData->hOs, (&pCtrlData->ctrlDataCurrentBSSID),
            (&pParamInfo->content.ctrlDataCurrentBSSID));
        break; 

    case CTRL_DATA_CURRENT_BSS_TYPE_PARAM:
        if( pParamInfo->content.ctrlDataCurrentBssType != BSS_INFRASTRUCTURE &&
            pParamInfo->content.ctrlDataCurrentBssType != BSS_INDEPENDENT )
            return(PARAM_VALUE_NOT_VALID);

        pCtrlData->ctrlDataCurrentBssType = pParamInfo->content.ctrlDataCurrentBssType;        
        break; 

    case CTRL_DATA_CURRENT_MODULATION_TYPE_PARAM: 
        pCtrlData->ctrlDataCurrentModulationType = pParamInfo->content.ctrlDataCurrentModulationType;   
        /* update rate modulatin table for Rate adaptation algorithm */

        rateAdaptation_updateModulation(pCtrlData->pRateAdaptation,
                                        pCtrlData->ctrlDataCurrentModulationType,
                                        pCtrlData->ctrlDataCurrentBssType);

        break; 

    case CTRL_DATA_CURRENT_PREAMBLE_TYPE_PARAM: 
        if( pParamInfo->content.ctrlDataCurrentPreambleType != PREAMBLE_LONG &&
            pParamInfo->content.ctrlDataCurrentPreambleType != PREAMBLE_SHORT )
            return(PARAM_VALUE_NOT_VALID);
 
        pCtrlData->ctrlDataCurrentPreambleType = pParamInfo->content.ctrlDataCurrentPreambleType;        

        break; 

    case CTRL_DATA_MAC_ADDRESS: 
        {
            int status;
			status = whalCtrl_SetMacAddress(pCtrlData->hWhalCtrl, &pParamInfo->content.ctrlDataDeviceMacAddress);
			WLAN_REPORT_ERROR(pCtrlData->hReport, TX_DATA_MODULE_LOG,  
							  (" ctrlData_setParam() : MAC ADDRESS SET STATUS: %d \n",status));
			if(status == OK)
				MAC_COPY(pCtrlData->hOs, (&pCtrlData->ctrlDataDeviceMacAddress),
						 (&pParamInfo->content.ctrlDataDeviceMacAddress)); 

        }
        break;

    case CTRL_DATA_CURRENT_BASIC_RATE_PARAM:             

        pCtrlData->ctrlDataCurrentBasicRate = pParamInfo->content.ctrlDataCurrentBasicRate;  
        /* for Basic Rate Set use the USER_RATE_CLASS (otherwise we could get 0 rates) */
        pCtrlData->ctrlDataBasicRateBitMask = rateAdaptation_Utils_buildRateBitMap(pCtrlData->pRateAdaptation,
                                                                                   pCtrlData->ctrlDataCurrentRateTable,
                                                                                   pCtrlData->ctrlDataCurrentBasicRate,         
                                                                                   pCtrlData->ctrlDataCurrentRateMask,
                                                                                   pCtrlData->currClientRateMask[USER_RATE_CLASS]);
        break;
 
    case CTRL_DATA_CURRENT_BASIC_RATE_MASK_PARAM: 
        pCtrlData->ctrlDataBasicRateBitMask = pParamInfo->content.ctrlDataBasicRateBitMask;
        break; 

    case CTRL_DATA_CURRENT_BASIC_MODULATION_PARAM: 
        pCtrlData->ctrlDataCurrentBasicModulationType = pParamInfo->content.ctrlDataCurrentBasicModulationType;
        break; 


    case CTRL_DATA_CURRENT_SUPPORTED_RATE_MASK_PARAM:
        pCtrlData->ctrlDataCurrentRateMask = pParamInfo->content.ctrlDataCurrentRateMask;
        selectRateTable(pCtrlData, pCtrlData->ctrlDataCurrentRateMask);
        rateAdaptation_buildRateMapTable(pCtrlData->pRateAdaptation,
                                         pCtrlData->ctrlDataCurrentRateTable,
                                         pCtrlData->ctrlDataCurrentRateMask,
                                         pCtrlData->currClientRateMask[pCtrlData->currClientRateID],
                                         (modulationType_e)pCtrlData->ctrlDataCerruentFourXstate,
                                         pCtrlData->ctrlDataCurrentModulationType,
                                         pCtrlData->ctrlDataCurrentBssType);


        break; 

    case CTRL_DATA_CURRENT_ACTIVE_RATE_PARAM:
        rateAdaptation_setCurrentRate(pCtrlData->pRateAdaptation, pParamInfo->content.ctrlDataCurrentActiveRate);

        break;

    case CTRL_DATA_TSRS_PARAM:
        ctrlData_storeTSRateSet(pCtrlData, &pParamInfo->content.txDataQosParams);

        break;

    case CTRL_DATA_CURRENT_PROTECTION_STATUS_PARAM:
        if (pCtrlData->ctrlDataProtectionEnabled != pParamInfo->content.ctrlDataProtectionEnabled)
        {
            pCtrlData->ctrlDataProtectionEnabled = pParamInfo->content.ctrlDataProtectionEnabled;

            /* set indication also to TNET */
            param.paramType = HAL_CTRL_CTS_TO_SELF_PARAM;
            if(pCtrlData->ctrlDataProtectionEnabled == TRUE)
                param.content.halCtrlCtsToSelf = CTS_TO_SELF_ENABLE;
            else
                param.content.halCtrlCtsToSelf = CTS_TO_SELF_DISABLE;

            whalCtrl_SetParam(pCtrlData->hWhalCtrl,&param);


            /* In case of using protection fragmentation should be disabled */
            param.paramType = HAL_CTRL_FRAG_THRESHOLD_PARAM;
            if(pCtrlData->ctrlDataProtectionEnabled == TRUE)
            {
                /* save last non-protection mode fragmentation threshold */
                whalCtrl_GetParam(pCtrlData->hWhalCtrl,&param);
                pCtrlData->lastFragmentThreshold = param.content.halCtrlFragThreshold;
                /* set fragmentation threshold to max (disable) */
                param.content.halCtrlFragThreshold = HAL_CTRL_FRAG_THRESHOLD_MAX;
            }
            else
                param.content.halCtrlFragThreshold = pCtrlData->lastFragmentThreshold;
            
            whalCtrl_SetParam(pCtrlData->hWhalCtrl,&param);
        }

        break;

    case CTRL_DATA_CURRENT_IBSS_PROTECTION_PARAM:
        pCtrlData->ctrlDataIbssProtectionType = pParamInfo->content.ctrlDataIbssProtecionType;

        /* set indication also to TNET */
        param.paramType = HAL_CTRL_CTS_TO_SELF_PARAM;
        if(pCtrlData->ctrlDataIbssProtectionType != ERP_PROTECTION_NONE)
            param.content.halCtrlCtsToSelf = CTS_TO_SELF_ENABLE;
        else
            param.content.halCtrlCtsToSelf = CTS_TO_SELF_DISABLE;
                
        whalCtrl_SetParam(pCtrlData->hWhalCtrl,&param);

        break;

    case CTRL_DATA_CURRENT_RTS_CTS_STATUS_PARAM:
        pCtrlData->ctrlDataRtsCtsStatus = pParamInfo->content.ctrlDataRtsCtsStatus;
        break;
    case CTRL_DATA_CLSFR_TYPE:
        ctrlData_clsfrSetClsfrType (pCtrlData->pClsfr,pParamInfo->content.ctrlDataClsfrType);
        break;

    case CTRL_DATA_CLSFR_CONFIG:
            Classifier_InsertClsfrEntry(pCtrlData->pClsfr, 1, &pParamInfo->content.ctrlDataClsfrInsertTable);
        break;

    case CTRL_DATA_CLSFR_REMOVE_ENTRY:
         classifier_RemoveClsfrEntry(pCtrlData->pClsfr, &pParamInfo->content.ctrlDataClsfrInsertTable);
       break;
       
    case CTRL_DATA_GET_USER_PRIORITY_OF_STREAM:
       Classifier_deriveUserPriorityFromStream (pCtrlData->pClsfr,&pParamInfo->content.ctrlDataUpOfStream);
       break;
            /*
         * NOTE: currently supporting only ONE txRatePolicy!!!!!!!!!
         */
    case CTRL_DATA_SHORT_RETRY_LIMIT_PARAM:
        pCtrlData->ctrlDataTxRatePolicy.rateClass[USER_RATE_CLASS].shortRetryLimit = pParamInfo->content.TxRatePolicy.rateClass[USER_RATE_CLASS].shortRetryLimit;
        break;

        /*
         * NOTE: currently supporting only ONE txRatePolicy!!!!!!!!!
         */
    case CTRL_DATA_LONG_RETRY_LIMIT_PARAM:
        pCtrlData->ctrlDataTxRatePolicy.rateClass[USER_RATE_CLASS].longRetryLimit = pParamInfo->content.TxRatePolicy.rateClass[USER_RATE_CLASS].longRetryLimit;
        break;

    case CTRL_DATA_TOGGLE_TRAFFIC_INTENSITY_EVENTS:

            /* Enable or disable events according to flag */
            ctrlData_ToggleTrafficIntensityNotification (pCtrlData, (BOOL)pParamInfo->content.ctrlDataTrafficIntensityEventsFlag);
        
        break;

    case CTRL_DATA_TRAFFIC_INTENSITY_THRESHOLD:
        {
            OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS *localParams = &pParamInfo->content.ctrlDataTrafficIntensityThresholds;
            BOOL savedEnableFlag;   /* Used to save previous enable/disable flag - before stopping/starting events for change in params */
        
            /* If any of the parameters has changed, we need to re-register with the Traffic Monitor */
            if ((localParams->uHighThreshold != pCtrlData->ctrlDataTrafficIntensityThresholds.uHighThreshold) ||
                (localParams->uLowThreshold != pCtrlData->ctrlDataTrafficIntensityThresholds.uLowThreshold) ||
                (localParams->TestInterval != pCtrlData->ctrlDataTrafficIntensityThresholds.TestInterval))
            {

                os_memoryCopy(pCtrlData->hOs, &pCtrlData->ctrlDataTrafficIntensityThresholds,
                                localParams,
                                sizeof(OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS));

                savedEnableFlag = pCtrlData->ctrlDataTrafficIntensityEventsEnabled;

                /* Turn off traffic events */
                ctrlData_ToggleTrafficIntensityNotification (pCtrlData, FALSE);

                /* Unregister current events */
                ctrlData_UnregisterTrafficIntensityEvents (pCtrlData);
                
                /* And re-register with new thresholds */
                ctrlData_RegisterTrafficIntensityEvents (pCtrlData);

                /* Enable events if they were enabled  */
                ctrlData_ToggleTrafficIntensityNotification (pCtrlData, savedEnableFlag);

            }
        }
        
        break;

    default:
        WLAN_REPORT_ERROR(pCtrlData->hReport, TX_DATA_MODULE_LOG,  
            (" ctrlData_setParam() : PARAMETER NOT SUPPORTED \n"));
        return (PARAM_NOT_SUPPORTED);
/*        break; - unrechable */
    }

    return (OK);
}

/***************************************************************************
*                           ctrlData_getTspecsRateThresholds                                   *
****************************************************************************
* DESCRIPTION:  The function retrieves the current low/high phy rate thresholds.
*
* INPUTS:       hCtrlData - the object
*               uAC       - The AC number.
*               
* OUTPUT:       pHighThreshold - The current phy rate high threshold
*               pHighThreshold - The current phy rate low threshold
*
* RETURNS:      
***************************************************************************/
void ctrlData_getTspecsRateThresholds(TI_HANDLE hCtrlData, UINT8 uAC, UINT32* pHighThreshold, UINT32* pLowThreshold)
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;
    rateAdaptation_t* pRateAdaptation = (rateAdaptation_t*)pCtrlData->pRateAdaptation;
    
    *pHighThreshold = hostToUtilityRate(RateNumberToHost(pRateAdaptation->tspecsRateParameters[uAC].highRateThreshold));
    *pLowThreshold = hostToUtilityRate(RateNumberToHost(pRateAdaptation->tspecsRateParameters[uAC].lowRateThreshold));
}

/***************************************************************************
*                           selectRateTable                                *
****************************************************************************
* DESCRIPTION:  
*
* INPUTS:       hCtrlData - the object
*               
* OUTPUT:       
*
* RETURNS:      
***************************************************************************/

static void selectRateTable(TI_HANDLE hCtrlData, UINT32 rateMask)
{
    paramInfo_t param;
    rate_e      rate;
    
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;

    rate = getMaxRatefromBitmap(rateMask);

    param.paramType = SITE_MGR_OPERATIONAL_MODE_PARAM;
    siteMgr_getParam(pCtrlData->hSiteMgr, &param);

    switch(param.content.siteMgrDot11OperationalMode)
    {
        case DOT11_B_MODE:
        {
            if(rate == DRV_RATE_22M)
            {
                pCtrlData->ctrlDataCurrentRateTable = &pCtrlData->ctrlDataRateTables.ctrlDataPbccRateTable;
                pCtrlData->pCurrPolicyClassRatesArray = pCtrlData->policyClassRatesArrayPbcc;
            }
            else
            {
                pCtrlData->ctrlDataCurrentRateTable = &pCtrlData->ctrlDataRateTables.ctrlDataCckRateTable;
                pCtrlData->pCurrPolicyClassRatesArray = pCtrlData->policyClassRatesArrayCck;

            }
        }
        break;

        case DOT11_G_MODE:
            if( (rate == DRV_RATE_22M) ||
                (rate == DRV_RATE_11M) || 
                (rate == DRV_RATE_5_5M)|| 
                (rate == DRV_RATE_2M)  || 
                (rate == DRV_RATE_1M)    )
            {
                pCtrlData->ctrlDataCurrentRateTable = &pCtrlData->ctrlDataRateTables.ctrlDataPbccRateTable;
                pCtrlData->pCurrPolicyClassRatesArray = pCtrlData->policyClassRatesArrayPbcc;
            }
            else
            {
                pCtrlData->ctrlDataCurrentRateTable = &pCtrlData->ctrlDataRateTables.ctrlDataOfdmRateTable ;
                pCtrlData->pCurrPolicyClassRatesArray = pCtrlData->policyClassRatesArrayOfdm;
            }
        break;

        case DOT11_A_MODE:
            pCtrlData->ctrlDataCurrentRateTable = &pCtrlData->ctrlDataRateTables.ctrlDataOfdmARateTable;
            pCtrlData->pCurrPolicyClassRatesArray = pCtrlData->policyClassRatesArrayOfdmA;

        break;
        case DOT11_DUAL_MODE:
        case DOT11_MAX_MODE:
            WLAN_REPORT_ERROR(pCtrlData->hReport,CTRL_DATA_MODULE_LOG,
                ("%s ctrlDataCurrentRateTable not configured !!!",__FUNCTION__));
            break;

    }
            


}

/***************************************************************************
*                           ctrlData_start                                 *
****************************************************************************
* DESCRIPTION:  This function start the link control algorithms. It start
*               each algorithm (Rate Adaptation) if needed.
* 
* INPUTS:       hCtrlData - the object
*       
* OUTPUT:       
* 
* RETURNS:      OK
*               NOK
***************************************************************************/

TI_STATUS ctrlData_start(TI_HANDLE hCtrlData)   
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;

    pCtrlData->ctrlDataStartStoplinkControlAlg = TRUE;


    /* start Rate Adaptation if needed */
    if(pCtrlData->ctrlDataRateControlEnable == TRUE)
    {
        rateAdaptation_start(pCtrlData->pRateAdaptation);
    }


    WLAN_REPORT_INFORMATION(pCtrlData->hReport, CTRL_DATA_MODULE_LOG, 
        (" ctrlData_start() : Link control algorithms started successfully \n"));

    return OK;
}

/***************************************************************************
*                           ctrlData_stop                                  *
****************************************************************************
* DESCRIPTION:  This function stop the link control algorithms. 
*
* INPUTS:       hCtrlData - the object
*       
* OUTPUT:       
* 
* RETURNS:      OK
*               NOK
***************************************************************************/

TI_STATUS ctrlData_stop(TI_HANDLE hCtrlData)    
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;

    /* stop link control algorithm */
    pCtrlData->ctrlDataStartStoplinkControlAlg = FALSE;

    /* set modulation option to default value*/
    pCtrlData->ctrlDataCurrentModulationType = DEF_CURRENT_MUDULATION_TYPE;

    /* set Preamble length option to default value*/
    pCtrlData->ctrlDataCurrentPreambleType = DEF_CURRENT_PREAMBLE;

    /* set mgmt rate to default value */
    pCtrlData->ctrlDataCurrentBasicRate = DEF_BASIC_RATE;
    pCtrlData->ctrlDataBasicRateBitMask = DEF_BASIC_RATE_MASK;
    pCtrlData->ctrlDataCurrentBasicModulationType = DRV_MODULATION_QPSK;

    os_memoryZero(pCtrlData->hOs, 
                  &pCtrlData->tsrsParameters, 
                  sizeof(pCtrlData->tsrsParameters));

    WLAN_REPORT_INFORMATION(pCtrlData->hReport, CTRL_DATA_MODULE_LOG,
        (" ctrlData_stop() : Link control algorithms stoped \n"));

    rateAdaptation_stop(pCtrlData->pRateAdaptation);

    return OK;
}

/***************************************************************************
*                       ctrlData_receiveParamFromRx                        *
****************************************************************************
* DESCRIPTION:  This function receive Msdu Rx parameters from the Rx data
*               module, update counters.
*
* INPUTS:       hCtrlData - the object
*               pRxMsduInfo - Information about the receive msdu
* OUTPUT:       
* 
* RETURNS:      OK
*               NOK
***************************************************************************/
#ifdef SUPPORT_4X
TI_STATUS ctrlData_rxMsdu(TI_HANDLE         hCtrlData, 
                          mem_MSDU_T        **pRxMsdu)  
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;
    
    if(pCtrlData->ctrlDataStartStoplinkControlAlg == FALSE)
        return OK;

    if((pCtrlData->ctrlDataFourXEnable == TRUE) &&
       (pCtrlData->ctrlDataCurrentBssType == BSS_INFRASTRUCTURE))
    {
        /* Call fourX function */
        if(fourX_rxMsdu(pCtrlData->pFourX, pRxMsdu) != OK)
        {
            WLAN_REPORT_ERROR(pCtrlData->hReport, CTRL_DATA_MODULE_LOG,
                (" failed in fourX_rxMsdu\n"));
            return NOK;
        }
    }

    return OK;
}
#endif
/***************************************************************************
*                       ctrlData_getTxAttributes                           *
****************************************************************************
* DESCRIPTION:  This function set the transmited parameters for a 
*               specific msdu
*
* INPUTS:       hCtrlData - the object
*               txFlags - Information about the msdu
*               
* OUTPUT:       pTxAttr - pointer to the tx parameters structure
* 
* RETURNS:      OK
*               NOK
***************************************************************************/

TI_STATUS ctrlData_getTxAttributes(TI_HANDLE hCtrlData, UINT32 txFlags, 
                                txData_attr_t *pTxAttr, UINT32 ac)  
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;
    rateModulation4x_table_t* pRateModulation;

    os_memoryZero(pCtrlData->hOs, pTxAttr, sizeof(txData_attr_t));


    if( ((txFlags & TX_DATA_MULTICAST_FRAME) && (pCtrlData->ctrlDataCurrentBssType == BSS_INDEPENDENT)) || 
            (txFlags & TX_DATA_MGMT_MSDU) )
    {
        /* BCAST packets in IBSS should be sent at 2M and not in the highest basic rate. */
        if (pCtrlData->ctrlDataCurrentRateMask & DRV_RATE_MASK_2_BARKER)
        {
            pTxAttr->Rate = DRV_RATE_2M;        
        } 
        else 
        {
            pTxAttr->Rate = ctrlData_getClosestTSRate(pCtrlData, ac, pCtrlData->ctrlDataCurrentBasicRate);
        }
 
        /* by default use USER_RATE_CLASS for this kind of packets */
        pTxAttr->txRatePolicyId = pCtrlData->tsrsParameters[ac].fwPolicyID[USER_RATE_CLASS];
    }
    /* For regular data packets use the rate-adaptation rates. */
    else
    {
        pRateModulation = rateAdaptation_getCurrent(pCtrlData->pRateAdaptation);
        pTxAttr->Rate = ctrlData_getClosestTSRate(pCtrlData, ac, pRateModulation->rate);

        /* rate class Id ( retries profile per rate) */
        pTxAttr->txRatePolicyId = pCtrlData->tsrsParameters[ac].fwPolicyID[pCtrlData->currClientRateID];

        /* For voice delivery PSPoll, use only basic rates */
        if (txFlags & TX_DATA_PS_POLL)
        {
            paramInfo_t param;
            modulationType_e tempModulation;

            param.paramType = SITE_MGR_OPERATIONAL_MODE_PARAM;
            siteMgr_getParam(pCtrlData->hSiteMgr, &param);
            
            /* Get the max rate and modulation from the BasicRateBitMask. */
            getMaxRate(pCtrlData->ctrlDataBasicRateBitMask, 
                       &pTxAttr->Rate, 
                       &tempModulation, 
                       param.content.siteMgrDot11OperationalMode);
        }


    }

    /* convert Application rate to HW rate */
    ConvertAppRateToHwBitMapRate(pTxAttr->Rate, &(pTxAttr->HwRate));

    WLAN_REPORT_DEBUG_TX(pCtrlData->hReport, 
            ("%s: Rate = %d, HwRate = 0x%x\n",
                __FUNCTION__,
                pTxAttr->Rate,
                pTxAttr->HwRate));

    return OK;
}

/***************************************************************************
*                        ctrlData_txCompleteStatus                         *
****************************************************************************
* DESCRIPTION:  This function is called by the Hal for every Tx supportedBitMap 
*               Interrupt - it update the rate adaptation algorithm about
*               the status of the last transmission and used as a trigger
*               for the Tx scheduler.
*
* INPUTS:       hCtrlData - the object
*               CmpltTxAttr - Information structure about the last 
*               transmission
*
* OUTPUT:       
* 
* RETURNS:      void
***************************************************************************/
void ctrlData_txCompleteStatus( TI_HANDLE hCtrlData,
                                       txCompleteAttr_t *pTxCompleteAttr )
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;
    UINT8       txCompleteFlags;
    BOOL        frameDataType = 0;
    rate_e      txActualRate;
    rate_e      txRequestRate;
    UINT8       qId;
    txPacketIdAttr_t    *pPacketId = (txPacketIdAttr_t*)pTxCompleteAttr->packetId; 


    /* 
     * perform rate adaptation algorithm if host processes packets 
     * and  not TNET.
     * NOTE: MSDU was already freed in sendPacketTransfer
     */
    frameDataType = pPacketId->bDataMsdu;
    qId = pPacketId->txQid; 

    ConvertHwBitRateToAppRate(pTxCompleteAttr->rate, &txActualRate);
    ConvertHwBitRateToAppRate(pPacketId->maxTransmitRate, &txRequestRate);

    /* perform rate adaptation algorithm if needed */
    if((pCtrlData->ctrlDataRateControlEnable == TRUE ) && 
       (pCtrlData->ctrlDataStartStoplinkControlAlg == TRUE) &&
           (frameDataType))
        
    {
        rateAdaptation_updateRateAdaptation(pCtrlData->pRateAdaptation, txActualRate,
                                            txRequestRate,pTxCompleteAttr->status, 0);
    }
    txCompleteFlags = pPacketId->txCompleteFlags;

    if(txCompleteFlags & TX_DATA_DISASSOC_SYNC_TRIG)
    {
        WLAN_REPORT_INFORMATION(pCtrlData->hReport, CTRL_DATA_MODULE_LOG,("Call disconnect test upon NULL data"));
            pCtrlData->disassocSentCBFunc( pCtrlData->disassocSentCBObj );
    }


    if(txCompleteFlags & TX_DATA_DEAUTH_SYNC_TRIG)
    {
            WLAN_REPORT_INFORMATION(pCtrlData->hReport, CTRL_DATA_MODULE_LOG,
            ("De Auth TxCmplt: txStatus = %d, txActualRate = %d  \n",pTxCompleteAttr->status,pTxCompleteAttr->rate));
            pCtrlData->disassocSentCBFunc( pCtrlData->disassocSentCBObj );
    }
    if(txData_isQueueUseMediumTime(pCtrlData->hTxData , qId) == TRUE )
    {
        WLAN_REPORT_INFORMATION(pCtrlData->hReport, CTRL_DATA_MODULE_LOG, 
            (" ctrlData_txCompleteStatus() :usedTime = %d  qNum = %d\n", 
            pTxCompleteAttr->actualDurationInAir, qId));
        
        txData_updateUsedTime(pCtrlData->hTxData, 
                              qId, 
                              pTxCompleteAttr->actualDurationInAir);
    }   
    /*
     * update tx complete status to txData (which schedule another packet also );
     */
    txData_txCompleteUpdate( pCtrlData->hTxData, pTxCompleteAttr );

}

/***************************************************************************
*                        ctrlData_resetCounters                            *
****************************************************************************
* DESCRIPTION:  This function reset the Ctrl Data module counters
*
* INPUTS:       hCtrlData - the object
*
* OUTPUT:       
* 
* RETURNS:      void
***************************************************************************/
static void ctrlData_resetCounters(TI_HANDLE hCtrlData)
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;

    os_memoryZero(pCtrlData->hOs,&pCtrlData->ctrlDataCounters, 
                                sizeof(ctrlDataCounters_t));
}

/***************************************************************************
*                   ctrlData_getCurrBssTypeAndCurrBssId                    *
****************************************************************************
* DESCRIPTION:  This function return the current BSSID and the 
*               current BSS Type
*
* INPUTS:       hCtrlData - the object
*               
* OUTPUT:       pCurrBssid - pointer to return the current bssid    
*               pCurrBssType - pointer to return the current bss type 
*
* RETURNS:      void
***************************************************************************/
void ctrlData_getCurrBssTypeAndCurrBssId(TI_HANDLE hCtrlData, macAddress_t *pCurrBssid, 
                                           bssType_e *pCurrBssType)
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;
    
    MAC_COPY(pCtrlData->hOs, (pCurrBssid), (&pCtrlData->ctrlDataCurrentBSSID));
    *pCurrBssType = pCtrlData->ctrlDataCurrentBssType;

}

#ifdef SUPPORT_4X
/***************************************************************************
*                   ctrlData_get4xInfoElemnt                               *
****************************************************************************
* DESCRIPTION:  
*
* INPUTS:       hCtrlData - the object
*               
* OUTPUT:       
*               
* RETURNS:      
***************************************************************************/
TI_STATUS ctrlData_get4xInfoElemnt(TI_HANDLE hCtrlData, 
                                   dot11_4X_t* fourXInfoElemnt)
{
    TI_STATUS Status = NOK;

    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;
    
    if((pCtrlData->ctrlDataFourXEnable == TRUE) &&
       (pCtrlData->ctrlDataCurrentBssType == BSS_INFRASTRUCTURE))
    {
        Status = fourXManager_get4xInfoElemnt(pCtrlData->pFourX, fourXInfoElemnt);
    }
    
    return Status;
}
#endif
/***************************************************************************
*                           ctrlData_get4xStatus                           *
****************************************************************************
* DESCRIPTION:  
*
* INPUTS:       hCtrlData - the object
*               
* OUTPUT:       
*               
* RETURNS:      
***************************************************************************/
TI_STATUS ctrlData_get4xStatus(TI_HANDLE hCtrlData,BOOL* fourXEnable)
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;

    *fourXEnable = pCtrlData->ctrlDataCerruentFourXstate;
    return OK;
}
#ifdef SUPPORT_4X
/***************************************************************************
*                           ctrlData_evalSite                              *
****************************************************************************
* DESCRIPTION:  
*
* INPUTS:       hCtrlData - the object
*               
* OUTPUT:       
*               
* RETURNS:      
***************************************************************************/
TI_STATUS ctrlData_evalSite(TI_HANDLE hCtrlData, 
                            dot11_4X_t* site4xParams, 
                            UINT32 *matchingLevel)
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;

    if((pCtrlData->ctrlDataFourXEnable == TRUE) &&
       (pCtrlData->ctrlDataCurrentBssType == BSS_INFRASTRUCTURE))
    {
        return (fourXManager_evalSite(pCtrlData->pFourX, site4xParams, matchingLevel));
    }

    return OK;
}
#endif
/***************************************************************************
*                           ctrlData_setSite                               *
****************************************************************************
* DESCRIPTION:  
*
* INPUTS:       hCtrlData - the object
*               
* OUTPUT:       
*               
* RETURNS:      
***************************************************************************/

TI_STATUS ctrlData_setSite(TI_HANDLE hCtrlData,
                           dot11_4X_t* site4xParams)
{
    TI_STATUS   status = NOK;
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;
#ifdef SUPPORT_4X
    if((pCtrlData->ctrlDataFourXEnable == TRUE) &&
       (pCtrlData->ctrlDataCurrentBssType == BSS_INFRASTRUCTURE))
    {
        status = fourXManager_setSite(pCtrlData->pFourX, site4xParams);
    }
#endif
    if(status != OK)
        pCtrlData->ctrlDataCerruentFourXstate = FALSE;
    else
        pCtrlData->ctrlDataCerruentFourXstate = TRUE;

    return status;
}

#ifdef SUPPORT_4X
/***************************************************************************
*                       ctrlData_txDequeueMsdu                            *
****************************************************************************
* DESCRIPTION:  
*
* INPUTS:       hCtrlData - the object
*               
* OUTPUT:       
*               
* RETURNS:      
***************************************************************************/
TI_STATUS ctrlData_txDequeueMsdu(TI_HANDLE          hCtrlData, 
                                 mem_MSDU_T**       buildMsduPtr,
                                 MsduList_t*        pMsduList, 
                                 whalTx_attr_t*     pWhalTx_attr,
                                 hwTxInformation_t* pHwTxInformation)
{
    TI_STATUS   status;
    UINT32      currNumOfMsdu;

    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;

    *buildMsduPtr = NULL;

    currNumOfMsdu = msduList_getCurrNumOfMsdu(pMsduList);
    if(currNumOfMsdu == 0)
        return DO_NOT_SEND_MSDU;

    if(pCtrlData->ctrlDataCerruentFourXstate == TRUE)
    {
        /* call 4x */ 
        status = fourX_txMsduDeQueue(pCtrlData->pFourX, buildMsduPtr, pMsduList, pHwTxInformation);
        if(status != OK)
        {
            return DO_NOT_SEND_MSDU;
        }

        ctrlData_getTxAttributes(pCtrlData, (*buildMsduPtr)->txFlags, pWhalTx_attr, QOS_AC_BE); /* stub */
        return SEND_ONE_MSDU;

    }
    return FOUR_X_DISABLE;

}

/***************************************************************************
*                       ctrlData_txMsdu                                    *
****************************************************************************
* DESCRIPTION:  
*
* INPUTS:       hCtrlData - the object
*               
* OUTPUT:       
*               
* RETURNS:      
***************************************************************************/
TI_STATUS ctrlData_txMsdu(TI_HANDLE         hCtrlData, 
                          mem_MSDU_T**      msduPtr)
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;

    if(pCtrlData->ctrlDataCerruentFourXstate == TRUE)
    {
        fourX_txMsduBeforInsertToQueue(pCtrlData->pFourX, msduPtr);
    }

    return OK;
}
#endif /* SUPPORT_4X */

/***************************************************************************
*                       ctrlData_setTspecsRateEvent                       *
****************************************************************************
* DESCRIPTION:  
*
* INPUTS:       hCtrlData - the object
*               
* OUTPUT:       
*               
* RETURNS:      
***************************************************************************/

void ctrlData_setTspecsRateEvent(TI_HANDLE          hCtrlData,
                                    UINT8           acID,
                                    BOOL            enableEvent)
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;

     rateAdaptation_setTspecsRateEvent(pCtrlData->pRateAdaptation, acID, enableEvent);

}

/***************************************************************************
*                   ctrlData_setTspecsRateThresholds                      *
****************************************************************************
* DESCRIPTION:  
*
* INPUTS:       hCtrlData - the object
*               
* OUTPUT:       
*               
* RETURNS:      
***************************************************************************/

void ctrlData_setTspecsRateThresholds(TI_HANDLE     hCtrlData,
                                      UINT8         acID,
                                      UINT8     highRateThreshold,
                                      UINT8     lowRateThreshold)
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;
    
    rateAdaptation_setTspecsRateThresholds(pCtrlData->pRateAdaptation, acID, highRateThreshold, lowRateThreshold);
}

/************************************************************************
 *                        Classifier functions
 ************************************************************************ */

/************************************************************************
 *                        ctrlData_clsfrClassifyTxMSDU
 ************************************************************************
      
Input:  

* hCtrlData: hCtrlData - the object
* pMsdu: pointer to the MSDU
* packet_DTag: NDIS Packet 802.1 user priority (UP)

Output:  

OK on success and PARAM_VALUE_NOT_VALID in case of input parameters problems.
If the value PARAM_VALUE_NOT_VALID is returned, the MSDU qosTag field is zero. 

Description:  

This function performs the classification algorithm on the MSDU by calling the 
Classifier_classifyTxMSDU API.

************************************************************************/

TI_STATUS ctrlData_ClsfrClassifyTxMSDU(TI_HANDLE    hCtrlData, 
                                       mem_MSDU_T   *pMsdu, 
                                       UINT8        packet_DTag)
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;

    /* check parameters validity */
    if(!hCtrlData)
        return NOK;

    if (pMsdu == NULL) 
    {
        WLAN_REPORT_ERROR(pCtrlData->hReport, CTRL_DATA_MODULE_LOG,
            (" ctrlData_ClsfrClassifyTxMSDU() : parametrs value error (the MSDU's qosTag is not updated) \n"));
        return PARAM_VALUE_NOT_VALID;
    }

    return (Classifier_classifyTxMSDU(pCtrlData->pClsfr, pMsdu, packet_DTag));

}



/************************************************************************
 *                        ctrlData_clsfrSetClsfrType
 ************************************************************************
      
Input:  

* hCtrlData: hCtrlData - the object
* newClsfrType: the new classifier type

Output:  

OK on success and PARAM_VALUE_NOT_VALID in case of input parameters problems.
If the value PARAM_VALUE_NOT_VALID is returned, the classifier type is not updated.

Description:  

This function changes the active classifier type by calling the 
Classifier_setClsfrType API.

************************************************************************/


TI_STATUS ctrlData_clsfrSetClsfrType(TI_HANDLE          hCtrlData,
                                    clsfrTypeAndSupport     newClsfrType)
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;
    /* check parameters validity */
    if(!hCtrlData)
        return PARAM_VALUE_NOT_VALID;
        
    return (Classifier_setClsfrType(pCtrlData->pClsfr, (clsfr_type_e)newClsfrType.ClsfrType));
    
}


/*-----------------------------------------------------------------------------
Routine Name: ctrlData_ToggleTrafficIntensityNotification
Routine Description: turns ON/OFF traffic intensity notification events
                     from Traffic Monitor module
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
void ctrlData_ToggleTrafficIntensityNotification (TI_HANDLE hCtrlData, BOOL enabledFlag)
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;
    UINT8 idx;

   if (enabledFlag)
   {
      for (idx=0; idx < CTRL_DATA_TRAFFIC_INTENSITY_MAX_EVENTS; idx++)
      {
         TrafficMonitor_StartEventNotif (pCtrlData->hTrafficMonitor,pCtrlData->ctrlDataTrafficThresholdEvents[idx]);
      }
      WLAN_REPORT_INFORMATION(pCtrlData->hReport, CTRL_DATA_MODULE_LOG,
         ("ctrlData_ToggleTrafficIntensityNotification (TRUE)\n"));
   }
   else
   {
      for (idx=0; idx < CTRL_DATA_TRAFFIC_INTENSITY_MAX_EVENTS; idx++)
      {
         TrafficMonitor_StopEventNotif (pCtrlData->hTrafficMonitor,pCtrlData->ctrlDataTrafficThresholdEvents[idx]);
      }
      WLAN_REPORT_INFORMATION(pCtrlData->hReport, CTRL_DATA_MODULE_LOG,
         ("ctrlData_ToggleTrafficIntensityNotification (FALSE)\n"));
   }
   pCtrlData->ctrlDataTrafficIntensityEventsEnabled = enabledFlag;

}

/*-----------------------------------------------------------------------------
Routine Name: ctrlData_UnregisterTrafficIntensityEvents
Routine Description: unregisters existing events from traffic monitor
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
static void ctrlData_UnregisterTrafficIntensityEvents (TI_HANDLE hCtrlData)
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;
    UINT8 idx;

    /* Loop through events and unregister them */
    for (idx=0; idx < CTRL_DATA_TRAFFIC_INTENSITY_MAX_EVENTS; idx++)
    {
       TrafficMonitor_UnregEvent (pCtrlData->hTrafficMonitor,pCtrlData->ctrlDataTrafficThresholdEvents[idx]);
    }

    WLAN_REPORT_INFORMATION(pCtrlData->hReport, CTRL_DATA_MODULE_LOG,
       ("ctrlData_UnregisterTrafficIntensityEvents: Unregistered all events\n"));

}


/*-----------------------------------------------------------------------------
Routine Name: ctrlData_RegisterTrafficIntensityEvents
Routine Description: Registers traffic intensity threshold events through traffic monitor
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
static void ctrlData_RegisterTrafficIntensityEvents (TI_HANDLE hCtrlData)
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;
    TrafficAlertRegParm_t TrafficAlertRegParm;
    TI_STATUS status;

    /* Register high threshold "direction up" event */
    TrafficAlertRegParm.CallBack = ctrlData_TrafficThresholdCrossed;
    TrafficAlertRegParm.Context = hCtrlData; 
    TrafficAlertRegParm.Cookie =  CTRL_DATA_TRAFFIC_INTENSITY_HIGH_CROSSED_ABOVE;    
    TrafficAlertRegParm.Direction = TRAFF_UP;
    TrafficAlertRegParm.Trigger = TRAFF_EDGE;
    TrafficAlertRegParm.TimeIntervalMs = pCtrlData->ctrlDataTrafficIntensityThresholds.TestInterval;
    TrafficAlertRegParm.Threshold = pCtrlData->ctrlDataTrafficIntensityThresholds.uHighThreshold;
    TrafficAlertRegParm.MonitorType = TX_RX_DIRECTED_FRAMES;
    pCtrlData->ctrlDataTrafficThresholdEvents[0] = TrafficMonitor_RegEvent(pCtrlData->hTrafficMonitor,&TrafficAlertRegParm,FALSE); 

    if (pCtrlData->ctrlDataTrafficThresholdEvents[0] == NULL)
    {
         WLAN_REPORT_ERROR(pCtrlData->hReport, CTRL_DATA_MODULE_LOG,
            (" ctrlData_RegisterTrafficIntensityEvents() : Failed to register high treshold event (TRAFF_UP) \n"));
         return;
    }

    /* Register high threshold "direction down" event*/
    TrafficAlertRegParm.Cookie =  CTRL_DATA_TRAFFIC_INTENSITY_HIGH_CROSSED_BELOW;    
    TrafficAlertRegParm.Direction = TRAFF_DOWN;
    TrafficAlertRegParm.Trigger = TRAFF_EDGE;
    TrafficAlertRegParm.Threshold = pCtrlData->ctrlDataTrafficIntensityThresholds.uHighThreshold;
    pCtrlData->ctrlDataTrafficThresholdEvents[1] = TrafficMonitor_RegEvent(pCtrlData->hTrafficMonitor,&TrafficAlertRegParm,FALSE); 

    if (pCtrlData->ctrlDataTrafficThresholdEvents[1] == NULL)
    {
         WLAN_REPORT_ERROR(pCtrlData->hReport, CTRL_DATA_MODULE_LOG,
            (" ctrlData_RegisterTrafficIntensityEvents() : Failed to register high treshold event (TRAFF_DOWN) \n"));
         return;
    }

    /* Define the "direction below" and "direction above" events as opposites (events that reset eachother)*/
    status = TrafficMonitor_SetRstCondition(pCtrlData->hTrafficMonitor,
                                            pCtrlData->ctrlDataTrafficThresholdEvents[0],
                                            pCtrlData->ctrlDataTrafficThresholdEvents[1],
                                            TRUE);

    if (status != OK)
    {
      WLAN_REPORT_ERROR (pCtrlData->hReport, CTRL_DATA_MODULE_LOG,
         ("ctrlData_RegisterTrafficIntensityEvents: TrafficMonitor_SetRstCondition returned status = %d\n",status));
    }

    /* Register low threshold "direction up" event */
    TrafficAlertRegParm.Cookie =  CTRL_DATA_TRAFFIC_INTENSITY_LOW_CROSSED_ABOVE;    
    TrafficAlertRegParm.Direction = TRAFF_UP;
    TrafficAlertRegParm.Trigger = TRAFF_EDGE;
    TrafficAlertRegParm.Threshold = pCtrlData->ctrlDataTrafficIntensityThresholds.uLowThreshold;
    pCtrlData->ctrlDataTrafficThresholdEvents[2] = TrafficMonitor_RegEvent(pCtrlData->hTrafficMonitor,&TrafficAlertRegParm,FALSE); 

    if (pCtrlData->ctrlDataTrafficThresholdEvents[2] == NULL)
    {
         WLAN_REPORT_ERROR(pCtrlData->hReport, CTRL_DATA_MODULE_LOG,
            (" ctrlData_RegisterTrafficIntensityEvents() : Failed to register low treshold event (TRAFF_UP) \n"));
         return;
    }

    /* Register low threshold "direction below" event */
    TrafficAlertRegParm.Cookie =  CTRL_DATA_TRAFFIC_INTENSITY_LOW_CROSSED_BELOW;
    TrafficAlertRegParm.Direction = TRAFF_DOWN;
    TrafficAlertRegParm.Trigger = TRAFF_EDGE;
    TrafficAlertRegParm.Threshold = pCtrlData->ctrlDataTrafficIntensityThresholds.uLowThreshold;
    pCtrlData->ctrlDataTrafficThresholdEvents[3] = TrafficMonitor_RegEvent(pCtrlData->hTrafficMonitor,&TrafficAlertRegParm,FALSE); 

    if (pCtrlData->ctrlDataTrafficThresholdEvents[3] == NULL)
    {
         WLAN_REPORT_ERROR(pCtrlData->hReport, CTRL_DATA_MODULE_LOG,
            (" ctrlData_RegisterTrafficIntensityEvents() : Failed to register low treshold event (TRAFF_DOWN) \n"));
         return;
    }

    /* Define the "direction below" and "direction above" events as opposites (events that reset eachother)*/
    status = TrafficMonitor_SetRstCondition(pCtrlData->hTrafficMonitor,
                                            pCtrlData->ctrlDataTrafficThresholdEvents[2],
                                            pCtrlData->ctrlDataTrafficThresholdEvents[3],
                                            TRUE);

    if (status != OK)
    {
      WLAN_REPORT_ERROR (pCtrlData->hReport, CTRL_DATA_MODULE_LOG,
         ("ctrlData_RegisterTrafficIntensityEvents: TrafficMonitor_SetRstCondition returned status = %d\n",status));
    }
  
    WLAN_REPORT_INFORMATION(pCtrlData->hReport, CTRL_DATA_MODULE_LOG,
       (" ctrlData_RegisterTrafficIntensityEvents() : finished registering all events \n"));

}


/*-----------------------------------------------------------------------------
Routine Name: ctrlData_TrafficThresholdCrossed
Routine Description: called whenever traffic intensity threshold is crossed. 
                     notifies event handler to send appropriate event with threshold parameters.
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
void ctrlData_TrafficThresholdCrossed(TI_HANDLE Context,UINT32 Cookie)
{
    ctrlData_t *pCtrlData = (ctrlData_t *)Context;
    trafficIntensityThresholdCross_t crossInfo;

    switch(Cookie) 
    {
    case CTRL_DATA_TRAFFIC_INTENSITY_HIGH_CROSSED_ABOVE:
            crossInfo.thresholdCross = (UINT32)HIGH_THRESHOLD_CROSS;
            crossInfo.thresholdCrossDirection = (UINT32)CROSS_ABOVE;
            EvHandlerSendEvent(pCtrlData->hEvHandler, IPC_EVENT_TRAFFIC_INTENSITY_THRESHOLD_CROSSED, (UINT8 *)&crossInfo,sizeof(trafficIntensityThresholdCross_t));
       break;

    case CTRL_DATA_TRAFFIC_INTENSITY_HIGH_CROSSED_BELOW:
            crossInfo.thresholdCross = (UINT32)HIGH_THRESHOLD_CROSS;
            crossInfo.thresholdCrossDirection = (UINT32)CROSS_BELOW;
            EvHandlerSendEvent(pCtrlData->hEvHandler, IPC_EVENT_TRAFFIC_INTENSITY_THRESHOLD_CROSSED, (UINT8 *)&crossInfo,sizeof(trafficIntensityThresholdCross_t));
       break;

    case CTRL_DATA_TRAFFIC_INTENSITY_LOW_CROSSED_ABOVE:
            crossInfo.thresholdCross = (UINT32)LOW_THRESHOLD_CROSS;
            crossInfo.thresholdCrossDirection = (UINT32)CROSS_ABOVE;
            EvHandlerSendEvent(pCtrlData->hEvHandler, IPC_EVENT_TRAFFIC_INTENSITY_THRESHOLD_CROSSED, (UINT8 *)&crossInfo,sizeof(trafficIntensityThresholdCross_t));
       break;

    case CTRL_DATA_TRAFFIC_INTENSITY_LOW_CROSSED_BELOW:
            crossInfo.thresholdCross = (UINT32)LOW_THRESHOLD_CROSS;
            crossInfo.thresholdCrossDirection = (UINT32)CROSS_BELOW;
            EvHandlerSendEvent(pCtrlData->hEvHandler, IPC_EVENT_TRAFFIC_INTENSITY_THRESHOLD_CROSSED, (UINT8 *)&crossInfo,sizeof(trafficIntensityThresholdCross_t));
       break;
    default:
         WLAN_REPORT_ERROR(pCtrlData->hReport, CTRL_DATA_MODULE_LOG,
                (" ctrlData_TrafficThresholdCrossed() : Unknown cookie received from traffic monitor !!! \n"));
       break;
   }
    
}

/***************************************************************************
*                           ctrlData_buildHwBitMapFromArray                 *
****************************************************************************
* DESCRIPTION:  builds HwRatesBitMap (supported rates) for txRatePolicy  
*               using an array that consist number of retries for each rate
*               all ew do is : if retry > 0 than the HwBit is ON.
* 
* INPUTS:       Array of retries
*       
* OUTPUT:       Bit Map of Hw rates.
* 
* RETURNS:      Bit Map of Hw rates.
*           
***************************************************************************/
static UINT32 ctrlData_buildHwBitMapFromArray(policyClassRatesArray_t *pArray)
{
    txRateClassId_e  TxRateIndex;
    UINT32 policyRateMask = 0;
    rateMask_e tempArray[MAX_NUM_OF_TX_RATES_IN_CLASS] = 
    {
    DRV_RATE_MASK_54_OFDM,DRV_RATE_MASK_48_OFDM,DRV_RATE_MASK_36_OFDM,
    DRV_RATE_MASK_24_OFDM,DRV_RATE_MASK_22_PBCC,DRV_RATE_MASK_18_OFDM,
    DRV_RATE_MASK_12_OFDM,DRV_RATE_MASK_11_CCK,DRV_RATE_MASK_9_OFDM,
    DRV_RATE_MASK_6_OFDM,DRV_RATE_MASK_5_5_CCK,DRV_RATE_MASK_2_BARKER,
    DRV_RATE_MASK_1_BARKER};



    for (TxRateIndex = txPolicy54; TxRateIndex < MAX_NUM_OF_TX_RATES_IN_CLASS; TxRateIndex++)
    {
        if (pArray->txRate[TxRateIndex] > 0 )
        {
            policyRateMask |= tempArray[TxRateIndex]; 
        }
    }

    return policyRateMask;
}


/*************************************************************************
 *                                                                       *
 *                          DEBUG FUNCTIONS                              *
 *                                                                       *
 *************************************************************************/

void ctrlData_printTxParameters(TI_HANDLE hCtrlData)
{
    WLAN_OS_REPORT(("            Tx Parameters            \n"));
    WLAN_OS_REPORT(("-------------------------------------\n"));
    WLAN_OS_REPORT(("currentPreamble                     = %d\n\n",((ctrlData_t *)hCtrlData)->ctrlDataCurrentPreambleType));
    WLAN_OS_REPORT(("currentModulation                   = %d\n",  ((ctrlData_t *)hCtrlData)->ctrlDataCurrentModulationType));
    WLAN_OS_REPORT(("ctrlDataCurrentBasicModulationType  = %d\n",  ((ctrlData_t *)hCtrlData)->ctrlDataCurrentBasicModulationType));
    WLAN_OS_REPORT(("ctrlDataCurrentBasicRate            = %d\n",  ((ctrlData_t *)hCtrlData)->ctrlDataCurrentBasicRate));
    WLAN_OS_REPORT(("ctrlDataBasicRateBitMask            = 0x%X\n",((ctrlData_t *)hCtrlData)->ctrlDataBasicRateBitMask));
    WLAN_OS_REPORT(("ctrlDataCurrentRateMask             = 0x%X\n",((ctrlData_t *)hCtrlData)->ctrlDataCurrentRateMask));
}  

void ctrlData_printRateAdaptation(TI_HANDLE hCtrlData)
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;

    rateAdaptation_print(pCtrlData->pRateAdaptation);
}
#ifdef SUPPORT_4X
void ctrlData_printFourX(TI_HANDLE hCtrlData)
{
    ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;

    fourX_printParams(pCtrlData->pFourX);
}
#endif

void ctrlData_printCtrlBlock(TI_HANDLE hCtrlData)
{
    rateClassClients_e  clientID;
    txRateClassId_e  TxRateIndex;

    WLAN_OS_REPORT(("    CtrlData BLock    \n"));
    WLAN_OS_REPORT(("----------------------\n"));

    WLAN_OS_REPORT(("hSiteMgr = 0x%X\n",((ctrlData_t *)hCtrlData)->hSiteMgr));
    WLAN_OS_REPORT(("hTxData = 0x%X\n",((ctrlData_t *)hCtrlData)->hTxData));
    WLAN_OS_REPORT(("hWhalCtrl = 0x%X\n",((ctrlData_t *)hCtrlData)->hWhalCtrl));
    WLAN_OS_REPORT(("hOs = 0x%X\n",((ctrlData_t *)hCtrlData)->hOs));
    WLAN_OS_REPORT(("hReport = 0x%X\n",((ctrlData_t *)hCtrlData)->hReport));

    WLAN_OS_REPORT(("ctrlDataDeviceMacAddress = 0x%X.0x%X.0x%X.0x%X.0x%X.0x%X. \n", ((ctrlData_t *)hCtrlData)->ctrlDataDeviceMacAddress.addr[0],
                                                                                    ((ctrlData_t *)hCtrlData)->ctrlDataDeviceMacAddress.addr[1],
                                                                                    ((ctrlData_t *)hCtrlData)->ctrlDataDeviceMacAddress.addr[2],
                                                                                    ((ctrlData_t *)hCtrlData)->ctrlDataDeviceMacAddress.addr[3],
                                                                                    ((ctrlData_t *)hCtrlData)->ctrlDataDeviceMacAddress.addr[4],
                                                                                    ((ctrlData_t *)hCtrlData)->ctrlDataDeviceMacAddress.addr[5]));

    WLAN_OS_REPORT(("ctrlDataCurrentBSSID = 0x%X.0x%X.0x%X.0x%X.0x%X.0x%X. \n", ((ctrlData_t *)hCtrlData)->ctrlDataCurrentBSSID.addr[0],
                                                                                ((ctrlData_t *)hCtrlData)->ctrlDataCurrentBSSID.addr[1],
                                                                                ((ctrlData_t *)hCtrlData)->ctrlDataCurrentBSSID.addr[2],
                                                                                ((ctrlData_t *)hCtrlData)->ctrlDataCurrentBSSID.addr[3],
                                                                                ((ctrlData_t *)hCtrlData)->ctrlDataCurrentBSSID.addr[4],
                                                                                ((ctrlData_t *)hCtrlData)->ctrlDataCurrentBSSID.addr[5]));

    WLAN_OS_REPORT(("ctrlDataRateControlEnable = %d\n",((ctrlData_t *)hCtrlData)->ctrlDataRateControlEnable));
    WLAN_OS_REPORT(("ctrlDataFourXEnable = %d\n",((ctrlData_t *)hCtrlData)->ctrlDataFourXEnable));
    WLAN_OS_REPORT(("ctrlDataFourCurrState = %d\n",((ctrlData_t *)hCtrlData)->ctrlDataCerruentFourXstate));
    WLAN_OS_REPORT(("ctrlDataStartStoplinkControlAlg = %d\n",((ctrlData_t *)hCtrlData)->ctrlDataStartStoplinkControlAlg));

    WLAN_OS_REPORT(("ctrlDataCurrentBssType = %d\n",((ctrlData_t *)hCtrlData)->ctrlDataCurrentBssType));
    WLAN_OS_REPORT(("ctrlDataCurrentModulationType = %d\n",((ctrlData_t *)hCtrlData)->ctrlDataCurrentModulationType));
    WLAN_OS_REPORT(("ctrlDataCurrentBasicRate = %d\n",((ctrlData_t *)hCtrlData)->ctrlDataCurrentBasicRate));

    WLAN_OS_REPORT(("ctrlDataBasicRateBitMask = 0x%X\n",((ctrlData_t *)hCtrlData)->ctrlDataBasicRateBitMask));
    WLAN_OS_REPORT(("ctrlDataCurrentRateMask = 0x%X\n",((ctrlData_t *)hCtrlData)->ctrlDataCurrentRateMask));
    
    WLAN_OS_REPORT(("ctrlDataCurrentBasicModulationType = %d\n",((ctrlData_t *)hCtrlData)->ctrlDataCurrentBasicModulationType));
    WLAN_OS_REPORT(("ctrlDataCurrentPreambleType = %d\n",((ctrlData_t *)hCtrlData)->ctrlDataCurrentPreambleType));

    WLAN_OS_REPORT(("Traffic Intensity threshold events status: %s\n", (((ctrlData_t *)hCtrlData)->ctrlDataTrafficIntensityEventsEnabled ? "Enabled" : "Disabled")));
    WLAN_OS_REPORT(("Traffic Intensity high threshold: %d packets/sec \n", ((ctrlData_t *)hCtrlData)->ctrlDataTrafficIntensityThresholds.uHighThreshold));
    WLAN_OS_REPORT(("Traffic Intensity low threshold: %d packets/sec \n", ((ctrlData_t *)hCtrlData)->ctrlDataTrafficIntensityThresholds.uLowThreshold));
    WLAN_OS_REPORT(("Traffic Intensity test interval: %d ms\n", ((ctrlData_t *)hCtrlData)->ctrlDataTrafficIntensityThresholds.TestInterval));

    for (clientID = (rateClassClients_e)0 ; clientID < NUM_OF_RATE_CLASS_CLIENTS ; clientID++)
    {
        WLAN_OS_REPORT((" client = %d : bIsClassAvailable = %s currMask = 0x%x nextMask = 0x%x \n",
             clientID,(TRUE == ((ctrlData_t *)hCtrlData)->bIsClassAvailable[clientID] ? "TRUE" : "FALSE"),
             ((ctrlData_t *)hCtrlData)->currClientRateMask[clientID],((ctrlData_t *)hCtrlData)->nextClientRateMask[clientID]));
        WLAN_OS_REPORT(("Policy for client %d (starting from 54): \n",clientID));
        for (TxRateIndex = txPolicy54; TxRateIndex < MAX_NUM_OF_TX_RATES_IN_CLASS; TxRateIndex++)
        {
            WLAN_OS_REPORT(("retries for rate %d = %d\n",TxRateIndex,((ctrlData_t *)hCtrlData)->ctrlDataTxRatePolicy.rateClass[clientID*MAX_NUM_OF_AC].txRate[TxRateIndex]));   
        }
        WLAN_OS_REPORT(("Long retry = %d  ,  Short retry = %d\n",
            ((ctrlData_t *)hCtrlData)->ctrlDataTxRatePolicy.rateClass[clientID].longRetryLimit,((ctrlData_t *)hCtrlData)->ctrlDataTxRatePolicy.rateClass[clientID*MAX_NUM_OF_AC].shortRetryLimit));
    }
    WLAN_OS_REPORT(("current used client %d\n",((ctrlData_t *)hCtrlData)->currClientRateID));
}

void ctrlData_printCtrlCounters(TI_HANDLE hCtrlData)
{
    WLAN_OS_REPORT(("    CtrlData Counters    \n"));
    WLAN_OS_REPORT(("-------------------------\n"));
    
    WLAN_OS_REPORT(("icvFailCounter        = %d\n",((ctrlData_t *)hCtrlData)->ctrlDataCounters.icvFailCounter));
    WLAN_OS_REPORT(("keyNotFoundCounter    = %d\n",((ctrlData_t *)hCtrlData)->ctrlDataCounters.keyNotFoundCounter));
    WLAN_OS_REPORT(("MicFailureCounter     = %d\n",((ctrlData_t *)hCtrlData)->ctrlDataCounters.MicFailureCounter));
}


#ifdef TI_DBG

void ctrlData_printClsfrTable ( TI_HANDLE hCtrlData )
{
   ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;
   Classifier_dbgPrintClsfrTable (pCtrlData->pClsfr);
}

void ctrlData_clearClsfrTable ( TI_HANDLE hCtrlData )
{
   clsfrTypeAndSupport myLocalType;
   ctrlData_t *pCtrlData = (ctrlData_t *)hCtrlData;

   Classifier_getClsfrType (pCtrlData->pClsfr,&myLocalType);
   ctrlData_clsfrSetClsfrType (pCtrlData,myLocalType);
}

#endif


/***************************************************************************
*                           ctrlData_storeTSRateSet                        
****************************************************************************
* DESCRIPTION:  This function translates TSRS rates into map of retransmissions
*               similar to predefined clients rates retransmissions, and stores
*               in the Ctrl database
*
* INPUTS:       pCtrlData - the object
*               acID
*               rates array
*
* RETURNS:      -
****************************************************************************/
static void ctrlData_storeTSRateSet(ctrlData_t *pCtrlData, txDataQosParams_t *tsrsParams)
{
    UINT32 rateCount;
    UINT32 acID, rateID;
    UINT32 tsrsRequestedMap;
    rate_e rateNumber;
    rateClassClients_e clientNumber;
    txRateClassId_e rate_e_to_txRateClassId_e[DRV_RATE_MAX+1] = 
    {
        txPolicy1,  txPolicy1, txPolicy2, txPolicy5_5, txPolicy11,
        txPolicy22, txPolicy6, txPolicy9, txPolicy12,  txPolicy18,
        txPolicy24, txPolicy36, txPolicy48, txPolicy54
    };

    acID = tsrsParams->acID;
    os_memoryZero(pCtrlData->hOs, 
                  &(pCtrlData->tsrsParameters[acID]), 
                  sizeof(pCtrlData->tsrsParameters[acID]));


    for (clientNumber = (rateClassClients_e)0; clientNumber < NUM_OF_RATE_CLASS_CLIENTS; clientNumber++) 
    {
        tsrsRequestedMap = 0;

        for (rateCount = 0; rateCount < tsrsParams->tsrsArrLen; rateCount++) 
        {
            /* Erase Most significant bit in case it was raised to mark nominal PHY rates (& 0x7F) */
            /* Convert multiplication of 500kb/sec to rate_e and then to txRateClassId_e */
            /* and update retransmission map in accordance to USER_RATE client definitions */
            rateNumber = RateNumberToHost((tsrsParams->tsrsArr[rateCount] & 0x7F) >> 1);
            rateID = rate_e_to_txRateClassId_e[rateNumber];

            /* Update Rate Fallback policy map according to the class predefined policy map */
            if (pCtrlData->pCurrPolicyClassRatesArray[clientNumber].txRate[rateID] > 0)
            {
                pCtrlData->tsrsParameters[acID].policyClassRateMask[clientNumber] |= (1<<(rateNumber-1));
            }
            tsrsRequestedMap |= (1<<(rateNumber-1));
        }
        /* Update supportedRatesMask according to TSRS rates and rates supported for this class */
        pCtrlData->tsrsParameters[acID].supportedRatesMask[clientNumber] = 
            pCtrlData->nextClientRateMask[clientNumber] & tsrsRequestedMap;

        /* Check that Rate Fallback policy map is not empty; if this is a case, ignore pre-defined policy */
        if (pCtrlData->tsrsParameters[acID].policyClassRateMask[clientNumber] == 0)
        {
            pCtrlData->tsrsParameters[acID].policyClassRateMask[clientNumber] = 
                pCtrlData->tsrsParameters[acID].supportedRatesMask[clientNumber];
        }
    }
}

/***************************************************************************
*                           ctrlData_getClosestTSRate                          
****************************************************************************
* DESCRIPTION:  This function checks if the TSRS is defined for the requested
*               access category or not; if requested, it chooses from the TSRS
*               a rate that is as close as possible to the requestede one.
*
* INPUTS:       pCtrlData - the object
*               acID
*
* RETURNS:      -
****************************************************************************/
static rate_e ctrlData_getClosestTSRate(ctrlData_t *pCtrlData, UINT32 ac, rate_e givenRate)
{
    UINT32 tsRate;
    rate_e resultRate;

    if (pCtrlData->tsrsParameters[ac].supportedRatesMask[pCtrlData->currClientRateID] != 0)
    {
        /* From TSRS rates, choose the closest to the basic rate */
        /* ((1 << givenRate) - 1) gives a map of all rates <= givenRate */
        /* Logical AND with TSRS supported rates gives a map of supported rates <= givenRate */
        /* Now just pick the maximal */
        tsRate = pCtrlData->tsrsParameters[ac].supportedRatesMask[pCtrlData->currClientRateID]
                    & ((1 << givenRate) - 1);
        resultRate = calculateMaxSupportedRate(&tsRate);

        if (resultRate == DRV_RATE_INVALID)
        {
            return givenRate;
        }
        else
        {
            return resultRate;
        }
    }
    else
    {
        return givenRate;
    }
}

