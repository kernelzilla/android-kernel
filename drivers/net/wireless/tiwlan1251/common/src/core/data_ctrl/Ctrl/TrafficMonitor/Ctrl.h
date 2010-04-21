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
/*    MODULE:   tx.h                                                       */
/*    PURPOSE:  Tx module Header file                                      */
/*                                                                         */
/***************************************************************************/
#ifndef _CTRL_H_
#define _CTRL_H_

#include "osTIType.h"
#include "paramIn.h"
#include "paramOut.h"
#include "rxXfer_api.h"
#include "RateAdaptation.h"
#include "Clsfr.h"
#include "fourX.h"

#define     DEF_CURRENT_PREAMBLE                        PREAMBLE_LONG
#define     DEF_CURRENT_MUDULATION_TYPE                 DRV_MODULATION_CCK
#define     DEF_BASIC_RATE                              DRV_RATE_2M
#define     DEF_BASIC_RATE_MASK                         0x0003
#define     ALL_RATES_AVAILABLE                         0xFFFFFFFF                      

#define     DEF_RATE_CONTROL_ENABLE                     FALSE
#define     DEF_START_STOP_LINK_CTRL_ALG                FALSE

typedef enum
{
CTRL_DATA_TRAFFIC_INTENSITY_HIGH_CROSSED_ABOVE,
CTRL_DATA_TRAFFIC_INTENSITY_HIGH_CROSSED_BELOW,
CTRL_DATA_TRAFFIC_INTENSITY_LOW_CROSSED_ABOVE,
CTRL_DATA_TRAFFIC_INTENSITY_LOW_CROSSED_BELOW,
CTRL_DATA_TRAFFIC_INTENSITY_MAX_EVENTS,
} ctrlData_trafficIntensityEvents_e;


#define TS_EXCEEDS(currTime,expTime) (currTime > expTime)
#define TS_ADVANCE(currTime,expTime,delta) (expTime = currTime + (delta))

typedef struct 
{
    UINT32 dbgNumOfMsduFreeInTxComplete[MAX_NUM_OF_TX_QUEUES];
}ctrlDataDbgCounters_t;

typedef struct 
{
    UINT32 supportedRatesMask[NUM_OF_RATE_CLASS_CLIENTS];
    UINT32 policyClassRateMask[NUM_OF_RATE_CLASS_CLIENTS];
    UINT32 fwPolicyID[NUM_OF_RATE_CLASS_CLIENTS];
}tsrsParameters_t;

typedef struct
{
    TI_HANDLE           hSiteMgr;
    TI_HANDLE           hTxData;
    TI_HANDLE           hRxData;
    TI_HANDLE           hWhalCtrl;  
    TI_HANDLE           hOs;
    TI_HANDLE           hReport;
    TI_HANDLE           hAPConn;
    TI_HANDLE           hEvHandler;
    TI_HANDLE           hTrafficMonitor;
    TI_HANDLE           hMemMngr;
    

    rateAdaptation_t*   pRateAdaptation;
#ifdef SUPPORT_4X
    fourX_t*            pFourX;
#endif
    classifier_t*       pClsfr;

    BOOL                ctrlDataRateControlEnable;
    BOOL                ctrlDataFourXEnable;    
    BOOL                ctrlDataCerruentFourXstate; 
    BOOL                ctrlDataStartStoplinkControlAlg;

    macAddress_t        ctrlDataCurrentBSSID; 
    bssType_e           ctrlDataCurrentBssType; 
    UINT32              ctrlDataBasicRateBitMask;
    UINT32              ctrlDataCurrentRateMask;
    modulationType_e    ctrlDataCurrentModulationType;
    rate_e              ctrlDataCurrentBasicRate;
    modulationType_e    ctrlDataCurrentBasicModulationType; 
    preamble_e          ctrlDataCurrentPreambleType; 
    macAddress_t        ctrlDataDeviceMacAddress; 

    BOOL                ctrlDataProtectionEnabled;
    RtsCtsStatus_e      ctrlDataRtsCtsStatus;
    erpProtectionType_e ctrlDataIbssProtectionType;

    /* rate adaptation tables */
    rateTables_t        ctrlDataRateTables;
    ctrlData_rateAdapt_t* ctrlDataCurrentRateTable;

    /* Control module counters */
    ctrlDataCounters_t  ctrlDataCounters;
    /*
     * txRatePolicy section
     */

    /* txRatePolicies - here we store the policy and set it to the FW */
    txRatePolicy_t      ctrlDataTxRatePolicy;

    /* Client supported rates - currently User (all supported) or SG (configured) */
    UINT32              currClientRateMask[NUM_OF_RATE_CLASS_CLIENTS];

    /* changing supported rates is done in the next array. We use the change only
        on connection - we copy currClientRateMask = nextClientRateMask */
    UINT32              nextClientRateMask[NUM_OF_RATE_CLASS_CLIENTS];

    BOOL                bIsClassAvailable[NUM_OF_RATE_CLASS_CLIENTS];

    /* the class ID to use for Tx Data Packets */
    rateClassClients_e  currClientRateID;
    /* Saves the last wanted configuration of the Client ID, used when in a connection we get
        0 supported rates , but in the next connection we get supported rates, thus we use the last one configured */
    rateClassClients_e  configuredClientRateID;
    
    /* number of retries for each rate in each class in the policy that we set to the FW */ 
    policyClassRatesArray_t policyClassRatesArrayCck  [NUM_OF_RATE_CLASS_CLIENTS];
    policyClassRatesArray_t policyClassRatesArrayPbcc [NUM_OF_RATE_CLASS_CLIENTS];
    policyClassRatesArray_t policyClassRatesArrayOfdm [NUM_OF_RATE_CLASS_CLIENTS];
    policyClassRatesArray_t policyClassRatesArrayOfdmA[NUM_OF_RATE_CLASS_CLIENTS];

    /* holds the current used array */
    policyClassRatesArray_t *pCurrPolicyClassRatesArray;
    /* debug counter */
    ctrlDataDbgCounters_t ctrlDataDbgCounters;

    /* Callback for disassociation notification */
    disassocSentCB_t    disassocSentCBFunc;
    TI_HANDLE           disassocSentCBObj;

    /* Flag to indicate whether traffic intensity events should be sent or not */
    BOOL                ctrlDataTrafficIntensityEventsEnabled;
    OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS ctrlDataTrafficIntensityThresholds;
    TI_HANDLE ctrlDataTrafficThresholdEvents[CTRL_DATA_TRAFFIC_INTENSITY_MAX_EVENTS];

    tsrsParameters_t    tsrsParameters[MAX_NUM_OF_AC];

    /* holds last fragmentation threshold */
    UINT16              lastFragmentThreshold;

} ctrlData_t;


#endif
