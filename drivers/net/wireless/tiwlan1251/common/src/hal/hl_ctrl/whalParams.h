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
 *
 *   MODULE:  whalParams.h
 *   PURPOSE: Holds all the whal parameters
 * 
 ****************************************************************************/

#ifndef _WHAL_PARAMS_H
#define _WHAL_PARAMS_H

#include  "802_11Defs.h"
#include  "whalHwDefs.h"
#include  "whalCtrl_api.h"
#include  "whalBus_Defs.h"
#include  "commonTypes.h"

#define MAX_FRAGMENTATION_THRESHOLD     2312
#define MAX_SSID_SIZE       32

 /* Trace Buffer Size (DebugTraceXfer Buffer)
   Mote : The Trace Buffer Size are in UINT32 (1024 bytes,and 2048 for both buffers) */
#define TRACE_BUFFER_MAX_SIZE 256 


#define DEFAULT_HW_RADIO_CHANNEL        11


/* Constants */

/* RSSI roaming trigger configuration */
/* minimal amount of packets to count in average RSSI before actually reporting trigger */
#define RSSI_DEFAULT_DEPTH  (10) 
/* the weight in % of the new packet relative to the previous average value of RSSI */
#define RSSI_DEFAULT_WEIGHT (10) 
#define RSSI_DEFAULT_THRESHOLD  (-80) 

/* SNR roaming trigger configuration */
/* minimal amount of packets to count in average SNR before actually reporting trigger */
#define SNR_DEFAULT_DEPTH   (10) 
/* the weight in % of the new packet relative to the previous average value of SNR */
#define SNR_DEFAULT_WEIGHT  (10) 
#define SNR_DEFAULT_THRESHOLD   (0) 

/* 'No beacon' roaming trigger configuration */
/* Number of consecutive beacons (or DTIM periods) missed before 
  'Out of Sync' event is raised */
#define OUT_OF_SYNC_DEFAULT_THRESHOLD   (10)
/* IBSS - Number of consecutive beacons (or DTIM periods) missed before 
  'Out of Sync' event is raised */
#define OUT_OF_SYNC_IBSS_THRESHOLD  (200)
/* period of time between 'Out of sync' and 'No beacon' events */
#define NO_BEACON_DEFAULT_TIMEOUT       (100) /* in tu-s*/

/* Consecutive NACK roaming trigger configuration */
#define NO_ACK_DEFAULT_THRESHOLD    (20) 

/* Low Rx rate roaming trigger configuration */
#define LOW_RATE_DEFAULT_THRESHOLD  (2) 

typedef struct
{
    UINT8    rxFilterCommand;
    filter_e rxFilterAction;
    UINT8    rxFilterNumFieldPatterns;
    UINT8    rxFilterLenFieldPatterns;
    UINT8    rxFilterFieldPatterns[MAX_DATA_FILTER_SIZE];

}RxFilterConfig_t;

/* 
 * Dot11 params
 * ------------
 */


typedef struct
{
    UINT16  RtsThreshold;
    UINT8   CtsToSelf;
    rxTimeOut_t rxTimeOut;
    UINT16  FragmentThreshold;
    UINT8   ListenInterval;
    UINT16  Capabilities;
    UINT32  MaxTxMsduLifetime;
    UINT32  MaxRxMsduLifetime;
    UINT8   calibrationChannel2_4;
    UINT8   calibrationChannel5_0;
    UINT16  Aid;
    UINT8   CurrAntenna;
    UINT8   TxAntenna;
    UINT8   RxAntenna;
    UINT8   Hw_TxAntenna;
    UINT8   Hw_RxAntenna;
    UINT16  CwMin;

    UINT8   RateFallback;
    UINT32  RxConfigOption;
    UINT32  RxFilterOption;
	UINT8	BetEnable;             
	UINT8   MaximumConsecutiveET;
    UINT8   TxCompleteThreshold;
    BOOL    WiFiWmmPS;

    /* ARP IP Addr table*/
    UINT32  arp_IP_ver;
    UINT32  isArpIpFilteringEnabled ;
    IpAddress_t arp_IP_addr;

    /*mac addresses filter*/
    UINT8 isMacAddrFilteringnabled ;
    UINT8 numGroupAddrs;
    macAddress_t Group_addr[MAX_MULTICAST_GROUP_ADDRS];

    UINT32  FeatureOptions;
    UINT32  FeatureDataFlowOptions;
    UINT8   SlotTime;
    UINT8   preamble;
    UINT8   RadioBand;
    UINT8   MacClock;
    UINT8   ArmClock;
    UINT8   Enable4x;
    UINT8   CurrPowerSaveState;
    SoftGeminiEnableModes_e SoftGeminiEnable;
    SoftGeminiParam_t SoftGeminiParams;

    UINT8   maxSitesFragCollect;
    UINT8   hwAccessMethod;

    UINT8   FragmentationOnHal;

    UINT32  nullTemplateSize;

    UINT32  beaconTemplateSize;
    

    UINT32  probeRequestTemplateSize;
    UINT32  probeResponseTemplateSize;
    UINT32  PsPollTemplateSize;
    UINT32  qosNullDataTemplateSize;

    BOOL    RxEnergyDetection;
    BOOL    TxEnergyDetection;

    UINT8   PacketDetectionThreshold;
    UINT8   FcsErrThrsh;
    UINT8   UseDeviceErrorInterrupt;
    BOOL    RetryPreemption;

    UINT32  radioType;
    UINT32  minorE2Ver;
    UINT32  majorE2Ver;
    UINT32  bugfixE2Ver;
    BOOL    RxDisableBroadcast; /* this flag indicate if to discards all broadcast frames */
    BOOL    RecoveryEnable;  /* Indicate if the recovery process is enabled */
    BOOL    bJoin;              /* Indicate if the station is joined */
    /* hardware ACI parameters */
    UINT8   ACIMode;
    UINT8   inputCCA;
    UINT8   qualifiedCCA;
    UINT8   stompForRx;
    UINT8   stompForTx;
    UINT8   txCCA;

    UINT8   AntDiversity ;

    acQueuesParams_t acQueuesParams[MAX_NUM_OF_AC];

    /* Parameters for roaming triggers configuration */
    whalCtrl_roamingTriggerCmd_t roamTriggers;

    /* antenna diversity parameters */
    whalCtrl_antennaDiversityOptions_t antennaDiversityOptions;

    /* power control param */
    powerAutho_PowerPolicy_e minPowerLevel;

    BcnBrcOptions_t BcnBrcOptions;

    UINT8           ConsecutivePsPollDeliveryFailureThreshold;

    beaconFilterIETable_t beaconFilterIETable;
    beaconFilterParams_t beaconFilterParams;
    
    queueTrafficParams_t QtrafficParams;
    acQosParams_t acQosParams;

    /*mac preamble*/
    UINT8 earlyWakeUp;

    /* Rx Data Filter */
    BOOL rxFilterDefaultAction;
    filter_e rxFilterDefaultEnable;
    RxFilterConfig_t rxFilterCgf[MAX_DATA_FILTERS];
    
} WlanParams_T;


/* 
 * BssInfo params
 * --------------
 */

typedef struct
{
    UINT8           ReqBssType;
    UINT8           BssType;
    UINT16          BeaconInterval;
    UINT8           DtimInterval; 
    UINT8           RadioChannel;

    UINT8           BssId[MAC_ADDR_SIZE];
    dot11_SSID_t    WlanElm_Ssid;
    
    txRatePolicy_t  TxRateClassParams; /* Policy for recovery   */  
    UINT16          BasicRateSet;   
    UINT16          SupportedRateSet;
    rate_e          txCtrlFrmRateDriverFormat;
    UINT8           txCtrlFrmRate;
    UINT8           txCtrlFrmModulation;
    UINT8           Ctrl; /* Only bit 7 is currently in use , bit 7 indicates if to flash the Tx queues */
    UINT8           txMgmtFrmRate;
    UINT8           txMgmtFrmModulation;
    UINT16          ATimWindow;     /* ATIM window of IBSS*/
                                    /* Note that when ATIM window is zero the*/
                                    /* initiated IBSS does not support powersave*/
    UINT8           DefaultPreamble;/* Specifies the PLCP preamble type used*/
                                    /* 0 for long preamble*/
                                    /* 1 for short preamble*/


    
} BssInfoParams_T;

/* 
 * General params
 * --------------
 */

typedef struct
{
    int     UseTxDataInterrupt;

    UINT8   TraceEnable;
    UINT8   TraceOut;

    UINT32  PbccDynamicEnable;
    UINT32  PbccDynamicInterval;
    UINT32  PbccDynamicIgnoreMcast;
} GenParams_T;


/* 
 * General counters
 * ----------------
 */

typedef struct
{
    UINT32  FcsErrCnt;
} GenCounters_T;


/*
 * HwInfoParams_t - wlan hardware info
 * -----------------------------------
 */
typedef struct
{
    UINT8                   SrcMacAddr[MAC_ADDR_SIZE];
    UINT32                  Pad0;
    UINT32                  Pad1;
    ACXRevision_t           AcxVersion;             /* Fw version (read from the wlan hardware) */
} HwInfo_T;


/*
 * queuesParam_T - Queue params for Quality Of Service
 * ------------------------------------------
 */
typedef struct
{
    queueTrafficParams_t        queues[MAX_NUM_OF_TX_QUEUES];
    BOOL                        isQueueConfigured[MAX_NUM_OF_TX_QUEUES];
} QueuesParam_t;

typedef struct 
{
    acQosParams_t               ac[MAX_NUM_OF_AC];
    BOOL                        isAcConfigured[MAX_NUM_OF_AC];
}AcConfParam_t;

typedef struct
{
    /* Tx Parameters */
    UINT8               TxPowerDbm;
    
    UINT16              txCompleteTimeout;
    UINT8               txCompleteThreshold;

    UINT8               QidToAcTable[MAX_NUM_OF_TX_QUEUES];
    BOOL                AckPolicy[MAX_NUM_OF_AC];

    /* Information Elements */
    acQueuesParams_t    halAcQueueParams[MAX_NUM_OF_AC];
    whaCtrl_acTrafficParams_t   halTrafficParams[MAX_NUM_OF_AC];

} TxParam_t;


/* 
 * Templates params
 * ----------------
 */



typedef struct
{
    TemplateParams_T    Beacon;
    TemplateParams_T    ProbeReq;
    TemplateParams_T    ProbeResp;
    TemplateParams_T    NullData;
    TemplateParams_T    PsPoll;
    TemplateParams_T    QosNullData;
} TemplateListParams_T;

/*
 * ----------------------------------------------------------------
 *                  MAIN PARAMETERS STRUCTURE
 * ----------------------------------------------------------------
 */
typedef struct _WhalParams_T
{
    DmaParams_T             DmaParams;      /* Rx/Tx queue parameters   */
    QueuesParam_t           QueuesParams;   /* Queues params for QOS    */
    AcConfParam_t           AcParams;       /* AC params for QoS        */
    TxParam_t               TxParams;       /* Tx params for QOS        */
    WlanParams_T            WlanParams;     /* Wlan parameters          */
    BssInfoParams_T         BssInfoParams;  /* Bss information          */
    GenParams_T             GenParams;      /* General parameters       */
    HwInfo_T                HwInfoParams;   /* Hw eeprom, hw versions   */
    GenCounters_T           GenCounters;    /* General counters         */
    TemplateListParams_T    TemplateList;   /* templates for recovery   */

    TI_HANDLE hOs;
    TI_HANDLE hReport;

} WhalParams_T;

/*
 * ----------------------------------------------------------------
 *                  WHAL pARAMS OBJECT API
 * ----------------------------------------------------------------
 */
WhalParams_T *whal_params_Create(TI_HANDLE hOs, BOOL TxFlashEnable);
int whal_params_Destroy(WhalParams_T *pWhalParams);
int whal_params_Config (WhalParams_T *pWhalParams, TI_HANDLE hReport);

/* 
 * DmaParams - Rx/Tx queue parameters
 */
DmaParams_T *whal_ParamsGetDmaParams(WhalParams_T *pWhalParams);
int whal_ParamsSetDmaParams(WhalParams_T *pWhalParams);

/* 
 * WlanParams api 
 */

int     whal_ParamsSetRoamingParams(WhalParams_T *pWhalParams) ;
WlanParams_T *whal_ParamsGetWlanParams(WhalParams_T *pWhalParams);
void   whal_ParamsSetFragmentThreshold  (WhalParams_T *pWhalParams, int FragSize);
UINT32 whal_ParamsGetFragmentThreshold  (WhalParams_T *pWhalParams);
UINT8  whal_ParamsIsFragmentOnHal       (WhalParams_T *pWhalParams);
void   whal_ParamsPrintFragmentThreshold(WhalParams_T *pWhalParams);
UINT8  whal_ParamsGetMaxSitesFragCollect(WhalParams_T *pWhalParams);
void   whal_ParamsSetRtsThreshold       (WhalParams_T *pWhalParams, int RtsSize);
void   whal_ParamsSetListenInterval     (WhalParams_T *pWhalParams, UINT8 Val);
UINT8  whal_ParamsGetListenInterval     (WhalParams_T *pWhalParams);
void   whal_ParamsSetRxFilter           (WhalParams_T *pWhalParams, UINT32 RxConfigOption, UINT32 RxFilterOption);
void   whal_ParamsGetRxFilter           (WhalParams_T *pWhalParams, UINT32* pRxConfigOption, UINT32* pRxFilterOption);
void   whal_ParamsSetarpIpAddressesTable(WhalParams_T *pWhalParams, IpAddress_t * IP_addr, IPver_e IP_ver);
void   whal_ParamsGetarpIpAddressesTable(WhalParams_T * pWhalParams, IpAddress_t * IP_addr, IPver_e* pIP_ver);
void   whal_ParamsSetarpIpFilterEnabled(WhalParams_T *pWhalParams, UINT8 isEnabled);
void   whal_ParamsGetarpIpFilterEnabled(WhalParams_T *pWhalParams, UINT8* pisEnabled);
void   whal_ParamsSetGroupAddressesTable(WhalParams_T *pWhalParams, UINT8 isEnabled, UINT8 numGroupAddrs, macAddress_t *Group_addr);
void   whal_ParamsGetGroupAddressesTable(WhalParams_T *pWhalParams, UINT8* pisEnabled, UINT8* pnumGroupAddrs, macAddress_t *Group_addr);
UINT8  whal_ParamsGetCurrAntenna        (WhalParams_T *pWhalParams);
void   whal_ParamsSetPowerSaveState(WhalParams_T *pWhalParams, UINT8 CurrPowerSaveState);
UINT8  whal_ParamsGetPowerSaveState(WhalParams_T *pWhalParams);


/* 
 * Bss Info Params api 
 */
BssInfoParams_T *whal_ParamsGetBssInfoParams(WhalParams_T *pWhalParams);
UINT8   *whal_ParamsGetBssId        (WhalParams_T *pWhalParams);
void     whal_ParamsSetBssId        (WhalParams_T *pWhalParams, char *BssId);
void     whal_ParamsSetSsid         (WhalParams_T *pWhalParams, char *Ssid, UINT8 SsidLength);
dot11_SSID_t *whal_ParamsGetElm_Ssid(WhalParams_T *pWhalParams);
void   whal_ParamsSetReqBssType     (WhalParams_T *pWhalParams, int Val);
UINT8  whal_ParamsGetReqBssType(WhalParams_T *pWhalParams);
void   whal_ParamsSetBssType        (WhalParams_T *pWhalParams, int Val);
void whal_ParamsSetRadioBand(WhalParams_T *pWhalParams, int RadioBand);
UINT8  whal_ParamsGetRadioBand(WhalParams_T *pWhalParams);
UINT8  whal_ParamsGetBssType        (WhalParams_T *pWhalParams);
void   whal_ParamsSetBeaconInterval (WhalParams_T *pWhalParams, UINT16 Val);
UINT16 whal_ParamsGetBeaconInterval (WhalParams_T *pWhalParams);
void   whal_ParamsSetDtimCount      (WhalParams_T *pWhalParams, UINT8 Val);
UINT8  whal_ParamsGetDtimCount      (WhalParams_T *pWhalParams);
UINT8  whal_ParamsGetRadioChannel   (WhalParams_T *pWhalParams);
void   whal_ParamsSetRadioChannel   (WhalParams_T *pWhalParams, int Channel);
void   whal_ParamsSetBasicRatesSet  (WhalParams_T *pWhalParams, UINT16 BasicRateSet);
void   whal_ParamsSetSupportedRatesSet(WhalParams_T *pWhalParams, UINT16 SupportedRateSet);
void   whal_ParamsSetHwGenTxParams  (WhalParams_T *pWhalParams, rate_e TxRate, BOOL bCtrlFrame);
UINT8  whal_ParamsGetDefaultChannel (WhalParams_T *pWhalParams);
void whal_ParamsSetTxRateClassParams(WhalParams_T *pWhalParams,txRatePolicy_t *pTxRatePolicy);
txRatePolicy_t* whal_ParamsGetTxRateClassParams(WhalParams_T *pWhalParams);
void   whal_ParamsSetAtimWindow     (WhalParams_T *pWhalParams, UINT16 ATimWindow);
UINT16 whal_ParamsGetAtimWindow     (WhalParams_T *pWhalParams);
void   whal_ParamsSetDefaultPreamble(WhalParams_T *pWhalParams, UINT8 DefaultPreamble);
UINT8  whal_ParamsGetDefaultPreamble(WhalParams_T *pWhalParams);
UINT32 whal_ParamsGetTraceBufferSize(WhalParams_T *pWhalParams);
void   whal_ParamsGetMacPreambleParams(WhalParams_T *pWhalParams, UINT8* earlyWakeUp);


/* 
 * general AP parameters 
 */
GenParams_T *whal_ParamsGetGenParams    (WhalParams_T *pWhalParams);
UINT32 whal_ParamsGetPbccDynamicEnableVal(WhalParams_T *pWhalParams);
void   whal_ParamsSetPbccDynamicEnableVal(WhalParams_T *pWhalParams, int Val);

/* 
 * Wlan hardware info params 
 */
UINT8   *whal_ParamsGetSrcMac   (WhalParams_T *pWhalParams);
void     whal_ParamsSetSrcMac   (WhalParams_T *pWhalParams, char *SrcMac);
char whal_ParamsGetRadioType    (WhalParams_T *pWhalParams);
void whal_ParamsHwNvramPrint    (WhalParams_T *pWhalParams);
ACXRevision_t *whal_ParamsGetAcxVersion(WhalParams_T *pWhalParams);
void whal_ParamsPrintFwVersion  (WhalParams_T *pWhalParams);
UINT8 *whal_ParamsGetFwVersion  (WhalParams_T *pWhalParams);
UINT8  whal_ParamsGetE2MajorVersion (WhalParams_T *pWhalParams);
UINT8  whal_ParamsGetE2MinorVersion (WhalParams_T *pWhalParams);
UINT8  whal_ParamsGetE2LastVersion (WhalParams_T *pWhalParams);
UINT8  whal_ParamsGetTiInternalVer (WhalParams_T *pWhalParams);

/*
 * ----------------------------------------------------------------
 *                          WME TX PARAMETERS
 * ----------------------------------------------------------------
 */

int whal_ParamsSetQueueParams(WhalParams_T *pWhalParams,queueTrafficParams_t *pQtrafficParams) ;
void whal_ParamsSetAcParams(WhalParams_T *pWhalParams,acQosParams_t *pAcQosParams);


TxParam_t* whal_ParamsGetTxParams (WhalParams_T *pWhalParams);

/*
 *  Traffic Parameters: (IE ACX_TID_CFG)
 */
int whal_ParamsSetTrafficParams(WhalParams_T *pWhalParams,whaCtrl_acTrafficParams_t* pTconfParams) ;
whaCtrl_acTrafficParams_t* whal_ParamsGetTrafficParams(WhalParams_T *pWhalParams, UINT8 AcID);

/*
 *  Access Category Parameters :(IE ACX_AC_CFG)
 */
int whal_ParamsSetAccessCategoryParams(WhalParams_T *pWhalParams,acQueuesParams_t* pTconfParams) ;

/*
 *  Qos Type : use in the Tx Descriptor 
 */
BOOL whal_ParamsGetQosMode(WhalParams_T *pWhalParams);
int   whal_ParamsSetQosMode(WhalParams_T *pWhalParams,qosProtocols_e QosType);

/*
 *  Ack Policy Parameters: use in the Tx Descriptor 
 */
int   whal_ParamsSetAccessCategoryAckPolicy(WhalParams_T *pWhalParams,BOOL AckPolicy, UINT8 Qid);

/*
 *  
 */
UINT8 whal_ParamsGetAcIdFromQid(WhalParams_T *pWhalParams,UINT8 Qid);

#endif /*_WHAL_PARAMS_H*/
