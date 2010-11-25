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
 *   MODULE:  whalHwMboxConfig.h
 *   PURPOSE: wlan hardware configuration information elements
 * 
 ****************************************************************************/

#ifndef _WHAl_HW_MBOX_CONFIG_H
#define _WHAl_HW_MBOX_CONFIG_H

#include "whalCommon.h"
#include "whalHwDefs.h"
#include "whalCtrl_api.h"


#define MAX_CONFIG_TX_QUEUES 5
#define MAX_CONFIG_RX_QUEUES 5


typedef struct
{
	UINT32 HwQueueAddr;
	UINT8  HwQueueAttr;
} QueueParams_T;


typedef struct
{
	UINT32 BaseAddr;
	UINT32 TxQueuesAddr;
	UINT32 TxQueuesSize;
	UINT32 RxQueuesAddr;
	UINT32 RxQueuesSize;
	UINT32 QindicatorAddr;
	UINT32 QindicatorSize;
	UINT32 NumTxQueues;
	UINT32 NumRxQueues;
	QueueParams_T TxQueuesParams[MAX_CONFIG_TX_QUEUES];
	QueueParams_T RxQueuesParams[MAX_CONFIG_RX_QUEUES];
} MemQueueParams_T;


typedef struct _HwMboxConfig_T
{
    TI_HANDLE           hCmdMboxQueue;
    TI_HANDLE           hOs;
    TI_HANDLE           hReport;
    ACXStatistics_t     pAcxStatistic;
    MemoryMap_t         MemMap;
} HwMboxConfig_T;


HwMboxConfig_T* whal_hwMboxConfig_Create (TI_HANDLE hOs);
int  whal_hwMboxConfig_Destroy               (HwMboxConfig_T *pHwMboxConfig);
int  whal_hwMboxConfig_Config                (HwMboxConfig_T *pHwMboxConfig, TI_HANDLE hCmdMboxQueue, TI_HANDLE hReport);
int  whal_hwInfoElemMemoryMapSet             (HwMboxConfig_T *pHwMboxConfig, MemoryMap_t *apMap);
int  whal_hwInfoElemMemoryMapGet             (HwMboxConfig_T *pHwMboxConfig, MemoryMap_t *apMap, void *fCb, TI_HANDLE hCb);
void whal_hwInfoElemMemoryMapPrint           (HwMboxConfig_T *pHwMboxConfig);
int  whal_hwInfoElemRxConfigSet              (HwMboxConfig_T *pHwMboxConfig, UINT32* apRxConfigOption, UINT32* apRxFilterOption);
int whal_hwInfoElemBETSet(HwMboxConfig_T* pHwMboxConfig, UINT8 Enable, UINT8 MaximumConsecutiveET);
int  whal_hwInfoElemSetRxDataFiltersParams   (HwMboxConfig_T *pHwMboxConfig, BOOL enabled, filter_e defaultAction);
int  whal_hwInfoElemSetRxDataFilter          (HwMboxConfig_T *pHwMboxConfig, UINT8 index, UINT8 command, filter_e action, UINT8 numFieldPatterns, UINT8 lenFieldPatterns, UINT8 * fieldPatterns);
int  whal_hwInfoElemGetRxDataFiltersStatistics(HwMboxConfig_T * pHwMboxConfig, void * CB_Func, TI_HANDLE CB_handle, void * CB_Buf);
int  whal_hwInfoElemStationIdSet             (HwMboxConfig_T *pHwMboxConfig, UINT8* apStationId);
int  whal_hwInfoElemStationIdGet             (HwMboxConfig_T *pHwMboxConfig, void *fCb, TI_HANDLE hCb, void *pCb);
int  whal_hwInfoElemConfigOptionsRead        (HwMboxConfig_T *pHwMboxConfig, void* pElm);
int  whal_hwInfoElemAcxRevisionGet           (HwMboxConfig_T *pHwMboxConfig, void *fCb, TI_HANDLE hCb, void *pCb);
int  whal_hwInfoElemWepDefaultKeyIdSet       (HwMboxConfig_T *pHwMboxConfig, UINT8* apKeyVal, void *CB_Func, TI_HANDLE CB_handle);
int  whal_hwInfoElemWepDefaultKeyIdGet       (HwMboxConfig_T *pHwMboxConfig, UINT8* apKeyVal, void *CB_Func, TI_HANDLE CB_handle);
int  whal_hwInfoElemStatisticsPrint          (HwMboxConfig_T *pHwMboxConfig);
int  whal_hwInfoElemFcsErrorCntGet           (HwMboxConfig_T *pHwMboxConfig, UINT32* pFcsErrCnt);
int  whal_hwInfoElemAcxPMConfigSet           (HwMboxConfig_T *pHwMboxConfig, ACXConfigPM_t* pWlanElm_PowerMgmtOptions);                                     
int  whal_hwInfoElemAcxSleepAuthoSet         (HwMboxConfig_T *pHwMboxConfig, ACXSleepAuth_t* pWlanElm_SleepAutho);                                     
int  whal_hwInfoElemAcxwakeUpConditionSet    (HwMboxConfig_T *pHwMboxConfig, WakeUpCondition_t* pWlanElm_wakeUpCondition);                                   
int  whal_hwInfoElemFeatureConfigSet         (HwMboxConfig_T *pHwMboxConfig, UINT32 Options, UINT32 DataFlowOptions);
int  whal_hwInfoElemAcxBeaconFilterOptionsSet(HwMboxConfig_T *pHwMboxConfig, ACXBeaconFilterOptions_t* pWlanElm_BeaconFilterOptions);                                          
int  whal_hwInfoElemAcxBeaconFilterIETableSet(HwMboxConfig_T *pHwMboxConfig, UINT8* numberOfIEs, UINT8* IETable, UINT8* IETableSize);
int  whal_hwInfoElemarpIpAddressesTableSet   (HwMboxConfig_T *pHwMboxConfig, IpAddress_t *IP_addr, UINT32 isFilteringEnabled); 
int  whal_hwInfoElemGroupAdressesTableSet    (HwMboxConfig_T *pHwMboxConfig, UINT8* numGroupAddrs, macAddress_t *Group_addr, UINT8* isEnabled);
int  whal_hwInfoElemAidSet                   (HwMboxConfig_T *pHwMboxConfig, UINT16* apAidVal);
int  whal_hwInfoElemTxPowerSet				 (HwMboxConfig_T *pHwMboxConfig, UINT8 *TxPowerDbm);
int  whal_hwInfoElemNoiseHistogramResultsGet (HwMboxConfig_T *pHwMboxConfig, interogateCmdCBParams_t noiseHistCBParams);
int  whal_hwInfoElemPowerLevelTableGet		 (HwMboxConfig_T *pHwMboxConfig, interogateCmdCBParams_t powerLevelCBParams);
int  whal_hwInfoElemStationIdForRecoveryGet  (HwMboxConfig_T *pHwMboxConfig, void *CB_Func,TI_HANDLE CB_handle, dot11StationIDStruct *CB_Buf);
int  whal_hwInfoElemSoftGeminiEnableSet      (HwMboxConfig_T *pHwMboxConfig, SoftGeminiEnableModes_e SoftGeminiEnableModes);
int  whal_hwInfoElemSoftGeminiParamsSet      (HwMboxConfig_T *pHwMboxConfig, SoftGeminiParam_t *SoftGeminiParam);
int  whal_hwInfoElemSoftGeminiParamsGet      (HwMboxConfig_T *pHwMboxConfig, void *CB_Func, TI_HANDLE CB_handle, void* CB_Buf);
int  whal_hwInfoElemAcxLowSNRThresholdSet    (HwMboxConfig_T *pHwMboxConfig, ACXLowSNRTriggerParameters_t* AcxElm_LowThresholdOptions);
int  whal_hwInfoElemAcxLowRSSIThresholdSet   (HwMboxConfig_T *pHwMboxConfig, ACXLowRSSITriggerParameters_t* pWlanElm_LowRSSIThresholdOptions);
int  whal_hwInfoElemAcxGetAverageRSSIGet     (HwMboxConfig_T *pHwMboxConfig, INT8* averageRSSI);
int  whal_hwInfoElemAcxBssLossTsfThresholdSet(HwMboxConfig_T *pHwMboxConfig, AcxConnectionMonitorOptions* pWlanElm_BssLossTsfSynchronize);
int  whal_hwInfoElemMiscTableSet             (HwMboxConfig_T *pHwMboxConfig, ACXMisc_t *pCfg);
int  whal_hwInfoElemMiscTableGet             (HwMboxConfig_T *pHwMboxConfig, ACXMisc_t *pCfg, void *fCb, TI_HANDLE hCb);
int  whal_hwInfoElemConfigMemorySet          (HwMboxConfig_T *pHwMboxConfig, DmaParams_T *pDmaParams);
int  whal_hwInfoElemQueueHeadGet             (HwMboxConfig_T *pHwMboxConfig, int NumTxQueues, int pElem, void *fCb, TI_HANDLE hCb);
int  whal_hwInfoElemSlotTimeSet              (HwMboxConfig_T *pHwMboxConfig, UINT8* apSlotTime);
int  whal_hwInfoElemPreambleSet              (HwMboxConfig_T *pHwMboxConfig, UINT8* apPreamble);
int  whal_hwInfoElemGeneratedFrameRateSet    (HwMboxConfig_T *pHwMboxConfig, UINT8* txCtrlFrmRate, UINT8* txCtrlFrmMod, UINT8* txMgmtFrmRate, UINT8* txMgmtFrmMod);
int  whal_hwInfoElemAntennaDiversitySet      (HwMboxConfig_T *pHwMboxConfig, whalCtrl_antennaDiversityOptions_t* pAntennaDiversityOptions, UINT32 antNum );
int  whal_hwInfoElemAcxStatisiticsGet        (HwMboxConfig_T *pHwMboxConfig, acxStatisitcs_t *acxStatisitcs);
int  whal_hwInfoElemAcxReadGwsiStatisiticsGet(HwMboxConfig_T *pHwMboxConfig, void * CB_Func, TI_HANDLE CB_handle, void * CB_Buf);
int  whal_hwInfoElemAcxReadGwsiCountersGet   (HwMboxConfig_T *pHwMboxConfig, void * CB_Func, TI_HANDLE CB_handle, void * CB_Buf);
int  whal_hwInfoElemAcxStatisiticsSet        (HwMboxConfig_T *pHwMboxConfig);
int  whal_hwInfoElemMediumOccupancyGet       (HwMboxConfig_T *pHwMboxConfig, interogateCmdCBParams_t  interogateCmdCBParams);
int  whal_hwInfoElemTfsDtimGet               (HwMboxConfig_T *pHwMboxConfig, interogateCmdCBParams_t  interogateCmdCBParams);
int  whal_hwInfoElemEventMaskSet             (HwMboxConfig_T *pHwMboxConfig, UINT32 MaskVector);
int  whal_hwInfoElemPacketDetectionThresholdSet
                                             (HwMboxConfig_T *pHwMboxConfig, UINT32* pPdThreshold);
int  whal_hwInfoElemCcaThresholdSet          (HwMboxConfig_T *pHwMboxConfig, UINT16* ccaThreshold, BOOL bTxEnergyDetection);
int  whal_hwInfoElemDtimPeriodSet            (HwMboxConfig_T *pHwMboxConfig, UINT8* dtimPeriod, UINT16*TBTT);
int  whal_hwInfoElemDtimPeriodGet            (HwMboxConfig_T *pHwMboxConfig, UINT8* dtimPeriod, UINT16*TBTT);
int  whal_hwInfoElemTxRatePolicyConfigurationSet
                                             (HwMboxConfig_T *pHwMboxConfig, txRatePolicy_t *pTxRatePolicy);                                        
int  whal_hwInfoElemACIConfigurationSet      (HwMboxConfig_T *pHwMboxConfig, UINT8 ACIMode, UINT8 inputCCA, UINT8 qualifiedCCA, UINT8 stompForRx, UINT8 stompForTx, UINT8 txCCA);
int  whal_hwInfoElemRSSIGet                  (HwMboxConfig_T *pHwMboxConfig, void *CB_Func, TI_HANDLE CB_handle, PUINT8 CB_Buf);
int  whal_hwInfoElemTxConfigOptionsSet       (HwMboxConfig_T *pHwMboxConfig, UINT8 txCompleteThreshold, UINT16 txCompleteTimeout);
int  whal_hwInfoElemRtsThresholdSet          (HwMboxConfig_T *pHwMboxConfig, UINT16 RtsThreshold);
int  whal_hwInfoElemCtsToSelfSet             (HwMboxConfig_T *pHwMboxConfig, UINT8 CtsToSelf);
int  whal_hwInfoElemAcxBcnBrcOptionsSet      (HwMboxConfig_T *pHwMboxConfig, ACXBeaconAndBroadcastOptions_t* pWlanElm_BcnBrcOptions);
int  whal_hwInfoElemAcxBcnBrcOptionsGet      (HwMboxConfig_T *pHwMboxConfig, ACXBeaconAndBroadcastOptions_t* pWlanElm_BcnBrcOptions);
int  whal_hwInfoElemWiFiWmmPSWASet           (HwMboxConfig_T *pHwMboxConfig, BOOL enableWA);


int whalCtrl_getConsumptionStatistics        (HwMboxConfig_T * pHwMboxConfig, void * CB_Func, TI_HANDLE CB_handle, void * CB_Buf);


/*
 *	Data Path Configure API
 */
int  whal_hwInfoElemDataPathParamsSet        (HwMboxConfig_T *pHwMboxConfig, UINT16 rxPacketRingChunkSize, UINT16 txPacketRingChunkSize, UINT8 rxPacketRingChunkNum, UINT8 txPacketRingChunkNum, UINT8 txCompleteThreshold, UINT8 txCompleteRingDepth,UINT32 txCompleteTimeOut);
int  whal_hwInfoElemDataPathParamsGet        (HwMboxConfig_T *pHwMboxConfig, ACXDataPathParamsResp_t* apDataPathParams, void *fCb, TI_HANDLE hCb);
int  whal_hwInfoElemQueueConfigurationSet    (HwMboxConfig_T *pHwMboxConfig, queueTrafficParams_t *pQtrafficParams);
int  whal_hwInfoElemAcParamsConfigurationSet (HwMboxConfig_T *pHwMboxConfig, configureCmdCBParams_t *pConfigureCommand);
int  whal_hwInfoElemAcParamsConfigurationGet (HwMboxConfig_T *pHwMboxConfig, configureCmdCBParams_t *pConfigureCommand);
int  whal_hwInfoElemAcxSetMaxTxRetrySet      (HwMboxConfig_T *pHwMboxConfig, ACXConsTxFailureTriggerParameters_t* pWlanElm_SetMaxTxRetry);
int  whal_hwInfoElemTxQueueCfgSet            (HwMboxConfig_T *pHwMboxConfig, acQueuesParams_t* pAcQueuesParams, UINT32 numOfTxBlk);
int  whal_hwInfoElemRxTimeOutSet             (HwMboxConfig_T *pHwMboxConfig, rxTimeOut_t* pRxTimeOut);
int  whal_hwInfoElemRxMsduLifeTimeSet        (HwMboxConfig_T *pHwMboxConfig, UINT32 RxMsduLifeTime);
                         

#endif

