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
 *   MODULE:  whalHwCtrl.h
 *   PURPOSE: Implements action on the Acx card (Reset, Run, SendCmd, Sw Download)
 *
 ****************************************************************************/

#ifndef _WHAL_HW_CTRL_H
#define _WHAL_HW_CTRL_H

#include "whalCommon.h"
#include "whalHwMboxCmd.h"
#include "whalHwMboxCmdBit.h"
#include "whalHwMboxConfig.h"
#include "whalBus_Api.h"
#include "whalCtrl.h"
#include "whalCtrl_api.h"
#include "whalParams.h"
#include "CmdQueue_api.h"
#include "commonTypes.h"


typedef struct
{  
    /* Mbox */
    UINT32	NumMboxFailures;
    UINT32	NumMboxErrDueToPeriodicBuiltInTestCheck;

} whalCtrl_hwStatus_t;
                      
typedef struct _HwCtrl_T
{
    MemoryMap_t             MemMap;
    ACXDataPathParamsResp_t DataPathParams;

    HwMboxConfig_T          *pHwMboxConfig;
    HwMboxCmd_T             *pHwMboxCmd;
    HwMboxCmdBit_T          *pHwMboxCmdBit;
    struct WHAL_CTRL        *pWhalCtrl;
    WhalParams_T            *pWhalParams;
    whalCtrl_hwStatus_t     HwStatus;

    UINT16                  SecuritySeqNumLow;
    UINT32                  SecuritySeqNumHigh;

    TI_HANDLE               *hHwRadio;

    TI_HANDLE               hWhalBus;
    TI_HANDLE               hOs;
    TI_HANDLE               hReport;
    TI_HANDLE               hWhalCtrl;
    TI_HANDLE               hEventMbox;

    UINT32                  uFwBuf;
    UINT32                  uFwAddr; 
    UINT32                  uEEEPROMBuf;
    UINT32                  uEEEPROMLen; 
 
    void                    *fCb;
    TI_HANDLE               hCb;

    dot11StationIDStruct    mbox;

} HwCtrl_T;


TI_STATUS  whal_hwCtrl_FinalizeDownload  (TI_HANDLE hHwCtrl, BootAttr_T *pBootAttr);
TI_STATUS  whal_hwCtrl_FinalizeOnFailure (TI_HANDLE hHwCtrl);

extern HwCtrl_T *whal_hwCtrl_Create(TI_HANDLE hOs, WhalParams_T *pWhalParams);
extern int whal_hwCtrl_Destroy				(HwCtrl_T *pHwCtrl);
extern TI_STATUS whal_hwCtrl_Config(HwCtrl_T *pHwCtrl, TI_HANDLE hWhalCtrl,UINT8 AccessMode, UINT32 AcxRegAddr, 
                                    UINT32 AcxMemAddr, TI_HANDLE hReport, TI_HANDLE hMemMgr,UINT32 *pFWImage,
                                    TI_HANDLE hEventMbox);
extern TI_HANDLE whal_hwCtrl_GetTnentwifHandle (HwCtrl_T *pHwCtrl);
extern int whal_hwCtrl_ConfigHw				(HwCtrl_T *pHwCtrl, void *fCb, TI_HANDLE hCb, BOOL bRecovery);
extern TI_STATUS whal_hwCtrl_Initiate       (HwCtrl_T *pHwCtrl);
extern int whal_hwCtrl_SetMacAddress		(HwCtrl_T *pHwCtrl, macAddress_t *macAddr);
extern int whal_hwCtrl_StartJoin			(HwCtrl_T *pHwCtrl, bssType_e BssType, void *JoinCompleteCB, TI_HANDLE CB_handle);
extern int whal_hwCtrl_SetBssType			(HwCtrl_T *pHwCtrl, bssType_e BssType, UINT8 *HwBssType);
extern int whal_hwCtrl_setRxFilters			(HwCtrl_T *pHwCtrl, UINT32 RxConfigOption, UINT32 RxFilterOption);
extern int whal_hwCtrl_GetRxFilters			(HwCtrl_T *pHwCtrl, UINT32* pRxConfigOption, UINT32* pRxFilterOption);
extern int whal_hwCtrl_setRxDataFiltersParams(HwCtrl_T * pHwCtrl, BOOL enabled, filter_e defaultAction);
extern int whal_hwCtrl_setRxDataFilter      (HwCtrl_T *pHwCtrl, UINT8 index, UINT8 command, filter_e action, UINT8 numFieldPatterns, UINT8 lenFieldPatterns, UINT8 * fieldPatterns);
extern int whal_hwCtrl_StartScan			(HwCtrl_T *pHwCtrl, ScanParameters_t* pScanVals,void* ScanCommandResponseCB, TI_HANDLE CB_handle);
extern int whal_hwCtrl_StartSPSScan         (HwCtrl_T *pHwCtrl, ScheduledScanParameters_t* pScanVals, void* ScanCommandResponseCB, TI_HANDLE CB_handle);
extern int whal_hwCtrl_StopScan				(HwCtrl_T *pHwCtrl, void *ScanCommandResponseCB, TI_HANDLE CB_handle);
extern int whal_hwCtrl_StopSPSScan          (HwCtrl_T *pHwCtrl, void* ScanCommandResponseCB, TI_HANDLE CB_handle);
extern int whal_hwCtrl_GenCmd				(HwCtrl_T *pHwCtrl, short CmdID, char* pBuf, UINT32 Length);
extern int whal_hwCtrl_EnableDataPath       (HwCtrl_T *pHwCtrl);
extern int whal_hwCtrl_EnableTx				(HwCtrl_T *pHwCtrl, int channel);
extern int whal_hwCtrl_DisableTx			(HwCtrl_T *pHwCtrl);
extern int whal_hwCtrl_DisableDataPath      (HwCtrl_T *pHwCtrl);
extern void whal_hwCtrl_Reset		        (HwCtrl_T *pHwCtrl);
extern int whal_hwCtrl_PMConfig				(HwCtrl_T *pHwCtrl, whalCtrl_powerMgmtConfig_t *pPMConfig);
extern int whal_hwCtrl_BcnBrcOptions		(HwCtrl_T *pHwCtrl, whalCtrl_powerMgmtConfig_t *pPMConfig);
extern int whal_hwCtrl_wakeUpCondition		(HwCtrl_T *pHwCtrl, whalCtrl_powerMgmtConfig_t *pPMConfig);
extern int whal_hwCtrl_SetEnergyDetection	(HwCtrl_T *pHwCtrl, BOOL energyDetection);
extern int whal_hwCtrl_WepDefaultKeyAdd     (HwCtrl_T *pHwCtrl, securityKeys_t* aSecurityKey, void *CB_Func, TI_HANDLE CB_handle);
extern int whal_hwCtrl_WepDefaultKeyRemove  (HwCtrl_T *pHwCtrl, securityKeys_t* aSecurityKey, void *CB_Func, TI_HANDLE CB_handle);
extern int whal_hwCtrl_WepMappingKeyAdd     (HwCtrl_T *pHwCtrl, securityKeys_t* aSecurityKey, void *CB_Func, TI_HANDLE CB_handle);
extern int whal_hwCtrl_WepMappingKeyRemove  (HwCtrl_T *pHwCtrl, securityKeys_t* aSecurityKey, void *CB_Func, TI_HANDLE CB_handle);
extern int whal_hwCtrl_AesMappingKeyAdd		(HwCtrl_T *pHwCtrl, securityKeys_t* aSecurityKey, void *CB_Func, TI_HANDLE CB_handle);
extern int whal_hwCtrl_AesMappingKeyRemove  (HwCtrl_T *pHwCtrl, securityKeys_t* aSecurityKey, void *CB_Func, TI_HANDLE CB_handle);
extern int whal_hwCtrl_TkipMicMappingKeyAdd	(HwCtrl_T *pHwCtrl, securityKeys_t* aSecurityKey, void *CB_Func, TI_HANDLE CB_handle);
extern int whal_hwCtrl_TkipMicMappingKeyRemove (HwCtrl_T *pHwCtrl, securityKeys_t* aSecurityKey, void *CB_Func, TI_HANDLE CB_handle);
extern int whal_hwCtrl_TkipKeyEnable        (HwCtrl_T *pHwCtrl, char* MACaddr, BOOL aKey0RxEnable, BOOL aKey0TxEnable);
extern int whal_hwCtrl_DefaultKeyIdSet		(HwCtrl_T *pHwCtrl, UINT8 aKeyIdVal, void *CB_Func, TI_HANDLE CB_handle);
extern int whal_hwCtrl_DefaultKeyIdGet		(HwCtrl_T *pHwCtrl, UINT8 *pKeyIdVal);
extern int whal_hwCtrl_SetBeaconFiltering	(HwCtrl_T *pHwCtrl, UINT8 beaconFilteringStatus, UINT8 numOfBeaconsToBuffer);

extern int whal_hwCtrl_SetBeaconFilterIETable(HwCtrl_T *pHwCtrl, UINT8* numberOfIEs, UINT8 * IETable, UINT8* IETableSize);

extern int whal_hwCtrl_GetGroupAddressesTable (HwCtrl_T *pHwCtrl,UINT8* pisEnabled, UINT8* pnumGroupAddrs, macAddress_t *Group_addr);
extern int whal_hwCtrl_SetGroupAddressesTable (HwCtrl_T *pHwCtrl,UINT8 numGroupAddrs, macAddress_t *Group_addr, UINT8 isEnabled);
extern int whal_hwCtrl_SetRtsThreshold (HwCtrl_T *pHwCtrl,UINT16 RtsThreshold);
extern int whal_hwCtrl_SetarpIpAddressesTable(HwCtrl_T *pHwCtrl, IpAddress_t *IP_addr, UINT8 isEnabled , IPver_e IP_ver);
extern int whalCtrl_GetArpIpAddressesTable (HwCtrl_T *pHwCtrl, IpAddress_t *IP_addr, UINT8* pisEnabled , IPver_e* pIP_ver);
extern int whal_hwCtrl_SetarpIpFilterEnabled(HwCtrl_T *pHwCtrl, UINT8 isEnabled ) ;
extern void whal_hwCtrl_Stop				(HwCtrl_T *pHwCtrl);
extern BOOL whal_hwCtrl_isElpSupported		(HwCtrl_T *pHwCtrl);
extern int whal_hwCtrl_AidSet				(HwCtrl_T *pHwCtrl, UINT16 aAidVal);

extern int whal_hwCtrl_SaveAntennaDiversityOptions (HwCtrl_T *pHwCtrl, 
                                                    whalCtrl_antennaDiversityOptions_t* pAntennaDivresityOptions);
extern int whal_hwCtrl_CurrentAntennaDiversitySendCmd (HwCtrl_T *pHwCtrl);
extern int whal_hwCtrl_SetTxAntenna (HwCtrl_T *pHwCtrl, UINT8 TxAntenna);
extern int whal_hwCtrl_GetTxAntenna (HwCtrl_T *pHwCtrl, UINT8* TxAntenna);
extern int whal_hwCtrl_SetRxAntenna (HwCtrl_T *pHwCtrl, UINT8 RxAntenna);
extern int whal_hwCtrl_GetRxAntenna (HwCtrl_T *pHwCtrl, UINT8* RxAntenna);

extern int whal_hwCtrl_PowerMgmtOptionsPrint(HwCtrl_T *pHwCtrl);
extern int whal_hwCtrl_PowerMgmtConfigurationSet (HwCtrl_T *pHwCtrl,
                                                        whalCtrl_powerSaveParams_t* powerSaveParams);
/*										whalCtrl_powerMgmtConfig_t *thePowerMgmtOptionsConfig);*/
extern int whal_hwCtrl_switchChannel		(HwCtrl_T *pHwCtrl, UINT8 channel);
extern int whal_hwCtrl_SetRegulatoryDomain	(HwCtrl_T *pHwCtrl, radioBand_e radioBand);
extern int whal_hwCtrl_SetACIConfiguration 	(HwCtrl_T *pHwCtrl, UINT8 ACIMode, UINT8 inputCCA, UINT8 qualifiedCCA,
                                            UINT8 stompForRx, UINT8 stompFortTx, UINT8 txCCA);
extern int whal_hwCtrl_CurrentAssociationIdGet (HwCtrl_T *pHwCtrl, UINT16  *pAidVal);

extern int whal_HwCtrl_enableMboxAsyncMode	(HwCtrl_T *pHwCtrl);
extern int whal_HwCtrl_resetMacRx (HwCtrl_T *pHwCtrl);
extern int whal_HwCtrl_LNAControl (HwCtrl_T *pHwCtrl, UINT8 LNAControlField);
extern int whal_hwCtrl_NoiseHistogramCmd (HwCtrl_T *pHwCtrl, whalCtrl_noiseHistogram_t* pNoiseHistParams);
extern int whal_hwCtrl_SwitchChannelCmd (HwCtrl_T *pHwCtrl, whalCtrl_switchChannelCmd_t* pSwitchChannelCmd);
extern int whal_hwCtrl_SwitchChannelCancelCmd (HwCtrl_T *pHwCtrl);

extern int whal_hwCtrl_SetSNRParams(HwCtrl_T *pHwCtrl, whalCtrl_roamingTriggerCmd_t* pRoamingTriggerCmd) ;
extern int whal_hwCtrl_SetRSSIParams(HwCtrl_T *pHwCtrl, whalCtrl_roamingTriggerCmd_t* pRoamingTriggerCmd) ;
extern int whal_hwCtrl_SetMaxTxRetryParams(HwCtrl_T *pHwCtrl, whalCtrl_roamingTriggerCmd_t* pRoamingTriggerCmd);
extern int whal_hwCtrl_SetBssLossTsfThresholdParams(	HwCtrl_T *pHwCtrl,whalCtrl_roamingTriggerCmd_t* pRoamingTriggerCmd);
extern int whal_hwCtrl_GetAsynRSSI (HwCtrl_T *pHwCtrl,void *CB_Func, TI_HANDLE CB_handle, PUINT8 CB_Buf);

extern int whal_hwCtrl_ClkRunEnableSet  (HwCtrl_T *pHwCtrl, BOOL aClkRunEnable);

extern int whal_hwCtrl_ArmClockSet (HwCtrl_T *pHwCtrl, UINT32 ArmClock);
extern int whal_hwCtrl_MacClockSet (HwCtrl_T *pHwCtrl, UINT32 MacClock);

extern TI_HANDLE whal_hwCtrl_GetBusHandle(HwCtrl_T *pHwCtrl);
extern HwMboxConfig_T *whal_hwCtrl_GetMboxConfig(HwCtrl_T *pHwCtrl);
extern HwMboxCmd_T    *whal_hwCtrl_GetMboxCmd(HwCtrl_T *pHwCtrl);

extern int whal_hwCtrl_getTsf(HwCtrl_T *pHwCtrl, UINT32 *pTsf);

extern int whal_hwCtrl_SetSlotTime    	(HwCtrl_T *pHwCtrl, slotTime_e SlotTimeVal);
extern int whal_hwCtrl_SetPreamble	(HwCtrl_T *pHwCtrl, Preamble_e preambleVal);
int whal_hwCtrl_SetFrameRate(HwCtrl_T *pHwCtrl, 
                                UINT8	txCtrlFrmRateVal,
                                UINT8  	txCtrlFrmModVal,
                                UINT8  	txMgmtFrmRateVal,
                                UINT8  	txMgmtFrmModVal);
extern int whal_hwCtrl_EncDecrSet     (HwCtrl_T *pHwCtrl, BOOL aHwEncEnable, BOOL aHwDecEnable);
extern int whal_hwCtrl_RxMsduFormatSet(HwCtrl_T *pHwCtrl, BOOL aRxMsduForamtEnable);
extern int  whal_hwCtrl_PacketDetectionThreshold (TI_HANDLE hHwCtrl, UINT8* pPdThreshold);
extern int whal_hwCtrl_FwDisconnect(HwCtrl_T *pHwCtrl, uint32 ConfigOptions, uint32 FilterOptions);
extern int whal_hwCtrl_MinPowerLevelSet (HwCtrl_T *pHwCtrl, powerAutho_PowerPolicy_e minPowerLevel);

extern int whal_hwCtrl_SoftGeminiEnable(HwCtrl_T *pHwCtrl,SoftGeminiEnableModes_e SgEnable);
extern int whal_hwCtrl_SetSoftGeminiParams(HwCtrl_T *pHwCtrl,SoftGeminiParam_t *SgParam);
int whal_hwCtrl_GetSoftGeminiParams (HwCtrl_T *pHwCtrl, void *CB_Func, TI_HANDLE CB_handle, void* CB_Buf);
extern int whal_hwCtrl_TxRatePolicy(TI_HANDLE hHwCtrl,txRatePolicy_t *pTxRatePolicy);
extern txRatePolicy_t* whal_hwCtrl_GetTxRatePolicy(TI_HANDLE hHwCtrl);
extern int whal_hwCtrl_ReJoinBss (TI_HANDLE hHwCtrl);
int whal_hwCtrl_setBetParams(HwCtrl_T * pHwCtrl, UINT8 Enable, UINT8 MaximumConsecutiveET);

/*
 *	WME TX API
 */
extern int  whal_hwCtrl_TrafficConf(TI_HANDLE hHwCtrl, queueTrafficParams_t *pQtrafficParams);
extern int  whal_hwCtrl_AcParamsConf(TI_HANDLE hHwCtrl,configureCmdCBParams_t *pConfigureCommand);
extern int  whal_hwCtrl_QueueConf(TI_HANDLE hHwCtrl, acQueuesParams_t* pAcQueuesParams);

#if 0
extern int  whal_hwCtrl_TrafficConf       (TI_HANDLE hHwCtrl, whaCtrl_acTrafficParams_t* pAcTrafficParams);
extern int  whal_hwCtrl_TxConfigOptions   (TI_HANDLE hHwCtrl);
#endif

/* Measurement */
extern int whal_hwCtrl_measurement (HwCtrl_T *pHwCtrl, whalCtrl_MeasurementParameters_t* pMeasurementParams,
                             void* MeasureCommandResponseCB, TI_HANDLE CB_handle);
extern int whal_hwCtrl_measurementStop (HwCtrl_T *pHwCtrl, void* MeasureCommandResponseCB, TI_HANDLE CB_handle);
extern int whal_hwCtrl_ApDiscovery (HwCtrl_T *pHwCtrl, whalCtrl_ApDiscoveryParameters_t* pMeasurementParams);
extern int whal_hwCtrl_ApDiscoveryStop (HwCtrl_T *pHwCtrl);
extern int whal_hwCtrl_healthCheck (HwCtrl_T *pHwCtrl);


/*
PowerMgmtOption enum
*/
typedef enum
{
    /* beacon/dtim/listen interval */
    POWER_MGMT_OPTIONS_WAKEUP_ON_ALL_BEACONS_VAL = 0x0,

    POWER_MGMT_OPTIONS_WAKEUP_ON_ALL_DITMS_VAL = 0x1,

    POWER_MGMT_OPTIONS_WAKEUP_ON_EVERY_LISTEN_INTERVAL_VAL = 0x2,

    POWER_MGMT_OPTIONS_WAKEUP_ON_N_DITM_VAL = 0x5,

    POWER_MGMT_OPTIONS_WAKEUP_ON_MASK = 0x7,

    /* gpio/wake on host */
    POWER_MGMT_OPTIONS_WAKEUP_ON_GPIO_OFFSET = 6,

    POWER_MGMT_OPTIONS_WAKEUP_ON_GPIO_VAL = 1<<POWER_MGMT_OPTIONS_WAKEUP_ON_GPIO_OFFSET,

    POWER_MGMT_OPTIONS_WAKEUP_ON_GPIO_MASK = 1<<POWER_MGMT_OPTIONS_WAKEUP_ON_GPIO_OFFSET,

    /* 802.11 ps Enable */
    POWER_MGMT_OPTIONS_PS_802_11_ENABLE_OFFSET = 7,

    POWER_MGMT_OPTIONS_PS_802_11_ENABLE_VAL = 1<<POWER_MGMT_OPTIONS_PS_802_11_ENABLE_OFFSET,

    POWER_MGMT_OPTIONS_PS_802_11_ENABLE_MASK = 1<<POWER_MGMT_OPTIONS_PS_802_11_ENABLE_OFFSET,

    /* 802.11 ps Disable send of null data frame on exit from PS */
    POWER_MGMT_OPTIONS_PS_802_11_DISABLE_NULL_DATA_SEND_ON_EXIT_OFFSET = 5,

    POWER_MGMT_OPTIONS_PS_802_11_DISABLE_NULL_DATA_SEND_ON_EXIT_VAL = 1<<POWER_MGMT_OPTIONS_PS_802_11_DISABLE_NULL_DATA_SEND_ON_EXIT_OFFSET,

    POWER_MGMT_OPTIONS_PS_802_11_DISABLE_NULL_DATA_SEND_ON_EXIT_MASK = 1<<POWER_MGMT_OPTIONS_PS_802_11_DISABLE_NULL_DATA_SEND_ON_EXIT_OFFSET,


    /* Rx Broadcast (ignore broadcast / proxy arp) */
    POWER_MGMT_OPTIONS_RX_BROADCAST_OFFEST = 0,

    POWER_MGMT_OPTIONS_RX_BROADCAST_VAL = 1<<POWER_MGMT_OPTIONS_RX_BROADCAST_OFFEST,

    POWER_MGMT_OPTIONS_RX_BROADCAST_MASK = 1<<POWER_MGMT_OPTIONS_RX_BROADCAST_OFFEST,

    /* HW PS POLL */
    POWER_MGMT_OPTIONS_HW_PS_POLL_OFFSET = 1,

    POWER_MGMT_OPTIONS_HW_PS_POLL_OFF_VAL = 1<<POWER_MGMT_OPTIONS_HW_PS_POLL_OFFSET,

    POWER_MGMT_OPTIONS_HW_PS_POLL_MASK = 1<<POWER_MGMT_OPTIONS_HW_PS_POLL_OFFSET

}whalHwCtrl_PowerMgmtOptionsDefinitions;

/*
BeaconFiltering enum
*/
typedef enum
{
    /* beacon filtering activation */
    BEACON_FILTER_DISABLE_VAL = 0,

    BEACON_FILTER_ENABLE_VAL = 1

}whalHwCtrl_BeaconFilteringOptionsDefinitions;


/* 
 * EEPROM defines
 */
/*******************
 Radio constants
 *******************/
#define RADIO_MAXIM2820_ID    (0x0D)
#define RADIO_RFMD_ID         (0x11)
#define RADIO_RADIA_BG_ID     (0x16)
#define RADIO_RADIA_ABG_ID    (0x17)
#define RADIO_RADIA_BG_CRT_ID (0x19)
#define RADIO_RADIA_CRT_ID    (0x19)
#define RADIO_RADIA_WBR_ID    (0x1A)
#define RADIO_RADIA_DCR_ID    (0x1B)
#define RADIO_RADIA_DCR_1251_ID (0x1C)
#define DEFAULT_RADIO_TYPE	   RADIO_MAXIM2820_ID


#endif

