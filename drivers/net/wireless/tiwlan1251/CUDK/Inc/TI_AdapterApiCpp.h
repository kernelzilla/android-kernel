/*******************************************************************************
**+--------------------------------------------------------------------------+**
**|                                                                          |**
**| Copyright 1998-2008 Texas Instruments, Inc. - http://www.ti.com/         |**
**|                                                                          |**
**| Licensed under the Apache License, Version 2.0 (the "License");          |**
**| you may not use this file except in compliance with the License.         |**
**| You may obtain a copy of the License at                                  |**
**|                                                                          |**
**|     http://www.apache.org/licenses/LICENSE-2.0                           |**
**|                                                                          |**
**| Unless required by applicable law or agreed to in writing, software      |**
**| distributed under the License is distributed on an "AS IS" BASIS,        |**
**| WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. |**
**| See the License for the specific language governing permissions and      |**
**| limitations under the License.                                           |**
**|                                                                          |**
**+--------------------------------------------------------------------------+**
*******************************************************************************/


/*--------------------------------------------------------------------------*/
/* Module:		TI_AdapterApiCpp.h*/
/**/
/* Purpose:*/
/**/
/*--------------------------------------------------------------------------*/
#ifndef _TIWLAN_INTERFACE_H
#define _TIWLAN_INTERFACE_H

#include "paramOut.h"
#include "tiwlnif.h"
#include "roamingMngrTypes.h"

#ifdef EXC_MODULE_INCLUDED
    #define EXC_MODULE_SUPPORT_AH    ADAPTER_EXC_SUPPORT_AH
#else
    #define EXC_MODULE_SUPPORT_AH
#endif /*EXC_MODULE_INCLUDED */

struct   TI_WLAN_AdapterAPI
{	
    virtual ~TI_WLAN_AdapterAPI(){};
    virtual tiINT32     GetApiVersion               ( tiUINT32*    pdwApiVersion )                                  = 0;

    virtual tiINT32     GetBSSID                    ( OS_802_11_MAC_ADDRESS*    pAddrBSSID)							= 0;
    virtual tiINT32     SetBSSID                    ( OS_802_11_MAC_ADDRESS*    pAddrBSSID)	                        = 0;
    virtual tiINT32     GetBSSIDList                ( OS_802_11_BSSID_LIST_EX** ppBSSIDlist )                       = 0;
    virtual tiINT32     GetFullBSSIDList            ( OS_802_11_BSSID_LIST_EX** ppBSSIDlist )                       = 0;

    virtual tiINT32     GetCurrentAddress           ( OS_802_11_MAC_ADDRESS*    pCurrentAddr)                       = 0;
     
    virtual tiINT32     SetDesiredChannel           ( tiUINT32  uDesiredChannel   )                                 = 0;
    virtual tiINT32     GetDesiredChannel           ( tiUINT32* puDesiredChannel  )                                 = 0;
    virtual tiINT32     GetCurrentChannel           ( tiUINT32* puCurrentChannel  )                                 = 0;
        
    virtual tiINT32     GetDesiredRate              ( tiUINT32* puDesiredRates    )                                 = 0;
    virtual tiINT32     GetCurrentRate              ( tiUINT32* puCurrentRates    )                                 = 0;

    virtual tiINT32     SetFragmentThreshold        ( tiUINT32  uFragmentThreshold  )                               = 0;
    virtual tiINT32     GetFragmentThreshold        ( tiUINT32* puFragmentThreshold )                               = 0;

    virtual tiINT32     SetBSSType                  ( OS_802_11_NETWORK_MODE  uBSSType  )                           = 0;
    virtual tiINT32     GetBSSType                  ( OS_802_11_NETWORK_MODE* puBSSType )                           = 0;

    virtual tiINT32     SetLongRetry                ( tiUINT32  uLongRetry  )                                       = 0;
    virtual tiINT32     GetLongRetry                ( tiUINT32* puLongRetry )                                       = 0;
    
    virtual tiINT32     ConfigPowerManagement       ( OS_802_11_POWER_PROFILE thePowerMgrProfile )                  = 0;

    virtual tiINT32     SetRTSThreshold             ( tiUINT32  uRTSThreshold  )                                    = 0;
    virtual tiINT32     GetRTSThreshold             ( tiUINT32* puRTSThreshold )                                    = 0;    

    virtual tiINT32     SetShortPreamble            ( tiUINT32  uShortPreamble )                                    = 0;
    virtual tiINT32     GetShortPreamble            ( tiUINT32* puShortPreamble)                                    = 0;

    virtual tiINT32     SetShortRetry               ( tiUINT32  uShortRetry )                                       = 0;
    virtual tiINT32     GetShortRetry               ( tiUINT32* puShortRetry )                                      = 0;

    virtual tiINT32     SetSSID                     ( tiUINT8*   pSSIDname )                                         = 0;
    virtual tiINT32     GetDesiredSSID              ( OS_802_11_SSID*   pSSID       )                               = 0;
    virtual tiINT32     GetCurrentSSID              ( OS_802_11_SSID*   pSSID       )                               = 0;
    
    virtual tiINT32     GetStatistics               ( TIWLN_STATISTICS* ptiStatistics )                             = 0;
    virtual tiINT32     GetTxStatistics             ( TIWLN_TX_STATISTICS* ptiTxStatistics, UINT32 clearStatsFlag ) = 0;
    
    virtual tiINT32     EnableDisableRxDataFilters  ( tiBOOL  enabled )                                             = 0;
    virtual tiINT32     GetRxDataFiltersStatistics  ( TIWLAN_DATA_FILTER_STATISTICS * pStatistics )                 = 0;
    virtual tiINT32     AddRxDataFilter             ( TIWLAN_DATA_FILTER_REQUEST * pRequest )                       = 0;
    virtual tiINT32     RemoveRxDataFilter          ( TIWLAN_DATA_FILTER_REQUEST * pRequest )                       = 0;
    
    virtual tiINT32     SetSupportedRates           ( tiUINT8*  pSupportedRatesLst, tiUINT32  uBufLength)           = 0;
    virtual tiINT32     GetSupportedRates           ( tiUINT8*  pSupportedRatesLst, tiUINT32  uBufLength)           = 0;

    virtual tiINT32     GetDriverVersion            ( TIWLN_VERSION_EX*  pdrvVersion )                              = 0;

    virtual tiINT32     SetIBSSProtection           ( tiUINT32  uProtection )                                       = 0;
    virtual tiINT32     GetIBSSProtection           ( tiUINT32* puProtection)                                       = 0;

    virtual tiINT32     GetDriverState              ( driverState_e* puDriverState )                                = 0;

    virtual tiINT32     SetShortSlot                ( tiUINT32  uShortSlot )                                        = 0;
    virtual tiINT32     GetShortSlot                ( tiUINT32* puShortSlot)                                        = 0;

    virtual tiINT32     SetTxPowerDbm	            ( tiUINT8  uTxPower  )                                         = 0;
    virtual tiINT32     GetTxPowerLevel             ( tiCHAR* puTxPower )                                         = 0;
    virtual tiINT32     GetTxPowerDbm               ( tiCHAR* puTxPower )                                         = 0;

    virtual tiINT32     GetSupportedNetworkTypes    ( OS_802_11_NETWORK_TYPE* pNetTypeLst, tiUINT32 uMaxNetTypes )  = 0;
    virtual tiINT32     SetNetworkTypeInUse         ( OS_802_11_NETWORK_TYPE  uNetType   )                          = 0;
    virtual tiINT32     GetNetworkTypeInUse         ( OS_802_11_NETWORK_TYPE* puNetType  )                          = 0;
           
    virtual tiINT32     GetNumberOfAntennas         ( tiUINT32* puNumberOfAntennas    )                             = 0;
    virtual tiINT32     SetAntennaDiversityParams   ( PTIWLAN_ANT_DIVERSITY pAntennaDiversityOptions )              = 0;

    virtual tiINT32     EnableDisable_802_11d       ( tiUINT8  enableDisable_802_11d )                              = 0;
    virtual tiINT32     EnableDisable_802_11h       ( tiUINT8  enableDisable_802_11h )                              = 0;
    virtual tiINT32     Get_802_11d                 ( tiUINT8*  enableDisable_802_11d )                             = 0;
    virtual tiINT32     Get_802_11h                 ( tiUINT8*  enableDisable_802_11h )                             = 0;
    virtual tiINT32     Set_countryIeFor2_4_Ghz     ( country_t countryIe)                                          = 0;
    virtual tiINT32     Get_countryIeFor2_4_Ghz     ( tiUINT8 **countryString)                                       = 0;
    virtual tiINT32     Set_countryIeFor5_Ghz        ( country_t countryIe)                                          = 0;
    virtual tiINT32     Get_countryIeFor5_Ghz       ( tiUINT8 **countryString)                                       = 0;
    virtual tiINT32     Set_minMaxDfsChannels       (DFS_ChannelRange_t DFS_ChannelRange)                           = 0;
    virtual tiINT32     Get_minMaxDfsChannels       (DFS_ChannelRange_t *DFS_ChannelRange)                          = 0;


    virtual tiINT32     Start                       ()                                                              = 0;
    virtual tiINT32     Stop                        ()                                                              = 0;
    virtual tiINT32     Suspend                     ()                                                              = 0;
    virtual tiINT32     StartSM                     ()                                                              = 0;
    virtual tiINT32     StopSM                      ()                                                              = 0;
    
    virtual tiINT32     GetRSSI                     ( tiINT32* pRssi )						                        = 0;
	virtual tiINT32     GetSNR                      ( tiUINT32* pSnr )												= 0;
   
    virtual tiINT32     Disassociate                ( )                                                             = 0;
    
    virtual tiINT32     SetAuthenticationMode       ( OS_802_11_AUTHENTICATION_MODE  uAuthenticationMode )          = 0;
    virtual tiINT32     GetAuthenticationMode       ( OS_802_11_AUTHENTICATION_MODE* puAuthenticationMode)          = 0;

    virtual tiINT32     SetEAPType                  ( OS_802_11_EAP_TYPES  uEAPType )                               = 0;
    virtual tiINT32     SetEAPTypeDriver            ( OS_802_11_EAP_TYPES  uEAPType )                               = 0;
    virtual tiINT32     GetEAPType                  ( OS_802_11_EAP_TYPES* puEAPType)                               = 0;
 
    virtual tiINT32     SetEncryptionType           ( OS_802_11_ENCRYPTION_TYPES  uEncryptType )                    = 0;
    virtual tiINT32     GetEncryptionType           ( OS_802_11_ENCRYPTION_TYPES* puEncryptType)                    = 0;

    virtual tiINT32     SetCredentials              ( tiCHAR* pszUserName, tiCHAR* pszPassword )                    = 0;

    virtual tiINT32     SetPSK                      ( tiCHAR* pszPSK )                                              = 0;

    virtual tiINT32     SetKeyType                  ( OS_802_11_KEY_TYPES uKeyType )                                = 0;

 /*   virtual tiINT32     SetCertificateParameters    ( TI_SHA1_HASH* pSha1Hash, tiBOOL bValidateServerCert )         = 0;*/
    virtual tiINT32     SetCertParamsSHA1           ( TI_SHA1_HASH* pSha1Hash, tiBOOL bValidateServerCert )         = 0;
    virtual tiINT32     SetCertParamsFileName       ( tiCHAR* pszFileName,     tiBOOL bValidateServerCert )         = 0;

    virtual tiINT32     AddWEPKey                   ( OS_802_11_WEP*        pWEP        )                           = 0;
    virtual tiINT32     RemoveWEPKey                ( tiUINT32 uKeyIndex )                                          = 0;
    virtual tiINT32     GetDefaultWepKey            (tiUINT32* puKeyIndex )                                         = 0;
    
    virtual tiINT32     AddKey                      ( OS_802_11_KEY*        pKey        )                           = 0;
    virtual tiINT32     RemoveKey                   ( OS_802_11_REMOVE_KEY* pRemoveKey  )                           = 0;
    virtual tiINT32     SetMixedMode                ( tiBOOL    bStatus     )                                      = 0;
    virtual tiINT32     GetMixedMode                ( tiBOOL*    pbStatus     )                                      = 0;
    virtual tiINT32     SetPMKIDmap                 ( OS_802_11_PMKID*          pPMKIDMap   )                       = 0;

    virtual tiINT32     GetAssociationInfo          ( OS_802_11_ASSOCIATION_INFORMATION** ppInfo )                  = 0;

    virtual tiINT32     RegisterEvent               ( IPC_EVENT_PARAMS* pEventParams)                               = 0;
    virtual tiINT32     UnRegisterEvent             ( IPC_EVENT_PARAMS* pEventParams)                               = 0;

    virtual tiINT32     GetDriverCapabilities       ( OS_802_11_DRIVER_CAPABILITIES* pDriverCapabilities )          = 0;
	virtual tiINT32	    GetSelectedBSSIDInfo        ( OS_802_11_BSSID_EX  *pSelectedBSSIDInfo)                      = 0;
    virtual tiINT32	    GetPrimaryBSSIDInfo         ( OS_802_11_BSSID_EX  *pSelectedBSSIDInfo)                      = 0;


    virtual tiINT32		SetTrafficIntensityThresholds ( OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS* pTrafficThresholds) = 0;
    virtual tiINT32		GetTrafficIntensityThresholds ( OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS* pTrafficThresholds) = 0;
    virtual tiINT32		ToggleTrafficIntensityEvents ( tiUINT32 NewStatus )                                         = 0;

    virtual tiINT32     StartScan                   ( scan_Params_t *pScanParams )                                  = 0;
    virtual tiINT32     StopScan                    ( )                                                             = 0;
    virtual tiINT32     SetScanPolicy               ( UINT8* buffer, UINT16 bufferLength )                          = 0;
    virtual tiINT32     GetScanBssList              ( bssList_t* bssList )                                          = 0;

    virtual tiINT32	    PollApPackets               ( )                                                             = 0;
    virtual tiINT32	    PollApPacketsFromAC         ( tiUINT32 AC )                                                 = 0;
    virtual tiINT32	    SetDTagToAcMappingTable     ( acTrfcType_e* pDtagToAcTable )                         = 0;
    virtual tiINT32	    SetVAD                      ( txDataVadTimerParams_t* pVadTimer )                           = 0;
    virtual tiINT32	    GetVAD                      ( txDataVadTimerParams_t* pVadTimer )                           = 0;
    virtual tiINT32	    SetQosParameters            ( OS_802_11_QOS_PARAMS* pQosParams )                            = 0;
	virtual tiINT32	    SetRxTimeOut                ( OS_802_11_QOS_RX_TIMEOUT_PARAMS* pRxTimeOut )                 = 0;


	virtual tiINT32		GetAPQosParameters			( OS_802_11_AC_QOS_PARAMS* pACQosParams)						= 0;
    virtual	tiINT32		GetAPQosCapabilitesParameters ( OS_802_11_AP_QOS_CAPABILITIES_PARAMS* pAPQosCapabiltiesParams) = 0;
	virtual	tiINT32		AddTspec					( OS_802_11_QOS_TSPEC_PARAMS* pTspecParams)						= 0;
	virtual	tiINT32		GetTspecParameters			( OS_802_11_QOS_TSPEC_PARAMS* pTspecParams)						= 0;
	virtual	tiINT32		DeleteTspec					( OS_802_11_QOS_DELETE_TSPEC_PARAMS* pDelTspecParams)			= 0;
	virtual	tiINT32		GetCurrentACStatus      	( OS_802_11_AC_UPSD_STATUS_PARAMS *pAcStatusParams)             = 0;
	virtual	tiINT32		SetMediumUsageThreshold		( OS_802_11_THRESHOLD_CROSS_PARAMS* pThresholdCrossParams)		= 0;
	virtual	tiINT32		SetPhyRateThreshold			( OS_802_11_THRESHOLD_CROSS_PARAMS* pThresholdCrossParams)		= 0;
	virtual	tiINT32		GetMediumUsageThreshold		( OS_802_11_THRESHOLD_CROSS_PARAMS* pThresholdCrossParams)		= 0;
	virtual	tiINT32		GetPhyRateThreshold			( OS_802_11_THRESHOLD_CROSS_PARAMS* pThresholdCrossParams)		= 0;
	virtual	tiINT32		GetDesiredPsMode			( OS_802_11_QOS_DESIRED_PS_MODE* pDesiredPsMode)				= 0;

    virtual tiINT32     GetUserPriorityOfStream     ( STREAM_TRAFFIC_PROPERTIES* streamProperties)                  = 0;

    virtual tiINT32     ConfigTxClassifier          ( tiUINT32 inParamsBuffLen, tiUINT8  *pInParamsBuff)            = 0;
    virtual tiINT32	    RemoveClassifierEntry       ( clsfr_tableEntry_t *pClsfrEntry )                             = 0;
    virtual tiINT32     GetClsfrType                ( clsfrTypeAndSupport *currClsfrType )                          = 0;
    virtual tiINT32     Set4XState                   ( tiBOOL    bStatus     )                                      = 0;
    virtual tiINT32     Get4XState                   ( tiBOOL*   lpbStatus   )                                      = 0;
 
    virtual tiINT32     SetRSSITrigger               ( tiBOOL    bRSSItr     )                                      = 0;    
    virtual tiINT32     GetRSSITrigger               ( tiBOOL*   lpbRSSItr   )                                      = 0;
    
    virtual tiINT32     SetWEPStatus                 ( tiUINT32  dwWEPStatus     )                                  = 0;
    virtual tiINT32     GetWEPStatus                 ( tiUINT32* lpdwWEPStatus   )                                  = 0;

    virtual tiINT32     SetConfiguration             ( OS_802_11_CONFIGURATION*  pConfiguration )                   = 0;
    virtual tiINT32     GetConfiguration             ( OS_802_11_CONFIGURATION*  pConfiguration )                   = 0;    

    virtual tiINT32     SetPrivacyFilter             ( tiUINT32  dwPrivacyFilter )                                  = 0;
    virtual tiINT32     GetPrivacyFilter             ( tiUINT32* pdwPrivacyFilter)                                  = 0;

    virtual tiINT32     SetExtRatesIE                ( tiUINT32  dwExtRatesIE    )                                  = 0;
    virtual tiINT32     GetExtRatesIE                ( tiUINT32* pdwExtRatesIE   )                                  = 0;

    virtual tiINT32     SetEarlyWakeupMode                ( tiUINT8  uEarlyWakeup   )                                  = 0;
    virtual tiINT32     GetEarlyWakeupMode                ( tiUINT8* uEarlyWakeup   )                                  = 0;

    virtual tiINT32     IsDriverRun                 (tiBOOL* pbStatus )                                             = 0;

    virtual tiINT32     GWSICommand					 (tiUINT32* pGWSICommand )                                      = 0;
    virtual tiINT32     GWSIInitialize				 (tiUINT32* pGWSICommand )                                      = 0;
    virtual tiINT32     GWSIConfig					 (tiUINT32* pGWSICommand )                                      = 0;
    virtual tiINT32     GWSIRelease					 (tiUINT32* pGWSICommand )                                      = 0;
	virtual tiINT32     GWSIGetInitTable			 (tiUINT32* pGWSICommand )										= 0;

    virtual tiINT32     hwReadRegister               ( tiUINT32 dwRegisterAddr, tiUINT32* pdwValue )                = 0;
    virtual tiINT32     hwWriteRegister              ( tiUINT32 dwRegisterAddr, tiUINT32  dwValue  )                = 0;

    virtual tiINT32     ReloadDefaults              ( )                                                             = 0;
    virtual tiINT32     IsDriverLoaded              ( )                                                             = 0;

	virtual tiINT32     SetBtCoeEnable              ( tiUINT32 uModeEnable )											= 0;
	virtual tiINT32     SetBtCoeRate				( tiUINT8 *pRate )												= 0;
	virtual tiINT32     SetBtCoeConfig				( tiUINT32 *pConfig )											= 0;
	virtual tiINT32     SetBtCoeGetStatus			( tiUINT32 *pStatus )											= 0;

#ifdef TI_DBG    
    virtual tiINT32     GetDebugBuffer              ( tiUINT8* pBuffer, tiUINT32  dwLenght  )                       = 0;
   
    virtual tiINT32     SetReportModule             ( tiUINT8* pData )                                             = 0;
    virtual tiINT32     GetReportModule             ( tiUINT8* pData )                                             = 0;    

    virtual tiINT32     SetReportSeverity           ( tiUINT8* pData )                                             = 0;
    virtual tiINT32     GetReportSeverity           ( tiUINT8* pData )                                             = 0;    

    virtual tiINT32     SetOsDbgState               ( tiUINT32  uData )                                             = 0;
    virtual tiINT32     GetOsDbgState               ( tiUINT32* puData )                                             = 0;    

    virtual tiINT32     SetReportPPMode             ( tiUINT32  uData )                                             = 0;

    virtual tiINT32     DisplayStats                ( tiUINT8*  puDbgBuffer,    tiUINT32 uBuffSize)                 = 0;

#ifdef DRIVER_PROFILING
    virtual tiINT32     ProfileReport               ()                                                              = 0;
    virtual tiINT32     CpuEstimatorCommand         (tiUINT8 uType, tiUINT32 uData)                                 = 0;
#endif

#endif /*TI_DBG*/
    virtual tiINT32     SetWPAOptions               ( tiUINT32      fWPA_options)                                                       = 0;
    virtual tiINT32     GetWPAOptions               ( tiUINT32 * fWPA_options)                                      = 0;

    virtual tiINT32     GetRegDomainTable           ( TIWLN_REGDOMAINS*     pRegDomainTable )                       = 0;
    virtual tiINT32     GetMediumUsage              ( TIWLN_MEDIUM_USAGE*   pMediumUsage    )                       = 0;
    
    virtual tiINT32     SetPowerMode                ( OS_802_11_POWER_PROFILE  uPower   )                           = 0;
    virtual tiINT32     GetPowerMode                ( OS_802_11_POWER_PROFILE* puPower  )                           = 0;

	virtual tiINT32 SetPowerLevelPS( OS_802_11_POWER_LEVELS   uPower) = 0;
	virtual tiINT32 GetPowerLevelPS( OS_802_11_POWER_LEVELS* puPower) = 0;	

	virtual tiINT32 SetPowerLevelDefault( OS_802_11_POWER_LEVELS   uPower) = 0;
	virtual tiINT32 GetPowerLevelDefault( OS_802_11_POWER_LEVELS* puPower) = 0;	
	
	virtual tiINT32 SetPowerLevelDozeMode( OS_802_11_POWER_PROFILE   uPower) = 0;
	virtual tiINT32 GetPowerLevelDozeMode( OS_802_11_POWER_PROFILE* puPower) = 0;	

    
	virtual tiINT32 SetBeaconFilterDesiredState( OS_802_11_BEACON_FILTER_MODE   uBeaconFilterMode) = 0;
	virtual tiINT32 GetBeaconFilterDesiredState( tiUINT8*  pBeaconFilterMode) = 0;


    virtual tiINT32     Open_EAPOL_Interface        ( )                                                             = 0;
    virtual tiINT32     Close_EAPOL_Interface       ( )                                                             = 0;
    virtual tiINT32     Send_EAPOL_Packet           ( tiVOID* pData, tiUINT32 uSize )                               = 0;

	virtual tiINT32     Set_RoamingConfParams   	( UINT8* buffer, UINT16 bufferLength)  						    = 0;
	virtual tiINT32     Get_RoamingConfParams   	( UINT8* buffer, UINT16 bufferLength ) 						    = 0;
    
    virtual tiINT32     GetPowerConsumptionStatistics( PowerConsumptionTimeStat_t * pStatistics)                    = 0;
    
	/*PLT*/
	virtual tiINT32		PLT_ReadRegister			( UINT32 uRegisterAddr, UINT32* uRegisterData )					= 0;
	virtual tiINT32		PLT_WriteRegister			( UINT32 uRegisterAddr, UINT32 uRegisterData )					= 0;
	virtual tiINT32		PLT_RxPerStart				()																= 0;
	virtual tiINT32		PLT_RxPerStop				()																= 0;
	virtual tiINT32		PLT_RxPerClear				()																= 0;
	virtual tiINT32		PLT_RxPerGetResults			( PltRxPer_t* pPltRxPer )										= 0;
	virtual tiINT32		PLT_TxCW					( TestCmdChannelBand_t* pPltTxCW)                                     = 0;
	virtual tiINT32		PLT_TxContiues				( PltTxContinues_t* pPltTxContinues)                            = 0;
	virtual tiINT32		PLT_TxStop  				()																= 0;
	virtual tiINT32		PLT_ReadMIB					( PLT_MIB_t* pMib )                         					= 0;
	virtual tiINT32		PLT_WriteMIB				( PLT_MIB_t* pMib )                         					= 0;
	virtual tiINT32		PLT_TxCalGainGet		    (PltGainGet_t* pPLTGainGet)                                     = 0;
	virtual tiINT32		PLT_TxCalGainAdjust	        (tiINT32   uTxGainChange)                                       = 0;
	virtual tiINT32		PLT_TxCalStart	            (PltTxCalibrationRequest_t* pPLTTxCal)                          = 0;
    virtual tiINT32		PLT_TxCalStop 	            ()                                                              = 0;
	virtual tiINT32		PLT_RxTxCalNVSUpdateBuffer	(PltNvsResultsBuffer_t* pPLT_NVSUpdateBuffer)                   = 0;
	virtual tiINT32		PLT_RxCal                   (PltRxCalibrationRequest_t* pPltRxCalibration_t)                = 0; 
    virtual tiINT32     PLT_RadioTune               (TestCmdChannelBand_t* pChannelBand)                                   = 0;

    #ifdef _WINDOWS
	#endif
    

    EXC_MODULE_SUPPORT_AH
};	/* TI_WLAN_AdapterAPI*/

#ifdef  __cplusplus
extern "C" {
#endif
    TI_WLAN_AdapterAPI* TI_AdapterCppInit           ( tiCHAR* pszDeviceName , tiBOOL bForce ); 
     tiINT32             TI_AdapterCppDeinit         ( TI_WLAN_AdapterAPI*  pAdapter, tiBOOL bForce);
#ifdef  __cplusplus
}
#endif

#endif /*_TIWLAN_INTERFACE_H*/
