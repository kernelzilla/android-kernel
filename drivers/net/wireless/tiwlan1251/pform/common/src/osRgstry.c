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

#if defined(_WINDOWS)
#elif defined( __LINUX__ )
#include "osrgstry_parser.h"
#elif defined(__ARMCC__) 
#include "osrgstry_parser.h"
#include "string.h"
#endif

#include "osRgstry.h"
#include "paramOut.h"
#include "osDot11.h"
#include "osApi.h"


/* TODO: remove this line!*/
#include "802_11Defs.h"

extern rate_e  RateNumberToHost(UINT8 rateIn);

#define MAX_KEY_BUFFER_LEN      256

#define N_STR(str)              NDIS_STRING_CONST(str)
#define INIT_TBL_OFF(field)     FIELD_OFFSET(initTable_t, field)

/* Reports */
NDIS_STRING STR_ReportSeverityTable          = NDIS_STRING_CONST( "ReportSeverityTable" );
NDIS_STRING STR_ReportModuleTable            = NDIS_STRING_CONST( "ReportModuleTable" );


NDIS_STRING STRFilterEnabled            = NDIS_STRING_CONST( "Mac_Filter_Enabled");
NDIS_STRING STRnumGroupAddrs            = NDIS_STRING_CONST( "numGroupAddrs" );
NDIS_STRING STRGroup_addr0              = NDIS_STRING_CONST( "Group_addr0" );
NDIS_STRING STRGroup_addr1              = NDIS_STRING_CONST( "Group_addr1" );
NDIS_STRING STRGroup_addr2              = NDIS_STRING_CONST( "Group_addr2" );
NDIS_STRING STRGroup_addr3              = NDIS_STRING_CONST( "Group_addr3" );
NDIS_STRING STRGroup_addr4              = NDIS_STRING_CONST( "Group_addr4" );
NDIS_STRING STRGroup_addr5              = NDIS_STRING_CONST( "Group_addr5" );
NDIS_STRING STRGroup_addr6              = NDIS_STRING_CONST( "Group_addr6" );
NDIS_STRING STRGroup_addr7              = NDIS_STRING_CONST( "Group_addr7" );

/* Beacon timing */
/* If Early Wakeup is Enabled, 1251 wakes-up EARLY_WAKEUP_TIME before expected Beacon reception occasion */
/* If Early Wakeup is Disabled, 1251 wakes-up at the expected Beacon reception occasion. */
NDIS_STRING STREarlyWakeup                 = NDIS_STRING_CONST( "EarlyWakeup" );
    
NDIS_STRING STRArp_Ip_Addr              = NDIS_STRING_CONST( "ArpIp_Addr" );
NDIS_STRING STRArp_Ip_Filter_Ena        = NDIS_STRING_CONST( "ArpIp_Filter_ena");


NDIS_STRING STRBeaconFilterDesiredState = NDIS_STRING_CONST( "Beacon_Filter_Desired_State") ;
NDIS_STRING STRBeaconFilterStored       = NDIS_STRING_CONST( "Beacon_Filter_Stored") ;  

/*this is for configuring table from ini file*/
NDIS_STRING STRBeaconIETableSize            = NDIS_STRING_CONST( "Beacon_IE_Table_Size") ;  
NDIS_STRING STRBeaconIETable                = NDIS_STRING_CONST( "Beacon_IE_Table") ;
NDIS_STRING STRBeaconIETableNumOfElem       = NDIS_STRING_CONST( "Beacon_IE_Num_Of_Elem") ; 

NDIS_STRING STRTxXferBufferFullTimeToRecovery = NDIS_STRING_CONST( "TxXferBufferFullTimeToRecovery" );

/* ------------------------------------------------------ */
NDIS_STRING STRFirmwareDebug                = NDIS_STRING_CONST( "FirmwareDebug" );
NDIS_STRING STRTraceBufferSize              = NDIS_STRING_CONST( "TraceBufferSize" );
NDIS_STRING STRPrintTrace                   = NDIS_STRING_CONST( "PrintTrace" );

NDIS_STRING STRHwACXAccessMethod            = NDIS_STRING_CONST( "HwACXAccessMethod" );
NDIS_STRING STRMaxSitesFragCollect          = NDIS_STRING_CONST( "MaxSitesFragCollect" );

NDIS_STRING STRNumACXRxDescriptors          = NDIS_STRING_CONST( "NumACXRxDescriptors" );
NDIS_STRING STRNumACXTxDescriptors          = NDIS_STRING_CONST( "NumACXTxDescriptors" );
NDIS_STRING STRTxFlashEnable                = NDIS_STRING_CONST( "TxFlashEnable" );

NDIS_STRING STRBetEnable					= NDIS_STRING_CONST( "BetEnable");
NDIS_STRING STRBetMaxConsecutive			= NDIS_STRING_CONST( "BetMaxConsecutive");
NDIS_STRING STRMaxFullBeaconInterval        = NDIS_STRING_CONST( "MaxlFullBeaconReceptionInterval" );
NDIS_STRING STRBetEnableThreshold			= NDIS_STRING_CONST( "BetEnableThreshold");
NDIS_STRING STRBetDisableThreshold			= NDIS_STRING_CONST( "BetDisableThreshold");

NDIS_STRING STRNumHostRxDescriptors         = NDIS_STRING_CONST( "NumHostRxDescriptors" );
NDIS_STRING STRNumHostTxDescriptors         = NDIS_STRING_CONST( "NumHostTxDescriptors" );

NDIS_STRING STRACXMemoryBlockSize           = NDIS_STRING_CONST( "ACXMemoryBlockSize" );
NDIS_STRING STRACXRxMemoryBlockSize         = NDIS_STRING_CONST( "ACXMemoryBlockSize" );
NDIS_STRING STRACXTxMemoryBlockSize         = NDIS_STRING_CONST( "ACXMemoryBlockSize" );

NDIS_STRING STRACXUseTxDataInterrupt        = NDIS_STRING_CONST( "ACXUseTxDataInterrupt" );
NDIS_STRING STRACXUseInterruptThreshold     = NDIS_STRING_CONST( "ACXUseInterruptThreshold" );

NDIS_STRING STRCalibrationChannel2_4        = NDIS_STRING_CONST( "CalibrationChannel24" );
NDIS_STRING STRCalibrationChannel5_0        = NDIS_STRING_CONST( "CalibrationChannel5" );
NDIS_STRING STRdot11RTSThreshold            = NDIS_STRING_CONST( "dot11RTSThreshold" );
NDIS_STRING STRRxDisableBroadcast           = NDIS_STRING_CONST( "RxDisableBroadcast" );
NDIS_STRING STRRecoveryEnable               = NDIS_STRING_CONST( "RecoveryEnable" );
NDIS_STRING STRdot11TxAntenna               = NDIS_STRING_CONST( "dot11TxAntenna" );
NDIS_STRING STRdot11RxAntenna               = NDIS_STRING_CONST( "dot11RxAntenna" );
NDIS_STRING STRTxCompleteThreshold          = NDIS_STRING_CONST( "TxCompleteThreshold" );

NDIS_STRING STRdot11FragThreshold           = NDIS_STRING_CONST( "dot11FragmentationThreshold" );
NDIS_STRING STRdot11MaxTxMSDULifetime       = NDIS_STRING_CONST( "dot11MaxTransmitMSDULifetime" );
NDIS_STRING STRdot11MaxReceiveLifetime      = NDIS_STRING_CONST( "dot11MaxReceiveLifetime" );
NDIS_STRING STRdot11RateFallBackRetryLimit  = NDIS_STRING_CONST( "dot11RateFallBackRetryLimit");

NDIS_STRING STRListenInterval               = NDIS_STRING_CONST( "dot11ListenInterval" );
NDIS_STRING STRExternalMode                 = NDIS_STRING_CONST( "DriverExternalMode" );
NDIS_STRING STRWiFiAdHoc                    = NDIS_STRING_CONST( "WiFiAdhoc" );
NDIS_STRING STRWiFiWmmPS                    = NDIS_STRING_CONST( "WiFiWmmPS" );
NDIS_STRING STRKeepAliveEnable              = NDIS_STRING_CONST( "KeepAliveEnable" );
NDIS_STRING STRdot11DesiredChannel          = NDIS_STRING_CONST( "dot11DesiredChannel");
NDIS_STRING STRdot11DesiredSSID             = NDIS_STRING_CONST( "dot11DesiredSSID" );
NDIS_STRING STRdot11DesiredBSSType          = NDIS_STRING_CONST( "dot11DesiredBSSType" );

NDIS_STRING STRdot11BasicRateMask_B           = NDIS_STRING_CONST( "dot11BasicRateMaskB");
NDIS_STRING STRdot11SupportedRateMask_B       = NDIS_STRING_CONST( "dot11SupportedRateMaskB");
NDIS_STRING STRdot11BasicRateMask_G           = NDIS_STRING_CONST( "dot11BasicRateMaskG");
NDIS_STRING STRdot11SupportedRateMask_G       = NDIS_STRING_CONST( "dot11SupportedRateMaskG");
NDIS_STRING STRdot11BasicRateMask_A           = NDIS_STRING_CONST( "dot11BasicRateMaskA");
NDIS_STRING STRdot11SupportedRateMask_A       = NDIS_STRING_CONST( "dot11SupportedRateMaskA");
NDIS_STRING STRdot11BasicRateMask_AG           = NDIS_STRING_CONST( "dot11BasicRateMaskAG");
NDIS_STRING STRdot11SupportedRateMask_AG       = NDIS_STRING_CONST( "dot11SupportedRateMaskAG");

NDIS_STRING STRdot11DesiredTxRate           = NDIS_STRING_CONST( "dot11DesiredTxRate");
NDIS_STRING STRdot11MgmtCtrlTxRateSelection = NDIS_STRING_CONST( "dot11MgmtCtrlTxRateSelection");
NDIS_STRING STRdot11MgmtCtrlTxRate          = NDIS_STRING_CONST( "dot11MgmtCtrlTxRate");


NDIS_STRING STRRadio11_RxLevel              = NDIS_STRING_CONST( "Radio11_RxLevel");
NDIS_STRING STRRadio11_LNA                  = NDIS_STRING_CONST( "Radio11_LNA");
NDIS_STRING STRRadio11_RSSI                 = NDIS_STRING_CONST( "Radio11_RSSI");
NDIS_STRING STRRadio0D_RxLevel              = NDIS_STRING_CONST( "Radio0D_RxLevel");
NDIS_STRING STRRadio0D_LNA                  = NDIS_STRING_CONST( "Radio0D_LNA");
NDIS_STRING STRRadio0D_RSSI                 = NDIS_STRING_CONST( "Radio0D_RSSI");

NDIS_STRING STRdot11DesiredNetworkType      = NDIS_STRING_CONST( "dot11NetworkType");
NDIS_STRING STRdot11DefaultNetworkType      = NDIS_STRING_CONST( "dot11DefaultNetworkType");
NDIS_STRING STRdot11SlotTime                = NDIS_STRING_CONST( "ShortSlotTime");
NDIS_STRING STRdot11IbssProtection          = NDIS_STRING_CONST( "IbssProtectionType");
NDIS_STRING STRdot11RtsCtsProtection        = NDIS_STRING_CONST( "dot11RtsCtsProtection");

NDIS_STRING STRRxEnergyDetection              = NDIS_STRING_CONST( "RxEnergyDetection" );
NDIS_STRING STRTxEnergyDetection              = NDIS_STRING_CONST( "TxEnergyDetection" );
NDIS_STRING STRCrtCalibrationInterval       = NDIS_STRING_CONST( "CrtCalibrationInterval" );
NDIS_STRING STRTddCalibrationInterval       = NDIS_STRING_CONST( "TddCalibrationInterval" );
NDIS_STRING STRMacClockRate                 = NDIS_STRING_CONST( "MacClockRate" );
NDIS_STRING STRArmClockRate                 = NDIS_STRING_CONST( "ArmClockRate" );
NDIS_STRING STRg80211DraftNumber            = NDIS_STRING_CONST( "g80211DraftNumber" );

NDIS_STRING STRRateAdaptationTable0         = NDIS_STRING_CONST( "RateAdaptationTable0" );
NDIS_STRING STRRateAdaptationFBThd0         = NDIS_STRING_CONST( "RateAdaptationFBThd0" );
NDIS_STRING STRRateAdaptationSUThd0         = NDIS_STRING_CONST( "RateAdaptationSUThd0" );

NDIS_STRING STRRateAdaptationTable1         = NDIS_STRING_CONST( "RateAdaptationTable1" );
NDIS_STRING STRRateAdaptationFBThd1         = NDIS_STRING_CONST( "RateAdaptationFBThd1" );
NDIS_STRING STRRateAdaptationSUThd1         = NDIS_STRING_CONST( "RateAdaptationSUThd1" );

NDIS_STRING STRRateAdaptationTable2         = NDIS_STRING_CONST( "RateAdaptationTable2" );
NDIS_STRING STRRateAdaptationFBThd2         = NDIS_STRING_CONST( "RateAdaptationFBThd2" );
NDIS_STRING STRRateAdaptationSUThd2         = NDIS_STRING_CONST( "RateAdaptationSUThd2" );

NDIS_STRING STRRateAdaptationTable3         = NDIS_STRING_CONST( "RateAdaptationTable3" );
NDIS_STRING STRRateAdaptationFBThd3         = NDIS_STRING_CONST( "RateAdaptationFBThd3" );
NDIS_STRING STRRateAdaptationSUThd3         = NDIS_STRING_CONST( "RateAdaptationSUThd3" );

/* Tspec Rate Thresholds */
NDIS_STRING STRRateAdaptationHighTrshAcVO   = NDIS_STRING_CONST( "RateAdaptationHighTrshAcVO" );
NDIS_STRING STRRateAdaptationHighTrshAcVI   = NDIS_STRING_CONST( "RateAdaptationHighTrshAcVI" );
NDIS_STRING STRRateAdaptationHighTrshAcBE   = NDIS_STRING_CONST( "RateAdaptationHighTrshAcBE" );
NDIS_STRING STRRateAdaptationHighTrshAcBK   = NDIS_STRING_CONST( "RateAdaptationHighTrshAcBK" );

NDIS_STRING STRRateAdaptationLowTrshAcVO   = NDIS_STRING_CONST( "RateAdaptationLowTrshAcVO" );
NDIS_STRING STRRateAdaptationLowTrshAcVI   = NDIS_STRING_CONST( "RateAdaptationLowTrshAcVI" );
NDIS_STRING STRRateAdaptationLowTrshAcBE   = NDIS_STRING_CONST( "RateAdaptationLowTrshAcBE" );
NDIS_STRING STRRateAdaptationLowTrshAcBK   = NDIS_STRING_CONST( "RateAdaptationLowTrshAcBK" );

NDIS_STRING STRdot11ShortPreambleInvoked    = NDIS_STRING_CONST( "dot11ShortPreambleInvoked" );

NDIS_STRING STRdot11BeaconPeriod            = NDIS_STRING_CONST( "dot11BeaconPeriod" );
NDIS_STRING STRdot11MaxScanTime             = NDIS_STRING_CONST( "dot11MaxScanTime" );
NDIS_STRING STRdot11MinScanTime             = NDIS_STRING_CONST( "dot11MinScanTime" );
NDIS_STRING STRdot11MaxSiteLifetime         = NDIS_STRING_CONST( "dot11MaxSiteLifetime" );

NDIS_STRING STRdot11MaxAuthRetry            = NDIS_STRING_CONST( "dot11MaxAuthRetry" );
NDIS_STRING STRdot11MaxAssocRetry           = NDIS_STRING_CONST( "dot11MaxAssocRetry" );
NDIS_STRING STRdot11AuthRespTimeout         = NDIS_STRING_CONST( "dot11AuthenticationResponseTimeout" );
NDIS_STRING STRdot11AssocRespTimeout        = NDIS_STRING_CONST( "dot11AssociationResponseTimeout" );

NDIS_STRING STRConnSelfTimeout              = NDIS_STRING_CONST( "ConnSelfTimeout" );

NDIS_STRING STRNumTxDataQueues              = NDIS_STRING_CONST( "NumTxDataQueues" );
NDIS_STRING STRCreditCalcTimout             = NDIS_STRING_CONST( "CreditCalcTimout" );
NDIS_STRING STRCreditCalcTimerEnabled       = NDIS_STRING_CONST( "CreditCalcTimerEnabled" );
NDIS_STRING STRFracOfLifeTimeToDrop         = NDIS_STRING_CONST( "FracOfLifeTimeToDrop" );
NDIS_STRING STRAdmCtrlDelayDueToMediumTimeOverUsage = NDIS_STRING_CONST( "AdmCtrlDelayDueToMediumTimeOverUsage" );
NDIS_STRING STRAdmissionDownGradeEnable     = NDIS_STRING_CONST( "AdmissionDownGradeEnable" );

NDIS_STRING STRTrafficAdmControlTimeout     = NDIS_STRING_CONST("TrafficAdmControlTimeout");
NDIS_STRING STRTrafficAdmControlUseFixedMsduSize = NDIS_STRING_CONST("TrafficAdmCtrlUseFixedMsduSize");
NDIS_STRING STRDesiredMaxSpLen              = NDIS_STRING_CONST("DesiredMaxSpLen");

NDIS_STRING STRRateContThreshold            = NDIS_STRING_CONST( "RateContThreshold");
NDIS_STRING STRRateStepUpThreshold          = NDIS_STRING_CONST( "RateStepUpThreshold");
NDIS_STRING STRFBShortInterval              = NDIS_STRING_CONST( "FBShortInterval");
NDIS_STRING STRFBLongInterval               = NDIS_STRING_CONST( "FBLongInterval");
NDIS_STRING STRRateAdaptationTimeout        = NDIS_STRING_CONST( "RateAdaptationTimeout");
NDIS_STRING STRRateControlEnable            = NDIS_STRING_CONST( "RateControlEnable" );

NDIS_STRING STRRatePolicyUserShortRetryLimit   = NDIS_STRING_CONST( "RatePolicyUserShortRetryLimit" );
NDIS_STRING STRRatePolicyUserLongRetryLimit    = NDIS_STRING_CONST( "RatePolicyUserLongRetryLimit" );
NDIS_STRING STRRatePolicyUserRetriesPerRateCck     = NDIS_STRING_CONST( "RatePolicyUserRetriesPerRateCck" );
NDIS_STRING STRRatePolicyUserRetriesPerRatePbcc    = NDIS_STRING_CONST( "RatePolicyUserRetriesPerRatePbcc" );
NDIS_STRING STRRatePolicyUserRetriesPerRateOfdm    = NDIS_STRING_CONST( "RatePolicyUserRetriesPerRateOfdm" );
NDIS_STRING STRRatePolicyUserRetriesPerRateOfdmA   = NDIS_STRING_CONST( "RatePolicyUserRetriesPerRateOfdmA" );

NDIS_STRING STRRatePolicySGShortRetryLimit   = NDIS_STRING_CONST( "RatePolicySGShortRetryLimit" );
NDIS_STRING STRRatePolicySGLongRetryLimit    = NDIS_STRING_CONST( "RatePolicySGLongRetryLimit" );
NDIS_STRING STRRatePolicySGRetriesPerRateCck     = NDIS_STRING_CONST( "RatePolicySGRetriesPerRateCck" );
NDIS_STRING STRRatePolicySGRetriesPerRatePbcc    = NDIS_STRING_CONST( "RatePolicySGRetriesPerRatePbcc" );
NDIS_STRING STRRatePolicySGRetriesPerRateOfdm    = NDIS_STRING_CONST( "RatePolicySGRetriesPerRateOfdm" );
NDIS_STRING STRRatePolicySGRetriesPerRateOfdmA   = NDIS_STRING_CONST( "RatePolicySGRetriesPerRateOfdmA" );


NDIS_STRING STRdot11FourXEnable             = NDIS_STRING_CONST( "Mode4x" );

NDIS_STRING STRdot11AuthenticationMode      = NDIS_STRING_CONST( "dot11AuthenticationMode" );
NDIS_STRING STRdot11WEPStatus               = NDIS_STRING_CONST( "dot11WEPStatus" );
NDIS_STRING STRdot11ExcludeUnencrypted      = NDIS_STRING_CONST( "dot11ExcludeUnencrypted" );
NDIS_STRING STRdot11WEPKeymappingLength     = NDIS_STRING_CONST( "dot11WEPKeymappingLength" );
NDIS_STRING STRdot11WEPDefaultKeyID         = NDIS_STRING_CONST( "dot11WEPDefaultKeyID" );

NDIS_STRING STRMixedMode                    = NDIS_STRING_CONST( "MixedMode" );

NDIS_STRING STRWPAMixedMode                  = NDIS_STRING_CONST( "WPAMixedMode");
NDIS_STRING STRRSNPreAuth                    = NDIS_STRING_CONST( "RSNPreAuthentication");
NDIS_STRING STRRSNPreAuthTimeout             = NDIS_STRING_CONST( "RSNPreAuthTimeout" );

NDIS_STRING STRTimeToResetCountryMs         = NDIS_STRING_CONST( "TimeToResetCountryMs" );
NDIS_STRING STRMultiRegulatoryDomainEnabled = NDIS_STRING_CONST( "MultiRegulatoryDomain" );
NDIS_STRING STRSpectrumManagementEnabled    = NDIS_STRING_CONST( "SpectrumManagement" );
NDIS_STRING STRScanControlTable24           = NDIS_STRING_CONST( "AllowedChannelsTable24" );
NDIS_STRING STRScanControlTable5            = NDIS_STRING_CONST( "AllowedChannelsTable5" );

/*
Power Manager
*/
NDIS_STRING STRPowerMode                    = NDIS_STRING_CONST( "dot11PowerMode" );
NDIS_STRING STRBeaconReceiveTime            = NDIS_STRING_CONST( "BeaconReceiveTime" );
NDIS_STRING STRBaseBandWakeUpTime           = NDIS_STRING_CONST( "BaseBandWakeUpTime" );
NDIS_STRING STRHangoverPeriod               = NDIS_STRING_CONST( "HangoverPeriod" );
NDIS_STRING STRBeaconListenInterval         = NDIS_STRING_CONST( "BeaconListenInterval" );
NDIS_STRING STRDtimListenInterval         = NDIS_STRING_CONST( "DtimListenInterval" );
NDIS_STRING STRNConsecutiveBeaconsMissed    = NDIS_STRING_CONST( "NConsecutiveBeaconsMissed" );
NDIS_STRING STREnterTo802_11PsRetries       = NDIS_STRING_CONST( "EnterTo802_11PsRetries" );
NDIS_STRING STRAutoPowerModeInterval        = NDIS_STRING_CONST( "AutoPowerModeInterval" );
NDIS_STRING STRAutoPowerModeActiveTh        = NDIS_STRING_CONST( "AutoPowerModeActiveTh" );
NDIS_STRING STRAutoPowerModeDozeTh          = NDIS_STRING_CONST( "AutoPowerModeDozeTh" );
NDIS_STRING STRAutoPowerModeDozeMode        = NDIS_STRING_CONST( "AutoPowerModeDozeMode" );
NDIS_STRING STRDefaultPowerLevel        = NDIS_STRING_CONST( "defaultPowerLevel" );
NDIS_STRING STRPowerSavePowerLevel  = NDIS_STRING_CONST( "PowerSavePowerLevel" );

NDIS_STRING STRPsPollDeliveryFailureRecoveryPeriod     = NDIS_STRING_CONST( "PsPollDeliveryFailureRecoveryPeriod" );

NDIS_STRING STRPowerMgmtHangOverPeriod      = NDIS_STRING_CONST( "PowerMgmtHangOverPeriod" );
NDIS_STRING STRPowerMgmtMode                = NDIS_STRING_CONST( "PowerMgmtMode" );
NDIS_STRING STRPowerMgmtNeedToSendNullData  = NDIS_STRING_CONST( "PowerMgmtNeedToSendNullData" );
NDIS_STRING STRPowerMgmtNullPktRateModulation = NDIS_STRING_CONST( "PowerMgmtNullPktRateModulation" );
NDIS_STRING STRPowerMgmtNumNullPktRetries   = NDIS_STRING_CONST( "PowerMgmtNumNullPktRetries" );
NDIS_STRING STRPowerMgmtPllLockTime         = NDIS_STRING_CONST( "PllLockTime" );

NDIS_STRING STRBeaconRxTimeout     = NDIS_STRING_CONST( "BeaconRxTimeout" );
NDIS_STRING STRBroadcastRxTimeout  = NDIS_STRING_CONST( "BroadcastRxTimeout" );
NDIS_STRING STRRxBroadcastInPs     = NDIS_STRING_CONST( "RxBroadcastInPs" );

NDIS_STRING STRConsecutivePsPollDeliveryFailureThreshold = NDIS_STRING_CONST( "ConsecutivePsPollDeliveryFailureThreshold" );

NDIS_STRING STRTxPower                      = NDIS_STRING_CONST( "TxPower" );

/* Scan SRV */
NDIS_STRING STRNumberOfNoScanCompleteToRecovery         = NDIS_STRING_CONST( "NumberOfNoScanCompleteToRecovery" );
NDIS_STRING STRTriggeredScanTimeOut                     = NDIS_STRING_CONST( "TriggeredScanTimeOut" );

/*-----------------------------------*/
/*   Bluetooth support               */
/*-----------------------------------*/
NDIS_STRING STRBThWlanCoexistEnable                     = NDIS_STRING_CONST( "BThWlanCoexistEnable" );
NDIS_STRING STRBThWlanCoexistRate                       = NDIS_STRING_CONST( "BThWlanCoexistRate" );
NDIS_STRING STRBThWlanCoexistParamsbtHpMaxTime          = NDIS_STRING_CONST( "BThWlanCoexistParamsbtHpMaxTime" );
NDIS_STRING STRBThWlanCoexistParamswlanHpMaxTime            = NDIS_STRING_CONST( "BThWlanCoexistParamswlanHpMaxTime" );
NDIS_STRING STRBThWlanCoexistParamssenseDisableTimer        = NDIS_STRING_CONST( "BThWlanCoexistParamssenseDisableTimer" );
NDIS_STRING STRBThWlanCoexistParamsprotectiveRxTimeBeforeBtHp       = NDIS_STRING_CONST( "BThWlanCoexistParamsprotectiveRxTimeBeforeBtHp" );
NDIS_STRING STRBThWlanCoexistParamsprotectiveTxTimeBeforeBtHp           = NDIS_STRING_CONST( "BThWlanCoexistParamstimeoutWlanPacketCount" );
NDIS_STRING STRBThWlanCoexistParamsprotectiveRxTimeBeforeBtHpFastAp     = NDIS_STRING_CONST( "BThWlanCoexistParamsprotectiveRxTimeBeforeBtHpFastAp" );
NDIS_STRING STRBThWlanCoexistParamsprotectiveTxTimeBeforeBtHpFastAp     = NDIS_STRING_CONST( "BThWlanCoexistParamsprotectiveTxTimeBeforeBtHpFastAp" );
NDIS_STRING STRBThWlanCoexistParamsprotectiveWlanCycleTimeForFastAp     = NDIS_STRING_CONST( "BThWlanCoexistParamsprotectiveWlanCycleTimeForFastAp" );
NDIS_STRING STRBThWlanCoexistParamstimeoutNextBtLpPacket = NDIS_STRING_CONST( "BThWlanCoexistParamstimeoutNextBtLpPacket" );
NDIS_STRING STRBThWlanCoexistParamssgAntennaType        = NDIS_STRING_CONST( "BThWlanCoexistParamssgAntennaType" );
NDIS_STRING STRBThWlanCoexistParamssignalingType        = NDIS_STRING_CONST( "BThWlanCoexistParamssignalingType" );
NDIS_STRING STRBThWlanCoexistParamsafhLeverageOn    = NDIS_STRING_CONST( "BThWlanCoexistParamsafhLeverageOn" );
NDIS_STRING STRBThWlanCoexistParamsnumberQuietCycle         = NDIS_STRING_CONST( "BThWlanCoexistParamsnumberQuietCycle" );
NDIS_STRING STRBThWlanCoexistParamsmaxNumCts    = NDIS_STRING_CONST( "BThWlanCoexistParamsmaxNumCts" );
NDIS_STRING STRBThWlanCoexistParamsnumberOfWlanPackets          = NDIS_STRING_CONST( "BThWlanCoexistParamsnumberOfWlanPackets" );
NDIS_STRING STRBThWlanCoexistParamsnumberOfBtPackets            = NDIS_STRING_CONST( "BThWlanCoexistParamsnumberOfBtPackets" );
NDIS_STRING STRBThWlanCoexistParamsnumberOfMissedRxForAvalancheTrigger          = NDIS_STRING_CONST( "BThWlanCoexistParamsnumberOfMissedRxForAvalancheTrigger" );
NDIS_STRING STRBThWlanCoexistParamswlanElpHpSupport         = NDIS_STRING_CONST( "BThWlanCoexistParamswlanElpHpSupport" );
NDIS_STRING STRBThWlanCoexistParamsbtAntiStarvationPeriod           = NDIS_STRING_CONST( "BThWlanCoexistParamsbtAntiStarvationPeriod" );
NDIS_STRING STRBThWlanCoexistParamsbtAntiStarvationNumberOfCyclesWithinThePeriod            = NDIS_STRING_CONST( "BThWlanCoexistParamsbtAntiStarvationNumberOfCyclesWithinThePeriod" );
NDIS_STRING STRBThWlanCoexistParamsackModeDuringBtLpInDualAnt            = NDIS_STRING_CONST( "BThWlanCoexistParamsackModeDuringBtLpInDualAnt" );
NDIS_STRING STRBThWlanCoexistParamsallowPaSdToggleDuringBtActivityEnable = NDIS_STRING_CONST( "BThWlanCoexistParamsallowPaSdToggleDuringBtActivityEnable" );
NDIS_STRING STRBThWlanCoexistParamswakeUpTimeBeforeBeacon            = NDIS_STRING_CONST( "BThWlanCoexistParamswakeUpTimeBeforeBeacon" );

NDIS_STRING STRBThWlanCoexistParamshpdmMaxGuardTime = NDIS_STRING_CONST( "BThWlanCoexistParamshpdmMaxGuardTime" );
NDIS_STRING STRBThWlanCoexistParamstimeoutNextWlanPacket = NDIS_STRING_CONST( "BThWlanCoexistParamstimeoutNextWlanPacket" );
NDIS_STRING STRBThWlanCoexistParamssgAutoModeNoCts = NDIS_STRING_CONST( "BThWlanCoexistParamssgAutoModeNoCts" );
NDIS_STRING STRBThWlanCoexistParamsnumOfBtHpRespectedReq = NDIS_STRING_CONST( "BThWlanCoexistParamsnumOfBtHpRespectedReq" );
NDIS_STRING STRBThWlanCoexistParamswlanRxMinRateToRespectBtHp = NDIS_STRING_CONST( "BThWlanCoexistParamswlanRxMinRateToRespectBtHp" );

NDIS_STRING STRBThWlanCoexistScanNumberOfProbes                 = NDIS_STRING_CONST( "BThWlanCoexistScanNumberOfProbes" );
NDIS_STRING STRBThWlanCoexistScanCompensationPercent            = NDIS_STRING_CONST( "BThWlanCoexistScanCompensationPercent" );
NDIS_STRING STRBThWlanCoexistScanCompensationMaxTime            = NDIS_STRING_CONST( "BThWlanCoexistScanCompensationMaxTime" );
NDIS_STRING STRBThWlanCoexistBSSLossCompensationPercent         = NDIS_STRING_CONST( "BThWlanCoexistBSSLossCompensationPercent" );

NDIS_STRING STRDisableSsidPending           = NDIS_STRING_CONST( "DisableSsidPending" );

/*-----------------------------------*/
/*   SME Init Params                 */
/*-----------------------------------*/
NDIS_STRING STRdot11SmeScanEnabled          = NDIS_STRING_CONST( "FirstConnScanEnabled" );
NDIS_STRING STRdot11SmeInterScanMin         = NDIS_STRING_CONST( "FirstConnInterScanMinTimout" );
NDIS_STRING STRdot11SmeInterScanMax         = NDIS_STRING_CONST( "FirstConnInterScanMaxTimout" );
NDIS_STRING STRdot11SmeInterScanDelta       = NDIS_STRING_CONST( "FirstConnInterScanDeltaTimout" );

/*      SME B/G Scan Params             */
NDIS_STRING STRdot11SmeScanBGChannelList    = NDIS_STRING_CONST( "FirstConnScanBandB_ChannelList" );
NDIS_STRING STRdot11SmeScanBGMinDwellTime   = NDIS_STRING_CONST( "FirstConnScanBandB_MinDwellTime" );
NDIS_STRING STRdot11SmeScanBGMaxDwellTime   = NDIS_STRING_CONST( "FirstConnScanBandB_MaxDwellTime" );
NDIS_STRING STRdot11SmeScanBGNumProbReq     = NDIS_STRING_CONST( "FirstConnScanBandB_NumOfProbReqs" );
NDIS_STRING STRdot11SmeScanBGProbReqRate    = NDIS_STRING_CONST( "FirstConnScanBandB_ProbReqRate" );
NDIS_STRING STRdot11SmeScanBGTxPowerLevel   = NDIS_STRING_CONST( "FirstConnScanBandB_TxPowerLevel" );

/*      SME A Scan Params           */
NDIS_STRING STRdot11SmeScanAChannelList     = NDIS_STRING_CONST( "FirstConnScanBandA_ChannelList" );
NDIS_STRING STRdot11SmeScanAMinDwellTime    = NDIS_STRING_CONST( "FirstConnScanBandA_MinDwellTime" );
NDIS_STRING STRdot11SmeScanAMaxDwellTime    = NDIS_STRING_CONST( "FirstConnScanBandA_MaxDwellTime" );
NDIS_STRING STRdot11SmeScanANumProbReq      = NDIS_STRING_CONST( "FirstConnScanBandA_NumOfProbReqs" );
NDIS_STRING STRdot11SmeScanAProbReqRate     = NDIS_STRING_CONST( "FirstConnScanBandA_ProbReqRate" );
NDIS_STRING STRdot11SmeScanATxPowerLevel    = NDIS_STRING_CONST( "FirstConnScanBandA_TxPowerLevel" );


/*-----------------------------------*/
/*   Health Check Init Params        */
/*-----------------------------------*/
NDIS_STRING STRHealthMonitorCheckPeriod         = NDIS_STRING_CONST( "HealthMonitorCheckPeriod" );
NDIS_STRING STRRecoveryEnabledNoScanComplete    = NDIS_STRING_CONST( "RecoveryEnabledNoScanComplete" );
NDIS_STRING STRRecoveryEnabledMboxFailure       = NDIS_STRING_CONST( "RecoveryEnabledMboxFailure" );
NDIS_STRING STRRecoveryEnabledHwAwakeFailure    = NDIS_STRING_CONST( "RecoveryEnabledHwAwakeFailure" );
NDIS_STRING STRRecoveryEnabledBusError          = NDIS_STRING_CONST( "RecoveryEnabledBusError" );
NDIS_STRING STRRecoveryEnabledDeviceError       = NDIS_STRING_CONST( "RecoveryEnabledDeviceError" );
NDIS_STRING STRRecoveryEnabledTxStuck           = NDIS_STRING_CONST( "RecoveryEnabledTxStuck" );
NDIS_STRING STRRecoveryEnabledDisconnectTimeout = NDIS_STRING_CONST( "RecoveryEnabledDisconnectTimeout" );
NDIS_STRING STRRecoveryEnabledPowerSaveFailure  = NDIS_STRING_CONST( "RecoveryEnabledPowerSaveFailure" );
NDIS_STRING STRRecoveryEnabledMeasurementFailure= NDIS_STRING_CONST( "RecoveryEnabledMeasurementFailure" );

/*-----------------------------------*/
/*   Hardware ACI recovery           */
/*-----------------------------------*/
NDIS_STRING STRHardwareACIMode                  = NDIS_STRING_CONST("HardwareACIMode" );
NDIS_STRING STRHardwareACIInputCCA              = NDIS_STRING_CONST("HardwareACIInputCCA" );
NDIS_STRING STRHardwareACIQualifiedCCA          = NDIS_STRING_CONST("HardwareACIQualifiedCCA" );
NDIS_STRING STRHardwareACIStompForRx            = NDIS_STRING_CONST("HardwareACIStompForRx" );
NDIS_STRING STRHardwareACIStompForTx            = NDIS_STRING_CONST("HardwareACIStompForTx" );
NDIS_STRING STRHardwareACITxCCA                 = NDIS_STRING_CONST("HardwareACITxCCA" );

/*-----------------------------------*/
/* Tx Power control     */
/*-----------------------------------*/
NDIS_STRING STRTxPowerCheckTime                 = NDIS_STRING_CONST("TxPowerCheckTime");
NDIS_STRING STRTxPowerControlOn                 = NDIS_STRING_CONST("TxPowerControlOn");
NDIS_STRING STRTxPowerRssiThresh                = NDIS_STRING_CONST("TxPowerRssiThresh");
NDIS_STRING STRTxPowerRssiRestoreThresh         = NDIS_STRING_CONST("TxPowerRssiRestoreThresh");
NDIS_STRING STRTxPowerTempRecover              = NDIS_STRING_CONST("TxPowerTempRecover");


/*-----------------------------------*/
/*-----------------------------------*/
/*        QOS Parameters             */
/*-----------------------------------*/
NDIS_STRING STRWMEEnable                        = NDIS_STRING_CONST("WME_Enable");
NDIS_STRING STRTrafficAdmCtrlEnable             = NDIS_STRING_CONST("TrafficAdmCtrl_Enable");
NDIS_STRING STRdesiredPsMode                    = NDIS_STRING_CONST("desiredPsMode");
NDIS_STRING STRQOSmsduLifeTimeBE                = NDIS_STRING_CONST("QOS_msduLifeTimeBE");
NDIS_STRING STRQOSmsduLifeTimeBK                = NDIS_STRING_CONST("QOS_msduLifeTimeBK");
NDIS_STRING STRQOSmsduLifeTimeVI                = NDIS_STRING_CONST("QOS_msduLifeTimeVI");
NDIS_STRING STRQOSmsduLifeTimeVO                = NDIS_STRING_CONST("QOS_msduLifeTimeVO");
NDIS_STRING STRQOSrxTimeOutPsPoll               = NDIS_STRING_CONST("QOS_rxTimeoutPsPoll");
NDIS_STRING STRQOSrxTimeOutUPSD                 = NDIS_STRING_CONST("QOS_rxTimeoutUPSD");
NDIS_STRING STRQOStxQueue0Size                  = NDIS_STRING_CONST("QOS_txQueue0Size");
NDIS_STRING STRQOStxQueue1Size                  = NDIS_STRING_CONST("QOS_txQueue1Size");
NDIS_STRING STRQOStxQueue2Size                  = NDIS_STRING_CONST("QOS_txQueue2Size");
NDIS_STRING STRQOStxQueue3Size                  = NDIS_STRING_CONST("QOS_txQueue3Size");
NDIS_STRING STRQOSwmePsModeBE                   = NDIS_STRING_CONST("QOS_wmePsModeBE");
NDIS_STRING STRQOSwmePsModeBK                   = NDIS_STRING_CONST("QOS_wmePsModeBK");
NDIS_STRING STRQOSwmePsModeVI                   = NDIS_STRING_CONST("QOS_wmePsModeVI");
NDIS_STRING STRQOSwmePsModeVO                   = NDIS_STRING_CONST("QOS_wmePsModeVO");
NDIS_STRING STRQOSShortRetryLimitBE             = NDIS_STRING_CONST("QOS_ShortRetryLimitBE");
NDIS_STRING STRQOSShortRetryLimitBK             = NDIS_STRING_CONST("QOS_ShortRetryLimitBK");
NDIS_STRING STRQOSShortRetryLimitVI             = NDIS_STRING_CONST("QOS_ShortRetryLimitVI");
NDIS_STRING STRQOSShortRetryLimitVO             = NDIS_STRING_CONST("QOS_ShortRetryLimitVO");
NDIS_STRING STRQOSLongRetryLimitBE              = NDIS_STRING_CONST("QOS_LongRetryLimitBE");
NDIS_STRING STRQOSLongRetryLimitBK              = NDIS_STRING_CONST("QOS_LongRetryLimitBK");
NDIS_STRING STRQOSLongRetryLimitVI              = NDIS_STRING_CONST("QOS_LongRetryLimitVI");
NDIS_STRING STRQOSLongRetryLimitVO              = NDIS_STRING_CONST("QOS_LongRetryLimitVO");

NDIS_STRING STRQOSAckPolicyBE                   = NDIS_STRING_CONST("QOS_AckPolicyBE");
NDIS_STRING STRQOSAckPolicyBK                   = NDIS_STRING_CONST("QOS_AckPolicyBK");
NDIS_STRING STRQOSAckPolicyVI                   = NDIS_STRING_CONST("QOS_AckPolicyVI");
NDIS_STRING STRQOSAckPolicyVO                   = NDIS_STRING_CONST("QOS_AckPolicyVO");
NDIS_STRING STRQoSqueue0OverFlowPolicy          = NDIS_STRING_CONST("QOS_queue0OverFlowPolicy");
NDIS_STRING STRQoSqueue1OverFlowPolicy          = NDIS_STRING_CONST("QOS_queue1OverFlowPolicy");
NDIS_STRING STRQoSqueue2OverFlowPolicy          = NDIS_STRING_CONST("QOS_queue2OverFlowPolicy");
NDIS_STRING STRQoSqueue3OverFlowPolicy          = NDIS_STRING_CONST("QOS_queue3OverFlowPolicy");



/* HW Tx queues buffers allocation thresholds */
NDIS_STRING STRQOStxBlksHighPrcntBE             = NDIS_STRING_CONST("QOS_txBlksHighPrcntBE");
NDIS_STRING STRQOStxBlksHighPrcntBK             = NDIS_STRING_CONST("QOS_txBlksHighPrcntBK");
NDIS_STRING STRQOStxBlksHighPrcntVI             = NDIS_STRING_CONST("QOS_txBlksHighPrcntVI");
NDIS_STRING STRQOStxBlksHighPrcntVO             = NDIS_STRING_CONST("QOS_txBlksHighPrcntVO");
NDIS_STRING STRQOStxBlksLowPrcntBE              = NDIS_STRING_CONST("QOS_txBlksLowPrcntBE");
NDIS_STRING STRQOStxBlksLowPrcntBK              = NDIS_STRING_CONST("QOS_txBlksLowPrcntBK");
NDIS_STRING STRQOStxBlksLowPrcntVI              = NDIS_STRING_CONST("QOS_txBlksLowPrcntVI");
NDIS_STRING STRQOStxBlksLowPrcntVO              = NDIS_STRING_CONST("QOS_txBlksLowPrcntVO");

/* Traffic Intensity parameters*/
NDIS_STRING STRTrafficIntensityThresHigh        = NDIS_STRING_CONST("TrafficIntensityThresHigh");
NDIS_STRING STRTrafficIntensityThresLow         = NDIS_STRING_CONST("TrafficIntensityThresLow");
NDIS_STRING STRTrafficIntensityTestInterval     = NDIS_STRING_CONST("TrafficIntensityTestInterval");
NDIS_STRING STRTrafficIntensityThresholdEnabled = NDIS_STRING_CONST("TrafficIntensityThresholdEnabled");
NDIS_STRING STRTrafficMonitorMinIntervalPercentage = NDIS_STRING_CONST("TrafficMonitorMinIntervalPercent");


/* Packet Burst parameters */
NDIS_STRING STRQOSPacketBurstEnable             = NDIS_STRING_CONST("QOS_PacketBurstEnable");
NDIS_STRING STRQOSPacketBurstTxOpLimit          = NDIS_STRING_CONST("QOS_PacketBurstTxOpLimit");

/*-----------------------------------*/
/*        QOS classifier Parameters  */
/*-----------------------------------*/
NDIS_STRING STRClsfr_Type                       = NDIS_STRING_CONST("Clsfr_Type");
NDIS_STRING STRNumOfCodePoints                  = NDIS_STRING_CONST("NumOfCodePoints");
NDIS_STRING STRNumOfDstPortClassifiers          = NDIS_STRING_CONST("NumOfDstPortClassifiers");
NDIS_STRING STRNumOfDstIPPortClassifiers        = NDIS_STRING_CONST("NumOfDstIPPortClassifiers");

NDIS_STRING STRDSCPClassifier00_CodePoint       = NDIS_STRING_CONST("DSCPClassifier00_CodePoint");
NDIS_STRING STRDSCPClassifier01_CodePoint       = NDIS_STRING_CONST("DSCPClassifier01_CodePoint");
NDIS_STRING STRDSCPClassifier02_CodePoint       = NDIS_STRING_CONST("DSCPClassifier02_CodePoint");
NDIS_STRING STRDSCPClassifier03_CodePoint       = NDIS_STRING_CONST("DSCPClassifier03_CodePoint");
NDIS_STRING STRDSCPClassifier04_CodePoint       = NDIS_STRING_CONST("DSCPClassifier04_CodePoint");
NDIS_STRING STRDSCPClassifier05_CodePoint       = NDIS_STRING_CONST("DSCPClassifier05_CodePoint");
NDIS_STRING STRDSCPClassifier06_CodePoint       = NDIS_STRING_CONST("DSCPClassifier06_CodePoint");
NDIS_STRING STRDSCPClassifier07_CodePoint       = NDIS_STRING_CONST("DSCPClassifier07_CodePoint");
NDIS_STRING STRDSCPClassifier08_CodePoint       = NDIS_STRING_CONST("DSCPClassifier08_CodePoint");
NDIS_STRING STRDSCPClassifier09_CodePoint       = NDIS_STRING_CONST("DSCPClassifier09_CodePoint");
NDIS_STRING STRDSCPClassifier10_CodePoint       = NDIS_STRING_CONST("DSCPClassifier10_CodePoint");
NDIS_STRING STRDSCPClassifier11_CodePoint       = NDIS_STRING_CONST("DSCPClassifier11_CodePoint");
NDIS_STRING STRDSCPClassifier12_CodePoint       = NDIS_STRING_CONST("DSCPClassifier12_CodePoint");
NDIS_STRING STRDSCPClassifier13_CodePoint       = NDIS_STRING_CONST("DSCPClassifier13_CodePoint");
NDIS_STRING STRDSCPClassifier14_CodePoint       = NDIS_STRING_CONST("DSCPClassifier14_CodePoint");
NDIS_STRING STRDSCPClassifier15_CodePoint       = NDIS_STRING_CONST("DSCPClassifier15_CodePoint");

NDIS_STRING STRDSCPClassifier00_DTag        = NDIS_STRING_CONST("DSCPClassifier00_DTag");
NDIS_STRING STRDSCPClassifier01_DTag        = NDIS_STRING_CONST("DSCPClassifier01_DTag");
NDIS_STRING STRDSCPClassifier02_DTag        = NDIS_STRING_CONST("DSCPClassifier02_DTag");
NDIS_STRING STRDSCPClassifier03_DTag        = NDIS_STRING_CONST("DSCPClassifier03_DTag");
NDIS_STRING STRDSCPClassifier04_DTag        = NDIS_STRING_CONST("DSCPClassifier04_DTag");
NDIS_STRING STRDSCPClassifier05_DTag        = NDIS_STRING_CONST("DSCPClassifier05_DTag");
NDIS_STRING STRDSCPClassifier06_DTag        = NDIS_STRING_CONST("DSCPClassifier06_DTag");
NDIS_STRING STRDSCPClassifier07_DTag        = NDIS_STRING_CONST("DSCPClassifier07_DTag");
NDIS_STRING STRDSCPClassifier08_DTag        = NDIS_STRING_CONST("DSCPClassifier08_DTag");
NDIS_STRING STRDSCPClassifier09_DTag        = NDIS_STRING_CONST("DSCPClassifier09_DTag");
NDIS_STRING STRDSCPClassifier10_DTag        = NDIS_STRING_CONST("DSCPClassifier10_DTag");
NDIS_STRING STRDSCPClassifier11_DTag        = NDIS_STRING_CONST("DSCPClassifier11_DTag");
NDIS_STRING STRDSCPClassifier12_DTag        = NDIS_STRING_CONST("DSCPClassifier12_DTag");
NDIS_STRING STRDSCPClassifier13_DTag        = NDIS_STRING_CONST("DSCPClassifier13_DTag");
NDIS_STRING STRDSCPClassifier14_DTag        = NDIS_STRING_CONST("DSCPClassifier14_DTag");
NDIS_STRING STRDSCPClassifier15_DTag        = NDIS_STRING_CONST("DSCPClassifier15_DTag");


NDIS_STRING STRPortClassifier00_Port            = NDIS_STRING_CONST("PortClassifier00_Port");
NDIS_STRING STRPortClassifier01_Port            = NDIS_STRING_CONST("PortClassifier01_Port");
NDIS_STRING STRPortClassifier02_Port            = NDIS_STRING_CONST("PortClassifier02_Port");
NDIS_STRING STRPortClassifier03_Port            = NDIS_STRING_CONST("PortClassifier03_Port");
NDIS_STRING STRPortClassifier04_Port            = NDIS_STRING_CONST("PortClassifier04_Port");
NDIS_STRING STRPortClassifier05_Port            = NDIS_STRING_CONST("PortClassifier05_Port");
NDIS_STRING STRPortClassifier06_Port            = NDIS_STRING_CONST("PortClassifier06_Port");
NDIS_STRING STRPortClassifier07_Port            = NDIS_STRING_CONST("PortClassifier07_Port");
NDIS_STRING STRPortClassifier08_Port            = NDIS_STRING_CONST("PortClassifier08_Port");
NDIS_STRING STRPortClassifier09_Port            = NDIS_STRING_CONST("PortClassifier09_Port");
NDIS_STRING STRPortClassifier10_Port            = NDIS_STRING_CONST("PortClassifier10_Port");
NDIS_STRING STRPortClassifier11_Port            = NDIS_STRING_CONST("PortClassifier11_Port");
NDIS_STRING STRPortClassifier12_Port            = NDIS_STRING_CONST("PortClassifier12_Port");
NDIS_STRING STRPortClassifier13_Port            = NDIS_STRING_CONST("PortClassifier13_Port");
NDIS_STRING STRPortClassifier14_Port            = NDIS_STRING_CONST("PortClassifier14_Port");
NDIS_STRING STRPortClassifier15_Port            = NDIS_STRING_CONST("PortClassifier15_Port");

NDIS_STRING STRPortClassifier00_DTag            = NDIS_STRING_CONST("PortClassifier00_DTag");
NDIS_STRING STRPortClassifier01_DTag            = NDIS_STRING_CONST("PortClassifier01_DTag");
NDIS_STRING STRPortClassifier02_DTag            = NDIS_STRING_CONST("PortClassifier02_DTag");
NDIS_STRING STRPortClassifier03_DTag            = NDIS_STRING_CONST("PortClassifier03_DTag");
NDIS_STRING STRPortClassifier04_DTag            = NDIS_STRING_CONST("PortClassifier04_DTag");
NDIS_STRING STRPortClassifier05_DTag            = NDIS_STRING_CONST("PortClassifier05_DTag");
NDIS_STRING STRPortClassifier06_DTag            = NDIS_STRING_CONST("PortClassifier06_DTag");
NDIS_STRING STRPortClassifier07_DTag            = NDIS_STRING_CONST("PortClassifier07_DTag");
NDIS_STRING STRPortClassifier08_DTag            = NDIS_STRING_CONST("PortClassifier08_DTag");
NDIS_STRING STRPortClassifier09_DTag            = NDIS_STRING_CONST("PortClassifier09_DTag");
NDIS_STRING STRPortClassifier10_DTag            = NDIS_STRING_CONST("PortClassifier10_DTag");
NDIS_STRING STRPortClassifier11_DTag            = NDIS_STRING_CONST("PortClassifier11_DTag");
NDIS_STRING STRPortClassifier12_DTag            = NDIS_STRING_CONST("PortClassifier12_DTag");
NDIS_STRING STRPortClassifier13_DTag            = NDIS_STRING_CONST("PortClassifier13_DTag");
NDIS_STRING STRPortClassifier14_DTag            = NDIS_STRING_CONST("PortClassifier14_DTag");
NDIS_STRING STRPortClassifier15_DTag            = NDIS_STRING_CONST("PortClassifier15_DTag");

NDIS_STRING STRIPPortClassifier00_IPAddress     = NDIS_STRING_CONST("IPPortClassifier00_IPAddress");
NDIS_STRING STRIPPortClassifier01_IPAddress     = NDIS_STRING_CONST("IPPortClassifier01_IPAddress");
NDIS_STRING STRIPPortClassifier02_IPAddress     = NDIS_STRING_CONST("IPPortClassifier02_IPAddress");
NDIS_STRING STRIPPortClassifier03_IPAddress     = NDIS_STRING_CONST("IPPortClassifier03_IPAddress");
NDIS_STRING STRIPPortClassifier04_IPAddress     = NDIS_STRING_CONST("IPPortClassifier04_IPAddress");
NDIS_STRING STRIPPortClassifier05_IPAddress     = NDIS_STRING_CONST("IPPortClassifier05_IPAddress");
NDIS_STRING STRIPPortClassifier06_IPAddress     = NDIS_STRING_CONST("IPPortClassifier06_IPAddress");
NDIS_STRING STRIPPortClassifier07_IPAddress     = NDIS_STRING_CONST("IPPortClassifier07_IPAddress");
NDIS_STRING STRIPPortClassifier08_IPAddress     = NDIS_STRING_CONST("IPPortClassifier08_IPAddress");
NDIS_STRING STRIPPortClassifier09_IPAddress     = NDIS_STRING_CONST("IPPortClassifier09_IPAddress");
NDIS_STRING STRIPPortClassifier10_IPAddress     = NDIS_STRING_CONST("IPPortClassifier10_IPAddress");
NDIS_STRING STRIPPortClassifier11_IPAddress     = NDIS_STRING_CONST("IPPortClassifier11_IPAddress");
NDIS_STRING STRIPPortClassifier12_IPAddress     = NDIS_STRING_CONST("IPPortClassifier12_IPAddress");
NDIS_STRING STRIPPortClassifier13_IPAddress     = NDIS_STRING_CONST("IPPortClassifier13_IPAddress");
NDIS_STRING STRIPPortClassifier14_IPAddress     = NDIS_STRING_CONST("IPPortClassifier14_IPAddress");
NDIS_STRING STRIPPortClassifier15_IPAddress     = NDIS_STRING_CONST("IPPortClassifier15_IPAddress");

NDIS_STRING STRIPPortClassifier00_Port          = NDIS_STRING_CONST("IPPortClassifier00_Port");
NDIS_STRING STRIPPortClassifier01_Port          = NDIS_STRING_CONST("IPPortClassifier01_Port");
NDIS_STRING STRIPPortClassifier02_Port          = NDIS_STRING_CONST("IPPortClassifier02_Port");
NDIS_STRING STRIPPortClassifier03_Port          = NDIS_STRING_CONST("IPPortClassifier03_Port");
NDIS_STRING STRIPPortClassifier04_Port          = NDIS_STRING_CONST("IPPortClassifier04_Port");
NDIS_STRING STRIPPortClassifier05_Port          = NDIS_STRING_CONST("IPPortClassifier05_Port");
NDIS_STRING STRIPPortClassifier06_Port          = NDIS_STRING_CONST("IPPortClassifier06_Port");
NDIS_STRING STRIPPortClassifier07_Port          = NDIS_STRING_CONST("IPPortClassifier07_Port");
NDIS_STRING STRIPPortClassifier08_Port          = NDIS_STRING_CONST("IPPortClassifier08_Port");
NDIS_STRING STRIPPortClassifier09_Port          = NDIS_STRING_CONST("IPPortClassifier09_Port");
NDIS_STRING STRIPPortClassifier10_Port          = NDIS_STRING_CONST("IPPortClassifier10_Port");
NDIS_STRING STRIPPortClassifier11_Port          = NDIS_STRING_CONST("IPPortClassifier11_Port");
NDIS_STRING STRIPPortClassifier12_Port          = NDIS_STRING_CONST("IPPortClassifier12_Port");
NDIS_STRING STRIPPortClassifier13_Port          = NDIS_STRING_CONST("IPPortClassifier13_Port");
NDIS_STRING STRIPPortClassifier14_Port          = NDIS_STRING_CONST("IPPortClassifier14_Port");
NDIS_STRING STRIPPortClassifier15_Port          = NDIS_STRING_CONST("IPPortClassifier15_Port");

NDIS_STRING STRIPPortClassifier00_DTag          = NDIS_STRING_CONST("IPPortClassifier00_DTag");
NDIS_STRING STRIPPortClassifier01_DTag          = NDIS_STRING_CONST("IPPortClassifier01_DTag");
NDIS_STRING STRIPPortClassifier02_DTag          = NDIS_STRING_CONST("IPPortClassifier02_DTag");
NDIS_STRING STRIPPortClassifier03_DTag          = NDIS_STRING_CONST("IPPortClassifier03_DTag");
NDIS_STRING STRIPPortClassifier04_DTag          = NDIS_STRING_CONST("IPPortClassifier04_DTag");
NDIS_STRING STRIPPortClassifier05_DTag          = NDIS_STRING_CONST("IPPortClassifier05_DTag");
NDIS_STRING STRIPPortClassifier06_DTag          = NDIS_STRING_CONST("IPPortClassifier06_DTag");
NDIS_STRING STRIPPortClassifier07_DTag          = NDIS_STRING_CONST("IPPortClassifier07_DTag");
NDIS_STRING STRIPPortClassifier08_DTag          = NDIS_STRING_CONST("IPPortClassifier08_DTag");
NDIS_STRING STRIPPortClassifier09_DTag          = NDIS_STRING_CONST("IPPortClassifier09_DTag");
NDIS_STRING STRIPPortClassifier10_DTag          = NDIS_STRING_CONST("IPPortClassifier10_DTag");
NDIS_STRING STRIPPortClassifier11_DTag          = NDIS_STRING_CONST("IPPortClassifier11_DTag");
NDIS_STRING STRIPPortClassifier12_DTag          = NDIS_STRING_CONST("IPPortClassifier12_DTag");
NDIS_STRING STRIPPortClassifier13_DTag          = NDIS_STRING_CONST("IPPortClassifier13_DTag");
NDIS_STRING STRIPPortClassifier14_DTag          = NDIS_STRING_CONST("IPPortClassifier14_DTag");
NDIS_STRING STRIPPortClassifier15_DTag          = NDIS_STRING_CONST("IPPortClassifier15_DTag");

/*-----------------------------
   Rx Data Filter parameters
-----------------------------*/
NDIS_STRING STRRxDataFiltersEnabled             = NDIS_STRING_CONST("RxDataFilters_Enabled");
NDIS_STRING STRRxDataFiltersDefaultAction       = NDIS_STRING_CONST("RxDataFilters_DefaultAction");

NDIS_STRING STRRxDataFiltersFilter1Offset       = NDIS_STRING_CONST("RxDataFilters_Filter1Offset");
NDIS_STRING STRRxDataFiltersFilter1Mask         = NDIS_STRING_CONST("RxDataFilters_Filter1Mask");
NDIS_STRING STRRxDataFiltersFilter1Pattern      = NDIS_STRING_CONST("RxDataFilters_Filter1Pattern");

NDIS_STRING STRRxDataFiltersFilter2Offset       = NDIS_STRING_CONST("RxDataFilters_Filter2Offset");
NDIS_STRING STRRxDataFiltersFilter2Mask         = NDIS_STRING_CONST("RxDataFilters_Filter2Mask");
NDIS_STRING STRRxDataFiltersFilter2Pattern      = NDIS_STRING_CONST("RxDataFilters_Filter2Pattern");

NDIS_STRING STRRxDataFiltersFilter3Offset       = NDIS_STRING_CONST("RxDataFilters_Filter3Offset");
NDIS_STRING STRRxDataFiltersFilter3Mask         = NDIS_STRING_CONST("RxDataFilters_Filter3Mask");
NDIS_STRING STRRxDataFiltersFilter3Pattern      = NDIS_STRING_CONST("RxDataFilters_Filter3Pattern");

NDIS_STRING STRRxDataFiltersFilter4Offset       = NDIS_STRING_CONST("RxDataFilters_Filter4Offset");
NDIS_STRING STRRxDataFiltersFilter4Mask         = NDIS_STRING_CONST("RxDataFilters_Filter4Mask");
NDIS_STRING STRRxDataFiltersFilter4Pattern      = NDIS_STRING_CONST("RxDataFilters_Filter4Pattern");


/*---------------------------
    Measurement parameters
-----------------------------*/
NDIS_STRING STRMeasurTrafficThreshold           = NDIS_STRING_CONST( "MeasurTrafficThreshold" );
NDIS_STRING STRMeasurMaxDurationOnNonServingChannel = NDIS_STRING_CONST( "MeasurMaxDurationOnNonServingChannel" );

/*---------------------------
      EXC Manager parameters
-----------------------------*/
#ifdef EXC_MODULE_INCLUDED
NDIS_STRING STRExcModeEnabled                   = NDIS_STRING_CONST( "ExcModeEnabled" );
#endif

NDIS_STRING STRExcTestIgnoreDeAuth0             = NDIS_STRING_CONST( "ExcTestRogeAP" );

/*-----------------------------------*/
/*   EEPROM-less support             */
/*-----------------------------------*/
NDIS_STRING STREEPROMlessModeSupported          = NDIS_STRING_CONST( "EEPROMlessModeSupported" );
NDIS_STRING STRstationMacAddress                = NDIS_STRING_CONST("dot11StationID");


/*-----------------------------------*/
/*   INI file to configuration       */
/*-----------------------------------*/

NDIS_STRING SendINIBufferToUser                 = NDIS_STRING_CONST("SendINIBufferToUserMode");

void regConvertStringtoMACAddress(UINT8 *staMACAddressString,UINT8 *MacAddressArray);
void regConvertStringtoIpAddress(UINT8 *staIpAddressString,UINT8 *IpAddressArray);
void regConvertStringtoBeaconIETable(UINT8 *staIpAddressString,UINT8 *BeaconIEArray, UINT8 size);

/*-----------------------------------*/
/*   Scan concentrator parameters    */
/*-----------------------------------*/
NDIS_STRING STRPassiveScanDwellTime      = NDIS_STRING_CONST( "DriverPassiveScanDefaultDwellTime" );
// TRS: Scan changes from TI
NDIS_STRING STRMinimumDurationBetweenOidScans = NDIS_STRING_CONST( "MinimumDurationBetweenOidScans" );
//TRS: end of Scan changes from TI

/*
NDIS_STRING STRPctACXRxMemoryPool       = NDIS_STRING_CONST( "PctACXRxMemoryPool" );
NDIS_STRING STRSendPacketsPerOID        = NDIS_STRING_CONST( "Dot11SendPacketsPerOID" );
NDIS_STRING STRFragCacheSize            = NDIS_STRING_CONST( "FragCacheSize" );
*/

static int decryptWEP( PCHAR pSrc, PCHAR pDst, ULONG len);
short _btoi ( char *sptr, short slen, int *pi, short base );
static VOID initRadioValusFromRgstryString(  PCHAR pSrc,  PCHAR pDst,  ULONG len);



static void readRates(PTIWLN_ADAPTER_T pAdapter, initTable_t *pInitTable);
static void decryptScanControlTable(PUCHAR src, PUCHAR dst, USHORT len);

static UINT32 regReadIntegerTable(PTIWLN_ADAPTER_T  pAdapter,
                                PNDIS_STRING        pParameterName,
                                PCHAR               pDefaultValue,
                                USHORT              defaultLen,
                                PUCHAR              pParameter);

static void assignRegValue(PULONG lValue, PNDIS_CONFIGURATION_PARAMETER ndisParameter);

static void parse_filter_request(rxDataFilterRequest_t * request, UINT8 offset, char * mask, UINT8 maskLength, char * pattern, UINT8 patternLength);

/* ---------------------------------------------------------------------------*/
/* Converts a string to a signed int. Assumes base 10. Assumes positive*/
/*    number*/
/**/
/* Returns value on success, -1 on failure*/
/**/
/* ---------------------------------------------------------------------------*/
ULONG
tiwlnstrtoi(char *num, UINT length)
{
  ULONG value;

  if(num == NULL || length == 0 )
  {
    return 0;
  }

  for(value=0;length&&*num;num++,length--)
  {
    if(*num<='9'&&*num>= '0')
    {
      value=(value*10)+(*num - '0');
    }
    else { /* Out of range*/
      break;
    }
  }
  return value;
}



/*-----------------------------------------------------------------------------

Routine Name:

    regReadConfigString

Routine Description:


Arguments:


Return Value:

    None

-----------------------------------------------------------------------------*/
BOOLEAN
regReadConfigString(
    IN NDIS_HANDLE hConfig,
    IN NDIS_STRING *pParmName,
    OUT PCHAR pStringOut,
    OUT USHORT cbString
    )
{

    BOOLEAN fRC = FALSE;
    NDIS_STATUS rc;
    PNDIS_CONFIGURATION_PARAMETER pParameter;

    ANSI_STRING strAnsi;

    do {

        /* Read the string...*/
        NdisReadConfiguration( &rc, &pParameter, hConfig, pParmName,
            NdisParameterString );

        if ( NDIS_STATUS_SUCCESS != rc ) break;

        /* convert the string...*/
        strAnsi.Length        = 0;
        strAnsi.MaximumLength = cbString;
        strAnsi.Buffer        = pStringOut;

        rc = NdisUnicodeStringToAnsiString(&strAnsi,
            &pParameter->ParameterData.StringData);

        if ( NDIS_STATUS_SUCCESS != rc ) break;

        /* NULL terminate the output string.  If we cannot fit the NULL, that's*/
        /*  an error...*/
        if ( cbString < strAnsi.Length + 1 ) break;

        pStringOut[ strAnsi.Length ] = '\0';

        fRC = TRUE;

    } while( 0 );

    return( fRC );
}


/*-----------------------------------------------------------------------------

Routine Name:

    regReadRatesConfigArray

Routine Description:


Arguments:


Return Value:

    None

-----------------------------------------------------------------------------*/
BOOLEAN
regReadRatesConfigArray(
    NDIS_HANDLE SubKeyHandle,
    PNDIS_STRING Str,
    PUCHAR target,
    PUCHAR def,
    PULONG size
    )
{
    BOOLEAN fRC = TRUE;
    CHAR str[200];
    UCHAR val;
    int i,j;

    if (!regReadConfigString(SubKeyHandle, Str, (PCHAR)str, 200) ) {

        fRC = FALSE;

    }
    i=j=0;
    do
    {
        val = 0;
        if (!fRC) {
            val = def[i];
            if(!val) break;
        }
        else
        {
            if(!str[j])
                break;

            while (str[j] && (str[j]<'0' || str[j]>'9')) j++;
            if (str[j])
            {
                while (str[j] && str[j]>='0' && str[j]<='9')
                {
                    val=10*val+str[j]-'0';
                    j++;
                }
            }
        }
        target[i++] = val;
    }
    while (val);
    *size = i;

    return fRC;
}


/*-----------------------------------------------------------------------------

Routine Name:

    convertRatesFromRegistryFormatToDriver

Routine Description:


Arguments:


Return Value:

    None

-----------------------------------------------------------------------------*/
rate_e  convertRatesFromRegistryFormatToDriver(UCHAR rateIn)
{
    switch(rateIn)
    {
    case 0x1:   return DRV_RATE_1M;
    case 0x2:   return DRV_RATE_2M;
    case 0x5:   return DRV_RATE_5_5M;
    case 0xB:   return DRV_RATE_11M;
    case 0x16:  return DRV_RATE_22M;
    case 0x6:   return DRV_RATE_6M;
    case 0x9:   return DRV_RATE_9M;
    case 0xC:   return DRV_RATE_12M;
    case 0x12:  return DRV_RATE_18M;
    case 0x18:  return DRV_RATE_24M;
    case 0x24:  return DRV_RATE_36M;
    case 0x30:  return DRV_RATE_48M;
    case 0x36:  return DRV_RATE_54M;
    default:    return DRV_RATE_6M;
    }
}


/*-----------------------------------------------------------------------------

Routine Name:

    regConvertStringtoMACAddress

Routine Description: Converts the MAC Adrress in a form of string readen from the Registry 
to the MAC Address Array to be stored in the init_table struct 


Arguments:


Return Value:

    None

-----------------------------------------------------------------------------*/
void regConvertStringtoMACAddress(UINT8 *staMACAddressString,UINT8 *MacAddressArray)
{
    char *ptr;
    UINT8 *tmpMacAddr;
    UINT8 value = 0, value_l, value_h, add_value;
    int i, str_len;

    /* Take the pointer to the string MAC Address to convert it to the Array MAC Address */
    ptr = (char *)staMACAddressString;
    tmpMacAddr = MacAddressArray;
    str_len = 3 * MAC_ADDR_LEN - 1;
#if 0 
    for(i=0; i<MAC_ADDR_LEN ; ptr++)
    {
        value_l = (*ptr-'0');

        /* PRINTF(DBG_REGISTRY,("value_l [%d] *ptr %c value %d\n",value_l,*ptr,value));*/

        if( value_l  < 9)
        {
            value = value*10 + value_l;             
            /* PRINTF(DBG_REGISTRY,("value %d value_l %d  \n",value,value_l));*/
        }
        else
        {
            tmpMacAddr[i] = value;
            /* PRINTF(DBG_REGISTRY,("tmpMacAddr[%d]  is %d\n",i,tmpMacAddr[i]));*/
            value = 0;
            i++;
        }
    }
#else
    for(i=0;(i < MAC_ADDR_LEN);ptr++,str_len--)
    {
        if (str_len > 0) {
            /* The value can be or "0-9" or from "a-f" */
            value_l = (*ptr - '0');
            value_h = (*ptr - 'a');
        }
        else { /* last element */
            value_l = value_h = 16;
        }
        /*PRINTF(DBG_REGISTRY,("value_l [%d] value_h [%d] *ptr %c value %d\n",value_l,value_h,*ptr,value));*/

        if( (value_l <= 9) || (value_h <= 15 ) )
        {
            /* We are in an expected range */
            /* nCheck if 0-9 */
            if(value_l <= 9 )
            {
                add_value = value_l;
            }
            /* Check if a-f */
            else
            {
                /* 'a' is in fact 10 decimal in hexa */
                add_value = value_h + 10;
            }
            value = value * 16 + add_value;
            /*PRINTF(DBG_REGISTRY,("value %d add_value %d  \n",value,add_value));*/
        }
        else
        {
            tmpMacAddr[i] = value;
            /*PRINTF(DBG_REGISTRY,("tmpMacAddr[%d]  is %x\n",i,tmpMacAddr[i]));*/
            value = 0;
            i++;
        }
    }
#endif

}



/*-----------------------------------------------------------------------------

Routine Name:

    regFillInitTable

Routine Description:


Arguments:


Return Value:

    None

-----------------------------------------------------------------------------*/
VOID
regFillInitTable(
                PTIWLN_ADAPTER_T pAdapter,
                PVOID pInitTable
                )
{
    UINT8 radioString[3*RX_LEVEL_TABLE_SIZE];
    UINT8 bssidBroadcast[MAC_ADDR_LEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

    UINT8* RFMD_RxLevel = "bc b3 a9 9a 90 89 80 79 70 68 64 61 60 55 00";
    UINT8* RFMD_Lna = "01 01 01 01 01 01 01 01 01 01 01 01 01 01 01";
    UINT8* RFMD_Rssi = "0a 0f 14 19 1e 23 28 2d 32 37 3c 41 46 4b c8";

    UINT8* MAXIM_RxLevel = "55 4f 4a 43 3a 32 5d 56 4e 46 3e 37 2a 23 1f";
    UINT8* MAXIM_Lna = "00 00 00 00 00 00 01 01 01 01 01 01 01 01 01";
    UINT8* MAXIM_Rssi = "0a 0f 14 19 1e 23 28 2d 32 37 3c 41 46 4b 5a";
    char  dummySsidString[MAX_SSID_LEN];


    ctrlData_rateAdapt_t RateAdapt;
    UCHAR temp[MAX_SUPPORTED_RATES], i;

    UINT8  *ClsfrIp = "0a 03 01 c9";
    UINT8  ClsfrIpString[16];
    UINT8 ClsfrIpStringSize;

  /* EEPROM-less : MAC address */
    UINT8 regMACstrLen = REG_MAC_ADDR_STR_LEN;
    UINT8 staMACAddress[REG_MAC_ADDR_STR_LEN];
    UINT8 defStaMacAddress[]= "00 22 11 33 44 55";

    UINT8 regArpIpStrLen = REG_ARP_IP_ADDR_STR_LEN ; 
    UINT8 staArpIpAddress[REG_ARP_IP_ADDR_STR_LEN];
    UINT8 defArpIpAddress[] =  "0a 00 00 0a" ;       /*value by default*/

    /*defaults values for beacon IE table*/
    /*UINT8 defBeaconIETableSize = 0 ;*/
    static UINT8 defBeaconIETable[] = "00 01 01 01 32 01 2a 01 03 01 06 01 07 01 20 01 25 01 23 01 30 01 28 01 2e 01 85 01 dd 01 00 52 f2 02 00 01";
    /*UINT8 tmpIeTable[BEACON_FILTER_TABLE_MAX_SIZE] ;*/
    UINT8 staBeaconFilterIETable[BEACON_FILTER_STRING_MAX_LEN] ;
    UINT8 tmpIeTableSize = 35;
    UINT8 strSize = 104 ;

    UINT filterOffset = 0;
    char filterMask[16];
    UINT8 filterMaskLength;
    char filterPattern[16];
    UINT8 filterPatternLength;

    initTable_t* p = (initTable_t*) pInitTable;
    USHORT  tableLen = 0;
    USHORT  loopIndex = 0;
    static UINT8   ScanControlTable24Tmp[2 * NUM_OF_CHANNELS_24];
    static UINT8   ScanControlTable5Tmp[2 * A_5G_BAND_NUM_CHANNELS];
    static UINT8   ScanControlTable24Def[2* NUM_OF_CHANNELS_24] = "FFFFFFFFFFFFFFFFFFFFFFFFFFFF";
    static UINT8   ScanControlTable5Def[2 * A_5G_BAND_NUM_CHANNELS] = "FF000000FF000000FF000000FF000000FF000000FF000000FF000000FF0000000000000000000000000000000000000000000000000000000000000000000000FF000000FF000000FF000000FF000000FF000000FF000000FF000000FF000000FF000000FF000000FF0000000000000000FF000000FF000000FF000000FF00000000000000000000000000000000000000";
    UINT8   reportSeverityTableDefaults[WLAN_MAX_SEVERITIES] = "00000000000";
    UINT8   reportModuleTableDefaults[WLAN_MAX_LOG_MODULES];
    UINT16  reportSeverityTableLen;
    UINT16  reportModuleTableLen;
    
    int macIndex ; /*used for group address filtering*/
    UINT32  localNumTxDesc;
    PRINT(DBG_REGISTRY_LOUD, "TIWL: Reading InitTable parameters\n");
    
    /*set all report modules.as default*/
    memset(reportModuleTableDefaults, '1', WLAN_MAX_LOG_MODULES );
    
    /* Reset structure */
    NdisZeroMemory(p, sizeof(initTable_t));
    NdisZeroMemory(&RateAdapt, sizeof(ctrlData_rateAdapt_t));
    
    /*reads the arp ip from table*/
    regReadStringParameter(pAdapter ,&STRArp_Ip_Addr,
                            (PCHAR)(defArpIpAddress),REG_ARP_IP_ADDR_STR_LEN,
                            (PUCHAR)staArpIpAddress,&regArpIpStrLen ) ;

    regReadIntegerParameter(pAdapter, &STRArp_Ip_Filter_Ena,
                            DEF_FILTER_ENABLE_VALUE, MIN_FILTER_ENABLE_VALUE, MAX_FILTER_ENABLE_VALUE,
                            sizeof p->TnetwDrv_InitParams.arpIpFilterParams.isFilterEnabled, 
                            (PUCHAR)&p->TnetwDrv_InitParams.arpIpFilterParams.isFilterEnabled );


    regConvertStringtoIpAddress(staArpIpAddress, (PUCHAR)&(p->TnetwDrv_InitParams.arpIpFilterParams.arpIpInitParams.addr[0]) ); 
    
    /* Beacon filter*/
    /*is the desired state ENABLED ?*/
    regReadIntegerParameter(pAdapter, &STRBeaconFilterDesiredState,
                            DEF_BEACON_FILTER_ENABLE_VALUE, MIN_BEACON_FILTER_ENABLE_VALUE, MAX_BEACON_FILTER_ENABLE_VALUE,
                            sizeof p->siteMgrInitParams.beaconFilterParams.desiredState, 
                            (PUCHAR)&p->siteMgrInitParams.beaconFilterParams.desiredState );
    
    regReadIntegerParameter(pAdapter, &STRBeaconFilterStored,
                            DEF_NUM_STORED_FILTERS, MIN_NUM_STORED_FILTERS, MAX_NUM_STORED_FILTERS,
                            sizeof p->siteMgrInitParams.beaconFilterParams.numOfStored, 
                            (PUCHAR)&p->siteMgrInitParams.beaconFilterParams.numOfStored );

    /*Read the beacon filter IE table*/
    /*Read the size of the table*/
    regReadIntegerParameter(pAdapter, &STRBeaconIETableSize,
                            BEACON_FILTER_IE_TABLE_MIN_SIZE, BEACON_FILTER_IE_TABLE_MIN_SIZE,
                            BEACON_FILTER_IE_TABLE_MAX_SIZE,
                            sizeof p->siteMgrInitParams.beaconFilterParams.IETableSize, 
                            (PUCHAR)(&p->siteMgrInitParams.beaconFilterParams.IETableSize) );
    
    tmpIeTableSize = p->siteMgrInitParams.beaconFilterParams.IETableSize;
    
    /*Read the number of elements in the table ( this is because 221 has 5 values following it )*/
    regReadIntegerParameter(pAdapter, &STRBeaconIETableNumOfElem,
                            DEF_BEACON_FILTER_IE_TABLE_NUM, BEACON_FILTER_IE_TABLE_MIN_NUM,
                            BEACON_FILTER_IE_TABLE_MAX_NUM,
                            sizeof p->siteMgrInitParams.beaconFilterParams.numOfElements, 
                            (PUCHAR)(&p->siteMgrInitParams.beaconFilterParams.numOfElements) );

    /*printk("\n  OsRgstr tmpIeTableSize = %d numOfElems = %d" , tmpIeTableSize , p->siteMgrInitParams.beaconFilterParams.numOfElements) ;*/
    strSize = tmpIeTableSize*2 +tmpIeTableSize - 1 ; /*includes spaces between bytes*/
    if ( ( tmpIeTableSize  > 0 ) && ( tmpIeTableSize <= BEACON_FILTER_IE_TABLE_MAX_SIZE) )
    {
    
        regReadStringParameter(pAdapter, &STRBeaconIETable ,
                            (PCHAR)(defBeaconIETable), strSize,
                            (PUCHAR)staBeaconFilterIETable, &strSize);

        regConvertStringtoBeaconIETable(staBeaconFilterIETable , (PUCHAR)&p->siteMgrInitParams.beaconFilterParams.IETable[0]/*(PUCHAR)&(tmpIeTable[0] )*/ , tmpIeTableSize);
    }

    /* MAC ADDRESSES FILTER*/
    regReadIntegerParameter(pAdapter, &STRFilterEnabled,
                            DEF_FILTER_ENABLE_VALUE, MIN_FILTER_ENABLE_VALUE,
                            MAX_FILTER_ENABLE_VALUE,
                            sizeof p->TnetwDrv_InitParams.macAddrFilterParams.isFilterEnabled, 
                            (PUCHAR) &p->TnetwDrv_InitParams.macAddrFilterParams.isFilterEnabled);

    regReadIntegerParameter(pAdapter, &STRnumGroupAddrs,
                            NUM_GROUP_ADDRESS_VALUE_DEF, NUM_GROUP_ADDRESS_VALUE_MIN,
                            NUM_GROUP_ADDRESS_VALUE_MAX,
                            sizeof p->TnetwDrv_InitParams.macAddrFilterParams.numOfMacAddresses, 
                            (PUCHAR) &p->TnetwDrv_InitParams.macAddrFilterParams.numOfMacAddresses);
    
    /*printk("\nOsRgstry Num Of Group Addr:%d \n" , p->TnetwDrv_InitParams.macAddrFilterParams.numOfMacAddresses) ;*/

    macIndex = p->TnetwDrv_InitParams.macAddrFilterParams.numOfMacAddresses -1 ;
    switch( macIndex )
    {
    case 7:
        {

        regReadStringParameter(pAdapter, &STRGroup_addr7,
                            (PCHAR)(defStaMacAddress), REG_MAC_ADDR_STR_LEN,
                            (PUCHAR)staMACAddress, &regMACstrLen);

        regConvertStringtoMACAddress(staMACAddress,(PUCHAR) &p->TnetwDrv_InitParams.macAddrFilterParams.macAddrTable[7].addr);      
        --macIndex;
        }

    case 6:
        {

        regReadStringParameter(pAdapter, &STRGroup_addr6,
                            (PCHAR)(defStaMacAddress), REG_MAC_ADDR_STR_LEN,
                            (PUCHAR)staMACAddress, &regMACstrLen);

        regConvertStringtoMACAddress(staMACAddress,(PUCHAR) &p->TnetwDrv_InitParams.macAddrFilterParams.macAddrTable[6].addr);  
        --macIndex;
        }

    case 5: 
        {

        regReadStringParameter(pAdapter, &STRGroup_addr5,
                            (PCHAR)(defStaMacAddress), REG_MAC_ADDR_STR_LEN,
                            (PUCHAR)staMACAddress, &regMACstrLen);

        regConvertStringtoMACAddress(staMACAddress,(PUCHAR) &p->TnetwDrv_InitParams.macAddrFilterParams.macAddrTable[5].addr);  
        --macIndex;
        }

    case 4: 
        {

        regReadStringParameter(pAdapter, &STRGroup_addr4,
                            (PCHAR)(defStaMacAddress), REG_MAC_ADDR_STR_LEN,
                            (PUCHAR)staMACAddress, &regMACstrLen);

        regConvertStringtoMACAddress(staMACAddress,(PUCHAR) &p->TnetwDrv_InitParams.macAddrFilterParams.macAddrTable[4].addr);  
        --macIndex;
        }

    case 3:
        {

        regReadStringParameter(pAdapter, &STRGroup_addr3,
                            (PCHAR)(defStaMacAddress), REG_MAC_ADDR_STR_LEN,
                            (PUCHAR)staMACAddress, &regMACstrLen);

        regConvertStringtoMACAddress(staMACAddress,(PUCHAR) &p->TnetwDrv_InitParams.macAddrFilterParams.macAddrTable[3].addr);      
        --macIndex;
        }

    case 2:
        {

        regReadStringParameter(pAdapter, &STRGroup_addr2,
                            (PCHAR)(defStaMacAddress), REG_MAC_ADDR_STR_LEN,
                            (PUCHAR)staMACAddress, &regMACstrLen);

        regConvertStringtoMACAddress(staMACAddress,(PUCHAR) &p->TnetwDrv_InitParams.macAddrFilterParams.macAddrTable[2].addr ); 
        --macIndex;
        }

    case 1:
        {

        regReadStringParameter(pAdapter, &STRGroup_addr1,
                            (PCHAR)(defStaMacAddress), REG_MAC_ADDR_STR_LEN,
                            (PUCHAR)staMACAddress, &regMACstrLen);

        regConvertStringtoMACAddress(staMACAddress,(PUCHAR) &p->TnetwDrv_InitParams.macAddrFilterParams.macAddrTable[1].addr);  
        --macIndex;
        }

    
    case 0:
        {

        regReadStringParameter(pAdapter, &STRGroup_addr0,
                            (PCHAR)(defStaMacAddress), REG_MAC_ADDR_STR_LEN,
                            (PUCHAR)staMACAddress, &regMACstrLen);

        regConvertStringtoMACAddress(staMACAddress,(PUCHAR) &p->TnetwDrv_InitParams.macAddrFilterParams.macAddrTable[0].addr);  

        
        }

    default:
        {

        }
    }

    /* Read Beacon early wakeup parmeter */
    regReadIntegerParameter(pAdapter, &STREarlyWakeup,
                            EARLY_WAKEUP_ENABLE_DEF, EARLY_WAKEUP_ENABLE_MIN,
                            EARLY_WAKEUP_ENABLE_MAX,
                            sizeof(p->TnetwDrv_InitParams.macPreambleParams.earlyWakeUp), 
                            (PUCHAR) &p->TnetwDrv_InitParams.macPreambleParams.earlyWakeUp);

    
    /* Read TX XFER init parameters */
    regReadIntegerParameter(pAdapter, &STRTxXferBufferFullTimeToRecovery,
                            TX_XFER_HW_BUFFER_FULL_DUR_RECOVERY_DEF, TX_XFER_HW_BUFFER_FULL_DUR_RECOVERY_MIN,
                            TX_XFER_HW_BUFFER_FULL_DUR_RECOVERY_MAX,
                            sizeof (p->TnetwDrv_InitParams.txXferInitParams.timeToTxStuckMs), 
                            (PUCHAR) &(p->TnetwDrv_InitParams.txXferInitParams.timeToTxStuckMs));


    /************************/
    /* Read severity table */
    /**********************/

    regReadStringParameter(pAdapter, &STR_ReportSeverityTable,
                    (PCHAR)reportSeverityTableDefaults,
                    (UCHAR)WLAN_MAX_SEVERITIES,
                    (PUCHAR)p->TnetwDrv_InitParams.reportParams.SeverityTable,
                    (PUCHAR)&reportSeverityTableLen);


    /***********************/
    /* Read modules table */
    /*********************/

    regReadStringParameter(pAdapter, &STR_ReportModuleTable,
                    (PCHAR)reportModuleTableDefaults,
                    (UCHAR)WLAN_MAX_LOG_MODULES,
                    (PUCHAR)p->TnetwDrv_InitParams.reportParams.ModuleTable,
                    (PUCHAR)&reportModuleTableLen);


    /*
        Default SSID should be non-Valid SSID, hence the STA will not try to connect
    */
    for(loopIndex = 0; loopIndex < MAX_SSID_LEN; loopIndex++)
            dummySsidString[loopIndex] = (loopIndex+1);

    /*
     * Read CCK table
    */
    regReadRatesConfigArray(pAdapter->ConfigHandle, &STRRateAdaptationTable0,
        temp, "\x1\x2\x5\xB",
        (PULONG) &RateAdapt.len);
    for(i=0; i<RateAdapt.len; i++) RateAdapt.rateAdaptRatesTable[i] =
            RateNumberToHost(temp[i]);

    NdisZeroMemory(temp, sizeof(temp));

    regReadRatesConfigArray(pAdapter->ConfigHandle, &STRRateAdaptationFBThd0,
        &RateAdapt.rateAdaptFBTable[0], "\x32\x32\x32\x32",
        (PULONG) &RateAdapt.len);

    regReadRatesConfigArray(pAdapter->ConfigHandle, &STRRateAdaptationSUThd0,
        &RateAdapt.rateAdaptSUTable[0], "\x1E\x1E\x1E\x1E",
        (PULONG) &RateAdapt.len);

    NdisMoveMemory(&p->ctrlDataInitParams.rateTable.ctrlDataCckRateTable,
        &RateAdapt, sizeof(ctrlData_rateAdapt_t));


    /*
     * Read PBCC table
    */
    regReadRatesConfigArray(pAdapter->ConfigHandle, &STRRateAdaptationTable1,
        temp, "\x1\x2\x5\xB\x16",
        (PULONG) &RateAdapt.len);
    for(i=0; i<RateAdapt.len; i++) RateAdapt.rateAdaptRatesTable[i] =
            RateNumberToHost(temp[i]);

    NdisZeroMemory(temp, sizeof(temp));

    regReadRatesConfigArray(pAdapter->ConfigHandle, &STRRateAdaptationFBThd1,
        &RateAdapt.rateAdaptFBTable[0], "\x32\x32\x32\x32\x32",
        (PULONG) &RateAdapt.len);

    regReadRatesConfigArray(pAdapter->ConfigHandle, &STRRateAdaptationSUThd1,
        &RateAdapt.rateAdaptSUTable[0], "\x1E\x1E\x1E\x1E\x1E",
        (PULONG) &RateAdapt.len);

    NdisMoveMemory(&p->ctrlDataInitParams.rateTable.ctrlDataPbccRateTable,
        &RateAdapt, sizeof(ctrlData_rateAdapt_t));

    /*
     * Read OFDM table
    */
    regReadRatesConfigArray(pAdapter->ConfigHandle, &STRRateAdaptationTable2, 
        temp, "\x1\x2\x5\x6\x9\xB\xC\x12\x18\x24\x30\x36", 
        (PULONG) &RateAdapt.len);
    for(i=0; i<RateAdapt.len; i++) RateAdapt.rateAdaptRatesTable[i] =
            RateNumberToHost(temp[i]);

    NdisZeroMemory(temp, sizeof(temp));

    regReadRatesConfigArray(pAdapter->ConfigHandle, &STRRateAdaptationFBThd2,
        &RateAdapt.rateAdaptFBTable[0], "\x32\x32\x32\x32\x32\x32\x32\x2D\x2D\x2D\x2D\x14", 
        (PULONG) &RateAdapt.len);

    regReadRatesConfigArray(pAdapter->ConfigHandle, &STRRateAdaptationSUThd2,
        &RateAdapt.rateAdaptSUTable[0], "\x1E\x1E\x1E\x1E\x1E\x1E\x1E\x1E\x1E\xA\x5\x0", 
        (PULONG) &RateAdapt.len);

    NdisMoveMemory(&p->ctrlDataInitParams.rateTable.ctrlDataOfdmRateTable,
        &RateAdapt, sizeof(ctrlData_rateAdapt_t));


    /*
     * Read OFDMA table
    */
    regReadRatesConfigArray(pAdapter->ConfigHandle, &STRRateAdaptationTable3,
        temp, "\x6\x9\xC\x12\x18\x24\x30\x36",
        (PULONG) &RateAdapt.len);
    for(i=0; i<RateAdapt.len; i++) RateAdapt.rateAdaptRatesTable[i] =
            RateNumberToHost(temp[i]);

    NdisZeroMemory(temp, sizeof(temp));

    regReadRatesConfigArray(pAdapter->ConfigHandle, &STRRateAdaptationFBThd3,
        &RateAdapt.rateAdaptFBTable[0], "\x32\x32\x32\x32\x2D\x2D\x2D\x14",
        (PULONG) &RateAdapt.len);

    regReadRatesConfigArray(pAdapter->ConfigHandle, &STRRateAdaptationSUThd3,
        &RateAdapt.rateAdaptSUTable[0], "\x1E\x1E\x1E\x1E\x1E\xA\x5\x0",
        (PULONG) &RateAdapt.len);

    NdisMoveMemory(&p->ctrlDataInitParams.rateTable.ctrlDataOfdmARateTable,
        &RateAdapt, sizeof(ctrlData_rateAdapt_t));


    regReadIntegerParameter(pAdapter, &STRRateAdaptationLowTrshAcBK,
                            RATE_ADAPT_LOW_TRSH_AC_BK_DEF, RATE_ADAPT_LOW_TRSH_AC_BK_MIN,
                            RATE_ADAPT_LOW_TRSH_AC_BK_MAX,
                            sizeof p->ctrlDataInitParams.rateAdaptationInitParam.tspecsRateParameters[QOS_AC_BK].lowRateThreshold, 
                            (PUCHAR)&p->ctrlDataInitParams.rateAdaptationInitParam.tspecsRateParameters[QOS_AC_BK].lowRateThreshold);

    regReadIntegerParameter(pAdapter, &STRRateAdaptationLowTrshAcBE,
                            RATE_ADAPT_LOW_TRSH_AC_BE_DEF, RATE_ADAPT_LOW_TRSH_AC_BE_MIN,
                            RATE_ADAPT_LOW_TRSH_AC_BE_MAX,
                            sizeof p->ctrlDataInitParams.rateAdaptationInitParam.tspecsRateParameters[QOS_AC_BE].lowRateThreshold, 
                            (PUCHAR)&p->ctrlDataInitParams.rateAdaptationInitParam.tspecsRateParameters[QOS_AC_BE].lowRateThreshold);

    regReadIntegerParameter(pAdapter, &STRRateAdaptationLowTrshAcVI,
                            RATE_ADAPT_LOW_TRSH_AC_VI_DEF, RATE_ADAPT_LOW_TRSH_AC_VI_MIN,
                            RATE_ADAPT_LOW_TRSH_AC_VI_MAX,
                            sizeof p->ctrlDataInitParams.rateAdaptationInitParam.tspecsRateParameters[QOS_AC_VI].lowRateThreshold, 
                            (PUCHAR)&p->ctrlDataInitParams.rateAdaptationInitParam.tspecsRateParameters[QOS_AC_VI].lowRateThreshold);

    regReadIntegerParameter(pAdapter, &STRRateAdaptationLowTrshAcVO,
                            RATE_ADAPT_HIGH_TRSH_AC_VO_DEF, RATE_ADAPT_HIGH_TRSH_AC_VO_MIN,
                            RATE_ADAPT_HIGH_TRSH_AC_VO_MAX,
                            sizeof p->ctrlDataInitParams.rateAdaptationInitParam.tspecsRateParameters[QOS_AC_VO].lowRateThreshold, 
                            (PUCHAR)&p->ctrlDataInitParams.rateAdaptationInitParam.tspecsRateParameters[QOS_AC_VO].lowRateThreshold);

    regReadIntegerParameter(pAdapter, &STRRateAdaptationHighTrshAcBK,
                            RATE_ADAPT_HIGH_TRSH_AC_BK_DEF, RATE_ADAPT_HIGH_TRSH_AC_BK_MIN,
                            RATE_ADAPT_HIGH_TRSH_AC_BK_MAX,
                            sizeof p->ctrlDataInitParams.rateAdaptationInitParam.tspecsRateParameters[QOS_AC_BK].highRateThreshold, 
                            (PUCHAR)&p->ctrlDataInitParams.rateAdaptationInitParam.tspecsRateParameters[QOS_AC_BK].highRateThreshold);

    regReadIntegerParameter(pAdapter, &STRRateAdaptationHighTrshAcBE,
                            RATE_ADAPT_HIGH_TRSH_AC_BE_DEF, RATE_ADAPT_HIGH_TRSH_AC_BE_MIN,
                            RATE_ADAPT_HIGH_TRSH_AC_BE_MAX,
                            sizeof p->ctrlDataInitParams.rateAdaptationInitParam.tspecsRateParameters[QOS_AC_BE].highRateThreshold, 
                            (PUCHAR)&p->ctrlDataInitParams.rateAdaptationInitParam.tspecsRateParameters[QOS_AC_BE].highRateThreshold);

    regReadIntegerParameter(pAdapter, &STRRateAdaptationHighTrshAcVI,
                            RATE_ADAPT_HIGH_TRSH_AC_VI_DEF, RATE_ADAPT_HIGH_TRSH_AC_VI_MIN,
                            RATE_ADAPT_HIGH_TRSH_AC_VI_MAX,
                            sizeof p->ctrlDataInitParams.rateAdaptationInitParam.tspecsRateParameters[QOS_AC_VI].highRateThreshold, 
                            (PUCHAR)&p->ctrlDataInitParams.rateAdaptationInitParam.tspecsRateParameters[QOS_AC_VI].highRateThreshold);

    regReadIntegerParameter(pAdapter, &STRRateAdaptationHighTrshAcVO,
                            RATE_ADAPT_HIGH_TRSH_AC_VO_DEF, RATE_ADAPT_HIGH_TRSH_AC_VO_MIN,
                            RATE_ADAPT_HIGH_TRSH_AC_VO_MAX,
                            sizeof p->ctrlDataInitParams.rateAdaptationInitParam.tspecsRateParameters[QOS_AC_VO].highRateThreshold, 
                            (PUCHAR)&p->ctrlDataInitParams.rateAdaptationInitParam.tspecsRateParameters[QOS_AC_VO].highRateThreshold);

    /* rate Policy Params */
    regReadIntegerParameter(pAdapter, &STRRatePolicyUserShortRetryLimit,
                            CTRL_DATA_RATE_POLICY_USER_SHORT_RETRY_LIMIT_DEF, 
                            CTRL_DATA_RATE_POLICY_USER_SHORT_RETRY_LIMIT_MIN,
                            CTRL_DATA_RATE_POLICY_USER_SHORT_RETRY_LIMIT_MAX,
                            sizeof p->ctrlDataInitParams.ctrlDataTxRatePolicy[USER_RATE_CLASS].shortRetryLimit, 
                            (PUCHAR)&p->ctrlDataInitParams.ctrlDataTxRatePolicy[USER_RATE_CLASS].shortRetryLimit);

    regReadIntegerParameter(pAdapter, &STRRatePolicyUserLongRetryLimit,
                            CTRL_DATA_RATE_POLICY_USER_LONG_RETRY_LIMIT_DEF,
                            CTRL_DATA_RATE_POLICY_USER_LONG_RETRY_LIMIT_MIN,
                            CTRL_DATA_RATE_POLICY_USER_LONG_RETRY_LIMIT_MAX,
                            sizeof p->ctrlDataInitParams.ctrlDataTxRatePolicy[USER_RATE_CLASS].longRetryLimit, 
                            (PUCHAR)&p->ctrlDataInitParams.ctrlDataTxRatePolicy[USER_RATE_CLASS].longRetryLimit);

   regReadIntegerTable(pAdapter, &STRRatePolicyUserRetriesPerRateCck, CTRL_DATA_RATE_POLICY_USER_RETRIES_PER_RATE_CCK_DEF,
                              CTRL_DATA_RATE_POLICY_RETRIES_PER_RATE_MAX_LEN,
                              (PUCHAR)&p->ctrlDataInitParams.policyClassRatesArrayCck[USER_RATE_CLASS]);

   regReadIntegerTable(pAdapter, &STRRatePolicyUserRetriesPerRatePbcc, CTRL_DATA_RATE_POLICY_USER_RETRIES_PER_RATE_PBCC_DEF,
                              CTRL_DATA_RATE_POLICY_RETRIES_PER_RATE_MAX_LEN,
                              (PUCHAR)&p->ctrlDataInitParams.policyClassRatesArrayPbcc[USER_RATE_CLASS]);

   regReadIntegerTable(pAdapter, &STRRatePolicyUserRetriesPerRateOfdm, CTRL_DATA_RATE_POLICY_USER_RETRIES_PER_RATE_OFDM_DEF,
                              CTRL_DATA_RATE_POLICY_RETRIES_PER_RATE_MAX_LEN,
                              (PUCHAR)&p->ctrlDataInitParams.policyClassRatesArrayOfdm[USER_RATE_CLASS]);

   regReadIntegerTable(pAdapter, &STRRatePolicyUserRetriesPerRateOfdmA, CTRL_DATA_RATE_POLICY_USER_RETRIES_PER_RATE_OFDMA_DEF,
                              CTRL_DATA_RATE_POLICY_RETRIES_PER_RATE_MAX_LEN,
                              (PUCHAR)&p->ctrlDataInitParams.policyClassRatesArrayOfdmA[USER_RATE_CLASS]);


    regReadIntegerParameter(pAdapter, &STRRatePolicySGShortRetryLimit,
                            CTRL_DATA_RATE_POLICY_SG_SHORT_RETRY_LIMIT_DEF, 
                            CTRL_DATA_RATE_POLICY_SG_SHORT_RETRY_LIMIT_MIN,
                            CTRL_DATA_RATE_POLICY_SG_SHORT_RETRY_LIMIT_MAX,
                            sizeof p->ctrlDataInitParams.ctrlDataTxRatePolicy[SG_RATE_CLASS].shortRetryLimit, 
                            (PUCHAR)&p->ctrlDataInitParams.ctrlDataTxRatePolicy[SG_RATE_CLASS].shortRetryLimit);

    regReadIntegerParameter(pAdapter, &STRRatePolicySGLongRetryLimit,
                            CTRL_DATA_RATE_POLICY_SG_LONG_RETRY_LIMIT_DEF,
                            CTRL_DATA_RATE_POLICY_SG_LONG_RETRY_LIMIT_MIN,
                            CTRL_DATA_RATE_POLICY_SG_LONG_RETRY_LIMIT_MAX,
                            sizeof p->ctrlDataInitParams.ctrlDataTxRatePolicy[SG_RATE_CLASS].longRetryLimit, 
                            (PUCHAR)&p->ctrlDataInitParams.ctrlDataTxRatePolicy[SG_RATE_CLASS].longRetryLimit);
    
    regReadIntegerTable(pAdapter, &STRRatePolicySGRetriesPerRateCck, CTRL_DATA_RATE_POLICY_SG_RETRIES_PER_RATE_CCK_DEF,
                              CTRL_DATA_RATE_POLICY_RETRIES_PER_RATE_MAX_LEN,
                              (PUCHAR)&p->ctrlDataInitParams.policyClassRatesArrayCck[SG_RATE_CLASS]);
    regReadIntegerTable(pAdapter, &STRRatePolicySGRetriesPerRatePbcc, CTRL_DATA_RATE_POLICY_SG_RETRIES_PER_RATE_PBCC_DEF,
                              CTRL_DATA_RATE_POLICY_RETRIES_PER_RATE_MAX_LEN,
                              (PUCHAR)&p->ctrlDataInitParams.policyClassRatesArrayPbcc[SG_RATE_CLASS]);
    regReadIntegerTable(pAdapter, &STRRatePolicySGRetriesPerRateOfdm, CTRL_DATA_RATE_POLICY_SG_RETRIES_PER_RATE_OFDM_DEF,
                              CTRL_DATA_RATE_POLICY_RETRIES_PER_RATE_MAX_LEN,
                              (PUCHAR)&p->ctrlDataInitParams.policyClassRatesArrayOfdm[SG_RATE_CLASS]);
    regReadIntegerTable(pAdapter, &STRRatePolicySGRetriesPerRateOfdmA, CTRL_DATA_RATE_POLICY_SG_RETRIES_PER_RATE_OFDMA_DEF,
                              CTRL_DATA_RATE_POLICY_RETRIES_PER_RATE_MAX_LEN,
                              (PUCHAR)&p->ctrlDataInitParams.policyClassRatesArrayOfdmA[SG_RATE_CLASS]);




    regReadIntegerParameter(pAdapter, &STRRxEnergyDetection,
            FALSE, FALSE, TRUE, 
            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlRxEnergyDetection,
            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlRxEnergyDetection);

    regReadIntegerParameter(pAdapter, &STRTxEnergyDetection,
            FALSE, FALSE, TRUE, 
            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlTxEnergyDetection,
            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlTxEnergyDetection);

    regReadIntegerParameter(pAdapter, &STRTddCalibrationInterval,
            300, 1, 0xFFFFFFFF, 
            sizeof p->TnetwDrv_InitParams.whalCtrl_init.tddRadioCalTimout,
            (PUCHAR)&p->TnetwDrv_InitParams.whalCtrl_init.tddRadioCalTimout);

    regReadIntegerParameter(pAdapter, &STRCrtCalibrationInterval,
            2, 1, 0xFFFFFFFF, 
            sizeof p->TnetwDrv_InitParams.whalCtrl_init.CrtRadioCalTimout,
            (PUCHAR)&p->TnetwDrv_InitParams.whalCtrl_init.CrtRadioCalTimout);

    regReadIntegerParameter(pAdapter, &STRMacClockRate,
            80, 0, 255, 
            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlMacClock,
            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlMacClock);

    regReadIntegerParameter(pAdapter, &STRArmClockRate,
            80, 0, 255, 
            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlArmClock,
            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlArmClock);

    
    regReadIntegerParameter(pAdapter, &STRg80211DraftNumber,
            DRAFT_6_AND_LATER, DRAFT_5_AND_EARLIER, DRAFT_6_AND_LATER,
            sizeof p->siteMgrInitParams.siteMgrUseDraftNum,
            (PUCHAR)&p->siteMgrInitParams.siteMgrUseDraftNum);

     
    regReadIntegerParameter(pAdapter, &STRTraceBufferSize,
            /*1024, 0, 1024, sizeof(ULONG), */
            16, 16, 16, 
            sizeof p->TnetwDrv_InitParams.whalCtrl_init.TraceBufferSize,
            (PUCHAR)&p->TnetwDrv_InitParams.whalCtrl_init.TraceBufferSize);
   
    regReadIntegerParameter(pAdapter, &STRPrintTrace, 
            FALSE, FALSE, TRUE, 
            sizeof p->TnetwDrv_InitParams.whalCtrl_init.bDoPrint, 
            (PUCHAR)&p->TnetwDrv_InitParams.whalCtrl_init.bDoPrint);
   
    regReadIntegerParameter(pAdapter, &STRFirmwareDebug, 
            FALSE, FALSE, TRUE, 
            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlFirmwareDebug, 
            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlFirmwareDebug);


#ifndef TIWLN_WINCE30
    regReadIntegerParameter(pAdapter, &STRHwACXAccessMethod,
                            HAL_CTRL_HW_ACCESS_METHOD_DEF, HAL_CTRL_HW_ACCESS_METHOD_MIN,
                            HAL_CTRL_HW_ACCESS_METHOD_MAX,
                            sizeof p->TnetwDrv_InitParams.whalCtrl_init.hwAccessMethod, 
                            (PUCHAR)&p->TnetwDrv_InitParams.whalCtrl_init.hwAccessMethod);
#else
        /* Slave indirect*/
    p->TnetwDrv_InitParams.whalCtrl_init.hwAccessMethod = 0;
#endif
    regReadIntegerParameter(pAdapter, &STRMaxSitesFragCollect,
                            HAL_CTRL_SITE_FRAG_COLLECT_DEF, HAL_CTRL_SITE_FRAG_COLLECT_MIN,
                            HAL_CTRL_SITE_FRAG_COLLECT_MAX,
                            sizeof p->TnetwDrv_InitParams.whalCtrl_init.maxSitesFragCollect, 
                            (PUCHAR)&p->TnetwDrv_InitParams.whalCtrl_init.maxSitesFragCollect);

    regReadIntegerParameter(pAdapter, &STRBetEnable,
                            HAL_CTRL_BET_ENABLE_DEF, HAL_CTRL_BET_ENABLE_MIN,
                            HAL_CTRL_BET_ENABLE_MAX,
                            sizeof p->PowerMgrInitParams.BetEnable, 
                            (PUCHAR)&p->PowerMgrInitParams.BetEnable);

    regReadIntegerParameter(pAdapter, &STRBetMaxConsecutive,
                            HAL_CTRL_BET_MAX_CONSC_DEF, HAL_CTRL_BET_MAX_CONSC_MIN,
                            HAL_CTRL_BET_MAX_CONSC_MAX,
                            sizeof p->PowerMgrInitParams.MaximumConsecutiveET, 
                            (PUCHAR)&p->PowerMgrInitParams.MaximumConsecutiveET);

    /*--------------- Maximal time between full beacon reception ------------------*/
    regReadIntegerParameter(pAdapter, &STRMaxFullBeaconInterval,
                            HAL_CTRL_MAX_FULL_BEACON_DEF, HAL_CTRL_MAX_FULL_BEACON_MIN,
                            HAL_CTRL_MAX_FULL_BEACON_MAX,
                            sizeof p->PowerMgrInitParams.MaximalFullBeaconReceptionInterval,
                            (PUCHAR)&p->PowerMgrInitParams.MaximalFullBeaconReceptionInterval);

    regReadIntegerParameter(pAdapter, &STRBetEnableThreshold,
                            HAL_CTRL_BET_ENABLE_THRESHOLD_DEF, HAL_CTRL_BET_ENABLE_THRESHOLD_MIN,
                            HAL_CTRL_BET_ENABLE_THRESHOLD_MAX,
                            sizeof p->PowerMgrInitParams.BetEnableThreshold,
                            (PUCHAR)&p->PowerMgrInitParams.BetEnableThreshold);

    regReadIntegerParameter(pAdapter, &STRBetDisableThreshold,
                            HAL_CTRL_BET_DISABLE_THRESHOLD_DEF, HAL_CTRL_BET_DISABLE_THRESHOLD_MIN,
                            HAL_CTRL_BET_DISABLE_THRESHOLD_MAX,
                            sizeof p->PowerMgrInitParams.BetDisableThreshold,
                            (PUCHAR)&p->PowerMgrInitParams.BetDisableThreshold);

    p->TnetwDrv_InitParams.whalCtrl_init.rxMemBlkNumber = 60;
    p->TnetwDrv_InitParams.whalCtrl_init.txMinMemBlkNumber = 60;
    p->TnetwDrv_InitParams.whalCtrl_init.txCompleteTimeout = 500;  
    p->TnetwDrv_InitParams.whalCtrl_init.txCompleteThreshold = 1; 

    p->TnetwDrv_InitParams.whalCtrl_init.blockSize     = HAL_CTRL_ACX_BLOCK_SIZE_DEF;
    p->TnetwDrv_InitParams.whalCtrl_init.UseTxDataInterrupt = HAL_CTRL_USE_TX_DATA_INTR_DEF;
    p->TnetwDrv_InitParams.whalCtrl_init.UsePlcpHeader      = HAL_CTRL_USE_PLCP_HDR_DEF;

    regReadIntegerParameter(pAdapter, &STRNumACXRxDescriptors,
                            HAL_CTRL_ACX_RX_DESC_DEF, HAL_CTRL_ACX_RX_DESC_MIN,
                            HAL_CTRL_ACX_RX_DESC_MAX,
                            sizeof p->TnetwDrv_InitParams.whalCtrl_init.rxDescNum, 
                            (PUCHAR)&p->TnetwDrv_InitParams.whalCtrl_init.rxDescNum);

    regReadIntegerParameter(pAdapter, &STRNumACXTxDescriptors,
                            HAL_CTRL_ACX_TX_DESC_DEF, HAL_CTRL_ACX_TX_DESC_MIN,
                            HAL_CTRL_ACX_TX_DESC_MAX,
                            sizeof localNumTxDesc, 
                            (PUCHAR)&localNumTxDesc);

    regReadIntegerParameter(pAdapter, &STRTxFlashEnable,
                            HAL_CTRL_TX_FLASH_ENABLE_DEF, HAL_CTRL_TX_FLASH_ENABLE_MIN,
                            HAL_CTRL_TX_FLASH_ENABLE_MAX,
                            sizeof p->TnetwDrv_InitParams.whalCtrl_init.TxFlashEnable, 
                            (PUCHAR)&p->TnetwDrv_InitParams.whalCtrl_init.TxFlashEnable);


    p->TnetwDrv_InitParams.whalCtrl_init.numTxQueues    = MAX_NUM_OF_TX_QUEUES;
    for (i=0;i<p->TnetwDrv_InitParams.whalCtrl_init.numTxQueues;i++){

        p->TnetwDrv_InitParams.whalCtrl_init.tx_attrib_queue[i].numDesc  = localNumTxDesc;/*needed for validation phase only*/
        /* (!!!) must use different priority for each queue */
        p->TnetwDrv_InitParams.whalCtrl_init.tx_attrib_queue[i].priority = i;
    }



    p->TnetwDrv_InitParams.whalCtrl_init.beaconTemplateSize = sizeof(probeRspTemplate_t);
    p->TnetwDrv_InitParams.whalCtrl_init.probeRequestTemplateSize = sizeof(probeReqTemplate_t);
    p->TnetwDrv_InitParams.whalCtrl_init.probeResponseTemplateSize = sizeof(probeRspTemplate_t);
    p->TnetwDrv_InitParams.whalCtrl_init.nullTemplateSize = sizeof(nullDataTemplate_t);
    p->TnetwDrv_InitParams.whalCtrl_init.PsPollTemplateSize = sizeof(psPollTemplate_t);
    p->TnetwDrv_InitParams.whalCtrl_init.qosNullDataTemplateSize = sizeof(QosNullDataTemplate_t);

    regReadIntegerParameter(pAdapter,
                            &STRBeaconRxTimeout,
                            BCN_RX_TIMEOUT_DEF_VALUE,
                            BCN_RX_TIMEOUT_MIN_VALUE,
                            BCN_RX_TIMEOUT_MAX_VALUE,
                            sizeof p->TnetwDrv_InitParams.whalCtrl_init.BeaconRxTimeout,
                            (PUCHAR)&p->TnetwDrv_InitParams.whalCtrl_init.BeaconRxTimeout);

    regReadIntegerParameter(pAdapter,
                            &STRBroadcastRxTimeout,
                            BROADCAST_RX_TIMEOUT_DEF_VALUE,
                            BROADCAST_RX_TIMEOUT_MIN_VALUE,
                            BROADCAST_RX_TIMEOUT_MAX_VALUE,
                            sizeof p->TnetwDrv_InitParams.whalCtrl_init.BroadcastRxTimeout,
                            (PUCHAR)&p->TnetwDrv_InitParams.whalCtrl_init.BroadcastRxTimeout);

    regReadIntegerParameter(pAdapter,
                            &STRRxBroadcastInPs,
                            RX_BROADCAST_IN_PS_DEF_VALUE,
                            RX_BROADCAST_IN_PS_MIN_VALUE,
                            RX_BROADCAST_IN_PS_MAX_VALUE,
                            sizeof p->TnetwDrv_InitParams.whalCtrl_init.RxBroadcastInPs,
                            (PUCHAR)&p->TnetwDrv_InitParams.whalCtrl_init.RxBroadcastInPs);

    regReadIntegerParameter(pAdapter, &STRCalibrationChannel2_4,
                            HAL_CTRL_CALIBRATION_CHANNEL_2_4_DEF, HAL_CTRL_CALIBRATION_CHANNEL_2_4_MIN,
                            HAL_CTRL_CALIBRATION_CHANNEL_2_4_MAX,
                            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlCalibrationChannel2_4, 
                            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlCalibrationChannel2_4);

    regReadIntegerParameter(pAdapter, &STRCalibrationChannel5_0,
                            HAL_CTRL_CALIBRATION_CHANNEL_5_0_DEF, HAL_CTRL_CALIBRATION_CHANNEL_5_0_MIN,
                            HAL_CTRL_CALIBRATION_CHANNEL_5_0_MAX,
                            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlCalibrationChannel5_0, 
                            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlCalibrationChannel5_0);

    regReadIntegerParameter(pAdapter,
                            &STRConsecutivePsPollDeliveryFailureThreshold,
                            CONSECUTIVE_PS_POLL_FAILURE_DEF,
                            CONSECUTIVE_PS_POLL_FAILURE_MIN,
                            CONSECUTIVE_PS_POLL_FAILURE_MAX,
                            sizeof p->TnetwDrv_InitParams.whalCtrl_init.ConsecutivePsPollDeliveryFailureThreshold,
                            (PUCHAR)&p->TnetwDrv_InitParams.whalCtrl_init.ConsecutivePsPollDeliveryFailureThreshold);


    regReadIntegerParameter(pAdapter, &STRdot11RTSThreshold,
                            HAL_CTRL_RTS_THRESHOLD_DEF, HAL_CTRL_RTS_THRESHOLD_MIN,
                            HAL_CTRL_RTS_THRESHOLD_MAX,
                            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlRtsThreshold, 
                            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlRtsThreshold);

    regReadIntegerParameter(pAdapter, &STRRxDisableBroadcast,
                            HAL_CTRL_RX_DISABLE_BROADCAST_DEF, HAL_CTRL_RX_DISABLE_BROADCAST_MIN,
                            HAL_CTRL_RX_DISABLE_BROADCAST_MAX,
                            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlRxDisableBroadcast, 
                            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlRxDisableBroadcast);

    regReadIntegerParameter(pAdapter, &STRRecoveryEnable,
                            HAL_CTRL_RECOVERY_ENABLE_DEF, HAL_CTRL_RECOVERY_ENABLE_MIN,
                            HAL_CTRL_RECOVERY_ENABLE_MAX,
                            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlRecoveryEnable, 
                            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlRecoveryEnable);

    p->healthMonitorInitParams.FullRecoveryEnable = (BOOL)p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlRecoveryEnable;

    regReadIntegerParameter(pAdapter, &STRdot11FragThreshold,
                            HAL_CTRL_FRAG_THRESHOLD_DEF, HAL_CTRL_FRAG_THRESHOLD_MIN,
                            HAL_CTRL_FRAG_THRESHOLD_MAX,
                            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlFragThreshold, 
                            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlFragThreshold);

    regReadIntegerParameter(pAdapter, &STRdot11MaxTxMSDULifetime,
                            HAL_CTRL_MAX_TX_MSDU_LIFETIME_DEF, HAL_CTRL_MAX_TX_MSDU_LIFETIME_MIN,
                            HAL_CTRL_MAX_TX_MSDU_LIFETIME_MAX,
                            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlMaxTxMsduLifetime, 
                            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlMaxTxMsduLifetime);

    regReadIntegerParameter(pAdapter, &STRdot11MaxReceiveLifetime,
                            HAL_CTRL_MAX_RX_MSDU_LIFETIME_DEF, HAL_CTRL_MAX_RX_MSDU_LIFETIME_MIN,
                            HAL_CTRL_MAX_RX_MSDU_LIFETIME_MAX,
                            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlMaxRxMsduLifetime, 
                            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlMaxRxMsduLifetime);
    
    regReadIntegerParameter(pAdapter, &STRdot11RateFallBackRetryLimit,
                            HAL_CTRL_RATE_FB_RETRY_LIMIT_DEF, HAL_CTRL_RATE_FB_RETRY_LIMIT_MIN,
                            HAL_CTRL_RATE_FB_RETRY_LIMIT_MAX,
                            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlRateFallbackRetry, 
                            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlRateFallbackRetry);

    regReadIntegerParameter(pAdapter, &STRListenInterval,
                            HAL_CTRL_LISTEN_INTERVAL_DEF, HAL_CTRL_LISTEN_INTERVAL_MIN,
                            HAL_CTRL_LISTEN_INTERVAL_MAX,
                            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlListenInterval, 
                            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlListenInterval);

    regReadIntegerParameter(pAdapter, &STRdot11TxAntenna,
                            HAL_CTRL_TX_ANTENNA_DEF, HAL_CTRL_TX_ANTENNA_MIN,
                            HAL_CTRL_TX_ANTENNA_MAX,
                            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlTxAntenna, 
                            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlTxAntenna);
    /* reverse tx antenna value - ACX and utility have reversed values */
    if (p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlTxAntenna == TX_ANTENNA_2)
       p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlTxAntenna = TX_ANTENNA_1;
    else
       p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlTxAntenna = TX_ANTENNA_2;


    regReadIntegerParameter(pAdapter, &STRdot11RxAntenna,
                            HAL_CTRL_RX_ANTENNA_DEF, HAL_CTRL_RX_ANTENNA_MIN,
                            HAL_CTRL_RX_ANTENNA_MAX,
                            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlRxAntenna, 
                            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlRxAntenna);

    regReadIntegerParameter(pAdapter, &STRTxCompleteThreshold,
                            HAL_CTRL_TX_CMPLT_THRESHOLD_DEF, HAL_CTRL_TX_CMPLT_THRESHOLD_MIN,
                            HAL_CTRL_TX_CMPLT_THRESHOLD_MAX,
                            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlTxCompleteThreshold, 
                            (PUCHAR)&(p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlTxCompleteThreshold));

    pAdapter->ExtMode = TRUE;

    regReadIntegerParameter(pAdapter, &STRdot11DesiredChannel,
                        SITE_MGR_CHANNEL_DEF, SITE_MGR_CHANNEL_MIN, SITE_MGR_CHANNEL_MAX,
                        sizeof p->siteMgrInitParams.siteMgrDesiredChannel, 
                        (PUCHAR)&p->siteMgrInitParams.siteMgrDesiredChannel);

    memcpy((void *)p->siteMgrInitParams.siteMgrDesiredBSSID.addr, &bssidBroadcast, MAC_ADDR_LEN);
    
    regReadStringParameter(pAdapter, &STRdot11DesiredSSID,
                    (PCHAR)dummySsidString,
                    (UCHAR)MAX_SSID_LEN,
                    (PUCHAR)p->siteMgrInitParams.siteMgrDesiredSSID.ssidString,
                    (PUCHAR)&p->siteMgrInitParams.siteMgrDesiredSSID.len);

     
    regReadIntegerParameter(pAdapter, &STRdot11DesiredNetworkType,
                            SITE_MGR_DOT_11_MODE_DEF, SITE_MGR_DOT_11_MODE_MIN, SITE_MGR_DOT_11_MODE_MAX,
                            sizeof p->siteMgrInitParams.siteMgrDesiredDot11Mode, 
                            (PUCHAR)&p->siteMgrInitParams.siteMgrDesiredDot11Mode);

    regReadIntegerParameter(pAdapter, &STRdot11SlotTime,
        PHY_SLOT_TIME_SHORT, PHY_SLOT_TIME_LONG, PHY_SLOT_TIME_SHORT,
        sizeof p->siteMgrInitParams.siteMgrDesiredSlotTime,
        (PUCHAR)&p->siteMgrInitParams.siteMgrDesiredSlotTime);

    regReadIntegerParameter(pAdapter, &STRdot11RtsCtsProtection,
        0, 0, 1, 
        sizeof p->ctrlDataInitParams.ctrlDataDesiredCtsRtsStatus,
        (PUCHAR)&p->ctrlDataInitParams.ctrlDataDesiredCtsRtsStatus);

    regReadIntegerParameter(pAdapter, &STRdot11IbssProtection,
        ERP_PROTECTION_STANDARD, ERP_PROTECTION_NONE, ERP_PROTECTION_TI_TRICK,
        sizeof p->ctrlDataInitParams.ctrlDataDesiredIbssProtection,
        (PUCHAR)&p->ctrlDataInitParams.ctrlDataDesiredIbssProtection);

    /* When working in band A, minimum channel is 36 and not 1*/
    if (p->siteMgrInitParams.siteMgrDesiredDot11Mode  == DOT11_A_MODE)
    {
        if (p->siteMgrInitParams.siteMgrDesiredChannel < SITE_MGR_CHANNEL_A_MIN)
            p->siteMgrInitParams.siteMgrDesiredChannel = SITE_MGR_CHANNEL_A_MIN;
    }
    /* should be read from the registry */
    p->siteMgrInitParams.siteMgrRadioValues.siteMgr_radiaRadioValues.factorRSSI = 88;

    if(1)
    {
        UINT32 Freq2ChannelTable[] = {0,2412000,2417000,2422000,2427000,2432000,2437000,
                                      2442000,2447000,2452000,2457000,
                                      2462000,2467000,2472000,2484000};

        memcpy(p->siteMgrInitParams.siteMgrFreq2ChannelTable,
               Freq2ChannelTable,
               SITE_MGR_CHANNEL_MAX+1);
    }

    /* read TX rates from registry */
    readRates(pAdapter, p);

    regReadIntegerParameter(pAdapter, &STRdot11DesiredBSSType,
                            SITE_MGR_BSS_TYPE_DEF, BSS_INDEPENDENT, BSS_ANY,
                            sizeof p->siteMgrInitParams.siteMgrDesiredBSSType,
                            (PUCHAR)&p->siteMgrInitParams.siteMgrDesiredBSSType);

    regReadIntegerParameter(pAdapter, &STRdot11BeaconPeriod,
                            SITE_MGR_BEACON_INTERVAL_DEF, SITE_MGR_BEACON_INTERVAL_MIN,
                            SITE_MGR_BEACON_INTERVAL_MAX,
                            sizeof p->siteMgrInitParams.siteMgrDesiredBeaconInterval,
                            (PUCHAR)&p->siteMgrInitParams.siteMgrDesiredBeaconInterval);

    regReadIntegerParameter(pAdapter, &STRdot11ShortPreambleInvoked,
                            SITE_MGR_PREAMBLE_TYPE_DEF, PREAMBLE_LONG, PREAMBLE_SHORT,
                            sizeof p->siteMgrInitParams.siteMgrDesiredPreambleType,
                            (PUCHAR)&p->siteMgrInitParams.siteMgrDesiredPreambleType);

    /* RFMD */
    regReadStringParameter(pAdapter, &STRRadio11_RxLevel,
                            (PCHAR)RFMD_RxLevel, (USHORT)SITE_MGR_RX_LEVEL_TABLE_SIZE_DEF,
                            radioString, (PUCHAR)&p->siteMgrInitParams.siteMgrRadioValues.RxLevelTableSize);

    initRadioValusFromRgstryString( (PCHAR)radioString,
                                    (PCHAR)p->siteMgrInitParams.siteMgrRadioValues.siteMgr_rfmdRadioValues.siteMgr_radioRxLevel,
                                    RX_LEVEL_TABLE_SIZE);

    regReadStringParameter(pAdapter, &STRRadio11_LNA,
                            (PCHAR)RFMD_Lna, (USHORT)SITE_MGR_RX_LEVEL_TABLE_SIZE_DEF,
                            radioString, (PUCHAR)&p->siteMgrInitParams.siteMgrRadioValues.RxLevelTableSize);

    initRadioValusFromRgstryString( (PCHAR)radioString,
                                    (PCHAR)p->siteMgrInitParams.siteMgrRadioValues.siteMgr_rfmdRadioValues.siteMgr_radioLNA,
                                    RX_LEVEL_TABLE_SIZE);

    regReadStringParameter(pAdapter, &STRRadio11_RSSI,
                            (PCHAR)RFMD_Rssi, (USHORT)SITE_MGR_RX_LEVEL_TABLE_SIZE_DEF,
                            radioString, (PUCHAR)&p->siteMgrInitParams.siteMgrRadioValues.RxLevelTableSize);

    initRadioValusFromRgstryString( (PCHAR)radioString,
                                    (PCHAR)p->siteMgrInitParams.siteMgrRadioValues.siteMgr_rfmdRadioValues.siteMgr_radioRSSI,
                                    RX_LEVEL_TABLE_SIZE);
    /* MAXIM */

    regReadStringParameter(pAdapter, &STRRadio0D_RxLevel,
                           (PCHAR)MAXIM_RxLevel, (USHORT)SITE_MGR_RX_LEVEL_TABLE_SIZE_DEF,
                            radioString, (PUCHAR)&p->siteMgrInitParams.siteMgrRadioValues.RxLevelTableSize);

    initRadioValusFromRgstryString( (PCHAR)radioString,
                                    (PCHAR)p->siteMgrInitParams.siteMgrRadioValues.siteMgr_maximRadioValues.siteMgr_radioRxLevel,
                                    RX_LEVEL_TABLE_SIZE);

    regReadStringParameter(pAdapter, &STRRadio0D_LNA,
                           (PCHAR)MAXIM_Lna, (USHORT)SITE_MGR_RX_LEVEL_TABLE_SIZE_DEF,
                            radioString, (PUCHAR)&p->siteMgrInitParams.siteMgrRadioValues.RxLevelTableSize);

    initRadioValusFromRgstryString((PCHAR)radioString,
                                    (PCHAR)p->siteMgrInitParams.siteMgrRadioValues.siteMgr_maximRadioValues.siteMgr_radioLNA,
                                    RX_LEVEL_TABLE_SIZE);

    regReadStringParameter(pAdapter, &STRRadio0D_RSSI,
                            (PCHAR)MAXIM_Rssi, (USHORT)SITE_MGR_RX_LEVEL_TABLE_SIZE_DEF,
                            radioString, (PUCHAR)&p->siteMgrInitParams.siteMgrRadioValues.RxLevelTableSize);

    initRadioValusFromRgstryString((PCHAR)radioString,
                                    (PCHAR)p->siteMgrInitParams.siteMgrRadioValues.siteMgr_maximRadioValues.siteMgr_radioRSSI,
                                    RX_LEVEL_TABLE_SIZE);

    regReadIntegerParameter(pAdapter, &STRExternalMode,
                            SITE_MGR_EXTERNAL_MODE_DEF, SITE_MGR_EXTERNAL_MODE_MIN,
                            SITE_MGR_EXTERNAL_MODE_MAX,
                            sizeof p->siteMgrInitParams.siteMgrExternalConfiguration,
                            (PUCHAR)&p->siteMgrInitParams.siteMgrExternalConfiguration);

    regReadIntegerParameter(pAdapter, &STRWiFiAdHoc,
                            SITE_MGR_WiFiAdHoc_DEF, SITE_MGR_WiFiAdHoc_MIN, SITE_MGR_WiFiAdHoc_MAX,
                            sizeof p->siteMgrInitParams.siteMgrWiFiAdhoc,
                            (PUCHAR)&p->siteMgrInitParams.siteMgrWiFiAdhoc);

    regReadIntegerParameter(pAdapter, &STRWiFiWmmPS,
						    WIFI_WMM_PS_DEF, WIFI_WMM_PS_MIN, WIFI_WMM_PS_MAX,
						    sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.WiFiWmmPS,
						    (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.WiFiWmmPS);

    regReadIntegerParameter(pAdapter, &STRKeepAliveEnable,
                            SITE_MGR_KEEP_ALIVE_DEF, SITE_MGR_KEEP_ALIVE_MIN, SITE_MGR_KEEP_ALIVE_MAX,
                            sizeof p->siteMgrInitParams.siteMgrDesiredkeepAliveEnable,
                            (PUCHAR)&p->siteMgrInitParams.siteMgrDesiredkeepAliveEnable);

    pAdapter->IntMode = (BOOLEAN) p->siteMgrInitParams.siteMgrExternalConfiguration;

    regReadIntegerParameter(pAdapter, &STRConnSelfTimeout,
                            CONN_SELF_TIMEOUT_DEF, CONN_SELF_TIMEOUT_MIN, CONN_SELF_TIMEOUT_MAX,
                            sizeof p->connInitParams.connSelfTimeout,
                            (PUCHAR)&p->connInitParams.connSelfTimeout);

    regReadIntegerParameter(pAdapter, &STRdot11AuthRespTimeout,
                            AUTH_RESPONSE_TIMEOUT_DEF, AUTH_RESPONSE_TIMEOUT_MIN, AUTH_RESPONSE_TIMEOUT_MAX,
                            sizeof p->authInitParams.authResponseTimeout,
                            (PUCHAR)&p->authInitParams.authResponseTimeout);

    regReadIntegerParameter(pAdapter, &STRdot11MaxAuthRetry,
                            AUTH_MAX_RETRY_COUNT_DEF, AUTH_MAX_RETRY_COUNT_MIN, AUTH_MAX_RETRY_COUNT_MAX,
                            sizeof p->authInitParams.authMaxRetryCount,
                            (PUCHAR)&p->authInitParams.authMaxRetryCount);

    regReadIntegerParameter(pAdapter, &STRdot11AssocRespTimeout,
                            ASSOC_RESPONSE_TIMEOUT_DEF, ASSOC_RESPONSE_TIMEOUT_MIN, ASSOC_RESPONSE_TIMEOUT_MAX,
                            sizeof p->assocInitParams.assocResponseTimeout,
                            (PUCHAR)&p->assocInitParams.assocResponseTimeout);

    regReadIntegerParameter(pAdapter, &STRdot11MaxAssocRetry,
                            ASSOC_MAX_RETRY_COUNT_DEF, ASSOC_MAX_RETRY_COUNT_MIN, ASSOC_MAX_RETRY_COUNT_MAX,
                            sizeof p->assocInitParams.assocMaxRetryCount,
                            (PUCHAR)&p->assocInitParams.assocMaxRetryCount);

    regReadIntegerParameter(pAdapter, &STRRxDataFiltersEnabled,
                            RX_DATA_FILTERS_ENABLED_DEF, RX_DATA_FILTERS_ENABLED_MIN, RX_DATA_FILTERS_ENABLED_MAX,
                            sizeof p->rxDataInitParams.rxDataFiltersEnabled,
                            (PUCHAR)&p->rxDataInitParams.rxDataFiltersEnabled);

    regReadIntegerParameter(pAdapter, &STRRxDataFiltersFilter1Offset,
                            RX_DATA_FILTERS_FILTER_OFFSET_DEF, RX_DATA_FILTERS_FILTER_OFFSET_MIN, RX_DATA_FILTERS_FILTER_OFFSET_MAX,
                            sizeof filterOffset,
                            (PUCHAR) &filterOffset);

    regReadStringParameter(pAdapter, &STRRxDataFiltersFilter1Mask,
                            RX_DATA_FILTERS_FILTER_MASK_DEF, RX_DATA_FILTERS_FILTER_MASK_LEN_DEF,
                            (PUCHAR) filterMask,
                            (PUCHAR) &filterMaskLength);

    regReadStringParameter(pAdapter, &STRRxDataFiltersFilter1Pattern,
                            RX_DATA_FILTERS_FILTER_PATTERN_DEF, RX_DATA_FILTERS_FILTER_PATTERN_LEN_DEF,
                            (PUCHAR) filterPattern,
                            (PUCHAR) &filterPatternLength);

    parse_filter_request(&p->rxDataInitParams.rxDataFilterRequests[0], filterOffset, filterMask, filterMaskLength, filterPattern, filterPatternLength);

    regReadIntegerParameter(pAdapter, &STRRxDataFiltersFilter2Offset,
                            RX_DATA_FILTERS_FILTER_OFFSET_DEF, RX_DATA_FILTERS_FILTER_OFFSET_MIN, RX_DATA_FILTERS_FILTER_OFFSET_MAX,
                            sizeof filterOffset,
                            (PUCHAR) &filterOffset);

    regReadStringParameter(pAdapter, &STRRxDataFiltersFilter2Mask,
                            RX_DATA_FILTERS_FILTER_MASK_DEF, RX_DATA_FILTERS_FILTER_MASK_LEN_DEF,
                            (PUCHAR) filterMask,
                            (PUCHAR) &filterMaskLength);

    regReadStringParameter(pAdapter, &STRRxDataFiltersFilter2Pattern,
                            RX_DATA_FILTERS_FILTER_PATTERN_DEF, RX_DATA_FILTERS_FILTER_PATTERN_LEN_DEF,
                            (PUCHAR) filterPattern,
                            (PUCHAR) &filterPatternLength);

    parse_filter_request(&p->rxDataInitParams.rxDataFilterRequests[1], filterOffset, filterMask, filterMaskLength, filterPattern, filterPatternLength);

    regReadIntegerParameter(pAdapter, &STRRxDataFiltersFilter3Offset,
                            RX_DATA_FILTERS_FILTER_OFFSET_DEF, RX_DATA_FILTERS_FILTER_OFFSET_MIN, RX_DATA_FILTERS_FILTER_OFFSET_MAX,
                            sizeof filterOffset,
                            (PUCHAR) &filterOffset);

    regReadStringParameter(pAdapter, &STRRxDataFiltersFilter3Mask,
                            RX_DATA_FILTERS_FILTER_MASK_DEF, RX_DATA_FILTERS_FILTER_MASK_LEN_DEF,
                            (PUCHAR) filterMask,
                            (PUCHAR) &filterMaskLength);

    regReadStringParameter(pAdapter, &STRRxDataFiltersFilter3Pattern,
                            RX_DATA_FILTERS_FILTER_PATTERN_DEF, RX_DATA_FILTERS_FILTER_PATTERN_LEN_DEF,
                            (PUCHAR) filterPattern,
                            (PUCHAR) &filterPatternLength);

    parse_filter_request(&p->rxDataInitParams.rxDataFilterRequests[2], filterOffset, filterMask, filterMaskLength, filterPattern, filterPatternLength);

    regReadIntegerParameter(pAdapter, &STRRxDataFiltersFilter4Offset,
                            RX_DATA_FILTERS_FILTER_OFFSET_DEF, RX_DATA_FILTERS_FILTER_OFFSET_MIN, RX_DATA_FILTERS_FILTER_OFFSET_MAX,
                            sizeof filterOffset,
                            (PUCHAR) &filterOffset);

    regReadStringParameter(pAdapter, &STRRxDataFiltersFilter4Mask,
                            RX_DATA_FILTERS_FILTER_MASK_DEF, RX_DATA_FILTERS_FILTER_MASK_LEN_DEF,
                            (PUCHAR) filterMask,
                            (PUCHAR) &filterMaskLength);

    regReadStringParameter(pAdapter, &STRRxDataFiltersFilter4Pattern,
                            RX_DATA_FILTERS_FILTER_PATTERN_DEF, RX_DATA_FILTERS_FILTER_PATTERN_LEN_DEF,
                            (PUCHAR) filterPattern,
                            (PUCHAR) &filterPatternLength);

    parse_filter_request(&p->rxDataInitParams.rxDataFilterRequests[3], filterOffset, filterMask, filterMaskLength, filterPattern, filterPatternLength);

    regReadIntegerParameter(pAdapter, &STRRxDataFiltersDefaultAction,
                            RX_DATA_FILTERS_DEFAULT_ACTION_DEF, RX_DATA_FILTERS_DEFAULT_ACTION_MIN,
                            RX_DATA_FILTERS_DEFAULT_ACTION_MAX,
                            sizeof p->rxDataInitParams.rxDataFiltersDefaultAction,
                            (PUCHAR)&p->rxDataInitParams.rxDataFiltersDefaultAction);

    regReadIntegerParameter(pAdapter, &STRNumTxDataQueues,
                            TX_DATA_NUMBER_OF_DATA_QUEUES_DEF, TX_DATA_NUMBER_OF_DATA_QUEUES_MIN,
                            TX_DATA_NUMBER_OF_DATA_QUEUES_MAX,
                            sizeof p->txDataInitParams.txDataNumOfDataQueues,
                            (PUCHAR)&p->txDataInitParams.txDataNumOfDataQueues);

    regReadIntegerParameter(pAdapter, &STRCreditCalcTimout,
                            TX_DATA_CREDIT_CALC_TIMOEUT_DEF, TX_DATA_CREDIT_CALC_TIMOEUT_MIN,
                            TX_DATA_CREDIT_CALC_TIMOEUT_MAX,
                            sizeof p->txDataInitParams.creditCalculationTimeout,
                            (PUCHAR)&p->txDataInitParams.creditCalculationTimeout);

	regReadIntegerParameter(pAdapter, &STRCreditCalcTimerEnabled,
							FALSE, FALSE, TRUE,
							sizeof p->txDataInitParams.bCreditCalcTimerEnabled,
							(PUCHAR)&p->txDataInitParams.bCreditCalcTimerEnabled);

    regReadIntegerParameter(pAdapter, &STRFracOfLifeTimeToDrop,
                            TX_DATA_FRAC_OF_LIFE_TIME_TO_DROP_DEF, TX_DATA_FRAC_OF_LIFE_TIME_TO_DROP_MIN,
                            TX_DATA_FRAC_OF_LIFE_TIME_TO_DROP_MAX,
                            sizeof p->txDataInitParams.uFracOfLifeTimeToDrop,
                            (PUCHAR)&p->txDataInitParams.uFracOfLifeTimeToDrop);

    regReadIntegerParameter(pAdapter, &STRAdmCtrlDelayDueToMediumTimeOverUsage,
                            TX_DATA_ADM_CTRL_DELAY_DUE_TO_MEDIUM_OVER_USAGE_DEF, TX_DATA_ADM_CTRL_DELAY_DUE_TO_MEDIUM_OVER_USAGE_MIN,
                            TX_DATA_ADM_CTRL_DELAY_DUE_TO_MEDIUM_OVER_USAGE_MAX,
                            sizeof p->txDataInitParams.admCtrlDelayDueToMediumTimeOverUsage,
                            (PUCHAR)&p->txDataInitParams.admCtrlDelayDueToMediumTimeOverUsage);
  
    regReadIntegerParameter(pAdapter, &STRAdmissionDownGradeEnable,
                            TX_DATA_ADM_CTRL_DOWN_GRADE_DEF, TX_DATA_ADM_CTRL_DOWN_GRADE_MIN,
                            TX_DATA_ADM_CTRL_DOWN_GRADE_MAX,
                            sizeof p->txDataInitParams.admissionDownGradeEnable,
                            (PUCHAR)&p->txDataInitParams.admissionDownGradeEnable);

    regReadIntegerParameter(pAdapter, &STRTrafficAdmControlTimeout,
                    TRAFFIC_ADM_CONTROL_TIMEOUT_DEF, TRAFFIC_ADM_CONTROL_TIMEOUT_MIN,
                    TRAFFIC_ADM_CONTROL_TIMEOUT_MAX,
                    sizeof p->qosMngrInitParams.trafficAdmCtrlInitParams.trafficAdmCtrlResponseTimeout,
                    (PUCHAR)&p->qosMngrInitParams.trafficAdmCtrlInitParams.trafficAdmCtrlResponseTimeout);

    regReadIntegerParameter(pAdapter, &STRTrafficAdmControlUseFixedMsduSize,
                    FALSE, FALSE, TRUE,
                    sizeof p->qosMngrInitParams.trafficAdmCtrlInitParams.trafficAdmCtrlUseFixedMsduSize,
                    (PUCHAR)&p->qosMngrInitParams.trafficAdmCtrlInitParams.trafficAdmCtrlUseFixedMsduSize);

    regReadIntegerParameter(pAdapter, &STRDesiredMaxSpLen,
                    QOS_MAX_SP_LEN_DEF, QOS_MAX_SP_LEN_MIN,
                    QOS_MAX_SP_LEN_MAX,
                    sizeof p->qosMngrInitParams.desiredMaxSpLen,
                    (PUCHAR)&p->qosMngrInitParams.desiredMaxSpLen);

    regReadIntegerParameter(pAdapter, &STRRateContThreshold,
                            CTRL_DATA_CONT_TX_THRESHOLD_DEF, CTRL_DATA_CONT_TX_THRESHOLD_MIN,
                            CTRL_DATA_CONT_TX_THRESHOLD_MAX,
                            sizeof p->ctrlDataInitParams.rateAdaptationInitParam.contTxPacketsThreshold,
                            (PUCHAR)&p->ctrlDataInitParams.rateAdaptationInitParam.contTxPacketsThreshold);

    regReadIntegerParameter(pAdapter, &STRRateStepUpThreshold,
                            CTRL_DATA_STEP_UP_TX_THRESHOLD_DEF, CTRL_DATA_STEP_UP_TX_THRESHOLD_MIN,
                            CTRL_DATA_STEP_UP_TX_THRESHOLD_MAX,
                            sizeof p->ctrlDataInitParams.rateAdaptationInitParam.stepUpTxPacketsThreshold,
                            (PUCHAR)&p->ctrlDataInitParams.rateAdaptationInitParam.stepUpTxPacketsThreshold);

    regReadIntegerParameter(pAdapter, &STRFBShortInterval,
                            CTRL_DATA_FB_SHORT_INTERVAL_DEF, CTRL_DATA_FB_SHORT_INTERVAL_MIN,
                            CTRL_DATA_FB_SHORT_INTERVAL_MAX,
                            sizeof p->ctrlDataInitParams.rateAdaptationInitParam.ctrlDataFBShortInterval,
                            (PUCHAR)&p->ctrlDataInitParams.rateAdaptationInitParam.ctrlDataFBShortInterval);

    regReadIntegerParameter(pAdapter, &STRFBLongInterval,
                            CTRL_DATA_FB_LONG_INTERVAL_DEF, CTRL_DATA_FB_LONG_INTERVAL_MIN,
                            CTRL_DATA_FB_LONG_INTERVAL_MAX,
                            sizeof p->ctrlDataInitParams.rateAdaptationInitParam.ctrlDataFBLongInterval,
                            (PUCHAR)&p->ctrlDataInitParams.rateAdaptationInitParam.ctrlDataFBLongInterval);

    regReadIntegerParameter(pAdapter, &STRRateAdaptationTimeout,
                            RATE_ADAPTATION_TIMEOUT_DEF , RATE_ADAPTATION_TIMEOUT_MIN, RATE_ADAPTATION_TIMEOUT_MAX, 
                            sizeof p->ctrlDataInitParams.rateAdaptationInitParam.rateAdapt_timeout,
                            (PUCHAR)&p->ctrlDataInitParams.rateAdaptationInitParam.rateAdapt_timeout);


    regReadIntegerParameter(pAdapter, &STRRateControlEnable,
                            CTRL_DATA_RATE_CONTROL_ENABLE_DEF, CTRL_DATA_RATE_CONTROL_ENABLE_MIN, CTRL_DATA_RATE_CONTROL_ENABLE_MAX,
                            sizeof p->ctrlDataInitParams.ctrlDataRateControlEnable,
                            (PUCHAR)&p->ctrlDataInitParams.ctrlDataRateControlEnable);

/*                              SME Initialization Parameters                           */
/*                          ====================================                        */


    regReadIntegerParameter(pAdapter, &STRdot11SmeScanEnabled,
                            ENABLE_SME_SCAN_DEF, ENABLE_SME_SCAN_MIN, ENABLE_SME_SCAN_MAX,
                            sizeof p->smeInitParams.EnableFirstConnScan,
                            (PUCHAR)&p->smeInitParams.EnableFirstConnScan);

    regReadIntegerParameter(pAdapter, &STRdot11SmeInterScanMin,
                            SME_INTER_SCAN_MIN_DEF, SME_INTER_SCAN_MIN_MIN, SME_INTER_SCAN_MIN_MAX,
                            sizeof p->smeInitParams.InterScanIntervalMin,
                            (PUCHAR)&p->smeInitParams.InterScanIntervalMin);

    regReadIntegerParameter(pAdapter, &STRdot11SmeInterScanMax,
                            SME_INTER_SCAN_MAX_DEF, SME_INTER_SCAN_MAX_MIN, SME_INTER_SCAN_MAX_MAX,
                            sizeof p->smeInitParams.InterScanIntervalMax,
                            (PUCHAR)&p->smeInitParams.InterScanIntervalMax);

    regReadIntegerParameter(pAdapter, &STRdot11SmeInterScanDelta,
                            SME_INTER_SCAN_DELTA_DEF, SME_INTER_SCAN_DELTA_MIN, SME_INTER_SCAN_DELTA_MAX,
                            sizeof p->smeInitParams.InterScanIntervalDelta,
                            (PUCHAR)&p->smeInitParams.InterScanIntervalDelta);



    /*          B/G scan first scan params                    */
    /*         ----------------------------                   */
    regReadIntegerTable(pAdapter, &STRdot11SmeScanBGChannelList, SME_SCAN_BG_LIST_BAND_VAL_DEF,
                              SME_SCAN_BG_LIST_BAND_STRING_MAX_SIZE,
                              (PUCHAR)&p->smeInitParams.scanParamsBG.channelsList);

    regReadIntegerParameter(pAdapter, &STRdot11SmeScanBGMinDwellTime,
                            SME_SCAN_BG_MIN_DWELL_TIME_DEF, SME_SCAN_BG_MIN_DWELL_TIME_MIN, SME_SCAN_BG_MIN_DWELL_TIME_MAX,
                            sizeof p->smeInitParams.scanParamsBG.minDwellTime,
                            (PUCHAR)&p->smeInitParams.scanParamsBG.minDwellTime);
    
    regReadIntegerParameter(pAdapter, &STRdot11SmeScanBGMaxDwellTime,
                            SME_SCAN_BG_MAX_DWELL_TIME_DEF, SME_SCAN_BG_MAX_DWELL_TIME_MIN, SME_SCAN_BG_MAX_DWELL_TIME_MAX,
                            sizeof p->smeInitParams.scanParamsBG.maxDwellTime,
                            (PUCHAR)&p->smeInitParams.scanParamsBG.maxDwellTime);

    regReadIntegerParameter(pAdapter, &STRdot11SmeScanBGNumProbReq,
                            SME_SCAN_BG_NUM_PROB_REQ_DEF, SME_SCAN_BG_NUM_PROB_REQ_MIN, SME_SCAN_BG_NUM_PROB_REQ_MAX,
                            sizeof p->smeInitParams.scanParamsBG.probeReqNumber,
                            (PUCHAR)&p->smeInitParams.scanParamsBG.probeReqNumber);
    
    regReadIntegerParameter(pAdapter, &STRdot11SmeScanBGProbReqRate,
                            SME_SCAN_BG_PROB_REQ_RATE_DEF, SME_SCAN_BG_PROB_REQ_RATE_MIN, SME_SCAN_BG_NUM_PROB_REQ_RATE_MAX,
                            sizeof p->smeInitParams.scanParamsBG.probeRequestRate,
                            (PUCHAR)&p->smeInitParams.scanParamsBG.probeRequestRate);

    regReadIntegerParameter(pAdapter, &STRdot11SmeScanBGTxPowerLevel,
                            SME_SCAN_BG_TX_POWER_DEF, SME_SCAN_BG_TX_POWER_MIN, SME_SCAN_BG_TX_POWER_MAX,
                            sizeof p->smeInitParams.scanParamsBG.txPowerDbm,
                            (PUCHAR)&p->smeInitParams.scanParamsBG.txPowerDbm);
    



    /*          A band first scan params                      */
    /*         ----------------------------                   */
    regReadIntegerTable(pAdapter, &STRdot11SmeScanAChannelList, SME_SCAN_A_LIST_BAND_VAL_DEF,
                              SME_SCAN_A_LIST_BAND_STRING_MAX_SIZE,
                              (PUCHAR)&p->smeInitParams.scanParamsA.channelsList);

    regReadIntegerParameter(pAdapter, &STRdot11SmeScanAMinDwellTime,
                            SME_SCAN_A_MIN_DWELL_TIME_DEF, SME_SCAN_A_MIN_DWELL_TIME_MIN, SME_SCAN_A_MIN_DWELL_TIME_MAX,
                            sizeof p->smeInitParams.scanParamsA.minDwellTime,
                            (PUCHAR)&p->smeInitParams.scanParamsA.minDwellTime);
    
    regReadIntegerParameter(pAdapter, &STRdot11SmeScanAMaxDwellTime,
                            SME_SCAN_A_MAX_DWELL_TIME_DEF, SME_SCAN_A_MAX_DWELL_TIME_MIN, SME_SCAN_A_MAX_DWELL_TIME_MAX,
                            sizeof p->smeInitParams.scanParamsA.maxDwellTime,
                            (PUCHAR)&p->smeInitParams.scanParamsA.maxDwellTime);

    regReadIntegerParameter(pAdapter, &STRdot11SmeScanANumProbReq,
                            SME_SCAN_A_NUM_PROB_REQ_DEF, SME_SCAN_A_NUM_PROB_REQ_MIN, SME_SCAN_A_NUM_PROB_REQ_MAX,
                            sizeof p->smeInitParams.scanParamsA.probeReqNumber,
                            (PUCHAR)&p->smeInitParams.scanParamsA.probeReqNumber);
    
    regReadIntegerParameter(pAdapter, &STRdot11SmeScanAProbReqRate,
                            SME_SCAN_A_PROB_REQ_RATE_DEF, SME_SCAN_A_PROB_REQ_RATE_MIN, SME_SCAN_A_NUM_PROB_REQ_RATE_MAX,
                            sizeof p->smeInitParams.scanParamsA.probeRequestRate,
                            (PUCHAR)&p->smeInitParams.scanParamsA.probeRequestRate);

    regReadIntegerParameter(pAdapter, &STRdot11SmeScanATxPowerLevel,
                            SME_SCAN_A_TX_POWER_DEF, SME_SCAN_A_TX_POWER_MIN, SME_SCAN_A_TX_POWER_MAX,
                            sizeof p->smeInitParams.scanParamsA.txPowerDbm,
                            (PUCHAR)&p->smeInitParams.scanParamsA.txPowerDbm);

    regReadIntegerParameter(pAdapter, &STRdot11AuthenticationMode,
                            RSN_AUTH_SUITE_DEF, RSN_AUTH_SUITE_MIN, RSN_AUTH_SUITE_MAX,
                            sizeof p->rsnInitParams.authSuite,
                            (PUCHAR)&p->rsnInitParams.authSuite);

    regReadIntegerParameter(pAdapter, &STRdot11FourXEnable,
                            CTRL_DATA_FOUR_X_ENABLE_DEF, CTRL_DATA_FOUR_X_ENABLE_MIN, CTRL_DATA_FOUR_X_ENABLE_MAX,
                            sizeof p->ctrlDataInitParams.ctrlDataFourXEnable,
                            (PUCHAR)&p->ctrlDataInitParams.ctrlDataFourXEnable);

    /* Soft Gemini Section */
    
    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistEnable,
                            SOFT_GEMINI_ENABLED_DEF, SOFT_GEMINI_ENABLED_MIN, SOFT_GEMINI_ENABLED_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiEnable, 
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiEnable);

    regReadIntegerTable(pAdapter, &STRBThWlanCoexistRate, SG_RATES_DEF,
                              SG_RATES_STRING_MAX_DEF,
                              (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiRate);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamsbtHpMaxTime,
                            SOFT_GEMINI_PARAMS_BT_HP_MAXTIME_DEF, SOFT_GEMINI_PARAMS_BT_HP_MAXTIME_MIN, SOFT_GEMINI_PARAMS_BT_HP_MAXTIME_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.btHpMaxTime,
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.btHpMaxTime);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamswlanHpMaxTime,
                            SOFT_GEMINI_PARAMS_WLAN_HP_MAX_TIME_DEF, SOFT_GEMINI_PARAMS_WLAN_HP_MAX_TIME_MIN, SOFT_GEMINI_PARAMS_WLAN_HP_MAX_TIME_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.wlanHpMaxTime,
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.wlanHpMaxTime);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamssenseDisableTimer,
                            SOFT_GEMINI_PARAMS_SENSE_DISABLE_TIMER_DEF, SOFT_GEMINI_PARAMS_SENSE_DISABLE_TIMER_MIN, SOFT_GEMINI_PARAMS_SENSE_DISABLE_TIMER_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.senseDisableTimer,
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.senseDisableTimer);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamsprotectiveRxTimeBeforeBtHp,
                            SOFT_GEMINI_PARAMS_PROTECTIVE_RX_TIME_DEF, SOFT_GEMINI_PARAMS_PROTECTIVE_RX_TIME_MIN, SOFT_GEMINI_PARAMS_PROTECTIVE_RX_TIME_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.protectiveRxTimeBeforeBtHp,
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.protectiveRxTimeBeforeBtHp);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamsprotectiveTxTimeBeforeBtHp,
                            SOFT_GEMINI_PARAMS_PROTECTIVE_TX_TIME_DEF, SOFT_GEMINI_PARAMS_PROTECTIVE_TX_TIME_MIN, SOFT_GEMINI_PARAMS_PROTECTIVE_TX_TIME_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.protectiveTxTimeBeforeBtHp,
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.protectiveTxTimeBeforeBtHp);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamsprotectiveRxTimeBeforeBtHpFastAp,
                            SOFT_GEMINI_PARAMS_PROTECTIVE_RX_TIME_FAST_DEF, SOFT_GEMINI_PARAMS_PROTECTIVE_RX_TIME_FAST_MIN, SOFT_GEMINI_PARAMS_PROTECTIVE_RX_TIME_FAST_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.protectiveRxTimeBeforeBtHpFastAp,
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.protectiveRxTimeBeforeBtHpFastAp);
 
    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamsprotectiveTxTimeBeforeBtHpFastAp,
                            SOFT_GEMINI_PARAMS_PROTECTIVE_TX_TIME_FAST_DEF, SOFT_GEMINI_PARAMS_PROTECTIVE_TX_TIME_FAST_MIN, SOFT_GEMINI_PARAMS_PROTECTIVE_TX_TIME_FAST_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.protectiveTxTimeBeforeBtHpFastAp, 
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.protectiveTxTimeBeforeBtHpFastAp);
 
    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamsprotectiveWlanCycleTimeForFastAp,
                            SOFT_GEMINI_PARAMS_CYCLE_TIME_FAST_DEF, SOFT_GEMINI_PARAMS_CYCLE_TIME_FAST_MIN, SOFT_GEMINI_PARAMS_CYCLE_TIME_FAST_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.protectiveWlanCycleTimeForFastAp, 
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.protectiveWlanCycleTimeForFastAp);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamstimeoutNextBtLpPacket,
                            SOFT_GEMINI_PARAMS_TIMEOUT_NEXT_BT_LP_PACKET_DEF, SOFT_GEMINI_PARAMS_TIMEOUT_NEXT_BT_LP_PACKET_MIN, SOFT_GEMINI_PARAMS_TIMEOUT_NEXT_BT_LP_PACKET_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.timeoutNextBtLpPacket,
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.timeoutNextBtLpPacket);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamssgAntennaType,
                            SOFT_GEMINI_PARAMS_SG_ANTENNA_TYPE_DEF, SOFT_GEMINI_PARAMS_SG_ANTENNA_TYPE_MIN, SOFT_GEMINI_PARAMS_SG_ANTENNA_TYPE_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.sgAntennaType,
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.sgAntennaType);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamssignalingType,
                            SOFT_GEMINI_PARAMS_SIGNALING_TYPE_DEF, SOFT_GEMINI_PARAMS_SIGNALING_TYPE_MIN, SOFT_GEMINI_PARAMS_SIGNALING_TYPE_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.signalingType,
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.signalingType);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamsafhLeverageOn,
                            SOFT_GEMINI_PARAMS_AFH_LEVERAGE_ON_DEF, SOFT_GEMINI_PARAMS_AFH_LEVERAGE_ON_MIN, SOFT_GEMINI_PARAMS_AFH_LEVERAGE_ON_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.afhLeverageOn,
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.afhLeverageOn);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamsnumberQuietCycle,
                            SOFT_GEMINI_PARAMS_NUMBER_QUIET_CYCLE_DEF, SOFT_GEMINI_PARAMS_NUMBER_QUIET_CYCLE_MIN, SOFT_GEMINI_PARAMS_NUMBER_QUIET_CYCLE_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.numberQuietCycle,
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.numberQuietCycle);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamsmaxNumCts,
                            SOFT_GEMINI_PARAMS_MAX_NUM_CTS_DEF, SOFT_GEMINI_PARAMS_MAX_NUM_CTS_MIN, SOFT_GEMINI_PARAMS_MAX_NUM_CTS_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.maxNumCts,
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.maxNumCts);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamsnumberOfWlanPackets,
                            SOFT_GEMINI_PARAMS_NUMBER_OF_WLAN_PACKETS_DEF, SOFT_GEMINI_PARAMS_NUMBER_OF_WLAN_PACKETS_MIN, SOFT_GEMINI_PARAMS_NUMBER_OF_WLAN_PACKETS_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.numberOfWlanPackets,
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.numberOfWlanPackets);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamsnumberOfBtPackets,
                            SOFT_GEMINI_PARAMS_NUMBER_OF_BT_PACKETS_DEF, SOFT_GEMINI_PARAMS_NUMBER_OF_BT_PACKETS_MIN, SOFT_GEMINI_PARAMS_NUMBER_OF_BT_PACKETS_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.numberOfBtPackets,
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.numberOfBtPackets);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamsnumberOfMissedRxForAvalancheTrigger,
                            SOFT_GEMINI_PARAMS_RX_FOR_AVALANCHE_DEF, SOFT_GEMINI_PARAMS_RX_FOR_AVALANCHE_MIN, SOFT_GEMINI_PARAMS_RX_FOR_AVALANCHE_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.numberOfMissedRxForAvalancheTrigger,
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.numberOfMissedRxForAvalancheTrigger);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamswlanElpHpSupport,
                            SOFT_GEMINI_PARAMS_ELP_HP_DEF, SOFT_GEMINI_PARAMS_ELP_HP_MIN, SOFT_GEMINI_PARAMS_ELP_HP_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.wlanElpHpSupport,
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.wlanElpHpSupport);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamsbtAntiStarvationPeriod,
                            SOFT_GEMINI_PARAMS_ANTI_STARVE_PERIOD_DEF, SOFT_GEMINI_PARAMS_ANTI_STARVE_PERIOD_MIN, SOFT_GEMINI_PARAMS_ANTI_STARVE_PERIOD_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.btAntiStarvationPeriod,
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.btAntiStarvationPeriod);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamsbtAntiStarvationNumberOfCyclesWithinThePeriod,
                            SOFT_GEMINI_PARAMS_ANTI_STARVE_NUM_CYCLE_DEF, SOFT_GEMINI_PARAMS_ANTI_STARVE_NUM_CYCLE_MIN, SOFT_GEMINI_PARAMS_ANTI_STARVE_NUM_CYCLE_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.btAntiStarvationNumberOfCyclesWithinThePeriod ,
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.btAntiStarvationNumberOfCyclesWithinThePeriod);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamsackModeDuringBtLpInDualAnt,
                            SOFT_GEMINI_PARAMS_ACK_MODE_DEF, SOFT_GEMINI_PARAMS_ACK_MODE_MIN, SOFT_GEMINI_PARAMS_ACK_MODE_MAX,
                            sizeof p->SoftGeminiInitParams.SoftGeminiParam.ackModeDuringBtLpInDualAnt ,
                            (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.ackModeDuringBtLpInDualAnt);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamsallowPaSdToggleDuringBtActivityEnable,
        SOFT_GEMINI_PARAMS_ALLOW_PA_SD_DEF, SOFT_GEMINI_PARAMS_ALLOW_PA_SD_MIN, SOFT_GEMINI_PARAMS_ALLOW_PA_SD_MAX,
        sizeof p->SoftGeminiInitParams.SoftGeminiParam.allowPaSdToggleDuringBtActivityEnable ,
        (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.allowPaSdToggleDuringBtActivityEnable);
    
    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamswakeUpTimeBeforeBeacon,
        SOFT_GEMINI_PARAMS_TIME_BEFORE_BEACON_DEF, SOFT_GEMINI_PARAMS_TIME_BEFORE_BEACON_MIN, SOFT_GEMINI_PARAMS_TIME_BEFORE_BEACON_MAX,
        sizeof p->SoftGeminiInitParams.SoftGeminiParam.wakeUpTimeBeforeBeacon ,
        (PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.wakeUpTimeBeforeBeacon);

	regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamshpdmMaxGuardTime,
		SOFT_GEMINI_PARAMS_HPDM_MAX_TIME_DEF, SOFT_GEMINI_PARAMS_HPDM_MAX_TIME_MIN, SOFT_GEMINI_PARAMS_HPDM_MAX_TIME_MAX,
		sizeof p->SoftGeminiInitParams.SoftGeminiParam.hpdmMaxGuardTime ,
		(PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.hpdmMaxGuardTime);

	regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamstimeoutNextWlanPacket,
		SOFT_GEMINI_PARAMS_TIME_OUT_NEXT_WLAN_DEF, SOFT_GEMINI_PARAMS_TIME_OUT_NEXT_WLAN_MIN, SOFT_GEMINI_PARAMS_TIME_OUT_NEXT_WLAN_MAX,
		sizeof p->SoftGeminiInitParams.SoftGeminiParam.timeoutNextWlanPacket ,
		(PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.timeoutNextWlanPacket);

	regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamssgAutoModeNoCts,
		SOFT_GEMINI_PARAMS_AUTO_MODE_NO_CTS_DEF, SOFT_GEMINI_PARAMS_AUTO_MODE_NO_CTS_MIN, SOFT_GEMINI_PARAMS_AUTO_MODE_NO_CTS_MAX,
		sizeof p->SoftGeminiInitParams.SoftGeminiParam.sgAutoModeNoCts ,
		(PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.sgAutoModeNoCts);

	regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamsnumOfBtHpRespectedReq,
		SOFT_GEMINI_PARAMS_BT_HP_RESPECTED_DEF, SOFT_GEMINI_PARAMS_BT_HP_RESPECTED_MIN, SOFT_GEMINI_PARAMS_BT_HP_RESPECTED_MAX,
		sizeof p->SoftGeminiInitParams.SoftGeminiParam.numOfBtHpRespectedReq ,
		(PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.numOfBtHpRespectedReq);

	regReadIntegerParameter(pAdapter, &STRBThWlanCoexistParamswlanRxMinRateToRespectBtHp,
		SOFT_GEMINI_PARAMS_WLAN_RX_MIN_RATE_DEF, SOFT_GEMINI_PARAMS_WLAN_RX_MIN_RATE_MIN, SOFT_GEMINI_PARAMS_WLAN_RX_MIN_RATE_MAX,
		sizeof p->SoftGeminiInitParams.SoftGeminiParam.wlanRxMinRateToRespectBtHp ,
		(PUCHAR)&p->SoftGeminiInitParams.SoftGeminiParam.wlanRxMinRateToRespectBtHp);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistScanNumberOfProbes,
                            SOFT_GEMINI_SCAN_NUMBER_OF_PROBE_REQUEST_DEF, SOFT_GEMINI_SCAN_NUMBER_OF_PROBE_REQUEST_MIN, SOFT_GEMINI_SCAN_NUMBER_OF_PROBE_REQUEST_MAX,
                            sizeof p->SoftGeminiInitParams.scanNumOfProbeRequest, 
                            (PUCHAR)&p->SoftGeminiInitParams.scanNumOfProbeRequest);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistScanCompensationPercent,
                            SOFT_GEMINI_SCAN_COMPENSATION_PERCENT_DEF, SOFT_GEMINI_SCAN_COMPENSATION_PERCENT_MIN, SOFT_GEMINI_SCAN_COMPENSATION_PERCENT_MAX,
                            sizeof p->SoftGeminiInitParams.scanCompensationPercent, 
                            (PUCHAR)&p->SoftGeminiInitParams.scanCompensationPercent);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistScanCompensationMaxTime,
                            SOFT_GEMINI_SCAN_COMPENSATION_MAX_TIME_DEF, SOFT_GEMINI_SCAN_COMPENSATION_MAX_TIME_MIN, SOFT_GEMINI_SCAN_COMPENSATION_MAX_TIME_MAX,
                            sizeof p->SoftGeminiInitParams.scanCompensationMaxTime,
                            (PUCHAR)&p->SoftGeminiInitParams.scanCompensationMaxTime);

    regReadIntegerParameter(pAdapter, &STRBThWlanCoexistBSSLossCompensationPercent,
                            SOFT_GEMINI_BSS_LOSS_COMPENSATION_PERCENT_DEF, SOFT_GEMINI_BSS_LOSS_COMPENSATION_PERCENT_MIN, SOFT_GEMINI_BSS_LOSS_COMPENSATION_PERCENT_MAX,
                            sizeof p->SoftGeminiInitParams.BSSLossCompensationPercent,
                            (PUCHAR)&p->SoftGeminiInitParams.BSSLossCompensationPercent);


    /* update hal 4x params */
    p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlEnable4x = p->ctrlDataInitParams.ctrlDataFourXEnable;

    /*
    Power Manager
    */
    regReadIntegerParameter(pAdapter,
                            &STRPowerMode,
                            POWER_MODE_DEF_VALUE,
                            POWER_MODE_MIN_VALUE,
                            POWER_MODE_MAX_VALUE,
                            sizeof p->PowerMgrInitParams.powerMode,
                            (PUCHAR)&p->PowerMgrInitParams.powerMode);

    regReadIntegerParameter(pAdapter,
                            &STRBeaconReceiveTime,
                            BEACON_RECEIVE_TIME_DEF_VALUE,
                            BEACON_RECEIVE_TIME_MIN_VALUE,
                            BEACON_RECEIVE_TIME_MAX_VALUE,
                            sizeof p->PowerMgrInitParams.beaconReceiveTime,
                            (PUCHAR)&p->PowerMgrInitParams.beaconReceiveTime);

    regReadIntegerParameter(pAdapter,
                            &STRBaseBandWakeUpTime,
                            BASE_BAND_WAKE_UP_TIME_DEF_VALUE,
                            BASE_BAND_WAKE_UP_TIME_MIN_VALUE,
                            BASE_BAND_WAKE_UP_TIME_MAX_VALUE,
                            sizeof p->PowerMgrInitParams.BaseBandWakeUpTime,
                            (PUCHAR)&p->PowerMgrInitParams.BaseBandWakeUpTime);

    regReadIntegerParameter(pAdapter,
                            &STRHangoverPeriod,
                            HANGOVER_PERIOD_DEF_VALUE,
                            HANGOVER_PERIOD_MIN_VALUE,
                            HANGOVER_PERIOD_MAX_VALUE,
                            sizeof p->PowerMgrInitParams.hangoverPeriod,
                            (PUCHAR)&p->PowerMgrInitParams.hangoverPeriod);

    regReadIntegerParameter(pAdapter,
                            &STRBeaconListenInterval,
                            BEACON_LISTEN_INTERVAL_DEF_VALUE,
                            BEACON_LISTEN_INTERVAL_MIN_VALUE,
                            BEACON_LISTEN_INTERVAL_MAX_VALUE,
                            sizeof p->PowerMgrInitParams.beaconListenInterval,
                            (PUCHAR)&p->PowerMgrInitParams.beaconListenInterval);

    regReadIntegerParameter(pAdapter,
                            &STRDtimListenInterval,
                            DTIM_LISTEN_INTERVAL_DEF_VALUE,
                            DTIM_LISTEN_INTERVAL_MIN_VALUE,
                            DTIM_LISTEN_INTERVAL_MAX_VALUE,
                            sizeof p->PowerMgrInitParams.dtimListenInterval,
                            (PUCHAR)&p->PowerMgrInitParams.dtimListenInterval);

    regReadIntegerParameter(pAdapter,
                            &STRNConsecutiveBeaconsMissed,
                            N_CONSECUTIVE_BEACONS_MISSED_DEF_VALUE,
                            N_CONSECUTIVE_BEACONS_MISSED_MIN_VALUE,
                            N_CONSECUTIVE_BEACONS_MISSED_MAX_VALUE,
                            sizeof p->PowerMgrInitParams.nConsecutiveBeaconsMissed,
                            (PUCHAR)&p->PowerMgrInitParams.nConsecutiveBeaconsMissed);

    regReadIntegerParameter(pAdapter,
                            &STREnterTo802_11PsRetries,
                            ENTER_TO_802_11_POWER_SAVE_RETRIES_DEF_VALUE,
                            ENTER_TO_802_11_POWER_SAVE_RETRIES_MIN_VALUE,
                            ENTER_TO_802_11_POWER_SAVE_RETRIES_MAX_VALUE,
                            sizeof p->PowerMgrInitParams.EnterTo802_11PsRetries,
                            (PUCHAR)&p->PowerMgrInitParams.EnterTo802_11PsRetries);

    regReadIntegerParameter(pAdapter,
                            &STRAutoPowerModeInterval,
                            AUTO_POWER_MODE_INTERVAL_DEF_VALUE,
                            AUTO_POWER_MODE_INTERVAL_MIN_VALUE,
                            AUTO_POWER_MODE_INTERVAL_MAX_VALUE,
                            sizeof p->PowerMgrInitParams.autoModeInterval,
                            (PUCHAR)&p->PowerMgrInitParams.autoModeInterval);

    regReadIntegerParameter(pAdapter,
                            &STRAutoPowerModeActiveTh,
                            AUTO_POWER_MODE_ACTIVE_TH_DEF_VALUE,
                            AUTO_POWER_MODE_ACTIVE_TH_MIN_VALUE,
                            AUTO_POWER_MODE_ACTIVE_TH_MAX_VALUE,
                            sizeof p->PowerMgrInitParams.autoModeActiveTH,
                            (PUCHAR)&p->PowerMgrInitParams.autoModeActiveTH);

    regReadIntegerParameter(pAdapter,
                            &STRAutoPowerModeDozeTh,
                            AUTO_POWER_MODE_DOZE_TH_DEF_VALUE,
                            AUTO_POWER_MODE_DOZE_TH_MIN_VALUE,
                            AUTO_POWER_MODE_DOZE_TH_MAX_VALUE,
                            sizeof p->PowerMgrInitParams.autoModeDozeTH,
                            (PUCHAR)&p->PowerMgrInitParams.autoModeDozeTH);

    regReadIntegerParameter(pAdapter,
                            &STRAutoPowerModeDozeMode,
                            AUTO_POWER_MODE_DOZE_MODE_DEF_VALUE,
                            AUTO_POWER_MODE_DOZE_MODE_MIN_VALUE,
                            AUTO_POWER_MODE_DOZE_MODE_MAX_VALUE,
                            sizeof p->PowerMgrInitParams.autoModeDozeMode,
                            (PUCHAR)&p->PowerMgrInitParams.autoModeDozeMode);

    regReadIntegerParameter(pAdapter,
                            &STRDefaultPowerLevel,
                            DEFAULT_POWER_LEVEL_DEF_VALUE,
                            DEFAULT_POWER_LEVEL_MIN_VALUE,
                            DEFAULT_POWER_LEVEL_MAX_VALUE,
                            sizeof p->PowerMgrInitParams.defaultPowerLevel,
                            (PUCHAR)&p->PowerMgrInitParams.defaultPowerLevel);

    regReadIntegerParameter(pAdapter,
                            &STRPowerSavePowerLevel,
                            PS_POWER_LEVEL_DEF_VALUE,
                            PS_POWER_LEVEL_MIN_VALUE,
                            PS_POWER_LEVEL_MAX_VALUE,
                            sizeof p->PowerMgrInitParams.PowerSavePowerLevel,
                            (PUCHAR)&p->PowerMgrInitParams.PowerSavePowerLevel);

/*---------------------- Power Management Configuration -----------------------*/
    regReadIntegerParameter(pAdapter,
                            &STRPowerMgmtHangOverPeriod,
                            HANGOVER_PERIOD_DEF_VALUE,
                            HANGOVER_PERIOD_MIN_VALUE,
                            HANGOVER_PERIOD_MAX_VALUE,
                            sizeof p->PowerMgrInitParams.hangOverPeriod,
                            (PUCHAR)&p->PowerMgrInitParams.hangOverPeriod);

    regReadIntegerParameter(pAdapter,
                            &STRPowerMgmtNeedToSendNullData,
                            POWER_MGMNT_NEED_TO_SEND_NULL_PACKET_DEF_VALUE,
                            POWER_MGMNT_NEED_TO_SEND_NULL_PACKET_MIN_VALUE,
                            POWER_MGMNT_NEED_TO_SEND_NULL_PACKET_MAX_VALUE,
                            sizeof p->PowerMgrInitParams.needToSendNullData,
                            (PUCHAR)&p->PowerMgrInitParams.needToSendNullData);
    
    regReadIntegerParameter(pAdapter,
                            &STRPowerMgmtNullPktRateModulation,
                            POWER_MGMNT_NULL_PACKET_RATE_MOD_DEF_VALUE,
                            POWER_MGMNT_NULL_PACKET_RATE_MOD_MIN_VALUE,
                            POWER_MGMNT_NULL_PACKET_RATE_MOD_MAX_VALUE,
                            sizeof p->PowerMgrInitParams.NullPktRateModulation,
                            (PUCHAR)&p->PowerMgrInitParams.NullPktRateModulation);
    
    regReadIntegerParameter(pAdapter,
                            &STRPowerMgmtNumNullPktRetries,
                            POWER_MGMNT_NUM_NULL_PACKET_RETRY_DEF_VALUE,
                            POWER_MGMNT_NUM_NULL_PACKET_RETRY_MIN_VALUE,
                            POWER_MGMNT_NUM_NULL_PACKET_RETRY_MAX_VALUE,
                            sizeof p->PowerMgrInitParams.numNullPktRetries,
                            (PUCHAR)&p->PowerMgrInitParams.numNullPktRetries);

    regReadIntegerParameter(pAdapter,
                            &STRPowerMgmtPllLockTime,
                            PLL_LOCK_TIME_DEF_VALUE,
                            PLL_LOCK_TIME_MIN_VALUE,
                            PLL_LOCK_TIME_MAX_VALUE,
                            sizeof p->PowerMgrInitParams.PLLlockTime,
                            (PUCHAR)&p->PowerMgrInitParams.PLLlockTime);

    regReadIntegerParameter(pAdapter,
                            &STRPsPollDeliveryFailureRecoveryPeriod,
                            PS_POLL_FAILURE_PERIOD_DEF,
                            PS_POLL_FAILURE_PERIOD_MIN,
                            PS_POLL_FAILURE_PERIOD_MAX,
                            sizeof p->PowerMgrInitParams.PsPollDeliveryFailureRecoveryPeriod,
                            (PUCHAR)&p->PowerMgrInitParams.PsPollDeliveryFailureRecoveryPeriod);

        
    /*--------------- Power Management Wake up conditions ------------------*/

    regReadIntegerParameter(pAdapter, &STRListenInterval,
                            HAL_CTRL_LISTEN_INTERVAL_DEF, HAL_CTRL_LISTEN_INTERVAL_MIN,
                            HAL_CTRL_LISTEN_INTERVAL_MAX,
                            sizeof p->PowerMgrInitParams.listenInterval,
                            (PUCHAR)&p->PowerMgrInitParams.listenInterval);

    /*-----------------------------------------------------------------------*/

    /*--------------- Power Server Init Parameters ------------------*/
    regReadIntegerParameter(pAdapter,
                            &STRPowerMgmtNumNullPktRetries,
                            POWER_MGMNT_NUM_NULL_PACKET_RETRY_DEF_VALUE,
                            POWER_MGMNT_NUM_NULL_PACKET_RETRY_MIN_VALUE,
                            POWER_MGMNT_NUM_NULL_PACKET_RETRY_MAX_VALUE,
                            sizeof p->TnetwDrv_InitParams.PowerSrvInitParams.numNullPktRetries,
                            (PUCHAR)&p->TnetwDrv_InitParams.PowerSrvInitParams.numNullPktRetries);

        regReadIntegerParameter(pAdapter,
                            &STRPowerMgmtHangOverPeriod,
                            HANGOVER_PERIOD_DEF_VALUE,
                            HANGOVER_PERIOD_MIN_VALUE,
                            HANGOVER_PERIOD_MAX_VALUE,
                            sizeof p->TnetwDrv_InitParams.PowerSrvInitParams.hangOverPeriod,
                            (PUCHAR)&p->TnetwDrv_InitParams.PowerSrvInitParams.hangOverPeriod);
    /*-----------------------------------------------------------------------*/
    

    /* Scan SRV */
    regReadIntegerParameter(pAdapter, &STRNumberOfNoScanCompleteToRecovery,
                            SCAN_SRV_NUMBER_OF_NO_SCAN_COMPLETE_TO_RECOVERY_DEF,
                            SCAN_SRV_NUMBER_OF_NO_SCAN_COMPLETE_TO_RECOVERY_MIN,
                            SCAN_SRV_NUMBER_OF_NO_SCAN_COMPLETE_TO_RECOVERY_MAX,
                            sizeof (p->TnetwDrv_InitParams.scanSrvInitParams.numberOfNoScanCompleteToRecovery),
                            (PUCHAR)&(p->TnetwDrv_InitParams.scanSrvInitParams.numberOfNoScanCompleteToRecovery) );

    regReadIntegerParameter(pAdapter, &STRTriggeredScanTimeOut,
        SCAN_SRV_TRIGGERED_SCAN_TIME_OUT_DEF,
        SCAN_SRV_TRIGGERED_SCAN_TIME_OUT_MIN,
        SCAN_SRV_TRIGGERED_SCAN_TIME_OUT_MAX,
        sizeof (p->TnetwDrv_InitParams.scanSrvInitParams.uTriggeredScanTimeOut),
        (PUCHAR)&(p->TnetwDrv_InitParams.scanSrvInitParams.uTriggeredScanTimeOut) );

    /* Regulatory Domain */

    /* Indicate the time in which the STA didn't receive any country code and was not connected, and therefore
       will delete its current country code */
    regReadIntegerParameter(pAdapter, &STRTimeToResetCountryMs,
                        REGULATORY_DOMAIN_COUNTRY_TIME_RESET_DEF, REGULATORY_DOMAIN_COUNTRY_TIME_RESET_MIN,
                        REGULATORY_DOMAIN_COUNTRY_TIME_RESET_MAX, 
                        sizeof p->regulatoryDomainInitParams.uTimeOutToResetCountryMs,
                        (PUCHAR)&(p->regulatoryDomainInitParams.uTimeOutToResetCountryMs));

    /* 802.11d/h */
    regReadIntegerParameter(pAdapter, &STRMultiRegulatoryDomainEnabled,
                            MULTI_REGULATORY_DOMAIN_ENABLED_DEF, MULTI_REGULATORY_DOMAIN_ENABLED_MIN,
                            MULTI_REGULATORY_DOMAIN_ENABLED_MAX, 
                            sizeof p->regulatoryDomainInitParams.multiRegulatoryDomainEnabled,
                            (PUCHAR)&(p->regulatoryDomainInitParams.multiRegulatoryDomainEnabled));

    regReadIntegerParameter(pAdapter, &STRSpectrumManagementEnabled,
                            SPECTRUM_MANAGEMENT_ENABLED_DEF, SPECTRUM_MANAGEMENT_ENABLED_MIN,
                            SPECTRUM_MANAGEMENT_ENABLED_MAX, 
                            sizeof p->regulatoryDomainInitParams.spectrumManagementEnabled,
                            (PUCHAR)&(p->regulatoryDomainInitParams.spectrumManagementEnabled));

    regReadIntegerParameter(pAdapter, &STRSpectrumManagementEnabled,
                            SPECTRUM_MANAGEMENT_ENABLED_DEF, SPECTRUM_MANAGEMENT_ENABLED_MIN,
                            SPECTRUM_MANAGEMENT_ENABLED_MAX, 
                            sizeof p->SwitchChannelInitParams.dot11SpectrumManagementRequired,
                            (PUCHAR)&(p->SwitchChannelInitParams.dot11SpectrumManagementRequired));


    /* Scan Control Tables */
    regReadStringParameter(pAdapter, &STRScanControlTable24,
                           (PCHAR)&ScanControlTable24Def[0],(USHORT)(2 * NUM_OF_CHANNELS_24),
                            (PUCHAR)&(ScanControlTable24Tmp[0]),
                            (PUSHORT)&tableLen);

    for( loopIndex = tableLen ; loopIndex < 2 * NUM_OF_CHANNELS_24 ; loopIndex++)
        ScanControlTable24Tmp[loopIndex] = '0';

    decryptScanControlTable(ScanControlTable24Tmp,(PUCHAR)&(p->regulatoryDomainInitParams.desiredScanControlTable.ScanControlTable24.tableString[0]),2 * NUM_OF_CHANNELS_24);


    /* Scan Control Tables for 5 Ghz*/
    regReadStringParameter(pAdapter, &STRScanControlTable5,
                           (PCHAR)&ScanControlTable5Def[0],(USHORT)(2 * A_5G_BAND_NUM_CHANNELS),
                            (PUCHAR)&(ScanControlTable5Tmp[0]),
                            (PUSHORT)&tableLen);


    for( loopIndex = tableLen ; loopIndex < 2 * A_5G_BAND_NUM_CHANNELS ; loopIndex++)
        ScanControlTable5Tmp[loopIndex] = '0';

    decryptScanControlTable(ScanControlTable5Tmp,(PUCHAR)&(p->regulatoryDomainInitParams.desiredScanControlTable.ScanControlTable5.tableString[0]),2 * A_5G_BAND_NUM_CHANNELS);


    /* Tx Power */
    regReadIntegerParameter(pAdapter, &STRTxPower,
                            MAX_TX_POWER, MIN_TX_POWER, MAX_TX_POWER,
                            sizeof p->regulatoryDomainInitParams.desiredTxPower,
                            (PUCHAR)&p->regulatoryDomainInitParams.desiredTxPower);

    regReadIntegerParameter(pAdapter, &STRdot11WEPStatus,
                            RSN_WEP_STATUS_DEF, RSN_WEP_STATUS_MIN, RSN_WEP_STATUS_MAX,
                            sizeof p->rsnInitParams.privacyOn,
                            (PUCHAR)&p->rsnInitParams.privacyOn);
    /* reverse privacy value - windows is setting 1 as off */
    /*
        p->rsnInitParams.privacyMode = !(p->rsnInitParams.privacyOn);
        p->rsnInitParams.privacyOn = !(p->rsnInitParams.privacyOn);
    */

    regReadIntegerParameter(pAdapter, &STRdot11WEPDefaultKeyID,
                            RSN_DEFAULT_KEY_ID_DEF, RSN_DEFAULT_KEY_ID_MIN,
                            RSN_DEFAULT_KEY_ID_MAX,
                            sizeof p->rsnInitParams.defaultKeyId,
                            (PUCHAR)&p->rsnInitParams.defaultKeyId);


    regReadIntegerParameter(pAdapter, &STRMixedMode,
                            RSN_WEPMIXEDMODE_ENABLED_DEF, RSN_WEPMIXEDMODE_ENABLED_MIN,
                            RSN_WEPMIXEDMODE_ENABLED_MAX,
                            sizeof p->rsnInitParams.mixedMode,
                            (PUCHAR)&p->rsnInitParams.mixedMode);

    regReadIntegerParameter(pAdapter, &STRWPAMixedMode,
                            RSN_WPAMIXEDMODE_ENABLE_DEF, RSN_WPAMIXEDMODE_ENABLE_MIN, 
                            RSN_WPAMIXEDMODE_ENABLE_MAX,                         
                            sizeof p->rsnInitParams.WPAMixedModeEnable,
                            (PUCHAR)&p->rsnInitParams.WPAMixedModeEnable);         

    regReadIntegerParameter(pAdapter, &STRRSNPreAuth,
                            RSN_PREAUTH_ENABLE_DEF, RSN_PREAUTH_ENABLE_MIN,
                            RSN_PREAUTH_ENABLE_MAX,
                            sizeof p->rsnInitParams.preAuthSupport,
                            (PUCHAR)&p->rsnInitParams.preAuthSupport);

    regReadIntegerParameter(pAdapter, &STRRSNPreAuthTimeout,
                            RSN_PREAUTH_TIMEOUT_DEF, RSN_PREAUTH_TIMEOUT_MIN,
                            RSN_PREAUTH_TIMEOUT_MAX,
                            sizeof p->rsnInitParams.preAuthTimeout,
                            (PUCHAR)&p->rsnInitParams.preAuthTimeout);

    regReadWepKeyParameter(pAdapter, (PUCHAR)p->rsnInitParams.keys, p->rsnInitParams.defaultKeyId);


    /*---------------------------
            QOS parameters
    -----------------------------*/

    regReadIntegerParameter(pAdapter, &STRClsfr_Type,
                            CLSFR_TYPE_DEF, CLSFR_TYPE_MIN, 
                            CLSFR_TYPE_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.clsfrType,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.clsfrType); 

    switch(p->ctrlDataInitParams.ClsfrInitParam.clsfrType)
    {
        case D_TAG_CLSFR:
            /* Trivial mapping D-tag to D-tag - no need to read more keys*/
        break;

        case DSCP_CLSFR:

            regReadIntegerParameter(pAdapter, &STRNumOfCodePoints,
                            NUM_OF_CODE_POINTS_DEF, NUM_OF_CODE_POINTS_MIN, 
                            NUM_OF_CODE_POINTS_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.NumOfActiveEntries,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.NumOfActiveEntries);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier00_CodePoint,
                            DSCP_CLASSIFIER_CODE_POINT_DEF, CLASSIFIER_CODE_POINT_MIN, 
                            CLASSIFIER_CODE_POINT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[0].Dscp.CodePoint,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[0].Dscp.CodePoint);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier01_CodePoint,
                            DSCP_CLASSIFIER_CODE_POINT_DEF, CLASSIFIER_CODE_POINT_MIN, 
                            CLASSIFIER_CODE_POINT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[1].Dscp.CodePoint,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[1].Dscp.CodePoint);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier02_CodePoint,
                            DSCP_CLASSIFIER_CODE_POINT_DEF, CLASSIFIER_CODE_POINT_MIN, 
                            CLASSIFIER_CODE_POINT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[2].Dscp.CodePoint,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[2].Dscp.CodePoint);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier03_CodePoint,
                            DSCP_CLASSIFIER_CODE_POINT_DEF, CLASSIFIER_CODE_POINT_MIN, 
                            CLASSIFIER_CODE_POINT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[3].Dscp.CodePoint,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[3].Dscp.CodePoint);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier04_CodePoint,
                            DSCP_CLASSIFIER_CODE_POINT_DEF, CLASSIFIER_CODE_POINT_MIN, 
                            CLASSIFIER_CODE_POINT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[4].Dscp.CodePoint,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[4].Dscp.CodePoint);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier05_CodePoint,
                            DSCP_CLASSIFIER_CODE_POINT_DEF, CLASSIFIER_CODE_POINT_MIN, 
                            CLASSIFIER_CODE_POINT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[5].Dscp.CodePoint,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[5].Dscp.CodePoint);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier06_CodePoint,
                            DSCP_CLASSIFIER_CODE_POINT_DEF, CLASSIFIER_CODE_POINT_MIN, 
                            CLASSIFIER_CODE_POINT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[6].Dscp.CodePoint,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[6].Dscp.CodePoint);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier07_CodePoint,
                            DSCP_CLASSIFIER_CODE_POINT_DEF, CLASSIFIER_CODE_POINT_MIN, 
                            CLASSIFIER_CODE_POINT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[7].Dscp.CodePoint,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[7].Dscp.CodePoint);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier08_CodePoint,
                            CLASSIFIER_CODE_POINT_DEF, CLASSIFIER_CODE_POINT_MIN, 
                            CLASSIFIER_CODE_POINT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[8].Dscp.CodePoint,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[8].Dscp.CodePoint);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier09_CodePoint,
                            CLASSIFIER_CODE_POINT_DEF, CLASSIFIER_CODE_POINT_MIN, 
                            CLASSIFIER_CODE_POINT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[9].Dscp.CodePoint,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[9].Dscp.CodePoint);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier10_CodePoint,
                            CLASSIFIER_CODE_POINT_DEF, CLASSIFIER_CODE_POINT_MIN, 
                            CLASSIFIER_CODE_POINT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[10].Dscp.CodePoint,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[10].Dscp.CodePoint);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier11_CodePoint,
                            CLASSIFIER_CODE_POINT_DEF, CLASSIFIER_CODE_POINT_MIN, 
                            CLASSIFIER_CODE_POINT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[11].Dscp.CodePoint,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[11].Dscp.CodePoint);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier12_CodePoint,
                            CLASSIFIER_CODE_POINT_DEF, CLASSIFIER_CODE_POINT_MIN, 
                            CLASSIFIER_CODE_POINT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[12].Dscp.CodePoint,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[12].Dscp.CodePoint);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier13_CodePoint,
                            CLASSIFIER_CODE_POINT_DEF, CLASSIFIER_CODE_POINT_MIN, 
                            CLASSIFIER_CODE_POINT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[13].Dscp.CodePoint,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[13].Dscp.CodePoint);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier14_CodePoint,
                            CLASSIFIER_CODE_POINT_DEF, CLASSIFIER_CODE_POINT_MIN, 
                            CLASSIFIER_CODE_POINT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[14].Dscp.CodePoint,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[14].Dscp.CodePoint);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier15_CodePoint,
                            CLASSIFIER_CODE_POINT_DEF, CLASSIFIER_CODE_POINT_MIN, 
                            CLASSIFIER_CODE_POINT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[15].Dscp.CodePoint,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[15].Dscp.CodePoint);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier00_DTag,
                            DSCP_CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[0].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[0].DTag);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier01_DTag,
                            DSCP_CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[1].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[1].DTag);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier02_DTag,
                            DSCP_CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[2].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[2].DTag);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier03_DTag,
                            DSCP_CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[3].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[3].DTag);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier04_DTag,
                            DSCP_CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[4].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[4].DTag);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier05_DTag,
                            DSCP_CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[5].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[5].DTag);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier06_DTag,
                            DSCP_CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[6].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[6].DTag);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier07_DTag,
                            DSCP_CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[7].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[7].DTag);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier08_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[8].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[8].DTag);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier09_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[9].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[9].DTag);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier10_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[10].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[10].DTag);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier11_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[11].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[11].DTag);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier12_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[12].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[12].DTag);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier13_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[13].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[13].DTag);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier14_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[14].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[14].DTag);
            regReadIntegerParameter(pAdapter, &STRDSCPClassifier15_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[15].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[15].DTag);


        break;

        case PORT_CLSFR:

            regReadIntegerParameter(pAdapter, &STRNumOfDstPortClassifiers,
                            NUM_OF_PORT_CLASSIFIERS_DEF, NUM_OF_PORT_CLASSIFIERS_MIN, 
                            NUM_OF_PORT_CLASSIFIERS_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.NumOfActiveEntries,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.NumOfActiveEntries);
            regReadIntegerParameter(pAdapter, &STRPortClassifier00_Port,
                            PORT_CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[0].Dscp.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[0].Dscp.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRPortClassifier01_Port,
                            PORT_CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[1].Dscp.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[1].Dscp.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRPortClassifier02_Port,
                            PORT_CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[2].Dscp.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[2].Dscp.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRPortClassifier03_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[3].Dscp.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[3].Dscp.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRPortClassifier04_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[4].Dscp.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[4].Dscp.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRPortClassifier05_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[5].Dscp.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[5].Dscp.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRPortClassifier06_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[6].Dscp.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[6].Dscp.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRPortClassifier07_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[7].Dscp.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[7].Dscp.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRPortClassifier08_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[8].Dscp.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[8].Dscp.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRPortClassifier09_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[9].Dscp.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[9].Dscp.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRPortClassifier10_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[10].Dscp.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[10].Dscp.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRPortClassifier11_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[11].Dscp.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[11].Dscp.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRPortClassifier12_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[12].Dscp.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[12].Dscp.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRPortClassifier13_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[13].Dscp.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[13].Dscp.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRPortClassifier14_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[14].Dscp.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[14].Dscp.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRPortClassifier15_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[15].Dscp.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[15].Dscp.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRPortClassifier00_DTag,
                            PORT_CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[0].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[0].DTag);
            regReadIntegerParameter(pAdapter, &STRPortClassifier01_DTag,
                            PORT_CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[1].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[1].DTag);
            regReadIntegerParameter(pAdapter, &STRPortClassifier02_DTag,
                            PORT_CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[2].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[2].DTag);
            regReadIntegerParameter(pAdapter, &STRPortClassifier03_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[3].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[3].DTag);
            regReadIntegerParameter(pAdapter, &STRPortClassifier04_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[4].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[4].DTag);
            regReadIntegerParameter(pAdapter, &STRPortClassifier05_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[5].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[5].DTag);
            regReadIntegerParameter(pAdapter, &STRPortClassifier06_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[6].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[6].DTag);
            regReadIntegerParameter(pAdapter, &STRPortClassifier07_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[7].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[7].DTag);
            regReadIntegerParameter(pAdapter, &STRPortClassifier08_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[8].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[8].DTag);
            regReadIntegerParameter(pAdapter, &STRPortClassifier09_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[9].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[9].DTag);
            regReadIntegerParameter(pAdapter, &STRPortClassifier10_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[10].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[10].DTag);
            regReadIntegerParameter(pAdapter, &STRPortClassifier11_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[11].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[11].DTag);
            regReadIntegerParameter(pAdapter, &STRPortClassifier12_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[12].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[12].DTag);
            regReadIntegerParameter(pAdapter, &STRPortClassifier13_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[13].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[13].DTag);
            regReadIntegerParameter(pAdapter, &STRPortClassifier14_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[14].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[14].DTag);
            regReadIntegerParameter(pAdapter, &STRPortClassifier15_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[15].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[15].DTag);
    
        break;


        case IPPORT_CLSFR: 

            regReadIntegerParameter(pAdapter, &STRNumOfDstIPPortClassifiers,
                            NUM_OF_IPPORT_CLASSIFIERS_DEF, NUM_OF_IPPORT_CLASSIFIERS_MIN, 
                            NUM_OF_IPPORT_CLASSIFIERS_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.NumOfActiveEntries,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.NumOfActiveEntries);

            regReadStringParameter(pAdapter, &STRIPPortClassifier00_IPAddress, (PCHAR)(ClsfrIp), 11, (PUCHAR)ClsfrIpString, &ClsfrIpStringSize);
            initRadioValusFromRgstryString( (PCHAR)(ClsfrIpString), (PCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[0].Dscp.DstIPPort.DstIPAddress, 4);

            regReadStringParameter(pAdapter, &STRIPPortClassifier01_IPAddress, (PCHAR)(ClsfrIp), 11, (PUCHAR)ClsfrIpString, &ClsfrIpStringSize);
            initRadioValusFromRgstryString( (PCHAR)(ClsfrIpString), (PCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[1].Dscp.DstIPPort.DstIPAddress, 4);

            regReadStringParameter(pAdapter, &STRIPPortClassifier02_IPAddress, (PCHAR)(ClsfrIp), 11, (PUCHAR)ClsfrIpString, &ClsfrIpStringSize);
            initRadioValusFromRgstryString( (PCHAR)(ClsfrIpString), (PCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[2].Dscp.DstIPPort.DstIPAddress, 4);

            regReadStringParameter(pAdapter, &STRIPPortClassifier03_IPAddress, (PCHAR)(ClsfrIp), 11, (PUCHAR)ClsfrIpString, &ClsfrIpStringSize);
            initRadioValusFromRgstryString( (PCHAR)(ClsfrIpString), (PCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[3].Dscp.DstIPPort.DstIPAddress, 4);

            regReadStringParameter(pAdapter, &STRIPPortClassifier04_IPAddress, (PCHAR)(ClsfrIp), 11, (PUCHAR)ClsfrIpString, &ClsfrIpStringSize);
            initRadioValusFromRgstryString( (PCHAR)(ClsfrIpString), (PCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[4].Dscp.DstIPPort.DstIPAddress, 4);

            regReadStringParameter(pAdapter, &STRIPPortClassifier05_IPAddress, (PCHAR)(ClsfrIp), 11, (PUCHAR)ClsfrIpString, &ClsfrIpStringSize);
            initRadioValusFromRgstryString( (PCHAR)(ClsfrIpString), (PCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[5].Dscp.DstIPPort.DstIPAddress, 4);

            regReadStringParameter(pAdapter, &STRIPPortClassifier06_IPAddress, (PCHAR)(ClsfrIp), 11, (PUCHAR)ClsfrIpString, &ClsfrIpStringSize);
            initRadioValusFromRgstryString( (PCHAR)(ClsfrIpString), (PCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[6].Dscp.DstIPPort.DstIPAddress, 4);

            regReadStringParameter(pAdapter, &STRIPPortClassifier07_IPAddress, (PCHAR)(ClsfrIp), 11, (PUCHAR)ClsfrIpString, &ClsfrIpStringSize);
            initRadioValusFromRgstryString( (PCHAR)(ClsfrIpString), (PCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[7].Dscp.DstIPPort.DstIPAddress, 4);

            regReadStringParameter(pAdapter, &STRIPPortClassifier08_IPAddress, (PCHAR)(ClsfrIp), 11, (PUCHAR)ClsfrIpString, &ClsfrIpStringSize);
            initRadioValusFromRgstryString( (PCHAR)(ClsfrIpString), (PCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[8].Dscp.DstIPPort.DstIPAddress, 4);

            regReadStringParameter(pAdapter, &STRIPPortClassifier09_IPAddress, (PCHAR)(ClsfrIp), 11, (PUCHAR)ClsfrIpString, &ClsfrIpStringSize);
            initRadioValusFromRgstryString( (PCHAR)(ClsfrIpString), (PCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[9].Dscp.DstIPPort.DstIPAddress, 4);

            regReadStringParameter(pAdapter, &STRIPPortClassifier10_IPAddress, (PCHAR)(ClsfrIp), 11, (PUCHAR)ClsfrIpString, &ClsfrIpStringSize);
            initRadioValusFromRgstryString( (PCHAR)(ClsfrIpString), (PCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[10].Dscp.DstIPPort.DstIPAddress, 4);

            regReadStringParameter(pAdapter, &STRIPPortClassifier11_IPAddress, (PCHAR)(ClsfrIp), 11, (PUCHAR)ClsfrIpString, &ClsfrIpStringSize);
            initRadioValusFromRgstryString( (PCHAR)(ClsfrIpString), (PCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[11].Dscp.DstIPPort.DstIPAddress, 4);

            regReadStringParameter(pAdapter, &STRIPPortClassifier12_IPAddress, (PCHAR)(ClsfrIp), 11, (PUCHAR)ClsfrIpString, &ClsfrIpStringSize);
            initRadioValusFromRgstryString( (PCHAR)(ClsfrIpString), (PCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[12].Dscp.DstIPPort.DstIPAddress, 4);

            regReadStringParameter(pAdapter, &STRIPPortClassifier13_IPAddress, (PCHAR)(ClsfrIp), 11, (PUCHAR)ClsfrIpString, &ClsfrIpStringSize);
            initRadioValusFromRgstryString( (PCHAR)(ClsfrIpString), (PCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[13].Dscp.DstIPPort.DstIPAddress, 4);

            regReadStringParameter(pAdapter, &STRIPPortClassifier14_IPAddress, (PCHAR)(ClsfrIp), 11, (PUCHAR)ClsfrIpString, &ClsfrIpStringSize);
            initRadioValusFromRgstryString( (PCHAR)(ClsfrIpString), (PCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[14].Dscp.DstIPPort.DstIPAddress, 4);

            regReadStringParameter(pAdapter, &STRIPPortClassifier15_IPAddress, (PCHAR)(ClsfrIp), 11, (PUCHAR)ClsfrIpString, &ClsfrIpStringSize);
            initRadioValusFromRgstryString( (PCHAR)(ClsfrIpString), (PCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[15].Dscp.DstIPPort.DstIPAddress, 4);

            regReadIntegerParameter(pAdapter, &STRIPPortClassifier00_Port,
                            IPPORT_CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[0].Dscp.DstIPPort.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[0].Dscp.DstIPPort.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier01_Port,
                            IPPORT_CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[1].Dscp.DstIPPort.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[1].Dscp.DstIPPort.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier02_Port,
                            IPPORT_CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[2].Dscp.DstIPPort.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[2].Dscp.DstIPPort.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier03_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[3].Dscp.DstIPPort.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[3].Dscp.DstIPPort.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier04_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[4].Dscp.DstIPPort.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[4].Dscp.DstIPPort.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier05_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[5].Dscp.DstIPPort.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[5].Dscp.DstIPPort.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier06_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[6].Dscp.DstIPPort.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[6].Dscp.DstIPPort.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier07_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[7].Dscp.DstIPPort.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[7].Dscp.DstIPPort.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier08_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[8].Dscp.DstIPPort.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[8].Dscp.DstIPPort.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier09_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[9].Dscp.DstIPPort.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[9].Dscp.DstIPPort.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier10_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[10].Dscp.DstIPPort.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[10].Dscp.DstIPPort.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier11_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[11].Dscp.DstIPPort.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[11].Dscp.DstIPPort.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier12_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[12].Dscp.DstIPPort.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[12].Dscp.DstIPPort.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier13_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[13].Dscp.DstIPPort.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[13].Dscp.DstIPPort.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier14_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[14].Dscp.DstIPPort.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[14].Dscp.DstIPPort.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier15_Port,
                            CLASSIFIER_PORT_DEF, CLASSIFIER_PORT_MIN, 
                            CLASSIFIER_PORT_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[15].Dscp.DstIPPort.DstPortNum,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[15].Dscp.DstIPPort.DstPortNum);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier00_DTag,
                            IPPORT_CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[0].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[0].DTag);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier01_DTag,
                            IPPORT_CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[1].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[1].DTag);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier02_DTag,
                            IPPORT_CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[2].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[2].DTag);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier03_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[3].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[3].DTag);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier04_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[4].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[4].DTag);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier05_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[5].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[5].DTag);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier06_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[6].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[6].DTag);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier07_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[7].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[7].DTag);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier08_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[8].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[8].DTag);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier09_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[9].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[9].DTag);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier10_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[10].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[10].DTag);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier11_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[11].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[11].DTag);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier12_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[12].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[12].DTag);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier13_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[13].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[13].DTag);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier14_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[14].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[14].DTag);
            regReadIntegerParameter(pAdapter, &STRIPPortClassifier15_DTag,
                            CLASSIFIER_DTAG_DEF, CLASSIFIER_DTAG_MIN, 
                            CLASSIFIER_DTAG_MAX,                         
                            sizeof p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[15].DTag,
                            (PUCHAR)&p->ctrlDataInitParams.ClsfrInitParam.ClsfrTable[15].DTag);
    
        break;

    }



  /* ---------------------------

       Traffic Intensity Threshold 

   ---------------------------*/
    regReadIntegerParameter(pAdapter, &STRTrafficIntensityThresHigh, 
                            CTRL_DATA_TRAFFIC_THRESHOLD_HIGH_DEF, 
                            CTRL_DATA_TRAFFIC_THRESHOLD_HIGH_MIN, 
                            CTRL_DATA_TRAFFIC_THRESHOLD_HIGH_MAX, 
                            sizeof p->ctrlDataInitParams.ctrlDataTrafficThreshold.uHighThreshold, 
                            (PUCHAR)&p->ctrlDataInitParams.ctrlDataTrafficThreshold.uHighThreshold);

    regReadIntegerParameter(pAdapter, &STRTrafficIntensityThresLow, 
                            CTRL_DATA_TRAFFIC_THRESHOLD_LOW_DEF, 
                            CTRL_DATA_TRAFFIC_THRESHOLD_LOW_MIN, 
                            CTRL_DATA_TRAFFIC_THRESHOLD_LOW_MAX, 
                            sizeof p->ctrlDataInitParams.ctrlDataTrafficThreshold.uLowThreshold,
                            (PUCHAR)&p->ctrlDataInitParams.ctrlDataTrafficThreshold.uLowThreshold);

    regReadIntegerParameter(pAdapter, &STRTrafficIntensityTestInterval, 
                            CTRL_DATA_TRAFFIC_THRESHOLD_INTERVAL_DEF, 
                            CTRL_DATA_TRAFFIC_THRESHOLD_INTERVAL_MIN, 
                            CTRL_DATA_TRAFFIC_THRESHOLD_INTERVAL_MAX, 
                            sizeof p->ctrlDataInitParams.ctrlDataTrafficThreshold.TestInterval,
                            (PUCHAR)&p->ctrlDataInitParams.ctrlDataTrafficThreshold.TestInterval);

    regReadIntegerParameter(pAdapter, &STRTrafficIntensityThresholdEnabled, 
                            CTRL_DATA_TRAFFIC_THRESHOLD_ENABLED_DEF, 
                            CTRL_DATA_TRAFFIC_THRESHOLD_ENABLED_MIN, 
                            CTRL_DATA_TRAFFIC_THRESHOLD_ENABLED_MAX, 
                            sizeof p->ctrlDataInitParams.ctrlDataTrafficThresholdEnabled,
                            (PUCHAR)&p->ctrlDataInitParams.ctrlDataTrafficThresholdEnabled);

    regReadIntegerParameter(pAdapter, &STRTrafficMonitorMinIntervalPercentage, 
                            TRAFFIC_MONITOR_MIN_INTERVAL_PERCENT_DEF, 
                            TRAFFIC_MONITOR_MIN_INTERVAL_PERCENT_MIN, 
                            TRAFFIC_MONITOR_MIN_INTERVAL_PERCENT_MAX, 
                            sizeof(BOOL), (PUCHAR)&p->trafficMonitorMinIntervalPercentage);

    regReadIntegerParameter(pAdapter, &STRWMEEnable,
                            WME_ENABLED_DEF, WME_ENABLED_MIN,
                            WME_ENABLED_MAX,
                            sizeof p->qosMngrInitParams.wmeEnable,
                            (PUCHAR)&p->qosMngrInitParams.wmeEnable);

    regReadIntegerParameter(pAdapter, &STRTrafficAdmCtrlEnable,
                            QOS_TRAFFIC_ADM_CTRL_ENABLED_DEF, QOS_TRAFFIC_ADM_CTRL_ENABLED_MIN, 
                            QOS_TRAFFIC_ADM_CTRL_ENABLED_MAX,                         
                            sizeof p->qosMngrInitParams.trafficAdmCtrlEnable,
                            (PUCHAR)&p->qosMngrInitParams.trafficAdmCtrlEnable); 

    regReadIntegerParameter(pAdapter, &STRdesiredPsMode,
                            QOS_DESIRED_PS_MODE_DEF, QOS_DESIRED_PS_MODE_MIN, 
                            QOS_DESIRED_PS_MODE_MAX,                         
                            sizeof p->qosMngrInitParams.desiredPsMode,
                            (PUCHAR)&p->qosMngrInitParams.desiredPsMode); 

    regReadIntegerParameter(pAdapter, &STRQOSmsduLifeTimeBE,
                    QOS_MSDU_LIFE_TIME_BE_DEF, QOS_MSDU_LIFE_TIME_BE_MIN,
                    QOS_MSDU_LIFE_TIME_BE_MAX,
                    sizeof p->qosMngrInitParams.MsduLifeTime[QOS_AC_BE],
                    (PUCHAR)&p->qosMngrInitParams.MsduLifeTime[QOS_AC_BE]);

    regReadIntegerParameter(pAdapter, &STRQOSmsduLifeTimeBK,
                            QOS_MSDU_LIFE_TIME_BK_DEF, QOS_MSDU_LIFE_TIME_BK_MIN,
                            QOS_MSDU_LIFE_TIME_BK_MAX,
                            sizeof p->qosMngrInitParams.MsduLifeTime[QOS_AC_BK],
                            (PUCHAR)&p->qosMngrInitParams.MsduLifeTime[QOS_AC_BK]);

    regReadIntegerParameter(pAdapter, &STRQOSmsduLifeTimeVI,
                            QOS_MSDU_LIFE_TIME_VI_DEF, QOS_MSDU_LIFE_TIME_VI_MIN,
                            QOS_MSDU_LIFE_TIME_VI_MAX,
                            sizeof p->qosMngrInitParams.MsduLifeTime[QOS_AC_VI],
                            (PUCHAR)&p->qosMngrInitParams.MsduLifeTime[QOS_AC_VI]);

    regReadIntegerParameter(pAdapter, &STRQOSmsduLifeTimeVO,
                            QOS_MSDU_LIFE_TIME_VO_DEF, QOS_MSDU_LIFE_TIME_VO_MIN,
                            QOS_MSDU_LIFE_TIME_VO_MAX,
                            sizeof p->qosMngrInitParams.MsduLifeTime[QOS_AC_VO],
                            (PUCHAR)&p->qosMngrInitParams.MsduLifeTime[QOS_AC_VO]);


    regReadIntegerParameter(pAdapter, &STRQOSrxTimeOutPsPoll,
                    QOS_RX_TIMEOUT_PS_POLL_DEF, QOS_RX_TIMEOUT_PS_POLL_MIN,
                    QOS_RX_TIMEOUT_PS_POLL_MAX,
                    sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.rxTimeOut.psPoll,
                    (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.rxTimeOut.psPoll);

    regReadIntegerParameter(pAdapter, &STRQOSrxTimeOutUPSD,
                    QOS_RX_TIMEOUT_UPSD_DEF, QOS_RX_TIMEOUT_UPSD_MIN,
                    QOS_RX_TIMEOUT_UPSD_MAX,
                    sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.rxTimeOut.UPSD,
                    (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.rxTimeOut.UPSD);

    /* Note: The PsPoll wait timeout should be aligned with the UPSD setting */
   /* p->PowerMgrInitParams.HwPsPollResponseTimeout = (UINT8)p->qosMngrInitParams.rxTimeout.UPSD;*/

    regReadIntegerParameter(pAdapter, &STRQOStxQueue0Size,
                            QOS_TX_QUEUE0_SIZE_DEF, QOS_TX_QUEUE0_SIZE_MIN,
                            QOS_TX_QUEUE0_SIZE_MAX,
                            sizeof p->qosMngrInitParams.TxQueueSize[0],
                            (PUCHAR)&p->qosMngrInitParams.TxQueueSize[0]);

    regReadIntegerParameter(pAdapter, &STRQOStxQueue1Size,
                            QOS_TX_QUEUE1_SIZE_DEF, QOS_TX_QUEUE1_SIZE_MIN,
                            QOS_TX_QUEUE1_SIZE_MAX,
                            sizeof p->qosMngrInitParams.TxQueueSize[1],
                            (PUCHAR)&p->qosMngrInitParams.TxQueueSize[1]);

    regReadIntegerParameter(pAdapter, &STRQOStxQueue2Size,
                        QOS_TX_QUEUE2_SIZE_DEF, QOS_TX_QUEUE2_SIZE_MIN,
                        QOS_TX_QUEUE2_SIZE_MAX,
                        sizeof p->qosMngrInitParams.TxQueueSize[2],
                        (PUCHAR)&p->qosMngrInitParams.TxQueueSize[2]);

    regReadIntegerParameter(pAdapter, &STRQOStxQueue3Size,
                        QOS_TX_QUEUE3_SIZE_DEF, QOS_TX_QUEUE3_SIZE_MIN,
                        QOS_TX_QUEUE3_SIZE_MAX,
                        sizeof p->qosMngrInitParams.TxQueueSize[3],
                        (PUCHAR)&p->qosMngrInitParams.TxQueueSize[3]);

    regReadIntegerParameter(pAdapter, &STRQOSwmePsModeBE,
                            QOS_WME_PS_MODE_BE_DEF, QOS_WME_PS_MODE_BE_MIN,
                            QOS_WME_PS_MODE_BE_MAX,
                            sizeof p->qosMngrInitParams.desiredWmeAcPsMode[QOS_AC_BE],
                            (PUCHAR)&p->qosMngrInitParams.desiredWmeAcPsMode[QOS_AC_BE]);

    regReadIntegerParameter(pAdapter, &STRQOSwmePsModeBK,
                            QOS_WME_PS_MODE_BK_DEF, QOS_WME_PS_MODE_BK_MIN,
                            QOS_WME_PS_MODE_BK_MAX,
                            sizeof p->qosMngrInitParams.desiredWmeAcPsMode[QOS_AC_BK],
                            (PUCHAR)&p->qosMngrInitParams.desiredWmeAcPsMode[QOS_AC_BK]);

    regReadIntegerParameter(pAdapter, &STRQOSwmePsModeVI,
                        QOS_WME_PS_MODE_VI_DEF, QOS_WME_PS_MODE_VI_MIN,
                        QOS_WME_PS_MODE_VI_MAX,
                        sizeof p->qosMngrInitParams.desiredWmeAcPsMode[QOS_AC_VI],
                        (PUCHAR)&p->qosMngrInitParams.desiredWmeAcPsMode[QOS_AC_VI]);

    regReadIntegerParameter(pAdapter, &STRQOSwmePsModeVO,
                        QOS_WME_PS_MODE_VO_DEF, QOS_WME_PS_MODE_VO_MIN,
                        QOS_WME_PS_MODE_VO_MAX,
                        sizeof p->qosMngrInitParams.desiredWmeAcPsMode[QOS_AC_VO],
                        (PUCHAR)&p->qosMngrInitParams.desiredWmeAcPsMode[QOS_AC_VO]);

    /* HW Tx queues buffers allocation high threshold */
    regReadIntegerParameter(pAdapter, &STRQOStxBlksHighPrcntBE,
                            QOS_TX_BLKS_HIGH_PRCNT_BE_DEF, QOS_TX_BLKS_HIGH_PRCNT_MIN,
                            QOS_TX_BLKS_HIGH_PRCNT_MAX,
                            sizeof p->TnetwDrv_InitParams.whalCtrl_init.TxBlocksHighPercentPerAc[QOS_AC_BE],
                            (PUCHAR)&p->TnetwDrv_InitParams.whalCtrl_init.TxBlocksHighPercentPerAc[QOS_AC_BE]);

    regReadIntegerParameter(pAdapter, &STRQOStxBlksHighPrcntBK,
                            QOS_TX_BLKS_HIGH_PRCNT_BK_DEF, QOS_TX_BLKS_HIGH_PRCNT_MIN,
                            QOS_TX_BLKS_HIGH_PRCNT_MAX,
                            sizeof p->TnetwDrv_InitParams.whalCtrl_init.TxBlocksHighPercentPerAc[QOS_AC_BK],
                            (PUCHAR)&p->TnetwDrv_InitParams.whalCtrl_init.TxBlocksHighPercentPerAc[QOS_AC_BK]);
    
    regReadIntegerParameter(pAdapter, &STRQOStxBlksHighPrcntVI,
                            QOS_TX_BLKS_HIGH_PRCNT_VI_DEF, QOS_TX_BLKS_HIGH_PRCNT_MIN,
                            QOS_TX_BLKS_HIGH_PRCNT_MAX,
                            sizeof p->TnetwDrv_InitParams.whalCtrl_init.TxBlocksHighPercentPerAc[QOS_AC_VI],
                            (PUCHAR)&p->TnetwDrv_InitParams.whalCtrl_init.TxBlocksHighPercentPerAc[QOS_AC_VI]);

    regReadIntegerParameter(pAdapter, &STRQOStxBlksHighPrcntVO,
                            QOS_TX_BLKS_HIGH_PRCNT_VO_DEF, QOS_TX_BLKS_HIGH_PRCNT_MIN,
                            QOS_TX_BLKS_HIGH_PRCNT_MAX,
                            sizeof p->TnetwDrv_InitParams.whalCtrl_init.TxBlocksHighPercentPerAc[QOS_AC_VO],
                            (PUCHAR)&p->TnetwDrv_InitParams.whalCtrl_init.TxBlocksHighPercentPerAc[QOS_AC_VO]);

    /* HW Tx queues buffers allocation low threshold */
    regReadIntegerParameter(pAdapter, &STRQOStxBlksLowPrcntBE,
                            QOS_TX_BLKS_LOW_PRCNT_BE_DEF, QOS_TX_BLKS_HIGH_PRCNT_MIN,
                            QOS_TX_BLKS_HIGH_PRCNT_MAX,
                            sizeof p->TnetwDrv_InitParams.whalCtrl_init.TxBlocksLowPercentPerAc[QOS_AC_BE],
                            (PUCHAR)&p->TnetwDrv_InitParams.whalCtrl_init.TxBlocksLowPercentPerAc[QOS_AC_BE]);

    regReadIntegerParameter(pAdapter, &STRQOStxBlksLowPrcntBK,
                            QOS_TX_BLKS_LOW_PRCNT_BK_DEF, QOS_TX_BLKS_HIGH_PRCNT_MIN,
                            QOS_TX_BLKS_HIGH_PRCNT_MAX,
                            sizeof p->TnetwDrv_InitParams.whalCtrl_init.TxBlocksLowPercentPerAc[QOS_AC_BK],
                            (PUCHAR)&p->TnetwDrv_InitParams.whalCtrl_init.TxBlocksLowPercentPerAc[QOS_AC_BK]);
    
    regReadIntegerParameter(pAdapter, &STRQOStxBlksLowPrcntVI,
                            QOS_TX_BLKS_LOW_PRCNT_VI_DEF, QOS_TX_BLKS_HIGH_PRCNT_MIN,
                            QOS_TX_BLKS_HIGH_PRCNT_MAX,
                            sizeof p->TnetwDrv_InitParams.whalCtrl_init.TxBlocksLowPercentPerAc[QOS_AC_VI],
                            (PUCHAR)&p->TnetwDrv_InitParams.whalCtrl_init.TxBlocksLowPercentPerAc[QOS_AC_VI]);

    regReadIntegerParameter(pAdapter, &STRQOStxBlksLowPrcntVO,
                            QOS_TX_BLKS_LOW_PRCNT_VO_DEF, QOS_TX_BLKS_HIGH_PRCNT_MIN,
                            QOS_TX_BLKS_HIGH_PRCNT_MAX,
                            sizeof p->TnetwDrv_InitParams.whalCtrl_init.TxBlocksLowPercentPerAc[QOS_AC_VO],
                            (PUCHAR)&p->TnetwDrv_InitParams.whalCtrl_init.TxBlocksLowPercentPerAc[QOS_AC_VO]);

    regReadIntegerParameter(pAdapter, &STRQOSShortRetryLimitBE,
                            QOS_SHORT_RETRY_LIMIT_BE_DEF, QOS_SHORT_RETRY_LIMIT_BE_MIN,
                            QOS_SHORT_RETRY_LIMIT_BE_MAX,
                            sizeof p->qosMngrInitParams.ShortRetryLimit[QOS_AC_BE],
                            (PUCHAR)&p->qosMngrInitParams.ShortRetryLimit[QOS_AC_BE]);

    regReadIntegerParameter(pAdapter, &STRQOSShortRetryLimitBK,
                            QOS_SHORT_RETRY_LIMIT_BK_DEF, QOS_SHORT_RETRY_LIMIT_BK_MIN,
                            QOS_SHORT_RETRY_LIMIT_BK_MAX,
                            sizeof p->qosMngrInitParams.ShortRetryLimit[QOS_AC_BK],
                            (PUCHAR)&p->qosMngrInitParams.ShortRetryLimit[QOS_AC_BK]);

    regReadIntegerParameter(pAdapter, &STRQOSShortRetryLimitVI,
                            QOS_SHORT_RETRY_LIMIT_VI_DEF, QOS_SHORT_RETRY_LIMIT_VI_MIN,
                            QOS_SHORT_RETRY_LIMIT_VI_MAX,
                            sizeof p->qosMngrInitParams.ShortRetryLimit[QOS_AC_VI],
                            (PUCHAR)&p->qosMngrInitParams.ShortRetryLimit[QOS_AC_VI]);

    regReadIntegerParameter(pAdapter, &STRQOSShortRetryLimitVO,
                            QOS_SHORT_RETRY_LIMIT_VO_DEF, QOS_SHORT_RETRY_LIMIT_VO_MIN,
                            QOS_SHORT_RETRY_LIMIT_VO_MAX,
                            sizeof p->qosMngrInitParams.ShortRetryLimit[QOS_AC_VO],
                            (PUCHAR)&p->qosMngrInitParams.ShortRetryLimit[QOS_AC_VO]);

    regReadIntegerParameter(pAdapter, &STRQOSLongRetryLimitBE,
                            QOS_LONG_RETRY_LIMIT_BE_DEF, QOS_LONG_RETRY_LIMIT_BE_MIN,
                            QOS_LONG_RETRY_LIMIT_BE_MAX,
                            sizeof p->qosMngrInitParams.LongRetryLimit[QOS_AC_BE],
                            (PUCHAR)&p->qosMngrInitParams.LongRetryLimit[QOS_AC_BE]);

    regReadIntegerParameter(pAdapter, &STRQOSLongRetryLimitBK,
                            QOS_LONG_RETRY_LIMIT_BK_DEF, QOS_LONG_RETRY_LIMIT_BK_MIN,
                            QOS_LONG_RETRY_LIMIT_BK_MAX,
                            sizeof p->qosMngrInitParams.LongRetryLimit[QOS_AC_BK],
                            (PUCHAR)&p->qosMngrInitParams.LongRetryLimit[QOS_AC_BK]);

    regReadIntegerParameter(pAdapter, &STRQOSLongRetryLimitVI,
                            QOS_LONG_RETRY_LIMIT_VI_DEF, QOS_LONG_RETRY_LIMIT_VI_MIN,
                            QOS_LONG_RETRY_LIMIT_VI_MAX,
                            sizeof p->qosMngrInitParams.LongRetryLimit[QOS_AC_VI],
                            (PUCHAR)&p->qosMngrInitParams.LongRetryLimit[QOS_AC_VI]);

    regReadIntegerParameter(pAdapter, &STRQOSLongRetryLimitVO,
                            QOS_LONG_RETRY_LIMIT_VO_DEF, QOS_LONG_RETRY_LIMIT_VO_MIN,
                            QOS_LONG_RETRY_LIMIT_VO_MAX,
                            sizeof p->qosMngrInitParams.LongRetryLimit[QOS_AC_VO],
                            (PUCHAR)&p->qosMngrInitParams.LongRetryLimit[QOS_AC_VO]);

    regReadIntegerParameter(pAdapter, &STRQOSAckPolicyBE,
                            QOS_ACK_POLICY_BE_DEF, QOS_ACK_POLICY_BE_MIN,
                            QOS_ACK_POLICY_BE_MAX,
                            sizeof p->qosMngrInitParams.acAckPolicy[QOS_AC_BE],
                            (PUCHAR)&p->qosMngrInitParams.acAckPolicy[QOS_AC_BE]);

    regReadIntegerParameter(pAdapter, &STRQOSAckPolicyBK,
                            QOS_ACK_POLICY_BK_DEF, QOS_ACK_POLICY_BK_MIN,
                            QOS_ACK_POLICY_BK_MAX,
                            sizeof p->qosMngrInitParams.acAckPolicy[QOS_AC_BK],
                            (PUCHAR)&p->qosMngrInitParams.acAckPolicy[QOS_AC_BK]);

    regReadIntegerParameter(pAdapter, &STRQOSAckPolicyVI,
                            QOS_ACK_POLICY_VI_DEF, QOS_ACK_POLICY_VI_MIN,
                            QOS_ACK_POLICY_VI_MAX,
                            sizeof p->qosMngrInitParams.acAckPolicy[QOS_AC_VI],
                            (PUCHAR)&p->qosMngrInitParams.acAckPolicy[QOS_AC_VI]);

    regReadIntegerParameter(pAdapter, &STRQOSAckPolicyVO,
                            QOS_ACK_POLICY_VO_DEF, QOS_ACK_POLICY_VO_MIN,
                            QOS_ACK_POLICY_VO_MAX,
                            sizeof p->qosMngrInitParams.acAckPolicy[QOS_AC_VO],
                            (PUCHAR)&p->qosMngrInitParams.acAckPolicy[QOS_AC_VO]);


    regReadIntegerParameter(pAdapter, &STRQoSqueue0OverFlowPolicy,
                    QOS_QUEUE_0_OVFLOW_POLICY_DEF, QOS_QUEUE_0_OVFLOW_POLICY_MIN,
                    QOS_QUEUE_0_OVFLOW_POLICY_MAX,
                    sizeof p->qosMngrInitParams.QueueOvFlowPolicy[0],
                    (PUCHAR)&p->qosMngrInitParams.QueueOvFlowPolicy[0]);
    
    regReadIntegerParameter(pAdapter, &STRQoSqueue1OverFlowPolicy,
                    QOS_QUEUE_1_OVFLOW_POLICY_DEF, QOS_QUEUE_1_OVFLOW_POLICY_MIN,
                    QOS_QUEUE_1_OVFLOW_POLICY_MAX,
                    sizeof p->qosMngrInitParams.QueueOvFlowPolicy[1],
                    (PUCHAR)&p->qosMngrInitParams.QueueOvFlowPolicy[1]);

    regReadIntegerParameter(pAdapter, &STRQoSqueue2OverFlowPolicy,
                    QOS_QUEUE_2_OVFLOW_POLICY_DEF, QOS_QUEUE_2_OVFLOW_POLICY_MIN,
                    QOS_QUEUE_2_OVFLOW_POLICY_MAX,
                    sizeof p->qosMngrInitParams.QueueOvFlowPolicy[2],
                    (PUCHAR)&p->qosMngrInitParams.QueueOvFlowPolicy[2]);

    regReadIntegerParameter(pAdapter, &STRQoSqueue3OverFlowPolicy,
                    QOS_QUEUE_3_OVFLOW_POLICY_DEF, QOS_QUEUE_3_OVFLOW_POLICY_MIN,
                    QOS_QUEUE_3_OVFLOW_POLICY_MAX,
                    sizeof p->qosMngrInitParams.QueueOvFlowPolicy[3],
                    (PUCHAR)&p->qosMngrInitParams.QueueOvFlowPolicy[3]);

    /* Packet Burst parameters    */

    regReadIntegerParameter(pAdapter, &STRQOSPacketBurstEnable,
                            QOS_PACKET_BURST_ENABLE_DEF, QOS_PACKET_BURST_ENABLE_MIN, 
                            QOS_PACKET_BURST_ENABLE_MAX,                         
                            sizeof p->qosMngrInitParams.PacketBurstEnable,
                            (PUCHAR)&p->qosMngrInitParams.PacketBurstEnable); 
    PRINTF(DBG_REGISTRY,( "STRQOSPacketBurstEnable = %d\n", p->qosMngrInitParams.PacketBurstEnable));
    regReadIntegerParameter(pAdapter, &STRQOSPacketBurstTxOpLimit,
                            QOS_PACKET_BURST_TXOP_LIMIT_DEF, QOS_PACKET_BURST_TXOP_LIMIT_MIN, 
                            QOS_PACKET_BURST_TXOP_LIMIT_MAX,                         
                            sizeof p->qosMngrInitParams.PacketBurstTxOpLimit,
                            (PUCHAR)&p->qosMngrInitParams.PacketBurstTxOpLimit); 



    /*---------------------------
        Measurement parameters
    -----------------------------*/

    regReadIntegerParameter(pAdapter, &STRMeasurTrafficThreshold,
                            MEASUREMENT_TRAFFIC_THRSHLD_DEF, MEASUREMENT_TRAFFIC_THRSHLD_MIN, MEASUREMENT_TRAFFIC_THRSHLD_MAX,
                            sizeof p->measurementInitParams.trafficIntensityThreshold,
                            (PUCHAR)&p->measurementInitParams.trafficIntensityThreshold);   

    regReadIntegerParameter(pAdapter, &STRMeasurMaxDurationOnNonServingChannel,
                            MEASUREMENT_MAX_DUR_NON_SRV_CHANNEL_DEF, MEASUREMENT_MAX_DUR_NON_SRV_CHANNEL_MIN, MEASUREMENT_MAX_DUR_NON_SRV_CHANNEL_MAX,
                            sizeof p->measurementInitParams.maxDurationOnNonServingChannel,
                            (PUCHAR)&p->measurementInitParams.maxDurationOnNonServingChannel);  


    /*---------------------------
          EXC Manager parameters
    -----------------------------*/
#ifdef EXC_MODULE_INCLUDED

    regReadIntegerParameter(pAdapter, &STRExcModeEnabled,
                            EXC_MNGR_ENABLE_DEF, EXC_MNGR_ENABLE_MIN, EXC_MNGR_ENABLE_MAX,
                            sizeof p->excMngrParams.excEnabled,
                            (PUCHAR)&p->excMngrParams.excEnabled);


    p->measurementInitParams.excEnabled = p->excMngrParams.excEnabled;  

#endif

    regReadIntegerParameter(pAdapter, &STRExcTestIgnoreDeAuth0,
                            EXC_TEST_IGNORE_DEAUTH_0_DEF, EXC_TEST_IGNORE_DEAUTH_0_MIN, EXC_TEST_IGNORE_DEAUTH_0_MAX,
                            sizeof p->apConnParams.ignoreDeauthReason0,
                            (PUCHAR)&p->apConnParams.ignoreDeauthReason0);
    
    /*---------------------------
      EEPROM less support
    -----------------------------*/
    regReadIntegerParameter(pAdapter, &STREEPROMlessModeSupported,
                            HAL_CTRL_EEPROMLESS_ENABLE_DEF, HAL_CTRL_EEPROMLESS_ENABLE_MIN,
                            HAL_CTRL_EEPROMLESS_ENABLE_MAX,
                            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlEepromLessEnable,
                            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlEepromLessEnable);

    pAdapter->EepromSupported = p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlEepromLessEnable;

    
    regReadStringParameter(pAdapter, &STRstationMacAddress,
                            (PCHAR)(defStaMacAddress), 11,
                            (PUCHAR)staMACAddress, &regMACstrLen);
    
    /*reads the arp ip from table*/
    regReadStringParameter(pAdapter ,&STRArp_Ip_Addr,
                            (PCHAR)(defArpIpAddress),REG_ARP_IP_ADDR_STR_LEN,
                            (PUCHAR)staArpIpAddress,&regArpIpStrLen ) ;

    regReadIntegerParameter(pAdapter, &STRArp_Ip_Filter_Ena,
                            DEF_FILTER_ENABLE_VALUE, MIN_FILTER_ENABLE_VALUE, MAX_FILTER_ENABLE_VALUE,
                            sizeof p->TnetwDrv_InitParams.arpIpFilterParams.isFilterEnabled,
                            (PUCHAR)&p->TnetwDrv_InitParams.arpIpFilterParams.isFilterEnabled);


    regConvertStringtoIpAddress(staArpIpAddress, (PUCHAR)&(p->TnetwDrv_InitParams.arpIpFilterParams.arpIpInitParams.addr[0]) ); 
    
    initRadioValusFromRgstryString( (PCHAR)(staMACAddress),
                                    (PCHAR)&(p->TnetwDrv_InitParams.whalCtrl_init.StaMacAddress[0]),
                                    6);
/*fource FragThreshold to be even value (round it down)MR WLAN00003501*/
    p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlFragThreshold &= 0xFFFE;





/*----------------------------------
    Health Monitor registry init
------------------------------------*/

    /* this time out is common to HealthCheck and TX power adjust */
    regReadIntegerParameter(pAdapter, &STRHealthMonitorCheckPeriod,
                            DEF_TX_POWER_ADJUST_TIME_OUT, 0, 65000,   /* in msec */
                            sizeof p->healthMonitorInitParams.healthCheckPeriod,
                            (PUCHAR)&p->healthMonitorInitParams.healthCheckPeriod);

    /* No scan complete recovery enabled */
    regReadIntegerParameter(pAdapter, &STRRecoveryEnabledNoScanComplete,
                            1, 0, 1,   /* default is enabled */
                            sizeof (p->healthMonitorInitParams.recoveryTriggerEnabled[ NO_SCAN_COMPLETE_FAILURE ]),
                            (PUCHAR)&(p->healthMonitorInitParams.recoveryTriggerEnabled[ NO_SCAN_COMPLETE_FAILURE ]) );
    
    /* Mailbox failure recovery enabled */
    regReadIntegerParameter(pAdapter, &STRRecoveryEnabledMboxFailure,
                            1, 0, 1,   /* default is enabled */
                            sizeof (p->healthMonitorInitParams.recoveryTriggerEnabled[ MBOX_FAILURE ]),
                            (PUCHAR)&(p->healthMonitorInitParams.recoveryTriggerEnabled[ MBOX_FAILURE ]) );

    /* HW awake failure recovery enabled */
    regReadIntegerParameter(pAdapter, &STRRecoveryEnabledHwAwakeFailure,
                            1, 0, 1,   /* default is enabled */
                            sizeof (p->healthMonitorInitParams.recoveryTriggerEnabled[ HW_AWAKE_FAILURE ]),
                            (PUCHAR)&(p->healthMonitorInitParams.recoveryTriggerEnabled[ HW_AWAKE_FAILURE ]) );
    
    /* Bus error recovery enabled */
    regReadIntegerParameter(pAdapter, &STRRecoveryEnabledBusError,
                            1, 0, 1,   /* default is enabled */
                            sizeof (p->healthMonitorInitParams.recoveryTriggerEnabled[ BUS_ERROR ]),
                            (PUCHAR)&(p->healthMonitorInitParams.recoveryTriggerEnabled[ BUS_ERROR ]) );
    
    /* Device error recovery enabled */
    regReadIntegerParameter(pAdapter, &STRRecoveryEnabledDeviceError,
                            1, 0, 1,   /* default is enabled */
                            sizeof (p->healthMonitorInitParams.recoveryTriggerEnabled[ DEVICE_ERROR ]),
                            (PUCHAR)&(p->healthMonitorInitParams.recoveryTriggerEnabled[ DEVICE_ERROR ]) );
    
    /* TX stuck recovery enabled */
    regReadIntegerParameter(pAdapter, &STRRecoveryEnabledTxStuck,
                            1, 0, 1,   /* default is enabled */
                            sizeof (p->healthMonitorInitParams.recoveryTriggerEnabled[ TX_STUCK ]),
                            (PUCHAR)&(p->healthMonitorInitParams.recoveryTriggerEnabled[ TX_STUCK ]) );
    
    /* disconnect timeout recovery enabled */
    regReadIntegerParameter(pAdapter, &STRRecoveryEnabledDisconnectTimeout,
                            0, 0, 1,   /* default is enabled */
                            sizeof (p->healthMonitorInitParams.recoveryTriggerEnabled[ DISCONNECT_TIMEOUT ]),
                            (PUCHAR)&(p->healthMonitorInitParams.recoveryTriggerEnabled[ DISCONNECT_TIMEOUT ]) );
    
    /* Power save failure recovery enabled */
    regReadIntegerParameter(pAdapter, &STRRecoveryEnabledPowerSaveFailure,
                            1, 0, 1,   /* default is enabled */
                            sizeof (p->healthMonitorInitParams.recoveryTriggerEnabled[ POWER_SAVE_FAILURE ]),
                            (PUCHAR)&(p->healthMonitorInitParams.recoveryTriggerEnabled[ POWER_SAVE_FAILURE ]) );
    
    /* Measurement failure recovery enabled */
    regReadIntegerParameter(pAdapter, &STRRecoveryEnabledMeasurementFailure,
                            1, 0, 1,   /* default is enabled */
                            sizeof (p->healthMonitorInitParams.recoveryTriggerEnabled[ MEASUREMENT_FAILURE ]),
                            (PUCHAR)&(p->healthMonitorInitParams.recoveryTriggerEnabled[ MEASUREMENT_FAILURE ]) );

/*-----------------------------------*/
/*   Hardware ACI recovery           */
/*-----------------------------------*/

    regReadIntegerParameter(pAdapter, &STRHardwareACIMode,
                            HAL_CTRL_ACI_MODE_DEF, HAL_CTRL_ACI_MODE_MIN,
                            HAL_CTRL_ACI_MODE_MAX,
                            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlACIMode,
                            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlACIMode);

    regReadIntegerParameter(pAdapter, &STRHardwareACIInputCCA,
                            HAL_CTRL_ACI_INPUT_CCA_DEF, HAL_CTRL_ACI_INPUT_CCA_MIN,
                            HAL_CTRL_ACI_INPUT_CCA_MAX,
                            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlInputCCA,
                            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlInputCCA);

    regReadIntegerParameter(pAdapter, &STRHardwareACIQualifiedCCA,
                            HAL_CTRL_ACI_QUALIFIED_CCA_DEF, HAL_CTRL_ACI_QUALIFIED_CCA_MIN,
                            HAL_CTRL_ACI_QUALIFIED_CCA_MAX,
                            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlQualifiedCCA,
                            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlQualifiedCCA);

    regReadIntegerParameter(pAdapter, &STRHardwareACIStompForRx,
                            HAL_CTRL_ACI_STOMP_FOR_RX_DEF, HAL_CTRL_ACI_STOMP_FOR_RX_MIN,
                            HAL_CTRL_ACI_STOMP_FOR_RX_MAX,
                            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlStompForRx,
                            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlStompForRx);

    regReadIntegerParameter(pAdapter, &STRHardwareACIStompForTx,
                            HAL_CTRL_ACI_STOMP_FOR_TX_DEF, HAL_CTRL_ACI_STOMP_FOR_TX_MIN,
                            HAL_CTRL_ACI_STOMP_FOR_TX_MAX,
                            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlStompForTx,
                            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlStompForTx);

    regReadIntegerParameter(pAdapter, &STRHardwareACITxCCA,
                            HAL_CTRL_ACI_TX_CCA_DEF, HAL_CTRL_ACI_TX_CCA_MIN,
                            HAL_CTRL_ACI_TX_CCA_MAX,
                            sizeof p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlTxCCA,
                            (PUCHAR)&p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlTxCCA);

/*----------------------------------
 TX power adjust
------------------------------------*/

    regReadIntegerParameter(pAdapter, &STRTxPowerCheckTime,
                            1, 1, 1200,  /* in units of 5000 ms */
                            sizeof p->siteMgrInitParams.TxPowerCheckTime,
                            (PUCHAR)&p->siteMgrInitParams.TxPowerCheckTime);


    regReadIntegerParameter(pAdapter, &STRTxPowerControlOn,
                            1, 0, 1,  /* on/off (1/0) default is on */
                            sizeof p->siteMgrInitParams.TxPowerControlOn,
                            (PUCHAR)&p->siteMgrInitParams.TxPowerControlOn);

    regReadIntegerParameter(pAdapter, &STRTxPowerRssiThresh,
                            38, 0, 200,  /* the value is positive and will be translated by driver */
                            sizeof p->siteMgrInitParams.TxPowerRssiThresh,
                            (PUCHAR)&p->siteMgrInitParams.TxPowerRssiThresh);

    regReadIntegerParameter(pAdapter, &STRTxPowerRssiRestoreThresh,
                            45, 0, 200,  /* the value is positive and will be translated by driver */
                            sizeof p->siteMgrInitParams.TxPowerRssiRestoreThresh,
                            (PUCHAR)&p->siteMgrInitParams.TxPowerRssiRestoreThresh);

    regReadIntegerParameter(pAdapter, &STRTxPowerTempRecover,
                            MIN_TX_POWER, MIN_TX_POWER, MAX_TX_POWER,  /* Set Min value for Atheros fix */
                            sizeof p->regulatoryDomainInitParams.uTemporaryTxPower,
                            (PUCHAR)&p->regulatoryDomainInitParams.uTemporaryTxPower);

/*----------------------------------
 end of TX power adjust
------------------------------------*/

/*----------------------------------
 Scan Concentrator
------------------------------------*/
    regReadIntegerParameter( pAdapter, &STRPassiveScanDwellTime,
                             SCAN_CNCN_DRIVER_DEFAULT_DWELL_TIME_DEF, SCAN_CNCN_DRIVER_DEFAULT_DWELL_TIME_MIN, SCAN_CNCN_DRIVER_DEFAULT_DWELL_TIME_MAX,
                             sizeof p->scanConcentratorInitParams.passiveScanDwellTime,
                             (PUCHAR)&p->scanConcentratorInitParams.passiveScanDwellTime );
//TRS: Scan changes from TI
	regReadIntegerParameter( pAdapter, &STRMinimumDurationBetweenOidScans,
							 SCAN_CNCN_MIN_DURATION_FOR_OID_SCANS_DEF, SCAN_CNCN_MIN_DURATION_FOR_OID_SCANS_MIN, SCAN_CNCN_MIN_DURATION_FOR_OID_SCANS_MAX,
							 sizeof p->scanConcentratorInitParams.minimumDurationBetweenOidScans,
							 (PUCHAR)&p->scanConcentratorInitParams.minimumDurationBetweenOidScans );
//TRS: end ofScan changes from TI
   /*
    *   set etherMaxPayloadSize parameter for MTU size setting
    */
    if(p->ctrlDataInitParams.ctrlDataFourXEnable == TRUE)
    {
        if(p->siteMgrInitParams.siteMgrDesiredBSSType == BSS_ANY)
        {
            /*disable 4x if we are in any mode*/
            p->ctrlDataInitParams.ctrlDataFourXEnable = FALSE;
            pAdapter->etherMaxPayloadSize = NOT_FOUR_X_MODE_PAYLOAD_SIZE;
        }
        else
        if(p->siteMgrInitParams.siteMgrDesiredBSSType == BSS_INDEPENDENT)
        {
            pAdapter->etherMaxPayloadSize = IBSS_FOUR_X_MODE_PAYLOAD_SIZE;
            p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlRtsThreshold = HAL_CTRL_RTS_THRESHOLD_MAX;
            p->TnetwDrv_InitParams.halCtrlConfigParams.halCtrlFragThreshold = HAL_CTRL_FRAG_THRESHOLD_MAX;
            p->ctrlDataInitParams.ctrlDataFourXEnable = FALSE;
        }
        else
        if(p->siteMgrInitParams.siteMgrDesiredBSSType ==  BSS_INFRASTRUCTURE)
        {
            pAdapter->etherMaxPayloadSize = NOT_FOUR_X_MODE_PAYLOAD_SIZE;
        }
    }
    else
    {
        pAdapter->etherMaxPayloadSize = NOT_FOUR_X_MODE_PAYLOAD_SIZE;
    }
}


/*-----------------------------------------------------------------------------

Routine Name:

    regReadParameters

Routine Description:


Arguments:


Return Value:

    None

-----------------------------------------------------------------------------*/
VOID
regReadIntegerParameter(
                 PTIWLN_ADAPTER_T       pAdapter,
                 PNDIS_STRING           pParameterName,
                 ULONG                  defaultValue,
                 ULONG                  minValue,
                 ULONG                  maxValue,
                 UCHAR                  parameterSize,
                 PUCHAR                 pParameter
                 )
{
    PNDIS_CONFIGURATION_PARAMETER   RetValue;
    NDIS_STATUS                     Status;
    ULONG                           value;

    NdisReadConfiguration(&Status, &RetValue,
                          pAdapter->ConfigHandle, pParameterName,
                          NdisParameterInteger);

    if(Status != NDIS_STATUS_SUCCESS) {

        NdisReadConfiguration(&Status, &RetValue,
              pAdapter->ConfigHandle, pParameterName,
              NdisParameterString
              );

        if(Status == NDIS_STATUS_SUCCESS) {
            assignRegValue(&value, RetValue);
            RetValue->ParameterData.IntegerData = value;

        }

    }


    if (Status != NDIS_STATUS_SUCCESS ||
        RetValue->ParameterData.IntegerData < minValue ||
        RetValue->ParameterData.IntegerData > maxValue)
    {
        PRINTF(DBG_REGISTRY,( "NdisReadConfiguration fail\n"));
        value = defaultValue;

    } else
    {
        value = RetValue->ParameterData.IntegerData;
    }

    switch (parameterSize)
    {
    case 1:
        *((PUCHAR) pParameter) = (UCHAR) value;
        break;

    case 2:
        *((PUSHORT) pParameter) = (USHORT) value;
        break;

    case 4:
        *((PULONG) pParameter) = (ULONG) value;
        break;

    default:
        PRINT(DBG_REGISTRY_ERROR, "TIWL: Illegal Registry parameter size\n");
        break;

    }

}

/*-----------------------------------------------------------------------------

Routine Name:

    regReadParameters

Routine Description:


Arguments:


Return Value:

    None

-----------------------------------------------------------------------------*/
VOID
regReadStringParameter(
                 PTIWLN_ADAPTER_T       pAdapter,
                 PNDIS_STRING           pParameterName,
                 PCHAR                  pDefaultValue,
                 USHORT                 defaultLen,
                 PUCHAR                 pParameter,
                 void*                  pParameterSize
                 )
{
    PNDIS_CONFIGURATION_PARAMETER   RetValue;
    NDIS_STATUS                     Status;
    ANSI_STRING                     ansiString;
    PUCHAR                          pSizeChar = 0;
    PUSHORT                         pSizeShort = 0;

    if(defaultLen <= 256)
    {
        pSizeChar = (PUCHAR)pParameterSize;
        ansiString.MaximumLength = 256;
    }
    else
    {
        pSizeShort = (PUSHORT)pParameterSize;
        ansiString.MaximumLength = 32576;
    }

    NdisReadConfiguration(&Status, &RetValue,
                          pAdapter->ConfigHandle, pParameterName,
                          NdisParameterString);

    if (Status == NDIS_STATUS_SUCCESS)
    {
        ansiString.Buffer = (PCHAR)pParameter;

        NdisUnicodeStringToAnsiString(&ansiString, &RetValue->ParameterData.StringData);
        if(defaultLen <= 256)
            *pSizeChar = (UCHAR)ansiString.Length;
        else
            *pSizeShort = (USHORT)ansiString.Length;
    } else
    {
        if(defaultLen <= 256)
            *pSizeChar = (UCHAR)defaultLen;
        else
            *pSizeShort = (USHORT)defaultLen;

        memcpy(pParameter, pDefaultValue, defaultLen);
    }

    PRINTF(DBG_REGISTRY_LOUD, ("Read String Registry:  %c%c%c%c%c%c%c%c%c%c%c%c = %s\n",
                          pParameterName->Buffer[0],
                          pParameterName->Buffer[1],
                          pParameterName->Buffer[2],
                          pParameterName->Buffer[3],
                          pParameterName->Buffer[4],
                          pParameterName->Buffer[5],
                          pParameterName->Buffer[6],
                          pParameterName->Buffer[7],
                          pParameterName->Buffer[8],
                          pParameterName->Buffer[9],
                          pParameterName->Buffer[10],
                          pParameterName->Buffer[11],
                          pParameter));

}


/*-----------------------------------------------------------------------------

Routine Name:

    regReadParameters

Routine Description:


Arguments:


Return Value:

    None

-----------------------------------------------------------------------------*/
VOID
regReadUnicodeStringParameter(
                 PTIWLN_ADAPTER_T       pAdapter,
                 PNDIS_STRING           pParameterName,
                 PCHAR                  pDefaultValue,
                 UCHAR                  defaultLen,
                 PUCHAR                 pParameter,
                 PUCHAR                 pParameterSize
                 )
{
    PNDIS_CONFIGURATION_PARAMETER   RetValue;
    NDIS_STATUS                     Status;

    NdisReadConfiguration(&Status, &RetValue,
                          pAdapter->ConfigHandle, pParameterName,
                          NdisParameterString);

    if (Status == NDIS_STATUS_SUCCESS)
    {
        *pParameterSize = (UCHAR)RetValue->ParameterData.StringData.Length;
        memcpy(pParameter, (PUCHAR)RetValue->ParameterData.StringData.Buffer, *pParameterSize);
    } else
    {
        *pParameterSize = defaultLen;
        memcpy(pParameter, pDefaultValue, defaultLen);
    }

}

/*-----------------------------------------------------------------------------

Routine Name:

    regReadParameters

Routine Description:


Arguments:


Return Value:

    None

-----------------------------------------------------------------------------*/
VOID
regReadWepKeyParameter(
                 PTIWLN_ADAPTER_T       pAdapter,
                 PUCHAR                 pKeysStructure,
                 UINT8                  defaultKeyId
                 )
{
    NDIS_STATUS                         status;
    securityKeys_t                      *pSecKeys;
    int i;
    int len;
    UCHAR                               Buf[MAX_KEY_BUFFER_LEN];
    PNDIS_CONFIGURATION_PARAMETER       RetValue;
    ANSI_STRING                         ansiString;
    NDIS_STRING                         STRdot11DefaultWEPKey[4] =
                                                {   NDIS_STRING_CONST( "dot11WEPDefaultKey1" ),
                                                    NDIS_STRING_CONST( "dot11WEPDefaultKey2" ),
                                                    NDIS_STRING_CONST( "dot11WEPDefaultKey3" ),
                                                    NDIS_STRING_CONST( "dot11WEPDefaultKey4" )
                                                };



    PRINTF(DBG_REGISTRY_LOUD, ("Reading WEP keys\n"));

    pSecKeys = (securityKeys_t*)pKeysStructure;

    /**/
    /* Read WEP from registry*/
    /**/
    for ( i = 0; i < DOT11_MAX_DEFAULT_WEP_KEYS; i++ )
    {
        NdisReadConfiguration(&status, &RetValue,
                              pAdapter->ConfigHandle, &STRdot11DefaultWEPKey[i],
                              NdisParameterString);

        if(status == NDIS_STATUS_SUCCESS)
        {
            ansiString.Buffer = (PCHAR)Buf;
            ansiString.MaximumLength = MAX_KEY_BUFFER_LEN;

            pSecKeys->keyIndex = i;
            pSecKeys->keyType = WEP_KEY;
            NdisZeroMemory((void *)pSecKeys->macAddress.addr, 6);

            if(((char *)(RetValue->ParameterData.StringData.Buffer))[1] == 0)
            {
                NdisUnicodeStringToAnsiString(&ansiString, &RetValue->ParameterData.StringData);

                len = decryptWEP((PCHAR)Buf, (PCHAR)pSecKeys->encKey, ansiString.Length);
            } else {
                len = decryptWEP((PCHAR)RetValue->ParameterData.StringData.Buffer,
                                 (PCHAR)pSecKeys->encKey,
                                 RetValue->ParameterData.StringData.Length);
            }

            if(len < ACX_64BITS_WEP_KEY_LENGTH_BYTES)
            {
                PRINTF(DBG_REGISTRY_ERROR, ("Error: minimum WEP key size is 5 bytes(%d)\n", len));
                pSecKeys->keyType = NULL_KEY;
                len = 0;
            }
            else if(len < ACX_128BITS_WEP_KEY_LENGTH_BYTES)
            {
                len = ACX_64BITS_WEP_KEY_LENGTH_BYTES;
            }
            else if(len < ACX_256BITS_WEP_KEY_LENGTH_BYTES)
            {
                len = ACX_128BITS_WEP_KEY_LENGTH_BYTES;
            }
            else
                len = ACX_256BITS_WEP_KEY_LENGTH_BYTES;

            pSecKeys->encLen = (UINT8)len;

        } else
        {
            pSecKeys->keyType = NULL_KEY;
            pSecKeys->encLen = 0;
    }
        /*create local keys cache*/
        pAdapter->DefaultWepKeys[i].KeyIndex = i;
        if(i==defaultKeyId)
            pAdapter->DefaultWepKeys[i].KeyIndex |= 0x80000000;
        pAdapter->DefaultWepKeys[i].KeyLength = pSecKeys->encLen;
        NdisMoveMemory((void *)pAdapter->DefaultWepKeys[i].KeyMaterial,
            (void *)pSecKeys->encKey, pSecKeys->encLen);
        pAdapter->DefaultWepKeys[i].Length = sizeof(OS_802_11_WEP);
        pSecKeys++;
    }
}

#define iswhite(c) ( (c==' ') || (c=='\t') || (c=='\n') )

/*
 *
 *       Fun:   isnumber
 *
 *       Desc:  check if the ascii character is a number in the given base
 *
 *       Ret:   1 if number is a digit, 0 if not.
 *
 *       Notes: none
 *
 *       File:  btoi.c
 *
 */
BOOLEAN
isnumber ( short *pi, char c, short base )
{

    /* return 1 if c is a digit in the give base, else return 0 */
    /* place value of digit at pi */
    if ( base == 16 )
    {
        if ( '0' <= c && c <= '9' )
        {
            *pi =  c - '0';
            return (1);
        }
        else if ( 'a' <= c && c <= 'f' )
        {
            *pi =  c - 'a' + 10 ;
            return (1);
        }
        else if ( 'A' <= c && c <= 'F' )
        {
            *pi =  c - 'A' + 10 ;
            return (1);
        }
        else
        {
            return (0);
        }
    }
    c -= '0';
    if ( 0 <= (signed char)c && c < base )
    {
        *pi =  c ;
        return (1);
    }
    else
    {
        return (0);
    }
} /* end of isnumber */


short
_btoi ( char *sptr, short slen, int *pi, short base )
{
    char    *s, c ;
    short   d, sign ;
    int     result ;
    char    saved ;

    s           =  sptr ;
    result      =  0 ;
    saved       =  sptr [slen];
    sptr [slen] =  '\0';

    /* skip initial white space */
/*    while ( (c = *s++) && iswhite(c) ); */
    do
    {
       c = *s++;
       if (!(c  && iswhite(c))) 
         break;
    }while(1); 
  
    --s ;

    /* recognize optional sign */
    if ( *s == '-' )
    {
        sign =  - 1 ;
        s++ ;
    }
    else if ( *s == '+' )
    {
        sign =  1 ;
        s++ ;
    }
    else
    {
        sign =  1 ;
    }

    /* recognize optional hex# prefix */
    if ((base == 16) && ((*s == '0') && ((*(s + 1) == 'x') || (*(s + 1) == 'X'))
       ))
        s += 2 ;

    /* recognize digits */

/*    for (; (c = *s++) && isnumber(&d, c, base) ; )
    {
        result =  base * result + d ;
    }
*/    
    while(1)
    {
      c = *s++;
      if (!(c && isnumber(&d, c, base)))	
        break;
      result =  base * result + d ;
    };

    *pi         =  sign * result ;
    sptr [slen] =  saved ; /* restore character which we changed to null */
    return (s - sptr - 1);
} /* end of _btoi */

static int decryptWEP
(
  PCHAR pSrc,
  PCHAR pDst,
  ULONG len
)
{
  /**/
  /* key to use for encryption*/
  /**/
  static LPCSTR lpEncryptKey = "jkljz98c&2>a+t)cl5[d=n3;\"f_um6\\d~v%$HO1";
  int cnEncryptLen = strlen(lpEncryptKey);

  char cIn, cCrypt, cHex[3];
  int i, j, nLen;
  int nPos;

  nLen = len / 2;
  nPos = len;

  /* start reading from end*/
  nPos = len - 2;

  for(i = 0; (i < nLen) && (nPos >= 0); i++, nPos -= 2)
  {
    /* get hex character*/
    cHex[0] = pSrc[nPos];
    cHex[1] = pSrc[nPos + 1];
    cHex[2] = 0;

    _btoi ( cHex, 2, &j, 16);
    cIn = (char) j;

    cCrypt = lpEncryptKey[i % cnEncryptLen];
    cIn = cIn ^ cCrypt;

    pDst[i] = cIn;
  }

  PRINTF(DBG_REGISTRY_LOUD, ("First 5 bytes of WEP: %x-%x-%x-%x-%x\n",
    pDst[0],
    pDst[1],
    pDst[2],
    pDst[3],
    pDst[4]));

  return nLen;
}

static VOID initRadioValusFromRgstryString
(
  PCHAR pSrc,
  PCHAR pDst,
  ULONG len
)
{
    int j;
    ULONG count;
    for (count = 0 ; count < len ; count++)
    {
        _btoi((char *) (pSrc+(count*3)),  2, &j, 16 );

        pDst[count] = (UINT8) j;
    }
}
/*-----------------------------------------------------------------------------

Routine Name:

    regReadParameters

Routine Description:


Arguments:


Return Value:

    None

-----------------------------------------------------------------------------*/
VOID
regReadParameters(
    PTIWLN_ADAPTER_T        pAdapter
                 )
{
    NDIS_STRING SlotNumberStr = NDIS_STRING_CONST("SlotNumber");
    PNDIS_CONFIGURATION_PARAMETER RetValue;
    NDIS_STATUS Status;

    NdisReadConfiguration(&Status, &RetValue,
                          pAdapter->ConfigHandle, &SlotNumberStr,
                          NdisParameterInteger);

    if (Status != NDIS_STATUS_SUCCESS)
        pAdapter->SlotNumber = 0;
    else
        pAdapter->SlotNumber = RetValue->ParameterData.IntegerData;


    PRINTF(DBG_REGISTRY_VERY_LOUD, ("TIWL: SlotNumber-%ld\n", pAdapter->SlotNumber));
}


/*-----------------------------------------------------------------------------

Routine Name:

    regWriteInstanceNumber

Routine Description:


Arguments:


Return Value:

    None

-----------------------------------------------------------------------------*/
VOID
regWriteInstanceNumber(
                      PTIWLN_ADAPTER_T pAdapter
                      )
{
#ifdef _WINDOWS
#endif /* _WINDOWS */
}


#ifdef TI_DBG


VOID
regReadLastDbgState(
                   PTIWLN_ADAPTER_T pAdapter
                   )
{
    NDIS_STRING OsDbgStr = NDIS_STRING_CONST("OsDbgState");
    PNDIS_CONFIGURATION_PARAMETER Value;
    NDIS_STATUS Status;

    NdisReadConfiguration(&Status, &Value,
                          pAdapter->ConfigHandle, &OsDbgStr,
                          NdisParameterInteger
                         );

    if (Status != NDIS_STATUS_SUCCESS)
    {

        TiDebugFlag = ((DBG_NDIS_OIDS | DBG_INIT | DBG_RECV | DBG_SEND | DBG_IOCTL | DBG_INTERRUPT) << 16) |
                DBG_SEV_VERY_LOUD | DBG_SEV_INFO | DBG_SEV_LOUD | DBG_SEV_ERROR | DBG_SEV_FATAL_ERROR;

    } else
    {

        PRINTF(DBG_REGISTRY_VERY_LOUD, ("TIWL: New Flag - 0x%lX\n", Value->ParameterData.IntegerData));

        TiDebugFlag = Value->ParameterData.IntegerData;

    }
}


VOID
regWriteLastDbgState(
                    PTIWLN_ADAPTER_T pAdapter
                    )
{
    NDIS_STRING OsDbgStr = NDIS_STRING_CONST("OsDbgState");
    NDIS_CONFIGURATION_PARAMETER Value;
    NDIS_STATUS Status;

    Value.ParameterType = NdisParameterInteger;
    Value.ParameterData.IntegerData = TiDebugFlag;

    NdisWriteConfiguration(&Status, pAdapter->ConfigHandle,
                           &OsDbgStr, &Value);
}
#endif



static void readRates(PTIWLN_ADAPTER_T pAdapter, initTable_t *pInitTable)
{
    /*
    ** B band
    */
    regReadIntegerParameter(pAdapter, &STRdot11BasicRateMask_B,
                            BASIC_RATE_SET_1_2_5_5_11, BASIC_RATE_SET_1_2, BASIC_RATE_SET_1_2_5_5_11,
                            sizeof pInitTable->siteMgrInitParams.siteMgrRegstryBasicRate[DOT11_B_MODE], 
                            (PUCHAR)&pInitTable->siteMgrInitParams.siteMgrRegstryBasicRate[DOT11_B_MODE]);

    regReadIntegerParameter(pAdapter, &STRdot11SupportedRateMask_B,
                          SUPPORTED_RATE_SET_1_2_5_5_11_22, SUPPORTED_RATE_SET_1_2, SUPPORTED_RATE_SET_1_2_5_5_11_22,
                            sizeof pInitTable->siteMgrInitParams.siteMgrRegstrySuppRate[DOT11_B_MODE], 
                            (PUCHAR)&pInitTable->siteMgrInitParams.siteMgrRegstrySuppRate[DOT11_B_MODE]);
    /*
    ** G band (B&G rates)
    */
    regReadIntegerParameter(pAdapter, &STRdot11BasicRateMask_G,
                            BASIC_RATE_SET_1_2_5_5_11, BASIC_RATE_SET_1_2, BASIC_RATE_SET_1_2_5_5_11,
                            sizeof pInitTable->siteMgrInitParams.siteMgrRegstryBasicRate[DOT11_G_MODE], 
                            (PUCHAR)&pInitTable->siteMgrInitParams.siteMgrRegstryBasicRate[DOT11_G_MODE]);

    regReadIntegerParameter(pAdapter, &STRdot11SupportedRateMask_G,
                            SUPPORTED_RATE_SET_ALL, SUPPORTED_RATE_SET_1_2, SUPPORTED_RATE_SET_ALL,
                            sizeof pInitTable->siteMgrInitParams.siteMgrRegstrySuppRate[DOT11_G_MODE], 
                            (PUCHAR)&pInitTable->siteMgrInitParams.siteMgrRegstrySuppRate[DOT11_G_MODE]);

    /*
    ** A band
    */
    regReadIntegerParameter(pAdapter, &STRdot11BasicRateMask_A,
                            BASIC_RATE_SET_6_12_24, BASIC_RATE_SET_6_12_24, BASIC_RATE_SET_6_12_24,
                            sizeof pInitTable->siteMgrInitParams.siteMgrRegstryBasicRate[DOT11_A_MODE], 
                            (PUCHAR)&pInitTable->siteMgrInitParams.siteMgrRegstryBasicRate[DOT11_A_MODE]);

    regReadIntegerParameter(pAdapter, &STRdot11SupportedRateMask_A,
                            SUPPORTED_RATE_SET_UP_TO_54, SUPPORTED_RATE_SET_1_2, SUPPORTED_RATE_SET_UP_TO_54,
                            sizeof pInitTable->siteMgrInitParams.siteMgrRegstrySuppRate[DOT11_A_MODE], 
                            (PUCHAR)&pInitTable->siteMgrInitParams.siteMgrRegstrySuppRate[DOT11_A_MODE]);

    /*
    ** Dual band (A&G)
    */
    regReadIntegerParameter(pAdapter, &STRdot11BasicRateMask_AG,
                            BASIC_RATE_SET_1_2, BASIC_RATE_SET_1_2, BASIC_RATE_SET_1_2,
                            sizeof pInitTable->siteMgrInitParams.siteMgrRegstryBasicRate[DOT11_DUAL_MODE],
                            (PUCHAR)&pInitTable->siteMgrInitParams.siteMgrRegstryBasicRate[DOT11_DUAL_MODE]);

    regReadIntegerParameter(pAdapter, &STRdot11SupportedRateMask_AG,
                            SUPPORTED_RATE_SET_ALL_OFDM, SUPPORTED_RATE_SET_1_2, SUPPORTED_RATE_SET_ALL_OFDM,
                            sizeof pInitTable->siteMgrInitParams.siteMgrRegstrySuppRate[DOT11_DUAL_MODE],
                            (PUCHAR)&pInitTable->siteMgrInitParams.siteMgrRegstrySuppRate[DOT11_DUAL_MODE]);

    /* Tx Rate */
    regReadIntegerParameter(pAdapter, &STRdot11DesiredTxRate,
                            REG_RATE_AUTO_BIT, REG_RATE_AUTO_BIT, REG_RATE_54M_OFDM_BIT,
                            sizeof pInitTable->siteMgrInitParams.siteMgrRegstryDesiredTxRate,
                            (PUCHAR)&pInitTable->siteMgrInitParams.siteMgrRegstryDesiredTxRate);

    /* Management & Ctrl Tx rate Selection (Fix rate/MinBasic/Max basic) */
    regReadIntegerParameter(pAdapter, &STRdot11MgmtCtrlTxRateSelection,
                            MAX_BASIC_TX_RATE, MIN_BASIC_TX_RATE, SPECIFIC_TX_RATE,
                            sizeof pInitTable->siteMgrInitParams.siteMgrRegstryDesiredMgmtCtrlTxRateOption,
                            (PUCHAR)&pInitTable->siteMgrInitParams.siteMgrRegstryDesiredMgmtCtrlTxRateOption);
    
    /* Management & Ctrl Tx rate (HW generated packets) */
    regReadIntegerParameter(pAdapter, &STRdot11MgmtCtrlTxRate,
                            REG_RATE_2M_BIT, REG_RATE_1M_BIT, REG_RATE_54M_OFDM_BIT,
                            sizeof pInitTable->siteMgrInitParams.siteMgrRegstryDesiredMgmtCtrlTxRate,
                            (PUCHAR)&pInitTable->siteMgrInitParams.siteMgrRegstryDesiredMgmtCtrlTxRate);

}


static void decryptScanControlTable(PUCHAR src, PUCHAR dst, USHORT len)
{

    USHORT i;
    int parityFlag = 0;
    char tmp = 0;
    char finalChar = 0;

    for(i=0; i < len; i++)
    {
        switch(src[i])
        {
        case 'A':
        case 'a':
            tmp = 10;
            break;
        case 'B':
        case 'b':
            tmp = 11;
            break;
        case 'C':
        case 'c':
            tmp = 12;
            break;
        case 'D':
        case 'd':
            tmp = 13;
            break;
        case 'E':
        case 'e':
            tmp = 14;
            break;
        case 'F':
        case 'f':
            tmp = 15;
            break;
        default:
            if( (src[i] >='0') && (src[i] <= '9') )
                tmp = (src[i] - '0');
            else
                return; /* ERROR input char */
        }
        if(parityFlag == 0)
            finalChar =  tmp << 4;
        else
        {
            finalChar |= (tmp & 0x0f);
            dst[i/2] = finalChar;
        }
        parityFlag = 1-parityFlag;
    }
}

VOID regReadNetworkAddress(PTIWLN_ADAPTER_T pAdapter)
{
  NDIS_STATUS Status;
  PVOID pvNetworkAddress;
  UINT cbNetworkAddress;

  NdisReadNetworkAddress( &Status, &pvNetworkAddress, &cbNetworkAddress, pAdapter->ConfigHandle );
  if ( ( NDIS_STATUS_SUCCESS == Status ) && ( ETH_ADDR_SIZE == cbNetworkAddress ) ) {
    NdisMoveMemory( pAdapter->CurrentAddr, pvNetworkAddress, ETH_ADDR_SIZE);
        pAdapter->bCurrentAddrFromRegistry = TRUE;
  }
}

/*-----------------------------------------------------------------------------

Routine Name:

    regReadIntegerTable

Routine Description:
    reads any table format and insert it to another string.
    the delimiters of the tables can be:
     - space (" ")
     - comma (",")
    the table reads only integers thus its reads the following chars:
     - "0" till "9"
     - minus sign ("-")

Arguments:


Return Value:

    zero on success else - error number.

-----------------------------------------------------------------------------*/
UINT32
regReadIntegerTable(
                     PTIWLN_ADAPTER_T       pAdapter,
                     PNDIS_STRING           pParameterName,
                     PCHAR                  pDefaultValue,
                     USHORT                 defaultLen,
                     PUCHAR                 pParameter
                     )
{
    UINT32 parameterIndex = 0;
    int myNumber;

    UINT32 index;
    UINT32 bufferSize = 0;

    char tempBuffer[15];
    char *pTempBuffer = tempBuffer;
    UINT32 tempBufferIndex = 0;

    BOOL isDigit;
    BOOL numberReady;
    BOOL isSign;
    BOOL    endOfLine;

    UINT32 debugInfo = 0;

    CHAR Buffer[MAX_KEY_BUFFER_LEN];
    PCHAR pBuffer = (PCHAR)&Buffer;

    regReadStringParameter(pAdapter,
                           pParameterName,
                           pDefaultValue,
                           defaultLen,
                           (PUCHAR)pBuffer,
                           &bufferSize);



    index=0;
    do { /* Parsing one line */

        isSign = FALSE;
        isDigit = FALSE;
        numberReady = FALSE;
        tempBufferIndex = 0;
        endOfLine = FALSE;

        while ((numberReady==FALSE) && (index<bufferSize))
        {
            /* Parsing one number */
            switch (pBuffer[index])
            {
                case '0':
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                case '8':
                case '9':
                    pTempBuffer[tempBufferIndex] = pBuffer[index];
                    ++tempBufferIndex;
                    isDigit = TRUE;
                    break;

                case '-':
                    pTempBuffer[tempBufferIndex] = pBuffer[index];
                    ++tempBufferIndex;
                    if (isDigit==TRUE)
                    {
                        PRINTF(DBG_REGISTRY_INFO, ("Error in read parameter %c in line index %d\n\
                                               The sign '-' isn't in place!\n",pBuffer[index],index));
                        debugInfo = 1;
                    }
                    isSign = TRUE;
                    break;

                case ' ':
                case '\t': /* tab char */
                    /* for space discard*/
                    if ((isDigit==FALSE) && (isSign==FALSE))
                    {
                        break;
                    }
                    /*
                    else we are continue to the code of the case ','
                    */
                case '\0':
                    endOfLine = TRUE;

                case ',':
                    /* end of number reading */
                    pTempBuffer[tempBufferIndex] = '\0';
                    if (isDigit == FALSE)
                    {
                        PRINTF(DBG_REGISTRY_INFO, ("Error in end of number delimiter. number isn't ready.\
                            check index %d",index));
                        debugInfo = 2;
                    }
                    numberReady = TRUE;
                    break;

                default:
                    PRINTF(DBG_REGISTRY_INFO, ("%s(%d) Error - unexpected parameter %c.\n",
                        __FILE__,__LINE__,pBuffer[index]));
                    debugInfo = 3;
                    break;
            }/* switch( pBuffer[index] ) */

            if (debugInfo != 0)
            {
                return debugInfo;
            }
            ++index;

        }/* while (numberReady==FALSE)*/

        if (pTempBuffer[0] == '-')
        {
            ++pTempBuffer;
            myNumber = tiwlnstrtoi(pTempBuffer,tempBufferIndex-1);
            myNumber = -(myNumber);
        }
        else
        {
            myNumber = tiwlnstrtoi(pTempBuffer,tempBufferIndex);
        }


        pParameter[parameterIndex] = myNumber;
        ++parameterIndex;
    
    }while ((index<bufferSize)&&(endOfLine==FALSE));

    return debugInfo;
}

void assignRegValue(PULONG lValue, PNDIS_CONFIGURATION_PARAMETER ndisParameter)
{ 
    char b[8]; 
    ANSI_STRING a = {0, 0, 0};

    a.MaximumLength = sizeof(b);
    a.Buffer=(PCHAR)b;

    if(ndisParameter->ParameterData.StringData.Length <= sizeof (b) * 2) 
    { 
      if ( ((char *)(ndisParameter->ParameterData.StringData.Buffer))[1] == 0 )
      {
        NdisUnicodeStringToAnsiString ( &a, &(ndisParameter)->ParameterData.StringData );
        *lValue = tiwlnstrtoi ( (char *)a.Buffer, a.Length ); 
      } else { 
        *lValue = tiwlnstrtoi ( (char *)(ndisParameter->ParameterData.StringData.Buffer), ndisParameter->ParameterData.StringData.Length); 
      } 
    } else {
      *lValue = 0;
    }
  }

/*-----------------------------------------------------------------------------

Routine Name:

    regConvertStringtoIpAddress

Routine Description: Converts the Ip Adrress in a form of string readen from the Registry 
to the Ip Address Array to be stored in the init_table struct 


Arguments:


Return Value:

    None

-----------------------------------------------------------------------------*/
void regConvertStringtoIpAddress(UINT8 *staIpAddressString,UINT8 *IpAddressArray)
{

char *ptr;
UINT8 *tmpIpAddr;
UINT8 value=0,value_l,value_h,add_value;
int i;


    /* Take the pointer to the string MAC Address to convert it to the Array MAC Address */
    ptr=(char *)staIpAddressString;
    tmpIpAddr = IpAddressArray;

#if 0 
    for(i=0 ; i<4 ; ptr++)
    {
        value_l = (*ptr-'0');

        /* PRINTF(DBG_REGISTRY,("value_l [%d] *ptr %c value %d\n",value_l,*ptr,value));*/

        if( value_l  < 9)
        {
            value = value*10 + value_l;             
            /* PRINTF(DBG_REGISTRY,("value %d value_l %d  \n",value,value_l));*/


        }
        else
        {
            tmpIpAddr[i] = value;
            /* PRINTF(DBG_REGISTRY,("tmpMacAddr[%d]  is %d\n",i,tmpMacAddr[i]));*/
            value = 0;
            i++;
        }

    }

#else

    for(i=0 ; i<4 ; ptr++)
    {
        
        /* The value can be or "0-9" or from "a-f" */
        value_l = (*ptr-'0');
        value_h = (*ptr - 'a');

        /*PRINTF(DBG_REGISTRY,("value_l [%d] value_h [%d] *ptr %c value %d\n",value_l,value_h,*ptr,value));*/

        if( (value_l <= 9) || (value_h <= 15 ) )
        {
            /* We are in an expected range */
            /* nCheck if 0-9 */
            if(value_l <= 9 )
            {
                add_value = value_l;
            }
            /* Check if a-f */
            else
            {
                /* 'a' is in fact 10 decimal in hexa */
                add_value = value_h + 10;
            }
            value = value*16 + add_value;               
        /*  PRINTF(DBG_REGISTRY,("value %d add_value %d  \n",value,add_value));*/


        }
        else
        {
            tmpIpAddr[i] = value;
        /*  PRINTF(DBG_REGISTRY,("tmpMacAddr[%d]  is %x\n",i,tmpMacAddr[i]));*/
            value = 0;
            i++;
        }

    }
#endif

}




/*-----------------------------------------------------------------------------

Routine Name:

    regConvertStringtoIpAddress

Routine Description: Converts the Ip Adrress in a form of string readen from the Registry 
to the Ip Address Array to be stored in the init_table struct 


Arguments:


Return Value:

    None

-----------------------------------------------------------------------------*/
void regConvertStringtoBeaconIETable(UINT8 *staIpAddressString,UINT8 *IpAddressArray, UINT8 size)
{
    char *ptr;
    UINT8 *tmpIpAddr;
    UINT8 value = 0, value_l, value_h, add_value;
    int i, str_len;

    /* Take the pointer to the string MAC Address to convert it to the Array MAC Address */
    ptr = (char *)staIpAddressString;
    tmpIpAddr = IpAddressArray;
    str_len = 3 * size - 1;
#if 0 
    for(i=0 ; i<size ; ptr++)
    {
        value_l = (*ptr-'0');

        /* PRINTF(DBG_REGISTRY,("value_l [%d] *ptr %c value %d\n",value_l,*ptr,value));*/

        if( value_l  < 9)
        {
            value = value*10 + value_l;             
            /* PRINTF(DBG_REGISTRY,("value %d value_l %d  \n",value,value_l));*/
        }
        else
        {
            tmpIpAddr[i] = value;
            /* PRINTF(DBG_REGISTRY,("tmpMacAddr[%d]  is %d\n",i,tmpMacAddr[i]));*/
            value = 0;
            i++;
        }

    }
#else
    for(i=0;(i < size);ptr++,str_len--)
    {
        if (str_len > 0) {
            /* The value can be or "0-9" or from "a-f" */
            value_l = (*ptr - '0');
            value_h = (*ptr - 'a');
        }
        else { /* last element */
            value_l = value_h = 16;
        }
        /*PRINTF(DBG_REGISTRY,("value_l [%d] value_h [%d] *ptr %c value %d\n",value_l,value_h,*ptr,value));*/

        if( (value_l <= 9) || (value_h <= 15 ) )
        {
            /* We are in an expected range */
            /* nCheck if 0-9 */
            if(value_l <= 9 )
            {
                add_value = value_l;
            }
            /* Check if a-f */
            else
            {
                /* 'a' is in fact 10 decimal in hexa */
                add_value = value_h + 10;
            }
            value = value * 16 + add_value;
            /*PRINTF(DBG_REGISTRY,("value %d add_value %d  \n",value,add_value));*/
        }
        else
        {
            tmpIpAddr[i] = value;
            /*PRINTF(DBG_REGISTRY,("tmpMacAddr[%d]  is %x\n",i,tmpMacAddr[i]));*/
            value = 0;
            i++;
        }
    }
#endif
}

//TRS:WDK provide callback functions
#if defined(_WINDOWS)
#endif
//TRS end


static void parse_hex_string(char * pString, tiUINT8 StrLength, tiUINT8 * pBuffer, tiUINT8 * Length)
{
    char ch;
    int iter = 0;

    while ((iter < StrLength) && ((ch = pString[iter]) != '\0'))
    {
        UINT8 val = ((ch >= '0' && ch <= '9') ? (ch - '0') : 
                     (ch >= 'A' && ch <= 'F') ? (0xA + ch - 'A') :
                     (ch >= 'a' && ch <= 'f') ? (0xA + ch - 'a') : 0);

        /* even indexes go to the lower nibble, odd indexes push them to the */
        /* higher nibble and then go themselves to the lower nibble. */
        if (iter % 2)
            pBuffer[iter / 2] = ((pBuffer[iter / 2] << (BIT_TO_BYTE_FACTOR / 2)) | val);
        else
            pBuffer[iter / 2] = val;

        ++iter;
    }

    /* iter = 0 len = 0, iter = 1 len = 1, iter = 2 len = 1, and so on... */
    *Length = (iter + 1) / 2;
}

static void parse_binary_string(char * pString, tiUINT8 StrLength, tiUINT8 * pBuffer, tiUINT8 * Length)
{
    char ch;
    int iter = 0;

    while ((iter < StrLength) && ((ch = pString[iter]) != '\0'))
    {
        UINT8 val = (ch == '1' ? 1 : 0);

        if (iter % BIT_TO_BYTE_FACTOR)
            pBuffer[iter / BIT_TO_BYTE_FACTOR] |= (val << (iter % BIT_TO_BYTE_FACTOR));
        else
            pBuffer[iter / BIT_TO_BYTE_FACTOR] = val;

        ++iter;
    }

    /* iter = 0 len = 0, iter = 1 len = 1, iter = 8 len = 1, and so on... */
    *Length = (iter + BIT_TO_BYTE_FACTOR - 1) / BIT_TO_BYTE_FACTOR;
}

static void parse_filter_request(rxDataFilterRequest_t * request, UINT8 offset, char * mask, UINT8 maskLength, char * pattern, UINT8 patternLength)
{
    request->offset = offset;
    request->maskLength = request->patternLength = 0;

    if (maskLength > 0)
    {
        parse_binary_string(mask, maskLength, request->mask, &request->maskLength);
        parse_hex_string(pattern, patternLength, request->pattern, &request->patternLength);
    }
}
