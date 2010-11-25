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

#ifndef __OSUTIL_H_
#define __OSUTIL_H_

/*used for asynchron ioctl purposes*/
void os_IoctlComplete(PTIWLN_ADAPTER_T pAdapter, TI_STATUS ReturnStatus) ;

NTSTATUS 
DispatchCommand(
	PTIWLN_ADAPTER_T pAdapter,
	ULONG ioControlCode,
	PULONG outBufLen,
	ULONG inBufLen,
	PVOID ioBuffer,
	PUINT8 pIoCompleteFlag
	);

ULONG UtilInfoCodeQueryInformation(	PTIWLN_ADAPTER_T pAdapter,PUCHAR pData,PULONG Length);
ULONG UtilInfoCodeSetInformation(	PTIWLN_ADAPTER_T pAdapter,PUCHAR pData,ULONG Length);

ULONG UtilGetSwVersion(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilSetBSSID(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilGetBSSID(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilSetSSID(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilGetSSID(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilNetworkTypesSupported(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilNetworkTypeInUseSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilNetworkTypeInUseGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilGetCurrentRssiLevel(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

VOID  RssiUtilIoctlCompleteCB(TI_HANDLE hAdapter,TI_STATUS status,PUINT8 pReadBuff);
VOID  SnrUtilIoctlCompleteCB(TI_HANDLE hAdapter,TI_STATUS status,PUINT8 pReadBuff);

ULONG UtilBssidListGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length, BOOLEAN ExtBssid, BOOLEAN allVarIes);
ULONG UtilBssidListScanOid(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilInfrastructureModeSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilInfrastructureModeGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilFragmentationThresholdSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilFragmentationThresholdGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilRtsThresholdSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilRtsThresholdGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilSupportedRates(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilSupportedRatesSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilCurrentRatesGet(PTIWLN_ADAPTER_T pAdapter,PUCHAR pData, PULONG Length);
ULONG UtilDesiredRatesGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilBasicRatesGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilBasicRatesSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilConfigurationGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilConfigurationSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilReadReg(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilWriteReg(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilGetCounter(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Offset);
ULONG UtilStatistics(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilTxStatistics(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG inLength, PULONG outLength);

ULONG UtilAddWep(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length, BOOLEAN CalledFromIoctl);
ULONG UtilRemoveWep(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilAddKey(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilRemoveKey(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilWepStatusSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilWepStatusGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilAuthenticationModeSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilAuthenticationModeGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilExtAuthenticationModeSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilExtAuthenticationModeGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilAssociationInfoGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG Util802CapabilityGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG Util802PmkidGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG Util802PmkidSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG Util802FSWAvailableOptionsGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG Util802FSWOptionsGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG Util802FSWOptionsSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG Util802EapTypeGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG Util802EapTypeSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);


ULONG UtilBthWlanCoeEnable(PTIWLN_ADAPTER_T pAdapter,	PUCHAR pData,	ULONG Length);
ULONG UtilBthWlanCoeRate(PTIWLN_ADAPTER_T pAdapter,	PUCHAR pData,	ULONG Length);
ULONG UtilBthWlanCoeConfig(PTIWLN_ADAPTER_T pAdapter,	PUCHAR pData,	ULONG Length);
ULONG UtilBthWlanCoeGetStatus(PTIWLN_ADAPTER_T pAdapter,	PUCHAR pData,	ULONG *Length);


ULONG UtilDesiredChannelSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilDesiredChannelGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilPowerModeSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilPowerModeGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilPowerLevelPSGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilPowerLevelPSSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilPowerLevelDefaultGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilPowerLevelDefaultSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilPowerLevelDozeModeGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilPowerLevelDozeModeSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilBeaconFilterDesiredStateSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilBeaconFilterDesiredStateGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
 
ULONG UtilShortPreambleSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilShortPreambleGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilShortRetryGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilShortRetrySet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilLongRetryGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilLongRetrySet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilRegulatoryDomain_enableDisable_802_11d( PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilRegulatoryDomain_enableDisable_802_11h( PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilRegulatoryDomain_Get_802_11d( PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilRegulatoryDomain_Get_802_11h( PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilRegulatoryDomain_setCountryIE_2_4( PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilRegulatoryDomain_getCountryIE_2_4( PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilRegulatoryDomain_setCountryIE_5( PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilRegulatoryDomain_getCountryIE_5( PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilRegulatoryDomain_setMinMaxDfsChannels( PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);  
ULONG UtilRegulatoryDomain_getMinMaxDfsChannels( PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length); 


ULONG UtilCurrentRegDomainGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilCurrentRegDomainSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilGetRegDomainTable(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilNumberOfAntennas(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilRxAntennaGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilRxAntennaSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilTxAntennaGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilTxAntennaSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilAntennaDivresitySet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilWepStatusGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG Util4xActiveStateGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilGetDesiredSSID(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilChannelGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilDesiredInfrastructureModeGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilDriverStatusGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilDriverStatusSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilIbssProtectionGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilIbssProtectionSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilShortSlotGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilShortSlotSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilExtRatesIeGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilExtRatesIeSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilSetTrafficIntensityThresholds(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilGetTrafficIntensityThresholds(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilToggleTrafficIntensityEvents(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilSendDiscreteScanTrigger(PTIWLN_ADAPTER_T pAdapter, PUCHAR ioBuffer, ULONG inBufLen);
ULONG UtilPollApPackets(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilPollApPacketsFromAC(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilConfigTxClassifier(PTIWLN_ADAPTER_T pAdapter,PUCHAR ioBuffer,ULONG inBufLen);
ULONG UtilRemoveClassifierEntry(PTIWLN_ADAPTER_T pAdapter,PUCHAR ioBuffer, ULONG inBufLen);
ULONG UtilGetClsfrType(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilGetAPQosParams(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilGetAsyncCurrentRssiLevel (PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilGetAsyncCurrentSnrRatio (PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilGetAPQosCapabilities(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilAddTspec(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilGetTspecParams(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilDeleteTspec(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilGetCurrentAcStatus(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilSetMediumUsageThreshold(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilSetPhyRateThreshold(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilGetMediumUsageThreshold(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilGetPhyRateThreshold(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilGetUserPriorityOfStream(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilGetDesiredPsMode(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilConfigTxClass(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilEnableEvent(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilDisableEvent(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilConfigEvents(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilGetDrvCapabilities(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilGetSelectedBSSIDInfo(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilGetPrimaryBSSIDInfo(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilGetDriverState (PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilStartAppScanSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilStopAppScanSet(PTIWLN_ADAPTER_T pAdapter,	PUCHAR pData, ULONG Length);
ULONG UtilDrvScanParamSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilScanPolicyParamSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilScanBssListGet(PTIWLN_ADAPTER_T pAdapter,	PUCHAR pData, PULONG Length);

ULONG UtilRssiGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilQosSetParams(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilQosSetRxTimeOut(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilSetDTagToAcMappingTable(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilSetVAD(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilGetVAD (PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilConfigRSSI(PTIWLN_ADAPTER_T pAdapter, UINT32 pData, ULONG Length);
ULONG UtilConfigPERLevel(PTIWLN_ADAPTER_T pAdapter, UINT32 pData, ULONG Length);

ULONG UtilPrivacyFilterSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilPrivacyFilterGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilDisassociate(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

VOID UtilDeviceSuspend(PTIWLN_ADAPTER_T pAdapter);
VOID UtilDeviceResume(PTIWLN_ADAPTER_T pAdapter);

ULONG UtilGetTxPowerValue(PTIWLN_ADAPTER_T pAdapter,externalParam_e ParamType,PUCHAR pData,ULONG Length);

ULONG UtilSetTxPowerDbm(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length	);
ULONG UtilGetTxPowerLevel(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length	);

ULONG UtilEnableDisableRxDataFilters(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilGetRxDataFiltersStatistics(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilAddRxDataFilter(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilRemoveRxDataFilter(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilGetAPTxPowerLevel(PTIWLN_ADAPTER_T pAdapter,externalParam_e ParamType,PUCHAR pData,ULONG Length);


ULONG UtilTxPowerLevelDbmGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilTxPowerLevelDbmSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);


ULONG UtilGetCountryCode(PTIWLN_ADAPTER_T pAdapter,externalParam_e ParamType,PUCHAR pData,ULONG Length);
ULONG UtilGetRegDomainBand(PTIWLN_ADAPTER_T pAdapter,externalParam_e ParamType,PUCHAR pData,ULONG Length);
ULONG UtilTestOid(PTIWLN_ADAPTER_T pAdapter,PUCHAR pData,ULONG Length);

ULONG UtilEarlyWakeupIeGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilEarlyWakeupIeSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilExcConfigurationGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilExcConfigurationSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilExcNetworkEapGet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilExcNetworkEapSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilExcRogueApDetectedSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilExcReportRogueApSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilExcAuthSuccessSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilExcCckmStartSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilExcCckmRequestSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilExcCckmResultSet(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilGetMACAddress(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG SendUserPacket(PTIWLN_ADAPTER_T pAdapter, TI_HANDLE pPacket,ULONG PacketLen);
ULONG UtilSetMixedMode(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilGetMixedMode(PTIWLN_ADAPTER_T pAdapter, externalParam_e ParamType, PUCHAR pData, PULONG Length);

ULONG UtilDriverSuspend(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilConfigRoamingParamsSet( PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilConfigRoamingParamsGet( PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);

ULONG UtilMeasurementEnableDisableParamsSet( PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilMeasurementMaxDurationParamsSet( PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);

ULONG UtilGetPowerConsumptionStatistics(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);


ULONG UtilPltReadRegister(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
VOID  UtilPltReadRegisterCB(TI_HANDLE hAdapter,TI_STATUS status,PUINT8 pReadBuff);
ULONG UtilPltWriteRegister(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
VOID  UtilPltRxPerCB(TI_HANDLE hAdapter,TI_STATUS status,PUINT8 pReadBuff);
ULONG UtilPltRxPerStart(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilPltRxPerStop(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilPltRxPerClear(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilPltRxPerGetResults(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length);
ULONG UtilPltTxStop(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilPltTxCW(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilPltTxContinues(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilPltWriteMib(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length);
ULONG UtilPltReadMib(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG pOutLength, ULONG InLength);
VOID UtilPltReadMibCB(TI_HANDLE hAdapter,TI_STATUS status,PUINT8 pReadBuff);
ULONG UtilPltRxTxCal(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG pOutLength, ULONG InLength);
VOID UtilPltRxTxCalCB(TI_HANDLE hAdapter,TI_STATUS status,PUINT8 pReadBuff);
ULONG UtilPltRxCal(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG pOutLength, ULONG InLength);
ULONG utilRxCalibrationStatus(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG pOutLength, ULONG InLength);
 
#endif		/*__OSUTIL_H_*/
