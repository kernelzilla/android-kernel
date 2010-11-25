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

//--------------------------------------------------------------------------
// Module:		TI_AdapterApi.cpp
//
// Purpose:		C interface implementation
//
//--------------------------------------------------------------------------

#include "CommonAdapter.h"
#ifdef _WINDOWS
#endif

tiBOOL TI_CheckAdapterObject(void *pObj);

/********************************************************************/
tiINT32      
TI_GetStatistics(TI_HANDLE hAdapter, TIWLN_STATISTICS* pStatistics)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetStatistics(pStatistics);
}
/********************************************************************/
tiINT32      
TI_GetTxStatistics(TI_HANDLE hAdapter, TIWLN_TX_STATISTICS* pTxStatistics, UINT32 clearStatsFlag)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetTxStatistics(pTxStatistics, clearStatsFlag);
}
/********************************************************************/
tiINT32
TI_EnableDisableRxDataFilters(TI_HANDLE hAdapter, tiBOOL enabled)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->EnableDisableRxDataFilters(enabled);
}
/********************************************************************/
tiINT32
TI_GetRxDataFiltersStatistics(TI_HANDLE hAdapter, TIWLAN_DATA_FILTER_STATISTICS * pStatistics)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetRxDataFiltersStatistics(pStatistics);
}
/********************************************************************/
tiINT32
TI_GetPowerConsumptionStatistics(TI_HANDLE hAdapter, PowerConsumptionTimeStat_t * pStatistics)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetPowerConsumptionStatistics(pStatistics);
                                              
}
/********************************************************************/
tiINT32
TI_AddRxDataFilter(TI_HANDLE hAdapter, TIWLAN_DATA_FILTER_REQUEST * pRequest)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->AddRxDataFilter(pRequest);
}
/********************************************************************/
tiINT32
TI_RemoveRxDataFilter(TI_HANDLE hAdapter, TIWLAN_DATA_FILTER_REQUEST * pRequest)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->RemoveRxDataFilter(pRequest);
}
/********************************************************************/
//returns RSSI
tiINT32      
TI_GetRSSI(TI_HANDLE hAdapter, tiINT32* pRssi)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetRSSI(pRssi);
}


/********************************************************************/
//returns SNR
tiINT32      
TI_GetSNR(TI_HANDLE hAdapter, tiUINT32* pSnr)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetSNR(pSnr);
}

/********************************************************************/
tiINT32      
TI_SetDTagToAcMappingTable(TI_HANDLE  hAdapter, acTrfcType_e* pDtagToAcTable )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetDTagToAcMappingTable(pDtagToAcTable);
}

/********************************************************************/
tiINT32      
TI_SetVAD(TI_HANDLE  hAdapter, txDataVadTimerParams_t* pVadTimer )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetVAD(pVadTimer);
}

/********************************************************************/
tiINT32      
TI_GetVAD(TI_HANDLE  hAdapter, txDataVadTimerParams_t* pVadTimer )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetVAD(pVadTimer);
}

/********************************************************************/
tiINT32      
TI_SetQosParameters(TI_HANDLE  hAdapter, OS_802_11_QOS_PARAMS* pQosParams )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetQosParameters(pQosParams);
}

/********************************************************************/
tiINT32      
TI_SetQosRxTimeOut(TI_HANDLE  hAdapter, OS_802_11_QOS_RX_TIMEOUT_PARAMS* pRxTimeOut )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetRxTimeOut(pRxTimeOut);
}

/********************************************************************/
tiINT32      
TI_GetAPQosParameters(TI_HANDLE  hAdapter, OS_802_11_AC_QOS_PARAMS* pACQosParams)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetAPQosParameters(pACQosParams);
}
/********************************************************************/
tiINT32      
TI_GetAPQosCapabilitesParameters(TI_HANDLE  hAdapter, OS_802_11_AP_QOS_CAPABILITIES_PARAMS* pAPQosCapabiltiesParams)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetAPQosCapabilitesParameters(pAPQosCapabiltiesParams);
}
/********************************************************************/
tiINT32      
TI_AddTspec	(TI_HANDLE  hAdapter, OS_802_11_QOS_TSPEC_PARAMS* pTspecParams)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->AddTspec(pTspecParams);
}
/********************************************************************/
tiINT32      
TI_GetTspecParameters(TI_HANDLE  hAdapter, OS_802_11_QOS_TSPEC_PARAMS* pTspecParams)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetTspecParameters(pTspecParams);
}
/********************************************************************/
tiINT32      
TI_DeleteTspec(TI_HANDLE  hAdapter, OS_802_11_QOS_DELETE_TSPEC_PARAMS* pDelTspecParams)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->DeleteTspec(pDelTspecParams);
}
/********************************************************************/
tiINT32      
TI_GetCurrentACStatus(TI_HANDLE  hAdapter, OS_802_11_AC_UPSD_STATUS_PARAMS *pAcStatusParams)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetCurrentACStatus(pAcStatusParams);
}
/********************************************************************/
tiINT32      
TI_SetMediumUsageThreshold(TI_HANDLE  hAdapter, OS_802_11_THRESHOLD_CROSS_PARAMS* pThresholdCrossParams)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetMediumUsageThreshold(pThresholdCrossParams);
}
/********************************************************************/
tiINT32      
TI_SetPhyRateThreshold(TI_HANDLE  hAdapter, OS_802_11_THRESHOLD_CROSS_PARAMS* pThresholdCrossParams)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetPhyRateThreshold(pThresholdCrossParams);
}
/********************************************************************/
tiINT32      
TI_GetMediumUsageThreshold(TI_HANDLE  hAdapter, OS_802_11_THRESHOLD_CROSS_PARAMS* pThresholdCrossParams)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetMediumUsageThreshold(pThresholdCrossParams);
}
/********************************************************************/
tiINT32      
TI_GetPhyRateThreshold(TI_HANDLE  hAdapter, OS_802_11_THRESHOLD_CROSS_PARAMS* pThresholdCrossParams)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetPhyRateThreshold(pThresholdCrossParams);
}
/********************************************************************/
tiINT32	    
TI_PollApPackets(TI_HANDLE  hAdapter)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->PollApPackets();
}
/********************************************************************/
tiINT32     
TI_PollApPacketsFromAC(TI_HANDLE  hAdapter, tiUINT32 AC)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->PollApPacketsFromAC(AC);
}
/********************************************************************/
tiINT32
TI_SetTrafficIntensityThresholds (TI_HANDLE  hAdapter, OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS* pTrafficThresholds)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetTrafficIntensityThresholds(pTrafficThresholds);
}
/********************************************************************/
tiINT32
TI_GetTrafficIntensityThresholds (TI_HANDLE  hAdapter, OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS* pTrafficThresholds)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetTrafficIntensityThresholds(pTrafficThresholds);
}
/********************************************************************/
tiINT32
TI_ToggleTrafficIntensityEvents (TI_HANDLE  hAdapter, tiUINT32 NewStatus )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->ToggleTrafficIntensityEvents(NewStatus);
}
/********************************************************************/
tiINT32     
TI_GetBSSIDList(TI_HANDLE  hAdapter, OS_802_11_BSSID_LIST_EX** ppBSSIDlist)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetBSSIDList(ppBSSIDlist);
}
/********************************************************************/
tiINT32     
TI_GetFullBSSIDList(TI_HANDLE  hAdapter, OS_802_11_BSSID_LIST_EX** ppBSSIDlist)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetFullBSSIDList(ppBSSIDlist);
}

/********************************************************************/
tiINT32      
TI_GetCurrentSSID(TI_HANDLE  hAdapter, OS_802_11_SSID* pSSID)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetCurrentSSID(pSSID);
}
/********************************************************************/
tiINT32      
TI_GetCurrentRate(TI_HANDLE  hAdapter, tiUINT32*  puCurrentRate)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetCurrentRate(puCurrentRate);
}
/********************************************************************/
tiINT32      
TI_GetCurrentAddress(TI_HANDLE  hAdapter, OS_802_11_MAC_ADDRESS*    pCurrentAddr)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetCurrentAddress(pCurrentAddr);
}
/********************************************************************/
tiINT32      
TI_GetDesiredSSID(TI_HANDLE  hAdapter, OS_802_11_SSID* pSSID)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetDesiredSSID(pSSID);
}
/********************************************************************/
tiINT32      
TI_SetSSID(TI_HANDLE hAdapter, tiUINT8* pSSIDname)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetSSID(pSSIDname);
}
/********************************************************************/
tiINT32 
TI_SetBSSType(TI_HANDLE hAdapter, OS_802_11_NETWORK_MODE uBSSType)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetBSSType(uBSSType);
}
/********************************************************************/
tiINT32      
TI_GetBSSType(TI_HANDLE hAdapter, OS_802_11_NETWORK_MODE* puBSSType)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetBSSType(puBSSType);
}
/********************************************************************/
tiINT32      
TI_SetEAPType(TI_HANDLE hAdapter, OS_802_11_EAP_TYPES uEAPType)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetEAPType(uEAPType);
}

/********************************************************************/
tiINT32      
TI_SetEAPTypeDriver(TI_HANDLE hAdapter, OS_802_11_EAP_TYPES uEAPType)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetEAPTypeDriver(uEAPType);
}

/********************************************************************/
tiINT32      
TI_GetEAPType(TI_HANDLE hAdapter, OS_802_11_EAP_TYPES* puEAPType)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetEAPType(puEAPType);
}

/********************************************************************/
tiINT32      
TI_SetEncryptionType(TI_HANDLE hAdapter, OS_802_11_ENCRYPTION_TYPES uEncryptType)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetEncryptionType(uEncryptType);
}
/********************************************************************/
tiINT32      
TI_GetEncryptionType(TI_HANDLE hAdapter, OS_802_11_ENCRYPTION_TYPES* puEncryptType)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetEncryptionType(puEncryptType);
}
/********************************************************************/
tiINT32     
TI_SetCredentials(TI_HANDLE  hAdapter, tiCHAR* pszUserName, tiCHAR* pszPassword )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetCredentials( pszUserName, pszPassword );
}
/********************************************************************
Sets the PSK password phrase for WPA encryption.
Parameters:
    hAdapter        -   The handle to the Adapter object returned 
                        by TI_AdapterInit 
    pszPSK          -   A null-terminated string that contains the 
                        PSK password phrase
********************************************************************/
tiINT32     
TI_SetPSK(TI_HANDLE  hAdapter, tiCHAR* pszPSK )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetPSK( pszPSK);
}                                   

/********************************************************************/
tiINT32     
TI_SetCertificateParameters(TI_HANDLE  hAdapter, tiVOID* pData, tiBOOL bValidateServerCert )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
#ifdef _WINDOWS
#else
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetCertParamsFileName((tiCHAR*) pData, bValidateServerCert);
#endif
}
/********************************************************************/
tiINT32      
TI_SetTxPowerDbm(TI_HANDLE hAdapter, tiUINT8 uTxPower)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetTxPowerDbm(uTxPower);
}
/********************************************************************/
tiINT32      
TI_GetTxPowerLevel(TI_HANDLE hAdapter, tiCHAR* puTxPower)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetTxPowerLevel(puTxPower);
}

/********************************************************************/
tiINT32      
TI_GetTxPowerDbm(TI_HANDLE hAdapter, tiCHAR* puTxPower)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetTxPowerDbm(puTxPower);
}

/********************************************************************/
tiINT32
TI_Set4XState(TI_HANDLE hAdapter, tiBOOL bStatus)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->Set4XState(bStatus);
}
/********************************************************************/
tiINT32      
TI_Get4XState(TI_HANDLE hAdapter, tiBOOL* pbStatus)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->Get4XState(pbStatus);
}

/********************************************************************/
tiINT32      
TI_GetDesiredRate(TI_HANDLE hAdapter, tiUINT32* puDesiredRates)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetDesiredRate(puDesiredRates);
}
/********************************************************************/
tiINT32      
TI_SetFragmentThreshold(TI_HANDLE hAdapter, tiUINT32 uFragmentThreshold)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetFragmentThreshold(uFragmentThreshold);
}
/********************************************************************/
tiINT32
TI_GetFragmentThreshold(TI_HANDLE hAdapter, tiUINT32* puFragmentThreshold)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetFragmentThreshold(puFragmentThreshold);
}
/********************************************************************/
tiINT32      
TI_SetRTSThreshold(TI_HANDLE  hAdapter, tiUINT32 uRTSThreshold)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetRTSThreshold(uRTSThreshold);
}
/********************************************************************/
tiINT32      
TI_GetRTSThreshold(TI_HANDLE hAdapter, tiUINT32* puRTSThreshold)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetRTSThreshold(puRTSThreshold);
}
/********************************************************************/
tiINT32      
TI_SetShortPreamble(TI_HANDLE hAdapter, tiUINT32 uShortPreamble)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetShortPreamble(uShortPreamble);
}
/********************************************************************/
tiINT32      
TI_GetShortPreamble(TI_HANDLE  hAdapter, tiUINT32* puShortPreamble)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetShortPreamble(puShortPreamble);
}
/********************************************************************/
tiINT32      
TI_SetShortRetry(TI_HANDLE  hAdapter,tiUINT32 uShortRetry)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetShortRetry(uShortRetry);
}
/********************************************************************/
tiINT32
TI_GetShortRetry(TI_HANDLE hAdapter, tiUINT32* puShortRetry )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetShortRetry(puShortRetry);
}
/********************************************************************/
tiINT32      
TI_SetLongRetry(TI_HANDLE hAdapter, tiUINT32 uLongRetry)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetLongRetry(uLongRetry);
}
/********************************************************************/
tiINT32      
TI_GetLongRetry(TI_HANDLE hAdapter, tiUINT32* puLongRetry)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetLongRetry(puLongRetry);
}
/********************************************************************/
tiINT32
TI_GetSupportedNetworkTypes(TI_HANDLE hAdapter, OS_802_11_NETWORK_TYPE* pNetTypeLst, tiUINT32 uMaxNetTypes)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetSupportedNetworkTypes(pNetTypeLst, uMaxNetTypes);
}
/********************************************************************/
tiINT32      
TI_SetNetworkTypeInUse(TI_HANDLE  hAdapter, OS_802_11_NETWORK_TYPE uNetType)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetNetworkTypeInUse(uNetType);
}
/********************************************************************/
tiINT32      
TI_GetNetworkTypeInUse(TI_HANDLE  hAdapter, OS_802_11_NETWORK_TYPE* puNetType)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetNetworkTypeInUse(puNetType);
}
/********************************************************************/
tiINT32	   
TI_SetKeyType(TI_HANDLE hAdapter, OS_802_11_KEY_TYPES uKeyType )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetKeyType(uKeyType);
}
/********************************************************************/
tiINT32	   
TI_SetMixedMode(TI_HANDLE  hAdapter,tiBOOL  bStatus)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetMixedMode(bStatus);
}
/********************************************************************/
tiINT32
TI_GetMixedMode(TI_HANDLE  hAdapter, tiBOOL* pbStatus)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetMixedMode(pbStatus);
}
/********************************************************************/
tiINT32      
TI_GetBSSID(TI_HANDLE  hAdapter,OS_802_11_MAC_ADDRESS* pAddrBSSID)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetBSSID(pAddrBSSID);
}
/********************************************************************/
tiINT32 
TI_SetBSSID(TI_HANDLE  hAdapter,OS_802_11_MAC_ADDRESS* pAddrBSSID)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetBSSID(pAddrBSSID);
}

/********************************************************************/
tiINT32
TI_SetRSSITrigger(TI_HANDLE  hAdapter,tiBOOL  bRSSItr)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetRSSITrigger(bRSSItr);
}
/********************************************************************/
tiINT32
TI_GetRSSITrigger(TI_HANDLE  hAdapter, tiBOOL* pbRSSItr)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetRSSITrigger(pbRSSItr);
}
/********************************************************************/
tiINT32      
TI_SetAntennaDiversityParams( TI_HANDLE hAdapter, PTIWLAN_ANT_DIVERSITY pAntennaDiversityOptions )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetAntennaDiversityParams(pAntennaDiversityOptions);
}
/********************************************************************/
tiINT32      
TI_SetWEPStatus(TI_HANDLE  hAdapter, tiUINT32 uWEPStatus)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetWEPStatus(uWEPStatus);
}
/********************************************************************/
tiINT32      
TI_GetWEPStatus(TI_HANDLE  hAdapter,tiUINT32* puWEPStatus)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetWEPStatus(puWEPStatus);
}
/********************************************************************/
tiINT32      
TI_SetDesiredChannel(TI_HANDLE  hAdapter,tiUINT32 uDesiredChannel)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetDesiredChannel(uDesiredChannel);
}
/********************************************************************/
tiINT32      
TI_GetDesiredChannel(TI_HANDLE  hAdapter,tiUINT32* puDesiredChannel)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetDesiredChannel(puDesiredChannel);
}
/********************************************************************/
tiINT32 
TI_GetCurrentChannel(TI_HANDLE  hAdapter, tiUINT32*  puCurrentChannel )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetCurrentChannel(puCurrentChannel);
}
/********************************************************************/
tiINT32      
TI_SetSupportedRates(TI_HANDLE  hAdapter, tiUINT8* pSupportedRatesLst, tiUINT32 uBufLength)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetSupportedRates(pSupportedRatesLst, uBufLength);
}
/********************************************************************/
tiINT32      
TI_GetSupportedRates(TI_HANDLE  hAdapter, tiUINT8* pSupportedRatesLst, tiUINT32 uBufLength )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetSupportedRates(pSupportedRatesLst, uBufLength);
}
/********************************************************************/
tiINT32      
TI_SetConfiguration(TI_HANDLE  hAdapter, OS_802_11_CONFIGURATION* pConfiguration)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetConfiguration(pConfiguration);
}
/********************************************************************/
tiINT32
TI_GetConfiguration(TI_HANDLE hAdapter, OS_802_11_CONFIGURATION* pConfiguration)    
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetConfiguration(pConfiguration);
}
/********************************************************************/
tiINT32
TI_SetAuthenticationMode(TI_HANDLE  hAdapter, OS_802_11_AUTHENTICATION_MODE uAuthenticationMode)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetAuthenticationMode(uAuthenticationMode);
}
/********************************************************************/
tiINT32      
TI_GetAuthenticationMode(TI_HANDLE  hAdapter, OS_802_11_AUTHENTICATION_MODE* puAuthenticationMode)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetAuthenticationMode(puAuthenticationMode);
}
/********************************************************************/
tiINT32
TI_SetPrivacyFilter(TI_HANDLE  hAdapter, tiUINT32 uPrivacyFilter)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetPrivacyFilter(uPrivacyFilter);
}
/********************************************************************/
tiINT32      
TI_GetPrivacyFilter(TI_HANDLE  hAdapter, tiUINT32* puPrivacyFilter)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetPrivacyFilter(puPrivacyFilter);
}

/********************************************************************/
tiINT32      
TI_EnableDisable_802_11d(TI_HANDLE  hAdapter, tiUINT8 enableDisable_802_11d)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->EnableDisable_802_11d(enableDisable_802_11d);
}
/********************************************************************/
tiINT32      
TI_Get_802_11d(TI_HANDLE  hAdapter, tiUINT8 *enableDisable_802_11d)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->Get_802_11d(enableDisable_802_11d);
}

/********************************************************************/
tiINT32      
TI_EnableDisable_802_11h(TI_HANDLE  hAdapter, tiUINT8 enableDisable_802_11h)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->EnableDisable_802_11h(enableDisable_802_11h);
}

/********************************************************************/
tiINT32      
TI_Get_802_11h(TI_HANDLE  hAdapter, tiUINT8 *enableDisable_802_11h)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->Get_802_11h(enableDisable_802_11h);
}

/********************************************************************/
tiINT32      
TI_Set_countryIeFor2_4_Ghz(TI_HANDLE  hAdapter, country_t countryIe)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->Set_countryIeFor2_4_Ghz(countryIe);
}

/********************************************************************/
tiINT32      
TI_Get_countryIeFor2_4_Ghz(TI_HANDLE  hAdapter, tiUINT8 **countryString)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->Get_countryIeFor2_4_Ghz(countryString);
}

/********************************************************************/
tiINT32      
TI_Set_countryIeFor5_Ghz(TI_HANDLE  hAdapter, country_t countryIe)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->Set_countryIeFor5_Ghz(countryIe);
}

/********************************************************************/
tiINT32      
TI_Get_countryIeFor5_Ghz(TI_HANDLE  hAdapter, tiUINT8 **countryString)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->Get_countryIeFor5_Ghz(countryString);
}

/********************************************************************/
tiINT32      
TI_Set_minMaxDfsChannels(TI_HANDLE  hAdapter, DFS_ChannelRange_t DFS_ChannelRange)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->Set_minMaxDfsChannels(DFS_ChannelRange);
}

/********************************************************************/
tiINT32      
TI_Get_minMaxDfsChannels(TI_HANDLE  hAdapter, DFS_ChannelRange_t *DFS_ChannelRange)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->Get_minMaxDfsChannels(DFS_ChannelRange);
}

/********************************************************************/
tiINT32
TI_GetDriverState(TI_HANDLE  hAdapter, driverState_e* puDriverState )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetDriverState(puDriverState);
}
/********************************************************************/
tiINT32
TI_SetIBSSProtection(TI_HANDLE  hAdapter, tiUINT32   uProtection )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetIBSSProtection(uProtection);
}
/********************************************************************/
tiINT32
TI_GetIBSSProtection(TI_HANDLE  hAdapter, tiUINT32*  puProtection)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetIBSSProtection(puProtection);
}
/********************************************************************/
tiINT32      
TI_SetShortSlot(TI_HANDLE  hAdapter, tiUINT32 uShortSlot)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetShortSlot(uShortSlot);
}
/********************************************************************/
tiINT32
TI_GetShortSlot(TI_HANDLE  hAdapter, tiUINT32*  puShortSlot)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetShortSlot(puShortSlot);
}
/********************************************************************/
tiINT32      
TI_SetExtRatesIE(TI_HANDLE  hAdapter, tiUINT32 uExtRatesIE)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetExtRatesIE(uExtRatesIE);
}
/********************************************************************/
tiINT32      
TI_SetEarlyWakeupMode(TI_HANDLE  hAdapter, tiUINT8 uEarlyWakeup)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetEarlyWakeupMode(uEarlyWakeup);
}
/********************************************************************/
tiINT32      
TI_GetEarlyWakeupMode(TI_HANDLE  hAdapter, tiUINT8* uEarlyWakeup)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetEarlyWakeupMode(uEarlyWakeup);
}
/********************************************************************/
tiINT32      
TI_GetExtRatesIE(TI_HANDLE  hAdapter, tiUINT32* puExtRatesIE)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetExtRatesIE(puExtRatesIE);
}
/********************************************************************/
tiINT32
TI_AddWEPKey(TI_HANDLE  hAdapter, OS_802_11_WEP* pWEP)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->AddWEPKey(pWEP);
}
/********************************************************************/
tiINT32      
TI_RemoveWEPKey(TI_HANDLE  hAdapter, tiUINT32 uKeyIndex)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->RemoveWEPKey(uKeyIndex);
}
/********************************************************************/
tiINT32
TI_AddKey(TI_HANDLE  hAdapter, OS_802_11_KEY* pKey)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->AddKey(pKey);
}
/********************************************************************/
tiINT32     
TI_RemoveKEY(TI_HANDLE  hAdapter, OS_802_11_REMOVE_KEY* pRemoveKey  ) 
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->RemoveKey(pRemoveKey);
}
/********************************************************************/
tiINT32     
TI_GetPowerMode(TI_HANDLE  hAdapter, OS_802_11_POWER_PROFILE* pPowerProfile )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetPowerMode(pPowerProfile);
}
/********************************************************************/
tiINT32
TI_SetPowerMode(TI_HANDLE  hAdapter, OS_802_11_POWER_PROFILE uPowerProfile )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetPowerMode(uPowerProfile);
}
/********************************************************************/
tiINT32     
TI_SetPowerLevelPS(TI_HANDLE  hAdapter, OS_802_11_POWER_LEVELS uPowerLevel )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetPowerLevelPS(uPowerLevel);
}
/********************************************************************/
tiINT32     
TI_GetPowerLevelPS(TI_HANDLE  hAdapter, OS_802_11_POWER_LEVELS* pPowerLevel )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetPowerLevelPS(pPowerLevel);
}
/********************************************************************/
tiINT32     
TI_SetPowerLevelDefault(TI_HANDLE  hAdapter, OS_802_11_POWER_LEVELS uPowerLevel )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetPowerLevelDefault(uPowerLevel);
}
/********************************************************************/
tiINT32     
TI_GetPowerLevelDefault(TI_HANDLE  hAdapter, OS_802_11_POWER_LEVELS* pPowerLevel )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetPowerLevelDefault(pPowerLevel);
}
/********************************************************************/
tiINT32     
TI_SetPowerLevelDozeMode(TI_HANDLE  hAdapter, OS_802_11_POWER_PROFILE uPowerLevel )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetPowerLevelDozeMode(uPowerLevel);
}
/********************************************************************/
tiINT32     
TI_GetPowerLevelDozeMode(TI_HANDLE  hAdapter, OS_802_11_POWER_PROFILE* pPowerLevel )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetPowerLevelDozeMode(pPowerLevel);
}
/********************************************************************/
tiINT32     
TI_SetBeaconFilterDesiredState(TI_HANDLE  hAdapter, OS_802_11_BEACON_FILTER_MODE uBeaconFilterDesiredState )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetBeaconFilterDesiredState(uBeaconFilterDesiredState);
}

/********************************************************************/
tiINT32     
TI_GetBeaconFilterDesiredState(TI_HANDLE  hAdapter, tiUINT8* pBeaconFilterState )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetBeaconFilterDesiredState(pBeaconFilterState);
}


/********************************************************************/
tiINT32
TI_ConfigPowerManagement(TI_HANDLE  hAdapter, OS_802_11_POWER_PROFILE uPowerProfile )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->ConfigPowerManagement(uPowerProfile);
}

/********************************************************************/
tiINT32     
TI_RegisterEvent(TI_HANDLE  hAdapter, IPC_EVENT_PARAMS*  pEventParams )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->RegisterEvent(pEventParams);
}
/********************************************************************/
tiINT32     
TI_UnRegisterEvent(TI_HANDLE  hAdapter, IPC_EVENT_PARAMS*	 pEventParams)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->UnRegisterEvent(pEventParams);
}
/********************************************************************/
tiINT32      
TI_hwReadRegister(TI_HANDLE  hAdapter, tiUINT32 uRegisterAddr, tiUINT32* puValue )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->hwReadRegister(uRegisterAddr, puValue);
}

/********************************************************************/
tiINT32     
TI_StartScan(TI_HANDLE  hAdapter, scan_Params_t *pScanParams)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->StartScan(pScanParams);
}
/********************************************************************/
tiINT32     
TI_StopScan(TI_HANDLE  hAdapter)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->StopScan();
}
/********************************************************************/
tiINT32
TI_SetScanPolicy(TI_HANDLE  hAdapter, UINT8* buffer, UINT16 bufferLength)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetScanPolicy(buffer, bufferLength);
}
/********************************************************************/
tiINT32
TI_GetScanBssList(TI_HANDLE  hAdapter, bssList_t* bssList)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetScanBssList(bssList);
}
/********************************************************************/
tiINT32	    
TI_ConfigTxClassifier(TI_HANDLE  hAdapter, 
                      tiUINT32 inParamsBuffLen,
                      tiUINT8  *inParamsBuff)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->ConfigTxClassifier(inParamsBuffLen,inParamsBuff);
}
/********************************************************************/
tiINT32	    
TI_RemoveClassifierEntry(TI_HANDLE  hAdapter, clsfr_tableEntry_t *pClsfrEntry)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->RemoveClassifierEntry(pClsfrEntry);
}
/********************************************************************/
tiINT32 
TI_GetClsfrType(TI_HANDLE  hAdapter, 
                clsfrTypeAndSupport *currClsfrType)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetClsfrType(currClsfrType);
}
/********************************************************************/
tiINT32 
TI_GetDesiredPsMode(TI_HANDLE  hAdapter, OS_802_11_QOS_DESIRED_PS_MODE *desiredPsMode)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetDesiredPsMode(desiredPsMode);
}
/********************************************************************/
tiINT32 
TI_GetDriverCapabilities (TI_HANDLE  hAdapter, OS_802_11_DRIVER_CAPABILITIES* pDriverCapabilities )
{
   if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetDriverCapabilities(pDriverCapabilities);
}
/********************************************************************/
tiINT32	TI_GetSelectedBSSIDInfo(TI_HANDLE hAdapter, OS_802_11_BSSID_EX  *pSelectedBSSIDInfo )
{
	tiINT32 Rssi;

   if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;

   /* Query the RSSI so it will be updated in the Site Manager in CORE so the BSSID List retrive will be updated with
   the correct current RSSI */
   ((TI_WLAN_AdapterAPI *) hAdapter)->GetRSSI(&Rssi);	
   
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetSelectedBSSIDInfo(pSelectedBSSIDInfo);
}
/********************************************************************/
tiINT32	TI_GetPrimaryBSSIDInfo(TI_HANDLE hAdapter, OS_802_11_BSSID_EX  *pSelectedBSSIDInfo)
{
   tiINT32 Rssi;

   if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;

   /* Query the RSSI so it will be updated in the Site Manager in CORE so the BSSID List retrive will be updated with
   the correct current RSSI */
   ((TI_WLAN_AdapterAPI *) hAdapter)->GetRSSI(&Rssi);	   

    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetPrimaryBSSIDInfo(pSelectedBSSIDInfo);
}
/********************************************************************/
tiINT32      
TI_hwWriteRegister(TI_HANDLE  hAdapter, tiUINT32 uRegisterAddr, tiUINT32 dwValue )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->hwWriteRegister(uRegisterAddr, dwValue);
}
/********************************************************************/
tiINT32 
TI_Disassociate(TI_HANDLE  hAdapter)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->Disassociate();
}
/********************************************************************/
tiUINT32      
TI_ReloadDefaults(TI_HANDLE  hAdapter)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->ReloadDefaults();
}
/********************************************************************/
tiINT32	    
TI_IsDriverLoaded(TI_HANDLE  hAdapter)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->IsDriverLoaded();
}
/********************************************************************/
tiINT32      
TI_GetNumberOfAntennas(TI_HANDLE hAdapter, tiUINT32* puNumberOfAntennas)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetNumberOfAntennas(puNumberOfAntennas);
}
/********************************************************************/
tiINT32      
TI_GetDriverVersion(TI_HANDLE  hAdapter, TIWLN_VERSION_EX* pdrvVersion)               
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetDriverVersion(pdrvVersion);
}


/********************************************************************/
tiINT32      
TI_SetBtCoeEnable(TI_HANDLE  hAdapter, tiUINT32 ModeEnable)               
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetBtCoeEnable(ModeEnable);
}
/********************************************************************/
tiINT32      
TI_SetBtCoeRate(TI_HANDLE  hAdapter, tiUINT8 *pRate)               
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetBtCoeRate(pRate);
}
/********************************************************************/
tiINT32      
TI_SetBtCoeConfig(TI_HANDLE  hAdapter, tiUINT32 *pConfig)               
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetBtCoeConfig(pConfig);
}

/********************************************************************/
tiINT32      
TI_SetBtCoeGetStatus(TI_HANDLE  hAdapter, tiUINT32 *pStatus)               
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetBtCoeGetStatus(pStatus);
}



#ifdef TI_DBG
/********************************************************************/
tiUINT32      
TI_DebugBuffer(TI_HANDLE  hAdapter, tiUINT8* pBuffer, tiUINT32  uLenght)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetDebugBuffer(pBuffer, uLenght);
}
/********************************************************************/
tiINT32
TI_SetReportModule(TI_HANDLE  hAdapter, tiUINT8* pData)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetReportModule(pData);
}
/********************************************************************/
tiINT32
TI_GetReportModule(TI_HANDLE  hAdapter, tiUINT8* pData)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetReportModule(pData);
}
/********************************************************************/
tiINT32     
TI_SetOsDbgState(TI_HANDLE  hAdapter, tiUINT32 uData)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetOsDbgState(uData);
}
/********************************************************************/
tiINT32     
TI_GetOsDbgState(TI_HANDLE  hAdapter, tiUINT32* puData)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetOsDbgState(puData);
}
/********************************************************************/
tiINT32     
TI_SetReportSeverity(TI_HANDLE  hAdapter, tiUINT8* pData)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetReportSeverity(pData);
}
/********************************************************************/
tiINT32     
TI_GetReportSeverity(TI_HANDLE  hAdapter, tiUINT8* pData)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetReportSeverity(pData);
}
/********************************************************************/
tiINT32
TI_SetReportPPMode(TI_HANDLE  hAdapter, tiUINT32 uData)	    
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetReportPPMode(uData);
}
/********************************************************************/
tiINT32
TI_DisplayStats(TI_HANDLE  hAdapter, tiUINT8* puDbgBuffer, tiUINT32 uBuffSize)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->DisplayStats(puDbgBuffer, uBuffSize);
}
#endif //TI_DBG
/********************************************************************/
tiINT32      
TI_GetRegDomainTable(TI_HANDLE  hAdapter, TIWLN_REGDOMAINS* pRegDomainTable)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetRegDomainTable(pRegDomainTable);
}
/********************************************************************/
tiINT32      
TI_GetMediumUsage(TI_HANDLE  hAdapter, TIWLN_MEDIUM_USAGE* pMediumUsage)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetMediumUsage(pMediumUsage);
}
/********************************************************************/
tiINT32
TI_GetApiVersion(TI_HANDLE  hAdapter, tiUINT32* puApiVersion)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetApiVersion(puApiVersion);
}
/********************************************************************/
tiINT32
TI_StartSM(TI_HANDLE  hAdapter)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->StartSM();
}
/********************************************************************/
tiINT32     
TI_StopSM( TI_HANDLE  hAdapter )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->StopSM();
}
/********************************************************************/
tiINT32     
TI_GetAssociationInfo(TI_HANDLE  hAdapter, OS_802_11_ASSOCIATION_INFORMATION** ppInfo) 
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;

    if ( ppInfo == NULL)
        return TI_RESULT_FAILED;

    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetAssociationInfo( ppInfo );
}
/********************************************************************/
TI_HANDLE   
TI_AdapterInit(tiCHAR*  pszAdapterName)
{
    return (TI_HANDLE) TI_AdapterCppInit(pszAdapterName, FALSE); 
}

/********************************************************************/
tiINT32      
TI_AdapterDeinit(TI_HANDLE  hAdapter)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    TI_AdapterCppDeinit((TI_WLAN_AdapterAPI *) hAdapter, FALSE );
    return TI_RESULT_OK;
}
/********************************************************************/
tiINT32     
TI_Start(TI_HANDLE  hAdapter)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->Start();
}
/********************************************************************/    
tiINT32     
TI_Stop(TI_HANDLE  hAdapter)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->Stop();
}
/********************************************************************/
tiINT32     
TI_WLAN_IsDriverRun(TI_HANDLE  hAdapter, tiBOOL* pbStatus)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->IsDriverRun(pbStatus);
}
/********************************************************************/
tiINT32     
TI_SetWpaOptions(TI_HANDLE  hAdapter, tiUINT32      fWPAOptions )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->SetWPAOptions(fWPAOptions);
}
/********************************************************************/
tiINT32 TI_GetWpaOptions(TI_HANDLE  hAdapter, tiUINT32 * fWPAOptions )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
	return TI_RESULT_FAILED;

     //return ((TI_WLAN_AdapterAPI *) hAdapter)->GetWPAOptions(fWPAOptions);
}

/********************************************************************/
tiINT32     
TI_SetRoamingConfiguration(TI_HANDLE  hAdapter, UINT8* buffer, UINT16 bufferLength)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->Set_RoamingConfParams(buffer, bufferLength);
}

tiINT32     
TI_GetRoamingConfiguration(TI_HANDLE  hAdapter, UINT8* buffer, UINT16 bufferLength)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;

    return ((TI_WLAN_AdapterAPI *) hAdapter)->Get_RoamingConfParams(buffer, bufferLength);
}
/********************************************************************/
tiINT32     
TI_GWSICommand(TI_HANDLE  hAdapter, tiUINT32* pGWSICommand )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GWSICommand(pGWSICommand);

}

/********************************************************************/
tiINT32     
TI_GWSIInitialize(TI_HANDLE  hAdapter, tiUINT32* pGWSICommand )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GWSIInitialize(pGWSICommand);
	
}

/********************************************************************/
tiINT32     
TI_GWSIConfig(TI_HANDLE  hAdapter, tiUINT32* pGWSICommand )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GWSIConfig(pGWSICommand);
	
}

/********************************************************************/
tiINT32     
TI_GWSIRelease(TI_HANDLE  hAdapter, tiUINT32* pGWSICommand )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GWSIRelease(pGWSICommand);
	
}

/********************************************************************/

tiINT32     
TI_GWSIGetInitTable(TI_HANDLE  hAdapter, tiUINT32* pGWSICommand )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GWSIGetInitTable(pGWSICommand);

}
/********************************************************************/
  
//PLT

tiINT32
TI_PLT_ReadRegister(TI_HANDLE  hAdapter, UINT32 uRegisterAddr, PUINT32 uRegisterData )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->PLT_ReadRegister( uRegisterAddr,  uRegisterData );
}

tiINT32
TI_PLT_WriteRegister(TI_HANDLE  hAdapter, UINT32 uRegisterAddr, UINT32 uRegisterData )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->PLT_WriteRegister( uRegisterAddr, uRegisterData );
}


tiINT32
TI_PLT_RxPerStart(TI_HANDLE  hAdapter)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->PLT_RxPerStart();
}

tiINT32
TI_PLT_RxPerStop(TI_HANDLE  hAdapter)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->PLT_RxPerStop();
}

tiINT32
TI_PLT_RxPerClear(TI_HANDLE  hAdapter)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->PLT_RxPerClear();
}

tiINT32
TI_PLT_RxPerGetResults(TI_HANDLE  hAdapter, PltRxPer_t* pPltRxPer )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->PLT_RxPerGetResults( pPltRxPer );
}

tiINT32
TI_PLT_TxCW(TI_HANDLE  hAdapter, TestCmdChannelBand_t* pPltTxCW)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->PLT_TxCW(pPltTxCW);
}

tiINT32
TI_PLT_TxContiues(TI_HANDLE  hAdapter, PltTxContinues_t* pPltTxContinues)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->PLT_TxContiues( pPltTxContinues);
}
	
tiINT32
TI_PLT_TxStop(TI_HANDLE  hAdapter)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->PLT_TxStop( );
}

tiINT32
TI_PLT_ReadMIB(TI_HANDLE  hAdapter, PLT_MIB_t* pMib )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->PLT_ReadMIB(pMib);
}

tiINT32
TI_PLT_WriteMIB(TI_HANDLE  hAdapter, PLT_MIB_t* pMib )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->PLT_WriteMIB(pMib);
}

tiINT32		
TI_GetDefaultWepKey(TI_HANDLE  hAdapter, tiUINT32* puKeyIndex )
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->GetDefaultWepKey(puKeyIndex);
}

tiINT32
TI_PLT_TxCalGainGet(TI_HANDLE  hAdapter, PltGainGet_t* pPLTGainGet)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->PLT_TxCalGainGet(pPLTGainGet);
}

tiINT32
TI_PLT_TxCalGainAdjust(TI_HANDLE  hAdapter, tiINT32   uTxGainChange)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->PLT_TxCalGainAdjust(uTxGainChange);
}

tiINT32		
TI_PLT_TxCalStart(TI_HANDLE  hAdapter, PltTxCalibrationRequest_t* pPLTTxCal)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->PLT_TxCalStart(pPLTTxCal);
}

tiINT32		
TI_PLT_TxCalStop(TI_HANDLE  hAdapter)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->PLT_TxCalStop();
}

tiINT32		
TI_PLT_RxTxCalNVSUpdateBuffer(TI_HANDLE  hAdapter, PltNvsResultsBuffer_t* pPLT_NVSUpdateBuffer)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->PLT_RxTxCalNVSUpdateBuffer(pPLT_NVSUpdateBuffer);
}

tiINT32		
TI_PLT_RxCal(TI_HANDLE  hAdapter, PltRxCalibrationRequest_t* pPltRxCalibration)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->PLT_RxCal(pPltRxCalibration);
}

tiINT32		
TI_PLT_RadioTune(TI_HANDLE  hAdapter, TestCmdChannelBand_t* pChannelBand)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->PLT_RadioTune(pChannelBand);
}

/********************************************************************/
#ifdef _WINDOWS
#endif

/********************************************************************/

#ifdef TI_DBG

#ifdef DRIVER_PROFILING

tiINT32     
TI_ProfileReport(TI_HANDLE  hAdapter)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->ProfileReport();
}

tiINT32                 
TI_CpuEstimatorCommand(TI_HANDLE  hAdapter, tiUINT8 uType, tiUINT32 uData)
{
    if ( !TI_CheckAdapterObject(hAdapter) )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->CpuEstimatorCommand(uType, uData);
}

#endif

#endif // TI_DBG

/********************************************************************/
tiINT32     
TI_Send_EAPOL_Packet( TI_HANDLE  hAdapter, tiVOID* pData, tiUINT32 uSize )
{
    if ( !TI_CheckAdapterObject(hAdapter) || !pData )
        return TI_RESULT_FAILED;
    return ((TI_WLAN_AdapterAPI *) hAdapter)->Send_EAPOL_Packet(pData,uSize);
}
/********************************************************************/
