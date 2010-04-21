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

#undef BINARY_COMPATIBLE
#define BINARY_COMPATIBLE 0

#ifdef _WINDOWS
#endif

#include "osAdapter.h"
#include "tiioctl.h"
#include "paramOut.h"
#include "osUtil.h"
#include "srcApi.h"
#include "configMgr.h"
#include "TNETW_Driver_api.h"

#ifdef GWSI_DRIVER
#include "gwsi_tester_parser.h"
#endif

#include "IPCKernelApi.h"

#ifdef TI_DBG
#ifndef TIWLAN_MSM7000
#include "debug.h"
#endif
#endif

TI_STATUS report_setParam(TI_HANDLE hReport, whalParamInfo_t *pParam );
TI_STATUS report_getParam(TI_HANDLE hReport, whalParamInfo_t *pParam );

NTSTATUS
DispatchCommand(
    PTIWLN_ADAPTER_T pAdapter,
    ULONG ioControlCode,
    PULONG outBufLen,
    ULONG inBufLen,
    PVOID ioBuffer,
    PUINT8 pIoCompleteFlag
    )
{
    NTSTATUS    ntStatus = STATUS_SUCCESS;

#ifdef TI_DBG
    whalParamInfo_t     param;
#endif

    /*BOOL    builtInStatus;*/
/**/
/*     Block entrance from utility to the driver when the recovery is in progress */
/*    UtilDriverBuiltInTestStsGet(pAdapter, &builtInStatus, sizeof(builtInStatus));*/
/*    if(builtInStatus)*/
/*        return STATUS_INVALID_PARAMETER;*/

    // os_protectLock(pAdapter, pAdapter->SystemProtect); /* Bug. Replaced with the line below -- MCS00035801 */
    /* Dm: os_protectLock(( (configMgr_t  *)(pAdapter->CoreHalCtx) )->hOs, pAdapter->SystemProtect); */


    /* Initialiaze to TRUE all IOCTL's : Only in few cases there will be IOCTL that will
    be implemented ar REALread from FW and then will have their completion callback later called */
    *pIoCompleteFlag = TRUE;

    switch(ioControlCode) {
    case TIWLN_802_11_ENABLE_EVENT:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_ENABLE_EVENT.\n");
        ntStatus = configMgr_RegisterEvent(pAdapter->CoreHalCtx, ioBuffer, inBufLen);
    break;

    case TIWLN_802_11_DISABLE_EVENT:
 
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_DISABLE_EVENT.\n");
#ifdef _WINDOWS
#else
        ntStatus = configMgr_UnRegisterEvent(pAdapter->CoreHalCtx, *(TI_HANDLE*)ioBuffer);
#endif
    break;

#ifdef _WINDOWS
#endif
    case TIWLN_GET_SW_VERSION:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_GET_SW_VERSION.\n");
        ntStatus = UtilGetSwVersion(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_BSSID_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_BSSID_GET.\n");
        ntStatus = UtilGetBSSID(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_BSSID_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_BSSID_SET.\n");
        ntStatus = UtilSetBSSID(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_SSID_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_SSID_GET.\n");
        ntStatus = UtilGetSSID(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_SSID_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_SSID_SET.\n");
        ntStatus = UtilSetSSID(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_DISASSOCIATE:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_DISASSOCIATE.\n");
        ntStatus = UtilDisassociate(pAdapter, ioBuffer, inBufLen);
       break;

    case TIWLN_802_11_NETWORK_TYPES_SUPPORTED:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_NETWORK_TYPES_SUPPORTED.\n");
        ntStatus = UtilNetworkTypesSupported(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_NETWORK_TYPE_IN_USE_GET:
        PRINT(DBG_IOCTL, "...IOCTL TIWLN_802_11_NETWORK_TYPE_IN_USE_GET.\n");
        ntStatus = UtilNetworkTypeInUseGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_NETWORK_TYPE_IN_USE_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_NETWORK_TYPE_IN_USE_SET.\n");
        break;

    case TIWLN_802_11_POWER_MODE_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_POWER_MODE_SET.\n");
        ntStatus = UtilPowerModeSet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_POWER_MODE_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_POWER_MODE_GET.\n");
        ntStatus = UtilPowerModeGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_POWER_LEVEL_PS_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_POWER_LEVEL_PS_GET.\n");
            ntStatus = UtilPowerLevelPSGet(pAdapter, ioBuffer, outBufLen);
            break;

    case TIWLN_802_11_POWER_LEVEL_PS_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_POWER_LEVEL_PS_SET.\n");
            ntStatus = UtilPowerLevelPSSet(pAdapter, ioBuffer, outBufLen);
            break;

    case TIWLN_802_11_POWER_LEVEL_DEFAULT_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_POWER_LEVEL_DEFAULT_GET.\n");
            ntStatus = UtilPowerLevelDefaultGet(pAdapter, ioBuffer, outBufLen);
            break;

    case TIWLN_802_11_POWER_LEVEL_DEFAULT_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_POWER_LEVEL_DEFAULT_SET.\n");
            ntStatus = UtilPowerLevelDefaultSet(pAdapter, ioBuffer, outBufLen);
            break;

    case TIWLN_802_11_POWER_LEVEL_DOZE_MODE_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_POWER_LEVEL_DOZE_MODE_GET.\n");
            ntStatus = UtilPowerLevelDozeModeGet(pAdapter, ioBuffer, outBufLen);
            break;

    case TIWLN_802_11_POWER_LEVEL_DOZE_MODE_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_POWER_LEVEL_DOZE_MODE_SET.\n");
            ntStatus = UtilPowerLevelDozeModeSet(pAdapter, ioBuffer, outBufLen);
            break;

	case TIWLN_802_11_BEACON_FILTER_DESIRED_STATE_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_BEACON_FILTER_DESIRED_STATE_SET.\n");
            ntStatus = UtilBeaconFilterDesiredStateSet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_BEACON_FILTER_DESIRED_STATE_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_BEACON_FILTER_DESIRED_STATE_GET.\n");
            ntStatus = UtilBeaconFilterDesiredStateGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_SET_TRAFFIC_INTENSITY_THRESHOLDS:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_SET_TRAFFIC_INTENSITY_THRESHOLDS.\n");
       ntStatus = UtilSetTrafficIntensityThresholds(pAdapter, ioBuffer, inBufLen);
       break;
    case TIWLN_802_11_GET_TRAFFIC_INTENSITY_THRESHOLDS:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_GET_TRAFFIC_INTENSITY_THRESHOLDS.\n");
       ntStatus = UtilGetTrafficIntensityThresholds(pAdapter, ioBuffer, outBufLen);
       break;

    case TIWLN_802_11_TOGGLE_TRAFFIC_INTENSITY_EVENTS:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_TOGGLE_TRAFFIC_INTENSITY_EVENTS.\n");
       ntStatus = UtilToggleTrafficIntensityEvents(pAdapter, ioBuffer, inBufLen);
       break;

    /* TBD: CAN BE REMOVED ??? */
    case TIWLN_802_11_POWER_MGR_PROFILE:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_POWER_MGR_PROFILE.\n");
        ntStatus = UtilPowerModeSet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_TX_POWER_LEVEL_GET:
        PRINT(DBG_IOCTL, "...IOCTL TIWLN_802_11_TX_POWER_LEVEL_GET.\n");
        ntStatus = UtilGetTxPowerLevel (pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_TX_POWER_DBM_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_TX_POWER_DBM_GET.\n");
        ntStatus = UtilTxPowerLevelDbmGet (pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_TX_POWER_DBM_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_TX_POWER_DBM_SET.\n");
        ntStatus = UtilSetTxPowerDbm (pAdapter, ioBuffer, inBufLen);
        break;

    /*************************************************************************
    ****  RSSI :Real Read from FW asynchronous then init the flag to False   ***
    ***** The completion will happen when the Completion                     ***
    ***** Callback IoctlCompleteCB function is called                        ***
    *************************************************************************/
    case TIWLN_802_11_RSSI:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_RSSI.\n");
        *pIoCompleteFlag = FALSE;
        ntStatus = UtilGetAsyncCurrentRssiLevel (pAdapter, ioBuffer, outBufLen);

        /* ntStatus = UtilGetCurrentRssiLevel (pAdapter, ioBuffer, outBufLen);*/
        break;

    case TIWLN_802_11_SNR:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_SNR.\n");
        *pIoCompleteFlag = FALSE;
        ntStatus = UtilGetAsyncCurrentSnrRatio (pAdapter, ioBuffer, outBufLen);
        break;

        /* TBD: REMOVE ???? */
    case TIWLN_802_11_RSSI_TRIGGER_GET:
        PRINT(DBG_IOCTL, "...IOCTL TIWLN_802_11_RSSI_TRIGGER_GET.\n");
        *outBufLen = 4;
        break;

        /* TBD: REMOVE ???? */
    case TIWLN_802_11_RSSI_TRIGGER_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_RSSI_TRIGGER_SET.\n");
        *outBufLen = 4;
        break;

    case TIWLN_802_11_BSSID_LIST:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_BSSID_LIST.\n");
        ntStatus = UtilBssidListGet(pAdapter, ioBuffer, outBufLen, TRUE, FALSE);
        break;

    case TIWLN_802_11_FULL_BSSID_LIST:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_FULL_BSSID_LIST.\n");
        ntStatus = UtilBssidListGet(pAdapter, ioBuffer, outBufLen, TRUE, TRUE);
        break;

    case TIWLN_802_11_INFRASTRUCTURE_MODE_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_INFRASTRUCTURE_MODE_GET.\n");
        ntStatus = UtilInfrastructureModeGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_INFRASTRUCTURE_MODE_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_INFRASTRUCTURE_MODE_SET.\n");
        ntStatus = UtilInfrastructureModeSet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_FRAGMENTATION_THRESHOLD_GET:
        PRINT(DBG_IOCTL, "...IOCTL TIWLN_802_11_FRAGMENTATION_THRESHOLD_GET.\n");
        ntStatus = UtilFragmentationThresholdGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_FRAGMENTATION_THRESHOLD_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_FRAGMENTATION_THRESHOLD_SET.\n");
        ntStatus = UtilFragmentationThresholdSet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_RTS_THRESHOLD_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_FRAGMENTATION_THRESHOLD_GET.\n");
        ntStatus = UtilRtsThresholdGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_RTS_THRESHOLD_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_FRAGMENTATION_THRESHOLD_SET.\n");
        ntStatus = UtilRtsThresholdSet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_SUPPORTED_RATES:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_SUPPORTED_RATES.\n");
        ntStatus = UtilSupportedRates(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_SUPPORTED_RATES_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_SUPPORTED_RATES_SET.\n");
        ntStatus = UtilSupportedRatesSet(pAdapter, ioBuffer, inBufLen);
        break;
    
    case TIWLN_802_11_DESIRED_RATES_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_DESIRED_RATES_GET.\n");
        ntStatus = UtilDesiredRatesGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_CURRENT_RATES_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_DESIRED_RATES_GET.\n");
        ntStatus = UtilCurrentRatesGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_STATISTICS:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_STATISTICS.\n");
        ntStatus = UtilStatistics(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_TX_STATISTICS:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_TX_STATISTICS.\n");
        ntStatus = UtilTxStatistics(pAdapter, ioBuffer, inBufLen, outBufLen);
        break;

    case TIWLN_802_11_ADD_WEP:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_ADD_WEP.\n");
        ntStatus = UtilAddWep(pAdapter, ioBuffer, inBufLen, TRUE);
        break;

    case TIWLN_802_11_REMOVE_WEP:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_REMOVE_WEP.\n");
        ntStatus = UtilRemoveWep(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_WEP_STATUS_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_WEP_STATUS_GET.\n");
        ntStatus = UtilWepStatusGet(pAdapter, ioBuffer, outBufLen);

            switch (*(cipherSuite_e*)ioBuffer)
            {
            case RSN_CIPHER_NONE:
                *(NDIS_802_11_WEP_STATUS*)ioBuffer = (NDIS_802_11_WEP_STATUS)os802_11WEPDisabled;
                break;
            case RSN_CIPHER_WEP: 
                *(NDIS_802_11_WEP_STATUS*)ioBuffer = (NDIS_802_11_WEP_STATUS)os802_11WEPEnabled;
                break; 
            case RSN_CIPHER_TKIP:
                *(NDIS_802_11_WEP_STATUS*)ioBuffer = (NDIS_802_11_WEP_STATUS)os802_11Encryption2Enabled;
                break;
            case RSN_CIPHER_AES_CCMP:
                *(NDIS_802_11_WEP_STATUS*)ioBuffer = (NDIS_802_11_WEP_STATUS)os802_11Encryption3Enabled;
                break;
            default: 
                break;
            }

        break;

    case TIWLN_802_11_WEP_STATUS_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_WEP_STATUS_SET.\n");

        /* Convert from cipher suite to encryption status */

        switch (*(NDIS_802_11_WEP_STATUS*)ioBuffer)
        {
        case os802_11WEPDisabled:
            *(cipherSuite_e*)ioBuffer = RSN_CIPHER_NONE;
            break;
        case os802_11WEPEnabled: 
            *(cipherSuite_e*)ioBuffer = RSN_CIPHER_WEP;
            break; 
        case os802_11Encryption2Enabled:
            *(cipherSuite_e*)ioBuffer = RSN_CIPHER_TKIP;
            break;
        case os802_11Encryption3Enabled:
            *(cipherSuite_e*)ioBuffer = RSN_CIPHER_AES_CCMP;
            break;
        default: 
            break;
        }

        ntStatus = UtilWepStatusSet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_EAP_TYPE_GET:
        PRINT(DBG_NDIS_OIDS_LOUD, "TIWL: TIWLN_802_11_EAP_TYPE_GET\n");
        ntStatus = Util802EapTypeGet(pAdapter, ioBuffer, outBufLen);
        break;
        
    case TIWLN_802_11_EAP_TYPE_DRIVER_SET:
        PRINT(DBG_NDIS_OIDS_LOUD, "TIWL: TIWLN_802_11_EAP_TYPE_DRIVER_SET\n");
        ntStatus = Util802EapTypeSet(pAdapter, ioBuffer, inBufLen);
        break;

        /* WPA2 start */

    case TIWLN_802_11_PMKID_GET:
        PRINT(DBG_NDIS_OIDS_LOUD, "TIWL: OID_802_11_PMKID GET\n");
        ntStatus = Util802PmkidGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_PMKID_SET:
        PRINT(DBG_NDIS_OIDS_LOUD, "TIWL: OID_802_11_PMKID SET\n");
        ntStatus = Util802PmkidSet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_CAPABILITY_GET:
        PRINT(DBG_NDIS_OIDS_LOUD, "TIWL: TIWLN_802_11_CAPABILITY_GET \n");
        ntStatus = Util802CapabilityGet(pAdapter, ioBuffer, outBufLen);
        break;
        
    case  TIWLN_802_11_AVAILABLE_OPTIONS_GET:
        PRINT(DBG_NDIS_OIDS_LOUD, "TIWL: TIWLN_802_11_AVAILABLE_OPTIONS_GET \n");
        ntStatus = Util802FSWAvailableOptionsGet(pAdapter, ioBuffer, outBufLen);
        break;

    case  TIWLN_802_11_WPA_OPTIONS_GET:
        PRINT(DBG_NDIS_OIDS_LOUD, "TIWL: TIWLN_802_11_WPA_OPTIONS_GET\n");
        ntStatus = Util802FSWOptionsGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_WPA_OPTIONS_SET:
        PRINT(DBG_NDIS_OIDS_LOUD, "TIWL: TIWLN_802_11_WPA_OPTIONS_SET\n");
        ntStatus = Util802FSWOptionsSet(pAdapter, ioBuffer, inBufLen);
        break;

        /* wpa2 - end */

    case TIWLN_802_11_AUTHENTICATION_MODE_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_AUTHENTICATION_MODE_GET.\n");
        ntStatus = UtilExtAuthenticationModeGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_AUTHENTICATION_MODE_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_AUTHENTICATION_MODE_SET.\n");
        ntStatus = UtilExtAuthenticationModeSet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_NUMBER_OF_ANTENNAS:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_NUMBER_OF_ANTENNAS.\n");
        ntStatus = UtilNumberOfAntennas(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_RX_ANTENNA_SELECTED_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_RX_ANTENNA_SELECTED_GET.\n");
        ntStatus = UtilRxAntennaGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_RX_ANTENNA_SELECTED_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_RX_ANTENNA_SELECTED_SET.\n");
        ntStatus = UtilRxAntennaSet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_TX_ANTENNA_SELECTED_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_TX_ANTENNA_SELECTED_GET.\n");
        ntStatus = UtilTxAntennaGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_TX_ANTENNA_SELECTED_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_TX_ANTENNA_SELECTED_SET.\n");
        ntStatus = UtilTxAntennaSet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLAN_802_11_ANTENNA_DIVERSITY_PARAM_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLAN_802_11_ANTENNA_DIVERSITY_PARAM_SET.\n");
        ntStatus = UtilAntennaDivresitySet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_DESIRED_CHANNEL_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_DESIRED_CHANNEL_GET.\n");
        ntStatus = UtilDesiredChannelGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_DESIRED_CHANNEL_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_DESIRED_CHANNEL_SET.\n");
        ntStatus = UtilDesiredChannelSet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_SHORT_PREAMBLE_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_SHORT_PREAMBLE_GET.\n");
        ntStatus = UtilShortPreambleGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_SHORT_PREAMBLE_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_SHORT_PREAMBLE_SET.\n");
        ntStatus = UtilShortPreambleSet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_CURRENT_REGDOMAIN_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_CURRENT_REGDOMAIN_GET.\n");
        ntStatus = UtilCurrentRegDomainGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_REG_DOMAIN_ENABLE_DISABLE_802_11D:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_REG_DOMAIN_ENABLE_DISABLE_802_11D.\n");
        ntStatus = UtilRegulatoryDomain_enableDisable_802_11d(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_REG_DOMAIN_ENABLE_DISABLE_802_11H:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_REG_DOMAIN_ENABLE_DISABLE_802_11H.\n");
        ntStatus = UtilRegulatoryDomain_enableDisable_802_11h(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_REG_DOMAIN_GET_802_11D:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_REG_DOMAIN_GET_802_11D.\n");
        ntStatus = UtilRegulatoryDomain_Get_802_11d(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_REG_DOMAIN_GET_802_11H:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_REG_DOMAIN_GET_802_11D.\n");
        ntStatus = UtilRegulatoryDomain_Get_802_11h(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_REG_DOMAIN_SET_COUNTRY_2_4:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_REG_DOMAIN_SET_COUNTRY_2_4.\n");
        ntStatus = UtilRegulatoryDomain_setCountryIE_2_4(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_REG_DOMAIN_GET_COUNTRY_2_4:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_REG_DOMAIN_GET_COUNTRY_2_4.\n");
        ntStatus = UtilRegulatoryDomain_getCountryIE_2_4(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_REG_DOMAIN_SET_COUNTRY_5:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_REG_DOMAIN_SET_COUNTRY_5.\n");
        ntStatus = UtilRegulatoryDomain_setCountryIE_5(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_REG_DOMAIN_GET_COUNTRY_5:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_REG_DOMAIN_GET_COUNTRY_5.\n");
        ntStatus = UtilRegulatoryDomain_getCountryIE_5(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_REG_DOMAIN_SET_DFS_RANGE:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_REG_DOMAIN_SET_DFS_RANGE.\n");
        ntStatus = UtilRegulatoryDomain_setMinMaxDfsChannels(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_REG_DOMAIN_GET_DFS_RANGE:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_REG_DOMAIN_GET_DFS_RANGE.\n");
        ntStatus = UtilRegulatoryDomain_getMinMaxDfsChannels(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_SHORT_RETRY_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_SHORT_RETRY_GET.\n");
        ntStatus = UtilShortRetryGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_SHORT_RETRY_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_SHORT_RETRY_SET.\n");
        ntStatus = UtilShortRetrySet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_LONG_RETRY_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_LONG_RETRY_GET.\n");
        ntStatus = UtilLongRetryGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_LONG_RETRY_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_LONG_RETRY_SET.\n");
        ntStatus = UtilLongRetrySet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_4XACTIVESTATE_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_4XACTIVESTATE_GET.\n");
        ntStatus = Util4xActiveStateGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_DESIRED_SSID_GET:
            PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_DESIRED_SSID_GET.\n");
        ntStatus = UtilGetDesiredSSID(pAdapter, ioBuffer, outBufLen);
        break;
    
    case TIWLN_802_11_CHANNEL_GET:
            PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_CHANNEL_GET.\n");
        ntStatus = UtilChannelGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_DESIRED_INFRASTRUCTURE_MODE:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_DESIRED_INFRASTRUCTURE_MODE.\n");
        ntStatus = UtilDesiredInfrastructureModeGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_DRIVER_STATUS_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_DRIVER_STATUS_GET.\n");
        ntStatus = UtilDriverStatusGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_DRIVER_STATUS_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_DRIVER_STATUS_SET.\n");
        ntStatus = UtilDriverStatusSet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_DRIVER_SUSPEND:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_DRIVER_STATUS_SET.\n");
            UtilDriverSuspend(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_IBSS_PROTECTION_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_IBSS_PROTECTION_GET.\n");
        ntStatus = UtilIbssProtectionGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_IBSS_PROTECTION_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_IBSS_PROTECTION_GET.\n");
        ntStatus = UtilIbssProtectionSet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_SHORT_SLOT_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_SHORT_SLOT_GET.\n");
        ntStatus = UtilShortSlotGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_SHORT_SLOT_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_SHORT_SLOT_SET.\n");
        ntStatus = UtilShortSlotSet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_EXT_RATES_IE_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_EXT_RATES_IE_GET.\n");
        ntStatus = UtilExtRatesIeGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_EXT_RATES_IE_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_EXT_RATES_IE_SET.\n");
        ntStatus = UtilExtRatesIeSet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_REMOVE_KEY:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_REMOVE_KEY\n");
        ntStatus = UtilRemoveKey(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_ADD_KEY:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_ADD_KEY\n");
        ntStatus = UtilAddKey(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_MIXED_MODE_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_MIXED_MODE_SET\n");
        ntStatus = UtilSetMixedMode(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_MIXED_MODE_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_MIXED_MODE_GET\n");
        ntStatus = UtilGetMixedMode(pAdapter, RSN_MIXED_MODE, ioBuffer, outBufLen);

        break;
#ifdef EXC_MODULE_INCLUDED
    case TIWLN_802_11_EXC_CONFIGURATION_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_EXC_CONFIGURATION_SET\n");
        ntStatus = UtilExcConfigurationSet(pAdapter, ioBuffer, inBufLen);
        break;
           
    case TIWLN_802_11_EXC_CONFIGURATION_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_EXC_CONFIGURATION_GET\n");
        ntStatus = UtilExcConfigurationGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_EXC_ROGUE_AP_DETECTED:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_EXC_ROGUE_AP_DETECTED\n");
        ntStatus = UtilExcRogueApDetectedSet(pAdapter, ioBuffer, inBufLen);
        break;

    case    TIWLN_802_11_EXC_REPORT_ROGUE_APS:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_EXC_REPORT_ROGUE_APS\n");
        ntStatus = UtilExcReportRogueApSet(pAdapter, ioBuffer, inBufLen);
        break;

    case    TIWLN_802_11_EXC_CCKM_REQUEST:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_EXC_CCKM_REQUEST\n");
        ntStatus = UtilExcCckmRequestSet(pAdapter, ioBuffer, inBufLen);
        break;

    case    TIWLN_802_11_EXC_CCKM_RESULT:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_EXC_CCKM_RESULT\n");
        ntStatus = UtilExcCckmResultSet(pAdapter, ioBuffer, inBufLen);
        break;

    case    TIWLN_802_11_EXC_NETWORK_EAP_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_EXC_NETWORK_EAP_SET\n");
        ntStatus = UtilExcNetworkEapSet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_EXC_NETWORK_EAP_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_EXC_NETWORK_EAP_GET\n");
        ntStatus = UtilExcNetworkEapGet(pAdapter, ioBuffer, outBufLen);
        break;
    case TIWLN_802_11_EXC_AUTH_SUCCESS:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_EXC_AUTH_SUCCESS\n");
    
        *outBufLen = UtilExcAuthSuccessSet(pAdapter, ioBuffer, inBufLen);

        break;
/******************** measurement *********************/

    case TIWLN_802_11_MEASUREMENT_ENABLE_DISABLE_PARAMS_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_MEASUREMENT_MAX_DURATION_PARAMS_SET.\n");
        ntStatus = UtilMeasurementEnableDisableParamsSet(pAdapter, ioBuffer, inBufLen);
         break;

    case TIWLN_802_11_MEASUREMENT_MAX_DURATION_PARAMS_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_MEASUREMENT_MAX_DURATION_PARAMS_SET.\n");
        ntStatus = UtilMeasurementMaxDurationParamsSet(pAdapter, ioBuffer, inBufLen);
         break;

#else
         
    case TIWLN_802_11_EXC_CONFIGURATION_SET:
    case TIWLN_802_11_EXC_CONFIGURATION_GET:
    case TIWLN_802_11_EXC_ROGUE_AP_DETECTED:
    case TIWLN_802_11_EXC_REPORT_ROGUE_APS:
    case TIWLN_802_11_EXC_CCKM_REQUEST:
    case TIWLN_802_11_EXC_CCKM_RESULT:
    case TIWLN_802_11_EXC_NETWORK_EAP_SET:
    case TIWLN_802_11_EXC_NETWORK_EAP_GET:
    case TIWLN_802_11_EXC_AUTH_SUCCESS:
    case TIWLN_802_11_MEASUREMENT_ENABLE_DISABLE_PARAMS_SET:
    case TIWLN_802_11_MEASUREMENT_MAX_DURATION_PARAMS_SET:
    PRINT(DBG_IOCTL_LOUD, "...EXC IOCTL NOT Supported.\n");   
    break;

#endif /* EXC_MODULE_INCLUDED */

/******************** QOS **********************/

    case TIWLN_802_11_SET_QOS_PARAMS:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_SET_QOS_PARAMS.\n");
       ntStatus = UtilQosSetParams(pAdapter, ioBuffer, inBufLen);
       break;

    case TIWLN_802_11_CONFIG_TX_CLASS:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_CONFIG_TX_CLASS.\n");
       ntStatus = UtilConfigTxClassifier(pAdapter,ioBuffer,inBufLen);
       break;

    case TIWLN_802_11_REMOVE_CLSFR_ENTRY:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_REMOVE_CLSFR_ENTRY.\n");
       ntStatus = UtilRemoveClassifierEntry(pAdapter,ioBuffer,inBufLen);
       break;
 
    case TIWLN_802_11_GET_CLSFR_TYPE:
         PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_GET_CLSFR_TYPE.\n");
       ntStatus = UtilGetClsfrType(pAdapter, ioBuffer, outBufLen);
       break;
 
    case TIWLN_802_11_GET_AP_QOS_PARAMS:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_GET_AP_QOS_PARAMS.\n");
       ntStatus = UtilGetAPQosParams(pAdapter, ioBuffer, outBufLen);
       break;

    case TIWLN_802_11_GET_AP_QOS_CAPABILITIES:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_GET_AP_QOS_CAPABILITIES.\n");
       ntStatus = UtilGetAPQosCapabilities(pAdapter, ioBuffer, outBufLen);
       break;
 
    case TIWLN_802_11_ADD_TSPEC:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_ADD_TSPEC.\n");
       ntStatus = UtilAddTspec(pAdapter, ioBuffer, inBufLen);
       break;

    case TIWLN_802_11_GET_TSPEC_PARAMS:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_GET_TSPEC_PARAMS.\n");
       ntStatus = UtilGetTspecParams(pAdapter, ioBuffer, outBufLen);
       break;

    case TIWLN_802_11_DELETE_TSPEC:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_DELETE_TSPEC.\n");
       ntStatus = UtilDeleteTspec(pAdapter, ioBuffer, inBufLen);
       break;
 
    case TIWLN_802_11_GET_CURRENT_AC_STATUS:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_GET_CURRENT_AC_STATUS.\n");
       ntStatus = UtilGetCurrentAcStatus(pAdapter, ioBuffer, outBufLen);
       break;
 
    case TIWLN_802_11_SET_MEDIUM_USAGE_THRESHOLD:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_SET_MEDIUM_USAGE_THRESHOLD.\n");
       ntStatus = UtilSetMediumUsageThreshold(pAdapter, ioBuffer, inBufLen);
       break;
 
    case TIWLN_802_11_SET_PHY_RATE_THRESHOLD:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_SET_PHY_RATE_THRESHOLD.\n");
       ntStatus = UtilSetPhyRateThreshold(pAdapter, ioBuffer, inBufLen);
       break;
 
    case TIWLN_802_11_GET_MEDIUM_USAGE_THRESHOLD:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_GET_MEDIUM_USAGE_THRESHOLD.\n");
       ntStatus = UtilGetMediumUsageThreshold(pAdapter, ioBuffer, outBufLen);
       break;
 
    case TIWLN_802_11_GET_PHY_RATE_THRESHOLD:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_GET_PHY_RATE_THRESHOLD.\n");
       ntStatus = UtilGetPhyRateThreshold(pAdapter, ioBuffer, outBufLen);
       break;

    case TIWLN_802_11_GET_USER_PRIORITY_OF_STREAM:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_GET_USER_PRIORITY_OF_STREAM.\n");
       ntStatus = UtilGetUserPriorityOfStream(pAdapter, ioBuffer, outBufLen);
       break;

    case TIWLN_802_11_GET_DESIRED_PS_MODE:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_GET_DESIRED_PS_MODE.\n");
       ntStatus = UtilGetDesiredPsMode(pAdapter, ioBuffer, outBufLen);
       break;

    case TIWLN_802_11_POLL_AP_PACKETS:
      PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_POLL_AP_PACKETS.\n");
       ntStatus = UtilPollApPackets(pAdapter, ioBuffer, inBufLen);
       break;

    case TIWLN_802_11_POLL_AP_PACKETS_FROM_AC:
      PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_POLL_AP_PACKETS_FROM_AC.\n");
       ntStatus = UtilPollApPacketsFromAC(pAdapter, ioBuffer, inBufLen);
       break;

    case TIWLN_802_11_CONFIG_EVENTS_RSSI:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_CONFIG_EVENTS.\n");
       ntStatus = UtilConfigRSSI(pAdapter, (UINT32) ioBuffer, inBufLen);
       break;

    case TIWLN_802_11_GET_DRIVERS_CAPABILITIES:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_GET_DRIVERS_CAPABILITIES.\n");
       ntStatus = UtilGetDrvCapabilities(pAdapter, ioBuffer, outBufLen);
       break;

    case TIWLN_802_11_GET_SELECTED_BSSID_INFO:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_GET_SELECTED_BSSID_INFO.\n");
       ntStatus = UtilGetSelectedBSSIDInfo(pAdapter, ioBuffer, outBufLen);
       break;

    case TIWLN_802_11_GET_PRIMARY_BSSID_INFO:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_GET_PRIMARY_BSSID_INFO.\n");
       ntStatus = UtilGetPrimaryBSSIDInfo(pAdapter, ioBuffer, outBufLen);
       break;

    case TIWLN_802_11_GET_DRIVER_STATE:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_GET_DRIVER_STATE.\n");
       ntStatus = UtilGetDriverState(pAdapter, ioBuffer, outBufLen);
       break;

    case TIWLN_802_11_SET_RX_TIMEOUT:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_SET_RX_TIMEOUT.\n");
       ntStatus = UtilQosSetRxTimeOut(pAdapter, ioBuffer, inBufLen);
       break;

    case TIWLN_802_11_SET_DTAG_TO_AC_MAPPING_TABLE:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_SET_DTAG_TO_AC_MAPPING_TABLE.\n");
       ntStatus = UtilSetDTagToAcMappingTable(pAdapter, ioBuffer, inBufLen);
       break;
       
    case TIWLN_802_11_SET_VAD:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_SET_VAD.\n");
       ntStatus = UtilSetVAD(pAdapter, ioBuffer, inBufLen);
       break;
 
    case TIWLN_802_11_GET_VAD:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_GET_VAD.\n");
       ntStatus = UtilGetVAD(pAdapter, ioBuffer, outBufLen);
       break;
 
/******************** scan *********************/

    case TIWLN_802_11_START_APP_SCAN_SET:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_START_APP_SCAN_SET.\n");
       ntStatus = UtilStartAppScanSet(pAdapter, ioBuffer, inBufLen);
        break;
    
    case TIWLN_802_11_STOP_APP_SCAN_SET:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_STOP_APP_SCAN_SET.\n");
       ntStatus = UtilStopAppScanSet(pAdapter, ioBuffer, inBufLen);
        break;
    
    case TIWLN_802_11_SCAN_POLICY_PARAM_SET:
       PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_SCAN_POLICY_PARAM_SET.\n");
       ntStatus = UtilScanPolicyParamSet(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_11_SCAN_BSS_LIST_GET:
        PRINT(DBG_IOCTL_LOUD, "...TIWLN_802_11_SCAN_BSS_LIST_GET.\n");
       ntStatus = UtilScanBssListGet(pAdapter, ioBuffer, outBufLen);
        break;

/******************** Roaming *********************/

    case TIWLN_802_11_ROAMING_CONFIG_PARAMS_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_ROAMING_CONFIG_PARAMS_SET.\n");
        ntStatus = UtilConfigRoamingParamsSet(pAdapter, ioBuffer, inBufLen);
         break;

    case TIWLN_802_11_ROAMING_CONFIG_PARAMS_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_ROAMING_CONFIG_PARAMS_GET.\n");
        ntStatus = UtilConfigRoamingParamsGet(pAdapter, ioBuffer, outBufLen);
         break;

/******************** misc *********************/

    case TIWLN_HW_READ_REGISTER:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_HW_READ_REGISTER.\n");
        ntStatus = UtilReadReg(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_HW_WRITE_REGISTER:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_HW_WRITE_REGISTER.\n");
        ntStatus = UtilWriteReg(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_802_3_CURRENT_ADDRESS:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_3_CURRENT_ADDRESS.\n");
        ntStatus = UtilGetMACAddress(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_802_11_ASSOCIATION_INFORMATION:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_802_11_ASSOCIATION_INFORMATION.\n");
        ntStatus = UtilAssociationInfoGet(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_IOCTL_OID_QUERY_INFORMATION:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_IOCTL_OID_QUERY_INFORMATION.\n");
        ntStatus = UtilInfoCodeQueryInformation(pAdapter, (PUCHAR)ioBuffer, outBufLen);
        break;

    case TIWLN_IOCTL_OID_SET_INFORMATION:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_IOCTL_OID_SET_INFORMATION.\n");
        ntStatus = UtilInfoCodeSetInformation(pAdapter, (PUCHAR)ioBuffer, inBufLen);
        break;

    case GWSI_GET_INIT_TABLE_COMMAND:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL GWSI_GET_INIT_TABLE_COMMAND.\n");
        configMgr_GetInitParams (pAdapter->CoreHalCtx, (UINT8 *) ioBuffer, (UINT16 *) outBufLen);
        break;

    case BT_COEXSISTANCE_SET_ENABLE:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL BT_COEXSISTANCE_SET_ENABLE.\n");
        ntStatus = UtilBthWlanCoeEnable(pAdapter, (PUCHAR)ioBuffer, inBufLen);
        break;

    case BT_COEXSISTANCE_SET_RATE:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL BT_COEXSISTANCE_SET_RATE.\n");
        ntStatus = UtilBthWlanCoeRate(pAdapter, (PUCHAR)ioBuffer, inBufLen);
        break;

    case BT_COEXSISTANCE_SET_CONFIG:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL BT_COEXSISTANCE_SET_CONFIG.\n");
        ntStatus = UtilBthWlanCoeConfig(pAdapter, (PUCHAR)ioBuffer, inBufLen);
        break;

    case BT_COEXSISTANCE_GET_STATUS:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL BT_COEXSISTANCE_GET_STATUS.\n");
        ntStatus = UtilBthWlanCoeGetStatus(pAdapter, (PUCHAR)ioBuffer, outBufLen);
        break;

    case TIWLN_EARLY_WAKEUP_IE_SET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_EARLY_WAKEUP_IE_SET.\n");
        ntStatus = UtilEarlyWakeupIeSet(pAdapter, (PUCHAR)ioBuffer, inBufLen);
        break;

    case TIWLN_EARLY_WAKEUP_IE_GET:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_EARLY_WAKEUP_IE_GET.\n");
        ntStatus = UtilEarlyWakeupIeGet(pAdapter, (PUCHAR)ioBuffer, outBufLen);
        break;

#ifdef TI_DBG

    case TIWLN_GET_DEBUG_FLAG:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_GET_DEBUG_FLAG.\n");
        *(PULONG)ioBuffer = TiDebugFlag;
        break;

    case TIWLN_SET_DEBUG_FLAG:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_SET_DEBUG_FLAG - %#lX.\n", *(PULONG)ioBuffer));
        TiDebugFlag = *(PULONG)ioBuffer;
        break;

    case TIWLN_DISPLAY_STATS:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_DISPLAY_STATS. Code:%lu  Data:%lu\n", *(PULONG)ioBuffer, *(PULONG)((PUCHAR)ioBuffer+4)));
        debugFunction(pAdapter->CoreHalCtx, *(PULONG)ioBuffer, ((PUCHAR)ioBuffer+4));
        break;

    case TIWLN_REPORT_MODULE_GET:
    case TIWLN_GET_MODULE:
        PRINT(DBG_IOCTL_LOUD, "...TIWLN_GET_MODULE\n");
        param.paramType = REPORT_MODULE_TABLE_PARAM;
        report_getParam(((configMgr_t *)pAdapter->CoreHalCtx)->hReport, &param);

        os_memoryCopy(NULL, ioBuffer, param.content.ModuleTable, sizeof(param.content.ModuleTable));
        *outBufLen = sizeof(param.content.ModuleTable);
        break;

    case TIWLN_REPORT_MODULE_SET:  
    case TIWLN_SET_MODULE:
        PRINTF(DBG_IOCTL_LOUD, ("...TIWLN_SET_MODULE\n"));
        param.paramType = REPORT_MODULE_TABLE_PARAM;
        os_memoryCopy(NULL, param.content.ModuleTable, ioBuffer, sizeof(param.content.ModuleTable));

        report_setParam(((configMgr_t *)pAdapter->CoreHalCtx)->hReport, &param);
        *outBufLen = 0;
        break;

    case TIWLN_REPORT_SEVERITY_SET:
    case TIWLN_SET_SEVERITY:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_SET_SEVERITY - %#lX\n", *(PULONG)ioBuffer));
        param.paramType = REPORT_SEVERITY_TABLE_PARAM;
        os_memoryCopy(NULL, param.content.SeverityTable, ioBuffer, sizeof(param.content.SeverityTable));        

        report_setParam(((configMgr_t *)pAdapter->CoreHalCtx)->hReport, &param);
        *outBufLen = 0;
        break;

    case TIWLN_REPORT_SEVERITY_GET:
    case TIWLN_GET_SEVERITY:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_GET_SEVERITY (%X)\n",
            TIWLN_GET_SEVERITY));
        param.paramType = REPORT_SEVERITY_TABLE_PARAM;
        report_getParam(((configMgr_t *)pAdapter->CoreHalCtx)->hReport, &param);
        os_memoryCopy(NULL, ioBuffer, param.content.SeverityTable, sizeof(param.content.SeverityTable));

        *outBufLen = sizeof(param.content.SeverityTable);
        break;

    case TIWLN_REPORT_PPMODE_SET:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_SET_PPMODE - %#lX\n", *(PULONG)ioBuffer));
        param.paramType = REPORT_PPMODE_VALUE_PARAM;
        *(PULONG)&param.content = *(PULONG)ioBuffer;
        report_setParam(((configMgr_t *)pAdapter->CoreHalCtx)->hReport, &param);
        *outBufLen = 0;
        break;

   case TIWLN_OS_DBG_STATE_GET:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_OS_DBG_STATE_GET - %#lX\n", *(PULONG)ioBuffer));
        param.paramType = REPORT_OS_DBG_STATE_VALUE_PARAM;
        param.content.osDbgState = TiDebugFlag;
        *outBufLen = sizeof(param.content.osDbgState);
        break;

    case TIWLN_OS_DBG_STATE_SET:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_OS_DBG_STATE_SET - %#lX\n", *(PULONG)ioBuffer));
        param.paramType = REPORT_OS_DBG_STATE_VALUE_PARAM;
        *(PULONG)&param.content = *(PULONG)ioBuffer;
        TiDebugFlag = param.content.osDbgState;
        *outBufLen = 0;
        break;


#endif
    case SET_IPC_EVENT_HANDLE:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL SET_IPC_EVENT_HANDLE (%X).\n",SET_IPC_EVENT_HANDLE));
        *outBufLen = 0;
        ntStatus = IPCKernelInit(pAdapter,ioBuffer);
        
    break;

    case DESTROY_IPC_EVENT_HANDLE:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL DESTROY_IPC_EVENT_HANDLE (%X).\n",DESTROY_IPC_EVENT_HANDLE));
        *outBufLen = 0;

        IPCKernelDeInit(pAdapter);

           /*KeSetEvent (pAdapter->IPC.pRxIPCEvent, 0, FALSE) ;*/
           /* KeClearEvent (pAdapter->pRxIPCEvent) ;*/
    break;

    case TIWLN_ENABLE_DISABLE_RX_DATA_FILTERS:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_ENABLE_DISABLE_RX_DATA_FILTERS.\n");
        ntStatus = UtilEnableDisableRxDataFilters(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_GET_RX_DATA_FILTERS_STATISTICS:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_GET_RX_DATA_FILTERS_STATISTICS.\n");
        *pIoCompleteFlag = FALSE;
        ntStatus = UtilGetRxDataFiltersStatistics(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_ADD_RX_DATA_FILTER:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_ADD_RX_DATA_FILTER.\n");
        ntStatus = UtilAddRxDataFilter(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_REMOVE_RX_DATA_FILTER:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_REMOVE_RX_DATA_FILTER.\n");
        ntStatus = UtilRemoveRxDataFilter(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_GET_POWER_CONSUMPTION_STATISTICS:
        PRINT(DBG_IOCTL_LOUD, "...IOCTL TIWLN_GET_POWER_CONSUMPTION_STATISTICS.\n");
        *pIoCompleteFlag = FALSE;
        ntStatus = UtilGetPowerConsumptionStatistics(pAdapter, ioBuffer, outBufLen);
        break;

/* PLT*/
    case TIWLN_PLT_WRITE_REGISTER:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_PLT_WRITE_REGISTER (%X).\n", TIWLN_PLT_WRITE_REGISTER));        
        ntStatus = UtilPltWriteRegister(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_PLT_READ_REGISTER:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_PLT_READ_REGISTER (%X).\n", TIWLN_PLT_READ_REGISTER));
        *pIoCompleteFlag = FALSE;
        ntStatus = UtilPltReadRegister(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_PLT_RX_PER_START:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_PLT_RX_PER_START (%X).\n", TIWLN_PLT_RX_PER_START));
        ntStatus = UtilPltRxPerStart(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_PLT_RX_PER_STOP:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_PLT_RX_PER_STOP (%X).\n", TIWLN_PLT_RX_PER_STOP));
        ntStatus = UtilPltRxPerStop(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_PLT_RX_PER_CLEAR:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_PLT_RX_PER_CLEAR (%X).\n", TIWLN_PLT_RX_PER_CLEAR));
        ntStatus = UtilPltRxPerClear(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_PLT_RX_PER_GET_RESULTS:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_PLT_RX_PER_GET_RESULTS (%X).\n", TIWLN_PLT_RX_PER_GET_RESULTS));
        *pIoCompleteFlag = FALSE;
        ntStatus = UtilPltRxPerGetResults(pAdapter, ioBuffer, outBufLen);
        break;

    case TIWLN_PLT_TX_CW:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_PLT_TX_CW (%X).\n", TIWLN_PLT_TX_CW));
        ntStatus = UtilPltTxCW(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_PLT_TX_CONTINUES:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_PLT_TX_CONTINUES (%X).\n", TIWLN_PLT_TX_CONTINUES));
        ntStatus = UtilPltTxContinues(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_PLT_TX_STOP:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_PLT_TX_STOP (%X).\n", TIWLN_PLT_TX_STOP));
        ntStatus = UtilPltTxStop(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_PLT_MIB_WRITE:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_PLT_MIB_WRITE (%X).\n", TIWLN_PLT_MIB_WRITE));
        ntStatus = UtilPltWriteMib(pAdapter, ioBuffer, inBufLen);
        break;

    case TIWLN_PLT_MIB_READ:
        {
            PLT_MIB_t* pMib = (PLT_MIB_t*)ioBuffer;
            PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_PLT_MIB_READ (%X).\n", TIWLN_PLT_MIB_READ));
            /* find which MIBs return in the same contects*/
            switch(pMib->aMib)
            {
            /*Sync mode MIBs - reads the MIBs data from the driver and not from the FW */
            case PLT_MIB_dot11MaxReceiveLifetime:
            case PLT_MIB_ctsToSelf:
            case PLT_MIB_arpIpAddressesTable:
            case PLT_MIB_dot11GroupAddressesTable:
            case PLT_MIB_rxFilter:
            case PLT_MIB_templateFrame:
            case PLT_MIB_beaconFilterIETable:
            case PLT_MIB_txRatePolicy:
                break;

            /*all other MIBs are Async mode MIBs - reads the MIBs data from the FW */
            default:
                *pIoCompleteFlag = FALSE;
            }
            ntStatus = UtilPltReadMib(pAdapter, ioBuffer, outBufLen, inBufLen);

        }
        break;

    case TIWLN_PLT_RX_TX_CAL:
        *pIoCompleteFlag = FALSE;
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_PLT_RX_TX_CAL (%#lX).\n", ioControlCode));
        ntStatus = UtilPltRxTxCal(pAdapter, ioBuffer, outBufLen, inBufLen);
        break;

    case TIWLN_PLT_RX_CAL:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_PLT_RX_CAL (%#lX).\n", ioControlCode));
        ntStatus = UtilPltRxCal(pAdapter, ioBuffer, outBufLen, inBufLen);
        break;

    case TIWLN_PLT_RX_CAL_RESULT:
        PRINTF(DBG_IOCTL_LOUD, ("...IOCTL TIWLN_PLT_RX_CAL_RESULT (%#lX).\n", ioControlCode));
        ntStatus = utilRxCalibrationStatus(pAdapter, ioBuffer,outBufLen, inBufLen);
        break;

    #ifdef _WINDOWS
    #endif

    default:
        PRINTF(DBG_IOCTL_ERROR, ("...UNKNOWN IOCTL: %#lX\n", ioControlCode));
        ntStatus = (NTSTATUS)STATUS_SUCCESS;

    }

    // os_protectUnlock(pAdapter, pAdapter->SystemProtect); /* Bug. Replaced with the line below. -- MCS00035801 */
    /* Dm: os_protectUnlock( ((configMgr_t  *)(pAdapter->CoreHalCtx))->hOs, pAdapter->SystemProtect); */
    
    return ntStatus;
}
