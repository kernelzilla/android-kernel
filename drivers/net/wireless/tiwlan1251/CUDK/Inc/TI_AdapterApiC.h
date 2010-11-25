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
/* Module:      TI_AdapterApiC.H*/
/**/
/* Purpose:     This module contains "C" interface for TI WLAN Utility Adapter.*/
/**/
/*--------------------------------------------------------------------------*/
#ifndef _TI_ADAPTER_API_H
#define _TI_ADAPTER_API_H

#include "paramOut.h"
#include "roamingMngrTypes.h"
#include "tiwlnif.h"
#include "TI_IPC_Api.h"


#include "TI_AdapterQOS.h"
#include "TI_AdapterGWSI.h"
#include "TI_AdapterSEC.h"
#include "TI_AdapterSG.h"
#include "TI_AdapterPM.h"

#ifdef EXC_MODULE_INCLUDED
    #include "TI_AdapterEXC.h"
    #define EXC_SUPPORT_H    ADAPTER_EXC
#else
    #define EXC_SUPPORT_H
#endif /*EXC_MODULE_INCLUDED*/

#ifdef TI_DBG     
#include "TI_AdapterDBG.h"
#endif/* TI_DBG*/


#ifdef __cplusplus
extern "C" {
#endif

    
/******************************************************************************

    Name:   TI_AdapterInit
    Desc:   Create and initialize the Utility Adapter object
    Params: pszDeviceName - Pointer to a null-terminated string that contains 
                            the name of the specific WLAN device, If this parameter
                            is NULL, TI_AdapterInit() returns a handle to the adapter
                            object that will work with first founded WLAN device.
    
    Return: If the function succeeds, the return value is a handle to the 
            specified Adapter. If the function fails, the return value is NULL.
    
******************************************************************************/
    TI_HANDLE   TI_AdapterInit              (tiCHAR*    pszDeviceName );

/******************************************************************************

    Name:   TI_AdapterDeinit
    Desc:   This function destroys the Utility Adapter object.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
    tiINT32     TI_AdapterDeinit            (TI_HANDLE  hAdapter      );

/******************************************************************************

    Name:   TI_GetApiVersion
    Desc:   This function retrieves the API version information.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
    tiINT32     TI_GetApiVersion            (TI_HANDLE  hAdapter, tiUINT32* puApiVersion);


/******************************************************************************

    Name:   TI_GetDriverVersion
    Desc:   This function retrieves the driver and firmware version information. 
            PdrvVersion points to a TIWLN_VERSION structure, which is defined in
            tiwlnif.h.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pdrvVersion - A pointer to TIWLN_VERSION_EX structure.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetDriverVersion      (TI_HANDLE  hAdapter, 
                                      TIWLN_VERSION_EX* pdrvVersion ); 


/******************************************************************************

    Name:   TI_GetCurrentAddress
    Desc:   This function retrieves the MAC Address of the wireless card.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pCurrentAddr - A pointer to OS_802_11_MAC_ADDRESS that contains the 
                           MAC Address.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetCurrentAddress     (TI_HANDLE  hAdapter, 
                                      OS_802_11_MAC_ADDRESS* pCurrentAddr);



/******************************************************************************

    Name:   TI_GetDriverCapabilities
    Desc:   This function retrieves the driver capabilities list.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pDrvCapabilities - A pointer to a OS_802_11_DRIVER_CAPABILITIES 
                               structure that contains the WiLink™ 4.0 driver 
                               capabilities.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetDriverCapabilities(TI_HANDLE  hAdapter, 
                                     OS_802_11_DRIVER_CAPABILITIES* pDrvCapabilities);

/******************************************************************************

    Name:   TI_SetBSSID
    Desc:   Specify the BSSID to connect.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pAddrBSSID - A pointer to an OS_802_11_MAC_ADDRESS structure.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_SetBSSID                 (TI_HANDLE  hAdapter, 
                                         OS_802_11_MAC_ADDRESS *pAddrBSSID);

/******************************************************************************

    Name:   TI_GetBSSID
    Desc:   This function retrieves the BSSID of the current connection
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pAddrBSSID - A pointer to an OS_802_11_MAC_ADDRESS structure.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetBSSID                 (TI_HANDLE  hAdapter, 
                                         OS_802_11_MAC_ADDRESS *pAddrBSSID);

/******************************************************************************

    Name:   TI_GetBSSIDList
    Desc:   This function retrieves the BSSID list from the driver after a scan
            operation completes. ppBSSIDlist points to an OS_802_11_BSSID_LIST_EX
            structure, which is defined in osDot11.h.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pAddrBSSID - A pointer to an OS_802_11_BSSID_LIST structure.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetBSSIDList             (TI_HANDLE  hAdapter, 
                                         OS_802_11_BSSID_LIST_EX** ppBSSIDlist );



/******************************************************************************

    Name:   TI_GetFullBSSIDList
    Desc:   This function is almost the same as TI_SetBSSIDList, the only 
            difference is that is retrieves more information than TI_SetBSSIDList.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            ppBSSIDlist - A pointer to an OS_802_11_BSSID_LIST structure.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetFullBSSIDList         (TI_HANDLE  hAdapter, 
                                         OS_802_11_BSSID_LIST_EX** ppBSSIDlist );

/******************************************************************************

    Name:   TI_GetSelectedBSSIDInfo
    Desc:   Retrieves the BSSID information from the driver after a scan operation
            completes and select was performed. The information structure is 
            published in OS_802_11_BSSID_EX format (see TI_GetBSSIDList() command).
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pSelectedBSSIDInfo - A pointer to a structure to be filled with the 
                                 selected BSSID information.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetSelectedBSSIDInfo     (TI_HANDLE hAdapter, 
                                         OS_802_11_BSSID_EX  *pSelectedBSSIDInfo);

/******************************************************************************

    Name:   TI_GetPrimaryBSSIDInfo
    Desc:   Retrieves the primary BSSID information from the driver, i.e. the BSSID
                The STA is currently connected to. The information structure is 
            published in OS_802_11_BSSID_EX format (see TI_GetBSSIDList() command).
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pSelectedBSSIDInfo - A pointer to a structure to be filled with the 
                                 selected BSSID information.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetPrimaryBSSIDInfo      (TI_HANDLE hAdapter, 
                                         OS_802_11_BSSID_EX  *pSelectedBSSIDInfo);

/******************************************************************************

    Name:   TI_SetDesiredChannel
    Desc:   This function sets the desired operating channel. uDesiredChannel 
            points to a channel number, which can range from 1 to 14, 
            36,40,44,48,52,56,60,64 depended from mode.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            uDesiredChannel - A pointer to a tiUINT32 that contains the channel
                              number.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_SetDesiredChannel        (TI_HANDLE  hAdapter, 
                                         tiUINT32   uDesiredChannel);

/******************************************************************************

    Name:   TI_GetDesiredChannel
    Desc:   This function retrieves the desired operating channel. 
            puDesiredChannel points to a channel number, which can range from 
            1 to 14, 36,40,44,48,52,56,60,64 depended on mode (a/b/g).
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            puDesiredChannel - A pointer to a tiUINT32 that contains the channel
                               number.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetDesiredChannel        (TI_HANDLE  hAdapter, 
                                         tiUINT32*  puDesiredChannel);

/******************************************************************************

    Name:   TI_GetCurrentChannel
    Desc:   This function retrieves the current operating channel. 
            puDesiredChannel points to a channel number, which can range from 
            1 to 14, 36,40,44,48,52,56,60,64 depended on mode (a/b/g).
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            puCurrentChannel - A pointer to a tiUINT32 that contains the current
                               channel number.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetCurrentChannel        (TI_HANDLE  hAdapter, 
                                         tiUINT32*  puCurrentChannel);

/******************************************************************************

    Name:   TI_GetDesiredRate
    Desc:   This function retrieves the desired transmission rate for the adapter.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            puDesiredRate - A pointer to a tiUINT32 that contains the desired 
                            transmission rate
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetDesiredRate           (TI_HANDLE  hAdapter, 
                                         tiUINT32*  puDesiredRate);
/******************************************************************************

    Name:   TI_GetCurrentRate
    Desc:   This function retrieves the current transmission rate for the 
            adapter. The adapter may change the desired rate if using AUTO rate.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            puCurrentRate - A pointer to a tiUINT32 that contains the current 
                            transmission rate
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetCurrentRate           (TI_HANDLE  hAdapter, 
                                         tiUINT32*  puCurrentRate);

/******************************************************************************

    Name:   TI_SetFragmentThreshold
    Desc:   This function sets the current fragmentation threshold.
            Only packets that are greater than the fragmentation threshold are
            fragmented.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            uFragmentThreshold - a pointer to a tiUINT32 that contains the 
                                 fragmentation threshold in bytes.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_SetFragmentThreshold     (TI_HANDLE  hAdapter, 
                                         tiUINT32   uFragmentThreshold );

/******************************************************************************

    Name:   TI_GetFragmentThreshold
    Desc:   This function retrieves the current fragmentation threshold.
            Only packets that are greater than the fragmentation threshold 
            are fragmented.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            uFragmentThreshold - A pointer to a tiUINT32 that contains the 
                                 fragmentation threshold in bytes.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetFragmentThreshold     (TI_HANDLE  hAdapter, 
                                         tiUINT32*  puFragmentThreshold);

/******************************************************************************

    Name:   TI_SetBSSType
    Desc:   This function sets the network mode, either Infrastructure or Ad Hoc.
            uBSSType points to an OS_802_11_NETWORK_MODE enum, which is defined in
            osDot11.h.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            uBSSType - Points to the network mode (OS_802_11_NETWORK_MODE enum):
                       os802_11IBSS - for Ad Hoc mode.
                       os802_11Infrastructure - for infrastructure mode.
                       os802_11AutoUnknown - for automatic mode in which the 
                                             adapter can switch between Ad Hoc and 
                                             infrastructure modes as required.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_SetBSSType               (TI_HANDLE  hAdapter, 
                                         OS_802_11_NETWORK_MODE  uBSSType );

/******************************************************************************

    Name:   TI_GetBSSType
    Desc:   This function retrieves the network mode. For more information see 
            TI_SetBSSID().
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            puBSSType - A pointer to the network mode (OS_802_11_NETWORK_MODE enum):
                        see TI_SetBSSType.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetBSSType               (TI_HANDLE  hAdapter, 
                                         OS_802_11_NETWORK_MODE* puBSSType);



/******************************************************************************

    Name:   TI_SetBeaconFilterDesiredState
    Desc:   
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            uBeaconFilterMode - 
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_SetBeaconFilterDesiredState(TI_HANDLE  hAdapter, 
                                           OS_802_11_BEACON_FILTER_MODE uBeaconFilterMode );

/******************************************************************************

    Name:   TI_GetBeaconFilterDesiredState
    Desc:   
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            uBeaconFilterMode - 
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetBeaconFilterDesiredState(TI_HANDLE  hAdapter, 
                                           tiUINT8* uBeaconFilterMode );

/******************************************************************************

    Name:   TI_SetRTSThreshold
    Desc:   This function sets the current RTS (Request to Send) threshold.
            The value specifies the packet size, in bytes, beyond which the WLAN
            invokes its RTS/CTS mechanism. Packets that exceed the specified RTS
            threshold trigger the RTS/CTS mechanism. The NIC transmits smaller
            packets without RTS/CTS.
            An RTS threshold value of 0 indicates that the NIC should transmit all
            packets with RTS/CTS.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            uRTSThreshold - Contains the RTS Threshold in bytes.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_SetRTSThreshold          (TI_HANDLE  hAdapter, 
                                         tiUINT32   uRTSThreshold  );

/******************************************************************************

    Name:   TI_GetRTSThreshold
    Desc:   This function retrieves the current RTS (Request to Send) threshold.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            puRTSThreshold - A pointer to a tiUINT32 that contains the RTS 
                             Threshold in bytes.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetRTSThreshold          (TI_HANDLE  hAdapter, 
                                         tiUINT32*  puRTSThreshold );  

/******************************************************************************

    Name:   TI_SetShortPreamble
    Desc:   This function sets the current preamble length. 
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
    uShortPreamble - Indicates the preamble length. A value of 0 specifies long 
                     preambles and a value of 1 specifies short preambles.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_SetShortPreamble         (TI_HANDLE  hAdapter, 
                                         tiUINT32   uShortPreamble );

/******************************************************************************

    Name:   TI_GetShortPreamble
    Desc:   This function retrieves the current preamble length. A value of 0 
            (zero) in puShortPreamble specifies long preambles and a value of 
            1 (one) specifies short preambles.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            puShortPreamble - A pointer to a tiUINT32 that indicates the 
                              preamble length.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetShortPreamble         (TI_HANDLE  hAdapter, 
                                         tiUINT32*  puShortPreamble);


/******************************************************************************

    Name:   TI_SetSSID
    Desc:   This function sets desired SSID.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pSSIDname - Pointer to a null-terminated string that contains a 
                        desired SSID
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_SetSSID                  (TI_HANDLE  hAdapter, 
                                         tiUINT8*   pSSIDname   );

/******************************************************************************

    Name:   TI_GetDesiredSSID
    Desc:   This function retrieves the desired SSID.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pSSID - Pointer to a null-terminated string that contains a desired
                    SSID.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetDesiredSSID           (TI_HANDLE  hAdapter, 
                                         OS_802_11_SSID* pSSID  );

/******************************************************************************

    Name:   TI_GetCurrentSSID
    Desc:   This function retrieves the current SSID.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pSSID - Pointer to a null-terminated string that contains a current
                    SSID.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetCurrentSSID           (TI_HANDLE  hAdapter, 
                                         OS_802_11_SSID* pSSID   );

/******************************************************************************

    Name:   TI_GetStatistics
    Desc:   This function retrieves driver statistics. pStatistics points to a 
            TIWLN_STATISTICS structure, which is defined in tiwlnif.h. The 
            OS_802_11* types are defined in osDot11.h.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pStatistics - A pointer to a TIWLN_STATISTICS structure.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetStatistics            (TI_HANDLE  hAdapter, 
                                         TIWLN_STATISTICS* pStatistics );

/******************************************************************************

    Name:   TI_GetTxStatistics
    Desc:   This function retrieves driver statistics. pStatistics points to a
            TIWLN_TX_STATISTICS structure, which is defined in tiwlnif.h. The 
            OS_802_11* types are defined in osDot11.h. The TIWLN_TX_STATISTICS
            structure includes the structure txDataCounters_t.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pTxStatistics - A pointer to a TIWLN_TX_STATISTICS structure.
            clearStatsFlag - Indication whether to clear the statistic counters
                             upon read.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetTxStatistics          (TI_HANDLE  hAdapter, 
                                         TIWLN_TX_STATISTICS* pTxStatistics, 
                                         UINT32 clearStatsFlag );

/******************************************************************************

Name:   TI_EnableDisableRxDataFilters
Desc:   This function enables or disables the Rx Data Filter feature.

Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
        enable - 0 to disable the feature, any other value to enable
Return: TI_RESULT_OK on success. Any other value indicates an error.

******************************************************************************/
tiINT32     TI_EnableDisableRxDataFilters(TI_HANDLE hAdapter,
                                          tiBOOL enabled);


/******************************************************************************

Name:   TI_AddRxDataFilter
Desc:   This function adds the given filter to the WLAN driver's list of
data filters.

Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
pRequest - A pointer to the data filter request.
Return: TI_RESULT_OK on success, RX_FILTER_ALREADY_EXISTS if filter
        already exists. Any other value indicates an error.

******************************************************************************/
tiINT32     TI_AddRxDataFilter          (TI_HANDLE hAdapter,
                                         TIWLAN_DATA_FILTER_REQUEST * pRequest);


/******************************************************************************

Name:   TI_GetRxDataFiltersStatistics
Desc:   This function adds the given filter to the WLAN driver's list of
        data filters.

Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
        pRequest - A pointer to the data filter request.
Return: TI_RESULT_OK on success. Any other value indicates an error.

******************************************************************************/
tiINT32     TI_GetRxDataFiltersStatistics(TI_HANDLE hAdapter,
                                          TIWLAN_DATA_FILTER_STATISTICS * pStatistics);


/******************************************************************************

Name:   TI_GetPowerConsumptionStatistics
Desc:   This function retrieves the power consumption statisticts from the FW.

Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
        pStatistics - A pointer to the statistics data structure.
Return: TI_RESULT_OK on success. Any other value indicates an error.

******************************************************************************/
tiINT32     TI_GetPowerConsumptionStatistics(TI_HANDLE hAdapter, PowerConsumptionTimeStat_t * pStatistics);



/******************************************************************************

Name:   TI_RemoveRxDataFilter
Desc:   This function removes a previously added filter with the given
        details from the WLAN driver's list of data filters. Note that the
        request must be identical to the one given when the filter was added.

Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
        pRequest - A pointer to the data filter request.
Return: TI_RESULT_OK on success, RX_FILTER_DOES_NOT_EXIST if filter
        doesn't exist. Any other value indicates an error.

******************************************************************************/
tiINT32     TI_RemoveRxDataFilter          (TI_HANDLE hAdapter,
                                            TIWLAN_DATA_FILTER_REQUEST * pRequest);


/******************************************************************************

    Name:   TI_SetSupportedRates
    Desc:   This function sets the transmission rates supported by the driver.
            This is the list of basic and supported rates. Basic rates have MSB
            set to 1 (one).
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pSupportedRatesLst - A pointer to a buffer that contains list of rates.
            uBufLength - Contains a size of buffer.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_SetSupportedRates        (TI_HANDLE  hAdapter, 
                                         tiUINT8*   pSupportedRatesLst, 
                                         tiUINT32  uBufLength );

/******************************************************************************

    Name:   TI_GetSupportedRates
    Desc:   This function retrieves the transmission rates supported by the driver.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pSupportedRatesLst - A pointer to a buffer that contains list of rates.
            uBufLength - Contains a size of buffer
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetSupportedRates        (TI_HANDLE  hAdapter, 
                                         tiUINT8*   pSupportedRatesLst, 
                                         tiUINT32  uBufLength );


/******************************************************************************

    Name:   TI_SetIBSSProtection
    Desc:   Sets the 802.11g extended rate protection (ERP) configuration of 
            the driver. Configuration of ERP is only possible when the adapter 
            is operating in Ad Hoc mode. In infrastructure mode, the driver uses
            the ERP method supported by the AP.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            uProtection - Indicates the 802.11g protections.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    Note:   This function supported only on WiLink™ 4.0 - based designs.
    
******************************************************************************/
tiINT32     TI_SetIBSSProtection        (TI_HANDLE  hAdapter, 
                                         tiUINT32  uProtection );

/******************************************************************************

    Name:   TI_GetIBSSProtection
    Desc:   Returns the 802.11g extended rate protection (ERP) configuration of 
            the driver. For more information see TI_SetIBSSProtection().
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            puProtection - A pointer to a tiUINT32 that indicates the 802.11g
                           protections.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetIBSSProtection        (TI_HANDLE  hAdapter, 
                                         tiUINT32* puProtection);

/******************************************************************************

    Name:   TI_GetDriverState
    Desc:   This function returns a driver's state.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            puDriverState - A pointer to a driverState_e that indicates the 
                            driver state:
                            DRIVER_STATE_IDLE = 0,
                            DRIVER_STATE_SCANNING = 1,
                            DRIVER_STATE_SELECTING = 2,
                            DRIVER_STATE_CONNECTING = 3,
                            DRIVER_STATE_CONNECTED = 4,
                            DRIVER_STATE_DISCONNECTED = 5,
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetDriverState           (TI_HANDLE  hAdapter,  
                                         driverState_e* puDriverState );

/******************************************************************************

    Name:   TI_SetShortSlot
    Desc:   Sets the 802.11g slot time. 
            A value of 0 (zero) in uShortSlot indicates a long slot time (20 uSec) 
            A value of 1 (one) in uShortSlot indicates a short slot time (9 uSec).
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            uShortSlot - Indicates the 802.11g slot time.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    Note: This function supported only on WiLink™ 4.0 - based designs.
    
******************************************************************************/
tiINT32     TI_SetShortSlot             (TI_HANDLE  hAdapter, 
                                         tiUINT32  uShortSlot  );

/******************************************************************************

    Name:   TI_GetShortSlot
    Desc:   Returns the 802.11g slot time. Refer to TI_SetShortSlot.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            puShortSlot - pointer to a tiUINT32 that indicates the 802.11g slot
                          time.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetShortSlot             (TI_HANDLE  hAdapter, 
                                         tiUINT32* puShortSlot );

/******************************************************************************

    Name:   TI_SetTxPowerDbm
    Desc:   This function sets the maximum station transmit power in Dbm. The station 
            also takes into consideration two additional power level settings 
            AP-IE and Chip ID. The final setting is the minimum of among these
            three settings.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            uTxPower - Contains a station power level
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_SetTxPowerDbm          (TI_HANDLE  hAdapter, 
                                         tiUINT8  uTxPower);

/******************************************************************************

    Name:   TI_GetTxPowerLevel
    Desc:   Retrieve the current station power level table.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            puTxPower - A pointer to a tiCHAR that contains the station power 
                        level table.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetTxPowerLevel          (TI_HANDLE  hAdapter, 
                                         tiCHAR* puTxPower);

/******************************************************************************

    Name:   TI_GetTxPowerDbm
    Desc:   Retrieve the current Tx Power in Dbm/10 value.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            puTxPower - A pointer to a tiCHAR that contains the station TX power
                        in Dbm value.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetTxPowerDbm            (TI_HANDLE  hAdapter, 
                                         tiCHAR* puTxPower);

/******************************************************************************

    Name:   TI_GetSupportedNetworkTypes
    Desc:   This function retrieves the supported network types.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pNetTypeLst - Pointer to the buffer that contains list of supported 
                          network types.
            uMaxNetTypes - Maximum number of types that will contains in the 
                           buffer pNetTypeLst.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetSupportedNetworkTypes (TI_HANDLE  hAdapter, 
                                         OS_802_11_NETWORK_TYPE* pNetTypeLst, 
                                         tiUINT32 uMaxNetTypes);

/******************************************************************************

    Name:   TI_SetNetworkTypeInUse
    Desc:   This function sets the network type.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            uNetType - Contains network type:
                        os802_11FH,
                        os802_11DS,
                        os802_11OFDM5,
                        os802_11OFDM24,
                        os802_11OFDM24_AND_5 
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_SetNetworkTypeInUse      (TI_HANDLE  hAdapter, 
                                         OS_802_11_NETWORK_TYPE   uNetType  );

/******************************************************************************

    Name:   TI_GetNetworkTypeInUse
    Desc:   This function retrieves the current network type in use.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            puNetType - Pointer to the buffer that contains value of network type.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetNetworkTypeInUse      (TI_HANDLE  hAdapter, 
                                         OS_802_11_NETWORK_TYPE*  puNetType );


/******************************************************************************

    Name:   TI_GetNumberOfAntennas
    Desc:   This function retrieves the number of antennas.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            puNumberOfAntennas - 
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetNumberOfAntennas      (TI_HANDLE  hAdapter, 
                                         tiUINT32*  puNumberOfAntennas    );

/******************************************************************************

    Name:   TI_SetAntennaDiversityParams
    Desc:   This function sets various antenna diversity parameters.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pAntennaDiversityOptions - Pointer to antenna diversity parameters 
                  structure holding required parameters:
                  enableRxDiversity - specifies whether antenna diversity should
                                      be enables for reception.
                  rxSelectedAntenna - specifies which antenna to use for reception.
                  enableTxDiversity - specifies whether antenna diversity should 
                                      be enables for transmission.
                  txSelectedAntenna - specifies which antenna to use for 
                                      transmission.
                  rxTxSharedAnts    - specifies whether to share reception and
                                      transmission antennas.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_SetAntennaDiversityParams(TI_HANDLE  hAdapter, 
                                         PTIWLAN_ANT_DIVERSITY pAntennaDiversityOptions);


/******************************************************************************

    Name:   TI_GetRegDomainTable
    Desc:   This function retrieves the regularity domain table.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pRegDomainTable - Pointer to TIWLAN_REGDOMAINS structure which includes
                              regulatory domains table and its size.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetRegDomainTable(TI_HANDLE  hAdapter, 
                                 TIWLN_REGDOMAINS* pRegDomainTable);                                         
                                         
/******************************************************************************

    Name:   TI_EnableDisable_802_11d
    Desc:   This function enables or disables the 802.11d protocol.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            enableDisable_802_11d - Enable or Disable value
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_EnableDisable_802_11d    (TI_HANDLE  hAdapter, 
                                         tiUINT8 enableDisable_802_11d);

/******************************************************************************

    Name:   TI_Get_802_11d
    Desc:   This function retrieves whether the 802.11d protocol in enabled or 
            disabled.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            enableDisable_802_11d - A pointer to a tiUINT8 which returns an 
                                    Enable or Disable value.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_Get_802_11d              (TI_HANDLE  hAdapter, 
                                         tiUINT8 *enableDisable_802_11d);

/******************************************************************************

    Name:   TI_EnableDisable_802_11h
    Desc:   This function enables or disables the 802.11h protocol.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            enableDisable_802_11h - Enable or Disable value
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_EnableDisable_802_11h    (TI_HANDLE  hAdapter, 
                                         tiUINT8 enableDisable_802_11h);

/******************************************************************************

    Name:   TI_Get_802_11h
    Desc:   This function retrieves whether the 802.11h protocol in enabled or
            disabled.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            enableDisable_802_11h - A pointer to a tiUINT8 which returns an 
                                    Enable or Disable value.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_Get_802_11h              (TI_HANDLE  hAdapter, 
                                         tiUINT8 *enableDisable_802_11h);

/******************************************************************************

    Name:   TI_Set_countryIeFor2_4_Ghz
    Desc:   
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            countryIe - 
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_Set_countryIeFor2_4_Ghz  (TI_HANDLE  hAdapter, 
                                         country_t countryIe);

/******************************************************************************

    Name:   TI_Get_countryIeFor2_4_Ghz
    Desc:   
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            countryString - 
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_Get_countryIeFor2_4_Ghz  (TI_HANDLE  hAdapter, 
                                         tiUINT8 **countryString);

/******************************************************************************

    Name:   TI_Set_countryIeFor5_Ghz
    Desc:   
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            countryIe - 
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_Set_countryIeFor5_Ghz    (TI_HANDLE  hAdapter, 
                                         country_t countryIe);

/******************************************************************************

    Name:   TI_Get_countryIeFor5_Ghz
    Desc:   
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            countryString - 
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_Get_countryIeFor5_Ghz    (TI_HANDLE  hAdapter, 
                                         tiUINT8 **countryString);

/******************************************************************************

    Name:   TI_Set_minMaxDfsChannels
    Desc:   
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            DFS_ChannelRange - Minimum and maximum cahnnel numbers for which 
                               DFS is used
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_Set_minMaxDfsChannels    (TI_HANDLE  hAdapter,
                                         DFS_ChannelRange_t DFS_ChannelRange);

/******************************************************************************

    Name:   TI_Get_minMaxDfsChannels
    Desc:   
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            DFS_ChannelRange - Minimum and maximum cahnnel numbers for which 
                               DFS is used
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_Get_minMaxDfsChannels    (TI_HANDLE  hAdapter,
                                         DFS_ChannelRange_t *DFS_ChannelRange);

/******************************************************************************

    Name:   TI_Start
    Desc:   This command starts the driver operation. The driver will start 
            scanning and will try to connect according to its configuration.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    Note:   Used for Linux only.

******************************************************************************/
    tiINT32     TI_Start                    (TI_HANDLE  hAdapter);

/******************************************************************************

    Name:   TI_Stop
    Desc:   This command blocks the driver's API and turns off the WiLink™ 4.0 
            HW. The driver will be kept loaded and will keep its configuration
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    Note:   Used for Linux only.
    
******************************************************************************/
    tiINT32     TI_Stop                     (TI_HANDLE  hAdapter);

/******************************************************************************

    Name:   TI_StartSM
    Desc:   This command starts the Supplicant Manager module operation.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
    tiINT32     TI_StartSM                  (TI_HANDLE  hAdapter);

/******************************************************************************

    Name:   TI_StopSM
    Desc:   This command stops the supplicant manager operation.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
    tiINT32     TI_StopSM                   (TI_HANDLE  hAdapter);

/******************************************************************************

    Name:   TI_GetRSSI
    Desc:   This function returns the current RSSI.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pRSSI - The current RSSI level.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetRSSI                  (TI_HANDLE  hAdapter, 
                                         tiINT32* pRSSI); 

/******************************************************************************

    Name:   TI_GetSNR
    Desc:   This function returns the current SNR.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pSNR - The current SNR level.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetSNR                   (TI_HANDLE  hAdapter, 
                                         tiUINT32* pSNR); 

/******************************************************************************

    Name:   TI_Disassociate
    Desc:   This command sets Junk SSID to the Driver. It makes the WiLink™ 4.0
            to disassociate any current AP and to return to idle state. The 
            Driver does not attempt to connect to any other AP until a valid 
            SSID is set.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
    tiINT32     TI_Disassociate             (TI_HANDLE  hAdapter);



/******************************************************************************

    Name:   TI_RegisterEvent
    Desc:   This function registers a driver event, which will trigger the 
            specified callback function.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pEventParams - Information about the event to which you are registering.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_RegisterEvent            (TI_HANDLE  hAdapter, 
                                         IPC_EVENT_PARAMS*     pEventParams );

/******************************************************************************

    Name:   TI_UnRegisterEvent
    Desc:   This function un-registers a driver event.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pEventParams - Information about the event to which you are 
                           registering.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_UnRegisterEvent          (TI_HANDLE  hAdapter, 
                                         IPC_EVENT_PARAMS*     pEventParams );

/******************************************************************************

    Name:   TI_StartScan
    Desc:   Starts a scan operation. The user can define the scan parameters.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pScanParams - The parameters for the requested scan.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    Note:   up to 16 scanned channels.
    
******************************************************************************/
tiINT32     TI_StartScan                (TI_HANDLE  hAdapter, 
                                         scan_Params_t *pScanParams);

/******************************************************************************

    Name:   TI_StopScan
    Desc:   Send a command to the WiLink™ 4.0 driver to terminate the scan process.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
    tiINT32     TI_StopScan                 (TI_HANDLE  hAdapter);
    
/******************************************************************************

    Name:   TI_SetScanPolicy
    Desc:   Sends a command buffer to the driver scan manager logic.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            buffer - A pointer to the buffer holding the scan policy.
            bufferLength - The length of the above buffer.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_SetScanPolicy            (TI_HANDLE  hAdapter, 
                                         UINT8* buffer, 
                                         UINT16 bufferLength);

/******************************************************************************

    Name:   TI_GetScanBssList
    Desc:   Retrieves the scan manager tracking BSS list. This function should 
            be used for debug purposes only.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            bssList - A pointer to a buffer in which the BSS list will be stored.                      
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetScanBssList           (TI_HANDLE  hAdapter, 
                                         bssList_t* bssList);






    /* ****************************************** */
/******************************************************************************

    Name:   TI_WLAN_IsDriverRun 
    Desc:   This command returns the driver status, running or Idle.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pbStatus - The Driver status, running or IDLE.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_WLAN_IsDriverRun             (TI_HANDLE  hAdapter, 
                                             tiBOOL* pbStatus);

/******************************************************************************

    Name:   TI_Set4XState   
    Desc:   This command sets the 4X state.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            bStatus - TRUE-4X enabled, FALSE – disabled..
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    Note:   NOT SUPPORTED !!!
    
******************************************************************************/
tiINT32     TI_Set4XState                   (TI_HANDLE  hAdapter, 
                                             tiBOOL bStatus);

/******************************************************************************

    Name:   TI_Get4XState   
    Desc:   This command returns 4X status.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pbStatus - .
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_Get4XState                   (TI_HANDLE  hAdapter, 
                                             tiBOOL* pbStatus);

 
/******************************************************************************

    Name:   TI_SetExtRatesIE    
    Desc:   This command sets the Draft number.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            uExtRatesIE - The Draft number: 5 or earlier, 6 or later..
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_SetExtRatesIE                (TI_HANDLE  hAdapter, 
                                             tiUINT32 uExtRatesIE);

/******************************************************************************

    Name:   TI_GetExtRatesIE    
    Desc:   This command returns the Draft number.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            puExtRatesIE - The Draft number: 5 or earlier, 6 or later..
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetExtRatesIE                (TI_HANDLE  hAdapter, 
                                             tiUINT32* puExtRatesIE);

/******************************************************************************

    Name:   TI_SetEarlyWakeupMode    
    Desc:   This command sets the Early Wakeup mode.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            uEarlyWakeup - The early Wakeup  mode: 0 - disabled, 1 - enabled
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32      TI_SetEarlyWakeupMode          (TI_HANDLE  hAdapter,
                                              tiUINT8 uEarlyWakeup);

/******************************************************************************

    Name:   TI_GetEarlyWakeupMode    
    Desc:   This command returns the Early Wakeup mode.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            uEarlyWakeup - The early Wakeup  mode: 0 - disabled, 1 - enabled
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32      TI_GetEarlyWakeupMode         (TI_HANDLE  hAdapter,
                                              tiUINT8* uEarlyWakeup);


/******************************************************************************

    Name:   TI_SetRoamingConfiguration  
    Desc:   The TI_SetRoamingConfiguration() function sends a command buffer to 
            the driver roaming manager logic.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            buffer - The command buffer.
            bufferLength - The roam command buffer length (bytes).
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_SetRoamingConfiguration      (TI_HANDLE  hAdapter, 
                                             UINT8* buffer, 
                                             UINT16 bufferLength);

/******************************************************************************

    Name:   TI_GetRoamingConfiguration  
    Desc:   The TI_SetRoamingConfiguration() function sends a command buffer to 
            the driver roaming manager logic.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            buffer - The command buffer.
            bufferLength - The roam command buffer length (bytes).
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
tiINT32     TI_GetRoamingConfiguration      (TI_HANDLE  hAdapter, 
                                             UINT8* buffer, 
                                             UINT16 bufferLength);




/* PLT */
/******************************************************************************

    Name:   TI_PLT_ReadRegister 
    Desc:   This command reads a firmware register value.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            uRegisterAddr  - Register address.
            puRegisterData - Pointer to the register data.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
    tiINT32     TI_PLT_ReadRegister     (TI_HANDLE  hAdapter,
                                         UINT32 uRegisterAddr,
                                         PUINT32 puRegisterData );

/******************************************************************************

    Name:   TI_PLT_WriteRegister    
    Desc:   This command writes a firmware register value.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            uRegisterAddr - Register address.
            uRegisterData - register data.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
    tiINT32     TI_PLT_WriteRegister    (TI_HANDLE  hAdapter,
                                         UINT32 uRegisterAddr,
                                         UINT32 uRegisterData );
/******************************************************************************

    Name:   TI_PLT_RxPerStart   
    Desc:   Start or resume the PER measurement. 
            This function will put the device in promiscuous mode, and resume counters update. 
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
    tiINT32     TI_PLT_RxPerStart       (TI_HANDLE  hAdapter);

/******************************************************************************

    Name:   TI_PLT_RxPerStop    
    Desc:   Stop Rx PER measurements. 
            This function stop counters update and make it is safe to read the PER test result.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
    tiINT32     TI_PLT_RxPerStop        (TI_HANDLE  hAdapter);

/******************************************************************************

    Name:   TI_PLT_RxPerClear   
    Desc:   Clear the Rx PER test results.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
    tiINT32     TI_PLT_RxPerClear       (TI_HANDLE  hAdapter);

/******************************************************************************

    Name:   TI_PLT_RxPerGetResults  
    Desc:   Get the last Rx PER test results.
            The RX PER test is conducted in order to evaluate the PER of received packets and is basically done by measuring the ratio between the amount of packets received with FCS errors and the total amount of packets received at a certain period of time.
            You can Start and Stop the frame accumulation several times, and read the total frame count after the last Stop command.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pPltRxPer - The PLT PER results.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
    tiINT32     TI_PLT_RxPerGetResults  (TI_HANDLE  hAdapter,
                                         PltRxPer_t* pPltRxPer );
/******************************************************************************

    Name:   TI_PLT_TxCW 
    Desc:   Generate carrier wave in a specific channel and band..
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pPltTxCW -  The Carrier wave channel and band.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
    tiINT32     TI_PLT_TxCW             (TI_HANDLE  hAdapter,
                                         TestCmdChannelBand_t* pPltTxCW);
/******************************************************************************

    Name:   TI_PLT_TxContiues   
    Desc:   Continuous transmit series of numbers with a valid MAC header (
            "CAFE BABE" & "DEAD BEEF" as MAC address). However there is no 802.11 air access compliance.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pPltTxContinues -  Infrmation about continuess transmition.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
    tiINT32     TI_PLT_TxContiues       (TI_HANDLE  hAdapter,
                                         PltTxContinues_t* pPltTxContinues);
/******************************************************************************

    Name:   TI_PLT_TxStop   
    Desc:   Stop packet transmission initiated by the TI_PLT_TxCW() and TI_PLT_TxContiues() functions.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
    tiINT32     TI_PLT_TxStop           (TI_HANDLE  hAdapter);

/******************************************************************************

    Name:   TI_PLT_ReadMIB  
    Desc:   Reads a PLT MIB.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pMib - Holds the MIB structure.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
    tiINT32     TI_PLT_ReadMIB          (TI_HANDLE  hAdapter,
                                         PLT_MIB_t* pMib);
/******************************************************************************

    Name:   TI_PLT_WriteMIB 
    Desc:   Writes a PLT MIB.
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            pMib - Holds the MIB structure.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
    tiINT32     TI_PLT_WriteMIB         (TI_HANDLE  hAdapter,
                                         PLT_MIB_t* pMib);

/******************************************************************************
          TX Calibration functions
******************************************************************************/

/******************************************************************************
    Name:   TI_PLT_TxCalGainGet	
    Desc:	Retrieves the TX chain gain settings.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pPLTGainGet - Holds the return gain results.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
	tiINT32		TI_PLT_TxCalGainGet		(TI_HANDLE  hAdapter,
                                         PltGainGet_t* pPLTGainGet);

/******************************************************************************

    Name:   TI_PLT_TxCalGainAdjust	
    Desc:	Changes the TX chain gain settings.
            Value is provided in 0.25 dB steps. (e.g. 0xfe is -0.5dB ; 0x10 is +4 dB)
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            uTxGainChange - the TX gain change from current value in 2's complement.
				Value is defined in steps of 0.25dB.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
	tiINT32		TI_PLT_TxCalGainAdjust	(TI_HANDLE  hAdapter,
                                         tiINT32   uTxGainChange);

/******************************************************************************

    Name:   TI_PLT_TxCalStart	
    Desc:	This PLT function handles all the activities required before initiating the TX calibration procedure.
            As part of it activities it should make sure that CLPC is disabled and the gain control loop is open.            
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
			pPLTTxCal - Band, Channel, Reference Tx power.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
	tiINT32		TI_PLT_TxCalStart	(TI_HANDLE  hAdapter,
									 PltTxCalibrationRequest_t* pPLTTxCal);

/******************************************************************************

    Name:   TI_PLT_TxCalStop	
    Desc:	This PLT function is a cleanup functions for the radio to be able to resume normal operation.
            As part of its activities it should put the CLPC mode back into operation.            
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
	tiINT32		TI_PLT_TxCalStop	(TI_HANDLE  hAdapter);
    
/******************************************************************************

    Name:   TI_PLT_TxCalStop	
    Desc:	This PLT function provides the all information required by the upper 
            driver in order to update the NVS image. It received a parameter 
            defining the type of update information required and provides an array 
            of elements defining the data bytes to be written to the NVS image and 
            the byte offset in which they should be written.            
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pPLT_NVSUpdateBuffer - The data to be updated in the NVS file.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
	tiINT32		TI_PLT_RxTxCalNVSUpdateBuffer	(TI_HANDLE  hAdapter,
                                             PltNvsResultsBuffer_t* pPLT_NVSUpdateBuffer);

    
/******************************************************************************

    Name:   TI_PLT_TxCalGainAdjust	
    Desc:	This PLT function perform a RSSI measurement on the selected channel, 
            and returns the data bytes to be written to the NVS image and 
            the byte offset in which they should be written. 
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pPltRxCalibration_t - The input and output parametrs needed for the RX calibration.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
	tiINT32		TI_PLT_RxCal(TI_HANDLE  hAdapter,
                             PltRxCalibrationRequest_t* pPltRxCalibration_t); 

    
/******************************************************************************

    Name:   TI_PLT_RadioTune 
    Desc:   Generate carrier wave in a specific channel and band..
    Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
            TestCmdChannelBand_t -  The channel and band.
    Return: TI_RESULT_OK on success. Any other value indicates an error.
    
******************************************************************************/
    tiINT32     TI_PLT_RadioTune         (TI_HANDLE  hAdapter,
                                         TestCmdChannelBand_t* pChannelBand);

#ifdef _WINDOWS
#endif 


/******************************************************************************

Name:   TI_PLT_TxPowerRef 
Desc:   Set Tx power reference.
Params: hAdapter - The Adapter handle returned by TI_AdapterInit().
uTxPower -  Tx power in Dbm/10.
Return: TI_RESULT_OK on success. Any other value indicates an error.

******************************************************************************/
	tiINT32     TI_PLT_TxPowerRef		     (TI_HANDLE  hAdapter,
											 tiUINT32 uTxPower);

	

#ifdef TI_DBG
#ifdef DRIVER_PROFILING
    tiINT32     TI_ProfileReport                (TI_HANDLE  hAdapter);
    tiINT32     TI_CpuEstimatorCommand          (TI_HANDLE  hAdapter, tiUINT8 uType, tiUINT32 uData);
#endif
#endif

    tiINT32     TI_Open_EAPOL_Interface        ( TI_HANDLE  hAdapter);
    tiINT32     TI_Close_EAPOL_Interface       ( TI_HANDLE  hAdapter);
    tiINT32     TI_Send_EAPOL_Packet           ( TI_HANDLE  hAdapter, tiVOID* pData, tiUINT32 uSize );
    tiINT32     TI_GetAssociationInfo       (TI_HANDLE  hAdapter, OS_802_11_ASSOCIATION_INFORMATION** ppInfo );

    tiINT32     TI_AddKey                   (TI_HANDLE  hAdapter, OS_802_11_KEY*        pKey    );
    tiINT32     TI_RemoveKey                (TI_HANDLE  hAdapter, OS_802_11_REMOVE_KEY* pRemoveKey);

/******************************************************************************

    Add support for EXC API functions

******************************************************************************/
  EXC_SUPPORT_H

/*****************************************************************************/



#ifdef __cplusplus
}
#endif

tiBOOL TI_CheckAdapterObject(void *pObj);

#endif /* _TI_ADAPTER_API_H*/

