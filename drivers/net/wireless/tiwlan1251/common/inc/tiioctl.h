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

#ifndef __TIIOCTL_H__
#define __TIIOCTL_H__

/* OID Mode*/
#define IOCTRL_SET    0
#define IOCTRL_GET    1
#define IOCTRL_SET_GET 2

#ifndef FILE_DEVICE_UNKNOWN

/**/
/* Macro definition for defining IOCTL and FSCTL function control codes.  Note*/
/* that function codes 0-2047 are reserved for Microsoft Corporation, and*/
/* 2048-4095 are reserved for customers.*/
/**/

#define CTL_CODE( DeviceType, Function, Method, Access ) (                 \
    ((DeviceType) << 16) | ((Access) << 14) | ((Function) << 2) | (Method) \
)

#define FILE_DEVICE_UNKNOWN     0x00000022
#define METHOD_BUFFERED         0
#define FILE_ANY_ACCESS         0

#endif     /* FILE_DEVICE_UNKNOWN */

/* IOCTL info, needs to be visible for application. Should be in a custom range (0x800..)*/
#define SHELLDRV_IOCTL_INDEX  0x00800

/* Offsets for IOTCLS bases*/
#define D11BASIC_IOCTLS_OFFSET          0x0
#define D11PACKET_PARAMS_IOCTLS_OFFSET  0x100
#define RATES_IOCTLS_OFFSET             0x200
#define CHANNEL_IOCTLS_OFFSET           0x300
#define POWER_IOCTLS_OFFSET             0x400
#define SECURITY_IOCTLS_OFFSET          0x500
#define MISC_IOCTLS_OFFSET              0x600
#define DEBUG_IOCTLS_OFFSET             0x700
#define SCAN_IOCTLS_OFFSET              0x800
#define VOICE_QOS_IOCTLS_OFFSET         0x900
#define ROAMING_IOCTLS_OFFSET           0xa00
#define MEASUREMENT_IOCTLS_OFFSET       0xa80
#define PLT_IOCTLS_OFFSET               0xb00

#ifdef _WINDOWS // Windows Mobile specific IOCTL's
#endif /* ifdef _WINDOWS */
/********************************************/

typedef struct tagDeviceInfo
{
char csKeyName[260];
char csDriverKey[260];
char csDescription[260];
} TIWLNDEVINFO, *PTIWLNDEVINFO;


/**********************  Basic dot11 Functionality ****************************************/

#define TIWLN_802_11_BSSID_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11BASIC_IOCTLS_OFFSET + 1, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_BSSID_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11BASIC_IOCTLS_OFFSET + 2, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)                            

#define TIWLN_802_11_BSSID_LIST CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11BASIC_IOCTLS_OFFSET + 3, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_SSID_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11BASIC_IOCTLS_OFFSET + 4, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_SSID_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11BASIC_IOCTLS_OFFSET + 5, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_DESIRED_SSID_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11BASIC_IOCTLS_OFFSET + 6, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_DISASSOCIATE CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11BASIC_IOCTLS_OFFSET + 7, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_ASSOCIATION_INFORMATION CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11BASIC_IOCTLS_OFFSET + 8, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_NETWORK_TYPES_SUPPORTED CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11BASIC_IOCTLS_OFFSET + 9, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_NETWORK_TYPE_IN_USE_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11BASIC_IOCTLS_OFFSET + 10, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_NETWORK_TYPE_IN_USE_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11BASIC_IOCTLS_OFFSET + 11, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_INFRASTRUCTURE_MODE_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11BASIC_IOCTLS_OFFSET + 12, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_INFRASTRUCTURE_MODE_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11BASIC_IOCTLS_OFFSET + 13, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_DESIRED_INFRASTRUCTURE_MODE CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11BASIC_IOCTLS_OFFSET + 14, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_AUTHENTICATION_MODE_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11BASIC_IOCTLS_OFFSET + 15, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_AUTHENTICATION_MODE_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11BASIC_IOCTLS_OFFSET + 16, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_CONFIGURATION_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11BASIC_IOCTLS_OFFSET + 17, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_CONFIGURATION_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11BASIC_IOCTLS_OFFSET + 18, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_FULL_BSSID_LIST  CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11BASIC_IOCTLS_OFFSET + 19, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)


/**********************  Rates Functionality  ****************************************/

#define TIWLN_802_11_SUPPORTED_RATES CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + RATES_IOCTLS_OFFSET + 1, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_DESIRED_RATES_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + RATES_IOCTLS_OFFSET + 2, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_SUPPORTED_RATES_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + RATES_IOCTLS_OFFSET + 3, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_CURRENT_RATES_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + RATES_IOCTLS_OFFSET + 4, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_EXT_RATES_IE_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + RATES_IOCTLS_OFFSET + 5, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_EXT_RATES_IE_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + RATES_IOCTLS_OFFSET + 6, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)


/**********************  Channel Functionality  ****************************************/

#define TIWLN_802_11_DESIRED_CHANNEL_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + CHANNEL_IOCTLS_OFFSET + 1, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_DESIRED_CHANNEL_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + CHANNEL_IOCTLS_OFFSET + 2, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_CHANNEL_GET   CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + CHANNEL_IOCTLS_OFFSET + 3, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_CURRENT_REGDOMAIN_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + CHANNEL_IOCTLS_OFFSET + 4, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_CURRENT_REGDOMAIN_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + CHANNEL_IOCTLS_OFFSET + 5, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_REGDOMAIN_TABLE   CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + CHANNEL_IOCTLS_OFFSET + 6, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_ROAM_PROFILE_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + CHANNEL_IOCTLS_OFFSET + 7, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_ROAM_PROFILE_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + CHANNEL_IOCTLS_OFFSET + 8, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_REG_DOMAIN_ENABLE_DISABLE_802_11D CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + CHANNEL_IOCTLS_OFFSET + 9, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_REG_DOMAIN_ENABLE_DISABLE_802_11H CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + CHANNEL_IOCTLS_OFFSET + 10, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_REG_DOMAIN_GET_802_11D CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + CHANNEL_IOCTLS_OFFSET + 11, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_REG_DOMAIN_GET_802_11H CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + CHANNEL_IOCTLS_OFFSET + 12, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_REG_DOMAIN_GET_COUNTRY_2_4 CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + CHANNEL_IOCTLS_OFFSET + 13, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_REG_DOMAIN_SET_COUNTRY_2_4 CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + CHANNEL_IOCTLS_OFFSET + 14, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_REG_DOMAIN_GET_COUNTRY_5 CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + CHANNEL_IOCTLS_OFFSET + 15, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_REG_DOMAIN_SET_COUNTRY_5 CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + CHANNEL_IOCTLS_OFFSET + 16, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_REG_DOMAIN_SET_DFS_RANGE CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + CHANNEL_IOCTLS_OFFSET + 17, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_REG_DOMAIN_GET_DFS_RANGE CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + CHANNEL_IOCTLS_OFFSET + 18, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

/**********************  Power Functionality  ****************************************/

#define TIWLN_802_11_POWER_MODE_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 1, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_POWER_MODE_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 2, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_TX_POWER_LEVEL_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 3, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_TX_POWER_DBM_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 4, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_NUMBER_OF_ANTENNAS CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 5, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_RX_ANTENNA_SELECTED_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 6, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_RX_ANTENNA_SELECTED_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 7, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_TX_ANTENNA_SELECTED_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 8, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_TX_ANTENNA_SELECTED_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 9, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLAN_802_11_ANTENNA_DIVERSITY_PARAM_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 10, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_RSSI CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 11, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_RSSI_TRIGGER_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 12, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_RSSI_TRIGGER_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 13, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_SLEEP_CMD CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 14, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_TX_POWER_DBM_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 15, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_POWER_MGR_PROFILE CTL_CODE(FILE_DEVICE_UNKNOWN, \
                                       SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 16, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_POWER_LEVEL_BOUNDARY CTL_CODE(FILE_DEVICE_UNKNOWN, \
                                       SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 17, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)


#define TIWLN_802_11_PERODIC_WAKEUP_MODE CTL_CODE(FILE_DEVICE_UNKNOWN, \
                                       SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 18, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_PERODIC_WAKEUP_TIMEOUT CTL_CODE(FILE_DEVICE_UNKNOWN, \
                                       SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 19, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_POWER_LEVEL_DEFAULT_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                                       SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 20, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_POWER_LEVEL_DEFAULT_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                                       SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 21, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_POWER_LEVEL_PS_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                                       SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 22, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_POWER_LEVEL_PS_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                                       SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 23, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_BEACON_FILTER_DESIRED_STATE_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                                            SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 24, \
                                            METHOD_BUFFERED,          \
                                            FILE_ANY_ACCESS)



#define TIWLN_802_11_SNR CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 25, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_BEACON_FILTER_DESIRED_STATE_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                                            SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 26, \
                                            METHOD_BUFFERED,          \
                                            FILE_ANY_ACCESS) 

#define TIWLN_802_11_POWER_LEVEL_DOZE_MODE_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                                       SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 27, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_POWER_LEVEL_DOZE_MODE_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                                       SHELLDRV_IOCTL_INDEX + POWER_IOCTLS_OFFSET + 28, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)


/**********************  dot11 Network Packet Parameters ****************************************/

#define TIWLN_802_11_FRAGMENTATION_THRESHOLD_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11PACKET_PARAMS_IOCTLS_OFFSET + 1, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_FRAGMENTATION_THRESHOLD_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11PACKET_PARAMS_IOCTLS_OFFSET + 2, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_RTS_THRESHOLD_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11PACKET_PARAMS_IOCTLS_OFFSET + 3, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_RTS_THRESHOLD_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11PACKET_PARAMS_IOCTLS_OFFSET + 4, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_SHORT_PREAMBLE_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11PACKET_PARAMS_IOCTLS_OFFSET + 5, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_SHORT_PREAMBLE_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11PACKET_PARAMS_IOCTLS_OFFSET + 6, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_SHORT_RETRY_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11PACKET_PARAMS_IOCTLS_OFFSET + 7, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_SHORT_RETRY_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11PACKET_PARAMS_IOCTLS_OFFSET + 8, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_LONG_RETRY_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11PACKET_PARAMS_IOCTLS_OFFSET + 9, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_LONG_RETRY_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11PACKET_PARAMS_IOCTLS_OFFSET + 11, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_SHORT_SLOT_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11PACKET_PARAMS_IOCTLS_OFFSET + 12, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_SHORT_SLOT_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11PACKET_PARAMS_IOCTLS_OFFSET + 13, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_IBSS_PROTECTION_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11PACKET_PARAMS_IOCTLS_OFFSET + 14, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_IBSS_PROTECTION_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + D11PACKET_PARAMS_IOCTLS_OFFSET + 15, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)


/************************** Security **************************************/

#define TIWLN_802_11_ADD_WEP CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 1, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_REMOVE_WEP CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 2, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_WEP_STATUS_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 3, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_WEP_STATUS_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 4, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_PRIVACY_FILTER_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 5, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_PRIVACY_FILTER_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 6, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)


#define TIWLN_802_11_ADD_KEY CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 13, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_REMOVE_KEY CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 14, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_OPEN_EAPOL_INTERFACE CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 15, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_CLOSE_EAPOL_INTERFACE CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 16, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_SEND_EAPOL_PACKET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 17, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_MIXED_MODE_SET CTL_CODE(FILE_DEVICE_UNKNOWN,   \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 18, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_MIXED_MODE_GET  CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 19, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_EXC_SECURITY_TYPE_SET  CTL_CODE(FILE_DEVICE_UNKNOWN,   \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 20, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_EXC_SECURITY_TYPE_GET  CTL_CODE(FILE_DEVICE_UNKNOWN,   \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 21, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

/* Supplicant use only*/
#define TIWLN_SUPPL_INIT CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 22, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)
/* Supplicant use only*/
#define TIWLN_SUPPL_TERMINATE CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 23, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)
/* Supplicant use only*/
#define TIWLN_802_11_PSK_SET         CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 24, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)
/* Supplicant use only*/
#define TIWLN_802_11_EAP_TYPE_SET      CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 25, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)
/* Supplicant use only*/
#define TIWLN_802_11_USER_ID_SET        CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 26, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)
/* Supplicant use only*/
#define TIWLN_802_11_USER_PASSWORD_SET        CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 27, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)
/* Supplicant use only*/
#define TIWLN_802_11_CERT_PARAMS_SHA1_SET    CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 28, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_EXC_ROGUE_AP_DETECTED    CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 29, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_EXC_REPORT_ROGUE_APS    CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 30, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_EXC_CCKM_REQUEST    CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 31, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_EXC_CCKM_RESULT    CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 32, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_PMKID_GET    CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 33, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)
/* Supplicant use only*/
#define TIWLN_802_11_KEY_TYPE_SET    CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 34, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_EXC_CONFIGURATION_SET   CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 35, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_EXC_CONFIGURATION_GET   CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 36, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_EXC_NETWORK_EAP_SET   CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 37, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_EXC_NETWORK_EAP_GET   CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 38, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)
/* Supplicant use only*/
#define TIWLN_802_11_CERT_PARAMS_FILE_NAME_SET   CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 39, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_EXC_AUTH_SUCCESS        CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 40, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_PMKID_SET    CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 41, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_WPA_OPTIONS_GET            CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 42, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)  
                                           
#define TIWLN_802_11_WPA_OPTIONS_SET            CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 43, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)                 

#define TIWLN_802_11_CAPABILITY_GET            CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 44, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS) 
                                            
#define TIWLN_802_11_AVAILABLE_OPTIONS_GET            CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 45, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS) 


#define TIWLN_802_11_EAP_TYPE_GET      CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 46, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_EAP_TYPE_DRIVER_SET      CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SECURITY_IOCTLS_OFFSET + 47, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

/************************** Misc **************************************/

#define TIWLN_IOCTL_OID_QUERY_INFORMATION CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 1, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_IOCTL_OID_SET_INFORMATION CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 2, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_MEDIUMUSAGE CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 3, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_STATISTICS CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 4, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_RELOAD_DEFAULTS CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 5, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_DRIVER_STATUS_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 6, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_DRIVER_STATUS_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 7, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_HW_READ_REGISTER CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 8, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_HW_WRITE_REGISTER CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 9, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_HW_RESET_HW CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 10, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_ENABLE_EVENT CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 11, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_DISABLE_EVENT CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 12, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_SET_INIT_INFO CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 13, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_3_CURRENT_ADDRESS CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 14, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_APIP_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 15, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_4XACTIVESTATE_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 16, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_GET_SW_VERSION CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 17, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_GET_EVENT_DATA CTL_CODE(FILE_DEVICE_UNKNOWN,    \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 18, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_DRIVER_SUSPEND CTL_CODE(FILE_DEVICE_UNKNOWN,    \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 19, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define SET_IPC_EVENT_HANDLE CTL_CODE(FILE_DEVICE_UNKNOWN,    \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 20, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define DESTROY_IPC_EVENT_HANDLE CTL_CODE(FILE_DEVICE_UNKNOWN,    \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 21, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_CONFIG_EVENTS_RSSI     CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 22, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_GET_DRIVERS_CAPABILITIES     CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 23, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_GET_SELECTED_BSSID_INFO     CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 24, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_GET_DRIVER_STATE CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 25, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)
        
#define BT_COEXSISTANCE_SET_ENABLE CTL_CODE(FILE_DEVICE_UNKNOWN,    \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 26, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define BT_COEXSISTANCE_SET_RATE   CTL_CODE(FILE_DEVICE_UNKNOWN,    \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 27, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define BT_COEXSISTANCE_SET_CONFIG CTL_CODE(FILE_DEVICE_UNKNOWN,    \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 28, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define BT_COEXSISTANCE_GET_STATUS CTL_CODE(FILE_DEVICE_UNKNOWN,    \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 29, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)


#define TIWLN_802_11_TX_STATISTICS CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 30, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define GWSI_DISPATCH_COMMAND CTL_CODE(FILE_DEVICE_UNKNOWN,    \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 31, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define GWSI_GET_INIT_TABLE_COMMAND CTL_CODE(FILE_DEVICE_UNKNOWN,    \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 32, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define GWSI_INITIALIZE_COMMAND CTL_CODE(FILE_DEVICE_UNKNOWN,    \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 33, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define GWSI_CONFIGURE_TABLE_COMMAND CTL_CODE(FILE_DEVICE_UNKNOWN,    \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 34, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define GWSI_RELEASE_COMMAND CTL_CODE(FILE_DEVICE_UNKNOWN,    \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 36, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define GWSI_DISPATCH_GET_CMD_LEN(_p_cmd)   (*((tiUINT16 *)_p_cmd + 1))

#define TIWLN_802_11_SET_TRAFFIC_INTENSITY_THRESHOLDS CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 37, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_GET_TRAFFIC_INTENSITY_THRESHOLDS CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 38, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_TOGGLE_TRAFFIC_INTENSITY_EVENTS CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 39, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_GET_PRIMARY_BSSID_INFO CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 40, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_EARLY_WAKEUP_IE_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 41, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_EARLY_WAKEUP_IE_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 42, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_ENABLE_DISABLE_RX_DATA_FILTERS CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 43, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_ADD_RX_DATA_FILTER CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 44, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_REMOVE_RX_DATA_FILTER CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 45, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_GET_RX_DATA_FILTERS_STATISTICS CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 46, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_GET_POWER_CONSUMPTION_STATISTICS CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MISC_IOCTLS_OFFSET + 47, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)


/************************** Debug **************************************/
                    
#ifdef TI_DBG


#define TIWLN_GET_DBG_BUFFER CTL_CODE(FILE_DEVICE_UNKNOWN,   \
                            SHELLDRV_IOCTL_INDEX + DEBUG_IOCTLS_OFFSET + 2, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_SET_MODULE CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                            SHELLDRV_IOCTL_INDEX + DEBUG_IOCTLS_OFFSET + 3, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_GET_MODULE CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                            SHELLDRV_IOCTL_INDEX + DEBUG_IOCTLS_OFFSET + 4, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_SET_DEBUG_FLAG CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                            SHELLDRV_IOCTL_INDEX + DEBUG_IOCTLS_OFFSET + 5, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_GET_DEBUG_FLAG CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                            SHELLDRV_IOCTL_INDEX + DEBUG_IOCTLS_OFFSET + 6, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_DISPLAY_STATS CTL_CODE(FILE_DEVICE_UNKNOWN,   \
                            SHELLDRV_IOCTL_INDEX + DEBUG_IOCTLS_OFFSET + 7, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_SET_SEVERITY CTL_CODE(FILE_DEVICE_UNKNOWN,    \
                            SHELLDRV_IOCTL_INDEX + DEBUG_IOCTLS_OFFSET + 8, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_GET_SEVERITY CTL_CODE(FILE_DEVICE_UNKNOWN,    \
                            SHELLDRV_IOCTL_INDEX + DEBUG_IOCTLS_OFFSET + 9, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_REPORT_MODULE_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + DEBUG_IOCTLS_OFFSET + 10, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_REPORT_MODULE_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + DEBUG_IOCTLS_OFFSET + 11, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_REPORT_SEVERITY_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + DEBUG_IOCTLS_OFFSET + 12, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_REPORT_SEVERITY_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + DEBUG_IOCTLS_OFFSET + 13, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)
#define TIWLN_DRIVER_DEBUG_PRINT CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + DEBUG_IOCTLS_OFFSET + 14, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_REPORT_PPMODE_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + DEBUG_IOCTLS_OFFSET + 15, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_OS_DBG_STATE_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + DEBUG_IOCTLS_OFFSET + 16, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_OS_DBG_STATE_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + DEBUG_IOCTLS_OFFSET + 17, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#ifdef DRIVER_PROFILING

#define TIWLAN_PROFILING_REPORT CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + DEBUG_IOCTLS_OFFSET + 18, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLAN_PROFILING_CPU_ESTIMATOR_CMD CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + DEBUG_IOCTLS_OFFSET + 19, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLAN_PROFILING_CPU_ESTIMATOR_CMD_START    0x01
#define TIWLAN_PROFILING_CPU_ESTIMATOR_CMD_STOP     0x02
#define TIWLAN_PROFILING_CPU_ESTIMATOR_CMD_RESET    0x03

#endif /* DRIVER_PROFILING */

#endif  /* TI_DBG*/


/************************************** Scan ********************************************/

#define TIWLN_802_11_START_APP_SCAN_SET     CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SCAN_IOCTLS_OFFSET + 1,\
                            METHOD_BUFFERED,                              \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_STOP_APP_SCAN_SET     CTL_CODE(FILE_DEVICE_UNKNOWN,  \
                            SHELLDRV_IOCTL_INDEX + SCAN_IOCTLS_OFFSET + 2,\
                            METHOD_BUFFERED,                              \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_SCAN_POLICY_PARAM_SET  CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SCAN_IOCTLS_OFFSET + 3,\
                            METHOD_BUFFERED,                              \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_SCAN_BSS_LIST_GET      CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + SCAN_IOCTLS_OFFSET + 4,\
                            METHOD_BUFFERED,                              \
                            FILE_ANY_ACCESS)

/*************************************** Voice & QOS ********************************************/

#define TIWLN_802_11_SET_QOS_PARAMS     CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 2, \
                                        METHOD_BUFFERED,          \
                                        FILE_ANY_ACCESS)

#define TIWLN_802_11_POLL_AP_PACKETS   CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 3, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_CONFIG_TX_CLASS   CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 4, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_REMOVE_CLSFR_ENTRY   CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 5, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_GET_CLSFR_TYPE CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 6, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_GET_AP_QOS_PARAMS CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 7, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_GET_AP_QOS_CAPABILITIES CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 8, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_ADD_TSPEC CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 9, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_GET_TSPEC_PARAMS CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 10, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_DELETE_TSPEC CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 11, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_GET_CURRENT_AC_STATUS CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 12, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_SET_MEDIUM_USAGE_THRESHOLD CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 13, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_SET_PHY_RATE_THRESHOLD CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 14, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_GET_MEDIUM_USAGE_THRESHOLD CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 15, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_GET_PHY_RATE_THRESHOLD CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 16, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)


#define TIWLN_802_11_GET_USER_PRIORITY_OF_STREAM CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 17, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_POLL_AP_PACKETS_FROM_AC   CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 18, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_GET_DESIRED_PS_MODE CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 19, \
                            METHOD_BUFFERED,          \
                            FILE_ANY_ACCESS)

#define TIWLN_802_11_SET_RX_TIMEOUT     CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 20, \
                                        METHOD_BUFFERED,          \
                                        FILE_ANY_ACCESS)
                                        
#define TIWLN_802_11_SET_DTAG_TO_AC_MAPPING_TABLE     CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 21, \
                                        METHOD_BUFFERED,          \
                                        FILE_ANY_ACCESS)
                                        
#define TIWLN_802_11_SET_VAD     CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 22, \
                                        METHOD_BUFFERED,          \
                                        FILE_ANY_ACCESS) 
 
#define TIWLN_802_11_GET_VAD     CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + VOICE_QOS_IOCTLS_OFFSET + 23, \
                                        METHOD_BUFFERED,          \
                                        FILE_ANY_ACCESS)                                                                                
                                                                               
/*****************************************************************************************************/
/***  Roaming Manager Configuration Parameters  ***/

#define TIWLN_802_11_ROAMING_CONFIG_PARAMS_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + ROAMING_IOCTLS_OFFSET + 1, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_802_11_ROAMING_CONFIG_PARAMS_GET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + ROAMING_IOCTLS_OFFSET + 2, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)


/*****************************************************************************************************/
/***  measurement Manager Configuration Parameters  ***/

#define TIWLN_802_11_MEASUREMENT_ENABLE_DISABLE_PARAMS_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MEASUREMENT_IOCTLS_OFFSET + 1, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)


#define TIWLN_802_11_MEASUREMENT_MAX_DURATION_PARAMS_SET CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + MEASUREMENT_IOCTLS_OFFSET + 2, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)





/*************************************** PLT ********************************************/

#define TIWLN_PLT_WRITE_REGISTER CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + PLT_IOCTLS_OFFSET + 1, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_PLT_READ_REGISTER CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + PLT_IOCTLS_OFFSET + 2, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_PLT_RX_PER_START CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + PLT_IOCTLS_OFFSET + 3, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_PLT_RX_PER_STOP CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + PLT_IOCTLS_OFFSET + 4, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_PLT_RX_PER_CLEAR CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + PLT_IOCTLS_OFFSET + 5, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_PLT_RX_PER_GET_RESULTS CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + PLT_IOCTLS_OFFSET + 6, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_PLT_TX_CW CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + PLT_IOCTLS_OFFSET + 7, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_PLT_TX_CONTINUES CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + PLT_IOCTLS_OFFSET + 8, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_PLT_TX_STOP CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + PLT_IOCTLS_OFFSET + 9, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_PLT_MIB_WRITE CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + PLT_IOCTLS_OFFSET + 10, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_PLT_MIB_READ CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + PLT_IOCTLS_OFFSET + 11, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_PLT_RX_TX_CAL CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + PLT_IOCTLS_OFFSET + 12, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_PLT_RX_CAL CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + PLT_IOCTLS_OFFSET + 13, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#define TIWLN_PLT_RX_CAL_RESULT CTL_CODE(FILE_DEVICE_UNKNOWN, \
                            SHELLDRV_IOCTL_INDEX + PLT_IOCTLS_OFFSET + 14, \
                                       METHOD_BUFFERED,          \
                                       FILE_ANY_ACCESS)

#ifdef _WINDOWS
#endif /* ifdef _WINDOWS */

#endif




