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

#ifndef __PARAM_MGR_H__
#define __PARAM_MGR_H__

 /* This file contains the definitions for the parameters that can be Set/Get from outside.
    The parmeters that can be Set/Get from inside the driver only are defined in the file paramIn.h */

/****************************************************************************
                                PARAMETERS ISSUE
    Each parameter in the system is defined as UINT32. The parameter
    structue is as following:

 bit   31   30 - 24     23    22 - 16    15 - 8       7 - 0
    +-----+----------+-----+----------+-----------+-----------+
    | Set | Reserved | Get | Reserved | Module    | Parameter |
    | bit |          | bit |          | number    | number    |
    +-----+----------+-----+----------+-----------+-----------+

  The 'set' bit indicates whteher this parameter can be set from OS abstraction layer.
  The 'get' bit indicates whteher this parameter can be get from OS abstraction layer.
  (All the parameters can be Get/Set from insied the driver.)
  The module number indicated who is the oner of the parameter.
  The parameter number is the parameter unique number used to identify it.

****************************************************************************/

#define EXTERNAL_SET_ENABLE(x) (x & 0x80000000)
#define EXTERNAL_GET_ENABLE(x) (x & 0x00800000)
#define GET_PARAM_MODULE_NUMBER(x) ((x & 0x0000FF00) >> 8)
#ifdef GWSI_LIB
/* In some compilers the macro definition defaults to int, and 0x80000000 exceeds that limit */
#define     SET_BIT         ((int)0x80000000)
#else
#define     SET_BIT         0x80000000
#endif
#define     GET_BIT         0x00800000

 /* Following are the modules numbers */
/* NOTICE! whenever you add a module, you have to increment MAX_PARAM_MODULE_NUMBER as well!!! */
typedef enum
{
    AUTH_MODULE_PARAM               = 0x0100,
    ASSOC_MODULE_PARAM              = 0x0200,
    RX_DATA_MODULE_PARAM            = 0x0300,
    TX_DATA_MODULE_PARAM            = 0x0400,
    CTRL_DATA_MODULE_PARAM          = 0x0500,
    SITE_MGR_MODULE_PARAM           = 0x0600,
    CONN_MODULE_PARAM               = 0x0700,
    RSN_MODULE_PARAM                = 0x0800,
    ADM_CTRL_MODULE_PARAM           = 0x0900,
    HAL_CTRL_MODULE_PARAM           = 0x0A00,
    REPORT_MODULE_PARAM             = 0x0B00,
    SME_SM_MODULE_PARAM             = 0x0C00,
    MLME_SM_MODULE_PARAM            = 0x0D00,
    REGULATORY_DOMAIN_MODULE_PARAM  = 0x0E00,
    MEASUREMENT_MODULE_PARAM        = 0x0F00,
    EXC_MANAGER_MODULE_PARAM        = 0x1000,
    ROAMING_MANAGER_MODULE_PARAM    = 0x1100,
    SOFT_GEMINI_PARAM               = 0x1200,
    QOS_MANAGER_PARAM               = 0x1300,
    POWER_MANAGER_PARAM             = 0x1400,
    SCAN_CNCN_PARAM                 = 0x1500,
    SCAN_MNGR_PARAM                 = 0x1600,

    /*
    Last module - DO NOT TOUCH!
    */
    MODULE_PARAM_LAST_MODULE

}   moduleParam_e;

enum
{
    /*
    the MAX_PARAM_MODULE_PARAM is the module param last module +1 therefore there is a need
    to -1 to get to real last module number.
    */
    MAX_PARAM_MODULE_PARAM = MODULE_PARAM_LAST_MODULE - 1
};

/* Following are the parameters numbers. Each module can have 256 parameters */
typedef enum
{
    /* Driver General section */
    DRIVER_STATUS_PARAM                         =           GET_BIT                         | 0x00,

    /* HAL Control section */
    HAL_CTRL_RTS_THRESHOLD_PARAM                = SET_BIT | GET_BIT | HAL_CTRL_MODULE_PARAM | 0x01,
    HAL_CTRL_FRAG_THRESHOLD_PARAM               = SET_BIT | GET_BIT | HAL_CTRL_MODULE_PARAM | 0x02,
    HAL_CTRL_COUNTERS_PARAM                     =           GET_BIT | HAL_CTRL_MODULE_PARAM | 0x03,
    HAL_CTRL_LISTEN_INTERVAL_PARAM              = SET_BIT | GET_BIT | HAL_CTRL_MODULE_PARAM | 0x04,
    HAL_CTRL_CURRENT_BEACON_INTERVAL_PARAM      =           GET_BIT | HAL_CTRL_MODULE_PARAM | 0x05,
    HAL_CTRL_TX_POWER_PARAM                     = SET_BIT | GET_BIT | HAL_CTRL_MODULE_PARAM | 0x06,
    HAL_CTRL_TX_ANTENNA_PARAM                   = SET_BIT | GET_BIT | HAL_CTRL_MODULE_PARAM | 0x07,
    HAL_CTRL_RX_ANTENNA_PARAM                   = SET_BIT | GET_BIT | HAL_CTRL_MODULE_PARAM | 0x08,
    HAL_CTRL_MIN_POWER_LEVEL          = SET_BIT |           HAL_CTRL_MODULE_PARAM | 0x09,
    HAL_CTRL_CLK_RUN_ENABLE                     = SET_BIT |           HAL_CTRL_MODULE_PARAM | 0x0A,
    HAL_CTRL_QUEUES_PARAMS                      = SET_BIT |           HAL_CTRL_MODULE_PARAM | 0x0B, 
    HAL_CTRL_AC_PARAMS                          = SET_BIT |           HAL_CTRL_MODULE_PARAM | 0x0C, 
    HAL_CTRL_TX_RATE_CLASS_PARAMS               = SET_BIT | GET_BIT | HAL_CTRL_MODULE_PARAM | 0x0D,
    HAL_CTRL_DOT11_MAX_TX_MSDU_LIFE_TIME        = SET_BIT | GET_BIT | HAL_CTRL_MODULE_PARAM | 0x0E,
    HAL_CTRL_DOT11_MAX_RX_MSDU_LIFE_TIME        = SET_BIT | GET_BIT | HAL_CTRL_MODULE_PARAM | 0x0F,
    HAL_CTRL_PS_POLL_GENERATION_MODE            = SET_BIT | GET_BIT | HAL_CTRL_MODULE_PARAM | 0x10,
    HAL_CTRL_CTS_TO_SELF_PARAM                  = SET_BIT | GET_BIT | HAL_CTRL_MODULE_PARAM | 0x11,
    HAL_CTRL_TX_ACK_POLICY                      = SET_BIT | GET_BIT | HAL_CTRL_MODULE_PARAM | 0x12,

    HAL_CTRL_TX_COUNTERS_PARAM                  =           GET_BIT | HAL_CTRL_MODULE_PARAM | 0x14,
    HAL_CTRL_RX_TIME_OUT_PARAM                  = SET_BIT |           HAL_CTRL_MODULE_PARAM | 0x15,

    HAL_CTRL_ANTENNA_DIVERSITY_PARAMS           = SET_BIT           | HAL_CTRL_MODULE_PARAM | 0x18,
    HAL_CTRL_CURRENT_CHANNEL                    =           GET_BIT | HAL_CTRL_MODULE_PARAM | 0x19,
    HAL_CTRL_RSSI_LEVEL_PARAM                   =           GET_BIT | HAL_CTRL_MODULE_PARAM | 0x1a,
    HAL_CTRL_SNR_RATIO_PARAM                    =           GET_BIT | HAL_CTRL_MODULE_PARAM | 0x1b,
    HAL_CTRL_BCN_BRC_OPTIONS                    =           GET_BIT | HAL_CTRL_MODULE_PARAM | 0x1c,
    
   /* PLT params */    
    HAL_CTRL_PLT_READ_REGISTER                  =           GET_BIT | HAL_CTRL_MODULE_PARAM | 0x1d,
    HAL_CTRL_PLT_WRITE_REGISTER                 = SET_BIT           | HAL_CTRL_MODULE_PARAM | 0x1e,

    HAL_CTRL_PLT_RX_PER_START                   = SET_BIT           | HAL_CTRL_MODULE_PARAM | 0x1f,
    HAL_CTRL_PLT_RX_PER_STOP                    = SET_BIT           | HAL_CTRL_MODULE_PARAM | 0x20,
    HAL_CTRL_PLT_RX_PER_CLEAR                   = SET_BIT           | HAL_CTRL_MODULE_PARAM | 0x21,
    HAL_CTRL_PLT_RX_PER_GET_RESULTS             =           GET_BIT | HAL_CTRL_MODULE_PARAM | 0x22,
    HAL_CTRL_PLT_TX_CW                          = SET_BIT           | HAL_CTRL_MODULE_PARAM | 0x23,
    HAL_CTRL_PLT_TX_CONTINUES                   = SET_BIT           | HAL_CTRL_MODULE_PARAM | 0x24,
    HAL_CTRL_PLT_TX_STOP                        = SET_BIT           | HAL_CTRL_MODULE_PARAM | 0x25,
    HAL_CTRL_PLT_WRITE_MIB                      = SET_BIT           | HAL_CTRL_MODULE_PARAM | 0x26,
    HAL_CTRL_PLT_READ_MIB                       =           GET_BIT | HAL_CTRL_MODULE_PARAM | 0x27,

    HAL_CTRL_PLT_RX_TX_CAL                      =           GET_BIT | HAL_CTRL_MODULE_PARAM | 0x28,
    HAL_CTRL_PLT_RX_CAL_STATUS                  =           GET_BIT | HAL_CTRL_MODULE_PARAM | 0x29,

    /* misc section */
    HAL_CTRL_EARLY_WAKEUP                       = SET_BIT | GET_BIT | HAL_CTRL_MODULE_PARAM | 0x30,
    HAL_CTRL_POWER_CONSUMPTION                  =           GET_BIT | HAL_CTRL_MODULE_PARAM | 0x31,


    /* Site manager section */
    SITE_MGR_DESIRED_CHANNEL_PARAM              = SET_BIT | GET_BIT | SITE_MGR_MODULE_PARAM | 0x01,
    SITE_MGR_DESIRED_BSSID_PARAM                = SET_BIT | GET_BIT | SITE_MGR_MODULE_PARAM | 0x02,
    SITE_MGR_DESIRED_SSID_PARAM                 = SET_BIT | GET_BIT | SITE_MGR_MODULE_PARAM | 0x03,
    SITE_MGR_DESIRED_BSS_TYPE_PARAM             = SET_BIT | GET_BIT | SITE_MGR_MODULE_PARAM | 0x04,
    SITE_MGR_DESIRED_SUPPORTED_RATE_SET_PARAM   = SET_BIT | GET_BIT | SITE_MGR_MODULE_PARAM | 0x05,
    SITE_MGR_DESIRED_TX_RATE_PARAM              =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x07,
    SITE_MGR_DESIRED_MODULATION_TYPE_PARAM      = SET_BIT | GET_BIT | SITE_MGR_MODULE_PARAM | 0x08,
    SITE_MGR_DESIRED_BEACON_INTERVAL_PARAM      = SET_BIT | GET_BIT | SITE_MGR_MODULE_PARAM | 0x09,
    SITE_MGR_DESIRED_PREAMBLE_TYPE_PARAM        = SET_BIT | GET_BIT | SITE_MGR_MODULE_PARAM | 0x0A,

    SITE_MGR_CURRENT_RADIO_TYPE_PARAM           =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x0D,
    SITE_MGR_CURRENT_CHANNEL_PARAM              = SET_BIT | GET_BIT | SITE_MGR_MODULE_PARAM | 0x0E,
    SITE_MGR_CURRENT_SSID_PARAM                 =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x0F,
    SITE_MGR_CURRENT_RATE_PAIR_PARAM            =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x10,
    SITE_MGR_CURRENT_MODULATION_TYPE_PARAM      =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x11,
    SITE_MGR_CURRENT_SIGNAL_PARAM               = SET_BIT | GET_BIT | SITE_MGR_MODULE_PARAM | 0x12,
    SITE_MGR_BSSID_LIST_PARAM                   =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x13,
    SITE_MGR_TI_WLAN_COUNTERS_PARAM             =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x14,
    SITE_MGR_PRIMARY_SITE_PARAM                 =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x15,
    SITE_MGR_EEPROM_VERSION_PARAM               =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x16,
    SITE_MGR_FIRMWARE_VERSION_PARAM             =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x17,
    SITE_MGR_DESIRED_DOT11_MODE_PARAM           = SET_BIT | GET_BIT | SITE_MGR_MODULE_PARAM | 0x18,
    SITE_MGR_OPERATIONAL_MODE_PARAM             =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x19,
    SITE_MGR_USE_DRAFT_NUM_PARAM                = SET_BIT | GET_BIT | SITE_MGR_MODULE_PARAM | 0x1A,
    SITE_MGR_DESIRED_SLOT_TIME_PARAM            = SET_BIT | GET_BIT | SITE_MGR_MODULE_PARAM | 0x1B,
    SITE_MGR_CURRENT_SLOT_TIME_PARAM            =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x1C,
    SITE_MGR_CURRENT_PREAMBLE_TYPE_PARAM        =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x1D,
    SITE_MGR_BUILT_IN_TEST_STATUS_PARAM         =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x1E,
    SITE_MGR_CONFIGURATION_PARAM                = SET_BIT | GET_BIT | SITE_MGR_MODULE_PARAM | 0x1F,
    SITE_MGR_DISASSOCIATE_PARAM                 = SET_BIT           | SITE_MGR_MODULE_PARAM | 0x20,
    SITE_MGR_DEAUTHENTICATE_PARAM               = SET_BIT           | SITE_MGR_MODULE_PARAM | 0x21,
    SITE_MGR_BSSID_LIST_SCAN_PARAM              = SET_BIT           | SITE_MGR_MODULE_PARAM | 0x22,
    SITE_MGR_AP_TX_POWER_PARAM                  =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x23,
    SITE_MGR_DESIRED_TX_RATE_PRCT_SET           = SET_BIT           | SITE_MGR_MODULE_PARAM | 0x26,
    SITE_MGR_DESIRED_RSSI_THRESHOLD_SET         = SET_BIT           | SITE_MGR_MODULE_PARAM | 0x27,

    SITE_MGR_SITE_ENTRY_BY_INDEX                =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x28,
    SITE_MGR_CUR_NUM_OF_SITES                   =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x29,
    SITE_MGR_CURRENT_TSF_TIME_STAMP             =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x2A,    
    SITE_MGR_GET_SELECTED_BSSID_INFO            =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x2B,
    SITE_MGR_DESIRED_CONS_TX_ERRORS_THREH       = SET_BIT | GET_BIT | SITE_MGR_MODULE_PARAM | 0x2C,
    SITE_MGR_SUPPORTED_NETWORK_TYPES            =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x2D,
    SITE_MGR_GET_AP_QOS_CAPABILITIES            =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x2E,
    SITE_MGR_CURRENT_BSSID_PARAM                =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x2F,
    SITE_MGR_LAST_RX_RATE_PARAM                 =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x30,
    SITE_MGR_LAST_BEACON_BUF_PARAM              =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x31,
    SITE_MGR_CURRENT_TX_RATE_PARAM              =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x32,
    SITE_MGR_CURRENT_BSS_TYPE_PARAM             =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x33,
    SITE_MGR_BSSID_FULL_LIST_PARAM              =           GET_BIT | SITE_MGR_MODULE_PARAM | 0x34,
    SITE_MGR_BEACON_FILTER_DESIRED_STATE_PARAM  = SET_BIT | GET_BIT | SITE_MGR_MODULE_PARAM | 0x35,
    SITE_MGR_ALLOW_TX_POWER_CHECK               = SET_BIT | GET_BIT | SITE_MGR_MODULE_PARAM | 0x36,
    SITE_MGR_NETWORK_TYPE_IN_USE				=           GET_BIT | SITE_MGR_MODULE_PARAM | 0x37,

    /* MLME section */
    MLME_BEACON_RECV                            =           GET_BIT | MLME_SM_MODULE_PARAM  | 0x01,

    /* SME SM section */
    SITE_MGR_CONNECTION_STATUS_PARAM            =           GET_BIT | SME_SM_MODULE_PARAM   | 0x01,
    SME_SM_STATE_PARAM                          =           GET_BIT | SME_SM_MODULE_PARAM   | 0x02,
    SME_SCAN_ENABLED_PARAM                      = SET_BIT | GET_BIT | SME_SM_MODULE_PARAM   | 0x03,
    
    /* Scan concentrator section */
    SCAN_CNCN_START_APP_SCAN                    = SET_BIT |           SCAN_CNCN_PARAM       | 0x01,
    SCAN_CNCN_STOP_APP_SCAN                     = SET_BIT |           SCAN_CNCN_PARAM       | 0x02,
    SCAN_CNCN_BSSID_LIST_SCAN_PARAM             = SET_BIT           | SCAN_CNCN_PARAM       | 0x03,

    /* Scan Manager module */
    SCAN_MNGR_SET_CONFIGURATION                 = SET_BIT |           SCAN_MNGR_PARAM       | 0x01,
    SCAN_MNGR_BSS_LIST_GET                      =           GET_BIT | SCAN_MNGR_PARAM       | 0x02,

    /* Connection section */
    CONN_SELF_TIMEOUT_PARAM                     = SET_BIT | GET_BIT | CONN_MODULE_PARAM | 0x01,

    /* Auth section */
    AUTH_RESPONSE_TIMEOUT_PARAM                 = SET_BIT | GET_BIT | AUTH_MODULE_PARAM | 0x01,
    AUTH_COUNTERS_PARAM                         =           GET_BIT | AUTH_MODULE_PARAM | 0x02,

    /* Assoc section */
    ASSOC_RESPONSE_TIMEOUT_PARAM                = SET_BIT | GET_BIT | ASSOC_MODULE_PARAM | 0x01,
    ASSOC_COUNTERS_PARAM                        =           GET_BIT | ASSOC_MODULE_PARAM | 0x02,
    ASSOC_ASSOCIATION_INFORMATION_PARAM         =           GET_BIT | ASSOC_MODULE_PARAM | 0x03,
    ASSOC_ASSOCIATION_RESP_PARAM                =           GET_BIT | ASSOC_MODULE_PARAM | 0x04,

    /* RSN section */
    RSN_PRIVACY_OPTION_IMPLEMENTED_PARAM        =           GET_BIT | RSN_MODULE_PARAM | 0x01,
    RSN_KEY_PARAM                               = SET_BIT | GET_BIT | RSN_MODULE_PARAM | 0x02,
    RSN_SECURITY_STATE_PARAM                    =           GET_BIT | RSN_MODULE_PARAM | 0x03,
    RSN_ENCRYPTION_STATUS_PARAM                 = SET_BIT | GET_BIT | RSN_MODULE_PARAM | 0x04,
    RSN_ADD_KEY_PARAM                           = SET_BIT | GET_BIT | RSN_MODULE_PARAM | 0x05,
    RSN_REMOVE_KEY_PARAM                        = SET_BIT           | RSN_MODULE_PARAM | 0x06,
    RSN_EXT_AUTHENTICATION_MODE                 = SET_BIT | GET_BIT | RSN_MODULE_PARAM | 0x07,
    RSN_MIXED_MODE                              = SET_BIT | GET_BIT | RSN_MODULE_PARAM | 0x08,
    RSN_DEFAULT_KEY_ID                          = SET_BIT | GET_BIT | RSN_MODULE_PARAM | 0x09,
    RSN_EXC_NETWORK_EAP                         = SET_BIT | GET_BIT | RSN_MODULE_PARAM | 0x0A,
    RSN_AUTH_ENCR_CAPABILITY                    =           GET_BIT | RSN_MODULE_PARAM | 0x11,
    RSN_PMKID_LIST                              = SET_BIT | GET_BIT | RSN_MODULE_PARAM | 0x12,
    RSN_WPA_PROMOTE_AVAILABLE_OPTIONS           =           GET_BIT | RSN_MODULE_PARAM | 0x13,
    RSN_WPA_PROMOTE_OPTIONS                     = SET_BIT | GET_BIT | RSN_MODULE_PARAM | 0x14,
    RSN_PRE_AUTH_STATUS                         =           GET_BIT | RSN_MODULE_PARAM | 0x15,
    RSN_EAP_TYPE                                = SET_BIT | GET_BIT | RSN_MODULE_PARAM | 0x16,
    WPA_801_1X_AKM_EXISTS                       =           GET_BIT | RSN_MODULE_PARAM | 0x17,


    /* RX data section */
    RX_DATA_COUNTERS_PARAM                      =           GET_BIT | RX_DATA_MODULE_PARAM | 0x01,
    RX_DATA_EXCLUDE_UNENCRYPTED_PARAM           = SET_BIT | GET_BIT | RX_DATA_MODULE_PARAM | 0x02,
    RX_DATA_EXCLUDE_BROADCAST_UNENCRYPTED_PARAM = SET_BIT | GET_BIT | RX_DATA_MODULE_PARAM | 0x03,
    RX_DATA_ENABLE_DISABLE_RX_DATA_FILTERS      = SET_BIT | GET_BIT | RX_DATA_MODULE_PARAM | 0x04,
    RX_DATA_ADD_RX_DATA_FILTER                  = SET_BIT           | RX_DATA_MODULE_PARAM | 0x05,
    RX_DATA_REMOVE_RX_DATA_FILTER               = SET_BIT           | RX_DATA_MODULE_PARAM | 0x06,
    RX_DATA_GET_RX_DATA_FILTERS_STATISTICS      =           GET_BIT | RX_DATA_MODULE_PARAM | 0x07,


    /* TX data section */
    TX_DATA_PORT_STATUS_PARAM                   =           GET_BIT | TX_DATA_MODULE_PARAM | 0x01,
    TX_DATA_COUNTERS_PARAM                      =           GET_BIT | TX_DATA_MODULE_PARAM | 0x02,
    TX_DATA_RESET_COUNTERS_PARAM                = SET_BIT           | TX_DATA_MODULE_PARAM | 0x03,
    TX_DATA_ENCRYPTION_FIELD_SIZE               = SET_BIT           | TX_DATA_MODULE_PARAM | 0x04,
    TX_DATA_PS_MODE_PARAM                       = SET_BIT           | TX_DATA_MODULE_PARAM | 0x05,
    TX_DATA_CONFIG_TX_QUEUE_SIZE                = SET_BIT           | TX_DATA_MODULE_PARAM | 0x07,
    TX_DATA_CONVERT_HEADER_MODE                 = SET_BIT | GET_BIT | TX_DATA_MODULE_PARAM | 0x08,
    TX_DATA_CONVERT_TAG_ZERO_HEADER_MODE        = SET_BIT           | TX_DATA_MODULE_PARAM | 0x09,
    TX_DATA_TAG_TO_AC_CLASSIFIER_TABLE          = SET_BIT           | TX_DATA_MODULE_PARAM | 0x0A,
    TX_DATA_PS_STATUS                           = SET_BIT           | TX_DATA_MODULE_PARAM | 0x0B,
    TX_DATA_SET_AC_QUEUE_INDEX                  = SET_BIT           | TX_DATA_MODULE_PARAM | 0x0D,
    TX_DATA_CONFIG_TX_QUEUE_OVFLOW_POLICY       = SET_BIT           | TX_DATA_MODULE_PARAM | 0x0E,
    TX_DATA_CONFIG_AC_MSDU_LIFE_TIME            = SET_BIT           | TX_DATA_MODULE_PARAM | 0x0F,
    TX_DATA_CONFIG_AC_ACK_POLICY                = SET_BIT           | TX_DATA_MODULE_PARAM | 0x10,
    TX_DATA_AC_ADMISSION_STATE                  = SET_BIT           | TX_DATA_MODULE_PARAM | 0x11,
    TX_DATA_SET_MEDIUM_USAGE_THRESHOLD          = SET_BIT           | TX_DATA_MODULE_PARAM | 0x12,
    TX_DATA_GET_MEDIUM_USAGE_THRESHOLD          = SET_BIT | GET_BIT | TX_DATA_MODULE_PARAM | 0x13,
    TX_DATA_POLL_AP_PACKETS_FROM_AC             = SET_BIT           | TX_DATA_MODULE_PARAM | 0x14,
    TX_DATA_REPORT_TS_STATISTICS                =           GET_BIT | TX_DATA_MODULE_PARAM | 0x15,
    TX_DATA_SET_VAD                             = SET_BIT           | TX_DATA_MODULE_PARAM | 0x16,
    TX_DATA_GET_VAD                             =           GET_BIT | TX_DATA_MODULE_PARAM | 0x17,

    /* CTRL data section */
    CTRL_DATA_COUNTERS_PARAM                    =           GET_BIT | CTRL_DATA_MODULE_PARAM | 0x01,
    CTRL_DATA_RATE_CONTROL_ENABLE_PARAM         = SET_BIT | GET_BIT | CTRL_DATA_MODULE_PARAM | 0x02,
    CTRL_DATA_CURRENT_BSSID_PARAM               =           GET_BIT | CTRL_DATA_MODULE_PARAM | 0x03,
    CTRL_DATA_CURRENT_BSS_TYPE_PARAM            =           GET_BIT | CTRL_DATA_MODULE_PARAM | 0x04,
    CTRL_DATA_CURRENT_SUPPORTED_RATE_MASK_PARAM =           GET_BIT | CTRL_DATA_MODULE_PARAM | 0x05,
    CTRL_DATA_CURRENT_PREAMBLE_TYPE_PARAM       =           GET_BIT | CTRL_DATA_MODULE_PARAM | 0x06,
    CTRL_DATA_CURRENT_PROTECTION_STATUS_PARAM   = SET_BIT | GET_BIT | CTRL_DATA_MODULE_PARAM | 0x07,
    CTRL_DATA_MAC_ADDRESS                       = SET_BIT | GET_BIT | CTRL_DATA_MODULE_PARAM | 0x08,
    CTRL_DATA_CURRENT_IBSS_PROTECTION_PARAM     = SET_BIT | GET_BIT | CTRL_DATA_MODULE_PARAM | 0x09,
    CTRL_DATA_CURRENT_RTS_CTS_STATUS_PARAM      = SET_BIT | GET_BIT | CTRL_DATA_MODULE_PARAM | 0x0A,
    CTRL_DATA_FOUR_X_ENABLE_PARAM               = SET_BIT | GET_BIT | CTRL_DATA_MODULE_PARAM | 0x0B,
    CTRL_DATA_FOUR_X_CURRRENT_STATUS_PARAM      = SET_BIT | GET_BIT | CTRL_DATA_MODULE_PARAM | 0x0C,
    CTRL_DATA_CLSFR_TYPE                        =           GET_BIT | CTRL_DATA_MODULE_PARAM | 0x0D,
    CTRL_DATA_CLSFR_CONFIG                      = SET_BIT           | CTRL_DATA_MODULE_PARAM | 0x0E,
    CTRL_DATA_CLSFR_REMOVE_ENTRY                = SET_BIT           | CTRL_DATA_MODULE_PARAM | 0x0F,
    CTRL_DATA_GET_USER_PRIORITY_OF_STREAM       = SET_BIT | GET_BIT | CTRL_DATA_MODULE_PARAM | 0x10,
    CTRL_DATA_SHORT_RETRY_LIMIT_PARAM           = SET_BIT | GET_BIT | CTRL_DATA_MODULE_PARAM | 0x11,
    CTRL_DATA_LONG_RETRY_LIMIT_PARAM            = SET_BIT | GET_BIT | CTRL_DATA_MODULE_PARAM | 0x12,
    CTRL_DATA_CURRENT_RATE_CLASS_CLIENT         = SET_BIT | GET_BIT | CTRL_DATA_MODULE_PARAM | 0x13,
    CTRL_DATA_NEXT_RATE_MASK_FOR_CLIENT         = SET_BIT | GET_BIT | CTRL_DATA_MODULE_PARAM | 0x14,

    CTRL_DATA_TRAFFIC_INTENSITY_THRESHOLD       = SET_BIT | GET_BIT | CTRL_DATA_MODULE_PARAM | 0x15,
    CTRL_DATA_TOGGLE_TRAFFIC_INTENSITY_EVENTS   = SET_BIT           | CTRL_DATA_MODULE_PARAM | 0x16,
    CTRL_DATA_TSRS_PARAM                        = SET_BIT           | CTRL_DATA_MODULE_PARAM | 0x17,

    /* REPORT section */
    REPORT_MODULE_ON_PARAM                      = SET_BIT | GET_BIT | REPORT_MODULE_PARAM | 0x01,
    REPORT_MODULE_OFF_PARAM                     = SET_BIT | GET_BIT | REPORT_MODULE_PARAM | 0x02,
    REPORT_MODULE_TABLE_PARAM                   = SET_BIT | GET_BIT | REPORT_MODULE_PARAM | 0x03,
    REPORT_SEVERITY_ON_PARAM                    = SET_BIT | GET_BIT | REPORT_MODULE_PARAM | 0x04,
    REPORT_SEVERITY_OFF_PARAM                   = SET_BIT | GET_BIT | REPORT_MODULE_PARAM | 0x05,
    REPORT_SEVERITY_TABLE_PARAM                 = SET_BIT | GET_BIT | REPORT_MODULE_PARAM | 0x06,
    REPORT_PPMODE_VALUE_PARAM                   = SET_BIT | GET_BIT | REPORT_MODULE_PARAM | 0x07,
    REPORT_OS_DBG_STATE_VALUE_PARAM             = SET_BIT | GET_BIT | REPORT_MODULE_PARAM | 0x08,

    /* regulatory domain section */
    REGULATORY_DOMAIN_MANAGEMENT_CAPABILITY_ENABLED_PARAM   =           GET_BIT | REGULATORY_DOMAIN_MODULE_PARAM | 0x02,
    REGULATORY_DOMAIN_ENABLED_PARAM                         =           GET_BIT | REGULATORY_DOMAIN_MODULE_PARAM | 0x03,
    REGULATORY_DOMAIN_CURRENT_REGULATORY_DOMAIN_PARAM       =           GET_BIT | REGULATORY_DOMAIN_MODULE_PARAM | 0x04,
    REGULATORY_DOMAIN_TX_POWER_LEVEL_TABLE_PARAM            =			GET_BIT | REGULATORY_DOMAIN_MODULE_PARAM | 0x07,
    REGULATORY_DOMAIN_CURRENT_TX_POWER_IN_DBM_PARAM         = SET_BIT | GET_BIT | REGULATORY_DOMAIN_MODULE_PARAM | 0x08,
    REGULATORY_DOMAIN_UPDATE_CHANNEL_VALIDITY               = SET_BIT |           REGULATORY_DOMAIN_MODULE_PARAM | 0x09,
    REGULATORY_DOMAIN_TEMPORARY_TX_ATTENUATION_PARAM        = SET_BIT |           REGULATORY_DOMAIN_MODULE_PARAM | 0x0B,
    REGULATORY_DOMAIN_ENABLE_DISABLE_802_11D                = SET_BIT |           REGULATORY_DOMAIN_MODULE_PARAM | 0x0C,
    REGULATORY_DOMAIN_ENABLE_DISABLE_802_11H                = SET_BIT |           REGULATORY_DOMAIN_MODULE_PARAM | 0x0D,
    REGULATORY_DOMAIN_COUNTRY_2_4_PARAM                     = SET_BIT | GET_BIT | REGULATORY_DOMAIN_MODULE_PARAM | 0x0E,
    REGULATORY_DOMAIN_COUNTRY_5_PARAM                       = SET_BIT | GET_BIT | REGULATORY_DOMAIN_MODULE_PARAM | 0x0F,
    REGULATORY_DOMAIN_DFS_CHANNELS_RANGE                    = SET_BIT | GET_BIT | REGULATORY_DOMAIN_MODULE_PARAM | 0x10,


    /* measurement section */
    MEASUREMENT_ENABLE_DISABLE_PARAM                        = SET_BIT |          MEASUREMENT_MODULE_PARAM | 0x01,
    MEASUREMENT_MAX_DURATION_PARAM                          = SET_BIT |          MEASUREMENT_MODULE_PARAM | 0x02,

#ifdef EXC_MODULE_INCLUDED
    /* EXC */
    
    EXC_CONFIGURATION                                   = SET_BIT | GET_BIT | EXC_MANAGER_MODULE_PARAM | 0x01,
    EXC_ROGUE_AP_DETECTED                               = SET_BIT           | EXC_MANAGER_MODULE_PARAM | 0x02,
    EXC_REPORT_ROGUE_APS                                = SET_BIT           | EXC_MANAGER_MODULE_PARAM | 0x03,
    EXC_AUTH_SUCCESS                                    = SET_BIT           | EXC_MANAGER_MODULE_PARAM | 0x04,
    EXC_CCKM_REQUEST                                    = SET_BIT           | EXC_MANAGER_MODULE_PARAM | 0x05,
    EXC_CCKM_RESULT                                     = SET_BIT           | EXC_MANAGER_MODULE_PARAM | 0x06,
    EXC_ENABLED                                         = SET_BIT | GET_BIT | EXC_MANAGER_MODULE_PARAM | 0x07,
    EXC_CURRENT_AP_SUPPORTED_VERSION                    =           GET_BIT | EXC_MANAGER_MODULE_PARAM | 0x08,
#endif

    /* Roaming manager */
    ROAMING_MNGR_APPLICATION_CONFIGURATION          = SET_BIT | GET_BIT | ROAMING_MANAGER_MODULE_PARAM | 0x01,

    /* Parameters used for DEBUG */
    ROAMING_MNGR_TRIGGER_EVENT                      = SET_BIT           | ROAMING_MANAGER_MODULE_PARAM | 0x02,
    ROAMING_MNGR_CONN_STATUS                        = SET_BIT           | ROAMING_MANAGER_MODULE_PARAM | 0x03, 
    ROAMING_MNGR_CONF_PARAM                         =           GET_BIT | ROAMING_MANAGER_MODULE_PARAM | 0x04,
#ifdef TI_DBG
    ROAMING_MNGR_PRINT_STATISTICS                   =           GET_BIT | ROAMING_MANAGER_MODULE_PARAM | 0x05,
    ROAMING_MNGR_RESET_STATISTICS                   =           GET_BIT | ROAMING_MANAGER_MODULE_PARAM | 0x06,
    ROAMING_MNGR_PRINT_CURRENT_STATUS               =           GET_BIT | ROAMING_MANAGER_MODULE_PARAM | 0x07,
    ROAMING_MNGR_PRINT_CANDIDATE_TABLE              =           GET_BIT | ROAMING_MANAGER_MODULE_PARAM | 0x08,
#endif


    SOFT_GEMINI_SET_ENABLE                              = SET_BIT |           SOFT_GEMINI_PARAM        | 0x01,
    SOFT_GEMINI_SET_RATE                                = SET_BIT |           SOFT_GEMINI_PARAM        | 0x02,
    SOFT_GEMINI_SET_CONFIG                              = SET_BIT |           SOFT_GEMINI_PARAM        | 0x03,
    SOFT_GEMINI_GET_STATUS                              =           GET_BIT | SOFT_GEMINI_PARAM        | 0x04,


    /* QOS manager params */
    QOS_MNGR_SHORT_RETRY_LIMIT_PARAM                    = SET_BIT | GET_BIT | QOS_MANAGER_PARAM | 0x01,
    QOS_MNGR_LONG_RETRY_LIMIT_PARAM                     = SET_BIT | GET_BIT | QOS_MANAGER_PARAM | 0x02,
    QOS_PACKET_BURST_ENABLE                             = SET_BIT | GET_BIT | QOS_MANAGER_PARAM | 0x03,
    QOS_MNGR_SET_SITE_PROTOCOL                          = SET_BIT |           QOS_MANAGER_PARAM | 0x04,
    QOS_MNGR_SET_802_11_POWER_SAVE_STATUS               = SET_BIT |           QOS_MANAGER_PARAM | 0x05,
    QOS_MNGR_SET_OS_PARAMS                              = SET_BIT |           QOS_MANAGER_PARAM | 0x07,
    QOS_MNGR_SET_OPERATIONAL_MODE                       = SET_BIT |           QOS_MANAGER_PARAM | 0x08,
    QOS_MNGR_CURRENT_PS_MODE                            = SET_BIT | GET_BIT | QOS_MANAGER_PARAM | 0x09,
    QOS_MNGR_AP_QOS_PARAMETERS                          = SET_BIT | GET_BIT | QOS_MANAGER_PARAM | 0x0A,
    QOS_MNGR_OS_TSPEC_PARAMS                            = SET_BIT |           QOS_MANAGER_PARAM | 0x0B,
    QOS_MNGR_AC_STATUS                                  = SET_BIT | GET_BIT | QOS_MANAGER_PARAM | 0x0C,
    QOS_MNGR_ADD_TSPEC_REQUEST                          = SET_BIT           | QOS_MANAGER_PARAM | 0x0D,
    QOS_MNGR_DEL_TSPEC_REQUEST                          = SET_BIT           | QOS_MANAGER_PARAM | 0x0E,
    QOS_MNGR_ACTIVE_PROTOCOL                            =           GET_BIT | QOS_MANAGER_PARAM | 0x0F,
    QOS_SET_RATE_THRESHOLD                              = SET_BIT           | QOS_MANAGER_PARAM | 0x10,
    QOS_GET_RATE_THRESHOLD                              = SET_BIT | GET_BIT | QOS_MANAGER_PARAM | 0x11,
    QOS_MNGR_GET_DESIRED_PS_MODE                        =           GET_BIT | QOS_MANAGER_PARAM | 0x12,
    QOS_SET_RX_TIME_OUT                                 = SET_BIT           | QOS_MANAGER_PARAM | 0x14, 
    QOS_MNGR_VOICE_RE_NEGOTIATE_TSPEC                   = SET_BIT | GET_BIT | QOS_MANAGER_PARAM | 0x15,
    QOS_MNGR_RESEND_TSPEC_REQUEST                       = SET_BIT           | QOS_MANAGER_PARAM | 0x16,

    /* Power Manager params */
    POWER_MGR_POWER_MODE                                = SET_BIT | GET_BIT | POWER_MANAGER_PARAM | 0x01,
    POWER_MGR_DISABLE_PRIORITY                          = SET_BIT |           POWER_MANAGER_PARAM | 0x02,   
    POWER_MGR_ENABLE_PRIORITY                           = SET_BIT |           POWER_MANAGER_PARAM | 0x03,    
    POWER_MGR_POWER_LEVEL_PS                            = SET_BIT | GET_BIT | POWER_MANAGER_PARAM | 0x04,
    POWER_MGR_POWER_LEVEL_DEFAULT                       = SET_BIT | GET_BIT | POWER_MANAGER_PARAM | 0x05,
    POWER_MGR_POWER_LEVEL_DOZE_MODE                     = SET_BIT | GET_BIT | POWER_MANAGER_PARAM | 0x06,

}   externalParam_e;


#endif /* __PARAM_MGR_H__ */
