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

#ifndef _WHAL_DEFAULT_PARAMS_H
#define _WHAL_DEFAULT_PARAMS_H


/*****************************************************************************
 **                                                                         **
 **                                                                         **
 **                       CONSTANTS                                         **
 **                                                                         **
 **                                                                         **
 *****************************************************************************/

/* PALAU Group Address Default Values */
#define NUM_GROUP_ADDRESS_VALUE_DEF 0   
#define NUM_GROUP_ADDRESS_VALUE_MIN 0
#define NUM_GROUP_ADDRESS_VALUE_MAX 8

/* Early Wakeup Default Values */
#define EARLY_WAKEUP_ENABLE_MIN         (FALSE)
#define EARLY_WAKEUP_ENABLE_MAX         (TRUE)
#define EARLY_WAKEUP_ENABLE_DEF         (TRUE)

/* ARP IP Filter Default Values */
#define MIN_FILTER_ENABLE_VALUE 0
#define MAX_FILTER_ENABLE_VALUE 2
#define DEF_FILTER_ENABLE_VALUE 0
#define FILTER_ENABLE_FLAG_LEN  1

/* Beacon filter Deafult Values */
#define DEF_BEACON_FILTER_ENABLE_VALUE 1
#define DEF_BEACON_FILTER_IE_TABLE_NUM 15
#define MIN_BEACON_FILTER_ENABLE_VALUE 0
#define MAX_BEACON_FILTER_ENABLE_VALUE 1
#define BEACON_FILTER_IE_TABLE_DEF_SIZE 35
#define BEACON_FILTER_IE_TABLE_MAX_SIZE 100
#define BEACON_FILTER_IE_TABLE_MIN_SIZE 0 
#define BEACON_FILTER_IE_TABLE_MAX_NUM (6+32)
#define BEACON_FILTER_IE_TABLE_MIN_NUM 0 

#define HAL_CTRL_BET_ENABLE_MIN					0
#define HAL_CTRL_BET_ENABLE_MAX					1
#define HAL_CTRL_BET_ENABLE_DEF					1

#define HAL_CTRL_BET_MAX_CONSC_MIN				1
#define HAL_CTRL_BET_MAX_CONSC_MAX				50
#define HAL_CTRL_BET_MAX_CONSC_DEF				8

/* TX XFER parameters */
#define TX_XFER_HW_BUFFER_FULL_DUR_RECOVERY_DEF     50
#define TX_XFER_HW_BUFFER_FULL_DUR_RECOVERY_MIN     30
#define TX_XFER_HW_BUFFER_FULL_DUR_RECOVERY_MAX     1000

/* Default Value for Atheros time out value */
#define DEF_TX_POWER_ADJUST_TIME_OUT            5000

#define DEF_NUM_STORED_FILTERS 1
#define MIN_NUM_STORED_FILTERS 1
#define MAX_NUM_STORED_FILTERS 8

#define HAL_CTRL_HW_ACCESS_METHOD_MIN           0
#define HAL_CTRL_HW_ACCESS_METHOD_MAX           2
#define HAL_CTRL_HW_ACCESS_METHOD_DEF           1

#define HAL_CTRL_SITE_FRAG_COLLECT_MIN          2
#define HAL_CTRL_SITE_FRAG_COLLECT_MAX          10
#define HAL_CTRL_SITE_FRAG_COLLECT_DEF          3


#define HAL_CTRL_HOST_RX_DESC_MIN               1
#define HAL_CTRL_HOST_RX_DESC_MAX               127
#define HAL_CTRL_HOST_RX_DESC_DEF               32 /* instead of 40 - for a bigger TKIP FW*/

#define HAL_CTRL_HOST_TX_DESC_MIN               1
#define HAL_CTRL_HOST_TX_DESC_MAX               127
#define HAL_CTRL_HOST_TX_DESC_DEF               32 /* instead of 40 - for a bigger TKIP FW*/

#define HAL_CTRL_ACX_RX_DESC_MIN                1
#define HAL_CTRL_ACX_RX_DESC_MAX                127
#define HAL_CTRL_ACX_RX_DESC_DEF                32

#define HAL_CTRL_ACX_TX_DESC_MIN                1
#define HAL_CTRL_ACX_TX_DESC_MAX                127
#define HAL_CTRL_ACX_TX_DESC_DEF                16

#define HAL_CTRL_ACX_BLOCK_SIZE_MIN             256
#define HAL_CTRL_ACX_BLOCK_SIZE_MAX             2000
#define HAL_CTRL_ACX_BLOCK_SIZE_DEF             256

#define HAL_CTRL_RX_BLOCKS_RATIO_MIN            0
#define HAL_CTRL_RX_BLOCKS_RATIO_MAX            100
#define HAL_CTRL_RX_BLOCKS_RATIO_DEF            50

#define HAL_CTRL_USE_PLCP_HDR_DEF               1
#define HAL_CTRL_USE_PLCP_HDR_MAX               1
#define HAL_CTRL_USE_PLCP_HDR_MIN               0

#define HAL_CTRL_TX_FLASH_ENABLE_MIN            FALSE
#define HAL_CTRL_TX_FLASH_ENABLE_MAX            TRUE
#define HAL_CTRL_TX_FLASH_ENABLE_DEF            TRUE

#define HAL_CTRL_USE_INTR_TRHESHOLD_MIN         0
#define HAL_CTRL_USE_INTR_TRHESHOLD_MAX         1
#define HAL_CTRL_USE_INTR_TRHESHOLD_DEF         0

#define HAL_CTRL_USE_TX_DATA_INTR_MIN           0
#define HAL_CTRL_USE_TX_DATA_INTR_MAX           1

#if (!defined TIWLN_WINCE30) || (defined EMBEDDED_BOARD1)
#define HAL_CTRL_USE_TX_DATA_INTR_DEF           1
#else
#define HAL_CTRL_USE_TX_DATA_INTR_DEF           0
#endif

#define NUM_OF_CHANNELS_24                      (14)
#define A_5G_BAND_MIN_CHANNEL       			36
#define A_5G_BAND_MAX_CHANNEL       			180
#define A_5G_BAND_NUM_CHANNELS  				(A_5G_BAND_MAX_CHANNEL-A_5G_BAND_MIN_CHANNEL+1)


#define HAL_CTRL_CALIBRATION_CHANNEL_2_4_MIN              1
#define HAL_CTRL_CALIBRATION_CHANNEL_2_4_MAX              NUM_OF_CHANNELS_24
#define HAL_CTRL_CALIBRATION_CHANNEL_2_4_DEF              1

#define HAL_CTRL_CALIBRATION_CHANNEL_5_0_MIN              34
#define HAL_CTRL_CALIBRATION_CHANNEL_5_0_MAX              A_5G_BAND_MAX_CHANNEL
#define HAL_CTRL_CALIBRATION_CHANNEL_5_0_DEF              36

#define HAL_CTRL_CALIBRATION_CHANNEL_4_9_MIN              8
#define HAL_CTRL_CALIBRATION_CHANNEL_4_9_MAX              16
#define HAL_CTRL_CALIBRATION_CHANNEL_4_9_DEF              12

#define HAL_CTRL_RTS_THRESHOLD_MIN              0
#define HAL_CTRL_RTS_THRESHOLD_MAX              4096
#define HAL_CTRL_RTS_THRESHOLD_DEF              2347

#define HAL_CTRL_BCN_RX_TIME_OUT_MIN            10      /* ms */
#define HAL_CTRL_BCN_RX_TIME_OUT_MAX            1000    /* ms */
#define HAL_CTRL_BCN_RX_TIME_OUT_DEF            10      /* ms */

#define HAL_CTRL_RX_DISABLE_BROADCAST_MIN       FALSE
#define HAL_CTRL_RX_DISABLE_BROADCAST_MAX       TRUE
#define HAL_CTRL_RX_DISABLE_BROADCAST_DEF       FALSE

/* Indicate if the recovery process is active or not */
#define HAL_CTRL_RECOVERY_ENABLE_MIN            FALSE
#define HAL_CTRL_RECOVERY_ENABLE_MAX            TRUE
#define HAL_CTRL_RECOVERY_ENABLE_DEF            TRUE

#define HAL_CTRL_FRAG_THRESHOLD_MIN             256
#define HAL_CTRL_FRAG_THRESHOLD_MAX             4096
#define HAL_CTRL_FRAG_THRESHOLD_DEF             2312

#define HAL_CTRL_MAX_TX_MSDU_LIFETIME_MIN       0
#define HAL_CTRL_MAX_TX_MSDU_LIFETIME_MAX       3000
#define HAL_CTRL_MAX_TX_MSDU_LIFETIME_DEF       512

#define HAL_CTRL_MAX_RX_MSDU_LIFETIME_MIN       0
#define HAL_CTRL_MAX_RX_MSDU_LIFETIME_MAX       0xFFFFFFFF
#define HAL_CTRL_MAX_RX_MSDU_LIFETIME_DEF       512000


#define HAL_CTRL_LISTEN_INTERVAL_MIN            1
#define HAL_CTRL_LISTEN_INTERVAL_MAX            10
#define HAL_CTRL_LISTEN_INTERVAL_DEF            3

#define HAL_CTRL_MAX_FULL_BEACON_MIN            0
#define HAL_CTRL_MAX_FULL_BEACON_MAX            10000
#define HAL_CTRL_MAX_FULL_BEACON_DEF            1000

#define HAL_CTRL_BET_ENABLE_THRESHOLD_MIN       0
#define HAL_CTRL_BET_ENABLE_THRESHOLD_MAX       255
#define HAL_CTRL_BET_ENABLE_THRESHOLD_DEF       8

#define HAL_CTRL_BET_DISABLE_THRESHOLD_MIN       0
#define HAL_CTRL_BET_DISABLE_THRESHOLD_MAX       255
#define HAL_CTRL_BET_DISABLE_THRESHOLD_DEF       12

/* This field indicates the number of transmit retries to attempt at
    the rate specified in the TNETW1130 Tx descriptor before
    falling back to the next lowest rate.
    If this field is set to 0xff, then rate fallback is disabled.
    If this field is 0, then there will be 0 retries before starting fallback.*/
#define HAL_CTRL_RATE_FB_RETRY_LIMIT_MIN        0   /* => No retries before starting RateFallBack */
#define HAL_CTRL_RATE_FB_RETRY_LIMIT_MAX        255 /* =>0xff for disabling Rate fallback */
#define HAL_CTRL_RATE_FB_RETRY_LIMIT_DEF        0

#define HAL_CTRL_TX_ANTENNA_MIN                 TX_ANTENNA_2
#define HAL_CTRL_TX_ANTENNA_MAX                 TX_ANTENNA_1
#define HAL_CTRL_TX_ANTENNA_DEF                 TX_ANTENNA_1

#define HAL_CTRL_RX_ANTENNA_MIN                 RX_ANTENNA_1
#define HAL_CTRL_RX_ANTENNA_MAX                 RX_ANTENNA_PARTIAL
#define HAL_CTRL_RX_ANTENNA_DEF                 RX_ANTENNA_FULL

#define HAL_CTRL_TX_CMPLT_THRESHOLD_DEF         0
#define HAL_CTRL_TX_CMPLT_THRESHOLD_MIN         0
#define HAL_CTRL_TX_CMPLT_THRESHOLD_MAX         15

#define HAL_CTRL_ACI_MODE_MIN                   0
#define HAL_CTRL_ACI_MODE_MAX                   255
#define HAL_CTRL_ACI_MODE_DEF                   0
    
#define HAL_CTRL_ACI_INPUT_CCA_MIN              0
#define HAL_CTRL_ACI_INPUT_CCA_MAX              255
#define HAL_CTRL_ACI_INPUT_CCA_DEF              1
    
#define HAL_CTRL_ACI_QUALIFIED_CCA_MIN          0
#define HAL_CTRL_ACI_QUALIFIED_CCA_MAX          255
#define HAL_CTRL_ACI_QUALIFIED_CCA_DEF          3
    
#define HAL_CTRL_ACI_STOMP_FOR_RX_MIN           0
#define HAL_CTRL_ACI_STOMP_FOR_RX_MAX           255
#define HAL_CTRL_ACI_STOMP_FOR_RX_DEF           2
    
#define HAL_CTRL_ACI_STOMP_FOR_TX_MIN           0
#define HAL_CTRL_ACI_STOMP_FOR_TX_MAX           255
#define HAL_CTRL_ACI_STOMP_FOR_TX_DEF           0
    
#define HAL_CTRL_ACI_TX_CCA_MIN                 0
#define HAL_CTRL_ACI_TX_CCA_MAX                 255
#define HAL_CTRL_ACI_TX_CCA_DEF                 1

/************************************/      
/*      Rates values                */  
/************************************/


#define BASIC_RATE_SET_1_2                  0
#define BASIC_RATE_SET_1_2_5_5_11           1


#define BASIC_RATE_SET_UP_TO_12             2
#define BASIC_RATE_SET_UP_TO_18             3
#define BASIC_RATE_SET_1_2_5_5_6_11_12_24   4
#define BASIC_RATE_SET_UP_TO_36             5
#define BASIC_RATE_SET_UP_TO_48             6
#define BASIC_RATE_SET_UP_TO_54             7
#define BASIC_RATE_SET_UP_TO_24             8
#define BASIC_RATE_SET_6_12_24              9


/* Keep increasing define values - related to increasing suported rates */
#define SUPPORTED_RATE_SET_1_2              0
#define SUPPORTED_RATE_SET_1_2_5_5_11       1
#define SUPPORTED_RATE_SET_1_2_5_5_11_22    2
#define SUPPORTED_RATE_SET_UP_TO_18         3
#define SUPPORTED_RATE_SET_UP_TO_24         4
#define SUPPORTED_RATE_SET_UP_TO_36         5
#define SUPPORTED_RATE_SET_UP_TO_48         6
#define SUPPORTED_RATE_SET_UP_TO_54         7
#define SUPPORTED_RATE_SET_ALL              8
#define SUPPORTED_RATE_SET_ALL_OFDM         9


/*****************************************************************************
 **                                                                         **
 **                                                                         **
 **                       ENUMS                                             **
 **                                                                         **
 **                                                                         **
 *****************************************************************************/

typedef enum
{
    BSS_INDEPENDENT         = 0,
    BSS_INFRASTRUCTURE      = 1,
    BSS_ANY                 = 2,
    BSS_AP                  = 3
} bssType_e;


typedef enum
{
    PREAMBLE_LONG           = 0,
    PREAMBLE_SHORT          = 1,
    PREAMBLE_UNSPECIFIED    = 0xFF
} preamble_e;

typedef enum
{
    PHY_SLOT_TIME_LONG = 0,
    PHY_SLOT_TIME_SHORT = 1
} slotTime_e;

typedef enum
{
    NULL_KEY = 0,
    WEP_KEY,
    TKIP_KEY,
    AES_KEY,
    EXC_KEY,
} keyType_e;

/* make it same as "rate_e" */
typedef enum
{
  REG_RATE_AUTO_BIT         = 0, /* This value is reserved if this enum is used for MgmtCtrlTxRate  - The auto mode is noly valid for data packets */
  REG_RATE_1M_BIT           = 1,
  REG_RATE_2M_BIT           = 2,
  REG_RATE_5_5M_CCK_BIT     = 3,
  REG_RATE_11M_CCK_BIT      = 4,
  REG_RATE_22M_PBCC_BIT     = 5,
  REG_RATE_6M_OFDM_BIT      = 6,
  REG_RATE_9M_OFDM_BIT      = 7,
  REG_RATE_12M_OFDM_BIT     = 8,
  REG_RATE_18M_OFDM_BIT     = 9,
  REG_RATE_24M_OFDM_BIT     = 10,
  REG_RATE_36M_OFDM_BIT     = 11,
  REG_RATE_48M_OFDM_BIT     = 12,
  REG_RATE_54M_OFDM_BIT     = 13
} registryTxRate_e;






#endif /* _WHAL_DEFAULT_PARAMS_H */
