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

#ifndef _CORE_DEFAULT_PARAMS_H
#define _CORE_DEFAULT_PARAMS_H
				  
#if defined(__ARMCC__) 
#include "public_infoele.h"
#endif
/************************************/
/*      Min, Max & Default values   */
/************************************/

/* In this section are defined default, min & max values for parameters, according to the MIB */
/* This values are used as following:
        - By the OS abstraction layer in order to fill the init table with the default values
        if the NVRAM/Registry value for the parameter is invalid
        - By the core modules in order to perform validity check upon setting a parameter. */


#define SITE_MGR_CHANNEL_MIN                    1
#define SITE_MGR_CHANNEL_A_MIN                  36 /*   band A*/
#define SITE_MGR_CHANNEL_B_G_MAX                14 /*   band B&G*/
#define SITE_MGR_CHANNEL_MAX                    201
#define SITE_MGR_CHANNEL_DEF                    11

#define SITE_MGR_DOT_11_MODE_MIN                DOT11_B_MODE
#define SITE_MGR_DOT_11_MODE_MAX                DOT11_DUAL_MODE
#define SITE_MGR_DOT_11_MODE_DEF                DOT11_G_MODE

#define SITE_MGR_BSSID_DEF                      "DEADDEADDEAD"

#define SITE_MGR_SSID_STRING_DEF                ""
#define SITE_MGR_SSID_LEN_DEF                   0

#define SITE_MGR_BSS_TYPE_DEF                   BSS_INFRASTRUCTURE

#define SITE_MGR_DEF_RATE_SET_MAX_BASIC_DEF     DRV_RATE_2M
#define SITE_MGR_DEF_RATE_SET_MAX_ACTIVE_DEF    DRV_RATE_11M

#define SITE_MGR_MGMT_FRAME_RATE_MIN            DRV_RATE_1M
#define SITE_MGR_MGMT_FRAME_RATE_MAX            DRV_RATE_22M
#define SITE_MGR_MGMT_FRAME_RATE_DEF            DRV_RATE_2M

#define SITE_MGR_MODULATION_TYPE_DEF            DRV_MODULATION_CCK

#define SITE_MGR_BEACON_INTERVAL_MIN            1
#define SITE_MGR_BEACON_INTERVAL_MAX            65535
#define SITE_MGR_BEACON_INTERVAL_DEF            200

/* number of events to wake up on -
    For WakeOnBeacon- Aging interval =  SITE_MGR_NUMBER_OF_EVENTS_BEFORE_AGING * BeaconInterval
    For WakeOnDtim - Aging interval =  SITE_MGR_NUMBER_OF_EVENTS_BEFORE_AGING * BeaconInterval * DtimPeriod */
#define SITE_MGR_NUMBER_OF_EVENTS_BEFORE_AGING_MIN  2
#define SITE_MGR_NUMBER_OF_EVENTS_BEFORE_AGING_MAX  20
#define SITE_MGR_NUMBER_OF_EVENTS_BEFORE_AGING_DEF  10

#define SITE_MGR_NUMBER_OF_TX_FAILURE_BEFORE_AGING_MIN  3
#define SITE_MGR_NUMBER_OF_TX_FAILURE_BEFORE_AGING_MAX  100
#define SITE_MGR_NUMBER_OF_TX_FAILURE_BEFORE_AGING_DEF  6

#define SITE_MGR_ROAMING_STATS_RESET_TIMEOUT_MIN   5        /* in seconds */
#define SITE_MGR_ROAMING_STATS_RESET_TIMEOUT_MAX   60
#define SITE_MGR_ROAMING_STATS_RESET_TIMEOUT_DEF   10

#define SITE_MGR_LNA_BEACON_INT_COUNT_MIN       2   
#define SITE_MGR_LNA_BEACON_INT_COUNT_MAX       0xFFFF
#define SITE_MGR_LNA_BEACON_INT_COUNT_DEF       3

#define SITE_MGR_LNA_PD_THRESHOLD_LOW_MIN       0x00    
#define SITE_MGR_LNA_PD_THRESHOLD_LOW_MAX       0xff    
#define SITE_MGR_LNA_PD_THRESHOLD_LOW_DEF       0x90    

#define SITE_MGR_LNA_PD_THRESHOLD_HIGH_MIN      0x00    
#define SITE_MGR_LNA_PD_THRESHOLD_HIGH_MAX      0xff    
#define SITE_MGR_LNA_PD_THRESHOLD_HIGH_DEF      0xD5    

#define SITE_MGR_LNA_EN_DINAMYC_TX_ALGO_DEF     0   
#define SITE_MGR_LNA_EN_DINAMYC_TX_ALGO_MAX     1
#define SITE_MGR_LNA_EN_DINAMYC_TX_ALGO_MIN     0

#define SITE_MGR_PREAMBLE_TYPE_DEF              PREAMBLE_SHORT

#define SITE_MGR_EXTERNAL_MODE_MIN              0
#define SITE_MGR_EXTERNAL_MODE_MAX              1
#define SITE_MGR_EXTERNAL_MODE_DEF              0

#define SITE_MGR_PERFORM_BUILD_IN_TEST_RECOVEY_MIN      FALSE
#define SITE_MGR_PERFORM_BUILD_IN_TEST_RECOVEY_MAX      TRUE
#define SITE_MGR_PERFORM_BUILD_IN_TEST_RECOVEY_DEF      FALSE

#define SITE_MGR_WiFiAdHoc_MIN                  0
#define SITE_MGR_WiFiAdHoc_MAX                  1
#define SITE_MGR_WiFiAdHoc_DEF                  0



#define SITE_MGR_BROADCAST_BACKGROUND_SCAN_MIN  FALSE
#define SITE_MGR_BROADCAST_BACKGROUND_SCAN_MAX  TRUE
#define SITE_MGR_BROADCAST_BACKGROUND_SCAN_DEF  FALSE

#define SITE_MGR_PERIODIC_BROADCAST_BACKGROUND_SCAN_MIN FALSE
#define SITE_MGR_PERIODIC_BROADCAST_BACKGROUND_SCAN_MAX TRUE
#define SITE_MGR_PERIODIC_BROADCAST_BACKGROUND_SCAN_DEF FALSE

#define SITE_MGR_PERIODIC_BROADCAST_BACKGROUND_SCAN_INTERVAL_TIME_MIN   10000  /* in miliseconds */
#define SITE_MGR_PERIODIC_BROADCAST_BACKGROUND_SCAN_INTERVAL_TIME_MAX   3600000
#define SITE_MGR_PERIODIC_BROADCAST_BACKGROUND_SCAN_INTERVAL_TIME_DEF   60000

#define SITE_MGR_KEEP_ALIVE_MIN                 FALSE
#define SITE_MGR_KEEP_ALIVE_MAX                 TRUE
#define SITE_MGR_KEEP_ALIVE_DEF                 TRUE

#define SITE_MGR_RX_LEVEL_TABLE_SIZE_DEF        44

#define MAX_SITES_BG_BAND   32
#define MAX_SITES_A_BAND    20
#define MAX_HASH_ENTRIES    32 /* must a power of 2  and more or less the site table size */

#define NUM_OF_SITE_TABLE   2

/* Beacon broadcast options */
#define BCN_RX_TIMEOUT_DEF_VALUE 10000
#define BCN_RX_TIMEOUT_MIN_VALUE 1
#define BCN_RX_TIMEOUT_MAX_VALUE 65535
    
#define	BROADCAST_RX_TIMEOUT_DEF_VALUE 20000
#define BROADCAST_RX_TIMEOUT_MIN_VALUE 1 
#define BROADCAST_RX_TIMEOUT_MAX_VALUE 65535

#define	RX_BROADCAST_IN_PS_DEF_VALUE 1
#define RX_BROADCAST_IN_PS_MIN_VALUE 0
#define RX_BROADCAST_IN_PS_MAX_VALUE 1

#define	CONSECUTIVE_PS_POLL_FAILURE_DEF 4
#define CONSECUTIVE_PS_POLL_FAILURE_MIN 1
#define CONSECUTIVE_PS_POLL_FAILURE_MAX 100

#define	PS_POLL_FAILURE_PERIOD_DEF 20
#define PS_POLL_FAILURE_PERIOD_MIN 0       /* '0' is disabled */
#define PS_POLL_FAILURE_PERIOD_MAX 60000

/*---------------------------*/
/*  Classifier parameters    */
/*---------------------------*/

#define CLSFR_TYPE_MIN						1 /* 1 - Dtag, 2 - Port, 3 - IP & port */ 
#define CLSFR_TYPE_DEF						3
#define CLSFR_TYPE_MAX						3

/* general values of D-tags */
#define CLASSIFIER_DTAG_MIN					0 
#define CLASSIFIER_DTAG_MAX					7
#define CLASSIFIER_DTAG_DEF					0 

/* general values of code points in 
the DSCP classification table*/
#define CLASSIFIER_CODE_POINT_MIN		0 
#define CLASSIFIER_CODE_POINT_MAX		63
#define CLASSIFIER_CODE_POINT_DEF		0

/* general values of port numbers */
#define CLASSIFIER_PORT_MIN					1 
#define CLASSIFIER_PORT_MAX					65535
#define CLASSIFIER_PORT_DEF					1024 

/* general values of IP addresses */
#define CLASSIFIER_IPADDRESS_MIN			0x0			/* TBD according to spec!*/
#define CLASSIFIER_IPADDRESS_DEF			0x0A030DC4	/* MY IP ... TBD according to spec!*/
#define CLASSIFIER_IPADDRESS_MAX			0xFFFFFFFF  /* TBD according to spec!*/

/* DSCP (differentiated services code 
point) classifier parameters  
--------------------------------*/
/* number of classifier entries in the 
   classification table (in case of DSCP classifier) */

#define NUM_OF_CODE_POINTS_MIN				0
#define NUM_OF_CODE_POINTS_MAX				16
#define NUM_OF_CODE_POINTS_DEF				0

/* def values of code points in the DSCP classification table*/
#define DSCP_CLASSIFIER_CODE_POINT_DEF	0x0

/* def values of D-tags in the DSCP classification table*/
#define DSCP_CLASSIFIER_DTAG_DEF			0

/* Port Classifier parameters 
--------------------------------*/

/* number of entries in the classification table (in case of destination port classifier) */
#define NUM_OF_PORT_CLASSIFIERS_MIN			0
#define NUM_OF_PORT_CLASSIFIERS_MAX			16
#define NUM_OF_PORT_CLASSIFIERS_DEF			0

/* def values of port numbers in the destination port classification table*/
#define PORT_CLASSIFIER_PORT_DEF			5000

/* def values of D-tags in the destination port classification table*/
#define PORT_CLASSIFIER_DTAG_DEF			7

/* IP&Port Classifier parameters 
--------------------------------*/

/* number of active entries in the 
IP&Port classification table  */
#define NUM_OF_IPPORT_CLASSIFIERS_MIN		0
#define NUM_OF_IPPORT_CLASSIFIERS_MAX		16
#define NUM_OF_IPPORT_CLASSIFIERS_DEF		0

/* def values of IP addresses in the IP&Port classification table*/
#define IPPORT_CLASSIFIER_IPADDRESS_DEF	167972292

/* def values of port numbers in the IP&Port classification table*/
#define IPPORT_CLASSIFIER_PORT_DEF		5004

/* def values of D-tags in the IP&Port classification table*/
#define IPPORT_CLASSIFIER_DTAG_DEF		7

/* end of classifier parameters */

#define MAX_USER_PRIORITY			(7)



#define  WME_ENABLED_MIN                       (FALSE)
#define  WME_ENABLED_MAX                       (TRUE)
#define  WME_ENABLED_DEF                       (TRUE)

#define  QOS_TRAFFIC_ADM_CTRL_ENABLED_MIN				   (FALSE)
#define  QOS_TRAFFIC_ADM_CTRL_ENABLED_MAX				   (TRUE) 
#define  QOS_TRAFFIC_ADM_CTRL_ENABLED_DEF				   (TRUE) 

#define  QOS_DESIRED_PS_MODE_MIN			PS_SCHEME_LEGACY
#define  QOS_DESIRED_PS_MODE_MAX			MAX_PS_SCHEME
#define  QOS_DESIRED_PS_MODE_DEF			PS_SCHEME_UPSD_TRIGGER

#define  QOS_TAG_ZERO_PRIO_MIN                 (FALSE)
#define  QOS_TAG_ZERO_PRIO_MAX                 (TRUE)
#define  QOS_TAG_ZERO_PRIO_DEF                 (TRUE)


/* for the AC */
#define  QOS_TX_OP_CONTINUATION_MIN             0
#define  QOS_TX_OP_CONTINUATION_MAX             1
#define  QOS_TX_OP_CONTINUATION_DEF            1

#define  QOS_TX_OP_LIMIT_MIN                   0
#define  QOS_TX_OP_LIMIT_MAX                   32000     
#define  QOS_TX_OP_LIMIT_DEF                   0

/* for packet burst in non-qos protocol */
#define  QOS_PACKET_BURST_ENABLE_MIN             0
#define  QOS_PACKET_BURST_ENABLE_DEF            0
#define  QOS_PACKET_BURST_ENABLE_MAX             1

#define  QOS_PACKET_BURST_TXOP_LIMIT_MIN         0
#define  QOS_PACKET_BURST_TXOP_LIMIT_MAX         1000     
#define  QOS_PACKET_BURST_TXOP_LIMIT_DEF         93

#define  QOS_RX_TIMEOUT_PS_POLL_MIN                0
#define  QOS_RX_TIMEOUT_PS_POLL_MAX                (200000)
#define  QOS_RX_TIMEOUT_PS_POLL_DEF                15

#define  QOS_RX_TIMEOUT_UPSD_MIN                   0
#define  QOS_RX_TIMEOUT_UPSD_MAX                   (200000)
#define  QOS_RX_TIMEOUT_UPSD_DEF                   15

#define  QOS_MSDU_LIFE_TIME_MIN                0
#define  QOS_MSDU_LIFE_TIME_MAX                1024

#define  QOS_MSDU_LIFE_TIME_BE_MIN             (QOS_MSDU_LIFE_TIME_MIN)
#define  QOS_MSDU_LIFE_TIME_BE_MAX             (QOS_MSDU_LIFE_TIME_MAX)
#define  QOS_MSDU_LIFE_TIME_BE_DEF             (512)

#define  QOS_MSDU_LIFE_TIME_BK_MIN             (QOS_MSDU_LIFE_TIME_MIN)
#define  QOS_MSDU_LIFE_TIME_BK_MAX             (QOS_MSDU_LIFE_TIME_MAX)
#define  QOS_MSDU_LIFE_TIME_BK_DEF             (100)

#define  QOS_MSDU_LIFE_TIME_VI_MIN             (QOS_MSDU_LIFE_TIME_MIN)
#define  QOS_MSDU_LIFE_TIME_VI_MAX             (QOS_MSDU_LIFE_TIME_MAX)
#define  QOS_MSDU_LIFE_TIME_VI_DEF             (100)

#define  QOS_MSDU_LIFE_TIME_VO_MIN             (QOS_MSDU_LIFE_TIME_MIN)
#define  QOS_MSDU_LIFE_TIME_VO_MAX             (QOS_MSDU_LIFE_TIME_MAX)
#define  QOS_MSDU_LIFE_TIME_VO_DEF             (40)

#define  QOS_TX_QUEUE_SIZE_MIN                 1
#define  QOS_TX_QUEUE_SIZE_MAX                 128

#define  QOS_TX_QUEUE0_SIZE_MIN                (QOS_TX_QUEUE_SIZE_MIN)
#define  QOS_TX_QUEUE0_SIZE_MAX                (QOS_TX_QUEUE_SIZE_MAX)
#define  QOS_TX_QUEUE0_SIZE_DEF                (32)

#define  QOS_TX_QUEUE1_SIZE_MIN                (QOS_TX_QUEUE_SIZE_MIN)
#define  QOS_TX_QUEUE1_SIZE_MAX                (QOS_TX_QUEUE_SIZE_MAX)
#define  QOS_TX_QUEUE1_SIZE_DEF                (32)

#define  QOS_TX_QUEUE2_SIZE_MIN                (QOS_TX_QUEUE_SIZE_MIN)
#define  QOS_TX_QUEUE2_SIZE_MAX                (QOS_TX_QUEUE_SIZE_MAX)
#define  QOS_TX_QUEUE2_SIZE_DEF                (32)

#define  QOS_TX_QUEUE3_SIZE_MIN                (QOS_TX_QUEUE_SIZE_MIN)
#define  QOS_TX_QUEUE3_SIZE_MAX                (QOS_TX_QUEUE_SIZE_MAX)
#define  QOS_TX_QUEUE3_SIZE_DEF                (32)

#define  QOS_WME_PS_MODE_BE_MIN                (PS_SCHEME_LEGACY)
#define  QOS_WME_PS_MODE_BE_MAX                (MAX_PS_SCHEME)
#define  QOS_WME_PS_MODE_BE_DEF                (PS_SCHEME_LEGACY_PSPOLL)

#define  QOS_WME_PS_MODE_BK_MIN                (PS_SCHEME_LEGACY)      
#define  QOS_WME_PS_MODE_BK_MAX                (MAX_PS_SCHEME)          
#define  QOS_WME_PS_MODE_BK_DEF                (PS_SCHEME_LEGACY_PSPOLL)

#define  QOS_WME_PS_MODE_VI_MIN                (PS_SCHEME_LEGACY)      
#define  QOS_WME_PS_MODE_VI_MAX                (MAX_PS_SCHEME)          
#define  QOS_WME_PS_MODE_VI_DEF                (PS_SCHEME_LEGACY_PSPOLL)

#define  QOS_WME_PS_MODE_VO_MIN                (PS_SCHEME_LEGACY)      
#define  QOS_WME_PS_MODE_VO_MAX                (MAX_PS_SCHEME)          
#define  QOS_WME_PS_MODE_VO_DEF                (PS_SCHEME_LEGACY_PSPOLL)


/* 
 * new host interface method 
 * sum of High threshold TxBlocks > 100% of Tx blocks 
 */
#define  QOS_TX_BLKS_HIGH_PRCNT_MIN            (0)
#define  QOS_TX_BLKS_HIGH_PRCNT_MAX            (100)

#define  QOS_TX_BLKS_HIGH_PRCNT_BK_DEF         (25)
#define  QOS_TX_BLKS_HIGH_PRCNT_BE_DEF         (35)   
#define  QOS_TX_BLKS_HIGH_PRCNT_VI_DEF         (35)
#define  QOS_TX_BLKS_HIGH_PRCNT_VO_DEF         (35)
/*
 * sum of Low threshold TxBlocks < 100% of Tx blocks 
 */
#define  QOS_TX_BLKS_LOW_PRCNT_BK_DEF         (15)
#define  QOS_TX_BLKS_LOW_PRCNT_BE_DEF         (25)   
#define  QOS_TX_BLKS_LOW_PRCNT_VI_DEF         (25)
#define  QOS_TX_BLKS_LOW_PRCNT_VO_DEF         (25)



#define  QOS_QID_MIN                           0
#define  QOS_QID_MAX                           3

#define  QOS_AC_MIN                            QOS_QID_MIN
#define  QOS_AC_MAX                            QOS_QID_MAX

#define  QOS_AIFS_MIN                          1
#define  QOS_AIFS_MAX                          15

#define QOS_CWMIN_MIN                          0
#define QOS_CWMIN_MAX                          15

#define QOS_CWMAX_MIN                          0
#define QOS_CWMAX_MAX                          15

#define QOS_TIMEOUT_MIN                        0
#define QOS_TIMEOUT_MAX                        65535

#define QOS_ACK_POLICY_MIN                     0
#define QOS_ACK_POLICY_MAX                     1

#define QOS_TRAFFIC_TYPE_MIN                   0
#define QOS_TRAFFIC_TYPE_MAX                   1

#define QOS_SHORT_RETRY_LIMIT_MIN              1
#define QOS_SHORT_RETRY_LIMIT_MAX              255
#define QOS_SHORT_RETRY_LIMIT_DEF              10

#define QOS_SHORT_RETRY_LIMIT_BE_MIN           (QOS_SHORT_RETRY_LIMIT_MIN)
#define QOS_SHORT_RETRY_LIMIT_BE_MAX           (QOS_SHORT_RETRY_LIMIT_MAX)
#define QOS_SHORT_RETRY_LIMIT_BE_DEF           (QOS_SHORT_RETRY_LIMIT_DEF)

#define QOS_SHORT_RETRY_LIMIT_BK_MIN           (QOS_SHORT_RETRY_LIMIT_MIN)
#define QOS_SHORT_RETRY_LIMIT_BK_MAX           (QOS_SHORT_RETRY_LIMIT_MAX)
#define QOS_SHORT_RETRY_LIMIT_BK_DEF           (QOS_SHORT_RETRY_LIMIT_DEF)

#define QOS_SHORT_RETRY_LIMIT_VI_MIN           (QOS_SHORT_RETRY_LIMIT_MIN)
#define QOS_SHORT_RETRY_LIMIT_VI_MAX           (QOS_SHORT_RETRY_LIMIT_MAX)
#define QOS_SHORT_RETRY_LIMIT_VI_DEF           (QOS_SHORT_RETRY_LIMIT_DEF)

#define QOS_SHORT_RETRY_LIMIT_VO_MIN           (QOS_SHORT_RETRY_LIMIT_MIN)
#define QOS_SHORT_RETRY_LIMIT_VO_MAX           (QOS_SHORT_RETRY_LIMIT_MAX)
#define QOS_SHORT_RETRY_LIMIT_VO_DEF           (4)


#define QOS_LONG_RETRY_LIMIT_MIN               1
#define QOS_LONG_RETRY_LIMIT_MAX               255
#define QOS_LONG_RETRY_LIMIT_DEF               4

#define QOS_LONG_RETRY_LIMIT_BE_MIN           (QOS_LONG_RETRY_LIMIT_MIN)
#define QOS_LONG_RETRY_LIMIT_BE_MAX           (QOS_LONG_RETRY_LIMIT_MAX)
#define QOS_LONG_RETRY_LIMIT_BE_DEF           (QOS_LONG_RETRY_LIMIT_DEF)

#define QOS_LONG_RETRY_LIMIT_BK_MIN           (QOS_LONG_RETRY_LIMIT_MIN)
#define QOS_LONG_RETRY_LIMIT_BK_MAX           (QOS_LONG_RETRY_LIMIT_MAX)
#define QOS_LONG_RETRY_LIMIT_BK_DEF           (QOS_LONG_RETRY_LIMIT_DEF)

#define QOS_LONG_RETRY_LIMIT_VI_MIN           (QOS_LONG_RETRY_LIMIT_MIN)
#define QOS_LONG_RETRY_LIMIT_VI_MAX           (QOS_LONG_RETRY_LIMIT_MAX)
#define QOS_LONG_RETRY_LIMIT_VI_DEF           (QOS_LONG_RETRY_LIMIT_DEF)

#define QOS_LONG_RETRY_LIMIT_VO_MIN           (QOS_LONG_RETRY_LIMIT_MIN)
#define QOS_LONG_RETRY_LIMIT_VO_MAX           (QOS_LONG_RETRY_LIMIT_MAX)
#define QOS_LONG_RETRY_LIMIT_VO_DEF           (QOS_LONG_RETRY_LIMIT_DEF)



#define QOS_QUEUE_0_OVFLOW_POLICY_MIN          (DROP_NEW_PACKET)
#define QOS_QUEUE_0_OVFLOW_POLICY_MAX          (DROP_OLD_PACKET)
#define QOS_QUEUE_0_OVFLOW_POLICY_DEF          (DROP_NEW_PACKET)

#define QOS_QUEUE_1_OVFLOW_POLICY_MIN          (DROP_NEW_PACKET)
#define QOS_QUEUE_1_OVFLOW_POLICY_MAX          (DROP_OLD_PACKET)
#define QOS_QUEUE_1_OVFLOW_POLICY_DEF          (DROP_NEW_PACKET)

#define QOS_QUEUE_2_OVFLOW_POLICY_MIN          (DROP_NEW_PACKET)
#define QOS_QUEUE_2_OVFLOW_POLICY_MAX          (DROP_OLD_PACKET)
#define QOS_QUEUE_2_OVFLOW_POLICY_DEF          (DROP_NEW_PACKET)

#define QOS_QUEUE_3_OVFLOW_POLICY_MIN          (DROP_NEW_PACKET)
#define QOS_QUEUE_3_OVFLOW_POLICY_MAX          (DROP_OLD_PACKET)
#define QOS_QUEUE_3_OVFLOW_POLICY_DEF          (DROP_NEW_PACKET)

#define QOS_ACK_POLICY_BE_MIN             (ACK_POLICY_LEGACY)
#define QOS_ACK_POLICY_BE_MAX			  (MAX_ACK_POLICY)	
#define QOS_ACK_POLICY_BE_DEF             (ACK_POLICY_LEGACY)

#define QOS_ACK_POLICY_BK_MIN             (ACK_POLICY_LEGACY)
#define QOS_ACK_POLICY_BK_MAX			  (MAX_ACK_POLICY)   
#define QOS_ACK_POLICY_BK_DEF             (ACK_POLICY_LEGACY)

#define QOS_ACK_POLICY_VI_MIN             (ACK_POLICY_LEGACY)
#define QOS_ACK_POLICY_VI_MAX			  (MAX_ACK_POLICY)   
#define QOS_ACK_POLICY_VI_DEF             (ACK_POLICY_LEGACY)

#define QOS_ACK_POLICY_VO_MIN             (ACK_POLICY_LEGACY)
#define QOS_ACK_POLICY_VO_MAX			  (MAX_ACK_POLICY)   
#define QOS_ACK_POLICY_VO_DEF             (ACK_POLICY_LEGACY)


/* MAX_SP_LEN_VALUES
  00 - all buffered frames 
  01 - 2
  10 - 4 
  11 - 6
*/

#define QOS_MAX_SP_LEN_MIN						0
#define QOS_MAX_SP_LEN_MAX						3
#define QOS_MAX_SP_LEN_DEF						1 /* means maxSpLen = 2 (changed for SoftGemini requiremnet) */


/*---------------------------
      ROAMING parameters
-----------------------------*/
#define ROAMING_MNGR_ENABLE_MIN             0
#define ROAMING_MNGR_ENABLE_MAX             1
#define ROAMING_MNGR_ENABLE_DEF             0

#define ROAMING_MNGR_ENABLE_PERIODIC_SCAN_MIN       0
#define ROAMING_MNGR_ENABLE_PERIODIC_SCAN_MAX       1
#define ROAMING_MNGR_ENABLE_PERIODIC_SCAN_DEF       0

#define ROAMING_MNGR_RSSI_GAP_MIN                   0
#define ROAMING_MNGR_RSSI_GAP_MAX                   50
#define ROAMING_MNGR_RSSI_GAP_DEF                   10

#define ROAMING_MNGR_PERIODIC_SCAN_TIEMOUT_MIN      1000
#define ROAMING_MNGR_PERIODIC_SCAN_TIEMOUT_MAX      10000
#define ROAMING_MNGR_PERIODIC_SCAN_TIEMOUT_DEF      3000

#define ROAMING_MNGR_PERIODIC_SCAN_MIN_CH_MIN       5
#define ROAMING_MNGR_PERIODIC_SCAN_MIN_CH_MAX       60
#define ROAMING_MNGR_PERIODIC_SCAN_MIN_CH_DEF       5

#define ROAMING_MNGR_PERIODIC_SCAN_MAX_CH_MIN       5
#define ROAMING_MNGR_PERIODIC_SCAN_MAX_CH_MAX       60
#define ROAMING_MNGR_PERIODIC_SCAN_MAX_CH_DEF       20

#define ROAMING_MNGR_PERIODIC_SCAN_ET_MODE_MIN      0
#define ROAMING_MNGR_PERIODIC_SCAN_ET_MODE_MAX      3
#define ROAMING_MNGR_PERIODIC_SCAN_ET_MODE_DEF      3

#define ROAMING_MNGR_PERIODIC_SCAN_MAX_NUM_FRAMES_MIN       1
#define ROAMING_MNGR_PERIODIC_SCAN_MAX_NUM_FRAMES_MAX       30
#define ROAMING_MNGR_PERIODIC_SCAN_MAX_NUM_FRAMES_DEF       1

#define ROAMING_MNGR_PERIODIC_SCAN_NUM_PROBE_REQ_MIN        1
#define ROAMING_MNGR_PERIODIC_SCAN_NUM_PROBE_REQ_MAX        10
#define ROAMING_MNGR_PERIODIC_SCAN_NUM_PROBE_REQ_DEF        2

/*---------------------------
    Measurement parameters
-----------------------------*/
#define MEASUREMENT_TRAFFIC_THRSHLD_MIN             1       /* Packets Per Second threshold */
#define MEASUREMENT_TRAFFIC_THRSHLD_MAX             1000
#define MEASUREMENT_TRAFFIC_THRSHLD_DEF             400

#define MEASUREMENT_MAX_DUR_NON_SRV_CHANNEL_MIN            1           /* In ms */
#define MEASUREMENT_MAX_DUR_NON_SRV_CHANNEL_MAX             1000
#define MEASUREMENT_MAX_DUR_NON_SRV_CHANNEL_DEF             300


/*---------------------------
      EXC Manager parameters
-----------------------------*/
#define EXC_MNGR_ENABLE_MIN             EXC_MODE_DISABLED
#define EXC_MNGR_ENABLE_MAX             EXC_MODE_STANDBY
#define EXC_MNGR_ENABLE_DEF             EXC_MODE_ENABLED

#define EXC_TEST_IGNORE_DEAUTH_0_DEF            1
#define EXC_TEST_IGNORE_DEAUTH_0_MIN            0
#define EXC_TEST_IGNORE_DEAUTH_0_MAX            1

#define SITE_MGR_ROAMING_TX_RATE_PERCENTAGE_MIN         30
#define SITE_MGR_ROAMING_TX_RATE_PERCENTAGE_MAX         75
#define SITE_MGR_ROAMING_TX_RATE_PERCENTAGE_DEF         40


#define SITE_MGR_ROAMING_RSSI_MIN                       0
#define SITE_MGR_ROAMING_RSSI_MAX                       100
#define SITE_MGR_ROAMING_RSSI_DEF                       80

#define SITE_MGR_ROAMING_CONS_TX_ERRORS_MIN				1
#define SITE_MGR_ROAMING_CONS_TX_ERRORS_MAX				200	
#define SITE_MGR_ROAMING_CONS_TX_ERRORS_DEF				10


#define SITE_MGR_POSTDISCONNECT_TIMEOUT_DEF     6000 /*6 sec*/
#define SITE_MGR_POSTDISCONNECT_TIMEOUT_MIN     1000
#define SITE_MGR_POSTDISCONNECT_TIMEOUT_MAX     10000

#define CONN_SELF_TIMEOUT_MIN                   1 * 1000        /* 1 seconds */
#define CONN_SELF_TIMEOUT_MAX                   60 * 1000       /* 1 minute */
#define CONN_SELF_TIMEOUT_DEF                   10 * 1000       /* 10 seconds */

#define AUTH_RESPONSE_TIMEOUT_MIN               100
#define AUTH_RESPONSE_TIMEOUT_MAX               5000
#define AUTH_RESPONSE_TIMEOUT_DEF               500

#define AUTH_MAX_RETRY_COUNT_MIN                1
#define AUTH_MAX_RETRY_COUNT_MAX                5
#define AUTH_MAX_RETRY_COUNT_DEF                2

#define ASSOC_RESPONSE_TIMEOUT_MIN              1000
#define ASSOC_RESPONSE_TIMEOUT_MAX              5000
#define ASSOC_RESPONSE_TIMEOUT_DEF              2000

#define ASSOC_MAX_RETRY_COUNT_MIN               1
#define ASSOC_MAX_RETRY_COUNT_MAX               5
#define ASSOC_MAX_RETRY_COUNT_DEF               2

#define RX_DATA_FILTERS_ENABLED_MIN      FALSE
#define RX_DATA_FILTERS_ENABLED_MAX      TRUE
#define RX_DATA_FILTERS_ENABLED_DEF      FALSE

#define RX_DATA_FILTERS_DEFAULT_ACTION_MIN      FILTER_DROP
#define RX_DATA_FILTERS_DEFAULT_ACTION_MAX      FILTER_FW_HANDLE
#define RX_DATA_FILTERS_DEFAULT_ACTION_DEF      FILTER_DROP

#define RX_DATA_FILTERS_FILTER_OFFSET_DEF       0
#define RX_DATA_FILTERS_FILTER_OFFSET_MIN       0
#define RX_DATA_FILTERS_FILTER_OFFSET_MAX       255

#define RX_DATA_FILTERS_FILTER_MASK_DEF         ""
#define RX_DATA_FILTERS_FILTER_MASK_LEN_DEF     0

#define RX_DATA_FILTERS_FILTER_PATTERN_DEF      ""
#define RX_DATA_FILTERS_FILTER_PATTERN_LEN_DEF  0

#define TX_DATA_NUMBER_OF_DATA_QUEUES_MIN       1
#define TX_DATA_NUMBER_OF_DATA_QUEUES_MAX       10
#define TX_DATA_NUMBER_OF_DATA_QUEUES_DEF       4

#define TX_DATA_CREDIT_CALC_TIMOEUT_DEF			100
#define TX_DATA_CREDIT_CALC_TIMOEUT_MIN			20
#define TX_DATA_CREDIT_CALC_TIMOEUT_MAX			1000

#define TX_DATA_FRAC_OF_LIFE_TIME_TO_DROP_DEF	50
#define TX_DATA_FRAC_OF_LIFE_TIME_TO_DROP_MIN	1   /* 0% means we drop everything... so make it 1 */
#define TX_DATA_FRAC_OF_LIFE_TIME_TO_DROP_MAX	100 /* don't drop anything (unless time expired)   */

#define TX_DATA_ADM_CTRL_DELAY_DUE_TO_MEDIUM_OVER_USAGE_DEF				FALSE
#define TX_DATA_ADM_CTRL_DELAY_DUE_TO_MEDIUM_OVER_USAGE_MIN				FALSE
#define TX_DATA_ADM_CTRL_DELAY_DUE_TO_MEDIUM_OVER_USAGE_MAX				TRUE

#define TX_DATA_ADM_CTRL_DOWN_GRADE_DEF			TRUE
#define TX_DATA_ADM_CTRL_DOWN_GRADE_MIN			FALSE
#define TX_DATA_ADM_CTRL_DOWN_GRADE_MAX			TRUE

#define TRAFFIC_ADM_CONTROL_TIMEOUT_MIN       (10)
#define TRAFFIC_ADM_CONTROL_TIMEOUT_MAX       (10000)
#define TRAFFIC_ADM_CONTROL_TIMEOUT_DEF       (5000)

#define CTRL_DATA_TRAFFIC_THRESHOLD_HIGH_MIN    1           /* Traffic intensity threshold - Measured in packets */
#define CTRL_DATA_TRAFFIC_THRESHOLD_HIGH_MAX    1000
#define CTRL_DATA_TRAFFIC_THRESHOLD_HIGH_DEF    100

#define CTRL_DATA_TRAFFIC_THRESHOLD_LOW_MIN     1           /* Traffic intensity threshold - Measured in packets */
#define CTRL_DATA_TRAFFIC_THRESHOLD_LOW_MAX     1000
#define CTRL_DATA_TRAFFIC_THRESHOLD_LOW_DEF     25

#define CTRL_DATA_TRAFFIC_THRESHOLD_INTERVAL_MIN   50       /* Traffic intensity threshold - Traffic test interval - measured in ms */
#define CTRL_DATA_TRAFFIC_THRESHOLD_INTERVAL_MAX   10000
#define CTRL_DATA_TRAFFIC_THRESHOLD_INTERVAL_DEF   1000

#define CTRL_DATA_TRAFFIC_THRESHOLD_ENABLED_MIN FALSE
#define CTRL_DATA_TRAFFIC_THRESHOLD_ENABLED_MAX TRUE
#define CTRL_DATA_TRAFFIC_THRESHOLD_ENABLED_DEF FALSE

#define TRAFFIC_MONITOR_MIN_INTERVAL_PERCENT_MIN   10
#define TRAFFIC_MONITOR_MIN_INTERVAL_PERCENT_MAX   90
#define TRAFFIC_MONITOR_MIN_INTERVAL_PERCENT_DEF   50

#define CTRL_DATA_CONT_TX_THRESHOLD_MIN  2
#define CTRL_DATA_CONT_TX_THRESHOLD_MAX  256
#define CTRL_DATA_CONT_TX_THRESHOLD_DEF  30

#define CTRL_DATA_STEP_UP_TX_THRESHOLD_MIN    2
#define CTRL_DATA_STEP_UP_TX_THRESHOLD_MAX    256
#define CTRL_DATA_STEP_UP_TX_THRESHOLD_DEF    10

#define CTRL_DATA_FB_SHORT_INTERVAL_MIN         20
#define CTRL_DATA_FB_SHORT_INTERVAL_MAX         2000
#define CTRL_DATA_FB_SHORT_INTERVAL_DEF         50

#define CTRL_DATA_FB_LONG_INTERVAL_MIN          100
#define CTRL_DATA_FB_LONG_INTERVAL_MAX          10000
#define CTRL_DATA_FB_LONG_INTERVAL_DEF          2000

#define RATE_ADAPTATION_TIMEOUT_MIN             1
#define RATE_ADAPTATION_TIMEOUT_MAX             3600
#define RATE_ADAPTATION_TIMEOUT_DEF             300

#define RATE_ADAPT_HIGH_TRSH_AC_VO_MIN			0
#define RATE_ADAPT_HIGH_TRSH_AC_VO_MAX			54	
#define RATE_ADAPT_HIGH_TRSH_AC_VO_DEF			0

#define RATE_ADAPT_HIGH_TRSH_AC_VI_MIN			0
#define RATE_ADAPT_HIGH_TRSH_AC_VI_MAX			54	
#define RATE_ADAPT_HIGH_TRSH_AC_VI_DEF			0

#define RATE_ADAPT_HIGH_TRSH_AC_BE_MIN			0
#define RATE_ADAPT_HIGH_TRSH_AC_BE_MAX			54	
#define RATE_ADAPT_HIGH_TRSH_AC_BE_DEF			0

#define RATE_ADAPT_HIGH_TRSH_AC_BK_MIN			0
#define RATE_ADAPT_HIGH_TRSH_AC_BK_MAX			54	
#define RATE_ADAPT_HIGH_TRSH_AC_BK_DEF			0

#define RATE_ADAPT_LOW_TRSH_AC_VO_MIN			0
#define RATE_ADAPT_LOW_TRSH_AC_VO_MAX			54	
#define RATE_ADAPT_LOW_TRSH_AC_VO_DEF			0

#define RATE_ADAPT_LOW_TRSH_AC_VI_MIN			0
#define RATE_ADAPT_LOW_TRSH_AC_VI_MAX			54	
#define RATE_ADAPT_LOW_TRSH_AC_VI_DEF			0

#define RATE_ADAPT_LOW_TRSH_AC_BE_MIN			0
#define RATE_ADAPT_LOW_TRSH_AC_BE_MAX			54	
#define RATE_ADAPT_LOW_TRSH_AC_BE_DEF			0

#define RATE_ADAPT_LOW_TRSH_AC_BK_MIN			0
#define RATE_ADAPT_LOW_TRSH_AC_BK_MAX			54	
#define RATE_ADAPT_LOW_TRSH_AC_BK_DEF			0

#define CTRL_DATA_RATE_CONTROL_ENABLE_MIN       FALSE
#define CTRL_DATA_RATE_CONTROL_ENABLE_MAX       TRUE
#define CTRL_DATA_RATE_CONTROL_ENABLE_DEF       FALSE

#define CTRL_DATA_FOUR_X_ENABLE_MIN             FALSE
#define CTRL_DATA_FOUR_X_ENABLE_MAX             TRUE
#define CTRL_DATA_FOUR_X_ENABLE_DEF             FALSE

#define CTRL_DATA_RATE_POLICY_USER_SHORT_RETRY_LIMIT_MIN 1
#define CTRL_DATA_RATE_POLICY_USER_SHORT_RETRY_LIMIT_MAX 255
#define CTRL_DATA_RATE_POLICY_USER_SHORT_RETRY_LIMIT_DEF 10

#define CTRL_DATA_RATE_POLICY_USER_LONG_RETRY_LIMIT_MIN 1  
#define CTRL_DATA_RATE_POLICY_USER_LONG_RETRY_LIMIT_MAX 255
#define CTRL_DATA_RATE_POLICY_USER_LONG_RETRY_LIMIT_DEF 4  

#define CTRL_DATA_RATE_POLICY_USER_RETRIES_PER_RATE_CCK_DEF		"1,1,1,1,1,1,1,1,1,1,1,1,1"
#define CTRL_DATA_RATE_POLICY_USER_RETRIES_PER_RATE_PBCC_DEF	"1,1,1,1,1,1,1,1,1,1,1,1,1"
#define CTRL_DATA_RATE_POLICY_USER_RETRIES_PER_RATE_OFDM_DEF	"0,0,0,1,0,0,0,1,0,0,1,1,1"
#define CTRL_DATA_RATE_POLICY_USER_RETRIES_PER_RATE_OFDMA_DEF	"0,0,0,1,0,0,1,0,0,1,0,0,0"

#define CTRL_DATA_RATE_POLICY_SG_SHORT_RETRY_LIMIT_MIN 1
#define CTRL_DATA_RATE_POLICY_SG_SHORT_RETRY_LIMIT_MAX 255
#define CTRL_DATA_RATE_POLICY_SG_SHORT_RETRY_LIMIT_DEF 10

#define CTRL_DATA_RATE_POLICY_SG_LONG_RETRY_LIMIT_MIN 1  
#define CTRL_DATA_RATE_POLICY_SG_LONG_RETRY_LIMIT_MAX 255
#define CTRL_DATA_RATE_POLICY_SG_LONG_RETRY_LIMIT_DEF 4  

#define CTRL_DATA_RATE_POLICY_SG_RETRIES_PER_RATE_CCK_DEF	"1,1,1,1,1,1,1,5,1,1,1,1,1"
#define CTRL_DATA_RATE_POLICY_SG_RETRIES_PER_RATE_PBCC_DEF	"1,1,1,1,1,1,1,5,1,1,1,1,1"
#define CTRL_DATA_RATE_POLICY_SG_RETRIES_PER_RATE_OFDM_DEF	"1,1,1,1,1,1,1,5,1,1,1,1,1"
#define CTRL_DATA_RATE_POLICY_SG_RETRIES_PER_RATE_OFDMA_DEF	"1,1,1,1,1,1,1,5,1,1,1,1,1"

#define CTRL_DATA_RATE_POLICY_RETRIES_PER_RATE_MAX_LEN 100

#define REPORT_SEVERITY_VALUE_MIN               0
#define REPORT_SEVERITY_VALUE_MAX               0xFF
#define REPORT_SEVERITY_VALUE_DEF               0xB8    /* WLAN_SEVERITY_WARNING | WLAN_SEVERITY_ERROR | WLAN_SEVERITY_FATAL_ERROR | WLAN_SEVERITY_CONSOLE */

#define RSN_AUTH_SUITE_MIN                      RSN_AUTH_OPEN
#define RSN_AUTH_SUITE_MAX                      RSN_AUTH_NONE
#define RSN_AUTH_SUITE_DEF                      RSN_AUTH_OPEN

#define RSN_DEFAULT_KEY_ID_MIN                  0
#define RSN_DEFAULT_KEY_ID_MAX                  (DOT11_MAX_DEFAULT_WEP_KEYS - 1)
#define RSN_DEFAULT_KEY_ID_DEF                  0

#define RSN_PMKSA_LIFETIME_MIN					1		  	/* 1 sec */
#define RSN_PMKSA_LIFETIME_MAX					4233600 	/* 49 days in sec */
#define RSN_PMKSA_LIFETIME_DEF					86400  		/* 1 day in sec */

#define RSN_WEP_STATUS_MIN                      0
#define RSN_WEP_STATUS_MAX                      1
#define RSN_WEP_STATUS_DEF                      0

#define RSN_WEPMIXEDMODE_ENABLED_MIN                    0
#define RSN_WEPMIXEDMODE_ENABLED_MAX                    1
#define RSN_WEPMIXEDMODE_ENABLED_DEF                    0

#define RSN_WPAMIXEDMODE_ENABLE_MIN             0
#define RSN_WPAMIXEDMODE_ENABLE_MAX             1
#define RSN_WPAMIXEDMODE_ENABLE_DEF             1


#define RSN_PREAUTH_ENABLE_MIN                  0
#define RSN_PREAUTH_ENABLE_MAX                  1
#define RSN_PREAUTH_ENABLE_DEF                  1

#define RSN_PREAUTH_TIMEOUT_MIN                  500
#define RSN_PREAUTH_TIMEOUT_MAX                  60000
#define RSN_PREAUTH_TIMEOUT_DEF                  2000  /* In mSec units */


#define  RSN_PMKIDCANDLIST_DELAY_MIN            3000
#define  RSN_PMKIDCANDLIST_DELAY_MAX            9000
#define  RSN_PMKIDCANDLIST_DELAY_DEF            4000


/* 4X VALUES */
#define DESIRED_CONCATENATION_ENABLE_DEF        TRUE
#define DESIRED_CWMIN_ENABLE_DEF                TRUE
#define DESIRED_CWCOMBO_ENABLE_DEF              FALSE
#define DESIRED_ACKEMULATION_ENABLE_DEF         FALSE
#define DESIRED_ERP_PROTECTION_ENABLE_DEF       FALSE
#define MAX_CONCAT_SIZE_DEF                     4032
#define IBSS_FOUR_X_MODE_PAYLOAD_SIZE           4032
#define INFRASTRUCTURE_FOUR_X_MODE_PAYLOAD_SIZE 1300
#define NOT_FOUR_X_MODE_PAYLOAD_SIZE            1500

/* SME Values */

#define ENABLE_SME_SCAN_DEF			            1
#define ENABLE_SME_SCAN_MIN                     0
#define ENABLE_SME_SCAN_MAX                     1

#define SME_INTER_SCAN_MIN_DEF                  10000 /* 10 seconds */ 
#define SME_INTER_SCAN_MIN_MIN		            1000
#define SME_INTER_SCAN_MIN_MAX              	3600000

#define SME_INTER_SCAN_MAX_DEF                  60000 /* 60 seconds */ 
#define SME_INTER_SCAN_MAX_MIN		            1000
#define SME_INTER_SCAN_MAX_MAX              	3600000

#define SME_INTER_SCAN_DELTA_DEF                1000 /* 1sec*/ 
#define SME_INTER_SCAN_DELTA_MIN		        100
#define SME_INTER_SCAN_DELTA_MAX              	10000


/*        B\G First Scan Params              */
/*       ----------------------              */
#define SME_SCAN_BG_LIST_BAND_STRING_MAX_SIZE    100
#define SME_SCAN_BG_LIST_BAND_VAL_DEF			"1,2,3,4,5,6,7,8,9,10,11,12,13,14"   /* All chaneels */

#define SME_SCAN_BG_MIN_DWELL_TIME_DEF			30000
#define SME_SCAN_BG_MIN_DWELL_TIME_MIN			100
#define SME_SCAN_BG_MIN_DWELL_TIME_MAX			1000000

#define SME_SCAN_BG_MAX_DWELL_TIME_DEF			60000
#define SME_SCAN_BG_MAX_DWELL_TIME_MIN			100
#define SME_SCAN_BG_MAX_DWELL_TIME_MAX			1000000

#define	SME_SCAN_BG_NUM_PROB_REQ_DEF			3
#define SME_SCAN_BG_NUM_PROB_REQ_MIN			1
#define SME_SCAN_BG_NUM_PROB_REQ_MAX			5

#define SME_SCAN_BG_PROB_REQ_RATE_DEF		    0x2         /* Represented as bitmask */
#define SME_SCAN_BG_PROB_REQ_RATE_MIN			0x1			/* 1M=0x1, 2M=0x2, 5.5M=0x4, 11M=0x8,    */
#define SME_SCAN_BG_NUM_PROB_REQ_RATE_MAX		0x1000 		/* 22M=0x10, 6M=0x20, 9M=0x40, 12M=0x80, */
															/* 18M=0x100, 24M=0x200, 36M=0x400, */
															/* 48M=0x800, 54M=0x1000 */ 

#define SME_SCAN_BG_TX_POWER_DEF				MAX_TX_POWER	/* Dbm/10 Units */
#define SME_SCAN_BG_TX_POWER_MIN				MIN_TX_POWER
#define SME_SCAN_BG_TX_POWER_MAX				MAX_TX_POWER


/*        A First Scan Params              */
/*       ----------------------              */
#define SME_SCAN_A_LIST_BAND_STRING_MAX_SIZE    100
#define SME_SCAN_A_LIST_BAND_VAL_DEF			"36,40,44,48,52,56,60,64"   /* All chaneels */

#define SME_SCAN_A_MIN_DWELL_TIME_DEF			30000
#define SME_SCAN_A_MIN_DWELL_TIME_MIN			100
#define SME_SCAN_A_MIN_DWELL_TIME_MAX			1000000

#define SME_SCAN_A_MAX_DWELL_TIME_DEF			60000
#define SME_SCAN_A_MAX_DWELL_TIME_MIN			100
#define SME_SCAN_A_MAX_DWELL_TIME_MAX			1000000

#define	SME_SCAN_A_NUM_PROB_REQ_DEF				3
#define SME_SCAN_A_NUM_PROB_REQ_MIN				1
#define SME_SCAN_A_NUM_PROB_REQ_MAX				5

#define SME_SCAN_A_PROB_REQ_RATE_DEF		    0x20         /* Represented as bitmask */
#define SME_SCAN_A_PROB_REQ_RATE_MIN			0x20		/* 1M=0x1, 2M=0x2, 5.5M=0x4, 11M=0x8,    */
#define SME_SCAN_A_NUM_PROB_REQ_RATE_MAX		0x1000 		/* 22M=0x10, 6M=0x20, 9M=0x40, 12M=0x80, */
															/* 18M=0x100, 24M=0x200, 36M=0x400, */
															/* 48M=0x800, 54M=0x1000 */ 
#define SME_SCAN_A_TX_POWER_DEF				MAX_TX_POWER	/* Dbm/10 */
#define SME_SCAN_A_TX_POWER_MIN				MIN_TX_POWER
#define SME_SCAN_A_TX_POWER_MAX				MAX_TX_POWER

/* Scan SRV parameters */
#define SCAN_SRV_NUMBER_OF_NO_SCAN_COMPLETE_TO_RECOVERY_DEF		3
#define SCAN_SRV_NUMBER_OF_NO_SCAN_COMPLETE_TO_RECOVERY_MIN		1
#define	SCAN_SRV_NUMBER_OF_NO_SCAN_COMPLETE_TO_RECOVERY_MAX		1000000

#define SCAN_SRV_TRIGGERED_SCAN_TIME_OUT_DEF		50000
#define SCAN_SRV_TRIGGERED_SCAN_TIME_OUT_MIN		0
#define	SCAN_SRV_TRIGGERED_SCAN_TIME_OUT_MAX		0xffffffff

/*
  EEPROM-less support
*/
#define REG_MAC_ADDR_STR_LEN                    17
#define REG_ARP_IP_ADDR_STR_LEN					11
#define REG_MAC_ADDR_PREAMBLE_STR_LEN			9
#define BEACON_FILTER_STRING_MAX_LEN			300

#define HAL_CTRL_EEPROMLESS_ENABLE_DEF          1
#define HAL_CTRL_EEPROMLESS_ENABLE_MIN          0
#define HAL_CTRL_EEPROMLESS_ENABLE_MAX          1

/* Scanning Channel Values */
#define MAX_CHAN_BITMAP_BYTES                   (26)

#define MAX_CHANNEL_IN_BAND_2_4					14

#define SCAN_CONTROL_TABLE_ENTRY_MIN            (0x00)
#define SCAN_CONTROL_TABLE_ENTRY_MAX            (0xff)
#define SCAN_CONTROL_TABLE_ENTRY_DEF            (0xff)

/* country code reset time out */
#define REGULATORY_DOMAIN_COUNTRY_TIME_RESET_MIN     (1000)       /* 1 sec   */
#define REGULATORY_DOMAIN_COUNTRY_TIME_RESET_MAX     (1000000000) /* 11 days */
#define REGULATORY_DOMAIN_COUNTRY_TIME_RESET_DEF     (60000)      /* 60 Sec  */

/* d/h Enabling */

#define MULTI_REGULATORY_DOMAIN_ENABLED_MIN     (FALSE) /* 802.11d */
#define MULTI_REGULATORY_DOMAIN_ENABLED_MAX     (TRUE)
#define MULTI_REGULATORY_DOMAIN_ENABLED_DEF     (FALSE)

#define SPECTRUM_MANAGEMENT_ENABLED_MIN         (FALSE) /* 802.11h */
#define SPECTRUM_MANAGEMENT_ENABLED_MAX         (TRUE)
#define SPECTRUM_MANAGEMENT_ENABLED_DEF         (FALSE)

/* Tx Power table (Power level to Dbm)*/
#define TX_POWER_LEVEL_TABLE_24                 "21,13,10,7"
#define TX_POWER_LEVEL_TABLE_5                  "20,12,9,6" 

/* Scan concentrator init parameters - default dwell time values for driver passive scan */
#define SCAN_CNCN_DRIVER_DEFAULT_DWELL_TIME_DEF 200000
#define SCAN_CNCN_DRIVER_DEFAULT_DWELL_TIME_MIN 10000
#define SCAN_CNCN_DRIVER_DEFAULT_DWELL_TIME_MAX 500000

#define SCAN_CNCN_MIN_DURATION_FOR_OID_SCANS_DEF 30
#define SCAN_CNCN_MIN_DURATION_FOR_OID_SCANS_MIN 0
#define SCAN_CNCN_MIN_DURATION_FOR_OID_SCANS_MAX 1000000

/* Packet Filtering Define */
#define MIN_NUM_OF_BEACONS_IN_BUFFER 1
#define DEF_NUM_OF_BEACONS_IN_BUFFER 5
#define MAX_NUM_OF_BEACONS_IN_BUFFER 10

/* Soft Gemini Enabling */
#define SOFT_GEMINI_ENABLED_MIN						(SG_ENABLE)
#define SOFT_GEMINI_ENABLED_MAX						(SG_SENSE_NO_ACTIVITY) /* same as Auto*/
#define SOFT_GEMINI_ENABLED_DEF						(SG_DISABLE)	/* we don't use SG_SENSE_ACTIVE*/

#define SOFT_GEMINI_PARAMS_BT_HP_MAXTIME_MIN				(100)  
#define SOFT_GEMINI_PARAMS_BT_HP_MAXTIME_MAX				(15000)
#define SOFT_GEMINI_PARAMS_BT_HP_MAXTIME_DEF				(2000) 

#define SOFT_GEMINI_PARAMS_WLAN_HP_MAX_TIME_MIN				(100)
#define SOFT_GEMINI_PARAMS_WLAN_HP_MAX_TIME_MAX				(15000)
#define SOFT_GEMINI_PARAMS_WLAN_HP_MAX_TIME_DEF				(5000)

#define SOFT_GEMINI_PARAMS_SENSE_DISABLE_TIMER_MIN				(100)
#define SOFT_GEMINI_PARAMS_SENSE_DISABLE_TIMER_MAX				(15000)
#define SOFT_GEMINI_PARAMS_SENSE_DISABLE_TIMER_DEF				(1350)

#define SOFT_GEMINI_PARAMS_PROTECTIVE_RX_TIME_MIN				(10)
#define SOFT_GEMINI_PARAMS_PROTECTIVE_RX_TIME_MAX				(2300)
#define SOFT_GEMINI_PARAMS_PROTECTIVE_RX_TIME_DEF				(1500)

#define SOFT_GEMINI_PARAMS_PROTECTIVE_TX_TIME_MIN				(10)
#define SOFT_GEMINI_PARAMS_PROTECTIVE_TX_TIME_MAX				(2300)
#define SOFT_GEMINI_PARAMS_PROTECTIVE_TX_TIME_DEF				(1500)

#define SOFT_GEMINI_PARAMS_TIMEOUT_NEXT_BT_LP_PACKET_MIN				(400)
#define SOFT_GEMINI_PARAMS_TIMEOUT_NEXT_BT_LP_PACKET_MAX				(10000)
#define SOFT_GEMINI_PARAMS_TIMEOUT_NEXT_BT_LP_PACKET_DEF				(3000)

#define SOFT_GEMINI_PARAMS_SG_ANTENNA_TYPE_MIN				(0)
#define SOFT_GEMINI_PARAMS_SG_ANTENNA_TYPE_MAX				(7)
#define SOFT_GEMINI_PARAMS_SG_ANTENNA_TYPE_DEF				(0)

#define SOFT_GEMINI_PARAMS_SIGNALING_TYPE_MIN				(0)
#define SOFT_GEMINI_PARAMS_SIGNALING_TYPE_MAX				(3)
#define SOFT_GEMINI_PARAMS_SIGNALING_TYPE_DEF				(1)

#define SOFT_GEMINI_PARAMS_AFH_LEVERAGE_ON_MIN					(0)
#define SOFT_GEMINI_PARAMS_AFH_LEVERAGE_ON_MAX					(2)
#define SOFT_GEMINI_PARAMS_AFH_LEVERAGE_ON_DEF					(0)

#define SOFT_GEMINI_PARAMS_NUMBER_QUIET_CYCLE_MIN		(0)  
#define SOFT_GEMINI_PARAMS_NUMBER_QUIET_CYCLE_MAX		(10)
#define SOFT_GEMINI_PARAMS_NUMBER_QUIET_CYCLE_DEF		(0) 

#define SOFT_GEMINI_PARAMS_MAX_NUM_CTS_MIN			(0)  
#define SOFT_GEMINI_PARAMS_MAX_NUM_CTS_MAX			(10)
#define SOFT_GEMINI_PARAMS_MAX_NUM_CTS_DEF			(3) 

#define SOFT_GEMINI_PARAMS_NUMBER_OF_WLAN_PACKETS_MIN			(1)  
#define SOFT_GEMINI_PARAMS_NUMBER_OF_WLAN_PACKETS_MAX			(10)
#define SOFT_GEMINI_PARAMS_NUMBER_OF_WLAN_PACKETS_DEF			(2) 

#define SOFT_GEMINI_PARAMS_NUMBER_OF_BT_PACKETS_MIN			(2)  
#define SOFT_GEMINI_PARAMS_NUMBER_OF_BT_PACKETS_MAX			(10)
#define SOFT_GEMINI_PARAMS_NUMBER_OF_BT_PACKETS_DEF			(2) 

#define SOFT_GEMINI_PARAMS_PROTECTIVE_RX_TIME_FAST_MIN				(10)
#define SOFT_GEMINI_PARAMS_PROTECTIVE_RX_TIME_FAST_MAX				(20000)
#define SOFT_GEMINI_PARAMS_PROTECTIVE_RX_TIME_FAST_DEF				(1500)

#define SOFT_GEMINI_PARAMS_PROTECTIVE_TX_TIME_FAST_MIN				(10)
#define SOFT_GEMINI_PARAMS_PROTECTIVE_TX_TIME_FAST_MAX				(20000)
#define SOFT_GEMINI_PARAMS_PROTECTIVE_TX_TIME_FAST_DEF				(3000)

#define SOFT_GEMINI_PARAMS_CYCLE_TIME_FAST_MIN				(2000)
#define SOFT_GEMINI_PARAMS_CYCLE_TIME_FAST_MAX				(65535)
#define SOFT_GEMINI_PARAMS_CYCLE_TIME_FAST_DEF				(8700)

#define SOFT_GEMINI_PARAMS_RX_FOR_AVALANCHE_MIN			(1)  
#define SOFT_GEMINI_PARAMS_RX_FOR_AVALANCHE_MAX			(255)
#define SOFT_GEMINI_PARAMS_RX_FOR_AVALANCHE_DEF			(5) 

#define SOFT_GEMINI_PARAMS_ELP_HP_MIN			(0)  
#define SOFT_GEMINI_PARAMS_ELP_HP_MAX			(1)
#define SOFT_GEMINI_PARAMS_ELP_HP_DEF			(0) 

#define SOFT_GEMINI_PARAMS_ANTI_STARVE_PERIOD_MIN			(0)  
#define SOFT_GEMINI_PARAMS_ANTI_STARVE_PERIOD_MAX			(15000)
#define SOFT_GEMINI_PARAMS_ANTI_STARVE_PERIOD_DEF			(500) 

#define SOFT_GEMINI_PARAMS_ANTI_STARVE_NUM_CYCLE_MIN			(0)  
#define SOFT_GEMINI_PARAMS_ANTI_STARVE_NUM_CYCLE_MAX			(15)
#define SOFT_GEMINI_PARAMS_ANTI_STARVE_NUM_CYCLE_DEF			(4) 

#define SOFT_GEMINI_PARAMS_ALLOW_PA_SD_MIN			(0)  
#define SOFT_GEMINI_PARAMS_ALLOW_PA_SD_MAX			(1)
#define SOFT_GEMINI_PARAMS_ALLOW_PA_SD_DEF			(1) 

#define SOFT_GEMINI_PARAMS_TIME_BEFORE_BEACON_MIN			(0)  
#define SOFT_GEMINI_PARAMS_TIME_BEFORE_BEACON_MAX			(20000)
#define SOFT_GEMINI_PARAMS_TIME_BEFORE_BEACON_DEF			(6300) 


#define SOFT_GEMINI_PARAMS_HPDM_MAX_TIME_MIN			(0)  
#define SOFT_GEMINI_PARAMS_HPDM_MAX_TIME_MAX			(50000)
#define SOFT_GEMINI_PARAMS_HPDM_MAX_TIME_DEF			(1600) 

#define SOFT_GEMINI_PARAMS_TIME_OUT_NEXT_WLAN_MIN			(100)  
#define SOFT_GEMINI_PARAMS_TIME_OUT_NEXT_WLAN_MAX			(50000)
#define SOFT_GEMINI_PARAMS_TIME_OUT_NEXT_WLAN_DEF			(2550) 

#define SOFT_GEMINI_PARAMS_AUTO_MODE_NO_CTS_MIN			(0)  
#define SOFT_GEMINI_PARAMS_AUTO_MODE_NO_CTS_MAX			(1)
#define SOFT_GEMINI_PARAMS_AUTO_MODE_NO_CTS_DEF			(0) 

#define SOFT_GEMINI_PARAMS_BT_HP_RESPECTED_MIN			(0)  
#define SOFT_GEMINI_PARAMS_BT_HP_RESPECTED_MAX			(20)
#define SOFT_GEMINI_PARAMS_BT_HP_RESPECTED_DEF			(3) 

#define SOFT_GEMINI_PARAMS_WLAN_RX_MIN_RATE_MIN			(0)  
#define SOFT_GEMINI_PARAMS_WLAN_RX_MIN_RATE_MAX			(54)
#define SOFT_GEMINI_PARAMS_WLAN_RX_MIN_RATE_DEF			(24) 


#define SOFT_GEMINI_PARAMS_ACK_MODE_MIN		            	(0)  
#define SOFT_GEMINI_PARAMS_ACK_MODE_MAX		            	(1)
#define SOFT_GEMINI_PARAMS_ACK_MODE_DEF		            	(1) 

#define SOFT_GEMINI_SCAN_NUMBER_OF_PROBE_REQUEST_MIN		(0)  
#define SOFT_GEMINI_SCAN_NUMBER_OF_PROBE_REQUEST_MAX		(255)
#define SOFT_GEMINI_SCAN_NUMBER_OF_PROBE_REQUEST_DEF		(8) 

#define SOFT_GEMINI_SCAN_COMPENSATION_PERCENT_MIN			(0)  
#define SOFT_GEMINI_SCAN_COMPENSATION_PERCENT_MAX			(1000)
#define SOFT_GEMINI_SCAN_COMPENSATION_PERCENT_DEF			(50) 

#define SOFT_GEMINI_SCAN_COMPENSATION_MAX_TIME_MIN			(1)  
#define SOFT_GEMINI_SCAN_COMPENSATION_MAX_TIME_MAX			(1000000)
#define SOFT_GEMINI_SCAN_COMPENSATION_MAX_TIME_DEF			(120000) 

#define SOFT_GEMINI_BSS_LOSS_COMPENSATION_PERCENT_MIN			(0)  
#define SOFT_GEMINI_BSS_LOSS_COMPENSATION_PERCENT_MAX			(1000)
#define SOFT_GEMINI_BSS_LOSS_COMPENSATION_PERCENT_DEF			(100) 

#define WIFI_WMM_PS_MIN			(0)  
#define WIFI_WMM_PS_MAX			(1)
#define WIFI_WMM_PS_DEF			(0) 


/*  TX FLAGS    */
/*--------------*/
#define TX_DATA_MGMT_MSDU           0x0001
#define TX_DATA_DATA_MSDU           0x0002
#define TX_DATA_EAPOL_MSDU          0x0004
#define TX_DATA_NULL_MSDU           0x0008 /* used for sending null frame before and after measuring a non serving channel */
#define TX_DATA_MULTICAST_FRAME     0x0010
#define TX_DATA_FROM_OS             0x0020
#define TX_DATA_IAPP_MSDU           0x0040
#define TX_DATA_PS_POLL             0x0080
#define TX_DATA_ENCRYPT_MSDU        0x0100


/*  TX FLAGS for tx complete 2  - used for requesting txComplete*/
/*--------------*/
#define TX_DATA_USE_TX_COMPLETE     0x01 /* need only for TxComplete indication */
#define TX_DATA_DISCONNECT_TEST     0x02
#define TX_DATA_VO_SYNC_TRIG        0x04
#define TX_DATA_DISASSOC_SYNC_TRIG  0x08
#define TX_DATA_DEAUTH_SYNC_TRIG    0x10



/* Structures definitions */
PACKED_STRUCT( rates_t,

    UINT8       len;
    UINT8       ratesString[MAX_SUPPORTED_RATES];
);

/* Configurable Scan Rate */
#define SCAN_RATE_MODE_B_MIN    (DRV_RATE_1M)
#define SCAN_RATE_MODE_B_MAX    (DRV_RATE_11M)
#define SCAN_RATE_MODE_B_DEF    (DRV_RATE_2M)

#define SCAN_RATE_MODE_G_MIN    (DRV_RATE_1M)
#define SCAN_RATE_MODE_G_MAX    (DRV_RATE_54M)
#define SCAN_RATE_MODE_G_DEF    (DRV_RATE_2M)

#define SCAN_RATE_MODE_A_MIN    (DRV_RATE_6M)
#define SCAN_RATE_MODE_A_MAX    (DRV_RATE_54M)
#define SCAN_RATE_MODE_A_DEF    (DRV_RATE_6M)

/* Probe request number during scan */
#define SCAN_PROBE_REQ_NUMBER_MIN   1
#define SCAN_PROBE_REQ_NUMBER_MAX   7
#define SCAN_PROBE_REQ_NUMBER_DEF   3


/*****************************************************************************
 **         POWER MANAGER MODULE REGISTRY DEFINITIONS                       **
 *****************************************************************************/
/** \enum PowerMode_e */
/* MUST be sync with OS_802_11_POWER_PROFILE */
typedef enum 
{
    POWER_MODE_AUTO,        /**< In this mode the power manager module is toggle states
                             * (ACTIVE, SHORT_DOZE and LONG_DOZE) by its own inner algorithm.
                             */

    POWER_MODE_ACTIVE,      /**< In this mode there is no power save, the host interface & the radio
                             * is always active. The TNET is constantly awake. This mode is used,
                             * for example, when the device is powered from an AC power source,
                             * and provides maximum throughput and minimal latency.
                             */

    POWER_MODE_SHORT_DOZE,  /**< In this mode the system is going to ELP state and awakes (by the
                             * FW) every beacon. The F/W wakes up the host on every Beacon passes
                             * the Beacon to the driver and returns to ELP Doze as soon as possible.
                             */

    POWER_MODE_LONG_DOZE,    /**< In this mode the system is going to ELP state and awakes (by the
                             * FW) every DTIM or listen interval. This mode consumes low power,
                             * while still waking-up for Beacons once in a while. The system spends
                             * a lot of time in ELP-Doze, and the F/W rarely wakes up the host.
                             */

    POWER_MODE_PS_ONLY,     /**< In this mode the system is setting the Ps as ON. 
							 * the ELP state is changing to SHORT or LONG DOZE (According to last configuration). 
							 * Auto mode won't be used here.
                             */

    POWER_MODE_MAX
}PowerMgr_PowerMode_e;


/** \enum PowerMgr_Priority_e */
typedef enum 
{
    POWER_MANAGER_USER_PRIORITY,           /**< indicates the default user priority. */
    POWER_MANAGER_SG_PRIORITY,             /**< Indicate the Soft Gemini priority */
    POWER_MANAGER_PS_POLL_FAILURE_PRIORITY,/**< After receiving the PsPoll failure event */
    POWER_MANAGER_MAX_PRIORITY                                    					
}PowerMgr_Priority_e;


enum PowerMgr_registryDefinitions
{
    POWER_MODE_MIN_VALUE = POWER_MODE_AUTO,
    POWER_MODE_MAX_VALUE = POWER_MODE_LONG_DOZE,
    POWER_MODE_DEF_VALUE = POWER_MODE_AUTO,

    BEACON_RECEIVE_TIME_MIN_VALUE = 10,
    BEACON_RECEIVE_TIME_MAX_VALUE = 1000,
    BEACON_RECEIVE_TIME_DEF_VALUE = 50,

    BASE_BAND_WAKE_UP_TIME_MIN_VALUE = 100,      /* in micro seconds */
    BASE_BAND_WAKE_UP_TIME_MAX_VALUE = 10000,
    BASE_BAND_WAKE_UP_TIME_DEF_VALUE = 2000,

    PLL_LOCK_TIME_MIN_VALUE = 500,
    PLL_LOCK_TIME_MAX_VALUE = 20000,
    PLL_LOCK_TIME_DEF_VALUE = 4000,

    HANGOVER_PERIOD_MIN_VALUE = 5,
    HANGOVER_PERIOD_MAX_VALUE = 255,
    HANGOVER_PERIOD_DEF_VALUE = 5,

    BEACON_LISTEN_INTERVAL_MIN_VALUE = 1,
    BEACON_LISTEN_INTERVAL_MAX_VALUE = 50,
    BEACON_LISTEN_INTERVAL_DEF_VALUE = 1,

    DTIM_LISTEN_INTERVAL_MIN_VALUE = 1,
    DTIM_LISTEN_INTERVAL_MAX_VALUE = 50,
    DTIM_LISTEN_INTERVAL_DEF_VALUE = 1,

    BEACON_FILTERING_MIN_VALUE = 0,
    BEACON_FILTERING_MAX_VALUE = 30,
    BEACON_FILTERING_DEF_VALUE = 10,

    N_CONSECUTIVE_BEACONS_MISSED_MIN_VALUE = 0,
    N_CONSECUTIVE_BEACONS_MISSED_MAX_VALUE = 50,
    N_CONSECUTIVE_BEACONS_MISSED_DEF_VALUE = 1,

    ENTER_TO_802_11_POWER_SAVE_RETRIES_MIN_VALUE = 0,
    ENTER_TO_802_11_POWER_SAVE_RETRIES_MAX_VALUE = 50,
    ENTER_TO_802_11_POWER_SAVE_RETRIES_DEF_VALUE = 5,

    AUTO_POWER_MODE_INTERVAL_MIN_VALUE = 100,
    AUTO_POWER_MODE_INTERVAL_MAX_VALUE = 30000,
    AUTO_POWER_MODE_INTERVAL_DEF_VALUE = 1000,

    AUTO_POWER_MODE_ACTIVE_TH_MIN_VALUE = 2,
    AUTO_POWER_MODE_ACTIVE_TH_MAX_VALUE = 30000,
    AUTO_POWER_MODE_ACTIVE_TH_DEF_VALUE = 15,

    AUTO_POWER_MODE_DOZE_TH_MIN_VALUE = 1,
    AUTO_POWER_MODE_DOZE_TH_MAX_VALUE = 30000,
    AUTO_POWER_MODE_DOZE_TH_DEF_VALUE = 8,

    AUTO_POWER_MODE_DOZE_MODE_MIN_VALUE = POWER_MODE_SHORT_DOZE,
    AUTO_POWER_MODE_DOZE_MODE_MAX_VALUE = POWER_MODE_LONG_DOZE,
    AUTO_POWER_MODE_DOZE_MODE_DEF_VALUE = POWER_MODE_LONG_DOZE,

    DEFAULT_POWER_LEVEL_MIN_VALUE = POWERAUTHO_POLICY_ELP,
    DEFAULT_POWER_LEVEL_MAX_VALUE = POWERAUTHO_POLICY_AWAKE,
    DEFAULT_POWER_LEVEL_DEF_VALUE = POWERAUTHO_POLICY_ELP,

	PS_POWER_LEVEL_MIN_VALUE = POWERAUTHO_POLICY_ELP,
   	PS_POWER_LEVEL_MAX_VALUE = POWERAUTHO_POLICY_AWAKE,
    PS_POWER_LEVEL_DEF_VALUE = POWERAUTHO_POLICY_ELP,

	POWER_MGMNT_MODE_DEF_VALUE = 1,
    POWER_MGMNT_MODE_MIN_VALUE = 0,
    POWER_MGMNT_MODE_MAX_VALUE = 1,

	POWER_MGMNT_NEED_TO_SEND_NULL_PACKET_DEF_VALUE = 1,
    POWER_MGMNT_NEED_TO_SEND_NULL_PACKET_MIN_VALUE = 0,
    POWER_MGMNT_NEED_TO_SEND_NULL_PACKET_MAX_VALUE = 1,

	/*
	 bit14 - "1" send Prob Request in PBCC
	 bit15 - "1" short preamble, "0" long preammle
	 bit0:bit12  Rates 
	 */
	POWER_MGMNT_NULL_PACKET_RATE_MOD_DEF_VALUE =  ((1<<DRV_RATE_1M) | (1<<DRV_RATE_2M)),
    POWER_MGMNT_NULL_PACKET_RATE_MOD_MIN_VALUE = 0,
    POWER_MGMNT_NULL_PACKET_RATE_MOD_MAX_VALUE = 255 ,

	POWER_MGMNT_NUM_NULL_PACKET_RETRY_DEF_VALUE = 5,
    POWER_MGMNT_NUM_NULL_PACKET_RETRY_MIN_VALUE = 1,
    POWER_MGMNT_NUM_NULL_PACKET_RETRY_MAX_VALUE = 255,
};

/*****************************************************************************
 **         END POWER MANAGER MODULE REGISTRY DEFINITIONS                   **
 *****************************************************************************/


typedef enum
{
    ERP_PROTECTION_NONE       = 0,
    ERP_PROTECTION_STANDARD   = 1,
    ERP_PROTECTION_TI_TRICK   = 2
} erpProtectionType_e;


#endif /* _CORE_DEFAULT_PARAMS_H */
