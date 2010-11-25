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

#ifndef __COMMON_TYPES_H__
#define __COMMON_TYPES_H__

#include "osTIType.h"
#include "osDot11.h"
#include "tiwlnif.h"
#include "paramMng.h"
#include "whalDefaultParams.h"

/*****************************************************************************
 **                                                                         **
 **                                                                         **
 **                       CONSTANTS                                         **
 **                                                                         **
 **                                                                         **
 *****************************************************************************/

/************************************/
/*      Report Module values        */
/************************************/

#define CONFIG_MGR_MODULE_LOG               (0x00)
#define SME_SM_MODULE_LOG                   (0x01)
#define SITE_MGR_MODULE_LOG                 (0x02)
#define CONN_MODULE_LOG                     (0x03)
#define MLME_SM_MODULE_LOG                  (0x04)
#define AUTH_MODULE_LOG                     (0x05)
#define ASSOC_MODULE_LOG                    (0x06)
#define RX_DATA_MODULE_LOG                  (0x07)
#define TX_DATA_MODULE_LOG                  (0x08)
#define CTRL_DATA_MODULE_LOG                (0x09)
#define RSN_MODULE_LOG                      (0x0A)
#define HAL_RX_MODULE_LOG                   (0x0B)
#define HAL_TX_MODULE_LOG                   (0x0C)
#define HAL_CTRL_MODULE_LOG                 (0x0D)
#define HAL_SECURITY_MODULE_LOG             (0x0E)
#define MEM_MGR_MODULE_LOG                  (0x0F)
#define REPORT_MODULE_LOG                   (0x10)
#define SITE_UPDATE_MODULE_LOG              (0x11)
#define REGULATORY_DOMAIN_MODULE_LOG        (0x12)
#define MEASUREMENT_MNGR_MODULE_LOG         (0x13)
#define MEASUREMENT_SRV_MODULE_LOG          (0x14)
#define SOFT_GEMINI_MODULE_LOG              (0x15)
#define SC_MODULE_LOG                       (0x16)
#define EXC_MANAGER_MODULE_LOG              (0x17)
#define ROAMING_MANAGER_MODULE_LOG          (0x18)
#define QOS_MANAGER_MODULE_LOG              (0x19)
#define TRAFFIC_ADM_CTRL_MODULE_LOG         (0x1A)
#define POWER_MANAGER_MODULE_LOG            (0x1B)
#define POWER_CONTROL_MODULE_LOG            (0x1C)      
#define POWER_SERVER_MODULE_LOG             (0x1D)
#define ELP_MODULE_LOG                      (0x1E)
#define SCR_MODULE_LOG                      (0x1F)
#define SCAN_SRV_MODULE_LOG                 (0x20)
#define SCAN_CNCN_MODULE_LOG                (0x21)
#define SCAN_MNGR_MODULE_LOG                (0x22)
#define GWSI_ADAPT_MODULE_LOG               (0x23)
#define GWSI_ADAPT_CB_MODULE_LOG            (0x24)
#define CORE_ADAPT_MODULE_LOG               (0x25)
#define TX_HW_QUEUE_MODULE_LOG              (0x26)
#define TX_CTRL_BLK_MODULE_LOG              (0x27)
#define TX_RESULT_MODULE_LOG                (0x28)
#define TNETW_IF_MODULE_LOG                 (0x29)
#define TNETW_ARBITER_MODULE_LOG            (0x2a)
#define CURR_BSS_MODULE_LOG                 (0x2b)
#define FW_EVENT_MODULE_LOG                 (0x2c)
#define CMD_MBOX_MODULE_LOG                 (0x2d)
#define CMDQUEUE_MODULE_LOG                 (0x2e)
#define EVENT_MBOX_MODULE_LOG               (0x2f)
#define TNETW_DRV_MODULE_LOG                (0x30)
#define TNETW_XFER_MODULE_LOG               (0x31)
#define RECOVERY_MGR_MODULE_LOG				(0x32)
#define RECOVERY_CTRL_MODULE_LOG			(0x33)
#define HW_INIT_MODULE_LOG					(0x34)

#define WLAN_MAX_LOG_MODULES                (HW_INIT_MODULE_LOG + 1)

                                            
/************************************/      
/*      Report Severity values      */  
/************************************/
#define WLAN_SEVERITY_INIT                  1
#define WLAN_SEVERITY_INFORMATION           2
#define WLAN_SEVERITY_WARNING               3
#define WLAN_SEVERITY_ERROR                 4
#define WLAN_SEVERITY_FATAL_ERROR           5
#define WLAN_SEVERITY_SM                    6
#define WLAN_SEVERITY_CONSOLE               7
#define WLAN_SEVERITY_DEBUG_RX              8
#define WLAN_SEVERITY_DEBUG_TX              9
#define WLAN_SEVERITY_DEBUG_CONTROL        10
#define WLAN_SEVERITY_GWSI_RECORDING       11

#define WLAN_MAX_SEVERITIES                (WLAN_SEVERITY_GWSI_RECORDING + 1)



#define     MAX_INFO_ELEMENT_LEN    (32)

#define     IP_ADDR_LEN             4
#define     IP_V4_ADDR_LEN          4
#define     IP_V6_ADDR_LEN          6


#define     MAX_MULTICAST_GROUP_ADDRS   8


#define NUM_OF_NOISE_HISTOGRAM_COUNTERS (8)


#define     MAX_SUPPORTED_RATES         32
#define     RATES_SET_LEN               2

#define     MIN_DEFAULT_KEY_ID          0
#define     MAX_DEFAULT_KEY_ID          3

#define     KEY_RSC_LEN                 8

#define     MIN_KEY_LEN                 5
#define     MAX_KEY_LEN                 32

#define     MAX_IDENTITY_LEN            64
#define     MAX_PASSWD_LEN              128

#define     MAX_NUM_OUI                 16
#define     DOT11_OUI_LEN               3
                            
/* The maximum number of multiple SSIDs that can be used in a single scan command */
#define SCAN_MAX_SSID_NUM           (1)

/* The maximum number of information elements that sent in an active scan probe request 
   including the SSID */
#define SCAN_MAX_INFO_ELEMENTS      (3)

/* WoneIndex value when running as station */
#define STATION_WONE_INDEX          (0)

/* Power translation table definitions */
#define NUM_POWER_LEVELS                    (4)
#define MAX_POWER_LEVEL						(0)
#define MIN_POWER_LEVEL						(NUM_POWER_LEVELS - 1)
#define DBM_TO_TX_POWER_FACTOR				10

/* TX_POWER is in Dbm/10 units */
#define MAX_TX_POWER						255 
#define MIN_TX_POWER						0   
#define NUM_SUB_BANDS_FOR_POWER_TABLE		5
#define BAND_2_4_POWER_TABLE				0

#define MAX_NUM_OF_TX_RATE_CLASS_POLICIES 8 /* max num of policies */
#define MAX_PARAM_MODULE_NUMBER         (GET_PARAM_MODULE_NUMBER(MAX_PARAM_MODULE_PARAM)) /*19*/
#define FW_VERSION_LEN  20
#ifndef TI_STATUS
#define TI_STATUS systemStatus_e
#endif
#define NUM_OF_MAX_TRIPLET_CHANNEL (32)
#define COUNTRY_STRING_LEN 3

/* Definitions for Rx Filter MIB.                                           */
#define PLT_MIB_RX_FILTER_PROMISCOUS_SET    (UINT8) 0x01       /* set  Enable: Forward all frames to host driver*/
#define PLT_MIB_RX_FILTER_PROMISCOUS_CLEAR  (UINT8) 0x00       /* cleared  Disable: Do not orward all frames to the host driver*/
#define PLT_MIB_RX_FILTER_BSSID_SET         (UINT8) 0x02       /* set  filter enabled: receive only those frames that match the BSSID given in the Join command*/
#define PLT_MIB_RX_FILTER_BSSID_CLEAR       (UINT8) 0x00       /* cleared  filter disabled: ignore BSSID in receiving*/



/*****************************************************************************
 **                                                                         **
 **                                                                         **
 **                       ENUMS                                             **
 **                                                                         **
 **                                                                         **
 *****************************************************************************/

typedef enum
{
    txPolicy54 = 0,
    txPolicy48,
    txPolicy36,
    txPolicy24,
    txPolicy22,
    txPolicy18,
    txPolicy12,
    txPolicy11,
    txPolicy9,
    txPolicy6,
    txPolicy5_5,
    txPolicy2,
    txPolicy1,
    MAX_NUM_OF_TX_RATES_IN_CLASS
}txRateClassId_e;


typedef enum{
    DROP_NEW_PACKET = 0,
    DROP_OLD_PACKET
}qOvFlowPolicy_e;

typedef enum
{
   SG_ENABLE                = 0,
   SG_DISABLE                  ,
   SG_SENSE_NO_ACTIVITY        ,
   SG_SENSE_ACTIVE
} SoftGeminiEnableModes_e;


typedef enum
{
    IP_VER_4 = 0,
    IP_VER_6
} IPver_e;

typedef enum
{
    DRV_MODULATION_NONE     = 0,
    DRV_MODULATION_CCK      = 1,
    DRV_MODULATION_PBCC     = 2,
    DRV_MODULATION_QPSK     = 3,
    DRV_MODULATION_OFDM     = 4,
} modulationType_e;

/* tx antenna */
typedef enum
{
    TX_ANTENNA_2        = 0,
    TX_ANTENNA_1        = 1
} txAntenna_e;

/* rx antenna */
typedef enum
{
    RX_ANTENNA_1        = 0,
    RX_ANTENNA_2        = 1,
    RX_ANTENNA_FULL     = 2,
    RX_ANTENNA_PARTIAL  = 3
} rxAntenna_e;

typedef enum
{
    HW_CLOCK_40_MHZ = 40,
    HW_CLOCK_80_MHZ = 80
} hwClock_e;

typedef enum
{
  MAXIM                 = 0,
  RFMD                  = 1,
  RADIA_BG              = 2,
  RADIA_ABG             = 3,
  UNKNOWN_RADIO_TYPE    = 4
} radioType_e;

 /** Available cipher suites for admission control */
typedef enum
{
    RSN_CIPHER_NONE     = 0,        /**< no chpiher suite */
    RSN_CIPHER_WEP      = 1,        /**< WEP-40 chpiher suite */
    RSN_CIPHER_TKIP     = 2,        /**< TKIP chpiher suite */
    RSN_CIPHER_AES_WRAP = 3,        /**< AES WRAP chpiher suite */
    RSN_CIPHER_AES_CCMP = 4,        /**< AES CCMP chpiher suite */
    RSN_CIPHER_WEP104   = 5,        /**< WEP-104 chpiher suite */
    RSN_CIPHER_CKIP     = 6,        /**< CKIP chpiher suite */
    RSN_CIPHER_UNKNOWN  = 255       /**< UNKNOWN chpiher suite */
} cipherSuite_e;


/** RSN supported authentication suites */
typedef enum
{
    RSN_AUTH_OPEN           = 0,        /*< Legacy Open authentication suite */
    RSN_AUTH_SHARED_KEY     = 1,        /*< Legacy Shared Key authentication suite */
    RSN_AUTH_AUTO_SWITCH    = 2,        /*< Automatic authentication suite */
    RSN_AUTH_NONE           = 255       /*< no authentication suite */
} authSuite_e;

/* Available External authentication modes for admission control */
typedef enum
{
   RSN_EXT_AUTH_MODE_OPEN           =   RSN_AUTH_OPEN,
   RSN_EXT_AUTH_MODE_SHARED_KEY     =   RSN_AUTH_SHARED_KEY,
   RSN_EXT_AUTH_MODE_AUTO_SWITCH    =   RSN_AUTH_AUTO_SWITCH,
   RSN_EXT_AUTH_MODE_WPA,
   RSN_EXT_AUTH_MODE_WPAPSK,
   RSN_EXT_AUTH_MODE_WPANONE,
   RSN_EXT_AUTH_MODE_WPA2,
   RSN_EXT_AUTH_MODE_WPA2PSK,
   RSN_EXT_AUTH_MODEMAX          /* Not a real mode, defined as upper bound */
} externalAuthMode_e;

typedef enum
{
    RSN_AUTH_STATUS_INVALID_TYPE                = 0x0001,
    RSN_AUTH_STATUS_TIMEOUT                     = 0x0002,
    RSN_AUTH_STATUS_CHALLENGE_FROM_AP_FAILED    = 0x0003,
    RSN_AUTH_STATUS_CHALLENGE_TO_AP_FAILED      = 0x0004
} authStatus_e;

/************************************/
/*      System return values.       */
/************************************/
#undef OK
#undef NOK

typedef enum
{
    /* System section */
#if !defined(OK) || (OK!=0)
    OK                          = 0,
#endif
#if !defined(NOK) || (NOK!=1)
    NOK                         = 1,
#endif
    /* GWSI status */
    GWSI_FAILED                 = 1,
    PARAM_NOT_SUPPORTED         = 2,
    PARAM_VALUE_NOT_VALID       = 3,
    CONFIGURATION_NOT_VALID     = 4,
    NO_SITE_SELECTED_YET        = 5,
    RE_SCAN_NEEDED              = 6,
    EXTERNAL_SET_PARAM_DENIED   = 7,
    EXTERNAL_GET_PARAM_DENIED   = 8,
    PARAM_MODULE_NUMBER_INVALID = 9,
    STATION_IS_NOT_RUNNING      = 10,
    CARD_IS_NOT_INSTALLED       = 11,

    /* Data path section */
    RX_MIC_FAILURE_ERROR        = 12,
    RX_DECRYPT_FAILURE          = 13,
    RX_STATUS_FAILURE           = 14,
    TX_QUEUE_SELECTED_OK        = 15,
    NO_TX_QUEUE_SELECTED        = 16,
    TX_STATUS_PENDING           = 17,
    TX_STATUS_NO_RESOURCES      = 18,
    TX_STATUS_FAILURE           = 19,
    TX_STATUS_OK                = 20,

    /* 4x section */
    MAKE_CONCATENATION          = 21,
    SEND_ONE_MSDU               = 22,
    DO_NOT_SEND_MSDU            = 23,
    FOUR_X_DISABLE              = 24,

    /* Scanning section */
    NO_COUNTRY                  = 25,
    SCAN_ALREADY_IN_PROGRESS    = 26,
    NO_SCAN_IN_PROGRESS         = 27,

    /* Setting same power */
    TX_POWER_SET_SAME_VALUE  = 28,
    /* changing service channel */
    CHANNEL_CHANGED             = 29,
    SUPPORT_IMMEDIATE_MEASUREMENT_ONLY = 30,
    MEASUREMENT_TYPE_NOT_SUPPORT = 31,
    MEASUREMENT_CAN_NOT_EXECUTED_IN_PARALLEL = 32,
    MEASUREMENT_REQUEST_IGNORED = 33,
    CANNOT_SET_MEASUREMENT_PARAM_WHEN_ACTIVATED = 34,
    CANNOT_SET_CHANNEL_THAT_IS_NOT_SUPPORTED = 35,

    /* rsn */
    STATUS_BAD_KEY_PARAM = 36,
    STATUS_RX_MIC_FAIL   = 37,

    /* site Manager */
    STATUS_FIRST_PRIMARY_SITE_SET = 38,

    /*
    Power Management
    */
    POWER_SAVE_802_11_SUCCESS = 39,
    POWER_SAVE_802_11_FAIL = 40,
    POWER_SAVE_802_11_NOT_ALLOWED = 41,
    PENDING = 42,

    /* GWSI TX packet sending status */
    SEND_COMPLETE_SUCCESS       = 44,
    SEND_COMPLETE_RETRY_EXCEEDED = 45,
    SEND_COMPLETE_LIFETIME_EXCEEDED = 46,
    SEND_COMPLETE_NO_LINK       = 47,
    SEND_COMPLETE_MAC_CRASHED   = 48,
    /*POWER_SAVE_802_11_NOT_ALLOWED = 39,*/
    POWER_SAVE_802_11_IS_CURRENT = 49,

    /* GWSI TX Send-Packet status */
    SEND_PACKET_XFER_DONE   = 50, /* Xfer completed, another packet can be sent, Xfer-Done won't be called. */
    SEND_PACKET_SUCCESS     = 51, /* Xfer in process, another packet can be sent, Xfer-Done will be called. */
    SEND_PACKET_PENDING     = 52, /* Xfer in process, another packet CAN-NOT be sent. Xfer-Done will be called. */
    SEND_PACKET_BUSY        = 53, /* Packet rejected due to queue lack of resources. 
                                     Should be sent again after resources are freed on Tx-complete. */
    SEND_PACKET_ERROR       = 54, /* Packet rejected due to API violation (sending in PENDING state or wrong params. */
    SEND_PACKET_RECOVERY    = 55, /* Recovery happened during Xfer */

    /* QoSMngr */
    TI_WLAN_QOS_RETURN_CODES, /* detailed in tiQosTypes.h */

    /* TNETWIF Return Errors */
    TNETWIF_NONE,
    TNETWIF_OK,
    TNETWIF_COMPLETE,
    TNETWIF_PENDING,
    TNETWIF_ERROR,
    TNETWIF_MORE,

    /* Rx Data Filters */
    RX_NO_AVAILABLE_FILTERS,
    RX_FILTER_ALREADY_EXISTS,
    RX_FILTER_DOES_NOT_EXIST,

	/* Soft Gemini */
	SG_REJECT_MEAS_SG_ACTIVE,

} systemStatus_e;

typedef enum
{
    NO_FAILURE = -1,
    NO_SCAN_COMPLETE_FAILURE = 0,
    MBOX_FAILURE,
    HW_AWAKE_FAILURE,
    BUS_ERROR,
    DEVICE_ERROR,
    TX_STUCK,
    DISCONNECT_TIMEOUT,
    POWER_SAVE_FAILURE,
    MEASUREMENT_FAILURE,
    MAX_FAILURE_EVENTS
} failureEvent_e;

/** \enum TnetWakeOn_e */
typedef enum 
{
    
    TNET_WAKE_ON_BEACON,           /**< Indicate the wake on event of the HW - beacon.
                                    * In this event the HW configure to be awake on every beacon.
                                    */

    TNET_WAKE_ON_DTIM,             /**< Indicate the wake on event of the HW - DTIM. In this event
                                    * the HW configure to be awake on every DITM (configure by the AP).
                                    */

    TNET_WAKE_ON_N_BEACON,          /**< Indicate the wake on event of the HW - listen interval.
                                    * In this event the HW configure to be awake on every
                                    * configured number of beacons.
                                    */

    TNET_WAKE_ON_N_DTIM,            /**< Indicate the wake on event of the HW - listen interval.
                                    * In this event the HW configure to be awake on every
                                    * configured number of beacons.
                                    */

    TNET_WAKE_ON_HOST              /**< Indicate the wake on event of the HW - Host access only
                                    */
                                
}PowerMgr_TnetWakeOn_e;


/** \enum PowerMgr_RequestFor_802_11_PS_e */
typedef enum 
{
    REQUEST_TO_ENTER_POWER_SAVE_802_11,                 /**< request to enter to power save
                                                         * of 802.11
                                                         */

    REQUEST_NOT_TO_CHANGE_POWER_SAVE_802_11,            /**< request to not change the
                                                         * power save of 802.11
                                                         */

    REQUEST_TO_EXIT_POWER_SAVE_802_11                  /**< request to exit from power save
                                                         * of 802.11
                                                         */
}PowerMgr_RequestFor_802_11_PS_e;

typedef enum 
{
    POWER_SAVE_OFF,                  /**< power save of 802.11
                                                         */

    POWER_SAVE_ON ,                     /**< power save on 802.11
                                                         */

    POWER_SAVE_KEEP_CURRENT    /**< power save 802.11 don't change
                                                         */
}PowerMgr_802_11_PsMode_e;

typedef enum 
{
    POWERAUTHO_POLICY_ELP       = 0,
    POWERAUTHO_POLICY_PD        = 1,
    POWERAUTHO_POLICY_AWAKE     = 2,
    POWERAUTHO_POLICY_NUM
} powerAutho_PowerPolicy_e;

typedef enum 
{
    ELPCTRL_MODE_NORMAL = 0,
    ELPCTRL_MODE_KEEP_AWAKE
} elpCtrl_Mode_e;

/*
 * this enum defines the protocol modes of the QOS management object.
 */
typedef enum{
    WME = 0,
    NONE_QOS,
}qosProtocols_e;

typedef enum
{
    RX_PACKET_TYPE_DATA = 0,
    RX_PACKET_TYPE_MANAGEMENT
}rxPacketType_e;

typedef enum
{
    DOT11_B_MODE    = 1,
    DOT11_A_MODE    = 2,
    DOT11_G_MODE    = 3,
    DOT11_DUAL_MODE = 4,

    DOT11_MAX_MODE
} dot11mode_e;

/* hw access method*/
typedef enum
{
    HW_ACCESS_BUS_SLAVE_INDIRECT    = 0,
    HW_ACCESS_BUS_SLAVE_DIRECT      = 1,
    HW_ACCESS_BUS_MASTER            = 2
} hwAccessMethod_e;

/*
 * this enum includes the header converting modes configured to dataCtrl object.
 */
typedef enum{
    NO_CONVERT = 0,
    QOS_CONVERT,
    LEGACY_CONVERT,
}headerConvetMode_e;

/* * this enum defines the admission state configured to dataCtrl object.
 */
typedef enum{
    ADMISSION_NOT_REQUIRED = 0,
    ADMISSION_REQUIRED = 1,
}admissionState_e;


/*****************************************************************************
 **                                                                         **
 **                                                                         **
 **                       TYPEDEFS                                          **
 **                                                                         **
 **                                                                         **
 *****************************************************************************/
/**<
* Callback for 802.11 PS - Success/Fail
*/
typedef void (*ps802_11_NotificationCB_t)(TI_HANDLE module,
                                          TI_STATUS thePsSuccess);

/**<
* Asynchronous init mode callback function type
*/
typedef void (*fnotify_t)(TI_HANDLE module, TI_STATUS status);
                                          
typedef struct
{
    UINT8 txRate[MAX_NUM_OF_TX_RATES_IN_CLASS];
    UINT8 shortRetryLimit;
    UINT8 longRetryLimit;
    UINT8 flags;
}txRateClass_t;

typedef struct
{
    UINT32      numOfRateClasses;
    txRateClass_t rateClass[MAX_NUM_OF_TX_RATE_CLASS_POLICIES];
}txRatePolicy_t;

PACKED_STRUCT( e2Version_t,

    UINT16 major;
    UINT8  minor;
    UINT8  last;
    UINT16 bugfix;
);

PACKED_STRUCT( TripletCahnnel_t,

    UINT8           firstChannelNumber;
    UINT8           numberOfChannels;
    UINT8           maxTxPowerLevel;
);

PACKED_STRUCT( IpAddress_t,

    UINT8 addr[IP_ADDR_LEN];
);

PACKED_STRUCT( informationElementHeader_t,

    UINT8 eleId;
    UINT8 eleLen;
);

PACKED_STRUCT( informationElement_t,

    informationElementHeader_t  hdr;    
    UINT8                       info[MAX_INFO_ELEMENT_LEN];             
);

PACKED_STRUCT( countryIE_t,

    UINT8               CountryString[COUNTRY_STRING_LEN];
    TripletCahnnel_t    tripletChannels[NUM_OF_MAX_TRIPLET_CHANNEL];
);

/* Struct retrieved from NVS */
typedef struct   
{
	UINT8					uDbm[NUM_SUB_BANDS_FOR_POWER_TABLE][NUM_POWER_LEVELS]; 
} powerLevelTable_t;

PACKED_STRUCT( SoftGeminiParam_t,
	UINT32														wlanRxMinRateToRespectBtHp; /* Integer rate number. Note that Fw gets it in Index rate */
    UINT16                                                      btHpMaxTime;                          /* the maximum length of time the BT HP will be respected*/
    UINT16                                                      wlanHpMaxTime;                     /* the maximum length of time the WLAN HP will be respected*/
    UINT16                                                      senseDisableTimer; /* the length of time when working in SENSE mode that the BT needs to be inactive in order to DISABLE the SG*/
    UINT16                                                      protectiveRxTimeBeforeBtHp;
    UINT16                                                      protectiveTxTimeBeforeBtHp; 
    UINT16                                                      protectiveRxTimeBeforeBtHpFastAp;   /*new      range: 10-20000    default: 1500*/
    UINT16                                                      protectiveTxTimeBeforeBtHpFastAp;   /*new      range: 10-20000    default: 3000*/
    UINT16                                                      protectiveWlanCycleTimeForFastAp;   /*new      range: 2000-65535                default: 8700*/
    UINT16                                                      btAntiStarvationPeriod;             /* 0 - 15000Msec */
    UINT16                                                      timeoutNextBtLpPacket;
	UINT16                                                      wakeUpTimeBeforeBeacon;
	UINT16														hpdmMaxGuardTime; 					/* 0-50000 */
    UINT16														timeoutNextWlanPacket;
    UINT8                                                       sgAntennaType;
    UINT8                                                       signalingType; 
    UINT8                                                       afhLeverageOn;                      /* specifies whether to use the AFH information from the BT */
    UINT8                                                       numberQuietCycle;
    UINT8                                                       maxNumCts;       
    UINT8                                                       numberOfWlanPackets;  
    UINT8                                                       numberOfBtPackets;     
    UINT8                                                       numberOfMissedRxForAvalancheTrigger;             /*new      range: 1-255          default: 5*/
    UINT8                                                       wlanElpHpSupport;                                                 /* new     range: 0-1              default: 1*/
    UINT8                                                       btAntiStarvationNumberOfCyclesWithinThePeriod; /* 0 - 15 Cycles */
    UINT8                                                       ackModeDuringBtLpInDualAnt;
    UINT8                                                       allowPaSdToggleDuringBtActivityEnable;
	UINT8														sgAutoModeNoCts;
	UINT8													    numOfBtHpRespectedReq;				 

); /* Parameters directly to FW */

 
PACKED_STRUCT( interogateCmdHdr_t,

    UINT16      id;
    UINT16      len;
);

/* Struct for retrieving powerLevelTable_t with interrogate IE */
typedef struct   
{
	interogateCmdHdr_t      tPowerLevelResCmdHdr;
	powerLevelTable_t		tTable; 
} powerLevelTableInterrogate_t;

typedef struct
{
    interogateCmdHdr_t      noiseHistResCmdHdr;
    UINT32  counters[NUM_OF_NOISE_HISTOGRAM_COUNTERS];
    UINT32  numOfLostCycles;
    UINT32  numOfTxHwGenLostCycles;
    UINT32  numOfRxLostCycles;
} noiseHistogramResults_t;
/*
 * interogateCmdCBParams_t:
 * Note that this structure is used by the GWSI 
 * both for setting (writing to the device) and
 * for retreiving (Reading from the device),
 * while being called with a completion CB
 */
typedef struct
{
    void*       CB_Func;    /* Completion CB function*/
    TI_HANDLE   CB_handle;  /* CB handle*/
    UINT8*      CB_buf;     /* Buffer contains the content to be written or the retrieved content*/
} interogateCmdCBParams_t;


typedef struct
{
    void*       CB_Func;
    TI_HANDLE   CB_handle;
    UINT8*      CB_buf;
} configureCmdCBParams_t;

PACKED_STRUCT( securityKeys_t,

    keyType_e       keyType;                /* key type (WEP, TKIP etc.) */

    UINT32          encLen;
    UINT8           encKey[MAX_KEY_LEN];

    UINT8           micRxKey[MAX_KEY_LEN];
    UINT8           micTxKey[MAX_KEY_LEN];

    UINT32          keyIndex;                       /* id=0 is broadcast key */
    macAddress_t    macAddress;
    UINT8           keyRsc[KEY_RSC_LEN];
);


typedef struct
{
    UINT8   queueID;
    UINT8   channelType;
    UINT8   tsid;
    UINT32  dot11EDCATableMSDULifeTime;
    UINT8   psScheme;
    UINT8   ackPolicy;
    UINT32  APSDConf[2];
} queueTrafficParams_t;

typedef struct
{
    UINT8  ac;
    UINT8  cwMin;
    UINT16 cwMax;
    UINT8  aifsn;
    UINT16 txopLimit;
}acQosParams_t;

PACKED_STRUCT( rxTimeOut_t, 
    UINT16  psPoll;
    UINT16  UPSD;
);

PACKED_STRUCT( QOS_AC_IE_ParametersRecord_t,

    UINT8           ACI_AIFSN;
    UINT8           ECWmin_ECWmax;
    UINT16          TXOPLimit;
);

PACKED_STRUCT( ACParameters_t,

    QOS_AC_IE_ParametersRecord_t        ACBEParametersRecord;
    QOS_AC_IE_ParametersRecord_t        ACBKParametersRecord;
    QOS_AC_IE_ParametersRecord_t        ACVIParametersRecord;
    QOS_AC_IE_ParametersRecord_t        ACVOParametersRecord;
);


typedef struct{
    UINT8          PsMode;             /*  power save mode.        */
    UINT16           TxQueueSize;
    UINT8            QueueIndex;
    qOvFlowPolicy_e  QueueOvFlowPolicy;
    UINT8    ackPolicy;
    UINT32           MsduLifeTime;
}acTrfcCtrl_t;

typedef struct{
    headerConvetMode_e      headerConverMode;                             /* header converting mode        */
    BOOL                    convertTagZeroFrames;                         /* flag for converting zero tags */
    trafficAdmState_e       admissionState;                             /* AC admission state            */
    admissionState_e        admissionRequired;                          /* AC admission is mandatory.    */
    acTrfcType_e            tag_ToAcClsfrTable[MAX_NUM_OF_802_1d_TAGS]; /* tag to AC classification      */
}qosParams_t;

typedef struct{
    acTrfcCtrl_t  acTrfcCtrl;
    qosParams_t   qosParams;
    UINT8       *tsrsArr;
    UINT8         tsrsArrLen;
    UINT8         acID;
}txDataQosParams_t;

typedef struct{
	UINT8                       voiceTspecConfigure;
	UINT8                       videoTspecConfigure;
}TspecConfigure_t;


/*************************************/
/*   TNETW Driver init table.        */
/*************************************/

PACKED_STRUCT(whalCtrl_tx_Queue_t,
    UINT8       numDesc;
    UINT8       priority;
);


PACKED_STRUCT(whalCtrl_init_t,
    UINT8           hwAccessMethod;
    UINT8           maxSitesFragCollect;
    UINT8           packetDetectionThreshold;
    UINT32          blockSize;
    UINT8           rxDescNum;
    UINT8           txDescNum;
    UINT32          nullTemplateSize;
    UINT32          beaconTemplateSize;
    UINT32          probeRequestTemplateSize;
    UINT32          probeResponseTemplateSize;
    UINT32          PsPollTemplateSize;
    UINT32          qosNullDataTemplateSize;
    UINT32          tddRadioCalTimout;
    UINT32          CrtRadioCalTimout;
    int             UseMboxInterrupt;
    int             UseTxDataInterrupt;
    UINT32      TraceBufferSize;
    BOOLEAN         bDoPrint;
    UINT8           StaMacAddress[MAC_ADDR_LEN];
    UINT8           UsePlcpHeader;
    UINT8           numTxQueues;
    whalCtrl_tx_Queue_t tx_attrib_queue[MAX_NUM_OF_TX_QUEUES];
    BOOL            TxFlashEnable;
    UINT8           rxMemBlkNumber;
    UINT8           txMinMemBlkNumber;
    UINT16          txCompleteTimeout;
    UINT8           txCompleteThreshold;
    UINT8      TxBlocksHighPercentPerAc[MAX_NUM_OF_AC];
    UINT8      TxBlocksLowPercentPerAc[MAX_NUM_OF_AC];
    UINT16      BeaconRxTimeout;
    UINT16      BroadcastRxTimeout;
    UINT8       RxBroadcastInPs;
    UINT8       ConsecutivePsPollDeliveryFailureThreshold;
);


PACKED_STRUCT(halCtrlConfigParams_t,
    UINT8                   halCtrlCalibrationChannel2_4;
    UINT8                   halCtrlCalibrationChannel5_0;
    UINT16                  halCtrlRtsThreshold;
    UINT16                  halCtrlFragThreshold;
    UINT32                  halCtrlMaxTxMsduLifetime;
    UINT32                  halCtrlMaxRxMsduLifetime;
    UINT8                   halCtrlRateFallbackRetry;
    UINT16                  halCtrlListenInterval;
    BOOL                    halCtrlEnable4x;
    txAntenna_e             halCtrlTxAntenna;
    rxAntenna_e             halCtrlRxAntenna;
    UINT8                   halCtrlMacClock;
    UINT8                   halCtrlArmClock;
    BOOL                    halCtrlRxEnergyDetection;
    BOOL                    halCtrlTxEnergyDetection;
    BOOL                    halCtrlEepromLessEnable;
    UINT16                  halCtrlBcnRxTime;
    BOOL                    halCtrlRxDisableBroadcast;
    BOOL                    halCtrlRecoveryEnable;
    BOOL                    halCtrlFirmwareDebug;
    /* hardware ACI parameters */
    UINT8                   halCtrlACIMode;
    UINT8                   halCtrlInputCCA;
    UINT8                   halCtrlQualifiedCCA;
    UINT8                   halCtrlStompForRx;
    UINT8                   halCtrlStompForTx;
    UINT8                   halCtrlTxCCA;
    rxTimeOut_t             rxTimeOut;
    UINT8                   halCtrlTxCompleteThreshold;
    BOOL					WiFiWmmPS;

);

typedef struct
{
    UINT16                  gpioBitNumForRadioDisableFeature;
}radioDisableParams_t;

 /* New Power*/
PACKED_STRUCT(PowerSrvInitParams_t,
    /* powerMgmtConfig IE */
    UINT8                       numNullPktRetries; 
    UINT8                       hangOverPeriod;
);

PACKED_STRUCT( scanSrvInitParams_t,
    UINT32                      numberOfNoScanCompleteToRecovery;
    UINT32                      uTriggeredScanTimeOut; /* i.e. split scan */
);

PACKED_STRUCT(reportInitParams_t,
    UINT8   SeverityTable[WLAN_MAX_SEVERITIES];
    UINT8   ModuleTable[WLAN_MAX_LOG_MODULES];
);

PACKED_STRUCT(beaconFilterParams_t,
    UINT8                           desiredState;
    UINT8                           currentState;
    UINT8                           numOfStored;
    UINT8                           IETable[BEACON_FILTER_IE_TABLE_MAX_SIZE]; 
    UINT8                           numOfElements;
    UINT8                           IETableSize;
        
);

PACKED_STRUCT(arpIpFilterParams_t,
    UINT8                           isFilterEnabled;
    IpAddress_t                     arpIpInitParams;    
);

PACKED_STRUCT(macAddrFilterParams_t,
    UINT8                           isFilterEnabled;
    UINT8                           numOfMacAddresses;
    macAddress_t                    macAddrTable[MAX_MULTICAST_GROUP_ADDRS];

);

PACKED_STRUCT(txXferInitParams_t,
    /* Duration in which HW buffer is full until recovery is triggered */
    UINT32                      timeToTxStuckMs; 
);

PACKED_STRUCT(macPreambleParams_t,
    UINT8                       earlyWakeUp;
);

/*
 * TnetwDrv_InitParams_t:
 * This structure encapsulates the initialization data required by the TnetwDrv layer
 */
PACKED_STRUCT(TnetwDrv_InitParams_t,
    whalCtrl_init_t                 whalCtrl_init;
    halCtrlConfigParams_t           halCtrlConfigParams;
    reportInitParams_t              reportParams;
    PowerSrvInitParams_t            PowerSrvInitParams;
    scanSrvInitParams_t             scanSrvInitParams;
    arpIpFilterParams_t             arpIpFilterParams;
    macAddrFilterParams_t           macAddrFilterParams;
    beaconFilterParams_t            beaconFilterParams;
    txXferInitParams_t              txXferInitParams;
    macPreambleParams_t             macPreambleParams;
);

/* PLT*/
typedef struct 
{
    UINT32      FCSErrorCount;  /* increment when an FCS error is detected in a received MPDU*/
    UINT32      TotalFrameCount;/* increment for each packet.*/	
	UINT32      PLCPErrorCount;  /* increment when an PLCP error is detected in a received MPDU*/
    UINT32      SeqNumMissCount; /* Increment when an sequence number of arrived packet is not in order */
    UINT32      SeqNumMissCountRef; /* holds the reference value for seqNumMissCount test start */
}PltRxPer_t;

typedef struct 
{
    UINT32 chID; /* Channel number */
    UINT32 rate;    /* index of the transmit */
    UINT32 InterPacketDelay;/*(ms) */
    UINT32 NumOfFrames;
    UINT32 aPacketLength;
    UINT32 aSeqNumMode; /* fixed / incremented */
    macAddress_t  aPeerMacAddr;
    UINT8 preamble;  /* {PREAMBLE_LONG | PREAMBLE_SHORT } */
    UINT8 band; /* {PBCC_MODULATION_MASK |OFDM_MODULATION_MASK } */
    UINT8 mode; /* {AUTO_SEQ_NUMBER | RANDOM_DATA | ZOZO_DATA} */
    UINT8 aPadding[3];
}PltTxContinues_t;

 /* ChannelBand_t -                                                    */
/*              This structure holds the parameters for TX carrier test and radio tune*/
typedef struct  
{
    UINT32 channel; /* Channel number*/
    UINT32 band;    /* Band */
} ChannelBand_t;


 /* PLT - MIBs structures */
/* TMIB -                                                                   */
/*              Specifies the type of a MIB element                         */
typedef enum
{
    PLT_MIB_dot11StationId = 0x1001,
    PLT_MIB_dot11MaxReceiveLifetime,
    PLT_MIB_dot11SlotTime,
    PLT_MIB_dot11GroupAddressesTable,
    PLT_MIB_dot11WepDefaultKeyId,
    PLT_MIB_dot11CurrentTxPowerLevel,
    PLT_MIB_dot11RTSThreshold,

    PLT_MIB_ctsToSelf = 0x1101,
    PLT_MIB_arpIpAddressesTable,
    PLT_MIB_templateFrame,
    PLT_MIB_rxFilter,
    PLT_MIB_beaconFilterIETable,
    PLT_MIB_beaconFilterEnable,
    PLT_MIB_sleepMode,
    PLT_MIB_wlanWakeUpInterval,
    PLT_MIB_beaconLostCount,
    PLT_MIB_rcpiThreshold,
    PLT_MIB_statisticsTable,
    PLT_MIB_ibssPsConfig,
    PLT_MIB_txRatePolicy,
    PLT_MIB_countersTable,
    PLT_MIB_btCoexsitenceMode,
    PLT_MIB_btCoexistenceParameters
} PLT_MIB_e;


#define PLT_MAX_MULTICAST_GROUP_ADDRS 8

typedef struct 
{
    macAddress_t   GroupTable[PLT_MAX_MULTICAST_GROUP_ADDRS];        
	tiUINT8        bFilteringEnable;
    tiUINT8        nNumberOfAddresses;
}PLT_MIB_GroupAdressTable_t;


typedef struct 
{
    tiUINT32    FilteringEnable;
    UCHAR       addr[IP_V4_ADDR_LEN];
}PLT_MIB_ArpIpAddressesTable_t;


/* TTemplateType -                                                          */
/*              Specifies the type of a templateFrame                       */
typedef enum
{                                       /* Frame type:                  */
    PLT_TEMPLATE_TYPE_BEACON,               /* 0 for beacon template,       */
    PLT_TEMPLATE_TYPE_PROBE_REQUEST,        /* 1 for probe request template,*/
    PLT_TEMPLATE_TYPE_NULL_FRAME,           /* 2 for NULL data frame        */
    PLT_TEMPLATE_TYPE_PROBE_RESPONSE,       /* 3 for probe response frame   */
    PLT_TEMPLATE_TYPE_QOS_NULL_FRAME,       /* 4 for QOS NULL data frame    */
    PLT_TEMPLATE_TYPE_PS_POLL,              /* 5 for PS-Poll frame          */
} PLT_MIB_TemplateType_t;

#define PLT_MIB_TEMPLATE_DATA_MAX_LEN 256
typedef struct 
{
    PLT_MIB_TemplateType_t   FrameType;
    tiUINT32                 Rate;
    tiUINT16                 Length;
    tiUINT8                  Data[PLT_MIB_TEMPLATE_DATA_MAX_LEN];
}PLT_TemplateFrame_t;

typedef struct 
{
    tiUINT32  WakeUpInterval;
    tiUINT8   ListenInterval;    /* Listen interval in unit of the beacon/DTIM */
}PLT_MIB_WlanWakeUpInterval_t;


typedef struct 
{
    tiUINT32 PLCPErrorCount;
    tiUINT32 FCSErrorCount;
}PLT_MIB_CounterTable_t;


/*the max table sized is : ( number of 221 * 8 bytes ) + ( non-221 * 2 bytes )
  Must be synchronized with the size of ACX defined in public_infoele.h interface
  with the FW
*/
#define PLT_MIB_MAX_SIZE_OF_IE_TABLE 112 
/* NOTE: struct is only meant to be used as a pointer reference to an actual*/
/*       buffer. Table size is not a constant and is derived from the buffer*/
/*       size given with the WriteMIB command or readMIBComplete event      */
typedef struct 
{
    UINT8   iNumberOfIEs;       /* number of information elements in table  */
    UINT8   iIETable[PLT_MIB_MAX_SIZE_OF_IE_TABLE];         
}PLT_SBeaconFilterIETable_t;

typedef union 
{
     macAddress_t StationId;
     tiUINT32 MaxReceiveLifeTime;
     UINT32 SlotTime;
     PLT_MIB_GroupAdressTable_t GroupAddressTable;
     tiUINT8   WepDefaultKeyId;
     tiUINT8   PowerLevel;
     tiUINT16  RTSThreshold;
     tiUINT32  CTSToSelfEnable;
     PLT_MIB_ArpIpAddressesTable_t ArpIpAddressesTable;
     PLT_TemplateFrame_t TemplateFrame;
     tiUINT8  RxFilter;
     PLT_MIB_WlanWakeUpInterval_t  WlanWakeUpInterval;
     PLT_MIB_CounterTable_t CounterTable;
     PLT_SBeaconFilterIETable_t BeaconFilter;
     txRatePolicy_t txRatePolicy;
}PLT_MIB_data_u;


typedef struct{
    PLT_MIB_e      aMib;  
    UINT32         Length;
    PLT_MIB_data_u aData;   
}PLT_MIB_t;


#endif /* __COMMON_TYPES_H__ */
