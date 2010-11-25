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

#ifndef __PARAM_OUT_H__
#define __PARAM_OUT_H__

#include "osTIType.h"
#include "osDot11.h"
#include "tiwlnif.h"
#include "ratesTypes.h"
#include "scanTypes.h"
#include "bssTypes.h"
#include "roamingMngrTypes.h"
#include "public_commands.h"
#include "public_infoele.h"
#include "public_radio.h"

#ifdef EXC_MODULE_INCLUDED
#include "paramOutExc.h"
#else
#define   EXC_PARAM_FIELDS
#endif
 
#include "paramMng.h"
#include "commonTypes.h"
#include "coreDefaultParams.h"

#define DOT11_MAX_DEFAULT_WEP_KEYS          ( 4  )
#define ACX_64BITS_WEP_KEY_LENGTH_BYTES     ( 5  )
#define ACX_128BITS_WEP_KEY_LENGTH_BYTES    ( 13 )
#define ACX_256BITS_WEP_KEY_LENGTH_BYTES    ( 29 )
#define ACX_MAX_WEP_KEY_LENGTH_BYTES        ( 29 )

#define RX_LEVEL_TABLE_SIZE             (15)
#define SPECIAL_BG_CHANNEL              (14)

#define BEACON_FILTER_STRING_MAX_LEN 300  /*this is the max possible string length from INI file*/
    
#define BEACON_FILTER_IE_TABLE_MIN_SIZE 0 
#define BEACON_FILTER_IE_TABLE_MAX_NUM (6+32)
#define BEACON_FILTER_IE_TABLE_MIN_NUM 0 

#define RX_DATA_FILTER_MAX_MASK_SIZE            (8)
#define RX_DATA_FILTER_MAX_PATTERN_SIZE         (64)
#define RX_DATA_FILTER_MAX_FIELD_PATTERNS       (8)
#define RX_DATA_FILTER_FILTER_BOUNDARY          (256)

#define RX_DATA_FILTER_FLAG_NO_BIT_MASK         (0)
#define RX_DATA_FILTER_FLAG_USE_BIT_MASK        (1)
#define RX_DATA_FILTER_FLAG_IP_HEADER           (0)
#define RX_DATA_FILTER_FLAG_ETHERNET_HEADER     (2)

#define RX_DATA_FILTER_ETHERNET_HEADER_BOUNDARY (14)


/* Soft gemini  values */

#define NUM_OF_RATES_IN_SG MAX_NUM_OF_TX_RATES_IN_CLASS /* all rates (13)... */
#define SG_RATES_STRING_MAX_DEF 100
#define SG_RATES_DEF "0,0,1,0,0,1,1,1,0,1,1,1,1" /* all rates but 1,2,6,9,22 */
#define NUM_OF_CONFIG_PARAMS_IN_SG 28
#define NUM_OF_STATUS_PARAMS_IN_SG 28


/*used by UtilInfoCodeQueryInformation , UtilInfoCodeSetInformation*/
#define VAL_TX_POWER_VALUE			100
#define VAL_NETWORK_TYPE			101
#define VAL_AP_TX_POWER_LEVEL	    102
/* #define VAL_COUNTRY_CODE    	        103 */ 
/* #define VAL_REG_DOMAIN_BAND_24	    104 */
/* #define VAL_REG_DOMAIN_BAND_50	    105 */
#define VAL_PACKET_BURSTING			106
#define VAL_MIXED_MODE				107
#define VAL_PRIVACY_MODE			108
#define VAL_EXC_SECURITY			109
#define VAL_DEFAULT_KEY_ID			110
#define VAL_AP_SUPPORT_CHANELS 		111



typedef enum
{
    DRAFT_5_AND_EARLIER = 5,
    DRAFT_6_AND_LATER   = 6

} draftNumber_t;

PACKED_STRUCT( ratePair_t,

  rate_e    maxBasic;
  rate_e    maxActive;
);

typedef enum
{
    RTS_CTS_DISABLED = 0,
    RTS_CTS_ENABLED  = 1

} RtsCtsStatus_e;

/* Parameters Structures Definitions per parameter type */
typedef enum
{
    AUTH_LEGACY_OPEN_SYSTEM     = 0,
    AUTH_LEGACY_SHARED_KEY      = 1,
    AUTH_LEGACY_AUTO_SWITCH     = 2,
    AUTH_LEGACY_RESERVED1       = 128,
    AUTH_LEGACY_NONE            = 255,
} legacyAuthType_e;

typedef enum
{
    CONNECTION_NONE             = 0,
    CONNECTION_INFRA            = 1,
    CONNECTION_IBSS             = 2,
    CONNECTION_SELF             = 3,
} connectionType_e;

typedef enum
{
    RADIO_IN_STAND_BY           = 0,
    RADIO_OUT_OF_STAND_BY       = 1,
}radioStandByState_t;

/**** Regulatory Domain module types ****/

/* Scan Control Table for 2.4-G band type */
PACKED_STRUCT( scanControlTable24_t,

    UINT8       tableString[NUM_OF_CHANNELS_24];
);

/* Scan Control Table for 5G-band type */
PACKED_STRUCT( scanControlTable5_t,

    UINT8       tableString[A_5G_BAND_NUM_CHANNELS];
);

/* Scan Control Table type */
PACKED_STRUCT( scanControlTable_t,

    scanControlTable5_t     ScanControlTable5;
    scanControlTable24_t    ScanControlTable24;
);

PACKED_STRUCT( country_t,

    UINT8		elementId;
	UINT8		len;
    countryIE_t countryIE;
);

PACKED_STRUCT( channelPair_t,

    UINT8   firstChennelNum;
    UINT8   NumOfChannels;
);

typedef enum
{
    ACTIVE_SCANNING     = 0,
    PASSIVE_SCANNING    = 1,
} regulatoryDomain_scanOption_e;

PACKED_STRUCT( regulatoryDomainParam_t,

    UINT8*      pChannelBitMap;
    UINT8       channelCnt;
    INT8        txPower;
);

PACKED_STRUCT( powerCapability_t,

    UINT8       minTxPower;
    UINT8       maxTxPower;
);


/* SoftGemini module init parameters */
typedef struct
{
    SoftGeminiEnableModes_e SoftGeminiEnable;
	UINT8					SoftGeminiRate[NUM_OF_RATES_IN_SG];
    SoftGeminiParam_t		SoftGeminiParam;
	UINT8					scanNumOfProbeRequest;
	UINT32					scanCompensationPercent;			
	UINT32					scanCompensationMaxTime;
	UINT32					BSSLossCompensationPercent;
 } SoftGeminiInitParams_t;

typedef enum
{
    PHY_UNKNOWN         = 0,
    PHY_FH              = 1,
    PHY_DSS             = 2,
    PHY_UN_USED         = 3,
    PHY_OFDM            = 4,
    PHY_HIGH_RATE_DSS   = 5,
    PHY_ERP             = 6
} phyType_e;


typedef enum
{
    CLOSE           = 0,
    OPEN_NOTIFY     = 1,
    OPEN_EAPOL      = 2,
    OPEN            = 3,
    MAX_NUM_OF_RX_PORT_STATUS,
} portStatus_e;

typedef enum
{
    TX_DATA_CLOSED = 0,
    TX_DATA_OPEN   = 1

} txDataHalInterfaceStatus_t;


typedef enum
{
    GWSI_PENDING   = 0,
    GWSI_OPEN   = 1
} txDataGwsiInterfaceStatus_e;


typedef enum
{
    DRIVER_STATUS_IDLE              = 0,
    DRIVER_STATUS_RUNNING           = 1,
} driverStatus_e;

typedef enum
{
    OS_ABS_LAYER    = 0,
    RSN             = 1,
} eapolDestination_e;

/* enumerator for PRE_AUTH event */
typedef enum
{
   RSN_PRE_AUTH_START,
   RSN_PRE_AUTH_END,
} preAuthStatusEvent_e;


typedef enum
{
    STATUS_SCANNING         = 0,
    STATUS_SCAN_COMPLETE    = 1,
} scanStatus_e;

typedef enum
{
    SCAN_DISABLED   = 0,	/* FALSE*/
    SCAN_ENABLED    = 1,	/* TRUE*/
	SKIP_NEXT_SCAN	= 2		/* Skip only one next coming scan, then set this parameter to TRUE*/
} scanEnabledOptions_e;




PACKED_STRUCT( rxDataCounters_t,

    UINT32      RecvOk;                 /* the number of frames that the NIC receives without errors */
    UINT32      DirectedBytesRecv;      /* the number of bytes in directed packets that are received without errors */
    UINT32      DirectedFramesRecv;     /* the number of directed packets that are received without errors */
    UINT32      MulticastBytesRecv;     /* the number of bytes in multicast/functional packets that are received without errors */
    UINT32      MulticastFramesRecv;    /* the number of multicast/functional packets that are received without errors */
    UINT32      BroadcastBytesRecv;     /* the number of bytes in broadcast packets that are received without errors. */
    UINT32      BroadcastFramesRecv;    /* the number of broadcast packets that are received without errors. */
    UINT32      LastSecBytesRecv;       /* the number of bytes received without errors during last second */

);

typedef struct rxDataFilterRequest_t
{
    UINT8       offset;
    UINT8       maskLength;
    UINT8       patternLength;
    UINT8       mask[RX_DATA_FILTER_MAX_MASK_SIZE];
    UINT8       pattern[RX_DATA_FILTER_MAX_PATTERN_SIZE];
} rxDataFilterRequest_t;

typedef struct rxDataFilterFieldPattern_t
{
    UINT8       offset;
    UINT8       length;
    UINT8       flag;
    UINT8       pattern[RX_DATA_FILTER_MAX_PATTERN_SIZE];
    UINT8       mask[RX_DATA_FILTER_MAX_PATTERN_SIZE];
} rxDataFilterFieldPattern_t;

PACKED_STRUCT( ctrlDataCounters_t,

    UINT32      icvFailCounter;
    UINT32      keyNotFoundCounter;
    UINT32      MicFailureCounter;

);


typedef struct 
{
	PowerMgr_PowerMode_e	PowerMode;
	PowerMgr_Priority_e 	powerMngPriority; 
}PowerMgr_PowerMode_t;


typedef struct 
{
	void	*handler;
	void	*callback; 
}QoS_renegVoiceTspecReq_t;

/* Authentication/encryption capability */
#define MAX_AUTH_ENCR_PAIR 13

typedef struct 
{
	externalAuthMode_e  authenticationMode;
	cipherSuite_e       cipherSuite;

} authEncrPairList_t;

typedef struct 
{
	UINT32                 NoOfPMKIDs;
	UINT32                 NoOfAuthEncrPairSupported;
	authEncrPairList_t     authEncrPairs[MAX_AUTH_ENCR_PAIR];

} rsnAuthEncrCapability_t;

typedef struct 
{
	UINT32              numOfPreAuthBssids;
	macAddress_t     	*listOfPreAuthBssid;

} rsnPreAuthBssidList_t;


PACKED_STRUCT( signal_t,

    INT32       rssi;
    UINT8       snr;
);

PACKED_STRUCT( rateMask_t,

    UINT32  basicRateMask;
    UINT32  supportedRateMask;
);

PACKED_STRUCT( assocInformation_t,

    UINT8        *assocRespBuffer;
    UINT32       assocRespLen;
    UINT8        *assocReqBuffer;
    UINT32       assocReqLen;

);

/* QOS Parameters Structure */

typedef struct
{
    macAddress_t    siteMacAddress;
    BOOL            priority;
} siteMgr_prioritySite_t;

/*MULTIPLE QUEUES STRUCTURE */




#define TX_POLICY_FLAGS_TRUNCATE          0x1
#define TX_POLICY_FLAGS_PEEAMBLE_OVERRIDE 0x2
#define TX_POLICY_FLAGS_SHORT_PREAMBLE    0x4


typedef struct{
	UINT32 thresholdCross;                /* high or low */
	UINT32 thresholdCrossDirection;       /* direction of crossing */
} trafficIntensityThresholdCross_t;

/************************************/
/*      QOS edcf params             */
/************************************/

/*
#define CW_MIN_DEF                         15
#define CW_MIN_MAX                         31
#define CW_MAX_DEF                         1023
*/
#define CW_MIN_DEF                         4 /* the power of 2 - cwMin = 2^4-1 = 15 */
#define CW_MIN_MAX                         5 /* the power of 2 - cwMax = 2^5-1 = 31 */
#define CW_MAX_DEF                         10

#define AIFS_DEF                            2
#define NO_RX_TIME_OUT                      0
#define NO_RX_ACK_POLICY                    0
#define DATA_DCF                            0    /* MSDUs are sent completely including retrys - normal legacy traffic */
#define QOS_DATA_EDCF                       1    /* MPDUs are sent according to TXOP limits - */
#define RETRY_PREEMPTION_DISABLE            0
#define QOS_CONTROL_TAG_MASK                0x0007
#define QOS_CONTROL_EOSP_MASK                0x0010



/* this enum is used for the different txRateClass_t which are components of txRatePolicy_t */
typedef enum {
	USER_RATE_CLASS		= 0,
	SG_RATE_CLASS		= 1,
	NUM_OF_RATE_CLASS_CLIENTS = 2
} rateClassClients_e;

typedef struct {
	rateClassClients_e	clientID;
	UINT32				clientRateMask;
}	rateClassRateMask_t;


typedef enum{
    AC_ACTIVE = 0,
    AC_NOT_ACTIVE,
}acActive;


typedef struct
{
	UINT8	*buffer;
	UINT16	bufLength;
	UINT8	isBeacon; 	/* If true, Beacon packet is returned, otherwise it is Probe Response */
} BufferParameters_t;



typedef struct{
	UINT32		trafficAdmCtrlResponseTimeout;
    BOOL        trafficAdmCtrlUseFixedMsduSize;
}trafficAdmCtrlInitParams_t;

typedef struct{
    BOOL       wmeEnable;
    BOOL       trafficAdmCtrlEnable;
    BOOL       qosTagZeroConverHeader;
	UINT8      PacketBurstEnable;
	UINT32     PacketBurstTxOpLimit;
    UINT32     TxOpLimit[MAX_NUM_OF_AC];
    UINT32     MsduLifeTime[MAX_NUM_OF_AC];
    rxTimeOut_t     rxTimeOut;
    UINT8      ShortRetryLimit[MAX_NUM_OF_AC];
    UINT8      LongRetryLimit[MAX_NUM_OF_AC];
    UINT16      TxQueueSize[MAX_NUM_OF_TX_QUEUES];
    UINT8   desiredWmeAcPsMode[MAX_NUM_OF_AC];       /* wme per ac power save mode */
    qOvFlowPolicy_e   QueueOvFlowPolicy[MAX_NUM_OF_TX_QUEUES];
	UINT8       acAckPolicy[MAX_NUM_OF_AC];          /* ack policy per AC */
    trafficAdmCtrlInitParams_t	trafficAdmCtrlInitParams;
	UINT8			desiredPsMode;						 /* The desired PS mode of the station */
	UINT8				desiredMaxSpLen;

}QosMngrInitParams_t;



/*END OF MULTIPLE QUEUES STRUCTURE*/


typedef struct
{
	UINT16		bufferSize;
	UINT8		*buffer;
} applicationConfigBuffer_t;

typedef struct
{
    macAddress_t	bssID;
    UINT16			channel;
} apChannelPair_t;

typedef struct
{
    apChannelPair_t	*apChannelPairs;
    UINT16      	numOfEntries;
} neighbor_AP_t;

typedef struct
{
    /* One channel max duration time. (time slot 0 - 65000) */
    UINT16          maxChannelDuration;
    /* One channel max duration time. (time slot 0 - 65000) */
    UINT16          minChannelDuration;
    /* 0 = Stay until max duration time. 1 = Terminate scan in
    a channel upon a reception of Prob-Res or Beacon. 2 = Terminate scan
    in a channel upon a reception of any frame*/
    UINT8           earlyTerminationMode;
    /* number of AP frames (beacon/probe_resp) to trigger Early termination.
    Applicable only when EarlyTerminationMode = 1 */
    UINT8           eTMaxNumOfAPframes;
    /* Number of probe request transmitted on each channel */
    UINT8           numOfProbeReq;

} periodicScanParams_t;


typedef struct
{	
	UINT16 		channelNum;
	BOOL		channelValidity;
	radioBand_e		band;
} channelValidity_t;

typedef struct
{
	BOOL 	channelValidity; /*TRUE-valid, FALSE-invalid */
	UINT8	maxTxPowerDbm; 		/* In Dbm/10 units */
}	channelCapabilityRet_t;

typedef struct
{
	UINT8		*listOfChannels;
	UINT8		sizeOfList;
} supportedChannels_t;

typedef struct
{
	regulatoryDomain_scanOption_e 	scanOption; /* Passive or Active */
	UINT8	channelNum; 	
	radioBand_e                     band; 		
}	channelCapabilityReq_t;

typedef struct
{
	UINT16	minDFS_channelNum; 	
	UINT16	maxDFS_channelNum; 	
}	DFS_ChannelRange_t;

typedef struct
{
	txDataCounters_t 	*pTxDataCounters;
	UINT8				acID;
}	reportTsStatisticsReq_t;

typedef struct
{
	UINT16				vadTimerEnabled;
	UINT16				vadTimerDuration;
} txDataVadTimerParams_t;

/* General Parameters Structure */

typedef struct{
    UINT32              paramType;
    UINT32              paramLength;

    union
    {
        /* Driver General section */
        driverStatus_e          driverStatus;

        /* HAL Control section */
        UINT8                   halCtrlCtsToSelf;
        UINT8                   halCtrlTxPowerDbm;

        /* site manager section */
        UINT8                   siteMgrDesiredChannel;
        macAddress_t            siteMgrDesiredBSSID;
        ssid_t                  siteMgrDesiredSSID;
        bssType_e               siteMgrDesiredBSSType;
        ratePair_t              siteMgrDesiredRatePair;
        rates_t                 siteMgrDesiredBasicRateSet;
        rates_t                 siteMgrDesiredSupportedRateSet;
        rateMask_t              siteMgrCurrentRateMask;
        UINT8                   siteMgrDesiredTxRate;
        UINT8                   siteMgrCurrentTxRate;
        modulationType_e        siteMgrDesiredModulationType;
        UINT16                  siteMgrDesiredBeaconInterval;
        preamble_e              siteMgrDesiredPreambleType;
        preamble_e              siteMgrCurrentPreambleType;
        radioType_e             siteMgrRadioType;
        radioBand_e             siteMgrRadioBand;
        OS_802_11_BSSID_LIST_EX *pSiteMgrBssidList;
        OS_802_11_BSSID_EX      *pSiteMgrSelectedSiteInfo;
        OS_802_11_BSSID         *pSiteMgrPrimarySiteDesc;
        dot11mode_e             siteMgrDot11Mode;
        dot11mode_e             siteMgrDot11OperationalMode;
        draftNumber_t           siteMgrUseDraftNum;
        UINT8                   siteMgrCurrentChannel;
        ssid_t                  siteMgrCurrentSSID;
		bssType_e				siteMgrCurrentBSSType;
        modulationType_e        siteMgrCurrentModulationType;
        slotTime_e              siteMgrSlotTime;
        signal_t                siteMgrCurrentSignal;
        UINT8                   siteMgrNumberOfSites;
        TIWLN_COUNTERS          siteMgrTiWlanCounters;
        BOOL                    siteMgrBuiltInTestStatus;
        UINT8                   siteMgrFwVersion[FW_VERSION_LEN]; /* Firmware version - null terminated string*/
        e2Version_t             siteMgrEEpromVersion;             /* EEPROM version*/
        UINT32                  siteMgrDisAssocReason;
        UINT32                  siteMgrNextDtimTimeStamp;
        UINT16                  siteMgrSiteCapability;
        BOOL                    siteMgrFourxParam;
        UINT16                  beaconInterval;
        UINT8                   APTxPower;
        BOOL                    siteMgrQuietScanInProcess;
        BOOL                    siteMgrScanSliceCurrentlyActive;
        UINT8                   siteMgrRoamingRssiGapThreshold;
        UINT8                   timeStamp[8];
        BOOL                    siteMgrBeaconRecv;
        UINT32                  siteMgrDtimPeriod;
        INT32                   siteMgrCurrentRssi;
        UINT8                   siteMgrIndexOfDesiredSiteEntry;
        UINT8                    *pSiteMgrDesiredSiteEntry;
        UINT8                   siteMgrCurrentTsfTimeStamp[8];
        UINT8                   siteMgrUsrConfigTxPower;


        OS_802_11_CONFIGURATION *pSiteMgrConfiguration;
        siteMgr_prioritySite_t  siteMgrPrioritySite;
		BufferParameters_t		siteMgrLastBeacon;
		UINT8					siteMgrDesiredBeaconFilterState;
		BOOL					siteMgrAllowTxPowerCheck;

        /* SME SM section */
        scanStatus_e            smeSmScanStatus;
		scanEnabledOptions_e	smeSMScanEnabled;
        TIWLN_DOT11_STATUS      smeSmConnectionStatus;
        UINT8                   smeSmState;

        /* connection SM section */
        UINT32                  connSelfTimeout;

        /* auth SM section */
        UINT32                  authResponseTimeout;

        /* assoc SM section */
        UINT32                  assocResponseTimeout;
#ifndef GWSI_LIB
        OS_802_11_ASSOCIATION_INFORMATION      assocAssociationInformation;
#endif /* GWSI_LIB */
		
        /* RSN section */
        BOOL                    rsnPrivacyOptionImplemented;
        authSuite_e             rsnDesiredAuthType;
        OS_802_11_KEY           rsnOsKey;
        rsnAuthEncrCapability_t *pRsnAuthEncrCapability;
        UINT32                  rsnNoOfPMKIDs;
        OS_802_11_PMKID         rsnPMKIDList;
        UINT32                  rsnWPAPromoteFlags;
        UINT32                  rsnWPAMixedModeSupport;
        UINT32                  rsnAuthState; /* supp_1XStates */
        cipherSuite_e           rsnEncryptionStatus;
        UINT8                   rsnHwEncDecrEnable; /* 0- disable, 1- enable*/
        securityKeys_t          *pRsnKey;
        UINT8                   rsnDefaultKeyID;

        externalAuthMode_e      rsnExtAuthneticationMode;
        BOOL                    rsnMixedMode;
		BOOL					rsnPreAuthStatus;
		macAddress_t			rsnApMac;
        OS_802_11_EAP_TYPES     eapType;
        BOOL                    wpa_802_1x_AkmExists;


        /* Rx Data section */
        rxDataCounters_t        rxDataCounters;
        BOOL                    rxDataFilterEnableDisable;
        TIWLAN_DATA_FILTER_REQUEST rxDataFilterRequest;

        /* Tx Data section */
        portStatus_e            txDataPortStatus;
        txDataCounters_t        *pTxDataCounters;
		reportTsStatisticsReq_t tsMetricsCounters;
        OS_802_11_THRESHOLD_CROSS_PARAMS  txDataMediumUsageThreshold;
		txDataHalInterfaceStatus_t  txDataHalInterfaceStatus;
        UINT8                       txDataEncryptionFieldSize;

        /* Ctrl Data section */
        ctrlDataCounters_t      ctrlDataCounters;
        BOOL                    ctrlDataRateControlEnable;
        BOOL                    ctrlDataPowerSaveEnable;
        BOOL                    ctrlDataPowerSaveForce;
        BOOL                    ctrlDataFourXEnable;
        BOOL                    ctrlDatapowerSaveEnhanceAlgorithm;
        erpProtectionType_e     ctrlDataIbssProtecionType;
        RtsCtsStatus_e          ctrlDataRtsCtsStatus;
        BOOL                    ctrlDataProtectionEnabled;
        BOOL                    ctrlDataCerruentFourXstate;

        macAddress_t            ctrlDataCurrentBSSID;
        bssType_e               ctrlDataCurrentBssType;
        UINT32                  ctrlDataCurrentRateMask;
        rate_e                  ctrlDataCurrentBasicRate;
        preamble_e              ctrlDataCurrentPreambleType;
        rate_e                  ctrlDataCurrentActiveRate;
        macAddress_t            ctrlDataDeviceMacAddress;
        STREAM_TRAFFIC_PROPERTIES   ctrlDataUpOfStream;
		clsfr_tableEntry_t		ctrlDataClsfrInsertTable;
        clsfrTypeAndSupport     ctrlDataClsfrType;
        OS_802_11_THRESHOLD_CROSS_PARAMS  ctrlDataRateThreshold;
		rateClassClients_e		ctrlDataRateClassID;
		rateClassRateMask_t		ctrlDataRateClassMask;

 		ULONG					ctrlDataTrafficIntensityEventsFlag;
		OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS ctrlDataTrafficIntensityThresholds;

        connectionType_e        connType;

        /* MLME SM section */
        legacyAuthType_e        mlmeLegacyAuthType;
        legacyAuthType_e        authLegacyAuthType;
        BOOL                    mlmeReAssoc;


        BOOL                    rxDataExcludeUnencrypted;
        eapolDestination_e      rxDataEapolDestination;
        portStatus_e            rxDataPortStatus;

        BOOL                    txDataCurrentPrivacyInvokedMode;
        BOOL                    txDataEapolEncryptionStatus;
        UINT32                  txDataPollApPacketsFromACid;      /* AC to poll AP packets from */

        modulationType_e        ctrlDataCurrentModulationType;
        modulationType_e        ctrlDataCurrentBasicModulationType;
        UINT32                  ctrlDataBasicRateBitMask;

        /* regulatory Domain section */
        regulatoryDomainParam_t regulatoryDomainParam;
        UINT8                   channel;
        country_t*              pCountry;
        UINT8                   pCountryString[COUNTRY_STRING_LEN];
        BOOL                    spectrumManagementEnabled;
        BOOL                    regulatoryDomainEnabled;
        powerCapability_t       powerCapability;
        UINT8*                  pSupportedChannel;
        UINT8                   powerConstraint;
        UINT8                   desiredTxPower; /* The desired Tx power inforced by the User (Utility),
                                                 or The desired Tx power (in Dbm) as forced by teh OS */
        UINT8                   ExternTxPowerPreferred; /*for other extern elements that want        
                                                        to effect the transmit power*/
		powerLevelTable_t		powerLevelTable;
		channelValidity_t		channelValidity;
		channelCapabilityRet_t	channelCapabilityRet;
		channelCapabilityReq_t	channelCapabilityReq;
		supportedChannels_t		supportedChannels;					
        BOOL                    enableDisable_802_11d;
        BOOL                    enableDisable_802_11h;
		BOOL					bActivateTempPowerFix;
		BOOL					bIsCountryFound;
		BOOL					bIsChannelSupprted;
        DFS_ChannelRange_t      DFS_ChannelRange;
        radioBand_e             eRadioBand;

        /* Measurement Manager section */
		UINT32					measurementEnableDisableStatus;
        UINT16					measurementTrafficThreshold;
		UINT16					measurementMaxDuration;
        interogateCmdCBParams_t     interogateCmdCBParams;


        /* soft Gemini section */
        SoftGeminiEnableModes_e		SoftGeminiEnable;
		UINT8						SoftGeminiRate[NUM_OF_RATES_IN_SG];
        UINT32						SoftGeminiParamArray[NUM_OF_CONFIG_PARAMS_IN_SG];

		/* case EXC MODULE INCLUDED */
		EXC_PARAM_FIELDS

        /* Application Config Parameters Manager */
		applicationConfigBuffer_t	applicationConfigBuffer;
		roamingMngrConfigParams_t	roamingConfigBuffer;
		UINT32						roamingTriggerType;
		UINT32						roamingConnStatus;
        bssList_t*              pScanBssList;
        scan_Params_t*          pScanParams;

        /* tx data qos related parameters */
        txDataQosParams_t           txDataQosParams;

        txDataVadTimerParams_t		txDataVadTimerParams;

        /* QOS Manager */
        qosProtocols_e              qosSiteProtocol;
		UINT8						qosPacketBurstEnb;     /* Packet Burst Enable */
		dot11mode_e					qosMngrOperationalMode;
		UINT8					desiredPsMode;
		UINT8					currentPsMode;
		TspecConfigure_t 		TspecConfigure;

        /* Qos params from Os */
		OS_802_11_QOS_RX_TIMEOUT_PARAMS	rxTimeOut;

        OS_802_11_QOS_PARAMS        qosOsParams;
		OS_802_11_AC_QOS_PARAMS		qosApQosParams;
		
        /* AP Qos Capabilities */
        OS_802_11_AP_QOS_CAPABILITIES_PARAMS qosApCapabilities;

        /* Qos current AC status */
        OS_802_11_AC_UPSD_STATUS_PARAMS   qosCurrentAcStatus;

        OS_802_11_QOS_DELETE_TSPEC_PARAMS   qosDelTspecRequest;
        OS_802_11_QOS_TSPEC_PARAMS     qosAddTspecRequest;
		QoS_renegVoiceTspecReq_t	   qosRenegotiateTspecRequest;

        OS_802_11_QOS_TSPEC_PARAMS     qosTspecParameters;

		OS_802_11_THRESHOLD_CROSS_PARAMS  QOSRateThreshold;
		OS_802_11_QOS_DESIRED_PS_MODE	qosDesiredPsMode;

    	/* Power Manager */
        PowerMgr_PowerMode_e    PowerMode;
        powerAutho_PowerPolicy_e PowerSavePowerLevel;
        powerAutho_PowerPolicy_e DefaultPowerLevel;
        PowerMgr_PowerMode_t   	powerMngPowerMode;
        PowerMgr_Priority_e 		powerMngPriority;
        PowerMgr_PowerMode_e	powerMngDozeMode;


        /* txRatePolicy params */
        txRatePolicy_t         TxRatePolicy;
	
        TIWLN_RADIO_RX_QUALITY RxRadioQuality ;
		
        /*PLT MIB*/
        PLT_MIB_t PltMib;

    } content;
} paramInfo_t;


/* paramInfoPartial_t is part of paramInfo_t it is implemented to reduce stack usage */
typedef struct{
    UINT32              paramType;
    UINT32              paramLength;

    union
    {
	  TspecConfigure_t 		TspecConfigure;
	  BOOL					rsnPreAuthStatus;
	  macAddress_t			rsnApMac;
	  cipherSuite_e         rsnEncryptionStatus;
	  BOOL                  rsnMixedMode;

	  /* Application Config Parameters Manager */
	  applicationConfigBuffer_t	applicationConfigBuffer;

	  /* ctrl data section */
	  preamble_e              ctrlDataCurrentPreambleType;

    } content;
}paramInfoPartial_t;

/* Set/get params function prototype */
typedef TI_STATUS (*paramFunc_t)(TI_HANDLE handle, paramInfo_t	*pParam);


typedef enum
{
  MIN_BASIC_TX_RATE         = 0, /* The rate of the CTL & MGMT packets will be the minimal rate advertised in the Basic rate set */
  MAX_BASIC_TX_RATE         = 1, /* The rate of the CTL &MGMT packets will be the maximal rate advertised in the Basic rate set */
  SPECIFIC_TX_RATE          = 2, /* The rate of the CTL & MGMT packets will be according to the configuration in the MgmtCtrlTxRate registry */
} mgmtCtrlTxRateOption_e;



/*-----------------------------------------------------*/
/*      EEPROM-less support                            */
/*-----------------------------------------------------*/
#define MAX_CALL_DATA_REG_NUM                30
#define HW_EEPROM_PRESENTED                  1
#define HW_EEPROM_NOT_PRESENTED              0

PACKED_STRUCT( ELPTable_t,

    UINT8   ClockControl;
    UINT16  ClockWakupTime;
    UINT8   Reserved1;
    UINT16  Reserved2;
    UINT8   A_1_8_Control;
    UINT16  A_1_8_WakeupTime;
    UINT8   VsyncControl;
    UINT16  VsyncWakeupTime;
    UINT8   GcVccControl;
    UINT16  GcVccWakeupTime;
    UINT8   Reserved3;
    UINT16  BBRadioWakeupTime;
    UINT8   Reserved4;
    UINT16  ClockIdleTime;

);

PACKED_STRUCT( MiscTable_t,

    UINT16  TxActivityLED;
    UINT16  InitLED;
    UINT16  DiagLED;
    UINT8   Reserved1;

);


PACKED_STRUCT( PhyRegisters_t,

    UINT16  RegAddress;
    UINT16  RegValue;

);


typedef enum
{
    PS_MODE_ELP         = 0,
    PS_MODE_POWER_DOWN  = 1,
    PS_MODE_ACTIVE      = 2,
    PS_MODE_WAKE_TNET   = 3,
} powerSaveModes_e;


/**************************** Beginning of Init Params ************************************/


typedef struct
{
    UINT8                   siteMgr_radioRxLevel[RX_LEVEL_TABLE_SIZE];
    UINT8                   siteMgr_radioLNA[RX_LEVEL_TABLE_SIZE];
    UINT8                   siteMgr_radioRSSI[RX_LEVEL_TABLE_SIZE];
    UINT32                  factorRSSI; /* for RADIA only */
}radioValues_t;

typedef struct
{
    radioType_e         siteMgr_radioType;
    UINT8               RxLevelTableSize;
    radioValues_t*      pSiteMgr_selectedRadioValues;
    radioValues_t       siteMgr_rfmdRadioValues;
    radioValues_t       siteMgr_maximRadioValues;
    radioValues_t       siteMgr_radiaRadioValues;
}siteMgr_radioValues_t;
	

typedef struct
{
    UINT8                   siteMgrDesiredChannel;
    macAddress_t            siteMgrDesiredBSSID;
    ssid_t                  siteMgrDesiredSSID;
    bssType_e               siteMgrDesiredBSSType;
    dot11mode_e             siteMgrDesiredDot11Mode;
    radioBand_e             siteMgrSupportedBand;
    draftNumber_t           siteMgrUseDraftNum;
    UINT32                  siteMgrRegstryBasicRate[DOT11_MAX_MODE];
    UINT32                  siteMgrRegstrySuppRate[DOT11_MAX_MODE];
    UINT32                  siteMgrRegstryBasicRateMask;
    UINT32                  siteMgrRegstrySuppRateMask;
    rateMask_t              siteMgrCurrentDesiredRateMask;
    ratePair_t              siteMgrDesiredRatePair;
    UINT32                  siteMgrMatchedBasicRateMask;
    UINT32                  siteMgrMatchedSuppRateMask;
    UINT32                  siteMgrMatchedMaxBasicRate;
    UINT32                  siteMgrMatchedMaxActiveRate;
    rate_e                  siteMgrRegstryDesiredTxRate;
    rate_e                  siteMgrCurrentDesiredTxRate;
	mgmtCtrlTxRateOption_e	siteMgrRegstryDesiredMgmtCtrlTxRateOption;
	rate_e					siteMgrRegstryDesiredMgmtCtrlTxRate;
    modulationType_e        siteMgrDesiredModulationType;
    preamble_e              siteMgrDesiredPreambleType;
    slotTime_e              siteMgrDesiredSlotTime;
    UINT16                  siteMgrDesiredBeaconInterval;
    siteMgr_radioValues_t   siteMgrRadioValues;
    UINT8                   siteMgrFwVersion[FW_VERSION_LEN]; /* Firmware version - null terminated string*/
    e2Version_t             siteMgrEEpromVersion;             /* EEPROM version*/
    UINT32                  siteMgrDesiredAtimWindow;
    UINT32                  siteMgrFreq2ChannelTable[SITE_MGR_CHANNEL_MAX+1];
    
    BOOL                    siteMgrDesiredkeepAliveEnable;
    UINT8                   siteMgrExternalConfiguration;
    UINT8                   siteMgrPrivacyMode;
    BOOL                    siteMgrWiFiAdhoc;

	/* TX Power Control parameters */
    UINT32                  TxPowerCheckTime;
    UINT32                  TxPowerControlOn;
    INT32                   TxPowerRssiThresh;
    INT32                   TxPowerRssiRestoreThresh;
	
	beaconFilterParams_t	beaconFilterParams; /*contains the desired state*/

} siteMgrInitParams_t;


/** \struct scan_Params_t
 * \brief This structure contains parameters for a scan operation
 */
typedef struct
{
	UINT8				txPowerDbm;    			/* In units of Dbm/10 */ 
    UINT8               probeReqNumber;         /**< number of probe requests to send (for active scan) */
    rateMask_e          probeRequestRate;       /**< the rate at which to send the probe requests */
    UINT8               numOfChannels;          /**< number of channels for BG (2.4) band */
    UINT8 				channelsList[ MAX_NUMBER_OF_CHANNELS_PER_SCAN ];  /* scan channels list for BG */
	UINT32				minDwellTime;
	UINT32				maxDwellTime;
} sme_scan_Params_t;


typedef struct
{
    BOOL                    EnableFirstConnScan;
    UINT32                  InterScanIntervalMin;
    UINT32                  InterScanIntervalMax;
	UINT32                  InterScanIntervalDelta;
	sme_scan_Params_t   	scanParamsBG;
    sme_scan_Params_t   	scanParamsA;
} smeInitParams_t;

typedef struct
{
    UINT32                  connSelfTimeout;
} connInitParams_t;

typedef struct
{
    UINT32                  authResponseTimeout;
    UINT32                  authMaxRetryCount;
} authInitParams_t;

typedef struct
{
    UINT32                  assocResponseTimeout;
    UINT32                  assocMaxRetryCount;
} assocInitParams_t;

typedef struct
{
	UINT8				highRateThreshold;
	UINT8				lowRateThreshold;
	BOOL				enableEvent;
}tspecsRateParameters_t;

typedef struct
{
    UINT8                   contTxPacketsThreshold;
    UINT8                   stepUpTxPacketsThreshold;
    UINT32                  ctrlDataFBShortInterval;
    UINT32                  ctrlDataFBLongInterval;
    UINT32                  rateAdapt_timeout;
	tspecsRateParameters_t	tspecsRateParameters[MAX_NUM_OF_AC];

}rateAdaptationInitParam_t;


typedef struct
{
    BOOL                    ctrlDataPowerSaveEnhanceAlgorithm;
    UINT16                  ctrlDataPowerSaveTimeOut;
    UINT8                   ctrlDataPowerSaveTxThreshold;
    UINT8                   ctrlDataPowerSaveRxThreshold;

}powerSaveInitParams_t;

typedef struct
{
    BOOL                    desiredConcatenationEnable;
    BOOL                    desiredCWMinEnable;
    BOOL                    desiredCWComboEnable;
    BOOL                    desiredAckEmulationEnable;
    BOOL                    desiredERP_ProtectionEnable;
    UINT32                  desiredMaxConcatSize;
    UINT16                  desiredCWMin;
    UINT16                  desiredCWMax;
}fourXInitParams_t;


typedef struct
{
    UINT32  len;
    rate_e  rateAdaptRatesTable[MAX_SUPPORTED_RATES];
    UINT8   rateAdaptFBTable[MAX_SUPPORTED_RATES];
    UINT8   rateAdaptSUTable[MAX_SUPPORTED_RATES];
} ctrlData_rateAdapt_t;

typedef struct
{
    ctrlData_rateAdapt_t    ctrlDataCckRateTable;
    ctrlData_rateAdapt_t    ctrlDataPbccRateTable;
    ctrlData_rateAdapt_t    ctrlDataOfdmRateTable;
    ctrlData_rateAdapt_t    ctrlDataOfdmARateTable;
} rateTables_t;

typedef struct
{
	UINT8 longRetryLimit;
	UINT8 shortRetryLimit;
}txRatePolicyParams;

typedef struct  
{
	UINT8 txRate[MAX_NUM_OF_TX_RATES_IN_CLASS];
} policyClassRatesArray_t;

typedef struct
{
    BOOL                        ctrlDataRateControlEnable;
    BOOL                        ctrlDataPowerSaveEnable;
    BOOL                        ctrlDataFourXEnable;
    BOOL                        ctrlDataSoftGeminiEnable;
    macAddress_t                ctrlDataDeviceMacAddress;
    rateAdaptationInitParam_t   rateAdaptationInitParam;
    powerSaveInitParams_t       powerSaveInitParams;
    fourXInitParams_t           fourXInitParams;
	clsfr_Params_t				ClsfrInitParam;
    rateTables_t                rateTable;
    erpProtectionType_e         ctrlDataDesiredIbssProtection;
    RtsCtsStatus_e              ctrlDataDesiredCtsRtsStatus;
    OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS   ctrlDataTrafficThreshold;
    BOOL                        ctrlDataTrafficThresholdEnabled;
    txRatePolicyParams          ctrlDataTxRatePolicy      [NUM_OF_RATE_CLASS_CLIENTS];
	policyClassRatesArray_t		policyClassRatesArrayCck  [NUM_OF_RATE_CLASS_CLIENTS];
	policyClassRatesArray_t		policyClassRatesArrayPbcc [NUM_OF_RATE_CLASS_CLIENTS];
	policyClassRatesArray_t		policyClassRatesArrayOfdm [NUM_OF_RATE_CLASS_CLIENTS];
	policyClassRatesArray_t		policyClassRatesArrayOfdmA[NUM_OF_RATE_CLASS_CLIENTS];

} ctrlDataInitParams_t;

typedef struct
{
    UINT8                   txDataNumOfDataQueues;
    UINT32                  uFracOfLifeTimeToDrop;
	UINT32					creditCalculationTimeout;
    BOOL                    admCtrlDelayDueToMediumTimeOverUsage;
	BOOL					admissionDownGradeEnable;
	BOOL					bCreditCalcTimerEnabled;
	/* IMPORT_FROM_4_0_1 */
	BOOL					txDataHostPacketProcessing;

} txDataInitParams_t;

#define MAX_KEYS_NUM                    4

typedef struct
{
    authSuite_e             authSuite;
    BOOL                    privacyOn;
    securityKeys_t          keys[MAX_KEYS_NUM];
    UINT8                   defaultKeyId;
    externalAuthMode_e      externalAuthMode;
    BOOL                    mixedMode;
    BOOL                    WPAMixedModeEnable;
    BOOL                    preAuthSupport;
	UINT32					preAuthTimeout;
} rsnInitParams_t;

typedef enum
{
    RADIO_B_G_INDEX = 0,
    RADIO_A_B_G_INDEX = 1,
    NUM_OF_RADIO_TYPES = 2
} regulatoryDomain_radioIndexType_e;

/* Regulatory Domain module init parameters */
typedef struct
{
    UINT32                      uTimeOutToResetCountryMs;   /* Time after which country code will be reset */
    UINT8                       multiRegulatoryDomainEnabled; /* 802.11d */
    UINT8                       spectrumManagementEnabled; /* 802.11h */
    UINT8                       desiredTxPower;
	UINT8						uTemporaryTxPower;
    scanControlTable_t          desiredScanControlTable;/* for 5 and 2.4 Ghz*/
} regulatoryDomainInitParams_t;

#ifdef EXC_MODULE_INCLUDED
typedef enum
{
    EXC_MODE_DISABLED,
    EXC_MODE_ENABLED,
    EXC_MODE_STANDBY
} excMngr_mode_t;

typedef struct
{
    excMngr_mode_t  excEnabled;
} excMngrParams_t;
#endif

/* Measurement module init parameters */
typedef struct
{
    UINT16              trafficIntensityThreshold;
    UINT16              maxDurationOnNonServingChannel;
#ifdef EXC_MODULE_INCLUDED
    excMngr_mode_t      excEnabled;
#endif
} measurementInitParams_t;

/* Switch Channel Module module init parameters */
typedef struct
{
    BOOL              dot11SpectrumManagementRequired;

} SwitchChannelInitParams_t;

typedef struct
{
  UINT32       qosClassifierTable[MAX_NUM_OF_802_1d_TAGS];
}
clsfrParams_t;

/* WDK pack structure */
#ifdef _WINDOWS
#endif

typedef struct
{
    PowerMgr_PowerMode_e        powerMode;
    UINT32                      beaconReceiveTime;
    UINT8                       hangoverPeriod;
    UINT8                       beaconListenInterval;
    UINT8				 		dtimListenInterval;
    UINT8                       nConsecutiveBeaconsMissed;
    UINT8                       EnterTo802_11PsRetries;
    UINT8                       HwPsPollResponseTimeout;
    UINT16                      autoModeInterval;
    UINT16                      autoModeActiveTH;
    UINT16                      autoModeDozeTH;
    PowerMgr_PowerMode_e        autoModeDozeMode;

    powerAutho_PowerPolicy_e defaultPowerLevel;
	powerAutho_PowerPolicy_e PowerSavePowerLevel;     	

	
	/* powerMgmtConfig IE */
    UINT8						mode;
    UINT8						needToSendNullData;  
    UINT8						numNullPktRetries; 
    UINT8						hangOverPeriod;
    UINT16						NullPktRateModulation; 

	/* PMConfigStruct */
	UINT32						ELPEnable;			/* based on "elpType" */
	UINT32						WakeOnGPIOenable;	/* based on "hwPlatformType" */
	UINT32						BaseBandWakeUpTime;	/* BBWakeUpTime */
	UINT32						PLLlockTime;

	/* ACXWakeUpCondition */
    UINT8						listenInterval;

    /* BET */
    UINT32  MaximalFullBeaconReceptionInterval; /* maximal "beacon periods" between full beacon reception */
    UINT8   BetEnableThreshold;
    UINT8   BetDisableThreshold;
    UINT8	BetEnable;             
    UINT8   MaximumConsecutiveET;

    UINT32						PsPollDeliveryFailureRecoveryPeriod;
}PowerMgrInitParams_t;


typedef struct
{
	UINT32 healthCheckPeriod;
	UINT8  FullRecoveryEnable;
	BOOL   recoveryTriggerEnabled[ MAX_FAILURE_EVENTS ];
} healthMonitorInitParams_t;

typedef struct
{
    BOOL   ignoreDeauthReason0;
} apConnParams_t;

typedef struct
{
    UINT32 passiveScanDwellTime;
    UINT32 minimumDurationBetweenOidScans;
} scanConcentratorInitParams_t;


typedef struct
{
	BOOL                rxDataHostPacketProcessing;
    BOOL                rxDataFiltersEnabled;
    filter_e            rxDataFiltersDefaultAction;
    rxDataFilterRequest_t rxDataFilterRequests[MAX_DATA_FILTERS];
}rxDataInitParams_t;


typedef struct
{
    uint32 activeTimeCnt_Low;
    uint32 activeTimeCnt_Hi;
    uint32 powerDownTimeCnt_Low;
    uint32 powerDownTimeCnt_Hi;
    uint32 elpTimeCnt_Low;
    uint32 elpTimeCnt_Hi;
}PowerConsumptionTimeStat_t;


/* This table is forwarded to the driver upon creation by the Os abstraction layer. */
typedef struct
{
	TnetwDrv_InitParams_t		TnetwDrv_InitParams;

    siteMgrInitParams_t             siteMgrInitParams;
    connInitParams_t                connInitParams;
    authInitParams_t                authInitParams;
    assocInitParams_t               assocInitParams;
    txDataInitParams_t              txDataInitParams;
    ctrlDataInitParams_t            ctrlDataInitParams;
    rsnInitParams_t                 rsnInitParams;
    regulatoryDomainInitParams_t    regulatoryDomainInitParams;
    measurementInitParams_t         measurementInitParams;
    smeInitParams_t                 smeInitParams;
    SoftGeminiInitParams_t          SoftGeminiInitParams;
    QosMngrInitParams_t             qosMngrInitParams;
    clsfrParams_t                   clsfrParams;
#ifdef EXC_MODULE_INCLUDED
    excMngrParams_t                 excMngrParams;
#endif
	SwitchChannelInitParams_t		SwitchChannelInitParams;
	healthMonitorInitParams_t		healthMonitorInitParams;
    apConnParams_t                  apConnParams;
    PowerMgrInitParams_t            PowerMgrInitParams;
    scanConcentratorInitParams_t    scanConcentratorInitParams;
	rxDataInitParams_t              rxDataInitParams;
	BOOL							SendINIBufferToUser;
    /* Traffic Monitor */
    UINT8                           trafficMonitorMinIntervalPercentage;
} initTable_t;

/* WDK end usage of packing */
#ifdef _WINDOWS
#endif

/**************************** End of Init Params ************************************/



#define P_BUFFER_ADD_UINT8(_p_buffer, _uint8)				\
		{														\
			*(tiUINT8 *)(_p_buffer++) = _uint8; 				\
		}

#define P_BUFFER_ADD_UINT16(_p_buffer, _uint16)			  \
		{														  \
			*(tiUINT8 *)(_p_buffer++) = (_uint16 & 0x00FF);		  \
			*(tiUINT8 *)(_p_buffer++) = ((_uint16 & 0xFF00) >> 8);\
		}

#define P_BUFFER_ADD_UINT32(_p_buffer, _uint32)						   \
{																	   \
			*(tiUINT8 *)(_p_buffer++) = (_uint32 & 0x000000FF);		   \
			*(tiUINT8 *)(_p_buffer++) = ((_uint32 & 0x0000FF00) >> 8); \
			*(tiUINT8 *)(_p_buffer++) = ((_uint32 & 0x00FF0000) >> 16);\
			*(tiUINT8 *)(_p_buffer++) = ((_uint32 & 0xFF000000) >> 24);\
		}

#define P_BUFFER_ADD_DATA(_p_buffer, _p_data, _len)		\
		{														\
			memcpy(_p_buffer, _p_data, _len);					\
			_p_buffer += _len;									\
		}		  

#define P_BUFFER_GET_UINT8(_p_buffer, _uint8)					\
		{														\
			_uint8 = *(tiUINT8 *)(_p_buffer++);	 				\
		}

#define P_BUFFER_GET_UINT16(_p_buffer, _uint16)				    \
		{													    \
			_uint16 = *(tiUINT8 *)(_p_buffer++);			    \
			_uint16 |= (*(tiUINT8 *)(_p_buffer++) << 8);		\
		}


#define P_BUFFER_GET_UINT32(_p_buffer, _uint32)					\
		{														\
			_uint32 = *(tiUINT8 *)(_p_buffer++);			    \
			_uint32 |= (*(tiUINT8 *)(_p_buffer++) << 8);		\
			_uint32 |= (*(tiUINT8 *)(_p_buffer++) << 16);		\
			_uint32 |= (*(tiUINT8 *)(_p_buffer++) << 24);		\
		}

#define P_BUFFER_ADD_HDR_PARAMS(_p_buffer, _op, _status)	\
		{														\
			*(tiUINT8 *)(_p_buffer + 0) = (_op & 0x00FF);		\
			*(tiUINT8 *)(_p_buffer + 1) = ((_op & 0xFF00) >> 8);\
			*(tiUINT8 *)(_p_buffer + 2) = _status;				\
			_p_buffer += 3;										\
		}




#endif /* __PARAM_OUT_H__ */

