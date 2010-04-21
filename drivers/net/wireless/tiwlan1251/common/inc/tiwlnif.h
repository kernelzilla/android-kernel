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

#ifndef __TIWLNIF_H__
#define __TIWLNIF_H__

#include "osDot11.h"
#ifndef TIWLNAPI_EXPORTS
#include "osTIType.h"
#endif

#define REGDOMAIN_TABLE_SIZE 16

#define MAX_NUM_OF_TX_QUEUES  4


typedef enum _tiwlnOSVersion
{
    tiwlnIsWin95   = 1,
    tiwlnIsWin98   = 2,
    tiwlnIsWinNT   = 3,
    tiwlnIsWin98SE = 4,
    tiwlnIsWinME   = 5,
    tiwlnIsWin2K   = 6,
    tiwlnIsWinXP   = 7
} TIWLN_OS_VERSION;

typedef enum _TIWLN_PRIVACY_MODE
{
    TIWLN_PRIVACY_NONE = 0,
    TIWLN_PRIVACY_WEP,
    TIWLN_PRIVACY_ZCU,
    TIWLN_PRIVACY_EXC
}TIWLN_PRIVACY_MODE;

PACKED_STRUCT( _dbgBufferHdr ,

    tiCHAR*        pBuff;
    tiUINT32*       puIndex;
    tiUINT32        uSize;
);
typedef _dbgBufferHdr DBG_BUFFER_HDR;

PACKED_STRUCT( _TIWLN_REG_RW ,

    tiUINT32 regSize;
    tiUINT32 regAddr;
    tiUINT32 regValue;
);
typedef _TIWLN_REG_RW TIWLN_REG_RW;

PACKED_STRUCT( _ACX_VERSION_DEF , 

    tiUINT8 major;
    tiUINT8 minor;
    tiUINT8 bugfix;
    tiUINT8 subld;
    tiUINT8 build;
);
typedef _ACX_VERSION_DEF acxVersionDef, *pacxVersionDef;

PACKED_STRUCT( _TIWLN_VERSION ,

    acxVersionDef AppVersion;
    acxVersionDef DrvVersion;
    acxVersionDef FWVersion;
    acxVersionDef HWVersion;
    tiUINT32      osNdisVersion;   /* for OS Windows - Ndis, for Linux - WirelessExt ver.*/
);
typedef _TIWLN_VERSION TIWLN_VERSION, *PTIWLN_VERSION;

PACKED_STRUCT( _TIWLN_VERSION_EX ,

    acxVersionDef AppVersion;
    acxVersionDef DrvVersion;
    acxVersionDef FWVersion;
    acxVersionDef HWVersion;
    tiUINT32      osNdisVersion;       /* for OS Windows - Ndis, for Linux - WirelessExt ver.*/
    tiUINT32      extVerSign;            /* Extended version signature*/
    acxVersionDef NVVersion;
);
typedef _TIWLN_VERSION_EX TIWLN_VERSION_EX, *PTIWLN_VERSION_EX;

typedef enum _TIWLN_RATES
{
    TIWLN_RATE_1          = 0x0A,
    TIWLN_RATE_2          = 0x14,
    TIWLN_RATE_5_5        = 0x37,
    TIWLN_RATE_5_5_PBCC   = 0xB7,
    TIWLN_RATE_11         = 0x6E,
    TIWLN_RATE_11_PBCC    = 0xEE,
    TIWLN_RATE_22_PBCC    = 0xDC
} TIWLN_RATES;

typedef enum _TIWLN_DOT11_STATUS
{
    eDot11Idle           = 0,
    eDot11Scaning        = 1,
    eDot11Connecting     = 2,
    eDot11Associated     = 3,
    eDot11Disassociated  = 4,
    eDot11RadioDisabled  = 5,
    eDot11Error          = 1000,
} TIWLN_DOT11_STATUS;

typedef enum _TIWLN_SECURITY_STATE  /* Values are compatible with 802.1x'S IMSTATE*/
{
    eSecurityStateHalted = 0,           /* Security state machine halted*/
    eSecurityStateStarting,         /* state machine is starting*/
    eSecurityStateInitializing,     /* state machine is initializing*/
    eSecurityStateDisabled,         /* state machine is disabled*/
    eSecurityStateNotAuthenticated, /* Not authenticated state*/
    eSecurityStateAuthenticating,       /* Authentication request is sent*/
    eSecurityStateAuthenticated     /* Authenticated state*/
} TIWLN_SECURITY_STATE;

PACKED_STRUCT( _OS_802_11_DRIVER_CAPABILITIES ,

    tiUINT8 EXCVersion; 
);
typedef _OS_802_11_DRIVER_CAPABILITIES OS_802_11_DRIVER_CAPABILITIES;


/* SME SM definitions - Need to update this whenever the SME SM states change */
/* Table is used for "translating" SME SM states into more "simple" states reported by the TI_GetDriverState API call */
typedef enum
{
    SM_STATE_IDLE                           = 0,
    SM_STATE_SCANNING                       = 1,
    SM_STATE_SELECTING                      = 2,
    SM_STATE_CONNECTING                     = 3,
    SM_STATE_CONNECTED                      = 4,
    SM_STATE_QUIET_SCAN                     = 5,
    SM_STATE_INTER_SCAN_TIMEOUT             = 6,
    SM_STATE_ROAMING_QUIET_SCAN             = 7,
    SM_STATE_RADIO_STAND_BY                 = 8,
    SM_STATE_MEASUREMENT                    = 9,
    SM_STATE_POWER_MNGR_PENDS_QUIET_SCAN    = 10
} stateDrvSme_e;

typedef enum
{
    DRIVER_STATE_IDLE                    = 0,
    DRIVER_STATE_SCANNING                = 1,
    DRIVER_STATE_SELECTING               = 2,
    DRIVER_STATE_CONNECTING              = 3,
    DRIVER_STATE_CONNECTED               = 4,
    DRIVER_STATE_DISCONNECTED            = 5,
} driverState_e;

PACKED_STRUCT( _TIWLAN_CONFIG_SCAN_PARAMS ,

    UINT32    uMaxChanScanTime;
    UINT32    uMinChanScanTime;
    UINT32    uEarlyTerminationMode;
    UINT32    uETMaxNumOfAPframes;
    UINT32    uScanInterval;
    UINT32    uNumOfProbeReq;
);
typedef _TIWLAN_CONFIG_SCAN_PARAMS TIWLAN_CONFIG_SCAN_PARAMS;


/* The Tx path delay histogram ranges in msec. */
typedef enum
{
    TX_DELAY_RANGE_MIN        = 0,

    TX_DELAY_RANGE_0_TO_1     = 0,
    TX_DELAY_RANGE_1_TO_10    = 1,
    TX_DELAY_RANGE_10_TO_20   = 2,
    TX_DELAY_RANGE_20_TO_40   = 3,
    TX_DELAY_RANGE_40_TO_60   = 4,
    TX_DELAY_RANGE_60_TO_80   = 5,
    TX_DELAY_RANGE_80_TO_100  = 6,
    TX_DELAY_RANGE_100_TO_200 = 7,
    TX_DELAY_RANGE_ABOVE_200  = 8,

    TX_DELAY_RANGE_MAX        = 8,
    TX_DELAY_RANGES_NUM       = 9,
} TxDelayRanges_e;

#define TX_RETRY_HISTOGRAM_SIZE 16

PACKED_STRUCT( txDataCounters_t ,

    UINT32      XmitOk;                 /* the number of frames that were transferred to TNET without errors */
    UINT32      DirectedBytesXmit;      /* the number of bytes in directed packets that are transmitted without errors */
    UINT32      DirectedFramesXmit;     /* the number of directed packets that are transmitted without errors */
    UINT32      MulticastBytesXmit;     /* the number of bytes in multicast/functional packets that are transmitted without errors.*/
    UINT32      MulticastFramesXmit;    /* the number of multicast/functional packets that are transmitted without errors.*/
    UINT32      BroadcastBytesXmit;     /* the number of bytes in broadcast packets that are transmitted without */
    UINT32      BroadcastFramesXmit;    /* the number of broadcast packets that are transmitted without errors */

    UINT32      RetryHistogram[ TX_RETRY_HISTOGRAM_SIZE ];
                                        /* Histogram counting the number of packets xfered with any retry number */
    UINT32      RetryFailCounter;       /* Number of packets that failed transmission due to retry number exceeded */
    UINT32      TxTimeoutCounter;       /* Number of packets that failed transmission due to lifetime expiry */
    UINT32      NoLinkCounter;          /* Number of packets that failed transmission due to link failure */
    UINT32      OtherFailCounter;       /* Number of packets that failed transmission due to other reasons */
    UINT32      MaxConsecutiveRetryFail;/* Maximum consecutive packets that failed transmission due to retry limit exceeded */

    /*  TX path delay statistics  */
    UINT32      txDelayHistogram[TX_DELAY_RANGES_NUM];/* Histogram of Tx path delay (host + MAC). */
    UINT32      NumPackets;             /* For average calculation - Total packets counted. */
    UINT32      SumTotalDelayMs;        /* For average calculation - the sum of packets total delay. */
    UINT32      SumFWDelayUs;           /* For average calculation - The sum of packets FW delay. */
    UINT32      SumMacDelayUs;          /* For average calculation - the sum of packets MAC delay. */
); 

typedef struct
{
    txDataCounters_t  txCounters[ MAX_NUM_OF_TX_QUEUES ];
} TIWLN_TX_STATISTICS;

PACKED_STRUCT( TIWLN_COUNTERS ,

    UINT32  RecvOk;              /* num of frames that the NIC receives without errors*/
    UINT32  RecvError;           /* num of frames that a NIC receives but does not indicate to the protocols due to errors*/
    UINT32  RecvNoBuffer;        /* num of frames that the NIC cannot receive due to lack of NIC receive buffer space     */
    UINT32  DirectedBytesRecv;   /* num of bytes in directed packets that are received without errors                     */
    UINT32  DirectedFramesRecv;  /* num of directed packets that are received without errors                              */
    UINT32  MulticastBytesRecv;  /* num of bytes in multicast/functional packets that are received without errors         */
    UINT32  MulticastFramesRecv; /* num of multicast/functional packets that are received without errors                  */
    UINT32  BroadcastBytesRecv;  /* num of bytes in broadcast packets that are received without errors.                   */
    UINT32  BroadcastFramesRecv; /* num of broadcast packets that are received without errors.                            */

    UINT32  FragmentsRecv;
    UINT32  FrameDuplicates;
    UINT32  FcsErrors;

    UINT32  BeaconsXmit;
    UINT32  BeaconsRecv;
    UINT32  AssocRejects;
    UINT32  AssocTimeouts;
    UINT32  AuthRejects;
    UINT32  AuthTimeouts;
);

PACKED_STRUCT( _TIWLN_STATISTICS ,

    /**/
    /* config info*/
    /**/
    tiUINT32                        dot11CurrentTxRate;
    tiUINT32                        dot11CurrentChannel;
    OS_802_11_MAC_ADDRESS           currentMACAddress;
    OS_802_11_SSID                  dot11DesiredSSID; 
    OS_802_11_NETWORK_MODE          dot11BSSType;
    OS_802_11_AUTHENTICATION_MODE   AuthenticationMode;
    tiBOOL                          bShortPreambleUsed;
    tiUINT32                        RTSThreshold;
    tiUINT32                        FragmentationThreshold;
    tiBOOL                          bDefaultWEPKeyDefined;
    OS_802_11_WEP_STATUS            WEPStatus;
    tiUINT32                        TxAntenna;
    tiUINT32                        RxAntenna;
    tiUINT32                        TxPowerDbm;
    tiUINT32                        PowerMode;
    tiINT32                         RxLevel;

    /**/
    /* status & AP info*/
    /**/
    TIWLN_DOT11_STATUS  dot11State;
    OS_802_11_BSSID     targetAP; 

    /**/
    /* network layer statistics (except Tx statistics which are handled sparately)*/
    /**/
    TIWLN_COUNTERS tiCounters;

    /**/
    /* other statistics*/
    /**/
    tiUINT32  dwSecuritySuit;           /* Security suit bitmask (see defines)*/
    tiUINT32  dwSecurityState;          /* 802.1x security protocol state*/
    tiUINT32  dwSecurityAuthStatus;     /* Security suit authentication status*/
    tiUINT32  dwFeatureSuit;            /* Additional features suit bitmask (see defines)*/

);
typedef _TIWLN_STATISTICS TIWLN_STATISTICS;

/* Statistics security suit bitmasks*/
#define TIWLN_STAT_SECURITY_RESERVE_1   0x0001
#define TIWLN_STAT_SECURITY_SSN  0x0002

/* Features suit bitmasks*/
#define TIWLN_FEATURE_4XENABLED  0x0001
#define TIWLN_FEATURE_4XACTIVE   0x0002

PACKED_STRUCT( _TIWLN_REGDOMAINS ,

    tiUINT8 Length;
    tiUINT8 Table[REGDOMAIN_TABLE_SIZE];
);
typedef _TIWLN_REGDOMAINS TIWLN_REGDOMAINS;

PACKED_STRUCT( _TIWLN_MEDIUM_USAGE ,

    tiUINT32            MediumUsage;
    tiUINT32            Period;
);
typedef _TIWLN_MEDIUM_USAGE TIWLN_MEDIUM_USAGE;

typedef struct _RADIO_RX_QUALITY
{
    tiINT32             Snr;
    tiINT32             Rssi;
}TIWLN_RADIO_RX_QUALITY;

#define MAX_NUM_DATA_FILTERS                4

PACKED_STRUCT( _TIWLAN_DATA_FILTER_STATISTICS ,

    tiUINT32            UnmatchedPacketsCount;
    tiUINT32            MatchedPacketsCount[MAX_NUM_DATA_FILTERS];
);
typedef _TIWLAN_DATA_FILTER_STATISTICS TIWLAN_DATA_FILTER_STATISTICS;

#define MAX_DATA_FILTER_MASK_LENGTH         8
#define MAX_DATA_FILTER_PATTERN_LENGTH      64

PACKED_STRUCT ( _TIWLAN_DATA_FILTER_REQUEST ,

    tiUINT8             Offset;
    tiUINT8             MaskLength;
    tiUINT8             Mask[MAX_DATA_FILTER_MASK_LENGTH];
    tiUINT8             PatternLength;
    tiUINT8             Pattern[MAX_DATA_FILTER_PATTERN_LENGTH];
);
typedef _TIWLAN_DATA_FILTER_REQUEST TIWLAN_DATA_FILTER_REQUEST;


#define MAX_NUM_PROFILES 4

PACKED_STRUCT( _TIWLN_PROFILE ,

    tiBOOL                          bPresent;
    OS_802_11_SSID                  dot11DesiredSSID; 
    OS_802_11_NETWORK_MODE          dot11BSSType;
    OS_802_11_AUTHENTICATION_MODE   AuthenticationMode;
    OS_802_11_WEP                   WepDefaultKey[4];
    tiUINT32                        WepDefaultKeyID;
    tiBOOL                          bWepDefaultKeySet;
    OS_802_11_WEP_STATUS            WEPStatus;

    /**/
    /* Which one should be included???*/
    /**/
    tiUINT32                        dot11TxRate;
    tiUINT32                        dot11Channel;
    tiBOOL                          bShortPreambleUsed;
    tiBOOL                          bHighSpeed;
    tiUINT32                        RTSThreshold;
    tiUINT32                        FragmentationThreshold;

    tiUINT32                        TxAntenna;
    tiUINT32                        RxAntenna;
    tiUINT32                        TxPowerDbm;
    tiUINT32                        PowerMode;
);
typedef _TIWLN_PROFILE TIWLN_PROFILE, *PTIWLN_PROFILE;


/* SHA1 hash (of certificate)*/
typedef struct _TI_SHA1_HASH
{
    tiUINT8 aHash[20];
} TI_SHA1_HASH;

PACKED_STRUCT( _TIWLAN_ANT_DIVERSITY ,

    tiUINT8   enableRxDiversity;
    tiUINT8   rxSelectedAntenna;
    tiUINT8   enableTxDiversity;
    tiUINT8   txSelectedAntenna;
    tiUINT8   rxTxSharedAnts;
);
typedef _TIWLAN_ANT_DIVERSITY TIWLAN_ANT_DIVERSITY, *PTIWLAN_ANT_DIVERSITY;

#define TI_NUM_OF_SUB_BANDS 5
#define TI_NUM_OF_POWER_LEVEL 4

typedef struct
{
	tiUINT8 uTxPower[TI_NUM_OF_SUB_BANDS][TI_NUM_OF_POWER_LEVEL]; /* Maximun Dbm in Dbm/10 units */
} TIWLAN_POWER_LEVEL_TABLE;

#endif /* __TIWLNIF_H__*/
