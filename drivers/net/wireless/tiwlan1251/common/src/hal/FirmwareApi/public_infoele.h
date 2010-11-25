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

/** \file public_infoele.h
 *  \brief Contains information element defines/structures used by the FW and host.
 *
 */

#ifndef PUBLIC_INFOELE_H
#define PUBLIC_INFOELE_H


#include "public_types.h"
#include "public_commands.h"
#include "public_radio.h" 


typedef enum
{
    ACX_WAKE_UP_CONDITIONS      = 0x0002,
    ACX_MEM_CFG                 = 0x0003,
    ACX_SLOT                    = 0x0004,
    ACX_QUEUE_HEAD              = 0x0005, /* for MASTER mode only!!!*/
    ACX_AC_CFG                  = 0x0007,
    ACX_MEM_MAP                 = 0x0008,
    ACX_AID                     = 0x000A,
    ACX_RADIO_PARAM             = 0x000B, /* Not in use !!! */
    ACX_CFG                     = 0x000C, /* Not in use !!!*/
    ACX_FW_REV                  = 0x000D,
    ACX_FCS_ERROR_CNT           = 0x000E, /* OBSOLETE (replaced by ACX_ERROR_CNT) !!!*/
    ACX_MEDIUM_USAGE            = 0x000F,
    ACX_RX_CFG                  = 0x0010,
    ACX_TX_QUEUE_CFG            = 0x0011,
    ACX_BSS_IN_PS               = 0x0012, /* for AP only (OBSOLETE???)	*/
    ACX_STATISTICS              = 0x0013, /* Debug API*/
    ACX_PWR_CONSUMPTION_STATISTICS  = 0x0014,
    ACX_FEATURE_CFG             = 0x0015,
    ACX_MISC_CFG                = 0x0017, /* Not in use !!!*/
    ACX_TID_CFG                 = 0x001A,
    ACX_CAL_ASSESSMENT          = 0x001E, /* OBSOLETE !!!*/
    ACX_BEACON_FILTER_OPT       = 0x001F,
    ACX_LOW_RSSI                = 0x0020,
    ACX_NOISE_HIST              = 0x0021,
    ACX_HDK_VERSION             = 0x0022, /* ???*/
    ACX_PD_THRESHOLD            = 0x0023,
    ACX_DATA_PATH_PARAMS        = 0x0024, /* WO*/
	ACX_DATA_PATH_RESP_PARAMS   = 0x0024, /* RO*/
    ACX_CCA_THRESHOLD           = 0x0025,
    ACX_EVENT_MBOX_MASK         = 0x0026,
#ifdef FW_RUNNING_AS_AP
    ACX_DTIM_PERIOD             = 0x0027, /* for AP only !!!*/
#else
    ACX_WR_TBTT_AND_DTIM        = 0x0027, /* STA only*/
#endif
    ACX_ACI_OPTION_CFG          = 0x0029, /* OBSOLETE !!! (for 1251)*/
    ACX_GPIO_CFG                = 0x002A, /* Not in use !!!*/
    ACX_GPIO_SET                = 0x002B, /* Not in use !!!*/
    ACX_PM_CFG                  = 0x002C, /* ??? (To Be Documented)*/
    ACX_CONN_MONIT_PARAMS       = 0x002D,
    ACX_AVERAGE_RSSI            = 0x002E, /* Not in use !!!*/
    ACX_CONS_TX_FAILURE         = 0x002F,
    ACX_BCN_DTIM_OPTIONS        = 0x0031,
    ACX_SG_ENABLE               = 0x0032,
    ACX_SG_CFG                  = 0x0033,
    ACX_ANTENNA_DIVERSITY_CFG   = 0x0035, /* ??? (To Be Documented)*/
	ACX_LOW_SNR					= 0x0037, /* To Be Documented*/
    ACX_BEACON_FILTER_TABLE     = 0x0038,
    ACX_ARP_IP_FILTER           = 0x0039,
    ACX_ROAMING_STATISTICS_TBL  = 0x003B,
    ACX_RATE_POLICY             = 0x003D, 
    ACX_CTS_PROTECTION          = 0x003E, 
    ACX_SLEEP_AUTH              = 0x003F,
	ACX_PREAMBLE_TYPE			= 0x0040,
    ACX_ERROR_CNT               = 0x0041,
	ACX_FW_GEN_FRAME_RATES      = 0x0042,
	ACX_IBSS_FILTER				= 0x0044,
    ACX_SERVICE_PERIOD_TIMEOUT  = 0x0045,
	ACX_TSF_INFO                = 0x0046,
    ACX_CONFIG_PS_WMM           = 0x0049,
    ACX_ENABLE_RX_DATA_FILTER   = 0x004A,
    ACX_SET_RX_DATA_FILTER      = 0x004B,
    ACX_GET_DATA_FILTER_STATISTICS = 0x004C,
	ACX_POWER_LEVEL_TABLE       = 0x004D,
    ACX_BET_ENABLE              = 0x0050,
    DOT11_STATION_ID           =  0x1001,
    DOT11_RX_MSDU_LIFE_TIME     = 0x1004,
    DOT11_CUR_TX_PWR           =  0x100D,
    DOT11_DEFAULT_KEY          =  0x1010,
    DOT11_RX_DOT11_MODE        =  0x1012,
    DOT11_RTS_THRESHOLD        =  0x1013, 
    DOT11_GROUP_ADDRESS_TBL    =  0x1014,
     
    MAX_DOT11_IE = DOT11_GROUP_ADDRESS_TBL,
	
    MAX_IE = 0xFFFF   /*force enumeration to 16bits*/
} InfoElement_enum;


#ifdef HOST_COMPILE
typedef uint16 InfoElement_e;
#else
typedef InfoElement_enum InfoElement_e;
#endif


typedef struct
{
    InfoElement_e id;
    uint16 length;
    uint32 dataLoc; /*use this to point to for following variable-length data*/
} InfoElement_t;


typedef struct 
{
    uint16 id;
    uint16 len;
} EleHdrStruct;


#ifdef HOST_COMPILE
#define INFO_ELE_HDR    EleHdrStruct    EleHdr;
#else
#define INFO_ELE_HDR
#endif

/******************************************************************************

    Name:	ACX_WAKE_UP_CONDITIONS
	Type:	Configuration
	Access:	Write Only
	Length:	2

******************************************************************************/
typedef enum
{
	WAKE_UP_EVENT_BEACON_BITMAP		= 0x01, /* Wake on every Beacon*/
	WAKE_UP_EVENT_DTIM_BITMAP		= 0x02,	/* Wake on every DTIM*/
	WAKE_UP_EVENT_N_DTIM_BITMAP		= 0x04, /* Wake on every Nth DTIM (Listen interval)*/
	WAKE_UP_EVENT_N_BEACONS_BITMAP	= 0x08, /* Wake on every Nth Beacon (Nx Beacon)*/
	WAKE_UP_EVENT_BITS_MASK			= 0x0F
} WakeUpEventBitMask_e;

typedef struct
{
    INFO_ELE_HDR
    uint8  wakeUpConditionBitmap;	/* The host can set one bit only. */
									/* WakeUpEventBitMask_e describes the Possible */
									/* Wakeup configuration bits*/

    uint8  listenInterval;			/* 0 for Beacon and Dtim, */
									/* xDtims (1-10) for Listen Interval and */
									/* xBeacons (1-255) for NxBeacon*/
    uint8  padding[2];              /* alignment to 32bits boundry   */
}WakeUpCondition_t;

/******************************************************************************

    Name:	ACX_MEM_CFG
	Type:	Configuration
	Access:	Write Only
	Length:	12

******************************************************************************/

/* Host Bus/Memory Mode - The following table specifies the possible host bus modes and */
/* memory organizations that the Wilink should use during operation. */
typedef enum{
    HOSTIF_PCI_MASTER_HOST_INDIRECT,
    HOSTIF_PCI_MASTER_HOST_DIRECT,
    HOSTIF_SLAVE,
    HOSTIF_PKT_RING,  
    HOSTIF_DONTCARE = 0xFF
} HostIFConfig_enum;

#ifdef HOST_COMPILE
typedef uint8 HostIFConfig_e;
#else
typedef HostIFConfig_enum HostIFConfig_e;
#endif

typedef struct
{
	INFO_ELE_HDR
    uint16  numStations;				/* specifies the number of STAs to be supported. */
										/* The FW uses this field to allocate memory */
										/* for STA context data such as security keys*/
    uint16  reserved1;
    uint8   rxMemblockNumber;			/* specifies the number of memory buffers that */
										/* is allocated to the Rx memory pool. The */
										/* actual number allocated may be less than*/
										/* this number if there are not enough memory */
										/* blocks left over for the Minimum Number of */
										/* Tx Blocks. Returns the actual number of RX */
										/* buffers allocated in the memory map*/
    uint8   Reserved2;
    uint8   numTxQueues;				/* specifies the number of descriptor queues */
										/* that are to be used for transmit operations. */
										/* Valid values are 1 to 16*/

    HostIFConfig_e hostifOptions;		/* specifies the memory configuration options */
										/* for the adaptor. The format of this field */
										/* is shown in HostIFConfig_enum.*/

    uint8   txMinimumMemblockNumber;	/* specifies the minimum number of blocks that */
										/* must be allocated to the TX pool. Follows */
										/* this limit even if the Number of Rx Memory */
										/* Blocks parameter is ignored.*/

    uint8   numSsidProfiles;			/* specifies the number of SSID profiles used */
										/* in the AP. Enables working with different */
										/* profiles for different stations.*/

    uint16  debugBufferSize;			/* This field specifies the number of words */
										/* allocated for each debug buffer if the */
										/* FW trace is enabled.*/
#ifndef HOST_COMPILE
    uint8   variableData;				/* contents vary starting here - should be */
    uint8   pedding[3];                                 /* aligned to 32 bits boundry*/
#endif
} ACXConfigMemory_t;

typedef struct
{
    uint8 numDescs;
    uint8  Reserved;
    uint8  Type;
    uint8  Priority;
    uint32 dmaAddress;
} ACXrxQueueConfig;

typedef struct
{
    uint8  numDescs;
    uint8  reserved1[2];
    uint8  attributes;   /* QPriority_e qPriority;*/
} ACXtxQueueConfig;

#define QUEUE_CONFIG_MAX_TX_QUEUES	5

typedef struct 
{
   ACXConfigMemory_t   memConfig;
   ACXrxQueueConfig        RxQueueConfig;
   ACXtxQueueConfig        TxQueueConfig[QUEUE_CONFIG_MAX_TX_QUEUES];
} ACXConfigMemoryStruct_t; 



/******************************************************************************

    Name:	ACX_SLOT
	Type:	Configuration
	Access:	Write Only
	Length:	8

******************************************************************************/

typedef enum
{
    SLOT_TIME_LONG = 0,		/* the WiLink uses long (20 us) slots*/
    SLOT_TIME_SHORT = 1,	/* the WiLink uses short (9 us) slots*/
    DEFAULT_SLOT_TIME = SLOT_TIME_SHORT,
    MAX_SLOT_TIMES = 0xFF
} SlotTime_enum;

#ifdef HOST_COMPILE
typedef uint8 SlotTime_e;
#else
typedef SlotTime_enum SlotTime_e;
#endif


typedef struct
{
    INFO_ELE_HDR
    uint8      woneIndex;	/* reserved*/

    SlotTime_e slotTime;	/* The slot size to be used. refer to SlotTime_enum.    */
    uint8      reserved[6];
} ACXSlot_t;

/******************************************************************************

    Name:	ACX_QUEUE_HEAD
	Type:	Configuration
	Access:	Read Only
	Length:	56
	Note:	for MASTER mode only!!!

******************************************************************************/

typedef struct
{
    uint32 addr;
    uint8  priority;
    uint8  padding[3];
} ACXoneQueueHead;

#define NUM_ACCESS_CATEGORIES_QUEUES	5		/* This takes into account the */
												/* broadcast AC queue*/
typedef struct
{
   INFO_ELE_HDR
   uint32  *txMemBlkQ;
   uint32  *rxMemBlkQ;
   ACXoneQueueHead rxQueueHead;
   ACXoneQueueHead txQueueHead[NUM_ACCESS_CATEGORIES_QUEUES];
} ACXQosQueueHead_t;


/******************************************************************************

    Name:	ACX_AC_CFG
	Type:	Configuration
	Access:	Write Only
	Length:	8

******************************************************************************/
typedef enum
{
	AC_BE = 0,			/* Best Effort/Legacy*/
	AC_BK = 1,			/* Background*/
	AC_VI = 2,			/* Video*/
	AC_VO = 3,			/* Voice*/
	AC_BCAST = 4,		/* Broadcast dummy access category*/
	NUM_ACCESS_CATEGORIES = 4
} AccessCategory_enum;

#define AC_FAST_TEMPLATE				4		/* Access Category for SG2.0 Fast CTS Tx */ 
#define AC_REQUEST						0xfe	/* Special access category type for */
												/* requests*/
#define AC_INVALID						0xff	/* Empty Access category in scheduler */
												/* Access Category DB*/

#define AC_ANY_TID						0xFF	/* Any TID/AC for the Split Scan */

/* following are defult values for the IE fields*/
#define CWMIN_BK  15
#define CWMIN_BE  15
#define CWMIN_VI  7
#define CWMIN_VO  3
#define CWMAX_BK  1023
#define CWMAX_BE  63
#define CWMAX_VI  15
#define CWMAX_VO  7
#define AIFS_PIFS 1 /* slot number setting to start transmission at PIFS interval */
#define AIFS_DIFS 2 /* slot number setting to start transmission at DIFS interval - */
					/* normal DCF access */
#define AIFSN_BK  7
#define AIFSN_BE  3
#define AIFSN_VI  AIFS_PIFS
#define AIFSN_VO  AIFS_PIFS
#define TXOP_BK   0
#define TXOP_BE   0
#define TXOP_VI   3008
#define TXOP_VO   1504
#define DEFAULT_AC_SHORT_RETRY_LIMIT 7
#define DEFAULT_AC_LONG_RETRY_LIMIT 4

/* rxTimeout values */
#define NO_RX_TIMEOUT 0

typedef struct 
{
	INFO_ELE_HDR
	uint8 	ac;			/* Access Category - The TX queue's access category */
						/* (refer to AccessCategory_enum)*/
    uint8   cwMin;		/* The contention window minimum size (in slots) for */
						/* the access class.*/
    uint16  cwMax;		/* The contention window maximum size (in slots) for */
						/* the access class.*/
	uint8   aifsn;		/* The AIF value (in slots) for the access class.*/
	uint8	reserved;
	uint16  txopLimit;	/* The TX Op Limit (in microseconds) for the access class.*/
} ACXAcCfg_t;


/******************************************************************************

    Name:	ACX_MEM_MAP
	Type:	Configuration
	Access:	Read Only
	Length:	72
	Note:	Except for the numTxMemBlks, numRxMemBlks fields, this is
			used in MASTER mode only!!!
	
******************************************************************************/
#define MEM_MAP_NUM_FIELDS	18

typedef struct
{
    INFO_ELE_HDR
    void *codeStart;				
    void *codeEnd;                  
    void *wepDefaultKeyStart;
    void *wepDefaultKeyEnd;         
    void *staTableStart;
    void *staTableEnd;              
    void *packetTemplateStart;
    void *packetTemplateEnd;        
    void *queueMemoryStart;
    void *queueMemoryEnd; 
    void *packetMemoryPoolStart;
    void *packetMemoryPoolEnd;
    void *debugBuffer1Start;
    void *debugBuffer1End;
    void *debugBuffer2Start;
    void *debugBuffer2End;
    uint32 numTxMemBlks;	/* Number of blocks that FW allocated for TX packets.*/
    uint32 numRxMemBlks;	/* Number of blocks that FW allocated for RX packets.	*/
} MemoryMap_t;


/******************************************************************************

    Name:	ACX_AID
	Type:	Configuration
	Access:	Write Only
	Length: 2
	
******************************************************************************/

typedef struct
{
    INFO_ELE_HDR
    uint16  Aid;	/* The Association ID to the WiLink. The WiLink uses this */
					/* field to determine when the STA's AID bit is set in a */
					/* received beacon and when a PS Poll frame should be */
					/* transmitted to the AP. The host configures this information */
					/* element after it has associated with an AP. This information */
					/* element does not need to be set in Ad Hoc mode.*/
    uint8  padding[2];  /* alignment to 32bits boundry   */
} ACXAid_t;

/******************************************************************************

    Name:	ACX_CFG
	Type:	Configuration
	Access:	Write Only
	Length: 25
	Note:   Not in use !!! 

******************************************************************************/

typedef struct ACXConfig_variableEntry_t
{
    uint8 dot11_ID;
    uint8 num_supported;
    uint8 dot11_Table;  /*cast and deref this as array of Sizeof-dot11_ID-Type.*/
    uint8 padding;              /* alignment to 32bits boundry   */
} ACXConfig_variableEntry_t;


typedef struct ACXConfigOptionsStruct_t
{
    INFO_ELE_HDR
    char   nvsVer[8];
    uint32 endMemLoc;
    uint16 VendorSpecificArea;
    uint8  dot11CCAModeSupported;
    uint8  dot11DiversitySupport;
    uint8  dot11ShortPreambleOptionImplemented;
    uint8  dot11PBCCOptionImplemented;
    uint8  dot11ChanneAgilityPresent;
    uint8  dot11PHYType;
    uint8  dot11TempType;
    uint8  numVarEntries;
    uint8  padding[2];  /* alignment to 32bits boundry   */

    ACXConfig_variableEntry_t vardata;
        /**/
        /*Begin variable portion of Config data...*/
        /**/
} ACXConfigOptionsStruct_t;

/******************************************************************************

    Name:	ACX_FW_REV
	Type:	Configuration
	Access:	Write Only
	Length:	24
	
******************************************************************************/

typedef struct
{
    INFO_ELE_HDR
	char FWVersion[20];		/* The WiLink firmware version, an ASCII string x.x.x.x.x */
							/* that uniquely identifies the current firmware. */
							/* The left most digit is incremented each time a */
							/* significant change is made to the firmware, such as */
							/* WLAN new project.*/
							/* The second and third digit is incremented when major enhancements*/
							/* are added or major fixes are made.*/
							/* The fourth digit is incremented for each SP release */
                            /* and it indicants the costumer private brench */
							/* The fifth digit is incremented for each build.*/
		
    uint32 HardWareVersion; /* This 4 byte field specifies the WiLink hardware version. */
							/* bits 0  - 15: Reserved.*/
							/* bits 16 - 23: Version ID - The WiLink version ID  */
							/*              (1 = first spin, 2 = second spin, and so on).*/
							/* bits 24 - 31: Chip ID - The WiLink chip ID. */
} ACXRevision_t;

/******************************************************************************

    Name:	ACX_FCS_ERROR_CNT
	Type:	Operation
	Access:	Read Only
	Length: 4
	Note:   OBSOLETE (replaced by ACX_ERROR_CNT) !!! 
	
******************************************************************************/

typedef struct
{
    INFO_ELE_HDR
    uint32 FCSErrorCount;
} ACXFCSErrorCount_t;

/******************************************************************************

    Name:	ACX_ERROR_CNT
	Type:	Operation
	Access:	Read Only
	Length: 12
	
******************************************************************************/
typedef struct
{
    INFO_ELE_HDR
    uint32 PLCPErrorCount;  /* The number of PLCP errors since the last time this */
	                        /* information element was interrogated. This field is */
	                        /* automatically cleared when it is interrogated.*/
	
    uint32 FCSErrorCount;   /* The number of FCS errors since the last time this */
	                        /* information element was interrogated. This field is */
	                        /* automatically cleared when it is interrogated.*/
	
    uint32 validFrameCount; /* The number of MPDUÂ’s without PLCP header errors received*/
                            /* since the last time this information element was interrogated. */
                            /* This field is automatically cleared when it is interrogated.*/

    uint32 seqNumMissCount; /* the number of missed sequence numbers in the squentially */
                            /* values of frames seq numbers */

} ACXErrorCounters_t;

/******************************************************************************

    Name:	ACX_MEDIUM_USAGE
	Type:	Configuration
	Access:	Read Only
	Length: 8

******************************************************************************/

typedef struct
{
    INFO_ELE_HDR
    uint32 mediumUsage; /* report to the host the value of medium usage registers*/
    uint32 period;		/* report to the host the value of medium period registers*/
} ACXMediumUsage_t;

/******************************************************************************

    Name:	ACX_RX_CFG
	Type:	Filtering Configuration
	Access:	Write Only
	Length: 8
	
******************************************************************************/
/*
 * Rx configuration (filter) information element
 * ---------------------------------------------
 */
/*
	RX ConfigOptions Table
	Bit		Definition
	===		==========
	31:14	Reserved
	13		Copy RX Status - when set, write three receive status words to top of 
			rx'd MPDU.
			When clear, do not write three status words (added rev 1.5)
	12		Reserved
	11		RX Complete upon FCS error - when set, give rx complete interrupt for 
			FCS errors, after the rx filtering, e.g. unicast frames not to us with 
			FCS error will not generate an interrupt
	10		SSID Filter Enable - When set, the WiLink discards all beacon, 
			probe request, and probe response frames with an SSID that does not 
			match the SSID specified by the host in the START/JOIN command. 
			When clear, the WiLink receives frames with any SSID.
	9		Broadcast Filter Enable - When set, the WiLink discards all broadcast 
			frames. When clear, the WiLink receives all received broadcast frames.
	8:6		Reserved
	5		BSSID Filter Enable - When set, the WiLink discards any frames with a 
			BSSID that does not match the BSSID specified by the host. 
			When clear, the WiLink receives frames from any BSSID.
	4		MAC Addr Filter - When set, the WiLink discards any frames with a 
			destination address that does not match the MAC address of the adaptor. 
			When clear, the WiLink receives frames destined to any MAC address.
	3		Promiscuous - When set, the WiLink receives all valid frames 
			(i.e., all frames that pass the FCS check). 
			When clear, only frames that pass the other filters specified are received.
	2		FCS - When set, the WiLink includes the FCS with the received frame. 
			When clear, the FCS is discarded.
	1		PLCP header - When set, write all data from baseband to frame buffer 
			including PHY header.
	0		Reserved - Always equal to 0.

	RX FilterOptions Table
	Bit		Definition
	===		==========
	31:12	Reserved - Always equal to 0.
	11		Association - When set, the WiLink receives all association related frames 
			(association request/response, reassocation request/response, and 
			disassociation). When clear, these frames are discarded.
	10		Auth/De auth - When set, the WiLink receives all authentication and 
			de-authentication frames. When clear, these frames are discarded.
	9		Beacon - When set, the WiLink receives all beacon frames. When clear, 
			these frames are discarded.
	8		Contention Free - When set, the WiLink receives all contention free frames. 
			When clear, these frames are discarded.
	7		Control - When set, the WiLink receives all control frames. 
			When clear, these frames are discarded.
	6		Data - When set, the WiLink receives all data frames.	
			When clear, these frames are discarded.
	5		FCS Error - When set, the WiLink receives frames that have FCS errors. 
			When clear, these frames are discarded.
	4		Management - When set, the WiLink receives all management frames. 
			When clear, these frames are discarded.
	3		Probe Request - When set, the WiLink receives all probe request frames. 
			When clear, these frames are discarded.
	2		Probe Response - When set, the WiLink receives all probe response frames. 
			When clear, these frames are discarded.
	1		RTS/CTS/ACK - When set, the WiLink receives all RTS, CTS and ACK frames. 
			When clear, these frames are discarded.
	0		Rsvd Type/Sub Type - When set, the WiLink receives all frames that 
			have reserved frame types and sub types as defined by the 802.11 
			specification. 
			When clear, these frames are discarded.
*/
typedef struct
{
    INFO_ELE_HDR
    uint32          ConfigOptions;	/* The configuration of the receiver in the WiLink. */
									/* "RX ConfigOptions Table" describes the format of */
									/* this field.*/
    uint32          FilterOptions;	/* The types of frames that the WiLink can receive. */
									/* "RX FilterOptions Table" describes the format of */
									/* this field.*/
} ACXRxConfig_t;

/******************************************************************************

    Name:	ACX_BEACON_FILTER_OPT
	Desc:   This information element enables the host to activate beacon filtering. 
            The filter can only be activated when the STA is in PS mode. 
            When activated, either the host is not notified about beacons whose 
            unicast TIM bit is not set, or these beacons are buffered first and 
            the host is notified only after the buffer reaches a predetermined size.
            The host should not activate the filter if it configures the firmware 
            to listen to broadcasts (see the VBM Options field in the 
            ACXPowerMgmtOptions information element). The filter only affects beacons, 
            and not other MSDUs - the firmware notifies the host immediately about 
            their arrival.
	Type:	Filtering Configuration
	Access:	Write Only
	Length: 2
 
******************************************************************************/
typedef struct  
{
    INFO_ELE_HDR
    uint8   enable;                /* Indicates whether the filter is enabled. */
                                   /* 1 - enabled, 0 - disabled. */
    uint8   maxNumOfBeaconsStored; /* The number of beacons without the unicast TIM */
                                   /* bit set that the firmware buffers before */
                                   /* signaling the host about ready frames. */
	                               /* When set to 0 and the filter is enabled, beacons */
	                               /* without the unicast TIM bit set are dropped.*/
    uint8  padding[2];             /* alignment to 32bits boundry   */
} ACXBeaconFilterOptions_t;


/******************************************************************************

    Name:	ACX_BEACON_FILTER_TABLE
	Desc:   This information element configures beacon filtering handling for the
	        set of information elements. An information element in a beacon can be 
			set to be: ignored (never compared, and changes will not cause beacon 
			transfer), checked (compared, and transferred in case of a change), or 
			transferred (transferred to the host for each appearance or disappearance).
	        The table contains all information elements that are subject to monitoring 
			for host transfer. 
			All information elements that are not in the table should be ignored for 
			monitoring.
	        This functionality is only enabled when beacon filtering is enabled by 
			ACX_BEACON_FILTER_OPT.
	Type:	Filtering Configuration
	Access:	Write Only
	Length: 101
	Notes:  the field measuring the value of received beacons for which the device 
	        wakes up the host in ACX_BEACON_FILTER_OPT does not affect 
			this information element.
	
******************************************************************************/

/*
    ACXBeaconFilterEntry (not 221)
    Byte Offset     Size (Bytes)    Definition 
	===========     ============    ==========
	0				1               IE identifier
    1               1               Treatment bit mask

    ACXBeaconFilterEntry (221)
    Byte Offset     Size (Bytes)    Definition 
    ===========     ============    ==========
    0               1               IE identifier
    1               1               Treatment bit mask
    2               3               OUI
    5               1               Type
    6               2               Version


    Treatment bit mask - The information element handling:
                         bit 0 - The information element is compared and transferred
						         in case of change.
                         bit 1 - The information element is transferred to the host 
						         with each appearance or disappearance.
                         Note that both bits can be set at the same time.
*/
#define	BEACON_FILTER_TABLE_MAX_IE_NUM						(32)
#define BEACON_FILTER_TABLE_MAX_VENDOR_SPECIFIC_IE_NUM		(6)
#define BEACON_FILTER_TABLE_IE_ENTRY_SIZE					(2)
#define BEACON_FILTER_TABLE_EXTRA_VENDOR_SPECIFIC_IE_SIZE	(6)
#define BEACON_FILTER_TABLE_MAX_SIZE	((BEACON_FILTER_TABLE_MAX_IE_NUM * BEACON_FILTER_TABLE_IE_ENTRY_SIZE) + \
                                         (BEACON_FILTER_TABLE_MAX_VENDOR_SPECIFIC_IE_NUM * BEACON_FILTER_TABLE_EXTRA_VENDOR_SPECIFIC_IE_SIZE))

typedef struct ACXBeaconFilterIETableStruct {
	INFO_ELE_HDR
	uint8 NumberOfIEs;							/* The number of IE's in the table*/
                                                /* 0 - clears the table.*/

    uint8 padding[3];  /* alignment to 32bits boundry   */
	uint8 IETable[BEACON_FILTER_TABLE_MAX_SIZE];
} ACXBeaconFilterIETable_t;

/******************************************************************************

    Name:	ACX_ARP_IP_FILTER 
	Type:	Filtering Configuration
	Access:	Write Only
	Length: 20

******************************************************************************/

typedef struct  
{    
    INFO_ELE_HDR
	uint8     ipVersion;       /* The IP version of the IP address: 4 - IPv4, 6 - IPv6.*/
    uint8     arpFilterEnable; /* 1 - ARP filtering is enabled. */
	                           /* 0 - ARP filtering is disabled.*/
    uint8     padding[2];      /* alignment to 32bits boundry   */
    uint8     address[16];     /* The IP address used to filter ARP packets. ARP packets */
	                           /* that do not match this address are dropped. */
	                           /* When the IP Version is 4, the last 12 bytes of */
	                           /* the address are ignored.*/
	
} ACXConfigureIP_t;


/******************************************************************************

  Name:	    ACX_IBSS_FILTER
  Type:	    Filtering Configuration
  Access:	Write Only
  Length:   1
  
******************************************************************************/
typedef struct  
{
    INFO_ELE_HDR
    uint8   enable; /* if set (i.e. IBSS mode), forward beacons from the same SSID*/
	                /* (also from different BSSID), with bigger TSF then the this of */
	                /* the current BSS.*/
    uint8   padding[3]; /* alignment to 32bits boundry   */
} ACXIBSSFilterOptions_t;


/******************************************************************************

  Name:	    ACX_SERVICE_PERIOD_TIMEOUT
  Type:	    Configuration
  Access:	Write Only
  Length:   1
  
******************************************************************************/
typedef struct 
{    
	INFO_ELE_HDR
	uint16 PsPollTimeout; /* the maximum time that the device will wait to receive */
	                      /* traffic from the AP after transmission of PS-poll.*/
	
    uint16 UpsdTimeout;	  /* the maximum time that the device will wait to receive */
	                      /* traffic from the AP after transmission from UPSD enabled*/
	                      /* queue.*/
} ACXRxTimeout_t;

/******************************************************************************

    Name:	ACX_TX_QUEUE_CFG
	Type:	Configuration
	Access:	Write Only
	Length: 8
	
******************************************************************************/
typedef struct
{
    INFO_ELE_HDR
	uint8	qID;						/* The TX queue ID number.*/
    uint8   padding[3];                 /* alignment to 32bits boundry   */
    uint16	numberOfBlockHighThreshold; /* The maximum memory blocks allowed in the */
										/* queue.*/
    uint16	numberOfBlockLowThreshold;	/* The minimum memory blocks that are */
										/* guaranteed for this queue.*/
} ACXTxQueueCfg_t;


/******************************************************************************

    Name:	ACX_BSS_IN_PS
	Type:	Configuration
	Access:	Write Only
	Length:	1
	Note:	for AP only (OBSOLETE???)
	
******************************************************************************/

typedef enum
{
    AP_POWER_ACTIVE_MODE = FALSE,
    AP_POWER_SAVE_MODE = TRUE
} APPowerMgmtMode_enum;

#ifdef HOST_COMPILE
typedef uint8 APPowerMgmtMode_e;
#else
typedef APPowerMgmtMode_enum APPowerMgmtMode_e;
#endif


typedef struct
{
    INFO_ELE_HDR
    APPowerMgmtMode_e BSSinPowerSave;
    uint8             padding[3];  /* alignment to 32bits boundry   */
} ACXBSSPowerSave_t;



/******************************************************************************

    Name:	ACX_STATISTICS
	Type:	Statistics
	Access:	Write Only
	Length: 
	Note:	Debug API

******************************************************************************/

typedef struct
{
    uint32 TxInternalDescOverflow;
} TxStatistics_t;


typedef struct
{
    uint32 RxOutOfMem;
    uint32 RxHdrOverflow;
    uint32 RxHWStuck;
    uint32 RxDroppedFrame;
    uint32 RxFcsErr;
    uint32 RxXfrHintTrig;
    uint32 RxPathReset;
    uint32 RxResetCounter;
} RxStatistics_t;


typedef struct
{
    uint32 RxDMARequested;
    uint32 RxDMAErrors;
    uint32 TxDMARequested;
    uint32 TxDMAErrors;
} DMAStatistics_t;


typedef struct
{
    uint32 Cmd_Cmplt;         /* Host command complete */
    uint32 FIQs;              /* fiqisr() */
    uint32 RxHeaders;         /* (INT_STS_ND & INT_TRIG_RX_HEADER) */
    uint32 RxCompletes;       /* (INT_STS_ND & INT_TRIG_RX_CMPLT) */
    uint32 RxMemOverflow;     /* (INT_STS_ND & INT_TRIG_NO_RX_BUF) */
    uint32 RxRdys;            /* (INT_STS_ND & INT_TRIG_S_RX_RDY) */
    uint32 IRQs;              /* irqisr() */
    uint32 ACXTxProcs;        /* (INT_STS_ND & INT_TRIG_TX_PROC) */
    uint32 DecryptDone;       /* (INT_STS_ND & INT_TRIG_DECRYPT_DONE) */
    uint32 DMA0Done;          /* (INT_STS_ND & INT_TRIG_DMA0) */
    uint32 DMA1Done;          /* (INT_STS_ND & INT_TRIG_DMA1) */
    uint32 ACXTxExchComplete; /* (INT_STS_ND & INT_TRIG_TX_EXC_CMPLT) */
    uint32 ACXCommands;       /* (INT_STS_ND & INT_TRIG_COMMAND) */
    uint32 ACXRxProcs;        /* (INT_STS_ND & INT_TRIG_RX_PROC) */
    uint32 HwPMModeChanges;   /* (INT_STS_ND & INT_TRIG_PM_802) */
    uint32 HostAcknowledges;  /* (INT_STS_ND & INT_TRIG_ACKNOWLEDGE) */
    uint32 PCI_PM;            /* (INT_STS_ND & INT_TRIG_PM_PCI) */
    uint32 ACMWakeups;        /* (INT_STS_ND & INT_TRIG_ACM_WAKEUP) */
    uint32 LowRssi;           /* (INT_STS_ND & INT_TRIG_LOW_RSSI) */
} IsrStatistics_t;


typedef struct WepStatistics_t
{
    uint32 WepAddrKeyCount;      /* Count of WEP address keys configured*/
    uint32 WepDefaultKeyCount;   /* Count of default keys configured*/
    uint32 reserved;
    uint32 WepKeyNotFound;       /* count of number of times that WEP key not found on lookup*/
    uint32 WepDecryptFail;       /* count of number of times that WEP key decryption failed*/
    uint32 WepPackets;           /* WEP Packets Decrypted*/
    uint32 WepInterrupt;         /* WEP Decrypt Interrupts*/
} WepStatistics_t;


#define PWR_STAT_MAX_CONT_MISSED_BCNS_SPREAD 10
typedef struct PwrStatistics_t
{
    uint32 PSEnterCnt;          /* Count the amount of enters into power save mode (both PD & ELP)*/
    uint32 ELPEnterCnt;         /* Count the amount of enters into ELP mode.*/
    uint32 MissingBcnsCnt;      /* Count the amount of missing beacon interrupts to the host.*/
    uint32 WakeOnHostCnt;       /* Count the amount of wake on host-access times.*/
    uint32 WakeOnTimerExpCnt;   /* Count the amount of wake on timer-expire.*/
    uint32 TxWithPSCnt;         /* Count the number of packets that were transmitted with PS bit set*/
    uint32 TxWithoutPSCnt;      /* Count the number of packets that were transmitted with PS bit clear*/
    uint32 RcvdBeaconsCnt;      /* Count the number of received beacons.*/
    uint32 PowerSaveOffCnt;     /* Count the number of entering into PowerOn (power save off).*/
    uint16 EnablePSCnt;         /* Count the number of entries into power save mode.*/
    uint16 DisablePSCnt;        /* Count the number of exits from power save (not including the PS_FAIL route.*/
    uint32 FixTsfPSCnt;         /* Count the number of times the TSF counter was adjusted because of drift.*/
    uint32 ContMissBcnsSpread[PWR_STAT_MAX_CONT_MISSED_BCNS_SPREAD];  /* Gives statistics about the spread continuous missed beacons.*/
                                    /* The 16 LSB are dedicated for the PS mode.*/
                                    /* The 16 MSB are dedicated for the PS mode.*/
                                    /* ContMissBcnsSpread[0] - single missed beacon.*/
                                    /* ContMissBcnsSpread[1] - two continuous missed beacons.*/
                                    /* ContMissBcnsSpread[2] - three continuous missed beacons.*/
                                    /* ...*/
                                    /* ContMissBcnsSpread[9] - ten and more continuous missed beacons.*/
    uint32 RcvdAwakeBeaconsCnt; /* Count the number of beacons in awake mode.*/
} PwrStatistics_t;


typedef struct MicStatistics_t
{
    uint32 MicRxPkts;
    uint32 MicCalcFailure;
} MicStatistics_t;


typedef struct AesStatisticsStruct
{
    uint32 AesEncryptFail;
    uint32 AesDecryptFail;
    uint32 AesEncryptPackets;
    uint32 AesDecryptPackets;
    uint32 AesEncryptInterrupt;
    uint32 AesDecryptInterrupt;
} AesStatistics_t;


typedef struct EventStatistics_t
{
    uint32 heartbeat;
    uint32 calibration;
    uint32 rxMismatch;
    uint32 rxMemEmpty;
    uint32 rxPool;
    uint32 oomLate;
    uint32 phyTransmitError;
    uint32 txStuck;
} EventStatistics_t;


typedef struct PsPollUpsdStatistics_t
{
    uint32 psPollTimeOuts;
    uint32 upsdTimeOuts;
    uint32 upsdMaxSPTime;
    uint32 upsdMaxAPturn;
    uint32 psPollMaxAPturn;
    uint32 psPollUtilization;
    uint32 upsdUtilization;
} PsPollUpsdStatistics_t;


typedef struct
{
    uint32 RxPrepBeaconDrop;
    uint32 DescrHostIntTrigRxData;
    uint32 BeaconBufferThresHostIntTrigRxData;
    uint32 MissedBeaconHostIntTrigRxData;
    uint32 TxXfrHostIntTrigRxData;
} RxPipeStatistics_t;


typedef struct ACXStatisticsStruct
{
    INFO_ELE_HDR
    TxStatistics_t   tx;
    RxStatistics_t   rx;
    DMAStatistics_t  dma;
    IsrStatistics_t  isr;
    WepStatistics_t  wep;
    PwrStatistics_t  pwr;
    AesStatistics_t  aes;
    MicStatistics_t  mic;
    EventStatistics_t event;
#ifdef FW_RUNNING_AS_STA
    PsPollUpsdStatistics_t ps;
    RxPipeStatistics_t rxp;
#endif
} ACXStatistics_t;

/******************************************************************************

    Name:	ACX_ROAMING_STATISTICS_TBL
	Desc:   This information element reads the current roaming triggers 
	        counters/metrics. 
	Type:	Statistics
	Access:	Read Only
	Length: 6

******************************************************************************/
typedef struct 
{
	INFO_ELE_HDR
	uint32 MissedBeacons; /* The current number of consecutive lost beacons*/
	uint8  snr;           /* The current average SNR in db*/
    int8   rssi;          /* The current average RSSI*/
    uint8  padding[2];    /* alignment to 32bits boundry   */
}ACXRoamingStatisticsTable_t;


/******************************************************************************

    Name:	ACX_FEATURE_CFG
	Desc:   Provides expandability for future features
	Type:	Configuration
	Access:	Write Only
	Length: 8
	
******************************************************************************/

/* bit defines for Option: */
#define FEAT_PCI_CLK_RUN_ENABLE     0x00000002	/* Enable CLK_RUN on PCI bus */

/* bit defines for dataflowOptions: */
#define DF_ENCRYPTION_DISABLE       0x00000001	/* When set, enable encription in FW.*/
												/* when clear, disable encription. */
#define DF_SNIFF_MODE_ENABLE        0x00000080	/* When set, enable decryption in FW.*/
												/* when clear, disable decription. */
typedef struct
{
    INFO_ELE_HDR
    uint32 Options;         /* Data flow options - refer to above definitions*/
    uint32 dataflowOptions; /* Data flow options - refer to above definitions*/
} ACXFeatureConfig_t;



/******************************************************************************

    Name:	ACX_TID_CFG
	Type:	Configuration
	Access:	Write Only
	Length: 16
	
******************************************************************************/
typedef enum
{
	CHANNEL_TYPE_DCF = 0,   /* DC/LEGACY*/
	CHANNEL_TYPE_EDCF = 1,  /* EDCA*/
	CHANNEL_TYPE_HCCA = 2,  /* HCCA*/
	MAX_CHANNEL_TYPE = CHANNEL_TYPE_HCCA
} ChannelType_enum;

typedef enum
{
    PS_SCHEME_LEGACY         = 0, /* Regular PS: simple sending of packets*/
    PS_SCHEME_UPSD_TRIGGER   = 1, /* UPSD: sending a packet triggers a UPSD downstream*/
    PS_SCHEME_LEGACY_PSPOLL  = 2, /* Legacy PSPOLL: a PSPOLL packet will be sent before */
								  /* every data packet transmission in this queue.*/
    PS_SCHEME_SAPSD          = 3, /* Scheduled APSD mode.*/
    MAX_PS_SCHEME = PS_SCHEME_SAPSD
} PSScheme_enum;

typedef enum
{
	ACK_POLICY_LEGACY = 0,   /* ACK immediate policy*/
	ACK_POLICY_NO_ACK = 1,   /* no ACK policy*/
	ACK_POLICY_BLOCK  = 2,   /* block ack policy*/
	MAX_ACK_POLICY = ACK_POLICY_BLOCK
} AckPolicy_enum;


#ifdef HOST_COMPILE
typedef uint8 ChannelType_e;
typedef uint8 PSScheme_e;
typedef uint8 AckPolicy_e;
#else
typedef ChannelType_enum ChannelType_e;
typedef PSScheme_enum PSScheme_e;
typedef AckPolicy_enum AckPolicy_e;
#endif


/* rxTimeout values */
#define NO_RX_TIMEOUT 0


typedef struct
{
    INFO_ELE_HDR
    uint8 	queueID;        /* The TX queue ID number (0-7).*/
    uint8 	channelType;    /* Channel access type for the queue.*/
						    /* Refer to ChannelType_enum.*/
    uint8 	tsid;           /* for EDCA - the AC Index (0-3, refer to*/
	                        /* AccessCategory_enum).*/
                            /* For HCCA - HCCA Traffic Stream ID (TSID) of */
	                        /* the queue (8-15).*/
    PSScheme_e  psScheme;   /* The power save scheme of the specified queue.*/
	                        /* Refer to PSScheme_enum.*/
	AckPolicy_e ackPolicy;  /* The TX queue ACK policy. */
    uint8  padding[3];      /* alignment to 32bits boundry   */
    uint32 APSDConf[2];     /* Not supported in this version !!!*/
}ACXTIDConfig_t;



/******************************************************************************

    Name:	ACX_LOW_RSSI
	Desc:   This information element configures the Low and Regained RSSI interrupt 
            indicators. Low RSSI calculates the average RSSI by giving higher weight 
            to the old samples than to the current sample.
            The triggering of the Regained RSSI Event cannot be configured and 
            is always an "Edge" indication.
            The RSSI is reset on JoinBSS - the filter history CurrentRSSI is reset.
            The average RSSI is calculated as follows: 
            averageRssi = ((int32)((100-roamingTriggerParameters.lowRssi.rssiFilterWeight) * averageRssi) + 
                          (int32)(roamingTriggerParameters.lowRssi.rssiFilterWeight * (int8)rxInfo->rxLevel)) / 100;
	Type:	Configuration
	Access:	Write Only
	Length: 4
	
******************************************************************************/

typedef enum
{
    LOW_RSSI_EVENT_LEVEL = 0,  /* The event is a "Level" indication which keeps */
		                       /* triggering as long as the average RSSI is below*/
							   /* the threshold.*/

	LOW_RSSI_EVENT_EDGE = 1    /* The event is an "Edge" indication which triggers*/
	                           /* only when the RSSI threshold is crossed from above.*/
} LowRSSIEventType_enum;

#ifdef HOST_COMPILE
typedef uint8 LowRSSIEventType_e;
#else
typedef LowRSSIEventType_enum LowRSSIEventType_e;
#endif
 

typedef struct
{
    INFO_ELE_HDR
	int8  rssiThreshold;                 /* The threshold (in dBm) below (or above */
	                                     /* after low rssi indication) which the */
	                                     /* firmware generates an interrupt to the host. */
	                                     /* This parameter is signed.*/

    uint8 rssiFilterWeight;              /* The weight of the current RSSI sample, */
	                                     /* before adding the new sample, that is used */
	                                     /* to calculate the average RSSI.*/

	uint8 rssiFilterDepth;               /* The number of Beacons/Probe response frames */
                                         /* that will be received before issuing the */
	                                     /* Low or Regained RSSI event*/
	
	LowRSSIEventType_e LowRSSIEventType; /* This parameter configures how the Low RSSI */
	                                     /* Event is triggered. */
	                                     /* Refer to LowRSSIEventType_enum.*/
} ACXLowRSSITriggerParameters_t;

/******************************************************************************

    Name:	ACX_AVERAGE_RSSI
	Type:	Configuration
	Access:	Read Only
	Length: 8
	Note:   Not in use !!!
	
******************************************************************************/

typedef struct
{
    INFO_ELE_HDR
    int8 avaregeRSSI; /* in DBM*/
    uint8 padding[3];  /* alignment to 32bits boundry   */
} ACXAvaregeRSSI_t;

/******************************************************************************

    Name:	ACX_NOISE_HIST
	Desc:   Noise Histogram activation is done by special command from host which
            is responsible to read the results using this IE.
	Type:	Configuration
	Access:	Read Only
	Length: 48 (NOISE_HIST_LEN=8)
	
******************************************************************************/

typedef struct
{
    INFO_ELE_HDR
    uint32 counters[NOISE_HIST_LEN]; /* This array of eight 32 bit counters describes */
	                                 /* the histogram created by the FW noise */
	                                 /* histogram engine.*/

    uint32 numOfLostCycles;          /* This field indicates the number of measurement */
	                                 /* cycles with failure because Tx was active.*/

    uint32 numOfTxHwGenLostCycles;   /* This field indicates the number of measurement */
	                                 /* cycles with failure because Tx (FW Generated)*/
	                                 /* was active.*/

    uint32 numOfRxLostCycles;        /* This field indicates the number of measurement */
	                                 /* cycles because the Rx CCA was active. */
} NoiseHistResult_t;

/******************************************************************************

    Name:	ACX_PD_THRESHOLD
	Type:	Configuration
	Access:	Write Only
	Length: 4

******************************************************************************/

typedef struct
{
    INFO_ELE_HDR
    uint32 pdThreshold; /* The packet detection threshold in the PHY.*/
} ACXPacketDetection_t;


/******************************************************************************

    Name:	ACX_DATA_PATH_PARAMS
	Desc:   configure Data path and Tx complete parameters
	Type:	Configuration
	Access:	Write Only
	Length: 12

******************************************************************************/
typedef struct 
{
    INFO_ELE_HDR
    uint16     	rxPacketRingChunkSize;	/* size in bytes of each buffer in a */
	                                    /* multi-buffers transfer mechanism (size of */
	                                    /* largest possible packet).*/

    uint16     	txPacketRingChunkSize; 	/* size in bytes of each buffer in a */
	                                    /* multi-buffers transfer mechanism (size of */
	                                    /* largest possible packet).*/

    uint8     	rxPacketRingChunkNum;	/* number of buffers in a multi-buffers */
	                                    /* transfer mechanism (2 for double buffer).*/

    uint8     	txPacketRingChunkNum; 	/* number of buffers in a multi-buffers */
	                                    /* transfer mechanism (2 for double buffer).*/

    uint8       txCompleteThreshold;	/* maximum number of packets that can be */
	                                    /* gathered in the TX complete ring before */
	                                    /* interrupt is generated.*/

    uint8       txCompleteRingDepth;	/* number of pending (waiting to be read by*/
	                                    /* the Host) TX complete entries in cyclic ring.*/

	uint32		txCompleteTimeOut;		/* the maximum time in micro-seconds since a */
	                                    /* packet enters the TX complete ring until */
	                                    /* interrupt is generated.*/
} ACXDataPathParams_t;

/******************************************************************************

    Name:	ACX_DATA_PATH_RESP_PARAMS
	Desc:   Get data path specific parameters.
	Type:	Configuration
	Access:	Read Only
	Length: 28
    Note:   The interrogation of this IE can be done only after the configuration
			of the corresponding Data Path parameters IE.

******************************************************************************/
typedef struct 
{
    INFO_ELE_HDR
    uint16     	rxPacketRingChunkSize; /* actual size in bytes of each buffer */
	                                   /* in a multi-buffers transfer mechanism.*/

    uint16     	txPacketRingChunkSize; /* actual size in bytes of each buffer */
	                                   /* in a multi-buffers transfer mechanism.*/

    uint8     	rxPacketRingChunkNum;  /* actual number of buffers in a */
	                                   /* multi-buffers transfer mechanism.	*/

    uint8     	txPacketRingChunkNum;  /* actual number of buffers in a */
	                                   /* multi-buffers transfer mechanism.*/

    uint8       padding[2];            /* alignment to 32bits boundry   */
    uint32		rxPacketRingAddr;      /* base address of the the multi-buffer space.*/

    uint32		txPacketRingAddr;      /* base address of the the multi-buffer space.*/

    uint32		rxControlAddr;         /* address of Rx Control register.*/

    uint32		txControlAddr;         /* address of Rx Control register.*/

    uint32		txCompleteAddr;	       /* base address of Tx Complete ring.*/
} ACXDataPathParamsResp_t;

/******************************************************************************

    Name:	ACX_RATE_POLICY
	Type:	Configuration
	Access:	Write Only
	Length: 132

******************************************************************************/
typedef enum
{
	RATE_CLASS_54M,
	RATE_CLASS_48M,
	RATE_CLASS_36M,
	RATE_CLASS_24M,
	RATE_CLASS_22M,
	RATE_CLASS_18M,
	RATE_CLASS_12M,
	RATE_CLASS_11M,
	RATE_CLASS_9M,
	RATE_CLASS_6M,
	RATE_CLASS_5_5M,
	RATE_CLASS_2M,
	RATE_CLASS_1M,
	RATE_CLASSES_SIZE 
} RatePolicy_enum;
#define MAX_RATE_POLICIES       (8)

/* 
aflag definition
bit field		description
=========		===========
7:3				Reserved
2				Preamble Type - The type of the preamble to be used by the policy. 
                0 - long preamble, 
				1 - short preamble. 
1				Preamble Override - Indicates if the preamble type should be used in TX. 
0				Truncate - If set, then attempts to send a frame stop when the total 
                valid per-rate attempts have been exhausted; 
				otherwise transmissions will continue at the lowest available rate 
				until the appropriate one of the Short Retry Limit, Long Retry Limit, 
				dot11MaxTransmitMsduLifetime, or MAX TX Life Time (in ACXTIDConfig), 
				if supported and supplied, is exhausted.
*/


/* definition of single rate policy*/
typedef struct 
{
    uint8               rateClass[RATE_CLASSES_SIZE]; /* The number of attempts for TX */
	                                                  /* for each rate class.  */
	                                                  /* RatePolicy_enum describes the */
													  /* indices for the table.*/

    uint8               shortRetryLimit;              /* The dot11ShortRetryLimit used */
                                                      /* for Tx retries.*/

    uint8               longRetryLimit;               /* The dot11LongRetryLimit used */
	                                                  /* for Tx retries.  */
	
    uint8               aflags;                       /* Flags controlling attributes */
	                                                  /* of the transmission. */
	                                                  /* see above description for the */
	                                                  /* structure of this field.*/
}txAttrClass_t;
     

typedef struct 
{
    INFO_ELE_HDR
    uint32        numOfClasses;                    /* The number of transmission rate */
	                                               /* fallback policy classes.*/

    txAttrClass_t rateClasses[MAX_RATE_POLICIES];  /* Rate Policies table*/
}ACXTxAttrClasses_t;


/******************************************************************************

    Name:	ACX_FW_GEN_FRAME_RATES
	Desc:   FW generated (template) frames rates and modulation. 
	Type:	Configuration
	Access:	Write Only
	Length: 4
	
******************************************************************************/

/*
Values for the Frame rate configuration
Value	Transmit Rate
=====   =============
0x0A	1 Mbps
0x14	2 Mbps
0x37	5.5 Mbps
0x6E	11 Mbps (CCK)
0xDC	22 Mbps
0x0B	6 Mbps
0x0F	9 Mbps
0x0A	12 Mbps
0x0E	18 Mbps
0x09	24 Mbps
0x0D	36 Mbps
0x08	48 Mbps
0x0C	54 Mbps
*/

/*
Values for the modulation configuration
Value	Modulation
=====   ==========
0	    CCK_LONG
1       CCK_SHORT
128	    PBCC_LONG
129	    PBCC_SHORT
64	    OFDM
*/

typedef struct
{
	INFO_ELE_HDR
	uint8 	txCtrlFrmRate; /* This field indicates the rate at which the WiLink */
	                       /* transmits RTS, CTS, PS Poll and QoS-Null frames. */
	                       /* Valid values are listed in the above table.*/

    uint8  	txCtrlFrmMod;  /* Modultion type for the above template frame. */
                           /* Valid values are listed in the above table.*/

    uint8  	txMgmtFrmRate; /* This field indicates the rate at which the WiLink */
	                       /* transmits beacon, probe response  frames. */
	                       /* If the host is configuring the WiLink as an AP or Ad Hoc */
	                       /* STA, you must configure the beacon template before the */
	                       /* START/JOIN command is issued. */
	                       /* Valid values are listed in the above table.*/

    uint8  	txMgmtFrmMod;  /* Modultion type for the above template frame.*/
                           /* Valid values are listed in the above table.*/
}ACXFwGeneratedFrameRates_t;

/******************************************************************************

    Name:	ACX_CTS_PROTECTION
	Type:	Configuration
	Access:	Write Only
	Length: 1
	
******************************************************************************/

typedef struct 
{
	INFO_ELE_HDR
	uint8   ctsProtectMode; /* This field is a flag enabling or disabling the*/
	                            /* CTS-to-self protection mechanism:*/
	                            /* 0 - disable, 1 - enable*/
    uint8  padding[3];          /* alignment to 32bits boundry   */
}ACXCtsProtection_t;

/******************************************************************************

    Name:	ACX_SLEEP_AUTH
	Desc:   configuration of sleep authorization level
	Type:	System Configuration
	Access:	Write Only
	Length: 1

******************************************************************************/

typedef struct 
{
    INFO_ELE_HDR
    uint8   sleepAuth; /* The sleep level authorization of the device. */
	                   /* 0 - Always active*/
	                   /* 1 - Power down mode: light / fast sleep*/
                       /* 2 - ELP mode: Deep / Max sleep*/
		
    uint8  padding[3]; /* alignment to 32bits boundry   */
}ACXSleepAuth_t;

/******************************************************************************

    Name:	ACX_PREAMBLE_TYPE
	Type:	Configuration
	Access:	Write Only
	Length: 1
	
******************************************************************************/

typedef enum
{
    ACX_PREAMBLE_LONG = 0,
    ACX_PREAMBLE_SHORT = 1,
    ACX_DEFAULT_PREAMBLE = ACX_PREAMBLE_LONG
} Preamble_enum;

#ifdef HOST_COMPILE
typedef uint8 Preamble_e;
#else
typedef Preamble_enum Preamble_e;
#endif


typedef struct
{
    INFO_ELE_HDR
    Preamble_e preamble; /* When set, the WiLink transmits beacon, probe response, */
	                     /* RTS and PS Poll frames with a short preamble. */
	                     /* When clear, the WiLink transmits the frame with a long */
	                     /* preamble.*/
    uint8  padding[3];  /* alignment to 32bits boundry   */
} ACXPreamble_t;


/******************************************************************************

    Name:	ACX_CCA_THRESHOLD
	Type:	Configuration
	Access:	Write Only
	Length: 2
	
******************************************************************************/

typedef struct
{
    INFO_ELE_HDR
    uint16 rxCCAThreshold; /* The Rx Clear Channel Assessment threshold in the PHY*/
	                       /* (the energy threshold).*/
    Bool_e txEnergyDetection; 
    uint8  padding;
} ACXEnergyDetection_t;
	  

/******************************************************************************

    Name:	ACX_EVENT_MBOX_MASK
	Type:	Operation
	Access:	Write Only
	Length: 8
	
******************************************************************************/

typedef struct
{
    INFO_ELE_HDR
    uint32 lowEventMask;   /* Indicates which events are masked and which are not*/
	                       /* Refer to EventMBoxId_enum in public_event_mbox.h.*/
	
    uint32 highEventMask;  /* Not in use (should always be set to 0xFFFFFFFF).*/
} ACXEventMboxMask_t;


/******************************************************************************

    Name:	ACX_WR_TBTT_AND_DTIM
	Type:	Configuration
	Access:	Write Only
	Length: 3
	Note:   STA Only
	
******************************************************************************/
#ifdef FW_RUNNING_AS_STA
typedef struct
{
    INFO_ELE_HDR
    uint16 tbtt;         /* Time in TUs between two consecutive Beacons*/
    uint8  dtimInterval; /* DTIM interval: */
	                     /* For BSS: Number of TBTTs in a DTIM period (range: 1-10).*/
	                     /* For IBSS: value shall be set to 1.*/
    uint8  padding;      /* alignment to 32bits boundry   */
} ACXDtimPeriodCfg_t;
#endif

/******************************************************************************

    Name:	ACX_DTIM_PERIOD
	Type:	Configuration
	Access:	Write Only
	Length: 2
	Note:   for AP only !!!
	
******************************************************************************/
#ifdef FW_RUNNING_AS_AP
typedef struct
{
    uint16 dtimPeriod;
    uint8  padding[2];  /* alignment to 32bits boundry   */
} ACXDtimPeriodCfg_t;
#endif


/******************************************************************************

    Name:	ACX_ACI_OPTION_CFG
	Type:	Configuration
	Access:	Write Only
	Length: 6
	Note:   OBSOLETE !!! (for 1251) 

******************************************************************************/

typedef struct 
{
    INFO_ELE_HDR
    uint8 ACIMode;
    uint8 inputCCA;
    uint8 txCCA;
    uint8 qualifiedCCA;
    uint8 stompForRx;
    uint8 stompForTx;
    uint8 padding[2];  /* alignment to 32bits boundry   */
} ACXConfigACI_t;


/******************************************************************************

    Name:	ACX_PM_CFG
	Desc:   Configure the power managment option.
	Type:	Configuration
	Access:	Write Only
	Length: 16
	Note:   ??? (To Be Documented)

******************************************************************************/

typedef struct
{
    INFO_ELE_HDR
    Bool32 ELPEnable;
    Bool32 WakeOnGPIOenable;
    uint32 BBWakeUpTime;
    uint32 PLLlockTime;
} ACXConfigPM_t;

/******************************************************************************

    Name:	ACX_CONN_MONIT_PARAMS
	Desc:   This information element configures the SYNCHRONIZATION_TIMEOUT 
	        interrupt indicator. It configures the number of missed Beacons 
			before issuing the SYNCHRONIZATION_TIMEOUT event.
	Type:	Configuration
	Access:	Write Only
	Length: 8

******************************************************************************/

typedef struct
{
    INFO_ELE_HDR
    uint32 TSFMissedThreshold; /* The number of consecutive beacons that can be */
	                           /* lost before the WiLink raises the */
	                           /* SYNCHRONIZATION_TIMEOUT event.*/

    uint32 BSSLossTimeout;     /* The delay (in time units) between the time at */
	                           /* which the device issues the SYNCHRONIZATION_TIMEOUT*/
	                           /* event until, if no probe response or beacon is */
	                           /* received a BSS_LOSS event is issued.*/
} AcxConnectionMonitorOptions;

/******************************************************************************

    Name:	ACX_CONS_TX_FAILURE
	Desc:   This information element configures the number of frames transmission
	        failures before issuing the "Max Tx Retry" event. The counter is 
			incremented only for unicast frames or frames that require Ack 
	Type:	Configuration
	Access:	Write Only
    Length: 1
	
******************************************************************************/

typedef struct
{
    INFO_ELE_HDR
    uint8 maxTxRetry; /* the number of frames transmission failures before */
                      /* issuing the "Max Tx Retry" event*/
    uint8  padding[3];  /* alignment to 32bits boundry   */
} ACXConsTxFailureTriggerParameters_t;

/******************************************************************************

    Name:	ACX_LOW_SNR
	Desc:   This information element configures the Low/Regained SNR interrupt 
	        indicator.
	Type:	Configuration
	Access:	Write Only
	Length: 4
	Note:   To Be Documented

******************************************************************************/

typedef enum
{
    LOW_SNR_EVENT_LEVEL = 0,
    LOW_SNR_EVENT_EDGE = 1
} LowSNREventType_enum;
 

#ifdef HOST_COMPILE
typedef uint8 LowSNREventType_e;
#else
typedef LowSNREventType_enum LowSNREventType_e;
#endif
 

typedef struct
{
    INFO_ELE_HDR
    uint8        SNRThreshold;
    uint8        SNRFilterWeight;
    uint8  SNRFilterDepth;
    LowSNREventType_e LowSNREventType;
} ACXLowSNRTriggerParameters_t;


/******************************************************************************

    Name:	ACX_BCN_DTIM_OPTIONS
	Type:	Configuration
	Access:	Write Only
	Length: 5
	
******************************************************************************/

typedef struct 
{    
	INFO_ELE_HDR
    uint16 beaconRxTimeOut;
    uint16 broadcastTimeOut;
    uint8  rxBroadcastInPS;  /* if set, enables receive of broadcast packets */
	                         /* in Power-Save mode.*/
	uint8  consecutivePsPollDeliveryFailureThr;			/* Consecutive PS Poll Fail before updating the Driver */
    uint8  padding[2];       /* alignment to 32bits boundry   */
} ACXBeaconAndBroadcastOptions_t;


/******************************************************************************

    Name:	ACX_SG_ENABLE
	Desc:   This command instructs the WiLink to set the Soft Gemini (BT co-existence)
	        state to either enable/disable or sense mode. 
	Type:	Configuration
	Access:	Write Only
	Length: 1
	
******************************************************************************/
typedef struct
{
    INFO_ELE_HDR
	uint8   Enable; /* specifies whether the SG feature will be enabled (0), */
	                /* disabled(1), in a "sense not active" (3) mode in which */
	                /* upon seeing BT activity a host interrupt will be sent or*/
	                /* in a "sense active" (4) mode which specifies the device */
	                /* should switch on the SG in response to the driver receiving */
	                /* the host interrupt. */
    uint8  padding[3];  /* alignment to 32bits boundry   */
} ACXBluetoothWlanCoEnableStruct;

/******************************************************************************

    Name:	ACX_ANTENNA_DIVERSITY_CFG
	Desc:   Set antenna diversity parameters
	Type:	Configuration
	Access:	Write Only
	Length: 7
	Note:   ??? (To Be Documented)
	
******************************************************************************/

/* Antenna Diversity Tx definitions*/
typedef enum
{
    DIVS_TX_START_ANT1       = 0,     /* Start TX antenna 1.*/
    DIVS_TX_START_ANT2       = 1,     /* Start TX antenna 2.*/
    DIVS_TX_START_SWITCH     = 2     /* Switch starting Tx Antenna.    */
} TxAntDivsStartOption_enum;


/* Antenna Diversity Rx definitions*/
typedef enum
{
    DIVS_RX_START_ANT1       = 0,     /* Start RX antenna 1.*/
    DIVS_RX_START_ANT2       = 1,     /* Start RX antenna 2.*/
    DIVS_RX_START_LAST_RX    = 2,     /* Start RX Last RX Antenna mode.*/
    DIVS_RX_START_SWITCH     = 3     /* Switch starting Rx Antenna.    */
} RxAntDivsStartOption_enum;

#ifdef HOST_COMPILE
typedef uint8 RxAntDivsStartOption_e;
typedef uint8 TxAntDivsStartOption_e;
#else
typedef RxAntDivsStartOption_enum RxAntDivsStartOption_e;
typedef TxAntDivsStartOption_enum TxAntDivsStartOption_e;
#endif

typedef struct
{
    INFO_ELE_HDR
    uint8                   enableRxDiversity;
    RxAntDivsStartOption_e  rxSelectedAntenna;
    uint8                   enableTxDiversity;
    TxAntDivsStartOption_e  txSelectedAntenna;
    uint8                   rxAntNum;
    uint8                   txAntNum;
    uint8                   rxTxSharedAnts;
    uint8                   padding;  /* alignment to 32bits boundry   */
} AcxSetAntennaDiversityOptions_t;



/******************************************************************************

    Name:	ACX_SG_CFG
	Desc:   This command instructs the WiLink to set the Soft Gemini (BT co-existence) 
	        parameters to the desired values. 
	Type:	Configuration
	Access:	Write Only
	Length: 1
	
******************************************************************************/
typedef struct

{
    INFO_ELE_HDR
	RateIndex_e wlanRxMinConvertedRateToRespectBtHp; /* Range: 802.11 b,g Rates*/
												     /* The minimum rate of a received WLAN packet in the STA, */ 
												     /* during protective mode, of which a new BT-HP request */ 
												     /* during this Rx will always be respected and gain the antenna*/	

	uint16 btHpMaxTime;                         /* the maximum length of time the BT HP */
	                                            /* will be respected (Limitation on BT */
	                                            /* HP time, afterwards will switch to LP).*/

    uint16 wlanHpMaxTime;                       /* the maximum length of time the WLAN HP*/
	                                            /* will be respected (Limitation on WLAN */
	                                            /* HP time, afterwards will switch to LP).*/

    uint16 senseDisableTimer;                   /* The time after the last BT activity */
	                                            /* when the sense mode will return the */
	                                            /* SG state to "SENSE_INACTIVE"*/

    uint16 protectiveRxTimeBeforeBtHp;          /* The time before the next BT HP */
	                                            /* instance in which to send the fast */
	                                            /* CTS.*/

    uint16 protectiveTxTimeBeforeBtHp;          /* The time before the next BT HP */
	                                            /* instance in which to suspend the */
	                                            /* WLAN TX*/
	
    uint16 protectiveRxTimeBeforeBtHpFastAp;    /* range: 10-20000    default: 1500*/
    uint16 protectiveTxTimeBeforeBtHpFastAp;    /* range: 10-20000    default: 3000*/
    uint16 protectiveWlanCycleTimeForFastAp;    /* range: 2000-65535  default: 8700*/
    uint16 btAntiStarvationPeriod;				/* range: 0 - 15000 (Msec) default: 1000 */
    uint16 timeoutNextBtLpPacket;				/* range 400-10000(Usec) default: 3000 */
	
	uint16 wakeUpTimeBeforeBeacon;              /* The default value is worse case of */
												/* BT DH5 traffic                     */
	uint16 hpdmMaxGuardTime;					/* range: 0-50000(Usec) default: 1050*/

	uint16 timeoutNextWlanPacket;				/* Range: 100-50000(Usec) default:2550*/
												/* This timeout purpose is to prevent both BT & WLAN */
												/* antenna starvation. */

    uint8  sgAntennaType;                       /* "0" - shared antenna ; */
	                                            /* "1" - dual antenna.*/

    uint8  signalingType;                       /* "0" - TI legacy signaling ; */
	                                            /* "1" - Palau signaling       */

    uint8  afhLeverageOn;                       /* How to receive information regarding */
	                                            /* the AFH status of the BT. */
	                                            /* "0" - no AFH;*/
	                                            /* "1" - from dedicated GPIO.*/
	                                            /* "2" - AFH on (from host).*/
	
    uint8  numberQuietCycle;                    /* the number of cycles during which no*/
	                                            /* TX will be sent after 1 cycle of RX */
	                                            /* transaction in protective mode*/

    uint8  maxNumCts;                           /* The maximum number of CTSs that will*/
	                                            /* be sent for receiving RX packet in */
	                                            /* protective mode*/

    uint8  numberOfWlanPackets;                 /* The number of WLAN packets */
	                                            /* transferred in common mode before */
                                                /* switching to the BT.*/

    uint8  numberOfBtPackets;                   /* The number of BT packets transferred*/
	                                            /* in common mode before switching to */
	                                            /* the WLAN.*/
			
    uint8  numberOfMissedRxForAvalancheTrigger; /* range: 1-255  default: 5*/
    uint8  wlanElpHpSupport;                    /* range: 0-1    default: 1*/

	uint8  btAntiStarvationNumberOfCyclesWithinThePeriod; /* range: 0 - 15  default: 4 */
	
    uint8  ackModeDuringBtLpInDualAnt;                  /* 0 or 1  */

	
    Bool_e allowPaSdToggleDuringBtActivityEnable; /* Allow PA_SD assertion/de-assertion */ 
                                                  /* during enabled BT activity         */

	Bool_e sgAutoModeNoCts;					    /* Enable/Disable SG2.0 in auto mode: */
												/* Support Both Active & P.S modes */
	
	uint8  numOfBtHpRespectedReq;				/*range: 0 - 20  default: 1*/	
    
} ACXBluetoothWlanCoParamsStruct;
  


/******************************************************************************

    Name:	ACX_TSF_INFO
	Type:	Operation
	Access:	Read Only
	Length: 20

******************************************************************************/
typedef struct ACX_fwTSFInformation
{
    INFO_ELE_HDR
    uint32 CurrentTSFHigh;
    uint32 CurrentTSFLow;
    uint32 lastTBTTHigh;
    uint32 lastTBTTLow;
    uint8 LastDTIMCount;
    uint8  padding[3];  /* alignment to 32bits boundry   */
}ACX_fwTSFInformation_t;

 


/******************************************************************************

    Name:	ACX_GPIO_CFG
	Type:	Board Configuration
	Access:	Write Only
	Length: 2
	Note:   Not in use !!!
	
******************************************************************************/

#ifndef _WINDOWS
#define GPIO_DIR_OUTPUT    0
#define GPIO_DIR_INPUT     1
#endif /* ifndef _WINDOWS */

typedef struct
{
    INFO_ELE_HDR
    uint8   number;
    uint8   direction;
    uint8   padding[2];  /* alignment to 32bits boundry   */
} ACXConfigGPIO_t;


/******************************************************************************

    Name:	ACX_GPIO_SET
	Type:	Board Configuration
	Access:	Write Only
	Length: 2
	Note:   Not in use !!!
	
******************************************************************************/

typedef struct
{
    INFO_ELE_HDR
    uint8   number;
    uint8   value;
    uint8   padding[2];  /* alignment to 32bits boundry   */
} ACXSetGPIO_t;

/******************************************************************************

    Name:	ACX_MISC_CFG
	Type:	Board Configuration
	Access:	Read/Write
	Length:	8
	Note:	GPIO_OUT bits to be used for LEDs are defined in the
			NVS Miscellaneous table.  An API to NVS is used by the
			LED-Init routine to fill this table with the LED bit values.
			Not in use !!!
	
******************************************************************************/

typedef struct
{
    INFO_ELE_HDR
    uint16 txActivityLed;      /* GPIO_OUT bit for this LED*/
    uint16 fwInitLed;          /* GPIO_OUT bit for this LED*/
    uint16 diagnosticLed;      /* GPIO_OUT bit for this LED*/
    uint16 faaRadioOffGpio;    /* GPIO_OUT bit for the FAA Radio Off feature*/
} ACXMisc_t;


/******************************************************************************

Name:	ACX_BET_ENABLE
Desc:   Enable or Disable the Beacon Early Termination module. In addition initialized the
        Max Dropped beacons parameter
Type:	Configuration
Access:	Write 
Length: 6
Note:  
******************************************************************************/
typedef struct

{
    INFO_ELE_HDR
    uint8           Enable;                                     /* specifies if beacon early termination procedure is enabled or disabled: 0 – disabled, 1 – enabled */
    uint8           MaximumConsecutiveET;           /* specifies the maximum number of consecutive beacons that may be early terminated. After this number is reached 
                                                       at least one full beacon must be correctly received in FW before beacon ET resumes.  Legal range: 0 – 255 */
    uint8           padding[2];
}ACXBet_Enable_t;


/******************************************************************************

    Name:	DOT11_STATION_ID
	Desc:   This information element specifies the MAC Address assigned to the STA. 
	        This default value is the permanent MAC address that is stored in the 
			adaptor's non volatile memory. The host can change the MAC address; 
			however, the WiLink always reverts to the default value after power up 
			or reset.
	Type:	Configuration
	Access:	Read / Write 
	Length: 6
	Note:   The byte order of the MAC address must be reversed in this field. 
	        For example, if the MAC address is 00 7E 99 11 22 33, this field must 
			read 33 22 11 99 7E 00.
	
******************************************************************************/

typedef struct
{
    INFO_ELE_HDR
    uint8 dot11StationID[6]; /* The MAC address for the STA.*/
    uint8 padding[2];  /* alignment to 32bits boundry   */
} dot11StationIDStruct;


/******************************************************************************

    Name:	DOT11_RX_MSDU_LIFE_TIME
	Type:	Operation
	Access:	Write Only
	Length: 4
	
******************************************************************************/

typedef struct
{
    INFO_ELE_HDR
    uint32 RxMsduLifeTime; /* The maximum amount of time, in TU, that the WiLink */
	                       /* should attempt to collect fragments of an MSDU before */
	                       /* discarding them. */
                           /* The default value for this field is 512.*/
} dot11RxMsduLifeTime_t;


/******************************************************************************

    Name:	DOT11_CUR_TX_PWR
	Desc:   This IE indicates the maximum TxPower in Dbm/10 currently being used to transmit data.
	Type:	Operation
	Access:	Write Only
	Length: 1
	
******************************************************************************/

typedef struct
{ 
    INFO_ELE_HDR
    uint8 dot11CurrentTxPower; /* the max Power in Dbm/10 to be used to transmit data.*/
    uint8  padding[3];  /* alignment to 32bits boundry   */
} dot11CurrentTxPowerStruct ;



/******************************************************************************

    Name:	DOT11_DEFAULT_KEY
	Desc:   This information element indicates the default key to use to encrypt
	        transmit frames.
	Type:	Configuration
	Access:	Write Only
	Length: 1
	
******************************************************************************/

typedef enum
{
    DEFAULT_KEY_0 = 0,
    DEFAULT_KEY_1,
    DEFAULT_KEY_2,
    DEFAULT_KEY_3,
    MAX_NUM_WEP_DEFAULT_KEY
} DefaultKey_enum;

#ifdef HOST_COMPILE
typedef uint8 DefaultKey_e;
#else
typedef DefaultKey_enum DefaultKey_e;
#endif

typedef struct
{
    INFO_ELE_HDR
    DefaultKey_e DefaultKeyId; /*  refer to DefaultKey_enum*/
    uint8  padding[3];  /* alignment to 32bits boundry   */
} dot11WEPDefaultKeyId_t;


/******************************************************************************

    Name:	DOT11_RX_DOT11_MODE
	Desc:   This IE indicates the current Rx Mode used by DSSS PHY.
	Type:	Configuration
	Access:	Write Only
    Length: 4
	
******************************************************************************/
/*
Possible values for Rx DOT11 Mode are the following:
Value	Description
=====   ===========
3	    11g - processing of both a and b packet formats is enabled
2	    11b - processing of b packet format is enabled
1	    11a - processing of a packet format is enabled
0	    undefined
*/

typedef struct
{
    INFO_ELE_HDR
    uint32 dot11RxDot11Mode; /* refer to above table*/
} dot11RxDot11ModeStruct;


/******************************************************************************

    Name:	DOT11_RTS_THRESHOLD 
	Type:	Configuration
	Access:	Write Only
	Length: 2

******************************************************************************/

typedef struct 
{
    INFO_ELE_HDR
    uint16  RTSThreshold; /* The number of octets in an MPDU, below which an */
	                      /* RTS/CTS handshake is not performed.*/
	
    uint8  padding[2];  /* alignment to 32bits boundry   */
}dot11RTSThreshold_t;


/******************************************************************************

	Name:	DOT11_GROUP_ADDRESS_TBL
	Desc:   The variable lengths of MAC addresses that are define as listening for
	        multicast. The field Number of groups identifies how many MAC Addresses 
			are relevant in that information element.
	Type:	Configuration
	Access:	Write Only
	Length: up to 50 bytes

******************************************************************************/
#define ADDRESS_GROUP_MAX		(8)
#define ADDRESS_GROUP_MAX_LEN	(6 * ADDRESS_GROUP_MAX)
typedef struct 
{
    INFO_ELE_HDR
	uint8	fltrState;	                         /* 1 - multicast filtering is enabled. */
	                                             /* 0 - multicast filtering is disabled.*/

    uint8   numOfGroups;                         /* number of relevant multicast */
	                                             /* addresses.*/

    uint8   padding[2];  /* alignment to 32bits boundary   */
    uint8   dataLocation[ADDRESS_GROUP_MAX_LEN]; /* table of MAC addresses.*/
}dot11MulticastGroupAddrStart_t;

/******************************************************************************

   ACX_CONFIG_PS_WMM (Patch for Wi-Fi Bug)

******************************************************************************/

typedef struct 
{    
    INFO_ELE_HDR
    Bool32      ConfigPsOnWmmMode;  /* TRUE  - Configure PS to work on WMM mode - do not send the NULL/PS_POLL 
                                               packets even if TIM is set.
                                       FALSE - Configure PS to work on Non-WMM mode - work according to the 
                                               standard. */
} IEConfigPsWmm_t;

/******************************************************************************

    ACX_SET_RX_DATA_FILTER  

******************************************************************************/
/* data filter action */

#ifdef HOST_COMPILE

#define FILTER_DROP  0
#define FILTER_SIGNAL  1
#define FILTER_FW_HANDLE  2

#else

typedef enum {
	FILTER_DROP = 0,
	FILTER_SIGNAL  ,
    FILTER_FW_HANDLE, 
	FILTER_MAX  = 0xFF
}filter_enum;

#endif

#ifdef HOST_COMPILE
typedef uint8 filter_e;
#else
typedef filter_enum filter_e;
#endif

/* data filter command */
#define REMOVE_FILTER   0
#define ADD_FILTER      1

/* limitation */
#define MAX_DATA_FILTERS 4
#define MAX_DATA_FILTER_SIZE 90

typedef struct 
{
    INFO_ELE_HDR
    uint8                command; // 0-remove, 1-add
    uint8                index;
    filter_e             action;
    uint8                numOfFields;
    uint8                FPTable; //filter fields starts here. unknown size.
} DataFilterConfig_t;

/******************************************************************************

    ACX_ENABLE_RX_DATA_FILTER  

******************************************************************************/

typedef struct  
{
    INFO_ELE_HDR
    uint8       enable;
    filter_e    action;
} DataFilterDefault_t;


/******************************************************************************

    ACX_GET_DATA_FILTER_STATISTICS  

******************************************************************************/

typedef struct 
{
    INFO_ELE_HDR
    uint32  unmatchedPacketsCount;
    uint32  matchedPacketsCount[MAX_DATA_FILTERS];
} ACXDataFilteringStatistics_t;


/******************************************************************************

Name:	ACX_POWER_LEVEL_TABLE
Desc:   Retrieve Maximum Dbm per power level and sub-band.
Type:	Configuration
Access:	Read Only
Length: 20 

******************************************************************************/

typedef struct
{
	INFO_ELE_HDR

	uint8 txPowerTable[NUM_OF_SUB_BANDS][NUM_OF_POWER_LEVEL]; /* Maximun Dbm in Dbm/10 units */
} PowerLevelTable_t;


/******************************************************************************

Name:	ACX_PWR_CONSUMPTION_STATISTICS
Desc:   Retrieve time statistics of the different power states.
Type:	Configuration
Access:	Read Only
Length: 20 

******************************************************************************/

typedef struct
{
	INFO_ELE_HDR
    uint32 activeTimeCnt_Low;
    uint32 activeTimeCnt_Hi;
    uint32 powerDownTimeCnt_Low;
    uint32 powerDownTimeCnt_Hi;
    uint32 elpTimeCnt_Low;
    uint32 elpTimeCnt_Hi;
}ACXPowerConsumptionTimeStat_t;

#endif // PUBLIC_INFOELE_H

