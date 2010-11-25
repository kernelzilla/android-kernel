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

#ifndef _WHAL_CTRL_API_H
#define _WHAL_CTRL_API_H

#include "whalCtrl_prm.h"
#include "public_infoele.h"
#include "whalBus_Defs.h"
#include "MacServices_api.h"


#define MEASUREMENT_NOISE_HISTOGRAM_NUM_OF_RANGES       8
#define MAX_CHANNELS_IN_REG_DOMAIN	40

#define		CTS_TO_SELF_DISABLE		0
#define		CTS_TO_SELF_ENABLE		1

#define MAX_TEMPLATE_SIZE	256
typedef struct
{
	UINT32	Size;
	UINT8	Buffer[MAX_TEMPLATE_SIZE];
} TemplateParams_T;

typedef enum
{
    NULL_DATA_TEMPLATE = 0,
    BEACON_TEMPLATE,        
    PROBE_REQUEST_TEMPLATE,     
    PROBE_RESPONSE_TEMPLATE,
    QOS_NULL_DATA_TEMPLATE,
    PS_POLL_TEMPLATE,
} whalCtrl_templateType_e;

typedef enum{
    WHAL_WME = 0,
    WHAL_NONE_QOS,
}whalCtrl_qosProtocols_e;

/* Do not change the order */
typedef enum
{
    PS_DISABLE = 0,
    PS_ENABLE = 1,
} whalCtrl_psState_e;

typedef enum
{
    PS_POLL_ACCESS_MNGR_GEN= 0,
    PS_POLL_HOST_ECPU_GEN,
} whalCtrl_psPollGen_e;

typedef enum
{
	ENTER_PS_FAIL,
	ENTER_PS_SUCCESS,
	EXIT_PS_FAIL,
	EXIT_PS_SUCCESS,
}Hw_Event_PS_Status;

/** Available cipher suites for admission control */
typedef enum
{
    HAL_CTRL_CIPHER_NONE     = 0,        /**< no chpiher suite */
    HAL_CTRL_CIPHER_WEP      = 1,        /**< WEP-40 chpiher suite */
    HAL_CTRL_CIPHER_TKIP     = 2,        /**< TKIP chpiher suite */
    HAL_CTRL_CIPHER_AES_WRAP = 3,        /**< AES WRAP chpiher suite */
    HAL_CTRL_CIPHER_AES_CCMP = 4,        /**< AES CCMP chpiher suite */
    HAL_CTRL_CIPHER_WEP104   = 5,        /**< WEP-104 chpiher suite */
    HAL_CTRL_CIPHER_CKIP     = 6,        /**< CKIP chpiher suite */
    HAL_CTRL_CIPHER_UNKNOWN  = 255       /**< UNKNOWN chpiher suite */
} halCtrl_CipherSuite_e;


/*
 *	The Event Mail Box ID : Clients That expected an event should
 *							register,Mask/UnMask Events with this ID
 */

typedef enum 
{
    /*Regular events*/
	HAL_EVENT_MEASUREMENT_START,			/*0*/ 
	HAL_EVENT_SCAN_CMPLT,					/*1*/ 
	HAL_EVENT_CALIB_CMPLT,					/*2*/ 
	HAL_EVENT_RSSI_LEVEL,					/*3*/ 
	HAL_EVENT_PS_REPORT,					/*4*/ 
	HAL_EVENT_SYNCHRONIZATION_TIMEOUT,		/*5*/ 
	HAL_EVENT_HEALTH_REPORT,				/*6*/ 
	HAL_EVENT_ACI,							/*7*/ 
	HAL_EVENT_DEBUG_MESSAGE,				/*8*/ 
	HAL_EVENT_MAC_STATUS,					/*9*/ 
	HAL_EVENT_DISCONNECT_COMPLETE,			/*10*/
    HAL_EVENT_JOIN_CMPLT,					/*11*/
	HAL_EVENT_SWITCH_CHANNEL_CMPLT,			/*12*/
	HAL_EVENT_BSS_LOSE,						/*13*/
	HAL_EVENT_MAX_TX_RETRY,					/*14*/
    HAL_EVENT_MEASUREMENT_COMPLETE,			/*15*/
	HAL_EVENT_AP_DISCOVERY_COMPLETE,		/*16*/
	HAL_EVENT_SPS_SCAN_CMPLT,				/*17*/
	HAL_EVENT_BSS_REGAIN,					/*18*/
	HAL_EVENT_RSSI_LEVEL_REGAIN,			/*19*/
	HAL_EVENT_LOW_SNR,						/*20*/
	HAL_EVENT_SOFT_GEMINI_SENSE,			/*21*/
	HAL_EVENT_SOFT_GEMINI_PREDICTION,		/*22*/
	HAL_EVENT_SOFT_GEMINI_AVALANCHE,	  	/*23*/
    HAL_EVENT_PLT_RX_CALIBRATION_COMPLETE,	/*24*/
    HAL_EVENT_PSPOLL_DELIVERY_FAILURE,      /*25*/
    HAL_EVENT_BSS_RESET,            	    /*26*/
	HAL_EVENT_ALL,      /* 27 */
	MAX_NUM_OF_EVENT, /* 28 */
}Hw_Event_ID;   


typedef struct
{
    macAddress_t    macAddress;
    preamble_e      preamble;
    radioType_e     radioType;
    radioBand_e     radioBand;
    UINT8           fwVer[FW_VERSION_LEN]; /* Firmware version - null terminated string*/
    e2Version_t     e2Ver;                 /* EEPROM version*/
} whalCtrl_chip_t;  

typedef struct
{
    TI_HANDLE			hRxXfer; 
	TI_HANDLE			hTxXfer;
	TI_HANDLE			hTxResult;
	TI_HANDLE			hTxHwQueue; 
	TI_HANDLE			hReport; 
	TI_HANDLE			hMemMgr;
	TI_HANDLE			hEventMbox;
	TI_HANDLE			hFwEvent;
    TI_HANDLE           hCmdQueue;
#ifdef TI_DBG
	TI_HANDLE			hDebugTrace;
#endif /* TI_DBG */
} whalCtrl_config_t;



typedef struct
{
    bssType_e   bssType;
    UINT16      beaconInterval;
    UINT16      dtimInterval;
    UINT8       channel;
    UINT8*      pBSSID;
    UINT8*      pSSID;
    UINT8       ssidLength;
    rate_e      hwGenCtrlTxRate;
    rate_e      hwGenMgmtTxRate;		/* Beacon and Probe-Response Tx rate (in IBSS). */
    UINT16      basicRateSet;       
    UINT16      supportedRateSet;   
    radioBand_e radioBand;
    preamble_e  preamble;
} whalCtrl_joinBss_t;

typedef struct
{
    whalCtrl_templateType_e templateType;
    UINT8*                  pTemplate;
    UINT32                  templateLen;
} whalCtrl_setTemplate_t;

PACKED_STRUCT( whalCtrl_antennaDiversityOptions_t,

    uint8   enableRxDiversity;
    uint8   rxSelectedAntenna;
    uint8   enableTxDiversity;
    uint8   txSelectedAntenna;
    uint8   rxTxSharedAnts;
);

typedef enum
{
     STOP_NOISE_HIST  = 0,
     START_NOISE_HIST = 1
} NoiseHistogramCmd_e;

typedef struct
{
    NoiseHistogramCmd_e     cmd;
    UINT16                  sampleInterval;
    UINT8                   ranges[MEASUREMENT_NOISE_HISTOGRAM_NUM_OF_RANGES];
} whalCtrl_noiseHistogram_t;

/* use this struct when expecting a CB after a CONFIGURE_ACTION */
PACKED_STRUCT( acxStatisitcs_t,

    UINT32  FWpacketReceived;
    UINT32  HALpacketReceived;
);

PACKED_STRUCT( mediumOccupancy_t,

    interogateCmdHdr_t  mediumOccupCmdHdr;
    UINT32  MediumUsage;
    UINT32  Period;
);

PACKED_STRUCT( tsf_dtim_mib_t,

    interogateCmdHdr_t  tsf_dtim_mibCmdHdr;
    uint32 CurrentTSFHigh;
    uint32 CurrentTSFLow;
    uint32 lastTBTTHigh;
    uint32 lastTBTTLow;
    uint8 LastDTIMCount;
    uint8 Reserved[3];
);


typedef struct 
{
	UINT16		BeaconRxTimeout;
	UINT16		BroadcastRxTimeout;
	UINT8		RxBroadcastInPs;
} BcnBrcOptions_t;

typedef struct
{
	UINT8 		numberOfIEs;
	UINT8 		IETable[BEACON_FILTER_TABLE_MAX_SIZE];
	UINT8 		IETableSize;
} beaconFilterIETable_t;


typedef struct
{
    UINT8   beaconListenInterval;
    UINT8   beaconFiltering;
    UINT8   DTIMListenInterval;
    UINT8   NConsecutiveBeaconMiss;
    UINT8   hangoverPeriod;
	UINT8	HwPsPollResponseTimeout;
	UINT32  BaseBandWakeUpTime;
    UINT32  beaconReceiveTime;
    BOOLEAN beaconMissInterruptEnable;
    BOOLEAN rxBroadcast;
    BOOLEAN hwPsPoll;
    

	/* powerMgmtConfig IE */
    BOOLEAN 	ps802_11Enable;
    UINT8		needToSendNullData;  
    UINT8		numNullPktRetries; 
    UINT8		hangOverPeriod;
    UINT16		NullPktRateModulation; 

	/* PMConfigStruct */
	BOOLEAN		ELPEnable;
	UINT32		BBWakeUpTime;
	UINT32		PLLlockTime;

	/* AcxBcnBrcOptions */
	BcnBrcOptions_t BcnBrcOptions;

	/* ACXWakeUpCondition */
    PowerMgr_TnetWakeOn_e tnetWakeupOn;  
    UINT8	listenInterval;

    /* No answer after Ps-Poll work-around */
    UINT8  ConsecutivePsPollDeliveryFailureThreshold;
} whalCtrl_powerMgmtConfig_t;

typedef struct
{
    
	/* powerMgmtConfig IE */
    BOOLEAN 					ps802_11Enable;
    UINT8						needToSendNullData;  
    UINT8						numNullPktRetries; 
    UINT8						hangOverPeriod;
    UINT16					NullPktRateModulation;
    void * 						powerSaveCBObject;
    MacServices_powerSaveCmdResponseCB_t	powerSavecmdResponseCB;
	
	
} whalCtrl_powerSaveParams_t;

/*MULTIPLE QUEUES STRUCTURE */

typedef struct
{
    UINT16      txopLimit;
    UINT16      rxTimeout;
    UINT8       aifsn;
    UINT8       cwMin;
    UINT16      cwMax;
    UINT8       acId;
    UINT8       shortRetryLimit;
    UINT8       longRetryLimit;
    UINT8       retryPreemption;
    UINT8       txopContinuation;
	UINT8       PsParameters;
} whaCtrl_acTrafficParams_t;


typedef struct  
{
	UINT8              acId;
    UINT8              qId;
    UINT16 		       percentOfBlockHighThreshold; /* Sum of ACs > numTxMemBlks in ~10%    */
    UINT16 		       percentOfBlockLowThreshold;  /* Sum of ACs <  numTxMemBlks           */
} acQueuesParams_t; 


typedef enum{
	HAL_CTRL_AC_NOT_ADMITTED,
	HAL_CTRL_AC_WAIT_ADMISSION,
	HAL_CTRL_AC_ADMITTED
} whalCtrl_trafficAdmState_e;

typedef enum{
	HAL_CTRL_UNIDIRECTION,
	HAL_CTRL_BIDIRECTION,
} whalCtrl_streamDirection_e;

typedef enum{
	HAL_CTRL_ADDTS_RESPONSE_ACCEPT = 0,
/*	HAL_CTRL_ADDTS_RESPONSE_REJECT,  - according to the standard*/
	HAL_CTRL_ADDTS_RESPONSE_AP_PARAM_INVALID = 253,
	HAL_CTRL_ADDTS_RESPONSE_TIMEOUT = 254,
	HAL_CTRL_TSPEC_DELETED_BY_AP = 255,
} whalCtrl_addtsRaeasonCode_e;

typedef enum{
	HAL_CTRL_HIGH_THRESHOLD_CROSS,
	HAL_CTRL_LOW_THRESHOLD_CROSS,
} whalCtrl_thresholdCross_e;

typedef enum{
	HAL_CTRL_CROSS_ABOVE,
	HAL_CTRL_CROSS_BELOW,
} whalCtrl_thresholdCrossDirection_e;

typedef struct{
	UINT32 acID;
	thresholdCross_e thresholdCross;
} tspecRateCross_t;

typedef struct{
	UINT32 acID;
	UINT32 thresholdCross;
	UINT32 thresholdCrossDirection;
} mediumTimeCross_t;

/************************************/
/*      QOS edcf params             */
/************************************/

/*=================== Mesurement =====================*/

typedef struct 

{
	uint32      ConfigOptions;
    uint32      FilterOptions;
	uint32      duration;
	Channel_e   channel;
	RadioBand_e band;
} whalCtrl_MeasurementParameters_t;

typedef struct 

{
	uint32      	ConfigOptions;
    uint32      	FilterOptions;
	uint32      	scanDuration;
	uint16      	scanOptions;
	uint8       	numOfProbRqst;
	TxdRateSet_t 	txdRateSet;
	uint8 			txPowerDbm;    
} whalCtrl_ApDiscoveryParameters_t;

/*=======================================================*/


typedef struct
{
    /* ACXLowRSSITriggerParameters */
	INT8   rssiThreshold;
    UINT8  rssiFilterWeight;
    UINT8  rssiFilterDepth;
    UINT8  lowRSSIEventType;

    /* ACXLowSNRTriggerParameters */
    UINT8  snrThreshold;
    UINT8  snrFilterWeight;
    UINT8  snrFilterDepth;
    UINT8  lowSNREventType;

	/* ACXConsNackTriggerParameters */
    UINT8  maxTxRetry;

	/* ACXBssLossTsfSynchronize */
    UINT16  TsfMissThreshold;
    UINT16  BssLossTimeout;
} whalCtrl_roamingTriggerCmd_t;


typedef struct
{
    UINT8     		channelNumber;
    UINT8           switchTime;
    UINT8           txFlag;
	UINT8			flush;
} whalCtrl_switchChannelCmd_t;

typedef scan_Params_t whalCtrl_scan_t;


PACKED_STRUCT( whalCtrl_counters_t,

    UINT32  RecvError;            /* the number of frames that a NIC receives but does not indicate to the protocols due to errors*/
    UINT32  RecvNoBuffer;         /* the number of frames that the NIC cannot receive due to lack of NIC receive buffer space     */
    UINT32  FragmentsRecv;
    UINT32  FrameDuplicates;
    UINT32  FcsErrors;
);

/*
PACKED_STRUCT(whalCtrl_PLT_TX_Continues_t,
	UINT8 rate;
	UINT8 chID;
	UINT8 preamble;
	UINT8 bandID;
	UINT8 Modulation;
	UINT8 TestMode;
);

PACKED_STRUCT(whalCtrl_PLT_TX_CW_t,
	UINT8 chID;
	UINT8 bandID;
);
*/

PACKED_STRUCT(whalCtrl_WriteRegister_t,
	UINT32 RegAdress;
	UINT32 RegData;
);


typedef struct
{
    uint8       channel;
    int8        maxRegPower;
} whalCtrl_RegulatoryChannelPowerConstraint_t;

typedef struct 
{
    uint8       numberOfChannels;
    RadioBand_e band;
    whalCtrl_RegulatoryChannelPowerConstraint_t channelListConstraint[MAX_CHANNELS_IN_REG_DOMAIN];
} whalCtrl_RegulatoryPowerConstraint_t;

typedef struct  
{
    INFO_ELE_HDR
    int8           powerConstraintOnBss;  /* The attenuation from the regulatory */
                                            /* power constraint as declared by the AP */
                                            /* Units: dBm*/
                                            /* Range: -20 - 30*/
} whalCtrl_ApPowerConstraint_t;

typedef struct  
{
    INFO_ELE_HDR
    uint8           attenuation;            /* Attenuation from the maximum radio transmit */
                                            /* power as declared by the driver/application*/
                                            /* Units: dB*/
                                            /* Range: 0 - 20*/
} whalCtrl_PowerAttenuation_t;

typedef struct  
{
    INFO_ELE_HDR
    int8            currentTransmitPower;   /* Current transmit power of the radio.*/
                                                /* Units: dBm*/
} whalCtrl_GetCurrentTxPower_t;

/*
 *	TX Ack Policy
 */
typedef struct 
{
	AckPolicy_e  AckPolicy;
	UINT8 AcId;
}AcAckPolicy_t;

typedef union
{

    
        /* HAL Control section */
        UINT16                  halCtrlRtsThreshold;
		UINT8					halCtrlCtsToSelf;
		rxTimeOut_t				halCtrlRxTimeOut;
        UINT16                  halCtrlFragThreshold;
        UINT16                  halCtrlListenInterval;
        UINT16                  halCtrlCurrentBeaconInterval;
        UINT8                   halCtrlTxPowerDbm;
        txAntenna_e             halCtrlTxAntenna;
        rxAntenna_e             halCtrlRxAntenna;
        UINT8                   halCtrlAifs;
        BOOL                    halCtrlTxMemPoolQosAlgo;
        BOOL                    halCtrlClkRunEnable;
        whalCtrl_counters_t     halCtrlCounters;

		PltTxContinues_t        PltTxContinues;
		TestCmdChannelBand_t	PltCW;
        PLT_MIB_t               PltMib;
		

        UINT8                   halCtrlCurrentChannel;

        UINT8                   earlyWakeup;
		
        /* tx data qos related parameters */
        txDataQosParams_t       txDataQosParams;
        
        /*
         *	WME Tx Parameters 
         */

		/* ac queues parameters */
		acQueuesParams_t acQueuesParams;

        /* Access Category Ack Policy */
        AcAckPolicy_t			AcAckPolicy;
        /* Qos Type For Tx Descriptor */
        qosProtocols_e          halCtrlTxQosType;
        /* ac traffic parameters */
        whaCtrl_acTrafficParams_t  acTrafficParams;
        /* queue params */
        acQueuesParams_t        acQueuesParam;

		/* ac Qos parameters */
		queueTrafficParams_t	*pQueueTrafficParams;
      
        /* Security related parameters */
#ifdef EXC_MODULE_INCLUDED
        BOOL                    rsnExcSwEncFlag;
        BOOL                    rsnExcMicFieldFlag;
#endif
        halCtrl_CipherSuite_e   rsnEncryptionStatus;
        UINT8                   rsnHwEncDecrEnable; /* 0- disable, 1- enable*/
        securityKeys_t			*pRsnKey;
        UINT8                   rsnDefaultKeyID;

        /*
        Power Control
        */

		powerAutho_PowerPolicy_e minPowerPolicy;
        
        /* Measurements section */
        acxStatisitcs_t             acxStatisitics;
        mediumOccupancy_t           mediumOccupancy;
        BOOLEAN                     halTxOpContinuation;

		tsf_dtim_mib_t				fwTsfDtimInfo;

        interogateCmdCBParams_t	interogateCmdCBParams;
		configureCmdCBParams_t	configureCmdCBParams;

		txRatePolicy_t			*pTxRatePlicy;

        /* Antenna diversity options */
        whalCtrl_antennaDiversityOptions_t antennaDiversityOptions;
        UINT8                   antennaNum;
        
        /* WARNING!!! This section is used to set/get internal params only. */
        UINT16                  halCtrlAid;
		

		SoftGeminiEnableModes_e    SoftGeminiEnable;
		SoftGeminiParam_t     SoftGeminiParam;
		
		UINT32			halCtrlMaxRxMsduLifetime;
        
		/* Utils section */
		UINT8			SeverityTable[WLAN_MAX_SEVERITIES];
		UINT8			ModuleTable[WLAN_MAX_LOG_MODULES];
		UINT32			reportPPMode;
        UINT32          osDbgState;

		/* Beacon Broadcast options */
		BcnBrcOptions_t	BcnBrcOptions;

        /* PLT tests */
        TI_STATUS             PltRxCalibrationStatus;

}whalParamContents;


typedef struct{

    UINT32              paramType;
    UINT32              paramLength;
	whalParamContents	content;
}whalParamInfo_t;

typedef enum
{
    GWSI_TX_CB_MODULE_OWNER               = 0x0100,  /* WHAL TX MODULE */
    GWSI_RX_CB_MODULE_OWNER               = 0x0200,	 /* WHAL RX MODULE */
    GWSI_EVENT_IND_CB_MODULE_OWNER        = 0x0400,	 /* WHAL HW EVENT MBOX */

    /*
    Last CB module owner- DO NOT TOUCH!
    */
    GWSI_CB_LAST_MODULE_OWNER			  = 0x0500

}   GwsiCB_ModuleOwner_e;

typedef enum 
{
    PLT_RX_PER_START,
    PLT_RX_PER_STOP,
    PLT_RX_PER_CLEAR,
    PLT_RX_PER_GETRESULTS,
    PLT_RX_PER_START_PH2,
    PLT_RX_PER_MAX    /* Must be last*/
}PLT_RxPerCmd_e;


typedef enum
{
	/* Tx Data Path Callbacks */
    GWSI_TX_SEND_PACKET_TRANSFER_CB       =  GWSI_TX_CB_MODULE_OWNER | 0x01,  /* WHAL TX MODULE */
    GWSI_TX_SEND_PACKET_COMPLETE_CB       =  GWSI_TX_CB_MODULE_OWNER | 0x02,  /* WHAL TX MODULE */
    GWSI_TX_QUEUE_FREE_EVENT_CB			  =  GWSI_TX_CB_MODULE_OWNER | 0x03,  /* WHAL TX MODULE */

	/* Rx Data Path Callbacks */
    GWSI_RX_REQUEST_FOR_BUFFER_CB         =  GWSI_RX_CB_MODULE_OWNER | 0x01,  /* WHAL RX MODULE */
    GWSI_RX_RECEIVE_PACKET_CB			  =  GWSI_RX_CB_MODULE_OWNER | 0x02,  /* WHAL RX MODULE */

}GwsiCB_Type_e;

typedef enum
{
	/* Tx Data Path Callbacks */
    HAL_INT_SEND_PACKET_TRANSFER	=  0x00	,	/* WHAL TX MODULE */
    HAL_INT_SEND_PACKET_COMPLETE			,	/* WHAL TX MODULE */
    HAL_INT_QUEUE_FREE_EVENT				,	/* WHAL TX MODULE */

	/* Rx Data Path Callbacks */
    HAL_INT_RECEIVE_PACKET			=  0x10	,  /* WHAL RX MODULE */
    HAL_INT_REQUEST_FOR_BUFFER				,  /* WHAL RX MODULE */
	
	/* Ctrl Callbacks */
	HAL_INT_COMMAND_COMPLETE		=  0x20 ,	
	
	HAL_INTERNAL_EVENT_FAILURE			

}Hw_RegisterID_e;

/*
 * WHAL CTRL Class API
 */         
TI_HANDLE whalCtrl_Create (TI_HANDLE hOs);
int  whalCtrl_Stop              (TI_HANDLE hWhalCtrl);
TI_STATUS whalCtrl_Config    (TI_HANDLE hWhalCtrl, TI_HANDLE hTNETW_Driver, whalCtrl_config_t* pWhalCtrlCfg,UINT32 *pFWImage);
TI_HANDLE whalCtrl_GetTnetwifHandle (TI_HANDLE hWhalCtrl);
TI_HANDLE whalCtrl_GetWhalParams (TI_HANDLE hWhalCtrl);
int  whalCtrl_ConfigHw       (TI_HANDLE hWhalCtrl, TnetwDrv_InitParams_t* pInitParams, void *fCb, TI_HANDLE hCb);
int  whalCtrl_ReConfig          (TI_HANDLE hWhalCtrl, int DoReJoin);
void whalCtrl_RecoveryEnded  (TI_HANDLE hWhalCtrl);
int  whalCtrl_Destroy	  (TI_HANDLE pWhalCtrl);
void whalCtrl_Register_CB	(TI_HANDLE pWhalCtrl,tiUINT32 CallBackID,void *CBFunc,TI_HANDLE CBObj);
void			whalCtrl_PreRecoveryProcess(TI_HANDLE hWhalCtrl);
TI_STATUS whalCtrl_FinalizeDownload (TI_HANDLE hWhalCtrl);
TI_STATUS whalCtrl_FinalizeOnFailure (TI_HANDLE hWhalCtrl);

/* 
 *	Set/Get API
 */
int		whalCtrl_setRxFilters		 (TI_HANDLE hWhalCtrl, UINT32 RxConfigOption, UINT32 RxFilterOption);
int	 whalCtrl_GetRxFilters		(TI_HANDLE hWhalCtrl, UINT32* pRxConfigOption, UINT32* pRxFilterOption);
int  whalCtrl_setRxDataFiltersParams(TI_HANDLE hWhalCtrl, BOOL enabled, filter_e defaultAction);
int  whalCtrl_setRxDataFilter   (TI_HANDLE hWhalCtrl, UINT8 index, UINT8 command, filter_e action, UINT8 numFieldPatterns, UINT8 lenFieldPatterns, UINT8 * fieldPatterns);
int  whalCtrl_getRxDataFiltersStatistics(TI_HANDLE hWhalCtrl, void * CB_Func, TI_HANDLE CB_handle, void * CB_Buf);
int  whalCtrl_SetMacAddress     (TI_HANDLE hWhalCtrl, macAddress_t *macAddr);
int  whalCtrl_SetTemplate       (TI_HANDLE pWhalCtrl, whalCtrl_setTemplate_t* pTemplateParams);
int  whalCtrl_SetTemplateWithCB (TI_HANDLE hWhalCtrl, whalCtrl_setTemplate_t* pTemplateParams,void *CBFunc,TI_HANDLE CBObj);
TemplateParams_T * whalCtrl_GetTemplate(TI_HANDLE hWhalCtrl, whalCtrl_templateType_e templateType);
int		whalCtrl_SetSlotTime		(TI_HANDLE hWhalCtrl, slotTime_e SlotTimeVal);
int 	whalCtrl_SetPreamble 	    (TI_HANDLE hWhalCtrl, preamble_e preambleVal);
int     whalCtrl_SetFrameRate       (TI_HANDLE hWhalCtrl, rate_e txFrmRate, BOOL bCtrlFrame);
int		whalCtrl_SetBeaconFiltering	 (TI_HANDLE hWhalCtrl, UINT8 beaconFilteringStatus, UINT8 numOfBeaconsToBuffer);
int		whalCtrl_GetBeaconFiltering	 (TI_HANDLE hWhalCtrl);

int		whalCtrl_SetBeaconFilterIETable(TI_HANDLE hWhalCtrl, UINT8* numberOfIEs, UINT8 * IETable, UINT8* IETableSize);
int		whalCtrl_GetBeaconFilterIETable(TI_HANDLE hWhalCtrl, UINT8* numberOfIEs, UINT8 * IETable, UINT8* IETableSize);
int		whalCtrl_SetarpIpAddressesTable(TI_HANDLE hWhalCtrl, IpAddress_t * IP_addr, UINT8 isEnabled , IPver_e IP_ver);
int		whalCtrl_SetarpIpFilterEnabled(TI_HANDLE hWhalCtrl,UINT8 isEnabled );
int		whalCtrl_SetGroupAddressesTable(TI_HANDLE hWhalCtrl, UINT8 numGroupAddrs, macAddress_t *Group_addr,UINT8 isEnabled);
int     whalCtrl_GetGroupAddressesTable(TI_HANDLE hWhalCtrl, UINT8* pisEnabled, UINT8* pnumGroupAddrs, macAddress_t *Group_addr);

int		whalCtrl_SetCwMin			 (TI_HANDLE hWhalCtrl, UINT8 CwMin);
int		whalCtrl_getSend4xWackInfo   (TI_HANDLE hWhalCtrl, UINT8 *Send4xWackInfo);
int		whalCtrl_setSend4xWackInfo   (TI_HANDLE hWhalCtrl, UINT8 Send4xWackInfo);
void	whalCtrl_SetBeaconInterval   (TI_HANDLE hWhalCtrl , UINT16 Val);
UINT16  whalCtrl_GetBeaconInterval   (TI_HANDLE hWhalCtrl);
void    whalCtrl_SetInfoElemEventMask(TI_HANDLE hWhalCtrl,UINT32 eventMask);

int		whalCtrl_getTsf				 (TI_HANDLE hwHalCtrl, UINT32 *pTsf);
int		whalCtrl_setDtimPeriod		 (TI_HANDLE hWhalCtrl, UINT8 dtimPeriod, UINT16 TBTT);
UINT8   whalCtrl_GetDtimCount        (TI_HANDLE hWhalCtrl);

TI_STATUS  whalCtrl_SetParam            (TI_HANDLE pWhalCtrl, whalParamInfo_t* pParamInfo);
TI_STATUS  whalCtrl_GetParam            (TI_HANDLE pWhalCtrl, whalParamInfo_t* pParamInfo);

int whalCtrl_ElpCtrl_SetMode(TI_HANDLE hWhalCtrl, elpCtrl_Mode_e mode);
int whalCtrl_setBetParams(TI_HANDLE hWhalCtrl, UINT8 Enable, UINT8 MaximumConsecutiveET);

/*
 *	Read/Write Reg API
 */
UINT32  whalCtrlReadMacReg		(TI_HANDLE hWhalCtrl, UINT32 addr);
void	whalCtrlWriteMacReg		(TI_HANDLE hWhalCtrl, UINT32 addr, UINT32	val);
UINT32  whalCtrlReadPhyReg		(TI_HANDLE hWhalCtrl, UINT32 addr);
void	whalCtrlWritePhyReg		(TI_HANDLE hWhalCtrl, UINT32 addr, UINT32	val);
int		whalCtrl_GetRadioStandByState(TI_HANDLE hWhalCtrl);
TI_STATUS  whalCtrl_GetFWInfo	(TI_HANDLE hWhalCtrl, whalCtrl_chip_t *pChip_Version);


/*
 *	Rx API
 */
int  whalCtrl_resetMacRx 		(TI_HANDLE hWhalCtrl);

/*
 *	Tx API
 */

int     whalCtrl_DisableTx			(TI_HANDLE hWhalCtrl);
int     whalCtrl_EnableTx			(TI_HANDLE hWhalCtrl);
UINT32  whalCtrl_GetTime			(TI_HANDLE hWhalCtrl);
void	whalCtrl_resetTxCounters	(TI_HANDLE hWhalCtrl);
void	whalCtrl_updateSecuritySeqNum(TI_HANDLE hWhalCtrl, UINT8 securitySeqNumLsByte);
/*
 *	Radio Handle API
 */
int  whalCtrl_SetRadioBand      (TI_HANDLE hWhalCtrl, radioBand_e RadioBand);
int  whalCtrl_SwitchChannel		(TI_HANDLE hWhalCtrl , UINT8 channel);

/*
 *	Recovery API
 */
int  whalCtrl_CheckMailboxCb	(TI_HANDLE hWhalCtrl,UINT16 MboxStatus,char *InterrogateParamsBuf);
int  whalCtrl_SetPacketDetectionThreshold (TI_HANDLE hWhalCtrl, UINT8 PDThreshold);
int  whalCtrl_SetEnergyDetection (TI_HANDLE hWhalCtrl, BOOL energyDetection);
int  whalCtrl_CheckHwStatus     (TI_HANDLE hWhalCtrl);
int  whalCtrl_InitHwStatus      (TI_HANDLE hWhalCtrl);
int  whalCtrl_PrintHwStatus     (TI_HANDLE hWhalCtrl);
int  whalCtrl_PeriodicCheckMailboxCb(TI_HANDLE hWhalCtrl,UINT16 MboxStatus,char *InterrogateParamsBuf);
void whalCtrl_ResetBusAfterHardBoot(TI_HANDLE hWhalCtrl);

/*
 *	Event Mail Box API
 */
int  whalCtrl_EventMbox_RegisterForEvent(TI_HANDLE hWhalCtrl, int EventBit, void *CbFunc, void *CbObj);
int  whalCtrl_EventMbox_Disable			(TI_HANDLE hWhalCtrl, int EventBit);
int  whalCtrl_EventMbox_Enable			(TI_HANDLE hWhalCtrl, int EventBit);

/*
 *	LNA API
 */
int  whalCtrl_LNAControl		(TI_HANDLE hWhalCtrl, UINT8 LNAControlField);

/*
 *	HW Interrupt API
 */
void whalCtrl_HandleBusTxn_Complete(TI_HANDLE hWhalCtrl);
int whalCtrl_HandleInterrupts	(TI_HANDLE pWhalCtrl);
void whalCtrl_EnableInterrupts	(TI_HANDLE pWhalCtrl);
void whalCtrl_DisableInterrupts (TI_HANDLE pWhalCtrl);
UINT32  whalCtrl_CheckInterrupts	(TI_HANDLE pWhalCtrl);
void  whalCtr_SlaveAckMaskNotification (TI_HANDLE hWhalCtrl);



/*
 *	Power Ctrl API
 */
TI_STATUS whalCtrl_powerMgmtConfig(TI_HANDLE theWhalCtrlHandle,
					whalCtrl_powerSaveParams_t* powerSaveParams);
/*                                   whalCtrl_powerMgmtConfig_t thePowerMgmtConfig);*/
TI_STATUS whalCtrl_wakeUpCondition(TI_HANDLE theWhalCtrlHandle,
                                   whalCtrl_powerMgmtConfig_t thePowerMgmtConfig);
TI_STATUS whalCtrl_PMConfig(TI_HANDLE theWhalCtrlHandle,
                            whalCtrl_powerMgmtConfig_t thePowerMgmtConfig);
TI_STATUS whalCtrl_BcnBrcOptions(TI_HANDLE theWhalCtrlHandle,
                            whalCtrl_powerMgmtConfig_t thePowerMgmtConfig);
TI_STATUS whalCtrl_SetMinPowerLevel(TI_HANDLE theWhalCtrlHandle,
									powerAutho_PowerPolicy_e minPowerPolicy);
TI_STATUS whalCtrl_GetMinPowerLevel(TI_HANDLE theWhalCtrlHandle,
									powerAutho_PowerPolicy_e* minPowerPolicy);


/*
 * Measurement API  	
 */
int whalCtrl_NoiseHistogramCmd  (TI_HANDLE hWhalCtrl, whalCtrl_noiseHistogram_t* pNoiseHistParams);
INT8 whalCtrl_convertRSSIToRxLevel(TI_HANDLE hWhalCtrl, INT32 rssiVal);
int whalCtrl_getMaxNumberOfCommandsInQueue (TI_HANDLE hWhalCtrl);
int whalCtrl_InterrogateGwsiStatisitics(TI_HANDLE hWhalCtrl, void * CB_Func, TI_HANDLE CB_handle, void * CB_Buf);
int whalCtrl_InterrogateGwsiCounters(TI_HANDLE hWhalCtrl, void * CB_Func, TI_HANDLE CB_handle, void * CB_Buf);
int whalCtrl_InterrogateMbox(TI_HANDLE hWhalCtrl , void *CB_Func, TI_HANDLE CB_handle, void *CB_Buf);
int  whalCtrl_JoinBss			(TI_HANDLE pWhalCtrl, whalCtrl_joinBss_t* pJoinBssParams);
int  whalCtrl_ReJoin			(TI_HANDLE hWhalCtrl);
BOOL whalCtrl_isCardIn			(TI_HANDLE hWhalCtrl);
int  whalCtrl_SendGenCmd		(TI_HANDLE hWhalCtrl, char* pBuf, UINT32 Length);
void whalCtrl_exitFromInitMode(TI_HANDLE hWhalCtrl);
void whalCtrl_exitFromInitModePart1(TI_HANDLE hWhalCtrl);
void whalCtrl_exitFromInitModePart2(TI_HANDLE hWhalCtrl);
int  whalCtrl_IsCardInstalled	(TI_HANDLE hWhalCtrl) ;

/* ----------------------------------------------------------------------------------
                         Scan Functions
   ----------------------------------------------------------------------------------*/
int	whalCtrl_StartScan       (TI_HANDLE hWhalCtrl, whalCtrl_scan_t* pScanVals, BOOLEAN bHighPriority , void* ScanCommandResponseCB, TI_HANDLE CB_handle);
int whalCtrl_StartSPSScan    (TI_HANDLE hWhalCtrl, whalCtrl_scan_t* pScanVals , void* ScanCommandResponseCB, TI_HANDLE CB_handle);
int whalCtrl_StopScan		 (TI_HANDLE pWhalCtrl ,void* ScanCommandResponseCB, TI_HANDLE CB_handle);
int whalCtrl_StopSPSScan     (TI_HANDLE hWhalCtrl , void* ScanCommandResponseCB, TI_HANDLE CB_handle);

TI_STATUS whalCtrl_SetSplitScanTimeOut (TI_HANDLE hWhalCtrl, UINT32 uTimeOut);

int whalCtrl_SwitchChannelCmd (TI_HANDLE hWhalCtrl, whalCtrl_switchChannelCmd_t* pSwitchChannelCmd);
int whalCtrl_SwitchChannelCancelCmd (TI_HANDLE hWhalCtrl, UINT8 channel);

/* ----------------------------------------------------------------------------------
                        Roaming Trigger  Functions
   ----------------------------------------------------------------------------------*/
int whalCtrl_SetSNRParamsCmd (TI_HANDLE hWhalCtrl, whalCtrl_roamingTriggerCmd_t* pRoamingTriggerCmd);
int whalCtrl_SetRSSIParamsCmd (TI_HANDLE hWhalCtrl, whalCtrl_roamingTriggerCmd_t* pRoamingTriggerCmd);
int whalCtrl_SetMaxTxRetryParamsCmd (TI_HANDLE hWhalCtrl, whalCtrl_roamingTriggerCmd_t* pRoamingTriggerCmd);
int whalCtrl_SetBssLossTsfThresholdParamsCmd (TI_HANDLE hWhalCtrl, whalCtrl_roamingTriggerCmd_t* pRoamingTriggerCmd);
int whalCtrl_GetAverageRSSI (TI_HANDLE hWhalCtrl, INT8* averageRSSI);
int whalCtrl_FwDisconnect(TI_HANDLE hWhalCtrl, UINT32 ConfigOptions, UINT32 FilterOptions);
int whalCtrl_GetAsynRSSI (TI_HANDLE hWhalCtrl,void *CB_Func, TI_HANDLE CB_handle, PUINT8 CB_Buf);

/* ----------------------------------------------------------------------------------
                        Measurement  Functions
   ----------------------------------------------------------------------------------*/
int whalCtrl_measurementCmd (TI_HANDLE hWhalCtrl, whalCtrl_MeasurementParameters_t* pMeasurementParams,
                             void* CommandResponseCB, TI_HANDLE CB_handle);
int whalCtrl_measurementStop (TI_HANDLE hWhalCtrl, void* CommandResponseCB, TI_HANDLE CB_handle);
int whalCtrl_ApDiscoveryCmd (TI_HANDLE hWhalCtrl, whalCtrl_ApDiscoveryParameters_t* pMeasurementParams);
int whalCtrl_ApDiscoveryStop (TI_HANDLE hWhalCtrl);

/* ----------------------------------------------------------------------------------
                        PLT  Functions
   ----------------------------------------------------------------------------------*/
int whalCtrl_RxPER(TI_HANDLE hWhalCtrl, PLT_RxPerCmd_e eRxPerCmd, TI_HANDLE CB_Handle, void *CB_Func);
int whalCtrl_TxCW(TI_HANDLE hWhalCtrl, TestCmdChannelBand_t* PltTxCarrier, void * CB_Func, TI_HANDLE CB_handle, void * CB_Buf);
int whalCtrl_TxContinues(TI_HANDLE hWhalCtrl, PltTxContinues_t* pPLT_TX_Continues, void * CB_Func, TI_HANDLE CB_handle, void * CB_Buf);
int whalCtrl_WriteRegister(TI_HANDLE hWhalCtrl, TI_HANDLE CB_Handle, void* CB_Func, void* CB_Buf);
int whalCtrl_ReadRegister(TI_HANDLE hWhalCtrl, TI_HANDLE CB_Handle, void* CB_Func, void* CB_Buf);
int whalCtrl_ReadMib(TI_HANDLE hWhalCtrl, TI_HANDLE CB_Handle, void* CB_Func, void* CB_Buf);
int whalCtrl_WriteMib(TI_HANDLE hWhalCtrl, PLT_MIB_t* pMib);

#endif /* _WHAL_CTRL_API_H*/
