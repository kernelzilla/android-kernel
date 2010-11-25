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

/*--------------------------------------------------------------------------*/
/* Module:		tiQosTypes.h*/
/**/
/* Purpose:		*/
/**/
/*--------------------------------------------------------------------------*/

#if !defined TI_QOS_TYPES_H
#define TI_QOS_TYPES_H


#define TI_WLAN_QOS_RETURN_CODES \
	NOT_CONNECTED,\
	TRAFIC_ADM_PENDING,\
	NO_QOS_AP,\
	ADM_CTRL_DISABLE,\
	AC_ALREADY_IN_USE,\
    USER_PRIORITY_NOT_ADMITTED\
/*
	NOT_CONNECTED       - Not connected to AP 
	TRAFIC_ADM_PENDING  - TSPEC request sent - awaiting response
	NO_QOS_AP           - primary AP does not support QOS (WME)
	ADM_CTRL_DISABLE    - Admission control disabled
	AC_ALREADY_IN_USE   - A TSPEC has already been sent on this specific AC
    USER_PRIORITY_NOT_ADMITTED - User priority is not admitted (TSPEC not sent)
*/


typedef enum{
    QOS_AC_BE = 0,
    QOS_AC_BK,
    QOS_AC_VI,
    QOS_AC_VO,
    QOS_HIGHEST_AC_INDEX = QOS_AC_VO,

}acTrfcType_e; /* for TI_HANDLE hAcTrfcCtrl */

#define MAX_NUM_OF_AC					( QOS_HIGHEST_AC_INDEX+1 )
#define FIRST_AC_INDEX					QOS_AC_BE
#define MAX_NUM_OF_802_1d_TAGS          8

#define AC_PARAMS_MAX_TSID              15
#define MAX_APSD_CONF                   0xffff


/*
 * this enum defines FW PS policing modes
 */
typedef struct _OS_802_11_QOS_PARAMS
{
    tiUINT32 acID;
    tiUINT32 MaxLifeTime;
    tiUINT32 VoiceDeliveryProtocol;
    tiUINT32 PSDeliveryProtocol;
} OS_802_11_QOS_PARAMS;

typedef struct  {
	tiUINT32 psPoll;
	tiUINT32 UPSD;
} OS_802_11_QOS_RX_TIMEOUT_PARAMS;

typedef struct _OS_802_11_AC_QOS_PARAMS
{
    	tiUINT32 uAC;
    	tiUINT32 uAssocAdmissionCtrlFlag;
		tiUINT32 uAIFS;
		tiUINT32 uCwMin;
		tiUINT32 uCwMax;
		tiUINT32 uTXOPLimit;
} OS_802_11_AC_QOS_PARAMS;

typedef struct _OS_802_11_AP_QOS_CAPABILITIES_PARAMS
{
    	tiUINT32 uQOSFlag;
    	tiUINT32 uAPSDFlag;
} OS_802_11_AP_QOS_CAPABILITIES_PARAMS;

typedef struct _OS_802_11_QOS_TSPEC_PARAMS
{
    tiUINT32 uUserPriority;
    tiUINT32 uNominalMSDUsize;	/* in bytes */
	tiUINT32 uMeanDataRate;		/* bits per second */
	tiUINT32 uMinimumPHYRate;	/* 1,2,5,6,9,11,12,18,......*/
	tiUINT32 uSurplusBandwidthAllowance;
    tiUINT32 uAPSDFlag;
	tiUINT32 uMediumTime;
    tiUINT32 uReasonCode;
} OS_802_11_QOS_TSPEC_PARAMS;

typedef struct _OS_802_11_QOS_DELETE_TSPEC_PARAMS
{
    tiUINT32 uUserPriority;
	tiUINT32 uReasonCode;
} OS_802_11_QOS_DELETE_TSPEC_PARAMS;

typedef struct _OS_802_11_QOS_DESIRED_PS_MODE
{
    tiUINT32 uDesiredPsMode;
	tiUINT32 uDesiredWmeAcPsMode[MAX_NUM_OF_AC];
} OS_802_11_QOS_DESIRED_PS_MODE;



/* When this value is added to reason code in TSPEC events, it indicates a TSPEC response which was unexpected at the time */
/* For example, a TSPEC response arrives after a TSPEC timeout */
#define TSPEC_RESPONSE_UNEXPECTED      0x1000   

typedef enum{
	ADDTS_RESPONSE_ACCEPT = 0,
/*	ADDTS_RESPONSE_REJECT,  - according to the standard*/
	ADDTS_RESPONSE_AP_PARAM_INVALID = 253,
	ADDTS_RESPONSE_TIMEOUT = 254,
	TSPEC_DELETED_BY_AP = 255,
} tspec_status_e;

typedef struct _OS_802_11_AC_UPSD_STATUS_PARAMS
{
   tiUINT32 uAC;
   tiUINT32 uCurrentUAPSDStatus;
   tiUINT32 pCurrentAdmissionStatus;
} OS_802_11_AC_UPSD_STATUS_PARAMS;

typedef struct _OS_802_11_THRESHOLD_CROSS_PARAMS
{
    tiUINT32 uAC;
    tiUINT32 uHighThreshold;
	tiUINT32 uLowThreshold;
} OS_802_11_THRESHOLD_CROSS_PARAMS;

typedef struct OS_802_11_THRESHOLD_CROSS_INDICATION_PARAMS
{
    tiUINT32 uAC;
	tiUINT32 uHighOrLowThresholdFlag;  /* According to thresholdCross_e enum */
    tiUINT32 uAboveOrBelowFlag;        /* According to thresholdCrossDirection_e enum */
} OS_802_11_THRESHOLD_CROSS_INDICATION_PARAMS;

typedef enum{
	HIGH_THRESHOLD_CROSS,
	LOW_THRESHOLD_CROSS,
} thresholdCross_e;

typedef enum{
	CROSS_ABOVE,
	CROSS_BELOW,
} thresholdCrossDirection_e;

typedef struct _STREAM_TRAFFIC_PROPERTIES{
   UINT32 dstIpAddress;
   UINT32 dstPort;
   UINT32 PktTag;
   UINT32 userPriority;
} STREAM_TRAFFIC_PROPERTIES;


typedef enum{
	AC_NOT_ADMITTED,
	AC_WAIT_ADMISSION,
	AC_ADMITTED
} trafficAdmState_e;

typedef enum{
   UPLINK_DIRECTION = 0,
   DOWNLINK_DIRECTION = 1,
   RESERVED_DIRECTION = 2,
   BI_DIRECTIONAL = 3,
} streamDirection_e;


/* classification algorithms: 
  0) D-tag to D-tag
  1) DSCP to D-tag
  2) Destination port number to D-tag 
  3) Destination IP&Port to D-tag
*/
typedef enum{
	D_TAG_CLSFR = 0,
	DSCP_CLSFR =1,
	PORT_CLSFR =2,
	IPPORT_CLSFR =3,
	CLSFR_TYPE_MAX = IPPORT_CLSFR,
} clsfr_type_e;

typedef struct clsfrTypeAndSupport
{
   ULONG        ClsfrType;
   UINT8        oldVersionSupport;
} clsfrTypeAndSupport;


/*************************/
/*   classifier params   */
/*************************/

/* This type represents a pair of 
destination IP address and destination port number. */
typedef struct 
{
	UINT32	DstIPAddress;
	UINT16	DstPortNum;
} IP_Port_t;

/* Classification mapping 
   table.
*/
typedef struct 
{
	union	
	{
		IP_Port_t	DstIPPort; /* for destination IP&Port classifier*/
		UINT16		DstPortNum; /* for destination Port classifier*/
		UINT8		CodePoint; /* for DSCP classifier*/
	}Dscp;
	UINT8		DTag; 
} clsfr_tableEntry_t;

/* Classifier parameters */

/* number of classifier entries in the classification table */
#define NUM_OF_CLSFR_TABLE_ENTRIES	16

typedef struct
{
	clsfr_type_e		clsfrType; /* The type of the classifier: D-tag, DSCP, Port or IP&Port */
	UINT8				NumOfActiveEntries; /* The number of active entries in the classification table */
	clsfr_tableEntry_t	
	ClsfrTable[NUM_OF_CLSFR_TABLE_ENTRIES]; /* Classification table - size changed from 15 to 16*/
} clsfr_Params_t;

/* This type is use by setParam to insert classifier table entries */
typedef struct
{
	UINT8	EntriesNum;
	UINT32	*BufferPtr; 
} clsfr_insertEntry_t;

#endif /* TI_QOS_TYPES_H */

