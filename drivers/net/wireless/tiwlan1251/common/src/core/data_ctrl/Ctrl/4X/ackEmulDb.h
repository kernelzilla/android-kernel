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

/***************************************************************************/
/*                                                                         */
/*   MODULE:	ackEmulDb.h                                                */
/*   PURPOSE:	Ack emulation database                                     */
/*                                                                         */
/***************************************************************************/
#ifndef _ACK_EMULATION_DB_H_
#define _ACK_EMULATION_DB_H_

#include "osTIType.h"

#define ACK_EMUL_MODULE_LOG		CTRL_DATA_MODULE_LOG
void ackEGetPktType(unsigned char* pPacket , UINT16 *packetInclude, UINT16 *tcpDataSize);

#define TCP_LOOKAHEAD_SIZE 64
#define MAX_ACK_TEMPLATE_SIZE 100
#define MAX_AE_STATIONS 1


#define IP_PROTOCOL_NUMBER          0x0800
#define TCP_PROTOCOL                0x06
#define IP_TOTAL_LEN_FIELD          2
#define IP_IDENTIFIER_FIELD         4

#define IP_PROTOCOL_FIELD           9
#define IP_CHECKSUM_FIELD           10
#define DEST_PORT_FIELD             2
#define IP_SRC_ADDRESS_FIELD		   12
#define IP_DEST_ADDRESS_FIELD		   16


#define TCP_SEQUENCE_NUMBER_FIELD	4
#define TCP_ACK_NUMBER_FIELD        8
#define TCP_OFFSET_FIELD	         12
#define TCP_CHECKSUM_FIELD	         16

#define WTARGET_ACTIVE_TIME_OUT 500000 /* was 500 */
#define WTARGET_TERMINATE_TIME_OUT 1000
#define WSOURCE_SESSION_TIME_OUT 700

#define MAX_ACIVE_SESSION	2
#define MAX_STANDBY_SESSION 8

#define INDEX_FREE 0
#define INDEX_BUSY 1


typedef struct {
  int	lastUsedIndex;
  int	currentActiveState;
  int	currentStandbyState;
} sessionsTableManager_T;

typedef struct {
	int status;			/* INDEX_FREE or INDEX_BUSY */
	int monitorIndex ; /* Index in wdrv_aeWTargetTable */
}activeIndexTable_T;



typedef struct {  
   UINT8 data[MAX_ACK_TEMPLATE_SIZE];
   UINT8 ipHeaderLen;
   UINT8 tcpHeaderLen;
} ackTemplate_T;

typedef struct {  
   UINT16	sPorts;                              /* Include source port  */
   UINT16	dPorts;                              /* Include destination port */
   UINT32	srcIPAddr;                          /* source ip address */
   UINT32	destIPAddr;                         /* destination ip address */
   UINT32   segmentSize;                        /* Stable segment size */
   UINT32   sequenceNumber;	                  /* The data sequence number */
   UINT32   ackNumber;	                        /* Specifies the next byte expected.*/
   UINT32   timeStamp;                          /*  */
   UINT32   ackCounter;	                        /* The number of 2 * Segement_size in ACK number  */
   ackTemplate_T    ackTemplate;			      	/* Ack template block */ 
   UINT8    monitorState;                       /* current monitor state */
   UINT8    equalSegmentSizeCounter;            /* count the number of equal segment size. */
   UINT8    yTagFlag;                           /* True if send Y tag */
   UINT8    activeIndex;                        /* Active session index */
   UINT16    sourceStationIndex;                 /* Source station index */
 
} ackEmulationWTargetDbTyple_T;


typedef struct {  
   UINT32   segmentSize;                        /* Stable segment size */
   UINT32   ackNumber;	                        /* Specifies the next byte expected.*/
   UINT32   ackCounter;	                        /* The number of 2 * Segement_size in ACK number  */
   UINT32   timeStamp;	                        /* The number of 2 * Segement_size in ACK number  */
   UINT32   ackReorderProblem;                
   ackTemplate_T    ackTemplate;			      	/* Ack template block */ 
}ackEmulationWSourceDbTyple_T;


/* Ack emulation monitor states */
typedef enum {
	REORDER_PROBLEM_OFF =0,
	REORDER_PROBLEM_ON,
	REORDER_PROBLEM_PRE,
} wdrv_eaAckReorderProblem_T;


typedef enum {
	AE_INACTIVE =0,
	AE_STANDBY,
	AE_CANDIDATE_ACTIVE,
	AE_ACTIVE,
	AE_TERMINATE,
} wdrv_eaMonitorStatus_T;


typedef struct 
{
	TI_HANDLE	hOs;
	TI_HANDLE	hReport;


	/* This table store the WTarget monitor session */ 
	ackEmulationWTargetDbTyple_T wdrv_aeWTargetTable[MAX_ACIVE_SESSION+MAX_STANDBY_SESSION];
	sessionsTableManager_T sessionsTableManager;

	/* This table store the WSource monitor session */ 
	ackEmulationWSourceDbTyple_T wdrv_aeWSourceTable[MAX_AE_STATIONS][MAX_ACIVE_SESSION];

	/* This table store the Xtag status */ 
	UINT8 ackEmulationXTagTable[MAX_AE_STATIONS];

	/* This table store the active monitor session index */ 
	activeIndexTable_T activeIndexTable[MAX_ACIVE_SESSION];
			
}ackEmulDB_t;


ackEmulDB_t* ackEmulDb_create(TI_HANDLE hOs);

TI_STATUS ackEmulDb_config(ackEmulDB_t*	ackEmulDB,
						   TI_HANDLE	hOs,
						   TI_HANDLE	hReport);

TI_STATUS ackEmulDb_destroy(ackEmulDB_t*	ackEmulDB);

void wdrv_aeDbInit(ackEmulDB_t*	ackEmulDB);
void wdrv_aeWTargetDbPrint(ackEmulDB_t*	ackEmulDB);
int wdrv_aeWTargetDbFindDataSession(ackEmulDB_t*	ackEmulDB,UINT8 *pktBuf,UINT8 *sessionIndex, UINT8 *monitorState);
int wdrv_aeWTargetDbFindAckSession(ackEmulDB_t*	ackEmulDB,UINT8 *pktBuf,UINT8 *sessionIndex, UINT8 *monitorState);
int wdrv_aeWTargetDbAddSession(ackEmulDB_t*	ackEmulDB,UINT8 *pktBuf);

int wdrv_aeWTargetDbDelSession(ackEmulDB_t*	ackEmulDB,UINT8 SessionIndex);
int wdrv_aeWTargetDbSetActiveState(ackEmulDB_t*	ackEmulDB,UINT8 index , UINT8 *activeIndex);
void wdrv_aeWTargetDbSaveAckTemplate(ackEmulDB_t*	ackEmulDB,UINT8 index, UINT8 *pIpHeader);
void wdrv_aeWTargetDbUpdateAckTemplate(ackEmulDB_t*	ackEmulDB,UINT8 index, UINT32 sequenceNumber);
int wdrv_aeWTargetDbCmpAckTemplate(ackEmulDB_t*	ackEmulDB,UINT8 index, UINT8 *pIpHeader);
void wdrv_aeWTargetDbResetTuple(ackEmulDB_t*	ackEmulDB,int index);

void wdrv_aeWTargetDbGetSessionSequenceNumber(ackEmulDB_t*	ackEmulDB,UINT8 index, UINT32 *sequenceNumber);
void wdrv_aeWTargetDbGetSessionAckNumber(ackEmulDB_t*	ackEmulDB,UINT8 index, UINT32 *ackNumber);
void wdrv_aeWTargetDbGetSessionSegmentSize(ackEmulDB_t*	ackEmulDB,UINT8 index, UINT32 *segmentSize);
void wdrv_aeWTargetDbGetIncrSessionEqualSegmentSizeCounter(ackEmulDB_t*	ackEmulDB,UINT8 index, UINT8 *equalSegmentSizeCounter);
void wdrv_aeWTargetDbGetSessionAckCounter(ackEmulDB_t*	ackEmulDB,UINT8 index, UINT32 *ackCounter);
void wdrv_aeWTargetDbGetSessionMmonitorState(ackEmulDB_t*	ackEmulDB,UINT8 index, UINT8 *monitorState);
void wdrv_aeWTargetDbGetSessionActiveIndex(ackEmulDB_t*	ackEmulDB,UINT8 index, UINT8 *activeIndex);

void wdrv_aeWTargetDbSetSessionSequenceNumber(ackEmulDB_t*	ackEmulDB,UINT8 index, UINT32 sequenceNumber);
void wdrv_aeWTargetDbSetSessionAckNumber(ackEmulDB_t*	ackEmulDB,UINT8 index, UINT32 ackNumber);
void wdrv_aeWTargetDbSetSessionSegmentSize(ackEmulDB_t*	ackEmulDB,UINT8 index, UINT32 segmentSize);
void wdrv_aeWTargetDbSetSessionAckCounter(ackEmulDB_t*	ackEmulDB,UINT8 index, UINT32 ackCounter);
void wdrv_aeWTargetDbSetSessionMonitorState(ackEmulDB_t*	ackEmulDB,UINT8 index, UINT8 monitorState);
void wdrv_aeWTargetDbSetSessionEqualSegmentSizeCounter(ackEmulDB_t*	ackEmulDB,UINT8 index, UINT8 equalSegmentSizeCounter);
void wdrv_aeWTargetDbGetSessionTimeStamp(ackEmulDB_t*	ackEmulDB,UINT8 index, UINT32 *timeStamp);
void wdrv_aeWTargetDbSetSessionTimeStamp(ackEmulDB_t*	ackEmulDB,UINT8 index, UINT32 timeStamp);

void wdrv_aeWSourceDbUpdateTemplate(ackEmulDB_t*	ackEmulDB,UINT8 *pktBuf,UINT8 stationIndex,UINT8 *sessionIndex);
void wdrv_aeWSourceDbResetSession(ackEmulDB_t*	ackEmulDB,int stationIndex,int activeIndex);

void wdrv_aeWSourceSaveAckTemplate(ackEmulDB_t*	ackEmulDB,UINT8 stationIndex,UINT8 activeIndex,
											  UINT8* pDot11Header, UINT8 *pWlanSnapHeader, UINT8 *pIpHeader 
												,UINT16 dataLen,UINT16 segmentSize);

void wdrv_aeWSourceDbGetSessionAckCounter(ackEmulDB_t*	ackEmulDB,UINT16 stationIndex, UINT8 activeIndex, UINT32 *ackCounter);

void wdrv_aeWSourceDbSetSessionAckCounter(ackEmulDB_t*	ackEmulDB,UINT16 stationIndex, UINT8 activeIndex, UINT32 ackCounter);

void wdrv_aeWSourceDbGetSessionAckNumber(ackEmulDB_t*	ackEmulDB,UINT16 stationIndex, UINT8 activeIndex, UINT32 *ackNumber);
void wdrv_aeWSourceDbSetSessionAckNumber(ackEmulDB_t*	ackEmulDB,UINT16 stationIndex, UINT8 activeIndex, UINT32 ackNumber);

void wdrv_aeWSourceDbGetSessionSegmentSize(ackEmulDB_t*	ackEmulDB,UINT16 stationIndex, UINT8 activeIndex, UINT32 *segmentSize);
void wdrv_aeWSourceDbSetSessionSegmentSize(ackEmulDB_t*	ackEmulDB,UINT16 stationIndex, UINT8 activeIndex, UINT32 segmentSize);

void wdrv_aeWSourceDbGetSessionTimeStamp(ackEmulDB_t*	ackEmulDB,UINT16 stationIndex, UINT8 activeIndex, UINT32 *timeStamp);
void wdrv_aeWSourceDbSetSessionTimeStamp(ackEmulDB_t*	ackEmulDB,UINT16 stationIndex, UINT8 activeIndex, UINT32 timeStamp);

void wdrv_aeWSourceDbGetSessionAckReorderProblem(ackEmulDB_t*	ackEmulDB,UINT16 stationIndex, UINT8 activeIndex, UINT32 *ackReorderProblem);
void wdrv_aeWSourceDbSetSessionAckReorderProblem(ackEmulDB_t*	ackEmulDB,UINT16 stationIndex, UINT8 activeIndex, UINT32 ackReorderProblem);


void wdrv_aeWSourceDbGetAckTemplate(ackEmulDB_t*	ackEmulDB,UINT16 stationIndex, UINT8 activeIndex, UINT8 **pTeplate,
													 UINT8 *ipHeaderLen, UINT8 *tcpHeaderLen);


void wdrv_aeDbGetXTagStatus(ackEmulDB_t*	ackEmulDB,UINT8 sessionIndex, UINT8 *status);
void wdrv_aeDbSetXTagStatus(ackEmulDB_t*	ackEmulDB,UINT8 sessionIndex, UINT8 status);

#endif
