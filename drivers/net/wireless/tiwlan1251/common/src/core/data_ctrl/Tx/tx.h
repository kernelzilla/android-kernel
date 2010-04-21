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
/*																		   */
/*	  MODULE:	tx.h												       */
/*    PURPOSE:	Tx module Header file		 							   */
/*																		   */
/***************************************************************************/
#ifndef _TX_DATA_H_
#define _TX_DATA_H_


#include "MsduList.h"
#include "paramOut.h"
#include "DataCtrl_Api.h"

#define DEF_TX_PORT_STATUS					CLOSE

#define DEF_CURRENT_PRIVACY_INVOKED_MODE	FALSE

#define DEF_EAPOL_ENCRYPTION_STATUS			FALSE

#define DEF_SCHEDULER_THRESHOLD				32
#define DEF_IS_SCHEDULER_IN_WORK			FALSE
#define SCHEDULER_TIMER						1000
#define THROUGHPUT_TIMER                    1000

#define DEF_HAL_INTERFACE_STATUS			TX_DATA_OPEN
/*Ronnie: add GWSI IF status and the following defines*/
#define DEF_GWSI_INTERFACE_STATUS			GWSI_OPEN


#define MAX_NUMBER_OF_PRIORITIES	1

typedef struct 
{
	UINT32 	dbgDropedPacketsCounter;					            /* Pkts that failed to be inserted into Q */
    UINT32  dbgInsertToMsduListBytes[MAX_NUM_OF_TX_QUEUES];	        /* Bytes inserted into Q */
	UINT32  dbgInsertToMsduListPackets[MAX_NUM_OF_TX_QUEUES];		/* Pkts that inserted into Q */
	UINT32  dbgDropedFromMsduListPackets[MAX_NUM_OF_TX_QUEUES]; 	/* Pkts that failed to be inserted into Q */
	UINT32  dbgScheduledOutPackets[MAX_NUM_OF_TX_QUEUES];			/* Pkts scheduled for transmission per Q */
	UINT32  dbgSendToGwsiQosPackets[MAX_NUM_OF_TX_QUEUES];			/* Pkts sent to GWSI layer */
	UINT32  dbgDroppedDueExpiryTimePackets[MAX_NUM_OF_TX_QUEUES];	/* Pkts dropped due to expiry time */
	UINT32  dbgNumOfMsduFreeInTxTransfer[MAX_NUM_OF_TX_QUEUES];		/* Pkts freed on XFER */
	UINT32  dbgNumOfMsduTxTransferCB[MAX_NUM_OF_TX_QUEUES];			/* num of XFER done CB calls */
	UINT32	dbgNumOfMsduXferDoneInShceduler[MAX_NUM_OF_TX_QUEUES];	/* Pkts for which XFER done was received in scheduler */
	UINT32	dbgNumOfMsduSuccessInScheduler[MAX_NUM_OF_TX_QUEUES];	/* Pkts fof which success was received in scheduler */
	UINT32	dbgNumOfMsduPendingInScheduler[MAX_NUM_OF_TX_QUEUES];	/* Pkts for which pending was received in scheduler */
	UINT32	dbgNumOfMsduBusyInScheduler[MAX_NUM_OF_TX_QUEUES];		/* Pkts for which busy was received in scheduler */
	UINT32	dbgNumOfMsduErrorInScheduler[MAX_NUM_OF_TX_QUEUES];		/* Pkts for which error was received in scheduler */
	UINT32  dbgNumOfsendPacketComplete[MAX_NUM_OF_TX_QUEUES];		/* Pkts that reached complete CB */
	UINT32  dbgTxCmpltOk[MAX_NUM_OF_TX_QUEUES];                     /* Pkts that reached complete CB with status OK */
    UINT32  dbgTxCmpltError[MAX_NUM_OF_TX_QUEUES];                  /* Pkts that reached complete CB with status NOK */
    UINT32  dbgTxCmpltOkBytes[MAX_NUM_OF_TX_QUEUES];                /* Acknowledged bytes (complete status OK) */
	UINT32  dbgNumOfNullMsdu;										/* number of NULL MSDUs altogether */
} txDataDbgCounters_t;


typedef  struct 
{
    struct 
    {
        UINT32  core;            /* Cumulative core jitter */ 
        UINT32  xfer;            /* Cumulative XFER jitter */ 
        UINT32  fw;              /* Cumulative firmware jitter */
        UINT32  air;             /* Cumulative air jitter */
    } jitter;

    struct 
    {
        UINT32  core;            /* Cumulative core delay */ 
        UINT32  xfer;            /* Cumulative XFER delay */
        UINT32  wait;            /* Cumulative XFER wait for firmware to wake-up */
        UINT32  fw;              /* Cumulative firmware delay */
        UINT32  air;             /* Cumulative air delay */ 
    } delay;

    struct 
    {
        UINT32  core;            /* Last core delay */
        UINT32  xfer;            /* Last xfer delay */
        UINT32  fw;              /* Last firmware duration */
        UINT32  air;             /* Last TX air duration */
    } last_delay;

    struct 
    {
        UINT32  core;            /* Maximum core delay */
        UINT32  xfer;            /* Maximum xfer delay */
        UINT32  fw;              /* Maximum firmware duration */
        UINT32  air;             /* Maximum TX air duration */
    } max_delay;

    struct
    {
        UINT32  core;            /* Number of packets passed through core */
        UINT32  xfer;            /* Number of packets transferred via XFER during last second */
        UINT32  wait;            /* Number of times XFER waited for firmware to wake-up */
        UINT32  fw;              /* Packets that reached complete CB with status OK */
        UINT32  fw_err;          /* Packets that reached complete CB with status NOK */
    } count;
 
} txDataJitter_t;

typedef struct
{
    headerConvetMode_e  headerConverMode;                           /* header converting mode        */
    acTrfcType_e        tag_ToAcClsfrTable[MAX_NUM_OF_802_1d_TAGS]; /* tag to AC classification      */
} txQosParams_t;

typedef struct 
{
	/* Handles */
	TI_HANDLE			hCtrlData;
	TI_HANDLE 			hTnetwDrv;
	TI_HANDLE 			hWhalCtrl;
	TI_HANDLE			hOs;
	TI_HANDLE			hReport;
	TI_HANDLE			hMemMngr;
	TI_HANDLE			pSchedulerTimer;
	TI_HANDLE			pCreditTimer;
	TI_HANDLE			hSiteMgr;
    TI_HANDLE			hEvHandler;
	TI_HANDLE			hQosMngr;
	TI_HANDLE			hPowerMgr;
	TI_HANDLE           pThroughputTimer;
	TI_HANDLE           pAirThroughputTimer;
	TI_HANDLE           pJitterTimer;
    TI_HANDLE           TxEventDistributor;
    TI_HANDLE           hBufferPool;                                /* Packet ID buffer pool */
    TI_HANDLE			pVadTimer;			/* VAD timer handle */
	/* critical section protect */
	void*				hCriticalSectionProtect;

    BOOL                bSchedulerTimerRunning;

	/* Tx flow parameters */
	portStatus_e  		txDataPortStatus;
    portStatus_e  		savePortStatus;                     /* save value for recovery */
	BOOL				txDataCurrentPrivacyInvokedMode;
	BOOL                saveTxDataCurrentPrivacyInvokedMode;/* save value for recovery */
	BOOL 				txDataEapolEncryptionStatus;
	BOOL                saveTxDataEapolEncryptionStatus;    /* save value for recovery */
	BOOL				txDataIsSchedulerInWork;

	/* Tx queues */
	UINT8				txDataNumOfQueues;
	MsduList_t*   		mngMsduList; 
	MsduList_t*   		dataMsduListArr[MAX_NUM_OF_TX_QUEUES];  
    int                 txDataNumOfMsdusToTransmit; 

	UINT8				QidToAcTable[MAX_NUM_OF_TX_QUEUES];

	UINT32				creditCalculationTimeout;
    
    /* indicate the percentage of the MSDU lifetime under which the driver   *
     * will send it to the Fw,                                               */
    UINT8               uFracOfLifeTimeToDrop;
	/* GWSI state */
	txDataGwsiInterfaceStatus_e	txDataGwsiInterfaceStatus;

	txDataHalInterfaceStatus_t	txDataHalInterfaceStatus;


	/* Counters */
	txDataCounters_t	txDataCounters[MAX_NUM_OF_TX_QUEUES]; /* Save Tx statistics per Tx-queue. */
	txDataCounters_t	txDataReportedCounters[MAX_NUM_OF_TX_QUEUES]; /* Tx statistics per Tx-queue, cleared on read */
    txDataCounters_t	tempTxDataCounters[MAX_NUM_OF_TX_QUEUES]; /* temporary storage for IOCTL retrieval. */
	UINT32				currentConsecutiveRetryFail; /* current consecutive number of tx failures due to max retry */

	/* debug counters */
	txDataDbgCounters_t	txDataDbgCounters;

    /* Jitter debug info */
  #if defined(TI_DBG)
    txDataJitter_t      txJitter [MAX_NUM_OF_TX_QUEUES];
  #endif

	/* Tx Disabling flag */
	txDisableReason_e	txDisable;

	/*  QOS parameters */
	acTrfcCtrl_t        txDataAcTrfcCtrl[MAX_NUM_OF_AC];
    txQosParams_t       txDataQosParams;

	/* timer throughput per tx queue */
	BOOL                txThroughputTimerEnable;

    /* timer air throughput per tx queue */
    BOOL                txAirThroughputTimerEnable;

    /* jitter periodic timer flag */
    BOOL                txJitterTimerEnable;

	/* credit calculation timer is enabled from registry */
	BOOL				bCreditCalcTimerEnabled;
	/* credit calculation timer is running */
	BOOL				bCreditCalcTimerRunning;

	/* enable to delay MSDU because of medium usage exeeded */
	BOOL				admCtrlDelayDueToMediumTimeOverUsage;

	/* enable/disable adm down grade */
	BOOL				admissionDownGradeEnable;
	
	/* power control params and flag */
    BOOL				hwRequest;             /* HW request flag */
	UINT32              powerCtrlId;           /* txDta Id  for power control */

	BOOL                txDataAvailableQueue[MAX_NUM_OF_TX_QUEUES];

    /* encryption params */
    UINT8               encryptionFieldSize;    /* size to reserve in WLAN header fpr encryption */
	UINT8               saveEncryptionFieldSize;/* save value for recovery */

	BOOL				bVadTimerEnabled;
	UINT16				vadTimerDuration;		/* in milliseconds */
} txData_t;

/* test functions */
/*----------------*/
void Test_HeaderConvertion(TI_HANDLE hTxData, mem_MSDU_T *pMsdu);

#endif
