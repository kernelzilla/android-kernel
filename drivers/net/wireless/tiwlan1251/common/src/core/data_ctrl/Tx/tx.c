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
/*      MODULE: Tx.c                                                       */
/*    PURPOSE:  Tx module functions                                        */
/*                                                                         */
/***************************************************************************/
#include "osTIType.h"
#include "paramIn.h"
#include "paramOut.h"
#include "osApi.h"
#include "tx.h"
#include "DataCtrl_Api.h"
#include "siteMgrApi.h"
#include "802_11Defs.h"
#include "Ethernet.h"
/* GWSI_LAYER */
#include "Core_AdaptTx.h"

#include "memMngrEx.h"
#include "report.h"
#include "utils.h"
#include "measurementMgrApi.h"
#include "TI_IPC_Api.h"
#include "EvHandler.h"
#include "qosMngr_API.h"
#include "PowerMgr_API.h"
#include "bufferPoolApi.h"
#include "TNETW_Driver_api.h"

/*
This is an Ethernet Version 2 frame:

       +--------------+
       |              | The destination address is a six byte Media Access
       | Destination  | Control (MAC) address, usually burned into the
       |   6 bytes    | ROM of the Ethernet card.
       +--------------+
       |              | The source address is a six byte MAC address, and
       |   Source     | can signify a physical station or a broadcast.
       |   6 bytes    |
       +--------------+
       |     Type     | The Type field it must be grater then 1500 dec.
       |    2 bytes   |
       +--------------+
       |              |  Any higher layer information is placed in the
       |    Data      |  data field, which could contain protocol
       |              |  information or user data.
       ~              ~
       ~              ~
       |  46 to 1500  |
       |    bytes     |
       |              |
       +--------------+
       |     FCS      |
       |   4 bytes    |
       +--------------+



*/
/*

        802.2 SNAP DATA Frame
       +----------------+
       |                |
       |  Destination   |
       |    6 bytes     |
       +----------------+
       |                |
       |     Source     |
       |    6 bytes     |
       +----------------+
       |  Frame Length  | Must be <= 1500 Dec.
       |    2 bytes     |
       +----------------+
       |  DSAP - 1 byte | = 0xAA ( SNAP )
       +----------------+
       |  SSAP - 1 byte | = 0xAA ( SNAP )
       +----------------+
       |Control - 1 byte| = 0x03
       +----------------+
       | OUI - 3 bytes  | = 0x0
       |                |
       +----------------+
       | Type - 2 bytes |  = Ethernet type (IP=0x0800)
       +----------------+
       |                |
       |      Data      |
       |                |
       ~                ~
       ~                ~
       |   46 to 1500   |
       |     bytes      |
       |                |
       +----------------+
       |      FCS       |
       |    4 bytes     |
       +----------------+




        802.11 DATA Frame
       +----------------+
       |                |
       |  Frame Control |
       |    2 bytes     |
       +----------------+
       |                |
       |  Duration ID   |
       |    2 bytes     |
       +----------------+
       |                |
       |   Address 1    |
       |    6 bytes     |
       +----------------+
       |                |
       |   Address 2    |
       |    6 bytes     |
       +----------------+
       |                |
       |   Address 3    |
       |    6 bytes     |
       +----------------+
       |                |
       | Sequence Cntrl |
       |    2 bytes     |
       +----------------+
       |                |
       |   Address 4    |
       |    6 bytes     |
       +----------------+
       |  DSAP - 1 byte | = 0xAA ( SNAP )
       +----------------+
       |  SSAP - 1 byte | = 0xAA ( SNAP )
       +----------------+
       |Control - 1 byte| = 0x03
       +----------------+
       | OUI - 3 bytes  | = 0x0
       |                |
       +----------------+
       | Type - 2 bytes |  = Ethernet type (IP=0x0800)
       +----------------+
       |                |
       |      Data      |
       |                |
       ~                ~
       ~                ~
       |   46 to 1500   |
       |     bytes      |
       |                |
       +----------------+
       |      FCS       |
       |    4 bytes     |
       +----------------+

*/

/* Tx queue selection definitions */    
#define Q_LEAST_WEIGHT              0xFFFFFFFF /* Use least possible weight as init value (highest value is lowest priority). */
#define Q_SELECTION_HISTORY_LEVEL   5   /* Count down from this value if queue is selected to Tx. */


#define MANAGEMENT_QUEUE_SIZE           16

#define DEFAULT_QUEUE_TO_HAL            1

#define EAPOL_PACKET                    0x8E88

/* defined in QosMngr.c - used to update QosControl (userPriority) of a given packet after it has been "downgraded" due to admission control */
extern UINT8 wmeAcToUpIndex[MAX_NUM_OF_AC];

/* defined in qosMngr.c - used to identify voice packets in NON QOS APs */
extern int WMEQosTagToACTable[MAX_NUM_OF_802_1d_TAGS];

/* this macro accesses the WME Tag-to-AC conversion array in order to enable identifying voice packets even on NON QOS APs */
#define GET_WME_AC_TYPE_FROM_MSDU(pMsdu)  (WMEQosTagToACTable[pMsdu->qosTag])

#define GET_QUEUE_INDEX(pTxData,acIndex)  (pTxData->txDataAcTrfcCtrl[acIndex].QueueIndex)

#define ABS(a)  (((int)(a) >= 0) ? (a) : -((int)(a)))

static void txData_convertEthToWlanHeader (txData_t *pTxData, mem_MSDU_T *pMsdu);

static TI_STATUS txData_schedulerSelectQueueToTransmitFrom( TI_HANDLE hTxData, MsduList_t** pMsduListPtr,UINT8 *selectedQueueIndex );

static void txData_startTxSchedulerFromTimer(TI_HANDLE hTxData);

static void txData_calcCreditFromTimer(TI_HANDLE hTxData);

static void txData_UpdateTxCounters( TI_HANDLE hTxData, txCompleteAttr_t *pTxCompleteAttr );

static void txData_SetTxDelayCounters( TI_HANDLE hTxData, UINT32 txQid, txCompleteAttr_t *pTxCompleteAttr, UINT32 driverDelay );

static int txData_selectQueueAndUpdateUserPriority (txData_t *pTxData, mem_MSDU_T *pMsdu, int *selectedQueue, acTrfcType_e *selectedAc);

static int txData_getHighestAdmittedAc(txData_t *pTxData, int startingAcIndex);

static void txData_startVadTimer(TI_HANDLE hTxData, UINT16 voiceDuration);
static void txData_stopVadTimer(TI_HANDLE hTxData);
static void txData_setVadTimer(TI_HANDLE hTxData, BOOL vadEnabled, UINT16 duration);
static void txData_resetVadTimer(TI_HANDLE hTxData);
static void txData_vadTimeout(TI_HANDLE hTxData);
#ifdef TI_DBG

static void txData_printTxThroughputPerQueue(TI_HANDLE hTxData);

static void txData_printTxAirThroughputPerQueue(TI_HANDLE hTxData);

static void txData_printJitter(TI_HANDLE hTxData);

#endif

static BOOL txData_acVoPsPollMode(txData_t *pTxData);

static UINT32 txDataTimeToMsduExpiry( TI_HANDLE htxData, mem_MSDU_T* pMsdu, UINT8 Qid);

static UINT8 txData_GetAcIdFromQid(TI_HANDLE hTxData,UINT8 Qid);

TI_STATUS txData_setMediumUsageThresholds(TI_HANDLE     hTxData,
                                      UINT8             acID,
                                      INT32             highMediumUsageThreshold,
                                      INT32             lowMediumUsageThreshold);

static void txData_SetQidToAcTable(TI_HANDLE hTxData,UINT8 QidStart, UINT8 QidEnd,UINT8 AcId);

/* The TX delay histogram ranges start and end in msec. */
static UINT32 txDelayRangeStart[TX_DELAY_RANGES_NUM] = { 0,  1, 10, 20, 40, 60,  80, 100, 200 };
static UINT32 txDelayRangeEnd  [TX_DELAY_RANGES_NUM] = { 1, 10, 20, 40, 60, 80, 100, 200, 0xFFFFFFFF };

/*************************************************************************
*                        txData_create                                   *
**************************************************************************
* DESCRIPTION:  This function initializes the Tx module.
*
* INPUT:        hOs - handle to Os Abstraction Layer
*               txDataInitParams - Tx Data creation parameters
* OUTPUT:
*
* RETURN:       Handle to the allocated Tx data control block
*************************************************************************/
TI_HANDLE txData_create(txDataInitParams_t *txDataInitParams,
                        TI_HANDLE hOs)
{

    txData_t *hTxData;
    MsduList_t *mgmtMsduList;
    MsduList_t *dataMsduListArr[MAX_NUM_OF_TX_QUEUES];
    void* pTimer;
  #ifdef TI_DBG
    void* pThroughputTimer;
    void* pAirThroughputTimer;
    void* pJitterTimer;
  #endif
    void* pCreditTimer;
    void* pVadTimer;
    int queueIndex = 0;


    /* check parameters validity */
    if( txDataInitParams->txDataNumOfDataQueues > MAX_NUM_OF_TX_QUEUES ||
        txDataInitParams->txDataNumOfDataQueues <= 0 )
    {
        WLAN_OS_REPORT(("FATAL ERROR - UNABLE TO CREATE TX MODULE. Number of queues error = %d.",txDataInitParams->txDataNumOfDataQueues));
        return NULL;
    }

    /* allocate Tx module control block */
    hTxData = os_memoryAlloc(hOs, (sizeof(txData_t)));

    if(!hTxData)
        return NULL;

    /* reset tx control block */
    os_memoryZero(hOs, hTxData, (sizeof(txData_t)));

    /* allocate the buffer pool */
    hTxData->hBufferPool = bufferPool_create( hOs, 
                                              HAL_CTRL_ACX_TX_DESC_DEF * MAX_NUM_OF_TX_QUEUES, 
                                              sizeof(txPacketIdAttr_t) );
    if ( NULL == hTxData->hBufferPool )
    {
        utils_nullMemoryFree(hOs, hTxData, sizeof(txData_t));
        return NULL;
    }

    /* allocate Timer to use for scheduler trigger */
    pTimer = os_timerCreate(hOs, txData_startTxSchedulerFromTimer, hTxData);
    if(!pTimer)
    {
        bufferPool_destroy( hTxData->hBufferPool );
        utils_nullMemoryFree(hOs, hTxData, sizeof(txData_t));
        return NULL;
    }

    /* allocate Timer for vad operation */
    pVadTimer = os_timerCreate(hOs, txData_vadTimeout, hTxData);
    if(!pVadTimer)
    {
        bufferPool_destroy( hTxData->hBufferPool );
        utils_nullTimerDestroy (hOs, pTimer);        
        utils_nullMemoryFree(hOs, hTxData, sizeof(txData_t));
        return NULL;
    }
  #ifdef TI_DBG
    /* allocate timer for debug throughput per queue */
    pThroughputTimer = os_timerCreate(hOs, txData_printTxThroughputPerQueue, hTxData);
    if(!pThroughputTimer)
    {
        bufferPool_destroy( hTxData->hBufferPool );
        utils_nullTimerDestroy(hOs, pTimer);
        utils_nullMemoryFree(hOs, hTxData, sizeof(txData_t));
        return NULL;
    }

    /* allocate timer for debug throughput per queue */
    pAirThroughputTimer = os_timerCreate (hOs, txData_printTxAirThroughputPerQueue, hTxData);
    if (!pAirThroughputTimer)
    {
        bufferPool_destroy (hTxData->hBufferPool);
        utils_nullTimerDestroy (hOs, pTimer);
        utils_nullTimerDestroy (hOs, pThroughputTimer);
        utils_nullMemoryFree (hOs, hTxData, sizeof(txData_t));
        return NULL;
    }

    /* allocate timer for debug throughput per queue */
    pJitterTimer = os_timerCreate (hOs, txData_printJitter, hTxData);
    if (!pJitterTimer)
    {
        bufferPool_destroy (hTxData->hBufferPool);
        utils_nullTimerDestroy (hOs, pTimer);
        utils_nullTimerDestroy (hOs, pThroughputTimer);
        utils_nullTimerDestroy (hOs, pAirThroughputTimer);
        utils_nullMemoryFree (hOs, hTxData, sizeof(txData_t));
        return NULL;
    }
  #endif

    /* allocate timer for credit calculation */
    pCreditTimer = os_timerCreate(hOs, txData_calcCreditFromTimer, hTxData);
    if(!pCreditTimer)
    {
        bufferPool_destroy( hTxData->hBufferPool );
        utils_nullTimerDestroy(hOs, pTimer);
      #ifdef TI_DBG
        utils_nullTimerDestroy(hOs, pThroughputTimer);
        utils_nullTimerDestroy(hOs, pAirThroughputTimer);
        utils_nullTimerDestroy(hOs, pJitterTimer);
      #endif
        utils_nullMemoryFree(hOs, hTxData, sizeof(txData_t));
        return NULL;
    }

    /* initialize management queue */
    mgmtMsduList = msduList_CreateNewMsduList( hOs);
    if(!mgmtMsduList)
    {
        bufferPool_destroy( hTxData->hBufferPool );
        utils_nullTimerDestroy(hOs, pTimer);
      #ifdef TI_DBG
        utils_nullTimerDestroy(hOs, pThroughputTimer);
        utils_nullTimerDestroy(hOs, pAirThroughputTimer);
        utils_nullTimerDestroy(hOs, pJitterTimer);
      #endif
        utils_nullTimerDestroy(hOs, pCreditTimer);
        utils_nullMemoryFree(hOs, hTxData, sizeof(txData_t));
        return NULL;
    }

    /* initialize tx data queues */
    for(queueIndex = 0 ; queueIndex < MAX_NUM_OF_TX_QUEUES ; queueIndex++)
    {
        dataMsduListArr[queueIndex] = msduList_CreateNewMsduList( hOs);
        if(!dataMsduListArr[queueIndex])
        {
            bufferPool_destroy( hTxData->hBufferPool );
            utils_nullTimerDestroy(hOs, pTimer);
          #ifdef TI_DBG
            utils_nullTimerDestroy(hOs, pThroughputTimer);
            utils_nullTimerDestroy(hOs, pAirThroughputTimer);
            utils_nullTimerDestroy(hOs, pJitterTimer);
          #endif
            utils_nullTimerDestroy(hOs, pCreditTimer);
            utils_nullMemoryFree(hOs, mgmtMsduList, sizeof(MsduList_t));
            queueIndex--;
            for(; queueIndex >= 0 ; queueIndex--)
            {
                utils_nullMemoryFree(hOs, dataMsduListArr[queueIndex], sizeof(MsduList_t));
            }
            utils_nullMemoryFree(hOs, hTxData, sizeof(txData_t));
            return NULL;
        }
    }

    hTxData->txDataNumOfQueues = txDataInitParams->txDataNumOfDataQueues;

    /* Threshold to decide whether we drop the packet, or sending it to Fw      */
    /* example: value of 75 will drop any MSDU that stayed it the Driver        */
    /* Queues for more than 75% of the time that was originally defined for it  */
    hTxData->uFracOfLifeTimeToDrop = txDataInitParams->uFracOfLifeTimeToDrop;

    hTxData->TxEventDistributor = DistributorMgr_Create(hOs,MAX_TX_NOTIF_REQ_ELMENTS);

    hTxData->mngMsduList = mgmtMsduList;
    for(queueIndex = 0 ; queueIndex < MAX_NUM_OF_TX_QUEUES ; queueIndex++)
    {
        hTxData->dataMsduListArr[queueIndex] = dataMsduListArr[queueIndex];
    }
    hTxData->pSchedulerTimer = pTimer;
    hTxData->pVadTimer = pVadTimer;
    hTxData->bSchedulerTimerRunning = FALSE;
  #ifdef TI_DBG
    hTxData->pThroughputTimer = pThroughputTimer;
    hTxData->pAirThroughputTimer = pAirThroughputTimer;
    hTxData->pJitterTimer = pJitterTimer;
  #endif
    hTxData->pCreditTimer = pCreditTimer;

    hTxData->hOs = hOs;

	hTxData->bVadTimerEnabled = FALSE;
	hTxData->vadTimerDuration = 0;

    hTxData->creditCalculationTimeout = txDataInitParams->creditCalculationTimeout;
    hTxData->bCreditCalcTimerEnabled  = txDataInitParams->bCreditCalcTimerEnabled;
    hTxData->admCtrlDelayDueToMediumTimeOverUsage = txDataInitParams->admCtrlDelayDueToMediumTimeOverUsage;
    hTxData->admissionDownGradeEnable = txDataInitParams->admissionDownGradeEnable;
    

    return(hTxData);
}

/***************************************************************************
*                           txData_config                                  *
****************************************************************************
* DESCRIPTION:  This function configures the Tx Data module
*
* INPUTS:       hTxData - The object
*               hCtrlData - Handle to the Ctrl Data object
*               hOs - Handle to the Os Abstraction Layer
*               hReport - Handle to the Report object
*               hMemMngr - Handle to the Memory manager object

* OUTPUT:
*
* RETURNS:      OK - Configuration unsuccessful
*               NOK - Configuration unsuccessful
***************************************************************************/

TI_STATUS txData_config(TI_HANDLE       hTxData,
                     TI_HANDLE      hCtrlData,
                     TI_HANDLE      hTnetwDrv,
                     TI_HANDLE      hWhalCtrl,
                     TI_HANDLE      hOs,
                     TI_HANDLE      hReport,
                     TI_HANDLE      hMemMngr,
                     TI_HANDLE      hSiteMgr,
                     TI_HANDLE      hEvHandler,
                     TI_HANDLE      hQosMngr,
                     TI_HANDLE      hPowerMgr)
{
    int qIndex;
    txData_t *pTxData = (txData_t *)hTxData;

    /* configure modules handles */
    pTxData->hCtrlData = hCtrlData;
    pTxData->hTnetwDrv = hTnetwDrv;
    pTxData->hWhalCtrl= hWhalCtrl;
    pTxData->hOs = hOs;
    pTxData->hReport = hReport;
    pTxData->hMemMngr = hMemMngr;
    pTxData->hSiteMgr = hSiteMgr;
    pTxData->hEvHandler = hEvHandler;
    pTxData->hQosMngr = hQosMngr;
    pTxData->hPowerMgr = hPowerMgr;

    /* set Tx parameters */
    pTxData->txDataPortStatus = DEF_TX_PORT_STATUS;
    pTxData->savePortStatus = DEF_TX_PORT_STATUS;
    pTxData->txDataCurrentPrivacyInvokedMode = DEF_CURRENT_PRIVACY_INVOKED_MODE;
    pTxData->saveTxDataCurrentPrivacyInvokedMode = DEF_CURRENT_PRIVACY_INVOKED_MODE;
    pTxData->txDataEapolEncryptionStatus = DEF_EAPOL_ENCRYPTION_STATUS;
    pTxData->saveTxDataEapolEncryptionStatus = DEF_EAPOL_ENCRYPTION_STATUS;

    pTxData->txDataIsSchedulerInWork = DEF_IS_SCHEDULER_IN_WORK;
    pTxData->txDataHalInterfaceStatus = DEF_HAL_INTERFACE_STATUS;

    /* Initialize the parameters related to GWSI and to Scheduler in Work */
    pTxData->txDataGwsiInterfaceStatus = GWSI_OPEN;
    pTxData->txDataIsSchedulerInWork = FALSE;

    pTxData->bCreditCalcTimerRunning = FALSE;
    
    /* encryption header size */
    pTxData->encryptionFieldSize = 0;
    pTxData->saveEncryptionFieldSize = 0;

    /* configure the packet ID buffer pool */
    bufferPool_config( pTxData->hBufferPool, hReport );

    /* configure the Tx queues (msdu lists) */
    /* num of elements is configured by qosMngr */
    for (qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
    {
        if( (msduList_ConfigMsduList( pTxData->dataMsduListArr[qIndex], pTxData->hMemMngr,
                                    pTxData->hReport,pTxData->hOs,0 )) != OK )
        {
            WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
                    ("Error configure DataMsduList num: %d\n",qIndex));
        }

        /*
         * Mark all queues available for TX
         */
        pTxData->txDataAvailableQueue[qIndex] = TRUE;
    }

    if( (msduList_ConfigMsduList( pTxData->mngMsduList, pTxData->hMemMngr,
                                pTxData->hReport, pTxData->hOs,MANAGEMENT_QUEUE_SIZE )) != OK )
    {
        WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
                ("Error configure MgmtMsduList\n"));
    }

    /* reset counters */
    txData_resetCounters (pTxData);
    txData_resetDbgCounters (pTxData);

    WLAN_REPORT_INIT(pTxData->hReport, TX_DATA_MODULE_LOG,
                (".....Tx Data configured successfully\n"));

#ifdef NO_COPY_NDIS_BUFFERS 
   WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
                     ("Driver configured to work in NO COPY MSDU BUFFERS."));
#else
   WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
                     ("Driver is configured to work in COPY MSDU BUFFERS."));
#endif


    return OK;

}

/***************************************************************************
*                           txData_unLoad                                  *
****************************************************************************
* DESCRIPTION:  This function unload the tx data module. It first free the
*               MsduLists and then free the Tx data control block
*
* INPUTS:       hTxData - the object
*
* OUTPUT:
*
* RETURNS:      OK - Unload succesfull
*               NOK - Unload unsuccesfull
***************************************************************************/

TI_STATUS txData_unLoad(TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t *)hTxData;
    TI_STATUS   status = OK;
    int       queueIndex;

    /* check parameters validity */
    if( pTxData == NULL )
    {
        return NOK;
    }

    /* free Data queue msdu list */
    for(queueIndex = 0;queueIndex < MAX_NUM_OF_TX_QUEUES; queueIndex++)
    {
        if (msduList_FreeMsduList( pTxData->dataMsduListArr[queueIndex]) != OK)
        {
            WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
                (" txData_unLoad() : fail to free Data MsduList number: %d\n",queueIndex));
            status = NOK;
        }
    }

    /* free Mgmt queue msdu list */
    if (msduList_FreeMsduList( pTxData->mngMsduList) != OK)
    {
        WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
            (" txData_unLoad() : fail to free Mgmt MsduList \n"));
        status = NOK;
    }

    DistributorMgr_Destroy(pTxData->TxEventDistributor);

    /* free Timer */
    utils_nullTimerDestroy(pTxData->hOs, pTxData->pSchedulerTimer);
    utils_nullTimerDestroy(pTxData->hOs, pTxData->pVadTimer);
  #ifdef TI_DBG
    utils_nullTimerDestroy(pTxData->hOs, pTxData->pThroughputTimer);
    utils_nullTimerDestroy(pTxData->hOs, pTxData->pAirThroughputTimer);
    utils_nullTimerDestroy(pTxData->hOs, pTxData->pJitterTimer);
  #endif
    utils_nullTimerDestroy(pTxData->hOs, pTxData->pCreditTimer);

    /* release the packet ID buffer pool */
    bufferPool_destroy( pTxData->hBufferPool );

    /* free Tx Data control block */
    os_memoryFree(pTxData->hOs, pTxData, sizeof(txData_t));

    return status;
}

/****************************************************************************
*                               txData_stop                                 *
*****************************************************************************
* DESCRIPTION:  this function stop the tx data. It empties the tx queues (msdu
*               lists) from the msdu's and return all tx data parameters to
*               default values
*
* INPUTS:       hTxData - the object
*
* OUTPUT:
*
* RETURNS:      OK - stop successful
*               NOK - stop unsuccessful
****************************************************************************/

TI_STATUS txData_stop(TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t *)hTxData;
    int      queueIndex;

    pTxData->savePortStatus = pTxData->txDataPortStatus;

    /* stop scheduler timer trigger */
    if ( TRUE == pTxData->bSchedulerTimerRunning )
    {
        os_timerStop(pTxData->hOs, pTxData->pSchedulerTimer);
    }

    /* stop vad timer */
    if ( TRUE == pTxData->bVadTimerEnabled )
    {
        os_timerStop(pTxData->hOs, pTxData->pVadTimer);
    }
    /* stop throughput timer */
    if(pTxData->txThroughputTimerEnable == TRUE)
    {
        os_timerStop(pTxData->hOs, pTxData->pThroughputTimer);
        pTxData->txThroughputTimerEnable = FALSE;
    }

    /* stop throughput timer */
    if (pTxData->txAirThroughputTimerEnable)
    {
        os_timerStop (pTxData->hOs, pTxData->pAirThroughputTimer);
        pTxData->txAirThroughputTimerEnable = FALSE;
    }

    /* stop credit calculation timer */
    if ( pTxData->bCreditCalcTimerRunning )
    {
        os_timerStop(pTxData->hOs, pTxData->pCreditTimer);
        pTxData->bCreditCalcTimerRunning = FALSE;
    }

    /* empty Tx data queue from Msdus */
    for(queueIndex = 0;queueIndex < MAX_NUM_OF_TX_QUEUES; queueIndex++)
    {
        if( msduList_EmptyMsduList( pTxData->dataMsduListArr[queueIndex] ) != OK)
        {
            WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
                (" txData_stop() : fail to empty Data Msdu List number: %d\n",queueIndex));
            return NOK;
        }
    }
    /* empty Tx Mgmt queue from Msdus */
    if( msduList_EmptyMsduList( pTxData->mngMsduList ) != OK)
    {
        WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
            (" txData_stop() : fail to empty Mgmt Msdu List \n"));
        return NOK;
    }

    /* set Tx parameters to default values */
    pTxData->txDataPortStatus = DEF_TX_PORT_STATUS;
    pTxData->saveTxDataCurrentPrivacyInvokedMode = pTxData->txDataCurrentPrivacyInvokedMode;
    pTxData->saveTxDataEapolEncryptionStatus = pTxData->txDataEapolEncryptionStatus;
    pTxData->saveEncryptionFieldSize = pTxData->encryptionFieldSize;

    pTxData->txDataCurrentPrivacyInvokedMode = DEF_CURRENT_PRIVACY_INVOKED_MODE;
    pTxData->txDataEapolEncryptionStatus = DEF_EAPOL_ENCRYPTION_STATUS;
    pTxData->encryptionFieldSize = 0;


    WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
        (" txData_stop() :  Succeeded.\n"));

    return OK;

}

/****************************************************************************
*                               txData_start                                *
*****************************************************************************
* DESCRIPTION:  this function start the tx data.
*
* INPUTS:       hTxData - the object
*
* OUTPUT:
*
* RETURNS:      OK - stop succesfull
*               NOK - stop unsuccesfull
****************************************************************************/
TI_STATUS txData_start(TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t *)hTxData;

    /* check parameters validity */
    if( pTxData == NULL )
    {
        WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
            (" txData_start() : Illegal value for hTxData\n"));
        return NOK;
    }

    pTxData->txDataPortStatus = pTxData->savePortStatus;

    return OK;
}
 
/****************************************************************************
*                       txData_recoveryIndication                           *
*****************************************************************************
* DESCRIPTION:  this function clears information on recovery.
*
* INPUTS:       hTxData - the object
*
* OUTPUT:
*
* RETURNS:      
****************************************************************************/
void txData_recoveryIndication (TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t *)hTxData;
    int queueIndex;

    /* Reinitializes packet ID buffer pool (mark all buffers as available) */
    bufferPool_reinit (pTxData->hBufferPool);

    /* Empty Tx data queue from Msdus */
    for (queueIndex = 0; queueIndex < MAX_NUM_OF_TX_QUEUES; queueIndex++)
    {
        if (msduList_EmptyMsduList (pTxData->dataMsduListArr[queueIndex]) != OK)
        {
            WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
                (" txData_stop() : fail to empty data MSDU list number: %d\n", queueIndex));
        }
    }

    /* Empty Tx management queue from MSDU's */
    if (msduList_EmptyMsduList (pTxData->mngMsduList) != OK)
    {
        WLAN_REPORT_ERROR (pTxData->hReport, TX_DATA_MODULE_LOG,
                (" txData_stop() : fail to empty management MSDU list\n"));
    }

    /* 
     * Set GWSI interfaces status as 'opened' 
     * because it can be undefined upon recovery
     */
    pTxData->txDataGwsiInterfaceStatus = GWSI_OPEN;

    for (queueIndex = 0; queueIndex < MAX_NUM_OF_TX_QUEUES; queueIndex++)
    {
        /*
         * Mark all queues available for TX
         */
        pTxData->txDataAvailableQueue[queueIndex] = TRUE;
    }
}


/***************************************************************************
*                           txData_checkQueueSize                          *
****************************************************************************
* DESCRIPTION:  Check the Tx Queue size
*
* INPUTS:       hTxData - the object
*
* OUTPUT:
*
* RETURNS:      OK
*               NOK  - The queue is full
***************************************************************************/
TI_STATUS txData_checkQueueSize(TI_HANDLE hTxData,UINT8 qIndex)
{
    txData_t *pTxData = (txData_t *)hTxData;
    TI_STATUS Status = OK;

    txData_startTxScheduler(pTxData);

    if (pTxData->dataMsduListArr[qIndex]->CurrNumOfMsdu == pTxData->dataMsduListArr[qIndex]->maxNumOfMsdu)
        Status = NOK;

    return Status;
}

/***************************************************************************
*                           txData_copyPacketToMsdu                        *
****************************************************************************
* DESCRIPTION:
*
* INPUTS:       hTxData - the object
*
* OUTPUT:
*
* RETURNS:      OK
*               NOK  - The queue is full
***************************************************************************/
TI_STATUS txData_copyPacketToMsdu(TI_HANDLE hTxData,mem_MSDU_T **pMsdu, int FreeOldMsdu)
{ 
#ifdef NO_COPY_NDIS_BUFFERS
    txData_t *pTxData = (txData_t *)hTxData;

    TI_STATUS       Status = OK;
    mem_BD_T        *pCurrBd;
    mem_MSDU_T      *tempMsdu;
    UINT8           *pMsduData;
    UINT8           *pCurrBufData;
    dot11_header_t  *pdot11Header;


   /*
    * Allocate MSDU+BD+BUFFER+TX_DESCRIPTOR_SIZE to copy to !!
    */
    if(wlan_memMngrAllocMSDU(pTxData->hMemMngr,&tempMsdu,(*pMsdu)->dataLen+TX_TOTAL_OFFSET_BEFORE_DATA,TX_MODULE) != OK)
    {
        WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
            (" txData_copyPacketToMsdu() : No Memory in MemMgr \n"));

        return NOK;
    }

    tempMsdu->dataLen = 0;
    tempMsdu->firstBDPtr->length = 0;
    tempMsdu->txFlags = (*pMsdu)->txFlags;

    pCurrBd = (*pMsdu)->firstBDPtr;
    pMsduData = tempMsdu->firstBDPtr->data + TX_TOTAL_OFFSET_BEFORE_DATA;
    while(pCurrBd)
    {
        pCurrBufData = pCurrBd->data + pCurrBd->dataOffset;
        /* Copy the packet */
        os_memoryCopy(pTxData->hOs, pMsduData, pCurrBufData, pCurrBd->length);
        tempMsdu->dataLen += pCurrBd->length - TNETWIF_WRITE_OFFSET_BYTES;
        tempMsdu->firstBDPtr->length += pCurrBd->length;
        pMsduData += pCurrBd->length;

        pCurrBd = pCurrBd->nextBDPtr;
    }

    tempMsdu->headerLen = (*pMsdu)->headerLen;

    txData_convertEthToWlanHeader( pTxData, tempMsdu );

    /* set wep bit if needed */
    if((tempMsdu->txFlags & TX_DATA_DATA_MSDU) && (pTxData->txDataCurrentPrivacyInvokedMode))
    {
        pdot11Header = (dot11_header_t*)(memMgr_BufData(pMsdu->firstBDPtr) + memMgr_BufOffset(pMsdu->firstBDPtr));
        pdot11Header->fc |= DOT11_FC_WEP;
    }
    else if ((tempMsdu->txFlags & TX_DATA_EAPOL_MSDU ) && (pTxData->txDataEapolEncryptionStatus))
    {
        pdot11Header = (dot11_header_t*)(memMgr_BufData(pMsdu->firstBDPtr) + memMgr_BufOffset(pMsdu->firstBDPtr));
        pdot11Header->fc |= DOT11_FC_WEP;
    }

    if (FreeOldMsdu)
        wlan_memMngrFreeMSDU(pTxData->hMemMngr, memMgr_MsduHandle((*pMsdu)));

    (*pMsdu) = tempMsdu;

#endif

    return OK;
}

/***************************************************************************
*                           txData_sendPktToWlan                           *
****************************************************************************
* DESCRIPTION:  This function is called by the Os Abstraction Layer in order
*               to send packet to the wireless LAN. It calls the header
*               conversion function and passes the to sendMsdu function.
*
* INPUTS:       hTxData - the object
*               pMsdu - pointer the packet in 802.3 format
*
* OUTPUT:
*
* RETURNS:      OK
*               NOK
***************************************************************************/

TI_STATUS txData_sendPktToWlan(TI_HANDLE hTxData, mem_MSDU_T *pMsdu, UINT8 pkt_DTag)
{
    EthernetHeader_t   *pEthHeader;
    UINT16              TypeLength;
    TI_STATUS Status;
    mem_BD_T*           tempBd;
    BOOL                UseConvertHeader = TRUE;

    txData_t *pTxData = (txData_t *)hTxData;

    /* check parameters validity */
    if( pTxData == NULL || pMsdu == NULL )
    {
        return NOK;
    }

    pEthHeader = (EthernetHeader_t*)(memMgr_BufData(pMsdu->firstBDPtr)+memMgr_BufOffset(pMsdu->firstBDPtr));

    /* check if the frame is multicast/broadcast - need for the transmission rate */
    if(IsMacAddressGroup( &pEthHeader->DstAddr ))
        pMsdu->txFlags |= TX_DATA_MULTICAST_FRAME;

    TypeLength = pEthHeader->TypeLength;

      /* Call the Classify function in the Control Module to set the qosTag of the MSDU  */
     if (ctrlData_ClsfrClassifyTxMSDU(pTxData->hCtrlData, pMsdu, pkt_DTag) != OK)
     {
        WLAN_REPORT_DEBUG_TX(pTxData->hReport,
                     (" txData_sendPktToWlan(): No matching classifier found\n"));              
     }


    /* filter MSDU according to Tx Port Status and the Eth Type */
    if ( pTxData->txDataPortStatus != OPEN )
    {
        int queueIndex;
        int acIndex;

        WLAN_REPORT_WARNING(pTxData->hReport, TX_DATA_MODULE_LOG,
                (" txData_sendPktToWlan() : try to transmit Msdu while port is not open (txDataPortStatus = 0x%x)\n", pTxData->txDataPortStatus));
        
        /* Find AC and Tx-queue in order to update the correct Queue measurements */
        acIndex = GET_WME_AC_TYPE_FROM_MSDU(pMsdu);
        queueIndex = GET_QUEUE_INDEX(pTxData,acIndex);
        
        
        /* updating the measurements - dropped packet counter */
        pTxData->txDataReportedCounters[queueIndex].OtherFailCounter++;
        pTxData->txDataReportedCounters[queueIndex].NumPackets++;
                
        if((pTxData->txDataPortStatus == CLOSE) || (pTxData->txDataPortStatus == OPEN_NOTIFY) )
        {
            WLAN_REPORT_WARNING(pTxData->hReport, TX_DATA_MODULE_LOG,
                (" txData_sendPktToWlan() : try to transmit Msdu while port is not open (txDataPortStatus = 0x%x)\n", pTxData->txDataPortStatus));

            /* In case the function return status is NOK, no need to call Os free function  */
            /* Set freefunc in the msdu to null. In this case the MemMngr will not call     */
            /* the Os free function                                                         */
            memMgr_MsduFreeFuncGet(pMsdu) = NULL;

            /* free MSDU */
            if( (wlan_memMngrFreeMSDU(pTxData->hMemMngr, memMgr_MsduHandle(pMsdu))) != OK )
            {
                WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
                    (" txData_sendPktToWlan() : free msdu failed \n"));
            }

            return NOK;
        }

        /* filter Data frames while port is open only for Eapol's */
        if( (pTxData->txDataPortStatus == OPEN_EAPOL) && (TypeLength != EAPOL_PACKET) )
        {
            WLAN_REPORT_WARNING(pTxData->hReport, TX_DATA_MODULE_LOG,
                (" txData_sendPktToWlan() : try to transmit Non Eapol packet while port is open for eapols only\n"));

            /* In case the function return status is NOK, no need to call Os free function. */
            /* Set freefunc in the msdu to null. In this case the MemMngr will not call     */
            /* the Os free function  - because the return staus to the Os is NOK the buffer */
            /* free by the Os                                                               */
            memMgr_MsduFreeFuncGet(pMsdu) = NULL;

            /* frre MSDU */
            if ((wlan_memMngrFreeMSDU(pTxData->hMemMngr, memMgr_MsduHandle(pMsdu))) != OK)
            {
                WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
                    (" txData_sendPktToWlan() : free msdu failed \n"));
            }

            return NOK;
        }
    }

    /* PORT IS OPEN */
#ifdef CORE_TX_DBG
    WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
                (" txData_sendPktToWlan: port is open \n"));
#endif
    

    if( TypeLength != EAPOL_PACKET )
    {
        pMsdu->txFlags |= TX_DATA_DATA_MSDU;
    }
    else
    {
        WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
            (" txData_sendPktToWlan() : Tx an EAPOL frame tranferred to HAL\n"));

        /* because EAPOL is sent from Mgmt queue */
        pMsdu->txFlags |= TX_DATA_EAPOL_MSDU;
    }

    UseConvertHeader = TRUE; /* a patch for WinCe */
#ifdef NO_COPY_NDIS_BUFFERS
    if (pMsdu->txFlags & TX_DATA_FROM_OS)
        UseConvertHeader = FALSE;   /* don't convert on external (OS) packets */
#endif

        if (UseConvertHeader == TRUE)
    {
        /* Header conversion from Ethernet to 802.11 */
        txData_convertEthToWlanHeader( pTxData, pMsdu );
    }

    /* update last BD */
    tempBd = pMsdu->firstBDPtr;
    while(tempBd->nextBDPtr)
    {
        tempBd = tempBd->nextBDPtr;
    }
    pMsdu->lastBDPtr = tempBd;

    Status = txData_txSendMsdu( pTxData, pMsdu );
    if( Status == NOK )
    {
        WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
            (" txData_sendPktToWlan() : txData_txSendMsdu failed \n"));
    }

    if ((OK == Status) && (GET_WME_AC_TYPE_FROM_MSDU(pMsdu) == QOS_AC_VO))
    {
	txData_resetVadTimer(pTxData);
    }
    return Status;
}

/***************************************************************************
*                           txData_txSendMsdu                              *
****************************************************************************
* DESCRIPTION:  This function is insert the MSDU to transmit to the proper
*               tx queue and give a trigger to the scheduler to start
*               transmission to the wireless LAN.
*
* INPUTS:       hTxData - the object
*               pMsdu - pointer the MSDU in 802.11 format
*
* OUTPUT:
*
* RETURNS:  OK
*           NOK
***************************************************************************/

TI_STATUS txData_txSendMsdu(TI_HANDLE hTxData, mem_MSDU_T *pMsdu )
{
    dot11_header_t     *pdot11Header;
    TI_STATUS Status = OK;
    int queueIndex;
    acTrfcType_e acIndex;


    txData_t *pTxData = (txData_t *)hTxData;

    /* ctrlData_txMsdu(pTxData->hCtrlData,&pMsdu); 4x related */
    
    if(pMsdu == NULL)
    {
        WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
            (" txData_txSendMsdu() : DISCARD Packet...... \n"));
        txData_startTxScheduler(pTxData);

        return NOK;
    }


#ifndef NO_COPY_NDIS_BUFFERS /* buffers are copied by OAL*/
    /* set wep bit if needed */
    if((pMsdu->txFlags & TX_DATA_DATA_MSDU) && (pTxData->txDataCurrentPrivacyInvokedMode))
    {
        /* update offset of header */
        pdot11Header = (dot11_header_t*)(memMgr_BufData(pMsdu->firstBDPtr) + memMgr_BufOffset(pMsdu->firstBDPtr));
        pdot11Header->fc |= DOT11_FC_WEP;
    }
    else if ((pMsdu->txFlags & TX_DATA_EAPOL_MSDU ) && (pTxData->txDataEapolEncryptionStatus))
    {
        /* update offset of header */
        pdot11Header = (dot11_header_t*)(memMgr_BufData(pMsdu->firstBDPtr) + memMgr_BufOffset(pMsdu->firstBDPtr));
        pdot11Header->fc |= DOT11_FC_WEP;
    }
#endif /*NO_COPY_NDIS_BUFFERS*/

#ifdef EXC_MODULE_INCLUDED
    if ((pMsdu->txFlags & TX_DATA_IAPP_MSDU) && (pTxData->txDataCurrentPrivacyInvokedMode))
    {
        /* update offset of header */
        pdot11Header = (dot11_header_t*)(memMgr_BufData(pMsdu->firstBDPtr) + memMgr_BufOffset(pMsdu->firstBDPtr));
        pdot11Header->fc |= DOT11_FC_WEP;
    }
#endif


    WLAN_REPORT_DEBUG_TX(pTxData->hReport,
        (" txData_txSendMsdu() : Sending PAcket pMsdu %x pMsdu->txFlags %x \n",pMsdu,pMsdu->txFlags));

    /* insert the msdu to the the appropriate queue */
    if( (pMsdu->txFlags & TX_DATA_DATA_MSDU) || (pMsdu->txFlags & TX_DATA_NULL_MSDU) )
    {
        /* Find Tx-queue and AC to use for Tx, and if downgraded from original one update the UP. */
        txData_selectQueueAndUpdateUserPriority (pTxData, pMsdu, &queueIndex, &acIndex);
        
        /* set insertion time for further expiry timeout calculation */
        pMsdu->insertionTime = os_timeStampUs (pTxData->hOs);

        /* see if the frame is tagged VO. */
        /* Note: Check actual tag even if current not working in WME, to support voice anyway. */
        if( GET_WME_AC_TYPE_FROM_MSDU(pMsdu) == QOS_AC_VO)
        {
            /*
             * If the frame is tagged VO and power save is on, send psPoll before the VO frame.
             */
            if(txData_acVoPsPollMode(pTxData) == TRUE)
            {
                mem_MSDU_T *pMsduPsPoll;

                if(txData_getPsPollFrame(pTxData,&pMsduPsPoll) == OK)
                {
                    /* increment debug counters */
                    pTxData->txDataDbgCounters.dbgInsertToMsduListBytes[queueIndex] += pMsduPsPoll->dataLen;
                    pTxData->txDataDbgCounters.dbgInsertToMsduListPackets[queueIndex]++;

                    /* set insertion time for further expiry timeout calculation */
                    pMsduPsPoll->insertionTime = os_timeStampUs (pTxData->hOs);

                    /* insert to queueIndex queue */
                    if( msduList_Insert( pTxData->dataMsduListArr[queueIndex] , &pMsduPsPoll ) != OK )
                    {
                        pTxData->txDataDbgCounters.dbgDropedFromMsduListPackets[queueIndex]++;
                        /* the first msdu in list has removed and the new one has inserted */
                        WLAN_REPORT_WARNING(pTxData->hReport, TX_DATA_MODULE_LOG,
                                            (" txData_sendPktToWlan() : Msdu List num %d was full \n",queueIndex));

                        /* free the msdu which removed from the list (returned in pMsdu) */
                        /*---------------------------------------------------------------*/
                        /* set msdu tx status to Fail (this status is used by OS) */
                        memMgr_MsduFreeArg2Get(pMsduPsPoll) = NOK;

                        /* free the msdu */
                        wlan_memMngrFreeMSDU(pTxData->hMemMngr, memMgr_MsduHandle(pMsduPsPoll));
                        pTxData->txDataDbgCounters.dbgDropedPacketsCounter++;
                    }/*msduList_Insert*/
                }/*txData_getPsPollFrame*/
            }/*txData_acVoPsPollMode*/
        }

        /* insert to data queue */
        /* if we didn't succeed to insert psPolls exclude VO packet also */
        if( msduList_Insert( pTxData->dataMsduListArr[queueIndex] , &pMsdu ) != OK )
        {

            pTxData->txDataDbgCounters.dbgDropedFromMsduListPackets[queueIndex]++;
            /* the first msdu in list has removed and the new one has inserted */
            WLAN_REPORT_WARNING(pTxData->hReport, TX_DATA_MODULE_LOG,
                (" txData_txSendMsdu() : Data Msdu [%d] List was full \n",queueIndex));

            /* free the msdu which removed from the list (returned in pMsdu) */
            /*---------------------------------------------------------------*/
            /* set msdu tx status to Fail (this status is used by OS) */
            memMgr_MsduFreeArg2Get(pMsdu) = NOK;

            /* free the msdu */
            wlan_memMngrFreeMSDU(pTxData->hMemMngr, memMgr_MsduHandle(pMsdu));
            pTxData->txDataDbgCounters.dbgDropedPacketsCounter++;
            
            pTxData->txDataReportedCounters[queueIndex].NumPackets++; 
            pTxData->txDataReportedCounters[queueIndex].OtherFailCounter++;

            return OK;
        }

        /* increament debug counters */
        pTxData->txDataDbgCounters.dbgInsertToMsduListBytes[queueIndex] += pMsdu->dataLen;
        pTxData->txDataDbgCounters.dbgInsertToMsduListPackets[queueIndex]++;

        WLAN_REPORT_DEBUG_TX(pTxData->hReport,
            (" txData_txSendMsdu() : insert data packet to queue # %d \n",queueIndex));
    }
    else
    {
        /* Management frame, Eapol and null frame (for measuring a non serving channel)
            are also sent from the mgmt queue */

        /* set insertion time for further expiry timeout calculation */
        pMsdu->insertionTime = os_timeStampUs (pTxData->hOs);

        WLAN_REPORT_DEBUG_TX(pTxData->hReport,
            (" txData_txSendMsdu() : insert mngt packet to Management queue pMsdu %x \n",pMsdu));

        if( msduList_Insert( pTxData->mngMsduList , &pMsdu ) != OK )
        {
            /* the first msdu in list has removed and the new one has inserted */
            WLAN_REPORT_WARNING(pTxData->hReport, TX_DATA_MODULE_LOG,
                (" txData_txSendMsdu() : Mgmt Msdu List was full \n"));

            /* free the msdu which removed from the list (returned in pMsdu) */
            /*---------------------------------------------------------------*/
            /* set msdu tx status to Fail (this status is used by OS) */
            memMgr_MsduFreeArg2Get(pMsdu) = NOK;

            /* free the msdu */
            wlan_memMngrFreeMSDU(pTxData->hMemMngr, memMgr_MsduHandle(pMsdu));
            pTxData->txDataDbgCounters.dbgDropedPacketsCounter++;
        }
    }

    {
        /* call the scheduler in order to transmit the frame to the Hal */
    Status = txData_startTxScheduler(pTxData);
        if ( Status == NOK )
        {
            WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
                (" txData_txSendMsdu() : txData_startTxScheduler error \n"));
        }
    }

    /* this is the status of the current Msdu */
    return OK;
}

/***************************************************************************
*                       txData_startTxScheduler                            *
****************************************************************************
* DESCRIPTION:  This function start the transmission process.
*               It select msdu to transmit from the tx queues and send
*               it to the Hal
*
* INPUTS:       hTxData - the object
*
* OUTPUT:
*
* RETURNS:  OK
*           NOK
***************************************************************************/

TI_STATUS txData_startTxScheduler(TI_HANDLE hTxData)
{
    txData_t            *pTxData = (txData_t *)hTxData;
    mem_MSDU_T          *pMsdu;
    MsduList_t          *pMsduList = NULL;
    txData_attr_t       txAttr;
    UINT8               count, selectedQueueIndex;
    TI_STATUS           txStatus;
    txPacketIdAttr_t    *pPacketId;
    dot11_header_t      *pDot11Header;
    bssType_e           currBssType;
    macAddress_t        currBssId;
    UINT32              msduTimeToExpiry;

#ifdef SUPPORT_4X
    hwTxInformation_t   hwTxInformation;
    BOOL                fourXEnable;
#endif

    /* check if HAL is open now - used for recovery only!!! */
    if ( pTxData->txDataHalInterfaceStatus == TX_DATA_CLOSED )
    {
        return OK; /* TODO ronen: doesn't seem to be used any more, remove */
    }

    /* Checking if Tx is now disabled */
    if(pTxData->txDisable == DISABLE_IMMEDIATELY)
    {
        WLAN_REPORT_WARNING(pTxData->hReport, TX_DATA_MODULE_LOG,
                (" txData_startTxScheduler() : Tx Disabled!!! \n"));

        /* start scheduler timer */
        if ( FALSE == pTxData->bSchedulerTimerRunning )
        {
            os_timerStart( pTxData->hOs, pTxData->pSchedulerTimer, SCHEDULER_TIMER, FALSE );
            pTxData->bSchedulerTimerRunning = TRUE;
        }

        pTxData->txDataIsSchedulerInWork = FALSE;
        return OK;
    }

   /*
    * in case of non serialized system no need to
    * run the scheduler if it is already in work
    */
    if(pTxData->txDataIsSchedulerInWork == TRUE)
    {
        WLAN_REPORT_WARNING(pTxData->hReport, TX_DATA_MODULE_LOG,
            (" txData_startTxScheduler() : Scheduler already in work...........!!!! \n"));
        return OK;
    }
    else
    {
        pTxData->txDataIsSchedulerInWork = TRUE;
    }

    /* try to transmit DEF_SCHEDULER_THRESHOLD MSDUs */
    count = DEF_SCHEDULER_THRESHOLD;

    /*
     * if non of the queues is available non msdu will be selected
     */
       
    while ( (0 < count--) && (GWSI_OPEN == pTxData->txDataGwsiInterfaceStatus) )
    {
        /* select queue to transmit from */
        if( txData_schedulerSelectQueueToTransmitFrom( pTxData, &pMsduList,&selectedQueueIndex ) != TX_QUEUE_SELECTED_OK)
        {
            WLAN_REPORT_DEBUG_TX(pTxData->hReport,
                    (" txData_startTxScheduler() : No Msdu waiting to transmit  \n"));

           pTxData->txDataIsSchedulerInWork = FALSE;
            return OK;
        }

        WLAN_REPORT_DEBUG_TX(pTxData->hReport,
            (" txData_startTxScheduler() : txData_schedulerSelectQueueToTransmitFrom Returned from selectedQueueIndex %d   \n",selectedQueueIndex));

        
#ifdef SUPPORT_4X
        
        ctrlData_get4xStatus(pTxData->hCtrlData,&fourXEnable);

        if(fourXEnable == TRUE  && pMsduList != pTxData->mngMsduList)
        {
           /*
            * receive Acx tx queuue information for transmission decision
            */
            whalTx_getTxQueueInfo(pTxData->hWhalTx,
                                    DEFAULT_QUEUE_TO_HAL,
                                    &hwTxInformation.hwNumOfFreeMsdu,
                                    &hwTxInformation.hwNumOfBusyMsdu,
                                    &hwTxInformation.hwNumOfFreeBDs,
                                    &hwTxInformation.hwTotalAvailMem );

           /*
            * call ctrl Data - DequeueMsdu...
            * pMsdu - the msdu to transmit.
            * txAttr - the tx attributes for msdu transmission
            */
            status = ctrlData_txDequeueMsdu(pTxData->hCtrlData, &pMsdu, pMsduList, &txAttr, &hwTxInformation);

            if(status == DO_NOT_SEND_MSDU)
            {
        WLAN_REPORT_ERROR(pTxData->hReport,
                    (" txData_startTxScheduler() : TX_STATUS_PENDING  \n"));

                pTxData->txDataIsSchedulerInWork = FALSE;

                return TX_STATUS_PENDING;
            }
        }

#else  /*  if not SUPPORT_4X  */

        {
            /* increment debug counter */
            pTxData->txDataDbgCounters.dbgScheduledOutPackets[selectedQueueIndex]++;

            if( msduList_WatchFirst( pMsduList ,&pMsdu) != OK )
            {
                WLAN_REPORT_ERROR( pTxData->hReport, TX_DATA_MODULE_LOG, 
                                   ("Unable to retrieve first MSDU from queue index:%d\n", selectedQueueIndex) );
                pTxData->txDataIsSchedulerInWork = FALSE;
                return OK;
            }

            WLAN_REPORT_DEBUG_TX(pTxData->hReport,
                    (" txData_startTxScheduler() : After msduList_WatchFirst pMsdu %d   \n",pMsdu));

#ifdef NO_COPY_NDIS_BUFFERS
            
            if(pMsdu->txFlags & TX_DATA_FROM_OS)
            {
                if(txData_copyPacketToMsdu(pTxData, &pMsdu, 1 /* do FreeOldMsdu */) != OK)
                {
                    WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
                        (" txData_startTxScheduler() : txData_copyPacketToMsdu FAILED  \n"));

                    pTxData->txDataIsSchedulerInWork = FALSE;
                    return NOK;
                }
            }

#endif /* NO_COPY_NDIS_BUFFERS */

        }

#endif /* SUPPORT_4X */

        /* check MSDU expiry time, and if it didn't expire send it to GWSI */
        if ((msduTimeToExpiry = txDataTimeToMsduExpiry (hTxData, pMsdu, selectedQueueIndex)) == 0)
        {
            /* MSDU time expired - drop it */
            WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
                ("txData_startTxScheduler drop msdu due to expiry time, queueId = %d \n",selectedQueueIndex));

            /* Get MSDU with dequeuing */
            if ( (msduList_GetFirst( pMsduList, &pMsdu )) != OK )
            {
                /* No MSDU is waiting to transmit */
                WLAN_REPORT_ERROR (pTxData->hReport, TX_DATA_MODULE_LOG,
                    ("%s: can't get Msdu from pMsduList[ %d ]  \n", __FUNCTION__, selectedQueueIndex));
                pTxData->txDataIsSchedulerInWork = FALSE;
                return NOK;
            }


            pTxData->txDataDbgCounters.dbgDroppedDueExpiryTimePackets[selectedQueueIndex]++;
            
            pTxData->txDataReportedCounters[selectedQueueIndex].NumPackets++; 
            pTxData->txDataReportedCounters[selectedQueueIndex].OtherFailCounter++; 
            
            /* free the MSDU */
            memMgr_MsduFreeArg2Get(pMsdu) = OK;
    
                if( (wlan_memMngrFreeMSDU(pTxData->hMemMngr, memMgr_MsduHandle(pMsdu))) != OK )
                {
                    WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
                        (" txData_startTxScheduler() : free msdu failed \n"));
                }
    
            /* continue to next packet */
            continue;
        }

        /* Collect txAttr from control */
        ctrlData_getTxAttributes(pTxData->hCtrlData, pMsdu->txFlags, &txAttr, selectedQueueIndex);

        /* allocate a buffer for packet ID storage */
        pPacketId = (txPacketIdAttr_t*)bufferPool_allocateBuffer( pTxData->hBufferPool );
        if ( BUFFER_POOL_NO_BUFFER == pPacketId )
        {
            WLAN_REPORT_ERROR (pTxData->hReport, TX_DATA_MODULE_LOG,
                (" %s : No PacketID Available!!\n", __FUNCTION__));

            /* shouldn't happen, since we have enough packet ID buffers as the FW has TX descriptors! */
            pTxData->txDataIsSchedulerInWork = FALSE;
            return NOK;
        }

        /* store necessary information in packet ID */
        pPacketId->pMsdu = pMsdu;
        if (pMsdu->txFlags & TX_DATA_DATA_MSDU)
        {
            pPacketId->bDataMsdu = TRUE;
        }
        else
        {
            pPacketId->bDataMsdu = FALSE;
        }
        pPacketId->txQid = selectedQueueIndex;
        pPacketId->txCompleteFlags = pMsdu->txCompleteFlags; 
        pPacketId->maxTransmitRate = txAttr.HwRate;
      #ifdef TI_DBG
        os_memoryZero (pTxData->hOs, pPacketId->timeStamp, sizeof(pPacketId->timeStamp));
        pPacketId->timeStamp[0] = os_timeStampUs (pTxData->hOs);
      #endif

        pPacketId->msduDataLen = pMsdu->dataLen;

        /* destination mac */
        pDot11Header = (dot11_header_t*)(memMgr_BufData(pMsdu->firstBDPtr)+ memMgr_BufOffset(pMsdu->firstBDPtr));
        ctrlData_getCurrBssTypeAndCurrBssId(pTxData->hCtrlData, &currBssId, &currBssType);
        if(currBssType == BSS_INDEPENDENT)
             MAC_COPY( pTxData->hOs, &(pPacketId->destinationMac), &(pDot11Header->address1) );
        else
             MAC_COPY( pTxData->hOs, &(pPacketId->destinationMac), &(pDot11Header->address3) );

        /* mark in packet ID that XFER done and TX complete were not called yet */
        pPacketId->bTxCompleteCalled = pPacketId->bXferDoneCalled = FALSE;


      #ifdef TI_DBG
        if ((pMsdu->txFlags & TX_DATA_DATA_MSDU) != 0 && pMsdu->timeStampNum > 0)
        {
            UINT32 uCoreDelay;

            /* add time stamp */
            wlan_memMngrAddTimeStamp (pTxData->hMemMngr, pMsdu);

            uCoreDelay = pMsdu->timeStamp[pMsdu->timeStampNum - 1] - pMsdu->timeStamp[0];

            /* update core delay and jitter */
            pTxData->txJitter[selectedQueueIndex].jitter.core += 
                ABS (pTxData->txJitter[selectedQueueIndex].last_delay.core - uCoreDelay);
            pTxData->txJitter[selectedQueueIndex].last_delay.core = uCoreDelay;
            pTxData->txJitter[selectedQueueIndex].delay.core += uCoreDelay;
            pTxData->txJitter[selectedQueueIndex].count.core ++;
            if (uCoreDelay > pTxData->txJitter[selectedQueueIndex].max_delay.core)
                pTxData->txJitter[selectedQueueIndex].max_delay.core = uCoreDelay;
        }
      #endif

        /* send the packet to the GWSI layer */
        txStatus = CORE_AdaptTx_SendPacket( CORE_AdaptTx_handle, selectedQueueIndex, pMsdu, &txAttr, (UINT32)pPacketId, msduTimeToExpiry );

        WLAN_REPORT_DEBUG_TX (pTxData->hReport,
                              ("txData_startTxScheduler(): called CORE_AdaptTx_SendPacket pMsdu=%d, selectedQueueIndex=%d,txStatus=%d\n",
                              pMsdu,selectedQueueIndex,txStatus));

        switch ( txStatus )
        {
        /* 
         * XFER_DONE is received when the packet has been synchronously transferred to the FW.
         * in this case, XFER complete will not be called, only TX complete
         */
        case SEND_PACKET_XFER_DONE:

            pPacketId->driverHandlingTime = (os_timeStampUs (pTxData->hOs) - pMsdu->insertionTime) / 1000;

            /* Get MSDU with dequeuing */
            if ( (msduList_GetFirst( pMsduList, &pMsdu )) != OK )
            {
                /* No MSDU is waiting to transmit */
                WLAN_REPORT_ERROR (pTxData->hReport, TX_DATA_MODULE_LOG,
                    (" %s : can't get Msdu from pMsduList[ %d ]  \n", __FUNCTION__, selectedQueueIndex));
                pTxData->txDataIsSchedulerInWork = FALSE;
                return NOK;
            }

            /* free the MSDU, since XFER complete won't be called */
            if ( (wlan_memMngrFreeMSDU(pTxData->hMemMngr, memMgr_MsduHandle(pMsdu))) != OK )
            {
               WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
                   ("%s: free msdu failed \n", __FUNCTION__));
               pTxData->txDataIsSchedulerInWork = FALSE;
               return NOK;
            }
            else
            {
                pTxData->txDataDbgCounters.dbgNumOfMsduFreeInTxTransfer[ pPacketId->txQid ]++;
            }

            /* Nullify the MSDU pointer, just in case */
            pPacketId->pMsdu = NULL;

            /* mark in packet ID that XFER done was called */
            pPacketId->bXferDoneCalled = TRUE;

            /* increment debug counter */
            pTxData->txDataDbgCounters.dbgSendToGwsiQosPackets[ selectedQueueIndex ]++;
            pTxData->txDataDbgCounters.dbgNumOfMsduXferDoneInShceduler[ selectedQueueIndex ]++;
            break;

        /* 
         * SUCCESS is received when the packet has not yet been sent to the FW, but another packet
         * transfer can start immediately
         */
        case SEND_PACKET_SUCCESS:
            /* Get msdu with dequeuing */
            if ( (msduList_GetFirst( pMsduList, &pMsdu )) != OK )
            {
                /* No msdu is waiting to transmit */
                WLAN_REPORT_ERROR (pTxData->hReport, TX_DATA_MODULE_LOG,
                    (" %s : can't get Msdu from pMsduList[ %d ]  \n", __FUNCTION__, selectedQueueIndex));
                pTxData->txDataIsSchedulerInWork = FALSE;
                return NOK;
            }

            /* increment debug counter */
            pTxData->txDataDbgCounters.dbgNumOfMsduSuccessInScheduler[ selectedQueueIndex ]++;
            pTxData->txDataDbgCounters.dbgSendToGwsiQosPackets[selectedQueueIndex]++;
            break;

        /*
         * PENDING is received when the packet has been received by the GWSI layer, and the double buffer
         * mechanism is busy. It indicates that no more packets (from all queues) are to be sent until a
         * XFER complete indication is received
         */
        case SEND_PACKET_PENDING:
            /* Get msdu with dequeuing */
            if ( (msduList_GetFirst( pMsduList, &pMsdu )) != OK )
            {
                /* No msdu is waiting to transmit */
                WLAN_REPORT_ERROR (pTxData->hReport, TX_DATA_MODULE_LOG,
                    (" %s : can't get Msdu from pMsduList[ %d ]  \n", __FUNCTION__, selectedQueueIndex));
                pTxData->txDataIsSchedulerInWork = FALSE;
                return NOK;
            }

            /* mark the GWSI interface state as pending (no more frames can be sent) */
            pTxData->txDataGwsiInterfaceStatus = GWSI_PENDING;                

            /* increment debug counter */
            pTxData->txDataDbgCounters.dbgNumOfMsduPendingInScheduler[ selectedQueueIndex ]++;
            pTxData->txDataDbgCounters.dbgSendToGwsiQosPackets[selectedQueueIndex]++;
            break;

        /*
         * BUSY is received when the packet has NOT been received by the GWSI layer (and needs to be 
         * retransmitted). It indicates that the specific FW queue is full and no more packets from this
         * specific queue are to be sent until a TX complete indication for this queue is received (but 
         * packets from other queues may be sent).
         */
        case SEND_PACKET_BUSY:
            /* don't dequeue the packet! it was not sent! */

            /* in addition, release the packet ID buffer */
            bufferPool_releaseBuffer( pTxData->hBufferPool, pPacketId );

            /* mark the specific queue as not available */
            pTxData->txDataAvailableQueue[ selectedQueueIndex ] = FALSE;

            /* update debug counters */
            pTxData->txDataDbgCounters.dbgNumOfMsduBusyInScheduler[ selectedQueueIndex ]++;

            break;

        /* 
         * ERROR is received when a frame is sent although a PENDING indication had been previously 
         * received and no XFER complete had been receive since, or because a BUSY indication had been
         * received for a specific queue and no TX complete for this queue had been received since (and
         * a packet from this queue had been sent now).
         */
        case SEND_PACKET_ERROR:
            /* don't dequeue the packet! it was not sent! */

            /* in addition, release the packet ID buffer */
            bufferPool_releaseBuffer( pTxData->hBufferPool, pPacketId );

            /* start scheduler timer */
            if ( FALSE == pTxData->bSchedulerTimerRunning )
            {
                os_timerStart( pTxData->hOs, pTxData->pSchedulerTimer, SCHEDULER_TIMER, FALSE );
                pTxData->bSchedulerTimerRunning = TRUE;
            }
            pTxData->txDataIsSchedulerInWork = FALSE;
            WLAN_REPORT_ERROR( pTxData->hReport, TX_DATA_MODULE_LOG, 
                                (" %s: received status SEND_PACKET_ERROR from CORE_AdaptTx_SendPacket\n", __FUNCTION__) );

            /* update debug counters */
            pTxData->txDataDbgCounters.dbgNumOfMsduErrorInScheduler[ selectedQueueIndex ]++;

            return NOK;

/*            break; - unreachable*/

        case SEND_PACKET_RECOVERY:
            break;

        default:
            WLAN_REPORT_ERROR( pTxData->hReport, TX_DATA_MODULE_LOG, 
                               (" %s: received status %d from CORE_AdaptTx_SendPacket\n", __FUNCTION__, txStatus) );
            break;
            }
    
            WLAN_REPORT_INFORMATION (pTxData->hReport, TX_DATA_MODULE_LOG, 
                ("txData_startTxScheduler() : MSDU sent: TxQ = %d TxStatus = %d\n", selectedQueueIndex, txStatus));
    }  /* end of while (count) */

    pTxData->txDataIsSchedulerInWork = FALSE;

    return OK;
}
/***************************************************************************
*                           txData_sendPacketTransfer                      *
****************************************************************************
* DESCRIPTION:  GWSI sendPacketTransfer CB. called after transferring a packet
*               to TNET. 
*               The function free the transfered MSDU and set GWSI port status
*               to OPEN. This is the only function which synchronized GWSI port
*               status.
*               TNET queues buffer status are updated and the scheduler is executed
*               trying to send another MSDU.
*
* INPUTS:       hTxData - the object
*
* OUTPUT:
*
* RETURNS:  void
*
***************************************************************************/
void txData_sendPacketTransfer(TI_HANDLE          hTxData,
                               UINT32    aPacketIdAttr)
{
    txData_t *pTxData = (txData_t *)hTxData;
    txPacketIdAttr_t* pPacketId = (txPacketIdAttr_t*)aPacketIdAttr;


    WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
            ("%s: XFER complete CB called\n", __FUNCTION__));
    pTxData->txDataDbgCounters.dbgNumOfMsduTxTransferCB[ pPacketId->txQid ]++;

    WLAN_REPORT_DEBUG_TX(pTxData->hReport, 
        ("%s: XFER complete CB called\n", __FUNCTION__));

    /* updating GWSI status to open */
    pTxData->txDataGwsiInterfaceStatus = GWSI_OPEN;

    /* free the MSDU */
    if ( pPacketId->pMsdu != NULL )
    {
        pPacketId->driverHandlingTime = (os_timeStampUs (pTxData->hOs) - pPacketId->pMsdu->insertionTime) / 1000;

        if( (wlan_memMngrFreeMSDU(pTxData->hMemMngr, memMgr_MsduHandle(pPacketId->pMsdu))) != OK )
        {
           WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
               ("%s: free msdu failed \n", __FUNCTION__));
        }
        else
        {
            pTxData->txDataDbgCounters.dbgNumOfMsduFreeInTxTransfer[ pPacketId->txQid ]++;
        }
    }
    else
    {
        WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
                ("%s: pMsdu = NULL !!!!\n", __FUNCTION__));
            pTxData->txDataDbgCounters.dbgNumOfNullMsdu++;
    }


    /* nullify the MSDU pointer, just in case */
    pPacketId->pMsdu = NULL;

    /* mark in packet ID that XFER done was called */
    pPacketId->bXferDoneCalled = TRUE;
    
    /* 
     * and check if to release packet ID. In rare cases, TX complete can be called before XFER done. In these
     * cases, the packet ID should be released here.
     */
    if ( TRUE == pPacketId->bTxCompleteCalled )
    {
        bufferPool_releaseBuffer( pTxData->hBufferPool, (bufferPool_buffer_t)aPacketIdAttr );
    }

    /* 
     * check port status - if it is disconnected, it means the STA is disconnected, and thus
     * the XFER complete is ignored, to avoid race conditions (when, for example, the MSDU was 
     * already freed on txData_stop)
     */
    if ( CLOSE == pTxData->txDataPortStatus )
    {
        WLAN_REPORT_WARNING( pTxData->hReport, TX_DATA_MODULE_LOG,
                             (" %s: XFER complete CB called when port is CLOSED!", __FUNCTION__)); 
        return;
    }

    /* try to schedule another MSDU */
    txData_startTxScheduler(hTxData);
}


/***************************************************************************
*                           txData_txCompleteUpdate                        *
****************************************************************************
* DESCRIPTION:  check if there are more packets in the queue to transmit,
*               and that the queues in the HW are empty. if so then release
*               the HW. else call to the scheduler.
*
* INPUTS:       hTxData - the object
*               txStatus - status of Tx (OK = ok, other = failed)
*               TxQid - The Tx queue index.
*
* OUTPUT:
*
* RETURNS:  TI_STATUS - is success then OK, else NOK
*
***************************************************************************/
TI_STATUS txData_txCompleteUpdate(TI_HANDLE hTxData, txCompleteAttr_t *pCmpltAttr)
{
    txData_t *pTxData = (txData_t *)hTxData;
    UINT8 qIndex;
    txPacketIdAttr_t* pPacketId = (txPacketIdAttr_t*)pCmpltAttr->packetId;    

    /*
     * when host processes the packets , we not working with Queue free event
     * so queue status in TNET is updated in tx complete and the scheduler is triggered.
     */
    qIndex = pPacketId->txQid;

    WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
            ("txData_txCompleteUpdate , queueId: %d  status: %d\n", qIndex, pCmpltAttr->status));

    /*
     * first update that TNET Queue is available to get another packet 
     */
    pTxData->txDataAvailableQueue[qIndex] = TRUE;

    /* update TX counters for txDistributer */
    txData_UpdateTxCounters( hTxData, pCmpltAttr );

    /* update dbg counters */
    pTxData->txDataDbgCounters.dbgNumOfsendPacketComplete[qIndex]++;
    if (pCmpltAttr->status == SEND_COMPLETE_SUCCESS) 
    {
        pTxData->txDataDbgCounters.dbgTxCmpltOk[qIndex]++;
        pTxData->txDataDbgCounters.dbgTxCmpltOkBytes[qIndex] += pPacketId->msduDataLen;
      #if defined(TI_DBG)
        pTxData->txJitter[qIndex].jitter.air += 
            ABS (pTxData->txJitter[qIndex].last_delay.air - pCmpltAttr->actualDurationInAir);
        pTxData->txJitter[qIndex].jitter.fw += 
            ABS (pTxData->txJitter[qIndex].last_delay.fw - pCmpltAttr->fwHandlingTime);
        pTxData->txJitter[qIndex].last_delay.fw = pCmpltAttr->fwHandlingTime;
        pTxData->txJitter[qIndex].last_delay.air = pCmpltAttr->actualDurationInAir;
        pTxData->txJitter[qIndex].delay.fw += pCmpltAttr->fwHandlingTime;
        pTxData->txJitter[qIndex].delay.air += pCmpltAttr->actualDurationInAir;
        if (pCmpltAttr->fwHandlingTime > pTxData->txJitter[qIndex].max_delay.fw)
            pTxData->txJitter[qIndex].max_delay.fw = pCmpltAttr->fwHandlingTime; 
        if (pCmpltAttr->actualDurationInAir > pTxData->txJitter[qIndex].max_delay.air)
            pTxData->txJitter[qIndex].max_delay.air = pCmpltAttr->actualDurationInAir; 
      #endif
    }
    else
    {
        pTxData->txDataDbgCounters.dbgTxCmpltError[qIndex]++;
        /* on error, update TX counters (xmit error count) */
    }


    /* check asynchronous in packetId */
    if ( (pTxData->txDataDbgCounters.dbgNumOfMsduTxTransferCB[qIndex] +  pTxData->txDataDbgCounters.dbgNumOfMsduXferDoneInShceduler[qIndex])< 
         pTxData->txDataDbgCounters.dbgNumOfsendPacketComplete[qIndex])
    {
        WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
                ("txData_txCompleteUpdate ,  qIndex = %d Num of XFER done CB + XFER done status = %d  dbgNumOfsendPacketComplete = %d!!!!\n",
                qIndex, pTxData->txDataDbgCounters.dbgNumOfMsduTxTransferCB[qIndex] +  
                 pTxData->txDataDbgCounters.dbgNumOfMsduXferDoneInShceduler[qIndex],
                pTxData->txDataDbgCounters.dbgNumOfsendPacketComplete[qIndex]));
    }

    /* check if XFER done was called for this packet */
    if ( TRUE == pPacketId->bXferDoneCalled )
    {
        /* free the packet ID buffer */
        bufferPool_releaseBuffer( pTxData->hBufferPool, (bufferPool_buffer_t)pPacketId );
    }
    else
    {
        /* 
         * in rare cases, TX complete can be called before XFER done. If this is the case,
         * simply mark that TX complete was called. The packet ID will be freed on XFER done
         */
        pPacketId->bTxCompleteCalled = TRUE;
    }

    /* run the scheduler */
    txData_startTxScheduler(hTxData);

    return OK;
}

/***************************************************************************
*                           txData_sendPacketDebug                         *
****************************************************************************
* DESCRIPTION:  GWSI sendPacketDebug CB, called upon issuing interrupt to TNET.
*               The function calculates GWSI delay and jitter.
*
* INPUTS:       hTxData    - the object
*               uPacketId  - packet handle
* OUTPUT:
*
* RETURNS:  void
*
***************************************************************************/
#ifdef TI_DBG
void txData_sendPacketDebug (TI_HANDLE hTxData, UINT32 uPacketId, UINT32 uDebugInfo)
{

    txData_t *pTxData = (txData_t *)hTxData;
    txPacketIdAttr_t* pPacketId = (txPacketIdAttr_t *)uPacketId;

    if (pTxData != NULL && pPacketId != NULL)
    {
        UINT32 uXferDelay = os_timeStampUs (pTxData->hOs);

        switch (uDebugInfo)
        {
        case 0:
            /* Calculate full XFER delay */
            if (uXferDelay >= pPacketId->timeStamp[0])
            {
                uXferDelay -= pPacketId->timeStamp[0];

                /* Update jitter statistics */
                pTxData->txJitter[pPacketId->txQid].delay.xfer += uXferDelay;
                pTxData->txJitter[pPacketId->txQid].jitter.xfer +=
                ABS (uXferDelay - pTxData->txJitter[pPacketId->txQid].last_delay.xfer);
                pTxData->txJitter[pPacketId->txQid].last_delay.xfer = uXferDelay;
                pTxData->txJitter[pPacketId->txQid].count.xfer ++; 
                if (uXferDelay > pTxData->txJitter[pPacketId->txQid].max_delay.xfer)
                    pTxData->txJitter[pPacketId->txQid].max_delay.xfer = uXferDelay;
            }
            break;

        default:
            if (uDebugInfo < 5)
                pPacketId->timeStamp[uDebugInfo] = uXferDelay;
            break;
        }
    }
}
#endif


/***************************************************************************
*                           txData_getHighestAdmittedAc                     *
****************************************************************************
* DESCRIPTION:  This function calculate the highest admitted AC starting from
*               a given ac index. if it fails it returns acIndex of Best effort.
*
* INPUTS:       hTxData - the object
*               qosTag - Qos tag
*
* OUTPUT:
*
* RETURNS:  OK
*           NOK
***************************************************************************/

static int txData_getHighestAdmittedAc(txData_t *pTxData, int startingAcIndex)
{
    int qIndex;

    if ((startingAcIndex > MAX_NUM_OF_AC - 1) || (startingAcIndex < FIRST_AC_INDEX))
    {
        WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
                          (" txData_getHighestAdmittedAc() :  failed, startingAcIndex = %d \n",startingAcIndex));
        return QOS_AC_BE;
    }

    qIndex = GET_QUEUE_INDEX(pTxData, startingAcIndex);

    /* If desired queue is not admitted, find highest Tx queue that doesn't require admission. */
    if(pTxData->dataMsduListArr[qIndex]->admissionState == AC_NOT_ADMITTED)
    {
        while(qIndex >= 0)
        {
            if(pTxData->dataMsduListArr[qIndex]->admissionRequired == ADMISSION_NOT_REQUIRED)
                break;
            qIndex--;
        }
    }
    
    if(qIndex < 0)
    {
        WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
            (" txData_getHighestAdmittedAc() :  failed, qIndex = %d \n",qIndex));
        return startingAcIndex;
    }

    return pTxData->dataMsduListArr[qIndex]->acId;
}



/***************************************************************************
*                           txData_selectQueueAndUpdateUserPriority                           *
****************************************************************************
* DESCRIPTION:  This function calculate the queue index according to msdu qos tag
*               if the queue isn't admitted tt returns the highest admitted queue
*               bellow.
*               In addition, if the MSDU has been downgraded due to admission control,
*               we update the QosControl accordingly
*
* INPUTS:       hTxData - the object
*               pMSDU - pointer to packet
*
* OUTPUT:       selectedQueue - pointer to result variable which will hold the selected queue index
*               acIndex - selected admission control
*
* RETURNS:  OK
*           NOK
***************************************************************************/
static int txData_selectQueueAndUpdateUserPriority (txData_t *pTxData, mem_MSDU_T *pMsdu, int *selectedQueue, acTrfcType_e *selectedAc)
{
    int startingAcIndex;
    int acIndex;
    dot11_header_t      *pdot11Header;

    if (pMsdu->qosTag > MAX_NUM_OF_802_1d_TAGS - 1)
    {
        WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
            (" txData_selectQueueAndUpdateUserPriority() : txData_getQueueIndex failed, pMsdu->qosTag = %d \n",pMsdu->qosTag));
        *selectedAc = QOS_AC_BE;
        *selectedQueue = GET_QUEUE_INDEX(pTxData, QOS_AC_BE);
        return NOK;
    }
    /* calc ac according tag to ac table */
    acIndex = startingAcIndex = pTxData->txDataQosParams.tag_ToAcClsfrTable[pMsdu->qosTag];

    /* get highest admitted AC */
    if(pTxData->admissionDownGradeEnable == TRUE)
        acIndex = txData_getHighestAdmittedAc(pTxData,startingAcIndex);

    /* If the highest admitted AC is not the "starting" AC that we originally desired, we were downgraded due to admission control
       and we should update the QosControl field accordingly 
       In addition, we do not want to modify the QosControl unless the header actually contains Qos data */
    if ((acIndex != startingAcIndex) && (pTxData->txDataQosParams.headerConverMode == QOS_CONVERT))
    {
      pdot11Header = (dot11_header_t*)(memMgr_BufData(pMsdu->firstBDPtr)+ memMgr_BufOffset(pMsdu->firstBDPtr));      

      pdot11Header->qosControl = wmeAcToUpIndex[acIndex];
    }

    /* convert acIndex to qIndex */
    *selectedQueue = pTxData->txDataAcTrfcCtrl[acIndex].QueueIndex;
    *selectedAc = (acTrfcType_e)acIndex;

    return OK;
}



/***************************************************************************
*                           txData_acVoPsPollMode                          *
****************************************************************************
* DESCRIPTION:  This function determines if we are current in power save mode
*               sending voice packet with SW ps-poll method.
*
* INPUTS:       hTxData - the object
*               qosTag - Qos tag
*
* OUTPUT:
*
* RETURNS:  TRUE
*           FALSE
***************************************************************************/

static BOOL txData_acVoPsPollMode(txData_t *pTxData)
{
    BOOL ps_status = FALSE;

    ps_status = PowerMgr_getPsStatus(pTxData->hPowerMgr);

    if (ps_status &&
        pTxData->txDataAcTrfcCtrl[QOS_AC_VO].PsMode == PS_SCHEME_LEGACY_PSPOLL)
    {
        return TRUE;
    }

    return FALSE;
}



/***************************************************************************
*                       txData_disableTransmission                         *
****************************************************************************
* DESCRIPTION:  This function sets an internal flag in order to diable
*                   transmission.
*
* INPUTS:       hTxData - the object
*               reason  - indicates if the reason for transmission disable
*                           is measuring of non serving channel or Switch Channel command
*
* OUTPUT:
*
* RETURNS:       OK on success, NOK otherwise
***************************************************************************/
TI_STATUS txData_disableTransmission(TI_HANDLE hTxData,txDisableReason_e reason)
{
    txData_t *pTxData = (txData_t *)hTxData;
    pTxData->txDisable = reason;
    return OK;
}

/***************************************************************************
*                       txData_enableTransmission                          *
****************************************************************************
* DESCRIPTION:  This function sets an internal flag in order to enable
*                   back the transmission.
*
* INPUTS:       hTxData - the object
*
*
* OUTPUT:
*
* RETURNS:       OK on success, NOK otherwise
***************************************************************************/
TI_STATUS txData_enableTransmission(TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t *)hTxData;
    pTxData->txDisable = NO_DISABLE;
    txData_startTxScheduler(pTxData);

    return OK;
}



/***************************************************************************
*               txData_schedulerSelectQueueToTransmitFrom                  *
****************************************************************************
* DESCRIPTION:  This function selects the tx queue to transmit MSDU from.
*               Management MSDUs are selected in first priority.
*               For data MSDUs, the queue selection is done by the following order:
*               1) The queue is not empty.
*               2) It is permitted to transmit (admitted and not blocked by mediumTime).
*               3) It has minimal number of Hw blocks waiting for Tx.
*                   Note: this is to fill all queues for efficient EDCA process in FW.
*               4) It has minimal history-counter (shows it wasn't selected lately).
*               5) It is the highest priority queue.
*
*               Note: Selection between queues is done by step 'n' only if all steps before it
*                       gave identical results. Also, steps 1 & 2 are mandatory for selection.
*
* INPUTS:       hTxData - the object
*
* OUTPUT:       pMsduListPtr -       a pointer to the selected queue list-pointer.
*               selectedQueueIndex - a pointer to the selected queue index.
*
* RETURNS:      TX_QUEUE_SELECTED_OK - a queue was selected.
*               NO_TX_QUEUE_SELECTED - no msdu to transmit.
***************************************************************************/

static TI_STATUS txData_schedulerSelectQueueToTransmitFrom( TI_HANDLE hTxData, MsduList_t** pMsduListPtr,UINT8 *selectedQueueIndex )
{
    txData_t    *pTxData = (txData_t *)hTxData;
    UINT8       acIndex;
    int         qIndex;
    UINT32      currentTimeStamp = os_timeStampMs(pTxData->hOs);
    UINT32      AllQueuesMinTime = 0;
    UINT32      currQueuesTime = 0;
    MsduList_t  *pMsduList;
    UINT32      currentQueueWeight; 
    UINT32      bestWeight = Q_LEAST_WEIGHT; /* Weight of preferred queue (lowest value is selected). */
    int         bestWeightQueueIndex = 0;    /* Index of preferred queue. */
    MsduList_t  *bestWeightQueueList = NULL; /* Pointer to the preferred queue list. */

    /* If management MSDU is waiting, select it and return. */
    if( pTxData->mngMsduList->CurrNumOfMsdu > 0 )
    {
        *pMsduListPtr = pTxData->mngMsduList;

        /* get highest admitted AC starting from VO */
        acIndex = txData_getHighestAdmittedAc(pTxData,QOS_AC_VO);

        /* convert acIndex to queue index */
        *selectedQueueIndex = GET_QUEUE_INDEX(pTxData,acIndex);


        if (pTxData->txDataAvailableQueue[*selectedQueueIndex] == TRUE)
        {
            WLAN_REPORT_INFORMATION (pTxData->hReport, TX_DATA_MODULE_LOG,
               (" txData_schedulerSelectQueueToTransmitFrom() : Management MSDU selected to Tx\n"));
            return TX_QUEUE_SELECTED_OK;
        }
        else
        {
            WLAN_REPORT_INFORMATION (pTxData->hReport, TX_DATA_MODULE_LOG,
                (" txData_schedulerSelectQueueToTransmitFrom() : pTxData->txDataAvailableQueue[%d] = FALSE  \n",*selectedQueueIndex));
        }
    }


    /* 
     * Loop over all core data Tx queues and look for the preferred one to transmit from:
     * =================================================================================
     *   Note:  Starting from the highest priority ensures that if multiple queues have 
     *            identical status, the highest priority one among them will be selected.  
     */
    for (qIndex = MAX_NUM_OF_TX_QUEUES - 1; qIndex >= 0; qIndex--)
    {
        pMsduList = pTxData->dataMsduListArr[qIndex];

        if (pMsduList->selectionHistoryCounter)
            pMsduList->selectionHistoryCounter--;
        
        /* If queue is empty or is not admitted to transmit, continue to next queue.  */
        if ((pMsduList->CurrNumOfMsdu == 0) || (pMsduList->admissionState != AC_ADMITTED))
            continue;

        /*
         * see only if Queue is available.
         * continue to the next Q if not available
         */
        if (pTxData->txDataAvailableQueue[qIndex] == FALSE)
            continue;

            /* If we are configured to enforce excess Tx time limits, handle the algorithm. */
            if (pMsduList->useAdmissionAlgo) 
            {
            /* Handle case of timer wraparound. */
                if (currentTimeStamp < pMsduList->lastTimeStamp)
                    pMsduList->lastTimeStamp = 0;
                
            /* If we it's not time to enable Tx yet for this queue, update time to try again if needed. */ 
            if (currentTimeStamp < pMsduList->lastTimeStamp + pMsduList->enableTransmissionTime)
            {
                /* currQueuesTime represents the amount of time the tx scheduler has to wait before 
                    it will be able to transmit from this queue. */
                    currQueuesTime = (pMsduList->lastTimeStamp + pMsduList->enableTransmissionTime - currentTimeStamp);
                    
                /* Update the minimum time left till we can Tx from any of the queues. */
                if (AllQueuesMinTime == 0)
                        AllQueuesMinTime = currQueuesTime;
                    else
                        AllQueuesMinTime = MIN(AllQueuesMinTime, currQueuesTime);
            
                /* We can't Tx from this queue yet so continue to next queue. */
                continue;
            }
        } 

        /****  If we got here, this queue has something to Tx and it is permitted to Tx now. ****/

        /* Now calculate this queue's weight for selection:  
         *      Higher 16 bits:  Number of used HW blocks by this AC.
         *      Lower  16 bits:  Count down from last time it was selected. */
        currentQueueWeight  = (UINT32)TnetwDrv_txHwQueue_GetUsedHwBlks( pTxData->hTnetwDrv, qIndex ) << 16;
        currentQueueWeight |= (UINT32)pMsduList->selectionHistoryCounter;

        /* If current queue's weight isn't lower (lower is better!) than previous queues, 
             continue to next queue. */
        if (currentQueueWeight >= bestWeight)
            continue;

        /* Save weight, index and list-pointer of best candidate queue so far . */
        bestWeight = currentQueueWeight;
        bestWeightQueueIndex = qIndex;
        bestWeightQueueList = pMsduList;
    }
    
    
    /* If we have a delayed queue, we need to re-trigger the scheduler later by timer. */
    if (AllQueuesMinTime != 0)
    {   
        /* msdu is waiting to transmit */
        WLAN_REPORT_INFORMATION (pTxData->hReport, TX_DATA_MODULE_LOG, 
            (" txData_schedulerSelectQueueToTransmitFrom() : Start mediumTime timer for - %d ms\n", AllQueuesMinTime));
        
        if ( pTxData->bSchedulerTimerRunning == TRUE )
        {
            os_timerStop(pTxData->hOs, pTxData->pSchedulerTimer);
        }
        os_timerStart(pTxData->hOs,pTxData->pSchedulerTimer, AllQueuesMinTime, FALSE);
        pTxData->bSchedulerTimerRunning = TRUE;
    }


    /* If we have a queue we can transmit from:  */
    if (bestWeight != Q_LEAST_WEIGHT)
    {
        /* Set selected queue history counter to its initial value, indicating it was selected
            lately and its priority is temporarily reduced. */
        bestWeightQueueList->selectionHistoryCounter = Q_SELECTION_HISTORY_LEVEL;

        /* Provide the selected queue index and list pointer. */
        *pMsduListPtr = bestWeightQueueList;
        *selectedQueueIndex = bestWeightQueueIndex;

        WLAN_REPORT_INFORMATION (pTxData->hReport, TX_DATA_MODULE_LOG, 
            (" txData_schedulerSelectQueueToTransmitFrom() : Selected-TxQ = %d,  Weight = 0x%x\n",
                bestWeightQueueIndex, bestWeight));
        
        return TX_QUEUE_SELECTED_OK;
    }

    
    /* If we got here, no queue is currently suitable for transmition. */
    WLAN_REPORT_DEBUG_TX(pTxData->hReport, 
        (" txData_schedulerSelectQueueToTransmitFrom() : No queue selected for Tx\n"));

    return NO_TX_QUEUE_SELECTED;
}



/***************************************************************************
*                           txData_getParam                                *
****************************************************************************
* DESCRIPTION:  get a specific parameter
*
* INPUTS:       hTxData - the object
*
* OUTPUT:       pParamInfo - structure which include the value of
*               the requested parameter
*
* RETURNS:      OK
*               NOK
***************************************************************************/
TI_STATUS txData_getParam(TI_HANDLE hTxData, paramInfo_t *pParamInfo)
{
    txData_t *pTxData = (txData_t *)hTxData;
    UINT32  tID;

    /* check handle validity */
    if( pTxData == NULL  )
    {
        WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
            (" txData_getParam() : Illegal parametrs value \n"));
        return NOK;
    }

    switch (pParamInfo->paramType)
    {
        case TX_DATA_PORT_STATUS_PARAM:
            pParamInfo->content.txDataPortStatus = pTxData->txDataPortStatus;
            break;

        case TX_DATA_CURRENT_PRIVACY_INVOKE_MODE_PARAM:
            pParamInfo->content.txDataCurrentPrivacyInvokedMode = pTxData->txDataCurrentPrivacyInvokedMode;
            break;

        case TX_DATA_EAPOL_ENCRYPTION_STATUS_PARAM:
            pParamInfo->content.txDataEapolEncryptionStatus = pTxData->txDataEapolEncryptionStatus;
            break;

        case TX_DATA_CONVERT_HEADER_MODE:
            pParamInfo->content.txDataQosParams.qosParams.headerConverMode = pTxData->txDataQosParams.headerConverMode;
            break;

        case TX_DATA_COUNTERS_PARAM:
            os_memoryCopy( pTxData->hOs, &(pTxData->tempTxDataCounters[ 0 ]), &(pTxData->txDataCounters[ 0 ]), 
                           sizeof(txDataCounters_t) * MAX_NUM_OF_TX_QUEUES );
            pParamInfo->content.pTxDataCounters = &(pTxData->tempTxDataCounters[ 0 ]);
            break;

        case TX_DATA_REPORT_TS_STATISTICS:
            tID = GET_QUEUE_INDEX(pTxData, pParamInfo->content.tsMetricsCounters.acID);
            os_memoryCopy(pTxData->hOs, 
                          pParamInfo->content.tsMetricsCounters.pTxDataCounters, 
                          &(pTxData->txDataReportedCounters[tID]), 
                          sizeof(txDataCounters_t));
            os_memoryZero(pTxData->hOs, 
                          &(pTxData->txDataReportedCounters[tID]), 
                          sizeof(txDataCounters_t));
            break;

        case TX_DATA_GET_VAD:
            pParamInfo->content.txDataVadTimerParams.vadTimerEnabled  = pTxData->bVadTimerEnabled;
            pParamInfo->content.txDataVadTimerParams.vadTimerDuration = pTxData->vadTimerDuration;
            
       	    WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
	    	    ("txData_setParams: GET_VAD (enable=%d; duration=%d ms)\n", 
    		      pTxData->bVadTimerEnabled, pTxData->vadTimerDuration));
 
        	break;

        default:
            return (PARAM_NOT_SUPPORTED);
/*            WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
                (" txData_getParam() : PARAMETER NOT SUPPORTED \n"));
            return NOK;
            break; - unreachable*/
    }

    return (OK);
}



/***************************************************************************
*                           txData_setParam                                *
****************************************************************************
* DESCRIPTION:  set a specific parameter
*
* INPUTS:       hTxData - the object
*               pParamInfo - structure which include the value to set for
*               the requested parameter
*
* OUTPUT:
*
* RETURNS:      OK
*               NOK
***************************************************************************/
TI_STATUS txData_setParam(TI_HANDLE hTxData, paramInfo_t *pParamInfo)
{
    txData_t *pTxData = (txData_t *)hTxData;
    UINT8 queueIndex;
    UINT8 acID = pParamInfo->content.txDataQosParams.acID; /* Note: acID is relevant only in  
                                                                    some of the param-types!! */

    /* check handle validity */
    if( pTxData == NULL  )
    {
        WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
            (" txData_setParam() : Illegal parametrs value \n"));
        return NOK;
    }

    
    switch (pParamInfo->paramType)
    {
    case TX_DATA_PORT_STATUS_PARAM:

    {
        /* Set the new TX port status CLOSE/OPEN_NOTIFY/OPEN_EAPOL/OPEN */
        WLAN_REPORT_DEBUG_TX (pTxData->hReport, 
                              (("txData_setParam: Set txDataPortStatus from 0x%x to 0x%x\n"),
                                pTxData->txDataPortStatus , pParamInfo->content.txDataPortStatus));

        pTxData->txDataPortStatus = pParamInfo->content.txDataPortStatus;
        break;
    }

    case TX_DATA_CURRENT_PRIVACY_INVOKE_MODE_PARAM:
        pTxData->txDataCurrentPrivacyInvokedMode = pParamInfo->content.txDataCurrentPrivacyInvokedMode;
        break;

    case TX_DATA_EAPOL_ENCRYPTION_STATUS_PARAM:
        pTxData->txDataEapolEncryptionStatus = pParamInfo->content.txDataEapolEncryptionStatus;
        break;

    case TX_DATA_HAL_INTERFACE_STATUS_PARAM:
        pTxData->txDataHalInterfaceStatus = pParamInfo->content.txDataHalInterfaceStatus;
        break;

    case TX_DATA_PS_MODE_PARAM:
        pTxData->txDataAcTrfcCtrl[acID].PsMode = 
            pParamInfo->content.txDataQosParams.acTrfcCtrl.PsMode;
        break;

    case TX_DATA_CONFIG_TX_QUEUE_SIZE:
        queueIndex = pParamInfo->content.txDataQosParams.acTrfcCtrl.QueueIndex;
           pTxData->txDataAcTrfcCtrl[acID].TxQueueSize = 
               pParamInfo->content.txDataQosParams.acTrfcCtrl.TxQueueSize;
        if(queueIndex >= MAX_NUM_OF_TX_QUEUES)
           {
               WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
               ("Error configure Data MsduList , wrong queue index = %d\n", queueIndex));
            return NOK;
        }
        pTxData->dataMsduListArr[queueIndex]->acId = acID;
        pTxData->txDataAcTrfcCtrl[acID].QueueIndex = queueIndex;

        /* also set the opposute direction conversion table (queue ID -> ac ID) */
        txData_SetQidToAcTable( hTxData, queueIndex, queueIndex, acID );

        if( (msduList_SetMsduListNumOfElements( pTxData->dataMsduListArr[queueIndex],
                pTxData->txDataAcTrfcCtrl[acID].TxQueueSize)) != OK )
           {
               WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
                       ("Error configure MgmtMsduList\n"));
               return NOK;
           }
        break;

    case TX_DATA_CONFIG_TX_QUEUE_OVFLOW_POLICY:
        queueIndex = pParamInfo->content.txDataQosParams.acTrfcCtrl.QueueIndex;
        pTxData->txDataAcTrfcCtrl[acID].QueueOvFlowPolicy = pParamInfo->content.txDataQosParams.acTrfcCtrl.QueueOvFlowPolicy;
        if(queueIndex >= MAX_NUM_OF_TX_QUEUES)
        {
            WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
                 ("Error configure MsduList over flow policy , wrong queue index = %d\n",queueIndex));
            return NOK;
        }
        pTxData->txDataAcTrfcCtrl[acID].QueueIndex  = queueIndex;
        msduList_SetMsduListOverFlowPolicy(pTxData->dataMsduListArr[pTxData->txDataAcTrfcCtrl[acID].QueueIndex],
                                           pTxData->txDataAcTrfcCtrl[acID].QueueOvFlowPolicy);
        break;

    case TX_DATA_CONFIG_AC_MSDU_LIFE_TIME:
        pTxData->txDataAcTrfcCtrl[acID].MsduLifeTime = 1000 * pParamInfo->content.txDataQosParams.acTrfcCtrl.MsduLifeTime;
        break;

    case TX_DATA_CONFIG_AC_ACK_POLICY:
        pTxData->txDataAcTrfcCtrl[acID].ackPolicy = pParamInfo->content.txDataQosParams.acTrfcCtrl.ackPolicy;
        break;

    case TX_DATA_AC_ADMISSION_STATE:
        pTxData->dataMsduListArr[GET_QUEUE_INDEX(pTxData, acID)]->admissionRequired = 
            pParamInfo->content.txDataQosParams.qosParams.admissionRequired;

        pTxData->dataMsduListArr[GET_QUEUE_INDEX(pTxData, acID)]->admissionState = 
            pParamInfo->content.txDataQosParams.qosParams.admissionState;
        break;

    case TX_DATA_CONVERT_HEADER_MODE:
        pTxData->txDataQosParams.headerConverMode = pParamInfo->content.txDataQosParams.qosParams.headerConverMode;
        break;

    case TX_DATA_TAG_TO_AC_CLASSIFIER_TABLE:
        os_memoryCopy(pTxData->hOs,(pTxData->txDataQosParams.tag_ToAcClsfrTable),
            pParamInfo->content.txDataQosParams.qosParams.tag_ToAcClsfrTable,sizeof(acTrfcType_e) * MAX_NUM_OF_802_1d_TAGS);
        break;

    case TX_DATA_SET_AC_QUEUE_INDEX:
        queueIndex = pParamInfo->content.txDataQosParams.acTrfcCtrl.QueueIndex;

        pTxData->txDataAcTrfcCtrl[pParamInfo->content.txDataQosParams.acID].QueueIndex = queueIndex;
        /* also set the opposute direction conversion table (queue ID -> ac ID) */
        txData_SetQidToAcTable( hTxData, queueIndex, queueIndex, acID );
        break;

    case TX_DATA_SET_MEDIUM_USAGE_THRESHOLD:
       txData_setMediumUsageThresholds (hTxData,
            (UINT8)pParamInfo->content.txDataMediumUsageThreshold.uAC,
            pParamInfo->content.txDataMediumUsageThreshold.uHighThreshold,
            pParamInfo->content.txDataMediumUsageThreshold.uLowThreshold);
       break;

    case TX_DATA_GET_MEDIUM_USAGE_THRESHOLD:
        /* SET operation is performed, but actually this is only for AC parameter transfer from Utility Adapter to driver, since copy
          of user supplied block of data (and vice versa) is only performed in SetParam calls, the driver can also modify the supplied
          structure and thus return it to user mode */
        queueIndex = pTxData->txDataAcTrfcCtrl[pParamInfo->content.txDataMediumUsageThreshold.uAC].QueueIndex;

        /* get threshold */
        pParamInfo->content.txDataMediumUsageThreshold.uHighThreshold = pTxData->dataMsduListArr[queueIndex]->highMediumUsageThreshold;
        pParamInfo->content.txDataMediumUsageThreshold.uLowThreshold = pTxData->dataMsduListArr[queueIndex]->lowMediumUsageThreshold;
        
        break;

    case TX_DATA_POLL_AP_PACKETS_FROM_AC:

         queueIndex = pTxData->txDataAcTrfcCtrl[pParamInfo->content.txDataPollApPacketsFromACid].QueueIndex;

         if (queueIndex <= QOS_AC_MAX)
            return (txData_sendVadFrame (pTxData, queueIndex));
         else
            return PARAM_VALUE_NOT_VALID;
       
/*       break; - unreachable */

    case TX_DATA_ENCRYPTION_FIELD_SIZE:
        /* set the space to reserve for encrypted frames */
        pTxData->encryptionFieldSize = pParamInfo->content.txDataEncryptionFieldSize;
        break;

    case TX_DATA_RESET_COUNTERS_PARAM:
        txData_resetCounters( hTxData );
        break;

    case TX_DATA_SET_VAD:
	{
	    BOOL bVadTimerEnabled;
	    UINT16 vadTimerDuration;

       	    bVadTimerEnabled = pParamInfo->content.txDataVadTimerParams.vadTimerEnabled;
            vadTimerDuration = pParamInfo->content.txDataVadTimerParams.vadTimerDuration;    
   	    WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
    	    	("txData_setParams: SET_VAD (enable=%d; duration=%d ms)\n", 
    	    	  bVadTimerEnabled, vadTimerDuration));

    	    txData_setVadTimer(hTxData, bVadTimerEnabled, vadTimerDuration);
	}

    	break;

    default:
        return (PARAM_NOT_SUPPORTED);
/*        WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
            (" txData_setParam() : PARAMETER NOT SUPPORTED \n"));
        return NOK;
        break; - unreachable */
    }

    return (OK);
}


/***************************************************************************
*                   txData_convertEthToWlanHeader                          *
****************************************************************************
* DESCRIPTION:  this function convert the msdu header from ethernet format
*               to the 802.11 header format
*
* INPUTS:       hTxData - the object
*               pMsdu - msdu in ethernet format
*
* OUTPUT:       pMsdu - msdu in 802.11 format
*
* RETURNS:      
***************************************************************************/
static void txData_convertEthToWlanHeader (txData_t *pTxData, mem_MSDU_T *pMsdu)
{
    EthernetHeader_t    *pEthHeader;
    dot11_header_t      dot11Header;
    Wlan_LlcHeader_T    WlanSnapHeader;
    UINT16              swapedTypeLength;
    bssType_e           currBssType;
    macAddress_t        currBssId;
    UINT8               SNAP_OUI_802_1H[] = SNAP_OUI_802_1H_BYTES;
    UINT8               SNAP_OUI_RFC1042[] = SNAP_OUI_RFC1042_BYTES;
    acTrfcType_e        acIndex;
    char*               pData;

    /* initialize the frame header length */
    pMsdu->headerLen = txData_GetWlanHeaderLength( pTxData,
                                                   memMgr_BufData(pMsdu->firstBDPtr) + 
                                                   memMgr_BufOffset(pMsdu->firstBDPtr),
                                                   pMsdu->txFlags );

    /*
     * Set the Eth pointer to the beginning of the first Bd
        */  
       pEthHeader = (EthernetHeader_t*)(memMgr_BufData(pMsdu->firstBDPtr) + memMgr_BufOffset(pMsdu->firstBDPtr));

       /*
     * Initialize working copy of the dot11header to zero
        */
    os_memoryZero(pTxData->hOs,&dot11Header,sizeof(dot11_header_t));
        
        /* 
         * Convert the header 802.3 ---> 802.11 onto the dot11Header working copy
         */
    /* set Qos control */
    if( pTxData->txDataQosParams.headerConverMode == QOS_CONVERT )
    {
        /* set qos Tag */
        dot11Header.qosControl = ((UINT16)pMsdu->qosTag & QOS_CONTROL_TAG_MASK);

        /* 
                 * set ac ack policy according to the 
                 * Ack policy defined for the particular AC
                 */
        acIndex = (acTrfcType_e)GET_WME_AC_TYPE_FROM_MSDU(pMsdu);
        if(pTxData->txDataAcTrfcCtrl[acIndex].ackPolicy == ACK_POLICY_LEGACY)
        {
            dot11Header.qosControl &= ~DOT11_QOS_CONTROL_DONT_ACK;
        }
        else
        {
            dot11Header.qosControl |= DOT11_QOS_CONTROL_DONT_ACK;
        }
    }

    /* receive BssId and Bss Type from control module */
    ctrlData_getCurrBssTypeAndCurrBssId(pTxData->hCtrlData, &currBssId, &currBssType);

    if (currBssType == BSS_INDEPENDENT)
    {
        MAC_COPY(pTxData->hOs,(&dot11Header.address1),(&pEthHeader->DstAddr));
        MAC_COPY(pTxData->hOs,(&dot11Header.address2),(&pEthHeader->SrcAddr));
        MAC_COPY(pTxData->hOs,(&dot11Header.address3),(&currBssId));

        if( pTxData->txDataQosParams.headerConverMode == QOS_CONVERT )
            dot11Header.fc = DOT11_FC_DATA_QOS;
        else
            dot11Header.fc = DOT11_FC_DATA;
    }
    else /* infrastructure BSS */
    {
        MAC_COPY(pTxData->hOs,(&dot11Header.address1),(&currBssId));
        MAC_COPY(pTxData->hOs,(&dot11Header.address2),(&pEthHeader->SrcAddr));
        MAC_COPY(pTxData->hOs,(&dot11Header.address3),(&pEthHeader->DstAddr));

        if( pTxData->txDataQosParams.headerConverMode == QOS_CONVERT )
            dot11Header.fc = DOT11_FC_DATA_QOS | DOT11_FC_TO_DS;
        else
            dot11Header.fc = DOT11_FC_DATA | DOT11_FC_TO_DS;
    }

    swapedTypeLength = wlan_htons(pEthHeader->TypeLength);

    /* Detect the packet type and decide if to create a     */
    /*          new SNAP or leave the original LLC.         */
    /*------------------------------------------------------*/
    if( swapedTypeLength > ETHERNET_MAX_PAYLOAD_SIZE )
    {
        /* Create the SNAP Header:     */
        /*-----------------------------*/
        /*
         * Make a working copy of the SNAP header 
         * initialised to zero
         */
        os_memoryZero(pTxData->hOs,&WlanSnapHeader,sizeof(Wlan_LlcHeader_T));
        WlanSnapHeader.DSAP = SNAP_CHANNEL_ID;
        WlanSnapHeader.SSAP = SNAP_CHANNEL_ID;
        WlanSnapHeader.Control = LLC_CONTROL_UNNUMBERED_INFORMATION;

        /* Check to see if the Ethertype matches anything in the translation     */
        /* table (Appletalk AARP or DixII/IPX).  If so, add the 802.1h           */
        /* SNAP.                                                                 */

        if(( ETHERTYPE_APPLE_AARP == swapedTypeLength ) ||
           ( ETHERTYPE_DIX_II_IPX == swapedTypeLength ))
        {
            /* Fill out the SNAP Header with 802.1H extention   */
            os_memoryCopy(pTxData->hOs, &WlanSnapHeader.OUI, SNAP_OUI_802_1H,
                            sizeof( WlanSnapHeader.OUI ) );
        }
        else
        {
            /* otherwise, add the RFC1042 SNAP   */
            os_memoryCopy(pTxData->hOs, &WlanSnapHeader.OUI, SNAP_OUI_RFC1042,
                            sizeof( WlanSnapHeader.OUI) );
        }
        /* set type length */
        WlanSnapHeader.Type = pEthHeader->TypeLength;
    }

    /*
     * Now - copy wlan header and snap overriding Ethernet header.
     */

    /*
       first the wlan header
       the header might not include the qosControl which will be erase 
       later by the snap header
     */
    /* pData starts after the reserved place for bus txn and the TxDescriptor */
    
    /* update data offset - bus txn extra space + TxDescriptor */
    memMgr_BufOffset(pMsdu->firstBDPtr) = TX_TOTAL_OFFSET_BEFORE_DATA;

    pData = memMgr_BufData(pMsdu->firstBDPtr) + memMgr_BufOffset(pMsdu->firstBDPtr);
    os_memoryCopy(pTxData->hOs,pData,&dot11Header,pMsdu->headerLen);

    /* update data length */
    pMsdu->dataLen = pMsdu->dataLen - ETHERNET_HDR_LEN + pMsdu->headerLen;

    /* now the snap */
    if( swapedTypeLength > ETHERNET_MAX_PAYLOAD_SIZE )
    {
        pData += (pMsdu->headerLen - sizeof(Wlan_LlcHeader_T));
        os_memoryCopy(pTxData->hOs,pData,&WlanSnapHeader,sizeof(Wlan_LlcHeader_T));
    }

    return;
}

/***************************************************************************
*                       txData_resetCounters                               *
****************************************************************************
* DESCRIPTION:  Reset the tx data module counters
*
* INPUTS:       hTxData - the object
*
* OUTPUT:
*
* RETURNS:
***************************************************************************/
void txData_resetCounters(TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t *)hTxData;

    os_memoryZero(pTxData->hOs, &pTxData->txDataCounters, sizeof(txDataCounters_t) * MAX_NUM_OF_TX_QUEUES);
    pTxData->currentConsecutiveRetryFail = 0;
}

/***************************************************************************
*                       txData_resetDbgCounters                            *
****************************************************************************
* DESCRIPTION:  Reset the tx data module debug counters
*
* INPUTS:       hTxData - the object
*
* OUTPUT:
*
* RETURNS:
***************************************************************************/
void txData_resetDbgCounters(TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t *)hTxData;

    os_memoryZero(pTxData->hOs, &pTxData->txDataDbgCounters, sizeof(txDataDbgCounters_t));
}

/***************************************************************************
*                       txData_DistributorTxEvent                          *
****************************************************************************
* DESCRIPTION:
*
*
* INPUTS:
*
*
*
* OUTPUT:
*
* RETURNS:
*
***************************************************************************/
static VOID txData_DistributorTxEvent(txData_t *pTxData,UINT16 Mask,int DataLen)
{
   if(pTxData->TxEventDistributor)
     DistributorMgr_EventCall(pTxData->TxEventDistributor,Mask,DataLen);

}

/***************************************************************************
*                       txData_RegNotif                                    *
****************************************************************************/
TI_HANDLE txData_RegNotif(TI_HANDLE hTxData,UINT16 EventMask,GeneralEventCall_t CallBack,TI_HANDLE context,UINT32 Cookie)
{
    txData_t *pTxData = (txData_t *)hTxData;
    if (!hTxData)
        return NULL;
    return DistributorMgr_Reg(pTxData->TxEventDistributor,EventMask,(TI_HANDLE)CallBack,context,Cookie);
}


/***************************************************************************
*                       txData_AddToNotifMask                              *
****************************************************************************/
TI_STATUS txData_AddToNotifMask(TI_HANDLE hTxData,TI_HANDLE Notifh,UINT16 EventMask)
{
    txData_t *pTxData = (txData_t *)hTxData;
    if (!hTxData)
        return NOK;
    return DistributorMgr_AddToMask(pTxData->TxEventDistributor,Notifh,EventMask);
}


/***************************************************************************
*                       TxData_UnRegNotif                                  *
****************************************************************************/
TI_STATUS txData_UnRegNotif(TI_HANDLE hTxData,TI_HANDLE RegEventHandle)
{
    TI_STATUS status;
    txData_t *pTxData = (txData_t *)hTxData;

    if (!hTxData)
        return NOK;

    status = DistributorMgr_UnReg(pTxData->TxEventDistributor,RegEventHandle);
    return status;
}

/****************************************************************************
 *                      txData_SetTxDelayCounters()
 ****************************************************************************
 * DESCRIPTION:          Update transmission path delay counters.
*
* INPUTS:       hTxData - the object
*               txQid - the queue to count delay for
*               pTxCompleteAttr - struct containing the necessary FW delay data
*               driverDelay - the time consumed in driver for packet transmission
*
* OUTPUT:
*
* RETURNS:
 ****************************************************************************/
static void txData_SetTxDelayCounters( TI_HANDLE hTxData, UINT32 txQid, txCompleteAttr_t *pTxCompleteAttr, UINT32 driverDelay )
{
    txData_t *pTxData = (txData_t *)hTxData;
    int     rangeIndex;
    UINT32  totalTxDelayMs;

    /* Add 1 to the total time so that Total time will always be greater than fwHandlingTime */
    totalTxDelayMs = driverDelay + (pTxCompleteAttr->fwHandlingTime / 1000) + 1;

    /* Increment the delay range counter that the current packet Tx delay falls in. */
    for (rangeIndex = TX_DELAY_RANGE_MIN; rangeIndex <= TX_DELAY_RANGE_MAX; rangeIndex++)
    {
        if ( (totalTxDelayMs >= txDelayRangeStart[rangeIndex]) &&
             (totalTxDelayMs <= txDelayRangeEnd  [rangeIndex]) )
        {
            pTxData->txDataCounters[ txQid ].txDelayHistogram[ rangeIndex ]++;
            pTxData->txDataReportedCounters[ txQid ].txDelayHistogram[ rangeIndex ]++;
            break;
        }
    }
    
    /* Update total delay and MAC delay sums and packets number for average delay calculation. */
    if (pTxData->txDataCounters[ txQid ].SumTotalDelayMs < 0x7FFFFFFF) /* verify we are not close to the edge. */
    {
        pTxData->txDataCounters[ txQid ].NumPackets++; 
        pTxData->txDataCounters[ txQid ].SumTotalDelayMs += totalTxDelayMs;   
        pTxData->txDataCounters[ txQid ].SumFWDelayUs += pTxCompleteAttr->fwHandlingTime;
        pTxData->txDataCounters[ txQid ].SumMacDelayUs += pTxCompleteAttr->mediumDelay;
    }
    else  /* If we get close to overflow, restart average accumulation. */
    {
        pTxData->txDataCounters[ txQid ].NumPackets = 1; 
        pTxData->txDataCounters[ txQid ].SumTotalDelayMs = totalTxDelayMs;   
        pTxData->txDataCounters[ txQid ].SumFWDelayUs = pTxCompleteAttr->fwHandlingTime;
        pTxData->txDataCounters[ txQid ].SumMacDelayUs = pTxCompleteAttr->mediumDelay;
    }
    pTxData->txDataReportedCounters[ txQid ].NumPackets++; 
    pTxData->txDataReportedCounters[ txQid ].SumTotalDelayMs += totalTxDelayMs;   
    pTxData->txDataReportedCounters[ txQid ].SumFWDelayUs += pTxCompleteAttr->fwHandlingTime;
    pTxData->txDataReportedCounters[ txQid ].SumMacDelayUs += pTxCompleteAttr->mediumDelay;
}

/***************************************************************************
*                       txData_UpdateTxCounters                            *
****************************************************************************
* DESCRIPTION:  free the transmitted msdu
*
* INPUTS:       hTxData - the object
*               pTxCompletAttr - all atributes passed along with the TX 
*                                complete indication
*
* OUTPUT:
*
* RETURNS:
***************************************************************************/
void txData_UpdateTxCounters( TI_HANDLE hTxData, txCompleteAttr_t *pTxCompleteAttr )
{
    txData_t *pTxData = (txData_t*)hTxData;
    txPacketIdAttr_t *pPacketId = (txPacketIdAttr_t *)(pTxCompleteAttr->packetId);
    UINT16 EventMask = 0;
    UINT32 dataLen, TxQid = pPacketId->txQid;
    UINT32 retryHistogramIndex;

    switch ( pTxCompleteAttr->status )
    {
    case SEND_COMPLETE_SUCCESS:
        /* update the retry histogram */
        retryHistogramIndex = (pTxCompleteAttr->ackFailures >= TX_RETRY_HISTOGRAM_SIZE ? 
                               TX_RETRY_HISTOGRAM_SIZE - 1 :
                               pTxCompleteAttr->ackFailures);
        pTxData->txDataCounters[ TxQid ].RetryHistogram[ retryHistogramIndex ]++;

        /* update delay histogram */
        txData_SetTxDelayCounters( hTxData, TxQid, pTxCompleteAttr, pPacketId->driverHandlingTime );

        if ( (TRUE == pPacketId->bDataMsdu) && (pTxData->txDataQosParams.headerConverMode == QOS_CONVERT) )
        {
            dataLen = pPacketId->msduDataLen - (WLAN_WITH_SNAP_QOS_HEADER_MAX_SIZE - ETHERNET_HDR_LEN);
        }
        else
        {
            dataLen = pPacketId->msduDataLen - (WLAN_WITH_SNAP_HEADER_MAX_SIZE - ETHERNET_HDR_LEN);
        }

        if ( IsMacAddressDirected( &(pPacketId->destinationMac) ) )
        {
            /* Directed frame */
            pTxData->txDataCounters[TxQid].DirectedFramesXmit++;
            pTxData->txDataCounters[TxQid].DirectedBytesXmit += dataLen;
            EventMask |= DIRECTED_BYTES_XFER;
            EventMask |= DIRECTED_FRAMES_XFER;
        }
        else if ( IsMacAddressBroadcast( &(pPacketId->destinationMac) ) )
        {
            /* Broadcast frame */
            pTxData->txDataCounters[TxQid].BroadcastFramesXmit++;
            pTxData->txDataCounters[TxQid].BroadcastBytesXmit += dataLen;
            EventMask |= BROADCAST_BYTES_XFER;
            EventMask |= BROADCAST_FRAMES_XFER;
        }
        else
        {
            /* Multicast Address */
            pTxData->txDataCounters[TxQid].MulticastFramesXmit++;
            pTxData->txDataCounters[TxQid].MulticastBytesXmit += dataLen;
            EventMask |= MULTICAST_BYTES_XFER;
            EventMask |= MULTICAST_FRAMES_XFER;
        }
        pTxData->txDataCounters[TxQid].XmitOk++;
        EventMask |= XFER_OK;

        /* update the max consecutive retry failures (if needed) */
        if (pTxData->currentConsecutiveRetryFail > pTxData->txDataCounters[ TxQid ].MaxConsecutiveRetryFail)
        {
            pTxData->txDataCounters[TxQid].MaxConsecutiveRetryFail = pTxData->currentConsecutiveRetryFail;
        }
        pTxData->currentConsecutiveRetryFail = 0;

        txData_DistributorTxEvent( pTxData, EventMask, dataLen );
        break;

    case SEND_COMPLETE_RETRY_EXCEEDED:

        pTxData->txDataCounters[ TxQid ].RetryFailCounter++;
        pTxData->currentConsecutiveRetryFail++;
        pTxData->txDataReportedCounters[ TxQid ].OtherFailCounter++;
        break;

    case SEND_COMPLETE_LIFETIME_EXCEEDED:
        pTxData->txDataCounters[ TxQid ].TxTimeoutCounter++;
        pTxData->txDataReportedCounters[ TxQid ].OtherFailCounter++;
        break;

    case SEND_COMPLETE_NO_LINK:
        pTxData->txDataCounters[ TxQid ].NoLinkCounter++;
        pTxData->txDataReportedCounters[ TxQid ].OtherFailCounter++;
        break;

    case SEND_COMPLETE_MAC_CRASHED: /* curently not used */
    default:
        pTxData->txDataCounters[ TxQid ].OtherFailCounter++;
        pTxData->txDataReportedCounters[ TxQid ].OtherFailCounter++;
        break;
    }
}

static void txData_startVadTimer(TI_HANDLE hTxData, UINT16 voiceDuration)
{
    txData_t *pTxData = (txData_t*)hTxData;

    if (FALSE == pTxData->bVadTimerEnabled)
    {
        WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
            ("txData_startVadTimer: (voiceDuration is %d) .....................\n", voiceDuration));
	pTxData->vadTimerDuration = voiceDuration;
	pTxData->bVadTimerEnabled = TRUE;
	os_timerStart(pTxData->hOs, pTxData->pVadTimer, voiceDuration, TRUE);
		
    }
    else
    {
        WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
    	    ("txData_startVadTimer: nothing done. VAD is already started ........\n", voiceDuration));
    }
}

static void txData_stopVadTimer(TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t*)hTxData;

    if (pTxData->bVadTimerEnabled)
    {
    	WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG, ("txData_stopVadTimer ...\n"));
	os_timerStop(pTxData->hOs, pTxData->pVadTimer);
	pTxData->bVadTimerEnabled = FALSE;
    }
}

static void txData_setVadTimer(TI_HANDLE hTxData, BOOL vadEnabled, UINT16 duration)
{
    txData_t *pTxData = (txData_t*)hTxData;

    WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG, ("txData_setVadTimer (%d, %d)\n", vadEnabled, duration));
    if (vadEnabled)
	txData_startVadTimer(hTxData, duration);
    else
	txData_stopVadTimer(hTxData);
}

static void txData_resetVadTimer(TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t*)hTxData;
//    UINT32 timeStamp;

    if (pTxData->bVadTimerEnabled)
    {
/*
        timeStamp = os_timeStampMs(pTxData->hOs);
	printk("resetVadTimer: timestamp = %d\n", timeStamp);
*/
	os_timerStop(pTxData->hOs, pTxData->pVadTimer);
	os_timerStart(pTxData->hOs, pTxData->pVadTimer, pTxData->vadTimerDuration, TRUE);
    }
}

static void txData_vadTimeout(TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t*)hTxData;
//    UINT32 timeStamp;
/*
    timeStamp = os_timeStampMs(pTxData->hOs);
    printk("vadTimeout: timestamp = %d\n", timeStamp);
*/
    txData_sendVadFrame (pTxData, QOS_AC_VO);
}

/***************************************************************************
*                       txData_startTxSchedulerFromTimer                   *
****************************************************************************
* DESCRIPTION:
*
* INPUTS:
*
* OUTPUT:
*
* RETURNS:
***************************************************************************/
static void txData_startTxSchedulerFromTimer(TI_HANDLE hTxData)
{
    WLAN_REPORT_DEBUG_TX(((txData_t *)hTxData)->hReport, 
        ("in txData_startTxSchedulerFromTimer.....................\n"));

    txData_startTxScheduler(hTxData);
}

/***********************************************************************
 *                        txData_sendNullFrame
 ***********************************************************************
DESCRIPTION:    Send Null frame Function.
                The function does the following:
                -   Builds Null Data Frame with PS bit set to On or Off.
                -   Allocates MSDU frame.
                -   Sends the frame.

INPUT:      hTxData         -   Tx Data Pointer.
            powerSaveMode   -   Indicates if to switch the Power Save
                                mode to On or Off.
            module          -   The calling module.

OUTPUT:     None

RETURN:     OK on success, NOK otherwise
************************************************************************/
TI_STATUS txData_sendNullFrame(TI_HANDLE hTxData,
                               BOOL powerSaveOn,
                               allocatingModule_e module)
{
    TI_STATUS       status;
    mem_MSDU_T      *pMsdu;
    paramInfo_t     daParam, saParam;
    dot11_header_t  *pFrame; /* Note : there is no body for null frame */
    txData_t        *pTxData = (txData_t *)hTxData;

    /* Getting new msdu */
    status = wlan_memMngrAllocMSDU(pTxData->hMemMngr, &pMsdu, WLAN_HDR_LEN + TX_TOTAL_OFFSET_BEFORE_DATA, module);
    if (status != OK)
        return NOK;

    pFrame = (dot11_header_t*)(pMsdu->firstBDPtr->data + TX_TOTAL_OFFSET_BEFORE_DATA);

    /* Setting the Frame Control with Data frame type and Null frame sub type*/
    pFrame->fc = 0;
    pFrame->fc |= DOT11_FC_DATA_NULL_FUNCTION;
    pFrame->fc |= DOT11_FC_TO_DS;

    /* setting the Power Save bit in the Frame control field*/
    if(powerSaveOn == TRUE)
        pFrame->fc |= (0x1 << DOT11_FC_PWR_MGMT_SHIFT);

    daParam.paramType = CTRL_DATA_CURRENT_BSSID_PARAM;
    status = ctrlData_getParam(pTxData->hCtrlData, &daParam);
    if (status != OK)
    {
        wlan_memMngrFreeMSDU(pTxData->hMemMngr, pMsdu->handle);
        return NOK;
    }

    /* copy destination mac address */
    MAC_COPY(pTxData->hOs, (&pFrame->address3), (&daParam.content.ctrlDataCurrentBSSID));

    saParam.paramType = CTRL_DATA_MAC_ADDRESS;
    status = ctrlData_getParam(pTxData->hCtrlData, &saParam);
    if (status != OK)
    {
        wlan_memMngrFreeMSDU(pTxData->hMemMngr, pMsdu->handle);
        return NOK;
    }

    /* copy source mac address */
    MAC_COPY(pTxData->hOs, (&pFrame->address2), (&saParam.content.ctrlDataCurrentBSSID));

    /* copy BSSID (destination mac address) */
    MAC_COPY(pTxData->hOs, (&pFrame->address1), (&daParam.content.ctrlDataCurrentBSSID));

    /* Update MSDU parameters */
    pMsdu->headerLen = pMsdu->dataLen =  WLAN_HDR_LEN;
    pMsdu->firstBDPtr->length = pMsdu->dataLen + TX_TOTAL_OFFSET_BEFORE_DATA;

    /* send the packet to the TX */
    pMsdu->qosTag = 0;
    pMsdu->txFlags |= TX_DATA_NULL_MSDU;

    status = txData_txSendMsdu(hTxData, pMsdu);

    return status;
}

/***********************************************************************
 *                        txData_sendVadFrame
 ***********************************************************************
DESCRIPTION:    Send a polling frame to retrieve downlink traffic from
                the AP. Activated by the voice application when there is 
                  no uplink traffic during a voice call.
                The polling is either PS-Poll (for legacy PS) or QoS-Null
                (for UPSD).
                Null frame is currently added after PS-Poll to trigger
                the triggered-scan which is only triggered by data frames!
                Note that currently the acID parameter is ignored to insure this
                function is not activated for other ACs than Voice (to add this
                flexibility the txData_txSendMsdu() function should be updated.

INPUT:      acID - Currently not used!  Supporting only AC_VO!

OUTPUT:     None

RETURN:     OK on success, NOK otherwise
************************************************************************/

TI_STATUS txData_sendVadFrame(TI_HANDLE hTxData, UINT8 acID)
{
    mem_MSDU_T *pMsdu;
    TI_STATUS  status;
    txData_t   *pTxData = (txData_t *)hTxData;
    paramInfo_t param;

    if (pTxData->txDataPortStatus != OPEN)
    {
        WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
                                ("txData_sendVadFrame(): While port status is %d\n", pTxData->txDataPortStatus ));
        return NOK;
     }

    param.paramType = QOS_MNGR_ACTIVE_PROTOCOL;
    status = qosMngr_getParams(pTxData->hQosMngr, &param);

    /* For WME - send QoS-Null-Data. */
    if (param.content.qosSiteProtocol ==  WME)  
    {
        /* Send QoS-Null frame to retrieve downlink packets and to trigger the triggered-scan. */
        /* Note: If VO mode is PSPOLL, the txData_txSendMsdu() will add the PS-Poll frame.  */
        if((status = txData_buildQosNullDataFrame(pTxData,&pMsdu,wmeAcToUpIndex[QOS_AC_VO])) == OK)
            status = txData_txSendMsdu(hTxData, pMsdu); 
        
        WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
            ("txData_sendVadFrame(): WME site...Sending QoS-Null frame, Status=%d\n", status));
    }
    
    /* For non-WME - send PS-Poll plus Null-Data. */
    else    
    {
        /* Send PS-Poll to retrieve downlink packets. */
        if((status = txData_getPsPollFrame(pTxData,&pMsdu)) == OK)
            status = txData_txSendMsdu(hTxData, pMsdu);

        /* Send also a null frame to trigger the triggered-scan in the FW. */
        /* Note: Needed because the scan is triggered only by data frames, so PS-Poll is not enough!. */
        /* Note: The txData_txSendMsdu() won't send another PS-Poll before the null frame since 
                 pMsdu->qosTag is set to 0 for null (and not voice). */
        status = txData_sendNullFrame(hTxData, FALSE, TX_MODULE);
        
        WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
            ("txData_sendVadFrame(): Non-WME site...Sending PS-Poll and Null frame, Status=%d\n", status));
    }
    
    return status;
}


/***********************************************************************
 *                        txData_getPsPollFrame
 ***********************************************************************
DESCRIPTION:    builds PS POLL frame Function.
                The function does the following:
                -   Builds PS POLL Control Frame with PS bit set to On .
                -   Allocates MSDU frame.
                -   Sends the frame.

----------------------------
PS Poll - 802.11 Mac Header
----------------------------
  Version              0
  Type                 %01          (Control)
  Subtype              %1010        (PS Poll)
  Frame Control Flag   %00010000    (Power Management Bit set -> Power Save mode)
  Assoc ID             association ID
  BSSID                MAC addr of BSSID
  Transmitter          MAC addr of sender
----------------------------

typedef struct
{
  UINT16        fc;
  UINT16        AID;
  macAddress_t  BSSID;
  macAddress_t  TA;
} dot11_PsPollFrameHeader_t;


INPUT:

OUTPUT:     None

RETURN:     OK on success, NOK otherwise
************************************************************************/
TI_STATUS txData_getPsPollFrame(TI_HANDLE hTxData,mem_MSDU_T **pMsduPsPoll)
{
    TI_STATUS       status = OK;
    UINT32          timeStamp ;
    paramInfo_t     daParam, saParam;
    whalParamInfo_t whalParam;
    dot11_PsPollFrameHeader_t   *pFrame; /* Note : there is no body for null frame */
    txData_t *pTxData = (txData_t *)hTxData;
    mem_MSDU_T      *pMsdu;
    /* Getting new msdu */
    status = wlan_memMngrAllocMSDU(pTxData->hMemMngr, &pMsdu, WLAN_HDR_LEN + TX_TOTAL_OFFSET_BEFORE_DATA, TX_MODULE);
    if (status != OK)
    {
        return NOK;
    }


    pFrame = (dot11_PsPollFrameHeader_t*)(pMsdu->firstBDPtr->data + TX_TOTAL_OFFSET_BEFORE_DATA);

    /*
    **   Building the Frame Control word (16 bits)
    ** ---------------------------------------------
    */
    pFrame->fc = 0;

    /*
    ** Type = Control
    ** SubType = Power Save (PS) POLL,  */
    pFrame->fc |= DOT11_FC_PS_POLL;

    /*
    ** setting the Power Management bit in the Frame control field
    ** to be "Power Save mode"
    */
    pFrame->fc |= (0x1 << DOT11_FC_PWR_MGMT_SHIFT);

    /*
    **   Association ID
    ** -----------------
    */
    whalParam.paramType = HAL_CTRL_AID_PARAM;
    status = whalCtrl_GetParam (pTxData->hWhalCtrl, &whalParam) ;
    if (status != OK)
    {
        wlan_memMngrFreeMSDU(pTxData->hMemMngr, pMsdu->handle);
        return NOK;
    }
    /* AID should have its two MSB bit Set to "1"*/
    pFrame->AID = whalParam.content.halCtrlAid | 0xC000;
    WLAN_REPORT_WARNING(pTxData->hReport, TX_DATA_MODULE_LOG,
            (" AID 4 = %d  \n", (whalParam.content.halCtrlAid | 0xC000)));

    /*
    **   BSSID
    ** ---------
    */
    daParam.paramType = CTRL_DATA_CURRENT_BSSID_PARAM;
    status = ctrlData_getParam(pTxData->hCtrlData, &daParam);
    if (status != OK)
    {
        wlan_memMngrFreeMSDU(pTxData->hMemMngr, pMsdu->handle);
        return NOK;
    }
    /* copy destination mac address */
    MAC_COPY(pTxData->hOs, (&pFrame->BSSID), (&daParam.content.ctrlDataCurrentBSSID));




    /*
    **  TA - Transmiter (MAC) Address
    ** -------------------------------
    */
    saParam.paramType = CTRL_DATA_MAC_ADDRESS;
    status = ctrlData_getParam(pTxData->hCtrlData, &saParam);
    if (status != OK)
    {
        wlan_memMngrFreeMSDU(pTxData->hMemMngr, pMsdu->handle);
        return NOK;
    }
   /* copy source mac address */
    MAC_COPY(pTxData->hOs, (&pFrame->TA), (&saParam.content.ctrlDataCurrentBSSID));

    timeStamp = os_timeStampMs(pTxData->hOs);

    WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
            (" txData_sendPsPollFrame() : time = %d insert PS_POLL frame to Driver queue \n", timeStamp));

    /* Update MSDU parameters */
    pMsdu->headerLen = pMsdu->dataLen =  sizeof(dot11_PsPollFrameHeader_t);
    pMsdu->firstBDPtr->length = pMsdu->dataLen + TX_TOTAL_OFFSET_BEFORE_DATA;

    pMsdu->txFlags |= TX_DATA_PS_POLL;


    if (status == OK)
        WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
        ("in txData_sendPsPollFrame: enter PS_POLL to queue\n"));
    else
        WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
        ("in txData_sendPsPollFrame: didn't enter PS_POLL to queue\n"));

    *pMsduPsPoll = pMsdu;

    return status;
}


/***********************************************************************
 *                        txData_updateUsedTime
 ***********************************************************************
DESCRIPTION:    This function is called for every txComplete in order
                to update the transmisssion time.

INPUT:          hTxData - handale to the ts data object
                qNum    - the queue that the frame transmitted from
                usedTime - the time of the transmission (in microseconds)

OUTPUT:     None

RETURN:     OK on success
************************************************************************/
TI_STATUS txData_updateUsedTime(TI_HANDLE hTxData, UINT32 qNum, UINT16 usedTime)
{
    txData_t *pTxData = (txData_t *)hTxData;

    /* addd the used time for the specific queue */
    pTxData->dataMsduListArr[qNum]->totalUsedTime += usedTime;

    return OK;
}


/***********************************************************************
 *                        txData_calcCreditFromTimer
 ***********************************************************************
DESCRIPTION:    This function is called when credit calculation timer
                is expired. it calculate the credit for the admission ctrl
                credit algorithm


INPUT:          hTxData - handle to the ts data object

OUTPUT:     None

RETURN:     void
************************************************************************/
static void txData_calcCreditFromTimer(TI_HANDLE hTxData)
{
    UINT32 qNum;
    OS_802_11_THRESHOLD_CROSS_INDICATION_PARAMS mediumTimeCross;
    txData_t *pTxData = (txData_t *)hTxData;
    INT32       prevCredit;
    INT32       highCreditThreshold;
    INT32       lowCreditThreshold;
    MsduList_t  *pMsduList;
    INT32       usageRatio;
    INT32       currUsage;
    INT32       prevUsage;

    /* get current time stamp */
    UINT32 currentTimeStamp = os_timeStampMs(pTxData->hOs);
    
    
    for(qNum = 0 ; qNum< MAX_NUM_OF_TX_QUEUES ; qNum++)
    {
        pMsduList = pTxData->dataMsduListArr[qNum];

        /* check if this queue is under admission ctrl opration */
        if(pMsduList->mediumTime == 0)
        {
            WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG, 
                (" txData_calcCreditFromTimer() :qNum = %d mediumTime = 0 \n",qNum));
            
            continue;
        }

        /* in case of wraparound */
        if(currentTimeStamp < pMsduList->lastTimeStamp)
            pMsduList->lastTimeStamp = 0;
        
        /* store prev credit */
        prevCredit = pMsduList->credit;

        /* Calculate the medium usage ratio:    totalUsedTime / mediumTime * 1000
           Note that since the totalUsedTime is in usec and not msec we don't multiply by 1000.  */
        usageRatio = pMsduList->totalUsedTime / pMsduList->mediumTime;

        /* calculate credit */
        pMsduList->credit = pMsduList->credit + (currentTimeStamp - pMsduList->lastTimeStamp) - usageRatio;
        
        /* update last time stamp */
        pMsduList->lastTimeStamp = currentTimeStamp;

        /* check if credit exceeds above mediumTime or below -mediumTime */
        if (pMsduList->credit > (INT32)(pMsduList->mediumTime) )
        {
            /* in case of credit is big than mediumTime -> set credit to medium time */
            pMsduList->credit = pMsduList->mediumTime;
            pMsduList->enableTransmissionTime = 0;
        }
        
        /* If credit is lower than -mediumTime we exceed the admitted time. */
        else if (pMsduList->credit <= (INT32)(0 - pMsduList->mediumTime))
        {
            /* Only delay transmission on this AC if the "delay" registry key is TRUE */
            if(pTxData->admCtrlDelayDueToMediumTimeOverUsage == TRUE)
                pMsduList->enableTransmissionTime = (0 - pMsduList->credit) - pMsduList->mediumTime;
        }

       WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
          ("credit = %d  | TotalUsedTime = %d  | enableTransmissionTime = %d\n",
          pMsduList->credit, pMsduList->totalUsedTime/1000, pMsduList->enableTransmissionTime));

        /* Check medium-usage threshold cross events */
        /*********************************************/
        /*
         * The medium-usage events are defined as follows:
         * The high threshold triggers event only when crossed upward (traffic increased above threshold).
         * The low threshold triggers event only when crossed downward (traffic decreased below threshold).
         * Thus, the two thresholds provide hysteresis and prevent multiple triggering.
         * The high threshold should be greater than the low threshold. 
         */

        highCreditThreshold = (INT32)((pMsduList->mediumTime)*(pMsduList->highMediumUsageThreshold)/100); 
        lowCreditThreshold  = (INT32)((pMsduList->mediumTime)*(pMsduList->lowMediumUsageThreshold)/100);

        /* The credit is getting more negative as we get closer to the medium usage limit, so we invert
             it before comparing to the thresholds (lower credit means higher usage). */
        currUsage = -pMsduList->credit;
        prevUsage = -prevCredit;

        /* crossing below the low threshold */
        if ( (currUsage < lowCreditThreshold) && (prevUsage >= lowCreditThreshold) )
        {
            /* send event */
            mediumTimeCross.uAC = txData_GetAcIdFromQid( hTxData, qNum );
            mediumTimeCross.uHighOrLowThresholdFlag = (UINT32)LOW_THRESHOLD_CROSS;
            mediumTimeCross.uAboveOrBelowFlag = (UINT32)CROSS_BELOW;

            EvHandlerSendEvent(pTxData->hEvHandler, IPC_EVENT_MEDIUM_TIME_CROSS, (UINT8 *)&mediumTimeCross,sizeof(OS_802_11_THRESHOLD_CROSS_INDICATION_PARAMS));
            WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
                ("crossed below low threshold !!! prevUsage = %d, currUsage = %d, lowCreditThreshold = %d\n",
                prevUsage, currUsage, lowCreditThreshold));
        }
        
        /* crossing above the high threshold */
        else if ( (currUsage > highCreditThreshold) && (prevUsage <= highCreditThreshold) )
        {
            /* send event */
            mediumTimeCross.uAC = txData_GetAcIdFromQid( hTxData, qNum );
            mediumTimeCross.uHighOrLowThresholdFlag = (UINT32)HIGH_THRESHOLD_CROSS;
            mediumTimeCross.uAboveOrBelowFlag = (UINT32)CROSS_ABOVE;

            EvHandlerSendEvent(pTxData->hEvHandler, IPC_EVENT_MEDIUM_TIME_CROSS, (UINT8 *)&mediumTimeCross,sizeof(OS_802_11_THRESHOLD_CROSS_INDICATION_PARAMS));
            WLAN_REPORT_INFORMATION(pTxData->hReport, TX_DATA_MODULE_LOG,
                ("crossed above high threshold !!! prevUsage = %d, currUsage = %d, highCreditThreshold = %d\n",
                prevUsage, currUsage, highCreditThreshold));
        }

        /* reset totalUsedTime */
        pMsduList->totalUsedTime = 0;
        
    }
    
}


/***********************************************************************
 *                        txData_setAdmisionCtrlParams
 ***********************************************************************
DESCRIPTION:    This function is called for add/delete a tspec in order
                to update parameters.

INPUT:          hTxData - handale to the ts data object
                acID - the AC of the tspec
                mediumTime  - tha alocated medium time for this UP 
                minimumPHYRate - the min phy rate to send a packet of this UP
                admFlag - indicate if the its addition or deletion of tspec 

OUTPUT:     None

RETURN:     void
************************************************************************/
TI_STATUS txData_setAdmisionCtrlParams(TI_HANDLE hTxData, 
                                       UINT8 acID,
                                       UINT16 mediumTime, 
                                       UINT32 minimumPHYRate,
                                       BOOL admFlag)
{
    UINT8   queueIndex;
    UINT8   i;
    txData_t *pTxData = (txData_t *)hTxData;
    MsduList_t  *pMsduList;

    /* find queue from AC */
    queueIndex = pTxData->txDataAcTrfcCtrl[acID].QueueIndex;
    
    pMsduList = pTxData->dataMsduListArr[queueIndex];

    if(admFlag == TRUE) 
    {
        /* tspaec added */
        pMsduList->mediumTime = mediumTime;
        
        /* in case of medium time>0 set relevant parameters to the credit algo calculation */
        pMsduList->admissionState = AC_ADMITTED;

        /* enable admission algo for this queue */
        pMsduList->useAdmissionAlgo = TRUE;

        pMsduList->lastTimeStamp = os_timeStampMs(pTxData->hOs);
        pMsduList->enableTransmissionTime = 0;
        pMsduList->credit = mediumTime;
    }
    else
    {
        /* tspaec deleted */
        pMsduList->mediumTime = 0;
        
        /* in case of medium time=0 reset relevant parameters */
        pMsduList->admissionState = AC_NOT_ADMITTED;

        pMsduList->useAdmissionAlgo = FALSE;
        pMsduList->lastTimeStamp = 0;
        pMsduList->enableTransmissionTime = 0;
        pMsduList->credit = 0;

    }
    
    /* If the timer was not enabled in registry than we will never set it */
    if ( pTxData->bCreditCalcTimerEnabled )
    {
        /* enable disable credit calculation timer */
        for(i = 0 ; i < MAX_NUM_OF_TX_QUEUES ; i++)
        {
            if(pTxData->dataMsduListArr[i]->useAdmissionAlgo == TRUE)
            {
                if(pTxData->bCreditCalcTimerRunning == FALSE)
                {
                    pTxData->bCreditCalcTimerRunning = TRUE;
                    os_timerStart(pTxData->hOs, pTxData->pCreditTimer, pTxData->creditCalculationTimeout, TRUE);
                }
                
                return OK;
            }
        }
        /* in all queues useAdmissionAlgo is not TRUE */
        if ( pTxData->bCreditCalcTimerRunning )
        {
            os_timerStop(pTxData->hOs, pTxData->pCreditTimer);
            pTxData->bCreditCalcTimerRunning = FALSE;
        }
    }

    return OK;

}


/***********************************************************************
 *                        txData_setPsVoiceDeliveryMode
 ***********************************************************************
DESCRIPTION:    This function is called for add/delete a tspec in order
                to set the PS mode for a specific UP

INPUT:          hTxData - handale to the ts data object
                userPriority - the user priority of the tspec
                PsMode  - tha PS mode for the specific UP

OUTPUT:     None

RETURN:     OK on success, NOK otherwise
************************************************************************/
TI_STATUS txData_setPsVoiceDeliveryMode(TI_HANDLE hTxData, PSScheme_e   PsMode)
{
    txData_t *pTxData = (txData_t *)hTxData;
    pTxData->txDataAcTrfcCtrl[QOS_AC_VO].PsMode = PsMode;
    return OK;
}

/***********************************************************************
 *                        txData_setMediumUsageThresholds
 ***********************************************************************
DESCRIPTION:    This function is called in order to set the threshold
                for the medium time usage

INPUT:          hTxData - handale to the ts data object
                acID - the AC
                highMediumUsageThreshold - high threshold
                lowMediumUsageThreshold - lowhreshold

OUTPUT:     None

RETURN:     OK on success, NOK otherwise
************************************************************************/
TI_STATUS txData_setMediumUsageThresholds(TI_HANDLE     hTxData,
                                      UINT8             acID,
                                      INT32             highMediumUsageThreshold,
                                      INT32             lowMediumUsageThreshold)
{
    txData_t *pTxData = (txData_t *)hTxData;

    UINT8 queueIndex;

    /* validate AC */
    if(acID >= MAX_NUM_OF_AC)
        return NOK;

    /* find queu from ac */
    queueIndex = pTxData->txDataAcTrfcCtrl[acID].QueueIndex;

    /* set threshold */
    pTxData->dataMsduListArr[queueIndex]->lowMediumUsageThreshold = lowMediumUsageThreshold;
    pTxData->dataMsduListArr[queueIndex]->highMediumUsageThreshold = highMediumUsageThreshold;

    return OK;

}

/***********************************************************************
 *                        txData_isQueueUseMediumTime
 ***********************************************************************
DESCRIPTION:    

INPUT:          hTxData - handale to the ts data object

OUTPUT:     None

RETURN:     OK on success, NOK otherwise
************************************************************************/

BOOL txData_isQueueUseMediumTime(TI_HANDLE hTxData, UINT8 qNum)
{
    txData_t *pTxData = (txData_t *)hTxData;

    if(pTxData->dataMsduListArr[qNum]->mediumTime == 0)
        return FALSE;
    else
        return TRUE;

}



void Test_HeaderConvertion(TI_HANDLE hTxData, mem_MSDU_T *pMsdu)
{
    txData_t *pTxData = (txData_t *)hTxData;

    print_MsduDataHeader(pTxData->hMemMngr, pMsdu);

    txData_convertEthToWlanHeader( pTxData, pMsdu );

    print_MsduDataHeader(pTxData->hMemMngr, pMsdu);

}


#ifdef TI_DBG
void txData_printTxBlock(TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t *)hTxData;
    int i;

    WLAN_OS_REPORT(("hCtrlData = 0x%X\n", pTxData->hCtrlData));
    WLAN_OS_REPORT(("hTnetwDrv = 0x%X\n", pTxData->hTnetwDrv));
    WLAN_OS_REPORT(("hOs = 0x%X\n", pTxData->hOs));
    WLAN_OS_REPORT(("hReport = 0x%X\n", pTxData->hReport));
    WLAN_OS_REPORT(("hMemMngr = 0x%X\n", pTxData->hMemMngr));
    WLAN_OS_REPORT(("pSchedulerTimer = 0x%X\n", pTxData->pSchedulerTimer));

    WLAN_OS_REPORT(("hCriticalSectionProtect = 0x%X\n", pTxData->hCriticalSectionProtect));

    WLAN_OS_REPORT(("txDataPortStatus = %d\n", pTxData->txDataPortStatus));
    WLAN_OS_REPORT(("txDataCurrentPrivacyInvokedMode = %d\n", pTxData->txDataCurrentPrivacyInvokedMode));
    WLAN_OS_REPORT(("txDataEapolEncryptionStatus = %d\n", pTxData->txDataEapolEncryptionStatus));

    WLAN_OS_REPORT(("txDataIsSchedulerInWork = %d\n", pTxData->txDataIsSchedulerInWork));

    WLAN_OS_REPORT(("txDataNumOfQueues = %d\n", pTxData->txDataNumOfQueues));
    WLAN_OS_REPORT(("mngMsduList = 0x%X\n", pTxData->mngMsduList));
    for(i=0 ; i < MAX_NUM_OF_TX_QUEUES; i++ )
    {
        WLAN_OS_REPORT(("dataMsduList %d = 0x%X\n",i, pTxData->dataMsduListArr[i]));
    }

}


/*
void printFullMsduList(MsduList_t *this);
void printMsduList(MsduList_t *this);
*/
void txData_printDataMsduList(TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t *)hTxData;
    int i;

    for(i=0 ; i < MAX_NUM_OF_TX_QUEUES; i++ )
    {
        WLAN_OS_REPORT(("List : %d \n", i));
        printMsduList(pTxData->dataMsduListArr[i]);
    }


}

void txData_fullPrintDataMsduList(TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t *)hTxData;
    int i;

    for(i=0 ; i < MAX_NUM_OF_TX_QUEUES; i++ )
    {
        WLAN_OS_REPORT(("List : %d \n", i));
        printFullMsduList(pTxData->dataMsduListArr[i]);
    }
}

void txData_printMgmtMsduList(TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t *)hTxData;

    printMsduList(pTxData->mngMsduList);


}

void txData_fullPrintMgmtMsduList(TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t *)hTxData;

    printFullMsduList(pTxData->mngMsduList);
}


void txData_printTxCounters(TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t *)hTxData;
    int TxQid;

    for (TxQid = 0; TxQid < MAX_NUM_OF_TX_QUEUES; TxQid++)
    {
        WLAN_OS_REPORT(("Tx Queue %d:\n", TxQid));
        WLAN_OS_REPORT(("===========\n"));
        WLAN_OS_REPORT(("XmitOk = %d\n", pTxData->txDataCounters[TxQid].XmitOk));
        WLAN_OS_REPORT(("DirectedBytesXmit = %d\n", pTxData->txDataCounters[TxQid].DirectedBytesXmit));
        WLAN_OS_REPORT(("DirectedFramesXmit = %d\n", pTxData->txDataCounters[TxQid].DirectedFramesXmit));
        WLAN_OS_REPORT(("MulticastBytesXmit = %d\n", pTxData->txDataCounters[TxQid].MulticastBytesXmit));
        WLAN_OS_REPORT(("MulticastFramesXmit = %d\n", pTxData->txDataCounters[TxQid].MulticastFramesXmit));
        WLAN_OS_REPORT(("BroadcastBytesXmit = %d\n", pTxData->txDataCounters[TxQid].BroadcastBytesXmit));
        WLAN_OS_REPORT(("BroadcastFramesXmit = %d\n", pTxData->txDataCounters[TxQid].BroadcastFramesXmit));
    }

    /* dbg functions */
    WLAN_OS_REPORT(("\nTx Debug info:\n", TxQid));
    WLAN_OS_REPORT(("==============\n"));
    WLAN_OS_REPORT(("DropedPacketsCounter = %d\n", pTxData->txDataDbgCounters.dbgDropedPacketsCounter));
    WLAN_OS_REPORT(("NumOfNullMsdu (in sendPacketTransfer) = %d\n", pTxData->txDataDbgCounters.dbgNumOfNullMsdu));
}


void txData_printTxQosCounters(TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t *)hTxData;
    int qIndex;

    WLAN_OS_REPORT(("-------------- Tx Queues Statistics ---------------\n\n"));
    WLAN_OS_REPORT(("Successed copied = Scheduled - Droped\n"));
    WLAN_OS_REPORT(("Successed copied = Scheduled out - check size failed\n"));

    WLAN_OS_REPORT(("-------------- Scheduled To Core queues ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
    {
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTxData->txDataDbgCounters.dbgInsertToMsduListPackets[qIndex]));
    }

    WLAN_OS_REPORT(("-------------- Droped From Core queues ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
    {
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTxData->txDataDbgCounters.dbgDropedFromMsduListPackets[qIndex]));
    }

    WLAN_OS_REPORT(("-------------- Scheduled out from Core queues ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
    {
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTxData->txDataDbgCounters.dbgScheduledOutPackets[qIndex]));
    }

    WLAN_OS_REPORT(("--Dropped due Expiry Time in Core Queues (dropped after %d %% of total time)--\n",pTxData->uFracOfLifeTimeToDrop));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
    {
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTxData->txDataDbgCounters.dbgDroppedDueExpiryTimePackets[qIndex]));
    }
    WLAN_OS_REPORT(("-------------- Free MSDUs in sendPacketTransfer ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
    {
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTxData->txDataDbgCounters.dbgNumOfMsduFreeInTxTransfer[qIndex]));
    }

    WLAN_OS_REPORT(("-------------- sendPacketTransfer CB number ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
    {
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTxData->txDataDbgCounters.dbgNumOfMsduTxTransferCB[qIndex]));
    }

    WLAN_OS_REPORT(("-------------- XFER done in scheduler ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
    {
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTxData->txDataDbgCounters.dbgNumOfMsduXferDoneInShceduler[qIndex]));
    }

    WLAN_OS_REPORT(("-------------- Success in scheduler ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
    {
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTxData->txDataDbgCounters.dbgNumOfMsduSuccessInScheduler[qIndex]));
    }

    WLAN_OS_REPORT(("-------------- Pending in scheduler ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
    {
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTxData->txDataDbgCounters.dbgNumOfMsduPendingInScheduler[qIndex]));
    }

    WLAN_OS_REPORT(("-------------- Busy in scheduler ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
    {
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTxData->txDataDbgCounters.dbgNumOfMsduBusyInScheduler[qIndex]));
    }

    WLAN_OS_REPORT(("-------------- Error in scheduler ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
    {
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTxData->txDataDbgCounters.dbgNumOfMsduErrorInScheduler[qIndex]));
    }

    WLAN_OS_REPORT(("-------------- sendPacketComplete ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
    {
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTxData->txDataDbgCounters.dbgNumOfsendPacketComplete[qIndex]));
    }

    WLAN_OS_REPORT(("-------------- sendPacketComplete Error---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
    {
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTxData->txDataDbgCounters.dbgTxCmpltError[qIndex]));
    }
            


    WLAN_OS_REPORT(("-------------- Number of MSDUs successfuly sent to GWSI layer ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
    {
        WLAN_OS_REPORT(("Queue %d = %d\n",qIndex, pTxData->txDataDbgCounters.dbgSendToGwsiQosPackets[qIndex]));
    }

    WLAN_OS_REPORT(("-------------- Current GWSI port status: %s --------------\n", 
                    (GWSI_OPEN == pTxData->txDataGwsiInterfaceStatus ? "OPEN" : "PENDING") ));

    WLAN_OS_REPORT(("-------------- HW queue available status ---------------\n"));
    for(qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
    {
        WLAN_OS_REPORT(("Queue %d %s\n",qIndex, 
                        (TRUE == pTxData->txDataAvailableQueue[qIndex] ? "Available" : "Not available") ));
    }   
}


void txData_printQosParams(TI_HANDLE hTxData)
{
    UINT8 acID;
    UINT8 qIndex;
    txData_t *pTxData = (txData_t *)hTxData;

    for(acID = FIRST_AC_INDEX;acID < MAX_NUM_OF_AC; acID++)
    {
        switch(acID)
        {
        case QOS_AC_BE:
            WLAN_OS_REPORT(("BE Params:\n"));
            break;
        case QOS_AC_BK:
            WLAN_OS_REPORT(("BK params:\n"));
            break;
        case QOS_AC_VI:
            WLAN_OS_REPORT(("VI params:\n"));
            break;
        case QOS_AC_VO:
            WLAN_OS_REPORT(("VO params:\n"));
            break;
        default:
            break;
        }

        switch(pTxData->txDataAcTrfcCtrl[acID].PsMode)
        {
        case PS_SCHEME_UPSD_TRIGGER:
            WLAN_OS_REPORT(("PsMode = UPSD\n"));

            break;
        case PS_SCHEME_LEGACY_PSPOLL:
            WLAN_OS_REPORT(("PsMode = PS_POLL\n"));

            break;
        case PS_SCHEME_LEGACY:
            WLAN_OS_REPORT(("PsMode = PS_SCHEME_REGULAR\n"));
            break;

        case PS_SCHEME_SAPSD:
            WLAN_OS_REPORT(("PsMode = S-APSD\n"));
            break;
    
        default:
            WLAN_OS_REPORT(("Error: PsMode = %d\n", pTxData->txDataAcTrfcCtrl[acID].PsMode));
            break;
        }
        WLAN_OS_REPORT(("QueueIndex = %d\n", pTxData->txDataAcTrfcCtrl[acID].QueueIndex));
        WLAN_OS_REPORT(("TxQueueSize = %d\n", pTxData->txDataAcTrfcCtrl[acID].TxQueueSize));

        qIndex = GET_QUEUE_INDEX(pTxData, acID);

        if(pTxData->dataMsduListArr[qIndex]->admissionState == AC_NOT_ADMITTED)
            WLAN_OS_REPORT(("admissionState = Not Admitted\n\n" ));
        else
            WLAN_OS_REPORT(("admissionState = Admitted\n\n" ));

    }

    switch(pTxData->txDataQosParams.headerConverMode)
    {
    case NO_CONVERT:
        WLAN_OS_REPORT(("headerConverMode = NO_CONVERT\n"));
        break;
    case QOS_CONVERT:
        WLAN_OS_REPORT(("headerConverMode = QOS_CONVERT\n"));
        break;
    case LEGACY_CONVERT:
        WLAN_OS_REPORT(("headerConverMode = LEGACY_CONVERT\n"));
        break;
    }

    WLAN_OS_REPORT(("tag_ToQueueClfrTable = %d, %d, %d, %d, %d, %d, %d, %d\n",
                    pTxData->txDataQosParams.tag_ToAcClsfrTable[0],
                    pTxData->txDataQosParams.tag_ToAcClsfrTable[1],
                    pTxData->txDataQosParams.tag_ToAcClsfrTable[2],
                    pTxData->txDataQosParams.tag_ToAcClsfrTable[3],
                    pTxData->txDataQosParams.tag_ToAcClsfrTable[4],
                    pTxData->txDataQosParams.tag_ToAcClsfrTable[5],
                    pTxData->txDataQosParams.tag_ToAcClsfrTable[6],
                    pTxData->txDataQosParams.tag_ToAcClsfrTable[7]));
}

void txData_StartTxThroughputTimer(TI_HANDLE hTxData)
{
    int counterIndex;

    txData_t *pTxData = (txData_t *)hTxData;

    if(pTxData->txThroughputTimerEnable == FALSE)
    {
        for(counterIndex = 0 ; counterIndex < MAX_NUM_OF_TX_QUEUES ; counterIndex++)
        {
            /* reset throughput counters */
            pTxData->txDataDbgCounters.dbgInsertToMsduListBytes[counterIndex] = 0;


        }

        pTxData->txThroughputTimerEnable = TRUE;

        /* start throughput timer */
        os_timerStart(pTxData->hOs, pTxData->pThroughputTimer, THROUGHPUT_TIMER, TRUE);
    }
}

void txData_StopTxThroughputTimer(TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t *)hTxData;

    if(pTxData->txThroughputTimerEnable == TRUE)
    {
        os_timerStop(pTxData->hOs, pTxData->pThroughputTimer);
        pTxData->txThroughputTimerEnable = FALSE;
    }
}

static void txData_printTxThroughputPerQueue(TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t *)hTxData;

    int qIndex;

    WLAN_OS_REPORT(("\n"));
    WLAN_OS_REPORT(("\n"));
    WLAN_OS_REPORT(("-------------- Tx Throughput per Queues Statistics ---------------\n"));
    WLAN_OS_REPORT(("-------------- Send To Wlan Per Queue Throughput---------------\n"));

    for(qIndex = 0 ; qIndex < MAX_NUM_OF_TX_QUEUES ; qIndex++)
    {

        WLAN_OS_REPORT(("Queue %d = %d KBits/sec\n", qIndex,pTxData->txDataDbgCounters.dbgInsertToMsduListBytes[qIndex]*8/1024));
        /* reset throughput counters */
        pTxData->txDataDbgCounters.dbgInsertToMsduListBytes[qIndex] = 0;
    }
}

void txData_StartTxAirThroughputTimer (TI_HANDLE hTxData)
{
    unsigned counterIndex;

    txData_t *pTxData = (txData_t *)hTxData;

    if (!pTxData->txAirThroughputTimerEnable)
    {
        for (counterIndex = 0; counterIndex < MAX_NUM_OF_TX_QUEUES; counterIndex++)
        {
            /* reset throughput counters */
            pTxData->txDataDbgCounters.dbgTxCmpltOkBytes[counterIndex] = 0;
        }

        pTxData->txAirThroughputTimerEnable = TRUE;

        /* start throughput timer */
        os_timerStart (pTxData->hOs, pTxData->pAirThroughputTimer, THROUGHPUT_TIMER, TRUE);
    }
}

void txData_StopTxAirThroughputTimer (TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t *)hTxData;

    if (pTxData->txAirThroughputTimerEnable)
    {
        os_timerStop (pTxData->hOs, pTxData->pAirThroughputTimer);
        pTxData->txAirThroughputTimerEnable = FALSE;
    }
}

static void txData_printTxAirThroughputPerQueue (TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t *)hTxData;

    int qIndex;

    WLAN_OS_REPORT (("\n"));
    WLAN_OS_REPORT (("\n"));
    WLAN_OS_REPORT (("-------------- Tx Air Throughput per Queue Statistics ---------------\n"));
    WLAN_OS_REPORT (("-------------- Send to WLAN per Queue Throughput---------------\n"));
    
    for (qIndex = 0; qIndex < MAX_NUM_OF_TX_QUEUES; qIndex++)
    {

        WLAN_OS_REPORT (("Queue %d = %d KBits/sec\n", qIndex, pTxData->txDataDbgCounters.dbgTxCmpltOkBytes[qIndex] * 8 / 1024));
        /* reset throughput counters */
        pTxData->txDataDbgCounters.dbgTxCmpltOkBytes[qIndex] = 0;
    }
}

void txData_StartJitterTimer (TI_HANDLE hTxData)
{
    unsigned u_ac;

    txData_t *pTxData = (txData_t *)hTxData;

    if (!pTxData->txJitterTimerEnable)
    {
        for (u_ac = 0; u_ac < MAX_NUM_OF_TX_QUEUES; u_ac++)
        {
            /* reset jitter intervals */
            pTxData->txJitter[u_ac].jitter.core = 0;
            pTxData->txJitter[u_ac].jitter.xfer = 0;
            pTxData->txJitter[u_ac].jitter.air  = 0;
            pTxData->txJitter[u_ac].jitter.fw   = 0;

            pTxData->txJitter[u_ac].delay.core  = 0;
            pTxData->txJitter[u_ac].delay.xfer  = 0;
            pTxData->txJitter[u_ac].delay.wait  = 0;
            pTxData->txJitter[u_ac].delay.fw    = 0;
            pTxData->txJitter[u_ac].delay.air   = 0;

            pTxData->txJitter[u_ac].last_delay.core = 0;
            pTxData->txJitter[u_ac].last_delay.xfer = 0;
            pTxData->txJitter[u_ac].last_delay.fw   = 0;
            pTxData->txJitter[u_ac].last_delay.air  = 0;

            pTxData->txJitter[u_ac].max_delay.core  = 0;
            pTxData->txJitter[u_ac].max_delay.xfer  = 0;
            pTxData->txJitter[u_ac].max_delay.fw    = 0;
            pTxData->txJitter[u_ac].max_delay.air   = 0;

            pTxData->txJitter[u_ac].count.core  = 0;
            pTxData->txJitter[u_ac].count.xfer  = 0;
            pTxData->txJitter[u_ac].count.wait  = 0;
            pTxData->txJitter[u_ac].count.fw = 
                pTxData->txDataDbgCounters.dbgTxCmpltOk[u_ac];
            pTxData->txJitter[u_ac].count.fw_err = 
                pTxData->txDataDbgCounters.dbgTxCmpltError[u_ac];
        }

        pTxData->txJitterTimerEnable = TRUE;

        /* start throughput timer */
        os_timerStart (pTxData->hOs, pTxData->pJitterTimer, THROUGHPUT_TIMER, TRUE);
    }
}

void txData_StopJitterTimer (TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t *)hTxData;

    if (pTxData->txJitterTimerEnable)
    {
        os_timerStop (pTxData->hOs, pTxData->pJitterTimer);
        pTxData->txJitterTimerEnable = FALSE;
    }
}    

static void txData_printJitter (TI_HANDLE hTxData)
{
    unsigned  u_ac;
    txData_t *pTxData = (txData_t *)hTxData;

    WLAN_OS_REPORT (("\n"));
    WLAN_OS_REPORT (("\n"));
    WLAN_OS_REPORT (("-------------- Tx Jitter per Queue Statistics ---------------\n"));
    
    for (u_ac = 0; u_ac < MAX_NUM_OF_TX_QUEUES; u_ac ++)
    {
        UINT32 u_ok   = pTxData->txDataDbgCounters.dbgTxCmpltOk[u_ac] -
                        pTxData->txJitter[u_ac].count.fw;
        UINT32 u_nok  = pTxData->txDataDbgCounters.dbgTxCmpltError[u_ac] -
                        pTxData->txJitter[u_ac].count.fw_err;

        if (u_ok + u_nok) 
        {
            WLAN_OS_REPORT (("Queue [%d], drop=%d%%\n", u_ac, u_nok * 100 / (u_ok + u_nok)));

            if (pTxData->txJitter[u_ac].count.core)
            {
                WLAN_OS_REPORT (("  Core: avg.delay=%6d, max.delay=%6d, jitter=%6d\n",
                                 pTxData->txJitter[u_ac].delay.core / pTxData->txJitter[u_ac].count.core,
                                 pTxData->txJitter[u_ac].max_delay.core,
                                 pTxData->txJitter[u_ac].jitter.core / pTxData->txJitter[u_ac].count.core));
            }
            if (pTxData->txJitter[u_ac].count.xfer)
            {
                WLAN_OS_REPORT (("  Xfer: avg.delay=%6d, max.delay=%6d, jitter=%6d\n", 
                                 pTxData->txJitter[u_ac].delay.xfer / pTxData->txJitter[u_ac].count.xfer,
                                 pTxData->txJitter[u_ac].max_delay.xfer,
                                 pTxData->txJitter[u_ac].jitter.xfer / pTxData->txJitter[u_ac].count.xfer));
            }
            if (pTxData->txJitter[u_ac].count.wait)
            {
                WLAN_OS_REPORT (("  Wait: avg.delay=%6d\n", 
                                 pTxData->txJitter[u_ac].delay.wait / pTxData->txJitter[u_ac].count.wait));
            }
            if (u_ok > 0)
            {
                WLAN_OS_REPORT (("  Fw:   avg.delay=%6d, max.delay=%6d, jitter=%6d\n", 
                                 pTxData->txJitter[u_ac].delay.fw / u_ok,
                                 pTxData->txJitter[u_ac].max_delay.fw,
                                 pTxData->txJitter[u_ac].jitter.fw / u_ok));
                WLAN_OS_REPORT (("  Air:  avg.delay=%6d, max.delay=%6d, jitter=%6d\n", 
                                 pTxData->txJitter[u_ac].delay.air / u_ok,
                                 pTxData->txJitter[u_ac].max_delay.air,
                                 pTxData->txJitter[u_ac].jitter.air / u_ok));
            }
        }

        /* Update/reset jitter info */
        pTxData->txJitter[u_ac].jitter.core = 0;
        pTxData->txJitter[u_ac].jitter.xfer = 0;
        pTxData->txJitter[u_ac].jitter.fw   = 0;    
        pTxData->txJitter[u_ac].jitter.air  = 0;

        pTxData->txJitter[u_ac].delay.core  = 0;
        pTxData->txJitter[u_ac].delay.xfer  = 0;
        pTxData->txJitter[u_ac].delay.wait  = 0;
        pTxData->txJitter[u_ac].delay.fw    = 0;
        pTxData->txJitter[u_ac].delay.air   = 0;

        pTxData->txJitter[u_ac].max_delay.core  = 0;
        pTxData->txJitter[u_ac].max_delay.xfer  = 0;
        pTxData->txJitter[u_ac].max_delay.fw    = 0;
        pTxData->txJitter[u_ac].max_delay.air   = 0;

        pTxData->txJitter[u_ac].count.core  = 0;
        pTxData->txJitter[u_ac].count.xfer  = 0;
        pTxData->txJitter[u_ac].count.wait  = 0;
        pTxData->txJitter[u_ac].count.fw = 
            pTxData->txDataDbgCounters.dbgTxCmpltOk[u_ac];
        pTxData->txJitter[u_ac].count.fw_err = 
            pTxData->txDataDbgCounters.dbgTxCmpltError[u_ac];
    }
}

#endif /* TI_BDG */

/***********************************************************************
 *                     txData_buildQosNullDataFrame
 ***********************************************************************
DESCRIPTION:    builds QOS_NULL_DATA frame.

INPUT:

OUTPUT:     None

RETURN:     OK on success, NOK otherwise
************************************************************************/
TI_STATUS txData_buildQosNullDataFrame(TI_HANDLE hTxData,mem_MSDU_T **pMsduPsPoll, UINT8 userPriority)
{
    TI_STATUS       status = OK;
    paramInfo_t     daParam, saParam;
    dot11_header_t   *pFrame; /* Note : there is no body for null frame */

    txData_t *pTxData = (txData_t *)hTxData;
    mem_MSDU_T      *pMsdu;

    status = wlan_memMngrAllocMSDU(pTxData->hMemMngr, &pMsdu, WLAN_QOS_HDR_LEN + TX_TOTAL_OFFSET_BEFORE_DATA, TX_MODULE);

    if (status != OK)
    {
        return NOK;
    }
    pFrame = (dot11_header_t*)(pMsdu->firstBDPtr->data+ TX_TOTAL_OFFSET_BEFORE_DATA);

    /* Build frame control */
    pFrame->fc = DOT11_FC_DATA_NULL_QOS;
    pFrame->fc |= (0x1 << DOT11_FC_TO_DS_SHIFT);

    pFrame->qosControl = (userPriority << QOS_CONTROL_UP_SHIFT);

    pFrame->fc = ENDIAN_HANDLE_WORD(pFrame->fc);

    /* BSSID */
    daParam.paramType = CTRL_DATA_CURRENT_BSSID_PARAM;
    status = ctrlData_getParam(pTxData->hCtrlData, &daParam);
    if (status != OK)
    {
        wlan_memMngrFreeMSDU(pTxData->hMemMngr, pMsdu->handle);
        return NOK;
    }
    MAC_COPY(pTxData->hOs, (&pFrame->address3), (&daParam.content.ctrlDataCurrentBSSID));
    MAC_COPY(pTxData->hOs, (&pFrame->address1), (&daParam.content.ctrlDataCurrentBSSID));

    /* Source MAC address */
    saParam.paramType = CTRL_DATA_MAC_ADDRESS;
    status = ctrlData_getParam(pTxData->hCtrlData, &saParam);
    if (status != OK)
    {
        wlan_memMngrFreeMSDU(pTxData->hMemMngr, pMsdu->handle);
        return NOK;
    }
   /* copy source mac address */
    MAC_COPY(pTxData->hOs, (&pFrame->address2), (&saParam.content.ctrlDataDeviceMacAddress));

    /* Update MSDU parameters */
    pMsdu->headerLen = WLAN_QOS_HDR_LEN;
    pMsdu->dataLen = WLAN_QOS_HDR_LEN;
    pMsdu->firstBDPtr->length = WLAN_QOS_HDR_LEN;
 
    pMsdu->qosTag = userPriority;
    pMsdu->txFlags |= TX_DATA_NULL_MSDU;

    // all data or mgmt packets built on host must have the correct 
    // offset set to point to the start of the mac frame
    memMgr_BufOffset(pMsdu->firstBDPtr) = TX_TOTAL_OFFSET_BEFORE_DATA;

    
    *pMsduPsPoll = pMsdu;

    return status;
}


/****************************************************************************
 *                      txData_SetQidToAcTable()
 ****************************************************************************
 * DESCRIPTION: set Qid according to Queue ID
 ****************************************************************************/
void txData_SetQidToAcTable(TI_HANDLE hTxData,UINT8 QidStart, UINT8 QidEnd,UINT8 AcId)
{
    int i;
    txData_t *pTxData = (txData_t *)hTxData;

    for(i = QidStart ; i <= QidEnd; i++) 
        pTxData->QidToAcTable[i] = AcId;

}

/****************************************************************************
 *                      txData_GetAcIdFromQid()
 ****************************************************************************
 * DESCRIPTION: Get the Ac Id according to the Queue id 
 ****************************************************************************/
UINT8 txData_GetAcIdFromQid(TI_HANDLE hTxData,UINT8 Qid)
{
    txData_t *pTxData = (txData_t *)hTxData;
    return (pTxData->QidToAcTable[Qid]);
}

/****************************************************************************
 *                      txData_GetWlanHeaderLength()
 ****************************************************************************
 * DESCRIPTION: calculates the WLAN header length, according to QoS,
 * current encryption, and packet type
 ****************************************************************************/
UINT32 txData_GetWlanHeaderLength( TI_HANDLE hTxData, void *pData, UINT32 txFlags )
{
    txData_t            *pTxData = (txData_t*)hTxData;
    UINT32              wlanHeaderLength = 0;
    EthernetHeader_t    *pEthHeader;
    UINT16              swapedTypeLength;


    /* management frames never have QoS or encryption padding */
    if ( txFlags & TX_DATA_MGMT_MSDU )
    {
        return WLAN_HDR_LEN;
    }
    
    /* 
     * Determine 802.11 header length 
     * QoS Header is longer (2 Bytes)
     */
    if ( pTxData->txDataQosParams.headerConverMode == QOS_CONVERT )
    {
        wlanHeaderLength = WLAN_QOS_HDR_LEN;
    }
    else
    {
        wlanHeaderLength = WLAN_HDR_LEN;
    }

    if ( txFlags & TX_DATA_EAPOL_MSDU )
    {
        /* EAPOLs should always contain SNAP */
        wlanHeaderLength += WLAN_SNAP_HDR_LEN;
        /* EAPOL encryption is set by the RSN module */
        if ( TRUE == pTxData->txDataEapolEncryptionStatus )
        {
            wlanHeaderLength += pTxData->encryptionFieldSize; 
        }
        return wlanHeaderLength;
    }
    
    /* add encryption overhead - 4 bytes for TKIP, 8 for AES. Actual decision was done at RSN */
    if (pTxData->txDataCurrentPrivacyInvokedMode == TRUE)
    {
        wlanHeaderLength += pTxData->encryptionFieldSize;
    }

    /* 
     * IAPP header should always include SNAP, but this is already included in the frame
     * body itself, so it is not added here
     */
    if ( txFlags & TX_DATA_IAPP_MSDU )
    {
        return wlanHeaderLength;
    }

    /* stat frames are checked if they already contain SNAP header or not */
    if ( txFlags & TX_DATA_DATA_MSDU )
    {
        /*
         * Detect the packet type and decide if to create a    
         * new SNAP or leave the original LLC.         
         */
        pEthHeader = (EthernetHeader_t *)pData;
        swapedTypeLength = wlan_htons(pEthHeader->TypeLength);

        /* 
         * if the "type" field is greater than 1500 bytes, it means the frame we received has
         * Ethernet II header (destination MAC, source MAC, type, 14 bytes total), and therefore
         * we need to reserve 8 more bytes after the 802.11 header for LLC/SNAP header.
         * If this field is smaller than or equal to 1500, this is not really a "type" field of 
         * Ethernet II header, but rather a 802.3 size field. since 802.3 already has 8 bytes
         * LLC/SNAP header (other than the 802.3 14 bytes of destination MAC, source MAC and size),
         * there is no need to reserve 8 more bytes.
         */
        if ( swapedTypeLength > ETHERNET_MAX_PAYLOAD_SIZE )
        {
            return wlanHeaderLength + WLAN_SNAP_HDR_LEN;
        }
        else
        {
            return wlanHeaderLength;
        }
    }
    else
    {
        WLAN_REPORT_ERROR( pTxData->hReport, TX_DATA_MODULE_LOG,
                           (" %s: trying to get header length for packet with txFlags: 0x%x\n", __FUNCTION__, txFlags) );
        return 0;
    }
}

/****************************************************************************
 *                      txDataMsduTimeExpired()
 ****************************************************************************
 * DESCRIPTION: calculates the time left until MSDU will expire, retunrs the
 * time in TUs, or 0 if the MSDU has already expired.
 ****************************************************************************/
UINT32 txDataTimeToMsduExpiry (TI_HANDLE hTxData, mem_MSDU_T* pMsdu, UINT8 qID)
{
    txData_t    *pTxData = (txData_t*)hTxData;
    UINT32       acID, uPassedTime;

    /* translate queue ID to AC ID */
    acID = txData_GetAcIdFromQid (hTxData, qID);

    /* get time passed since insertion */
    uPassedTime = os_timeStampUs (pTxData->hOs) - pMsdu->insertionTime;

    /* if the passed time is smaller than a certain proportion of the MsduLifeTime - it's O.K. */
    if (uPassedTime < 
        ((pTxData->uFracOfLifeTimeToDrop * pTxData->txDataAcTrfcCtrl[acID].MsduLifeTime) / 100))
    {
        return (pTxData->txDataAcTrfcCtrl[acID].MsduLifeTime - uPassedTime) >> 10;
    }
    
    /* timer expired, or about to expire --> drop it.*/
    return 0;
}


/****************************************************************************
*                               txData_startAfterRecovery                                *
*****************************************************************************
* DESCRIPTION:  This function start the tx data after recovery.
*               It uses params that save before recovery.
*
* INPUTS:       hTxData - the object
*
* OUTPUT:
*
* RETURNS:      OK - stop succesfull
*               NOK - stop unsuccesfull
****************************************************************************/
TI_STATUS txData_startAfterRecovery(TI_HANDLE hTxData)
{
    txData_t *pTxData = (txData_t *)hTxData;

    /* check parameters validity */
    if( pTxData == NULL )
    {
        WLAN_REPORT_ERROR(pTxData->hReport, TX_DATA_MODULE_LOG,
            (" txData_start() : Illegal value for hTxData\n"));
        return NOK;
    }

    pTxData->txDataPortStatus = pTxData->savePortStatus;
    pTxData->txDataCurrentPrivacyInvokedMode = pTxData->saveTxDataCurrentPrivacyInvokedMode;
    pTxData->txDataEapolEncryptionStatus = pTxData->saveTxDataEapolEncryptionStatus;
    pTxData->encryptionFieldSize = pTxData->saveEncryptionFieldSize;

    /* start scheduler timer */
    os_timerStart(pTxData->hOs, pTxData->pSchedulerTimer, SCHEDULER_TIMER, TRUE);
    pTxData->bSchedulerTimerRunning = TRUE;

    return OK;
}
 

