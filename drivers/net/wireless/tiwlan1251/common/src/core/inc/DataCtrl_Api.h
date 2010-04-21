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
/*                                                                         */
/*      PURPOSE:    DataCtrl module api functions header file              */
/*                                                                         */
/***************************************************************************/

#ifndef _DATA_CTRL_API_H_
#define _DATA_CTRL_API_H_

#include "paramOut.h"
#include "paramIn.h"
#include "rxXfer_api.h"
#include "802_11Defs.h"
#include "MsduList.h"
#include "GeneralUtilApi.h"


typedef struct
{
    UINT32 hwNumOfFreeMsdu;
    UINT32 hwNumOfBusyMsdu;
    UINT32 hwNumOfFreeBDs;
    UINT32 hwTotalAvailMem; 
}hwTxInformation_t;

/*  RX MODULE   */
/*--------------*/

/* Rx module interface functions */

#define RECV_OK                  0x1
#define DIRECTED_BYTES_RECV      0x2
#define DIRECTED_FRAMES_RECV     0x4
#define MULTICAST_BYTES_RECV     0x8
#define MULTICAST_FRAMES_RECV    0x10   
#define BROADCAST_BYTES_RECV     0x20    
#define BROADCAST_FRAMES_RECV    0x40

#define NO_RX_NOTIFICATION  0x0

#define ALL_RCV_FRAME (DIRECTED_FRAMES_RECV|MULTICAST_FRAMES_RECV|BROADCAST_FRAMES_RECV)

#define  MAX_RX_NOTIF_REQ_ELMENTS 8


/*TI_HANDLE rxData_create (msduReceiveCB_t* msduReceiveCB, TI_HANDLE hOs);  */
TI_HANDLE rxData_create (TI_HANDLE hOs);    

TI_STATUS rxData_config(TI_HANDLE       hRxData, 
                     TI_HANDLE          hCtrlData, 
                     TI_HANDLE          hTxData,
                     TI_HANDLE          hTnetwDrv,
                        TI_HANDLE   hHalCtrl,
                     TI_HANDLE          hMlme, 
                     TI_HANDLE          hRsn, 
                     TI_HANDLE          hSiteMgr, 
                     TI_HANDLE          hExcMngr, 
                     TI_HANDLE          hOs, 
                     TI_HANDLE          hReport,
                     TI_HANDLE          hMemMngr,
                        TI_HANDLE   hEvHandler,
                        rxDataInitParams_t * rxDataInitParams);

void rxData_receiveMsduFromWlan(TI_HANDLE hRxData, mem_MSDU_T *pMsdu, Rx_attr_t* pRxAttr);


TI_STATUS rxData_stop(TI_HANDLE hRxData);

TI_STATUS rxData_unLoad(TI_HANDLE hRxData); 

TI_STATUS rxData_getParam(TI_HANDLE hRxData, paramInfo_t *pParamInfo);  

TI_STATUS rxData_setParam(TI_HANDLE hRxData, paramInfo_t *pParamInfo);  

TI_STATUS rxData_getTiwlnCounters(TI_HANDLE hRxData, TIWLN_COUNTERS *pTiwlnCounters);

TI_STATUS txData_copyPacketToMsdu(TI_HANDLE hTxData,mem_MSDU_T **pMsdu, int FreeOldMsdu);

void rxData_resetCounters(TI_HANDLE hRxData);

TI_STATUS txData_updateUsedTime(TI_HANDLE hTxData, UINT32 qNum, UINT16 usedTime);

TI_STATUS txData_setAdmisionCtrlParams(TI_HANDLE hTxData, 
                                       UINT8 acID,
                                       UINT16 mediumTime, 
                                       UINT32 minimumPHYRate,
                                       BOOL admFlag);

TI_STATUS txData_setPsVoiceDeliveryMode(TI_HANDLE hTxData, PSScheme_e   PsMode);

TI_HANDLE rxData_RegNotif(TI_HANDLE hRxData,UINT16 EventMask,GeneralEventCall_t CallBack,TI_HANDLE context,UINT32 Cookie);

TI_STATUS rxData_UnRegNotif(TI_HANDLE hRxData,TI_HANDLE RegEventHandle);

TI_STATUS rxData_AddToNotifMask(TI_HANDLE hRxData,TI_HANDLE Notifh,UINT16 EventMask);


/* debug functions */
void rxData_resetDbgCounters(TI_HANDLE hRxData);
void rxData_printRxBlock(TI_HANDLE hRxData);
void rxData_printRxCounters(TI_HANDLE hRxData);
void rxData_startRxThroughputTimer(TI_HANDLE hRxData); 
void rxData_stopRxThroughputTimer(TI_HANDLE hRxData); 
void rxData_printRxDataFilter(TI_HANDLE hRxData);


/*  TX MODULE   */
/*--------------*/

/* Tx module data types */

typedef enum 
{
    NO_DISABLE                  = 0,
    DISABLE_IMMEDIATELY         = 1,
} txDisableReason_e;

typedef struct
{
    UINT32      HwRate;
    rate_e      Rate;   
    UINT8       txRatePolicyId;
} txData_attr_t; 

typedef struct
{
    mem_MSDU_T          *pMsdu;
    BOOLEAN             bDataMsdu;
    UINT8               txQid;
    UINT8               txCompleteFlags;
    UINT32              maxTransmitRate;
    BOOLEAN             bXferDoneCalled;
    BOOLEAN             bTxCompleteCalled;
    UINT32              msduDataLen;
    macAddress_t        destinationMac;
    UINT32              driverHandlingTime;
    UINT32              timeStamp [6];  
} txPacketIdAttr_t;

typedef struct
{
    TI_STATUS               status;
    UINT32                  packetId;
    UINT32                  rate;
    UINT8                   ackFailures;
    UINT32                  actualDurationInAir;
    UINT32                  fwHandlingTime;
    UINT32                  mediumDelay;
} txCompleteAttr_t;

#define XFER_OK                 0x1
#define DIRECTED_BYTES_XFER     0x2
#define DIRECTED_FRAMES_XFER    0x4
#define MULTICAST_BYTES_XFER    0x8
#define MULTICAST_FRAMES_XFER   0x10
#define BROADCAST_BYTES_XFER    0x20
#define BROADCAST_FRAMES_XFER   0x40

#define NO_TX_NOTIFICATION      0x0

#define ALL_XMIT_FRAMES (DIRECTED_FRAMES_XMIT|MULTICAST_FRAMES_XMIT|BROADCAST_FRAMES_XMIT)

#define  MAX_TX_NOTIF_REQ_ELMENTS 8

#define POLL_AP_PACKETS_FORCE_PS_POLL  0xFF /* indicates a "force" PS POLL for the PollAPPackets routine (for backwards compatibility) */


/* Tx module interface functions */
TI_HANDLE txData_create(txDataInitParams_t *txDataInitParams, TI_HANDLE hOs);

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
                       TI_HANDLE      hPowerMgr);

TI_STATUS txData_stop(TI_HANDLE hTxData);

TI_STATUS txData_start(TI_HANDLE hTxData);
TI_STATUS txData_startAfterRecovery(TI_HANDLE hTxData);


void txData_recoveryIndication( TI_HANDLE hTxData );

TI_STATUS txData_getParam(TI_HANDLE hTxData, paramInfo_t *pParamInfo);  

TI_STATUS txData_setParam(TI_HANDLE hTxData, paramInfo_t *pParamInfo);  

TI_STATUS txData_unLoad(TI_HANDLE hTxData); 

TI_STATUS txData_getTiwlnCounters(TI_HANDLE hTxData, TIWLN_COUNTERS *pTiwlnCounters);   

TI_STATUS txData_checkQueueSize(TI_HANDLE hTxData,UINT8 qIndex);

TI_STATUS txData_sendPktToWlan(TI_HANDLE hTxData, mem_MSDU_T *pMsdu, UINT8 pkt_DTag);

TI_STATUS txData_txSendMsdu(TI_HANDLE hTxData, mem_MSDU_T *pMsdu);

TI_STATUS txData_startTxScheduler(TI_HANDLE hTxData);

TI_STATUS txData_txCompleteUpdate( TI_HANDLE hTxData, txCompleteAttr_t *pTxCompleteAttr );

void      txData_sendPacketTransfer(TI_HANDLE hTxData, UINT32 pPacketIdAttr);
void      txData_sendPacketDebug(TI_HANDLE hTxData, UINT32 pPacketIdAttr, UINT32 uDebugInfo);



TI_STATUS txData_disableTransmission(TI_HANDLE hTxData,txDisableReason_e reason);

TI_STATUS txData_enableTransmission(TI_HANDLE hTxData);
TI_STATUS txData_sendNullFrame(TI_HANDLE hTxData,
                               BOOL powerSaveOn,
                               allocatingModule_e module);


TI_STATUS txData_getPsPollFrame(TI_HANDLE hTxData,mem_MSDU_T **pMsduPsPoll);

TI_STATUS txData_buildQosNullDataFrame(TI_HANDLE hTxData,mem_MSDU_T **pMsduPsPoll, UINT8 userPriority);

TI_STATUS txData_sendVadFrame(TI_HANDLE hTxData, UINT8 acID);

void txData_resetCounters(TI_HANDLE hTxData);

TI_HANDLE txData_RegNotif(TI_HANDLE hTxData,UINT16 EventMask,GeneralEventCall_t CallBack,TI_HANDLE context,UINT32 Cookie);

TI_STATUS txData_UnRegNotif(TI_HANDLE hTxData,TI_HANDLE RegEventHandle);

TI_STATUS txData_AddToNotifMask(TI_HANDLE hTxData,TI_HANDLE Notifh,UINT16 EventMask);

BOOL txData_isQueueUseMediumTime(TI_HANDLE hTxData, UINT8 qNum);

UINT32 txData_GetWlanHeaderLength( TI_HANDLE hTxData, void *pData, UINT32 txFlags );

/* debug functions */
void txData_printTxBlock(TI_HANDLE hTxData);
void txData_printTxCounters(TI_HANDLE hTxData);
void txData_printQosParams(TI_HANDLE hTxData);
void txData_resetDbgCounters(TI_HANDLE hTxData);
void txData_printDataMsduList(TI_HANDLE hTxData);
void txData_fullPrintDataMsduList(TI_HANDLE hTxData);
void txData_printMgmtMsduList(TI_HANDLE hTxData);
void txData_fullPrintMgmtMsduList(TI_HANDLE hTxData);
void txData_StartTxThroughputTimer(TI_HANDLE hTxData);
void txData_StopTxThroughputTimer(TI_HANDLE hTxData);
void txData_StartTxAirThroughputTimer(TI_HANDLE hTxData);
void txData_StopTxAirThroughputTimer(TI_HANDLE hTxData);
void txData_StartJitterTimer(TI_HANDLE hTxData);
void txData_StopJitterTimer(TI_HANDLE hTxData);
void txData_printTxQosCounters(TI_HANDLE hTxData);



/* CONTROL MODULE */
/*----------------*/


typedef struct
{
    macAddress_t    ctrlDataDeviceMacAddress; 
} ctrlDataConfig_t; 

/*******************************/
/* Control module interface functions */
/*TI_HANDLE ctrlData_create(TI_HANDLE hOs,*/
/*                           TxCompleteStatusCB_t* TxCmplt_CB);*/
TI_HANDLE ctrlData_create(TI_HANDLE hOs);

TI_STATUS ctrlData_config(TI_HANDLE         hCtrlData,
                       TI_HANDLE            hWhalCtrl, 
                       TI_HANDLE            hSiteMgrHandle, 
                       TI_HANDLE            hTxData, 
                       TI_HANDLE            hRxData, 
                       TI_HANDLE            hOs, 
                       TI_HANDLE            hReport, 
                       TI_HANDLE            hMemMngr, 
                       TI_HANDLE            hEvHandler,
                       TI_HANDLE            hAPConnection,
                       TI_HANDLE            hTrafficMonitor,
                       disassocSentCB_t     disassocSentCBFunc,
                       TI_HANDLE            disassocSentCBObj,  
                       ctrlDataInitParams_t *ctrlDataInitParams);

TI_STATUS ctrlData_unLoad(TI_HANDLE hCtrlData); 

TI_STATUS ctrlData_getParam(TI_HANDLE hCtrlData, paramInfo_t *pParamInfo);  

TI_STATUS ctrlData_setParam(TI_HANDLE hCtrlData, paramInfo_t *pParamInfo);  

TI_STATUS ctrlData_start(TI_HANDLE hCtrlData);  

TI_STATUS ctrlData_stop(TI_HANDLE hCtrlData);

TI_STATUS ctrlData_getTiwlnCounters(TI_HANDLE hCtrlData, TIWLN_COUNTERS *pTiwlnCounters);   

TI_STATUS ctrlData_rxMsdu(TI_HANDLE         hCtrlData, 
                          mem_MSDU_T        **pRxMsdu);
#ifdef SUPPORT_4X
TI_STATUS ctrlData_txDequeueMsdu(TI_HANDLE          hCtrlData, 
                                 mem_MSDU_T**       buildMsduPtr,
                                 MsduList_t*        pMsduList, 
                                 whalTx_attr_t*     pWhalTx_attr,
                                 hwTxInformation_t* pHwTxInformation);

TI_STATUS ctrlData_txMsdu(TI_HANDLE         hCtrlData, 
                          mem_MSDU_T**      msduPtr);
#endif

TI_STATUS ctrlData_getTxAttributes(TI_HANDLE hCtrlData , 
                                   UINT32 txFlags, 
                                   txData_attr_t *pTxAttr,
                                   UINT32 ac);  

TI_STATUS ctrlData_ClsfrClassifyTxMSDU(TI_HANDLE    hCtrlData, 
                                       mem_MSDU_T   *pMsdu, 
                                       UINT8        packet_DTag);

TI_STATUS ctrlData_clsfrSetClsfrType(TI_HANDLE          hCtrlData,
                                    clsfrTypeAndSupport     newClsfrType);

    

void ctrlData_getCurrBssTypeAndCurrBssId(TI_HANDLE hCtrlData, macAddress_t *pCurrBssid, 
                                         bssType_e *pCurrBssType);  

#ifdef SUPPORT_4X
TI_STATUS ctrlData_get4xInfoElemnt(TI_HANDLE hCtrlData, 
                                   dot11_4X_t* fourXInfoElemnt);

TI_STATUS ctrlData_get4xStatus(TI_HANDLE hCtrlData,BOOL* fourXEnable);

TI_STATUS ctrlData_evalSite(TI_HANDLE hCtrlData, 
                            dot11_4X_t* site4xParams, 
                            UINT32 *matchingLevel);
#endif

TI_STATUS ctrlData_setSite(TI_HANDLE hCtrlData,
                           dot11_4X_t* site4xParams);

void ctrlData_setTspecsRateEvent(TI_HANDLE          hCtrlData,
                                    UINT8               acID,
                                    BOOL                enableEvent);

void ctrlData_setTspecsRateThresholds(TI_HANDLE     hCtrlData,
                                      UINT8         acID,
                                      UINT8     highRateThreshold,
                                      UINT8     lowRateThreshold);

void ctrlData_txCompleteStatus( TI_HANDLE hCtrlData,
                               txCompleteAttr_t *pTxCompleteAttr);


void ctrlData_getTspecsRateThresholds(TI_HANDLE hCtrlData, UINT8 uAC, UINT32* pHighThreshold, UINT32* pLowThreshold);
void ctrlData_ToggleTrafficIntensityNotification (TI_HANDLE hCtrlData, BOOL enabledFlag);


/* dbg functions */
/*---------------*/
void ctrlData_printRateAdaptation(TI_HANDLE hCtrlData);
void ctrlData_printTxParameters(TI_HANDLE hCtrlData);
void ctrlData_printCtrlBlock(TI_HANDLE hCtrlData);
void ctrlData_printCtrlCounters(TI_HANDLE hCtrlData);
void ctrlData_printFourX(TI_HANDLE hCtrlData);

#ifdef TI_DBG
void ctrlData_clearClsfrTable ( TI_HANDLE hCtrlData );
void ctrlData_printClsfrTable ( TI_HANDLE hCtrlData );
#endif

/* TEST FUNCTION */
/*---------------*/
void Test_OsFreeFunction(TI_HANDLE hOs, TI_HANDLE pAddr);
TI_HANDLE Test_rxData_receiveMsduFromWlan(TI_HANDLE hRxData, mem_MSDU_T *pMsdu, Rx_attr_t* pRxAttr);

#endif

