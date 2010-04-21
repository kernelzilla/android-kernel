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
/*                                                                          */
/*    MODULE:   TNETW-Driver                                                */
/*                                                                          */
/*    PURPOSE:  Driver API functions prototypes.                            */
/*                                                                          */
/***************************************************************************/

#ifndef TNETW_DRIVER_API_H
#define TNETW_DRIVER_API_H


#include "commonTypes.h"
#include "TNETW_Driver_types.h"
#include "802_11Defs.h"

/************************************************************************
                TNETW-Driver  Common  API  Functions
************************************************************************/

/* Functions that handle common procedures for multiple driver modules (mainly initialization). */
TI_HANDLE   TnetwDrv_Create              (TI_HANDLE hOs);
TI_STATUS   TnetwDrv_Init                (TI_HANDLE hTnetwDrv, TI_HANDLE hReport, TI_HANDLE hMemMgr, TI_HANDLE hUser, UINT32 *pFWImage, TnetwDrv_InitParams_t* pInitParams, TnetDrv_callback_t fUserConf);
TI_STATUS   TnetwDrv_Configure           (TI_HANDLE hTnetwDrv, TnetwDrv_InitParams_t* pInitParams);
void        TnetwDrv_Destroy             (TI_HANDLE hTnetwDrv);
void        TnetwDrv_Register_CB         (TI_HANDLE hTnetwDrv,tiUINT32 EventID,void *CBFunc, void *pData);
void        TnetwDrv_GetInitParams       (TI_HANDLE hTnetwDrv, UINT8 *pcommand, UINT16 *OutBufLen);
void        TnetwDrv_PrintInfo           (TI_HANDLE hTnetwDrv, TnetwDrv_PrintInfoType_e printInfo);
TI_STATUS   TnetwDrv_FinalizeDownload    (TI_HANDLE hTnetwDrv);
TI_STATUS   TnetwDrv_FinalizeOnFailure   (TI_HANDLE hTnetwDrv);


/*  TEMPORARY!! - untill the new TNETW-Driver architecture is completed!!  */
void        TnetwDrv_TEMP_GetHandles     (TI_HANDLE hTnetwDrv, TI_HANDLE *pHalCtrl, TI_HANDLE *pMacServices);

#ifdef GWSI_SPI_TEST
TI_HANDLE TnetwDrv_GetTnetwifHandle (TI_HANDLE hTnetwDrv);
#endif /* GWSI_SPI_TEST */

/************************************************************************
                TNETW-Driver  Tx  API  Functions
************************************************************************/

txCtrlBlkEntry_t   *TnetwDrv_txCtrlBlk_alloc         (TI_HANDLE hTnetwDrv);
void                TnetwDrv_txCtrlBlk_free          (TI_HANDLE hTnetwDrv, txCtrlBlkEntry_t *pCurrentEntry);
txCtrlBlkEntry_t   *TnetwDrv_txCtrlBlk_GetPointer    (TI_HANDLE hTnetwDrv, UINT8 descId);
TI_STATUS           TnetwDrv_txHwQueue_alloc         (TI_HANDLE hTnetwDrv, txCtrlBlkEntry_t *pPktCtrlBlk);
TI_STATUS           TnetwDrv_txHwQueue_free          (TI_HANDLE hTnetwDrv, txCtrlBlkEntry_t *pPktCtrlBlk);
UINT8               TnetwDrv_txHwQueue_GetUsedHwBlks (TI_HANDLE hTnetwDrv, int TxQid);
UINT8               TnetwDrv_txGetAckPolicy          (TI_HANDLE hTnetwDrv, int TxQid, BOOL bIsMultiCastAndIBSS);
void                TnetwDrv_printInfo               (TI_HANDLE hTnetwDrv);

systemStatus_e      TnetwDrv_txXfer_sendPacket       (TI_HANDLE hTnetwDrv, 
                const void  *aFrame,        /* Pointer to the packet content. points to */
                                            /* the place that the actual packet begins. */
                                            /* a size of TX_TOTAL_OFFSET_BEFORE_DATA    */
                                            /* must be saved before that pointer        */
                UINT16      aLength,        /* MSDU length from first byte of MAC       */
                                            /*   header to last byteof frame body.      */
                UINT8       aQueueId,       /* Tx queue as defined in ConfigureQueue.   */
                UINT8       aTxRateClassId, /* Tx rate class ID defined in txRatePolicy.*/
                UINT16      aMaxTransmitRate,/* A bit mask that specifies the initial   */
                                            /*     (highest) rate to use.               */
                BOOL        aMore,          /* Tells if there is another packet coming  */
                                            /*   shortly after this one.                */
                UINT32      aPacketId,      /* Packet identifier used as a context by   */
                                            /*   the host driver.                   */
                UINT8       aPowerLevel,    /* Transmission power level.                */
                UINT32      aExpiryTime,    /* Time left for this MSDU to live.         */
                void        *aReserved);    /* Optional parameters pointer.             */

#if !defined(GWSI_DRIVER) && !defined(GWSI_LIB)
void TnetwDrv_StartRecovery(TI_HANDLE hTnetwDrv, void *endOfRecoveryCB, TI_HANDLE hRecoveryMgr);
TI_STATUS TnetwDrv_InitHw_FinalizeDownload(TI_HANDLE hTnetwDrv);
#endif

#endif  /* TNETW_DRIVER_TYPES_H */

