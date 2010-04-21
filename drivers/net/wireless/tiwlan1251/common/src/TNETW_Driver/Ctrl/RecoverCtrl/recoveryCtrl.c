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


/*******************************************************************************/
/*                                                                             */
/*  MODULE:  recoveryCtrl.c                                                     */
/*  PURPOSE: The responsibility of RecoveryCtrl module is to provide main API   */
/*           to HelthMonitor that invokes the recovery process if failure is   */
/*           detected. It performs disable/enable inputs from outside, calls   */
/*           restart of TWD and informs STAD modules that recovery has been    */
/*           performed.                                                        */
/*                                                                             */
/*******************************************************************************/

#include "paramOut.h"
#include "osApi.h"
#include "report.h"
#include "recoveryCtrl.h"
#include "recoveryCtrl_API.h"

#include "TNETWIF.h"
#include "PowerAuthorization.h"
#include "ScanSrv.h"
#include "MeasurementSrv.h"
#include "PowerSrv_API.h"
#include "FwEvent_api.h"
#include "rxXfer_api.h"
#include "MacServices.h"
#include "txHwQueue_api.h"
#include "txXfer_api.h"
#include "txResult_api.h"
#include "CmdMBox_api.h"
#include "CmdQueue_api.h"
#include "whalParams.h"
#include "whalCtrl.h"
#include "whalSecurity.h"

#include "TNETWArb.h"
#include "ElpCtrl.h"
#include "HwInit_api.h"

/* static function */
#ifdef USE_RECOVERY
static void recoveryCtrl_SM(TI_HANDLE hRecoveryCtrl);
static int recoveryCtrl_ReConfig (TI_HANDLE hRecoveryCtrl);
static int recoveryCtrl_ReJoinBss(TI_HANDLE hRecoveryCtrl);
#endif /* USE_RECOVERY */

/*******************************************************************************
*                       PUBLIC  FUNCTIONS  IMPLEMENTATION                      *
********************************************************************************/


/*************************************************************************
*                        recoveryCtrl_create                              *
**************************************************************************
* DESCRIPTION:  This function initializes the RecoveryCtrl module.
*
* INPUT:        hOs - handle to Os Abstraction Layer
*               
* RETURN:       Handle to the allocated RecoveryCtrl module
*************************************************************************/
TI_HANDLE recoveryCtrl_create(TI_HANDLE hOs)
{
#ifdef USE_RECOVERY
    recoveryCtrl_t *hRecoveryCtrl;

    /* allocate RecoverCtrl module */
    hRecoveryCtrl = os_memoryAlloc(hOs, (sizeof(recoveryCtrl_t)));

    if(!hRecoveryCtrl)
    {
        WLAN_OS_REPORT(("Error allocating the RecoverCtrl Module\n"));
        return NULL;
    }

    /* Reset RecoverCtrl module */
    os_memoryZero(hOs, hRecoveryCtrl, (sizeof(recoveryCtrl_t)));

    hRecoveryCtrl->hOs = hOs;

    return(hRecoveryCtrl);
#else
    return NULL;
#endif /* USE_RECOVERY */
} /* recoveryCtrl_create */


/***************************************************************************
*                           recoveryCtrl_config                             *
****************************************************************************
* DESCRIPTION:  This function configures the recoveryCtrl module
*
* RETURNS:      OK - Configuration successful
*               NOK - Configuration unsuccessful
***************************************************************************/
TI_STATUS recoveryCtrl_config(TI_HANDLE hRecoveryCtrl, 
                              TI_HANDLE hReport,
                              TI_HANDLE hTNETWIF,
                              TI_HANDLE hTxXfer,     
                              TI_HANDLE hRxXfer,     
                              TI_HANDLE hTxResult,   
                              TI_HANDLE hMacServices,
                              TI_HANDLE hTxCtrlBlk,  
                              TI_HANDLE hTxHwQueue,  
                              TI_HANDLE hHalCtrl,    
                              TI_HANDLE hHwIntr,     
                              TI_HANDLE hWhalParams, 
                              TI_HANDLE hCmdQueue,   
                              TI_HANDLE hFwEvent,    
                              TI_HANDLE hCmdMBox,
                              TI_HANDLE hHwInit)
{
#ifdef USE_RECOVERY
    recoveryCtrl_t *pRecoveryCtrl = (recoveryCtrl_t *)hRecoveryCtrl;
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;
    MacServices_t *pMacServices = (MacServices_t*)hMacServices;

    /* configure modules handles */
    pRecoveryCtrl->hReport = hReport;
    pRecoveryCtrl->hTNETWIF = hTNETWIF;    
    pRecoveryCtrl->hTxXfer = hTxXfer;
    pRecoveryCtrl->hRxXfer = hRxXfer;     
    pRecoveryCtrl->hTxResult = hTxResult;   
    pRecoveryCtrl->hMacServices = hMacServices;
    pRecoveryCtrl->hTxCtrlBlk = hTxCtrlBlk;  
    pRecoveryCtrl->hTxHwQueue = hTxHwQueue;  
    pRecoveryCtrl->hHalCtrl = hHalCtrl;    
    pRecoveryCtrl->hHwIntr = hHwIntr;     
    pRecoveryCtrl->hWhalParams = hWhalParams; 
    pRecoveryCtrl->hCmdQueue = hCmdQueue;   
    pRecoveryCtrl->hFwEvent = hFwEvent;    
    pRecoveryCtrl->hCmdMBox = hCmdMBox;    
    pRecoveryCtrl->hHwInit = hHwInit;
    pRecoveryCtrl->hTNETWArb = pTNETWIF->hTNETWArb;

    pRecoveryCtrl->hELPCtrl = pTNETWIF->hELPCtrl;
    pRecoveryCtrl->hScanSRV = pMacServices->hScanSRV;
    pRecoveryCtrl->hMeasurementSRV = pMacServices->hMeasurementSRV;
    pRecoveryCtrl->hPowerSrv = pMacServices->hPowerSrv;
    pRecoveryCtrl->hPowerAutho = pMacServices->hPowerAutho;

    /* pRecoveryCtrl->hRecoveryMgr will be initialized while calling to recoveryCtrl_restartTWD() */

    pRecoveryCtrl->smState = REC_CTRL_STATE_IDLE;
    
    WLAN_REPORT_INIT(pRecoveryCtrl->hReport, RECOVERY_CTRL_MODULE_LOG,
        ("RecoveryCtrl configured successfully\n"));
#endif /* USE_RECOVERY */
    return OK;
} /* recoveryCtrl_config */


/***************************************************************************
*                           recoveryCtrl_destroy                            *
****************************************************************************
* DESCRIPTION:  This function unload the RecoverCtrl module. 
*
* INPUTS:       hRecoveryCtrl - the object
*
* OUTPUT:
*
* RETURNS:      OK - Unload succesfull
*               NOK - Unload unsuccesfull
***************************************************************************/
TI_STATUS recoveryCtrl_destroy(TI_HANDLE hRecoveryCtrl)
{
#ifdef USE_RECOVERY
    recoveryCtrl_t *pRecoveryCtrl = (recoveryCtrl_t *)hRecoveryCtrl;

    /* free RecoverCtrl Module */
    os_memoryFree(pRecoveryCtrl->hOs, pRecoveryCtrl, sizeof(recoveryCtrl_t));
#endif /* USE_RECOVERY */
    return OK;
}

/**********************************************************************************************
 *                  recoveryCtrl_SM()
 **********************************************************************************************
 * DESCRIPTION:  
   ============
    This is the recoveryCtrl state machine.
    The inceptive event for RecoveryCtrl SM is invoking the restart of TWD by RecoveryMgr;
    Perform ASYNC restart of BusTxn (if not idle then wait until end of current txn and 
    Invoke CB upon TxnComplete) and FW Download;
    HwInit module performs HW Init process;
    Call RecoverMgr CB endOfRecovery() at the end of TWD restart

    The SM supports both Sync and Async accesses to the HW.
    It loops and progresses from state to state as long as the HW is accessed synchronously.
    Once the access is Asynchronous (TNETWIF_PENDING), it exits and is called later
      by the TNETWIF when the HW is ready.
    That's why it uses unspecified-mode accesses (e.g. TNETWIF_ReadMemOpt) which
      selects either Sync or Async automatically according to the platform and length.
    Yet, the short transactions (EOB and Interrupt-Request 32 bit writes) are done using Sync 
      access to simplify the SM 
    NOTE: MCS projects may require full Sync/Async support, so the Sync accesses may need to be modified. 

    NOTE:  The recoveryCtrl-SM detailed description is provided in "CE-2.0 Recovery LLD.doc".

 **********************************************************************************************/
#ifdef USE_RECOVERY
static void recoveryCtrl_SM(TI_HANDLE hRecoveryCtrl)
{
    recoveryCtrl_t *pRecoveryCtrl = (recoveryCtrl_t *)hRecoveryCtrl;
    TI_STATUS tnetwifStatus = TNETWIF_ERROR; /* Last HW operation status: Complete (Sync) or Pending (Async). */
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)pRecoveryCtrl->hHalCtrl;

#ifdef TI_DBG
    if (hRecoveryCtrl == NULL)
    {
        WLAN_REPORT_ERROR(pRecoveryCtrl->hReport, RECOVERY_CTRL_MODULE_LOG,  
            ("recoveryCtrl_SM(): ****  Called with NULL handle!!  ****\n"));
        return;
    }
#endif
    
    /* 
     * Loop through the states sequence as long as the process is synchronous.
     * Exit when finished or if an Asynchronous process is required. In this case
     *   the SM process will be resumed later (called back by TNETWIF). 
     */
    while (1)
    {
        switch (pRecoveryCtrl->smState)
        {
            case REC_CTRL_STATE_IDLE:
                FwEvent_Stop(pRecoveryCtrl->hFwEvent);

                tnetwifStatus = TNETWIF_COMPLETE;

                pRecoveryCtrl->smState = REC_CTRL_STATE_WAIT_END_CURR_TXN;
                break;

            case REC_CTRL_STATE_WAIT_END_CURR_TXN:
                TNETWArb_Recovery(pRecoveryCtrl->hTNETWArb, pRecoveryCtrl->hELPCtrl);

                elpCtrl_Stop(pRecoveryCtrl->hELPCtrl);
                CmdMBox_Restart(pRecoveryCtrl->hCmdMBox);
                CmdQueue_StartReconfig(pRecoveryCtrl->hCmdQueue);

                pRecoveryCtrl->smState = REC_CTRL_STATE_INIT_CMPLT;
                tnetwifStatus = hwInit_recovery(pRecoveryCtrl->hHwInit, (TI_HANDLE)(pWhalCtrl->pHwCtrl), (void*)recoveryCtrl_SM, hRecoveryCtrl);
                return;
    
            case REC_CTRL_STATE_INIT_CMPLT:
                elpCtrl_Start(pRecoveryCtrl->hELPCtrl);

                /* reconfig FW */
                recoveryCtrl_ReConfig(hRecoveryCtrl);

                pRecoveryCtrl->smState = REC_CTRL_STATE_END_RECONFIG;
                whal_hwInfoElemMiscTableGet (pWhalCtrl->pHwCtrl->pHwMboxConfig, 
                                                    &pWhalCtrl->misc,
                                                    (void *)recoveryCtrl_SM,
                                                    hRecoveryCtrl);
                return;

            case REC_CTRL_STATE_END_RECONFIG:
                powerAutho_Restart(pRecoveryCtrl->hMacServices);
                RxXfer_ReStart(pRecoveryCtrl->hRxXfer);
                scanSRV_restart(pRecoveryCtrl->hScanSRV);
                measurementSRV_restart(pRecoveryCtrl->hMeasurementSRV);
                powerSrv_restart(pRecoveryCtrl->hPowerSrv);

                /*Change the State of the mboxQueue and the interrupt Module */
/* moved later              whalCtrl_exitFromInitMode(pRecoveryCtrl->hHalCtrl);*/ /* call inside CmdMBox_SetModeNormal */

                /* 
                Indicates the MboxQueue that Reconfig Ended in Order To Call the CallBacks
                That Was saved before the recovery process started 
                */
                CmdQueue_EndReconfig(pRecoveryCtrl->hCmdQueue);

                pRecoveryCtrl->smState = REC_CTRL_STATE_IDLE;

                /* call End Of Recovery CB */
                pRecoveryCtrl->endOfRecoveryCB(pRecoveryCtrl->hRecoveryMgr);

                return; /* recovery process ended */
    
            default:
                WLAN_REPORT_ERROR(pRecoveryCtrl->hReport, RECOVERY_CTRL_MODULE_LOG,  
                        ("recoveryCtrl_SM(): Unexpected state, smState=%d\n", pRecoveryCtrl->smState));
                return;

        }  /* switch (pRecoveryCtrl->smState) */

        WLAN_REPORT_INFORMATION(pRecoveryCtrl->hReport, RECOVERY_CTRL_MODULE_LOG,  
            ("recoveryCtrl_SM(): smState=%d\n", pRecoveryCtrl->smState));

        /* 
         * If the last HW access request was pended, exit the SM (Asynchronous process).
         * The SM will be called back when the HW access is done.
         * Also reset the Sync flag to notify that the Xfer wasn't completed in the SendPacket context.
         */
        if (tnetwifStatus == TNETWIF_PENDING)
        {
            return;  /**********    Exit State Machine (to be called back by TNETWIF)   **********/
        }

#ifdef TI_DBG
        else if (tnetwifStatus == TNETWIF_ERROR)
        {
            WLAN_REPORT_ERROR(pRecoveryCtrl->hReport, RECOVERY_CTRL_MODULE_LOG,  
                ("recoveryCtrl_SM(): Unexpected state, smState=%d\n", pRecoveryCtrl->smState));
            return;
        }
#endif /* TI_DBG */
    }  /* while (1) */
} /* recoveryCtrl_SM */
#endif /* USE_RECOVERY */

/***************************************************************************
*                           recoveryCtrl_restartTWD                        *
****************************************************************************
* DESCRIPTION:  Start TWD recovery. Restart TWD from bottom up.
*               Init HW: (using HW init module in FW Transfer component).
*               Reconfigure FW.
*
* INPUTS:       hRecoveryCtrl - the object
*
* OUTPUT:
*
* RETURNS:      OK - succesfull
*               NOK - unsuccesfull
***************************************************************************/
TI_STATUS recoveryCtrl_restartTWD(TI_HANDLE hRecoveryCtrl,void *endOfRecoveryCB, TI_HANDLE hRecoveryMgr)
{
#ifdef USE_RECOVERY
    recoveryCtrl_t *pRecoveryCtrl = (recoveryCtrl_t *)hRecoveryCtrl;

    /* register RecoveryMgr handle and endOfRecoveryCB*/
    pRecoveryCtrl->hRecoveryMgr = hRecoveryMgr;
    pRecoveryCtrl->endOfRecoveryCB = (EndOfRecoveryCB_t)endOfRecoveryCB;

    recoveryCtrl_SM(hRecoveryCtrl);
#endif /* USE_RECOVERY */
    return OK;
}

#ifdef USE_RECOVERY
int recoveryCtrl_ReConfig (TI_HANDLE hRecoveryCtrl)
{
    recoveryCtrl_t *pRecoveryCtrl = (recoveryCtrl_t *)hRecoveryCtrl;
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)pRecoveryCtrl->hHalCtrl;
    WlanParams_T *pWlanParams = whal_ParamsGetWlanParams(pWhalCtrl->pWhalParams);
    whalParamInfo_t param;
    DmaParams_T     *pDmaParams     = whal_ParamsGetDmaParams(pWhalCtrl->pWhalParams);

    int index = 0;
    int Stt;
    
    if(!pWlanParams->RecoveryEnable)
    {
        WLAN_OS_REPORT(("whalCtrl_ReConfig: Recovery is disabled in registry, abort recovery process\n"));
        return OK;
    }

    FwEvent_SetHwInfo (pWhalCtrl->hFwEvent, &(pWhalCtrl->pHwCtrl->DataPathParams));
    txXfer_restart(pWhalCtrl->hTxXfer); 
    txResult_restart(pWhalCtrl->hTxResult); 

    txXfer_setHwInfo (pWhalCtrl->hTxXfer, &(pWhalCtrl->pHwCtrl->DataPathParams));    

    txResult_setHwInfo (pWhalCtrl->hTxResult, &(pWhalCtrl->pHwCtrl->DataPathParams));

    rxXfer_SetDoubleBufferAddr (pWhalCtrl->hRxXfer, &(pWhalCtrl->pHwCtrl->DataPathParams)); 

    /* Provide number of HW Tx-blocks and descriptors to Tx-HW-Queue module */
    txHwQueue_setHwInfo (pWhalCtrl->hTxHwQueue, pDmaParams);

    whalSecur_HwEncDecrEnable (pWhalCtrl->pWhalSecurity, 1);

    /*send the table regardless to the state */
    whal_hwCtrl_SetBeaconFilterIETable(pWhalCtrl->pHwCtrl, 
                                       &pWhalCtrl->pWhalParams->WlanParams.beaconFilterIETable.numberOfIEs, 
                                       pWhalCtrl->pWhalParams->WlanParams.beaconFilterIETable.IETable, 
                                       &pWhalCtrl->pWhalParams->WlanParams.beaconFilterIETable.IETableSize );

    /*
     * ReConfig the wlan hardware Queues according to the required Quality Of Service.
     */
    /* Reconfig Roaming thresholds */
    /* SNR */
    whalCtrl_SetSNRParamsCmd (pWhalCtrl, &pWhalCtrl->pWhalParams->WlanParams.roamTriggers);

    /* RSSI */
    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, RECOVERY_CTRL_MODULE_LOG,
                            ("\n SetRSSIParamsCmd :\n \
                              RSSIthreshold = %d\n \
                              RSSIFilterWeight = %d\n \
                              RSSIFilterDepth = %d \n ",
                              pWhalCtrl->pWhalParams->WlanParams.roamTriggers.rssiThreshold,
                              pWhalCtrl->pWhalParams->WlanParams.roamTriggers.rssiFilterWeight,
                              pWhalCtrl->pWhalParams->WlanParams.roamTriggers.rssiFilterDepth));
    whalCtrl_SetRSSIParamsCmd(pWhalCtrl, &pWhalCtrl->pWhalParams->WlanParams.roamTriggers);

    /* Max consecutive NACK */
    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, RECOVERY_CTRL_MODULE_LOG,
                            ("\n SetMaxTxRetryParamsCmdCmd :\n \
                              maxTxRetry = %d \n ",
                              pWhalCtrl->pWhalParams->WlanParams.roamTriggers.maxTxRetry));
    whalCtrl_SetMaxTxRetryParamsCmd(pWhalCtrl, &pWhalCtrl->pWhalParams->WlanParams.roamTriggers);
    /* Out of sync */
    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, RECOVERY_CTRL_MODULE_LOG,
                            ("\n whalCtrl_SetBssLossTsfThresholdParamsCmd :\n \
                              BssLossTimeout = %d\n \
                              TsfMissThreshold = %d \n ",
                              pWhalCtrl->pWhalParams->WlanParams.roamTriggers.BssLossTimeout,
                              pWhalCtrl->pWhalParams->WlanParams.roamTriggers.TsfMissThreshold));
    whalCtrl_SetBssLossTsfThresholdParamsCmd(pWhalCtrl, &pWhalCtrl->pWhalParams->WlanParams.roamTriggers);

    whalCtrl_setBetParams(pWhalCtrl,
                          pWhalCtrl->pWhalParams->WlanParams.BetEnable, 
                          pWhalCtrl->pWhalParams->WlanParams.MaximumConsecutiveET);

    whalCtrl_setRxDataFiltersParams(pWhalCtrl, 
                                    pWhalCtrl->pWhalParams->WlanParams.rxFilterDefaultEnable, 
                                    pWhalCtrl->pWhalParams->WlanParams.rxFilterDefaultAction);

    for (index = 0; index < MAX_DATA_FILTERS; index++)
    {
        if (pWhalCtrl->pWhalParams->WlanParams.rxFilterCgf[index].rxFilterCommand == ADD_FILTER)
        {
            whalCtrl_setRxDataFilter(pWhalCtrl, 
                                     index,
                                     pWhalCtrl->pWhalParams->WlanParams.rxFilterCgf[index].rxFilterCommand,
                                     pWhalCtrl->pWhalParams->WlanParams.rxFilterCgf[index].rxFilterAction,
                                     pWhalCtrl->pWhalParams->WlanParams.rxFilterCgf[index].rxFilterNumFieldPatterns, 
                                     pWhalCtrl->pWhalParams->WlanParams.rxFilterCgf[index].rxFilterLenFieldPatterns, 
                                     pWhalCtrl->pWhalParams->WlanParams.rxFilterCgf[index].rxFilterFieldPatterns);
        }
    }

#ifdef BTH_COEXISTENCE /* it's solution for collision of BTH and WLAN (by Gemini protocol), we don't need it */
    /* Soft Gemini Section */
    /* ReConfig the BTH enable */
    param.paramType = HAL_CTRL_SG_ENABLE_PARAM;
    param.content.SoftGeminiEnable = pWhalCtrl->pWhalParams->WlanParams.SoftGeminiEnable;
    whalCtrl_SetParam(pRecoveryCtrl->hHalCtrl, &param);

    /* ReConfig the BTH config */
    param.paramType = HAL_CTRL_SG_CONFIG_PARAM;
    os_memoryCopy(pWhalCtrl->hOs, &param.content.SoftGeminiParam, &pWhalCtrl->pWhalParams->WlanParams.SoftGeminiParams, sizeof(SoftGeminiParam_t));
    whalCtrl_SetParam(pRecoveryCtrl->hHalCtrl, &param);
#endif


#ifdef DO_CALIBRATION_IN_DRIVER
    /* Perform single calibration for APP scan usage. */
    whalCtrl_sendRadioAction(pWhalCtrl ,MANUAL_CALIB);
#endif/* O_CALIBRATION_IN_DRIVER */
    /*
     * JOIN (use the local parameters), otherwize the CORE will reconnect
     */
    if (pWlanParams->bJoin)
    {
        /* set TxRatePolicy */
        param.paramType = HAL_CTRL_TX_RATE_CLASS_PARAMS;
        param.content.pTxRatePlicy = &pWhalCtrl->pWhalParams->BssInfoParams.TxRateClassParams;
    
        whalCtrl_SetParam(pRecoveryCtrl->hHalCtrl,&param);
        
        Stt = recoveryCtrl_ReJoinBss(hRecoveryCtrl);
        if (Stt != OK)
        {
            WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, RECOVERY_CTRL_MODULE_LOG,
                ("whalCtrl_ReConfig: recoveryCtrl_ReJoinBss failed\n"));
            return NOK;
        }
        whal_hwCtrl_AidSet (pWhalCtrl->pHwCtrl, pWhalCtrl->pWhalParams->WlanParams.Aid);
        
    
        /* Slot time must be setting after doing join */
        whal_hwCtrl_SetSlotTime(pWhalCtrl->pHwCtrl, (slotTime_e)pWhalCtrl->pWhalParams->WlanParams.SlotTime);               

            /* Reconfig security keys, default key Id and encryption/decryption control to the FW*/
        if (whalSecur_KeysReconfig (pWhalCtrl->pWhalSecurity) != OK)
        {
            WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, RECOVERY_CTRL_MODULE_LOG,
                ("whalCtrl_ReConfig: ## whalSecur_KeysReconfig failed\n"));
            return NOK;
        }
    }

    return (OK);
}


/*
 * ----------------------------------------------------------------------------
 * Function : recoveryCtrl_ReJoinBss
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */

int recoveryCtrl_ReJoinBss (TI_HANDLE hRecoveryCtrl)
{
    recoveryCtrl_t *pRecoveryCtrl = (recoveryCtrl_t *)hRecoveryCtrl;
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)pRecoveryCtrl->hHalCtrl;
    BssInfoParams_T *pBssInfoParams = &pWhalCtrl->pWhalParams->BssInfoParams;
    TemplateListParams_T *pWhalTemplates = &pWhalCtrl->pWhalParams->TemplateList;
    HwMboxCmd_T *pHwCmd = whal_hwCtrl_GetMboxCmd(pWhalCtrl->pHwCtrl);

    /*
     * set the templates 
     */

    if (pWhalTemplates->Beacon.Size != 0)
        whal_hwMboxCmd_ConfigureTemplateFrame (pHwCmd, 
                                               pWhalTemplates->Beacon.Buffer, 
                                               (UINT16)pWhalTemplates->Beacon.Size,
                                               CMD_BEACON,
                                               NULL,
                                               NULL);
    
    if (pWhalTemplates->ProbeResp.Size != 0)
        whal_hwMboxCmd_ConfigureTemplateFrame (pHwCmd, 
                                               pWhalTemplates->ProbeResp.Buffer, 
                                               (UINT16)pWhalTemplates->ProbeResp.Size,
                                               CMD_PROBE_RESP,
                                               NULL,
                                               NULL);

    if (pWhalTemplates->ProbeReq.Size != 0)
        whal_hwMboxCmd_ConfigureTemplateFrame (pHwCmd, 
                                               pWhalTemplates->ProbeReq.Buffer, 
                                               (UINT16)pWhalTemplates->ProbeReq.Size,
                                               CMD_PROBE_REQ,
                                               NULL,
                                               NULL);
    
    if (pWhalTemplates->NullData.Size != 0)
        whal_hwMboxCmd_ConfigureTemplateFrame (pHwCmd, 
                                               pWhalTemplates->NullData.Buffer, 
                                               (UINT16)pWhalTemplates->NullData.Size,
                                               CMD_NULL_DATA,
                                               NULL,
                                               NULL);
    
    if (pWhalTemplates->PsPoll.Size != 0)
        whal_hwMboxCmd_ConfigureTemplateFrame (pHwCmd, 
                                               pWhalTemplates->PsPoll.Buffer, 
                                               (UINT16)pWhalTemplates->PsPoll.Size,
                                               CMD_PS_POLL,
                                               NULL,
                                               NULL);

    if (pWhalTemplates->QosNullData.Size != 0)
        whal_hwMboxCmd_ConfigureTemplateFrame (pHwCmd, 
                                               pWhalTemplates->QosNullData.Buffer,
                                               (UINT16)pWhalTemplates->QosNullData.Size,
                                               CMD_QOS_NULL_DATA,
                                               NULL,
                                               NULL);


    /*
     * call the hardware to start/join the bss 
     */
    return whal_hwCtrl_StartJoin(pWhalCtrl->pHwCtrl, (bssType_e)pBssInfoParams->ReqBssType, NULL, NULL);

}
#endif

