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

#include "whalCommon.h"
#include "whalCtrl_api.h"
#include "whalCtrl.h"
#include "txHwQueue_api.h"
#include "txXfer_api.h"
#include "txResult_api.h"
#include "whalSecurity.h"
#include "eventMbox_api.h" 
#include "whalBus_Api.h"
#include "TNETW_Driver_api.h"
#include "commonTypes.h"
#include "TNETW_Driver.h"
#include "DebugTraceXfer_api.h"
#include "FwEvent_api.h"

/* 
    Rx filter field is mostly hard-coded.
   This filter value basically pass only valid beacons / probe responses. For exact bit description,
   consult either the DPG or the FPG (or both, and Yoel...)
*/
#define CFG_RX_PRSP_EN_ 4
#define CFG_RX_MGMT_EN_ 0x10
#define CFG_RX_BCN_EN_  0x200
#define RX_FILTER_CFG_ (CFG_RX_PRSP_EN_ | CFG_RX_MGMT_EN_ | CFG_RX_BCN_EN_)

#define SIZE_OF_HEADER 4

void whalCtrl_PrintAll (TI_HANDLE hWhalCtrl);


#ifdef ROAMING_TRIGGER_DBG
static void whalCtrl_dbg_RSSI_LEVEL(TI_HANDLE hWhalCtrl,char* str , UINT32 strLen);
TI_STATUS whalCtrl_dbg_SYNCHRONIZATION(TI_HANDLE hWhalCtrl);
TI_STATUS whalCtrl_dbg_BSS_LOSE(TI_HANDLE hWhalCtrl);
TI_STATUS whalCtrl_dbg_MAX_TX_RETRY(TI_HANDLE hWhalCtrl);
TI_STATUS whalCtrl_dbgRegisterRoamingEventCB(TI_HANDLE hWhalCtrl);
int whalCtrl_dbgRoamingCommands (TI_HANDLE hWhalCtrl);
#endif

static int whalCtrl_ReadTemplateFrameMib(TI_HANDLE hWhalCtrl, TI_HANDLE CB_Handle, void* CB_Func, void* CB_Buf);
static int whalCtrl_WriteTemplateFrameMib(TI_HANDLE hWhalCtrl, PLT_MIB_t* pMib);
static int whalCtrl_PltMibSetBeaconFilterIETable(TI_HANDLE hWhalCtrl, PLT_MIB_t* pMib);
static int whalCtrl_PltMibGetBeaconFilterIETable(TI_HANDLE hWhalCtrl, TI_HANDLE CB_Handle, void* CB_Func, void* CB_Buf);
static int whalCtrl_PLT_ReadMIB_TxRatePolicy(TI_HANDLE hWhalCtrl, TI_HANDLE CB_Handle, void* CB_Func, void* CB_Buf);
static int whalCtrl_PLT_WriteMIB_TxRatePolicy(TI_HANDLE hWhalCtrl, PLT_MIB_t* pMib);

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_Create
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
TI_HANDLE whalCtrl_Create (TI_HANDLE hOs)
{
    WHAL_CTRL       *pWhalCtrl;
    WlanParams_T    *pWlanParams;
    DmaParams_T     *pDmaParams;

    /* 
    allocate the HAL Ctrl 
    */
    pWhalCtrl = (WHAL_CTRL *)os_memoryAlloc (hOs, sizeof(WHAL_CTRL));
    if (pWhalCtrl == NULL)
    {
        WLAN_OS_REPORT(("whalCtrl_Create: Error memory Allocation\n"));
        return NULL;
    }
    os_memoryZero (hOs, (void *)pWhalCtrl, sizeof(WHAL_CTRL));

    pWhalCtrl->hOs = hOs;
    pWhalCtrl->EncDecEnableFlag = FALSE;
    
    
    /* 
    Create the Params object
    */
    pWhalCtrl->pWhalParams = whal_params_Create (hOs,TRUE);
    if (pWhalCtrl->pWhalParams == NULL)
    {
        WLAN_OS_REPORT(("whalCtrl_Create: Error whal_params_Create\n"));
        whalCtrl_Destroy(pWhalCtrl);
        return NULL;
    }

    /* Initialize the Params object database fields*/
    pWlanParams = whal_ParamsGetWlanParams(pWhalCtrl->pWhalParams);
    pWlanParams->hwAccessMethod = HW_ACCESS_BUS_SLAVE_INDIRECT;
    pWlanParams->maxSitesFragCollect = HAL_CTRL_SITE_FRAG_COLLECT_DEF;
    pWlanParams->RtsThreshold       = HAL_CTRL_RTS_THRESHOLD_DEF; 
    pWlanParams->bJoin              = FALSE;

    /*soft gemini defaults*/
    pWlanParams->SoftGeminiEnable   = SG_DISABLE;
     

    /*becon filter defaults*/
    pWlanParams->beaconFilterParams.desiredState    = FALSE;
    pWlanParams->beaconFilterParams.numOfElements   = DEF_NUM_STORED_FILTERS;
    pWlanParams->beaconFilterIETable.numberOfIEs    = DEF_BEACON_FILTER_IE_TABLE_NUM;
    pWlanParams->beaconFilterIETable.IETableSize        = BEACON_FILTER_IE_TABLE_DEF_SIZE;

    
    /* set the dma params */
    pDmaParams = whal_ParamsGetDmaParams(pWhalCtrl->pWhalParams);
    whal_ParamsSetDmaParams(pWhalCtrl->pWhalParams);
    
    

    /* 
    Configure the hardware control object 
    */
    pWhalCtrl->pHwCtrl = whal_hwCtrl_Create(hOs, pWhalCtrl->pWhalParams);   
    if (pWhalCtrl->pHwCtrl == NULL)
    {
        WLAN_OS_REPORT(("whalCtrl_Create: Error whal_hwCtrl_Create\n"));
        whalCtrl_Destroy(pWhalCtrl);
        return NULL;
    }
    
    /* set the Roaming  params */
    /* Configure  the Low RSSI, the Low SNR and the Missed beacon Defaults */
    whal_ParamsSetRoamingParams(pWhalCtrl->pWhalParams);

    
    /* 
    Create the Security Object
    */
    pWhalCtrl->pWhalSecurity = whalSecur_Create (hOs, pWhalCtrl, pDmaParams->NumStations);

    if (pWhalCtrl->pWhalSecurity == NULL)
    {
        WLAN_OS_REPORT(("whalCtrl_Create: Error whalSecur_Create\n"));
        whalCtrl_Destroy(pWhalCtrl);
        return NULL;
    }
    

    WLAN_INIT_REPORT(("whalCtrl_Create end %x\n",(TI_HANDLE)pWhalCtrl));

    return (pWhalCtrl);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_GetSetHwAddr
 *
 * Input    :
 * Output   :
 * Process  :  Configure the hardware control object
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_GetSetHwAddr (WHAL_CTRL* pWhalCtrl, TI_HANDLE hMemMgr,UINT32 *pFWImage)
{
    int Status;
    UINT32 AcxRegAddr;
    UINT32 AcxMemAddr;

    /*
     * Initiate and get the wlan hardware register and memory addresses
     */
    AcxRegAddr = (UINT32)os_hwGetRegistersAddr(pWhalCtrl->hOs);
    AcxMemAddr = (UINT32)os_hwGetMemoryAddr(pWhalCtrl->hOs);

    /* Get the handle of the Interrupt handler object */
    pWhalCtrl->hWhalBus = whal_hwCtrl_GetBusHandle(pWhalCtrl->pHwCtrl);

    /*
     * Configure wlan hardware control object
     */
    Status = whal_hwCtrl_Config( pWhalCtrl->pHwCtrl,(TI_HANDLE)pWhalCtrl, 
                                 whal_ParamsGetWlanParams(pWhalCtrl->pWhalParams)->hwAccessMethod, 
                                 AcxRegAddr, 
                                 AcxMemAddr, 
                                 pWhalCtrl->hReport, 
                                 hMemMgr, 
                                 pFWImage,

                                 pWhalCtrl->hEventMbox);
    if (Status == TNETWIF_ERROR)
    {
        WLAN_REPORT_ERROR(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,  (" whalCtrl_GetSetHwAddr: whal_hwCtrl_Config failure \n"));
    }
    
    return Status;
}


typedef int (*fcallback_t) (TI_HANDLE);


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_ConfigHwCb2
 *
 * Input    :
 * Output   :
 * Process  :  last configuration call to the low level hal
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
static int whalCtrl_ConfigHwCb2 (TI_HANDLE hWhalCtrl, TI_STATUS status)
{
    WHAL_CTRL    *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    if (status != OK)
    {
        return NOK;
    }

    /* Call the upper layer callback */
    return (*((fcallback_t)pWhalCtrl->fCb)) (pWhalCtrl->hCb);
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_ConfigHwCb1
 *
 * Input    :
 * Output   :
 * Process  :  last configuration call to the low level hal
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
static int whalCtrl_ConfigHwCb1 (TI_HANDLE hWhalCtrl, TI_STATUS status)
{
    WHAL_CTRL    *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    DmaParams_T  *pDmaParams = whal_ParamsGetDmaParams (pWhalCtrl->pWhalParams);

    /* 
     * Store the addresses of the double buffer (Rx/Tx) 
     * and the path status and control (Tx/Rx) in the corresponding modules
     */
	FwEvent_SetHwInfo (pWhalCtrl->hFwEvent, &(pWhalCtrl->pHwCtrl->DataPathParams));

    txXfer_setHwInfo (pWhalCtrl->hTxXfer, &(pWhalCtrl->pHwCtrl->DataPathParams));    

    txResult_setHwInfo (pWhalCtrl->hTxResult, &(pWhalCtrl->pHwCtrl->DataPathParams));

    rxXfer_SetDoubleBufferAddr (pWhalCtrl->hRxXfer, &(pWhalCtrl->pHwCtrl->DataPathParams)); 

    /* Provide number of HW Tx-blocks and descriptors to Tx-HW-Queue module */
    txHwQueue_setHwInfo (pWhalCtrl->hTxHwQueue, pDmaParams);

  #ifdef TI_DBG
    /* Set the debug trace addresses */
    debugTrace_ConfigHw (pWhalCtrl->hDebugTrace,
                         (UINT32)pWhalCtrl->pHwCtrl->MemMap.debugBuffer1Start,
                         (UINT32)pWhalCtrl->pHwCtrl->MemMap.debugBuffer2Start);
  #endif /* TI_DBG */

    /* 
     * Register all health/sanityCheck/Recovery callbacks 
     * --------------------------------------------------
     *      DeviceError - CORE callback for full recovery
     *      HealthReport - Event Mbox callback for extracting device error
     *      MacStatus - Event Mbox callback for checkHwStatus FCS_ERR, RX_FREE_MEM regs
     *      AciIndication - Event Mbox callback for accumulate SCR_PAD8 image
     *      Mailbox error - Mailbox queue callback for case of timeout/error  
     *      Power control error - PowerMngr callback in case of power ctrl timeout 
     *      Failure event - CORE Callback in case of MailboxError or PowerCtrl error 
     */
  #ifdef USE_RECOVERY
    whalCtrl_RegisterErrorsCallbacks (hWhalCtrl);
  #endif

  #ifdef ROAMING_TRIGGER_DBG
    whalCtrl_dbgRegisterRoamingEventCB (hWhalCtrl);
    whalCtrl_dbgRoamingCommands (hWhalCtrl);
  #endif

    return whal_hwInfoElemMiscTableGet (pWhalCtrl->pHwCtrl->pHwMboxConfig, 
                                        &pWhalCtrl->misc,
                                        (void *)whalCtrl_ConfigHwCb2,
                                        hWhalCtrl);
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_ConfigHw
 *
 * Input    :
 * Output   :
 * Process  :  last configuration call to the low level hal
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_ConfigHw 
(
    TI_HANDLE              hWhalCtrl, 
    TnetwDrv_InitParams_t *pInitParams, 
    void                  *fCb,
    TI_HANDLE              hCb
)
{
    WHAL_CTRL       *pWhalCtrl      = (WHAL_CTRL *)hWhalCtrl;
    TxParam_t       *pTxParams      = whal_ParamsGetTxParams (pWhalCtrl->pWhalParams);
    WlanParams_T    *pWlanParams    = whal_ParamsGetWlanParams (pWhalCtrl->pWhalParams);
    UINT8           acID;
    int k = 0;

    pWhalCtrl->fCb = fCb;
    pWhalCtrl->hCb = hCb;
       
    if (NULL != pInitParams)
    {
    /* whalCtrl_init_t */
    pWlanParams->PacketDetectionThreshold   = pInitParams->whalCtrl_init.packetDetectionThreshold;
    pWlanParams->qosNullDataTemplateSize    = pInitParams->whalCtrl_init.qosNullDataTemplateSize;
    pWlanParams->PsPollTemplateSize         = pInitParams->whalCtrl_init.PsPollTemplateSize;
    pWlanParams->probeResponseTemplateSize  = pInitParams->whalCtrl_init.probeResponseTemplateSize;
    pWlanParams->probeRequestTemplateSize   = pInitParams->whalCtrl_init.probeRequestTemplateSize;
    pWlanParams->beaconTemplateSize         = pInitParams->whalCtrl_init.beaconTemplateSize;
    pWlanParams->nullTemplateSize           = pInitParams->whalCtrl_init.nullTemplateSize;
    /* Beacon broadcast options */
    pWlanParams->BcnBrcOptions.BeaconRxTimeout      = pInitParams->whalCtrl_init.BeaconRxTimeout;
    pWlanParams->BcnBrcOptions.BroadcastRxTimeout   = pInitParams->whalCtrl_init.BroadcastRxTimeout;
    pWlanParams->BcnBrcOptions.RxBroadcastInPs      = pInitParams->whalCtrl_init.RxBroadcastInPs;

    pWlanParams->ConsecutivePsPollDeliveryFailureThreshold = 
        pInitParams->whalCtrl_init.ConsecutivePsPollDeliveryFailureThreshold;

    pTxParams->txCompleteTimeout            = pInitParams->whalCtrl_init.txCompleteTimeout;
    pTxParams->txCompleteThreshold          = pInitParams->whalCtrl_init.txCompleteThreshold;
    
    /* halCtrlConfigParams_t */
    pWlanParams->RxEnergyDetection          = pInitParams->halCtrlConfigParams.halCtrlRxEnergyDetection;
    pWlanParams->TxEnergyDetection          = pInitParams->halCtrlConfigParams.halCtrlTxEnergyDetection;
    pWlanParams->ACIMode                    = pInitParams->halCtrlConfigParams.halCtrlACIMode;     
    pWlanParams->inputCCA                   = pInitParams->halCtrlConfigParams.halCtrlInputCCA;    
    pWlanParams->qualifiedCCA               = pInitParams->halCtrlConfigParams.halCtrlQualifiedCCA;
    pWlanParams->stompForRx                 = pInitParams->halCtrlConfigParams.halCtrlStompForRx;  
    pWlanParams->stompForTx                 = pInitParams->halCtrlConfigParams.halCtrlStompForTx;  
    pWlanParams->txCCA                      = pInitParams->halCtrlConfigParams.halCtrlTxCCA;       
    pWlanParams->RxDisableBroadcast         = pInitParams->halCtrlConfigParams.halCtrlRxDisableBroadcast;
    pWlanParams->calibrationChannel2_4      = pInitParams->halCtrlConfigParams.halCtrlCalibrationChannel2_4;
    pWlanParams->calibrationChannel5_0      = pInitParams->halCtrlConfigParams.halCtrlCalibrationChannel5_0;

    /* Not used but need by Palau */
    pWlanParams->RtsThreshold               = pInitParams->halCtrlConfigParams.halCtrlRtsThreshold;
    pWlanParams->CtsToSelf                  = CTS_TO_SELF_DISABLE; 
    
    pWlanParams->WiFiWmmPS 					= pInitParams->halCtrlConfigParams.WiFiWmmPS;

    pWlanParams->MaxTxMsduLifetime          = pInitParams->halCtrlConfigParams.halCtrlMaxTxMsduLifetime;  
    pWlanParams->MaxRxMsduLifetime          = pInitParams->halCtrlConfigParams.halCtrlMaxRxMsduLifetime;  

    pWlanParams->rxTimeOut.psPoll           = pInitParams->halCtrlConfigParams.rxTimeOut.psPoll;  
    pWlanParams->rxTimeOut.UPSD             = pInitParams->halCtrlConfigParams.rxTimeOut.UPSD;  

    /* No used */
    pWlanParams->FragmentThreshold          = pInitParams->halCtrlConfigParams.halCtrlFragThreshold;
    pWlanParams->ListenInterval             = (UINT8)pInitParams->halCtrlConfigParams.halCtrlListenInterval;
    pWlanParams->RateFallback               = pInitParams->halCtrlConfigParams.halCtrlRateFallbackRetry;        
    pWlanParams->MacClock                   = pInitParams->halCtrlConfigParams.halCtrlMacClock;     
    pWlanParams->ArmClock                   = pInitParams->halCtrlConfigParams.halCtrlArmClock;     
    pWlanParams->Enable4x                   = pInitParams->halCtrlConfigParams.halCtrlEnable4x;
    pWlanParams->TxCompleteThreshold        = pInitParams->halCtrlConfigParams.halCtrlTxCompleteThreshold;
    
    /* Configure ARP IP */
    
    pWlanParams->isArpIpFilteringEnabled =  pInitParams->arpIpFilterParams.isFilterEnabled ;
    os_memoryCopy(pWhalCtrl->hOs,(void *)pWlanParams->arp_IP_addr.addr,(void *)pInitParams->arpIpFilterParams.arpIpInitParams.addr , IP_V4_ADDR_LEN) ;
    
    /* Configure address group */
    pWlanParams->numGroupAddrs = pInitParams->macAddrFilterParams.numOfMacAddresses;
    pWlanParams->isMacAddrFilteringnabled = pInitParams->macAddrFilterParams.isFilterEnabled;
    
    for (k = 0; k < pWlanParams->numGroupAddrs; k++)
    {
            os_memoryCopy (pWhalCtrl->hOs,(void*)pWlanParams->Group_addr[k].addr, (void*)pInitParams->macAddrFilterParams.macAddrTable[k].addr, MAC_ADDR_LEN);
    }

    /* Configure beacon timing (early wakeup parmeter) */
    pWlanParams->earlyWakeUp = pInitParams->macPreambleParams.earlyWakeUp;

    /* QoS configure queue */
    for (acID = 0; acID < MAX_NUM_OF_AC; acID++)
    {
        /*
         * Setting ac queues params for AccessCategoryCfg (TNET configuration)
         */
        pWlanParams->acQueuesParams[acID].acId                        = acID;
        pWlanParams->acQueuesParams[acID].qId                         = acID;
        pWlanParams->acQueuesParams[acID].percentOfBlockHighThreshold = pInitParams->whalCtrl_init.TxBlocksHighPercentPerAc[acID];
        pWlanParams->acQueuesParams[acID].percentOfBlockLowThreshold  = pInitParams->whalCtrl_init.TxBlocksLowPercentPerAc[acID];
    }
    }

    /*
     * Configure the WLAN hardware after config all the hardware objects
     */
    if (whal_hwCtrl_ConfigHw (pWhalCtrl->pHwCtrl, (void *)whalCtrl_ConfigHwCb1, hWhalCtrl, FALSE) != OK)
        return NOK;

    return OK;
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_Config
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
TI_STATUS whalCtrl_Config (TI_HANDLE hWhalCtrl,TI_HANDLE hTNETW_Driver, whalCtrl_config_t* pWhalCtrlCfg,UINT32 *pFWImage)
{
    int                 Stt             = OK;

    whalSecur_config_t  securCfg;
    WHAL_CTRL           *pWhalCtrl      = (WHAL_CTRL *)hWhalCtrl;
    WlanParams_T        *pWlanParams    = whal_ParamsGetWlanParams(pWhalCtrl->pWhalParams);
      
    /* Save config parameters */
    pWhalCtrl->hReport = pWhalCtrlCfg->hReport;
    pWhalCtrl->hFwEvent = pWhalCtrlCfg->hFwEvent;
    pWhalCtrl->hRxXfer = pWhalCtrlCfg->hRxXfer;
    pWhalCtrl->hTxXfer = pWhalCtrlCfg->hTxXfer;
    pWhalCtrl->hTxHwQueue = pWhalCtrlCfg->hTxHwQueue;
    pWhalCtrl->hTxResult = pWhalCtrlCfg->hTxResult;
    pWhalCtrl->hTNETW_Driver = hTNETW_Driver;
    pWhalCtrl->hEventMbox = pWhalCtrlCfg->hEventMbox;
    pWhalCtrl->hCmdQueue = pWhalCtrlCfg->hCmdQueue;
#ifdef TI_DBG
    pWhalCtrl->hDebugTrace = pWhalCtrlCfg->hDebugTrace;
#endif /* TI_DBG */
    /* 
    Config the Params object 
    */
    whal_params_Config (pWhalCtrl->pWhalParams, pWhalCtrl->hReport);

    /* 
    Configure the security object
    */
    securCfg.hMemMgr = pWhalCtrlCfg->hMemMgr;
    securCfg.hReport = pWhalCtrl->hReport;
    if (whalSecur_Config (pWhalCtrl->pWhalSecurity, &securCfg) != OK)
    {
        WLAN_REPORT_ERROR (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG, ("Error on whalSecur_Config\n"));
    }
    
    /* 
    Initialize the Params object database fields
    */

    pWlanParams->FragmentationOnHal = 0;

#if !defined(GWSI_DRIVER) && !defined(GWSI_LIB)
    /* NO pInitTableCopy in GWSI yet */
    pWlanParams->RecoveryEnable     = 
        ((TnetwDrv_InitParams_t*)(((TnetwDrv_t *)hTNETW_Driver)->pInitTableCopy))->halCtrlConfigParams.halCtrlRecoveryEnable;
#endif /* GWSI_DRIVER */
    
    /* 
    Call local function to configure the hardware control object 
    */
    /* This will at the end call the download function */
    Stt = whalCtrl_GetSetHwAddr(pWhalCtrl, pWhalCtrlCfg->hMemMgr,pFWImage);
    /*  This could return TNETWIF__ERROR,TNETWIF_COMPLETE or TNETWIF_PENDING */
    return (TI_STATUS)Stt;
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_GetTnetwifHandle
 *
 * Input    :
 * Output   :
 * Process  :  
 * Note(s)  :  
 * -----------------------------------------------------------------------------
 */
TI_HANDLE whalCtrl_GetTnetwifHandle (TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL  *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    return whal_hwCtrl_GetTnentwifHandle (pWhalCtrl->pHwCtrl);
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_GetTnetwifHandle
 *
 * Input    :
 * Output   :
 * Process  :  
 * Note(s)  :  
 * -----------------------------------------------------------------------------
 */
TI_HANDLE whalCtrl_GetWhalParams (TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL  *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    return (TI_HANDLE)pWhalCtrl->pWhalParams;
}


/****************************************************************************
 *                      whalCtrl_FinalizeDownload()
 ****************************************************************************
 * DESCRIPTION: Finalize all the remaining initialization after the download has finished 
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
TI_STATUS whalCtrl_FinalizeDownload (TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL  *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    /* Call the upper layer to finalize all download action and then send the InitComplete Callback */
#if !defined(GWSI_DRIVER) && !defined(GWSI_LIB)
    return TnetwDrv_InitHw_FinalizeDownload(pWhalCtrl->hTNETW_Driver);
#else
    /* GWSI SA deosn't suport recovery, so the normal finalize function is called directly */
    return TnetwDrv_FinalizeDownload (pWhalCtrl->hTNETW_Driver);
#endif
}


/****************************************************************************
 *                      whalCtrl_FinalizeOnFailure()
 ****************************************************************************
 * DESCRIPTION: Finalize all the remaining initialization after the download has failed 
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
TI_STATUS whalCtrl_FinalizeOnFailure (TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL  *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    /* Call the upper layer to finalize all download */
    return TnetwDrv_FinalizeOnFailure (pWhalCtrl->hTNETW_Driver);
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_Register_CB
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  : 
 * -----------------------------------------------------------------------------
 */
void whalCtrl_Register_CB(TI_HANDLE hWhalCtrl,tiUINT32 CallBackID,void *CBFunc,TI_HANDLE CBObj)
{
    WHAL_CTRL* pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG, ("whalCtrl_Register_CB (Value = 0x%x)\n", CallBackID));

    switch(CallBackID)
    {
    case HAL_INTERNAL_EVENT_FAILURE:
        pWhalCtrl->FailureEvent_CB        = (failureEventCB_t)CBFunc;
        pWhalCtrl->FailureEvent_CB_handle = CBObj;
        /* Register all health/sanityCheck/Recovery callbacks 
                DeviceError - CORE callback for full recovery
                HealthReport - Event Mbox callback for extracting device error
                MacStatus - Event Mbox callback for checkHwStatus FCS_ERR, RX_FREE_MEM regs
                Mailbox error - Mailbox queue callback for case of timeout/error */
      #ifdef USE_RECOVERY
        whalCtrl_RegisterErrorsCallbacks(hWhalCtrl);
      #else
        CmdQueue_RegisterForErrorCB(pWhalCtrl->hCmdQueue, CBFunc, CBObj);
      #endif 
        break;
    case HAL_INT_COMMAND_COMPLETE:
        whalCtrl_RegisterCmdCompleteGenericCB(hWhalCtrl,CBFunc,CBObj);      
        break;
    default:
        /* register to the Event MBOX the corresponding Callback */
        whalCtrl_EventMbox_RegisterForEvent(hWhalCtrl, (int)(CallBackID), CBFunc, CBObj);
        whalCtrl_EventMbox_Enable(hWhalCtrl, (int)(CallBackID));
    }
    
    return;
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_PreRecoveryProcess
 *
 * Input    : TI_HANDLE hWhalCtrl
 * Output   :
 * Process  : prepare for recovery - save all mbox callbacks
 * Note(s)  : 
 * -----------------------------------------------------------------------------
 */

void whalCtrl_PreRecoveryProcess(TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL* pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    
    CmdQueue_StartReconfig( pWhalCtrl->hCmdQueue );
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetMacAddress
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SetMacAddress(TI_HANDLE hWhalCtrl, macAddress_t *macAddr)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;  

    return whal_hwCtrl_SetMacAddress(pWhalCtrl->pHwCtrl, macAddr);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetParam
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
TI_STATUS whalCtrl_SetParam (TI_HANDLE hWhalCtrl, whalParamInfo_t* pParamInfo)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    HwMboxConfig_T *pInfoElemConfig = whal_hwCtrl_GetMboxConfig(pWhalCtrl->pHwCtrl);
    WlanParams_T *pWlanParams = &pWhalCtrl->pWhalParams->WlanParams;
    TxParam_t    *pTxParams = whal_ParamsGetTxParams(pWhalCtrl->pWhalParams);


    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                             ("whalCtrl_SetParam : paramType=0x%X\n", pParamInfo->paramType));
    
    switch ((externalParam_e)pParamInfo->paramType)
    {
        case HAL_CTRL_RTS_THRESHOLD_PARAM:

            if  (pParamInfo->content.halCtrlRtsThreshold > HAL_CTRL_RTS_THRESHOLD_MAX)
            {
                WLAN_REPORT_ERROR(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,  (" whalCtrl########HAL_CTRL_RTS_THRESHOLD_PARAM: Value out of permitted range 0x%x\n",
                       pParamInfo->content.halCtrlRtsThreshold));
                return (PARAM_VALUE_NOT_VALID);
            }

            if (whal_hwInfoElemRtsThresholdSet (pInfoElemConfig,pParamInfo->content.halCtrlRtsThreshold) == OK)
            {
                WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,  (" whalCtrl########HAL_CTRL_RTS_THRESHOLD_PARAM 0x%x\n",
                                       pParamInfo->content.halCtrlRtsThreshold));
                pWlanParams->RtsThreshold = pParamInfo->content.halCtrlRtsThreshold;
            }


            break;

        case HAL_CTRL_CTS_TO_SELF_PARAM:

            if (whal_hwInfoElemCtsToSelfSet (pInfoElemConfig, pParamInfo->content.halCtrlCtsToSelf) == OK)
                pWlanParams->CtsToSelf = pParamInfo->content.halCtrlCtsToSelf;
        else
            return NOK;

            break;

        case HAL_CTRL_RX_TIME_OUT_PARAM:

            if (whal_hwInfoElemRxTimeOutSet (pInfoElemConfig, &pParamInfo->content.halCtrlRxTimeOut) == OK)
            {
                pWlanParams->rxTimeOut.psPoll = pParamInfo->content.halCtrlRxTimeOut.psPoll;
                pWlanParams->rxTimeOut.UPSD   = pParamInfo->content.halCtrlRxTimeOut.UPSD;  
            }
            break;


        case HAL_CTRL_FRAG_THRESHOLD_PARAM:
            if ((pParamInfo->content.halCtrlFragThreshold < HAL_CTRL_FRAG_THRESHOLD_MIN) ||
                (pParamInfo->content.halCtrlFragThreshold > HAL_CTRL_FRAG_THRESHOLD_MAX))
                return (PARAM_VALUE_NOT_VALID);

            pWlanParams->FragmentThreshold  = pParamInfo->content.halCtrlFragThreshold;
            pWlanParams->FragmentationOnHal = 0;
            break;
            
    case HAL_CTRL_DOT11_MAX_RX_MSDU_LIFE_TIME:
            if (whal_hwInfoElemRxMsduLifeTimeSet (pInfoElemConfig, pParamInfo->content.halCtrlMaxRxMsduLifetime) == OK)
               pWlanParams->MaxRxMsduLifetime = (UINT32)pParamInfo->content.halCtrlMaxRxMsduLifetime;
            break;
        
            case HAL_CTRL_ACX_STATISTICS_PARAM:
            if (whal_hwInfoElemAcxStatisiticsSet (pInfoElemConfig) != OK)
                return (NOK);
            break;
        
        case HAL_CTRL_LISTEN_INTERVAL_PARAM:
            if ((pParamInfo->content.halCtrlListenInterval < HAL_CTRL_LISTEN_INTERVAL_MIN) ||
                (pParamInfo->content.halCtrlListenInterval > HAL_CTRL_LISTEN_INTERVAL_MAX))
                return (PARAM_VALUE_NOT_VALID);

            pWlanParams->ListenInterval = (UINT8)pParamInfo->content.halCtrlListenInterval;
            break;
                            
        case HAL_CTRL_AID_PARAM:
            pWlanParams->Aid = pParamInfo->content.halCtrlAid;
            
            /* Configure the ACXAID info element*/
            if (whal_hwCtrl_AidSet (pWhalCtrl->pHwCtrl, pParamInfo->content.halCtrlAid) != OK)
               return (NOK);
            break;

        case HAL_CTRL_RSN_HW_ENC_DEC_ENABLE_PARAM:

            WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                                    (" whalCtrl########HW_ENC_DEC_ENABLE %d\n", pParamInfo->content.rsnHwEncDecrEnable));
            
            /* Set the Encryption/Decryption on the HW*/
            if (whalSecur_HwEncDecrEnable (pWhalCtrl->pWhalSecurity, pParamInfo->content.rsnHwEncDecrEnable) != OK)
                return (NOK);
            break;

        case HAL_CTRL_RSN_KEY_ADD_PARAM:

            WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,  (" whalCtrl########KEY_ADD\n"));
            
            if (whalSecur_KeyAdd (pWhalCtrl->pWhalSecurity,
                                 (securityKeys_t *) pParamInfo->content.configureCmdCBParams.CB_buf,
                                 FALSE,
                                 pParamInfo->content.configureCmdCBParams.CB_Func, 
                                 pParamInfo->content.configureCmdCBParams.CB_handle) != OK)
                return (NOK);
            break;

        case HAL_CTRL_RSN_KEY_REMOVE_PARAM:
    
            WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,  (" whalCtrl########KEY_REMOVE\n"));
    
            if (whalSecur_KeyRemove (pWhalCtrl->pWhalSecurity, 
                                    (securityKeys_t *) pParamInfo->content.configureCmdCBParams.CB_buf,
                                    FALSE,
                                    pParamInfo->content.configureCmdCBParams.CB_Func, 
                                    pParamInfo->content.configureCmdCBParams.CB_handle) != OK)
                return (NOK);
            break;

        case HAL_CTRL_RSN_DEFAULT_KEY_ID_PARAM:
            if ((int) *pParamInfo->content.configureCmdCBParams.CB_buf > MAX_DEFAULT_KEY_ID)
                return (PARAM_VALUE_NOT_VALID);

            WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,  (" whalCtrl########DEFAULT_KEY_ID %d\n",
                                   (UINT8)pParamInfo->content.rsnDefaultKeyID));
            
            if (whalSecur_DefaultKeyIdSet(pWhalCtrl->pWhalSecurity,
                                 *pParamInfo->content.interogateCmdCBParams.CB_buf,
                                 pParamInfo->content.interogateCmdCBParams.CB_Func, 
                                 pParamInfo->content.interogateCmdCBParams.CB_handle) != OK)
                return (NOK);

            break;

        case HAL_CTRL_RSN_SECURITY_MODE_PARAM:
            WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                                    (" whalCtrl########SECURITY_MODE_SET %d\n", pParamInfo->content.rsnEncryptionStatus));
            if (whalSecur_SecurModeSet (pWhalCtrl->pWhalSecurity, (cipherSuite_e)pParamInfo->content.rsnEncryptionStatus) != OK)
                return (NOK);
            break;

#ifdef EXC_MODULE_INCLUDED
        case HAL_CTRL_RSN_EXC_SW_ENC_ENABLE_PARAM:
            WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                                    (" whalCtrl########EXC_SW_ENC_ENABLE %d\n", pParamInfo->content.rsnExcSwEncFlag));
#ifdef CKIP_ENABLED
            if (whalSecur_SwEncEnable (pWhalCtrl->pWhalSecurity, pParamInfo->content.rsnExcSwEncFlag) != OK)
                return (NOK);
#endif
            /* when SW encryption is ON, HW encryption should be turned OFF and vice versa */
            
            WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                                    (" whalCtrl: Set HwEncDecrEnable to %d\n", !pParamInfo->content.rsnExcSwEncFlag));
                
            /* Set the Encryption/Decryption on the HW*/
            if (whalSecur_HwEncDecrEnable (pWhalCtrl->pWhalSecurity, (BOOL)(!(pParamInfo->content.rsnExcSwEncFlag))) != OK)
                return (NOK);
            break;

        case HAL_CTRL_RSN_EXC_MIC_FIELD_ENABLE_PARAM:
            WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                                    (" whalCtrl########EXC_MIC_FIELD_ENABLE %d\n", pParamInfo->content.rsnExcMicFieldFlag));
#ifdef CKIP_ENABLED
            if (whalSecur_MicFieldEnable (pWhalCtrl->pWhalSecurity, pParamInfo->content.rsnExcMicFieldFlag) != OK)
                return (NOK);
#endif
            break;
#endif /* EXC_MODULE_INCLUDED*/

        case HAL_CTRL_TX_POWER_PARAM:
            
            WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                                    (" whalCtrl########TX_POWER_LEVEL old = %d new = %d\n", 
									pTxParams->TxPowerDbm, pParamInfo->content.halCtrlTxPowerDbm));

			if ( pTxParams->TxPowerDbm == pParamInfo->content.halCtrlTxPowerDbm)
			{	/* Save up time if we set the same value */
				return TX_POWER_SET_SAME_VALUE;
			}

            pTxParams->TxPowerDbm = pParamInfo->content.halCtrlTxPowerDbm;
            
            /* configure the wlan hardware */
            if (whal_hwInfoElemTxPowerSet (pInfoElemConfig,
                                                &pTxParams->TxPowerDbm) != OK)
                return (NOK);
            break;


        case HAL_CTRL_SG_ENABLE_PARAM:
            return (TI_STATUS)whal_hwCtrl_SoftGeminiEnable(pWhalCtrl->pHwCtrl,(SoftGeminiEnableModes_e)pParamInfo->content.SoftGeminiEnable);

        case HAL_CTRL_SG_CONFIG_PARAM:
            return (TI_STATUS)whal_hwCtrl_SetSoftGeminiParams(pWhalCtrl->pHwCtrl,&(pParamInfo->content.SoftGeminiParam));

        case HAL_CTRL_ANTENNA_DIVERSITY_PARAMS:
            /* save parameters */
            whal_hwCtrl_SaveAntennaDiversityOptions (pWhalCtrl->pHwCtrl, &(pParamInfo->content.antennaDiversityOptions));
            /* Configure FW with new parameters */
            whal_hwCtrl_CurrentAntennaDiversitySendCmd (pWhalCtrl->pHwCtrl);
            break;

        case HAL_CTRL_TX_ANTENNA_PARAM:
            /* save antenna number */
            whal_hwCtrl_SetTxAntenna( pWhalCtrl->pHwCtrl, pParamInfo->content.antennaNum );
            /* Write parameters to FW */
            whal_hwCtrl_CurrentAntennaDiversitySendCmd( pWhalCtrl->pHwCtrl );
            break;

        case HAL_CTRL_RX_ANTENNA_PARAM:
            /* save antenna number */
            whal_hwCtrl_SetRxAntenna( pWhalCtrl->pHwCtrl, pParamInfo->content.antennaNum );
            /* Write parameters to FW */
            whal_hwCtrl_CurrentAntennaDiversitySendCmd( pWhalCtrl->pHwCtrl );
            break;

    /*
     *  TX Parameters 
     */ 

    case HAL_CTRL_TX_RATE_CLASS_PARAMS:
        return (TI_STATUS)whal_hwCtrl_TxRatePolicy(pWhalCtrl->pHwCtrl,pParamInfo->content.pTxRatePlicy);
/*      break; */

    case HAL_CTRL_QUEUES_PARAMS:
        os_memoryCopy (pWhalCtrl->hOs, &pWhalCtrl->pWhalParams->WlanParams.QtrafficParams,
                       pParamInfo->content.pQueueTrafficParams, sizeof(queueTrafficParams_t));

        return (TI_STATUS)whal_hwCtrl_TrafficConf(pWhalCtrl->pHwCtrl,pParamInfo->content.pQueueTrafficParams);
/*      break; */

    case HAL_CTRL_AC_PARAMS:
        os_memoryCopy (pWhalCtrl->hOs, &pWhalCtrl->pWhalParams->WlanParams.acQosParams,
                       pParamInfo->content.configureCmdCBParams.CB_buf, sizeof(acQosParams_t));
        return (TI_STATUS)whal_hwCtrl_AcParamsConf(pWhalCtrl->pHwCtrl,&pParamInfo->content.configureCmdCBParams);
/*      break; */

        
    
#if 0

    /*Tx Ack/No Ack*/
    case HAL_CTRL_TX_ACK_POLICY:
            return (whal_ParamsSetAccessCategoryAckPolicy(pWhalCtrl->pWhalParams, 
                                                          pParamInfo->content.AcAckPolicy.AckPolicy,
                                                          pParamInfo->content.AcAckPolicy.AcId));
            break;

#endif

    case HAL_CTRL_MIN_POWER_LEVEL:
        whalCtrl_SetMinPowerLevel(pWhalCtrl,
                                    pParamInfo->content.minPowerPolicy);
            break;

    
    case HAL_CTRL_CLK_RUN_ENABLE:
        WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                                (" whalCtrl_SetParam: CLK_RUN_ENABLE %d\n", pParamInfo->content.halCtrlClkRunEnable));

        /* Set the Encryption/Decryption on the HW*/
        if (whal_hwCtrl_ClkRunEnableSet (pWhalCtrl->pHwCtrl, pParamInfo->content.halCtrlClkRunEnable) != OK)
            return (NOK);
        break;

    case HAL_CTRL_EARLY_WAKEUP:
        WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                                (" whalCtrl_SetParam: SET EARLY WAKEUP to %d\n", pParamInfo->content.earlyWakeup));
        pWlanParams->earlyWakeUp = pParamInfo->content.earlyWakeup;
        break;

        /* PLT params */    
    case HAL_CTRL_PLT_WRITE_MIB:
        WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,  ("%s - HAL_CTRL_PLT_WRITE_MIB(0x%x) \n", __FUNCTION__, pParamInfo->paramType));
        return (TI_STATUS)whalCtrl_WriteMib(hWhalCtrl, &pParamInfo->content.PltMib);
        
/*        break; */
        
    case HAL_CTRL_PLT_RX_PER_START:
        WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,  ("%s - HAL_CTRL_PLT_RX_PER_START(0x%x) \n", __FUNCTION__, pParamInfo->paramType));
        return (TI_STATUS)whalCtrl_RxPER(pWhalCtrl, PLT_RX_PER_START, 
            pParamInfo->content.interogateCmdCBParams.CB_handle, 
            pParamInfo->content.interogateCmdCBParams.CB_Func);            
/*        break; */
        
    case HAL_CTRL_PLT_RX_PER_STOP:
        WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,  ("%s - HAL_CTRL_PLT_RX_PER_STOP(0x%x) \n", __FUNCTION__, pParamInfo->paramType));
        return (TI_STATUS)whalCtrl_RxPER(pWhalCtrl, PLT_RX_PER_STOP, 
            pParamInfo->content.interogateCmdCBParams.CB_handle, 
            pParamInfo->content.interogateCmdCBParams.CB_Func);            
/*        break; */
        
    case HAL_CTRL_PLT_RX_PER_CLEAR:
        WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,  ("%s - HAL_CTRL_PLT_RX_PER_CLEAR(0x%x) \n", __FUNCTION__, pParamInfo->paramType));
        return (TI_STATUS)whalCtrl_RxPER(pWhalCtrl, PLT_RX_PER_CLEAR, 
            pParamInfo->content.interogateCmdCBParams.CB_handle, 
            pParamInfo->content.interogateCmdCBParams.CB_Func);            
/*        break; */
    case HAL_CTRL_PLT_TX_CW:
        WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,  ("%s - HAL_CTRL_PLT_TX_CW(0x%x) \n", __FUNCTION__, pParamInfo->paramType));
        return (TI_STATUS)whalCtrl_TxCW(pWhalCtrl, &pParamInfo->content.PltCW,
                                NULL, NULL, NULL);
        /*break;*/
        
    case HAL_CTRL_PLT_TX_CONTINUES:
        WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,  ("%s - HAL_CTRL_PLT_TX_CONTINUES(0x%x) \n", __FUNCTION__, pParamInfo->paramType));
        return (TI_STATUS)whalCtrl_TxContinues(pWhalCtrl, &pParamInfo->content.PltTxContinues,
                                    NULL, NULL, NULL);
        /*break;*/

    case HAL_CTRL_PLT_TX_STOP:
        WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,  ("%s - HAL_CTRL_PLT_TX_STOP(0x%x) \n", __FUNCTION__, pParamInfo->paramType));
        return (TI_STATUS)whal_hwCmdBit_perTxStop(pWhalCtrl->pHwCtrl->pHwMboxCmdBit, 
                                       NULL, NULL, NULL);
        /*break;*/
        
    case HAL_CTRL_PLT_WRITE_REGISTER:
        WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,  ("%s - HAL_CTRL_PLT_WRITE_REGISTER(0x%x) \n", __FUNCTION__, pParamInfo->paramType));
        return (TI_STATUS)whalCtrl_WriteRegister(pWhalCtrl, pParamInfo->content.interogateCmdCBParams.CB_handle, pParamInfo->content.interogateCmdCBParams.CB_Func, pParamInfo->content.interogateCmdCBParams.CB_buf);
/*        break;  */
        
    default:
        WLAN_REPORT_ERROR(pWhalCtrl->hReport,
                      HAL_CTRL_MODULE_LOG,
                      ("%s(%d) - whalCtrl_SetParam - ERROR - Param is not supported, %d\n\n",
                       __FILE__,__LINE__,pParamInfo->paramType));
        return (PARAM_NOT_SUPPORTED);
    }
    
    return (OK);
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_GetParam
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
TI_STATUS whalCtrl_GetParam (TI_HANDLE hWhalCtrl, whalParamInfo_t* pParamInfo)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    WlanParams_T *pWlanParams = &pWhalCtrl->pWhalParams->WlanParams;
    HwMboxConfig_T *pInfoElemConfig = whal_hwCtrl_GetMboxConfig(pWhalCtrl->pHwCtrl);
    TxParam_t      *pTxParams = whal_ParamsGetTxParams(pWhalCtrl->pWhalParams);

    switch ((externalParam_e)pParamInfo->paramType)
    {
        case HAL_CTRL_RTS_THRESHOLD_PARAM:
            pParamInfo->content.halCtrlRtsThreshold = pWlanParams->RtsThreshold;
            break;
        
        case HAL_CTRL_FRAG_THRESHOLD_PARAM:
            pParamInfo->content.halCtrlFragThreshold = pWlanParams->FragmentThreshold;
            break;

        case HAL_CTRL_COUNTERS_PARAM:
            /* Constant zero because the ACX last buffer next pointer is always pointed
               to itself, so it's like an endless buffer*/
            pParamInfo->content.halCtrlCounters.RecvNoBuffer = 0;
            pParamInfo->content.halCtrlCounters.FragmentsRecv = 0; /* not supported;*/
            pParamInfo->content.halCtrlCounters.FrameDuplicates = 0;/* not supported*/
            pParamInfo->content.halCtrlCounters.FcsErrors = pWhalCtrl->pWhalParams->GenCounters.FcsErrCnt;
            pParamInfo->content.halCtrlCounters.RecvError = pWhalCtrl->pWhalParams->GenCounters.FcsErrCnt;
            break;
        
        case HAL_CTRL_LISTEN_INTERVAL_PARAM:
            pParamInfo->content.halCtrlListenInterval = pWlanParams->ListenInterval;
            break;
                            
        case HAL_CTRL_RSN_DEFAULT_KEY_ID_PARAM:
            /* Not implemented */
            return NOK;
/*            break; */

        case HAL_CTRL_TX_POWER_PARAM:
            pParamInfo->content.halCtrlTxPowerDbm = pTxParams->TxPowerDbm;
            break;
        
        case HAL_CTRL_ACX_STATISTICS_PARAM:
            /* Not implemented */
         #if 0
            {
                acxStatisitcs_t     acxStatisitics;
                pParamInfo->content.acxStatisitics.FWpacketReceived = acxStatisitics.FWpacketReceived;
                /* Not supported */
                pParamInfo->content.acxStatisitics.HALpacketReceived = 0; 
            }
         #endif
            return NOK;

        case HAL_CTRL_MEDIUM_OCCUPANCY_PARAM:
            if (whal_hwInfoElemMediumOccupancyGet (pInfoElemConfig,
                                                   pParamInfo->content.interogateCmdCBParams) != OK)
                return (NOK);

            break;

        case HAL_CTRL_TSF_DTIM_MIB:
            if (whal_hwInfoElemTfsDtimGet(pInfoElemConfig,
                                          pParamInfo->content.interogateCmdCBParams) != OK)
                return (NOK);
            
            break;


    case HAL_CTRL_AID_PARAM:
        if (whal_hwCtrl_CurrentAssociationIdGet(pWhalCtrl->pHwCtrl,
                                     &(pParamInfo->content.halCtrlAid)) != OK)
            return (NOK);

        WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                                (" AID 2 %d\n", pParamInfo->content.halCtrlAid));


        break;

    case HAL_CTRL_NOISE_HISTOGRAM_PARAM:
        if (whal_hwInfoElemNoiseHistogramResultsGet (pInfoElemConfig,
                                                     pParamInfo->content.interogateCmdCBParams) != OK)
        {
            return (NOK);
        }

        break;

    case HAL_CTRL_TX_ANTENNA_PARAM:
        /* get antenna number */
        whal_hwCtrl_GetTxAntenna( pWhalCtrl->pHwCtrl, &(pParamInfo->content.antennaNum) );
        break;

    case HAL_CTRL_RX_ANTENNA_PARAM:
        /* get antenna number */
        whal_hwCtrl_GetRxAntenna( pWhalCtrl->pHwCtrl, &(pParamInfo->content.antennaNum) );
        break;

    case HAL_CTRL_CURRENT_CHANNEL:
        /* get current channel number */
        pParamInfo->content.halCtrlCurrentChannel = whal_ParamsGetRadioChannel( pWhalCtrl->pWhalParams );

    case HAL_CTRL_MIN_POWER_LEVEL:
        whalCtrl_GetMinPowerLevel(pWhalCtrl, &pParamInfo->content.minPowerPolicy);
        break;

        /*SNR and RSSI belongs to the same MIB, and the relevant CB is passed here*/
    case HAL_CTRL_RSSI_LEVEL_PARAM:
    case HAL_CTRL_SNR_RATIO_PARAM:
        /* Retrive the Callback function and read buffer pointer that are in fact stored in the TIWLAN_ADAPTER and then send it to the Command Mailbox */
        whalCtrl_GetAsynRSSI (pWhalCtrl,pParamInfo->content.interogateCmdCBParams.CB_Func, pParamInfo->content.interogateCmdCBParams.CB_handle , pParamInfo->content.interogateCmdCBParams.CB_buf);
        break;

    case HAL_CTRL_BCN_BRC_OPTIONS:
        pParamInfo->content.BcnBrcOptions.BeaconRxTimeout    = pWlanParams->BcnBrcOptions.BeaconRxTimeout;
        pParamInfo->content.BcnBrcOptions.BroadcastRxTimeout = pWlanParams->BcnBrcOptions.BroadcastRxTimeout;
        pParamInfo->content.BcnBrcOptions.RxBroadcastInPs    = pWlanParams->BcnBrcOptions.RxBroadcastInPs;
        break;

    case HAL_CTRL_DOT11_MAX_RX_MSDU_LIFE_TIME:
        pParamInfo->content.halCtrlMaxRxMsduLifetime = pWlanParams->MaxRxMsduLifetime;
        break;
        
        
        /* PLT params */    
    case HAL_CTRL_PLT_RX_PER_GET_RESULTS:
        return (TI_STATUS)whalCtrl_RxPER(pWhalCtrl, PLT_RX_PER_GETRESULTS, 
            pParamInfo->content.interogateCmdCBParams.CB_handle, 
            pParamInfo->content.interogateCmdCBParams.CB_Func);            
/*        break; */
        
    case HAL_CTRL_PLT_READ_MIB:
        return (TI_STATUS)whalCtrl_ReadMib(pWhalCtrl, 
            pParamInfo->content.interogateCmdCBParams.CB_handle,
            pParamInfo->content.interogateCmdCBParams.CB_Func, 
            pParamInfo->content.interogateCmdCBParams.CB_buf);
        
/*        break; */
        
    case HAL_CTRL_PLT_READ_REGISTER:        
        whalCtrl_ReadRegister(pWhalCtrl, pParamInfo->content.interogateCmdCBParams.CB_handle,
            pParamInfo->content.interogateCmdCBParams.CB_Func, 
            pParamInfo->content.interogateCmdCBParams.CB_buf);
        break;

    case HAL_CTRL_PLT_RX_TX_CAL:
         WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,  ("%s - HAL_CTRL_PLT_RX_TX_CAL(0x%x) \n", __FUNCTION__, pParamInfo->paramType));         
         return (TI_STATUS)whal_hwCmdBit_TestCmd(pWhalCtrl->pHwCtrl->pHwMboxCmdBit,                                       
                                      pParamInfo->content.interogateCmdCBParams.CB_Func, 
                                      pParamInfo->content.interogateCmdCBParams.CB_handle,
                                      (TestCmd_t*)pParamInfo->content.interogateCmdCBParams.CB_buf);
         /*break*/

    case HAL_CTRL_PLT_RX_CAL_STATUS:
         whal_hwCmdBit_GetPltRxCalibrationStatus( pWhalCtrl->pHwCtrl->pHwMboxCmdBit, 
                                                  &(pParamInfo->content.PltRxCalibrationStatus) );
         return OK;
         /* break */

    case HAL_CTRL_CTS_TO_SELF_PARAM:
        pParamInfo->content.halCtrlCtsToSelf = pWlanParams->CtsToSelf;        
        break;

    case HAL_CTRL_TX_RATE_CLASS_PARAMS:
        pParamInfo->content.pTxRatePlicy = whal_hwCtrl_GetTxRatePolicy(pWhalCtrl->pHwCtrl);
        break;

    case HAL_CTRL_SG_CONFIG_PARAM:
        return (TI_STATUS)whal_hwCtrl_GetSoftGeminiParams(pWhalCtrl->pHwCtrl,
                                      pParamInfo->content.interogateCmdCBParams.CB_Func, 
                                      pParamInfo->content.interogateCmdCBParams.CB_handle,
                                      (void*)pParamInfo->content.interogateCmdCBParams.CB_buf);

    case HAL_CTRL_REVISION:
        return (TI_STATUS)whal_hwInfoElemAcxRevisionGet (pInfoElemConfig,
                                                         pParamInfo->content.interogateCmdCBParams.CB_Func, 
                                                         pParamInfo->content.interogateCmdCBParams.CB_handle,
                                                         (void*)pParamInfo->content.interogateCmdCBParams.CB_buf);

	case HAL_CTRL_RSN_SECURITY_MODE_PARAM:
		pParamInfo->content.rsnEncryptionStatus = (halCtrl_CipherSuite_e)whalSecur_SecurModeGet (pWhalCtrl->pWhalSecurity);

		WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
								(" whalCtrl########SECURITY_MODE_GET %d\n", pParamInfo->content.rsnEncryptionStatus));
        break; 

    case HAL_CTRL_EARLY_WAKEUP:
        WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                                (" whalCtrl_SetParam: GET EARLY WAKEUP is %d\n", pWlanParams->earlyWakeUp));
        pParamInfo->content.earlyWakeup = pWlanParams->earlyWakeUp;

        break; 

	case HAL_CTRL_POWER_LEVEL_TABLE_PARAM:
		if (whal_hwInfoElemPowerLevelTableGet (pInfoElemConfig,
			pParamInfo->content.interogateCmdCBParams) != OK)
		{
			return (NOK);
		}

		break;
    case HAL_CTRL_POWER_CONSUMPTION:
        whalCtrl_getConsumptionStatistics(pWhalCtrl->pHwCtrl->pHwMboxConfig, 
                    pParamInfo->content.interogateCmdCBParams.CB_Func,
                    pParamInfo->content.interogateCmdCBParams.CB_handle, 
                    (void*)pParamInfo->content.interogateCmdCBParams.CB_buf);
        break;

    default:
        WLAN_REPORT_ERROR(pWhalCtrl->hReport,
            HAL_CTRL_MODULE_LOG,
            ("%s(%d) - whalCtrl_GetParam - ERROR - Param is not supported, %d\n\n",
            __FILE__,__LINE__,pParamInfo->paramType));
        return (PARAM_NOT_SUPPORTED);
/*        break; */
    }

    return (OK);
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_setRxFilters
 *
 * Input    :
 * Output   :
 * Process  :  Configures the Rx filters
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_setRxFilters(TI_HANDLE hWhalCtrl, UINT32 RxConfigOption, UINT32 RxFilterOption)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    return whal_hwCtrl_setRxFilters(pWhalCtrl->pHwCtrl, RxConfigOption, RxFilterOption);
}

/*
* ----------------------------------------------------------------------------
* Function : whalCtrl_GetRxFilters
*
* Input    :
* Output   :
* Process  :  Configures the Rx filters
* Note(s)  :  Done
* -----------------------------------------------------------------------------
*/
int  whalCtrl_GetRxFilters      (TI_HANDLE hWhalCtrl, UINT32* pRxConfigOption, UINT32* pRxFilterOption)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    
    return whal_hwCtrl_GetRxFilters(pWhalCtrl->pHwCtrl, pRxConfigOption, pRxFilterOption);
}

/*
 * ----------------------------------------------------------------------------
* Function : whalCtrl_getRxDataFiltersStatistics
*
* Input    : Retrieve Statistics
* Output   :
* Process  :
* Note(s)  : Done
* -----------------------------------------------------------------------------
*/
int whalCtrl_getRxDataFiltersStatistics(TI_HANDLE hWhalCtrl, void * CB_Func, TI_HANDLE CB_handle, void * CB_Buf)
{
    WHAL_CTRL * pWhalCtrl = (WHAL_CTRL *) hWhalCtrl;

    whal_hwInfoElemGetRxDataFiltersStatistics(pWhalCtrl->pHwCtrl->pHwMboxConfig, CB_Func, CB_handle, CB_Buf);

    return OK;
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_setRxDataFiltersParams
 *
 * Input    :   enabled             - 0 to disable data filtering, any other value to enable.
 *              defaultAction       - The default action to take on non-matching packets.
 * Output   :
 * Process  :  Enable or disable the Rx Data Filtering feature.
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_setRxDataFiltersParams(TI_HANDLE hWhalCtrl, BOOL enabled, filter_e defaultAction)
{
    WHAL_CTRL * pWhalCtrl = (WHAL_CTRL *) hWhalCtrl;

    pWhalCtrl->pWhalParams->WlanParams.rxFilterDefaultEnable = enabled;
    pWhalCtrl->pWhalParams->WlanParams.rxFilterDefaultAction = defaultAction;

    return whal_hwCtrl_setRxDataFiltersParams(pWhalCtrl->pHwCtrl, enabled, defaultAction);
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_setRxDataFilter
 *
 * Input    :   index               - Index of the Rx Data filter
 *              command             - Disable or enable the filter
 *              action              - Action to take on packets matching the pattern
 *              numFieldPatterns    - Number of field patterns in the filter
 *              lenFieldPatterns    - Length of the field pattern series
 *              fieldPatterns       - Series of field patterns
 * Output   :
 * Process  :  Adds or removes the Rx Data Filter at the specified entry.
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_setRxDataFilter(TI_HANDLE hWhalCtrl, UINT8 index, UINT8 command, filter_e action, UINT8 numFieldPatterns, UINT8 lenFieldPatterns, UINT8 * fieldPatterns)
{
    WHAL_CTRL * pWhalCtrl = (WHAL_CTRL *) hWhalCtrl;

    pWhalCtrl->pWhalParams->WlanParams.rxFilterCgf[index].rxFilterCommand = command;
    pWhalCtrl->pWhalParams->WlanParams.rxFilterCgf[index].rxFilterAction = action;
    pWhalCtrl->pWhalParams->WlanParams.rxFilterCgf[index].rxFilterNumFieldPatterns = numFieldPatterns;
    pWhalCtrl->pWhalParams->WlanParams.rxFilterCgf[index].rxFilterLenFieldPatterns = lenFieldPatterns;

    os_memoryCopy(pWhalCtrl->hOs, (pWhalCtrl->pWhalParams->WlanParams.rxFilterCgf[index].rxFilterFieldPatterns), 
                  fieldPatterns, lenFieldPatterns);

    return whal_hwCtrl_setRxDataFilter(pWhalCtrl->pHwCtrl, index, command, action, numFieldPatterns, lenFieldPatterns, fieldPatterns);
}




/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetarpIpAddressesTable
 *
 * Input    :
 * Output   :
 * Process  :  Configures the ARP table
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SetarpIpAddressesTable(TI_HANDLE hWhalCtrl, IpAddress_t * IP_addr, UINT8 isEnabled , IPver_e IP_ver)
{
    WHAL_CTRL * pWhalCtrl = (WHAL_CTRL *) hWhalCtrl;

    return whal_hwCtrl_SetarpIpAddressesTable(pWhalCtrl->pHwCtrl, IP_addr, isEnabled , IP_ver);
}


/*
 * ----------------------------------------------------------------------------
* Function : whalCtrl_GetGroupIpAddressesTable
*
* Input    :
* Output   :
* Process  :  Retrieve the ARP IP address table
* -----------------------------------------------------------------------------
*/
int whalCtrl_GetGroupIpAddressesTable(TI_HANDLE hWhalCtrl, UINT8* pisEnabled, UINT8* pnumGroupAddrs, macAddress_t *Group_addr)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    return whal_hwCtrl_GetGroupAddressesTable(pWhalCtrl->pHwCtrl, 
        pisEnabled, pnumGroupAddrs, Group_addr);
}


/*
* ----------------------------------------------------------------------------
 * Function : whalCtrl_SetarpIpFilterEnabled
 *
 * Input    :
 * Output   :
 * Process  :  Configures the ARP table
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SetarpIpFilterEnabled(TI_HANDLE hWhalCtrl,UINT8 isEnabled )
{
    WHAL_CTRL * pWhalCtrl = (WHAL_CTRL *) hWhalCtrl;
    return whal_hwCtrl_SetarpIpFilterEnabled(pWhalCtrl->pHwCtrl, isEnabled ) ;
    
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_StartScan
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_StartScan (TI_HANDLE hWhalCtrl, whalCtrl_scan_t* pScanVals, BOOLEAN bHighPriority ,void* ScanCommandResponseCB, TI_HANDLE CB_handle)
{

    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    BasicScanChannelParameters_t* chanPtr;
    ScanParameters_t    tnetScanParams;

    UINT8*              pBSSID;
    int i;

    /* Convert general scan data to tnet structure */
    tnetScanParams.basicScanParameters.tidTrigger = pScanVals->Tid;
    tnetScanParams.basicScanParameters.numOfProbRqst = pScanVals->probeReqNumber;
    tnetScanParams.basicScanParameters.ssidLength = pScanVals->desiredSsid.len;
    os_memoryCopy( pWhalCtrl->hOs, (void *)tnetScanParams.basicScanParameters.ssidStr, 
                   (void *)pScanVals->desiredSsid.ssidString, tnetScanParams.basicScanParameters.ssidLength );
    
    /* 
        scan options field is composed of scan type and band selection. 
        First, use the lookup table to convert the scan type 
    */                  

    tnetScanParams.basicScanParameters.scanOptions = 0;

    switch ( pScanVals->scanType )
    {
    case SCAN_TYPE_NORMAL_ACTIVE : 
        tnetScanParams.basicScanParameters.scanOptions = SCAN_ACTIVE;
        break;
    
    case SCAN_TYPE_NORMAL_PASSIVE : 
        tnetScanParams.basicScanParameters.scanOptions = SCAN_PASSIVE;
        break;
    
    case SCAN_TYPE_TRIGGERED_ACTIVE :
        tnetScanParams.basicScanParameters.scanOptions = SCAN_ACTIVE | TRIGGERED_SCAN;
        break;
    
    case SCAN_TYPE_TRIGGERED_PASSIVE :
        tnetScanParams.basicScanParameters.scanOptions = SCAN_PASSIVE | TRIGGERED_SCAN;
        break;

    default:
        WLAN_REPORT_ERROR( pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                           ("Invalid scan type:%d\n", pScanVals->scanType) );
        return NOK;
    }

    /* Add the band selection */
    if ( RADIO_BAND_5_0_GHZ == pScanVals->band )
    {
        tnetScanParams.basicScanParameters.scanOptions |= SCAN_5GHZ_BAND;
    }

    /* Add high priority bit */
    if ( bHighPriority )
    {
        tnetScanParams.basicScanParameters.scanOptions |= SCAN_PRIORITY_HIGH;
    }

    tnetScanParams.basicScanParameters.scanOptions = ENDIAN_HANDLE_WORD( tnetScanParams.basicScanParameters.scanOptions );

    /* important note: BSSID filter (0x0010) is DISABLED, because the FW sets it according
       to BSSID value (broadcast does not filter, any other value will */
    tnetScanParams.basicScanParameters.rxCfg.ConfigOptions = 0x12802 ;
    tnetScanParams.basicScanParameters.rxCfg.FilterOptions = ENDIAN_HANDLE_LONG( RX_FILTER_CFG_ );

    /* If the SSID is not broadcast SSID, also filter according to SSID */
    if ( FALSE == utils_isAnySSID( &(pScanVals->desiredSsid) ) )
    {
        tnetScanParams.basicScanParameters.rxCfg.ConfigOptions |= 0x0400;
    }
    tnetScanParams.basicScanParameters.rxCfg.ConfigOptions = ENDIAN_HANDLE_LONG( tnetScanParams.basicScanParameters.rxCfg.ConfigOptions );

    /* Rate conversion is done in the HAL */
    whalUtils_ConvertAppRatesBitmap(pScanVals->probeRequestRate, 0, &(tnetScanParams.basicScanParameters.txdRateSet));
    tnetScanParams.basicScanParameters.txdRateSet = ENDIAN_HANDLE_WORD( tnetScanParams.basicScanParameters.txdRateSet );
    
    tnetScanParams.basicScanParameters.numChannels = ENDIAN_HANDLE_WORD( pScanVals->numOfChannels );

    /* copy channel specific scan data to HAL structure */
    for ( i = 0; i < pScanVals->numOfChannels; i++ )
    {
        int j;
        UINT8*  macAddr;

        macAddr = (UINT8*)&tnetScanParams.basicScanChannelParameters[ i ].bssIdL;

        /* copy the MAC address, upside down (CHIP structure) */
        for ( j = 0; j < MAC_ADDR_LEN; j++ )
        {
            macAddr[ j ] = pScanVals->channelEntry[ i ].normalChannelEntry.bssId.addr[ MAC_ADDR_LEN - 1 - j ];
        }
        tnetScanParams.basicScanChannelParameters[ i ].scanMinDuration = 
            ENDIAN_HANDLE_LONG( pScanVals->channelEntry[ i ].normalChannelEntry.minChannelDwellTime );
        tnetScanParams.basicScanChannelParameters[ i ].scanMaxDuration = 
            ENDIAN_HANDLE_LONG( pScanVals->channelEntry[ i ].normalChannelEntry.maxChannelDwellTime );
        tnetScanParams.basicScanChannelParameters[ i ].ETCondCount = 
            pScanVals->channelEntry[ i ].normalChannelEntry.ETMaxNumOfAPframes |
            pScanVals->channelEntry[ i ].normalChannelEntry.earlyTerminationEvent;
        tnetScanParams.basicScanChannelParameters[ i ].txPowerAttenuation = 
            pScanVals->channelEntry[ i ].normalChannelEntry.txPowerDbm;
        tnetScanParams.basicScanChannelParameters[ i ].channel = 
            pScanVals->channelEntry[ i ].normalChannelEntry.channel;
    }

    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                             ("RxCfg = 0x%x\n \
                              RxFilterCfg = 0x%x\n \
                              scanOptions = 0x%x\n \
                              numChannels = %d\n \
                              probeNumber = %d\n \
                              probeRateModulation = 0x%x\n \
                              tidTrigger = %d\n" , 
                              tnetScanParams.basicScanParameters.rxCfg.ConfigOptions, 
                              tnetScanParams.basicScanParameters.rxCfg.FilterOptions,
                              tnetScanParams.basicScanParameters.scanOptions, 
                              tnetScanParams.basicScanParameters.numChannels, 
                              tnetScanParams.basicScanParameters.numOfProbRqst,
                              tnetScanParams.basicScanParameters.txdRateSet, 
                              tnetScanParams.basicScanParameters.tidTrigger));
    
    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
        ("Channel      BSSID           MinTime     MaxTime     ET     TxPower   probChan\n"));
   
    for( i=0; i < pScanVals->numOfChannels; i++)
    {
        chanPtr = &tnetScanParams.basicScanChannelParameters[i];
        pBSSID = (UINT8*)&chanPtr->bssIdL;

        WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
            ("%6d   %02x:%02x:%02x:%02x:%02x:%02x    %5d %5d %2d %5d %5d\n",i,  
            pBSSID[5],pBSSID[4],pBSSID[3],pBSSID[2],pBSSID[1],pBSSID[0],
            chanPtr->scanMinDuration, chanPtr->scanMaxDuration, chanPtr->ETCondCount,
            chanPtr->txPowerAttenuation,    chanPtr->channel));
    }
    /* Send the scan command*/
    return (whal_hwCtrl_StartScan (pWhalCtrl->pHwCtrl, &tnetScanParams , ScanCommandResponseCB , CB_handle));
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_StartSPSScan
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_StartSPSScan( TI_HANDLE hWhalCtrl, whalCtrl_scan_t* pScanVals, void* ScanCommandResponseCB, TI_HANDLE CB_handle)
{

    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    ScheduledScanParameters_t   tnetSPSScanParams;
    int i;

    /* Convert general scan data to TNET structure */
    tnetSPSScanParams.scheduledGeneralParameters.scanOptions = SCAN_PASSIVE;
    /* Add the band selection */
    if ( RADIO_BAND_5_0_GHZ == pScanVals->band )
    {
        tnetSPSScanParams.scheduledGeneralParameters.scanOptions |= SCAN_5GHZ_BAND;
    }
    tnetSPSScanParams.scheduledGeneralParameters.scanOptions = ENDIAN_HANDLE_WORD( tnetSPSScanParams.scheduledGeneralParameters.scanOptions );

    /* important note: BSSID filter (0x0010) is DISABLED, because the FW sets it according
       to BSSID value (broadcast does not filter, any other value will */
    /* If the SSID is not broadcast SSID, also filter according to SSID */
    tnetSPSScanParams.scheduledGeneralParameters.rxCfg.ConfigOptions = 0x12802;
    tnetSPSScanParams.scheduledGeneralParameters.rxCfg.FilterOptions = ENDIAN_HANDLE_LONG( RX_FILTER_CFG_ );
    tnetSPSScanParams.scheduledGeneralParameters.rxCfg.ConfigOptions = ENDIAN_HANDLE_LONG( tnetSPSScanParams.scheduledGeneralParameters.rxCfg.ConfigOptions );

    /* latest TSF value - used to discover TSF error (AP recovery) */
    tnetSPSScanParams.scheduledGeneralParameters.scanCmdTime_h = ENDIAN_HANDLE_LONG( INT64_HIGHER(pScanVals->latestTSFValue) );
    tnetSPSScanParams.scheduledGeneralParameters.scanCmdTime_l = ENDIAN_HANDLE_LONG( INT64_LOWER(pScanVals->latestTSFValue) );

    tnetSPSScanParams.scheduledGeneralParameters.numChannels = pScanVals->numOfChannels;

    /* copy channel specific scan data to HAL structure */
    for ( i = 0; i < pScanVals->numOfChannels; i++ )
    {
        int j;
        UINT8*  macAddr;

        macAddr = (UINT8*)&tnetSPSScanParams.scheduledChannelParameters[ i ].bssIdL;

        /* copy the MAC address, upside down (CHIP structure) */
        for ( j = 0; j < MAC_ADDR_LEN; j++ )
        {
            macAddr[ j ] = pScanVals->channelEntry[ i ].normalChannelEntry.bssId.addr[ MAC_ADDR_LEN - 1 - j ];
        }
        tnetSPSScanParams.scheduledChannelParameters[ i ].scanMaxDuration = 
            ENDIAN_HANDLE_LONG( pScanVals->channelEntry[ i ].SPSChannelEntry.scanDuration );
        tnetSPSScanParams.scheduledChannelParameters[ i ].scanStartTime = 
            ENDIAN_HANDLE_LONG( pScanVals->channelEntry[ i ].SPSChannelEntry.scanStartTime );
        tnetSPSScanParams.scheduledChannelParameters[ i ].ETCondCount =
            pScanVals->channelEntry[ i ].SPSChannelEntry.ETMaxNumOfAPframes | 
            pScanVals->channelEntry[ i ].SPSChannelEntry.earlyTerminationEvent;
        tnetSPSScanParams.scheduledChannelParameters[ i ].channel = 
            pScanVals->channelEntry[ i ].SPSChannelEntry.channel;
    }
#ifdef TI_DBG
    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                             ("RxCfg = 0x%x\n \
                              RxFilterCfg = 0x%x\n \
                              scanOptions = 0x%x\n \
                              numChannels = %d\n",
                              tnetSPSScanParams.scheduledGeneralParameters.rxCfg.ConfigOptions, 
                              tnetSPSScanParams.scheduledGeneralParameters.rxCfg.FilterOptions,
                              tnetSPSScanParams.scheduledGeneralParameters.scanOptions, 
                              tnetSPSScanParams.scheduledGeneralParameters.numChannels));
    
    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
        ("Channel      BSSID           StartTime     Duration     ET     probChan\n"));
   
    for( i=0; i < tnetSPSScanParams.scheduledGeneralParameters.numChannels; i++)
    {
        ScheduledChannelParameters_t* chanPtr = &tnetSPSScanParams.scheduledChannelParameters[ i ];
        UINT8* pBSSID = (UINT8*)&chanPtr->bssIdL;

        WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
            ("%6d   %02x:%02x:%02x:%02x:%02x:%02x    %5d %5d %2d %5d\n",i,  
            pBSSID[5],pBSSID[4],pBSSID[3],pBSSID[2],pBSSID[1],pBSSID[0],
            chanPtr->scanStartTime, chanPtr->scanMaxDuration, chanPtr->ETCondCount,
            chanPtr->channel));
    }
#endif /* TI_DBG */

    /* Send the scan command*/
    return (whal_hwCtrl_StartSPSScan (pWhalCtrl->pHwCtrl, &tnetSPSScanParams, ScanCommandResponseCB,CB_handle));
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_StopScan
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_StopScan (TI_HANDLE hWhalCtrl , void* ScanCommandResponseCB, TI_HANDLE CB_handle)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    return (whal_hwCtrl_StopScan (pWhalCtrl->pHwCtrl, ScanCommandResponseCB, CB_handle));
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_StopSPSScan
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_StopSPSScan (TI_HANDLE hWhalCtrl, void* ScanCommandResponseCB, TI_HANDLE CB_handle)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    return (whal_hwCtrl_StopSPSScan (pWhalCtrl->pHwCtrl, ScanCommandResponseCB, CB_handle));
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetSplitScanTimeOut
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */

TI_STATUS whalCtrl_SetSplitScanTimeOut (TI_HANDLE hWhalCtrl, UINT32 uTimeOut)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    /* Go all the way to the CmdQueue - According to new architecture ??? */
    return (TI_STATUS)(CmdQueue_Command (pWhalCtrl->hCmdQueue, CMD_TRIGGER_SCAN_TO, (char*)&uTimeOut, sizeof(uTimeOut)));
    
}
/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_JoinBss
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_JoinBss (TI_HANDLE hWhalCtrl, whalCtrl_joinBss_t* pJoinBssParams)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    WlanParams_T *pWlanParams       = whal_ParamsGetWlanParams(pWhalCtrl->pWhalParams);
#ifdef TI_DBG
    UINT8 dbgSsidStr[33];
    BssInfoParams_T *pBssInfoParams = whal_ParamsGetBssInfoParams(pWhalCtrl->pWhalParams);
#endif /* TI_DBG */


    /* for debug purpose, can be removed later*/
    if (pJoinBssParams->ssidLength > 32)
        pJoinBssParams->ssidLength = 32;

#ifdef TI_DBG
    os_memoryCopy(pWhalCtrl->hOs, (void *)dbgSsidStr, (void *)pJoinBssParams->pSSID, pJoinBssParams->ssidLength);
    dbgSsidStr[pJoinBssParams->ssidLength] = '\0';

    /* HW generate packets - CTS - should be at CCK rates only */
    if ((pJoinBssParams->radioBand == RADIO_BAND_2_4_GHZ) &&
        (pJoinBssParams->hwGenCtrlTxRate > DRV_RATE_11M))
    {
        pJoinBssParams->hwGenCtrlTxRate = DRV_RATE_2M; /* default value, if no CCK rate is in the basic rates */
        if(pJoinBssParams->basicRateSet & DRV_RATE_MASK_1_BARKER) pJoinBssParams->hwGenCtrlTxRate = DRV_RATE_1M;
        if(pJoinBssParams->basicRateSet & DRV_RATE_MASK_2_BARKER) pJoinBssParams->hwGenCtrlTxRate = DRV_RATE_2M;
        if(pJoinBssParams->basicRateSet & DRV_RATE_MASK_5_5_CCK)  pJoinBssParams->hwGenCtrlTxRate = DRV_RATE_5_5M;
        if(pJoinBssParams->basicRateSet & DRV_RATE_MASK_11_CCK)   pJoinBssParams->hwGenCtrlTxRate = DRV_RATE_11M;

        WLAN_REPORT_WARNING(pWhalCtrl->hReport,HAL_CTRL_MODULE_LOG,
            ("%s  hwGenCtrlTxRate > 11 !!! changed to %d (rate_e)\n",__FUNCTION__,pJoinBssParams->hwGenCtrlTxRate));

    }

    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                            ("\n whalCtrl_JoinBss :\n \
                              bssType = %d\n \
                              beaconInterval = %d\n \
                              dtimInterval = %d\n \
                              channel = %d \n \
                              BSSID = %x-%x-%x-%x-%x-%x \n \
                              SSID = %s \n \
                              ssidLength = %d \n \
                              basicRateSet = 0x%x \n \
                              supportedRateSet = 0x%x \n ",
                              pJoinBssParams->bssType, 
                              pJoinBssParams->beaconInterval,
                              pJoinBssParams->dtimInterval, 
                              pJoinBssParams->channel,
                              pJoinBssParams->pBSSID[0],
                              pJoinBssParams->pBSSID[1],
                              pJoinBssParams->pBSSID[2],
                              pJoinBssParams->pBSSID[3],
                              pJoinBssParams->pBSSID[4],
                              pJoinBssParams->pBSSID[5],
                              dbgSsidStr, 
                              pJoinBssParams->ssidLength,
                              pJoinBssParams->basicRateSet,
                              pJoinBssParams->supportedRateSet));

      WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                              ("RadioBand = %d \n \
                                Ctrl = 0x%x \n \
                                hwGenCtrlTxRate = 0x%x  \n \
                                hwGenMgmtTxRate = 0x%x  \n \
                                preamble = 0x%x \n",
                                pJoinBssParams->radioBand,
                                pBssInfoParams->Ctrl,
                                pJoinBssParams->hwGenCtrlTxRate, 
                                pJoinBssParams->hwGenMgmtTxRate,
                                pJoinBssParams->preamble));
#endif /* TI_DBG */
    /*
     * save Bss info parameters
     */
    whal_ParamsSetReqBssType(pWhalCtrl->pWhalParams, pJoinBssParams->bssType);
    whal_ParamsSetBssId(pWhalCtrl->pWhalParams, (char *)pJoinBssParams->pBSSID);
    whal_ParamsSetSsid(pWhalCtrl->pWhalParams, (char *)pJoinBssParams->pSSID, pJoinBssParams->ssidLength);
    whal_ParamsSetBeaconInterval(pWhalCtrl->pWhalParams, (UINT16)pJoinBssParams->beaconInterval);
    whal_ParamsSetDtimCount(pWhalCtrl->pWhalParams, (UINT8)pJoinBssParams->dtimInterval);
    whal_ParamsSetRadioChannel(pWhalCtrl->pWhalParams, pJoinBssParams->channel);
    whal_ParamsSetRadioBand(pWhalCtrl->pWhalParams, pJoinBssParams->radioBand);
    whal_ParamsSetBasicRatesSet(pWhalCtrl->pWhalParams, pJoinBssParams->basicRateSet);
    whal_ParamsSetSupportedRatesSet(pWhalCtrl->pWhalParams, pJoinBssParams->supportedRateSet);
    
    /*
     *  Save the frame rates in whalParams and configure it to the Fw later. That command was previously included
     *  in the join command and it is now separated. 
     */
    /* Set the Ctrl frames rate and modulation */
    whal_ParamsSetHwGenTxParams(pWhalCtrl->pWhalParams, pJoinBssParams->hwGenCtrlTxRate, TRUE);
    /* Set the Management frame rate and modulation */
    whal_ParamsSetHwGenTxParams(pWhalCtrl->pWhalParams, pJoinBssParams->hwGenMgmtTxRate, FALSE);
     

    /* In case we're joining a new BSS, reset the TKIP/AES sequence counter. */
    /* The firmware resets its own counter - so we won't have mismatch in the following TX complete events */
    pWhalCtrl->pHwCtrl->SecuritySeqNumLow = 0;
    pWhalCtrl->pHwCtrl->SecuritySeqNumHigh = 0;

    pWlanParams->bJoin = TRUE;
    /*
     * call the hardware to start/join the bss
     */
    return whal_hwCtrl_StartJoin(pWhalCtrl->pHwCtrl, pJoinBssParams->bssType, NULL, NULL);
    
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_ReJoinBss
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_ReJoinBss (TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    return whal_hwCtrl_ReJoinBss ((TI_HANDLE)pWhalCtrl->pHwCtrl);
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_Stop
 *
 * Input    : None
 * Output   :
 * Process  : Send command to the ACX to instruct it to enter a low-power sleep
 *            state
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_Stop (TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    whal_hwCtrl_Stop (pWhalCtrl->pHwCtrl);
    
    return (OK);
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetTemplate
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SetTemplate (TI_HANDLE hWhalCtrl, whalCtrl_setTemplate_t* pTemplateParams)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    HwMboxCmd_T *pHwCmd = whal_hwCtrl_GetMboxCmd(pWhalCtrl->pHwCtrl);
    TemplateListParams_T *pWhalTemplates = &pWhalCtrl->pWhalParams->TemplateList;
    int Stt;

    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                            ("Template Type=%x :: size=%d\n", pTemplateParams->templateType , pTemplateParams->templateLen));

    switch(pTemplateParams->templateType)
    {
        case BEACON_TEMPLATE:
            pWhalTemplates->Beacon.Size = pTemplateParams->templateLen;
            os_memoryCopy(pWhalCtrl->hOs, (void *)&pWhalTemplates->Beacon.Buffer, (void *)pTemplateParams->pTemplate, pTemplateParams->templateLen);
            Stt = whal_hwMboxCmd_ConfigureTemplateFrame(pHwCmd, pTemplateParams->pTemplate, (UINT16)pTemplateParams->templateLen,
                                                        CMD_BEACON,NULL,NULL);
            break;
        case PROBE_RESPONSE_TEMPLATE:
            pWhalTemplates->ProbeResp.Size = pTemplateParams->templateLen;
            os_memoryCopy(pWhalCtrl->hOs, (void *)&pWhalTemplates->ProbeResp.Buffer, (void *)pTemplateParams->pTemplate, pTemplateParams->templateLen);
            Stt = whal_hwMboxCmd_ConfigureTemplateFrame(pHwCmd, pTemplateParams->pTemplate, (UINT16)pTemplateParams->templateLen,
                                                        CMD_PROBE_RESP,NULL,NULL);
            break;
        case PROBE_REQUEST_TEMPLATE:
            pWhalTemplates->ProbeReq.Size = pTemplateParams->templateLen;
            os_memoryCopy(pWhalCtrl->hOs, (void *)&pWhalTemplates->ProbeReq.Buffer, (void *)pTemplateParams->pTemplate, pTemplateParams->templateLen);
            Stt = whal_hwMboxCmd_ConfigureTemplateFrame(pHwCmd, pTemplateParams->pTemplate, (UINT16)pTemplateParams->templateLen,
                                                        CMD_PROBE_REQ,NULL,NULL);
            break;
        case NULL_DATA_TEMPLATE:
            pWhalTemplates->NullData.Size = pTemplateParams->templateLen;
            os_memoryCopy(pWhalCtrl->hOs, (void *)&pWhalTemplates->NullData.Buffer, (void *)pTemplateParams->pTemplate, pTemplateParams->templateLen);
            Stt = whal_hwMboxCmd_ConfigureTemplateFrame(pHwCmd, pTemplateParams->pTemplate, (UINT16)pTemplateParams->templateLen,
                                                        CMD_NULL_DATA,NULL,NULL);
            break;
        case PS_POLL_TEMPLATE:
            pWhalTemplates->PsPoll.Size = pTemplateParams->templateLen;
            os_memoryCopy(pWhalCtrl->hOs, (void *)&pWhalTemplates->PsPoll.Buffer, (void *)pTemplateParams->pTemplate, pTemplateParams->templateLen);
            Stt = whal_hwMboxCmd_ConfigureTemplateFrame(pHwCmd, pTemplateParams->pTemplate, (UINT16)pTemplateParams->templateLen,
                                                        CMD_PS_POLL,NULL,NULL);
            break;
        case QOS_NULL_DATA_TEMPLATE:
            pWhalTemplates->QosNullData.Size = pTemplateParams->templateLen;
            os_memoryCopy(pWhalCtrl->hOs, (void *)&pWhalTemplates->QosNullData.Buffer, (void *)pTemplateParams->pTemplate, pTemplateParams->templateLen);
            Stt = whal_hwMboxCmd_ConfigureTemplateFrame(pHwCmd, pTemplateParams->pTemplate, (UINT16)pTemplateParams->templateLen,
                                                        CMD_QOS_NULL_DATA,NULL,NULL);
            break;
        default:
            Stt = NOK;
            break;
    }

    return (Stt);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetTemplateWithCB
 *
 * Input    :   Same as whalCtrl_SetTemplate but with CB
 * Output   :
 * Process  :
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SetTemplateWithCB (TI_HANDLE hWhalCtrl, whalCtrl_setTemplate_t* pTemplateParams,void *CBFunc,TI_HANDLE CBObj)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    HwMboxCmd_T *pHwCmd = whal_hwCtrl_GetMboxCmd(pWhalCtrl->pHwCtrl);
    TemplateListParams_T *pWhalTemplates = &pWhalCtrl->pWhalParams->TemplateList;
    int Stt;

    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                            ("%s Template Type=%x :: size=%d\n",__FUNCTION__,pTemplateParams->templateType , pTemplateParams->templateLen));

    switch(pTemplateParams->templateType)
    {

    case PROBE_REQUEST_TEMPLATE:
        pWhalTemplates->ProbeReq.Size = pTemplateParams->templateLen;
        os_memoryCopy(pWhalCtrl->hOs, (void *)&pWhalTemplates->ProbeReq.Buffer, (void *)pTemplateParams->pTemplate, pTemplateParams->templateLen);
        Stt = whal_hwMboxCmd_ConfigureTemplateFrame(pHwCmd, pTemplateParams->pTemplate, (UINT16)pTemplateParams->templateLen,CMD_PROBE_REQ,CBFunc,CBObj);
        break;
    default:
            WLAN_REPORT_ERROR (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                            ("%s not implemented yet !!!\n",__FUNCTION__,pTemplateParams->templateType , pTemplateParams->templateLen));
            Stt = NOK;
            break;
    }

    return (Stt);
}
/*
 * ----------------------------------------------------------------------------
 Function : whalCtrl_GetTemplate
 *
 * Input    :  pWhalCtrl - handle to whal ctrl object
 *             templateType - type of template to retrieve
 * Output   :  The template buffer that is saved in the Hal
 * Process  :  If the template type is legal the template buffer and length are
 *             returned. Otherwise NULL is returned
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
TemplateParams_T * whalCtrl_GetTemplate(TI_HANDLE hWhalCtrl, whalCtrl_templateType_e templateType)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    TemplateParams_T * returnTemplate;
    
    switch(templateType)
    {
    case BEACON_TEMPLATE:
        returnTemplate = &pWhalCtrl->pWhalParams->TemplateList.Beacon;
        break;
    case PROBE_RESPONSE_TEMPLATE:
        returnTemplate = &pWhalCtrl->pWhalParams->TemplateList.ProbeResp;
        break;
    case PROBE_REQUEST_TEMPLATE:
        returnTemplate = &pWhalCtrl->pWhalParams->TemplateList.ProbeReq;
        break;
    case NULL_DATA_TEMPLATE:
        returnTemplate = &pWhalCtrl->pWhalParams->TemplateList.NullData;
        break;
    default:
        returnTemplate = NULL;
        break;
    }

    return (returnTemplate);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_Destroy
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_Destroy (TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    DmaParams_T  *pDmaParams=NULL;

    if (pWhalCtrl == NULL)
        return OK;

    if (pWhalCtrl->pWhalParams != NULL)
    {
    pDmaParams = whal_ParamsGetDmaParams(pWhalCtrl->pWhalParams);
    }

    if( (pDmaParams != NULL) && (pWhalCtrl->pWhalSecurity != NULL) )
    {
    if (whalSecur_Destroy (pWhalCtrl->pWhalSecurity, pDmaParams->NumStations) != OK)
        WLAN_REPORT_ERROR(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,  (" whalCtrl_Destroy: whalSecur_Destroy failure \n"));
    }
    
    if (pWhalCtrl->pHwCtrl != NULL)
    {
    if (whal_hwCtrl_Destroy (pWhalCtrl->pHwCtrl) != OK)
        WLAN_REPORT_ERROR(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,  (" whalCtrl_Destroy: whal_hwCtrl_Destroy failure \n"));
    }

    if (pWhalCtrl->pWhalParams != NULL)
    {
    whal_params_Destroy(pWhalCtrl->pWhalParams);
    }
    
    /* free the whalCtrl data structure*/
    os_memoryFree (pWhalCtrl->hOs, pWhalCtrl, sizeof(WHAL_CTRL));
    
    return (OK);
}

/****************************************************************************
 *                      whal_hwCtrl_GetWhalParamsHandle()
 ****************************************************************************
 * DESCRIPTION: Return the handle of the whal params object.
 *
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: handle of the WhalParams object
 ****************************************************************************/
WhalParams_T *whalCtrl_GetWhalParamsHandle(WHAL_CTRL *pWhalCtrl)
{
    return pWhalCtrl->pWhalParams;
}



/*
 * ----------------------------------------------------------------------------
 * Function : Handle DMA Done interrupt 
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
void whalCtrl_HandleBusTxn_Complete(TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    /*
    At this point the interrupts are masked!
    it register at this position and remove its request after the interrupts are enabled again.
    */
    whalBus_TNETWIF_HandleBusTxn_Complete(pWhalCtrl->hWhalBus);
}


/*
 * ----------------------------------------------------------------------------
 * Function : Enable/Disable/Check/Handle interrupts - call HwCtrl object
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_HandleInterrupts (TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    /*
    At this point the interrupts are masked!
    it register at this position and remove its request after the interrupts are enabled again.
    */
    return FwEvent(pWhalCtrl->hFwEvent);
}

void whalCtrl_EnableInterrupts (TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    FwEvent_EnableInterrupts(pWhalCtrl->hFwEvent);
}

void whalCtrl_DisableInterrupts (TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    FwEvent_DisableInterrupts(pWhalCtrl->hFwEvent);
}

UINT32  whalCtrl_CheckInterrupts  (TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    return FwEvent_CheckInterrupts(pWhalCtrl->hFwEvent);
}


void  whalCtr_SlaveAckMaskNotification (TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    FwEvent_StateChanged(pWhalCtrl->hFwEvent);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_isCardIn
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
BOOL whalCtrl_isCardIn (TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    return whalBus_FwCtrl_isCardIn(pWhalCtrl->pHwCtrl->hWhalBus);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_setSend4xWackInfo
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalCtrl_setSend4xWackInfo(TI_HANDLE hWhalCtrl, UINT8 Send4xWackInfo)
{
    /* not implemented */
    return NOK;
}
/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_getSend4xWackInfo
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
int whalCtrl_getSend4xWackInfo(TI_HANDLE hWhalCtrl, UINT8 *Send4xWackInfo)
{
    /* not implemented */
    return NOK;
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetSlotTime
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SetSlotTime (TI_HANDLE hWhalCtrl, slotTime_e SlotTimeVal)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    pWhalCtrl->pWhalParams->WlanParams.SlotTime = SlotTimeVal;      
    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG, ("whalCtrl_SetSlotTime : Slot time = %d\n",
              SlotTimeVal));

    /* Configure the new Slot-Time value to the FW. */
    return whal_hwCtrl_SetSlotTime(pWhalCtrl->pHwCtrl, SlotTimeVal);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetPreamble
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SetPreamble (TI_HANDLE hWhalCtrl, preamble_e preambleVal)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    pWhalCtrl->pWhalParams->WlanParams.preamble= preambleVal;      
    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG, ("whalCtrl_SetPreamble : preamble = %d\n",
              preambleVal));

    return whal_hwCtrl_SetPreamble(pWhalCtrl->pHwCtrl, preambleVal);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetFrameRate
 *
 * Input    : bCtrlFrame - Whether to set new Ctrl rate+modulation or new Mgmt rate+modulation 
 * Output   :
 * Process  :
 * Note(s)  : Modulation is determined using the rate and the previously configured preamble
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SetFrameRate (TI_HANDLE hWhalCtrl,
                            rate_e   txFrmRate,
                            BOOL     bCtrlFrame)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    
    whal_ParamsSetHwGenTxParams (pWhalCtrl->pWhalParams, txFrmRate, bCtrlFrame);            

    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG, 
                ("whalCtrl_SetFrameRate : txCtrlFrmRate = %d , txCtrlFrmModulation = %d , txMgmtFrmRate = %d , txMgmtFrmModulation = %d\n",
                pWhalCtrl->pWhalParams->BssInfoParams.txCtrlFrmRate,pWhalCtrl->pWhalParams->BssInfoParams.txCtrlFrmModulation, 
                pWhalCtrl->pWhalParams->BssInfoParams.txMgmtFrmRate,pWhalCtrl->pWhalParams->BssInfoParams.txMgmtFrmModulation));

    return whal_hwCtrl_SetFrameRate(pWhalCtrl->pHwCtrl,
                                    pWhalCtrl->pWhalParams->BssInfoParams.txCtrlFrmRate,
                                    pWhalCtrl->pWhalParams->BssInfoParams.txCtrlFrmModulation,
                                    pWhalCtrl->pWhalParams->BssInfoParams.txMgmtFrmRate,
                                    pWhalCtrl->pWhalParams->BssInfoParams.txMgmtFrmModulation);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetCwMin
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SetCwMin (TI_HANDLE hWhalCtrl, UINT8 CwMin)
{
  WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    pWhalCtrl->pWhalParams->WlanParams.CwMin = CwMin;
    return OK;
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetPacketDetectionThreshold
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  : Done 
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SetPacketDetectionThreshold (TI_HANDLE hWhalCtrl, UINT8 PDThreshold)
{
  WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
  
  pWhalCtrl->pWhalParams->WlanParams.PacketDetectionThreshold = PDThreshold;
    return whal_hwCtrl_PacketDetectionThreshold(pWhalCtrl->pHwCtrl, &PDThreshold);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetEnergyDetection
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  : Done 
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SetEnergyDetection (TI_HANDLE hWhalCtrl, BOOL energyDetection)
{
  WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    return whal_hwCtrl_SetEnergyDetection(pWhalCtrl->pHwCtrl, energyDetection);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SwitchChannel
 *
 * Input    : channel - newChannelNumber
 * Output   :
 * Process  :
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SwitchChannel(TI_HANDLE hWhalCtrl , UINT8 channel)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    
    return whal_hwCtrl_switchChannel(pWhalCtrl->pHwCtrl, channel);
    
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_DisableTx
 *
 * Input    : None
 * Output   :
 * Process  :
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_DisableTx(TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    
    return whal_hwCtrl_DisableTx(pWhalCtrl->pHwCtrl);
    
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_EnableTx
 *
 * Input    : None
 * Output   :
 * Process  :
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_EnableTx(TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
        
    return whal_hwCtrl_EnableTx(pWhalCtrl->pHwCtrl, whal_ParamsGetRadioChannel(pWhalCtrl->pWhalParams));
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_GetTime
 *
 * Input    : None
 * Output   :
 * Process  :
 * Note(s)  : Get Mac time from the Event Mbox (For Tx expiry Time)
 * -----------------------------------------------------------------------------
 */
UINT32  whalCtrl_GetTime(TI_HANDLE hWhalCtrl)
{
    return 0;
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_setDtimPeriod
 *
 * Input    : dtimPeriod - new Dtim Period
 * Output   :
 * Process  :
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_setDtimPeriod(TI_HANDLE hWhalCtrl, UINT8 dtimPeriod, UINT16 TBTT)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    UINT8 localDtimPeriod = (UINT8)dtimPeriod;
    UINT16 localTBTT = (UINT16)TBTT;

    whal_ParamsSetDtimCount(pWhalCtrl->pWhalParams, (UINT8)dtimPeriod);
 
    whal_hwInfoElemDtimPeriodSet (pWhalCtrl->pHwCtrl->pHwMboxConfig, &localDtimPeriod, &localTBTT);

    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
        ("whalCtrl_setDtimPeriod: DITIM=%d TBTT=%d\n",localDtimPeriod,localTBTT));

    return OK;  
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_GetDtimCount
 *
 * Input    :
 * Output   : UINT8
 * Process  :
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
UINT8 whalCtrl_GetDtimCount(TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    return whal_ParamsGetDtimCount(pWhalCtrl->pWhalParams);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_InterrogateMbox
 *
 * Input    :
 * Output   : UINT8
 * Process  :
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_InterrogateMbox(TI_HANDLE hWhalCtrl , void *CB_Func, TI_HANDLE CB_handle, void *CB_Buf)
{
   WHAL_CTRL           *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
     
   whal_hwInfoElemStationIdForRecoveryGet (pWhalCtrl->pHwCtrl->pHwMboxConfig, CB_Func, CB_handle, CB_Buf);
   
   return OK;
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_InterrogateGwsiStatisitics
 *
 * Input    : Retrieve Statistics
 * Output   :
 * Process  :
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_InterrogateGwsiStatisitics(TI_HANDLE hWhalCtrl, void * CB_Func, TI_HANDLE CB_handle, void * CB_Buf)
{
   WHAL_CTRL * pWhalCtrl = (WHAL_CTRL *) hWhalCtrl;
     
   whal_hwInfoElemAcxReadGwsiStatisiticsGet (pWhalCtrl->pHwCtrl->pHwMboxConfig, CB_Func, CB_handle, CB_Buf);

   return OK;
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_InterrogateGwsiCounters
 *
 * Input    : Retrieve Counters
 * Output   :
 * Process  :
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_InterrogateGwsiCounters(TI_HANDLE hWhalCtrl, void * CB_Func, TI_HANDLE CB_handle, void * CB_Buf)
{
   WHAL_CTRL * pWhalCtrl = (WHAL_CTRL *) hWhalCtrl;
     
   whal_hwInfoElemAcxReadGwsiCountersGet (pWhalCtrl->pHwCtrl->pHwMboxConfig, CB_Func, CB_handle, CB_Buf);

   return OK;
}

    
/* ----------------------------------------------------------------------------
 * Function : whalCtrl_getTsf
 *
 * Input    : hwHalCtrl handle, pTsf container for the FW mac timer
 * Output   :
 * Process  :
 * Note(s)  : The time will be in usec
 * -----------------------------------------------------------------------------
 */
int whalCtrl_getTsf(TI_HANDLE hwHalCtrl, UINT32 *pTsf)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hwHalCtrl;

    if (pTsf == NULL){
            WLAN_REPORT_FATAL_ERROR(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                ("whalCtrl_getTsf: got pTsf paramerter as NULL\n"));

        return NOK;
    }

    return(whal_hwCtrl_getTsf(pWhalCtrl->pHwCtrl, pTsf));
}


int whalCtrl_SendGenCmd (TI_HANDLE hWhalCtrl, char* pBuf, UINT32 Length)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;  
    short CmdId;
    
    if ((Length < CMD_DISABLE_RX) || (pBuf == NULL)){
        WLAN_REPORT_REPLY(pWhalCtrl->hReport, HAL_HW_DATA_MODULE_LOG,
        ("whalCtrl_SendGenCmd: Parameter error\n"));

        return NOK;
    }

    os_memoryCopy(pWhalCtrl, (void *)&CmdId, (void *)pBuf, sizeof(CmdId));
    return (whal_hwCtrl_GenCmd(pWhalCtrl->pHwCtrl, CmdId, pBuf+SIZE_OF_HEADER, (Length - SIZE_OF_HEADER)));
}


int whalCtrl_IsCardInstalled(TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    return whalBus_FwCtrl_isCardIn(pWhalCtrl->pHwCtrl->hWhalBus);
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetBeaconInterval
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
void whalCtrl_SetBeaconInterval(TI_HANDLE hWhalCtrl , UINT16 Val)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    whal_ParamsSetBeaconInterval(pWhalCtrl->pWhalParams ,Val);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_GetBeaconInterval
 *
 * Input    :
 * Output   : UINT16
 * Process  :
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
UINT16 whalCtrl_GetBeaconInterval(TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    return whal_ParamsGetBeaconInterval(pWhalCtrl->pWhalParams);
}


/****************************************************************************
 *                      whalCtrl_exitFromInitMode()
 ****************************************************************************
 * DESCRIPTION: change the interrupt module to work in operational mode
 *              and the Queue to work in Async Mode
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: OK.
 ****************************************************************************/
void whalCtrl_exitFromInitMode(TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    
    /* Notify Event MailBox about init complete */
    eventMbox_InitComplete (pWhalCtrl->hEventMbox);

    /*
    this call must be the last cmd send to the FW because upon its completion the os_complete will be called 
    */
    
    whal_hwInfoElemStationIdGet (pWhalCtrl->pHwCtrl->pHwMboxConfig,
                                     (void*)((TnetwDrv_t*)pWhalCtrl->hTNETW_Driver)->fConfigureEndCB,
                                     ((TnetwDrv_t*)pWhalCtrl->hTNETW_Driver)->fConfigureEndObj,
                                     &pWhalCtrl->pHwCtrl->mbox);

    /* 
     * In case of full asynchronous mode the code below is executed earlier -
     * upon starting configuring the firmware via command mailbox
     */

  #if defined(USE_SYNC_API)

    whalBus_ExitFromInitMode (pWhalCtrl->hWhalBus); 
 
    os_enableIrq (pWhalCtrl->hOs);

  #endif
}

/****************************************************************************
 *                      whalCtrl_exitFromInitModePart1()
 ****************************************************************************
 * DESCRIPTION: Notify Event MailBox about init complete 
 *              
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: OK.
 ****************************************************************************/
void whalCtrl_exitFromInitModePart1(TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    
    /* Notify Event MailBox about init complete */
    eventMbox_InitComplete (pWhalCtrl->hEventMbox);

}

/****************************************************************************
 *                      whalCtrl_exitFromInitModePart2()
 ****************************************************************************
 * DESCRIPTION: change the interrupt module to work in operational mode
 *              and the Queue to work in Async Mode
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: OK.
 ****************************************************************************/
void whalCtrl_exitFromInitModePart2(TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    

    /*
    this call must be the last cmd send to the FW because upon its completion the os_complete will be called 
    */
    
    whal_hwInfoElemStationIdGet (pWhalCtrl->pHwCtrl->pHwMboxConfig,
                                     (void*)((TnetwDrv_t*)pWhalCtrl->hTNETW_Driver)->fConfigureEndCB,
                                     ((TnetwDrv_t*)pWhalCtrl->hTNETW_Driver)->fConfigureEndObj,
                                     &pWhalCtrl->pHwCtrl->mbox);

    /* 
     * In case of full asynchronous mode the code below is executed earlier -
     * upon starting configuring the firmware via command mailbox
     */

  #if defined(USE_SYNC_API)

    whalBus_ExitFromInitMode (pWhalCtrl->hWhalBus); 
 
    os_enableIrq (pWhalCtrl->hOs);

  #endif
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_NoiseHistogramCmd
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_NoiseHistogramCmd(TI_HANDLE hWhalCtrl, whalCtrl_noiseHistogram_t* pNoiseHistParams)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;  

    /* Send the Noise Histogram command*/
    return (whal_hwCtrl_NoiseHistogramCmd(pWhalCtrl->pHwCtrl, pNoiseHistParams));
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_powerMgmtOptionsConfig
 *
 * Input    : 1) TI_HANDLE - handle to the WhalCtrl object.
 *
 * Output   :  TI_STATUS - OK on success else NOK.
 *
 * Process  : configuration of the power managment options mailbox command.
 *
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
TI_STATUS whalCtrl_powerMgmtConfig(TI_HANDLE theWhalCtrlHandle,
                                    whalCtrl_powerSaveParams_t* powerSaveParams)                                            
 /*whalCtrl_powerMgmtConfig_t thePowerMgmtConfig)*/
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL*)theWhalCtrlHandle;
    

    
    int powerMgmtConfStatus;

    /*
    breaking the debug information into 2 section due to the fact that this message is
    too long and exceed message buffer limitation.
    */
    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport,
                            HAL_CTRL_MODULE_LOG,
                            ("whalCtrlPowerSaveParams -:\n\
                             ps802_11Enable = 0x%X\n\
                             hangOverPeriod = 0x%X\n\
                 needToSendNullData = 0x%X\n\
                 numNullPktRetries = 0x%X\n\
                             NullPktRateModulation = 0x%X\n",
                             powerSaveParams->ps802_11Enable,
                             powerSaveParams->hangOverPeriod,
                 powerSaveParams->needToSendNullData,
                 powerSaveParams->numNullPktRetries,
                             powerSaveParams->NullPktRateModulation));


    /*
    PowerMgmtOptions IE
    */

    powerMgmtConfStatus = whal_hwCtrl_PowerMgmtConfigurationSet (pWhalCtrl->pHwCtrl,
                                                                powerSaveParams);

    return (TI_STATUS)powerMgmtConfStatus;
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetBeaconFiltering
 *
 * Input    : UINT8, UINT8
 * Output   :
 * Process  :
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SetBeaconFiltering(TI_HANDLE hWhalCtrl, UINT8 beaconFilteringStatus, UINT8 numOfBeaconsToBuffer)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    

    pWhalCtrl->pWhalParams->WlanParams.beaconFilterParams.desiredState = beaconFilteringStatus;
    pWhalCtrl->pWhalParams->WlanParams.beaconFilterParams.numOfElements = numOfBeaconsToBuffer;


    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport,
                            HAL_CTRL_MODULE_LOG,
                            ("whalCtrl_SetBeaconFiltering  :\n\
                              beaconFilteringStatus = %s \n\
                              numberOfBeaconsToBuffer = %d\n",
                              (beaconFilteringStatus == FALSE) ? "BUFFERING" : "FILTERING",
                              numOfBeaconsToBuffer));
    
    return whal_hwCtrl_SetBeaconFiltering(pWhalCtrl->pHwCtrl, beaconFilteringStatus, numOfBeaconsToBuffer);
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_GetBeaconFiltering
 *
 * Input    : UINT8, UINT8
 * Output   :
 * Process  :
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_GetBeaconFiltering(TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    
    if ( NULL == pWhalCtrl )
    {
        WLAN_OS_REPORT(("whalCtrl_GetBeaconFiltering-hWhalCtrl = NULL !"));
        return NOK ;
    }
    else
    {
        if ( NULL == pWhalCtrl->pWhalParams )
        {
            WLAN_REPORT_ERROR(pWhalCtrl->hReport , HAL_CTRL_MODULE_LOG , ("whalCtrl_GetBeaconFiltering : pWhalParams = NULL !!!")) ;
            return NOK ;
        }
        return (OK);
    }
    
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetBeaconFilterIETable
 *
 * Input    : Number of IE in table, Table, Table szie
 * Output   :
 * Process  : transfer paramaters to the HAL, check for size limit
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SetBeaconFilterIETable(TI_HANDLE hWhalCtrl, UINT8 *numberOfIEs, UINT8 * IETable, UINT8 *IETableSize)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    
    if (*IETableSize > BEACON_FILTER_TABLE_MAX_SIZE)
    {
        WLAN_REPORT_ERROR(pWhalCtrl->hReport,
                          HAL_CTRL_MODULE_LOG,
                          ("whalCtrl_SetBeaconFilterIETable : Table size is too big %d (>%d)\n",
            *IETableSize, BEACON_FILTER_TABLE_MAX_SIZE));    
        return PARAM_VALUE_NOT_VALID;
    }
    
    os_memoryZero (pWhalCtrl->hOs, (void *)pWhalCtrl->pWhalParams->WlanParams.beaconFilterIETable.IETable, BEACON_FILTER_TABLE_MAX_SIZE);
    os_memoryCopy(pWhalCtrl->hOs, (void *)pWhalCtrl->pWhalParams->WlanParams.beaconFilterIETable.IETable, (void *)IETable, *IETableSize);
    pWhalCtrl->pWhalParams->WlanParams.beaconFilterIETable.numberOfIEs  = *numberOfIEs;
    pWhalCtrl->pWhalParams->WlanParams.beaconFilterIETable.IETableSize  = *IETableSize;


    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport,
                            HAL_CTRL_MODULE_LOG,
                            ("whalCtrl_SetBeaconFilterIETable : \n\
                              Number of IE = %d \n\
                              IETable = 0x%p \n\
                              IETableSize = %d\n",
                              *numberOfIEs, IETable, *IETableSize));     
    
    return whal_hwCtrl_SetBeaconFilterIETable(pWhalCtrl->pHwCtrl, numberOfIEs, IETable, IETableSize ) ;
}
  


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_wakeUpCondition
 *
 * Input    : 1) TI_HANDLE - handle to the WhalCtrl object.
 *
 * Output   :  TI_STATUS - OK on success else NOK.
 *
 * Process  : configuration of the power managment options mailbox command.
 *
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
TI_STATUS whalCtrl_wakeUpCondition(TI_HANDLE theWhalCtrlHandle,
                                   whalCtrl_powerMgmtConfig_t thePowerMgmtConfig)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL*)theWhalCtrlHandle;
    whalCtrl_powerMgmtConfig_t *pPowerMgmtOptionsConfig = &thePowerMgmtConfig;
    int status;

    /*
    breaking the debug information into 2 section due to the fact that this message is
    too long and exceed message buffer limitation.
    */
    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport,
                            HAL_CTRL_MODULE_LOG,
                            ("whalCtrl_wakeUpCondition :\n\
                             listenInterval = 0x%X\n",
                             pPowerMgmtOptionsConfig->listenInterval));
                             
    status = whal_hwCtrl_wakeUpCondition (pWhalCtrl->pHwCtrl,
                                         pPowerMgmtOptionsConfig);

    return (TI_STATUS)status;
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_PMConfig
 *
 * Input    : 1) TI_HANDLE - handle to the WhalCtrl object.
 *
 * Output   :  TI_STATUS - OK on success else NOK.
 *
 * Process  : configuration of the power managment options mailbox command.
 *
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
TI_STATUS whalCtrl_PMConfig(TI_HANDLE theWhalCtrlHandle,
                            whalCtrl_powerMgmtConfig_t thePowerMgmtConfig)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL*)theWhalCtrlHandle;
    whalCtrl_powerMgmtConfig_t *pPowerMgmtOptionsConfig = &thePowerMgmtConfig;
    TI_STATUS  status;

    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport,
                            HAL_CTRL_MODULE_LOG,
                            ("whalCtrl_PMConfig :\n\
                             ELPEnable = 0x%X\n\
                             BBWakeUpTime = 0x%X\n\
                             PLLlockTime = 0x%X\n",
                             pPowerMgmtOptionsConfig->ELPEnable,
                             pPowerMgmtOptionsConfig->BBWakeUpTime,
                             pPowerMgmtOptionsConfig->PLLlockTime));

    status = (TI_STATUS)whal_hwCtrl_PMConfig (pWhalCtrl->pHwCtrl, pPowerMgmtOptionsConfig);
    return status;
}
    
/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_BcnBrcOptions
 *
 * Input    : 1) TI_HANDLE - handle to the WhalCtrl object.
 *
 * Output   :  TI_STATUS - OK on success else NOK.
 *
 * Process  : configuration of the power managment options mailbox command.
 *
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
TI_STATUS whalCtrl_BcnBrcOptions(TI_HANDLE theWhalCtrlHandle,
                            whalCtrl_powerMgmtConfig_t thePowerMgmtConfig)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL*)theWhalCtrlHandle;
    whalCtrl_powerMgmtConfig_t *pPowerMgmtBcnBrcOptions = &thePowerMgmtConfig;
    TI_STATUS  status;

    /* Just take the last configured parameter of ConsecutivePsPollDeliveryFailureThreshold */
    pPowerMgmtBcnBrcOptions->ConsecutivePsPollDeliveryFailureThreshold = 
        (whal_ParamsGetWlanParams (pWhalCtrl->pWhalParams))->ConsecutivePsPollDeliveryFailureThreshold;

    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport,
                            HAL_CTRL_MODULE_LOG,
                            ("whalCtrl_BcnBrcOptions :\n\
                             BeaconRxTimeout = 0x%X\n\
                             BroadcastRxTimeout = 0x%X\n\
                             RxBroadcastInPs = 0x%X\n",
                             pPowerMgmtBcnBrcOptions->BcnBrcOptions.BeaconRxTimeout,
                             pPowerMgmtBcnBrcOptions->BcnBrcOptions.BroadcastRxTimeout,
                             pPowerMgmtBcnBrcOptions->BcnBrcOptions.RxBroadcastInPs));

    status = (TI_STATUS)whal_hwCtrl_BcnBrcOptions (pWhalCtrl->pHwCtrl, pPowerMgmtBcnBrcOptions);
    return status;
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_RegisterCmdCompleteGenericCB
 *
 * Input    : 1) hWhalCtrl - this
 *            2) CbFunc    - The Callback   
 *            
 *
 * Note(s)  :  None
 * -----------------------------------------------------------------------------
 */
int  whalCtrl_RegisterCmdCompleteGenericCB(TI_HANDLE hWhalCtrl, void *CbFunc, void *CbObj)
{
    WHAL_CTRL   *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    return CmdQueue_RegisterCmdCompleteGenericCB(((TnetwDrv_t*)pWhalCtrl->hTNETW_Driver)->hCmdQueue,CbFunc,CbObj);
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_EventMbox_RegisterForEvent
 *
 * Input    : 1) hWhalCtrl - this
 *            2) EventBit  - The Event id 
 *            3) CbFunc    - The Callback
 *            4) CbObj     - The Callback Handle
 *
 * Note(s)  :  None
 * -----------------------------------------------------------------------------
 */
int  whalCtrl_EventMbox_RegisterForEvent(TI_HANDLE hWhalCtrl, int EventBit, void *CbFunc, void *CbObj)
{
    WHAL_CTRL   *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    return eventMbox_RegisterEventCB(pWhalCtrl->hEventMbox, 
                                             EventBit, CbFunc, CbObj);
}
/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_EventMbox_Disable
 *
 * Input    : 1) hWhalCtrl - this
 *            2) EventBit  - The Event id 
 *
 * Note(s)  :  None
 * -----------------------------------------------------------------------------
 */
int whalCtrl_EventMbox_Disable(TI_HANDLE hWhalCtrl, int EventBit)
{
    WHAL_CTRL   *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    return eventMbox_EvMask(pWhalCtrl->hEventMbox, EventBit);
}
/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_EventMbox_Enable
 *
 * Input    : 1) hWhalCtrl - this
 *            2) EventBit  - The Event id 
 *
 * Note(s)  :  None
 * -----------------------------------------------------------------------------
 */
int whalCtrl_EventMbox_Enable(TI_HANDLE hWhalCtrl, int EventBit)
{
    WHAL_CTRL   *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl; 

    return eventMbox_EvUnMask(pWhalCtrl->hEventMbox, EventBit);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_GetRadioStandByState
 *
 * Input    : 
 *
 * Note(s)  :  None
 * -----------------------------------------------------------------------------
 */
int whalCtrl_GetRadioStandByState(TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL   *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl; 

    return whalBus_FwCtrl_GetRadioStandByState(pWhalCtrl->pHwCtrl->hWhalBus);
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_GetFWInfo
 *
 * Input    :
 * Output   :  FWInfo
 * Process  :  Retrieves the FWInfo
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
TI_STATUS  whalCtrl_GetFWInfo   (TI_HANDLE hWhalCtrl, whalCtrl_chip_t *pChip_Version)
{
    int i;
    char *StaId;

    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    WlanParams_T *pWlanParams = whal_ParamsGetWlanParams(pWhalCtrl->pWhalParams);

    /*
     * return parameters from the wlan hardware
     */
    StaId = (char *)(whal_ParamsGetSrcMac(pWhalCtrl->pWhalParams));
    for (i=0; i<6; i++)
    {
        pChip_Version->macAddress.addr[i] = StaId[i];
    }
    pChip_Version->preamble = (preamble_e)pWlanParams->preamble;


    /* update the EEPROM version*/
    pChip_Version->e2Ver.major = pWlanParams->majorE2Ver;
    pChip_Version->e2Ver.minor = pWlanParams->minorE2Ver;
    
    /*
     * get radio number and type
     */
    {
        UINT32 RadioType;
        UINT32 RadioNumber;
        
        whalCtrl_getRadioNumber(hWhalCtrl, &RadioType, &RadioNumber);
        
        pChip_Version->radioType = (radioType_e)RadioType;
        pChip_Version->e2Ver.last = RadioNumber;
    }
    
    /* update the firmware version*/
    os_memoryCopy(pWhalCtrl->hOs, (void *)pChip_Version->fwVer,
                  (void *)(whal_ParamsGetFwVersion(pWhalCtrl->pWhalParams)), FW_VERSION_LEN);
    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                             ("FW version is %s\n", pChip_Version->fwVer));

    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                             ("E2 Major version is %d\n", pChip_Version->e2Ver.major));
    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                             ("E2 Minor version is %d\n", pChip_Version->e2Ver.minor));
    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                             ("E2 Last version is %d\n", pChip_Version->e2Ver.last));

    return OK;
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SwitchChannelCmd
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SwitchChannelCmd (TI_HANDLE hWhalCtrl, whalCtrl_switchChannelCmd_t* pSwitchChannelCmd)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;


    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                            ("\n SwitchChannelCmd :\n \
                              channelNumber = %d\n \
                              switchTime = %d\n \
                              txFlag = %d\n \
                              flush = %d \n ",
                             pSwitchChannelCmd->channelNumber,
                             pSwitchChannelCmd->switchTime,
                             pSwitchChannelCmd->txFlag,
                             pSwitchChannelCmd->flush));

    /*
     * save Bss info parameters
     */
    pWhalCtrl->pWhalParams->BssInfoParams.RadioChannel = pSwitchChannelCmd->channelNumber;

    /*
     * call the hardware to start/join the bss
     */
    return whal_hwCtrl_SwitchChannelCmd(pWhalCtrl->pHwCtrl, pSwitchChannelCmd);

}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SwitchChannelCancelCmd
 *
 * Input    :
 * Output   :
 * Process  :
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SwitchChannelCancelCmd (TI_HANDLE hWhalCtrl, UINT8 channel)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                            ("\n whalCtrl_SwitchChannelCancelCmd :\n "));

    /*
     * save Bss info parameters
     */
    pWhalCtrl->pWhalParams->BssInfoParams.RadioChannel = channel;
    
    /*
     * call the hardware to start/join the bss
     */
    return whal_hwCtrl_SwitchChannelCancelCmd(pWhalCtrl->pHwCtrl);

}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetRSSIParamsCmd
 *
 * Input    :   pointer to stuct "whalCtrl_roamingTriggerCmd_t", but only the 
 *              following parameters are relevant;
 *              RSSIthreshold
 *              RSSIFilterWeight
 *              RSSIFilterDepth
 *
 * Output   :
 * Process  :
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SetRSSIParamsCmd (TI_HANDLE hWhalCtrl, whalCtrl_roamingTriggerCmd_t* pRoamingTriggerCmd)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                            ("\n SetRSSIParamsCmd :\n \
                              RSSIthreshold = %d\n \
                              RSSIFilterWeight = %d\n \
                              RSSIFilterDepth = %d \n ",
                              pRoamingTriggerCmd->rssiThreshold,
                              pRoamingTriggerCmd->rssiFilterWeight,
                              pRoamingTriggerCmd->rssiFilterDepth));

    pWhalCtrl->pWhalParams->WlanParams.roamTriggers.rssiThreshold = pRoamingTriggerCmd->rssiThreshold;
    pWhalCtrl->pWhalParams->WlanParams.roamTriggers.rssiFilterWeight = pRoamingTriggerCmd->rssiFilterWeight;
    pWhalCtrl->pWhalParams->WlanParams.roamTriggers.rssiFilterDepth = pRoamingTriggerCmd->rssiFilterDepth;
    pWhalCtrl->pWhalParams->WlanParams.roamTriggers.lowRSSIEventType = pRoamingTriggerCmd->lowRSSIEventType;

    return   whal_hwCtrl_SetRSSIParams( pWhalCtrl->pHwCtrl, pRoamingTriggerCmd);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetSNRParamsCmd
 *
 * Input    :   
 *
 * Output   :
 * Process  :
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SetSNRParamsCmd (TI_HANDLE hWhalCtrl, whalCtrl_roamingTriggerCmd_t *pRoamingTriggerCmd)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    

    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                            ("\n whalCtrl_SetSNRParamsCmd :\n \
                              SNRThreshold = %d\n \
                              SNRFilterWeight = %d\n \
                              SNRFilterDepth = %d \n \
                              EdgeLevel = %d \n ",
                              pRoamingTriggerCmd->snrThreshold,
                              pRoamingTriggerCmd->snrFilterWeight,
                              pRoamingTriggerCmd->snrFilterDepth,
                              pRoamingTriggerCmd->lowSNREventType));


    pWhalCtrl->pWhalParams->WlanParams.roamTriggers.snrThreshold        =  pRoamingTriggerCmd->snrThreshold;
    pWhalCtrl->pWhalParams->WlanParams.roamTriggers.snrFilterWeight         =  pRoamingTriggerCmd->snrFilterWeight;
    pWhalCtrl->pWhalParams->WlanParams.roamTriggers.snrFilterDepth      =  pRoamingTriggerCmd->snrFilterDepth;
    pWhalCtrl->pWhalParams->WlanParams.roamTriggers.lowSNREventType     =  pRoamingTriggerCmd->lowSNREventType;

    return whal_hwCtrl_SetSNRParams(pWhalCtrl->pHwCtrl, &pWhalCtrl->pWhalParams->WlanParams.roamTriggers);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetMaxTxRetryParamsCmd
 *
 * Input    :   pointer to stuct "whalCtrl_roamingTriggerCmd_t", but only the 
 *              following parameters are relevant;
 *              maxTxRetry
 *
 * Output   :
 * Process  :
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SetMaxTxRetryParamsCmd (TI_HANDLE hWhalCtrl, whalCtrl_roamingTriggerCmd_t* pRoamingTriggerCmd)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                            ("\n SetMaxTxRetryParamsCmdCmd :\n \
                              maxTxRetry = %d \n ",
                              pRoamingTriggerCmd->maxTxRetry));
    pWhalCtrl->pWhalParams->WlanParams.roamTriggers.maxTxRetry = pRoamingTriggerCmd->maxTxRetry;

    return   whal_hwCtrl_SetMaxTxRetryParams(pWhalCtrl->pHwCtrl, pRoamingTriggerCmd);
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetBssLossTsfThresholdParamsCmd
 *
 * Input    :   pointer to stuct "whalCtrl_roamingTriggerCmd_t", but only the 
 *              following parameters are relevant;
 *              BssLossTimeout
 *              TsfMissThreshold
 *
 * Output   :
 * Process  :
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SetBssLossTsfThresholdParamsCmd (TI_HANDLE hWhalCtrl, whalCtrl_roamingTriggerCmd_t* pRoamingTriggerCmd)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                            ("\n whalCtrl_SetBssLossTsfThresholdParamsCmd :\n \
                              BssLossTimeout = %d\n \
                              TsfMissThreshold = %d \n ",
                              pRoamingTriggerCmd->BssLossTimeout,
                              pRoamingTriggerCmd->TsfMissThreshold));

    pWhalCtrl->pWhalParams->WlanParams.roamTriggers.BssLossTimeout = pRoamingTriggerCmd->BssLossTimeout;
    pWhalCtrl->pWhalParams->WlanParams.roamTriggers.TsfMissThreshold = pRoamingTriggerCmd->TsfMissThreshold;

    return   whal_hwCtrl_SetBssLossTsfThresholdParams(  pWhalCtrl->pHwCtrl, pRoamingTriggerCmd);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_GetAverageRSSI
 *
 * Input    :   averageRSSI - pointer for return verage RSSI result
 *
 * Output   :   averageRSSI
 * Process  :
 * -----------------------------------------------------------------------------
 */
int whalCtrl_GetAverageRSSI (TI_HANDLE hWhalCtrl, INT8* averageRSSI)
{
#ifdef TI_DBG /* remove the #ifdef TI_DBG when implementing this function */
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                            ("\n GetAverageRSSI :\n \
                              averageRSSI = NOT IMPLEMENTED\n"));
#endif /* TI_DBG */
    return NOK;
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_GetAverageRSSI
 *
 * Input    :   averageRSSI - pointer for return verage RSSI result
 *
 * Output   :   averageRSSI
 * Process  :
 * -----------------------------------------------------------------------------
 */
int whalCtrl_GetAsynRSSI (TI_HANDLE hWhalCtrl,void *CB_Func, TI_HANDLE CB_handle, PUINT8 CB_Buf)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    int status ;

    status = whal_hwCtrl_GetAsynRSSI(pWhalCtrl->pHwCtrl,CB_Func,CB_handle,CB_Buf);

    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                            ("\n whalCtrl_GetAsynRSSI AYNC!!!! :\n "));

    return status;
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_FwDisconnect
 * Input    : None
 * Output   :
 * Process  :
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */

int whalCtrl_FwDisconnect(TI_HANDLE hWhalCtrl, UINT32 ConfigOptions, UINT32 FilterOptions)
{
    WHAL_CTRL *pWhalCtrl        = (WHAL_CTRL *)hWhalCtrl;  
    WlanParams_T *pWlanParams   = whal_ParamsGetWlanParams(pWhalCtrl->pWhalParams);

    pWlanParams->bJoin = FALSE;

    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                             ("Sending FW disconnect, ConfigOptions=%x, FilterOPtions=%x\n",
                              ConfigOptions, FilterOptions));

    return whal_hwCtrl_FwDisconnect(pWhalCtrl->pHwCtrl, ConfigOptions, FilterOptions);
    
} /* whalCtrl_FwDisconnect()*/

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_resetTxCounters
 * Input    : None
 * Output   :
 * Process  : Reset the HAL Tx statistics counters.
 * Note(s)  : Done
 * -----------------------------------------------------------------------------
 */

void whalCtrl_resetTxCounters(TI_HANDLE hWhalCtrl)
{
/*
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    whalBus_resetTxCounters(pWhalCtrl->hWhalBus);
*/
}


/*---------------------------------------------------------
  debug commands for testing the Roaming trigger functions
-----------------------------------------------------------*/
#ifdef ROAMING_TRIGGER_DBG
int whalCtrl_dbgRoamingCommands (TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    whalCtrl_roamingTriggerCmd_t roamingTriggerCmd;
    whalCtrl_roamingTriggerCmd_t *pCmd ;
    int stt1, stt4;
    INT8 rssiVal ;

    pCmd = &roamingTriggerCmd ;

    pCmd->rssiFilterDepth   = 15;
    pCmd->rssiFilterWeight  = 20;
    pCmd->rssiThreshold     = -70;
    pCmd->lowRSSIEventType  = 0;
    stt1 = whalCtrl_SetRSSIParamsCmd (pWhalCtrl, pCmd);

    pCmd->maxTxRetry        = 10;   
    stt1 = whalCtrl_SetMaxTxRetryParamsCmd (pWhalCtrl, pCmd);

    pCmd->BssLossTimeout    = 1;
    pCmd->TsfMissThreshold  = 6;
    stt1 = whalCtrl_SetBssLossTsfThresholdParamsCmd (pWhalCtrl, pCmd);

    stt4 = whalCtrl_GetAverageRSSI(pWhalCtrl,&rssiVal);
    
    return (OK);
}


TI_STATUS whalCtrl_dbgRegisterRoamingEventCB(TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    whalCtrl_EventMbox_RegisterForEvent(pWhalCtrl, 
                                        HAL_EVENT_RSSI_LEVEL, 
                                        (void *)whalCtrl_dbg_RSSI_LEVEL, 
                                        pWhalCtrl);
    whalCtrl_EventMbox_Enable(pWhalCtrl, HAL_EVENT_RSSI_LEVEL);

    whalCtrl_EventMbox_RegisterForEvent(pWhalCtrl, 
                                        HAL_EVENT_SYNCHRONIZATION_TIMEOUT, 
                                        (void *)whalCtrl_dbg_SYNCHRONIZATION, 
                                        pWhalCtrl);
    whalCtrl_EventMbox_Enable(pWhalCtrl, HAL_EVENT_SYNCHRONIZATION_TIMEOUT);

    whalCtrl_EventMbox_RegisterForEvent(pWhalCtrl, 
                                        HAL_EVENT_BSS_LOSE, 
                                        (void *)whalCtrl_dbg_BSS_LOSE, 
                                        pWhalCtrl);
    whalCtrl_EventMbox_Enable(pWhalCtrl, HAL_EVENT_BSS_LOSE);

    whalCtrl_EventMbox_RegisterForEvent(pWhalCtrl, 
                                        HAL_EVENT_MAX_TX_RETRY, 
                                        (void *)whalCtrl_dbg_MAX_TX_RETRY, 
                                        pWhalCtrl);
    whalCtrl_EventMbox_Enable(pWhalCtrl, HAL_EVENT_MAX_TX_RETRY);

    return (OK);
}





static void whalCtrl_dbg_RSSI_LEVEL(TI_HANDLE hWhalCtrl,char* str , UINT32 strLen)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    INT8       averageRssi ;

    os_memoryCopy(pWhalCtrl->hOs, (void *)&averageRssi, (void *)str, strLen);


    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, SITE_MGR_MODULE_LOG,
                           ("got event: whalCtrl_dbg_RSSI_LEVEL, averageRssi=0x%x\n",averageRssi));

}

TI_STATUS whalCtrl_dbg_SYNCHRONIZATION(TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, SITE_MGR_MODULE_LOG,
                           ("got event: whalCtrl_dbg_SYNCHRONIZATION\n"));

    return OK;
}

TI_STATUS whalCtrl_dbg_BSS_LOSE(TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, SITE_MGR_MODULE_LOG,
                           ("got event: whalCtrl_dbg_BSS_LOSE\n"));

    return OK;
}

TI_STATUS whalCtrl_dbg_MAX_TX_RETRY(TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, SITE_MGR_MODULE_LOG,
                           ("got event: whalCtrl_dbg_MAX_TX_RETRY\n"));

    return OK;
}
#endif
/*---------------------------------------------------------
  debug commands for testing the Roaming trigger functions
-----------------------------------------------------------*/


/****************************************************************************
 *                      whalCtrl_measurementParams()
 ****************************************************************************
 * DESCRIPTION: send Command for measurement configuration 
 *              to the mailbox
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whalCtrl_measurementCmd (TI_HANDLE hWhalCtrl, whalCtrl_MeasurementParameters_t *pMeasurementParams,
                             void* CommandResponseCB, TI_HANDLE CB_handle)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    
    return (whal_hwCtrl_measurement (pWhalCtrl->pHwCtrl, pMeasurementParams, CommandResponseCB, CB_handle));
}

/****************************************************************************
 *                      whalCtrl_measurementStop()
 ****************************************************************************
 * DESCRIPTION: send Command for stoping measurement  
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whalCtrl_measurementStop (TI_HANDLE hWhalCtrl,void* CommandResponseCB, TI_HANDLE CB_handle)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    
    return (whal_hwCtrl_measurementStop (pWhalCtrl->pHwCtrl, CommandResponseCB, CB_handle));
}

/****************************************************************************
 *                      whalCtrl_ApDiscoveryCmd()
 ****************************************************************************
 * DESCRIPTION: send Command for AP Discovery 
 *              to the mailbox
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whalCtrl_ApDiscoveryCmd (TI_HANDLE hWhalCtrl, whalCtrl_ApDiscoveryParameters_t* pApDiscoveryParams)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    
    return (whal_hwCtrl_ApDiscovery (pWhalCtrl->pHwCtrl, pApDiscoveryParams));
}

/****************************************************************************
 *                      whalCtrl_ApDiscoveryStop()
 ****************************************************************************
 * DESCRIPTION: send Command for stoping AP Discovery
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whalCtrl_ApDiscoveryStop (TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    
    return (whal_hwCtrl_ApDiscoveryStop (pWhalCtrl->pHwCtrl));
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetGroupIpAddressesTable
 *
 * Input    :
 * Output   :
 * Process  :  Configures the Group table
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_SetGroupAddressesTable(TI_HANDLE hWhalCtrl, 
                                      UINT8 numGroupAddrs, 
                                      macAddress_t *Group_addr,
                                      UINT8 isEnabled)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    if ( numGroupAddrs > MAX_MULTICAST_GROUP_ADDRS) 
    {
            WLAN_REPORT_ERROR(pWhalCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,
                    ("whalCtrl_SetGroupAddressesTable: numGroupAddrs=%d !!!\n", numGroupAddrs));        
        return PARAM_VALUE_NOT_VALID;
    }
    return whal_hwCtrl_SetGroupAddressesTable(pWhalCtrl->pHwCtrl, 
                                              numGroupAddrs, Group_addr,isEnabled);
}

/*
* ----------------------------------------------------------------------------
* Function : whalCtrl_GetGroupIpAddressesTable
*
* Input    :
* Output   :
* Process  :  Retrieve the Group table
* -----------------------------------------------------------------------------
*/
int whalCtrl_GetGroupAddressesTable(TI_HANDLE hWhalCtrl,UINT8* pisEnabled, UINT8* pnumGroupAddrs, macAddress_t *Group_addr)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    return whal_hwCtrl_GetGroupAddressesTable(pWhalCtrl->pHwCtrl, 
        pisEnabled, pnumGroupAddrs, Group_addr);
}


/****************************************************************************
 *                      whalCtrl_ElpCtrl_SetMode()
 ****************************************************************************
 * DESCRIPTION: wrapper function for the lower TNETWIF_ElpCtrl_Mode
 * 
 * INPUTS:  
 *      hWhalCtrl       The current context handle
 *      mode            The ElpCtrl mode
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whalCtrl_ElpCtrl_SetMode(TI_HANDLE hWhalCtrl, elpCtrl_Mode_e mode)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    
    return whalBus_TNETWIF_ElpCtrl_SetMode(pWhalCtrl->hWhalBus, mode);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetMinPowerLevel
 *
 * Input    :   1) theWhalCtrlHandle - handle to the WhalCtrl object.
 *          2) minPowerPolicy - the min power policy to set
 *
 * Output   :  TI_STATUS - OK on success else NOK.
 *
 * Process  : configuration of the min power policy to the FW.
 *
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
 TI_STATUS whalCtrl_SetMinPowerLevel(TI_HANDLE theWhalCtrlHandle,
                                    powerAutho_PowerPolicy_e minPowerPolicy)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL*)theWhalCtrlHandle;
    WlanParams_T *pWlanParams = &pWhalCtrl->pWhalParams->WlanParams;

    /* save th eparameter inside the WlanParams */
    pWlanParams->minPowerLevel = minPowerPolicy;
    
    return (TI_STATUS)whal_hwCtrl_MinPowerLevelSet(pWhalCtrl->pHwCtrl,minPowerPolicy);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_GetMinPowerLevel
 *
 * Input    :   1) theWhalCtrlHandle - handle to the WhalCtrl object.
 *          2) minPowerPolicy - a pointer to the min power policy to get
 *
 * Output   :  TI_STATUS - OK on success else NOK.
 *
 * Process  : gets the min power policy that was configured to the FW.
 *
 * Note(s)  :
 * -----------------------------------------------------------------------------
 */
 TI_STATUS whalCtrl_GetMinPowerLevel(TI_HANDLE theWhalCtrlHandle,
                                    powerAutho_PowerPolicy_e* minPowerPolicy)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL*)theWhalCtrlHandle;
    WlanParams_T *pWlanParams = &pWhalCtrl->pWhalParams->WlanParams;

    /* save th eparameter inside the WlanParams */
    *minPowerPolicy = pWlanParams->minPowerLevel;
    
    return OK;
}



 /*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_SetInfoElemEventMask
 *
 * Input    :  eventMask - Vector to be Masked
 *
 * Process  : set FW with the Masked Vector
 *
 * Note(s)  : called from eventMbox.c
 * -----------------------------------------------------------------------------
 */
 void whalCtrl_SetInfoElemEventMask(TI_HANDLE hWhalCtrl,UINT32 eventMask)
 {
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL*)hWhalCtrl;

    whal_hwInfoElemEventMaskSet (pWhalCtrl->pHwCtrl->pHwMboxConfig, eventMask);
 }
 


/*
* ----------------------------------------------------------------------------
* Function : whalCtrl_RxPER
*
* Input    :    
*
* Output   :    
* Process  :
* -----------------------------------------------------------------------------
*/
int whalCtrl_RxPER(TI_HANDLE hWhalCtrl, PLT_RxPerCmd_e eRxPerCmd, TI_HANDLE CB_Handle, void *CB_Func)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    int status ;
    status = whal_hwCmdBit_RxPER(pWhalCtrl->pHwCtrl->pHwMboxCmdBit, eRxPerCmd, CB_Handle, CB_Func);
    return status;
}

/*
* ----------------------------------------------------------------------------
* Function : whalCtrl_TxCW
*
* Input    :    
*
* Output   :    
* Process  :
* -----------------------------------------------------------------------------
*/
int whalCtrl_TxCW(TI_HANDLE hWhalCtrl, TestCmdChannelBand_t* PltTxCarrier, void * CB_Func, TI_HANDLE CB_handle, void * CB_Buf)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    int status ;
    
    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_HW_CTRL_MODULE_LOG, 
        ("whalCtrl_TxCW: chID = %d bandID = %d\n ",
        PltTxCarrier->channel,
        PltTxCarrier->band));
    
    status = whal_hwCmdBit_Telec(pWhalCtrl->pHwCtrl->pHwMboxCmdBit,PltTxCarrier->channel, PltTxCarrier->band, CB_Func, CB_handle, CB_Buf);
    return status;
}

/*
* ----------------------------------------------------------------------------
* Function : whalCtrl_TxContinues
*
* Input    :    
*
* Output   :    
* Process  :
* -----------------------------------------------------------------------------
*/
int whalCtrl_TxContinues(TI_HANDLE hWhalCtrl, PltTxContinues_t* pPLT_TX_Continues, void * CB_Func, TI_HANDLE CB_handle, void * CB_Buf)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    int status ;

	status = whal_hwCmdBit_Fcc(pWhalCtrl->pHwCtrl->pHwMboxCmdBit,
                      pPLT_TX_Continues->chID, pPLT_TX_Continues->rate,
					  pPLT_TX_Continues->preamble, pPLT_TX_Continues->band,
                      pPLT_TX_Continues->InterPacketDelay, pPLT_TX_Continues->mode, pPLT_TX_Continues->NumOfFrames,
                      pPLT_TX_Continues->aSeqNumMode, pPLT_TX_Continues->aPacketLength, (uint8*)&(pPLT_TX_Continues->aPeerMacAddr),
                      CB_Func, CB_handle, CB_Buf);
    return status;
}

/*
* ----------------------------------------------------------------------------
* Function : whalCtrl_WriteRegister
*
* Input    :    
*
* Output   :    
* Process  :
* -----------------------------------------------------------------------------
*/
int whalCtrl_WriteRegister(TI_HANDLE hWhalCtrl, TI_HANDLE CB_Handle, void* CB_Func, void* CB_Buf)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    int status ;
    
    status = whal_hwCmdBit_WriteRegister(pWhalCtrl->pHwCtrl->pHwMboxCmdBit, CB_Handle, CB_Func, CB_Buf);
    return status;
}


/*
* ----------------------------------------------------------------------------
* Function : whalCtrl_ReadRegister
*
* Input    :    
*
* Output   :    
* Process  :
* -----------------------------------------------------------------------------
*/
int whalCtrl_ReadRegister(TI_HANDLE hWhalCtrl, TI_HANDLE CB_Handle, void* CB_Func, void* CB_Buf)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    int status ;
    
    status = whal_hwCmdBit_ReadRegister(pWhalCtrl->pHwCtrl->pHwMboxCmdBit, CB_Handle, CB_Func, CB_Buf);
    return status;
}

/****************************************************************************************
*                        whalCtrl_ReadMib()                                 
****************************************************************************************
DESCRIPTION:      Read configuration information and statistics

  INPUT:          
  
    OUTPUT:
    
      RETURN:         
      
************************************************************************/
int whalCtrl_ReadMib(TI_HANDLE hWhalCtrl, TI_HANDLE CB_Handle, void* CB_Func, void* CB_Buf)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    PLT_MIB_t* pMibBuf = (PLT_MIB_t*)CB_Buf;
    CmdQueue_InterrogateCB_t RetFunc = (CmdQueue_InterrogateCB_t)CB_Func;
    TI_STATUS Status;
    
    
    
    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
        ("whalCtrl_ReadMib :pMibBuf %p:\n",pMibBuf));
    
    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
        ("whalCtrl_ReadMib :aMib %x:\n",pMibBuf->aMib));
    
    switch (pMibBuf->aMib)
    {
    case PLT_MIB_dot11StationId:
    /* 
    * Use the Station ID CallBack as the Read MIB Cb to get back context 
        */
        return(whalCtrl_InterrogateMbox(hWhalCtrl , CB_Func, CB_Handle, CB_Buf)); 
/*  break; */
        
    case PLT_MIB_dot11MaxReceiveLifetime:
        {
            whalParamInfo_t ParamInfo; 
            ParamInfo.paramType = (UINT32)HAL_CTRL_DOT11_MAX_RX_MSDU_LIFE_TIME;
            ParamInfo.paramLength = sizeof(ParamInfo.content.halCtrlMaxRxMsduLifetime);
            Status = whalCtrl_GetParam(hWhalCtrl, &ParamInfo);
            pMibBuf->aData.MaxReceiveLifeTime = ParamInfo.content.halCtrlMaxRxMsduLifetime / 1024; /* converting from usecs to TUs*/
            pMibBuf->Length = sizeof(pMibBuf->aData.MaxReceiveLifeTime);
            RetFunc(CB_Handle, Status, (void*)pMibBuf);
        }
        break;
        
        
    case PLT_MIB_dot11GroupAddressesTable:
        {
            Status = (TI_STATUS)whalCtrl_GetGroupAddressesTable(
                hWhalCtrl, 
                &pMibBuf->aData.GroupAddressTable.bFilteringEnable,
                &pMibBuf->aData.GroupAddressTable.nNumberOfAddresses,
                pMibBuf->aData.GroupAddressTable.GroupTable);
            
               pMibBuf->Length = sizeof(pMibBuf->aData.GroupAddressTable.bFilteringEnable) + 
                                 sizeof(pMibBuf->aData.GroupAddressTable.nNumberOfAddresses) +
                                 pMibBuf->aData.GroupAddressTable.nNumberOfAddresses * sizeof(macAddress_t);
            
            RetFunc(CB_Handle, Status, CB_Buf);
        }

        break;
        
    case PLT_MIB_ctsToSelf:
        {
            whalParamInfo_t ParamInfo;          
            ParamInfo.paramType = (UINT32)HAL_CTRL_CTS_TO_SELF_PARAM;
            ParamInfo.paramLength = sizeof(ParamInfo.content.halCtrlCtsToSelf);
            Status = whalCtrl_GetParam(hWhalCtrl, &ParamInfo);
            pMibBuf->aData.CTSToSelfEnable = ParamInfo.content.halCtrlCtsToSelf;
            pMibBuf->Length = sizeof(pMibBuf->aData.CTSToSelfEnable);
            RetFunc(CB_Handle, Status, CB_Buf);
        }
        break;
        
    case PLT_MIB_arpIpAddressesTable:
        {
            IpAddress_t IpAddress;  
            IPver_e IPver;
            UINT8 Enable;
            TI_STATUS status;
            
            pMibBuf->Length = sizeof(PLT_MIB_ArpIpAddressesTable_t);
            status = (TI_STATUS)whalCtrl_GetArpIpAddressesTable(pWhalCtrl->pHwCtrl, 
                                                    &IpAddress, 
                                                    &Enable,
                                                    &IPver);
            if (status == OK)
            {
                pMibBuf->aData.ArpIpAddressesTable.FilteringEnable = Enable;         

                if (IP_VER_4 == IPver) /* IP_VER_4 only */
                {
                    os_memoryCopy(pWhalCtrl->hOs,
                        (PVOID)pMibBuf->aData.ArpIpAddressesTable.addr, 
                        (PVOID)IpAddress.addr,
                        IP_V4_ADDR_LEN);                    
                }
                else
                {
                    status = NOK;
                }
            }
            RetFunc(CB_Handle, status, CB_Buf);
            return status;
        }


        /*break; Unreachble code*/
        
    case PLT_MIB_templateFrame:
        whalCtrl_ReadTemplateFrameMib(hWhalCtrl, CB_Handle, CB_Func,  CB_Buf);
        break;
        
    case PLT_MIB_rxFilter:
        {
            UINT32 RxConfigOption;
            UINT32 RxFilterOption;
            
            pMibBuf->Length = 1;
            pMibBuf->aData.RxFilter = 0;

            /*Get RX filter data*/
            Status = (TI_STATUS)whalCtrl_GetRxFilters(pWhalCtrl, &RxConfigOption, &RxFilterOption);
            if (OK == Status)
            {
                /*Translate to MIB bitmap*/
                if ((RxConfigOption & RX_CFG_MAC) == RX_CFG_ENABLE_ANY_DEST_MAC)
                    pMibBuf->aData.RxFilter |= PLT_MIB_RX_FILTER_PROMISCOUS_SET;
                
                if ((RxConfigOption & RX_CFG_BSSID) == RX_CFG_ENABLE_ONLY_MY_BSSID)
                    pMibBuf->aData.RxFilter |= PLT_MIB_RX_FILTER_BSSID_SET;
            }
            RetFunc(CB_Handle, Status, CB_Buf);         
        }
        break;

    case PLT_MIB_beaconFilterIETable:
        return (whalCtrl_PltMibGetBeaconFilterIETable(hWhalCtrl, CB_Handle, CB_Func, CB_Buf));
/*        break; */

    case PLT_MIB_txRatePolicy:
        return (whalCtrl_PLT_ReadMIB_TxRatePolicy(hWhalCtrl, CB_Handle, CB_Func, CB_Buf));
/*      break; */
        

    case PLT_MIB_countersTable:
        return(whalCtrl_InterrogateGwsiCounters (hWhalCtrl , CB_Func, CB_Handle, CB_Buf));
/*      break; */


    case PLT_MIB_statisticsTable:
        return (whalCtrl_InterrogateGwsiStatisitics(hWhalCtrl , CB_Func, CB_Handle, CB_Buf));
/*        break; */

    default:
        WLAN_REPORT_ERROR(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
            ("whalCtrl_ReadMib:MIB aMib 0x%x Not supported\n",pMibBuf->aMib));
        return NOK;
    }
    return OK;
    
}

/****************************************************************************************
*                        whalCtrl_WriteMib()                                 
****************************************************************************************
DESCRIPTION:      Set configuration information

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
int whalCtrl_WriteMib(TI_HANDLE hWhalCtrl, PLT_MIB_t* pMib)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
   
    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
        ("whalCtrl_WriteMib :pMib %p:\n",pMib));
    
    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
        ("whalCtrl_WriteMib :aMib %x:\n",pMib->aMib));

    WLAN_REPORT_HEX_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
        (PUINT8)pMib, min(sizeof(PLT_MIB_t), pMib->Length));
    
    if (NULL == pMib)
    {
        WLAN_REPORT_ERROR(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
        ("whalCtrl_WriteMib :pMib = NULL !!\n"));
        return PARAM_VALUE_NOT_VALID;
    }

  
    switch (pMib->aMib)
    {   
    case PLT_MIB_dot11MaxReceiveLifetime:
        {
            whalParamInfo_t ParamInfo;
            ParamInfo.paramType = (UINT32)HAL_CTRL_DOT11_MAX_RX_MSDU_LIFE_TIME;
            ParamInfo.paramLength = sizeof(ParamInfo.content.halCtrlMaxRxMsduLifetime);
            ParamInfo.content.halCtrlMaxRxMsduLifetime = pMib->aData.MaxReceiveLifeTime;
            ParamInfo.content.halCtrlMaxRxMsduLifetime *= 1024; /* converting from TUs to usecs */
            return whalCtrl_SetParam(hWhalCtrl, &ParamInfo);
        }
/*        break;  */
        
    case PLT_MIB_ctsToSelf:
        {
            whalParamInfo_t ParamInfo;
            ParamInfo.paramType = (UINT32)HAL_CTRL_CTS_TO_SELF_PARAM;
            ParamInfo.paramLength = sizeof(ParamInfo.content.halCtrlCtsToSelf);
            ParamInfo.content.halCtrlCtsToSelf = pMib->aData.CTSToSelfEnable;
            return whalCtrl_SetParam(hWhalCtrl, &ParamInfo);
        }
/*        break; */
        
    case PLT_MIB_dot11GroupAddressesTable: 
        {
            
            if ( NULL == pMib->aData.GroupAddressTable.GroupTable)
            {
                WLAN_REPORT_ERROR(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                    ("whalCtrl_WriteMib(PLT_MIB_dot11GroupAddressesTable) :GroupTable = NULL !!\n"));
                return PARAM_VALUE_NOT_VALID;
            }
            
            return whalCtrl_SetGroupAddressesTable(hWhalCtrl, 
                pMib->aData.GroupAddressTable.nNumberOfAddresses,
                pMib->aData.GroupAddressTable.GroupTable,
                pMib->aData.GroupAddressTable.bFilteringEnable);
        }
/*        break;  */
        
    case PLT_MIB_arpIpAddressesTable:
        {
            IpAddress_t IpAddress;
            IpAddress.addr[0] =  pMib->aData.ArpIpAddressesTable.addr[0];
            IpAddress.addr[1] =  pMib->aData.ArpIpAddressesTable.addr[1];
            IpAddress.addr[2] =  pMib->aData.ArpIpAddressesTable.addr[2];
            IpAddress.addr[3] =  pMib->aData.ArpIpAddressesTable.addr[3];

            WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                ("whalCtrl_WriteMib(PLT_MIB_arpIpAddressesTable) IpAddress:\n"));
            WLAN_REPORT_HEX_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                                        (PUINT8)&IpAddress, 4);

            return whalCtrl_SetarpIpAddressesTable(hWhalCtrl, 
                &IpAddress,
                pMib->aData.ArpIpAddressesTable.FilteringEnable,
                IP_VER_4);
        }
/*        break; */
        
    case PLT_MIB_templateFrame:
        return whalCtrl_WriteTemplateFrameMib(hWhalCtrl, pMib);
/*        break; */
        
    case PLT_MIB_beaconFilterIETable:
        return whalCtrl_PltMibSetBeaconFilterIETable(hWhalCtrl, pMib);
/*        break;  */
        
    case PLT_MIB_rxFilter:
        {
            UINT32 whal_rx_filter = 0;
            tiUINT8 Mib_Rx_Filter = pMib->aData.RxFilter;
            
            /* 
            * Construct the WHAL rx filter element
            */
            if (Mib_Rx_Filter & PLT_MIB_RX_FILTER_PROMISCOUS_SET )
            {
                WLAN_REPORT_INFORMATION(pWhalCtrl->hReport,HAL_CTRL_MODULE_LOG,("\n whalCtrl_WriteMib PLT_MIB_rxFilter - RX_CFG_ENABLE_ANY_DEST_MAC\n")) ;
                whal_rx_filter = RX_CFG_ENABLE_ANY_DEST_MAC;
            }
            else
            {
                whal_rx_filter = RX_CFG_ENABLE_ONLY_MY_DEST_MAC;
                WLAN_REPORT_INFORMATION(pWhalCtrl->hReport,HAL_CTRL_MODULE_LOG,("\n halCtrl_WriteMib PLT_MIB_rxFilter - RX_CFG_ENABLE_ONLY_MY_DEST_MAC\n")) ;
            }
            
            if ( Mib_Rx_Filter & PLT_MIB_RX_FILTER_BSSID_SET )
            {
                whal_rx_filter = whal_rx_filter | RX_CFG_ENABLE_ONLY_MY_BSSID;
                WLAN_REPORT_INFORMATION(pWhalCtrl->hReport,HAL_CTRL_MODULE_LOG,("\n halCtrl_WriteMib PLT_MIB_rxFilter - RX_CFG_ENABLE_ONLY_MY_BSSID\n")) ;
            }
            else
            {
                whal_rx_filter = whal_rx_filter | RX_CFG_ENABLE_ANY_BSSID;
                WLAN_REPORT_INFORMATION(pWhalCtrl->hReport,HAL_CTRL_MODULE_LOG,("\n halCtrl_WriteMib PLT_MIB_rxFilter - RX_CFG_ENABLE_ANY_BSSID\n") );
            }
            
            /*
            * Activates the whalCtrl_setRxFilters function 
            */
            return whalCtrl_setRxFilters(hWhalCtrl, whal_rx_filter, RX_FILTER_OPTION_DEF);
            
        }
/*        break;  */

    case PLT_MIB_txRatePolicy:
        return whalCtrl_PLT_WriteMIB_TxRatePolicy(hWhalCtrl, pMib);
/*      break;  */

    default:
        WLAN_REPORT_ERROR(pWhalCtrl->hReport,
            HAL_CTRL_MODULE_LOG,
            ("%s(%d) - whalCtrl_WriteMib - ERROR - MIB element not supported, %d\n\n",
            __FILE__,__LINE__,pMib->aMib));
        
        return NOK;
        
    } /* switch */
    
/*    return OK;*/
}


int whalCtrl_ReadTemplateFrameMib(TI_HANDLE hWhalCtrl, TI_HANDLE CB_Handle, void* CB_Func, void* CB_Buf)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    PLT_MIB_t* pMibBuf = (PLT_MIB_t*)CB_Buf;
    CmdQueue_InterrogateCB_t RetFunc = (CmdQueue_InterrogateCB_t)CB_Func;
    TemplateParams_T* pTemplateParams;
    whalCtrl_templateType_e templateType;
      
    switch(pMibBuf->aData.TemplateFrame.FrameType)
    {
    case PLT_TEMPLATE_TYPE_BEACON:
        templateType = BEACON_TEMPLATE;
        pMibBuf->aData.TemplateFrame.Rate = pWhalCtrl->pWhalParams->BssInfoParams.txCtrlFrmRateDriverFormat; 
        break;

    case PLT_TEMPLATE_TYPE_PROBE_REQUEST:
        templateType = PROBE_REQUEST_TEMPLATE;
        pMibBuf->aData.TemplateFrame.Rate = DRV_RATE_INVALID;
        break;
        
    case PLT_TEMPLATE_TYPE_NULL_FRAME:
        {
            TI_HANDLE hHalCtrl;
            TI_HANDLE hMacServices;
            TnetwDrv_TEMP_GetHandles(pWhalCtrl->hTNETW_Driver, &hHalCtrl, &hMacServices);
            pMibBuf->aData.TemplateFrame.Rate = MacServices_powerSrv_GetRateModulation(hMacServices);
            templateType = NULL_DATA_TEMPLATE;
        }
        break;

    case PLT_TEMPLATE_TYPE_PROBE_RESPONSE:
        templateType = PROBE_RESPONSE_TEMPLATE;
        pMibBuf->aData.TemplateFrame.Rate = pWhalCtrl->pWhalParams->BssInfoParams.txCtrlFrmRateDriverFormat; 
        break;

    case PLT_TEMPLATE_TYPE_QOS_NULL_FRAME:
        templateType = QOS_NULL_DATA_TEMPLATE;
        pMibBuf->aData.TemplateFrame.Rate = pWhalCtrl->pWhalParams->BssInfoParams.txCtrlFrmRateDriverFormat; 
        break;

    case PLT_TEMPLATE_TYPE_PS_POLL:
        templateType = PS_POLL_TEMPLATE;
        pMibBuf->aData.TemplateFrame.Rate = pWhalCtrl->pWhalParams->BssInfoParams.txCtrlFrmRateDriverFormat; 
        break;
    default:
           WLAN_REPORT_ERROR(pWhalCtrl->hReport,
                HAL_CTRL_MODULE_LOG,
                ("%s(%d) - whalCtrl_ReadTemplateFrameMib - ERROR - template is not supported, %d\n\n",
                __FILE__,__LINE__,pMibBuf->aData.TemplateFrame.FrameType));
            return PARAM_NOT_SUPPORTED;
        }
   
    pTemplateParams =  whalCtrl_GetTemplate(hWhalCtrl, templateType);
    
    if (pTemplateParams)
    {
        pMibBuf->Length = pTemplateParams->Size + 10;
        
        pMibBuf->aData.TemplateFrame.Length = pTemplateParams->Size;
        
        os_memoryCopy(pWhalCtrl->hOs,
            pMibBuf->aData.TemplateFrame.Data,
            pTemplateParams->Buffer,
            pTemplateParams->Size);
        
    
        pMibBuf->aData.TemplateFrame.Rate = whalUtils_DRV_RATE2GwsiRate(pMibBuf->aData.TemplateFrame.Rate);
        RetFunc(CB_Handle, OK, CB_Buf);
        return OK;
    }

    RetFunc(CB_Handle, NOK, CB_Buf); 
    return NOK;
}

int whalCtrl_WriteTemplateFrameMib(TI_HANDLE hWhalCtrl, PLT_MIB_t* pMib)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    rate_e rate;
    whalCtrl_setTemplate_t   whal_set_template_s;
    
    
    /*convert the rate to driver rate*/
    rate = (rate_e)whalUtils_GwsiRate2DRV_RATE(pMib->aData.TemplateFrame.Rate);
    
    /* 
    * Construct the template MIB element
    */
    switch(pMib->aData.TemplateFrame.FrameType)
    {
    case PLT_TEMPLATE_TYPE_BEACON:
        whal_set_template_s.templateType = BEACON_TEMPLATE;

        /* Set new Mgmt rate (write it to Fw along with the modulation) */
        whalCtrl_SetFrameRate(hWhalCtrl, rate, FALSE);
        
        break;
        
    case PLT_TEMPLATE_TYPE_PROBE_REQUEST:
        whal_set_template_s.templateType = PROBE_REQUEST_TEMPLATE;
        break;
        
    case PLT_TEMPLATE_TYPE_NULL_FRAME:
        {
            TI_HANDLE hHalCtrl;
            TI_HANDLE hMacServices;
            TnetwDrv_TEMP_GetHandles(pWhalCtrl->hTNETW_Driver, &hHalCtrl, &hMacServices);
            
            whal_set_template_s.templateType = NULL_DATA_TEMPLATE;
            MacServices_powerSrv_SetRateModulation(hMacServices,
                (UINT16)whalUtils_GwsiRate2DRV_RATE_MASK(pMib->aData.TemplateFrame.Rate));
        }
        break;
        
    case PLT_TEMPLATE_TYPE_PROBE_RESPONSE:
        whal_set_template_s.templateType = PROBE_RESPONSE_TEMPLATE;

        /* Set new Mgmt rate (write it to Fw along with the modulation) */        
        whalCtrl_SetFrameRate(hWhalCtrl, rate, FALSE);
        
        break;
        
    case PLT_TEMPLATE_TYPE_QOS_NULL_FRAME:
        whal_set_template_s.templateType = QOS_NULL_DATA_TEMPLATE;

        /* Set new Ctrl rate (write it to Fw along with the modulation) */        
        whalCtrl_SetFrameRate(hWhalCtrl, rate, TRUE);
        
        break;
        
    case PLT_TEMPLATE_TYPE_PS_POLL:
        whal_set_template_s.templateType = PS_POLL_TEMPLATE;

        /* Set new Ctrl rate (write it to Fw along with the modulation) */        
        whalCtrl_SetFrameRate(hWhalCtrl, rate, TRUE);
        
        break;
        
    default:
        WLAN_REPORT_ERROR(pWhalCtrl->hReport,
            HAL_CTRL_MODULE_LOG,
            ("%s(%d) - whalCtrl_WriteTemplateFrameMib - ERROR - template is not supported, %d\n\n",
            __FILE__,__LINE__,pMib->aData.TemplateFrame.FrameType));
        return PARAM_NOT_SUPPORTED;
    }
    
    whal_set_template_s.templateLen = pMib->aData.TemplateFrame.Length;
    whal_set_template_s.pTemplate = (UINT8 *) &(pMib->aData.TemplateFrame.Data);
    
    
    return whalCtrl_SetTemplate(hWhalCtrl, &whal_set_template_s);
}

int whalCtrl_PltMibSetBeaconFilterIETable(TI_HANDLE hWhalCtrl, PLT_MIB_t* pMib)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *) hWhalCtrl;
    UINT8 numOf221IE = 0 ;
    UINT8 i = 0 ;
    UINT8 IETableLen = 0 ;
    UINT8 numOfIEs = 0 ;
    UINT8 *IETable = NULL ;
    
    numOfIEs = pMib->aData.BeaconFilter.iNumberOfIEs;
    IETable = pMib->aData.BeaconFilter.iIETable;
    /*find the actual IETableLen */
    for ( i = 0 ; i < numOfIEs ; i++ )
    {
        if ( IETable[IETableLen] == 0xdd )
        {
             IETableLen += 8;
             numOf221IE++;
        }
        else
            IETableLen += 2;
    }
    
    WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
        ("\nwhalCtrl_PltMibSetBeaconFilterIETable,IETable=0x%x Num Of IE=%d ( including %d 221 ) - Table Len=%d\n",
        IETable , numOfIEs , numOf221IE , IETableLen ));
    
    return whalCtrl_SetBeaconFilterIETable(hWhalCtrl, &numOfIEs, IETable, &IETableLen);
}

int whalCtrl_PltMibGetBeaconFilterIETable(TI_HANDLE hWhalCtrl, TI_HANDLE CB_Handle, void* CB_Func, void* CB_Buf)
{
    PLT_MIB_t* pMib = (PLT_MIB_t*)CB_Buf;
    CmdQueue_InterrogateCB_t RetFunc = (CmdQueue_InterrogateCB_t)CB_Func;
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    UINT8 IETableSize = 0;


    /*Get params*/
    pMib->aData.BeaconFilter.iNumberOfIEs = pWhalCtrl->pWhalParams->WlanParams.beaconFilterIETable.numberOfIEs;
    IETableSize = pWhalCtrl->pWhalParams->WlanParams.beaconFilterIETable.IETableSize;
                  
    os_memoryZero (pWhalCtrl->hOs, 
                   pMib->aData.BeaconFilter.iIETable,
                   sizeof(pMib->aData.BeaconFilter.iIETable));

    os_memoryCopy(pWhalCtrl->hOs, 
                  pMib->aData.BeaconFilter.iIETable,
                  pWhalCtrl->pWhalParams->WlanParams.beaconFilterIETable.IETable,
                  IETableSize);
    pMib->Length = IETableSize + 1;     

    RetFunc(CB_Handle, OK, CB_Buf); 
    return OK;

}

/**
 * \author \n
 * \date \n
 * \brief Coordinates between legacy TxRatePolicy implementation and the MIB format: \n
 *        Converts the pGwsi_txRatePolicy back to whal commands 
 *        Activates the whal whalCtrl_set function 
 * Function Scope \e Public.\n
 * \param  - \n
 * \return \n
 */

int whalCtrl_PLT_WriteMIB_TxRatePolicy(TI_HANDLE hWhalCtrl, PLT_MIB_t* pMib)                /* Pointer to the MIB data*/
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    whalParamInfo_t param;

    if (NULL == pMib)
    {
        WLAN_REPORT_ERROR(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
            ("ERROR : whalCtrl_PLT_WriteMIB_TxRatePolicy pMib=NULL !!!"));
    }


    param.paramType = (UINT32)HAL_CTRL_TX_RATE_CLASS_PARAMS;
    param.content.pTxRatePlicy = &pMib->aData.txRatePolicy;

    /*
     * Call WhalCtrl Set I/F
     */
    return(whalCtrl_SetParam(hWhalCtrl, &param));
}

/**
 * \author \n
 * \date \n
 * \brief Coordinates between legacy TxRatePolicy implementation and the MIB format: \n
 *        Converts the pGwsi_txRatePolicy back to whal commands 
 *        Activates the whal whalCtrl_set function 
 * Function Scope \e Public.\n
 * \param  - \n
 * \return \n
 */

int whalCtrl_PLT_ReadMIB_TxRatePolicy(TI_HANDLE hWhalCtrl, TI_HANDLE CB_Handle, void* CB_Func, void* CB_Buf)
{
    PLT_MIB_t* pMib = (PLT_MIB_t*)CB_Buf;
    CmdQueue_InterrogateCB_t RetFunc = (CmdQueue_InterrogateCB_t)CB_Func;
    whalParamInfo_t     param;
    tiUINT32 Status = OK;


    param.paramType = (UINT32)HAL_CTRL_TX_RATE_CLASS_PARAMS;
    whalCtrl_GetParam(hWhalCtrl, &param);
    if (param.content.pTxRatePlicy == NULL)
        Status = NOK;

    /*Copy the data form the param to the MIB*/
    pMib->aData.txRatePolicy = *param.content.pTxRatePlicy;
    pMib->Length = pMib->aData.txRatePolicy.numOfRateClasses * sizeof(pMib->aData.txRatePolicy.rateClass[0]) + 
                       sizeof(pMib->aData.txRatePolicy.numOfRateClasses);
    RetFunc(CB_Handle, Status, CB_Buf);
    return Status;
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_updateSecuritySeqNum
 *
 * Process  : Update the current TKIP/AES security-sequence-number according to the last
 *              Tx data packet seq-number, for reloading it to the FW in case of recovery.
 *            The complete 32 bit number is deduced from the 8 LS bits provided by the FW
 *              in the Tx-Result, assuming the total number is never incremented in more 
 *              than 255 per one packet (limited by max fragments per packet).
 *
 * Input    : 1) theWhalCtrlHandle - handle to the WhalCtrl object.
 *            2) securitySeqNumLsByte - the LS byte of the last Tx frame security-sequence-number.
 *
 * -----------------------------------------------------------------------------
 */
 void whalCtrl_updateSecuritySeqNum(TI_HANDLE hWhalCtrl, UINT8 securitySeqNumLsByte)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL*)hWhalCtrl;
    HwCtrl_T  *pHwCtrl   = (HwCtrl_T *)pWhalCtrl->pHwCtrl;

    /* If 8 lsb wrap around occurred (new < old). */
    if ( (UINT16)securitySeqNumLsByte < (pHwCtrl->SecuritySeqNumLow & 0xFF))
    {
        /* Increment the upper byte of the 16 lsb. */       
        pHwCtrl->SecuritySeqNumLow += 0x100;
        
        /* If 16 bit wrap around occurred, increment the upper 32 bit. */
        if( !(pHwCtrl->SecuritySeqNumLow & 0xFF00) )
            pHwCtrl->SecuritySeqNumHigh++;
    }

    /* Save new sequence number 8 lsb (received from the FW). */
    pHwCtrl->SecuritySeqNumLow &= 0xFF00;
    pHwCtrl->SecuritySeqNumLow |= (UINT16)securitySeqNumLsByte;
}


 /*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_setBetParams
 *
 * Input    :   enabled               - 0 to disable BET, 0 to disable BET
 *              MaximumConsecutiveET  - Max number of consecutive beacons
 *                                      that may be early terminated.
 * Output   :
 * Process  :  Configures Beacon Early Termination information element.
 * Note(s)  :  None
 * -----------------------------------------------------------------------------
 */
int whalCtrl_setBetParams(TI_HANDLE hWhalCtrl, UINT8 Enable, UINT8 MaximumConsecutiveET)
{
    WHAL_CTRL * pWhalCtrl = (WHAL_CTRL *) hWhalCtrl;

    pWhalCtrl->pWhalParams->WlanParams.BetEnable = Enable;
    pWhalCtrl->pWhalParams->WlanParams.MaximumConsecutiveET = MaximumConsecutiveET;

    return whal_hwCtrl_setBetParams(pWhalCtrl->pHwCtrl, Enable, MaximumConsecutiveET);
}
 
