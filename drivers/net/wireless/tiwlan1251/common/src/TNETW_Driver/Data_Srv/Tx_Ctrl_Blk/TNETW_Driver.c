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

/****************************************************************************
 *
 *   MODULE:    TNETW_Driver.c
 *   PURPOSE:   TNETW_Driver layer used to handle the MAC Services 
 *              and the HAL Modules 
 *
 ****************************************************************************/

#include "commonTypes.h"
#include "report.h"
#include "TNETW_Driver_api.h"
#include "MacServices_api.h"
#include "txCtrlBlk_api.h"
#include "txHwQueue_api.h"
#include "txXfer_api.h"
#include "txResult_api.h"
#include "rxXfer_api.h"
#include "TNETWIF.h"
#include "FwEvent_api.h"
#include "CmdMBox_api.h"
#include "CmdQueue_api.h"
#include "eventMbox_api.h"
#ifdef TI_DBG
#include "DebugTraceXfer_api.h"
#endif /* TI_DBG */
#include "osApi.h"
#include "TNETW_Driver.h"
#include "recoveryCtrl_API.h"
#include "HwInit_api.h"

static TI_STATUS TnetwDrv_SetInitParams (TI_HANDLE hTnetwDrv, TnetwDrv_InitParams_t* pInitParams);

#ifdef GWSI_LIB
void GWSI_FinalizeDownload(TI_HANDLE hGwsiMgr, TI_STATUS eStatus);
#endif

void TnetwDrv_TxXferDone (TI_HANDLE hTnetwDrv, txCtrlBlkEntry_t *pPktCtrlBlk);
void TnetwDrv_TxComplete (TI_HANDLE hTnetwDrv, TxResultDescriptor_t *pTxResultInfo);
void TnetwDrv_TxXferDebug (TI_HANDLE hTnetwDrv, txCtrlBlkEntry_t *pPktCtrlBlk, UINT32 uDebugInfo);
static void TnetwDrv_ConfigureCb (TI_HANDLE hTnetwDrv);
static void TnetwDrv_ConfigureCb1 (TI_HANDLE hTnetwDrv);


/****************************************************************************/


/****************************************************************************/
/*                      TnetwDrv_Create()
 ****************************************************************************
 * DESCRIPTION: Create the Module
 *
 * INPUTS:  Handle to the OS
 *
 * OUTPUT:  
 *
 * RETURNS: Handle to the module on success or NULL on failure 
 ****************************************************************************/
TI_HANDLE TnetwDrv_Create (TI_HANDLE hOs)
{
    TnetwDrv_t *pTnetwDrv;

    /* Allocate the TNETW_Driver module */
    pTnetwDrv = (TnetwDrv_t *)os_memoryAlloc (hOs, sizeof(TnetwDrv_t));
    if (pTnetwDrv == NULL)
        return NULL;

    os_memoryZero (hOs, pTnetwDrv, sizeof(TnetwDrv_t));

    pTnetwDrv->hOs = hOs;

    /* Create the HAL Ctrl module */
    pTnetwDrv->hHalCtrl = whalCtrl_Create(hOs);
    if (pTnetwDrv->hHalCtrl == NULL)
    {
        WLAN_OS_REPORT(("TnetwDrv whalCtrl_Create failed!!!\n"));
        TnetwDrv_Destroy((TI_HANDLE)pTnetwDrv);
        return NULL;
    }

    pTnetwDrv->hTNETWIF = whalCtrl_GetTnetwifHandle (pTnetwDrv->hHalCtrl);

    /* Create the MAC Services module */
    pTnetwDrv->hMacServices = MacServices_create(hOs);
    if (pTnetwDrv->hMacServices == NULL)
    {
        WLAN_OS_REPORT(("TnetwDrv MacServices_create failed!!!\n"));
        TnetwDrv_Destroy((TI_HANDLE)pTnetwDrv);
        return NULL;
    }

    /* Create the Ctrl module */
    pTnetwDrv->hCmdQueue = CmdQueue_Create(hOs);
    if (pTnetwDrv->hCmdQueue == NULL)
    {
        WLAN_OS_REPORT(("TnetwDrv CmdQueue_Create failed!!!\n"));
        TnetwDrv_Destroy((TI_HANDLE)pTnetwDrv);
        return NULL;
    }

    /* 
     * Create the FW-Transfer modules:
     */

    pTnetwDrv->hTxXfer = txXfer_Create(hOs);
    if (pTnetwDrv->hTxXfer == NULL)
    {
        WLAN_OS_REPORT(("TnetwDrv txXfer_Create failed!!!\n"));
        TnetwDrv_Destroy((TI_HANDLE)pTnetwDrv);
        return NULL;
    }

    pTnetwDrv->hTxResult = txResult_Create(hOs);
    if (pTnetwDrv->hTxResult == NULL)
    {
        WLAN_OS_REPORT(("TnetwDrv txResult_Create failed!!!\n"));
        TnetwDrv_Destroy((TI_HANDLE)pTnetwDrv);
        return NULL;
    }

    pTnetwDrv->hRxXfer = rxXfer_Create(hOs);
    if (pTnetwDrv->hRxXfer == NULL)
    {
        WLAN_OS_REPORT(("TnetwDrv rxXfer_Create failed!!!\n"));
        TnetwDrv_Destroy((TI_HANDLE)pTnetwDrv);
        return NULL;
    }

    pTnetwDrv->hFwEvent = FwEvent_Create(hOs);
    if (pTnetwDrv->hFwEvent == NULL)
    {
        WLAN_OS_REPORT(("TnetwDrv FwEvent_Create failed!!!\n"));
        TnetwDrv_Destroy((TI_HANDLE)pTnetwDrv);
        return NULL;
    }

    pTnetwDrv->hEventMbox = eventMbox_Create(hOs);
    if (pTnetwDrv->hEventMbox == NULL)
    {
        WLAN_OS_REPORT(("TnetwDrv eventMbox_Create failed!!!\n"));
        TnetwDrv_Destroy((TI_HANDLE)pTnetwDrv);
        return NULL;
    }

#ifdef TI_DBG
    pTnetwDrv->hDebugTrace = debugTrace_Create(hOs);
    if (pTnetwDrv->hDebugTrace == NULL)
    {
        WLAN_OS_REPORT(("TnetwDrv debugTrace_Create failed!!!\n"));
        TnetwDrv_Destroy((TI_HANDLE)pTnetwDrv);
        return NULL;
    }
#endif /* TI_DBG */

    pTnetwDrv->hCmdMBox = CmdMBox_Create(hOs);
    if (pTnetwDrv->hCmdMBox == NULL)
    {
        WLAN_OS_REPORT(("TnetwDrv CmdMBox_Create failed!!!\n"));
        TnetwDrv_Destroy((TI_HANDLE)pTnetwDrv);
        return NULL;
    }

    /* 
     * Create the Data-Services modules:
     */

    pTnetwDrv->hTxCtrlBlk = txCtrlBlk_Create(hOs);
    if (pTnetwDrv->hTxCtrlBlk == NULL)
    {
        WLAN_OS_REPORT(("TnetwDrv txCtrlBlk_Create failed!!!\n"));
        TnetwDrv_Destroy((TI_HANDLE)pTnetwDrv);
        return NULL;
    }

    pTnetwDrv->hTxHwQueue = txHwQueue_Create(hOs);
    if (pTnetwDrv->hTxHwQueue == NULL)
    {
        WLAN_OS_REPORT(("TnetwDrv txHwQueue_Create failed!!!\n"));
        TnetwDrv_Destroy((TI_HANDLE)pTnetwDrv);
        return NULL;
    }

#if !defined(GWSI_DRIVER) && !defined(GWSI_LIB)
    pTnetwDrv->hRecoveryCtrl = recoveryCtrl_create(hOs);
#ifdef USE_RECOVERY
    if (pTnetwDrv->hRecoveryCtrl == NULL)
    {
        WLAN_OS_REPORT(("TnetwDrv recoveryCtrl_create failed!!!\n"));
        TnetwDrv_Destroy((TI_HANDLE)pTnetwDrv);
        return NULL;
    }
#endif /* USE_RECOVERY */
    pTnetwDrv->hHwInit = hwInit_create(hOs);
#ifdef USE_RECOVERY
    if (pTnetwDrv->hHwInit == NULL)
    {
        WLAN_OS_REPORT(("TnetwDrv hwInit_create failed!!!\n"));
        TnetwDrv_Destroy((TI_HANDLE)pTnetwDrv);
        return NULL;
    }
#endif /* USE_RECOVERY */
    pTnetwDrv->bRecoveryFlag = FALSE; /* init value is not Recovery */
#endif
    WLAN_INIT_REPORT (("TnetwDrv_Create: CREATED !!!\n"));

    return (TI_HANDLE)pTnetwDrv;
}



/****************************************************************************/
/*                      TnetwDrv_Destroy()
 ****************************************************************************
 * DESCRIPTION: Clear The module
 *
 * INPUTS:  Handle to the module
 *
 * OUTPUT:  
 *
 * RETURNS: 
 ****************************************************************************/
void TnetwDrv_Destroy (TI_HANDLE hTnetwDrv)
{

    TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;
    
    WLAN_INIT_REPORT(("TnetwDrv_Destroy !!!\n"));
    if (pTnetwDrv == NULL) 
    {
        return;
    }

    /* free the hal ctrl */
    if (pTnetwDrv->hHalCtrl != NULL)
    {
        whalCtrl_Destroy(pTnetwDrv->hHalCtrl);
        pTnetwDrv->hHalCtrl = NULL;
    }
    WLAN_INIT_REPORT(("TNETW_Driver_Destroy hHalCtrl released!!!\n"));
    
    /* free the MAC Services */
    if (pTnetwDrv->hMacServices != NULL)
    {
        MacServices_destroy(pTnetwDrv->hMacServices);
        pTnetwDrv->hMacServices = NULL;
    }
    WLAN_INIT_REPORT(("TNETW_Driver_Destroy hMacServices released!!!\n"));
    
    /*
     * Free the Ctrl modules
     */
    if (pTnetwDrv->hCmdQueue != NULL)
    {
        CmdQueue_Destroy(pTnetwDrv->hCmdQueue);
        pTnetwDrv->hCmdQueue = NULL;
    }

    /* 
     * Free the FW-Transfer modules:
     */

    if (pTnetwDrv->hTxXfer != NULL)
    {
        txXfer_Destroy(pTnetwDrv->hTxXfer);
        pTnetwDrv->hTxXfer = NULL;
    }

    if (pTnetwDrv->hTxResult != NULL)
    {
        txResult_Destroy(pTnetwDrv->hTxResult);
        pTnetwDrv->hTxResult = NULL;
    }

    if (pTnetwDrv->hRxXfer != NULL)
    {
        rxXfer_Destroy(pTnetwDrv->hRxXfer);
        pTnetwDrv->hRxXfer = NULL;
    }

    if (pTnetwDrv->hEventMbox != NULL) 
    {
        eventMbox_Destroy(pTnetwDrv->hEventMbox);
        pTnetwDrv->hEventMbox = NULL;
    }

#ifdef TI_DBG
    if (pTnetwDrv->hDebugTrace != NULL) 
    {
        debugTrace_Destroy(pTnetwDrv->hDebugTrace);
        pTnetwDrv->hDebugTrace = NULL;
    }
#endif /* TI_DBG */

    if (pTnetwDrv->hFwEvent != NULL)
    {
        FwEvent_Destroy(pTnetwDrv->hFwEvent);
        pTnetwDrv->hFwEvent = NULL;
    }

    if (pTnetwDrv->hCmdMBox != NULL)
    {
        CmdMBox_Destroy(pTnetwDrv->hCmdMBox);
        pTnetwDrv->hCmdMBox = NULL;
    }

    /* 
     * Free the Data-Services modules:
     */

    if (pTnetwDrv->hTxCtrlBlk != NULL)
    {
        txCtrlBlk_Destroy(pTnetwDrv->hTxCtrlBlk);
        pTnetwDrv->hTxCtrlBlk = NULL;
    }

    if (pTnetwDrv->hTxHwQueue != NULL)
    {
        txHwQueue_Destroy(pTnetwDrv->hTxHwQueue);
        pTnetwDrv->hTxHwQueue = NULL;
    }
    
#if !defined(GWSI_DRIVER) && !defined(GWSI_LIB)
    if (pTnetwDrv->hRecoveryCtrl != NULL)
    {
        recoveryCtrl_destroy(pTnetwDrv->hRecoveryCtrl);
        pTnetwDrv->hRecoveryCtrl = NULL;
    }

    if (pTnetwDrv->hHwInit != NULL)
    {
        hwInit_destroy(pTnetwDrv->hHwInit);
        pTnetwDrv->hHwInit = NULL;
    }
#endif
    
    /* free the TNETW driver */
    if ( NULL != pTnetwDrv->pInitTableCopy )
    {
        os_memoryFree( pTnetwDrv->hOs, pTnetwDrv->pInitTableCopy, sizeof(TnetwDrv_InitParams_t) );
        pTnetwDrv->pInitTableCopy = NULL;
    }

    os_memoryFree(pTnetwDrv->hOs, (TI_HANDLE)pTnetwDrv, sizeof(TnetwDrv_t));
    pTnetwDrv = NULL;
    
    WLAN_INIT_REPORT(("TNETW_Driver_Destroy pTNETW_Driver released!!!\n"));
    
    return;
}


/****************************************************************************/
/*                      TnetwDrv_Init()
 ****************************************************************************
 * DESCRIPTION: TNETW Driver Init
 *
 * INPUTS:  
 *
 * OUTPUT:  
 *
 * RETURNS: 
 ****************************************************************************/
TI_STATUS TnetwDrv_Init (TI_HANDLE hTnetwDrv, TI_HANDLE hReport, TI_HANDLE hMemMgr, TI_HANDLE hUser, UINT32 *pFWImage, TnetwDrv_InitParams_t* pInitParams, TnetDrv_callback_t fUserConf)
{ 
    TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;
    whalCtrl_config_t       whalCtrl_config;
    TI_STATUS               status;

    pTnetwDrv->bInitSuccess = FALSE;
    pTnetwDrv->hUser = hUser;
    pTnetwDrv->fUserConf = fUserConf;
    pTnetwDrv->hReport = hReport;
    pTnetwDrv->hMemMgr = hMemMgr;
    pTnetwDrv->hWhalParams = whalCtrl_GetWhalParams (pTnetwDrv->hHalCtrl);
    
    WLAN_REPORT_INIT(pTnetwDrv->hReport, HAL_CTRL_MODULE_LOG,
        ("TnetwDrv_Init %x\n",hReport));

    if (NULL != pInitParams)
    {       
        if (OK != TnetwDrv_SetInitParams (hTnetwDrv, (TnetwDrv_InitParams_t *)pInitParams))
        {
            TnetwDrv_Destroy (hTnetwDrv);
            return NOK;
        }
    }
    
    /* 
     * Configure the HAL Ctrl 
     */
    whalCtrl_config.hMemMgr             = hMemMgr;
    whalCtrl_config.hReport             = hReport;
    whalCtrl_config.hFwEvent            = pTnetwDrv->hFwEvent;
    whalCtrl_config.hRxXfer             = pTnetwDrv->hRxXfer;
    whalCtrl_config.hTxXfer             = pTnetwDrv->hTxXfer;
    whalCtrl_config.hTxHwQueue          = pTnetwDrv->hTxHwQueue;
    whalCtrl_config.hTxResult           = pTnetwDrv->hTxResult;
    whalCtrl_config.hEventMbox          = pTnetwDrv->hEventMbox;

        /* CB at the end of TnetwDrv_Configure(). not called if no registration was done */
        pTnetwDrv->fConfigureCmplteteCB     = NULL;
        pTnetwDrv->hConfigureCompleteOBj    = NULL;
    pTnetwDrv->fConfigureEndCB = TnetwDrv_ConfigureCb1;
    pTnetwDrv->fConfigureEndObj= hTnetwDrv;
    
    whalCtrl_config.hCmdQueue           = pTnetwDrv->hCmdQueue;
#ifdef TI_DBG
    whalCtrl_config.hDebugTrace         = pTnetwDrv->hDebugTrace;
#endif /* TI_DBG */

    /* Call the config func */
    if ((status = whalCtrl_Config (pTnetwDrv->hHalCtrl, hTnetwDrv, &whalCtrl_config, pFWImage)) == TNETWIF_ERROR)
    {
        WLAN_OS_REPORT(("TNETW_Driver whalCtrl_Config failed!!!\n"));
        TnetwDrv_Destroy (hTnetwDrv);
        return NOK;
    }

    return status;
}


/****************************************************************************
 * DESCRIPTION: Configure the TNET Driver Module callback
 *
 * INPUTS:  
 *
 * OUTPUT:  
 *
 * RETURNS: OK if succeeded, NOK if failed in HW configuration. 
 ****************************************************************************/
void TnetwDrv_ConfigureCb (TI_HANDLE hTnetwDrv)
{
    TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;

    /* Configure the Tx-HW-Queue module */
    txHwQueue_Config (pTnetwDrv->hTxHwQueue, pTnetwDrv->pInitTableCopy);
    
    /* Configure the TX XFER module */
    txXfer_config(pTnetwDrv->hTxXfer, pTnetwDrv->pInitTableCopy);
    
    /* Configure the MAC services */ 
    MacServices_config (pTnetwDrv->hMacServices, pTnetwDrv->pInitTableCopy);
    
#if !defined(GWSI_DRIVER) && !defined(GWSI_LIB)
    recoveryCtrl_config(pTnetwDrv->hRecoveryCtrl, 
                        pTnetwDrv->hReport,
                        pTnetwDrv->hTNETWIF,
                        pTnetwDrv->hTxXfer,     
                        pTnetwDrv->hRxXfer,     
                        pTnetwDrv->hTxResult,   
                        pTnetwDrv->hMacServices,
                        pTnetwDrv->hTxCtrlBlk,  
                        pTnetwDrv->hTxHwQueue,  
                        pTnetwDrv->hHalCtrl,    
                        pTnetwDrv->hHwIntr,     
                        pTnetwDrv->hWhalParams, 
                        pTnetwDrv->hCmdQueue,   
                        pTnetwDrv->hFwEvent,    
                        pTnetwDrv->hCmdMBox,
                        pTnetwDrv->hHwInit);

    hwInit_config(pTnetwDrv->hHwInit, pTnetwDrv->hReport, pTnetwDrv->hTNETWIF);
#endif
    /* Register the Data Path callback functions */
    TnetwDrv_Register_CB (pTnetwDrv, TNETW_DRIVER_TX_XFER_SEND_PKT_TRANSFER, (void *)TnetwDrv_TxXferDone, hTnetwDrv);

    /* Register the send packet debug callback */
  #ifdef TI_DBG
    TnetwDrv_Register_CB (pTnetwDrv, TNETW_DRIVER_TX_XFER_SEND_PKT_DEBUG, (void *)TnetwDrv_TxXferDebug, hTnetwDrv);
  #endif

    /* Register the send packet complete callback */
    TnetwDrv_Register_CB (pTnetwDrv, TNETW_DRIVER_TX_RESULT_SEND_PKT_COMPLETE, (void *)TnetwDrv_TxComplete, hTnetwDrv);

    /* Call user application configuration callback */
    if (pTnetwDrv->fUserConf)
        (*pTnetwDrv->fUserConf) (pTnetwDrv->hUser);

}

/****************************************************************************
 * DESCRIPTION: Configure the TNET Driver Module callback
 *
 * INPUTS:  
 *
 * OUTPUT:  
 *
 * RETURNS: OK if succeeded, NOK if failed in HW configuration. 
 ****************************************************************************/
void TnetwDrv_ConfigureCb1 (TI_HANDLE hTnetwDrv)
{
    TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;

        /* if the configure complete function was registered, we call it here - end of TnetwDrv_Configure stage */
    if (pTnetwDrv->fConfigureCmplteteCB) 
    {
        pTnetwDrv->fConfigureCmplteteCB(pTnetwDrv->hConfigureCompleteOBj);
    }   

#ifndef GWSI_LIB
    /* 
        * This will be the last thing that will be done here so all the download 
        * will go back down to HALto send FINISH to TNETWIF where it began 
        */
        os_Complete (pTnetwDrv->hOs);    
#endif

}



/****************************************************************************
 * DESCRIPTION: Configure the TNET Driver Module
 *
 * INPUTS:  
 *
 * OUTPUT:  
 *
 * RETURNS: OK if succeeded, NOK if failed in HW configuration. 
 ****************************************************************************/
TI_STATUS TnetwDrv_Configure (TI_HANDLE hTnetwDrv, TnetwDrv_InitParams_t* pInitParams)
{
    TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;
    TnetwDrv_InitParams_t* pInitParam;
    if (pTnetwDrv->bInitSuccess)
    {
        /* If called with init params as null - it means that this is a recovery */    
        if (NULL != pInitParams)
        {       
            if (OK != TnetwDrv_SetInitParams (hTnetwDrv, pInitParams))
            {
                return NOK;
            }
        }

#if !defined(GWSI_DRIVER) && !defined(GWSI_LIB)
        if(pTnetwDrv->bRecoveryFlag == TRUE)
            pInitParam = NULL;
        else
#endif
            pInitParam = pTnetwDrv->pInitTableCopy;


        /* If it's recovery call function with NULL instead of ini-file params */
        if (whalCtrl_ConfigHw (pTnetwDrv->hHalCtrl, 
                               pInitParam, 
                               (void *)TnetwDrv_ConfigureCb,
                               hTnetwDrv) != OK)
        {
            WLAN_REPORT_ERROR(pTnetwDrv->hReport, TNETW_DRV_MODULE_LOG,("\n.....WhalCtrl configuration failure \n"));
            return NOK;
        }

        return OK;
    }

    return NOK;
}


/****************************************************************************
 *                      TnetwDrv_FinalizeDownload()
 ****************************************************************************
 * DESCRIPTION: Finalize all the remaining initialization after the downloaD HAS FINISHED 
                Register the ERRORS indications events to the FW
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
TI_STATUS TnetwDrv_FinalizeDownload (TI_HANDLE hTnetwDrv)
{
    TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;

    /* Here at the end call the Initialize Complete callback that will release the user Init semaphore */
    WLAN_REPORT_INIT(pTnetwDrv->hReport, HAL_CTRL_MODULE_LOG,
                ("hTNETW_Driver %x!!!!!\n", hTnetwDrv));

    /* Here at the end call the Initialize Complete callback that will release the user Init semaphore */
    WLAN_REPORT_INIT(pTnetwDrv->hReport, HAL_CTRL_MODULE_LOG,
        ("Call MacServices_init!!!!!\n"));

    /*
     * Initialize the FW-Transfer modules
     */
    txXfer_init (pTnetwDrv->hTxXfer, pTnetwDrv->hReport, pTnetwDrv->hTNETWIF, pTnetwDrv->hTxResult);
    txResult_init (pTnetwDrv->hTxResult, pTnetwDrv->hReport, pTnetwDrv->hTNETWIF, pTnetwDrv->hFwEvent);
    rxXfer_Config (pTnetwDrv->hRxXfer, pTnetwDrv->hFwEvent, pTnetwDrv->hMemMgr, pTnetwDrv->hReport,pTnetwDrv->hTNETWIF);
        
#ifdef TI_DBG
    debugTrace_Config (pTnetwDrv->hDebugTrace,
                       pTnetwDrv->hWhalParams,
                       pTnetwDrv->hReport,
                       pTnetwDrv->hMemMgr,
                       pTnetwDrv->hTNETWIF,
                       pTnetwDrv->hFwEvent);
#endif /* TI_DBG */
    
    /* 
     * Initialize the MAC Services 
     */
    MacServices_init (pTnetwDrv->hMacServices,
                      pTnetwDrv->hReport,
                      pTnetwDrv->hHalCtrl);

    /*
     * Initialize the Data-Services modules
     */
    txCtrlBlk_init (pTnetwDrv->hTxCtrlBlk, pTnetwDrv->hReport);
    txHwQueue_init (pTnetwDrv->hTxHwQueue, pTnetwDrv->hReport, pTnetwDrv->hWhalParams);

    /* Here at the end call the Initialize Complete callback that will release the user Init semaphore */
    WLAN_REPORT_INIT(pTnetwDrv->hReport, HAL_CTRL_MODULE_LOG,
        ("Before sending the Init Complet callback !!!!!\n"));

    /* Sign that init has succeeded */
    pTnetwDrv->bInitSuccess = TRUE;    

    /* When working with GWSI Call the Init Complete callback */
#ifdef GWSI_LIB
    /*
     * The callback function does not need the handle of the GWSI 
     * since it takes it from the global handle 
     */
    GWSI_FinalizeDownload (pTnetwDrv->hUser, OK);
    /* 
     * When working with CORE call the os_Init_Complete 
     * that will release the OS semaphore that the
     * user is lock on it in the esta_drb to go on call the next stage 
     */
#else 
    /* Here at the end call the Initialize Complete callback that will release the user Init semaphore */
    WLAN_REPORT_INIT(pTnetwDrv->hReport, HAL_CTRL_MODULE_LOG,
        ("Call os_Complete !!!!!\n"));
    
    /* Start configuring driver */
    if (TnetwDrv_Configure (hTnetwDrv, NULL) != OK)
    {
        WLAN_REPORT_ERROR (pTnetwDrv->hReport, TNETW_DRV_MODULE_LOG,
                           ("TnetwDrv_FinalizeDownload: configuration failure!\n"));
    }

#endif

    return TNETWIF_COMPLETE;
}


/****************************************************************************
 *                      TnetwDrv_FinalizeOnFailue()
 ****************************************************************************
 * DESCRIPTION: Finalize all the initialization upon failure 
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
TI_STATUS TnetwDrv_FinalizeOnFailure (TI_HANDLE hTNETW_Driver)
{
    TnetwDrv_t  *pTnetwDrv = (TnetwDrv_t *)hTNETW_Driver;

#ifdef GWSI_LIB

    /* Stop Init phase of GWSI and return TNETWIF_ERROR */
    GWSI_FinalizeDownload (pTnetwDrv->hUser, TNETWIF_ERROR);

    /* When working with CORE call the os_Init_Complete that will release the OS semaphore taht the
    user is lock on it in the esta_drb to go on call the next stage */

#else 

    /* Here at the end call the Initialize Complete callback that will release the user Init semaphore */
    WLAN_REPORT_INIT (pTnetwDrv->hReport, HAL_CTRL_MODULE_LOG, ("Call os_Complete !!!!!\n"));           

    /* 
     * This will be the last thing that will be done here so all the download 
     * will go back down to HAL to send FINISH to TNETWIF where it began 
     */
    os_Complete (pTnetwDrv->hOs);    

#endif

    return TNETWIF_COMPLETE;
}


/****************************************************************************/
/*                      TNETW_Driver_Register_CB()
 ****************************************************************************
 * DESCRIPTION: Register the MAC Services and the HAL modules callbacks
 *
 * INPUTS:  
 *
 * OUTPUT:
 *
 * RETURNS: 
 ****************************************************************************/
void TnetwDrv_Register_CB (TI_HANDLE hTnetwDrv,tiUINT32 EventID,void *CBFunc, void *pData)
{

    TnetwDrv_t * pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;

    WLAN_REPORT_INFORMATION(pTnetwDrv->hReport, TNETW_DRV_MODULE_LOG, ("TnetwDrv_Register_CB (Value = 0x%x)\n", EventID));

    /* First detect which module is the owner */
    switch((tiUINT16)(EventID & TNETW_DRIVER_CB_MODULE_OWNER_MASK))
    {

    case  TNETW_DRIVER_TX_XFER_OWNER:
        EventID &= TNETW_DRIVER_CB_TYPE_MASK;
        WLAN_REPORT_INFORMATION(pTnetwDrv->hReport, TNETW_DRV_MODULE_LOG,("TnetwDrv_Register_CB: TNETW_DRIVER_TX_XFER_OWNER\n"));
        txXfer_RegisterCB(pTnetwDrv->hTxXfer, EventID, CBFunc, pData);
        break;

    case  TNETW_DRIVER_TX_RESULT_OWNER:
        EventID &= TNETW_DRIVER_CB_TYPE_MASK;
        WLAN_REPORT_INFORMATION(pTnetwDrv->hReport, TNETW_DRV_MODULE_LOG,("TnetwDrv_Register_CB: TNETW_DRIVER_TX_RESULT_OWNER\n"));
        txResult_RegisterCB(pTnetwDrv->hTxResult, EventID, CBFunc, pData);
        break;

    case TNETW_DRIVER_RX_XFER_OWNER:
        EventID &= TNETW_DRIVER_CB_TYPE_MASK;
        WLAN_REPORT_INFORMATION(pTnetwDrv->hReport, TNETW_DRV_MODULE_LOG,("TnetwDrv_Register_CB: TNETW_DRIVER_RX_XFER_OWNER\n"));
        rxXfer_Register_CB(pTnetwDrv->hRxXfer, EventID,CBFunc,pData);
        break;

    case TNETW_DRIVER_HAL_CTRL_OWNER:
        EventID &= TNETW_DRIVER_CB_TYPE_MASK;
        whalCtrl_Register_CB(pTnetwDrv->hHalCtrl, EventID,CBFunc,pData);
        break;
 
    case TNETW_DRIVER_MAC_SERVICES_OWNER:
        switch (EventID & TNETW_DRIVER_CB_TYPE_MASK)
        {
        case HAL_EVENT_SCAN_CMPLT:
            MacServices_scanSRV_registerScanCompleteCB(pTnetwDrv->hMacServices, (scan_srvCompleteCB_t)CBFunc, pData);
            break;
        default:
            WLAN_REPORT_WARNING(pTnetwDrv->hReport, TNETW_DRV_MODULE_LOG,("TNETW_DRIVER_MAC_SERVICES_OWNER - Illegal value\n"));        
        }
        break;

    case TNETW_DRIVER_TWD_OWNER:    
        WLAN_REPORT_INFORMATION(pTnetwDrv->hReport, TNETW_DRV_MODULE_LOG,("TnetwDrv_Register_CB: TNETW_DRIVER_TWD_OWNER\n"));
        pTnetwDrv->fConfigureCmplteteCB  = (TnetDrv_callback_t)CBFunc;
        pTnetwDrv->hConfigureCompleteOBj = (TI_HANDLE)pData;    
        break;

    default:
        if (EventID == HAL_INTERNAL_EVENT_FAILURE)
        {


            /* register the Hal failure event callback  including the RX
                 in the Hal Cttl the errors are :   
                MBOX_FAILURE,
                    BUS_ERROR,
                DEVICE_ERROR,
                DISCONNECT_TIMEOUT,*/
                
                
            
            EventID &= TNETW_DRIVER_CB_TYPE_MASK;
            whalCtrl_Register_CB(pTnetwDrv->hHalCtrl, EventID,CBFunc,pData);

            /* register the Elp controller failure event callback to the TNET interface
                HW_AWAKE_FAILURE*/
            TNETWIF_RegisterFailureEventCB(pTnetwDrv->hTNETWIF,CBFunc,pData);
            
            /* register the Mac services failure events callbacks 
                POWER_SAVE_FAILURE,
                MEASUREMENT_FAILURE,
                NO_SCAN_COMPLETE_FAILURE,*/
            MacServices_registerFailureEventCB(pTnetwDrv->hMacServices, CBFunc, pData);


            /* register the TX failure call back in the  Xfer
                TX_STUCK,*/
            txXfer_RegisterFailureEventCB(pTnetwDrv->hTxXfer, CBFunc, pData);
            
            
        }
        else
        WLAN_REPORT_WARNING(pTnetwDrv->hReport, TNETW_DRV_MODULE_LOG,("TnetwDrv_Register_CB - Illegal value\n"));       

    }
    return;
}

/****************************************************************************/
/*                      TnetwDrv_SetInitParams()
 ****************************************************************************/
static TI_STATUS TnetwDrv_SetInitParams (TI_HANDLE hTnetwDrv, TnetwDrv_InitParams_t* pInitParams)
{
    TnetwDrv_t * pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;        
    TnetwDrv_InitParams_t *pInitTableCopy;
    UINT32       index;
    
    pInitTableCopy = pTnetwDrv->pInitTableCopy = os_memoryAlloc (pTnetwDrv->hOs, sizeof(TnetwDrv_InitParams_t));
    if (pTnetwDrv->pInitTableCopy != NULL)
    {
        os_memoryZero (pTnetwDrv->hOs, pTnetwDrv->pInitTableCopy, sizeof(TnetwDrv_InitParams_t));
        /* Copy the init info to the buffer */
        os_memoryCopy (pTnetwDrv->hOs, pTnetwDrv->pInitTableCopy, pInitParams, sizeof(TnetwDrv_InitParams_t));
        /* Change the Severity table to character */ 
        for (index = 0; index < sizeof(((report_t *)pTnetwDrv->hReport)->SeverityTable); index++)
        {
            pInitTableCopy->reportParams.SeverityTable[index] += '0';
        }
        /* Change the module table to character */ 
        for (index = 0; index < sizeof(((report_t *)pTnetwDrv->hReport)->ModuleTable); index++)
        {
            pInitTableCopy->reportParams.ModuleTable[index] += '0';
        }
    }   
    else
    {
        WLAN_REPORT_ERROR(pTnetwDrv->hReport, TNETW_DRV_MODULE_LOG,
                          ("TnetwDrv_SetInitParams: unable to allocate init params buffer!\n") );
        return NOK;
    }

    return OK;
}

/****************************************************************************/
/*                      TnetwDrv_GetInitParams()
 ****************************************************************************/
void TnetwDrv_GetInitParams (TI_HANDLE hTnetwDrv, UINT8 *pcommand, UINT16 *OutBufLen)
{   
    TnetwDrv_t * pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;        

    if (pTnetwDrv->pInitTableCopy)
    {
        *(UINT32 *)pcommand = sizeof(TnetwDrv_InitParams_t);
        pcommand += sizeof(UINT32);
        os_memoryCopy(NULL, (void *)pcommand, (void *)pTnetwDrv->pInitTableCopy, sizeof(TnetwDrv_InitParams_t));
    }
    else
    {
        /* The table information is not available */
        *(UINT32 *)pcommand = 0;
        WLAN_OS_REPORT(("TNETW_Driver_GetInitParams :ERROR Getting Buffer for the INI File !!!\n"));
    }              

    *OutBufLen = (sizeof(TnetwDrv_InitParams_t) + sizeof(UINT32));
}



/****************************************************************************/
/*                      TnetwDrv_PrintInfo()
 ****************************************************************************
 * DESCRIPTION:  Call the requested print function.
 ****************************************************************************/
void  TnetwDrv_PrintInfo (TI_HANDLE hTnetwDrv, TnetwDrv_PrintInfoType_e printInfo)
{
    TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;

    switch(printInfo) 
    {
        case TNETW_DRV_PRINT_TX_CTRL_BLK_TBL:
            txCtrlBlk_printTable(pTnetwDrv->hTxCtrlBlk);
            break;

        case TNETW_DRV_PRINT_TX_HW_QUEUE_INFO:
            txHwQueue_printInfo(pTnetwDrv->hTxHwQueue);
            break;

        case TNETW_DRV_PRINT_TX_XFER_INFO:
            txXfer_printInfo(pTnetwDrv->hTxXfer);
            break;

        case TNETW_DRV_PRINT_TX_RESULT_INFO:
            txResult_printInfo(pTnetwDrv->hTxResult);
            break;

        case TNETW_DRV_CLEAR_TX_RESULT_INFO:
            txResult_clearInfo(pTnetwDrv->hTxResult);
            break;

        default:
            WLAN_REPORT_ERROR( pTnetwDrv->hReport, TNETW_DRV_MODULE_LOG,
                               ("$s: invalid print info request code: %d\n", __FUNCTION__, printInfo) );
    }
}




/****************************************************************************/
/*                      TnetwDrv_TEMP_GetHandles()
 ****************************************************************************
 * DESCRIPTION:  
 
       TEMPORARY!! - untill the new TNETW-Driver architecture is completed!!

       In the new architecture all external calls to the driver will be through  
         the hTnetwDrv handle.

       Called by the driver creation process.
       Gets the TNETW-Driver modules handles needed externally.
 
****************************************************************************/
void  TnetwDrv_TEMP_GetHandles(TI_HANDLE hTnetwDrv, TI_HANDLE *pHalCtrl, TI_HANDLE *pMacServices)
{
    TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;

    *pHalCtrl       = pTnetwDrv->hHalCtrl;
    *pMacServices   = pTnetwDrv->hMacServices;
}

#if !defined(GWSI_DRIVER) && !defined(GWSI_LIB)
/****************************************************************************/
/*                      TnetwDrv_StartRecovery()
 ****************************************************************************
 * DESCRIPTION: 
          API function called by RecoverMgr to start TWD recovery process 
****************************************************************************/
void TnetwDrv_StartRecovery(TI_HANDLE hTnetwDrv, void *endOfRecoveryCB, TI_HANDLE hRecoveryMgr)
{
    TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;
    pTnetwDrv->bRecoveryFlag = TRUE; 
    recoveryCtrl_restartTWD(pTnetwDrv->hRecoveryCtrl, endOfRecoveryCB, hRecoveryMgr);

}

/****************************************************************************/
/*                      TnetwDrv_InitHw_FinalizeDownload()
 ****************************************************************************
 * DESCRIPTION: 
          API function called by RecoverMgr to start TWD recovery process 
****************************************************************************/
TI_STATUS TnetwDrv_InitHw_FinalizeDownload(TI_HANDLE hTnetwDrv)
{
    TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)hTnetwDrv;

    if (pTnetwDrv->bRecoveryFlag)
    {
        pTnetwDrv->bRecoveryFlag = FALSE;
        return InitHw_FinalizeDownload(pTnetwDrv->hHwInit);
    }
    else
    {
        return TnetwDrv_FinalizeDownload(hTnetwDrv);
    }
}
#endif

#ifdef GWSI_SPI_TEST

TI_HANDLE TnetwDrv_GetTnetwifHandle (TI_HANDLE hTnetwDrv)
{
	return ((TnetwDrv_t *)hTnetwDrv)->hTNETWIF;
}

#endif /* GWSI_SPI_TEST */
