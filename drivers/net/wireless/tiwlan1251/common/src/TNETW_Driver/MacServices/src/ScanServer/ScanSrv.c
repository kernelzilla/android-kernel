/** \file ScanSrv.c
 *  \brief This file include the scan SRV module implementation
 *  \date 30-Dec-2004
 */
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

#include "report.h"
#include "ScanSrv.h"
#include "ScanSrvSM.h"
#include "utils.h"
#include "MacServices.h"
#include "MacServices_api.h"
#include "whalCtrl_api.h"
#include "public_event_mbox.h"

/**
 * \author Yuval Adler\n
 * \date 16-Oct-2004\n
 * \brief Creates the scan SRV object
 *
 * Function Scope \e Public.\n
 * \param hOS - handle to the OS object.\n
 * \return a handle to the scan SRV object, NULL if an error occurred.\n
 */
TI_HANDLE MacServices_scanSRV_create( TI_HANDLE hOS )
{
    /* allocate the scan SRV object */
    scanSRV_t *pScanSRV = os_memoryAlloc( hOS, sizeof(scanSRV_t) );
    if ( NULL == pScanSRV )
    {
        WLAN_OS_REPORT( ("ERROR: Failed to create scan SRV module") );
        return NULL;
    }

    /* allocate the state machine */
    if ( OK != fsm_Create( hOS, &(pScanSRV->SM), SCAN_SRV_NUM_OF_STATES, SCAN_SRV_NUM_OF_EVENTS ) )
    {
        WLAN_OS_REPORT( ("ERROR: Failed to allocate scan SRV state machine") );
        os_memoryFree( hOS, pScanSRV, sizeof(scanSRV_t) );
        return NULL;
    }
 
    /* create the timer */
    pScanSRV->timer = os_timerCreate( hOS, MacServices_scanSRV_scanTimerExpired, pScanSRV );
    if ( NULL == pScanSRV->timer )
    {
        WLAN_OS_REPORT( ("ERROR: Failed to create timer for scan SRV module") );
        fsm_Unload( hOS, pScanSRV->SM );
        os_memoryFree( hOS, pScanSRV, sizeof(scanSRV_t) );
        return NULL;
    }

    /* store the OS handle */
    pScanSRV->hOS = hOS;

    return pScanSRV;
}

/**
 * \author Yuval Adler\n
 * \date 29-Dec-2004\n
 * \brief Finalizes the scan SRV module (releasing memory and timer)
 *
 * Function Scope \e Public.\n
 * \param hScanSRV - handle to the scan SRV object.\n
 */
void MacServices_scanSRV_destroy( TI_HANDLE hScanSRV )
{
    scanSRV_t *pScanSRV = (scanSRV_t*)hScanSRV;

    /* free timer */
    os_timerStop( pScanSRV->hOS, pScanSRV->timer );
    os_timerDestroy( pScanSRV->hOS, pScanSRV->timer );
    
    /* free memory */
    fsm_Unload( pScanSRV->hOS, pScanSRV->SM );
    os_memoryFree( pScanSRV->hOS, (TI_HANDLE)pScanSRV , sizeof(scanSRV_t) );
}

/**
 * \author Yuval Adler\n
 * \date 29-Dec-2004\n
 * \brief Initializes the scan SRV module, registers SCAN_COMPLETE to HAL.
 *
 * Function Scope \e Public.\n
 * \param hScanSRV - handle to the scan SRV object.\n
 * \param hReport - handle to the report object.\n
 * \param hHalCtrl - handle to the HAL ctrl object.\n
  */
void MacServices_scanSRV_init( TI_HANDLE hMacServices, TI_HANDLE hReport,  TI_HANDLE hHalCtrl)
{
    MacServices_t* pMacServices =  (MacServices_t*)hMacServices;
    scanSRV_t *pScanSRV =  pMacServices->hScanSRV;

    /* store handles */
    pScanSRV->hHalCtrl = hHalCtrl;
    pScanSRV->hReport = hReport;
    pScanSRV->hPowerSrv = pMacServices->hPowerSrv;
    pScanSRV->commandResponseFunc = NULL;
    pScanSRV->commandResponseObj = NULL;  

    /* init state machine */
    scanSRVSM_init( (TI_HANDLE) pScanSRV );

    /* register our scan complete handler to the HAL events mailbox */
    whalCtrl_EventMbox_RegisterForEvent( pScanSRV->hHalCtrl, HAL_EVENT_SCAN_CMPLT, 
                                         (void *)MacServices_scanSRV_scanCompleteCB, (TI_HANDLE)pScanSRV ); 
    whalCtrl_EventMbox_RegisterForEvent( pScanSRV->hHalCtrl, HAL_EVENT_SPS_SCAN_CMPLT, 
                                         (void *)MacServices_scanSRV_scanCompleteCB, (TI_HANDLE)pScanSRV );

    /* init other stuff */
    pScanSRV->currentNumberOfConsecutiveNoScanCompleteEvents = 0;

    WLAN_REPORT_INIT( hReport, SCAN_SRV_MODULE_LOG,  (".....Scan SRV configured successfully.\n"));
}

/**
 * \brief Restart the scan SRV module upon recovery.
 *
 * Function Scope \e Public.\n
 * \param hScanSRV - handle to the scan SRV object.\n
 */
void scanSRV_restart( TI_HANDLE hScanSRV)
{
    scanSRV_t *pScanSRV =  (scanSRV_t *)hScanSRV;
    /* init state machine */
    /* initialize current state */
    pScanSRV->SMState = SCAN_SRV_STATE_IDLE;

    if ( TRUE == pScanSRV->bTimerRunning )
    {
        os_timerStop( pScanSRV->hOS, pScanSRV->timer );
        pScanSRV->bTimerRunning = FALSE;
    }
}

/**
 * \author Ronen Kalish\n
 * \date 26-July-2006\n
 * \brief Configures the scan SRV module with initialization values
 *
 * Function Scope \e Public.\n
 * \param hScanSRV - handle to the scan SRV object.\n
 * \param hReport - handle to the report object.\n
 * \param hHalCtrl - handle to the HAL ctrl object.\n
  */
void MacServices_scanSrv_config( TI_HANDLE hMacServices, scanSrvInitParams_t* pInitParams )
{
    MacServices_t* pMacServices =  (MacServices_t*)hMacServices;
    scanSRV_t *pScanSRV =  pMacServices->hScanSRV;

    pScanSRV->numberOfNoScanCompleteToRecovery = pInitParams->numberOfNoScanCompleteToRecovery;

    /* Set Triggered scan time out per channel */
    pScanSRV->uTriggeredScanTimeOut = pInitParams->uTriggeredScanTimeOut;
    whalCtrl_SetSplitScanTimeOut (pScanSRV->hHalCtrl, pScanSRV->uTriggeredScanTimeOut);
}

/**
 * \author Ronen Kalish\n
 * \date 29-Dec-2004\n
 * \brief Registers a complete callback for scan complete notifications.
 *
 * Function Scope \e Public.\n
 * \param hMacServices - handle to the MacServices object.\n
 * \param scanCompleteCB - the complete callback function.\n
 * \param hScanCompleteObj - handle to the object passed to the scan complete callback function.\n
 */
void MacServices_scanSRV_registerScanCompleteCB( TI_HANDLE hMacServices, 
                                     scan_srvCompleteCB_t scanCompleteCB, TI_HANDLE hScanCompleteObj )
{
    scanSRV_t *pScanSRV = (scanSRV_t*)((MacServices_t*)hMacServices)->hScanSRV;

    pScanSRV->scanCompleteNotificationFunc = scanCompleteCB;
    pScanSRV->scanCompleteNotificationObj = hScanCompleteObj;
}

/**
 * \brief Registers a failure event callback for scan error notifications.
 *
 * Function Scope \e member.\n
 * \param hScanSRV - handle to the Scan SRV object.\n
 * \param failureEventCB - the failure event callback function.\n
 * \param hFailureEventObj - handle to the object passed to the failure event callback function.\n
 */
void scanSRV_registerFailureEventCB( TI_HANDLE hScanSRV, 
                                     void * failureEventCB, TI_HANDLE hFailureEventObj )
{
    scanSRV_t *pScanSRV = (scanSRV_t*)(hScanSRV);

    pScanSRV->failureEventFunc  = (failureEventCB_t)failureEventCB;
    pScanSRV->failureEventObj   = hFailureEventObj;
}

/**
 * \author Yuval Adler\n
 * \date 27-Sep-2005\n
 * \brief This function is the CB which is called as response to 'StartScan' or 'StopScan' \n.
 *        here we check if there is a GWSI command response , and call it if necessary .\n
 * Function Scope \e Private.\n
 * \param hScanSrv - handle to the scan SRV object.\n
 * \param MboxStatus - mailbox status. \n
 */
void MacServices_scanSRVCommandMailBoxCB(TI_HANDLE hScanSrv,UINT16 MboxStatus)
{
    scanSRV_t* pScanSRV = (scanSRV_t*)hScanSrv;
    UINT16 responseStatus;
    CmdResponseCB_t CB_Func;
    TI_HANDLE  CB_Handle;

    WLAN_REPORT_INFORMATION( pScanSRV->hReport, SCAN_SRV_MODULE_LOG, ("%s status %u\n",__FUNCTION__,MboxStatus) );

    /* set response to OK or NOK */
    responseStatus = ((MboxStatus > 0) ? NOK : OK);
    
    /* if we have a Response Function (only in GWSI) we set it back to NULL and then 
        we call it */
    if (pScanSRV->commandResponseFunc != NULL)
    {
        CB_Func = pScanSRV->commandResponseFunc;
        CB_Handle = pScanSRV->commandResponseObj;

        pScanSRV->commandResponseFunc = NULL;
        pScanSRV->commandResponseObj = NULL;

        CB_Func(CB_Handle, responseStatus);
    }
     /* if scan request failed */
    if ( OK != responseStatus )
    {
        WLAN_REPORT_ERROR( pScanSRV->hReport, SCAN_SRV_MODULE_LOG,
                           ("Mail box returned error , quitting scan.\n" ));

        /* send a scan complete event. This will do all necessary clean-up (timer, power manager, notifying scan complete) */
        scanSRVSM_SMEvent( hScanSrv, (scan_SRVSMStates_e*)&pScanSRV->SMState, SCAN_SRV_EVENT_SCAN_COMPLETE );
    }
}

/**
 * \author Ronen Kalish\n
 * \date 29-Dec-2004\n
 * \brief Performs a scan
 *
 * Function Scope \e Public.\n
 * \param hMacServices - handle to the MacServices object.\n
 * \param scanParams - the scan specific parameters.\n
 * \param bHighPriority - whether to perform a high priority (overlaps DTIM) scan.\n
 * \param bDriverMode - whether to try to enter driver mode (with PS on) before issuing the scan command.\n
 * \param bScanOnDriverModeError - whether to proceed with the scan if requested to enter driver mode and failed.\n
 * \param psRequest - Parameter sent to PowerSaveServer on PS request to indicate PS on or "keep current" 
 * \param bSendNullData - whether to send Null data when exiting driver mode on scan complete.\n
 * \param commandResponseFunc - CB function which called after downloading the command. \n
 * \param commandResponseObj -  The CB function Obj (Notice : last 2 params are NULL in Legacy run). \n
 * \param psRequest - Parameter sent to PowerSaveServer on PS request to indicate PS on or "keep current" 
 * \return OK if successful (various, TBD codes if not).\n
 */
TI_STATUS MacServices_scanSRV_scan( TI_HANDLE hMacServices, scan_Params_t *scanParams, BOOLEAN bHighPriority,
                        BOOLEAN bDriverMode, BOOLEAN bScanOnDriverModeError, 
                        PowerMgr_802_11_PsMode_e psRequest, BOOLEAN bSendNullData,
                        CmdResponseCB_t commandResponseFunc, TI_HANDLE commandResponseObj)
{
   scanSRV_t *pScanSRV = (scanSRV_t*)((MacServices_t*)hMacServices)->hScanSRV;


    WLAN_REPORT_INFORMATION( pScanSRV->hReport, SCAN_SRV_MODULE_LOG, ("Scan request received.\n") );

    /* sanity check - scan can only start if the scan SRV is idle */
    if ( SCAN_SRV_STATE_IDLE != pScanSRV->SMState )
    {
        WLAN_REPORT_WARNING( pScanSRV->hReport, SCAN_SRV_MODULE_LOG, ("Scan request while scan is running!\n") );
        return NOK;
    }

    /* Response function for GWSI only. In Legacy run we get NULL and never use it. */
    pScanSRV->commandResponseFunc = commandResponseFunc;
    pScanSRV->commandResponseObj  = commandResponseObj;

    pScanSRV->bInRequest = TRUE;
    pScanSRV->returnStatus = OK;

    /* copy scan paramaters */
    pScanSRV->scanParams = scanParams;
    pScanSRV->bHighPriority = bHighPriority;
    pScanSRV->bScanOnDriverModeFailure = bScanOnDriverModeError;
    pScanSRV->bSendNullData = bSendNullData;
    pScanSRV->psRequest = psRequest;

    if ( SCAN_TYPE_SPS == scanParams->scanType )
    {
        pScanSRV->bSPSScan = TRUE;

    }
    else
    {
        pScanSRV->bSPSScan = FALSE;
    }


    /* check whether the scan will overlap DTIM frame */
    if ( (FALSE == bHighPriority) && (TRUE == bDriverMode) )
    {
        pScanSRV->bDtimOverlapping = FALSE;
    }
    else
    {
        pScanSRV->bDtimOverlapping = TRUE;
    }

    /* mark the no scan complete flag. The purpose of this flag is to be able to identify
       whether the scan complete is a normal process, or was it generated because a no scan ocmplete
       was identified, a stop scan command was snet to the FW, and thus a scan complete was received.
       In the former case we nullify the consecutive no scan complete counter, whereas in the latter
       we do not. */
    pScanSRV->bNoScanCompleteFlag = FALSE;

    /* if required to enter driver mode */
    if ( TRUE == bDriverMode )
    {
        pScanSRV->bExitFromDriverMode = TRUE;
        /* send a PS_REQUEST event */
        scanSRVSM_SMEvent( (TI_HANDLE)pScanSRV, (scan_SRVSMStates_e*)&(pScanSRV->SMState), SCAN_SRV_EVENT_REQUEST_PS );
    }
    /* no driver mode required */
    else
    {
        pScanSRV->bExitFromDriverMode = FALSE;
        /* send a PS_SUCCESS event - will start the scan */
        scanSRVSM_SMEvent( (TI_HANDLE)pScanSRV, (scan_SRVSMStates_e*)&pScanSRV->SMState, SCAN_SRV_EVENT_PS_SUCCESS );
    }

    pScanSRV->bInRequest = FALSE;
    
    return pScanSRV->returnStatus;
}

/**
 * \author Ronen Kalish\n
 * \date 29-Dec-2004\n
 * \brief Sends a Stop Scan command to FW, no matter if we are in scan progress or not
 *
 * Function Scope \e Public.\n
 * \param hMacServices - handle to the MacServices object.\n
 * \param bSendNullData - indicates whether to send Null data when exiting driver mode.\n
 * \param commandResponseFunc - CB function which called after downloading the command. \n
 * \param commandResponseObj -  The CB function Obj (Notice : last 2 params are NULL in Legacy run). \n
 * \return OK if successful (various, TBD codes if not).\n
 */
TI_STATUS MacServices_scanSRV_stopScan( TI_HANDLE hMacServices, BOOLEAN bSendNullData, CmdResponseCB_t ScanCommandResponseCB, TI_HANDLE CB_handle )
{
    scanSRV_t *pScanSRV = (scanSRV_t*)((MacServices_t*)hMacServices)->hScanSRV;
    int stopScanStatus;

    WLAN_REPORT_INFORMATION( pScanSRV->hReport, SCAN_SRV_MODULE_LOG, ("Stop scan request received.\n") );

    /* update the driver mode exit flag */
    pScanSRV->bSendNullData = bSendNullData;

    if ( TRUE == pScanSRV->bSPSScan )
    {
        stopScanStatus = whalCtrl_StopSPSScan( pScanSRV->hHalCtrl , (void *)ScanCommandResponseCB, CB_handle);
    }
    else
    {
        stopScanStatus = whalCtrl_StopScan( pScanSRV->hHalCtrl,(void *)ScanCommandResponseCB, CB_handle);
    }

    if (OK != stopScanStatus)
    {
        return NOK;
    }
    /* send a stop scan event */
    return scanSRVSM_SMEvent( (TI_HANDLE)pScanSRV, (scan_SRVSMStates_e*)&pScanSRV->SMState, SCAN_SRV_EVENT_STOP_SCAN );
}

/**
 * \author Ronen Kalish\n
 * \date 17-Jan-2005\n
 * \brief Notifies the scan SRV of a FW reset (that had originally been reported by a different module).\n
 *
 * Function Scope \e Public.\n
 * \param hMacServices - handle to the MacServices object.\n
 * \return OK if successful (various, TBD codes if not).\n
 */
TI_STATUS MacServices_scanSRV_stopOnFWReset( TI_HANDLE hMacServices )
{
   scanSRV_t *pScanSRV = (scanSRV_t*)((MacServices_t*)hMacServices)->hScanSRV;

    WLAN_REPORT_INFORMATION( pScanSRV->hReport, SCAN_SRV_MODULE_LOG, ("FW reset notification received.\n") );

    /* mark the return status */
    pScanSRV->returnStatus = NOK;

    /* send a FW reset event */
    return scanSRVSM_SMEvent( (TI_HANDLE)pScanSRV, (scan_SRVSMStates_e*)&pScanSRV->SMState, SCAN_SRV_EVENT_FW_RESET );
}

/**
 * \author Ronen Kalish\n
 * \date 29-Dec-2004\n
 * \brief callback function used by the power server to notify driver mode result
 *          this CB is used in requesting PS and exiting PS.
 * Function Scope \e Public.\n
 * \param hScanSRV - handle to the scan SRV object.\n
 * \param psStatus - the power save request status.\n
 */
void MacServices_scanSRV_powerSaveCB( TI_HANDLE hScanSRV, UINT8 PSMode,UINT8 psStatus )
{
    scanSRV_t *pScanSRV = (scanSRV_t*)hScanSRV;

    WLAN_REPORT_INFORMATION( pScanSRV->hReport, SCAN_SRV_MODULE_LOG, ("PS Call Back status %d .\n",psStatus) );

    /* if driver mode enter/exit succeedded */
    if ( (ENTER_POWER_SAVE_SUCCESS == psStatus) || (EXIT_POWER_SAVE_SUCCESS == psStatus) )
    {
        WLAN_REPORT_INFORMATION( pScanSRV->hReport, SCAN_SRV_MODULE_LOG, ("PS successful.\n") );

        /* send a PS_SUCCESS event */
        scanSRVSM_SMEvent( (TI_HANDLE)pScanSRV, (scan_SRVSMStates_e*)&pScanSRV->SMState, SCAN_SRV_EVENT_PS_SUCCESS );
    }
    /* driver mode entry failed, and scan is requested even on PS failure but we are entering PS and not Exiting */
    else if ( (TRUE == pScanSRV->bScanOnDriverModeFailure) && ( ENTER_POWER_SAVE_FAIL == psStatus) )
    {
        WLAN_REPORT_INFORMATION( pScanSRV->hReport, SCAN_SRV_MODULE_LOG, ("PS enter failed, continune scan .\n") );

        /* send a PS_SUCCESS event */
        scanSRVSM_SMEvent( (TI_HANDLE)pScanSRV, (scan_SRVSMStates_e*)&pScanSRV->SMState, SCAN_SRV_EVENT_PS_SUCCESS );
    }
    /* driver mode enter or exit failed */
    else
    {
        /* if we are trying to enter PS and fail to do so - return error on scan complete */
        if ( ENTER_POWER_SAVE_FAIL == psStatus) 
        {
            WLAN_REPORT_WARNING( pScanSRV->hReport, SCAN_SRV_MODULE_LOG, ("PS enter failed . quiting scan .\n") );
            /* Set the return status  */
            pScanSRV->returnStatus = NOK;
        }

        /* send a PS FAIL event */
        scanSRVSM_SMEvent( (TI_HANDLE)pScanSRV, (scan_SRVSMStates_e*)&pScanSRV->SMState, SCAN_SRV_EVENT_PS_FAIL );
    }
}


/**
 * \author Ronen Kalish\n
 * \date 29-Dec-2004\n
 * \brief Callback function used by the HAL ctrl to notify scan complete
 *
 * Function Scope \e Public.\n
 * \param hScanSRV - handle to the scan SRV object.\n
 * \param str - pointer to scan result buffer (holding SPS status for SPS scan only!).\n
 * \param strLen - scan result buffer length (should ALWAYS be 2, even for non SPS scans).\n
 */
void MacServices_scanSRV_scanCompleteCB( TI_HANDLE hScanSRV, char* str, UINT32 strLen )
{
    scanSRV_t *pScanSRV = (scanSRV_t*)hScanSRV;

    WLAN_REPORT_INFORMATION( pScanSRV->hReport, SCAN_SRV_MODULE_LOG, ("Scan complete notification from TNET.\n") );

    /* nullify the consecutive no scan complete events counter  - only if this is a scan complete that
       does not happen afetr a stop scan (due to a timer expiry) */
    if ( FALSE == pScanSRV->bNoScanCompleteFlag )
    {
        pScanSRV->currentNumberOfConsecutiveNoScanCompleteEvents = 0;
    }

    /* copy scan results according to scan type (only meaningful for SPS scan) */
    if ( FALSE == pScanSRV->bSPSScan ) 
    {
        /* normal scan - no result is available */
        pScanSRV->bTSFError = FALSE;
        WLAN_REPORT_INFORMATION( pScanSRV->hReport, SCAN_SRV_MODULE_LOG,
                                 ("Normal scan completed.\n") );
    }
    else
    {
        /* SPS scan - first byte indicates whether a TSF error (AP recovery) occured */
        if ( 0 != str[ 0 ] )
        {
            pScanSRV->bTSFError = TRUE;
        }
        else
        {
            pScanSRV->bTSFError = FALSE;
        }

        /* next two bytes indicates on which channels scan was attempted */
/*        pScanSRV->SPSScanResult = ENDIAN_HANDLE_WORD( *((UINT16*)&(str[ 1 ])) ); */

/* Swap of aligned UINT16* */
        COPY_UNALIGNED_WORD(&pScanSRV->SPSScanResult, &str[1]);
        pScanSRV->SPSScanResult = ENDIAN_HANDLE_WORD( pScanSRV->SPSScanResult );

        WLAN_REPORT_INFORMATION( pScanSRV->hReport, SCAN_SRV_MODULE_LOG,
                                 ("SPS scan completed. TSF error: %s, SPS result: %x\n",
                                  (TRUE == pScanSRV->bTSFError ? "Yes": "No"), pScanSRV->SPSScanResult) );
    }

    /* send a SCAN_COMPLETE event  */
    scanSRVSM_SMEvent( (TI_HANDLE)pScanSRV, (scan_SRVSMStates_e*)&pScanSRV->SMState, SCAN_SRV_EVENT_SCAN_COMPLETE );
}

/**
 * \author Ronen Kalish\n
 * \date 29-Dec-2004\n
 * \brief called when a scan timer expires. Completes the scan and starts a recovery process.
 *
 * Function Scope \e Public.\n
 * \param hScanSRV - handle to the scan SRV object.\n
 */
void MacServices_scanSRV_scanTimerExpired( TI_HANDLE hScanSRV )
{
    scanSRV_t *pScanSRV = (scanSRV_t*)hScanSRV;

    /* mark that no scan complete occured (see sanSRV_scan for more detailed explanation) */
    pScanSRV->bNoScanCompleteFlag = TRUE;

    /* send a TIMER_EXPIRED event */
    scanSRVSM_SMEvent( (TI_HANDLE)pScanSRV, (scan_SRVSMStates_e*)&pScanSRV->SMState, SCAN_SRV_EVENT_TIMER_EXPIRED );
}

/**
 * \author Ronen Kalish\n
 * \date 29-Dec-2004\n
 * \brief Calculates the maximal time required for a scan operation
 *
 * Function Scope \e Public.\n
 * \param hScanSRV - handle to the scan SRV object.\n
 * \param scanParams - the scan parameters
 * \param bConsiderDTIM - whether this scan overlaps DTIM
 * \return the time (in milliseconds)
 */
UINT32 MacServices_scanSRVcalculateScanTimeout( TI_HANDLE hScanSRV, scan_Params_t* scanParams, BOOLEAN bConsiderDTIM )
{
    UINT32 i, dtimPeriod, beaconInterval,timeout = 0;
    scanSRV_t *pScanSRV = (scanSRV_t*)hScanSRV;

    /********************************************************************************
        timeout calculation is performed according to scan type:
        1. for normal scan, multiply the channel time by the number of channels.
           if this scan is not overlapping DTIM, add the DTIM period (in case
           starting the scan right now will cause the very last milliseconds of the
           scan to overlap the next DTIM). Add the guard time.
        2. for triggered scan, multiply the channel time plus the trigger time
           constant (the maximum time between two frames from the Tid
           according to which the scan is triggered) by the number of channels.
           DTIM period is added only as precaution - since the scan is divided to
           channels, only very few of them will be delayed due to DTIM (in the worst 
           case), and this delay would be only the length of one channel scan.
           Eventually, Add the guard time.
        3. for SPS scan: Measure the time from current TSF to the TSF at which the 
           scan is scheduled to finish (done by the scan manager, and passed as 
           a parameter in the scan structure). Add guard time. DTIM overlapping is not 
           considered because if the scan overlaps DTIM the channels which are 
           scheduled during the DTIM (if any) won't be scanned.
     ********************************************************************************/

    /* get DTIM time, if scanning in connected mode and need to consider DTIM */
    if ( bConsiderDTIM ) 
    {  
        /* new dtimPeriod calculation */
        beaconInterval = whalCtrl_GetBeaconInterval(pScanSRV->hHalCtrl);
        beaconInterval = MacServices_scanSRVConvertTUToMsec( beaconInterval );
        dtimPeriod = whalCtrl_GetDtimCount(pScanSRV->hHalCtrl);
        dtimPeriod = beaconInterval*dtimPeriod; 
    }
    else
    {
        dtimPeriod = 0;
    }

    switch (scanParams->scanType)
    {
    case SCAN_TYPE_NORMAL_ACTIVE:
    case SCAN_TYPE_NORMAL_PASSIVE:
        /* the timeout is the scan duration on all channels */
        for ( i = 0; i < scanParams->numOfChannels; i++ )
        {
            timeout += scanParams->channelEntry[ i ].normalChannelEntry.maxChannelDwellTime;
        }
        timeout = (timeout / 1000) + dtimPeriod + SCAN_SRV_FW_GUARD_TIME_MS;
        break;

    case SCAN_TYPE_TRIGGERED_ACTIVE:
    case SCAN_TYPE_TRIGGERED_PASSIVE:
        /* the timeout is the scan duration on all channels, plus the maximum time that can pass 
           between two different frames from the same Tid */
        for ( i = 0; i < scanParams->numOfChannels; i++ )
        {
            timeout += scanParams->channelEntry[ i ].normalChannelEntry.maxChannelDwellTime;
        }
        timeout = (timeout / 1000) + dtimPeriod + 
                  ((pScanSRV->uTriggeredScanTimeOut / 1000 + 1) * scanParams->numOfChannels) + 
                  SCAN_SRV_FW_GUARD_TIME_MS;
        break;

    case SCAN_TYPE_SPS:
        timeout = scanParams->SPSScanDuration + SCAN_SRV_FW_GUARD_TIME_MS;
        break;

    default:
        WLAN_REPORT_ERROR( pScanSRV->hReport, SCAN_SRV_MODULE_LOG, 
                             ("Trying to calculate timeout for undefined scan type %d\n", scanParams->scanType) );
        break;
    }
    WLAN_REPORT_INFORMATION( pScanSRV->hReport, SCAN_SRV_MODULE_LOG, 
                         ("scanSRVcalculateScanTimeout, timeout = %d\n", timeout) );

    return timeout;
}

/**
 * \author Ronen Kalish\n
 * \date 16-Jan-2005\n
 * \brief Convert time units (1024 usecs) to millisecs
 *
 * Function Scope \e Private.\n
 * \param tu - the time in time units
 * \return the time in milliseconds
 */
UINT32 MacServices_scanSRVConvertTUToMsec( UINT32 tu )
{
    return (tu * 1024) / 1000;
}

#ifdef TI_DBG
/**
 * \author Shirit Brook\n
 * \date God knows when...\n
 * \brief Prints Scan Server SM status.\n
 *
 * Function Scope \e Public.\n
 * \param hMacServices - handle to the Mac Services object.\n
 * \return always OK.\n
 */
void MacServices_scanSrv_printDebugStatus(TI_HANDLE hMacServices)
{
   scanSRV_t *pScanSRV = (scanSRV_t*)((MacServices_t*)hMacServices)->hScanSRV;

    WLAN_OS_REPORT(("scanSrv State="));
    switch (pScanSRV->SMState)
    {
    case SCAN_SRV_STATE_IDLE:
        WLAN_OS_REPORT((" IDLE\n"));
        break;
    case SCAN_SRV_STATE_PS_WAIT:
        WLAN_OS_REPORT((" PS_WAIT\n"));
        break;
    case SCAN_SRV_STATE_PS_EXIT:
        WLAN_OS_REPORT((" PS_EXIT\n"));
        break;
    case SCAN_SRV_STATE_SCANNING:
        WLAN_OS_REPORT((" SCANNING\n"));
        break;
    case SCAN_SRV_STATE_STOPPING:
        WLAN_OS_REPORT((" STOPPING\n"));
        break;
    default:
        WLAN_OS_REPORT((" Invalid State=%d\n",pScanSRV->SMState));
        break;

    }

    if (NULL != pScanSRV->scanParams)
    {
        WLAN_OS_REPORT(("scanSrv bExitFromDriverMode=%d, bHighPriority=%d, bInRequest=%d,\n \
                        bScanOnDriverModeFailure=%d, bSendNullData=%d, bSPSScan=%d, bTimerRunning=%d, \n \
                        psRequest=%d, scanType=%d\n", 
                        pScanSRV->bExitFromDriverMode,
                        pScanSRV->bHighPriority,
                        pScanSRV->bInRequest,
                        pScanSRV->bScanOnDriverModeFailure,
                        pScanSRV->bSendNullData,
                        pScanSRV->bSPSScan,
                        pScanSRV->bTimerRunning,
                        pScanSRV->psRequest,
                        pScanSRV->scanParams->scanType));
    }
}
#endif

