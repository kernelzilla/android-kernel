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
/*  MODULE:  recoveryMgr.c                                                     */
/*  PURPOSE: The responsibility of RecoveryMgr module is to provide main API   */
/*           to HelthMonitor that invokes the recovery process if failure is   */
/*           detected. It performs disable/enable inputs from outside, calls   */
/*           restart of TWD and informs STAD modules that recovery has been    */
/*           performed.                                                        */
/*                                                                             */
/*******************************************************************************/

#include "paramOut.h"
#include "osApi.h"
#include "DataCtrl_Api.h"
#include "report.h"
#include "recoveryMgr.h"
#include "recoveryMgr_API.h"
#include "TNETW_Driver_api.h"

#include "healthMonitor.h"
#include "PowerMgr_API.h"
#include "siteMgrApi.h"
#include "currBss.h"
#include "whalCtrl.h"

#include "TNETW_Driver.h"
#include "TNETWIF.h"
#include "SoftGeminiApi.h"

/* static function */
#ifdef USE_RECOVERY
static void recoveryMgr_SM(TI_HANDLE hRecoveryMgr);
static TI_STATUS recoveryMgr_notifyStadAboutRecovery(TI_HANDLE hRecoveryMgr);
#endif /* USE_RECOVERY */

/*******************************************************************************
*                       PUBLIC  FUNCTIONS  IMPLEMENTATION                      *
********************************************************************************/

/*************************************************************************
*                        recoveryMgr_create                              *
**************************************************************************
* DESCRIPTION:  This function initializes the RecoveryMgr module.
*
* INPUT:        hOs - handle to Os Abstraction Layer
*               
* RETURN:       Handle to the allocated RecoveryMgr module
*************************************************************************/
TI_HANDLE recoveryMgr_create(TI_HANDLE hOs)
{
#ifdef USE_RECOVERY
    recoverMgr_t *hRecoveryMgr;

    /* allocate RecoverMgr module */
    hRecoveryMgr = os_memoryAlloc(hOs, (sizeof(recoverMgr_t)));

    if(!hRecoveryMgr)
    {
        WLAN_OS_REPORT(("Error allocating the RecoverMgr Module\n"));
        return NULL;
    }

    /* Reset RecoverMgr module */
    os_memoryZero(hOs, hRecoveryMgr, (sizeof(recoverMgr_t)));

    hRecoveryMgr->hOs = hOs;

    return(hRecoveryMgr);
#else
    return NULL;
#endif /* USE_RECOVERY */
} /* recoveryMgr_create */



/***************************************************************************
*                           recoveryMgr_config                             *
****************************************************************************
* DESCRIPTION:  This function configures the recoveryMgr module
*
* RETURNS:      OK - Configuration successful
*               NOK - Configuration unsuccessful
***************************************************************************/
TI_STATUS recoveryMgr_config(TI_HANDLE hRecoveryMgr,
                             TI_HANDLE hReport,
                             TI_HANDLE hTxData,
                             TI_HANDLE hTnetwDrv,
                             TI_HANDLE hScr, 
                             TI_HANDLE hCurrBss,
                             TI_HANDLE hPowerMgr,
                             TI_HANDLE hHealthMonitor,
							 TI_HANDLE hSoftGemini)
{
#ifdef USE_RECOVERY
    recoverMgr_t *pRecoverMgr = (recoverMgr_t *)hRecoveryMgr;

    /* configure modules handles */
    pRecoverMgr->hReport = hReport;
    pRecoverMgr->hTxData = hTxData;
    pRecoverMgr->hTnetwDrv = hTnetwDrv;
    pRecoverMgr->hScr = hScr;
    pRecoverMgr->hCurrBss = hCurrBss;
    pRecoverMgr->hPowerMgr = hPowerMgr;

    pRecoverMgr->hHealthMonitor = hHealthMonitor;
	pRecoverMgr->hSoftGemini = hSoftGemini;

    pRecoverMgr->fRecoveryInProcess = FALSE;
    pRecoverMgr->smState = REC_MGR_STATE_IDLE;
    
    WLAN_REPORT_INIT(pRecoverMgr->hReport, RECOVERY_MGR_MODULE_LOG,
        (".....RecoveryMgr configured successfully\n"));
#endif /* USE_RECOVERY */
    return OK;
} /* recoveryMgr_config */

/***************************************************************************
*                           recoveryMgr_destroy                            *
****************************************************************************
* DESCRIPTION:  This function unload the RecoverMgr module. It frees
*               the RecoveryMgr module
*
* INPUTS:       hRecoveryMgr - the object
*
* OUTPUT:
*
* RETURNS:      OK - Unload succesfull
*               NOK - Unload unsuccesfull
***************************************************************************/
TI_STATUS recoveryMgr_destroy(TI_HANDLE hRecoveryMgr)
{
#ifdef USE_RECOVERY
    recoverMgr_t *pRecoverMgr = (recoverMgr_t *)hRecoveryMgr;

    /* free RecoverMgr Module */
    os_memoryFree(pRecoverMgr->hOs, pRecoverMgr, sizeof(recoverMgr_t));
#endif /* USE_RECOVERY */
    return OK;
}


/**********************************************************************************************
 *                  recoveryMgr_SM()
 **********************************************************************************************
 * DESCRIPTION:  
   ============
    This is the recoveryMgr state machine.
    The inceptive event for RecoveryMgr SM is invoking the recovery process by HelthMonitor. 
    After disabling Outside Inputs and starting TWD Restart, RecoveryMgr SM waits end of TWD Restart, 
    then informs STAD about recovery and enables Outside Inputs. 
    The SM supports both Sync and Async accesses to the HW.
    It loops and progresses from state to state as long as the HW is accessed synchronously.
    Once the access is Asynchronous (TNETWIF_PENDING), it exits and is called later
      by the TNETWIF when the HW is ready.
    That's why it uses unspecified-mode accesses (e.g. TNETWIF_ReadMemOpt) which
      selects either Sync or Async automatically according to the platform and length.
    Yet, the short transactions (EOB and Interrupt-Request 32 bit writes) are done using Sync 
      access to simplify the SM 
    NOTE: MCS projects may require full Sync/Async support, so the Sync accesses may need to be modified. 

    NOTE:  The recoveryMgr-SM detailed description is provided in "CE-2.0 Recovery LLD.doc".

 **********************************************************************************************/
#ifdef USE_RECOVERY
static void recoveryMgr_SM(TI_HANDLE hRecoveryMgr)
{
    recoverMgr_t        *pRecoverMgr = (recoverMgr_t *)hRecoveryMgr;
    healthMonitor_t     *pHealthMonitor = (healthMonitor_t *)pRecoverMgr->hHealthMonitor;

#ifdef TI_DBG
    if (hRecoveryMgr == NULL)
    {
        WLAN_REPORT_ERROR(pRecoverMgr->hReport, RECOVERY_MGR_MODULE_LOG,  
                          ("recoveryMgr_SM(): ****  Called with NULL handle!!  ****\n"));
        return;
    }
#endif /* TI_DBG */

    WLAN_REPORT_INFORMATION(pRecoverMgr->hReport, RECOVERY_MGR_MODULE_LOG,  
                            ("recoveryMgr_SM(): smState=%d\n", pRecoverMgr->smState));

    /* 
     * Loop through the states sequence as long as the process is synchronous.
     * Exit when finished or if an Asynchronous process is required. In this case
     *   the SM process will be resumed later (called back by TNETWIF). 
     */
    switch (pRecoverMgr->smState)
    {
        
        case REC_MGR_STATE_IDLE:
            WLAN_REPORT_INFORMATION(pRecoverMgr->hReport, RECOVERY_MGR_MODULE_LOG,
                                    (".....REC_MGR_STATE_IDLE\n"));

            healthMonitor_suspendPeriodicTest(pRecoverMgr->hHealthMonitor);

            /* disabling Outside Inputs and starting TWD Restart */
            pRecoverMgr->fDisableInputsFromOs = TRUE;

            /* suspend TX */
            txData_stop(pHealthMonitor->hTxData);

            /* Disabling the IRQ line so the recovery will be an atomic action */
            os_disableIrq (pRecoverMgr->hOs);

            pRecoverMgr->smState = REC_MGR_STATE_WAIT_TWD_RESTART;

            TnetwDrv_StartRecovery(pRecoverMgr->hTnetwDrv, 
                                   (void*)recoveryMgr_endOfRecovery, 
                                   hRecoveryMgr);

            return;

        case REC_MGR_STATE_WAIT_TWD_RESTART:
            WLAN_REPORT_INFORMATION(pRecoverMgr->hReport, RECOVERY_MGR_MODULE_LOG,
                        (".....REC_MGR_STATE_WAIT_TWD_RESTART\n"));
            /* informs STAD about recovery */
            recoveryMgr_notifyStadAboutRecovery(hRecoveryMgr);

            /* TX resume */
            txData_startAfterRecovery(pHealthMonitor->hTxData);

            pRecoverMgr->fDisableInputsFromOs = FALSE;

            /* call inside CmdMBox_SetModeNormal */
            whalCtrl_exitFromInitMode(((healthMonitor_t *)pRecoverMgr->hHealthMonitor)->hHalCtrl);

            /* send the min power level to the FW */
            MacServices_powerAutho_ExitFromInit(((TnetwDrv_t *)pRecoverMgr->hTnetwDrv)->hMacServices);

			/* Soft Gemini Section */
			SoftGemini_handleRecovery(pRecoverMgr->hSoftGemini);

            WLAN_OS_REPORT((".....recoveryMgr: End Of Recovery\n"));

            healthMonitor_resumePeriodicTest(pRecoverMgr->hHealthMonitor);

            pRecoverMgr->fRecoveryInProcess = FALSE;

            pRecoverMgr->smState = REC_MGR_STATE_IDLE;
            return; /* recovery process ended */

        default:
                WLAN_REPORT_ERROR(pRecoverMgr->hReport, RECOVERY_MGR_MODULE_LOG,  
                    ("recoveryMgr_SM(): Unexpected state, smState=%d\n", pRecoverMgr->smState));
            return;

    }  /* switch (pRecoverMgr->smState) */
} /* recoveryMgr_SM */
#endif /* USE_RECOVERY */

/***************************************************************************
*                           recoveryMgr_recoveryProcess                    *
****************************************************************************
* DESCRIPTION:  Main interface that called if the WLAN driver detects error
*               during the Monitoring process.
*
* INPUTS:       hRecoveryMgr - the object
*
* OUTPUT:
*
* RETURNS:      OK - Recovery Process started
*               NOK - Recovery cannot started because it's in process already
***************************************************************************/
TI_STATUS recoveryMgr_recoveryProcess(TI_HANDLE hRecoveryMgr)
{
#ifdef USE_RECOVERY
    recoverMgr_t *pRecoverMgr = (recoverMgr_t *)hRecoveryMgr;
    healthMonitor_t     *pHealthMonitor = (healthMonitor_t *)pRecoverMgr->hHealthMonitor;
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)pHealthMonitor->hHalCtrl;
    WlanParams_T *pWlanParams = whal_ParamsGetWlanParams(pWhalCtrl->pWhalParams);

    if(!pWlanParams->RecoveryEnable)
    {
        WLAN_OS_REPORT(("recoveryMgr_recoveryProcess: Recovery is disabled in tiwlan.ini, abort recovery process\n"));
        return OK;
    }
    else
        WLAN_OS_REPORT((".....recoveryMgr_recoveryProcess\n"));

    if(pRecoverMgr->fRecoveryInProcess == FALSE)
    {
        pHealthMonitor->numOfRecoveryPerformed++;
        pRecoverMgr->fRecoveryInProcess = TRUE;
        recoveryMgr_SM(hRecoveryMgr);
        return OK;
    }
    else
    {
        WLAN_REPORT_ERROR(pRecoverMgr->hReport, RECOVERY_MGR_MODULE_LOG,  
            ("recoveryProcess(): ****  Recovery in process already!!  ****\n"));
        return NOK;
    }
#else
    return OK;
#endif /* USE_RECOVERY */
} /* recoveryMgr_recoveryProcess */




/***************************************************************************
*                           recoveryMgr_endOfRecovery                      *
****************************************************************************
* DESCRIPTION:  This function is the CB from the RecoveryCtrl that will 
*               issue the "EndOfTwdRestart" event to the RecoveryMgr SM. 
*               Indicates that TWD has performed its recovery.
*
* INPUTS:       hRecoveryMgr - the object
*
* OUTPUT:
*
* RETURNS:      OK - succesfull
*               NOK - unsuccesfull
***************************************************************************/
TI_STATUS recoveryMgr_endOfRecovery(TI_HANDLE hRecoveryMgr)
{
#ifdef USE_RECOVERY
    recoverMgr_t *pRecoverMgr = (recoverMgr_t *)hRecoveryMgr;

    WLAN_REPORT_INIT(pRecoverMgr->hReport, RECOVERY_MGR_MODULE_LOG,
        (".....recoveryMgr_endOfRecovery\n"));

    recoveryMgr_SM(hRecoveryMgr);
#endif /* USE_RECOVERY */
    return OK;
} /* recoveryMgr_endOfRecovery */

/***************************************************************************
*                 recoveryMgr_notifyStadAboutRecovery                      *
****************************************************************************
* DESCRIPTION:  Inform STAD that recovery has been performed: inform TX, 
*               SCR, Current BSS and Power Mgr about FW reset.
*
* INPUTS:       hRecoveryMgr - the object
*
* OUTPUT:
*
* RETURNS:      OK - succesfull
*               NOK - unsuccesfull
***************************************************************************/
#ifdef USE_RECOVERY
static TI_STATUS recoveryMgr_notifyStadAboutRecovery(TI_HANDLE hRecoveryMgr)
{
    recoverMgr_t *pRecoverMgr = (recoverMgr_t *)hRecoveryMgr;
    healthMonitor_t     *pHealthMonitor = (healthMonitor_t *)pRecoverMgr->hHealthMonitor;

    txData_recoveryIndication (pHealthMonitor->hTxData);
    TnetwDrv_RecoveryCtrlBlk(pHealthMonitor->hTnetwDrv);

    scr_notifyFWReset( pRecoverMgr->hScr );
    currBSS_performRecovery(pRecoverMgr->hCurrBss);
    PowerMgr_notifyFWReset(pRecoverMgr->hPowerMgr);

    return OK;
} /* recoveryMgr_notifyStadAboutRecovery */
#endif /* USE_RECOVERY */
/***************************************************************************
*                           recoveryMgr_recoveryProcess                    *
****************************************************************************
* DESCRIPTION:  check if Inputs From OS are Disabled.
*
* INPUTS:       hRecoveryMgr - the object
*
* OUTPUT:
*
* RETURN:       TRUE  - Inputs From OS are Disabled 
*               FALSE - Inputs From OS are not Disabled
***************************************************************************/
BOOL recoveryMgr_areInputsFromOsDisabled(TI_HANDLE hRecoveryMgr)
{
#ifdef USE_RECOVERY
    recoverMgr_t *pRecoverMgr = (recoverMgr_t *)hRecoveryMgr;

    return (pRecoverMgr->fDisableInputsFromOs);
#else
    return FALSE;
#endif /* USE_RECOVERY */
} /* recoveryMgr_recoveryProcess */



