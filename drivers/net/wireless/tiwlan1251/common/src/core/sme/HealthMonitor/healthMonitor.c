/** \file healthMonitor.c
 *  \brief Firmware Recovery Mechanism
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

/****************************************************************************/
/*                                                                          */
/*      MODULE:     healthMonitor.c                                         */
/*      PURPOSE:    Driver interface to OS abstraction layer                */
/*                                                                          */
/****************************************************************************/
#include "healthMonitor.h"

#ifdef _WINDOWS
#endif 

#include "osApi.h"
#include "utils.h"
#include "report.h"
#include "siteMgrApi.h"
#include "whalCtrl_api.h" 
#include "PowerMgr_API.h"
#include "currBss.h"
#include "DataCtrl_Api.h"
#include "TNETW_Driver_api.h"
#include "srcApi.h"
#include "SoftGeminiApi.h"
#include "currBss.h"
#include "whalCtrl_api.h"
#include "public_host_int.h"
#include "rsnApi.h"
#ifdef DEBUG_FIRMWARE
#include "whalCtrl.h"
#endif

#include "recoveryMgr_API.h"

/* Keep-alive period */
#define KEEP_ALIVE_TIEMOUT    10000


static void healthMonitor_proccessFailureEvent (TI_HANDLE hHealthMonitor);
static void healthMonitor_RSSI_CB (TI_HANDLE hHealthMonitor);


#ifdef REPORT_LOG

static char* sRecoveryTriggersNames [MAX_FAILURE_EVENTS] = 
{
    "NO_SCAN_COMPLETE_FAILURE",
    "MBOX_FAILURE",
    "HW_AWAKE_FAILURE",
    "BUS_ERROR",
    "DEVICE_ERROR",
    "TX_STUCK",
    "DISCONNECT_TIMEOUT",
    "POWER_SAVE_FAILURE",
    "MEASUREMENT_FAILURE",
};

#endif


/***********************************************************************
 *                        healthMonitor_create
 ***********************************************************************
DESCRIPTION: 
           

INPUT:      

OUTPUT:

RETURN: 

************************************************************************/
TI_HANDLE healthMonitor_create (TI_HANDLE hOs)
{
    healthMonitor_t *pHealthMonitor;
 
    /* Allocate memory for the health monitor object and nullify it */
    pHealthMonitor = (healthMonitor_t*)os_memoryAlloc (hOs, sizeof(healthMonitor_t));
    if (pHealthMonitor == NULL)
    {
        return NULL;
    }
    os_memoryZero (hOs, pHealthMonitor, sizeof(healthMonitor_t));

    /* Store OS object handle */
    pHealthMonitor->hOs = hOs;

    /* Create periodic health check timer */
    pHealthMonitor->hHealtheCheckTimer = os_timerCreate (hOs, healthMonitor_performTest, (TI_HANDLE)pHealthMonitor);
    if (NULL == pHealthMonitor->hHealtheCheckTimer)
    {
        healthMonitor_unload ((TI_HANDLE)pHealthMonitor);
        return NULL;
    }

    /* Create recovery request timer */
    pHealthMonitor->hFailTimer = os_timerCreate (hOs, healthMonitor_proccessFailureEvent, (TI_HANDLE)pHealthMonitor);
    if (NULL == pHealthMonitor->hFailTimer)
    {
        healthMonitor_unload ((TI_HANDLE)pHealthMonitor);
        return NULL;
    }

    return (TI_HANDLE)pHealthMonitor;
}


/***********************************************************************
 *                        healthMonitor_config
 ***********************************************************************
DESCRIPTION:            

INPUT:      

OUTPUT:

RETURN: 

************************************************************************/
TI_STATUS healthMonitor_config (TI_HANDLE    hHealthMonitor, 
                                TI_HANDLE    hReport,
                                TI_HANDLE    hHalCtrl,
                                TI_HANDLE    hSiteMgr,
                                TI_HANDLE    hScr,
                                TI_HANDLE    hSoftGemini,
                                TI_HANDLE    hTnetwDrv,
                                TI_HANDLE    hMemMgr,
                                TI_HANDLE    hConfigMgr,
                                TI_HANDLE    hTxData,
                                TI_HANDLE    hCurrBss,
                                TI_HANDLE    hRsn,
                                healthMonitorInitParams_t *healthMonitorInitParams,
								TI_HANDLE    hRecoveryMgr)
{
    healthMonitor_t *pHealthMonitor = hHealthMonitor;
    int i;

    pHealthMonitor->hReport         = hReport;
    pHealthMonitor->hHalCtrl        = hHalCtrl;
    pHealthMonitor->hSiteMgr        = hSiteMgr;
    pHealthMonitor->hScr            = hScr;
    pHealthMonitor->hSoftGemini     = hSoftGemini;
    pHealthMonitor->hTnetwDrv       = hTnetwDrv;
    pHealthMonitor->hMemMgr         = hMemMgr;
    pHealthMonitor->hConfigMgr      = hConfigMgr;
    pHealthMonitor->hTxData         = hTxData;
    pHealthMonitor->hCurrBss        = hCurrBss;
    pHealthMonitor->hRsn            = hRsn;
    pHealthMonitor->state           = HEALTH_MONITOR_STATE_DISCONNECTED;
    pHealthMonitor->bRunSoftRecovery = FALSE;
    pHealthMonitor->failureEvent    = (UINT32)NO_FAILURE;
    pHealthMonitor->hRecoveryMgr    = hRecoveryMgr;

    /* Registry configuration */
    pHealthMonitor->bFullRecoveryEnable = healthMonitorInitParams->FullRecoveryEnable;
    pHealthMonitor->timerInterval = healthMonitorInitParams->healthCheckPeriod;

    for (i = 0; i < MAX_FAILURE_EVENTS; i++)
    {
        pHealthMonitor->recoveryTriggerEnabled[i] = healthMonitorInitParams->recoveryTriggerEnabled[i];
    }

    /* 
     * Set the keep-alive Interval, which is used to signal at how many timer intervals
     * a keep alive (null data) packet needs to be sent to the AP. A packet should be sent every
     * KEEP_ALIVE_TIEMOUT seconds (default is 10 seconds).
     */
    if (pHealthMonitor->timerInterval != 0)
    {
        pHealthMonitor->keepAliveIntervals = KEEP_ALIVE_TIEMOUT / pHealthMonitor->timerInterval;
    }
    else
    {
        pHealthMonitor->keepAliveIntervals = 1;
    }
    pHealthMonitor->currentKeepAliveCounter = 0;

    /* Register the failure event callback */
    TnetwDrv_Register_CB (hTnetwDrv, 
                          TNETW_DRIVER_EVENT_FAILURE, 
                          (void *)healthMonitor_sendFailureEvent, 
                          hHealthMonitor);

    return OK;
}


/***********************************************************************
 *                        healthMonitor_unload
 ***********************************************************************
DESCRIPTION: 
           
INPUT:      

OUTPUT:

RETURN: 

************************************************************************/
TI_STATUS healthMonitor_unload (TI_HANDLE hHealthMonitor)
{
    healthMonitor_t *pHealthMonitor;

    pHealthMonitor = (healthMonitor_t*)hHealthMonitor;

    if (pHealthMonitor != NULL)
    {
        if (NULL != pHealthMonitor->hHealtheCheckTimer)
        {
            /* Release the timer */
            os_timerDestroy (pHealthMonitor->hOs, pHealthMonitor->hHealtheCheckTimer);
        }

        if (NULL != pHealthMonitor->hFailTimer)
        {
            /* Release the timer */
            os_timerDestroy (pHealthMonitor->hOs, pHealthMonitor->hFailTimer);
        }

        /* Freeing the object should be called last !!!!!!!!!!!! */
        os_memoryFree (pHealthMonitor->hOs, pHealthMonitor, sizeof(healthMonitor_t));
    }

    return OK;
}


/***********************************************************************
 *                        healthMonitor_setState
 ***********************************************************************
DESCRIPTION: 
           

INPUT:      

OUTPUT:

RETURN: 

************************************************************************/
void healthMonitor_setState (TI_HANDLE hHealthMonitor, healthMonitorState_e state)
{
    healthMonitor_t *pHealthMonitor = (healthMonitor_t*)hHealthMonitor;

    pHealthMonitor->state = state;

    switch (state)
    {
    case HEALTH_MONITOR_STATE_DISCONNECTED:
        /* Stop health monitor check */
        os_timerStop (pHealthMonitor->hOs, pHealthMonitor->hHealtheCheckTimer);
        break;

    case HEALTH_MONITOR_STATE_CONNECTED:
        /* Start health monitor check */
        os_timerStart (pHealthMonitor->hOs, pHealthMonitor->hHealtheCheckTimer, pHealthMonitor->timerInterval, TRUE);
        break;
    }
}


/***********************************************************************
 *                        healthMonitor_suspendPeriodicTest
 ***********************************************************************
DESCRIPTION: 
           
INPUT:      

OUTPUT:

RETURN: 

************************************************************************/
void healthMonitor_suspendPeriodicTest (TI_HANDLE hHealthMonitor)
{
    healthMonitor_t *pHealthMonitor = (healthMonitor_t*)hHealthMonitor;

    WLAN_REPORT_INFORMATION (pHealthMonitor->hReport, SITE_MGR_MODULE_LOG,
                             ("%s: state=%d, suspend=%d\n", 
                             __FUNCTION__, pHealthMonitor->state, pHealthMonitor->bSuspended));
    
    pHealthMonitor->bSuspended = TRUE;
}


/***********************************************************************
 *                        healthMonitor_resumePeriodicTest
 ***********************************************************************
DESCRIPTION: 
           

INPUT:      

OUTPUT:

RETURN: 

************************************************************************/
void healthMonitor_resumePeriodicTest(TI_HANDLE hHealthMonitor)
{
    healthMonitor_t *pHealthMonitor = (healthMonitor_t*)hHealthMonitor;

    WLAN_REPORT_INFORMATION (pHealthMonitor->hReport, SITE_MGR_MODULE_LOG,
                             ("%s: state=%d, suspend=%d\n",
                             __FUNCTION__, pHealthMonitor->state, pHealthMonitor->bSuspended) );

    pHealthMonitor->bSuspended = FALSE;
}


/***********************************************************************
 *                        healthMonitor_performTest
 ***********************************************************************
DESCRIPTION: every T s ( 5sec ) will be called by TX Power Adjust timer
           
INPUT:      

OUTPUT:

RETURN: 

************************************************************************/
void healthMonitor_performTest (TI_HANDLE hHealthMonitor)
{
    healthMonitor_t *pHealthMonitor = (healthMonitor_t*)hHealthMonitor;

    if (FALSE == pHealthMonitor->bSuspended)
    {
        /*
         * The following call is disabled, because there's no point to send health check command if
         * the get RSSI is sent anyhow. When these operations will be
         * separated, the health test should be returned.
         */

        /*
        pHealthMonitor->numOfHealthTests++;
      #ifdef USE_RECOVERY
        whalCtrl_CheckHwStatus(pHealthMonitor->hHalCtrl);
      #endif
         */

        /* NOTE: This call is important to update the siteMgr and roamingMgr, so be carefully if you wish to remove it */
        whalCtrl_GetAsynRSSI (pHealthMonitor->hHalCtrl, 
                              (PVOID)healthMonitor_RSSI_CB, 
                              hHealthMonitor, 
                              (PUINT8)&pHealthMonitor->statTable);

        pHealthMonitor->currentKeepAliveCounter++;
        if (pHealthMonitor->currentKeepAliveCounter >= pHealthMonitor->keepAliveIntervals)
        {
            siteMgr_keepAliveSendNullDataTimer (pHealthMonitor->hSiteMgr);
            pHealthMonitor->currentKeepAliveCounter = 0;
        }
    }
}


/***********************************************************************
 *                        healthMonitor_RSSI_CB
 ***********************************************************************
DESCRIPTION:           

INPUT:      

OUTPUT:

RETURN: 

************************************************************************/
void healthMonitor_RSSI_CB (TI_HANDLE hHealthMonitor)
{
    healthMonitor_t *pHealthMonitor = (healthMonitor_t*)hHealthMonitor;
    /* Update Rx signal in currBss in order to check if roaming trigger on BG scan occured  */
    /* and to update the site Mgr with the updated RSSI. (Those values are averaged)        */
    currBSS_updateRxSignal (pHealthMonitor->hCurrBss,
                            pHealthMonitor->statTable.snr,
                            pHealthMonitor->statTable.rssi,
                            TRUE);
 
    if (pHealthMonitor->state == HEALTH_MONITOR_STATE_CONNECTED)
    {
        /* TX Power Adjust - if the RSSI is good lower the Tx Power */
        siteMgr_checkTxPower( pHealthMonitor->hSiteMgr );
    }
}


/***********************************************************************
 *                        healthMonitor_sendFailureEvent
 ***********************************************************************
DESCRIPTION:    Entry point for all low level modules to send a failure evrnt

INPUT:          handle - health monitor handle
                failureEvent - the error

OUTPUT:

RETURN:    

************************************************************************/
void healthMonitor_sendFailureEvent (TI_HANDLE hHealthMonitor, failureEvent_e failureEvent)
{
    healthMonitor_t *pHealthMonitor = (healthMonitor_t*)hHealthMonitor;

    /* Check the recovery process is already running */
    if (pHealthMonitor->failureEvent < MAX_FAILURE_EVENTS)
    {
        WLAN_REPORT_WARNING (pHealthMonitor->hReport, SITE_MGR_MODULE_LOG,
                             ("%s: recovery process is already running\n", __FUNCTION__));
    }

    /* Recovery is performed only if this trigger is enabled in the .INI file */
    else if (TRUE == pHealthMonitor->recoveryTriggerEnabled[failureEvent])
    {
        pHealthMonitor->failureEvent = failureEvent;
        /* 
         * NOTE: start timer with minimum expiry for recovery will start
         *       from the top of the stack 
         */
        os_timerStart (pHealthMonitor->hOs, pHealthMonitor->hFailTimer, 1, FALSE);
    }
}


/***********************************************************************
 *                        healthMonitor_proccessFailureEvent
 ***********************************************************************
DESCRIPTION:    this is the central error function - will be passed as call back 
                to the TnetWDriver modules. it will parse the error and dispatch the 
                relevant action (recovery or not) 

INPUT:          handle - health monitor handle
                failureEvent - the error

OUTPUT:

RETURN:    

************************************************************************/
void healthMonitor_proccessFailureEvent (TI_HANDLE hHealthMonitor)
{
    healthMonitor_t *pHealthMonitor = (healthMonitor_t*)hHealthMonitor;

    /* Check failure event validity */
    if (pHealthMonitor->failureEvent < MAX_FAILURE_EVENTS)
    {
        pHealthMonitor->recoveryTriggersNumber[pHealthMonitor->failureEvent] ++;

        WLAN_OS_REPORT (("***** recovery trigger: %s *****\n", sRecoveryTriggersNames[pHealthMonitor->failureEvent]));

        recoveryMgr_recoveryProcess(pHealthMonitor->hRecoveryMgr);/* CE20 */

        pHealthMonitor->failureEvent = (UINT32)NO_FAILURE;
    }
    else
    {
        WLAN_REPORT_ERROR (pHealthMonitor->hReport, SITE_MGR_MODULE_LOG,
                           ("%s: unsupported failure event = %d\n", 
                           pHealthMonitor->failureEvent));
    }    
}


/***********************************************************************
 *                        healthMonitor_printFailureEvents
 ***********************************************************************
DESCRIPTION:

INPUT:

OUTPUT:

RETURN:
************************************************************************/
void healthMonitor_printFailureEvents(TI_HANDLE hHealthMonitor)
{
  #ifdef TI_DBG
    healthMonitor_t  *pHealthMonitor = (healthMonitor_t*)hHealthMonitor;
    int i;

    WLAN_OS_REPORT(("-------------- STA Health Failure Statistics ---------------\n"));
    WLAN_OS_REPORT(("FULL RECOVERY PERFORMED    = %d\n", pHealthMonitor->numOfRecoveryPerformed));
    for (i = 0; i < MAX_FAILURE_EVENTS; i++)
    {
        WLAN_OS_REPORT(("%27s= %d\n",
                        sRecoveryTriggersNames[ i ], pHealthMonitor->recoveryTriggersNumber[ i ]));
    }
    WLAN_OS_REPORT(("Maximum number of commands in mailbox queue = %d\n",whalCtrl_getMaxNumberOfCommandsInQueue(pHealthMonitor->hHalCtrl)));
    WLAN_OS_REPORT(("Health Test Perfomrd       = %d\n", pHealthMonitor->numOfHealthTests));
    WLAN_OS_REPORT(("\n"));

  #ifdef USE_RECOVERY
    whalCtrl_PrintHwStatus(pHealthMonitor->hHalCtrl);
  #endif

  #endif /* TI_DBG */
}

