/** \file healthMonitor.c
 *  \brief Firmware Recovery Mechanizem
 *
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
/*      MODULE:     healthMonitor.h                                         */
/*      PURPOSE:    Driver interface to OS abstraction layer                */
/*                                                                          */
/****************************************************************************/

#ifndef HEALTHMONITOR_H
#define HEALTHMONITOR_H

#include "paramOut.h"
#include "whalCtrl_api.h"


/* Config manager state machine defintions */
typedef enum healthMonitorState_e
{
    HEALTH_MONITOR_STATE_DISCONNECTED = 0,
    HEALTH_MONITOR_STATE_CONNECTED 

} healthMonitorState_e;


typedef struct
{
    /* handles to other modules */
    TI_HANDLE            hOs;                    /**< handle to the OS object */
    TI_HANDLE            hReport;                /**< handle to the report object */
    TI_HANDLE            hHalCtrl;               /**< handle to the HAL CTRL object */
    TI_HANDLE            hSiteMgr;               /**< handle to the site manager object */
    TI_HANDLE            hScr;                   /**< handle to the SCR object */
    TI_HANDLE            hSoftGemini;            /**< handle to the Soft Gemini object */
    TI_HANDLE            hTnetwDrv;              /**< handle to the TNETW driver object */
    TI_HANDLE            hMemMgr;                /**< handle to the memory manager object */
    TI_HANDLE            hConfigMgr;             /**< handle to the config manager object */
    TI_HANDLE            hTxData;                /**< handle to the TX data object */
    TI_HANDLE            hHealtheCheckTimer;     /**< periodic health check timer handle */
    TI_HANDLE            hCurrBss;               /**< handle to the currBss object */
    TI_HANDLE            hFailTimer;             /**< failure event timer */
    TI_HANDLE            hRsn;                   /**< handle to the RSN */
    TI_HANDLE            hRecoveryMgr;           /**< handle to the Recovery Mgr object */
    
    /* Management variables */
    UINT32               numOfHealthTests;       /**< number of health tests performed counter */
    healthMonitorState_e state;                  /**< health monitor state */
    BOOL                 bFullRecoveryEnable;    /**< full recovery enable flag */
    BOOL                 bSuspended;             /**< suspend periodic test flag */
    UINT32               timerInterval;          /**< health check interval */
    BOOL                 bRunSoftRecovery;       /**< soft recovery flag */
    BOOL                 recoveryTriggerEnabled [MAX_FAILURE_EVENTS];
                                                 /**< recovery enable flags per trigger type */
    UINT32               failureEvent;           /**< current recovery trigger */
    UINT32               keepAliveIntervals;     /**< number of health monitor timer intervals at which keep alive should be sent */
    UINT32               currentKeepAliveCounter;/**< counting how many timer intervals had passed w/o a keep alive */

    /* Recoveries Statistics */
    UINT32               recoveryTriggersNumber [MAX_FAILURE_EVENTS];                                                  
                                                 /**< Number of times each recovery trigger occured */
    UINT32               numOfRecoveryPerformed; /**< number of recoveries performed */
    
    ACXRoamingStatisticsTable_t statTable;       /**< needed by TX Power Adjust, to retrieve current rssi when beacon filter in ON */

} healthMonitor_t;



TI_HANDLE healthMonitor_create         (TI_HANDLE hOs);
TI_STATUS healthMonitor_config         (TI_HANDLE hHealthMonitor, 
                                        TI_HANDLE hReport,
                                        TI_HANDLE hHalCtrl,
                                        TI_HANDLE hSiteMgr,
                                        TI_HANDLE hScr,
                                        TI_HANDLE hSoftGemini,
                                        TI_HANDLE hTnetwDrv,
                                        TI_HANDLE hMemMgr,
                                        TI_HANDLE hConfigMgr,
                                        TI_HANDLE hTxData,
                                        TI_HANDLE hCurrBss,
                                        TI_HANDLE hRsn,
                                        healthMonitorInitParams_t *healthMonitorInitParams,
                                        TI_HANDLE hRecoveryMgr);
TI_STATUS healthMonitor_unload         (TI_HANDLE hHealthMonitor);
void healthMonitor_performTest         (TI_HANDLE hHealthMonitor);
void healthMonitor_setState            (TI_HANDLE hHealthMonitor, healthMonitorState_e state);
void healthMonitor_suspendPeriodicTest (TI_HANDLE hHealthMonitor);
void healthMonitor_resumePeriodicTest  (TI_HANDLE hHealthMonitor);
void healthMonitor_sendFailureEvent    (TI_HANDLE hHealthMonitor, failureEvent_e failureEvent);
void healthMonitor_printFailureEvents  (TI_HANDLE hHealthMonitor);

#endif
