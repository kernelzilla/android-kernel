/** \file configMgr.h
 *  \brief config manager module internal header file
 *
 *  \see configMgr.c
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

/***************************************************************************/
/*                                                                          */
/*    MODULE:   configMgr.h                                                 */
/*    PURPOSE:  Config manager module internal header file                          */
/*                                                                          */
/***************************************************************************/
#ifndef __CONFIG_MGR_H__
#define __CONFIG_MGR_H__

#include "osTIType.h"
#include "paramOut.h"
#include "paramIn.h"
#include "fsm.h"

/* Config manager state machine defintions */
typedef enum
{
    CFG_MGR_STATE_IDLE              = 0,
    CFG_MGR_STATE_RUNNING           = 1,
    CFG_MGR_STATE_STOPPED           = 2,
} stateCfg_e;

typedef struct
{
    paramFunc_t         set; 
    paramFunc_t         get; 
    TI_HANDLE           handle; 
} paramAccess_t;

/* Configuration manager handle */
typedef struct 
{
    UINT8               state;
    fsm_stateMachine_t  *pFsm;

    paramAccess_t       paramAccessTable[MAX_PARAM_MODULE_NUMBER];

    /* SME handles */
    TI_HANDLE           hSmeSm;
    TI_HANDLE           hSiteMgr;
    TI_HANDLE           hConn;

    /* MLME handles */
    TI_HANDLE           hMlmeSm;
    TI_HANDLE           hAuth;
    TI_HANDLE           hAssoc;

    /* Data handles */
    TI_HANDLE           hRxData;
    TI_HANDLE           hTxData;
    TI_HANDLE           hCtrlData;
    
    /* Traffic Monitor */
    TI_HANDLE           hTrafficMon;

    /* RSN handle */
    TI_HANDLE           hRsn;

    /* HAL handles */
    TI_HANDLE           hHalCtrl;

    /* Network Control */
    TI_HANDLE           hRegulatoryDomain;
    TI_HANDLE           hMeasurementMgr;
    TI_HANDLE           hSoftGemini;

    /* EXC Manager*/
    TI_HANDLE           hExcMngr;

    /* Roaming Manager */
    TI_HANDLE           hRoamingMngr;

    /* QOS Manager */
    TI_HANDLE           hQosMngr;

    /* Utils handles */
    TI_HANDLE           hReport;
    TI_HANDLE           hMemMgr;

    /* OS handle */
    TI_HANDLE           hOs;

    /* Power handles */
    TI_HANDLE           hPowerMgr;
    TI_HANDLE           hPowerSrv;

    /* Event Handler Handles */
    TI_HANDLE           hEvHandler;

    /* AP Connection Handles */
    TI_HANDLE           hAPConnection;

    /* Current BSS Handles */
    TI_HANDLE           hCurrBss;

    /* SwitchChannel Handles */
    TI_HANDLE           hSwitchChannel;

    /* Services handles */
    TI_HANDLE           hSCR;               /* SCR */
    TI_HANDLE           hMacServices;       /* MacServices */

    /* Management handles */
    TI_HANDLE           hScanCncn;          /* Scan Concentrator */

    /* Application handles */
    TI_HANDLE           hScanMngr;          /* Scan Manager */

    /* Health Monitor */
    TI_HANDLE           hHealthMonitor;  

	TI_HANDLE           hRecoveryMgr;

    /* Core Callbacks pointers to be used in the config phase */
    /*TI_HANDLE         hcoreCallbacks;*/

    TI_HANDLE           hTnetwDrv;

    /* Pointer to configuration parameters */
    initTable_t        *pInitTable;

    /* Pointer to driver's mac address */
    macAddress_t       *pMacAddr;

} configMgr_t;

#endif /* __CONFIG_MGR_H__ */
