/** \file PowerMgr.c
 *  \brief This is the PowerMgr module implementation.
 *  \author Assaf Azulay
 *  \date 24-Oct-2005
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

/****************************************************************************
 *                                                                          *
 *   MODULE:  PowerMgr                                                      *
 *   PURPOSE: PowerMgr Module implementation.                               *
 *                                                                          *
 ****************************************************************************/

#include "osTIType.h"
#include "osApi.h"
#include "paramOut.h"
#include "report.h"
#include "PowerMgr.h"
#include "PowerMgr_API.h"
#include "TrafficMonitorAPI.h"
#include "qosMngr_API.h"
#include "siteMgrApi.h"
#include "SoftGeminiApi.h"

/*****************************************************************************
 **         Defines                                                         **
 *****************************************************************************/
#define DEFAULT_LISTEN_INTERVAL (1)

#define BET_DISABLE 0
#define BET_ENABLE  1

/*****************************************************************************
 **         Private Function prototypes                                      **
 *****************************************************************************/

static void         powerSaveCompleteCB(TI_HANDLE hPowerMgr,UINT8 PSMode,UINT8 transStatus);
static void         PowerMgrTMThresholdCrossCB( TI_HANDLE hPowerMgr, UINT32 cookie );
static void         powerMgrDisableThresholdsIndications(TI_HANDLE hPowerMgr);
static void         powerMgrEnableThresholdsIndications(TI_HANDLE hPowerMgr);
static void         powerMgrStartAutoPowerMode(TI_HANDLE hPowerMgr);
static void         powerMgrRetryPsTimeout(TI_HANDLE hPowerMgr);
static void         powerMgrPowerProfileConfiguration(TI_HANDLE hPowerMgr, PowerMgr_PowerMode_e desiredPowerMode);
static TI_STATUS    powerMgrSendMBXWakeUpConditions(TI_HANDLE hPowerMgr,UINT8 listenInterval, PowerMgr_TnetWakeOn_e tnetWakeupOn);
static TI_STATUS    powerMgrNullPacketRateConfiguration(TI_HANDLE hPowerMgr);
static PowerMgr_PowerMode_e powerMgrGetHighestPriority(TI_HANDLE hPowerMgr);
static void         PowerMgr_setDozeModeInAuto(TI_HANDLE hPowerMgr,PowerMgr_PowerMode_e dozeMode);
static void         PowerMgrConfigBetToFw( TI_HANDLE hPowerMgr, UINT32 cookie );
static void         PowerMgr_PsPollFailureCB( TI_HANDLE hPowerMgr );
static void         powerMgr_PsPollFailureTimeout(TI_HANDLE hPowerMgr);
#ifdef CPU_LOAD
	static void 		powerMgr_CpuLoadTimeout(TI_HANDLE hPowerMgr);
#endif
static void powerMgr_SGSetUserDesiredwakeUpCond( TI_HANDLE hPowerMgr );


/*****************************************************************************
 **         Public Function prototypes                                      **
 *****************************************************************************/


/****************************************************************************************
 *                        PowerMgr_create                                                           *
 ****************************************************************************************
DESCRIPTION: Creates the object of the power Manager. 
                performs the following:
                -   Allocate the Power Manager handle
                -   Creates the retry timer
                                                                                                                   
INPUT:          - hOs - Handle to OS        
OUTPUT:     
RETURN:     Handle to the Power Manager module on success, NULL otherwise
****************************************************************************************/
TI_HANDLE PowerMgr_create(TI_HANDLE hOs)
{

    PowerMgr_t * pPowerMgr = NULL;
    pPowerMgr = (PowerMgr_t*) os_memoryAlloc (hOs, sizeof(PowerMgr_t));
    if ( pPowerMgr == NULL )
    {
        WLAN_OS_REPORT(("PowerMgr_create - Memory Allocation Error!\n"));
        return NULL;
    }

    os_memoryZero (hOs, pPowerMgr, sizeof(PowerMgr_t));

    pPowerMgr->hOS = hOs;

    /*create the timers */
    pPowerMgr->hRetryPsTimer = os_timerCreate(pPowerMgr->hOS,
                                              powerMgrRetryPsTimeout,
                                              pPowerMgr);

    pPowerMgr->hPsPollFailureTimer = os_timerCreate(pPowerMgr->hOS,
                                            powerMgr_PsPollFailureTimeout,
                                            pPowerMgr);

	#ifdef CPU_LOAD
		pPowerMgr->hCpuLoadTimer = os_timerCreate(pPowerMgr->hOS,
                                              powerMgr_CpuLoadTimeout,
                                              pPowerMgr);
	#endif


    if ( (pPowerMgr->hPsPollFailureTimer == NULL) || (pPowerMgr->hRetryPsTimer == NULL) )
    {
        WLAN_OS_REPORT(("PowerMgr_create - Error in creating timer!\n"));
        PowerMgr_destroy(pPowerMgr);
        return NULL;
    }

    return pPowerMgr;

}


/****************************************************************************************
*                        powerSrv_destroy                                                          *
****************************************************************************************
DESCRIPTION: Destroy the object of the power Manager.
               -   delete Power Manager alocation
               -   call the destroy function of the timer
                                                                                                                  
INPUT:          - hPowerMgr - Handle to the Power Manager   
OUTPUT:     
RETURN:    TI_STATUS - OK on success else NOK.
****************************************************************************************/
TI_STATUS PowerMgr_destroy(TI_HANDLE hPowerMgr)
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;

    if ( pPowerMgr->hRetryPsTimer != NULL )
    {
        os_timerDestroy(pPowerMgr->hOS,
                        pPowerMgr->hRetryPsTimer);
    }

    if ( pPowerMgr->hPsPollFailureTimer != NULL )
    {
        os_timerDestroy(pPowerMgr->hOS,
            pPowerMgr->hPsPollFailureTimer);
    }
    os_memoryFree(pPowerMgr->hOS , pPowerMgr , sizeof(PowerMgr_t));

    return OK;
}


/****************************************************************************************
*                        PowerMgr_init                                                         *
****************************************************************************************
DESCRIPTION: Power Manager init function, called in configure phase.
                                                                                                                 
INPUT:          - hPowerMgr             - Handle to the Power Manager
          - hReport           - Handle to report.
          - hSiteMgr          - Handle to site manager
          - trafficMonitor        - Handle to Trrafic monitor
          - pPowerMgrInitParams - initi parameters


OUTPUT:     
RETURN:    TI_STATUS - OK on success else NOK.
****************************************************************************************/
TI_STATUS PowerMgr_init(    TI_HANDLE               hPowerMgr,
                            TI_HANDLE               hMacServices,
                            TI_HANDLE               hReport,
                            TI_HANDLE               hSiteMgr,
                            TI_HANDLE               hWhalCtrl,
                            TI_HANDLE               hTrafficMonitor,
                            TI_HANDLE               hSoftGemini, 
                            PowerMgrInitParams_t *  pPowerMgrInitParams)
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;
    UINT8 index;
    /* used to initialize the Traffic Monitor for Auto Ps events */
    TrafficAlertRegParm_t tmRegParam;
    TI_STATUS status;

    pPowerMgr->hReport      = hReport;
    pPowerMgr->hTrafficMonitor = hTrafficMonitor;
    pPowerMgr->hSiteMgr         = hSiteMgr;
    pPowerMgr->hWhalCtrl        = hWhalCtrl;
    pPowerMgr->hMacServices= hMacServices;
    pPowerMgr->hSoftGemini = hSoftGemini;
    pPowerMgr->psEnable         = FALSE;

    /* init power management options */
    pPowerMgr->beaconListenInterval = pPowerMgrInitParams->beaconListenInterval;
    pPowerMgr->dtimListenInterval = pPowerMgrInitParams->dtimListenInterval;
    pPowerMgr->defaultPowerLevel =  pPowerMgrInitParams->defaultPowerLevel;
    pPowerMgr->PowerSavePowerLevel =  pPowerMgrInitParams->PowerSavePowerLevel;
    pPowerMgr->powerMngPriority  = POWER_MANAGER_USER_PRIORITY;
    pPowerMgr->maxFullBeaconInterval = pPowerMgrInitParams->MaximalFullBeaconReceptionInterval;
    pPowerMgr->PsPollDeliveryFailureRecoveryPeriod = pPowerMgrInitParams->PsPollDeliveryFailureRecoveryPeriod;

    /*
     set AUTO PS parameters 
     */
    pPowerMgr->autoModeInterval = pPowerMgrInitParams->autoModeInterval;
    pPowerMgr->autoModeActiveTH = pPowerMgrInitParams->autoModeActiveTH;
    pPowerMgr->autoModeDozeTH = pPowerMgrInitParams->autoModeDozeTH;
    pPowerMgr->autoModeDozeMode = pPowerMgrInitParams->autoModeDozeMode;

    /*
     register threshold in the traffic monitor.
     */

	pPowerMgr->betEnable = pPowerMgrInitParams->BetEnable; /* save BET enable flag for CLI configuration */
	pPowerMgr->betTrafficEnable = FALSE;                   /* starting without BET */

    /* BET thresholds */
    /* general parameters */
    tmRegParam.Context = pPowerMgr;
    tmRegParam.TimeIntervalMs = BET_INTERVAL_VALUE;
    tmRegParam.Trigger = TRAFF_EDGE;
    tmRegParam.MonitorType = TX_RX_ALL_802_11_DATA_FRAMES;
    tmRegParam.CallBack = PowerMgrConfigBetToFw;

    /* BET enable event */
    tmRegParam.Direction = TRAFF_DOWN;
    tmRegParam.Threshold = pPowerMgrInitParams->BetEnableThreshold;
	pPowerMgr->BetEnableThreshold = pPowerMgrInitParams->BetEnableThreshold;
    tmRegParam.Cookie = (UINT32)BET_ENABLE;
    pPowerMgr->betEnableTMEvent = TrafficMonitor_RegEvent(hTrafficMonitor,
                                                             &tmRegParam,
                                                             FALSE);
    /* BET disable event */
    tmRegParam.Direction = TRAFF_UP;
    tmRegParam.Threshold = pPowerMgrInitParams->BetDisableThreshold;
	pPowerMgr->BetDisableThreshold = pPowerMgrInitParams->BetDisableThreshold;
    tmRegParam.Cookie = (UINT32)BET_DISABLE;
    pPowerMgr->betDisableTMEvent = TrafficMonitor_RegEvent(hTrafficMonitor,
                                                             &tmRegParam,
                                                             FALSE);

    if ( (pPowerMgr->betDisableTMEvent == NULL) ||
         (pPowerMgr->betEnableTMEvent == NULL) )
    {
        WLAN_REPORT_INIT(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                         ("PowerMgr_init - TM - ERROR registering BET events - ABROTING init!\n"));
        return NOK;
    }
    /*
    set the events as resets for one another
    */
    status = TrafficMonitor_SetRstCondition(hTrafficMonitor,
                                            pPowerMgr->betDisableTMEvent,
                                            pPowerMgr->betEnableTMEvent,
                                            TRUE);
    if ( status != OK )
    {
        WLAN_REPORT_INIT(pPowerMgr->hReport,
                         POWER_MANAGER_MODULE_LOG,
                         ("PowerMgr_init - PowerMgr_init - ERROR binding BET events - ABROTING init!\n"));
        return NOK;
    }

    /* general parameters */
    tmRegParam.Context = pPowerMgr;
    tmRegParam.Cookie = (UINT32)POWER_MODE_ACTIVE;
    tmRegParam.TimeIntervalMs = pPowerMgr->autoModeInterval;
    tmRegParam.Trigger = TRAFF_EDGE;
    tmRegParam.MonitorType = TX_RX_ALL_802_11_DATA_FRAMES;

    /* Active threshold */
    tmRegParam.CallBack = PowerMgrTMThresholdCrossCB;
    tmRegParam.Direction = TRAFF_UP;
    tmRegParam.Threshold = pPowerMgr->autoModeActiveTH;
    pPowerMgr->passToActiveTMEvent = TrafficMonitor_RegEvent(hTrafficMonitor,
                                                             &tmRegParam,
                                                             FALSE);
    /* Doze threshold */
    tmRegParam.Direction = TRAFF_DOWN;
    tmRegParam.Threshold = pPowerMgr->autoModeDozeTH;
    tmRegParam.Cookie = (UINT32)POWER_MODE_SHORT_DOZE; /* diffrentiation between long / short doze is done at the 
                                                          CB, according to configuration at time of CB invokation */
    pPowerMgr->passToDozeTMEvent = TrafficMonitor_RegEvent(hTrafficMonitor,
                                                           &tmRegParam,
                                                           FALSE);

    if ( (pPowerMgr->passToActiveTMEvent == NULL) ||
         (pPowerMgr->passToDozeTMEvent == NULL) )
    {
        WLAN_REPORT_INIT(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                         ("PowerMgr_init - PowerMgr_init - ERROR registering Auto mode events - ABROTING init!\n"));
        return NOK;
    }

    /*
    set the events as resets for one another
    */
    status = TrafficMonitor_SetRstCondition(hTrafficMonitor,
                                            pPowerMgr->passToActiveTMEvent,
                                            pPowerMgr->passToDozeTMEvent,
                                            TRUE);
    if ( status != OK )
    {
        WLAN_REPORT_INIT(pPowerMgr->hReport,
                         POWER_MANAGER_MODULE_LOG,
                         ("PowerMgr_init - PowerMgr_init - ERROR binding Auto mode events - ABROTING init!\n"));
        return NOK;
    }

    /*
    configure the initialize power mode
    */
    pPowerMgr->desiredPowerModeProfile = pPowerMgrInitParams->powerMode;
    for ( index = 0;index < POWER_MANAGER_MAX_PRIORITY;index++ )
    {
        pPowerMgr->powerMngModePriority[index].powerMode = pPowerMgr->desiredPowerModeProfile;
        pPowerMgr->powerMngModePriority[index].priorityEnable = FALSE;
    }
    pPowerMgr->powerMngModePriority[POWER_MANAGER_USER_PRIORITY].priorityEnable = TRUE;
    /* set the defualt power policy */
    MacServices_powerAutho_PowerPolicyUpdate(pPowerMgr->hMacServices, pPowerMgr->defaultPowerLevel);

    /* Register and Enable the PsPoll failure */
    whalCtrl_EventMbox_RegisterForEvent( pPowerMgr->hWhalCtrl, 
        HAL_EVENT_PSPOLL_DELIVERY_FAILURE,
        (void *)PowerMgr_PsPollFailureCB, 
        hPowerMgr );
    whalCtrl_EventMbox_Enable( pPowerMgr->hWhalCtrl, HAL_EVENT_PSPOLL_DELIVERY_FAILURE );


    WLAN_REPORT_INIT(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                     ("PowerMgr_init - PowerMgr Initialized\n"));

    return OK;
}

/****************************************************************************************
 *                        PowerMgr_startPS                                                          *
 ****************************************************************************************
DESCRIPTION: Start the power save algorithm of the driver and also the 802.11 PS.
                                                                                                                   
INPUT:          - hPowerMgr             - Handle to the Power Manager

OUTPUT:     
RETURN:    TI_STATUS - OK or PENDING on success else NOK.\n
****************************************************************************************/
TI_STATUS PowerMgr_startPS(TI_HANDLE hPowerMgr)
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;
	int frameCount;

    WLAN_REPORT_INFORMATION(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                            ("PowerMgr_startPS - called\n"));


    if ( pPowerMgr->psEnable == TRUE )
    {
        WLAN_REPORT_ERROR(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                          ("PowerMgr_startPS - PS mechanism is already Enable! Aborting psEnable=%d !\n",
                           pPowerMgr->psEnable));
        /*
        this is a FATAL ERROR of the power manager!
        already enable power-save! thus return OK, but there is an error in the upper
        layer that call tp PowerMgr_startPS() twice - should know that power-save
        is already enable therefor print the Error message.
        or
        the state machine while NOT in PS can be only in ACTIVE state and in some cases in
        PS_PENDING state. therefore the state machine is out of sync from it logic!
        */
        return OK;
    }

    pPowerMgr->psEnable = TRUE;
    /*set the correct rate after connection*/
    powerMgrNullPacketRateConfiguration(hPowerMgr);
    /*
    if in auto mode then need to refer to the threshold cross indication from the traffic monitor,
    else it need to refer to the configured power mode profile from the user.
    */

    pPowerMgr->desiredPowerModeProfile = powerMgrGetHighestPriority(hPowerMgr);

    if ( pPowerMgr->desiredPowerModeProfile == POWER_MODE_AUTO )
    {
        powerMgrStartAutoPowerMode(hPowerMgr);
    }
    else /*not auto mode - according to the current profle*/
    {
        powerMgrPowerProfileConfiguration(hPowerMgr, pPowerMgr->desiredPowerModeProfile);
    }

	
	if (pPowerMgr->betEnable)
	{
		TrafficMonitor_StartEventNotif(pPowerMgr->hTrafficMonitor,
									   pPowerMgr->betEnableTMEvent);

		TrafficMonitor_StartEventNotif(pPowerMgr->hTrafficMonitor,
									   pPowerMgr->betDisableTMEvent);

	
		frameCount = TrafficMonitor_GetFrameBandwidth(pPowerMgr->hTrafficMonitor);
	
		if (frameCount < pPowerMgr->BetEnableThreshold) 
		{
			pPowerMgr->betTrafficEnable = TRUE;
		}
		else if (frameCount > pPowerMgr->BetDisableThreshold) 
		{
			pPowerMgr->betTrafficEnable = FALSE;
        }

		PowerMgrConfigBetToFw(hPowerMgr,pPowerMgr->betTrafficEnable);
	}
    
    return OK;
}


/****************************************************************************************
 *                        PowerMgr_stopPS                                                           *
 ****************************************************************************************
DESCRIPTION: Stop the power save algorithm of the driver and also the 802.11 PS.
                                                                                                                               
INPUT:          - hPowerMgr             - Handle to the Power Manager

OUTPUT:     
RETURN:    TI_STATUS - OK or PENDING on success else NOK.\n
****************************************************************************************/
TI_STATUS PowerMgr_stopPS(TI_HANDLE hPowerMgr)
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;
    /*TI_STATUS status;*/

    WLAN_REPORT_INFORMATION(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                            ("PowerMgr_stopPS - called\n"));

    if ( pPowerMgr->psEnable == FALSE )
    {
        WLAN_REPORT_INFORMATION(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                                ("PowerMgr_stopPS - PS is already Disable! Aborting!\n"));
        /*
        Print Info message incase callng PowerMgr_stopPS() more than once in a row, without
        calling to PowerMgr_startPS() in the middle.
        this will return with OK and not doing the Stop action!
        */
        return OK;
    }

    pPowerMgr->psEnable = FALSE;
    os_timerStop(pPowerMgr->hOS, pPowerMgr->hRetryPsTimer);

    /* Check if PsPoll work-around is currently enabled */
    if ( pPowerMgr->powerMngModePriority[POWER_MANAGER_PS_POLL_FAILURE_PRIORITY].priorityEnable == TRUE)
    {
        os_timerStop(pPowerMgr->hOS, pPowerMgr->hPsPollFailureTimer);
        /* Exit the PsPoll work-around */
        powerMgr_PsPollFailureTimeout( hPowerMgr );
    }

    if ( pPowerMgr->desiredPowerModeProfile == POWER_MODE_AUTO )
    {
        powerMgrDisableThresholdsIndications(hPowerMgr);
    }

    MacServices_powerSrv_SetPsMode(pPowerMgr->hMacServices, POWER_SAVE_OFF, FALSE,NULL, NULL, NULL);

    /* set the power policy of the system */
    MacServices_powerAutho_PowerPolicyUpdate(pPowerMgr->hMacServices, pPowerMgr->defaultPowerLevel);

	if (pPowerMgr->betEnable)
	{
		TrafficMonitor_StopEventNotif(pPowerMgr->hTrafficMonitor,
									  pPowerMgr->betEnableTMEvent);

		TrafficMonitor_StopEventNotif(pPowerMgr->hTrafficMonitor,
									  pPowerMgr->betDisableTMEvent);
	}

    return OK;
}


/****************************************************************************************
 *                        PowerMgr_getPsStatus                                                          *
 ****************************************************************************************
DESCRIPTION: returns the 802.11 power save status (enable / disable).
                                                                                                                               
INPUT:          - hPowerMgr             - Handle to the Power Manager

OUTPUT:     
RETURN:    BOOLEAN - TRUE if enable else FALSE.\n
****************************************************************************************/
BOOLEAN PowerMgr_getPsStatus(TI_HANDLE hPowerMgr)
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;
    return  MacServices_powerSrv_getPsStatus(pPowerMgr->hMacServices);

}


/****************************************************************************************
 *                        PowerMgr_setPowerMode                                                         *
 ****************************************************************************************
DESCRIPTION: Configure of the PowerMode profile (auto / active / short doze / long doze).
                                                                                                                               
INPUT:          - hPowerMgr             - Handle to the Power Manager
            - thePowerMode      - the requested power mode (auto / active / short doze / long doze).
OUTPUT:     
RETURN:    TI_STATUS - OK on success else NOK.\n
****************************************************************************************/
TI_STATUS PowerMgr_setPowerMode(TI_HANDLE hPowerMgr)
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;
    PowerMgr_PowerMode_e powerMode;

    /*in this way we will run with the highest priority that is enabled*/
    powerMode = powerMgrGetHighestPriority(hPowerMgr);

    /* sanity checking */
    if ( powerMode >= POWER_MODE_MAX)
    {
        WLAN_REPORT_ERROR(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                          ("PowerMgr_setPowerMode - unknown parameter: %d\n", powerMode));
        return NOK;
    }

    WLAN_REPORT_INFORMATION( pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                             ("PowerMgr_setPowerMode, power mode = %d\n", powerMode) );

    if ( pPowerMgr->desiredPowerModeProfile != powerMode )
    {
        PowerMgr_PowerMode_e previousPowerModeProfile;
        previousPowerModeProfile = pPowerMgr->desiredPowerModeProfile;
        pPowerMgr->desiredPowerModeProfile = powerMode;

        if ( pPowerMgr->desiredPowerModeProfile == POWER_MODE_AUTO )
        {
            if ( pPowerMgr->psEnable == TRUE )
            {
                powerMgrStartAutoPowerMode(hPowerMgr);
            }

            /*
            the transitions of state will be done according to the events from the
            traffic monitor - therefor abort and wait event from the traffic monitor.
            */
            return OK;
        }
        else if ( previousPowerModeProfile == POWER_MODE_AUTO )
        {
            /*
            if the old power mode is AUTO and the new power mode is NOT then need
            to disable the thresholds indications from the traffic monitor.
            */
            powerMgrDisableThresholdsIndications(hPowerMgr);
        }
        if ( pPowerMgr->psEnable == TRUE )
        {
            powerMgrPowerProfileConfiguration(hPowerMgr, powerMode);
        }
    }
    else
    {
        /*
        the power mode is already configure to the module - don't need to do anything!
        */
        WLAN_REPORT_WARNING(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                            ("PowerMgr_setPowerMode - desiredPowerModeProfile == thePowerMode (=%d), ABORTING!\n",
                             powerMode));
    }

    return OK;
}


/****************************************************************************************
 *                        PowerMgr_reloadPowerMode                                      *
 ****************************************************************************************
DESCRIPTION: Sends the current power mode configuration to the firmware.                                                                                        
INPUT:       - hPowerMgr             - Handle to the Power Manager
OUTPUT:     
RETURN:   
****************************************************************************************/
void PowerMgr_reloadPowerMode(TI_HANDLE hPowerMgr)
{
    PowerMgr_t * pPowerMgr = (PowerMgr_t *) hPowerMgr;

    pPowerMgr->desiredPowerModeProfile = powerMgrGetHighestPriority(hPowerMgr);

    if (pPowerMgr->psEnable == TRUE)
    {
        if (pPowerMgr->desiredPowerModeProfile == POWER_MODE_AUTO)
        {
            powerMgrStartAutoPowerMode(hPowerMgr);
        }
        else
        {
            powerMgrPowerProfileConfiguration(hPowerMgr, pPowerMgr->desiredPowerModeProfile);
        }
    }
}


/****************************************************************************************
 *                        PowerMgr_setDozeModeInAuto                                    *
 ****************************************************************************************
DESCRIPTION: Configure the doze mode (short-doze / long-doze) that auto mode will toggle between doze vs active.                                                                                        
INPUT:      - hPowerMgr             - Handle to the Power Manager
            - dozeMode      - the requested doze mode when Mgr is in Auto mode (short-doze / long-doze)
OUTPUT:     
RETURN:   
****************************************************************************************/
void PowerMgr_setDozeModeInAuto(TI_HANDLE hPowerMgr, PowerMgr_PowerMode_e dozeMode)
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;
    PowerMgr_PowerMode_e powerMode = powerMgrGetHighestPriority(hPowerMgr);

    /* check if we are trying to configure the same Doze mode */
    if ( dozeMode != pPowerMgr->autoModeDozeMode )
    {
        WLAN_REPORT_INFORMATION(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                                ("PowerMgr_setDozeModeInAuto - autoModeDozeMode == %d \n",
                                 dozeMode));

        pPowerMgr->autoModeDozeMode = dozeMode;

        /* in case we are already in Auto mode, we have to set the wake up condition MIB */
        if ( powerMode == POWER_MODE_AUTO )
        {
            if ( dozeMode == POWER_MODE_SHORT_DOZE )
            {
                if ( pPowerMgr->beaconListenInterval > 1 )
                {
                    powerMgrSendMBXWakeUpConditions(hPowerMgr,pPowerMgr->beaconListenInterval,TNET_WAKE_ON_N_BEACON);       
                }
                else
                {
                    powerMgrSendMBXWakeUpConditions(hPowerMgr,pPowerMgr->beaconListenInterval,TNET_WAKE_ON_BEACON);     
                }
            }
            else  /* POWER_MODE_LONG_DOZE */
            {
                if ( pPowerMgr->dtimListenInterval > 1 )
                {
                    powerMgrSendMBXWakeUpConditions(hPowerMgr,pPowerMgr->dtimListenInterval,TNET_WAKE_ON_N_DTIM);       
                }
                else
                {
                    powerMgrSendMBXWakeUpConditions(hPowerMgr,pPowerMgr->dtimListenInterval,TNET_WAKE_ON_DTIM);     
                }
            }

            WLAN_REPORT_INFORMATION(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                                    ("PowerMgr_setDozeModeInAuto - already in Auto\n"));
        }
    }
    else
    {
        WLAN_REPORT_WARNING(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                            ("PowerMgr_setDozeModeInAuto - autoModeDozeMode == %d (same same ...)\n",
                             dozeMode));
    }
}

/****************************************************************************************
 *                        PowerMgr_getPowerMode                                                         *
 ****************************************************************************************
DESCRIPTION: Get the current PowerMode of the PowerMgr module. 
                                                                                                                               
INPUT:          - hPowerMgr             - Handle to the Power Manager
OUTPUT:     
RETURN:    PowerMgr_PowerMode_e - (auto / active / short doze / long doze).\n
****************************************************************************************/
PowerMgr_PowerMode_e PowerMgr_getPowerMode(TI_HANDLE hPowerMgr)
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;

    return pPowerMgr->desiredPowerModeProfile;
}


/**
 * \author Yossi Peery
 * \date 2-August-2004\n
 * \brief reset the power manager module due to recovry event.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the PowerMgr object.\n
 * Return Value: TI_STATUS - OK on success else NOK.\n
 * \b Description:\n
 * simulate the stop power save function without writing to the HW. just doing the
 * logic parts of stop power save from the power manager to it state machine.
 * the power controller and it state machine are reset in the whalCtrl recovry proccess.
*/
TI_STATUS PowerMgr_swReset(TI_HANDLE hPowerMgr)
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;

    WLAN_REPORT_INFORMATION(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                            ("PowerMgr_swReset() called\n"));


    pPowerMgr->psEnable = FALSE;

    if ( pPowerMgr->desiredPowerModeProfile == POWER_MODE_AUTO )
    {
        powerMgrDisableThresholdsIndications(hPowerMgr);
    }
    os_timerStop(pPowerMgr->hOS, pPowerMgr->hRetryPsTimer);
    return OK;
}

TI_STATUS powerMgr_setParam(TI_HANDLE thePowerMgrHandle,
                            paramInfo_t *theParamP)
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)thePowerMgrHandle;

    switch ( theParamP->paramType )
    {
    case POWER_MGR_POWER_MODE:
        pPowerMgr->powerMngModePriority[theParamP->content.powerMngPowerMode.powerMngPriority].powerMode
                        = theParamP->content.powerMngPowerMode.PowerMode;
        PowerMgr_setPowerMode(thePowerMgrHandle);
		if (pPowerMgr->betEnable)
		{
			PowerMgrConfigBetToFw(thePowerMgrHandle, pPowerMgr->betTrafficEnable );
		}
        break;

    case POWER_MGR_DISABLE_PRIORITY:
        pPowerMgr->powerMngModePriority[theParamP->content.powerMngPriority].priorityEnable = FALSE;
        PowerMgr_setPowerMode(thePowerMgrHandle);
        break;

    case POWER_MGR_ENABLE_PRIORITY:
        pPowerMgr->powerMngModePriority[theParamP->content.powerMngPriority].priorityEnable = TRUE;
        PowerMgr_setPowerMode(thePowerMgrHandle);
        break;

    case POWER_MGR_POWER_LEVEL_PS:
        pPowerMgr->PowerSavePowerLevel = theParamP->content.PowerSavePowerLevel;
        /* set the power policy of the system */
		if (pPowerMgr->psEnable && (pPowerMgr->desiredPowerModeProfile!=POWER_MODE_AUTO))
		{	/* If Connected  and not in AUTO mode */
			MacServices_powerAutho_PowerPolicyUpdate(pPowerMgr->hMacServices, pPowerMgr->PowerSavePowerLevel);
		}
        break;

    case POWER_MGR_POWER_LEVEL_DEFAULT:
        pPowerMgr->defaultPowerLevel = theParamP->content.DefaultPowerLevel;
		/* set the power policy of the system */
		if (!pPowerMgr->psEnable)
		{	/* If not Connected */
			MacServices_powerAutho_PowerPolicyUpdate(pPowerMgr->hMacServices, pPowerMgr->defaultPowerLevel);
		}
        break;

    case POWER_MGR_POWER_LEVEL_DOZE_MODE:
        PowerMgr_setDozeModeInAuto(thePowerMgrHandle,theParamP->content.powerMngDozeMode);
		if (pPowerMgr->betEnable)
		{
			PowerMgrConfigBetToFw(thePowerMgrHandle, pPowerMgr->betTrafficEnable );
		}
        break;

    default:
        WLAN_REPORT_ERROR(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                          ("PowerMgr_setParam - ERROR - Param is not supported, %d\n\n",
                           theParamP->paramType));

        return PARAM_NOT_SUPPORTED;
    }

    return OK;
}



TI_STATUS powerMgr_getParam(TI_HANDLE thePowerMgrHandle,
                            paramInfo_t *theParamP)
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)thePowerMgrHandle;

    switch ( theParamP->paramType )
    {
    case POWER_MGR_POWER_MODE:
        theParamP->content.PowerMode = PowerMgr_getPowerMode(thePowerMgrHandle);
        break;

    case POWER_MGR_POWER_LEVEL_PS:
        theParamP->content.PowerSavePowerLevel = pPowerMgr->PowerSavePowerLevel;
        break;

    case POWER_MGR_POWER_LEVEL_DEFAULT:
        theParamP->content.DefaultPowerLevel = pPowerMgr->defaultPowerLevel;
        break;

    case POWER_MGR_POWER_LEVEL_DOZE_MODE:
        theParamP->content.powerMngDozeMode = pPowerMgr->autoModeDozeMode;
        break;

    default:
        WLAN_REPORT_ERROR(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                          ("PowerMgr_getParam - ERROR - Param is not supported, %d\n\n",
                           theParamP->paramType));
        return PARAM_NOT_SUPPORTED;
    }

    return OK;
}


/*****************************************************************************
 **         Private Function prototypes                                     **
 *****************************************************************************/


/****************************************************************************************
 *                        powerSaveCompleteCB                                                       *
 ****************************************************************************************
DESCRIPTION: Callback for the Power server complete - gets the result of the request 
              for PS or exit PS.
                                                                                                                               
INPUT:          - hPowerMgr             - Handle to the Power Manager
            - PSMode
            - trasStatus            - result string form the FW.
OUTPUT:     
RETURN:    void.\n
****************************************************************************************/
static void powerSaveCompleteCB(TI_HANDLE hPowerMgr,UINT8 PSMode,UINT8 transStatus)
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;

    WLAN_REPORT_INFORMATION( pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                             ("powerSaveCompleteCB, statud = %d\n", transStatus) );

    /* Handling the event*/
    switch ( (EventsPowerSave_e)transStatus )
    {
    case ENTER_POWER_SAVE_FAIL:
    case EXIT_POWER_SAVE_FAIL:
        pPowerMgr->lastPsTransaction = transStatus;
        os_timerStart(pPowerMgr->hOS,
                      pPowerMgr->hRetryPsTimer,
                      RE_ENTER_PS_TIMEOUT,
                      FALSE);
        break;

    case ENTER_POWER_SAVE_SUCCESS:
        if ( (pPowerMgr->lastPowerModeProfile == POWER_MODE_SHORT_DOZE) ||
             (pPowerMgr->lastPowerModeProfile == POWER_MODE_LONG_DOZE)  ||
			 (pPowerMgr->lastPowerModeProfile == POWER_MODE_PS_ONLY))
        {
            MacServices_powerAutho_PowerPolicyUpdate(pPowerMgr->hMacServices, pPowerMgr->PowerSavePowerLevel);
        }

    case EXIT_POWER_SAVE_SUCCESS:
        break;

    default:
        WLAN_REPORT_ERROR( pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                           ("powerSaveCompleteCB: invliad status: %d\n", transStatus) );
        break;
    }
}

/**
 * \author Ronen Kalish\n
 * \date 30-Aug-2006\n
 * \brief Power manager callback fro TM event notification
 *
 * Function Scope \e Public.\n
 * \param hPowerMgr - handle to the power maanger object.\n
 * \param cookie - values supplied during event registration (active / doze).\n
 */
static void PowerMgrTMThresholdCrossCB( TI_HANDLE hPowerMgr, UINT32 cookie )
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;

    WLAN_REPORT_INFORMATION(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                            ("PowerMgrTMThresholdCrossCB - TM notified threshold crossed, cookie: %d\n",
                             cookie));

    /* sanity cehcking - TM notifications should only be received when PM is enabled and in auto mode */
    if ( (pPowerMgr->psEnable == TRUE) && (pPowerMgr->desiredPowerModeProfile == POWER_MODE_AUTO) )
    {
        switch ((PowerMgr_PowerMode_e)cookie)
        {
        case POWER_MODE_ACTIVE:
            powerMgrPowerProfileConfiguration( hPowerMgr, POWER_MODE_ACTIVE );
            break;

        /* threshold crossed down - need to enter configured doze mode */
        case POWER_MODE_SHORT_DOZE:
            powerMgrPowerProfileConfiguration( hPowerMgr, pPowerMgr->autoModeDozeMode );
            break;

        default:
            WLAN_REPORT_ERROR( pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                               ("PowerMgrTMThresholdCrossCB: TM notification with invalid cookie: %d!\n",
                                cookie) );
            break;
        }
    }
    else
    {
        WLAN_REPORT_ERROR( pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                           ("PowerMgrTMThresholdCrossCB: TM motification when psEnable is :%d or desired profile is: %d\n",
                            pPowerMgr->psEnable, pPowerMgr->desiredPowerModeProfile) );
    }

}

/****************************************************************************************
*                        powerMgrDisableThresholdsIndications                                           *
*****************************************************************************************
DESCRIPTION: This will send a disable message to the traffic monitor,
                 to stop sending indications on threshold pass.

                                                                                                                              
INPUT:          - hPowerMgr             - Handle to the Power Manager
OUTPUT:     
RETURN:    void.\n
****************************************************************************************/
static void powerMgrDisableThresholdsIndications(TI_HANDLE hPowerMgr)
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;

    /*
    auto is not a static/fix state, else its a dynamic state that flows between
    the 3 static/fix states: active, short-doze and long-doze.
    */
    TrafficMonitor_StopEventNotif(pPowerMgr->hTrafficMonitor,
                                  pPowerMgr->passToActiveTMEvent);

    TrafficMonitor_StopEventNotif(pPowerMgr->hTrafficMonitor,
                                  pPowerMgr->passToDozeTMEvent);

}


/****************************************************************************************
*                        powerMgrEnableThresholdsIndications                                            *
*****************************************************************************************
DESCRIPTION: TThis will send an enable message to the traffic monitor,
                to start sending indications on threshold pass.

                                                                                                                              
INPUT:          - hPowerMgr             - Handle to the Power Manager
OUTPUT:     
RETURN:    void.\n
****************************************************************************************/
static void powerMgrEnableThresholdsIndications(TI_HANDLE hPowerMgr)
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;

    WLAN_REPORT_INFORMATION( pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                             ("powerMgrEnableThresholdsIndications called\n") );
    /*
    auto is not a static/fix state, but rather a dynamic state that flows between
    the 3 static/fix states: active, short-doze and long-doze.
    */
    TrafficMonitor_StartEventNotif(pPowerMgr->hTrafficMonitor,
                                   pPowerMgr->passToActiveTMEvent);

    TrafficMonitor_StartEventNotif(pPowerMgr->hTrafficMonitor,
                                   pPowerMgr->passToDozeTMEvent);

}


/****************************************************************************************
*                        powerMgrStartAutoPowerMode                                                 *
*****************************************************************************************
DESCRIPTION: configure the power manager to enter into AUTO power mode. 
             The power manager will deside what power level will be applied 
             acording to the traffic monitor.
                                                                                                                              
INPUT:          - hPowerMgr             - Handle to the Power Manager
OUTPUT:     
RETURN:    void.\n
****************************************************************************************/
static void powerMgrStartAutoPowerMode(TI_HANDLE hPowerMgr)
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;
    int frameCount;

    frameCount = TrafficMonitor_GetFrameBandwidth(pPowerMgr->hTrafficMonitor);

    WLAN_REPORT_INFORMATION( pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                             ("powerMgrStartAutoPowerMode: Starting auto power mode,"
                              "frame count=%d, TH=%d\n", frameCount, pPowerMgr->autoModeActiveTH) );

    /*Activates the correct profile*/
    if ( frameCount >= pPowerMgr->autoModeActiveTH )
    {
        powerMgrPowerProfileConfiguration(hPowerMgr, POWER_MODE_ACTIVE);
    }
    else
    {
        powerMgrPowerProfileConfiguration(hPowerMgr, pPowerMgr->autoModeDozeMode);

    }
    /* Activates the Trafic monitoe Events*/        
    powerMgrEnableThresholdsIndications(hPowerMgr);
}

/****************************************************************************************
*                        powerMgrRetryPsTimeout                                                     *
*****************************************************************************************
DESCRIPTION: Retry function if a PS/exit PS request failed
                                                                                                                              
INPUT:          - hPowerMgr             - Handle to the Power Manager
OUTPUT:     
RETURN:    void.\n
****************************************************************************************/
static void powerMgrRetryPsTimeout(TI_HANDLE hPowerMgr)
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;
    TI_STATUS powerStatus;

    WLAN_REPORT_INFORMATION( pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                             ("powerMgrRetryPsTimeout: timer expired.\n") );

    if ( pPowerMgr->lastPsTransaction == ENTER_POWER_SAVE_FAIL )
    {
        powerStatus = MacServices_powerSrv_SetPsMode(pPowerMgr->hMacServices, POWER_SAVE_ON, TRUE , hPowerMgr,powerSaveCompleteCB,NULL);/*NULL as GWSI callback*/
        if ( powerStatus == POWER_SAVE_802_11_IS_CURRENT )
        {
            if ( (pPowerMgr->lastPowerModeProfile == POWER_MODE_SHORT_DOZE) || 
                 (pPowerMgr->lastPowerModeProfile == POWER_MODE_LONG_DOZE)  ||
				 (pPowerMgr->lastPowerModeProfile == POWER_MODE_PS_ONLY))
            {
                MacServices_powerAutho_PowerPolicyUpdate(pPowerMgr->hMacServices, pPowerMgr->PowerSavePowerLevel);              
            }
        }
    }
    else
    {
        powerStatus = MacServices_powerSrv_SetPsMode(pPowerMgr->hMacServices, POWER_SAVE_OFF, TRUE , hPowerMgr,powerSaveCompleteCB,NULL);/*NULL as GWSI callback*/
    }

}


/****************************************************************************************
*                        powerMgrPowerProfileConfiguration                                          *
*****************************************************************************************
DESCRIPTION: This function is the " builder " of the Power Save profiles. 
             acording to the desired Power mode.
                                                                                                                              
INPUT:          - hPowerMgr             - Handle to the Power Manager
OUTPUT:     
RETURN:    void.\n
****************************************************************************************/
static void powerMgrPowerProfileConfiguration(TI_HANDLE hPowerMgr, PowerMgr_PowerMode_e desiredPowerMode)
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;
    TI_STATUS powerStatus;

    os_timerStop(pPowerMgr->hOS, pPowerMgr->hRetryPsTimer);

	pPowerMgr->lastPowerModeProfile = desiredPowerMode;

    switch ( desiredPowerMode )
    {
    case POWER_MODE_AUTO:
        WLAN_REPORT_INFORMATION(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                                ("PowerMode==AUTO - This mode should not be sent to the GWSI - we send AUTO instead\n"));
        break;

    case POWER_MODE_ACTIVE:
        /* set AWAKE through */
        MacServices_powerAutho_PowerPolicyUpdate(pPowerMgr->hMacServices, POWERAUTHO_POLICY_AWAKE);
        powerStatus = MacServices_powerSrv_SetPsMode(pPowerMgr->hMacServices,
                                                     POWER_SAVE_OFF, 
                                                     TRUE , 
                                                     hPowerMgr,
                                                     powerSaveCompleteCB,
                                                     NULL);
        WLAN_REPORT_INFORMATION(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                                ("PowerMode==ACTIVE\n"));
        break;

    case POWER_MODE_SHORT_DOZE:
        if ( pPowerMgr->beaconListenInterval > 1 )
        {
            powerMgrSendMBXWakeUpConditions(hPowerMgr,pPowerMgr->beaconListenInterval,TNET_WAKE_ON_N_BEACON);       
        }
        else
        {
            powerMgrSendMBXWakeUpConditions(hPowerMgr,pPowerMgr->beaconListenInterval,TNET_WAKE_ON_BEACON);     
        }

        powerStatus = MacServices_powerSrv_SetPsMode(pPowerMgr->hMacServices, 
                                                     POWER_SAVE_ON, 
                                                     TRUE , 
                                                     hPowerMgr,
                                                     powerSaveCompleteCB,
                                                     NULL);
        if ( powerStatus == POWER_SAVE_802_11_IS_CURRENT )
        {
            MacServices_powerAutho_PowerPolicyUpdate(pPowerMgr->hMacServices, pPowerMgr->PowerSavePowerLevel);
        }
        WLAN_REPORT_INFORMATION(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                                ("PowerMode==SHORT_DOZE\n"));
        break;

    case POWER_MODE_LONG_DOZE:
        if ( pPowerMgr->dtimListenInterval > 1 )
        {
            powerMgrSendMBXWakeUpConditions(hPowerMgr,pPowerMgr->dtimListenInterval,TNET_WAKE_ON_N_DTIM);       
        }
        else
        {
            powerMgrSendMBXWakeUpConditions(hPowerMgr,pPowerMgr->dtimListenInterval,TNET_WAKE_ON_DTIM);     
        }
        powerStatus = MacServices_powerSrv_SetPsMode(pPowerMgr->hMacServices, 
                                                     POWER_SAVE_ON, 
                                                     TRUE , 
                                                     hPowerMgr,
                                                     powerSaveCompleteCB,
                                                     NULL);
        if ( powerStatus == POWER_SAVE_802_11_IS_CURRENT )
        {
            MacServices_powerAutho_PowerPolicyUpdate(pPowerMgr->hMacServices, pPowerMgr->PowerSavePowerLevel);
        }

        WLAN_REPORT_INFORMATION(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                                ("PowerMode==LONG_DOZE\n"));
        break;

	case POWER_MODE_PS_ONLY:
		/* When in SG PS mode, configure the user desired wake-up condition */
		powerMgr_SGSetUserDesiredwakeUpCond(hPowerMgr);
        powerStatus = MacServices_powerSrv_SetPsMode(pPowerMgr->hMacServices, 
                                                     POWER_SAVE_ON, 
                                                     TRUE , 
                                                     hPowerMgr,
                                                     powerSaveCompleteCB,
                                                     NULL);

		/* In PS_ONLY we will use the last powerAutho policy of SHORT or LONG DOZE or ACTIVE */
		if ( powerStatus == POWER_SAVE_802_11_IS_CURRENT )
		{
			MacServices_powerAutho_PowerPolicyUpdate(pPowerMgr->hMacServices, pPowerMgr->PowerSavePowerLevel);
		}

        WLAN_REPORT_INFORMATION(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                                ("PowerMode==PS_ONLY\n"));
        break;

    default:
        WLAN_REPORT_ERROR(pPowerMgr->hReport,
                          POWER_MANAGER_MODULE_LOG,
                          ("PowerMgr_setWakeUpConfiguration - ERROR - PowerMode - unknown parameter: %d\n",
                           desiredPowerMode));
        return;
    }

}


/****************************************************************************************
*                        powerMgrSendMBXWakeUpConditions                                            *
*****************************************************************************************
DESCRIPTION: Tsend configuration of the power management option that holds in the command
                mailbox inner sturcture.
                                                                                                                              
INPUT:          - hPowerMgr             - Handle to the Power Manager
OUTPUT:     
RETURN:    TI_STATUS - OK on success else NOK.\n
****************************************************************************************/
static TI_STATUS powerMgrSendMBXWakeUpConditions(TI_HANDLE hPowerMgr,
                                                 UINT8 listenInterval,
                                                 PowerMgr_TnetWakeOn_e tnetWakeupOn)
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;
    whalCtrl_powerMgmtConfig_t powerMgmtConfig;
    TI_STATUS status = OK;

    powerMgmtConfig.listenInterval = listenInterval;
    powerMgmtConfig.tnetWakeupOn = tnetWakeupOn;

    WLAN_REPORT_INFORMATION(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                            ("powerMgrSendMBXWakeUpConditions: listenInterval = %d, tnetWakeupOn = %d\n",
                             listenInterval,tnetWakeupOn));

    status = whalCtrl_wakeUpCondition(pPowerMgr->hWhalCtrl,
                                      powerMgmtConfig);
    if ( status != OK )
    {
        WLAN_REPORT_ERROR(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                          ("powerMgrSendMBXWakeUpConditions - Error in wae up condition IE!\n"));
    }
    return status;
}




static TI_STATUS powerMgrNullPacketRateConfiguration(TI_HANDLE hPowerMgr)
{
    paramInfo_t     param;
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;

    param.paramType = SITE_MGR_CURRENT_RATE_PAIR_PARAM;
    if ( siteMgr_getParam(pPowerMgr->hSiteMgr, &param) == OK )
    {
        MacServices_powerSrv_SetRateModulation(pPowerMgr->hMacServices, (UINT16)param.content.siteMgrCurrentRateMask.basicRateMask);
    }
    else
    {
        MacServices_powerSrv_SetRateModulation(pPowerMgr->hMacServices, (DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER));
        WLAN_REPORT_ERROR(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                          ("powerMgrNullPacketRateConfiguration: error - faild to set rate so default was seted!\n"));
    }
    return OK;

}


static PowerMgr_PowerMode_e powerMgrGetHighestPriority(TI_HANDLE hPowerMgr)
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;
    int index;
    for ( index = POWER_MANAGER_MAX_PRIORITY-1;index >= 0;index-- )
    {
        if ( pPowerMgr->powerMngModePriority[index].priorityEnable )
        {

            return pPowerMgr->powerMngModePriority[index].powerMode;
        }

    }

    WLAN_REPORT_ERROR(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                      ("powerMgrGetHighestPriority - error - faild to get highest priority! sefault deseired mode was returned !!!\n"));
    return pPowerMgr->desiredPowerModeProfile;
}

 /****************************************************************************************
 *                        PowerMgr_notifyFWReset															*
 ****************************************************************************************
DESCRIPTION: Notify the object of the power Manager about FW reset (recovery).
				-	call PowerSrv module to Set Ps Mode
				-	call PowerAutho Power Policy Update
				                                                                                                   
INPUT:      	- hPowerMgr - Handle to the Power Manager	
OUTPUT:		
RETURN:    TI_STATUS - OK on success else NOK.
****************************************************************************************/
TI_STATUS PowerMgr_notifyFWReset(TI_HANDLE hPowerMgr)
{
	PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;

	if (pPowerMgr->psEnable)
	{
		if (pPowerMgr->lastPowerModeProfile == POWER_MODE_SHORT_DOZE || 
			pPowerMgr->lastPowerModeProfile == POWER_MODE_LONG_DOZE)
		{
			powerMgrPowerProfileConfiguration(hPowerMgr, pPowerMgr->lastPowerModeProfile);
		}
	}

    return OK;
}

 /****************************************************************************************
 *                        PowerMgrConfigBetToFw															*
 ****************************************************************************************
DESCRIPTION: callback from TM event notification.
				-	call PowerSrv module to Set Ps Mode
				-	call PowerAutho Power Policy Update
				                                                                                                   
INPUT:      	- hPowerMgr - Handle to the Power Manager	
                - BetEnable - cookie:values supplied during event registration
OUTPUT:		
RETURN:    None.
****************************************************************************************/
static void PowerMgrConfigBetToFw( TI_HANDLE hPowerMgr, UINT32 BetEnable )
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;
    UINT8 MaximumConsecutiveET;
    UINT32 listenInterval;
    paramInfo_t param;
    UINT32 beaconInterval;
    UINT32 dtimPeriod;
    PowerMgr_PowerMode_e powerMode;

    param.paramType = SITE_MGR_BEACON_INTERVAL_PARAM;
    siteMgr_getParam(pPowerMgr->hSiteMgr, &param);
    beaconInterval = param.content.beaconInterval;

    param.paramType = SITE_MGR_DTIM_PERIOD_PARAM;
    siteMgr_getParam(pPowerMgr->hSiteMgr, &param);
    dtimPeriod = param.content.siteMgrDtimPeriod;

    /* get actual Power Mode */
    if (pPowerMgr->desiredPowerModeProfile == POWER_MODE_AUTO)
    {
        powerMode = pPowerMgr->autoModeDozeMode;
    }
    else
    {
        powerMode = pPowerMgr->lastPowerModeProfile;
    }

    /* calc ListenInterval */
    if (powerMode == POWER_MODE_SHORT_DOZE)
    {
        listenInterval = beaconInterval * pPowerMgr->beaconListenInterval;
    }
    else if (powerMode == POWER_MODE_LONG_DOZE)
    {
        listenInterval = dtimPeriod * beaconInterval * pPowerMgr->dtimListenInterval;
    }
    else
    {
        listenInterval = beaconInterval;
    }

    if (listenInterval == 0)
    {
        WLAN_REPORT_WARNING(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                                ("PowerMgrConfigBetToFw: listenInterval is ZERO\n"));
        return;
    }

    /* MaximumConsecutiveET = MaximalFullBeaconReceptionInterval / MAX( BeaconInterval, ListenInterval) */
    MaximumConsecutiveET = pPowerMgr->maxFullBeaconInterval / listenInterval;

    WLAN_REPORT_INFORMATION(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                            ("PowerMgrConfigBetToFw:\n \
                            Power Mode = %d\n \
                            beaconInterval = %d\n \
                            listenInterval = %d\n \
                            Bet Enable = %d\n \
                            MaximumConsecutiveET = %d\n",
                            powerMode,
                            beaconInterval,
                            listenInterval,
                            BetEnable,
                            MaximumConsecutiveET));

    pPowerMgr->betTrafficEnable = BetEnable; /* save BET enable flag for CLI configuration */

    whalCtrl_setBetParams (pPowerMgr->hWhalCtrl, BetEnable, MaximumConsecutiveET);

}


/**
 * \date 10-April-2007\n
 * \brief Returns to the configured wakeup condition, when SG protective mode is done
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the PowerMgr object.\n
 * Return Value: void.\n
 */
static void powerMgr_SGSetUserDesiredwakeUpCond( TI_HANDLE hPowerMgr )
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;


    if (pPowerMgr->psEnable) 
    {
      /* set wakeup condition according to user mode power save profile */
      switch ( pPowerMgr->powerMngModePriority[ POWER_MANAGER_USER_PRIORITY ].powerMode )
      {
      case POWER_MODE_AUTO:
        /*set wakeup condition according to doze mode in auto and wakup interval */
        if ( pPowerMgr->autoModeDozeMode == POWER_MODE_SHORT_DOZE )
        {
            /* short doze */
            if ( pPowerMgr->beaconListenInterval > 1 )
            {
                powerMgrSendMBXWakeUpConditions(hPowerMgr,pPowerMgr->beaconListenInterval,TNET_WAKE_ON_N_BEACON);       
            }
            else
            {
                powerMgrSendMBXWakeUpConditions(hPowerMgr,pPowerMgr->beaconListenInterval,TNET_WAKE_ON_BEACON);     
            }
        }
        else
        {
            /* long doze */
            if ( pPowerMgr->dtimListenInterval > 1 )
            {
                powerMgrSendMBXWakeUpConditions(hPowerMgr,pPowerMgr->dtimListenInterval,TNET_WAKE_ON_N_DTIM);       
            }
            else
            {
                powerMgrSendMBXWakeUpConditions(hPowerMgr,pPowerMgr->dtimListenInterval,TNET_WAKE_ON_DTIM);     
            }
        }
        break;

      case POWER_MODE_ACTIVE:
        break;

      case POWER_MODE_SHORT_DOZE:
        if ( pPowerMgr->beaconListenInterval > 1 )
        {
            powerMgrSendMBXWakeUpConditions(hPowerMgr,pPowerMgr->beaconListenInterval,TNET_WAKE_ON_N_BEACON);       
        }
        else
        {
            powerMgrSendMBXWakeUpConditions(hPowerMgr,pPowerMgr->beaconListenInterval,TNET_WAKE_ON_BEACON);     
        }
        break;

      case POWER_MODE_LONG_DOZE:
        if ( pPowerMgr->dtimListenInterval > 1 )
        {
            powerMgrSendMBXWakeUpConditions(hPowerMgr,pPowerMgr->dtimListenInterval,TNET_WAKE_ON_N_DTIM);       
        }
        else
        {
            powerMgrSendMBXWakeUpConditions(hPowerMgr,pPowerMgr->dtimListenInterval,TNET_WAKE_ON_DTIM);     
        }
        break;

      default:
        WLAN_REPORT_ERROR(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG,
                          ("powerMgr_SGExitShortDoze - ERROR - PowerMode for user prioirty is: %d\n",
                           pPowerMgr->powerMngModePriority[ POWER_MANAGER_USER_PRIORITY ].powerMode));
      }
    }/*end of if (psEnable)*/
}



/****************************************************************************************
*                        PowerMgr_PsPollFailureCB															*
****************************************************************************************
DESCRIPTION: Work around to solve AP bad behavior.
         Some old AP's have trouble with Ps-Poll - The solution will be to exit PS for a 
         period of time
				                                                                                               
INPUT:      	- hPowerMgr - Handle to the Power Manager	
OUTPUT:		
RETURN:    
****************************************************************************************/
static void PowerMgr_PsPollFailureCB( TI_HANDLE hPowerMgr )
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;

    if ( pPowerMgr->PsPollDeliveryFailureRecoveryPeriod )
    {
        paramInfo_t param;
             
        WLAN_REPORT_WARNING(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG, 
            ("%s Oh boy, AP is not answering Ps-Poll's. enter active PS for %d Ms\n",
            __FUNCTION__, pPowerMgr->PsPollDeliveryFailureRecoveryPeriod));

        /* Disable Soft Gemini */
        SoftGemini_startPsPollFailure(pPowerMgr->hSoftGemini);

        /*
         * Set the system to Active power save 
         */
        param.paramType = POWER_MGR_POWER_MODE;
        param.content.powerMngPowerMode.PowerMode = POWER_MODE_ACTIVE;
        param.content.powerMngPowerMode.powerMngPriority = POWER_MANAGER_PS_POLL_FAILURE_PRIORITY;
        powerMgr_setParam(hPowerMgr,&param);
        
        param.paramType = POWER_MGR_ENABLE_PRIORITY;
        param.content.powerMngPriority = POWER_MANAGER_PS_POLL_FAILURE_PRIORITY;
        powerMgr_setParam(hPowerMgr,&param);

        /*
         * Set timer to exit the active mode
         */
        os_timerStart(pPowerMgr->hOS,
            pPowerMgr->hPsPollFailureTimer,
            pPowerMgr->PsPollDeliveryFailureRecoveryPeriod,
            FALSE);
    } 
    else    /* Work-around is disabled */
    {
        WLAN_REPORT_WARNING(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG, 
            ("%s Oh boy, AP is not answering Ps-Poll's !!!\n",__FUNCTION__));
    }

}

/****************************************************************************************
*                        powerMgr_PsPollFailureTimeout									*
****************************************************************************************
DESCRIPTION: After the timeout of ps-poll failure - return to normal behavior
				                                                                                               
INPUT:      	- hPowerMgr - Handle to the Power Manager	
OUTPUT:		
RETURN:    
****************************************************************************************/
static void powerMgr_PsPollFailureTimeout(TI_HANDLE hPowerMgr)
{
    PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;
    paramInfo_t param;

    WLAN_REPORT_INFORMATION(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG, 
        ("%s \n",__FUNCTION__));
    
    /* disable Ps-Poll priority */
    param.paramType = POWER_MGR_DISABLE_PRIORITY;
    param.content.powerMngPriority = POWER_MANAGER_PS_POLL_FAILURE_PRIORITY;
    powerMgr_setParam(hPowerMgr,&param);

    /* return to normal Soft Gemini */
    SoftGemini_endPsPollFailure(pPowerMgr->hSoftGemini);
}

void powerMgr_setCpuLoad(TI_HANDLE hPowerMgr, UINT32 uCpuLoad)
{
	#ifdef CPU_LOAD

		PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;
		
		pPowerMgr->uCpuLoad = uCpuLoad;
		WLAN_OS_REPORT(("\n PowerMgr_setCpuLoad - Cpu Load = %d \n",
						   pPowerMgr->uCpuLoad));

		if (pPowerMgr->uCpuLoad > 0) 
		{
			os_timerStart(pPowerMgr->hOS,
						  pPowerMgr->hCpuLoadTimer,
						  RE_ENTER_CPU_LOAD_TIMEOUT,
						  TRUE);
		}
		else
		{
			os_timerStop(pPowerMgr->hOS, pPowerMgr->hCpuLoadTimer);
		}
	#endif
}

#ifdef CPU_LOAD
	/****************************************************************************************
	*                        powerMgr_CpuLoadTimeout									*
	****************************************************************************************
	DESCRIPTION: After the timeout of cpu load - run in idle loop to simulate cpu load
																												   
	INPUT:      	- hPowerMgr - Handle to the Power Manager	
	OUTPUT:		
	RETURN:    
	****************************************************************************************/
	static void powerMgr_CpuLoadTimeout(TI_HANDLE hPowerMgr)
	{
		PowerMgr_t *pPowerMgr = (PowerMgr_t*)hPowerMgr;
		UINT32 uStallTime=0;
		
		WLAN_REPORT_INFORMATION(pPowerMgr->hReport, POWER_MANAGER_MODULE_LOG, 
			("CpuLoadTimeout : cpu= %d\n", pPowerMgr->uCpuLoad));

		#ifdef _WINDOWS
		#else
			while(uStallTime < (pPowerMgr->uCpuLoad * CPU_LOAD_MUL))
			{
				uStallTime++;
			}
		#endif
	}
#endif

