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
 *   MODULE:  whalRecovery.c
 *   PURPOSE: Handle Recovery and ACI in the Hal
 *
 *	Recovery
 *		CheckHwStatus - called periodically from the CORE to check the following triggers:
 *			Consecutive fcs error
 *			CheckMailbox - call interrogate with callback (only print)
 *			RxFreeMem register image
 *			TxQueue 
 *			TxQueue 
 *		Mailbox error - call the CORE failure event MBOX_FAILURE
 *		Event mailbox
 *			MacStatus event - receive the register value for CONS_FCS_ERR and Rx_FREE_MEM triggers
 *			Health event - device error callback to CORE  
 *		PowerCtrl timeout fail - call the CORE failure event HW_AWAKE_FAILURE
 *		StopHal - called from the CORE as first step in the recovery process
 *		Reconfig - called from the CORE as second step in the recovery process
 *	LNA/ACI
 *		ACI event - get SCR_PAD8 value, accumulate it locally for later read/write by the CORE
 *  Rx reset
 *		API to access the RX_RESET register
 *
 *	Issues
 *		Add API to 
 *		Move the whalCtrl_hwStatus_t from HwCtrl into Recovery object
 *		Remove CheckMailbox in CheckHwStatus
 *		Device error is not enabled
 *
 ****************************************************************************/

#include "whalCommon.h"
#include "whalCtrl_api.h"
#include "whalCtrl.h"
#include "whalSecurity.h"
#include "eventMbox_api.h" 
#include "CmdQueue_api.h"
#include "whalBus_Api.h"
#include "shmBus.h"
#include "whalHwAccess.h"
#include "TNETW_Driver.h"
#include "FwEvent_api.h"

#ifdef USE_RECOVERY

/* Aci Indication Callback */
/* typedef void (*AciIndicationCB_t)(TI_HANDLE handle, char* str , UINT32 strLen);*/


void whalCtrl_HealthReoprt_CB(TI_HANDLE hWhalCtrl, char* Report , UINT32 strLen);
void whalCtrl_MacStatus_CB(TI_HANDLE hWhalCtrl, char* Status , UINT32 strLen);
int  whalCtrl_PeriodicCheckMailboxCb(TI_HANDLE hWhalCtrl, UINT16 MboxStatus,char *InterrogateParamsBuf);

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_RegisterCallbacks
 *
 * Input    : 
 * Output   :
 * Process  :
 * Register all health/sanityCheck/Recovery callbacks 
 * --------------------------------------------------
 *		DeviceError - CORE callback for full recovery
 *		HealthReport - Event Mbox callback for extracting device error
 *		MacStatus - Event Mbox callback for checkHwStatus FCS_ERR, RX_FREE_MEM regs
 *		AciIndication - Event Mbox callback for accumulate SCR_PAD8 image
 *		Mailbox error - Mailbox queue callback for case of timeout/error  
 *		Power control error - PowerMngr callback in case of power ctrl timeout 
 *		Failure event - CORE Callback in case of MailboxError or PowerCtrl error 
 *			
 * Note(s)  : Done 
 * -----------------------------------------------------------------------------
 */
int whalCtrl_RegisterErrorsCallbacks(TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL 			*pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
	/*FW events : BUS_ERROR and DEVICE_ERROR*/
	pWhalCtrl->HealthReportCB_CB = whalCtrl_HealthReoprt_CB;
	pWhalCtrl->HealthReportCB_handle = pWhalCtrl;

	eventMbox_RegisterEventCB (pWhalCtrl->hEventMbox, HAL_EVENT_HEALTH_REPORT,(void*)pWhalCtrl->HealthReportCB_CB, pWhalCtrl);
	/* The health report event will be enabled in the ConfigHw stage */
	


	/*FW events : FCS_ERROR and HEALTH_TEST_OK*/
	pWhalCtrl->MacStatusCB_CB = whalCtrl_MacStatus_CB;
	pWhalCtrl->MacStatusCB_CB_handle = pWhalCtrl;
	
	eventMbox_RegisterEventCB(pWhalCtrl->hEventMbox, HAL_EVENT_MAC_STATUS, (void*)pWhalCtrl->MacStatusCB_CB, pWhalCtrl);
	eventMbox_EvUnMask(pWhalCtrl->hEventMbox, HAL_EVENT_MAC_STATUS);

	
	/* Register For Error Of Mailbox in case of timeout */
	CmdQueue_RegisterForErrorCB(((TnetwDrv_t*)pWhalCtrl->hTNETW_Driver)->hCmdQueue, (void *)whalCtrl_CheckMailboxCb, hWhalCtrl);
	
#ifdef HW_ACCESS_SDIO
	TNETWIF_RegisterBusFailureEventCB(((whalBus_T *)(pWhalCtrl->pHwCtrl->hWhalBus))->hTNETWIF,
                                                 (void *)whalCtrl_HealthReoprt_CB,pWhalCtrl);
#endif

	return OK;
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_CheckHwStatus
 *
 * Input    : 
 * Output   :
 * Process  :
 *		CheckFcsError
 *			Test the register CONS_FCR_ERR value that came periodically in event
 *			Try to recover by setting RX_RESET 
 *			Disable full recovery by StaDk_4.1 team !!!!!!!!!!!
 *		CheckMailbox
 *			Call interrogate (with callback) to StationId IE
 *			The callback is just for print
 *			There is no need for this trigger because of the event mechanizm
 *		CheckRxFreeMem
 *			Test the register RX_FREE_MEM value that came periodically in event
 *		CheckTxQueue
 *			Check the queue is not empty and no tx complete 
 *		CheckRxQueue
 *			Disabled 
 *			
 * Note(s)  : Done 
 * -----------------------------------------------------------------------------
 */
int whalCtrl_CheckHwStatus(TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL 			*pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
	
#if 0
	WLAN_REPORT_FATAL_ERROR(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
					("whalCtrl_CheckHwStatus: Start test\n"));
#endif
	/* 
	 * Trigger the FW health test command and wait for results. 
	 * -------------------------------------------------------
	 */
    whal_hwCtrl_healthCheck(pWhalCtrl->pHwCtrl);

	return OK;
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_RecoveryEnded
 *
 * Input    : 
 * Output   :
 * Process  :
 *		aanouce all the modules about the end of the recovery proccess.
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
void whalCtrl_RecoveryEnded(TI_HANDLE hWhalCtrl)
{
	WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

	/*Change the State of the mboxQueue and the interrupt Module and 
	After recovery we should enable back all interrupts according to the last interrupt shadow mask*/
	whalCtrl_exitFromInitMode(hWhalCtrl);
    
    /* 
    Indicates the MboxQueue that Reconfig Ended in Order To Call the CallBacks
	That Was saved before the recovery process started 
	*/
	CmdQueue_EndReconfig(((TnetwDrv_t*)pWhalCtrl->hTNETW_Driver)->hCmdQueue);
	
	WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
							 ("whalCtrl_ReConfig: End  (%d)\n", os_timeStampMs(pWhalCtrl->hOs)));
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_ReConfigCb
 *
 * Input    : 
 * Output   :
 * Process  :
 *      Do firmware download 
 *      Run the firmware
 *      Configure stage (ConfigHw)
 *      Re-Join if needed
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
static int whalCtrl_ReConfigCb (TI_HANDLE hWhalCtrl, TI_STATUS status)
{
    WHAL_CTRL      *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    WlanParams_T   *pWlanParams = whal_ParamsGetWlanParams (pWhalCtrl->pWhalParams);
    whalParamInfo_t param;
	

    if (status != OK)
    {
        WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
            ("whalCtrl_ReConfig: whal_hwCtrl_ConfigHw failed\n"));
        return NOK;
    }

    whalSecur_HwEncDecrEnable (pWhalCtrl->pWhalSecurity, 1);

    /* Re-config roaming thresholds */
    /* SNR */
	whal_hwCtrl_SetSNRParams(pWhalCtrl->pHwCtrl, &pWhalCtrl->pWhalParams->WlanParams.roamTriggers);
    
    /* RSSI */
    whalCtrl_SetRSSIParamsCmd (pWhalCtrl, &pWhalCtrl->pWhalParams->WlanParams.roamTriggers);
    /* Max consecutive NACK */
    whalCtrl_SetMaxTxRetryParamsCmd (pWhalCtrl, &pWhalCtrl->pWhalParams->WlanParams.roamTriggers);
    /* Out of sync */
    whalCtrl_SetBssLossTsfThresholdParamsCmd (pWhalCtrl, &pWhalCtrl->pWhalParams->WlanParams.roamTriggers);


  #ifdef DO_CALIBRATION_IN_DRIVER
    /* Perform single calibration for APP scan usage. */
    whalCtrl_sendRadioAction (pWhalCtrl, MANUAL_CALIB);
  #endif /* DO_CALIBRATION_IN_DRIVER */

    /* Join (use the local parameters), otherwise the core will reconnect */
    if (pWlanParams->bJoin)
    {
        /* Set TxRatePolicy */
        param.paramType = HAL_CTRL_TX_RATE_CLASS_PARAMS;
        param.content.pTxRatePlicy = &pWhalCtrl->pWhalParams->BssInfoParams.TxRateClassParams;
    
        whalCtrl_SetParam (pWhalCtrl->pHwCtrl, &param);
        
        status = (TI_STATUS)whalCtrl_ReJoinBss (hWhalCtrl);
        if (status != OK)
        {
            WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                ("whalCtrl_ReConfig: whalCtrl_ReJoinBss failed\n"));
            return NOK;
        }
        whal_hwCtrl_AidSet (pWhalCtrl->pHwCtrl, pWhalCtrl->pWhalParams->WlanParams.Aid);       
    
        /* Slot time must be setting after doing join */
        whal_hwCtrl_SetSlotTime (pWhalCtrl->pHwCtrl, (slotTime_e)pWhalCtrl->pWhalParams->WlanParams.SlotTime);               

        /* Re-config security keys, default key Id and encryption/decryption control to the FW */
        if (whalSecur_KeysReconfig (pWhalCtrl->pWhalSecurity) != OK)
        {
            WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                ("whalCtrl_ReConfig: ## whalSecur_KeysReconfig failed\n"));
            return NOK;
        }
    }

    return OK;
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_ReConfig
 *
 * Input    : 
 * Output   :
 * Process  :
 *      Do firmware download 
 *      Run the firmware
 *      Configure stage (ConfigHw)
 *      Re-Join if needed
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_ReConfig (TI_HANDLE hWhalCtrl, int DoReJoin)
{
    WHAL_CTRL    *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    WlanParams_T *pWlanParams = whal_ParamsGetWlanParams (pWhalCtrl->pWhalParams);
    int           Stt;
    
    if (!pWlanParams->RecoveryEnable)
    {
        WLAN_OS_REPORT(("Recovery is disabled in registry, abort recovery process\n"));
        return OK;
    }
    
  #if 0
    /* L.M. PATCH for card eject */
    if (!whalBus_FwCtrl_isCardIn (pWhalCtrl->pHwCtrl->hWhalBus))
    {
        WLAN_REPORT_REPLY (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                          ("whalCtrl_ReConfig: card was removed => not proceeding\n"));
        return OK;
    }
  #endif/*_WINDOWS*/

    /*
     * Initiate the wlan hardware (FW download).
     * -----------------------------------------
     */
    WLAN_REPORT_INFORMATION (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                             ("whalCtrl_ReConfig: Start(%d)\n", os_timeStampMs (pWhalCtrl->hOs)));
    Stt = whal_hwCtrl_Initiate (pWhalCtrl->pHwCtrl);
    if (Stt != OK)
    {
        WLAN_REPORT_ERROR (pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
            ("whalCtrl_ReConfig: whal_hwCtrl_Initiate failed\n"));
        return NOK;
    }

    /* Configure the WLAN hardware */
    Stt = whal_hwCtrl_ConfigHw (pWhalCtrl->pHwCtrl, (void *)whalCtrl_ReConfigCb, hWhalCtrl, TRUE);
    
    return OK;
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_CheckMailboxCb
 *
 * Input    : 
 * Output   :
 * Process  :
 *		When the command mailbox queue identify mailbox error or timeout, it will call 
 *		to this function to handle the error.
 *		Call the CORE callback with MBOX_FAILURE type to do the recovery
 *		
 * Note(s)  : Done 
 * -----------------------------------------------------------------------------
 */
int whalCtrl_CheckMailboxCb(TI_HANDLE hWhalCtrl,UINT16 MboxStatus, char *InterrogateParamsBuf)
{

	WHAL_CTRL 			*pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
	whalCtrl_hwStatus_t *pHwStatus = &pWhalCtrl->pHwCtrl->HwStatus;

	if(MboxStatus != OK)
	{
		++pHwStatus->NumMboxFailures;
        WLAN_REPORT_WARNING(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
        ("whalCtrl_PeriodicCheckMailboxCb: Periodic intorregate check - Command mailbox failure was occur \n errors failure # %d "
			,pHwStatus->NumMboxFailures));

		/* Indicating Upper Layer about Mbox Error */
		pWhalCtrl->FailureEvent_CB(pWhalCtrl->FailureEvent_CB_handle,MBOX_FAILURE);
	}

	return OK;
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_MacStatus_CB
 *			: Callback from the EventMbox in case Mac status Event Occur 
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  : Done 
 * -----------------------------------------------------------------------------
 */
void whalCtrl_MacStatus_CB(TI_HANDLE hWhalCtrl, char* Status , UINT32 strLen)
{
	WHAL_CTRL 	*pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
	UINT32		currFcsCounter;
	

   /* The FCS error is updated by the MacStatus event from the firmware */
	currFcsCounter = (*(UINT32*)Status);

	WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
			("whalCtrl_MacStatus_CB: Mac Status report currFcsCounter=%d LastConsFcsCounter=%d\n", 
             currFcsCounter));

	/* This event shouldn't be received, so it is not passed on to anybody */
}

#define		HEALTH_REPORT_DEVICE_ERROR_BIT  BIT_0
#define     HEALTH_REPORT_HOST2FW_SEQ_ERROR BIT_1


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_HealthReoprt_CB
 *			: Callback from the EventMbox in case HealthReport Event Occur 
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  : Done 
 * -----------------------------------------------------------------------------
 */

void whalCtrl_HealthReoprt_CB(TI_HANDLE hWhalCtrl, char* Report , UINT32 strLen)
{
	WHAL_CTRL 	*pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
	UINT16		HealthReport = *((UINT16*)Report);
	
	if(HealthReport & HEALTH_REPORT_DEVICE_ERROR_BIT)
	{ 
		WLAN_REPORT_ERROR(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
							("whalCtrl_HealthReoprt_CB: Firmware reports about DEVICE_ERROR, handle by callback"));
		/* Indicating Upper Layer about Device Error */
		pWhalCtrl->FailureEvent_CB(pWhalCtrl->FailureEvent_CB_handle, DEVICE_ERROR);
	}
    else
        if(HealthReport & HEALTH_REPORT_HOST2FW_SEQ_ERROR)
			/* For USB Health Check */
        {
            WLAN_REPORT_ERROR(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                                ("whalCtrl_HealthReoprt_CB: Firmware reports about HOST2FW_SEQ_ERROR"));
            /* This shall invoke softice, if occure we will handle this case locally*/
           /*  ASSERT(0);*/
        }
        else
			if (HealthReport & HEALTH_REPORT_BUS_ERROR) 
			{
				WLAN_REPORT_ERROR(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
									("whalCtrl_HealthReoprt_CB: Low level bus driver reported bus error. Performing hardware reset of the bus\n"));

				pWhalCtrl->FailureEvent_CB(pWhalCtrl->FailureEvent_CB_handle, BUS_ERROR);
			}
		else
        {
            WLAN_REPORT_ERROR(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,
                                ("whalCtrl_HealthReoprt_CB: UnKnown Health report ID 0x%x ", (int)HealthReport));
        }
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_resetMacRx
 *
 * Input    : 
 * Output   : 
 * Process  : Reset the Rx.
 * Note(s)  : Done 
 * -----------------------------------------------------------------------------
 */
int whalCtrl_resetMacRx (TI_HANDLE hWhalCtrl)
{
	WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;	
	
	WLAN_REPORT_ERROR(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,("whalCtrl_resetMacRx:\n"));
	
	return whal_HwCtrl_resetMacRx (pWhalCtrl->pHwCtrl);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_PrintHwStatus
 *
 * Input    : 
 * Output   :
 * Process  :
 *		Print the Recovery status
 * Note(s)  : Done 
 * -----------------------------------------------------------------------------
 */
int whalCtrl_PrintHwStatus(TI_HANDLE hWhalCtrl)
{
#ifdef REPORT_LOG

    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
	whalCtrl_hwStatus_t *pHwStatus = &pWhalCtrl->pHwCtrl->HwStatus;

    WLAN_OS_REPORT(("--------------- whalCtrl_PrintHwStatus ---------------\n\n"));
	WLAN_OS_REPORT(("NumMboxErrDueToPeriodicBuiltInTestCheck = %d\n", pHwStatus->NumMboxErrDueToPeriodicBuiltInTestCheck));
	WLAN_OS_REPORT(("NumMboxFailures = %d\n", pHwStatus->NumMboxFailures));

#endif
	
	return OK;
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_ResetBusAfterHardBoot
 *
 * Input    : 
 * Output   :
 * Process  :
 *		Restart the buss access layer after hard boot. 
 * Note(s)  : Done 
 * -----------------------------------------------------------------------------
 */
void whalCtrl_ResetBusAfterHardBoot(TI_HANDLE hWhalCtrl)
{
	
#ifdef HW_ACCESS_SDIO
	WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
	whalBus_ReConfig( pWhalCtrl->hWhalBus );
#endif

}

#endif /* USE_RECOVERY */


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_LNAControl
 *
 * Input    : LNAControlField, 0=> Turn Off , 1=> Turn On
 * Output   : 
 * Process  :
 *		Call the mailbox (need to be called directly without HwCtrl)
 * Note(s)  : Done 
 * -----------------------------------------------------------------------------
 */
int whalCtrl_LNAControl (TI_HANDLE hWhalCtrl, UINT8 LNAControlField)
{
	WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;	
	
	WLAN_REPORT_INFORMATION(pWhalCtrl->hReport, HAL_CTRL_MODULE_LOG,("whalCtrl_LNAControl: Set LAN to %d\n",LNAControlField));
	
	return whal_HwCtrl_LNAControl (pWhalCtrl->pHwCtrl, LNAControlField);
}

