/** \file softGemini.c
 *  \brief BlueTooth-Wlan coexistence module interface
 *
 *  \see softGemini.h
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

/****************************************************************************************************
*																									*
*		MODULE:		softGemini.c																	*
*		PURPOSE:	BlueTooth-Wlan coexistence module interface.							*
*					This module handles all data base (and Fw setting accordingly)					*
*					for Bluetooth-Wlan coexistence implementation.									*
*																						 			*
****************************************************************************************************/

#include "report.h"
#include "osApi.h"
#include "SoftGemini.h"
#include "whalCtrl_api.h"
#include "DataCtrl_Api.h"
#include "scrApi.h"
#include "PowerMgr_API.h"
#include "srcApi.h"
#include "ScanCncnApi.h"
#include "currBss.h"
#include "bssTypes.h"
#include "EvHandler.h"


#define SENSE_MODE_ENABLE		0x01
#define SENSE_MODE_DISABLE		0x00
#define PROTECTIVE_MODE_ON		0x01
#define PROTECTIVE_MODE_OFF		0x00

/********************************************************************************/
/*						Internal functions prototypes.							*/
/********************************************************************************/

static TI_STATUS SoftGemini_setEnableParam(TI_HANDLE hSoftGemini, SoftGeminiEnableModes_e SoftGeminiEnable, BOOL recovery);
static TI_STATUS SoftGemini_setRateParam(TI_HANDLE hSoftGemini, UINT8 *pRates);
static TI_STATUS SoftGemini_setParamsToFW(TI_HANDLE hSoftGemini, SoftGeminiParam_t *SoftGeminiParam);
static void SoftGemini_setConfigParam(TI_HANDLE hSoftGemini, UINT32 *param);
static TI_STATUS SoftGemini_EnableDriver(TI_HANDLE hSoftGemini);
static TI_STATUS SoftGemini_DisableDriver(TI_HANDLE hSoftGemini);
static void SoftGemini_EnableProtectiveMode(TI_HANDLE hSoftGemini);
static void SoftGemini_DisableProtectiveMode(TI_HANDLE hSoftGemini);
static void SoftGemini_RemoveProtectiveModeParameters(TI_HANDLE hSoftGemini);
static void SoftGemini_reconnect(TI_HANDLE hSoftGemini);
static TI_STATUS SoftGemini_SetPS(SoftGemini_t	*pSoftGemini);
static TI_STATUS SoftGemini_unSetPS(SoftGemini_t	*pSoftGemini);

#ifdef REPORT_LOG
static char* SoftGemini_ConvertModeToString(SoftGeminiEnableModes_e SoftGeminiEnable);
#endif

/********************************************************************************/
/*						Interface functions Implementation.						*/
/********************************************************************************/

/************************************************************************
 *                        SoftGemini_SetPSmode									*
 ************************************************************************
DESCRIPTION: SoftGemini module, called by the conn_Infra on connection
				performs the following:
				-	Enables SG if needed
                                -       Enables the SG power mode				                                                                                                   
INPUT:      hSoftGemini -		Handle to SoftGemini		

OUTPUT:		

RETURN:     

************************************************************************/
void SoftGemini_SetPSmode(TI_HANDLE hSoftGemini)
{
	SoftGemini_t *pSoftGemini = (SoftGemini_t *)hSoftGemini;

	if (pSoftGemini)
	{
		if (pSoftGemini->bDriverEnabled) 
		{
			SoftGemini_SetPS(pSoftGemini);
		}
		if (pSoftGemini->bProtectiveMode) 
		{
			SoftGemini_EnableProtectiveMode(hSoftGemini);
		}
	}
	else WLAN_REPORT_ERROR(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("  SoftGemini_SetPSmode() - Error hSoftGemini= NULL \n"));
}

/************************************************************************
 *                        SoftGemini_unSetPSmode									*
 ************************************************************************
DESCRIPTION: SoftGemini module, called by the conn_Infra after disconnecting 
				performs the following:
				-	Disables the SG
                                -       Disables the SG power mode				                                                                                                   
INPUT:      hSoftGemini -		Handle to SoftGemini		

OUTPUT:		

RETURN:     

************************************************************************/
void SoftGemini_unSetPSmode(TI_HANDLE hSoftGemini)
{
	SoftGemini_t *pSoftGemini = (SoftGemini_t *)hSoftGemini;

    if (pSoftGemini)
	{
		if (pSoftGemini->bDriverEnabled) 
		{
			SoftGemini_unSetPS(pSoftGemini);
		}
	}
	else WLAN_REPORT_ERROR(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  (" SoftGemini_unSetPSmode() - Error hSoftGemini= NULL \n"));
}

/************************************************************************
 *                        SoftGemini_create									*
 ************************************************************************
DESCRIPTION: SoftGemini module creation function, called by the config mgr in creation phase 
				performs the following:
				-	Allocate the SoftGemini handle
				                                                                                                   
INPUT:      hOs -			Handle to OS		


OUTPUT:		

RETURN:     Handle to the SoftGemini module on success, NULL otherwise

************************************************************************/
TI_HANDLE SoftGemini_create(TI_HANDLE hOs)
{
	SoftGemini_t			*pSoftGemini = NULL;

	/* allocating the SoftGemini object */
	pSoftGemini = os_memoryAlloc(hOs,sizeof(SoftGemini_t));

	if (pSoftGemini == NULL)
		return NULL;				

	pSoftGemini->hOs = hOs;

	return pSoftGemini;
}

/************************************************************************
 *                        SoftGemini_config						*
 ************************************************************************
DESCRIPTION: SoftGemini module configuration function, called by the config mgr in configuration phase
				performs the following:
				-	Reset & initializes local variables
				-	Init the handles to be used by the module
                                                                                                   
INPUT:      hSoftGemini	-	SoftGemini handle
			List of handles to be used by the module
			pSoftGeminiInitParams	-	Init table of the module.		


OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

TI_STATUS SoftGemini_config(	TI_HANDLE		hSoftGemini,
								TI_HANDLE		hCtrlData,
					  			TI_HANDLE		hHalCtrl,
								TI_HANDLE		hReport,
								TI_HANDLE		hSCR,
								TI_HANDLE		hPowerMgr,
								TI_HANDLE		hConfigMgr,
								TI_HANDLE		hScanCncn,
								TI_HANDLE		hCurrBss,
								TI_HANDLE		hEvHandler,
								SoftGeminiInitParams_t *pSoftGeminiInitParams)
{
	SoftGemini_t *pSoftGemini = (SoftGemini_t *)hSoftGemini;
	TI_STATUS status;

	/*******************/
	/* init Handles */
	/*****************/

	pSoftGemini->hCtrlData  = hCtrlData;
	pSoftGemini->hHalCtrl	= hHalCtrl;
	pSoftGemini->hReport	= hReport;
	pSoftGemini->hSCR       = hSCR;
	pSoftGemini->hPowerMgr  = hPowerMgr;
	pSoftGemini->hConfigMgr = hConfigMgr;
	pSoftGemini->hScanCncn  = hScanCncn;
	pSoftGemini->hCurrBss	= hCurrBss;
	pSoftGemini->hEvHandler= hEvHandler;

	/*************************************/
	/* Getting SoftGemini init Params */
	/***********************************/

	SoftGemini_setRateParam(hSoftGemini,pSoftGeminiInitParams->SoftGeminiRate);
	
	/* read parameters for scan but no need to use them yet, only used when we enable/disable driver */
	pSoftGemini->scanNumOfProbeRequest   = pSoftGeminiInitParams->scanNumOfProbeRequest;
	pSoftGemini->scanCompensationPercent = pSoftGeminiInitParams->scanCompensationPercent;
	pSoftGemini->scanCompensationMaxTime = pSoftGeminiInitParams->scanCompensationMaxTime;
	pSoftGemini->BSSLossCompensationPercent = pSoftGeminiInitParams->BSSLossCompensationPercent;

	os_memoryCopy(hSoftGemini,&pSoftGemini->SoftGeminiParam,&pSoftGeminiInitParams->SoftGeminiParam,sizeof(SoftGeminiParam_t));
	/* Send the configuration to the FW */
	status = SoftGemini_setParamsToFW(hSoftGemini, &pSoftGemini->SoftGeminiParam);

	/*******************************/
    /* register Indication interrupts  */
	/*****************************/

    whalCtrl_EventMbox_RegisterForEvent( pSoftGemini->hHalCtrl, 
                                         HAL_EVENT_SOFT_GEMINI_SENSE,
                                         (void *)SoftGemini_SenseIndicationCB, 
                                         hSoftGemini );
	whalCtrl_EventMbox_RegisterForEvent( pSoftGemini->hHalCtrl, 
                                         HAL_EVENT_SOFT_GEMINI_PREDICTION,
                                         (void *)SoftGemini_ProtectiveIndicationCB, 
                                         hSoftGemini );

	whalCtrl_EventMbox_RegisterForEvent( pSoftGemini->hHalCtrl, 
                                         HAL_EVENT_SOFT_GEMINI_AVALANCHE,
                                         (void *)SoftGemini_AvalancheIndicationCB, 
                                         hSoftGemini );

    whalCtrl_EventMbox_Enable( pSoftGemini->hHalCtrl, HAL_EVENT_SOFT_GEMINI_SENSE );

	whalCtrl_EventMbox_Enable( pSoftGemini->hHalCtrl, HAL_EVENT_SOFT_GEMINI_PREDICTION );

	whalCtrl_EventMbox_Enable( pSoftGemini->hHalCtrl, HAL_EVENT_SOFT_GEMINI_AVALANCHE );

	/* On system initialization SG is disabled but later calls to SoftGemini_setEnableParam() */
	pSoftGemini->bProtectiveMode = FALSE;	
	pSoftGemini->SoftGeminiEnable = SG_DISABLE; 
	pSoftGemini->bDriverEnabled = FALSE;
    pSoftGemini->bPsPollFailureActive = FALSE;

	if ((OK == status) && (pSoftGeminiInitParams->SoftGeminiEnable != SG_DISABLE))
	{	/* called only if different than SG_DISABLE */
		status = SoftGemini_setEnableParam(hSoftGemini, pSoftGeminiInitParams->SoftGeminiEnable, FALSE);
	}

	if (status == OK)
	{
		WLAN_REPORT_INIT(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("  SoftGemini_config() - configured successfully\n"));
	}
	else
	{
		WLAN_REPORT_ERROR(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("  SoftGemini_config() - Error configuring module \n"));
	}

	return status;
}

/************************************************************************
 *                        SoftGemini_destroy							*
 ************************************************************************
DESCRIPTION: SoftGemini module destroy function, called by the config mgr in the destroy phase 
				performs the following:
				-	Free all memory aloocated by the module
                                                                                                   
INPUT:      hSoftGemini	-	SoftGemini handle.		


OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS SoftGemini_destroy(TI_HANDLE hSoftGemini)
{
	SoftGemini_t	*pSoftGemini = (SoftGemini_t *)hSoftGemini;

	if (pSoftGemini != NULL)
	{
		os_memoryFree( pSoftGemini->hOs, (TI_HANDLE)pSoftGemini , sizeof(SoftGemini_t));
	}
	
	return OK;
}


/***********************************************************************
 *                        SoftGemini_setParam									
 ***********************************************************************
DESCRIPTION: SoftGemini set param function, called by the following:
			-	config mgr in order to set a parameter receiving from the OS abstraction layer.
			-	From inside the driver	
                                                                                                   
INPUT:      hSoftGemini	-	SoftGemini handle.
			pParam	-	Pointer to the parameter		

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS SoftGemini_setParam(TI_HANDLE	hSoftGemini,
											paramInfo_t	*pParam)
{
	SoftGemini_t *pSoftGemini = (SoftGemini_t *)hSoftGemini;
	TI_STATUS return_value = OK;

	WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG, ("  SoftGemini_setParam() (0x%x)\n", pParam->paramType));
	
	switch(pParam->paramType)
	{

	case SOFT_GEMINI_SET_ENABLE:
		
		return_value = SoftGemini_setEnableParam(hSoftGemini,pParam->content.SoftGeminiEnable, FALSE);
		break;

	case SOFT_GEMINI_SET_RATE:
		
		return_value = SoftGemini_setRateParam(hSoftGemini, pParam->content.SoftGeminiRate);
		break;
		
	case SOFT_GEMINI_SET_CONFIG:

		/* copy new params to SoftGemini module */
		SoftGemini_setConfigParam(hSoftGemini,pParam->content.SoftGeminiParamArray);
	
		/* set new params to FW */
		return_value = SoftGemini_setParamsToFW(hSoftGemini, &(pSoftGemini->SoftGeminiParam));
		break;
		
	default:
		WLAN_REPORT_ERROR(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG, ("  SoftGemini_setParam(), Params is not supported, %d\n\n", pParam->paramType));
		return PARAM_NOT_SUPPORTED;
	}

	return return_value;
}

/***********************************************************************
 *			      SoftGemini_getParam									
 ***********************************************************************
DESCRIPTION: SoftGemini get param function, called by the following:
			-	config mgr in order to get a parameter from the OS abstraction layer.
			-	From inside the dirver	
                                                                                                   
INPUT:      hSoftGemini	-	SoftGemini handle.
				

OUTPUT:		pParam	-	Pointer to the parameter	

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS SoftGemini_getParam(TI_HANDLE		hSoftGemini,
											paramInfo_t	*pParam)
{ 
	SoftGemini_printParams(hSoftGemini);
	
	return OK;
}
 


/***************************************************************************
*					SoftGemini_setEnableParam					    	       *
****************************************************************************
* DESCRIPTION:	The function sets the  appropriate Enable value,
*				configures SCR , POWER MGR , DATA CTRL , FW.   
*
* INPUTS:		pSoftGemini - the object		
***************************************************************************/
static TI_STATUS SoftGemini_setEnableParam(TI_HANDLE hSoftGemini, SoftGeminiEnableModes_e SoftGeminiEnable, BOOL recovery)
{
	SoftGemini_t *pSoftGemini = (SoftGemini_t *)hSoftGemini;
	whalParamInfo_t	param;
	TI_STATUS return_value = OK;

	WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG, ("  setSoftGeminiEnableParam() - Old value = %s, New value = %s\n",
		SoftGemini_ConvertModeToString(pSoftGemini->SoftGeminiEnable),SoftGemini_ConvertModeToString(SoftGeminiEnable)));


    /*
     * PsPoll work around is active. Just save the value and configure it later
     */
    if ( pSoftGemini->bPsPollFailureActive )
    {
        WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG, 
            ("  setSoftGeminiEnableParam() - while PsPollFailure is active\n"));

        pSoftGemini->PsPollFailureLastEnableValue = SoftGeminiEnable;
        return OK;
    }

	/**********************************/
	/* Sanity check on enable values */
	/********************************/

	/*				Old Value						New Value		    */        
	/*					|							    |			    */        
	/*			  	   \|/							   \|/			    */        

	if ((pSoftGemini->SoftGeminiEnable == SoftGeminiEnable) && !recovery)
	{
		WLAN_REPORT_ERROR(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG, ("  %s - setting same value \n",__FUNCTION__));
		return NOK;
	}

    
	/*******************************/
	/* Make the necessary actions */
	/*****************************/
	
	switch (SoftGeminiEnable)
	{
	case SG_ENABLE:
		
		if (!pSoftGemini->bDriverEnabled)
		{	
			SoftGemini_EnableDriver(hSoftGemini);
		}
		/* set FW with SG_ENABLE */ 
		param.paramType = HAL_CTRL_SG_ENABLE_PARAM;
		param.content.SoftGeminiEnable = SG_ENABLE;
		return_value = whalCtrl_SetParam(pSoftGemini->hHalCtrl, &param);

		/* Set SG status for the IPC_EVENT_BT_COEX_MODE event */
		pSoftGemini->btCoexStatus.state = TRUE;

		break;

	case SG_DISABLE:
		
		/* set FW with SG_DISABLE */
		param.paramType = HAL_CTRL_SG_ENABLE_PARAM;
		param.content.SoftGeminiEnable = SG_DISABLE;
		return_value = whalCtrl_SetParam(pSoftGemini->hHalCtrl, &param);

		if (pSoftGemini->bDriverEnabled)
		{	
			SoftGemini_DisableDriver(hSoftGemini);
		}
		
		/* Set SG status for the IPC_EVENT_BT_COEX_MODE event */
		pSoftGemini->btCoexStatus.state = FALSE;
		
		break;

	case SG_SENSE_NO_ACTIVITY:

		/* set FW with SG_SENSE_NO_ACTIVITY */
		param.paramType = HAL_CTRL_SG_ENABLE_PARAM;
		param.content.SoftGeminiEnable = SG_SENSE_NO_ACTIVITY;
		return_value = whalCtrl_SetParam(pSoftGemini->hHalCtrl, &param);

		if (pSoftGemini->bDriverEnabled)
		{	
			SoftGemini_DisableDriver(hSoftGemini);
		}

		/* Set SG status for the IPC_EVENT_BT_COEX_MODE event */
		pSoftGemini->btCoexStatus.state = FALSE;
		
		break;

	case SG_SENSE_ACTIVE:
				
		if (!pSoftGemini->bDriverEnabled)
		{	
			SoftGemini_EnableDriver(hSoftGemini); 
		}
		/* set FW with SG_SENSE_ACTIVE */ 
		param.paramType = HAL_CTRL_SG_ENABLE_PARAM;
		param.content.SoftGeminiEnable = SG_SENSE_ACTIVE;
		return_value = whalCtrl_SetParam(pSoftGemini->hHalCtrl, &param);
	
		/* Set SG status for the IPC_EVENT_BT_COEX_MODE event */
		pSoftGemini->btCoexStatus.state = TRUE;
	
		break;

	default:
		WLAN_REPORT_ERROR(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("%s defualt :%d\n",__FUNCTION__,SoftGeminiEnable)); 
		return NOK;
/*		break; - unreachable*/
	}

	/* Pass to the new enable state */
	pSoftGemini->SoftGeminiEnable = SoftGeminiEnable;

	if (OK != return_value)
	{
		WLAN_REPORT_ERROR(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG, 
			("%s can't configure enable param to FW :%s\n",__FUNCTION__,SoftGemini_ConvertModeToString(SoftGeminiEnable))); 
	}
	else
	{
		EvHandlerSendEvent(pSoftGemini->hEvHandler, IPC_EVENT_BT_COEX_MODE, (UINT8*)&pSoftGemini->btCoexStatus, sizeof(btCoexStatus_t));
	}
	
	return return_value;
}

/***************************************************************************
*					SoftGemini_setConfigParam				    	       *
****************************************************************************
* DESCRIPTION:	The function sets params 
*
* INPUTS:		pSoftGemini - the object
*				param       - params to be configured	
***************************************************************************/
static void SoftGemini_setConfigParam(TI_HANDLE hSoftGemini, UINT32 *param)
{
	SoftGemini_t *pSoftGemini = (SoftGemini_t *)hSoftGemini;
	UINT32 i = 0;

	pSoftGemini->SoftGeminiParam.wlanRxMinRateToRespectBtHp		= (UINT32)param[i++];
	pSoftGemini->SoftGeminiParam.btHpMaxTime					= (UINT16)param[i++];
	pSoftGemini->SoftGeminiParam.wlanHpMaxTime					= (UINT16)param[i++];
	pSoftGemini->SoftGeminiParam.senseDisableTimer				= (UINT16)param[i++];
	pSoftGemini->SoftGeminiParam.protectiveRxTimeBeforeBtHp		= (UINT16)param[i++];
	pSoftGemini->SoftGeminiParam.protectiveTxTimeBeforeBtHp		= (UINT16)param[i++];
	pSoftGemini->SoftGeminiParam.protectiveRxTimeBeforeBtHpFastAp = (UINT16)param[i++];
	pSoftGemini->SoftGeminiParam.protectiveTxTimeBeforeBtHpFastAp = (UINT16)param[i++];
	pSoftGemini->SoftGeminiParam.protectiveWlanCycleTimeForFastAp = (UINT16)param[i++];
    pSoftGemini->SoftGeminiParam.btAntiStarvationPeriod			  = (UINT16)param[i++];
    pSoftGemini->SoftGeminiParam.timeoutNextBtLpPacket            = (UINT16)param[i++];
	pSoftGemini->SoftGeminiParam.wakeUpTimeBeforeBeacon           = (UINT16)param[i++];
	pSoftGemini->SoftGeminiParam.hpdmMaxGuardTime				 = (UINT16)param[i++];
	pSoftGemini->SoftGeminiParam.timeoutNextWlanPacket           = (UINT16)param[i++];
	pSoftGemini->SoftGeminiParam.sgAntennaType					= (UINT8)param[i++];
	pSoftGemini->SoftGeminiParam.signalingType					= (UINT8)param[i++];
	pSoftGemini->SoftGeminiParam.afhLeverageOn					= (UINT8)param[i++];
	pSoftGemini->SoftGeminiParam.numberQuietCycle				= (UINT8)param[i++];
	pSoftGemini->SoftGeminiParam.maxNumCts						= (UINT8)param[i++];
	pSoftGemini->SoftGeminiParam.numberOfWlanPackets			= (UINT8)param[i++];
	pSoftGemini->SoftGeminiParam.numberOfBtPackets				= (UINT8)param[i++];
	pSoftGemini->SoftGeminiParam.numberOfMissedRxForAvalancheTrigger = (UINT8)param[i++];
	pSoftGemini->SoftGeminiParam.wlanElpHpSupport				= (UINT8)param[i++];
	pSoftGemini->SoftGeminiParam.btAntiStarvationNumberOfCyclesWithinThePeriod  = (UINT8)param[i++];
    pSoftGemini->SoftGeminiParam.ackModeDuringBtLpInDualAnt     = (UINT8)param[i++];
    pSoftGemini->SoftGeminiParam.allowPaSdToggleDuringBtActivityEnable = (UINT8)param[i++];
	/* 
	 * Check if SG is Active and sgAutoModeNoCts was changed on the fly.
	 *  If so - update powerMgr.
	 */
	pSoftGemini->SoftGeminiParam.sgAutoModeNoCts = (UINT8)param[i++];

    if ( pSoftGemini->SoftGeminiEnable == SG_SENSE_ACTIVE || pSoftGemini->SoftGeminiEnable == SG_ENABLE )
	{
        if (pSoftGemini->SoftGeminiParam.sgAutoModeNoCts)
		{
			/* Exit "Always PS" mode */
			SoftGemini_unSetPS(pSoftGemini);
		} 
		else /* Normal behavior - enter PS */
		{
			SoftGemini_SetPS(pSoftGemini);
		}
	}

	pSoftGemini->SoftGeminiParam.numOfBtHpRespectedReq = (UINT8)param[i++];

}

/***************************************************************************
*					setSoftGeminiRateParam				    	    	       *
****************************************************************************
* DESCRIPTION:	The function sets the Ctrl Data with the appropriate Rate value  
*
* INPUTS:		pSoftGemini - the object		
***************************************************************************/
static TI_STATUS SoftGemini_setRateParam(TI_HANDLE hSoftGemini, UINT8 *pRates)
{
	SoftGemini_t *pSoftGemini = (SoftGemini_t *)hSoftGemini;
	int i;
	UINT32 supportedRates = 0;
	paramInfo_t param;
	rateMask_e supportedRatesArray[NUM_OF_RATES_IN_SG] = 
	{DRV_RATE_MASK_1_BARKER,DRV_RATE_MASK_2_BARKER,DRV_RATE_MASK_5_5_CCK,DRV_RATE_MASK_6_OFDM,
	DRV_RATE_MASK_9_OFDM,DRV_RATE_MASK_11_CCK,DRV_RATE_MASK_12_OFDM,DRV_RATE_MASK_18_OFDM,
	DRV_RATE_MASK_22_PBCC,DRV_RATE_MASK_24_OFDM,DRV_RATE_MASK_36_OFDM,DRV_RATE_MASK_48_OFDM,DRV_RATE_MASK_54_OFDM};
	
	/* loop on all rates and mark the corresponding bit if the rate is enabled */ 
	for (i = 0 ; i < NUM_OF_RATES_IN_SG ; i++) 
	{
		WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  
			(" %s rate %u = %u \n", __FUNCTION__, i,pRates[i])); 
		pSoftGemini->SoftGeminiRate[i] = pRates[i];
		
		if (pRates[i] > 0)
		{
			/* Store the minTxRate */
			if (!supportedRates)
			{
				pSoftGemini->btCoexStatus.minTxRate = (UINT8)supportedRatesArray[i];
			}
			
			supportedRates |= supportedRatesArray[i]; 
		}	
	}
	
	if ((supportedRates & DRV_RATE_MASK_1_BARKER) || (supportedRates & DRV_RATE_MASK_2_BARKER)) 
	{
		WLAN_REPORT_ERROR(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  
			(" %s Using rates 1 or 2 in Soft Gemini is not recommended !!!\n", __FUNCTION__)); 
	}

	WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  
		(" %s rateMask = 0x%x \n", __FUNCTION__,supportedRates)); 

	param.paramType = CTRL_DATA_NEXT_RATE_MASK_FOR_CLIENT;
	param.content.ctrlDataRateClassMask.clientID = SG_RATE_CLASS;
	param.content.ctrlDataRateClassMask.clientRateMask = supportedRates;
	ctrlData_setParam(pSoftGemini->hCtrlData,&param);

	return OK;
}

/***************************************************************************
*					SoftGemini_printParams					    	       *
****************************************************************************
* DESCRIPTION:	Print SG Parameters.  
*
* INPUTS:		pSoftGemini - the object	
***************************************************************************/
void SoftGemini_printParams(TI_HANDLE hSoftGemini)
{
#ifdef REPORT_LOG

	SoftGemini_t *pSoftGemini = (SoftGemini_t *)hSoftGemini;
	SoftGeminiParam_t *SoftGeminiParam = &pSoftGemini->SoftGeminiParam;
	int i;

	WLAN_OS_REPORT(("Rates starting from 1 to 54: \n")); 

	for (i = 0 ; i < NUM_OF_RATES_IN_SG ; i++) 
	{
		WLAN_OS_REPORT((" rate[%u] = %u \n", i,pSoftGemini->SoftGeminiRate[i])); 
	}

	WLAN_OS_REPORT(("  wlanRxMinRateToRespectBtHp = %d\n", SoftGeminiParam->wlanRxMinRateToRespectBtHp)); 
	WLAN_OS_REPORT(("  btHpMaxTime = %d\n", SoftGeminiParam->btHpMaxTime)); 
	WLAN_OS_REPORT(("  wlanHpMaxTime = %d\n", SoftGeminiParam->wlanHpMaxTime)); 
	WLAN_OS_REPORT(("  senseDisableTimer = %d\n", SoftGeminiParam->senseDisableTimer)); 
	WLAN_OS_REPORT(("  protectiveRxTimeBeforeBtHp = %d\n", SoftGeminiParam->protectiveRxTimeBeforeBtHp)); 
	WLAN_OS_REPORT(("  protectiveTxTimeBeforeBtHp = %d\n", SoftGeminiParam->protectiveTxTimeBeforeBtHp)); 
	WLAN_OS_REPORT(("  protectiveRxTimeBeforeBtHpFastAp = %d\n", SoftGeminiParam->protectiveRxTimeBeforeBtHpFastAp)); 
	WLAN_OS_REPORT(("  protectiveTxTimeBeforeBtHpFastAp = %d\n", SoftGeminiParam->protectiveTxTimeBeforeBtHpFastAp)); 
	WLAN_OS_REPORT(("  protectiveWlanCycleTimeForFastAp = %d\n", SoftGeminiParam->protectiveWlanCycleTimeForFastAp)); 
    WLAN_OS_REPORT(("  btAntiStarvationPeriod  = %d\n", SoftGeminiParam->btAntiStarvationPeriod)); 
	WLAN_OS_REPORT(("  timeoutNextBtLpPacket = %d\n", SoftGeminiParam->timeoutNextBtLpPacket)); 
    WLAN_OS_REPORT(("  wakeUpTimeBeforeBeacon = %d\n", SoftGeminiParam->wakeUpTimeBeforeBeacon)); 
	WLAN_OS_REPORT(("  hpdmMaxGuardTime = %d\n", SoftGeminiParam->hpdmMaxGuardTime)); 
	WLAN_OS_REPORT(("  timeoutNextWlanPacket = %d\n", SoftGeminiParam->timeoutNextWlanPacket)); 
    WLAN_OS_REPORT(("  sgAntennaType = %d\n", SoftGeminiParam->sgAntennaType)); 
    WLAN_OS_REPORT(("  signalingType = %d\n", SoftGeminiParam->signalingType)); 
	WLAN_OS_REPORT(("  afhLeverageOn = %d\n", SoftGeminiParam->afhLeverageOn)); 
    WLAN_OS_REPORT(("  numberQuietCycle = %d\n", SoftGeminiParam->numberQuietCycle)); 
	WLAN_OS_REPORT(("  maxNumCts = %d\n", SoftGeminiParam->maxNumCts)); 
	WLAN_OS_REPORT(("  numberOfWlanPackets = %d\n", SoftGeminiParam->numberOfWlanPackets)); 
	WLAN_OS_REPORT(("  numberOfBtPackets = %d\n", SoftGeminiParam->numberOfBtPackets)); 	
	WLAN_OS_REPORT(("  numberOfMissedRxForAvalancheTrigger = %d\n", SoftGeminiParam->numberOfMissedRxForAvalancheTrigger)); 
	WLAN_OS_REPORT(("  wlanElpHpSupport = %d\n", SoftGeminiParam->wlanElpHpSupport)); 
	WLAN_OS_REPORT(("  btAntiStarvationNumberOfCyclesWithinThePeriod = %d\n", SoftGeminiParam->btAntiStarvationNumberOfCyclesWithinThePeriod)); 
    WLAN_OS_REPORT(("  ackModeDuringBtLpInDualAnt = %d\n", SoftGeminiParam->ackModeDuringBtLpInDualAnt)); 
    WLAN_OS_REPORT(("  allowPaSdToggleDuringBtActivityEnable = %d\n", SoftGeminiParam->allowPaSdToggleDuringBtActivityEnable)); 
	WLAN_OS_REPORT(("  sgAutoModeNoCts = %d\n", SoftGeminiParam->sgAutoModeNoCts)); 
	WLAN_OS_REPORT(("  numOfBtHpRespectedReq = %d\n\n", SoftGeminiParam->numOfBtHpRespectedReq)); 

	WLAN_OS_REPORT(("  Enable mode : %s\n", SoftGemini_ConvertModeToString(pSoftGemini->SoftGeminiEnable))); 
	WLAN_OS_REPORT(("  Driver Enabled : %s\n",(pSoftGemini->bDriverEnabled ? "YES" : "NO"))); 
	WLAN_OS_REPORT(("  Protective mode : %s\n", (pSoftGemini->bProtectiveMode ? "ON" : "OFF"))); 
    WLAN_OS_REPORT(("  PsPoll failure active : %s\n", (pSoftGemini->bPsPollFailureActive ? "YES" : "NO"))); 

#endif
}

/***********************************************************************
 *                        SoftGemini_reconnect									
 ***********************************************************************
DESCRIPTION: causes driver to disconnect and reconnect to current AP
                                                                                                   
INPUT:      hSoftGemini	-	SoftGemini handle.

************************************************************************/

void SoftGemini_reconnect(TI_HANDLE hSoftGemini)
{
	SoftGemini_t *pSoftGemini = (SoftGemini_t *)hSoftGemini;
	paramInfo_t	ssidParam;
	paramInfo_t	param;
	TI_STATUS	status;

	/* Get current SSID; if we are not connected, there is nothing to do, return */
	ssidParam.paramType = SITE_MGR_CURRENT_SSID_PARAM;
	status =  configMgr_getParam(pSoftGemini->hConfigMgr, &ssidParam);
	if (status != OK)
	{
		return;
	}

	/* Take care of scan enabled SME parameter: if is set to TRUE, set it to SKIP_ONCE */
	param.paramType = SME_SCAN_ENABLED_PARAM;
	configMgr_getParam(pSoftGemini->hConfigMgr, &param);

	if (param.content.smeSMScanEnabled == SCAN_ENABLED) 
	{
		param.content.smeSMScanEnabled = SKIP_NEXT_SCAN;
		configMgr_setParam(pSoftGemini->hConfigMgr, &param);
	}

	WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("%s - reconnecting !!!\n",__FUNCTION__)); 

	/* Re-set desired SSID */
	ssidParam.paramType = SITE_MGR_DESIRED_SSID_PARAM;
	configMgr_setParam(pSoftGemini->hConfigMgr, &ssidParam);
}

/***************************************************************************
*					SoftGemini_setParamsToFW					    	       *
****************************************************************************
* DESCRIPTION:	The function sets the FW with the appropriate parameters set.  
*
* INPUTS:		pSoftGemini - the object
*
*
* OUTPUT:			
* 
* RETURNS:		
***************************************************************************/
static TI_STATUS SoftGemini_setParamsToFW(TI_HANDLE hSoftGemini, SoftGeminiParam_t *SoftGeminiParam)
{
	SoftGemini_t *pSoftGemini = (SoftGemini_t *)hSoftGemini;
	whalParamInfo_t	whalParam;
	
	os_memoryCopy(pSoftGemini->hOs,&whalParam.content.SoftGeminiParam, SoftGeminiParam, sizeof(SoftGeminiParam_t));
	whalParam.paramType = HAL_CTRL_SG_CONFIG_PARAM;
	return whalCtrl_SetParam(pSoftGemini->hHalCtrl,&whalParam);
}


/***************************************************************************
*					SoftGemini_EnableDriver  		    	       *
****************************************************************************
* DESCRIPTION:	Activated when SG is enabled (after CLI or FW command)
*
* INPUTS:		pSoftGemini - the object
*	
***************************************************************************/
static TI_STATUS SoftGemini_EnableDriver(TI_HANDLE hSoftGemini)
{
	SoftGemini_t	*pSoftGemini = (SoftGemini_t *)hSoftGemini;
	TI_STATUS return_value = OK;
	paramInfo_t param;

	WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("%s\n",__FUNCTION__)); 

	pSoftGemini->bDriverEnabled = TRUE;
	
	/* Set Always PS only if sgAutoModeNoCts is FALSE, i.e. normal operation */
	if ( !pSoftGemini->SoftGeminiParam.sgAutoModeNoCts )
	{
		SoftGemini_SetPS(pSoftGemini);
	}

	/* set ctrlData to use SG rates */
	param.paramType = CTRL_DATA_CURRENT_RATE_CLASS_CLIENT;
	param.content.ctrlDataRateClassID = SG_RATE_CLASS;
	ctrlData_setParam(pSoftGemini->hCtrlData,&param);

	scr_setMode(pSoftGemini->hSCR, SCR_MID_SG);

	return return_value;
}

/***************************************************************************
*					SoftGemini_DisableDriver  		    	       *
****************************************************************************
* DESCRIPTION:	Activated when SG is disabled (after CLI or FW command)
*
* INPUTS:		pSoftGemini - the object
*	
***************************************************************************/
static TI_STATUS SoftGemini_DisableDriver(TI_HANDLE hSoftGemini)
{
	SoftGemini_t	*pSoftGemini = (SoftGemini_t *)hSoftGemini;
	TI_STATUS return_value = OK;
	paramInfo_t param;

	WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("%s\n",__FUNCTION__)); 

	pSoftGemini->bDriverEnabled = FALSE;

	/* set ctrlData to use default rates */
	param.paramType = CTRL_DATA_CURRENT_RATE_CLASS_CLIENT;
	param.content.ctrlDataRateClassID = USER_RATE_CLASS;
	ctrlData_setParam(pSoftGemini->hCtrlData,&param);

	scr_setMode(pSoftGemini->hSCR, SCR_MID_NORMAL);
	
	/* unSet Always PS only if sgAutoModeNoCts is FALSE, i.e. normal operation */
	if ( !pSoftGemini->SoftGeminiParam.sgAutoModeNoCts )
	{
		SoftGemini_unSetPS(pSoftGemini);
	}

	/* Undo the changes that were made when Protective mode was on */
	if (pSoftGemini->bProtectiveMode)
	{
		SoftGemini_DisableProtectiveMode(hSoftGemini);
	}
	
	return return_value;
	
}

/***************************************************************************
*					SoftGemini_SetPS  		    						   *
****************************************************************************
* DESCRIPTION:	Set Always PS to PowerMgr
*
* INPUTS:		pSoftGemini - the object
*	
***************************************************************************/
static TI_STATUS SoftGemini_SetPS(SoftGemini_t	*pSoftGemini)
{
	paramInfo_t param;
	bssEntry_t *pBssInfo=NULL;

    if (pSoftGemini->hCurrBss)
	{
		pBssInfo = currBSS_getBssInfo(pSoftGemini->hCurrBss);
	}
	else WLAN_REPORT_ERROR(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("SoftGemini_SetPS: hCurrBss = NULL!!!\n")); 


	WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("%s\n",__FUNCTION__)); 

	if (pBssInfo)
	{
		if ((pBssInfo->band == RADIO_BAND_2_4_GHZ) && (!pSoftGemini->SoftGeminiParam.sgAutoModeNoCts))
		{
			WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG, (" SG-setPS: band == RADIO_BAND_2_4_GHZ")); 
		
			/* Set Params to Power Mgr for SG priority */
			param.paramType = POWER_MGR_POWER_MODE;
			param.content.powerMngPowerMode.PowerMode = POWER_MODE_PS_ONLY;
			param.content.powerMngPowerMode.powerMngPriority = POWER_MANAGER_SG_PRIORITY;
			powerMgr_setParam(pSoftGemini->hPowerMgr,&param);
		
			/* enable SG priority for Power Mgr */
			param.paramType = POWER_MGR_ENABLE_PRIORITY;
			param.content.powerMngPriority = POWER_MANAGER_SG_PRIORITY;
			return powerMgr_setParam(pSoftGemini->hPowerMgr,&param);
		}
		else WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG, (" SG-setPS: band == RADIO_BAND_5_GHZ")); 
	}
	return OK;
}

/***************************************************************************
*					SoftGemini_unSetPS  		    						   *
****************************************************************************
* DESCRIPTION:	unSet Always PS to PowerMgr
*
* INPUTS:		pSoftGemini - the object
*	
***************************************************************************/
static TI_STATUS SoftGemini_unSetPS(SoftGemini_t	*pSoftGemini)
{
	paramInfo_t param;

	WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("%s, SG-unSetPS \n",__FUNCTION__)); 

     /* disable SG priority for Power Mgr*/
	param.paramType = POWER_MGR_DISABLE_PRIORITY;
	param.content.powerMngPriority = POWER_MANAGER_SG_PRIORITY;
	return powerMgr_setParam(pSoftGemini->hPowerMgr,&param);
	
}

/***************************************************************************
*					SoftGemini_EnableProtectiveMode  		    	       *
****************************************************************************
* DESCRIPTION:	Activated when FW inform us that protective mode is ON
*				
*
* INPUTS:		pSoftGemini - the object
*	
***************************************************************************/
void SoftGemini_EnableProtectiveMode(TI_HANDLE hSoftGemini)
{
	SoftGemini_t	*pSoftGemini = (SoftGemini_t *)hSoftGemini;
	paramInfo_t 	param;

	WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("%s\n",__FUNCTION__)); 

	pSoftGemini->bProtectiveMode = TRUE;

	/* set new configurations of SG roaming parameters */
	currBSS_SGconfigureBSSLoss(pSoftGemini->hCurrBss,pSoftGemini->BSSLossCompensationPercent,TRUE);

	/* set new configurations of scan to scancncn */
	scanConcentrator_SGconfigureScanParams(pSoftGemini->hScanCncn,TRUE,pSoftGemini->scanNumOfProbeRequest,
		pSoftGemini->scanCompensationMaxTime,pSoftGemini->scanCompensationPercent);

    /* Call the power manager to enter short doze */
	WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG, (" SoftGemini_EnableProtectiveMode set SD")); 

	/* Set Params to Power Mgr for SG priority */
	param.paramType = POWER_MGR_POWER_MODE;
	param.content.powerMngPowerMode.PowerMode = POWER_MODE_SHORT_DOZE;
	param.content.powerMngPowerMode.powerMngPriority = POWER_MANAGER_SG_PRIORITY;
	powerMgr_setParam(pSoftGemini->hPowerMgr,&param);
}

/***************************************************************************
*					SoftGemini_DisableProtectiveMode  		    	       *
****************************************************************************
* DESCRIPTION:	Activated when FW inform us that protective mode is OFF or SG is disabled
*
* INPUTS:		pSoftGemini - the object
*	
***************************************************************************/
void SoftGemini_DisableProtectiveMode(TI_HANDLE hSoftGemini)
{
	SoftGemini_t	*pSoftGemini = (SoftGemini_t *)hSoftGemini;

	WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("%s\n",__FUNCTION__)); 

	pSoftGemini->bProtectiveMode = FALSE;

	SoftGemini_RemoveProtectiveModeParameters(hSoftGemini);
}

/***************************************************************************
*					SoftGemini_DisableProtectiveMode  		    	       *
****************************************************************************
* DESCRIPTION:	Called from SoftGemini_DisableProtectiveMode() when FW inform 
*				us that protective mode is OFF or SG is disabled, or from
*				SoftGemini_unSetPSmode() when driver disconnects from AP.
*
* INPUTS:		pSoftGemini - the object
*	
***************************************************************************/

void SoftGemini_RemoveProtectiveModeParameters(TI_HANDLE hSoftGemini)
{
	SoftGemini_t	*pSoftGemini = (SoftGemini_t *)hSoftGemini;
	paramInfo_t  	param;

	WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("%s\n",__FUNCTION__)); 
    
	/* don't use the SG roaming parameters */
	currBSS_SGconfigureBSSLoss(pSoftGemini->hCurrBss,0,FALSE);

	/* don't use the SG scan parameters */
	scanConcentrator_SGconfigureScanParams(pSoftGemini->hScanCncn,FALSE,0,0,0);

    /* Call the power manager to exit short doze */
	/* Set Params to Power Mgr for SG priority */
	param.paramType = POWER_MGR_POWER_MODE;
	param.content.powerMngPowerMode.PowerMode = POWER_MODE_PS_ONLY;
	param.content.powerMngPowerMode.powerMngPriority = POWER_MANAGER_SG_PRIORITY;
	powerMgr_setParam(pSoftGemini->hPowerMgr,&param);
}

/***************************************************************************
*					SoftGemini_SenseIndicationCB  		    	       *
****************************************************************************
* DESCRIPTION:	This is the the function which is called for sense mode indication from FW
*				(i.e. we are in SENSE mode and FW detects BT activity )
*				SENSE_MODE_ENABLE - Indicates that FW detected BT activity
*				SENSE_MODE_DISABLE - Indicates that FW doesn't detect BT activity for a period of time
*
* INPUTS:		pSoftGemini - the object
* NOTE			This function is located in the API for debug purposes
***************************************************************************/

void SoftGemini_SenseIndicationCB( TI_HANDLE hSoftGemini, char* str, UINT32 strLen )
{
	SoftGemini_t	*pSoftGemini = (SoftGemini_t *)hSoftGemini;

	WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("%s with 0x%x\n",__FUNCTION__,*str)); 

	if ((SG_SENSE_NO_ACTIVITY == pSoftGemini->SoftGeminiEnable) && (SENSE_MODE_ENABLE == *str))
	{
		SoftGemini_setEnableParam(hSoftGemini, SG_SENSE_ACTIVE, FALSE);
	}
	else if ((SG_SENSE_ACTIVE == pSoftGemini->SoftGeminiEnable) && (SENSE_MODE_DISABLE == *str))
	{
		SoftGemini_setEnableParam(hSoftGemini, SG_SENSE_NO_ACTIVITY, FALSE);
	}
	else
	{
			WLAN_REPORT_WARNING(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("%s : %s called when SG mode is %s ? \n",
				__FUNCTION__,SoftGemini_ConvertModeToString(((SENSE_MODE_ENABLE == *str) ? SG_SENSE_ACTIVE : SG_SENSE_NO_ACTIVITY))
				,SoftGemini_ConvertModeToString(pSoftGemini->SoftGeminiEnable))); 
	}
}

/***************************************************************************
*					SoftGemini_ProtectiveIndicationCB  		    	       *
****************************************************************************
* DESCRIPTION:	This is the the function which is called when FW starts Protective mode (i.e BT voice) 
*
*				PROTECTIVE_MODE_ON - FW is activated on protective mode (BT voice is running)
*				PROTECTIVE_MODE_OFF - FW is not activated on protective mode
*
* INPUTS:		pSoftGemini - the object
* NOTE			This function is located in the API for debug purposes
***************************************************************************/

void SoftGemini_ProtectiveIndicationCB( TI_HANDLE hSoftGemini, char* str, UINT32 strLen )
{
	SoftGemini_t	*pSoftGemini = (SoftGemini_t *)hSoftGemini;
	
	WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("%s with 0x%x\n",__FUNCTION__,*str)); 

	if ((SG_ENABLE == pSoftGemini->SoftGeminiEnable) ||
		(SG_SENSE_ACTIVE == pSoftGemini->SoftGeminiEnable))
	{
		if ((!pSoftGemini->bProtectiveMode) && (PROTECTIVE_MODE_ON == *str))
		{
			SoftGemini_EnableProtectiveMode(hSoftGemini);
		}
		else if ((pSoftGemini->bProtectiveMode) && (PROTECTIVE_MODE_OFF == *str))
		{
			SoftGemini_DisableProtectiveMode(hSoftGemini);
		}
		else
		{
			WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("%s : Protective mode %s called when Protective mode is %s \n",
			__FUNCTION__,((PROTECTIVE_MODE_ON == *str) ? "ON" : "OFF"),((pSoftGemini->bProtectiveMode) ? "ON" : "OFF"))); 
		}
	}
	else
	{
		WLAN_REPORT_WARNING(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("%s : Protective mode %s called when SG mode is %s ? \n",
			__FUNCTION__,((PROTECTIVE_MODE_ON == *str) ? "ON" : "OFF"),SoftGemini_ConvertModeToString(pSoftGemini->SoftGeminiEnable))); 
	}
}

/***************************************************************************
*					SoftGemini_AvalancheIndicationCB  		    	       *
****************************************************************************
* DESCRIPTION:	This is the the function which is called when 
*				FW detect that our current connection quality is reducing
*				(AP decrease his rates with his rate adaptation mechanism because
*				of BT activity) the solution is reconnect to the same AP
*
* INPUTS:		pSoftGemini - the object
* NOTE			This function is located in the API for debug purposes
***************************************************************************/

void SoftGemini_AvalancheIndicationCB( TI_HANDLE hSoftGemini, char* str, UINT32 strLen )
{
	SoftGemini_t	*pSoftGemini = (SoftGemini_t *)hSoftGemini;

	WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("%s \n",__FUNCTION__)); 

	if ((SG_ENABLE == pSoftGemini->SoftGeminiEnable) ||
		(SG_SENSE_ACTIVE == pSoftGemini->SoftGeminiEnable)) 
	{
		SoftGemini_reconnect(hSoftGemini);
	}
	else
	{
		WLAN_REPORT_WARNING(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  
			("%s : Avalanche Avoidance called when SG mode is %s \n",
			__FUNCTION__,SoftGemini_ConvertModeToString(pSoftGemini->SoftGeminiEnable))); 
	}

}

/***************************************************************************
*					SoftGemini_ConvertModeToString  		    	       *
****************************************************************************/
#ifdef REPORT_LOG

char* SoftGemini_ConvertModeToString(SoftGeminiEnableModes_e SoftGeminiEnable)
{
	switch(SoftGeminiEnable)
	{
	case SG_ENABLE:				return "SG_ENABLE";
	case SG_DISABLE:			return "SG_DISABLE";
	case SG_SENSE_NO_ACTIVITY:  return "SG_SENSE_NO_ACTIVITY";
	case SG_SENSE_ACTIVE:		return "SG_SENSE_ACTIVE";
	default:
		return "ERROR";
	}
}

#endif

/***************************************************************************
*					SoftGemini_getSGMode						  		    	       *
****************************************************************************/
SoftGeminiEnableModes_e SoftGemini_getSGMode(TI_HANDLE hSoftGemini)
{
	SoftGemini_t	*pSoftGemini = (SoftGemini_t *)hSoftGemini;
	return pSoftGemini->SoftGeminiEnable;	
}

/***************************************************************************
*					SoftGemini_handleRecovery					    	       *
****************************************************************************
* DESCRIPTION:	The function reconfigures WHAL with the SG parameters.
*
* INPUTS:		pSoftGemini - the object		
***************************************************************************/
TI_STATUS SoftGemini_handleRecovery(TI_HANDLE hSoftGemini)
{
	SoftGemini_t	*pSoftGemini = (SoftGemini_t *)hSoftGemini;
	SoftGeminiEnableModes_e       realSoftGeminiEnableMode;

	realSoftGeminiEnableMode = pSoftGemini->SoftGeminiEnable;
    /* Disable the SG */
    SoftGemini_setEnableParam(hSoftGemini, SG_DISABLE, TRUE);
    WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("Disable SG \n")); 

	pSoftGemini->SoftGeminiEnable = realSoftGeminiEnableMode;
	/* Set enable param */
	if (pSoftGemini->SoftGeminiEnable == SG_SENSE_ACTIVE)
	{
		pSoftGemini->SoftGeminiEnable = SG_SENSE_NO_ACTIVITY;
	}
    SoftGemini_setEnableParam(hSoftGemini, pSoftGemini->SoftGeminiEnable, TRUE);
    WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  ("Set SG to-%d\n", pSoftGemini->SoftGeminiEnable)); 

	/* Config the params to FW */
	SoftGemini_setParamsToFW(hSoftGemini, &pSoftGemini->SoftGeminiParam);
	/*SoftGemini_printParams(hSoftGemini);*/
	return OK;
}
/***************************************************************************
*					SoftGemini_startPsPollFailure					       *
****************************************************************************
* DESCRIPTION:	After Ps-Poll failure we disable the SG
*
* INPUTS:		pSoftGemini - the object		
***************************************************************************/
void SoftGemini_startPsPollFailure(TI_HANDLE hSoftGemini)
{
    SoftGemini_t	*pSoftGemini = (SoftGemini_t *)hSoftGemini;

    WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG, ("%s\n",__FUNCTION__)); 

    if ( !pSoftGemini->bPsPollFailureActive && !pSoftGemini->SoftGeminiParam.sgAutoModeNoCts)
    {
        pSoftGemini->PsPollFailureLastEnableValue = pSoftGemini->SoftGeminiEnable;

        /* Disable SG if needed */
        if ( pSoftGemini->SoftGeminiEnable != SG_DISABLE )
        {	
            SoftGemini_setEnableParam(hSoftGemini, SG_DISABLE, FALSE);
        }

        pSoftGemini->bPsPollFailureActive = TRUE;
    }
    else /* Calling SoftGemini_startPsPollFailure twice ? */
    {
        WLAN_REPORT_WARNING(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  
            ("Calling  SoftGemini_startPsPollFailure while bPsPollFailureActive is TRUE\n")); 
    }
}

/***************************************************************************
*					SoftGemini_endPsPollFailure					    	   *
****************************************************************************
* DESCRIPTION:	Return to normal behavior after the PsPoll failure 
*
* INPUTS:		pSoftGemini - the object		
***************************************************************************/
void SoftGemini_endPsPollFailure(TI_HANDLE hSoftGemini)
{
    SoftGemini_t	*pSoftGemini = (SoftGemini_t *)hSoftGemini;

    WLAN_REPORT_INFORMATION(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG, ("%s\n",__FUNCTION__)); 

    if ( pSoftGemini->bPsPollFailureActive ) 
    {
        pSoftGemini->bPsPollFailureActive = FALSE;

        /* return to previous value */
        if ( pSoftGemini->PsPollFailureLastEnableValue != SG_DISABLE )
        {
            if ( pSoftGemini->PsPollFailureLastEnableValue == SG_SENSE_ACTIVE)
            {
                /* In this case Fw must get SG_SENSE_NO_ACTIVITY to start sensing again */    
                pSoftGemini->PsPollFailureLastEnableValue = SG_SENSE_NO_ACTIVITY;
            }

            SoftGemini_setEnableParam(hSoftGemini, pSoftGemini->PsPollFailureLastEnableValue, FALSE);
        }
    }
    else /* Calling SoftGemini_endPsPollFailure twice ? */
    {
        WLAN_REPORT_WARNING(pSoftGemini->hReport, SOFT_GEMINI_MODULE_LOG,  
            ("Calling  SoftGemini_endPsPollFailure while bPsPollFailureActive is FALSE\n")); 
    }
}


