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
 
/**********************************************************************************/
/*                                                                                */
/*   MODULE:  PowerAuthorization.c                                                */
/*   PURPOSE: PowerAuthorization implementation module							  */
/*                                                                                */
/**********************************************************************************/
#include "whalCtrl_api.h"
#include "report.h"
#include "MacServices_api.h"
#include "MacServices.h"
#include "PowerAuthorization.h"
#include "PowerAuthorization_internal.h"

/****************************************************************************
 *                      powerAutho_Create()
 ****************************************************************************
 * DESCRIPTION:	
 * 
 * INPUTS:	hOs - the handle to the OS layer
 *			hReport - the handle to the report module
 *			hELPCtrl - the handle to the ELPCtrl module
 *		
 * 
 * OUTPUT:	the context of the PowerAuthorization module
 * 
 * RETURNS:	the context of the PowerAuthorization module (NULL if error)
 ****************************************************************************/
TI_HANDLE powerAutho_Create(TI_HANDLE hOs)
{		 
	powerAutho_t *pObj;

	pObj = os_memoryAlloc(hOs, sizeof(powerAutho_t));
	if (pObj == NULL)
	{
		WLAN_OS_REPORT(("FATAL ERROR: powerAutho_Create(): Error allocating context\n"));
		return NULL;		
	}

	os_memoryZero(hOs, pObj, sizeof(powerAutho_t));

	pObj->hOs = hOs;
	pObj->hReport = NULL;
	pObj->hHalCtrl = NULL;

	/* set as 'before init complete' */
	pObj->initComplete = FALSE; 
	
	pObj->m_AwakeRequired = 0;
	pObj->m_PowerPolicy = POWERAUTHO_POLICY_PD;
	pObj->m_MinPowerLevel = POWERAUTHO_POLICY_PD;

	return pObj;
}


/****************************************************************************
 *                      powerAutho_Destroy()
 ****************************************************************************
 * DESCRIPTION:	
 * 
 * INPUTS:	hPowerAutho - the handle to the PowerAuthorization module.
 *		
 * 
 * OUTPUT:	
 * 
 * RETURNS:	OK
 ****************************************************************************/
int powerAutho_Destroy(TI_HANDLE hPowerAutho)
{			
	powerAutho_t *pPowerAutho = (powerAutho_t*)hPowerAutho;

	if (pPowerAutho)
		os_memoryFree(pPowerAutho->hOs, pPowerAutho, sizeof(powerAutho_t));

	return OK;
}

/****************************************************************************
 *                      powerAutho_Configure()
 ****************************************************************************
 * DESCRIPTION:	
 * 
 * INPUTS:	hPowerAutho - the handle to the PowerAuthorization module.
 *			aPowerPolicy - the power policy to configure.
 *		
 * 
 * OUTPUT:	
 * 
 * RETURNS:	OK
 ****************************************************************************/
int powerAutho_Configure(TI_HANDLE hPowerAutho, TI_HANDLE hReport, TI_HANDLE hHalCtrl, powerAutho_PowerPolicy_e aPowerPolicy)
{
	powerAutho_t *pPowerAutho = (powerAutho_t*)hPowerAutho;

	pPowerAutho->m_PowerPolicy = aPowerPolicy;

	pPowerAutho->m_ElpCtrl_Mode_LUT[POWERAUTHO_POLICY_AWAKE] =	ELPCTRL_MODE_KEEP_AWAKE;
	pPowerAutho->m_ElpCtrl_Mode_LUT[POWERAUTHO_POLICY_PD] =		ELPCTRL_MODE_KEEP_AWAKE;
	pPowerAutho->m_ElpCtrl_Mode_LUT[POWERAUTHO_POLICY_ELP] =	ELPCTRL_MODE_NORMAL;

	pPowerAutho->hReport = hReport;
	pPowerAutho->hHalCtrl = hHalCtrl;

	return OK;
}

/****************************************************************************
 *                      powerAutho_PowerPolicyUpdate()
 ****************************************************************************
 * DESCRIPTION:	updates the PowerPolicy and calcs the new MinPowerPolicy of the sustem
 * 
 * INPUTS:	hMacServices - the handle to the MacServices module.
 *			aPowerPolicy - the new power policy.
 *		
 * 
 * OUTPUT:	none
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int MacServices_powerAutho_PowerPolicyUpdate(TI_HANDLE hMacServices, powerAutho_PowerPolicy_e aPowerPolicy)
{
	powerAutho_t *pPowerAutho = (powerAutho_t*)(((MacServices_t*)hMacServices)->hPowerAutho);

	WLAN_REPORT_INFORMATION (pPowerAutho->hReport,ELP_MODULE_LOG,
									("MacServices_powerAutho_PowerPolicyUpdate: PowerPolicy = %d\n",aPowerPolicy));

	pPowerAutho->m_PowerPolicy = aPowerPolicy;

	return powerAutho_CalcMinPowerLevel(pPowerAutho);	
}

/****************************************************************************
 *                      powerAutho_AwakeRequiredUpdate()
 ****************************************************************************
 * DESCRIPTION:	updates the AwakeRequired and calcs the new MinPowerPolicy of the sustem
 * 
 * INPUTS:	hMacServices - the handle to the MacServices module.
 *			aAwakeRequired - the awake required parameter,
 *				can be according to the enum required or not_required.	
 *			aAwakeReason - the reason that the HW is required
 * 
 * OUTPUT:	none
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int MacServices_powerAutho_AwakeRequiredUpdate(TI_HANDLE hMacServices, MacServices_powerAutho_AwakeRequired_e aAwakeRequired, MacServices_powerAutho_AwakeReason_e aAwakeReason)
{
	powerAutho_t *pPowerAutho = (powerAutho_t*)(((MacServices_t*)hMacServices)->hPowerAutho);

	if(aAwakeRequired == POWERAUTHO_AWAKE_REQUIRED)
	{
		pPowerAutho->m_AwakeRequired |= (1<<aAwakeReason);
	}
	else
	{ /* aAwakeRequired == POWERAUTHO_AWAKE_NOT_REQUIRED*/
		pPowerAutho->m_AwakeRequired &= ~(1<<aAwakeReason);
	}
	
	WLAN_REPORT_INFORMATION (pPowerAutho->hReport,ELP_MODULE_LOG,
									("MacServices_powerAutho_AwakeRequiredUpdate: awake required sent %d (reason %d) and the updated is %d\n", aAwakeRequired, aAwakeReason, pPowerAutho->m_AwakeRequired));
	
	return powerAutho_CalcMinPowerLevel(pPowerAutho);	
}

/****************************************************************************
 *                      powerAutho_CalcMinPowerLevel()
 ****************************************************************************
 * DESCRIPTION:	calculate the min power level
 * 
 * INPUTS:	hPowerAutho - the handle to the PowerAuthorization module.
 * 
 * OUTPUT:	none
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int powerAutho_CalcMinPowerLevel(TI_HANDLE hPowerAutho)
{
	powerAutho_t *pPowerAutho = (powerAutho_t*)hPowerAutho;
	powerAutho_PowerPolicy_e newMinPowerLevel;
	whalParamInfo_t ParamInfo;

	/* calc the new MinPowerLevel */
	if(pPowerAutho->m_AwakeRequired >  0)
		newMinPowerLevel = POWERAUTHO_POLICY_AWAKE;
	else
		newMinPowerLevel = pPowerAutho->m_PowerPolicy;

	/* check if the MinPowerLevel changed */
	if(pPowerAutho->m_MinPowerLevel != newMinPowerLevel)
	{
		WLAN_REPORT_INFORMATION (pPowerAutho->hReport,ELP_MODULE_LOG,
									("powerAutho_CalcMinPowerLevel - new MinPowerLevel is = %d\n",newMinPowerLevel));
			
		pPowerAutho->m_MinPowerLevel = newMinPowerLevel;

		/* we do the update of the FW only after the init complete*/
		if(pPowerAutho->initComplete == TRUE)
		{
			/* Update interface mode */
			whalCtrl_ElpCtrl_SetMode(pPowerAutho->hHalCtrl, pPowerAutho->m_ElpCtrl_Mode_LUT[newMinPowerLevel]);

			/* Send MIB with PowerPolicy */
			ParamInfo.paramType = (UINT32)HAL_CTRL_MIN_POWER_LEVEL;
			ParamInfo.paramLength = sizeof(powerAutho_PowerPolicy_e);
			ParamInfo.content.minPowerPolicy = newMinPowerLevel;
			whalCtrl_SetParam(pPowerAutho->hHalCtrl, &ParamInfo);
            return OK;
		}
	}

	return TNETWIF_COMPLETE;	
}

/****************************************************************************
 *                      powerAutho_PowerPolicyUpdate()
 ****************************************************************************
 * DESCRIPTION:	send the min power level to the FW for the first time
 * 
 * INPUTS:	hMacServices - the handle to the MacServices module.
 *		
 * 
 * OUTPUT:	none
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int MacServices_powerAutho_ExitFromInit(TI_HANDLE hMacServices)
{
	powerAutho_t *pPowerAutho = (powerAutho_t*)(((MacServices_t*)hMacServices)->hPowerAutho);
	whalParamInfo_t ParamInfo;

	WLAN_REPORT_INFORMATION (pPowerAutho->hReport,ELP_MODULE_LOG,
									("MacServices_powerAutho_ExitFromInit: PowerPolicy = %d\n",pPowerAutho->m_MinPowerLevel ));
	/* set as 'after init complete' */
	pPowerAutho->initComplete = TRUE;

	/* Update interface mode */
	whalCtrl_ElpCtrl_SetMode(pPowerAutho->hHalCtrl, pPowerAutho->m_ElpCtrl_Mode_LUT[pPowerAutho->m_MinPowerLevel]);

	/* Send MIB with PowerPolicy */
	ParamInfo.paramType = (UINT32)HAL_CTRL_MIN_POWER_LEVEL;
	ParamInfo.paramLength = sizeof(powerAutho_PowerPolicy_e);
	ParamInfo.content.minPowerPolicy = pPowerAutho->m_MinPowerLevel;
	whalCtrl_SetParam(pPowerAutho->hHalCtrl, &ParamInfo);

	return OK;
}


/****************************************************************************
 *                      MacServices_powerAutho_Endrecovery()
 ****************************************************************************
 * DESCRIPTION:	initialize module after recovery
 * 
 * INPUTS:	hMacServices - the handle to the MacServices module.
 *		
 * 
 * OUTPUT:	none
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int powerAutho_Restart(TI_HANDLE hMacServices)
{			
	powerAutho_t *pPowerAutho = (powerAutho_t*)(((MacServices_t*)hMacServices)->hPowerAutho);

		/* set as 'before init complete' */
	pPowerAutho->initComplete = FALSE; 
	
	pPowerAutho->m_PowerPolicy = POWERAUTHO_POLICY_AWAKE;
	pPowerAutho->m_MinPowerLevel = POWERAUTHO_POLICY_AWAKE;


	return pPowerAutho->m_MinPowerLevel;
}


/****************************************************************************
 *                      MacServices_powerAutho_Endrecovery()
 ****************************************************************************
 * DESCRIPTION:	updates the PowerPolicy and calcs the new MinPowerPolicy of the sustem
 * 
 * INPUTS:	hMacServices - the handle to the MacServices module.
 *		
 * 
 * OUTPUT:	none
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int MacServices_powerAutho_EndRecovery(TI_HANDLE hMacServices)
{
	powerAutho_t *pPowerAutho = (powerAutho_t*)(((MacServices_t*)hMacServices)->hPowerAutho);

	WLAN_REPORT_INFORMATION (pPowerAutho->hReport,ELP_MODULE_LOG,
									("MacServices_powerAutho_Endrecovery: PowerPolicy = %d\n",pPowerAutho->m_PowerPolicy));

	return powerAutho_CalcMinPowerLevel(pPowerAutho);	
}


