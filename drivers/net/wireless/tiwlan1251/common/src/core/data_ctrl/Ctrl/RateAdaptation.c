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
/**************************************************************************/
/*                                                                        */
/*   MODULE:  Rate Adaptation.c                                           */
/*                                                                        */
/**************************************************************************/
#include "RateAdaptation.h"
#include "DataCtrl_Api.h"
#include "802_11Defs.h"
#include "osApi.h"
#include "report.h"
#include "utils.h"
#include "EvHandler.h"
#include "apConn.h"


static TI_STATUS rateAdaptationa_rateToIndexInRateMapTable(rateAdaptation_t* pRateAdaptation, 
														   rate_e rate, UINT8* Index);

static modulationType_e setModulationForRate(rateAdaptation_t	*pRateAdaptation,
   											 rate_e				rate,
  											 modulationType_e	modulation,
											 bssType_e			bssType);

BOOL rateAdaptation_isRateInTable(ctrlData_rateAdapt_t *currTable,
								rate_e			rate);

void rateAdaptation_getFallBackStepUp(ctrlData_rateAdapt_t *currTable,
										rate_e			rate,UINT8* FB,UINT8* SU);

static void rateAdaptation_rxTimeOut(TI_HANDLE hRateAdaptation);

static BOOL set4xEnableForRate(rateAdaptation_t*	pRateAdaptation, 
							   rate_e				rate,
							   BOOL					enable4x, 
							   bssType_e			bssType);

/*************************************************************************
*                        ctrlData_create                                 
**************************************************************************
* DESCRIPTION:	This function create the rateAdaptation module.                 
*                                                      
* INPUT:		hOs - handle to Os Abstraction Layer
*				
* OUTPUT:	
*
* RETURN:		Handle to the allocated rateAdaptation block
************************************************************************/

rateAdaptation_t* rateAdaptation_create(TI_HANDLE hOs)
{
	rateAdaptation_t* pRateAdaptation;
	void			*pTimer;

	if( hOs == NULL )
	{
	    WLAN_OS_REPORT(("FATAL ERROR: rateAdaptation_create(): OS handle Error - Aborting\n"));
		return NULL;
	}

	/* alocate RateAdaptation block */
	pRateAdaptation = os_memoryAlloc(hOs, (sizeof(rateAdaptation_t)));
	if(!pRateAdaptation)
		return NULL;

	/* alocate Timer to use in PowerSave algorithm */
	pTimer = os_timerCreate(hOs, rateAdaptation_rxTimeOut, pRateAdaptation);

	if (!pTimer)
	{
		utils_nullMemoryFree(hOs, pRateAdaptation, sizeof(rateAdaptation_t));
	    WLAN_OS_REPORT(("FATAL ERROR: rateAdaptation_create(): Error Creating rateAdaptation Module - Aborting\n"));
		return NULL;
	}


	/* reset RateAdaptation module block */
	os_memoryZero(hOs, pRateAdaptation, (sizeof(rateAdaptation_t)));

	pRateAdaptation->pTimer = pTimer;

	pRateAdaptation->hOs = hOs;

	return(pRateAdaptation);

}

/***************************************************************************
*						rateAdaptation_config					       
****************************************************************************
* DESCRIPTION:	This function initialize the Rate Adaptation algorithm
*
* INPUTS:		pRateAdaptation - the object
*				hOs - Handle to the Os Abstraction Layer
*				hReport - Handle to the Report object
*				rateAdaptationInitParam - pointer to Rate Adaptation
*										  module init parameters
*
* OUTPUT:			
* 
* RETURNS:		void
***************************************************************************/
TI_STATUS rateAdaptation_config(rateAdaptation_t*			pRateAdaptation, 
	   							TI_HANDLE					hOs, 
								TI_HANDLE					hReport, 
								TI_HANDLE					hCtrlData, 
                                TI_HANDLE					hEvHandler,
								TI_HANDLE					hAPConnection,
								rateAdaptationInitParam_t*	rateAdaptationInitParam)
{

	UINT32 i;

	if( (pRateAdaptation == NULL)  || (hOs == NULL) || 
		(hReport == NULL) ||  (rateAdaptationInitParam == NULL) )
	{
	    WLAN_OS_REPORT(("FATAL ERROR: rateAdaptation_config(): Parameters Error - Aborting\n"));
		return NOK;
	}

	pRateAdaptation->hOs = hOs;
	pRateAdaptation->hReport = hReport;
	pRateAdaptation->hCtrlData = hCtrlData;
    pRateAdaptation->hEvHandler = hEvHandler;
	pRateAdaptation->hAPConnection = hAPConnection;

	pRateAdaptation->contTxPacketsThreshold = rateAdaptationInitParam->contTxPacketsThreshold; 
	pRateAdaptation->stepUpTxPacketsThreshold = rateAdaptationInitParam->stepUpTxPacketsThreshold; 
	pRateAdaptation->ctrlDataFBShortInterval = rateAdaptationInitParam->ctrlDataFBShortInterval;
	pRateAdaptation->ctrlDataFBLongInterval = rateAdaptationInitParam->ctrlDataFBLongInterval;
	pRateAdaptation->lowRateThreshold = DEF_LOW_RATE_THRESHOLD;

	pRateAdaptation->rateAdapt_timeout = 1000*rateAdaptationInitParam->rateAdapt_timeout;

	pRateAdaptation->txCount = 0;
	pRateAdaptation->txSkipCount = 0;
	pRateAdaptation->txFailCount = 0;
    pRateAdaptation->txRateFallBackCount = 0;

	pRateAdaptation->stepUpFlag = FALSE;

	/* resset Tspecs Rate Parameters */
	for(i = 0 ; i < MAX_NUM_OF_AC ; i++)
	{
		pRateAdaptation->tspecsRateParameters[i].enableEvent = FALSE;
		if(rateAdaptationInitParam->tspecsRateParameters[i].highRateThreshold < rateAdaptationInitParam->tspecsRateParameters[i].lowRateThreshold)
		{
			WLAN_REPORT_ERROR(pRateAdaptation->hReport, RATE_ADAPTATION_MODULE_LOG,
					(" rateAdaptation_config() ERROR: highRateThreshold < lowRateThreshold,  ac = %d\n",i));

			pRateAdaptation->tspecsRateParameters[i].highRateThreshold = 0;
			pRateAdaptation->tspecsRateParameters[i].lowRateThreshold = 0;
		
		}
		/* if either one of the threshold is zero all threshold should be with zero default value */
		else if( (rateAdaptationInitParam->tspecsRateParameters[i].highRateThreshold == 0) ||
				 (rateAdaptationInitParam->tspecsRateParameters[i].lowRateThreshold == 0))
		{
			pRateAdaptation->tspecsRateParameters[i].highRateThreshold = 0;
			pRateAdaptation->tspecsRateParameters[i].lowRateThreshold = 0;
		}
		else
		{
			pRateAdaptation->tspecsRateParameters[i].highRateThreshold = rateAdaptationInitParam->tspecsRateParameters[i].highRateThreshold;
			pRateAdaptation->tspecsRateParameters[i].lowRateThreshold = rateAdaptationInitParam->tspecsRateParameters[i].lowRateThreshold;
		}
	}

	WLAN_REPORT_INFORMATION(pRateAdaptation->hReport, RATE_ADAPTATION_MODULE_LOG,  
			(" Rate Adaptation initialize success\n"));

	return OK;

}

/***************************************************************************
*							rateAdaptation_destroy			                   
****************************************************************************
* DESCRIPTION:	This function destroy the rateAdaptation object. 
* 
* INPUTS:		rateAdaptation - the object
* 
* RETURNS:		OK - Unload succesfull
*				NOK - Unload unsuccesfull
***************************************************************************/

TI_STATUS rateAdaptation_destroy(rateAdaptation_t* pRateAdaptation)	
{

	/* check parameters validity */
	if( pRateAdaptation == NULL )
	{
		WLAN_REPORT_ERROR(pRateAdaptation->hReport, RATE_ADAPTATION_MODULE_LOG,
			(" rateAdaptation_destroy() : parametrs value error \n"));
		return NOK;
	}

	/* free timer */
	os_timerStop(pRateAdaptation->hOs, pRateAdaptation->pTimer);
	utils_nullTimerDestroy(pRateAdaptation->hOs, pRateAdaptation->pTimer);


	/* free rateAdaptation block */
	os_memoryFree(pRateAdaptation->hOs, pRateAdaptation, sizeof(rateAdaptation_t));

	return OK;
}

/***************************************************************************
*							rateAdaptation_rxTimeOut		                   
****************************************************************************
* DESCRIPTION:	
****************************************************************************/

static void rateAdaptation_rxTimeOut(TI_HANDLE hRateAdaptation)
{
   rateAdaptation_t* pRateAdaptation = (rateAdaptation_t *) hRateAdaptation;
   	UINT8		prevIndex = pRateAdaptation->currRateIndex,i;
	OS_802_11_THRESHOLD_CROSS_INDICATION_PARAMS	tspecRateCross;

	os_timerStop(pRateAdaptation->hOs, pRateAdaptation->pTimer);
	os_timerStart(pRateAdaptation->hOs, pRateAdaptation->pTimer,pRateAdaptation->rateAdapt_timeout,FALSE);

	pRateAdaptation->txCount = 0;
	pRateAdaptation->txSkipCount = 0;
	pRateAdaptation->txFailCount = 0;
    pRateAdaptation->txRateFallBackCount = 0;
	pRateAdaptation->currRateIndex = pRateAdaptation->maxRateIndex;

	
   	/* update OS with the current rate */
	if(prevIndex != pRateAdaptation->currRateIndex)
	{
		UINT32					statusData;
		paramInfo_t				param;

	    pRateAdaptation->expirTimeTick = os_timeStampMs(pRateAdaptation->hOs) + pRateAdaptation->ctrlDataFBShortInterval;
	    pRateAdaptation->stepUpFlag = TRUE;

        WLAN_REPORT_WARNING(pRateAdaptation->hReport, RATE_ADAPTATION_MODULE_LOG,
			(" RateAdapt() : Time: %d, (60sec) OldRate(Index,Rate): %d,%d, NewRate(Index,Rate): %d,%d\n",
            os_timeStampMs(pRateAdaptation->hOs),
			prevIndex,pRateAdaptation->RatesMap[prevIndex].rateNumber,
			pRateAdaptation->currRateIndex, pRateAdaptation->RatesMap[pRateAdaptation->currRateIndex].rateNumber ));

        /* update OS with the current rate */
		param.paramType = CTRL_DATA_FOUR_X_CURRRENT_STATUS_PARAM;
		param.content.ctrlDataCerruentFourXstate = pRateAdaptation->RatesMap[pRateAdaptation->currRateIndex].fourXEnable;
		ctrlData_setParam(pRateAdaptation->hCtrlData, &param);

    	/* update OS with the current rate */

		statusData = hostToUtilityRate(pRateAdaptation->RatesMap[pRateAdaptation->currRateIndex].rate);

        EvHandlerSendEvent(pRateAdaptation->hEvHandler, IPC_EVENT_LINK_SPEED, (UINT8 *)&statusData,sizeof(UINT32));
		
		/* send Tspecs Rate Event */
		for(i = 0 ; i < MAX_NUM_OF_AC ; i++)
		{
			if(pRateAdaptation->tspecsRateParameters[i].enableEvent == FALSE)
			{
				continue;
			}
			else
			{
				if( ((pRateAdaptation->RatesMap[prevIndex].rateNumber) < (pRateAdaptation->tspecsRateParameters[i].highRateThreshold)) &&
					(pRateAdaptation->RatesMap[pRateAdaptation->currRateIndex].rateNumber >= (pRateAdaptation->tspecsRateParameters[i].highRateThreshold)) )
				{
					tspecRateCross.uAC = i;
					tspecRateCross.uHighOrLowThresholdFlag = HIGH_THRESHOLD_CROSS;
                    tspecRateCross.uAboveOrBelowFlag = CROSS_ABOVE;
					EvHandlerSendEvent(pRateAdaptation->hEvHandler, IPC_EVENT_TSPEC_RATE_STATUS, (UINT8 *)&tspecRateCross,sizeof(tspecRateCross));
				}
                else
                if( ((pRateAdaptation->RatesMap[prevIndex].rateNumber) < (pRateAdaptation->tspecsRateParameters[i].lowRateThreshold)) &&
						(hostRateToNumber((rate_e)pRateAdaptation->RatesMap[pRateAdaptation->currRateIndex].rateNumber) >= (pRateAdaptation->tspecsRateParameters[i].lowRateThreshold)) )
					{
						tspecRateCross.uAC = i;
						tspecRateCross.uHighOrLowThresholdFlag = LOW_THRESHOLD_CROSS;
                        tspecRateCross.uAboveOrBelowFlag = CROSS_ABOVE;
						EvHandlerSendEvent(pRateAdaptation->hEvHandler, IPC_EVENT_TSPEC_RATE_STATUS, (UINT8 *)&tspecRateCross,sizeof(OS_802_11_THRESHOLD_CROSS_INDICATION_PARAMS));
					}

			}
		}
	}
    else {
        /* no change */
        pRateAdaptation->expirTimeTick = os_timeStampMs(pRateAdaptation->hOs) + pRateAdaptation->ctrlDataFBLongInterval;
	    pRateAdaptation->stepUpFlag = FALSE;
    }

}
/***************************************************************************
*							rateAdaptation_getCurrent		                   
****************************************************************************
* DESCRIPTION:	get current state - Rate and Modulation
*               Note: since a pointer to the rates map is returned,
*               Its content should be treated as READ ONLY!!!                
****************************************************************************/
rateModulation4x_table_t* rateAdaptation_getCurrent(rateAdaptation_t* pRateAdaptation)	
{
    return &(pRateAdaptation->RatesMap[pRateAdaptation->currRateIndex]);
}
		
/***************************************************************************
*						rateAdaptation_getCurrentRate		               
****************************************************************************
* DESCRIPTION:	get current Rate 
****************************************************************************/
rate_e rateAdaptation_getCurrentRate(rateAdaptation_t* pRateAdaptation)	
{
	return pRateAdaptation->RatesMap[pRateAdaptation->currRateIndex].rate;
}	
	
/***************************************************************************
*					rateAdaptation_getCurrentModulation		               *
****************************************************************************
* DESCRIPTION:	get current Modulation
****************************************************************************/
modulationType_e rateAdaptation_getCurrentModulation(rateAdaptation_t* pRateAdaptation)	
{
	return pRateAdaptation->RatesMap[pRateAdaptation->currRateIndex].modulation;
}		

/***************************************************************************
*					rateAdaptation_getCurrentFourXEnable	               *
****************************************************************************
* DESCRIPTION:	get current fourx status
****************************************************************************/
BOOL rateAdaptation_getCurrentFourXEnable(rateAdaptation_t* pRateAdaptation)	
{
	return pRateAdaptation->RatesMap[pRateAdaptation->currRateIndex].fourXEnable;
}

/***************************************************************************
*					rateAdaptation_buildRateMapTable		               *
****************************************************************************
* DESCRIPTION:	build the rate map table
****************************************************************************/
TI_STATUS rateAdaptation_buildRateMapTable(rateAdaptation_t		*pRateAdaptation,
												   ctrlData_rateAdapt_t *currTable,
												   UINT32				supportedBitMap,
												   UINT32				clientBitMap,
												   modulationType_e		modulation,
												   BOOL					enable4x,
												   bssType_e			bssType)
{
	UINT8 i = 0;
	UINT8 index = 0;
	UINT32					statusData;
	UINT8 fallBack,stepUp;

	/* 
	Note : allRates[] is changed due to the fact that rate_e was set in the 
	wrong order : 6,9 were in higher numeric value then 5.5 and 11 !!!
	 */
	rate_e	allRates[] =   {DRV_RATE_1M,
							DRV_RATE_2M,
							DRV_RATE_5_5M,
							DRV_RATE_6M,
							DRV_RATE_9M,
							DRV_RATE_11M,
							DRV_RATE_12M,
							DRV_RATE_18M,
							DRV_RATE_22M,
							DRV_RATE_24M,
							DRV_RATE_36M,
							DRV_RATE_48M,
							DRV_RATE_54M
	};

	while(i < DRV_RATE_MAX)
	{
		if(rateAdaptation_Utils_IsRateInBitmap(pRateAdaptation,
											  supportedBitMap,
											  allRates[i]) == OK)
		{
			/* update rates parameters */
			pRateAdaptation->RatesMap[index].rate = allRates[i];
			pRateAdaptation->RatesMap[index].rateNumber = hostRateToNumber(allRates[i]);

			if((rateAdaptation_isRateInTable(currTable,pRateAdaptation->RatesMap[index].rate)) &&
				(rateAdaptation_Utils_IsRateInBitmap(pRateAdaptation,
													clientBitMap,
													allRates[i]) == OK))
			{
				pRateAdaptation->RatesMap[index].valid = TRUE;
				rateAdaptation_getFallBackStepUp(currTable,pRateAdaptation->RatesMap[index].rate,&fallBack,&stepUp);
				pRateAdaptation->RatesMap[index].rateAdaptFallBack = fallBack;
				pRateAdaptation->RatesMap[index].rateAdaptStepUp = stepUp;
				
				/* update modulation parameter */
				pRateAdaptation->RatesMap[index].modulation = setModulationForRate(pRateAdaptation,
																				   pRateAdaptation->RatesMap[index].rate,
																				   modulation,bssType);

				/* update 4x enable parameter */
				pRateAdaptation->RatesMap[index].fourXEnable = set4xEnableForRate(pRateAdaptation,
					 															  pRateAdaptation->RatesMap[index].rate,
																				  enable4x, bssType);

				pRateAdaptation->maxRateIndex = index;
				pRateAdaptation->currRateIndex = index;
			}
			else
			{
				pRateAdaptation->RatesMap[index].valid = FALSE;
				pRateAdaptation->RatesMap[index].modulation = setModulationForRate(pRateAdaptation,
																				   pRateAdaptation->RatesMap[index].rate,
																				   modulation,bssType);
			}

			index++;
		}

		i++;
	}

	/* report the current rate to OS */
	statusData = hostToUtilityRate(pRateAdaptation->RatesMap[pRateAdaptation->currRateIndex].rate);

    EvHandlerSendEvent(pRateAdaptation->hEvHandler, IPC_EVENT_LINK_SPEED, (UINT8 *)&statusData,sizeof(UINT32));

	return OK;
}		

/***************************************************************************
*							buildRateBitMap					               *
****************************************************************************
* DESCRIPTION:	
****************************************************************************/

UINT32 rateAdaptation_Utils_buildRateBitMap(rateAdaptation_t	*pRateAdaptation,
											ctrlData_rateAdapt_t *currTable,
											rate_e			rate,
											UINT32			supportedBitMap,
											UINT32			clientBitMap)
{
	UINT32 buildRateBitMap = 0;
	UINT8 rateNumber = hostRateToNumber(rate);

	if( (rateNumber >= hostRateToNumber(DRV_RATE_1M)) && (rateAdaptation_isRateInTable(currTable, DRV_RATE_1M) ) ) buildRateBitMap |= (clientBitMap & supportedBitMap & DRV_RATE_MASK_1_BARKER);
	if( (rateNumber >= hostRateToNumber(DRV_RATE_2M)) && (rateAdaptation_isRateInTable(currTable, DRV_RATE_2M) ) )  buildRateBitMap |= (clientBitMap & supportedBitMap & DRV_RATE_MASK_2_BARKER);
	if( (rateNumber >= hostRateToNumber(DRV_RATE_5_5M)) && (rateAdaptation_isRateInTable(currTable, DRV_RATE_5_5M) ) ) buildRateBitMap |= (clientBitMap & supportedBitMap & DRV_RATE_MASK_5_5_CCK); 
	if( (rateNumber >= hostRateToNumber(DRV_RATE_11M)) && (rateAdaptation_isRateInTable(currTable, DRV_RATE_11M) ) ) buildRateBitMap |= (clientBitMap & supportedBitMap & DRV_RATE_MASK_11_CCK); 
	if( (rateNumber >= hostRateToNumber(DRV_RATE_22M)) && (rateAdaptation_isRateInTable(currTable, DRV_RATE_22M) ) ) buildRateBitMap |= (clientBitMap & supportedBitMap & DRV_RATE_MASK_22_PBCC); 
	if( (rateNumber >= hostRateToNumber(DRV_RATE_6M)) && (rateAdaptation_isRateInTable(currTable, DRV_RATE_6M) ) )  buildRateBitMap |= (clientBitMap & supportedBitMap & DRV_RATE_MASK_6_OFDM); 
	if( (rateNumber >= hostRateToNumber(DRV_RATE_9M)) && (rateAdaptation_isRateInTable(currTable, DRV_RATE_9M) ) )  buildRateBitMap |= (clientBitMap & supportedBitMap & DRV_RATE_MASK_9_OFDM); 
	if( (rateNumber >= hostRateToNumber(DRV_RATE_12M)) && (rateAdaptation_isRateInTable(currTable, DRV_RATE_12M) ) ) buildRateBitMap |= (clientBitMap & supportedBitMap & DRV_RATE_MASK_12_OFDM); 
	if( (rateNumber >= hostRateToNumber(DRV_RATE_18M)) && (rateAdaptation_isRateInTable(currTable, DRV_RATE_18M) ) ) buildRateBitMap |= (clientBitMap & supportedBitMap & DRV_RATE_MASK_18_OFDM); 
	if( (rateNumber >= hostRateToNumber(DRV_RATE_24M)) && (rateAdaptation_isRateInTable(currTable, DRV_RATE_24M) ) ) buildRateBitMap |= (clientBitMap & supportedBitMap & DRV_RATE_MASK_24_OFDM); 
	if( (rateNumber >= hostRateToNumber(DRV_RATE_36M)) && (rateAdaptation_isRateInTable(currTable, DRV_RATE_36M) ) ) buildRateBitMap |= (clientBitMap & supportedBitMap & DRV_RATE_MASK_36_OFDM);
	if( (rateNumber >= hostRateToNumber(DRV_RATE_48M)) && (rateAdaptation_isRateInTable(currTable, DRV_RATE_48M) ) ) buildRateBitMap |= (clientBitMap & supportedBitMap & DRV_RATE_MASK_48_OFDM); 
	if( (rateNumber >= hostRateToNumber(DRV_RATE_54M)) && (rateAdaptation_isRateInTable(currTable, DRV_RATE_54M) ) ) buildRateBitMap |= (clientBitMap & supportedBitMap & DRV_RATE_MASK_54_OFDM);
	
	return buildRateBitMap;
}

/***************************************************************************
*							rateAdaptation_isRateInTable					               *
****************************************************************************
* DESCRIPTION:	check if a specific rate is in a rates table 
****************************************************************************/

BOOL rateAdaptation_isRateInTable(ctrlData_rateAdapt_t *currTable,
										rate_e			rate)
{
	UINT8 i = 0;

	while(i <= currTable->len)
	{
		if(currTable->rateAdaptRatesTable[i++] == rate)
			return TRUE;
	}
	return FALSE;
}

/***************************************************************************
*							rateAdaptation_getFallBackStepUp					               *
****************************************************************************
* DESCRIPTION:	return Fall Back & Step UP values of a certain rate 
****************************************************************************/

void rateAdaptation_getFallBackStepUp(ctrlData_rateAdapt_t *currTable,
										rate_e			rate,UINT8* FB,UINT8* SU)
{
	UINT8 i = 0;

	while(i <= currTable->len)
	{
		if(currTable->rateAdaptRatesTable[i++] == rate)
		{
			*FB = currTable->rateAdaptFBTable[--i];
			*SU = currTable->rateAdaptSUTable[i];
			return;
		}
	}

	*FB = 0;
	*SU = 0;
}


/***************************************************************************
*							IsRateInBitmap					               *
****************************************************************************
* DESCRIPTION:	check if a specific rate is in a rates bitmap 
****************************************************************************/
TI_STATUS rateAdaptation_Utils_IsRateInBitmap(rateAdaptation_t	*pRateAdaptation,
											UINT32				ratesBitMap,
											rate_e				rate)
{
	if( ( (rate == DRV_RATE_1M) && (ratesBitMap & DRV_RATE_MASK_1_BARKER) )||
		( (rate == DRV_RATE_2M) && (ratesBitMap & DRV_RATE_MASK_2_BARKER) ) ||
		( (rate == DRV_RATE_5_5M) && (ratesBitMap & DRV_RATE_MASK_5_5_CCK) ) ||
		( (rate == DRV_RATE_11M) && (ratesBitMap & DRV_RATE_MASK_11_CCK) ) ||
		( (rate == DRV_RATE_22M) && (ratesBitMap & DRV_RATE_MASK_22_PBCC) ) ||
		( (rate == DRV_RATE_6M) && (ratesBitMap & DRV_RATE_MASK_6_OFDM) ) ||
		( (rate == DRV_RATE_9M) && (ratesBitMap & DRV_RATE_MASK_9_OFDM) ) ||
		( (rate == DRV_RATE_12M) && (ratesBitMap & DRV_RATE_MASK_12_OFDM) ) ||
		( (rate == DRV_RATE_18M) && (ratesBitMap & DRV_RATE_MASK_18_OFDM) ) ||
		( (rate == DRV_RATE_24M) && (ratesBitMap & DRV_RATE_MASK_24_OFDM) ) ||
		( (rate == DRV_RATE_36M) && (ratesBitMap & DRV_RATE_MASK_36_OFDM) ) ||
		( (rate == DRV_RATE_48M) && (ratesBitMap & DRV_RATE_MASK_48_OFDM) ) ||
		( (rate == DRV_RATE_54M) && (ratesBitMap & DRV_RATE_MASK_54_OFDM) ) )
	{
		return OK;
	}

	return NOK;
}
/***************************************************************************
*							setModulationForRate			               *
****************************************************************************
* DESCRIPTION:	set modulation for the rate map table
****************************************************************************/

static modulationType_e setModulationForRate(rateAdaptation_t	*pRateAdaptation,
   											 rate_e				rate,
  											 modulationType_e	modulation,
											 bssType_e			bssType)
{
	switch(rate)
	{
		case DRV_RATE_1M:
			return DRV_MODULATION_QPSK;

		case DRV_RATE_2M:
			return DRV_MODULATION_QPSK;

		case DRV_RATE_5_5M:
			if( (modulation == DRV_MODULATION_PBCC) && 
				(bssType == BSS_INFRASTRUCTURE) )
				return DRV_MODULATION_PBCC;
			else
				return DRV_MODULATION_CCK;

		case DRV_RATE_11M:
			if( (modulation == DRV_MODULATION_PBCC) && 
				(bssType == BSS_INFRASTRUCTURE) )
				return DRV_MODULATION_PBCC;
			else
				return DRV_MODULATION_CCK;

		case DRV_RATE_22M:
			return DRV_MODULATION_PBCC;

		case DRV_RATE_6M:
			return DRV_MODULATION_OFDM;

		case DRV_RATE_9M:
			return DRV_MODULATION_OFDM;

		case DRV_RATE_18M:
			return DRV_MODULATION_OFDM;

		case DRV_RATE_12M:
			return DRV_MODULATION_OFDM;

		case DRV_RATE_24M:
			return DRV_MODULATION_OFDM;

		case DRV_RATE_36M:
			return DRV_MODULATION_OFDM;

		case DRV_RATE_48M:
			return DRV_MODULATION_OFDM;

		case DRV_RATE_54M:
			return DRV_MODULATION_OFDM;

		default:
			WLAN_REPORT_ERROR(pRateAdaptation->hReport, RATE_ADAPTATION_MODULE_LOG,  
						(" setModulationForRate(): unKnown rate !!! \n"));
			return DRV_MODULATION_NONE;

	}
}

static BOOL set4xEnableForRate(rateAdaptation_t*	pRateAdaptation, 
							   rate_e				rate,
   							   BOOL					enable4x, 
							   bssType_e			bssType)
{
	if(bssType == BSS_INDEPENDENT)
		return FALSE;

	switch(rate)
	{
		case DRV_RATE_1M:
			return FALSE;

		case DRV_RATE_2M:
			return FALSE;

		case DRV_RATE_5_5M:
			return FALSE;

		case DRV_RATE_11M:
			return enable4x;

		case DRV_RATE_22M:
			return enable4x;

		case DRV_RATE_6M:
			return FALSE;

		case DRV_RATE_9M:
			return FALSE;

		case DRV_RATE_18M:
			return enable4x;

		case DRV_RATE_12M:
			return enable4x;

		case DRV_RATE_24M:
			return enable4x;

		case DRV_RATE_36M:
			return enable4x;

		case DRV_RATE_48M:
			return enable4x;

		case DRV_RATE_54M:
			return enable4x;

		default:
			WLAN_REPORT_ERROR(pRateAdaptation->hReport, RATE_ADAPTATION_MODULE_LOG,  
						(" set4xEnableForRate(): unKnown rate !!! \n"));
			return DRV_MODULATION_NONE;

	}


}
/***************************************************************************
*					rateAdaptation_setMaxActivRate		               
****************************************************************************
* DESCRIPTION:	set current rate
****************************************************************************/
TI_STATUS rateAdaptation_setMaxActivRate(rateAdaptation_t* pRateAdaptation, rate_e rate)	
{
	UINT8  index;
	UINT32 status;

	status = rateAdaptationa_rateToIndexInRateMapTable(pRateAdaptation, rate, &index);
	
	if(status != OK)
		return NOK;

	pRateAdaptation->maxRateIndex = index;

	return OK;
}

/***************************************************************************
*					rateAdaptation_configLowRateThrsh		               
****************************************************************************
* DESCRIPTION:	set low rate threshold
****************************************************************************/
TI_STATUS rateAdaptation_configLowRateThrsh(rateAdaptation_t *pRateAdaptation, UINT8 rate)	
{
	pRateAdaptation->lowRateThreshold = rate;

	WLAN_REPORT_INFORMATION(pRateAdaptation->hReport, RATE_ADAPTATION_MODULE_LOG,  
			(" \nrateAdaptation_configLowRateThrsh:rate  %d, translated to %d \n", rate, pRateAdaptation->lowRateThreshold));

	return OK;
}

/***************************************************************************
*					rateAdaptation_updateModulation		               
****************************************************************************
* DESCRIPTION:	set current rate
****************************************************************************/
void rateAdaptation_updateModulation(rateAdaptation_t* pRateAdaptation,
									 modulationType_e modulation,
									 bssType_e bssType)
{
	UINT8 index = 0;

	for(index = 0 ; index < pRateAdaptation->maxRateIndex ; index++)
	{
		pRateAdaptation->RatesMap[index].modulation = setModulationForRate(pRateAdaptation,
   																			pRateAdaptation->RatesMap[index].rate,
  																			modulation,
																			bssType);
	}
}
/***************************************************************************
*					rateAdaptation_updateModulation		               
****************************************************************************
* DESCRIPTION:	set current rate
****************************************************************************/
void rateAdaptation_update4xEnable(rateAdaptation_t* pRateAdaptation,
								   BOOL				 enable4x,
								   bssType_e		 bssType)
{
	UINT8 index = 0;

	for(index = 0 ; index < pRateAdaptation->maxRateIndex ; index++)
	{
		pRateAdaptation->RatesMap[index].fourXEnable = set4xEnableForRate(pRateAdaptation,
																		  pRateAdaptation->RatesMap[index].rate,
																		  enable4x, bssType);
 
	}
}
/***************************************************************************
*						ctrlData_rateAdaptationStart				       
****************************************************************************
* DESCRIPTION:	This function start the Rate Adaptation algorithm
***************************************************************************/
TI_STATUS rateAdaptation_start(rateAdaptation_t* pRateAdaptation)
{
	UINT32					statusData;

	pRateAdaptation->txCount = 0;
	pRateAdaptation->txSkipCount = 0;
	pRateAdaptation->txFailCount = 0;
    pRateAdaptation->txRateFallBackCount = 0;

	pRateAdaptation->stepUpFlag = FALSE;

	os_timerStart(pRateAdaptation->hOs, pRateAdaptation->pTimer,pRateAdaptation->rateAdapt_timeout,FALSE);

	pRateAdaptation->expirTimeTick = os_timeStampMs(pRateAdaptation->hOs) + pRateAdaptation->ctrlDataFBLongInterval;

   	/* update OS with the current rate */
	statusData = hostToUtilityRate(pRateAdaptation->RatesMap[pRateAdaptation->currRateIndex].rate);

    EvHandlerSendEvent(pRateAdaptation->hEvHandler, IPC_EVENT_LINK_SPEED, (UINT8 *)&statusData,sizeof(UINT32));

	return OK;
}

/***************************************************************************
*						ctrlData_rateAdaptationStop					       
****************************************************************************
* DESCRIPTION:	This function stop the rate adaptation algorithm
***************************************************************************/
TI_STATUS rateAdaptation_stop(rateAdaptation_t* pRateAdaptation)
{
	os_timerStop(pRateAdaptation->hOs, pRateAdaptation->pTimer);

	pRateAdaptation->txCount = 0;
	pRateAdaptation->txSkipCount = 0;
	pRateAdaptation->txFailCount = 0;
    pRateAdaptation->txRateFallBackCount = 0;
	pRateAdaptation->stepUpFlag = FALSE;
	pRateAdaptation->expirTimeTick = 0;

	pRateAdaptation->currRateIndex = pRateAdaptation->maxRateIndex;


	return OK;
}

/***************************************************************************
*						rateAdaptation_stopTimer					       
****************************************************************************
* DESCRIPTION:	This function stop the rate adaptation timer
***************************************************************************/
TI_STATUS rateAdaptation_stopTimer(rateAdaptation_t* pRateAdaptation)
{
	os_timerStop(pRateAdaptation->hOs, pRateAdaptation->pTimer);

	return OK;
}


/***************************************************************************
*					rateAdaptation_setCurrentRate		               
****************************************************************************
* DESCRIPTION:	set current rate
****************************************************************************/
TI_STATUS rateAdaptation_setCurrentRate(rateAdaptation_t* pRateAdaptation,rate_e rate)
{
	UINT8					index;
	UINT32					statusData;

	if(rateAdaptationa_rateToIndexInRateMapTable(pRateAdaptation, rate, &index) == OK)
		pRateAdaptation->currRateIndex = index;
	else
		pRateAdaptation->currRateIndex = pRateAdaptation->maxRateIndex;

	/* report the current rate to OS */

	statusData = hostToUtilityRate(pRateAdaptation->RatesMap[pRateAdaptation->currRateIndex].rate);

    EvHandlerSendEvent(pRateAdaptation->hEvHandler, IPC_EVENT_LINK_SPEED, (UINT8 *)&statusData,sizeof(UINT32));

	return OK;
}

/***************************************************************************
*					ctrlData_rateToIndexInRateMapTable				       
****************************************************************************
* DESCRIPTION:	This function convert a specific rate to index in the 
*				Rate Adaptation algorithm ratemap table
*
* INPUTS:		pRateAdaptation - the object
*				rate - the rate to convert
*
* OUTPUT:		Index - the index of the rate in the table.	
* 
* RETURNS:		If the rate is not in the table - return NOK otherwise OK
***************************************************************************/
static TI_STATUS rateAdaptationa_rateToIndexInRateMapTable(rateAdaptation_t* pRateAdaptation, 
														   rate_e rate, UINT8* Index)
{
	UINT8 i;

	for(i = 0 ; i <= pRateAdaptation->maxRateIndex ; i++)
	{
		if(pRateAdaptation->RatesMap[i].rate == rate)
		{
			*Index = i;
			return OK;
		}
	}

	return NOK;
}

/***************************************************************************
*					getPrevRate					       
****************************************************************************
* DESCRIPTION:	
*
* INPUTS:		pRateAdaptation - the object
*
* RETURNS:		new rate
****************************************************************************/
UINT8 getPrevRate(rateAdaptation_t *pRateAdaptation)
{
    INT16 i;
    if(pRateAdaptation->currRateIndex != 0)
    {
        for(i = pRateAdaptation->currRateIndex - 1; i > 0; i--)
        {
            if(pRateAdaptation->RatesMap[i].valid)
                return (UINT8)i;
        }
    }
    return pRateAdaptation->currRateIndex;
}

 

/***************************************************************************
*					getNextRate					       
****************************************************************************
* DESCRIPTION:	
*
* INPUTS:		pRateAdaptation - the object
*
* RETURNS:		new rate
****************************************************************************/
UINT8 getNextRate(rateAdaptation_t *pRateAdaptation)
{
	UINT32 i;

	for(i = pRateAdaptation->currRateIndex + 1; i <= pRateAdaptation->maxRateIndex; i++)
	{
		if(pRateAdaptation->RatesMap[i].valid)
			return i;
	}

	return pRateAdaptation->currRateIndex;
}

/***************************************************************************
*					rateAdaptation_updateRateAdaptation					       
****************************************************************************
* DESCRIPTION:	This function perform the Rate Adaptation algorithm. It
*				called for every tx complete.
*
* INPUTS:		pRateAdaptation - the object
*				actualTxRate - the actual tx rate
*				requestTxRate - the request tx rate
*				TxStatus - the tx status
*               txNumWaiting - the number of descriptors in the HW with previous rate
*
* RETURNS:		OK/NOK
****************************************************************************/
TI_STATUS rateAdaptation_updateRateAdaptation(rateAdaptation_t* pRateAdaptation,
											  rate_e			actualTxRate, 
											  rate_e			requestTxRate, 
											  UINT32			TxStatus,
                                              int               txNumWaiting)
{
	
	UINT32		txFailPercent;
	UINT32		timeStamp;
	UINT8		actualTxRateNumber = hostRateToNumber(actualTxRate);
	UINT8		requestTxRateNumber = hostRateToNumber(requestTxRate);
	OS_802_11_THRESHOLD_CROSS_INDICATION_PARAMS	tspecRateCross;
	UINT8		i,prevIndex = pRateAdaptation->currRateIndex;

    /* Check if need to skip some packets due to rate update */
    if (pRateAdaptation->txSkipCount > 0) {
        pRateAdaptation->txSkipCount--;
        return OK;
    }

    timeStamp = os_timeStampMs(pRateAdaptation->hOs);
	
	pRateAdaptation->txCount++;

    /* if the TxStatus wasn't SUCCESS or the rate was falling back - we will update the mechanism, unless */
	/* the TxStatus is LIFETIME_EXCEEDED and there were no retries - we won't update the mechanism        */
	if(((TxStatus != SEND_COMPLETE_SUCCESS) && (TxStatus != SEND_COMPLETE_LIFETIME_EXCEEDED)) 
        || (actualTxRateNumber < requestTxRateNumber)) 
	{
		pRateAdaptation->txFailCount++;
        if (actualTxRateNumber < requestTxRateNumber)
           pRateAdaptation->txRateFallBackCount++;
	}

	/* Verify if the checking time has come. */
	if( TS_EXCEEDS(timeStamp, pRateAdaptation->expirTimeTick) )
	{
		/* If step up has just done wait for 10 packets only. */ 
		if((pRateAdaptation->stepUpFlag == TRUE) && 
			(pRateAdaptation->txCount > pRateAdaptation->stepUpTxPacketsThreshold ))
		{
			/* Calculate the packet fail percent. */
			txFailPercent = (pRateAdaptation->txFailCount * 100) / pRateAdaptation->txCount;
			
			pRateAdaptation->stepUpFlag = FALSE; /* Flag is off for next time. */
			
			if( (txFailPercent > pRateAdaptation->RatesMap[pRateAdaptation->currRateIndex].rateAdaptFallBack) && 
				(pRateAdaptation->currRateIndex > 0) )
			{
				pRateAdaptation->currRateIndex = getPrevRate(pRateAdaptation);
			}
		}
		else if (pRateAdaptation->txCount > pRateAdaptation->contTxPacketsThreshold )
		{ 
			/* Calculate the packet fail percents. */
			txFailPercent = (pRateAdaptation->txFailCount * 100) / pRateAdaptation->txCount;
			
			if ( (txFailPercent > pRateAdaptation->RatesMap[pRateAdaptation->currRateIndex].rateAdaptFallBack) && 
				(pRateAdaptation->currRateIndex > 0) )
			{
                pRateAdaptation->currRateIndex = getPrevRate(pRateAdaptation);
			}
			else if ((txFailPercent < pRateAdaptation->RatesMap[pRateAdaptation->currRateIndex].rateAdaptStepUp) && 
				(pRateAdaptation->currRateIndex < pRateAdaptation->maxRateIndex))
			{
				pRateAdaptation->currRateIndex = getNextRate(pRateAdaptation);
				pRateAdaptation->stepUpFlag = TRUE; /* Flag is on for next time. */
			}
		} 
		else
		{
			return OK;
		}
		
		/* update OS with the current rate */
		if(prevIndex != pRateAdaptation->currRateIndex)
		{
			UINT32			statusData;
			paramInfo_t		param;
			rate_e			currentRate = pRateAdaptation->RatesMap[pRateAdaptation->currRateIndex].rate;
			UINT8			currentRateNumber = pRateAdaptation->RatesMap[pRateAdaptation->currRateIndex].rateNumber;

		    WLAN_REPORT_WARNING(pRateAdaptation->hReport, RATE_ADAPTATION_MODULE_LOG,
			    (" RateAdapt() : Time: %d, Fail: %d, Count: %d,Skip: %d\n",
                timeStamp,pRateAdaptation->txFailCount,pRateAdaptation->txCount,txNumWaiting));
			
			WLAN_REPORT_WARNING(pRateAdaptation->hReport, RATE_ADAPTATION_MODULE_LOG,
			    (" OldRate(Index,Rate): %d,%d\n ",prevIndex,pRateAdaptation->RatesMap[prevIndex].rateNumber));
			WLAN_REPORT_WARNING(pRateAdaptation->hReport, RATE_ADAPTATION_MODULE_LOG,
				(" NewRate(Index,Rate): %d,%d\n ",pRateAdaptation->currRateIndex,currentRateNumber));

			/* If the current rate is lower or equal to the roaming threshold, issue roaming trigger. */
			if (currentRateNumber <= pRateAdaptation->lowRateThreshold)
			{
				roamingEventData_u RoamingEventData;

				RoamingEventData.rate = currentRate;
				apConn_reportRoamingEvent(pRateAdaptation->hAPConnection, ROAMING_TRIGGER_LOW_TX_RATE, &RoamingEventData);
			}
            
			/* Rate update - initialize the skip counter for packets that are already in the queue with old rates */
            if ((txNumWaiting >= 0) && (txNumWaiting <= 100)) {
                /* In reasonable range */
                pRateAdaptation->txSkipCount = (UINT32)txNumWaiting;
            } else {
                pRateAdaptation->txSkipCount = 30; /* something happened, at least skip 30 packets */
            }

    		/* update OS with the current rate */
			param.paramType = CTRL_DATA_FOUR_X_CURRRENT_STATUS_PARAM;
			param.content.ctrlDataCerruentFourXstate = pRateAdaptation->RatesMap[pRateAdaptation->currRateIndex].fourXEnable;
			ctrlData_setParam(pRateAdaptation->hCtrlData, &param);

    		/* update OS with the current rate */

			statusData = hostToUtilityRate(currentRate);

            EvHandlerSendEvent(pRateAdaptation->hEvHandler, IPC_EVENT_LINK_SPEED, (UINT8 *)&statusData,sizeof(UINT32));

			/* send Tspecs Rate Event */
			for(i = 0 ; i < MAX_NUM_OF_AC ; i++)
			{
				if(pRateAdaptation->tspecsRateParameters[i].enableEvent == FALSE)
				{
					continue;
				}
				else
				{
					if ( (pRateAdaptation->RatesMap[prevIndex].rateNumber < pRateAdaptation->tspecsRateParameters[i].highRateThreshold) &&
						(currentRateNumber >= pRateAdaptation->tspecsRateParameters[i].highRateThreshold) )
					{
						tspecRateCross.uAC = i;
						tspecRateCross.uHighOrLowThresholdFlag = HIGH_THRESHOLD_CROSS;
                        tspecRateCross.uAboveOrBelowFlag = CROSS_ABOVE;
						EvHandlerSendEvent(pRateAdaptation->hEvHandler, IPC_EVENT_TSPEC_RATE_STATUS, (UINT8 *)&tspecRateCross,sizeof(OS_802_11_THRESHOLD_CROSS_INDICATION_PARAMS));
					}
					else
					if( (pRateAdaptation->RatesMap[prevIndex].rateNumber >= pRateAdaptation->tspecsRateParameters[i].highRateThreshold) &&
						(currentRateNumber < pRateAdaptation->tspecsRateParameters[i].highRateThreshold) )
					{
						tspecRateCross.uAC = i;
						tspecRateCross.uHighOrLowThresholdFlag = HIGH_THRESHOLD_CROSS;
                        tspecRateCross.uAboveOrBelowFlag = CROSS_BELOW;
						EvHandlerSendEvent(pRateAdaptation->hEvHandler, IPC_EVENT_TSPEC_RATE_STATUS, (UINT8 *)&tspecRateCross,sizeof(OS_802_11_THRESHOLD_CROSS_INDICATION_PARAMS));
					}
                    else
					if( (pRateAdaptation->RatesMap[prevIndex].rateNumber >= pRateAdaptation->tspecsRateParameters[i].lowRateThreshold) &&
						(currentRateNumber < pRateAdaptation->tspecsRateParameters[i].lowRateThreshold) )
					{
						tspecRateCross.uAC = i;
						tspecRateCross.uHighOrLowThresholdFlag = LOW_THRESHOLD_CROSS;
                        tspecRateCross.uAboveOrBelowFlag = CROSS_BELOW;
						EvHandlerSendEvent(pRateAdaptation->hEvHandler, IPC_EVENT_TSPEC_RATE_STATUS, (UINT8 *)&tspecRateCross,sizeof(OS_802_11_THRESHOLD_CROSS_INDICATION_PARAMS));
					}
                    else
					if( (pRateAdaptation->RatesMap[prevIndex].rateNumber < pRateAdaptation->tspecsRateParameters[i].lowRateThreshold) &&
						(currentRateNumber >= pRateAdaptation->tspecsRateParameters[i].lowRateThreshold) )
					{
						tspecRateCross.uAC = i;
						tspecRateCross.uHighOrLowThresholdFlag = LOW_THRESHOLD_CROSS;
                        tspecRateCross.uAboveOrBelowFlag = CROSS_ABOVE;
						EvHandlerSendEvent(pRateAdaptation->hEvHandler, IPC_EVENT_TSPEC_RATE_STATUS, (UINT8 *)&tspecRateCross,sizeof(OS_802_11_THRESHOLD_CROSS_INDICATION_PARAMS));
					}

				}
			}

		}
		
		pRateAdaptation->txCount = 0;
		pRateAdaptation->txFailCount = 0;
        pRateAdaptation->txRateFallBackCount = 0;
		TS_ADVANCE( timeStamp, pRateAdaptation->expirTimeTick,
			(pRateAdaptation->stepUpFlag ? pRateAdaptation->ctrlDataFBShortInterval : pRateAdaptation->ctrlDataFBLongInterval)); 

	}
	
	return OK;	
} 

/***************************************************************************
*					rateAdaptation_setTspecsRateParams					       
****************************************************************************
* DESCRIPTION:	
*
* INPUTS:		pRateAdaptation - the object
**
* RETURNS:		OK/NOK
****************************************************************************/

void rateAdaptation_setTspecsRateEvent(rateAdaptation_t* pRateAdaptation,
											 UINT8			acID,
											BOOL			enableEvent)
{

   /* Prevent ENABLE_EVENTS if one of the threshold of that AC is ZERO */
  if (((pRateAdaptation->tspecsRateParameters[acID].highRateThreshold == 0) ||
	 (pRateAdaptation->tspecsRateParameters[acID].lowRateThreshold == 0)) && (enableEvent == TRUE))
   return;

	pRateAdaptation->tspecsRateParameters[acID].enableEvent = enableEvent;
}

/***************************************************************************
*					rateAdaptation_setTspecsRateThresholds					       
****************************************************************************
* DESCRIPTION:	
*
* INPUTS:		pRateAdaptation - the object
**
* RETURNS:		OK/NOK
****************************************************************************/

void rateAdaptation_setTspecsRateThresholds(rateAdaptation_t* pRateAdaptation,
											 UINT8			acID,
											 UINT8			highRateThreshold,
											 UINT8			lowRateThreshold)
{	
		if(highRateThreshold < lowRateThreshold)
		{
			WLAN_REPORT_ERROR(pRateAdaptation->hReport, RATE_ADAPTATION_MODULE_LOG,
					(" rateAdaptation_setTspecsRateThresholds() ERROR: highRateThreshold < lowRateThreshold,  ac = %d\n",acID));

			pRateAdaptation->tspecsRateParameters[acID].highRateThreshold = 0;
			pRateAdaptation->tspecsRateParameters[acID].lowRateThreshold = 0;
		
		}
		else
		{
			pRateAdaptation->tspecsRateParameters[acID].highRateThreshold = highRateThreshold;
			pRateAdaptation->tspecsRateParameters[acID].lowRateThreshold = lowRateThreshold;
		}
}


/*************************************************************************
 *																		 *
 *							DEBUG FUNCTIONS								 *
 *																		 *
 *************************************************************************/
void rateAdaptation_print(rateAdaptation_t *pRateAdaptation)
{
	UINT32 count;

	WLAN_OS_REPORT(("     Rate Adaptation Parameters    \n"));
	WLAN_OS_REPORT(("-----------------------------------\n"));
	WLAN_OS_REPORT(("txCount                   = %d\n",pRateAdaptation->txCount));
	WLAN_OS_REPORT(("txFailCount               = %d\n",pRateAdaptation->txFailCount));
    WLAN_OS_REPORT(("txRateFallBackCount       = %d\n",pRateAdaptation->txRateFallBackCount));
	WLAN_OS_REPORT(("txSkipCount               = %d\n",pRateAdaptation->txSkipCount));
	WLAN_OS_REPORT(("currRateIndex             = %d\n",pRateAdaptation->currRateIndex));
	WLAN_OS_REPORT(("currRate Number           = %d\n",pRateAdaptation->RatesMap[pRateAdaptation->currRateIndex].rateNumber));
	WLAN_OS_REPORT(("maxRateIndex              = %d\n",pRateAdaptation->maxRateIndex));
	WLAN_OS_REPORT(("maxRate Number            = %d\n",pRateAdaptation->RatesMap[pRateAdaptation->maxRateIndex].rateNumber));
	WLAN_OS_REPORT(("stepUpFlag                = %d\n",pRateAdaptation->stepUpFlag));
	WLAN_OS_REPORT(("expirTimeTick             = %d\n",pRateAdaptation->expirTimeTick));
	WLAN_OS_REPORT(("stepUpTxPacketsThreshold  = %d\n",pRateAdaptation->stepUpTxPacketsThreshold));
	WLAN_OS_REPORT(("contTxPacketsThreshold    = %d\n",pRateAdaptation->contTxPacketsThreshold)); 
	WLAN_OS_REPORT(("ctrlDataFBShortInterval   = %d\n",pRateAdaptation->ctrlDataFBShortInterval));
	WLAN_OS_REPORT(("ctrlDataFBLongInterval    = %d\n",pRateAdaptation->ctrlDataFBLongInterval));

	WLAN_OS_REPORT(("    Rate Adaptation Table    \n"));
	WLAN_OS_REPORT(("-----------------------------\n"));
	for(count = 0 ; count <  MAX_SUPPORTED_RATES ; count++)
	{
		WLAN_OS_REPORT(("Index = %d ,RateNumber = %d,StepUp = %d ,FallBack = %d , Modulation = %d 4X = %d valid = %d,\n",
									count, 
										  pRateAdaptation->RatesMap[count].rateNumber,
										  pRateAdaptation->RatesMap[count].rateAdaptStepUp,
										  pRateAdaptation->RatesMap[count].rateAdaptFallBack,
										  pRateAdaptation->RatesMap[count].modulation,
										  pRateAdaptation->RatesMap[count].fourXEnable,
										  pRateAdaptation->RatesMap[count].valid));
	}

	WLAN_OS_REPORT(("Tspecs Rate Parameters AC = %d  \n",count));
	WLAN_OS_REPORT(("-----------------------------\n"));
	for(count = 0 ; count < MAX_NUM_OF_AC ; count++)
	{
		WLAN_OS_REPORT(("AC = %d  \n",count));
		WLAN_OS_REPORT(("---------\n"));
		WLAN_OS_REPORT(("enableEvent = %d  \n",pRateAdaptation->tspecsRateParameters[count].enableEvent));
		WLAN_OS_REPORT(("highRateThreshold = %d  \n",pRateAdaptation->tspecsRateParameters[count].highRateThreshold));
		WLAN_OS_REPORT(("lowRateThreshold = %d  \n",pRateAdaptation->tspecsRateParameters[count].lowRateThreshold));
	}


}

