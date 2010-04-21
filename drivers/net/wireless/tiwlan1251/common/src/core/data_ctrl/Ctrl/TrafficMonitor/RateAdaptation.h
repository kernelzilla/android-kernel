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
/*																		   */
/*	  MODULE:	tx.h												       */
/*    PURPOSE:	Tx module Header file		 							   */
/*																		   */
/***************************************************************************/
#ifndef _RATE_ADAPTATION_H_
#define _RATE_ADAPTATION_H_

#include "osTIType.h"
#include "paramIn.h"
#include "paramOut.h"

#define 	DEF_LOW_RATE_THRESHOLD 				DRV_RATE_AUTO  /* TBD Below this rate generate roaming event */

#define RATE_ADAPTATION_MODULE_LOG				CTRL_DATA_MODULE_LOG


#define TS_EXCEEDS(currTime,expTime) (currTime > expTime)
#define TS_ADVANCE(currTime,expTime,delta) (expTime = currTime + (delta))


/*#define NUM_OF_RATES_ENTRIES			5*/

typedef struct
{
	rate_e				rate;
	UINT8				rateNumber; /* Rate as actual number - used to compare 2 rates */
	modulationType_e	modulation;
	BOOL				fourXEnable;
	UINT8				rateAdaptFallBack;
	UINT8				rateAdaptStepUp; 
	BOOL				valid;

}rateModulation4x_table_t;

typedef struct
{
	TI_HANDLE			hOs;
	TI_HANDLE			hReport;
	TI_HANDLE			hCtrlData;
    TI_HANDLE           hEvHandler;


	/* Rate Adaptation Algorithm Parameters */
	UINT32				expirTimeTick;
	UINT32				txCount;
	UINT32				txFailCount;
    UINT32              txRateFallBackCount;
    	UINT32				txSkipCount;
	UINT8				currRateIndex;
	UINT8				maxRateIndex;
	UINT8				stepUpFlag;
	UINT8				stepUpTxPacketsThreshold;		/* RATE_NUM_SETPUP_PKTS	10  The amount of packets to commite the alghorithem after step up.*/ 
	UINT8				contTxPacketsThreshold;      /*#define WDRV_TX_RATE_NUM_CONT_PKTS		30  The amount of packets to commite the alghorithem in contiuse.  */ 
	UINT8				lowRateThreshold;
	UINT32				ctrlDataFBShortInterval;
	UINT32				ctrlDataFBLongInterval;

	TI_HANDLE			pTimer;
	TI_HANDLE			hAPConnection;
	UINT32				rateAdapt_timeout;

	
	rateModulation4x_table_t	RatesMap[MAX_SUPPORTED_RATES];

	tspecsRateParameters_t		tspecsRateParameters[MAX_NUM_OF_AC];

} rateAdaptation_t;

rateAdaptation_t* rateAdaptation_create(TI_HANDLE hOs);

TI_STATUS rateAdaptation_config(rateAdaptation_t*			pRateAdaptation, 
	   							TI_HANDLE					hOs, 
								TI_HANDLE					hReport, 
								TI_HANDLE					hCtrlData, 
                                TI_HANDLE					hEvHandler,
								TI_HANDLE					hAPConnection,
								rateAdaptationInitParam_t*	rateAdaptationInitParam);

TI_STATUS rateAdaptation_destroy(rateAdaptation_t* pRateAdaptation);

TI_STATUS rateAdaptation_start(rateAdaptation_t* pRateAdaptation);

TI_STATUS rateAdaptation_stop(rateAdaptation_t* pRateAdaptation);

TI_STATUS rateAdaptation_stopTimer(rateAdaptation_t* pRateAdaptation);

TI_STATUS rateAdaptation_updateRateAdaptation(rateAdaptation_t* pRateAdaptation,
											  rate_e			actualTxRate,
											  rate_e			requestTxRate, 
											  UINT32			TxStatus,
                                              int               txNumWaiting);

TI_STATUS rateAdaptation_configLowRateThrsh(rateAdaptation_t* pRateAdaptation, UINT8 rate);

/* Get functions */
rateModulation4x_table_t* rateAdaptation_getCurrent(rateAdaptation_t* pRateAdaptation);	
rate_e rateAdaptation_getCurrentRate(rateAdaptation_t* pRateAdaptation);	
modulationType_e rateAdaptation_getCurrentModulation(rateAdaptation_t* pRateAdaptation);	
BOOL rateAdaptation_getCurrentFourXEnable(rateAdaptation_t* pRateAdaptation);	

/* Set functions */
TI_STATUS rateAdaptation_setCurrentRate(rateAdaptation_t* pRateAdaptation, rate_e rate);
TI_STATUS rateAdaptation_setMaxActivRate(rateAdaptation_t* pRateAdaptation, rate_e rate);	
void rateAdaptation_updateModulation(rateAdaptation_t* pRateAdaptation,
									 modulationType_e modulation,
									 bssType_e bssType);

void rateAdaptation_update4xEnable(rateAdaptation_t* pRateAdaptation,
								   BOOL				 enable4x,
								   bssType_e		 bssType);

TI_STATUS rateAdaptation_buildRateMapTable(rateAdaptation_t		*pRateAdaptation,
										   ctrlData_rateAdapt_t *currTable,
										   UINT32				supportedBitMap,
										   UINT32				clientBitMap,
										   modulationType_e		modulation,
										   BOOL					enable4x,
										   bssType_e			bssType);

/* utils functions */
TI_STATUS rateAdaptation_Utils_IsRateInBitmap(rateAdaptation_t	*pRateAdaptation,
								UINT32			ratesBitMap,
								rate_e			rate);

UINT32 rateAdaptation_Utils_buildRateBitMap(rateAdaptation_t	*pRateAdaptation,
											ctrlData_rateAdapt_t *currTable,
											rate_e			rate,
											UINT32			supportedBitMap,
											UINT32			clientBitMap);

BOOL rateAdaptation_isRateInTable(ctrlData_rateAdapt_t *currTable,
								rate_e			rate);

void rateAdaptation_setTspecsRateEvent(rateAdaptation_t* pRateAdaptation,
											 UINT8			acID,
											BOOL			enableEvent);

void rateAdaptation_setTspecsRateThresholds(rateAdaptation_t* pRateAdaptation,
											 UINT8			acID,
											 UINT8			highRateThreshold,
											 UINT8			lowRateThreshold);

void rateAdaptation_print(rateAdaptation_t* pRateAdaptation);

#endif
