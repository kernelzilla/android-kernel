/** \file SoftGemini.h
 *  \brief BlueTooth-Wlan coexistence module internal header file
 *
 *  \see SoftGemini.c
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
/*																			*/
/*	  MODULE:	SoftGemini.h												*/
/*    PURPOSE:	BlueTooth-Wlan coexistence module internal header file			*/
/*																			*/
/***************************************************************************/

#ifndef __SOFT_GEMINI_H__
#define __SOFT_GEMINI_H__

#include "paramOut.h"
#include "SoftGeminiApi.h"
#include "softGeminiTypes.h"


typedef struct 
{
	SoftGeminiEnableModes_e SoftGeminiEnable;						
    SoftGeminiEnableModes_e PsPollFailureLastEnableValue;
	UINT8					SoftGeminiRate[NUM_OF_RATES_IN_SG];		/* whether to use the specific rate arranged as
																	    5.5,11,12,18,24,36,48,54 */
	SoftGeminiParam_t		SoftGeminiParam;						/* for the FW */
	UINT8					scanNumOfProbeRequest;					/* Probes to send on any scan when SG enabled */
	UINT32					scanCompensationPercent;				/* Percentage of increasing dwell time on each channel */
	UINT32					scanCompensationMaxTime;				/* Max value of increasing dwell time */
	UINT32					BSSLossCompensationPercent;				/* Percentage of increasing value of Beacon loss parameter */
	BOOL					bProtectiveMode;
	BOOL					bDriverEnabled;							/* used to check if we should enable driver when we are switching different enable modes */
    BOOL					bPsPollFailureActive;
	btCoexStatus_t			btCoexStatus;
    TI_HANDLE				hCtrlData;
	TI_HANDLE				hHalCtrl;
	TI_HANDLE				hReport;
	TI_HANDLE				hOs;
	TI_HANDLE				hSCR;
	TI_HANDLE				hPowerMgr;
	TI_HANDLE				hConfigMgr;
	TI_HANDLE				hScanCncn;
	TI_HANDLE				hCurrBss;
	TI_HANDLE				hEvHandler;
} SoftGemini_t;

#endif /* __SOFT_GEMINI_H__*/


