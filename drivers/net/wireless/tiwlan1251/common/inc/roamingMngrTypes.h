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

/** \file roamingMngrTypes.h
 *  \brief Internal Roaming Manager Types API
 *
 *  \see roamingMngr.c
 */

/****************************************************************************
 *                                                                          *
 *   MODULE:  Roaming Manager                                               *
 *   PURPOSE: Roaming Manager Module Types                                  *    *                                                                          *
 ****************************************************************************/

#ifndef _ROAMING_MNGR_TYPES_H_
#define _ROAMING_MNGR_TYPES_H_


/* Constants */

/* Enumerations */

/* Typedefs */
/* configuration of Roaming manager */
#define ROAMING_ENABLED 		1
#define ROAMING_DISABLED		0

#pragma pack(1)
typedef struct
{
	UINT16	enableDisable; 					/* ROAMING_ENABLED, ROAMING_DISABLED - BOOL is not used, 
												beacuse of misdefinition between the Driver and CU */
    UINT16	lowPassFilterRoamingAttempt;	/* Time to wait in sec, before roaming due to the low connection quality */
	INT8	apQualityThreshold;				/* Quality indicator (RSSI) to be used when comparing AP List matching quality */

} roamingMngrConfig_t;
#pragma pack()

#pragma pack(1)
typedef struct
{
    /* Roaming trigger thresholds */ 
	/* Note - It's the applicatio's responsibility to chage the threshold when Voice is On */
	UINT8	dataRetryThreshold;				/* Consecutive number of TX retries */  
    UINT8	numExpectedTbttForBSSLoss;		/* number of expected TBTTs for BSS Loss event */
	UINT8	txRateThreshold;				/* TX rate (fallback) threshold */
	INT8	lowRssiThreshold;				/* low RSSI threshold */
	UINT8	lowSnrThreshold;				/* low SNR threshold */
	INT8	lowQualityForBackgroungScanCondition; /* Indicator used to increase the background scan period when quality is low. */
	INT8	normalQualityForBackgroungScanCondition; /* Indicator used to reduce the background scan period when quality is normal. */
	UINT8	rssiFilterWeight;				/* last RSSI weight in the AVG calculation */
	UINT8	snrFilterWeight;				/* last SNR weight in the AVG calculation */
} roamingMngrThresholdsConfig_t;
#pragma pack()



#pragma pack(1)
typedef struct
{
	roamingMngrConfig_t				roamingMngrConfig;
	roamingMngrThresholdsConfig_t	roamingMngrThresholdsConfig;
} roamingMngrConfigParams_t;
#pragma pack()



#endif /*  _ROAMING_MNGR_TYPES_H_*/

