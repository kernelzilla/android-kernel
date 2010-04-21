/** \file currBss.h
 *  \brief Current BSS module API
 *
 *  \see currBss.c
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
 *   MODULE:  Current BSS                                               *
 *   PURPOSE: Current BSS Module API                                    *
 *                                                                          *
 ****************************************************************************/

#ifndef _CURR_BSS_H_
#define _CURR_BSS_H_

#include "whalParams.h"
#include "siteMgrApi.h"
#include "roamingMngrTypes.h"

/* Constants */

/* Enumerations */

/** 
* Current BSS module configurable parameters type  
*/
typedef enum
{
    CURR_BSS_TYPE = 0,          /**< BSS or IBSS */
    CURR_BSS_CONNECTED_STATE,   /**< Connected or not connected, roaming enabled or not */
    CURR_BSS_LOW_RSSI_SCAN_COND,/**< Set by AP Connection when Roaming Manager configures low RSSI threshold for BG scan */
    CURR_BSS_HI_RSSI_SCAN_COND, /**< Set by AP Connection when Roaming Manager configures normal RSSI threshold for BG scan */
    CURR_BSS_QUALITY_THRESHOLD, /**< Set by AP Connection when Roaming Manager configures event of type 'Low RSSI' */
    CURR_BSS_NO_BSS_THRESHOLDS, /**< Set by AP Connection when Roaming Manager configures event of type 'BSS Loss' */
    CURR_BSS_NUM_OF_TEST_REPEAT,/**< Set by AP Connection when Roaming Manager configures event of type 'BSS Loss' */
    CURR_BSS_CONSEC_NACK,       /**< Set by AP Connection when Roaming Manager configures event of type 'Consecutive nack' */
    CURR_BSS_INFO               /**< Requested by AP Connection: includes quality and last beacon info */
} currBSS_configParam_e;


/* Structures */

/* Typedefs */

/* External data definitions */

/* External functions definitions */

/* Function prototypes */

TI_HANDLE currBSS_create(TI_HANDLE hOs);
TI_STATUS currBSS_init(TI_HANDLE hCurrBSS,
                       TI_HANDLE hMlme,
                       TI_HANDLE hPowerMgr,
                       TI_HANDLE hAPConnection,
                       TI_HANDLE hSME,
                       TI_HANDLE hHal,
                       TI_HANDLE hReport,
                       TI_HANDLE hMemMgr,
                       TI_HANDLE hTxData,
                       TI_HANDLE hSiteMngr,
                       TI_HANDLE hScanMngr,
                       TI_HANDLE hMacServices);
TI_STATUS currBSS_unload(TI_HANDLE hCurrBSS);

void currBSS_updateConnectedState(TI_HANDLE hCurrBSS, BOOLEAN isConnected, bssType_e type);
TI_STATUS currBSS_updateRoamingTriggers(TI_HANDLE hCurrBSS,
                                        roamingMngrThresholdsConfig_t *params);
void currBSS_SGconfigureBSSLoss(TI_HANDLE hCurrBSS,
                                        UINT32 SGcompensationPercent , BOOLEAN bUseSGParams);
bssEntry_t *currBSS_getBssInfo(TI_HANDLE hCurrBSS);

TI_STATUS currBSS_getRoamingParams(TI_HANDLE hCurrBSS,
                                   UINT8 * aNumExpectedTbttForBSSLoss,
                                   INT8 * aLowQualityForBackgroungScanCondition,
                                   INT8 * aNormalQualityForBackgroungScanCondition,
                                   UINT8 * rssiFilterWeight,
                                   UINT8 * snrFilterWeight);

TI_STATUS currBSS_probRespReceivedCallb(TI_HANDLE hCurrBSS,
                                        Rx_attr_t *pRxAttr,
                                        macAddress_t *bssid,
                                        mlmeFrameInfo_t *pFrameInfo,
                                        char *dataBuffer,
                                        UINT16 bufLength);

TI_STATUS currBSS_beaconReceivedCallb(TI_HANDLE hCurrBSS,
                                        Rx_attr_t *pRxAttr,
                                        macAddress_t *bssid,
                                        mlmeFrameInfo_t *pFrameInfo,
                                        char *dataBuffer,
                                        UINT16 bufLength);

TI_STATUS currBSS_performRecovery(TI_HANDLE hCurrBSS);

void currBSS_restartRssiCounting(TI_HANDLE hCurrBSS);
TI_STATUS currBSS_Bss_Regain_CallB(TI_HANDLE hCurrBSS);

void currBSS_updateRxSignal(TI_HANDLE hCurrBSS, UINT8 uSNR, INT8 iRSSI, BOOL bAveragedData);

TI_STATUS currBSS_Bss_Reset_CallB(TI_HANDLE hCurrBSS);


#endif /*  _CURR_BSS_H_*/

