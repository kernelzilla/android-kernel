/** \file apConn.h
 *  \brief AP Connection Module API
 *
 *  \see apConn.c
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
 *   MODULE:  AP Connection	    		                                *
 *   PURPOSE: AP Connection Module API                              		*
 *                                                                          *
 ****************************************************************************/

#ifndef _AP_CONNECTION_H_
#define _AP_CONNECTION_H_

#include "802_11Defs.h"
#include "apConnApi.h"

/* Typedefs */
/* This struct is used for ROAMING_TRIGGER_AP_DISCONNECT */
typedef struct  
{
	UINT16 uStatusCode;		/* status code of deauth/disassoc packet				   */
	BOOL   bDeAuthenticate;	/* Whether this packet is DeAuth ( if DisAssoc than FALSE) */
} APDisconnect_t;

typedef union
{
	APDisconnect_t APDisconnect;
	rate_e		   rate;
} roamingEventData_u;
/* Structures */

/* External data definitions */

/* External functions definitions */

/* Function prototypes */

/* Called by Config Manager */
TI_HANDLE apConn_create(TI_HANDLE hOs);
TI_STATUS apConn_unload(TI_HANDLE hAPConnection);
TI_STATUS apConn_config(TI_HANDLE hAPConnection, 
						TI_HANDLE hReport,
						TI_HANDLE hCurrAP,
                        TI_HANDLE hRoamMng,
                        TI_HANDLE hSme,
						TI_HANDLE hSiteMgr,
						TI_HANDLE hExcMngr,
						TI_HANDLE hConnSm,
						TI_HANDLE hPrivacy,
						TI_HANDLE hQos,
						TI_HANDLE hCtrl,
						TI_HANDLE hEvHandler,
                        TI_HANDLE hScr,
						TI_HANDLE hAssoc,
						TI_HANDLE hRegulatoryDomain,
                        apConnParams_t *pApConnParams);

/* Called by SME and Site Manager */
TI_STATUS apConn_start(TI_HANDLE hAPConnection, BOOLEAN roamingEnabled);
TI_STATUS apConn_stop(TI_HANDLE hAPConnection, BOOLEAN removeKeys, BOOLEAN radioOn);

void apConn_printStatistics(TI_HANDLE hAPConnection);


/* Called by Connection SM */
TI_STATUS apConn_ConnCompleteInd(TI_HANDLE hAPConnection, mgmtStatus_e status, UINT32 uStatusCode);

TI_STATUS apConn_DisconnCompleteInd(TI_HANDLE hAPConnection, mgmtStatus_e status, UINT32 uStatusCode);

/* Called by Current BSS, Rate Adaptation, RSN and other modules generating roaming events */
TI_STATUS apConn_reportRoamingEvent(TI_HANDLE hAPConnection,
									apConn_roamingTrigger_e roamingEventType,
									roamingEventData_u *pRoamingEventData);

/* Called by EXC Manager */
void apConn_RoamHandoffFinished(TI_HANDLE hAPConnection);
void apConn_getRoamingStatistics(TI_HANDLE hAPConnection, UINT8 *roamingCount, UINT16 *roamingDelay);
void apConn_resetRoamingStatistics(TI_HANDLE hAPConnection);

void apConn_updateNeighborAPsList(TI_HANDLE hAPConnection, neighborAPList_t *pListOfpriorityAps);

/* Called by Switch Channel */
TI_STATUS apConn_indicateSwitchChannelInProgress(TI_HANDLE hAPConnection);
TI_STATUS apConn_indicateSwitchChannelFinished(TI_HANDLE hAPConnection);

/* Called by Association SM */
TI_STATUS apConn_getVendorSpecificIE(TI_HANDLE hAPConnection, UINT8 *pRequest, UINT32 *len);

BOOLEAN apConn_isPsRequiredBeforeScan(TI_HANDLE hAPConnection);

#endif /*  _AP_CONNECTION_H_*/

