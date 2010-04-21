/** \file utilsReplvl.h
 *  \brief Report level API
 *
 *  \see utilsReplvl.c
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
/*																									*/
/*	  MODULE:	utilsReplvl.h																*/
/*    PURPOSE:	Report level API			 								*/
/*																									*/
/***************************************************************************/
#ifndef __RSN_API_H__
#define __RSN_API_H__

#include "osTIType.h"
#include "paramOut.h"
#include "paramIn.h"
#include "memMngrEx.h"
#include "802_11Defs.h"

/* Constants */

#define RSN_MAC_ADDR_LEN			6
#define MAX_KEY_RSC_LEN				8
#define MAX_SSN_KEY_DATA_LENGTH		32
#define RSN_AUTH_FAILURE_TIMEOUT    30000

/* Enumerations */

/** RSN key management suites */
typedef enum 
{
	RSN_KEY_MNG_NONE				= 0,		/**< no key management available */
	RSN_KEY_MNG_802_1X				= 1,		/**< "802.1X" key management */
	RSN_KEY_MNG_WPA			        = 2,		/**< "WPA 4 way handshake" key management */
	RSN_KEY_MNG_EXC			        = 3,		/**< "EXC" key management */
	RSN_KEY_MNG_UNKNOWN				= 255		/**< UNKNOWN key management available */
} rsn_keyMngSuite_e;

/** Available cipher suites for admission control */
typedef enum 
{
	RSN_IBSS            	= 0, 		/**< IBSS mode */
	RSN_INFRASTRUCTURE		= 1 		/**< Infrastructure mode */
} rsn_networkMode_t;

/** Port Access Entity role type */
typedef enum
{
	RSN_PAE_AP		= 0,
	RSN_PAE_SUPP	= 1
} rsn_paeRole_t;

/** RSN Events */
typedef enum {
	RSN_EVENT_EAPOL_RECV    		= 0x0,		/**< EAPOL frame received in the RX */
	RSN_EVENT_SEC_ATTACK_DETECT		= 0x1,		/**< Security Attack detection */
	RSN_EVENT_RAW_KEY_RECV			= 0x2,		/**< Raw key recive */
    RSN_EVENT_KEY_REMOVE            = 0x3		/**< Key remove event */
} rsn_event_e;  

/** Site ben levels */
typedef enum 
{
    RSN_SITE_BAN_LEVEL_HALF = 1,
    RSN_SITE_BAN_LEVEL_FULL  = 2
} rsn_siteBanLevel_e;


/* Typedefs */

/** Port Access Entity structure */
typedef struct
{
	externalAuthMode_e	authProtocol;
	rsn_keyMngSuite_e	keyExchangeProtocol;
	cipherSuite_e		unicastSuite;
	cipherSuite_e		broadcastSuite;
} rsn_paeConfig_t;

typedef struct
{
	BOOL 		    privacy;
	UINT8		    *pIe;
	UINT8		    ieLen;
} rsnData_t;

typedef struct 
{
   macAddress_t bssId;
   dot11_RSN_t  *pRsnIEs;
   UINT8        rsnIeLen;
} bssidRsnInfo_t;

#define MAX_NUM_OF_PRE_AUTH_BSSIDS 	16
typedef struct 
{
   UINT8          NumOfItems;
   bssidRsnInfo_t bssidList[MAX_NUM_OF_PRE_AUTH_BSSIDS];
} bssidList4PreAuth_t;


/* Prototypes */

TI_HANDLE rsn_create(TI_HANDLE hOs);

TI_STATUS rsn_unload(TI_HANDLE hRsn);

TI_STATUS rsn_config(TI_HANDLE 		hRsn,
				  TI_HANDLE			hTx,
				  TI_HANDLE			hRx,
				  TI_HANDLE			hConn,
				  TI_HANDLE			hMlme,
				  TI_HANDLE			hCtrlData,
				  TI_HANDLE			hWhalCtrl,
				  TI_HANDLE			hMemMgr,
                  TI_HANDLE         hSiteMgr,
				  TI_HANDLE 		hReport,
				  TI_HANDLE 		hOs,
				  TI_HANDLE 		hExcMngr,
				  TI_HANDLE 		hPowerMgr,
                  TI_HANDLE         hEvHandler,
                  TI_HANDLE         hSmeSm,
                  TI_HANDLE         hAPConn,
				  rsnInitParams_t	*pInitParam);

TI_STATUS rsn_reconfig(TI_HANDLE hRsn);

TI_STATUS rsn_start(TI_HANDLE hRsn);

TI_STATUS rsn_stop(TI_HANDLE hRsn, BOOL removeKeys);

TI_STATUS rsn_eventRecv(TI_HANDLE hRsn, rsn_event_e event, void* pData);

TI_STATUS rsn_setParam(TI_HANDLE hCtrlData, paramInfo_t	*pParam);

TI_STATUS rsn_getParam(TI_HANDLE hCtrlData, paramInfo_t	*pParam);

TI_STATUS rsn_evalSite(TI_HANDLE hRsn, rsnData_t *pRsnData, bssType_e bssType, macAddress_t bssid, UINT32 *pMetric);

TI_STATUS rsn_setSite(TI_HANDLE hRsn, rsnData_t *pRsnData, UINT8 *pAssocIe, UINT8 *pAssocIeLen);

TI_STATUS rsn_recvEapol(TI_HANDLE hRsn, mem_MSDU_T *pMsdu);

TI_STATUS rsn_getInfoElement(TI_HANDLE hRsn, UINT8 *pRsnIe, UINT8 *pRsnIeLen);

#ifdef EXC_MODULE_INCLUDED
TI_STATUS rsn_getExcExtendedInfoElement(TI_HANDLE hRsn, UINT8 *pRsnIe, UINT8 *pRsnIeLen);
#endif

TI_STATUS rsn_reportAuthFailure(TI_HANDLE hRsn, authStatus_e authStatus);

TI_STATUS rsn_reportMicFailure(TI_HANDLE hRsn, UINT8 *pType, UINT32 Length);

TI_STATUS rsn_resetPMKIDList(TI_HANDLE hRsn);

TI_STATUS rsn_removedDefKeys(TI_HANDLE hRsn);

TI_STATUS rsn_startPreAuth(TI_HANDLE hRsn, bssidList4PreAuth_t *pBssidList);

rsn_siteBanLevel_e rsn_banSite(TI_HANDLE hRsn, macAddress_t siteBssid, rsn_siteBanLevel_e banLevel, UINT32 durationMs);

BOOL rsn_isSiteBanned(TI_HANDLE hRsn, macAddress_t siteBssid);

void rsn_MboxFlushFinishCb(TI_HANDLE handle, UINT16 MboxStatus, char *InterrogateParamsBuf);


#endif /* __RSN_API_H__*/
