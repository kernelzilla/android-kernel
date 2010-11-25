/** \file MainSecSm.h
 *  \brief RSN main security SM
 *
 *  \see MainSecSm.c
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
/*																		   */
/*		MODULE:	MainSecSm.h												   */
/*    PURPOSE:	RSN main security SM									   */
/*																	 	   */
/***************************************************************************/

#ifndef _MAIN_SEC_SM_H
#define _MAIN_SEC_SM_H

#include "paramOut.h"
#include "fsm.h"
#include "rsnApi.h"
#include "rsn.h"
#include "keyTypes.h"

/* Constants */

/* Enumerations */

/* Typedefs */

typedef struct _mainSec_t    mainSec_t;

/* Main Sec SM functions */
typedef TI_STATUS (*mainSecSmStart_t)(struct _mainSec_t *pMainSec);
typedef TI_STATUS (*mainSecSmStop_t)(struct _mainSec_t *pMainSec);
typedef TI_STATUS (*mainSecSmReportAuthStatus_t)(struct _mainSec_t *pMainSec, TI_STATUS authStatus);
typedef TI_STATUS (*mainSecSmReportKeysStatus_t)(struct _mainSec_t *pMainSec, TI_STATUS keyStatus);
typedef TI_STATUS (*mainSecSmReportReauthentication_t)(struct _mainSec_t *pMainSec);
typedef TI_STATUS (*mainSecSmSetSessionKey_t)(struct _mainSec_t *pMainSec, UINT8* pKey, UINT8 keyLen);
typedef TI_STATUS (*mainSecSmGetSessionKey_t)(struct _mainSec_t *pMainSec, UINT8* pKey, UINT32* pKeyLen);
typedef TI_STATUS (*mainSecSmSetKey_t)(struct _mainSec_t *pMainSec, securityKeys_t *pKey);
typedef TI_STATUS (*mainSecSmRemoveKey_t)(struct _mainSec_t *pMainSec, securityKeys_t *pKey);
typedef TI_STATUS (*mainSecSmSetDefaultKeyId_t)(struct _mainSec_t *pMainSec, UINT8 keyId);
typedef TI_STATUS (*mainSecSmEapolRecv_t)(struct _mainSec_t *pMainSec, UINT8 *pPacket);
typedef TI_STATUS (*mainSecSm_getAuthIdentity_t)(struct _mainSec_t *pMainSec, authIdentity_t *authIdentity);
typedef TI_STATUS (*mainSecSm_getAuthState_t)(struct _mainSec_t *pMainSec, TIWLN_SECURITY_STATE *secState);
typedef TI_STATUS (*mainSecSm_reportAuthFailure_t)(struct _mainSec_t *pMainSec, authStatus_e authStatus);

/* Structures */

typedef struct
{
	rsn_paeConfig_t		*pPaeConfig;
	UINT32				heldPeriod;
	UINT32				startPeriod;
	UINT32				authPeriod;
	UINT32				maxStart;
	UINT8				*pUserId;
	UINT8				idLen;
	UINT8				*pUserPasswd;
	UINT8				pwdLen;
} mainSecInitData_t;

typedef struct
{
	TI_STATUS		status;
} mainSecData_t;

struct _mainSec_t
{
	UINT8                               currentState;
    UINT32				                startCount;
	fsm_stateMachine_t	                *pMainSecSm;
	BOOL				                PrivacyOptionImplemented;
    mainSecData_t						data;
	UINT8								sessionKey[SESSION_KEY_LEN];
	UINT8								sessionKeyLen;

    struct _rsn_t                      	*pParent;
	TI_HANDLE			                hReport;
	TI_HANDLE			                hOs;
	
    struct _mainKeys_t                  *pMainKeys; 
    struct _keyParser_t                 *pKeyParser;

    mainSecSmStart_t                    start;
    mainSecSmStop_t                     stop;
    mainSecSmReportAuthStatus_t         reportAuthStatus;
    mainSecSmReportKeysStatus_t       	reportKeysStatus;
    mainSecSmReportReauthentication_t   reportReAuthenticate;
	mainSecSmSetSessionKey_t			setSessionKey;
	mainSecSmGetSessionKey_t			getSessionKey;
	mainSecSmSetKey_t					setKey;
	mainSecSmRemoveKey_t				removeKey;
	mainSecSmSetDefaultKeyId_t			setDefaultKeyId;
    mainSecSm_getAuthIdentity_t  		getAuthIdentity;
    mainSecSm_getAuthIdentity_t  		setAuthIdentity;
	mainSecSm_getAuthState_t 			getAuthState;
    mainSecSm_reportAuthFailure_t       reportAuthFailure;

};

/* External data definitions */

/* External functions definitions */

/* Function prototypes */

mainSec_t* mainSec_create(TI_HANDLE hOs);

TI_STATUS mainSec_unload(mainSec_t *pMainSec);

TI_STATUS mainSec_config(mainSec_t *pMainSec,
                      mainSecInitData_t *pInitData, 
					  void *pParent,
					  TI_HANDLE hReport,
					  TI_HANDLE hOs,
                      TI_HANDLE hCtrlData,
                      TI_HANDLE hEvHandler,
                      TI_HANDLE hConn,
                      TI_HANDLE hHalCtrl);

#endif

