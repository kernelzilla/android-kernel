/** \file assocSM.h
 *  \brief 802.11 Association SM
 *
 *  \see assocSM.c
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
/*                                                                         */
/*      MODULE: assocSM.h                                                  */
/*    PURPOSE:  802.11 Association SM                                      */
/*                                                                         */
/***************************************************************************/

#ifndef _RSN_H
#define _RSN_H

#include "rsnApi.h"
#include "memMngrEx.h"
#include "paramOut.h"
#include "siteHash.h"

/* Constants */

#define RSN_MAX_NUMBER_OF_EVENTS        6
#define RSN_MAX_NUMBER_OF_BANNED_SITES  16
#define RSN_MIC_FAILURE_REPORT_WAIT     500
#define RSN_MIC_FAILURE_TIMEOUT         (60000 + RSN_MIC_FAILURE_REPORT_WAIT)
#define RSN_MAIN_KEYS_SESSION_TIMEOUT   RSN_AUTH_FAILURE_TIMEOUT 
#define RSN_MIC_FAILURE_RE_KEY          3000

/* Enumerations */
typedef enum 
{
    MIC_FAILURE_FALSE,
    MIC_FAILURE_TRUE,
    MIC_FAILURE_BLOCK
} rsn_micFailureStatus_e;

typedef enum 
{
    GROUP_KEY_UPDATE_FALSE,
    GROUP_KEY_UPDATE_TRUE
} rsn_groupKeyUpdate_e;

/* Typedefs */
typedef struct _rsn_t   rsn_t;
         
typedef TI_STATUS (*rsn_eventCallback_t)(void* pCtx, void *pData);
typedef TI_STATUS (*rsn_setPaeConfig_t)(rsn_t *pRsn, rsn_paeConfig_t *pPaeConfig);
typedef TI_STATUS (*rsn_getNetworkMode_t)(rsn_t *pRsn, rsn_networkMode_t *pNetMode);

typedef TI_STATUS (*rsn_setKey_t)(rsn_t *pMainSec, securityKeys_t *pKey);
typedef TI_STATUS (*rsn_removeKey_t)(rsn_t *pMainSec, securityKeys_t *pKey);
typedef TI_STATUS (*rsn_setDefaultKeyId_t)(rsn_t *pMainSec, UINT8 keyId);
typedef TI_STATUS (*rsn_reportStatus_t)(rsn_t *pRsn, TI_STATUS rsnStatus);
typedef TI_STATUS (*rsn_sendEapol_t)(rsn_t *pRsn, UINT8 *pPacket, UINT32 length);
typedef TI_STATUS (*rsn_recvEapol_t)(rsn_t *pRsn, mem_MSDU_T *pMsdu);
typedef TI_STATUS (*rsn_getSiteEntry_t)(rsn_t *pRsn, macAddress_t *macAddress, siteEntry_t *curSiteEntry);

typedef struct
{
    rsn_eventCallback_t     eventFunc;
    void                    *pCtx;
} rsn_eventStruct_t;

typedef struct
{
    char                id[MAX_IDENTITY_LEN];           /**< User identity string */
    UINT8               idLength;                       /**< User identity string length */
    char                password[MAX_PASSWD_LEN];       /**< User password string */
    UINT8               pwdLength;                      /**< User password string length */
} authIdentity_t;

typedef struct 
{
    rsn_siteBanLevel_e      banLevel;
    UINT32                  banStartedMs;
    UINT32                  banDurationMs;
    macAddress_t            siteBssid;
} rsn_siteBanEntry_t;

struct _rsn_t
{
    rsn_eventStruct_t       events[RSN_MAX_NUMBER_OF_EVENTS];
    rsn_paeConfig_t         paeConfig;
    BOOL                    PrivacyOptionImplemented;
    
    securityKeys_t          keys[MAX_KEYS_NUM];
    BOOL                    keys_en [MAX_KEYS_NUM];
    UINT8                   defaultKeyId;
    BOOL                    defaultKeysOn;
    BOOL                    wepDefaultKeys[MAX_KEYS_NUM];
    BOOL                    wepStaticKey;
    rsn_groupKeyUpdate_e    groupKeyUpdate;
    OS_802_11_EAP_TYPES     eapType;

    rsn_siteBanEntry_t      bannedSites[RSN_MAX_NUMBER_OF_BANNED_SITES];
    UINT8                   numOfBannedSites;

    TI_HANDLE               micFailureReportWaitTimer;
    TI_HANDLE               micFailureReKeyTimer;

    struct _admCtrl_t       *pAdmCtrl;
    struct _mainSec_t       *pMainSecSm;

    struct _keyParser_t     *pKeyParser;

    TI_HANDLE               hTx;
    TI_HANDLE               hRx;
    TI_HANDLE               hConn;
    TI_HANDLE               hCtrlData;
    TI_HANDLE               hWhalCtrl;
    TI_HANDLE               hMemMgr;
    TI_HANDLE               hSiteMgr;
    TI_HANDLE               hReport;
    TI_HANDLE               hOs;
    TI_HANDLE               hExcMngr;
    TI_HANDLE               hEvHandler;
    TI_HANDLE               hSmeSm;
    TI_HANDLE               hAPConn;
    
    rsn_setPaeConfig_t      setPaeConfig;
    rsn_getNetworkMode_t    getNetworkMode;
    rsn_setKey_t            setKey;
    rsn_removeKey_t         removeKey;
    rsn_setDefaultKeyId_t   setDefaultKeyId;
    rsn_reportStatus_t      reportStatus;

    UINT32                  rsnStartedTs;
    UINT32                  rsnCompletedTs;
};

/* Structures */

/* External data definitions */

/* External functions definitions */

/* Function prototypes */

TI_STATUS rsn_reportStatus(rsn_t *pRsn, TI_STATUS status);

TI_STATUS rsn_setPaeConfig(rsn_t *pRsn, rsn_paeConfig_t *pPaeConfig);

TI_STATUS rsn_getNetworkMode(rsn_t *pRsn, rsn_networkMode_t *pNetMode);

TI_STATUS rsn_setKey(rsn_t *pMainSec, securityKeys_t *pKey);

TI_STATUS rsn_removeKey(rsn_t *pMainSec, securityKeys_t *pKey);

TI_STATUS rsn_setDefaultKeyId(rsn_t *pMainSec, UINT8 keyId);

TI_STATUS rsn_setDefaultKeys(rsn_t *pHandle);


#endif

