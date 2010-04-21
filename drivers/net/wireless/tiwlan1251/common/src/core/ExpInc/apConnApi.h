/** \file apConnApi.h
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
 *   MODULE:  AP Connection                                                 *
 *   PURPOSE: AP Connection Module API                                      *
 *                                                                          *
 ****************************************************************************/

#ifndef _AP_CONNECTION_API_H_
#define _AP_CONNECTION_API_H_

#include "paramOut.h"
#include "rsnApi.h"
#include "roamingMngrTypes.h"

/* Constants */

#define AP_CONNECT_TRIGGER_IGNORED  0x0

/* Enumerations */

/** 
* Requests to connect types 
*/
typedef enum
{
    AP_CONNECT_RETAIN_CURR_AP = 0,  /**< Give-up on roaming, return to current AP without performing re-connection */
    AP_CONNECT_RECONNECT_CURR_AP,   /**< Perform roaming - connect to AP, registered as current AP */
    AP_CONNECT_FAST_TO_AP,          /**< Perform roaming - re-connect to new AP via RE-Assoc, parameters attached */
    AP_CONNECT_FULL_TO_AP           /**< Perform full connection - connect to new AP via Assoc, parameters attached */
} apConn_connRequest_e;

/* triggers for Roaming */
typedef enum
{
    ROAMING_TRIGGER_NONE,

    ROAMING_TRIGGER_LOW_QUALITY_FOR_BG_SCAN,
    ROAMING_TRIGGER_NORMAL_QUALITY_FOR_BG_SCAN,

    ROAMING_TRIGGER_LOW_TX_RATE,
    ROAMING_TRIGGER_LOW_SNR,
    ROAMING_TRIGGER_LOW_QUALITY,

    ROAMING_TRIGGER_MAX_TX_RETRIES,
    ROAMING_TRIGGER_BSS_LOSS,
    ROAMING_TRIGGER_SWITCH_CHANNEL,

    ROAMING_TRIGGER_AP_DISCONNECT, /* DE_AUTH, DIS_ASSOC*/
    ROAMING_TRIGGER_SECURITY_ATTACK,

    ROAMING_TRIGGER_LAST
} apConn_roamingTrigger_e;

typedef enum
{
    CONN_STATUS_CONNECTED,
    CONN_STATUS_NOT_CONNECTED,
    CONN_STATUS_HANDOVER_FAILURE,
    CONN_STATUS_HANDOVER_SUCCESS,
    CONN_STATUS_LAST
} apConn_connStatus_e;

typedef enum
{
    REG_DOMAIN_FIXED,
    REG_DOMAIN_80211D,
    REG_DOMAIN_80211H
} REG_DOMAIN_CAPABILITY;
/* Typedefs */

/** 
* Roaming Manager callback type  
*/

typedef TI_STATUS (*apConn_roamMngrCallb_t) (TI_HANDLE hRoamingMngr, void *pData);

/* Structures */

typedef struct _apConn_staCapabilities_t
{
    /* None, Shared, AutoSwitch, WPA, WPAPSK, WPANone, WPA2, WPA2PSK */
    OS_802_11_AUTHENTICATION_MODE   authMode;  
    /* None, WEP, TKIP, AES */
    OS_802_11_ENCRYPTION_TYPES      encryptionType; 
    /* 2.4G, 5G or Dual */
    OS_802_11_NETWORK_TYPE          networkType;  
    /* An array of 16 octets. Each octet contains a preferred data rate in units of 0.5 Mbps */
    OS_802_11_RATES_EX              rateMask;  
    /* TRUE - EXC enabled, FALSE - EXC disabled */
    BOOL                            excEnabled; 
    /* TRUE - QOS enabled, FALSE - QOS disabled */
    BOOL                            qosEnabled; 
    /* Fixed, 802.11D, 802.11H */
    REG_DOMAIN_CAPABILITY           regDomain;  
} apConn_staCapabilities_t;

typedef struct _apConn_connStatus_t
{
    apConn_connStatus_e     status;         /** Reported status of the connection */
    UINT32                  dataBufLength;  /** (Optional) length of attached buffer */
    char                    *dataBuf;       /** (Optional) attached buffer - can be used in case of vendor specific IEs in Assoc resp packet */
} apConn_connStatus_t;

typedef struct _apConn_connRequest_t
{
    apConn_connRequest_e    requestType;    /** Type of request to establish connection */
    UINT32                  dataBufLength;  /** (Optional) length of attached buffer */
    char                    *dataBuf;       /** (Optional) attached buffer - can be used in case of vendor specific IEs in Assoc req packet */
} apConn_connRequest_t;

/* External data definitions */

/* External functions definitions */

/* Function prototypes */

/* Called by Roaming Manager */
TI_STATUS apConn_setRoamThresholds(TI_HANDLE hAPConnection, roamingMngrThresholdsConfig_t *pParam);
TI_STATUS apConn_getRoamThresholds(TI_HANDLE hAPConnection, roamingMngrThresholdsConfig_t *pParam);

TI_STATUS apConn_registerRoamMngrCallb(TI_HANDLE hAPConnection, 
                                       apConn_roamMngrCallb_t roamEventCallb,
                                       apConn_roamMngrCallb_t reportStatusCallb,
                                       apConn_roamMngrCallb_t returnNeighborApsCallb);
TI_STATUS apConn_unregisterRoamMngrCallb(TI_HANDLE hAPConnection);

TI_STATUS apConn_disconnect(TI_HANDLE hAPConnection);
TI_STATUS apConn_getStaCapabilities(TI_HANDLE hAPConnection,
                                    apConn_staCapabilities_t *ie_list);
TI_STATUS apConn_connectToAP(TI_HANDLE hAPConnection,
                             bssEntry_t *newAP,
                             apConn_connRequest_t *request,
                             BOOL reNegotiateTspec);
bssEntry_t *apConn_getBSSParams(TI_HANDLE hAPConnection);

BOOL apConn_isSiteBanned(TI_HANDLE hAPConnection, macAddress_t * bssid);

BOOL apConn_getPreAuthAPStatus(TI_HANDLE hAPConnection,
                              macAddress_t *givenAp);
TI_STATUS apConn_preAuthenticate(TI_HANDLE hAPConnection, bssList_t *listAPs);
TI_STATUS apConn_prepareToRoaming(TI_HANDLE hAPConnection, apConn_roamingTrigger_e reason);

#endif /*  _AP_CONNECTION_API_H_*/

