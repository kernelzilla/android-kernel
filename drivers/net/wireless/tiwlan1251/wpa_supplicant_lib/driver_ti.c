/*
 * WPA Supplicant - driver interaction with TI station
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Alternatively, this software may be distributed under the terms of BSD
 * license.
 *
 */

/* Copyright © Texas Instruments Incorporated (Oct 2005)
 * THIS CODE/PROGRAM IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, 
 * EITHER EXPRESS OR IMPLIED, INCLUDED BUT NOT LIMITED TO , THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. 
 * This program has been modified from its original operation by Texas 
 * Instruments Incorporated. These changes are covered under version 2 
 * of the GNU General Public License, dated June 1991.
 *
 * Copyright © Google Inc (Feb 2008)
*/
/*-------------------------------------------------------------------*/
#include "includes.h"
#include <sys/ioctl.h>
#include <net/route.h>
#include <net/if.h>
#include <fcntl.h>
#include <netpacket/packet.h>
#include <stddef.h>
#include "common.h"
#include "driver.h"
#include "eloop.h"
#include "wpa.h"
#include "wpa_supplicant.h"
#include "config.h"
#include "wpa_supplicant_i.h"
#include "wpa_i.h"
#include "l2_packet.h"
#include "wpa_ctrl.h"
/*----- STA_DK files -----*/
#include "wspVer.h"
#include "driver_ti.h"
#include "scanmerge.h"
#include "scanMngrTypes.h"
#ifdef ANDROID
#include <cutils/properties.h>
#endif
/*-------------------------------------------------------------------*/
#define TI_DRIVER_MSG_PORT       9000
#define RX_SELF_FILTER           0
#define RX_BROADCAST_FILTER      1
#define RX_IPV4_MULTICAST_FILTER 2
#define RX_IPV6_MULTICAST_FILTER 3
#define TI2WPA_STATUS(s)         (((s) != OK) ? -1 : 0)
#define TI_CHECK_DRIVER(f,r)     \
    if( !(f) ) { \
        wpa_printf(MSG_ERROR,"TI: Driver not initialized yet...aborting..."); \
        return( r ); \
    }
/*-------------------------------------------------------------------*/
/* Lock file pointer - used to access pid-lock-file to prevent two instances of wpa_supplicant */
#ifdef CONFIG_TI_LOCKFILE
static int lfp;
#endif
/*-------------------------------------------------------------------*/
#ifdef ANDROID
typedef struct REG_DOMAIN_STRUCT {
    char tmzn_name[PROPERTY_VALUE_MAX];
    int size;
    int num_of_channels;
} reg_domain_struct_t;

reg_domain_struct_t reg_domain_str[] = {
    { "US", 2, NUMBER_SCAN_CHANNELS_FCC },
    { "AU", 2, NUMBER_SCAN_CHANNELS_FCC },
    { "SG", 2, NUMBER_SCAN_CHANNELS_FCC },
    { "CA", 2, NUMBER_SCAN_CHANNELS_FCC },
    { "GB", 2, NUMBER_SCAN_CHANNELS_ETSI },
    { "JP", 2, NUMBER_SCAN_CHANNELS_MKK1 },
    { "ZZ", 2, NUMBER_SCAN_CHANNELS_FCC }
};
#endif
/*-----------------------------------------------------------------------------
Routine Name: check_and_get_carrier_channels
Routine Description: get number of allowed channels according to locale
as determined by the carrier being used.
Arguments: None
Return Value: Number of channels
-----------------------------------------------------------------------------*/
static int check_and_get_carrier_channels( void )
{
#ifdef ANDROID
    char prop_status[PROPERTY_VALUE_MAX];
    char *prop_name = "ro.product.locale.region";
    int default_channels = NUMBER_SCAN_CHANNELS_ETSI;
    unsigned i;

    if( !property_get(prop_name, prop_status, NULL) )
        return default_channels;
    for(i=0;( i < (sizeof(reg_domain_str)/sizeof(reg_domain_struct_t)) );i++) {
        if( strncmp(prop_status, reg_domain_str[i].tmzn_name,
                    reg_domain_str[i].size) == 0 )
            return reg_domain_str[i].num_of_channels;
    }
    return( default_channels );
#else
    return( NUMBER_SCAN_CHANNELS_FCC );
#endif
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_event_receive
Routine Description: driver events callback, called from driver IPC
Arguments:
   priv - pointer to private data structure
   pData - pointer to event information
Return Value:
-----------------------------------------------------------------------------*/
void wpa_driver_tista_event_receive( IPC_EV_DATA *pData )
{
    struct wpa_driver_ti_data *mySuppl;
    struct sockaddr_in echoserver;
    int res, msg_size;

    wpa_printf(MSG_DEBUG,"wpa_driver_tista_event_receive called: %d",
               pData->uBufferSize);

    mySuppl = pData->EvParams.hUserParam;
    msg_size = (int)(pData->uBufferSize + offsetof(IPC_EV_DATA, uBuffer));

    os_memset( &echoserver, 0, sizeof(echoserver) );     /* Clear struct */
    echoserver.sin_family = AF_INET;                     /* Internet/IP */
    echoserver.sin_addr.s_addr = inet_addr("127.0.0.1"); /* IP address */
    echoserver.sin_port = htons(TI_DRIVER_MSG_PORT);     /* server port */

    res = sendto(mySuppl->driverEventsSocket, pData, msg_size, 0, (struct sockaddr *)&echoserver, sizeof(echoserver));
    echoserver.sin_port = htons(TI_DRIVER_MSG_PORT + 1);
    res = sendto(mySuppl->driverEventsSocket, pData, msg_size, 0, (struct sockaddr *)&echoserver, sizeof(echoserver));
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_register_events
Routine Description: register to driver events
Arguments:
   ctx - pointer to private data structure
Return Value: None
-----------------------------------------------------------------------------*/
void wpa_driver_tista_register_events( void *ctx )
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)ctx;
    IPC_EVENT_PARAMS pEvent;
    int i;

    os_memset( myDrv->hEvents, 0, sizeof(ULONG) * IPC_EVENT_MAX );
    for(i=IPC_EVENT_ASSOCIATED;( i < IPC_EVENT_MAX );i++) {
        /* Register to receive driver events */
        pEvent.uEventType       = i;
        pEvent.uDeliveryType    = DELIVERY_PUSH;
        pEvent.hUserParam       = (TI_HANDLE)myDrv;
        pEvent.pfEventCallback  = (TI_EVENT_CALLBACK)wpa_driver_tista_event_receive;
        TI_RegisterEvent( myDrv->hDriver, &pEvent );
        myDrv->hEvents[i] = pEvent.uEventID;
    }
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_unregister_events
Routine Description: unregister driver events
Arguments:
   ctx - pointer to private data structure
Return Value: None
-----------------------------------------------------------------------------*/
void wpa_driver_tista_unregister_events( void *ctx )
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)ctx;
    IPC_EVENT_PARAMS pEvent;
    int idx;

    for(idx=0;( idx < IPC_EVENT_MAX );idx++) {
        if( myDrv->hEvents[idx] ) {
            pEvent.uEventType = idx;
            pEvent.uEventID = myDrv->hEvents[idx];
            TI_UnRegisterEvent( myDrv->hDriver, &pEvent );
        }
    }
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_get_bssid
Routine Description: get current BSSID from driver
Arguments:
   priv - pointer to private data structure
   bssid - pointer to hold bssid
Return Value: 0 on success, -1 on failure
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_get_bssid( void *priv, u8 *bssid )
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;
    OS_802_11_MAC_ADDRESS tiAPMacAddr;
    TI_STATUS retValue;

    wpa_printf(MSG_DEBUG, "wpa_driver_tista_get_bssid called");

    /* If driver is not initialized yet - we cannot access it so return */
    TI_CHECK_DRIVER( myDrv->driver_is_loaded, -1 );

    /* Get MAC address of current AP */
    if( TI_GetBSSID( myDrv->hDriver, &tiAPMacAddr ) != TI_RESULT_OK )
        return( -1 );

    /* Copy BSSID into caller pointer provided in routine parameters */
    os_memcpy( (void *)bssid, (void *)&tiAPMacAddr, MAC_ADDR_LEN );
    wpa_hexdump(MSG_DEBUG, "get_bssid:", bssid, MAC_ADDR_LEN);
    return( 0 );
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_get_ssid
Routine Description: get current SSID
Arguments:
   priv - pointer to private data structure
   ssid - pointer to hold current bssid
Return Value: Length of SSID string
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_get_ssid( void *priv, u8 *ssid )
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;
    OS_802_11_SSID myssid;

    wpa_printf(MSG_DEBUG,"wpa_driver_tista_get_ssid called");

    /* If driver is not initialized yet - we cannot access it so return */
    TI_CHECK_DRIVER( myDrv->driver_is_loaded, -1 );

    /* Get current SSID from driver */
    if( TI_GetCurrentSSID(myDrv->hDriver, &myssid) != TI_RESULT_OK )
        return( -1 );

    /* Copy to user supplied pointer */
    os_memcpy( (void *)ssid, (void *)&myssid.Ssid, myssid.SsidLength );

    /* Return length of SSID */
    return( myssid.SsidLength );
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_set_ssid
Routine Description: sets current SSID (Associates)
Arguments:
   priv - pointer to private data structure
   ssid - pointer to ssid
   ssid_len - length of ssid
Return Value: 0 on success, -1 on failure
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_set_ssid( void *priv, const u8 *ssid, size_t ssid_len )
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;
    char ssidBuf[MAX_SSID_LEN];
    int ret;

    wpa_printf(MSG_DEBUG,"wpa_driver_tista_set_ssid called: %s", ssid);

    /* If driver is not initialized yet - we cannot access it so return */
    TI_CHECK_DRIVER( myDrv->driver_is_loaded, -1 );

    /* Copy user supplied SSID into local buffer */
    os_memset( ssidBuf, 0, MAX_SSID_LEN );
    os_memcpy( ssidBuf, ssid, ssid_len );

    /* Set local SSID buffer to driver - triggering connection process in driver */
    wpa_printf(MSG_DEBUG,"Associate: SSID = %s", ssidBuf); /* Dm: */
#ifdef STA_DK_VER_5_0_0_94
    ret = (int)TI_SetSSID( myDrv->hDriver, (char *)ssidBuf );
#else
    ret = (int)TI_SetSSID( myDrv->hDriver, (u8 *)ssidBuf );
#endif
    return( TI2WPA_STATUS(ret) );
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_set_wpa
Routine Description: enable/disable WPA support in driver - not implemented since not supported in driver. also obselete for wpa-suppl core
Arguments:
   priv - pointer to private data structure
   enabled - enable/disable flag
Return Value: 0 on success, -1 on failure
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_set_wpa(void *priv, int enabled)
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;

    wpa_printf(MSG_DEBUG,"wpa_driver_tista_set_wpa called: %d",enabled);
    return( 0 );
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_set_proto
Routine Description: set authentication protocol (WPA/WPA2(RSN))
Arguments:
   priv - pointer to private data structure
   proto - authentication suite
Return Value: 0 on success, -1 on failure
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_set_proto( void *priv, int proto )
{
   struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;

   wpa_printf(MSG_DEBUG,"wpa_driver_tista_set_proto called: %d", proto);
   myDrv->proto = proto;

   return( 0 );
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_set_wpa_options
Routine Description: set wpa_options
Arguments:
   priv - pointer to private data structure
   wpa_options - WPA options (0 - disable, 3 - enable)
Return Value: 0 on success, -1 on failure
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_set_wpa_options( void *priv, int key_mgmt_suite )
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;
    UINT32 wpa_opt = WPA_OPTIONS_DISABLE;
    int ret;

    wpa_printf(MSG_DEBUG,"wpa_driver_tista_set_wpa_options called: %d", key_mgmt_suite);
    /* If driver is not initialized yet - we cannot access it so return */
    TI_CHECK_DRIVER( myDrv->driver_is_loaded, -1 );

    myDrv->key_mgmt = key_mgmt_suite;

    switch( key_mgmt_suite ) {
        case KEY_MGMT_802_1X:          /* Dm: EAP */
            wpa_opt = WPA_OPTIONS_ENABLE; /* wpa_auth = 1; */
            break;

        case KEY_MGMT_PSK:             /* Dm: PSK */
            wpa_opt = WPA_OPTIONS_ENABLE; /* wpa_auth = 2; */
            break;

        case KEY_MGMT_802_1X_NO_WPA:   /* Dm: ??? */
            break;

        case KEY_MGMT_WPA_NONE:        /* Dm: ??? */
            break;

        case KEY_MGMT_NONE:
        default:
            wpa_opt = WPA_OPTIONS_DISABLE; /* wpa_auth = 255; */
            break;
    }

    /* Set WPA Options */
    ret = TI_SetWpaOptions( myDrv->hDriver, wpa_opt );
    return( TI2WPA_STATUS(ret) );
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_set_encryption
Routine Description: set authentication protocol (WPA/WPA2(RSN))
Arguments:
   priv - pointer to private data structure
   proto - authentication suite
Return Value: 0 on success, -1 on failure
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_set_encryption( void *priv, int encryption )
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;
    int ret = -1;
    OS_802_11_ENCRYPTION_TYPES wpa_cipher = OS_ENCRYPTION_TYPE_NONE;

    wpa_printf(MSG_DEBUG,"wpa_driver_tista_set_encryption called: %d",encryption);
    /* If driver is not initialized yet - we cannot access it so return */
    TI_CHECK_DRIVER( myDrv->driver_is_loaded, -1 );

    myDrv->encryption = encryption;
    switch( encryption ) {
        case CIPHER_WEP40:
        case CIPHER_WEP104:
            wpa_printf(MSG_DEBUG, "encryption: OS_ENCRYPTION_TYPE_WEP");
            wpa_cipher = OS_ENCRYPTION_TYPE_WEP;
            break;

        case CIPHER_TKIP:
            wpa_printf(MSG_DEBUG, "encryption: OS_ENCRYPTION_TYPE_TKIP");
            wpa_cipher = OS_ENCRYPTION_TYPE_TKIP;
            break;

        case CIPHER_CCMP:
            wpa_printf(MSG_DEBUG, "encryption: OS_ENCRYPTION_TYPE_AES");
            wpa_cipher = OS_ENCRYPTION_TYPE_AES;
            break;

        case CIPHER_NONE:
        default:
            wpa_printf(MSG_DEBUG, "encryption: OS_ENCRYPTION_TYPE_NONE");
            wpa_cipher = OS_ENCRYPTION_TYPE_NONE;
            break;
    }
    ret = TI_SetEncryptionType( myDrv->hDriver, wpa_cipher );
    return( TI2WPA_STATUS(ret) );
}

/*---------------------------------------------------------------------------*/
void wpa_driver_tista_print_auth_mode( OS_802_11_AUTHENTICATION_MODE myAuth )
{
    char *mode_name = NULL;

    switch( myAuth ) {
        case os802_11AuthModeOpen:
            mode_name = "os802_11AuthModeOpen";
            break;
        case os802_11AuthModeShared:
            mode_name = "os802_11AuthModeShared";
            break;
        case os802_11AuthModeAutoSwitch:
            mode_name = "os802_11AuthModeAutoSwitch";
            break;
        case os802_11AuthModeWPA:
            mode_name = "os802_11AuthModeWPA";
            break;
        case os802_11AuthModeWPAPSK:
            mode_name = "os802_11AuthModeWPAPSK";
            break;
        case os802_11AuthModeWPANone:
            mode_name = "os802_11AuthModeWPANone";
            break;
        case os802_11AuthModeWPA2:
            mode_name = "os802_11AuthModeWPA2";
            break;
        case os802_11AuthModeWPA2PSK:
            mode_name = "os802_11AuthModeWPA2PSK";
            break;
        case os802_11AuthModeMax:
        default:
            mode_name = "Unknown";
            break;
    }
    wpa_printf(MSG_DEBUG, "Selected AuthMode: %s", mode_name);
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_set_auth_mode
Routine Description:
Arguments:
   priv - pointer to private data structure
Return Value: 0 on success, -1 on failure
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_set_auth_mode( void *priv )
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;
    OS_802_11_AUTHENTICATION_MODE myAuth = os802_11AuthModeAutoSwitch;
    int ret;

    wpa_printf(MSG_DEBUG,"wpa_driver_tista_set_auth_mode called");

    /* If driver is not initialized yet - we cannot access it so return */
    TI_CHECK_DRIVER( myDrv->driver_is_loaded, -1 );

    wpa_driver_tista_set_proto( priv, ((struct wpa_supplicant *)myDrv->hWpaSupplicant)->wpa->proto );
    wpa_printf(MSG_DEBUG, "proto: %d", myDrv->proto); /* should be set BEFORE */
    wpa_printf(MSG_DEBUG, "auth_alg: %d",myDrv->auth_alg);

    if( myDrv->auth_alg == AUTH_ALG_OPEN_SYSTEM ) {
        switch( myDrv->key_mgmt ) {
            case KEY_MGMT_802_1X:
                if( myDrv->proto & WPA_PROTO_WPA ) {
                    myAuth = os802_11AuthModeWPA;
                }
                else if( myDrv->proto & WPA_PROTO_RSN ) {
                    myAuth = os802_11AuthModeWPA2;
                }
                break;
            case KEY_MGMT_PSK:
                if( myDrv->proto & WPA_PROTO_WPA ) {
                    myAuth = os802_11AuthModeWPAPSK;
                }
                else if( myDrv->proto & WPA_PROTO_RSN ) {
                    myAuth = os802_11AuthModeWPA2PSK;
                }
                break;
            case KEY_MGMT_802_1X_NO_WPA:
            case KEY_MGMT_WPA_NONE:
                myAuth = os802_11AuthModeWPANone;
                break;
            case KEY_MGMT_NONE:
            default:
                myAuth = os802_11AuthModeOpen;
                break;
        }
    }
    else if( myDrv->auth_alg == AUTH_ALG_SHARED_KEY ) {
        myAuth = os802_11AuthModeShared;
    }
    else if( myDrv->auth_alg == AUTH_ALG_LEAP ) {                 /* Dm: ??? */
        myAuth = os802_11AuthModeWPA;
    }
    wpa_driver_tista_print_auth_mode( myAuth );
    ret = TI_SetAuthenticationMode( myDrv->hDriver, myAuth );
    return( TI2WPA_STATUS(ret) );
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_del_key
Routine Description: remove key from driver
Arguments:
   priv - pointer to private data structure
   key_idx - key index
   addr - key address (unicast/broadcast)
Return Value: 0 on success, -1 on failure
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_del_key(void *priv, int key_idx,
                       const unsigned char *addr)
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;
    OS_802_11_REMOVE_KEY myKey;
    int ret;

    wpa_printf(MSG_DEBUG,"wpa_driver_tista_del_key called");
    wpa_printf(MSG_DEBUG,"key_idx = %d, addr = " TIMACSTR, key_idx, MAC2STR(addr));

    /* If driver is not initialized yet - we cannot access it so return */
    TI_CHECK_DRIVER( myDrv->driver_is_loaded, -1 );

    /* copy parameters (MAC of key to remove, etc) to local structure */
    myKey.Length = sizeof(OS_802_11_REMOVE_KEY);
    os_memcpy(&myKey.BSSID,addr,MAC_ADDR_LEN);
    myKey.KeyIndex = key_idx;

    /* Call Utility adapter to remove the key */
    ret = TI_RemoveKey( myDrv->hDriver, &myKey );
    return( TI2WPA_STATUS(ret) );
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_set_key
Routine Description: set key in driver
Arguments:
   priv - pointer to private data structure
   alg - type of key
   addr - key address (unicast/broadcast)
   key_idx - key index
   set_tx - use key for immidiate tx
   seq - sequence counter (for replay detection)
   seq_len - sequence counter buffer len
   key - key content
   key_len - length of key (32 for TKIP, 16 for AES)
Return Value: 0 on success, -1 on failure
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_set_key(void *priv, wpa_alg alg,
                       const unsigned char *addr, int key_idx, int set_tx,
                       const u8 *seq, size_t seq_len,
                       const u8 *key, size_t key_len)
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;
    OS_802_11_KEY myKey;
    OS_802_11_WEP myWepKey;
    UINT8 temp[TKIP_KEY_LENGTH];
    int ret = -1;

    wpa_printf(MSG_DEBUG,"wpa_driver_tista_set_key called");
    /* If driver is not initialized yet - we cannot access it so return */
    TI_CHECK_DRIVER( myDrv->driver_is_loaded, -1 );

    wpa_printf(MSG_DEBUG, "add_key (addr): " MACSTR, MAC2STR(addr));
    wpa_printf(MSG_DEBUG, "add_key (key_idx): %d", key_idx);
    wpa_printf(MSG_DEBUG, "add_key (alg): %d", alg);
    wpa_printf(MSG_DEBUG, "add_key (set_tx): %d", set_tx);
    wpa_hexdump(MSG_DEBUG, "add_key (seq):", seq, seq_len);
    wpa_hexdump(MSG_DEBUG, "add_key (key):", key, key_len);

    switch( key_len ) {
        case TKIP_KEY_LENGTH:          /* 32 */
        case AES_KEY_LENGTH:           /* 16 */
            /* Set key index */
            myKey.KeyIndex = key_idx;
            /* Set key length and content */
            myKey.KeyLength = key_len;
            /* Set structure size */
            myKey.Length = sizeof(OS_802_11_KEY);
            /* set key MAC address - FF:FF:FF:FF:FF:FF for broadcast, other for unicast */
            os_memcpy( &myKey.BSSID, addr, MAC_ADDR_LEN );

            if( seq_len ) {
                /* Set key RSC */
                os_memcpy( &myKey.KeyRSC, seq, seq_len );
                myKey.KeyIndex |= TIWLAN_KEY_FLAGS_SET_KEY_RSC;
            }

            if( set_tx ) {
                myKey.KeyIndex |= TIWLAN_KEY_FLAGS_TRANSMIT;
            }

            if( myKey.BSSID[0] != 0xFF ) {
                myKey.KeyIndex |= TIWLAN_KEY_FLAGS_PAIRWISE;
            }

            os_memcpy( &temp, key, AES_KEY_LENGTH );
            if( key_len == TKIP_KEY_LENGTH ) {
                /* need to switch RX and TX MIC set with key (to match driver API) */
                os_memcpy( (UINT8*)(((UINT8*)&temp)+24), (UINT8*)(((UINT8*)key)+16), 8 );
                os_memcpy( (UINT8*)(((UINT8*)&temp)+16), (UINT8*)(((UINT8*)key)+24), 8 );
            }
            os_memcpy( &myKey.KeyMaterial, &temp, key_len );
            ret = TI_AddKey( myDrv->hDriver, &myKey );
            break;

        case WEP_KEY_LENGTH_40:        /* 5 */
        case WEP_KEY_LENGTH_104:       /* 13 */
            if( key_len != 0 ) {
                /* Set key index */
                myWepKey.KeyIndex = key_idx;
                /* Set key length and content */
                myWepKey.KeyLength = key_len;
                /* Set structure size */
                myWepKey.Length = sizeof(OS_802_11_WEP);

                if( set_tx ) {
                    myWepKey.KeyIndex |= TIWLAN_KEY_FLAGS_TRANSMIT;
                    wpa_printf(MSG_DEBUG, "setting this key to be the index: 0x%x", myWepKey.KeyIndex);
                }

                os_memcpy( &myWepKey.KeyMaterial, key, key_len );
                wpa_printf(MSG_DEBUG, "Adding WEP key index: 0x%x", key_idx);
                ret = TI_AddWEPKey( myDrv->hDriver, &myWepKey );
            }
            else {
                wpa_printf(MSG_DEBUG, "Removing WEP key index: 0x%x", key_idx);
                ret = TI_RemoveWEPKey( myDrv->hDriver, key_idx );
            }
            break;

        default:
            wpa_printf(MSG_ERROR,"Set_key: Wrong Key\n");
            break;
    }
    return( TI2WPA_STATUS(ret) );
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_set_countermeasures
Routine Description: start/stop countermeasures (drop packets due to replay attack detection)
Arguments:
   priv - pointer to private data structure
   enabled - enable/disable flag
Return Value: 0 on success, -1 on failure
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_set_countermeasures(void *priv, int enabled)
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;

    wpa_printf(MSG_DEBUG,"wpa_driver_tista_set_countermeasures called: %d", enabled);
    return( 0 );
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_set_drop_unencrypted
Routine Description: enable/disable EAPOL-only tx by driver
Arguments:
   priv - pointer to private data structure
   enabled - enable/disable flag
Return Value: 0 on success, -1 on failure
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_set_drop_unencrypted(void *priv, int enabled)
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;

    wpa_printf(MSG_DEBUG,"TI: wpa_driver_tista_set_drop_unencrypted called: %d - not implemented", enabled);
    return( 0 );
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_deauthenticate
Routine Description: send deauthentication packet
Arguments:
   priv - pointer to private data structure
   addr - address to send deauth packet to
   reason_code - reason code supplied in deauth packet
Return Value: 0 on success, -1 on failure
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_deauthenticate(void *priv, const UINT8 *addr, int reason_code)
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;
    int ret;

    wpa_printf(MSG_DEBUG,"wpa_driver_tista_deauthenticate called");
    /* If driver is not initialized yet - we cannot access it so return */
    TI_CHECK_DRIVER( myDrv->driver_is_loaded, -1 );

    /* Block any pending disassoc event until successfully connected */
    if( myDrv->block_disassoc_events == NO_BLOCK )
        myDrv->block_disassoc_events = NO_BLOCK_DISASSOC_IN_PROGRESS;

    ret = TI_Disassociate( myDrv->hDriver );
    return( TI2WPA_STATUS(ret) );
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_disassociate
Routine Description: disassociate from AP
Arguments:
   priv - pointer to private data structure
   addr - address to send deauth packet to
   reason_code - reason code supplied in deauth packet
Return Value: 0 on success, -1 on failure
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_disassociate(void *priv, const UINT8 *addr, int reason_code)
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;
    int ret;

    wpa_printf(MSG_DEBUG,"TI: wpa_driver_tista_disassociate called");
    /* If driver is not initialized yet - we cannot access it so return */
    TI_CHECK_DRIVER( myDrv->driver_is_loaded, -1 );

    /* Block any pending disassoc event until successfully connected */
    if( myDrv->block_disassoc_events == NO_BLOCK )
        myDrv->block_disassoc_events = NO_BLOCK_DISASSOC_IN_PROGRESS;

    ret = TI_Disassociate( myDrv->hDriver );
    return( TI2WPA_STATUS(ret) );
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_associate
Routine Description: associate with AP
Arguments:
   priv - pointer to private data structure
   params - struct wpa_driver_associate_params (ssid, bssid, etc)
Return Value: 0 on success, -1 on failure
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_associate(void *priv, struct wpa_driver_associate_params *params)
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;
    OS_802_11_MAC_ADDRESS bssid = { 0xff,0xff,0xff,0xff,0xff,0xff };
    int wpa_opt, wpa_cipher, ret;

    wpa_printf(MSG_DEBUG,"TI: wpa_driver_tista_associate called");
    /* If driver is not initialized yet - we cannot access it so return */
    TI_CHECK_DRIVER( myDrv->driver_is_loaded, -1 );

    if( myDrv->block_disassoc_events == NO_BLOCK_DISASSOC_IN_PROGRESS )
        myDrv->block_disassoc_events = BLOCK_DISASSOC;

    if( params->bssid ) {
        wpa_printf(MSG_DEBUG, "TI: BSSID=" MACSTR, MAC2STR(params->bssid));
        /* if there is bssid -> set it */
        if( os_memcmp( params->bssid, "\x00\x00\x00\x00\x00\x00", ETH_ALEN ) != 0 ) {
            os_memcpy( &bssid, params->bssid, ETH_ALEN );
            TI_SetBSSID( myDrv->hDriver, &bssid );
        }
    }
    else {
        /* else set it to {0xff, 0xff, 0xff, 0xff, 0xff, 0xff} */
        TI_SetBSSID( myDrv->hDriver, &bssid );
    }

    /* Set driver network mode (Adhoc/Infrastructure) according to supplied parameters */
    if( params->mode == IEEE80211_MODE_INFRA ) {
        wpa_printf(MSG_DEBUG,"TI: setting os802_11Infrastructure mode...");
        TI_SetBSSType( myDrv->hDriver, os802_11Infrastructure );
    }
    else if( params->mode == IEEE80211_MODE_IBSS ) {
        wpa_printf(MSG_DEBUG,"TI: setting os802_11IBSS mode...");
        TI_SetBSSType( myDrv->hDriver, os802_11IBSS );
    }
    else {
        wpa_printf(MSG_ERROR,"TI: Associate: invalid mode specified...");
    }

    wpa_driver_tista_set_wpa_options( priv, params->key_mgmt_suite );
    wpa_driver_tista_set_auth_mode( priv );
    wpa_driver_tista_set_encryption( priv, params->pairwise_suite );

    /* And trigger connection/association process in driver by setting SSID */
    ret = wpa_driver_tista_set_ssid( priv, params->ssid, params->ssid_len);
    return( ret );
}

/*-----------------------------------------------------------------------------
Routine Name: ti_init_scan_params
Routine Description: int scan parameters before scan command
Arguments:
   pScanParams - pointer to scan paramters structure
   pScanPolicy - pointer to scan policy structure
   scanType    - scan type
   noOfChan    - number of allowed channels
Return Value: None
-----------------------------------------------------------------------------*/
static void ti_init_scan_params( scan_Params_t *pScanParams,
                                 scan_Policy_t *pScanPolicy,
                                 struct wpa_driver_ti_data *myDrv )
{
    UINT32 scanMaxDwellTime = SME_SCAN_BG_MAX_DWELL_TIME_DEF;
    UINT32 scanMinDwellTime = SME_SCAN_BG_MIN_DWELL_TIME_DEF;
    UINT32 chanMaxDwellTime = SME_SCAN_BG_MIN_DWELL_TIME_DEF;
    UINT32 chanMinDwellTime = SME_SCAN_BG_MIN_DWELL_TIME_DEF / 2;
    int scanType = myDrv->scan_type;
    int noOfChan = myDrv->scan_channels;
    int btCoexScan = myDrv->btcoex_scan;
    int i, j;
    UINT8 tid = 0, probeNum = 3;

    if( noOfChan > MAX_NUMBER_OF_CHANNELS_PER_SCAN )
        noOfChan = MAX_NUMBER_OF_CHANNELS_PER_SCAN;
    /* init application scan default params */
    pScanParams->desiredSsid.len = 0;
    pScanParams->band = RADIO_BAND_2_4_GHZ;
    if( btCoexScan ) { /* Changing scan parameteres to coexist with BT A2DP */
        if( scanType == SCAN_TYPE_NORMAL_PASSIVE )
            scanType = SCAN_TYPE_TRIGGERED_PASSIVE;
        else if( scanType == SCAN_TYPE_NORMAL_ACTIVE )
            scanType = SCAN_TYPE_TRIGGERED_ACTIVE;
        probeNum = 1;
        tid = 0xFF;
        scanMaxDwellTime /= 6;
        scanMinDwellTime /= 6;
        chanMaxDwellTime /= 6;
        chanMinDwellTime /= 6;
    }
    pScanParams->scanType = scanType;
    pScanParams->probeReqNumber = probeNum;
    pScanParams->Tid = tid;
    pScanParams->probeRequestRate = DRV_RATE_MASK_2_BARKER;
    pScanParams->numOfChannels = (UINT8)noOfChan;
    for(i=0;( i < noOfChan );i++) {
        for(j=0;( j < MAC_ADDR_LEN );j++) {
            pScanParams->channelEntry[ i ].normalChannelEntry.bssId.addr[ j ] = 0xff;
        }
        pScanParams->channelEntry[ i ].normalChannelEntry.earlyTerminationEvent = SCAN_ET_COND_DISABLE;
        pScanParams->channelEntry[ i ].normalChannelEntry.ETMaxNumOfAPframes = 0;
        pScanParams->channelEntry[ i ].normalChannelEntry.maxChannelDwellTime = scanMaxDwellTime;
        pScanParams->channelEntry[ i ].normalChannelEntry.minChannelDwellTime = scanMinDwellTime;
#ifdef STA_DK_VER_5_0_0_94
        pScanParams->channelEntry[ i ].normalChannelEntry.txPowerLevel = 1;
#else
        pScanParams->channelEntry[ i ].normalChannelEntry.txPowerDbm = MAX_TX_POWER;
#endif
        pScanParams->channelEntry[ i ].normalChannelEntry.channel = i + 1;
    }

    /* init default scan policy */
    pScanPolicy->normalScanInterval = 10000;
    pScanPolicy->deterioratingScanInterval = 5000;
    pScanPolicy->maxTrackFailures = 3;
    pScanPolicy->BSSListSize = 4;
    pScanPolicy->BSSNumberToStartDiscovery = 1;
    pScanPolicy->numOfBands = 1;
    pScanPolicy->bandScanPolicy[ 0 ].band = RADIO_BAND_2_4_GHZ;
    pScanPolicy->bandScanPolicy[ 0 ].rxRSSIThreshold = -80;
    pScanPolicy->bandScanPolicy[ 0 ].numOfChannles = (UINT8)noOfChan;
    pScanPolicy->bandScanPolicy[ 0 ].numOfChannlesForDiscovery = 3;
    for(i=0;( i < noOfChan );i++) {
        pScanPolicy->bandScanPolicy[ 0 ].channelList[ i ] = i + 1;
    }
    pScanPolicy->bandScanPolicy[ 0 ].trackingMethod.scanType = scanType;
    pScanPolicy->bandScanPolicy[ 0 ].trackingMethod.method.basicMethodParams.earlyTerminationEvent = SCAN_ET_COND_DISABLE;
    pScanPolicy->bandScanPolicy[ 0 ].trackingMethod.method.basicMethodParams.ETMaxNumberOfApFrames = 0;
    pScanPolicy->bandScanPolicy[ 0 ].trackingMethod.method.basicMethodParams.maxChannelDwellTime = chanMaxDwellTime;
    pScanPolicy->bandScanPolicy[ 0 ].trackingMethod.method.basicMethodParams.minChannelDwellTime = chanMinDwellTime;
    pScanPolicy->bandScanPolicy[ 0 ].trackingMethod.method.basicMethodParams.probReqParams.bitrate = DRV_RATE_MASK_1_BARKER;
    pScanPolicy->bandScanPolicy[ 0 ].trackingMethod.method.basicMethodParams.probReqParams.numOfProbeReqs = pScanParams->probeReqNumber;
#ifdef STA_DK_VER_5_0_0_94
    pScanPolicy->bandScanPolicy[ 0 ].trackingMethod.method.basicMethodParams.probReqParams.txLevel = 1;
#else
    pScanPolicy->bandScanPolicy[ 0 ].trackingMethod.method.basicMethodParams.probReqParams.txPowerDbm = MAX_TX_POWER;
#endif
    pScanPolicy->bandScanPolicy[ 0 ].discoveryMethod.scanType = scanType;
    pScanPolicy->bandScanPolicy[ 0 ].discoveryMethod.method.basicMethodParams.earlyTerminationEvent = SCAN_ET_COND_DISABLE;
    pScanPolicy->bandScanPolicy[ 0 ].discoveryMethod.method.basicMethodParams.ETMaxNumberOfApFrames = 0;
    pScanPolicy->bandScanPolicy[ 0 ].discoveryMethod.method.basicMethodParams.maxChannelDwellTime = chanMaxDwellTime;
    pScanPolicy->bandScanPolicy[ 0 ].discoveryMethod.method.basicMethodParams.minChannelDwellTime = chanMinDwellTime;
    pScanPolicy->bandScanPolicy[ 0 ].discoveryMethod.method.basicMethodParams.probReqParams.bitrate = DRV_RATE_MASK_2_BARKER;
    pScanPolicy->bandScanPolicy[ 0 ].discoveryMethod.method.basicMethodParams.probReqParams.numOfProbeReqs = pScanParams->probeReqNumber;
#ifdef STA_DK_VER_5_0_0_94
    pScanPolicy->bandScanPolicy[ 0 ].discoveryMethod.method.basicMethodParams.probReqParams.txLevel = 1;
#else
    pScanPolicy->bandScanPolicy[ 0 ].discoveryMethod.method.basicMethodParams.probReqParams.txPowerDbm = MAX_TX_POWER;
#endif
    pScanPolicy->bandScanPolicy[ 0 ].immediateScanMethod.scanType = scanType;
    pScanPolicy->bandScanPolicy[ 0 ].immediateScanMethod.method.basicMethodParams.earlyTerminationEvent = SCAN_ET_COND_DISABLE;
    pScanPolicy->bandScanPolicy[ 0 ].immediateScanMethod.method.basicMethodParams.ETMaxNumberOfApFrames = 0;
    pScanPolicy->bandScanPolicy[ 0 ].immediateScanMethod.method.basicMethodParams.maxChannelDwellTime = chanMaxDwellTime;
    pScanPolicy->bandScanPolicy[ 0 ].immediateScanMethod.method.basicMethodParams.minChannelDwellTime = chanMinDwellTime;
    pScanPolicy->bandScanPolicy[ 0 ].immediateScanMethod.method.basicMethodParams.probReqParams.bitrate = DRV_RATE_MASK_5_5_CCK;
    pScanPolicy->bandScanPolicy[ 0 ].immediateScanMethod.method.basicMethodParams.probReqParams.numOfProbeReqs = pScanParams->probeReqNumber;
#ifdef STA_DK_VER_5_0_0_94
    pScanPolicy->bandScanPolicy[ 0 ].immediateScanMethod.method.basicMethodParams.probReqParams.txLevel = 1;
#else
    pScanPolicy->bandScanPolicy[ 0 ].immediateScanMethod.method.basicMethodParams.probReqParams.txPowerDbm = MAX_TX_POWER;
#endif
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_scan
Routine Description: request scan from driver
Arguments:
   priv - pointer to private data structure
   ssid - ssid buffer
   ssid_len - length of ssid
Return Value: 0 on success, -1 on failure
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_scan( void *priv, const UINT8 *ssid, size_t ssid_len )
{
    scan_Params_t scanParams;
    scan_Policy_t scanPolicy;
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;
    int ret;

    wpa_printf(MSG_DEBUG,"wpa_driver_tista_scan called");
    /* If driver is not initialized yet - we cannot access it so return */
    TI_CHECK_DRIVER( myDrv->driver_is_loaded, -1 );

    ti_init_scan_params( &scanParams, &scanPolicy, myDrv );
    if (ssid && ssid_len > 0 && ssid_len <= sizeof(scanParams.desiredSsid.ssidString)) {
        os_memcpy(scanParams.desiredSsid.ssidString, ssid, ssid_len);
        scanParams.desiredSsid.len = ssid_len;
    }
    TI_SetScanPolicy( myDrv->hDriver, (UINT8 *)&scanPolicy, sizeof(scan_Policy_t) );
    myDrv->last_scan = myDrv->scan_type; /* Remember scan type for last scan */
    ret = TI_StartScan( myDrv->hDriver, (scan_Params_t *)&scanParams );
    return( TI2WPA_STATUS(ret) );
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_set_auth_alg
Routine Description: set authentication in driver
Arguments:
   priv - pointer to private data structure
   auth_alg - Open/Shared/LEAP
Return Value: 0 on success, -1 on failure
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_set_auth_alg( void *priv, int auth_alg )
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;

    wpa_printf(MSG_DEBUG,"wpa_driver_tista_set_auth_alg called: %d", auth_alg);
    /* If driver is not initialized yet - we cannot access it so return */
    TI_CHECK_DRIVER( myDrv->driver_is_loaded, -1 );
    myDrv->auth_alg = auth_alg;
    return( 0 );
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_get_bssid_info
Routine Description: retrieve bssid full info
Arguments:
   hDriver - pointer to driver structure
Return Value: pointer to BSSID structure or NULL
-----------------------------------------------------------------------------*/
static OS_802_11_BSSID_EX *wpa_driver_tista_get_bssid_info( TI_HANDLE hDriver )
{
    OS_802_11_BSSID_EX mySelectedBssidInfo;
    OS_802_11_BSSID_LIST_EX *bssid_list;
    OS_802_11_BSSID_EX *pBssid, *nBssid = NULL;
    int i, number_items, res;

    res = TI_GetSelectedBSSIDInfo( hDriver, &mySelectedBssidInfo );
    if( res != TI_RESULT_OK )
        return( nBssid );

    if( TI_GetBSSIDList( hDriver, &bssid_list ) || !bssid_list )
        return( nBssid );

    pBssid = &bssid_list->Bssid[0];
    number_items = (int)(bssid_list->NumberOfItems);
    for(i=0;( i < number_items );i++) {
        if( os_memcmp( mySelectedBssidInfo.Ssid.Ssid, pBssid->Ssid.Ssid, pBssid->Ssid.SsidLength ) == 0 ) {
            nBssid = (OS_802_11_BSSID_EX *)os_malloc( pBssid->Length );
            if( nBssid != NULL )
                os_memcpy( nBssid, pBssid, pBssid->Length );
            break;
        }
        pBssid = (OS_802_11_BSSID_EX *)(((char *)pBssid) + pBssid->Length);
    }
    os_free( bssid_list );
    return( nBssid );
}

/*-----------------------------------------------------------------------------
Compare function for sorting scan results. Return >0 if @b is considered better.
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_scan_result_compare(const void *a, const void *b)
{
    const struct wpa_scan_result *wa = a;
    const struct wpa_scan_result *wb = b;

    return( wb->level - wa->level );
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_get_scan_results
Routine Description: retrieve driver scan results
Arguments:
   priv - pointer to private data structure
   results - pointer to buffer
   max_size - maximum size of results buffer
Return Value: number of SSID on success, -1 on failure
-----------------------------------------------------------------------------*/
static int wpa_driver_tista_get_scan_results( void *priv,
                                     struct wpa_scan_result *results,
                                     size_t max_size )
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;
    OS_802_11_BSSID_LIST_EX *bssid_list;
    OS_802_11_BSSID_EX *pBssid;
    OS_802_11_FIXED_IEs *pFixedIes;
    OS_802_11_VARIABLE_IEs *pVarIes;
    unsigned int number_items, i, index;

    wpa_printf(MSG_DEBUG,"wpa_driver_tista_get_scan_results called");
    /* If driver is not initialized yet - we cannot access it so return */
    TI_CHECK_DRIVER( myDrv->driver_is_loaded, -1 );

    if( TI_GetBSSIDList(myDrv->hDriver, &bssid_list) || !bssid_list )
        return( -1 );

    pBssid = &bssid_list->Bssid[0];
    number_items = (int)(bssid_list->NumberOfItems);

    wpa_printf(MSG_MSGDUMP, "Received %d bytes of scan results (%d BSSes)",
           number_items * sizeof(OS_802_11_BSSID_EX), number_items);

    os_memset( results, 0, max_size * sizeof(struct wpa_scan_result) );
    number_items = MIN( number_items, max_size );

    for(index=0; index < number_items; index++) {
        os_memcpy( results[index].bssid, &pBssid->MacAddress, ETH_ALEN );
        os_memcpy( results[index].ssid, pBssid->Ssid.Ssid, pBssid->Ssid.SsidLength );
        results[index].ssid_len = pBssid->Ssid.SsidLength;
#ifdef STA_DK_VER_5_0_0_94
        results[index].freq = pBssid->Configuration.channel / 1000;
        results[index].caps = pBssid->Capabilities;
#else
        results[index].freq = pBssid->Configuration.Union.channel / 1000;
        results[index].caps = pBssid->Union.Capabilities;
#endif
        results[index].level = pBssid->Rssi;
        results[index].maxrate = 0; /* In units of 0.5 Mb/s */
        for (i = 0; i < sizeof(pBssid->SupportedRates); i++) {
            if (pBssid->SupportedRates[i] > (unsigned)results[index].maxrate) {
                results[index].maxrate = pBssid->SupportedRates[i];
            }
        }
        wpa_printf(MSG_DEBUG,"TI: Net: %s Cap: 0x%04x Priv: 0x%x NetType: 0x%x InfraMode: 0x%x IELen: %d",
#ifdef STA_DK_VER_5_0_0_94
                   pBssid->Ssid.Ssid, pBssid->Capabilities, pBssid->Privacy, pBssid->NetworkTypeInUse,
#else
                   pBssid->Ssid.Ssid, pBssid->Union.Capabilities, pBssid->Privacy, pBssid->NetworkTypeInUse,
#endif
                   pBssid->InfrastructureMode, pBssid->IELength );

        /* Fixed IEs from site entry - same Capabilities */
        pFixedIes = (OS_802_11_FIXED_IEs *)&pBssid->IEs[0];
        wpa_printf(MSG_DEBUG,"TI: Fixed IEs: Beacon: 0x%x Cap: 0x%x", pFixedIes->BeaconInterval, pFixedIes->Capabilities);
        for(i=sizeof(OS_802_11_FIXED_IEs); i < pBssid->IELength;) {
            pVarIes = (OS_802_11_VARIABLE_IEs *)&pBssid->IEs[i];
            wpa_printf(MSG_DEBUG,"TI: Variable IEs: ID: 0x%x Len: %d", pVarIes->ElementID, pVarIes->Length);
            wpa_hexdump(MSG_DEBUG,"TI: oui:", pVarIes->data, pVarIes->Length);
            switch( pVarIes->ElementID ) {
                case GENERIC_INFO_ELEM: /* 0xdd */
                    if( (pVarIes->Length > 3) && (os_memcmp(pVarIes->data, WPA_OUI, 4) == 0) ) {
                        results[index].wpa_ie_len = MIN((pVarIes->Length + 2), SSID_MAX_WPA_IE_LEN);
                        os_memcpy( results[index].wpa_ie, pVarIes, results[index].wpa_ie_len );
                    }
                    break;

                case RSN_INFO_ELEM:     /* 0x30 */
                    results[index].rsn_ie_len = MIN((pVarIes->Length + 2), SSID_MAX_WPA_IE_LEN);
                    os_memcpy( results[index].rsn_ie, pVarIes, results[index].rsn_ie_len );
                    break;
            }
            i += (pVarIes->Length + (2 * sizeof(tiUINT8)));
        }
        pBssid = (OS_802_11_BSSID_EX *)(((u8 *)pBssid) + pBssid->Length);
    }
    /* Merge new results with previous */
    number_items = scan_merge( myDrv, results, number_items, max_size );

    qsort( results, number_items, sizeof(struct wpa_scan_result),
           wpa_driver_tista_scan_result_compare );

    os_free( bssid_list );
    return( number_items );
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_get_scan_results
Routine Description: retrieve driver scan results
Arguments:
   sock - socket
   priv - pointer to private data structure
   sock_ctx - pointer to other private data
Return Value: None
-----------------------------------------------------------------------------*/
static void wpa_driver_tista_receive_driver_event( int sock, void *priv, void *sock_ctx )
{
    IPC_EV_DATA myBuf;
    UINT8 *buf;
    UINT32 *bufLong;
    union wpa_event_data myEventData;
    struct wpa_driver_ti_data *mySuppl;
    OS_802_11_ASSOCIATION_INFORMATION *pInfo = NULL;
    OS_802_11_AUTHENTICATION_REQUEST *pMediaSpecificBuf;
    OS_802_11_BSSID_EX *pSelectedBssidInfo = NULL;
    OS_802_11_NETWORK_MODE myBssType;
    IPC_EV_DATA *pData = &myBuf;
    int res;
#ifndef STA_DK_VER_5_0_0_94
    btCoexStatus_t *btCoexStatus;
#endif

    wpa_printf(MSG_DEBUG,"wpa_driver_tista_receive_driver_event...");

    res = recv( sock, &myBuf, sizeof(myBuf), 0 );
    if( res < 0 ) {
        wpa_printf(MSG_ERROR,"l2_packet_receive - recvfrom");
        return;
    }

    mySuppl = pData->EvParams.hUserParam;

    switch( ((IPC_EVENT_PARAMS*)pData)->uEventType ) {
        case IPC_EVENT_ASSOCIATED:
            /* Associated event is called after successfull ASSOC_RSP packet is received */
            wpa_printf(MSG_DEBUG,"wpa_supplicant - Associated");

            TI_GetBSSType( mySuppl->hDriver, &myBssType );
            wpa_printf(MSG_DEBUG,"myBssType = %d",myBssType);

            if( myBssType == os802_11Infrastructure ) {
                /* Get ASSOC_REQ and ASSOC_RSP IE */
                res = TI_GetAssociationInfo( mySuppl->hDriver, &pInfo );
                buf = (UINT8 *)pInfo;

                if( buf != NULL ) {
                    myEventData.assoc_info.req_ies = buf + pInfo->OffsetRequestIEs;
                    myEventData.assoc_info.req_ies_len = pInfo->RequestIELength;
                    myEventData.assoc_info.resp_ies = buf + pInfo->OffsetResponseIEs;
                    myEventData.assoc_info.resp_ies_len = pInfo->ResponseIELength;
                    myEventData.assoc_info.beacon_ies = NULL;
                    myEventData.assoc_info.beacon_ies_len = 0;

                    /* Get AP Beacon IEs - especially WPA/RSN IE */
                    pSelectedBssidInfo = wpa_driver_tista_get_bssid_info( mySuppl->hDriver );
                    if( pSelectedBssidInfo ) {
                        if( pSelectedBssidInfo->IELength && pSelectedBssidInfo->IEs ) { // Dm: Fixed IEs ???
                            myEventData.assoc_info.beacon_ies = (UINT8 *)pSelectedBssidInfo->IEs + sizeof(OS_802_11_FIXED_IEs);
                            myEventData.assoc_info.beacon_ies_len = pSelectedBssidInfo->IELength - sizeof(OS_802_11_FIXED_IEs);
                        }
                    }
                    wpa_printf(MSG_DEBUG,"myEventData.assoc_info.req_ies = 0x%x",(unsigned)myEventData.assoc_info.req_ies);
                    wpa_printf(MSG_DEBUG,"myEventData.assoc_info.req_ies_len = %d",myEventData.assoc_info.req_ies_len);
                    wpa_printf(MSG_DEBUG,"myEventData.assoc_info.resp_ies = 0x%x",(unsigned)myEventData.assoc_info.resp_ies);
                    wpa_printf(MSG_DEBUG,"myEventData.assoc_info.resp_ies_len = %d",myEventData.assoc_info.resp_ies_len);
                    wpa_printf(MSG_DEBUG,"myEventData.assoc_info.beacon_ies = 0x%x",(unsigned)myEventData.assoc_info.beacon_ies);
                    wpa_printf(MSG_DEBUG,"myEventData.assoc_info.beacon_ies_len = %d",myEventData.assoc_info.beacon_ies_len);
                    wpa_hexdump(MSG_DEBUG, "WPA: beacon_ies", myEventData.assoc_info.beacon_ies, myEventData.assoc_info.beacon_ies_len);

                    /* First we notify wpa_supplicant and give it all the above IEs */
                    wpa_supplicant_event( mySuppl->hWpaSupplicant, EVENT_ASSOCINFO, &myEventData );

                    /* Since both ASSOC_REQ/RSP and beacon IEs are allocated dynamically by the Utility Adapter - we need to free the buffers */
                    os_free( pInfo );
                    if( pSelectedBssidInfo ) {
                        os_free( pSelectedBssidInfo );
                    }
                }
            }
            else {
                myEventData.assoc_info.req_ies = NULL;
                myEventData.assoc_info.req_ies_len = 0;
                myEventData.assoc_info.resp_ies = NULL;
                myEventData.assoc_info.resp_ies_len = 0;
                myEventData.assoc_info.beacon_ies = NULL;
                myEventData.assoc_info.beacon_ies_len = 0;
                wpa_supplicant_event( mySuppl->hWpaSupplicant, EVENT_ASSOCINFO, &myEventData );
            }
            /* We now can notify wpa_supplicant of the association event so it could start key negotiation (if needed) */
            wpa_supplicant_event( mySuppl->hWpaSupplicant, EVENT_ASSOC, NULL );
            /* Allow Disassociation */
            mySuppl->block_disassoc_events = NO_BLOCK;
            break;

        case IPC_EVENT_DISASSOCIATED:
            if( mySuppl->block_disassoc_events != BLOCK_DISASSOC ) {
               wpa_printf(MSG_DEBUG,"wpa_supplicant - Disassociated");
               wpa_supplicant_event( mySuppl->hWpaSupplicant, EVENT_DISASSOC, NULL );
            }
            else {
               wpa_printf(MSG_INFO,"wpa_supplicant - Disassociated (blocked)");
            }
            break;

        case IPC_EVENT_SCAN_COMPLETE:
            wpa_printf(MSG_DEBUG,"wpa_supplicant - IPC_EVENT_SCAN_COMPLETE");
            wpa_supplicant_event( mySuppl->hWpaSupplicant, EVENT_SCAN_RESULTS, NULL );
            break;

        case IPC_EVENT_AUTH_SUCC:
            wpa_printf(MSG_INFO,"wpa_supplicant - IPC_EVENT_AUTH_SUCC");
            break;

        case IPC_EVENT_EAPOL:
            wpa_printf(MSG_DEBUG,"wpa_supplicant - EAPOL");
            buf = pData->uBuffer;
            wpa_supplicant_rx_eapol( mySuppl->hWpaSupplicant, (UINT8 *)(buf + MAC_ADDR_LEN),
                                     (UINT8 *)(buf + ETHERNET_HDR_LEN), (pData->uBufferSize - ETHERNET_HDR_LEN) );
            break;

        case IPC_EVENT_MEDIA_SPECIFIC:
            wpa_printf(MSG_DEBUG,"wpa_supplicant - Media_Specific");
            bufLong = (UINT32 *)pData->uBuffer;
            /* Check for Authentication type messages from driver */
            if( (*bufLong) == os802_11StatusType_Authentication ) {
                pMediaSpecificBuf = (OS_802_11_AUTHENTICATION_REQUEST *)(bufLong + 1);
                wpa_printf(MSG_DEBUG,"wpa_supplicant - Media_Specific - Authentication message detected: %u", pMediaSpecificBuf->Flags);
                /* Check for MIC failure event */
                if( (pMediaSpecificBuf->Flags == OS_802_11_REQUEST_PAIRWISE_ERROR) || (pMediaSpecificBuf->Flags == OS_802_11_REQUEST_GROUP_ERROR)) {
                    /* Notify wpa_supplicant of MIC failure */
                    myEventData.michael_mic_failure.unicast = 1;
                    wpa_supplicant_event( mySuppl->hWpaSupplicant, EVENT_MICHAEL_MIC_FAILURE, &myEventData );
                }
                /* OS_802_11_REQUEST_REAUTH - is handled automatically */
                /* OS_802_11_REQUEST_KEYUPDATE - no event of this type */
            }
            break;

        case IPC_EVENT_LINK_SPEED:
            bufLong = (UINT32 *)pData->uBuffer;
            wpa_printf(MSG_DEBUG,"wpa_supplicant - Link Speed = %u MB/s", ((*bufLong) / 2));
            /* wpa_msg(mySuppl->hWpaSupplicant, MSG_INFO, WPA_EVENT_LINK_SPEED "%u MB/s", ((*bufLong) / 2)); */
            mySuppl->link_speed = (unsigned)((*bufLong) / 2);
            break;

        case IPC_EVENT_WPA2_PREAUTHENTICATION:
            wpa_printf(MSG_DEBUG,"wpa_supplicant - WPA2_PREAUTHENTICATION");
            bufLong = (UINT32 *)pData->uBuffer;
            wpa_printf(MSG_DEBUG,"Preauth Status Code = %u",*bufLong);
            /* Dm: wpa_supplicant_event( mySuppl->hWpaSupplicant, EVENT_PMKID_CANDIDATE, &data); */
            break;

#ifndef STA_DK_VER_5_0_0_94
        case IPC_EVENT_BT_COEX_MODE:
            btCoexStatus = (btCoexStatus_t *)pData->uBuffer;
            if( (btCoexStatus != NULL) && btCoexStatus->state ) {
                wpa_printf(MSG_DEBUG,"wpa_supplicant - BT_COEX_MODE (SG is ON, minTxRate = %d)\n", btCoexStatus->minTxRate);
            }
            else {
                wpa_printf(MSG_DEBUG,"wpa_supplicant - BT_COEX_MODE (SG is OFF)\n");
            }
            break;
#endif
        case IPC_EVENT_LOW_SNR:
        case IPC_EVENT_LOW_RSSI:
            break;

        default:
            wpa_printf(MSG_ERROR,"wpa_supplicant - Unhandled driver event: %d", ((IPC_EVENT_PARAMS*)pData)->uEventType);
            break;
    }
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_init
Routine Description: init driver interface
Arguments:
   priv - pointer to private data structure
   ifname - return interface name
Return Value: pointer to driver structure (to be supplied later by supplicant to wrappers)
-----------------------------------------------------------------------------*/
static void *wpa_driver_tista_init( void *priv, const char *ifname )
{
    struct wpa_driver_ti_data *myDrv;
    int status;
    UINT32 driverStatus,res;
    struct sockaddr_in echoserver;
    OS_802_11_MAC_ADDRESS myMac;
#ifdef CONFIG_TI_LOCKFILE
    char str[10];
#endif

    wpa_printf(MSG_DEBUG,"Initializing STA-DK %s interface...",SW_VERSION_STR);
#ifdef CONFIG_TI_LOCKFILE
    /* Try to open the lock file */
    lfp = open("./.wpa_supplicant_lockfile.pid", (O_RDWR | O_CREAT | O_EXCL), 0640);
    if( lfp < 0 ) {
        wpa_printf(MSG_ERROR,"Cannot open pid-file...Aborting...\n");
        return( NULL );
    }
    /* Try to get a file lock on the pid-file. If another instance is running and already has a lock - we will fail */
#ifndef ANDROID /* Dm: !!! lockf is not implemented in Android */
    if( lockf(lfp,F_TLOCK,0) < 0 ) {
        wpa_printf(MSG_ERROR,"Another instance of wpa_supplicant is running...Aborting...\n");
        return( NULL );
    }
#endif
    /* If we got here - it means that no other instance is running - write process id to pid_file */
    sprintf(str, "%d\n", getpid());
    write(lfp,str,os_strlen(str)); /* record pid to lockfile */
#endif

    /* Allocate internal data structure to manage driver */
    myDrv = os_malloc( sizeof(struct wpa_driver_ti_data) );

    /* If failed to allocate */
    if( myDrv == NULL ) {
        wpa_printf(MSG_ERROR,"Failed to allocate memory for control structure...Aborting...");
        goto label_init_error_file;
    }

    /* Zero memory */
    os_memset( myDrv, 0, sizeof(struct wpa_driver_ti_data) );

    /* Initialize Utility Adapter module */
    myDrv->hDriver = TI_AdapterInit( TIWLAN_DRV_NAME );

    /* If couldn't initialize - return NULL to indicate error */
    if( myDrv->hDriver == 0 ) {
        wpa_printf(MSG_ERROR,"Error: failed to initialize Utility Adapter interface...");
        goto label_init_error_free;
    }

    /* Get WLAN interface mac address through Utility Adapter */
    res = TI_GetCurrentAddress( myDrv->hDriver, &myMac );

    if( res == (UINT32)-1 ) {
        wpa_printf(MSG_ERROR,"Error: failed to initialize Utility Adapter interface...");
        goto label_init_error_free;
    }

    os_memcpy( &myDrv->own_addr, &myMac, MAC_ADDR_LEN );

    wpa_driver_tista_register_events( myDrv );

    /* Block disassoc events until connected */
    if( myDrv->block_disassoc_events == NO_BLOCK )
        myDrv->block_disassoc_events = NO_BLOCK_DISASSOC_IN_PROGRESS;

    /* Store wpa_supplicant context */
    myDrv->hWpaSupplicant = priv;

    myDrv->driverEventsSocket = socket( PF_INET, SOCK_DGRAM, IPPROTO_UDP );

    if( myDrv->driverEventsSocket < 0 ) {
        wpa_printf(MSG_ERROR,"Error: failed to create driver events socket... (%s)", strerror(errno));
        goto label_init_error_free;
    }

    os_memset( &echoserver, 0, sizeof(echoserver) );  /* Clear struct */
    echoserver.sin_family = AF_INET;                  /* Internet/IP */
    echoserver.sin_addr.s_addr = htonl(INADDR_ANY);   /* IP address */
    echoserver.sin_port = htons(TI_DRIVER_MSG_PORT);  /* server port */

    if( bind(myDrv->driverEventsSocket, (struct sockaddr *) &echoserver, sizeof(echoserver)) < 0 ) {
        wpa_printf(MSG_ERROR,"Error: failed to create driver events socket... (%s)", strerror(errno));
        close(myDrv->driverEventsSocket);
        goto label_init_error_free;
    }

    status = eloop_register_read_sock( myDrv->driverEventsSocket, wpa_driver_tista_receive_driver_event, priv, myDrv );

    if( status != 0 ) {
        wpa_printf(MSG_ERROR,"Error: failed to register socket handler...");
    }

    wpa_printf(MSG_DEBUG,"driver events socket is 0x%x...",myDrv->driverEventsSocket);

    /* Signal that driver is not loaded yet */
    myDrv->driver_is_loaded = TRUE;

    /* Set default scan type */
    myDrv->scan_type = SCAN_TYPE_NORMAL_ACTIVE;
    scan_init( myDrv );

    /* Set default amount of channels */
    myDrv->scan_channels = check_and_get_carrier_channels();

    /* Link Speed will be set by the message from the driver */
    myDrv->link_speed = 0;

    /* BtCoex mode is read from tiwlan.ini file */
    myDrv->btcoex_mode = 1; /* SG_DISABLE */
    myDrv->btcoex_scan = FALSE;

    /* RTS Threshold is read from tiwlan.ini file */
    myDrv->rts_threshold = HAL_CTRL_RTS_THRESHOLD_MAX;

    /* Return pointer to our driver structure */
    return( myDrv );

label_init_error_free:
    os_free( myDrv );
label_init_error_file:
#ifdef CONFIG_TI_LOCKFILE
    /* Close and delete the pid-lock-file */
    close(lfp);
    unlink("./wpa_supplicant_lockfile.pid");
#endif
    return( NULL );
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_unload
Routine Description: unload driver
Arguments:
   priv - pointer to private data structure
Return Value: None
-----------------------------------------------------------------------------*/
static void wpa_driver_tista_unload( void *priv )
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;

    wpa_printf(MSG_DEBUG,"wpa_driver_tista_unload called");
    /* Unregister driver events */
    wpa_driver_tista_unregister_events( priv );
    /* Close connection socket */
    close(myDrv->driverEventsSocket);
    /* Unload Utility Adapter */
    TI_AdapterDeinit( myDrv->hDriver );
    /* Free all allocated memory */
    scan_exit( myDrv );
    os_free( myDrv );
#ifdef CONFIG_TI_LOCKFILE
    /* Close and delete the pid-lock-file */
    close(lfp);
    unlink("./wpa_supplicant_lockfile.pid");
#endif
    wpa_printf(MSG_DEBUG,"STA-DK %s interface Deinitialized...bye!", SW_VERSION_STR);
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_get_mac_addr
Routine Description: return WLAN MAC address
Arguments:
   priv - pointer to private data structure
Return Value: pointer to BSSID
-----------------------------------------------------------------------------*/
const u8 *wpa_driver_tista_get_mac_addr( void *priv )
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;

    wpa_printf(MSG_DEBUG,"wpa_driver_tista_get_mac_addr called");
    /* If driver is not initialized yet - we cannot access it so return */
    TI_CHECK_DRIVER( myDrv->driver_is_loaded, NULL );
    return( (const u8 *)&myDrv->own_addr );
}

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_send_eapol
Routine Description: transmit EAPOL
Arguments:
   priv - pointer to private data structure
   data - pointer to EAPOL data
   data_len - length of EAPOL data
Return Value: None
-----------------------------------------------------------------------------*/
static int ti_send_eapol( void *priv, const u8 *dest, u16 proto,
                          const u8 *data, size_t data_len )
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;
    u8* DataWithHeader = NULL;
    u16 protoNetwork;
    int ret = 0;

    wpa_printf(MSG_DEBUG,"TI send_eapol called");
#ifdef IEEE8021X_EAPOL
    DataWithHeader = os_malloc(data_len + ETHERNET_HDR_LEN); /* 14 bytes */
    if( DataWithHeader == NULL ) {
        wpa_printf(MSG_ERROR,"TI send_eapol failed to alloc full buffer");
        return( -1 );
    }

    os_memcpy(DataWithHeader, dest, MAC_ADDR_LEN); /* 6 bytes */
    os_memcpy(DataWithHeader+MAC_ADDR_LEN, myDrv->own_addr, MAC_ADDR_LEN);
    protoNetwork = htons(proto);
    os_memcpy(DataWithHeader+(MAC_ADDR_LEN<<1), &protoNetwork, sizeof(u16)); /* 2 bytes */

    os_memcpy(DataWithHeader+ETHERNET_HDR_LEN, data, data_len);
    data_len += ETHERNET_HDR_LEN;

    wpa_hexdump(MSG_DEBUG, "WPA: FULL TX EAPOL-Key", DataWithHeader, data_len);

    /* Transmit EAPOL packet */
    ret = TI_Send_EAPOL_Packet( myDrv->hDriver, (void *)DataWithHeader, data_len );
    os_free(DataWithHeader);
#endif
    return( TI2WPA_STATUS(ret) );
}

#ifndef STA_DK_VER_5_0_0_94
/*-----------------------------------------------------------------------------
Routine Name: prepare_filter_struct
Routine Description: fills rx data filter structure according to parameter type
Arguments:
   priv - pointer to private data structure
   type - type of mac address
   dfreq_ptr - pointer to TIWLAN_DATA_FILTER_REQUEST structure
Return Value: 0 - success, -1 - error
-----------------------------------------------------------------------------*/
static int prepare_filter_struct( void *priv, int type,
                                  TIWLAN_DATA_FILTER_REQUEST *dfreq_ptr )
{
    const u8 *macaddr;
    size_t len = 0;
    u8 mask;
    int ret = -1;

    wpa_printf(MSG_ERROR, "%s: type=%d", __func__, type);
    switch (type ) {
      case RX_SELF_FILTER:
        macaddr = wpa_driver_tista_get_mac_addr(priv);
        len = MAC_ADDR_LEN;
        mask = 0x3F; /* 6 bytes */
        break;
      case RX_BROADCAST_FILTER:
        macaddr = (const u8 *)"\xFF\xFF\xFF\xFF\xFF\xFF";
        len = MAC_ADDR_LEN;
        mask = 0x3F; /* 6 bytes */
        break;
      case RX_IPV4_MULTICAST_FILTER:
        macaddr = (const u8 *)"\x01\x00\x5E";
        len = 3;
        mask = 0x7; /* 3 bytes */
        break;
      case RX_IPV6_MULTICAST_FILTER:
        macaddr = (const u8 *)"\x33\x33";
        len = 2;
        mask = 0x3; /* 2 bytes */
        break;
    }
    if (len && macaddr) {
        dfreq_ptr->Offset = 0;
        dfreq_ptr->MaskLength = 1;
        dfreq_ptr->Mask[0] = mask;
        dfreq_ptr->PatternLength = len;
        os_memcpy( dfreq_ptr->Pattern, macaddr, len );
        ret = 0;
    }
    return( ret );
}
#endif

/*-----------------------------------------------------------------------------
Routine Name: wpa_driver_tista_driver_cmd
Routine Description: executes driver-specific commands
Arguments:
   priv - pointer to private data structure
   cmd - command
   buf - return buffer
   buf_len - buffer length
Return Value: actual buffer length - success, -1 - failure
-----------------------------------------------------------------------------*/
int wpa_driver_tista_driver_cmd( void *priv, char *cmd, char *buf, size_t buf_len )
{
    struct wpa_driver_ti_data *myDrv = (struct wpa_driver_ti_data *)priv;
    int ret = -1, prev_events;

    wpa_printf(MSG_DEBUG, "%s %s", __func__, cmd);

    if( os_strcasecmp(cmd, "start") == 0 ) {
        wpa_printf(MSG_DEBUG,"Start command");
        prev_events = myDrv->block_disassoc_events;
        myDrv->block_disassoc_events = myDrv->block_disassoc_prev;
        ret = TI_Start( myDrv->hDriver );
        if( ret == OK ) {
            /* Signal that driver is not loaded yet */
            myDrv->driver_is_loaded = TRUE;
            myDrv->scan_channels = check_and_get_carrier_channels();
            wpa_msg(myDrv->hWpaSupplicant, MSG_INFO, WPA_EVENT_DRIVER_STATE "STARTED");
        }
        else
            myDrv->block_disassoc_events = prev_events;
        return( TI2WPA_STATUS(ret) );
    }

    /* If driver is not initialized yet - we cannot access it so return */
    TI_CHECK_DRIVER( myDrv->driver_is_loaded, -1 );

    if( os_strcasecmp(cmd, "stop") == 0 ) {
        wpa_printf(MSG_DEBUG,"Stop command");
        myDrv->block_disassoc_prev = myDrv->block_disassoc_events;
        myDrv->block_disassoc_events = BLOCK_DISASSOC; /* Block message */
        ret = TI_Stop( myDrv->hDriver );
        if( ret == OK ) {
            /* Signal that driver is not loaded yet */
            myDrv->driver_is_loaded = FALSE;
            wpa_msg(myDrv->hWpaSupplicant, MSG_INFO, WPA_EVENT_DRIVER_STATE "STOPPED");
        }
        else
            myDrv->block_disassoc_events = myDrv->block_disassoc_prev;
    }
    else if( os_strcasecmp(cmd, "macaddr") == 0 ) {
        u8 *macaddr = (u8 *)wpa_driver_tista_get_mac_addr(priv);
        wpa_printf(MSG_DEBUG,"Macaddr command");
        wpa_printf(MSG_DEBUG, "   Macaddr = " MACSTR, MAC2STR(macaddr));
        ret = snprintf(buf, buf_len, "Macaddr = " MACSTR "\n", MAC2STR(macaddr));
        if (ret < (int)buf_len) {
            return( ret );
        }
    }
    else if( os_strcasecmp(cmd, "scan-passive") == 0 ) {
        wpa_printf(MSG_DEBUG,"Scan Passive command");
        myDrv->scan_type = SCAN_TYPE_NORMAL_PASSIVE;
        ret = 0;
    }
    else if( os_strcasecmp(cmd, "scan-active") == 0 ) {
        wpa_printf(MSG_DEBUG,"Scan Active command");
        myDrv->scan_type = SCAN_TYPE_NORMAL_ACTIVE;
        ret = 0;
    }
    else if( os_strcasecmp(cmd, "scan-mode") == 0 ) {
        wpa_printf(MSG_DEBUG,"Scan Mode command");
        ret = snprintf(buf, buf_len, "ScanMode = %u\n", myDrv->scan_type);
        if (ret < (int)buf_len) {
            return( ret );
        }
    }
    else if( os_strcasecmp(cmd, "linkspeed") == 0 ) {
        wpa_printf(MSG_DEBUG,"Link Speed command");
        ret = snprintf(buf, buf_len, "LinkSpeed %u\n", myDrv->link_speed);
        if (ret < (int)buf_len) {
            return( ret );
        }
    }
    else if( os_strncasecmp(cmd, "scan-channels", 13) == 0 ) {
        int noOfChan;
        char *cp = cmd + 13;
        char *endp;

        if (*cp != '\0') {
            noOfChan = strtol(cp, &endp, 0);
            if (endp != cp) {
                wpa_printf(MSG_DEBUG,"Scan Channels command = %d", noOfChan);
                if( (noOfChan > 0) && (noOfChan <= MAX_NUMBER_OF_CHANNELS_PER_SCAN) ) {
                    myDrv->scan_channels = noOfChan;
                    ret = 0;
                }
            }
        } else {
            ret = snprintf(buf, buf_len, "Scan-Channels = %d\n", myDrv->scan_channels);
            if (ret < (int)buf_len) {
                return( ret );
            }
        }
    }
    else if( os_strcasecmp(cmd, "rssi") == 0 ) {
#if 1
        u8 ssid[MAX_SSID_LEN];
        int rssi, len;

        wpa_printf(MSG_DEBUG,"rssi command");

        ret = TI_GetRSSI( myDrv->hDriver, &rssi );
        if( ret == OK ) {
            len = wpa_driver_tista_get_ssid( priv, (u8 *)ssid );
            if( (len > 0) && (len <= MAX_SSID_LEN) && (len < (int)buf_len)) {
                os_memcpy( (void *)buf, (void *)ssid, len );
                ret = len;
                ret += snprintf(&buf[ret], buf_len-len, " rssi %d\n", rssi);
                if (ret < (int)buf_len) {
                    return( ret );
                }
            }
        }
#else
        OS_802_11_BSSID_EX bssidInfo;

        wpa_printf(MSG_DEBUG,"rssi command");

        ret = TI_GetSelectedBSSIDInfo( myDrv->hDriver, (OS_802_11_BSSID_EX *)&bssidInfo );
        if( ret == OK ) {
            if( bssidInfo.Ssid.SsidLength != 0 && bssidInfo.Ssid.SsidLength < buf_len) {
                os_memcpy( (void *)buf, (void *)(bssidInfo.Ssid.Ssid), bssidInfo.Ssid.SsidLength );
                ret = bssidInfo.Ssid.SsidLength;
                ret += snprintf(&buf[ret], buf_len-ret, " rssi %d\n", bssidInfo.Rssi);
                if (ret < (int)buf_len) {
                    return( ret );
                }
            }
            ret = -1;
        }
#endif
    }
    else if( os_strncasecmp(cmd, "powermode", 9) == 0 ) {
        u32 rtsThreshold = myDrv->rts_threshold;
        u32 mode;
        char *cp = cmd + 9;
        char *endp;

        if (*cp != '\0') {
            mode = (u32)strtol(cp, &endp, 0);
            if (endp != cp) {
                wpa_printf(MSG_DEBUG,"Power Mode command = %u", mode);
                if( mode <= OS_POWER_MODE_LONG_DOZE )
                    ret = TI_ConfigPowerManagement( myDrv->hDriver, mode );
                if( mode == OS_POWER_MODE_ACTIVE )
                    rtsThreshold = 0;
                if( TI_SetRTSThreshold( myDrv->hDriver, rtsThreshold ) != OK )
                    wpa_printf(MSG_DEBUG,"Set RTS threshold = %u failed", rtsThreshold);
            }
        }
    }
    else if( os_strncasecmp(cmd, "getpower", 8) == 0 ) {
        u32 mode;

        ret = TI_GetPowerMode( myDrv->hDriver, (OS_802_11_POWER_PROFILE *)&mode);
        if( ret == OK ) {
            ret = snprintf(buf, buf_len, "powermode = %u\n", mode);
            if (ret < (int)buf_len) {
                return( ret );
            }
        }
    }
    else if( os_strncasecmp(cmd, "get-rts-threshold", 17) == 0 ) {
        tiUINT32 rtsThreshold = 0;

        ret = TI_GetRTSThreshold( myDrv->hDriver, &rtsThreshold );
        wpa_printf(MSG_DEBUG,"Get RTS Threshold command = %d", rtsThreshold);
        if( ret == OK ) {
            ret = snprintf(buf, buf_len, "rts-threshold = %u\n", rtsThreshold);
            if (ret < (int)buf_len) {
                return( ret );
            }
        }
    }
    else if( os_strncasecmp(cmd, "set-rts-threshold", 17) == 0 ) {
        tiUINT32 rtsThreshold = 0;
        char *cp = cmd + 17;
        char *endp;

        if (*cp != '\0') {
            rtsThreshold = (tiUINT32)strtol(cp, &endp, 0);
            if (endp != cp) {
                wpa_printf(MSG_DEBUG,"RTS Threshold command = %d", rtsThreshold);
                if( rtsThreshold <= HAL_CTRL_RTS_THRESHOLD_MAX ) {
                    ret = TI_SetRTSThreshold( myDrv->hDriver, rtsThreshold );
                    if( ret == OK ) {
                        myDrv->rts_threshold = rtsThreshold;
                    }
                }
            }
        }
    }
    else if( os_strcasecmp(cmd, "btcoexscan-start") == 0 ) {
        wpa_printf(MSG_DEBUG,"BT Coex Scan Start command");
        myDrv->btcoex_scan = TRUE;
        ret = 0;
    }
    else if( os_strcasecmp(cmd, "btcoexscan-stop") == 0 ) {
        wpa_printf(MSG_DEBUG,"BT Coex Scan Stop command");
        myDrv->btcoex_scan = FALSE;
        ret = 0;
    }
#ifndef STA_DK_VER_5_0_0_94
    else if( os_strcasecmp(cmd, "rxfilter-start") == 0 ) {
        wpa_printf(MSG_DEBUG,"Rx Data Filter Start command");
        ret = TI_EnableDisableRxDataFilters( myDrv->hDriver, TRUE );
    }
    else if( os_strcasecmp(cmd, "rxfilter-stop") == 0 ) {
        wpa_printf(MSG_DEBUG,"Rx Data Filter Stop command");
        ret = TI_EnableDisableRxDataFilters( myDrv->hDriver, FALSE );
    }
    else if( os_strcasecmp(cmd, "rxfilter-statistics") == 0 ) {
        TIWLAN_DATA_FILTER_STATISTICS stats;
        int len, i;

        wpa_printf(MSG_DEBUG,"Rx Data Filter Statistics command");
        ret = TI_GetRxDataFiltersStatistics( myDrv->hDriver, &stats );
        if( ret == OK ) {
            ret = snprintf(buf, buf_len, "RxFilterStat: %u",
                           stats.UnmatchedPacketsCount);
            for(i=0;( i < MAX_NUM_DATA_FILTERS );i++) {
                ret += snprintf(&buf[ret], buf_len-ret, " %u",
                           stats.MatchedPacketsCount[i]);
            }
            ret += snprintf(&buf[ret], buf_len-ret, "\n");
            if (ret < (int)buf_len) {
                return( ret );
            }
        }
    }
    else if( os_strncasecmp(cmd, "rxfilter-add", 12) == 0 ) {
        TIWLAN_DATA_FILTER_REQUEST dfreq;
        char *cp = cmd + 12;
        char *endp;
        int type;

        if (*cp != '\0') {
            type = (int)strtol(cp, &endp, 0);
            if (endp != cp) {
                wpa_printf(MSG_DEBUG,"Rx Data Filter Add [%d] command", type);
                ret = prepare_filter_struct( priv, type, &dfreq );
                if( ret == 0 ) {
                    ret = TI_AddRxDataFilter( myDrv->hDriver, &dfreq );
                }
            }
        }
    }
    else if( os_strncasecmp(cmd, "rxfilter-remove",15) == 0 ) {
        wpa_printf(MSG_DEBUG,"Rx Data Filter Remove command");
        TIWLAN_DATA_FILTER_REQUEST dfreq;
        char *cp = cmd + 15;
        char *endp;
        int type;

        if (*cp != '\0') {
            type = (int)strtol(cp, &endp, 0);
            if (endp != cp) {
                wpa_printf(MSG_DEBUG,"Rx Data Filter Remove [%d] command", type);
                ret = prepare_filter_struct( priv, type, &dfreq );
                if( ret == 0 ) {
                    ret = TI_RemoveRxDataFilter( myDrv->hDriver, &dfreq );
                }
            }
        }
    }
    else if( os_strcasecmp(cmd, "snr") == 0 ) {
        u32 snr;

        ret = TI_GetSNR( myDrv->hDriver, &snr );
        if( ret == OK ) {
            ret = snprintf(buf, buf_len, "snr = %u\n", snr);
            if (ret < (int)buf_len) {
                return( ret );
            }
        }
    }
    else if( os_strncasecmp(cmd, "btcoexmode", 10) == 0 ) {
        u32 mode;
        char *cp = cmd + 10;
        char *endp;

        if (*cp != '\0') {
            mode = (u32)strtol(cp, &endp, 0);
            if (endp != cp) {
                wpa_printf(MSG_DEBUG,"BtCoex Mode command = %u", mode);
                ret = TI_SetBtCoeEnable( myDrv->hDriver, mode );
                if( ret == OK ) {
                    myDrv->btcoex_mode = mode;
                }
            }
        }
    }
    else if( os_strcasecmp(cmd, "btcoexstat") == 0 ) {
        u32 status = myDrv->btcoex_mode;

        wpa_printf(MSG_DEBUG,"BtCoex Status");
        ret = TI_SetBtCoeGetStatus( myDrv->hDriver, (tiUINT32 *)&status );
        if( ret == OK ) {
            ret = snprintf(buf, buf_len, "btcoexstatus = 0x%x\n", status);
            if (ret < (int)buf_len) {
                return( ret );
            }
        }
    }
#endif
    else {
        wpa_printf(MSG_DEBUG,"Unsupported command");
    }
    return( TI2WPA_STATUS(ret) );
}

/* Fill driver_ops structure to provide wpa_supplicant core with wrapper routines */
struct wpa_driver_ops wpa_driver_custom_ops = {
    .name           = TIWLAN_DRV_NAME,
    .desc           = "TI Station Driver",
    .init           = wpa_driver_tista_init,
    .deinit         = wpa_driver_tista_unload,
    .get_bssid      = wpa_driver_tista_get_bssid,
    .get_ssid       = wpa_driver_tista_get_ssid,
    .set_wpa        = wpa_driver_tista_set_wpa,
    .set_key        = wpa_driver_tista_set_key,
    .set_param      = NULL,
    .set_countermeasures    = wpa_driver_tista_set_countermeasures,
    .set_drop_unencrypted   = wpa_driver_tista_set_drop_unencrypted,
    .scan                   = wpa_driver_tista_scan,
    .get_scan_results       = wpa_driver_tista_get_scan_results,
    .deauthenticate = wpa_driver_tista_deauthenticate,
    .disassociate   = wpa_driver_tista_disassociate,
    .associate      = wpa_driver_tista_associate,
    .set_auth_alg   = wpa_driver_tista_set_auth_alg,
    .get_mac_addr   = wpa_driver_tista_get_mac_addr,
    .send_eapol     = ti_send_eapol,
    .add_pmkid      = NULL,
    .remove_pmkid   = NULL,
    .flush_pmkid    = NULL,
    .get_capa       = NULL,
    .poll           = NULL,
    .get_ifname     = NULL, /* Not nesessary */
    .set_operstate  = NULL,
#ifdef CONFIG_CLIENT_MLME
    .get_hw_modes   = NULL,
    .set_channel    = NULL,
    .set_ssid       = wpa_driver_tista_set_ssid,
    .set_bssid      = NULL,
    .send_mlme      = NULL,
    .mlme_add_sta   = NULL,
    .mlme_remove_sta    = NULL,
    .mlme_setprotection = NULL,
#endif /* CONFIG_CLIENT_MLME */
    .driver_cmd     = wpa_driver_tista_driver_cmd
};
