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
#ifndef _DRIVER_TI_H_
#define _DRIVER_TI_H_

#include "wpa.h"
#include "config.h"
#include "osDot11.h"
#include "802_11Defs.h"
#include "TI_AdapterApiC.h"
#include "tiioctl.h"
#include "shlist.h"
/*-------------------------------------------------------------------*/
#define TIWLAN_DRV_NAME         "tiwlan0"
#define TKIP_KEY_LENGTH         32
#define AES_KEY_LENGTH          16
#define WEP_KEY_LENGTH_40       5
#define WEP_KEY_LENGTH_104      13

#define TIMAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define TIMACSTR "%02x:%02x:%02x:%02x:%02x:%02x"

typedef enum _TIWLAN_KEY_FLAGS
{
        TIWLAN_KEY_FLAGS_TRANSMIT       = 0x80000000,           /* Used whenever key should be immidiately used for TX */
        TIWLAN_KEY_FLAGS_PAIRWISE       = 0x40000000,           /* Used to indicate pairwise key */
        TIWLAN_KEY_FLAGS_SET_KEY_RSC    = 0x20000000,           /* Used to set RSC (receive sequence counter) to driver */
        TIWLAN_KEY_FLAGS_AUTHENTICATOR  = 0x10000000            /* Not used currently */
} TIWLAN_KEY_FLAGS;

#define NO_BLOCK                          0
#define NO_BLOCK_DISASSOC_IN_PROGRESS     1
#define BLOCK_DISASSOC                    2
#define NO_BLOCK_WPS                      3

#ifndef WPA_OUI
/* { 0x00, 0x50, 0xf2, 0x01 } */
#define WPA_OUI "\x00\x50\xf2\x01"
#endif

#define NUMBER_SCAN_CHANNELS_FCC        11
#define NUMBER_SCAN_CHANNELS_ETSI       13
#define NUMBER_SCAN_CHANNELS_MKK1       14

#ifndef ETHERNET_HDR_LEN
#define ETHERNET_HDR_LEN        14
#endif

#define WPA_OPTIONS_DISABLE     0
#define WPA_OPTIONS_ENABLE      3

#ifndef MIN
#define MIN(x,y)                (((x) < (y)) ? (x) : (y))
#endif

/* TI station control structure */
struct wpa_driver_ti_data {
    UINT8 own_addr[ETH_ALEN];          /* MAC address of WLAN interface */
    void *hWpaSupplicant;              /* Handle to wpa_supplicant */
    TI_HANDLE hDriver;                 /* Handle to self */
    TI_HANDLE hEvents[IPC_EVENT_MAX];  /* Event handles - needed to register to driver events */
    int key_mgmt;                      /* Key Management: 802_1X/PSK/NONE/802_1X_NO_WPA/WPA_NONE */
    int proto;                         /* Protocol (WPA/WPA2/RSN) */
    int encryption;                    /* Encryption type */
    int auth_alg;                      /* Authentication Alg: Open, Shared or LEAP */
    int driver_is_loaded;              /* TRUE/FALSE flag if driver is already loaded and can be accessed */
    int scan_type;                     /* SCAN_TYPE_NORMAL_ACTIVE or  SCAN_TYPE_NORMAL_PASSIVE */
    int scan_channels;                 /* Number of allowed scan channels */
    unsigned link_speed;               /* Link Speed */
    unsigned btcoex_mode;              /* BtCoex Mode */
    int btcoex_scan;                   /* BtCoex Scan */
    unsigned rts_threshold;            /* RTS Threshold */
    int last_scan;                     /* Last scan type */
    int force_merge_flag;              /* Force scan results merge */
    SHLIST scan_merge_list;            /* Previous scan list */
    int driverEventsSocket;
    int block_disassoc_events;
    int block_disassoc_prev;
};

/* external symbols from common.c - will be used to dynamically change debug level by IOCTLs */
extern int wpa_debug_level;
extern int wpa_debug_show_keys;
extern int wpa_debug_timestamp;
#endif /* _DRIVER_TI_H_ */
