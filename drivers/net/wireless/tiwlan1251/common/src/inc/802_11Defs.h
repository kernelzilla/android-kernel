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
 *
 *   MODULE:  802_11Defs.h
 *   PURPOSE: Contains 802.11 defines/structures
 *
 ****************************************************************************/

#ifndef _802_11_INFO_DEFS_H
#define _802_11_INFO_DEFS_H

#include "osTIType.h"
#include "commonTypes.h"
#include "paramIn.h"

/* FrameControl field of the 802.11 header  */

/**/
/* bit 15    14     13     12     11     10      9     8     7-4     3-2      1-0*/
/* +-------+-----+------+-----+-------+------+------+----+---------+------+----------+*/
/* | Order | WEP | More | Pwr | Retry | More | From | To | Subtype | Type | Protocol |*/
/* |       |     | Data | Mgmt|       | Frag | DS   | DS |         |      | Version  |*/
/* +-------+-----+------+-----+-------+------+------+----+---------+------+----------+*/
/*     1      1      1     1      1       1      1     1       4       2        2*/


#define DOT11_FC_PROT_VERSION_MASK   ( 3 << 0 )
#define DOT11_FC_PROT_VERSION        ( 0 << 0 )

#define DOT11_FC_TYPE_MASK           ( 3 << 2 )
typedef enum
{
  DOT11_FC_TYPE_MGMT         = ( 0 << 2 ),
  DOT11_FC_TYPE_CTRL         = ( 1 << 2 ),
  DOT11_FC_TYPE_DATA         = ( 2 << 2 )
} dot11_Fc_Type_e;

#define DOT11_FC_SUB_MASK           ( 0x0f << 4 )
typedef enum
{
  /* Management subtypes */
  DOT11_FC_SUB_ASSOC_REQ     = (    0 << 4 ),
  DOT11_FC_SUB_ASSOC_RESP    = (    1 << 4 ),
  DOT11_FC_SUB_REASSOC_REQ   = (    2 << 4 ),
  DOT11_FC_SUB_REASSOC_RESP  = (    3 << 4 ),
  DOT11_FC_SUB_PROBE_REQ     = (    4 << 4 ),
  DOT11_FC_SUB_PROBE_RESP    = (    5 << 4 ),
  DOT11_FC_SUB_BEACON        = (    8 << 4 ),
  DOT11_FC_SUB_ATIM          = (    9 << 4 ),
  DOT11_FC_SUB_DISASSOC      = (   10 << 4 ),
  DOT11_FC_SUB_AUTH          = (   11 << 4 ),
  DOT11_FC_SUB_DEAUTH        = (   12 << 4 ),
  DOT11_FC_SUB_ACTION        = (   13 << 4 ),

  /* Control subtypes */
  DOT11_FC_SUB_PS_POLL                = (   10 << 4 ),
  DOT11_FC_SUB_RTS                    = (   11 << 4 ),
  DOT11_FC_SUB_CTS                    = (   12 << 4 ),
  DOT11_FC_SUB_ACK                    = (   13 << 4 ),
  DOT11_FC_SUB_CF_END                 = (   14 << 4 ),
  DOT11_FC_SUB_CF_END_CF_ACK          = (   15 << 4 ),

  /* Data subtypes */
  DOT11_FC_SUB_DATA                   = (    0 << 4 ),
  DOT11_FC_SUB_DATA_CF_ACK            = (    1 << 4 ),
  DOT11_FC_SUB_DATA_CF_POLL           = (    2 << 4 ),
  DOT11_FC_SUB_DATA_CF_ACK_CF_POLL    = (    3 << 4 ),
  DOT11_FC_SUB_NULL_FUNCTION          = (    4 << 4 ),
  DOT11_FC_SUB_CF_ACK                 = (    5 << 4 ),
  DOT11_FC_SUB_CF_POLL                = (    6 << 4 ),
  DOT11_FC_SUB_CF_ACK_CF_POLL         = (    7 << 4 ),
  DOT11_FC_SUB_DATA_QOS               = (    8 << 4 ),
  DOT11_FC_SUB_DATA_NULL_QOS          = (   12 << 4 )
} dot11_Fc_Sub_Type_e;

#define  DOT11_FC_TYPESUBTYPE_MASK    ( DOT11_FC_TYPE_MASK | DOT11_FC_SUB_MASK )
typedef enum
{
  DOT11_FC_ASSOC_REQ           = ( DOT11_FC_TYPE_MGMT | DOT11_FC_SUB_ASSOC_REQ           ),
  DOT11_FC_ASSOC_RESP          = ( DOT11_FC_TYPE_MGMT | DOT11_FC_SUB_ASSOC_RESP          ),
  DOT11_FC_REASSOC_REQ         = ( DOT11_FC_TYPE_MGMT | DOT11_FC_SUB_REASSOC_REQ         ),
  DOT11_FC_REASSOC_RESP        = ( DOT11_FC_TYPE_MGMT | DOT11_FC_SUB_REASSOC_RESP        ),
  DOT11_FC_PROBE_REQ           = ( DOT11_FC_TYPE_MGMT | DOT11_FC_SUB_PROBE_REQ           ),
  DOT11_FC_PROBE_RESP          = ( DOT11_FC_TYPE_MGMT | DOT11_FC_SUB_PROBE_RESP          ),
  DOT11_FC_BEACON              = ( DOT11_FC_TYPE_MGMT | DOT11_FC_SUB_BEACON              ),
  DOT11_FC_ATIM                = ( DOT11_FC_TYPE_MGMT | DOT11_FC_SUB_ATIM                ),
  DOT11_FC_DISASSOC            = ( DOT11_FC_TYPE_MGMT | DOT11_FC_SUB_DISASSOC            ),
  DOT11_FC_AUTH                = ( DOT11_FC_TYPE_MGMT | DOT11_FC_SUB_AUTH                ),
  DOT11_FC_DEAUTH              = ( DOT11_FC_TYPE_MGMT | DOT11_FC_SUB_DEAUTH              ),
  DOT11_FC_ACTION              = ( DOT11_FC_TYPE_MGMT | DOT11_FC_SUB_ACTION              ),
  DOT11_FC_PS_POLL             = ( DOT11_FC_TYPE_CTRL | DOT11_FC_SUB_PS_POLL             ),
  DOT11_FC_RTS                 = ( DOT11_FC_TYPE_CTRL | DOT11_FC_SUB_RTS                 ),
  DOT11_FC_CTS                 = ( DOT11_FC_TYPE_CTRL | DOT11_FC_SUB_CTS                 ),
  DOT11_FC_ACK                 = ( DOT11_FC_TYPE_CTRL | DOT11_FC_SUB_ACK                 ),
  DOT11_FC_CF_END              = ( DOT11_FC_TYPE_CTRL | DOT11_FC_SUB_CF_END              ),
  DOT11_FC_CF_END_CF_ACK       = ( DOT11_FC_TYPE_CTRL | DOT11_FC_SUB_CF_END_CF_ACK       ),
  DOT11_FC_DATA                = ( DOT11_FC_TYPE_DATA | DOT11_FC_SUB_DATA                ),
  DOT11_FC_DATA_CF_ACK         = ( DOT11_FC_TYPE_DATA | DOT11_FC_SUB_DATA_CF_ACK         ),
  DOT11_FC_DATA_CF_POLL        = ( DOT11_FC_TYPE_DATA | DOT11_FC_SUB_DATA_CF_POLL        ),
  DOT11_FC_DATA_CF_ACK_CF_POLL = ( DOT11_FC_TYPE_DATA | DOT11_FC_SUB_DATA_CF_ACK_CF_POLL ),
  DOT11_FC_DATA_NULL_FUNCTION  = ( DOT11_FC_TYPE_DATA | DOT11_FC_SUB_NULL_FUNCTION       ),
  DOT11_FC_CF_ACK              = ( DOT11_FC_TYPE_DATA | DOT11_FC_SUB_CF_ACK              ),
  DOT11_FC_CF_POLL             = ( DOT11_FC_TYPE_DATA | DOT11_FC_SUB_CF_POLL             ),
  DOT11_FC_CF_ACK_CF_POLL      = ( DOT11_FC_TYPE_DATA | DOT11_FC_SUB_CF_ACK_CF_POLL      ),
  DOT11_FC_DATA_QOS            = ( DOT11_FC_TYPE_DATA | DOT11_FC_SUB_DATA_QOS            ),
  DOT11_FC_DATA_NULL_QOS       = ( DOT11_FC_TYPE_DATA | DOT11_FC_SUB_DATA_NULL_QOS       )
} dot11_Fc_Type_Sub_Type_e;

typedef enum
{
  DOT11_FC_TO_DS               = ( 1 << 8  ),
  DOT11_FC_FROM_DS             = ( 1 << 9  ),
  DOT11_FC_MORE_FRAG           = ( 1 << 10 ),
  DOT11_FC_RETRY               = ( 1 << 11 ),
  DOT11_FC_PWR_MGMT            = ( 1 << 12 ),
  DOT11_FC_MORE_DATA           = ( 1 << 13 ),
  DOT11_FC_WEP                 = ( 1 << 14 ),
  DOT11_FC_ORDER               = ( 1 << 15 )
} dot11_Fc_Other_e;

typedef enum
{
  DOT11_CAPABILITY_ESS               = ( 1 ),
  DOT11_CAPABILITY_IESS              = ( 1 << 1 ),
  DOT11_CAPABILITY_CF_POLLABE        = ( 1 << 2 ),
  DOT11_CAPABILITY_CF_POLL_REQ       = ( 1 << 3 ),
  DOT11_CAPABILITY_PRIVACY           = ( 1 << 4 ),
  DOT11_CAPABILITY_PREAMBLE          = ( 1 << 5 ),
  DOT11_CAPABILITY_PBCC              = ( 1 << 6 ),
  DOT11_CAPABILITY_AGILE             = ( 1 << 7 ),
} dot11_Capability_e;

#define  DOT11_FC_TO_DS_SHIFT        8
#define  DOT11_FC_FROM_DS_SHIFT      9
#define  DOT11_FC_MORE_FRAG_SHIFT   10
#define  DOT11_FC_RETRY_SHIFT       11
#define  DOT11_FC_PWR_MGMT_SHIFT    12
#define  DOT11_FC_MORE_DATA_SHIFT   13
#define  DOT11_FC_WEP_SHIFT         14
#define  DOT11_FC_ORDER_SHIFT       15

#define IS_WEP_ON(fc)       ((TRUE << DOT11_FC_WEP_SHIFT) & (fc))
#define IS_DATA(fc)         (((DOT11_FC_TYPE_MASK) & (fc)) == DOT11_FC_TYPE_DATA)
#define IS_LEGACY_DATA(fc)  (((DOT11_FC_TYPESUBTYPE_MASK) & (fc)) == DOT11_FC_DATA)
#define IS_AUTH(fc)         (((DOT11_FC_AUTH) & (fc)) == DOT11_FC_AUTH)
#define IS_QOS_FRAME(fc)    ( (((fc) & (DOT11_FC_TYPESUBTYPE_MASK)) == DOT11_FC_DATA_QOS)   ||   \
                              (((fc) & (DOT11_FC_TYPESUBTYPE_MASK)) == DOT11_FC_DATA_NULL_QOS) )



#define TUs_TO_MSECs(x)     (((x) << 10) / 1000)

#define TIME_STAMP_LEN  8

/* SequenceControl field of the 802.11 header */
/**/
/* bit    15 - 4           3 - 0*/
/* +-------------------+-----------+*/
/* |  Sequence Number  | Fragment  |*/
/* |                   |  Number   |*/
/* +-------------------+-----------+*/
/*         12                4*/

typedef enum
{
  DOT11_SC_FRAG_NUM_MASK = ( 0xf   << 0 ),
  DOT11_SC_SEQ_NUM_MASK  = ( 0xfff << 4 )
} dot11_Sc_t;

#define DOT11_QOS_CONTROL_ACK        0x0000
#define DOT11_QOS_CONTROL_DONT_ACK   0x0020

#pragma pack(1)
PACKED_STRUCT( dot11_header_t,

  UINT16        fc;
  UINT16        duration;
  macAddress_t  address1;
  macAddress_t  address2;
  macAddress_t  address3;
  UINT16        seqCtrl;
  UINT16        qosControl;
/*  macAddress_t    address4;*/
);
#pragma pack()

#pragma pack(1)
PACKED_STRUCT( legacy_dot11_header_t,

  UINT16        fc;
  UINT16        duration;
  macAddress_t  address1;
  macAddress_t  address2;
  macAddress_t  address3;
  UINT16        seqCtrl;
);
#pragma pack()



#pragma pack(1)
PACKED_STRUCT( dot11_mgmtHeader_t,

  UINT16        fc;
  UINT16        duration;
  macAddress_t  DA;
  macAddress_t  SA;
  macAddress_t  BSSID;
  UINT16        seqCtrl;
);
#pragma pack()

#pragma pack(1)
PACKED_STRUCT( Wlan_LlcHeader_T,

  UINT8     DSAP;
  UINT8     SSAP;
  UINT8     Control;
  UINT8     OUI[3];
  UINT16    Type;
);
#pragma pack()

#pragma pack(1)
PACKED_STRUCT( dot11_PsPollFrameHeader_t,

  UINT16        fc;
  UINT16        AID;
  macAddress_t  BSSID;
  macAddress_t  TA;
);
#pragma pack()



#define FCS_SIZE    4

#define WLAN_HDR_LEN                            24
#define WLAN_QOS_HDR_LEN                        26
#define WLAN_SNAP_HDR_LEN                       8
#define WLAN_WITH_SNAP_HEADER_MAX_SIZE          (WLAN_HDR_LEN + WLAN_SNAP_HDR_LEN)
#define WLAN_WITH_SNAP_QOS_HEADER_MAX_SIZE      (WLAN_QOS_HDR_LEN + WLAN_SNAP_HDR_LEN)

#define GET_MAX_HEADER_SIZE(macHeaderPointer,headerSize)   if( ( ((*(UINT16*)(macHeaderPointer)) & DOT11_FC_TYPESUBTYPE_MASK) == DOT11_FC_DATA_QOS ) || ( ((*(UINT16*)(macHeaderPointer)) & DOT11_FC_TYPESUBTYPE_MASK) == DOT11_FC_DATA_NULL_QOS ) ) *headerSize = WLAN_QOS_HDR_LEN; else *headerSize = WLAN_HDR_LEN;

/* data body max length */
#define MAX_DATA_BODY_LENGTH                2312

/****************************************************************************************
    The next table is defined in 802.11 spec section 7.2.2 for the address field contents :
    To DS   From DS     Address 1    Address 2  Address 3    Address 4
    -------------------------------------------------------------------
    0           0           DA          SA          BSSID       N/A
    0           1           DA          BSSID       SA          N/A
    1           0           BSSID       SA          DA          N/A
    1           1           RA          TA          DA          SA         
    
NOTE: We only support packets coming from within the DS (i.e. From DS = 0)
*****************************************************************************************/
/* return the destination address used in *dot11_header_t */
#define GET_DA_FROM_DOT11_HEADER_T(pDot11Hdr)   ((pDot11Hdr->fc & DOT11_FC_TO_DS) ? (&pDot11Hdr->address3) : (&pDot11Hdr->address1)) 


/*
 * MANAGEMENT
 * -----------------
 */

/* mgmt body max length */
#define MAX_MGMT_BODY_LENGTH                2312
/* maximal length of beacon body - note that actual beacons may actually be longer
   than this size, at least according to the spec, but so far no larger beacon was seen */
#define MAX_BEACON_BODY_LENGTH              300

/* general mgmt frame structure */

PACKED_STRUCT( dot11_mgmtFrame_t,

    dot11_mgmtHeader_t  hdr;
    char                body[MAX_MGMT_BODY_LENGTH];
);

/* Capabilities Information Field - IN THE AIR */
/**/
/*  bit  15      14       13         12        11         10      9      8      7   -   0*/
/* +----------+------+----------+---------+----------+---------+------+-----+---------------+*/
/* |  Channel |      |  Short   | Privacy | CF Poll  |   CF    |      |     |   RESERVED    |   */
/* |  Agility | PBCC | Preamble |         | Request  | Pollable| IBSS | ESS |               |*/
/* +----------+------+----------+---------+----------+---------+------+-----+---------------+   */
/*       1        1       1          1         1          1       1      1*/


/* Capabilities Information Field - IN THE MGMT SOFTWARE AFTER THE SWAP */
/**/
/* bit 15 - 8         7        6       5          4         3          2       1      0*/
/* +------------+----------+------+----------+---------+----------+---------+------+-----+*/
/* |            |  Channel |      |  Short   | Privacy | CF Poll  |   CF    |      |     |*/
/* |  Reserved  |  Agility | PBCC | Preamble |         | Request  | Pollable| IBSS | ESS |*/
/* +------------+----------+------+----------+---------+----------+---------+------+-----+*/
/*       8            1        1       1          1         1          1       1      1*/



typedef enum
{ 
  DOT11_CAPS_ESS             = ( 1 << 0 ),
  DOT11_CAPS_IBSS            = ( 1 << 1 ),
  DOT11_CAPS_CF_POLLABLE     = ( 1 << 2 ),
  DOT11_CAPS_CF_POLL_REQUEST = ( 1 << 3 ),
  DOT11_CAPS_PRIVACY         = ( 1 << 4 ),
  DOT11_CAPS_SHORT_PREAMBLE  = ( 1 << 5 ),
  DOT11_CAPS_PBCC            = ( 1 << 6 ),
  DOT11_CAPS_CHANNEL_AGILITY = ( 1 << 7 ),
  DOT11_SPECTRUM_MANAGEMENT  = ( 1 << 8 ),
  DOT11_CAPS_QOS_SUPPORTED   = ( 1 << 9 ),
  DOT11_CAPS_SHORT_SLOT_TIME = (1  << 10),

  DOT11_CAPS_APSD_SUPPORT    = ( 1 << 11),
} dot11_capabilities_e;

typedef enum 
{
    /* ESS */
    CAP_ESS_MASK            = 1,
    CAP_ESS_SHIFT           = 0,

    /* IBSS */
    CAP_IBSS_MASK           = 1,
    CAP_IBSS_SHIFT          = 1,

    /* CF Pollable */
    CAP_CF_POLL_MASK        = 1,
    CAP_CF_POLL_SHIFT       = 2,
    
    /* CF Poll request */
    CAP_CF_REQ_MASK         = 1,
    CAP_CF_REQ_SHIFT        = 3,
    
    /* Privacy */
    CAP_PRIVACY_MASK        = 1,
    CAP_PRIVACY_SHIFT       = 4,

    /* Short Preamble*/
    CAP_PREAMBLE_MASK       = 1,
    CAP_PREAMBLE_SHIFT      = 5,
    
    /* PBCC */
    CAP_PBCC_MASK           = 1,
    CAP_PBCC_SHIFT          = 6,
    
    /* Agile */
    CAP_AGILE_MASK          = 1,
    CAP_AGILE_SHIFT         = 7,

    /* Slot time */
    CAP_SLOT_TIME_MASK      = 1,
    CAP_SLOT_TIME_SHIFT     = 10,

    /* APSD */
    CAP_APSD_MASK           = 1,
    CAP_APSD_SHIFT          = 11,


} wdrv_mgmtCapabilities_e;


/*
 * 802.11 Information elements
 * ---------------------------
 */


PACKED_STRUCT( dot11_eleHdr_t,

  UINT8 eleId;
  UINT8 eleLen;
);

/* fixed fields lengths, except of currentAP & timestamp*/
#define FIX_FIELD_LEN       2

/* SSID Information Element */
#define DOT11_SSID_ELE_ID   0

PACKED_STRUCT( dot11_SSID_t,

  dot11_eleHdr_t    hdr;
  char              serviceSetId[MAX_SSID_LEN];
);


/* Supportted rates Information Element */
#define DOT11_SUPPORTED_RATES_ELE_ID        1
#define DOT11_EXT_SUPPORTED_RATES_ELE_ID        50

PACKED_STRUCT( dot11_RATES_t,

  dot11_eleHdr_t hdr;
  UINT8 rates[MAX_SUPPORTED_RATES];
);


#define ERP_IE_NON_ERP_PRESENT_MASK         0x1
#define ERP_IE_USE_PROTECTION_MASK          0x2
#define ERP_IE_BARKER_PREAMBLE_MODE_MASK    0x4
#define DOT11_ERP_IE_ID 42

PACKED_STRUCT( dot11_ERP_t,

    dot11_eleHdr_t  hdr;
    UINT8           ctrl;
);

/* RSN Information Element */
#define DOT11_RSN_MAX                       255 

PACKED_STRUCT( dot11_RSN_t,

  dot11_eleHdr_t hdr;
  UINT8 rsnIeData[DOT11_RSN_MAX];
);

/* general definitions needed by whalWpa.c and rsn.c */
#define IV_FIELD_SIZE   4 
#define ICV_FIELD_SIZE  4
#define MIC_FIELD_SIZE  8 
#define EIV_FIELD_SIZE  4
#define WEP_AFTER_HEADER_FIELD_SIZE  IV_FIELD_SIZE
#define TKIP_AFTER_HEADER_FIELD_SIZE (IV_FIELD_SIZE + EIV_FIELD_SIZE)
#define AES_AFTER_HEADER_FIELD_SIZE  8


/* DS params Information Element */
#define DOT11_DS_PARAMS_ELE_ID      3
#define DOT11_DS_PARAMS_ELE_LEN     1

PACKED_STRUCT( dot11_DS_PARAMS_t,

  dot11_eleHdr_t hdr;
  UINT8  currChannel;
);


/* DS params Information Element */
#define DOT11_IBSS_PARAMS_ELE_ID    6
#define DOT11_IBSS_PARAMS_ELE_LEN   2

PACKED_STRUCT( dot11_IBSS_PARAMS_t,

    dot11_eleHdr_t  hdr;
    UINT16          atimWindow;
);

#define DOT11_FH_PARAMS_ELE_ID      2
#define DOT11_FH_PARAMS_ELE_LEN     5

PACKED_STRUCT( dot11_FH_PARAMS_t,

    dot11_eleHdr_t  hdr;
    UINT16          dwellTime;
    UINT8           hopSet;
    UINT8           hopPattern;
    UINT8           hopIndex;
);

/* tim Information Element */
#define DOT11_TIM_ELE_ID    5
#define DOT11_PARTIAL_VIRTUAL_BITMAP_MAX    251

PACKED_STRUCT( dot11_TIM_t,

    dot11_eleHdr_t  hdr;
    UINT8           dtimCount;
    UINT8           dtimPeriod;
    UINT8           bmapControl;
    UINT8           partialVirtualBmap[DOT11_PARTIAL_VIRTUAL_BITMAP_MAX];
);

/* tim Information Element */
#define DOT11_CF_ELE_ID             4
#define DOT11_CF_PARAMS_ELE_LEN     6

PACKED_STRUCT( dot11_CF_PARAMS_t,

    dot11_eleHdr_t  hdr;
    UINT8           cfpCount;
    UINT8           cfpPeriod;
    UINT16          cfpMaxDuration;
    UINT16          cfpDurRemain;
);

/* Challenge text Information Element */
#define DOT11_CHALLENGE_TEXT_ELE_ID     16
#define DOT11_CHALLENGE_TEXT_MAX        253

PACKED_STRUCT( dot11_CHALLENGE_t,

    dot11_eleHdr_t  hdr;
    UINT8           text[ DOT11_CHALLENGE_TEXT_MAX ];
);


/* Country Inforamtion Element */
#define DOT11_COUNTRY_ELE_ID        7
#define DOT11_COUNTRY_ELE_LEN_MAX   ( ((NUM_OF_MAX_TRIPLET_CHANNEL+1)*3) + !((NUM_OF_MAX_TRIPLET_CHANNEL&0x1)))

PACKED_STRUCT( dot11_COUNTRY_t,

    dot11_eleHdr_t  hdr;
    countryIE_t     countryIE;
);


/* Power Constraint Information Element */
#define DOT11_POWER_CONSTRAINT_ELE_ID       (32)
#define DOT11_POWER_CONSTRAINT_ELE_LEN      (1)

PACKED_STRUCT( dot11_POWER_CONSTRAINT_t,

    dot11_eleHdr_t  hdr;
    UINT8           powerConstraint;
);



/* Power Capability Information Element */
#define DOT11_CAPABILITY_ELE_ID         (33)
#define DOT11_CAPABILITY_ELE_LEN        (2)

PACKED_STRUCT( dot11_CAPABILITY_t,

    dot11_eleHdr_t  hdr;
    UINT8           minTxPower;
    UINT8           maxTxPower;
);

/* TPC request Information Element */
#define DOT11_TPC_REQUEST_ELE_ID        (34)
#define DOT11_TPC_REQUEST_ELE_LEN       (0)

PACKED_STRUCT( dot11_TPC_REQUEST_t,

    dot11_eleHdr_t  hdr;
);

/* TPC report Information Element */
#define DOT11_TPC_REPORT_ELE_ID         (35)
#define DOT11_TPC_REPORT_ELE_LEN        (2)

PACKED_STRUCT( dot11_TPC_REPORT_t,

    dot11_eleHdr_t  hdr;
    UINT8           transmitPower;
    UINT8           linkMargin;
);

#ifdef EXC_MODULE_INCLUDED
/* Cell Transmit Power Information Element */
#define DOT11_CELL_TP_ELE_ID            (150)
#define DOT11_CELL_TP_ELE_LEN           (6)

PACKED_STRUCT( dot11_CELL_TP_t,

    dot11_eleHdr_t  hdr;
    UINT8           oui[4];
    UINT8           power;
    UINT8           reerved;
);

#define   DOT11_CELL_TP \
    dot11_CELL_TP_t         *cellTP;
    
#else
#define   DOT11_CELL_TP
#endif

/* Channel Supported Information Element */
#define DOT11_CHANNEL_SUPPORTED_ELE_ID  (36)
#define DOT11_CHANNEL_SUPPORTED_ELE_LEN (26)

PACKED_STRUCT( dot11_CHANNEL_SUPPORTED_t,

    dot11_eleHdr_t  hdr;
    UINT8           supportedChannel[DOT11_CHANNEL_SUPPORTED_ELE_LEN];

);

/* Channel Switch Announcement Information Element */
#define DOT11_CHANNEL_SWITCH_ELE_ID     (37)
#define DOT11_CHANNEL_SWITCH_ELE_LEN    (3)

PACKED_STRUCT( dot11_CHANNEL_SWITCH_t,

    dot11_eleHdr_t  hdr;
    UINT8           channelSwitchMode;
    UINT8           channelNumber;
    UINT8           channelSwitchCount;
);

#define MAX_NUM_REQ (16)

/* Measurement request Information Element */
#define DOT11_MEASUREMENT_REQUEST_ELE_ID        (38)
#define DOT11_MEASUREMENT_REQUEST_LEN           (2)
#define DOT11_MEASUREMENT_REQUEST_ELE_LEN       (3 + DOT11_MEASUREMENT_REQUEST_LEN*MAX_NUM_REQ)

PACKED_STRUCT( dot11_MEASUREMENT_REQUEST_t,

    dot11_eleHdr_t  hdr;
    UINT8           measurementToken;
    UINT8           measurementMode;
    UINT8           measurementType;
    UINT8           measurementRequests[DOT11_MEASUREMENT_REQUEST_LEN*MAX_NUM_REQ];
);


/* Measurement report Information Element */
#define DOT11_MEASUREMENT_REPORT_ELE_ID     (39)
#define DOT11_MAX_MEASUREMENT_REPORT_LEN    (4)
#define DOT11_MIN_MEASUREMENT_REPORT_IE_LEN (3)
#define DOT11_MEASUREMENT_REPORT_ELE_IE_LEN (DOT11_MIN_MEASUREMENT_REPORT_IE_LEN + DOT11_MAX_MEASUREMENT_REPORT_LEN*MAX_NUM_REQ)

PACKED_STRUCT( dot11_MEASUREMENT_REPORT_t,

    dot11_eleHdr_t  hdr;
    UINT8           measurementToken;
    UINT8           measurementMode;
    UINT8           measurementType;
    UINT8           measurementReports[DOT11_MAX_MEASUREMENT_REPORT_LEN*MAX_NUM_REQ];
);

/* Quiet Information Element */
#define DOT11_QUIET_ELE_ID              (40)
#define DOT11_QUIET_ELE_LEN             (6)

PACKED_STRUCT( dot11_QUIET_t,

    dot11_eleHdr_t  hdr;
    UINT8           quietCount;
    UINT8           quietPeriod;
    UINT16          quietDuration;
    UINT16          quietOffset;
);


/* QoS Capability Information Element */
#define DOT11_QOS_CAPABILITY_ELE_ID     (46)
#define DOT11_QOS_CAPABILITY_ELE_LEN    (1)

#define AC_APSD_FLAGS_MASK              (1)
#define Q_ACK_BITG_MASK                 (1)
#define MAX_SP_LENGTH_MASK              (3)
#define MORE_DATA_ACK_MASK              (1)

#define AC_VO_APSD_FLAGS_SHIFT          (0)
#define AC_VI_APSD_FLAGS_SHIFT          (1)
#define AC_BK_APSD_FLAGS_SHIFT          (2)
#define AC_BE_APSD_FLAGS_SHIFT          (3)
#define Q_ACK_FLAGS_SHIFT               (4)
#define MAX_SP_LENGTH_SHIFT             (5)
#define MORE_DATA_ACK_SHIFT             (7)

#define QOS_CONTROL_UP_SHIFT            (0)

#define AP_QOS_INFO_UAPSD_MASK          (1)
#define AP_QOS_INFO_UAPSD_SHIFT         (7)


PACKED_STRUCT( dot11_QOS_CAPABILITY_IE_t,

    dot11_eleHdr_t  hdr;
    UINT8           QosInfoField;
);

/* WPS Information Element */
#define DOT11_WPS_ELE_ID	(221)
#define DOT11_WPS_OUI		{0x00, 0x50, 0xF2, 0x04}
#define DOT11_WPS_OUI_LEN	4

/* WME Information Element */
#define DOT11_WME_ELE_ID                (221)
#define DOT11_WME_ELE_LEN               (7)

PACKED_STRUCT( dot11_WME_IE_t,

    dot11_eleHdr_t  hdr;
    UINT8           OUI[3];
    UINT8           OUIType;
    UINT8           OUISubType;
    UINT8           version;
    UINT8           ACInfoField;
);


/* WME Parameter Information Element */
#define DOT11_WME_PARAM_ELE_ID          (221)
#define DOT11_WME_PARAM_ELE_LEN         (24)

PACKED_STRUCT( dot11_WME_PARAM_t,

    dot11_eleHdr_t      hdr;
    UINT8               OUI[3];
    UINT8               OUIType;
    UINT8               OUISubType;
    UINT8               version;
    UINT8               ACInfoField;
    UINT8               reserved;
    ACParameters_t      WME_ACParameteres;
);

#define dot11_WPA_OUI_TYPE                  (1)
#define dot11_WME_OUI_TYPE                  (2)
#define dot11_WME_OUI_SUB_TYPE_IE           (0)
#define dot11_WME_OUI_SUB_TYPE_PARAMS_IE    (1)
#define dot11_WME_VERSION                   (1)
#define dot11_WME_ACINFO_MASK               0x0f

/* -------------------- TSPEC ----------------- */

#pragma pack(1)
PACKED_STRUCT( tsInfo_t,

    UINT8   tsInfoArr[3];

);
#pragma pack()


#pragma pack(1)

/* This structure is part of the TSPEC structure. It was seperated since there are some cases (such as DEL_TS), which we dont need
to send ALL the TSPEC structure, but only as far as TsInfo. The TSPEC structure contains this smaller structure */
PACKED_STRUCT( dot11_WME_TSPEC_IE_hdr_t,

    dot11_eleHdr_t  hdr;
    
    UINT8   OUI[3];
    UINT8   oui_type;
    UINT8   oui_subtype;
    UINT8   version;

    tsInfo_t tsInfoField;
);


PACKED_STRUCT( dot11_WME_TSPEC_IE_t,

    dot11_WME_TSPEC_IE_hdr_t tHdr;

    UINT16  nominalMSDUSize;
    UINT16  maximumMSDUSize;
    UINT32  minimumServiceInterval;
    UINT32  maximumServiceInterval;
    UINT32  inactivityInterval;
    UINT32  suspensionInterval;
    UINT32  serviceStartTime;
    UINT32  minimumDataRate;
    UINT32  meanDataRate;
    UINT32  peakDataRate;
    UINT32  maximumBurstSize;
    UINT32  delayBound;
    UINT32  minimumPHYRate;
    UINT16  surplusBandwidthAllowance;
    UINT16  mediumTime;
);
#pragma pack()

#define WME_TSPEC_IE_ID                         221
#define WME_TSPEC_IE_LEN                        61
#define WME_TSPEC_IE_TSINFO_LEN                 9                
#define WME_TSPEC_IE_OUI_TYPE                   0x02
#define WME_TSPEC_IE_OUI_SUB_TYPE               0x02
#define WME_TSPEC_IE_VERSION                    0x01

/* OUI TYPE values that can be present in management packets inside Cisco vendor specific IE */
typedef enum
{
    TS_METRIX_OUI_TYPE = 0x07,
    TS_RATE_SET_OUI_TYPE = 0x08,
    EDCA_LIFETIME_OUI_TYPE = 0x09
} EXC_IE_OUI_TYPE_t;

#define ADDTS_REQUEST_ACTION                    0x00
#define ADDTS_RESPONSE_ACTION                   0x01
#define DELTS_ACTION                            0x02

#define ADDTS_STATUS_CODE_SUCCESS               0x00
#define DELTS_CODE_SUCCESS                      0x00
 

#define TS_INFO_0_TRAFFIC_TYPE_MASK             0x01
#define TS_INFO_0_TSID_MASK                     0x1E
#define TS_INFO_0_DIRECTION_MASK                0x60
#define TS_INFO_0_ACCESS_POLICY_MASK            0x80

#define TS_INFO_1_ACCESS_POLICY_MASK            0x01
#define TS_INFO_1_AGGREGATION_MASK              0x02
#define TS_INFO_1_APSD_MASK                     0x04    
#define TS_INFO_1_USER_PRIORITY_MASK            0x38
#define TS_INFO_1_TSINFO_ACK_POLICY_MASK        0xC0

#define TS_INFO_2_SCHEDULE_MASK                 0x01
#define TS_INFO_2_RESERVED_MASK                 0xF7    

#define TRAFFIC_TYPE_SHIFT                      0
#define TSID_SHIFT                              1
#define DIRECTION_SHIFT                         5
#define ACCESS_POLICY_SHIFT                     7
#define AGGREGATION_SHIFT                       1
#define APSD_SHIFT                              2   
#define USER_PRIORITY_SHIFT                     3
#define TSINFO_ACK_POLICY_SHIFT                 6
#define SCHEDULE_SHIFT                          0
#define RESERVED_SHIFT                          1
#define SURPLUS_BANDWIDTH_ALLOW                 13  

#define TS_INFO_0_ACCESS_POLICY_EDCA            0x1                 
#define NORMAL_ACKNOWLEDGEMENT                  0x00        
#define NO_SCHEDULE                             0x00        
#define PS_UPSD                                 0x01
#define EDCA_MODE                               0x08
#define FIX_MSDU_SIZE                           0x8000


/* 4X Information Element */
#define DOT11_4X_ELE_ID     0xDD
#define DOT11_4X_MAX_LEN    64
#define DOT11_OUI_LEN       3
#define TI_OUI              {0x08,0x00,0x28}
#define WPA_IE_OUI          {0x00, 0x50, 0xf2}
#define EXC_OUI             {0x00, 0x40, 0x96}


#define _WlanTIcap_t(_) \
        _(TI_CAP_4X_CONCATENATION, =        1) \
        _(TI_CAP_4X_CONT_WINDOW, =          2) \
        _(TI_CAP_4X_CONT_WINDOW_COMBO, =    3) \
        _(TI_CAP_4X_TCP_ACK_EMUL, =         4) \
        _(TI_CAP_TRICK_PACKET_ERP, =        5)

PACKED_ENUM (WlanTIcap_t,_WlanTIcap_t);


PACKED_STRUCT( dot11_4X_t,

  dot11_eleHdr_t    hdr;
  UINT8             fourXCapabilities[DOT11_4X_MAX_LEN];
);

/* Action field structure
    used for extended management action such as spectrum management */ 

PACKED_STRUCT( dot11_ACTION_FIELD_t,

    UINT8   category;
    UINT8   action;
);


/* Management frames sub types */
typedef enum
{
    ASSOC_REQUEST       = 0,
    ASSOC_RESPONSE      = 1,
    RE_ASSOC_REQUEST    = 2,
    RE_ASSOC_RESPONSE   = 3,
    PROBE_REQUEST       = 4,
    PROBE_RESPONSE      = 5,
    BEACON              = 8,
    ATIM                = 9,
    DIS_ASSOC           = 10,
    AUTH                = 11,
    DE_AUTH             = 12,
    ACTION              = 13,
} dot11MgmtSubType_e;

/* Management frames element IDs */
typedef enum
{
    SSID_IE_ID                          = 0,
    SUPPORTED_RATES_IE_ID               = 1,
    FH_PARAMETER_SET_IE_ID              = 2,
    DS_PARAMETER_SET_IE_ID              = 3,
    CF_PARAMETER_SET_IE_ID              = 4,
    TIM_IE_ID                           = 5,
    IBSS_PARAMETER_SET_IE_ID            = 6,
    COUNTRY_IE_ID                       = 7,
    CHALLANGE_TEXT_IE_ID                = 16,
    POWER_CONSTRAINT_IE_ID              = 32,
    TPC_REPORT_IE_ID                    = 35,
    CHANNEL_SWITCH_ANNOUNCEMENT_IE_ID   = 37,
    QUIET_IE_ID                         = 40,
    ERP_IE_ID                           = 42,
    QOS_CAPABILITY_IE_ID                = 46,   
    RSN_IE_ID                           = 48,
    EXT_SUPPORTED_RATES_IE_ID           = 50,
    EXC_EXT_1_IE_ID                     = 133,
    EXC_EXT_2_IE_ID                     = 149,  
    CELL_POWER_IE                       = 150, /*EXC*/
    WPA_IE_ID                           = 221,
    TI_4X_IE_ID                         = WPA_IE_ID

} dot11MgmtIeId_e;

/* Spectrum Management Action fields */
typedef enum
{
    MEASUREMENT_REQUEST             = 0,
    MEASUREMENT_REPORT              = 1,
    TPC_REQUEST                     = 2,
    TPC_REPORT                      = 3,
    CHANNEL_SWITCH_ANNOUNCEMENT     = 4,
} dot11ActionFrameTypes_e;

/* Category fields (such as apectrum management)*/
typedef enum
{
    CATAGORY_SPECTRUM_MANAGEMENT        = 0,
    CATAGORY_QOS                        = 1,
    WME_CATAGORY_QOS                    = 17,
    CATAGORY_SPECTRUM_MANAGEMENT_ERROR  = 128,
} dot11CategoryTypes_e;


/* management templates to set to the HAL */

PACKED_STRUCT( probeReqTemplate_t,

    dot11_mgmtHeader_t  hdr;
    char                infoElements[sizeof( dot11_SSID_t ) + 
                                     sizeof( dot11_RATES_t ) +
                                     sizeof( dot11_RATES_t )    ];
);


PACKED_STRUCT( probeRspTemplate_t,

    dot11_mgmtHeader_t  hdr;
    UINT8               timeStamp[TIME_STAMP_LEN];
    UINT16              beaconInterval;
    UINT16              capabilities;
    char                infoElements[ sizeof( dot11_SSID_t ) + 
                                      sizeof( dot11_RATES_t ) +
                                      sizeof( dot11_RATES_t ) +
                                      sizeof( dot11_DS_PARAMS_t ) +
                                      sizeof( dot11_COUNTRY_t)      ];
);


PACKED_STRUCT( nullDataTemplate_t,

    dot11_mgmtHeader_t  hdr;
);


PACKED_STRUCT( psPollTemplate_t,

   dot11_PsPollFrameHeader_t   hdr;
);


PACKED_STRUCT( QosNullDataTemplate_t,

   dot11_header_t   hdr;
);

#pragma pack(1)
/* Traffic Stream Rate Set (TSRS) info-elements */
PACKED_STRUCT( dot11_TSRS_STA_IE_t,
    dot11_eleHdr_t  hdr;
    UINT8           OUI[3];
    UINT8           oui_type;
    UINT8           tsid;
    UINT8           tsNominalRate;
);

PACKED_STRUCT( dot11_TSRS_IE_t,
    dot11_eleHdr_t  hdr;
    UINT8           OUI[3];
    UINT8           oui_type;
    UINT8           tsid;
    UINT8           tsRates[8];
);

/* MSDU lifetime info-element */
PACKED_STRUCT( dot11_MSDU_LIFE_TIME_IE_t,
    dot11_eleHdr_t  hdr;
    UINT8           OUI[3];
    UINT8           oui_type;
    UINT8           tsid;
    UINT16          msduLifeTime;
);

PACKED_STRUCT( dot11_TS_METRICS_IE_t,
    dot11_eleHdr_t  hdr;
    UINT8           OUI[3];
    UINT8           oui_type;
    UINT8           tsid;
    UINT8           state;
    UINT16          measureInterval;
);

PACKED_STRUCT( EXCv4IEs_t,
    dot11_TSRS_IE_t             *trafficStreamParameter;
    dot11_MSDU_LIFE_TIME_IE_t   *edcaLifetimeParameter;
    dot11_TS_METRICS_IE_t       *tsMetrixParameter;
);
#pragma pack()

/* Disassociation frame structure */
#pragma pack(1)
PACKED_STRUCT( disAssoc_t,

    UINT16  reason;    
);
#pragma pack()

/* (Re)Association response frame structure */
#define ASSOC_RESP_FIXED_DATA_LEN 6
#pragma pack(1)
PACKED_STRUCT( assocRsp_t,

    UINT16          capabilities;      
    UINT16          status;    
    UINT16          aid;       
    dot11_RATES_t   *pRates;    
    dot11_RATES_t   *pExtRates;
    BOOL            useProtection;
    BOOL            ciscoIEPresent;
    preamble_e      barkerPreambleMode;
    BOOL            NonErpPresent;  
    dot11_4X_t                  *fourXParams;   
    dot11_WME_PARAM_t           *WMEParams;
    dot11_RSN_t                 *pRsnIe;
    UINT8                       rsnIeLen;
    dot11_QOS_CAPABILITY_IE_t   *QoSCapParameters;
    dot11_WME_TSPEC_IE_t        *tspecVoiceParameters;
    dot11_WME_TSPEC_IE_t        *tspecSignalParameters;
    EXCv4IEs_t                  excIEs[MAX_NUM_OF_AC];
);
#pragma pack()


/* Probe response frame structure */
/* Please notice, the order of fields in the beacon must be identical to the order of 
    field in the probe response. This is because of the parsing that is done by the site manager. */

/* In case the removing the PACKED_STRUCT for beacon_probeRsp_t it's possible to merge the 2 structures below */

#ifdef EXC_MODULE_INCLUDED

PACKED_STRUCT( beacon_probeRsp_t,

    char                        timestamp[TIME_STAMP_LEN];     
    UINT16                      beaconInerval;     
    UINT16                      capabilities;      
    dot11_SSID_t                *pSsid;    
    dot11_RATES_t               *pRates;
    dot11_COUNTRY_t             *country;
    dot11_POWER_CONSTRAINT_t    *powerConstraint;
    dot11_CHANNEL_SWITCH_t      *channelSwitch;
    dot11_QUIET_t               *quiet;
    dot11_TPC_REPORT_t          *TPCReport;
    dot11_CELL_TP_t             *cellTP;
    dot11_WME_PARAM_t           *WMEParams;

    dot11_RATES_t       *pExtRates;
    BOOL            useProtection;
    preamble_e      barkerPreambleMode;
    BOOL                NonErpPresent;  
    dot11_FH_PARAMS_t   *pFHParamsSet;     
    dot11_DS_PARAMS_t   *pDSParamsSet;     
    dot11_CF_PARAMS_t   *pCFParamsSet;     
    dot11_IBSS_PARAMS_t *pIBSSParamsSet;
    dot11_4X_t                  *fourXParams;           /* for probe response only */
    dot11_RSN_t         *pRsnIe;
    UINT8                rsnIeLen;
    dot11_QOS_CAPABILITY_IE_t   *QoSCapParameters;
    dot11_TIM_t                 *pTIM;                  /* for beacons only */
);

#else

PACKED_STRUCT( beacon_probeRsp_t,

    char                        timestamp[TIME_STAMP_LEN];     
    UINT16                      beaconInerval;     
    UINT16                      capabilities;      
    dot11_SSID_t                *pSsid;    
    dot11_RATES_t               *pRates;
    dot11_COUNTRY_t             *country;
    dot11_POWER_CONSTRAINT_t    *powerConstraint;
    dot11_CHANNEL_SWITCH_t      *channelSwitch;
    dot11_QUIET_t               *quiet;
    dot11_TPC_REPORT_t          *TPCReport;
    dot11_WME_PARAM_t           *WMEParams;
    dot11_RATES_t       *pExtRates;
    BOOL            useProtection;
    preamble_e      barkerPreambleMode;
    BOOL                NonErpPresent;  
    dot11_FH_PARAMS_t   *pFHParamsSet;     
    dot11_DS_PARAMS_t   *pDSParamsSet;     
    dot11_CF_PARAMS_t   *pCFParamsSet;     
    dot11_IBSS_PARAMS_t *pIBSSParamsSet;
    dot11_4X_t                  *fourXParams;           /* for probe response only */
    dot11_RSN_t         *pRsnIe;
    UINT8                rsnIeLen;
    dot11_QOS_CAPABILITY_IE_t   *QoSCapParameters;
    dot11_TIM_t                 *pTIM;                  /* for beacons only */
);

#endif

/* Authentication message frame structure */
#pragma pack(1)
PACKED_STRUCT( authMsg_t,

    UINT16              authAlgo;      
    UINT16              seqNum;    
    UINT16              status;    
    dot11_CHALLENGE_t   *pChallenge;       
);
#pragma pack()

/* DeAuthentication message frame structure */
#pragma pack(1)
PACKED_STRUCT( deAuth_t,

    UINT16  reason;    
);
#pragma pack()

/* Action message frame structure */
#pragma pack(1)
PACKED_STRUCT( action_t,

    UINT8   frameType;
    UINT8   category;
    UINT8   action;
);
#pragma pack()


/* TPCReport message frame structure */
#pragma pack(1)
PACKED_STRUCT( TPCReport_t,

    dot11_ACTION_FIELD_t    actionField;
    UINT8   dialogToken;
    dot11_TPC_REPORT_t  TPCReport;
);
#pragma pack()

/* Measurement Report message frame structure */
#pragma pack(1)
PACKED_STRUCT( MeasurementReportFrame_t,

    dot11_ACTION_FIELD_t    actionField;
    UINT8   dialogToken;
    dot11_MEASUREMENT_REPORT_t  measurementReportIE;
);
#pragma pack()

typedef enum 
{
    STATUS_SUCCESSFUL       		=   0,
    STATUS_UNSPECIFIED,      		
	STATUS_AUTH_REJECT,			
	STATUS_ASSOC_REJECT,			
    STATUS_SECURITY_FAILURE, 	
	STATUS_AP_DEAUTHENTICATE,	
	STATUS_AP_DISASSOCIATE,	
	STATUS_ROAMING_TRIGGER		

} mgmtStatus_e;

/* Used as a status code in case of STATUS_AUTH_REJECT or STATUS_ASSOC_REJECT that was not received at all */
#define STATUS_PACKET_REJ_TIMEOUT	0xFFFF

/* As defined in 802.11 spec section 7.3.1 - status codes for deAuth packet */
#define STATUS_CODE_802_1X_AUTHENTICATION_FAILED 23

/* map field included in measurement report IE (only in basic report) */
typedef enum
{
  DOT11_BSS_ONLY                    = (0x01),
  DOT11_OFDM_ONLY                   = (0x02),
  DOT11_RADAR_AND_UNIDENTIFIED      = (0x0C),
} dot11_Map_Sub_Field_e;


/* MACROS */
#define INRANGE(x,low,high)    (((x) >= (low)) && ((x) <= (high)))
#define OUTRANGE(x,low,high)   (((x) < (low)) || ((x) > (high)))

#define WLAN_4X_LEN_FIELD_LEN       2
#define WLAN_DA_FIELD_LEN           6
#define WLAN_DA_FIELD_OFFSET        16
#define WLAN_BSSID_FIELD_OFFSET     4
#define WLAN_SA_FIELD_OFFSET        10
#define WLAN_CONCAT_HDR_LEN     (WLAN_4X_LEN_FIELD_LEN + WLAN_DA_FIELD_LEN) /* 2+6 = 8 */
#define WLAN_CONCAT_HDR_OFFSET  (WLAN_HDR_LEN - WLAN_CONCAT_HDR_LEN) /* 24-8 = 16 */ 

/* 4X definitions */
#pragma pack(1)
PACKED_STRUCT( Wdrv4xHeader_t,

  UINT8         type;
  UINT8         headerLen;
  UINT16        txFlags;
);
#pragma pack()

#pragma pack(1)
PACKED_STRUCT( dot11_DataMsduHeader_t,

   dot11_header_t dot11Header;
   Wlan_LlcHeader_T  snapHeader;
);
#pragma pack()

#pragma pack(1)
PACKED_STRUCT( legacy_dot11_DataMsduHeader_t,

   legacy_dot11_header_t dot11Header;
   Wlan_LlcHeader_T  snapHeader;
);
#pragma pack()

#pragma pack(1)
PACKED_STRUCT( dot114xMsdu_t,

    legacy_dot11_DataMsduHeader_t msduHeader;
    Wdrv4xHeader_t  header4x;
);
#pragma pack()

#pragma pack(1)
PACKED_STRUCT( Wdrv4xConcatHeader_t,

  UINT16        len;
  macAddress_t  SaDa;
);
#pragma pack()

typedef enum 
{
    NOT_4X_MSDU    = -1,
    CONCATENATION  = 1,
    ACK_EMULATION  = 2,
    MANAGMENT_4X   = 3
} Wlan4XType_t;


#define WLAN_HEADER_TYPE_CONCATENATION 0x01
#define WLAN_CONCAT_HEADER_LEN 2

#define WLAN_4X_CONCAT_HDR_LEN   4

#define WLAN_4X_CONCAT_MORE_BIT   0x0001 


#endif   /* _802_11_INFO_DEFS_H */
