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
 *   MODULE:  whalBus_Defs.h
 *   PURPOSE: Bus access component structures definitions
 *
 ****************************************************************************/

#ifndef _WHAL_BUS_DEFS_H
#define _WHAL_BUS_DEFS_H

#include "public_types.h"
#include "osTIType.h"
#include "ratesTypes.h"
#include "commonTypes.h"
#include "public_host_int.h"
#include "public_descriptors.h"

/* Typedefs */

typedef struct
{
    rxPacketType_e packetType;
    UINT8          rxLevel;
    INT8           rssi;
    UINT8          SNR;
    UINT8          band;
    UINT32         TimeStamp;
} rxXfer_Reserved_t;

typedef struct
{
    rxPacketType_e packetType;
    TI_STATUS     status;
    rate_e      Rate;   
    UINT8       SNR;
    INT8        Rssi;   
    UINT8       channel;
    UINT32      packetInfo;
    UINT8       band;
    UINT32      TimeStamp;
}Rx_attr_t;



/* Callback for rx packet */
typedef void (*packetReceiveCB_t)(TI_HANDLE hObj,
                                  TI_STATUS aStatus,
                                  const void *aFrame,
                                  UINT16 aLength,
                                  UINT32 aRate,
                                  UINT8  aChannel,
                                  UINT8 aRCPI,
                                  void *Reserved,
                                  UINT32 aFlags);

/* CallBack for Buffer request */
typedef const void *(*requestForBufferCB_t)(TI_HANDLE hObj, UINT16 aLength, UINT32 uEncryptionFlag);


/* Scan complete Callback - This routine is called from the HAL upon TNET scan complete */
typedef void (*scanCompleteCB_t)(TI_HANDLE hScanSrv, char* str, UINT32 strLen);

/* Disassociation sent - This function is called by the HAL's Tx to indicate that dissasociation frame has been sent.*/
typedef void (*disassocSentCB_t)(TI_HANDLE Hobj);

/* Incoming Info Callback */
typedef void (*InfoCB_t)(TI_HANDLE handle, char* buf, UINT32 bufSize);
/* Device Error Callback */
typedef void (*deviceErrorCB_t)(TI_HANDLE siteMgr);

/* Mac status Callback */
typedef void (*MacStatusCB_t)(TI_HANDLE handle, char* str , UINT32 strLen);

/* Health Report Callback */
typedef void (*HealthReportCB_t)(TI_HANDLE handle, char* str , UINT32 strLen);

/* Aci Indication Callback */
typedef void (*AciIndicationCB_t)(TI_HANDLE handle, char* str , UINT32 strLen);

/* Failure Event Callback */
typedef void (*failureEventCB_t)(TI_HANDLE handle, failureEvent_e failureEvent);

/*
 * --------------------------------------------------------------
 *                  DmaParams_T - Rx/Tx Queues and Bufffers params
 * --------------------------------------------------------------
 */


typedef struct
{
    int     NumTxQueues;
    int     NumRxQueues;
    UINT32  BlockSize;
    int     NumTxBlocks;
    int     NumRxBlocks;
    int     TxNumDesc[MAX_NUM_OF_TX_QUEUES];
    int     RxNumDesc;
    int     TxQPriority[MAX_NUM_OF_TX_QUEUES];
    int     RxQPriority;

    UINT16  NumStations;
    UINT8   RxQueue_Priority;
    UINT8   RxQueue_Type;

    UINT32  TraceBufferSize;
    BOOLEAN TraceBufferDoPrint;

    UINT8   rxMemBlkNumber;
    UINT8   txMinMemBlkNumber;

} DmaParams_T;


/*
 * --------------------------------------------------------------
 *                  Internal hal attributes
 * --------------------------------------------------------------
 */                  



typedef enum
{
    SW_DIVS_TX_RESET       = 0,     /* reset, i.e. invalid value */
    SW_DIVS_TX_ANT2        = BIT_2, /* defined to match TX_PING0 & CFG_ANT_SEL*/
    SW_DIVS_TX_ANT1        = BIT_3, /* defined to match TX_PING0 & CFG_ANT_SEL*/
    SW_DIVS_RX_W_LAST_TX   = BIT_4,
    SW_DIVS_TOGGLE_DISABLE = BIT_5,
    SW_DIVS_TOGGLE_COUNT_MASK = BIT_6+BIT_7
} SwAntDivs_enum;


#define CTRL_NO_TX_COMPLETE 0x40


#define    RX_PACKET_FLAGS_MATCH_RXADDR1    0x00000001
#define    RX_PACKET_FLAGS_GROUP_ADDR       0x00000002
#define    RX_PACKET_FLAGS_BCAST            0x00000004
#define    RX_PACKET_FLAGS_STAINTIM         0x00000008
#define    RX_PACKET_FLAGS_VIRTUAL_BM       0x00000010
#define    RX_PACKET_FLAGS_MATCH_SSID       0x00000020
#define    RX_PACKET_FLAGS_MATCH_BSSID      0x00000040
#define    RX_PACKET_FLAGS_ENCRYPTION       0x00030000
#define    RX_PACKET_FLAGS_MORE_PACKETS     0x00040000
#define    RX_PACKET_FLAGS_MEASURMENT       0x00080000

#define    RX_DESC_FLAGS_ENCRYPTION			8
#define    RX_PACKET_FLAGS_ENCRYPTION_SHIFT 16
#define    RX_PACKET_FLAGS_ENCRYPTION_SHIFT_FROM_DESC      (RX_PACKET_FLAGS_ENCRYPTION_SHIFT - RX_DESC_FLAGS_ENCRYPTION)

/* The next definitions are used to decide which encryption is used by the Rx flags */
#define    RX_FLAGS_NO_SECURITY 0  
#define	   RX_FLAGS_WEP			1
#define	   RX_FLAGS_TKIP		2
#define	   RX_FLAGS_AES			3

#define    RX_DESC_PACKETID_SHIFT 11
#define    RX_MAX_PACKET_ID      3


/* Cal backs */

/* Callback for rx compleate */


typedef void (*WhalSendPacketTranferCB_t)(TI_HANDLE hWhalTx,
                                          TI_STATUS TxStatus,
                                          UINT32    aPacketId,
                                          void      *reserved);

#endif /* _WHAL_BUS_DEFS_H */
