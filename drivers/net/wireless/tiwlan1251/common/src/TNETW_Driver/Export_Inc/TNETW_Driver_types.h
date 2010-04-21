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
/*																			*/
/*	  MODULE:	TNETW-Driver												*/
/*																			*/
/*    PURPOSE:	Driver constants and type definitions						*/
/*																			*/
/***************************************************************************/

#ifndef TNETW_DRIVER_TYPES_H
#define TNETW_DRIVER_TYPES_H


#include "commonTypes.h"
#include "whalCtrl_api.h"
#include "public_descriptors.h"   /* The FW Tx-Descriptor and Tx-Result structures are used to ensure
									   FW compatibility and to avoid conversions in the data path. */
#if defined(HW_ACCESS_WSPI)
/* Read data offset */
#define TNETWIF_READ_OFFSET_BYTES  8

/* Write data offset */
#define TNETWIF_WRITE_OFFSET_BYTES 4
#else
#define TNETWIF_READ_OFFSET_BYTES  0
#define TNETWIF_WRITE_OFFSET_BYTES 0
#endif
/* Tx packet offset before the actual data */
#define TX_TOTAL_OFFSET_BEFORE_DATA (TNETWIF_WRITE_OFFSET_BYTES + sizeof(DbTescriptor)) 

/* Tx packet Control-Block flags bit-mask. */
#define TX_CTRL_BLK_FLAGS_XFER_DONE_ISSUED		0x0001	/* Xfer-Done already issued to upper driver. */
#define TX_CTRL_BLK_FLAGS_TX_COMPLETE_ISSUED	0x0002	/* Tx-Complete already issued to upper driver. */


/* TNETW Driver Callback Module owner */
typedef enum
{
    TNETW_DRIVER_TX_XFER_OWNER			= 0x0100,
	TNETW_DRIVER_RX_XFER_OWNER			= 0x0200,
	TNETW_DRIVER_HAL_CTRL_OWNER			= 0x0300,
	TNETW_DRIVER_MAC_SERVICES_OWNER		= 0x0400,
    TNETW_DRIVER_TX_RESULT_OWNER		= 0x0500,
	TNETW_DRIVER_TWD_OWNER				= 0x0600,

    TNETW_DRIVER_LAST_OWNER				= TNETW_DRIVER_TWD_OWNER

}   TNETW_DRIVER_CB_ModuleOwner_e;


/* TxXfer MODULE Callbacks */
#define  TX_XFER_SEND_PKT_TRANSFER		0x01   
#define  TX_XFER_SEND_PKT_DEBUG         0x02

/* TxResult MODULE Callbacks */
#define  TX_RESULT_SEND_PKT_COMPLETE	0x01



/* TNETW Driver Callback ID */
typedef enum
{
	/* Internal Failure Event Callbacks */
	TNETW_DRIVER_EVENT_FAILURE					=   HAL_INTERNAL_EVENT_FAILURE,	

	/* MAC Services Event Callbacks */
	TNETW_DRIVER_EVENT_SCAN_COMPLETE			= TNETW_DRIVER_MAC_SERVICES_OWNER | HAL_EVENT_SCAN_CMPLT,	  /* WHAL HW EVENT MBOX */
	TNETW_DRIVER_EVENT_PS_MODE_COMPLETE			= TNETW_DRIVER_MAC_SERVICES_OWNER | HAL_EVENT_PS_REPORT,	  /* WHAL HW EVENT MBOX */
	TNETW_DRIVER_EVENT_MEASUREMENT_COMPLETE		= TNETW_DRIVER_MAC_SERVICES_OWNER | HAL_EVENT_MEASUREMENT_COMPLETE,	  /* WHAL HW EVENT MBOX */


	/* Ctrl Event Callbacks */
	TNETW_DRIVER_EVENT_MEASUREMENT_START		=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_MEASUREMENT_START, 
	TNETW_DRIVER_EVENT_CALIB_CMPLT				=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_CALIB_CMPLT, 
	TNETW_DRIVER_EVENT_RSSI_LEVEL				=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_RSSI_LEVEL, 
	TNETW_DRIVER_EVENT_PS_REPORT				=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_PS_REPORT,
	TNETW_DRIVER_EVENT_SYNCHRONIZATION_TIMEOUT	=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_SYNCHRONIZATION_TIMEOUT, 
	TNETW_DRIVER_EVENT_HEALTH_REPORT			=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_HEALTH_REPORT,
	TNETW_DRIVER_EVENT_ACI						=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_ACI, 
	TNETW_DRIVER_EVENT_DEBUG_MESSAGE			=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_DEBUG_MESSAGE, 
	TNETW_DRIVER_EVENT_MAC_STATUS				=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_MAC_STATUS,
	TNETW_DRIVER_EVENT_DISCONNECT_COMPLETE		=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_DISCONNECT_COMPLETE,
	TNETW_DRIVER_EVENT_JOIN_CMPLT				=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_JOIN_CMPLT,
	TNETW_DRIVER_EVENT_SWITCH_CHANNEL_CMPLT		=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_SWITCH_CHANNEL_CMPLT,
	TNETW_DRIVER_EVENT_BSS_LOSE					=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_BSS_LOSE,
	TNETW_DRIVER_EVENT_MAX_TX_RETRY				=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_MAX_TX_RETRY,
	TNETW_DRIVER_EVENT_AP_DISCOVERY_COMPLETE	=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_AP_DISCOVERY_COMPLETE,
	TNETW_DRIVER_EVENT_SPS_SCAN_CMPLT			=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_SPS_SCAN_CMPLT, 
	TNETW_DRIVER_EVENT_BSS_REGAIN				=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_BSS_REGAIN,
	TNETW_DRIVER_EVENT_RSSI_LEVEL_REGAIN		=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_RSSI_LEVEL_REGAIN, 
	TNETW_DRIVER_EVENT_BT_COEX_SENSE			=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_SOFT_GEMINI_SENSE,		
	TNETW_DRIVER_EVENT_BT_COEX_PROTECTIVE_MODE	=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_SOFT_GEMINI_PREDICTION,
	TNETW_DRIVER_EVENT_BT_COEX_AVALANCHE		=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_EVENT_SOFT_GEMINI_AVALANCHE,
	TNETW_DRIVER_COMMAND_COMPLETE				=  TNETW_DRIVER_HAL_CTRL_OWNER | HAL_INT_COMMAND_COMPLETE,	
	
	/* Tx Data Path Callbacks */
    TNETW_DRIVER_TX_XFER_SEND_PKT_TRANSFER		=  TNETW_DRIVER_TX_XFER_OWNER	| TX_XFER_SEND_PKT_TRANSFER,   /* TxXfer MODULE */
    TNETW_DRIVER_TX_RESULT_SEND_PKT_COMPLETE	=  TNETW_DRIVER_TX_RESULT_OWNER | TX_RESULT_SEND_PKT_COMPLETE, /* TxResult MODULE */
    TNETW_DRIVER_TX_XFER_SEND_PKT_DEBUG         =  TNETW_DRIVER_TX_XFER_OWNER   | TX_XFER_SEND_PKT_DEBUG,      /* TxXfer MODULE */

	/* Rx Data Path Callbacks */
    TNETW_DRIVER_RX_RECEIVE_PACKET				=  TNETW_DRIVER_RX_XFER_OWNER | HAL_INT_RECEIVE_PACKET,			/* WHAL RX MODULE */
    TNETW_DRIVER_RX_REQUEST_FOR_BUFFER			=  TNETW_DRIVER_RX_XFER_OWNER | HAL_INT_REQUEST_FOR_BUFFER,		/* WHAL RX MODULE */

}TnetwDrv_CB_ID_e;



/* TNETW Driver print functions codes. */
typedef enum
{
	TNETW_DRV_PRINT_TX_CTRL_BLK_TBL,
	TNETW_DRV_PRINT_TX_HW_QUEUE_INFO,
	TNETW_DRV_PRINT_TX_XFER_INFO,
	TNETW_DRV_PRINT_TX_RESULT_INFO,
	TNETW_DRV_CLEAR_TX_RESULT_INFO
}TnetwDrv_PrintInfoType_e;



/* Tx Control-Block Packet parameters that are not included in the Tx-descriptor. */
typedef struct
{
	const void *pFrame;		/* Points to the whole packet including Tx-Descriptor + MAC-Header + Data. */
	UINT32 packetId;		/* The packet ID used by the upper driver to identify it when called back. */
	UINT16 headerFrameCtrl; /* The Frame Control field from the MAC header. */
	UINT16 flags;			/* See TX_CTRL_BLK_FLAGS_xxxx above. */

#ifdef TI_DBG
	UINT32 dbgPktSeqNum;	/* Packets sequence counter per queue. */
#endif

} TxPktParams_t;


/* 
 * txCtrlBlkEntry_t:
 * =================
 * Contains the Tx packet parameters required for the Tx process, including
 *   the Tx descriptor and the attributes required for HW-queue calculations.
 * Allocated for each packet sent from the upper driver and freed upon Tx-complete.
 * The entry index is the descriptor-ID written in the descriptor and copied back in 
 *   the tx-complete results.
 */
typedef struct _txCtrlBlkEntry_t
{
	struct _txCtrlBlkEntry_t * pNextFreeEntry;	/* Pointer to the next free entry. */
	
	DbTescriptor  txDescriptor; /* The packet descriptor copied to the FW. */

	TxPktParams_t	txPktParams;  /* Per packet parameters not included in the descriptor. */

} txCtrlBlkEntry_t;


typedef void (*TnetDrv_callback_t)(TI_HANDLE CB_Handle); 

/* Data path constant parametrs */
#define  DP_RX_PACKET_RING_CHUNK_SIZE 1600
#define  DP_TX_PACKET_RING_CHUNK_SIZE 1600 
#define  DP_RX_PACKET_RING_CHUNK_NUM 2	
#define  DP_TX_PACKET_RING_CHUNK_NUM 2
#define  DP_TX_COMPLETE_TIME_OUT 20

#endif  /* TNETW_DRIVER_TYPES_H */

