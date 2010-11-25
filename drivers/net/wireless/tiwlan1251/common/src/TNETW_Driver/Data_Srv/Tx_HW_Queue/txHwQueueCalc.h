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
 *   MODULE:  txHwQueueCalc.h
 *
 *   PURPOSE: Tx HW blocks and fragmentation threshold calculations module internal definitions.
 * 
 ****************************************************************************/

#ifndef _TX_HW_QUEUE_MEM_CALC_H
#define _TX_HW_QUEUE_MEM_CALC_H


#include "whalParams.h"
#include "TNETW_Driver_types.h"
#include "802_11Defs.h"

#define HW_BLOCK_SIZE			252   /* The net size per HW block (without block header). */
#define RTS_FRAG_DATA_TIME      248

#define MIN_FRAG_THRESH			256

#define FCS_LENGTH						4
#define MAX_MSDU_SECURITY_LENGTH		16 /* RSN + MIC = 8 + 8 = 16 bytes (worst case - AES). */
#define MAX_MPDU_SECURITY_LENGTH		16 /* RSN + MIC = 8 + 8 = 16 bytes (worst case - AES). */
#define DIFF_HEADER_LENGTH_LEGACY_TO_QOS (WLAN_QOS_HDR_LEN - WLAN_HDR_LEN)
#define MAX_MPDU_HEADER_AND_SECURITY	(WLAN_QOS_HDR_LEN + MAX_MPDU_SECURITY_LENGTH)

#define OFDM_PLCP_HDR_MICROSECONDS      24
#define SHORT_PREAMBLE_MICROSECONDS     96
#define LONG_PREAMBLE_MICROSECONDS      192
#define B_SIFS_MICROSECONDS             10
#define OFDM_SIGNAL_EXT_MICROSECONDS    6
#define NONOFDM_SHORT_DURATION_MICROSECONDS  (B_SIFS_MICROSECONDS + SHORT_PREAMBLE_MICROSECONDS)
#define NONOFDM_LONG_DURATION_MICROSECONDS   (B_SIFS_MICROSECONDS + LONG_PREAMBLE_MICROSECONDS)



/* The ACK duration in air (in uSec) per Tx rate,
	 including MAC header + SIFS, and excluding PLCP header. */
const UINT8 TxMemCalcAckDurationTable[] =
{
	0,		/* Auto (not used in this module)*/
    B_SIFS_MICROSECONDS + 112,    /* 1 Mbps */
    B_SIFS_MICROSECONDS + 56,     /* 2 Mbps*/
    B_SIFS_MICROSECONDS + 20,     /* 5.5 Mbps*/
    B_SIFS_MICROSECONDS + 10,     /* 11 Mbps*/
    B_SIFS_MICROSECONDS + 4,      /* 22 Mbps*/
    B_SIFS_MICROSECONDS + 19,     /* 6 Mbps*/
    B_SIFS_MICROSECONDS + 12,     /* 9 Mbps*/
    B_SIFS_MICROSECONDS + 6,      /* 18 Mbps*/
    B_SIFS_MICROSECONDS + 4,      /* 22 Mbps*/
    B_SIFS_MICROSECONDS + 4,      /* 24 Mbps*/
    B_SIFS_MICROSECONDS + 4,      /* 36 Mbps*/
    B_SIFS_MICROSECONDS + 4,      /* 48 Mbps, standard says ack at 36 Mbps*/
    B_SIFS_MICROSECONDS + 4       /* 54 Mbps, standard says ack at 36 Mbps*/
};


/* Translate rate code to bit rate in Mbps (5.5M has a factor of 10). */
const UINT8 TxMemCalcRateValueTable[] =
{
	0,		/* Auto (not used in this module)*/
    1,		/* 1 Mbps */
    2,		/* 2 Mbps*/
    55,		/* 5.5 Mbp*/
    11,		/* 11 Mbps*/
    22,		/* 22 Mbps*/
    6,		/* 6 Mbps*/
    9,		/* 9 Mbps*/
    12,		/* 18 Mbps*/
    18,		/* 22 Mbps*/
    24,		/* 24 Mbps*/
    36,		/* 36 Mbps*/
    48,		/* 48 Mbps*/
    54		/* 54 Mbps*/
};



/* Module local functions. */

static UINT16 CalcFragThreshold(TxHwQueueObj_t *pTxHwQueue, txCtrlBlkEntry_t *pPktCtrlBlk);
static UINT16 GetTxOpFragThreshold(TxHwQueueObj_t *pTxHwQueue, txCtrlBlkEntry_t *pPktCtrlBlk, 
								   UINT16 txOpLimit, WlanParams_T *pWlanParams);
static rate_e ConvertRateTnetToDriver (UINT16 tnetRate);



#endif /* _TX_HW_QUEUE_MEM_CALC_H */


