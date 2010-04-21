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
/*    MODULE:   txResult.h                                                 */
/*                                                                         */
/*    PURPOSE:  Handle packets Tx results upon Tx-complete from the FW.    */
/*                                                                         */
/***************************************************************************/
#ifndef _TX_RESULT_H_
#define _TX_RESULT_H_


#include "public_descriptors.h"



/* Callback function definition for Tx sendPacketComplete */
typedef void (* SendPacketCompleteCB_t)(TI_HANDLE CBObj, TxResultDescriptor_t *pTxResultInfo);

typedef enum
{
    TX_RESULT_STATE_IDLE,
    TX_RESULT_STATE_READING,
    TX_RESULT_STATE_WRITING1,
    TX_RESULT_STATE_WRITING2,
    TX_RESULT_STATE_EXIT
} TxResultState_e;

typedef enum
{
    TX_RESULT_NO_BUFFER = 0,
    TX_RESULT_ONE_BUFFER = 1,
    TX_RESULT_TWO_BUFFERS = 2
} TxResultNumOfBuffers_e;

typedef struct 
{ 
  UINT32 from; 
  UINT32 to; 
} TxResultEntry_t;

/* The TxResult module object. */
typedef struct
{

    TI_HANDLE               hOs;
    TI_HANDLE               hReport;
    TI_HANDLE               hTNETWIF;
    TI_HANDLE               hFwEvent;

    TI_STATUS               returnValue;                            /* used the return code to the FwEvent module */
    TxResultState_e         state;                                  /* current state of SM */
    
    /* use a struct to read buffers from the bus - used for extra bytes reserving */
    PADDING (TxResultDescriptor_t TxCmpltAttr[FW_TX_CMPLT_BLOCK_SIZE])   
																	/* The FW result table image from last read. */

    BOOL                    bSync;                                  /* indicate if we are in Synch bus or not */
    UINT32                  TxCmpltStartPointIterator;              /* Saves the last FW table index we got result-info from. */
    SendPacketCompleteCB_t  sendPacketCompleteCB;                   /* Tx-Complete callback function */
    TI_HANDLE               sendPacketCompleteHandle;               /* Tx-Complete callback function handle */
    UINT32                  txResultTableAddr;                      /* The HW Tx-Result Table address. */
    TxResultEntry_t         entry[2];                               /* address of start-end points of the new entries */
    TxResultNumOfBuffers_e  numOfBuffers;                           /* indicate how many buffers should be written to FW */
#ifdef TI_DBG
    UINT32                  txCompleteDepthHistogram[ FW_TX_CMPLT_BLOCK_SIZE + 1 ];
                                                                    /* The depth of the TX result array on INTR */
#endif
} txResultObj_t;



#endif  /* _TX_RESULT_H_  */
        

