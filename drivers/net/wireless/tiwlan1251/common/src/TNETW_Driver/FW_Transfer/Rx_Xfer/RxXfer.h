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
 *   MODULE:  rxXfer.h
 *
 *   PURPOSE: Rx Xfer module header file.
 * 
 ****************************************************************************/

#ifndef _RX_XFER_H
#define _RX_XFER_H

#include "rxXfer_api.h"
#include "whalBus_Defs.h"
#include "TNETWIF.h"
#include "public_descriptors.h"

#define RX_DESCRIPTOR_SIZE (sizeof(RxIfDescriptor_t))

typedef struct 
{
    UINT32      numPacketsRead;
    UINT32      numBytesRead;
    UINT32      numPacketsDroppedNoMem;
    UINT32      numPacketsDroppedPacketIDMismatch;
    UINT32      numIrq0;
    UINT32      numIrq1;
    UINT32      numAck0;
    UINT32      numAck1;
} RxXferStats_T;

typedef enum
{
    RX_XFER_STATE_IDLE,
    RX_XFER_STATE_READING_HDR,
    RX_XFER_STATE_READING_PKT,
    RX_XFER_STATE_EXITING       /* Sending Ack to the FW at the end of the packet read */
} RxXferState_e;

typedef struct 
{
    TI_HANDLE               hOs;
    TI_HANDLE               hReport;
    TI_HANDLE               hTNETWIF;
    TI_HANDLE               hFwEvent;
    TI_HANDLE               hMemMgr;

    RxXferState_e           state;
    TI_STATUS               returnValue;

    /* address of the 2 buffers in the Double Buffer */
    UINT32                  doubleBuffer[2];
    UINT32                  currBuffer; 
    UINT32                  lastPacketId;
    UINT32                  rxPathStatus;

    /* use a struct to read buffers from the bus - used for extra bytes reserving */
    PADDING (RxIfDescriptor_t rxDescriptor)
           
    void                    *pPacketBuffer;
    packetReceiveCB_t       ReceivePacketCB;
    TI_HANDLE               ReceivePacketCB_handle;
    requestForBufferCB_t    RequestForBufferCB;
    TI_HANDLE               RequestForBufferCB_handle;  
    BOOL                    bSync;  			   /* indicate if we are in Synch bus or not */
    TI_STATUS               packetStatus; 	 	   /* OK unless an error occurred ... */

#ifdef TI_DBG
    RxXferStats_T           DbgStats;
#endif /* TI_DBG */

} RxXfer_t;


#endif /* _RX_XFER_H */
