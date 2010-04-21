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
/*    MODULE:   FwEvent.h                                                  */
/*    PURPOSE:  Handle firmware events.                                    */
/*                                                                         */
/***************************************************************************/
#ifndef _FW_EVENT_
#define _FW_EVENT_

#include "FwEvent_api.h"

#define NUM_OF_READ_REG_LOOPS      1

#ifdef TI_DBG
  #define MAX_EVENT_NUM            8
#else
  #define MAX_EVENT_NUM            6
#endif

typedef enum
{
    FW_EVENT_STATE_IDLE,    
    FW_EVENT_STATE_WAIT_BUS_I,
    FW_EVENT_STATE_WAIT_BUS_II,
    FW_EVENT_STATE_WAIT_MASK,   
    FW_EVENT_STATE_WAIT_UNMUX,
    FW_EVENT_STATE_WAIT_HINT_READ,
    FW_EVENT_STATE_WAIT_READ_COUNTERS,
    FW_EVENT_STATE_HANDLE_EVENT,
    FW_EVENT_STATE_WAIT_UNMASK
} FwEventState_e;


/* The FwEvent module object. */
typedef struct 
{
    FwEventState_e  FwEventState;           /* State machine state */
    UINT32          EventVector;            /* Read interrupt status vector */ 
    UINT32          EventMask;              /* Static interrupt event mask */
    UINT32          EventNum;               /* Event interrupt counter */
    UINT32          IntrState;              /* Interrupt state */
    BOOL            PendingEvent;           /* Pending event indicator */
    UINT32          LoopCounter;   

    /* The next 3 variables are used for a work around solution of mismatch between Fw & Driver events */
    UINT32          RxControlAddr;
    UINT32          uNumOfRxHandled;
    /* use a struct to read buffers from the bus - used for extra bytes reserving */
    PADDING (UINT32          uFwRxCounter)
    

    TI_HANDLE       hOs;                    /* OS handle */
    TI_HANDLE       hReport;                /* Report handle */
    TI_HANDLE       hTNETWIF;               /* TNETWIF handle */
    TI_HANDLE       hClient [MAX_EVENT_NUM];/* Array of client handles */

} FwEventObj_t; 

#endif  /* _FW_EVENT_ */
        
