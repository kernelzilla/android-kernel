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

/** \file GWSI_Synchronizer.h
 *  \brief GWSI Synchronizer include file
 *
 *  \see GWSI_Synchronizer.c
 */


/***************************************************************************/
/*                                                                          */
/*    MODULE:   GWSI_Synchronizer.h                                         */
/*    PURPOSE:  GWSI  Synchronizer include file                         */
/*                                                                          */
/***************************************************************************/


#ifndef SYNCHRONIZER_H
#define SYNCHRONIZER_H

#include "TNETWIF.h"
#include "TNETWArbSM.h"
#include "TNETWArb_buffer.h"
#include "osTIType.h"  


#define TODO_LIST_INDEX                     0
#define RUNNING_LIST_INDEX                  1
#define NUM_OF_TNETWARB_QUEUES              2


#define TNETWARB_IS_EVENT_PENDING       0x100

/* These defines should be closed by default. They may be open for debug purposes */

typedef struct TNETWARB_INSTANCE_struct
{
    TI_HANDLE   instance_handle;    /* The handle of the module to be used when calling back the client */
    TNETWIF_callback_t  instance_callback;  /* The handler of the module to be used when calling back the client */
    UINT8       module_id;          /* The module_id of the Client */
    UINT8       priority;           /* The priority of the client instance */
} TNETWARB_INSTANCE_T;

#define TNETWARB_INSTANCE_SIZE sizeof(TNETWARB_INSTANCE_T)

typedef struct TNETWArbStat_t_
{
    UINT32             uStart;
    UINT32             uRestart;
    UINT32             uFinish; 
} TNETWArbStat_t;


/**************************** TNETWARB_CB ********************************/
/****************************************************************************/
typedef struct T_TNETWARB_CB
{
    TI_HANDLE           hOs;
    TI_HANDLE           hReport;
    TI_HANDLE           hTNETWArbSM;
    UINT8               TNETWArb_Client_Instance_Array[NUM_OF_TNETWIF_MODULES] [BUFFER_HDR_SIZE+TNETWARB_INSTANCE_SIZE];
    TNETWARB_INSTANCE_T *TNETWArb_Running_instance;
    TNETWARB_INSTANCE_T *pDefInst;
    BUFFER_Q            TNETWArbiter_Queues[NUM_OF_TNETWARB_QUEUES];
    TnetwArbSMEvents_e  event_to_dispatch; /* This is used after the client callback has potentially called itself a FOINISH or RESTART API 
                                              Then we remmember the Event to be sent to the TNETW Arbiter State Machine so we prevent nested entrance in the TNETW ARBITER MODULE */

  #ifdef TI_DBG
    TNETWArbStat_t      stat;
  #endif
} TNETWArb_t;




#define TNETWArb_is_idle(pTnetwArb_cb)              (!pTnetwArb_cb->TNETWArbiter_Queues[RUNNING_LIST_INDEX].count)
#define TNETWArb_is_running(pTnetwArb_cb)           (pTnetwArb_cb->TNETWArbiter_Queues[RUNNING_LIST_INDEX].count)



/*******************************************************************************/
/*    Each new client shall register itself to the synchronizer by inserting here 
      its start instance                                                        */
/*    The "extern" here is anyway to explicite to the user that this fucntion is not
      implemented in the synchronizer but in the client                         */
/*******************************************************************************/


/************************************************************************
 API for the TNETWIF to handle the running module's Callbacks.
************************************************************************/
TI_STATUS TNETWArb_CallClientCallback (TI_HANDLE hTNETWArb);
TI_STATUS TNETWArb_CallTxnCb (TI_HANDLE hTNETWArb);
void TNETWArb_TxnCb (TI_HANDLE hTNETWArb, UINT8 module_id, TI_STATUS status);
void TNETWArb_PrintStat (TI_HANDLE hTNETWArb);


/* API for external module in charge of creating the gwsi_synchronizer initialization */
TI_HANDLE TNETWArb_Init (TI_HANDLE hOs);
void TNETWArb_Config (TI_HANDLE hTNETWArb,TI_HANDLE hReport,TI_HANDLE hELPCtrl);
TI_STATUS TNETWArb_Destroy (TI_HANDLE hTNETWArb);
TI_STATUS TNETWArb_Recovery(TI_HANDLE hTNETWArb, TI_HANDLE hELPCtrl);


/************************************************************************
 API for the Client to send an event to the Synchronizer : It can be :
            EV_REG_ID (PERFORM_IMMEDIATE or start time for future use)
            EV_FINISH_ID (To be called by the client when if finishes its State Machine
************************************************************************/
TI_STATUS TNETWArb_Start (TI_HANDLE hTNETWArb,UINT8 module_id,TI_HANDLE ClientCallBack_Handle,TNETWIF_callback_t ClientCallBack_Func);
TI_STATUS TNETWArb_Restart (TI_HANDLE hTNETWArb, UINT8 module_id,TI_HANDLE CallBAck_Handle,TNETWIF_callback_t CallBack_Func);
TI_STATUS TNETWArb_Finish (TI_HANDLE hTNETWArb, UINT8 module_id,TI_HANDLE CallBAck_Handle,TNETWIF_callback_t CallBack_Func);

/************************************************************************
 API for the Client to register its handle 
************************************************************************/
void TNETWArb_register_handler(TI_HANDLE hTNETWArb,UINT8 module_id,TNETWIF_callback_t module_CB_Func,TI_HANDLE module_handle);


#endif /* SYNCHRONIZER_H */
