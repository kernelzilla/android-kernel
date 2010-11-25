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
 *   MODULE:  EventMbox.c : event Mail Box
 *   PURPOSE: Handle any event interrupt from the FW 
 *
 ****************************************************************************/

#include "commonTypes.h"
#include "FwEvent_api.h"
#include "whalBus_Api.h"
#include "eventMbox_api.h"
#include "whalCtrl_api.h"
#include "TNETWIF.h"


/*******************************************************************************
 *
 *                               Macros                                   
 *
 *******************************************************************************/
#define IS_EVENT_BIT_ON(EvVector,EvBit)  (  (EvBit) == ( (EvVector) &  (EvBit) ) )
#define IS_EVENT_BIT_OFF(EvVector,EvBit) ( ~(EvBit) == ( (EvVector) | ~(EvBit) ) )

#define SET_EVENT_BIT(EvVector,EvBit)    ( (EvVector) |=  (EvBit) ) 
#define CLEAR_EVENT_BIT(EvVector,EvBit)  ( (EvVector) &= ~(EvBit) )

#define EVENT_REPORT_SIZE 80

#define EVMBX_DBG 1


/*****************************************************************************
 **         Enumerations                                                    **
 *****************************************************************************/

typedef enum
{
    EVENT_MBOX_STATE_IDLE,
    EVENT_MBOX_STATE_READ_BUF,
    EVENT_MBOX_STATE_ACK_EVENT
} EventMboxState_e;


/*****************************************************************************
 **         Structures                                                      **
 *****************************************************************************/

/*
 * Definition for the Event Table
 */
typedef struct 
{
    /* Event bit mask */
    UINT32              bitMask;
    /* Event trace string */
    char*               str; 
    /* Event data length */  
    UINT8               dataLen;
} EventEntry_t;


/*
 * Event callback structure
 */
typedef struct
{ 
    /* Event callback function */
    void*               fCb;
    /* Event callback module handle */
    TI_HANDLE           hCb;
    /* Event data offset */
    UINT8*              pDataOffs;
    /* Event callback counter */
    UINT32              uCount;
} EventCB_t;


/*
 *  Event Mailbox object
 */
typedef struct 
{
    /* Offset for event A (0) or event B (1) */
    UINT32              offset[2];        
    /* 0 or 1 according to event A or B */
    UINT32              currentEvent;           
    /* Event mail box state machine state */
    EventMboxState_e    state;
    /* Return value */
    TI_STATUS           returnValue;
    /* Init complete flag */
    BOOL                bInitComplete;
    /* Indicate if we are in synchronous bus or not */
    BOOL                bSync;  
    /* Callback table */
    EventCB_t           cbTable[MAX_NUM_OF_EVENT];
    /* Use a struct to read buffers from the bus - used for extra bytes reserving */
    PADDING (EventMailBox_t  CompoundEvent)

    /* Handles */
    TI_HANDLE           hFwEvent; 
    TI_HANDLE           hTNETWIF;
    TI_HANDLE           hOs;
    TI_HANDLE           hReport;
    TI_HANDLE           hWhalCtrl;
#ifdef TI_DBG
    /* Count the compound event */
    UINT32              uCompounEvCount; 
    /* Count the total number of event sending in the compound */
    UINT32              uTotalEvCount;   
#endif /* TI_DBG */

    TNETWIF_callback_t  fCb;
    TI_HANDLE           hCb;
} EventMbox_t;

/********************************************************************************/
/*                      Internal functions prototypes.                          */
/********************************************************************************/

static void eventMbox_HandleEvent  (TI_HANDLE hEventMbox);
static void eventMbox_StateMachine (TI_HANDLE hEventMbox, UINT8 module_id, TI_STATUS status);
static void eventMbox_InitCbTable  (TI_HANDLE hEventMbox);


static const EventEntry_t eventTable [MAX_NUM_OF_EVENT] =
{   
/*==================================================================================
|                                                                                  |
|                                     EVENT TABLE                                  |
|                                                                                  |
 ===================================================================================
| Id  |     Event Mask Bit                    |   Event String            | Length |
 ===================================================================================*/

/*0*/ { MEASUREMENT_START_EVENT_ID,             "MEASUREMENT START "      , 0},       
/*1*/ { SCAN_COMPLETE_EVENT_ID ,                "SCAN CMPLT "             , 3},                          
/*2*/ { CALIBRATION_COMPLETE_EVENT_ID,          "CALIB CMPLT "            , 0},                                           
/*3*/ { ROAMING_TRIGGER_LOW_RSSI_EVENT_ID ,     "RSSI LEVEL "             , 1},      
/*4*/ { PS_REPORT_EVENT_ID,                     "PS_REPORT "              , 1},
/*5*/ { SYNCHRONIZATION_TIMEOUT_EVENT_ID,       "SYNCHRONIZATION TIMEOUT ", 0}, 
/*6*/ { HEALTH_REPORT_EVENT_ID,                 "HEALTH REPORT "          , 2},  
/*7*/ { ACI_DETECTION_EVENT_ID ,                "ACI INDICATION "         , 2},     
/*8*/ { DEBUG_REPORT_EVENT_ID,                  "DEBUG REPORT "           , 8},   
/*9*/ { MAC_STATUS_EVENT_ID,                    "MAC STATUS "             , 8},    
/*10*/{ DISCONNECT_EVENT_COMPLETE_ID,           "DISCONNECT COMPLETE "    , 0},
/*11*/{ JOIN_EVENT_COMPLETE_ID,                 "JOIN CMPLT "             , 0},
/*12*/{ CHANNEL_SWITCH_COMPLETE_EVENT_ID,       "SWITCH CHANNEL CMPLT "   , 0},
/*13*/{ BSS_LOSE_EVENT_ID,                      "BSS LOST "               , 0},
/*14*/{ ROAMING_TRIGGER_MAX_TX_RETRY_EVENT_ID,  "MAX TX RETRY "           , 0},
/*15*/{ MEASUREMENT_COMPLETE_EVENT_ID,          "BSS LOSE "               , 0},
/*16*/{ AP_DISCOVERY_COMPLETE_EVENT_ID,         "MAX TX RETRY "           , 0},
/*17*/{ SCHEDULED_SCAN_COMPLETE_EVENT_ID,       "SPS SCAN CMPLT "         , 3},
/*18*/{ REGAINED_BSS_EVENT_ID,                  "REGAINED BSS "           , 0},
/*19*/{ ROAMING_TRIGGER_REGAINED_RSSI_EVENT_ID, "REGAINED RSSI "          , 1},
/*20*/{ ROAMING_TRIGGER_LOW_SNR_EVENT_ID,       "LOW SNR "                , 1},
/*21*/{ SOFT_GEMINI_SENSE_EVENT_ID,             "SOFT GEMINI SENSE "      , 1},
/*22*/{ SOFT_GEMINI_PREDICTION_EVENT_ID,        "SOFT GEMINI PREDICTION " , 1},
/*23*/{ SOFT_GEMINI_AVALANCHE_EVENT_ID,         "SOFT GEMINI AVALANCHE "  , 0},
/*24*/{ PLT_RX_CALIBRATION_COMPLETE_EVENT_ID,   "PLT RX CALIBR. COMPLETE ", 0},
/*25*/{ PSPOLL_DELIVERY_FAILURE_EVENT_ID,       "PS-POLL DELIVERY FAILURE", 0},
/*26*/{ RESET_BSS_EVENT_ID,                     "EVENT RESET BSS "        , 0},
/*27*/{ EVENT_MBOX_ALL_EVENT_ID,                "ALL EVENTS "             , 0}, 

};

/********************************************************************************/
/*                   functions implementation                                   */
/********************************************************************************/


/****************************************************************************
 *                      eventMbox_InitCbTable()
 ****************************************************************************
 * DESCRIPTION: Initialization of callback table
 * 
 * INPUTS:  hEventMbox       eventMbox module handle
 *
 * RETURNS: none
 ****************************************************************************/
void eventMbox_InitCbTable (TI_HANDLE hEventMbox)
{
    EventMbox_t *pEventMbox = (EventMbox_t *)hEventMbox;
    UINT32       EvID;

    for (EvID = 0; EvID < MAX_NUM_OF_EVENT; EvID++)
    {
        pEventMbox->cbTable[EvID].pDataOffs = (UINT8*)&pEventMbox->CompoundEvent;
    }

    pEventMbox->cbTable[ 3].pDataOffs += offsetof (EventMailBox_t, averageRssiLevel);
    pEventMbox->cbTable[ 4].pDataOffs += offsetof (EventMailBox_t, psStatus);
    pEventMbox->cbTable[ 6].pDataOffs += offsetof (EventMailBox_t, healthReport);
    pEventMbox->cbTable[ 7].pDataOffs += offsetof (EventMailBox_t, badFFTCorrelationCounter);
    pEventMbox->cbTable[ 8].pDataOffs += offsetof (EventMailBox_t, debugReport);
    pEventMbox->cbTable[ 9].pDataOffs += offsetof (EventMailBox_t, consFcsErrCnt);
    pEventMbox->cbTable[17].pDataOffs += offsetof (EventMailBox_t, scheduledScanStatus);
    pEventMbox->cbTable[19].pDataOffs += offsetof (EventMailBox_t, averageRssiLevel);
    pEventMbox->cbTable[20].pDataOffs += offsetof (EventMailBox_t, averageSNRLevel);
    pEventMbox->cbTable[21].pDataOffs += offsetof (EventMailBox_t, softGeminiSenseInfo);
    pEventMbox->cbTable[22].pDataOffs += offsetof (EventMailBox_t, softGeminiProtectiveInfo);
}


/****************************************************************************
 *                      eventMbox_Create()
 ****************************************************************************
 * DESCRIPTION: Create the Bus Access mailbox object
 * 
 * RETURNS: The Created object
 ****************************************************************************/
TI_HANDLE eventMbox_Create (TI_HANDLE hOs)
{
    EventMbox_t *pEventMbox;

    pEventMbox = os_memoryAlloc (hOs, sizeof(EventMbox_t));
    if (pEventMbox == NULL)
    {
        WLAN_OS_REPORT (("eventMbox_Create: Error creating EventMbox object\n"));
        return NULL;
    }

    os_memoryZero (hOs, pEventMbox, sizeof(EventMbox_t));

    pEventMbox->hOs = hOs;
    pEventMbox->CompoundEvent.eventsMask = EVENT_MBOX_ALL_EVENT_ID;

    return (TI_HANDLE)pEventMbox;
}


/****************************************************************************
 *                      eventMbox_Destroy()
 ****************************************************************************
 * DESCRIPTION: Destroy the object 
 *
 ****************************************************************************/
void eventMbox_Destroy (TI_HANDLE hEventMbox)
{
    EventMbox_t *pEventMbox = (EventMbox_t *)hEventMbox;

    if (pEventMbox)
    {
        os_memoryFree (pEventMbox->hOs, pEventMbox, sizeof(EventMbox_t));
    }
}


/****************************************************************************
 *                      eventMbox_Config()
 ****************************************************************************
 * DESCRIPTION: Configure the object 
 * 
 * INPUTS: pEventMbox   this
 *         pHwIntr          Interrupt Object object
 *         hReport          Report Object
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
void eventMbox_Config (TI_HANDLE hEventMbox, TI_HANDLE hTNETWIF, TI_HANDLE hHwIntr, 
                       TI_HANDLE hReport,    TI_HANDLE hFwEvent, TI_HANDLE hWhalCtrl)
{
    EventMbox_t *pEventMbox = (EventMbox_t *)hEventMbox;

    pEventMbox->hTNETWIF = hTNETWIF;
    pEventMbox->hReport = hReport;
    pEventMbox->hFwEvent = hFwEvent;
    pEventMbox->hWhalCtrl = hWhalCtrl;
    pEventMbox->hReport = hReport;

    pEventMbox->state = EVENT_MBOX_STATE_IDLE;   
    pEventMbox->bInitComplete = FALSE;
    pEventMbox->CompoundEvent.eventsVector = 0;
    pEventMbox->currentEvent = 0;

    /* 
     * NOTE: don't set eventsMask = 0xffffffff;
     *       its value is used after recovery has finished
     */
   
    /* Init callback table */
    eventMbox_InitCbTable (hEventMbox); 
}


/****************************************************************************
 *                      eventMbox_InitComplete()
 ****************************************************************************
 * DESCRIPTION: ReConfigure the object, Send the Mask Vector to the FW 
 * 
 * INPUTS:  pEventMbox      this
 * 
 * RETURNS: 
 ****************************************************************************/
void eventMbox_InitComplete (TI_HANDLE hEventMbox)
{
    EventMbox_t *pEventMbox = (EventMbox_t *)hEventMbox;
    
    WLAN_REPORT_INFORMATION(pEventMbox->hReport, EVENT_MBOX_MODULE_LOG,
        ("eventMbox_InitComplete: mask=0x%x\n", pEventMbox->CompoundEvent.eventsMask));

    pEventMbox->bInitComplete = TRUE;

    /* Enable Events Interrupts */
    FwEvent_Enable (pEventMbox->hFwEvent, ACX_INTR_EVENT_A);
    FwEvent_Enable (pEventMbox->hFwEvent, ACX_INTR_EVENT_B);

    whalCtrl_SetInfoElemEventMask (pEventMbox->hWhalCtrl, pEventMbox->CompoundEvent.eventsMask);
}

/****************************************************************************
 *                      eventMbox_Stop()
 ****************************************************************************
 * DESCRIPTION:	Stop the object while recovery until Init Complete.
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int eventMbox_Stop(TI_HANDLE hEventMbox)
{
	EventMbox_t *pEventMbox = (EventMbox_t *)hEventMbox;

	pEventMbox->state = EVENT_MBOX_STATE_IDLE;   
	pEventMbox->CompoundEvent.eventsVector = 0;
	pEventMbox->currentEvent = 0;
	pEventMbox->bInitComplete = FALSE;

	return OK;
}


/****************************************************************************
 *                      eventMbox_ConfigCb()
 ****************************************************************************
 * DESCRIPTION:Read the SRAM Event mailbox address callback
 * 
 * RETURNS: None
 ****************************************************************************/
static void eventMbox_ConfigCb (TI_HANDLE hEventMbox, UINT8 module_id, TI_STATUS status)
{
    EventMbox_t *pEventMbox = (EventMbox_t *)hEventMbox;

    pEventMbox->offset[1] = pEventMbox->offset[0] + sizeof(EventMailBox_t); 
 
    WLAN_REPORT_INIT (pEventMbox->hReport, EVENT_MBOX_MODULE_LOG,  
                      ("eventMbox_ConfigCb: event A offset=0x%x, event B offset=0x%x, sizeof=%d\n",
                      pEventMbox->offset[0], pEventMbox->offset[1], sizeof(EventMailBox_t)));

    /* Call upper layer callback */
    pEventMbox->fCb (pEventMbox->hCb, module_id, OK);
}


/****************************************************************************
 *                      eventMbox_ConfigHw()
 ****************************************************************************
 * DESCRIPTION:Read the SRAM Event mailbox address 
 * 
 * RETURNS: None
 ****************************************************************************/
TI_STATUS eventMbox_ConfigHw (TI_HANDLE hEventMbox, UINT8 module_id, fnotify_t fCb, TI_HANDLE hCb)
{
    EventMbox_t *pEventMbox = (EventMbox_t *)hEventMbox;
    TI_STATUS    status;

    pEventMbox->fCb = (TNETWIF_callback_t)fCb;
    pEventMbox->hCb = hCb;

    /* 
     * Get the event mailbox pointer
     */
    status = TNETWIF_ReadRegOpt (pEventMbox->hTNETWIF, 
                                 REG_EVENT_MAILBOX_PTR,
                                 &pEventMbox->offset[0],
                                 module_id, 
                                 eventMbox_ConfigCb,
                                 hEventMbox);

    if (status == TNETWIF_COMPLETE)
    {
        pEventMbox->offset[1] = pEventMbox->offset[0] + sizeof(EventMailBox_t);  

        WLAN_REPORT_INIT (pEventMbox->hReport, EVENT_MBOX_MODULE_LOG,  
                          ("eventMbox_ConfigHw: event A offset=0x%x, event B offset=0x%x, sizeof=%d\n",
                          pEventMbox->offset[0], pEventMbox->offset[1], sizeof(EventMailBox_t)));
    }
    
    return status;
}


/****************************************************************************
 *                      eventMbox_RegisterEventCB()
 ****************************************************************************
 * DESCRIPTION: register callback function for Events
 *
 * INPUTS:  EvID    Event ID
 *          fCb     Call Back
 *          hCb     Call Back Handle
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int eventMbox_RegisterEventCB (TI_HANDLE hEventMbox, UINT32 EvID, void* fCb, TI_HANDLE hCb)                                    
{
    EventMbox_t *pEventMbox = (EventMbox_t *)hEventMbox;
        
    if (fCb == NULL || hCb == NULL)
    { 
        WLAN_REPORT_ERROR (pEventMbox->hReport, EVENT_MBOX_MODULE_LOG, ("eventMbox_RegisterEventCB: NULL parameters\n"));
        return NOK;
    }

    if (EvID >= HAL_EVENT_ALL)
    {
        WLAN_REPORT_ERROR(pEventMbox->hReport, EVENT_MBOX_MODULE_LOG, ("eventMbox_RegisterEventCB: invalid ID\n"));
        return NOK;
    }

    pEventMbox->cbTable[EvID].fCb = fCb;
    pEventMbox->cbTable[EvID].hCb = hCb;

    WLAN_REPORT_INFORMATION(pEventMbox->hReport, EVENT_MBOX_MODULE_LOG,
        ("eventMbox_RegisterEventCB: EVENT %s has registered\n", eventTable[EvID].str));

    return OK;
}


/****************************************************************************
 *                      eventMbox_EvMask()
 ****************************************************************************
 * DESCRIPTION: The function Mask the Event in the Local Mask Vector 
 *              And in the FW Mask Vector
 *
 * INPUTS:  Evid:   The Event ID
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int eventMbox_EvMask (TI_HANDLE hEventMbox, UINT32 EvID)
{
    EventMbox_t *pEventMbox = (EventMbox_t *)hEventMbox;
    UINT32      *pEvMask = (UINT32*)&pEventMbox->CompoundEvent.eventsMask;

    if (EvID >= HAL_EVENT_ALL)
    {
        WLAN_REPORT_ERROR(pEventMbox->hReport, EVENT_MBOX_MODULE_LOG, ("eventMbox_EvMask: invalid ID\n"));
        return NOK;
    }

    *pEvMask |= eventTable[EvID].bitMask;

    WLAN_REPORT_INFORMATION(pEventMbox->hReport, EVENT_MBOX_MODULE_LOG,
        ("eventMbox_EvMask: EVENT %s is masked\n", eventTable[EvID].str));

    if (pEventMbox->bInitComplete == TRUE)
    {
        whalCtrl_SetInfoElemEventMask (pEventMbox->hWhalCtrl, *pEvMask);
    }

    return OK;
}


/****************************************************************************
 *                      eventMbox_EvUnMask()
 ****************************************************************************
 * DESCRIPTION: The function UnMask the Event in the Local Mask Vector 
 *              And in the FW Mask Vector
 *
 * INPUTS:  Evid:   The Event ID
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int eventMbox_EvUnMask (TI_HANDLE hEventMbox, UINT32 EvID)
{
    EventMbox_t *pEventMbox = (EventMbox_t *)hEventMbox;
    UINT32      *pEvMask = (UINT32*)&pEventMbox->CompoundEvent.eventsMask;
    
    if (EvID >= HAL_EVENT_ALL)
    {
        WLAN_REPORT_ERROR(pEventMbox->hReport, EVENT_MBOX_MODULE_LOG, ("eventMbox_EvUnMask: invalid ID\n"));
        return NOK;
    }

    *pEvMask &= ~eventTable[EvID].bitMask;

    WLAN_REPORT_INFORMATION (pEventMbox->hReport, EVENT_MBOX_MODULE_LOG,
        ("eventMbox_EvUnMask: EVENT %s is unmasked\n", eventTable[EvID].str));

    if (pEventMbox->bInitComplete == TRUE)
    {
         whalCtrl_SetInfoElemEventMask (pEventMbox->hWhalCtrl, *pEvMask);
    }

    return OK;
}


/****************************************************************************
 *                      eventMbox_Event()
 *****************************************************************************  
 * DESCRIPTION: Called when Event A or B interrupt occur
 * 
 * INPUTS:  hEventMbox - The object
 *
 * RETURNS: TNETWIF_PENDING in case of Async and TNETWIF_OK on Sync 
 *
 *****************************************************************************/
TI_STATUS eventMbox_Event (TI_HANDLE hEventMbox)
{
    EventMbox_t *pEventMbox = (EventMbox_t *)hEventMbox;
    
    /* Assume the driver is synchronous until opposite is proven */
    pEventMbox->bSync = TRUE;

    eventMbox_StateMachine (pEventMbox, FW_EVENT_MODULE_ID, OK);

    return pEventMbox->returnValue;
}


/********************************************************************************/
/*                      Internal functions implementation.                      */
/********************************************************************************/

/****************************************************************************
 *                      eventMbox_StateMachine()
 ****************************************************************************
 * DESCRIPTION: Manage the EventMbox state machine 
 *
 *              The SM is running one event at a time (A or B) .
 *              The order of the states is always the same: IDLE --> READ_BUF --> ACK_EVENT
 *              The difference is whether we are using Synch or Asynch API.
 *              In the Synch case (SDIO) we are looping in the while-loop till we return to IDLE, and we return
 *              to FwEvent module a TNETWIF_OK status.
 *              In the Asynch case we use the SM CB to return to the SM after each Asynch call
 *              (In that case the return status is TNETWIF_PENDING, and we are waiting for the CB).
 *              In the Asynch case the FwEvent module gets TNETWIF_PENDING in return, and waits for 
 *              the FwEvent_EventComplete() call in order to move the FwEvent SM.
 * 
 * INPUTS:  hFwEvent - The object
 *          module_id - not used (for CB API only)
 *          status    - not used (for CB API only) 
 *      
 * OUTPUT:  None
 * 
 * RETURNS: TNETWIF_PENDING in case of Async and TNETWIF_OK on Sync 
 ****************************************************************************/
static void eventMbox_StateMachine (TI_HANDLE hEventMbox, UINT8 module_id, TI_STATUS status)
{
    EventMbox_t *pEventMbox = (EventMbox_t *)hEventMbox;

    pEventMbox->returnValue = OK;

    while (pEventMbox->returnValue != TNETWIF_PENDING)
    {
        switch (pEventMbox->state)
        {
            case EVENT_MBOX_STATE_IDLE:

                pEventMbox->returnValue = TNETWIF_ReadMemOpt (pEventMbox->hTNETWIF,
                                                              pEventMbox->offset[pEventMbox->currentEvent],
                                                              PADREAD (&pEventMbox->CompoundEvent),
                                                              EVENT_REPORT_SIZE,
                                                              FW_EVENT_MODULE_ID,
                                                              eventMbox_StateMachine,
                                                              hEventMbox);

                pEventMbox->state = EVENT_MBOX_STATE_READ_BUF;
            
                break;

            case EVENT_MBOX_STATE_READ_BUF:

                /* Notify The appropriate layer about the incoming event */
                eventMbox_HandleEvent (hEventMbox);

                /* Trigger the FW when finishing handle the event */
                pEventMbox->returnValue = TNETWIF_WriteRegOpt (pEventMbox->hTNETWIF, 
                                                               ACX_REG_INTERRUPT_TRIG, 
                                                               INTR_TRIG_EVENT_ACK,
                                                               FW_EVENT_MODULE_ID,
                                                               eventMbox_StateMachine,
                                                               hEventMbox);

                pEventMbox->state = EVENT_MBOX_STATE_ACK_EVENT;

                break;

            case EVENT_MBOX_STATE_ACK_EVENT:

                /* Handling of the event is done. Switch to the next buffer for the next time */ 
                pEventMbox->currentEvent = 1 - pEventMbox->currentEvent; 
                    
                if (FALSE == pEventMbox->bSync) 
                {   
                    /* Asynchronous bus - call FwEvent for notifying the completion */
                    FwEvent_EventComplete (pEventMbox->hFwEvent, TNETWIF_OK);
                }
                else    
                {
                    /* This is the Sync case and we return TNETWIF_OK */
                    pEventMbox->returnValue = TNETWIF_OK;
                }
                /* Exit SM */
                pEventMbox->state = EVENT_MBOX_STATE_IDLE;
                return;

            default:
                WLAN_REPORT_ERROR(pEventMbox->hReport, EVENT_MBOX_MODULE_LOG, 
                    ("eventMbox_StateMachine: unknown state !!!\n"));
                break;
        }
    }

    /* If we are here - we got TNETWIF_PENDING, so we are in asynchronous mode */
    pEventMbox->bSync = FALSE;
}


/****************************************************************************
 *                      eventMbox_HandleEvent()
 ****************************************************************************
 * DESCRIPTION: The functions reads the parameters in the Event mailBox
 *              and activates the appropriate CallBack function.
 *
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK. 
 ****************************************************************************/
static void eventMbox_HandleEvent (TI_HANDLE hEventMbox)
{
    EventMbox_t *pEventMbox = (EventMbox_t *)hEventMbox;
    EventCB_t   *pEventCb;
    UINT32       EvID;
    UINT32       EvMask   = pEventMbox->CompoundEvent.eventsMask;
    UINT32       EvVector = pEventMbox->CompoundEvent.eventsVector;

#ifdef TI_DBG
    pEventMbox->uCompounEvCount++;
#endif /* TI_DBG */
    
#if EVMBX_DBG
    WLAN_REPORT_INFORMATION(pEventMbox->hReport, EVENT_MBOX_MODULE_LOG,
                             ("eventMbox_HandleEvent: Event Vector = 0x%x\n", EvVector));
#endif
    
    /*
    Handle  Events
    */
    for (EvID = 0; EvID < HAL_EVENT_ALL; EvID++)
    {
        pEventCb = &pEventMbox->cbTable[EvID];
        /* Check if the Event Bit in the vector in set */
        if (IS_EVENT_BIT_ON (EvVector, eventTable[EvID].bitMask))
        {   
#ifdef TI_DBG
            pEventMbox->uTotalEvCount++;
#endif /* TI_DBG */
            pEventCb->uCount++;
            /* Check if the Mask Bit in the Mask vector in off */
            if (IS_EVENT_BIT_OFF (EvMask, eventTable[EvID].bitMask))
            { 
#if EVMBX_DBG
                WLAN_REPORT_INFORMATION(pEventMbox->hReport, EVENT_MBOX_MODULE_LOG,
                                        ("eventMbox_HandleEvent: EVENT %s has occurred\n", eventTable[EvID].str));
#endif
                if (pEventCb->fCb != NULL)
                {                  
                    if (eventTable[EvID].dataLen)
                    {
                        ((whal_hwMboxDataEvCB)pEventCb->fCb) (pEventCb->hCb,
                                                              (char *)pEventMbox->cbTable[EvID].pDataOffs,
                                                              eventTable[EvID].dataLen);
                    }
                    else
                    {
                        ((whal_hwMboxEvCB)pEventCb->fCb) (pEventCb->hCb);
                    }
                }        
            }    
        } 
    } /*End for*/
}   


/*
 *  eventMbox_Print: print the Event Mailbox statistic :Number 890
 */
void eventMbox_Print (TI_HANDLE hEventMbox)
{
#ifdef TI_DBG
    EventMbox_t *pEventMbox = (EventMbox_t *)hEventMbox;
    UINT32 i;
    UINT32 EvMask   = pEventMbox->CompoundEvent.eventsMask;
    UINT32 EvVector = pEventMbox->CompoundEvent.eventsVector;

    WLAN_REPORT_REPLY(pEventMbox->hReport, EVENT_MBOX_MODULE_LOG,
        ("------------------------- EventMbox  Print ----------------------------\n"));

    WLAN_REPORT_INFORMATION(pEventMbox->hReport, EVENT_MBOX_MODULE_LOG,
        (" eventMbox_HandleEvent: Event Vector = 0x%x\n", EvVector));
    WLAN_REPORT_INFORMATION(pEventMbox->hReport, EVENT_MBOX_MODULE_LOG,
        (" eventMbox_HandleEvent: Event Mask = 0x%x\n", EvMask));
    WLAN_REPORT_REPLY(pEventMbox->hReport, EVENT_MBOX_MODULE_LOG,
        (" Total Number Of Compound Event = %d: \n", pEventMbox->uCompounEvCount));
    WLAN_REPORT_REPLY(pEventMbox->hReport, EVENT_MBOX_MODULE_LOG,
        (" Total Number Of Events = %d: \n", pEventMbox->uTotalEvCount));
    WLAN_REPORT_REPLY(pEventMbox->hReport, EVENT_MBOX_MODULE_LOG,("\t\t\t\t *** Event Counters *** :\n"));
    for (i = 0; i < HAL_EVENT_ALL; i++)
    {
        WLAN_REPORT_REPLY(pEventMbox->hReport, EVENT_MBOX_MODULE_LOG,
            (" %d) Event Name = EVENT %s, Number of Event = %d\n",
            i, eventTable[i].str, pEventMbox->cbTable[i].uCount));
    }

    WLAN_REPORT_REPLY(pEventMbox->hReport, EVENT_MBOX_MODULE_LOG,
        ("------------------------- EventMbox  Print End ----------------------------\n"));
#endif /* TI_DBG */
}



