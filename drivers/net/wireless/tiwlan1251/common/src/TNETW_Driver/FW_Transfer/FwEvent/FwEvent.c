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
 *   MODULE:  FwEvent.c
 *   
 *   PURPOSE: Handle firmware events
 * 
 *   DESCRIPTION:  
 *   ============
 *      Call the appropriate event handler.
 *
 ****************************************************************************/

#include "osTIType.h"
#include "commonTypes.h"
#include "tnetwCommon.h"
#include "TNETWIF.h"
#include "TNETWArb.h"
#include "txResult_api.h"
#include "osApi.h"
#include "whalBus_Api.h"
#include "CmdMBox_api.h"
#include "TNETW_Driver.h"
#include "whalCtrl.h"
#include "shmBus.h"
#include "rxXfer_api.h" 
#include "FwEvent.h" 
#include "eventMbox_api.h"

#ifdef TI_DBG
#include "DebugTraceXfer_api.h"
#endif /* TI_DBG */

/* for debug only */
#undef  DEBUG_INTERRUPTS_PRINT

/********************* static function declerations *************************/
static void FwEvent_ReadRegCB (TI_HANDLE hFwEvent, UINT8 moduleID, TI_STATUS status);
static void FwEvent_WriteMaskCB (TI_HANDLE hFwEvent, UINT8 moduleID, TI_STATUS status);
static void FwEvent_WriteMuxCB (TI_HANDLE hFwEvent, UINT8 moduleID, TI_STATUS status);
static void FwEvent_UpdateRxBits (TI_HANDLE hFwEvent);

/* Client info structure */
typedef struct 
{
    /* Client event bit in interrupt status register */
    UINT32       event;
    /* Client handler */
    TI_STATUS  (*func) (TI_HANDLE hclient);
    /* Client corresponding trace message */
    char        *trace; 
} FwClient_t;


/* 
 * NOTE: Register clients in order of their priorities.
 *       The command mailbox priority is higher than that of event mailbox. 
 *       This ensures the command complete callback always arrives before event.  
 */
static const FwClient_t fwClient [MAX_EVENT_NUM] =
{
    { ACX_INTR_RX0_DATA,     rxXfer_RxEvent,         "ACX_INTR_RX0_DATA"     }, 
    { ACX_INTR_TX_RESULT,    txResult_TxCmpltIntrCB, "ACX_INTR_TX_RESULT"    }, 
    { ACX_INTR_RX1_DATA,     rxXfer_RxEvent,         "ACX_INTR_RX1_DATA"     }, 
    { ACX_INTR_CMD_COMPLETE, CmdMBox_CmdCmplt,       "ACX_INTR_CMD_COMPLETE" }, 
    { ACX_INTR_EVENT_A,      eventMbox_Event,        "ACX_INTR_EVENT_A"      }, 
    { ACX_INTR_EVENT_B,      eventMbox_Event,        "ACX_INTR_EVENT_B"      }, 
  #ifdef TI_DBG
    { ACX_INTR_TRACE_A,      debugTrace_Event,       "ACX_INTR_TRACE_A"      }, 
    { ACX_INTR_TRACE_B,      debugTrace_Event,       "ACX_INTR_TRACE_B"      }, 
  #endif
};


/****************************************************************************
*                      FwEvent_Create()
****************************************************************************
* DESCRIPTION: Create the FwEvent module object 
* 
* INPUTS:  None
* 
* OUTPUT:  None
* 
* RETURNS: The Created object
****************************************************************************/
TI_HANDLE FwEvent_Create (TI_HANDLE hOs)
{
    FwEventObj_t *pFwEvent;

    pFwEvent = os_memoryAlloc (hOs, sizeof(FwEventObj_t));
    if (pFwEvent == NULL)
    {
        return NULL;
    }

    os_memoryZero (hOs, pFwEvent, sizeof(FwEventObj_t));

    pFwEvent->hOs = hOs;

    return (TI_HANDLE)pFwEvent;
} /* FwEvent_Create() */


/****************************************************************************
*                      FwEvent_Destroy()
****************************************************************************
* DESCRIPTION: Destroy the FwEvent module object 
* 
* INPUTS:  hFwEvent - The object to free
* 
* OUTPUT:  None
* 
* RETURNS: OK 
****************************************************************************/
TI_STATUS FwEvent_Destroy (TI_HANDLE hFwEvent)
{
    FwEventObj_t *pFwEvent = (FwEventObj_t *)hFwEvent;

    if (pFwEvent)
    {
        os_memoryFree (pFwEvent->hOs, pFwEvent, sizeof(FwEventObj_t));
    }

    return TNETWIF_OK;
} /* FwEvent_Destroy() */


/****************************************************************************
*                      FwEvent_Config()
****************************************************************************
* DESCRIPTION: Config the FwEvent module object 
* 
* INPUTS:  hTNETW_Driver - TNETW Driver handle
*          hhFwEvent     - FwEvent handle;
* 
* OUTPUT:  None
* 
* RETURNS: None
****************************************************************************/
VOID FwEvent_Config (TI_HANDLE hFwEvent, TI_HANDLE hTnetwDrv)
{
    FwEventObj_t  *pFwEvent     = (FwEventObj_t *)hFwEvent;
    TnetwDrv_t    *pTnetwDrv    = (TnetwDrv_t *)hTnetwDrv;

    pFwEvent->hOs               = pTnetwDrv->hOs;
    pFwEvent->hReport           = pTnetwDrv->hReport;
    pFwEvent->hTNETWIF          = pTnetwDrv->hTNETWIF;

    /* Register clients in order of their priorities */
    pFwEvent->hClient[0]        = pTnetwDrv->hRxXfer;
    pFwEvent->hClient[1]        = pTnetwDrv->hTxResult;
    pFwEvent->hClient[2]        = pTnetwDrv->hRxXfer;
    pFwEvent->hClient[3]        = pTnetwDrv->hCmdMBox;
    pFwEvent->hClient[4]        = pTnetwDrv->hEventMbox;
    pFwEvent->hClient[5]        = pTnetwDrv->hEventMbox;
#ifdef TI_DBG
    pFwEvent->hClient[6]        = pTnetwDrv->hDebugTrace;
    pFwEvent->hClient[7]        = pTnetwDrv->hDebugTrace;
#endif /* TI_DBG */

    pFwEvent->FwEventState      = FW_EVENT_STATE_IDLE;
    pFwEvent->EventMask         = 0;

    /* Setting the RxControlAddr to 0 indicates that it shouldn't be used */
    pFwEvent->RxControlAddr     = 0;
    pFwEvent->uNumOfRxHandled   = 0;
    /* Before reading the first Fw Rx counters act like there's no Rx. This is done for the init phase */
    pFwEvent->uFwRxCounter      = 0;
} /* FwEvent_Config() */


/****************************************************************************
*                      FwEvent_SetHwInfo()
****************************************************************************
* DESCRIPTION: Set the rx control address. This is the register to be read 
                for the Fw Rx counters. Before this function is called we don't
                use that variable.
* 
* INPUTS:  hFwEvent         - FwEvent handle;
*          pDataPathParams  - struct to read the Address from
*
* OUTPUT:  None
* 
* RETURNS: None
****************************************************************************/
void FwEvent_SetHwInfo (TI_HANDLE hFwEvent, ACXDataPathParamsResp_t *pDataPathParams)
{
    FwEventObj_t *pFwEvent = (FwEventObj_t *)hFwEvent;

    pFwEvent->RxControlAddr = pDataPathParams->rxControlAddr;

    WLAN_REPORT_INFORMATION (pFwEvent->hReport, FW_EVENT_MODULE_LOG,
        ("%s: RxControlAddr=0x%x\n", __FUNCTION__, pFwEvent->RxControlAddr));
}


/****************************************************************************
 *                      FwEvent_CallHandler()
 ****************************************************************************
 * DESCRIPTION: Call FwEvent client's event handler 
 * 
 * INPUTS:  hFwEvent - The object



 * 
 * OUTPUT:  None
 * 
 * RETURNS: NOK, TNETWIF_COMPLETE, TNETWIF_PENDING 
 ****************************************************************************/
static TI_STATUS FwEvent_CallHandler (TI_HANDLE hFwEvent)
{
    FwEventObj_t *pFwEvent = (FwEventObj_t *)hFwEvent;
    TI_STATUS     rc;

    if (pFwEvent->EventVector == 0)
    { 
        return NOK;
    }

    while ((pFwEvent->EventNum < MAX_EVENT_NUM) &&
           (pFwEvent->EventVector & fwClient[pFwEvent->EventNum].event) == 0)
        pFwEvent->EventNum ++; 

    if (pFwEvent->EventNum < MAX_EVENT_NUM)
    {
        /* Negate corresponding bit in event vector */
        pFwEvent->EventVector &= ~fwClient[pFwEvent->EventNum].event;

        /* Call client handler */
        rc = fwClient[pFwEvent->EventNum].func (pFwEvent->hClient[pFwEvent->EventNum]);

        WLAN_REPORT_INFORMATION (pFwEvent->hReport, FW_EVENT_MODULE_LOG,
            ("FwEvent_CallHandler: %s, return=%u\n", fwClient[pFwEvent->EventNum].trace, rc));

        return rc;
    }

    return NOK;

} /* FwEvent_CallHandler */


/****************************************************************************
 *                      FwEvent_StateMachine()
 ****************************************************************************
 * DESCRIPTION: Manage the FwEvent state machine 
 * 
 * INPUTS:  hFwEvent - The object
 *          rc       - Code passed FwEvent_EventComplete(), either  OK or MORE
 * 
 * OUTPUT:  None
 * 
 * RETURNS: None 
 ****************************************************************************/
static void FwEvent_StateMachine (TI_HANDLE hFwEvent, systemStatus_e rc)
{
    FwEventObj_t *pFwEvent = (FwEventObj_t *)hFwEvent;

    while (rc != TNETWIF_PENDING)
    {       
        switch (pFwEvent->FwEventState)
        {      
            case FW_EVENT_STATE_WAIT_BUS_I:
                pFwEvent->PendingEvent = FALSE;
                pFwEvent->FwEventState = FW_EVENT_STATE_WAIT_MASK;
                /* 
                 * We reach this state after TNETWIF_Start CB
                 * mask the interrupts in the FW 
                 */
                rc = TNETWIF_WriteRegOpt (pFwEvent->hTNETWIF,
                                          HINT_MASK,
                                          ACX_INTR_ALL,
                                          FW_EVENT_MODULE_ID,
                                          FwEvent_WriteMaskCB,
                                          hFwEvent);

                break;

            case FW_EVENT_STATE_WAIT_MASK:  
                pFwEvent->FwEventState = FW_EVENT_STATE_WAIT_UNMUX;
                if (OK == TNETWIF_UnMux (pFwEvent->hTNETWIF))
                {
                    rc = TNETWIF_WriteELPOpt (pFwEvent->hTNETWIF, 
                                              0x1,/*ELPCTRL_WAKE_UP*/
                                              FW_EVENT_MODULE_ID,
                                              FwEvent_WriteMuxCB, 
                                              hFwEvent,
                                              TRUE);              
                }
                else
                {
                    rc = TNETWIF_COMPLETE;
                } 
                break;

            case FW_EVENT_STATE_WAIT_UNMUX:
            case FW_EVENT_STATE_WAIT_BUS_II:
                pFwEvent->LoopCounter++;
                /* Read the rx counters if the Address was configured */
                if (pFwEvent->RxControlAddr)
                {
                pFwEvent->FwEventState = FW_EVENT_STATE_WAIT_HINT_READ;
                }
                /* Rx Address was not configured yet. Jump over the read counters state */
                else    
                {
                    pFwEvent->FwEventState = FW_EVENT_STATE_WAIT_READ_COUNTERS;
                }
                /* 
                 * We reach this state after TNETWIF_Start CB
                 * mask the interrupts in the FW 
                 */
                rc = TNETWIF_ReadRegOpt (pFwEvent->hTNETWIF, 
                                         ACX_REG_INTERRUPT_CLEAR, 
                                         &pFwEvent->EventVector,
                                         FW_EVENT_MODULE_ID,
                                         FwEvent_ReadRegCB,
                                         hFwEvent);

                break;      

            case FW_EVENT_STATE_WAIT_HINT_READ:
                /* 
                 * Read Fw rx counters. This is needed due to a BUG in the Rx bits which causes to loose bits in the
                 * EventVector, and therefore we can't relay on the HINT read. The BUG is Fw/Hw related
                 */ 
                pFwEvent->FwEventState = FW_EVENT_STATE_WAIT_READ_COUNTERS;

                rc = TNETWIF_ReadMemOpt (pFwEvent->hTNETWIF, 
                                         pFwEvent->RxControlAddr, 
                                         PADREAD (&pFwEvent->uFwRxCounter),
                                         sizeof(UINT32),
                                         FW_EVENT_MODULE_ID,
                                         FwEvent_ReadRegCB,
                                         hFwEvent);

                break;     

            case FW_EVENT_STATE_WAIT_READ_COUNTERS:
                            
                WLAN_REPORT_INFORMATION(pFwEvent->hReport,FW_EVENT_MODULE_LOG,
                    ("Reading HostIntRegister = 0x%x, FwRxCounter = 0x%x DriverRxCounter = 0x%x\n", 
                    pFwEvent->EventVector, pFwEvent->uFwRxCounter, pFwEvent->uNumOfRxHandled));
                /* 
                 * Mask unwanted interrupts. 
                 */
                pFwEvent->EventVector &= pFwEvent->EventMask;
                
                /* Work-around: check if there's a missing Rx bit (or 2 bits) in the Register */
                FwEvent_UpdateRxBits (hFwEvent);
                
                if (pFwEvent->EventVector == 0)
                {
				   #ifdef LEVEL_IRQ
					/* if working level sensitive mode we must first enable IRQ source */
					os_enableIrq(pFwEvent->hOs);
				   #endif
                    pFwEvent->FwEventState = FW_EVENT_STATE_WAIT_UNMASK;
                    /* Unmask the interrupts in the FW */ 
                    rc = TNETWIF_WriteRegOpt (pFwEvent->hTNETWIF,
                                              HINT_MASK,
                                              ~pFwEvent->EventMask,
                                              FW_EVENT_MODULE_ID,
                                              FwEvent_WriteMaskCB,
                                              hFwEvent);    
                }
                else
                {
                    pFwEvent->FwEventState = FW_EVENT_STATE_HANDLE_EVENT;

                    rc = TNETWIF_COMPLETE;
                }
              
                break;

            case FW_EVENT_STATE_HANDLE_EVENT:                           
                if ((rc = FwEvent_CallHandler (hFwEvent)) == NOK)
                {      
                    /* All events have been handled, break the loop */
                    if (pFwEvent->LoopCounter < NUM_OF_READ_REG_LOOPS)
                    {
                        /* Restart */
                        pFwEvent->FwEventState = FW_EVENT_STATE_WAIT_BUS_II;
                        rc = TNETWIF_Restart (pFwEvent->hTNETWIF, 
                                              FW_EVENT_MODULE_ID, 
                                              hFwEvent, 
                                              FwEvent_BusReadyCB);
                    }
                    else    
                    {
					   #ifdef LEVEL_IRQ
						/* if working level sensitive mode we must first enable IRQ source */
						os_enableIrq(pFwEvent->hOs);
					   #endif
                        /* Unmask the interrupts in the FW */ 
                        pFwEvent->FwEventState = FW_EVENT_STATE_WAIT_UNMASK;                                
                        rc = TNETWIF_WriteRegOpt (pFwEvent->hTNETWIF,
                                                  HINT_MASK,
                                                  ~pFwEvent->EventMask,
                                                  FW_EVENT_MODULE_ID,
                                                  FwEvent_WriteMaskCB,
                                                  hFwEvent);
                    }                   
                }
                break;

            case FW_EVENT_STATE_WAIT_UNMASK:
                /* We get here after unmask CB */
                if (pFwEvent->PendingEvent)
                {
                    pFwEvent->PendingEvent = FALSE;
                    pFwEvent->FwEventState = FW_EVENT_STATE_WAIT_BUS_I;
                    TNETWIF_Restart (pFwEvent->hTNETWIF, 
                                     FW_EVENT_MODULE_ID, 
                                     hFwEvent, 
                                     FwEvent_BusReadyCB);                     
                }
                else
                {
                    pFwEvent->FwEventState = FW_EVENT_STATE_IDLE;
                    TNETWIF_Finish (pFwEvent->hTNETWIF, FW_EVENT_MODULE_ID, NULL, NULL);
                }
                rc = TNETWIF_PENDING;
                break;

            case FW_EVENT_STATE_IDLE:
            default:
                WLAN_REPORT_ERROR (pFwEvent->hReport, FW_EVENT_MODULE_LOG,
                                   ("FwEvent_StateMachine - invalid state\n"));
                rc = TNETWIF_PENDING;
                break;
                    
        } /* switch */

        if (TNETWIF_ERROR == rc)
        {
            WLAN_REPORT_ERROR (pFwEvent->hReport, FW_EVENT_MODULE_LOG,
                               ("FwEvent_StateMachine rc = TNETWIF_ERROR !!! in state = %d\n",
                               pFwEvent->FwEventState));
        }

    } /* while */

} /* FwEvent_StateMachine() */


/****************************************************************************
 *                      FwEvent()
 ****************************************************************************
 * DESCRIPTION: Start FwEvent 
 * 
 * INPUTS:  hFwEvent - The object 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK - if in Idle NOK - else
 ****************************************************************************/
TI_STATUS FwEvent (TI_HANDLE hFwEvent)
{
    FwEventObj_t *pFwEvent = (FwEventObj_t *)hFwEvent;

    /* NOTE: pFwEvent may be uninitialized at init stage */
    if (pFwEvent != NULL)
    {
        if (pFwEvent->FwEventState == FW_EVENT_STATE_IDLE)
        {
            pFwEvent->IntrState = STATE_DPC;
            pFwEvent->EventVector  = 0;
            pFwEvent->LoopCounter  = 0;
            pFwEvent->FwEventState = FW_EVENT_STATE_WAIT_BUS_I;

            /* NOTE: hTNETWIF may be uninitialized at init */
            if (pFwEvent->hTNETWIF != NULL)
            {
                TNETWIF_Start (pFwEvent->hTNETWIF, FW_EVENT_MODULE_ID, hFwEvent, FwEvent_BusReadyCB);                   
            }
        }
        else if (pFwEvent->FwEventState == FW_EVENT_STATE_WAIT_UNMASK)
        {
            pFwEvent->LoopCounter = 0;
            /*
             * If an interrupt receiving while unmasking the previous, sign it as pending and exit. 
             * It will be handled in the next iteration after Restart.
             */ 
            pFwEvent->PendingEvent = TRUE;
        }
        else
        {       
            WLAN_REPORT_WARNING (pFwEvent->hReport, FW_EVENT_MODULE_LOG,
                                 ("FwEvent() entering SM not in Idle !!! state: %d\n",
                                 pFwEvent->FwEventState));
            
            return NOK;
        }
    }

    return OK;
} /* FwEvent() */


/****************************************************************************
 *                      FwEvent_BusReadyCB()
 ****************************************************************************
 * DESCRIPTION: FwEvent_BusReadyCB 
 * 
 * INPUTS:  hFwEvent - The object
 * 
 * OUTPUT:  None
 * 
 * RETURNS: None
 ****************************************************************************/
void FwEvent_BusReadyCB (TI_HANDLE hFwEvent, UINT8 module_id, TI_STATUS status)
{
    FwEventObj_t *pFwEvent = (FwEventObj_t *)hFwEvent;

    if (pFwEvent->FwEventState == FW_EVENT_STATE_WAIT_BUS_I ||
        pFwEvent->FwEventState == FW_EVENT_STATE_WAIT_BUS_II)
    {
        /*
         * NOTE: init the EventNum here for it take effect both for Start and Restart
         */
        pFwEvent->EventNum = 0;
        FwEvent_StateMachine (hFwEvent, TNETWIF_NONE);
    }
    else
    {
        WLAN_REPORT_ERROR (pFwEvent->hReport, FW_EVENT_MODULE_LOG,
                           ("FwEvent_BusReadyCB() state(%d) is not FW_EVENT_STATE_READ_REG\n",
                           pFwEvent->FwEventState));
    }
} /* FwEvent_BusReadyCB() */


/****************************************************************************
 *                      FwEvent_WriteMaskCB()
 ****************************************************************************
 * DESCRIPTION: Write Mask CB 
 * 
 * INPUTS:  hFwEvent - The object
 * 
 * OUTPUT:  None
 * 
 * RETURNS: None
 ****************************************************************************/
void FwEvent_WriteMaskCB (TI_HANDLE hFwEvent, UINT8 moduleID, TI_STATUS status)
{
    FwEventObj_t *pFwEvent = (FwEventObj_t *)hFwEvent;

    if (pFwEvent->FwEventState == FW_EVENT_STATE_WAIT_MASK ||
        pFwEvent->FwEventState == FW_EVENT_STATE_WAIT_UNMASK)
    {
        FwEvent_StateMachine (hFwEvent, TNETWIF_NONE);
    }
    else
    {
        WLAN_REPORT_ERROR (pFwEvent->hReport, FW_EVENT_MODULE_LOG,
                           ("FwEvent_WriteMaskCB() shouldn't be called with state(%d)\n",
                           pFwEvent->FwEventState));
    }
} /* FwEvent_WriteMaskCB() */


/****************************************************************************
 *                      FwEvent_WriteMuxCB()
 ****************************************************************************
 * DESCRIPTION: Write Mask CB 
 * 
 * INPUTS:  hFwEvent - The object
 * 
 * OUTPUT:  None
 * 
 * RETURNS: None
 ****************************************************************************/
void FwEvent_WriteMuxCB (TI_HANDLE hFwEvent, UINT8 moduleID, TI_STATUS status)
{
    FwEventObj_t *pFwEvent = (FwEventObj_t *)hFwEvent;

    if (pFwEvent->FwEventState == FW_EVENT_STATE_WAIT_UNMUX)        
    {
        FwEvent_StateMachine (hFwEvent, TNETWIF_NONE);
    }
    else
    {
        WLAN_REPORT_ERROR (pFwEvent->hReport, FW_EVENT_MODULE_LOG,
                           ("FwEvent_WriteMaskCB() shouldn't be called with state(%d)\n",
                           pFwEvent->FwEventState));
    }
} /* FwEvent_WriteMaskCB() */


/****************************************************************************
 *                      FwEvent_ReadRegCB()
 ****************************************************************************
 * DESCRIPTION: FwEvent_ReadRegCB 
 * 
 * INPUTS:  hFwEvent - The object
 * 
 * OUTPUT:  None
 * 
 * RETURNS: None
 ****************************************************************************/
void FwEvent_ReadRegCB (TI_HANDLE hFwEvent, UINT8 moduleID, TI_STATUS status)
{
    FwEventObj_t *pFwEvent = (FwEventObj_t *)hFwEvent;
        
    if ((pFwEvent->FwEventState == FW_EVENT_STATE_WAIT_HINT_READ) || 
        (pFwEvent->FwEventState == FW_EVENT_STATE_WAIT_READ_COUNTERS))
    {
        FwEvent_StateMachine (hFwEvent, TNETWIF_NONE);
    }
    else
    {
        WLAN_REPORT_ERROR (pFwEvent->hReport, FW_EVENT_MODULE_LOG,
                           ("FwEvent_ReadRegCB() is in state(%d)\n",
                           pFwEvent->FwEventState));
    }
} /* FwEvent_BusReadyCB() */


/****************************************************************************
 *                      FwEvent_UpdateRxBits()
 ****************************************************************************
 * DESCRIPTION: Update the EventVector according to the Fw counter of Rx.
 *              The Rx bits from the EventVector are ignored and instead we are
 *              using the Fw counters in order to decide how many Rx should we read.
 *              Using this method is due to a Hw/Fw bug in which we miss some of the 
 *              Rx bits in the EventVector.
 * 
 * INPUTS:  pFwEvent - The object
 * 
 * OUTPUT:  pFwEvent->EventVector - Add Rx bit if needed
 * 
 * RETURNS: None
 ****************************************************************************/
void FwEvent_UpdateRxBits (TI_HANDLE hFwEvent)
{
    FwEventObj_t *pFwEvent = (FwEventObj_t *)hFwEvent;
    UINT32 uFwDriverDiff = pFwEvent->uFwRxCounter - pFwEvent->uNumOfRxHandled;
    
    /* use only the last 4 bits since the Fw is using 4 bits */
    uFwDriverDiff &= 0xf;   

    /* Use the diff to add the number of bits needed for handling */
    switch (uFwDriverDiff) 
    {
    case 0: 
        /* Erase Rx bits */
        pFwEvent->EventVector &= ~ACX_INTR_RX0_DATA;
        pFwEvent->EventVector &= ~ACX_INTR_RX1_DATA;
        break;
    case 1: 
        /* Add only one bit */
        pFwEvent->EventVector |=  ACX_INTR_RX0_DATA; 
        pFwEvent->EventVector &= ~ACX_INTR_RX1_DATA;
        break;
    case 2: 
        /* Add the 2 bits */
        pFwEvent->EventVector |=  ACX_INTR_RX0_DATA; 
        pFwEvent->EventVector |= ACX_INTR_RX1_DATA;
        break;
    default: 
        /*
         * This is a very bad case were there is no synchronization between Driver & FW. In order to recover from this 
         * state we will use the the EventVector "as-is" and hope for the best... 
         */
        WLAN_REPORT_ERROR (pFwEvent->hReport, FW_EVENT_MODULE_LOG,
            ("%s Fw = 0x%x Dr = 0x%x\n", __FUNCTION__, pFwEvent->uFwRxCounter, pFwEvent->uNumOfRxHandled)); 

        break;
    }

    /* This will make sure that next time we will be synchronized with the Fw */
    pFwEvent->uNumOfRxHandled = pFwEvent->uFwRxCounter; 

}

/****************************************************************************
 *                      FwEvent_EventComplete()
 ****************************************************************************
 * DESCRIPTION: FwEvent_EventComplete 
 * 
 * INPUTS:  hFwEvent - The object
 *          rc - OK or MORE
 * 
 * OUTPUT:  None
 * 
 * RETURNS: None
 ****************************************************************************/
void FwEvent_EventComplete (TI_HANDLE hFwEvent, systemStatus_e rc)
{
    FwEventObj_t *pFwEvent = (FwEventObj_t *)hFwEvent;

    if (pFwEvent->FwEventState == FW_EVENT_STATE_HANDLE_EVENT)
    {
        FwEvent_StateMachine (hFwEvent,rc);
    }
    else
    {
        WLAN_REPORT_ERROR (pFwEvent->hReport, FW_EVENT_MODULE_LOG,
                           ("FwEvent_EventComplete() state(%d) is not FW_EVENT_STATE_PENDING\n",
                           pFwEvent->FwEventState));
    }
} /* FwEvent_EventComplete() */


/****************************************************************************
 *                      FwEvent_Enable()
 ****************************************************************************
 * DESCRIPTION: enable specific interrupt
 * 
 * INPUTS:  
 * 
 * OUTPUT:
 ****************************************************************************/
void  FwEvent_Enable (TI_HANDLE hFwEvent, UINT32 uEventMask)
{
    FwEventObj_t *pFwEvent = (FwEventObj_t *)hFwEvent;
    
    pFwEvent->EventMask |= uEventMask;

    WLAN_REPORT_INFORMATION (pFwEvent->hReport, FW_EVENT_MODULE_LOG,
         ("%s: EventMask = 0x%x\n", __FUNCTION__, pFwEvent->EventMask));
}


/****************************************************************************
 *                      FwEvent_Disable()
 ****************************************************************************
 * DESCRIPTION: disables specific interrupt
 * 
 * INPUTS:  
 * 
 * OUTPUT:
 ****************************************************************************/
void  FwEvent_Disable (TI_HANDLE hFwEvent, UINT32 uEventMask)
{
    FwEventObj_t *pFwEvent = (FwEventObj_t *)hFwEvent;
    
    pFwEvent->EventMask &= ~uEventMask;

    WLAN_REPORT_INFORMATION (pFwEvent->hReport, FW_EVENT_MODULE_LOG,
         ("%s: EventMask = 0x%x\n", __FUNCTION__, pFwEvent->EventMask));
}


/****************************************************************************
 *                      FwEvent_GetEnabled()
 ****************************************************************************
 * DESCRIPTION: returns interrupt enabled bit mask
 * 
 * INPUTS:  
 * 
 * OUTPUT:
 ****************************************************************************/
UINT32 FwEvent_GetEnabled (TI_HANDLE hFwEvent)
{
    FwEventObj_t *pFwEvent = (FwEventObj_t *)hFwEvent;
    
    return pFwEvent->EventMask;
}

/******************************************************************************************************
*
*   Functions originally located at whalHwIntr.c - Not used in the current version and might be removed
*
*******************************************************************************************************/


/****************************************************************************
 *                      FwEvent_EnableInterrupts()
 ****************************************************************************
 * DESCRIPTION: Enable interrupts
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * NOTE: Originally located at whalHwIntr.c .
 ****************************************************************************/
void FwEvent_EnableInterrupts (TI_HANDLE hFwEvent)
{
#ifdef USE_SYNC_API

    FwEventObj_t *pFwEvent = (FwEventObj_t *)hFwEvent;

    /* Clearing all the interrupt status register sources */
    TNETWIF_WriteRegSync (pFwEvent->hTNETWIF, ACX_REG_INTERRUPT_MASK, ~pFwEvent->EventMask);
    
    /*
     * Setting the right operation of the interrupt
     * bit 5 - enable interrupt
     * bit 7 - active low 
     */
    TNETWIF_WriteRegSync (pFwEvent->hTNETWIF, HI_CFG, HI_CFG_DEF_VAL);

  #ifdef DEBUG_INTERRUPTS_PRINT
    WLAN_REPORT_INFORMATION (pFwEvent->hReport,
                             FW_EVENT_MODULE_LOG,
                             ("FwEvent_EnableInterrupts(0x%08X)",
                             pFwEvent->EventMask));
  #endif /* DEBUG_INTERRUPTS_PRINT */

  #if defined(HAL_ON_WIN)
    /* (!!!) only in CardBus, add HostIfType parameter */
    /* Enable Interrupt on a CardBus */
    TNETWIF_WriteRegSync (pFwEvent->hTNETWIF, FEMR, 0x8000);
  #endif

#endif /* USE_SYNC_API */

}

/****************************************************************************
 *                      FwEvent_DisableInterrupts()
 ****************************************************************************
 * DESCRIPTION: Disable interrupts
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS:
 ****************************************************************************/
void FwEvent_DisableInterrupts (TI_HANDLE hFwEvent)
{   
#ifdef USE_SYNC_API

    FwEventObj_t *pFwEvent = (FwEventObj_t *)hFwEvent;

    TNETWIF_WriteRegSync (pFwEvent->hTNETWIF, ACX_REG_INTERRUPT_MASK, ACX_INTR_ALL );

  #ifdef DEBUG_INTERRUPTS_PRINT
    WLAN_REPORT_INFORMATION (pFwEvent->hReport,
                             FW_EVENT_MODULE_LOG,
                             ("FwEvent_DisableInterrupts(0x%08X)",
                             ACX_INTR_ALL));
  #endif /* DEBUG_INTERRUPTS_PRINT */

#endif /* USE_SYNC_API */
}


/****************************************************************************
 *                      FwEvent_CheckInterrupts()
 ****************************************************************************
 * DESCRIPTION: Check if there is interrupts (only unmasked)
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: 0 - no interrupts, otherwise - there are interrupts
 ****************************************************************************/
UINT32 FwEvent_CheckInterrupts (TI_HANDLE hFwEvent)
{
#ifdef USE_SYNC_API

    FwEventObj_t     *pFwEvent  = (FwEventObj_t *)hFwEvent;
    register UINT32 CurrentIntr;
    UINT32 interruptRegVal;
    register UINT32 maskInterruptVal;
  #ifdef DEBUG_INTERRUPTS_PRINT
    UINT32 endReg;
  #endif /* DEBUG_INTERRUPTS_PRINT */

    if (pFwEvent->IntrState != STATE_OPERATIONAL)
    {
        /*
        ISR can't be called till the state will be operational again because the ISR
        disable the interrupts of the TNET thus if this function is called then it need
        to return 0!!!
        */
  #ifdef DEBUG_INTERRUPTS_PRINT
        WLAN_REPORT_WARNING (pFwEvent->hReport,
                             FW_EVENT_MODULE_LOG,
                             ("FwEvent_CheckInterrupts() - state isn't STATE_OPERATIONAL (=%d) - ABRTING!\n",
                             pFwEvent->IntrState));
  #endif /* DEBUG_INTERRUPTS_PRINT */
    }

  #ifdef HW_ACCESS_DEBUG_ACCESS_VIOLATION
    whal_hwAccess_setOverrideElpCheck ((TI_HANDLE)pFwEvent->hTNETWIF, TRUE);                                      
  #endif /* HW_ACCESS_DEBUG_ACCESS_VIOLATION */

    /*read the status register*/
    TNETWIF_ReadRegSync (pFwEvent->hTNETWIF, ACX_REG_INTERRUPT_NO_CLEAR, &interruptRegVal);
    
    CurrentIntr = interruptRegVal;

    /* 0xFFFF means that the card is disconnected !!! */
    if ((CurrentIntr & 0xffff) == 0xffff) /* error */
        CurrentIntr = 0;

    /* check with the interrupt mask register */
    maskInterruptVal = pFwEvent->EventMask;
    CurrentIntr &= maskInterruptVal;
    if (interruptRegVal != CurrentIntr)
    {
  #ifdef DEBUG_INTERRUPTS_PRINT
        WLAN_REPORT_ERROR(pFwEvent->hReport,
                          FW_EVENT_MODULE_LOG,
                          ("%s(%d) - interrupt vector include masked interrupts\n.\
                          interruptRegVal   = 0x%08X\n\
                          hwMaskInterruptVal= 0x%08X\n\
                          swMaskInterruptVal= 0x%08X\n\
                          currrentInt       = 0x%08X\n\
                          diverse           = 0x%08X\n\
                          IntrState         = %d\n",
                          __FILE__,__LINE__,
                          interruptRegVal,
                          maskInterruptVal,
                          pFwEvent->EventMask,
                          CurrentIntr,
                          (CurrentIntr ^ interruptRegVal),
                          pFwEvent->IntrState));
  #endif /* DEBUG_INTERRUPTS_PRINT */
    }

  #ifdef ACK_ON_CHECK_PHASE
    /* set ACK to the interrupts on the check phase */
    if (CurrentIntr != 0)
    {
        /* Save the occurring interrupts - to handle interrupt routine */
        pFwEvent->SaveIntrValue |= CurrentIntr;
        HW_INTR_ACK(pFwEvent->hTNETWIF, CurrentIntr);
        /*
        state is now wait for DPC
        */
        pFwEvent->IntrState = STATE_WAIT_FOR_DPC;
    }
  #endif /* ACK_ON_CHECK_PHASE */

  #ifdef DEBUG_INTERRUPTS_PRINT

    TNETWIF_ReadRegSync (pFwEvent->hTNETWIF, ACX_REG_INTERRUPT_NO_CLEAR, &endReg);
    WLAN_REPORT_INFORMATION(pFwEvent->hReport,
                            FW_EVENT_MODULE_LOG,
                            ("%s(%d) - finish ISR ,endReg... \n.\
                            Intr   = 0x%08X\n",
                            __FILE__,__LINE__,
                            endReg));
  #endif /* DEBUG_INTERRUPTS_PRINT */

  #ifdef HW_ACCESS_DEBUG_ACCESS_VIOLATION
    whal_hwAccess_setOverrideElpCheck ((TI_HANDLE)pFwEvent->hTNETWIF, FALSE);                                  
  #endif /* HW_ACCESS_DEBUG_ACCESS_VIOLATION */

    /* (!!!1150) Reset the interrupt line*/
    TNETWIF_WriteRegSync (pFwEvent->hTNETWIF, PCI_STATUS_CLR_REG, 0x80000000 /*v2p_intr was asserted*/);

    return CurrentIntr;

#else

    return 0;

#endif /* USE_SYNC_API */
}


/****************************************************************************
 *                      FwEvent_ChangeState()
 ****************************************************************************
 * DESCRIPTION: Disable interrupts
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS:
 ****************************************************************************/
void FwEvent_ChangeState (TI_HANDLE hFwEvent, int State)
{
    FwEventObj_t *pFwEvent = (FwEventObj_t *)hFwEvent;

    pFwEvent->IntrState = State;
}


/****************************************************************************
 *                      FwEvent_StateChanged()
 ****************************************************************************
 * DESCRIPTION: 
 *              StateChanged - change mask notification and 
 *                              interrupt acknowledge. Used for SDIO driver
 *
 * RETURNS: None 
 ****************************************************************************/
void FwEvent_StateChanged (TI_HANDLE hFwEvent)      
{
  #ifdef USE_SYNC_API
    FwEventObj_t *pFwEvent  = (FwEventObj_t *)hFwEvent;

    TNETWIF_WriteRegSync (pFwEvent->hTNETWIF, ACX_REG_INTERRUPT_TRIG, INTR_TRIG_STATE_CHANGED);
  #endif
}

/****************************************************************************
 *                      FwEvent_Stop()
 ****************************************************************************
 * DESCRIPTION:	Stop & reser FwEvent (called by the recovery)
 * 
 * INPUTS:	
 *			hhFwEvent - FwEvent handle;
 *
 * OUTPUT:	None
 * 
 * RETURNS:	None
 ****************************************************************************/
VOID  FwEvent_Stop(TI_HANDLE hFwEvent)
{
	FwEventObj_t *pFwEvent = (FwEventObj_t *)hFwEvent;

	pFwEvent->FwEventState	= FW_EVENT_STATE_IDLE;

	/* Setting the RxControlAddr to 0 indicates that it shouldn't be used */
	pFwEvent->RxControlAddr     = 0;
	pFwEvent->uNumOfRxHandled   = 0;
	/* Before reading the first Fw Rx counters act like there's no Rx. This is done for the init phase */
	pFwEvent->uFwRxCounter      = 0;

} /* FwEvent_Stop() */

