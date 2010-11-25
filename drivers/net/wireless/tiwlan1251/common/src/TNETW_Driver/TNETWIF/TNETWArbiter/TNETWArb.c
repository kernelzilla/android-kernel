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
 *   MODULE:  GWSI_Synchronizer.c
 *   PURPOSE: GWSI Synchronizer used to synchronize between the CMD,INT,TX 
 *
 ****************************************************************************/

#include "commonTypes.h"
#include "memMngrEx.h" /* MSDU */
#include "report.h"


/* First the TNETW Arbiter interface needs the HAL definitions to use them in its API */
#include "TNETWArb.h"
#include "TNETWIF.h"

/* Those are functions that he client can send to the GWSI Synchronizer */
/* The client wants to start its process */

/***********************************************************************************
 Internal Synchronizer function use
**************************************************************************************/

/****************************************************************************
 *                      TNETWArb_Init()
 ****************************************************************************
 * DESCRIPTION: Initialize the synchronizer database.
 *
 * INPUTS:  TI_HANDLE hOs
 *
 * OUTPUT:  
 *
 * RETURNS:  TI_HANDLE hTNETWArb - Handle of the TNETW Arbiter module
 ****************************************************************************/
TI_HANDLE TNETWArb_Init (TI_HANDLE hOs)
{
    /* Handle to TNETW Arbiter struct */
    TNETWArb_t  *pTNETWArb;

    pTNETWArb = (TNETWArb_t *)os_memoryAlloc (hOs, sizeof(TNETWArb_t));
    if (pTNETWArb == NULL)
        return NULL;

    os_memoryZero (hOs, pTNETWArb, sizeof(TNETWArb_t));

    pTNETWArb->hOs = hOs;



    /* Get the TNETW Arbiter SM handle */
    pTNETWArb->hTNETWArbSM = TNETWArbSM_Create(hOs);

    return (TI_HANDLE)pTNETWArb;
}


/****************************************************************************
 *                      TNETWArb_Config()
 ****************************************************************************
 * DESCRIPTION: Configure the TNETWIF module
 *
 * INPUTS:  TI_HANDLE hTNETWArb
 *          TI_HANDLE hReport
 *          TI_HANDLE hELPCtrl
 *
 * OUTPUT:  void
 *
 * RETURNS: void
 ****************************************************************************/

void TNETWArb_Config (TI_HANDLE hTNETWArb,TI_HANDLE hReport,TI_HANDLE hELPCtrl)
{

    /* Handle to TNETW Arbiter struct */
    TNETWArb_t  *pTNETWArb = (TNETWArb_t *)hTNETWArb;   
    UINT8       index;

    pTNETWArb->hReport = hReport;

    /* Init buffer Q lists */
    for(index = 0 ; index < NUM_OF_TNETWARB_QUEUES ; index++)
    {
        TNETWArb_init_q (&(pTNETWArb->TNETWArbiter_Queues[index]));
    }

    /* CAll the TNETW Arbiter Initializeto set the buffer free and their next buffer to NULL */
    TNETWArb_buffer_init(&(pTNETWArb->TNETWArb_Client_Instance_Array[0][0]));


    /* Clear the Event to dispatch bit */
    pTNETWArb->event_to_dispatch&=~TNETWARB_IS_EVENT_PENDING;

    /* For now the Handle to theBus Arbiter is NULL */
    TNETWArbSM_Init(pTNETWArb->hTNETWArbSM,pTNETWArb->hReport,hTNETWArb,hELPCtrl,NULL);
}


/****************************************************************************/
/*                      TNETWIF_Destroy()
 ****************************************************************************
 * DESCRIPTION: Destroy The TNETWIF Module
 *
 * INPUTS:  
 *
 * OUTPUT:  String the name of the Queue
 *
 * RETURNS: 
 ****************************************************************************/
TI_STATUS TNETWArb_Destroy (TI_HANDLE hTNETWArb)
{

    /* Handle to TNETW Arbiter struct */
    TNETWArb_t  *pTNETWArb = (TNETWArb_t *)hTNETWArb;   


    /* Call the TNETWArb SM Destruction function */
    if (pTNETWArb->hTNETWArbSM)
        TNETWArbSM_Destroy (pTNETWArb->hTNETWArbSM);

    /* Free the TNETW Arbiter memory */
    if (pTNETWArb)
        os_memoryFree(pTNETWArb->hOs, pTNETWArb, sizeof(TNETWArb_t));

    return OK;

}

/****************************************************************************
 *                      TNETWArb_register_handler()
 ****************************************************************************
 * DESCRIPTION: Register the Client Callback function to the TNETW Arbiter 
 *              This function will be called in case of DMA Done i.e BusTXn Complete
 *
 * INPUTS:  TI_HANDLE hTNETWArb
 *          UINT8 module_id
 *          TNETWIF_callback_t module_CB_Func
 *          TI_HANDLE module_handle
 *
 * OUTPUT:  void
 *
 * RETURNS: void
 ****************************************************************************/
void TNETWArb_register_handler(TI_HANDLE hTNETWArb,UINT8 module_id,TNETWIF_callback_t module_CB_Func,TI_HANDLE module_handle)
{   
    /* Handle to TNETW Arbiter struct */
    TNETWArb_t              *pTNETWArb = (TNETWArb_t *)hTNETWArb;   
    TNETWARB_INSTANCE_T     *pTNETWARB_Inst;

     /* Get the pointer to the TNETWARB_INSTANCE struct */
    pTNETWARB_Inst = (TNETWARB_INSTANCE_T *) (&(pTNETWArb->TNETWArb_Client_Instance_Array[module_id][BUFFER_HDR_SIZE]));

    pTNETWARB_Inst->module_id = module_id;
    pTNETWARB_Inst->instance_callback = module_CB_Func;
    pTNETWARB_Inst->instance_handle = module_handle;

    if (module_id == DEFAULT_MODULE_ID)
        pTNETWArb->pDefInst = pTNETWARB_Inst;
}


/****************************************************************************
 *                      TNETWArb_Start()
 ****************************************************************************
 * DESCRIPTION: Client registration to the TNETW Arbiter whishing to have   **
**              access to the Bus
 *
 * INPUTS:  TI_HANDLE hTNETWArb
 *          UINT8 module_id
 *          TI_HANDLE ClientCallBack_Handle
 *          TNETWIF_callback_t module_CB_Func
 *
 * OUTPUT:  void
 *
 * RETURNS: TI_STATUS - TNETWIF_ERROR - In case that the Client is already registered to the TNETWIF
 *                      TNETWIF_COMPLETE - In case that the Client callback has been immediately called
 *                      TNETWIF_PENDING - In case that the Client will be called later 
 ****************************************************************************/
TI_STATUS TNETWArb_Start (TI_HANDLE hTNETWArb, UINT8 module_id, TI_HANDLE ClientCallBack_Handle, TNETWIF_callback_t ClientCallBack_Func)
{   
    /* Handle to TNETW Arbiter struct */
    TNETWArb_t              *pTNETWArb = (TNETWArb_t *)hTNETWArb;   
    TNETWARB_INSTANCE_T     *pTNETWARB_Inst;
    TI_STATUS                ret;

	WLAN_REPORT_INFORMATION (pTNETWArb->hReport, TNETW_ARBITER_MODULE_LOG,
                             ("\n TNETWArb_Start: Register START to Synchronizer from module_id %x  \n",module_id));

  #ifdef TI_DBG
    pTNETWArb->stat.uStart ++;
  #endif

    /*  Get the instance buffer matching to the module_id */
    pTNETWARB_Inst = (TNETWARB_INSTANCE_T *) TNETWArb_getpoolbuf(hTNETWArb,module_id);

    /* If the instance is already allocated then return error to Client caller */
    if(pTNETWARB_Inst == NULL)
    {
        WLAN_REPORT_ERROR(pTNETWArb->hReport,TNETW_ARBITER_MODULE_LOG,("\n TNETWArb_Start  Module %s Already requested TNETWIF!!!! \n", TNETWIF_ModuleIdToString(module_id)));
		TNETWIF_printErrorLog();
        return TNETWIF_ERROR;
    }

    TNETWArb_Enqueue (&(pTNETWArb->TNETWArbiter_Queues[TODO_LIST_INDEX]), (void*)pTNETWARB_Inst);

    WLAN_REPORT_INFORMATION(pTNETWArb->hReport,TNETW_ARBITER_MODULE_LOG,("TNETWArb_Start: Enqueue in TODO: module %s, pTNETWARB_Inst %x  instance_handle: %X\n", TNETWIF_ModuleIdToString(module_id), pTNETWARB_Inst,ClientCallBack_Handle));

    /* Fill the Client instance with the callback handle and function passed in the  call */
    pTNETWARB_Inst->module_id = module_id;
    pTNETWARB_Inst->instance_callback = ClientCallBack_Func;
    pTNETWARB_Inst->instance_handle = ClientCallBack_Handle;   

    WLAN_REPORT_INFORMATION (pTNETWArb->hReport, TNETW_ARBITER_MODULE_LOG,
                            ("\n TNETWArb_Start: Call TNETWArbSM_SMEvent with TNETWARBSM_EV_START for module %s  \n",TNETWIF_ModuleIdToString(module_id)));

    /* Send the  event EV_REG to the TNETW Arbiter for handling and take appropriate actions */
    ret = TNETWArbSM_SMEvent (pTNETWArb->hTNETWArbSM,
                              (module_id == FW_EVENT_MODULE_ID) ? TNETWARBSM_EV_HW_AVAIL : TNETWARBSM_EV_START);

    return ret;
}



/*********************************************************************************************************
**                                                                                                      **
**  Function Name: TNETWArb_Restart                                                                     **
**                                                                                                      **
**  Description: Client Re-registration to the TNETW Arbiter whishing to                                **
**              regain access to the Bus                                                                **
**              This will do the following:                                                             **
**                  1) Remove the instance from the Running Instance                                    **
**                  2) Add it to the TODO Queue                                                         **  
**                  3) Will set the Current event to be dispatched later on                             **
**                      from where the Restart was called inside the TNETW Arbiter                      **
**                      This is done to prevent nesting operations from within the Client context       **  
**                                                                                                      **
**********************************************************************************************************/
TI_STATUS TNETWArb_Restart (TI_HANDLE hTNETWArb,UINT8 module_id,TI_HANDLE ClientCallBack_Handle,TNETWIF_callback_t ClientCallBack_Func)
{   
    /* Handle to TNETW Arbiter struct */
    TNETWArb_t              *pTNETWArb = (TNETWArb_t *)hTNETWArb;   
    TNETWARB_INSTANCE_T     *pTNETWARB_Inst;

#ifdef TNETWARB_DEBUG
    /* For debug, verify that the running instance is not NULL. */
    if(pTNETWArb->TNETWArb_Running_instance ==  NULL )
    {
        WLAN_REPORT_ERROR(pTNETWArb->hReport,TNETW_ARBITER_MODULE_LOG,
            ("TNETWArb_Restart: There is no module running, requested by module %d\n", module_id));
        return TNETWIF_ERROR;
    }

    /* For debug check that the instance requesting restart is really the running instance */
    if(pTNETWArb->TNETWArb_Running_instance->module_id !=  module_id )
    {
        /* In this case return to caller ERROR */
        return TNETWIF_ERROR;
    }
#endif

  #ifdef TI_DBG
    pTNETWArb->stat.uRestart ++;
  #endif

    /* Take the Running instance control block */
    pTNETWARB_Inst = pTNETWArb->TNETWArb_Running_instance ;

    /* First indicate that there is not any handle in the Running instance */
    pTNETWArb->TNETWArb_Running_instance = NULL;

    /* Enqueue the client request or i.e its instance in the TODO List */
    TNETWArb_Enqueue(&(pTNETWArb->TNETWArbiter_Queues[TODO_LIST_INDEX]), (void*)pTNETWARB_Inst);
    
    WLAN_REPORT_INFORMATION(pTNETWArb->hReport,TNETW_ARBITER_MODULE_LOG,("TNETWArb_Restart: Enqueue in TODO: module %s, instance_handle: %X\n", TNETWIF_ModuleIdToString(module_id), ClientCallBack_Handle));

    /* Fill the Client instance with the callback handle and function given by the  call */
    pTNETWARB_Inst->module_id = module_id;
    pTNETWARB_Inst->instance_callback = ClientCallBack_Func;
    pTNETWARB_Inst->instance_handle = ClientCallBack_Handle;
    

     /* Set the event to dispatch so that when this function will return back then the context will return to the TNETW Arbiter */
    /* Then the TNETW Arbiter will dispatch the event to the STate Machine within its context and not within the Client context 
    to prevent nesting of many calls and prevent to increse the stack */
    pTNETWArb->event_to_dispatch = TNETWARBSM_EV_RESTART;
    pTNETWArb->event_to_dispatch|=TNETWARB_IS_EVENT_PENDING;

    /* Return Pending since the Restart will be handled in a later context */
    return TNETWIF_PENDING;

}

/*********************************************************************************************************
**                                                                                                      **
**  Function Name: TNETWArb_Finish                                                                      **
**                                                                                                      **
**  Description: Client De-registration to the TNETW Arbiter whishing to                                **
**              release access to the Bus                                                               **
**              This will do the following:                                                             **
**                  1) Remove the instance from the Running Instance                                    **
**                  3) Will set the Current event to be dispatched later on                             **
**                      from where the Finish was called inside the TNETW Arbiter                       **
**                      This is done to prevent nesting operations from within the Client context       **  
**                                                                                                      **
**********************************************************************************************************/
TI_STATUS TNETWArb_Finish (TI_HANDLE hTNETWArb,UINT8 module_id,TI_HANDLE ClientCallBack_Handle,TNETWIF_callback_t ClientCallBack_Func)
{   
    /* Handle to TNETW Arbiter struct */
    TNETWArb_t              *pTNETWArb = (TNETWArb_t *)hTNETWArb;   
    TNETWARB_INSTANCE_T     *pTNETWARB_Inst;
    
#ifdef TNETWARB_DEBUG
    /* For debug, verify that the running instance is not NULL. */
    if(pTNETWArb->TNETWArb_Running_instance ==  NULL )
    {
        WLAN_REPORT_ERROR(pTNETWArb->hReport,TNETW_ARBITER_MODULE_LOG,
            ("TNETWArb_Finish: There is no module running, requested by module %d\n", module_id));
        return TNETWIF_ERROR;
    }

    /* For debug check that the instance requesting restart is really the running instance */
    if(pTNETWArb->TNETWArb_Running_instance->module_id !=  module_id )
    {
        WLAN_REPORT_ERROR(pTNETWArb->hReport,TNETW_ARBITER_MODULE_LOG,
                          ("TNETWArb_Finish: Module %s is not Running module %s is Running \n",
                          TNETWIF_ModuleIdToString(module_id), 
                          TNETWIF_ModuleIdToString(pTNETWArb->TNETWArb_Running_instance->module_id)));
        /* In this case return to caller ERROR */
        return TNETWIF_ERROR;
    }

#endif

  #ifdef TI_DBG
    pTNETWArb->stat.uFinish ++;
  #endif

    /* Take the Running instance control block */
    pTNETWARB_Inst = pTNETWArb->TNETWArb_Running_instance ;

    WLAN_REPORT_INFORMATION(pTNETWArb->hReport,TNETW_ARBITER_MODULE_LOG,
                            ("TNETWArb_Finish: module %s, instance_handle: %X\n",
                            TNETWIF_ModuleIdToString(module_id), 
                            pTNETWARB_Inst->instance_handle));

    /* Free the buffer instance */
    TNETWArb_freebuf((void *)pTNETWARB_Inst);

    /* Indicate that there is not any handle in the Running instance */
    pTNETWArb->TNETWArb_Running_instance = NULL;

    /* Fill the Client instance with NULL callback to ease debugging */
    pTNETWARB_Inst->module_id = module_id;
    pTNETWARB_Inst->instance_callback = NULL;
    pTNETWARB_Inst->instance_handle = NULL;
    

    /* Now check the Event to send to the TNETW Arbiter State Machine */
    /* If there are more reequest in the TODO Queue then Send PROCESS_NEXT_EV else send FINISH event to TNETW Arbiter */
    if(TNETWArb_getfirst(&(pTNETWArb->TNETWArbiter_Queues[TODO_LIST_INDEX])))
    {
        /* Set the event to dispatch so that when this function will return back then the context will return to the TNETW Arbiter */
        /* Then the TNETW Arbiter will dispatch the event to the STate Machine within its context and not within the Client context 
        to prevent nesting of many calls and prevent to increse the stack */
        pTNETWArb->event_to_dispatch = TNETWARBSM_EV_RESTART;
        WLAN_REPORT_INFORMATION(pTNETWArb->hReport,TNETW_ARBITER_MODULE_LOG ,
                                  ("TNETWArb_Finish: module %s, instance_handle: %x Send TNETWARBSM_EV_RESTART \n",
                                  TNETWIF_ModuleIdToString(module_id),
                                  pTNETWARB_Inst->instance_handle));

    }
    else
    {
        /* Set the event to dispatch so that when this function will return back then the context will return to the TNETW Arbiter */
        /*  Then the TNETW Arbiter will dispatch the event to the STate Machine within its context and not within the Client context 
        to prevent nesting of many calls and prevent to increse the stack */
        pTNETWArb->event_to_dispatch = TNETWARBSM_EV_FINISH;
        WLAN_REPORT_INFORMATION(pTNETWArb->hReport,TNETW_ARBITER_MODULE_LOG,
                                ("TNETWArb_Finish: module %s, TNETWARBSM_EV_FINISH \n", 
                                TNETWIF_ModuleIdToString(module_id), 
                                pTNETWARB_Inst->instance_handle));

    }


    pTNETWArb->event_to_dispatch|=TNETWARB_IS_EVENT_PENDING;

    /* Return Pending since the Process_Next/Finish Event will be handled in a later context */
    return TNETWIF_COMPLETE;

}


/****************************************************************************
 *                      TNETWArb_CallTxnCb()
 ****************************************************************************
 * DESCRIPTION: This function is used to call the client callback function that was previously
 *              supplied to the TNETWIF via 2 ways:
 *                  1) When the Client requested with TNETWIF_Start to access the Bus
 *                  2) When the Client call the TNETWIF with Async or Optimzed mode.
 *                      In this case the callback will be called when the DMA will finish.
 *
 *              Note :The Client can request FINISHor RESTART bu the TNETW Arbiter state machine will
 *                     be sent the event only after that the client function callback returns.
 *                      This is done to prevent a deep nesting in case of RESTART.
 *
 * INPUTS:  TI_HANDLE hTNETWArb
 *
 * OUTPUT:  void
 *
 * RETURNS: void
 ****************************************************************************/
TI_STATUS TNETWArb_CallTxnCb (TI_HANDLE hTNETWArb)
{
    TNETWArb_t              *pTNETWArb = (TNETWArb_t *)hTNETWArb;   
    TNETWARB_INSTANCE_T     *pTNETWArbInst;

    /* 
     * If the pDefInst is not NULL it is a special arbiter client (ELP control).
     * Call the ELP control callback.
     */
    if (pTNETWArb->pDefInst != NULL)
    {
        pTNETWArbInst = pTNETWArb->pDefInst;
        pTNETWArb->pDefInst = NULL;

        if (pTNETWArbInst->instance_callback != NULL)
            ((TNETWIF_callback_t)(pTNETWArbInst->instance_callback))
                (pTNETWArbInst->instance_handle, pTNETWArbInst->module_id, OK);

        return TNETWIF_COMPLETE;
    }

    /* Otherwise, call a regular arbiter client callback */
    else
    {
        return TNETWArb_CallClientCallback (hTNETWArb);
    }
} 



/****************************************************************************
 *                      TNETWArb_CallClientCallback()
 ****************************************************************************
 * DESCRIPTION: This function is used to call the client callback function that was previously
 *              supplied to the TNETWIF via 2 ways:
 *                  1) When the Client requested with TNETWIF_Start to access the Bus
 *                  2) When the Client call the TNETWIF with Async or Optimzed mode.
 *                      In this case the callback will be called when the DMA will finish.
 *
 *              Note :The Client can request FINISHor RESTART bu the TNETW Arbiter state machine will
 *                     be sent the event only after that the client function callback returns.
 *                      This is done to prevent a deep nesting in case of RESTART.
 *
 * INPUTS:  TI_HANDLE hTNETWArb
 *
 * OUTPUT:  void
 *
 * RETURNS: void
 ****************************************************************************/
TI_STATUS TNETWArb_CallClientCallback (TI_HANDLE hTNETWArb)
{
    /* Handle to TNETW Arbiter struct */
    TNETWArb_t              *pTNETWArb = (TNETWArb_t *)hTNETWArb;   
    TNETWARB_INSTANCE_T     *pTNETWARB_Inst;

    /* Take the Running instance control block */
    if (pTNETWArb->TNETWArb_Running_instance != NULL)
    {
        pTNETWARB_Inst = pTNETWArb->TNETWArb_Running_instance;
    }
    else
    {
        WLAN_REPORT_ERROR (pTNETWArb->hReport, TNETW_ARBITER_MODULE_LOG,
                           ("TNETWArb_CallClientCallback: no running instance\n"));
		TNETWIF_printErrorLog();

        return TNETWIF_ERROR;
    }
  
    /* Call the client callback */
    if (pTNETWARB_Inst->instance_callback != NULL)
        ((TNETWIF_callback_t)(pTNETWARB_Inst->instance_callback))
            (pTNETWARB_Inst->instance_handle, pTNETWARB_Inst->module_id, OK);

    /* Check if there is an event to be dispatch */
    if (pTNETWArb->event_to_dispatch & TNETWARB_IS_EVENT_PENDING)
    {
        TI_STATUS  status;

        pTNETWArb->event_to_dispatch &= ~TNETWARB_IS_EVENT_PENDING;


        /* Now check the Event to send to the TNETW Arbiter State Machine */
        /* If there are more request in the TODO Queue then Send PROCESS_NEXT_EV else send FINISH event to TNETW Arbiter */
        if (TNETWArb_getfirst (&pTNETWArb->TNETWArbiter_Queues[TODO_LIST_INDEX]))
        {
            /* Set the event to dispatch so that when this function will return back then the context will return to the TNETW Arbiter */
            /* Then the TNETW Arbiter will dispatch the event to the STate Machine within its context and not within the Client context 
            to prevent nesting of many calls and prevent to increase the stack */
            pTNETWArb->event_to_dispatch = TNETWARBSM_EV_RESTART;
            status = TNETWIF_PENDING;
            WLAN_REPORT_INFORMATION (pTNETWArb->hReport,TNETW_ARBITER_MODULE_LOG ,("TNETWArb_CallClientCallback: module %s, instance_handle: %x Send TNETWARBSM_EV_RESTART \n", TNETWIF_ModuleIdToString(pTNETWARB_Inst->module_id), pTNETWARB_Inst->instance_handle));
        }
        else
        {
            /* Set the event to dispatch so that when this function will return back then the context will return to the TNETW Arbiter */
            /*  Then the TNETW Arbiter will dispatch the event to the STate Machine within its context and not within the Client context 
            to prevent nesting of many calls and prevent to increase the stack */
            pTNETWArb->event_to_dispatch = TNETWARBSM_EV_FINISH;
            status = TNETWIF_COMPLETE;
            WLAN_REPORT_INFORMATION (pTNETWArb->hReport,TNETW_ARBITER_MODULE_LOG,("TNETWArb_CallClientCallback: module %s, TNETWARBSM_EV_FINISH \n", TNETWIF_ModuleIdToString(pTNETWARB_Inst->module_id), pTNETWARB_Inst->instance_handle));
        }

        WLAN_REPORT_INFORMATION (pTNETWArb->hReport, TNETW_ARBITER_MODULE_LOG, ("TNETWArb_CallClientCallback: module %s, instance_handle: %x Send pTNETWArb->event_to_dispatch %d \n", TNETWIF_ModuleIdToString(pTNETWARB_Inst->module_id), pTNETWARB_Inst->instance_handle, pTNETWArb->event_to_dispatch));

        TNETWArbSM_SMEvent (pTNETWArb->hTNETWArbSM, pTNETWArb->event_to_dispatch);

        return status;
    }

    return TNETWIF_PENDING;
}


/****************************************************************************
 *                      TNETWArb_TxnCb()
 ****************************************************************************
 * DESCRIPTION: ELP Controller Callabck - Indicate to the TNETWArb that the HW is now Available 
 *              The FW has waken up. 
 * 
 * INPUTS:  hTNETWArb - the handle to the TNETW Arbiter
 *       
 * OUTPUT:  
 * 
 * RETURNS: OK
 ****************************************************************************/
void TNETWArb_TxnCb (TI_HANDLE hTNETWArb, UINT8 module_id, TI_STATUS status)
{
    TNETWArb_t  *pTNETWArb = (TNETWArb_t *)hTNETWArb;

    TNETWArbSM_TxnCb (pTNETWArb->hTNETWArbSM);
}


#ifdef TI_DBG
void TNETWArb_PrintStat (TI_HANDLE hTNETWArb)
{
    TNETWArb_t  *pTNETWArb = (TNETWArb_t *)hTNETWArb;

    WLAN_OS_REPORT (("Num of start   = %u\n", pTNETWArb->stat.uStart));
    WLAN_OS_REPORT (("Num of restart = %u\n", pTNETWArb->stat.uRestart));
    WLAN_OS_REPORT (("Num of finish  = %u\n", pTNETWArb->stat.uFinish));

	TNETWIF_printErrorLog();
}
#endif


/****************************************************************************
 *                      BusArbiter_Recovery()
 ****************************************************************************
 * DESCRIPTION: handles the recovery 
 *
 * INPUTS:  TI_HANDLE hBusArbiter
 *
 * OUTPUT:  void
 *
 * RETURNS: void
 ****************************************************************************/
TI_STATUS TNETWArb_Recovery(TI_HANDLE hTNETWArb, TI_HANDLE hELPCtrl)
{
	TNETWArb_t  *pTNETWArb = (TNETWArb_t *)hTNETWArb;

	/* should be updated - see CE20 */
	TNETWArb_Config (hTNETWArb, pTNETWArb->hReport, hELPCtrl);
	return OK;
	
}

