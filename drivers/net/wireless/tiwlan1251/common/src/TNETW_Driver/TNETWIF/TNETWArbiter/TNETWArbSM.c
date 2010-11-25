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
#include "nrfsm.h"


/* First the GWSI interface needs the HAL definitions to use them in its API */
#include "TNETWIF.h"
#include "TNETWArb.h"
#include "ElpCtrl.h"
#include "whalHwAccess.h"

#undef BUS_GUARD_SUPPORT


/***********************************************************************************
 Internal TNWTW Arbiter SM Module Internal function use
**************************************************************************************/
static TI_STATUS TNETWArbSM_SeizeBusOnStart        (TI_HANDLE hTNETWArbSM);
static TI_STATUS TNETWArbSM_SeizeBusOnHwAvail      (TI_HANDLE hTNETWArbSM);
static TI_STATUS TNETWArbSM_SeizeBus               (TI_HANDLE hTNETWArbSM, BOOL bHwAvail);
static TI_STATUS TNETWArbSM_ReturnPendingOnStart   (TI_HANDLE hTNETWArbSM);
static TI_STATUS TNETWArbSM_ReturnPendingOnHwAvail (TI_HANDLE hTNETWArbSM);
static TI_STATUS TNETWArbSM_WakeHw                 (TI_HANDLE hTNETWArbSM);
static TI_STATUS TNETWArbSM_RunClientCb            (TI_HANDLE hTNETWArbSM);
static TI_STATUS TNETWArbSM_ReseizeBus             (TI_HANDLE hTNETWArbSM);
static TI_STATUS TNETWArbSM_PutSleepHw             (TI_HANDLE hTNETWArbSM);
static TI_STATUS TNETWArbSM_WaitIRQHw              (TI_HANDLE hTNETWArbSM);
static TI_STATUS TNETWArbSM_ExitWakeUp             (TI_HANDLE hTNETWArbSM);
static TI_STATUS TNETWArbSM_ReleaseBus             (TI_HANDLE hTNETWArbSM);
static TI_STATUS TNETWArbSM_DoNothing              (TI_HANDLE hTNETWArbSM);
static TI_STATUS TNETWArbSM_ActionUnexpected       (TI_HANDLE hTNETWArbSM);

#if defined(BUS_GUARD_SUPPORT)
static void TNETWArbSM_BusAvailCB                  (TI_HANDLE hTNETWArbSM);
#endif


/*****************************************************************************
 **         Public Function prototypes                                      **
 *****************************************************************************/

/**
 * \author Yossi Peery
 * \date 15-May-2005\n
 * \brief Creates the object of the PowerSrv.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the OS.\n
 * Return Value: TI_HANDLE - handle to the PowerSrv object.\n
 */
TI_HANDLE TNETWArbSM_Create (TI_HANDLE hOs)
{
    TNETWArbSM_t *pTNETWArbSM = NULL;
    TI_STATUS status;

    pTNETWArbSM = (TNETWArbSM_t*) os_memoryAlloc (hOs, sizeof(TNETWArbSM_t));
    if (pTNETWArbSM == NULL)
    {
        WLAN_OS_REPORT(("%s(%d) - Memory Allocation Error!\n",__FILE__,__LINE__));
        return NULL;
    }

    os_memoryZero (hOs, pTNETWArbSM, sizeof(TNETWArbSM_t));

    pTNETWArbSM->hOS = hOs;

    status = nrfsm_Create (hOs, &pTNETWArbSM->hFSM, TNETWARB_SM_STATE_NUM, TNETWARBSM_EVENT_NUM); 
    if (status != OK)
    {
        WLAN_OS_REPORT(("%s(%d) - Error in create FSM!\n",__FILE__,__LINE__));
        TNETWArbSM_Destroy(pTNETWArbSM);
        return NULL;
    }

    return pTNETWArbSM;
}


/**
 * \author Yossi Peery
 * \date 15-May-2005\n
 * \brief Destroy the object of the PowerSrvSM.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the PowerSrv object.\n
 * Return Value: TI_STATUS - OK on success else NOK.\n
 */
TI_STATUS TNETWArbSM_Destroy (TI_HANDLE hTNETWArbSM)
{
    TNETWArbSM_t *pTNETWArbSM = (TNETWArbSM_t *)hTNETWArbSM;
    TI_HANDLE hOs = pTNETWArbSM->hOS;

    if (pTNETWArbSM->hFSM != NULL)
    {
        nrfsm_Unload (pTNETWArbSM->hFSM);                  
    }

    os_memoryFree (hOs, pTNETWArbSM, sizeof(TNETWArbSM_t));

    WLAN_OS_REPORT (("%s(%d) -  TNETWArbSM destroyed\n", __FILE__, __LINE__));

    return OK;
}


/**
 * \date 15-May-2005\n
 * \brief Initialize the PowerSrvSM module.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the PowerSrvSM object.\n
 * 2) TI_HANDLE - handle to the Report object.
 * 3) TI_HANDLE - handle to the whalCtrl object.
 * 4) TI_HANDLE - handle to the QosMgr object.
 * 5) TI_HANDLE - handle to the Mlme object.
 * 6) TI_HANDLE - handle to the SiteMgr object.
 * 7) PowerSrvInitParams_t - the Power Server initialize parameters.\n
 * Return Value: TI_STATUS - OK on success else NOK.\n
 */
TI_STATUS TNETWArbSM_Init (TI_HANDLE hTNETWArbSM, TI_HANDLE hReport, TI_HANDLE hTNETWArb, TI_HANDLE hELPCtrl, TI_HANDLE hBusArbiter)
{
    TNETWArbSM_t    *pTNETWArbSM = (TNETWArbSM_t *)hTNETWArbSM;

    nrfsm_action_cell_t smMatrix[TNETWARB_SM_STATE_NUM][TNETWARBSM_EVENT_NUM] =
    {
        /*
         * State [TNETWARBSM_STATE_IDLE]
        */
        {
            /* {TNETWARBSM_EV_START} -> TNETWARBSM_STATE_WAIT_BUS */
            {TNETWARBSM_STATE_WAIT_BUS           , TNETWArbSM_SeizeBusOnStart},                 

            /* {TNETWARBSM_EV_BUS_AVAIL} -> TNETWARBSM_STATE_IDLE */
            {TNETWARBSM_STATE_IDLE               , TNETWArbSM_ActionUnexpected},                 

            /* {TNETWARBSM_EV_HW_AVAIL} -> TNETWARBSM_STATE_IDLE */
            {TNETWARBSM_STATE_WAIT_BUS           , TNETWArbSM_SeizeBusOnHwAvail},                 

            /* {TNETWARBSM_EV_RESTART} -> TNETWARBSM_STATE_IDLE */
            {TNETWARBSM_STATE_WAIT_BUS           , TNETWArbSM_ReseizeBus},                             

            /* {TNETWARBSM_EV_FINISH} -> TNETWARBSM_STATE_IDLE */
            {TNETWARBSM_STATE_IDLE               , TNETWArbSM_ActionUnexpected},                              

            /* {TNETWARBSM_EV_TXN_CMPLT} -> TNETWARBSM_STATE_IDLE */
            {TNETWARBSM_STATE_IDLE               , TNETWArbSM_ActionUnexpected}                              
        },

        /*
         * State [TNETWARBSM_STATE_WAIT_BUS]
         */
        {
            /* {TNETWARBSM_EV_START} -> TNETWARBSM_STATE_WAIT_BUS */
            {TNETWARBSM_STATE_WAIT_BUS           , TNETWArbSM_ReturnPendingOnStart},                   

            /* {TNETWARBSM_EV_BUS_AVAIL} -> TNETWARBSM_STATE_WAIT_HW */
            {TNETWARBSM_STATE_WAIT_HW            , TNETWArbSM_WakeHw}, 

            /* {TNETWARBSM_EV_HW_AVAIL} -> TNETWARBSM_STATE_WAIT_BUS */
            {TNETWARBSM_STATE_WAIT_BUS           , TNETWArbSM_ReturnPendingOnHwAvail}, 

            /* {TNETWARBSM_EV_RESTART} -> TNETWARBSM_STATE_WAIT_BUS */
            {TNETWARBSM_STATE_WAIT_BUS           , TNETWArbSM_ActionUnexpected}, 

            /* {TNETWARBSM_EV_FINISH} -> TNETWARBSM_STATE_WAIT_BUS */
            {TNETWARBSM_STATE_WAIT_BUS           , TNETWArbSM_ActionUnexpected},  

            /* {TNETWARBSM_EV_TXN_CMPLT} -> TNETWARBSM_STATE_WAIT_BUS */
            {TNETWARBSM_STATE_WAIT_BUS           , TNETWArbSM_ActionUnexpected}                              
        },

        /*
         * State [TNETWARBSM_STATE_WAIT_HW]
         */
        {
            /* {TNETWARBSM_EV_START} -> TNETWARBSM_STATE_WAIT_HW */
            {TNETWARBSM_STATE_WAIT_HW            , TNETWArbSM_ReturnPendingOnStart},

            /* {TNETWARBSM_EV_BUS_AVAIL} -> TNETWARBSM_STATE_WAIT_HW */
            {TNETWARBSM_STATE_WAIT_HW            , TNETWArbSM_DoNothing},

            /* {TNETWARBSM_EV_HW_AVAIL} -> TNETWARBSM_STATE_WAIT_TXN1 */
            {TNETWARBSM_STATE_WAIT_TXN1          , TNETWArbSM_ExitWakeUp}, 

            /* {TNETWARBSM_EV_RESTART} -> TNETWARBSM_STATE_WAIT_HW */
            {TNETWARBSM_STATE_WAIT_HW            , TNETWArbSM_ActionUnexpected},

            /* {TNETWARBSM_EV_FINISH} -> TNETWARBSM_STATE_WAIT_HW */
            {TNETWARBSM_STATE_WAIT_HW            , TNETWArbSM_ActionUnexpected}, 

            /* {TNETWARBSM_EV_TXN_CMPLT} -> TNETWARBSM_STATE_WAIT_TXN1 */
            {TNETWARBSM_STATE_WAIT_TXN1            , TNETWArbSM_WaitIRQHw}                              
        },

        /*
         * State [TNETWARBSM_STATE_WAIT_BUS_AFTER_HW]
         */
        {
            /* {TNETWARBSM_EV_START} -> TNETWARBSM_STATE_WAIT_HW */
            {TNETWARBSM_STATE_WAIT_BUS_AFTER_HW  , TNETWArbSM_ReturnPendingOnStart},

            /* {TNETWARBSM_EV_BUS_AVAIL} -> TNETWARBSM_STATE_WAIT_TXN1 */
            {TNETWARBSM_STATE_WAIT_TXN1          , TNETWArbSM_WaitIRQHw},

            /* {TNETWARBSM_EV_HW_AVAIL} -> TNETWARBSM_STATE_RUNNING */
            {TNETWARBSM_STATE_WAIT_BUS_AFTER_HW  , TNETWArbSM_ReturnPendingOnHwAvail}, 

            /* {TNETWARBSM_EV_RESTART} -> TNETWARBSM_STATE_WAIT_HW */
            {TNETWARBSM_STATE_WAIT_BUS_AFTER_HW  , TNETWArbSM_ActionUnexpected},

            /* {TNETWARBSM_EV_FINISH} -> TNETWARBSM_STATE_WAIT_HW */
            {TNETWARBSM_STATE_WAIT_BUS_AFTER_HW  , TNETWArbSM_ActionUnexpected}, 

            /* {TNETWARBSM_EV_TXN_CMPLT} -> TNETWARBSM_STATE_WAIT_BUS_AFTER_HW */
            {TNETWARBSM_STATE_WAIT_BUS_AFTER_HW  , TNETWArbSM_ActionUnexpected}                              
        },

        /*
         * State [TNETWARBSM_STATE_WAIT_TXN1]
         */
        {
            /* {TNETWARBSM_EV_START} -> TNETWARBSM_STATE_WAIT_TXN1 */
            {TNETWARBSM_STATE_WAIT_TXN1          , TNETWArbSM_ReturnPendingOnStart},

            /* {TNETWARBSM_EV_BUS_AVAIL} -> TNETWARBSM_STATE_WAIT_TXN1 */
            {TNETWARBSM_STATE_WAIT_TXN1          , TNETWArbSM_ActionUnexpected},

            /* {TNETWARBSM_EV_HW_AVAIL} -> TNETWARBSM_STATE_RUNNING */
            {TNETWARBSM_STATE_RUNNING          , TNETWArbSM_RunClientCb}, 

            /* {TNETWARBSM_EV_RESTART} -> TNETWARBSM_STATE_WAIT_TXN1 */
            {TNETWARBSM_STATE_WAIT_TXN1          , TNETWArbSM_ActionUnexpected},

            /* {TNETWARBSM_EV_FINISH} -> TNETWARBSM_STATE_WAIT_TXN1 */
            {TNETWARBSM_STATE_WAIT_TXN1          , TNETWArbSM_ActionUnexpected}, 

            /* {TNETWARBSM_EV_TXN_CMPLT} -> TNETWARBSM_STATE_RUNNING */
            {TNETWARBSM_STATE_RUNNING            , TNETWArbSM_RunClientCb} 
        },

        /*
         * State [TNETWARBSM_STATE_RUNNING]
         */
        {
            /* {TNETWARBSM_EV_START} -> TNETWARBSM_STATE_RUNNING */
            {TNETWARBSM_STATE_RUNNING            , TNETWArbSM_ReturnPendingOnStart},

            /* {TNETWARBSM_EV_BUS_AVAIL} -> TNETWARBSM_STATE_RUNNING */
            {TNETWARBSM_STATE_RUNNING            , TNETWArbSM_ActionUnexpected},

            /* {TNETWARBSM_EV_HW_AVAIL} -> TNETWARBSM_STATE_RUNNING */
            {TNETWARBSM_STATE_RUNNING            , TNETWArbSM_ReturnPendingOnHwAvail},

            /* {TNETWARBSM_EV_RESTART} -> TNETWARBSM_STATE_WAIT_BUS */
            {TNETWARBSM_STATE_WAIT_BUS           , TNETWArbSM_ReseizeBus},

            /* {TNETWARBSM_EV_FINISH} -> TNETWARBSM_STATE_IDLE */
            {TNETWARBSM_STATE_WAIT_TXN2          , TNETWArbSM_PutSleepHw}, 

            /* {TNETWARBSM_EV_TXN_CMPLT} -> TNETWARBSM_STATE_RUNNING */
            {TNETWARBSM_STATE_RUNNING            , TNETWArbSM_ActionUnexpected} 
        },

        /*
         * State [TNETWARBSM_STATE_WAIT_TXN2]
         */
        {
            /* {TNETWARBSM_EV_START} -> TNETWARBSM_STATE_WAIT_TXN2 */
            {TNETWARBSM_STATE_WAIT_TXN2          , TNETWArbSM_ReturnPendingOnStart},

            /* {TNETWARBSM_EV_BUS_AVAIL} -> TNETWARBSM_STATE_WAIT_TXN2 */
            {TNETWARBSM_STATE_WAIT_TXN2          , TNETWArbSM_ActionUnexpected},

            /* {TNETWARBSM_EV_HW_AVAIL} -> TNETWARBSM_STATE_WAIT_TXN2 */
            {TNETWARBSM_STATE_WAIT_TXN2          , TNETWArbSM_ReturnPendingOnHwAvail},

            /* {TNETWARBSM_EV_RESTART} -> TNETWARBSM_STATE_WAIT_TXN2 */
            {TNETWARBSM_STATE_WAIT_TXN2          , TNETWArbSM_ActionUnexpected},

            /* {TNETWARBSM_EV_FINISH} -> TNETWARBSM_STATE_WAIT_TXN2 */
            {TNETWARBSM_STATE_WAIT_TXN2          , TNETWArbSM_ActionUnexpected}, 

            /* {TNETWARBSM_EV_TXN_CMPLT} -> TNETWARBSM_STATE_IDLE */
            {TNETWARBSM_STATE_IDLE               , TNETWArbSM_ReleaseBus} 
        },
    };

    nrfsm_Config (pTNETWArbSM->hFSM,
                  (nrfsm_matrix_t)smMatrix,
                  TNETWARB_SM_STATE_NUM,
                  TNETWARBSM_EVENT_NUM);

    pTNETWArbSM->hReport = hReport;
    pTNETWArbSM->hELPCtrl = hELPCtrl;
    pTNETWArbSM->hBusArbiter = hBusArbiter;
    pTNETWArbSM->hTNETWArb =  hTNETWArb;

    /* TNETWArbSM start in IDLE State (No one is running in the TNETW Arbiter) */    
    nrfsm_SetState (pTNETWArbSM->hFSM, TNETWARBSM_STATE_IDLE);

    WLAN_REPORT_INIT (pTNETWArbSM->hReport, TNETW_ARBITER_MODULE_LOG, ("TNETWArbSM Initialized\n"));

    return OK;
}

#define TNETWArbSM_SMEvent__(hSm,ev) \
    (nrfsm_Event (((TNETWArbSM_t*)hSm)->hFSM, ((TNETWArbSM_t*)hSm)->event = ev, hSm) == OK) \
        ? ((TNETWArbSM_t*)hSm)->SMlastOperationStatus \
        : TNETWIF_PENDING


/**
 * \date 26-Oct-2005\n
 * \brief trigger events from the outside of the module into the state machine.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the TNETWArbSMf object.\n
 * 2) TnetwArbSMEvents_e - the input events to the state machine.
 * Return Value: TI_STATUS - OK on success else NOK.\n
 * \b Description:\n
 * this function will trigger the fsm generic state machine function.
 */
TI_STATUS TNETWArbSM_SMEvent (TI_HANDLE hTNETWArbSM, TnetwArbSMEvents_e event)                          
{
    return TNETWArbSM_SMEvent__ (hTNETWArbSM, event);
}


static TI_STATUS TNETWArbSM_SeizeBusOnStart (TI_HANDLE hTNETWArbSM)
{
    return TNETWArbSM_SeizeBus (hTNETWArbSM, FALSE);
}


static TI_STATUS TNETWArbSM_SeizeBusOnHwAvail (TI_HANDLE hTNETWArbSM)
{
    return TNETWArbSM_SeizeBus (hTNETWArbSM, TRUE);
}


/**
 * \date 26-Oct-2005\n
 * \brief trigger events from the outside of the module into the state machine.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the TNETWArbSMf object.\n
 * 2) TnetwArbSMEvents_e - the input events to the state machine.
 * Return Value: TI_STATUS - OK on success else NOK.\n
 * \b Description:\n
 * this function will trigger the fsm generic state machine function.
 */
static TI_STATUS TNETWArbSM_SeizeBus (TI_HANDLE hTNETWArbSM, BOOL bHwAvail)
{
    TNETWArbSM_t *pTNETWArbSM = (TNETWArbSM_t *)hTNETWArbSM;
    TI_STATUS     status;

    pTNETWArbSM->bHwAvail = bHwAvail;

  #if defined(BUS_GUARD_SUPPORT)
    status = BusArb_SeizeReq (pTNETWArbSM->hBusArbiter,
                              TNETWArbSM_BusAvailCB,
                              hTNETWArbSM);
#else
    /* For now assume that we always have the bus */
    status = TNETWIF_COMPLETE;
#endif

    /* In case we took the Bus then generate the BUS_AVAIL Event to the State Machine */
    if (status == TNETWIF_COMPLETE)
    {
        /* Then the status could be TNETWIF_COMPLETE at this end or also can be TNETWIF_PENDING if the HW is not Awake */
        TNETWArbSM_SMEvent__ (hTNETWArbSM, TNETWARBSM_EV_BUS_AVAIL);
    }
    /* Return ERROR or PENDING to Client */
    else
    {
        pTNETWArbSM->SMlastOperationStatus = TNETWIF_PENDING;
    }

    return OK;
}


/**
 * \date 26-Oct-2005\n
 * \brief trigger events from the outside of the module into the state machine.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the TNETWArbSMf object.\n
 * Return Value: TI_STATUS - OK on success else NOK.\n
 * \b Description:\n
 * this function will trigger the fsm generic state machine function.
 */
#if defined(BUS_GUARD_SUPPORT)
static void TNETWArbSM_BusAvailCB (TI_HANDLE hTNETWArbSM)
{
    TNETWArbSM_SMEvent__ (hTNETWArbSM, TNETWARBSM_EV_BUS_AVAIL);
}
#endif


/**
 * \date 26-Oct-2005\n
 * \brief trigger events from the outside of the module into the state machine.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the TNETWArbSMf object.\n
 * 2) TnetwArbSMEvents_e - the input events to the state machine.
 * Return Value: TI_STATUS - OK on success else NOK.\n
 * \b Description:\n
 * this function will trigger the fsm generic state machine function.
 */
static TI_STATUS TNETWArbSM_WakeHw (TI_HANDLE hTNETWArbSM)
{
    TNETWArbSM_t  *pTNETWArbSM = (TNETWArbSM_t *)hTNETWArbSM;

    /* Call the ELP Controller module to wake the FW */
        switch (elpCtrl_Wake (pTNETWArbSM->hELPCtrl,pTNETWArbSM->bHwAvail))
    {
        case ELPCTRL_AWAKE:
        case ELPCTRL_WLAN_RDY_COMPLETE:

            TNETWArbSM_SMEvent__ (hTNETWArbSM, TNETWARBSM_EV_TXN_CMPLT);
            break;

        case ELPCTRL_WLAN_RDY:

            TNETWArbSM_SMEvent__ (hTNETWArbSM, TNETWARBSM_EV_BUS_AVAIL);
            break;

        case ELPCTRL_ASLEEP:
	case ELPCTRL_COMPLETE:
            pTNETWArbSM->SMlastOperationStatus = TNETWIF_PENDING;
            break;
 
        default:
            pTNETWArbSM->SMlastOperationStatus = TNETWIF_ERROR;
            break;
    }

    return OK;
}


/**
 * \date 26-Oct-2005\n
 * \brief trigger events from the outside of the module into the state machine.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the TNETWArbSMf object.\n
 * Return Value: TI_STATUS - OK on success else NOK.\n
 * \b Description:\n
 * this function will trigger the fsm generic state machine function.
 */
void TNETWArbSM_TxnCb (TI_HANDLE hTNETWArbSM)
{
    TNETWArbSM_SMEvent__ (hTNETWArbSM, TNETWARBSM_EV_TXN_CMPLT);
}


/**
 * \date 26-Oct-2005\n
 * \brief trigger events from the outside of the module into the state machine.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the TNETWArbSMf object.\n
 * 2) TnetwArbSMEvents_e - the input events to the state machine.
 * Return Value: TI_STATUS - OK on success else NOK.\n
 * \b Description:\n
 * this function will do the following:
 *          1) Dequeue from the TODO Queue the first item or process request to start
 *          2) Set the Dequeued instance to be the Running instance
 *          3) Call the Client callback (must be the module start_instance )
 */
static TI_STATUS TNETWArbSM_RunClientCb (TI_HANDLE TNETWArbSM)
{
    TNETWArbSM_t         *pTNETWArbSM = (TNETWArbSM_t *)TNETWArbSM;
    TNETWArb_t           *pTNETWArb = (TNETWArb_t *)pTNETWArbSM->hTNETWArb;
    TNETWARB_INSTANCE_T  *pTNETWARB_Inst;

    /* Mark to ElpCtrl that the IRQ was received and it should get back to AWAKE */
    elpCtrl_ReceivedIRQ (pTNETWArbSM->hELPCtrl);
    
    /* First Dequeue the instance from the TODO Queue */
    pTNETWARB_Inst = (TNETWARB_INSTANCE_T *)TNETWArb_Dequeue 
        (&pTNETWArb->TNETWArbiter_Queues[TODO_LIST_INDEX]);

    WLAN_REPORT_INFORMATION (pTNETWArbSM->hReport,
                             TNETW_ARBITER_MODULE_LOG,
                             ("\n TNETWArbSM_ProcessToDoList: DeQueued pTNETWARB_Inst %x\n", 
                             pTNETWARB_Inst));

     /* If the instance is already allocated then return error to Client caller */
    if (pTNETWARB_Inst == NULL)
    {
        WLAN_REPORT_ERROR (pTNETWArbSM->hReport, 
                           TNETW_ARBITER_MODULE_LOG,
                           ("\nTNETWArbSM_RunClientCb: NULL instance\n")); 

         pTNETWArbSM->SMlastOperationStatus = TNETWIF_ERROR;

         return OK;
    }

    /* Set it to be the Running instance right now */
    pTNETWArb->TNETWArb_Running_instance = pTNETWARB_Inst;

    /* Call the Client Callback that must here be the start function of the module */
    pTNETWArbSM->SMlastOperationStatus = TNETWArb_CallClientCallback (pTNETWArbSM->hTNETWArb);

    return OK;
}


/**
 * \date 26-Oct-2005\n
 * \brief trigger events from the outside of the module into the state machine.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the TNETWArbSMf object.\n
 * 2) TnetwArbSMEvents_e - the input events to the state machine.
 * Return Value: TI_STATUS - OK on success else NOK.\n
 * \b Description:\n
 * this function will trigger the fsm generic state machine function.
 */
static TI_STATUS TNETWArbSM_ReseizeBus (TI_HANDLE hTNETWArbSM)
{
    TNETWArbSM_t    *pTNETWArbSM = (TNETWArbSM_t *)hTNETWArbSM;
    TI_STATUS        status;

  #if defined(BUS_GUARD_SUPPORT)
    status = BusArb_ReseizeReq (pTNETWArbSM->hBusArbiter,
                                TNETWArbSM_BusAvailCB,
                                hTNETWArbSM);

  #else
    /* For now assume that we always have the bus */
    status = TNETWIF_COMPLETE;
  #endif

    /* In the case we did not get the Bus now for second time then release the HW till we will get the Callback from the Bus ARbiter */
    if (status == TNETWIF_PENDING)
    {
        pTNETWArbSM->SMlastOperationStatus = TNETWIF_PENDING;
    }
    /* If we got TNETWIF_COMPLETE then we could get the Bus Semaphore again then send the BUS_AVAIL Event */
    else if (status == TNETWIF_COMPLETE)
    {
        /* Get the status again through sending the BUS_AVAIL Event to the TNETW ARbiter SM */
        /* Then the status could be TNETWIF_COMPLETE at this end or also can be TNETWIF_PENDING if the HW is not Awake */
        TNETWArbSM_SMEvent__ (hTNETWArbSM, TNETWARBSM_EV_BUS_AVAIL);
    }
    else 
    {
        pTNETWArbSM->SMlastOperationStatus = TNETWIF_ERROR;
    }

    return OK;
}


/**
 * \date 26-Oct-2005\n
 * \brief trigger events from the outside of the module into the state machine.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the TNETWArbSMf object.\n
 * 2) TnetwArbSMEvents_e - the input events to the state machine.
 * Return Value: TI_STATUS - OK on success else NOK.\n
 * \b Description:\n
 * this function will trigger the fsm generic state machine function.
 */
static TI_STATUS TNETWArbSM_PutSleepHw (TI_HANDLE hTNETWArbSM)
{
    TNETWArbSM_t    *pTNETWArbSM = (TNETWArbSM_t *)hTNETWArbSM;

    pTNETWArbSM->bHwAvail = FALSE;

    /* The HW is not needed anymore then the FW can go to sleep */
    if (elpCtrl_Sleep (pTNETWArbSM->hELPCtrl) == TNETWIF_COMPLETE)
    {
         return TNETWArbSM_SMEvent__ (hTNETWArbSM, TNETWARBSM_EV_TXN_CMPLT);

    }
  
    pTNETWArbSM->SMlastOperationStatus = TNETWIF_PENDING;

    return OK;
}


static TI_STATUS TNETWArbSM_WaitIRQHw (TI_HANDLE hTNETWArbSM)
{
    TNETWArbSM_t    *pTNETWArbSM = (TNETWArbSM_t *)hTNETWArbSM;

    /* check whether IRQ is about to happen or not, depending on the ElpCtrl Mux state */
    if ( elpCtrl_isIRQComing (pTNETWArbSM->hELPCtrl) )
{
        /* Please be patient - IRQ is about to come any minute */
        pTNETWArbSM->SMlastOperationStatus = TNETWIF_PENDING;
    } 
    else /* No IRQ is supposed to arrive - roll SM to next position */
    {
        TNETWArbSM_SMEvent__ (hTNETWArbSM, TNETWARBSM_EV_HW_AVAIL);
    }

    return OK;
}

static TI_STATUS TNETWArbSM_ExitWakeUp (TI_HANDLE hTNETWArbSM)
{
    TNETWArbSM_t    *pTNETWArbSM = (TNETWArbSM_t *)hTNETWArbSM;

    /* Inform ElpCtrl that it should exit the wake up sequence */
    if ( elpCtrl_exitWakeUpSeq (pTNETWArbSM->hELPCtrl) == ELPCTRL_AWAKE)
    {   
        /* We are already awake - send TXN_COMPLETE to roll SM forward */
        TNETWArbSM_SMEvent__ (hTNETWArbSM, TNETWARBSM_EV_TXN_CMPLT);
    } 
    else /* Wait till we get TXN_COMPLETE event */
    {
        pTNETWArbSM->SMlastOperationStatus = TNETWIF_PENDING;
}

    return OK;
}

static TI_STATUS TNETWArbSM_ReleaseBus (TI_HANDLE hTNETWArbSM)
{
    TNETWArbSM_t  *pTNETWArbSM = (TNETWArbSM_t *)hTNETWArbSM;
    TNETWArb_t    *pTNETWArb   = (TNETWArb_t *)pTNETWArbSM->hTNETWArb;

    if (TNETWArb_getfirst (&pTNETWArb->TNETWArbiter_Queues[TODO_LIST_INDEX]))
    {
        TNETWArbSM_SMEvent__ (hTNETWArbSM, TNETWARBSM_EV_RESTART);
    }

    else
    {
        /* The Bus is not needed anymore */
      #if defined(BUS_GUARD_SUPPORT)
        return BusArb_ReleaseReq (pTNETWArbSM->hBusArbiter, hTNETWArbSM);
      #else 
        pTNETWArbSM->SMlastOperationStatus = TNETWIF_COMPLETE;
      #endif
    }

    return OK;
}



/**
 * \date 26-Oct-2005\n
 * \brief trigger events from the outside of the module into the state machine.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the TNETWArbSMf object.\n
 * Return Value: TI_STATUS - OK on success else NOK.\n
 * \b Description:\n
 * this function will trigger the fsm generic state machine function.
 */
static TI_STATUS TNETWArbSM_ReturnPendingOnStart (TI_HANDLE hTNETWArbSM)
{
    TNETWArbSM_t  *pTNETWArbSM = (TNETWArbSM_t *)hTNETWArbSM;

    /* 
     * In this case return PENDING to the client
     * meaning that the client will not run immediately.
     * (Bus Busy/Hw not awake or other client running )
     */
    pTNETWArbSM->SMlastOperationStatus = TNETWIF_PENDING;

    return OK;
}


static TI_STATUS TNETWArbSM_ReturnPendingOnHwAvail (TI_HANDLE hTNETWArbSM)
{
    TNETWArbSM_t  *pTNETWArbSM = (TNETWArbSM_t *)hTNETWArbSM;

    pTNETWArbSM->bHwAvail = TRUE;   

    pTNETWArbSM->SMlastOperationStatus = TNETWIF_PENDING;

    return OK;
}


static TI_STATUS TNETWArbSM_DoNothing (TI_HANDLE hTNETWArbSM)
{
    TNETWArbSM_t  *pTNETWArbSM = (TNETWArbSM_t *)hTNETWArbSM;

    pTNETWArbSM->SMlastOperationStatus = TNETWIF_PENDING;

    return OK;
}


/**
 * \date 26-Oct-2005\n
 * \brief The Event received by the TNETW Arbiter Sm at this specific state is not expected.
 *
 * Function Scope \e Public.\n
 * Parameters:\n
 * 1) TI_HANDLE - handle to the TNETWArbSMf object.\n
 * Return Value: TI_STATUS - TNETWIF_ERROR
 * \b Description:\n
 * This function will only set a Log to indicate that this event is not expected in the SM
 */
TI_STATUS TNETWArbSM_ActionUnexpected (TI_HANDLE hTNETWArbSM)
{
    TNETWArbSM_t    *pTNETWArbSM = (TNETWArbSM_t *)hTNETWArbSM;
    UINT32           state;

    if (nrfsm_GetState (pTNETWArbSM->hFSM, &state) == OK) 
    {
        WLAN_REPORT_ERROR (pTNETWArbSM->hReport, 
                           TNETW_ARBITER_MODULE_LOG,
                           ("\nTNETWArbSM_ActionUnexpected: state=%u, event=%u\n", 
                           state,
                           pTNETWArbSM->event));
    }

    pTNETWArbSM->SMlastOperationStatus = TNETWIF_ERROR;

    return OK;
}
