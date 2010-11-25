/** \file PowerSrvSM.c
 *  \brief This is the PowerSrvSM module implementation.
 *  \author Assaf Azulay
 *  \date 19-OCT-2005
 */
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
 *                                                                          *
 *   MODULE:  PowerSrvSM                                                    *
 *   PURPOSE: PowerSrvSM Module implementation.                             *
 *                                                                          *
 ****************************************************************************/

#include "osTIType.h"
#include "osApi.h"
#include "commonTypes.h"
#include "fsm.h"
#include "report.h"
#include "PowerSrvSM.h"
#include "whalCtrl_api.h"


/*****************************************************************************
 **         Defines                                                         **
 *****************************************************************************/



/*****************************************************************************
 **         structs                                                         **
 *****************************************************************************/
#ifdef TI_DBG
static char stateDesc[POWER_SRV_SM_STATE_NUM][MAX_DESC_STRING_LEN] =
{
    "POWER_SRV_STATE_ACTIVE ",
    "POWER_SRV_STATE_PEND_PS" ,
    "POWER_SRV_STATE_PS" ,
    "POWER_SRV_STATE_PEND_ACTIVE" ,
    "POWER_SRV_STATE_ERROR_ACTIVE"
};



static char eventDesc[POWER_SRV_SM_EVENT_NUM][MAX_DESC_STRING_LEN] =
{
    "POWER_SRV_EVENT_REQUEST_ACTIVE" , 
    "POWER_SRV_EVENT_REQUEST_PS" ,
    "POWER_SRV_EVENT_SUCCESS",
    "POWER_SRV_EVENT_FAIL" ,

};
#endif /* TI_DBG */
/*****************************************************************************
 **         Private Function prototypes                                     **
 *****************************************************************************/

static TI_STATUS powerSrvSmSMEvent(UINT8* pCurrentState,
                                   UINT8 event,
                                   TI_HANDLE hPowerSrvSM);
static TI_STATUS powerSrvSmDoUpdateRequest(TI_HANDLE hPowerSrvSM);
static TI_STATUS powerSrvSmDoEnterPowerSave(TI_HANDLE hPowerSrvSM);
static TI_STATUS powerSrvSmDoExitPowerSave(TI_HANDLE hPowerSrvSM);
static TI_STATUS powerSrvSmDoPending(TI_HANDLE hPowerSrvSM);
static TI_STATUS powerSrvSmDoAllready(TI_HANDLE hPowerSrvSM);
static TI_STATUS powerSrvSMActionUnexpected(TI_HANDLE hPowerSrvSM);
static TI_STATUS powerSrvSMSendMBXConfiguration(TI_HANDLE hPowerSrvSM, BOOLEAN PS_disableEnable);

/***************************************************************************************
 **         Public Function prototypes                                      **
 ****************************************************************************************/

/****************************************************************************************
*                               powerSrvSMTimerExpired                                  *
*****************************************************************************************
DESCRIPTION: This function is called upon timer expiry - when the FW has not returned
a response within the defined tme (50 ms)

INPUT:      - hPowerSrvSM                       - handle to the PowerSrvSM object.
OUTPUT:    None
RETURN:    None
****************************************************************************************/
void powerSrvSMTimerExpired( TI_HANDLE hPowerSrvSM )
{
	PowerSrvSM_t *pPowerSrvSM = (PowerSrvSM_t*)hPowerSrvSM;

	/* Print an error message */
	WLAN_REPORT_ERROR( pPowerSrvSM->hReport, POWER_SERVER_MODULE_LOG,
		("%s(%d) - PS guard timer expired!\n",
		__FILE__,__LINE__));

	/* Call the error notification callback (triggering recovery) */
	pPowerSrvSM->failureEventCB( pPowerSrvSM->hFailureEventObj ,POWER_SAVE_FAILURE );
}

/****************************************************************************************
 *                        powerSrvSM_create                                                         *
 ****************************************************************************************
DESCRIPTION: Power Server SM module creation function, called by the Power Server create in creation phase 
                performs the following:
                -   Allocate the Power Server SM handle
                -   Creates the fsm.
                                                                                                                   
INPUT:          - hOs - Handle to OS        


OUTPUT:     

RETURN:     Handle to the Power Server SM module on success, NULL otherwise
****************************************************************************************/
TI_HANDLE powerSrvSM_create(TI_HANDLE hOsHandle)
{
    PowerSrvSM_t *pPowerSrvSM = NULL;
    fsm_stateMachine_t *pFsm = NULL;
    TI_STATUS status;

    pPowerSrvSM = (PowerSrvSM_t*) os_memoryAlloc (hOsHandle, sizeof(PowerSrvSM_t));
    if ( pPowerSrvSM == NULL )
    {
        WLAN_OS_REPORT(("%s(%d) - Memory Allocation Error!\n",__FILE__,__LINE__));
        return NULL;
    }

    os_memoryZero (hOsHandle, pPowerSrvSM, sizeof(PowerSrvSM_t));

    pPowerSrvSM->hOS = hOsHandle;

    /* create the generic state-machine */
    status = fsm_Create(hOsHandle,
                        &pFsm,
                        (UINT8)POWER_SRV_SM_STATE_NUM,
                        (UINT8)POWER_SRV_SM_EVENT_NUM);
    if ( status != OK )
    {
        WLAN_OS_REPORT(("%s(%d) - Error in create FSM!\n",__FILE__,__LINE__));
        powerSrvSM_destroy(pPowerSrvSM);
        return NULL;
    }

    /* create the timer */
    pPowerSrvSM->hTimer = os_timerCreate( hOsHandle, powerSrvSMTimerExpired, (TI_HANDLE)pPowerSrvSM );
    if ( NULL == pPowerSrvSM->hTimer )
    {
        WLAN_OS_REPORT( ("%s(%d) - Failed to create Power Save SRV timer\n", __FILE__,__LINE__) );
        powerSrvSM_destroy(pPowerSrvSM);
        return NULL;
    }


    pPowerSrvSM->hFSM = (TI_HANDLE)pFsm;

    return pPowerSrvSM;
}

 
/****************************************************************************************
 *                        powerSrvSM_destroy                                                            *
 ****************************************************************************************
DESCRIPTION: Power Server SM module destroy function, 
                -   delete Power Server SM allocation
                
                                                                                                                   
INPUT:          - hPowerSrvSM - Handle to the Power Server  SM


OUTPUT:     

RETURN:    TI_STATUS - OK on success else NOK.
****************************************************************************************/
TI_STATUS powerSrvSM_destroy(TI_HANDLE hPowerSrvSM)
{
    PowerSrvSM_t *pPowerSrvSM = (PowerSrvSM_t*)hPowerSrvSM;

    TI_HANDLE osHandle = pPowerSrvSM->hOS;

    /* free the timer */
    if ( NULL != pPowerSrvSM->hTimer )
    {
        os_timerDestroy( osHandle, pPowerSrvSM->hTimer );
    }

    /* free the generic SM */
    if ( pPowerSrvSM->hFSM != NULL )
    {
        fsm_Unload(osHandle,
                   (fsm_stateMachine_t*)pPowerSrvSM->hFSM);
    }

    /* free the Power Save SRV object */
    os_memoryFree(osHandle , pPowerSrvSM , sizeof(PowerSrvSM_t));

    WLAN_OS_REPORT(("%s(%d) - PowerSrvSM destroyed\n",__FILE__,__LINE__));

    return OK;
}


/****************************************************************************************
*                        powerSrvSM_init                                                           *
****************************************************************************************
DESCRIPTION: Power Server SM module initialize function, called by the Power Server init in configure phase 
               performs the following:
               -   init the Stet machine states.
               -   set Active as start state.
                                                                                                                  
INPUT:      - hPowerSrvSM       - handle to the PowerSrvSM object.
           - hReport           - handle to the Report object.
           - hWhalCtrl         - handle to the WhalCtrl object.    

OUTPUT: 
RETURN:    TI_STATUS - OK on success else NOK.
****************************************************************************************/
TI_STATUS powerSrvSM_init(TI_HANDLE hPowerSrvSM,
                          TI_HANDLE hReport,
                          TI_HANDLE hWhalCtrl)
{
    PowerSrvSM_t *pPowerSrvSM = (PowerSrvSM_t*)hPowerSrvSM;


    fsm_actionCell_t smMatrix[POWER_SRV_SM_STATE_NUM][POWER_SRV_SM_EVENT_NUM] =
    {
        /*
        next state and transition action for POWER_SRV_STATE_ACTIVE state
        */
        {
            /* POWER_SRV_EVENT_REQUEST_ACTIVE */
            {POWER_SRV_STATE_ACTIVE             , powerSrvSmDoAllready},

            /* POWER_SRV_EVENT_REQUEST_PS */
            {POWER_SRV_STATE_PEND_PS                , powerSrvSmDoEnterPowerSave},

            /* POWER_SRV_EVENT_SUCCESS */
            {POWER_SRV_STATE_ACTIVE                 , powerSrvSMActionUnexpected},

            /* POWER_SRV_EVENT_FAIL */
            {POWER_SRV_STATE_ACTIVE                 , powerSrvSMActionUnexpected}

        },

        /*
        next state and transition action for POWER_SRV_STATE_PEND_PS state
        */
        {
            /* POWER_SRV_EVENT_REQUEST_ACTIVE */
            {POWER_SRV_STATE_PEND_PS            , powerSrvSmDoPending},

            /* POWER_SRV_EVENT_REQUEST_PS */
            {POWER_SRV_STATE_PEND_PS        , powerSrvSmDoPending},

            /* POWER_SRV_EVENT_SUCCESS */
            {POWER_SRV_STATE_PS                 , powerSrvSmDoUpdateRequest},

            /* POWER_SRV_EVENT_FAIL */
            {POWER_SRV_STATE_ACTIVE             , powerSrvSmDoUpdateRequest}

        },
        /*
        next state and transition action for POWER_SRV_STATE_PS state
        */
        {
            /* POWER_SRV_EVENT_REQUEST_ACTIVE */
            {POWER_SRV_STATE_PEND_ACTIVE        , powerSrvSmDoExitPowerSave},

            /* POWER_SRV_EVENT_REQUEST_PS */
            {POWER_SRV_STATE_PS                 , powerSrvSmDoAllready},

            /* POWER_SRV_EVENT_SUCCESS */
            {POWER_SRV_STATE_PS                 , powerSrvSMActionUnexpected},

            /* POWER_SRV_EVENT_FAIL */
            {POWER_SRV_STATE_PS                 , powerSrvSMActionUnexpected}

        },
        /*
        next state and transition action for POWER_SRV_STATE_PEND_ACTIVE state
        */
        {
            /* POWER_SRV_EVENT_REQUEST_ACTIVE */
            {POWER_SRV_STATE_PEND_ACTIVE            , powerSrvSmDoPending},

            /* POWER_SRV_EVENT_REQUEST_PS */
            {POWER_SRV_STATE_PEND_ACTIVE        , powerSrvSmDoPending},

            /* POWER_SRV_EVENT_SUCCESS */
            {POWER_SRV_STATE_ACTIVE             , powerSrvSmDoUpdateRequest},

            /* POWER_SRV_EVENT_FAIL */
            {POWER_SRV_STATE_ERROR_ACTIVE       , powerSrvSmDoUpdateRequest}

        },
        /*
        next state and transition action for POWER_SRV_STATE_ERROR_ACTIVE state
        */
        {
            /* POWER_SRV_EVENT_REQUEST_ACTIVE */
            {POWER_SRV_STATE_PEND_ACTIVE            , powerSrvSmDoExitPowerSave},

            /* POWER_SRV_EVENT_REQUEST_PS */
            {POWER_SRV_STATE_PEND_PS        , powerSrvSmDoEnterPowerSave},

            /* POWER_SRV_EVENT_SUCCESS */
            {POWER_SRV_STATE_ERROR_ACTIVE       , powerSrvSMActionUnexpected},

            /* POWER_SRV_EVENT_FAIL */
            {POWER_SRV_STATE_ERROR_ACTIVE       , powerSrvSMActionUnexpected}

        },

    };

    fsm_Config(pPowerSrvSM->hFSM,
               (fsm_Matrix_t)smMatrix,
               POWER_SRV_SM_STATE_NUM,
               POWER_SRV_SM_EVENT_NUM,
               powerSrvSmSMEvent,
               pPowerSrvSM->hOS);

    pPowerSrvSM->hReport = hReport;
    pPowerSrvSM->hWhalCtrl = hWhalCtrl;



    /*
    the PowerSrvSM start in active mode (POWER_SRV_STATE_ACTIVE)
    the PowerSrvSM::currentState must be sync with the PowerSrv::desiredPowerModeProfile (POWER_MODE_ACTIVE).
    */
    pPowerSrvSM->currentState = POWER_SRV_STATE_ACTIVE;


    /*
    Null packet rate : 2,5.5 M
    Probe Request : Not PBCC modulation, Long Preamble */
    pPowerSrvSM->NullPktRateModulation= (DRV_RATE_MASK_1_BARKER | DRV_RATE_MASK_2_BARKER); 





    WLAN_REPORT_INIT(pPowerSrvSM->hReport,
                     POWER_SERVER_MODULE_LOG,
                     ("%s(%d) - PowerSrvSM Initialized\n",__FILE__,__LINE__));


    return OK;

}

/****************************************************************************************
*                        powerSrvSM_config                                                         *
****************************************************************************************
DESCRIPTION: Power Server SM module configuration function, called by the Power Server init in configure phase 
               performs the following:
               -   init the Stet machine states.
               -   set Active as start state.
                                                                                                                  
INPUT:      - hPowerSrvSM       - handle to the PowerSrvSM object.  
           - pPowerSrvInitParams   - the Power Server initialize parameters.

OUTPUT: 
RETURN:    TI_STATUS - OK on success else NOK.
****************************************************************************************/
TI_STATUS powerSrvSM_config(TI_HANDLE hPowerSrvSM,
                            PowerSrvInitParams_t *pPowerSrvInitParams)
{
    PowerSrvSM_t *pPowerSrvSM = (PowerSrvSM_t*)hPowerSrvSM;

    /*
    init PowerMgmtConfigration
    */
    pPowerSrvSM->hangOverPeriod =   pPowerSrvInitParams->hangOverPeriod;
    pPowerSrvSM->numNullPktRetries =    pPowerSrvInitParams->numNullPktRetries;

    return OK;
}
/****************************************************************************************
 *                        powerSrvSM_SMApi                                                           *
 *****************************************************************************************
DESCRIPTION: This function triggers events from the outside of the module into the state machine.
              
                                                                                                                                                                       
INPUT:      - hPowerSrvSM                       - handle to the PowerSrvSM object.  
            - theSMEvent                    - event from whal control.
            

OUTPUT: 
RETURN:    TI_STATUS OK / PENDING / NOK
****************************************************************************************/
TI_STATUS powerSrvSM_SMApi(TI_HANDLE hPowerSrvSM,
                           PowerSrvSMEvents_e theSMEvent)
{
    PowerSrvSM_t *pPowerSrvSM = (PowerSrvSM_t*)hPowerSrvSM;
    TI_STATUS status;

    switch ( theSMEvent )
    {
    case POWER_SRV_EVENT_REQUEST_ACTIVE :
    case POWER_SRV_EVENT_REQUEST_PS :
    case POWER_SRV_EVENT_FAIL :
    case POWER_SRV_EVENT_SUCCESS :

        WLAN_REPORT_INFORMATION(pPowerSrvSM->hReport,
                                POWER_SERVER_MODULE_LOG,
                                ("powerSrvSM_SMApi(%d) called - legal input parameter.",theSMEvent));
        break;

    default:
        WLAN_REPORT_WARNING(pPowerSrvSM->hReport,
                            POWER_SERVER_MODULE_LOG,
                            ("powerSrvSM_SMApi(%d) called, \
                             input parameter is illegal.",theSMEvent));
        return NOK;
    }


    status = powerSrvSmSMEvent((UINT8*)&pPowerSrvSM->currentState,
                               (UINT8)theSMEvent,
                               hPowerSrvSM);

    return status;
}


/****************************************************************************************
 *                        powerSrvSm_setSmRequest                                                    *
 *****************************************************************************************
DESCRIPTION: This function sets the current SM working request.
                                                                                                                   
INPUT:      - hPowerSrvSM                       - handle to the PowerSrvSM object.
            -powerSrvRequest_t*                 - pointer to the correct request in the Power server.

OUTPUT: 
RETURN:    TI_STATUS -  OK
****************************************************************************************/
TI_STATUS powerSrvSm_setSmRequest(TI_HANDLE hPowerSrvSM,powerSrvRequest_t* pSmRequest)
{
    PowerSrvSM_t *pPowerSrvSM = (PowerSrvSM_t*)hPowerSrvSM;
    pPowerSrvSM->pSmRequest = pSmRequest;
    return OK;
}


/****************************************************************************************
 *                        powerSrvSM_getCurrentState                                                         *
 *****************************************************************************************
DESCRIPTION: This function returns the current state of the SM.
                                                       
                                                                                                                   
INPUT:      - hPowerSrvSM                       - handle to the PowerSrvSM object.      
            

OUTPUT: 
RETURN:    PowerSrvSMStates_e current state
****************************************************************************************/
PowerSrvSMStates_e powerSrvSM_getCurrentState(TI_HANDLE hPowerSrvSM)
{
    PowerSrvSM_t *pPowerSrvSM = (PowerSrvSM_t*)hPowerSrvSM;

    return pPowerSrvSM->currentState; 
}

/****************************************************************************************
 *                        powerSrvSM_setRateModulation                                               *
 *****************************************************************************************
DESCRIPTION: This function sets the Rate Modulation
                                                       
                                                                                                                   
INPUT:      - hPowerSrvSM                       - handle to the PowerSrvSM object.      
            - rateModulation                        - desired rate

OUTPUT: 
RETURN:      void
****************************************************************************************/

void powerSrvSM_setRateModulation(TI_HANDLE hPowerSrvSM, UINT16 rateModulation)
{
    PowerSrvSM_t *pPowerSrvSM = (PowerSrvSM_t*)hPowerSrvSM;
    pPowerSrvSM->NullPktRateModulation= rateModulation; 
}

/****************************************************************************************
 *                        powerSrvSM_getRateModulation                                               *
 *****************************************************************************************
DESCRIPTION: This function sets the Rate Modulation
                                                       
                                                                                                                   
INPUT:      - hPowerSrvSM                       - handle to the PowerSrvSM object.      

OUTPUT: 
RETURN:      -  desired rate
****************************************************************************************/

UINT16 powerSrvSM_getRateModulation(TI_HANDLE hPowerSrvSM)
{
    PowerSrvSM_t *pPowerSrvSM = (PowerSrvSM_t*)hPowerSrvSM;
    return pPowerSrvSM->NullPktRateModulation;
}

/****************************************************************************************
 *                        powerSrvSM_printObject                                                         *
 *****************************************************************************************
DESCRIPTION: This function prints the SM object
                                                       
                                                                                                                   
INPUT:      - hPowerSrvSM                       - handle to the PowerSrvSM object.      
            

OUTPUT: 
RETURN:   void
****************************************************************************************/
void powerSrvSM_printObject(TI_HANDLE hPowerSrvSM)
{
    PowerSrvSM_t *pPowerSrvSM = (PowerSrvSM_t*)hPowerSrvSM;
    char *pString;

    WLAN_OS_REPORT(("\n+++++ powerSrvSM_printObject +++++\n"));

    WLAN_OS_REPORT(("Handle to the WhalCtrl is 0x%08X\n", pPowerSrvSM->hWhalCtrl));

    WLAN_OS_REPORT(("Handle to the OS is 0x%08X\n", pPowerSrvSM->hOS));

    WLAN_OS_REPORT(("Handle to the Report is 0x%08X\n", pPowerSrvSM->hReport));

    WLAN_OS_REPORT(("Handle to the FSM is 0x%08X\n", pPowerSrvSM->hFSM));


    switch ( pPowerSrvSM->currentState )
    {
    case POWER_SRV_STATE_ACTIVE:
        pString = "POWER_SRV_STATE_ACTIVE";
        break;

    case POWER_SRV_STATE_PEND_PS:
        pString = "POWER_SRV_STATE_PEND_PS";
        break;

    case POWER_SRV_STATE_PS:
        pString = "POWER_SRV_STATE_PS";
        break;

    case POWER_SRV_STATE_PEND_ACTIVE:
        pString = "POWER_SRV_STATE_PEND_ACTIVE";
        break;

    case POWER_SRV_STATE_ERROR_ACTIVE:
        pString = "POWER_SRV_STATE_ERROR_ACTIVE";
        break;


    default:
        pString = "UNKWON PARAMETER";
        break;
    }
    WLAN_OS_REPORT(("The current state of the state machine is %s (=%d)\n",
                    pString,
                    pPowerSrvSM->currentState));

}




/*****************************************************************************
 **         Private Function prototypes                                                             **
 *****************************************************************************/






/****************************************************************************************
 *                        powerSrvSmDoEnterPowerSave                                                 *
 *****************************************************************************************
DESCRIPTION: This function is an action of the state machine to move from active state to PS
                                                                                                                   
INPUT:      - hPowerSrvSM                       - handle to the PowerSrvSM object.

OUTPUT: 
RETURN:    TI_STATUS - OK / NOK
****************************************************************************************/

static TI_STATUS powerSrvSmDoEnterPowerSave(TI_HANDLE hPowerSrvSM)
{
    TI_STATUS status;
    PowerSrvSM_t *pPowerSrvSM = (PowerSrvSM_t*)hPowerSrvSM;
    pPowerSrvSM->pSmRequest->requestState = RUNNING_REQUEST;
    status = powerSrvSMSendMBXConfiguration(hPowerSrvSM, TRUE);
    return status;
}


/****************************************************************************************
 *                        powerSrvSmDoExitPowerSave                                              *
 *****************************************************************************************
DESCRIPTION: This function is an action of the state machine to move from PS state to Active
                                                                                                                   
INPUT:      - hPowerSrvSM                       - handle to the PowerSrvSM object.

OUTPUT: 
RETURN:    TI_STATUS - OK / NOK
****************************************************************************************/
static TI_STATUS powerSrvSmDoExitPowerSave(TI_HANDLE hPowerSrvSM)
{
    TI_STATUS status;
    PowerSrvSM_t *pPowerSrvSM = (PowerSrvSM_t*)hPowerSrvSM;
    pPowerSrvSM->pSmRequest->requestState = RUNNING_REQUEST;
    status = powerSrvSMSendMBXConfiguration(hPowerSrvSM, FALSE);
    return status;
}


/****************************************************************************************
 *                        powerSrvSmDoUpdateRequest                                                  *
 *****************************************************************************************
DESCRIPTION: This function is an action of the state machine to update a request when the SM 
              is already in the requested state is already 
                                                                                                                   
INPUT:      - hPowerSrvSM                       - handle to the PowerSrvSM object.

OUTPUT: 
RETURN:    TI_STATUS - OK / NOK
****************************************************************************************/

static TI_STATUS powerSrvSmDoUpdateRequest(TI_HANDLE hPowerSrvSM)
{
    PowerSrvSM_t *pPowerSrvSM = (PowerSrvSM_t*)hPowerSrvSM;

    /* request has completed - stop the guard timer */
    os_timerStop( pPowerSrvSM->hOS, pPowerSrvSM->hTimer );

    /*powerSrv_SetRequestState  will update the correct request (acording to the current active request)*/
    if ( pPowerSrvSM->pSmRequest->requestState == RUNNING_REQUEST )
    {
        pPowerSrvSM->pSmRequest->requestState = HANDLED_REQUEST;
    }

    return OK;
}


/****************************************************************************************
 *                        powerSrvSmDoPending                                                        *
 *****************************************************************************************
DESCRIPTION: This function is an action of the state machine returns Pending in case that there is a request 
              waiting to be finished (already sent to FW)
                                                                                                                   
INPUT:      - hPowerSrvSM                       - handle to the PowerSrvSM object.

OUTPUT: 
RETURN:    TI_STATUS - PENDING
****************************************************************************************/

static TI_STATUS powerSrvSmDoPending(TI_HANDLE hPowerSrvSM)
{
    PowerSrvSM_t *pPowerSrvSM = (PowerSrvSM_t*)hPowerSrvSM;

    /*powerSrv_SetRequestState will check the mode and will update the correct request (Driver of user)*/
    pPowerSrvSM->pSmRequest->requestState = PENDING_REQUEST;
    return PENDING;

}



/****************************************************************************************
 *                        powerSrvSmDoAllready                                                       *
 *****************************************************************************************
DESCRIPTION: This function is an action of the state machine stays in the same state since it the requested
              one in the request
                                                                                                                   
INPUT:      - hPowerSrvSM                       - handle to the PowerSrvSM object.

OUTPUT: 
RETURN:    TI_STATUS - OK
****************************************************************************************/
static TI_STATUS powerSrvSmDoAllready(TI_HANDLE hPowerSrvSM)
{
    PowerSrvSM_t *pPowerSrvSM = (PowerSrvSM_t*)hPowerSrvSM;

    /*powerSrv_SetRequestState will check the mode and will update the correct request (Driver of user)*/
    pPowerSrvSM->pSmRequest->requestState = HANDLED_REQUEST;
    return POWER_SAVE_802_11_IS_CURRENT;
}


/****************************************************************************************
 *                        powerSrvSMActionUnexpected                                                 *
 *****************************************************************************************
DESCRIPTION: This function is an action of the state machine stays in the same state and return that action
              was not expected
                                                                                                                   
INPUT:      - hPowerSrvSM                       - handle to the PowerSrvSM object.

OUTPUT: 
RETURN:    TI_STATUS - OK
****************************************************************************************/
static TI_STATUS powerSrvSMActionUnexpected(TI_HANDLE hPowerSrvSM)
{
#ifdef TI_DBG
    PowerSrvSM_t *pPowerSrvSM = (PowerSrvSM_t*)hPowerSrvSM;

    WLAN_REPORT_ERROR(pPowerSrvSM->hReport,
                      POWER_SERVER_MODULE_LOG,
                      ("called: powerSrvSMActionUnexpected"));
#endif /* TI_DBG */

    return OK;
}


/****************************************************************************************
 *                        powerSrvSmSMEvent                                                      *
 *****************************************************************************************
DESCRIPTION: This function is the manager of the state macine. its move the state machine
              from one state to the other depend on the receive event, and call to the appropriate
              action (function) for the move between the states.
                                                                                                                   
INPUT:      - pCurrentState
            - event
            - hPowerSrvSM                       - handle to the PowerSrvSM object.

OUTPUT: 
RETURN:    TI_STATUS 
****************************************************************************************/
static TI_STATUS powerSrvSmSMEvent(UINT8* pCurrentState,
                                   UINT8 event,
                                   TI_HANDLE hPowerSrvSM)
{
    PowerSrvSM_t *pPowerSrvSM = (PowerSrvSM_t*)hPowerSrvSM;
    TI_STATUS status = OK;
    UINT8 nextState;

    status = fsm_GetNextState((fsm_stateMachine_t*)pPowerSrvSM->hFSM,
                              (UINT8)pPowerSrvSM->currentState,
                              event,
                              &nextState);
    if ( status != OK )
    {
        WLAN_REPORT_SM(pPowerSrvSM->hReport,
                       POWER_SERVER_MODULE_LOG,
                       ("PowerSrvSM - State machine error, failed getting next state\n"));
        return(status);
    }


#ifdef TI_DBG
    WLAN_REPORT_SM(pPowerSrvSM->hReport,
                   POWER_SERVER_MODULE_LOG,
                   ("PowerSrvSM <state[%s] event[%s]> --> state[%s]\n\n",
                    stateDesc[*pCurrentState],
                    eventDesc[event],
                    stateDesc[nextState]));
#endif

    status = fsm_Event(pPowerSrvSM->hFSM,
                       pCurrentState,
                       event,
                       (void*)pPowerSrvSM);

    return status;
}


/****************************************************************************************
*                        powerSrvSMSendMBXConfiguration                                             *
*****************************************************************************************
DESCRIPTION: This function send configuration of the power save option that holds in the command
                mailbox inner sturcture.
                                                                                                                  
INPUT:      - hPowerSrvSM                       - handle to the PowerSrvSM object.
           - PS_disableEnable                      - true = PS , false = active

OUTPUT: 
RETURN:    TI_STATUS 
****************************************************************************************/
static TI_STATUS    powerSrvSMSendMBXConfiguration(TI_HANDLE hPowerSrvSM, BOOLEAN PS_disableEnable)
{
    PowerSrvSM_t *pPowerSrvSM = (PowerSrvSM_t*)hPowerSrvSM;
    whalCtrl_powerSaveParams_t powerSaveParams;
    TI_STATUS status;

    /*setting the params for the Hal*/
    powerSaveParams.hangOverPeriod          = pPowerSrvSM->hangOverPeriod;
    powerSaveParams.numNullPktRetries           = pPowerSrvSM->numNullPktRetries;
    powerSaveParams.NullPktRateModulation       = pPowerSrvSM->NullPktRateModulation;
    powerSaveParams.needToSendNullData      = pPowerSrvSM->pSmRequest->sendNullDataOnExit;
    powerSaveParams.ps802_11Enable          = PS_disableEnable;
    powerSaveParams.powerSavecmdResponseCB  = pPowerSrvSM->pSmRequest->powerSaveCmdResponseCB;
    if ( pPowerSrvSM->pSmRequest->powerSaveCmdResponseCB == NULL )
    {
        powerSaveParams.powerSaveCBObject       =  NULL; /*in order to not set to the Mailbox a response CB*/
    }
    else
    {
        powerSaveParams.powerSaveCBObject       = pPowerSrvSM->pSmRequest->powerSaveCBObject;       
    }

    /* start the FW guard timer, which is used to protect from FW stuck */
   os_timerStart( pPowerSrvSM->hOS, pPowerSrvSM->hTimer, POWER_SAVE_GUARD_TIME_MS, FALSE );

   /* that command should be sent to FW just in case we moved from Active to one of the PS modes
     * and vice versa, it shoul not be sent when moving between different PS modes */
    status = whalCtrl_powerMgmtConfig(pPowerSrvSM->hWhalCtrl,
                                      &powerSaveParams);
    if ( status != OK )
    {
        WLAN_REPORT_ERROR(pPowerSrvSM->hReport,
                          POWER_SERVER_MODULE_LOG,
                          ("%s(%d) - Error in configuring Power Manager paramters!\n",
                           __FILE__,__LINE__));
    }

    return status;
}

/****************************************************************************************
 *                        powerSrvRegisterFailureEventCB                                                    *
 ****************************************************************************************
DESCRIPTION: Registers a failure event callback for PS SM error notifications.
                
                                                                                                                   
INPUT:      - hPowerSrv         - handle to the PowerSrv object.        
            - failureEventCB    - the failure event callback function.\n
            - hFailureEventObj - handle to the object passed to the failure event callback function.

OUTPUT: 
RETURN:    void.
****************************************************************************************/
void powerSrvSM_RegisterFailureEventCB( TI_HANDLE hPowerSrvSM, 
                                        void *failureEventCB, TI_HANDLE hFailureEventObj )
{
    PowerSrvSM_t *pPowerSrvSM = (PowerSrvSM_t*)hPowerSrvSM;

    pPowerSrvSM->failureEventCB = (failureEventCB_t)failureEventCB;
    pPowerSrvSM->hFailureEventObj = hFailureEventObj;
}

