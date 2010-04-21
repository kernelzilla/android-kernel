/** \file nrfsm.c
 *  \brief non-recursive finite state machine source code
 *
 *  \see nrfsm.h
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

/***************************************************************************/
/*                                                                         */
/*      MODULE: fsm.c                                                      */
/*    PURPOSE:  Finite State Machine source code                           */
/*                                                                         */
/***************************************************************************/

#include "osTIType.h"
#include "osApi.h"
#include "utils.h"
#include "nrfsm.h"

/* Constants */

/* Enumerations */

/* Typedefs */

/* Structures */

/** General NR-FSM structure */
typedef struct
{
    TI_HANDLE               hOs;                    /**< OS handle */ 
    nrfsm_matrix_t          matrix;                 /**< State\event matrix */
    UINT32                  uMaxNoOfStates;         /**< Max number of states in the matrix */
    UINT32                  uMaxNoOfEvents;         /**< Max number of events in the matrix */
    UINT32                  uActNoOfStates;         /**< Actual number of states in the matrix */
    UINT32                  uActNoOfEvents;         /**< Actual number of events in the matrix */
    UINT32                  uInAction;              /**< Number of handled events */
    UINT32                  state;                  /**< Current state */
    UINT32                  event;                  /**< Last event sent */
    void                   *pData;                  /**< Last event data */
    BOOL                    bEventPending;          /**< Event pending indicator */

} nrfsm_t;


/* External data definitions */

/* External functions definitions */

/* Function prototypes */

/**
*
* nrfsm_Init  - Initialize the FSM structure
*
* \b Description: 
*
* Init The FSM structure. If matrix argument is NULL, allocate memory for
* new matrix.
*
* \b ARGS:
*
*  I   - hOs  - OS handler
*  O   - hFsm - the generated FSM module  \n
*  I   - uMaxNoOfStates - Number of states in the module \n
*  I   - uMaxNoOfEvents - Number of events in the module \n
*
* \b RETURNS:
*
*  OK on success, NOK on failure 
*
* \sa fsm_Event
*/
TI_STATUS nrfsm_Create (TI_HANDLE  hOs,
                        TI_HANDLE *hFsm,
                        UINT32     uMaxNoOfStates,
                        UINT32     uMaxNoOfEvents)
{
    nrfsm_t *pFsm;

    /* Check for preliminary conditions */
    if (hFsm == NULL || uMaxNoOfStates == 0 || uMaxNoOfEvents == 0)
    {
        return NOK;
    }

    /* Allocate memory for FSM context */
    pFsm = (nrfsm_t *)os_memoryAlloc (hOs, sizeof(nrfsm_t));
    if (pFsm == NULL)
    {
        return NOK;
    }

    /* Allocate memory for FSM matrix */
    pFsm->matrix = (nrfsm_matrix_t)os_memoryAlloc (hOs, uMaxNoOfStates * uMaxNoOfEvents * sizeof(nrfsm_action_cell_t));
    if (pFsm->matrix == NULL)
    {
        os_memoryFree (hOs, pFsm, sizeof(nrfsm_t));
        return NOK;
    }

    /* Update pFsm structure with parameters */
    pFsm->uMaxNoOfStates = uMaxNoOfStates;
    pFsm->uMaxNoOfEvents = uMaxNoOfEvents;
    pFsm->hOs = hOs;

    *hFsm = (TI_HANDLE)pFsm;

    return OK;
}


/**
*
* nrfsm_Unload  - free all memory allocated to FSM structure
*
* \b Description: 
*
* Unload the FSM structure.
*
* \b ARGS:
*
*  I   - hFsm - the generated FSM module handle \n
*
* \b RETURNS:
*
*  OK on success, NOK on failure 
*
* \sa fsm_Event
*/
TI_STATUS nrfsm_Unload (TI_HANDLE hFsm)
{
    nrfsm_t *pFsm = (nrfsm_t *)hFsm;

    /* Check for preliminary conditions */
    if (pFsm == NULL)
    {
        return NOK;
    }

    /* Free memory of FSM matrix */
    if (pFsm->matrix != NULL)
    {
        os_memoryFree (pFsm->hOs, 
                       pFsm->matrix,
                       pFsm->uMaxNoOfStates * pFsm->uMaxNoOfEvents * sizeof(nrfsm_action_cell_t));
    }

    /* Free memory for FSM context (no need to check for null) */
    os_memoryFree (pFsm->hOs, pFsm, sizeof(nrfsm_t));

    return OK;
}


/**
*
* fsm_Init  - Initialize the FSM structure
*
* \b Description: 
*
* Init The FSM structure. If matrix argument is NULL, allocate memory for
* new matrix.
*
* \b ARGS:
*
*  O   - hFsm - the generated FSM module handle \n
*  I   - uActNoOfStates - Actual number of states in the module \n
*  I   - uActNoOfEvents - Actual number of events in the module \n
*  I/O - pMatrix - the state event matrix pointer
*
* \b RETURNS:
*
*  OK on success, NOK on failure 
*
* \sa fsm_Event
*/
TI_STATUS nrfsm_Config (TI_HANDLE      hFsm,
                        nrfsm_matrix_t pMatrix,
                        UINT32         uActNoOfStates,
                        UINT32         uActNoOfEvents)
{
    nrfsm_t *pFsm = (nrfsm_t *)hFsm;   

    /* Check for preliminary conditions */
    if (pFsm == NULL || pMatrix == NULL)
    {
        return NOK;
    }

    if (uActNoOfStates > pFsm->uMaxNoOfStates || 
        uActNoOfEvents > pFsm->uMaxNoOfEvents)
    {
        return NOK;
    }

    /* Copy matrix to FSM context */
    os_memoryCopy (pFsm->hOs, 
                   pFsm->matrix, 
                   pMatrix,
                   uActNoOfStates * uActNoOfEvents * sizeof(nrfsm_action_cell_t));

    /* Update pFsm structure with parameters */
    pFsm->uActNoOfStates = uActNoOfStates;
    pFsm->uActNoOfEvents = uActNoOfEvents;
    pFsm->uInAction = 0;
    pFsm->state = 0;

    return OK;
}


/**
*
* nrfsm_Event  - perform event transition in the matrix
*
* \b Description: 
*
* Perform event transition in the matrix
*
* \b ARGS:
*
*  I   - hFsm - the generated FSM module handle handle \n
*  I   - event - event causing transition \n
*  I   - pData - data for activation function \n
*
* \b RETURNS:
*
*  OK on success, NOK on failure, 1 on pending in queue 
*
* \sa fsm_Init
*/
TI_STATUS nrfsm_Event (TI_HANDLE hFsm, UINT32 event, void *pData)
{
    nrfsm_t *pFsm = (nrfsm_t *)hFsm;   
    UINT32   uIndex;
    
    /* Check for FSM existance */
    if (pFsm == NULL)
    {
        return NOK;
    }

    /* Boundary check */
    if (pFsm->state >= pFsm->uActNoOfStates || event >= pFsm->uActNoOfEvents)
    {
        return NOK;
    }

    /* Store request action */
    pFsm->event = event;
    pFsm->pData = pData;
    pFsm->bEventPending = TRUE;
    /*pFsm->uInAction ++;*/
    
    /* If currently performing an action, return (requested event will be handled when current action is finished) */
    if (pFsm->uInAction > 0)
    {
        if (pFsm->uInAction > 1)
            return NOK;
        return (TI_STATUS)1;
    }
  
    /* Perform requested events (avoid recursion when an action sends another event to any SM) */
    while (pFsm->bEventPending/*pFsm->uInAction*/)
    {
        pFsm->uInAction ++;
        pFsm->bEventPending = FALSE;

        /* Calculate action cell index */
        uIndex = pFsm->state * pFsm->uActNoOfEvents + pFsm->event;

        /* Update current state */
        pFsm->state = pFsm->matrix[uIndex].nState;

        /* Activate transition function */
        (*pFsm->matrix[uIndex].fAction) (pFsm->pData);

        pFsm->uInAction --;
    }

    return OK;
}


/**
*
* nrfsm_SetState  - Set the initial state.
*
* \b Description: 
*
* Set the initial state.
*
* \b ARGS:
*
*  I   - hFsm - the generated FSM module handle \n
*  I   - state - current state of the SM \n
*
* \b RETURNS:
*
*  OK on success, NOK on failure 
*
* \sa 
*/
TI_STATUS nrfsm_SetState (TI_HANDLE hFsm, UINT32 state)                          
{
    nrfsm_t *pFsm = (nrfsm_t *)hFsm;   

    /* Boundary check */
    if (pFsm->state >= pFsm->uActNoOfStates)
    {
        return NOK;
    }
    else
    {
        pFsm->state = state; 
        return OK;
    }
}


/**
*
* nrfsm_GetNextState  - Return the current state.
*
* \b Description: 
*
* Return the current state.
*
* \b ARGS:
*
*  I   - hFsm - the generated FSM module handle \n
*  O   - state - current state of the SM \n
*
* \b RETURNS:
*
*  OK on success, NOK on failure 
*
* \sa 
*/
TI_STATUS nrfsm_GetState (TI_HANDLE hFsm, UINT32 *state)                       
{
    nrfsm_t *pFsm = (nrfsm_t *)hFsm;   

    *state = pFsm->state;

    return OK;
}


/**
*
* nrfsm_GetNextState  - Return the next state for a given current state and an event.
*
* \b Description: 
*
* Return the next state for a given current state and an event.
*
* \b ARGS:
*
*  I   - pFsm - the generated FSM module  \n
*  I   - event - event causing transition \n
*  O   - state - returned next state \n
*
* \b RETURNS:
*
*  OK on success, NOK on failure 
*
* \sa 
*/
TI_STATUS nrfsm_GetNextState (TI_HANDLE  hFsm, UINT32 event, UINT32 *state)
{
    nrfsm_t *pFsm = (nrfsm_t *)hFsm;   

    if (pFsm != NULL)
    {
        if (pFsm->state < pFsm->uActNoOfStates && event < pFsm->uActNoOfEvents)
        {
            *state = pFsm->matrix[pFsm->state * pFsm->uActNoOfEvents + event].nState;
            return OK;
        }
    }
    
    return NOK;
}


/*TI_STATUS action_nop(void *pData)
{
    return OK;
}*/

