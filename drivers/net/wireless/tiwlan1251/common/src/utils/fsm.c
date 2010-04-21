/** \file fsm.c
 *  \brief finite state machine source code
 *
 *  \see fsm.h
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
/*																		   */
/*		MODULE:	fsm.c													   */
/*    PURPOSE:	Finite State Machine source code						   */
/*																	 	   */
/***************************************************************************/

#include "osTIType.h"
#include "osApi.h"
#include "utils.h"
#include "report.h"
#include "fsm.h"

/* Constants */

/* Enumerations */

/* Typedefs */

/* Structures */

/* External data definitions */

/* External functions definitions */

/* Function prototypes */

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
*  O   - pFsm - the generated FSM module  \n
*  I   - noOfStates - Number of states in the module \n
*  I   - noOfStates - Number of events in the module \n
*  I/O - matrix - the state event matrix
*  I   - transFunc - Transition finction for the state machine \n
*
* \b RETURNS:
*
*  OK on success, NOK on failure 
*
* \sa fsm_Event
*/
TI_STATUS fsm_Create(TI_HANDLE				hOs,
				fsm_stateMachine_t		**pFsm,
				UINT8					MaxNoOfStates,
				UINT8					MaxNoOfEvents)
{
	/* check for perliminary conditions */
	if ((pFsm == NULL) || (MaxNoOfStates == 0) || (MaxNoOfEvents == 0))
	{
		return NOK;
	}

	/* allocate memory for FSM context */
	*pFsm = (fsm_stateMachine_t *)os_memoryAlloc(hOs, sizeof(fsm_stateMachine_t));
	if (*pFsm == NULL)
	{
		return NOK;
	}
	os_memoryZero(hOs, (*pFsm), sizeof(fsm_stateMachine_t)); /* Dm: Fix */

	/* allocate memory for FSM matrix */
	(*pFsm)->stateEventMatrix = (fsm_Matrix_t)os_memoryAlloc(hOs, MaxNoOfStates * MaxNoOfEvents * sizeof(fsm_actionCell_t));
	if ((*pFsm)->stateEventMatrix == NULL)
	{
		os_memoryFree(hOs, *pFsm, sizeof(fsm_stateMachine_t));
		return NOK;
	}
	os_memoryZero(hOs, (*pFsm)->stateEventMatrix, 
		(MaxNoOfStates * MaxNoOfEvents * sizeof(fsm_actionCell_t)));
	/* update pFsm structure with parameters */
	(*pFsm)->MaxNoOfStates = MaxNoOfStates;
	(*pFsm)->MaxNoOfEvents = MaxNoOfEvents;

	return(OK);
}

/**
*
* fsm_Unload  - free all memory allocated to FSM structure
*
* \b Description: 
*
* Unload the FSM structure.
*
* \b ARGS:
*
*  O   - pFsm - the generated FSM module  \n
*  I   - noOfStates - Number of states in the module \n
*  I   - noOfStates - Number of events in the module \n
*  I/O - matrix - the state event matrix
*  I   - transFunc - Transition finction for the state machine \n
*
* \b RETURNS:
*
*  OK on success, NOK on failure 
*
* \sa fsm_Event
*/
TI_STATUS fsm_Unload(TI_HANDLE				hOs,
				fsm_stateMachine_t		*pFsm)
{
	/* check for perliminary conditions */
	if (pFsm == NULL)
	{
		return NOK;
	}

	/* free memory of FSM matrix */
	if (pFsm->stateEventMatrix != NULL)
	{
		os_memoryFree(hOs, pFsm->stateEventMatrix,
					  pFsm->MaxNoOfStates * pFsm->MaxNoOfEvents * sizeof(fsm_actionCell_t));
	}

	/* free memory for FSM context (no need to check for null) */
	os_memoryFree(hOs, pFsm, sizeof(fsm_stateMachine_t));

	return(OK);
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
*  O   - pFsm - the generated FSM module  \n
*  I   - noOfStates - Number of states in the module \n
*  I   - noOfStates - Number of events in the module \n
*  I/O - matrix - the state event matrix
*  I   - transFunc - Transition finction for the state machine \n
*
* \b RETURNS:
*
*  OK on success, NOK on failure 
*
* \sa fsm_Event
*/
TI_STATUS fsm_Config(fsm_stateMachine_t	*pFsm,
				  fsm_Matrix_t			pMatrix,
				  UINT8					ActiveNoOfStates,
				  UINT8					ActiveNoOfEvents,
				  fsm_eventActivation_t	transFunc,
				  TI_HANDLE				hOs)
{
	/* check for perliminary conditions */
	if ((pFsm == NULL) ||
		(pMatrix == NULL))
	{
		return NOK;
	}

	if ((ActiveNoOfStates > pFsm->MaxNoOfStates) || 
		(ActiveNoOfEvents > pFsm->MaxNoOfEvents))
	{
		return NOK;
	}

	/* copy matrix to FSM context */
	os_memoryCopy(hOs, (void *)pFsm->stateEventMatrix, (void *)pMatrix,
				  ActiveNoOfStates * ActiveNoOfEvents * sizeof(fsm_actionCell_t));

	/* update pFsm structure with parameters */
	pFsm->ActiveNoOfStates = ActiveNoOfStates;
	pFsm->ActiveNoOfEvents = ActiveNoOfEvents;
	pFsm->transitionFunc = transFunc;
	return(OK);
}

/**
*
* fsm_Event  - perform event transition in the matrix
*
* \b Description: 
*
* Perform event transition in the matrix
*
* \b ARGS:
*
*  I   - pFsm - the generated FSM module  \n
*  I/O - currentState - current state of the SM \n
*  I   - event - event causing transition \n
*  I   - pData - data for activation function \n
*
* \b RETURNS:
*
*  OK on success, NOK on failure 
*
* \sa fsm_Init
*/
TI_STATUS fsm_Event(fsm_stateMachine_t		*pFsm,
				 UINT8					*currentState,
				 UINT8					event,
				 void					*pData)
{
	UINT8		oldState;
	TI_STATUS		status;

	/* check for FSM existance */
	if (pFsm == NULL)
	{
		return(NOK);
	}

	/* boundary check */
	if ((*currentState >= pFsm->ActiveNoOfStates) || (event >= pFsm->ActiveNoOfEvents))
	{
		return(NOK);
	}
	
	oldState = *currentState;
	/* update current state */
	*currentState = pFsm->stateEventMatrix[(*currentState * pFsm->ActiveNoOfEvents) + event].nextState;

	/* activate transition function */
	if( !(*pFsm->stateEventMatrix[(oldState * pFsm->ActiveNoOfEvents) + event].actionFunc) ) {
	    return(NOK);
	}
	status = (*pFsm->stateEventMatrix[(oldState * pFsm->ActiveNoOfEvents) + event].actionFunc)(pData);

	return status;
}


/**
*
* fsm_GetNextState  - Retrun the next state for a given current state and an event.
*
* \b Description: 
*
* Retrun the next state for a given current state and an event.
*
* \b ARGS:
*
*  I   - pFsm - the generated FSM module  \n
*  I   - currentState - current state of the SM \n
*  I   - event - event causing transition \n
*  O   - nextState - returned next state \n
*
* \b RETURNS:
*
*  OK on success, NOK on failure 
*
* \sa 
*/
TI_STATUS fsm_GetNextState(fsm_stateMachine_t		*pFsm,
						UINT8					currentState,
						UINT8					event,
						UINT8					*nextState)
{
	if (pFsm != NULL)
	{
		if ((currentState < pFsm->ActiveNoOfStates) && (event < pFsm->ActiveNoOfEvents))
		{
			*nextState = pFsm->stateEventMatrix[(currentState * pFsm->ActiveNoOfEvents) + event].nextState;
			return(OK);
		}
	}
	
	return(NOK);
}

TI_STATUS action_nop(void *pData)
{
	return OK;
}
