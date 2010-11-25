/** \file fsm.h
 *  \brief non-recursive finite state machine header file
 *
 *  \see fsm.c
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
/*      MODULE: fsm.h                                                      */
/*    PURPOSE:  Finite State Machine API                                   */
/*                                                                         */
/***************************************************************************/

#ifndef __NRFSM_H__
#define __NRFSM_H__


/* Constants */

/* Enumerations */

/* Typedefs */

/** action function type definition */
typedef TI_STATUS (*nrfsm_action_t) (void* pData);

/* Structures */

/* State\Event cell */
typedef  struct
{
    UINT32          nState;      /**< next state in transition */
    nrfsm_action_t  fAction;     /**< action function */
} nrfsm_action_cell_t;

/** matrix type */
typedef nrfsm_action_cell_t*  nrfsm_matrix_t;


/* External data definitions */

/* External functions definitions */

/* Function prototypes */

TI_STATUS nrfsm_Create       (TI_HANDLE       hOs,
                              TI_HANDLE      *hFsm,
                              UINT32          uMaxNoOfStates,
                              UINT32          uMaxNoOfEvents);

TI_STATUS nrfsm_Unload       (TI_HANDLE       hFsm);

TI_STATUS nrfsm_Config       (TI_HANDLE       hFsm,
                              nrfsm_matrix_t  pMatrix,
                              UINT32          uActNoOfStates,
                              UINT32          uActNoOfEvents);

TI_STATUS nrfsm_Event        (TI_HANDLE       hFsm,
                              UINT32          event,
                              void           *pData);

TI_STATUS nrfsm_GetState     (TI_HANDLE       hFsm,
                              UINT32         *state);

TI_STATUS nrfsm_SetState     (TI_HANDLE       hFsm,
                              UINT32          state);

TI_STATUS nrfsm_GetNextState (TI_HANDLE       hFsm,
                              UINT32          event,
                              UINT32         *state);

#endif /* __NRFSM_H__ */
