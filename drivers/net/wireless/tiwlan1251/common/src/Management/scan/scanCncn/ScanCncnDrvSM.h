/** \file ScanCncnDrvSM.h
 *  \brief This file include definitions for the scan concentrator Driver SM module.
 *  \author Ronen Kalish
 *  \date 02-Jan-2005
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

#ifndef __SCANCNCNDRVSM_H__
#define __SCANCNCNDRVSM_H__

#include "osApi.h"
#include "ScanCncn.h"
#include "fsm.h"

/*
 ***********************************************************************
 *	Constant definitions.
 ***********************************************************************
 */

/*
 ***********************************************************************
 *	Enums.
 ***********************************************************************
 */

/** \enum scan_drvSMEvents_e
 * \brief enumerates the different driver scan SM events
 */
typedef enum
{
    DRV_SCAN_EVENT_START_SCAN = 0,
    DRV_SCAN_EVENT_SCR_PEND,
    DRV_SCAN_EVENT_SCR_REJECT,
    DRV_SCAN_EVENT_PASSIVE_SCAN,
    DRV_SCAN_EVENT_ACTIVE_SCAN,
    DRV_SCAN_EVENT_ABORT_SCAN,
    DRV_SCAN_EVENT_FW_RESET,
    DRV_SCAN_EVENT_STOP_SCAN,
    DRV_SCAN_EVENT_SCAN_COMPLETE,
    DRV_SCAN_NUM_OF_EVENTS
} scan_drvSMEvents_e;

/** \enum scan_drvSMEvents_e
 * \brief enumerates the different driver scan SM states
 */
typedef enum
{
    DRV_SCAN_STATE_IDLE = 0,
    DRV_SCAN_STATE_SCR_REQUEST,
    DRV_SCAN_STATE_SCR_WAIT,
    DRV_SCAN_STATE_PASSIVE_SCANNING,
    DRV_SCAN_STATE_ACTIVE_SCANNING,
    DRV_SCAN_STATE_STOPPING,
    DRV_SCAN_NUM_OF_STATES
} scan_drvSMStates_e;

/*
 ***********************************************************************
 *	Typedefs.
 ***********************************************************************
 */

/*
 ***********************************************************************
 *	Structure definitions.
 ***********************************************************************
 */

/*
 ***********************************************************************
 *	External data definitions.
 ***********************************************************************
 */

/*
 ***********************************************************************
 *	External functions definitions
 ***********************************************************************
 */

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Initialize the scan concentrator driver SM.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorDrvSM_init( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief Processes an event.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param currentState - the current driver SM state.\n
 * \param event - the event to handle.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorDrvSM_SMEvent( TI_HANDLE hScanCncn, scan_drvSMStates_e* currentState, 
                                         scan_drvSMEvents_e event );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief SM action - handles a start scan event (by requesting the SCR)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorDrvSM_requestSCR( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief SM action - handles a FW reset event (by calling the complete CB)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorDrvSM_callCompleteCB( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief SM action - handles a passive scan event (by starting a passive scan)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorDrvSM_passiveScan( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief SM action - handles an active scan event (by starting an active scan)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorDrvSM_activeScan( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief SM action - handles an abort scan or stop scan event (by stopping the actual scan)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorDrvSM_abortScan( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 10-July-2005\n
 * \brief SM action - handles a recovery event (calls the scan SRV abort on FW reset and than finishes scan)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorDrvSM_recoveryDuringScan( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief SM action - handles a scan complete event (by releasing the SCR and calling the scan complete CB)
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorDrvSM_scanComplete( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 02-Jan-2005\n
 * \brief SM action - handles a scan reject event (abort scan before scan acrually started)\n
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorDrvSM_scanRejected( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 09-Jan-2005\n
 * \brief Determines the next event to send to the driver SM (when a scan can be run)\n
 *
 * Function Scope \e Private.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return the next event to use with the driver SM.\n
 */
scan_drvSMEvents_e scanConcentrator_getNextDriverEvent( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 07-Feb-2005\n
 * \brief Handles an error during scan operation
 *
 * Function Scope \e Private.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 */
void scanConcentratorDrvSM_handleScanError( TI_HANDLE hScanCncn );

#endif /* __SCANCNCNDRVSM_H__ */

