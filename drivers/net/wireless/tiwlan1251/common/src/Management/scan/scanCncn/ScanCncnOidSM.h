/** \file ScanCncnOidSM.h
 *  \brief This file include definitions for the scan concentrator OID request SM module.
 *  \author Ronen Kalish
 *  \date 11-May-2006
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

#ifndef __SCANCNCNOIDSM_H__
#define __SCANCNCNOIDSM_H__

#include "osTIType.h"
#include "osApi.h"

#include "scanTypes.h"
#include "commonTypes.h"

/*
 ***********************************************************************
 *	Constant definitions.
 ***********************************************************************
 */

#define SCAN_OID_DEFAULT_PROBE_REQUEST_RATE_G						DRV_RATE_MASK_2_BARKER
#define SCAN_OID_DEFAULT_PROBE_REQUEST_RATE_A						DRV_RATE_6M
#define SCAN_OID_DEFAULT_PROBE_REQUEST_NUMBER_G						3
#define SCAN_OID_DEFAULT_PROBE_REQUEST_NUMBER_A						3
#define SCAN_OID_DEFAULT_MAX_DWELL_TIME_PASSIVE_G					100000
#define SCAN_OID_DEFAULT_MAX_DWELL_TIME_PASSIVE_A					100000
#define SCAN_OID_DEFAULT_MAX_DWELL_TIME_ACTIVE_G					25000
#define SCAN_OID_DEFAULT_MAX_DWELL_TIME_ACTIVE_A					25000
#define SCAN_OID_DEFAULT_MIN_DWELL_TIME_PASSIVE_G					100000
#define SCAN_OID_DEFAULT_MIN_DWELL_TIME_PASSIVE_A					100000
#define SCAN_OID_DEFAULT_MIN_DWELL_TIME_ACTIVE_G					5000
#define SCAN_OID_DEFAULT_MIN_DWELL_TIME_ACTIVE_A					5000
#define SCAN_OID_DEFAULT_EARLY_TERMINATION_EVENT_PASSIVE_G			SCAN_ET_COND_BEACON
#define SCAN_OID_DEFAULT_EARLY_TERMINATION_EVENT_PASSIVE_A			SCAN_ET_COND_BEACON
#define SCAN_OID_DEFAULT_EARLY_TERMINATION_EVENT_ACTIVE_G			SCAN_ET_COND_ANY_FRAME
#define SCAN_OID_DEFAULT_EARLY_TERMINATION_EVENT_ACTIVE_A			SCAN_ET_COND_ANY_FRAME
#define SCAN_OID_DEFAULT_EARLY_TERMINATION_COUNT_PASSIVE_G			2
#define SCAN_OID_DEFAULT_EARLY_TERMINATION_COUNT_PASSIVE_A			2
#define SCAN_OID_DEFAULT_EARLY_TERMINATION_COUNT_ACTIVE_G			3
#define SCAN_OID_DEFAULT_EARLY_TERMINATION_COUNT_ACTIVE_A			3

/*
 ***********************************************************************
 *	Enums.
 ***********************************************************************
 */

/** \enum scan_oidSMEvents_e
 * \brief enumerates the different scan OID request SM events
 */
typedef enum
{
    OID_SCAN_EVENT_START_SCAN = 0,
    OID_SCAN_EVENT_SCAN_COMPLETE,
    OID_SCAN_EVENT_SCAN_FAILED,
    OID_SCAN_NUM_OF_EVENTS
} scan_oidSMEvents_e;

/** \enum scan_oidSMStates_e
 * \brief enumerates the different scan OID request SM states
 */
typedef enum
{
    OID_SCAN_STATE_IDLE = 0,
    OID_SCAN_STATE_SCAN_ON_G,
	OID_SCAN_STATE_SCAN_ON_A,
    OID_SCAN_NUM_OF_STATES
} scan_oidSMStates_e;

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
 * \date 11-May-2006\n
 * \brief Initialize the scan concentrator OID request SM.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorOidSM_init( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 11-May-2006\n
 * \brief Processes an event.
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param currentState - the current OID request SM state.\n
 * \param event - the event to handle.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorOidSM_SMEvent( TI_HANDLE hScanCncn, scan_oidSMStates_e* currentState, 
                                         scan_oidSMEvents_e event );

/**
 * \author Ronen Kalish\n
 * \date 11-May-2006\n
 * \brief SM action - starts a scan on G band
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorOidSM_actionStartGScan( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 11-May-2006\n
 * \brief SM action - starts a scan on A band
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorOidSM_actionStartAScan( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 14-May-2006\n
 * \brief SM action - Cleans up an OID scan operation
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \return OK if successful, NOK otherwise.\n
 */
TI_STATUS scanConcentratorOidSM_actionCleanup( TI_HANDLE hScanCncn );

/**
 * \author Ronen Kalish\n
 * \date 14-May-2006\n
 * \brief Fills a chhanel array with valid channels (and their params) according to band and scan type\n
 *
 * Function Scope \e Public.\n
 * \param hScanCncn - handle to the scan concentrator object.\n
 * \param band - band to extract channels for.\n
 * \param scanType - scan type tp ectract channels for.\n
 * \param channelArray - where to store allowed channels information.\n
 * \param maxDwellTime - maximum dwell time value to be used for each channel.\n
 * \param minDwellTime - minimum dwell time value to be used for each channel.\n
 * \param ETCondition - early termination condition value to be used for each channel.\n
 * \param ETFrameNumber - early termination frame number value to be used for each channel.\n
 * \return Number of allowed channels (that were placed in the given channels array).\n
 */
UINT32 scanConcentratorOidSM_FillAllAvailableChannels( TI_HANDLE hScanCncn, radioBand_e band, scan_Type_e scanType,
													   scan_channelEntry_u* channelArray, UINT32 maxDwellTime,
													   UINT32 minChannelTime, scan_ETCondition_e ETCondition,
													   UINT8 ETFrameNumber );

#endif /* __SCANCNCNOIDSM_H__ */

