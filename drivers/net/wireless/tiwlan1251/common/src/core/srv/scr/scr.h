/** \file Scr.h
 *  \brief This file includes internal (private) definitions to the SCR module
 *  \author Ronen Kalish
 *  \date 01-Dec-2004
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

#ifndef __SCR_H__
#define __SCR_H__

#include "scrApi.h"

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

/** \enum scr_clientState_e 
 * \brief enumerates the different states a client may be in .\n
 */
typedef enum
{
    SCR_CS_IDLE = 0,    /**< client is idle */
    SCR_CS_PENDING,     /**< client is pending to use the channel */
    SCR_CS_RUNNING,     /**< client is using the channel */
    SCR_CS_ABORTING     /**< 
                         * client was using the channel, but was aborted, 
                         * and complete notification is expected.
                         */
} scr_clientState_e;


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

/** \struct scr_client_t
 * \brief This structure contains information for a specific client
 */
typedef struct
{
	scr_clientState_e 	state;                                      /**< the client current state */
	scr_callback_t		clientRequestCB;                            /**< the client's callback function */
	TI_HANDLE		    ClientRequestCBObj;                         /**< the client's object */
    scr_pendReason_e    currentPendingReason;                       /**< 
                                                                     * the reason why this client is pending
                                                                     * (if at all)
                                                                     */
} scr_client_t;

/** \struct scr_t
 * \brief This structure contains the SCR object data
 */
typedef struct
{
    TI_HANDLE		        hOS;                                    /**< a handle to the OS object */
    TI_HANDLE               hReport;                                /**< a handle to the report object */
	BOOLEAN		            statusNotficationPending;               /**< 
                                                                     * whether the SCR is in the process of  
                                                                     * notifying a status change to a client
                                                                     * (used to solve re-entrance problem)
                                                                     */
    scr_clientId_e          runningClient;                          /**< 
                                                                     * The index of the current running client 
                                                                     * (-1 if none)
                                                                     */
    scr_groupId_e           currentGroup;                           /**< the current group */
	scr_modeId_e			currentMode;							/**< the current mode */
    scr_client_t	        clientArray[ SCR_CID_NUM_OF_CLIENTS ];  /**< array holding all clients' info */
} scr_t;


/*
 ***********************************************************************
 *	External functions definitions
 ***********************************************************************
 */
/**
 * \author Ronen Kalish\n
 * \date 01-Dec-2004\n
 * \brief Searches the client database for a client with matching state, from startFrom to endAt\n
 *
 * Function Scope \e Private.\n
 * \param hScr - handle to the SCR object.\n
 * \param requiredState - the state to match.\n
 * \param startFrom - the highest priority to begin searching from.\n
 * \param endAt - the lowest priority to include in the search.\n
 * \return the client ID if found, SCR_CID_NO_CLIENT otherwise.\n
 */
INT8 scrFindHighest( TI_HANDLE hScr,
                     scr_clientState_e requiredState,
                     int startFrom,
                     int endAt );

#endif /* __SCR_H__ */
