/** \file Scr.c
 *  \brief This file include the SCR module implementation
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

#include "report.h"
#include "scr.h"
#include "osApi.h"


/*
 ***********************************************************************
 *	External data definitions.
 ***********************************************************************
 */

/**
 * \brief This array holds configuration values for abort others field for different clients.\n
 */
static scr_clientId_e abortOthers[ SCR_CID_NUM_OF_CLIENTS ] = 
/* APP_SCAN           DRV_FG             CONT_SCAN          EXC_MSR            BASIC_MSR          CONNECT                IMMED_SCN              SWITCH_CHNL*/
 { SCR_CID_NO_CLIENT, SCR_CID_NO_CLIENT, SCR_CID_NO_CLIENT, SCR_CID_NO_CLIENT, SCR_CID_NO_CLIENT, SCR_CID_BASIC_MEASURE, SCR_CID_BASIC_MEASURE, SCR_CID_BASIC_MEASURE };

/**
 * \brief This array holds configuration values for the client status field for different clients and groups. \n
 */
static BOOLEAN clientStaus [SCR_MID_NUM_OF_MODES][ SCR_GID_NUM_OF_GROUPS ][ SCR_CID_NUM_OF_CLIENTS ] = 
				{	/* This is the table for Normal mode	*/
					{/* client status for idle group */
                        { FALSE,    /**< client status for SCR_CID_APP_SCAN */
                          FALSE,    /**< client status for SCR_CID_DRIVER_FG_SCAN */
                          FALSE,    /**< client status for SCR_CID_CONT_SCAN */
                          FALSE,    /**< client status for SCR_CID_EXC_MEASURE */
                          FALSE,    /**< client status for SCR_CID_BASIC_MEASURE */
                          FALSE,    /**< client status for SCR_CID_CONNECT */
                          FALSE,    /**< client status for SCR_CID_IMMED_SCAN */
                          FALSE,    /**< client status for SCR_CID_SWITCH_CHANNEL */ },
                       /* client status for inter scan gorup */
                        { TRUE,     /**< client status for SCR_CID_APP_SCAN */
                          FALSE,    /**< client status for SCR_CID_DRIVER_FG_SCAN */
                          FALSE,    /**< client status for SCR_CID_CONT_SCAN */
                          FALSE,    /**< client status for SCR_CID_EXC_MEASURE */
                          FALSE,    /**< client status for SCR_CID_BASIC_MEASURE */
                          FALSE,    /**< client status for SCR_CID_CONNECT */
                          FALSE,    /**< client status for SCR_CID_IMMED_SCAN */
                          FALSE,    /**< client status for SCR_CID_SWITCH_CHANNEL */ },
                       /* client status for connect group */
                        { FALSE,    /**< client status for SCR_CID_APP_SCAN */
                          TRUE,     /**< client status for SCR_CID_DRIVER_FG_SCAN */
                          FALSE,    /**< client status for SCR_CID_CONT_SCAN */
                          FALSE,    /**< client status for SCR_CID_EXC_MEASURE */
                          FALSE,    /**< client status for SCR_CID_BASIC_MEASURE */
                          TRUE,     /**< client status for SCR_CID_CONNECT */
                          FALSE,    /**< client status for SCR_CID_IMMED_SCAN */
                          FALSE,    /**< client status for SCR_CID_SWITCH_CHANNEL */ },                       
                       /* client status for connected group */
                        { TRUE,     /**< client status for SCR_CID_APP_SCAN */
                          FALSE,    /**< client status for SCR_CID_DRIVER_FG_SCAN */
                          TRUE,     /**< client status for SCR_CID_CONT_SCAN */
                          TRUE,     /**< client status for SCR_CID_EXC_MEASURE */
                          TRUE,     /**< client status for SCR_CID_BASIC_MEASURE */
                          FALSE,    /**< client status for SCR_CID_CONNECT */
                          FALSE,    /**< client status for SCR_CID_IMMED_SCAN */
                          TRUE,     /**< client status for SCR_CID_SWITCH_CHANNEL */ },
                       /* client status for roaming group */
                        { FALSE,    /**< client status for SCR_CID_APP_SCAN */
                          FALSE,    /**< client status for SCR_CID_DRIVER_FG_SCAN */
                          FALSE,    /**< client status for SCR_CID_CONT_SCAN */
                          FALSE,    /**< client status for SCR_CID_EXC_MEASURE */
                          FALSE,    /**< client status for SCR_CID_BASIC_MEASURE */
                          TRUE,     /**< client status for SCR_CID_CONNECT */
                          TRUE,     /**< client status for SCR_CID_IMMED_SCAN */
                          FALSE,    /**< client status for SCR_CID_SWITCH_CHANNEL */ }
                    },

					/* This is the table for the Soft gemini mode	*/

					{ /* client status for idle group */
                        { FALSE,    /**< client status for SCR_CID_APP_SCAN */
                          FALSE,    /**< client status for SCR_CID_DRIVER_FG_SCAN */
                          FALSE,    /**< client status for SCR_CID_CONT_SCAN */
                          FALSE,    /**< client status for SCR_CID_EXC_MEASURE */
                          FALSE,    /**< client status for SCR_CID_BASIC_MEASURE */
                          FALSE,    /**< client status for SCR_CID_CONNECT */
                          FALSE,    /**< client status for SCR_CID_IMMED_SCAN */
                          FALSE,    /**< client status for SCR_CID_SWITCH_CHANNEL */ },
                       /* client status for inter scan gorup */
                        { TRUE,     /**< client status for SCR_CID_APP_SCAN */
                          FALSE,    /**< client status for SCR_CID_DRIVER_FG_SCAN */
                          FALSE,    /**< client status for SCR_CID_CONT_SCAN */
                          FALSE,    /**< client status for SCR_CID_EXC_MEASURE */
                          FALSE,    /**< client status for SCR_CID_BASIC_MEASURE */
                          FALSE,    /**< client status for SCR_CID_CONNECT */
                          FALSE,    /**< client status for SCR_CID_IMMED_SCAN */
                          FALSE,    /**< client status for SCR_CID_SWITCH_CHANNEL */ },
                       /* client status for connect group */
                        { FALSE,    /**< client status for SCR_CID_APP_SCAN */
                          TRUE,     /**< client status for SCR_CID_DRIVER_FG_SCAN */
                          FALSE,    /**< client status for SCR_CID_CONT_SCAN */
                          FALSE,    /**< client status for SCR_CID_EXC_MEASURE */
                          FALSE,    /**< client status for SCR_CID_BASIC_MEASURE */
                          TRUE,     /**< client status for SCR_CID_CONNECT */
                          FALSE,    /**< client status for SCR_CID_IMMED_SCAN */
                          FALSE,    /**< client status for SCR_CID_SWITCH_CHANNEL */ },                       
                       /* client status for connected group */
                        { TRUE,     /**< client status for SCR_CID_APP_SCAN */
                          FALSE,    /**< client status for SCR_CID_DRIVER_FG_SCAN */
                          TRUE,     /**< client status for SCR_CID_CONT_SCAN */
                          TRUE,     /**< client status for SCR_CID_EXC_MEASURE */
                          TRUE,     /**< client status for SCR_CID_BASIC_MEASURE */
                          FALSE,    /**< client status for SCR_CID_CONNECT */
                          FALSE,    /**< client status for SCR_CID_IMMED_SCAN */
                          TRUE,     /**< client status for SCR_CID_SWITCH_CHANNEL */ },
                       /* client status for roaming group */
                        { FALSE,    /**< client status for SCR_CID_APP_SCAN */
                          FALSE,    /**< client status for SCR_CID_DRIVER_FG_SCAN */
                          FALSE,    /**< client status for SCR_CID_CONT_SCAN */
                          FALSE,    /**< client status for SCR_CID_EXC_MEASURE */
                          FALSE,    /**< client status for SCR_CID_BASIC_MEASURE */
                          TRUE,     /**< client status for SCR_CID_CONNECT */
                          TRUE,     /**< client status for SCR_CID_IMMED_SCAN */
                          FALSE,    /**< client status for SCR_CID_SWITCH_CHANNEL */ }
                    }
				};

				
/**
 * \author Ronen Kalish\n
 * \date 01-Dec-2004\n
 * \brief Creates the SCR object
 *
 * Function Scope \e Public.\n
 * \param hOS - handle to the OS object.\n
 * \return a handle to the SCR object.\n
 */
TI_HANDLE scr_create( TI_HANDLE hOS )
{
    /* allocate the SCR object */
    scr_t *pScr = os_memoryAlloc( hOS, sizeof(scr_t) );

    if ( NULL == pScr )
    {
        WLAN_OS_REPORT( ("ERROR: Failed to create SCR module") );
        return NULL;
    }

    /* store the OS handle */
    pScr->hOS = hOS;

    return pScr;
}

/**
 * \author Ronen Kalish\n
 * \date 01-Dec-2004\n
 * \brief Finalizes the SCR object (freeing memory)
 *
 * Function Scope \e Public.\n
 * \param hScr - handle to the SCR object.\n
 */
void scr_release( TI_HANDLE hScr )
{
    scr_t *pScr = (scr_t*)hScr;

    os_memoryFree( pScr->hOS, hScr, sizeof(scr_t) );
}

/**
 * \author Ronen Kalish\n
 * \date 01-Dec-2004\n
 * \brief Initializes the SCR object
 *
 * Function Scope \e Public.\n
 * \param hScr - handle to the SCR object.\n
 * \param hReport - handle to the report module.\n
 */
void scr_init( TI_HANDLE hScr, TI_HANDLE hReport )
{
    int i;
    scr_t *pScr = (scr_t*)hScr;

    /* store the report object */
    pScr->hReport = hReport;

    /* mark current group as idle */
    pScr->currentGroup = SCR_GID_IDLE;

	/* mark current mode as normal */
	pScr->currentMode = SCR_MID_NORMAL;

    /* signal not within request process */
    pScr->statusNotficationPending = FALSE;

    /* mark that no client is currently running */
    pScr->runningClient = SCR_CID_NO_CLIENT;

    /* initialize client array */
    for ( i = 0; i < SCR_CID_NUM_OF_CLIENTS; i++ )
    {
        pScr->clientArray[ i ].state = SCR_CS_IDLE;
        pScr->clientArray[ i ].currentPendingReason = SCR_PR_NONE;
        pScr->clientArray[ i ].clientRequestCB = NULL;
        pScr->clientArray[ i ].ClientRequestCBObj = NULL;
    }
    
    WLAN_REPORT_INIT( hReport, SCR_MODULE_LOG,  (".....SCR configured successfully\n"));
}

/**
 * \author Ronen Kalish\n
 * \date 01-Dec-2004\n
 * \brief Registers the callback function to be used per client.
 *
 * Function Scope \e Public.\n
 * \param hScr - handle to the SCR object.\n
 * \param client - the client ID.\n
 * \param callbackFunc - the address of the callback function to use.\n
 * \param callbackObj - the handle of the object to pass to the callback function (the client object).\n
 */
void scr_registerClientCB( TI_HANDLE hScr, 
                           scr_clientId_e client,
                           scr_callback_t callbackFunc, 
                           TI_HANDLE callbackObj )
{
    scr_t *pScr = (scr_t*)hScr;

#ifdef TI_DBG
    if (client >= SCR_CID_NUM_OF_CLIENTS)
    {
        WLAN_REPORT_ERROR( pScr->hReport, SCR_MODULE_LOG,
                           ("Attempting to register callback for invalid client %d\n", client) );
        return;
    }
#endif
    pScr->clientArray[ client ].clientRequestCB = callbackFunc;
    pScr->clientArray[ client ].ClientRequestCBObj = callbackObj;
}

/**
 * \author Ronen Kalish\n
 * \date 01-Dec-2004\n
 * \brief Notifies the running process upon a firmware reset.
 *
 * Function Scope \e Public.\n
 * \param hScr - handle to the SCR object.\n
 */
void scr_notifyFWReset( TI_HANDLE hScr )
{
    scr_t *pScr = (scr_t*)hScr;

    /* if a client is currently running, notify it of the recovery event */
    if ( SCR_CID_NO_CLIENT != pScr->runningClient )
    {
        WLAN_REPORT_INFORMATION( pScr->hReport, 
                                 SCR_MODULE_LOG, 
                                 ("FW reset occured. Client %d Notified.\n", pScr->runningClient) );
        if ( NULL != pScr->clientArray[ pScr->runningClient ].clientRequestCB )
        {
            pScr->clientArray[ pScr->runningClient ].clientRequestCB( pScr->clientArray[ pScr->runningClient ].ClientRequestCBObj,
                                                                      SCR_CRS_FW_RESET, SCR_PR_NONE );
        }
        else
        {
            WLAN_REPORT_ERROR( pScr->hReport, SCR_MODULE_LOG,
                               ("Trying to call client %d callback, which is NULL\n", pScr->runningClient) );
        }
    }
#ifdef TI_DBG
    else
    {
        WLAN_REPORT_INFORMATION( pScr->hReport, 
                                 SCR_MODULE_LOG, 
                                 ("FW reset occured. No client was running.\n") );
    }
#endif
}

/**
 * \author Ronen Kalish\n
 * \date 27-April-2005\n
 * \brief Changes the current SCR group.\n
 *
 * Function Scope \e Public.\n
 * \param hScr - handle to the SCR object.\n
 * \param newGroup - the new group to use.\n
 */
void scr_setGroup( TI_HANDLE hScr, scr_groupId_e newGroup )
{
    scr_t *pScr = (scr_t*)hScr;
    int i,highestPending;

    WLAN_REPORT_INFORMATION( pScr->hReport, SCR_MODULE_LOG, ("Setting group %d.\n", newGroup) );

#ifdef TI_DBG
    if (newGroup >= SCR_GID_NUM_OF_GROUPS)
    {
        WLAN_REPORT_ERROR( pScr->hReport, SCR_MODULE_LOG,
                           ("Attempting to set invalid group to %d\n", newGroup) );
        return;
    }
#endif

    /* keep the new group */
    pScr->currentGroup = newGroup;

    /* for all pending clients */
    for ( i = 0; i < SCR_CID_NUM_OF_CLIENTS; i++ )
    {
        /* if the pending reason has escalated */
        if ( (pScr->clientArray[ i ].state == SCR_CS_PENDING) && /* the client is pending */
             (pScr->clientArray[ i ].currentPendingReason < SCR_PR_DIFFERENT_GROUP_RUNNING) && /* the client was enabled in the previous group */
             (FALSE == clientStaus[pScr->currentMode][ newGroup ][ i ]) ) /* the client is not enabled in the new group */
        {
            /* mark the new pending reason */
            pScr->clientArray[ i ].currentPendingReason = SCR_PR_DIFFERENT_GROUP_RUNNING;
            
            /* notify the client of the change, using its callback */
            if ( NULL != pScr->clientArray[ i ].clientRequestCB )
            {
                pScr->clientArray[ i ].clientRequestCB( pScr->clientArray[ i ].ClientRequestCBObj, 
                                                        SCR_CRS_PEND, SCR_PR_DIFFERENT_GROUP_RUNNING );
            }
            else
            {
                WLAN_REPORT_ERROR( pScr->hReport, SCR_MODULE_LOG,
                                   ("Trying to call client %d callback, which is NULL\n", i) );
            }
        }
    }

	/* if no client is running call the client with the highest pending client  */
	/* because now , after group change , clients can be enabled				*/
	if ( SCR_CID_NO_CLIENT == pScr->runningClient )
    {
		highestPending = scrFindHighest( hScr, SCR_CS_PENDING, (UINT8)(SCR_CID_NUM_OF_CLIENTS - 1), 0 );
		if (SCR_CID_NO_CLIENT != highestPending)
		{
			pScr->clientArray[ highestPending ].state = SCR_CS_RUNNING;
            pScr->clientArray[ highestPending ].currentPendingReason = SCR_PR_NONE;
            pScr->runningClient = (scr_clientId_e)highestPending;
			if ( NULL != pScr->clientArray[ highestPending ].clientRequestCB )
            {
                pScr->clientArray[ highestPending ].clientRequestCB( pScr->clientArray[ highestPending ].ClientRequestCBObj, 
                                                        SCR_CRS_RUN, SCR_PR_NONE );
            }
            else
            {
                WLAN_REPORT_ERROR( pScr->hReport, SCR_MODULE_LOG,
                                   ("Trying to call client %d callback, which is NULL\n", highestPending) );
            }
        }
	}

}

/**
 * \author Yuval Adler\n
 * \date 23-Nov-2005\n
 * \brief Changes the current SCR mode.\n
 *
 * Function Scope \e Public.\n
 * \param hScr - handle to the SCR object.\n
 * \param newMode - the new mode to use.\n
 */
void scr_setMode( TI_HANDLE hScr, scr_modeId_e newMode )
{
    scr_t *pScr = (scr_t*)hScr;
    int i,highestPending;

    WLAN_REPORT_INFORMATION( pScr->hReport, SCR_MODULE_LOG, ("Setting mode %d.\n", newMode) );

#ifdef TI_DBG
    if (newMode >= SCR_MID_NUM_OF_MODES)
    {
        WLAN_REPORT_ERROR( pScr->hReport, SCR_MODULE_LOG,
                           ("Attempting to set invalid mode to %d\n", newMode) );
        return;
    }
#endif

    /* keep the new mode */
    pScr->currentMode= newMode;
	
	/* Stage I : if someone is running and shouldn't be running in the new mode - Abort it */
	if ( SCR_CID_NO_CLIENT != pScr->runningClient && 
		(FALSE == clientStaus[ pScr->currentMode ][ pScr->currentGroup ][ pScr->runningClient ]) )
	{
		/* abort the running client */
		pScr->clientArray[ pScr->runningClient ].state = SCR_CS_ABORTING;
		if ( NULL != pScr->clientArray[ pScr->runningClient ].clientRequestCB )
		{
			WLAN_REPORT_INFORMATION( pScr->hReport, SCR_MODULE_LOG,
	   								 ("Sending abort request to client %d\n", pScr->runningClient) );
			pScr->clientArray[ pScr->runningClient ].clientRequestCB( pScr->clientArray[ pScr->runningClient ].ClientRequestCBObj,
																	  SCR_CRS_ABORT,
																	  SCR_PR_NONE );
		}
		else
		{
			WLAN_REPORT_ERROR( pScr->hReport, SCR_MODULE_LOG,
							   ("Trying to call client %d callback, which is NULL\n", pScr->runningClient) );
		}

	}

	/* Stage II : notify escalated pending reason	*/
	/* for all pending clients */
    for ( i = 0; i < SCR_CID_NUM_OF_CLIENTS; i++ )
    {
        /* if the pending reason has escalated */
        if ( (pScr->clientArray[ i ].state == SCR_CS_PENDING) && /* the client is pending */
             (pScr->clientArray[ i ].currentPendingReason < SCR_PR_DIFFERENT_GROUP_RUNNING) && /* the client was enabled in the previous group */
             (FALSE == clientStaus[ pScr->currentMode ][ pScr->currentGroup ][ i ]) ) /* the client is not enabled in the new group/mode */
        {
            /* mark the new pending reason */
            pScr->clientArray[ i ].currentPendingReason = SCR_PR_DIFFERENT_GROUP_RUNNING;
            
            /* notify the client of the change, using its callback */
            if ( NULL != pScr->clientArray[ i ].clientRequestCB )
            {
                pScr->clientArray[ i ].clientRequestCB( pScr->clientArray[ i ].ClientRequestCBObj, 
                                                        SCR_CRS_PEND, SCR_PR_DIFFERENT_GROUP_RUNNING );
            }
            else
            {
                WLAN_REPORT_ERROR( pScr->hReport, SCR_MODULE_LOG,
                                   ("Trying to call client %d callback, which is NULL\n", i) );
            }
        }
    }

   
	/* Stage III : call Highest Pending Client who is enabled in the new mode	*/
	if ( SCR_CID_NO_CLIENT == pScr->runningClient )
    {
		highestPending = scrFindHighest( hScr, SCR_CS_PENDING, (UINT8)(SCR_CID_NUM_OF_CLIENTS - 1), 0 );
		if (SCR_CID_NO_CLIENT != highestPending)
		{
			pScr->clientArray[ highestPending ].state = SCR_CS_RUNNING;
            pScr->clientArray[ highestPending ].currentPendingReason = SCR_PR_NONE;
            pScr->runningClient = (scr_clientId_e)highestPending;
			if ( NULL != pScr->clientArray[ highestPending ].clientRequestCB )
            {
                pScr->clientArray[ highestPending ].clientRequestCB( pScr->clientArray[ highestPending ].ClientRequestCBObj, 
                                                        SCR_CRS_RUN, SCR_PR_NONE );
            }
            else
            {
                WLAN_REPORT_ERROR( pScr->hReport, SCR_MODULE_LOG,
                                   ("Trying to call client %d callback, which is NULL\n", highestPending) );
            }
        }
	}

}


/**
 * \author Ronen Kalish\n
 * \date 01-Dec-2004\n
 * \brief Request the channel use by a client
 *
 * Function Scope \e Public.\n
 * \param hScr - handle to the SCR object.\n
 * \param client - the client ID requesting the channel.\n
 * \param pPendReason - the reason for a pend reply.\n
 * \return The request status.\n
 * \retval SCR_CRS_REJECT the channel cannot be allocated to this client.
 * \retval SCR_CRS_PEND the channel is currently busy, and this client had been placed on the waiting list.
 * \retval SCR_CRS_RUN the channel is allocated to this client.
 */
scr_clientRequestStatus_e scr_clientRequest( TI_HANDLE hScr, scr_clientId_e client, scr_pendReason_e* pPendReason )
{
    scr_t *pScr = (scr_t*)hScr;

    WLAN_REPORT_INFORMATION( pScr->hReport, SCR_MODULE_LOG, ("Client %d requesting the channel.\n", client) );

#ifdef TI_DBG
    if (client >= SCR_CID_NUM_OF_CLIENTS)
    {
        WLAN_REPORT_ERROR( pScr->hReport, SCR_MODULE_LOG,
                           ("Attempting to request SCR for invalid client %d\n", client) );
        return SCR_CRS_PEND;
    }
#endif
    
    *pPendReason = SCR_PR_NONE;

    /* check if already inside a request - shouldn't happen!!! */
    if ( TRUE == pScr->statusNotficationPending )
    {
        WLAN_REPORT_ERROR( pScr->hReport, SCR_MODULE_LOG, ("request call while already in request!\n") );
        return SCR_CRS_PEND;
    }

    /* check if current running client is requesting */
    if ( client == pScr->runningClient )
    {
        WLAN_REPORT_WARNING( pScr->hReport, SCR_MODULE_LOG, ("Client %d re-requesting SCR\n", client) );
        return SCR_CRS_RUN;
    }
    
    /* check if the client is enabled in the current group */
    if ( TRUE != clientStaus[ pScr->currentMode ][ pScr->currentGroup ][ client ])
    {
        pScr->clientArray[ client ].state = SCR_CS_PENDING;
        pScr->clientArray[ client ].currentPendingReason = *pPendReason = SCR_PR_DIFFERENT_GROUP_RUNNING;
        return SCR_CRS_PEND;
    }
        
    /* check if a there's no running client at the moment */
    if ( SCR_CID_NO_CLIENT == pScr->runningClient )
    {
        /* no running or aborted client - allow access */
		WLAN_REPORT_INFORMATION( pScr->hReport, SCR_MODULE_LOG,
   								 ("channel allocated to client: %d\n", client) );
		pScr->clientArray[ client ].state = SCR_CS_RUNNING;
        pScr->runningClient = client;
        return SCR_CRS_RUN;
    }
    
    /* check if any client is aborting at the moment */
    if ( SCR_CS_ABORTING == pScr->clientArray[ pScr->runningClient ].state )
    {
		/* a client is currently aborting, but there still might be a pending client with higher priority
		   than the client currently requesting the SCR. If such client exists, the requesting client is
		   notified that it is pending because of this pending client, rather than the one currently aborting.
		*/
        scr_clientId_e highestPending;
		highestPending = (scr_clientId_e)scrFindHighest( hScr, SCR_CS_PENDING, (UINT8)(SCR_CID_NUM_OF_CLIENTS - 1), client );
		if ( (SCR_CID_NO_CLIENT == highestPending) ||
             (highestPending < client) )
		{
            /* if the requesting client has higher priority than the current highest priority pending client,
               the current highest priority pending client should be notified that its pending reason has 
               changed (it is no longer waiting for current running client to abort, but for the requesting
               client to finish, once the current has aborted */
            if ( (highestPending != SCR_CID_NO_CLIENT) &&
                 (SCR_PR_OTHER_CLIENT_ABORTING == pScr->clientArray[ highestPending ].currentPendingReason) )
            {

				if ( NULL != pScr->clientArray[ highestPending ].clientRequestCB )
                {
                    pScr->clientArray[ highestPending ].clientRequestCB( pScr->clientArray[ highestPending ].ClientRequestCBObj,
                                                                         SCR_CRS_PEND, SCR_PR_OTHER_CLIENT_RUNNING );
                }
                else
                {
                    WLAN_REPORT_ERROR( pScr->hReport, SCR_MODULE_LOG,
                                       ("Trying to call client %d callback, which is NULL\n", highestPending) );
                }
            }
			pScr->clientArray[ client ].currentPendingReason = *pPendReason = SCR_PR_OTHER_CLIENT_ABORTING;
		}
		else
		{
			pScr->clientArray[ client ].currentPendingReason = *pPendReason = SCR_PR_OTHER_CLIENT_RUNNING;
		}
   		pScr->clientArray[ client ].state = SCR_CS_PENDING;
        return SCR_CRS_PEND;
    }
 
    /* check if a client with higher priority is running */
    if (pScr->runningClient > client)
    {
        pScr->clientArray[ client ].state = SCR_CS_PENDING;
		pScr->clientArray[ client ].currentPendingReason = *pPendReason = SCR_PR_OTHER_CLIENT_RUNNING;
        return SCR_CRS_PEND;
    }

    /* if the client is not supposed to abort lower priority clients */
    if ( (SCR_CID_NO_CLIENT == abortOthers[ client ]) || /* client is not supposed to abort any other client */
         (pScr->runningClient > abortOthers[ client ]) ) /* client is not supposed to abort running client */
    {
        /* wait for the lower priority client */
        pScr->clientArray[ client ].state = SCR_CS_PENDING;
		pScr->clientArray[ client ].currentPendingReason = *pPendReason = SCR_PR_OTHER_CLIENT_RUNNING;
        return SCR_CRS_PEND;
    }

    /* at this point, there is a lower priority client running, that should be aborted: */
    /* mark the requesting client as pending (until the abort process will be completed) */
    pScr->clientArray[ client ].state = SCR_CS_PENDING;

    /* mark that we are in the middle of a request (if a re-entrance will occur in the complete) */
    pScr->statusNotficationPending = TRUE;

    /* abort the running client */
    pScr->clientArray[ pScr->runningClient ].state = SCR_CS_ABORTING;
    if ( NULL != pScr->clientArray[ pScr->runningClient ].clientRequestCB )
    {
		WLAN_REPORT_INFORMATION( pScr->hReport, SCR_MODULE_LOG,
	   							 ("Sending abort request to client %d\n", pScr->runningClient) );
		pScr->clientArray[ pScr->runningClient ].clientRequestCB( pScr->clientArray[ pScr->runningClient ].ClientRequestCBObj,
                                                                  SCR_CRS_ABORT,
                                                                  SCR_PR_NONE );
    }
    else
    {
        WLAN_REPORT_ERROR( pScr->hReport, SCR_MODULE_LOG,
                           ("Trying to call client %d callback, which is NULL\n", pScr->runningClient) );
    }

    /* mark that we have finished the request process */
    pScr->statusNotficationPending = FALSE;

    /* return the current status (in case the completion changed the client status to run) */
    if ( SCR_CS_RUNNING == pScr->clientArray[ client ].state )
    {
		WLAN_REPORT_INFORMATION( pScr->hReport, SCR_MODULE_LOG,
   								 ("channel allocated to client: %d\n", client) );
        return SCR_CRS_RUN;
    }
    else
    {
		pScr->clientArray[ client].currentPendingReason = *pPendReason = SCR_PR_OTHER_CLIENT_ABORTING;
        return SCR_CRS_PEND;
    }
}

/**
 * \author Ronen Kalish\n
 * \date 01-Dec-2004\n
 * \brief Notifies the SCR that the client doe not require the channel any longer
 *
 * This function can be called both by clients that are in possession of the channel, and by
 * clients that are pending to use the channel.\n
 * Function Scope \e Public.\n
 * \param hScr - handle to the SCR object.\n
 * \param client - the client releasing the channel.\n
 * \return OK if successful, NOK otherwise.\n
 */
void scr_clientComplete( TI_HANDLE hScr, scr_clientId_e client )
{
    int highestPending;
    scr_t *pScr = (scr_t*)hScr;

    WLAN_REPORT_INFORMATION( pScr->hReport, SCR_MODULE_LOG, ("Client %d releasing the channel.\n", client) );

#ifdef TI_DBG
    if (client >= SCR_CID_NUM_OF_CLIENTS)
    {
        WLAN_REPORT_ERROR( pScr->hReport, SCR_MODULE_LOG,
                           ("Attempting to release SCR for invalid client %d\n", client) );
        return;
    }
#endif

    /* mark client state as idle */
    pScr->clientArray[ client ].state = SCR_CS_IDLE;
	pScr->clientArray[ client ].currentPendingReason = SCR_PR_NONE;

    /* if completing client is running (or aborting) */
    if ( pScr->runningClient == client )   
    {
        /* mark no running client */
        pScr->runningClient = SCR_CID_NO_CLIENT;

        /* find the pending client with highest priority */
        highestPending = scrFindHighest( hScr, SCR_CS_PENDING, (UINT8)(SCR_CID_NUM_OF_CLIENTS-1), 0 );
    
        /* if a pending client exists */
        if ( SCR_CID_NO_CLIENT != highestPending )
        {
            /* mark the client with highest priority as running */
            pScr->clientArray[ highestPending ].state = SCR_CS_RUNNING;
            pScr->clientArray[ highestPending ].currentPendingReason = SCR_PR_NONE;
            pScr->runningClient = (scr_clientId_e)highestPending;
        
            /* if the SCR is not called from within a client request (re-entrance) */
            if ( FALSE == pScr->statusNotficationPending )
            {
                if ( NULL != pScr->clientArray[ highestPending ].clientRequestCB )
                {
                    pScr->clientArray[ highestPending ].clientRequestCB( pScr->clientArray[ highestPending ].ClientRequestCBObj,
                                                                         SCR_CRS_RUN, SCR_PR_NONE );
                }
                else
                {
                    WLAN_REPORT_ERROR( pScr->hReport, SCR_MODULE_LOG,
                                       ("Trying to call client %d callback, which is NULL\n", highestPending) );
                }
            }
        }
    }
}

/**
 * \author Ronen Kalish\n
 * \date 01-Dec-2004\n
 * \brief Searches the client database for a client with matching state, from startFrom to endAt (inclusive).
 * \brief Only searches for clients that are enabled at the current group!!!!\n
 *
 * Function Scope \e Private.\n
 * \param hScr - handle to the SCR object.\n
 * \param requiredState - the state to match.\n
 * \param startFrom - the highest priority to begin searching from.\n
 * \param endAt - the lowest priority to include in the search 
 * \return the client ID if found, -1 otherwise.\n
 */
INT8 scrFindHighest( TI_HANDLE hScr,
                     scr_clientState_e requiredState,
                     int startFrom,
                     int endAt )
{
    int i;
    scr_t *pScr = (scr_t*)hScr;

    /* loop on all clients, from start to end */
    for ( i = startFrom; i >= endAt; i-- )
    {
        /* check if the client state matches the required state */
        if ( (TRUE == clientStaus[ pScr->currentMode ][ pScr->currentGroup ][ i ]) && /* client is enabled in current group */
             (requiredState == pScr->clientArray[ i ].state) ) /* client is in required state */
        {
            /* and if so, return the client index */
            return i;
        }
    }

    return SCR_CID_NO_CLIENT;
}
