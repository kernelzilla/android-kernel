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


/** \file smeSm.c
 *  \brief SME SM implementation
 *
 *  \see smeSm.h
 */

#include "osTIType.h"
#include "osApi.h"
#include "paramOut.h"
#include "paramIn.h"
#include "srcApi.h"
#include "report.h"
#include "connApi.h"
#include "siteMgrApi.h"
#include "smeSmApi.h"
#include "utils.h"  
#include "fsm.h"  
#include "smeSm.h" 
#include "smeApi.h" 
#include "DataCtrl_Api.h"
#include "regulatoryDomainApi.h"
#include "TrafficMonitorAPI.h"
#include "PowerMgr_API.h"
#include "EvHandler.h"
#include "TI_IPC_Api.h"
#include "ScanCncnApi.h"
#ifdef EXC_MODULE_INCLUDED
#include "DataCtrl_Api.h"
#endif
#include "apConn.h"


/* State Machine Functions */
static TI_STATUS idle_to_interScan(void *pData);

static TI_STATUS interScan_to_scanning(void *pData);

static TI_STATUS interScan_to_idle(void *pData);

static TI_STATUS scan_to_idle(void *pData);

static TI_STATUS scan_to_scan(void *pData);

static TI_STATUS scan_to_select(void *pData);

static TI_STATUS scan_to_interScan(void *pData);

static TI_STATUS select_to_InterScan(void *pData);

static TI_STATUS select_to_connect(void *pData);

static TI_STATUS connecting_To_Disconnecting(void *pData);

static TI_STATUS connecting_to_connected(void *pData);

static TI_STATUS connecting_to_selecting(void *pData);

static TI_STATUS connected_to_interScan(void *pData);

static TI_STATUS connected_To_disconnecting(void *pData);

static TI_STATUS disconnecting_to_interScan(void *pData);


/* Local functions prototypes */
static TI_STATUS actionUnexpected(void *pData);

static TI_STATUS actionNop(void *pData);

static TI_STATUS smeCallScan(void *pData);

static TI_STATUS chooseScanBand(smeSm_t* pSmeSm, radioBand_e *band);

static TI_STATUS smeSm_changeBandParams(TI_HANDLE	hSmeSm, radioBand_e radioBand);

static TI_STATUS smeSm_startInterScanTimeout(TI_HANDLE hSmeSm);

static TI_STATUS smeSm_stopInterScanTimeout(TI_HANDLE hSmeSm);

static void		 smeSm_sendDisassociateEvent(smeSm_t* pSmeSm);

/****************************************************/
/*		Interface Functions Implementations			*/
/****************************************************/


/***********************************************************************
 *                        smeSm_smCreate									
 ***********************************************************************
DESCRIPTION: State machine creation function, called by the SME SM API. Allocates the state machine 
                                                                                                   
INPUT:      hOs	-	OS handle.

OUTPUT:		

RETURN:     State machine pointer on success, NULL otherwise

************************************************************************/
fsm_stateMachine_t *smeSm_smCreate(TI_HANDLE hOs)
{
	TI_STATUS status;
	fsm_stateMachine_t *pFsm;

	status = fsm_Create(hOs, &pFsm, SME_SM_NUM_STATES, SME_SM_NUM_EVENTS);
	
	if (status != OK)
		return NULL;

	return pFsm;
}

/***********************************************************************
 *                        smeSm_smConfig									
 ***********************************************************************
DESCRIPTION: State machine configuration function, called by the SME SM API. Configures the state machine 
                                                                                                   
INPUT:      pSmeSm	-	SME SM handle.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS smeSm_smConfig(smeSm_t *pSmeSm)
{
    paramInfo_t param;

	fsm_actionCell_t    smMatrix[SME_SM_NUM_STATES][SME_SM_NUM_EVENTS] =
	{
		/* next state and actions for IDLE state */
		{
			{SME_SM_STATE_INTER_SCAN, idle_to_interScan},       /*  "EVENT_START",                   */
			{SME_SM_STATE_IDLE, actionUnexpected},         		/*  "EVENT_STOP",                    */
			{SME_SM_STATE_IDLE, actionUnexpected},         		/*  "EVENT_SCAN_COMPLETE",           */
			{SME_SM_STATE_IDLE, actionUnexpected},         		/*  "EVENT_SELECT_SUCCESS",          */
			{SME_SM_STATE_IDLE, actionUnexpected},         		/*  "EVENT_SELECT_FAILURE",          */
			{SME_SM_STATE_IDLE, actionUnexpected},         		/*  "EVENT_CONN_SUCCESS",            */
			{SME_SM_STATE_IDLE, actionUnexpected},         		/*  "EVENT_CONN_FAILURE",                  */
			{SME_SM_STATE_IDLE, actionNop},     		    	/*  "EVENT_RESELECT",                */
			{SME_SM_STATE_IDLE, actionNop},     		    	/*  "EVENT_DISCONNECT"*/
		},

		/* next state and actions for SCANNING state */
		{	
			{SME_SM_STATE_SCANNING, actionUnexpected},          /*  "EVENT_START",                   */
			{SME_SM_STATE_IDLE,      scan_to_idle},				/*  "EVENT_STOP",                    */
			{SME_SM_STATE_SELECTING, scan_to_select},           /*  "EVENT_SCAN_COMPLETE",           */
			{SME_SM_STATE_SCANNING, actionUnexpected},          /*  "EVENT_SELECT_SUCCESS",          */
			{SME_SM_STATE_SCANNING, actionUnexpected},          /*  "EVENT_SELECT_FAILURE",          */
			{SME_SM_STATE_SCANNING, actionUnexpected},          /*  "EVENT_CONN_SUCCESS",            */
			{SME_SM_STATE_SCANNING, actionUnexpected},          /*  "EVENT_CONN_FAILURE",                  */
			{SME_SM_STATE_SCANNING, scan_to_scan},              /*  "EVENT_RESELECT",*/
			{SME_SM_STATE_INTER_SCAN, scan_to_interScan},     	/*  "EVENT_DISCONNECT"*/
		},

		/* next state and actions for SELECTING state */
		{	
			{SME_SM_STATE_SELECTING, actionUnexpected},         /*  "EVENT_START",                   */
			{SME_SM_STATE_SELECTING, actionUnexpected},			/*  "EVENT_STOP",                    */
			{SME_SM_STATE_SELECTING, actionUnexpected},         /*  "EVENT_SCAN_COMPLETE",           */
			{SME_SM_STATE_CONNECTING, select_to_connect},       /*  "EVENT_SELECT_SUCCESS",          */
			{SME_SM_STATE_INTER_SCAN, select_to_InterScan},	   	/*  "EVENT_SELECT_FAILURE",          */
			{SME_SM_STATE_SELECTING, actionUnexpected},         /*  "EVENT_CONN_SUCCESS",            */
			{SME_SM_STATE_SELECTING, actionUnexpected},         /*  "EVENT_CONN_FAILURE",                   */
    		{SME_SM_STATE_SELECTING, actionUnexpected},         /*  "EVENT_RESELECT",                */
			{SME_SM_STATE_SELECTING, actionUnexpected},         /*  "EVENT_DISCONNECT",*/
		},

		/* next state and actions for CONNECTING state */
		{	
			{SME_SM_STATE_SCANNING,      actionUnexpected},               /*  "EVENT_START",                   */
			{SME_SM_STATE_DISCONNECTING, connecting_To_Disconnecting},    /*  "EVENT_STOP",                    */
			{SME_SM_STATE_CONNECTING,    actionUnexpected},               /*  "EVENT_SCAN_COMPLETE",           */
			{SME_SM_STATE_CONNECTING,    actionUnexpected},               /*  "EVENT_SELECT_SUCCESS",          */
			{SME_SM_STATE_CONNECTING,    actionUnexpected},               /*  "EVENT_SELECT_FAILURE",          */
			{SME_SM_STATE_CONNECTED,     connecting_to_connected},        /*  "EVENT_CONN_SUCCESS",            */
			{SME_SM_STATE_SELECTING,     connecting_to_selecting},        /*  "EVENT_CONN_FAILURE",                  */
			{SME_SM_STATE_DISCONNECTING, connecting_To_Disconnecting},    /*  "EVENT_RESELECT",               */
			{SME_SM_STATE_DISCONNECTING, connecting_To_Disconnecting},    /*  "EVENT_DISCONNECT", */
		},

		/* next state and actions for CONNECTED state */
		{	
			{SME_SM_STATE_SCANNING, actionUnexpected},			           	/*  "EVENT_START",                   */
			{SME_SM_STATE_DISCONNECTING, connected_To_disconnecting},	   	/*  "EVENT_STOP",                    */
			{SME_SM_STATE_CONNECTED, actionUnexpected},				 		/* "EVENT_SCAN_COMPLETE",           */
			{SME_SM_STATE_CONNECTED, actionUnexpected},						/*  "EVENT_SELECT_SUCCESS",          */
			{SME_SM_STATE_CONNECTED, actionUnexpected},						/*  "EVENT_SELECT_FAILURE",          */
			{SME_SM_STATE_CONNECTED, actionUnexpected},		        		/*  "EVENT_CONN_SUCCESS",            */
			{SME_SM_STATE_INTER_SCAN, connected_to_interScan},		        /*  "EVENT_CONN_FAILURE",                   */
			{SME_SM_STATE_DISCONNECTING, connected_To_disconnecting},      	/*  "EVENT_RESELECT",                */
			{SME_SM_STATE_DISCONNECTING	,connected_To_disconnecting},      	/*  "EVENT_DISCONNECT",*/
		},

	    /* next state and actions for DISCONNECTING state */
		{	
			{SME_SM_STATE_DISCONNECTING,      actionNop},   				/*  "EVENT_START",                   */
			{SME_SM_STATE_DISCONNECTING,      actionNop},  					/*  "EVENT_STOP",                    */
			{SME_SM_STATE_DISCONNECTING, actionUnexpected}, 				/*  "EVENT_SCAN_COMPLETE",           */
			{SME_SM_STATE_DISCONNECTING, actionUnexpected}, 				/*  "EVENT_SELECT_SUCCESS",          */
			{SME_SM_STATE_DISCONNECTING, actionUnexpected}, 				/*  "EVENT_SELECT_FAILURE",          */
			{SME_SM_STATE_DISCONNECTING, actionUnexpected},  				/*  "EVENT_CONN_SUCCESS",            */
			{SME_SM_STATE_INTER_SCAN, disconnecting_to_interScan}, 			/*  "EVENT_CONN_FAILURE",                  */
			{SME_SM_STATE_DISCONNECTING,  	   actionNop}, 			  	    /*  "EVENT_RESELECT",               */
			{SME_SM_STATE_DISCONNECTING,  	   actionNop}, 			  	    /*  "EVENT_DISCONNECT",*/
		},


		/* next state and actions for STATE_INTER_SCAN_TIMEOUT state */
		{
			{SME_SM_STATE_INTER_SCAN, actionUnexpected},          /*  "EVENT_START",                   */
			{SME_SM_STATE_IDLE, interScan_to_idle},				  /*  "EVENT_STOP",                    */
			{SME_SM_STATE_INTER_SCAN, actionUnexpected},          /*  "EVENT_SCAN_COMPLETE",           */
			{SME_SM_STATE_INTER_SCAN, actionUnexpected},          /*  "EVENT_SELECT_SUCCESS",          */
			{SME_SM_STATE_INTER_SCAN, actionUnexpected},          /*  "EVENT_SELECT_FAILURE",          */
			{SME_SM_STATE_INTER_SCAN, actionUnexpected},          /*  "EVENT_CONN_SUCCESS",            */
			{SME_SM_STATE_INTER_SCAN, actionUnexpected},          /*  "EVENT_CONN_FAILURE",                  */
			{SME_SM_STATE_SCANNING, interScan_to_scanning},       /*  "EVENT_RESELECT",                */
			{SME_SM_STATE_INTER_SCAN, actionNop},     			  /*  "EVENT_DISCONNECT",*/
		},
	
	};

	pSmeSm->dualBandReScanFlag = FALSE;
	pSmeSm->reScanFlag = FALSE;
	pSmeSm->radioOn    = FALSE;
    pSmeSm->immediateShutdownRequired = FALSE;
	
    /* if desired SSID is junk SSID, don't connect (until new SSID is set) */
    param.paramType = SITE_MGR_DESIRED_SSID_PARAM;
    siteMgr_getParam( pSmeSm->hSiteMgr, &param );
    
    if ( utils_isJunkSSID( &(param.content.siteMgrDesiredSSID) ) )
    {
        pSmeSm->connectEnabled = FALSE;
    }
    else
    {
        pSmeSm->connectEnabled = TRUE;
    }

	/* First event should indicate to the upper layer that STA is disassociated. */
	pSmeSm->DisAssoc.mgmtStatus = STATUS_UNSPECIFIED;
	pSmeSm->DisAssoc.uStatusCode = 0;
	smeSm_sendDisassociateEvent(pSmeSm);

#ifdef TI_DBG
	/* clear statistics */
	smeSm_resetStats( (TI_HANDLE)pSmeSm );
#endif

	return fsm_Config(pSmeSm->pFsm, (fsm_Matrix_t)smMatrix, SME_SM_NUM_STATES, SME_SM_NUM_EVENTS, smeSm_SMEvent, pSmeSm->hOs);
}

/***********************************************************************
 *                        smeSm_smUnLoad									
 ***********************************************************************
DESCRIPTION: State machine unload function, called by the SME SM API. Unloads the state machine 
                                                                                                   
INPUT:      hOs		-	OS handle.
			pFsm	-	Pointer to the state machine

OUTPUT:		

RETURN:     State machine pointer on success, NULL otherwise

************************************************************************/
TI_STATUS smeSm_smUnLoad(TI_HANDLE hOs, fsm_stateMachine_t *pFsm)
{
	fsm_Unload(hOs, pFsm);
	
	return OK;
}

/***********************************************************************
 *                        smeSm_SMEvent									
 ***********************************************************************
DESCRIPTION: SME SM event processing function, called by the SME SM API
				Perform the following:
				-	Print the state movement as a result from the event
				-	Calls the generic state machine event processing function which preform the following:
					-	Calls the corresponding callback function
					-	Move to next state
				
INPUT:		currentState	-	Pointer to the connection current state.
			event	-	Received event
			pSmeSm	-	SME SM handle

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/

#ifdef REPORT_LOG

static char *stateDesc[SME_SM_NUM_STATES] = 
	{
		"STATE_IDLE",
		"STATE_SCANNING",
		"STATE_SELECTING",
		"STATE_CONNECTING",
		"STATE_CONNECTED",
		"STATE_DISCONNECTING",
		"STATE_INTER_SCAN",
	};

static char *eventDesc[SME_SM_NUM_EVENTS] = 
	{
		"EVENT_START",
		"EVENT_STOP",
		"EVENT_SCAN_COMPLETE",
		"EVENT_SELECT_SUCCESS",
		"EVENT_SELECT_FAILURE",
		"EVENT_CONN_SUCCESS",
		"EVENT_CONN_FAILURE",
		"EVENT_RESELECT",
		"EVENT_DISCONNECT",
	};

#endif


TI_STATUS smeSm_SMEvent(UINT8 *currentState, UINT8 event, TI_HANDLE hSmeSm)
{
   smeSm_t *pSmeSm = (smeSm_t *)hSmeSm;
	TI_STATUS 		status;
	UINT8		nextState;

	status = fsm_GetNextState(pSmeSm->pFsm, *currentState, event, &nextState);
	if (status != OK)
	{
		WLAN_REPORT_SM(pSmeSm->hReport, SME_SM_MODULE_LOG, ("State machine error, failed getting next state\n"));
		return(NOK);
	}

	WLAN_REPORT_SM(pSmeSm->hReport, SME_SM_MODULE_LOG, 
							  ("<%s, %s> --> %s\n\n",
							   stateDesc[*currentState],
							   eventDesc[event],
							   stateDesc[nextState]));

	status = fsm_Event(pSmeSm->pFsm, currentState, event, (void *)pSmeSm);

	return status;
}


/************************************************************************************************************/
/*		In the following section are listed the callback function used by the SME state machine				*/
/************************************************************************************************************/

/* START_SCAN */ 
TI_STATUS sme_startScan(void *pData)
{
	paramInfo_t	param;
	smeSm_t *pSmeSm = (smeSm_t *)pData;
   
   /*
	* Support Dual Mode Operation
	*/ 

    radioBand_e	band; 

	/*
	 * If scan is disabled then send self scan complete event, skipping the scan procedure.
	 */ 
	if (pSmeSm->scanEnabled != SCAN_ENABLED)
	{
		if (pSmeSm->scanEnabled == SKIP_NEXT_SCAN) 
		{
			pSmeSm->scanEnabled = SCAN_ENABLED;
		}
		return smeSm_SMEvent(&pSmeSm->state, SME_SM_EVENT_SCAN_COMPLETE , pSmeSm);
	}
	

    /* if performing a dual band rescan now */
    if ( TRUE == pSmeSm->dualBandReScanFlag )
    {
        /* mark that no more scans are needed */
        pSmeSm->dualBandReScanFlag = FALSE;
		chooseScanBand(pSmeSm, &band);
		smeSm_changeBandParams(pSmeSm, band);
    }
    else
    {
        param.paramType = SITE_MGR_DESIRED_DOT11_MODE_PARAM;
        siteMgr_getParam(pSmeSm->hSiteMgr, &param);

        if(param.content.siteMgrDot11Mode == DOT11_DUAL_MODE) 
        {
	        pSmeSm->dualBandReScanFlag = TRUE;
	        chooseScanBand(pSmeSm, &band);
	        smeSm_changeBandParams(pSmeSm, band);
        }
    }

    return smeCallScan(pData);
}


/* RESTART_SCAN when exiting from IDLE state*/ 
static TI_STATUS idle_to_interScan(void *pData)
{
	smeSm_t         *pSmeSm = (smeSm_t *)pData;

	pSmeSm->interScanTimeout = pSmeSm->interScanTimeoutMin;

    /* Set the SCR group to inter SCAN */
    scr_setGroup( pSmeSm->hScr, SCR_GID_INTER_SCAN );

    /*
	 * If the connection is enabled then initiate "reselect" event that moves the
	 * SM into scan state. 
	 */
	if( pSmeSm->connectEnabled ){
		return smeSm_SMEvent(&pSmeSm->state, SME_SM_EVENT_RESELECT , pSmeSm);
	}
	/* connectEnabled is FALSE, need to stay at this state until it will be changed. */

	return OK;
}



TI_STATUS interScan_to_scanning(void *pData)
{
	smeSm_t *pSmeSm = (smeSm_t *)pData;
	
	pSmeSm->bSendDisAssocEvent = FALSE; 
	
	smeSm_stopInterScanTimeout(pSmeSm);
		
    /* Set SCR to "first connection" */
	scr_setGroup( pSmeSm->hScr, SCR_GID_CONNECT );

#ifdef TI_DBG
	/* update statistics - count scan attempts for connection */
	pSmeSm->smeStats.currentNumberOfScanAttempts++;
#endif
	return (sme_startScan(pData));
}



static TI_STATUS disconnecting_to_interScan(void *pData)
{
	smeSm_t *pSmeSm = (smeSm_t *)pData;

	pSmeSm->interScanTimeout = pSmeSm->interScanTimeoutMin;

    scr_setGroup( pSmeSm->hScr, SCR_GID_INTER_SCAN );
    
    siteMgr_disSelectSite(pSmeSm->hSiteMgr);

	/*
	 *  Notify that the driver is disassociated to the supplicant\IP stack. 
	 */
	smeSm_sendDisassociateEvent(pSmeSm);

	if (!pSmeSm->scanEnabled) 
	{
		pSmeSm->connectEnabled = FALSE;
	}

	/* Radio ON and connection is enabled go to scanning */
	if( pSmeSm->radioOn )
	{
		if(	pSmeSm->connectEnabled )
			return smeSm_SMEvent(&pSmeSm->state, SME_SM_EVENT_RESELECT , pSmeSm);

		else {
			/* Radio is on but connection is disabled, stay at interScan state 
				without starting the interscan timer. */
	
			/* SCR is set to enable only APP scan */
			return OK;
		}
	}	

	else{ 
			return smeSm_SMEvent(&pSmeSm->state, SME_SM_EVENT_STOP , pSmeSm);
	}
}


static TI_STATUS connected_to_interScan(void *pData)
{
	smeSm_t *pSmeSm = (smeSm_t *)pData;

	pSmeSm->interScanTimeout = pSmeSm->interScanTimeoutMin;
	
	siteMgr_disSelectSite(pSmeSm->hSiteMgr);

	scr_setGroup( pSmeSm->hScr, SCR_GID_INTER_SCAN );

	/*
	 *  Notify that the driver is associated to the supplicant\IP stack. 
	 */
	smeSm_sendDisassociateEvent(pSmeSm);

	if (!pSmeSm->scanEnabled) 
	{
		pSmeSm->connectEnabled = FALSE;
	}

	return smeSm_SMEvent(&pSmeSm->state, SME_SM_EVENT_RESELECT , pSmeSm);
}


static TI_STATUS select_to_InterScan(void *pData)
{
	smeSm_t *pSmeSm = (smeSm_t *)pData;
		
	siteMgr_disSelectSite(pSmeSm->hSiteMgr);

	/* SCR is set to enable only APP scan */
    scr_setGroup( pSmeSm->hScr, SCR_GID_INTER_SCAN );
    
    if (pSmeSm->connectEnabled)
	{
		smeSm_startInterScanTimeout(pSmeSm);
	}

	if ((pSmeSm->bSendDisAssocEvent == TRUE) || (pSmeSm->scanEnabled == FALSE))
    {
		/*
		 *  Notify that the driver is disassociated to the supplicant\IP stack. 
		 */
		smeSm_sendDisassociateEvent(pSmeSm);
	}

	return OK;
}



static TI_STATUS scan_to_interScan(void *pData)
{
	smeSm_t *pSmeSm = (smeSm_t *)pData;
   	
	/* SCR is set to enable only APP scan */
	scr_setGroup( pSmeSm->hScr, SCR_GID_INTER_SCAN );

	pSmeSm->interScanTimeout = pSmeSm->interScanTimeoutMin;

	if (pSmeSm->connectEnabled)
	{
		smeSm_startInterScanTimeout(pSmeSm);
	}

	return OK;

}


/* STOP_SCAN, START_SCAN */ 
static TI_STATUS scan_to_scan(void *pData)
{
	smeSm_t *pSmeSm = (smeSm_t *)pData;
	
	WLAN_REPORT_INFORMATION(pSmeSm->hReport, SME_SM_MODULE_LOG, ("Setting reScanFlag to ON\n"));
	
	pSmeSm->reScanFlag = TRUE;
	
	scanConcentrator_stopScan( pSmeSm->hScanCncn, SCAN_SCC_DRIVER );
	
	return OK;
}



/* CANCEL INTER_SCAN TIMEOUT*/ 
static TI_STATUS interScan_to_idle(void *pData)
{
	smeSm_t *pSmeSm = (smeSm_t *)pData;

	smeSm_stopInterScanTimeout(pSmeSm);
	
    /* Change shutdown status flag to indicate SME is in IDLE state */
    pSmeSm->shutDownStatus |= DRIVER_SHUTDOWN_SME_STOPPED;

	/* Set SCR state to "Idle" */
    scr_setGroup( pSmeSm->hScr, SCR_GID_IDLE );
    
	return OK;
	
}


/* Stop current scanning and  go to idle */ 
static TI_STATUS scan_to_idle(void *pData)
{
	smeSm_t *pSmeSm = (smeSm_t *)pData;

	/* stopping the current active scan process */
	scanConcentrator_stopScan( pSmeSm->hScanCncn, SCAN_SCC_DRIVER );

    /* Change shutdown status flag to indicate SME is in IDLE state */
    pSmeSm->shutDownStatus |= DRIVER_SHUTDOWN_SME_STOPPED;

	/* Set the SCR group to "idle" */
    scr_setGroup( pSmeSm->hScr, SCR_GID_IDLE );

	return OK;
}

/* SELECT */ 
static TI_STATUS scan_to_select(void *pData)
{
	smeSm_t 		*pSmeSm = (smeSm_t *)pData;

	if (!pSmeSm->connectEnabled)
	{
		return smeSm_SMEvent(&pSmeSm->state, SME_SM_EVENT_SELECT_FAILURE, pSmeSm);
	}

	siteMgr_resetAttemptsNumberParameter(pSmeSm->hSiteMgr);

	return siteMgr_selectSite(pSmeSm->hSiteMgr);
}



/* START_TX, CONNECT */ 
static TI_STATUS select_to_connect(void *pData)
{
	smeSm_t *pSmeSm = (smeSm_t *)pData;
    paramInfo_t param;

	/* From this moment forward we will send DisAssociation event even if the connection failed */
	pSmeSm->bSendDisAssocEvent = TRUE;

    /* if we are about o start an IBSS, the conn SM will wait for few seconds before sending connection
       failure indication, so that the SME will rescan for IBSSes with the same SSID. To enable application
       scan during this period, in this case ONLY we change here the SCR group ID to inter scan */
   	param.paramType = CONN_TYPE_PARAM;
	conn_getParam(pSmeSm->hConn, &param);
	if (CONNECTION_SELF == param.content.connType)
	{
        /* Set SCR group to inter-scan */
        scr_setGroup( pSmeSm->hScr, SCR_GID_INTER_SCAN );
	}

	/* Configure QoS manager not to renegotiate TSPECs as this is first time connection */
	return conn_start(pSmeSm->hConn, CONN_TYPE_FIRST_CONN, smeSm_reportConnStatus, pSmeSm, FALSE, FALSE);
}  




/* Stop the connecting and go to disconnecting */ 
static TI_STATUS connecting_To_Disconnecting(void *pData)
{
	smeSm_t *pSmeSm = (smeSm_t *)pData;

	conn_stop(pSmeSm->hConn, DISCONN_TYPE_DEAUTH, STATUS_UNSPECIFIED,
					   TRUE, smeSm_reportConnStatus, pSmeSm);

	return OK;
}


/* Stop the connection and go to disconnecting */ 
static TI_STATUS connected_To_disconnecting(void *pData)
{
	TI_STATUS status;
	paramInfo_t		param;
	smeSm_t *pSmeSm = (smeSm_t *)pData;

	scr_setGroup( pSmeSm->hScr, SCR_GID_CONNECT );

	param.paramType = SITE_MGR_CURRENT_BSS_TYPE_PARAM;
	siteMgr_getParam(pSmeSm->hSiteMgr, &param);
	
	if(param.content.siteMgrDesiredBSSType == BSS_INFRASTRUCTURE)
	{
		/* Call the AP connection to perform disconnect - If immidiateShutdownRequired is TRUE, no need to send DISASSOC frame */
		 status = apConn_stop(pSmeSm->hApConn, TRUE, pSmeSm->immediateShutdownRequired);
	}
	else 
	{
	    /* In IBSS disconnect is done directly with the connection SM */ 
		status = conn_stop(pSmeSm->hConn, DISCONN_TYPE_DEAUTH, STATUS_UNSPECIFIED,
						   TRUE, smeSm_reportConnStatus,pSmeSm);
		if (status != OK)
		{
			WLAN_REPORT_ERROR(pSmeSm->hReport, SME_SM_MODULE_LOG, ("SME status=%d, have to return (%d)\n",status,__LINE__));
			return status; 
		}
	}


	return OK;
}

/* RESET_ATTEMPTS_NUMBER, START_SITE_AGING, START_DATA_ALGO */ 
/* CONNECTING -> CONNECTED due to event CONN_SUCC */
static TI_STATUS connecting_to_connected(void *pData)
{
	TI_STATUS status;
	paramInfo_t	param;
	
	smeSm_t *pSmeSm = (smeSm_t *)pData;

	/* Reset the DisAssociation reason since the SME saves that last reason, and if a new connection was made and than aborted by
	   unspecified reason - no one will clear this value */
	pSmeSm->DisAssoc.mgmtStatus = STATUS_UNSPECIFIED;
	pSmeSm->DisAssoc.uStatusCode = 0;

	status = siteMgr_resetPrevPrimarySiteRssi(pSmeSm->hSiteMgr);
	if (status != OK)
      {
         WLAN_REPORT_ERROR(pSmeSm->hReport, SME_SM_MODULE_LOG, ("SME status=%d, have to return (%d)\n",status,__LINE__));
         return status; 
      }

	status = siteMgr_resetPrimarySiteAttemptsNumber(pSmeSm->hSiteMgr);
	if (status != OK)
      {
         WLAN_REPORT_ERROR(pSmeSm->hReport, SME_SM_MODULE_LOG, ("SME status=%d, have to return (%d)\n",status,__LINE__));
         return status; 
      }

	status = siteMgr_resetEventStatisticsHistory(pSmeSm->hSiteMgr);
	if (status != OK)
      {
         WLAN_REPORT_ERROR(pSmeSm->hReport, SME_SM_MODULE_LOG, ("SME status=%d, have to return (%d)\n",status,__LINE__));
         return status; 
      }
    

	param.paramType = SITE_MGR_CURRENT_BSS_TYPE_PARAM;
	siteMgr_getParam(pSmeSm->hSiteMgr, &param);
	if (param.content.siteMgrDesiredBSSType == BSS_INFRASTRUCTURE)
	{
		/* Start the AP connection */
		apConn_start(pSmeSm->hApConn, siteMgr_isThereValidSSID(pSmeSm->hSiteMgr));
	}

    /* Set SCR group to connected */
    scr_setGroup( pSmeSm->hScr, SCR_GID_CONNECTED );

#ifdef TI_DBG
	/* update statistics - scan attempts for connection histogran */
	if ( SCAN_ATTAMEPTS_HISTOGRAM_SIZE < pSmeSm->smeStats.currentNumberOfScanAttempts )
	{
		pSmeSm->smeStats.scanAttemptsForConnectionHistogram[ SCAN_ATTAMEPTS_HISTOGRAM_SIZE - 1 ]++;
	}
	else
	{
		pSmeSm->smeStats.scanAttemptsForConnectionHistogram[ pSmeSm->smeStats.currentNumberOfScanAttempts - 1 ]++;
	}
	pSmeSm->smeStats.currentNumberOfScanAttempts = 0;
#endif
    return OK; 
}



/* STOP_RX, STOP_TX, UPDATE_PRIMARY_SITE_FAIL_STATUS,  */ 
static TI_STATUS connecting_to_selecting(void *pData)
{
	TI_STATUS status;
	paramInfo_t	param;
    smeSm_t *pSmeSm = (smeSm_t *)pData;

	conn_stop(pSmeSm->hConn, DISCONN_TYPE_IMMEDIATE, STATUS_UNSPECIFIED,
			  TRUE, smeSm_reportConnStatus,pSmeSm); 

	/* Remove primary site */
	status = siteMgr_updatePrimarySiteFailStatus(pSmeSm->hSiteMgr, TRUE);
	if (status != OK)
      {
         WLAN_REPORT_ERROR(pSmeSm->hReport, SME_SM_MODULE_LOG, ("SME status=%d, have to return (%d)\n",status,__LINE__));
         return status; 
      }
	
	status = siteMgr_disSelectSite(pSmeSm->hSiteMgr);
	if (status != OK)
      {
         WLAN_REPORT_ERROR(pSmeSm->hReport, SME_SM_MODULE_LOG, ("SME status=%d, have to return (%d)\n",status,__LINE__));
         return status; 
      }

	/* If we are in IBSS mode we don't try to select another site */
	param.paramType = SITE_MGR_DESIRED_BSS_TYPE_PARAM;
	siteMgr_getParam(pSmeSm->hSiteMgr, &param);

	if(param.content.siteMgrDesiredBSSType == BSS_INDEPENDENT)
		return smeSm_reportSelectStatus(pSmeSm, (mgmtStatus_e)SELECT_STATUS_FAILURE);
	else
		return siteMgr_selectSite(pSmeSm->hSiteMgr);
}




static TI_STATUS chooseScanBand(smeSm_t* pSmeSm, radioBand_e *band)
{		
	paramInfo_t	param;

    param.paramType = SITE_MGR_DESIRED_DOT11_MODE_PARAM;
    siteMgr_getParam( pSmeSm->hSiteMgr, &param );

	/* if working in dual band, select band according to dualBandReScanFlag */
    if ( param.content.siteMgrDot11Mode == DOT11_DUAL_MODE )
    {
        *band = ( TRUE == pSmeSm->dualBandReScanFlag ? RADIO_BAND_5_0_GHZ : RADIO_BAND_2_4_GHZ);
    }	
    /* if not working in dual band, return site manager current band */
    else
    {
    	param.paramType = SITE_MGR_RADIO_BAND_PARAM;
	    siteMgr_getParam(pSmeSm->hSiteMgr, &param);
	    *band = pSmeSm->currBand = param.content.siteMgrRadioBand;
    }
	return OK;				
}



static TI_STATUS smeCallScan(void *pData)
{
    smeSm_t	*pSmeSm = (smeSm_t *)pData;
    paramInfo_t param;
	scan_Params_t* pScanParams = &(pSmeSm->scanParams);
	sme_scan_Params_t 	*pSmeScanParams;
	scan_normalChannelEntry_t *pChanEntry;
	int	chan, k;

	/* 
	 * Check in which band we are in and prepare the scan command.
	 */
    pScanParams->scanType = SCAN_TYPE_NORMAL_ACTIVE;
    pScanParams->Tid = 0;


    param.paramType = SITE_MGR_OPERATIONAL_MODE_PARAM;
    siteMgr_getParam( pSmeSm->hSiteMgr, &param );

    if ( DOT11_A_MODE == param.content.siteMgrDot11OperationalMode )
	{	
        pScanParams->band = RADIO_BAND_5_0_GHZ;
		pSmeScanParams  = &(pSmeSm->scanParamsA);
	}
    else
	{
         pScanParams->band = RADIO_BAND_2_4_GHZ;
		 pSmeScanParams  = &(pSmeSm->scanParamsBG);
	}

	 pScanParams->probeRequestRate 	= pSmeScanParams->probeRequestRate;
	 pScanParams->numOfChannels 		= pSmeScanParams->numOfChannels;
	 pScanParams->probeReqNumber 		= pSmeScanParams->probeReqNumber;

	 WLAN_REPORT_INFORMATION(pSmeSm->hReport, SME_SM_MODULE_LOG,  
					("Global Scan Params: Rate=0x%x, Prob#=%d, Channels#=%d\n",
					 pScanParams->probeRequestRate, pScanParams->probeReqNumber, 
					 pScanParams->numOfChannels));  

	 for( chan=0; chan < pScanParams->numOfChannels; chan++ )
	 {
		pChanEntry = &(pScanParams->channelEntry[chan].normalChannelEntry);
		
		/* Fill channel ID */
		pChanEntry->channel = pSmeScanParams->channelsList[chan];
		
		/* Set broadcast BSSID */
		for ( k = 0; k < 6; k++ ) pChanEntry->bssId.addr[ k ] = 0xff;

		/* Set min & max dwell time */
		pChanEntry->minChannelDwellTime = pSmeScanParams->minDwellTime;
		pChanEntry->maxChannelDwellTime = pSmeScanParams->maxDwellTime;

		/* Set up early termination params. */	
		pChanEntry->earlyTerminationEvent =  SCAN_DEFAULT_EARLY_TERMINATION_EVENT;
		pChanEntry->ETMaxNumOfAPframes = SCAN_DEFAULT_EARLY_TERMINATION_NUM_OF_FRAMES;
		
		/* Set desired tx power */ 
		pChanEntry->txPowerDbm = pSmeScanParams->txPowerDbm;

		WLAN_REPORT_INFORMATION(pSmeSm->hReport, SME_SM_MODULE_LOG,  
					("Chan %d: CH=%d , DwellTime Min=%d Max=%d, ET=%d, TxPowr=%d, \n",
					 chan,
					 pScanParams->channelEntry[chan].normalChannelEntry.channel,
					 pScanParams->channelEntry[chan].normalChannelEntry.minChannelDwellTime,
					 pScanParams->channelEntry[chan].normalChannelEntry.maxChannelDwellTime,
					 pScanParams->channelEntry[chan].normalChannelEntry.earlyTerminationEvent,
					 pScanParams->channelEntry[chan].normalChannelEntry.txPowerDbm));
	 } /* Channel list setting */


    /*
     * Set the desired SSID (if any)
     */
    param.paramType = SITE_MGR_DESIRED_SSID_PARAM;
    siteMgr_getParam( pSmeSm->hSiteMgr, &param );

    if ( utils_isAnySSID( &(param.content.siteMgrDesiredSSID) ) ) 
    {
        pScanParams->desiredSsid.len = 0;
    }
    else
    {
        pScanParams->desiredSsid.len = param.content.siteMgrDesiredSSID.len;
        os_memoryCopy( pSmeSm->hOs, (void *)pScanParams->desiredSsid.ssidString,
                       (void *)param.content.siteMgrDesiredSSID.ssidString,
                       param.content.siteMgrDesiredSSID.len );
    }



	/*
	 * Prepare scan complete's aging, by increasing the scanned sites 
	 * scan attemps counter. The counter will be checked upon scan complete,  
	 * and the sites with no update scan results will be dropped.   
	 */
	siteMgr_setNotReceivedParameter(pSmeSm->hSiteMgr, &(pScanParams->desiredSsid), pScanParams->band );

#ifdef TI_DBG
	/* scan results count statistics - nullify the count before starting scan */
	pSmeSm->smeStats.currentNumberOfScanResults = 0;
#endif

	/* 
	 * Set and send the scan command. 
	 */
    if (scanConcentrator_scan( pSmeSm->hScanCncn, SCAN_SCC_DRIVER, pScanParams ) != 
            SCAN_CRS_SCAN_RUNNING)
	{
    	/* imitate scan complete event if scan could not be performed.*/
		smeSm_SMEvent(&pSmeSm->state, SME_SM_EVENT_SCAN_COMPLETE, pSmeSm);
	}

	return OK;
}


static TI_STATUS actionUnexpected(void *pData) 
{
	smeSm_t	*pSmeSm = (smeSm_t *)pData; 
	
	WLAN_REPORT_SM(pSmeSm->hReport, SME_SM_MODULE_LOG,  ("State machine error, unexpected Event\n\n"));
	return OK;
}



static TI_STATUS actionNop(void *pData)
{
	return OK;
}



static TI_STATUS smeSm_changeBandParams(TI_HANDLE	hSmeSm, radioBand_e radioBand)
{
	paramInfo_t	param;
	smeSm_t *pSmeSm = (smeSm_t *)hSmeSm;

	/* change dot11 mode */
	param.paramType = SITE_MGR_OPERATIONAL_MODE_PARAM;
	if(radioBand == RADIO_BAND_2_4_GHZ)
		param.content.siteMgrDot11OperationalMode = DOT11_G_MODE;
	else
		param.content.siteMgrDot11OperationalMode = DOT11_A_MODE;

	siteMgr_setParam(pSmeSm->hSiteMgr, &param);
	
	param.paramType = SITE_MGR_RADIO_BAND_PARAM;
	param.content.siteMgrRadioBand = radioBand;
	siteMgr_setParam(pSmeSm->hSiteMgr, &param);
	
	siteMgr_setCurrentTable(pSmeSm->hSiteMgr, radioBand);
	
	/* configure hal with common core-hal parameters */
	whalCtrl_SetRadioBand(pSmeSm->hHalCtrl, radioBand);
	
	return OK;
}

/***********************************************************************
 *                        smeSm_startInterScanTimeout									
 ***********************************************************************
DESCRIPTION: Starts interscan timeout
                                                                                                   
INPUT:      hSmeSm	-	SME SM handle.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
static TI_STATUS smeSm_startInterScanTimeout(TI_HANDLE hSmeSm)
{
	smeSm_t 		*pSmeSm = (smeSm_t *)hSmeSm;
	paramInfo_t		param;

	/* If the SME scan is disabled, (scan is performed by the application)
	 * don't start the inter scan timeout
	 */
	if (pSmeSm->scanEnabled == SCAN_DISABLED)
	{
		return OK;
	}

	if (pSmeSm->scanEnabled == SKIP_NEXT_SCAN) 
	{
		pSmeSm->scanEnabled = SCAN_ENABLED;
	}

	param.paramType = SITE_MGR_DESIRED_BSS_TYPE_PARAM;
	siteMgr_getParam(pSmeSm->hSiteMgr, &param);

	if (param.content.siteMgrDesiredBSSType == BSS_INDEPENDENT)
	{
		os_timerStart(pSmeSm->hOs, pSmeSm->interScanTimeoutTimer, IBSS_INTER_SCAN_PERIOD, FALSE);
	}
	else 
	{	
		os_timerStart(pSmeSm->hOs, pSmeSm->interScanTimeoutTimer, pSmeSm->interScanTimeout, FALSE);   
	}

	return OK;
}

/***********************************************************************
 *                        smeSm_stopInterScanTimeout									
 ***********************************************************************
DESCRIPTION: Stops scan timeout
                                                                                                   
INPUT:      hSmeSm	-	SME SM handle.

OUTPUT:		

RETURN:     OK on success, NOK otherwise

************************************************************************/
static TI_STATUS smeSm_stopInterScanTimeout(TI_HANDLE hSmeSm)
{
	smeSm_t *pSmeSm = (smeSm_t *)hSmeSm;

	os_timerStop(pSmeSm->hOs, pSmeSm->interScanTimeoutTimer);
	return OK;
}

/***********************************************************************
*                        smeSm_sendDisassociateEvent									
***********************************************************************
DESCRIPTION: Send disassociate event with the reason

INPUT:      pSmeSm	-	SME SM handle.

OUTPUT:		

RETURN:     

************************************************************************/
static void smeSm_sendDisassociateEvent(smeSm_t* pSmeSm)
{
	OS_802_11_DISASSOCIATE_REASON_T	eventReason;

	WLAN_REPORT_INFORMATION(pSmeSm->hReport, SME_SM_MODULE_LOG, 
		("%s uDissconnectType = %d, uStatusCode = %d\n",__FUNCTION__, pSmeSm->DisAssoc.mgmtStatus, pSmeSm->DisAssoc.uStatusCode));

	/* Convert reason to OS layer */
	switch(pSmeSm->DisAssoc.mgmtStatus)
	{
	case STATUS_UNSPECIFIED:
		eventReason.eDisAssocType = OS_DISASSOC_STATUS_UNSPECIFIED;
		break;
	case STATUS_AUTH_REJECT:
		eventReason.eDisAssocType = OS_DISASSOC_STATUS_AUTH_REJECT;
	    break;
	case STATUS_ASSOC_REJECT:
		eventReason.eDisAssocType = OS_DISASSOC_STATUS_ASSOC_REJECT;
	    break;
	case STATUS_SECURITY_FAILURE:
		eventReason.eDisAssocType = OS_DISASSOC_STATUS_SECURITY_FAILURE;
		break;
	case STATUS_AP_DEAUTHENTICATE:
		eventReason.eDisAssocType = OS_DISASSOC_STATUS_AP_DEAUTHENTICATE;
		break;
	case STATUS_AP_DISASSOCIATE:
		eventReason.eDisAssocType = OS_DISASSOC_STATUS_AP_DISASSOCIATE;
		break;
	case STATUS_ROAMING_TRIGGER:
		eventReason.eDisAssocType = OS_DISASSOC_STATUS_ROAMING_TRIGGER;
		break;
	default:
		eventReason.eDisAssocType = OS_DISASSOC_STATUS_UNSPECIFIED; 
	    break;
	}
	/* Use the same status code for OS */
	eventReason.uStatusCode		 = pSmeSm->DisAssoc.uStatusCode;

	EvHandlerSendEvent(pSmeSm->hEvHandler, IPC_EVENT_DISASSOCIATED, (UINT8*)&eventReason, sizeof(OS_802_11_DISASSOCIATE_REASON_T));

	/* Reset status after sending the event */
	pSmeSm->DisAssoc.mgmtStatus = STATUS_UNSPECIFIED;
	pSmeSm->DisAssoc.uStatusCode = 0;
}

#ifdef TI_DBG
/***********************************************************************
 *                        smeSm_resetStats									
 ***********************************************************************
DESCRIPTION: Reset SME statistics
                                                                                                   
INPUT:      hSmeSm	-	SME SM handle.

OUTPUT:		

RETURN:     

************************************************************************/
void smeSm_resetStats( TI_HANDLE hSmeSm )
{
	smeSm_t* pSmeSm = (smeSm_t*)hSmeSm;

	os_memoryZero( pSmeSm->hOs, &(pSmeSm->smeStats), sizeof(smeSmStats_t) );
}

/***********************************************************************
 *                        smeSm_printStats									
 ***********************************************************************
DESCRIPTION: Print SME statistics
                                                                                                   
INPUT:      hSmeSm	-	SME SM handle.

OUTPUT:		

RETURN:     

************************************************************************/
void smeSm_printStats( TI_HANDLE hSmeSm )
{
	smeSm_t* pSmeSm = (smeSm_t*)hSmeSm;

	WLAN_OS_REPORT(("----------------------- SME statistics -----------------------\n\n"));
	WLAN_OS_REPORT(("Scan attempts histogram:\n"));
	WLAN_OS_REPORT(("------------------------\n\n"));
	WLAN_OS_REPORT(("Attempts: %6d %6d %6d %6d %6d %6d %6d %6d\n", 1, 2, 3, 4, 5, 6, 7, 8));
	WLAN_OS_REPORT(("Count:    %6d %6d %6d %6d %6d %6d %6d %6d\n\n", pSmeSm->smeStats.scanAttemptsForConnectionHistogram[ 0 ],
					pSmeSm->smeStats.scanAttemptsForConnectionHistogram[ 1 ], pSmeSm->smeStats.scanAttemptsForConnectionHistogram[ 2 ],
					pSmeSm->smeStats.scanAttemptsForConnectionHistogram[ 3 ], pSmeSm->smeStats.scanAttemptsForConnectionHistogram[ 4 ],
					pSmeSm->smeStats.scanAttemptsForConnectionHistogram[ 5 ], pSmeSm->smeStats.scanAttemptsForConnectionHistogram[ 6 ],
					pSmeSm->smeStats.scanAttemptsForConnectionHistogram[ 7 ]));
	WLAN_OS_REPORT(("Scan result count histogram:\n"));
	WLAN_OS_REPORT(("----------------------------\n\n"));
	WLAN_OS_REPORT(("Results: %6d %6d %6d %6d %6d %6d %6d %6d\n", 0, 1, 2, 3, 4, 5, 6, 7));
	WLAN_OS_REPORT(("Scans:   %6d %6d %6d %6d %6d %6d %6d %6d\n\n", pSmeSm->smeStats.scanResulCountHistogram[ 0 ],
					pSmeSm->smeStats.scanResulCountHistogram[ 1 ], pSmeSm->smeStats.scanResulCountHistogram[ 2 ],
					pSmeSm->smeStats.scanResulCountHistogram[ 3 ], pSmeSm->smeStats.scanResulCountHistogram[ 4 ],
					pSmeSm->smeStats.scanResulCountHistogram[ 5 ], pSmeSm->smeStats.scanResulCountHistogram[ 6 ],
					pSmeSm->smeStats.scanResulCountHistogram[ 7 ]));
	WLAN_OS_REPORT(("Results: %6d %6d %6d %6d %6d %6d %6d %6d\n", 8, 9, 10, 11, 12, 13, 14, 15));
	WLAN_OS_REPORT(("Scans:   %6d %6d %6d %6d %6d %6d %6d %6d\n", pSmeSm->smeStats.scanResulCountHistogram[ 8 ],
					pSmeSm->smeStats.scanResulCountHistogram[ 9 ], pSmeSm->smeStats.scanResulCountHistogram[ 10 ],
					pSmeSm->smeStats.scanResulCountHistogram[ 11 ], pSmeSm->smeStats.scanResulCountHistogram[ 12 ],
					pSmeSm->smeStats.scanResulCountHistogram[ 13 ], pSmeSm->smeStats.scanResulCountHistogram[ 14 ],
					pSmeSm->smeStats.scanResulCountHistogram[ 15 ]));
}

/***********************************************************************
 *                        smeSm_dbgPrintObject									
 ***********************************************************************
DESCRIPTION: Print the SME object
                                                                                                   
INPUT:      hSmeSm	-	SME SM handle.

OUTPUT:		

RETURN:     

************************************************************************/
void smeSm_dbgPrintObject( TI_HANDLE hSmeSm )
{
	smeSm_t *pSmeSm = (smeSm_t *)hSmeSm;

	WLAN_OS_REPORT(("Current SME State is <%s>\n",stateDesc[pSmeSm->state]));
}

#endif

