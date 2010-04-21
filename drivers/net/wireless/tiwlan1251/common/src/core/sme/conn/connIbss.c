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


/** \file connIbss.c
 *  \brief IBSS connection implementation
 *
 *  \see connIbss.h
 */

#include "report.h"
#include "osTIType.h"
#include "osApi.h"
#include "conn.h"
#include "connIbss.h"
#include "fsm.h"
#include "siteMgrApi.h"
#include "smeSmApi.h"
#include "rsnApi.h"
#include "DataCtrl_Api.h"  
#include "paramIn.h"
#include "paramOut.h"
#include "connApi.h"
#include "EvHandler.h"
#include "currBss.h"
#include "TrafficMonitorAPI.h"
#include "healthMonitor.h"

/* Local functions prototypes */
/* Local functions prototypes */
static TI_STATUS idle_to_selfWait(void *pData);

static TI_STATUS idle_to_rsnWait(void *pData);
    
static TI_STATUS selfWait_to_idle(void *pData);

static TI_STATUS selfWait_to_rsnWait(void *pData);

static TI_STATUS rsnWait_to_idle(void *pData);

static TI_STATUS rsnWait_to_connected(void *pData);

static TI_STATUS connected_to_idle(void *pData);

static TI_STATUS actionUnexpected(void *pData);

static TI_STATUS actionNop(void *pData);

/********************************************/
/*      Functions Implementations           */
/********************************************/

/***********************************************************************
 *                        conn_ibssConfig                                   
 ***********************************************************************
DESCRIPTION: IBSS Connection configuration function, called by the conection set param function
                in the selection phase. Configures the connection state machine to IBSS connection mode
                                                                                                   
INPUT:      hConn   -   Connection handle.

OUTPUT:     

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS conn_ibssConfig(conn_t *pConn)
{

    fsm_actionCell_t    smMatrix[CONN_IBSS_NUM_STATES][CONN_IBSS_NUM_EVENTS] =
    {

        /* next state and actions for IDLE state */
        {   {STATE_CONN_IBSS_SELF_WAIT, idle_to_selfWait    },  /* CONN_IBSS_CREATE */
            {STATE_CONN_IBSS_RSN_WAIT,  idle_to_rsnWait     },  /* CONN_IBSS_CONNECT    */
            {STATE_CONN_IBSS_IDLE,      actionNop           },  /* CONN_IBSS_DISCONNECT */
            {STATE_CONN_IBSS_IDLE,      actionUnexpected    },  /* CONN_IBSS_RSN_SUCC */
            {STATE_CONN_IBSS_IDLE,      actionUnexpected    }   /* CONN_IBSS_STA_JOINED */
        },

        /* next state and actions for SELF_WAIT state */
        {   {STATE_CONN_IBSS_SELF_WAIT, actionUnexpected    },  /* CONN_IBSS_CREATE */
            {STATE_CONN_IBSS_SELF_WAIT, actionUnexpected    },  /* CONN_IBSS_CONNECT    */
            {STATE_CONN_IBSS_IDLE,      selfWait_to_idle    },  /* CONN_IBSS_DISCONNECT */
            {STATE_CONN_IBSS_SELF_WAIT, actionUnexpected    },  /* CONN_IBSS_RSN_SUCC */
            {STATE_CONN_IBSS_RSN_WAIT,  selfWait_to_rsnWait }   /* CONN_IBSS_STA_JOINED */
        },

        /* next state and actions for RSN_WAIT state */
        {   {STATE_CONN_IBSS_RSN_WAIT,  actionUnexpected    },  /* CONN_IBSS_CREATE */
            {STATE_CONN_IBSS_RSN_WAIT,  actionUnexpected    },  /* CONN_IBSS_CONNECT    */
            {STATE_CONN_IBSS_IDLE,      rsnWait_to_idle     },  /* CONN_IBSS_DISCONNECT */
            {STATE_CONN_IBSS_CONNECTED, rsnWait_to_connected},  /* CONN_IBSS_RSN_SUCC */
            {STATE_CONN_IBSS_RSN_WAIT,  actionNop           }   /* CONN_IBSS_STA_JOINED */
        },

        /* next state and actions for CONNECTED state */
        {   {STATE_CONN_IBSS_CONNECTED, actionUnexpected    },  /* CONN_IBSS_CREATE */
            {STATE_CONN_IBSS_CONNECTED, actionUnexpected    },  /* CONN_IBSS_CONNECT    */
            {STATE_CONN_IBSS_IDLE,      connected_to_idle   },  /* CONN_IBSS_DISCONNECT */
            {STATE_CONN_IBSS_CONNECTED, actionUnexpected    },  /* CONN_IBSS_RSN_SUCC */
            {STATE_CONN_IBSS_RSN_WAIT,  actionNop           }   /* CONN_IBSS_STA_JOINED */
        }
        
    };

    return fsm_Config(pConn->ibss_pFsm, (fsm_Matrix_t)smMatrix, CONN_IBSS_NUM_STATES, CONN_IBSS_NUM_EVENTS, conn_ibssSMEvent, pConn->hOs);
}


/***********************************************************************
 *                        conn_ibssSMEvent                                  
 ***********************************************************************
DESCRIPTION: IBSS Connection SM event processing function, called by the connection API
                Perform the following:
                -   Print the state movement as a result from the event
                -   Calls the generic state machine event processing function which preform the following:
                    -   Calls the correspoding callback function
                    -   Move to next state
                
INPUT:      currentState    -   Pointer to the connection current state.
            event   -   Received event
            pConn   -   Connection handle

OUTPUT:     

RETURN:     OK on success, NOK otherwise

************************************************************************/

#ifdef REPORT_LOG

static char *stateDesc[CONN_IBSS_NUM_STATES] = 
    {
        "STATE_CONN_IBSS_IDLE",
        "STATE_CONN_IBSS_SELF_WAIT",
        "STATE_CONN_IBSS_RSN_WAIT",
        "STATE_CONN_IBSS_CONNECTED"
    };

static char *eventDesc[CONN_IBSS_NUM_EVENTS] = 
    {
        "CONN_IBSS_START",
        "CONN_IBSS_CONNECT",
        "CONN_IBSS_DISCONNECT",
        "CONN_IBSS_RSN_SUCC",
        "CONN_IBSS_STA_JOINED"
    };

#endif


TI_STATUS conn_ibssSMEvent(UINT8 *currentState, UINT8 event, TI_HANDLE hConn)
{
   conn_t *pConn = (conn_t *)hConn;
    TI_STATUS       status;
    UINT8       nextState;

    status = fsm_GetNextState(pConn->ibss_pFsm, *currentState, event, &nextState);
    if (status != OK)
    {
        WLAN_REPORT_SM(pConn->hReport, CONN_MODULE_LOG, ("IBSS State machine error, failed getting next state\n"));
        return(NOK);
    }

    WLAN_REPORT_SM(pConn->hReport, CONN_MODULE_LOG, 
                              ("IBSS: <%s, %s> --> %s\n\n",
                               stateDesc[*currentState],
                               eventDesc[event],
                               stateDesc[nextState]));

    status = fsm_Event(pConn->ibss_pFsm, currentState, event, (void *)pConn);

    return status;
}


/************************************************************************************************************/
/*      In the following section are listed the callback function used by the IBSS connection state machine */
/************************************************************************************************************/



/***********************************************************************
 *                        idle_to_selfWait
 ***********************************************************************
DESCRIPTION: 


INPUT:   

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
static TI_STATUS idle_to_selfWait(void *pData)
{
    UINT16      randomTime;

    /*Request from ELP to stay awake*/
    MacServices_powerAutho_AwakeRequiredUpdate(((conn_t *)pData)->hMacServices, POWERAUTHO_AWAKE_REQUIRED, POWERAUTHO_AWAKE_REASON_IBSS);

    siteMgr_join(((conn_t *)pData)->hSiteMgr);

    txData_disableTransmission(((conn_t *)pData)->hTxData, NO_DISABLE);

    /* get a randomTime that is constructed of the lower 13 bits ot the system time to 
       get a MS random time of ~8000 ms */
    randomTime = os_timeStampMs(((conn_t *)pData)->hOs) & 0x1FFF;

    /* Update current BSS connection type and mode */
    currBSS_updateConnectedState(((conn_t *)pData)->hCurrBss, TRUE, BSS_INDEPENDENT);
    os_timerStart(((conn_t *)pData)->hOs, ((conn_t *)pData)->pTimer, (((conn_t *)pData)->timeout + randomTime), FALSE);

    return OK;
}


/***********************************************************************
 *                        idle_to_rsnWait
 ***********************************************************************
DESCRIPTION: 


INPUT:   

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
static TI_STATUS idle_to_rsnWait(void *pData)
{
    paramInfo_t param;

    /*Request from ELP to stay awake*/
    MacServices_powerAutho_AwakeRequiredUpdate(((conn_t *)pData)->hMacServices, POWERAUTHO_AWAKE_REQUIRED, POWERAUTHO_AWAKE_REASON_IBSS);
    
    siteMgr_join(((conn_t *)pData)->hSiteMgr);

    txData_disableTransmission(((conn_t *)pData)->hTxData, NO_DISABLE);

    param.paramType = RX_DATA_PORT_STATUS_PARAM;
    param.content.rxDataPortStatus = OPEN_EAPOL;
    rxData_setParam(((conn_t *)pData)->hRxData, &param);

    param.paramType = TX_DATA_PORT_STATUS_PARAM;
    param.content.txDataPortStatus = OPEN_EAPOL;
    txData_setParam(((conn_t *)pData)->hTxData, &param);
    
    /*
     *  Notify that the driver is associated to the supplicant\IP stack. 
     */
    EvHandlerSendEvent(((conn_t *)pData)->hEvHandler, IPC_EVENT_ASSOCIATED, NULL,0);

    /* Update current BSS connection type and mode */
    currBSS_updateConnectedState(((conn_t *)pData)->hCurrBss, TRUE, BSS_INDEPENDENT);
    
    return rsn_start(((conn_t *)pData)->hRsn);
}

/***********************************************************************
 *                        selfWait_to_idle
 ***********************************************************************
DESCRIPTION: 


INPUT:   

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
static TI_STATUS selfWait_to_idle(void *pData)
{
    paramInfo_t     param;  
    os_timerStop(((conn_t *)pData)->hOs, ((conn_t *)pData)->pTimer);

    
    siteMgr_removeSelfSite(((conn_t *)pData)->hSiteMgr);

    /* Update current BSS connection type and mode */
    currBSS_updateConnectedState(((conn_t *)pData)->hCurrBss, FALSE, BSS_INDEPENDENT);
    
    /* Release ELP  */
    MacServices_powerAutho_AwakeRequiredUpdate(((conn_t *)pData)->hMacServices, POWERAUTHO_AWAKE_NOT_REQUIRED, POWERAUTHO_AWAKE_REASON_IBSS);

    /* stop beacon generation  */
    param.paramType = RX_DATA_PORT_STATUS_PARAM;
    param.content.rxDataPortStatus = CLOSE;
    rxData_setParam(((conn_t *)pData)->hRxData, &param);

    param.paramType = TX_DATA_PORT_STATUS_PARAM;
    param.content.txDataPortStatus = CLOSE;
    txData_setParam(((conn_t *)pData)->hTxData, &param);
    
    whalCtrl_FwDisconnect(((conn_t *)pData)->hHalCtrl, 
                          RX_CONFIG_OPTION_MY_DST_MY_BSS, 
                          RX_FILTER_OPTION_FILTER_ALL);

    /* Inform the SME about the connection lost */
    smeSm_reportConnStatus(((conn_t *)pData)->hSmeSm, STATUS_UNSPECIFIED, 0);

    return OK;
}



/***********************************************************************
 *                        selfWait_to_rsnWait
 ***********************************************************************
DESCRIPTION: 


INPUT:   

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
static TI_STATUS selfWait_to_rsnWait(void *pData)
{
    paramInfo_t param;

    os_timerStop(((conn_t *)pData)->hOs, ((conn_t *)pData)->pTimer);

    param.paramType = RX_DATA_PORT_STATUS_PARAM;
    param.content.rxDataPortStatus = OPEN_EAPOL;
    rxData_setParam(((conn_t *)pData)->hRxData, &param);

    param.paramType = TX_DATA_PORT_STATUS_PARAM;
    param.content.txDataPortStatus = OPEN_EAPOL;
    txData_setParam(((conn_t *)pData)->hTxData, &param);

    /*
     *  Notify that the driver is associated to the supplicant\IP stack. 
     */
    EvHandlerSendEvent(((conn_t *)pData)->hEvHandler, IPC_EVENT_ASSOCIATED, NULL,0);

    return rsn_start(((conn_t *)pData)->hRsn);
}


/***********************************************************************
 *                        rsnWait_to_idle
 ***********************************************************************
DESCRIPTION: 


INPUT:   

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
static TI_STATUS rsnWait_to_idle(void *pData)
{
    paramInfo_t     param;  

    rsn_stop(((conn_t *)pData)->hRsn, FALSE);

    param.paramType = RX_DATA_PORT_STATUS_PARAM;
    param.content.rxDataPortStatus = CLOSE;
    rxData_setParam(((conn_t *)pData)->hRxData, &param);

    param.paramType = TX_DATA_PORT_STATUS_PARAM;
    param.content.txDataPortStatus = CLOSE;
    txData_setParam(((conn_t *)pData)->hTxData, &param);

    /* Update current BSS connection type and mode */
    currBSS_updateConnectedState(((conn_t *)pData)->hCurrBss, FALSE, BSS_INDEPENDENT);

    
    /* Release ELP  */
    MacServices_powerAutho_AwakeRequiredUpdate(((conn_t *)pData)->hMacServices, POWERAUTHO_AWAKE_NOT_REQUIRED, POWERAUTHO_AWAKE_REASON_IBSS);

    /* stop beacon generation */
    whalCtrl_FwDisconnect(((conn_t *)pData)->hHalCtrl, 
                          RX_CONFIG_OPTION_MY_DST_MY_BSS, 
                          RX_FILTER_OPTION_FILTER_ALL);



    smeSm_reportConnStatus(((conn_t *)pData)->hSmeSm, STATUS_UNSPECIFIED, 0);

    return OK;
}

/***********************************************************************
 *                        rsnWait_to_connected
 ***********************************************************************
DESCRIPTION: 


INPUT:   

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
static TI_STATUS rsnWait_to_connected(void *pData)
{
    paramInfo_t param;

    conn_t *pConn=(conn_t *)pData;

    ctrlData_start(pConn->hCtrlData);

    TrafficMonitor_Start( pConn->hTrafficMonitor );

    healthMonitor_setState(pConn->hHealthMonitor, HEALTH_MONITOR_STATE_CONNECTED);

    siteMgr_start(pConn->hSiteMgr);

    param.paramType = RX_DATA_PORT_STATUS_PARAM;
    param.content.rxDataPortStatus = OPEN;
    rxData_setParam(((conn_t *)pData)->hRxData, &param);

    param.paramType = TX_DATA_PORT_STATUS_PARAM;
    param.content.txDataPortStatus = OPEN;
    txData_setParam(((conn_t *)pData)->hTxData, &param);

    
    /* Update current BSS connection type and mode */
    currBSS_updateConnectedState(pConn->hCurrBss, TRUE, BSS_INDEPENDENT);

    return smeSm_reportConnStatus(((conn_t *)pData)->hSmeSm, STATUS_SUCCESSFUL, 0);
}

/***********************************************************************
 *                        connected_to_idle
 ***********************************************************************
DESCRIPTION: 


INPUT:   

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
static TI_STATUS connected_to_idle(void *pData)
{
    conn_t *pConn=(conn_t *)pData;

    TrafficMonitor_Stop(pConn->hTrafficMonitor);

    healthMonitor_setState(pConn->hHealthMonitor, HEALTH_MONITOR_STATE_DISCONNECTED);

    /* The logic of this action is identical to rsnWait_to_idle */
    return rsnWait_to_idle(pConn);
}



/***********************************************************************
 *                        actionUnexpected
 ***********************************************************************
DESCRIPTION: 


INPUT:   

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
static TI_STATUS actionUnexpected(void *pData) 
{
    conn_t *pConn = (conn_t *)pData; 
    
    WLAN_REPORT_SM(pConn->hReport, CONN_MODULE_LOG,  ("State machine error, unexpected Event\n\n"));
    return OK;
}

/***********************************************************************
 *                        actionNop
 ***********************************************************************
DESCRIPTION: 


INPUT:   

OUTPUT:

RETURN:     OK on success, NOK otherwise

************************************************************************/
static TI_STATUS actionNop(void *pData) 
{
    return OK;
}
