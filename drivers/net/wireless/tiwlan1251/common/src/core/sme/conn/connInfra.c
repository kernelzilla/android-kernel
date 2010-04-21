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


/** \file connInfra.c
 *  \brief Infra connection implementation
 *
 *  \see connInfra.h
 */

#include "report.h"
#include "osTIType.h"
#include "osApi.h"
#include "conn.h"
#include "connInfra.h"
#include "fsm.h"
#include "siteMgrApi.h"
#include "smeSmApi.h"
#include "rsnApi.h"
#include "DataCtrl_Api.h"  
#include "paramIn.h"
#include "paramOut.h"
#include "siteHash.h"
#include "smeSm.h"
#include "PowerMgr_API.h"
#include "measurementMgrApi.h"
#include "TrafficMonitorAPI.h"
#include "qosMngr_API.h"
#include "EvHandler.h"
#include "SwitchChannelApi.h"
#include "ScanCncnApi.h"
#include "currBss.h"
#include "EvHandler.h"
#include "healthMonitor.h"
#include "regulatoryDomainApi.h"
#include "SoftGeminiApi.h"

#ifdef EXC_MODULE_INCLUDED
#include "excMngr.h"
#include "excTSMngr.h"
#endif

#define DISCONNECT_TIMEOUT      800

/* Local functions prototypes */

static TI_STATUS actionUnexpected(void *pData);

static TI_STATUS actionNop(void *pData);

static TI_STATUS Idle_to_ScrWait(void *pData);

static TI_STATUS Idle_to_Idle(void *pData);

static TI_STATUS ScrWait_to_idle(void *pData);

static TI_STATUS ScrWait_to_JoinWait(void *pData);

static TI_STATUS JoinWait_to_mlmeWait(void *pData);

static TI_STATUS JoinWait_to_Idle(void *pData);

static TI_STATUS mlmeWait_to_Idle(void *pData);

static TI_STATUS mlmeWait_to_rsnWait(void *pData);

static TI_STATUS rsnWait_to_disassociate(void *pData);

static TI_STATUS rsnWait_to_configHW(void *pData);

static TI_STATUS configHW_to_connected(void *pData);

static TI_STATUS configHW_to_disassociate(void *pData);

static TI_STATUS connect_to_disassociate(void *pData);

static TI_STATUS connect_to_ScrWait(void *pData);

static TI_STATUS disAssocc_to_idle(void *pData);



static TI_STATUS stopModules( conn_t *pConn );

void InfraConnSM_ScrCB( TI_HANDLE hConn, scr_clientRequestStatus_e requestStatus,
                        scr_pendReason_e pendReason );

int conn_MboxFlushFinishCb(TI_HANDLE pData,UINT16 MboxStatus, char *InterrogateParamsBuf);

/********************************************/
/*      Functions Implementations           */
/********************************************/


/***********************************************************************
 *                        conn_infraConfig                                  
 ***********************************************************************
DESCRIPTION: Infra Connection configuration function, called by the conection set param function
                in the selection phase. Configures the connection state machine to Infra connection mode
                                                                                                   
INPUT:      hConn   -   Connection handle.

OUTPUT:     

RETURN:     OK on success, NOK otherwise

************************************************************************/
TI_STATUS conn_infraConfig(conn_t *pConn)
{
    fsm_actionCell_t    smMatrix[CONN_INFRA_NUM_STATES][CONN_INFRA_NUM_EVENTS] =
    {
        /* next state and actions for IDLE state */
        {   {STATE_CONN_INFRA_SCR_WAIT, Idle_to_ScrWait},               /* "EVENT_CONNECT"  */
            {STATE_CONN_INFRA_IDLE, actionNop       },                  /* "EVENT_SCR_SUCC"*/
            {STATE_CONN_INFRA_IDLE, actionNop       },                  /* "EVENT_JOIN_CMD_CMPLT */
            {STATE_CONN_INFRA_IDLE, Idle_to_Idle    },                  /* "EVENT_DISCONNECT"       */
            {STATE_CONN_INFRA_IDLE, actionUnexpected},                  /* "EVENT_MLME_SUCC"*/
            {STATE_CONN_INFRA_IDLE, actionUnexpected},                  /* "EVENT_RSN_SUCC" */
            {STATE_CONN_INFRA_IDLE, actionUnexpected},                  /* "EVENT_CONFIG_HW" */
            {STATE_CONN_INFRA_IDLE, actionUnexpected}                   /* "EVENT_DISASSOC_FRAME_SENT" */
        },

        /* next state and actions for SCR_WAIT state */
        {   {STATE_CONN_INFRA_SCR_WAIT, actionUnexpected},              /* "EVENT_CONNECT"  */
            {STATE_CONN_INFRA_WAIT_JOIN_CMPLT, ScrWait_to_JoinWait},    /* "EVENT_SCR_SUCC"*/
            {STATE_CONN_INFRA_SCR_WAIT, actionUnexpected},              /* "EVENT_JOIN_CMD_CMPLT */
            {STATE_CONN_INFRA_IDLE,     ScrWait_to_idle},               /* "EVENT_DISCONNECT"       */
            {STATE_CONN_INFRA_SCR_WAIT, actionUnexpected},              /* "EVENT_MLME_SUCC"*/
            {STATE_CONN_INFRA_SCR_WAIT, actionUnexpected},              /* "EVENT_RSN_SUCC" */
            {STATE_CONN_INFRA_SCR_WAIT, actionUnexpected},              /* "EVENT_CONFIG_HW "*/
            {STATE_CONN_INFRA_SCR_WAIT, actionNop}                      /* "EVENT_DISASSOC_FRAME_SENT" */
        },

        /* next state and actions for WAIT_JOIN_CMPLT */
        
        {   {STATE_CONN_INFRA_WAIT_JOIN_CMPLT, actionUnexpected},       /* "EVENT_CONNECT"    */
            {STATE_CONN_INFRA_WAIT_JOIN_CMPLT, actionUnexpected},       /* "EVENT_SCR_SUCC"*/
            {STATE_CONN_INFRA_MLME_WAIT,       JoinWait_to_mlmeWait},   /* "EVENT_JOIN_CMD_CMPLT"   */
            {STATE_CONN_INFRA_IDLE,            JoinWait_to_Idle},       /* "EVENT_DISCONNECT"       */
            {STATE_CONN_INFRA_WAIT_JOIN_CMPLT, actionUnexpected},       /* "EVENT_MLME_SUCC"*/
            {STATE_CONN_INFRA_WAIT_JOIN_CMPLT, actionUnexpected},       /* "EVENT_RSN_SUCC" */
            {STATE_CONN_INFRA_WAIT_JOIN_CMPLT, actionUnexpected},       /* "EVENT_CONFIG_HW"        */
            {STATE_CONN_INFRA_WAIT_JOIN_CMPLT, actionUnexpected}        /* "EVENT_DISASSOC_FRAME_SENT" */
        
        },

        /* next state and actions for MLME_WAIT state */
        {   {STATE_CONN_INFRA_MLME_WAIT, actionUnexpected},             /* "EVENT_CONNECT"  */
            {STATE_CONN_INFRA_MLME_WAIT, actionUnexpected},             /* "EVENT_SCR_SUCC" */
            {STATE_CONN_INFRA_MLME_WAIT, actionUnexpected},             /* "EVENT_JOIN_CMD_CMPLT"*/
            {STATE_CONN_INFRA_IDLE,      mlmeWait_to_Idle},             /* "EVENT_DISCONNECT"       */
            {STATE_CONN_INFRA_RSN_WAIT,  mlmeWait_to_rsnWait},          /* "EVENT_MLME_SUCC"*/
            {STATE_CONN_INFRA_MLME_WAIT, actionUnexpected},             /* "EVENT_RSN_SUCC" */
            {STATE_CONN_INFRA_MLME_WAIT, actionUnexpected},             /* "EVENT_CONFIG_HW" */
            {STATE_CONN_INFRA_MLME_WAIT, actionUnexpected}              /* "EVENT_DISASSOC_FRAME_SENT" */   
        },
        
        /* next state and actions for RSN_WAIT state */
        {   {STATE_CONN_INFRA_RSN_WAIT,     actionUnexpected},          /* "EVENT_CONNECT"  */
            {STATE_CONN_INFRA_RSN_WAIT,     actionUnexpected},          /* "EVENT_SCR_SUCC" */
            {STATE_CONN_INFRA_RSN_WAIT,     actionUnexpected},          /* "EVENT_JOIN_CMD_CMPLT"*/
            {STATE_CONN_INFRA_DISASSOCC,    rsnWait_to_disassociate},   /* "EVENT_DISCONNECT" */
            {STATE_CONN_INFRA_RSN_WAIT,     actionUnexpected},          /* "EVENT_MLME_SUCC"*/
            {STATE_CONN_INFRA_CONFIG_HW,    rsnWait_to_configHW},       /* "EVENT_RSN_SUCC" */
            {STATE_CONN_INFRA_RSN_WAIT,     actionUnexpected},          /* "EVENT_CONFIG_HW"        */
            {STATE_CONN_INFRA_RSN_WAIT,     actionUnexpected}           /* "EVENT_DISASSOC_FRAME_SENT" */
        },
        
        /* next state and actions for CONFIG_HW state */
        {   {STATE_CONN_INFRA_CONFIG_HW, actionUnexpected},             /* "EVENT_CONNECT"  */
            {STATE_CONN_INFRA_CONFIG_HW, actionUnexpected},             /* "EVENT_SCR_SUCC" */
            {STATE_CONN_INFRA_CONFIG_HW, actionUnexpected},             /* "EVENT_JOIN_CMD_CMPLT"*/
            {STATE_CONN_INFRA_DISASSOCC, configHW_to_disassociate},     /* "EVENT_DISCONNECT"       */
            {STATE_CONN_INFRA_CONFIG_HW, actionUnexpected},             /* "EVENT_MLME_SUCC"*/
            {STATE_CONN_INFRA_CONFIG_HW, actionUnexpected},             /* "EVENT_RSN_SUCC" */
            {STATE_CONN_INFRA_CONNECTED, configHW_to_connected},        /* "EVENT_CONFIG_HW"        */
            {STATE_CONN_INFRA_CONFIG_HW, actionUnexpected}              /* "EVENT_DISASSOC_FRAME_SENT" */   
        },

        /* next state and actions for CONNECTED state */
        {   {STATE_CONN_INFRA_SCR_WAIT, connect_to_ScrWait},        /* "EVENT_CONNECT"  */
            {STATE_CONN_INFRA_CONNECTED, actionUnexpected},         /* "EVENT_SCR_SUCC"*/
            {STATE_CONN_INFRA_CONNECTED, actionUnexpected},         /* "EVENT_JOIN_CMD_CMPLT" */
            {STATE_CONN_INFRA_DISASSOCC, connect_to_disassociate},  /* "EVENT_DISCONNECT"       */
            {STATE_CONN_INFRA_CONNECTED, actionUnexpected},         /* "EVENT_MLME_SUCC"*/
            {STATE_CONN_INFRA_CONNECTED, actionUnexpected},         /* "EVENT_RSN_SUCC" */
            {STATE_CONN_INFRA_CONNECTED, actionUnexpected},         /* "STATE_CONN_INFRA_CONFIG_HW" */
            {STATE_CONN_INFRA_CONNECTED, actionUnexpected}          /* "EVENT_DISASSOC_FRAME_SENT" */   
        },
        
            /* next state and actions for STATE_CONN_INFRA_DISASSOCC state */
        {   {STATE_CONN_INFRA_DISASSOCC, actionUnexpected},         /* "EVENT_CONNECT"  */
            {STATE_CONN_INFRA_DISASSOCC, actionUnexpected},         /* "STATE_CONN_INFRA_SCR_WAIT"*/
            {STATE_CONN_INFRA_DISASSOCC, actionUnexpected},         /* "EVENT_JOIN_CMD_CMPLT" */
            {STATE_CONN_INFRA_DISASSOCC, actionUnexpected},         /* "EVENT_DISCONNECT" */
            {STATE_CONN_INFRA_DISASSOCC, actionUnexpected},         /* "EVENT_MLME_SUCC"*/
            {STATE_CONN_INFRA_DISASSOCC, actionUnexpected},         /* "EVENT_RSN_SUCC" */
            {STATE_CONN_INFRA_DISASSOCC, actionNop       },         /* "STATE_CONN_INFRA_CONFIG_HW"  */
            {STATE_CONN_INFRA_IDLE,      disAssocc_to_idle}         /* "EVENT_DISASSOC_FRAME_SENT" */       
        }
        
    };

    scr_registerClientCB( pConn->hScr, SCR_CID_CONNECT, InfraConnSM_ScrCB, pConn );

    return fsm_Config(pConn->infra_pFsm, (fsm_Matrix_t)smMatrix, CONN_INFRA_NUM_STATES, CONN_INFRA_NUM_EVENTS, conn_infraSMEvent, pConn->hOs);
}

/***********************************************************************
 *                        conn_infraSMEvent                                 
 ***********************************************************************
DESCRIPTION: Infra Connection SM event processing function, called by the connection API
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

static char *stateDesc[CONN_INFRA_NUM_STATES] = 
    {
        "STATE_INFRA_IDLE",
        "STATE_INFRA_SCR_WAIT",
        "STATE_INFRA_WAIT_JOIN_CMPLT",
        "STATE_INFRA_MLME_WAIT",
        "STATE_INFRA_RSN_WAIT",
        "STATE_INFRA_CONFIG_HW",
        "STATE_INFRA_CONNECTED",
        "STATE_INFRA_DISASSOCC",
    };


static char *eventDesc[CONN_INFRA_NUM_EVENTS] = 
    {
        "EVENT_INFRA_CONNECT",
        "EVENT_INFRA_SCR_SUCC",
        "EVENT_INFRA_JOIN_CMD_CMPLT",
        "EVENT_INFRA_DISCONNECT",
        "EVENT_INFRA_MLME_SUCC",
        "EVENT_INFRA_RSN_SUCC",
        "EVENT_INFRA_HW_CONFIGURED",
        "EVENT_INFRA_DISCONN_COMPLETE",
    };

#endif

TI_STATUS conn_infraSMEvent(UINT8 *currentState, UINT8 event, TI_HANDLE hConn)
{
   conn_t *pConn = (conn_t *)hConn;
    TI_STATUS       status;
    UINT8       nextState;

    status = fsm_GetNextState(pConn->infra_pFsm, *currentState, event, &nextState);
    if (status != OK)
    {
        WLAN_REPORT_SM(pConn->hReport, CONN_MODULE_LOG, ("State machine error, failed getting next state\n"));
        return(NOK);
    }

    WLAN_REPORT_SM(pConn->hReport, CONN_MODULE_LOG, 
                              ("INFRA: <%s, %s> --> %s\n\n",
                               stateDesc[*currentState],
                               eventDesc[event],
                               stateDesc[nextState]));

    status = fsm_Event(pConn->infra_pFsm, currentState, event, (void *)pConn);

    return status;
}

/************************************************************************************************************/
/*      In the following section are listed the callback function used by the Infra connection state machine    */
/************************************************************************************************************/

/* JOIN, SET_DATA_PORT_NOTIFY, START_MLME */
static TI_STATUS ScrWait_to_JoinWait(void *pData)
{
    TI_STATUS status;
    conn_t *pConn = (conn_t *)pData; 

    /* set Hw available for the duration of the connection */
    MacServices_powerAutho_AwakeRequiredUpdate(pConn->hMacServices, POWERAUTHO_AWAKE_REQUIRED, POWERAUTHO_AWAKE_REASON_CONNECTION);

    status = siteMgr_join(((conn_t *)pData)->hSiteMgr );
    /* If the Join command was failed we report the SME that connection failure so it could exit connecting state */
    if (status != OK)
    {
       WLAN_REPORT_ERROR(pConn->hReport, CONN_MODULE_LOG,  ("Join command has failed!\n"));       
    }
    return status;
}


static TI_STATUS JoinWait_to_mlmeWait(void *pData)
{
    TI_STATUS status;
    paramInfo_t param;
    conn_t *pConn = (conn_t *)pData;

    txData_start(pConn->hTxData);

    param.paramType = REGULATORY_DOMAIN_TX_POWER_AFTER_SELECTION_PARAM;
    regulatoryDomain_setParam(pConn->hRegulatoryDomain, &param);

    param.paramType = RX_DATA_PORT_STATUS_PARAM;
    param.content.rxDataPortStatus = OPEN_NOTIFY;
    status = rxData_setParam(pConn->hRxData, &param);
    if (status != OK)
        return status;

    param.paramType = TX_DATA_PORT_STATUS_PARAM;
    param.content.txDataPortStatus = OPEN_NOTIFY;
    status = txData_setParam(pConn->hTxData, &param);
    if (status != OK)
        return status;

    /* 
     * Set the reassociation flag in the association logic.
     */ 
    param.paramType = MLME_RE_ASSOC_PARAM;

    if( pConn->connType == CONN_TYPE_ROAM )
        param.content.mlmeReAssoc = TRUE;
    else 
        param.content.mlmeReAssoc = FALSE;

    status = mlme_setParam(pConn->hMlmeSm, &param);

    return mlme_start(pConn->hMlmeSm);
}


/* STOP_MLME, SET_DATA_PORT_CLOSE, DIS_JOIN */
static TI_STATUS mlmeWait_to_Idle(void *pData)
{
    TI_STATUS status;
    paramInfo_t param;
    conn_t *pConn = (conn_t *)pData; 

    status = mlme_stop( pConn->hMlmeSm, pConn->disConnType, pConn->disConnReasonToAP );
    if (status != OK)
        return status;

    param.paramType = RX_DATA_PORT_STATUS_PARAM;
    param.content.rxDataPortStatus = CLOSE;
    rxData_setParam(pConn->hRxData, &param);


    param.paramType = TX_DATA_PORT_STATUS_PARAM;
    param.content.txDataPortStatus = CLOSE;
    txData_setParam(pConn->hTxData, &param);

    /* set Hw not available now that the connection process failed */
    MacServices_powerAutho_AwakeRequiredUpdate(pConn->hMacServices, POWERAUTHO_AWAKE_NOT_REQUIRED, POWERAUTHO_AWAKE_REASON_CONNECTION);
    
    whalCtrl_FwDisconnect(pConn->hHalCtrl, 
                          RX_CONFIG_OPTION_MY_DST_MY_BSS, 
                          RX_FILTER_OPTION_FILTER_ALL );

#ifdef EXC_MODULE_INCLUDED
    excMngr_updateIappInformation(pConn->hExcMngr, EXC_DISASSOC);
#endif

    scr_clientComplete(pConn->hScr, SCR_CID_CONNECT );

    pConn->scrRequested = FALSE;

    /*
     * Call the connection lost callback set by the SME or AP_CONN.
     */
    pConn->pConnStatusCB( pConn->connStatCbObj, pConn->smContext.disAssocEventReason, pConn->smContext.disAssocEventStatusCode);

    return OK;
}

/* This function is called from the WAIT_FOR_JOIN_CB_CMPLT state (before mlme_start)
  - all we need to do is call siteMgr_disJoin */
static TI_STATUS JoinWait_to_Idle(void *pData)
{
    conn_t *pConn = (conn_t *)pData; 

    whalCtrl_FwDisconnect(pConn->hHalCtrl, 
                           RX_CONFIG_OPTION_MY_DST_MY_BSS,
                           RX_FILTER_OPTION_FILTER_ALL );
    
    /* set Hw not available now that the connection process failed */
    MacServices_powerAutho_AwakeRequiredUpdate(pConn->hMacServices, POWERAUTHO_AWAKE_NOT_REQUIRED, POWERAUTHO_AWAKE_REASON_CONNECTION);
        
    scr_clientComplete( pConn->hScr, SCR_CID_CONNECT );
    pConn->scrRequested = FALSE;

    /*
     * Call the connection lost callback set by the SME or AP_CONN.
     */
	pConn->pConnStatusCB( pConn->connStatCbObj, pConn->smContext.disAssocEventReason, pConn->smContext.disAssocEventStatusCode);

   return OK;
}

/* SET_DATA_PORT_EAPOL, START_RSN */
static TI_STATUS mlmeWait_to_rsnWait(void *pData)
{
    TI_STATUS status;
    paramInfo_t param;
    conn_t *pConn = (conn_t *)pData;

    param.paramType = RX_DATA_PORT_STATUS_PARAM;
    param.content.rxDataPortStatus = OPEN_EAPOL;
    status = rxData_setParam(pConn->hRxData, &param);
    if (status != OK)
        return status;

    param.paramType = TX_DATA_PORT_STATUS_PARAM;
    param.content.txDataPortStatus = OPEN_EAPOL;
    status = txData_setParam(pConn->hTxData, &param);
    if (status != OK)
        return status;

    /*
     *  Notify that the driver is associated to the supplicant\IP stack. 
     */
    EvHandlerSendEvent(pConn->hEvHandler, IPC_EVENT_ASSOCIATED, NULL,0);

    return rsn_start(pConn->hRsn);
}



/* STOP_RSN, SET_DATA_PORT_CLOSE, STOP_MLME, DIS_JOIN */
static TI_STATUS rsnWait_to_disassociate(void *pData)
{
    TI_STATUS status;
    paramInfo_t param;
    conn_t *pConn = (conn_t *)pData;

    status = rsn_stop(pConn->hRsn, pConn->disConEraseKeys);
    if (status != OK)
        return status;

    param.paramType = RX_DATA_PORT_STATUS_PARAM;
    param.content.rxDataPortStatus = CLOSE;
    status = rxData_setParam(pConn->hRxData, &param);
    if (status != OK)
        return status;

    param.paramType = TX_DATA_PORT_STATUS_PARAM;
    param.content.txDataPortStatus = CLOSE;
    status = txData_setParam(pConn->hTxData, &param);
    if (status != OK)
        return status;

    /* Start the disconnect complete time out timer. 
        This should be done BEFORE calling mlme_stop, which invokes Disconect Complete
        event, which stops the timer. */
    os_timerStart(pConn->hOs, pConn->pTimer, DISCONNECT_TIMEOUT, FALSE);

    status = mlme_stop( pConn->hMlmeSm, pConn->disConnType, pConn->disConnReasonToAP );

    if (status != OK)
        return status;

    return OK;
}


/* STOP_RSN, SET_DATA_PORT_CLOSE, STOP_MLME, DIS_JOIN */
static TI_STATUS configHW_to_disassociate(void *pData)
{
    TI_STATUS status;
    paramInfo_t param;
    conn_t *pConn = (conn_t *)pData;

    status = rsn_stop(pConn->hRsn, pConn->disConEraseKeys );
    if (status != OK)
        return status;

    param.paramType = RX_DATA_PORT_STATUS_PARAM;
    param.content.rxDataPortStatus = CLOSE;
    status = rxData_setParam(pConn->hRxData, &param);
    if (status != OK)
        return status;

    param.paramType = TX_DATA_PORT_STATUS_PARAM;
    param.content.txDataPortStatus = CLOSE;
    status = txData_setParam(pConn->hTxData, &param);
    if (status != OK)
        return status;

    /* Start the disconnect complete time out timer. 
        This should be done BEFORE calling mlme_stop, which invokes Disconect Complete
        event, which stops the timer. */
    os_timerStart(pConn->hOs, pConn->pTimer, DISCONNECT_TIMEOUT, FALSE);

    status = mlme_stop( pConn->hMlmeSm, pConn->disConnType, pConn->disConnReasonToAP );
    if (status != OK)
        return status;

    param.paramType = REGULATORY_DOMAIN_DISCONNECT_PARAM;
    regulatoryDomain_setParam(pConn->hRegulatoryDomain, &param);

    /* Must be called AFTER mlme_stop. since De-Auth packet should be sent with the
        supported rates, and stopModules clears all rates. */
    stopModules(pConn);

    return OK;
}

static TI_STATUS connect_to_disassociate(void *pData)
{
    TI_STATUS status;
    paramInfo_t param;
    conn_t *pConn = (conn_t *)pData;

    status = rsn_stop(pConn->hRsn, pConn->disConEraseKeys);
    if (status != OK)
        return status;

    param.paramType = RX_DATA_PORT_STATUS_PARAM;
    param.content.rxDataPortStatus = CLOSE;
    status = rxData_setParam(pConn->hRxData, &param);
    if (status != OK)
        return status;

    param.paramType = TX_DATA_PORT_STATUS_PARAM;
    param.content.txDataPortStatus = CLOSE;
    status = txData_setParam(pConn->hTxData, &param);
    if (status != OK)
        return status;

    param.paramType = REGULATORY_DOMAIN_DISCONNECT_PARAM;
    regulatoryDomain_setParam(pConn->hRegulatoryDomain, &param);

    /* Start the disconnect complete time out timer. 
        This should be done BEFORE calling mlme_stop, which invokes Disconect Complete
        event, which stops the timer. */
    os_timerStart(pConn->hOs, pConn->pTimer, DISCONNECT_TIMEOUT, FALSE);

    status = mlme_stop( pConn->hMlmeSm, pConn->disConnType, pConn->disConnReasonToAP );
    if (status != OK)
        return status;

    /* Must be called AFTER mlme_stop. since De-Auth packet should be sent with the
        supported rates, and stopModules clears all rates. */
    stopModules(pConn);

    return OK;

}


static TI_STATUS rsnWait_to_configHW(void *pData)
{
    conn_t *pConn=(conn_t *)pData;
    TI_STATUS status;
    static UINT8    buf[20]; /* for mailbox interrogate leave the "static" flag !!!*/
    paramInfo_t     param;

    /* Open the RX to DATA */
    param.paramType = RX_DATA_PORT_STATUS_PARAM;
    param.content.rxDataPortStatus = OPEN;
    status = rxData_setParam(pConn->hRxData, &param);
    if (status != OK)
        return status;

    status = ctrlData_start(pConn->hCtrlData);
    if (status != OK)
      {
         WLAN_REPORT_ERROR(pConn->hReport, CONN_MODULE_LOG, ("Infra Conn status=%d, have to return (%d)\n",status,__LINE__));
         return status;
      }

    status = qosMngr_connect(pConn->hQosMngr);
    if (status != OK)
    {
         WLAN_REPORT_ERROR(pConn->hReport, CONN_MODULE_LOG, ("Infra Conn status=%d, have to return (%d)\n",status,__LINE__));
         return status;
    }

    status = measurementMgr_connected(pConn->hMeasurementMgr);
    if (status != OK)
      {
         WLAN_REPORT_ERROR(pConn->hReport, CONN_MODULE_LOG, ("Infra Conn status=%d, have to return (%d)\n",status,__LINE__));
         return status;
      }

    status = TrafficMonitor_Start(pConn->hTrafficMonitor);
    if (status != OK)
      {
         WLAN_REPORT_ERROR(pConn->hReport, CONN_MODULE_LOG, ("Infra Conn status=%d, have to return (%d)\n",status,__LINE__));
         return status;
      }

    healthMonitor_setState(pConn->hHealthMonitor, HEALTH_MONITOR_STATE_CONNECTED);

    switchChannel_start(pConn->hSwitchChannel);

    scanConcentrator_switchToConnected( pConn->hScanCnc );

    PowerMgr_startPS(pConn->hPwrMngr);
    
    whalCtrl_InterrogateMbox(pConn->hHalCtrl, (void *)conn_MboxFlushFinishCb,pData,&buf[0]);

    return OK;
}

/* Interrogate command CB to indicates that the Mbox is flushed*/
int conn_MboxFlushFinishCb(TI_HANDLE pData,UINT16 MboxStatus, char *InterrogateParamsBuf)
{
    conn_t *pConn = (conn_t *)pData;
    return conn_infraSMEvent(&pConn->state, CONN_INFRA_HW_CONFIGURED, pConn);
}


static TI_STATUS configHW_to_connected(void *pData)
{
    TI_STATUS status;
    paramInfo_t param;
    conn_t *pConn=(conn_t *)pData;


    param.paramType = TX_DATA_PORT_STATUS_PARAM;
    param.content.txDataPortStatus = OPEN;
    status = txData_setParam(pConn->hTxData, &param);
    if (status != OK)
        return status;

#ifdef EXC_MODULE_INCLUDED
    excMngr_updateIappInformation(pConn->hExcMngr, EXC_ASSOC_OK);
#endif

    /* Start keep alive process */
    siteMgr_start(pConn->hSiteMgr);

    scr_clientComplete( pConn->hScr, SCR_CID_CONNECT );
    pConn->scrRequested = FALSE;

    /* Update current BSS connection type and mode */
    currBSS_updateConnectedState(pConn->hCurrBss, TRUE, BSS_INFRASTRUCTURE);

    pConn->pConnStatusCB( pConn->connStatCbObj, STATUS_SUCCESSFUL, 0);

    /* set Hw not available now that the connection process ended successfully */
    MacServices_powerAutho_AwakeRequiredUpdate(pConn->hMacServices, POWERAUTHO_AWAKE_NOT_REQUIRED, POWERAUTHO_AWAKE_REASON_CONNECTION);

	SoftGemini_SetPSmode(pConn->hSoftGemini);
    
	WLAN_OS_REPORT(("************ NEW CONNECTION ************\n"));
    siteMgr_printPrimarySiteDesc(pConn->hSiteMgr);

    return OK;
}


static TI_STATUS actionUnexpected(void *pData) 
{
    conn_t *pConn = (conn_t *)pData; 
    
    WLAN_REPORT_SM(pConn->hReport, CONN_MODULE_LOG,  ("State machine error, unexpected Event\n\n"));
    return OK;
}

static TI_STATUS actionNop(void *pData) 
{
    return OK;
}


static TI_STATUS Idle_to_ScrWait(void *pData)
{
    scr_clientRequestStatus_e scrReplyStatus;
    scr_pendReason_e scrPendReason;
    
    conn_t *pConn = (conn_t *)pData;

    WLAN_REPORT_INFORMATION( pConn->hReport, CONN_MODULE_LOG,
                             ("Infra Connnect SM: Requesting SCR.\n") );
 
    scrReplyStatus = scr_clientRequest( pConn->hScr, SCR_CID_CONNECT, &scrPendReason );

    pConn->scrRequested = TRUE;

    /* request the SCR as application (either BG or FG) client, and act according to return status */
    switch ( scrReplyStatus )
    {
    case SCR_CRS_PEND:
        /* send a pend event to the SM */
        WLAN_REPORT_INFORMATION( pConn->hReport, CONN_MODULE_LOG, 
                                 ("Infra Conn: SCR pending with pend reason: %d, stay in wait SCR state.\n",
                                  scrPendReason) );
        break;

    case SCR_CRS_RUN:
        /* send an SCR SUCCESS event to the SM */
        WLAN_REPORT_INFORMATION( pConn->hReport, CONN_MODULE_LOG, ("Infra Conn: SCR acquired.\n") );
        
        conn_infraSMEvent(&pConn->state, CONN_INFRA_SCR_SUCC, (TI_HANDLE) pConn);
        break;

    default:
        WLAN_REPORT_ERROR( pConn->hReport, CONN_MODULE_LOG,
                             ("Infra Conn: SCR returned unrecognized status: %d.\n", scrReplyStatus) );
        return NOK;
    }

    return OK;
}



void InfraConnSM_ScrCB( TI_HANDLE hConn, scr_clientRequestStatus_e requestStatus,
                        scr_pendReason_e pendReason )
{
    conn_t *pConn = (conn_t *)hConn;

    WLAN_REPORT_INFORMATION( pConn->hReport, CONN_MODULE_LOG,
                             ("InfraConnSM_ScrCB called by SCR. Status is: %d.\n", requestStatus) );
    
    /* act according to the request staus */
    switch ( requestStatus )
    {
    case SCR_CRS_RUN:
        /* send an SCR SUCCESS event to the SM */
        WLAN_REPORT_INFORMATION( pConn->hReport, CONN_MODULE_LOG, ("Infra Conn: SCR acquired.\n") );
        
        conn_infraSMEvent(&pConn->state, CONN_INFRA_SCR_SUCC, (TI_HANDLE) pConn);
        break;

    case SCR_CRS_FW_RESET:
        /* Ignore FW reset, the MLME SM will handle re-try of the conn */
        WLAN_REPORT_INFORMATION( pConn->hReport, CONN_MODULE_LOG, ("Infra Conn: Recovery occured.\n") );
        break;

    default:
        WLAN_REPORT_ERROR( pConn->hReport, CONN_MODULE_LOG,
                           ("Illegal SCR request status:%d, pend reason:%d.\n", 
                           requestStatus, pendReason) );
        break;
    }
    
    return;
}



static TI_STATUS ScrWait_to_idle(void *pData)
{
    conn_t *pConn = (conn_t *)pData;

    WLAN_REPORT_INFORMATION( pConn->hReport, CONN_MODULE_LOG,
                             ("Infra Connnect SM: Stop event while in SCR wait, moving to IDLE.\n") );

    scr_clientComplete( pConn->hScr, SCR_CID_CONNECT );
    pConn->scrRequested = FALSE;

    /*
     * Call the connection lost callback set by the SME or AP_CONN.
     */
	pConn->pConnStatusCB( pConn->connStatCbObj, pConn->smContext.disAssocEventReason, pConn->smContext.disAssocEventStatusCode);

    return OK;

}


static TI_STATUS stopModules( conn_t *pConn )
{
   
    measurementMgr_disconnected(pConn->hMeasurementMgr);

    rxData_stop(pConn->hRxData);

    ctrlData_stop(pConn->hCtrlData);
    
    TrafficMonitor_Stop(pConn->hTrafficMonitor);

    switchChannel_stop(pConn->hSwitchChannel);

    healthMonitor_setState(pConn->hHealthMonitor, HEALTH_MONITOR_STATE_DISCONNECTED);

    siteMgr_stop(pConn->hSiteMgr);

    /* stopping power save */
    PowerMgr_stopPS(pConn->hPwrMngr);

    scanConcentrator_switchToNotConnected( pConn->hScanCnc );

    /* Set Current BSS Module to stop triggerring roaming events */
    currBSS_updateConnectedState(pConn->hCurrBss, FALSE, BSS_INFRASTRUCTURE);

	SoftGemini_unSetPSmode(pConn->hSoftGemini);

    return OK;
}


static TI_STATUS disAssocc_to_idle(void *pData)
{
    conn_t *pConn = (conn_t *)pData;

    /* Stop the disconnect timeout timer. */
    os_timerStop(((conn_t *)pData)->hOs, ((conn_t *)pData)->pTimer);

    /* 
     * Tx Data Stop and QoS disconnect must be called only after the disconnect (dissasociate/deauthenticate)
     * has been sent, or else no TX complete is received!
     */
    txData_stop(pConn->hTxData);
    qosMngr_disconnect(pConn->hQosMngr);

#ifdef EXC_MODULE_INCLUDED
    measurementMgr_disableTsMetrics(pConn->hMeasurementMgr, MAX_NUM_OF_AC);
#endif

    /* set Hw not available now that the connection process failed */
    MacServices_powerAutho_AwakeRequiredUpdate(pConn->hMacServices, POWERAUTHO_AWAKE_NOT_REQUIRED, POWERAUTHO_AWAKE_REASON_CONNECTION);

    whalCtrl_FwDisconnect(pConn->hHalCtrl, 
                          RX_CONFIG_OPTION_MY_DST_MY_BSS, 
                          RX_FILTER_OPTION_FILTER_ALL);

#ifdef EXC_MODULE_INCLUDED
    excMngr_updateIappInformation(pConn->hExcMngr, EXC_DISASSOC);
#endif

    /*
     * Call the connection lost callback set by the SME or AP_CONN.
     */
	pConn->pConnStatusCB( pConn->connStatCbObj, pConn->smContext.disAssocEventReason, pConn->smContext.disAssocEventStatusCode);

    /*
     * In case of connection failuer we might get here without freeing the SCR.
     */
    if( pConn->scrRequested == TRUE ){
         scr_clientComplete( pConn->hScr, SCR_CID_CONNECT );
         pConn->scrRequested = FALSE;
    }


    return OK;

}



static TI_STATUS connect_to_ScrWait(void *pData)
{
    TI_STATUS status;
    paramInfo_t param;
    conn_t *pConn = (conn_t *)pData;
    /*
     * This function performs roaming by two steps:
     * First - close the current connection without notify the SME.
     * Second - start new connection in reassociation mode.
     */ 


    status = rsn_stop(pConn->hRsn, pConn->disConEraseKeys);
    if (status != OK)
        return status;

    param.paramType = RX_DATA_PORT_STATUS_PARAM;
    param.content.rxDataPortStatus = CLOSE;
    status = rxData_setParam(pConn->hRxData, &param);
    if (status != OK)
        return status;

    param.paramType = TX_DATA_PORT_STATUS_PARAM;
    param.content.txDataPortStatus = CLOSE;
    status = txData_setParam(pConn->hTxData, &param);
    if (status != OK)
        return status;

    status = mlme_stop(pConn->hMlmeSm, DISCONN_TYPE_IMMEDIATE, pConn->disConnReasonToAP);
    if (status != OK)
        return status;

    param.paramType = REGULATORY_DOMAIN_DISCONNECT_PARAM;
    regulatoryDomain_setParam(pConn->hRegulatoryDomain, &param);

#ifdef EXC_MODULE_INCLUDED
    excMngr_updateIappInformation(pConn->hExcMngr, EXC_DISASSOC);
#endif
    /* Must be called AFTER mlme_stop. since De-Auth packet should be sent with the
        supported rates, and stopModules clears all rates. */
    stopModules(pConn);

    /* 
     * Tx Data Stop and QoS disconnect must be called only after the disconnect (dissasociate/deauthenticate)
     * has been sent. In this case no deauthenticate frame is sent bu still we keep the 
     * order.
     */
    txData_stop(pConn->hTxData);
    qosMngr_disconnect(pConn->hQosMngr);

    /* 
     * Start new connection.
     */ 
    Idle_to_ScrWait(pConn);

    return OK;
}

static TI_STATUS Idle_to_Idle(void *pData)
{
    conn_t *pConn = (conn_t *)pData;

    /* 
     * In case we are in IDLE and getting DISCONNECT event, we need to inform
     * the SME\AP_connection that we are disconnected. 
     * Call the connection lost callback set by the SME or AP_CONN.
     */
	pConn->pConnStatusCB( pConn->connStatCbObj, pConn->smContext.disAssocEventReason, pConn->smContext.disAssocEventStatusCode);

    return OK;
}

/***********************************************************************
                connInfra_JoinCmpltNotification
 ***********************************************************************
DESCRIPTION: Call back upon receving Join Event Complete.

INPUT:      hSiteMgr    -   site mgr handle.

OUTPUT:

RETURN:     
************************************************************************/
TI_STATUS connInfra_JoinCmpltNotification(TI_HANDLE hconn)
{
    conn_t *pConn = (conn_t *)hconn;
    
    WLAN_REPORT_INFORMATION(pConn->hReport, SITE_MGR_MODULE_LOG,
                           ("siteMgr_JoinCmplt: has been called\n"));

   txData_disableTransmission(pConn->hTxData, NO_DISABLE);

   if (pConn->currentConnType == CONNECTION_INFRA ) {
       conn_infraSMEvent(&pConn->state, CONN_INFRA_JOIN_CMD_CMPLT, pConn);
   }

   return OK;
}
