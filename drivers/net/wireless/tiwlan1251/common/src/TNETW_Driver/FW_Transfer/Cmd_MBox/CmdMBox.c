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

/****************************************************************************
 *
 *   MODULE:  CmdMBox.c
 *   PURPOSE: Handle the wlan hardware command mailbox
 *   
 ****************************************************************************/
#include "osTIType.h"
#include "whalCommon.h"
#include "TNETWIF.h"
#include "whalHwDefs.h"
#include "FwEvent_api.h"
#include "CmdQueue_api.h"
#include "CmdMBox_api.h"
#include "CmdMBox.h"
#include "tnetwCommon.h"
#include "osApi.h"

/* Check if HostIfReg is On. This function is used on SDIO only. */
int CmdMbox_CheckAndAck(TI_HANDLE hTNETWIF, UINT32 Intr);

#define CMDMBOX_DEBUG_PRINT 0

#ifdef REPORT_LOG
#define CMDMBOX_CHECK_STATUS \
    switch (pCmdMBox->HW_CmdMBox.cmdStatus) \
    { \
	case CMD_STATUS_SUCCESS: \
		{ \
			WLAN_REPORT_INFORMATION(pCmdMBox->hReport, CMD_MBOX_MODULE_LOG, \
                              ("CMD_MBOX: CMD_STATUS_SUCCESS\n")); \
			break; \
		} \
	case CMD_STATUS_REJECT_MEAS_SG_ACTIVE: \
		{ \
			WLAN_REPORT_INFORMATION(pCmdMBox->hReport, CMD_MBOX_MODULE_LOG, \
                              ("CMD_MBOX: ERROR: CMD_STATUS_REJECT_MEAS_SG_ACTIVE received\n")); \
            break; \
		} \
	case CMD_MAILBOX_IDLE: \
	case CMD_STATUS_UNKNOWN_CMD: \
	case CMD_STATUS_UNKNOWN_IE: \
    case CMD_STATUS_RX_BUSY: \
    case CMD_STATUS_INVALID_PARAM: \
    case CMD_STATUS_TEMPLATE_TOO_LARGE: \
    case CMD_STATUS_OUT_OF_MEMORY: \
    case CMD_STATUS_STA_TABLE_FULL: \
	case CMD_STATUS_RADIO_ERROR: \
	case CMD_STATUS_WRONG_NESTING: \
		{ \
			/* print the error */ \
            WLAN_REPORT_ERROR(pCmdMBox->hReport, CMD_MBOX_MODULE_LOG, \
                              ("%s: ERROR: %s (%d), command: %s (%d), IE: %s (%d)\n", \
                               __FUNCTION__, CmdQueue_GetErrorString(pCmdMBox->HW_CmdMBox.cmdStatus), \
                               pCmdMBox->HW_CmdMBox.cmdStatus, \
                               CmdQueue_GetCmdString(pCmdMBox->HW_CmdMBox.cmdID), \
                               pCmdMBox->HW_CmdMBox.cmdID, \
                               CmdQueue_GetIEString(pCmdMBox->HW_CmdMBox.cmdID, *(UINT16*)&(pCmdMBox->HW_CmdMBox.parameters)), \
                               *(UINT16*)&(pCmdMBox->HW_CmdMBox.parameters))); \
			/* continue as if the command succedded */ \
			pCmdMBox->HW_CmdMBox.cmdStatus = (uint16)CMD_STATUS_SUCCESS; \
			break; \
		} \
    case CMD_STATUS_TIMEOUT: \
	case CMD_STATUS_FW_RESET: \
	default: \
        /* if the FW is not responding, start recovery */ \
        { \
            CmdQueue_Error(pCmdMBox->hCmdQueue); \
			break; \
        } \
	} /* end of switch */
#else
#define CMDMBOX_CHECK_STATUS \
    switch (pCmdMBox->HW_CmdMBox.cmdStatus) \
    { \
	case CMD_STATUS_SUCCESS: \
		{ \
			break; \
		} \
	case CMD_STATUS_REJECT_MEAS_SG_ACTIVE: \
		{ \
            break; \
		} \
	case CMD_MAILBOX_IDLE: \
	case CMD_STATUS_UNKNOWN_CMD: \
	case CMD_STATUS_UNKNOWN_IE: \
    case CMD_STATUS_RX_BUSY: \
    case CMD_STATUS_INVALID_PARAM: \
    case CMD_STATUS_TEMPLATE_TOO_LARGE: \
    case CMD_STATUS_OUT_OF_MEMORY: \
    case CMD_STATUS_STA_TABLE_FULL: \
	case CMD_STATUS_RADIO_ERROR: \
	case CMD_STATUS_WRONG_NESTING: \
		{ \
			/* continue as if the command succedded */ \
			pCmdMBox->HW_CmdMBox.cmdStatus = (uint16)CMD_STATUS_SUCCESS; \
			break; \
		} \
    case CMD_STATUS_TIMEOUT: \
	case CMD_STATUS_FW_RESET: \
	default: \
        /* if the FW is not responding, start recovery */ \
        { \
            CmdQueue_Error(pCmdMBox->hCmdQueue); \
			break; \
        } \
	} /* end of switch */
#endif /* REPORT_LOG */


#if CMDMBOX_DEBUG_PRINT
static char *StateString_array[16] = {
    "CMDMBOX_STATE_SENDCMD_NORMAL_IDLE",  /* 0 */
    "CMDMBOX_STATE_SENDCMD_NORMAL_WAIT_BUS",  /* 1 */
    "CMDMBOX_STATE_SENDCMD_NORMAL_WAIT_TXN_BUF",  /* 2 */
    "CMDMBOX_STATE_SENDCMD_NORMAL_WRITE_TRIG_v",  /* 3 */
    "CMDMBOX_STATE_SENDCMD_NORMAL_WAIT_TXN_TRIG",  /* 4 */
    "CMDMBOX_STATE_SENDCMD_BLOCKING_IDLE",  /* 5 */
    "CMDMBOX_STATE_SENDCMD_BLOCKING_WAIT_BUS",  /* 6 */
    "CMDMBOX_STATE_SENDCMD_BLOCKING_WAIT_TXN_BUF",  /* 7 */
    "CMDMBOX_STATE_SENDCMD_BLOCKING_WRITE_TRIG_v",  /* 8 */
    "CMDMBOX_STATE_SENDCMD_BLOCKING_WAIT_TXN_TRIG",  /* 9 */
    "CMDMBOX_STATE_SENDCMD_BLOCKING_POLL_CMPLT_v",  /* 10 */
    "CMDMBOX_STATE_SENDCMD_BLOCKING_FINISH_v",  /* 11 */
    "CMDMBOX_STATE_GETRESULT_NORMAL_IDLE",  /* 12 */
    "CMDMBOX_STATE_GETRESULT_NORMAL_WAIT_TXN",  /* 13 */
    "CMDMBOX_STATE_GETRESULT_BLOCKING_IDLE",  /* 14 */
    "CMDMBOX_STATE_GETRESULT_BLOCKING_WAIT_TXN",  /* 15 */  
};
static char *StateString[NUM_COMMANDS];

static char *EventString[CMDMBOX_EVENT_NUM] = {
    "CMDMBOX_EVENT_SEND_CMD",  /* 1 */
    "CMDMBOX_EVENT_CMD_CMPLT",  /* 2 */
    "CMDMBOX_EVENT_BUS_READY",  /* 3 */
    "CMDMBOX_EVENT_TXN_CMPLT",  /* 4 */
    "CMDMBOX_EVENT_GET_RESULT",  /* 5 */
};

#endif

/****************************************************************************
 *                      CmdMBox_Create()
 ****************************************************************************
 * DESCRIPTION: Create the mailbox object
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: The Created object
 ****************************************************************************/
TI_HANDLE       CmdMBox_Create(TI_HANDLE hOs)
{
    CmdMBox_T *pObj;

    pObj = os_memoryAlloc (hOs, sizeof(CmdMBox_T));
    if (pObj == NULL)
    {
        WLAN_OS_REPORT(("FATAL ERROR: CmdMBox_Create(): Error Creating CmdMBox - Aborting\n"));
        return NULL;
    }

    /* reset control module control block */
    os_memoryZero(hOs, pObj, sizeof(CmdMBox_T));
    pObj->hOs = hOs;

    /* allocates Timer to use for CmdMBox timeout*/
    pObj->hTimer = os_timerCreate(hOs, CmdMBox_TimeOut, pObj);
    if (pObj->hTimer == NULL)
    {
        CmdMBox_Destroy(pObj);
        WLAN_OS_REPORT(("FATAL ERROR: CmdMBox_Create(): Error Creating CmdMBox Timer- Aborting\n"));
        return NULL;
    }

#if CMDMBOX_DEBUG_PRINT
    StateString[0] = StateString_array[0];
    StateString[1] = StateString_array[1];
    StateString[2] = StateString_array[2];
    StateString[3] = StateString_array[3];
    StateString[4] = StateString_array[4];
    StateString[10] = StateString_array[5];
    StateString[11] = StateString_array[6];
    StateString[12] = StateString_array[7];
    StateString[13] = StateString_array[8];
    StateString[14] = StateString_array[9];
    StateString[15] = StateString_array[10];
    StateString[16] = StateString_array[11];
    StateString[20] = StateString_array[12];
    StateString[21] = StateString_array[13];
    StateString[30] = StateString_array[14];
    StateString[31] = StateString_array[15];
#endif

    return(pObj);
}

/****************************************************************************
 *                      CmdMBox_Destroy()
 ****************************************************************************
 * DESCRIPTION: Destroy the object 
 * 
 * INPUTS:  
 *      hCmdMBox        The object to free
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int             CmdMBox_Destroy(TI_HANDLE hCmdMBox)
{
    CmdMBox_T* pCmdMBox = (CmdMBox_T*)hCmdMBox;

    /* free timer */
    if (pCmdMBox->hTimer)
        utils_nullTimerDestroy(pCmdMBox->hOs, pCmdMBox->hTimer);
    
    /* free context */
       os_memoryFree(pCmdMBox->hOs, pCmdMBox, sizeof(CmdMBox_T));
    
    return OK;
}

/****************************************************************************
 *                      CmdMBox_Config()
 *****************************************************************************
 * DESCRIPTION: Configure the object 
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int CmdMBox_Config (TI_HANDLE hCmdMBox, TI_HANDLE hTNETWIF, TI_HANDLE hFwEvent, TI_HANDLE hCmdQueue, TI_HANDLE hReport)
{
    CmdMBox_T* pCmdMBox = (CmdMBox_T*)hCmdMBox;
    
    pCmdMBox->hReport = hReport;
    pCmdMBox->hTNETWIF = hTNETWIF;
    pCmdMBox->hFwEvent = hFwEvent;
    pCmdMBox->hCmdQueue = hCmdQueue;
    
    pCmdMBox->GetResultNormal_State = CMDMBOX_STATE_GETRESULT_NORMAL_IDLE;
    pCmdMBox->GetResultBlocking_State = CMDMBOX_STATE_GETRESULT_BLOCKING_IDLE;
    pCmdMBox->SendCmdNormal_State = CMDMBOX_STATE_SENDCMD_NORMAL_IDLE;
    pCmdMBox->SendCmdBlocking_State = CMDMBOX_STATE_SENDCMD_BLOCKING_IDLE;
    pCmdMBox->ActiveSM = NULL;  
    
  #ifdef USE_SYNC_API /* Blocking mode is using Synch IF  */
    pCmdMBox->SendCmdSM = CmdMBox_SM_SendCmdBlocking;
    pCmdMBox->GetResultSM = CmdMBox_SM_GetResultBlocking;   
  #else
    pCmdMBox->SendCmdSM = CmdMBox_SM_SendCmdNormal;
    pCmdMBox->GetResultSM = CmdMBox_SM_GetResultNormal; 
  #endif

    pCmdMBox->CmdMBox_FW_address = 0;
    pCmdMBox->GetResult_ParamsBuf = NULL;
    pCmdMBox->GetResult_ParamsLen = 0;
   
    os_timerStop (pCmdMBox->hOs, pCmdMBox->hTimer);

    return OK;
}


/****************************************************************************
 *                      CmdMBox_ConfigCb()
 ****************************************************************************
 * DESCRIPTION: Configure the mailbox address callback
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
static void CmdMBox_ConfigHwCb (TI_HANDLE hCmdMBox, UINT8 module_id, TI_STATUS status)
{
    CmdMBox_T* pCmdMBox = (CmdMBox_T*)hCmdMBox;

    WLAN_REPORT_INIT (pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,  
                      ("CmdMBox_ConfigHw: CmdMBox FW address = 0x%x\n", 
                      pCmdMBox->CmdMBox_FW_address));

    /* Call upper layer callback */
    pCmdMBox->fCb (pCmdMBox->hCb, module_id, OK);
}


/****************************************************************************
 *                      CmdMBox_ConfigHw()
 ****************************************************************************
 * DESCRIPTION: Configure the mailbox address 
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
TI_STATUS CmdMBox_ConfigHw (TI_HANDLE hCmdMBox, UINT8 module_id, fnotify_t fCb, TI_HANDLE hCb)
{
    CmdMBox_T *pCmdMBox = (CmdMBox_T*)hCmdMBox;
    int        status;

    pCmdMBox->fCb = (TNETWIF_callback_t)fCb;
    pCmdMBox->hCb = hCb;

    /* 
     * Get the command mailbox address
     */
    status = TNETWIF_ReadRegOpt (pCmdMBox->hTNETWIF, 
                                 REG_COMMAND_MAILBOX_PTR, 
                                 &pCmdMBox->CmdMBox_FW_address,
                                 module_id,
                                 CmdMBox_ConfigHwCb,
                                 hCmdMBox);

    switch (status)
    {
    case TNETWIF_ERROR:
        WLAN_REPORT_FATAL_ERROR (pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,  
                                 ("CmdMBox_ConfigHw: ERROR reading Mailbox addresses (0x%x) !!!\n", 
                                 pCmdMBox->CmdMBox_FW_address ));
        break;

    case TNETWIF_COMPLETE:
        WLAN_REPORT_INIT (pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,  
                          ("CmdMBox_ConfigHw: CmdMBox FW address = 0x%x\n", 
                          pCmdMBox->CmdMBox_FW_address));
        break;
    }
       
    return (TI_STATUS)status;
}

/****************************************************************************
 *                      CmdMBox_SetMode()
 ****************************************************************************
 * DESCRIPTION: Set the operational mode from blocking to normal
 * 
 * RETURNS: None
 ****************************************************************************/
int CmdMBox_SetModeNormal (TI_HANDLE hCmdMBox)
{
  #if defined(USE_SYNC_API)

    CmdMBox_T* pCmdMBox = (CmdMBox_T*)hCmdMBox;

    /* Set the state to NORMAL */
    pCmdMBox->SendCmdSM = CmdMBox_SM_SendCmdNormal;
    pCmdMBox->GetResultSM = CmdMBox_SM_GetResultNormal; 
    
    FwEvent_Enable (pCmdMBox->hFwEvent, ACX_INTR_CMD_COMPLETE);
    
    WLAN_REPORT_INFORMATION (pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,
            ("CmdMBox_SetMode: CmdMBox mode is now NORMAL"));       


  #endif

    return OK;
}

/****************************************************************************
 *                      CmdMBox_Reconfig()
 ****************************************************************************
 * DESCRIPTION: 
 * 
 * INPUTS:  
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int                 CmdMBox_Restart(TI_HANDLE hCmdMBox)
{
    CmdMBox_T* pCmdMBox = (CmdMBox_T*)hCmdMBox;
    
    return CmdMBox_Config(hCmdMBox, pCmdMBox->hTNETWIF, pCmdMBox->hFwEvent, pCmdMBox->hCmdQueue, pCmdMBox->hReport);
}


/****************************************************************************
 *                      CmdMBox_SendCmd()
 ****************************************************************************
 * DESCRIPTION: Try to send the Command to the Mailbox
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int                 CmdMBox_SendCmd(TI_HANDLE hCmdMBox, Command_e cmdType, UINT8* pParamsBuf, UINT32 paramsLen)
{
    CmdMBox_T* pCmdMBox = (CmdMBox_T*)hCmdMBox;
    
    /* prepare the Cmd Hw template */
    pCmdMBox->HW_CmdMBox.cmdID = cmdType;
    pCmdMBox->HW_CmdMBox.cmdStatus = OK; 
    os_memoryCopy(pCmdMBox->hOs, (void*)pCmdMBox->HW_CmdMBox.parameters, (void*)pParamsBuf, paramsLen);
    /* must make sure that the length is multiple of 32bit */
    if(paramsLen&0x3)
        paramsLen  = (paramsLen + 4) & 0xFFFFFFFC;  
    pCmdMBox->CmdLen = paramsLen + CMDMBOX_HEADER_LEN;  

    pCmdMBox->ActiveSM = pCmdMBox->SendCmdSM;
    return  pCmdMBox->ActiveSM(hCmdMBox, CMDMBOX_EVENT_SEND_CMD);
}

/****************************************************************************
 *                      CmdMBox_GetResult()
 ****************************************************************************
 * DESCRIPTION: Get result of the Cmd
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int                 CmdMBox_GetResult(TI_HANDLE hCmdMBox, UINT8* pParamsBuf, UINT32 paramsLen, UINT32* pStatus)
{
    CmdMBox_T* pCmdMBox = (CmdMBox_T*)hCmdMBox;
    int rc;

    pCmdMBox->GetResult_ParamsBuf = pParamsBuf;
    pCmdMBox->GetResult_ParamsLen= paramsLen;
    

    pCmdMBox->ActiveSM = pCmdMBox->GetResultSM;
    rc = pCmdMBox->ActiveSM(hCmdMBox, CMDMBOX_EVENT_GET_RESULT);

	if (pCmdMBox->HW_CmdMBox.cmdStatus == CMD_STATUS_SUCCESS || pCmdMBox->HW_CmdMBox.cmdStatus == CMD_STATUS_REJECT_MEAS_SG_ACTIVE) 
		{
			*pStatus = OK;
		}
		else
		{
			*pStatus = NOK;
		}
 
    return rc;
}

/****************************************************************************
 *                      CmdMBox_CmdCmplt()
 ****************************************************************************
 * DESCRIPTION: CallBack for command complete interrupt
 * 
 * INPUTS:  CbFunc  The Callback will be called we upon command complete interrupt 
 * 
 * RETURNS: None
 ****************************************************************************/
TI_STATUS  CmdMBox_CmdCmplt(TI_HANDLE hCmdMBox)
{
    CmdMBox_T* pCmdMBox = (CmdMBox_T*)hCmdMBox;
    TI_STATUS  rc;

    pCmdMBox->ActiveSM = pCmdMBox->SendCmdSM;
    rc = (TI_STATUS)pCmdMBox->ActiveSM (hCmdMBox, CMDMBOX_EVENT_CMD_CMPLT);
    if (rc == TNETWIF_COMPLETE)
        rc = TNETWIF_OK;
    return rc;
}

/****************************************************************************
 *                      CmdMBox_TxnCmplt()
 ****************************************************************************
 * DESCRIPTION: CallBack for Txn complete
 * 
 * INPUTS:  
 * 
 * RETURNS: None
 ****************************************************************************/
void                CmdMBox_TxnCmplt(TI_HANDLE hCmdMBox, UINT8 module_id ,TI_STATUS status)
{
    CmdMBox_T* pCmdMBox = (CmdMBox_T*)hCmdMBox;
    
    pCmdMBox->ActiveSM(hCmdMBox, CMDMBOX_EVENT_TXN_CMPLT);
}

/****************************************************************************
 *                      CmdMBox_BusReady()
 ****************************************************************************
 * DESCRIPTION: CallBack for Txn complete
 * 
 * INPUTS:  
 * 
 * RETURNS: None
 ****************************************************************************/
void                CmdMBox_BusReady(TI_HANDLE hCmdMBox, UINT8 module_id ,TI_STATUS status)
{
    CmdMBox_T* pCmdMBox = (CmdMBox_T*)hCmdMBox;
    
    pCmdMBox->ActiveSM(hCmdMBox, CMDMBOX_EVENT_BUS_READY);
}

/****************************************************************************
 *                      CmdMBox_SM_GetResultNormal()
 ****************************************************************************
 * DESCRIPTION: CmdMBox SM
 * 
 * INPUTS:  
 * 
 * RETURNS: None
 ****************************************************************************/
int             CmdMBox_SM_GetResultNormal(CmdMBox_T* pCmdMBox, CmdMBox_SMEvents_e event)
{
    int rc = OK;
    BOOLEAN breakWhile = FALSE;
    
    while(!breakWhile)
    {
#if CMDMBOX_DEBUG_PRINT
        WLAN_OS_REPORT(("CmdMBox_SM_GetResultNormal: state = %s (%d) event = %s(%d)\n",
            StateString[pCmdMBox->GetResultNormal_State],
            pCmdMBox->GetResultNormal_State,
            EventString[event],
            event));
#endif
        switch(pCmdMBox->GetResultNormal_State)
        {
            /***************************************
            CMDMBOX_STATE_GETRESULT_NORMAL_IDLE
            ***************************************/
            case CMDMBOX_STATE_GETRESULT_NORMAL_IDLE:
                switch(event)
                {
                    case CMDMBOX_EVENT_GET_RESULT:

                        /* read the results */
                        if(pCmdMBox->GetResult_ParamsBuf)
                        {                       
                            /* need to read the results also */
                            rc = TNETWIF_ReadMemOpt (pCmdMBox->hTNETWIF, 
                                                     pCmdMBox->CmdMBox_FW_address, 
                                                     PADREAD (&pCmdMBox->HW_CmdMBox), 
                                                     pCmdMBox->GetResult_ParamsLen + CMDMBOX_HEADER_LEN,
                                                     FW_EVENT_MODULE_ID, 
                                                     CmdMBox_TxnCmplt, 
                                                     pCmdMBox);
                        }
                        else
                        {
                            /* need to read the status only */                          
                            rc = TNETWIF_ReadMemOpt (pCmdMBox->hTNETWIF, 
                                                     pCmdMBox->CmdMBox_FW_address, 
                                                     PADREAD (&pCmdMBox->HW_CmdMBox), 
                                                     CMDMBOX_HEADER_LEN,
                                                     FW_EVENT_MODULE_ID, 
                                                     CmdMBox_TxnCmplt, 
                                                     pCmdMBox);
                        }                       

                        if(rc == TNETWIF_PENDING)
                        {
                            pCmdMBox->GetResultNormal_State = CMDMBOX_STATE_GETRESULT_NORMAL_WAIT_TXN;
                        }
                        else
                        {

                            /* check the status */
                            CMDMBOX_CHECK_STATUS;                           

                            /* 
                            if GetResult_ParamsBuf is NULL then we only need to check the status and then 
                            we don't need to copy the results 
                            */
                            if(pCmdMBox->GetResult_ParamsBuf)
                            {
                                /* copy the results to the caller buffer */
                                os_memoryCopy(pCmdMBox->hOs, (void*)pCmdMBox->GetResult_ParamsBuf, (void*)pCmdMBox->HW_CmdMBox.parameters, pCmdMBox->GetResult_ParamsLen);
                            }
                            
                            pCmdMBox->GetResultNormal_State = CMDMBOX_STATE_GETRESULT_NORMAL_IDLE;
                        }               
                        breakWhile = TRUE;
                        break;
                    default:
                        WLAN_REPORT_ERROR(pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,
                            ("CmdMBox_SM_GetResultNormal: ** ERROR **  No such event (%d) for state CMDMBOX_STATE_GETRESULT_NORMAL_IDLE\n",event));     
                            return NOK;
                }
                break;
            /***************************************
            CMDMBOX_STATE_GETRESULT_NORMAL_WAIT_TXN
            ***************************************/
            case CMDMBOX_STATE_GETRESULT_NORMAL_WAIT_TXN:
                switch(event)
                {
                    case CMDMBOX_EVENT_TXN_CMPLT:

                        /* check the status */
                        CMDMBOX_CHECK_STATUS;

                        /* 
                        if GetResult_ParamsBuf is NULL then we only need to check the status and then 
                        we don't need to copy the results 
                        */
                        if(pCmdMBox->GetResult_ParamsBuf)
                        {
                            /* copy the results to the caller buffer */
                            os_memoryCopy(pCmdMBox->hOs, (void*)pCmdMBox->GetResult_ParamsBuf, (void*)pCmdMBox->HW_CmdMBox.parameters, pCmdMBox->GetResult_ParamsLen);
                        }
                        
                        /* call the CmdQueue CB */
                        if (pCmdMBox->HW_CmdMBox.cmdStatus == CMD_STATUS_SUCCESS || pCmdMBox->HW_CmdMBox.cmdStatus == CMD_STATUS_REJECT_MEAS_SG_ACTIVE) 
						{
							CmdQueue_ResultReceived(pCmdMBox->hCmdQueue, OK);
						}
						else
						{
							CmdQueue_ResultReceived(pCmdMBox->hCmdQueue, NOK);
						}
                        
                        FwEvent_EventComplete(pCmdMBox->hFwEvent, TNETWIF_OK);
                        pCmdMBox->GetResultNormal_State = CMDMBOX_STATE_GETRESULT_NORMAL_IDLE;
                        return OK;
                    default:
                        WLAN_REPORT_ERROR(pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,
                            ("CmdMBox_SM_GetResultNormal: ** ERROR **  No such event (%d) for state CMDMBOX_STATE_GETRESULT_NORMAL_WAIT_TXN\n",event));     
                            return NOK;
                }
/*                break;  */
            default:
                WLAN_REPORT_ERROR(pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,
                    ("CmdMBox_SM_GetResultNormal: ** ERROR **  No such state (%d)\n",pCmdMBox->GetResultNormal_State));     
                    return NOK;
        }
    }
	
    return rc;
    
}


#ifdef USE_SYNC_API

/****************************************************************************
 *                      CmdMBox_SM_GetResultBlocking()
 ****************************************************************************
 * DESCRIPTION: CmdMBox SM
 * 
 * INPUTS:  
 * 
 * RETURNS: None
 ****************************************************************************/
int             CmdMBox_SM_GetResultBlocking(CmdMBox_T* pCmdMBox, CmdMBox_SMEvents_e event)
{
    int rc = OK;
    BOOLEAN breakWhile = FALSE;
    
    while(!breakWhile)
    {
#if CMDMBOX_DEBUG_PRINT
        WLAN_OS_REPORT(("CmdMBox_SM_GetResultBlocking: state = %s (%d) event = %s(%d)\n",
            StateString[pCmdMBox->GetResultBlocking_State],
            pCmdMBox->GetResultBlocking_State,
            EventString[event],
            event));
#endif
        switch(pCmdMBox->GetResultBlocking_State)
        {
            /***************************************
            CMDMBOX_STATE_GETRESULT_BLOCKING_IDLE
            ***************************************/
            case CMDMBOX_STATE_GETRESULT_BLOCKING_IDLE:
                switch(event)
                {
                    case CMDMBOX_EVENT_GET_RESULT:
                        /* read the results */
                        if(pCmdMBox->GetResult_ParamsBuf)
                        {
                            /* need to read the results also */
                            rc = TNETWIF_ReadMemSync (pCmdMBox->hTNETWIF, 
                                                      pCmdMBox->CmdMBox_FW_address, 
                                                      PADREAD(&pCmdMBox->HW_CmdMBox), 
                                                      pCmdMBox->GetResult_ParamsLen + CMDMBOX_HEADER_LEN);
                        }
                        else
                        {
                            rc = TNETWIF_ReadMemSync (pCmdMBox->hTNETWIF, 
                                                      pCmdMBox->CmdMBox_FW_address, 
                                                      PADREAD (&pCmdMBox->HW_CmdMBox), 
                                                      CMDMBOX_HEADER_LEN);                         
                        }

                        if(rc == TNETWIF_PENDING)
                        {
                            pCmdMBox->GetResultBlocking_State = CMDMBOX_STATE_GETRESULT_BLOCKING_WAIT_TXN;
                        }
                        else
                        {
                            /* check the status */
                            CMDMBOX_CHECK_STATUS;

                            /* 
                            if GetResult_ParamsBuf is NULL then we only need to check the status and then 
                            we don't need to copy the results 
                            */
                            if(pCmdMBox->GetResult_ParamsBuf)
                            {
                                /* copy the results to the caller buffer */
                                os_memoryCopy(pCmdMBox->hOs, (void*)pCmdMBox->GetResult_ParamsBuf, (void*)pCmdMBox->HW_CmdMBox.parameters, pCmdMBox->GetResult_ParamsLen);
                            }               
                        }               
                        breakWhile = TRUE;
                        break;
                    default:
                        WLAN_REPORT_ERROR(pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,
                            ("CmdMBox_SM_GetResultBlocking: ** ERROR **  No such event (%d) for state CMDMBOX_STATE_GETRESULT_BLOCKING_IDLE\n",event));     
                            return NOK;
                }
                break;
            /***************************************
            CMDMBOX_STATE_GETRESULT_BLOCKING_WAIT_TXN
            ***************************************/
            case CMDMBOX_STATE_GETRESULT_BLOCKING_WAIT_TXN:
                switch(event)
                {
				case CMDMBOX_EVENT_TXN_CMPLT:
						
                        /* check the status */
                        CMDMBOX_CHECK_STATUS;

                        /* 
                        if GetResult_ParamsBuf is NULL then we only need to check the status and then 
                        we don't need to copy the results 
                        */
                        if(pCmdMBox->GetResult_ParamsBuf)
                        {
                            /* copy the results to the caller buffer */
                            os_memoryCopy(pCmdMBox->hOs, (void*)pCmdMBox->GetResult_ParamsBuf, (void*)pCmdMBox->HW_CmdMBox.parameters, pCmdMBox->GetResult_ParamsLen);
                        }

                        /* call the CmdQueue CB */
						if (pCmdMBox->HW_CmdMBox.cmdStatus == CMD_STATUS_SUCCESS || pCmdMBox->HW_CmdMBox.cmdStatus == CMD_STATUS_REJECT_MEAS_SG_ACTIVE) 
						{
							CmdQueue_ResultReceived(pCmdMBox->hCmdQueue, OK);
						}
						else
						{
							CmdQueue_ResultReceived(pCmdMBox->hCmdQueue, NOK);
						}
                        

                        pCmdMBox->GetResultBlocking_State = CMDMBOX_STATE_GETRESULT_BLOCKING_IDLE;
                        breakWhile = TRUE;
                        break;
                    default:
                        WLAN_REPORT_ERROR(pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,
                            ("CmdMBox_SM_GetResultBlocking: ** ERROR **  No such event (%d) for state CMDMBOX_STATE_GETRESULT_BLOCKING_WAIT_TXN\n",event));     
                            return NOK;
                }
                break;
            default:
                WLAN_REPORT_ERROR(pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,
                    ("CmdMBox_SM_GetResultBlocking: ** ERROR **  No such state (%d)\n",pCmdMBox->GetResultBlocking_State));     
                    return NOK;
        }
    }
    return rc;
    
}

#endif /* USE_SYNC_API */


/****************************************************************************
 *                      CmdMBox_SM_SendCmdNormal()
 ****************************************************************************
 * DESCRIPTION: CmdMBox SM
 * 
 * INPUTS:  
 * 
 * RETURNS: None
 ****************************************************************************/
int             CmdMBox_SM_SendCmdNormal(CmdMBox_T* pCmdMBox, CmdMBox_SMEvents_e event)
{
    int rc = OK;
    BOOLEAN breakWhile = FALSE;
    
    while(!breakWhile)
{
#if CMDMBOX_DEBUG_PRINT
        WLAN_OS_REPORT(("CmdMBox_SM_SendCmdNormal: state = %s (%d) event = %s(%d) rc = %d\n",
            StateString[pCmdMBox->SendCmdNormal_State],
            pCmdMBox->SendCmdNormal_State,
            EventString[event],
            event,rc));
#endif
        switch(pCmdMBox->SendCmdNormal_State)
        {
            /***************************************
            CMDMBOX_STATE_SENDCMD_NORMAL_IDLE
            ***************************************/
            case CMDMBOX_STATE_SENDCMD_NORMAL_IDLE:
                switch(event)
                {
                    case CMDMBOX_EVENT_SEND_CMD:                                            
                        pCmdMBox->SendCmdNormal_State = CMDMBOX_STATE_SENDCMD_NORMAL_WAIT_BUS;
                        /* ask for the bus */
                        TNETWIF_Start (pCmdMBox->hTNETWIF,   HAL_CMD_MODULE_ID, 
                            pCmdMBox, CmdMBox_BusReady);        
                        rc = TNETWIF_PENDING;
                        breakWhile = TRUE;
                        break;
                    case CMDMBOX_EVENT_CMD_CMPLT:
                        /* stop timeout timer */
                        os_timerStop(pCmdMBox->hOs, pCmdMBox->hTimer);

                        /* call the CmdQueue CB */
                        rc = CmdQueue_SendCmplt(pCmdMBox->hCmdQueue);                       
                        breakWhile = TRUE;
                        break;
                    default:
                        WLAN_REPORT_ERROR(pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,
                            ("CmdMBox_SM_SendCmdNormal: ** ERROR **  No such event (%d) for state CMDMBOX_STATE_SENDCMD_NORMAL_IDLE\n",event));     
                            return NOK;
                }
                break;
            /***************************************
            CMDMBOX_STATE_SENDCMD_NORMAL_WAIT_BUS
            ***************************************/
            case CMDMBOX_STATE_SENDCMD_NORMAL_WAIT_BUS:
                switch(event)
                {
                    case CMDMBOX_EVENT_BUS_READY:                       
                        /* write the Cmd - subtract offset for the bus reserved place */
            rc = TNETWIF_WriteMemOpt (pCmdMBox->hTNETWIF, 
                                                  pCmdMBox->CmdMBox_FW_address, 
                                                  PADWRITE (&pCmdMBox->HW_CmdMBox), 
                                                  pCmdMBox->CmdLen,
                                                  HAL_CMD_MODULE_ID, 
                                                  CmdMBox_TxnCmplt, 
                                                  pCmdMBox);
            if(rc == TNETWIF_PENDING)
                     {
                            pCmdMBox->SendCmdNormal_State = CMDMBOX_STATE_SENDCMD_NORMAL_WAIT_TXN_BUF;  
                            breakWhile = TRUE;
                     }
                     else
                     {
                            pCmdMBox->SendCmdNormal_State = CMDMBOX_STATE_SENDCMD_NORMAL_WRITE_TRIG_v;                              
                    }
                     break;
                        default:
                        WLAN_REPORT_ERROR(pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,
                                ("CmdMBox_SM_SendCmdNormal: ** ERROR **  No such event (%d) for state CMDMBOX_STATE_SENDCMD_NORMAL_WAIT_BUS\n",event));     
                            return NOK;
                }
                break;
            /***************************************
            CMDMBOX_STATE_SENDCMD_NORMAL_WAIT_TXN_BUF
            ***************************************/
            case CMDMBOX_STATE_SENDCMD_NORMAL_WAIT_TXN_BUF:
                switch(event)
                {
                    case CMDMBOX_EVENT_TXN_CMPLT:                       
                        pCmdMBox->SendCmdNormal_State = CMDMBOX_STATE_SENDCMD_NORMAL_WRITE_TRIG_v;                          
                        break;
                    default:
                        WLAN_REPORT_ERROR(pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,
                            ("CmdMBox_SM_SendCmdNormal: ** ERROR **  No such event (%d) for state CMDMBOX_STATE_SENDCMD_NORMAL_WAIT_TXN_BUF\n",event));     
                            return NOK;
                }
                break;
            /***************************************
            CMDMBOX_STATE_SENDCMD_NORMAL_WRITE_TRIG_v
            ***************************************/
            case CMDMBOX_STATE_SENDCMD_NORMAL_WRITE_TRIG_v:
                /* start timeout timer */
#ifdef DM_USE_WORKQUEUE
                os_timerStart(pCmdMBox->hOs, pCmdMBox->hTimer,
                      CMDMBOX_WAIT_TIMEOUT * 2, FALSE); /* Dm: Wait for 1000 ms */
#else
                os_timerStart(pCmdMBox->hOs, pCmdMBox->hTimer,
                      CMDMBOX_WAIT_TIMEOUT, FALSE);
#endif

                /* write the FW trigger */
                rc = TNETWIF_WriteRegOpt (pCmdMBox->hTNETWIF, 
                                          ACX_REG_INTERRUPT_TRIG, 
                                          INTR_TRIG_CMD,
                                          HAL_CMD_MODULE_ID, 
                                          CmdMBox_TxnCmplt, 
                                          pCmdMBox);
                if(rc == TNETWIF_PENDING)
                {
                    pCmdMBox->SendCmdNormal_State = CMDMBOX_STATE_SENDCMD_NORMAL_WAIT_TXN_TRIG;
                }
                else
                {
                    pCmdMBox->SendCmdNormal_State = CMDMBOX_STATE_SENDCMD_NORMAL_IDLE;
                    /* release the bus resource*/
                    TNETWIF_Finish (pCmdMBox->hTNETWIF, HAL_CMD_MODULE_ID, NULL, NULL);
                }
                breakWhile = TRUE;
                break;
            /***************************************
            CMDMBOX_STATE_SENDCMD_NORMAL_WAIT_TXN_TRIG
            ***************************************/
            case CMDMBOX_STATE_SENDCMD_NORMAL_WAIT_TXN_TRIG:
                switch(event)
                {
                    case CMDMBOX_EVENT_TXN_CMPLT:                       
                        pCmdMBox->SendCmdNormal_State = CMDMBOX_STATE_SENDCMD_NORMAL_IDLE;                          
                        /* release the bus resource*/
                        TNETWIF_Finish (pCmdMBox->hTNETWIF, HAL_CMD_MODULE_ID, NULL, NULL);
            rc = OK;
                        break;
                    default:
                        WLAN_REPORT_ERROR(pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,
                            ("CmdMBox_SM_SendCmdNormal: ** ERROR **  No such event (%d) for state CMDMBOX_STATE_SENDCMD_NORMAL_WAIT_TXN_TRIG\n",event));        
                            return NOK;
                }
                breakWhile = TRUE;
                break;
            default:
                WLAN_REPORT_ERROR(pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,
                    ("CmdMBox_SM_SendCmdNormal: ** ERROR **  No such state (%d)\n",pCmdMBox->SendCmdNormal_State));     
                    return NOK;
        }
    }
#if CMDMBOX_DEBUG_PRINT
        WLAN_OS_REPORT(("CmdMBox_SM_SendCmdNormal return = %d\n",rc));
#endif
    return rc;
    
}


#ifdef USE_SYNC_API

/****************************************************************************
 *                      CmdMBox_SM_SendCmdBlocking()
 ****************************************************************************
 * DESCRIPTION: CmdMBox SM
 * 
 * INPUTS:  
 * 
 * RETURNS: None
 ****************************************************************************/
int             CmdMBox_SM_SendCmdBlocking(CmdMBox_T* pCmdMBox, CmdMBox_SMEvents_e event)
{
    int rc = OK;
    BOOLEAN breakWhile = FALSE;
    BOOLEAN CmdCmpltFlag = FALSE;

    static UINT32 timeoutCounterExpire;
    static UINT32 timeoutCounter;
    
    while(!breakWhile)
    {
#if CMDMBOX_DEBUG_PRINT
        WLAN_OS_REPORT(("CmdMBox_SM_SendCmdBlocking: state = %s (%d) event = %s(%d) rc = %d\n",
            StateString[pCmdMBox->SendCmdBlocking_State],
            pCmdMBox->SendCmdBlocking_State,
            EventString[event],
            event,rc));
#endif
        switch(pCmdMBox->SendCmdBlocking_State)
        {
            /***************************************
            CMDMBOX_STATE_SENDCMD_BLOCKING_IDLE
            ***************************************/
            case CMDMBOX_STATE_SENDCMD_BLOCKING_IDLE:
                switch(event)
                {
                    case CMDMBOX_EVENT_SEND_CMD:                        

                        pCmdMBox->SendCmdBlocking_State = CMDMBOX_STATE_SENDCMD_BLOCKING_WAIT_BUS;
                       
                        /* use the bus directly - it's O.K. since it's the init phase */
                        event = CMDMBOX_EVENT_BUS_READY;

                        break;
                    default:
                        WLAN_REPORT_ERROR(pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,
                            ("CmdMBox_SM_SendCmdBlocking: ** ERROR **  No such event (%d) for state CMDMBOX_STATE_SENDCMD_BLOCKING_IDLE\n",event));     
                            return NOK;
                }
                break;
            /***************************************
            CMDMBOX_STATE_SENDCMD_BLOCKING_WAIT_BUS
            ***************************************/
            case CMDMBOX_STATE_SENDCMD_BLOCKING_WAIT_BUS:
                switch(event)
                {
                    case CMDMBOX_EVENT_BUS_READY:  
                        /* write the Cmd */
                        rc = TNETWIF_WriteMemSync (pCmdMBox->hTNETWIF, 
                                                   pCmdMBox->CmdMBox_FW_address, 
                                                   PADWRITE (&pCmdMBox->HW_CmdMBox), 
                                                   pCmdMBox->CmdLen);

                        pCmdMBox->SendCmdBlocking_State = CMDMBOX_STATE_SENDCMD_BLOCKING_WRITE_TRIG_v;

                        break;
                    default:
                        WLAN_REPORT_ERROR(pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,
                            ("CmdMBox_SM_SendCmdBlocking: ** ERROR **  No such event (%d) for state CMDMBOX_STATE_SENDCMD_BLOCKING_WAIT_BUS\n",event));     
                            return NOK;
                }
                break;
            /***************************************
            CMDMBOX_STATE_SENDCMD_BLOCKING_WAIT_TXN_BUF
            ***************************************/
            case CMDMBOX_STATE_SENDCMD_BLOCKING_WAIT_TXN_BUF:
                switch(event)
                {
                    case CMDMBOX_EVENT_TXN_CMPLT:                       
                        pCmdMBox->SendCmdBlocking_State = CMDMBOX_STATE_SENDCMD_BLOCKING_WRITE_TRIG_v;                          
                        break;
                    default:
                        WLAN_REPORT_ERROR(pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,
                            ("CmdMBox_SM_SendCmdBlocking: ** ERROR **  No such event (%d) for state CMDMBOX_STATE_SENDCMD_BLOCKING_WAIT_TXN_BUF\n",event));     
                            return NOK;
                }
                break;
            /***************************************
            CMDMBOX_STATE_SENDCMD_BLOCKING_WRITE_TRIG_v
            ***************************************/
            case CMDMBOX_STATE_SENDCMD_BLOCKING_WRITE_TRIG_v:
                /* init timeout counter */              
                timeoutCounterExpire = (CMDMBOX_WAIT_TIMEOUT*CMDMBOX_US_TO_MS)/CMDMBOX_WAIT_CMPLT_STALL_TIME;
                timeoutCounter = 0;

                /* write the FW trigger */
                if(pCmdMBox->useOpt) 
                rc = TNETWIF_WriteRegOpt  (pCmdMBox->hTNETWIF, 
                                               ACX_REG_INTERRUPT_TRIG, 
                                               INTR_TRIG_CMD,
                                               HAL_CMD_MODULE_ID, 
                                               CmdMBox_TxnCmplt, 
                                               pCmdMBox);
                else
                    rc = TNETWIF_WriteRegSync (pCmdMBox->hTNETWIF, 
                                               ACX_REG_INTERRUPT_TRIG, 
                                               INTR_TRIG_CMD);
               
                if(rc == TNETWIF_PENDING)
                {
                    pCmdMBox->SendCmdBlocking_State = CMDMBOX_STATE_SENDCMD_BLOCKING_WAIT_TXN_TRIG;
                    breakWhile = TRUE;
                }
                else
                {
                    pCmdMBox->SendCmdBlocking_State = CMDMBOX_STATE_SENDCMD_BLOCKING_POLL_CMPLT_v;
                }
                break;
            /***************************************
            CMDMBOX_STATE_SENDCMD_BLOCKING_WAIT_TXN_TRIG
            ***************************************/
            case CMDMBOX_STATE_SENDCMD_BLOCKING_WAIT_TXN_TRIG:
                switch(event)
                {
                    case CMDMBOX_EVENT_TXN_CMPLT:                       
                        pCmdMBox->SendCmdBlocking_State = CMDMBOX_STATE_SENDCMD_BLOCKING_POLL_CMPLT_v;                          
                        break;
                    default:
                        WLAN_REPORT_ERROR(pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,
                            ("CmdMBox_SM_SendCmdBlocking: ** ERROR **  No such event (%d) for state CMDMBOX_STATE_SENDCMD_BLOCKING_WAIT_TXN_TRIG\n",event));        
                            return NOK;
                }
                break;
            /***************************************
            CMDMBOX_STATE_SENDCMD_BLOCKING_POLL_CMPLT_v
            ***************************************/
            case CMDMBOX_STATE_SENDCMD_BLOCKING_POLL_CMPLT_v:                               
                /* check CmdCmplt */
                /* the following function is done in synchronize mode */
                timeoutCounter++;
                CmdCmpltFlag = CmdMbox_CheckAndAck(pCmdMBox->hTNETWIF, ACX_INTR_CMD_COMPLETE);
                pCmdMBox->SendCmdBlocking_State = CMDMBOX_STATE_SENDCMD_BLOCKING_FINISH_v;
                break;

            /***************************************
            CMDMBOX_STATE_SENDCMD_BLOCKING_FINISH_v
            ***************************************/
            case CMDMBOX_STATE_SENDCMD_BLOCKING_FINISH_v:                               
                /* check CmdCmplt */
                if(CmdCmpltFlag == FALSE)
                {
                    /* check timeout counter */
                    if(timeoutCounter == timeoutCounterExpire)
                        CmdMBox_TimeOut(pCmdMBox);
                        
                    pCmdMBox->SendCmdBlocking_State = CMDMBOX_STATE_SENDCMD_BLOCKING_POLL_CMPLT_v;
                    os_StalluSec(pCmdMBox->hOs, CMDMBOX_WAIT_CMPLT_STALL_TIME);
                }
                else
                {
                    /* stop timeout timer */
                    os_timerStop(pCmdMBox->hOs, pCmdMBox->hTimer);

                    pCmdMBox->SendCmdBlocking_State = CMDMBOX_STATE_SENDCMD_BLOCKING_IDLE;
                    
                    /* call the CmdQueue CB */
                    rc = CmdQueue_SendCmplt(pCmdMBox->hCmdQueue);
                    breakWhile = TRUE;
                }
                break;
            default:
                WLAN_REPORT_ERROR(pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,
                    ("CmdMBox_SM_SendCmdBlocking: ** ERROR **  No such state (%d)\n",pCmdMBox->SendCmdBlocking_State));     
                    return NOK;
        }
    }
#if CMDMBOX_DEBUG_PRINT
        WLAN_OS_REPORT(("CmdMBox_SM_SendCmdBlocking rc = %d\n",rc));
#endif
    return rc;
}
#endif /* USE_SYNC_API */
/****************************************************************************
 *                      CmdMBox_TimeOut()
 ****************************************************************************
 * DESCRIPTION: 
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK, ERROR
 ****************************************************************************/
void            CmdMBox_TimeOut(TI_HANDLE hCmdMBox)
{
    CmdMBox_T* pCmdMBox = (CmdMBox_T*)hCmdMBox;
    
    WLAN_REPORT_ERROR(pCmdMBox->hReport, CMD_MBOX_MODULE_LOG,
            ("CmdMBox_TimeOut: Timeout occured in CmdMBox\n"));

    /* call error CB */
    CmdQueue_Error(pCmdMBox->hCmdQueue);
    
    return;
}

/****************************************************************************
 *                      CmdMbox_CheckAndAck()
 ****************************************************************************
 * DESCRIPTION: Check if HostIfReg is On
 *              This function is used on SDIO only.
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK, ERROR
 ****************************************************************************/

int CmdMbox_CheckAndAck (TI_HANDLE hTNETWIF, UINT32 Intr)
{
  #ifdef USE_SYNC_API

    UINT32 Reg_IntrNoClear;

    TNETWIF_ReadRegSync(hTNETWIF,ACX_REG_INTERRUPT_NO_CLEAR, &Reg_IntrNoClear);  
    
    if (IS_MASK_ON (Reg_IntrNoClear, Intr))
    {
        TNETWIF_WriteRegSync (hTNETWIF, ACX_REG_INTERRUPT_ACK, Intr);
        return 1;
    }

  #endif /* USE_SYNC_API */

    return 0;   
}

TI_STATUS CmdMBox_GetStatus(TI_HANDLE hCmdMBox)
{
    CmdMBox_T* pCmdMBox = (CmdMBox_T*)hCmdMBox;
    TI_STATUS Status;
    Status = (pCmdMBox->HW_CmdMBox.cmdStatus == CMD_STATUS_SUCCESS) ? OK : NOK;

	switch (pCmdMBox->HW_CmdMBox.cmdStatus)
    { 
		case CMD_STATUS_SUCCESS: 
		{
            Status = OK;
			break;
		}
		case CMD_STATUS_REJECT_MEAS_SG_ACTIVE:
		{ 
			Status = SG_REJECT_MEAS_SG_ACTIVE;
            break;
		}
		case CMD_MAILBOX_IDLE: 
		case CMD_STATUS_UNKNOWN_CMD: 
		case CMD_STATUS_UNKNOWN_IE: 
		case CMD_STATUS_RX_BUSY: 
		case CMD_STATUS_INVALID_PARAM: 
		case CMD_STATUS_TEMPLATE_TOO_LARGE: 
		case CMD_STATUS_OUT_OF_MEMORY: 
		case CMD_STATUS_STA_TABLE_FULL: 
		case CMD_STATUS_RADIO_ERROR: 
		case CMD_STATUS_WRONG_NESTING: 
        case CMD_STATUS_TIMEOUT: 
		case CMD_STATUS_FW_RESET: 
		default: 
        { 
            Status = NOK;
			break; 
        }
	} /* end of switch */

    WLAN_REPORT_INFORMATION(pCmdMBox->hReport, HAL_HW_CTRL_MODULE_LOG,
        ("%s - TI_STATUS = %s(%d) <= pCmdMBox->HW_CmdMBox.cmdStatus = %d\n", __FUNCTION__, 
        (Status==OK)?"OK":"NOK",Status, pCmdMBox->HW_CmdMBox.cmdStatus));

    return Status;
}

