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
#include "osTIType.h"
#include "whalCommon.h"
#include "whalHwDefs.h"
#include "whalBus_Api.h"
#include "CmdMBox_api.h"
#include "CmdQueue_api.h"
#include "CmdQueue.h"

#ifdef TI_DBG
static char *StateString[CMDQUEUE_STATE_NUM] = {
	"CMDQUEUE_STATE_IDLE",  /* 0 */
	"CMDQUEUE_STATE_SEND_CMD_v",  /* 1 */
	"CMDQUEUE_STATE_WAIT_SEND_CMPLT",  /* 2 */
	"CMDQUEUE_STATE_INTERROGATE_v",  /* 3 */
	"CMDQUEUE_STATE_WAIT_RESULT",  /* 4 */
	"CMDQUEUE_STATE_FINISH_v",  /* 5 */
};

static char *EventString[CMDQUEUE_EVENT_NUM] = {
	"CMDQUEUE_EVENT_RUN",  /* 1 */
	"CMDQUEUE_EVENT_SEND_CMPLT",  /* 2 */
	"CMDQUEUE_EVENT_RESULT_RECEIVED",  /* 3 */
};

#endif /* TI_DBG */


/****************************************************************************
 *                      CmdQueue_Create()
 ****************************************************************************
 * DESCRIPTION: Create the CmdQueue object
 *
 * INPUTS:  TI_HANDLE *hOs
 *
 * OUTPUT:  None
 *
 * RETURNS: The Created object
 *****************************************************************************/
TI_HANDLE CmdQueue_Create(TI_HANDLE hOs)
{
    
    CmdQueue_T  *pObj;

    pObj = os_memoryAlloc(hOs, sizeof(CmdQueue_T));
    if (pObj == NULL)
    {
        WLAN_OS_REPORT(("FATAL ERROR: CmdQueue_Create(): Error Creating CmdQueue - Aborting\n"));
        return NULL;
    }

    /* reset control module control block */
    os_memoryZero(hOs, pObj, sizeof(CmdQueue_T));
    pObj->hOs = hOs;
    
    return(pObj);   
}


/****************************************************************************
 *                      CmdQueue_Destroy()
 ****************************************************************************
 * DESCRIPTION: Destroy the object
 *
 * INPUTS:  hCmdQueue   The object to free
 *  
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int					CmdQueue_Destroy(TI_HANDLE hCmdQueue)
{
	CmdQueue_T* pCmdQueue = (CmdQueue_T*)	hCmdQueue;

	/* free context */
       os_memoryFree(pCmdQueue->hOs, pCmdQueue, sizeof(CmdQueue_T));

	return OK;
}

/****************************************************************************
 *                      CmdQueue_Config()
 ****************************************************************************
 * DESCRIPTION: Config the CmdQueue object
 *
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int CmdQueue_Config (TI_HANDLE hCmdQueue, TI_HANDLE hCmdMBox, TI_HANDLE hReport)                                                       
{
    CmdQueue_T* pCmdQueue = (CmdQueue_T*) hCmdQueue;

    pCmdQueue->Head = 0;
    pCmdQueue->Tail = 0;
    pCmdQueue->NumberOfCommandInQueue = 0;
    pCmdQueue->MaxNumberOfCommandInQueue = 0;
    pCmdQueue->State = CMDQUEUE_STATE_IDLE;
    pCmdQueue->CmdCompleteGenericCB_Func = NULL;
    pCmdQueue->CmdCompleteGenericCB_Arg = NULL;
    pCmdQueue->FailureCB = NULL;
    pCmdQueue->FailureCbHandle = NULL;
    pCmdQueue->SM_RC = 0;
    pCmdQueue->hReport = hReport;
    pCmdQueue->hCmdMBox = hCmdMBox;
    pCmdQueue->ErrorFlag = FALSE;

    /*
     * NOTE: don't set NumberOfRecoveryNodes = 0; 
     *       its value is used by recovery process
     */

    return OK;
}

/****************************************************************************
 *                      CmdQueue_StartReconfig()
 ****************************************************************************
 * DESCRIPTION: Restart the module for recovery. Clean the queue but save al the CB in the queue.
 *
 * INPUTS:  
 *
 * OUTPUT:  
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int					CmdQueue_StartReconfig(TI_HANDLE hCmdQueue)
{
	CmdQueue_T* pCmdQueue = (CmdQueue_T*)	hCmdQueue;
	int CurrentCmdindex;
    	int first  = pCmdQueue->Head;
	CmdQueue_CmdNode_T* pHead ;
	CmdQueue_RecoveryNode_T* pRecoveryNode;
	
	/* 
	stop the SM
	*/
    	pCmdQueue->State = CMDQUEUE_STATE_IDLE;

	WLAN_REPORT_INFORMATION(pCmdQueue->hReport, CMD_MBOX_MODULE_LOG,
        							("CmdQueue_Clean: Cleaning CmdQueue Queue"));
    
	/*
	Save The Call Back Function in the Queue in order the return them after the recovery 
	with on error status 
	*/ 

	/* Clean The Command Call Back Counter */ 
	pCmdQueue->NumberOfRecoveryNodes = 0;
	pRecoveryNode = &pCmdQueue->RecoveryQueue[pCmdQueue->NumberOfRecoveryNodes];
	for(CurrentCmdindex = 0 ; CurrentCmdindex < pCmdQueue->NumberOfCommandInQueue ; CurrentCmdindex++)
    	{
		pHead  =  &pCmdQueue->CmdQueue[first];

		if(pHead->CB_Func != NULL)
		{ /*Copy the interrogate CB and the interrogate data buffer pointer */
			pRecoveryNode->CB_Func = pHead->CB_Func;
			pRecoveryNode->CB_Arg = pHead->CB_Arg;
			pRecoveryNode->interrogateParamsBuf = pHead->interrogateParamsBuf;
			pCmdQueue->NumberOfRecoveryNodes++;
			pRecoveryNode = &pCmdQueue->RecoveryQueue[pCmdQueue->NumberOfRecoveryNodes];
		}		
		first++;
		if(first == CMDQUEUE_QUEUE_DEPTH)
			first = 0;
	}

	/*
	init the queue
	*/
	pCmdQueue->Head = 0;
    pCmdQueue->Tail = 0;
    pCmdQueue->NumberOfCommandInQueue = 0;

	return OK;
}

/****************************************************************************
 *                      CmdQueue_EndReconfig()
 ****************************************************************************
 * DESCRIPTION: Call the stored CB to end the recovery of the MBox queue
 *
 * INPUTS:  
 *
 * OUTPUT:  
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int					CmdQueue_EndReconfig(TI_HANDLE hCmdQueue)
{
	CmdQueue_T* pCmdQueue = (CmdQueue_T*)	hCmdQueue;
	
	int	Cbindex;
	CmdQueue_RecoveryNode_T *pHead;

	for(Cbindex = 0; Cbindex < pCmdQueue->NumberOfRecoveryNodes; Cbindex++)
	{
		pHead  =  &pCmdQueue->RecoveryQueue[Cbindex];

		if(pHead->interrogateParamsBuf)
	    	{
    			((CmdQueue_InterrogateCB_t)pHead->CB_Func)(pHead->CB_Arg, CMD_STATUS_FW_RESET,pHead->interrogateParamsBuf);
    		}
		else
		{
			((CmdQueue_CB_t)pHead->CB_Func)(pHead->CB_Arg, CMD_STATUS_FW_RESET);
		}
	}

	pCmdQueue->NumberOfRecoveryNodes = 0;
	
	return OK;
}

/****************************************************************************
 *                 CmdQueue_RegisterCmdCompleteGenericCB()
 ****************************************************************************
 * DESCRIPTION: Register for a call back to be called when Command Complete
 *              Occur and the CmdMboxCB was NULL
 *
 * RETURNS:None
 ****************************************************************************/
int					CmdQueue_RegisterCmdCompleteGenericCB(TI_HANDLE hCmdQueue, void *CB_Func, TI_HANDLE CB_handle)
{
	CmdQueue_T* pCmdQueue = (CmdQueue_T*)	hCmdQueue;
	
	if ((CB_Func == NULL) || (CB_handle == NULL))
	{
		WLAN_REPORT_ERROR(pCmdQueue->hReport, HAL_CTRL_MODULE_LOG, ("CmdQueue_RegisterCmdCompleteGenericCB: NULL parameter\n"));
		return NOK;
	}

	pCmdQueue->CmdCompleteGenericCB_Func = (CmdQueue_GenericCB_t)CB_Func;
	pCmdQueue->CmdCompleteGenericCB_Arg = CB_handle;

	return OK;
}

/****************************************************************************
 *                      CmdQueue_RegisterForErrorCB()
 ****************************************************************************
 * DESCRIPTION: Register for a call back to be called when an Error (Timeout)
 *              Occur
 *
 * RETURNS:None
 ****************************************************************************/
int 					CmdQueue_RegisterForErrorCB(TI_HANDLE hCmdQueue, void *CB_Func, TI_HANDLE CB_handle)
{
	CmdQueue_T* pCmdQueue = (CmdQueue_T*)	hCmdQueue;
	
	if ((CB_Func == NULL) || (CB_handle == NULL))
	{
		WLAN_REPORT_ERROR(pCmdQueue->hReport, HAL_CTRL_MODULE_LOG, ("CmdQueue_RegisterForErrorCB: NULL parameters\n"));
		return NOK;
	}

    	pCmdQueue->FailureCbHandle = CB_handle;
    	pCmdQueue->FailureCB = (CmdQueue_CB_t)CB_Func;

	return OK;
}

/****************************************************************************
 *                      CmdQueue_CmdConfigure()
 ****************************************************************************
 * DESCRIPTION: Send configure command with its information element parameter
 *
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int 					CmdQueue_CmdConfigure(TI_HANDLE hCmdQueue, void *MboxBuf,UINT32 ParamsLen)
{
	int status;
	CmdQueue_T* pCmdQueue = (CmdQueue_T*)	hCmdQueue;

	CHECK_ERROR_FLAG(pCmdQueue->ErrorFlag);
	
	status = CmdQueue_Push(pCmdQueue, CMD_CONFIGURE,
                       				(UINT8*)MboxBuf, ParamsLen,
					   		NULL, NULL, NULL);
	
	CMDQUEUE_CONVERT_RC(status);
}

/****************************************************************************
 *                      CmdQueue_CmdConfigureWithCb()
 ****************************************************************************
 * DESCRIPTION: Send configure command with its information element parameter
 *
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int					CmdQueue_CmdConfigureWithCb(TI_HANDLE hCmdQueue, void *MboxBuf, UINT32 ParamsLen,
										 					void *CB_Func, TI_HANDLE CB_handle)
{

	int status;
	CmdQueue_T* pCmdQueue = (CmdQueue_T*)	hCmdQueue;

	CHECK_ERROR_FLAG(pCmdQueue->ErrorFlag);
	
	status = CmdQueue_Push(pCmdQueue, CMD_CONFIGURE,
                       				(UINT8*)MboxBuf, ParamsLen,
					   		CB_Func, CB_handle, NULL);
	
	CMDQUEUE_CONVERT_RC(status);
}


#if 0

/*
 * NOTE: The following function may NOT be used in fully asynchronous mode.
 *       Its source code remained only for easier backword rollback
 */

/****************************************************************************
 *                      CmdQueue_CmdInterrogate()
 ****************************************************************************
 * DESCRIPTION: Send interrogate command with its information element parameter
 *
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int     				CmdQueue_CmdInterrogate(TI_HANDLE hCmdQueue, void *MboxBuf, UINT32 ParamsLen)
{
	CmdQueue_T* pCmdQueue = (CmdQueue_T*)	hCmdQueue;
	int status = OK;

	CHECK_ERROR_FLAG(pCmdQueue->ErrorFlag);

	status = CmdQueue_Push(pCmdQueue, CMD_INTERROGATE,
                       				(UINT8*)MboxBuf, ParamsLen,
					   		NULL, NULL, (UINT8*)MboxBuf);

	/* 
	cause we called an interrogate cmd without a CB function then the Cmd needs to be finished 
	in this context 
	*/
	if (status == TNETWIF_PENDING)
	{
		WLAN_REPORT_ERROR(pCmdQueue->hReport, CMD_MBOX_MODULE_LOG,
        		("CmdQueue_CmdInterrogate:Cmd INTERROGATE ,MboxBuf = 0x%x, Len = %d \n"
		 	, MboxBuf, ParamsLen));
	}

	CMDQUEUE_CONVERT_RC(status);
}
#endif


/****************************************************************************
 *                      CmdQueue_CmdInterrogateWithCb()
 ****************************************************************************
 * DESCRIPTION: Send interrogate command with its information element parameter
 *
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int					CmdQueue_CmdInterrogateWithCb(TI_HANDLE hCmdQueue, void *MboxBuf, UINT32 ParamsLen,
															void *CB_Func, TI_HANDLE CB_handle, void *CB_Buf)
{
	CmdQueue_T* pCmdQueue = (CmdQueue_T*)	hCmdQueue;
	int status;

	CHECK_ERROR_FLAG(pCmdQueue->ErrorFlag);
	
	if((CB_Func == NULL) || (CB_handle == NULL) || (CB_Buf == NULL))
	{
		WLAN_REPORT_ERROR(pCmdQueue->hReport, HAL_CTRL_MODULE_LOG, 
			("CmdQueue_CommandWithCb: NULL parameters\n"));
		return NOK;
	}

	status = CmdQueue_Push(pCmdQueue, CMD_INTERROGATE,
                       				(UINT8*)MboxBuf, ParamsLen,
					   		CB_Func, CB_handle, (UINT8*)CB_Buf);    

	CMDQUEUE_CONVERT_RC(status);
}

/***************************************************************************
 *                      CmdQueue_Command()
 ****************************************************************************
 * DESCRIPTION: Send command to the wlan hardware command mailbox
 *
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int	    				CmdQueue_Command(TI_HANDLE hCmdQueue, Command_e MboxCmdType, char *MboxBuf, UINT32 ParamsLen)
{
	CmdQueue_T* pCmdQueue = (CmdQueue_T*)	hCmdQueue;
	int status;

	CHECK_ERROR_FLAG(pCmdQueue->ErrorFlag);
	
	status = CmdQueue_Push(pCmdQueue, MboxCmdType,
                       				(UINT8*)MboxBuf, ParamsLen,
					   		NULL, NULL, NULL);

	CMDQUEUE_CONVERT_RC(status);
}

/****************************************************************************
 *                      CmdQueue_CommandWithCb()
 ****************************************************************************
 * DESCRIPTION: Send command with CB to the wlan hardware command mailbox
 *
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int	   			 	CmdQueue_CommandWithCb(TI_HANDLE hCmdQueue, Command_e MboxCmdType, void *MboxBuf, UINT32 ParamsLen, 
															void *CB_Func, TI_HANDLE CB_handle, void* CB_Buf)
{
	CmdQueue_T* pCmdQueue = (CmdQueue_T*)hCmdQueue;
	int status;

	CHECK_ERROR_FLAG(pCmdQueue->ErrorFlag);
	
	if(((CB_Func != NULL) && (CB_handle == NULL)) || ((CB_Func == NULL) && (CB_handle != NULL)))
	{
		WLAN_REPORT_ERROR(pCmdQueue->hReport, HAL_CTRL_MODULE_LOG, 
			("CmdQueue_CommandWithCb: NULL Object with none NULL CB\n"));		
		return NOK;
	}

	status = CmdQueue_Push(pCmdQueue, MboxCmdType,
                       				(UINT8*)MboxBuf, ParamsLen,
					   		CB_Func, CB_handle, (UINT8*)CB_Buf);	

	CMDQUEUE_CONVERT_RC(status);
}


/****************************************************************************
 *                      CmdQueue_Push()
 ****************************************************************************
 * DESCRIPTION: Push the command Node to the Queue with its information element parameter
 *
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: NOK OK
 ****************************************************************************/
int				CmdQueue_Push(CmdQueue_T  *pCmdQueue, Command_e  cmdType,
                       							UINT8* pParamsBuf, UINT32 paramsLen,
					   					void *CB_Func, TI_HANDLE CB_Arg, UINT8* pCB_Buf)
{

#ifdef TI_DBG

	/*
	check if Queue is Full
	*/
    	if(pCmdQueue->NumberOfCommandInQueue == CMDQUEUE_QUEUE_DEPTH)
    	{
		WLAN_REPORT_ERROR(pCmdQueue->hReport, CMD_MBOX_MODULE_LOG,
            		("CmdQueue_Push: ** ERROR ** The Queue is full\n"
			 "CmdType = %s %s , Len = %d   InfoElemId = 0x%x\n",
			 CmdQueue_GetCmdString(cmdType),
			 CmdQueue_GetIEString( cmdType,*(UINT16 *)pParamsBuf), paramsLen, *(UINT16 *)pParamsBuf));
        	return  NOK;
    	}
#endif /* TI_DBG*/

    /* initializes the last Node in the Queue with the arrgs */
    	pCmdQueue->CmdQueue[pCmdQueue->Tail].cmdType   = cmdType;
    	pCmdQueue->CmdQueue[pCmdQueue->Tail].paramsLen = paramsLen;
    	pCmdQueue->CmdQueue[pCmdQueue->Tail].CB_Func = CB_Func;
	pCmdQueue->CmdQueue[pCmdQueue->Tail].CB_Arg = CB_Arg;	
	if(cmdType == CMD_INTERROGATE)
	{
		os_memoryCopy(pCmdQueue->hOs, pCmdQueue->CmdQueue[pCmdQueue->Tail].paramsBuf, pParamsBuf, CMDQUEUE_INFO_ELEM_HEADER_LEN);
	}
	else
		os_memoryCopy(pCmdQueue->hOs, pCmdQueue->CmdQueue[pCmdQueue->Tail].paramsBuf, pParamsBuf, paramsLen);
	pCmdQueue->CmdQueue[pCmdQueue->Tail].interrogateParamsBuf = pCB_Buf;
            
	/*advance the Queue tail*/
	pCmdQueue->Tail++;
	if(pCmdQueue->Tail == CMDQUEUE_QUEUE_DEPTH)
		pCmdQueue->Tail = 0;
    
	/* update counters */
    	pCmdQueue->NumberOfCommandInQueue++;
#ifdef TI_DBG    
    	if(pCmdQueue->MaxNumberOfCommandInQueue < pCmdQueue->NumberOfCommandInQueue)
      		pCmdQueue->MaxNumberOfCommandInQueue = pCmdQueue->NumberOfCommandInQueue;	  
#endif /* TI_DBG*/
	  
    WLAN_REPORT_INFORMATION(pCmdQueue->hReport, CMDQUEUE_MODULE_LOG,
            ("CmdQueue_Push: CmdType = %s (%s(%d))\n"
			"Len = %d, NumOfCmd = %d \n",
			CmdQueue_GetCmdString(cmdType),
			(pParamsBuf) ?  CmdQueue_GetIEString(cmdType,*(UINT16 *)pParamsBuf):"",			
			(pParamsBuf) ?  *(UINT16 *)pParamsBuf:0,			
			paramsLen, pCmdQueue->NumberOfCommandInQueue));

	/*if Queue has only one command trigger the send command form Queue */	
	if (pCmdQueue->NumberOfCommandInQueue == 1)
	{
		return (CmdQueue_SM(pCmdQueue,CMDQUEUE_EVENT_RUN));
	}
	else
		return (OK);            
}

/****************************************************************************
 *                     CmdQueue_SM()
 ****************************************************************************
 * DESCRIPTION: inplement the CmdQueue SM
 *
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int                 CmdQueue_SM(CmdQueue_T* pCmdQueue,CmdQueue_SMEvents_e event)
{   
    int rc = OK;
    int breakWhile = FALSE;
    CmdQueue_CmdNode_T* pHead;
    TI_STATUS status;
    
    while(!breakWhile)
    {
            WLAN_REPORT_INFORMATION(pCmdQueue->hReport, CMDQUEUE_MODULE_LOG,
            ("CmdQueue_SM: state = %s (%d) event = %s(%d), rc = %d\n",
            StateString[pCmdQueue->State],
            pCmdQueue->State,
            EventString[event],
            event,rc));

        switch(pCmdQueue->State)
        {
            /***************************************
            CMDQUEUE_STATE_IDLE
            ***************************************/
            case CMDQUEUE_STATE_IDLE:
                switch(event)
                {
                    case CMDQUEUE_EVENT_RUN:
                        pCmdQueue->State = CMDQUEUE_STATE_SEND_CMD_v;
                        break;
                    default:
                        WLAN_REPORT_ERROR(pCmdQueue->hReport, CMD_MBOX_MODULE_LOG,
                            ("CmdQueue_SM: ** ERROR **  No such event (%d) for state CMDQUEUE_STATE_IDLE\n",event));        
                            return NOK;                     
                }
                break;

            /***************************************
            CMDQUEUE_STATE_SEND_CMD_v
            ***************************************/
            case CMDQUEUE_STATE_SEND_CMD_v:
                pHead = &pCmdQueue->CmdQueue[pCmdQueue->Head];

                WLAN_REPORT_INFORMATION(pCmdQueue->hReport, CMDQUEUE_MODULE_LOG,
                    ("CmdQueue_SM: Send Cmd: CmdType = %s(%s)\n"
            			"Len = %d, NumOfCmd = %d \n",
            			CmdQueue_GetCmdString(pHead->cmdType),
		            (pHead->paramsBuf) ?  CmdQueue_GetIEString(pHead->cmdType,*(UINT16 *)pHead->paramsBuf):"",
		            pHead->paramsLen, pCmdQueue->NumberOfCommandInQueue));
                
#ifdef TI_DBG
                pCmdQueue->CmdSendCounter++;
#endif /* TI_DBG    */
                pCmdQueue->State = CMDQUEUE_STATE_INTERROGATE_v;
                    
                    /* send the command to TNET */
                if(pHead->cmdType == CMD_INTERROGATE)
                    rc = CmdMBox_SendCmd(pCmdQueue->hCmdMBox, 
                            pHead->cmdType, 
                            pHead->paramsBuf, 
                            CMDQUEUE_INFO_ELEM_HEADER_LEN); 
                else
                    rc = CmdMBox_SendCmd(pCmdQueue->hCmdMBox, 
                            pHead->cmdType, 
                            pHead->paramsBuf, 
                            pHead->paramsLen);
                
                if(rc == TNETWIF_PENDING)
                {
                    if(pCmdQueue->SM_RC)
                    {                       
                        rc = pCmdQueue->SM_RC;
                    }
                    pCmdQueue->State = CMDQUEUE_STATE_WAIT_SEND_CMPLT;
                    breakWhile = TRUE;
                }
                else
                {                   
                    breakWhile = TRUE;
                }
                break;

            /***************************************
            CMDQUEUE_STATE_WAIT_SEND_CMPLT
            ***************************************/
            case CMDQUEUE_STATE_WAIT_SEND_CMPLT:
                switch(event)
                {
                    case CMDQUEUE_EVENT_SEND_CMPLT:
#ifdef TI_DBG    
                            pCmdQueue->CmdCompltCounter++;
#endif /* TI_DBG                        */
                        pCmdQueue->State = CMDQUEUE_STATE_INTERROGATE_v;
                        break;
                    default:
                        WLAN_REPORT_ERROR(pCmdQueue->hReport, CMD_MBOX_MODULE_LOG,
                            ("CmdQueue_SM: ** ERROR **  No such event (%d) for state CMDQUEUE_STATE_WAIT_SEND_CMPLT\n",event));     
                            return NOK;
                }
                break;

            /***************************************
            CMDQUEUE_STATE_INTERROGATE_v
            ***************************************/
            case CMDQUEUE_STATE_INTERROGATE_v:
                pHead = &pCmdQueue->CmdQueue[pCmdQueue->Head];
                    rc = CmdMBox_GetResult(pCmdQueue->hCmdMBox,
                            pHead->interrogateParamsBuf, pHead->paramsLen, (UINT32*)&status);
                    if(rc == TNETWIF_PENDING)
                    {
                        pCmdQueue->State = CMDQUEUE_STATE_WAIT_RESULT;
                        breakWhile = TRUE;
                    }
                    else
                    {
                        if(status != OK)
                        {
                            pCmdQueue->ErrorFlag = TRUE;
                            return OK;
                        }
                        pCmdQueue->State = CMDQUEUE_STATE_FINISH_v;
                    }
                break;

            /***************************************
            CMDQUEUE_STATE_WAIT_RESULT
            ***************************************/
            case CMDQUEUE_STATE_WAIT_RESULT:                
                switch(event)
                {
                    case CMDQUEUE_EVENT_RESULT_RECEIVED:
                        rc = TNETWIF_COMPLETE;
                        pCmdQueue->State = CMDQUEUE_STATE_FINISH_v;
                        break;
                    default:
                        WLAN_REPORT_ERROR(pCmdQueue->hReport, CMD_MBOX_MODULE_LOG,
                            ("CmdQueue_SM: ** ERROR **  No such event (%d) for state CMDQUEUE_STATE_WAIT_RESULT\n",event));     
                            return NOK;
                }
                break;

            /***************************************
            CMDQUEUE_STATE_FINISH_v
            ***************************************/
            case CMDQUEUE_STATE_FINISH_v:
            {
                Command_e cmdType;
                UINT16        uParam;
                void *fCb, *hCb, *pCb;

                pHead = &pCmdQueue->CmdQueue[pCmdQueue->Head];

                /* Keep callback parameters in temporary variables */
                cmdType = pHead->cmdType;
                uParam  = *(UINT16 *)pHead->paramsBuf;
                fCb = pHead->CB_Func;
                hCb = pHead->CB_Arg;
                pCb = pHead->interrogateParamsBuf;

                /* 
                 * Delete the command from the queue before calling a callback 
                 * because there may be nested calls inside a callback
                 */
                pCmdQueue->Head ++;
                if (pCmdQueue->Head >= CMDQUEUE_QUEUE_DEPTH)
                    pCmdQueue->Head = 0;                
                pCmdQueue->NumberOfCommandInQueue --;                

                /* Check if queue is empty to send the next command */
                if (pCmdQueue->NumberOfCommandInQueue > 0)               
                {
                    pCmdQueue->SM_RC = rc;
                    pCmdQueue->State = CMDQUEUE_STATE_SEND_CMD_v;
                }
                else	
                {   
                    pCmdQueue->SM_RC = 0;
                    pCmdQueue->State = CMDQUEUE_STATE_IDLE;
                    breakWhile = TRUE;
                }

                /*
                 * Call the user callback after deleting the command from the queue 
                 * because there may be nested calls inside a callback
                 */
                status = CmdMBox_GetStatus(pCmdQueue->hCmdMBox);
                if (fCb)
                {   
                    if(pCb)
                    {
                        ((CmdQueue_InterrogateCB_t)fCb) (hCb, status, pCb); 
                    }
                    else
                    {
                        ((CmdQueue_CB_t)fCb) (hCb, status);
                    }
                }
                else
                {
                    /* Call the generic callback */
                    if (pCmdQueue->CmdCompleteGenericCB_Func)
                    {
                        pCmdQueue->CmdCompleteGenericCB_Func (pCmdQueue->CmdCompleteGenericCB_Arg,                             
                                                              cmdType, 
                                                              uParam, 
                                                              status);
                    }
                }
            }               
                break;

            default:
                WLAN_REPORT_ERROR(pCmdQueue->hReport, CMD_MBOX_MODULE_LOG,
                    ("CmdQueue_SM: ** ERROR **  No such state (%d)\n",pCmdQueue->State));       
                    return NOK;
        }
    }
            WLAN_REPORT_INFORMATION(pCmdQueue->hReport, CMDQUEUE_MODULE_LOG,
                             ("CmdQueue_SM: rc = %d\n",rc));

    return rc;
}

/*******************************************************************************************************
 *                     CmdQueue_SendCmplt()
 ******************************************************************************************************
 * DESCRIPTION: This function is the CB from the CmdMBox that will issue the "SendCmplt" event to 
 * 				the CmdQueue SM. Indicates that the Cmd was transferred to the FW.
 *
 * RETURNS: OK
 *************************************************************************************************/
int 			CmdQueue_SendCmplt(TI_HANDLE hCmdQueue)
{
	CmdQueue_T* pCmdQueue = (CmdQueue_T*)hCmdQueue;
    
	/* call the SM for further execution */
	return CmdQueue_SM(pCmdQueue,CMDQUEUE_EVENT_SEND_CMPLT);
}

/*******************************************************************************************************
 *                     CmdQueue_ResultReceived()
 ******************************************************************************************************
 * DESCRIPTION: This function is the CB from the CmdMBox that will issue the "ResultReceived" 
 * 				event to the CmdQueue SM. Indicates that the Cmd's results were read from 
 *				the FW.
 *
 * RETURNS: OK
 *************************************************************************************************/
int					CmdQueue_ResultReceived(TI_HANDLE hCmdQueue, UINT32 status)
{
    	CmdQueue_T* pCmdQueue = (CmdQueue_T*)hCmdQueue;

	/* call the SM for further execution */
	return CmdQueue_SM(pCmdQueue,CMDQUEUE_EVENT_RESULT_RECEIVED);
}

/****************************************************************************
 *                      CmdQueue_TimeOut()
 ****************************************************************************
 * DESCRIPTION: Called when a command timeout occur
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ******************************************************************************/
int 			CmdQueue_Error(TI_HANDLE hCmdQueue)
{
	
	CmdQueue_T* pCmdQueue = (CmdQueue_T*)hCmdQueue;		

#ifdef TI_DBG
	CmdQueue_CmdNode_T* pHead = &pCmdQueue->CmdQueue[pCmdQueue->Head];	
	UINT32 TimeStamp = os_timeStampMs(pCmdQueue->hOs);
	
   	WLAN_REPORT_ERROR(pCmdQueue->hReport, CMD_MBOX_MODULE_LOG,
    		("CmdQueue_Error: **ERROR**  Command Occured \n"
	     	"                                        Cmd = %s %s , Len = %d \n "
	   		"                                        NumOfCmd = %d \n"
			"                                        MAC TimeStamp on timeout = %d\n ",
    		CmdQueue_GetCmdString(pHead->cmdType), 
    		CmdQueue_GetIEString(pHead->cmdType, *(UINT16 *)pHead->paramsBuf),
    		pHead->paramsLen, 
    		pCmdQueue->NumberOfCommandInQueue, 
    		TimeStamp));
#endif

	/* Print The command that was sent before the timeout occur */
	CmdQueue_PrintHistory(pCmdQueue, CMDQUEUE_HISTORY_DEPTH);

	/* preform Recovery */
#ifdef TI_DBG
	if(pCmdQueue->FailureCB)
#endif		
    	pCmdQueue->FailureCB(pCmdQueue->FailureCbHandle,NOK);

	return OK;

	}

/****************************************************************************
 *                      CmdQueue_Print()
 ****************************************************************************
 * DESCRIPTION: 
 *
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: 
 ****************************************************************************/
void					CmdQueue_Print(TI_HANDLE hCmdQueue)
{
	CmdQueue_T* pCmdQueue = (CmdQueue_T*)hCmdQueue;	
	
    	WLAN_REPORT_REPLY(pCmdQueue->hReport, CMD_MBOX_MODULE_LOG,
       	("------------- CmdQueue Queue -------------------\n"));
    
    	WLAN_REPORT_REPLY(pCmdQueue->hReport, CMD_MBOX_MODULE_LOG,
        	("CmdQueue_Print:The Max NumOfCmd in Queue was = %d\n",
        	pCmdQueue->MaxNumberOfCommandInQueue));
    	WLAN_REPORT_REPLY(pCmdQueue->hReport, CMD_MBOX_MODULE_LOG,
        	("CmdQueue_Print:The Current NumOfCmd in Queue = %d\n",
        	pCmdQueue->NumberOfCommandInQueue));
#ifdef TI_DBG
    	WLAN_REPORT_REPLY(pCmdQueue->hReport, CMD_MBOX_MODULE_LOG,
        	("CmdQueue_Print:The Total number of Cmd send from Queue= %d\n",
        	pCmdQueue->CmdSendCounter));
    	WLAN_REPORT_REPLY(pCmdQueue->hReport, CMD_MBOX_MODULE_LOG,
        	("CmdQueue_Print:The Total number of Cmd Completed interrupt= %d\n",
        	pCmdQueue->CmdCompltCounter));
#endif

    	CmdQueue_PrintQueue(pCmdQueue);
}

/****************************************************************************
 *                      CmdQueue_PrintQueue()
 ****************************************************************************
 * DESCRIPTION: 
 *
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: 
 ****************************************************************************/
void				CmdQueue_PrintQueue(CmdQueue_T  *pCmdQueue)
{
	int CurrentCmdindex;
    	int first  = pCmdQueue->Head;
	CmdQueue_CmdNode_T* pHead;
    	int NumberOfCommand = pCmdQueue->NumberOfCommandInQueue;

	WLAN_OS_REPORT(("CmdQueue_PrintQueue \n"));

    	for(CurrentCmdindex = 0 ; CurrentCmdindex < NumberOfCommand ; CurrentCmdindex++)
    	{
		pHead = &pCmdQueue->CmdQueue[first];
#ifdef TI_DBG
        	WLAN_OS_REPORT(("Cmd index %d CmdType = %s %s, Len = %d, Place in Queue = %d \n",
            		CurrentCmdindex, 
            		CmdQueue_GetCmdString(pHead->cmdType),
            		CmdQueue_GetIEString(pHead->cmdType, (((pHead->cmdType == CMD_INTERROGATE)||(pHead->cmdType == CMD_CONFIGURE)) ? *(UINT16 *)pHead->paramsBuf : 0)),
            		pHead->paramsLen, 
            		first));    
#else
		WLAN_OS_REPORT(("Cmd index %d CmdType = %d %d, Len = %d, Place in Queue = %d \n",
            		CurrentCmdindex, 
            		pHead->cmdType,
            		(((pHead->cmdType == CMD_INTERROGATE)||(pHead->cmdType == CMD_CONFIGURE)||
                      (pHead->cmdType == CMD_READ_MEMORY)||(pHead->cmdType == CMD_WRITE_MEMORY)) ? *(UINT16 *)pHead->paramsBuf : 0),
            		pHead->paramsLen, 
            		first));    
#endif

        	first++;
		if(first == CMDQUEUE_QUEUE_DEPTH)
			first = 0;
    	}
}

/****************************************************************************
 *                      CmdQueue_PrintHistory()
 ****************************************************************************
 * DESCRIPTION: print the last command according to a value
 *
 * INPUTS:  NunOfCmd : the number of the last command to print 
 *
 ****************************************************************************/
void					CmdQueue_PrintHistory(TI_HANDLE hCmdQueue, int NunOfCmd)
{
	CmdQueue_T* pCmdQueue = (CmdQueue_T*)hCmdQueue;	
	int CurrentCmdindex;
    	int first  = pCmdQueue->Head;
	CmdQueue_CmdNode_T* pHead ;

    WLAN_OS_REPORT(("--------------- CmdQueue_PrintHistory of %d -------------------\n",NunOfCmd));
	
	for(CurrentCmdindex = 0 ; CurrentCmdindex < NunOfCmd ; CurrentCmdindex++)
	{
		pHead  =  &pCmdQueue->CmdQueue[first];

#ifdef TI_DBG
	    	WLAN_OS_REPORT(("Cmd index %d CmdType = %s %s, Len = %d, Place in Queue = %d \n",
            		CurrentCmdindex, 
            		CmdQueue_GetCmdString(pHead->cmdType),
            		CmdQueue_GetIEString(pHead->cmdType, (((pHead->cmdType == CMD_INTERROGATE)||(pHead->cmdType == CMD_CONFIGURE)) ? *(UINT16 *)pHead->paramsBuf : 0)),
            		pHead->paramsLen, 
            		first));
#else
		WLAN_OS_REPORT(("Cmd index %d CmdType = %d %d, Len = %d, Place in Queue = %d \n",
            		CurrentCmdindex, 
            		pHead->cmdType,
            		(((pHead->cmdType == CMD_INTERROGATE)||(pHead->cmdType == CMD_CONFIGURE)||
                      (pHead->cmdType == CMD_READ_MEMORY)||(pHead->cmdType == CMD_WRITE_MEMORY)) ? *(UINT16 *)pHead->paramsBuf : 0),
            		pHead->paramsLen, 
            		first));
#endif

		if(first == 0)
			first = CMDQUEUE_QUEUE_DEPTH-1;
		else
			first--;
	}

	WLAN_OS_REPORT(("-----------------------------------------------------------------------\n"));

}

/****************************************************************************
 *                      CmdQueue_GetMaxNumberOfCommands()
 ****************************************************************************
 * DESCRIPTION: returns maximum number of commands (ever) in CmdQueue queue
 *
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: maximum number of commands (ever) in mailbox queue
 ****************************************************************************/
int 					CmdQueue_GetMaxNumberOfCommands (TI_HANDLE hCmdQueue)
{
	CmdQueue_T* pCmdQueue = (CmdQueue_T*)hCmdQueue;

	return (pCmdQueue->MaxNumberOfCommandInQueue);
}

#ifdef REPORT_LOG

/****************************************************************************
 *                      CmdQueue_GetCmdString()
 ****************************************************************************
 * DESCRIPTION: 
 *
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: 
 ****************************************************************************/
char* 			CmdQueue_GetCmdString(int MboxCmdType)
{
	switch (MboxCmdType)
	{
		case 0: return "CMD_RESET";
		case 1: return "CMD_INTERROGATE"; 
	 	case 2: return "CMD_CONFIGURE";
	    	case 3: return "CMD_ENABLE_RX";
		case 4: return "CMD_ENABLE_TX";
		case 5: return "CMD_DISABLE_RX";
	    	case 6: return "CMD_DISABLE_TX";	
		case 8: return "CMD_SCAN";
		case 9: return "CMD_STOP_SCAN";	
	    	case 10: return "CMD_VBM";
		case 11: return "CMD_START_JOIN";	
		case 12: return "CMD_SET_KEYS";	
		case 13: return "CMD_READ_MEMORY";	
	    	case 14: return "CMD_WRITE_MEMORY";
		case 19: return "CMD_BEACON";
		case 20: return "CMD_PROBE_RESP";	
		case 21: return "CMD_NULL_DATA";	
	    	case 22: return "CMD_PROBE_REQ";	
		case 23: return "CMD_TEST";		
		case 27: return "CMD_ENABLE_RX_PATH";
		case 28: return "CMD_NOISE_HIST";	
	    	case 29: return "CMD_RX_RESET";	
		case 30: return "CMD_PS_POLL";	
		case 31: return "CMD_QOS_NULL_DATA";	
		case 32: return "CMD_LNA_CONTROL";	
		case 33: return "CMD_SET_BCN_MODE";	
		case 34: return "CMD_MEASUREMENT";	
		case 35: return "CMD_STOP_MEASUREMENT";
		case 36: return "CMD_DISCONNECT";		
		case 37: return "CMD_SET_PS_MODE";		
		case 38: return "CMD_CHANNEL_SWITCH";	
		case 39: return "CMD_STOP_CHANNEL_SWICTH";
		case 40: return "CMD_AP_DISCOVERY";
		case 41: return "CMD_STOP_AP_DISCOVERY";
		case 42: return "CMD_SPS_SCAN";			
		case 43: return "CMD_STOP_SPS_SCAN";		
		case 45: return "CMD_HEALTH_CHECK";		
		default: return " *** Error No Such CMD **** ";
	}
};

/****************************************************************************
 *                      CmdQueue_GetErrorString()
 ****************************************************************************
 * DESCRIPTION: 
 *
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: 
 ****************************************************************************/
char*            CmdQueue_GetErrorString(CommandStatus_e MboxError)
{
    switch (MboxError)
    {
    case CMD_MAILBOX_IDLE:
        return "CMD_MAILBOX_IDLE";
        /* break; to avoid compilation warning */

    case CMD_STATUS_SUCCESS:
        return "CMD_STATUS_SUCCESS";
        /* break; to avoid compilation warning */

    case CMD_STATUS_UNKNOWN_CMD:
        return "CMD_STATUS_UNKNOWN_CMD";
        /* break; to avoid compilation warning */

    case CMD_STATUS_UNKNOWN_IE:
        return "CMD_STATUS_UNKNOWN_IE";
        /* break; to avoid compilation warning */

    case CMD_STATUS_RX_BUSY:
        return "CMD_STATUS_RX_BUSY";
        /* break; to avoid compilation warning */

    case CMD_STATUS_INVALID_PARAM:
        return "CMD_STATUS_INVALID_PARAM";
        /* break; to avoid compilation warning */

    case CMD_STATUS_TEMPLATE_TOO_LARGE:
        return "CMD_STATUS_TEMPLATE_TOO_LARGE";
        /* break; to avoid compilation warning */

    case CMD_STATUS_OUT_OF_MEMORY:
        return "CMD_STATUS_OUT_OF_MEMORY";
        /* break; to avoid compilation warning */

    case CMD_STATUS_STA_TABLE_FULL:
        return "CMD_STATUS_STA_TABLE_FULL";
        /* break; to avoid compilation warning */

    case CMD_STATUS_RADIO_ERROR:
        return "CMD_STATUS_RADIO_ERROR";
        /* break; to avoid compilation warning */

    case CMD_STATUS_WRONG_NESTING:
        return "CMD_STATUS_WRONG_NESTING";
        /* break; to avoid compilation warning */

    case CMD_STATUS_TIMEOUT:
        return "CMD_STATUS_TIMEOUT";
        /* break; to avoid compilation warning */

    case CMD_STATUS_FW_RESET:
        return "CMD_STATUS_FW_RESET";
        /* break; to avoid compilation warning */

    default:
        return "Unrecognized error code";
        /* break; to avoid compilation warning */
    }
}


/****************************************************************************
 *                      CmdQueue_GetIEString()
 ****************************************************************************
 * DESCRIPTION: 
 *
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: 
 ****************************************************************************/
char* 			CmdQueue_GetIEString(int MboxCmdType, UINT16 Id)
{
	if( MboxCmdType== CMD_INTERROGATE || MboxCmdType == CMD_CONFIGURE)	
	{
		switch (Id)
		{
		case ACX_WAKE_UP_CONDITIONS: 		return " (ACX_WAKE_UP_CONDITIONS)";
		case ACX_MEM_CFG: 					return " (ACX_MEM_CFG)";                 
		case ACX_SLOT: 						return " (ACX_SLOT) ";                    
		case ACX_QUEUE_HEAD: 				return " (ACX_QUEUE_HEAD)";	
		case ACX_AC_CFG: 					return " (ACX_AC_CFG) ";                  
		case ACX_MEM_MAP: 					return " (ACX_MEM_MAP)";
		case ACX_AID: 						return " (ACX_AID)";
		case ACX_RADIO_PARAM: 				return " (ACX_RADIO_PARAM)";          
		case ACX_CFG: 						return " (ACX_CFG) ";                  
        case ACX_FW_REV: 					return " (ACX_FW_REV) ";                  
        case ACX_FCS_ERROR_CNT: 			return " (ACX_FCS_ERROR_CNT) ";               
        case ACX_MEDIUM_USAGE: 				return " (ACX_MEDIUM_USAGE) ";                  
		case ACX_RX_CFG: 					return " (ACX_RX_CFG) ";                  
		case ACX_TX_QUEUE_CFG: 				return " (ACX_TX_QUEUE_CFG) "; 
		case ACX_BSS_IN_PS: 				return " (ACX_BSS_IN_PS) ";
		case ACX_STATISTICS: 				return " (ACX_STATISTICS) ";
		case ACX_FEATURE_CFG: 				return " (ACX_FEATURE_CFG) ";                    
		case ACX_MISC_CFG: 					return " (ACX_MISC_CFG) ";               
		case ACX_TID_CFG: 					return " (ACX_TID_CFG) ";                    
		case ACX_CAL_ASSESSMENT: 			return " (ACX_CAL_ASSESSMENT) ";            
		case ACX_BEACON_FILTER_OPT: 		return " (ACX_BEACON_FILTER_OPT) ";             			      											  
		case ACX_LOW_RSSI: 					return " (ACX_LOW_RSSI)";              
        case ACX_NOISE_HIST: 				return " (ACX_NOISE_HIST)";           
		case ACX_HDK_VERSION: 				return " (ACX_HDK_VERSION)";         
		case ACX_PD_THRESHOLD: 				return " (ACX_PD_THRESHOLD) ";                 
		case ACX_DATA_PATH_PARAMS: 			return " (ACX_DATA_PATH_PARAMS) ";                
   	case ACX_CCA_THRESHOLD: 				return " (ACX_CCA_THRESHOLD)";            
		case ACX_EVENT_MBOX_MASK: 			return " (ACX_EVENT_MBOX_MASK) ";
#ifdef FW_RUNNING_AS_AP                            
		case ACX_DTIM_PERIOD: 				return " (ACX_DTIM_PERIOD) ";            
#else
		case ACX_WR_TBTT_AND_DTIM: 			return " (ACX_WR_TBTT_AND_DTIM) ";  
#endif
		case ACX_ACI_OPTION_CFG: 			return " (ACX_ACI_OPTION_CFG) ";                
        case ACX_GPIO_CFG: 					return " (ACX_GPIO_CFG) ";  
        case ACX_GPIO_SET: 					return " (ACX_GPIO_SET) ";  
		case ACX_PM_CFG: 					return " (ACX_PM_CFG) ";
		case ACX_CONN_MONIT_PARAMS: 		return " (ACX_CONN_MONIT_PARAMS) ";
		case ACX_AVERAGE_RSSI: 				return " (ACX_AVERAGE_RSSI) ";
		case ACX_CONS_TX_FAILURE: 			return " (ACX_CONS_TX_FAILURE) ";
		case ACX_BCN_DTIM_OPTIONS: 			return " (ACX_BCN_DTIM_OPTIONS) ";                             
		case ACX_SG_ENABLE: 				return " (ACX_SG_ENABLE) ";                                       
		case ACX_SG_CFG: 					return " (ACX_SG_CFG) ";                                       
		case ACX_ANTENNA_DIVERSITY_CFG: 	return " (ACX_ANTENNA_DIVERSITY_CFG) ";                                      
		case ACX_LOW_SNR: 					return " (ACX_LOW_SNR) ";
		case ACX_BEACON_FILTER_TABLE: 		return " (ACX_BEACON_FILTER_TABLE) ";
		case ACX_ARP_IP_FILTER: 			return " (ACX_ARP_IP_FILTER) ";
		case ACX_ROAMING_STATISTICS_TBL:	return " (ACX_ROAMING_STATISTICS_TBL) ";  
		case ACX_RATE_POLICY: 				return " (ACX_RATE_POLICY) ";  
		case ACX_CTS_PROTECTION: 			return " (ACX_CTS_PROTECTION) ";  
		case ACX_SLEEP_AUTH: 				return " (ACX_SLEEP_AUTH) ";  
		case ACX_PREAMBLE_TYPE: 			return " (ACX_PREAMBLE_TYPE) ";  
		case ACX_ERROR_CNT: 				return " (ACX_ERROR_CNT) ";  
		case ACX_FW_GEN_FRAME_RATES: 		return " (ACX_FW_GEN_FRAME_RATES) ";  
		case ACX_IBSS_FILTER: 				return " (ACX_IBSS_FILTER) ";  
		case ACX_SERVICE_PERIOD_TIMEOUT:	return " (ACX_SERVICE_PERIOD_TIMEOUT) ";  
		case ACX_TSF_INFO: 					return " (ACX_TSF_INFO) ";  
		case ACX_CONFIG_PS_WMM: 			return " (ACX_CONFIG_PS_WMM) "; 
		case ACX_ENABLE_RX_DATA_FILTER: 	return " (ACX_ENABLE_RX_DATA_FILTER) ";
		case ACX_SET_RX_DATA_FILTER: 		return " (ACX_SET_RX_DATA_FILTER) ";
		case ACX_GET_DATA_FILTER_STATISTICS:return " (ACX_GET_DATA_FILTER_STATISTICS) ";
		case ACX_POWER_LEVEL_TABLE: 		return " (ACX_POWER_LEVEL_TABLE) ";
		case ACX_BET_ENABLE: 				return " (ACX_BET_ENABLE) ";
		case DOT11_STATION_ID: 				return " (DOT11_STATION_ID) ";
		case DOT11_RX_MSDU_LIFE_TIME: 		return " (DOT11_RX_MSDU_LIFE_TIME) ";
		case DOT11_CUR_TX_PWR: 				return " (DOT11_CUR_TX_PWR) ";
		case DOT11_DEFAULT_KEY: 			return " (DOT11_DEFAULT_KEY) ";
		case DOT11_RX_DOT11_MODE: 			return " (DOT11_RX_DOT11_MODE) ";
		case DOT11_RTS_THRESHOLD: 			return " (DOT11_RTS_THRESHOLD) ";
		case DOT11_GROUP_ADDRESS_TBL: 		return " (DOT11_GROUP_ADDRESS_TBL) ";  
               
		default:	return " *** Error No Such IE **** ";
		}
	}
	return "";
}
#endif /* REPORT_LOG */

