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


/*********************************************************************************/
/*                                                                                */
/*   MODULE:  CmdQueue.h                                                */
/*   PURPOSE: CmdQueue internal H file											  */
/*                                                                                */
/**********************************************************************************/
#ifndef _CMDQUEUE_H_
#define _CMDQUEUE_H_

#include "whalCommon.h"
#include "whalHwDefs.h"

/*****************************************************************************
 **         Defines	                                                       **
 *****************************************************************************/
#define CMDQUEUE_QUEUE_DEPTH          50
#define CMDQUEUE_HISTORY_DEPTH        5
#define CMDQUEUE_INFO_ELEM_HEADER_LEN 4

#define CMDQUEUE_CONVERT_RC(rc) \
	if((rc == TNETWIF_OK)||(rc == TNETWIF_COMPLETE)||(rc == TNETWIF_PENDING)||(rc == OK)) \
		return OK; \
	else \
		return NOK \

#define CHECK_ERROR_FLAG(flag) \
	if(flag == TRUE) \
		return OK; \

/*****************************************************************************
 **         Enums                                                    **
 *****************************************************************************/
typedef enum 
{
	CMDQUEUE_EVENT_RUN,
	CMDQUEUE_EVENT_SEND_CMPLT,
	CMDQUEUE_EVENT_RESULT_RECEIVED,
	CMDQUEUE_EVENT_NUM,
} CmdQueue_SMEvents_e;

typedef enum 
{
	CMDQUEUE_STATE_IDLE,
	CMDQUEUE_STATE_SEND_CMD_v,
	CMDQUEUE_STATE_WAIT_SEND_CMPLT,
	CMDQUEUE_STATE_INTERROGATE_v,
	CMDQUEUE_STATE_WAIT_RESULT,
	CMDQUEUE_STATE_FINISH_v,
	CMDQUEUE_STATE_NUM,
} CmdQueue_SMStates_e;


/*****************************************************************************
 **         Structures                                                      **
 *****************************************************************************/

/*  CmdQueue Node */
typedef struct 
{
	Command_e    		cmdType;		/* Command Type Config/interrogat ... */
	UINT32				paramsLen; 
	void*				CB_Func;
	TI_HANDLE			CB_Arg;
	UINT8				paramsBuf[MAX_CMD_PARAMS]; /* param for config */
	UINT8*				interrogateParamsBuf; /* A returned value Buffer */	
}CmdQueue_CmdNode_T;

/*  Saved CallBack Node In case of Recovery*/
typedef struct 
{
	void*				CB_Func;
	TI_HANDLE			CB_Arg;
	UINT8*				interrogateParamsBuf; /* A returned value Buffer */		
}CmdQueue_RecoveryNode_T;

/* MailBox Queue */

typedef struct _CmdQueue_T
{	
	/* handles */
	TI_HANDLE		    		hOs;
	TI_HANDLE	        		hReport;
	TI_HANDLE           			hCmdMBox;

	/* SM */
	CmdQueue_SMStates_e		State;
	CmdQueue_GenericCB_t 	CmdCompleteGenericCB_Func;
	TI_HANDLE				CmdCompleteGenericCB_Arg;
	CmdQueue_CB_t			FailureCB;
	TI_HANDLE				FailureCbHandle;

	/* queues */
	CmdQueue_CmdNode_T		CmdQueue[CMDQUEUE_QUEUE_DEPTH];	
	CmdQueue_RecoveryNode_T	RecoveryQueue[CMDQUEUE_QUEUE_DEPTH];

	/* CmdQueue indexes & counters */
	int  						Head;
	int		    	  			Tail;
	int				    		NumberOfCommandInQueue;
	int						MaxNumberOfCommandInQueue;
	int						NumberOfRecoveryNodes;	
#ifdef TI_DBG
	UINT32					CmdSendCounter;
	UINT32					CmdCompltCounter;
#endif

	int 						SM_RC;
	/* error handling */
	int 						ErrorFlag;
}CmdQueue_T; 

/*****************************************************************************
 **         Internal functions definitions                                  **
 *****************************************************************************/
int				CmdQueue_SM(CmdQueue_T* pCmdQueue,CmdQueue_SMEvents_e event);
int				CmdQueue_Push(CmdQueue_T  *pCmdQueue, Command_e  cmdType,
                       							UINT8* pParamsBuf, UINT32 paramsLen,
					   					void *CB_Func, TI_HANDLE CB_Arg, UINT8* pCB_Buf);
void			CmdQueue_PrintQueue(CmdQueue_T  *pCmdQueue);

#endif
