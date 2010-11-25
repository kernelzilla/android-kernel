
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
/*   MODULE:  CmdMBox.h                                                       */
/*   PURPOSE: Acx mailbox object api                                              */
/*                                                                                */
/**********************************************************************************/
#ifndef _CMDMBOX_H_
#define _CMDMBOX_H_

#include "whalCommon.h"
#include "whalHwDefs.h"

/*****************************************************************************
 **         Defines                                                        **
 *****************************************************************************/
 /* wait for a Mail box command to complete  */
#define CMDMBOX_WAIT_TIMEOUT    500 /* ms */

#define CMDMBOX_HEADER_LEN 4
#define CMDMBOX_INFO_ELEM_HEADER_LEN 4
#define CMDMBOX_WAIT_CMPLT_STALL_TIME 50 /* us */
#define CMDMBOX_US_TO_MS 1000

/*****************************************************************************
 **         Enums                                                    **
 *****************************************************************************/
typedef enum 
{
    CMDMBOX_EVENT_SEND_CMD,
    CMDMBOX_EVENT_CMD_CMPLT,
    CMDMBOX_EVENT_BUS_READY,
    CMDMBOX_EVENT_TXN_CMPLT,
    CMDMBOX_EVENT_GET_RESULT,
    CMDMBOX_EVENT_NUM,
} CmdMBox_SMEvents_e;

typedef enum 
{
    CMDMBOX_STATE_SENDCMD_NORMAL_IDLE = 0,
    CMDMBOX_STATE_SENDCMD_NORMAL_WAIT_BUS,
    CMDMBOX_STATE_SENDCMD_NORMAL_WAIT_TXN_BUF,
    CMDMBOX_STATE_SENDCMD_NORMAL_WRITE_TRIG_v,
    CMDMBOX_STATE_SENDCMD_NORMAL_WAIT_TXN_TRIG,
} CmdMBox_SMStates_SendCmd_Normal_e;

typedef enum 
{
    CMDMBOX_STATE_SENDCMD_BLOCKING_IDLE = 10,
    CMDMBOX_STATE_SENDCMD_BLOCKING_WAIT_BUS,
    CMDMBOX_STATE_SENDCMD_BLOCKING_WAIT_TXN_BUF,
    CMDMBOX_STATE_SENDCMD_BLOCKING_WRITE_TRIG_v,
    CMDMBOX_STATE_SENDCMD_BLOCKING_WAIT_TXN_TRIG,
    CMDMBOX_STATE_SENDCMD_BLOCKING_POLL_CMPLT_v,
    CMDMBOX_STATE_SENDCMD_BLOCKING_FINISH_v,
} CmdMBox_SMStates_SendCmd_Blocking_e;

typedef enum 
{
    CMDMBOX_STATE_GETRESULT_NORMAL_IDLE = 20,
    CMDMBOX_STATE_GETRESULT_NORMAL_WAIT_TXN,
} CmdMBox_SMStates_GetResult_Normal_e;

typedef enum 
{
    CMDMBOX_STATE_GETRESULT_BLOCKING_IDLE = 30,
    CMDMBOX_STATE_GETRESULT_BLOCKING_WAIT_TXN,
    CMDMBOX_STATE_NUM,
} CmdMBox_SMStates_GetResult_Blocking_e;

/*****************************************************************************
 **         Types                                                    **
 *****************************************************************************/
typedef struct _CmdMBox_T CmdMBox_T; 
typedef int (*SM_Func_t)(CmdMBox_T* pCmdMBox, CmdMBox_SMEvents_e event);


/*****************************************************************************
 **         Structures                                                      **
 *****************************************************************************/

struct _CmdMBox_T
{   
    /* handles */
    TI_HANDLE               hOs;
    TI_HANDLE               hReport;
    TI_HANDLE               hTNETWIF;
    TI_HANDLE               hFwEvent;
    TI_HANDLE               hTimer; 
    TI_HANDLE               hCmdQueue;

    /* SM */
    CmdMBox_SMStates_GetResult_Normal_e     GetResultNormal_State;
    CmdMBox_SMStates_GetResult_Blocking_e   GetResultBlocking_State;
    CmdMBox_SMStates_SendCmd_Normal_e   SendCmdNormal_State;
    CmdMBox_SMStates_SendCmd_Blocking_e SendCmdBlocking_State;
    SM_Func_t               ActiveSM;
    SM_Func_t               SendCmdSM;
    SM_Func_t               GetResultSM;    

    /* HW params */
    /* use a struct to read buffers from the bus - used for extra bytes reserving */
    PADDING (Command_t  HW_CmdMBox)
    
    UINT32                  CmdMBox_FW_address;
    UINT32                  CmdLen;

    UINT8*                  GetResult_ParamsBuf;
    UINT32                  GetResult_ParamsLen;

    BOOLEAN                 useOpt;
    
    TNETWIF_callback_t      fCb;
    TI_HANDLE               hCb;
};

/*****************************************************************************
 **         Internal functions definitions                                  **
 *****************************************************************************/
int             CmdMBox_SM_GetResultNormal(CmdMBox_T* pCmdMBox, CmdMBox_SMEvents_e event);
int             CmdMBox_SM_GetResultBlocking(CmdMBox_T* pCmdMBox, CmdMBox_SMEvents_e event);
int             CmdMBox_SM_SendCmdNormal(CmdMBox_T* pCmdMBox, CmdMBox_SMEvents_e event);
int             CmdMBox_SM_SendCmdBlocking(CmdMBox_T* pCmdMBox, CmdMBox_SMEvents_e event);

/*****************************************************************************
 **         CB functions definitions                                  **
 *****************************************************************************/
void            CmdMBox_TimeOut(TI_HANDLE hCmdMBox);
void                CmdMBox_TxnCmplt(TI_HANDLE hCmdMBox, UINT8 module_id ,TI_STATUS status);
void                CmdMBox_BusReady(TI_HANDLE hCmdMBox, UINT8 module_id ,TI_STATUS status);

#endif
