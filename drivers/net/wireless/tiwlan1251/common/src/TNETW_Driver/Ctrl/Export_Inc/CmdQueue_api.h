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
/*   MODULE:  CmdQueue_api.h                                                */
/*   PURPOSE: CmdQueue api											  */
/*                                                                                */
/**********************************************************************************/
#ifndef _CMDQUEUE_API_H_
#define _CMDQUEUE_API_H_

#include "whalCommon.h"
#include "public_commands.h"

/*****************************************************************************
 **         Defines	                                                       **
 *****************************************************************************/

/*****************************************************************************
 **         Enums                                                    **
 *****************************************************************************/

/*****************************************************************************
 **         Structures                                                      **
 *****************************************************************************/

/*****************************************************************************
 **         Types                                                      **
 *****************************************************************************/

/*****************************************************************************
 **         APIs definitions                                  **
 *****************************************************************************/

TI_HANDLE 			CmdQueue_Create(TI_HANDLE hOS);
int					CmdQueue_Destroy(TI_HANDLE hCmdQueue);
int	 				CmdQueue_Config(TI_HANDLE hCmdQueue, TI_HANDLE hCmdMBox,
                         								TI_HANDLE hReport);
int					CmdQueue_StartReconfig(TI_HANDLE hCmdQueue);
int					CmdQueue_EndReconfig(TI_HANDLE hCmdQueue);
int					CmdQueue_RegisterCmdCompleteGenericCB(TI_HANDLE hCmdQueue, void *CB_Func, TI_HANDLE CB_handle);
int 					CmdQueue_RegisterForErrorCB(TI_HANDLE hCmdQueue, void *CB_Func, TI_HANDLE CB_handle);
int 					CmdQueue_CmdConfigure(TI_HANDLE hCmdQueue, void *MboxBuf,UINT32 ParamsLen);
int					CmdQueue_CmdConfigureWithCb(TI_HANDLE hCmdQueue, void *MboxBuf, UINT32 ParamsLen,
										 					void *CB_Func, TI_HANDLE CB_handle);
int     				CmdQueue_CmdInterrogate(TI_HANDLE hCmdQueue, void *MboxBuf, UINT32 ParamsLen);
int					CmdQueue_CmdInterrogateWithCb(TI_HANDLE hCmdQueue, void *MboxBuf, UINT32 ParamsLen,
															void *CB_Func, TI_HANDLE CB_handle, void *CB_Buf);
int	    				CmdQueue_Command(TI_HANDLE hCmdQueue, Command_e MboxCmdType, char *MboxBuf, UINT32 ParamsLen);
int	   			 	CmdQueue_CommandWithCb(TI_HANDLE hCmdQueue, Command_e MboxCmdType, void *MboxBuf, UINT32 ParamsLen, 
															void *CB_Func, TI_HANDLE CB_handle, void* CB_Buf);
int 					CmdQueue_GetMaxNumberOfCommands (TI_HANDLE hCmdQueue);
void					CmdQueue_Print(TI_HANDLE hCmdQueue);
void					CmdQueue_PrintHistory(TI_HANDLE hCmdQueue, int NunOfCmd);
int					CmdQueue_ResultReceived(TI_HANDLE hCmdQueue, UINT32 status);
int 					CmdQueue_Error	(TI_HANDLE hCmdQueue);
int					CmdQueue_SendCmplt(TI_HANDLE hCmdQueue);
#ifdef REPORT_LOG
char*               CmdQueue_GetIEString(int MboxCmdType, UINT16 Id);
char*               CmdQueue_GetCmdString(int MboxCmdType);
char*               CmdQueue_GetErrorString(CommandStatus_e MboxError);
#endif /* REPORT_LOG */

#endif
