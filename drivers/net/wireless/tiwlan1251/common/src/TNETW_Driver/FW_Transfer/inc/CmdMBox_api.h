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
/*   MODULE:  CmdMBox_api.h						*/
/*   PURPOSE: CmdMBox api h file					*/
/*                                                                                	*/
/**********************************************************************************/
#ifndef _CMDMBOX_API_H_
#define _CMDMBOX_API_H_

#include "whalCommon.h"
#include "public_commands.h"

/*****************************************************************************
 **         Defines	                                                       **
 *****************************************************************************/

/*****************************************************************************
 **         Enums                                                    **
 *****************************************************************************/

/*****************************************************************************
 **         Types                                                      **
 *****************************************************************************/
typedef int (*CmdMBox_CB_t )(TI_HANDLE objectHandle);


/*****************************************************************************
 **         API functions definitions                                  **
 *****************************************************************************/
TI_HANDLE       CmdMBox_Create(TI_HANDLE hOs);
int             CmdMBox_Destroy(TI_HANDLE hCmdMBox);
int             CmdMBox_Config(TI_HANDLE hCmdMBox, TI_HANDLE hTNETWIF, TI_HANDLE hFwEvent, TI_HANDLE hCmdQueue, TI_HANDLE hReport);
TI_STATUS       CmdMBox_ConfigHw(TI_HANDLE hCmdMBox, UINT8 module_id, fnotify_t fCb, TI_HANDLE hCb);
int             CmdMBox_SetModeNormal(TI_HANDLE hCmdMBox);
int             CmdMBox_Restart(TI_HANDLE hCmdMBox);
int             CmdMBox_SendCmd(TI_HANDLE hCmdMBox, Command_e cmdType, UINT8* pParamsBuf, UINT32 paramsLen);
int             CmdMBox_GetResult(TI_HANDLE hCmdMBox, UINT8* pParamsBuf, UINT32 paramsLen, UINT32* status);
TI_STATUS       CmdMBox_CmdCmplt(TI_HANDLE hCmdMBox);
TI_STATUS       CmdMBox_GetStatus(TI_HANDLE hCmdMBox);
#endif
