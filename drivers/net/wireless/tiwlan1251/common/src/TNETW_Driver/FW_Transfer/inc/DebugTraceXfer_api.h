
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
/*   MODULE:  DebugTraceXfer_api.h											          */
/*   PURPOSE: Handle The FW Debug Messages 										  */
/*                                                                                */
/**********************************************************************************/
#ifdef TI_DBG

#ifndef _DEBUG_TRCAE_XFER_API_H
#define _DEBUG_TRCAE_XFER_API_H


TI_HANDLE debugTrace_Create	(TI_HANDLE hOS);

void debugTrace_Destroy		(TI_HANDLE hDebugTrace);

void debugTrace_Config(TI_HANDLE hDebugTrace, TI_HANDLE hWhalParams, TI_HANDLE hReport,
					   TI_HANDLE hMemMgr, TI_HANDLE hTNETWIF, TI_HANDLE hFwEvent);

void debugTrace_ConfigHw	(TI_HANDLE hDebugTrace,UINT32 TraceAddA, UINT32 TraceAddB);

TI_STATUS debugTrace_Event	(TI_HANDLE hDebugTrace);

void debugTrace_Disable		(TI_HANDLE hDebugTrace);

void debugTrace_Enable		(TI_HANDLE hDebugTrace);

void debugTrace_Print		(TI_HANDLE hDebugTrace);

void debugTrace_EnablePrint	(TI_HANDLE hDebugTrace);

void debugTrace_DisablePrint(TI_HANDLE hDebugTrace);

void debugTrace_handleBuffer(TI_HANDLE hDebugTrace);

void debugTrace_UdpEnable	(TI_HANDLE hDebugTrace);

void debugTrace_UdpDisable	(TI_HANDLE hDebugTrace);

#endif /* _DEBUG_TRCAE_XFER_API_H */

#endif /* TI_DBG */
