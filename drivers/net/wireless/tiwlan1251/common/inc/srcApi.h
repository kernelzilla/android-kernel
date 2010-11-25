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

/**************************************************************************/
/*                                                                        */
/* MODULE:  srcApi.h							  */
/* PURPOSE: Header file of config Manager module			  */
/*                                                                        */
/**************************************************************************/

#ifndef __SRC_API_H__
#define __SRC_API_H__

#include "osTIType.h"
#include "paramOut.h"
#include "memMngrEx.h"

#define DRIVER_SHUTDOWN_SME_STOPPED       0x1
#define DRIVER_SHUTDOWN_COMPLETE          (DRIVER_SHUTDOWN_SME_STOPPED)

TI_HANDLE configMgr_create(TI_HANDLE        hOs,void *pWLAN_Images,
                           initTable_t      *pInitTable,
                           macAddress_t     *pMac);

TI_HANDLE configMgr_init  (TI_HANDLE        hOs,TI_HANDLE  hConfigManager,void *pWLAN_Images,
                           initTable_t      *pInitTable,
                           macAddress_t     *pMac);


/* Unload for windows */
TI_STATUS configMgr_unLoad(TI_HANDLE hConfigMgr);

/* Start unload process (stop SME) */
TI_STATUS configMgr_InitiateUnload(TI_HANDLE hConfigMgr);

/* Unload all modules (free memory etc) */
TI_STATUS configMgr_UnloadModules (TI_HANDLE hConfigMgr);

/* Return internal shutdown status */
UINT8 configMgr_DriverShutdownStatus(TI_HANDLE hConfigMgr);

TI_STATUS configMgr_start(TI_HANDLE hConfigMgr);

TI_STATUS configMgr_stop(TI_HANDLE hConfigMgr);

TI_STATUS configMgr_setParam(TI_HANDLE		hConfigMgr, 
						  paramInfo_t	*pParam);

TI_STATUS configMgr_getParam(TI_HANDLE		hConfigMgr, 
						  paramInfo_t	*pParam);

TI_STATUS configMgr_sendMsdu(TI_HANDLE		hConfigMgr, 
						mem_MSDU_T	*pMsdu,
						UINT8 packet_DTag);

TI_STATUS configMgr_PollApPackets(TI_HANDLE		hConfigMgr);


TI_STATUS configMgr_checkTxQueueSize(TI_HANDLE hConfigMgr,UINT8 qIndex);

/* Event Handler wrapper */
UINT32 configMgr_RegisterEvent(TI_HANDLE		hConfigMgr, PUCHAR pData, ULONG Length);

UINT32 configMgr_UnRegisterEvent(TI_HANDLE		hConfigMgr, TI_HANDLE uEventID);

UINT32 configMgr_MaskEvent(TI_HANDLE		hConfigMgr, UINT32 uEventID);

UINT32 configMgr_UnMaskEvent(TI_HANDLE		hConfigMgr, UINT32 uEventID);

/* Initiated from User Space to fetch event data*/
UINT32 configMgr_GetEventData                  (TI_HANDLE		hConfigMgr, PUCHAR pData,   ULONG* pLength);

/* Memory manager wrapper */
TI_STATUS configMgr_allocBDs(TI_HANDLE hConfigMgr, 
						  UINT32 bdNumber, 
						  mem_BD_T** bdPtr);

TI_STATUS configMgr_allocMSDU(TI_HANDLE hConfigMgr, 
						  mem_MSDU_T** MSDUPtr,	
						  UINT32 len, 
						  allocatingModule_e module);

TI_STATUS configMgr_allocMSDUBufferOnly(TI_HANDLE hConfigMgr, 
									mem_MSDU_T** MSDUPtr, 
									allocatingModule_e module);

TI_STATUS configMgr_memMngrFreeMSDU(TI_HANDLE hConfigMgr, 
								 UINT32 handle);

/* HAL	wrapper */
TI_STATUS configMgr_HandleBusTxn_Complete(TI_HANDLE hConfigMgr);

TI_STATUS configMgr_handleInterrupts(TI_HANDLE hConfigMgr);

TI_STATUS configMgr_enableInterrupts(TI_HANDLE hConfigMgr);

TI_STATUS configMgr_disableInterrupts(TI_HANDLE hConfigMgr);

UINT32  configMgr_checkInterrupts(TI_HANDLE hConfigMgr);

BOOL configMgr_isCardExist(TI_HANDLE hConfigMgr);

BOOL configMgr_areInputsFromOsDisabled(TI_HANDLE hConfigMgr);

UINT32 configMgr_ReadMacRegister(TI_HANDLE hConfigMgr, UINT32	addr);
void  configMgr_WriteMacRegister(TI_HANDLE hConfigMgr, UINT32	addr, UINT32	val);
UINT32 configMgr_ReadPhyRegister(TI_HANDLE hConfigMgr, UINT32	addr);
void configMgr_WritePhyRegister(TI_HANDLE hConfigMgr, UINT32	addr, UINT32	val);

UINT32 configMgr_getPacketHeaderLength(TI_HANDLE hConfigMgr, void *pData, UINT32 txFlags);

void configMgr_SlaveAckMaskNotification(TI_HANDLE hConfigMgr);

void configMgr_GetInitParams (TI_HANDLE hConfigMgr, UINT8* ioBuffer, UINT16 *outBufLen);

#endif /* __SRC_API_H__ */

