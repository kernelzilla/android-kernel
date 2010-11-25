/*******************************************************************************
**+--------------------------------------------------------------------------+**
**|                                                                          |**
**| Copyright 1998-2008 Texas Instruments, Inc. - http://www.ti.com/         |**
**|                                                                          |**
**| Licensed under the Apache License, Version 2.0 (the "License");          |**
**| you may not use this file except in compliance with the License.         |**
**| You may obtain a copy of the License at                                  |**
**|                                                                          |**
**|     http://www.apache.org/licenses/LICENSE-2.0                           |**
**|                                                                          |**
**| Unless required by applicable law or agreed to in writing, software      |**
**| distributed under the License is distributed on an "AS IS" BASIS,        |**
**| WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. |**
**| See the License for the specific language governing permissions and      |**
**| limitations under the License.                                           |**
**|                                                                          |**
**+--------------------------------------------------------------------------+**
*******************************************************************************/


#ifndef _IPC_EVENT
#define _IPC_EVENT


#include "ipc_event.h"
#include "osTIType.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

tiINT32 ipc_CreateInterface (tiVOID);


typedef struct IPC_CONFIG_PARAMS
{
    tiUINT32	len;
    tiINT32 (*F_ConfigNotification)(tiVOID*  pCuData, tiUINT32 nDataSize);
    tiUINT32      enable;
}IPC_CONFIG_PARAMS;

typedef struct config_registry
{
	tiUINT32	len;
	TI_HANDLE	hReceiver;
	PVOID       cfg_cb;
	tiUINT32    enable;
}config_registry_t;

typedef struct unbound_registry
{
	TI_HANDLE	hReceiver;
	PVOID       cfg_cb;
	tiUINT32    enable;
}unbound_registry_t;

typedef struct _reg_clients
{
	UINT32		EventCode;
    UINT32      ProcessId;
}reg_clients_t;
tiINT32 IPC_RegisterConfig(tiVOID* pEvParams, tiUINT32 EvParamsSize);

tiINT32 ipc_interfaces_init(tiVOID);

tiINT32 cnfg_open(tiVOID);

#ifdef __cplusplus
}
#endif

#endif

