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

#ifndef TICON_H
#define TICON_H
#include <stdio.h>
#include <string.h>

#include "osTIType.h"
#include "cli_cu_common.h"

#ifndef IFNAMSIZ
#define	IFNAMSIZ	16
#endif

extern 	char 	g_drv_name[IFNAMSIZ + 1];
IMPORT_TI_API extern TI_HANDLE 	g_id_adapter; //TRS:MEB support usage of DLL

#ifdef _WINDOWS
#endif
/*int iw_sockets_open(void); */


int console_printf_terminal(const char *arg_list ,...);

#ifndef _WINDOWS
	int console_send_buffer_to_host(tiUINT8 module_inedx, tiUINT8 *buffer, tiUINT16 length);
#endif /* __LINUX__ */

#endif /* #define TICON_H */
