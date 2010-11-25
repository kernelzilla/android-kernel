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

#ifndef _COMMON_ADAPTER_H
#define _COMMON_ADAPTER_H

#if defined(_WINDOWS)
#else
#include <stdlib.h>
#define _UTIL_ADAPTER
#endif /*_WINDOWS*/

#include <stdio.h>
#include "tiwlnif.h"

#include "tiioctl.h"

#include "osTIType.h"
#include "osDot11.h"
#include "public_commands.h"

#ifdef EXC_MODULE_INCLUDED
#include "TI_AdapterEXC.h"
#include "osDot11Exc.h"
#endif /*EXC_MODULE_INCLUDED */

#include "TI_IPC_Api.h"
#include "TI_Results.h"
#include "TI_OAL.h"

#include "TI_AdapterApiC.h"
#include "TI_AdapterApiCpp.h"
#include "CTI_Adapter.h"

#endif /* _COMMON_ADAPTER_H*/
