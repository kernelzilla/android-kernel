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

//--------------------------------------------------------------------------
// Module:		TI_Adapter.cpp
//
// Purpose:		
//
//--------------------------------------------------------------------------

#include "CommonAdapter.h"
#ifdef _WINDOWS
#endif

tiBOOL TI_CheckAdapterObject(void *pObj)
{
    tiBOOL bRes = FALSE;
    
    TI_OAL*  pOSLib = TI_OAL::GetInstance();
    if (pOSLib)
        bRes = !(pOSLib->TIIsBadWritePtr(pObj, sizeof(CTI_WLAN_AdapterAPI)));

    TI_OAL::FreeInstance();

    return bRes;
}

TI_WLAN_AdapterAPI* TI_AdapterCppInit(tiCHAR* pszDeviceName, tiBOOL bForce )
{
    return CTI_WLAN_AdapterAPI::GetTIWLANAdapter(pszDeviceName, bForce);
}

tiINT32 TI_AdapterCppDeinit( TI_WLAN_AdapterAPI* pAdapter, tiBOOL bForce )
{
    return CTI_WLAN_AdapterAPI::FreeTIWLANAdapter((CTI_WLAN_AdapterAPI*)pAdapter, bForce);
}

