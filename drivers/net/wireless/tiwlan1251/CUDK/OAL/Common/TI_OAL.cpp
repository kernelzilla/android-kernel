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

// TIOsLib.cpp : Defines the entry point for the DLL application.
//
#include "CommonOAL.h"

#ifdef _WINDOWS
#endif //_WINDOWS


//////////////////////////////////////////////////////////////////////
// TI_OAL Class
//////////////////////////////////////////////////////////////////////
TI_OAL*  TI_OAL::_instance        = NULL;
tiUINT32   TI_OAL::m_uReferenceCount= 0;
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

TI_OAL::TI_OAL()
{
}

TI_OAL::~TI_OAL()
{
}

TI_OSCriticalSection::TI_OSCriticalSection()
{
    m_pWrapCS = TI_OSWrapCriticalSection::CreateObject();
}

TI_OSCriticalSection::~TI_OSCriticalSection()
{
    if ( m_pWrapCS )
    {
	TI_OSWrapCriticalSection::DeleteObject(m_pWrapCS);
        m_pWrapCS = NULL;
    }
}

tiVOID    
TI_OSCriticalSection::Enter()
{
    if ( m_pWrapCS )
        m_pWrapCS->Enter();
}

tiVOID
TI_OSCriticalSection::Leave()
{
    if ( m_pWrapCS )
        m_pWrapCS->Leave();
}


/*******************************************************************************************/
/*******************************************************************************************/
/*******************************************************************************************/
/*
TI_OSEvent::TI_OSEvent()
{
    m_pWrapEvent = TI_OSWrapEvent::CreateObject();
}

TI_OSEvent::~TI_OSEvent()
{
    if ( m_pWrapEvent )
        delete m_pWrapEvent;
}

tiVOID    
TI_OSEvent::Set()
{
    if ( m_pWrapEvent )
        m_pWrapEvent->Set();
}

tiVOID    
TI_OSEvent::Reset()
{
    if ( m_pWrapEvent )
        m_pWrapEvent->Reset();
}

tiUINT32
TI_OSEvent::Wait(tiUINT32 uTime)
{
    if ( !m_pWrapEvent )
        return TI_RESULT_FAILED;
        
    return m_pWrapEvent->Wait(uTime);
}
*/
