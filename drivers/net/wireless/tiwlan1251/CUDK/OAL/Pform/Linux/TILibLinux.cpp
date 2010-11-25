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

// TILibLinux.cpp :
//
#include <stdio.h>
#include <errno.h>
#include <time.h>

#include "CommonOAL.h"
#include "TILibLinux.h"
#include <pthread.h>

//////////////////////////////////////////////////////////////////////
// CTI_LibLinux Class
//////////////////////////////////////////////////////////////////////
TIOALib_SINGLETON_CLASS_IMP( CTI_LibLinux )
TIOALib_OBJECT_CREATOR_IMP( CTI_OSCriticalSectionLinux, TI_OSWrapCriticalSection )

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CTI_LibLinux::CTI_LibLinux()
{
}

CTI_LibLinux::~CTI_LibLinux()
{
}

tiVOID        
CTI_LibLinux::TIOutputDebugString (tiCHAR* lpOutputString)
{
    if ( lpOutputString )
        fprintf(stderr, lpOutputString);
}

tiUINT32   
CTI_LibLinux::TILoadLibrary(tiCHAR* lpLibFileName)
{
    return 0;
}
tiUINT32
CTI_LibLinux::TIGetCurrentThreadId()
{
    return pthread_self();
}

tiBOOL        
CTI_LibLinux::TIFreeLibrary( tiUINT32 hLibModule )
{
    return 0;
}

tiUINT32        
CTI_LibLinux::TIRegisterWindowMessage (tiCHAR* lpszMsgName)
{
    return 0;
}

tiBOOL        
CTI_LibLinux::TIPostMessage(tiUINT32 hWnd, tiUINT32 uMsg, tiUINT32 wParam, tiUINT32 lParam)
{
    return 0;
}

tiUINT32 
CTI_LibLinux::TIGetProcAddress(tiUINT32 hModule, tiCHAR* lpProcName )
{
    return 0;
}

tiBOOL
CTI_LibLinux::TIIsBadWritePtr(tiVOID *lp, tiUINT32 ucb )
{
    return FALSE;   //IsBadWritePtr(lp, ucb );
}

tiVOID
CTI_LibLinux::TIPrintLastError(tiCHAR* lpsz)
{
#ifdef DEBUG_MESSAGES    
    tiCHAR szTmp[512];
    sprintf(szTmp,"%s LastError(0x%Xh)\n", lpsz, errno);
    TIOutputDebugString(szTmp);
#endif
}


tiUINT32
CTI_LibLinux::TICreateThread(tiPTHREAD_START_ROUTINE pStartAddress, tiVOID* pParameter )
{
    uxTHREAD_START_ROUTINE  thread_start_address = (uxTHREAD_START_ROUTINE)pStartAddress;
    pthread_t               supp_thread_id;
    pthread_attr_t          supp_thread_attrs;

    pthread_attr_init(&supp_thread_attrs);

    int iRet = pthread_create(&supp_thread_id, &supp_thread_attrs, thread_start_address, pParameter);

    pthread_attr_destroy(&supp_thread_attrs);

    if ( iRet == 0)
        return TI_RESULT_OK;

    return TI_RESULT_FAILED;
}


CTI_OSCriticalSectionLinux::CTI_OSCriticalSectionLinux()
{
  /*  m_pCS = (tiVOID*) new pthread_rwlock_t;

    if ( m_pCS )
    {
        memset( m_pCS, 0, sizeof(pthread_rwlock_t));
        pthread_rwlock_init((pthread_rwlock_t*) m_pCS, NULL )
    }
    */
}

CTI_OSCriticalSectionLinux::~CTI_OSCriticalSectionLinux()
{
    /*
    if (m_pCS)
    {
        pthread_rwlock_destroy((pthread_rwlock_t*) m_pCS);
        delete m_pCS;
        m_pCS = NULL;
    }
    */
}

tiVOID
CTI_OSCriticalSectionLinux::Enter()
{
    /*
    if ( m_pCS )
        pthread_rwlock_wrlock((pthread_rwlock_t*) m_pCS);
    */
}

tiVOID
CTI_OSCriticalSectionLinux::Leave()
{
    /*
    if ( m_pCS )
        pthread_rwlock_unlock((pthread_rwlock_t*) m_pCS);
    */
}

tiVOID
CTI_LibLinux::TISleep(tiUINT32 msec)
{
    struct timespec req;

    req.tv_sec = 0;
    req.tv_nsec = 100000; /* sleep for 100 msec */
    nanosleep( &req, NULL );
}

/*

CTI_OSEventLinux::CTI_OSEventLinux()
{   
    m_bSet = FALSE;

    pthread_condattr_init(&m_CondAttr); 
    pthread_cond_init(&m_Cond, &m_CondAttr);
    
    pthread_mutexattr_init(&m_MutexAttr);
    pthread_mutexattr_settype(&m_MutexAttr, PTHREAD_MUTEX_NORMAL);

    m_pEvent = (tiVOID*) new pthread_mutex_t;

    if ( m_pEvent )
    {
        memset( m_pEvent, 0, sizeof(pthread_mutex_t));
        pthread_mutex_init( (pthread_mutex_t*) m_pEvent, &m_MutexAttr);
    }
}

CTI_OSEventLinux::~CTI_OSEventLinux()
{
    if (m_pEvent)
    {
        pthread_mutex_destroy((pthread_mutex_t*) m_pEvent);
        delete m_pEvent;
        m_pEvent = NULL;
    }

    pthread_mutexattr_destroy(&m_attr);
    pthread_cond_destroy(&m_Cond);
    pthread_condattr_destroy(&m_CondAttr);  
}

tiVOID
CTI_OSEventLinux::Set()
{
    if ( m_pEvent )
        pthread_cond_signal((pthread_mutex_t*) m_pEvent);
}

tiUINT32
CTI_OSEventLinux::Wait(tiUINT32 uTimeout)
{
    if (m_pEvent)
    {
        pthread_mutex_lock((pthread_mutex_t*) m_pEvent);

        if (uTimeout == INFINITE)
        {
            while (!m_bSet) // have to wait for the event
                pthread_cond_wait(&m_Cond, (pthread_mutex_t*) m_pEvent );
        }
        else
        {
            timespec  timeOut;
            clock_gettime(CLOCK_REALTIME, &timeOut);
            
            // if there are seconds involved
            if (uTimeout >= 1000)
            {
                timeOut.tv_sec += uTimeout / 1000;
        
                // get rest
                int nRemain = nMilliseconds % 1000;
                if (nRemain != 0)
                {
                    timeOut.tv_nsec +=  nRemain * 1000000L;
            
                    // wrapped into the next second
                    if (timeOut.tv_nsec > 1000000000L)
                    {
                        timeOut.tv_nsec -= 1000000000L;
                        timeOut.tv_sec++;
                    }
                }
            }
            else
            {
                timeOut.tv_nsec +=  uTimeout * 1000000L;
            }

            while (!m_bSet) // have to wait for the event
            {
                // the following line releases the mutex and waits until the 
                // condition is signalled.  when the call returns, we own the
                // mutex again, unless an exception is thrown, in which case
                // the mutex is unlocked

                int nResult = pthread_cond_timedwait(&m_Cond, (pthread_mutex_t*) m_pEvent , &timeOut));
            
                if ( nResult != 0 ) 
                {
                    pthread_mutex_unlock((pthread_mutex_t*) m_pEvent);
                    // time out, we have the lock
                    pthread_mutex_lock((pthread_mutex_t*) m_pEvent);
                          
                    // return "time out" condition
                    return;  
               }

        }
};

inline _dcfTimeOut::_dcfTimeOut(unsigned int nMilliseconds)
{
    int nResult = clock_gettime(CLOCK_REALTIME, &_tsWhen);
    
#ifdef _DEBUG
    DCF_ASSERT(nResult == 0);
#endif
    
    
}
    }
    return TI_RESULT_OK;
}
*/

