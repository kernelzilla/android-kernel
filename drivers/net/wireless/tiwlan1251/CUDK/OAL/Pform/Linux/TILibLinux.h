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

/* TILibLinux.h: interface for the CTI_LibLinux class.*/
/**/
/*////////////////////////////////////////////////////////////////////*/

#if !defined(TI_LIB_LINUX_H_)
#define TI_LIB_LINUX_H_

class CTI_OSCriticalSectionLinux : public TI_OSWrapCriticalSection
{
    public: 
                    CTI_OSCriticalSectionLinux();
                   ~CTI_OSCriticalSectionLinux();
        tiVOID      Enter                   ();
        tiVOID      Leave                   ();
};
/*
class CTI_OSEventLinux : public TI_OSWrapEvent
{
        pthread_mutexattr_t m_MutexAttr;
        pthread_cond_t      m_Cond;
        pthread_condattr_t  m_CondAttr;
        tiBOOL              m_bSet;
    public:
                    CTI_OSEventLinux();
                   ~CTI_OSEventLinux();
    
        tiUINT32    Wait                (tiUINT32 uTime);
        tiVOID      Set                 ();
        tiVOID      Reset               ();
};
*/
typedef tiVOID* (* uxTHREAD_START_ROUTINE)( tiVOID* pThreadParameter );

class CTI_LibLinux: public TI_OAL  
{
    public:
                                CTI_LibLinux    ();
        virtual                ~CTI_LibLinux    ();

        static  TI_OAL*     GetInstance  ();
        static  tiVOID      FreeInstance();

                /* list of functions that will call from Utility Adapter and Utility GUI modules */
                tiVOID      TIOutputDebugString     (tiCHAR* lpOutputString);
                tiBOOL      TIIsBadWritePtr         (tiVOID* lp, tiUINT32 ucb );

        /* list of functions that will call from Windows Utility module */
                tiUINT32    TILoadLibrary           (tiCHAR*    pLibFileName);
                tiBOOL      TIFreeLibrary           (tiUINT32   hLibModule);
                tiUINT32    TIGetProcAddress        (tiUINT32   hModule, tiCHAR* lpProcName );
                tiUINT32    TIRegisterWindowMessage (tiCHAR*    pszMsgName );
                tiBOOL      TIPostMessage           (tiUINT32 hWnd, tiUINT32 Msg, tiUINT32 wParam, tiUINT32 lParam);
                tiVOID      TIPrintLastError        (tiCHAR*    psz);
                tiUINT32    TIGetCurrentThreadId();
                tiUINT32    TICreateThread          (tiPTHREAD_START_ROUTINE pStartAddress, tiVOID* pParameter );
                tiVOID      TISleep                 (tiUINT32 msec);
};

#endif /* !defined(TI_LIB_LINUX_H_)*/
