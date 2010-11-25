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

/*--------------------------------------------------------------------------*/
/* Module:      TI_OAL.h*/
/**/
/* Purpose:     This file contains a interface for the TI_OAL class.*/
/**/
/*////////////////////////////////////////////////////////////////////*/

#if !defined(AFX_TIUNILIB_H__F79CA4B8_8596_4F36_B541_2B2FCCF70197__INCLUDED_)
#define AFX_TIUNILIB_H__F79CA4B8_8596_4F36_B541_2B2FCCF70197__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif /* _MSC_VER > 1000*/

#include "osTIType.h"
#include "TI_Results.h"

#ifdef _UNICODE
    #define tisprintf   swprintf
    #define tistrncpy   wcsncpy
#else
    #define tisprintf   sprintf
    #define tistrncpy   strncpy
#endif


class  TI_OSWrapCriticalSection
{               
    protected:
                tiVOID*     m_pCS;   
    public: 
        static  TI_OSWrapCriticalSection*   CreateObject();
        static  tiVOID          DeleteObject(TI_OSWrapCriticalSection* pObj);
        
                    TI_OSWrapCriticalSection() {}
        virtual    ~TI_OSWrapCriticalSection() {}
        virtual tiVOID    Enter               ()  = 0;
        virtual tiVOID    Leave               ()  = 0;
};

class  TI_OSCriticalSection
{
        TI_OSWrapCriticalSection*               m_pWrapCS;
     
    public: 
                    TI_OSCriticalSection();
                   ~TI_OSCriticalSection();
                tiVOID    Enter               ();
                tiVOID    Leave               ();
};
/*
class  TI_OSWrapEvent
{               
    protected:
                tiVOID*     m_pEvent;   
    public: 
        static  TI_OSWrapEvent*   CreateObject();
        
                    TI_OSWrapEvent()    {}
                   ~TI_OSWrapEvent()    {}
        virtual tiUINT32    Wait    ( tiUINT32 uTime )  = 0;
        virtual tiVOID      Set     ()  = 0;
        virtual tiVOID      Reset   ()  = 0;
};

class  TI_OSEvent
{
        TI_OSWrapEvent*               m_pWrapEvent;
     
    public: 
                    TI_OSEvent();
                   ~TI_OSEvent();
             tiUINT32    Wait    ( tiUINT32 uTime );
             tiVOID      Set     ();
             tiVOID      Reset   ();
};
*/
typedef tiUINT32 (* tiPTHREAD_START_ROUTINE)( tiVOID* pThreadParameter );

class  TI_OAL  
{
    protected:
        
                                TI_OAL            ();
        virtual                ~TI_OAL            ();

        static  TI_OAL*       _instance;
        static  tiUINT32        m_uReferenceCount; 
        
    public:
        static  TI_OAL*     GetInstance             (); /* static function for create TI_OSlib object*/
        static  tiVOID      FreeInstance            (); /* static function for release TI_OAL object*/

        /* list of functions that will call from Utility Adapter and Utility GUI modules */
        virtual tiVOID      TIOutputDebugString     (tiCHAR* lpOutputString)                                        = 0;
        virtual tiBOOL      TIIsBadWritePtr         (tiVOID* lp, tiUINT32 ucb )                                     = 0;

        /* list of functions that will call from Windows Utility module */
        virtual tiUINT32    TILoadLibrary           (tiCHAR*    pLibFileName)                                       = 0;
        virtual tiBOOL      TIFreeLibrary           (tiUINT32   hLibModule)                                         = 0;
        virtual tiUINT32    TIGetProcAddress        (tiUINT32   hModule, tiCHAR* lpProcName )                       = 0;
        virtual tiUINT32    TIRegisterWindowMessage (tiCHAR*    pszMsgName )                                        = 0;
        virtual tiBOOL      TIPostMessage           (tiUINT32 hWnd, tiUINT32 Msg, tiUINT32 wParam, tiUINT32 lParam) = 0;
        virtual tiVOID      TIPrintLastError        (tiCHAR*    psz)                                                = 0;
        virtual tiUINT32    TIGetCurrentThreadId    ()                                                              = 0;
        virtual tiUINT32    TICreateThread          (tiPTHREAD_START_ROUTINE pStartAddress, tiVOID* pParameter )    = 0;
        virtual tiVOID      TISleep                 (tiUINT32 msec)                                                = 0;
};

#define TIOALib_OBJECT_CREATOR_IMP( ClassApi, ClassBase )   \
ClassBase* ClassBase::CreateObject(){ return (ClassBase*) new ClassApi;} \
tiVOID  ClassBase::DeleteObject(ClassBase* pObj){ClassApi* pRealObj = (ClassApi*)pObj; \
if(pRealObj)delete pRealObj; }


#define TIOALib_SINGLETON_CLASS_IMP( ClassApi )             \
TI_OAL* TI_OAL::GetInstance()                               \
{ return (TI_OAL*) ClassApi::GetInstance(); }               \
TI_OAL* ClassApi::GetInstance()                             \
{if( _instance == 0){_instance = new ClassApi();}           \
m_uReferenceCount++; return _instance;}                     \
tiVOID TI_OAL::FreeInstance()                               \
{ ClassApi::FreeInstance(); }                               \
tiVOID ClassApi::FreeInstance()                             \
{ m_uReferenceCount--;if(!m_uReferenceCount && _instance )  \
{delete (ClassApi*)_instance;_instance = NULL;}}                                                   
    
#endif /* !defined(AFX_TIUNILIB_H__F79CA4B8_8596_4F36_B541_2B2FCCF70197__INCLUDED_)*/

