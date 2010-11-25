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
// Module:		CTI_Adapter.cpp
//
// Purpose:		implementation of the CTI_WLAN_AdapterAPI class.
//
//--------------------------------------------------------------------------

#include <string.h>

#include "CommonAdapter.h"
#ifdef _WINDOWS
#endif

#ifndef _WINDOWS
	#include "g_tester.h"
#endif  

#include "paramOut.h"

#ifdef _WINDOWS
#endif
TI_OSCriticalSection	m_csInitAdapter; 

#define TI_WLAN_API_VER   0x00400002



_AdapterItem* CTI_WLAN_AdapterAPI::m_pAdaptersList = NULL;
//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CTI_WLAN_AdapterAPI::CTI_WLAN_AdapterAPI(tiCHAR* lpszAdapterName)
{
#ifdef TI_EMBEDDED_SUPPLICANT
    m_pSupplicant = NULL;
#endif /* ifdef TI_EMBEDDED_SUPPLICANT */
    m_pszAdapterName = NULL;
    m_pOSLib = TI_OAL::GetInstance();
    m_pRegistry = new TI_OSRegistry; // temporary solution
#ifdef _WINDOWS
#else
    m_pIPCmod = new TI_IPC(/*lpszAdapterName*/);
#endif 

#ifdef TI_EMBEDDED_SUPPLICANT
    m_bSupplicantInUse = FALSE;
#endif /* ifdef TI_EMBEDDED_SUPPLICANT */
    
    if (lpszAdapterName)
    {
        tiUINT32 uSize = 0;
#ifndef  _UNICODE
        uSize = strlen( lpszAdapterName );
#else
        uSize = wcslen( lpszAdapterName );
#endif
        tiUINT32 uBuffLength = sizeof(tiCHAR)*(uSize+1);
        m_pszAdapterName = new tiCHAR[uSize+1];
        
        if (!m_pszAdapterName)
            return ;
        
        memset(m_pszAdapterName, 0, uBuffLength);
        memcpy(m_pszAdapterName, lpszAdapterName, uBuffLength - sizeof(tiCHAR) );
#ifndef _WINDOWS
        m_pIPCmod->IPC_DeviceOpen(m_pszAdapterName);
#endif
    }
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::StartSM( )
{
#ifdef TI_EMBEDDED_SUPPLICANT
	#ifndef _WINDOWS
		m_bSupplicantInUse = TRUE;
		if ( m_pSupplicant == NULL )
			m_pSupplicant = new TI_IPC_Supplicant(m_pszAdapterName);
		else
			return TI_RESULT_FAILED;
	#else
	#endif
#endif /* ifdef TI_EMBEDDED_SUPPLICANT */
    
    return TI_RESULT_OK;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::StopSM( )
{
#ifdef TI_EMBEDDED_SUPPLICANT
#ifndef _WINDOWS
    if (m_bSupplicantInUse && m_pSupplicant != NULL )
    {
        delete m_pSupplicant;
        m_pSupplicant = NULL;
        m_bSupplicantInUse = FALSE;
    }
    else
        return TI_RESULT_FAILED;
#else
#endif
#endif
    return TI_RESULT_OK;
}
/********************************************************************/

CTI_WLAN_AdapterAPI::~CTI_WLAN_AdapterAPI()
{
#ifndef _WINDOWS
    if ( m_pIPCmod )
    {
        m_pIPCmod->IPC_DeviceClose();
        delete m_pIPCmod;
        m_pIPCmod = NULL;
    }
#else    
#endif
    if (m_pRegistry)
    {
        delete m_pRegistry;
        m_pRegistry = NULL;
    }
    
    if ( m_pOSLib )
    {
        m_pOSLib->FreeInstance();
        m_pOSLib = NULL;
    }
    
    if ( m_pszAdapterName )
    {
        delete [] m_pszAdapterName;
        m_pszAdapterName = NULL;
    }
#ifdef TI_EMBEDDED_SUPPLICANT
#ifndef _WINDOWS
    if (m_pSupplicant != NULL )
    {
        delete m_pSupplicant;
        m_pSupplicant = NULL;
    }
#endif    
#endif
}

/********************************************************************/

CTI_WLAN_AdapterAPI*
CTI_WLAN_AdapterAPI::GetTIWLANAdapter(tiCHAR* lpszAdapterName, tiBOOL bForce)
{
    
    m_csInitAdapter.Enter();
    
    if ( bForce )
    {
        CTI_WLAN_AdapterAPI* pAdapter = new CTI_WLAN_AdapterAPI(lpszAdapterName);
        m_csInitAdapter.Leave();
        return pAdapter;
    }
    
    _AdapterItem* pCurrentItem = m_pAdaptersList;
    
    BOOL  bNULL = FALSE;
    if (!lpszAdapterName)
    {
        bNULL = TRUE;
        m_csInitAdapter.Leave();
        return NULL;
    }
    
    while( pCurrentItem )
    {
        tiINT32 iResult = -1;
        
        if ( !bNULL )
#ifndef  _UNICODE
            iResult = strcmp(pCurrentItem->m_pAdapterName, lpszAdapterName );
#else
        iResult = wcscmp(pCurrentItem->m_pAdapterName, lpszAdapterName );
#endif
        else
        {
            if ( !pCurrentItem->m_pAdapterName )
                iResult = 0;    
        }   
        
        if (!iResult)
        {
            pCurrentItem->AddRef();
            m_csInitAdapter.Leave();
            return pCurrentItem->m_dwAdapterID;
        }
        
        pCurrentItem = pCurrentItem->m_pNextItem;
    }
    
    pCurrentItem = new _AdapterItem;
    
    pCurrentItem->m_pNextItem = m_pAdaptersList;
    
    if ( m_pAdaptersList )
        m_pAdaptersList->m_pPrevItem = pCurrentItem;
    
    m_pAdaptersList = pCurrentItem;
    
    if (lpszAdapterName)
    {
        tiUINT32 uSize = 0;
#ifndef  _UNICODE
        uSize = strlen( lpszAdapterName );
#else
        uSize = wcslen( lpszAdapterName );
#endif
        tiUINT32 uBuffLenght = sizeof(tiCHAR)*(uSize+1);
        tiCHAR* pBuff = new tiCHAR[uSize+1];
        
        if (!pBuff)
        {
            m_csInitAdapter.Leave();
            return NULL;
        } 
        
        memset(pBuff, 0, uBuffLenght);
        memcpy(pBuff, lpszAdapterName, uBuffLenght - sizeof(tiCHAR) );
        m_pAdaptersList->m_pAdapterName = pBuff;
    }
    
    pCurrentItem->m_dwAdapterID = new CTI_WLAN_AdapterAPI(lpszAdapterName);
    
    m_csInitAdapter.Leave();
    return m_pAdaptersList->m_dwAdapterID;
}

/********************************************************************/

tiINT32 
CTI_WLAN_AdapterAPI::FreeTIWLANAdapter(CTI_WLAN_AdapterAPI* pAdapter, tiBOOL bForce )
{
    m_csInitAdapter.Enter();
    
#ifndef _WINDOWS
    if ( bForce && pAdapter)
    {
        delete pAdapter;
        pAdapter = NULL;
        m_csInitAdapter.Leave();
        return TI_RESULT_OK;
    }
#else   
#endif
    if (
#ifndef _WINDOWS
		!pAdapter ||
#endif
    // TRS:PGK -- If there is no adapter list, exit.  Nothing to free.
		!m_pAdaptersList)
    {
        m_csInitAdapter.Leave();
        return TI_RESULT_FAILED;
    }
    
    _AdapterItem* pCurrentItem = m_pAdaptersList;
    while( pCurrentItem )
    {
        
        if (pCurrentItem->m_dwAdapterID == pAdapter )
        {
            pCurrentItem->DecRef();
            if ( !pCurrentItem->m_uRef )
            {
                _AdapterItem* pPrev = pCurrentItem->m_pPrevItem;
                if ( pPrev )
                    pPrev->m_pNextItem = pCurrentItem->m_pNextItem;
                
                _AdapterItem* pNext = pCurrentItem->m_pNextItem;
                if ( pNext )
                    pNext->m_pPrevItem = pCurrentItem->m_pPrevItem;
                
                if ( !pNext && !pPrev )
                    m_pAdaptersList = NULL;
                
                if ( pCurrentItem->m_pAdapterName )
                    delete [] pCurrentItem->m_pAdapterName;
                
                delete pCurrentItem->m_dwAdapterID;
                delete pCurrentItem;
                
            }
            
            m_csInitAdapter.Leave();
            return TI_RESULT_OK;
        }
        
        pCurrentItem = pCurrentItem->m_pNextItem;
    }
    
    if ( pAdapter )
        delete pAdapter;
    
    m_csInitAdapter.Leave();
    return TI_RESULT_FAILED;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::CheckObjectMemory(tiVOID *pObj, tiUINT32 uSizeObj)
{
    if ( !pObj || !uSizeObj )
        return FALSE;
    
    return !(m_pOSLib->TIIsBadWritePtr(pObj, uSizeObj));
}
/***************************************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::tiMiniportSetInformation(tiUINT32 dwInfoCode,tiVOID* lpInfoBuffer, tiUINT32* lpdwCbInfoBuffer)
{
    tiUINT32    dwRet       = 0;
    tiUINT32    dwLength    = *lpdwCbInfoBuffer + sizeof(dwInfoCode);
    tiUINT8*    lpBuffer    = new tiUINT8[dwLength];
    tiUINT32 dwRetSize;
    
    if ( !lpBuffer )
        return TI_RESULT_NOT_ENOUGH_MEMORY;
    
    memcpy(lpBuffer, &dwInfoCode, sizeof(dwInfoCode));
    memcpy((tiUINT8*)lpBuffer + sizeof(dwInfoCode), lpInfoBuffer, *lpdwCbInfoBuffer);
    
    dwRet = tiIoCtrl(TIWLN_IOCTL_OID_SET_INFORMATION,lpBuffer, dwLength,NULL,0,&dwRetSize);
    
    delete [] lpBuffer;
    return dwRet;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::tiIoCtrl(tiUINT32 dwIoCtrl, tiVOID* pInBuffer, tiUINT32 uInBufferSize,
                              tiVOID* pOutBuffer, tiUINT32 dwOutBufferSize, tiUINT32* dwBytesReturned)
{
	#ifdef TI_EMBEDDED_SUPPLICANT
		if ( m_bSupplicantInUse )
		{
			#ifdef _WINDOWS
			#endif  
				switch (dwIoCtrl)
				{
					case TIWLN_802_11_SSID_SET:
					case TIWLN_802_11_INFRASTRUCTURE_MODE_SET:
					case TIWLN_802_11_AUTHENTICATION_MODE_SET:
					case TIWLN_802_11_WEP_STATUS_SET:
					case TIWLN_802_11_PSK_SET:
					case TIWLN_802_11_EAP_TYPE_SET:
					case TIWLN_802_11_USER_ID_SET:
					case TIWLN_802_11_USER_PASSWORD_SET:
					case TIWLN_802_11_CERT_PARAMS_SHA1_SET:
					case TIWLN_802_11_CERT_PARAMS_FILE_NAME_SET:
					case TIWLN_802_11_KEY_TYPE_SET:
					case TIWLN_802_11_EXC_NETWORK_EAP_SET:
					case TIWLN_802_11_EXC_CONFIGURATION_SET:
					case TIWLN_802_11_ADD_WEP:
					case TIWLN_802_11_WPA_OPTIONS_SET:
					#ifdef _WINDOWS
					#else
						return m_pSupplicant->SendDataProxy(dwIoCtrl, pInBuffer, uInBufferSize);
					#endif
				}
				#ifdef _WINDOWS
				#endif
				}
	#endif /* ifdef TI_EMBEDDED_SUPPLICANT */
#ifdef _WINDOWS
#else   
    tiUINT32 bRet = m_pIPCmod->IPC_DeviceIoControl( dwIoCtrl, pInBuffer, uInBufferSize,
        pOutBuffer, dwOutBufferSize, dwBytesReturned); 
    return bRet;
#endif
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetStatistics(TIWLN_STATISTICS* ptiStatistics)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if (CheckObjectMemory(ptiStatistics, sizeof(TIWLN_STATISTICS)))
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_STATISTICS, NULL, 0, ptiStatistics, sizeof(TIWLN_STATISTICS),&dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetTxStatistics(TIWLN_TX_STATISTICS* ptiTxStatistics, UINT32 clearStatsFlag)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( ptiTxStatistics )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_TX_STATISTICS, 
            &clearStatsFlag, sizeof(UINT32), 
            ptiTxStatistics, sizeof(TIWLN_TX_STATISTICS),&dwRetSize);
    }
    
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetBSSIDList(OS_802_11_BSSID_LIST_EX** ppBSSIDlist)
{
    tiUINT32    nSize = 0;
    if ( ppBSSIDlist == NULL)
        return TI_RESULT_FAILED;
    
    tiINT32 bRet = GetVariableLengthOID(TIWLN_802_11_BSSID_LIST, (tiVOID**)ppBSSIDlist, &nSize, 10000);
    
    return bRet;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetFullBSSIDList(OS_802_11_BSSID_LIST_EX** ppBSSIDlist)
{
    tiUINT32    nSize = 0;
    if ( ppBSSIDlist == NULL)
        return TI_RESULT_FAILED;
    
    tiINT32 bRet = GetVariableLengthOID(TIWLN_802_11_FULL_BSSID_LIST, (tiVOID**)ppBSSIDlist, &nSize, 10000);
    
    return bRet;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetCurrentAddress( OS_802_11_MAC_ADDRESS*    pCurrentAddr)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if (CheckObjectMemory(pCurrentAddr, sizeof (OS_802_11_MAC_ADDRESS)) )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_3_CURRENT_ADDRESS, NULL, 0, pCurrentAddr, sizeof( OS_802_11_MAC_ADDRESS ), &dwRetSize);
    }
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetDesiredSSID( OS_802_11_SSID*  pDesiredSSID )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if (CheckObjectMemory(pDesiredSSID, sizeof(OS_802_11_SSID)) )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_SSID_GET, NULL, 0, pDesiredSSID, sizeof(OS_802_11_SSID),&dwRetSize);
    }
    
    return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetCurrentSSID( OS_802_11_SSID*  pCurrentSSID )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if (CheckObjectMemory(pCurrentSSID, sizeof(OS_802_11_SSID)) )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_SSID_GET, NULL, 0, pCurrentSSID, sizeof(OS_802_11_SSID),&dwRetSize);
    }
    
    return dwRetValue;
}

/********************************************************************/

tiINT32 CTI_WLAN_AdapterAPI::SetSSID( tiUINT8*  pszSSIDname )
{
#ifdef _WINDOWS     // TRS:WDK
#endif
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pszSSIDname && m_pRegistry )
    {
        tiUINT32    uSize = 0;
        
#ifdef _WINDOWS     
#else  // __LINUX__
#ifdef  _UNICODE
        uSize = wcslen( pszSSIDname );
#else
        uSize = strlen((char *)pszSSIDname);  
#endif
        tiUINT32 dwNameSize = 0;
        
        if ( uSize <= MAX_SSID_LENGTH )
            dwNameSize = uSize * sizeof(tiUINT8);
        else
            dwNameSize = MAX_SSID_LENGTH * sizeof(tiUINT8);
#endif
        
        if (pszSSIDname != NULL)
        {
            
            OS_802_11_SSID ssid;
            memset( &ssid, 0, sizeof(OS_802_11_SSID) );
            
#ifdef _WINDOWS     // TRS:WDK
#else // __LINUX__
#ifdef  _UNICODE
            wcstombs((tiCHAR* )ssid.Ssid, pszSSIDname, MAX_SSID_LENGTH);
            ssid.SsidLength = MAX_SSID_LENGTH;
#else
            memcpy((tiCHAR*)ssid.Ssid, pszSSIDname, dwNameSize );
            ssid.SsidLength = dwNameSize;
#endif
#endif
            
            dwRetValue = tiIoCtrl(TIWLN_802_11_SSID_SET, &ssid, sizeof(OS_802_11_SSID),NULL, 0,&dwRetSize);
#ifdef _WINDOWS
#endif           
        }    
    }
    
    return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::SetBSSType( OS_802_11_NETWORK_MODE   uBSSType )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    dwRetValue = tiIoCtrl(TIWLN_802_11_INFRASTRUCTURE_MODE_SET, &uBSSType, sizeof(OS_802_11_NETWORK_MODE),NULL, 0,&dwRetSize);
#ifdef _WINDOWS
#endif           
    return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetBSSType( OS_802_11_NETWORK_MODE*  puBSSType )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    if (CheckObjectMemory( puBSSType, sizeof(OS_802_11_NETWORK_MODE)))
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_INFRASTRUCTURE_MODE_GET, NULL, 0, puBSSType, sizeof(OS_802_11_NETWORK_MODE),&dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetEAPType( OS_802_11_EAP_TYPES  uEAPType )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    if (uEAPType)
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_EAP_TYPE_SET, &uEAPType, sizeof(OS_802_11_EAP_TYPES),NULL, 0,&dwRetSize); 
    } 
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetEAPTypeDriver( OS_802_11_EAP_TYPES  uEAPType )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_EAP_TYPE_DRIVER_SET, &uEAPType, sizeof(OS_802_11_EAP_TYPES),NULL,0,&dwRetSize); 
    
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetEAPType( OS_802_11_EAP_TYPES* puEAPType )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_EAP_TYPE_GET, NULL, 0, puEAPType, sizeof(OS_802_11_ENCRYPTION_STATUS),&dwRetSize); 
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetEncryptionType( OS_802_11_ENCRYPTION_TYPES uEncryptType )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    OS_802_11_ENCRYPTION_STATUS uEncryptStatus = os802_11WEPDisabled;
    
    switch( uEncryptType )
    {
    case    OS_ENCRYPTION_TYPE_NONE:
        uEncryptStatus = os802_11WEPDisabled;
        break;
    case    OS_ENCRYPTION_TYPE_WEP:
        uEncryptStatus = os802_11Encryption1Enabled;
        break;
    case    OS_ENCRYPTION_TYPE_TKIP:
        uEncryptStatus = os802_11Encryption2Enabled;
        break;
    case    OS_ENCRYPTION_TYPE_AES:
        uEncryptStatus = os802_11Encryption3Enabled;
        break;
    default:
        uEncryptStatus = os802_11WEPDisabled;
    }
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_WEP_STATUS_SET, &uEncryptStatus, sizeof(OS_802_11_ENCRYPTION_STATUS),NULL, 0,&dwRetSize);
    
    return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetEncryptionType( OS_802_11_ENCRYPTION_TYPES* puEncryptType )
{
    tiUINT32 dwRetValue = TI_RESULT_INVALID_PARAMETER;
    tiUINT32 dwRetSize;
    
    if ( puEncryptType && CheckObjectMemory( puEncryptType, sizeof(OS_802_11_ENCRYPTION_TYPES)) )
    {
        OS_802_11_ENCRYPTION_STATUS uEncryptStatus;
        dwRetValue = tiIoCtrl(TIWLN_802_11_WEP_STATUS_GET, NULL, 0, &uEncryptStatus, sizeof(OS_802_11_ENCRYPTION_STATUS),&dwRetSize); 
        
        if ( dwRetValue == TI_RESULT_OK )
        {
            switch( uEncryptStatus )
            {
            case    os802_11WEPDisabled:
                *puEncryptType = OS_ENCRYPTION_TYPE_NONE;
                break;
            case    os802_11Encryption1Enabled:
                *puEncryptType =  OS_ENCRYPTION_TYPE_WEP;
                break;
            case    os802_11Encryption2Enabled:
                *puEncryptType = OS_ENCRYPTION_TYPE_TKIP;
                break;
            case    os802_11Encryption3Enabled:
                *puEncryptType =  OS_ENCRYPTION_TYPE_AES;
                break;
            default:
                dwRetValue = TI_RESULT_FAILED;
            } // switch
        }
    } // end of memory check
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetCredentials( tiCHAR* pszUserName, tiCHAR* pszPassword )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pszUserName != NULL )
    {
        tiUINT32    uSize = 0;
        
#ifdef  _UNICODE
        uSize = wcslen( pszUserName );
#else
        uSize = strlen(pszUserName);  
#endif
        tiUINT32 dwNameSize = uSize * sizeof(tiCHAR);
        
        if ( CheckObjectMemory( pszUserName, dwNameSize ))
            dwRetValue = tiIoCtrl(TIWLN_802_11_USER_ID_SET, pszUserName, dwNameSize + sizeof(tiCHAR),NULL, 0,&dwRetSize);
    }
    else
        dwRetValue = tiIoCtrl(TIWLN_802_11_USER_ID_SET, NULL, 0l,NULL, 0,&dwRetSize);
    
    
    if ( pszPassword != NULL )
    {
        tiUINT32    uSize = 0;
        
#ifdef  _UNICODE
        uSize = wcslen( pszPassword );
#else
        uSize = strlen(pszPassword);  
#endif
        tiUINT32 dwNameSize = uSize * sizeof(tiCHAR);
        
        if ( CheckObjectMemory( pszPassword, dwNameSize ))
            dwRetValue = tiIoCtrl(TIWLN_802_11_USER_PASSWORD_SET, pszPassword, dwNameSize + sizeof(tiCHAR),NULL, 0,&dwRetSize);
    }
    else
        dwRetValue = tiIoCtrl(TIWLN_802_11_USER_PASSWORD_SET, NULL, 0l,NULL, 0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetPSK( tiCHAR* pszPSK )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pszPSK != NULL )
    {
        tiUINT32    uSize = 0;
        
#ifdef  _UNICODE
        uSize = wcslen( pszPSK );
#else
        uSize = strlen(pszPSK);  
#endif
        tiUINT32 dwNameSize = uSize * sizeof(tiCHAR);
        
        if ( CheckObjectMemory( pszPSK, dwNameSize ))
            dwRetValue = tiIoCtrl(TIWLN_802_11_PSK_SET, pszPSK, dwNameSize + sizeof(tiCHAR),NULL, 0,&dwRetSize);
    }
    else
        dwRetValue = tiIoCtrl(TIWLN_802_11_PSK_SET, NULL, 0l,NULL, 0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetCertParamsSHA1( TI_SHA1_HASH* pSha1Hash, tiBOOL bValidateServerCert )
{
    tiUINT32 dwRetValue = TI_RESULT_INVALID_PARAMETER;
    tiUINT32 dwRetSize;
    
    if ( pSha1Hash && CheckObjectMemory(pSha1Hash, sizeof(TI_SHA1_HASH)))
    {
        tiUINT32  uSize = sizeof(TI_SHA1_HASH) + sizeof(tiBOOL);
        tiUINT8*  pByte = new tiUINT8[uSize];
        if ( pByte == NULL )
            return TI_RESULT_NOT_ENOUGH_MEMORY;
        
        tiUINT8* pBufferTmp = pByte;
        
        memset(pByte, 0, uSize);
        memcpy(pBufferTmp, &bValidateServerCert, sizeof(tiBOOL));
        pBufferTmp += sizeof(tiBOOL);
        memcpy(pBufferTmp, pSha1Hash, sizeof(TI_SHA1_HASH));
        
        dwRetValue = tiIoCtrl(TIWLN_802_11_CERT_PARAMS_SHA1_SET, pByte, uSize,NULL, 0,&dwRetSize);
        
        delete [] pByte;
    }
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetCertParamsFileName( tiCHAR* pszFileName, tiBOOL bValidateServerCert )
{
    tiUINT32 dwRetValue = TI_RESULT_INVALID_PARAMETER;
    tiUINT32 dwRetSize;
    
    if ( pszFileName != NULL )
    {
        tiUINT32    uSize = 0;
        
#ifdef  _UNICODE
        uSize = wcslen( pszFileName );
#else
        uSize = strlen(pszFileName);  
#endif
        tiUINT32 dwSize = (uSize + 1) * sizeof(tiCHAR) + sizeof(tiBOOL);
        
        tiUINT8*  pByte = new tiUINT8[dwSize];
        
        if ( pByte == NULL )
            return TI_RESULT_NOT_ENOUGH_MEMORY;
        
        tiUINT8* pBufferTmp = pByte;
        
        memset(pByte, 0, dwSize);
        memcpy(pBufferTmp, &bValidateServerCert, sizeof(tiBOOL));
        pBufferTmp += sizeof(tiBOOL);
        memcpy(pBufferTmp, pszFileName, uSize);
        
        dwRetValue = tiIoCtrl(TIWLN_802_11_CERT_PARAMS_FILE_NAME_SET, pByte, dwSize,NULL, 0,&dwRetSize);
        
        delete [] pByte;
    }
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::AddWEPKey( OS_802_11_WEP*    pWEP )
{
    tiUINT32 dwRetValue = TI_RESULT_INVALID_PARAMETER;
    tiUINT32 dwRetSize;
    
    if (CheckObjectMemory(pWEP, sizeof(OS_802_11_WEP)))
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_ADD_WEP, pWEP, sizeof(OS_802_11_WEP),NULL, 0,&dwRetSize);
        
    }
    return dwRetValue;
}
/********************************************************************/
tiINT32                 
CTI_WLAN_AdapterAPI::RemoveWEPKey( tiUINT32 uKeyIndex )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_REMOVE_WEP, &uKeyIndex, sizeof(tiUINT32),NULL, 0,&dwRetSize);
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetWPAOptions( tiUINT32 fWPA_options)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_WPA_OPTIONS_SET, &fWPA_options, sizeof(tiUINT32),NULL, 0,&dwRetSize);
    return dwRetValue;
}
/********************************************************************/
tiINT32 CTI_WLAN_AdapterAPI::GetWPAOptions( tiUINT32 * fWPA_options)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if (CheckObjectMemory(fWPA_options, sizeof(tiUINT32)))
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_WPA_OPTIONS_GET, NULL, 0, fWPA_options, sizeof(tiUINT32),&dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/
tiINT32                 
CTI_WLAN_AdapterAPI::SetPMKIDmap(OS_802_11_PMKID*  pPMKIDMap)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if (CheckObjectMemory(pPMKIDMap, pPMKIDMap->Length))
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_PMKID_SET, pPMKIDMap, pPMKIDMap->Length,NULL, 0,&dwRetSize);
    }
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::AddKey( OS_802_11_KEY*   pKey )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if (CheckObjectMemory(pKey, pKey->Length))
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_ADD_KEY, pKey, pKey->Length,NULL, 0,&dwRetSize);
    }
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::RemoveKey( OS_802_11_REMOVE_KEY* pRemoveKey  )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if (CheckObjectMemory(pRemoveKey, sizeof(OS_802_11_REMOVE_KEY)))
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_REMOVE_KEY, pRemoveKey, sizeof(OS_802_11_REMOVE_KEY),NULL, 0,&dwRetSize);
    }
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::RegisterEvent( IPC_EVENT_PARAMS* pEventParams )
{
#ifdef _WINDOWS     // TRS:WDK
#else
    return m_pIPCmod->IPC_RegisterEvent(pEventParams);
#endif
}
/********************************************************************/
tiINT32                 
CTI_WLAN_AdapterAPI::UnRegisterEvent( IPC_EVENT_PARAMS* pEventParams/*tiINT32 iRegisterID*/ ) 
{
#ifdef _WINDOWS     // TRS:WDK
#else
    return m_pIPCmod->IPC_UnRegisterEvent(pEventParams);
#endif
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::ConfigPowerManagement( OS_802_11_POWER_PROFILE thePowerMgrProfile )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_POWER_MGR_PROFILE, &thePowerMgrProfile, sizeof(OS_802_11_POWER_PROFILE),NULL,0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetAssociationInfo( OS_802_11_ASSOCIATION_INFORMATION** ppInfo )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    
    tiUINT32 nSize = 0;
    
    if ( ppInfo == NULL)
        return TI_RESULT_FAILED;
    
    dwRetValue = GetVariableLengthOID(TIWLN_802_11_ASSOCIATION_INFORMATION, (tiVOID**)ppInfo, &nSize, 10000);
    if ( TI_SUCCEEDED (dwRetValue) )
    {
        if (nSize == 0 || nSize < sizeof(OS_802_11_ASSOCIATION_INFORMATION) || *ppInfo == NULL )
            dwRetValue = TI_RESULT_FAILED;
        
    }
    return dwRetValue;
}

/********************************************************************/
///////////////////////////////////////////////////////////////////////
// Assorted utility functions

// This function queries for a variable length OID. Starting from a
// suggested size (nNextAllocation), it keeps querying until the size
// requirement is met. It does this repeatedly rather than once because
// the size requirement for an OID can vary from one call to the next, and
// also because some adapters don't report the correct required size value.
tiINT32
CTI_WLAN_AdapterAPI::GetVariableLengthOID(tiUINT32 oid, tiVOID** pp, tiUINT32* pnSize, tiUINT32 nNextAllocation)
{
    tiVOID*     p = NULL;
    tiUINT32    nSize;
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    // reset return values
    *pp = NULL;
    *pnSize = 0;
    
    // query until we have a big enough buffer or get an error
    for( ; ; )
    {
        // try next allocation
        nSize = nNextAllocation;
        
        if ( nSize )
        {
            p = malloc(nSize);
            if (!p) 
                return TI_RESULT_FAILED;
            memset(p,0,nSize);
        }
        
        
        // get OID
        dwRetValue = tiIoCtrl(oid, p, nSize, p, nSize, &dwRetSize);
        
        if( dwRetSize && nNextAllocation <= nSize && nSize != 0 )
            break;
        else
            nNextAllocation = dwRetSize;
        
        // failed: free buffer
        if ( p )
        {
            free(p);
            p = NULL;
        }
        // if buffer overflow but new size is less than we used, return error
        // NOTE: this would be a NIC bug and we have to avoid an infinite loop
        if( nNextAllocation <= nSize )
            return TI_RESULT_FAILED;
    }
    
    // return pointer, size
    *pp = p;
    *pnSize = nNextAllocation;
    
    // success
    return TI_RESULT_OK;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::StartScan( scan_Params_t *pScanParams )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_START_APP_SCAN_SET, pScanParams, sizeof(scan_Params_t),NULL,0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::StopScan( )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_STOP_APP_SCAN_SET, NULL, 0,NULL,0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetScanPolicy( UINT8* buffer, UINT16 bufferLength )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_SCAN_POLICY_PARAM_SET, buffer, bufferLength,NULL,0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetScanBssList( bssList_t* bssList )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_SCAN_BSS_LIST_GET, NULL, 0, bssList, sizeof(bssList_t),&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32 
CTI_WLAN_AdapterAPI::PollApPackets( )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_POLL_AP_PACKETS, NULL, 0, NULL, 0,&dwRetSize);
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::PollApPacketsFromAC( tiUINT32 AC )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_POLL_AP_PACKETS_FROM_AC, &AC, sizeof(tiUINT32),NULL, 0,&dwRetSize);
    return dwRetValue;
}
/********************************************************************/
tiINT32 
CTI_WLAN_AdapterAPI::SetTrafficIntensityThresholds ( OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS* pTrafficThresholds )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if (CheckObjectMemory(pTrafficThresholds, sizeof(OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS)))
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_SET_TRAFFIC_INTENSITY_THRESHOLDS, pTrafficThresholds , sizeof(OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS),NULL, 0,&dwRetSize);
    }
    return dwRetValue;
}
/********************************************************************/
tiINT32 
CTI_WLAN_AdapterAPI::GetTrafficIntensityThresholds ( OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS* pTrafficThresholds )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_GET_TRAFFIC_INTENSITY_THRESHOLDS, pTrafficThresholds , sizeof(OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS), pTrafficThresholds , sizeof(OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS),&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32 
CTI_WLAN_AdapterAPI::ToggleTrafficIntensityEvents ( tiUINT32 NewStatus )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    tiUINT32 localVal = NewStatus;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_TOGGLE_TRAFFIC_INTENSITY_EVENTS, &localVal , sizeof(tiUINT32),NULL, 0,&dwRetSize);
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetDTagToAcMappingTable( acTrfcType_e* pDtagToAcTable )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    txDataQosParams_t	txDataQosParams;
    
    if (CheckObjectMemory(pDtagToAcTable, sizeof(acTrfcType_e)))
    {
        for (int i=0; i<MAX_NUM_OF_802_1d_TAGS; i++)
        {
            if (pDtagToAcTable[i] < MAX_NUM_OF_AC)
            {
                txDataQosParams.qosParams.tag_ToAcClsfrTable[i] = pDtagToAcTable[i];
            }
            else
            {
                return dwRetValue;
            }
	}

        dwRetValue = tiIoCtrl(TIWLN_802_11_SET_DTAG_TO_AC_MAPPING_TABLE, &txDataQosParams, sizeof(txDataQosParams_t), NULL, 0,&dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetVAD( txDataVadTimerParams_t* pVadTimer )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if (CheckObjectMemory(pVadTimer, sizeof(txDataVadTimerParams_t)))
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_SET_VAD, pVadTimer, sizeof(txDataVadTimerParams_t), NULL, 0,&dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetVAD( txDataVadTimerParams_t* pVadTimer )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if (CheckObjectMemory(pVadTimer, sizeof(txDataVadTimerParams_t)))
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_GET_VAD, pVadTimer, sizeof(txDataVadTimerParams_t), pVadTimer, sizeof(txDataVadTimerParams_t),&dwRetSize);
    }
    return dwRetValue;	
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetQosParameters( OS_802_11_QOS_PARAMS* pQosParams )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if (CheckObjectMemory(pQosParams, sizeof(OS_802_11_QOS_PARAMS)))
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_SET_QOS_PARAMS, pQosParams, sizeof(OS_802_11_QOS_PARAMS),NULL, 0,&dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetRxTimeOut( OS_802_11_QOS_RX_TIMEOUT_PARAMS* pRxTimeOut )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if (CheckObjectMemory(pRxTimeOut, sizeof(OS_802_11_QOS_PARAMS)))
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_SET_RX_TIMEOUT, pRxTimeOut, sizeof(OS_802_11_QOS_RX_TIMEOUT_PARAMS),NULL, 0,&dwRetSize);
    }
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetAPQosParameters( OS_802_11_AC_QOS_PARAMS* pACQosParams)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if (CheckObjectMemory(pACQosParams, sizeof(OS_802_11_AC_QOS_PARAMS)))
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_GET_AP_QOS_PARAMS, pACQosParams, sizeof(OS_802_11_AC_QOS_PARAMS), pACQosParams, sizeof(OS_802_11_AC_QOS_PARAMS),&dwRetSize);
    }
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetUserPriorityOfStream( STREAM_TRAFFIC_PROPERTIES* streamProperties)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if (CheckObjectMemory(streamProperties, sizeof(STREAM_TRAFFIC_PROPERTIES)))
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_GET_USER_PRIORITY_OF_STREAM, streamProperties, sizeof(STREAM_TRAFFIC_PROPERTIES), streamProperties, sizeof(STREAM_TRAFFIC_PROPERTIES),&dwRetSize);
    }
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetAPQosCapabilitesParameters( OS_802_11_AP_QOS_CAPABILITIES_PARAMS* pAPQosCapabiltiesParams )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_GET_AP_QOS_CAPABILITIES, NULL, 0, pAPQosCapabiltiesParams, sizeof(OS_802_11_AP_QOS_CAPABILITIES_PARAMS),&dwRetSize);
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::AddTspec ( OS_802_11_QOS_TSPEC_PARAMS* pTspecParams)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_ADD_TSPEC, pTspecParams, sizeof(OS_802_11_QOS_TSPEC_PARAMS),NULL, 0,&dwRetSize);
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetTspecParameters( OS_802_11_QOS_TSPEC_PARAMS* pTspecParams)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_GET_TSPEC_PARAMS, pTspecParams, sizeof(OS_802_11_QOS_TSPEC_PARAMS), pTspecParams, sizeof(OS_802_11_QOS_TSPEC_PARAMS),&dwRetSize);
    return dwRetValue;
}/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::DeleteTspec( OS_802_11_QOS_DELETE_TSPEC_PARAMS* pDelTspecParams)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_DELETE_TSPEC, pDelTspecParams, sizeof(OS_802_11_QOS_DELETE_TSPEC_PARAMS),NULL, 0,&dwRetSize);
    return dwRetValue;
}/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI:: GetCurrentACStatus( OS_802_11_AC_UPSD_STATUS_PARAMS *pAcStatusParams)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_GET_CURRENT_AC_STATUS, pAcStatusParams, sizeof(OS_802_11_AC_UPSD_STATUS_PARAMS), pAcStatusParams, sizeof(OS_802_11_AC_UPSD_STATUS_PARAMS),&dwRetSize);
    return dwRetValue;
}/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI:: SetMediumUsageThreshold(OS_802_11_THRESHOLD_CROSS_PARAMS* pThresholdCrossParams)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_SET_MEDIUM_USAGE_THRESHOLD, pThresholdCrossParams, sizeof(OS_802_11_THRESHOLD_CROSS_PARAMS),NULL, 0,&dwRetSize);
    return dwRetValue;
}/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI:: SetPhyRateThreshold(OS_802_11_THRESHOLD_CROSS_PARAMS* pThresholdCrossParams)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_SET_PHY_RATE_THRESHOLD, pThresholdCrossParams, sizeof(OS_802_11_THRESHOLD_CROSS_PARAMS),NULL, 0,&dwRetSize);
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI:: GetMediumUsageThreshold(OS_802_11_THRESHOLD_CROSS_PARAMS* pThresholdCrossParams)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_GET_MEDIUM_USAGE_THRESHOLD, pThresholdCrossParams, sizeof(OS_802_11_THRESHOLD_CROSS_PARAMS), pThresholdCrossParams, sizeof(OS_802_11_THRESHOLD_CROSS_PARAMS),&dwRetSize);
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI:: GetPhyRateThreshold(OS_802_11_THRESHOLD_CROSS_PARAMS* pThresholdCrossParams)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_GET_PHY_RATE_THRESHOLD, pThresholdCrossParams, sizeof(OS_802_11_THRESHOLD_CROSS_PARAMS), pThresholdCrossParams, sizeof(OS_802_11_THRESHOLD_CROSS_PARAMS),&dwRetSize);
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI:: GetDesiredPsMode(OS_802_11_QOS_DESIRED_PS_MODE* pDesiredPsMode)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_GET_DESIRED_PS_MODE, NULL, 0, pDesiredPsMode, sizeof(OS_802_11_QOS_DESIRED_PS_MODE),&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::ConfigTxClassifier(tiUINT32 inParamsBuffLen, tiUINT8  *pInParamsBuff)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pInParamsBuff && CheckObjectMemory(pInParamsBuff, inParamsBuffLen) )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_CONFIG_TX_CLASS, pInParamsBuff, inParamsBuffLen,NULL, 0,&dwRetSize);
    }
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::RemoveClassifierEntry(clsfr_tableEntry_t *pClsfrEntry)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if (pClsfrEntry)
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_REMOVE_CLSFR_ENTRY, pClsfrEntry, sizeof(clsfr_tableEntry_t),NULL, 0,&dwRetSize);
    }
    
    return dwRetValue;
}
/**********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetClsfrType (clsfrTypeAndSupport *currClsfrType)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_GET_CLSFR_TYPE, NULL, 0, currClsfrType, sizeof(clsfrTypeAndSupport),&dwRetSize);      
    return dwRetValue;
}
/********************************************************************/
tiINT32 
CTI_WLAN_AdapterAPI::GetDriverCapabilities (OS_802_11_DRIVER_CAPABILITIES* pDriverCapabilities )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_GET_DRIVERS_CAPABILITIES, pDriverCapabilities, sizeof(OS_802_11_DRIVER_CAPABILITIES), pDriverCapabilities, sizeof(OS_802_11_DRIVER_CAPABILITIES),&dwRetSize);
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetSelectedBSSIDInfo(OS_802_11_BSSID_EX  *pSelectedBSSIDInfo)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_GET_SELECTED_BSSID_INFO, pSelectedBSSIDInfo, sizeof(OS_802_11_BSSID_EX), pSelectedBSSIDInfo, sizeof(OS_802_11_BSSID_EX),&dwRetSize);
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetPrimaryBSSIDInfo(OS_802_11_BSSID_EX  *pSelectedBSSIDInfo)
{
	tiUINT32 dwRetValue = TI_RESULT_FAILED;
	tiUINT32 dwRetSize;
    tiUINT32 outBufLen;

    outBufLen = pSelectedBSSIDInfo->Length; //sizeof(OS_802_11_BSSID_EX) + sizeof(OS_802_11_FIXED_IEs) + 300;

   dwRetValue = tiIoCtrl(TIWLN_802_11_GET_PRIMARY_BSSID_INFO, pSelectedBSSIDInfo, outBufLen, pSelectedBSSIDInfo, outBufLen, &dwRetSize);
   return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::EnableDisableRxDataFilters(tiBOOL enabled)
{
    tiUINT32 dwRetSize;

    return tiIoCtrl(TIWLN_ENABLE_DISABLE_RX_DATA_FILTERS, &enabled, sizeof(enabled), NULL, 0, &dwRetSize);
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetRxDataFiltersStatistics(TIWLAN_DATA_FILTER_STATISTICS * pStatistics)
{
    tiUINT32 dwRetSize;

    return tiIoCtrl(TIWLN_GET_RX_DATA_FILTERS_STATISTICS, NULL, 0, pStatistics, sizeof(TIWLAN_DATA_FILTER_STATISTICS), &dwRetSize);
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetPowerConsumptionStatistics(PowerConsumptionTimeStat_t * pStatistics)
{
    tiUINT32 dwRetSize;

    return tiIoCtrl(TIWLN_GET_POWER_CONSUMPTION_STATISTICS, NULL, 0, pStatistics, sizeof(PowerConsumptionTimeStat_t), &dwRetSize);
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::AddRxDataFilter(TIWLAN_DATA_FILTER_REQUEST * pRequest)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;

    if ( pRequest )
    {
        dwRetValue = tiIoCtrl(TIWLN_ADD_RX_DATA_FILTER, pRequest, sizeof(TIWLAN_DATA_FILTER_REQUEST), NULL, 0, &dwRetSize);
    }

    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::RemoveRxDataFilter(TIWLAN_DATA_FILTER_REQUEST * pRequest)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;

    if ( pRequest )
    {
        dwRetValue = tiIoCtrl(TIWLN_REMOVE_RX_DATA_FILTER, pRequest, sizeof(TIWLAN_DATA_FILTER_REQUEST), NULL, 0, &dwRetSize);
    }

    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetTxPowerDbm(tiUINT8 uTxPower)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_TX_POWER_DBM_SET, &uTxPower, sizeof(UINT8),NULL, 0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetPowerMode(OS_802_11_POWER_PROFILE uPower )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_POWER_MODE_SET, &uPower, sizeof(OS_802_11_POWER_PROFILE),NULL, 0,&dwRetSize);
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetPowerMode(OS_802_11_POWER_PROFILE* puPower)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_POWER_MODE_GET, NULL, 0, puPower, sizeof(OS_802_11_POWER_PROFILE),&dwRetSize);
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetPowerLevelPS(OS_802_11_POWER_LEVELS uPower )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_POWER_LEVEL_PS_SET, &uPower, sizeof(OS_802_11_POWER_LEVELS),NULL, 0,&dwRetSize);
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetPowerLevelPS( OS_802_11_POWER_LEVELS* puPower)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_POWER_LEVEL_PS_GET, NULL, 0, puPower, sizeof(OS_802_11_POWER_LEVELS),&dwRetSize);
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetPowerLevelDefault(OS_802_11_POWER_LEVELS uPower )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_POWER_LEVEL_DEFAULT_SET, &uPower, sizeof(OS_802_11_POWER_LEVELS),NULL, 0,&dwRetSize);
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetPowerLevelDefault( OS_802_11_POWER_LEVELS* puPower)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_POWER_LEVEL_DEFAULT_GET, NULL, 0, puPower, sizeof(OS_802_11_POWER_LEVELS),&dwRetSize);
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetPowerLevelDozeMode(OS_802_11_POWER_PROFILE uPower )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_POWER_LEVEL_DOZE_MODE_SET, &uPower, sizeof(OS_802_11_POWER_PROFILE),NULL, 0,&dwRetSize);
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetPowerLevelDozeMode( OS_802_11_POWER_PROFILE* puPower)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_POWER_LEVEL_DOZE_MODE_GET, NULL, 0, puPower, sizeof(OS_802_11_POWER_PROFILE),&dwRetSize);
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetBeaconFilterDesiredState( OS_802_11_BEACON_FILTER_MODE   uBeaconFilterMode)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_BEACON_FILTER_DESIRED_STATE_SET, &uBeaconFilterMode, sizeof(OS_802_11_BEACON_FILTER_MODE),NULL, 0,&dwRetSize);
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetBeaconFilterDesiredState( tiUINT8* pDesiredState)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_BEACON_FILTER_DESIRED_STATE_GET, NULL, 0, pDesiredState, sizeof(UINT8),&dwRetSize);
    return dwRetValue;
}


/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetTxPowerLevel(tiCHAR* puTxPower )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( puTxPower )
    {
		dwRetValue = tiIoCtrl(TIWLN_802_11_TX_POWER_LEVEL_GET, NULL, 0, puTxPower, sizeof(TIWLAN_POWER_LEVEL_TABLE),&dwRetSize);      
    }
    
    return dwRetValue;
}


/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetTxPowerDbm(tiCHAR* puTxPower )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( puTxPower )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_TX_POWER_DBM_GET, NULL, 0, puTxPower, sizeof(UINT8),&dwRetSize);      
    }
    
    return dwRetValue;
}

/********************************************************************/

tiINT32  
CTI_WLAN_AdapterAPI::Set4XState(tiBOOL bStatus)
{
    tiUINT32 bRet = TI_RESULT_FAILED;
    
    bRet = m_pRegistry->PutDW(_T("Mode4x"), bStatus);
    
    return bRet;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::Get4XState(tiBOOL* lpbStatus)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_4XACTIVESTATE_GET, NULL, 0, lpbStatus, sizeof(tiBOOL),&dwRetSize);
    
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetDesiredRate(tiUINT32* puDesiredRates)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( puDesiredRates )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_DESIRED_RATES_GET, NULL, 0, puDesiredRates, sizeof(tiUINT32),&dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetCurrentRate(tiUINT32* puCurrentRates)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( puCurrentRates )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_CURRENT_RATES_GET, NULL, 0, puCurrentRates, sizeof(tiUINT32),&dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetFragmentThreshold(tiUINT32 dwFragmentThreshold)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_FRAGMENTATION_THRESHOLD_SET, &dwFragmentThreshold, sizeof(tiUINT32),NULL, 0,&dwRetSize);
    
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetFragmentThreshold(tiUINT32* lpdwFragmentThreshold)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( lpdwFragmentThreshold )
    {
        dwRetValue = tiIoCtrl( TIWLN_802_11_FRAGMENTATION_THRESHOLD_GET, NULL, 0,
            lpdwFragmentThreshold, sizeof(tiUINT32),&dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::SetRTSThreshold(tiUINT32 uRTSThreshold)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_RTS_THRESHOLD_SET, &uRTSThreshold, sizeof(tiUINT32),NULL, 0,&dwRetSize);
    
    return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetRTSThreshold(tiUINT32* puRTSThreshold )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( puRTSThreshold )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_RTS_THRESHOLD_GET, NULL, 0, puRTSThreshold, sizeof(tiUINT32),&dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::SetShortPreamble(tiUINT32 uShortPreamble)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_SHORT_PREAMBLE_SET, &uShortPreamble, sizeof(tiUINT32),NULL, 0,&dwRetSize);
    
    return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetShortPreamble(tiUINT32* puShortPreamble)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( puShortPreamble )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_SHORT_PREAMBLE_GET, NULL, 0, puShortPreamble, sizeof(tiUINT32),&dwRetSize);
        
    }
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetShortRetry(tiUINT32 uShortRetry)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_SHORT_RETRY_SET, &uShortRetry, sizeof(tiUINT32),NULL, 0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetShortRetry(tiUINT32* puShortRetry)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( puShortRetry )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_SHORT_RETRY_GET, NULL, 0, puShortRetry, sizeof(tiUINT32),&dwRetSize);
    }
    return dwRetValue;
}
/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::SetLongRetry(tiUINT32 uLongRetry)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_LONG_RETRY_SET, &uLongRetry, sizeof(tiUINT32),NULL, 0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetLongRetry(tiUINT32* puLongRetry)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( puLongRetry )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_LONG_RETRY_GET, NULL, 0, puLongRetry, sizeof(tiUINT32),&dwRetSize);
    }
    return dwRetValue;
}
/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetSupportedNetworkTypes(OS_802_11_NETWORK_TYPE* pNetTypeLst, tiUINT32 uMaxNetTypes )
{
    tiUINT32 dwRetValue = TI_RESULT_INVALID_PARAMETER;
    tiUINT32 dwRetSize;
    tiUINT32    uSizeList = sizeof(OS_802_11_NETWORK_TYPE)*uMaxNetTypes;
    
    if ( pNetTypeLst && CheckObjectMemory(pNetTypeLst, uSizeList) )
    {
        tiUINT32 dwNetTypesSize = sizeof(OS_802_11_NETWORK_TYPE_LIST) + (uMaxNetTypes - 1) * sizeof(OS_802_11_NETWORK_TYPE);
        
        OS_802_11_NETWORK_TYPE_LIST*    pNetworkTypeList = NULL;
        pNetworkTypeList = (OS_802_11_NETWORK_TYPE_LIST*) new tiUINT8[dwNetTypesSize];
        
        if( !pNetworkTypeList )
            return TI_RESULT_NOT_ENOUGH_MEMORY;
        
        memset(pNetworkTypeList, 0, dwNetTypesSize );
        
        pNetworkTypeList->NumberOfItems = uMaxNetTypes;
        
        dwRetValue = tiIoCtrl(TIWLN_802_11_NETWORK_TYPES_SUPPORTED, NULL, 0, pNetTypeLst, dwNetTypesSize,&dwRetSize);
        
        if ( dwRetSize )
        {
            dwRetValue = TI_RESULT_OK;
            memcpy(pNetTypeLst, pNetworkTypeList, uSizeList);
        } 
        
        delete [] pNetworkTypeList;
    }
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetNetworkTypeInUse( OS_802_11_NETWORK_TYPE uNetType )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_NETWORK_TYPE_IN_USE_SET, &uNetType, sizeof(OS_802_11_NETWORK_TYPE),NULL, 0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetNetworkTypeInUse( OS_802_11_NETWORK_TYPE*   puNetType )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( puNetType )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_NETWORK_TYPE_IN_USE_GET, NULL, 0, puNetType, sizeof(OS_802_11_NETWORK_TYPE),&dwRetSize);
    }
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetBSSID( OS_802_11_MAC_ADDRESS* pAddrBSSID )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pAddrBSSID && CheckObjectMemory(pAddrBSSID, sizeof(OS_802_11_MAC_ADDRESS)) )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_BSSID_GET, NULL, 0, pAddrBSSID, sizeof( OS_802_11_MAC_ADDRESS ),&dwRetSize);
    }
    return dwRetValue;
}


/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::SetBSSID( OS_802_11_MAC_ADDRESS* lpAddrBSSID )
{
    tiUINT32 dwRetValue;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_BSSID_SET, lpAddrBSSID, sizeof( OS_802_11_MAC_ADDRESS ),NULL, 0,&dwRetSize);
    
    return dwRetValue;
}



/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetRSSITrigger( tiBOOL bRSSItr )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_RSSI_TRIGGER_SET, &bRSSItr, sizeof(tiBOOL),NULL, 0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetRSSITrigger( tiBOOL* pbRSSItr )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_RSSI_TRIGGER_GET, NULL, 0, pbRSSItr, sizeof( tiBOOL ),&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetAntennaDiversityParams( PTIWLAN_ANT_DIVERSITY pAntennaDiversityOptions )
{
    tiINT32    bRet    = TI_RESULT_FAILED;
    tiUINT32   dwRetSize;
    
    bRet = tiIoCtrl(TIWLAN_802_11_ANTENNA_DIVERSITY_PARAM_SET, pAntennaDiversityOptions, sizeof(TIWLAN_ANT_DIVERSITY), NULL, 0, &dwRetSize);
    
    return bRet;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetWEPStatus(tiUINT32 dwWEPStatus)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_WEP_STATUS_SET, &dwWEPStatus, sizeof(tiUINT32),NULL, 0,&dwRetSize);
    
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetWEPStatus(tiUINT32*   lpdwWEPStatus)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( lpdwWEPStatus )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_WEP_STATUS_GET, NULL, 0, lpdwWEPStatus, sizeof(tiUINT32),&dwRetSize);
    }
    return dwRetValue;
}
/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::SetDesiredChannel( tiUINT32 dwDesiredChannel )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_DESIRED_CHANNEL_SET, &dwDesiredChannel, sizeof(tiUINT32),NULL, 0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetDesiredChannel( tiUINT32*   lpdwDesiredChannel )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( lpdwDesiredChannel )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_DESIRED_CHANNEL_GET, NULL, 0, lpdwDesiredChannel, sizeof(tiUINT32),&dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetCurrentChannel( tiUINT32* puCurrentChannel )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( puCurrentChannel )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_CHANNEL_GET, NULL, 0, puCurrentChannel, sizeof(tiUINT32),&dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::SetBtCoeEnable( tiUINT32 uModeEnable) 
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(BT_COEXSISTANCE_SET_ENABLE, &uModeEnable, sizeof(tiUINT32),NULL,0,&dwRetSize);
    
    return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::SetBtCoeRate( tiUINT8 *pRate )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(BT_COEXSISTANCE_SET_RATE, pRate, sizeof(tiUINT8)*NUM_OF_RATES_IN_SG,NULL,0,&dwRetSize);
    
    return dwRetValue;
}

/********************************************************************/


tiINT32
CTI_WLAN_AdapterAPI::SetBtCoeConfig( tiUINT32 *pConfig )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    
    dwRetValue = tiIoCtrl(BT_COEXSISTANCE_SET_CONFIG, pConfig, sizeof(tiUINT32) * NUM_OF_CONFIG_PARAMS_IN_SG,NULL,0,NULL);
    
    return dwRetValue;
}


tiINT32
CTI_WLAN_AdapterAPI::SetBtCoeGetStatus( tiUINT32 *pStatus )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(BT_COEXSISTANCE_GET_STATUS, NULL, 0, pStatus, sizeof(tiUINT32) * NUM_OF_STATUS_PARAMS_IN_SG,&dwRetSize);
    
    return dwRetValue;
}



#ifdef TI_DBG

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::DisplayStats( tiUINT8*  puDbgBuffer, tiUINT32 uBuffSize)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( puDbgBuffer && CheckObjectMemory(puDbgBuffer, uBuffSize) )
    {
        dwRetValue = tiIoCtrl( TIWLN_DISPLAY_STATS, puDbgBuffer, uBuffSize,NULL, 0,&dwRetSize);
    }
    
    return dwRetValue;
}

/********************************************************************/
tiINT32 
CTI_WLAN_AdapterAPI::SetReportModule( tiUINT8 *pData )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize = 0;
    
    if ( pData && CheckObjectMemory(pData, WLAN_MAX_LOG_MODULES) )
    {
        dwRetValue = tiIoCtrl( TIWLN_REPORT_MODULE_SET, pData, WLAN_MAX_LOG_MODULES, NULL, 0, &dwRetSize);
    }
    
    return dwRetValue;
}


/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetReportModule( tiUINT8* pData )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pData )
    {
        dwRetValue = tiIoCtrl(TIWLN_REPORT_MODULE_GET, NULL, 0, pData, WLAN_MAX_LOG_MODULES, &dwRetSize);
    }
    
    return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::SetReportSeverity( tiUINT8  *pData )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pData && CheckObjectMemory(pData, WLAN_MAX_SEVERITIES) )
    {
        dwRetValue = tiIoCtrl(TIWLN_REPORT_SEVERITY_SET, pData, WLAN_MAX_SEVERITIES, NULL, 0, &dwRetSize);
    }
    
    return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetReportSeverity( tiUINT8* pData )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pData )
    {
        dwRetValue = tiIoCtrl(TIWLN_REPORT_SEVERITY_GET, NULL, 0, pData, WLAN_MAX_SEVERITIES, &dwRetSize);
    }
    
    return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::SetOsDbgState( tiUINT32 uData )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_OS_DBG_STATE_SET, &uData, sizeof(tiUINT32), NULL, 0, &dwRetSize);
    
    return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetOsDbgState( tiUINT32* puData )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( puData )
    {
        dwRetValue = tiIoCtrl(TIWLN_OS_DBG_STATE_GET, NULL, 0, puData, sizeof(tiUINT32), &dwRetSize);
    }
    
    return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::SetReportPPMode( tiUINT32  uData )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_REPORT_PPMODE_SET, &uData, sizeof(tiUINT32), NULL, 0, &dwRetSize);
    
    return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetDebugBuffer( tiUINT8* pBuffer, tiUINT32  dwLenght)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pBuffer && CheckObjectMemory(pBuffer, sizeof(tiUINT8)*dwLenght) )
    {
        dwRetValue = tiIoCtrl(TIWLN_GET_DBG_BUFFER, NULL, 0, pBuffer, sizeof(tiUINT8)*dwLenght,&dwRetSize);
        
    }
    return dwRetValue;
}


#ifdef DRIVER_PROFILING
tiINT32                 
CTI_WLAN_AdapterAPI::ProfileReport()
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;

    dwRetValue =  tiIoCtrl(TIWLAN_PROFILING_REPORT,NULL, 0,NULL,0,&dwRetSize);

    return dwRetValue;
}

tiINT32
CTI_WLAN_AdapterAPI::CpuEstimatorCommand(tiUINT8 uType, tiUINT32 uData)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    tiUINT32 dwCommandParam;

    /* set command param: type | (24 LSB) data */
    dwCommandParam = (uData & 0xFFFFFF) | (((tiUINT32)uType) << 24);

    //printf("CpuEstimatorCommand: type=%d, data =%d, dwCommandParam = %x\n", uType, uData, dwCommandParam);
    dwRetValue =  tiIoCtrl(TIWLAN_PROFILING_CPU_ESTIMATOR_CMD,&dwCommandParam, sizeof(dwCommandParam),NULL, 0,&dwRetSize);

    return dwRetValue;
}
#endif

#endif //TI_DBG
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetSupportedRates( tiUINT8* pSupportedRatesLst, tiUINT32 uBufLength)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pSupportedRatesLst && CheckObjectMemory(pSupportedRatesLst, uBufLength) )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_SUPPORTED_RATES_SET, pSupportedRatesLst, uBufLength,NULL, 0,&dwRetSize);
        
    }
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetSupportedRates( tiUINT8* pSupportedRatesLst, tiUINT32 uBufLength)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pSupportedRatesLst && CheckObjectMemory(pSupportedRatesLst, uBufLength) )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_SUPPORTED_RATES, NULL, 0, pSupportedRatesLst, uBufLength,&dwRetSize);
    }
    return dwRetValue;
}
/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::SetConfiguration( OS_802_11_CONFIGURATION*   pConfiguration )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pConfiguration && CheckObjectMemory(pConfiguration, sizeof(OS_802_11_CONFIGURATION)) )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_CONFIGURATION_SET, pConfiguration, sizeof(OS_802_11_CONFIGURATION),NULL, 0,&dwRetSize);
        
    }
    return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetConfiguration( OS_802_11_CONFIGURATION*   pConfiguration )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pConfiguration && CheckObjectMemory(pConfiguration, sizeof(OS_802_11_CONFIGURATION)) )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_CONFIGURATION_GET, NULL, 0, pConfiguration, sizeof(OS_802_11_CONFIGURATION),&dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetAuthenticationMode( OS_802_11_AUTHENTICATION_MODE uAuthenticationMode )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_AUTHENTICATION_MODE_SET, &uAuthenticationMode, sizeof(OS_802_11_AUTHENTICATION_MODE),NULL, 0,&dwRetSize);
    return dwRetValue;
}
/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetAuthenticationMode( OS_802_11_AUTHENTICATION_MODE* puAuthenticationMode )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( puAuthenticationMode )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_AUTHENTICATION_MODE_GET, NULL, 0, puAuthenticationMode, sizeof(OS_802_11_AUTHENTICATION_MODE),&dwRetSize);
        
    }
    return dwRetValue;
}
/********************************************************************/
tiINT32 
CTI_WLAN_AdapterAPI::SetPrivacyFilter( tiUINT32 dwPrivacyFilter )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_PRIVACY_FILTER_SET, &dwPrivacyFilter, sizeof(tiUINT32),NULL, 0,&dwRetSize);
    return dwRetValue;
}
/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::GetPrivacyFilter( tiUINT32* pdwPrivacyFilter )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pdwPrivacyFilter )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_PRIVACY_FILTER_GET, NULL, 0, pdwPrivacyFilter, sizeof(tiUINT32),&dwRetSize);
    }
    return dwRetValue;
}
/********************************************************************/
tiINT32 
CTI_WLAN_AdapterAPI::SetKeyType( OS_802_11_KEY_TYPES uKeyType )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_KEY_TYPE_SET, &uKeyType, sizeof(OS_802_11_KEY_TYPES),NULL, 0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetMixedMode( tiBOOL bStatus )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_MIXED_MODE_SET, &bStatus, sizeof(tiBOOL),NULL, 0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetMixedMode( tiBOOL* pbStatus )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pbStatus )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_MIXED_MODE_GET, NULL, 0, pbStatus, sizeof( tiBOOL ),&dwRetSize);
    }
    return dwRetValue;
}


/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::EnableDisable_802_11d( tiUINT8 enableDisable_802_11d)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_REG_DOMAIN_ENABLE_DISABLE_802_11D, &enableDisable_802_11d, sizeof(tiUINT8), NULL, 0, &dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::Get_802_11d( tiUINT8 *enableDisable_802_11d)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( enableDisable_802_11d )
    {
        dwRetValue = tiIoCtrl(TIWLN_REG_DOMAIN_GET_802_11D, NULL, 0, enableDisable_802_11d, sizeof(tiUINT8),&dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::EnableDisable_802_11h( tiUINT8 enableDisable_802_11h)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_REG_DOMAIN_ENABLE_DISABLE_802_11H, &enableDisable_802_11h, sizeof(tiUINT8), NULL, 0, &dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::Get_802_11h( tiUINT8 *enableDisable_802_11h)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( enableDisable_802_11h )
    {
        dwRetValue = tiIoCtrl(TIWLN_REG_DOMAIN_GET_802_11H, NULL, 0, enableDisable_802_11h, sizeof(tiUINT8),&dwRetSize);
    }
    return dwRetValue;
}


/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::Set_countryIeFor2_4_Ghz( country_t countryIe)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_REG_DOMAIN_SET_COUNTRY_2_4, &countryIe, sizeof(country_t), NULL, 0, &dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::Get_countryIeFor2_4_Ghz( tiUINT8 **countryString)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( countryString )
    {
        dwRetValue = tiIoCtrl(TIWLN_REG_DOMAIN_GET_COUNTRY_2_4, NULL, 0, countryString, COUNTRY_STRING_LEN,&dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::Set_countryIeFor5_Ghz( country_t countryIe)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_REG_DOMAIN_SET_COUNTRY_5, &countryIe, sizeof(country_t), NULL, 0, &dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::Get_countryIeFor5_Ghz( tiUINT8 **countryString)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( countryString )
    {
        dwRetValue = tiIoCtrl(TIWLN_REG_DOMAIN_GET_COUNTRY_5, NULL, 0, countryString, COUNTRY_STRING_LEN,&dwRetSize);
    }
    return dwRetValue;
}


/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::Set_minMaxDfsChannels( DFS_ChannelRange_t DFS_ChannelRange)
{
	tiUINT32 dwRetValue = TI_RESULT_FAILED;
	tiUINT32 dwRetSize;

    dwRetValue = tiIoCtrl(TIWLN_REG_DOMAIN_SET_DFS_RANGE, &DFS_ChannelRange, sizeof(DFS_ChannelRange_t), NULL, 0, &dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::Get_minMaxDfsChannels( DFS_ChannelRange_t *DFS_ChannelRange)
{
	tiUINT32 dwRetValue = TI_RESULT_FAILED;
	tiUINT32 dwRetSize;

    if ( DFS_ChannelRange )
    {
        dwRetValue = tiIoCtrl(TIWLN_REG_DOMAIN_GET_DFS_RANGE, NULL, 0, DFS_ChannelRange, sizeof(DFS_ChannelRange_t), &dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetIBSSProtection( tiUINT32 uProtection )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_IBSS_PROTECTION_SET, &uProtection, sizeof(tiUINT32),NULL, 0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetIBSSProtection ( tiUINT32* puProtection )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( puProtection )
    {
        dwRetValue = tiIoCtrl(TIWLN_IBSS_PROTECTION_GET, NULL, 0, puProtection, sizeof(tiUINT32),&dwRetSize);
    }
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::SetShortSlot( tiUINT32 dwShortSlot )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_SHORT_SLOT_SET, &dwShortSlot, sizeof(tiUINT32),NULL, 0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetShortSlot( tiUINT32* pdwShortSlot)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pdwShortSlot )
    {
        dwRetValue = tiIoCtrl(TIWLN_SHORT_SLOT_GET, NULL, 0, pdwShortSlot, sizeof(tiUINT32),&dwRetSize);
    }
    return dwRetValue;
}
/********************************************************************/
tiINT32 
CTI_WLAN_AdapterAPI::SetExtRatesIE( tiUINT32 dwExtRatesIE)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_EXT_RATES_IE_SET, &dwExtRatesIE, sizeof(tiUINT32),NULL, 0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetExtRatesIE( tiUINT32* pdwExtRatesIE)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pdwExtRatesIE )
    {
        dwRetValue = tiIoCtrl(TIWLN_EXT_RATES_IE_GET, NULL, 0, pdwExtRatesIE, sizeof(tiUINT32),&dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/
tiINT32 
CTI_WLAN_AdapterAPI::SetEarlyWakeupMode( tiUINT8 dwEarlyWakeup)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_EARLY_WAKEUP_IE_SET, &dwEarlyWakeup, sizeof(tiUINT8),NULL, 0,&dwRetSize);
    
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetEarlyWakeupMode( tiUINT8* pdwEarlyWakeup)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pdwEarlyWakeup )
    {
        dwRetValue = tiIoCtrl(TIWLN_EARLY_WAKEUP_IE_GET, NULL, 0, pdwEarlyWakeup, sizeof(tiUINT8),&dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::Open_EAPOL_Interface( )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_OPEN_EAPOL_INTERFACE, NULL, 0, NULL, 0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::Close_EAPOL_Interface( )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_CLOSE_EAPOL_INTERFACE, NULL, 0, NULL, 0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::Send_EAPOL_Packet( tiVOID* pData, tiUINT32 uSize )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pData && CheckObjectMemory(pData, uSize) )
    {
        dwRetValue = tiIoCtrl(TIWLN_SEND_EAPOL_PACKET, pData, uSize,NULL, 0,&dwRetSize);
    }
    
    return dwRetValue;
}
/********************************************************************/
tiINT32 
CTI_WLAN_AdapterAPI::hwReadRegister( tiUINT32 dwRegisterAddr, tiUINT32* pdwValue )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    TIWLN_REG_RW sRegRead;
    
    if (!pdwValue )
        return TI_RESULT_FAILED;
    
    sRegRead.regSize = 4;
    sRegRead.regAddr = dwRegisterAddr;
    sRegRead.regValue = 0;
    
    dwRetValue = tiIoCtrl(TIWLN_HW_READ_REGISTER, &sRegRead, sizeof(TIWLN_REG_RW), &sRegRead, sizeof(TIWLN_REG_RW),&dwRetSize);
    
    *pdwValue = sRegRead.regValue;
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::hwWriteRegister( tiUINT32 dwRegisterAddr, tiUINT32 dwValue )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    TIWLN_REG_RW sRegRead;
    
    sRegRead.regSize     = 4;
    sRegRead.regAddr     = dwRegisterAddr;
    sRegRead.regValue    = dwValue;
    
    dwRetValue = tiIoCtrl(TIWLN_HW_WRITE_REGISTER, &sRegRead, sizeof(TIWLN_REG_RW),NULL, 0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::Disassociate( )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_DISASSOCIATE, NULL, 0,NULL, 0,&dwRetSize);
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::ReloadDefaults( )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_RELOAD_DEFAULTS, NULL, 0,NULL, 0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::IsDriverLoaded( )
{
    tiUINT32 bRet = TI_RESULT_FAILED;
#ifndef _WINDOWS
    
    TI_HANDLE hDevice = m_pIPCmod->IPC_DeviceOpen(m_pszAdapterName);
    if (hDevice)
        bRet = TI_RESULT_OK;
    
    m_pIPCmod->IPC_DeviceClose();
#endif  
    return bRet;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetRSSI( tiINT32* pRssi )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    
    if (CheckObjectMemory(pRssi, sizeof(tiUINT32)))
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_RSSI,NULL,0,pRssi, sizeof(tiUINT32),&dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetSNR( tiUINT32* pSnr )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    
    if (CheckObjectMemory(pSnr, sizeof(tiUINT32)))
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_SNR,NULL,0,pSnr, sizeof(tiUINT32),&dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetNumberOfAntennas(tiUINT32* puNumberOfAntennas)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( puNumberOfAntennas )
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_NUMBER_OF_ANTENNAS, NULL, 0, puNumberOfAntennas, sizeof(tiUINT32),&dwRetSize);
    }
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetDriverVersion( TIWLN_VERSION_EX* pdrvVersion )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pdrvVersion && CheckObjectMemory(pdrvVersion, sizeof(TIWLN_VERSION_EX)) )
    {
        dwRetValue = tiIoCtrl(TIWLN_GET_SW_VERSION, NULL, 0, pdrvVersion, sizeof(TIWLN_VERSION_EX),&dwRetSize);
    }
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetRegDomainTable( TIWLN_REGDOMAINS* pRegDomainTable )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pRegDomainTable && CheckObjectMemory(pRegDomainTable, sizeof(TIWLN_REGDOMAINS)) )
    {
        dwRetValue = tiIoCtrl(TIWLN_REGDOMAIN_TABLE, NULL, 0, pRegDomainTable, sizeof(TIWLN_REGDOMAINS),&dwRetSize);
    }
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetMediumUsage( TIWLN_MEDIUM_USAGE* pMediumUsage )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pMediumUsage && CheckObjectMemory(pMediumUsage, sizeof(TIWLN_MEDIUM_USAGE)) )
    {
        dwRetValue = tiIoCtrl(TIWLN_MEDIUMUSAGE, NULL, 0, pMediumUsage, sizeof(TIWLN_MEDIUM_USAGE),&dwRetSize);
    }
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetApiVersion( tiUINT32* pdwApiVersion )
{
    *pdwApiVersion = TI_WLAN_API_VER;
    return TI_RESULT_OK;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GetDriverState( driverState_e* puDriverState )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    *puDriverState = (driverState_e)0;
    dwRetValue = tiIoCtrl(TIWLN_802_11_GET_DRIVER_STATE, NULL, 0, puDriverState, sizeof(tiUINT32),&dwRetSize);
    
    switch (((driverState_e)*puDriverState) & 0xff)
    {
    case SM_STATE_IDLE:
        *puDriverState = DRIVER_STATE_IDLE;
        break;
    case SM_STATE_SCANNING:
        *puDriverState = DRIVER_STATE_SCANNING;
        break;
    case SM_STATE_SELECTING:
        *puDriverState = DRIVER_STATE_SELECTING;
        break;
    case SM_STATE_CONNECTING:
        *puDriverState = DRIVER_STATE_CONNECTING;
        break;
    case SM_STATE_CONNECTED:
    case SM_STATE_QUIET_SCAN:
    case SM_STATE_ROAMING_QUIET_SCAN:
    case SM_STATE_MEASUREMENT:
    case SM_STATE_POWER_MNGR_PENDS_QUIET_SCAN:
        *puDriverState = DRIVER_STATE_CONNECTED;
        break;
    case SM_STATE_INTER_SCAN_TIMEOUT:
        *puDriverState = DRIVER_STATE_DISCONNECTED;
        break;
    case SM_STATE_RADIO_STAND_BY:
        *puDriverState = DRIVER_STATE_IDLE;
        break;
    default:
        break;
    }
    
    return dwRetValue;
}
/********************************************************************/

tiINT32
CTI_WLAN_AdapterAPI::Start( )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    tiUINT32 Data = 1;
    
    dwRetValue = tiIoCtrl(TIWLN_DRIVER_STATUS_SET, &Data, sizeof(tiUINT32),NULL, 0,&dwRetSize);
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::Stop( )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    tiUINT32 Data = 0;
    dwRetValue = tiIoCtrl(TIWLN_DRIVER_STATUS_SET, &Data, sizeof(tiUINT32),NULL, 0,&dwRetSize);
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::Suspend( )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    tiUINT32 Data = 0;
    
    dwRetValue = tiIoCtrl(TIWLN_DRIVER_SUSPEND, &Data, sizeof(tiUINT32),NULL, 0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::IsDriverRun( tiUINT32* pbStatus )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if ( pbStatus && CheckObjectMemory(pbStatus, sizeof(tiUINT32)) )
    {
        dwRetValue = tiIoCtrl(TIWLN_DRIVER_STATUS_GET, NULL, 0, pbStatus, sizeof(tiUINT32),&dwRetSize);
    }
    
    return dwRetValue;
}
#define GWSI_DISPACH_OPCODE(_p_gwsi_buffer)				((*((tiUINT8 *)_p_gwsi_buffer + 0) | (*((tiUINT8 *)_p_gwsi_buffer + 1) << 8)))
#define GWSI_DISPACH_CALC_BUFFER_SHORT_LEN(_p_gwsi_buffer)	((*((tiUINT8 *)_p_gwsi_buffer + 2) | (*((tiUINT8 *)_p_gwsi_buffer + 3) << 8)) + 4)
#define GWSI_DISPACH_CALC_BUFFER_LONG_LEN(_p_gwsi_buffer)	(*((tiUINT8 *)_p_gwsi_buffer + 2) | (*((tiUINT8 *)_p_gwsi_buffer + 3) << 8) | (*((tiUINT8 *)_p_gwsi_buffer + 4) << 16) | (*((tiUINT8 *)_p_gwsi_buffer + 5) << 24) + 4)
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GWSICommand( tiUINT32* pGWSICommand )
{
    tiUINT32 bRet = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetSize = tiIoCtrl(GWSI_DISPATCH_COMMAND, pGWSICommand, GWSI_DISPACH_CALC_BUFFER_SHORT_LEN(pGWSICommand));
    
    if ( dwRetSize  )
        bRet = TI_RESULT_OK;
    
    
    return bRet;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GWSIInitialize( tiUINT32* pGWSICommand )
{
    tiUINT32 bRet = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    /* This command requires size of 4 bytes */
    dwRetSize = tiIoCtrl(GWSI_INITIALIZE_COMMAND, pGWSICommand, GWSI_DISPACH_CALC_BUFFER_LONG_LEN(pGWSICommand));
    
    if ( dwRetSize  )
        bRet = TI_RESULT_OK;
    
    
    return bRet;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GWSIConfig( tiUINT32* pGWSICommand )
{
    tiUINT32 bRet = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetSize = tiIoCtrl(GWSI_CONFIGURE_TABLE_COMMAND, pGWSICommand, GWSI_DISPACH_CALC_BUFFER_SHORT_LEN(pGWSICommand));
    
    if ( dwRetSize  )
        bRet = TI_RESULT_OK;
    
    
    return bRet;
}

/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::GWSIRelease( tiUINT32* pGWSICommand )
{
    tiUINT32 bRet = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetSize = tiIoCtrl(GWSI_RELEASE_COMMAND, pGWSICommand, GWSI_DISPACH_CALC_BUFFER_SHORT_LEN(pGWSICommand));
    
    if ( dwRetSize  )
        bRet = TI_RESULT_OK;
    
    
    return bRet;
}

/********************************************************************/
tiINT32     
CTI_WLAN_AdapterAPI::GWSIGetInitTable (tiUINT32* pGWSICommand )
{
    tiUINT32 bRet = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetSize = tiIoCtrl(GWSI_GET_INIT_TABLE_COMMAND, NULL, 0, pGWSICommand, (1024 * 5));
    
    if ( dwRetSize  )
        bRet = TI_RESULT_OK;
    
    
    return bRet;
}

#ifndef _WINDOWS
	/********************************************************************/
	tiINT32
	TI_IPC::IPC_DeviceIoControl(tiUINT32 dwIoControlCode, tiVOID* lpInBuffer,
								tiUINT32 nInBufferSize, tiVOID* lpOutBuffer,
								tiUINT32 nOutBufferSize, tiUINT32* lpBytesReturned)
	{
		if (!m_hDevice)
			return TI_RESULT_INVALIDE_HANDLE;
		
		return ::IPC_DeviceIoControl(m_hDevice, dwIoControlCode, lpInBuffer,
			nInBufferSize, lpOutBuffer, nOutBufferSize, lpBytesReturned);
	}
	
	TI_IPC::TI_IPC()    
	{ 
		m_hDevice = NULL;
		IPC_Init();
	}
	
	TI_IPC::~TI_IPC()    
	{ 
		IPC_DeInit();
	}
	
	TI_HANDLE
	TI_IPC::IPC_DeviceOpen (tiCHAR* pAdapterName)
	{
		m_hDevice = ::IPC_DeviceOpen(pAdapterName);
		return m_hDevice;
	}
	
	tiVOID
	TI_IPC::IPC_DeviceClose()
	{
		::IPC_DeviceClose(m_hDevice);
		m_hDevice = NULL;
	}
	
	tiINT32 
	TI_IPC::IPC_RegisterEvent( IPC_EVENT_PARAMS* pEventParams )
	{
		return ::IPC_RegisterEvent(m_hDevice, pEventParams);
	}
	
	
	tiINT32
	TI_IPC::IPC_UnRegisterEvent( IPC_EVENT_PARAMS* pEventParams )
	{
		return ::IPC_UnRegisterEvent(m_hDevice, pEventParams);
	}
#endif
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::Set_RoamingConfParams( UINT8* buffer, UINT16 bufferLength)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_802_11_ROAMING_CONFIG_PARAMS_SET, buffer, bufferLength,NULL,0,&dwRetSize);
    
    return dwRetValue;
}
/********************************************************************/
tiINT32
CTI_WLAN_AdapterAPI::Get_RoamingConfParams( UINT8* buffer, UINT16 bufferLength)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if (CheckObjectMemory(buffer, bufferLength))
    {
        dwRetValue = tiIoCtrl(TIWLN_802_11_ROAMING_CONFIG_PARAMS_GET, NULL, 0, buffer, bufferLength, &dwRetSize);
    }
    
    return dwRetValue;
}
/********************************************************************/

/****************************   PLT  ********************************/

/********************************************************************/
tiINT32		
CTI_WLAN_AdapterAPI::PLT_ReadRegister( UINT32 uRegisterAddr, PUINT32 puRegisterData )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if (CheckObjectMemory(puRegisterData, sizeof(UINT32)))
    {
        dwRetValue = tiIoCtrl(TIWLN_PLT_READ_REGISTER, &uRegisterAddr, sizeof(UINT32), puRegisterData, sizeof(UINT32), &dwRetSize);
    }
    
    return dwRetValue;
}

/********************************************************************/

tiINT32		
CTI_WLAN_AdapterAPI::PLT_WriteRegister( UINT32 uRegisterAddr, UINT32 uRegisterData )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    TIWLN_REG_RW sRegWrite;
    
    sRegWrite.regSize     = 4;
    sRegWrite.regAddr     = uRegisterAddr;
    sRegWrite.regValue    = uRegisterData;
    
    dwRetValue = tiIoCtrl(TIWLN_PLT_WRITE_REGISTER, &sRegWrite, sizeof(TIWLN_REG_RW),NULL, 0,&dwRetSize);
    
    
    return dwRetValue;
}


/********************************************************************/

tiINT32		
CTI_WLAN_AdapterAPI::PLT_RxPerStart()
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_PLT_RX_PER_START, NULL, 0, NULL, 0, &dwRetSize);
    
    return dwRetValue;
}

/********************************************************************/

tiINT32		
CTI_WLAN_AdapterAPI::PLT_RxPerStop()
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_PLT_RX_PER_STOP, NULL, 0, NULL, 0, &dwRetSize);
    
    return dwRetValue;
}

/********************************************************************/

tiINT32		
CTI_WLAN_AdapterAPI::PLT_RxPerClear()
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_PLT_RX_PER_CLEAR, NULL, 0, NULL, 0, &dwRetSize);
    
    return dwRetValue;
}

/********************************************************************/

tiINT32		
CTI_WLAN_AdapterAPI::PLT_RxPerGetResults( PltRxPer_t* pPltRxPer )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    if (CheckObjectMemory(pPltRxPer, sizeof(PltRxPer_t)))
    {
        dwRetValue = tiIoCtrl(TIWLN_PLT_RX_PER_GET_RESULTS, NULL, 0, pPltRxPer, sizeof(PltRxPer_t), &dwRetSize);
    }
    
    return dwRetValue;
}


/********************************************************************/

tiINT32		
CTI_WLAN_AdapterAPI::PLT_TxCW(TestCmdChannelBand_t* pPltTxCW)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_PLT_TX_CW, pPltTxCW, sizeof(*pPltTxCW), NULL, 0, &dwRetSize);
    
    return dwRetValue;
}

/********************************************************************/

tiINT32		
CTI_WLAN_AdapterAPI::PLT_TxContiues(PltTxContinues_t* pPltTxContinues)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_PLT_TX_CONTINUES, pPltTxContinues, sizeof(PltTxContinues_t), NULL, 0, &dwRetSize);
    
    return dwRetValue;
}

/********************************************************************/

tiINT32		
CTI_WLAN_AdapterAPI::PLT_TxStop()
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    
    dwRetValue = tiIoCtrl(TIWLN_PLT_TX_STOP, NULL, 0, NULL, 0, &dwRetSize);
    
    return dwRetValue;
}

/********************************************************************/
tiINT32		
CTI_WLAN_AdapterAPI::PLT_ReadMIB ( PLT_MIB_t* pMib )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;      
    tiUINT32 dwRetSize; 
    if (CheckObjectMemory(pMib, sizeof(PLT_MIB_t)))
    {
      dwRetValue = tiIoCtrl(TIWLN_PLT_MIB_READ,
							  (tiVOID*)pMib,(tiUINT32)sizeof(PLT_MIB_t),
							  (tiVOID*)pMib,(tiUINT32)sizeof(PLT_MIB_t),
							  &dwRetSize);
    }
    return dwRetValue;
}

/********************************************************************/
tiINT32		
CTI_WLAN_AdapterAPI::PLT_WriteMIB( PLT_MIB_t* pMib )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    dwRetValue = tiIoCtrl(TIWLN_PLT_MIB_WRITE,
						  pMib, sizeof(PLT_MIB_t),
						  NULL, 0,
						  &dwRetSize);
    return dwRetValue;
}


/********************************************************************/
tiINT32		
CTI_WLAN_AdapterAPI::GetDefaultWepKey(  tiUINT32* puKeyIndex )
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;
    tiUINT32 dwRetSize;
    UINT32 InfoCode = VAL_DEFAULT_KEY_ID;
    
    dwRetValue = tiIoCtrl(TIWLN_IOCTL_OID_QUERY_INFORMATION, &InfoCode, (tiUINT32)sizeof(InfoCode), puKeyIndex, sizeof(tiUINT32), &dwRetSize);
    return dwRetValue;
}

/********************************************************************/
tiINT32		
CTI_WLAN_AdapterAPI::PLT_TxCalGainGet(PltGainGet_t* pPLTGainGet)
{
    return PLT_RxTXCal((void*)pPLTGainGet, sizeof(PltGainGet_t), TEST_CMD_PLT_GAIN_GET);
}

/********************************************************************/
tiINT32		
CTI_WLAN_AdapterAPI::PLT_TxCalGainAdjust(tiINT32   uTxGainChange)
{
    return PLT_RxTXCal(&uTxGainChange, sizeof(tiINT32), TEST_CMD_PLT_GAIN_ADJUST);
}

/********************************************************************/
tiINT32		
CTI_WLAN_AdapterAPI::PLT_TxCalStart(PltTxCalibrationRequest_t* pPLTTxCal)
{
    return PLT_RxTXCal((void*)pPLTTxCal, sizeof(PltTxCalibrationRequest_t), TEST_CMD_PLT_TXPOWER_CAL_START);
}

/********************************************************************/
tiINT32		
CTI_WLAN_AdapterAPI::PLT_TxCalStop()
{
    return PLT_RxTXCal(NULL, 0, TEST_CMD_PLT_TXPOWER_CAL_STOP);
}

/********************************************************************/
tiINT32		
CTI_WLAN_AdapterAPI::PLT_RxTxCalNVSUpdateBuffer(PltNvsResultsBuffer_t* pPLT_NVSUpdateBuffer)
{
    return PLT_RxTXCal(pPLT_NVSUpdateBuffer, sizeof(PltNvsResultsBuffer_t), TEST_CMD_PLT_GET_NVS_UPDATE_BUFFER);
}

/********************************************************************/
tiINT32		
CTI_WLAN_AdapterAPI::PLT_RxCal(PltRxCalibrationRequest_t* pPltRxCalibration)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;      
    tiUINT32 dwRetSize;
    TestCmd_t TestCmd;

    if ( CheckObjectMemory(pPltRxCalibration, sizeof(PltRxCalibrationRequest_t)) )
    {   
        memset(&TestCmd, 0, sizeof(TestCmd));
        memcpy(&TestCmd.testCmd_u, pPltRxCalibration, sizeof(PltRxCalibrationRequest_t));
        TestCmd.testCmdId = TEST_CMD_PLT_RX_CALIBRATION;

        dwRetValue = tiIoCtrl(TIWLN_PLT_RX_CAL,
                              (tiVOID*) &TestCmd, sizeof(TestCmd),
                              (tiVOID*) &TestCmd, sizeof(TestCmd),
                              &dwRetSize);
    }
    /* 
     * for RX calibration, we query the status on a polling loop until it is complete.
     * This is done to avoid a WinMobile bug, where returning pending will make Win re-send
     * the OID after a while
     * This is a patch, and a unified a-synchronous method must be implemented
     * for all IOCTLs
     */
    if (TI_RESULT_OK == dwRetValue)
    {
        TI_STATUS   TIStatus = PENDING;
        while ( PENDING == TIStatus )
        {
            dwRetValue = PLT_RxCalStatus( &TIStatus );
            m_pOSLib->TISleep(100);
        }
    }
    return dwRetValue;
}

/********************************************************************/
tiINT32		
CTI_WLAN_AdapterAPI::PLT_RadioTune(TestCmdChannelBand_t* pChannelBand)
{
    return PLT_RxTXCal(pChannelBand, sizeof(TestCmdChannelBand_t), TEST_CMD_RADIO_TUNE);
}

/********************************************************************/
tiINT32 
CTI_WLAN_AdapterAPI::PLT_RxTXCal(void* pTestCmdData, tiUINT32 Length, TestCmdID_e TestCmdID)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;      
    tiUINT32 dwRetSize;
    TestCmd_t TestCmd;

    
    if ((Length==0) ||
        (CheckObjectMemory(pTestCmdData, Length)))
    {   
        memset(&TestCmd, 0, sizeof(TestCmd));
        memcpy(&TestCmd.testCmd_u, pTestCmdData, Length);
        TestCmd.testCmdId = TestCmdID;

        dwRetValue = tiIoCtrl(TIWLN_PLT_RX_TX_CAL,
                              (tiVOID*) &TestCmd, sizeof(TestCmd),
                              (tiVOID*) &TestCmd, sizeof(TestCmd),
                              &dwRetSize);
    }
    if (TI_RESULT_OK == dwRetValue)
    {
        memcpy(pTestCmdData, &TestCmd.testCmd_u, Length);
    }
    return dwRetValue;    
}

/********************************************************************/
tiINT32 
CTI_WLAN_AdapterAPI::PLT_RxCalStatus(TI_STATUS* pStatus)
{
    tiUINT32 dwRetValue = TI_RESULT_FAILED;      
    tiUINT32 dwRetSize;

    dwRetValue = tiIoCtrl(TIWLN_PLT_RX_CAL_RESULT,
                          NULL, 0, (tiVOID*)pStatus, sizeof(*pStatus), &dwRetSize);

    return dwRetValue;
}

/********************************************************************/
#ifdef _WINDOWS
#endif

