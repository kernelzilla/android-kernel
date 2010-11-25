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
/* Module:		TI_AdapterSEC.h*/
/**/
/* Purpose:		*/
/**/
/*--------------------------------------------------------------------------*/

#ifndef TI_ADAPTER_SEC_H
#define TI_ADAPTER_SEC_H

#ifdef __cplusplus
extern "C" {
#endif
    
/******************************************************************************

    Name:	TI_SetAuthenticationMode
	Desc:	This function sets the system's authentication mode.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
    uAuthMode - one of: os802_11AuthModeOpen,
                        os802_11AuthModeShared,
                        os802_11AuthModeAutoSwitch,
                        os802_11AuthModeWPA,
                        os802_11AuthModeWPAPSK,
                        os802_11AuthModeWPANone,
                        os802_11AuthModeWPA2,
                        os802_11AuthModeWPA2PSK,
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetAuthenticationMode (TI_HANDLE  hAdapter, 
                                      OS_802_11_AUTHENTICATION_MODE  uAuthMode);

/******************************************************************************

    Name:	TI_GetAuthenticationMode
	Desc:	This function retrieves the 802.11 authentication mode.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
    puAuthMode - one of: os802_11AuthModeOpen,
                         os802_11AuthModeShared,
                         os802_11AuthModeAutoSwitch,
                         os802_11AuthModeWPA,
                         os802_11AuthModeWPAPSK,
                         os802_11AuthModeWPANone,
                         os802_11AuthModeWPA2,
                         os802_11AuthModeWPA2PSK,
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_GetAuthenticationMode (TI_HANDLE  hAdapter, 
                                      OS_802_11_AUTHENTICATION_MODE* puAutMode);


/******************************************************************************

    Name:	TI_SetCertificateParameters
	Desc:	This function is used for WPA or EXC Certificate configuration.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pData - 
            bValidateServerCert - 
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetCertificateParameters (TI_HANDLE  hAdapter, 
                                         tiVOID* pData, 
                                         tiBOOL bValidateServerCert );

/******************************************************************************

    Name:	TI_SetEAPType
    Desc:	This function sets the current EAP type in the Supplicant.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            uEAPType - Contains value of current EAP type from enumeration 
                       OS_802_11_EAP_TYPES:
                        OS_EAP_TYPE_NONE = -1,
                        OS_EAP_TYPE_MD5_CHALLENGE = 4,
                        OS_EAP_TYPE_GENERIC_TOKEN_CARD = 6,
                        OS_EAP_TYPE_TLS = 13,
                        OS_EAP_TYPE_LEAP = 17,
                        OS_EAP_TYPE_TTLS = 21,
                        OS_EAP_TYPE_PEAP = 25,
                        OS_EAP_TYPE_MS_CHAP_V2 = 26,
                        OS_EAP_TYPE_FAST = 43
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
    Note:   Under Linux (ASD device): MD5_CHALLENGE, GENERIC_TOKEN_CARD, TTLS,
                                      PEAP and MS_CHAP_V2 are not supported.
    
******************************************************************************/
tiINT32     TI_SetEAPType               (TI_HANDLE  hAdapter,
                                         OS_802_11_EAP_TYPES  uEAPType );

/******************************************************************************

    Name:	TI_GetEAPType
    Desc:	This function retrieves the current EAP type.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            puEAPType - Pointer to a OS_802_11_EAP_TYPES value that contains
                        current EAP type
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_GetEAPType               (TI_HANDLE  hAdapter, 
                                         OS_802_11_EAP_TYPES* puEAPType);

/******************************************************************************

    Name:	TI_SetEAPTypeDriver
    Desc:	This function sets the current EAP type in the driver.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            uEAPType - Contains value of current EAP type from enumeration 
                       OS_802_11_EAP_TYPES.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
    Note:   Under Linux (ASD device): MD5_CHALLENGE, GENERIC_TOKEN_CARD, TTLS,
                                      PEAP and MS_CHAP_V2 are not supported.
    
******************************************************************************/
tiINT32     TI_SetEAPTypeDriver         (TI_HANDLE  hAdapter, 
                                         OS_802_11_EAP_TYPES  uEAPType );


/******************************************************************************

    Name:	TI_SetEncryptionType
    Desc:	This function sets the Encryption type. This function sets both 
            the driver and the supplicant via an IOCTL.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            uEncryptType - Contains value of current encrypt type from enumeration 
                           OS_802_11_ENCRYPTION_TYPES:
                               OS_ENCRYPTION_TYPE_NONE = 0,
                               OS_ENCRYPTION_TYPE_WEP,
                               OS_ENCRYPTION_TYPE_TKIP,
                               OS_ENCRYPTION_TYPE_AES,
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetEncryptionType        (TI_HANDLE  hAdapter, 
                                         OS_802_11_ENCRYPTION_TYPES  uEncryptType );

/******************************************************************************

    Name:	TI_GetEncryptionType
    Desc:	This function retrieves the current encryption type.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            puEncryptType - Pointer to a OS_802_11_EAP_TYPES value that contains 
                            current encrypt type
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_GetEncryptionType        (TI_HANDLE  hAdapter, 
                                         OS_802_11_ENCRYPTION_TYPES* puEncryptType);

/******************************************************************************

    Name:	TI_SetCredentials
    Desc:	This function sets the User Name and Password in the supplicant.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pszUserName - Pointer to a null-terminated string that contains the 
                          user name.
            pszPassword - Pointer to a null-terminated string that contains the 
                          password.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetCredentials           (TI_HANDLE  hAdapter, 
                                         tiCHAR* pszUserName, 
                                         tiCHAR* pszPassword );

/******************************************************************************

    Name:	TI_SetPSK
    Desc:	This function sets the PSK Password Phrase for WPA type encryption
            in the supplicant.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pszPSK - Pointer to a null-terminated string that contains the PSK
                     password phrase
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetPSK                   (TI_HANDLE  hAdapter, 
                                         tiCHAR* pszPSK );

/******************************************************************************

    Name:	TI_SetKeyType
    Desc:	This function sets the encryption key type, OS_KEY_TYPE_STATIC uses
            the regular 802.11 WEP, and OS_KEY_TYPE_DYNAMIC uses 802.1x (WPA)
            encryption.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            uKeyType - Define the security key type:
                        OS_KEY_TYPE_STATIC = 0,
                        OS_KEY_TYPE_DYNAMIC
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetKeyType               (TI_HANDLE  hAdapter, 
                                         OS_802_11_KEY_TYPES uKeyType );
                                            

/******************************************************************************

    Name:	TI_AddWEPKey
    Desc:	This function enables you to add a new static WEP key and to 
            indicate whether the key is the default key. The function should
            be used only in static key mode.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pWEP - A pointer to an OS_802_11_WEP structure:
                    Length      - specifies the length of the OS_802_11_WEP 
                                  structure in bytes.
                    KeyIndex    - specifies which key to add. KeyIndex can be 
                                  0 to 3. When the most significant bit is set
                                  to 1, the key is set as the default key.
                    KeyLength   - specifies the length of the KeyMaterial 
                                  character array in bytes.
                    KeyMaterial - specifies an array that identifies the WEP key.
                                  The length of this array is variable.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_AddWEPKey                (TI_HANDLE  hAdapter, 
                                         OS_802_11_WEP* pWEP    );

/******************************************************************************

    Name:	TI_RemoveWEPKey
    Desc:	This function removes a static WEP key.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            uKeyIndex - Contains the index of the key to remove.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_RemoveWEPKey             (TI_HANDLE  hAdapter, 
                                         tiUINT32   uKeyIndex   );
/******************************************************************************

    Name:	TI_GetDefaultWepKey
    Desc:	This function returns the default WEP key as it was previously set by TI_AddWEPKey function.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            puKeyIndex - Pointer to the index of the default WEP key.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32		TI_GetDefaultWepKey        (TI_HANDLE  hAdapter,
                                        tiUINT32* puKeyIndex );
 
/******************************************************************************

    Name:	TI_SetMixedMode
    Desc:	This function enables the station to connect to an AP with or without
            WEP. When disabled, only the configured security mode is possible.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            bStatus - FALSE for disable, TRUE for enable.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_SetMixedMode             (TI_HANDLE  hAdapter, 
                                         tiBOOL bStatus);

/******************************************************************************

    Name:	TI_GetMixedMode
    Desc:	This function checks if the station can connect to an AP with or 
            without WEP.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            pbStatus - FALSE indicates Mixed mode is disbaled, TRUE for enable.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_GetMixedMode             (TI_HANDLE  hAdapter, 
                                         tiBOOL* pbStatus);

/******************************************************************************

    Name:	TI_SetWpaOptions
    Desc:	This function sets WPA promotion options. The STA supports promotion
            of WPA2 authentication and AES encryption. Promotion means that if 
            the STA is configured to WPA , the STA connects to either WPA or 
            WPA2 according to the AP’s max support. This function sets both the 
            Driver and the Supplicant via an IOCTL.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            fWPAOptions - The required WPA options.
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
    Notes:  Supports only the ENABLE_ALL option.
	
******************************************************************************/
tiINT32     TI_SetWpaOptions            (TI_HANDLE  hAdapter, 
                                         tiUINT32   fWPAOptions );

/******************************************************************************

    Name:	TI_GetWpaOptions
    Desc:	This function retrieves the WPA promotion options.
    Params:	hAdapter - The Adapter handle returned by TI_AdapterInit().
            fWPAOptions - Current WPA options
    Return:	TI_RESULT_OK on success. Any other value indicates an error.
	
******************************************************************************/
tiINT32     TI_GetWpaOptions            (TI_HANDLE  hAdapter, 
                                         tiUINT32 *fWPAOptions );
#ifdef __cplusplus
}
#endif


#endif /* TI_ADAPTER_SEC_H*/
