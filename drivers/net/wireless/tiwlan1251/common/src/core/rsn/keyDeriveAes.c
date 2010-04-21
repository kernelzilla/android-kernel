/** \file keyDeriveAes.c
 * \brief AES encryption key derivation implementation.
 *
 * \see aesBroadcastKeyDerivation.h
*/
/****************************************************************************
**+-----------------------------------------------------------------------+**
**|                                                                       |**
**| Copyright(c) 1998 - 2008 Texas Instruments. All rights reserved.      |**
**| All rights reserved.                                                  |**
**|                                                                       |**
**| Redistribution and use in source and binary forms, with or without    |**
**| modification, are permitted provided that the following conditions    |**
**| are met:                                                              |**
**|                                                                       |**
**|  * Redistributions of source code must retain the above copyright     |**
**|    notice, this list of conditions and the following disclaimer.      |**
**|  * Redistributions in binary form must reproduce the above copyright  |**
**|    notice, this list of conditions and the following disclaimer in    |**
**|    the documentation and/or other materials provided with the         |**
**|    distribution.                                                      |**
**|  * Neither the name Texas Instruments nor the names of its            |**
**|    contributors may be used to endorse or promote products derived    |**
**|    from this software without specific prior written permission.      |**
**|                                                                       |**
**| THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   |**
**| "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     |**
**| LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR |**
**| A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  |**
**| OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, |**
**| SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT      |**
**| LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, |**
**| DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY |**
**| THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT   |**
**| (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE |**
**| OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  |**
**|                                                                       |**
**+-----------------------------------------------------------------------+**
****************************************************************************/

/****************************************************************************
 *                                                                          *
 *   MODULE:	AES broadcast key derivation                                *
 *   PURPOSE:   AES broadcast key derivation                                *
 *                                                                          *
 ****************************************************************************/

#include "osApi.h"
#include "utils.h"
#include "report.h"
#include "rsnApi.h"

#include "keyDerive.h"
#include "keyDeriveAes.h"

#include "mainKeysSm.h"

/**
*
* keyDeriveAes_config
*
* \b Description: 
*
* AES broadcast key derivation configuration function: 
*			- Initializes the derive & remove callback functions
* \b ARGS: 
*
*  None
*
* \b RETURNS:
*
*  OK on success, NOK otherwise.
*/

TI_STATUS keyDeriveAes_config(struct _keyDerive_t *pKeyDerive)
{
	pKeyDerive->derive = keyDeriveAes_derive;
	pKeyDerive->remove = keyDeriveAes_remove;

	return OK;
}


/**
*
* keyDeriveAes_derive
*
* \b Description: 
*
* AES key derivation function: 
*					- Decodes the key material.
*					- Distribute the decoded key material to the driver.
*
* \b ARGS: 
*
*  I - p - Pointer to the encoded key material.
*
* \b RETURNS:
*
*  OK on success, NOK otherwise.
*/

TI_STATUS keyDeriveAes_derive(struct _keyDerive_t *pKeyDerive, encodedKeyMaterial_t *pEncodedKey)
{
	TI_STATUS status;
	securityKeys_t	key;
	keyMaterialAes_t   *keyMaterialAes = NULL;
	
	/* Small verification */
	if ((pEncodedKey==NULL) || (pKeyDerive == NULL))
	{
		return NOK;
	}
	
	if (pEncodedKey->keyLen < sizeof(keyMaterialAes_t))
	{
		WLAN_REPORT_ERROR(pKeyDerive->hReport, RSN_MODULE_LOG,
						("KEY_DERIVE_AES: ERROR: wrong key length %d !!!\n",
						pEncodedKey->keyLen));
		return NOK;
	}

	keyMaterialAes = (keyMaterialAes_t*)pEncodedKey->pData;

	
	/* Fill security key structure */
	os_memoryZero(pKeyDerive->hOs, &key, sizeof(securityKeys_t));

	key.keyType   = AES_KEY;
	key.keyIndex  = (UINT8)pEncodedKey->keyId;
	key.encLen    = DERIVE_AES_KEY_LEN;
	os_memoryCopy(pKeyDerive->hOs, (void *)key.encKey, pEncodedKey->pData + MAC_ADDR_LEN+KEY_RSC_LEN, 
		          DERIVE_AES_KEY_LEN);

	/* Copy MAC address key */
	os_memoryCopy(pKeyDerive->hOs, (void *)key.macAddress.addr, (void *)keyMaterialAes->macAddress, MAC_ADDR_LEN);

	/* Copy RSC */
	os_memoryCopy(pKeyDerive->hOs, (void *)key.keyRsc, (void *)keyMaterialAes->keyRSC, KEY_RSC_LEN);

	status = pKeyDerive->pMainKeys->setKey(pKeyDerive->pMainKeys, &key);
	if (status == OK)
	{
		os_memoryCopy(pKeyDerive->hOs, &pKeyDerive->key, pEncodedKey, sizeof(encodedKeyMaterial_t));
	}
	
	return status;
}

/**
*
* keyDeriveAes_remove
*
* \b Description: 
*
* AES key remove function: 
*			- Remove the key material from the driver.
*
* \b ARGS: 
*
*  None.
*
* \b RETURNS:
*
*  OK on success, NOK otherwise.
*/

TI_STATUS keyDeriveAes_remove(struct _keyDerive_t *pKeyDerive, encodedKeyMaterial_t *pEncodedKey)
{
	TI_STATUS status;
	securityKeys_t	key;
	
	if ((pEncodedKey==NULL) || (pKeyDerive == NULL))
	{
		return NOK;
	}
	
	if (pEncodedKey->keyLen != DERIVE_AES_KEY_LEN) 
	{
		return NOK;
	}

	os_memoryZero(pKeyDerive->hOs, &key, sizeof(securityKeys_t));
	key.keyType  = AES_KEY;
	key.keyIndex = (UINT8)pEncodedKey->keyId;
	key.encLen   = (UINT16)pEncodedKey->keyLen;
	os_memoryCopy(pKeyDerive->hOs, (void *)key.macAddress.addr, pEncodedKey->pData, MAC_ADDR_LEN);

	status = pKeyDerive->pMainKeys->removeKey(pKeyDerive->pMainKeys, &key);
	if (status == OK)
	{
		os_memoryZero(pKeyDerive->hOs, &pKeyDerive->key, sizeof(encodedKeyMaterial_t));
	}
	
	return status;
}

