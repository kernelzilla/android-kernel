/** \file keyParserExternal.c
 * \brief External key parser implementation.
 *
 * \see keyParser.h
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
 *   MODULE:	External Key Parser                                             *
 *   PURPOSE:   EAP parser implementation                                   *
 *                                                                          *
 ****************************************************************************/

#include "osTIType.h"
#include "osApi.h"
#include "report.h"
#include "utils.h"

#include "keyTypes.h"

#include "keyParser.h"
#include "keyParserExternal.h"
#include "mainKeysSm.h"
#include "mainSecSm.h"
#include "admCtrl.h"

#include "unicastKeySM.h"
#include "broadcastKeySM.h"
#include "DataCtrl_Api.h"

#define  CKIP_KEY_LEN 16
#define  TKIP_KEY_LEN 32
#define  AES_KEY_LEN  16


/**
*
* Function  - Init KEY Parser module.
*
* \b Description: 
*
* Called by RSN Manager. 
* Registers the function 'rsn_keyParserRecv()' at the distributor to receive KEY frames upon receiving a KEY_RECV event.
*
* \b ARGS:
*
*  
* \b RETURNS:
*
*  TI_STATUS - 0 on success, any other value on failure. 
*
*/

TI_STATUS keyParserExternal_config(struct _keyParser_t *pKeyParser)
{
	pKeyParser->recv = keyParserExternal_recv;
	pKeyParser->replayReset = keyParser_nop;
	pKeyParser->remove = keyParserExternal_remove;
	return OK;
}


/**
*
* keyParserExternal_recv
*
* \b Description: 
*
* External key Parser receive function:
*							- Called by NDIS (Windows)  upon receiving an External Key.
*							- Filters the following keys:								
*								- Keys with invalid key index
*								- Keys with invalid MAC address
*
* \b ARGS:
*
*  I   - pKeyParser - Pointer to the keyParser context  \n
*  I   - pKeyData - A pointer to the Key Data. \n
*  I   - keyDataLen - The Key Data length. \n
*
* \b RETURNS:
*
*  OK on success, NOK otherwise.
*
*/

TI_STATUS keyParserExternal_recv(struct _keyParser_t *pKeyParser,
						  UINT8 *pKeyData, UINT32 keyDataLen)
{
	TI_STATUS						status;
	OS_802_11_KEY 	                *pKeyDesc;
	encodedKeyMaterial_t    		encodedKeyMaterial;
    paramInfo_t  					macParam;
	BOOL                            macEqual2Associated=FALSE;
	BOOL							macIsBroadcast=FALSE;
    BOOL                            wepKey = FALSE;
	UINT8							broadcastMacAddr[MAC_ADDR_LEN] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
	UINT8							nullMacAddr[MAC_ADDR_LEN] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    UINT8                           keyBuffer[MAC_ADDR_LEN+KEY_RSC_LEN+MAX_EXT_KEY_DATA_LENGTH];
    

    if (pKeyData == NULL)                             
	{                                                 
		WLAN_REPORT_ERROR(pKeyParser->hReport, RSN_MODULE_LOG,
								  ("EXT_KEY_PARSER: ERROR: NULL KEY Data\n"));
		return NOK;
	}
	
	pKeyDesc = (OS_802_11_KEY*)pKeyData;

    /* copy the key data, mac address and RSC */
	os_memoryCopy(pKeyParser->hOs, &keyBuffer[0], pKeyDesc->BSSID, MAC_ADDR_LEN);
	/* configure keyRSC value (if needed) */
    if (pKeyDesc->KeyIndex & EXT_KEY_RSC_KEY_MASK)
	{	/* set key recieve sequence counter */
        os_memoryCopy(pKeyParser->hOs, &keyBuffer[MAC_ADDR_LEN], (UINT8*)&(pKeyDesc->KeyRSC), KEY_RSC_LEN);
	}
    else
    {
        os_memoryZero(pKeyParser->hOs, &keyBuffer[MAC_ADDR_LEN], KEY_RSC_LEN);
    }

    /* check type and validity of keys */
    /* check MAC Address validity */
	macParam.paramType = CTRL_DATA_CURRENT_BSSID_PARAM;
	status = ctrlData_getParam(pKeyParser->hCtrlData, &macParam);

	if (status != OK)
	{
		WLAN_REPORT_ERROR(pKeyParser->hReport, RSN_MODULE_LOG,
                          ("EXT_KEY_PARSER: ERROR: Cannot get MAC address !!!\n"));
        return NOK;
	}

	/* check key length */
	if((pKeyDesc->KeyLength != WEP_KEY_LEN_40) && 
		(pKeyDesc->KeyLength != WEP_KEY_LEN_104) && 
		(pKeyDesc->KeyLength != WEP_KEY_LEN_232) &&
		(pKeyDesc->KeyLength != CKIP_KEY_LEN) && 
		(pKeyDesc->KeyLength != TKIP_KEY_LEN) && 
		(pKeyDesc->KeyLength != AES_KEY_LEN) )
		
	{
		WLAN_REPORT_ERROR(pKeyParser->hReport, RSN_MODULE_LOG,
		                 ("EXT_KEY_PARSER: ERROR: Incorrect key length - %d \n", pKeyDesc->KeyLength));
		return STATUS_BAD_KEY_PARAM;
	}
	if (!os_memoryCompare(pKeyParser->hOs, (void *)macParam.content.ctrlDataCurrentBSSID.addr, pKeyDesc->BSSID, MAC_ADDR_LEN))
	{	
        macEqual2Associated = TRUE;
   	}
	if (!os_memoryCompare(pKeyParser->hOs, pKeyDesc->BSSID, broadcastMacAddr, MAC_ADDR_LEN))
	{	
        macIsBroadcast = TRUE;
   	}
	if ((pKeyDesc->KeyLength == WEP_KEY_LEN_40) || 
		(pKeyDesc->KeyLength == WEP_KEY_LEN_104) || 
		(pKeyDesc->KeyLength == WEP_KEY_LEN_232))
	{	/* In Add WEP the MAC address is nulled, since it's irrelevant */
        macEqual2Associated = TRUE;
        wepKey = TRUE;
   	}

    if (pKeyDesc->KeyIndex & EXT_KEY_SUPP_AUTHENTICATOR_MASK)
    {  /* The key is being set by an Authenticator - not allowed in IBSS mode */
    	if (pKeyParser->pParent->pParent->pParent->pAdmCtrl->networkMode == RSN_IBSS)
        {	/* in IBSS only Broadcast MAC is allowed */
        	WLAN_REPORT_ERROR(pKeyParser->hReport, RSN_MODULE_LOG,
        					("EXT_KEY_PARSER: ERROR: Authenticator set key in IBSS mode !!!\n"));
        	return NOK;
        }

    }

    if (pKeyDesc->KeyIndex & EXT_KEY_REMAIN_BITS_MASK)
    {  /* the reamining bits in the key index are not 0 (when they should be) */
		WLAN_REPORT_ERROR(pKeyParser->hReport, RSN_MODULE_LOG,
						("EXT_KEY_PARSER: ERROR: Key index bits 8-27 should be 0 !!!\n"));
		return STATUS_BAD_KEY_PARAM;
    }
    
    encodedKeyMaterial.pData  = (char *) keyBuffer;
	/* Check key length according to the cipher suite - TKIP, etc...??? */
    if (wepKey)
    {
        if (!((pKeyDesc->KeyLength == WEP_KEY_LEN_40) || (pKeyDesc->KeyLength == WEP_KEY_LEN_104) 
              || (pKeyDesc->KeyLength == WEP_KEY_LEN_232)))
        {	/*Invalid key length*/
            WLAN_REPORT_ERROR(pKeyParser->hReport, RSN_MODULE_LOG,
                             ("WEP_KEY_PARSER: ERROR: Invalid Key length: %d !!!\n", pKeyDesc->KeyLength));
            return STATUS_BAD_KEY_PARAM;
        }

        os_memoryCopy(pKeyParser->hOs, &keyBuffer[0], pKeyDesc->KeyMaterial, pKeyDesc->KeyLength);
        if (!os_memoryCompare(pKeyParser->hOs, nullMacAddr, pKeyDesc->BSSID, MAC_ADDR_LEN))
        {   
            macIsBroadcast = TRUE;
        } 

        encodedKeyMaterial.keyLen = pKeyDesc->KeyLength;
    }
    else /* this is TKIP or CKIP */
    {   
        if ((pKeyDesc->KeyLength == CKIP_KEY_LEN) && (pKeyParser->pPaeConfig->unicastSuite == RSN_CIPHER_CKIP))
        {
            os_memoryCopy(pKeyParser->hOs, &keyBuffer[0], pKeyDesc->KeyMaterial, pKeyDesc->KeyLength);
            encodedKeyMaterial.keyLen = pKeyDesc->KeyLength;
        }
        else
        {
            os_memoryCopy(pKeyParser->hOs, &keyBuffer[MAC_ADDR_LEN+KEY_RSC_LEN],
                        pKeyDesc->KeyMaterial, pKeyDesc->KeyLength);

            encodedKeyMaterial.keyLen = MAC_ADDR_LEN+KEY_RSC_LEN+pKeyDesc->KeyLength;
        }
    }

    encodedKeyMaterial.keyId  = pKeyDesc->KeyIndex;

	WLAN_REPORT_INFORMATION(pKeyParser->hReport, RSN_MODULE_LOG,
						    ("EXT_KEY_PARSER: Key received keyId=%x, keyLen=%d \n",
						    pKeyDesc->KeyIndex, pKeyDesc->KeyLength
                             ));

    if (pKeyDesc->KeyIndex & EXT_KEY_PAIRWISE_GROUP_MASK)
    {	/* Pairwise key */
        /* check that the lower 8 bits of the key index are 0 */
        if (!wepKey && (pKeyDesc->KeyIndex & 0xff))
        {
			WLAN_REPORT_ERROR(pKeyParser->hReport, RSN_MODULE_LOG,
                              ("EXT_KEY_PARSER: ERROR: Pairwise key must have index 0 !!!\n"));
            return STATUS_BAD_KEY_PARAM;
        }

		if (macIsBroadcast)
		{
            WLAN_REPORT_ERROR(pKeyParser->hReport, RSN_MODULE_LOG,
                              ("EXT_KEY_PARSER: ERROR: Broadcast MAC address for unicast !!!\n"));
			return STATUS_BAD_KEY_PARAM;
		}
		if (pKeyDesc->KeyIndex & EXT_KEY_TRANSMIT_MASK)
		{	/* tx only pairwase key */
			/* set unicast keys */
        	if (pKeyParser->pUcastKey->recvSuccess!=NULL)
            {
        	status = pKeyParser->pUcastKey->recvSuccess(pKeyParser->pUcastKey, &encodedKeyMaterial);
            }
		} else {
			/* recieve only pairwase keys are not allowed */
            WLAN_REPORT_ERROR(pKeyParser->hReport, RSN_MODULE_LOG,
            ("EXT_KEY_PARSER: ERROR: recieve only pairwase keys are not allowed !!!\n"));
            return STATUS_BAD_KEY_PARAM;
		}

    }
    else
    {   /* set broadcast keys */
        if (!macIsBroadcast)
        {	/* not broadcast MAC */
        	if (pKeyParser->pParent->pParent->pParent->pAdmCtrl->networkMode == RSN_IBSS)
        	{	/* in IBSS only Broadcast MAC is allowed */
            	WLAN_REPORT_ERROR(pKeyParser->hReport, RSN_MODULE_LOG,
            					("EXT_KEY_PARSER: ERROR: not broadcast MAC in IBSS mode !!!\n"));
            	return STATUS_BAD_KEY_PARAM;
        	}
        	else if (!macEqual2Associated)
        	{	/* ESS mode and MAC is different than the associated one */
        		/* save the key for later */
				status = OK; /* pKeyParser->pBcastKey->saveKey(pKeyParser->pBcastKey, &encodedKey);*/
        	}
			else
			{	/* MAC is equal to the associated one - configure immediately */
                if (!wepKey)
				{
					os_memoryCopy(pKeyParser->hOs, &keyBuffer[0], broadcastMacAddr, MAC_ADDR_LEN);
				}
        		if (pKeyParser->pBcastKey->recvSuccess!=NULL)
                {
					status =  pKeyParser->pBcastKey->recvSuccess(pKeyParser->pBcastKey, &encodedKeyMaterial);
				}
			}
        }
		else
		{   /* MAC is broadcast - configure immediately */
			if (!wepKey)
			{
				os_memoryCopy(pKeyParser->hOs, &keyBuffer[0], broadcastMacAddr, MAC_ADDR_LEN);
			}
		 	
			/* set broadcast key */
			if (pKeyParser->pBcastKey->recvSuccess!=NULL)
			{
				status =  pKeyParser->pBcastKey->recvSuccess(pKeyParser->pBcastKey, &encodedKeyMaterial);
			}

			if (pKeyDesc->KeyIndex & EXT_KEY_TRANSMIT_MASK)
			{	/* Group key used to transmit */
				/* set as unicast key as well */
				if (pKeyParser->pUcastKey->recvSuccess!=NULL)
				{
					status = pKeyParser->pUcastKey->recvSuccess(pKeyParser->pUcastKey, &encodedKeyMaterial);
				}
			}
		}
    }
				  
	return status;
}




TI_STATUS keyParserExternal_remove(struct _keyParser_t *pKeyParser, UINT8 *pKeyData, UINT32 keyDataLen)
{
	TI_STATUS				status;
	OS_802_11_KEY	 		*pKeyDesc;
    paramInfo_t  			macParam;
	encodedKeyMaterial_t    encodedKeyMaterial;
	UINT8					broadcastMacAddr[MAC_ADDR_LEN] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
    UINT8                   keyBuffer[MAC_ADDR_LEN+KEY_RSC_LEN+MAX_EXT_KEY_DATA_LENGTH];

	if (pKeyData == NULL)
	{
		WLAN_REPORT_ERROR(pKeyParser->hReport, RSN_MODULE_LOG,
								  ("EXT_KEY_PARSER: ERROR: NULL KEY Data\n"));
		return NOK;
	}
	
	pKeyDesc = (OS_802_11_KEY*)pKeyData;

    if (pKeyDesc->KeyIndex & EXT_KEY_TRANSMIT_MASK)
	{	/* Bit 31 should always be zero */
		WLAN_REPORT_ERROR(pKeyParser->hReport, RSN_MODULE_LOG,
						  ("EXT_KEY_PARSER: ERROR: Remove TX bit in key index can't be 1\n"));
		return NOK;
	}
	if (pKeyDesc->KeyIndex & EXT_KEY_REMAIN_BITS_MASK)
	{	/* Bits 8-29 should always be zero */
		WLAN_REPORT_ERROR(pKeyParser->hReport, RSN_MODULE_LOG,
						  ("EXT_KEY_PARSER: ERROR: Remove none zero key index\n"));
		return NOK;
	}
	
	encodedKeyMaterial.keyId = pKeyDesc->KeyIndex;
	encodedKeyMaterial.keyLen = 0;
    encodedKeyMaterial.pData = (char *) keyBuffer;

	if (pKeyDesc->KeyIndex & EXT_KEY_PAIRWISE_GROUP_MASK)
	{	/* delete all pairwise keys or for the current BSSID */
		if (os_memoryCompare(pKeyParser->hOs, pKeyDesc->BSSID, broadcastMacAddr, MAC_ADDR_LEN))
		{
			os_memoryCopy(pKeyParser->hOs, keyBuffer, pKeyDesc->BSSID, MAC_ADDR_LEN);
		} else {
			macParam.paramType = CTRL_DATA_CURRENT_BSSID_PARAM;
			status = ctrlData_getParam(pKeyParser->hCtrlData, &macParam);
			if (status != OK)
			{
				WLAN_REPORT_ERROR(pKeyParser->hReport, RSN_MODULE_LOG,
								  ("EXT_KEY_PARSER: ERROR: Cannot get MAC address !!!\n"));
				return NOK;
			}
			
			os_memoryCopy(pKeyParser->hOs, keyBuffer, (void *)macParam.content.ctrlDataCurrentBSSID.addr, MAC_ADDR_LEN);
		}

        status =  pKeyParser->pUcastKey->pKeyDerive->remove(pKeyParser->pUcastKey->pKeyDerive, &encodedKeyMaterial);
	}
	else
	{	/* delete all group keys or for the current BSSID */
		os_memoryCopy(pKeyParser->hOs, keyBuffer, broadcastMacAddr, MAC_ADDR_LEN);
        status =  pKeyParser->pBcastKey->pKeyDerive->remove(pKeyParser->pUcastKey->pKeyDerive, &encodedKeyMaterial);
	}

	return status;
}
