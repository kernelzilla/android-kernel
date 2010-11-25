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
/***************************************************************************/
/*																		   */
/*		MODULE:													   */
/*		PURPOSE:		 						   */
/*																		   */
/***************************************************************************/
#include "fourX.h"
#include "report.h"
#include "osApi.h"
#include "utils.h"


static void fourXManager_resetAll4xCapabilities(fourX_t* pFourX);
static void fourXManager_resetAll_AP_4xCapabilities(fourX_t* pFourX);

static TI_STATUS fourXManager_InfoElementParsing(fourX_t* pFourX, 
												 dot11_4X_t* site4xParams,
												 fourX_Capabilities_t* pFourX_Capabilities);

static void setDefault4xCapabilities(fourX_Capabilities_t* pFourX_Capabilities);
static TI_STATUS build4XInfoElementVersion0(fourX_t* pFourX, 
									   dot11_4X_t* fourXInfoElemnt);

static TI_STATUS build4XInfoElementVersion1(fourX_t* pFourX, 
									   dot11_4X_t* fourXInfoElemnt);




/* 4X manager */
TI_STATUS fourXManager_evalSite(fourX_t* pFourX, 
								dot11_4X_t* site4xParams,
								UINT32 *matchingLevel)
{
	/* TODO - define 4x matching level */
	return OK;
}

TI_STATUS fourXManager_setSite(fourX_t* pFourX, 
							   dot11_4X_t* site4xParams)
{
	TI_STATUS	status;
	whalParamInfo_t param;


	status = fourXManager_InfoElementParsing(pFourX, 
											 site4xParams, 
											 &pFourX->ApFourX_Capabilities);
	if(status != OK)
	{
		/* set all 4x capabilities to 0 */
		fourXManager_resetAll4xCapabilities(pFourX);
		return NOK;
	}

   /*
	* configure fourX module and driver 4x Parameters
	*/

	/* configure concatenation parameters */
	if( (pFourX->desiredConcatenationEnable == TRUE) &&
		(pFourX->ApFourX_Capabilities.concatenationParams.enableDisable == TRUE) )
	{
		pFourX->concatenationEnable = TRUE;
	}
	else
	{
		pFourX->concatenationEnable = FALSE;
	}
	pFourX->currentMaxConcatSize = MIN(pFourX->desiredMaxConcatSize,pFourX->ApFourX_Capabilities.concatenationParams.concatenationSize);

   /*
	* set Hal Parameters
	*/
	param.paramType = HAL_CTRL_RTS_THRESHOLD_PARAM;
	param.content.halCtrlRtsThreshold = HAL_CTRL_RTS_THRESHOLD_MAX;

	whalCtrl_SetParam(pFourX->hWhalCtrl, &param);
	
	
	/* configure contentionWindow parameters */
	if( (pFourX->desiredCWMinEnable == TRUE) &&
		(pFourX->ApFourX_Capabilities.contentionWindowParams.enableDisable == TRUE) )
	{
		pFourX->CWMinEnable = TRUE;
	}
	else
	{
		pFourX->CWMinEnable = FALSE;

	}
	pFourX->currentCWMax = MIN(pFourX->desiredCWMax,pFourX->ApFourX_Capabilities.contentionWindowParams.CWMax);
	pFourX->currentCWMax = MAX(pFourX->desiredCWMin,pFourX->ApFourX_Capabilities.contentionWindowParams.CWMin);

	/* configure CWCombo parameters - DO NOT Supported */
	if( (pFourX->desiredCWComboEnable == TRUE) &&
		(pFourX->ApFourX_Capabilities.CWCombParams.enableDisable == TRUE) )
	{
		pFourX->CWComboEnable = TRUE;
	}
	else
	{
		pFourX->CWComboEnable = FALSE;

	}

	/* configure CWCombo parameters */
	if( (pFourX->desiredAckEmulationEnable == TRUE) &&
		(pFourX->ApFourX_Capabilities.ackEmulationParams.enableDisable == TRUE) )
	{
#ifdef ACK_EMUL
		pFourX->ackEmulationEnable = TRUE;
#else
		pFourX->ackEmulationEnable = FALSE;
#endif
	}
	else
	{
		pFourX->ackEmulationEnable = FALSE;

	}
	
	/* configure CWCombo parameters - DO NOT Supported */
	if( (pFourX->desiredERP_ProtectionEnable == TRUE) &&
		(pFourX->ApFourX_Capabilities.ERP_ProtectionParams.enableDisable == TRUE) )
	{
		pFourX->ERP_ProtectionEnable = TRUE;
	}
	else
	{
		pFourX->ERP_ProtectionEnable = FALSE;

	}

	return OK;
}


static TI_STATUS fourXManager_InfoElementParsing(fourX_t* pFourX, 
												 dot11_4X_t* site4xParams,
												 fourX_Capabilities_t* pFourX_Capabilities)
{
	char *capPtr;
	UINT32 len;
    UINT16 capLen;
	WlanTIcap_t cap_type;
	UINT8 ti_oui[] = TI_OUI;

	if(site4xParams == NULL)
	{
		fourXManager_resetAll_AP_4xCapabilities(pFourX);
		return NOK;
	}

   /*
	* IE parsing 
	*/
	if( (site4xParams->hdr.eleId != DOT11_4X_ELE_ID) ||
		(site4xParams->hdr.eleLen > DOT11_4X_MAX_LEN) ||
		(os_memoryCompare(pFourX->hOs ,ti_oui, (PUINT8)site4xParams->fourXCapabilities, DOT11_OUI_LEN) != 0) )
	{
		fourXManager_resetAll_AP_4xCapabilities(pFourX);
		return NOK;
	}

   /* 
	* check protocol version 
	*/
	if( ((site4xParams->fourXCapabilities[DOT11_OUI_LEN]) & 0xF) == FOUR_X_PROTOCOL_VERSION_0)
	{
	   /* 
		* version 0 
		*/
		pFourX_Capabilities->fourXProtocolVersion = FOUR_X_PROTOCOL_VERSION_0;
		setDefault4xCapabilities(pFourX_Capabilities);
		return OK;
	}
	else
	if( ((site4xParams->fourXCapabilities[DOT11_OUI_LEN]) & 0xF) == FOUR_X_PROTOCOL_VERSION_1)
	{
	   /* 
		* version 1
		*/
		pFourX_Capabilities->fourXProtocolVersion = FOUR_X_PROTOCOL_VERSION_1;

		capPtr = (char *)site4xParams;
		capPtr += (sizeof(dot11_eleHdr_t) + DOT11_OUI_LEN + 1); /*IE header + TI_OUI + version field */; 

		len = DOT11_OUI_LEN+1; /* TI_OUI + version field */
						
		while(len < (UINT32)(site4xParams->hdr.eleLen)) /* Dm: Security fix */
		{
			cap_type = (WlanTIcap_t)(*((UINT16*)(capPtr)));
			
			switch(cap_type)
			{
			case TI_CAP_4X_CONCATENATION:							
				pFourX_Capabilities->concatenationParams.enableDisable = TRUE;
				capPtr += 2;
				capLen = *((UINT16*)(capPtr));
				capPtr += 2;
				pFourX_Capabilities->concatenationParams.concatenationSize = *((UINT16*)(capPtr));
				capPtr += 2;
				len += sizeof(UINT16) /* type field */ + sizeof(UINT16) /* len field */ + (UINT32)capLen /* val fields */;
				break;

			case TI_CAP_4X_CONT_WINDOW:
				pFourX_Capabilities->contentionWindowParams.enableDisable = TRUE;
				capPtr += 2;
				capLen = *((UINT16*)(capPtr));
				capPtr += 2;
				pFourX_Capabilities->contentionWindowParams.CWMin = *((UINT16*)(capPtr));
				capPtr += 2;
				pFourX_Capabilities->contentionWindowParams.CWMax = *((UINT16*)(capPtr));
				capPtr += 2;
				len += sizeof(UINT16) /* type field */ + sizeof(UINT16) /* len field */ + (UINT32)capLen /* val field */;
				break;

			case TI_CAP_4X_CONT_WINDOW_COMBO:
				pFourX_Capabilities->CWCombParams.enableDisable = TRUE;
				capPtr += 2;
				capLen = *((UINT16*)(capPtr));
				capPtr += 2;
				pFourX_Capabilities->CWCombParams.DIFS = *((UINT16*)(capPtr));
				capPtr += 2;
				pFourX_Capabilities->CWCombParams.SLOT = *((UINT16*)(capPtr));
				capPtr += 2;
				pFourX_Capabilities->CWCombParams.CWMin = *((UINT16*)(capPtr));
				capPtr += 2;
				len += sizeof(UINT16) /* type field */ + sizeof(UINT16) /* len field */ + (UINT32)capLen /* val field */;
				break;

			case TI_CAP_4X_TCP_ACK_EMUL:
				pFourX_Capabilities->ackEmulationParams.enableDisable = TRUE;
				capPtr += 2;
				capLen = *((UINT16*)(capPtr));
				capPtr += 2;
				len += sizeof(UINT16) /* type field */ + sizeof(UINT16) /* len field */ + (UINT32)capLen /* val field */;
				break;

			case TI_CAP_TRICK_PACKET_ERP:
				pFourX_Capabilities->ERP_ProtectionParams.enableDisable = TRUE;
				capPtr += 2;
				capLen = *((UINT16*)(capPtr));
				capPtr += 2;
				len += sizeof(UINT16) /* type field */ + sizeof(UINT16) /* len field */ + (UINT32)capLen /* val field */;
				break;
			
			default:
				WLAN_REPORT_INFORMATION(pFourX->hReport, FOUR_X_MODULE_LOG, 
						("unrecognized capability in TI IE: %d\n",cap_type));
				return NOK;
			}
		}
	}
	else
	{
	   /* 
		* unknown version
		*/
		fourXManager_resetAll_AP_4xCapabilities(pFourX);
		return NOK;
	}

	return OK;
}

TI_STATUS fourXManager_get4xInfoElemnt(fourX_t* pFourX, 
									   dot11_4X_t* fourXInfoElemnt)
{
    TI_STATUS Status;
	if(pFourX->ApFourX_Capabilities.fourXProtocolVersion ==	FOUR_X_PROTOCOL_VERSION_0)
	{
		Status = build4XInfoElementVersion0(pFourX, fourXInfoElemnt);
	}
	else
	if(pFourX->ApFourX_Capabilities.fourXProtocolVersion == FOUR_X_PROTOCOL_VERSION_1)
	{
		Status = build4XInfoElementVersion1(pFourX, fourXInfoElemnt);
	}
	else
	{
		WLAN_REPORT_INFORMATION(pFourX->hReport, FOUR_X_MODULE_LOG, 
				("fourXManager_get4xInfoElemnt: Versionm unknown\n"));
		return NOK;
	}
	
	return Status;
}

static TI_STATUS build4XInfoElementVersion0(fourX_t* pFourX, 
									   dot11_4X_t* fourXInfoElemnt)
{
	UINT8 ti_oui[] = TI_OUI;

	WLAN_REPORT_INFORMATION(pFourX->hReport, FOUR_X_MODULE_LOG, 
				("build4XInfoElementVersion0: build IE version 0\n"));

	/* 4x Protocol version should support ALL  version 0 features */
	if( (pFourX->desiredConcatenationEnable == FALSE) ||
		(pFourX->desiredCWMinEnable == FALSE) )
	{
		return NOK;
	}

	fourXInfoElemnt->hdr.eleId = TI_4X_IE_ID;
	fourXInfoElemnt->hdr.eleLen = FOUR_X_INFO_ELEMENT_VERSION_0_LEN;

	os_memoryCopy(pFourX->hOs, (PUINT8)fourXInfoElemnt->fourXCapabilities, ti_oui, DOT11_OUI_LEN);
	fourXInfoElemnt->fourXCapabilities[DOT11_OUI_LEN] = FOUR_X_PROTOCOL_VERSION_0;
    return OK;
}

static TI_STATUS build4XInfoElementVersion1(fourX_t* pFourX, 
									   dot11_4X_t* fourXInfoElemnt)
{
	UINT8 len = 0;
	UINT8 *capPtr; 
	UINT8 ti_oui[] = TI_OUI;

	WLAN_REPORT_INFORMATION(pFourX->hReport, FOUR_X_MODULE_LOG, 
				("build4XInfoElementVersion0: build IE version 1\n"));

	fourXInfoElemnt->hdr.eleId = TI_4X_IE_ID;

	capPtr = (UINT8 *)fourXInfoElemnt->fourXCapabilities;

	os_memoryCopy(pFourX->hOs, capPtr, ti_oui, DOT11_OUI_LEN);
	capPtr += DOT11_OUI_LEN ;
	len += DOT11_OUI_LEN ;

	*capPtr = FOUR_X_PROTOCOL_VERSION_1;
	capPtr += 1;
	len += 1;


	if(pFourX->desiredConcatenationEnable == TRUE)
	{
		*((UINT16*)capPtr) = FOUR_X_CONCAT_CAP_ID;
		capPtr += 2;
		len += 2;
		*((UINT16*)capPtr) = FOUR_X_CONCAT_CAP_LEN;
		capPtr += 2;
		len += 2;
		*((UINT16*)capPtr) = 4095;
		capPtr += 2;
		len += 2;
	}
		
	
	fourXInfoElemnt->hdr.eleLen = len;
    return OK;

}

static void fourXManager_resetAll4xCapabilities(fourX_t* pFourX)
{
		pFourX->concatenationEnable = FALSE;
		pFourX->CWMinEnable = FALSE;
		pFourX->CWComboEnable = FALSE;
		pFourX->ackEmulationEnable = FALSE;
		pFourX->ERP_ProtectionEnable = FALSE;
}

static void fourXManager_resetAll_AP_4xCapabilities(fourX_t* pFourX)
{
	os_memoryZero(pFourX->hOs, &pFourX->ApFourX_Capabilities, (sizeof(fourX_Capabilities_t)));
}

static void setDefault4xCapabilities(fourX_Capabilities_t* pFourX_Capabilities)
{
	/* Concatenation */
	pFourX_Capabilities->concatenationParams.enableDisable = TRUE;
	pFourX_Capabilities->concatenationParams.concatenationSize = MAX_CONCAT_SIZE_DEF;
	
	/* CW min */
	pFourX_Capabilities->contentionWindowParams.enableDisable = TRUE;
	pFourX_Capabilities->contentionWindowParams.CWMax = DEF_CW_MAX;
	pFourX_Capabilities->contentionWindowParams.CWMin = DEF_CW_MIN;

	/* CW combo */
	pFourX_Capabilities->CWCombParams.enableDisable = FALSE;
	pFourX_Capabilities->CWCombParams.CWMin = DEF_CW_COMBO_CW_MIN;
	pFourX_Capabilities->CWCombParams.DIFS = DEF_CW_COMBO_DIFS;
	pFourX_Capabilities->CWCombParams.SLOT = DEF_CW_COMBO_SLOT;

	/* Ack Emulation */
	pFourX_Capabilities->ackEmulationParams.enableDisable = TRUE;

	/* ERP protection */
	pFourX_Capabilities->ERP_ProtectionParams.enableDisable = FALSE;


}


