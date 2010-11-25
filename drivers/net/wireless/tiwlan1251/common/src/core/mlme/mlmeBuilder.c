 /** \file mlmeBuilder.c
 *  \brief 802.11 MLME Builder
 *
 *  \see mlmeBuilder.h
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

/***************************************************************************/
/*																		   */
/*		MODULE:	mlmeBuilder.c											   */
/*    PURPOSE:	802.11 MLME Builder										   */
/*																	 	   */
/***************************************************************************/



#include "osApi.h"

#include "paramOut.h"
#include "paramIn.h"

#include "utils.h"

#include "report.h"

#include "802_11Defs.h"

#include "DataCtrl_Api.h"
#include "memMngrEx.h"

#include "mlmeApi.h"
#include "mlmeSm.h"
#include "mlmeBuilder.h"
#include "srcApi.h"
#include "TNETW_Driver_types.h"
/* Constants */

/* Enumerations */

/* Typedefs */

/* Structures */

/* External data definitions */

/* External functions definitions */

/* Local function prototypes */

/* Functions */

TI_STATUS mlmeBuilder_sendFrame(TI_HANDLE hMlme, 
							 dot11MgmtSubType_e type, 
							 UINT8 *pDataBuff, 
							 UINT32 dataLen,
							 UINT8	setWepOpt)
{
	TI_STATUS			status;
	mlme_t			*pHandle;
	mem_MSDU_T		*pMsdu;
	paramInfo_t		daParam, saParam;
	dot11_mgmtFrame_t	*pFrame;



	if (hMlme == NULL)
	{
		return NOK;
	}

	pHandle = (mlme_t*)hMlme;


	/* GET NEW MSDU !!! */
	status = wlan_memMngrAllocMSDU(pHandle->hMemMgr, &pMsdu, 
        MAX_MANAGEMENT_FRAME_BODY_LEN + 
            configMgr_getPacketHeaderLength(pHandle->hConfigMgr, NULL, TX_DATA_MGMT_MSDU), 
        MLME_MODULE);
	
    if (status != OK)
		return NOK;
	pFrame = (dot11_mgmtFrame_t*)(pMsdu->firstBDPtr->data + TX_TOTAL_OFFSET_BEFORE_DATA);

	status = mlmeBuilder_buildFrameCtrl(pHandle, type, (UINT16 *)&pFrame->hdr.fc, setWepOpt);
	if (status != OK)
	{
		wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);
		return NOK;
	}

	daParam.paramType = CTRL_DATA_CURRENT_BSSID_PARAM;
	status = ctrlData_getParam(pHandle->hCtrlData, &daParam);
	if (status != OK)
	{
		wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);
		return NOK;
	}

	/* copy destination mac address */
	MAC_COPY(pHandle->hOs, (&pFrame->hdr.DA), (&daParam.content.ctrlDataCurrentBSSID));

	saParam.paramType = CTRL_DATA_MAC_ADDRESS;
	status = ctrlData_getParam(pHandle->hCtrlData, &saParam);
	if (status != OK)
	{
		wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);
		return NOK;
	}

	/* copy source mac address */
	MAC_COPY(pHandle->hOs, (&pFrame->hdr.SA), (&saParam.content.ctrlDataCurrentBSSID));

	/* copy BSSID (destination mac address) */
	MAC_COPY(pHandle->hOs, (&pFrame->hdr.BSSID), (&daParam.content.ctrlDataCurrentBSSID));

	if (pDataBuff != NULL)
	{
		os_memoryCopy(pHandle->hOs, (void *)pFrame->body, pDataBuff, dataLen);
	}

	/* Update MSDU parameters */
	pMsdu->headerLen = sizeof(dot11_mgmtHeader_t);
	pMsdu->dataLen = sizeof(dot11_mgmtHeader_t) + dataLen;
	pMsdu->firstBDPtr->dataOffset = TX_TOTAL_OFFSET_BEFORE_DATA;
    pMsdu->firstBDPtr->length = pMsdu->dataLen + pMsdu->firstBDPtr->dataOffset;
 
	/* send the packet to the TX */
	pMsdu->qosTag = 0;
	pMsdu->txFlags |= TX_DATA_MGMT_MSDU;
	
	/* 
     * sign the Disassoc packet 
     * A disassociate indication (if occurs) will be provided via TxComplete
     */
	if (type == DIS_ASSOC)
		pMsdu->txCompleteFlags |= TX_DATA_DISASSOC_SYNC_TRIG;
	
	/* sign the De Auth packet 
     * A De Auth indication (if occurs) will be provided via TxComplete
     */
	if (type == DE_AUTH)
		pMsdu->txCompleteFlags |= TX_DATA_DEAUTH_SYNC_TRIG;

	status = txData_txSendMsdu(pHandle->hTxData, pMsdu);

	return status;
}

TI_STATUS mlmeBuilder_buildFrameCtrl(mlme_t* pMlme, dot11MgmtSubType_e type, UINT16* pFctrl, UINT8 setWepOpt)
{
	*pFctrl = 0;

	switch (type)
	{
	case ASSOC_REQUEST:
		*pFctrl |= DOT11_FC_ASSOC_REQ;
		break;
	case ASSOC_RESPONSE:
		*pFctrl |= DOT11_FC_ASSOC_RESP;
		break;
	case RE_ASSOC_REQUEST:
		*pFctrl |= DOT11_FC_REASSOC_REQ;
		break;
	case RE_ASSOC_RESPONSE:
		*pFctrl |= DOT11_FC_REASSOC_RESP;
		break;
	case DIS_ASSOC:
		*pFctrl |= DOT11_FC_DISASSOC;
		break;
	case AUTH:
		*pFctrl |= DOT11_FC_AUTH;
		break;
	case DE_AUTH:
		*pFctrl |= DOT11_FC_DEAUTH;
		break;
	case ACTION:
		*pFctrl |= DOT11_FC_ACTION;
		break;
	default:
		return NOK;
	}
	
	if (setWepOpt)
	{
		*pFctrl |= DOT11_FC_WEP;
	}


	return OK;
}

