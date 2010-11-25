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
 *
 *   MODULE:  whalHwMboxCmd.c
 *   PURPOSE: wlan hardware commands handler
 * 
 ****************************************************************************/

#include "whalCommon.h"
#include "whalHwMboxCmd.h"
#include "whalBus_Api.h"
#include "CmdQueue_api.h"

/* Local Macros */

#define MAC_TO_VENDOR_PREAMBLE(mac) ((mac[0] << 16) | (mac[1] << 8) | mac[2])


/****************************************************************************
 *                      whal_hwMboxCmd_Create()
 ****************************************************************************
 * DESCRIPTION:	Create the mailbox commands object
 * 
 * INPUTS:	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	The Created object
 ****************************************************************************/
HwMboxCmd_T *whal_hwMboxCmd_Create(TI_HANDLE hOs, WhalParams_T *pWhalParams)
{
	HwMboxCmd_T *pObj;

	pObj = os_memoryAlloc(hOs, sizeof(HwMboxCmd_T));
	if (pObj == NULL)
		return NULL;

	os_memoryZero(hOs, (void *)pObj, sizeof(HwMboxCmd_T));

	pObj->hOs = hOs;
	pObj->pWhalParams = pWhalParams;

	return(pObj);
}

/****************************************************************************
 *                      whal_hwMboxCmd_Destroy()
 ****************************************************************************
 * DESCRIPTION:	Destroy the object 
 * 
 * INPUTS:	
 *		pHwMboxCmd		The object to free
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_Destroy(HwMboxCmd_T *pHwMboxCmd)
{
	if (pHwMboxCmd)
		os_memoryFree(pHwMboxCmd->hOs, pHwMboxCmd, sizeof(HwMboxCmd_T));
	return OK;
}

/****************************************************************************
 *                      whal_hwMboxCmd_Config()
 ****************************************************************************
 * DESCRIPTION:	Configure the object 
 * 
 * INPUTS: 	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_Config(HwMboxCmd_T *pHwMboxCmd, TI_HANDLE hCmdMboxQueue, TI_HANDLE hReport)
{
	pHwMboxCmd->hReport = hReport;
	pHwMboxCmd->hCmdMboxQueue = hCmdMboxQueue;
	return OK;
}

/****************************************************************************
 *                      whal_hwMboxCmd_StartBss()
 ****************************************************************************
 * DESCRIPTION:	Construct the StartBss command fileds and send it to the mailbox
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_StartBss(HwMboxCmd_T *pHwMboxCmd, BSS_e BssType, void *JoinCompleteCB, TI_HANDLE CB_handle)
{
	StartJoinRequest_t AcxCmd_StartBss;
	StartJoinRequest_t *pCmd = &AcxCmd_StartBss;
	dot11_SSID_t *pWlanElm_Ssid = whal_ParamsGetElm_Ssid(pHwMboxCmd->pWhalParams);
	BssInfoParams_T *pBssInfoParams = whal_ParamsGetBssInfoParams(pHwMboxCmd->pWhalParams);
	UINT8 *BssId;
	UINT8 *cmdBssId;
	UINT16 HwBasicRatesBitmap;
	UINT16 HwSupportedRatesBitmap;
	int i; 
	
	os_memoryZero(pHwMboxCmd->hOs, (void *)pCmd, sizeof(StartJoinRequest_t));

	/*
    * Set RxCfg and RxFilterCfg values
    */
    pCmd->rxFilter.ConfigOptions = ENDIAN_HANDLE_LONG( pHwMboxCmd->pWhalParams->WlanParams.RxConfigOption );
    pCmd->rxFilter.FilterOptions = ENDIAN_HANDLE_LONG( pHwMboxCmd->pWhalParams->WlanParams.RxFilterOption );

    pCmd->beaconInterval = ENDIAN_HANDLE_WORD(whal_ParamsGetBeaconInterval(pHwMboxCmd->pWhalParams));
	pCmd->dtimInterval = whal_ParamsGetDtimCount(pHwMboxCmd->pWhalParams); 
	pCmd->channelNumber = whal_ParamsGetRadioChannel(pHwMboxCmd->pWhalParams);
	

	pCmd->bssType = BssType;
	
	/* Add radio band */
	pCmd->bssType |= whal_ParamsGetRadioBand (pHwMboxCmd->pWhalParams) << 4;
    pCmd->ctrl = pBssInfoParams->Ctrl; /* Only bit 7 is currently in use in the Ctrl field, bit 7 indicates if to flash the Tx queues */

	
    /*
	 * BasicRateSet
	 * The wlan hardware uses pHwMboxCmd field to determine the rate at which to transmit 
	 * control frame responses (such as ACK or CTS frames)
	 */
	whalUtils_ConvertAppRatesBitmap(pBssInfoParams->BasicRateSet, 0, &HwBasicRatesBitmap);

	
    /*
	 * SupportedRateSet
	 * The wlan hardware uses pHwMboxCmd information to determine which rates are allowed 
	 * when rate fallback is enabled
	 */
	whalUtils_ConvertAppRatesBitmap(pBssInfoParams->SupportedRateSet, 0, &HwSupportedRatesBitmap);
	pCmd->basicRateSet  = ENDIAN_HANDLE_WORD(HwBasicRatesBitmap);

	/*
	 * ACX100_Frm_Rate
	 * pHwMboxCmd field indicates the rate at which the wlan hardware transmits 
	 * Beacon, Probe Response, RTS, and PS-Poll frames
	 */
	pCmd->txCtrlFrmRate = pBssInfoParams->txCtrlFrmRate;

	WLAN_REPORT_INFORMATION(pHwMboxCmd->hReport, HAL_HW_CTRL_MODULE_LOG,  
		("txCtrlFrmRate = 0x%x\n", pCmd->txCtrlFrmRate));
	/*
	 * PreambleOpt
	 * indicates the PLCP preamble type to use when transmitting 
	 * Beacon, Probe Response, RTS, and PS-Poll frames.
	 */
	pCmd->txCtrlFrmMod = pBssInfoParams->txCtrlFrmModulation;

	/* BSS ID - reversed order (see wlan hardware spec) */
	BssId = whal_ParamsGetBssId(pHwMboxCmd->pWhalParams);
    cmdBssId = (UINT8*)&pCmd->bssIdL;
	for (i=0; i<MAC_ADDR_SIZE; i++)
		cmdBssId[i] = BssId[MAC_ADDR_SIZE-1-i];

    /* SSID string */ 
    pCmd->ssidLength = pWlanElm_Ssid->hdr.eleLen;
    os_memoryCopy(pHwMboxCmd->hOs, (void *)(pCmd->ssidStr),(void *)pWlanElm_Ssid->serviceSetId, pWlanElm_Ssid->hdr.eleLen);

	/* Set the Frame Rate and The Frame Mode */
	pCmd->txMgmtFrmRate = pBssInfoParams->txMgmtFrmRate;
	pCmd->txMgmtFrmMod  = pBssInfoParams->txMgmtFrmModulation;

    /* Send the command */
	if(JoinCompleteCB == NULL)
		return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CMD_START_JOIN, (char *)pCmd, sizeof(*pCmd)));
	else
		return (CmdQueue_CommandWithCb(pHwMboxCmd->hCmdMboxQueue, CMD_START_JOIN, (char *)pCmd, sizeof(*pCmd), (void *)JoinCompleteCB, CB_handle, NULL));

}


/****************************************************************************
 *                      whal_hwMboxCmd_LNAControl()
 ****************************************************************************
 * DESCRIPTION:	Construct the LNA control command fileds and send it to the mailbox
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_LNAControl(HwMboxCmd_T *pHwMboxCmd, UINT8 LNAControlField)
{
	LNAControl_t AcxCmd_LNAControl;
	LNAControl_t *pCmd = &AcxCmd_LNAControl;
	
	pCmd->LNAControlField = LNAControlField;
	
	return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CMD_LNA_CONTROL , (char *)pCmd, sizeof(*pCmd)));
}


/****************************************************************************
 *                      whal_hwMboxCmd_Reset()
 ****************************************************************************
 * DESCRIPTION:	Reset the Rx wlan hardware
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_RxReset(HwMboxCmd_T *pHwMboxCmd)
{
	/*
	 * no parameters 
	 */
	
	/*
	 * Send the command 
	 */
	return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CMD_RX_RESET, NULL, 0));
}

/****************************************************************************
 *                      whal_hwMboxCmd_Reset()
 ****************************************************************************
 * DESCRIPTION:	Reset the wlan hardware
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_Reset(HwMboxCmd_T *pHwMboxCmd)
{
	/*
	 * no parameters 
	 */
	
	/*
	 * Send the command 
	 */
	return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CMD_RESET, NULL, 0));
}

/****************************************************************************
 *                      whal_hwMboxCmd_EnableRx()
 ****************************************************************************
 * DESCRIPTION:	Construct the EnableRx command fileds and send it to the mailbox
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_EnableRx(HwMboxCmd_T *pHwMboxCmd)
{
	UINT8 ChannelNumber;

	ChannelNumber = whal_ParamsGetDefaultChannel(pHwMboxCmd->pWhalParams);

	/*
	 * Send the command 
	 */
	return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CMD_ENABLE_RX, (char *)&ChannelNumber, sizeof(UINT8)));
}

/****************************************************************************
 *                      whal_hwMboxCmd_EnableTx()
 ****************************************************************************
 * DESCRIPTION:	Construct the EnableTx command fileds and send it to the mailbox
 *				Note: This Enable_TX command is used also for changing the serving 
 *				channel.
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_EnableTx(HwMboxCmd_T *pHwMboxCmd ,UINT8	defaultChannel)
{
	UINT8 ChannelNumber;
	
	ChannelNumber = defaultChannel;
	/*
	 * Send the command 
	 */
	return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CMD_ENABLE_TX, (char *)&ChannelNumber, sizeof(UINT8)));
}

/****************************************************************************
 *                      whal_hwMboxCmd_DisableRx()
 ****************************************************************************
 * DESCRIPTION:	Construct the DisableRx command fileds and send it to the mailbox
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_DisableRx(HwMboxCmd_T *pHwMboxCmd)
{
	/*
	 * no parameters 
	 */
	
	/*
	 * Send the command 
	 */
	return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CMD_DISABLE_RX, NULL, 0));
}

/****************************************************************************
 *                      whal_hwMboxCmd_DisableTx()
 ****************************************************************************
 * DESCRIPTION:	Construct the DisableTx command fileds and send it to the mailbox
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_DisableTx(HwMboxCmd_T *pHwMboxCmd)
{
	/*
	 * no parameters 
	 */
	
	/*
	 * Send the command 
	 */
	return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CMD_DISABLE_TX, NULL, 0));
}

/****************************************************************************
 *                      whal_hwMboxCmd_ConfigureTemplateFrame()
 ****************************************************************************
 * DESCRIPTION:	Generic function which sets the Fw with a template frame according
 *              to the given template type.
 * 
 * INPUTS: templateType - CMD_BEACON, CMD_PROBE_REQ, CMD_PROBE_RESP etc.
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_ConfigureTemplateFrame(HwMboxCmd_T *pHwMboxCmd, UINT8 *pFrame, UINT16 FrameSize,
                                          Command_e templateType, void *CBFunc,TI_HANDLE CBObj)
{
    PktTemplate_t AcxCmd_PktTemplate;
	PktTemplate_t *pCmd = &AcxCmd_PktTemplate;

    /* if the frame size is too big - we truncate the frame template */
    if (FrameSize > MAX_TEMPLATES_SIZE)
    {
        WLAN_REPORT_ERROR(pHwMboxCmd->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("%s: Frame size (=%d) is bigger than MAX_TEMPLATES_SIZE(=%d) !!!\n",
            __FUNCTION__,FrameSize,MAX_TEMPLATES_SIZE));

        FrameSize = MAX_TEMPLATES_SIZE;
    }
    
    /* if pFrame is NULL than it means that we just want to reserve place in Fw, and there is no need to copy */
    if (pFrame != NULL) 
    {
        /* Set configuration fields */
	    os_memoryCopy(pHwMboxCmd->hOs, (void *)&pCmd->templateStart, (void *)pFrame, FrameSize);
    }
	pCmd->len = ENDIAN_HANDLE_WORD(FrameSize);

    if (NULL == CBFunc)
	{
		return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, templateType, (char *)pCmd, FrameSize + sizeof(pCmd->len)));
	}
	else
	{
		return (CmdQueue_CommandWithCb(pHwMboxCmd->hCmdMboxQueue, templateType, (char *)pCmd, FrameSize + sizeof(pCmd->len),
			CBFunc,CBObj,NULL));
	}
}

/****************************************************************************
 *                      whal_hwMboxCmd_TimTemplate()
 ****************************************************************************
 * DESCRIPTION:	Build the tim template fileds and send it to the mailbox
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_TimTemplate(HwMboxCmd_T *pHwMboxCmd, char BmapControl, char *PartialBmapVec, int PartialBmapLen)
{ 
	VBMUpdateRequest_t AcxCmd_ConfTim;
	VBMUpdateRequest_t *pCmd = &AcxCmd_ConfTim;

	/*
	 * Set command fields 
	 */
	os_memoryZero(pHwMboxCmd->hOs, (void *)pCmd, sizeof(*pCmd));

	/* 
	 * Copy the fields and set sizes 
	 */
	pCmd->tim.BitMap_ctrl = BmapControl;
	/* dtimCount and dtimPeriod will be filled by the wlan hardware */
	os_memoryCopy(pHwMboxCmd->hOs, (void *)pCmd->tim.PVB_field, (void *)PartialBmapVec, PartialBmapLen);
	pCmd->tim.identity  = DOT11_TIM_ELE_ID;
	pCmd->tim.length    = 3+PartialBmapLen;
	pCmd->len = ENDIAN_HANDLE_WORD(5+PartialBmapLen);

	/*
	 * Send the command 
	 */
	return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CMD_VBM, (char *)pCmd, sizeof(*pCmd)));

}


/****************************************************************************
 *                      whal_hwMboxCmd_SetKey()
 ****************************************************************************
 * DESCRIPTION:	Construct the SetKey command fileds and send it to the mailbox
 * 
 * INPUTS: 
 *		Action		- add/remove key
 *		MacAddr		- relevant only for mapping keys
 *		KeySize		- key size
 *		KeyType		- default/mapping/TKIP
 *		KeyId		- relevant only for default keys
 *		Key			- key data
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_SetKey(HwMboxCmd_T *pHwMboxCmd, int Action, char *MacAddr, int KeySize,
						  int KeyType, int KeyId, char *Key, UINT16 SecuritySeqNumLow, UINT32 SecuritySeqNumHigh,
						  void *CB_Func, TI_HANDLE CB_handle)
{
	SetKey_t AcxCmd_SetKey;
	SetKey_t *pCmd = &AcxCmd_SetKey;

	/*
	* Set command fields
	*/
	os_memoryZero(pHwMboxCmd->hOs, (void *)pCmd, sizeof(*pCmd));

	/*
	* Command fields
	*/
	os_memoryCopy(pHwMboxCmd->hOs, (void *)pCmd->addr, (void *)MacAddr, MAC_ADDR_SIZE);
	if (KeySize > MAX_KEY_SIZE)
	{
		os_memoryCopy(pHwMboxCmd->hOs, (void *)pCmd->key, (void *)Key, MAX_KEY_SIZE);
	} else {
		os_memoryCopy(pHwMboxCmd->hOs, (void *)pCmd->key, (void *)Key, KeySize);
	}
	pCmd->action = (UINT8)Action;
	pCmd->keySize = (UINT8)KeySize;
	pCmd->type = (UINT8)KeyType;
	pCmd->id = (UINT8)KeyId;
	pCmd->ssidProfile = 0;

	/*
	* Preserve TKIP/AES security sequence number after recovery.
	* Note that our STA Tx is currently using only one sequence-counter
	* for all ACs (unlike the Rx which is separated per AC).
	*/
	pCmd->AcSeqNum16[0] = SecuritySeqNumLow;
	pCmd->AcSeqNum16[1] = 0;
	pCmd->AcSeqNum16[2] = 0;
	pCmd->AcSeqNum16[3] = 0;

	pCmd->AcSeqNum32[0] = SecuritySeqNumHigh;
	pCmd->AcSeqNum32[1] = 0;
	pCmd->AcSeqNum32[2] = 0;
	pCmd->AcSeqNum32[3] = 0;


	WLAN_REPORT_WARNING(pHwMboxCmd->hReport, HAL_HW_CTRL_MODULE_LOG,
		("*********************** \n"));

	WLAN_REPORT_WARNING(pHwMboxCmd->hReport, HAL_HW_CTRL_MODULE_LOG,
		("whal_hwMboxCmd_SetKey \n"));

	WLAN_REPORT_WARNING(pHwMboxCmd->hReport, HAL_HW_CTRL_MODULE_LOG,
		("*********************** \n"));

    WLAN_REPORT_WARNING(pHwMboxCmd->hReport, HAL_HW_CTRL_MODULE_LOG,
		("addr = %x:%x:%x:%x:%x:%x \n",
         pCmd->addr[0], pCmd->addr[1], pCmd->addr[2], pCmd->addr[3], pCmd->addr[4], pCmd->addr[5] ));

	WLAN_REPORT_WARNING(pHwMboxCmd->hReport, HAL_HW_CTRL_MODULE_LOG,
		(" key= %x\n, action= %d\n keySize= %d\n type= %d\n id= %d\n AcSeqNum16[0]= %d\n AcSeqNum32[0]= %d\n",
		pCmd->key,pCmd->action,pCmd->keySize,pCmd->type, pCmd->id, pCmd->AcSeqNum16[0], pCmd->AcSeqNum32[0] ));

	WLAN_REPORT_WARNING(pHwMboxCmd->hReport, HAL_HW_CTRL_MODULE_LOG,
		("*********************** \n"));

	/*
	* Send the command
	*/
	if((CB_Func == NULL) || (CB_handle == NULL))
	{
		WLAN_REPORT_WARNING(pHwMboxCmd->hReport, HAL_HW_CTRL_MODULE_LOG,
			("*** whal_hwMboxCmd_SetKey:Calling CmdQueue_Command *** \n"));

		return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CMD_SET_KEYS, (char *)pCmd, sizeof(*pCmd)));
	}
	else
	{
		WLAN_REPORT_WARNING(pHwMboxCmd->hReport, HAL_HW_CTRL_MODULE_LOG,
			("!!! whal_hwMboxCmd_SetKey:CmdQueue_CommandwithCB !!! \n"));

		return (CmdQueue_CommandWithCb(pHwMboxCmd->hCmdMboxQueue, CMD_SET_KEYS, (char *)pCmd, sizeof(*pCmd), CB_Func, CB_handle, NULL));
	}
}


/****************************************************************************
 *                      whal_hwMboxCmd_Test()
 ****************************************************************************
 * DESCRIPTION:	Test NO WAIT (use EnableRx command)
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_Test(HwMboxCmd_T *pHwMboxCmd)
{
	UINT8 ChannelNumber = whal_ParamsGetRadioChannel(pHwMboxCmd->pWhalParams);
	
	/*
	 * Send the command 
	 */
	return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CMD_ENABLE_RX, (char *)&ChannelNumber, sizeof(UINT8)));
}

/****************************************************************************
 *                      whal_hwMboxCmd_StartScan ()
 ****************************************************************************
 * DESCRIPTION:	Send SCAN Command
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_StartScan (HwMboxCmd_T *pHwMboxCmd, ScanParameters_t* pScanParams ,void* ScanCommandResponseCB, TI_HANDLE CB_handle)
{
		return CmdQueue_CommandWithCb(pHwMboxCmd->hCmdMboxQueue,CMD_SCAN, (char *)pScanParams,  sizeof(ScanParameters_t),
                                 (void *)ScanCommandResponseCB, CB_handle, NULL /* void* CB_Buf */);
}

/****************************************************************************
 *                      whal_hwMboxCmd_StartSPSScan ()
 ****************************************************************************
 * DESCRIPTION:	Send SPS SCAN Command
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_StartSPSScan (HwMboxCmd_T *pHwMboxCmd, ScheduledScanParameters_t* pScanParams, void* ScanCommandResponseCB, TI_HANDLE CB_handle)
{
		return (CmdQueue_CommandWithCb(pHwMboxCmd->hCmdMboxQueue, CMD_SPS_SCAN, (char *)pScanParams, sizeof(ScheduledScanParameters_t),
			(void *)ScanCommandResponseCB, CB_handle, NULL ));
}

/****************************************************************************
 *                      whal_hwMboxCmd_StopScan ()
 ****************************************************************************
 * DESCRIPTION:	Construct the STOP_SCAN command fields and send it to the
 *				mailbox 
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_StopScan (HwMboxCmd_T *pHwMboxCmd, void *ScanCommandResponseCB, TI_HANDLE CB_handle)
{
	WLAN_REPORT_INFORMATION(pHwMboxCmd->hReport, HAL_HW_CTRL_MODULE_LOG,  
		("whal_hwMboxCmd_StopScan: -------------- \n"));

	if (NULL != ScanCommandResponseCB)
		return (CmdQueue_CommandWithCb(pHwMboxCmd->hCmdMboxQueue, CMD_STOP_SCAN, 0, 0, (void *)ScanCommandResponseCB,CB_handle,NULL));
	else
		return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CMD_STOP_SCAN, 0, 0));
}

/****************************************************************************
 *                      whal_hwMboxCmd_StopSPSScan ()
 ****************************************************************************
 * DESCRIPTION:	Construct the STOP_SPS_SCAN command fields and send it to the
 *				mailbox 
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_StopSPSScan (HwMboxCmd_T *pHwMboxCmd, void* ScanCommandResponseCB, TI_HANDLE CB_handle)
{
	WLAN_REPORT_INFORMATION(pHwMboxCmd->hReport, HAL_HW_CTRL_MODULE_LOG,  
		("whal_hwMboxCmd_StopSPSScan: -------------- \n"));
	if (NULL != ScanCommandResponseCB)
		return (CmdQueue_CommandWithCb(pHwMboxCmd->hCmdMboxQueue, CMD_STOP_SPS_SCAN, 0, 0, (void *)ScanCommandResponseCB, CB_handle, NULL ));
	else
		return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CMD_STOP_SPS_SCAN, 0, 0));

}

/****************************************************************************
 *                      whal_hwMboxCmd_GenCmd ()
 ****************************************************************************
 * DESCRIPTION:	Send any command
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_GenCmd(HwMboxCmd_T *pHwMboxCmd,short CmdId, char* pBuf, UINT32 Length)
{
	WLAN_REPORT_INFORMATION(pHwMboxCmd->hReport, HAL_HW_CTRL_MODULE_LOG,  
		("whal_hwMboxCmd_GenCmd: -------------- \n"));

	return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CmdId, pBuf, Length));
}

/****************************************************************************
 *                      whal_hwMboxCmd_NoiseHistogramCmd ()
 ****************************************************************************
 * DESCRIPTION:	Send NOISE_HISTOGRAM Command
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_NoiseHistogramCmd (HwMboxCmd_T *pHwMboxCmd, whalCtrl_noiseHistogram_t* pNoiseHistParams)
{
    NoiseHistRequest_t AcxCmd_NoiseHistogram;
	NoiseHistRequest_t *pCmd = &AcxCmd_NoiseHistogram;

    os_memoryZero(pHwMboxCmd->hOs, (void *)pCmd, sizeof(*pCmd));

    pCmd->mode = (uint16)pNoiseHistParams->cmd;
    pCmd->sampleIntervalUSec = pNoiseHistParams->sampleInterval;

    os_memoryCopy(pHwMboxCmd->hOs, (void *)&(pCmd->thresholds[0]), (void *)&(pNoiseHistParams->ranges[0]), MEASUREMENT_NOISE_HISTOGRAM_NUM_OF_RANGES);

    /* Send the command */
    return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CMD_NOISE_HIST, (char *)pCmd, sizeof(*pCmd)));
}

/****************************************************************************
 *                      whal_hwMboxCmd_PowerMgmtConfiguration()
 ****************************************************************************
 * DESCRIPTION:	send Command for Power Management configuration 
 *              to the mailbox
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_PowerMgmtConfiguration (HwMboxCmd_T *pHwMboxCmd,whalCtrl_powerSaveParams_t* powerSaveParams)
/*whalCtrl_powerMgmtConfig_t* pPowerMgmtParams)*/
{
	PSModeParameters_t	 Cmd_PowerMgmtCnf;
	PSModeParameters_t *	pCmd = &Cmd_PowerMgmtCnf;

	os_memoryZero(pHwMboxCmd->hOs, (void *)pCmd, sizeof(*pCmd));

	if(powerSaveParams->ps802_11Enable == TRUE)
		pCmd->mode = 1;

	else
		pCmd->mode = 0;
	
	pCmd->hangOverPeriod 			= powerSaveParams->hangOverPeriod;
    	pCmd->needToSendNullData 		= powerSaveParams->needToSendNullData;
	pCmd->rateToTransmitNullData 		= ENDIAN_HANDLE_WORD(powerSaveParams->NullPktRateModulation);
	pCmd->numberOfRetries 			= powerSaveParams->numNullPktRetries;


	/*
	 * Send the command 
	 */
	return (CmdQueue_CommandWithCb(pHwMboxCmd->hCmdMboxQueue, CMD_SET_PS_MODE, (char *)pCmd, sizeof(*pCmd),(void *)powerSaveParams->powerSavecmdResponseCB,powerSaveParams->powerSaveCBObject,NULL));
}


/****************************************************************************
 *                      whal_hwMboxCmd_SwitchChannelCmd ()
 ****************************************************************************
 * DESCRIPTION:	Send CMD_SWITCH_CHANNEL Command
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_SwitchChannelCmd (HwMboxCmd_T *pHwMboxCmd, whalCtrl_switchChannelCmd_t *pSwitchChannelCmd)
{
    ChannelSwitchParameters_t AcxCmd_SwitchChannel;
	ChannelSwitchParameters_t *pCmd = &AcxCmd_SwitchChannel;

    os_memoryZero(pHwMboxCmd->hOs, (void *)pCmd, sizeof(*pCmd));

    pCmd->channel = pSwitchChannelCmd->channelNumber;
    pCmd->switchTime = pSwitchChannelCmd->switchTime;
	pCmd->txSuspend = pSwitchChannelCmd->txFlag;
	pCmd->flush = pSwitchChannelCmd->flush;

    /* Send the command */
    return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CMD_CHANNEL_SWITCH, (char *)pCmd, sizeof(*pCmd)));
}


/****************************************************************************
 *                      whal_hwMboxCmd_SwitchChannelCancelCmd ()
 ****************************************************************************
 * DESCRIPTION:	Send CMD_SWITCH_CHANNEL_CANCEL Command
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_SwitchChannelCancelCmd (HwMboxCmd_T *pHwMboxCmd)
{

    /* Send the command */
    return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CMD_STOP_CHANNEL_SWICTH, 0, 0));
}

/****************************************************************************
 *                      whal_hwMboxCmd_FwDisconnect()
 ****************************************************************************
 * DESCRIPTION:	Construct the Disconnect command fileds and send it to the mailbox
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_FwDisconnect(HwMboxCmd_T *pHwMboxCmd, uint32 ConfigOptions, uint32 FilterOptions)
{
    DisconnectParameters_t AcxCmd_Disconnect;
	
	AcxCmd_Disconnect.rxFilter.ConfigOptions = ConfigOptions;
	AcxCmd_Disconnect.rxFilter.FilterOptions = FilterOptions;
	/*
	 * Send the command 
	 */
	return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CMD_DISCONNECT, (char *)&AcxCmd_Disconnect, sizeof(AcxCmd_Disconnect)));

} /* whal_hwMboxCmd_FWDisconnect()*/





/****************************************************************************
 *                      whal_hwMboxCmd_measurementParams()
 ****************************************************************************
 * DESCRIPTION:	send Command for measurement configuration 
 *              to the mailbox
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_measurement (HwMboxCmd_T *pHwMboxCmd, whalCtrl_MeasurementParameters_t* pMeasurementParams,
								void* MeasureCommandResponseCB, TI_HANDLE CB_handle)
{
	MeasurementParameters_t Cmd_MeasurementParam;
	MeasurementParameters_t *pCmd = &Cmd_MeasurementParam;

	os_memoryZero(pHwMboxCmd->hOs, (void *)pCmd, sizeof(*pCmd));
	
	pCmd->band =					pMeasurementParams->band;
	pCmd->channel =					pMeasurementParams->channel;
	pCmd->duration =				ENDIAN_HANDLE_LONG(pMeasurementParams->duration);
	pCmd->rxFilter.ConfigOptions =	ENDIAN_HANDLE_LONG(pMeasurementParams->ConfigOptions);
	pCmd->rxFilter.FilterOptions =	ENDIAN_HANDLE_LONG(pMeasurementParams->FilterOptions);
	/*
	 * Send the command 
	 */
	return (CmdQueue_CommandWithCb(pHwMboxCmd->hCmdMboxQueue, CMD_MEASUREMENT, (char *)pCmd, sizeof(*pCmd),
										(void *)MeasureCommandResponseCB, CB_handle, NULL));
}

/****************************************************************************
 *                      whal_hwMboxCmd_measurementStop()
 ****************************************************************************
 * DESCRIPTION:	send Command for stoping measurement  
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_measurementStop (HwMboxCmd_T *pHwMboxCmd, void* MeasureCommandResponseCB, TI_HANDLE CB_handle)
{
	/*
	 * Send the command 
	 */
    return (CmdQueue_CommandWithCb(pHwMboxCmd->hCmdMboxQueue, CMD_STOP_MEASUREMENT, 0, 0,
										(void *)MeasureCommandResponseCB, CB_handle, NULL));
}
/****************************************************************************
 *                      whal_hwMboxCmd_ApDiscovery()
 ****************************************************************************
 * DESCRIPTION:	send Command for AP Discovery 
 *              to the mailbox
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_ApDiscovery (HwMboxCmd_T *pHwMboxCmd, whalCtrl_ApDiscoveryParameters_t* pApDiscoveryParams)
{
	ApDiscoveryParameters_t Cmd_ApDiscovery;
	ApDiscoveryParameters_t *pCmd = &Cmd_ApDiscovery;

	os_memoryZero(pHwMboxCmd->hOs, (void *)pCmd, sizeof(*pCmd));
	
	pCmd->txPowerAttenuation = pApDiscoveryParams->txPowerDbm;
	pCmd->numOfProbRqst = pApDiscoveryParams->numOfProbRqst;
	pCmd->scanDuration 	=  ENDIAN_HANDLE_LONG(pApDiscoveryParams->scanDuration);
	pCmd->scanOptions 	=  ENDIAN_HANDLE_WORD(pApDiscoveryParams->scanOptions);
	pCmd->txdRateSet	=  ENDIAN_HANDLE_WORD(pApDiscoveryParams->txdRateSet);
	pCmd->rxFilter.ConfigOptions =	ENDIAN_HANDLE_LONG(pApDiscoveryParams->ConfigOptions);
	pCmd->rxFilter.FilterOptions =	ENDIAN_HANDLE_LONG(pApDiscoveryParams->FilterOptions);
	/*
	 * Send the command 
	 */
	return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CMD_AP_DISCOVERY, (char *)pCmd, sizeof(*pCmd)));
}
/****************************************************************************
 *                      whal_hwMboxCmd_ApDiscoveryStop()
 ****************************************************************************
 * DESCRIPTION:	send Command for stoping AP Discovery
 * 
 * INPUTS: None	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_hwMboxCmd_ApDiscoveryStop (HwMboxCmd_T *pHwMboxCmd)
{
	/*
	 * Send the command 
	 */
    return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CMD_STOP_AP_DISCOVERY, 0, 0));

}


/****************************************************************************
 *                      whal_hwMboxCmd_HealthCheck()
 ****************************************************************************
 * DESCRIPTION:	
 * 
 * INPUTS: 	
 * 
 * OUTPUT:	
 * 
 * RETURNS:
 ****************************************************************************/
int whal_hwMboxCmd_HealthCheck(HwMboxCmd_T *pHwMboxCmd)
{
	/*
	 * no parameters 
	 */
	
	/*
	 * Send the command 
	 */
	return (CmdQueue_Command(pHwMboxCmd->hCmdMboxQueue, CMD_HEALTH_CHECK, NULL, 0));
}

