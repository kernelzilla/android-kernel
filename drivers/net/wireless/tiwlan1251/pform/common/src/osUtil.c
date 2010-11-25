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

#ifdef _WINDOWS    /*nick*/
#elif defined(__ARMCC__) 
#include "string.h"
#endif

#include "osAdapter.h"
#include "srcApi.h"
#include "tiwlnif.h"
#include "osDot11.h"
#include "osUtil.h"
#include "paramOut.h"
#include "wspVer.h"
#include "osClsfr.h"
#include "whalHwMboxCmdBit.h"

static TI_STATUS
UtilRegulatoryDomain_setCountryIE(
								 PTIWLN_ADAPTER_T pAdapter,
								 externalParam_e ParamType,
								 PUCHAR pData,
								 ULONG Length
								 );

#ifdef TIWLN_WINCE30

gprintf(const char *format ,... )
{
#if 1
#ifdef DEBUG_PB
	wchar_t Buf[500];
#endif

	FILE *Fpn;  

	Fpn = fopen("TILog.txt","a"); 

	if (Fpn)
	{
		char Msg[500];
		va_list ap;
		va_start(ap,format);
		_vsnprintf(Msg,500,format,ap);
		fprintf(Fpn,"%s", Msg);
#ifdef DEBUG_PB
		mbstowcs(Buf,Msg,strlen(Msg)+1);    
		DEBUGMSG(1,(Buf));
#endif

		fclose(Fpn);

	}
#endif
}

#endif


/*-----------------------------------------------------------------------------
Routine Name: UtilSetParam
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
TI_STATUS
UtilSetParam(
			PTIWLN_ADAPTER_T pAdapter,
			externalParam_e ParamType,
			PUCHAR pData,
			ULONG Length
			)
{
	paramInfo_t Param;
	TI_STATUS Status;

	Param.paramType = ParamType;
	if (Length > sizeof(Param.content))
	{
		PRINTF(DBG_NDIS_OIDS_VERY_LOUD, (" UtilSetParam: Buffer for parameter 0x%X is bigger(%d) then Param size(%d)\n", ParamType, (int)Length, sizeof(Param.content)));
		Param.paramLength = sizeof(Param.content);
		NdisMoveMemory(&Param.content, pData, sizeof(Param.content));
	} else
	{
		Param.paramLength = Length;
		NdisMoveMemory(&Param.content, pData, Length);
	}

	Status = configMgr_setParam(pAdapter->CoreHalCtx, &Param);

	return Status;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilGetParam
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
TI_STATUS
UtilGetParam(
			PTIWLN_ADAPTER_T pAdapter,
			externalParam_e ParamType,
			PUCHAR pData,
			ULONG Length
			)
{
	paramInfo_t Param;
	TI_STATUS Status;

	Param.paramType = ParamType;
	Param.paramLength = Length;

	Status = configMgr_getParam(pAdapter->CoreHalCtx, &Param);

	if (Status == NOK)
	{
		PRINTF(DBG_NDIS_OIDS_VERY_LOUD, (" UtilGetParam: ERROR on return from get param, status=%d, param=%d\n",
										 Status, ParamType));
	} else if ( Status != NOK )
	{
		PRINTF(DBG_NDIS_OIDS_LOUD, (" UtilGetParam: WARNING on return from get param, status=%d, param=%d\n",
									Status, ParamType));
	}

	if (Length > sizeof(Param.content))
	{
		PRINTF(DBG_NDIS_OIDS_VERY_LOUD, (" UtilGetParam: Buffer for parameter 0x%X is bigger then Param size\n", ParamType));
		NdisMoveMemory(pData, &Param.content, sizeof(Param.content));
	} else
	{
		NdisMoveMemory(pData, &Param.content, Length);
	}

	return Status;
}

/*-----------------------------------------------------------------------------
Routine Name: UtilSetGetParam
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
TI_STATUS
UtilSetGetParam(
			   PTIWLN_ADAPTER_T pAdapter,
			   externalParam_e ParamType,
			   PUCHAR pData,
			   ULONG Length
			   )
{
	paramInfo_t Param;
	TI_STATUS Status;

	Param.paramType = ParamType;
	if (Length > sizeof(Param.content))
	{
		PRINTF(DBG_NDIS_OIDS_VERY_LOUD, (" UtilSetParam: Buffer for parameter 0x%X is bigger(%d) then Param size(%d)\n", ParamType, (int)Length, sizeof(Param.content)));
		Param.paramLength = sizeof(Param.content);
		NdisMoveMemory(&Param.content, pData, sizeof(Param.content));
	} else
	{
		Param.paramLength = Length;
		NdisMoveMemory(&Param.content, pData, Length);
	}

	Status = configMgr_setParam(pAdapter->CoreHalCtx, &Param);

	if (Length > sizeof(Param.content))
	{
		PRINTF(DBG_NDIS_OIDS_VERY_LOUD, (" UtilGetParam: Buffer for parameter 0x%X is bigger then Param size\n", ParamType));
		NdisMoveMemory(pData, &Param.content, sizeof(Param.content));
	} else
	{
		NdisMoveMemory(pData, &Param.content, Length);
	}

	return Status;
}

/*-----------------------------------------------------------------------------
Routine Name: UtilGetTxPowerValue
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilGetTxPowerValue(
				   PTIWLN_ADAPTER_T pAdapter,
				   externalParam_e ParamType,
				   PUCHAR pData,
				   ULONG Length
				   )
{
	paramInfo_t Param;
	TI_STATUS Status;

	Param.paramType = ParamType;
	Param.paramLength = Length;
	Status = configMgr_getParam(pAdapter->CoreHalCtx, &Param);

	*(PULONG)pData = (ULONG)Param.content.regulatoryDomainParam.txPower;

	return Status;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilSetTxPowerDbm
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilSetTxPowerDbm(PTIWLN_ADAPTER_T pAdapter,
						  PUCHAR pData,
						  ULONG Length)
{
	ULONG retValue;

	retValue = UtilSetParam(pAdapter, REGULATORY_DOMAIN_CURRENT_TX_POWER_IN_DBM_PARAM, pData, sizeof(UINT8));

	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilGetTxPowerLevel
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilGetTxPowerLevel(PTIWLN_ADAPTER_T pAdapter,
						  PUCHAR pData,
						  PULONG Length)
{
	ULONG retValue;

	retValue = UtilGetParam(pAdapter, REGULATORY_DOMAIN_TX_POWER_LEVEL_TABLE_PARAM, pData, sizeof(TIWLAN_POWER_LEVEL_TABLE));
	*Length=sizeof(TIWLAN_POWER_LEVEL_TABLE);

	return retValue;
}



/*-----------------------------------------------------------------------------
Routine Name:	UtilEnableDisableRxDataFilters
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilEnableDisableRxDataFilters(PTIWLN_ADAPTER_T pAdapter,
                                     PUCHAR pData,
                                     ULONG Length)
{
    ULONG retValue;

    retValue = UtilSetParam(pAdapter, RX_DATA_ENABLE_DISABLE_RX_DATA_FILTERS, pData, Length);

    return retValue;
}



/*-----------------------------------------------------------------------------
Routine Name:	UtilGetRxDataFiltersStatisticsCB

Routine Description: This is the CB triggered when Rx Data Filter statistics 
					 are returned by the FW.
Arguments:

Return Value:
-----------------------------------------------------------------------------*/
static VOID UtilGetRxDataFiltersStatisticsCB(TI_HANDLE hAdapter, TI_STATUS status, PUINT8 pReadBuff)
{
	PTIWLN_ADAPTER_T pAdapter = (PTIWLN_ADAPTER_T) hAdapter;
    ACXDataFilteringStatistics_t * pStatistics = (ACXDataFilteringStatistics_t *) pReadBuff;
    TIWLAN_DATA_FILTER_STATISTICS * pResult = (TIWLAN_DATA_FILTER_STATISTICS *) &(pAdapter->pIoBuffer[0]);
    int i;

    pResult->UnmatchedPacketsCount = pStatistics->unmatchedPacketsCount;

    for (i = 0; i < MAX_DATA_FILTERS; ++i)
    {
        pResult->MatchedPacketsCount[i] = pStatistics->matchedPacketsCount[i];
    }

    *(pAdapter->pIoCompleteBuffSize) = sizeof(TIWLAN_DATA_FILTER_STATISTICS);

    /* indicate that the buffer is ready */
	os_IoctlComplete(pAdapter, status);
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilEnableDisableRxDataFilters
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilGetRxDataFiltersStatistics(PTIWLN_ADAPTER_T pAdapter,
                                     PUCHAR pData,
                                     PULONG Length)
{
    paramInfo_t Param;

    memset(&(pAdapter->IoCompleteBuff[0]) , 0xFF , MAX_IO_BUFFER_COMPLETE_SIZE);

    pAdapter->pIoBuffer = pData;
    pAdapter->pIoCompleteBuffSize = Length;

    Param.paramType = RX_DATA_GET_RX_DATA_FILTERS_STATISTICS;
    Param.paramLength = sizeof(TIWLAN_DATA_FILTER_STATISTICS);
    Param.content.interogateCmdCBParams.CB_handle = (TI_HANDLE) pAdapter;
    Param.content.interogateCmdCBParams.CB_Func = (PVOID) UtilGetRxDataFiltersStatisticsCB;
    Param.content.interogateCmdCBParams.CB_buf = &(pAdapter->IoCompleteBuff[0]) ;

    return configMgr_getParam(pAdapter->CoreHalCtx, &Param);
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilGetPowerConsumptionStatisticsCB

Routine Description: This is the CB triggered when Power consumption statistics 
					 are returned by the FW.
Arguments:

Return Value:
-----------------------------------------------------------------------------*/
static VOID UtilGetPowerConsumptionStatisticsCB(TI_HANDLE hAdapter, TI_STATUS status, PUINT8 pReadBuff)
{
	PTIWLN_ADAPTER_T pAdapter = (PTIWLN_ADAPTER_T) hAdapter;
    ACXPowerConsumptionTimeStat_t * pStatistics = (ACXPowerConsumptionTimeStat_t *) pReadBuff;
    PowerConsumptionTimeStat_t * pResult = (PowerConsumptionTimeStat_t *) &(pAdapter->pIoBuffer[0]);

    pResult->activeTimeCnt_Hi = pStatistics->activeTimeCnt_Hi;
    pResult->activeTimeCnt_Low = pStatistics->activeTimeCnt_Low;
    pResult->elpTimeCnt_Hi = pStatistics->elpTimeCnt_Hi;
    pResult->elpTimeCnt_Low = pStatistics->elpTimeCnt_Low;
    pResult->powerDownTimeCnt_Hi = pStatistics->powerDownTimeCnt_Hi;
    pResult->powerDownTimeCnt_Low = pStatistics->powerDownTimeCnt_Low;
    
    *(pAdapter->pIoCompleteBuffSize) = sizeof(PowerConsumptionTimeStat_t);

    /* indicate that the buffer is ready */
	os_IoctlComplete(pAdapter, status);
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilGetPowerConsumptionStatistics
Routine Description: Request the power consumption statistics from the FW
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilGetPowerConsumptionStatistics(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length)
{
    paramInfo_t Param;

    memset(&(pAdapter->IoCompleteBuff[0]) , 0xFF , MAX_IO_BUFFER_COMPLETE_SIZE);

    pAdapter->pIoBuffer = pData;
    pAdapter->pIoCompleteBuffSize = Length;

    Param.paramType = HAL_CTRL_POWER_CONSUMPTION;
    Param.paramLength = sizeof(PowerConsumptionTimeStat_t);
    Param.content.interogateCmdCBParams.CB_handle = (TI_HANDLE) pAdapter;
    Param.content.interogateCmdCBParams.CB_Func = (PVOID) UtilGetPowerConsumptionStatisticsCB;
    Param.content.interogateCmdCBParams.CB_buf = &(pAdapter->IoCompleteBuff[0]);

    return configMgr_getParam(pAdapter->CoreHalCtx, &Param);
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilAddRxDataFilter
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilAddRxDataFilter(PTIWLN_ADAPTER_T pAdapter,
                          PUCHAR pData,
                          ULONG Length)
{
    ULONG retValue;

    retValue = UtilSetParam(pAdapter, RX_DATA_ADD_RX_DATA_FILTER, pData, Length);

    return retValue;
}



/*-----------------------------------------------------------------------------
Routine Name:	UtilRemoveRxDataFilter
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilRemoveRxDataFilter(PTIWLN_ADAPTER_T pAdapter,
                             PUCHAR pData,
                             ULONG Length)
{
    ULONG retValue;

    retValue = UtilSetParam(pAdapter, RX_DATA_REMOVE_RX_DATA_FILTER, pData, Length);

    return retValue;
}



/*-----------------------------------------------------------------------------
Routine Name:	UtilGetCurrentRssiLevel
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilGetCurrentRssiLevel(PTIWLN_ADAPTER_T pAdapter,
							  PUCHAR pData,
							  PULONG Length)
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, SITE_MGR_CURRENT_SIGNAL_PARAM, pData, sizeof(INT32));
	*Length = sizeof(INT32);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	RssiUtilIoctlCompleteCB

Routine Description: This is the CB triggered when  Rssi/Snr have been 
					returned by FW - return RSSI only to user
Arguments:

Return Value:
-----------------------------------------------------------------------------*/
VOID RssiUtilIoctlCompleteCB(TI_HANDLE hAdapter, TI_STATUS status, PUINT8 pReadBuff)
{
	PTIWLN_ADAPTER_T pAdapter = (PTIWLN_ADAPTER_T)hAdapter;
	TIWLN_RADIO_RX_QUALITY tmpRadioRxQuality;
	paramInfo_t Param;
	ACXRoamingStatisticsTable_t * radioResults = (ACXRoamingStatisticsTable_t *) pReadBuff;

	tmpRadioRxQuality.Rssi = radioResults->rssi;
	tmpRadioRxQuality.Snr = (INT32) radioResults->snr;

	/* here we update the site manager about these new values */
	Param.paramType = SITE_MGR_CURRENT_SIGNAL_PARAM;
	Param.paramLength = sizeof(INT32);
	Param.content.siteMgrCurrentRssi = tmpRadioRxQuality.Rssi;
	configMgr_setParam(pAdapter->CoreHalCtx, &Param);


	*(pAdapter->pIoCompleteBuffSize) = sizeof(INT32);
	os_memoryCopy(hAdapter, (PVOID) &(pAdapter->pIoBuffer[0]), (PVOID) &(tmpRadioRxQuality.Rssi), sizeof(INT32));

	/* Call back the Completion that will indicate to the user that the buffer is ready to be read */
	os_IoctlComplete(pAdapter, status);
}

/*-----------------------------------------------------------------------------
Routine Name:	SnrUtilIoctlCompleteCB

Routine Description: This is the CB triggered when  Rssi/Snr have been 
					returned by FW - return SNR only to user
Arguments:

Return Value:
-----------------------------------------------------------------------------*/
VOID SnrUtilIoctlCompleteCB(TI_HANDLE hAdapter, TI_STATUS status, PUINT8 pReadBuff)
{
	PTIWLN_ADAPTER_T pAdapter = (PTIWLN_ADAPTER_T) hAdapter;
	TIWLN_RADIO_RX_QUALITY tmpRadioRxQuality;
	ACXRoamingStatisticsTable_t * radioResults = (ACXRoamingStatisticsTable_t *) pReadBuff;

    tmpRadioRxQuality.Rssi = radioResults->rssi;
	/* The SNR returned by FW is not true. We have to divide it by 2 and turns it to a signed */
	tmpRadioRxQuality.Snr = (INT32) radioResults->snr;

	*(pAdapter->pIoCompleteBuffSize) = sizeof(INT32);

	os_memoryCopy(hAdapter, (PVOID) &(pAdapter->pIoBuffer[0]), (PVOID) &(tmpRadioRxQuality.Snr), sizeof(INT32));

	os_IoctlComplete(pAdapter, status);
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilGetAsyncCurrentRssiLevel
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilGetAsyncCurrentRssiLevel(PTIWLN_ADAPTER_T pAdapter,
								   PUCHAR pData,
								   PULONG Length)
{
	paramInfo_t Param;


	TI_STATUS Status;

	memset(&(pAdapter->IoCompleteBuff[0]) , 0xFF , MAX_IO_BUFFER_COMPLETE_SIZE );

	/* To implement the Async IOCTL store the user buffer pointer to be filled at
	the Command Completion calback */
	pAdapter->pIoBuffer =  pData;
	pAdapter->pIoCompleteBuffSize =  Length ;

	/* Fill the IOCTL struct to the Command Mailbox by giving a stack parameter */
	Param.paramType = HAL_CTRL_RSSI_LEVEL_PARAM;
	Param.paramLength = sizeof(INT32);
	Param.content.interogateCmdCBParams.CB_handle  =  (TI_HANDLE)pAdapter;
	Param.content.interogateCmdCBParams.CB_Func    =  (PVOID)RssiUtilIoctlCompleteCB;
	Param.content.interogateCmdCBParams.CB_buf     =  &(pAdapter->IoCompleteBuff[0]) ;

	/* This Get Param will in fact get till the HAL and will interrogate the FW */
	Status = configMgr_getParam(pAdapter->CoreHalCtx, &Param);

	return Status;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilGetAsyncCurrentRssiLevel
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilGetAsyncCurrentSnrRatio(PTIWLN_ADAPTER_T pAdapter,
								  PUCHAR pData,
								  PULONG Length)
{
	paramInfo_t Param;


	TI_STATUS Status;

	memset(&(pAdapter->IoCompleteBuff[0]) , 0xFF , MAX_IO_BUFFER_COMPLETE_SIZE );

	pAdapter->pIoBuffer =  pData;
	pAdapter->pIoCompleteBuffSize =  Length ;

	Param.paramType = HAL_CTRL_SNR_RATIO_PARAM;
	Param.paramLength = sizeof(INT32);
	Param.content.interogateCmdCBParams.CB_handle  =  (TI_HANDLE)pAdapter;
	Param.content.interogateCmdCBParams.CB_Func    =  (PVOID)SnrUtilIoctlCompleteCB;
	Param.content.interogateCmdCBParams.CB_buf     =  &(pAdapter->IoCompleteBuff[0]) ;

	Status = configMgr_getParam(pAdapter->CoreHalCtx, &Param);

	return Status;


}


/*-----------------------------------------------------------------------------
Routine Name: UtilGetAPTxPowerLevel
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilGetAPTxPowerLevel(
					 PTIWLN_ADAPTER_T pAdapter,
					 externalParam_e ParamType,
					 PUCHAR pData,
					 ULONG Length
					 )
{
	paramInfo_t Param;
	TI_STATUS Status;

	Param.paramType = ParamType;
	Param.paramLength = Length;
	Status = configMgr_getParam(pAdapter->CoreHalCtx, &Param);

	*(PULONG)pData = (ULONG)Param.content.APTxPower;

	return Status;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilGetCountryCode
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilGetCountryCode(
				  PTIWLN_ADAPTER_T pAdapter,
				  externalParam_e ParamType,
				  PUCHAR pData,
				  ULONG Length
				  )
{
	paramInfo_t Param;
	TI_STATUS Status;

	Param.paramType = ParamType;
	Param.paramLength = Length;
	Status = configMgr_getParam(pAdapter->CoreHalCtx, &Param);

	NdisMoveMemory(pData, Param.content.pCountryString, Length);

	return Status;
}



/*-----------------------------------------------------------------------------
Routine Name: UtilGetRegDomainBand
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilGetRegDomainBand(
					PTIWLN_ADAPTER_T pAdapter,
					externalParam_e ParamType,
					PUCHAR pData,
					ULONG Length
					)
{
	paramInfo_t Param;
	TI_STATUS Status;

	Param.paramType = ParamType;
	Param.paramLength = Length;

	Status = configMgr_getParam(pAdapter->CoreHalCtx, &Param);

	*(PULONG)pData = (ULONG) *(PUCHAR)&Param.content;

	return Status;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilGetPacketBursting
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilGetPacketBursting(
					 PTIWLN_ADAPTER_T pAdapter,
					 externalParam_e ParamType,
					 PUCHAR pData,
					 ULONG Length
					 )
{
	paramInfo_t Param;
	TI_STATUS Status;

	Param.paramType = ParamType;
	Param.paramLength = Length;

	Status = configMgr_getParam(pAdapter->CoreHalCtx, &Param);

	*(PULONG)pData = (ULONG)Param.content.qosPacketBurstEnb;
	return Status;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilGetMixedMode
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilGetMixedMode(
				PTIWLN_ADAPTER_T pAdapter,
				externalParam_e ParamType,
				PUCHAR pData,
				PULONG Length
				)
{
	paramInfo_t Param;
	TI_STATUS Status;

	Param.paramType = ParamType;

	Status = configMgr_getParam(pAdapter->CoreHalCtx, &Param);

	*(PULONG)pData = (ULONG)Param.content.rsnMixedMode;
	return Status;
}

/*-----------------------------------------------------------------------------
Routine Name: UtilGetDefaultKeyId
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilGetDefaultKeyId(
				   PTIWLN_ADAPTER_T pAdapter,
				   externalParam_e ParamType,
				   PUCHAR pData,
				   PULONG Length
				   )
{
	paramInfo_t Param;
	TI_STATUS Status;

	Param.paramType = ParamType;

	Status = configMgr_getParam(pAdapter->CoreHalCtx, &Param);

	*(PULONG)pData = (ULONG)Param.content.rsnDefaultKeyID;
	return Status;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilSetTrafficIntensityThresholds
Routine Description: Sets the traffic intensity thresholds
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilSetTrafficIntensityThresholds(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter,CTRL_DATA_TRAFFIC_INTENSITY_THRESHOLD , pData, Length);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilGetTrafficIntensityThresholds
Routine Description: retrieves the traffic intensity thresholds
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilGetTrafficIntensityThresholds(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length)
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, CTRL_DATA_TRAFFIC_INTENSITY_THRESHOLD, pData, (*Length));
	*Length = sizeof(OS_802_11_TRAFFIC_INTENSITY_THRESHOLD_PARAMS);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilToggleTrafficIntensityEvents
Routine Description: Toggles ON/OFF traffic intensity events
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilToggleTrafficIntensityEvents(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter,CTRL_DATA_TOGGLE_TRAFFIC_INTENSITY_EVENTS , pData, Length);
	return retValue;
}




/*-----------------------------------------------------------------------------
Routine Name:

	UtilSetBSSID
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilSetBSSID(
			PTIWLN_ADAPTER_T pAdapter,
			PUCHAR pData,
			ULONG Length
			)
{
	return UtilSetParam(pAdapter, SITE_MGR_DESIRED_BSSID_PARAM, pData, ETH_ADDR_SIZE);
}

/*-----------------------------------------------------------------------------
Routine Name: UtilGetBSSID
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilGetBSSID(
			PTIWLN_ADAPTER_T pAdapter,
			PUCHAR pData,
			PULONG Length
			)
{
	TI_STATUS res;

	if (!Length)
		return NOK;

	res = UtilGetParam(pAdapter, CTRL_DATA_CURRENT_BSSID_PARAM, pData, ETH_ADDR_SIZE);

	*Length = ETH_ADDR_SIZE;

	return res;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilGetSSID
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilGetSSID(
		   PTIWLN_ADAPTER_T pAdapter,
		   PUCHAR pData,
		   PULONG Length
		   )
{
	ULONG size;
	ssid_t ssid;
	OS_802_11_SSID* RetSsid;

	if (*Length<sizeof(OS_802_11_SSID))
		return NOK;

	size = sizeof(ssid_t);
	UtilGetParam(pAdapter, SITE_MGR_CURRENT_SSID_PARAM, (PUCHAR)&ssid, size);

	RetSsid = (OS_802_11_SSID*) pData;

	RetSsid->SsidLength = ssid.len;
	NdisMoveMemory((void *)RetSsid->Ssid, (void *)ssid.ssidString, ssid.len);

	*Length = sizeof(OS_802_11_SSID);

	return OK;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilGetDesiredSSID
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilGetDesiredSSID(
				  PTIWLN_ADAPTER_T pAdapter,
				  PUCHAR pData,
				  PULONG Length
				  )
{
	ULONG size,retValue;
	ssid_t ssid;
	OS_802_11_SSID* RetSsid;

	if (!(*Length))
	{
		*Length = sizeof(OS_802_11_SSID);
		return NOK;
	}

	size = sizeof(ssid_t);
	retValue = UtilGetParam(pAdapter, SITE_MGR_DESIRED_SSID_PARAM, (PUCHAR)&ssid, size);

	RetSsid = (OS_802_11_SSID*) pData;

	RetSsid->SsidLength = ssid.len;
	NdisMoveMemory((void *)RetSsid->Ssid, (void *)ssid.ssidString, ssid.len);

	*Length = sizeof(OS_802_11_SSID);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilSetSSID
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilSetSSID(
		   PTIWLN_ADAPTER_T pAdapter,
		   PUCHAR pData,
		   ULONG Length
		   )
{
	OS_802_11_SSID* UtilSsid;
	ssid_t ssid;

	UtilSsid = (OS_802_11_SSID*) pData;

	if (UtilSsid->SsidLength<=MAX_SSID_LEN)
	{
		ssid.len = (UINT8)UtilSsid->SsidLength;

		NdisMoveMemory((void *)ssid.ssidString, (void *)UtilSsid->Ssid, ssid.len);

		/* The driver should support setting the SSID to NULL string */
		if (ssid.len == 0)
			ssid.ssidString[0] = '\0';

#ifdef TI_DBG
		{    
			UCHAR   tempName[33];

			NdisMoveMemory(tempName, (void *)UtilSsid->Ssid, ssid.len);
			tempName[ssid.len] ='\0';

			PRINTF(DBG_NDIS_OIDS_LOUD, ("  SET SSID: Len=%d %s\n", ssid.len, tempName));
		}
#endif

		UtilSetParam(pAdapter, SITE_MGR_DESIRED_SSID_PARAM, (PUCHAR)&ssid, sizeof(ssid_t));
	}

	return OK;

}


/*-----------------------------------------------------------------------------
Routine Name: UtilNetworkTypesSupported
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilNetworkTypesSupported(
						 PTIWLN_ADAPTER_T pAdapter,
						 PUCHAR pData,
						 PULONG Length
						 )
{
	OS_802_11_NETWORK_TYPE_LIST * pList;
	ULONG mode, size;

	UtilGetParam(pAdapter, SITE_MGR_DESIRED_DOT11_MODE_PARAM, 
							(PUCHAR)&mode, sizeof(ULONG));

	if (!(*Length))
	{

		/**/
		/* Return the maximum size*/
		/**/
		size = sizeof(OS_802_11_NETWORK_TYPE_LIST) +
			   3 * sizeof(OS_802_11_NETWORK_TYPE) -
			   sizeof(OS_802_11_NETWORK_TYPE);

		*Length = size;
		return NOK;

	}

	pList = (OS_802_11_NETWORK_TYPE_LIST *) pData;

	switch (mode)
	{
	
	case 1:
		pList->NumberOfItems = 1;
		pList->NetworkType[0] = os802_11DS;
		break;

	case 2:
		pList->NumberOfItems = 1;
		pList->NetworkType[0] = os802_11OFDM5;
		break;

	case 3:
		pList->NumberOfItems = 2;
		pList->NetworkType[0] = os802_11DS;
		pList->NetworkType[1] = os802_11OFDM24;
		break;

	case 4:
		pList->NumberOfItems = 3;
		pList->NetworkType[0] = os802_11DS;
		pList->NetworkType[1] = os802_11OFDM24;
		pList->NetworkType[2] = os802_11OFDM5;
		break;

	default:
		pList->NumberOfItems = 1;
		pList->NetworkType[0] = os802_11DS;
		break;
	}

	size = sizeof(OS_802_11_NETWORK_TYPE_LIST) +
		   pList->NumberOfItems * sizeof(OS_802_11_NETWORK_TYPE) -
		   sizeof(OS_802_11_NETWORK_TYPE);

	*Length = size;
	return OK;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilNetworkTypeInUseGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilNetworkTypeInUseGet(
					   PTIWLN_ADAPTER_T pAdapter,
					   PUCHAR pData,
					   PULONG Length
					   )
{
	ULONG mode;
	if (!(*Length))
	{

		*Length = sizeof(OS_802_11_NETWORK_TYPE);
		return NOK;

	}

	UtilGetParam(pAdapter, SITE_MGR_DESIRED_DOT11_MODE_PARAM, 
				 (PUCHAR)&mode, sizeof(ULONG));

	switch (mode)
	{
	
	case 1:
		*((OS_802_11_NETWORK_TYPE *) pData) = os802_11DS;
		break;

	case 2:
		*((OS_802_11_NETWORK_TYPE *) pData) = os802_11OFDM5;
		break;

	case 3:
		*((OS_802_11_NETWORK_TYPE *) pData) = os802_11OFDM24;
		break;

	case 4:
		*((OS_802_11_NETWORK_TYPE *) pData) = os802_11Automode;
		break;

	default:
		*((OS_802_11_NETWORK_TYPE *) pData) = os802_11DS;
		break;

	}

	*Length = sizeof(OS_802_11_NETWORK_TYPE);
	return OK;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilNetworkTypeInUseSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilNetworkTypeInUseSet(
					   PTIWLN_ADAPTER_T pAdapter,
					   PUCHAR pData,
					   ULONG Length
					   )
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, SITE_MGR_DESIRED_DOT11_MODE_PARAM, pData, sizeof(ULONG));
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilSetPacketBursting
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilSetPacketBursting(
					 PTIWLN_ADAPTER_T pAdapter,
					 PUCHAR pData,
					 ULONG Length
					 )
{
	paramInfo_t Param;
	TI_STATUS Status;

	Param.paramType = QOS_PACKET_BURST_ENABLE;
	Param.content.qosPacketBurstEnb = *((UINT8*)pData);
	Status = configMgr_setParam(pAdapter->CoreHalCtx, &Param);
	return Status;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilSetMixedMode
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilSetMixedMode(
				PTIWLN_ADAPTER_T pAdapter,
				PUCHAR pData,
				ULONG Length
				)
{
	paramInfo_t Param;
	TI_STATUS Status;

	Param.paramType = RSN_MIXED_MODE;
	Param.content.rsnMixedMode = *((UINT32*)pData);
	Status = configMgr_setParam(pAdapter->CoreHalCtx, &Param);

	return Status;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilPowerModeSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilPowerModeSet(
				PTIWLN_ADAPTER_T pAdapter,
				PUCHAR pData,
				ULONG Length
				)
{
	ULONG retValue;
	PowerMgr_PowerMode_t PowerMgr_PowerMode;
	PowerMgr_PowerMode.PowerMode = (PowerMgr_PowerMode_e)*pData;
	PowerMgr_PowerMode.powerMngPriority = POWER_MANAGER_USER_PRIORITY;
	retValue = UtilSetParam(pAdapter, POWER_MGR_POWER_MODE,(PUCHAR)&PowerMgr_PowerMode, sizeof(PowerMgr_PowerMode));
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilPowerModeGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilPowerModeGet(
				PTIWLN_ADAPTER_T pAdapter,
				PUCHAR pData,
				PULONG Length
				)
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, POWER_MGR_POWER_MODE, pData, sizeof(PowerMgr_PowerMode_e));
	*Length = sizeof(PowerMgr_PowerMode_e);
	return retValue;
}



/*-----------------------------------------------------------------------------
Routine Name: UtilPowerLevelPSGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilPowerLevelPSGet(
				   PTIWLN_ADAPTER_T pAdapter,
				   PUCHAR pData,
				   PULONG Length
				   )
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, POWER_MGR_POWER_LEVEL_PS, pData, sizeof(powerAutho_PowerPolicy_e));
	*Length = sizeof(powerAutho_PowerPolicy_e);
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name: UtilPowerLevelPSSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilPowerLevelPSSet(
				   PTIWLN_ADAPTER_T pAdapter,
				   PUCHAR pData,
				   PULONG Length
				   )
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, POWER_MGR_POWER_LEVEL_PS, pData, sizeof(powerAutho_PowerPolicy_e));
	*Length = sizeof(powerAutho_PowerPolicy_e);
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name: UtilPowerLevelDefaultGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilPowerLevelDefaultGet(
						PTIWLN_ADAPTER_T pAdapter,
						PUCHAR pData,
						PULONG Length
						)
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, POWER_MGR_POWER_LEVEL_DEFAULT, pData, sizeof(powerAutho_PowerPolicy_e));
	*Length = sizeof(powerAutho_PowerPolicy_e);
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name: UtilPowerLevelDefaultSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilPowerLevelDefaultSet(
						PTIWLN_ADAPTER_T pAdapter,
						PUCHAR pData,
						PULONG Length
						)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, POWER_MGR_POWER_LEVEL_DEFAULT, pData, sizeof(powerAutho_PowerPolicy_e));
	*Length = sizeof(powerAutho_PowerPolicy_e);
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name: UtilPowerLevelDozeModeGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilPowerLevelDozeModeGet(
    PTIWLN_ADAPTER_T pAdapter,
    PUCHAR pData,
    PULONG Length
    )
{
   ULONG retValue;
   retValue = UtilGetParam(pAdapter, POWER_MGR_POWER_LEVEL_DOZE_MODE, pData, sizeof(PowerMgr_PowerMode_e));
   *Length = sizeof(PowerMgr_PowerMode_e);
   return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name: UtilPowerLevelDozeModeSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilPowerLevelDozeModeSet(
    PTIWLN_ADAPTER_T pAdapter,
    PUCHAR pData,
    PULONG Length
    )
{
   ULONG retValue;
   retValue = UtilSetParam(pAdapter, POWER_MGR_POWER_LEVEL_DOZE_MODE, pData, sizeof(PowerMgr_PowerMode_e));
   *Length = sizeof(PowerMgr_PowerMode_e);
   return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilBeaconFilterDesiredStateSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilBeaconFilterDesiredStateSet(
							   PTIWLN_ADAPTER_T pAdapter,
							   PUCHAR pData,
							   PULONG Length
							   )
{
	ULONG retValue;

	retValue = UtilSetParam(pAdapter, SITE_MGR_BEACON_FILTER_DESIRED_STATE_PARAM, pData, sizeof(UINT8));
	*Length = sizeof(UINT8);
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name: UtilBeaconFilterDesiredStateGet
Routine Description: gets the current beacon filter state
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilBeaconFilterDesiredStateGet(
							   PTIWLN_ADAPTER_T pAdapter,
							   PUCHAR pData,
							   PULONG Length
							   )
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, SITE_MGR_BEACON_FILTER_DESIRED_STATE_PARAM, pData, sizeof(UINT8));
	*Length = sizeof(UINT8);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilBssidListGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilBssidListGet(
				PTIWLN_ADAPTER_T pAdapter,
				PUCHAR pData,
				PULONG Length,
				BOOLEAN ExtBssid,
				BOOLEAN allVarIes
				)
{
	OS_802_11_BSSID_LIST_EX* pListEx;
	OS_802_11_BSSID_EX* pBssidEx;
	OS_802_11_BSSID* pBssid;
	OS_802_11_BSSID_LIST* pList;
	paramInfo_t Param;
	ULONG RetSize, i;
	TI_STATUS Status;
	PUCHAR pTempData=NULL;
	ULONG LocalLength = sizeof(OS_802_11_BSSID_LIST_EX)+NUM_OF_SITE_TABLE*MAX_SITES_BG_BAND*sizeof(OS_802_11_BSSID);


	/******  At the callback of RSSI update the RSSI in the Site TAble in the Site Manager *
	 Update Site Table in order to represent the RSSI of current AP correctly in the utility 
	param.paramType = SITE_MGR_CURRENT_SIGNAL_PARAM;
	param.content.siteMgrCurrentSignal.rssi = pCurrBSS->averageRssi;
	siteMgr_setParam(pCurrBSS->hSiteMgr, &param);
	***************************************/


	if (allVarIes)
	{
		Param.paramType = SITE_MGR_BSSID_FULL_LIST_PARAM;
	} else
	{
		Param.paramType = SITE_MGR_BSSID_LIST_PARAM;
	}

	if (ExtBssid)
	{
		*(PULONG)&Param.content = (ULONG)pData;
		Param.paramLength = *Length;
	} else
	{
		if (*Length)
		{
			pTempData = os_memoryAlloc(pAdapter, LocalLength);

			if (!pTempData)
			{
				*(PULONG)pData = LocalLength;
				*Length = 0;
				return NOK;
			}

			*(PULONG)&Param.content = (ULONG)pTempData;
			Param.paramLength = *Length;
		} else
		{
			*Length = LocalLength;
			return NOK;
		}
	}

	Status = configMgr_getParam(pAdapter->CoreHalCtx, &Param);

	if (!(*Length))
	{

		*Length = Param.paramLength * 4;
		return NOK;

	}

	if (Status != OK)
	{
		*(PULONG)pData = Param.paramLength;
		PRINT(DBG_IOCTL_LOUD, "...More buffer space needed\n");
		if (!ExtBssid)
			os_memoryFree(pAdapter, pTempData, LocalLength);

		*Length = 0;
		return NOK;

	}

	if (!ExtBssid)
	{

		pListEx = (OS_802_11_BSSID_LIST_EX*) pTempData;

		if (pListEx->NumberOfItems)
		{

			if ((sizeof(OS_802_11_BSSID_LIST) + 
				 sizeof(OS_802_11_BSSID)*pListEx->NumberOfItems - 
				 sizeof(OS_802_11_BSSID)) > *Length)
			{
				PRINT(DBG_IOCTL_LOUD, "Utility buffer is too small\n");
				os_memoryFree(pAdapter, pTempData, LocalLength);
				*Length = 0;
				return NOK;
			}
            if (pListEx->NumberOfItems > 
                ((0xFFFFFFFFUL - ((ULONG)sizeof(OS_802_11_BSSID_LIST) - 1)) / 
                 (ULONG)sizeof(OS_802_11_BSSID) + 1)) /* Dm: Security fix */
            {
                printk("TI: %s - Security Error\n", __FUNCTION__);
                PRINT(DBG_IOCTL_LOUD, "Number of AP is too big\n");
                os_memoryFree(pAdapter, pTempData, LocalLength);
                *Length = 0;
                return NOK;
            }

            pList = (OS_802_11_BSSID_LIST *)pData;
			pList->NumberOfItems = pListEx->NumberOfItems;

			*Length = RetSize = sizeof(OS_802_11_BSSID_LIST) + 
					  sizeof(OS_802_11_BSSID)*pList->NumberOfItems - 
					  sizeof(OS_802_11_BSSID);

			pBssidEx = pListEx->Bssid;

			for (i=0; i<pListEx->NumberOfItems; i++)
			{

				pBssid = (OS_802_11_BSSID*) pBssidEx;

				NdisMoveMemory(&pList->Bssid[i], pBssid,
							   sizeof(OS_802_11_BSSID));

				pList->Bssid[i].Length = sizeof(OS_802_11_BSSID);

				pBssidEx = (OS_802_11_BSSID_EX*) ((PUCHAR)pBssidEx + 
												  pBssidEx->Length);

			}

		}

		else
		{

			pList = (OS_802_11_BSSID_LIST*) pData;
			pList->NumberOfItems = 0;

			RetSize = sizeof(OS_802_11_BSSID_LIST);
			*Length = RetSize;

		}

		PRINT(DBG_IOCTL_LOUD, "...Copy done.\n");

		os_memoryFree(pAdapter, pTempData, LocalLength);

	}

	else
	{

		RetSize = Param.paramLength;
		*Length = RetSize;

	}

	return OK;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilStartAppScanSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilStartAppScanSet(
				   PTIWLN_ADAPTER_T pAdapter,
				   PUCHAR pData,
				   ULONG Length
				   )
{
	ULONG retValue;              
	/* scan concentrator will start an application scan */
	retValue = UtilSetParam(pAdapter, SCAN_CNCN_START_APP_SCAN, (PUCHAR)&pData, sizeof(PUCHAR));
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilStopAppScanSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilStopAppScanSet(
				  PTIWLN_ADAPTER_T pAdapter,
				  PUCHAR pData,
				  ULONG Length
				  )
{
	ULONG retValue;
	/* scan concentrator will stop the running application scan (if any) */
	retValue = UtilSetParam(pAdapter, SCAN_CNCN_STOP_APP_SCAN, NULL, 0);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilScanPolicyParamSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilScanPolicyParamSet(
					  PTIWLN_ADAPTER_T pAdapter,
					  PUCHAR pData,
					  ULONG Length
					  )
{
	ULONG retValue;

	applicationConfigBuffer_t applicationConfigBuffer;

	applicationConfigBuffer.buffer = pData;
	applicationConfigBuffer.bufferSize = (UINT16)Length;

	/* set the scan manager policy */
	retValue = UtilSetParam( pAdapter, SCAN_MNGR_SET_CONFIGURATION, (PUCHAR)&applicationConfigBuffer, sizeof(applicationConfigBuffer_t) );

	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilScanBssListGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilScanBssListGet(
				  PTIWLN_ADAPTER_T pAdapter,
				  PUCHAR pData,
				  PULONG Length
				  )
{
	paramInfo_t param;
	TI_STATUS status;

	param.paramType = SCAN_MNGR_BSS_LIST_GET;
	param.paramLength = sizeof(PUCHAR);

	status = configMgr_getParam(pAdapter->CoreHalCtx, &param);

	NdisMoveMemory( pData, param.content.pScanBssList, *Length );
	*Length = sizeof(bssList_t);

	return status;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilBssidListScanOid
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilBssidListScanOid(
					PTIWLN_ADAPTER_T pAdapter,
					PUCHAR pData,
					ULONG Length
					)
{
	TI_STATUS Status;
//TRS: Scan changes from TI
    Status = UtilSetParam(pAdapter, SCAN_CNCN_BSSID_LIST_SCAN_PARAM, pData, 0);
//TRS: end of Scan changes from TI
	return Status;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilInfrastructureModeGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilInfrastructureModeGet(
						 PTIWLN_ADAPTER_T pAdapter,
						 PUCHAR pData,
						 PULONG Length
						 )
{ 
	ULONG retValue;

	if (!Length)
		return NOK;

	retValue = UtilGetParam(pAdapter, CTRL_DATA_CURRENT_BSS_TYPE_PARAM, pData, sizeof(bssType_e));
	*Length = sizeof(ULONG);
	return(retValue);
}


/*-----------------------------------------------------------------------------
Routine Name: UtilDesiredInfrastructureModeGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilDesiredInfrastructureModeGet(
								PTIWLN_ADAPTER_T pAdapter,
								PUCHAR pData,
								PULONG Length
								)
{ 
	ULONG retValue;

	if (!Length)
		return NOK;

	retValue = UtilGetParam(pAdapter, SITE_MGR_DESIRED_BSS_TYPE_PARAM, pData, sizeof(bssType_e));

	*Length = sizeof(ULONG);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilInfrastructureModeSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilInfrastructureModeSet(
						 PTIWLN_ADAPTER_T pAdapter,
						 PUCHAR pData,
						 ULONG Length
						 )
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, SITE_MGR_DESIRED_BSS_TYPE_PARAM, pData, sizeof(bssType_e));
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilFragmentationThresholdGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilFragmentationThresholdGet(
							 PTIWLN_ADAPTER_T pAdapter,
							 PUCHAR pData,
							 PULONG Length
							 )
{
	UINT16 FragThreshold;
	ULONG retValue;

	if (!Length)
	{
		*Length = sizeof(ULONG);
		return NOK;
	}

	retValue = UtilGetParam(pAdapter, HAL_CTRL_FRAG_THRESHOLD_PARAM, pData, sizeof(ULONG));

	FragThreshold = *(PUINT16)pData;
	*(PULONG)pData = FragThreshold;

	*Length = sizeof(ULONG);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilFragmentationThresholdSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilFragmentationThresholdSet(
							 PTIWLN_ADAPTER_T pAdapter,
							 PUCHAR pData,
							 ULONG Length
							 )
{
	UINT16 FragThreshold = (UINT16) *(PULONG)pData;
	ULONG retValue;

	FragThreshold = ((FragThreshold+1)>>1) << 1; /*make it always even(GreenA)*/
	retValue = UtilSetParam(pAdapter, HAL_CTRL_FRAG_THRESHOLD_PARAM, (PUCHAR)&FragThreshold, sizeof(UINT16));

	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilRtsThresholdGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilRtsThresholdGet(
				   PTIWLN_ADAPTER_T pAdapter,
				   PUCHAR pData,
				   PULONG Length
				   )
{
	UINT16 RtsThreshold;
	ULONG retValue;

	if (!Length)
	{
		*Length = sizeof(ULONG);
		return NOK;
	}

	retValue = UtilGetParam(pAdapter, HAL_CTRL_RTS_THRESHOLD_PARAM, pData, sizeof(ULONG));

	RtsThreshold = *(PUINT16)pData;
	*(PULONG)pData = RtsThreshold;
	*Length = sizeof (ULONG);

	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilSupportedRates
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilSupportedRates(
				  PTIWLN_ADAPTER_T pAdapter,
				  PUCHAR pData,
				  PULONG Length
				  )
{
	rates_t rateSet;
	ULONG retValue;

	retValue = UtilGetParam(pAdapter, SITE_MGR_DESIRED_SUPPORTED_RATE_SET_PARAM, (PUCHAR)&rateSet, sizeof(rates_t));

	NdisMoveMemory(pData, (PUCHAR)&rateSet, *Length);
	*Length = rateSet.len +1; /* 1 is added for the length field itself */

	return retValue; 
}


/*-----------------------------------------------------------------------------
Routine Name: UtilSupportedRatesSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilSupportedRatesSet(
					 PTIWLN_ADAPTER_T pAdapter,
					 PUCHAR pData,
					 ULONG Length
					 )
{
	rates_t rateSet;
	ULONG retValue;

	NdisMoveMemory(&rateSet, pData, Length);

	retValue = UtilSetParam(pAdapter, SITE_MGR_DESIRED_SUPPORTED_RATE_SET_PARAM, (PUCHAR)&rateSet, sizeof(rates_t));

	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilRtsThresholdSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilRtsThresholdSet(
				   PTIWLN_ADAPTER_T pAdapter,
				   PUCHAR pData,
				   ULONG Length
				   )
{
	UINT16 RtsThreshold = (UINT16) *(PULONG)pData;
	ULONG retValue;

	retValue = UtilSetParam(pAdapter, HAL_CTRL_RTS_THRESHOLD_PARAM, 
							(PUCHAR)&RtsThreshold, sizeof(UINT16));

	return retValue;

}


/*-----------------------------------------------------------------------------
Routine Name: UtilChannelGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilChannelGet(
			  PTIWLN_ADAPTER_T pAdapter,
			  PUCHAR pData,
			  PULONG Length
			  )
{
	ULONG Channel,retValue; 

	if (!Length)
		return NOK;

	retValue = UtilGetParam(pAdapter, SITE_MGR_CURRENT_CHANNEL_PARAM, pData, sizeof(ULONG));

	Channel = *(PUCHAR)pData;
	*(PULONG)pData = (ULONG) Channel;
	*Length = sizeof (ULONG);

	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilDesiredChannelGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilDesiredChannelGet(
					 PTIWLN_ADAPTER_T pAdapter,
					 PUCHAR pData,
					 PULONG Length
					 )
{
	ULONG Channel,retValue; 

	if (!Length)
		return NOK;

	retValue = UtilGetParam(pAdapter, SITE_MGR_DESIRED_CHANNEL_PARAM, pData, sizeof(ULONG));

	Channel = *(PUCHAR)pData;
	*(PULONG)pData = (ULONG) Channel;
	*Length = sizeof (ULONG);

	return retValue;

}


/*-----------------------------------------------------------------------------
Routine Name: UtilDesiredChannelSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilDesiredChannelSet(
					 PTIWLN_ADAPTER_T pAdapter,
					 PUCHAR pData,
					 ULONG Length
					 )
{
	UINT8 Channel = *pData;
	ULONG retValue;

	retValue = UtilSetParam(pAdapter, SITE_MGR_DESIRED_CHANNEL_PARAM, &Channel, sizeof(UCHAR));

	return retValue;

}


/*-----------------------------------------------------------------------------
Routine Name: UtilShortPreambleGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilShortPreambleGet(
					PTIWLN_ADAPTER_T pAdapter,
					PUCHAR pData,
					PULONG Length
					)
{
	ULONG retValue;
	if (!Length)
	{
		*Length = sizeof(ULONG);
		return NOK;
	}

	retValue = UtilGetParam(pAdapter, SITE_MGR_DESIRED_PREAMBLE_TYPE_PARAM, pData, sizeof(ULONG));

	*Length = sizeof (ULONG);

	return retValue;

}


/*-----------------------------------------------------------------------------
Routine Name: UtilShortPreambleSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilShortPreambleSet(
					PTIWLN_ADAPTER_T pAdapter,
					PUCHAR pData,
					ULONG Length
					)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, SITE_MGR_DESIRED_PREAMBLE_TYPE_PARAM, 
							pData, sizeof(ULONG));

	return retValue;

}


/*-----------------------------------------------------------------------------
Routine Name: UtilRegulatoryDomain_enableDisable_802_11d
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilRegulatoryDomain_enableDisable_802_11d(
										  PTIWLN_ADAPTER_T pAdapter,
										  PUCHAR pData,
										  ULONG Length
										  )
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, REGULATORY_DOMAIN_ENABLE_DISABLE_802_11D, pData, sizeof(UINT8));

	return retValue;

}

/*-----------------------------------------------------------------------------
Routine Name: UtilRegulatoryDomain_enableDisable_802_11h
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilRegulatoryDomain_enableDisable_802_11h(
										  PTIWLN_ADAPTER_T pAdapter,
										  PUCHAR pData,
										  ULONG Length
										  )
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, REGULATORY_DOMAIN_ENABLE_DISABLE_802_11H, pData, sizeof(UINT8));

	return retValue;

}


/*-----------------------------------------------------------------------------
Routine Name: UtilRegulatoryDomain_Get_802_11d
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilRegulatoryDomain_Get_802_11d(
								PTIWLN_ADAPTER_T pAdapter,
								PUCHAR pData,
								PULONG Length
								)
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, REGULATORY_DOMAIN_ENABLED_PARAM, pData, sizeof(UINT8));

	return retValue;

}

/*-----------------------------------------------------------------------------
Routine Name: UtilRegulatoryDomain_Get_802_11h
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilRegulatoryDomain_Get_802_11h(
								PTIWLN_ADAPTER_T pAdapter,
								PUCHAR pData,
								PULONG Length
								)
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, REGULATORY_DOMAIN_MANAGEMENT_CAPABILITY_ENABLED_PARAM, pData, sizeof(UINT8));

	return retValue;

}

/*-----------------------------------------------------------------------------
Routine Name: UtilRegulatoryDomain_setCountryIE
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
static TI_STATUS
UtilRegulatoryDomain_setCountryIE(
								 PTIWLN_ADAPTER_T pAdapter,
								 externalParam_e ParamType,
								 PUCHAR pData,
								 ULONG Length
								 )
{
	paramInfo_t Param;
	TI_STATUS Status;
	country_t  countryIe;

	Param.paramType = ParamType;
	Param.paramLength = sizeof(country_t);
	NdisMoveMemory(&countryIe, pData, Length);

	Param.content.pCountry = &countryIe;
	Status = configMgr_setParam(pAdapter->CoreHalCtx, &Param);

	return Status;

}


/*-----------------------------------------------------------------------------
Routine Name: UtilRegulatoryDomain_setCountryIE_2_4
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilRegulatoryDomain_setCountryIE_2_4(
									 PTIWLN_ADAPTER_T pAdapter,
									 PUCHAR pData,
									 ULONG Length)
{
	ULONG retValue;

	retValue = UtilRegulatoryDomain_setCountryIE(pAdapter, REGULATORY_DOMAIN_COUNTRY_2_4_PARAM, pData, sizeof(country_t));

	return retValue;

}



/*-----------------------------------------------------------------------------
Routine Name: UtilRegulatoryDomain_getCountryIE_2_4
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilRegulatoryDomain_getCountryIE_2_4(
									 PTIWLN_ADAPTER_T pAdapter,
									 PUCHAR pData,
									 PULONG Length
									 )
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, REGULATORY_DOMAIN_COUNTRY_2_4_PARAM, pData, COUNTRY_STRING_LEN);

	return retValue;

}

/*-----------------------------------------------------------------------------
Routine Name: UtilRegulatoryDomain_setCountryIE_5
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilRegulatoryDomain_setCountryIE_5(
								   PTIWLN_ADAPTER_T pAdapter,
								   PUCHAR pData,
								   ULONG Length
								   )
{
	ULONG retValue;
	retValue = UtilRegulatoryDomain_setCountryIE(pAdapter, REGULATORY_DOMAIN_COUNTRY_5_PARAM, pData, sizeof(country_t));

	return retValue;

}


/*-----------------------------------------------------------------------------
Routine Name: UtilRegulatoryDomain_getCountryIE_5
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilRegulatoryDomain_getCountryIE_5(
								   PTIWLN_ADAPTER_T pAdapter,
								   PUCHAR pData,
								   PULONG Length
								   )
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, REGULATORY_DOMAIN_COUNTRY_5_PARAM, pData, COUNTRY_STRING_LEN);

	return retValue;

}

/*-----------------------------------------------------------------------------
Routine Name: UtilRegulatoryDomain_setMinMaxDfsChannels
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilRegulatoryDomain_setMinMaxDfsChannels(
    PTIWLN_ADAPTER_T pAdapter,
    PUCHAR pData,
    ULONG Length
    )
{
    ULONG retValue;

    retValue = UtilSetParam(pAdapter, REGULATORY_DOMAIN_DFS_CHANNELS_RANGE, pData, sizeof(DFS_ChannelRange_t));

    return retValue;

}


/*-----------------------------------------------------------------------------
Routine Name: UtilRegulatoryDomain_getMinMaxDfsChannels
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilRegulatoryDomain_getMinMaxDfsChannels(
    PTIWLN_ADAPTER_T pAdapter,
    PUCHAR pData,
    PULONG Length
    )
{
    ULONG retValue;
    retValue = UtilGetParam(pAdapter, REGULATORY_DOMAIN_DFS_CHANNELS_RANGE, pData, sizeof(DFS_ChannelRange_t));

    return retValue;

}


/*-----------------------------------------------------------------------------
Routine Name: UtilShortRetryGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilShortRetryGet(
				 PTIWLN_ADAPTER_T pAdapter,
				 PUCHAR pData,
				 PULONG Length
				 )

{
	ULONG retValue;


	txRatePolicy_t  TxRatePolicy;
	/*
	 * NOTE: currently supporting only ONE txRatePolicy!!!!!!!!!
	 */


	if (!Length)
		return sizeof(ULONG);

	retValue = UtilGetParam(pAdapter, CTRL_DATA_SHORT_RETRY_LIMIT_PARAM, 
							(PUCHAR)(&TxRatePolicy), sizeof(txRatePolicy_t));

	*(PULONG)pData = TxRatePolicy.rateClass[0].shortRetryLimit;

	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilShortRetrySet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilShortRetrySet(
				 PTIWLN_ADAPTER_T pAdapter,
				 PUCHAR pData,
				 ULONG Length
				 )
{
	ULONG retValue;
	txRatePolicy_t  TxRatePolicy;

	/*
	 * NOTE: currently supporting only ONE txRatePolicy!!!!!!!!!
	 */


	TxRatePolicy.rateClass[0].shortRetryLimit = (UINT8) *(PULONG)pData;

	retValue = UtilSetParam(pAdapter, CTRL_DATA_SHORT_RETRY_LIMIT_PARAM, 
							(PUCHAR)(&TxRatePolicy), sizeof(txRatePolicy_t));

	return retValue ;

}


/*-----------------------------------------------------------------------------
Routine Name: UtilLongRetryGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilLongRetryGet(
				PTIWLN_ADAPTER_T pAdapter,
				PUCHAR pData,
				PULONG Length
				)
{
	ULONG retValue;
	txRatePolicy_t  TxRatePolicy;
	/*
	 * NOTE: currently supporting only ONE txRatePolicy!!!!!!!!!
	 */


	if (!Length)
		return NOK;

	retValue = UtilGetParam(pAdapter, CTRL_DATA_LONG_RETRY_LIMIT_PARAM, 
							(PUCHAR)(&TxRatePolicy), sizeof(txRatePolicy_t));

	*(PULONG)pData = TxRatePolicy.rateClass[0].longRetryLimit;
	return retValue;

}


/*-----------------------------------------------------------------------------
Routine Name: UtilLongRetrySet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilLongRetrySet(
				PTIWLN_ADAPTER_T pAdapter,
				PUCHAR pData,
				ULONG Length
				)
{
	txRatePolicy_t  TxRatePolicy;
	ULONG retValue;

	/*
	 * NOTE: currently supporting only ONE txRatePolicy!!!!!!!!!
	 */


	TxRatePolicy.rateClass[0].longRetryLimit = (UINT8) *(PULONG)pData;

	retValue = UtilSetParam(pAdapter, CTRL_DATA_SHORT_RETRY_LIMIT_PARAM, 
							(PUCHAR)(&TxRatePolicy), sizeof(txRatePolicy_t));

	return retValue;
}
/*-----------------------------------------------------------------------------*/
ULONG
UtilDesiredRatesGet(
				   PTIWLN_ADAPTER_T pAdapter,
				   PUCHAR pData,
				   PULONG Length
				   )
{
	UCHAR rate;
	ULONG retValue;

	if (!Length)
		return NOK;

	retValue = UtilGetParam(pAdapter, SITE_MGR_DESIRED_TX_RATE_PARAM, (PUCHAR)&rate, sizeof(UCHAR));

	*Length = sizeof(UCHAR);
	*(PUCHAR)pData = rate;

	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name: UtilCurrentRatesGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilCurrentRatesGet(
				   PTIWLN_ADAPTER_T pAdapter,
				   PUCHAR pData,
				   PULONG Length
				   )
{
	UCHAR rate;
	ULONG retValue;

	rate = (UCHAR) *(PULONG)pData;

	retValue = UtilGetParam(pAdapter, SITE_MGR_CURRENT_TX_RATE_PARAM, (PUCHAR)&rate, sizeof(UCHAR));
	*(PUCHAR)pData = rate;
	*Length = sizeof(UCHAR);

	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilConfigurationGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilConfigurationGet(
					PTIWLN_ADAPTER_T pAdapter,
					PUCHAR pData,
					PULONG Length
					)
{
	paramInfo_t Param;
	ULONG retValue;

	if (!Length)
	{
		*Length = sizeof(OS_802_11_CONFIGURATION);
		return NOK;
	}

	Param.paramType = SITE_MGR_CONFIGURATION_PARAM;
	Param.paramLength = *Length;
	*(PULONG)&Param.content = (ULONG)pData;

	retValue = configMgr_getParam(pAdapter->CoreHalCtx, &Param);

	*Length = sizeof(OS_802_11_CONFIGURATION);

	return retValue;

}


/*-----------------------------------------------------------------------------
Routine Name: UtilConfigurationSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilConfigurationSet(
					PTIWLN_ADAPTER_T pAdapter,
					PUCHAR pData,
					ULONG Length
					)
{
	paramInfo_t Param;
	ULONG retValue;

	Param.paramType = SITE_MGR_CONFIGURATION_PARAM;
	Param.paramLength = Length;
	*(PULONG)&Param.content = (ULONG)pData;

	retValue = configMgr_setParam(pAdapter->CoreHalCtx, &Param);

	return retValue;

}

/*-----------------------------------------------------------------------------
Routine Name: UtilGetCounter
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilGetCounter(
			  PTIWLN_ADAPTER_T pAdapter,
			  PUCHAR pData,
			  ULONG Offset
			  )
{
	TIWLN_COUNTERS TiCounters;

	UtilGetParam(pAdapter, SITE_MGR_TI_WLAN_COUNTERS_PARAM, 
				 (PUCHAR)&TiCounters, sizeof(TIWLN_COUNTERS));

	NdisMoveMemory(pData, (PUCHAR)&TiCounters + Offset, sizeof(ULONG));

	return sizeof(ULONG);
}

/*-----------------------------------------------------------------------------
Routine Name: UtilStatistics
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilStatistics(
			  PTIWLN_ADAPTER_T pAdapter,
			  PUCHAR pData,
			  PULONG Length
			  )
{
	TIWLN_STATISTICS* pStats;
	paramInfo_t Param;
	ULONG RetSize, data,retValue,dataSize;


    if (*Length >= sizeof(TIWLN_STATISTICS)) //TRS:GAA allow larger than needed buffer
	{
		pStats = (TIWLN_STATISTICS *) pData;
		NdisZeroMemory(pStats, sizeof(TIWLN_STATISTICS));

		RetSize = sizeof(TIWLN_STATISTICS);

		NdisMoveMemory(&pStats->currentMACAddress, pAdapter->CurrentAddr, ETH_ADDR_SIZE);

		dataSize = sizeof(tiUINT32);
		if ((retValue = UtilPowerModeGet(pAdapter, (PUCHAR)&pStats->PowerMode, &dataSize)) != OK)
			return retValue;

		dataSize = sizeof(OS_802_11_SSID);
		if ((retValue = UtilGetSSID(pAdapter, (PUCHAR)&pStats->dot11DesiredSSID, &dataSize)) != OK)
			return retValue;

		dataSize = sizeof(UINT32);
		if ((retValue = UtilChannelGet(pAdapter, (PUCHAR)&pStats->dot11CurrentChannel, &dataSize)) != OK)
			return retValue;

		dataSize = sizeof(ULONG);
		if ((retValue = UtilExtAuthenticationModeGet(pAdapter, (PUCHAR)&pStats->AuthenticationMode, &dataSize)) != OK)
			return retValue;

		dataSize = sizeof(ULONG);
		if ((retValue = UtilRtsThresholdGet(pAdapter, (PUCHAR)&pStats->RTSThreshold,    &dataSize)) != OK)
			return retValue;

		dataSize = sizeof(ULONG);
		if ((retValue = UtilFragmentationThresholdGet(pAdapter, (PUCHAR)&pStats->FragmentationThreshold, &dataSize)) != OK)
			return retValue;

		dataSize = sizeof(ULONG);
        if ((retValue = UtilGetParam(pAdapter, REGULATORY_DOMAIN_CURRENT_TX_POWER_IN_DBM_PARAM, 
									(PUCHAR)&pStats->TxPowerDbm, dataSize)) != OK)
			return retValue;

		dataSize = sizeof(ULONG);
		if ((retValue = UtilInfrastructureModeGet(pAdapter, (PUCHAR)&pStats->dot11BSSType, &dataSize)) != OK)
			return retValue;

		dataSize = sizeof(ULONG);
		if ((retValue = UtilWepStatusGet(pAdapter, (PUCHAR)&pStats->WEPStatus, &dataSize)) != OK)
			return retValue;

		if ((retValue = UtilGetParam(pAdapter, SITE_MGR_CONNECTION_STATUS_PARAM, (PUCHAR)&pStats->dot11State, sizeof(ULONG))) != OK)
			return retValue;

		pStats->dot11CurrentTxRate = pAdapter->LinkSpeed/5000;

		if ((retValue = UtilGetParam(pAdapter, SITE_MGR_CURRENT_PREAMBLE_TYPE_PARAM, (PUCHAR)&data, sizeof(ULONG))) != OK)
			return retValue;

		pStats->bShortPreambleUsed = (BOOLEAN) data;

		Param.paramType = SITE_MGR_GET_SELECTED_BSSID_INFO;
		Param.content.pSiteMgrPrimarySiteDesc = &pStats->targetAP;
		if ((retValue = configMgr_getParam(pAdapter->CoreHalCtx, &Param)) != OK)
			return retValue;

		PRINTF(DBG_IOCTL_LOUD, ("...RSSI: %d\n", pStats->targetAP.Rssi));
		pStats->RxLevel = pStats->targetAP.Rssi;

		dataSize = sizeof(ULONG);
		if ((retValue = UtilTxAntennaGet(pAdapter, (PUCHAR)&pStats->TxAntenna, &dataSize)) != OK)
			return retValue;

		dataSize = sizeof(ULONG);
		if ((retValue = UtilRxAntennaGet(pAdapter, (PUCHAR)&pStats->RxAntenna, &dataSize)) != OK)
			return retValue;

#ifdef EXC_MODULE_INCLUDED		
		dataSize = sizeof(BOOL);
		if ((retValue = UtilExcNetworkEapGet(pAdapter, (PUCHAR)&pStats->dwSecuritySuit, &dataSize)) != OK)
			return retValue;
		if ((pStats->dwSecuritySuit==OS_EXC_NETWORK_EAP_ON) && (pStats->WEPStatus==os802_11WEPEnabled))
		{
			pStats->dwSecuritySuit = TIWLN_STAT_SECURITY_RESERVE_1;
		} else
#else
		{
			pStats->dwSecuritySuit = 0;
		}
#endif
		if ((retValue = UtilGetParam(pAdapter, RSN_SECURITY_STATE_PARAM, (PUCHAR)&pStats->dwSecurityState, sizeof(ULONG))) != OK)
			return retValue;

		pStats->dwSecurityAuthStatus = 0;
		pStats->dwFeatureSuit = 0;

		if ((retValue = UtilGetParam(pAdapter, SITE_MGR_TI_WLAN_COUNTERS_PARAM, (PUCHAR)&pStats->tiCounters, sizeof(TIWLN_COUNTERS))) != OK)
			return retValue;

        if ((retValue = UtilGetParam(pAdapter, MLME_BEACON_RECV, (PUCHAR)&pStats->tiCounters, sizeof(TIWLN_COUNTERS))) != OK)
			return retValue;
	}

	else
	{

		NdisZeroMemory(pData, *Length);
		*Length = 0;
		return NOK;

	}

	*Length = RetSize;
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilTxStatistics
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilTxStatistics(
				PTIWLN_ADAPTER_T pAdapter,
				PUCHAR pData,
				ULONG inLength,
				PULONG outLength
				)
{
	ULONG retValue;
	txDataCounters_t  *pTxDataCounters;
	UINT32 resetStatsFlag;

	if (*outLength == sizeof(TIWLN_TX_STATISTICS))
	{
		/* check whether statistics clear is also requested */
		resetStatsFlag = *pData;

		/* note that only the pointer (by reference!) is passed to UtilGetParam, and the actual copying of data
		   is done here */
		if ((retValue = UtilGetParam( pAdapter, TX_DATA_COUNTERS_PARAM, 
									  (PUCHAR)&pTxDataCounters, sizeof(txDataCounters_t*))) != OK)
			return retValue;

		NdisMoveMemory( pData, pTxDataCounters, sizeof(TIWLN_TX_STATISTICS) );

		*outLength = sizeof(TIWLN_TX_STATISTICS);

		if ( 1 == resetStatsFlag )
		{
			UtilSetParam( pAdapter, TX_DATA_RESET_COUNTERS_PARAM, NULL, 0 );
		}
	} else
	{
		NdisZeroMemory(pData, *outLength);
		*outLength = 0;
		return NOK;
	}

	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilAddWep
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilAddWep(
		  PTIWLN_ADAPTER_T pAdapter,
		  PUCHAR pData,
		  ULONG Length,
		  BOOLEAN CalledFromIoctl
		  )
{
	OS_802_11_WEP* pWep;
	OS_802_11_KEY  key;
	UINT32         keyIndexTxRx;
	TI_STATUS Status;

	pWep = (OS_802_11_WEP*) pData;

	if ((pWep->KeyIndex & 0x3FFFFFFF) > 3)
	{
		return 0;
	}

	if (CalledFromIoctl)
	{
		NdisMoveMemory(&pAdapter->DefaultWepKeys[pWep->KeyIndex & 0x3FFFFFFF],
					   pWep, sizeof(OS_802_11_WEP));
	}
	key.Length = pWep->Length;
	/* Convert the Key index to match OS_802_11_KEY index */

	keyIndexTxRx = (pWep->KeyIndex & 0x80000000); 

	key.KeyIndex = keyIndexTxRx | /*(keyIndexTxRx>>1) |*/ 
				   (pWep->KeyIndex & 0x3FFFFFFF);

	key.KeyLength = pWep->KeyLength;  

    if( pWep->KeyLength > sizeof(key.KeyMaterial) ) { /* Dm: Security fix */
        printk("TI: %s - Security Error\n", __FUNCTION__);
        return EXTERNAL_SET_PARAM_DENIED;
    }

	NdisMoveMemory(key.KeyMaterial, (void *)pWep->KeyMaterial, pWep->KeyLength);
	/* Set the MAC Address to zero for WEP */
	NdisZeroMemory(key.BSSID, sizeof(key.BSSID));

	Status = UtilSetParam(pAdapter, RSN_ADD_KEY_PARAM, 
						  (PUCHAR)&key, sizeof(OS_802_11_KEY));

	return Status;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilRemoveWep
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilRemoveWep(
			 PTIWLN_ADAPTER_T pAdapter,
			 PUCHAR pData,
			 ULONG Length
			 )
{
	UINT32          keyIndex;
	OS_802_11_KEY  key;
	TI_STATUS Status;

	keyIndex = *(UINT32*)pData;

	/* Convert the Key index to match OS_802_11_KEY index */
	NdisZeroMemory(&key, sizeof(OS_802_11_KEY));

	key.KeyIndex = keyIndex;

	Status = UtilSetParam(pAdapter, RSN_REMOVE_KEY_PARAM, 
						  (PUCHAR)&key, sizeof(OS_802_11_KEY));

	return(Status);
}


#define ADD_KEY_HEADER_LENGTH 26


/*-----------------------------------------------------------------------------
Routine Name: UtilAddKey
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilAddKey(
		  PTIWLN_ADAPTER_T pAdapter,
		  PUCHAR pData,
		  ULONG Length
		  )
{   
	TI_STATUS   status;
	OS_802_11_KEY* pKey;

	pKey = (OS_802_11_KEY*) pData;

	status = UtilSetParam(pAdapter, RSN_ADD_KEY_PARAM, pData, pKey->Length);

	return status;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilRemoveKey
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilRemoveKey(
			 PTIWLN_ADAPTER_T pAdapter,
			 PUCHAR pData,
			 ULONG Length
			 )
{   
	TI_STATUS               status;
	OS_802_11_REMOVE_KEY    *pRemoveKey;
	OS_802_11_KEY           key;

	pRemoveKey = (OS_802_11_REMOVE_KEY*)pData;

	key.KeyIndex = pRemoveKey->KeyIndex;
	NdisMoveMemory(key.BSSID, (void *)pRemoveKey->BSSID, sizeof(key.BSSID));

	status = UtilSetParam(pAdapter, RSN_REMOVE_KEY_PARAM, 
						  (PUCHAR)&key, sizeof(OS_802_11_KEY));

	return(status);
}


/*-----------------------------------------------------------------------------
Routine Name: UtilExtAuthenticationModeSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilExtAuthenticationModeSet(
							PTIWLN_ADAPTER_T pAdapter,
							PUCHAR pData,
							ULONG Length
							)
{       
	ULONG retValue;

	retValue = UtilSetParam(pAdapter, RSN_EXT_AUTHENTICATION_MODE, pData, sizeof(ULONG));

	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilExtAuthenticationModeGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilExtAuthenticationModeGet(
							PTIWLN_ADAPTER_T pAdapter,
							PUCHAR pData,
							PULONG Length
							)
{       
	ULONG retValue;

	if (!Length)
		return NOK;

	retValue = UtilGetParam(pAdapter, RSN_EXT_AUTHENTICATION_MODE, pData, sizeof(ULONG));

	*Length = sizeof (ULONG);

	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: Util802CapabilityGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG Util802CapabilityGet(
						  PTIWLN_ADAPTER_T pAdapter, 
						  PUCHAR pData, 
						  PULONG Length)
{
	OS_802_11_CAPABILITY         *capability_802_11;
	rsnAuthEncrCapability_t       rsnAuthEncrCap;
	OS_802_11_ENCRYPTION_STATUS   encrStatus = os802_11EncryptionDisabled;
	ULONG                         neededLength = 0;
	UINT                          i = 0;
	paramInfo_t                   Param;
	TI_STATUS                     status;

	/* If length of the input buffer less than needed length, do nothing, */
	/* return the needed length                                           */
	neededLength = sizeof(OS_802_11_CAPABILITY) + 
				   (sizeof(OS_802_11_AUTH_ENCRYPTION) * (MAX_AUTH_ENCR_PAIR -1));

	if (*Length < neededLength)
	{
		*Length = neededLength;
		return NOK;
	}

	NdisZeroMemory (pData, neededLength);
	capability_802_11  = (OS_802_11_CAPABILITY *)pData;

	/* Fill Param fields and get the 802_11 capability information */
	Param.paramType   = RSN_AUTH_ENCR_CAPABILITY;
	Param.paramLength = neededLength;
	Param.content.pRsnAuthEncrCapability = &rsnAuthEncrCap;

	status = configMgr_getParam(pAdapter->CoreHalCtx, &Param);

	if (status != OK)
	{
		/* return the default values only */
		/* PMKIDs is 0, AUTH/Encr pairs is 1, Auth/Encr is OPEN/NONE (0/0) */
		capability_802_11->Length     = sizeof(OS_802_11_CAPABILITY);
		capability_802_11->Version    = OID_CAPABILITY_VERSION;
		capability_802_11->NoOfPmKIDs = 0;
		capability_802_11->NoOfAuthEncryptPairsSupported = 1;
		capability_802_11->AuthEncryptionSupported[0].AuthModeSupported  = 
		os802_11AuthModeOpen;
		capability_802_11->AuthEncryptionSupported[0].EncryptionStatusSupported = 
		os802_11EncryptionDisabled;


		*Length = sizeof(OS_802_11_CAPABILITY);
		return NOK;
	}

	/* Copy the received info to the OS_802_11_CAPABILITY needed format */
	capability_802_11->Length     = neededLength;
	capability_802_11->Version    = OID_CAPABILITY_VERSION;
	capability_802_11->NoOfPmKIDs = rsnAuthEncrCap.NoOfPMKIDs;
	capability_802_11->NoOfAuthEncryptPairsSupported = 
	rsnAuthEncrCap.NoOfAuthEncrPairSupported;

	/* Convert received cipher suite type to encr.status type */
	for (i = 0; i < rsnAuthEncrCap.NoOfAuthEncrPairSupported; i ++)
	{
		capability_802_11->AuthEncryptionSupported[i].AuthModeSupported = 
		(OS_802_11_AUTHENTICATION_MODE)rsnAuthEncrCap.authEncrPairs[i].authenticationMode;

		switch (rsnAuthEncrCap.authEncrPairs[i].cipherSuite)
		{
		case RSN_CIPHER_NONE:
			encrStatus = os802_11EncryptionDisabled;
			break;

		case RSN_CIPHER_WEP:
			encrStatus = os802_11WEPEnabled;
			break;

		case RSN_CIPHER_TKIP:
			encrStatus = os802_11Encryption2Enabled;
			break;

		case RSN_CIPHER_AES_CCMP:
			encrStatus = os802_11Encryption3Enabled;
			break;

		default:
			encrStatus = os802_11EncryptionDisabled;
			break;

		}
		capability_802_11->AuthEncryptionSupported[i].EncryptionStatusSupported
		= encrStatus;
	}

	/* Success; return the actual length of the written data */
	*Length = neededLength;
	return status;

}


/*-----------------------------------------------------------------------------
Routine Name:	Util802PmkidGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG Util802PmkidGet(
					 PTIWLN_ADAPTER_T pAdapter, 
					 PUCHAR pData, 
					 PULONG Length)
{
	OS_802_11_PMKID         *pPmkidList = (OS_802_11_PMKID *)pData;
	TI_STATUS                status = NOK;

	/* Check the data buffer size */
	if (*Length < sizeof(OS_802_11_PMKID))
	{
		*Length = (sizeof(OS_802_11_PMKID));
		return NOK;
	}

	NdisZeroMemory(pData, sizeof(OS_802_11_PMKID));
	pPmkidList->Length = *Length;

	status = UtilGetParam(pAdapter, RSN_PMKID_LIST, pData, *Length);

	if (status != OK)
	{
		if (*Length < (pPmkidList->Length))
			*Length = pPmkidList->Length;
		else
			*Length	= 0;
	} 
    else
	{
		*Length = pPmkidList->Length;
	}

	return status;
}

/*-----------------------------------------------------------------------------
Routine Name:	Util802PmkidSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
Util802PmkidSet(
			   PTIWLN_ADAPTER_T pAdapter,
			   PUCHAR pData,
			   ULONG Length
			   )
{   
	paramInfo_t Param;
	TI_STATUS   status;

	Param.paramType = RSN_PMKID_LIST;
    Param.paramLength = Length;
    if( Length > sizeof(Param.content) ) { /* Dm: Security fix */
        printk("TI: %s - Security Error\n",__FUNCTION__);
        return EXTERNAL_SET_PARAM_DENIED;
    }
	NdisMoveMemory(&Param.content, pData, Length);

	status = configMgr_setParam(pAdapter->CoreHalCtx, &Param);

	return(status);
}

/*-----------------------------------------------------------------------------
Routine Name:	Util802FSWAvailableOptionsGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
Util802FSWAvailableOptionsGet(
							 PTIWLN_ADAPTER_T pAdapter, 
							 PUCHAR pData, 
							 PULONG Length)
{
	ULONG retValue;

	if (!Length)
	{
		*Length = sizeof(ULONG);
		return NOK;
	}

	retValue = UtilGetParam(pAdapter, RSN_WPA_PROMOTE_AVAILABLE_OPTIONS, 
							pData, sizeof(ULONG));
	*Length = sizeof(ULONG);

	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	Util802FSWOptionsGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
Util802FSWOptionsGet(
					PTIWLN_ADAPTER_T pAdapter, 
					PUCHAR pData, 
					PULONG Length)
{
	ULONG retValue;

	if (!Length)
	{
		*Length = sizeof(ULONG);
		return NOK;
	}

	retValue = UtilGetParam(pAdapter, RSN_WPA_PROMOTE_OPTIONS, pData, sizeof(ULONG));

	*Length = sizeof(ULONG);
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name:	Util802FSWOptionsSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/

ULONG
Util802FSWOptionsSet(
					PTIWLN_ADAPTER_T pAdapter, 
					PUCHAR pData, 
					ULONG Length)
{
	ULONG retValue;

	retValue = UtilSetParam(pAdapter, RSN_WPA_PROMOTE_OPTIONS, pData, sizeof(ULONG));

	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name:	UtilWepStatusGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilWepStatusGet(
				PTIWLN_ADAPTER_T pAdapter,
				PUCHAR pData,
				PULONG Length
				)
{
	ULONG retValue;

	if (!Length)
	{
		*Length = sizeof(ULONG);
		return NOK;
	}

	retValue = UtilGetParam(pAdapter, RSN_ENCRYPTION_STATUS_PARAM, pData, sizeof(ULONG));

	*Length = sizeof(ULONG);
	return retValue;

}


/*-----------------------------------------------------------------------------
Routine Name:	UtilWepStatusSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilWepStatusSet(
				PTIWLN_ADAPTER_T pAdapter,
				PUCHAR pData,
				ULONG Length
				)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, RSN_ENCRYPTION_STATUS_PARAM, pData, sizeof(ULONG));
	return retValue;
}



/*-----------------------------------------------------------------------------
Routine Name:	UtilAssociationInfoGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilAssociationInfoGet(
					  PTIWLN_ADAPTER_T pAdapter,
					  PUCHAR pData,
					  PULONG Length
					  )
{
	OS_802_11_ASSOCIATION_INFORMATION  *assocInformation;
	UINT8 *requestIEs;
	UINT8 *responseIEs;
	ULONG TotalLength = 0,retValue;
	paramInfo_t param;

	if (*Length < sizeof(OS_802_11_ASSOCIATION_INFORMATION))
	{
		PRINT(DBG_IOCTL_LOUD, "...More buffer space needed\n");
		return(sizeof(OS_802_11_ASSOCIATION_INFORMATION));
	}

	param.paramType   = ASSOC_ASSOCIATION_INFORMATION_PARAM;
	param.paramLength = *Length;

	retValue = configMgr_getParam(pAdapter->CoreHalCtx, &param);

	TotalLength =  sizeof(OS_802_11_ASSOCIATION_INFORMATION) + 
				   param.content.assocAssociationInformation.RequestIELength +
				   param.content.assocAssociationInformation.ResponseIELength;

	if (TotalLength <= *Length)
	{
		NdisMoveMemory(pData, (UINT8 *)&param.content, sizeof(OS_802_11_ASSOCIATION_INFORMATION));
		assocInformation = (OS_802_11_ASSOCIATION_INFORMATION*)pData;
		requestIEs = (UINT8*)pData + sizeof(OS_802_11_ASSOCIATION_INFORMATION);

		if (assocInformation->RequestIELength > 0)
		{

			NdisMoveMemory(requestIEs, (UINT8*)assocInformation->OffsetRequestIEs, 
						   assocInformation->RequestIELength); 

			assocInformation->OffsetRequestIEs = sizeof(OS_802_11_ASSOCIATION_INFORMATION);
		}

		if (assocInformation->ResponseIELength > 0)
		{

			responseIEs = requestIEs + assocInformation->RequestIELength;

			NdisMoveMemory(responseIEs, (UINT8*)assocInformation->OffsetResponseIEs,
						   assocInformation->ResponseIELength); 

			assocInformation->OffsetResponseIEs = 
			assocInformation->OffsetRequestIEs + assocInformation->RequestIELength;
		}

		PRINTF(DBG_IOCTL_LOUD, ("UtilAssociationInfoGet: pData=%p, "
								"OffsetRequestIEs=0x%lx, OffsetResponseIEs=0x%lx\n",
								pData, (long)assocInformation->OffsetRequestIEs, 
								(long)assocInformation->OffsetResponseIEs));
	} else
	{
		*(PULONG)pData = TotalLength;
		PRINT(DBG_IOCTL_LOUD, "...More buffer space needed\n");
	}

	*Length = TotalLength;
	return retValue;

}

/*-----------------------------------------------------------------------------
Routine Name:	UtilCurrentRegDomainGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilCurrentRegDomainGet(
					   PTIWLN_ADAPTER_T pAdapter,
					   PUCHAR pData,
					   PULONG Length
					   )
{
	ULONG retValue;

	if (!Length)
		return NOK;

	retValue = UtilGetParam(pAdapter, REGULATORY_DOMAIN_CURRENT_REGULATORY_DOMAIN_PARAM, pData, sizeof(UINT8));

	*Length = sizeof(UINT8);

	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name:	Util4xActiveStateGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
Util4xActiveStateGet(
					PTIWLN_ADAPTER_T pAdapter,
					PUCHAR pData,
					PULONG Length
					)
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, CTRL_DATA_FOUR_X_CURRRENT_STATUS_PARAM, pData, sizeof(UINT8));
	*Length = sizeof(UINT8);
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name:	power
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
static int power(int x, int y)
{
	int i = 0,z = 1;

	for (i = 0; i < y; i++)
		z *= x;

	return z;
}

/*-----------------------------------------------------------------------------
Routine Name:	UtilGetSwVersion
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilGetSwVersion(
				PTIWLN_ADAPTER_T pAdapter,
				PUCHAR pData,
				PULONG Length
				)
{
	TIWLN_VERSION* swVer;
	ULONG retValue,tmpLen;
	UCHAR FwVersion[FW_VERSION_LEN];
	e2Version_t EepromVersion;
	int i, start = 0, end = 0, temp = 0;

	swVer = (TIWLN_VERSION *) pData;

	swVer->DrvVersion.major = SW_VERSION_MAJOR;
	swVer->DrvVersion.minor = SW_VERSION_MINOR;
	swVer->DrvVersion.bugfix = SW_VERSION_PATCH;
	swVer->DrvVersion.subld = SW_VERSION_SUBLD; 
        swVer->DrvVersion.build = SW_VERSION_BUILD; 

	NdisZeroMemory(&swVer->FWVersion, sizeof(swVer->FWVersion));

	UtilGetParam(pAdapter, SITE_MGR_FIRMWARE_VERSION_PARAM, FwVersion, FW_VERSION_LEN);

	/* major */
	start = end = temp = 4;
	while (FwVersion[end++] != '.');
	temp = end;
	end -= 2;
	for (i = end; i>= start; i--)
    {
		swVer->FWVersion.major += (FwVersion[i] - 0x30)*power(10, end - i);
    }
		

	/* minor */
	start = end = temp;
	while (FwVersion[end++] != '.');
	temp = end;
	end -= 2;
	for (i = end; i>= start; i--)
    {
		swVer->FWVersion.minor += (FwVersion[i] - 0x30)*power(10, end - i);
    }

	/* bug fix */
	start = end = temp;
	while (FwVersion[end++] != '.');
	temp = end;
	end -= 2;
	for (i = end; i>= start; i--)
    {
		swVer->FWVersion.bugfix += (FwVersion[i] - 0x30)*power(10, end - i);
    }
    

    /* build */
	start = end = temp;
	while (FwVersion[end++] != '.');
	temp = end;
	end -= 2;
	for (i = end; i>= start; i--)
    {
		swVer->FWVersion.subld += (FwVersion[i] - 0x30)*power(10, end - i);
    }

	/* minor build */
	start = end = temp;
	while (FwVersion[end++] != 0);
	temp = end;
	end -= 2;
	for (i = end; i>= start; i--)
    {
		swVer->FWVersion.build += (FwVersion[i] - 0x30)*power(10, end - i);
    }


	NdisZeroMemory(&swVer->HWVersion, sizeof(swVer->HWVersion));

	retValue = UtilGetParam(pAdapter, SITE_MGR_EEPROM_VERSION_PARAM, (PUCHAR)&EepromVersion, sizeof(e2Version_t));

	swVer->HWVersion.major = (UCHAR) EepromVersion.major;
	swVer->HWVersion.minor = (UCHAR) EepromVersion.minor;
	swVer->HWVersion.bugfix = (UCHAR) EepromVersion.bugfix;

	swVer->osNdisVersion  = (TIWLN_MAJOR_VERSION  << 16) + TIWLN_MINOR_VERSION;

	tmpLen = sizeof(TIWLN_VERSION);

    if (*Length >= sizeof(TIWLN_VERSION_EX)) //TRS:GAA allow larger than needed buffer
	{

		((PTIWLN_VERSION_EX)swVer)->extVerSign = 2;

		((PTIWLN_VERSION_EX)swVer)->NVVersion.bugfix = EepromVersion.last;

		((PTIWLN_VERSION_EX)swVer)->NVVersion.minor = EepromVersion.minor;

		((PTIWLN_VERSION_EX)swVer)->NVVersion.major = 
		(UCHAR)EepromVersion.major;

		((PTIWLN_VERSION_EX)swVer)->NVVersion.subld = 
		(UCHAR)EepromVersion.bugfix;

		tmpLen = sizeof(TIWLN_VERSION_EX);

	}

	*Length = tmpLen;

	return retValue;
}



/*-----------------------------------------------------------------------------
Routine Name:	UtilRxAntennaSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilRxAntennaSet(
				PTIWLN_ADAPTER_T pAdapter,
				PUCHAR pData,
				ULONG Length
				)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, HAL_CTRL_RX_ANTENNA_PARAM, pData, sizeof(ULONG));
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilRxAntennaGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilRxAntennaGet(
				PTIWLN_ADAPTER_T pAdapter,
				PUCHAR pData,
				PULONG Length
				)
{
	ULONG retValue;

	if ((!Length) || (*Length == 0))
		return NOK;

	retValue = UtilGetParam(pAdapter, HAL_CTRL_RX_ANTENNA_PARAM, pData, sizeof(UINT8));
	*Length = sizeof(UINT8);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilTxAntennaSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilTxAntennaSet(
				PTIWLN_ADAPTER_T pAdapter,
				PUCHAR pData,
				ULONG Length
				)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, HAL_CTRL_TX_ANTENNA_PARAM, pData, sizeof(ULONG));
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilTxAntennaGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilTxAntennaGet(
				PTIWLN_ADAPTER_T pAdapter,
				PUCHAR pData,
				PULONG Length
				)
{
	ULONG retValue;

	if ((!Length) || (*Length == 0))
		return NOK;

	retValue = UtilGetParam(pAdapter, HAL_CTRL_TX_ANTENNA_PARAM, pData, sizeof(UINT8));
	*Length = sizeof (UINT8);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilNumberOfAntennas
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilNumberOfAntennas(
					PTIWLN_ADAPTER_T pAdapter,
					PUCHAR pData,
					PULONG Length
					)
{
	if ((!Length) || (*Length == 0))
		return NOK;

	*(PULONG)pData = 2;
	*Length = sizeof(ULONG);
	return OK;
}

/*-----------------------------------------------------------------------------
Routine Name:
	UtilAntennaDivresitySet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilAntennaDivresitySet(
					   PTIWLN_ADAPTER_T pAdapter,
					   PUCHAR pData,
					   ULONG Length
					   )
{

	return(UtilSetParam(pAdapter, HAL_CTRL_ANTENNA_DIVERSITY_PARAMS, pData, Length));
}

/*-----------------------------------------------------------------------------
Routine Name:	UtilDriverStatusGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilDriverStatusGet(
				   PTIWLN_ADAPTER_T pAdapter,
				   PUCHAR pData,
				   PULONG Length
				   )
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, DRIVER_STATUS_PARAM, pData, sizeof(ULONG));
	*Length = sizeof (ULONG);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilDriverSuspend
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilDriverSuspend(
				 PTIWLN_ADAPTER_T pAdapter,
				 PUCHAR pData,
				 ULONG Length
				 )
{
	ULONG retValue;
	OS_802_11_SSID FakeSsid;
	UINT32 loopIndex;

	for (loopIndex = 0; loopIndex < MAX_SSID_LEN; loopIndex++)
		FakeSsid.Ssid[loopIndex] = (loopIndex+1);

	FakeSsid.SsidLength = MAX_SSID_LEN;
	retValue = UtilSetSSID(pAdapter, (PUCHAR)&FakeSsid, sizeof(OS_802_11_SSID));

	return retValue;

}

/*-----------------------------------------------------------------------------
Routine Name:	UtilDriverStatusSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilDriverStatusSet(
				   PTIWLN_ADAPTER_T pAdapter,
				   PUCHAR pData,
				   ULONG Length
				   )
{
	ULONG retValue;

	if (*(PULONG)pData)
	{
		retValue = configMgr_start(pAdapter->CoreHalCtx);
	} else
	{
		retValue = configMgr_stop(pAdapter->CoreHalCtx);
	}

	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilRssiGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilRssiGet(
		   PTIWLN_ADAPTER_T pAdapter,
		   PUCHAR pData,
		   PULONG Length
		   )
{
	TIWLN_STATISTICS pStats;
	paramInfo_t Param;
	ULONG retValue;

	if (!Length)
		return NOK;

	NdisZeroMemory(&pStats, sizeof(TIWLN_STATISTICS));

	Param.paramType = SITE_MGR_GET_SELECTED_BSSID_INFO;
	Param.content.pSiteMgrPrimarySiteDesc = &pStats.targetAP;

	retValue = configMgr_getParam(pAdapter->CoreHalCtx, &Param);

	*(PULONG)pData = pStats.targetAP.Rssi;

	*Length = sizeof(ULONG);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilDeviceSuspend
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
VOID
UtilDeviceSuspend(
				 PTIWLN_ADAPTER_T pAdapter
				 )
{
	configMgr_stop(pAdapter->CoreHalCtx);
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilDeviceResume
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
VOID
UtilDeviceResume(
				PTIWLN_ADAPTER_T pAdapter
				)
{
	configMgr_start(pAdapter->CoreHalCtx);
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilIbssProtectionGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilIbssProtectionGet(
					 PTIWLN_ADAPTER_T pAdapter,
					 PUCHAR pData,
					 PULONG Length
					 )
{
	ULONG retValue;
	if (!Length)
		return NOK;
	retValue = UtilGetParam(pAdapter, CTRL_DATA_CURRENT_IBSS_PROTECTION_PARAM, pData, sizeof(ULONG));
	*Length = sizeof(ULONG);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilIbssProtectionSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilIbssProtectionSet(
					 PTIWLN_ADAPTER_T pAdapter,
					 PUCHAR pData,
					 ULONG Length
					 )
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, CTRL_DATA_CURRENT_IBSS_PROTECTION_PARAM, pData, sizeof(ULONG));
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilShortSlotGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilShortSlotGet(
				PTIWLN_ADAPTER_T pAdapter,
				PUCHAR pData,
				PULONG Length
				)
{
	ULONG retValue;

	if (!Length)
		return NOK;

	retValue = UtilGetParam(pAdapter, SITE_MGR_DESIRED_SLOT_TIME_PARAM, pData, sizeof(ULONG));
	*Length = sizeof (ULONG);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilShortSlotSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilShortSlotSet(
				PTIWLN_ADAPTER_T pAdapter,
				PUCHAR pData,
				ULONG Length
				)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, SITE_MGR_DESIRED_SLOT_TIME_PARAM, pData, sizeof(ULONG));
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilExtRatesIeGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilExtRatesIeGet(
				 PTIWLN_ADAPTER_T pAdapter,
				 PUCHAR pData,
				 PULONG Length
				 )
{
	ULONG retValue;

	if (!Length)
		return NOK;

	retValue = UtilGetParam(pAdapter, SITE_MGR_USE_DRAFT_NUM_PARAM, pData, sizeof(ULONG));

	*Length = sizeof(ULONG);

	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilExtRatesIeSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilExtRatesIeSet(
				 PTIWLN_ADAPTER_T pAdapter,
				 PUCHAR pData,
				 ULONG Length
				 )
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, SITE_MGR_USE_DRAFT_NUM_PARAM, pData, sizeof(ULONG));
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilQosSetParams
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilQosSetParams(PTIWLN_ADAPTER_T pAdapter,
					   PUCHAR pData,
					   ULONG Length)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, QOS_MNGR_SET_OS_PARAMS, pData, Length);
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name:	UtilQosSetParams
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilQosSetRxTimeOut(PTIWLN_ADAPTER_T pAdapter,
						  PUCHAR pData,
						  ULONG Length)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, QOS_SET_RX_TIME_OUT, pData, Length);
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name:	UtilSetDTagToAcMappingTable
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilSetDTagToAcMappingTable(PTIWLN_ADAPTER_T pAdapter,
						  		  PUCHAR pData,
						  		  ULONG Length)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, TX_DATA_TAG_TO_AC_CLASSIFIER_TABLE, pData, Length);
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name:	UtilSetVAD
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilSetVAD(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, TX_DATA_SET_VAD, pData, Length);
	return retValue;
}

ULONG UtilGetVAD (PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length)
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, TX_DATA_GET_VAD , pData, sizeof(txDataVadTimerParams_t));
	*Length = sizeof(txDataVadTimerParams_t);
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name: UtilRemoveClassifierEntry
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilRemoveClassifierEntry(PTIWLN_ADAPTER_T pAdapter,
								PUCHAR ioBuffer,
								ULONG inBufLen)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, CTRL_DATA_CLSFR_REMOVE_ENTRY,ioBuffer, inBufLen);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilConfigTxClassifier
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilConfigTxClassifier(PTIWLN_ADAPTER_T pAdapter,
							 PUCHAR ioBuffer,
							 ULONG inBufLen)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, CTRL_DATA_CLSFR_CONFIG,ioBuffer, inBufLen);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilGetClsfrType
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilGetClsfrType(PTIWLN_ADAPTER_T pAdapter,
					   PUCHAR pData,
					   PULONG Length)
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, CTRL_DATA_CLSFR_TYPE , pData, sizeof(clsfrTypeAndSupport));
	*Length = sizeof(clsfrTypeAndSupport);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilGetAPQosParams
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilGetAPQosParams(PTIWLN_ADAPTER_T pAdapter,
						 PUCHAR pData,
						 PULONG Length)
{
	ULONG retValue;
	retValue = UtilSetGetParam(pAdapter, QOS_MNGR_AP_QOS_PARAMETERS , pData, sizeof(OS_802_11_AC_QOS_PARAMS));
	*Length = sizeof(OS_802_11_AC_QOS_PARAMS);
	return(retValue); 
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilGetAPQosCapabilities
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilGetAPQosCapabilities(PTIWLN_ADAPTER_T pAdapter,
							   PUCHAR pData,
							   PULONG Length)
{
	ULONG retValue;
	*Length=sizeof(OS_802_11_AP_QOS_CAPABILITIES_PARAMS);
	retValue = UtilGetParam(pAdapter, SITE_MGR_GET_AP_QOS_CAPABILITIES , pData, sizeof(OS_802_11_AP_QOS_CAPABILITIES_PARAMS));
	return(retValue);
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilAddTspec
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilAddTspec(PTIWLN_ADAPTER_T pAdapter,
				   PUCHAR pData,
				   ULONG Length)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, QOS_MNGR_ADD_TSPEC_REQUEST , pData, sizeof(OS_802_11_QOS_TSPEC_PARAMS));
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilGetTspecParams
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilGetTspecParams(PTIWLN_ADAPTER_T pAdapter,
						 PUCHAR pData,
						 PULONG Length)
{
	ULONG retValue;
	retValue = UtilSetGetParam(pAdapter, QOS_MNGR_OS_TSPEC_PARAMS , pData, sizeof(OS_802_11_QOS_TSPEC_PARAMS));
	*Length = sizeof(OS_802_11_QOS_TSPEC_PARAMS);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilDeleteTspec
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilDeleteTspec(PTIWLN_ADAPTER_T pAdapter,
					  PUCHAR pData,
					  ULONG Length)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, QOS_MNGR_DEL_TSPEC_REQUEST , pData, sizeof(OS_802_11_QOS_DELETE_TSPEC_PARAMS));
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilGetCurrentAcStatus
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilGetCurrentAcStatus(PTIWLN_ADAPTER_T pAdapter,
							 PUCHAR pData,
							 PULONG Length)
{
	ULONG retValue;
	retValue = UtilSetGetParam(pAdapter, QOS_MNGR_AC_STATUS , pData, sizeof(OS_802_11_AC_UPSD_STATUS_PARAMS));
	*Length = sizeof(OS_802_11_AC_UPSD_STATUS_PARAMS);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilGetUserPriorityOfStream
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilGetUserPriorityOfStream(PTIWLN_ADAPTER_T pAdapter,
								  PUCHAR pData,
								  PULONG Length)
{
	ULONG retValue;
	retValue = UtilSetGetParam(pAdapter, CTRL_DATA_GET_USER_PRIORITY_OF_STREAM , pData, sizeof(STREAM_TRAFFIC_PROPERTIES));
	*Length = sizeof(STREAM_TRAFFIC_PROPERTIES);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilSetMediumUsageThreshold
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilSetMediumUsageThreshold(PTIWLN_ADAPTER_T pAdapter,
								  PUCHAR pData,
								  ULONG Length)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, TX_DATA_SET_MEDIUM_USAGE_THRESHOLD , pData, sizeof(OS_802_11_THRESHOLD_CROSS_PARAMS));
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilSetPhyRateThreshold
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilSetPhyRateThreshold(PTIWLN_ADAPTER_T pAdapter,
							  PUCHAR pData,
							  ULONG Length)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, QOS_SET_RATE_THRESHOLD , pData, sizeof(OS_802_11_THRESHOLD_CROSS_PARAMS));
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilGetMediumUsageThreshold
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilGetMediumUsageThreshold(PTIWLN_ADAPTER_T pAdapter,
								  PUCHAR pData,
								  PULONG Length)
{
	ULONG retValue;
	retValue = UtilSetGetParam(pAdapter, TX_DATA_GET_MEDIUM_USAGE_THRESHOLD , pData, sizeof(OS_802_11_THRESHOLD_CROSS_PARAMS));
	*Length = sizeof(OS_802_11_THRESHOLD_CROSS_PARAMS);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilGetPhyRateThreshold
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilGetPhyRateThreshold(PTIWLN_ADAPTER_T pAdapter,
							  PUCHAR pData,
							  PULONG Length)
{
	ULONG retValue;
	retValue = UtilSetGetParam(pAdapter, QOS_GET_RATE_THRESHOLD , pData, sizeof(OS_802_11_THRESHOLD_CROSS_PARAMS));
	*Length = sizeof(OS_802_11_THRESHOLD_CROSS_PARAMS);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilGetDesiredPsMode
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilGetDesiredPsMode(PTIWLN_ADAPTER_T pAdapter,
						   PUCHAR pData,
						   PULONG  Length)
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, QOS_MNGR_GET_DESIRED_PS_MODE , pData, sizeof(OS_802_11_QOS_DESIRED_PS_MODE));
	*Length = sizeof(OS_802_11_QOS_DESIRED_PS_MODE);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilPollApPackets
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilPollApPackets(PTIWLN_ADAPTER_T pAdapter,
						PUCHAR pData,
						ULONG Length)
{
	ULONG retValue;
	retValue = configMgr_PollApPackets(pAdapter->CoreHalCtx);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilPollApPacketsFromAC
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilPollApPacketsFromAC(PTIWLN_ADAPTER_T pAdapter,
							  PUCHAR pData,
							  ULONG Length)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, TX_DATA_POLL_AP_PACKETS_FROM_AC , (unsigned char *)pData, Length);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilEnableEvent
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilEnableEvent(PTIWLN_ADAPTER_T pAdapter,
					  PUCHAR pData,
					  ULONG Length)
{
	/*UtilSetParam(pAdapter,  , pData, Length);  EITAN TBD */
	return PARAM_NOT_SUPPORTED;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilDisableEvent
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilDisableEvent(PTIWLN_ADAPTER_T pAdapter,
					   PUCHAR pData,
					   ULONG Length)
{
	/*UtilSetParam(pAdapter,  , pData, Length);  EITAN TBD */
	return PARAM_NOT_SUPPORTED;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilConfigRSSI
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilConfigRSSI(PTIWLN_ADAPTER_T pAdapter,
					 UINT32 pData,
					 ULONG Length)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, SITE_MGR_DESIRED_RSSI_THRESHOLD_SET , (unsigned char *)pData, Length);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilConfigPERLevel
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilConfigPERLevel(PTIWLN_ADAPTER_T pAdapter,
						 UINT32 pData,
						 ULONG Length)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, SITE_MGR_DESIRED_TX_RATE_PRCT_SET , (unsigned char *)pData, Length);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilGetDrvCapabilities
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilGetDrvCapabilities(PTIWLN_ADAPTER_T pAdapter,
							 PUCHAR pData,
							 PULONG Length)
{
	/*UtilSetParam(pAdapter,  , pData, Length);  EITAN TBD */
	return PARAM_NOT_SUPPORTED;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilGetPrimaryBSSIDInfo
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilGetPrimaryBSSIDInfo(PTIWLN_ADAPTER_T pAdapter,
                       PUCHAR pData,
                       PULONG Length)
{
	paramInfo_t Param;
	TI_STATUS Status;

	if ( *Length < sizeof(OS_802_11_BSSID_EX) )
    {
		PRINTF(DBG_NDIS_OIDS_ERROR, ("UtilGetPrimaryBSSIDInfo: ERROR Length is:%ld < %d\n", 
                                         *Length, sizeof(OS_802_11_BSSID)) );
        return NOK;
    }

    Param.paramType = SITE_MGR_PRIMARY_SITE_PARAM;
	Param.paramLength = *Length;
	Param.content.pSiteMgrSelectedSiteInfo = (OS_802_11_BSSID_EX*)pData;
    Status = configMgr_getParam(pAdapter->CoreHalCtx, &Param);

	if(Status != OK) {
		PRINTF(DBG_NDIS_OIDS_ERROR, (" UtilGetPrimaryBSSIDInfo: ERROR on return from get param SITE_MGR_PRIMARY_SITE_PARAM\n"));
    }
    else
    {
        *Length = Param.paramLength;    
    }

	return Status;
}

/*-----------------------------------------------------------------------------
Routine Name:	UtilGetSelectedBSSIDInfo
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilGetSelectedBSSIDInfo(PTIWLN_ADAPTER_T pAdapter,
							   PUCHAR pData,
							   PULONG Length)
{
	paramInfo_t Param;
	TI_STATUS Status;

	if ( *Length < sizeof(OS_802_11_BSSID_EX) )
	{
		PRINTF(DBG_NDIS_OIDS_ERROR, ("UtilGetSelectedBSSIDInfo: ERROR Length is:%ld < %d", 
										 *Length, sizeof(OS_802_11_BSSID)) );
		return NOK;
	}

	Param.paramType = SITE_MGR_GET_SELECTED_BSSID_INFO;
	Param.paramLength = *Length;
	Param.content.pSiteMgrPrimarySiteDesc = (OS_802_11_BSSID*)pData;
	Status = configMgr_getParam(pAdapter->CoreHalCtx, &Param);

	if(Status != OK)
        {
		PRINTF(DBG_NDIS_OIDS_ERROR, (" UtilGetSelectedBSSIDInfo: ERROR on return from get param SITE_MGR_GET_SELECTED_BSSID_INFO"));
	}

	return Status;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilGetDriverState
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilGetDriverState (PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG Length)
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, SME_SM_STATE_PARAM , pData, *Length);
	*Length = sizeof (ULONG);
	return retValue;
}


/*#ifdef NDIS51_MINIPORT*/

/*-----------------------------------------------------------------------------
Routine Name:	UtilPrivacyFilterGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilPrivacyFilterGet(
					PTIWLN_ADAPTER_T pAdapter,
					PUCHAR pData,
					PULONG Length
					)
{
	ULONG WepStatus,retValue,dataSize;

	if (!Length)
		return NOK;

	dataSize = sizeof (ULONG);
	retValue = UtilWepStatusGet(pAdapter, (PUCHAR)&WepStatus, &dataSize);

	if (WepStatus)
	{
		*(PULONG)pData = os802_11PrivFilterAcceptAll;
	} else
	{
		*(PULONG)pData = os802_11PrivFilter8021xWEP;
	}
	*Length = sizeof (ULONG);

	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilPrivacyFilterSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilPrivacyFilterSet(
					PTIWLN_ADAPTER_T pAdapter,
					PUCHAR pData,
					ULONG Length
					)
{
	ULONG WepStatus,dataSize,retValue;

	dataSize = sizeof(ULONG);
	retValue = UtilWepStatusGet(pAdapter, (PUCHAR)&WepStatus, &dataSize);

	if ((WepStatus && (*(PULONG)pData == os802_11PrivFilter8021xWEP)) || (retValue != OK))
		return NOK;	/* was return -1 */

	if ((!WepStatus) && (retValue == OK))
	{
		*(PULONG)pData = 0;
		retValue = UtilSetParam(pAdapter, RX_DATA_EXCLUDE_UNENCRYPTED_PARAM, pData, sizeof(ULONG));
	}

	return retValue;
}



/*#endif*/

/*-----------------------------------------------------------------------------
Routine Name:	UtilReadReg
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilReadReg(
		   PTIWLN_ADAPTER_T pAdapter,
		   PUCHAR pData,
		   PULONG Length
		   )
{
	TIWLN_REG_RW * pReg;

	pReg = (TIWLN_REG_RW *) pData;

#if defined(TNETW1150)
	if (pReg->regAddr >= 0x3C0000)
		pReg->regValue = configMgr_ReadPhyRegister(pAdapter->CoreHalCtx, pReg->regAddr);
	else
		pReg->regValue = configMgr_ReadMacRegister(pAdapter->CoreHalCtx, pReg->regAddr);
#else
	if (pReg->regAddr >= 0x1000)
		pReg->regValue = configMgr_ReadPhyRegister(pAdapter->CoreHalCtx, pReg->regAddr);
	else
		pReg->regValue = configMgr_ReadMacRegister(pAdapter->CoreHalCtx, pReg->regAddr);
#endif


#ifdef __LINUX__
	print_info("Register %#x=%#x(%d)\n", pReg->regAddr, pReg->regValue, pReg->regValue );
#endif
	*Length = sizeof(TIWLN_REG_RW);
	return OK;
}

/*-----------------------------------------------------------------------------
Routine Name:	UtilWriteReg
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilWriteReg(
			PTIWLN_ADAPTER_T pAdapter,
			PUCHAR pData,
			ULONG Length
			)
{
	TIWLN_REG_RW * pReg;

	pReg = (TIWLN_REG_RW *) pData;

#if defined(TNETW1150)
	if (pReg->regAddr >= 0x3C0000)
		configMgr_WritePhyRegister(pAdapter->CoreHalCtx, pReg->regAddr, pReg->regValue);
	else
		configMgr_WriteMacRegister(pAdapter->CoreHalCtx, pReg->regAddr, pReg->regValue);
#else
	if (pReg->regAddr >= 0x1000)
		configMgr_WritePhyRegister(pAdapter->CoreHalCtx, pReg->regAddr, pReg->regValue);
	else
		configMgr_WriteMacRegister(pAdapter->CoreHalCtx, pReg->regAddr, pReg->regValue);
#endif

	return OK;
}



/*-----------------------------------------------------------------------------
Routine Name:	UtilDisassociate
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilDisassociate(
				PTIWLN_ADAPTER_T pAdapter,
				PUCHAR pData,
				ULONG Length
				)
{
	OS_802_11_SSID FakeSsid;
	UINT32 loopIndex;
	ULONG retValue;

	/*
	 * Clean up desired SSID value
	*/
	for (loopIndex = 0; loopIndex < MAX_SSID_LEN; loopIndex++)
		FakeSsid.Ssid[loopIndex] = (loopIndex+1);

	FakeSsid.SsidLength = MAX_SSID_LEN;

	retValue = UtilSetSSID(pAdapter, (PUCHAR)&FakeSsid, sizeof(OS_802_11_SSID));

	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name:	UtilInfoCodeQueryInformation
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilInfoCodeQueryInformation(
							PTIWLN_ADAPTER_T pAdapter,
							PUCHAR pData,
							PULONG Length
							)
{
	UINT32 InfoCode, retVal, PureInfoLength;
	ULONG dataSize;
	PRINT(DBG_IOCTL_LOUD, "UtilInfoCodeQueryInformation\n");

	retVal = OK;

	if (*Length<sizeof(InfoCode))
	{
		*Length = sizeof(ULONG);
		return NOK;
	}

	InfoCode = *((UINT32*)pData);
	#ifndef _WINDOWS
		PureInfoLength = *Length - sizeof(InfoCode);
	#else 
	#endif

	switch (InfoCode)
	{
	case VAL_TX_POWER_VALUE:
		PRINT(DBG_IOCTL_LOUD, "case VAL_TX_POWER_VALUE (100)\n");
		retVal = UtilGetTxPowerValue(pAdapter, REGULATORY_DOMAIN_CURRENT_TX_POWER_IN_DBM_PARAM, pData, PureInfoLength);
		break;
	case VAL_NETWORK_TYPE:
		PRINT(DBG_IOCTL_LOUD, "case VAL_NETWORK_TYPE (101)\n");
		dataSize = PureInfoLength;
		retVal = UtilNetworkTypeInUseGet(pAdapter, pData, &dataSize);
		break;
	case VAL_AP_TX_POWER_LEVEL:
		PRINT(DBG_IOCTL_LOUD, "case VAL_AP_TX_POWER_LEVEL (102)\n");
		retVal = UtilGetAPTxPowerLevel(pAdapter, SITE_MGR_AP_TX_POWER_PARAM, pData, PureInfoLength);
		break;
	case VAL_PACKET_BURSTING:
		PRINT(DBG_IOCTL_LOUD, "case VAL_PACKET_BURSTING (106)\n");
		retVal = UtilGetPacketBursting(pAdapter, QOS_PACKET_BURST_ENABLE, pData, PureInfoLength);
		break;
	case VAL_MIXED_MODE:
		dataSize = PureInfoLength;
		retVal = UtilGetMixedMode(pAdapter, RSN_MIXED_MODE, pData, &dataSize);
		break;
	case VAL_DEFAULT_KEY_ID:
		PRINT(DBG_IOCTL_LOUD, "case VAL_DEFAULT_KEY_ID (110)\n");
		dataSize = PureInfoLength;
		retVal = UtilGetDefaultKeyId(pAdapter, RSN_DEFAULT_KEY_ID, pData, &dataSize);
		break;
	default:
		PRINT(DBG_IOCTL_LOUD, "case default\n");
		break;
	}
	*Length = PureInfoLength;
	return retVal;
}

/*-----------------------------------------------------------------------------
Routine Name:	UtilInfoCodeSetInformation
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilInfoCodeSetInformation(
						  PTIWLN_ADAPTER_T pAdapter,
						  PUCHAR pData,
						  ULONG Length
						  )
{
	UINT32 InfoCode, retVal, PureInfoLength;

	PRINT(DBG_IOCTL_LOUD, "UtilInfoCodeSetInformation\n");

	if (Length<sizeof(UINT32))
		return NOK;

	InfoCode = *((UINT32*)pData);
	retVal = PureInfoLength = Length - sizeof(InfoCode);

	switch (InfoCode)
	{
	case VAL_TX_POWER_VALUE:
		PRINT(DBG_IOCTL_LOUD, "case VAL_TX_POWER_VALUE (100)\n");
		retVal = UtilSetParam(pAdapter, REGULATORY_DOMAIN_CURRENT_TX_POWER_IN_DBM_PARAM, pData+sizeof(InfoCode), PureInfoLength);
		break;
	case VAL_NETWORK_TYPE:
		PRINT(DBG_IOCTL_LOUD, "case VAL_NETWORK_TYPE (101)\n");
		retVal = UtilNetworkTypeInUseSet(pAdapter, pData+sizeof(InfoCode), PureInfoLength);
		break;
	case VAL_PACKET_BURSTING:
		PRINT(DBG_IOCTL_LOUD, "case VAL_PACKET_BURSTING (106)\n");
		retVal = UtilSetPacketBursting(pAdapter, pData+sizeof(InfoCode), PureInfoLength);
		break;
	case VAL_MIXED_MODE:
		retVal = UtilSetMixedMode(pAdapter, pData+sizeof(InfoCode), PureInfoLength);
		break;
	case VAL_DEFAULT_KEY_ID:
		retVal = UtilSetParam(pAdapter, RSN_DEFAULT_KEY_ID, pData+sizeof(InfoCode), PureInfoLength);
		break;
	default:
		PRINT(DBG_IOCTL_LOUD, "case default\n");
		break;
	}
	return retVal;
}

#ifdef _WINDOWS
#endif /* _WINDOWS */

/*-----------------------------------------------------------------------------
Routine Name:	UtilTxPowerLevelDbmGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilTxPowerLevelDbmGet(
					  PTIWLN_ADAPTER_T pAdapter,
					  PUCHAR pData,
					  PULONG Length
					  )
{ 
	ULONG retValue;

	if (!Length)
		return NOK;

	retValue = UtilGetParam(pAdapter, REGULATORY_DOMAIN_CURRENT_TX_POWER_IN_DBM_PARAM, pData, sizeof(UINT8));
	*Length = sizeof(INT8);

	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilTxPowerLevelDbmSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilTxPowerLevelDbmSet(
					  PTIWLN_ADAPTER_T pAdapter,
					  PUCHAR pData,
					  ULONG Length
					  )
{ 
ULONG mW, Dbm, power,retValue;

	if (!Length)
		return NOK;

	mW = *(PULONG)pData;

	for (power=1; mW/10; mW/=10, power++); 

	Dbm = 20 * power;


	retValue = UtilSetParam(pAdapter, REGULATORY_DOMAIN_CURRENT_TX_POWER_IN_DBM_PARAM, (PUCHAR)&Dbm, sizeof(ULONG));
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name:	Util802EapTypeGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
Util802EapTypeGet(
				 PTIWLN_ADAPTER_T pAdapter,
				 PUCHAR pData,
				 PULONG Length
				 )
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, RSN_EAP_TYPE, pData, sizeof(ULONG));
	*Length = sizeof(ULONG);
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name:	Util802EapTypeSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
Util802EapTypeSet(
				 PTIWLN_ADAPTER_T pAdapter,
				 PUCHAR pData,
				 ULONG Length
				 )
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, RSN_EAP_TYPE, pData, sizeof(ULONG));
	return retValue;
}

#ifdef EXC_MODULE_INCLUDED

/*-----------------------------------------------------------------------------
Routine Name:	UtilExcConfigurationGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilExcConfigurationGet(
					   PTIWLN_ADAPTER_T pAdapter,
					   PUCHAR pData,
					   PULONG Length
					   )
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, EXC_CONFIGURATION, pData, sizeof(ULONG));
	*Length = sizeof(ULONG);
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name:	UtilExcNetworkEapGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilExcNetworkEapGet(
					PTIWLN_ADAPTER_T pAdapter,
					PUCHAR pData,
					PULONG Length
					)
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, RSN_EXC_NETWORK_EAP, pData, sizeof(ULONG));
	*Length = sizeof(ULONG);
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name:	UtilExcConfigurationSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilExcConfigurationSet(
					   PTIWLN_ADAPTER_T pAdapter,
					   PUCHAR pData,
					   ULONG Length
					   )
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, EXC_CONFIGURATION, pData, sizeof(ULONG));
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name:	UtilExcNetworkEapSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilExcNetworkEapSet(
					PTIWLN_ADAPTER_T pAdapter,
					PUCHAR pData,
					ULONG Length
					)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, RSN_EXC_NETWORK_EAP, pData, sizeof(ULONG));
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name: UtilExcRogueApDetectedSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilExcRogueApDetectedSet(
						 PTIWLN_ADAPTER_T pAdapter,
						 PUCHAR pData,
						 ULONG Length
						 )
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, EXC_ROGUE_AP_DETECTED, pData, sizeof(OS_EXC_ROGUE_AP_DETECTED));
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name:	UtilExcReportRogueApSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilExcReportRogueApSet(
					   PTIWLN_ADAPTER_T pAdapter,
					   PUCHAR pData,
					   ULONG Length
					   )
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, EXC_REPORT_ROGUE_APS, pData, 0);
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name:	UtilExcAuthSuccessSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilExcAuthSuccessSet(
					 PTIWLN_ADAPTER_T pAdapter,
					 PUCHAR pData,
					 ULONG Length
					 )
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, EXC_AUTH_SUCCESS, pData, sizeof(OS_EXC_AUTH_SUCCESS));
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilExcCckmRequestSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilExcCckmRequestSet(
					 PTIWLN_ADAPTER_T pAdapter,
					 PUCHAR pData,
					 ULONG Length
					 )
{
	ULONG retValue;

	OS_EXC_CCKM_REQUEST *cckmRequest = (OS_EXC_CCKM_REQUEST*)pData;
	ULONG reqLength = cckmRequest->AssociationRequestIELength+sizeof(cckmRequest->RequestCode)+sizeof(cckmRequest->AssociationRequestIELength);

	PRINTF(DBG_IOCTL_LOUD, ("UtilExcCckmRequestSet, In Length = %d, Required Length=%d\n",
							(int) Length, (int) reqLength));

	if ((cckmRequest==NULL) || (reqLength > Length))
	{
		PRINTF(DBG_IOCTL_LOUD, ("UtilExcCckmRequestSet, wrong size or pointer, In Length = %d, Required Length=%d\n",
								(int) Length, (int) reqLength));
		return NOK;
	}
	retValue = UtilSetParam(pAdapter, EXC_CCKM_REQUEST, pData, reqLength);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilExcCckmResultSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilExcCckmResultSet(
					PTIWLN_ADAPTER_T pAdapter,
					PUCHAR pData,
					ULONG Length
					)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, EXC_CCKM_RESULT, pData, sizeof(ULONG));
	return retValue;
}

#endif /* EXC_MODULE_INCLUDED */


/*-----------------------------------------------------------------------------
Routine Name: UtilGetMACAddress
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilGetMACAddress(
				 PTIWLN_ADAPTER_T pAdapter,
				 PUCHAR pData,
				 PULONG Length
				 )
{
	ULONG retValue;

	if ((*Length) < MAC_ADDR_LEN)
	{
		*Length = MAC_ADDR_LEN;
		return NOK;
	}

	retValue = UtilGetParam(pAdapter, CTRL_DATA_MAC_ADDRESS, pData, MAC_ADDR_LEN);
	*Length = MAC_ADDR_LEN;
	return retValue;
}     


/*-----------------------------------------------------------------------------
Routine Name:	UtilConfigRoamingParamsSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilConfigRoamingParamsSet(
						  PTIWLN_ADAPTER_T pAdapter,
						  PUCHAR pData,
						  ULONG Length
						  )
{
	ULONG retValue;
	applicationConfigBuffer_t applicationConfigBuffer;

	applicationConfigBuffer.buffer = pData;
	applicationConfigBuffer.bufferSize = (UINT16)Length;

	retValue = UtilSetParam(pAdapter, ROAMING_MNGR_APPLICATION_CONFIGURATION, (PUCHAR)&applicationConfigBuffer, sizeof(applicationConfigBuffer_t));

	return retValue;
}     

/*-----------------------------------------------------------------------------
Routine Name:	UtilConfigRoamingParamsGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilConfigRoamingParamsGet(
						  PTIWLN_ADAPTER_T pAdapter,
						  PUCHAR pData,
						  PULONG Length
						  )
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, ROAMING_MNGR_APPLICATION_CONFIGURATION, pData, *Length);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilMeasurementEnableDisableParamsSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilMeasurementEnableDisableParamsSet(
									 PTIWLN_ADAPTER_T pAdapter,
									 PUCHAR pData,
									 ULONG Length
									 )
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, MEASUREMENT_ENABLE_DISABLE_PARAM, pData, Length);
	return retValue;
}     

/*-----------------------------------------------------------------------------
Routine Name:	UtilMeasurementMaxDurationParamsSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilMeasurementMaxDurationParamsSet(
								   PTIWLN_ADAPTER_T pAdapter,
								   PUCHAR pData,
								   ULONG Length
								   )
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, MEASUREMENT_MAX_DURATION_PARAM, pData, sizeof(UINT32));
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name:	UtilEarlyWakeupIeGet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilEarlyWakeupIeGet(
				 PTIWLN_ADAPTER_T pAdapter,
				 PUCHAR pData,
				 PULONG Length
				 )
{
	ULONG retValue;

	if (!Length)
		return NOK;

	retValue = UtilGetParam(pAdapter, HAL_CTRL_EARLY_WAKEUP, pData, sizeof(UINT8));

	*Length = sizeof(UINT8);

	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilEarlyWakeupIeSet
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilEarlyWakeupIeSet(
				 PTIWLN_ADAPTER_T pAdapter,
				 PUCHAR pData,
				 ULONG Length
				 )
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, HAL_CTRL_EARLY_WAKEUP, pData, sizeof(ULONG));
	return retValue;
}



/*-----------------------------------------------------------------------------
Routine Name:	UtilBthWlanCoeEnable
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilBthWlanCoeEnable(
					PTIWLN_ADAPTER_T pAdapter,
					PUCHAR pData,
					ULONG Length
					)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, SOFT_GEMINI_SET_ENABLE, pData, sizeof(ULONG));
	return retValue;
}     

/*-----------------------------------------------------------------------------
Routine Name:	UtilBthWlanCoeRate
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilBthWlanCoeRate(
				  PTIWLN_ADAPTER_T pAdapter,
				  PUCHAR pData,
				  ULONG Length
				  )
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, SOFT_GEMINI_SET_RATE, pData, sizeof(ULONG)*NUM_OF_RATES_IN_SG);
	return retValue;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilBthWlanCoeConfig
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilBthWlanCoeConfig(
					PTIWLN_ADAPTER_T pAdapter,
					PUCHAR pData,
					ULONG Length
					)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, SOFT_GEMINI_SET_CONFIG, pData, sizeof(ULONG) * NUM_OF_CONFIG_PARAMS_IN_SG);
	return retValue;
}    


/*-----------------------------------------------------------------------------
Routine Name:	UtilBthWlanCoeGetStatus
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilBthWlanCoeGetStatus(
					   PTIWLN_ADAPTER_T pAdapter,
					   PUCHAR pData,
					   PULONG Length
					   )
{
	ULONG retValue;
	retValue = UtilGetParam(pAdapter, SOFT_GEMINI_GET_STATUS, pData, 0);
	*Length = 0;
	return retValue;
} 


/*-----------------------------------------------------------------------------
Routine Name: UtilPltRxPerStart
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilPltRxPerStart(
				 PTIWLN_ADAPTER_T pAdapter,
				 PUCHAR pData,
				 ULONG Length
				 )
{
	ULONG Status;
	interogateCmdCBParams_t interogateCmdCBParams;
	memset(&(pAdapter->IoCompleteBuff[0]) , 0xFF , MAX_IO_BUFFER_COMPLETE_SIZE );

	/* Fill the IOCTL struct to the Command Mailbox by giving a stack parameter */
	interogateCmdCBParams.CB_handle  =  (TI_HANDLE)pAdapter;
	interogateCmdCBParams.CB_Func    =  NULL;
	interogateCmdCBParams.CB_buf     =  NULL;
	Status = UtilSetParam(pAdapter, HAL_CTRL_PLT_RX_PER_START, (PUCHAR)&interogateCmdCBParams, sizeof(interogateCmdCBParams));

	return Status;
}

/*-----------------------------------------------------------------------------
Routine Name: UtilPltRxPerStop
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilPltRxPerStop(
				PTIWLN_ADAPTER_T pAdapter,
				PUCHAR pData,
				ULONG Length
				)
{
	ULONG Status;
	interogateCmdCBParams_t interogateCmdCBParams;

	/* Fill the IOCTL struct to the Command Mailbox by giving a stack parameter */
	interogateCmdCBParams.CB_handle  =  (TI_HANDLE)pAdapter;
	interogateCmdCBParams.CB_Func    =  NULL;
	interogateCmdCBParams.CB_buf     =  NULL;
	Status = UtilSetParam(pAdapter, HAL_CTRL_PLT_RX_PER_STOP, (PUCHAR)&interogateCmdCBParams, sizeof(interogateCmdCBParams));
	return Status;
}

/*-----------------------------------------------------------------------------
Routine Name: UtilPltRxPerClear
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilPltRxPerClear(
				 PTIWLN_ADAPTER_T pAdapter,
				 PUCHAR pData,
				 ULONG Length
				 )
{
	ULONG Status;
	interogateCmdCBParams_t interogateCmdCBParams;
	memset(&(pAdapter->IoCompleteBuff[0]) , 0xFF , MAX_IO_BUFFER_COMPLETE_SIZE );

	/* Fill the IOCTL struct to the Command Mailbox by giving a stack parameter */
	interogateCmdCBParams.CB_handle  =  (TI_HANDLE)pAdapter;
	interogateCmdCBParams.CB_Func    =  NULL;
	interogateCmdCBParams.CB_buf     =  NULL;
	Status = UtilSetParam(pAdapter, HAL_CTRL_PLT_RX_PER_CLEAR, (PUCHAR)&interogateCmdCBParams, sizeof(interogateCmdCBParams));
	return Status;
}

/*-----------------------------------------------------------------------------
Routine Name: UtilPltRxPerGetResults
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilPltRxPerGetResults(
					  PTIWLN_ADAPTER_T pAdapter,
					  PUCHAR pData,
					  PULONG pLength
					  )
{

	ULONG Status;
	paramInfo_t Param;
	interogateCmdCBParams_t* pInterogateCmdCBParams = &Param.content.interogateCmdCBParams;

	memset(&(pAdapter->IoCompleteBuff[0]) , 0xFF , MAX_IO_BUFFER_COMPLETE_SIZE );

	/* To implement the Async IOCTL store the user buffer pointer to be filled at
	the Command Completion calback */
	pAdapter->pIoBuffer =  pData;
	pAdapter->pIoCompleteBuffSize =  pLength ;


	/* Fill the IOCTL struct to the Command Mailbox by giving a stack parameter */
	pInterogateCmdCBParams->CB_handle  =  (TI_HANDLE)pAdapter;
	pInterogateCmdCBParams->CB_Func    =  (PVOID)UtilPltRxPerCB;
	pInterogateCmdCBParams->CB_buf     =  &(pAdapter->IoCompleteBuff[0]);

	Param.paramType = HAL_CTRL_PLT_RX_PER_GET_RESULTS;
	Param.paramLength = sizeof(interogateCmdCBParams_t);

	Status = configMgr_getParam(pAdapter->CoreHalCtx, &Param);

	return Status;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilPltRxPerCB

Routine Description: This is the CB triggered when  PltRX command been 
					returned by FW.
Arguments:

Return Value:
-----------------------------------------------------------------------------*/
VOID UtilPltRxPerCB(TI_HANDLE hAdapter,TI_STATUS status,PUINT8 pReadBuff)
{
	HwMboxCmdBit_RxPer_t* pRxPer =  (HwMboxCmdBit_RxPer_t* )pReadBuff;
	PTIWLN_ADAPTER_T pAdapter = (PTIWLN_ADAPTER_T)hAdapter;
	
	if (pRxPer->CB_RxPerCmd == PLT_RX_PER_GETRESULTS)
	{
		*(pAdapter->pIoCompleteBuffSize) = sizeof(pRxPer->PltRxPer);
		os_memoryCopy(hAdapter, (void*)&(pAdapter->pIoBuffer[0]) ,(void*) &(pRxPer->PltRxPer) , *(pAdapter->pIoCompleteBuffSize));
		/* Call back the Completion that will indicate to the user that the buffer is ready to be read */
		os_IoctlComplete(pAdapter, status);

	}
}

/*-----------------------------------------------------------------------------
Routine Name: UtilPltTxCW
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilPltTxCW(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, HAL_CTRL_PLT_TX_CW, pData, Length);
	return retValue;
}
/*-----------------------------------------------------------------------------
Routine Name: UtilPltTxContinues
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilPltTxContinues(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, HAL_CTRL_PLT_TX_CONTINUES, pData, Length);
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name: UtilPltTxStop
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilPltTxStop(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, HAL_CTRL_PLT_TX_STOP, pData, 0);
	return retValue;
}

/*-----------------------------------------------------------------------------
Routine Name: UtilPltWriteMib
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilPltWriteMib(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, ULONG Length)
{
	ULONG retValue;
	retValue = UtilSetParam(pAdapter, HAL_CTRL_PLT_WRITE_MIB, pData, Length);
	return retValue;
}
/*-----------------------------------------------------------------------------
Routine Name: UtilPltReadMib
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilPltReadMib(PTIWLN_ADAPTER_T pAdapter, PUCHAR pData, PULONG pOutLength, ULONG InLength)
{
	ULONG Status;
	paramInfo_t Param;
	
	/* To implement the Async IOCTL store the user buffer pointer to be filled at
	the Command Completion calback */
	pAdapter->pIoBuffer =  pData;
	pAdapter->pIoCompleteBuffSize =  pOutLength;

	os_memoryCopy((TI_HANDLE)pAdapter, (PVOID)pAdapter->IoCompleteBuff, pData, InLength);

	/* Fill the IOCTL struct to the Command Mailbox by giving a stack parameter */
	Param.content.interogateCmdCBParams.CB_handle  =  (TI_HANDLE)pAdapter;
	Param.content.interogateCmdCBParams.CB_Func  =  (PVOID)UtilPltReadMibCB;
	Param.content.interogateCmdCBParams.CB_buf  =  pAdapter->IoCompleteBuff;

	Param.paramType = HAL_CTRL_PLT_READ_MIB;
	Param.paramLength = sizeof(interogateCmdCBParams_t);
	Status = configMgr_getParam(pAdapter->CoreHalCtx, &Param);

	return Status;
}


/*-----------------------------------------------------------------------------
Routine Name:	UtilPltReadMibCB

Routine Description: This is the CB triggered when  PltReadRegister command been 
					returned by FW.
Arguments:

Return Value:
-----------------------------------------------------------------------------*/
VOID UtilPltReadMibCB(TI_HANDLE hAdapter,TI_STATUS status,PUINT8 pReadBuff)
{
	PTIWLN_ADAPTER_T pAdapter = (PTIWLN_ADAPTER_T)hAdapter;
	PLT_MIB_t* pReturnMib = (PLT_MIB_t*)pAdapter->pIoBuffer;
	tiBOOL IsAsync = TRUE;

	switch (pReturnMib->aMib)
	{
	case PLT_MIB_dot11StationId:
		{
			dot11StationIDStruct* pdot11StationID = (dot11StationIDStruct*)pReadBuff;
			pReturnMib->Length = sizeof(pReturnMib->aData.StationId);
			os_memoryCopy(hAdapter, (PVOID)pReturnMib->aData.StationId.addr, (PVOID)pdot11StationID->dot11StationID, sizeof(pReturnMib->aData.StationId));
		}         
		break;

	case PLT_MIB_countersTable:
		{
			ACXErrorCounters_t* pACXErrorCounters = (ACXErrorCounters_t*)pReadBuff;
			pReturnMib->Length = sizeof(pReturnMib->aData.CounterTable);
			pReturnMib->aData.CounterTable.FCSErrorCount = pACXErrorCounters->FCSErrorCount;
			pReturnMib->aData.CounterTable.PLCPErrorCount = pACXErrorCounters->PLCPErrorCount;
		}
		break;

		/* MIBs with data which is already in pReadBuff and in the correct form. */
	case PLT_MIB_ctsToSelf:
	case PLT_MIB_dot11MaxReceiveLifetime:
	case PLT_MIB_dot11GroupAddressesTable:
	case PLT_MIB_arpIpAddressesTable:
	case PLT_MIB_rxFilter:
	case PLT_MIB_templateFrame:
	case PLT_MIB_beaconFilterIETable:
	case PLT_MIB_txRatePolicy:
		IsAsync = FALSE;
		PRINTF(DBG_NDIS_OIDS_LOUD, ("UtilPltReadMibCB:MIB aMib 0x%x \n",pReturnMib->aMib));
		os_memoryCopy(hAdapter, (PVOID)pAdapter->pIoBuffer, (PVOID)pAdapter->IoCompleteBuff, sizeof(PLT_MIB_t));
		break;
	default:
		PRINTF(DBG_NDIS_OIDS_LOUD, ("UtilPltReadMibCB:MIB aMib 0x%x Not supported\n",pReturnMib->aMib));
	}
	/* Call back the Completion that will indicate to the user that the buffer is ready to be read */
	if (IsAsync)
		os_IoctlComplete(pAdapter, status);
}

/*-----------------------------------------------------------------------------
Routine Name: UtilPltReadRegister
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilPltReadRegister(
				   PTIWLN_ADAPTER_T pAdapter,
				   PUCHAR pData,
				   PULONG Length
				   )
{

	ULONG Status;
	paramInfo_t Param;
	interogateCmdCBParams_t* pInterogateCmdCBParams = &Param.content.interogateCmdCBParams;
	ReadWriteCommand_t* pReadWriteCommandStruct = (ReadWriteCommand_t*)pAdapter->IoCompleteBuff;
	UINT32* pRegAdress = (UINT32*)pData;

	/* To implement the Async IOCTL store the user buffer pointer to be filled at
	the Command Completion calback */
	pAdapter->pIoBuffer =  pData;
	pAdapter->pIoCompleteBuffSize =  Length ;

	memset(&(pAdapter->IoCompleteBuff[0]) , 0xFF , MAX_IO_BUFFER_COMPLETE_SIZE );
	pReadWriteCommandStruct->addr = *pRegAdress;
	pReadWriteCommandStruct->size = 4;

	/* Fill the IOCTL struct to the Command Mailbox by giving a stack parameter */
	pInterogateCmdCBParams->CB_handle  =  (TI_HANDLE)pAdapter;
	pInterogateCmdCBParams->CB_Func    =  (PVOID)UtilPltReadRegisterCB;
	pInterogateCmdCBParams->CB_buf     =  &(pAdapter->IoCompleteBuff[0]) ;

	Param.paramType = HAL_CTRL_PLT_READ_REGISTER;
	Param.paramLength = sizeof(interogateCmdCBParams_t);

	Status = configMgr_getParam(pAdapter->CoreHalCtx, &Param);
	return Status;
}

/*-----------------------------------------------------------------------------
Routine Name:	UtilPltReadRegisterCB

Routine Description: This is the CB triggered when  PltReadRegister command been 
					returned by FW.
Arguments:

Return Value:
-----------------------------------------------------------------------------*/
VOID UtilPltReadRegisterCB(TI_HANDLE hAdapter,TI_STATUS status,PUINT8 pReadBuff)
{
	PTIWLN_ADAPTER_T pAdapter = (PTIWLN_ADAPTER_T)hAdapter;
	ReadWriteCommand_t* pReadWriteCommandStruct = (ReadWriteCommand_t*)pReadBuff;
	UINT32* pRegDataReturn = (UINT32*)pAdapter->pIoBuffer;

	*(pAdapter->pIoCompleteBuffSize) = sizeof(INT32);

	/*Convert the returned data structure from ReadWriteCommandStruct to UINT32*/
	os_memoryCopy(hAdapter, (PVOID)pRegDataReturn, (PVOID)pReadWriteCommandStruct->value, sizeof(UINT32));

	/* Call back the Completion that will indicate to the user that the buffer is ready to be read */
	os_IoctlComplete(pAdapter, status);
}

/*-----------------------------------------------------------------------------
Routine Name: UtilPltWriteRegister
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG
UtilPltWriteRegister(
					PTIWLN_ADAPTER_T pAdapter,
					PUCHAR pData,
					ULONG Length
					)
{
	ReadWriteCommand_t* pReadWriteCommandStruct = (ReadWriteCommand_t*)pAdapter->IoCompleteBuff;
	TIWLN_REG_RW* pReg = (TIWLN_REG_RW*)pData;
	interogateCmdCBParams_t interogateCmdCBParams;
	UINT32 Status;


	pReadWriteCommandStruct->addr = pReg->regAddr;
	pReadWriteCommandStruct->size = pReg->regSize;
	os_memoryCopy((TI_HANDLE)pAdapter, (PVOID)pReadWriteCommandStruct->value, (PVOID)&pReg->regValue, pReadWriteCommandStruct->size);

	/* Fill the IOCTL struct to the Command Mailbox by giving a stack parameter */
	interogateCmdCBParams.CB_handle  =  (TI_HANDLE)pAdapter;
	interogateCmdCBParams.CB_Func    =  NULL;
	interogateCmdCBParams.CB_buf     =  pAdapter->IoCompleteBuff;
	Status = UtilSetParam(pAdapter, HAL_CTRL_PLT_WRITE_REGISTER, (PUCHAR)&interogateCmdCBParams, sizeof(interogateCmdCBParams));
	return Status;
}

/*-----------------------------------------------------------------------------
Routine Name: utilPltRxTxCal
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilPltRxTxCal(
				   PTIWLN_ADAPTER_T pAdapter,
				   PUCHAR pData,
				   PULONG pOutLength,
				   ULONG  InLength)

{
	paramInfo_t Param;
	interogateCmdCBParams_t* pInterogateCmdCBParams = &Param.content.interogateCmdCBParams;

	/* To implement the Async IOCTL store the user buffer pointer to be filled at
	the Command Completion callback */
	pAdapter->pIoBuffer =  (PUINT8)pData;
	pAdapter->pIoCompleteBuffSize =  pOutLength;

	/* Fill the IOCTL struct to the Command Mailbox by giving a stack parameter */
	pInterogateCmdCBParams->CB_handle  =  (TI_HANDLE)pAdapter;
	pInterogateCmdCBParams->CB_Func    =  (PVOID)UtilPltRxTxCalCB;
    pInterogateCmdCBParams->CB_buf     = pAdapter->pIoBuffer;

	Param.paramType = HAL_CTRL_PLT_RX_TX_CAL;
	Param.paramLength = sizeof(interogateCmdCBParams_t);

	return configMgr_getParam(pAdapter->CoreHalCtx, &Param);
}

/*-----------------------------------------------------------------------------
Routine Name:	UtilPltRxTxCalCB

Routine Description: This is the CB triggered when  utilPltRxTxCal command been 
					returned by FW.
Arguments:          pReadBuff - Should return the TestCmd_t

Return Value:
-----------------------------------------------------------------------------*/
VOID UtilPltRxTxCalCB(TI_HANDLE hAdapter,TI_STATUS status,PUINT8 pReadBuff)
{
	PTIWLN_ADAPTER_T pAdapter = (PTIWLN_ADAPTER_T)hAdapter;

    *(pAdapter->pIoCompleteBuffSize) = sizeof(TestCmd_t);

     /* Call back the Completion that will indicate to the user that the buffer is ready to be read */
	os_IoctlComplete(pAdapter, status);
}

/*-----------------------------------------------------------------------------
Routine Name: UtilPltRxCal
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG UtilPltRxCal(
                   PTIWLN_ADAPTER_T pAdapter,
                   PUCHAR pData,
                   PULONG pOutLength,
                   ULONG InLength)
{
    whalParamInfo_t Param;

    Param.paramType = HAL_CTRL_PLT_RX_TX_CAL;
    Param.paramLength = sizeof(TestCmd_t*);
    Param.content.interogateCmdCBParams.CB_Func = NULL;
    Param.content.interogateCmdCBParams.CB_handle = NULL;
    Param.content.interogateCmdCBParams.CB_buf = (PUINT8)pData;

    return configMgr_getParam(pAdapter->CoreHalCtx, (paramInfo_t*)&Param);
}

/*-----------------------------------------------------------------------------
Routine Name: utilRxCalibrationStatus
Routine Description:
Arguments:
Return Value:
-----------------------------------------------------------------------------*/
ULONG utilRxCalibrationStatus(
                   PTIWLN_ADAPTER_T pAdapter,
                   PUCHAR pData,
                   PULONG pOutLength,
                   ULONG  InLength)
{
    whalParamInfo_t Param;
    ULONG status;

    Param.paramType = HAL_CTRL_PLT_RX_CAL_STATUS;
    Param.paramLength = sizeof(TI_STATUS);

    status = configMgr_getParam(pAdapter->CoreHalCtx, (paramInfo_t*)&Param);

    *pData = Param.content.PltRxCalibrationStatus;

    return status;
}
