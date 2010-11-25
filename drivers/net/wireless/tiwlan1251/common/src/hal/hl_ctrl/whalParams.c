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
 *   MODULE:  whalParams.c
 *   PURPOSE: Holds all the whal parameters
 * 
 ****************************************************************************/

#include "whalCommon.h"
#include "whalHwDefs.h"
#include "802_11Defs.h"
#include "whalParams.h"
#include "whalBus_Api.h"

/******************************* definitions ********************************/
#define HW_MODULATION_PBCC	0x80
#define HW_MODULATION_OFDM	0x40 

/*
 * Internals
 */
void whal_ParamsInitDmaParams	 (WhalParams_T *pWhalParams);
int  whal_ParamsInitWlanParams	 (WhalParams_T *pWhalParams);
int  whal_ParamsInitBssInfoParams(WhalParams_T *pWhalParams, BOOL TxFlashEnable);
int  whal_ParamsInitGenParams	 (WhalParams_T *pWhalParams);
int  whal_ParamsInitHwInfoParams (WhalParams_T *pWhalParams);
int  whal_ParamsInitGenCounters	 (WhalParams_T *pWhalParams);
void whal_ParamsSetQidToAcTable  (WhalParams_T *pWhalParams,UINT8 Qid,UINT8 AcId);

/****************************************************************************
 *                      whal_params_Create()
 ****************************************************************************
 * DESCRIPTION:	Create the whal parameters(data base) object
 * 
 * INPUTS:	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	The Created object
 ****************************************************************************/
WhalParams_T *whal_params_Create(TI_HANDLE hOs, BOOL TxFlashEnable)
{
	WhalParams_T *pObj;

	pObj = os_memoryAlloc(hOs, sizeof(WhalParams_T));
	if (pObj == NULL)
		return NULL;

	os_memoryZero(hOs, (void *)pObj, sizeof(WhalParams_T));

	pObj->hOs = hOs;

	whal_ParamsInitDmaParams(pObj);
	whal_ParamsInitGenParams(pObj);
	whal_ParamsInitBssInfoParams(pObj,TxFlashEnable);
	whal_ParamsInitWlanParams(pObj);
	whal_ParamsInitGenCounters(pObj);

	return(pObj);
}

/****************************************************************************
 *                      whal_params_Destroy()
 ****************************************************************************
 * DESCRIPTION:	Destroy the object 
 * 
 * INPUTS:	
 *		WhalParams_T		The object to free
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_params_Destroy(WhalParams_T *pWhalParams)
{
	if (pWhalParams)
		os_memoryFree(pWhalParams->hOs, pWhalParams, sizeof(WhalParams_T));
	return OK;
}

/****************************************************************************
 *                      whal_params_Config()
 ****************************************************************************
 * DESCRIPTION:	Config the object 
 * 
 * INPUTS:	
 *		pWhalParams		The object
 *		hReport			The reports objects
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
int whal_params_Config(WhalParams_T *pWhalParams, TI_HANDLE hReport)
{
	pWhalParams->hReport = hReport;
	return OK;
}

/*
 * -------------------------------------------------------------------------- 
 *			DmaParams_T - Rx/Tx Queues and Bufs params
 * -------------------------------------------------------------------------- 
 */
/* defaults for large queue size */
#define DEFAULT_UCAST_PRIORITY		0
#define DEFAULT_RX_Q_PRIORITY		0

#define DEFAULT_NUM_STATIONS		1
#define DEFAULT_RXQ_PRIORITY		0		/* low 0 .. 15 high  */
#define DEFAULT_RXQ_TYPE			0x07	/* All frames, Data/Ctrl/Mgmt -- not documented well */

/* only for AP */
#define DEFAULT_NUM_BCAST_TX_DESC		16  /*  8 increase number of BC frames */
#define DEFAULT_BCAST_PRIORITY		0x81

/****************************************************************************
 *                      whal_ParamsInitDmaParams()
 ****************************************************************************
 * DESCRIPTION:	Initiate DmaParams to the default values
 * 
 * INPUTS:	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	OK or NOK
 ****************************************************************************/
void whal_ParamsInitDmaParams(WhalParams_T *pWhalParams)
{
}

/****************************************************************************
 *                      whal_ParamsGetDmaParams()
 ****************************************************************************
 * DESCRIPTION:	return DmaParams pointer
 ****************************************************************************/
DmaParams_T *whal_ParamsGetDmaParams(WhalParams_T *pWhalParams) 
{	
	return (&pWhalParams->DmaParams); 
}

/****************************************************************************
 *                      whal_ParamsSetDmaParams()
 ****************************************************************************
 * DESCRIPTION:	set new DmaParams
 ****************************************************************************/
int whal_ParamsSetDmaParams(WhalParams_T *pWhalParams) 
{	
	DmaParams_T *pDmaParams = whal_ParamsGetDmaParams(pWhalParams);
	int i;

	/* Initialize the Params object database fields*/
	pDmaParams->rxMemBlkNumber		= 35;							/*pInitParams->rxMemBlkNumber;*/
	pDmaParams->txMinMemBlkNumber	= 64;							/*pInitParams->txMinMemBlkNumber;*/
	pDmaParams->BlockSize			= HAL_CTRL_ACX_BLOCK_SIZE_DEF;	/*pInitParams->blockSize;*/
	pDmaParams->NumRxQueues			= 1;
	pDmaParams->NumTxQueues			= MAX_NUM_OF_TX_QUEUES;			/*pInitParams->numTxQueues;*/
	pDmaParams->RxNumDesc			= HAL_CTRL_ACX_RX_DESC_DEF;		/*pInitParams->rxDescNum;*/
	/* TRACE_BUFFER_MAX_SIZE is UINT32 so multiply by 4 to have the actual length */
	pDmaParams->TraceBufferSize		= (TRACE_BUFFER_MAX_SIZE * 4);	/*pInitParams->TraceBufferSize;*/
    pDmaParams->TraceBufferDoPrint	= FALSE;						/*pInitParams->bDoPrint;*/

	if ((pDmaParams->NumTxQueues > MAX_NUM_OF_TX_QUEUES) || (pDmaParams->NumTxQueues < 1) )
	{
		WLAN_REPORT_REPLY(pWhalParams->hReport, HAL_HW_CTRL_MODULE_LOG,
			("\nwhal_ParamsSetDmaParams: numTxQueues is invalid, setting it to 1. numTxQueues=%d\n",pDmaParams->NumTxQueues ));
		pDmaParams->NumTxQueues = 1;
	}
	
	for (i=0;i<pDmaParams->NumTxQueues;i++)
	{
		pDmaParams->TxNumDesc[i]	= HAL_CTRL_ACX_TX_DESC_DEF;		/*pInitParams->tx_attrib_queue[i].numDesc;*/
		pDmaParams->TxQPriority[i]	= i;							/*pInitParams->tx_attrib_queue[i].priority;		*/
	}

	/* default values */
	pDmaParams->RxQPriority			= DEFAULT_RX_Q_PRIORITY;
	pDmaParams->NumStations			= DEFAULT_NUM_STATIONS;
	pDmaParams->RxQueue_Priority	= DEFAULT_RXQ_PRIORITY;
	pDmaParams->RxQueue_Type		= DEFAULT_RXQ_TYPE;

	/* only for AP */
#ifdef CONFIGURE_BSS_TYPE_AP 
	pDmaParams->NumTxQueues			= 2;
	pDmaParams->TxNumDesc[1]		= DEFAULT_NUM_BCAST_TX_DESC;
	pDmaParams->TxQPriority[1]		= DEFAULT_BCAST_PRIORITY;
#endif

	return OK; 
}

/****************************************************************************
 *                      whal_ParamsSetRoamingParams()
 ****************************************************************************
 * DESCRIPTION:	set new DmaParams
 ****************************************************************************/
int whal_ParamsSetRoamingParams(WhalParams_T *pWhalParams) 
{	
	
	pWhalParams->WlanParams.roamTriggers.rssiThreshold 		=  RSSI_DEFAULT_THRESHOLD;
	pWhalParams->WlanParams.roamTriggers.rssiFilterWeight 		=  RSSI_DEFAULT_WEIGHT;
	pWhalParams->WlanParams.roamTriggers.rssiFilterDepth 		=  RSSI_DEFAULT_DEPTH;
	pWhalParams->WlanParams.roamTriggers.lowRSSIEventType 	= LOW_RSSI_EVENT_LEVEL;

	pWhalParams->WlanParams.roamTriggers.snrThreshold 		= SNR_DEFAULT_THRESHOLD;
	pWhalParams->WlanParams.roamTriggers.snrFilterWeight 		= SNR_DEFAULT_WEIGHT;
	pWhalParams->WlanParams.roamTriggers.snrFilterDepth 		= SNR_DEFAULT_DEPTH;
	pWhalParams->WlanParams.roamTriggers.lowSNREventType 	= LOW_SNR_EVENT_LEVEL;

	pWhalParams->WlanParams.roamTriggers.BssLossTimeout = NO_BEACON_DEFAULT_TIMEOUT;
	pWhalParams->WlanParams.roamTriggers.TsfMissThreshold = OUT_OF_SYNC_DEFAULT_THRESHOLD;
	
	return OK; 
}


/****************************************************************************
 *                      whal_ParamsSetQueueParams()
 ****************************************************************************
 * DESCRIPTION:	set new QueueParams
 ****************************************************************************/
int whal_ParamsSetQueueParams(WhalParams_T *pWhalParams,queueTrafficParams_t *pQtrafficParams) 
{	
	pWhalParams->QueuesParams.isQueueConfigured[pQtrafficParams->queueID] = TRUE;
	os_memoryCopy(pWhalParams->hOs,(void *)&(pWhalParams->QueuesParams.queues[pQtrafficParams->queueID]),(void *)pQtrafficParams,sizeof(queueTrafficParams_t));
	
	return OK;
}
/****************************************************************************
 *                      whal_ParamsSetAcParams()
 ****************************************************************************
 * DESCRIPTION:	set new QueueParams
 ****************************************************************************/

void whal_ParamsSetAcParams(WhalParams_T *pWhalParams,acQosParams_t *pAcQosParams)
{
	pWhalParams->AcParams.isAcConfigured[pAcQosParams->ac] = TRUE;
	os_memoryCopy(pWhalParams->hOs,(void *)&(pWhalParams->AcParams.ac[pAcQosParams->ac]),(void *)pAcQosParams,sizeof(acQosParams_t));
}

/****************************************************************************
 *                      whal_ParamsGetTxParams()
 ****************************************************************************
 * DESCRIPTION:	set new DmaParams
 ****************************************************************************/
TxParam_t *whal_ParamsGetTxParams (WhalParams_T *pWhalParams)
{
	return (&pWhalParams->TxParams);
}

/****************************************************************************
 *                      whal_ParamsSetTrafficParams()
 ****************************************************************************
 * DESCRIPTION: 	Traffic Parameters :(IE ACX_TID_CFG)
 ****************************************************************************/
int whal_ParamsSetTrafficParams(WhalParams_T *pWhalParams,whaCtrl_acTrafficParams_t* pTconfParams) 
{	 
	os_memoryCopy(pWhalParams->hOs,
				  (void *)&(pWhalParams->TxParams.halTrafficParams[pTconfParams->acId]),
				  (void *)pTconfParams,sizeof(whaCtrl_acTrafficParams_t));
	
	return OK;
}

/****************************************************************************
 *                      whal_ParamsSetTxRateClassParams()
 ****************************************************************************
 * DESCRIPTION:	set new TxRateParams
 ****************************************************************************/
void whal_ParamsSetTxRateClassParams(WhalParams_T *pWhalParams,txRatePolicy_t *pTxRatePolicy)
{
	UINT8 i;
	txRateClass_t *pTxRateClass;

	for(i = 0; i < pTxRatePolicy->numOfRateClasses; i++)
	{
		pTxRateClass = &pTxRatePolicy->rateClass[i];
		os_memoryCopy(pWhalParams->hOs,(void *)&(pWhalParams->BssInfoParams.TxRateClassParams.rateClass[i]),(void *)pTxRateClass,sizeof(txRateClass_t));
	}

	pWhalParams->BssInfoParams.TxRateClassParams.numOfRateClasses = pTxRatePolicy->numOfRateClasses;

}

/****************************************************************************
 *                      whal_ParamsSetTxRateClassParams()
 ****************************************************************************
 * DESCRIPTION:	set new TxRateParams
 ****************************************************************************/
txRatePolicy_t* whal_ParamsGetTxRateClassParams(WhalParams_T *pWhalParams)
{
    return &pWhalParams->BssInfoParams.TxRateClassParams;
}
/****************************************************************************
 *                      whal_ParamsGetTxParams()
 ****************************************************************************
 *	Traffic Parameters :(IE ACX_TID_CFG)
 ****************************************************************************/
whaCtrl_acTrafficParams_t* whal_ParamsGetTrafficParams(WhalParams_T *pWhalParams,UINT8 AcID) 
{	 
	return (&pWhalParams->TxParams.halTrafficParams[AcID]);
}

/****************************************************************************
 *                      whal_ParamsSetAccessCategoryParams()
 ****************************************************************************
 * DESCRIPTION:	Access Category Parameters :(IE ACX_AC_CFG)
 ****************************************************************************/
int whal_ParamsSetAccessCategoryParams(WhalParams_T *pWhalParams,acQueuesParams_t* pAcQueuesParams) 
{

	/* Calculate the Qid from the Ac and save in a local table */
	whal_ParamsSetQidToAcTable(pWhalParams, pAcQueuesParams->qId, pAcQueuesParams->acId);

	os_memoryCopy(pWhalParams->hOs, 
				  (void *)&(pWhalParams->TxParams.halAcQueueParams[pAcQueuesParams->qId]),
				  (void *)pAcQueuesParams, sizeof(acQueuesParams_t));

	return OK;
}


/****************************************************************************
 *                      whal_ParamsSetAccessCategoryAckPolicy()
 ****************************************************************************
 * DESCRIPTION:	set Ac Ack Policy according to Ac ID , No Ack = 0, Ack = 1
 ****************************************************************************/
int whal_ParamsSetAccessCategoryAckPolicy(WhalParams_T *pWhalParams, BOOL AckPolicy,UINT8 AcID)
{	
	WLAN_REPORT_INFORMATION(pWhalParams->hReport, HAL_HW_CTRL_MODULE_LOG,
	("\n whal_ParamsSetAccessCategoryAckPolicy: Set Access Category =%d  to Ack Policy = %d \n", AcID, AckPolicy));

	pWhalParams->TxParams.AckPolicy[AcID] = AckPolicy;

	return OK;
}


/****************************************************************************
 *                      whal_ParamsSetQidToAcTable()
 ****************************************************************************
 * DESCRIPTION:	set Qid according to Queue ID
 ****************************************************************************/
void whal_ParamsSetQidToAcTable(WhalParams_T *pWhalParams,UINT8 Qid,UINT8 AcId)
{

	pWhalParams->TxParams.QidToAcTable[Qid] = AcId;

}

/****************************************************************************
 *                      whal_ParamsGetAcIdFromQid()
 ****************************************************************************
 * DESCRIPTION:	Get the Ac Id according to the Queue id 
 ****************************************************************************/
UINT8 whal_ParamsGetAcIdFromQid(WhalParams_T *pWhalParams,UINT8 Qid)
{
	return (pWhalParams->TxParams.QidToAcTable[Qid]);
}

/****************************************************************************
 *                      whal_ParamsGetAtimWindow()
 ****************************************************************************
 * DESCRIPTION:	return the ATim Window
 ****************************************************************************/
UINT16 whal_ParamsGetAtimWindow		(WhalParams_T *pWhalParams)
{
	return (pWhalParams->BssInfoParams.ATimWindow);
}

/****************************************************************************
 *                      whal_ParamsSetAtimWindow()
 ****************************************************************************
 * DESCRIPTION:	set the ATim Window
 ****************************************************************************/
void whal_ParamsSetAtimWindow(WhalParams_T *pWhalParams, UINT16 ATimWindow)
{
	pWhalParams->BssInfoParams.ATimWindow = ATimWindow;
}

/* 
 * -------------------------------------------------------------------------- 
 *							Bss Info params
 * -------------------------------------------------------------------------- 
 */

#define DEFAULT_HW_GEN_PREAMBLE_TYPE	CCK_LONG	/* Long Preamble */
#define DEFAULT_HW_GEN_TX_RATE			RATE_2MBPS  
#define DEFAULT_HW_GEN_TX_RATE_DRIVER_FORMAT   DRV_RATE_2M

int genMacCopy(char *Mac2, char *Mac1)
{
	register int MacSize=6;

	while(MacSize--)
		*Mac2++ = *Mac1++;
	return OK;
}

/****************************************************************************
 *                      whal_ParamsInitBssInfoParams()
 ****************************************************************************
 * DESCRIPTION:	Initiate BssInfo to the default values
 * 
 * INPUTS:	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	
 ****************************************************************************/
int whal_ParamsInitBssInfoParams(WhalParams_T *pWhalParams, BOOL TxFlashEnable)
{
	BssInfoParams_T *pBssInfoParams = &pWhalParams->BssInfoParams;


	pBssInfoParams->txCtrlFrmRateDriverFormat = DEFAULT_HW_GEN_TX_RATE_DRIVER_FORMAT;
	pBssInfoParams->txCtrlFrmModulation = DEFAULT_HW_GEN_PREAMBLE_TYPE;
	pBssInfoParams->txCtrlFrmRate 		= DEFAULT_HW_GEN_TX_RATE;

	pBssInfoParams->txMgmtFrmModulation = DEFAULT_HW_GEN_PREAMBLE_TYPE;
	pBssInfoParams->txMgmtFrmRate 		= DEFAULT_HW_GEN_TX_RATE;

	pBssInfoParams->RadioChannel = DEFAULT_HW_RADIO_CHANNEL;
	pBssInfoParams->Ctrl = 0;

	/* Intilaize the ctrl field in the BSS join structure */
	/* Only bit_7 in the ctrl field is vurrently in use.
	  If bit_7 is on => Doing Tx flash before joining new AP */
	if(TxFlashEnable)
		pBssInfoParams->Ctrl |= JOIN_CMD_CTRL_TX_FLUSH;
	
	return OK;
}
  
/****************************************************************************
 *                      whal_ParamsGetBssInfoParams()
 ****************************************************************************
 * DESCRIPTION:	return pointer to the BssInfo params
 ****************************************************************************/
BssInfoParams_T *whal_ParamsGetBssInfoParams(WhalParams_T *pWhalParams)
{
	return (&pWhalParams->BssInfoParams);
}

/****************************************************************************
 *                      whal_ParamsGetRadioChannel()
 ****************************************************************************
 * DESCRIPTION:	return the radio channel 
 ****************************************************************************/
UINT8 whal_ParamsGetRadioChannel(WhalParams_T *pWhalParams)
{
	return (pWhalParams->BssInfoParams.RadioChannel);
}

/****************************************************************************
 *                      whal_ParamsSetRadioChannel()
 ****************************************************************************
 * DESCRIPTION:	set the radio channel 
 ****************************************************************************/
void whal_ParamsSetRadioChannel(WhalParams_T *pWhalParams, int Channel)
{
	pWhalParams->BssInfoParams.RadioChannel = (UINT8)Channel;
}

/****************************************************************************
 *                      whal_ParamsGetDefaultChannel()
 ****************************************************************************
 * DESCRIPTION:	return the default channel to start from 
 ****************************************************************************/
UINT8 whal_ParamsGetDefaultChannel(WhalParams_T *pWhalParams)
{
	WlanParams_T *pWlanParams = &pWhalParams->WlanParams;

	if (RADIO_BAND_5_0_GHZ == pWlanParams->RadioBand)
	{
		return pWlanParams->calibrationChannel5_0;
	}
	else
	{
		return pWlanParams->calibrationChannel2_4;
	}
}

/****************************************************************************
 *                      whal_ParamsSetReqBssType()
 ****************************************************************************
 * DESCRIPTION:	set the bss type (driver enum)
 ****************************************************************************/
void whal_ParamsSetReqBssType(WhalParams_T *pWhalParams, int Val)
{
	pWhalParams->BssInfoParams.ReqBssType = Val;
}

/****************************************************************************
 *                      whal_ParamsGetReqBssType()
 ****************************************************************************
 * DESCRIPTION:	get the bss type (driver enum)
 ****************************************************************************/
UINT8 whal_ParamsGetReqBssType(WhalParams_T *pWhalParams)
{
	return pWhalParams->BssInfoParams.ReqBssType;
}

/****************************************************************************
 *                      whal_ParamsSetBssType()
 ****************************************************************************
 * DESCRIPTION:	set the bss type (public enum)
 ****************************************************************************/
void whal_ParamsSetBssType(WhalParams_T *pWhalParams, int Val)
{
	pWhalParams->BssInfoParams.BssType = Val;
}
/****************************************************************************
 *                      whal_ParamsSetRadioBand()
 ****************************************************************************
 * DESCRIPTION:	set the radio band 
 ****************************************************************************/
void whal_ParamsSetRadioBand(WhalParams_T *pWhalParams, int RadioBand)
{
	pWhalParams->WlanParams.RadioBand = (UINT8)RadioBand;
}

/****************************************************************************
 *                      whal_ParamsGetRadioBand()
 ****************************************************************************
 * DESCRIPTION:	get the radio band
 ****************************************************************************/
UINT8  whal_ParamsGetRadioBand(WhalParams_T *pWhalParams)
{
		return (pWhalParams->WlanParams.RadioBand);
}

 /****************************************************************************
 *                      whal_ParamsSetPowerSaveState()
 ****************************************************************************
 * DESCRIPTION:	set the power save state 
 ****************************************************************************/
void whal_ParamsSetPowerSaveState(WhalParams_T *pWhalParams, UINT8 CurrPowerSaveState)
{
	pWhalParams->WlanParams.CurrPowerSaveState = (UINT8)CurrPowerSaveState;
}

/****************************************************************************
 *                      whal_ParamsGetPowerSaveState()
 ****************************************************************************
 * DESCRIPTION:	get the power save state 
 ****************************************************************************/
UINT8  whal_ParamsGetPowerSaveState(WhalParams_T *pWhalParams)
{
		return (pWhalParams->WlanParams.CurrPowerSaveState);
}


/****************************************************************************
 *                      whal_ParamsGetBssType()
 ****************************************************************************
 * DESCRIPTION:	return the bss type 
 ****************************************************************************/
UINT8 whal_ParamsGetBssType(WhalParams_T *pWhalParams)
{
	return (pWhalParams->BssInfoParams.BssType);
}

/****************************************************************************
 *                      whal_ParamsSetBeaconInterval()
 ****************************************************************************
 * DESCRIPTION:	set the beacon interval 
 ****************************************************************************/
void whal_ParamsSetBeaconInterval(WhalParams_T *pWhalParams, UINT16 Val)
{
	pWhalParams->BssInfoParams.BeaconInterval = Val;
}

/****************************************************************************
 *                      whal_ParamsGetBeaconInterval()
 ****************************************************************************
 * DESCRIPTION:	return the beacon interval
 ****************************************************************************/
UINT16 whal_ParamsGetBeaconInterval(WhalParams_T *pWhalParams)
{
	return (pWhalParams->BssInfoParams.BeaconInterval);
}

/****************************************************************************
 *                      whal_ParamsSetDtimCount()
 ****************************************************************************
 * DESCRIPTION:	set the dtim count interval 
 ****************************************************************************/
void whal_ParamsSetDtimCount(WhalParams_T *pWhalParams, UINT8 Val)
{
	pWhalParams->BssInfoParams.DtimInterval = Val;
}

/****************************************************************************
 *                      whal_ParamsGetDtimCount()
 ****************************************************************************
 * DESCRIPTION:	return the Dtim count interval
 ****************************************************************************/
UINT8 whal_ParamsGetDtimCount(WhalParams_T *pWhalParams)
{
	return (pWhalParams->BssInfoParams.DtimInterval);
}

/****************************************************************************
 *                      whal_ParamsGetBssId()
 ****************************************************************************
 * DESCRIPTION:	return the Bss Id
 ****************************************************************************/
UINT8 *whal_ParamsGetBssId(WhalParams_T *pWhalParams)
{
	return (pWhalParams->BssInfoParams.BssId);
}

/****************************************************************************
 *                      whal_ParamsSetBssId()
 ****************************************************************************
 * DESCRIPTION:	set the Bss Id
 ****************************************************************************/
void whal_ParamsSetBssId(WhalParams_T *pWhalParams, char *BssId)
{
    genMacCopy((char *)pWhalParams->BssInfoParams.BssId, BssId);
}

/****************************************************************************
 *                      whal_ParamsGetElm_Ssid()
 ****************************************************************************
 * DESCRIPTION:	return the SSID info element
 ****************************************************************************/
dot11_SSID_t  *whal_ParamsGetElm_Ssid(WhalParams_T *pWhalParams)
{
	return (&pWhalParams->BssInfoParams.WlanElm_Ssid);
}

/****************************************************************************
 *                      whal_ParamsSetSsid()
 ****************************************************************************
 * DESCRIPTION:	set the SSID
 ****************************************************************************/
void whal_ParamsSetSsid(WhalParams_T *pWhalParams, char *InputSsid, UINT8 SsidLength)
{
	BssInfoParams_T *pBssInfoParams = &pWhalParams->BssInfoParams;

	pBssInfoParams->WlanElm_Ssid.hdr.eleLen = SsidLength;
	os_memoryZero(pWhalParams->hOs, (void *)pBssInfoParams->WlanElm_Ssid.serviceSetId, 
				  sizeof(pBssInfoParams->WlanElm_Ssid.serviceSetId));
	os_memoryCopy(pWhalParams->hOs, (void *)pBssInfoParams->WlanElm_Ssid.serviceSetId, (void *)InputSsid, SsidLength);
}

/****************************************************************************
 *                      whal_ParamsSetBasicRates()
 ****************************************************************************
 * DESCRIPTION:	set the basic rate set
 ****************************************************************************/
void whal_ParamsSetBasicRatesSet(WhalParams_T *pWhalParams, UINT16 BasicRateSet)
{    
	pWhalParams->BssInfoParams.BasicRateSet = BasicRateSet;
}

/****************************************************************************
 *                      whal_ParamsSetHwGenTxParams()
 ****************************************************************************
 * DESCRIPTION:	set the hardware and firmware generated Tx rate (convert to HW coding).
 *
 ****************************************************************************/
void whal_ParamsSetHwGenTxParams(WhalParams_T *pWhalParams, rate_e TxRate, BOOL bCtrlFrame)
{ 
	/* Set The Rate & modulation for CTS, RTS, and PS-Poll frames (not including CTS-protection). */
	if (bCtrlFrame)
	{
	    pWhalParams->BssInfoParams.txCtrlFrmRateDriverFormat = TxRate;
	    whalUtils_ConvertAppRate(TxRate,  &pWhalParams->BssInfoParams.txCtrlFrmRate);
	    whalUtils_FindHwModulationByDrvRate(TxRate, &pWhalParams->BssInfoParams.txCtrlFrmModulation, pWhalParams->WlanParams.preamble); 
	}
	/* Set The Rate & modulation for Beacon and Probe-Response frames (IBSS). */
	else
	{
		whalUtils_ConvertAppRate(TxRate, &pWhalParams->BssInfoParams.txMgmtFrmRate);
		whalUtils_FindHwModulationByDrvRate(TxRate, &pWhalParams->BssInfoParams.txMgmtFrmModulation, pWhalParams->WlanParams.preamble);
	}
}
 
/****************************************************************************
 *                      whal_ParamsSetBasicRates()
 ****************************************************************************
 * DESCRIPTION:	set the basic rate set
 ****************************************************************************/
void whal_ParamsSetSupportedRatesSet(WhalParams_T *pWhalParams, UINT16 SupportedRateSet)
{
	pWhalParams->BssInfoParams.SupportedRateSet = SupportedRateSet;
}


/****************************************************************************
 *                      whal_ParamsGetMacPreambleParams()
 ****************************************************************************
 * DESCRIPTION:	return pointer to the Mac Preamble Params
 ****************************************************************************/
void   whal_ParamsGetMacPreambleParams(WhalParams_T *pWhalParams, UINT8* earlyWakeUp)
{	
    *earlyWakeUp = pWhalParams->WlanParams.earlyWakeUp;
}


/* 
 * -------------------------------------------------------------------------- 
 *							wlan params
 * -------------------------------------------------------------------------- 
 */

#define DEFAULT_CW_MIN	15
#define DEFAULT_USE_DEVICE_ERROR_INTERRUPT	1

/****************************************************************************
 *                      whal_ParamsInitWlanParams()
 ****************************************************************************
 * DESCRIPTION:	Initiate GenParams to the default values
 * 
 * INPUTS:	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	
 ****************************************************************************/
int whal_ParamsInitWlanParams(WhalParams_T *pWhalParams)
{
	WlanParams_T *pWlanParams = &pWhalParams->WlanParams;

	/* 
	 * init filters as station (start/join with BssType will overwrite the values)
	 */
	whal_ParamsSetRxFilter(pWhalParams, RX_CONFIG_OPTION_MY_DST_MY_BSS, RX_FILTER_OPTION_FILTER_ALL);

	pWlanParams->UseDeviceErrorInterrupt = DEFAULT_USE_DEVICE_ERROR_INTERRUPT;
	
	/*
	 * All other params are zero and will be set externally on the Create/Config phase
	 */

	return OK;
}								 

/****************************************************************************
 *                      whal_ParamsGetWlanParams()
 ****************************************************************************
 * DESCRIPTION:	return pointer to the GenParams
 ****************************************************************************/
WlanParams_T *whal_ParamsGetWlanParams(WhalParams_T *pWhalParams)
{
	return (&pWhalParams->WlanParams);
}

/****************************************************************************
 *                      whal_ParamsSetFragmentThreshold()
 ****************************************************************************
 * DESCRIPTION:	set the fragmentation threshold 
 ****************************************************************************/
void whal_ParamsSetFragmentThreshold(WhalParams_T *pWhalParams, int FragSize)
{
	pWhalParams->WlanParams.FragmentThreshold = FragSize;
}

/****************************************************************************
 *                      whal_ParamsPrintFragmentThreshold()
 ****************************************************************************
 * DESCRIPTION:	print the fragmentation threshold 
 ****************************************************************************/
void whal_ParamsPrintFragmentThreshold(WhalParams_T *pWhalParams)
{
	WLAN_REPORT_REPLY(pWhalParams->hReport, HAL_HW_CTRL_MODULE_LOG,  
		("\n FragmentThreshold = %d\n", pWhalParams->WlanParams.FragmentThreshold));
}

/****************************************************************************
 *                      whal_ParamsPrintFragmentThreshold()
 ****************************************************************************
 * DESCRIPTION:	print the fragmentation threshold 
 ****************************************************************************/
UINT8 whal_ParamsIsFragmentOnHal(WhalParams_T *pWhalParams)
{
	return (pWhalParams->WlanParams.FragmentationOnHal);
}
/****************************************************************************
 *                      whal_ParamsGetFragmentThreshold()
 ****************************************************************************
 * DESCRIPTION:	return the fragmentation threshold
 ****************************************************************************/
UINT32 whal_ParamsGetFragmentThreshold(WhalParams_T *pWhalParams)
{
	return (pWhalParams->WlanParams.FragmentThreshold);
}

/****************************************************************************
 *                      whal_ParamsGetFragmentThreshold()
 ****************************************************************************
 * DESCRIPTION:	return the fragmentation threshold
 ****************************************************************************/
UINT8 whal_ParamsGetMaxSitesFragCollect(WhalParams_T *pWhalParams)
{
	return (pWhalParams->WlanParams.maxSitesFragCollect);
}

/****************************************************************************
 *                      whal_ParamsSetRtsThreshold()
 ****************************************************************************
 * DESCRIPTION:	set the rts threshold 
 ****************************************************************************/
void whal_ParamsSetRtsThreshold(WhalParams_T *pWhalParams, int RtsSize)
{
	pWhalParams->WlanParams.RtsThreshold = RtsSize;
}

/****************************************************************************
 *                      whal_ParamsSetListenInterval()
 ****************************************************************************
 * DESCRIPTION:	set the Listen Interval 
 ****************************************************************************/
void whal_ParamsSetListenInterval(WhalParams_T *pWhalParams, UINT8 Val)
{
	pWhalParams->WlanParams.ListenInterval = Val;
}

/****************************************************************************
 *                      whal_ParamsGetListenInterval()
 ****************************************************************************
 * DESCRIPTION:	get the Listen Interval
 ****************************************************************************/
UINT8 whal_ParamsGetListenInterval(WhalParams_T *pWhalParams)
{
	return (pWhalParams->WlanParams.ListenInterval);
}

/****************************************************************************
 *                      whal_ParamsSetRxFilter()
 ****************************************************************************
 * DESCRIPTION:	set the wlan hardware filters
 ****************************************************************************/
void whal_ParamsSetRxFilter(WhalParams_T *pWhalParams, UINT32 RxConfigOption, UINT32 RxFilterOption)
{
	pWhalParams->WlanParams.RxConfigOption = RxConfigOption;
	pWhalParams->WlanParams.RxFilterOption = RxFilterOption;

	pWhalParams->WlanParams.RxConfigOption |= RX_CFG_ENABLE_PHY_HEADER_PLCP;

#if defined (TNETW_MASTER_MODE) || defined (TNETW_USB_MODE)
	pWhalParams->WlanParams.RxConfigOption |= RX_CFG_COPY_RX_STATUS;
#endif	

	if(pWhalParams->WlanParams.RxDisableBroadcast)
		pWhalParams->WlanParams.RxConfigOption |= RX_CFG_DISABLE_BCAST;
}

/****************************************************************************
 *                      whal_ParamsGetRxFilter()
 ****************************************************************************
 * DESCRIPTION:	Get the wlan hardware filters
 ****************************************************************************/
void whal_ParamsGetRxFilter(WhalParams_T *pWhalParams, UINT32* pRxConfigOption, UINT32* pRxFilterOption)
{
	*pRxConfigOption = pWhalParams->WlanParams.RxConfigOption;
	*pRxFilterOption = pWhalParams->WlanParams.RxFilterOption;
}


/****************************************************************************
*                      whal_ParamsSetarpIpAddressesTable()
****************************************************************************
* DESCRIPTION:	set the wlan hardware filters
****************************************************************************/
void whal_ParamsSetarpIpAddressesTable(WhalParams_T * pWhalParams, IpAddress_t * IP_addr, IPver_e IP_ver)
{    
    pWhalParams->WlanParams.arp_IP_ver = IP_ver;

    if (IP_ver == IP_VER_4) 
    {
        os_memoryCopy(pWhalParams->hOs,  (PVOID)pWhalParams->WlanParams.arp_IP_addr.addr,  (PVOID)IP_addr, IP_V4_ADDR_LEN);
    }
    else /* IP_VER_6*/
    {
        os_memoryCopy(pWhalParams->hOs,  (PVOID)pWhalParams->WlanParams.arp_IP_addr.addr,  (PVOID)IP_addr, IP_V6_ADDR_LEN);
    }
}

/****************************************************************************
*                      whal_ParamsGetarpIpAddressesTable()
****************************************************************************
* DESCRIPTION:	get the wlan hardware filters
****************************************************************************/
void whal_ParamsGetarpIpAddressesTable(WhalParams_T * pWhalParams, IpAddress_t * IP_addr, IPver_e* pIP_ver)
{
    
    *pIP_ver = (IPver_e)pWhalParams->WlanParams.arp_IP_ver;

    if (*pIP_ver == IP_VER_4) 
    {
        os_memoryCopy(pWhalParams->hOs,  (PVOID)IP_addr,  (PVOID)pWhalParams->WlanParams.arp_IP_addr.addr, IP_V4_ADDR_LEN);
    }
    else /* IP_VER_6*/
    {
        os_memoryCopy(pWhalParams->hOs,  (PVOID)IP_addr,  (PVOID)pWhalParams->WlanParams.arp_IP_addr.addr, IP_V6_ADDR_LEN);
    }
}


/****************************************************************************
 *                      whal_ParamsSetarpIpFilterEnabled()
 ****************************************************************************
 * DESCRIPTION:	set the wlan hardware filters
 ****************************************************************************/
void whal_ParamsSetarpIpFilterEnabled(WhalParams_T * pWhalParams, UINT8 isEnabled)
{
	if ( NULL != pWhalParams )
	{
		pWhalParams->WlanParams.isArpIpFilteringEnabled = isEnabled;
	}

}

/****************************************************************************
 *                      whal_ParamsGetarpIpFilterEnabled()
 ****************************************************************************
 * DESCRIPTION:	set the wlan hardware filters
 ****************************************************************************/
void whal_ParamsGetarpIpFilterEnabled(WhalParams_T * pWhalParams, UINT8* pisEnabled)
{
	if ( NULL != pWhalParams )
	{
		*pisEnabled = pWhalParams->WlanParams.isArpIpFilteringEnabled;
	}
}

/****************************************************************************
*                      whal_ParamsSetGroupAddressesTable()
****************************************************************************
* DESCRIPTION:	set the Group addr table
****************************************************************************/
void whal_ParamsSetGroupAddressesTable(WhalParams_T *pWhalParams, UINT8 isEnabled, UINT8 numGroupAddrs, macAddress_t *Group_addr)
{
	int i;
	
	pWhalParams->WlanParams.numGroupAddrs = numGroupAddrs;
    pWhalParams->WlanParams.isMacAddrFilteringnabled = isEnabled;
	
	for ( i=0 ; i < numGroupAddrs; i++) 
	{
		os_memoryCopy(pWhalParams->hOs, (PVOID)&(pWhalParams->WlanParams.Group_addr[i]), (PVOID)&(Group_addr->addr[MAC_ADDR_SIZE*i]), MAC_ADDR_SIZE);
	}

}

/****************************************************************************
*                      whal_ParamsGetGroupAddressesTable()
****************************************************************************
* DESCRIPTION:	get the Group addr table
****************************************************************************/
void   whal_ParamsGetGroupAddressesTable(WhalParams_T *pWhalParams, UINT8* pisEnabled, UINT8* pnumGroupAddrs, macAddress_t *Group_addr)
{
	int i;
	
	*pnumGroupAddrs = pWhalParams->WlanParams.numGroupAddrs;
    *pisEnabled = pWhalParams->WlanParams.isMacAddrFilteringnabled;

	os_memoryZero(pWhalParams->hOs, Group_addr, sizeof(Group_addr));
	for (i=0 ; i < *pnumGroupAddrs; i++) 
	{
		os_memoryCopy(pWhalParams->hOs, (void *)&(Group_addr->addr[MAC_ADDR_SIZE*i]), &pWhalParams->WlanParams.Group_addr[i], MAC_ADDR_SIZE);
	}

}


/****************************************************************************
 *                      whal_ParamsGetCurrAntenna()
 ****************************************************************************
 * DESCRIPTION:	get the current antenna setting
 ****************************************************************************/
UINT8 whal_ParamsGetCurrAntenna(WhalParams_T *pWhalParams)
{
	return (pWhalParams->WlanParams.CurrAntenna);
}

/* 
 * -------------------------------------------------------------------------- 
 *							General params
 * -------------------------------------------------------------------------- 
 */

#define DEFAULT_TRACE_ENABLE			0
#define DEFAULT_TRACE_OUT				0

#define DEFAULT_PBCC_DYNAMIC_ENABLE_VAL		0
#define DEFAULT_PBCC_DYNAMIC_INTERVAL		500	/* ticks */
#define DEFAULT_PBCC_DYNAMIC_IGNORE_MCAST	0

/****************************************************************************
 *                      whal_ParamsInitGenParams()
 ****************************************************************************
 * DESCRIPTION:	Initiate GenParams to the default values
 * 
 * INPUTS:	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	
 ****************************************************************************/
int whal_ParamsInitGenParams(WhalParams_T *pWhalParams)
{
	GenParams_T *pGenParams = &pWhalParams->GenParams;

	whal_ParamsSetPbccDynamicEnableVal(pWhalParams, DEFAULT_PBCC_DYNAMIC_ENABLE_VAL);
	pGenParams->PbccDynamicInterval	= DEFAULT_PBCC_DYNAMIC_INTERVAL;
	pGenParams->PbccDynamicIgnoreMcast	= DEFAULT_PBCC_DYNAMIC_IGNORE_MCAST;


	pGenParams->TraceEnable			= DEFAULT_TRACE_ENABLE;
	pGenParams->TraceOut			= DEFAULT_TRACE_OUT;

	return OK;
}

/****************************************************************************
 *                      whal_ParamsInitGenCounters()
 ****************************************************************************
 * DESCRIPTION:	Initiate GenCounters to the default values
 * 
 * INPUTS:	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	
 ****************************************************************************/
int whal_ParamsInitGenCounters(WhalParams_T *pWhalParams)
{
	GenCounters_T *pGenCounters = &pWhalParams->GenCounters;
											  
	pGenCounters->FcsErrCnt = 0;

	return OK;
}

/****************************************************************************
 *                      whal_ParamsGetGenParams()
 ****************************************************************************
 * DESCRIPTION:	return pointer to the GenParams
 ****************************************************************************/
GenParams_T *whal_ParamsGetGenParams(WhalParams_T *pWhalParams)
{
	return (&pWhalParams->GenParams);
}

/****************************************************************************
 *                      whal_ParamsGetPbccDynamicEnableVal()
 ****************************************************************************
 * DESCRIPTION:	return the use of Dynamic pbcc enable 
 ****************************************************************************/
UINT32 whal_ParamsGetPbccDynamicEnableVal(WhalParams_T *pWhalParams)
{
	return (pWhalParams->GenParams.PbccDynamicEnable);
}

/****************************************************************************
 *                      whal_ParamsSetPbccDynamic()
 ****************************************************************************
 * DESCRIPTION:	set the use of Dynamic pbcc enable 
 ****************************************************************************/
void whal_ParamsSetPbccDynamicEnableVal(WhalParams_T *pWhalParams, int EnableVal)
{
	pWhalParams->GenParams.PbccDynamicEnable = EnableVal;

	if (EnableVal)
		whal_ParamsSetRxFilter(pWhalParams, (UINT16)RX_CONFIG_OPTION_ANY_DST_ANY_BSS, (UINT16)RX_FILTER_OPTION_DEF);
	else
		whal_ParamsSetRxFilter(pWhalParams, (UINT16)RX_CONFIG_OPTION_ANY_DST_MY_BSS, (UINT16)RX_FILTER_OPTION_DEF);
}

/*
 * -------------------------------------------------------------------------- 
 *							Wlan hardware Info params
 * -------------------------------------------------------------------------- 
 */

/****************************************************************************
 *                      whal_ParamsInitHwInfoParams()
 ****************************************************************************
 * DESCRIPTION:	Initiate Hw info params
 * 
 * INPUTS:	
 * 
 * OUTPUT:	None
 * 
 * RETURNS:	
 ****************************************************************************/
int whal_ParamsInitHwInfoParams(WhalParams_T *pWhalParams)
{
	return OK;
}

/****************************************************************************
 *                      whal_ParamsGetSrcMac()
 ****************************************************************************
 * DESCRIPTION:	return the AP mac address
 ****************************************************************************/
UINT8 *whal_ParamsGetSrcMac(WhalParams_T *pWhalParams)
{
	return (pWhalParams->HwInfoParams.SrcMacAddr);
}

/****************************************************************************
 *                      whal_ParamsSetSrcMac()
 ****************************************************************************
 * DESCRIPTION:	set the SrcMac
 ****************************************************************************/
void whal_ParamsSetSrcMac(WhalParams_T *pWhalParams, char *SrcMac)
{
	genMacCopy((char *)pWhalParams->HwInfoParams.SrcMacAddr, SrcMac);
}

/****************************************************************************
 *                      whal_ParamsGetRadioType()
 ****************************************************************************
 * DESCRIPTION:	return the Radio type from the ConfigOptions
 ****************************************************************************/
char whal_ParamsGetRadioType(WhalParams_T *pWhalParams)
{
	return (pWhalParams->WlanParams.radioType);
}

/****************************************************************************
 *                      whal_ParamsHwNvramPrint()
 ****************************************************************************
 * DESCRIPTION:	Print ConfigOptions
 ****************************************************************************/
void whal_ParamsHwNvramPrint(WhalParams_T *pWhalParams)
{
 /* for future use */
}

/****************************************************************************
 *                      whal_ParamsGetAcxVersion()
 ****************************************************************************
 * DESCRIPTION:	return wlan hardware/software version
 ****************************************************************************/
ACXRevision_t *whal_ParamsGetAcxVersion(WhalParams_T *pWhalParams)
{
	return (&pWhalParams->HwInfoParams.AcxVersion);
}

/****************************************************************************
 *                      whal_ParamsGetFwVersion()
 ****************************************************************************
 * DESCRIPTION:	return wlan firmware version
 ****************************************************************************/
UINT8 *whal_ParamsGetFwVersion(WhalParams_T *pWhalParams)
{
    return (UINT8 *)(pWhalParams->HwInfoParams.AcxVersion.FWVersion);
}

/****************************************************************************
 *                      whal_ParamsPrintFwVersion()
 ****************************************************************************
 * DESCRIPTION:	print the fw version
 ****************************************************************************/
void whal_ParamsPrintFwVersion(WhalParams_T *pWhalParams)
{      
#ifdef TI_DBG
	UINT8 *StaId = whal_ParamsGetSrcMac(pWhalParams);

    WLAN_REPORT_INIT (pWhalParams->hReport, 
                      HAL_HW_CTRL_MODULE_LOG, 
                      ("Firmware version: %s\n", 
                      pWhalParams->HwInfoParams.AcxVersion.FWVersion));
    WLAN_REPORT_INIT (pWhalParams->hReport, 
                      HAL_HW_CTRL_MODULE_LOG, 
                      ("Station Id: %02X-%02X-%02X-%02X-%02X-%02X\n", 
                      StaId[0], StaId[1], StaId[2], StaId[3], StaId[4], StaId[5]));
#endif /* TI_DBG */
}
    
/****************************************************************************
 *                      whal_ParamsGetTraceBufferSize()
 ****************************************************************************
 * DESCRIPTION:	get trace buffer size
 ****************************************************************************/
UINT32 whal_ParamsGetTraceBufferSize(WhalParams_T *pWhalParams)
{
	return pWhalParams->DmaParams.TraceBufferSize;
}
/*
 * -------------------------------------------------------------------------- 
 *							Convert functions
 * -------------------------------------------------------------------------- 
 */

int whalUtils_ConvertHwRate(UINT8 HwRate, UINT8 HwModulation, rate_e *AppRate, modulationType_e *AppModulation)
{
	rate_e Rate = DRV_RATE_AUTO;
	modulationType_e Modulation = DRV_MODULATION_NONE;
	int Stt = OK;

	switch (HwRate)
	{
		case RATE_1MBPS:		Rate = DRV_RATE_1M;		Modulation = DRV_MODULATION_QPSK;	break;
		case RATE_2MBPS:		Rate = DRV_RATE_2M;		Modulation = DRV_MODULATION_QPSK;	break;         
		case RATE_5_5MBPS:		Rate = DRV_RATE_5_5M;	Modulation = DRV_MODULATION_CCK;	break;
		case RATE_11MBPS:		Rate = DRV_RATE_11M;	Modulation = DRV_MODULATION_CCK;	break;
		case RATE_22MBPS:	    Rate = DRV_RATE_22M;	Modulation = DRV_MODULATION_PBCC;	break;         
		case RATE_6MBPS:		Rate = DRV_RATE_6M;		Modulation = DRV_MODULATION_OFDM;	break;
		case RATE_9MBPS:		Rate = DRV_RATE_9M;		Modulation = DRV_MODULATION_OFDM;	break;
		case RATE_18MBPS:		Rate = DRV_RATE_18M;	Modulation = DRV_MODULATION_OFDM;	break;
		case RATE_24MBPS:		Rate = DRV_RATE_24M;	Modulation = DRV_MODULATION_OFDM;	break;
		case RATE_36MBPS:		Rate = DRV_RATE_36M;	Modulation = DRV_MODULATION_OFDM;	break;
		case RATE_48MBPS:		Rate = DRV_RATE_48M;	Modulation = DRV_MODULATION_OFDM;	break;
		case RATE_54MBPS:		Rate = DRV_RATE_54M;	Modulation = DRV_MODULATION_OFDM;	break;
		default:				Rate = DRV_RATE_1M;		Modulation = DRV_MODULATION_NONE;
			Stt = NOK;
			break;
	}
  
	/* patch for 12M (same code as 1M) */
	if ((HwRate == RATE_12MBPS) && (HwModulation == HW_MODULATION_OFDM))
	{
		Rate = DRV_RATE_12M;	Modulation = DRV_MODULATION_OFDM;
	}

	if (Stt == OK)
	{
		*AppRate = Rate;
		*AppModulation = Modulation;
	}

	return Stt;
}


int  whalUtils_ConvertAppRate (rate_e AppRate, UINT8 *HwRate)
{
	UINT8 Rate = 0;
	int Stt = OK;

	switch (AppRate)
	{
		/*
		 *	The handle for 5.5/11/22 PBCC was removed !!!
		 */

		case DRV_RATE_1M:			Rate = RATE_1MBPS;			break;
		case DRV_RATE_2M:			Rate = RATE_2MBPS;			break;
		case DRV_RATE_5_5M:   		Rate = RATE_5_5MBPS;		break;
		case DRV_RATE_11M:			Rate = RATE_11MBPS;			break;
		case DRV_RATE_22M:			Rate = RATE_22MBPS;			break;
		case DRV_RATE_6M:			Rate = RATE_6MBPS;			break;
		case DRV_RATE_9M:			Rate = RATE_9MBPS;			break;
		case DRV_RATE_12M:			Rate = RATE_12MBPS;			break;
		case DRV_RATE_18M:			Rate = RATE_18MBPS;			break;
		case DRV_RATE_24M:			Rate = RATE_24MBPS;			break;
		case DRV_RATE_36M:			Rate = RATE_36MBPS;			break;
		case DRV_RATE_48M:			Rate = RATE_48MBPS;			break;
		case DRV_RATE_54M:			Rate = RATE_54MBPS;			break;

		default:
            WLAN_OS_REPORT(("%s wrong app rate = %d\n",__FUNCTION__,AppRate));
			Stt = NOK;
			break;
	}

	if (Stt == OK)
		*HwRate = Rate;
	else
		*HwRate = RATE_1MBPS; 

	return (Stt);
}

int  whalUtils_FindHwModulationByDrvRate (rate_e AppRate, UINT8 *HwModu, UINT8 preamble)
{
	int Stt = OK;


	switch (AppRate)
	{
		/*
		 *	The handle for 5.5/11/22 PBCC was removed !!!
		 */

		case DRV_RATE_1M:			
            *HwModu = CCK_LONG;		
            break;

		case DRV_RATE_2M:					
		case DRV_RATE_5_5M:   				
		case DRV_RATE_11M:				
            if (preamble == 0) /* PREAMBLE_LONG*/
                *HwModu = CCK_LONG;
            else
                *HwModu = CCK_SHORT;
            break;

        case DRV_RATE_22M:				
            if (preamble == 0) /* PREAMBLE_LONG*/
                *HwModu = PBCC_LONG;
            else
                *HwModu = PBCC_SHORT;
            break;

		case DRV_RATE_6M:			
		case DRV_RATE_9M:			
		case DRV_RATE_12M:			
		case DRV_RATE_18M:			
		case DRV_RATE_24M:			
		case DRV_RATE_36M:			
		case DRV_RATE_48M:			
		case DRV_RATE_54M:		
            *HwModu = OFDM;			
            break;
	default:
            WLAN_OS_REPORT(("%s wrong rate = %d\n",__FUNCTION__,AppRate));
			*HwModu = CCK_LONG;
			Stt = NOK;
			break;
	}

	return (Stt);
}



int whalUtils_ConvertAppRatesBitmap(UINT32 AppRatesBitmap, UINT32 AppModulation, UINT16 *HwRatesBitmap)
{
	UINT16 RatesBitmap = 0;
   
	if (AppRatesBitmap & DRV_RATE_MASK_1_BARKER)	RatesBitmap |= HW_BIT_RATE_1MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_2_BARKER)	RatesBitmap |= HW_BIT_RATE_2MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_5_5_CCK)		RatesBitmap |= HW_BIT_RATE_5_5MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_11_CCK)		RatesBitmap |= HW_BIT_RATE_11MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_22_PBCC)		RatesBitmap |= HW_BIT_RATE_22MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_6_OFDM)		RatesBitmap |= HW_BIT_RATE_6MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_9_OFDM)		RatesBitmap |= HW_BIT_RATE_9MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_12_OFDM)		RatesBitmap |= HW_BIT_RATE_12MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_18_OFDM)		RatesBitmap |= HW_BIT_RATE_18MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_24_OFDM)		RatesBitmap |= HW_BIT_RATE_24MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_36_OFDM)		RatesBitmap |= HW_BIT_RATE_36MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_48_OFDM)		RatesBitmap |= HW_BIT_RATE_48MBPS;
	if (AppRatesBitmap & DRV_RATE_MASK_54_OFDM)		RatesBitmap |= HW_BIT_RATE_54MBPS;
	
	*HwRatesBitmap = RatesBitmap;

	return (OK);
}

int whalUtils_ConvertHwRatesBitmap(UINT8 HwRatesBitmap, UINT8 HwModulation, UINT32 *AppRate, UINT32 *AppModulation)
{
	UINT16 RatesBitmap = 0;

	if (HwRatesBitmap & HW_BIT_RATE_1MBPS)			RatesBitmap |= DRV_RATE_MASK_1_BARKER;
	if (HwRatesBitmap & HW_BIT_RATE_2MBPS)			RatesBitmap |= DRV_RATE_MASK_2_BARKER;
	if (HwRatesBitmap & HW_BIT_RATE_5_5MBPS)		RatesBitmap |= DRV_RATE_MASK_5_5_CCK;
	if (HwRatesBitmap & HW_BIT_RATE_11MBPS)			RatesBitmap |= DRV_RATE_MASK_11_CCK;
	if (HwRatesBitmap & HW_BIT_RATE_22MBPS)			RatesBitmap |= DRV_RATE_MASK_22_PBCC;
	if (HwRatesBitmap & HW_BIT_RATE_6MBPS)			RatesBitmap |= DRV_RATE_MASK_6_OFDM;
	if (HwRatesBitmap & HW_BIT_RATE_9MBPS)			RatesBitmap |= DRV_RATE_MASK_9_OFDM;
	if (HwRatesBitmap & HW_BIT_RATE_12MBPS)			RatesBitmap |= DRV_RATE_MASK_12_OFDM;
	if (HwRatesBitmap & HW_BIT_RATE_18MBPS)			RatesBitmap |= DRV_RATE_MASK_18_OFDM;
	if (HwRatesBitmap & HW_BIT_RATE_24MBPS)			RatesBitmap |= DRV_RATE_MASK_24_OFDM;
	if (HwRatesBitmap & HW_BIT_RATE_36MBPS)			RatesBitmap |= DRV_RATE_MASK_36_OFDM;
	if (HwRatesBitmap & HW_BIT_RATE_48MBPS)			RatesBitmap |= DRV_RATE_MASK_48_OFDM;
	if (HwRatesBitmap & HW_BIT_RATE_54MBPS)			RatesBitmap |= DRV_RATE_MASK_54_OFDM;
		
	*AppRate = RatesBitmap;

	return (OK);
}

void whalUtils_ConvertBitmapToMaxRate(UINT16 HwRatesBitmap, UINT8 *HwModulation, UINT8 *HwRate)
{
	/*
	* !!!!!! MUST KEEP THE ORDER
	*/
	if (HwRatesBitmap & HW_BIT_RATE_54MBPS)
	{
		*HwRate = RATE_54MBPS;
		*HwModulation = HW_MODULATION_OFDM;
	}
	else if (HwRatesBitmap & HW_BIT_RATE_48MBPS)
	{
		*HwRate = RATE_48MBPS; 
		*HwModulation = HW_MODULATION_OFDM;
	}
	else if (HwRatesBitmap & HW_BIT_RATE_36MBPS)
	{
		*HwRate = RATE_36MBPS; 
		*HwModulation = HW_MODULATION_OFDM;
	}
	else if (HwRatesBitmap & HW_BIT_RATE_24MBPS)
	{
		*HwRate = RATE_24MBPS; 
		*HwModulation = HW_MODULATION_OFDM;
	}
	else if (HwRatesBitmap & HW_BIT_RATE_22MBPS)
	{
		*HwRate = RATE_22MBPS; 
		*HwModulation = HW_MODULATION_PBCC;
	}
	else if (HwRatesBitmap & HW_BIT_RATE_18MBPS)
	{
		*HwRate = RATE_18MBPS; 
		*HwModulation = HW_MODULATION_OFDM;
	}
	else if (HwRatesBitmap & HW_BIT_RATE_12MBPS)
	{
		*HwRate = RATE_12MBPS; 
		*HwModulation = HW_MODULATION_OFDM;
	}
	else if (HwRatesBitmap & HW_BIT_RATE_11MBPS)
	{
		*HwRate = RATE_11MBPS;
		*HwModulation = HW_MODULATION_PBCC;
	}
	else if (HwRatesBitmap & HW_BIT_RATE_9MBPS)
	{
		*HwRate = RATE_9MBPS; 
		*HwModulation = HW_MODULATION_OFDM;
	}
	else if (HwRatesBitmap & HW_BIT_RATE_6MBPS)
	{
		*HwRate = RATE_6MBPS; 
		*HwModulation = HW_MODULATION_OFDM;
	}
	else if (HwRatesBitmap & HW_BIT_RATE_5_5MBPS)
	{
		*HwRate = RATE_5_5MBPS;
		*HwModulation = HW_MODULATION_PBCC;
	}
	else if (HwRatesBitmap & HW_BIT_RATE_2MBPS)
	{
		*HwRate = RATE_2MBPS; 
		*HwModulation = HW_MODULATION_PBCC;
	}
	else 
	{
		*HwRate = RATE_1MBPS; 
		*HwModulation = HW_MODULATION_PBCC;
	}
}

/* Note the difference between this function and the next one */
UINT32  whalUtils_GwsiRate2DRV_RATE (UINT32 gwsiRate)
{
	switch (gwsiRate)
	{
		/*
		 *	The handle for 33 removed !!!
		 */

		case GWSI_1Mbits:	return   	DRV_RATE_1M; /*	break; */
		case GWSI_2Mbits:	return 	DRV_RATE_2M;	/*break; */
		case GWSI_5_5Mbits:	return 	DRV_RATE_5_5M;  /*break; */
		case GWSI_6Mbits	:	return 	DRV_RATE_6M; /* break; */	
		case GWSI_9Mbits	:	return 	DRV_RATE_9M; /*	break; */	
		case GWSI_11Mbits:	return 	DRV_RATE_11M; /*	break; */	
		case GWSI_12Mbits:	return 	DRV_RATE_12M; /*	break; */	
		case GWSI_18Mbits:	return 	DRV_RATE_18M; /*	break; */	
		case GWSI_22Mbits:	return 	DRV_RATE_22M; /*	break; */	
		case GWSI_24Mbits:	return 	DRV_RATE_24M; /*	break; */	
		case GWSI_36Mbits:	return 	DRV_RATE_36M; /*	break; */	
		case GWSI_48Mbits:	return 	DRV_RATE_48M; /*	break; */	
		case GWSI_54Mbits:	return 	DRV_RATE_54M; /*	break; */	
		
		default:
			
			return   	DRV_RATE_1M;
		/*	break; */
	}
}

rateMask_e  whalUtils_GwsiRate2DRV_RATE_MASK (UINT32 gwsiRate)
{
	switch (gwsiRate)
	{
		case GWSI_1Mbits:	return  DRV_RATE_MASK_1_BARKER;	 
		case GWSI_2Mbits:	return 	DRV_RATE_MASK_2_BARKER;	      
		case GWSI_5_5Mbits:	return 	DRV_RATE_MASK_5_5_CCK;         
		case GWSI_6Mbits:	return 	DRV_RATE_MASK_6_OFDM;	    
		case GWSI_9Mbits:	return 	DRV_RATE_MASK_9_OFDM;	   
		case GWSI_11Mbits:	return 	DRV_RATE_MASK_11_CCK;	               
		case GWSI_12Mbits:	return 	DRV_RATE_MASK_12_OFDM;	        
		case GWSI_18Mbits:	return 	DRV_RATE_MASK_18_OFDM;	        
		case GWSI_22Mbits:	return 	DRV_RATE_MASK_22_PBCC;	       
		case GWSI_24Mbits:	return 	DRV_RATE_MASK_24_OFDM;	       
		case GWSI_36Mbits:	return 	DRV_RATE_MASK_36_OFDM;	       
		case GWSI_48Mbits:	return 	DRV_RATE_MASK_48_OFDM;	       
		case GWSI_54Mbits:	return 	DRV_RATE_MASK_54_OFDM;	       
																  
		default:			return  DRV_RATE_MASK_1_BARKER;
	}
}

UINT32  whalUtils_DRV_RATE2GwsiRate (UINT32 Rate)
{
	switch (Rate)
	{
		/*
		 *	The handle for 33 removed !!!
		 */

		case DRV_RATE_1M:	return   GWSI_1Mbits; /*	break;*/
		case DRV_RATE_2M:	return 	GWSI_2Mbits;  /*	break;*/
		case DRV_RATE_5_5M:	return 	GWSI_5_5Mbits;/*   break;*/
		case DRV_RATE_6M:	return 	GWSI_6Mbits	; /*	break;*/	
		case DRV_RATE_9M:	return 	GWSI_9Mbits	; /*	break;*/	
		case DRV_RATE_11M:	return 	GWSI_11Mbits; /*	break;*/	
		case DRV_RATE_12M:	return 	GWSI_12Mbits; /*	break;*/	
		case DRV_RATE_18M:	return 	GWSI_18Mbits; /*	break;*/	
		case DRV_RATE_22M:	return 	GWSI_22Mbits; /*	break;*/	
		case DRV_RATE_24M:	return 	GWSI_24Mbits; /*	break;*/	
		case DRV_RATE_36M:	return 	GWSI_36Mbits; /*	break;*/	
		case DRV_RATE_48M:	return 	GWSI_48Mbits; /*	break;*/	
		case DRV_RATE_54M:	return 	GWSI_54Mbits; /*	break;*/	
		
		default:
			return   	GWSI_1Mbits;
	/*		break; */
	}
}

UINT32  whalUtils_DRV_RATE_MASK2GwsiRate (rateMask_e rateMask)
{
	switch (rateMask)
	{
		case DRV_RATE_MASK_1_BARKER:	return  GWSI_1Mbits;	 
		case DRV_RATE_MASK_2_BARKER:	return 	GWSI_2Mbits;	      
		case DRV_RATE_MASK_5_5_CCK:	return 	GWSI_5_5Mbits;         
		case DRV_RATE_MASK_6_OFDM:	return 	GWSI_6Mbits;	    
		case DRV_RATE_MASK_9_OFDM:	return 	GWSI_9Mbits;	   
		case DRV_RATE_MASK_11_CCK:	return 	GWSI_11Mbits;	               
		case DRV_RATE_MASK_12_OFDM:	return 	GWSI_12Mbits;	        
		case DRV_RATE_MASK_18_OFDM:	return 	GWSI_18Mbits;	        
		case DRV_RATE_MASK_22_PBCC:	return 	GWSI_22Mbits;	       
		case DRV_RATE_MASK_24_OFDM:	return 	GWSI_24Mbits;	       
		case DRV_RATE_MASK_36_OFDM:	return 	GWSI_36Mbits;	       
		case DRV_RATE_MASK_48_OFDM:	return 	GWSI_48Mbits;	       
		case DRV_RATE_MASK_54_OFDM:	return 	GWSI_54Mbits;	       
																  
		default:			return  GWSI_1Mbits;
	}
}

