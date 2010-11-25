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
/*                                                                         */
/*    MODULE:          Rx.c                                                */
/*    PURPOSE:  Rx module functions                                        */
/*                                                                         */
/***************************************************************************/
#include "osTIType.h"
#include "paramIn.h"
#include "paramMng.h"
#include "paramOut.h" 
#include "memMngrEx.h"
#include "rx.h"
#include "osApi.h"
#include "DataCtrl_Api.h"
#include "Ctrl.h"
#include "802_11Defs.h"
#include "Ethernet.h"
#include "report.h"
#include "utils.h"
#include "mlmeApi.h"
#include "rsnApi.h"
#include "smeApi.h"
#include "siteMgrApi.h"
#include "GeneralUtil.h"   
#include "EvHandler.h"
#ifdef EXC_MODULE_INCLUDED
#include "excMngr.h"
#endif
#include "TNETW_Driver_api.h"

#define EAPOL_PACKET					0x8E88
#define IAPP_PACKET						0x0000
#define PREAUTH_EAPOL_PACKET			0xC788

/* CallBack for recieving packet from rxXfer */
static void rxData_ReceivePacket ( TI_HANDLE   hRxData,TI_STATUS   aStatus,const void *aFrame,
                                   UINT16      aLength,UINT32      aRate,UINT8       aRCPI,
                                   UINT8       aChannel,void       *Reserved,UINT32      aFlags);

static void *rxData_RequestForBuffer (TI_HANDLE   hRxData,UINT16 aLength, UINT32 uEncryptionFlag);

#if 0
static TI_STATUS rxData_checkBssIdAndBssType(TI_HANDLE hRxData, dot11_header_t* dot11_header,
										  macAddress_t **rxBssid, bssType_e	*currBssType,
										  macAddress_t	*currBssId);
#endif
static TI_STATUS rxData_convertWlanToEthHeader (TI_HANDLE hRxData, mem_MSDU_T *pMsdu, UINT16 * etherType);
static void rxData_dataMsduDisptcher(TI_HANDLE hRxData, mem_MSDU_T *pMsdu, Rx_attr_t* pRxAttr);
static void	rxData_discardMsdu(TI_HANDLE hRxData, mem_MSDU_T *pMsdu, Rx_attr_t* pRxAttr);
static void	rxData_discardMsduVlan(TI_HANDLE hRxData, mem_MSDU_T *pMsdu, Rx_attr_t* pRxAttr);
static void rxData_rcvMsduInOpenNotify(TI_HANDLE hRxData, mem_MSDU_T *pMsdu, Rx_attr_t* pRxAttr);
static void rxData_rcvMsduEapol(TI_HANDLE hRxData, mem_MSDU_T *pMsdu, Rx_attr_t* pRxAttr);
static void rxData_rcvMsduData(TI_HANDLE hRxData, mem_MSDU_T *pMsdu, Rx_attr_t* pRxAttr);
static TI_STATUS rxData_enableDisableRxDataFilters(TI_HANDLE hRxData, BOOL enabled);
static TI_STATUS rxData_addRxDataFilter(TI_HANDLE hRxData, rxDataFilterRequest_t * request);
static TI_STATUS rxData_removeRxDataFilter(TI_HANDLE hRxData, rxDataFilterRequest_t * request);

#ifdef EXC_MODULE_INCLUDED
static void rxData_rcvMsduIapp(TI_HANDLE hRxData, mem_MSDU_T *pMsdu, Rx_attr_t* pRxAttr);
#endif
#ifdef TI_DBG
static void rxData_printRxThroughput(TI_HANDLE hRxData);
#endif



/*************************************************************************
*                        rxData_create                                   *
**************************************************************************
* DESCRIPTION:	This function initializes the Rx data module.                 
*                                                      
* INPUT:		hOs - handle to Os Abstraction Layer
*				msduReceiveCB - call back function that return to 
*				configMngr in order to register in the Hal
* OUTPUT:      
*
* RETURN:		Handle to the allocated Rx data control block
************************************************************************/
TI_HANDLE rxData_create ( TI_HANDLE hOs)
{
	rxData_t *hRxData;

	/* check parameters validity */
	if( hOs  == NULL)
	{
	    WLAN_OS_REPORT(("FATAL ERROR: rxData_create(): OS handle Error - Aborting\n"));
		return NULL;
	}
	

	/* alocate Rx module control block */
	hRxData = os_memoryAlloc(hOs, (sizeof(rxData_t)));

	if( !hRxData )
	{
		utils_nullMemoryFree(hOs, hRxData, sizeof(rxData_t));
	    WLAN_OS_REPORT(("FATAL ERROR: rxData_create(): Error Creating Rx Module - Aborting\n"));
		return(NULL);
	}

	/* reset Rx control block */
	os_memoryZero(hOs, hRxData, (sizeof(rxData_t)));

    hRxData->RxEventDistributor = DistributorMgr_Create(hOs,MAX_RX_NOTIF_REQ_ELMENTS);

	hRxData->hOs = hOs;

    /* allocate timer for debug throughput */
#ifdef TI_DBG
    hRxData->hThroughputTimer = os_timerCreate (hOs, rxData_printRxThroughput, hRxData);
    if (!hRxData->hThroughputTimer)
    {
        utils_nullMemoryFree (hOs, hRxData, sizeof(rxData_t));
        return NULL;
    }
    hRxData->rxThroughputTimerEnable = FALSE;
#endif

	return(hRxData);
}

/***************************************************************************
*							rxData_config				                   *
****************************************************************************
* DESCRIPTION:	This function configures the Rx Data module		
* 
* INPUTS:		hRxData - The object
*				hCtrlData - Handle to the Ctrl Data object
*				hMlme - Handle to the Mlme object
*				hRsn - Handle to the Rsn object
*				hOs - Handle to the Os Abstraction Layer
*				hReport - Handle to the Report object
* 				hMemMngr - Handle to the MemMngr object
* OUTPUT:		
* 
* RETURNS:		OK - Configuration succesfull
*				NOK - Configuration unsuccesfull
***************************************************************************/

TI_STATUS rxData_config(TI_HANDLE		hRxData, 
					 TI_HANDLE			hCtrlData, 
					 TI_HANDLE			hTxData,
				 	 TI_HANDLE	 		hTnetwDrv,
                        TI_HANDLE   hHalCtrl,
					 TI_HANDLE			hMlme, 
					 TI_HANDLE			hRsn, 
					 TI_HANDLE			hSiteMgr, 
					 TI_HANDLE			hExcMngr, 
					 TI_HANDLE			hOs, 
					 TI_HANDLE			hReport,
					 TI_HANDLE			hMemMngr,
                        TI_HANDLE   hEvHandler,
                        rxDataInitParams_t * rxDataInitParams)
{
	rxData_t *pRxData = (rxData_t *)hRxData;
    int i;

	/* check parameters validity */
	if( hRxData == NULL || hCtrlData == NULL || hMlme == NULL || hRsn == NULL || hHalCtrl == NULL ||
		hSiteMgr == NULL || hOs == NULL || hReport == NULL || hTxData == NULL || hTnetwDrv == NULL)
	{
	    WLAN_OS_REPORT(("FATAL ERROR: rxData_config(): Parameters Error - Aborting\n"));
		return NOK;
	}
	pRxData->hCtrlData = hCtrlData; 
	pRxData->hTxData = hTxData;
	pRxData->hTnetwDrv = hTnetwDrv;
    pRxData->hHalCtrl = hHalCtrl;
	pRxData->hMlme = hMlme; 
	pRxData->hRsn = hRsn;
	pRxData->hSiteMgr = hSiteMgr;
	pRxData->hOs = hOs;
	pRxData->hReport = hReport;
	pRxData->hMemMngr = hMemMngr;
	pRxData->hExcMgr = hExcMngr;
    pRxData->hEvHandler = hEvHandler;
	
	pRxData->rxDataExcludeUnencrypted = DEF_EXCLUDE_UNENCYPTED; 
    pRxData->rxDataExludeBroadcastUnencrypted = DEF_EXCLUDE_UNENCYPTED;
	pRxData->rxDataEapolDestination = DEF_EAPOL_DESTINATION;
	pRxData->rxDataPortStatus = DEF_RX_PORT_STATUS;

	/*
	 * configure rx data dispatcher 
	 */


	/* port status close */
	pRxData->rxData_dispatchMsdu[CLOSE][DATA_DATA_PACKET]  = &rxData_discardMsdu;       /* data  */
	pRxData->rxData_dispatchMsdu[CLOSE][DATA_EAPOL_PACKET] = &rxData_discardMsdu;       /* eapol */
	pRxData->rxData_dispatchMsdu[CLOSE][DATA_IAPP_PACKET]  = &rxData_discardMsdu;       /* Iapp  */
    pRxData->rxData_dispatchMsdu[CLOSE][DATA_VLAN_PACKET]  = &rxData_discardMsduVlan;   /* VLAN  */

	/* port status open notify */
	pRxData->rxData_dispatchMsdu[OPEN_NOTIFY][DATA_DATA_PACKET]  = &rxData_rcvMsduInOpenNotify; /* data  */ 
	pRxData->rxData_dispatchMsdu[OPEN_NOTIFY][DATA_EAPOL_PACKET] = &rxData_rcvMsduInOpenNotify; /* eapol */ 
	pRxData->rxData_dispatchMsdu[OPEN_NOTIFY][DATA_IAPP_PACKET]  = &rxData_rcvMsduInOpenNotify; /* Iapp  */ 
    pRxData->rxData_dispatchMsdu[OPEN_NOTIFY][DATA_VLAN_PACKET]  = &rxData_discardMsduVlan;     /* VLAN  */

	/* port status open eapol */
	pRxData->rxData_dispatchMsdu[OPEN_EAPOL][DATA_DATA_PACKET]  = &rxData_discardMsdu;      /* data  */ 
	pRxData->rxData_dispatchMsdu[OPEN_EAPOL][DATA_EAPOL_PACKET] = &rxData_rcvMsduEapol;     /* eapol */ 
	pRxData->rxData_dispatchMsdu[OPEN_EAPOL][DATA_IAPP_PACKET]  = &rxData_discardMsdu;      /* Iapp  */ 
    pRxData->rxData_dispatchMsdu[OPEN_EAPOL][DATA_VLAN_PACKET]  = &rxData_discardMsduVlan;  /* VLAN  */

	/* port status open */
	pRxData->rxData_dispatchMsdu[OPEN][DATA_DATA_PACKET]  = &rxData_rcvMsduData;    /* data  */ 
	pRxData->rxData_dispatchMsdu[OPEN][DATA_EAPOL_PACKET] = &rxData_rcvMsduEapol;   /* eapol */ 
#ifdef EXC_MODULE_INCLUDED
	pRxData->rxData_dispatchMsdu[OPEN][DATA_IAPP_PACKET]  = &rxData_rcvMsduIapp;    /* Iapp  */ 
#else
	pRxData->rxData_dispatchMsdu[OPEN][DATA_IAPP_PACKET]  = &rxData_rcvMsduData;    /* Iapp  */ 
#endif
    pRxData->rxData_dispatchMsdu[OPEN][DATA_VLAN_PACKET]  = &rxData_discardMsduVlan;/* VLAN  */

    /* register CB's for request buffer and receive CB to the lower layers */
	TnetwDrv_Register_CB(pRxData->hTnetwDrv,TNETW_DRIVER_RX_RECEIVE_PACKET,(void *)rxData_ReceivePacket,hRxData);
	TnetwDrv_Register_CB(pRxData->hTnetwDrv,TNETW_DRIVER_RX_REQUEST_FOR_BUFFER,(void *)rxData_RequestForBuffer,hRxData);

    /* init rx data filters */
    pRxData->filteringEnabled = rxDataInitParams->rxDataFiltersEnabled;
    pRxData->filteringDefaultAction = rxDataInitParams->rxDataFiltersDefaultAction;

	whalCtrl_setRxDataFiltersParams(pRxData->hHalCtrl, pRxData->filteringEnabled, pRxData->filteringDefaultAction);

	for (i = 0; i < MAX_DATA_FILTERS; ++i)
	{
		if (rxDataInitParams->rxDataFilterRequests[i].maskLength > 0)
		{
			if (rxData_addRxDataFilter(hRxData, &rxDataInitParams->rxDataFilterRequests[i]) != OK)
			{
				WLAN_REPORT_ERROR(pRxData->hReport, RX_DATA_MODULE_LOG, 
						("%s: Invalid Rx Data Filter configured at init stage (at index %d)!\n", __FUNCTION__, i));
			}
		}
	}

  #ifdef TI_DBG
	/* reset counters */
	rxData_resetCounters(pRxData);
	rxData_resetDbgCounters(pRxData);
  #endif

	WLAN_REPORT_INIT(pRxData->hReport, RX_DATA_MODULE_LOG, 
		(".....Rx Data configured successfully\n"));

	return OK;

}

/***************************************************************************
*							rxData_unLoad				                   *
****************************************************************************
* DESCRIPTION:	This function unload the Rx data module. 
* 
* INPUTS:		hRxData - the object
*		
* OUTPUT:		
* 
* RETURNS:		OK - Unload succesfull
*				NOK - Unload unsuccesfull
***************************************************************************/


TI_STATUS rxData_unLoad(TI_HANDLE hRxData)
{
	rxData_t *pRxData = (rxData_t *)hRxData;

	/* check parameters validity */
	if( pRxData == NULL )
	{
		WLAN_REPORT_ERROR(pRxData->hReport, RX_DATA_MODULE_LOG, 
			(" rxData_unLoad()	: Illegal value for hRxData\n"));
		return NOK;
	}

    DistributorMgr_Destroy(pRxData->RxEventDistributor);

    /* destroy periodic rx throughput timer */
  #ifdef TI_DBG
    utils_nullTimerDestroy (pRxData->hOs, pRxData->hThroughputTimer);
  #endif

	/* free Rx Data controll block */
	os_memoryFree(pRxData->hOs, pRxData, sizeof(rxData_t));

	return OK;
}


/***************************************************************************
*							rxData_stop					                   *
****************************************************************************
* DESCRIPTION:	this function stop the rx data. 
* 
* INPUTS:		hRxData - the object
*		
* OUTPUT:		
* 
* RETURNS:		OK - stop succesfull
*				NOK - stop unsuccesfull
***************************************************************************/

TI_STATUS rxData_stop(TI_HANDLE hRxData)
{
	rxData_t *pRxData = (rxData_t *)hRxData;

	/* check parameters validity */
	if( pRxData == NULL )
	{
		WLAN_REPORT_ERROR(pRxData->hReport, RX_DATA_MODULE_LOG, 
			(" rxData_stop() : Illegal value for hRxData\n"));
		return NOK;
	}

	pRxData->rxDataExcludeUnencrypted = DEF_EXCLUDE_UNENCYPTED; 
    pRxData->rxDataExludeBroadcastUnencrypted = DEF_EXCLUDE_UNENCYPTED;
	pRxData->rxDataEapolDestination = DEF_EAPOL_DESTINATION;
	pRxData->rxDataPortStatus = DEF_RX_PORT_STATUS;

#ifdef TI_DBG
	/* reset counters */
	/*rxData_resetCounters(pRxData);*/
	/*rxData_resetDbgCounters(pRxData);*/

    /* stop throughput timer */
    if (pRxData->rxThroughputTimerEnable)
    {
        os_timerStop (pRxData->hOs, pRxData->hThroughputTimer);
        pRxData->rxThroughputTimerEnable = FALSE;
    }
#endif

	WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, 
		(" rxData_stop() :  Succeeded.\n"));

	return OK;

}

/***************************************************************************
*							rxData_getParam				                   *
****************************************************************************
* DESCRIPTION:	get a specific parameter
* 
* INPUTS:		hRxData - the object
*				
* OUTPUT:		pParamInfo - structure which include the value of 
*				the requested parameter
* 
* RETURNS:		OK
*				NOK
***************************************************************************/

TI_STATUS rxData_getParam(TI_HANDLE hRxData, paramInfo_t *pParamInfo)
{
	rxData_t *pRxData = (rxData_t *)hRxData;

	/* check handle validity */
	if( pRxData == NULL  )
	{
		WLAN_REPORT_ERROR(pRxData->hReport, RX_DATA_MODULE_LOG,
			(" rxData_getParam() :	Illegal parametrs value \n"));
		return NOK;
	}

	switch (pParamInfo->paramType)
	{
		case RX_DATA_EXCLUDE_UNENCRYPTED_PARAM:
			pParamInfo->content.rxDataExcludeUnencrypted = pRxData->rxDataExcludeUnencrypted;
			break;

		case RX_DATA_EAPOL_DESTINATION_PARAM:
			pParamInfo->content.rxDataEapolDestination = pRxData->rxDataEapolDestination;
			break;

		case RX_DATA_PORT_STATUS_PARAM:
			pParamInfo->content.rxDataPortStatus = pRxData->rxDataPortStatus;
			break;

		case RX_DATA_COUNTERS_PARAM:
			pParamInfo->content.siteMgrTiWlanCounters.RecvOk = pRxData->rxDataCounters.RecvOk;				
			pParamInfo->content.siteMgrTiWlanCounters.DirectedBytesRecv = pRxData->rxDataCounters.DirectedBytesRecv;		
			pParamInfo->content.siteMgrTiWlanCounters.DirectedFramesRecv = pRxData->rxDataCounters.DirectedFramesRecv;		
			pParamInfo->content.siteMgrTiWlanCounters.MulticastBytesRecv = pRxData->rxDataCounters.MulticastBytesRecv;		
			pParamInfo->content.siteMgrTiWlanCounters.MulticastFramesRecv = pRxData->rxDataCounters.MulticastFramesRecv;	
			pParamInfo->content.siteMgrTiWlanCounters.BroadcastBytesRecv = pRxData->rxDataCounters.BroadcastBytesRecv;		
			pParamInfo->content.siteMgrTiWlanCounters.BroadcastFramesRecv = pRxData->rxDataCounters.BroadcastFramesRecv;	
			break;

        case RX_DATA_GET_RX_DATA_FILTERS_STATISTICS:
            whalCtrl_getRxDataFiltersStatistics(pRxData->hHalCtrl, 
                    pParamInfo->content.interogateCmdCBParams.CB_Func,
                    pParamInfo->content.interogateCmdCBParams.CB_handle, 
                    pParamInfo->content.interogateCmdCBParams.CB_buf);

            break;


		default:
			return (PARAM_NOT_SUPPORTED);
/*			WLAN_REPORT_ERROR(pRxData->hReport, RX_DATA_MODULE_LOG, 
				(" rxData_getParam() : PARAMETER NOT SUPPORTED \n"));
			return NOK;
			break; - unreachable */
	}


	return (OK);

}

/***************************************************************************
*							rxData_setParam				                   *
****************************************************************************
* DESCRIPTION:	set a specific parameter
* 
* INPUTS:		hRxData - the object
*				pParamInfo - structure which include the value to set for 
*				the requested parameter
*		
* OUTPUT:		
* 
* RETURNS:		OK
*				NOK
***************************************************************************/

TI_STATUS rxData_setParam(TI_HANDLE hRxData, paramInfo_t *pParamInfo)
{
	rxData_t *pRxData = (rxData_t *)hRxData;

	/* check handle validity */
	if( pRxData == NULL  )
	{
		WLAN_REPORT_ERROR(pRxData->hReport, RX_DATA_MODULE_LOG,
			(" rxData_setParam(): Illegal parametrs value \n"));
		return NOK;
	}

	switch (pParamInfo->paramType)
	{
		case RX_DATA_EXCLUDE_UNENCRYPTED_PARAM:
			pRxData->rxDataExcludeUnencrypted = pParamInfo->content.rxDataExcludeUnencrypted;
			break;
        case RX_DATA_EXCLUDE_BROADCAST_UNENCRYPTED_PARAM:
            pRxData->rxDataExludeBroadcastUnencrypted = pParamInfo->content.rxDataExcludeUnencrypted;
            break;
		case RX_DATA_EAPOL_DESTINATION_PARAM:
			pRxData->rxDataEapolDestination = pParamInfo->content.rxDataEapolDestination;
			break;

		case RX_DATA_PORT_STATUS_PARAM:
			pRxData->rxDataPortStatus = pParamInfo->content.rxDataPortStatus;
			break;

        case RX_DATA_ENABLE_DISABLE_RX_DATA_FILTERS:
            return rxData_enableDisableRxDataFilters(hRxData, pParamInfo->content.rxDataFilterEnableDisable);

        case RX_DATA_ADD_RX_DATA_FILTER:
        {
            TIWLAN_DATA_FILTER_REQUEST * pRequest = &pParamInfo->content.rxDataFilterRequest;
            rxDataFilterRequest_t filterRequest;

            filterRequest.offset = pRequest->Offset;
            filterRequest.maskLength = pRequest->MaskLength;
            filterRequest.patternLength = pRequest->PatternLength;

            os_memoryCopy(pRxData->hOs, (PVOID) filterRequest.mask, (PVOID) pRequest->Mask, sizeof(filterRequest.mask));
            os_memoryCopy(pRxData->hOs, (PVOID) filterRequest.pattern, (PVOID) pRequest->Pattern, sizeof(filterRequest.pattern));

            return rxData_addRxDataFilter(hRxData, &filterRequest);
        }

        case RX_DATA_REMOVE_RX_DATA_FILTER:
        {
            TIWLAN_DATA_FILTER_REQUEST * pRequest = &pParamInfo->content.rxDataFilterRequest;
            rxDataFilterRequest_t filterRequest;

            filterRequest.offset = pRequest->Offset;
            filterRequest.maskLength = pRequest->MaskLength;
            filterRequest.patternLength = pRequest->PatternLength;
            
            os_memoryCopy(pRxData->hOs, (PVOID) filterRequest.mask, (PVOID) pRequest->Mask, sizeof(filterRequest.mask));
            os_memoryCopy(pRxData->hOs, (PVOID) filterRequest.pattern, (PVOID) pRequest->Pattern, sizeof(filterRequest.pattern));

            return rxData_removeRxDataFilter(hRxData, &filterRequest);
        }

		default:
			return (PARAM_NOT_SUPPORTED);
/*			WLAN_REPORT_ERROR(pRxData->hReport, RX_DATA_MODULE_LOG, 
				(" rxData_setParam() : PARAMETER NOT SUPPORTED \n"));
			return NOK;
			break; - unreachable */
	}

	return (OK);
}


/***************************************************************************
*                     rxData_enableDisableRxDataFilters		               *
****************************************************************************
* DESCRIPTION:	
*				
* 
* INPUTS:		
*				
*				
*		
* OUTPUT:		
* 
* RETURNS:		
*				
***************************************************************************/
static TI_STATUS rxData_enableDisableRxDataFilters(TI_HANDLE hRxData, BOOL enabled)
{
    rxData_t * pRxData = (rxData_t *) hRxData;

    /* assert 0 or 1 */
    if (enabled != 0)
        enabled = 1;

    if (enabled == pRxData->filteringEnabled)
        return OK;

    pRxData->filteringEnabled = enabled;

	
	return (TI_STATUS) whalCtrl_setRxDataFiltersParams(pRxData->hHalCtrl, pRxData->filteringEnabled, pRxData->filteringDefaultAction);
}

/***************************************************************************
*                          findFilterRequest     		                   *
****************************************************************************
* DESCRIPTION:	
*				
* 
* INPUTS:		
*				
*				
*		
* OUTPUT:		
* 
* RETURNS:		
*				
***************************************************************************/
static int findFilterRequest(TI_HANDLE hRxData, rxDataFilterRequest_t * request)
{
    rxData_t * pRxData = (rxData_t *) hRxData;
    int i;
    
    for (i = 0; i < MAX_DATA_FILTERS; ++i)
    {
        if (pRxData->isFilterSet[i])
        {
            if ((pRxData->filterRequests[i].offset == request->offset) &&
                (pRxData->filterRequests[i].maskLength == request->maskLength) &&
                (pRxData->filterRequests[i].patternLength == request->patternLength))
            {
                if ((os_memoryCompare(pRxData->hOs, pRxData->filterRequests[i].mask, request->mask, request->maskLength) == 0) &&
                    (os_memoryCompare(pRxData->hOs, pRxData->filterRequests[i].pattern, request->pattern, request->patternLength) == 0))
                    return i;
            }
        }
    }

    return -1;
}

/***************************************************************************
*                            closeFieldPattern                             *
****************************************************************************
* DESCRIPTION:	
*				
* 
* INPUTS:		
*				
*				
*		
* OUTPUT:		
* 
* RETURNS:		
*				
***************************************************************************/
static void closeFieldPattern(rxData_t * pRxData, rxDataFilterFieldPattern_t * fieldPattern, UINT8 * fieldPatterns, UINT8 * lenFieldPatterns)
{
    fieldPatterns[*lenFieldPatterns] = fieldPattern->offset;
    *lenFieldPatterns += sizeof(fieldPattern->offset);

    fieldPatterns[*lenFieldPatterns] = fieldPattern->length;
    *lenFieldPatterns += sizeof(fieldPattern->length);

    fieldPatterns[*lenFieldPatterns] = fieldPattern->flag;
    *lenFieldPatterns += sizeof(fieldPattern->flag);

    os_memoryCopy(pRxData->hOs, fieldPatterns + *lenFieldPatterns, fieldPattern->pattern, fieldPattern->length);
    *lenFieldPatterns += fieldPattern->length;

    /* if the pattern bit mask is enabled add it to the end of the request */
    if ((fieldPattern->flag & RX_DATA_FILTER_FLAG_USE_BIT_MASK) == RX_DATA_FILTER_FLAG_USE_BIT_MASK)
    {
        os_memoryCopy(pRxData->hOs, fieldPatterns + *lenFieldPatterns, fieldPattern->mask, fieldPattern->length);
        *lenFieldPatterns += fieldPattern->length;
    }

    WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, ("%s: Closed field pattern, length = %d, total length = %d, pattern bit mask = %d.\n", __FUNCTION__, fieldPattern->length, *lenFieldPatterns, ((fieldPattern->flag & RX_DATA_FILTER_FLAG_USE_BIT_MASK) == RX_DATA_FILTER_FLAG_USE_BIT_MASK)));
}


/***************************************************************************
*                         parseRxDataFilterRequest                         *
****************************************************************************
* DESCRIPTION:	
*				
* 
* INPUTS:		
*				
*				
*		
* OUTPUT:		
* 
* RETURNS:		
*				
***************************************************************************/
static int parseRxDataFilterRequest(TI_HANDLE hRxData, rxDataFilterRequest_t * request, UINT8 * numFieldPatterns, UINT8 * lenFieldPatterns, UINT8 * fieldPatterns)
{
    rxData_t * pRxData = (rxData_t *) hRxData;

    int maskIter;
    int patternIter = 0;

    /* used to store field patterns while they are built */
    BOOL isBuildingFieldPattern = FALSE;
    rxDataFilterFieldPattern_t fieldPattern;

    for (maskIter = 0; maskIter < request->maskLength * BIT_TO_BYTE_FACTOR; ++maskIter)
    {
        /* which byte in the mask and which bit in the byte we're at */
        int bit = maskIter % BIT_TO_BYTE_FACTOR;
        int byte = maskIter / BIT_TO_BYTE_FACTOR;

        /* is the bit in the mask set */
        BOOL isSet = ((request->mask[byte] & (1 << bit)) == (1 << bit));

        WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG,
                ("%s: MaskIter = %d, Byte = %d, Bit = %d, isSet = %d\n", __FUNCTION__, maskIter, byte, bit, isSet));

        /* if we're in the midst of building a field pattern, we need to close in case */
        /* the current bit is not set or we've reached the ethernet header boundary */
        if (isBuildingFieldPattern)
        {
            if ((isSet == FALSE) || (request->offset + maskIter == RX_DATA_FILTER_ETHERNET_HEADER_BOUNDARY))
            {
                closeFieldPattern(hRxData, &fieldPattern, fieldPatterns, lenFieldPatterns);

                isBuildingFieldPattern = FALSE;
            }
        }

        /* nothing to do in case the bit is not set */
        if (isSet)
        {
            /* if not already building a field pattern, create a new one */
            if (isBuildingFieldPattern == FALSE)
            {
                WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG,
                        ("%s: Creating a new field pattern.\n", __FUNCTION__));

                isBuildingFieldPattern = TRUE;
                ++(*numFieldPatterns);

                if (*numFieldPatterns > RX_DATA_FILTER_MAX_FIELD_PATTERNS)
                {
                    WLAN_REPORT_ERROR(pRxData->hReport, RX_DATA_MODULE_LOG,
                            ("%s: Invalid filter, too many field patterns, maximum of %u is allowed!\n", __FUNCTION__, RX_DATA_FILTER_MAX_FIELD_PATTERNS));

                    return NOK;
                }

                fieldPattern.offset = request->offset + maskIter;
                fieldPattern.length = 0;

                /* we don't support the mask per bit feature yet. */
                fieldPattern.flag = RX_DATA_FILTER_FLAG_NO_BIT_MASK;

                /* first 14 bits are used for the Ethernet header, rest for the IP header */
                if (fieldPattern.offset < RX_DATA_FILTER_ETHERNET_HEADER_BOUNDARY)
                {
                    fieldPattern.flag |= RX_DATA_FILTER_FLAG_ETHERNET_HEADER;
                }
                else
                {
                    fieldPattern.flag |= RX_DATA_FILTER_FLAG_IP_HEADER;
                    fieldPattern.offset -= RX_DATA_FILTER_ETHERNET_HEADER_BOUNDARY;
                }

                WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG,
                        ("%s: offset = %d, flag = %d.\n", __FUNCTION__, fieldPattern.offset, fieldPattern.flag));
            }

            /* check that the pattern is long enough */
            if (patternIter > request->patternLength)
            {
                WLAN_REPORT_ERROR(pRxData->hReport, RX_DATA_MODULE_LOG,
                        ("%s: Invalid filter, mask and pattern lengths are not consistent!\n", __FUNCTION__));

                return NOK;
            }

            /* add the current pattern byte to the field pattern */
            fieldPattern.pattern[fieldPattern.length++] = request->pattern[patternIter++];

            /* check pattern matching boundary */
            if (fieldPattern.offset + fieldPattern.length >= RX_DATA_FILTER_FILTER_BOUNDARY)
            {
                WLAN_REPORT_ERROR(pRxData->hReport, RX_DATA_MODULE_LOG,
                        ("%s: Invalid filter, pattern matching cannot exceed first %u characters.\n", __FUNCTION__, RX_DATA_FILTER_FILTER_BOUNDARY));

                return NOK;
            }
        }
    }

    /* check that the pattern is long enough */
    if (patternIter != request->patternLength)
    {
        WLAN_REPORT_ERROR(pRxData->hReport, RX_DATA_MODULE_LOG,
            ("%s: Invalid filter, mask and pattern lengths are not consistent!\n", __FUNCTION__));

        return NOK;
    }

    /* close the last field pattern if needed */
    if (isBuildingFieldPattern)
    {
        closeFieldPattern(hRxData, &fieldPattern, fieldPatterns, lenFieldPatterns);
    }

    return OK;
}


/***************************************************************************
*                           rxData_setRxDataFilter		                   *
****************************************************************************
* DESCRIPTION:	
*				
* 
* INPUTS:		
*				
*				
*		
* OUTPUT:		
* 
* RETURNS:		
*				
***************************************************************************/
static TI_STATUS rxData_addRxDataFilter(TI_HANDLE hRxData, rxDataFilterRequest_t * request)
{
    rxData_t * pRxData = (rxData_t *) hRxData;

    /* firmware request fields */
    UINT8 index = 0;
    UINT8 numFieldPatterns = 0;
    UINT8 lenFieldPatterns = 0;
    UINT8 fieldPatterns[MAX_DATA_FILTER_SIZE];

	WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG,
   ("rxData_addRxDataFilter, offset=0x%x, maskLength=0x%x, patternLength=0x%x\n",
					request->offset,
					request->maskLength,
					request->patternLength));

	WLAN_REPORT_HEX_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, request->mask, request->maskLength);
	WLAN_REPORT_HEX_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, request->pattern, request->patternLength);

    /* does the filter already exist? */
    if (findFilterRequest(hRxData, request) >= 0)
    {
        WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG,
                ("%s: Filter already exists.\n", __FUNCTION__));

        return RX_FILTER_ALREADY_EXISTS;
    }

    /* find place for insertion */
    for (index = 0; index < MAX_DATA_FILTERS; ++index)
    {
        if (pRxData->isFilterSet[index] == FALSE)
            break;
    }

    /* are all filter slots taken? */
    if (index == MAX_DATA_FILTERS)
    {
        WLAN_REPORT_ERROR(pRxData->hReport, RX_DATA_MODULE_LOG,
                ("%s: No place to insert filter!\n", __FUNCTION__));

        return RX_NO_AVAILABLE_FILTERS;
    }

    WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG,
            ("%s: Inserting filter at index %d.\n", __FUNCTION__, index));

    /* parse the filter request into discrete field patterns */
    if (parseRxDataFilterRequest(hRxData, request, &numFieldPatterns, &lenFieldPatterns, fieldPatterns) != OK)
        return NOK;

    if (numFieldPatterns == 0)
        return NOK;

    /* store configuration for future manipulation */
    pRxData->isFilterSet[index] = TRUE;
    os_memoryCopy(pRxData->hOs, &pRxData->filterRequests[index], request, sizeof(pRxData->filterRequests[index]));

    /* send configuration to firmware */
	return (TI_STATUS) whalCtrl_setRxDataFilter(pRxData->hHalCtrl, index, ADD_FILTER, FILTER_SIGNAL, 
			numFieldPatterns, lenFieldPatterns, fieldPatterns);

	return OK;
	
}

/***************************************************************************
*                         rxData_removeRxDataFilter		                   *
****************************************************************************
* DESCRIPTION:	
*				
* 
* INPUTS:		
*				
*				
*		
* OUTPUT:		
* 
* RETURNS:		
*				
***************************************************************************/
static TI_STATUS rxData_removeRxDataFilter(TI_HANDLE hRxData, rxDataFilterRequest_t * request)
{
    rxData_t * pRxData = (rxData_t *) hRxData;

    int index = findFilterRequest(hRxData, request);

	WLAN_REPORT_INFORMATION(pRxData->hOs, RX_DATA_MODULE_LOG,
		("rxData_removeRxDataFilter, offset=0x%x, maskLength=0x%x, patternLength=0x%x\n",
					request->offset,
					request->maskLength,
					request->patternLength));

	WLAN_REPORT_HEX_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, request->mask, request->maskLength);
	WLAN_REPORT_HEX_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, request->pattern, request->patternLength);

    /* does the filter exist? */
    if (index < 0)
    {
        WLAN_REPORT_WARNING(pRxData->hOs, RX_DATA_MODULE_LOG,
                ("%s: Remove data filter request received but the specified filter was not found!", __FUNCTION__));

        return RX_FILTER_DOES_NOT_EXIST;
    }

    WLAN_REPORT_INFORMATION(pRxData->hOs, RX_DATA_MODULE_LOG,
            ("%s: Removing filter at index %d.", __FUNCTION__, index));

    pRxData->isFilterSet[index] = FALSE;

	return (TI_STATUS) whalCtrl_setRxDataFilter(pRxData->hHalCtrl, index, REMOVE_FILTER, 
				FILTER_SIGNAL, 0, 0, NULL);

	return OK;

	
}

/***************************************************************************
*						rxData_DistributorRxEvent		                   *
****************************************************************************
* DESCRIPTION:	
*				
* 
* INPUTS:		
*				
*				
*		
* OUTPUT:		
* 
* RETURNS:		
*				
***************************************************************************/
static VOID rxData_DistributorRxEvent(rxData_t *pRxData,UINT16 Mask,int DataLen)
{
    DistributorMgr_EventCall(pRxData->RxEventDistributor,Mask,DataLen);
}

/***************************************************************************
*						rxData_RegNotif         		                   *
****************************************************************************/
TI_HANDLE rxData_RegNotif(TI_HANDLE hRxData,UINT16 EventMask,GeneralEventCall_t CallBack,TI_HANDLE context,UINT32 Cookie)
{
    rxData_t *pRxData = (rxData_t *)hRxData;
    if (!hRxData)
        return NULL;
    return DistributorMgr_Reg(pRxData->RxEventDistributor,EventMask,(TI_HANDLE)CallBack,context,Cookie);
}

/***************************************************************************
*						rxData_AddToNotifMask      		                   *
****************************************************************************/
TI_STATUS rxData_AddToNotifMask(TI_HANDLE hRxData,TI_HANDLE Notifh,UINT16 EventMask)
{
    rxData_t *pRxData = (rxData_t *)hRxData;
    if (!hRxData)
        return NOK;
    return DistributorMgr_AddToMask(pRxData->RxEventDistributor,Notifh,EventMask);
}


/***************************************************************************
*						rxData_UnRegNotif         		                   *
****************************************************************************/
TI_STATUS rxData_UnRegNotif(TI_HANDLE hRxData,TI_HANDLE RegEventHandle)
{
    TI_STATUS status;
    rxData_t *pRxData = (rxData_t *)hRxData;
    
    if (!hRxData)
        return NOK;

    status = DistributorMgr_UnReg(pRxData->RxEventDistributor,RegEventHandle);
    return (status);
}


/***************************************************************************
*						rxData_receiveMsduFromWlan		                   *
****************************************************************************
* DESCRIPTION:	this function is called by the GWSI for each received msdu.
*				It filter and distribute the received msdu. 
* 
* INPUTS:		hRxData - the object
*				pMsdu - the received msdu.
*				pRxAttr - Rx attributes
*		
* OUTPUT:		
* 
* RETURNS:		
***************************************************************************/
void rxData_receiveMsduFromWlan(TI_HANDLE hRxData, mem_MSDU_T *pMsdu, Rx_attr_t* pRxAttr)
{
	rxData_t *pRxData = (rxData_t *)hRxData;
	macAddress_t		address3;
	dot11_header_t		*pDot11Hdr;
	UINT16 tmpFCtrl;

	WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, 
			(" rxData_receiveMsduFromWlan() : pRxAttr->packetType = %d\n", pRxAttr->packetType));

	switch (pRxAttr->packetType)
	{
	case RX_PACKET_TYPE_MANAGEMENT:

		WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, 
				(" rxData_receiveMsduFromWlan() : Received management msdu len = %d\n", pMsdu->dataLen));

		/* update siteMngr 
		 *
		 * the BSSID in mgmt frames is always addr3 in the header 
		 * must copy address3 since msdu is freed in mlmeParser_recv
		 */  
		pDot11Hdr = (dot11_header_t*)(memMgr_BufData(pMsdu->firstBDPtr)
									 +memMgr_BufOffset(pMsdu->firstBDPtr));
		os_memoryCopy(pRxData->hOs, &address3, &pDot11Hdr->address3, sizeof(address3));

		tmpFCtrl = ((pDot11Hdr->fc & DOT11_FC_SUB_MASK) >> 4) ;
		if((tmpFCtrl == BEACON) || (tmpFCtrl == PROBE_RESPONSE)) 
		{
			if (NOK == siteMgr_CheckRxSignalValidity(pRxData->hSiteMgr, pRxAttr->Rssi, pRxAttr->channel,&address3))
			{
				wlan_memMngrFreeMSDU(pRxData->hMemMngr, memMgr_MsduHandle(pMsdu));
				break;
			}
		}

		/* distribute mgmt msdu to mlme */
		wlan_memMngrChangeMsduOwner(pRxData->hMemMngr,MLME_RX_MODULE,pMsdu);

		if( mlmeParser_recv(pRxData->hMlme, pMsdu, pRxAttr) != OK )
		{
			WLAN_REPORT_WARNING(pRxData->hReport, RX_DATA_MODULE_LOG, 
				(" rxData_receiveMsduFromWlan() :	error sending msdu to MLME \n"));
			break;
		}


		if((tmpFCtrl == BEACON) || (tmpFCtrl == PROBE_RESPONSE))
		{
			siteMgr_updateRxSignal(pRxData->hSiteMgr, pRxAttr->SNR, 
				pRxAttr->Rssi, pRxAttr->Rate, &address3, FALSE); 
			/*WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG,
					(" SIGNAL QUALITY :RX_LEVEL = %d :: SNR = %d \n", pRxAttr->RxLevel,pRxAttr->SNR));	*/
		}

		break;

	case RX_PACKET_TYPE_DATA:
		WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, 
				(" rxData_receiveMsduFromWlan() : Received Data MSDU len = %d\n", pMsdu->dataLen));

		/* send MSDU to data dispatcher */
		rxData_dataMsduDisptcher(hRxData, pMsdu, pRxAttr);

		break;

	default:
		WLAN_REPORT_ERROR(pRxData->hReport, RX_DATA_MODULE_LOG, 
				(" rxData_receiveMsduFromWlan() : Received unspecified packet type !!! \n"));

		WLAN_REPORT_DEBUG_RX(pRxData->hReport, 
							 (" rxData_receiveMsduFromWlan() : Received unspecified packet type !!! \n"));


		wlan_memMngrFreeMSDU(pRxData->hMemMngr, memMgr_MsduHandle(pMsdu)); 
		break;
	}
}
/***************************************************************************
*						rxData_dataMsduDisptcher		                   *
****************************************************************************
* DESCRIPTION:	this function is called upon receving data MSDU,
*				it dispatches the packet to the approciate function according to 
*               data packet type and rx port status. 
* 
* INPUTS:		hRxData - the object
*				pMsdu - the received msdu.
*				pRxAttr - Rx attributes
*		
* OUTPUT:		
* 
* RETURNS:		
***************************************************************************/

static void rxData_dataMsduDisptcher(TI_HANDLE hRxData, mem_MSDU_T *pMsdu, Rx_attr_t* pRxAttr)
{
	rxData_t *pRxData = (rxData_t *)hRxData;
	portStatus_e DataPortStatus;
	rxDataPacketType_e DataPacketType;


	/* get rx port status */
	DataPortStatus = pRxData->rxDataPortStatus;

	/* discard data packets received while rx data port is closed */
	if (DataPortStatus == CLOSE)
	{
		WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, 
				(" rxData_dataMsduDisptcher() : Received Data msdu while Rx data port is closed \n", pMsdu->dataLen));

		rxData_discardMsdu(hRxData,pMsdu, pRxAttr);
		return;
	}

	/* get data packet type */

#ifdef EXC_MODULE_INCLUDED
	if (excMngr_isIappPacket(pRxData->hExcMgr, pMsdu) == TRUE)
	{
		WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, 
				(" rxData_dataMsduDisptcher() : Received Iapp msdu  \n"));

		DataPacketType = DATA_IAPP_PACKET;

	}
    else
#endif
	{
		UINT16 etherType;
		EthernetHeader_t * pEthernetHeader;

		/* 
		 * if Host processes received packets, the header translation
		 * from WLAN to ETH is done here. The conversion has been moved
		 * here so that IAPP packets aren't converted.
		 */
		rxData_convertWlanToEthHeader(hRxData,pMsdu,&etherType);

		pEthernetHeader = (EthernetHeader_t *)(memMgr_BufData(pMsdu->firstBDPtr) + memMgr_BufOffset(pMsdu->firstBDPtr));

        if (etherType == ETHERTYPE_802_1D)
        {
			WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, 
					(" rxData_dataMsduDisptcher() : Received VLAN msdu  \n"));

			DataPacketType = DATA_VLAN_PACKET;

        }
		else if(pEthernetHeader->TypeLength == EAPOL_PACKET)
		{
			WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, 
					(" rxData_dataMsduDisptcher() : Received Eapol msdu  \n"));

			DataPacketType = DATA_EAPOL_PACKET;

		}
		else
		{
			WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, 
					(" rxData_dataMsduDisptcher() : Received Data msdu  \n"));

			DataPacketType = DATA_DATA_PACKET;
		}
	}

	/* dispatch Msdu according to packet type and current rx data port status */
	pRxData->rxData_dispatchMsdu[DataPortStatus][DataPacketType](hRxData,pMsdu,pRxAttr);

}
/***************************************************************************
*						rxData_discardMsdu		                           *
****************************************************************************
* DESCRIPTION:	this function is called to discard MSDU
* 
* INPUTS:		hRxData - the object
*				pMsdu - the received msdu.
*				pRxAttr - Rx attributes
*		
* OUTPUT:		
* 
* RETURNS:		
***************************************************************************/
static void	rxData_discardMsdu(TI_HANDLE hRxData, mem_MSDU_T *pMsdu, Rx_attr_t* pRxAttr)
{

	rxData_t *pRxData = (rxData_t *)hRxData;

	WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, 
			(" rxData_discardMsdu: rx port status = %d , Msdu status = %d  \n",pRxData->rxDataPortStatus,pRxAttr->status));

	WLAN_REPORT_DEBUG_RX(pRxData->hReport,
			(" rxData_discardMsdu: rx port status = %d , Msdu status = %d  \n",pRxData->rxDataPortStatus,pRxAttr->status));


	pRxData->rxDataDbgCounters.excludedFrameCounter++;


	/* free Msdu */
	wlan_memMngrFreeMSDU(pRxData->hMemMngr, memMgr_MsduHandle(pMsdu)); 


}

/***************************************************************************
*						rxData_discardMsduVlan		                           *
****************************************************************************
* DESCRIPTION:	this function is called to discard MSDU
* 
* INPUTS:		hRxData - the object
*				pMsdu - the received msdu.
*				pRxAttr - Rx attributes
*		
* OUTPUT:		
* 
* RETURNS:		
***************************************************************************/
static void	rxData_discardMsduVlan(TI_HANDLE hRxData, mem_MSDU_T *pMsdu, Rx_attr_t* pRxAttr)
{

	rxData_t *pRxData = (rxData_t *)hRxData;


    WLAN_REPORT_WARNING(pRxData->hReport, RX_DATA_MODULE_LOG, 
			(" rxData_discardMsduVlan : drop packet that contains VLAN tag\n"));

	pRxData->rxDataDbgCounters.rxDroppedDueToVLANIncludedCnt++;


	/* free Msdu */
	wlan_memMngrFreeMSDU(pRxData->hMemMngr, memMgr_MsduHandle(pMsdu)); 
}

/***************************************************************************
*						rxData_rcvMsduInOpenNotify                         *
****************************************************************************
* DESCRIPTION:	this function is called upon receving data Eapol packet type 
*               while rx port status is "open notify"
* 
* INPUTS:		hRxData - the object
*				pMsdu - the received msdu.
*				pRxAttr - Rx attributes
*		
* OUTPUT:		
* 
* RETURNS:		
***************************************************************************/
static void rxData_rcvMsduInOpenNotify(TI_HANDLE hRxData, mem_MSDU_T *pMsdu, Rx_attr_t* pRxAttr)
{
	rxData_t *pRxData = (rxData_t *)hRxData;

	WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, 
			(" rxData_rcvMsduInOpenNotify: receiving data packet while in rx port status is open notify\n"));

	WLAN_REPORT_DEBUG_RX(pRxData->hReport, 
						 (" rxData_rcvMsduInOpenNotify: ERROR !!! receiving data packet while in rx port status is open notify\n"));
	
	pRxData->rxDataDbgCounters.rcvUnicastFrameInOpenNotify++;

	/* free msdu */
	wlan_memMngrFreeMSDU(pRxData->hMemMngr, memMgr_MsduHandle(pMsdu));

}
/***************************************************************************
*						rxData_rcvMsduEapol                               *
****************************************************************************
* DESCRIPTION:	this function is called upon receving data Eapol packet type 
*               while rx port status is "open  eapol"
* 
* INPUTS:		hRxData - the object
*				pMsdu - the received msdu.
*				pRxAttr - Rx attributes
*		
* OUTPUT:		
* 
* RETURNS:		
***************************************************************************/
static void rxData_rcvMsduEapol(TI_HANDLE hRxData, mem_MSDU_T *pMsdu, Rx_attr_t* pRxAttr)
{
	rxData_t *pRxData = (rxData_t *)hRxData;

	WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, 
	(" rxData_rcvMsduEapol() : Received an EAPOL frame tranferred to OS\n"));

	WLAN_REPORT_DEBUG_RX(pRxData->hReport,
						 (" rxData_rcvMsduEapol() : Received an EAPOL frame tranferred to OS\n"));

	EvHandlerSendEvent(pRxData->hEvHandler, IPC_EVENT_EAPOL,
					  (UINT8*)(pMsdu->firstBDPtr->data + pMsdu->firstBDPtr->dataOffset),
					  pMsdu->firstBDPtr->length);

	wlan_memMngrChangeMsduOwner(pRxData->hMemMngr,OS_ABS_RX_MODULE,pMsdu);

	os_receivePacket(pRxData->hOs, pMsdu, (UINT16)pMsdu->dataLen);

}
/***************************************************************************
*						rxData_rcvMsduData                                 *
****************************************************************************
* DESCRIPTION:	this function is called upon receving data "data" packet type 
*               while rx port status is "open"
* 
* INPUTS:		hRxData - the object
*				pMsdu - the received msdu.
*				pRxAttr - Rx attributes
*		
* OUTPUT:		
* 
* RETURNS:		
***************************************************************************/
static void rxData_rcvMsduData(TI_HANDLE hRxData, mem_MSDU_T *pMsdu, Rx_attr_t* pRxAttr)
{
#if defined(CONFIG_TROUT_PWRSINK) || defined(CONFIG_HTC_PWRSINK)
	extern unsigned long num_rx_pkt_new;
#endif
	rxData_t *pRxData = (rxData_t *)hRxData;
	EthernetHeader_t *pEthernetHeader;
	UINT16 EventMask = 0;		
	ctrlData_t *pCtrlData;

	WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, 
	(" rxData_rcvMsduData() : Received DATA frame tranferred to OS\n"));

	WLAN_REPORT_DEBUG_RX(pRxData->hReport,
						 (" rxData_rcvMsduData() : Received DATA frame tranferred to OS\n"));

	/* check encryption status */
	pEthernetHeader = (EthernetHeader_t *)(memMgr_BufData(pMsdu->firstBDPtr) + memMgr_BufOffset(pMsdu->firstBDPtr));
	if (IsMacAddressDirected(&pEthernetHeader->DstAddr))
	{  /* unicast frame */
		if((pRxData->rxDataExcludeUnencrypted) && (!(pRxAttr->packetInfo & RX_PACKET_FLAGS_ENCRYPTION)))
		{
			pRxData->rxDataDbgCounters.excludedFrameCounter++;
			/* free msdu */
			WLAN_REPORT_WARNING(pRxData->hReport, RX_DATA_MODULE_LOG, 
				(" rxData_rcvMsduData() : exclude unicast unencrypted is TRUE & packet encryption is OFF\n"));

			wlan_memMngrFreeMSDU(pRxData->hMemMngr, memMgr_MsduHandle(pMsdu));
			return;
		}

	}
	else
	{  /* broadcast frame */
		if ((pRxData->rxDataExludeBroadcastUnencrypted) && (!(pRxAttr->packetInfo & RX_PACKET_FLAGS_ENCRYPTION)))
		{
			pRxData->rxDataDbgCounters.excludedFrameCounter++;
			/* free msdu */
			WLAN_REPORT_WARNING(pRxData->hReport, RX_DATA_MODULE_LOG, 
				(" rxData_receiveMsduFromWlan() : exclude broadcast unencrypted is TRUE & packet encryption is OFF\n"));

			wlan_memMngrFreeMSDU(pRxData->hMemMngr, memMgr_MsduHandle(pMsdu));
			return;
		}

		/*
		 * Discard multicast/broadcast frames that we sent ourselves.
		 * Per IEEE 802.11-2007 section 9.2.7: "STAs shall filter out
		 * broadcast/multicast messages that contain their address as
		 * the source address."
		 */
		pCtrlData = (ctrlData_t *)pRxData->hCtrlData;
		if (IsMacAddressEqual(&pCtrlData->ctrlDataDeviceMacAddress, &pEthernetHeader->SrcAddr))
		{
			pRxData->rxDataDbgCounters.excludedFrameCounter++;
			/* free msdu */
			wlan_memMngrFreeMSDU(pRxData->hMemMngr, memMgr_MsduHandle(pMsdu));
			return;
		}
	}

	/* update traffic monitor parameters */
	pRxData->rxDataCounters.RecvOk++;
	EventMask |= RECV_OK;
	if ( IsMacAddressDirected(&pEthernetHeader->DstAddr)) 
	{
		/* Directed frame */
		pRxData->rxDataCounters.DirectedFramesRecv++;
		pRxData->rxDataCounters.DirectedBytesRecv += pMsdu->dataLen;
		EventMask |= DIRECTED_BYTES_RECV;  
		EventMask |= DIRECTED_FRAMES_RECV;
#if defined(CONFIG_TROUT_PWRSINK) || defined(CONFIG_HTC_PWRSINK)
		num_rx_pkt_new++;
#endif
	}
	else if ( IsMacAddressBroadcast(&pEthernetHeader->DstAddr)) 
	{
		/* Broadcast frame */
		pRxData->rxDataCounters.BroadcastFramesRecv++;
		pRxData->rxDataCounters.BroadcastBytesRecv += pMsdu->dataLen;
		EventMask |= BROADCAST_BYTES_RECV;  
		EventMask |= BROADCAST_FRAMES_RECV;

	}
	else 
	{
		/* Multicast Address */
		pRxData->rxDataCounters.MulticastFramesRecv++;
		pRxData->rxDataCounters.MulticastBytesRecv += pMsdu->dataLen;
		EventMask |= MULTICAST_BYTES_RECV;  
		EventMask |= MULTICAST_FRAMES_RECV;
#if defined(CONFIG_TROUT_PWRSINK) || defined(CONFIG_HTC_PWRSINK)
		num_rx_pkt_new++;
#endif
	}
	pRxData->rxDataCounters.LastSecBytesRecv += pMsdu->dataLen;

	/*Handle PREAUTH_EAPOL_PACKET*/
	if(pEthernetHeader->TypeLength == PREAUTH_EAPOL_PACKET)
	{
		WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, 
		(" rxData_receiveMsduFromWlan() : Received an Pre-Auth EAPOL frame tranferred to OS\n"));
		
		WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, 
			("rxData_receiveMsduFromWlan(): Received an Pre-Auth EAPOL frame tranferred to OS\n"));
		EvHandlerSendEvent(pRxData->hEvHandler, IPC_EVENT_PREAUTH_EAPOL,
		(UINT8*)(pMsdu->firstBDPtr->data + pMsdu->firstBDPtr->dataOffset),
		pMsdu->firstBDPtr->length);
	}


	rxData_DistributorRxEvent(pRxData,EventMask,(int)pMsdu->dataLen);

	/* deliver packet to os */
	wlan_memMngrChangeMsduOwner(pRxData->hMemMngr,OS_ABS_RX_MODULE,pMsdu);
	os_receivePacket(pRxData->hOs, pMsdu, (UINT16)pMsdu->dataLen);
}


/***************************************************************************
*						rxData_rcvMsduIapp                                 *
****************************************************************************
* DESCRIPTION:	this function is called upon receving data IAPP packet type 
*               while rx port status is "open"
* 
* INPUTS:		hRxData - the object
*				pMsdu - the received msdu.
*				pRxAttr - Rx attributes
*		
* OUTPUT:		
* 
* RETURNS:		
***************************************************************************/
#ifdef EXC_MODULE_INCLUDED

static void rxData_rcvMsduIapp(TI_HANDLE hRxData, mem_MSDU_T *pMsdu, Rx_attr_t* pRxAttr)
{
	rxData_t *pRxData = (rxData_t *)hRxData;

	WLAN_REPORT_INFORMATION(pRxData->hReport, RX_DATA_MODULE_LOG, 
	(" rxData_rcvMsduIapp() : Received IAPP frame tranferred to excMgr\n"));

	WLAN_REPORT_DEBUG_RX(pRxData->hReport,
						 (" rxData_rcvMsduIapp() : Received IAPP frame tranferred to excMgr\n"));

	/* tranfer packet to excMgr */
	wlan_memMngrChangeMsduOwner(pRxData->hMemMngr,EXC_MANAGER_RX_MODULE,pMsdu);
	excMngr_recvIAPPPacket(pRxData->hExcMgr, pMsdu);

	/* free msdu */
	wlan_memMngrFreeMSDU(pRxData->hMemMngr, memMgr_MsduHandle(pMsdu));

}

#endif


/****************************************************************************
*						rxData_convertWlanToEthHeader   	                *
*****************************************************************************
* DESCRIPTION:	this function convert the msdu header from 802.11 header 
*				format to ethernet format
* 
* INPUTS:		hRxData - the object
*				pMsdu - msdu in 802.11 format
*		
* OUTPUT:		pMsdu - msdu in ethernet format
* 
* RETURNS:		OK/NOK
***************************************************************************/
static TI_STATUS rxData_convertWlanToEthHeader (TI_HANDLE hRxData, mem_MSDU_T *pMsdu, UINT16 * etherType)
{
	
	EthernetHeader_t	EthHeader;
	Wlan_LlcHeader_T	*pWlanSnapHeader;   
	UINT8 				*dataBuf;
	dot11_header_t  	*pDot11Header;
	UINT32   			lengthDelta;
	UINT8				createEtherIIHeader;
	UINT16				swapedTypeLength;
	UINT32              headerLength;
	rxData_t *pRxData = (rxData_t *)hRxData;

	*etherType = 0;
	headerLength = pMsdu->headerLen;
	
	dataBuf = (UINT8 *)memMgr_BufData(pMsdu->firstBDPtr)+memMgr_BufOffset(pMsdu->firstBDPtr);
	pDot11Header = (dot11_header_t*) dataBuf;

	if(pMsdu->firstBDPtr->length == headerLength)
	{
		if (memMgr_BufNext(pMsdu->firstBDPtr))
		{
			pWlanSnapHeader = (Wlan_LlcHeader_T*)(memMgr_BufData(pMsdu->firstBDPtr->nextBDPtr)
												  +memMgr_BufOffset(pMsdu->firstBDPtr->nextBDPtr));
		} 
		else 
		{
			return OK;
		}
	}
    else
	pWlanSnapHeader = (Wlan_LlcHeader_T*)((UINT32)dataBuf + (UINT32)headerLength);


	swapedTypeLength = wlan_ntohs(pWlanSnapHeader->Type);
    *etherType = swapedTypeLength;

	/* See if the LLC header in the frame shows the SAP SNAP... */
	if((SNAP_CHANNEL_ID == pWlanSnapHeader->DSAP) &&
	   (SNAP_CHANNEL_ID == pWlanSnapHeader->SSAP) &&
	   (LLC_CONTROL_UNNUMBERED_INFORMATION == pWlanSnapHeader->Control)) 
	{
		/* Check for the Bridge Tunnel OUI in the SNAP Header... */
		if((SNAP_OUI_802_1H_BYTE0 == pWlanSnapHeader->OUI[ 0 ]) &&
		   (SNAP_OUI_802_1H_BYTE1 == pWlanSnapHeader->OUI[ 1 ]) &&
		   (SNAP_OUI_802_1H_BYTE2 == pWlanSnapHeader->OUI[ 2 ])) 
		{
			/* Strip the SNAP header by skipping over it.                  */
			/* Start moving data from the Ethertype field in the SNAP      */
			/* header.  Move to the TypeLength field in the 802.3 header.  */
			createEtherIIHeader = TRUE;        
		}
		/* Check for the RFC 1042 OUI in the SNAP Header   */
		else 
		{
			/* See if the Ethertype is in our selective translation table  */
			/* (Appletalk AARP and DIX II IPX are the two protocols in     */
			/* our 'table')                                                */
			if((ETHERTYPE_APPLE_AARP == swapedTypeLength) ||
			   (ETHERTYPE_DIX_II_IPX == swapedTypeLength)) 
			{
				/* Strip the SNAP header by skipping over it. */
				createEtherIIHeader = FALSE;
			}
			/* All the rest SNAP types are transformed to EthernetII format */
			else
			{
				createEtherIIHeader = TRUE;
			}
		}
	}
	else 
	{ 
		/* Non-SNAP packets strip out the WLAN header and create the dst,src,len header. */
		createEtherIIHeader = FALSE;
	}
	
	

	/* Prepare the Ethernet header. */
	if(pDot11Header->fc & DOT11_FC_FROM_DS)
	{	/* Infrastructure  bss */
		MAC_COPY(pRxData->hOs, (&EthHeader.DstAddr), (&pDot11Header->address1));
		MAC_COPY(pRxData->hOs, (&EthHeader.SrcAddr), (&pDot11Header->address3));
	}
	else
	{	/* Independent bss */
		MAC_COPY(pRxData->hOs, (&EthHeader.DstAddr), (&pDot11Header->address1));
		MAC_COPY(pRxData->hOs, (&EthHeader.SrcAddr), (&pDot11Header->address2));
	}
	
	if( createEtherIIHeader == TRUE )
	{
		EthHeader.TypeLength = pWlanSnapHeader->Type;

        if(pMsdu->firstBDPtr->length == headerLength)
        {
            /* Replace the 802.11 header and the LLC with Ethernet packet. */
            lengthDelta = (pMsdu->firstBDPtr->nextBDPtr->data 
						 + pMsdu->firstBDPtr->nextBDPtr->dataOffset 
						 - pMsdu->firstBDPtr->data)
							+ WLAN_SNAP_HDR_LEN - ETHERNET_HDR_LEN;
            dataBuf += lengthDelta;
            os_memoryCopy(pRxData->hOs, dataBuf, (void*)&EthHeader, ETHERNET_HDR_LEN );
            pMsdu->firstBDPtr->dataOffset = lengthDelta;
            pMsdu->dataLen -= headerLength + WLAN_SNAP_HDR_LEN - ETHERNET_HDR_LEN;
			pMsdu->firstBDPtr->length = pMsdu->dataLen;

			/* UDI - modify code to release second BD to test NDIS */
			wlan_memMngrFreeBD(pRxData->hMemMngr, pMsdu->firstBDPtr->nextBDPtr->handle);
			pMsdu->firstBDPtr->nextBDPtr = NULL;
            return OK;

        }
		/* The LEN/TYPE bytes are set to TYPE, the entire WLAN+SNAP is removed.*/
		lengthDelta = headerLength + WLAN_SNAP_HDR_LEN - ETHERNET_HDR_LEN;
		EthHeader.TypeLength = pWlanSnapHeader->Type;
	}
	else 
	{
		/* The LEN/TYPE bytes are set to frame LEN, only the WLAN header is removed, */
		/* the entire 802.3 or 802.2 header is not removed.*/
		EthHeader.TypeLength = wlan_ntohs((UINT16)(pMsdu->dataLen - headerLength));
		lengthDelta = headerLength - ETHERNET_HDR_LEN;
	}
	
	/* Replace the 802.11 header and the LLC with Ethernet packet. */
	dataBuf += lengthDelta;
	os_memoryCopy(pRxData->hOs, dataBuf, (void*)&EthHeader, ETHERNET_HDR_LEN );
	memMgr_BufOffset(pMsdu->firstBDPtr) += lengthDelta;
	pMsdu->dataLen -= lengthDelta;
	pMsdu->firstBDPtr->length -= lengthDelta;
	pMsdu->headerLen = ETHERNET_HDR_LEN;
	return OK;
				
}


/****************************************************************************************
 *                        rxData_ReceivePacket                                              *
 ****************************************************************************************
DESCRIPTION:    receive packet CB from RxXfer.
                parse the status and other parameters and forward the frame to  
                rxData_receiveMsduFromWlan()
                
INPUT:          Rx frame with its parameters    

OUTPUT:

RETURN:         

************************************************************************/
static void rxData_ReceivePacket ( TI_HANDLE   hRxData,
                                   TI_STATUS   aStatus,
                                   const void *aFrame,
                                   UINT16      aLength,
                                   UINT32      aRate,
                                   UINT8       aRCPI,
                                   UINT8       aChannel,
                                   void       *Reserved,
                                   UINT32      aFlags)
{
    rxData_t *pRxData = (rxData_t *)hRxData;
    mem_MSDU_T *pMsdu = pRxData->pReqForBufMsdu;

    if (pMsdu)
    {
        rxXfer_Reserved_t *pWhalReserved = (rxXfer_Reserved_t *)Reserved;
        Rx_attr_t RxAttr; 
        dot11_header_t *pHdr;
        
        /*
         * First thing we do is getting the dot11_header, and than we check the status, since the header is
         * needed for RX_MIC_FAILURE_ERROR 
         */

        /* The next field includes the data only, excluding the TNETWIF_READ_OFFSET_BYTES */
        pMsdu->dataLen = aLength;
        /* NOTE !!! the length field in the rx path exclude the TNETWIF_READ_OFFSET_BYTES !!! (while in the tx path its included) */ 
        pMsdu->firstBDPtr->length = aLength;
		/* Actual data starts after the TNETWIF_READ_OFFSET_BYTES reserved for the bus txn */
		pMsdu->firstBDPtr->dataOffset = TNETWIF_READ_OFFSET_BYTES ; 
		
        pHdr = (dot11_header_t *)(memMgr_BufData(pMsdu->firstBDPtr) + memMgr_BufOffset(pMsdu->firstBDPtr));

        /* Check status */
        switch (aStatus)
        {
            case OK:
                break;

            case RX_MIC_FAILURE_ERROR:
            {
                UINT8 uKeyType; 
                paramInfo_t Param;
                macAddress_t* pMac = &pHdr->address1; /* hold the first mac address */
                /* Get BSS type */
                Param.paramType = SITE_MGR_CURRENT_BSS_TYPE_PARAM;
                siteMgr_getParam (pRxData->hSiteMgr, &Param);

                /* For multicast/broadcast frames or in IBSS the key used is GROUP, else - it's Pairwise */
                if (MAC_MULTICAST(pMac) || Param.content.siteMgrCurrentBSSType == BSS_INDEPENDENT)
                {
                    uKeyType = (UINT8)KEY_TKIP_MIC_GROUP;
                }
                /* Unicast on infrastructure */
                else 
                {
                    uKeyType = (UINT8)KEY_TKIP_MIC_PAIRWISE;
                }

                WLAN_REPORT_ERROR(pRxData->hReport, RX_DATA_MODULE_LOG, 
                    ("rxData_ReceivePacket: Received MSDU MIC failure. Type = %s\n",
                    ((uKeyType == KEY_TKIP_MIC_GROUP) ? "GROUP" : "PAIRWISE")));

                rsn_reportMicFailure (pRxData->hRsn, &uKeyType, sizeof(uKeyType));
                wlan_memMngrFreeMSDU (pRxData->hMemMngr, memMgr_MsduHandle(pMsdu)); 
                return;
            }
            
            case RX_DECRYPT_FAILURE:
                /* This error is not important before the Connection, so we ignore it when portStatus is not OPEN */
                if (pRxData->rxDataPortStatus == OPEN) 
                {
                    WLAN_REPORT_WARNING(pRxData->hReport, RX_DATA_MODULE_LOG, 
                        ("rxData_ReceivePacket: Received MSDU with RX_DECRYPT_FAILURE\n"));
                }
                
                wlan_memMngrFreeMSDU (pRxData->hMemMngr, memMgr_MsduHandle(pMsdu)); 
                return;

            default:    
                /* Unknown error - free packet and return */
                WLAN_REPORT_ERROR(pRxData->hReport, RX_DATA_MODULE_LOG, 
                    ("rxData_ReceivePacket: Received MSDU with unknown status = %s\n",
                    convertTI_STATUS_toString (aStatus)));

                wlan_memMngrFreeMSDU (pRxData->hMemMngr, memMgr_MsduHandle(pMsdu)); 
                return;
        }

        WLAN_REPORT_INFORMATION (pRxData->hReport, RX_DATA_MODULE_LOG, ("ReceivePacket\n"));
                                 
        /*
         * Set rx attributes 
         */ 
        RxAttr.channel    = aChannel;
        RxAttr.packetInfo = aFlags;
        RxAttr.packetType = pWhalReserved->packetType;
        /* Rate is converted in RxXfer module */
        RxAttr.Rate       = (rate_e)aRate;
        RxAttr.Rssi       = pWhalReserved->rssi;
        RxAttr.SNR        = pWhalReserved->SNR;
        RxAttr.status     = aStatus;
        RxAttr.band       = pWhalReserved->band;
        RxAttr.TimeStamp  = pWhalReserved->TimeStamp;
        
        /* Setting the mac header len according to the received FrameControl field in the Mac Header */
        GET_MAX_HEADER_SIZE (pHdr, &pMsdu->headerLen);
        
        WLAN_REPORT_DEBUG_RX(pRxData->hReport, 
                             ("rxData_ReceivePacket: channel=%d, info=0x%x, type=%d, rate=0x%x, RSSI=%d, SNR=%d, status=%d\n",
                             RxAttr.channel,  
                             RxAttr.packetInfo,
                             RxAttr.packetType,
                             RxAttr.Rate,
                             RxAttr.Rssi,
                             RxAttr.SNR,
                             RxAttr.status));

        rxData_receiveMsduFromWlan (hRxData, pMsdu, &RxAttr);

        /* MSDU MUST be freed until now */
        if (pMsdu != NULL)
        {
            if (pMsdu->module != MODULE_FREE_MSDU &&
                pMsdu->module != OS_ABS_TX_MODULE && 
                pMsdu->module != TX_MODULE)
            {
                WLAN_REPORT_ERROR(pRxData->hReport, RX_DATA_MODULE_LOG,  
                   ("rxData_ReceivePacket: ERROR pMsdu->module != MODULE_FREE_MSDU, module = %d\n", pMsdu->module));

                HexDumpData ((UINT8*)(memMgr_MsduHdrAddr(pMsdu)), memMgr_MsduHdrLen(pMsdu));
            }
        }
    }

    else
    {
        WLAN_REPORT_ERROR (pRxData->hReport, RX_DATA_MODULE_LOG,  
            ("rxData_ReceivePacket: null MSDU received"));
    }
}


/****************************************************************************************
 *                        RequestForBuffer                                              *
 ****************************************************************************************
DESCRIPTION:     RX request for buffer

INPUT:          

OUTPUT:

RETURN:         

************************************************************************/
static void *rxData_RequestForBuffer (TI_HANDLE   hRxData,
                                      UINT16 aLength, UINT32 uEncryptionflag)
{
    rxData_t *pRxData = (rxData_t *)hRxData;

	WLAN_REPORT_INFORMATION (pRxData->hReport, RX_DATA_MODULE_LOG,
								(" RequestForBuffer, length = %d \n",aLength));

	if (wlan_memMngrAllocMSDU(pRxData->hMemMngr, &pRxData->pReqForBufMsdu, aLength, HAL_RX_MODULE) != OK)
	{
		WLAN_REPORT_ERROR(pRxData->hReport, HAL_CTRL_MODULE_LOG,  
			("RequestForBuffer: wlan_memMngrAllocMSDU error\n"));
		return NULL;
	}

	return memMgr_BufData (pRxData->pReqForBufMsdu->firstBDPtr);
}

/***************************************************************************
*						 rxData_resetCounters				               *
****************************************************************************
* DESCRIPTION:	This function reset the Rx Data module counters
*
* INPUTS:		hRxData - the object
*
* OUTPUT:		
* 
* RETURNS:		void
***************************************************************************/
#ifdef TI_DBG
void rxData_resetCounters(TI_HANDLE hRxData)
{
	rxData_t *pRxData = (rxData_t *)hRxData;

	os_memoryZero(pRxData->hOs,	&pRxData->rxDataCounters, sizeof(rxDataCounters_t));
}

/***************************************************************************
*						 rxData_resetDbgCounters			               *
****************************************************************************
* DESCRIPTION:	This function reset the Rx Data module debug counters
*
* INPUTS:		hRxData - the object
*
* OUTPUT:		
* 
* RETURNS:		void
***************************************************************************/

void rxData_resetDbgCounters(TI_HANDLE hRxData)
{
	rxData_t *pRxData = (rxData_t *)hRxData;

	os_memoryZero(pRxData->hOs,	&pRxData->rxDataDbgCounters, sizeof(rxDataDbgCounters_t));
}


/***************************************************************************
*							 test functions					               *
***************************************************************************/
void rxData_printRxCounters(TI_HANDLE hRxData)
{
	rxData_t *pRxData = (rxData_t *)hRxData;

    if(pRxData) 
    {
        WLAN_OS_REPORT(("RecvOk = %d\n", pRxData->rxDataCounters.RecvOk));
    	WLAN_OS_REPORT(("DirectedBytesRecv = %d\n", pRxData->rxDataCounters.DirectedBytesRecv));
    	WLAN_OS_REPORT(("DirectedFramesRecv = %d\n", pRxData->rxDataCounters.DirectedFramesRecv));
    	WLAN_OS_REPORT(("MulticastBytesRecv = %d\n", pRxData->rxDataCounters.MulticastBytesRecv));
    	WLAN_OS_REPORT(("MulticastFramesRecv = %d\n", pRxData->rxDataCounters.MulticastFramesRecv));
    	WLAN_OS_REPORT(("BroadcastBytesRecv = %d\n", pRxData->rxDataCounters.BroadcastBytesRecv));
    	WLAN_OS_REPORT(("BroadcastFramesRecv = %d\n", pRxData->rxDataCounters.BroadcastFramesRecv));
    
    	/* debug counters */
        WLAN_OS_REPORT(("excludedFrameCounter = %d\n", pRxData->rxDataDbgCounters.excludedFrameCounter));
	    WLAN_OS_REPORT(("rxDroppedDueToVLANIncludedCnt = %d\n", pRxData->rxDataDbgCounters.rxDroppedDueToVLANIncludedCnt));    
        WLAN_OS_REPORT(("rxWrongBssTypeCounter = %d\n", pRxData->rxDataDbgCounters.rxWrongBssTypeCounter));
	    WLAN_OS_REPORT(("rxWrongBssIdCounter = %d\n", pRxData->rxDataDbgCounters.rxWrongBssIdCounter));
        WLAN_OS_REPORT(("rcvUnicastFrameInOpenNotify = %d\n", pRxData->rxDataDbgCounters.rcvUnicastFrameInOpenNotify));        
    }
}


void rxData_printRxBlock(TI_HANDLE hRxData)
{
	rxData_t *pRxData = (rxData_t *)hRxData;

	WLAN_OS_REPORT(("hCtrlData = 0x%X\n", pRxData->hCtrlData));
	WLAN_OS_REPORT(("hMlme = 0x%X\n", pRxData->hMlme));
	WLAN_OS_REPORT(("hOs = 0x%X\n", pRxData->hOs));
	WLAN_OS_REPORT(("hReport = 0x%X\n", pRxData->hReport));
	WLAN_OS_REPORT(("hRsn = 0x%X\n", pRxData->hRsn));
	WLAN_OS_REPORT(("hSiteMgr = 0x%X\n", pRxData->hSiteMgr));
	WLAN_OS_REPORT(("hMemMngr = 0x%X\n", pRxData->hMemMngr));

	WLAN_OS_REPORT(("hCtrlData = 0x%X\n", pRxData->hCtrlData));
	WLAN_OS_REPORT(("hMlme = 0x%X\n", pRxData->hMlme));
	WLAN_OS_REPORT(("hOs = 0x%X\n", pRxData->hOs));
	WLAN_OS_REPORT(("hReport = 0x%X\n", pRxData->hReport));
	WLAN_OS_REPORT(("hRsn = 0x%X\n", pRxData->hRsn));
	WLAN_OS_REPORT(("hSiteMgr = 0x%X\n", pRxData->hSiteMgr));
	WLAN_OS_REPORT(("hMemMngr = 0x%X\n", pRxData->hMemMngr));

	WLAN_OS_REPORT(("rxDataPortStatus = %d\n", pRxData->rxDataPortStatus));
	WLAN_OS_REPORT(("rxDataExcludeUnencrypted = %d\n", pRxData->rxDataExcludeUnencrypted));
	WLAN_OS_REPORT(("rxDataEapolDestination = %d\n", pRxData->rxDataEapolDestination));

}


void rxData_startRxThroughputTimer (TI_HANDLE hRxData)
{
    rxData_t *pRxData = (rxData_t *)hRxData;

    if (!pRxData->rxThroughputTimerEnable)
    {
        /* reset throughput counter */
        pRxData->rxDataCounters.LastSecBytesRecv = 0;
        pRxData->rxThroughputTimerEnable = TRUE;

        /* start 1 sec throughput timer */
        os_timerStart (pRxData->hOs, pRxData->hThroughputTimer, 1000, TRUE);
    }
}


void rxData_stopRxThroughputTimer (TI_HANDLE hRxData)
{

    rxData_t *pRxData = (rxData_t *)hRxData;

    if (pRxData->rxThroughputTimerEnable)
    {
        os_timerStop (pRxData->hOs, pRxData->hThroughputTimer);
        pRxData->rxThroughputTimerEnable = FALSE;
    } 
}


static void rxData_printRxThroughput (TI_HANDLE hRxData)
{
    rxData_t *pRxData = (rxData_t *)hRxData;

    WLAN_OS_REPORT (("\n"));
    WLAN_OS_REPORT (("-------------- Rx Throughput Statistics ---------------\n"));
    WLAN_OS_REPORT (("Throughput = %d KBits/sec\n", pRxData->rxDataCounters.LastSecBytesRecv * 8 / 1024));

    /* reset throughput counter */
    pRxData->rxDataCounters.LastSecBytesRecv = 0;
}

void rxData_printRxDataFilter (TI_HANDLE hRxData)
{
	UINT32 index;
    rxData_t *pRxData = (rxData_t *)hRxData;

	for (index=0; index<MAX_DATA_FILTERS; index++)
	 {
	 	if (pRxData->isFilterSet[index])
	 	{
			WLAN_OS_REPORT (("index=%d, pattern & mask\n", index));
			HexDumpData(pRxData->filterRequests[index].pattern, pRxData->filterRequests[index].patternLength);
			HexDumpData(pRxData->filterRequests[index].mask, pRxData->filterRequests[index].maskLength);
	 	}
	 	else
	 	{
	 		WLAN_OS_REPORT (("No Filter defined for index-%d\n", index));
	 	}
	 }
}

#endif /*TI_DBG*/
