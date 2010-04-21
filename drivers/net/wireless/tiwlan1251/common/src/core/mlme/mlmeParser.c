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
/*                                                                         */
/*      MODULE: mlmeBuilder.c                                              */
/*    PURPOSE:  802.11 MLME Builder                                        */
/*                                                                         */
/***************************************************************************/



#include "802_11Defs.h"

#include "osApi.h"

#include "paramOut.h"
#include "paramIn.h"

#include "utils.h"
#include "memMngrEx.h"
#include "report.h"

#include "DataCtrl_Api.h"
#include "smeApi.h"

#include "mlmeApi.h"
#include "mlmeSm.h"
#include "Assoc/AssocSM.h"
#include "Auth/authSm.h"

#include "mlmeParser.h"
#include "measurementMgrApi.h"
#include "siteMgrApi.h"
#include "spectrumMngmntMgr.h"
#include "currBss.h"
#include "apConn.h"
#include "SwitchChannelApi.h"
#include "regulatoryDomainApi.h"
#include "qosMngr_API.h"


/* Constants */

/* Enumerations */

/* Typedefs */

/* Structures */

/* External data definitions */

/* External functions definitions */

/* Local function prototypes */

/* Functions */

#define BcnMissTst 0
#define BcnMissTstWithScrPad7 0
#define CHECK_PARSING_ERROR_CONDITION_PRINT 0

#if BcnMissTst
static void mlmeParser_printBeaconDebugInfo(TI_HANDLE theMlmeHandle,
                                            mlmeFrameInfo_t theFrame);
#endif

BOOL MissingBcnInt = FALSE ;
extern int WMEQosTagToACTable[MAX_NUM_OF_802_1d_TAGS];

TI_STATUS mlmeParser_recv(TI_HANDLE hMlme, mem_MSDU_T *pMsdu, Rx_attr_t* pRxAttr)
{
    TI_STATUS               status = NOK;
    mlme_t                  *pHandle;
    UINT8                   *pData;
    INT32                   bodyDataLen;
    UINT32                  readLen;
    dot11_eleHdr_t          *pEleHdr;
    dot11_mgmtFrame_t       *pMgmtFrame;
    dot11MgmtSubType_e      msgType;
    paramInfo_t             param;
    macAddress_t            recvBssid;
	macAddress_t            recvSa;
    UINT8                   rsnIeIdx = 0;
    UINT8                   wpaIeOuiIe[] = WPA_IE_OUI;
    UINT8                   ti_oui[] = TI_OUI;
#ifdef EXC_MODULE_INCLUDED
    UINT8                   exc_oui[] = EXC_OUI;
    EXCv4IEs_t              *pExcIeParameter;
#endif
    BOOL                    ciscoIEPresent = FALSE;
#if BcnMissTst
    /* debug info */
    UINT32                  currProbeRspTSFTime = 0;
    UINT32                  deltaProbeRspTSFTime = 0;
#endif
    

    if (hMlme == NULL)
    {       
        return NOK;
    }

    pHandle = (mlme_t*)hMlme;

    if (pMsdu == NULL)
    {       
        return NOK;
    }

    /* zero frame content */
    os_memoryZero(pHandle->hOs, &(pHandle->tempFrameInfo), sizeof(mlmeIEParsingParams_t));

    pMgmtFrame = (dot11_mgmtFrame_t*)(memMgr_BufData(pMsdu->firstBDPtr) + memMgr_BufOffset(pMsdu->firstBDPtr)) ;

    /* get frame type */
    status = mlmeParser_getFrameType(pHandle, (UINT16 *)&pMgmtFrame->hdr.fc, &msgType);
    if (status != OK)
    {
        wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);
        return OK;
    }

    pHandle->tempFrameInfo.frame.subType = msgType;

    /* We have to ignore management frames from other BSSIDs (except beacons & probe responses) */
    param.paramType = CTRL_DATA_CURRENT_BSSID_PARAM;
    ctrlData_getParam(pHandle->hCtrlData, &param);

    os_memoryCopy(pHandle->hOs, &(recvBssid.addr), &(pMgmtFrame->hdr.BSSID), MAC_ADDR_LEN);

	os_memoryCopy(pHandle->hOs, &(recvSa.addr), &(pMgmtFrame->hdr.SA), MAC_ADDR_LEN);

	if (MAC_EQUAL((&(param.content.ctrlDataCurrentBSSID)), (&(recvBssid))))
        pHandle->tempFrameInfo.myBssid = TRUE;
    else
        pHandle->tempFrameInfo.myBssid = FALSE;

	if (MAC_EQUAL((&(param.content.ctrlDataCurrentBSSID)), (&(recvSa))))
		pHandle->tempFrameInfo.mySa = TRUE;
	else
		pHandle->tempFrameInfo.mySa = FALSE;

	/* The Default value of the myDst flag is false, only in case of unicast packet with the STA's destination address, the flag is set to True */
	pHandle->tempFrameInfo.myDst = FALSE;

    /* check destination MAC address for broadcast */

    if (MAC_BROADCAST((&pMgmtFrame->hdr.DA)))
    {
        pHandle->tempFrameInfo.frame.extesion.destType = MSG_BROADCAST;
    }
        else
        {
        if (MAC_MULTICAST((&pMgmtFrame->hdr.DA)))
        {
            pHandle->tempFrameInfo.frame.extesion.destType = MSG_MULTICAST;
        }
        else
        {
            pHandle->tempFrameInfo.frame.extesion.destType = MSG_UNICAST;

			param.paramType = CTRL_DATA_MAC_ADDRESS;
		    ctrlData_getParam(pHandle->hCtrlData, &param);

			/* Verifying whether the received unicast packet is for the STA, if yes we set the flag to True */
			if (MAC_EQUAL( (&(param.content.ctrlDataDeviceMacAddress)), (&(pMgmtFrame->hdr.DA)) )  )
				pHandle->tempFrameInfo.myDst = TRUE;
			
		
        }
    }

    MAC_COPY(pHandle->hOs, (&(pHandle->tempFrameInfo.bssid)), (&pMgmtFrame->hdr.BSSID));

    pData = (UINT8 *)(pMgmtFrame->body);

    /* length of body (msdu without 802.11 header and FCS) */
    bodyDataLen = pMsdu->dataLen - WLAN_HDR_LEN;

    switch (msgType)
    {
    case ASSOC_REQUEST:
        WLAN_REPORT_SM(pHandle->hReport, MLME_SM_MODULE_LOG,
                       ("MLME_PARSER: recieved ASSOC_REQ message \n"));
        break;
    case RE_ASSOC_REQUEST:
        WLAN_REPORT_SM(pHandle->hReport, MLME_SM_MODULE_LOG,
                       ("MLME_PARSER: recieved RE_ASSOC_REQ message \n"));
        break;
    case RE_ASSOC_RESPONSE:
        WLAN_REPORT_SM(pHandle->hReport, MLME_SM_MODULE_LOG,
                       ("MLME_PARSER: recieved RE_ASSOC_RSP message \n"));
      /*  break;*/
    case ASSOC_RESPONSE:
		/* if the assoc response is not directed to our STA or not from the current AP */
        if ((!pHandle->tempFrameInfo.myBssid) || (!pHandle->tempFrameInfo.mySa) || (pHandle->tempFrameInfo.myDst == FALSE))
            break;

		/* Save the association response message */
        assoc_saveAssocRespMessage(pHandle->hAssoc, (UINT8 *)(pMgmtFrame->body), bodyDataLen);

        /* init frame fields */
        pHandle->tempFrameInfo.frame.content.assocRsp.barkerPreambleMode = PREAMBLE_UNSPECIFIED;

        /* read capabilities */
        pHandle->tempFrameInfo.frame.content.assocRsp.capabilities = ENDIAN_HANDLE_WORD(*(UINT16*)pData);
        pData += 2;
        /* read status */
        pHandle->tempFrameInfo.frame.content.assocRsp.status = ENDIAN_HANDLE_WORD(*(UINT16*)pData);
        pData += 2;
        /* read AID */
        pHandle->tempFrameInfo.frame.content.assocRsp.aid = ENDIAN_HANDLE_WORD(*(UINT16*)pData);
        pData += 2;

        bodyDataLen -= ASSOC_RESP_FIXED_DATA_LEN;
        /***************************/

        pHandle->tempFrameInfo.frame.content.assocRsp.pRsnIe = NULL;
        pHandle->tempFrameInfo.frame.content.assocRsp.rsnIeLen = 0;
        while (bodyDataLen > 2)
        {
            pEleHdr = (dot11_eleHdr_t*)pData;

            if (pEleHdr->eleLen > (bodyDataLen - 2))
            {
                WLAN_REPORT_ERROR(pHandle->hReport, MLME_SM_MODULE_LOG,
                                  ("MLME_PARSER: IE %d with length %d out of bounds %d\n", pEleHdr->eleId, pEleHdr->eleLen, (bodyDataLen - 2)));
                wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);
                return NOK;
            }

            switch (pEleHdr->eleId)
            {
                        /* read rates */
            case SUPPORTED_RATES_IE_ID:
                pHandle->tempFrameInfo.frame.content.assocRsp.pRates = &(pHandle->tempFrameInfo.rates);
                status = mlmeParser_readRates(pHandle, pData, bodyDataLen, &readLen, &(pHandle->tempFrameInfo.rates));
                if (status != OK)
                {
                    WLAN_REPORT_ERROR(pHandle->hReport, MLME_SM_MODULE_LOG,
                                      ("MLME_PARSER: error reading RATES\n"));
                    wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);
                    return NOK;
                }
                break;

            case EXT_SUPPORTED_RATES_IE_ID:
                /* read rates */
                pHandle->tempFrameInfo.frame.content.assocRsp.pExtRates = &(pHandle->tempFrameInfo.extRates);
                status = mlmeParser_readRates(pHandle, pData, bodyDataLen, &readLen, &(pHandle->tempFrameInfo.extRates));
                if (status != OK)
                {
                    wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);
                    WLAN_REPORT_ERROR(pHandle->hReport, MLME_SM_MODULE_LOG,("MLME_PARSER: error reading RATES\n"));
                    return NOK;
                }
                break;

            case ERP_IE_ID:
                status = mlmeParser_readERP(pHandle, pData, bodyDataLen, &readLen,
                                            (BOOL *)&(pHandle->tempFrameInfo.frame.content.assocRsp.useProtection),
                                            (preamble_e *)&(pHandle->tempFrameInfo.frame.content.assocRsp.barkerPreambleMode));
                if (status != OK)
                {
                    wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);                  
                    WLAN_REPORT_ERROR(pHandle->hReport, MLME_SM_MODULE_LOG,("MLME_PARSER: error reading ERP\n"));
                    return NOK;
                }
                break;

            case DOT11_4X_ELE_ID:
                /* Note : WPA, WME, TSRS, MSDU lifetime and 4X use the same Element ID */
                /*  Its assumes that:
                        4X - uses OUI = 0x08,0x00,0x28;
                        TSRS and MSDU lifetime use OUI = 0x00,0x40,0x96 (=Cisco) but
                        use different OUI Type:
                            TSRS          uses OUI Type 8
                            MSDU lifetime uses OUI Type 9;
                        WPA and WME use the same OUI = 0x00,0x50,0xf2 but
                        use different OUI Type:
                            WPA - uses OUI Type with value  - 1
                            WME - uses OUI Type with value  - 2.
                */

                /* check if this is 4X IE */
                if(os_memoryCompare(pHandle->hOs ,ti_oui, pData+2, DOT11_OUI_LEN) == 0)
                {
                    pHandle->tempFrameInfo.frame.content.assocRsp.fourXParams = &(pHandle->tempFrameInfo.fourXParams);
                    status = mlmeParser_read4Xxarams(pHandle, pData, bodyDataLen, &readLen, 
                                                     &(pHandle->tempFrameInfo.fourXParams));
                    if (status != OK)
                    {
                        WLAN_REPORT_ERROR(pHandle->hReport, MLME_SM_MODULE_LOG,
                                          ("MLME_PARSER: error reading 4X parameters\n"));
                        wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);
                        return NOK;
                    }
                }
                /* check if this is WME IE */
                else if((os_memoryCompare(pHandle->hOs, wpaIeOuiIe, pData+2, DOT11_OUI_LEN) == 0) && 
                        ((*(UINT8*)(pData+5)) == dot11_WME_OUI_TYPE))
                {
                    pHandle->tempFrameInfo.frame.content.assocRsp.WMEParams = &(pHandle->tempFrameInfo.WMEParams);
                    status = mlmeParser_readWMEParams(pHandle, pData, bodyDataLen, &readLen, 
                                                      &(pHandle->tempFrameInfo.WMEParams),
                                                      &(pHandle->tempFrameInfo.frame.content.assocRsp));
                    if (status != OK)
                    {
                        WLAN_REPORT_ERROR(pHandle->hReport, MLME_SM_MODULE_LOG,
                                          ("MLME_PARSER: error reading WME parameters\n"));
                        wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);
                        return NOK;
                    }
                }
#ifdef EXC_MODULE_INCLUDED
                /* check if this is EXC vendor specific OUI */
                else if (os_memoryCompare(pHandle->hOs, exc_oui, pData+2, DOT11_OUI_LEN) == 0) 
                {
                    pExcIeParameter = &(pHandle->tempFrameInfo.frame.content.assocRsp.excIEs[WMEQosTagToACTable[*(pData+6)]]);
                    mlmeParser_readExcOui(pData, bodyDataLen, &readLen, pExcIeParameter);
                }
#endif
                else
                {
                    /* skip this IE */
                    readLen = pEleHdr->eleLen + 2;
                }
                break;
            case EXC_EXT_1_IE_ID:
                ciscoIEPresent = TRUE;
                pHandle->tempFrameInfo.frame.content.assocRsp.pRsnIe = &(pHandle->tempFrameInfo.rsnIe[0]);
                status = mlmeParser_readRsnIe(pHandle, pData, bodyDataLen, &readLen, 
                                              &(pHandle->tempFrameInfo.rsnIe[rsnIeIdx]));
                if (status != OK)
                {
                    WLAN_REPORT_ERROR(pHandle->hReport, MLME_SM_MODULE_LOG,
                                      ("MLME_PARSER: error reading EXC EXT1 IE\n"));
                    wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);
                    return NOK;
                }
    
                pHandle->tempFrameInfo.frame.content.assocRsp.rsnIeLen += readLen;
                rsnIeIdx ++;
                break;

            case EXC_EXT_2_IE_ID:
                ciscoIEPresent = TRUE;
                pHandle->tempFrameInfo.frame.content.assocRsp.pRsnIe   = &(pHandle->tempFrameInfo.rsnIe[0]);
                status = mlmeParser_readRsnIe(pHandle, pData, bodyDataLen, &readLen,
                                              &(pHandle->tempFrameInfo.rsnIe[rsnIeIdx]));
                if (status != OK)
                {
                    WLAN_REPORT_ERROR(pHandle->hReport, MLME_SM_MODULE_LOG,
                                      ("MLME_PARSER: error reading RSN IP ADDR IE\n"));
                    wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);
                    return NOK;
                }
    
                pHandle->tempFrameInfo.frame.content.assocRsp.rsnIeLen += readLen;
                rsnIeIdx ++;
                break;

            case DOT11_QOS_CAPABILITY_ELE_ID:
                pHandle->tempFrameInfo.frame.content.assocRsp.QoSCapParameters = &(pHandle->tempFrameInfo.QosCapParams);
                status = mlmeParser_readQosCapabilityIE(pHandle, pData, bodyDataLen, &readLen, 
                                                        &(pHandle->tempFrameInfo.QosCapParams));
                if (status != OK)
                {
                    wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);
                    WLAN_REPORT_ERROR(pHandle->hReport, MLME_SM_MODULE_LOG,("MLME_PARSER: error reading QOS\n"));                   
                    return NOK;
                }
                break;

            default:
                WLAN_REPORT_INFORMATION(pHandle->hReport, MLME_SM_MODULE_LOG,
                                  ("MLME_PARSER: unsupported IE found (%d)\n", pEleHdr->eleId));
                readLen = pEleHdr->eleLen + 2;
                status = OK;
                break;
            }

            pData += readLen;
            bodyDataLen -= readLen;
        }
        /***************************/

        /* set the appropriate flag in the association response frame */
        /* to indicate whether or not we encountered a Cisco IE, i.e., */
        /* if we have any indication as to whether the AP we've associated */
        /* with is a Cisco AP. */
        pHandle->tempFrameInfo.frame.content.assocRsp.ciscoIEPresent = ciscoIEPresent;

        WLAN_REPORT_INFORMATION(pHandle->hReport, MLME_SM_MODULE_LOG,
                ("MLME_PARSER: ciscoIEPresent = %d\n", ciscoIEPresent));

        status = assoc_recv(pHandle->hAssoc, &(pHandle->tempFrameInfo.frame));
        break;

    case PROBE_REQUEST:
        WLAN_REPORT_SM(pHandle->hReport, MLME_SM_MODULE_LOG,
                       ("MLME_PARSER: recieved PROBE_REQ message \n"));
        break;
    case PROBE_RESPONSE:

        WLAN_REPORT_SM(pHandle->hReport, MLME_SM_MODULE_LOG,
                       ("MLME_PARSER: recieved PROBE_RESPONSE message \n"));

		if(pMsdu->dataLen-WLAN_HDR_LEN-TIME_STAMP_LEN-4 > MAX_BEACON_BODY_LENGTH)
		{
			WLAN_REPORT_INFORMATION(pHandle->hReport, MLME_SM_MODULE_LOG,
			("mlmeParser_recv: probe response length out of range. length=%d, band=%d, channel=%d\n", pMsdu->dataLen-WLAN_HDR_LEN-TIME_STAMP_LEN-4, pRxAttr->band, pRxAttr->channel));
			/* Error in parsing probe response packet - exit */
			wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);
			return NOK;

		}

		/* init frame fields */
        pHandle->tempFrameInfo.frame.content.iePacket.barkerPreambleMode = PREAMBLE_UNSPECIFIED;

        /* read time stamp */
        os_memoryCopy(pHandle->hOs, (void *)pHandle->tempFrameInfo.frame.content.iePacket.timestamp, pData, TIME_STAMP_LEN);
        pData += TIME_STAMP_LEN;

        bodyDataLen -= TIME_STAMP_LEN;
        /* read beacon interval */
        pHandle->tempFrameInfo.frame.content.iePacket.beaconInerval = ENDIAN_HANDLE_WORD(*(UINT16*)pData);
        pData += 2;
        /* read capabilities */
        pHandle->tempFrameInfo.frame.content.iePacket.capabilities = ENDIAN_HANDLE_WORD(*(UINT16*)pData);
        pData += 2;

        bodyDataLen -= 4;
        pHandle->tempFrameInfo.frame.content.iePacket.pRsnIe = NULL;
        pHandle->tempFrameInfo.frame.content.iePacket.rsnIeLen = 0;

        pHandle->tempFrameInfo.band = pRxAttr->band;
        pHandle->tempFrameInfo.rxChannel = pRxAttr->channel;

       if ((pRxAttr->band == RADIO_BAND_2_4_GHZ) && (pRxAttr->channel > NUM_OF_CHANNELS_24))
        {
            WLAN_REPORT_ERROR(pHandle->hReport, MLME_SM_MODULE_LOG,
                ("mlmeParser_recv, band=%d, channel=%d\n", pRxAttr->band, pRxAttr->channel));
            /* Error in parsing Probe response packet - exit */
            wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);
            return NOK;

        }
        else if ((pRxAttr->band == RADIO_BAND_5_0_GHZ) && (pRxAttr->channel <= NUM_OF_CHANNELS_24))
        {
            WLAN_REPORT_ERROR(pHandle->hReport, MLME_SM_MODULE_LOG,
            ("mlmeParser_recv, band=%d, channel=%d\n", pRxAttr->band, pRxAttr->channel));
            /* Error in parsing Probe response packet - exit */
            wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);
            return NOK;

        }
        if (mlmeParser_parseIEs(hMlme, pData, bodyDataLen, &(pHandle->tempFrameInfo)) != OK)
        {
            /* Error in parsing Probe response packet - exit */
            WLAN_REPORT_WARNING(pHandle->hReport, MLME_SM_MODULE_LOG,("mlmeParser_parseIEs: Error in parsing probe response \n"));          
            wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);
            return NOK;
        }

        /* Check if there is a scan in progress */
        if ( NULL != pHandle->resultCBFunc )
        {
            /* result CB is registered - results are sent to the registered callback */
            pHandle->resultCBFunc( pHandle->resultCBObj, &(pHandle->tempFrameInfo.bssid), &(pHandle->tempFrameInfo.frame), pRxAttr,
                                   (UINT8 *)(pMgmtFrame->body+TIME_STAMP_LEN+4),
                                   pMsdu->dataLen-WLAN_HDR_LEN-TIME_STAMP_LEN-4 );
        }
        currBSS_probRespReceivedCallb(pHandle->hCurrBss, pRxAttr, &(pHandle->tempFrameInfo.bssid), 
                                      &(pHandle->tempFrameInfo.frame), 
                                      (char *)pMgmtFrame->body+TIME_STAMP_LEN+4, 
                                      pMsdu->dataLen-WLAN_HDR_LEN-TIME_STAMP_LEN-4);

/* For Debug */
#if BcnMissTst

        /* Printing all the required parameters of the received ProbeRsp */
        currProbeRspTSFTime = (UINT32)ENDIAN_HANDLE_LONG(*(INT32*)(pHandle->tempFrameInfo.frame.content.iePacket.timestamp));
        deltaProbeRspTSFTime = (UINT32)((UINT32)currProbeRspTSFTime - (UINT32)pHandle->debug_lastProbeRspTSFTime);

        if(pHandle->tempFrameInfo.frame.content.iePacket.pSsid != NULL)
        {
            WLAN_REPORT_INFORMATION(pHandle->hReport,
                                    MLME_SM_MODULE_LOG,
                                    ("ProbeRsp ssid=%8s TS=0x%x TSDelta=0x%x beaconInt=%d HostTime = %d\n",
                        pHandle->tempFrameInfo.frame.content.iePacket.pSsid->serviceSetId,
                        currProbeRspTSFTime,
                        deltaProbeRspTSFTime,
                        pHandle->tempFrameInfo.frame.content.iePacket.beaconInerval,
                        os_timeStampMs(pHandle)));
        }

        if(pHandle->tempFrameInfo.frame.content.iePacket.pSsid == NULL)
            WLAN_REPORT_INFORMATION(pHandle->hReport,
                                    MLME_SM_MODULE_LOG,
                                    ("SSID null\n"));

        pHandle->debug_lastProbeRspTSFTime = currProbeRspTSFTime;

#endif /* BcnMissTst */
        if(pHandle->tempFrameInfo.recvChannelSwitchAnnoncIE == FALSE)
        {
            switchChannel_recvCmd(pHandle->hSwitchChannel, NULL, pRxAttr->channel);
        }

        break;
    case BEACON:

        WLAN_REPORT_INFORMATION(pHandle->hReport, MLME_SM_MODULE_LOG,
                       ("MLME_PARSER: recieved BEACON message, TS= %ld\n", os_timeStampMs(pHandle->hOs)));
        WLAN_REPORT_INFORMATION(pHandle->hReport, MLME_SM_MODULE_LOG, ("beacon MSDU") );

		if(pMsdu->dataLen-WLAN_HDR_LEN-TIME_STAMP_LEN-4 > MAX_BEACON_BODY_LENGTH)
		{
			WLAN_REPORT_ERROR(pHandle->hReport, MLME_SM_MODULE_LOG,
			("mlmeParser_recv: beacon length out of range. length=%d, band=%d, channel=%d\n", pMsdu->dataLen-WLAN_HDR_LEN-TIME_STAMP_LEN-4, pRxAttr->band, pRxAttr->channel));
			/* Error in parsing beacon packet - exit */
			wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);
			return NOK;

		}

		/* init frame fields */
        pHandle->tempFrameInfo.frame.content.iePacket.barkerPreambleMode = PREAMBLE_UNSPECIFIED;

        /* read time stamp */
        os_memoryCopy(pHandle->hOs, (void *)pHandle->tempFrameInfo.frame.content.iePacket.timestamp, pData, TIME_STAMP_LEN);
        pData += TIME_STAMP_LEN;

        bodyDataLen -= TIME_STAMP_LEN;
        /* read beacon interval */
        pHandle->tempFrameInfo.frame.content.iePacket.beaconInerval = ENDIAN_HANDLE_WORD(*(UINT16*)pData);
        pData += 2;
        /* read capabilities */
        pHandle->tempFrameInfo.frame.content.iePacket.capabilities = ENDIAN_HANDLE_WORD(*(UINT16*)pData);
        pData += 2;

        bodyDataLen -= 4;
        pHandle->tempFrameInfo.frame.content.iePacket.pRsnIe = NULL;
        pHandle->tempFrameInfo.frame.content.iePacket.rsnIeLen = 0;

        if ((pRxAttr->band == RADIO_BAND_2_4_GHZ) && (pRxAttr->channel > NUM_OF_CHANNELS_24))
        {
            WLAN_REPORT_ERROR(pHandle->hReport, MLME_SM_MODULE_LOG,
            ("mlmeParser_recv, band=%d, channel=%d\n", pRxAttr->band, pRxAttr->channel));
            /* Error in parsing Probe response packet - exit */
            wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);
            return NOK;

        }
        else if ((pRxAttr->band == RADIO_BAND_5_0_GHZ) && (pRxAttr->channel <= NUM_OF_CHANNELS_24))
        {
            WLAN_REPORT_ERROR(pHandle->hReport, MLME_SM_MODULE_LOG,
            ("mlmeParser_recv, band=%d, channel=%d\n", pRxAttr->band, pRxAttr->channel));
            /* Error in parsing Probe response packet - exit */
            wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);
            return NOK;

        }
        pHandle->tempFrameInfo.band = pRxAttr->band;
        pHandle->tempFrameInfo.rxChannel = pRxAttr->channel;

        if (mlmeParser_parseIEs(hMlme, pData, bodyDataLen, &(pHandle->tempFrameInfo)) != OK)
        {
            WLAN_REPORT_WARNING(pHandle->hReport, MLME_SM_MODULE_LOG,("mlmeParser_parseIEs - Error in parsing Beacon \n"));                                 
            /* Error in parsing Beacon packet - exit */
            wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle);
            return NOK;
        }

        /* Check if there is a scan in progress */
        if ( NULL != pHandle->resultCBFunc )
        {
            /* result CB is registered - results are sent to the registered callback */
            pHandle->resultCBFunc( pHandle->resultCBObj, &(pHandle->tempFrameInfo.bssid), &(pHandle->tempFrameInfo.frame), pRxAttr,
                                   (UINT8 *)(pMgmtFrame->body+TIME_STAMP_LEN+4),
                                   pMsdu->dataLen-WLAN_HDR_LEN-TIME_STAMP_LEN-4 );
        }

        /* Counting the number of recieved beacons - used for statistics */
        pHandle->BeaconsCounterPS++;

        currBSS_beaconReceivedCallb(pHandle->hCurrBss, pRxAttr, &(pHandle->tempFrameInfo.bssid), 
                                    &(pHandle->tempFrameInfo.frame), (char *)pMgmtFrame->body+TIME_STAMP_LEN+4, 
                                    pMsdu->dataLen-WLAN_HDR_LEN-TIME_STAMP_LEN-4);

#if BcnMissTst
        if ( pHandle->tempFrameInfo.myBssid ) 
        {
            mlmeParser_printBeaconDebugInfo(hMlme,
                                            pHandle->tempFrameInfo.frame);
        }
#endif
        if (pHandle->tempFrameInfo.recvChannelSwitchAnnoncIE == FALSE)
        {
            switchChannel_recvCmd(pHandle->hSwitchChannel, NULL, pRxAttr->channel);
        }

        break;
    case ATIM:
        if (!pHandle->tempFrameInfo.myBssid)
            break;

        WLAN_REPORT_SM(pHandle->hReport, MLME_SM_MODULE_LOG,
                       ("MLME_PARSER: recieved ATIM message \n"));
        break;
    case DIS_ASSOC:
        if ((!pHandle->tempFrameInfo.myBssid) || (!pHandle->tempFrameInfo.mySa))
            break;

        /* consider the DisAssoc frame if it is one of the following:
			1) unicast frame and directed to our STA
			2) broadcast frame
		*/
		if((pHandle->tempFrameInfo.frame.extesion.destType == MSG_UNICAST) && 
			 (pHandle->tempFrameInfo.myDst == FALSE))
            break;

        /* read Reason interval */
        pHandle->tempFrameInfo.frame.content.disAssoc.reason = ENDIAN_HANDLE_WORD(*(UINT16*)pData);

		{	/* Send roaming trigger */
			roamingEventData_u RoamingEventData;
			RoamingEventData.APDisconnect.uStatusCode = pHandle->tempFrameInfo.frame.content.disAssoc.reason;
			RoamingEventData.APDisconnect.bDeAuthenticate = FALSE; /* i.e. This is not DeAuth packet */
			apConn_reportRoamingEvent(pHandle->hApConn, ROAMING_TRIGGER_AP_DISCONNECT, &RoamingEventData);
		}

		break;
    case AUTH:
        /* Auth response frame is should be directed to our STA, and from the current AP */
		if ((!pHandle->tempFrameInfo.myBssid) || (!pHandle->tempFrameInfo.mySa) || (pHandle->tempFrameInfo.myDst == FALSE))
            break;

        /* read Algorithm interval */
        pHandle->tempFrameInfo.frame.content.auth.authAlgo = ENDIAN_HANDLE_WORD(*(UINT16*)pData);
        pData += 2;
        /* read Sequence number */
        pHandle->tempFrameInfo.frame.content.auth.seqNum = ENDIAN_HANDLE_WORD(*(UINT16*)pData);
        pData += 2;
        /* read status */
        pHandle->tempFrameInfo.frame.content.auth.status = ENDIAN_HANDLE_WORD(*(UINT16*)pData);
        pData += 2;

        WLAN_REPORT_INFORMATION(pHandle->hReport, MLME_SM_MODULE_LOG,
                                ("MLME_PARSER: Read Auth: algo=%d, seq=%d, status=%d\n",
                                pHandle->tempFrameInfo.frame.content.auth.authAlgo,
                                pHandle->tempFrameInfo.frame.content.auth.seqNum,
                                pHandle->tempFrameInfo.frame.content.auth.status));
        bodyDataLen -= 6;
        /* read Challenge */
        pHandle->tempFrameInfo.frame.content.auth.pChallenge = &(pHandle->tempFrameInfo.challenge);
        status = mlmeParser_readChallange(pHandle, pData, bodyDataLen, &readLen, &(pHandle->tempFrameInfo.challenge));
        if (status != OK)
        {
            pHandle->tempFrameInfo.challenge.hdr.eleLen = 0;
            readLen = 0;
        }
        pData += readLen;

        status = auth_recv(pHandle->hAuth, &(pHandle->tempFrameInfo.frame));
        break;
    case DE_AUTH:
        if ((!pHandle->tempFrameInfo.myBssid) || (!pHandle->tempFrameInfo.mySa))
            break;

        /* consider the Assoc frame if it is one of the following:
			1) unicast frame and directed to our STA
			2) broadcast frame
		*/
		if((pHandle->tempFrameInfo.frame.extesion.destType == MSG_UNICAST) && 
			(pHandle->tempFrameInfo.myDst == FALSE))
            break;

        /* read Reason */
        pHandle->tempFrameInfo.frame.content.deAuth.reason = ENDIAN_HANDLE_WORD(*(UINT16*)pData);

		{	/* Send roaming trigger */
			roamingEventData_u RoamingEventData;
			RoamingEventData.APDisconnect.uStatusCode = pHandle->tempFrameInfo.frame.content.disAssoc.reason;
			RoamingEventData.APDisconnect.bDeAuthenticate = TRUE; /* i.e. This is DeAuth packet */
			apConn_reportRoamingEvent(pHandle->hApConn, ROAMING_TRIGGER_AP_DISCONNECT, &RoamingEventData);
		}
        break;

	case ACTION:
		param.paramType = CTRL_DATA_CURRENT_BSS_TYPE_PARAM;
		ctrlData_getParam(pHandle->hCtrlData, &param);

        if ((!pHandle->tempFrameInfo.myBssid) || 
			((!pHandle->tempFrameInfo.mySa) && (param.content.ctrlDataCurrentBssType == BSS_INFRASTRUCTURE)))
            break;

		/* if the action frame is unicast and not directed to our STA, we should ignore it */
		if((pHandle->tempFrameInfo.frame.extesion.destType == MSG_UNICAST) && 
				(pHandle->tempFrameInfo.myDst == FALSE))
            break;
		

        /* read Category field */
        pHandle->tempFrameInfo.frame.content.action.category = *pData;
        pData ++;
        bodyDataLen --;

        /* Checking if the category field is valid */
        if(( pHandle->tempFrameInfo.frame.content.action.category != CATAGORY_SPECTRUM_MANAGEMENT) &&
            (pHandle->tempFrameInfo.frame.content.action.category != CATAGORY_QOS)  &&
            (pHandle->tempFrameInfo.frame.content.action.category != WME_CATAGORY_QOS) )   
        {
            WLAN_REPORT_ERROR(pHandle->hReport, MLME_SM_MODULE_LOG,
                          ("MLME_PARSER: Error category is invalid for action management frame %d \n",
                           pHandle->tempFrameInfo.frame.content.action.category ));
            break;
        }

        switch(pHandle->tempFrameInfo.frame.content.action.category)
        {
            case CATAGORY_QOS:
            case WME_CATAGORY_QOS:           
                /* read action field */
                pHandle->tempFrameInfo.frame.content.action.action = *pData;
                pData ++;
                bodyDataLen --;

                QosMngr_receiveActionFrames(pHandle->hQosMngr, pData, pHandle->tempFrameInfo.frame.content.action.action, bodyDataLen); 
                break;
                
            case CATAGORY_SPECTRUM_MANAGEMENT:
                /* read action field */
                pHandle->tempFrameInfo.frame.content.action.action = *pData;
                pData ++;
                bodyDataLen --;
                
                switch(pHandle->tempFrameInfo.frame.content.action.action)
                {
                    case MEASUREMENT_REQUEST:
                        /* Checking the frame type  */
                        if(pHandle->tempFrameInfo.frame.extesion.destType == MSG_BROADCAST)
                            pHandle->tempFrameInfo.frame.content.action.frameType = MSR_FRAME_TYPE_BROADCAST;
                        else
                            pHandle->tempFrameInfo.frame.content.action.frameType = MSR_FRAME_TYPE_UNICAST;
                        
                            /*measurementMgr_receiveFrameRequest(pHandle->hMeasurementMgr,
                            pHandle->tempFrameInfo.frame.content.action.frameType,
                            bodyDataLen,pData);*/
                        break;

                    case TPC_REQUEST:
                        /*measurementMgr_receiveTPCRequest(pHandle->hMeasurementMgr,(UINT8)bodyDataLen,pData);*/
                        break;
                        
                    case CHANNEL_SWITCH_ANNOUNCEMENT:
                        if (pHandle->tempFrameInfo.myBssid)
                        {   /* Ignore Switch Channel commands from non my BSSID */
                            mlmeParser_readChannelSwitch(pHandle,pData,bodyDataLen,&readLen,&(pHandle->tempFrameInfo.channelSwitch),
                                pRxAttr->channel);
                        }
                        break;
                        
                    default:
                        WLAN_REPORT_ERROR(pHandle->hReport, MLME_SM_MODULE_LOG,
                            ("MLME_PARSER: Error, category is invalid for action management frame %d \n",
                            pHandle->tempFrameInfo.frame.content.action.category ));
                        break;
                }
                
                break;

                
            default:
                status = NOK;
                break;
                    
        }
    }

    /* release MSDU */
    if (wlan_memMngrFreeMSDU(pHandle->hMemMgr, pMsdu->handle) != OK)
    {
        WLAN_REPORT_ERROR(pHandle->hReport, MLME_SM_MODULE_LOG,
                          ("MLME_PARSER: Error releasing MSDU handle %d \n", pMsdu->handle));
    }

    return OK;
}

TI_STATUS mlmeParser_getFrameType(mlme_t *pMlme, UINT16* pFrameCtrl, dot11MgmtSubType_e *pType)
{
    if ((*pFrameCtrl & DOT11_FC_PROT_VERSION_MASK) != DOT11_FC_PROT_VERSION)
    {
        WLAN_REPORT_ERROR(pMlme->hReport, MLME_SM_MODULE_LOG,
                          ("MLME_PARSER: Error Wrong protocol version (not %d) \n", DOT11_FC_PROT_VERSION));
        return NOK;
    }

    if ((*pFrameCtrl & DOT11_FC_TYPE_MASK) != DOT11_FC_TYPE_MGMT)
    {
        WLAN_REPORT_ERROR(pMlme->hReport, MLME_SM_MODULE_LOG,
                          ("MLME_PARSER: Error not MANAGEMENT frame\n"));
        return NOK;
    }

    *pType = (dot11MgmtSubType_e)((*pFrameCtrl & DOT11_FC_SUB_MASK) >> 4);

    return OK;
}

TI_STATUS mlmeParser_read4Xxarams(mlme_t *pMlme,UINT8 *pData, UINT32 dataLen, UINT32 *pReadLen, dot11_4X_t *fourXParams)
{
    fourXParams->hdr.eleId = *pData;
    fourXParams->hdr.eleLen = *(pData+1);

    *pReadLen = fourXParams->hdr.eleLen + 2;

    if ((dataLen < 2) || (dataLen < (UINT32)(fourXParams->hdr.eleLen + 2)))
    {
        return NOK;
    }

    if (fourXParams->hdr.eleLen > DOT11_4X_MAX_LEN)
    {
        return NOK;
    }

    os_memoryCopy(pMlme->hOs, (void *)fourXParams->fourXCapabilities, pData+2, fourXParams->hdr.eleLen);

    return OK;
}


#ifdef EXC_MODULE_INCLUDED
void mlmeParser_readExcOui (UINT8 *pData, UINT32 dataLen, UINT32 *pReadLen, EXCv4IEs_t *excIEs)
{
    UINT8 ieLen;
    UINT8 ouiType;

    ieLen = *(pData+1) + 2;

    if (dataLen < ieLen)
    {
        /* Wrong length of info-element, skip to the end of the packet */
        *pReadLen = dataLen;
        return;
    }

    *pReadLen = ieLen;
    ouiType = *(pData+5);

    switch (ouiType) 
    {
        case TS_METRIX_OUI_TYPE:
            excIEs->tsMetrixParameter = (dot11_TS_METRICS_IE_t *)pData;
            break;
        case TS_RATE_SET_OUI_TYPE:
            excIEs->trafficStreamParameter = (dot11_TSRS_IE_t *)pData;
            break;
        case EDCA_LIFETIME_OUI_TYPE:
            excIEs->edcaLifetimeParameter = (dot11_MSDU_LIFE_TIME_IE_t *)pData;
            break;
    }
    return;
}
#endif


TI_STATUS mlmeParser_readERP(mlme_t *pMlme, UINT8 *pData, UINT32 dataLen, UINT32 *pReadLen,
                             BOOL *useProtection, preamble_e *barkerPreambleMode)
{

    UINT32 erpIElen;
    UINT16 ctrl;
    UINT16 ShortData=0;

    erpIElen = *(pData+1);
    *pReadLen = erpIElen + 2;

    if (dataLen < (UINT32)(erpIElen + 2))
    {
        return NOK;
    }
#ifdef FOUR_ALIGNMENT /*fix for un-aligned exception on ARM */
    ((UINT8 *)&ShortData)[0] = pData[2];
    ((UINT8 *)&ShortData)[1] = pData[3];
    ctrl = ENDIAN_HANDLE_WORD(ShortData);
#else
    ctrl = ENDIAN_HANDLE_WORD(*(UINT16*)(pData+2));
#endif

    *useProtection = (ctrl & 0x2) >>1;
    *barkerPreambleMode = ((ctrl & 0x4) >>2) ? PREAMBLE_LONG : PREAMBLE_SHORT;

    return OK;
}


TI_STATUS mlmeParser_readRates(mlme_t *pMlme, UINT8 *pData, UINT32 dataLen, UINT32 *pReadLen, dot11_RATES_t *pRates)
{
    pRates->hdr.eleId = *pData;
    pRates->hdr.eleLen = *(pData+1);

    *pReadLen = pRates->hdr.eleLen + 2;

    if (dataLen < (UINT32)(pRates->hdr.eleLen + 2))
    {
        return NOK;
    }

    if (pRates->hdr.eleLen > MAX_SUPPORTED_RATES)
    {
        return NOK;
    }

    os_memoryCopy(pMlme->hOs, (void *)pRates->rates, pData+2, pRates->hdr.eleLen);

    return OK;
}

TI_STATUS mlmeParser_readSsid(mlme_t *pMlme, UINT8 *pData, UINT32 dataLen, UINT32 *pReadLen, dot11_SSID_t *pSsid)
{
    pSsid->hdr.eleId = *pData;
    pSsid->hdr.eleLen = *(pData+1);

    *pReadLen = pSsid->hdr.eleLen + 2;

    if ((dataLen < 2) || (dataLen < (UINT32)(pSsid->hdr.eleLen + 2)))
    {
        return NOK;
    }

    if (pSsid->hdr.eleLen > MAX_SSID_LEN)
    {
        return NOK;
    }

    os_memoryCopy(pMlme->hOs, (void *)pSsid->serviceSetId, pData+2, pSsid->hdr.eleLen);

    return OK;
}

TI_STATUS mlmeParser_readFhParams(mlme_t *pMlme, UINT8 *pData, UINT32 dataLen, UINT32 *pReadLen, dot11_FH_PARAMS_t *pFhParams)
{
    UINT16 ShortData=0;
    pFhParams->hdr.eleId = *pData;
    pFhParams->hdr.eleLen = *(pData+1);
    pData += 2;

    if ((dataLen < 2) || (dataLen < (UINT32)(pFhParams->hdr.eleLen + 2)))
    {
        return NOK;
    }

#if 1
    ((UINT8 *)&ShortData)[0] = pData[0];
    ((UINT8 *)&ShortData)[1] = pData[1];
#else
    /* Fix for possible alignment problem */
    COPY_UNALIGNED_WORD(&ShortData , pData);
#endif

    pFhParams->dwellTime = ENDIAN_HANDLE_WORD(ShortData);
    pData += 2;

    pFhParams->hopSet = *pData;
    pFhParams->hopPattern = *(pData+1);
    pFhParams->hopIndex = *(pData+2);

    *pReadLen = pFhParams->hdr.eleLen + 2;

    return OK;
}

TI_STATUS mlmeParser_readDsParams(mlme_t *pMlme, UINT8 *pData, UINT32 dataLen, UINT32 *pReadLen, dot11_DS_PARAMS_t *pDsParams)
{
    pDsParams->hdr.eleId = *pData;
    pDsParams->hdr.eleLen = *(pData+1);

    if ((dataLen < 2) || (dataLen < (UINT32)(pDsParams->hdr.eleLen + 2)))
    {
        return NOK;
    }
	
    pDsParams->currChannel = *(pData+2);

    *pReadLen = pDsParams->hdr.eleLen + 2;

    return OK;
}

TI_STATUS mlmeParser_readCfParams(mlme_t *pMlme, UINT8 *pData, UINT32 dataLen, UINT32 *pReadLen, dot11_CF_PARAMS_t *pCfParams)
{
    UINT16 ShortData=0;
    pCfParams->hdr.eleId = *pData;
    pCfParams->hdr.eleLen = *(pData+1);
    pData += 2;

    if ((dataLen < 2) || (dataLen < (UINT32)(pCfParams->hdr.eleLen + 2)))
    {
        return NOK;
    }

    pCfParams->cfpCount = *pData;
    pCfParams->cfpPeriod = *(pData+1);
    pData += 2;

#if 1
    /* Fix for possible alignment problem */
    ((UINT8 *)&ShortData)[0] = pData[0];
    ((UINT8 *)&ShortData)[1] = pData[1];
#else
    COPY_UNALIGNED_WORD(&ShortData, pData);
#endif
    pCfParams->cfpMaxDuration = ENDIAN_HANDLE_WORD(ShortData);

    pData += 2;

#if 1
    /* Fix for possible alignment problem */
    ((UINT8 *)&ShortData)[0] = pData[0];
    ((UINT8 *)&ShortData)[1] = pData[1];
#else
    /* Fix for possible alignment problem */
    COPY_UNALIGNED_WORD(&ShortData, pData);
#endif
    pCfParams->cfpDurRemain = ENDIAN_HANDLE_WORD(ShortData);

    *pReadLen = pCfParams->hdr.eleLen + 2;

    return OK;
}

TI_STATUS mlmeParser_readIbssParams(mlme_t *pMlme, UINT8 *pData, UINT32 dataLen, UINT32 *pReadLen, dot11_IBSS_PARAMS_t *pIbssParams)
{
    UINT16 ShortData=0;
    pIbssParams->hdr.eleId = *pData;
    pIbssParams->hdr.eleLen = *(pData+1);
    pData += 2;

    if ((dataLen < 2) || (dataLen < (UINT32)(pIbssParams->hdr.eleLen + 2)))
    {
        return NOK;
    }

#if 1
    /* Fix for possible alignment problem */
    ((UINT8 *)&ShortData)[0] = pData[0];
    ((UINT8 *)&ShortData)[1] = pData[1];
#else
    /* Fix for possible alignment problem */
    COPY_UNALIGNED_WORD(&ShortData, pData);
#endif
    pIbssParams->atimWindow = ENDIAN_HANDLE_WORD(ShortData);

    *pReadLen = pIbssParams->hdr.eleLen + 2;

    return OK;
}

TI_STATUS mlmeParser_readTim(mlme_t *pMlme, UINT8 *pData, UINT32 dataLen, UINT32 *pReadLen, dot11_TIM_t *pTim)
{
    pTim->hdr.eleId = *pData;
    pTim->hdr.eleLen = *(pData+1);
    pData += 2;

    if ((dataLen < 2) || (dataLen < (UINT32)(pTim->hdr.eleLen + 2)) || (pTim->hdr.eleLen < 3))
    {
        return NOK;
    }

    pTim->dtimCount = *pData;
    pTim->dtimPeriod = *(pData+1);
    pTim->bmapControl = *(pData+2);
    pData += 3;

    if ((pTim->hdr.eleLen - 3) > DOT11_PARTIAL_VIRTUAL_BITMAP_MAX) /* Dm: Security fix */
    {
        WLAN_REPORT_ERROR(pMlme->hReport, MLME_SM_MODULE_LOG,
                          ("MLME_PARSER: Security Error: eleLen=%d, maxLen=%d\n",
                          pTim->hdr.eleLen, DOT11_PARTIAL_VIRTUAL_BITMAP_MAX));
        return NOK;
    }
    os_memoryCopy(pMlme->hOs, (void *)pTim->partialVirtualBmap, pData, pTim->hdr.eleLen - 3);

    *pReadLen = pTim->hdr.eleLen + 2;

    return OK;
}

TI_STATUS mlmeParser_readCountry(mlme_t *pMlme,UINT8 *pData, UINT32 dataLen, UINT32 *pReadLen, dot11_COUNTRY_t *countryIE)
{
    countryIE->hdr.eleId = *pData;
    countryIE->hdr.eleLen = *(pData+1);

    *pReadLen = countryIE->hdr.eleLen + 2;

    if ((dataLen < 8) || (dataLen < (UINT32)(countryIE->hdr.eleLen + 2)))
    {
        return NOK;
    }

    if (countryIE->hdr.eleLen > DOT11_COUNTRY_ELE_LEN_MAX)
    {
        WLAN_REPORT_ERROR(pMlme->hReport, MLME_SM_MODULE_LOG,
                          ("MLME_PARSER: country IE error: eleLen=%d, maxLen=%d\n",
                          countryIE->hdr.eleLen, DOT11_COUNTRY_ELE_LEN_MAX));
        return NOK;
    }

    os_memoryCopy(pMlme->hOs,&(countryIE->countryIE), pData+2, countryIE->hdr.eleLen);

    return OK;
}

TI_STATUS mlmeParser_readWMEParams(mlme_t *pMlme,UINT8 *pData, UINT32 dataLen, 
                                   UINT32 *pReadLen, dot11_WME_PARAM_t *pWMEParamIE, 
                                   assocRsp_t *assocRsp)
{
    UINT8 ieSubtype;
    UINT8 ac;

    pWMEParamIE->hdr.eleId = *pData;
    pWMEParamIE->hdr.eleLen = *(pData+1);

    *pReadLen = pWMEParamIE->hdr.eleLen + 2;

    if (dataLen < *pReadLen)
    {
        WLAN_REPORT_ERROR(pMlme->hReport, MLME_SM_MODULE_LOG,
                          ("MLME_PARSER: WME Parameter: eleLen=%d is too long (%d)\n", *pReadLen, dataLen));
        *pReadLen = dataLen;
        return NOK;
    }

    if ((pWMEParamIE->hdr.eleLen > WME_TSPEC_IE_LEN) || (pWMEParamIE->hdr.eleLen < DOT11_WME_ELE_LEN))
    {
        WLAN_REPORT_ERROR(pMlme->hReport, MLME_SM_MODULE_LOG,
                          ("MLME_PARSER: WME Parameter IE error: eleLen=%d\n", pWMEParamIE->hdr.eleLen));
        return NOK;
    }

    ieSubtype = *((UINT8*)(pData+6));
    switch (ieSubtype)
    {
        case dot11_WME_OUI_SUB_TYPE_IE:
        case dot11_WME_OUI_SUB_TYPE_PARAMS_IE:
            /* Checking WME Version validity */
            if (*((UINT8*)(pData+7)) != dot11_WME_VERSION )
            {
                WLAN_REPORT_INFORMATION(pMlme->hReport, MLME_SM_MODULE_LOG,
                                  ("MLME_PARSER: WME Parameter IE error: Version =%d is unsupported\n",
                                  *((UINT8*)(pData+7)) ));
                return NOK;
            }
            /* Copy either the WME-Params IE or the WME-Info IE (Info is a subset of Params)! */
            os_memoryCopy(pMlme->hOs,&(pWMEParamIE->OUI), pData+2, pWMEParamIE->hdr.eleLen);
            break;

        case WME_TSPEC_IE_OUI_SUB_TYPE:
            /* Read renegotiated TSPEC parameters */
            if (assocRsp == NULL) 
            {
                WLAN_REPORT_ERROR(pMlme->hReport, MLME_SM_MODULE_LOG,
                                  ("MLME_PARSER: WME Parameter IE error: TSPEC Sub Type in beacon or probe resp\n"));
                return NOK;
            }
            ac = WMEQosTagToACTable[((((dot11_WME_TSPEC_IE_t *)pData)->tHdr.tsInfoField.tsInfoArr[1]) & TS_INFO_1_USER_PRIORITY_MASK) >> USER_PRIORITY_SHIFT];

            if (ac == QOS_AC_VO)
            {
                assocRsp->tspecVoiceParameters = (dot11_WME_TSPEC_IE_t *)pData;
            }
            else if (ac == QOS_AC_VI)
            {
                assocRsp->tspecSignalParameters = (dot11_WME_TSPEC_IE_t *)pData;
            }
            break;

        default:
            /* Checking OUI Sub Type validity */
            WLAN_REPORT_ERROR(pMlme->hReport, MLME_SM_MODULE_LOG,
                              ("MLME_PARSER: WME Parameter IE error: Sub Type =%d is invalid\n",
                              ieSubtype));
            return NOK;
    }
    return OK;
}


TI_STATUS mlmeParser_readQosCapabilityIE(mlme_t *pMlme,UINT8 *pData, UINT32 dataLen, UINT32 *pReadLen, dot11_QOS_CAPABILITY_IE_t *QosCapParams)
{
    QosCapParams->hdr.eleId = *pData;
    QosCapParams->hdr.eleLen = *(pData+1);

    *pReadLen = QosCapParams->hdr.eleLen + 2;

    if (dataLen < (UINT32)(QosCapParams->hdr.eleLen + 2))
    {
        return NOK;
    }

    if (QosCapParams->hdr.eleLen > DOT11_QOS_CAPABILITY_ELE_LEN)
    {
        WLAN_REPORT_ERROR(pMlme->hReport, MLME_SM_MODULE_LOG,
                          ("MLME_PARSER: QOS Capability  IE error: eleLen=%d, maxLen=%d\n",
                          QosCapParams->hdr.eleLen, DOT11_QOS_CAPABILITY_ELE_LEN));
        return NOK;
    }

   QosCapParams->QosInfoField = (*(pData+1));
    return OK;
}


TI_STATUS mlmeParser_readChallange(mlme_t *pMlme, UINT8 *pData, UINT32 dataLen, UINT32 *pReadLen, dot11_CHALLENGE_t *pChallange)
{
    if (dataLen < 2)
    {
        return NOK;
    }

    pChallange->hdr.eleId = *pData;
    pChallange->hdr.eleLen = *(pData+1);
    pData += 2;

    if ((dataLen < 2) || (dataLen < (UINT32)(pChallange->hdr.eleLen + 2)))
    {
        return NOK;
    }

    if (pChallange->hdr.eleLen > DOT11_CHALLENGE_TEXT_MAX)
    {
        return NOK;
    }

    os_memoryCopy(pMlme->hOs, (void *)pChallange->text, pData, pChallange->hdr.eleLen);

    *pReadLen = pChallange->hdr.eleLen + 2;

    return OK;
}

TI_STATUS mlmeParser_readRsnIe(mlme_t *pMlme, UINT8 *pData, UINT32 dataLen, UINT32 *pReadLen, dot11_RSN_t *pRsnIe)
{
    pRsnIe->hdr.eleId = *pData;
    pRsnIe->hdr.eleLen = *(pData+1);
    pData += 2;

    if ((dataLen < 2) || (dataLen < (UINT32)(pRsnIe->hdr.eleLen + 2)))
    {
        return NOK;
    }

    os_memoryCopy(pMlme->hOs, (void *)pRsnIe->rsnIeData, pData, pRsnIe->hdr.eleLen);

    *pReadLen = pRsnIe->hdr.eleLen + 2;

    return OK;
}

TI_STATUS mlmeParser_readPowerConstraint(mlme_t *pMlme,UINT8 *pData, UINT32 dataLen, UINT32 *pReadLen, dot11_POWER_CONSTRAINT_t *powerConstraintIE)
{
    powerConstraintIE->hdr.eleId = *pData;
    powerConstraintIE->hdr.eleLen = *(pData+1);

    *pReadLen = powerConstraintIE->hdr.eleLen + 2;

    if ((dataLen < 2) || (dataLen < (UINT32)(powerConstraintIE->hdr.eleLen + 2)))
    {
        return NOK;
    }

    if (powerConstraintIE->hdr.eleLen > DOT11_POWER_CONSTRAINT_ELE_LEN)
    {
        return NOK;
    }

    os_memoryCopy(pMlme->hOs,(void *)&(powerConstraintIE->powerConstraint), pData+2, powerConstraintIE->hdr.eleLen);

    return OK;
}


TI_STATUS mlmeParser_readChannelSwitch(mlme_t *pMlme,UINT8 *pData, UINT32 dataLen, UINT32 *pReadLen, dot11_CHANNEL_SWITCH_t *channelSwitch, UINT8 channel)
{
    channelSwitch->hdr.eleId = *pData++;
    channelSwitch->hdr.eleLen = *pData++;

    *pReadLen = channelSwitch->hdr.eleLen + 2;

    if ((dataLen < 2) || (dataLen < (UINT32)(channelSwitch->hdr.eleLen + 2)))
    {
        return NOK;
    }

    if (channelSwitch->hdr.eleLen > DOT11_CHANNEL_SWITCH_ELE_LEN)
    {
        return NOK;
    }

    channelSwitch->channelSwitchMode = *pData++;
    channelSwitch->channelNumber = *pData++;
    channelSwitch->channelSwitchCount = *pData;


    switchChannel_recvCmd(pMlme->hSwitchChannel, channelSwitch, channel);
    return OK;
}

TI_STATUS mlmeParser_readQuiet(mlme_t *pMlme,UINT8 *pData, UINT32 dataLen, UINT32 *pReadLen, dot11_QUIET_t *quiet)
{
    quiet->hdr.eleId = *pData++;
    quiet->hdr.eleLen = *pData++;

    *pReadLen = quiet->hdr.eleLen + 2;

    if ((dataLen < 2) || (dataLen < (UINT32)(quiet->hdr.eleLen + 2)))
    {
        return NOK;
    }

    if (quiet->hdr.eleLen > DOT11_QUIET_ELE_LEN)
    {
        return NOK;
    }

    quiet->quietCount = *pData++;
    quiet->quietPeriod = *pData++;
    quiet->quietDuration = *((UINT16*)pData); pData+=2; /* Dm: */
    quiet->quietOffset = *((UINT16*)pData);

    return OK;
}


TI_STATUS mlmeParser_readTPCReport(mlme_t *pMlme,UINT8 *pData, UINT32 dataLen, UINT32 *pReadLen, dot11_TPC_REPORT_t *TPCReport)
{
    TPCReport->hdr.eleId = *pData;
    TPCReport->hdr.eleLen = *(pData+1);

    *pReadLen = TPCReport->hdr.eleLen + 2;

    if ((dataLen < 2) || (dataLen < (UINT32)(TPCReport->hdr.eleLen + 2)))
    {
        return NOK;
    }

    if (TPCReport->hdr.eleLen > DOT11_TPC_REPORT_ELE_LEN)
    {
        return NOK;
    }

    TPCReport->transmitPower = *(pData+2);

    return OK;
}


#ifdef EXC_MODULE_INCLUDED
TI_STATUS mlmeParser_readCellTP(mlme_t *pMlme, UINT8 *pData, UINT32 dataLen, UINT32 *pReadLen, dot11_CELL_TP_t *cellTP)
{
    UINT8 exc_OUI[] = EXC_OUI;

    cellTP->hdr.eleId = *pData++;
    cellTP->hdr.eleLen = *pData++;

    *pReadLen = cellTP->hdr.eleLen + 2;

    if ((dataLen < 2) || (dataLen < (UINT32)(cellTP->hdr.eleLen + 2)))
    {
        return NOK;
    }

    if (cellTP->hdr.eleLen > DOT11_CELL_TP_ELE_LEN)
    {
        return NOK;
    }

    os_memoryCopy(pMlme->hOs, (PVOID)cellTP->oui, pData, cellTP->hdr.eleLen);

    if (os_memoryCompare(pMlme->hOs, (PVOID)cellTP->oui, exc_OUI, 3) != 0)
    {
        return NOK;
    }

    return OK;
}
#endif

TI_STATUS mlmeParser_registerForBeaconAndProbeResp( TI_HANDLE hMlme, 
                                                    mlme_resultCB_t resultCBFunc, 
                                                    TI_HANDLE resultCBObj )
{
    mlme_t* pMlme = (mlme_t*)hMlme;

    if ( NULL != pMlme->resultCBFunc )
    {
        WLAN_REPORT_WARNING( pMlme->hReport, MLME_SM_MODULE_LOG,
                             ("trying to register for beacons and probe responses when someone is already registered!.\n") );
        return NOK;
    }

    pMlme->resultCBFunc = resultCBFunc;
    pMlme->resultCBObj = resultCBObj;
    return OK;
}

void mlmeParser_unregisterForBeaconAndProbeResp( TI_HANDLE hMlme )
{
    mlme_t* pMlme = (mlme_t*)hMlme;

    pMlme->resultCBFunc = NULL;
    pMlme->resultCBObj = NULL;
}

void mlme_beaconReceivedPrint(TI_HANDLE hMlme)
{
#if BcnMissTst
    mlme_t *pMlme = (mlme_t*) hMlme;
    WLAN_REPORT_INFORMATION(pMlme->hReport,MLME_SM_MODULE_LOG,
                                ("Beacon Missed - FW interrupt\n"));
#endif
}
#if BcnMissTst
static void mlmeParser_printBeaconDebugInfo(TI_HANDLE theMlmeHandle,
                                            mlmeFrameInfo_t theFrame)
{
    mlme_t      *pHandle = (mlme_t*)theMlmeHandle;
    paramInfo_t param;
    UINT32      CurNumBcnMissed = 0;
    BOOL        BeaconMissiedFlag = FALSE;
    UINT32      currBeaconTSFTime = 0;
    UINT32      deltaBeaconTSFTime = 0;
    INT32       dtimPeriod;
    INT32       dtimCount;
    UINT8       bmapControl = 0;
    UINT8       partialVirtualBmap = 0;
    UINT8       aid = 0;


    param.paramType = TX_DATA_PORT_STATUS_PARAM;
    txData_getParam(pHandle->hTxData,&param);
    if(param.content.txDataPortStatus == CLOSE)
    {
        return;
    }

    param.paramType = SITE_MGR_CURRENT_SSID_PARAM;
    siteMgr_getParam(pHandle->hSiteMgr,&param);
    if(strcmp(theFrame.content.iePacket.pSsid->serviceSetId , param.content.siteMgrCurrentSSID.ssidString) != 0)
    {
        /* If the beacon is not from the primary site, ignore it */
        return;
    }


        if(theFrame.content.iePacket.pTIM != NULL)
        {
            whalParamInfo_t whalParam;

            dtimPeriod  = theFrame.content.iePacket.pTIM->dtimPeriod;
            dtimCount   = theFrame.content.iePacket.pTIM->dtimCount;
            bmapControl = theFrame.content.iePacket.pTIM->bmapControl;

            whalParam.paramType = HAL_CTRL_AID_PARAM;
            whalCtrl_GetParam (pHandle->hHalCtrl, &whalParam) ;
            aid = whalParam.content.halCtrlAid;
            if (aid < DOT11_PARTIAL_VIRTUAL_BITMAP_MAX)
            {
                partialVirtualBmap = theFrame.content.iePacket.pTIM->partialVirtualBmap[aid/8] & (1 << (aid%8) );
            }
            else
            {
                WLAN_REPORT_ERROR(pHandle->hReport,
                                  MLME_SM_MODULE_LOG,
                                  ("%s(%d) - Invalid AID (=%d)\n",
                                  __FILE__,__LINE__,aid));
            }
        }
        else
        {
            WLAN_REPORT_INFORMATION(pHandle->hReport,
                                    MLME_SM_MODULE_LOG,
                                    ("Beacon without TIM element\n"));
            dtimPeriod  = 1;
            dtimCount   = 0;
        }

    /* Printing all the required parameters of the received beacon only if the station is connected*/
    currBeaconTSFTime = (UINT32)ENDIAN_HANDLE_LONG(*(INT32*)(theFrame.content.iePacket.timestamp));


    /*
    **  in case Configured to wake up on DTIM
    **  ---------------------------------------
    **  in case received beacon is DTIM
    **  if delta time between the current DTIM and the former DTIM is Bigger then
    **  DTIM Time -> there is a beacon miss (DTIM)
    **
    */
    if ((theFrame.content.iePacket.pTIM != NULL) && (dtimPeriod > 1))
    {
        if(pHandle->debug_isFunctionFirstTime == TRUE)
        {
            if( (dtimCount == 0) && (dtimPeriod > 1))
            {
                pHandle->debug_isFunctionFirstTime = FALSE;
                pHandle->debug_lastDtimBcnTSFTime = currBeaconTSFTime ;
            }
        }


        /* to avoid dummy BeaconMiss, just until first DTIM received */
        if((pHandle->debug_isFunctionFirstTime == TRUE) && (dtimPeriod > 1))
        {
            return;
        }

        if(dtimCount == 0)
        {
            deltaBeaconTSFTime = (UINT32)((UINT32)currBeaconTSFTime - (UINT32)pHandle->debug_lastDtimBcnTSFTime);

            CurNumBcnMissed = (deltaBeaconTSFTime + ((UINT32)theFrame.content.iePacket.beaconInerval * 500) ) /
                    ((UINT32)theFrame.content.iePacket.beaconInerval * dtimPeriod * 1000);

            if(CurNumBcnMissed >=1)
            {
                CurNumBcnMissed -- ;
            }

            if (CurNumBcnMissed != 0)
            {
                WLAN_REPORT_INFORMATION(pHandle->hReport,
                                        MLME_SM_MODULE_LOG,
                                        ("BeaconMiss : currBcnTS=0x%x TSDelta=0x%x DtimCnt=%d DtimPriod=%d beaconInt=%d lastDtimBcnTSFTime = 0x%x \n",
                                        currBeaconTSFTime,
                                        deltaBeaconTSFTime,
                                        dtimCount ,
                                        dtimPeriod ,
                                        theFrame.content.iePacket.beaconInerval,
                                        pHandle->debug_lastDtimBcnTSFTime));
            }

            pHandle->debug_lastDtimBcnTSFTime = currBeaconTSFTime ;
        }

        else /* (dtimCount != 0) */
        {
            deltaBeaconTSFTime = (UINT32)((UINT32)currBeaconTSFTime - (UINT32)pHandle->debug_lastDtimBcnTSFTime);

            if (deltaBeaconTSFTime > ((UINT32)theFrame.content.iePacket.beaconInerval * dtimPeriod * 1000))
            {
                CurNumBcnMissed =   (deltaBeaconTSFTime -
                                    ((UINT32)(dtimPeriod - dtimCount) * (UINT32)theFrame.content.iePacket.beaconInerval * 1000)  +
                                    ((UINT32)theFrame.content.iePacket.beaconInerval * 500) ) /
                                    ((UINT32)theFrame.content.iePacket.beaconInerval * dtimPeriod * 1000);

            }
            else
            {
                CurNumBcnMissed =   0 ;
            }

            if (CurNumBcnMissed != 0)
            {
                WLAN_REPORT_INFORMATION(pHandle->hReport,
                                        MLME_SM_MODULE_LOG,
                                        ("BeaconMiss : currBcnTS=0x%x TSDelta=0x%x DtimCnt=%d DtimPriod=%d beaconInt=%d lastDtimBcnTSFTime = 0x%x \n",
                                        currBeaconTSFTime,
                                        deltaBeaconTSFTime,
                                        dtimCount ,
                                        dtimPeriod ,
                                        theFrame.content.iePacket.beaconInerval,
                                        pHandle->debug_lastDtimBcnTSFTime));
            }
        }

    } /* Configured to wake up on DTIM */


    /*
    **  in case Configured to wake up on Beacon
    **  ---------------------------------------
    */
    else
    {
        if(pHandle->debug_isFunctionFirstTime == TRUE)
        {
            pHandle->debug_isFunctionFirstTime = FALSE;
            pHandle->debug_lastBeaconTSFTime = currBeaconTSFTime;
        }

        deltaBeaconTSFTime = (UINT32)((UINT32)currBeaconTSFTime - (UINT32)pHandle->debug_lastBeaconTSFTime);

        CurNumBcnMissed = (deltaBeaconTSFTime + ((UINT32)theFrame.content.iePacket.beaconInerval * 500) ) /
                          ((UINT32)theFrame.content.iePacket.beaconInerval * dtimPeriod * 1000);

        if(CurNumBcnMissed >=1)
        {
            CurNumBcnMissed -- ;
        }

        if (CurNumBcnMissed != 0)
        {
            WLAN_REPORT_INFORMATION(pHandle->hReport, MLME_SM_MODULE_LOG,
                                    ("BeaconMiss : currBcnTS=0x%x TSDelta=0x%x DtimCnt=%d DtimPriod=%d beaconInt=%d lastBcnTSFTime = 0x%x \n",
                                    currBeaconTSFTime,
                                    deltaBeaconTSFTime,
                                    dtimCount ,
                                    dtimPeriod ,
                                    theFrame.content.iePacket.beaconInerval,
                                    pHandle->debug_lastBeaconTSFTime));
        }

        pHandle->debug_lastBeaconTSFTime = currBeaconTSFTime;

    } /* Configured to wake up on Beacon */



    if (CurNumBcnMissed != 0)
    {
        BeaconMissiedFlag = TRUE;
    }

    /* in case DTIM beacon miss */
    if (BeaconMissiedFlag == TRUE)
    {
        pHandle->totalMissingBeaconsCounter += CurNumBcnMissed;
        if(CurNumBcnMissed > pHandle->maxMissingBeaconSequence)
        {
            pHandle->maxMissingBeaconSequence = CurNumBcnMissed;
        }

        WLAN_REPORT_INFORMATION(pHandle->hReport,
                                MLME_SM_MODULE_LOG,
                                ("Beacon Missed Total = %d and currently %d beacon missed\n",pHandle->totalMissingBeaconsCounter,CurNumBcnMissed));
    }


#if BcnMissTstWithScrPad7
    if (BeaconMissiedFlag == TRUE)
    {
        if (MissingBcnInt == TRUE)
        {
            WLAN_REPORT_INFORMATION(pHandle->hReport,
                                    MLME_SM_MODULE_LOG,
                                    ("Driver write to SCR_PAD7 0x1\n"));

            whalCtrlWriteMacReg(pHandle->hHalCtrl, SCR_PAD7 ,0x1);
        }
        else
        {
            WLAN_REPORT_INFORMATION(pHandle->hReport,
                                            MLME_SM_MODULE_LOG,
                                            ("Driver write to SCR_PAD7 0x3\n"));

            whalCtrlWriteMacReg(pHandle->hHalCtrl, SCR_PAD7 ,0x3);
        }
    }
    else
    {
        if (MissingBcnInt == TRUE)
        {
            WLAN_REPORT_INFORMATION(pHandle->hReport,
                                    MLME_SM_MODULE_LOG,
                                    ("Driver write to SCR_PAD7 0x4\n"));

            whalCtrlWriteMacReg(pHandle->hHalCtrl, SCR_PAD7 ,0x4);
        }
        else
        {
            WLAN_REPORT_INFORMATION(pHandle->hReport,
                                    MLME_SM_MODULE_LOG,
                                    ("Driver write to SCR_PAD7 0x2\n"));

            whalCtrlWriteMacReg(pHandle->hHalCtrl, SCR_PAD7 ,0x2);
        }
    }
#endif

    if(theFrame.content.iePacket.pTIM != NULL && theFrame.content.iePacket.pSsid != NULL)
    {
        WLAN_REPORT_INFORMATION(pHandle->hReport,
                                MLME_SM_MODULE_LOG,
                                ("Beacon ssid=%8s TS=0x%x TSDelta=0x%x DtimCnt=%d DtimPriod=%d bmapControl=0x%02X partialVirtualBmap=0x%02X beaconInt=%d HostTime=%d Total=%d\n",
                                theFrame.content.iePacket.pSsid->serviceSetId,
                                currBeaconTSFTime,
                                deltaBeaconTSFTime,
                                dtimCount ,
                                dtimPeriod ,
                                bmapControl,
                                partialVirtualBmap,
                                theFrame.content.iePacket.beaconInerval,
                                os_timeStampMs(pHandle),
                                ++(pHandle->totalRcvdBeaconsCounter)));

#if BcnMissTstWithScrPad7
        if(BeaconMissiedFlag != TRUE)
        {
            whalCtrlWriteMacReg(pHandle->hHalCtrl, SCR_PAD7 ,0x2);
        }
#endif
    }

    if(theFrame.content.iePacket.pTIM == NULL)
    {
        WLAN_REPORT_INFORMATION(pHandle->hReport,
                                MLME_SM_MODULE_LOG,
                                ("Tim null\n"));
    }
    if(theFrame.content.iePacket.pSsid == NULL)
    {
        WLAN_REPORT_INFORMATION(pHandle->hReport,
                                MLME_SM_MODULE_LOG,
                                ("SSID null\n"));
    }
}

#endif

#if CHECK_PARSING_ERROR_CONDITION_PRINT
    #define CHECK_PARSING_ERROR_CONDITION(x, msg, bDump)       \
            if ((x)) \
            { \
                WLAN_REPORT_ERROR(pHandle->hReport, MLME_SM_MODULE_LOG, msg); \
                if (bDump) {\
                    WLAN_REPORT_ERROR(pHandle->hReport, MLME_SM_MODULE_LOG, ("Buff len = %d \n", packetLength)); \
                    HexDumpData(pPacketBody, packetLength); }\
                return NOK; \
            }
#else
    #define CHECK_PARSING_ERROR_CONDITION(x, msg, bDump) \
         if ((x)) return NOK;
#endif

TI_STATUS mlmeParser_parseIEs(TI_HANDLE *hMlme, 
                             UINT8 *pData,
                             INT32 bodyDataLen,
                             mlmeIEParsingParams_t *params)
{
    dot11_eleHdr_t      *pEleHdr;
    BOOL                skipIE = FALSE;
    UINT32              readLen;
    TI_STATUS           status = NOK;
    UINT8               rsnIeIdx = 0;
    UINT8               wpaIeOuiIe[4] = { 0x00, 0x50, 0xf2, 0x01};
    UINT8               ti_oui[] = TI_OUI;
    UINT8               lastIE = 0;
    beacon_probeRsp_t   *frame = &(params->frame.content.iePacket);
    mlme_t              *pHandle = (mlme_t *)hMlme;
#ifdef EXC_MODULE_INCLUDED
    BOOL                allowCellTP = TRUE;
#endif
#if CHECK_PARSING_ERROR_CONDITION_PRINT
    INT32               packetLength = bodyDataLen;
    UINT8               *pPacketBody = pData;
#endif

    params->recvChannelSwitchAnnoncIE = FALSE;

    while (bodyDataLen > 1)
    {
        pEleHdr = (dot11_eleHdr_t *)pData;
    
        CHECK_PARSING_ERROR_CONDITION((pEleHdr->eleLen > (bodyDataLen - 2)), ("MLME_PARSER: IE %d with length %d out of bounds %d\n", pEleHdr->eleId, pEleHdr->eleLen, (bodyDataLen - 2)),TRUE);

        /* IEs in the packet must be ordered in increased order        */
        /* exept of WPA and RSN IEs! I.e. if current IE id less than   */
        /* the last one , we just skip such element.                   */
        /* If the current IE is more than the last IE id (and it is not*/
        /* WPA or RSN one), we save its IE id  as the last id.         */
    
        skipIE = FALSE;
        if((pEleHdr->eleId != WPA_IE_ID) && (pEleHdr->eleId != RSN_IE_ID))
        {
            if (pEleHdr->eleId < lastIE)
            {
                readLen = 2 + pEleHdr->eleLen;
                skipIE = TRUE;
            }
            else 
            {
                lastIE = pEleHdr->eleId;
            }
        }
    
        if(!skipIE)
        {
            switch (pEleHdr->eleId)
            {
            /* read SSID */
            case SSID_IE_ID:
                frame->pSsid = &params->ssid;
                status = mlmeParser_readSsid(pHandle, pData, bodyDataLen, &readLen, frame->pSsid);
                CHECK_PARSING_ERROR_CONDITION((status != OK), ("MLME_PARSER: error reading SSID\n"),TRUE);
                break;
            /* read rates */
            case SUPPORTED_RATES_IE_ID:
                frame->pRates = &params->rates;
                status = mlmeParser_readRates(pHandle, pData, bodyDataLen, &readLen, frame->pRates);
                CHECK_PARSING_ERROR_CONDITION((status != OK), ("MLME_PARSER: error reading RATES\n"),TRUE);
                break;
            case EXT_SUPPORTED_RATES_IE_ID:
                frame->pExtRates = &params->extRates;
                status = mlmeParser_readRates(pHandle, pData, bodyDataLen, &readLen, frame->pExtRates);
                CHECK_PARSING_ERROR_CONDITION((status != OK), ("MLME_PARSER: error reading EXT RATES\n"),TRUE);
                break;
    
            case ERP_IE_ID:
                status = mlmeParser_readERP(pHandle, pData, bodyDataLen, &readLen,
                                            (BOOL *)&frame->useProtection,
                                            (preamble_e *)&frame->barkerPreambleMode);
                CHECK_PARSING_ERROR_CONDITION((status != OK), ("MLME_PARSER: error reading ERP\n"),TRUE);
                break;
            /* read FH parameter set */
            case FH_PARAMETER_SET_IE_ID:
                frame->pFHParamsSet = &params->fhParams;
                status = mlmeParser_readFhParams(pHandle, pData, bodyDataLen, &readLen, frame->pFHParamsSet);
                CHECK_PARSING_ERROR_CONDITION((status != OK), ("MLME_PARSER: error reading FH parameters\n"),TRUE);
                break;
            /* read DS parameter set */
            case DS_PARAMETER_SET_IE_ID:
                frame->pDSParamsSet = &params->dsParams;
                status = mlmeParser_readDsParams(pHandle, pData, bodyDataLen, &readLen, frame->pDSParamsSet);
                CHECK_PARSING_ERROR_CONDITION((status != OK), ("MLME_PARSER: error reading DS parameters\n"),TRUE);
                if (RADIO_BAND_2_4_GHZ == params->band )
                {
                    CHECK_PARSING_ERROR_CONDITION((frame->pDSParamsSet->currChannel != params->rxChannel), 
                            ("Channel ERROR - incompatible channel source information: Frame=%d Vs Radio=%d.\n\
                            parser ABORTED!!!\n",
                            frame->pDSParamsSet->currChannel , params->rxChannel),FALSE);
                }
                break;
            /* read CF parameter set */
            case CF_PARAMETER_SET_IE_ID:
                frame->pCFParamsSet = &params->cfParams;
                status = mlmeParser_readCfParams(pHandle, pData, bodyDataLen, &readLen, frame->pCFParamsSet);
                CHECK_PARSING_ERROR_CONDITION((status != OK), ("MLME_PARSER: error reading CF parameters\n"),TRUE);
                break;
            /* read IBSS parameter set */
            case IBSS_PARAMETER_SET_IE_ID:
                frame->pIBSSParamsSet = &params->ibssParams;
                status = mlmeParser_readIbssParams(pHandle, pData, bodyDataLen, &readLen, frame->pIBSSParamsSet);
                CHECK_PARSING_ERROR_CONDITION((status != OK), ("MLME_PARSER: error reading IBSS parameters\n"),TRUE);
                break;

            /* read TIM */
            case TIM_IE_ID:
                frame->pTIM = &params->tim;
                status = mlmeParser_readTim(pHandle, pData, bodyDataLen, &readLen, frame->pTIM);
                CHECK_PARSING_ERROR_CONDITION((status != OK), ("MLME_PARSER: error reading TIM\n"),TRUE);
                break;
    
            /* read Country */
            case COUNTRY_IE_ID:
                frame->country = &params->country;
                status = mlmeParser_readCountry(pHandle, pData, bodyDataLen, &readLen, frame->country);
                CHECK_PARSING_ERROR_CONDITION((status != OK), ("MLME_PARSER: error reading country parameters\n"),TRUE);
                break;
    
            /* read Power Constraint */
            case POWER_CONSTRAINT_IE_ID:
#ifdef EXC_MODULE_INCLUDED
                allowCellTP = FALSE;
#endif
                frame->powerConstraint = &params->powerConstraint;
                status = mlmeParser_readPowerConstraint(pHandle, pData, bodyDataLen, &readLen, frame->powerConstraint);
                CHECK_PARSING_ERROR_CONDITION((status != OK), ("MLME_PARSER: error reading Power Constraint parameters\n"),TRUE);
                break;
    
            /* read Channel Switch Mode */
            case CHANNEL_SWITCH_ANNOUNCEMENT_IE_ID:
                if (params->myBssid)
                {   /* Ignore Switch Channel commands from non my BSSID */
                    params->recvChannelSwitchAnnoncIE = TRUE;
                    frame->channelSwitch = &params->channelSwitch;
                    status = mlmeParser_readChannelSwitch(pHandle, pData, bodyDataLen, &readLen, frame->channelSwitch, params->rxChannel);
                    if (status != OK)
                    {
                        /*
                         * PATCH for working with AP-DK 4.0.51 that use IE 37 (with length 20) for RSNE
                         * Ignore the IE instead of rejecting the whole Msdu (beacon or probe response)
                         */
                        WLAN_REPORT_WARNING(pHandle->hReport, MLME_SM_MODULE_LOG,
                                          ("MLME_PARSER: error reading Channel Switch announcement parameters - ignore IE\n"));
                    }
                }
                break;
    
            /* read Quiet IE */
            case QUIET_IE_ID:
                frame->quiet = &params->quiet;
                status = mlmeParser_readQuiet(pHandle, pData, bodyDataLen, &readLen, frame->quiet);
                CHECK_PARSING_ERROR_CONDITION((status != OK), ("MLME_PARSER: error reading Quiet parameters\n"),TRUE);
                break;
    
            /* read TPC report IE */
            case TPC_REPORT_IE_ID:
                frame->TPCReport = &params->TPCReport;
                status = mlmeParser_readTPCReport(pHandle, pData, bodyDataLen, &readLen, frame->TPCReport);
                CHECK_PARSING_ERROR_CONDITION((status != OK), ("MLME_PARSER: error reading TPC report parameters\n"),TRUE);
                break;
    
            case EXC_EXT_1_IE_ID:
                frame->pRsnIe   = &params->rsnIe[0];
                status = mlmeParser_readRsnIe(pHandle, pData, bodyDataLen, &readLen, &params->rsnIe[rsnIeIdx]);
                CHECK_PARSING_ERROR_CONDITION((status != OK), ("MLME_PARSER: error reading RSN IE\n"),TRUE);
    
                frame->rsnIeLen += readLen;
                rsnIeIdx ++;
                break;
    
            case RSN_IE_ID:
                frame->pRsnIe = &params->rsnIe[0];
                status = mlmeParser_readRsnIe(pHandle, pData, bodyDataLen, &readLen, &params->rsnIe[rsnIeIdx]);
                CHECK_PARSING_ERROR_CONDITION((status != OK), ("MLME_PARSER: error reading RSN IE\n"),TRUE);
    
                frame->rsnIeLen += readLen;
                rsnIeIdx ++;
                break;
    
            case DOT11_QOS_CAPABILITY_ELE_ID:
                frame->QoSCapParameters = &params->QosCapParams;
                status = mlmeParser_readQosCapabilityIE(pHandle, pData, bodyDataLen, &readLen, &params->QosCapParams);
                CHECK_PARSING_ERROR_CONDITION((status != OK), ("MLME_PARSER: error reading QOS CapabilityIE\n"),TRUE);
                break;

            case WPA_IE_ID:
                if (!os_memoryCompare(pHandle->hOs, pData+2, wpaIeOuiIe, 3))
                {
                    /* Note : WPA and WME use the same OUI */
                    /*  Its assumes that:
                            WPA uses OUI Type with value  - 1
                            WME uses OUI Type with value  - 2
                    */
    
                    /* Check the OUI sub Type to verify whether this is a WME IE or WPA IE*/
                    if( (*(UINT8*)(pData+5)) == dot11_WPA_OUI_TYPE)
                    {
                        /* If we are here - the following is WPA IE */
                        frame->pRsnIe = &params->rsnIe[0];
                        status = mlmeParser_readRsnIe(pHandle, pData, bodyDataLen,
                                                      &readLen, &params->rsnIe[rsnIeIdx]);
                        frame->rsnIeLen += readLen;
                        rsnIeIdx ++;
    
                        CHECK_PARSING_ERROR_CONDITION((status != OK), ("MLME_PARSER: error reading RSN IE\n"),TRUE);
                    }
                    if( ( (*(UINT8*)(pData+5)) == dot11_WME_OUI_TYPE ) && 
                        ( ( (*(UINT8*)(pData+6)) == dot11_WME_OUI_SUB_TYPE_PARAMS_IE) ||
                          ( (*(UINT8*)(pData+6)) == dot11_WME_OUI_SUB_TYPE_IE) ) )
                    {
                        /* If we are here - the following is WME-Params IE, WME-Info IE or TSPEC IE. */
                        /* Note that we are using the WMEParams struct also to hold the WME-Info IE
                             which is a subset of WMEParams, and only one of them is sent in a frame. */
                        frame->WMEParams = &params->WMEParams;
                        status = mlmeParser_readWMEParams(pHandle, pData, bodyDataLen, &readLen, frame->WMEParams, NULL);
                        
                        CHECK_PARSING_ERROR_CONDITION((status != OK), ("MLME_PARSER: error reading WME params\n"),TRUE);
                    }
                    else
                    {
                        /* Unrecognized OUI type */
                        readLen = pEleHdr->eleLen + 2;
                    }
                }
                else
                /* check if this is 4X IE */
                if(os_memoryCompare(pHandle->hOs, ti_oui, pData+2, DOT11_OUI_LEN) == 0)
                {
                    frame->fourXParams = &params->fourXParams;
                    status = mlmeParser_read4Xxarams(pHandle, pData, bodyDataLen, &readLen, frame->fourXParams);
                    CHECK_PARSING_ERROR_CONDITION((status != OK), ("MLME_PARSER: error reading 4X parameters\n"),TRUE);
                }
                else
                {
                    /* not 4x IE */
                    readLen = pEleHdr->eleLen + 2;
                }

                break;

#ifdef EXC_MODULE_INCLUDED
            case CELL_POWER_IE:
                /* We mustn't take the Cell Transmit Power IE into account if */
                /* there's a Power Constraint IE. Since the IEs must be in increasing */
                /* order, it's enough to perform the check here, because if the Power */
                /* Constraint IE is present it must have already been processed. */ 
                if (allowCellTP)
                {
                    frame->cellTP = &params->cellTP;
                    status = mlmeParser_readCellTP(pHandle, pData, bodyDataLen, &readLen, frame->cellTP);
                    CHECK_PARSING_ERROR_CONDITION((status != OK), ("MLME_PARSER: error reading Cell Transmit Power params.\n"),TRUE);
                }
                break;
#endif

            default:
                WLAN_REPORT_INFORMATION(pHandle->hReport, MLME_SM_MODULE_LOG,
                                      ("MLME_PARSER: unsupported IE found (%d)\n", pEleHdr->eleId));
                readLen = pEleHdr->eleLen + 2;
                break;
            }
        }
        pData += readLen;
        bodyDataLen -= readLen;
        CHECK_PARSING_ERROR_CONDITION((bodyDataLen < 0), ("MLME_PARSER: negative bodyDataLen %d bytes\n", bodyDataLen),TRUE);
    }
    return OK;
}

mlmeIEParsingParams_t *mlmeParser_getParseIEsBuffer(TI_HANDLE *hMlme)
{
    return (&(((mlme_t *)hMlme)->tempFrameInfo));
}

