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
 *   MODULE:  whalHwMboxConfig.c
 *   PURPOSE: Handle the wlan hardware information elements:
 *
 ****************************************************************************/

#include "whalCommon.h"
#include "whalHwDefs.h"
#include "public_infoele.h"
#include "CmdQueue_api.h"
#include "whalHwMboxConfig.h"

/****************************************************************************
 *                      whal_hwMboxConfig_Create()
 ****************************************************************************
 * DESCRIPTION: Create the mailbox configuration commands object
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: The Created object
 ****************************************************************************/
HwMboxConfig_T* whal_hwMboxConfig_Create (TI_HANDLE hOs)
{
    HwMboxConfig_T* pObj;

    pObj = os_memoryAlloc (hOs, sizeof(HwMboxConfig_T));
    if (pObj == NULL)
        return NULL;

    os_memoryZero (hOs, (void *)pObj, sizeof(HwMboxConfig_T));

    pObj->hOs = hOs;

    return(pObj);
}

/****************************************************************************
 *                      whal_hwMboxConfig_Destroy()
 ****************************************************************************
 * DESCRIPTION: Destroy the object
 *
 * INPUTS:
 *      pHwMboxConfig       The object to free
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwMboxConfig_Destroy (HwMboxConfig_T* pHwMboxConfig)
{
    if (pHwMboxConfig)
        os_memoryFree(pHwMboxConfig->hOs, pHwMboxConfig, sizeof(HwMboxConfig_T));

    return (OK);
}

/****************************************************************************
 *                      whal_hwMboxConfig_Config()
 ****************************************************************************
 * DESCRIPTION: Configure the object
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwMboxConfig_Config (HwMboxConfig_T* pHwMboxConfig,TI_HANDLE hCmdMboxQueue, TI_HANDLE hReport)
{
    pHwMboxConfig->hReport = hReport;
    pHwMboxConfig->hCmdMboxQueue = hCmdMboxQueue;

    return (OK);
}



/****************************************************************************
 *                      whal_hwInfoElemConfigMemorySet()
 ****************************************************************************
 * DESCRIPTION: Configure wlan hardware memory
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemConfigMemorySet (HwMboxConfig_T* pHwMboxConfig, DmaParams_T *pDmaParams)
{   
    ACXConfigMemoryStruct_t AcxElm_ConfigMemory;
    ACXConfigMemoryStruct_t *pCfg = &AcxElm_ConfigMemory;
    int Qid;
    
    os_memoryZero(pHwMboxConfig->hOs, (void *)pCfg, sizeof(*pCfg));
    /*
     * Set information element header
     */
    pCfg->memConfig.EleHdr.id = ACX_MEM_CFG;
    pCfg->memConfig.EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /*
     * Set configuration fields
     */
    pCfg->memConfig.numStations      = ENDIAN_HANDLE_WORD(pDmaParams->NumStations); /*(1)*/
    pCfg->memConfig.rxMemblockNumber = pDmaParams->rxMemBlkNumber;  /*(60)*/
    pCfg->memConfig.txMinimumMemblockNumber = pDmaParams->txMinMemBlkNumber; /*(60)*/
    pCfg->memConfig.numTxQueues      = pDmaParams->NumTxQueues; /*(4)*/
    pCfg->memConfig.hostifOptions    = 3; /*(3 - HOST_IF_PKT_RING)*/
    pCfg->memConfig.numSsidProfiles = 1;
    pCfg->memConfig.debugBufferSize  = ENDIAN_HANDLE_WORD(pDmaParams->TraceBufferSize/4); /*(4)*/
    
        /*
     * Rx queue config
     */
    pCfg->RxQueueConfig.dmaAddress  = 0;  
    pCfg->RxQueueConfig.numDescs    = (UINT8)pDmaParams->RxNumDesc;
    pCfg->RxQueueConfig.Priority    = (UINT8)pDmaParams->RxQPriority;
    pCfg->RxQueueConfig.Type        = pDmaParams->RxQueue_Type;

    /*
     * Tx queue config
     */
    for (Qid=0; Qid<pDmaParams->NumTxQueues; Qid++)
    {
        pCfg->TxQueueConfig[Qid].numDescs      = pDmaParams->TxNumDesc[Qid];
        pCfg->TxQueueConfig[Qid].attributes    = pDmaParams->TxQPriority[Qid];
    }

    /* The structure contain array of TxQueueConfig_T
     * The size of the array is QUEUE_CONFIG_MAX_TX_QUEUES buf the actual number of
     * Queues are pCfg->NumTxQueues so the structure length must be fixed */
    pCfg->memConfig.EleHdr.len -= (NUM_ACCESS_CATEGORIES_QUEUES - pDmaParams->NumTxQueues) * sizeof(ACXtxQueueConfig);

    /*
     * Send the configuration command
     */
    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
}


/****************************************************************************
 *                      whal_hwInfoElemDataPathParamsSet()
 ****************************************************************************
 * DESCRIPTION: configure Data path and TX complete parameters
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int  whal_hwInfoElemDataPathParamsSet (HwMboxConfig_T* pHwMboxConfig,UINT16 rxPacketRingChunkSize,
                                       UINT16 txPacketRingChunkSize, UINT8 rxPacketRingChunkNum,
                                       UINT8 txPacketRingChunkNum, UINT8 txCompleteThreshold,
                                       UINT8  txCompleteRingDepth,   UINT32 txCompleteTimeOut)
{
    ACXDataPathParams_t DataPathParams;
    ACXDataPathParams_t*  pCfg = &DataPathParams;

    /*
     * Set information element header
     */
    pCfg->EleHdr.id = ACX_DATA_PATH_PARAMS;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

     /*
     * Set configuration fields
     */
    pCfg->rxPacketRingChunkSize = ENDIAN_HANDLE_WORD(rxPacketRingChunkSize);
    pCfg->txPacketRingChunkSize = ENDIAN_HANDLE_WORD(txPacketRingChunkSize );
    pCfg->rxPacketRingChunkNum  = rxPacketRingChunkNum; 
    pCfg->txPacketRingChunkNum  = txPacketRingChunkNum; 
    pCfg->txCompleteThreshold   = txCompleteThreshold;  
    pCfg->txCompleteRingDepth   = txCompleteRingDepth;
    pCfg->txCompleteTimeOut     = txCompleteTimeOut;

    /* Send the configuration command*/
    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
}


/****************************************************************************
 *                      whal_hwInfoElemDataPathParamsGet()
 ****************************************************************************
 * DESCRIPTION: Get data path specific parameters
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int  whal_hwInfoElemDataPathParamsGet (HwMboxConfig_T* pHwMboxConfig, ACXDataPathParamsResp_t* pCfg, void *fCb, TI_HANDLE hCb)
{
    int rc;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_DATA_PATH_PARAMS;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Send the interrogation command*/
    if ((rc = CmdQueue_CmdInterrogateWithCb (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg), fCb, hCb, pCfg)) == OK)
    {
    }

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemQueueHeadGet()
 ****************************************************************************
 * DESCRIPTION: Read the Queue addresses after memory init
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemQueueHeadGet (HwMboxConfig_T* pHwMboxConfig, int NumTxQueues, int pElem, void *fCb, TI_HANDLE hCb)
{
#if 0 /*Benzy: should be fixed after the firmware/driver integration*/
    int Qid;
    int Stt;

    /* only interrogate is enabled to this info element */
    if (aAction != INTERROGATE_ACTION)
        return (NOK);

    /*
     * Set information element header
     */
    pElem->EleHdr.id  = ACX_QUEUE_HEAD;
    pElem->EleHdr.len = sizeof(*pElem) - sizeof(EleHdrStruct);

    /*
     * Send the interrogation command, and fill the structure
     */
    if ((Stt=CmdQueue_CmdInterrogateWithCb (pHwMboxConfig->hCmdMboxQueue, pElem, sizeof(*pElem), fCb, hCb, pElem)) != OK)
        return (Stt);

    /*
     * Solve endian problem
     */
    pElem->rxMemBlkQ  = (UINT32 *)ENDIAN_HANDLE_LONG((UINT32)(pElem->rxMemBlkQ));
    pElem->txMemBlkQ  = (UINT32 *)ENDIAN_HANDLE_LONG((UINT32)(pElem->txMemBlkQ));
    pElem->rxQueueHead.addr = ENDIAN_HANDLE_LONG(pElem->rxQueueHead.addr);

    for (Qid=0; Qid<NumTxQueues; Qid++)
    {
        pElem->txQueueHead[Qid].addr = ENDIAN_HANDLE_LONG(pElem->txQueueHead[Qid].addr);
    }
#endif
    WLAN_OS_REPORT(("Command Disabled: whal_hwInfoElemQueueHeadGet\n"));
    return OK;
}


/****************************************************************************
 *                      whal_hwInfoElemSlotTimeSet()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate the Slot Time
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemSlotTimeSet (HwMboxConfig_T* pHwMboxConfig, UINT8* apSlotTime)                                
{
    int rc;
    ACXSlot_t   AcxElm_SlotTime;
    ACXSlot_t   *pCfg = &AcxElm_SlotTime;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_SLOT;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Set configuration fields */
    /* woneIndex is not relevant to station implementation */
    pCfg->woneIndex = STATION_WONE_INDEX;
    pCfg->slotTime = *apSlotTime;


    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
        ("%s: Sending info elem to firmware, Slot Time = %d\n", __FUNCTION__, (UINT8)pCfg->slotTime));

    /* Send the configuration command */
    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemPreambleSet()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate the Preamble
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemPreambleSet (HwMboxConfig_T* pHwMboxConfig, UINT8* apPreamble)
{
    int rc;
    ACXPreamble_t   AcxElm_Preamble;
    ACXPreamble_t   *pCfg = &AcxElm_Preamble;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_PREAMBLE_TYPE;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Set configuration fields */
    /* woneIndex is not relevant to station implementation */
    pCfg->preamble = *apPreamble;

    /* Send the configuration command */
    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemGeneraedFrameRate()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate the rate
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemGeneratedFrameRateSet (HwMboxConfig_T *pHwMboxConfig,
                                          UINT8          *txCtrlFrmRate,
                                          UINT8          *txCtrlFrmMod,
                                          UINT8          *txMgmtFrmRate,
                                          UINT8          *txMgmtFrmMod)
{
    int rc;
    ACXFwGeneratedFrameRates_t   AcxElm_FrameRate;
    ACXFwGeneratedFrameRates_t   *pCfg = &AcxElm_FrameRate;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_FW_GEN_FRAME_RATES;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Set configuration fields */
    /* woneIndex is not relevant to station implementation */
    pCfg->txCtrlFrmRate = *txCtrlFrmRate;
    pCfg->txCtrlFrmMod  = *txCtrlFrmMod;
    pCfg->txMgmtFrmRate = *txMgmtFrmRate;
    pCfg->txMgmtFrmMod  = *txMgmtFrmMod;

    /* Send the configuration command */
    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemRxConfigSet()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate RxConfig information element
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemRxConfigSet (HwMboxConfig_T* pHwMboxConfig, UINT32* apRxConfigOption, UINT32* apRxFilterOption)
{
    int rc;
    ACXRxConfig_t AcxElm_RxConfig;
    ACXRxConfig_t* pCfg = &AcxElm_RxConfig;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_RX_CFG;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Set configuration fields */
    pCfg->ConfigOptions = ENDIAN_HANDLE_LONG(*apRxConfigOption);
    pCfg->FilterOptions = ENDIAN_HANDLE_LONG(*apRxFilterOption);

    /* Send the configuration command */
    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));

    return rc;
}

/****************************************************************************
 *                      whal_hwInfoElemBETSet()
 ****************************************************************************
 * DESCRIPTION: Configures Beacon Early Termination information element
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemBETSet(HwMboxConfig_T* pHwMboxConfig, UINT8 Enable, UINT8 MaximumConsecutiveET)
{
    int rc;
    ACXBet_Enable_t ACXBet_Enable;
    ACXBet_Enable_t* pCfg = &ACXBet_Enable;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_BET_ENABLE;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Set configuration fields */
    pCfg->Enable = Enable;
    pCfg->MaximumConsecutiveET = MaximumConsecutiveET;

    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
        ("%s: Sending info elem to firmware, Enable=%d, MaximumConsecutiveET=%d\n", 
		 __FUNCTION__, (UINT8)pCfg->Enable, (UINT8)pCfg->MaximumConsecutiveET));

    /* Send the configuration command */
    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));

    return rc;
}

/****************************************************************************
*                 whal_hwInfoElemSetRxDataFiltersParams()
*****************************************************************************
* DESCRIPTION: Enables or disables Rx data filtering.
*
* INPUTS:  enabled             - 0 to disable data filtering, any other value to enable 
*          defaultAction       - The default action to take on non-matching packets.
*
* OUTPUT:  None
*
* RETURNS: OK or NOK
****************************************************************************/
int whal_hwInfoElemSetRxDataFiltersParams(HwMboxConfig_T * pHwMboxConfig, BOOL enabled, filter_e defaultAction)
{
    DataFilterDefault_t dataFilterDefault;
    DataFilterDefault_t * pCfg = &dataFilterDefault;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_ENABLE_RX_DATA_FILTER;
    pCfg->EleHdr.len = 0;

    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
        ("%s: Rx Data Filter configuration:\n", __FUNCTION__));
    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
        ("%s: enabled = %d, defaultAction = %d\n", __FUNCTION__, enabled, defaultAction));

    /* Set information element configuration fields */
    pCfg->enable = enabled;
    pCfg->action = defaultAction;
    pCfg->EleHdr.len += sizeof(pCfg->enable) + sizeof(pCfg->action);

    WLAN_REPORT_HEX_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG, (UINT8 *) pCfg, sizeof(dataFilterDefault));

    return CmdQueue_CmdConfigure(pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
}


/****************************************************************************
*                      whal_hwInfoElemSetRxDataFilter()
*****************************************************************************
* DESCRIPTION: Add/remove Rx Data filter information element.
*
* INPUTS:  index               - Index of the Rx Data filter
*          command             - Add or remove the filter
*          action              - Action to take on packets matching the pattern
*          numFieldPatterns    - Number of field patterns in the filter
*          lenFieldPatterns    - Length of the field pattern series
*          fieldPatterns       - Series of field patterns
*
* OUTPUT:  None
*
* RETURNS: OK or NOK
****************************************************************************/
int whal_hwInfoElemSetRxDataFilter(HwMboxConfig_T * pHwMboxConfig, UINT8 index, UINT8 command, filter_e action, UINT8 numFieldPatterns, UINT8 lenFieldPatterns, UINT8 * fieldPatterns)
{
    UINT8 dataFilterConfig[sizeof(DataFilterConfig_t) + MAX_DATA_FILTER_SIZE];
    DataFilterConfig_t * pCfg = (DataFilterConfig_t *) &dataFilterConfig;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_SET_RX_DATA_FILTER;
    pCfg->EleHdr.len = 0;

    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
        ("%s: Rx Data Filter configuration:\n", __FUNCTION__));
    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
        ("%s: command = %d, index = %d, action = %d, numFieldPatterns = %d\n", __FUNCTION__,
        command, index, action, numFieldPatterns));

    /* Set information element configuration fields */
    pCfg->command = command;
    pCfg->index = index;
    pCfg->EleHdr.len += sizeof(pCfg->command) + sizeof(pCfg->index);

    /* When removing a filter only the index and command are to be sent */
    if (command == ADD_FILTER)
    {
        pCfg->action = action;
        pCfg->numOfFields = numFieldPatterns;
        pCfg->EleHdr.len += sizeof(pCfg->action) + sizeof(pCfg->numOfFields);

        if (fieldPatterns == NULL)
        {
            WLAN_REPORT_ERROR(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
                ("%s: Null pattern table argument received!\n", __FUNCTION__));

            return PARAM_VALUE_NOT_VALID;
        }

        os_memoryCopy(pHwMboxConfig->hOs, &pCfg->FPTable, fieldPatterns, lenFieldPatterns);
        pCfg->EleHdr.len += lenFieldPatterns;
    }

    WLAN_REPORT_HEX_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG, (UINT8 *) pCfg, sizeof(dataFilterConfig));

    return CmdQueue_CmdConfigure(pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(dataFilterConfig));
}

/****************************************************************************
 *                      whal_hwInfoElemGetRxDataFiltersStatistics()
 ****************************************************************************
 * DESCRIPTION: Get the ACX GWSI counters 
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemGetRxDataFiltersStatistics(HwMboxConfig_T * pHwMboxConfig, 
                                              void * CB_Func, 
                                              TI_HANDLE CB_handle, 
                                              void * CB_Buf)
{
    ACXDataFilteringStatistics_t acx;
    ACXDataFilteringStatistics_t * pCfg = &acx;

    /* Set information element header */
    pCfg->EleHdr.id  = ACX_GET_DATA_FILTER_STATISTICS;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    WLAN_REPORT_HEX_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG, (UINT8 *) pCfg, sizeof(ACXDataFilteringStatistics_t));

    /* Send the interrogation command */
    return CmdQueue_CmdInterrogateWithCb(pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg), CB_Func, CB_handle, CB_Buf);
}


/****************************************************************************
 *                      whalCtrl_getConsumptionStatistics()
 ****************************************************************************
 * DESCRIPTION: Get the ACX Power consumption statistics 
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whalCtrl_getConsumptionStatistics(HwMboxConfig_T * pHwMboxConfig, void * CB_Func, TI_HANDLE CB_handle, void * CB_Buf)
{
    ACXPowerConsumptionTimeStat_t acx;
    ACXPowerConsumptionTimeStat_t * pCfg = &acx;

    /* Set information element header */
    pCfg->EleHdr.id  = ACX_PWR_CONSUMPTION_STATISTICS;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);
    
    /* Send the interrogation command */
    return CmdQueue_CmdInterrogateWithCb(pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg), CB_Func, CB_handle, CB_Buf);
}


/****************************************************************************
 *                      whal_hwInfoElemarpIpAddressesTableSet()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate ARP addr table information element for
 *              ipV4 only
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemarpIpAddressesTableSet (HwMboxConfig_T* pHwMboxConfig, 
                                           IpAddress_t *IP_addr,
                                           UINT32 isFilteringEnabled)
{
    int rc;
    ACXConfigureIP_t AcxElm_CmdConfigureIP;
    ACXConfigureIP_t *pCfg = &AcxElm_CmdConfigureIP;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_ARP_IP_FILTER;
    pCfg->EleHdr.len = sizeof(ACXConfigureIP_t) - sizeof(EleHdrStruct);

    pCfg->arpFilterEnable = isFilteringEnabled;
        
    /* IP address */
    /* Note that in the case of IPv4 it is assumed that the extra two bytes are zero */
    os_memoryCopy (pHwMboxConfig->hOs, (PVOID)pCfg->address, (PVOID)IP_addr->addr, IP_V4_ADDR_LEN);
    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,("ip filtering : %d.%d.%d.%d Enabled = %d \n" , pCfg->address[0] , pCfg->address[1] , pCfg->address[2] , pCfg->address[3] , isFilteringEnabled)) ;

    /* Send the configuration command */
    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(ACXConfigureIP_t));

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemGroupAdressesTableSet()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate Group addr table information element
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemGroupAdressesTableSet (HwMboxConfig_T* pHwMboxConfig, 
                                       UINT8* numGroupAddrs, 
                                       macAddress_t *Group_addr, 
                                       UINT8* isEnabled)
{
    int i = 0;
    int rc = OK;
    UINT8   *tmpLoc = NULL;
    dot11MulticastGroupAddrStart_t  AcxElm_CmdConfigureMulticastIp;
    dot11MulticastGroupAddrStart_t* pCfg = &AcxElm_CmdConfigureMulticastIp;
    
    if ( NULL == pHwMboxConfig )
    {
        return PARAM_VALUE_NOT_VALID;
    }
    
    os_memoryZero (pHwMboxConfig->hOs, (void *)pCfg, sizeof(dot11MulticastGroupAddrStart_t));

    /* Set information element header */
    pCfg->EleHdr.id = DOT11_GROUP_ADDRESS_TBL;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);
     
    pCfg->numOfGroups = *numGroupAddrs;
    pCfg->fltrState = *isEnabled;
    tmpLoc = pCfg->dataLocation;
        
    if (NULL != Group_addr)
    {
        for (i=0; i<*numGroupAddrs; i++) 
        {
            os_memoryCopy(pHwMboxConfig->hOs, (PVOID)&(tmpLoc[MAC_ADDR_SIZE*i]), (PVOID)&(Group_addr->addr[MAC_ADDR_SIZE*i]), MAC_ADDR_SIZE);
            WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
                ("whal_hwInfoElemGroupAdressesTable: MAC %x: %x:%x:%x:%x:%x:%x\n",
                i,
                tmpLoc[MAC_ADDR_SIZE*i+0] ,
                tmpLoc[MAC_ADDR_SIZE*i+1] ,
                tmpLoc[MAC_ADDR_SIZE*i+2] ,
                tmpLoc[MAC_ADDR_SIZE*i+3] ,
                tmpLoc[MAC_ADDR_SIZE*i+4] ,
                tmpLoc[MAC_ADDR_SIZE*i+5]));        
        }
    }

    /* Send the configuration command */
    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(dot11MulticastGroupAddrStart_t));

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemStationIdSet ()
 ****************************************************************************
 * DESCRIPTION: Configure StationId information element to/from
 *      the wlan hardware.
 *      This information element specifies the MAC Address assigned to the
 *      STATION or AP.
 *      This default value is the permanent MAC address that is stored in the
 *      adaptor's non-volatile memory.
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemStationIdSet 
(
    HwMboxConfig_T           *pHwMboxConfig, 
    UINT8                    *apStationId
) 
{
    int i;
    int rc;
    dot11StationIDStruct AcxElm_StationId;
    dot11StationIDStruct* pCfg = &AcxElm_StationId;

    /* Set information element header*/
    pCfg->EleHdr.id = DOT11_STATION_ID;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Set configuration field (reversed order, see docs)*/
    for (i = 0; i < 6; i++)
        pCfg->dot11StationID[i] = apStationId[5-i];

    /* Send the configuration command*/
    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemStationIdGet ()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate StationId information element to/from
 *      the wlan hardware.
 *      This information element specifies the MAC Address assigned to the
 *      STATION or AP.
 *      This default value is the permanent MAC address that is stored in the
 *      adaptor's non-volatile memory.
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemStationIdGet 
(
    HwMboxConfig_T* pHwMboxConfig, 
    void*           fCb,
    TI_HANDLE       hCb,
    void*           pCb
) 
{
    int rc;
    dot11StationIDStruct AcxElm_StationId;
    dot11StationIDStruct* pCfg = &AcxElm_StationId;

    /* Set information element header*/
    pCfg->EleHdr.id = DOT11_STATION_ID;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    if ((rc = CmdQueue_CmdInterrogateWithCb (pHwMboxConfig->hCmdMboxQueue, 
                                             pCfg, 
                                             sizeof(*pCfg),
                                             fCb,
                                             hCb,
                                             pCb)) == OK)
    {
    }

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemStationIdForRecoveryGet ()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate StationId information element to/from
 *      the wlan hardware.
 *      This information element specifies the MAC Address assigned to the
 *      STATION or AP.
 *      This default value is the permanent MAC address that is stored in the
 *      adaptors non-volatile memory.
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemStationIdForRecoveryGet (HwMboxConfig_T* pHwMboxConfig,
                                            void *CB_Func, TI_HANDLE CB_handle, dot11StationIDStruct* CB_Buf)
{

    dot11StationIDStruct AcxElm_StationId;
    dot11StationIDStruct* pCfg = &AcxElm_StationId;

    /* Set information element header*/
    pCfg->EleHdr.id = DOT11_STATION_ID;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    CmdQueue_CmdInterrogateWithCb (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg), CB_Func, CB_handle, CB_Buf);

    return OK;
}


/****************************************************************************
 *                      whal_hwInfoElemRSSIGet ()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate StationId information element to/from
 *      the wlan hardware.
 *      This information element specifies the MAC Address assigned to the
 *      STATION or AP.
 *      This default value is the permanent MAC address that is stored in the
 *      adaptor's non-volatile memory.
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemRSSIGet (HwMboxConfig_T* pHwMboxConfig, void *CB_Func, TI_HANDLE CB_handle, PUINT8 CB_Buf)
{
    int rc;
    ACXRoamingStatisticsTable_t AcxElm_GetAverageRSSI;
    ACXRoamingStatisticsTable_t* pCfg = &AcxElm_GetAverageRSSI;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_ROAMING_STATISTICS_TBL;
    pCfg->EleHdr.len = sizeof(ACXRoamingStatisticsTable_t) - sizeof(EleHdrStruct);
    
    rc = CmdQueue_CmdInterrogateWithCb (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(ACXRoamingStatisticsTable_t), CB_Func, CB_handle, CB_Buf);

    /* Send the configuration command */
    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemACIConfigurationSet()
 ****************************************************************************
 * DESCRIPTION: Configure the hardware ACI parameters
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemACIConfigurationSet (HwMboxConfig_T* pHwMboxConfig, UINT8 ACIMode,
                                        UINT8 inputCCA, UINT8 qualifiedCCA,
                                        UINT8 stompForRx, UINT8 stompForTx,
                                        UINT8 txCCA)
{
    int rc;
    ACXConfigACI_t AcxElm_AciConfig;
    ACXConfigACI_t* pCfg = &AcxElm_AciConfig;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_ACI_OPTION_CFG;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Set sequence number */
    pCfg->ACIMode = ACIMode;
    pCfg->inputCCA = inputCCA;
    pCfg->qualifiedCCA = qualifiedCCA;
    pCfg->stompForRx = stompForRx;
    pCfg->stompForTx = stompForTx;
    pCfg->txCCA = txCCA;

    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
    
    return rc;
}



/****************************************************************************
 *                      whal_hwInfoElemBssPowerSaveGet()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate the Bss in/not power save
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemBssPowerSaveGet (HwMboxConfig_T* pHwMboxConfig, UINT8* apBssInPs_Val)                                
{
    int rc;
    ACXBSSPowerSave_t   AcxElm_BssPowerSave;
    ACXBSSPowerSave_t   *pCfg = &AcxElm_BssPowerSave;

    /* Set information element heade r*/
    pCfg->EleHdr.id = ACX_BSS_IN_PS;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Set configuration fields */
    pCfg->BSSinPowerSave = *apBssInPs_Val;

    /* Send the configuration command */
    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemSoftGeminiEnableSet()
 ****************************************************************************
 * DESCRIPTION: Enable/Disable the BTH-WLAN  
 *
 * INPUTS:  Enable flag
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemSoftGeminiEnableSet (HwMboxConfig_T* pHwMboxConfig, SoftGeminiEnableModes_e SoftGeminiEnableModes)
{
    ACXBluetoothWlanCoEnableStruct        AcxElm_BluetoothWlanEnable;
    ACXBluetoothWlanCoEnableStruct* pCfg = &AcxElm_BluetoothWlanEnable;

    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
        ("whal_hwInfoElemSoftGeminiEnableSet: Enable flag = %d\n", SoftGeminiEnableModes));

    /* Set information element header */
    pCfg->EleHdr.id = ACX_SG_ENABLE;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Set enable field */
    pCfg->Enable = (UINT8)SoftGeminiEnableModes;

    /* Send the configuration command */
    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
}


/****************************************************************************
 *                      whal_hwInfoElemSoftGeminiParamsSet()
 ****************************************************************************
 * DESCRIPTION: Configure the BTH-WLAN co-exsistance   
 *
 * INPUTS:  Configuration structure pointer 
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemSoftGeminiParamsSet (HwMboxConfig_T* pHwMboxConfig, SoftGeminiParam_t *SoftGeminiParam)
{
    ACXBluetoothWlanCoParamsStruct          AcxElm_BluetoothWlanEnable;
    ACXBluetoothWlanCoParamsStruct* pCfg = &AcxElm_BluetoothWlanEnable;

    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
        ("whal_hwInfoElemSoftGeminiParamsSet. \n"));

    /* Set information element header */
    pCfg->EleHdr.id = ACX_SG_CFG;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);
    pCfg->afhLeverageOn         = SoftGeminiParam->afhLeverageOn;
    pCfg->btHpMaxTime           = SoftGeminiParam->btHpMaxTime;
    pCfg->maxNumCts             = SoftGeminiParam->maxNumCts;
    pCfg->numberOfBtPackets     = SoftGeminiParam->numberOfBtPackets;
    pCfg->numberOfWlanPackets   = SoftGeminiParam->numberOfWlanPackets;
    pCfg->numberQuietCycle      = SoftGeminiParam->numberQuietCycle;
    pCfg->protectiveRxTimeBeforeBtHp = SoftGeminiParam->protectiveRxTimeBeforeBtHp;
    pCfg->protectiveTxTimeBeforeBtHp = SoftGeminiParam->protectiveTxTimeBeforeBtHp;
    pCfg->protectiveRxTimeBeforeBtHpFastAp = SoftGeminiParam->protectiveRxTimeBeforeBtHpFastAp;
    pCfg->protectiveTxTimeBeforeBtHpFastAp = SoftGeminiParam->protectiveTxTimeBeforeBtHpFastAp;
    pCfg->protectiveWlanCycleTimeForFastAp = SoftGeminiParam->protectiveWlanCycleTimeForFastAp;
    pCfg->senseDisableTimer     = SoftGeminiParam->senseDisableTimer;
    pCfg->sgAntennaType         = SoftGeminiParam->sgAntennaType;
    pCfg->signalingType         = SoftGeminiParam->signalingType;
    pCfg->timeoutNextBtLpPacket = SoftGeminiParam->timeoutNextBtLpPacket;
    pCfg->wlanHpMaxTime         = SoftGeminiParam->wlanHpMaxTime;
    pCfg->numberOfMissedRxForAvalancheTrigger = SoftGeminiParam->numberOfMissedRxForAvalancheTrigger;
    pCfg->wlanElpHpSupport      = SoftGeminiParam->wlanElpHpSupport;
    pCfg->btAntiStarvationNumberOfCyclesWithinThePeriod = SoftGeminiParam->btAntiStarvationNumberOfCyclesWithinThePeriod;
    pCfg->btAntiStarvationPeriod = SoftGeminiParam->btAntiStarvationPeriod;
    pCfg->ackModeDuringBtLpInDualAnt = SoftGeminiParam->ackModeDuringBtLpInDualAnt;
    pCfg->allowPaSdToggleDuringBtActivityEnable = SoftGeminiParam->allowPaSdToggleDuringBtActivityEnable;
    pCfg->wakeUpTimeBeforeBeacon = SoftGeminiParam->wakeUpTimeBeforeBeacon;
	pCfg->hpdmMaxGuardTime = SoftGeminiParam->hpdmMaxGuardTime;
	pCfg->timeoutNextWlanPacket = SoftGeminiParam->timeoutNextWlanPacket;
	pCfg->sgAutoModeNoCts = SoftGeminiParam->sgAutoModeNoCts;
	pCfg->numOfBtHpRespectedReq = SoftGeminiParam->numOfBtHpRespectedReq;

	/* Convert from pure number to Index. '0' is any rate */
	if ( SoftGeminiParam->wlanRxMinRateToRespectBtHp )
	{
		pCfg->wlanRxMinConvertedRateToRespectBtHp = rateNumberToIndex((UINT8)SoftGeminiParam->wlanRxMinRateToRespectBtHp);

		if (pCfg->wlanRxMinConvertedRateToRespectBtHp == INVALID_RATE_INDEX)
		{
			WLAN_REPORT_ERROR(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
				("%s wlanRxMinRateToRespectBtHp from %d to 0 (any rate). \n",
				__FUNCTION__, SoftGeminiParam->wlanRxMinRateToRespectBtHp));

			pCfg->wlanRxMinConvertedRateToRespectBtHp = RATE_INDEX_1MBPS;
		}
	}
	else
	{
		pCfg->wlanRxMinConvertedRateToRespectBtHp = RATE_INDEX_1MBPS;
	}

	WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
		("%s wlanRxMinRateToRespectBtHp from %d to %d. \n",
		__FUNCTION__, SoftGeminiParam->wlanRxMinRateToRespectBtHp, pCfg->wlanRxMinConvertedRateToRespectBtHp));

    /* Send the configuration command */
    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
}

/****************************************************************************
 *                      whal_hwInfoElemSoftGeminiParamsGet()
 ****************************************************************************
 * DESCRIPTION: Get the BTH-WLAN co-exsistance parameters from the Fw   
 *
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemSoftGeminiParamsGet (HwMboxConfig_T* pHwMboxConfig, void *CB_Func, TI_HANDLE CB_handle, void* CB_Buf)
{
    ACXBluetoothWlanCoParamsStruct          AcxElm_BluetoothWlanEnable;
    ACXBluetoothWlanCoParamsStruct* pCfg = &AcxElm_BluetoothWlanEnable;

    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
        ("whal_hwInfoElemSoftGeminiParamsGet. \n"));

    /* Set information element header */
    pCfg->EleHdr.id = ACX_SG_CFG;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Send the configuration command */
    return CmdQueue_CmdInterrogateWithCb (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(ACXBluetoothWlanCoParamsStruct), CB_Func, CB_handle, CB_Buf);
}

/****************************************************************************
 *                      whal_hwInfoElemMemoryMapSet ()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate MemoryMap information element
 *
 * INPUTS:
 *      AcxElm_MemoryMap_T *apMap   pointer to the memory map structure
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemMemoryMapSet 
(
    HwMboxConfig_T *pHwMboxConfig, 
    MemoryMap_t    *apMap
)
{
    int rc;  
    MemoryMap_t SwapMap;
    UINT32 *pSwap, *pOrig, i;

    /* Set information element header */
    SwapMap.EleHdr.id  = ACX_MEM_MAP;
    SwapMap.EleHdr.len = sizeof(SwapMap) - sizeof(EleHdrStruct);

    /* Solve endian problem (all fields are 32 bit) */
    pOrig = (UINT32* )&apMap->codeStart;
    pSwap = (UINT32* )&SwapMap.codeStart;
    for (i = 0; i < MEM_MAP_NUM_FIELDS; i++)
        pSwap[i] = ENDIAN_HANDLE_LONG(pOrig[i]);

    /* Send the configuration command */
    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, &SwapMap, sizeof(SwapMap));

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemMemoryMapGet ()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate MemoryMap information element
 *
 * INPUTS:
 *      AcxElm_MemoryMap_T *apMap   pointer to the memory map structure
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemMemoryMapGet 
(
    HwMboxConfig_T *pHwMboxConfig, 
    MemoryMap_t    *apMap,
    void           *fCb,
    TI_HANDLE       hCb
)
{
    int rc;
   
    /* Set information element header */
    apMap->EleHdr.id  = ACX_MEM_MAP;
    apMap->EleHdr.len = sizeof(*apMap) - sizeof(EleHdrStruct);

    /* Send the interrogation command */
    if ((rc = CmdQueue_CmdInterrogateWithCb (pHwMboxConfig->hCmdMboxQueue, apMap, sizeof(*apMap), fCb, hCb, apMap)) == OK)
    {
    }

    return rc;
}


static int whal_hwInfoElemMemoryMapPrintCb (TI_HANDLE hHwMboxConfig, TI_STATUS status, void *pData)
{
#ifdef TI_DBG
    HwMboxConfig_T *pHwMboxConfig = (HwMboxConfig_T *)hHwMboxConfig;
    MemoryMap_t    *pMemMap = &pHwMboxConfig->MemMap;

    /* Print the memory map */
    WLAN_OS_REPORT (("whal_hwInfoElemMemoryMapPrint:\n"));
    WLAN_OS_REPORT (("\tCode  (0x%08x, 0x%08x)\n\tWep  (0x%08x, 0x%08x)\n\tTmpl (0x%08x, 0x%08x)\n "
                    "\tQueue (0x%08x, 0x%08x)\n\tPool (0x%08x, 0x%08x)\n\tTraceBuffer (A = 0x%08x, B = 0x%08x)\n",
                    pMemMap->codeStart, pMemMap->codeEnd,
                    pMemMap->wepDefaultKeyStart, pMemMap->wepDefaultKeyEnd,
                    pMemMap->packetTemplateStart, pMemMap->packetTemplateEnd,
                    pMemMap->queueMemoryStart, pMemMap->queueMemoryEnd,
                    pMemMap->packetMemoryPoolStart, pMemMap->packetMemoryPoolEnd,
                    pMemMap->debugBuffer1Start, pMemMap->debugBuffer2Start));
#endif /* TI_DBG */
    return OK;
}


/****************************************************************************
 *                      whal_hwInfoElemMemoryMapPrint ()
 ****************************************************************************
 * DESCRIPTION: Print some of the MemoryMap information element fields
 *
 * INPUTS:
 *          HwMboxConfig_T* pHwMboxConfig pointer to the acx mailbox
 *
 * OUTPUT:  None
 *
 * RETURNS: None
 ****************************************************************************/
void whal_hwInfoElemMemoryMapPrint (HwMboxConfig_T* pHwMboxConfig)
{
    whal_hwInfoElemMemoryMapGet (pHwMboxConfig, 
                                 &pHwMboxConfig->MemMap, 
                                 (void *)whal_hwInfoElemMemoryMapPrintCb,
                                 (TI_HANDLE)pHwMboxConfig);
}


/****************************************************************************
 *                      whal_hwInfoElemConfigOptionsRead ()
 ****************************************************************************
 * DESCRIPTION: Read ConfigOption information element from the wlan hardware.
 *      This is a special case where the data is already in the mailbox
 *      after wlan hardware reset and no interrogate command should be sent.
 *      This read-only IE provides information stored in the adaptor’s
 *      non-volatile memory to host.
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 *
 * NOTE : This function is not in use and must be reexamined (especially the Fw-Driver API) 
 *
 ****************************************************************************/
int whal_hwInfoElemConfigOptionsRead (HwMboxConfig_T* pHwMboxConfig, void* pElm)
{   
    WLAN_OS_REPORT(("%s not implemented\n",__FUNCTION__));
    /* The ConfigOptions information element is ready after reset on the mailbox */
    return 0;
}


/****************************************************************************
 *                      whal_hwInfoElemAcxRevisionGet()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate the ACX revision (FW and HW version)
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAcxRevisionGet 
(
     HwMboxConfig_T *pHwMboxConfig, 
     void           *fCb,
     TI_HANDLE       hCb,
     void           *pCb
)
{
    ACXRevision_t   aElm;
    ACXRevision_t  *apElm = &aElm; 
    int rc;

    /* Set information element header */
    apElm->EleHdr.id  = ACX_FW_REV;
    apElm->EleHdr.len = sizeof(*apElm) - sizeof(EleHdrStruct);

    /* Send the command*/
    rc = CmdQueue_CmdInterrogateWithCb (pHwMboxConfig->hCmdMboxQueue, apElm, sizeof(*apElm), fCb, hCb, pCb);

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemWepDefaultKeyIdSet()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate The default Key Id
 *
 * INPUTS:
 *      UINT8* Key    The default key id to use
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemWepDefaultKeyIdSet (HwMboxConfig_T* pHwMboxConfig, UINT8* apKeyVal,
                                       void *CB_Func, TI_HANDLE CB_handle)
{
    int rc;
    dot11WEPDefaultKeyId_t  WlanElm_WepDefaultKeyId;
    dot11WEPDefaultKeyId_t  *pCfg = &WlanElm_WepDefaultKeyId;

    /* Set information element header*/
    pCfg->EleHdr.id = DOT11_DEFAULT_KEY;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Send the command */
    pCfg->DefaultKeyId = *apKeyVal;

    /* Send the configuration command */
    rc = CmdQueue_CmdConfigureWithCb (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg), CB_Func, CB_handle);

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemWepDefaultKeyIdGet()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate The default Key Id
 *
 * INPUTS:
 *      UINT8* Key    The default key id to use
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemWepDefaultKeyIdGet (HwMboxConfig_T* pHwMboxConfig, UINT8* apKeyVal,
                                       void *CB_Func, TI_HANDLE CB_handle)
{
    int rc = OK;
    dot11WEPDefaultKeyId_t  WlanElm_WepDefaultKeyId;
    dot11WEPDefaultKeyId_t  *pCfg = &WlanElm_WepDefaultKeyId;

    /* Set information element header */
    pCfg->EleHdr.id = DOT11_DEFAULT_KEY;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Not implemented */

  #if 0
    *apKeyVal = pCfg->DefaultKeyId;
  #endif

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemAidSet()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate the AID info element
 *
 * INPUTS:
 *      UINT16* apAidVal     The AID value
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAidSet (HwMboxConfig_T* pHwMboxConfig, UINT16* apAidVal)
{
    int rc;
    ACXAid_t    WlanElm_AID;
    ACXAid_t    *pCfg = &WlanElm_AID;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_AID;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Send the command */
    pCfg->Aid = ENDIAN_HANDLE_WORD(*apAidVal);

    /* Send the configuration command */
    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemAcxwakeUpConditionSet()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate the power management option
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAcxwakeUpConditionSet (HwMboxConfig_T* pHwMboxConfig,
                                          WakeUpCondition_t* pWlanElm_wakeUpCondition)
{
    int rc;
    WakeUpCondition_t* pCfg = pWlanElm_wakeUpCondition;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_WAKE_UP_CONDITIONS;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Send the command */
    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemAcxPMConfigSet()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate the power management option
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAcxPMConfigSet (HwMboxConfig_T* pHwMboxConfig,
                                   ACXConfigPM_t* pWlanElm_PowerMgmtOptions)
{
    int rc;
    ACXConfigPM_t* pCfg = pWlanElm_PowerMgmtOptions;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_PM_CFG;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Send the command */
    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemAcxSleepAuthoSet()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate the power management option
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAcxSleepAuthoSet (HwMboxConfig_T* pHwMboxConfig,
                                     ACXSleepAuth_t* pWlanElm_SleepAutho)
{
    int rc;
    ACXSleepAuth_t* pCfg = pWlanElm_SleepAutho;

    /* Set information element header*/
    pCfg->EleHdr.id = ACX_SLEEP_AUTH;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Send the command */
    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemAcxBcnBrcOptionsSet()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate the power management option
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAcxBcnBrcOptionsSet (HwMboxConfig_T* pHwMboxConfig,
                                        ACXBeaconAndBroadcastOptions_t* pWlanElm_BcnBrcOptions)
{
    int rc;
    ACXBeaconAndBroadcastOptions_t* pCfg = pWlanElm_BcnBrcOptions;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_BCN_DTIM_OPTIONS;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Send the command */
    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemAcxBcnBrcOptionsGet()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate the power management option
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAcxBcnBrcOptionsGet (HwMboxConfig_T* pHwMboxConfig,
                                        ACXBeaconAndBroadcastOptions_t* pWlanElm_BcnBrcOptions)
{
    ACXBeaconAndBroadcastOptions_t* pCfg = pWlanElm_BcnBrcOptions;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_BCN_DTIM_OPTIONS;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* The STA does not support interrogate action for this Ie */
    return NOK;
}


/****************************************************************************
 *                      whal_hwInfoElemFeatureConfigSet()
                                    ACXBeaconAndBroadcastOptions_t* pWlanElm_BcnBrcOptions,
 ****************************************************************************
 * DESCRIPTION: Configure the feature config info element
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int  whal_hwInfoElemFeatureConfigSet (HwMboxConfig_T* pHwMboxConfig,  UINT32 Options, UINT32 DataFlowOptions)
{
    ACXFeatureConfig_t  WlanElm_FeatureConfig;
    ACXFeatureConfig_t  *pCfg = &WlanElm_FeatureConfig;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_FEATURE_CFG;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Set fields */
    pCfg->Options = ENDIAN_HANDLE_LONG(Options);
    pCfg->dataflowOptions = ENDIAN_HANDLE_LONG(DataFlowOptions);

    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
        ("whal_hwInfoElemFeatureConfigSet: ## Option=0x%x, DFOption=0x%x\n", Options, DataFlowOptions));

    /* Send the configuration command */
    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
}


/****************************************************************************
 *                      whal_hwInfoElemAntennaDiversitySet ()
 ****************************************************************************
 * DESCRIPTION: Set antenna diversity parameters
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAntennaDiversitySet (HwMboxConfig_T* pHwMboxConfig, 
                                     whalCtrl_antennaDiversityOptions_t* pAntennaDiversityOptions,
                                     UINT32 antNum)
{
    AcxSetAntennaDiversityOptions_t ACXAntennaDiversityOptions;

    WLAN_REPORT_INFORMATION( pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
                             ("RX diversity enabled: %d TX diversity enabled:%d\n", 
                              pAntennaDiversityOptions->enableRxDiversity,
                              pAntennaDiversityOptions->enableTxDiversity) );
    WLAN_REPORT_INFORMATION( pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
                             ("Rx Antenna: %d TX antenna: %d\n",
                              pAntennaDiversityOptions->rxSelectedAntenna,
                              pAntennaDiversityOptions->txSelectedAntenna) );
    WLAN_REPORT_INFORMATION( pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
                             ("Share TX and RX antennas: %d\n",
                              pAntennaDiversityOptions->rxTxSharedAnts) );
    
    /* Set information element header */
    ACXAntennaDiversityOptions.EleHdr.id = ACX_ANTENNA_DIVERSITY_CFG;
    ACXAntennaDiversityOptions.EleHdr.len = sizeof( AcxSetAntennaDiversityOptions_t ) - 
                                                sizeof( EleHdrStruct );

    /* Set information element fields */
    ACXAntennaDiversityOptions.enableRxDiversity = pAntennaDiversityOptions->enableRxDiversity;
    ACXAntennaDiversityOptions.rxSelectedAntenna = pAntennaDiversityOptions->rxSelectedAntenna;
    ACXAntennaDiversityOptions.enableTxDiversity = pAntennaDiversityOptions->enableTxDiversity;
    ACXAntennaDiversityOptions.txSelectedAntenna = pAntennaDiversityOptions->txSelectedAntenna;
    ACXAntennaDiversityOptions.rxAntNum = antNum;
    ACXAntennaDiversityOptions.txAntNum = antNum;
    ACXAntennaDiversityOptions.rxTxSharedAnts = pAntennaDiversityOptions->rxTxSharedAnts;

    /* Send the command */
    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, 
                                  &ACXAntennaDiversityOptions,
                                  sizeof(AcxSetAntennaDiversityOptions_t));
}


/****************************************************************************
 *                      whal_hwInfoElemTxPowerSet ()
 ****************************************************************************
 * DESCRIPTION: Set the Tx power 
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemTxPowerSet (HwMboxConfig_T* pHwMboxConfig, UINT8 *TxPowerDbm)
{
    dot11CurrentTxPowerStruct  dot11CurrentTxPower;
    dot11CurrentTxPowerStruct  *pCfg = &dot11CurrentTxPower;
    int rc;

	WLAN_REPORT_INFORMATION( pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
		("%s power = %d\n", __FUNCTION__, *TxPowerDbm));

    /* Set information element header*/
    pCfg->EleHdr.id = DOT11_CUR_TX_PWR;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Send the command */
    pCfg->dot11CurrentTxPower = *TxPowerDbm;
    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemAcxStatisiticsSet ()
 ****************************************************************************
 * DESCRIPTION: Set the ACX statistics counters to zero.
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAcxStatisiticsSet (HwMboxConfig_T* pHwMboxConfig)
{
    ACXStatistics_t  acx;
    ACXStatistics_t  *pCfg = &acx;

    /* Set information element header */
    pCfg->EleHdr.id  = ACX_STATISTICS;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Send the config command */
    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
}


/****************************************************************************
 *                      whal_hwInfoElemAcxStatisiticsGet ()
 ****************************************************************************
 * DESCRIPTION: Get the ACX statistics that are required for basic measurement
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAcxStatisiticsGet (HwMboxConfig_T* pHwMboxConfig, acxStatisitcs_t *acxStatisitcs)
{
    ACXStatistics_t     acx;
    ACXStatistics_t     *pCfg = &acx;
    int rc = OK;

    /* Set information element header */
    pCfg->EleHdr.id  = ACX_STATISTICS;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Not implemented */

    /* Setting the output params */
  #if 0
    acxStatisitcs->FWpacketReceived = pCfg->isr.RxHeaders;
  #endif

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemAcxReadGwsiStatisiticsGet ()
 ****************************************************************************
 * DESCRIPTION: Get the ACX GWSI statistics 
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAcxReadGwsiStatisiticsGet (HwMboxConfig_T * pHwMboxConfig, 
                                              void * CB_Func, 
                                              TI_HANDLE CB_handle, 
                                              void * CB_Buf)
{
    ACXRoamingStatisticsTable_t acx;
    ACXRoamingStatisticsTable_t * pCfg = &acx;

    /* 
     * Set information element header
     */
    pCfg->EleHdr.id  = ACX_ROAMING_STATISTICS_TBL;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* 
     * Send the interrogation command
     */
    return CmdQueue_CmdInterrogateWithCb (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg), CB_Func, CB_handle, CB_Buf);
}


/****************************************************************************
 *                      whal_hwInfoElemAcxReadGwsiCountersGet ()
 ****************************************************************************
 * DESCRIPTION: Get the ACX GWSI counters 
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAcxReadGwsiCountersGet (HwMboxConfig_T * pHwMboxConfig, 
                                           void * CB_Func, 
                                           TI_HANDLE CB_handle, 
                                           void * CB_Buf)
{
    ACXErrorCounters_t acx;
    ACXErrorCounters_t * pCfg = &acx;

    /* 
     * Set information element header
     */
    pCfg->EleHdr.id  = ACX_ERROR_CNT;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* 
     * Send the interrogation command
     */
    return CmdQueue_CmdInterrogateWithCb (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg), CB_Func, CB_handle, CB_Buf);
}


/****************************************************************************
 *                      whal_hwInfoElemMediumOccupancyGet ()
 ****************************************************************************
 * DESCRIPTION: Get the Medium Occupancy.
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemMediumOccupancyGet (HwMboxConfig_T* pHwMboxConfig,
                                       interogateCmdCBParams_t  mediumUsageCBParams)
{
    ACXMediumUsage_t    medium;
    ACXMediumUsage_t    *pCfg = &medium;

    /* Set information element header */
    pCfg->EleHdr.id  = ACX_MEDIUM_USAGE;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Send the interrogation command */
    return CmdQueue_CmdInterrogateWithCb (pHwMboxConfig->hCmdMboxQueue, pCfg,
                                          sizeof(*pCfg),
                                          mediumUsageCBParams.CB_Func,
                                          mediumUsageCBParams.CB_handle,
                                          mediumUsageCBParams.CB_buf);
}


/****************************************************************************
 *                      whal_hwInfoElemTfsDtimGet ()
 ****************************************************************************
 * DESCRIPTION: Get the Tsf and Dtim counter from Fw
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemTfsDtimGet (HwMboxConfig_T* pHwMboxConfig,
                               interogateCmdCBParams_t  mediumUsageCBParams)
{
    ACX_fwTSFInformation_t    fwTsfDtimMib;
    ACX_fwTSFInformation_t    *pCfg = &fwTsfDtimMib;
    int sendOp = 0;

    /* Set information element header*/
    pCfg->EleHdr.id  = ACX_TSF_INFO;
    pCfg->EleHdr.len = sizeof(ACX_fwTSFInformation_t) - sizeof(EleHdrStruct);

    /* Send the interrogation command*/
    sendOp = CmdQueue_CmdInterrogateWithCb (pHwMboxConfig->hCmdMboxQueue, pCfg,
                                               sizeof(*pCfg), 
                                               mediumUsageCBParams.CB_Func,
                                               mediumUsageCBParams.CB_handle,
                                               mediumUsageCBParams.CB_buf);
    if (0 == sendOp)
    {
        WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  ("whal_hwInfoElemTfsDtimGet ACX_fwTSFInformation command sent with to FW wait for results\n"));
    }
    else
    {
        WLAN_REPORT_ERROR(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  ("whal_hwInfoElemTfsDtimGet ACX_fwTSFInformation command not sent , FAILURE =%d\n" ,sendOp) );
    }

    return sendOp;
}


static int  whal_hwInfoElemStatisticsReadCB (HwMboxConfig_T* pHwMboxConfig,UINT16 MboxStatus, ACXStatistics_t* pElem);


/****************************************************************************
 *                      whal_hwInfoElemStatisticsPrint ()
 ****************************************************************************
 * DESCRIPTION: Print the statistics from the input IE statistics
 *
 * INPUTS:
 *          ACXStatisticsStruct* pElem  The Statistics information element
 *                                      to be printed
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemStatisticsPrint (HwMboxConfig_T *pHwMboxConfig)
{
    int rc;

    /* Set information element header */
    pHwMboxConfig->pAcxStatistic.EleHdr.id  = ACX_STATISTICS;
    pHwMboxConfig->pAcxStatistic.EleHdr.len = sizeof(pHwMboxConfig->pAcxStatistic) - sizeof(EleHdrStruct);

    /* Send the interrogation command */
    if ((rc = CmdQueue_CmdInterrogateWithCb (pHwMboxConfig->hCmdMboxQueue, 
                                             &pHwMboxConfig->pAcxStatistic, 
                                             sizeof(pHwMboxConfig->pAcxStatistic),
                                             (void *)whal_hwInfoElemStatisticsReadCB,
                                             pHwMboxConfig,
                                             &pHwMboxConfig->pAcxStatistic)) != OK)
        return rc;

    return OK;
}


/****************************************************************************
 *                      whal_hwInfoElemStatisticsReadCB ()
 ****************************************************************************
 * DESCRIPTION: Interrogate Statistics from the wlan hardware
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
static int whal_hwInfoElemStatisticsReadCB (HwMboxConfig_T* pHwMboxConfig, UINT16 MboxStatus, ACXStatistics_t* pElem)
{
    if (MboxStatus != OK)
        return NOK;

    /* Solve endian problem*/
    /* Isr */
    pElem->isr.ACXRxProcs       = ENDIAN_HANDLE_LONG(pElem->isr.ACXRxProcs);
    pElem->isr.ACXTxProcs       = ENDIAN_HANDLE_LONG(pElem->isr.ACXTxProcs);
    pElem->isr.Cmd_Cmplt        = ENDIAN_HANDLE_LONG(pElem->isr.Cmd_Cmplt);
    pElem->isr.FIQs             = ENDIAN_HANDLE_LONG(pElem->isr.FIQs);
    pElem->isr.RxHeaders        = ENDIAN_HANDLE_LONG(pElem->isr.RxHeaders);
    pElem->isr.RxCompletes      = ENDIAN_HANDLE_LONG(pElem->isr.RxCompletes);
    pElem->isr.RxMemOverflow    = ENDIAN_HANDLE_LONG(pElem->isr.RxMemOverflow);
    pElem->isr.RxRdys           = ENDIAN_HANDLE_LONG(pElem->isr.RxRdys);
    pElem->isr.IRQs             = ENDIAN_HANDLE_LONG(pElem->isr.IRQs);
    pElem->isr.DecryptDone      = ENDIAN_HANDLE_LONG(pElem->isr.DecryptDone);
    pElem->isr.DMA0Done         = ENDIAN_HANDLE_LONG(pElem->isr.DMA0Done);
    pElem->isr.DMA1Done         = ENDIAN_HANDLE_LONG(pElem->isr.DMA1Done);
    pElem->isr.ACXTxExchComplete= ENDIAN_HANDLE_LONG(pElem->isr.ACXTxExchComplete);
    pElem->isr.ACXCommands      = ENDIAN_HANDLE_LONG(pElem->isr.ACXCommands);
    pElem->isr.HwPMModeChanges  = ENDIAN_HANDLE_LONG(pElem->isr.HwPMModeChanges);
    pElem->isr.HostAcknowledges = ENDIAN_HANDLE_LONG(pElem->isr.HostAcknowledges);
    pElem->isr.PCI_PM           = ENDIAN_HANDLE_LONG(pElem->isr.PCI_PM);
    pElem->isr.ACMWakeups       = ENDIAN_HANDLE_LONG(pElem->isr.ACMWakeups);

    /* Rx */
    pElem->rx.RxDroppedFrame    = ENDIAN_HANDLE_LONG(pElem->rx.RxDroppedFrame);
    pElem->rx.RxFcsErr          = ENDIAN_HANDLE_LONG(pElem->rx.RxFcsErr);
    pElem->rx.RxHdrOverflow     = ENDIAN_HANDLE_LONG(pElem->rx.RxHdrOverflow);
    pElem->rx.RxHWStuck         = ENDIAN_HANDLE_LONG(pElem->rx.RxHWStuck);
    pElem->rx.RxOutOfMem        = ENDIAN_HANDLE_LONG(pElem->rx.RxOutOfMem);
    pElem->rx.RxXfrHintTrig     = ENDIAN_HANDLE_LONG(pElem->rx.RxXfrHintTrig);
    pElem->rx.RxResetCounter    = ENDIAN_HANDLE_LONG(pElem->rx.RxResetCounter);

    /* Tx */
    pElem->tx.TxInternalDescOverflow = ENDIAN_HANDLE_LONG(pElem->tx.TxInternalDescOverflow);

    /* Dma */
    pElem->dma.RxDMARequested   = ENDIAN_HANDLE_LONG(pElem->dma.RxDMARequested);
    pElem->dma.RxDMAErrors      = ENDIAN_HANDLE_LONG(pElem->dma.RxDMAErrors);
    pElem->dma.TxDMARequested   = ENDIAN_HANDLE_LONG(pElem->dma.TxDMARequested);
    pElem->dma.TxDMAErrors      = ENDIAN_HANDLE_LONG(pElem->dma.TxDMAErrors);

    /* Wep */
    pElem->wep.WepAddrKeyCount      = ENDIAN_HANDLE_LONG(pElem->wep.WepAddrKeyCount);
    pElem->wep.WepDecryptFail       = ENDIAN_HANDLE_LONG(pElem->wep.WepDecryptFail);
    pElem->wep.WepDefaultKeyCount   = ENDIAN_HANDLE_LONG(pElem->wep.WepDefaultKeyCount);
    pElem->wep.WepKeyNotFound       = ENDIAN_HANDLE_LONG(pElem->wep.WepKeyNotFound);

    /* PS */
    pElem->pwr.PSEnterCnt           = ENDIAN_HANDLE_LONG(pElem->pwr.PSEnterCnt);
    pElem->pwr.ELPEnterCnt          = ENDIAN_HANDLE_LONG(pElem->pwr.ELPEnterCnt);
    pElem->pwr.MissingBcnsCnt       = ENDIAN_HANDLE_LONG(pElem->pwr.MissingBcnsCnt);
    pElem->pwr.WakeOnHostCnt        = ENDIAN_HANDLE_LONG(pElem->pwr.WakeOnHostCnt);
    pElem->pwr.WakeOnTimerExpCnt    = ENDIAN_HANDLE_LONG(pElem->pwr.WakeOnTimerExpCnt);
    pElem->pwr.TxWithPSCnt          = ENDIAN_HANDLE_LONG(pElem->pwr.TxWithPSCnt);
    pElem->pwr.TxWithoutPSCnt       = ENDIAN_HANDLE_LONG(pElem->pwr.TxWithoutPSCnt);
    pElem->pwr.RcvdBeaconsCnt       = ENDIAN_HANDLE_LONG(pElem->pwr.RcvdBeaconsCnt);
    pElem->pwr.PowerSaveOffCnt      = ENDIAN_HANDLE_LONG(pElem->pwr.PowerSaveOffCnt);
    pElem->pwr.EnablePSCnt          = ENDIAN_HANDLE_LONG(pElem->pwr.EnablePSCnt);
    pElem->pwr.DisablePSCnt         = ENDIAN_HANDLE_LONG(pElem->pwr.DisablePSCnt);
    pElem->pwr.FixTsfPSCnt          = ENDIAN_HANDLE_LONG(pElem->pwr.FixTsfPSCnt);
    pElem->pwr.ContMissBcnsSpread[0]= ENDIAN_HANDLE_LONG(pElem->pwr.ContMissBcnsSpread[0]);
    pElem->pwr.ContMissBcnsSpread[1]= ENDIAN_HANDLE_LONG(pElem->pwr.ContMissBcnsSpread[1]);
    pElem->pwr.ContMissBcnsSpread[2]= ENDIAN_HANDLE_LONG(pElem->pwr.ContMissBcnsSpread[2]);
    pElem->pwr.ContMissBcnsSpread[3]= ENDIAN_HANDLE_LONG(pElem->pwr.ContMissBcnsSpread[3]);
    pElem->pwr.ContMissBcnsSpread[4]= ENDIAN_HANDLE_LONG(pElem->pwr.ContMissBcnsSpread[4]);
    pElem->pwr.ContMissBcnsSpread[5]= ENDIAN_HANDLE_LONG(pElem->pwr.ContMissBcnsSpread[5]);
    pElem->pwr.ContMissBcnsSpread[6]= ENDIAN_HANDLE_LONG(pElem->pwr.ContMissBcnsSpread[6]);
    pElem->pwr.ContMissBcnsSpread[7]= ENDIAN_HANDLE_LONG(pElem->pwr.ContMissBcnsSpread[7]);
    pElem->pwr.ContMissBcnsSpread[8]= ENDIAN_HANDLE_LONG(pElem->pwr.ContMissBcnsSpread[8]);
    pElem->pwr.ContMissBcnsSpread[9]= ENDIAN_HANDLE_LONG(pElem->pwr.ContMissBcnsSpread[9]);

    pElem->ps.psPollTimeOuts        = ENDIAN_HANDLE_LONG(pElem->ps.psPollTimeOuts);
    pElem->ps.upsdTimeOuts          = ENDIAN_HANDLE_LONG(pElem->ps.upsdTimeOuts);
    pElem->ps.upsdMaxSPTime         = ENDIAN_HANDLE_LONG(pElem->ps.upsdMaxSPTime);
    pElem->ps.upsdMaxAPturn         = ENDIAN_HANDLE_LONG(pElem->ps.upsdMaxAPturn); 
    pElem->ps.psPollMaxAPturn       = ENDIAN_HANDLE_LONG(pElem->ps.psPollMaxAPturn);
    pElem->ps.psPollUtilization     = ENDIAN_HANDLE_LONG(pElem->ps.psPollUtilization);
    pElem->ps.upsdUtilization       = ENDIAN_HANDLE_LONG(pElem->ps.upsdUtilization);

    /* Isr */
    WLAN_OS_REPORT(("------  Isr statistics  -------------------\n"));
    WLAN_OS_REPORT(("ACXRxProcs  = %8d\n", pElem->isr.ACXRxProcs));
    WLAN_OS_REPORT(("RxHeaders   = %8d, RxCompletes       = %8d\n", pElem->isr.RxHeaders, pElem->isr.RxCompletes));
    WLAN_OS_REPORT(("RxRdys      = %8d, RxMemOverflow     = %8d\n", pElem->isr.RxRdys, pElem->isr.RxMemOverflow));
    WLAN_OS_REPORT(("ACXTxProcs  = %8d, ACXTxExchComplete = %8d\n", pElem->isr.ACXTxProcs, pElem->isr.ACXTxExchComplete));
    WLAN_OS_REPORT(("DecryptDone       = %8d\n", pElem->isr.DecryptDone));
    WLAN_OS_REPORT(("HwPMModeChanges   = %8d\n", pElem->isr.HwPMModeChanges));
    WLAN_OS_REPORT(("HostAcknowledges  = %8d\n", pElem->isr.HostAcknowledges));
    WLAN_OS_REPORT(("PCI_PM            = %8d\n", pElem->isr.PCI_PM));
    WLAN_OS_REPORT(("ACMWakeups        = %8d\n", pElem->isr.ACMWakeups));
    WLAN_OS_REPORT(("LowRSSI           = %8d\n", pElem->isr.LowRssi));
    WLAN_OS_REPORT(("ACXCommands = %8d, Cmd_Cmplt= %8d\n", pElem->isr.ACXCommands, pElem->isr.Cmd_Cmplt));
    WLAN_OS_REPORT(("DMA0Done    = %8d, DMA1Done = %8d\n", pElem->isr.DMA0Done, pElem->isr.DMA1Done));
    WLAN_OS_REPORT(("IRQs = %8d, FIQs = %8d\n", pElem->isr.IRQs, pElem->isr.FIQs));

    /* Rx */
    WLAN_OS_REPORT(("------  Rx  statistics  -------------------\n"));
    WLAN_OS_REPORT(("RxDroppedFrame    = %d\n", pElem->rx.RxDroppedFrame));
    WLAN_OS_REPORT(("RxFcsErr      = %d\n", pElem->rx.RxFcsErr));
    WLAN_OS_REPORT(("RxHdrOverflow     = %d\n", pElem->rx.RxHdrOverflow));
    WLAN_OS_REPORT(("RxHWStuck         = %d\n", pElem->rx.RxHWStuck));
    WLAN_OS_REPORT(("RxOutOfMem        = %d\n", pElem->rx.RxOutOfMem));
    WLAN_OS_REPORT(("RxXfrHintTrig     = %d\n", pElem->rx.RxXfrHintTrig));
    WLAN_OS_REPORT(("RxResetCounter    = %d\n", pElem->rx.RxResetCounter));

    /* Tx */
    WLAN_OS_REPORT(("------  Tx  statistics  -------------------\n"));
    WLAN_OS_REPORT(("TxInDescOverflow  = %d\n", pElem->tx.TxInternalDescOverflow));

    /* Dma */
    WLAN_REPORT_REPLY(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("------  Dma  statistics  -------------------\n"));
    WLAN_REPORT_REPLY(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("RxDMARequested  = %d\n", pElem->dma.RxDMARequested));
    WLAN_REPORT_REPLY(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("RxDMAErrors  = %d\n", pElem->dma.RxDMAErrors));
    WLAN_REPORT_REPLY(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("TxDMARequested  = %d\n", pElem->dma.TxDMARequested));
    WLAN_REPORT_REPLY(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("TxDMAErrors  = %d\n", pElem->dma.TxDMAErrors));

    /* Wep */
    WLAN_OS_REPORT(("------  Wep statistics  -------------------\n"));
    WLAN_OS_REPORT(("WepDefaultKeyCount= %d\n", pElem->wep.WepDefaultKeyCount));
    WLAN_OS_REPORT(("WepAddrKeyCount   = %d\n", pElem->wep.WepAddrKeyCount));
    WLAN_OS_REPORT(("WepDecryptFail    = %d\n", pElem->wep.WepDecryptFail));
    WLAN_OS_REPORT(("WepKeyNotFound    = %d\n", pElem->wep.WepKeyNotFound));

    /* AES statistics */
    WLAN_OS_REPORT(("------------  AES Statistics !!!!  ---------------\n"));
    WLAN_OS_REPORT(("Aes Encryption Failure     = %8d, Aes Decryption Failure    = %8d\n", 
                      pElem->aes.AesEncryptFail, pElem->aes.AesDecryptFail));

    WLAN_OS_REPORT(("Aes Encrypted Packets      = %8d, Aes Decrypted Packets     = %8d\n", 
               pElem->aes.AesEncryptPackets, pElem->aes.AesDecryptPackets));

    WLAN_OS_REPORT(("Aes Encryption Interrupt   = %8d, Aes Decrryption Interrupt = %8d\n\n",
              pElem->aes.AesEncryptInterrupt, pElem->aes.AesDecryptInterrupt));

    /* events */
    WLAN_OS_REPORT(("------  Events  -------------------\n"));
    WLAN_OS_REPORT(("Heartbeat    = %d\n", pElem->event.heartbeat));

    WLAN_OS_REPORT(("Calibration    = %d\n", pElem->event.calibration));
    WLAN_OS_REPORT(("rxMismatch    = %d\n", pElem->event.rxMismatch));
    WLAN_OS_REPORT(("rxMemEmpty    = %d\n", pElem->event.rxMemEmpty));
    WLAN_OS_REPORT(("rxPool    = %d\n", pElem->event.rxPool));
    WLAN_OS_REPORT(("oomLate  = %d\n", pElem->event.oomLate));
    WLAN_OS_REPORT(("phyTransmitError    = %d\n", pElem->event.phyTransmitError));
    WLAN_OS_REPORT(("txStuck    = %d\n", pElem->event.txStuck));

    /* AES statistics */
    WLAN_REPORT_REPLY(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("------------  AES Statistics !!!!  ---------------\n"));
    WLAN_REPORT_REPLY(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("Aes Encryption Failure     = %8d, Aes Decryption Failure     = %8d\n", 
                      pElem->aes.AesEncryptFail, pElem->aes.AesDecryptFail));

    WLAN_REPORT_REPLY(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("Aes Encrypted Packets      = %8d, Aes Decrypted Packets     = %8d\n", 
               pElem->aes.AesEncryptPackets, pElem->aes.AesDecryptPackets));

    WLAN_REPORT_REPLY(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("Aes Encryption Interrupt   = %8d, Aes Decrryption Interrupt   = %8d\n", 
              pElem->aes.AesEncryptInterrupt, pElem->aes.AesDecryptInterrupt));
   /* PsPoll/Upsd */ 
    WLAN_OS_REPORT(("----------- PsPoll / Upsd -----------\n"));
    WLAN_OS_REPORT(("psPollTimeOuts     = %d\n",pElem->ps.psPollTimeOuts));
    WLAN_OS_REPORT(("upsdTimeOuts       = %d\n",pElem->ps.upsdTimeOuts));
    WLAN_OS_REPORT(("upsdMaxSPTime      = %d\n",pElem->ps.upsdMaxSPTime));
    WLAN_OS_REPORT(("upsdMaxAPturn      = %d\n",pElem->ps.upsdMaxAPturn));
    WLAN_OS_REPORT(("psPollMaxAPturn    = %d\n",pElem->ps.psPollMaxAPturn));
    WLAN_OS_REPORT(("psPollUtilization  = %d\n",pElem->ps.psPollUtilization));
    WLAN_OS_REPORT(("upsdUtilization    = %d\n",pElem->ps.upsdUtilization));

    /* Power Save Counters */
    WLAN_OS_REPORT(("------  Power management  ----------\n"));
    WLAN_OS_REPORT(("PSEnterCnt    = %d\n", pElem->pwr.PSEnterCnt));
    WLAN_OS_REPORT(("ELPEnterCnt    = %d\n", pElem->pwr.ELPEnterCnt));
    if(pElem->pwr.RcvdBeaconsCnt != 0)
    {
        WLAN_OS_REPORT(("MissingBcnsCnt    = %d (percentage <= %d) \n", 
                pElem->pwr.MissingBcnsCnt,
                ((pElem->pwr.MissingBcnsCnt * 100) / (pElem->pwr.RcvdBeaconsCnt + pElem->pwr.MissingBcnsCnt)) ));
    }
    else
    {
        WLAN_OS_REPORT(("MissingBcnsCnt    = %d (percentage = 0) \n", pElem->pwr.MissingBcnsCnt));
    }
    WLAN_OS_REPORT(("WakeOnHostCnt    = %d\n", pElem->pwr.WakeOnHostCnt));
    WLAN_OS_REPORT(("WakeOnTimerExpCnt    = %d\n", pElem->pwr.WakeOnTimerExpCnt));
    WLAN_OS_REPORT(("TxWithPSCnt    = %d\n", pElem->pwr.TxWithPSCnt));
    WLAN_OS_REPORT(("TxWithoutPSCnt    = %d\n", pElem->pwr.TxWithoutPSCnt));
    WLAN_OS_REPORT(("RcvdBeaconsCnt    = %d\n", pElem->pwr.RcvdBeaconsCnt));
    WLAN_OS_REPORT(("PowerSaveOffCnt    = %d\n", pElem->pwr.PowerSaveOffCnt));
    WLAN_OS_REPORT(("EnablePS    = %d\n", pElem->pwr.EnablePSCnt));
    WLAN_OS_REPORT(("DisablePS    = %d\n", pElem->pwr.DisablePSCnt));
    WLAN_OS_REPORT(("FixTsfPSCnt    = %d\n\n", pElem->pwr.FixTsfPSCnt));
    WLAN_OS_REPORT(("Single Missed Beacon           = %d\n", (pElem->pwr.ContMissBcnsSpread[0] & 0xFFFF)));
    WLAN_OS_REPORT(("2 Continuous Missed Beacons    = %d\n", (pElem->pwr.ContMissBcnsSpread[1] & 0xFFFF)));
    WLAN_OS_REPORT(("3 Continuous Missed Beacons    = %d\n", (pElem->pwr.ContMissBcnsSpread[2] & 0xFFFF)));
    WLAN_OS_REPORT(("4 Continuous Missed Beacons    = %d\n", (pElem->pwr.ContMissBcnsSpread[3] & 0xFFFF)));
    WLAN_OS_REPORT(("5 Continuous Missed Beacons    = %d\n", (pElem->pwr.ContMissBcnsSpread[4] & 0xFFFF)));
    WLAN_OS_REPORT(("6 Continuous Missed Beacons    = %d\n", (pElem->pwr.ContMissBcnsSpread[5] & 0xFFFF)));
    WLAN_OS_REPORT(("7 Continuous Missed Beacons    = %d\n", (pElem->pwr.ContMissBcnsSpread[6] & 0xFFFF)));
    WLAN_OS_REPORT(("8 Continuous Missed Beacons    = %d\n", (pElem->pwr.ContMissBcnsSpread[7] & 0xFFFF)));
    WLAN_OS_REPORT(("9 Continuous Missed Beacons    = %d\n", (pElem->pwr.ContMissBcnsSpread[8] & 0xFFFF)));
    WLAN_OS_REPORT((">=10 Continuous Missed Beacons = %d\n\n", (pElem->pwr.ContMissBcnsSpread[9] & 0xFFFF)));

    WLAN_OS_REPORT(("RcvdAwakeBeaconsCnt    = %d\n", pElem->pwr.RcvdAwakeBeaconsCnt));
    WLAN_OS_REPORT(("Single Missed Beacon        [Awake] = %d\n", (pElem->pwr.ContMissBcnsSpread[0] >> 16)));
    WLAN_OS_REPORT(("2 Continuous Missed Beacons [Awake] = %d\n", (pElem->pwr.ContMissBcnsSpread[1] >> 16)));
    WLAN_OS_REPORT(("3 Continuous Missed Beacons [Awake] = %d\n", (pElem->pwr.ContMissBcnsSpread[2] >> 16)));
    WLAN_OS_REPORT(("4 Continuous Missed Beacons [Awake] = %d\n", (pElem->pwr.ContMissBcnsSpread[3] >> 16)));
    WLAN_OS_REPORT(("5 Continuous Missed Beacons [Awake] = %d\n", (pElem->pwr.ContMissBcnsSpread[4] >> 16)));
    WLAN_OS_REPORT(("6 Continuous Missed Beacons [Awake] = %d\n", (pElem->pwr.ContMissBcnsSpread[5] >> 16)));
    WLAN_OS_REPORT(("7 Continuous Missed Beacons [Awake] = %d\n", (pElem->pwr.ContMissBcnsSpread[6] >> 16)));
    WLAN_OS_REPORT(("8 Continuous Missed Beacons [Awake] = %d\n", (pElem->pwr.ContMissBcnsSpread[7] >> 16)));
    WLAN_OS_REPORT(("9 Continuous Missed Beacons [Awake] = %d\n", (pElem->pwr.ContMissBcnsSpread[8] >> 16)));
    WLAN_OS_REPORT((">=10 Continuous Missed Beacons [Awake] = %d\n", (pElem->pwr.ContMissBcnsSpread[9] >> 16)));

    return OK;
}


/****************************************************************************
 *                      whal_hwInfoElemFcsErrorCntGet ()
 ****************************************************************************
 * DESCRIPTION: Interrogate Fcs error counter from the ACX
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 *
 * NOTE: This field (at the ACX) is automatically cleared when interrogated
 ****************************************************************************/
int whal_hwInfoElemFcsErrorCntGet (HwMboxConfig_T* pHwMboxConfig, UINT32* pFcsErrCnt)
{
    ACXFCSErrorCount_t  WlanElm_FcsError;
    ACXFCSErrorCount_t  *pCfg = &WlanElm_FcsError;

    /* Set information element header*/
    pCfg->EleHdr.id = ACX_FCS_ERROR_CNT;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /* Send the interrogation command*/

    /* Not implemented */
    {
        /* Solve endian problem*/
        /* *pFcsErrCnt = ENDIAN_HANDLE_LONG(pCfg->FCSErrorCount); */

    }

    return NOK;
}


/****************************************************************************
 *                      whal_hwInfoElemMiscTableSet ()
 ****************************************************************************
 * DESCRIPTION: Interrogate Statistics from the wlan hardware
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemMiscTableSet
(
    HwMboxConfig_T *pHwMboxConfig, 
    ACXMisc_t      *pCfg
)
{
    int rc = OK;
    ACXMisc_t  aCfg;

    aCfg.EleHdr.id  = ACX_MISC_CFG;
    aCfg.EleHdr.len = sizeof(ACXMisc_t) - sizeof(EleHdrStruct);

    /* Solve endian problem */
    aCfg.txActivityLed = ENDIAN_HANDLE_WORD(pCfg->txActivityLed);
    aCfg.fwInitLed     = ENDIAN_HANDLE_WORD(pCfg->fwInitLed);
    aCfg.diagnosticLed = ENDIAN_HANDLE_WORD(pCfg->diagnosticLed);

    /* Send the interrogation command */
    if ((rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, &aCfg, sizeof(aCfg))) != OK)
    {
    }

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemMiscTableGet ()
 ****************************************************************************
 * DESCRIPTION: Interrogate Statistics from the wlan hardware
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemMiscTableGet
(
    HwMboxConfig_T *pHwMboxConfig, 
    ACXMisc_t      *pCfg,
    void           *fCb,
    TI_HANDLE       hCb
)
{
    int rc = OK;

    /* Set information element header */
    pCfg->EleHdr.id  = ACX_MISC_CFG;
    pCfg->EleHdr.len = sizeof(ACXMisc_t) - sizeof(EleHdrStruct);

    /* Send the interrogation command */
    if ((rc = CmdQueue_CmdInterrogateWithCb (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(ACXMisc_t), fCb, hCb, pCfg)) != OK)
    {
    }

    return rc;
}


#if 0
/****************************************************************************
 *                      whal_hwInfoElemTxTrafficCategorySet()
 ****************************************************************************
 * DESCRIPTION: Write the Traffic  configuration (For Quality Of Service)
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemTxTrafficCategorySet (HwMboxConfig_T* pHwMboxConfig, whaCtrl_acTrafficParams_t* pTconfParams)
{
    TrafficCategoryCfgType    TrafficCategoryCfg;
    TrafficCategoryCfgType*   pCfg = &TrafficCategoryCfg;

    os_memoryZero( pHwMboxConfig->hOs, (void *)pCfg, sizeof(*pCfg) );
    
    /*
     * Set information element header
     * ==============================
     */
    pCfg->EleHdr.id = ACX_TID_CFG;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /*
     * Set information element Data
     * ==============================
     */
    pCfg->acId = pTconfParams->acId;
    pCfg->aifsn = pTconfParams->aifsn;
    pCfg->cwMax = pTconfParams->cwMax;
    pCfg->cwMin = pTconfParams->cwMin;
    pCfg->longRetryLimit = pTconfParams->longRetryLimit;
    pCfg->shortRetryLimit = pTconfParams->shortRetryLimit;
    pCfg->txopLimit = pTconfParams->txopLimit;
    pCfg->rxTimeout = pTconfParams->rxTimeout;
    pCfg->deliveryTriggerType = pTconfParams->PsParameters;

    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  ("whal_hwInfoElemTxTrafficCategory : \n"));
    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  ("pCfg->cfg.acID= %d\n",pCfg->acId));
    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  ("pCfg->cfg.edcfCfg.aifsn= %d\n",pCfg->aifsn));
    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  ("pCfg->cfg.edcfCfg.cwMax= %d\n",pCfg->cwMax));
    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  ("pCfg->cfg.edcfCfg.cwMin= %d\n",pCfg->cwMin));
    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  ("pCfg->cfg.edcfCfg.longRetryLimit= %d\n",pCfg->longRetryLimit));
    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  ("pCfg->cfg.edcfCfg.shortRetryLimit= %d\n",pCfg->shortRetryLimit));
    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  ("pCfg->cfg.edcfCfg.rxTimeout= %d\n", pCfg->rxTimeout));
    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  ("pCfg->cfg.edcfCfg.txopLimit= %d\n", pCfg->txopLimit));
    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,  ("pCfg->deliveryTriggerType= %d\n", pCfg->deliveryTriggerType));

    /*
     * Send the configuration command
     * ==============================
     */
    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
}
#endif


/****************************************************************************
 *                      whal_hwInfoElemQueueConfigurationSet()
 ****************************************************************************
 * DESCRIPTION: Write the Queue configuration (For Quality Of Service)
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemQueueConfigurationSet (HwMboxConfig_T* pHwMboxConfig,
                                           queueTrafficParams_t* pQtrafficParams)
{
    ACXTIDConfig_t    TrafficCategoryCfg;
    ACXTIDConfig_t *  pCfg = &TrafficCategoryCfg;

    os_memoryZero (pHwMboxConfig->hOs, (void *)pCfg, sizeof(*pCfg));

    /*
     * Set information element header
     * ==============================
     */
    pCfg->EleHdr.id = ACX_TID_CFG;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /*
     * Set information element Data
     * ==============================
     */
    pCfg->queueID       = pQtrafficParams->queueID;
    pCfg->channelType   = pQtrafficParams->channelType;
    pCfg->tsid          = pQtrafficParams->tsid;
    pCfg->psScheme      = pQtrafficParams->psScheme; 
    pCfg->APSDConf[0]   = pQtrafficParams->APSDConf[0];
    pCfg->APSDConf[1]   = pQtrafficParams->APSDConf[1];

    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport,HAL_HW_CTRL_MODULE_LOG,
        ("%s queueID = 0x%x, channelType = 0x%x, tsid = 0x%x, psScheme = 0x%x\n",
            __FUNCTION__,pCfg->queueID,pCfg->channelType,pCfg->tsid,pCfg->psScheme));
    
    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport,HAL_HW_CTRL_MODULE_LOG,
        ("APSDConf[0] = 0x%x, APSDConf[1] = 0x%x, len = 0x%x\n",
            pCfg->APSDConf[0],pCfg->APSDConf[0],pCfg->EleHdr.len));

    /*
     * Send the configuration command
     * ==============================
     */
    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
}


/****************************************************************************
 *                      whal_hwInfoElemAcParansConfigurationWrite()
 ****************************************************************************
 * DESCRIPTION: Write the AC configuration (For Quality Of Service)
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAcParamsConfigurationSet (HwMboxConfig_T* pHwMboxConfig,
                                              configureCmdCBParams_t *pConfigureCommand)
{
    ACXAcCfg_t     AcCfg;
    ACXAcCfg_t    *pCfg  = &AcCfg;
    acQosParams_t *pAcQosParams = (acQosParams_t*)(pConfigureCommand->CB_buf);

    os_memoryZero (pHwMboxConfig->hOs, (void *)pCfg, sizeof(*pCfg));

    /*
     * Set information element header
     * ==============================
     */
    pCfg->EleHdr.id = ACX_AC_CFG;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /*
     * Set information element Data
     * ==============================
     */

    pCfg->ac        = pAcQosParams->ac;
    pCfg->aifsn     = pAcQosParams->aifsn;
    pCfg->cwMax     = ENDIAN_HANDLE_WORD(pAcQosParams->cwMax);
    pCfg->cwMin     = pAcQosParams->cwMin;
    pCfg->txopLimit = ENDIAN_HANDLE_WORD(pAcQosParams->txopLimit);

    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport,HAL_HW_CTRL_MODULE_LOG,
        ("whal_hwInfoElemAcParamsConfigurationWrite:\n ac = 0x%x, aifsn = 0x%x, cwMax = 0x%x, cwMin = 0x%x,txopLimit = 0x%x \n",
        pCfg->ac,pCfg->aifsn,pCfg->cwMax,pCfg->cwMin,pCfg->txopLimit));

    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport,HAL_HW_CTRL_MODULE_LOG,
        ("whal_hwInfoElemAcParamsConfigurationWrite:\n ac = 0x%x, aifsn = 0x%x, cwMax = 0x%x, cwMin = 0x%x,txopLimit = 0x%x \n",
        pCfg->ac,pCfg->aifsn,pCfg->cwMax,pCfg->cwMin,pCfg->txopLimit));

    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
}


/****************************************************************************
 *                      whal_hwInfoElemAcParansConfigurationGet()
 ****************************************************************************
 * DESCRIPTION: Write the AC configuration (For Quality Of Service)
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAcParamsConfigurationGet (HwMboxConfig_T* pHwMboxConfig,
                                             configureCmdCBParams_t *pConfigureCommand)
{
    ACXAcCfg_t  AcCfg;
    ACXAcCfg_t *pCfg  = &AcCfg;
    acQosParams_t *pAcQosParams = (acQosParams_t*)(pConfigureCommand->CB_buf);

    os_memoryZero (pHwMboxConfig->hOs, (void *)pCfg, sizeof(*pCfg));

    /*
     * Set information element header
     * ==============================
     */
    pCfg->EleHdr.id = ACX_AC_CFG;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /*
     * Set information element Data
     * ==============================
     */
    pCfg->ac        = pAcQosParams->ac;
    pCfg->aifsn     = pAcQosParams->aifsn;
    pCfg->cwMax     = ENDIAN_HANDLE_WORD(pAcQosParams->cwMax);
    pCfg->cwMin     = pAcQosParams->cwMin;
    pCfg->txopLimit = ENDIAN_HANDLE_WORD(pAcQosParams->txopLimit);

    return CmdQueue_CmdConfigureWithCb (pHwMboxConfig->hCmdMboxQueue, 
                                        pCfg, 
                                        sizeof(*pCfg),
                                        pConfigureCommand->CB_Func,
                                        pConfigureCommand->CB_handle);
}


/****************************************************************************
 *                      whal_hwInfoElemTxQueueCfgSet()
 ****************************************************************************
 * DESCRIPTION: Write the Access category configuration (For Quality Of Service)
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemTxQueueCfgSet(HwMboxConfig_T* pHwMboxConfig,
                                    acQueuesParams_t* pAcQueuesParams,
                                 UINT32 numOfTxBlk)
{
    ACXTxQueueCfg_t    AccessCategory;
    ACXTxQueueCfg_t *pCfg = &AccessCategory;
    UINT16 HighblkRatio, LowBlkRatio;

    /*
     * Set information element header      
     * ==============================
     */
    pCfg->EleHdr.id = ACX_TX_QUEUE_CFG;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);
    
    /*
     * Set information element Data
     * ==============================
     */
    HighblkRatio = pAcQueuesParams->percentOfBlockHighThreshold;
    LowBlkRatio  = pAcQueuesParams->percentOfBlockLowThreshold;

    pCfg->qID = pAcQueuesParams->qId;
    pCfg->numberOfBlockHighThreshold = ENDIAN_HANDLE_WORD((HighblkRatio * numOfTxBlk)/100);
    pCfg->numberOfBlockLowThreshold  = ENDIAN_HANDLE_WORD((LowBlkRatio * numOfTxBlk)/100);

    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, 
                                  pCfg, 
                                  sizeof(*pCfg));
}


/****************************************************************************
 *                      whal_hwInfoElemPacketDetectionThresholdSet()
 ****************************************************************************
 * DESCRIPTION:  Set the PacketDetection threshold
 *
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemPacketDetectionThresholdSet (HwMboxConfig_T* pHwMboxConfig, UINT32* pPdThreshold)
{
    ACXPacketDetection_t    PacketDetectionThresholdCfg;
    ACXPacketDetection_t *pCfg = &PacketDetectionThresholdCfg;

    /*
     * Set information element header
     * ==============================
     */
    pCfg->EleHdr.id = ACX_PD_THRESHOLD;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    /*
     * Set information element Data
     * ==============================
     */
    pCfg->pdThreshold = ENDIAN_HANDLE_LONG(*pPdThreshold);

    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
        ("%s: pdThreshold = 0x%x , len = 0x%x \n",__FUNCTION__,pCfg->pdThreshold,pCfg->EleHdr.len));

    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
}


/****************************************************************************
 *                      whal_hwInfoElemNoiseHistogramResultsGet()
 ****************************************************************************
 * DESCRIPTION: Get the Noise Histogram Measurement Results.
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemNoiseHistogramResultsGet (HwMboxConfig_T* pHwMboxConfig,
                                             interogateCmdCBParams_t noiseHistCBParams)
{
    NoiseHistResult_t   results;
    NoiseHistResult_t   *pCfg = &results;

    /* Set information element header*/
    pCfg->EleHdr.id  = ACX_NOISE_HIST;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    return CmdQueue_CmdInterrogateWithCb (pHwMboxConfig->hCmdMboxQueue, 
                                          pCfg, 
                                          sizeof(*pCfg),
                                          noiseHistCBParams.CB_Func,
                                          noiseHistCBParams.CB_handle,
                                          noiseHistCBParams.CB_buf);
}

/****************************************************************************
*                      whal_hwInfoElemPowerLevelTableGet()
****************************************************************************
* DESCRIPTION: Get the Power level table from NVS.
*
* INPUTS:
*
* OUTPUT:  None
*
* RETURNS: OK or NOK
****************************************************************************/
int  whal_hwInfoElemPowerLevelTableGet		 (HwMboxConfig_T *pHwMboxConfig, 
											  interogateCmdCBParams_t powerLevelCBParams)
{
	PowerLevelTable_t   results;
	PowerLevelTable_t   *pCfg = &results;

	/* Set information element header*/
	pCfg->EleHdr.id  = ACX_POWER_LEVEL_TABLE;
	pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

	return CmdQueue_CmdInterrogateWithCb (pHwMboxConfig->hCmdMboxQueue, 
		pCfg, 
		sizeof(*pCfg), 
		powerLevelCBParams.CB_Func,
		powerLevelCBParams.CB_handle,
		powerLevelCBParams.CB_buf);
}									

/****************************************************************************
 *                      whal_hwInfoElemAcxBeaconFilterOptionsSet()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate the beacon filtering option
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAcxBeaconFilterOptionsSet (HwMboxConfig_T* pHwMboxConfig,
                                              ACXBeaconFilterOptions_t* pWlanElm_BeaconFilterOptions)
{
    int rc;
    ACXBeaconFilterOptions_t* pCfg = pWlanElm_BeaconFilterOptions;
    
    if (NULL == pWlanElm_BeaconFilterOptions)
    {
        return NOK;
    }

    /* Set information element header */
    pCfg->EleHdr.id = ACX_BEACON_FILTER_OPT;
    pCfg->EleHdr.len = sizeof(ACXBeaconFilterOptions_t) - sizeof(EleHdrStruct);

    /* Send the command */
    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue,
                                pCfg,
                                sizeof(ACXBeaconFilterOptions_t));

    return rc;
}


/****************************************************************************
 *                     whal_hwInfoElemAcxBeaconFilterIETableSet
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate the beacon filter IE table
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAcxBeaconFilterIETableSet (HwMboxConfig_T* pHwMboxConfig,
                                              UINT8* numberOfIEs, 
                                              UINT8* IETable,
                                              UINT8* IETableSize)
{
    int rc = OK;
    ACXBeaconFilterIETable_t beaconFilterIETableStruct;
    ACXBeaconFilterIETable_t *pCfg = &beaconFilterIETableStruct;
    
    if (( NULL == IETable ) || ( NULL == pHwMboxConfig )) 
    {
        return PARAM_VALUE_NOT_VALID;
    }

    pCfg->EleHdr.id = ACX_BEACON_FILTER_TABLE;
    pCfg->EleHdr.len = *IETableSize + 1; 
    pCfg->NumberOfIEs = *numberOfIEs;
        
    os_memoryZero (pHwMboxConfig->hOs, (void *)pCfg->IETable, BEACON_FILTER_TABLE_MAX_SIZE);
    os_memoryCopy (pHwMboxConfig->hOs, (void *)pCfg->IETable, (void *)IETable, *IETableSize);
        
    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(ACXBeaconFilterIETable_t));
    
    return rc;
}
 

/****************************************************************************
 *                      whal_hwInfoElemAcxTxOptionsSet()
 ****************************************************************************
 * DESCRIPTION: Change the Event Vector Mask in the FW
 * 
 * INPUTS: MaskVector   The Updated Vector Mask
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemEventMaskSet (HwMboxConfig_T* pHwMboxConfig, UINT32 MaskVector)
{
    int status;

    ACXEventMboxMask_t EventMboxData;
    ACXEventMboxMask_t *pCfg = &EventMboxData;

    /* Set information element header*/
    pCfg->EleHdr.id = ACX_EVENT_MBOX_MASK; 
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);
    pCfg->lowEventMask = MaskVector;
    pCfg->highEventMask = 0xffffffff; /* Not in Use */

    WLAN_REPORT_INFORMATION (pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG, ("whal_hwInfoElemEventMaskSet:\n"));

    status = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
    
    if (status != OK)
    {
        WLAN_REPORT_ERROR (pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
                          ("whal_hwInfoElemEventMaskSet: Error Configure Mask\n"));
        return NOK;
    }

    return OK;
}


/****************************************************************************
 *                      whal_hwInfoElemCcaThresholdSet()
 ****************************************************************************
 * DESCRIPTION: Configure Tx and Rx CCA detection
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemCcaThresholdSet (HwMboxConfig_T* pHwMboxConfig, UINT16* ccaThreshold, BOOL bTxEnergyDetection)
{
    int rc;
    ACXEnergyDetection_t AcxElm_CcaThreshold;
    ACXEnergyDetection_t *pCfg = &AcxElm_CcaThreshold;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_CCA_THRESHOLD;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);
    
    pCfg->rxCCAThreshold = ENDIAN_HANDLE_WORD(*ccaThreshold);
    pCfg->txEnergyDetection = (Bool_e)bTxEnergyDetection;

    /* Send the configuration command */
    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
    
    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemDtimPeriodSet()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate the Slot Time
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemDtimPeriodSet (HwMboxConfig_T* pHwMboxConfig, UINT8* dtimPeriod, UINT16*TBTT)
{
    int rc;
    ACXDtimPeriodCfg_t AcxElm_DtimPeriod;
    ACXDtimPeriodCfg_t *pCfg = &AcxElm_DtimPeriod;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_WR_TBTT_AND_DTIM;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);
    
    pCfg->dtimInterval = (*dtimPeriod);
    pCfg->tbtt = ENDIAN_HANDLE_WORD(*TBTT);

    /* Send the configuration command */
    rc = CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
    
    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemDtimPeriodGet()
 ****************************************************************************
 * DESCRIPTION: Configure/Interrogate the Slot Time
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemDtimPeriodGet (HwMboxConfig_T* pHwMboxConfig, UINT8* dtimPeriod, UINT16*TBTT)
{
    int rc = OK;
    ACXDtimPeriodCfg_t AcxElm_DtimPeriod;
    ACXDtimPeriodCfg_t *pCfg = &AcxElm_DtimPeriod;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_WR_TBTT_AND_DTIM;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);
 
    /* Not implemented */   
  #if 0
            *dtimPeriod = pCfg->dtimInterval;
  #endif
    
    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemAcxLowSNRThresholdSet()
 ****************************************************************************
 * DESCRIPTION: Configure the RSSI threshold parameters
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAcxLowSNRThresholdSet (HwMboxConfig_T* pHwMboxConfig,
                                          ACXLowSNRTriggerParameters_t* AcxElm_LowThresholdOptions)
{
    /* Set information element header */
    AcxElm_LowThresholdOptions->EleHdr.id = ACX_LOW_SNR;
    AcxElm_LowThresholdOptions->EleHdr.len = sizeof(*AcxElm_LowThresholdOptions) - sizeof(EleHdrStruct);

    /* Send the configuration command */
    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, AcxElm_LowThresholdOptions, sizeof(*AcxElm_LowThresholdOptions));
}


/****************************************************************************
 *                      whal_hwInfoElemAcxLowRSSIThresholdSet()
 ****************************************************************************
 * DESCRIPTION: Configure the RSSI threshold parameters
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAcxLowRSSIThresholdSet (HwMboxConfig_T* pHwMboxConfig,
                                           ACXLowRSSITriggerParameters_t* pWlanElm_LowRSSIThresholdOptions)
{
    ACXLowRSSITriggerParameters_t AcxElm_LowRSSIThresholdOptions;
    ACXLowRSSITriggerParameters_t* pCfg = &AcxElm_LowRSSIThresholdOptions;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_LOW_RSSI;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);
    
    pCfg->rssiFilterDepth   = pWlanElm_LowRSSIThresholdOptions->rssiFilterDepth;
    pCfg->rssiFilterWeight = pWlanElm_LowRSSIThresholdOptions->rssiFilterWeight;
    pCfg->rssiThreshold = pWlanElm_LowRSSIThresholdOptions->rssiThreshold;
    pCfg->LowRSSIEventType  = pWlanElm_LowRSSIThresholdOptions->LowRSSIEventType;

    /* Send the configuration command */
    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
}


/****************************************************************************
 *                      whal_hwInfoElemAcxSetMaxTxRetrySet()
 ****************************************************************************
 * DESCRIPTION: Configure the Max Tx Retry parameters
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAcxSetMaxTxRetrySet (HwMboxConfig_T* pHwMboxConfig,
                                        ACXConsTxFailureTriggerParameters_t* pWlanElm_SetMaxTxRetry)
{
    ACXConsTxFailureTriggerParameters_t AcxElm_SetMaxTxRetry;
    ACXConsTxFailureTriggerParameters_t* pCfg = &AcxElm_SetMaxTxRetry;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_CONS_TX_FAILURE;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);
    
    pCfg->maxTxRetry = pWlanElm_SetMaxTxRetry->maxTxRetry;

    /* Send the configuration command */
    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
}


/****************************************************************************
 *                      whal_hwInfoElemAcxBssLossTsfThresholdSet()
 ****************************************************************************
 * DESCRIPTION: Configure the Bss Lost Timeout & TSF miss threshold
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAcxBssLossTsfThresholdSet (HwMboxConfig_T* pHwMboxConfig,                                            
                                              AcxConnectionMonitorOptions* pWlanElm_BssLossTsfSynchronize)
{
    AcxConnectionMonitorOptions AcxElm_SetBssLossTsfThreshold;
    AcxConnectionMonitorOptions* pCfg = &AcxElm_SetBssLossTsfThreshold;

    /* Set information element header */
    pCfg->EleHdr.id     = ACX_CONN_MONIT_PARAMS;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);
    
    pCfg->BSSLossTimeout        = pWlanElm_BssLossTsfSynchronize->BSSLossTimeout;
    pCfg->TSFMissedThreshold    = pWlanElm_BssLossTsfSynchronize->TSFMissedThreshold;

    /* Send the configuration command */
    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
}


/****************************************************************************
 *                      whal_hwInfoElemAcxGetAverageRSSIGet()
 ****************************************************************************
 * DESCRIPTION: Configure the Max Tx Retry parameters
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemAcxGetAverageRSSIGet (HwMboxConfig_T* pHwMboxConfig, INT8* averageRSSI)
{
    int rc = OK;
    ACXAvaregeRSSI_t AcxElm_GetAverageRSSI;
    ACXAvaregeRSSI_t* pCfg = &AcxElm_GetAverageRSSI;

    /* Set information element header */
    pCfg->EleHdr.id = ACX_AVERAGE_RSSI ;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);
    
    /* Not implemented */
  #if 0     
    *averageRSSI = pCfg->avaregeRSSI;
  #endif

    return rc;
}


/****************************************************************************
 *                      whal_hwInfoElemTxRatePolicyConfigurationSet()
 ****************************************************************************
 * DESCRIPTION: Write the TxRateClass configuration 
 *
 * INPUTS:
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemTxRatePolicyConfigurationSet (HwMboxConfig_T* pHwMboxConfig,
                                         txRatePolicy_t *pTxRatePolicy)
{
    ACXTxAttrClasses_t  TxClassCfg;
    ACXTxAttrClasses_t *pCfg  = &TxClassCfg;
    UINT8 PolicyId;
    
    os_memoryZero (pHwMboxConfig->hOs, (void *)pCfg, sizeof(*pCfg));

    /*
     * Set information element header
     * ==============================
     */
    pCfg->EleHdr.id = ACX_RATE_POLICY;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);
    pCfg->numOfClasses = pTxRatePolicy->numOfRateClasses;

    for (PolicyId = 0; PolicyId < pTxRatePolicy->numOfRateClasses; PolicyId++)
    {
        os_memoryCopy (pHwMboxConfig->hOs,
                       (void *)&(pCfg->rateClasses[PolicyId]),
                       (void *)&(pTxRatePolicy->rateClass[PolicyId]),
                       sizeof(txRateClass_t));
    }
    
    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
}


/****************************************************************************
 *                      whal_hwInfoElemRtsThresholdSet()
 ****************************************************************************
 * DESCRIPTION: Configure The RTS threshold
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemRtsThresholdSet (HwMboxConfig_T* pHwMboxConfig,
                                    UINT16 RtsThreshold)
{
    dot11RTSThreshold_t AcxElm_RtsThreshold;
    dot11RTSThreshold_t *pCfg = &AcxElm_RtsThreshold;

    /* Set information element header*/
    pCfg->EleHdr.id = DOT11_RTS_THRESHOLD;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    pCfg->RTSThreshold = RtsThreshold;

    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
}


/****************************************************************************
 *                      whal_hwInfoElemCtsToSelfSet()
 ****************************************************************************
 * DESCRIPTION: Configure The Cts to self feature
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemCtsToSelfSet (HwMboxConfig_T* pHwMboxConfig, UINT8 CtsToSelf)
{
    ACXCtsProtection_t AcxElm_CtsToSelf;
    ACXCtsProtection_t *pCfg = &AcxElm_CtsToSelf;

    /* Set information element header*/
    pCfg->EleHdr.id = ACX_CTS_PROTECTION;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    pCfg->ctsProtectMode = CtsToSelf;

    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
}


/****************************************************************************
 *                      whal_hwInfoElemRxMsduLifeTimeSet()
 ****************************************************************************
 * DESCRIPTION: Configure The Cts to self feature
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemRxMsduLifeTimeSet (HwMboxConfig_T* pHwMboxConfig, UINT32 RxMsduLifeTime)
{
    dot11RxMsduLifeTime_t   AcxElm_RxMsduLifeTime;
    dot11RxMsduLifeTime_t *pCfg = &AcxElm_RxMsduLifeTime;

    /* Set information element header*/
    pCfg->EleHdr.id = DOT11_RX_MSDU_LIFE_TIME;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);
    pCfg->RxMsduLifeTime = RxMsduLifeTime;

    WLAN_REPORT_INFORMATION(pHwMboxConfig->hReport, HAL_HW_CTRL_MODULE_LOG,
        ("%s: RxMsduLifeTime = 0x%x, len = 0x%x\n",__FUNCTION__,pCfg->RxMsduLifeTime,pCfg->EleHdr.len));

    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
}


/****************************************************************************
 *                      whal_hwInfoElemRxTimeOutSet()
 ****************************************************************************
 * DESCRIPTION: Configure The Rx Time Out
 *
 * INPUTS:  None
 *
 * OUTPUT:  None
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemRxTimeOutSet (HwMboxConfig_T* pHwMboxConfig, rxTimeOut_t* pRxTimeOut)                            
{
    ACXRxTimeout_t AcxElm_rxTimeOut;
    ACXRxTimeout_t *pCfg = &AcxElm_rxTimeOut;

    /* Set information element header*/
    pCfg->EleHdr.id = ACX_SERVICE_PERIOD_TIMEOUT;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    pCfg->PsPollTimeout = pRxTimeOut->psPoll;
    pCfg->UpsdTimeout   = pRxTimeOut->UPSD;

    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
}

/****************************************************************************
 *                      whal_hwInfoElemWiFiWmmPSWASet()
 ****************************************************************************
 * DESCRIPTION: Configure The PS for WMM
 *
 * INPUTS:   TRUE  - Configure PS to work on WMM mode - do not send the NULL/PS_POLL 
 *                   packets even if TIM is set.
 *           FALSE - Configure PS to work on Non-WMM mode - work according to the 
 *                   standard
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwInfoElemWiFiWmmPSWASet (HwMboxConfig_T* pHwMboxConfig, BOOL enableWA)
{
    IEConfigPsWmm_t  ConfigPsWmm;
    IEConfigPsWmm_t *pCfg = &ConfigPsWmm;

    /*
     * Set information element header
     */
    pCfg->EleHdr.id = ACX_CONFIG_PS_WMM;
    pCfg->EleHdr.len = sizeof(*pCfg) - sizeof(EleHdrStruct);

    pCfg->ConfigPsOnWmmMode = enableWA;

    /* Report the meesage only if we are using the WiFi patch */
    if (enableWA)
    {
        WLAN_OS_REPORT(("%s PS is on WMM mode\n",__FUNCTION__));
    }
    
    return CmdQueue_CmdConfigure (pHwMboxConfig->hCmdMboxQueue, pCfg, sizeof(*pCfg));
}
