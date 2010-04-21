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
 *   MODULE:  whalHwCtrl.c
 *   PURPOSE: Implements action on the wlan hardware card (Reset, Run, SendCmd, Sw Download)
 *
 ****************************************************************************/

#include "802_11Defs.h"
#include "Ethernet.h"
#include "whalCommon.h"
  
#include "whalCtrl_api.h"
#include "whalHwDefs.h"
#include "whalHwCtrl.h"
#include "whalHwMboxCmd.h"
#include "whalHwMboxConfig.h"
#include "eventMbox_api.h"
#include "whalParams.h"
#include "commonTypes.h"
#include "txResult_api.h"
#include "TNETW_Driver_api.h"
#include "TNETW_Driver.h"
#include "whalSecurity.h"


#define ACX_POWER_MGMT_OPTIONS_STRUCT_DEBUG 0

int  whal_hwCtrl_ConfigTemplates(HwCtrl_T *pHwCtrl);
int  whal_hwCtrl_ConfigQueues(HwCtrl_T *pHwCtrl, UINT32 MemoryStart);
void whal_hwCtrl_OverridePhyRegsDefaults(HwCtrl_T *pHwCtrl);


#define CF_FORM_FACTOR          3 /* Compact Flash*/

#define CB_FORM_FACTOR          1 /* Card Bus */

/****************************************************************************
 *                      whal_hwCtrl_Create()
 ****************************************************************************
 * DESCRIPTION: Create the wlan hardware control object
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: The Created object
 ****************************************************************************/
HwCtrl_T *whal_hwCtrl_Create(TI_HANDLE hOs, WhalParams_T *pWhalParams)
{
    HwCtrl_T *pHwCtrl;

    pHwCtrl = os_memoryAlloc(hOs, sizeof(HwCtrl_T));
    if (pHwCtrl == NULL)
        return NULL;

    os_memoryZero(hOs, (void*)pHwCtrl, sizeof(HwCtrl_T));

    pHwCtrl->hOs = hOs;
    pHwCtrl->pWhalParams = pWhalParams;
    pHwCtrl->pHwMboxCmd     = whal_hwMboxCmd_Create(hOs, pHwCtrl->pWhalParams);
    pHwCtrl->pHwMboxCmdBit  = whal_hwMboxCmdBit_Create(hOs);
    pHwCtrl->pHwMboxConfig  = whal_hwMboxConfig_Create(hOs);
    pHwCtrl->hWhalBus       = whalBus_Create(hOs);

    if ( (!pHwCtrl->pHwMboxCmd) || (!pHwCtrl->pHwMboxConfig) || (!pHwCtrl->hWhalBus) )
    {
        whal_hwCtrl_Destroy(pHwCtrl);
        return NULL;
    }

    return(pHwCtrl);
}

/****************************************************************************
 *                      whal_hwCtrl_Destroy()
 ****************************************************************************
 * DESCRIPTION: Destroy the object 
 * 
 * INPUTS:  
 *      pHwCtrl     The object to free
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_Destroy(HwCtrl_T *pHwCtrl)
{
    if (pHwCtrl == NULL)
        return OK;

    whal_hwMboxCmd_Destroy(pHwCtrl->pHwMboxCmd);
    whal_hwMboxCmdBit_Destroy(pHwCtrl->pHwMboxCmdBit);
    whal_hwMboxConfig_Destroy(pHwCtrl->pHwMboxConfig);
    whalBus_Destroy(pHwCtrl->hWhalBus);

    os_memoryFree(pHwCtrl->hOs, pHwCtrl, sizeof(HwCtrl_T));
    return OK;
}


/****************************************************************************
 *                      whal_hwCtrl_GetTnentwifHandle()
 ****************************************************************************
 * DESCRIPTION: Return TNETWIF handle 
 * 
 * INPUTS:  
 *      pHwCtrl     The object to free
 * 
 * OUTPUT:  None
 * 
 * RETURNS: TNETWIF handle
 ****************************************************************************/
TI_HANDLE whal_hwCtrl_GetTnentwifHandle (HwCtrl_T *pHwCtrl)
{
    return whalBus_GetTnentwifHandle (pHwCtrl->hWhalBus);
}


/****************************************************************************
 *                      whal_hwCtrl_StartJoin()
 ****************************************************************************
 * DESCRIPTION: Enable Rx/Tx and send Start/Join command 
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_StartJoin(HwCtrl_T *pHwCtrl, bssType_e BssType, void *JoinCompleteCB, TI_HANDLE CB_handle)
{
    HwMboxCmd_T *pHwMboxCmd = pHwCtrl->pHwMboxCmd;
    UINT8 HwBssType;
#ifdef TI_DBG  
    UINT8 *pBssId = whal_ParamsGetBssId(pHwCtrl->pWhalParams);

    WLAN_REPORT_INIT(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                     ("whal_hwCtrl_StartJoin: Enable Tx, Rx and Start the Bss, type=%d\n", BssType));
    WLAN_REPORT_INIT(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                     ("------------------------------------------------------------\n"));
    WLAN_REPORT_INIT(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                     ("START/JOIN, SSID=%s, BSSID=%02X-%02X-%02X-%02X-%02X-%02X, Chan=%d\n", whal_ParamsGetElm_Ssid(pHwCtrl->pWhalParams)->serviceSetId, pBssId[0], pBssId[1], pBssId[2], pBssId[3], pBssId[4], pBssId[5], whal_ParamsGetRadioChannel(pHwCtrl->pWhalParams)));
    WLAN_REPORT_INIT(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                     ("------------------------------------------------------------\n"));
#endif /* TI_DBG */

    /* 
     * Set frame rates according to the values previously configured:
     * Driver join -> as configured to whalCtrl_JoinBss()
     * GWSI join   -> as configured to the template framed before, or default values
     * Recovery    -> Saved parameters from last Join command
     */
    whal_hwCtrl_SetFrameRate(pHwCtrl,
                             pHwCtrl->pWhalParams->BssInfoParams.txCtrlFrmRate,
                             pHwCtrl->pWhalParams->BssInfoParams.txCtrlFrmModulation,
                             pHwCtrl->pWhalParams->BssInfoParams.txMgmtFrmRate,
                             pHwCtrl->pWhalParams->BssInfoParams.txMgmtFrmModulation);
    /*
     * set RxFilter (but don't write it to the FW, this is done in the join command),
     * Configure templates content, ...
     */
    whal_hwCtrl_SetBssType(pHwCtrl, BssType, &HwBssType);

    return whal_hwMboxCmd_StartBss(pHwMboxCmd, HwBssType, JoinCompleteCB, CB_handle);

}


/****************************************************************************
 *                      whal_hwCtrl_switchChannel()
 ****************************************************************************
 * DESCRIPTION: Switching the serving channel 
 * 
 * INPUTS: channel  -   new channel number  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_switchChannel(HwCtrl_T *pHwCtrl,UINT8 channel)
{
    HwMboxCmd_T *pHwMboxCmd = pHwCtrl->pHwMboxCmd;

    return whal_hwMboxCmd_EnableTx(pHwMboxCmd,channel);
}


/****************************************************************************
 *                      whal_hwCtrl_DisableTx()
 ****************************************************************************
 * DESCRIPTION: Disable Tx path. 
 * 
 * INPUTS: None
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_DisableTx(HwCtrl_T *pHwCtrl)
{
    HwMboxCmd_T *pHwMboxCmd = pHwCtrl->pHwMboxCmd;

    return whal_hwMboxCmd_DisableTx(pHwMboxCmd);
}

/****************************************************************************
 *                      whal_hwCtrl_EnableTx()
 ****************************************************************************
 * DESCRIPTION: Disable Tx path. 
 * 
 * INPUTS: channel  -   new channel number
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_EnableTx(HwCtrl_T *pHwCtrl, int channel)
{
    HwMboxCmd_T *pHwMboxCmd = pHwCtrl->pHwMboxCmd;

    return whal_hwMboxCmd_EnableTx(pHwMboxCmd, (UINT8)channel);
}


/****************************************************************************
 *                      whal_hwCtrl_EnableDataPath()
 ****************************************************************************
 * DESCRIPTION: Enable Rx/Tx and send Start/Join command 
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_EnableDataPath(HwCtrl_T *pHwCtrl)
{
    HwMboxCmd_T *pHwMboxCmd = pHwCtrl->pHwMboxCmd;

    whal_hwMboxCmd_EnableRx(pHwMboxCmd);
    whal_hwMboxCmd_EnableTx(pHwMboxCmd, whal_ParamsGetDefaultChannel(pHwCtrl->pWhalParams));
    

#ifdef WDBG_POLLING /* (!!!) ONLY FOR DEBUG WHEN THERE ARE NO INTERRUPTS */

    /* allocate OS timer memory */
    hal_timer = os_timerCreate(pHwCtrl->hOs, whal_hwCtrl_RxPollingTimeout, (TI_HANDLE) pHwCtrl);
    if (!hal_timer)
        return NOK;

    os_timerStart(pHwCtrl->hOs, hal_timer, 20, FALSE);
#endif

    return OK;
}

/****************************************************************************
 *                      whal_hwCtrl_EnableDataPath()
 ****************************************************************************
 * DESCRIPTION: Enable Rx/Tx and send Start/Join command 
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_DisableDataPath(HwCtrl_T *pHwCtrl)
{
#if 0
    HwMboxCmd_T *pHwMboxCmd = pHwCtrl->pHwMboxCmd;
    /*
     * L.M. removed because of two reasons: 
     * 1. When the FW is dead, it only adds delay to recovery.
     * 2. WSP does not have it.
     */

    whal_hwMboxCmd_DisableTx(pHwMboxCmd);
    whal_hwMboxCmd_DisableRx(pHwMboxCmd);

    
/*  use FwEvent ... whalBus_hwIntr_Disable(pHwCtrl->hWhalBus, HAL_ALL_INTERRUPTS); */
#endif

    return OK;
}

/****************************************************************************
 *                      whal_hwCtrl_SetBssType()
 ****************************************************************************
 * DESCRIPTION: Set Bss type, set RxFilter 
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_SetBssType(HwCtrl_T *pHwCtrl, bssType_e BssType, UINT8 *HwBssType)
{
    switch (BssType)
    {
    case BSS_AP:
        whal_ParamsSetBssType(pHwCtrl->pWhalParams, BSS_TYPE_AP_BSS);
        whal_ParamsSetRxFilter(pHwCtrl->pWhalParams, RX_CONFIG_OPTION_ANY_DST_MY_BSS, RX_FILTER_OPTION_DEF);
        break;

    case BSS_INFRASTRUCTURE:
        whal_ParamsSetBssType(pHwCtrl->pWhalParams, BSS_TYPE_STA_BSS);
        whal_ParamsSetRxFilter(pHwCtrl->pWhalParams, RX_CONFIG_OPTION_FOR_JOIN, RX_FILTER_OPTION_JOIN);
        break;

    case BSS_INDEPENDENT:
        whal_ParamsSetBssType(pHwCtrl->pWhalParams, BSS_TYPE_IBSS);
	#ifdef GWSI_LIB
	 /* In GWSI we filter with SSID. This is not done in the full driver because of RTP version. 
	   * In the future leave only GWSI option for both cases.
	   */
        whal_ParamsSetRxFilter(pHwCtrl->pWhalParams,  RX_CONFIG_OPTION_FOR_IBSS_JOIN, RX_FILTER_OPTION_DEF);
	#else
	 whal_ParamsSetRxFilter(pHwCtrl->pWhalParams, RX_CONFIG_OPTION_FOR_JOIN, RX_FILTER_OPTION_DEF);
	#endif
        break;

    default: 
        WLAN_REPORT_FATAL_ERROR(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                                ("whal_hwCtrl_SetBssType: FATAL_ERROR, unknown BssType %d\n", BssType));
        return NOK;
    }

    *HwBssType = whal_ParamsGetBssType(pHwCtrl->pWhalParams);

    return OK;
}


/****************************************************************************
 *                      whal_hwCtrl_setRxFilters()
 ****************************************************************************
 * DESCRIPTION: Sets the filters according to the given configuration. 
 * 
 * INPUTS:  RxConfigOption  - The given Rx filters configuration
 *          RxFilterOption  - The given Rx filters options
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_setRxFilters(HwCtrl_T *pHwCtrl, UINT32 RxConfigOption, UINT32 RxFilterOption)
{
    whal_ParamsSetRxFilter(pHwCtrl->pWhalParams, RxConfigOption, RxFilterOption);

    return whal_hwInfoElemRxConfigSet (pHwCtrl->pHwMboxConfig,
                                   &pHwCtrl->pWhalParams->WlanParams.RxConfigOption,
                                       &pHwCtrl->pWhalParams->WlanParams.RxFilterOption);
}


/****************************************************************************
 *                      whal_hwCtrl_GetRxFilters()
 ****************************************************************************
 * DESCRIPTION: Sets the filters according to the given configuration. 
 * 
 * INPUTS:  RxConfigOption  - The given Rx filters configuration
 *          RxFilterOption  - The given Rx filters options
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_GetRxFilters(HwCtrl_T *pHwCtrl, UINT32* pRxConfigOption, UINT32* pRxFilterOption)
{
    whal_ParamsGetRxFilter(pHwCtrl->pWhalParams, pRxConfigOption, pRxFilterOption);

    return OK;
}


/****************************************************************************
 *                 whal_hwCtrl_setRxDataFiltersParams()
 ****************************************************************************
 * DESCRIPTION: Enables or disables Rx data filtering.
 * 
 * INPUTS:  enabled             - 0 to disable data filtering, any other value to enable.
 *          defaultAction       - The default action to take on non-matching packets.
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_setRxDataFiltersParams(HwCtrl_T * pHwCtrl, BOOL enabled, filter_e defaultAction)
{
    return whal_hwInfoElemSetRxDataFiltersParams(pHwCtrl->pHwMboxConfig, enabled, defaultAction);
}


/****************************************************************************
 *                      whal_hwCtrl_setRxDataFilter()
 ****************************************************************************
 * DESCRIPTION: Sets the filters according to the given configuration. 
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
int whal_hwCtrl_setRxDataFilter(HwCtrl_T * pHwCtrl, UINT8 index, UINT8 command, filter_e action, UINT8 numFieldPatterns, UINT8 lenFieldPatterns, UINT8 * fieldPatterns)
{
    return whal_hwInfoElemSetRxDataFilter(pHwCtrl->pHwMboxConfig, 
        index, command, action, numFieldPatterns, lenFieldPatterns, fieldPatterns);
}


/****************************************************************************
 *                      whal_hwCtrl_SetarpIpAddressesTable()
 ****************************************************************************
 * DESCRIPTION: Sets the ARP IP table according to the given configuration. 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_SetarpIpAddressesTable (HwCtrl_T *pHwCtrl, IpAddress_t *IP_addr, UINT8 isEnabled , IPver_e IP_ver)
{
    if ( NULL == IP_addr )
    {
        WLAN_REPORT_ERROR(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,
                    ("whal_hwCtrl_SetarpIpAddressesTable: Ip Addr ptr = NULL !!!\n"));  

        return PARAM_VALUE_NOT_VALID ;
    }

    whal_ParamsSetarpIpAddressesTable(pHwCtrl->pWhalParams, IP_addr, IP_ver);
    whal_ParamsSetarpIpFilterEnabled(pHwCtrl->pWhalParams, isEnabled);

    WLAN_REPORT_DEBUG_CONTROL (pHwCtrl->hReport,
                              ("\n  whal_hwCtrl_SetarpIpAddressesTable - ip filtering : %d.%d.%d.%d \n" , IP_addr->addr[0] , IP_addr->addr[1] , IP_addr->addr[2] , IP_addr->addr[3] )) ;

    /* Set the new ip with the current state (e/d) */
    return whal_hwInfoElemarpIpAddressesTableSet (pHwCtrl->pHwMboxConfig, 
                                              IP_addr, 
                                                  (UINT32)isEnabled);
}

 /****************************************************************************
 *                      whalCtrl_GetArpIpAddressesTable()
 ****************************************************************************
 * DESCRIPTION: Sets the Group table according to the given configuration. 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whalCtrl_GetArpIpAddressesTable (HwCtrl_T *pHwCtrl, IpAddress_t *IP_addr, UINT8* pisEnabled , IPver_e* pIP_ver)
{
    if ( NULL == pHwCtrl ) 
    {
        WLAN_REPORT_ERROR(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,
            ("whalCtrl_GetArpIpAddressesTable  = pHwCtrl NULL !!!\n")); 

        return PARAM_VALUE_NOT_VALID ;
    }

        if ( NULL ==  pHwCtrl->pWhalParams ) 
    {
        WLAN_REPORT_ERROR(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,
            ("whalCtrl_GetArpIpAddressesTable  = pHwCtrl->pWhalParams NULL !!!\n"));    

        return PARAM_VALUE_NOT_VALID ;

    }

    whal_ParamsGetarpIpAddressesTable(pHwCtrl->pWhalParams, IP_addr, pIP_ver);
    whal_ParamsGetarpIpFilterEnabled(pHwCtrl->pWhalParams, pisEnabled);
    return OK;
}

 /****************************************************************************
 *                      whal_hwCtrl_SetarpIpFilterEnabled()
 ****************************************************************************
 * DESCRIPTION: Enable\Disable the ARP filter 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_SetarpIpFilterEnabled(HwCtrl_T *pHwCtrl, UINT8 isEnabled ) 
{
    IpAddress_t *IP_addr = &(pHwCtrl->pWhalParams->WlanParams.arp_IP_addr) ;
    if ( NULL == pHwCtrl )
    {
        WLAN_REPORT_ERROR(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,
                    ("whal_hwCtrl_SetarpIpFilterEnabled  = pHwCtrl NULL !!!\n"));   

        return PARAM_VALUE_NOT_VALID ;
    }

    /* set the current ip address with the new state (e/d) */
    whal_ParamsSetarpIpFilterEnabled(pHwCtrl->pWhalParams, isEnabled);
    return whal_hwInfoElemarpIpAddressesTableSet (pHwCtrl->pHwMboxConfig,  
                                              IP_addr,
                                                  (UINT32)isEnabled);
}

/****************************************************************************
 *                      whal_hwCtrl_SetGroupAddressesTable()
 ****************************************************************************
 * DESCRIPTION: Sets the Group table according to the given configuration. 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_SetGroupAddressesTable (HwCtrl_T *pHwCtrl,
                                        UINT8 numGroupAddrs, 
                                        macAddress_t *Group_addr,
                                        UINT8 isEnabled)
{
    if ( NULL == pHwCtrl ) 
    {
        return PARAM_VALUE_NOT_VALID;
    }

    if ( NULL == Group_addr)
    {
        WLAN_REPORT_ERROR(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,
                    ("whal_hwCtrl_SetGroupAddressesTable: numGroupAddrs=%d Group_addr=0x%x  !!!\n", numGroupAddrs , Group_addr));       
        return PARAM_VALUE_NOT_VALID;
    }

   /* Keeps the parameters in the whal */
    whal_ParamsSetGroupAddressesTable(pHwCtrl->pWhalParams, isEnabled, numGroupAddrs, Group_addr);

    /* Keeps the parameters in the whal for recovery */
    return whal_hwInfoElemGroupAdressesTableSet (pHwCtrl->pHwMboxConfig, 
                                                 &numGroupAddrs, 
                                                 Group_addr,   
                                                 &isEnabled);
}

/****************************************************************************
 *                      whal_hwCtrl_SetRtsThreshold()
 ****************************************************************************
 * DESCRIPTION: Sets the Rts Threshold. 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK  pWlanParams->RtsThreshold
 ****************************************************************************/
int whal_hwCtrl_SetRtsThreshold (HwCtrl_T *pHwCtrl,UINT16 RtsThreshold)
{
    return whal_hwInfoElemRtsThresholdSet (pHwCtrl->pHwMboxConfig, RtsThreshold);
}

/****************************************************************************
 *                      whal_hwCtrl_ConfigCb()
 ****************************************************************************
 * DESCRIPTION: Config the object 
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
static void whal_hwCtrl_ConfigCb (TI_HANDLE hHwCtrl, TI_STATUS status)
{
    HwCtrl_T     *pHwCtrl = (HwCtrl_T*)hHwCtrl; 
    WHAL_CTRL    *pWhalCtrl = (WHAL_CTRL *)pHwCtrl->hWhalCtrl;
 
    whal_hwMboxCmd_Config(pHwCtrl->pHwMboxCmd, ((TnetwDrv_t*)pWhalCtrl->hTNETW_Driver)->hCmdQueue, pHwCtrl->hReport);
    CmdQueue_Config(((TnetwDrv_t*)pWhalCtrl->hTNETW_Driver)->hCmdQueue,
                    ((TnetwDrv_t*)pWhalCtrl->hTNETW_Driver)->hCmdMBox, pHwCtrl->hReport);
    whal_hwMboxCmdBit_Config(pHwCtrl->hWhalCtrl, pHwCtrl->pHwMboxCmdBit, ((TnetwDrv_t*)pWhalCtrl->hTNETW_Driver)->hCmdQueue, pHwCtrl->hReport);
    whal_hwMboxConfig_Config(pHwCtrl->pHwMboxConfig, ((TnetwDrv_t*)pWhalCtrl->hTNETW_Driver)->hCmdQueue, pHwCtrl->hReport);

    /* This will initiate the download to the FW */
    status = whal_hwCtrl_Initiate (pHwCtrl);
    if (status == TNETWIF_ERROR)
    {
        WLAN_REPORT_ERROR (pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                           ("whal_hwCtrl_Config: failed to initialize\n"));
    }
}

 /****************************************************************************
 *                      whal_hwCtrl_GetGroupAddressesTable()
 ****************************************************************************
 * DESCRIPTION: Sets the Group table according to the given configuration. 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_GetGroupAddressesTable (HwCtrl_T *pHwCtrl,
                                        UINT8* pisEnabled, UINT8* pnumGroupAddrs, macAddress_t *Group_addr)
{
    if ( NULL == pHwCtrl ) 
    {
        return PARAM_VALUE_NOT_VALID;
    }

    if ( (NULL == pisEnabled) || (NULL == pnumGroupAddrs) || (NULL == Group_addr))
    {
        WLAN_REPORT_ERROR(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,
                    ("whal_hwCtrl_GetGroupAddressesTable: pisEnabled=0x%p pnumGroupAddrs=0x%p  Group_addr=0x%p !!!\n", pisEnabled , pnumGroupAddrs, Group_addr));       
        return PARAM_VALUE_NOT_VALID;
    }

    whal_ParamsGetGroupAddressesTable(pHwCtrl->pWhalParams, pisEnabled, pnumGroupAddrs, Group_addr);
    return OK;
}


/****************************************************************************
 *                      whal_hwCtrl_Config()
 ****************************************************************************
 * DESCRIPTION: Config the object 
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
TI_STATUS whal_hwCtrl_Config
(
    HwCtrl_T   *pHwCtrl, 
    TI_HANDLE   hWhalCtrl,
    UINT8       AccessMode, 
    UINT32      AcxRegAddr, 
    UINT32      AcxMemAddr, 
    TI_HANDLE   hReport, 
    TI_HANDLE   hMemMgr,
    UINT32     *pFWImage,
    TI_HANDLE   hEventMbox
)
{
    pHwCtrl->hReport = hReport;
    pHwCtrl->hWhalCtrl = hWhalCtrl;
    pHwCtrl->hEventMbox = hEventMbox;

    /* 
     * NOTE: Save firmware image parameters before the 1st TNETWIF call.
     *       These parameters are passed from the user application and
     *       may be lost in a case TNETWIF call is asynchronous. 
     */
    pHwCtrl->uFwBuf = pFWImage[0];
    pHwCtrl->uFwAddr = pFWImage[1];
    pHwCtrl->uEEEPROMBuf = pFWImage[2];
    pHwCtrl->uEEEPROMLen = pFWImage[3];

    return whalBus_Config (pHwCtrl->hWhalBus, 
                           hWhalCtrl, 
                           AccessMode, 
                           AcxRegAddr, 
                           AcxMemAddr, 
                           hReport, 
                           hMemMgr, 
                           whal_hwCtrl_ConfigCb, 
                           pHwCtrl);
}   


/****************************************************************************
 *                      whal_hwCtrl_FinalizeDownloadCb2()
 ****************************************************************************
 * DESCRIPTION: Finalize all the remaining initialization after the download has finished 
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
static void whal_hwCtrl_FinalizeDownloadCb2 (TI_HANDLE hHwCtrl, TI_STATUS status, void *pData)
{
    HwCtrl_T      *pHwCtrl = (HwCtrl_T *)hHwCtrl;     
    whalCtrl_antennaDiversityOptions_t antennaDiversityOptions;                   

    /* Print firmware version */
    whal_ParamsPrintFwVersion (pHwCtrl->pWhalParams);

    /*
     * Configure antenna diversity parameters, same for both radio types.
     * (the only difference between DCR and WBR is the antennas number, which is 
     * hard-coded in the mbox config function per radio type 
     */
    antennaDiversityOptions.enableRxDiversity = FALSE;
    antennaDiversityOptions.rxSelectedAntenna = DIVS_RX_START_ANT2;
    antennaDiversityOptions.enableTxDiversity = FALSE;
    antennaDiversityOptions.txSelectedAntenna = DIVS_TX_START_ANT2;
    antennaDiversityOptions.rxTxSharedAnts = TRUE;
    whal_hwCtrl_SaveAntennaDiversityOptions (pHwCtrl, &antennaDiversityOptions);

    whalCtrl_FinalizeDownload (pHwCtrl->hWhalCtrl);
}


/****************************************************************************
 *                      whal_hwCtrl_FinalizeDownloadCb1()
 ****************************************************************************
 * DESCRIPTION: Finalize all the remaining initialization after the download has finished 
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
static void whal_hwCtrl_FinalizeDownloadCb1 (TI_HANDLE hHwCtrl, TI_STATUS status, void *pData)
{
    HwCtrl_T      *pHwCtrl = (HwCtrl_T *)hHwCtrl;     
    ACXRevision_t *pACXRevision = whal_ParamsGetAcxVersion (pHwCtrl->pWhalParams);   
    UINT8         *pStationId = ((dot11StationIDStruct*)pData)->dot11StationID;
    UINT32         i;

    /* Swap bytes of the station id */
    for (i = 0; i < 3; i++)
    {
        UINT8 uTmp = pStationId[i];
        pStationId[i] = pStationId[5 - i]; 
        pStationId[5 - i] = uTmp; 
    }

    whal_ParamsSetSrcMac (pHwCtrl->pWhalParams, (char*)pStationId);

    /* Get firmware version */
    whal_hwInfoElemAcxRevisionGet (pHwCtrl->pHwMboxConfig, 
                                   (void *)whal_hwCtrl_FinalizeDownloadCb2,
                                   hHwCtrl,
                                   pACXRevision);
}
                            

/****************************************************************************
 *                      whal_hwCtrl_FinalizeDownload()
 ****************************************************************************
 * DESCRIPTION: Finalize all the remaining initialization after the download has finished 
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
TI_STATUS whal_hwCtrl_FinalizeDownload (TI_HANDLE hHwCtrl, BootAttr_T *pBootAttr)
{
    HwCtrl_T *pHwCtrl= (HwCtrl_T *)hHwCtrl;     

    /* 
     * Just comment it since we may need it in future version when we will read from the NVS the 
     * Configure options (for example power levels)
     */
    WlanParams_T *pWlanParams = whal_ParamsGetWlanParams (pHwCtrl->pWhalParams);
    
    /* Read NVS version */
    pWlanParams->radioType   = pBootAttr->radioType;
    pWlanParams->minorE2Ver  = pBootAttr->minorE2Ver;
    pWlanParams->majorE2Ver  = pBootAttr->majorE2Ver;
    pWlanParams->bugfixE2Ver = pBootAttr->bugfixE2Ver;
    
    /*
     * Read config options (WLAN hardware EEPROM). Must be before any configuration 
     * because the WLAN hardware put the data in the mbox after running the FW
     * Not used by now but keep it in code since the data may be requested later on when the 
     * NVS data will be read from the driver and not from the INI: 
     * For example the number of power level 
     * whal_hwInfoElemConfigOptionsRead(pHwCtrl->pHwMboxConfig, pConfigOptions);
     */
    if (whal_hwInfoElemStationIdGet (pHwCtrl->pHwMboxConfig, 
                                     (void *)whal_hwCtrl_FinalizeDownloadCb1,
                                     hHwCtrl,
                                     &pHwCtrl->mbox) != OK)
    {
        WLAN_REPORT_ERROR (pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                           ("whal_hwCtrl_Config: Error on whal_hwInfoElemStationIdGet\n"));
        /* For driver debug only, don't return error */
        /* return NOK; */
    }

    return OK;
}


/****************************************************************************
 *                      whal_hwCtrl_FinalizeOnFailure()
 ****************************************************************************
 * DESCRIPTION: Finalize all the initialization upon failure
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
TI_STATUS whal_hwCtrl_FinalizeOnFailure (TI_HANDLE hHwCtrl)
{
    HwCtrl_T *pHwCtrl= (HwCtrl_T *)hHwCtrl;     

    return whalCtrl_FinalizeOnFailure (pHwCtrl->hWhalCtrl);
}





typedef int (*fcallback_t) (TI_HANDLE, TI_STATUS);


/****************************************************************************
 *                      whal_hwCtrl_ConfigHwCb2()
 ****************************************************************************
 * DESCRIPTION: Configure the WLAN hardware
 * 
 * INPUTS: None 
 * 
 * OUTPUT: None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
static int whal_hwCtrl_ConfigHwCb2 (HwCtrl_T *pHwCtrl, TI_STATUS status, void *pData)
{
    WlanParams_T   *pWlanParams = whal_ParamsGetWlanParams (pHwCtrl->pWhalParams);
    UINT8          *pSrcMacAddr = whal_ParamsGetSrcMac (pHwCtrl->pWhalParams);
    UINT32          acID; 
    whalCtrl_powerMgmtConfig_t powerMgmtConfig;                   

    /* Arrived from callback */
    if (pData)
    {
        ACXDataPathParamsResp_t *pCfg = &pHwCtrl->DataPathParams;

        WLAN_REPORT_INIT (pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG, 
            ("%s: rxPacketRingChunkSize = 0x%x,txPacketRingChunkSize = 0x%x,rxPacketRingAddr = 0x%x\n",
            __FUNCTION__,pCfg->rxPacketRingChunkSize,pCfg->txPacketRingChunkSize,pCfg->rxPacketRingAddr));
        
        WLAN_REPORT_INIT (pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG, 
            ("(cont')%s: txPacketRingAddr = 0x%x,rxControlAddr = 0x%x,txControlAddr = 0x%x,txCompleteAddr = 0x%x\n",
            __FUNCTION__,pCfg->txPacketRingAddr,pCfg->rxControlAddr,pCfg->txControlAddr,pCfg->txCompleteAddr));     

        pCfg->rxPacketRingChunkSize = ENDIAN_HANDLE_WORD(pCfg->rxPacketRingChunkSize);
        pCfg->txPacketRingChunkSize = ENDIAN_HANDLE_WORD(pCfg->txPacketRingChunkSize);
        pCfg->rxPacketRingAddr      = ENDIAN_HANDLE_LONG(pCfg->rxPacketRingAddr); 
        pCfg->txPacketRingAddr      = ENDIAN_HANDLE_LONG(pCfg->txPacketRingAddr);
        pCfg->rxControlAddr         = ENDIAN_HANDLE_LONG(pCfg->rxControlAddr);   
        pCfg->txControlAddr         = ENDIAN_HANDLE_LONG(pCfg->txControlAddr);   
        pCfg->txCompleteAddr        = ENDIAN_HANDLE_LONG(pCfg->txCompleteAddr);
    }

    /* Configure WEP maximum space */
    WLAN_REPORT_INIT (pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG, ("whal_hwCtrl_Configure: WEP cache - none\n"));                  

  #ifdef CORE_5_0 
    whal_hwInfoElemMemoryMapPrint (pHwCtrl->pHwMboxConfig);
  #endif

    /* Override WLAN hardware defaults */
    whal_hwInfoElemStationIdSet (pHwCtrl->pHwMboxConfig, pSrcMacAddr);
    /* Configure the Rx Msdu Life Time (expiry time of de-fragmentation in FW) */
    whal_hwInfoElemRxMsduLifeTimeSet (pHwCtrl->pHwMboxConfig, pWlanParams->MaxRxMsduLifetime);
    whal_hwInfoElemRxConfigSet (pHwCtrl->pHwMboxConfig, &pWlanParams->RxConfigOption, &pWlanParams->RxFilterOption);

  #if 0 
    /* Use firmware default parameters for ant. which is ant 2 for both TX and RX */
    whal_hwCtrl_CurrentAntennaDiversitySendCmd (pHwCtrl);
  #endif

    for (acID = 0; acID < MAX_NUM_OF_AC; acID++)
    {
        whal_hwCtrl_QueueConf (pHwCtrl, &pWlanParams->acQueuesParams[acID]);

        /*
         * NOTE: Set following parameters only if they were configured.
         *       Otherwise, they contain garbage.
         */

        if (pHwCtrl->pWhalParams->AcParams.isAcConfigured[acID])
        {
            configureCmdCBParams_t configureCmdAc = {NULL,NULL,NULL};

            configureCmdAc.CB_buf = (UINT8*)&pHwCtrl->pWhalParams->AcParams.ac[acID];
            whal_hwCtrl_AcParamsConf (pHwCtrl, &configureCmdAc);
        }

        if (pHwCtrl->pWhalParams->QueuesParams.isQueueConfigured[acID])
        {
            whal_hwCtrl_TrafficConf (pHwCtrl, &pHwCtrl->pWhalParams->QueuesParams.queues[acID]);
        }
    }

    whal_hwCtrl_PacketDetectionThreshold (pHwCtrl, &pHwCtrl->pWhalParams->WlanParams.PacketDetectionThreshold);
    whal_hwCtrl_SetSlotTime (pHwCtrl, (slotTime_e )pWlanParams->SlotTime);
    whal_hwCtrl_SetarpIpAddressesTable (pHwCtrl, 
                                        &pWlanParams->arp_IP_addr, 
                                        pWlanParams->isArpIpFilteringEnabled, 
                                        IP_VER_4);
    whal_hwCtrl_SetGroupAddressesTable (pHwCtrl, 
                                        pWlanParams->numGroupAddrs, 
                                        pWlanParams->Group_addr, 
                                        pWlanParams->isMacAddrFilteringnabled);
    whal_hwInfoElemRxTimeOutSet (pHwCtrl->pHwMboxConfig, &pWlanParams->rxTimeOut);
    whal_hwCtrl_SetRtsThreshold (pHwCtrl, pWlanParams->RtsThreshold);
    
    /* Set The Beacon Filter in HAL */
    whal_hwCtrl_SetBeaconFiltering (pHwCtrl, 
                                    pWlanParams->beaconFilterParams.desiredState,
                                    pWlanParams->beaconFilterParams.numOfElements);
    whal_hwCtrl_SetBeaconFilterIETable (pHwCtrl, 
                                        &pWlanParams->beaconFilterIETable.numberOfIEs,
                                        pWlanParams->beaconFilterIETable.IETable,
                                        &pWlanParams->beaconFilterIETable.IETableSize);

    /* Set SG Params only in Init Phase. */
    /* In recovery it will be handled in SoftGemini_hanfleRecovery() */
    if (pData)
    {
        WLAN_REPORT_INIT (pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG, ("whal_hwCtrl_Configure: Setting the Soft Gemini state\n"));    

        /* Set the Soft Gemini state */
        if (pWlanParams->SoftGeminiEnable == SG_SENSE_ACTIVE)
        {
            whal_hwCtrl_SoftGeminiEnable (pHwCtrl, SG_SENSE_NO_ACTIVITY);
        }
        else
        {
            whal_hwCtrl_SoftGeminiEnable (pHwCtrl, pWlanParams->SoftGeminiEnable);
        }

        /* Set the Soft Gemini params */
        whal_hwCtrl_SetSoftGeminiParams (pHwCtrl, &pWlanParams->SoftGeminiParams);
    }

    /* For recovery decision */
    eventMbox_EvUnMask (pHwCtrl->hEventMbox, HAL_EVENT_HEALTH_REPORT);
    whal_hwCtrl_OverridePhyRegsDefaults (pHwCtrl);
  #ifdef TNETW1150
    whal_hwCtrl_SetACIConfiguration (pHwCtrl, 
                                     pWlanParams->ACIMode, 
                                     pWlanParams->inputCCA, 
                                     pWlanParams->qualifiedCCA,
                                     pWlanParams->stompForRx, 
                                     pWlanParams->stompForTx, 
                                     pWlanParams->txCCA);
  #endif/*TNETW1150*/

    /* Beacon broadcast options */
    powerMgmtConfig.BcnBrcOptions = pWlanParams->BcnBrcOptions;
    powerMgmtConfig.ConsecutivePsPollDeliveryFailureThreshold = pWlanParams->ConsecutivePsPollDeliveryFailureThreshold;
    whal_hwCtrl_BcnBrcOptions (pHwCtrl, &powerMgmtConfig);

    /* Enable rx/tx path on the hardware */
    if (whal_hwCtrl_EnableDataPath (pHwCtrl) != OK)
        return NOK;

    /* ACX for a work around for Wi-Fi test */
    whal_hwInfoElemWiFiWmmPSWASet (pHwCtrl->pHwMboxConfig, pWlanParams->WiFiWmmPS);

    /* Enable the scan complete interrupt source */
    eventMbox_EvUnMask (pHwCtrl->hEventMbox, HAL_EVENT_SCAN_CMPLT);
    eventMbox_EvUnMask (pHwCtrl->hEventMbox, HAL_EVENT_SPS_SCAN_CMPLT);

    /* Call the upper layer callback */
    return (*((fcallback_t)pHwCtrl->fCb)) (pHwCtrl->hCb, OK);
}


/****************************************************************************
 *                      whal_hwCtrl_ConfigHwCb1()
 ****************************************************************************
 * DESCRIPTION: Configure the WLAN hardware
 * 
 * INPUTS: None 
 * 
 * OUTPUT: None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
static int whal_hwCtrl_ConfigHwCb1 (HwCtrl_T *pHwCtrl, TI_STATUS status, void *pData)
{
    MemoryMap_t    *pMemMap = &pHwCtrl->MemMap;
    DmaParams_T    *pDmaParams = whal_ParamsGetDmaParams (pHwCtrl->pWhalParams);
    WlanParams_T   *pWlanParams = whal_ParamsGetWlanParams (pHwCtrl->pWhalParams);

    /* Arrived from callback */
    if (pData)
    {
        UINT32         *pSwap, i;

        /* Solve endian problem (all fields are 32 bit) */
        pSwap = (UINT32* )&(pMemMap->codeStart);
        for (i = 0; i < MEM_MAP_NUM_FIELDS; i++)
            pSwap[i] = ENDIAN_HANDLE_LONG(pSwap[i]);
    }

    /* Save number of TX blocks */
    pDmaParams->NumTxBlocks = pMemMap->numTxMemBlks;

    /* 
     * Configure DataPath parameters to default 
     * values and Read the Addresses of the FW data path buffers
     */

    /* Set Data path parameters to constant value to emulate the original double buffer*/
    whal_hwInfoElemDataPathParamsSet (pHwCtrl->pHwMboxConfig, 
                                      DP_RX_PACKET_RING_CHUNK_SIZE, 
                                      DP_TX_PACKET_RING_CHUNK_SIZE, 
                                      DP_RX_PACKET_RING_CHUNK_NUM, 
                                      DP_TX_PACKET_RING_CHUNK_NUM, 
                                      pWlanParams->TxCompleteThreshold, 
                                      FW_TX_CMPLT_BLOCK_SIZE,
                                      DP_TX_COMPLETE_TIME_OUT);

    /* Arrived from callback */
    if (pData)
    {
        /* Get the double buffers and registers address values */
        return whal_hwInfoElemDataPathParamsGet (pHwCtrl->pHwMboxConfig, 
                                                 &pHwCtrl->DataPathParams,
                                                 (void *)whal_hwCtrl_ConfigHwCb2,
                                                 pHwCtrl);
    }

    /* Called directly */
    else
    {
        return whal_hwCtrl_ConfigHwCb2 (pHwCtrl, OK, NULL);
    }
}


/****************************************************************************
 *                      whal_hwCtrl_ConfigHw()
 ****************************************************************************
 * DESCRIPTION: Configure the WLAN hardware
 * 
 * INPUTS: None 
 * 
 * OUTPUT: None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_ConfigHw (HwCtrl_T *pHwCtrl, void *fCb, TI_HANDLE hCb, BOOL bRecovery)
{
    MemoryMap_t    *pMemMap = &pHwCtrl->MemMap;

    /*
     * The addresses of the Double buffer, The Tx Path Status, 
     * Rx Path Status, Tx Path Control, Rx Path Control 
     */
    /* ACXDataPathParamsResp_t  *DataPathParam = &pHwCtrl->DataPathParams; */

    /* 
     * The DmaParams_T is the same struct as the halTxRxQueueGlobalsParams_t struct 
     * but is defined in the whalBus_Defs.h and not in the paramOut.h as done by BCIL 
     */
    DmaParams_T    *pDmaParams = whal_ParamsGetDmaParams (pHwCtrl->pWhalParams);

    pHwCtrl->fCb = fCb;
    pHwCtrl->hCb = hCb; 

    /* Configure the WLAN hardware memory (WEP, Templates, Queue, Buffers) */

    /* Configure packet templates */
    WLAN_REPORT_INIT (pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG, ("whal_hwCtrl_Configure: templates \n"));                
    whal_hwCtrl_ConfigTemplates (pHwCtrl);

    /* Configure RX/TX queues */
    WLAN_REPORT_INIT (pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG, ("whal_hwCtrl_Configure: queues\n"));
                     
    /* Configure the weight among the different hardware queues */
    whal_hwInfoElemConfigMemorySet (pHwCtrl->pHwMboxConfig, pDmaParams);

    /* Extract total number of blocks in the pool */
    if (bRecovery)
        return whal_hwCtrl_ConfigHwCb1 (pHwCtrl, OK, NULL);
    else
        return whal_hwInfoElemMemoryMapGet (pHwCtrl->pHwMboxConfig, 
                                         pMemMap, 
                                         (void *)whal_hwCtrl_ConfigHwCb1,
                                         pHwCtrl);
}


#ifdef TNETW1150
/****************************************************************************
 *                      whal_hwCtrl_SetACIConfiguration()
 ****************************************************************************
 * DESCRIPTION: Set the hardware ACI configuration
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_SetACIConfiguration (HwCtrl_T *pHwCtrl, UINT8 ACIMode,
                                        UINT8 inputCCA, UINT8 qualifiedCCA,
                                        UINT8 stompForRx, UINT8 stompForTx,
                                        UINT8 txCCA)
{
   return (whal_hwInfoElemACIConfigurationSet (pHwCtrl->pHwMboxConfig, ACIMode,
                                        inputCCA, qualifiedCCA, stompForRx,
                                        stompForTx, txCCA));
}
#endif/*TNETW1150*/

/****************************************************************************
 *                      whal_hwCtrl_SetMacAddress()
 ****************************************************************************
 * DESCRIPTION: 
 * 
 * INPUTS:  None    
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_SetMacAddress(HwCtrl_T *pHwCtrl, macAddress_t *macAddr)
{
    whal_ParamsSetSrcMac(pHwCtrl->pWhalParams, (char*)macAddr->addr);

    return whal_hwInfoElemStationIdSet (pHwCtrl->pHwMboxConfig, (UINT8*)macAddr->addr);
}


/****************************************************************************
 *                      whal_hwCtrl_ConfigTemplates()
 ****************************************************************************
 * DESCRIPTION: Configure the packet templates 
 *
 *      AP          - beacon, probe response, tim
 *      STA (INFRA) - probe request
 *      STA (IBSS)  - beacon, probe response, probe request
 *      know yet the bss type
 * 
 * INPUTS:      
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_ConfigTemplates(HwCtrl_T *pHwCtrl)
{
    UINT8 PartialVirtualBmap[DOT11_PARTIAL_VIRTUAL_BITMAP_MAX];
    UINT8 BmapControl;
    WlanParams_T *pWlanParams = whal_ParamsGetWlanParams(pHwCtrl->pWhalParams);

    /* 
     * Probe request template 
     */
    whal_hwMboxCmd_ConfigureTemplateFrame(pHwCtrl->pHwMboxCmd, NULL, pWlanParams->probeRequestTemplateSize,
                                            CMD_PROBE_REQ,NULL,NULL); 

    /* 
     * Null Data template 
     */
    whal_hwMboxCmd_ConfigureTemplateFrame(pHwCtrl->pHwMboxCmd, NULL, pWlanParams->nullTemplateSize,
                                            CMD_NULL_DATA,NULL,NULL); 

    /* 
     * Ps Poll template 
     */
      whal_hwMboxCmd_ConfigureTemplateFrame(pHwCtrl->pHwMboxCmd, NULL, pWlanParams->PsPollTemplateSize,
                                            CMD_PS_POLL,NULL,NULL); 
  
    /* 
     * Qos Null Data template
     */
      whal_hwMboxCmd_ConfigureTemplateFrame(pHwCtrl->pHwMboxCmd, NULL, pWlanParams->qosNullDataTemplateSize,
                                            CMD_QOS_NULL_DATA,NULL,NULL); 

    /* 
     * Probe response template 
     */
    whal_hwMboxCmd_ConfigureTemplateFrame(pHwCtrl->pHwMboxCmd, NULL, pWlanParams->probeResponseTemplateSize,
                                            CMD_PROBE_RESP,NULL,NULL); 
    /* 
     * Beacon template 
     */
    whal_hwMboxCmd_ConfigureTemplateFrame(pHwCtrl->pHwMboxCmd, NULL, pWlanParams->beaconTemplateSize,
                                            CMD_BEACON,NULL,NULL); 

    /* 
     * Tim template, first reserve space (len=MAX), second init to empty 
     */
    BmapControl = 0;
    os_memoryZero(pHwCtrl->hOs, (void*)PartialVirtualBmap, DOT11_PARTIAL_VIRTUAL_BITMAP_MAX);
    whal_hwMboxCmd_TimTemplate(pHwCtrl->pHwMboxCmd, BmapControl, (char*)PartialVirtualBmap, DOT11_PARTIAL_VIRTUAL_BITMAP_MAX);
    whal_hwMboxCmd_TimTemplate(pHwCtrl->pHwMboxCmd, BmapControl, (char*)PartialVirtualBmap, 1);

    return OK;
}


/****************************************************************************
 *                      whal_hwCtrl_SetSlotTime()
 ****************************************************************************
 * DESCRIPTION: Set the Slot field in ACM_IFS_CFG1 hardware register
 *
 * INPUTS:  
 *      SlotTimeVal     The Short SlotTime bit value in the Capabilities
 * 
 * OUTPUT:  None
 * 
 * RETURNS: None
 ****************************************************************************/
int whal_hwCtrl_SetSlotTime(HwCtrl_T *pHwCtrl, slotTime_e SlotTimeVal)
{
    UINT8        slotTime;

    WLAN_REPORT_DEBUG_CONTROL(pHwCtrl->hReport,(" whal_hwCtrl_SetSlotTime: SlotTimeVal = 0x%x\n",SlotTimeVal));

    if (SlotTimeVal == SLOT_TIME_LONG)
        slotTime = (UINT8) SLOT_TIME_LONG;
    else
        slotTime = (UINT8) SLOT_TIME_SHORT;

    return whal_hwInfoElemSlotTimeSet (pHwCtrl->pHwMboxConfig, &slotTime);

}


/****************************************************************************
 *                      whal_hwCtrl_SetPreamble()
 ****************************************************************************
 * DESCRIPTION: Set the preamble in ?????? hardware register
 *
 * INPUTS:  
 *      preambleVal     
 * 
 * OUTPUT:  None
 * 
 * RETURNS: None
 ****************************************************************************/
int whal_hwCtrl_SetPreamble(HwCtrl_T *pHwCtrl, Preamble_e preambleVal)
{
    UINT8        preamble;

    preamble = (UINT8)preambleVal;

    return whal_hwInfoElemPreambleSet (pHwCtrl->pHwMboxConfig, &preamble);
}

/****************************************************************************
 *                      whal_hwCtrl_SetFrameRate()
 ****************************************************************************
 * DESCRIPTION: Set the Frame Rate to HW
 *
 * INPUTS:  
 *  Rate_e  txCtrlFrmRate;
 *    Mod_e     txCtrlFrmMod;
 *    Rate_e    txMgmtFrmRate;
 *    Mod_e     txMgmtFrmMod;       
 * 
 * OUTPUT:  None
 * 
 * RETURNS: None
 ****************************************************************************/
int whal_hwCtrl_SetFrameRate (HwCtrl_T *pHwCtrl, 
                                UINT8   txCtrlFrmRateVal,
                                UINT8   txCtrlFrmModVal,
                                UINT8   txMgmtFrmRateVal,
                                UINT8   txMgmtFrmModVal)
{
    UINT8        txCtrlFrmRate;
    UINT8        txCtrlFrmMod;
    UINT8        txMgmtFrmRate;
    UINT8        txMgmtFrmMod;
    
    txCtrlFrmRate   = txCtrlFrmRateVal;
    txCtrlFrmMod        = txCtrlFrmModVal;
    txMgmtFrmRate   = txMgmtFrmRateVal;
    txMgmtFrmMod    = txMgmtFrmModVal;


    return whal_hwInfoElemGeneratedFrameRateSet (pHwCtrl->pHwMboxConfig,
                                                &txCtrlFrmRate,
                                                &txCtrlFrmMod,
                                                &txMgmtFrmRate,
                                                 &txMgmtFrmMod);

}

/****************************************************************************
 *                      whal_hwCtrl_PMConfig()
 ****************************************************************************
 * DESCRIPTION: Configure the wlan hardware
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_PMConfig(HwCtrl_T *pHwCtrl, whalCtrl_powerMgmtConfig_t *pPMConfig)
{
    
    ACXConfigPM_t AcxElm_PMConfig;
    ACXConfigPM_t *pCfg = &AcxElm_PMConfig;


    pCfg->BBWakeUpTime      = pPMConfig->BBWakeUpTime;

    pCfg->ELPEnable         = pPMConfig->ELPEnable;

    pCfg->PLLlockTime       = pPMConfig->PLLlockTime;

    WLAN_REPORT_INFORMATION(pHwCtrl->hReport,
                            HAL_HW_CTRL_MODULE_LOG,
                            (" whal_hwCtrl_PMConfig  BBWakeUpTime=%d ELPEnable=%d PLLlockTime=%d WakeOnGPIOenable=0x%x\n"
                             ,pCfg->BBWakeUpTime,pCfg->ELPEnable,pCfg->PLLlockTime,pCfg->WakeOnGPIOenable));

    /*
     * Set the desired features 
     */
    return whal_hwInfoElemAcxPMConfigSet (pHwCtrl->pHwMboxConfig, pCfg);
}

/****************************************************************************
 *                      whal_hwCtrl_BcnBrcOptions()
 ****************************************************************************
 * DESCRIPTION: Configure the wlan hardware
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_BcnBrcOptions(HwCtrl_T *pHwCtrl, whalCtrl_powerMgmtConfig_t *pPMConfig)
{
    ACXBeaconAndBroadcastOptions_t AcxElm_BcnBrcOptions;
    ACXBeaconAndBroadcastOptions_t *pCfg = &AcxElm_BcnBrcOptions;


    pCfg->beaconRxTimeOut       = pPMConfig->BcnBrcOptions.BeaconRxTimeout;

    pCfg->broadcastTimeOut  = pPMConfig->BcnBrcOptions.BroadcastRxTimeout;

    pCfg->rxBroadcastInPS       = pPMConfig->BcnBrcOptions.RxBroadcastInPs;

    pCfg->consecutivePsPollDeliveryFailureThr = pPMConfig->ConsecutivePsPollDeliveryFailureThreshold;

    WLAN_REPORT_INFORMATION(pHwCtrl->hReport,
        HAL_HW_CTRL_MODULE_LOG,
        (" whal_hwCtrl_BcnBrcOptions  BeaconRxTimeout=%d BroadcastRxTimeout=%d RxBroadcastInPs=0x%x ConsecutivePsPoll = %d\n"
         ,pCfg->beaconRxTimeOut,pCfg->broadcastTimeOut,pCfg->rxBroadcastInPS,pCfg->consecutivePsPollDeliveryFailureThr));
    /*
     * Set the desired features 
     */
    return whal_hwInfoElemAcxBcnBrcOptionsSet (pHwCtrl->pHwMboxConfig, pCfg);
}

/****************************************************************************
 *                      whal_hwCtrl_wakeUpCondition()
 ****************************************************************************
 * DESCRIPTION: Configure the wlan hardware
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_wakeUpCondition(HwCtrl_T *pHwCtrl, whalCtrl_powerMgmtConfig_t *pPMConfig)
{
    WakeUpCondition_t AcxElm_WakeUpCondition;
    WakeUpCondition_t *pCfg = &AcxElm_WakeUpCondition;


    switch (pPMConfig->tnetWakeupOn)
    {
        case TNET_WAKE_ON_BEACON:
            pCfg->wakeUpConditionBitmap = WAKE_UP_EVENT_BEACON_BITMAP;
            break;
        case TNET_WAKE_ON_DTIM:
            pCfg->wakeUpConditionBitmap = WAKE_UP_EVENT_DTIM_BITMAP;
            break;
        case TNET_WAKE_ON_N_BEACON:
            pCfg->wakeUpConditionBitmap = WAKE_UP_EVENT_N_BEACONS_BITMAP;
            break;
        case TNET_WAKE_ON_N_DTIM:
            pCfg->wakeUpConditionBitmap = WAKE_UP_EVENT_N_DTIM_BITMAP;
            break;
        default:
            pCfg->wakeUpConditionBitmap = WAKE_UP_EVENT_BEACON_BITMAP;
            break;
    }

    pCfg->listenInterval        = pPMConfig->listenInterval;

    WLAN_REPORT_INFORMATION(pHwCtrl->hReport,
                            HAL_HW_CTRL_MODULE_LOG,
                            (" whal_hwCtrl_wakeUpCondition  tnetWakeupOn=0x%x listenInterval=%d\n",pCfg->wakeUpConditionBitmap,pCfg->listenInterval));
    /*
     * Set the desired features 
     */
    return whal_hwInfoElemAcxwakeUpConditionSet (pHwCtrl->pHwMboxConfig, pCfg);
}

/****************************************************************************
 *                      whal_hwCtrl_PowerMgmtConfigurationSet ()
 ****************************************************************************
 * DESCRIPTION: Set the ACX power management option IE
 * 
 * INPUTS: whalHwCtrl_powerMgmtOptionsConfig
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_PowerMgmtConfigurationSet (HwCtrl_T *pHwCtrl,
                    whalCtrl_powerSaveParams_t* powerSaveParams)

/*whalCtrl_powerMgmtConfig_t *thePowerMgmtOptionsConfig)*/
{
    whalCtrl_powerSaveParams_t AcxElm_PowerMgmtConfiguration;
    whalCtrl_powerSaveParams_t *pCfg = &AcxElm_PowerMgmtConfiguration;


        pCfg->ps802_11Enable    = powerSaveParams->ps802_11Enable;
    pCfg->hangOverPeriod        = powerSaveParams->hangOverPeriod;
        pCfg->needToSendNullData    = powerSaveParams->needToSendNullData;
    pCfg->numNullPktRetries     = powerSaveParams->numNullPktRetries;
    pCfg->powerSaveCBObject     = powerSaveParams->powerSaveCBObject;
    pCfg->powerSavecmdResponseCB = powerSaveParams->powerSavecmdResponseCB;
    /* Rate conversion is done in the HAL */
    whalUtils_ConvertAppRatesBitmap(powerSaveParams->NullPktRateModulation, 0, &(pCfg->NullPktRateModulation));


    WLAN_REPORT_INFORMATION(pHwCtrl->hReport,
                            HAL_HW_CTRL_MODULE_LOG,
                            (" whal_hwCtrl_PowerMgmtConfigurationSet  ps802_11Enable=0x%x hangOverPeriod=%d needToSendNullData=0x%x  numNullPktRetries=%d  NullPktRateModulation=0x%x\n"
                             ,pCfg->ps802_11Enable,pCfg->hangOverPeriod,pCfg->needToSendNullData,pCfg->numNullPktRetries,pCfg->NullPktRateModulation));
  
    return(whal_hwMboxCmd_PowerMgmtConfiguration (pHwCtrl->pHwMboxCmd,pCfg));
}


/****************************************************************************
 *                      whal_hwCtrl_MinPowerLevelSet ()
 ****************************************************************************
 * DESCRIPTION: Set the min power level
 * 
 * INPUTS: 
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_MinPowerLevelSet (HwCtrl_T *pHwCtrl,
                                powerAutho_PowerPolicy_e minPowerLevel)
{
    ACXSleepAuth_t AcxElm_SleepAuth;
    ACXSleepAuth_t *pCfg = &AcxElm_SleepAuth;

    /* in the info element the enums are in reverse */
    switch(minPowerLevel)
    {
        case POWERAUTHO_POLICY_ELP:
            pCfg->sleepAuth = 2;
            break;
        case POWERAUTHO_POLICY_AWAKE:
            pCfg->sleepAuth = 0;
            break;
        default:
            pCfg->sleepAuth = minPowerLevel;
    }
 
    WLAN_REPORT_INFORMATION(pHwCtrl->hReport,
                            HAL_HW_CTRL_MODULE_LOG,
                            (" whal_hwCtrl_MinPowerLevelSet  sleepAuth=%d\n",
                            minPowerLevel));

    /*
     * Set the desired min power level
     */
    return whal_hwInfoElemAcxSleepAuthoSet (pHwCtrl->pHwMboxConfig, pCfg);
}


/****************************************************************************
 *                      whal_hwCtrl_PowerMgmtOptionsPrint ()
 ****************************************************************************
 * DESCRIPTION: Print the ACX power management option
 * 
 * INPUTS:  
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_PowerMgmtOptionsPrint (HwCtrl_T *pHwCtrl)
{
    int Stt1, Stt2;
    
    ACXBeaconAndBroadcastOptions_t AcxElm_BcnBrcOptions;
    ACXBeaconAndBroadcastOptions_t *pCfgBcnBrcOptions = &AcxElm_BcnBrcOptions;
    
    ACXDtimPeriodCfg_t AcxElm_TbttAndDtim;
    ACXDtimPeriodCfg_t *pCfgTbttAndDtim = &AcxElm_TbttAndDtim;
    
    Stt1 = whal_hwInfoElemAcxBcnBrcOptionsGet (pHwCtrl->pHwMboxConfig, pCfgBcnBrcOptions);

    WLAN_REPORT_INFORMATION(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                            ("BeaconRxTimeout=0x%X\n", pCfgBcnBrcOptions->beaconRxTimeOut));

    WLAN_REPORT_INFORMATION(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                            ("BroadcastRxTimeout=0x%X\n", pCfgBcnBrcOptions->broadcastTimeOut));

    WLAN_REPORT_INFORMATION(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                            ("RxBroadcastInPs=0x%X\n", pCfgBcnBrcOptions->rxBroadcastInPS));


    Stt2 = whal_hwInfoElemDtimPeriodGet (pHwCtrl->pHwMboxConfig, 
                                         &(pCfgTbttAndDtim->dtimInterval),
                                         &(pCfgTbttAndDtim->tbtt));

    WLAN_REPORT_INFORMATION(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                            ("Time Between Beacon(TBTT)=NOT IMPLEMENTED\n"));

    WLAN_REPORT_INFORMATION(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                            ("dtimPeriod=NOT IMPLEMENTED\n"));

    if ((Stt1 == OK) && (Stt2 == OK))
        return(OK);
    else
        return(NOK);
}

/****************************************************************************
 *                      whal_hwCtrl_SetFeatureOptions()
 ****************************************************************************
 * DESCRIPTION: Configure the wlan hardware
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_SetEnergyDetection(HwCtrl_T *pHwCtrl, BOOL energyDetection)
{
    WlanParams_T *pWlanParams = whal_ParamsGetWlanParams(pHwCtrl->pWhalParams);
    UINT16 ccaThreshold = 0xefff;

    pWlanParams->RxEnergyDetection = energyDetection;
    
    if (energyDetection) 
        ccaThreshold = ACX_PHI_CCA_THRSH_ENABLE_ENERGY_D; /* enable energy detect */
    else
        ccaThreshold = ACX_PHI_CCA_THRSH_DISABLE_ENERGY_D; /* disable energy detect */
    
    whal_hwInfoElemCcaThresholdSet (pHwCtrl->pHwMboxConfig, &ccaThreshold, pWlanParams->TxEnergyDetection);

    return OK;
}


#if defined(TNETW1150)
/****************************************************************************
 *                      whal_hwCtrl_ArmClockSet()
 ****************************************************************************
 * DESCRIPTION: Configure the arm clock
 *  !!! Note that the firmware will set the slot time according to the new clock
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_ArmClockSet (HwCtrl_T *pHwCtrl, UINT32 ArmClock)
{
    WlanParams_T *pWlanParams = &pHwCtrl->pWhalParams->WlanParams;

    pWlanParams->ArmClock = ArmClock;

    WLAN_REPORT_REPLY(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwCtrl_ArmClockSet: Arm=%d (Mac=%d)\n", pWlanParams->ArmClock, pWlanParams->MacClock));

    /* Illegal combination Mac=80, Arm=40 ==> force setting 40/40*/
    if ((pWlanParams->MacClock == HW_CLOCK_80_MHZ) && (pWlanParams->ArmClock == HW_CLOCK_40_MHZ))
    {
        WLAN_REPORT_ERROR(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwCtrl_ArmClockSet: ---------- Illegal combination Mac=80, Arm=40 ==> force setting 40/40\n"));
        pWlanParams->MacClock = HW_CLOCK_40_MHZ;
    }

    return whal_hwInfoElemFeatureConfigSet (pHwCtrl->pHwMboxConfig, pWlanParams->FeatureOptions, pWlanParams->FeatureDataFlowOptions);
}
#endif

/****************************************************************************
 *                      whal_hwCtrl_MacClockSet()
 ****************************************************************************
 * DESCRIPTION: Configure the mac clock
 *  !!! Note that the firmware will set the slot time according to the new clock
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_MacClockSet (HwCtrl_T *pHwCtrl, UINT32 MacClock)
{
    WlanParams_T *pWlanParams = &pHwCtrl->pWhalParams->WlanParams;

    pWlanParams->MacClock = MacClock;

    /* force same clock - for printing */
    pWlanParams->ArmClock = MacClock;
    WLAN_REPORT_REPLY(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwCtrl_MacClockSet: Mac=%d (Arm=%d)\n", pWlanParams->MacClock, pWlanParams->ArmClock));

    /* Illegal combination Mac=80, Arm=40 ==> force setting 40/40*/
    if ((pWlanParams->MacClock == HW_CLOCK_80_MHZ) && (pWlanParams->ArmClock == HW_CLOCK_40_MHZ))
    {
        WLAN_REPORT_ERROR(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwCtrl_MacClockSet: ---------- Illegal combination Mac=80, Arm=40 ==> force setting 40/40\n"));
        pWlanParams->MacClock = HW_CLOCK_40_MHZ;
    }

    return whal_hwInfoElemFeatureConfigSet (pHwCtrl->pHwMboxConfig, pWlanParams->FeatureOptions, pWlanParams->FeatureDataFlowOptions);
}

/****************************************************************************
 *                      whal_hwCtrl_WepDefaultKeyAdd()
 ****************************************************************************
 * DESCRIPTION: Set the actual default key
 * 
 * INPUTS:  
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_WepDefaultKeyAdd (HwCtrl_T *pHwCtrl, securityKeys_t* aSecurityKey, void *CB_Func, TI_HANDLE CB_handle) 
{
    int Stt;

    char MacAddr_Dummy[6];

    Stt = whal_hwMboxCmd_SetKey(pHwCtrl->pHwMboxCmd, KEY_ADD_OR_REPLACE,
                                (char*)MacAddr_Dummy, 
                                aSecurityKey->encLen, KEY_WEP_DEFAULT,
                                aSecurityKey->keyIndex, 
                                (char*)aSecurityKey->encKey, 0, 0,
                                CB_Func, CB_handle);

    WLAN_REPORT_INFORMATION(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                            ("whal_hwCtrl_WepDefaultKeyAdd: ## len=%d, id=%d encKey[5 entries]=0x %x %x %x %x %x\n", 
                             aSecurityKey->encLen,
                             aSecurityKey->keyIndex,
                             aSecurityKey->encKey[0], aSecurityKey->encKey[1], aSecurityKey->encKey[2], aSecurityKey->encKey[3], aSecurityKey->encKey[4] ));

    return Stt;
}

/****************************************************************************
 *                      whal_hwCtrl_WepDefaultKeyRemove()
 ****************************************************************************
 * DESCRIPTION: Set the actual default key
 * 
 * INPUTS:  
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_WepDefaultKeyRemove (HwCtrl_T *pHwCtrl, securityKeys_t* aSecurityKey, void *CB_Func, TI_HANDLE CB_handle) 
{
    int Stt;

    char MacAddr_Dummy[6];

    Stt = whal_hwMboxCmd_SetKey(pHwCtrl->pHwMboxCmd, KEY_REMOVE,
                                (char*)MacAddr_Dummy, 
                                aSecurityKey->encLen, KEY_WEP_DEFAULT,
                                aSecurityKey->keyIndex, 
                                (char*)aSecurityKey->encKey, 0, 0,
                                CB_Func, CB_handle);

    WLAN_REPORT_INFORMATION(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                            ("whal_hwCtrl_WepDefaultKeyRemove: ## id=%d \n", 
                             aSecurityKey->keyIndex));

    return Stt;
}

/****************************************************************************
 *                      whal_hwCtrl_WepMappingKeyAdd()
 ****************************************************************************
 * DESCRIPTION: Set the actual mapping key
 * 
 * INPUTS:  
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_WepMappingKeyAdd (HwCtrl_T *pHwCtrl, securityKeys_t* aSecurityKey, void *CB_Func, TI_HANDLE CB_handle)
{
    int Stt;

    Stt = whal_hwMboxCmd_SetKey(pHwCtrl->pHwMboxCmd, KEY_ADD_OR_REPLACE,
                                (char*)aSecurityKey->macAddress.addr, 
                                aSecurityKey->encLen, KEY_WEP_ADDR,
                                aSecurityKey->keyIndex, 
                                (char*)aSecurityKey->encKey, 0, 0,
                                CB_Func, CB_handle);
    
    WLAN_REPORT_INFORMATION(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwCtrl_WepMappingKeyAdd: ## len=%d, id=%d encKey[5 entries]=0x %x %x %x %x %x\n", 
        aSecurityKey->encLen,
        aSecurityKey->keyIndex,
        aSecurityKey->encKey[0], aSecurityKey->encKey[1], aSecurityKey->encKey[2], aSecurityKey->encKey[3], aSecurityKey->encKey[4] ));


    return Stt;
}

/****************************************************************************
 *                      whal_hwCtrl_WepMappingKeyRemove()
 ****************************************************************************
 * DESCRIPTION: Set the actual mapping key
 * 
 * INPUTS:  
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_WepMappingKeyRemove (HwCtrl_T *pHwCtrl, securityKeys_t* aSecurityKey, void *CB_Func, TI_HANDLE CB_handle)
{
    int Stt;

    Stt = whal_hwMboxCmd_SetKey(pHwCtrl->pHwMboxCmd, KEY_REMOVE,
                                (char*)aSecurityKey->macAddress.addr, 
                                aSecurityKey->encLen, KEY_WEP_ADDR,
                                aSecurityKey->keyIndex, 
                                (char*)aSecurityKey->encKey, 0, 0,
                                CB_Func, CB_handle);

    WLAN_REPORT_INFORMATION(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwCtrl_WepMappingKeyRemove: ## id=%d \n", 
        aSecurityKey->keyIndex));

    return Stt;
}

/****************************************************************************
 *                      whal_hwCtrl_TkipMicMappingKeyAdd()
 ****************************************************************************
 * DESCRIPTION: Set the actual mapping key
 * 
 * INPUTS:  
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_TkipMicMappingKeyAdd (HwCtrl_T *pHwCtrl, securityKeys_t* aSecurityKey, void *CB_Func, TI_HANDLE CB_handle)
{
    int Stt = OK;

    UINT8   keyType;
    UINT8   keyBuffer[KEY_SIZE_TKIP];

    keyType = (IsMacAddressGroup(&(aSecurityKey->macAddress))==1) ? KEY_TKIP_MIC_GROUP: KEY_TKIP_MIC_PAIRWISE;

    os_memoryCopy(pHwCtrl->hOs, (PVOID)(&keyBuffer[0]), (PVOID)aSecurityKey->encKey, 16);
    os_memoryCopy(pHwCtrl->hOs, (PVOID)(&keyBuffer[16]), (PVOID)aSecurityKey->micRxKey, 8);
    os_memoryCopy(pHwCtrl->hOs, (PVOID)(&keyBuffer[24]), (PVOID)aSecurityKey->micTxKey, 8);

    Stt = whal_hwMboxCmd_SetKey(pHwCtrl->pHwMboxCmd, KEY_ADD_OR_REPLACE,
                                (char*)aSecurityKey->macAddress.addr, 
                                KEY_SIZE_TKIP, keyType,
                                aSecurityKey->keyIndex , 
                                (char*)keyBuffer, pHwCtrl->SecuritySeqNumLow, pHwCtrl->SecuritySeqNumHigh,
                                CB_Func, CB_handle);

    return Stt;
}

/****************************************************************************
 *                      whal_hwCtrl_TkipMappingKeyAdd()
 ****************************************************************************
 * DESCRIPTION: Set the actual mapping key
 * 
 * INPUTS:  
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_TkipMicMappingKeyRemove (HwCtrl_T *pHwCtrl, securityKeys_t* aSecurityKey, void *CB_Func, TI_HANDLE CB_handle)
{
    int Stt;
    /* UINT8 bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; */

    UINT8   keyType;

    keyType = (IsMacAddressGroup(&(aSecurityKey->macAddress))==1) ? KEY_TKIP_MIC_GROUP: KEY_TKIP_MIC_PAIRWISE;

    Stt = whal_hwMboxCmd_SetKey(pHwCtrl->pHwMboxCmd, KEY_REMOVE,
                                (char*)aSecurityKey->macAddress.addr, 
                                aSecurityKey->encLen, keyType,
                                aSecurityKey->keyIndex , 
                                (char*)aSecurityKey->encKey, 0, 0,
                                CB_Func, CB_handle);

    return Stt;
}

/****************************************************************************
 *                      whal_hwCtrl_AesMappingKeyAdd()
 ****************************************************************************
 * DESCRIPTION: Set the actual Aes mapping key
 * 
 * INPUTS:  
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
 int whal_hwCtrl_AesMappingKeyAdd    (HwCtrl_T *pHwCtrl, securityKeys_t* aSecurityKey, void *CB_Func, TI_HANDLE CB_handle)
 {
    int Stt;
    UINT8   keyType;

    keyType = IsMacAddressGroup(&(aSecurityKey->macAddress)) ? 
                                        KEY_AES_GROUP: KEY_AES_PAIRWISE;

    Stt = whal_hwMboxCmd_SetKey(pHwCtrl->pHwMboxCmd, KEY_ADD_OR_REPLACE,
        (char*)aSecurityKey->macAddress.addr, 
        aSecurityKey->encLen, keyType,
        aSecurityKey->keyIndex , 
        (char*)aSecurityKey->encKey, pHwCtrl->SecuritySeqNumLow, pHwCtrl->SecuritySeqNumHigh,
        CB_Func, CB_handle);
    
    WLAN_REPORT_INFORMATION(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                            ("whal_hwCtrl_AesMappingKeyAdd: SecuritySeqNumHigh=%ld, pHwCtrl->SecuritySeqNumLow=%ld \n", 
                             pHwCtrl->SecuritySeqNumHigh, pHwCtrl->SecuritySeqNumLow));
    

    return Stt;

 }


 /****************************************************************************
 *                      whal_hwCtrl_AesMappingKeyRemove()
 ****************************************************************************
 * DESCRIPTION: Remove  Aes mapping key
 * 
 * INPUTS:  
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
 int whal_hwCtrl_AesMappingKeyRemove    (HwCtrl_T *pHwCtrl, securityKeys_t* aSecurityKey, void *CB_Func, TI_HANDLE CB_handle)
 {
    int Stt;
    UINT8   keyType;

    keyType = IsMacAddressGroup(&(aSecurityKey->macAddress)) ? 
                                        KEY_AES_GROUP: KEY_AES_PAIRWISE;

    Stt = whal_hwMboxCmd_SetKey(pHwCtrl->pHwMboxCmd, KEY_REMOVE,
        (char*)aSecurityKey->macAddress.addr, 
        aSecurityKey->encLen, keyType,
        aSecurityKey->keyIndex , 
        (char*)aSecurityKey->encKey, 0, 0,
        CB_Func, CB_handle);
    
    return Stt;
 }

/****************************************************************************
 *                      whal_hwCtrl_DefaultKeyIdSet()
 ****************************************************************************
 * DESCRIPTION: Set the default key ID
 * 
 * INPUTS:  
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_DefaultKeyIdSet (HwCtrl_T *pHwCtrl, UINT8 aKeyIdVal, void *CB_Func, TI_HANDLE CB_handle)
{
    WLAN_REPORT_INFORMATION(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                            ("whal_hwCtrl_DefaultKeyIdSet: ## Id=%d\n", aKeyIdVal));

    WLAN_REPORT_INFORMATION(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                            ("whal_hwCtrl_DefaultKeyIdSet: ## Id=%d\n", aKeyIdVal));

    /* Set the default key Id */
    return whal_hwInfoElemWepDefaultKeyIdSet (pHwCtrl->pHwMboxConfig, &aKeyIdVal, CB_Func, CB_handle);
}

/****************************************************************************
 *                      whal_hwCtrl_DefaultKeyIdGet()
 ****************************************************************************
 * DESCRIPTION: Get the default key ID
 * 
 * INPUTS:  
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_DefaultKeyIdGet (HwCtrl_T *pHwCtrl, UINT8 *pKeyIdVal)
{
    /* Get the default key Id */
    return whal_hwInfoElemWepDefaultKeyIdGet (pHwCtrl->pHwMboxConfig, pKeyIdVal, NULL, NULL);
}

/****************************************************************************
 *                      whal_hwCtrl_Initiate()
 ****************************************************************************
 * DESCRIPTION: Download firmware code to the Hardware and run it
 * 
 * INPUTS:  None    
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
TI_STATUS whal_hwCtrl_Initiate (HwCtrl_T *pHwCtrl)
{
    WlanParams_T *pWlanParams = whal_ParamsGetWlanParams(pHwCtrl->pWhalParams);
    BootAttr_T    BootAttr;
    TI_STATUS     status;

    BootAttr.MacClock = pWlanParams->MacClock;
    BootAttr.ArmClock = pWlanParams->ArmClock;

    if ((status = whalBus_FwCtrl_Boot (pHwCtrl->hWhalBus, (TI_HANDLE)pHwCtrl, &BootAttr)) == TNETWIF_ERROR)
    {
        WLAN_REPORT_WARNING (pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                             ("whal_hwCtrl_Initiate: whalBus_FwCtrl_Boot failure!!!\n"));
    }

    return status;
}



/****************************************************************************
 *                      whal_hwCtrl_Stop()
 ****************************************************************************
 * DESCRIPTION: Stop the Hardware firmware
 * 
 * INPUTS:  None    
 * 
 * OUTPUT:  None
 * 
 * RETURNS: None
 ****************************************************************************/
void whal_hwCtrl_Stop(HwCtrl_T *pHwCtrl)
{

#ifdef WDBG_POLLING /* (!!!) ONLY FOR DEBUG WHEN THERE ARE NO INTERRUPTS */
    os_timerStop(pHwCtrl->hOs, hal_timer);
#endif

    /*
     * Stop Acx Cpu
     */
    whalBus_FwCtrl_Halt(pHwCtrl->hWhalBus);
    return;
}

/****************************************************************************
 *                      whal_hwCtrl_GetBusHandle()
 ****************************************************************************
 * DESCRIPTION: Return the handle of the Bus object.
 *
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: handle of the HwIntr object
 ****************************************************************************/
TI_HANDLE whal_hwCtrl_GetBusHandle(HwCtrl_T *pHwCtrl)
{
    return pHwCtrl->hWhalBus;
}

/****************************************************************************
 *                      whal_hwCtrl_GetMboxConfig()
 ****************************************************************************
 * DESCRIPTION: Return the handle of the MboxConfig object.
 *
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: handle of the MboxConfig object
 ****************************************************************************/
HwMboxConfig_T *whal_hwCtrl_GetMboxConfig(HwCtrl_T *pHwCtrl)
{
    return pHwCtrl->pHwMboxConfig;
}

/****************************************************************************
 *                      whal_hwCtrl_GetMboxCmd()
 ****************************************************************************
 * DESCRIPTION: Return the handle of the MboxCmd object.
 *
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: handle of the MboxCmd object
 ****************************************************************************/
HwMboxCmd_T *whal_hwCtrl_GetMboxCmd(HwCtrl_T *pHwCtrl)
{
    return pHwCtrl->pHwMboxCmd;
}

/****************************************************************************
 *                      whal_hwCtrl_StartScan()
 ****************************************************************************
 * DESCRIPTION: Send Start Scan command 
 * 
 * INPUTS: None  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_StartScan (HwCtrl_T *pHwCtrl, ScanParameters_t* pScanVals, void* ScanCommandResponseCB, TI_HANDLE CB_handle)
{
    return (whal_hwMboxCmd_StartScan (pHwCtrl->pHwMboxCmd, pScanVals, ScanCommandResponseCB,CB_handle));
}

/****************************************************************************
 *                      whal_hwCtrl_StartSPSScan()
 ****************************************************************************
 * DESCRIPTION: Send Start SPS Scan command 
 * 
 * INPUTS: None  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_StartSPSScan( HwCtrl_T *pHwCtrl, ScheduledScanParameters_t* pScanVals, void* ScanCommandResponseCB, TI_HANDLE CB_handle)
{
    return (whal_hwMboxCmd_StartSPSScan (pHwCtrl->pHwMboxCmd, pScanVals, ScanCommandResponseCB, CB_handle));
}

/****************************************************************************
 *                      whal_hwCtrl_StopScan()
 ****************************************************************************
 * DESCRIPTION: Send Stop Scan command 
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_StopScan (HwCtrl_T *pHwCtrl, void *ScanCommandResponseCB, TI_HANDLE CB_handle)
{
    return(whal_hwMboxCmd_StopScan (pHwCtrl->pHwMboxCmd, ScanCommandResponseCB, CB_handle));
}

/****************************************************************************
 *                      whal_hwCtrl_StopSPSScan()
 ****************************************************************************
 * DESCRIPTION: Send Stop SPS Scan command 
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_StopSPSScan (HwCtrl_T *pHwCtrl, void* ScanCommandResponseCB, TI_HANDLE CB_handle)
{
    return(whal_hwMboxCmd_StopSPSScan (pHwCtrl->pHwMboxCmd, ScanCommandResponseCB, CB_handle));
}

/****************************************************************************
 *                      whal_hwCtrl_GenCmd()
 ****************************************************************************
 * DESCRIPTION: Send any command to hw MB command
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_GenCmd(HwCtrl_T *pHwCtrl, short CmdID, char* pBuf, UINT32 Length)
{
    return (whal_hwMboxCmd_GenCmd(pHwCtrl->pHwMboxCmd, CmdID, pBuf, Length));
}

/****************************************************************************
 *                      whal_hwCtrl_isElpSupported ()
 ****************************************************************************
 * DESCRIPTION: Check if ELP feature is supported based on the HW device
 *              
 * INPUTS: 
 * 
 * OUTPUT:  
 * 
 * RETURNS: ELP feature is supported/not
 ****************************************************************************/
BOOL whal_hwCtrl_isElpSupported (HwCtrl_T *pHwCtrl)
{
    return TRUE;
}

/****************************************************************************
 *                      whal_hwCtrl_AidSet()
 ****************************************************************************
 * DESCRIPTION: Set the AID
 * 
 * INPUTS:  
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_AidSet (HwCtrl_T *pHwCtrl, UINT16 aAidVal)
{
    /* Set the Aid */
    return whal_hwInfoElemAidSet (pHwCtrl->pHwMboxConfig, &aAidVal);
}


/****************************************************************************
 *                      whal_hwCtrl_CurrentTxRxAntennaSendCmd()
 ****************************************************************************
 * DESCRIPTION: send Diversity command to F/W with the pre-stored antenna 
 *              diversity parameters 
 *
 * INPUTS:
 *
 * OUTPUT:
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_CurrentAntennaDiversitySendCmd (HwCtrl_T *pHwCtrl)
{
    int status;

    WLAN_REPORT_INFORMATION(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,("whal_hwCtrl_CurrentRxAntennaSet\n"));
    
    /* Write the current antenna diversity values to the HW*/
    if ( RADIO_RADIA_DCR_ID == pHwCtrl->pWhalParams->WlanParams.radioType )
    {
        status = whal_hwInfoElemAntennaDiversitySet (pHwCtrl->pHwMboxConfig, 
                                                  &(pHwCtrl->pWhalParams->WlanParams.antennaDiversityOptions),
                                                  1);
    }
    else
    {
        status = whal_hwInfoElemAntennaDiversitySet (pHwCtrl->pHwMboxConfig, 
                                                  &(pHwCtrl->pWhalParams->WlanParams.antennaDiversityOptions),
                                                  2);
    }

    return(status);
}

/****************************************************************************
 *                      whal_hwCtrl_SetTxAntenna()
 ****************************************************************************
 * DESCRIPTION: Save TX antenna
 *
 * INPUTS:
 *
 * OUTPUT:
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_SetTxAntenna (HwCtrl_T *pHwCtrl, UINT8 TxAntenna)
{
    if (TxAntenna == 1)
    {
        pHwCtrl->pWhalParams->WlanParams.antennaDiversityOptions.txSelectedAntenna = DIVS_TX_START_ANT1;
    }
    else if (TxAntenna == 2)
    {
        pHwCtrl->pWhalParams->WlanParams.antennaDiversityOptions.txSelectedAntenna = DIVS_TX_START_ANT2;
    }
    else
    {
        WLAN_REPORT_ERROR(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                          ("whal_hwCtrl_SetTxAntenna: wrong antenna param %d\n", TxAntenna));
        return PARAM_VALUE_NOT_VALID;
    }

    return OK;
}

/****************************************************************************
 *                      whal_hwCtrl_GetTxAntenna()
 ****************************************************************************
 * DESCRIPTION: retrieve TX antenna
 *
 * INPUTS:
 *
 * OUTPUT:
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_GetTxAntenna (HwCtrl_T *pHwCtrl, UINT8* TxAntenna)
{
    if (pHwCtrl->pWhalParams->WlanParams.antennaDiversityOptions.txSelectedAntenna == DIVS_RX_START_ANT1)
    {
        *TxAntenna = 1;
    }
    else if (pHwCtrl->pWhalParams->WlanParams.antennaDiversityOptions.txSelectedAntenna == DIVS_RX_START_ANT2)
    {
        *TxAntenna = 2;
    }
    else
    {
        WLAN_REPORT_ERROR(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                          ("whal_hwCtrl_GetTxAntenna: wrong configured antenna param %d\n", pHwCtrl->pWhalParams->WlanParams.antennaDiversityOptions.txSelectedAntenna));
        return CONFIGURATION_NOT_VALID;
    }

    return OK;
}

/****************************************************************************
 *                      whal_hwCtrl_SetRxAntenna()
 ****************************************************************************
 * DESCRIPTION: Save RX antenna
 *
 * INPUTS:
 *
 * OUTPUT:
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_SetRxAntenna (HwCtrl_T *pHwCtrl, UINT8 RxAntenna)
{
    if (RxAntenna == 1)
    {
        pHwCtrl->pWhalParams->WlanParams.antennaDiversityOptions.rxSelectedAntenna = DIVS_RX_START_ANT1;
    }
    else if (RxAntenna == 2)
    {
        pHwCtrl->pWhalParams->WlanParams.antennaDiversityOptions.rxSelectedAntenna = DIVS_RX_START_ANT2;
    }
    else
    {
        WLAN_REPORT_ERROR(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                          ("whal_hwCtrl_SetRxAntenna: wrong antenna param %d\n", RxAntenna));
        return PARAM_VALUE_NOT_VALID;
    }

    return OK;
}

/****************************************************************************
 *                      whal_hwCtrl_GetRxAntenna()
 ****************************************************************************
 * DESCRIPTION: retrieve RX antenna
 *
 * INPUTS:
 *
 * OUTPUT:
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_GetRxAntenna (HwCtrl_T *pHwCtrl, UINT8* RxAntenna)
{
    if (pHwCtrl->pWhalParams->WlanParams.antennaDiversityOptions.rxSelectedAntenna == DIVS_RX_START_ANT1)
    {
        *RxAntenna = 1;
}
    else if (pHwCtrl->pWhalParams->WlanParams.antennaDiversityOptions.rxSelectedAntenna == DIVS_RX_START_ANT2)
    {
        *RxAntenna = 2;
    }
    else
    {
        WLAN_REPORT_ERROR(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                          ("whal_hwCtrl_GetRxAntenna: wrong configured antenna param %d\n", pHwCtrl->pWhalParams->WlanParams.antennaDiversityOptions.rxSelectedAntenna));
        return CONFIGURATION_NOT_VALID;
    }

    return OK;
}

/****************************************************************************
 *                      whal_hwCtrl_SaveAntennaDiversityOptions()
 ****************************************************************************
 * DESCRIPTION: Save antenna diversity parameters
 *
 * INPUTS:
 *
 * OUTPUT:
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_SaveAntennaDiversityOptions (HwCtrl_T *pHwCtrl, 
                                             whalCtrl_antennaDiversityOptions_t* pAntennaDivresityOptions )
{
    os_memoryCopy( pHwCtrl->hOs, (void*)&(pHwCtrl->pWhalParams->WlanParams.antennaDiversityOptions),
                   (void*)pAntennaDivresityOptions, sizeof( whalCtrl_antennaDiversityOptions_t ) );
    whal_hwCtrl_SetTxAntenna(pHwCtrl, pAntennaDivresityOptions->txSelectedAntenna);
    whal_hwCtrl_SetRxAntenna(pHwCtrl, pAntennaDivresityOptions->rxSelectedAntenna);
    return OK;
}

/****************************************************************************
 *                      whal_hwCtrl_CurrentAssociationIdGet()
 ****************************************************************************
 * DESCRIPTION: Get the current TX antenna 
 * 
 * INPUTS:  
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_CurrentAssociationIdGet (HwCtrl_T *pHwCtrl, UINT16  *pAidVal)
{
    *pAidVal = pHwCtrl->pWhalParams->WlanParams.Aid;
    return OK;
}

/****************************************************************************
 *                      whal_hwCtrl_OverridePhyRegsDefaults()
 ****************************************************************************
 * DESCRIPTION: Set phy register for short preamble problem
 * 
 * INPUTS:  
 * 
 * OUTPUT:  
 * 
 * RETURNS: 
 ****************************************************************************/
void whal_hwCtrl_OverridePhyRegsDefaults(HwCtrl_T *pHwCtrl)
{
    /* 
     * Configure the energy detection 
     */
    whal_hwCtrl_SetEnergyDetection(pHwCtrl, pHwCtrl->pWhalParams->WlanParams.RxEnergyDetection);

    /* 
     * Disable OFDM receiver in channel 14 (overcome FCS errors problem) 
     */
    /* moved to the firmware */
}


int whal_hwCtrl_EncDecrSet (HwCtrl_T *pHwCtrl, BOOL aHwEncEnable, BOOL aHwDecEnable)
{
    WlanParams_T *pWlanParams = &pHwCtrl->pWhalParams->WlanParams;

    if (aHwEncEnable)
        pWlanParams->FeatureDataFlowOptions &= ~DF_ENCRYPTION_DISABLE;
    else
        pWlanParams->FeatureDataFlowOptions |= DF_ENCRYPTION_DISABLE;

    /* Set bit DF_SNIFF_MODE_ENABLE to enable or prevent decryption in fw */
    /* WARNING: Have to check how to control the decryption (which bit) and then set/reset
                the  appropriate bit*/ 
    if (aHwDecEnable)
        pWlanParams->FeatureDataFlowOptions &= ~DF_SNIFF_MODE_ENABLE;
    else
        pWlanParams->FeatureDataFlowOptions |= DF_SNIFF_MODE_ENABLE;

    return whal_hwInfoElemFeatureConfigSet (pHwCtrl->pHwMboxConfig, pWlanParams->FeatureOptions, pWlanParams->FeatureDataFlowOptions);
}

int whal_hwCtrl_ClkRunEnableSet (HwCtrl_T *pHwCtrl, BOOL aClkRunEnable)
{
    WlanParams_T *pWlanParams = &pHwCtrl->pWhalParams->WlanParams;

    if (aClkRunEnable)
        pWlanParams->FeatureDataFlowOptions |= FEAT_PCI_CLK_RUN_ENABLE;
    else
        pWlanParams->FeatureDataFlowOptions &= ~FEAT_PCI_CLK_RUN_ENABLE;

    return whal_hwInfoElemFeatureConfigSet (pHwCtrl->pHwMboxConfig, pWlanParams->FeatureOptions, pWlanParams->FeatureDataFlowOptions);
}

int whal_hwCtrl_RxMsduFormatSet (HwCtrl_T *pHwCtrl, BOOL aRxMsduForamtEnable)
{
#if 1
    /* WARNING:  Have to check how to control the Rx Frame format select (which bit)
                 and then access the HW*/
    return(OK);
#else
    WlanParams_T *pWlanParams = &pHwCtrl->pWhalParams->WlanParams;
    if (aRxMsduForamtEnable)
        pWlanParams->FeatureDataFlowOptions |= DATA_FLOW_RX_MSDU_FRAME;
    else
        pWlanParams->FeatureDataFlowOptions &= ~DATA_FLOW_RX_MSDU_FRAME;

    return whal_hwInfoElemFeatureConfigSet (pHwCtrl->pHwMboxConfig, pWlanParams->FeatureOptions, pWlanParams->FeatureDataFlowOptions);
#endif
}

/****************************************************************************
 *                      whal_hwCtrl_getTsf()
 ****************************************************************************
 * DESCRIPTION: Get the current time stamp from the FW
 * 
 * INPUTS:  hwHalCtrl handle, pTsf container for the FW mac timer
 * 
 * OUTPUT:  pTsf FW mac timer
 * 
 * RETURNS: OK, NOK
 *
 * NOTES: The time will be in usec
 ****************************************************************************/

int whal_hwCtrl_getTsf(HwCtrl_T *pHwCtrl, UINT32 *pTsf)
{
    /* for debug only - Not implemented as direct access to register */
    return(OK);
}


/****************************************************************************
 *                      whal_hwCtrl_NoiseHistogramCmd()
 ****************************************************************************
 * DESCRIPTION: Send Noise Histogram command 
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_NoiseHistogramCmd (HwCtrl_T *pHwCtrl, whalCtrl_noiseHistogram_t* pNoiseHistParams)
{
    return (whal_hwMboxCmd_NoiseHistogramCmd (pHwCtrl->pHwMboxCmd, pNoiseHistParams));
}

/****************************************************************************
 *                      whal_hwCtrl_TrafficConf()
 ****************************************************************************
 * DESCRIPTION: configure Queue traffic params
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int  whal_hwCtrl_TrafficConf(TI_HANDLE hHwCtrl, queueTrafficParams_t *pQtrafficParams)
{
    TI_STATUS   status;
    HwCtrl_T *pHwCtrl = (HwCtrl_T*)hHwCtrl;


    WLAN_REPORT_DEBUG_CONTROL(pHwCtrl->hReport,(" whal_hwCtrl_TrafficConf: pQtrafficParams->aQueueId = 0x%x , pQtrafficParams->channelType %x pQtrafficParams->tsid %d pQtrafficParams->dot11EDCATableMSDULifeTime %d \n",
    pQtrafficParams->queueID,pQtrafficParams->channelType,pQtrafficParams->tsid,pQtrafficParams->dot11EDCATableMSDULifeTime));

    WLAN_REPORT_DEBUG_CONTROL(pHwCtrl->hReport,("whal_hwCtrl_TrafficConf : psScheme = 0x%x , ackPolicy %d APSDConf[0] = 0x%x ,APSDConf[1] = 0x%x\n",pQtrafficParams->psScheme,pQtrafficParams->ackPolicy,pQtrafficParams->APSDConf[0],pQtrafficParams->APSDConf[1]));


    /* Setting the queue configuration into the HW */
    status = (TI_STATUS)whal_hwInfoElemQueueConfigurationSet (pHwCtrl->pHwMboxConfig,pQtrafficParams);  

    /* Set the queue param object database fields according to the succeded configuration (for recovery) */
    if (status == OK)
        whal_ParamsSetQueueParams(pHwCtrl->pWhalParams,pQtrafficParams);

    return status;
}
/****************************************************************************
 *                      whal_hwCtrl_AcParamsConf()
 ****************************************************************************
 * DESCRIPTION: configure AC params
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/

int whal_hwCtrl_AcParamsConf(TI_HANDLE hHwCtrl,configureCmdCBParams_t *pConfigureCommand)
{
    
    TI_STATUS   status;
    HwCtrl_T *pHwCtrl = (HwCtrl_T*)hHwCtrl;
    acQosParams_t *pAcQosParams = (acQosParams_t*)(pConfigureCommand->CB_buf);

    WLAN_REPORT_DEBUG_CONTROL(pHwCtrl->hReport,(" whal_hwCtrl_AcParamsConf: Index = 0x%x, aCwMin = 0x%x, aCwMax = 0x%x, aAIFS = 0x%x, aTxOpLimit = 0x%x\n",
    pAcQosParams->ac,pAcQosParams->cwMin,pAcQosParams->cwMax,pAcQosParams->aifsn,pAcQosParams->txopLimit));

    /* Setting the AC configuration into the HW */

    if (pConfigureCommand->CB_Func == NULL)    
        status = (TI_STATUS)whal_hwInfoElemAcParamsConfigurationSet (pHwCtrl->pHwMboxConfig,pConfigureCommand);  
    else
        status = (TI_STATUS)whal_hwInfoElemAcParamsConfigurationGet (pHwCtrl->pHwMboxConfig,pConfigureCommand);  

    /* Set the AC param object database fields according to the succeeded configuration (for recovery) */
    if (status == OK)
        whal_ParamsSetAcParams(pHwCtrl->pWhalParams,pAcQosParams);

    return status;

}
/****************************************************************************
 *                      whal_hwCtrl_AccessCategoryConf()
 ****************************************************************************
 * DESCRIPTION: Send Access Category Configuration 
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int  whal_hwCtrl_QueueConf(TI_HANDLE hHwCtrl, acQueuesParams_t* pAcQueuesParams)
{
    TI_STATUS   status;
    HwCtrl_T *pHwCtrl = (HwCtrl_T*)hHwCtrl;

    /* Setting the queue configuration into the HW */
    status = (TI_STATUS)whal_hwInfoElemTxQueueCfgSet (pHwCtrl->pHwMboxConfig,
                                             pAcQueuesParams, 
                                           pHwCtrl->MemMap.numTxMemBlks);

    /* Set the queue param object database fields according to the succeeds configuration (for recovery) */
    if (status == OK)
        whal_ParamsSetAccessCategoryParams(pHwCtrl->pWhalParams, pAcQueuesParams);
 
    return status;
}

/****************************************************************************
 *                      whal_hwCtrl_PacketDetectionThreshold()
 ****************************************************************************
 * DESCRIPTION: Send Noise Histogram command 
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_PacketDetectionThreshold (TI_HANDLE hHwCtrl, UINT8* pPdThreshold)
{
    TI_STATUS   status;
    HwCtrl_T    *pHwCtrl = (HwCtrl_T*)hHwCtrl;
    UINT32      packetDetection = *pPdThreshold;

    /* Setting the queue configuration into the HW */
    status = (TI_STATUS)whal_hwInfoElemPacketDetectionThresholdSet (pHwCtrl->pHwMboxConfig, &packetDetection);  
    
    return status;
}



/****************************************************************************
 *                     whal_hwCtrl_SetBeaconFiltering
 ****************************************************************************
 * DESCRIPTION: Sets Beacon filtering state 
 * 
 * INPUTS:  None    
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_SetBeaconFiltering(HwCtrl_T *pHwCtrl, UINT8 beaconFilteringStatus, UINT8 numOfBeaconsToBuffer)
{
    ACXBeaconFilterOptions_t AcxElm_BeaconFilterOptions;
    ACXBeaconFilterOptions_t *pCfg = &AcxElm_BeaconFilterOptions;
    
    pCfg->enable = beaconFilteringStatus;
    pCfg->maxNumOfBeaconsStored = numOfBeaconsToBuffer;

    WLAN_REPORT_DEBUG_CONTROL(pHwCtrl->hReport , ("Set beacon filter to %d" , beaconFilteringStatus) ) ;
    
    return whal_hwInfoElemAcxBeaconFilterOptionsSet (pHwCtrl->pHwMboxConfig, pCfg);
}


/****************************************************************************
 *                     whal_hwCtrl_SetBeaconFilterIETable
 ****************************************************************************
 * DESCRIPTION: Sets Beacon filtering state 
 * 
 * INPUTS:  None    
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_SetBeaconFilterIETable(HwCtrl_T *pHwCtrl, UINT8* numberOfIEs, UINT8 * IETable, UINT8* IETableSize)
{
    int counter = 0 ;
    if ( NULL == pHwCtrl)
    {
        return PARAM_VALUE_NOT_VALID ;
    }

    WLAN_REPORT_DEBUG_CONTROL(pHwCtrl->hReport,
                             ("\n  whal_hwCtrl_SetBeaconFilterIETable Beacon IE Table:\n"));
        
    for ( counter = 0 ; counter < * IETableSize ; counter++)
    {
        WLAN_REPORT_DEBUG_CONTROL(pHwCtrl->hReport,
                                 ("%2.x " , IETable[counter]));
    }

        WLAN_REPORT_DEBUG_CONTROL(pHwCtrl->hReport,
                                 ("\n "));


    return whal_hwInfoElemAcxBeaconFilterIETableSet (pHwCtrl->pHwMboxConfig,
                                                     numberOfIEs, 
                                                     IETable, 
                                                     IETableSize);
}

 
/****************************************************************************
 *                      whal_HwCtrl_enableMboxAsyncMode()
 ****************************************************************************
 * DESCRIPTION: 
 * 
 * INPUTS:  
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK, NOK
 *
 * NOTES: 
 ****************************************************************************/
int whal_HwCtrl_enableMboxAsyncMode(HwCtrl_T *pHwCtrl)
{
    eventMbox_InitComplete(pHwCtrl->hEventMbox);
    return OK;
}

/****************************************************************************
 *                      whal_HwCtrl_resetMacRx()
 ****************************************************************************
 * DESCRIPTION: Reset the Rx Max module
 *
 * INPUTS:
 *
 * OUTPUT:  
 *
 * RETURNS: OK, NOK
 *
 * NOTES:
 ****************************************************************************/
int whal_HwCtrl_resetMacRx(HwCtrl_T *pHwCtrl)
{
    return whal_hwMboxCmd_RxReset(pHwCtrl->pHwMboxCmd);
}

/****************************************************************************
 *                      whal_HwCtrl_LNAControl()
 ****************************************************************************
 * DESCRIPTION: Control the LNA (On <-> Off)
 * 
 * INPUTS:  
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK, NOK
 *
 * NOTES: 
 ****************************************************************************/
int whal_HwCtrl_LNAControl(HwCtrl_T *pHwCtrl, UINT8 LNAControlField)
{
    return whal_hwMboxCmd_LNAControl(pHwCtrl->pHwMboxCmd, LNAControlField);
}

/****************************************************************************
 *                      whal_hwCtrl_SwitchChannelCmd()
 ****************************************************************************
 * DESCRIPTION: Send Switch Channel command 
 * 
 * INPUTS: None  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_SwitchChannelCmd (HwCtrl_T *pHwCtrl, whalCtrl_switchChannelCmd_t* pSwitchChannelCmd)
{
    WLAN_REPORT_INFORMATION(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwCtrl_SwitchChannelCmd\n"));
    return (whal_hwMboxCmd_SwitchChannelCmd (pHwCtrl->pHwMboxCmd, pSwitchChannelCmd));
}

/****************************************************************************
 *                      whal_hwCtrl_SwitchChannelCmd()
 ****************************************************************************
 * DESCRIPTION: Send Switch Channel command 
 * 
 * INPUTS: None  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_SwitchChannelCancelCmd (HwCtrl_T *pHwCtrl)
{
    WLAN_REPORT_INFORMATION(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwCtrl_SwitchChannelCmd\n"));
    return (whal_hwMboxCmd_SwitchChannelCancelCmd (pHwCtrl->pHwMboxCmd));
}


/*----------------------------------------*/
/* Roaming Trigger Configuration Commands */
/*----------------------------------------*/

/****************************************************************************
 *                      whal_hwCtrl_SetSNRParameters()
 ****************************************************************************
 * DESCRIPTION: Set SNR parameters.
 *
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: None
 ****************************************************************************/
int whal_hwCtrl_SetSNRParams(HwCtrl_T *pHwCtrl, whalCtrl_roamingTriggerCmd_t* pRoamingTriggerCmd) 
{
    ACXLowSNRTriggerParameters_t AcxElm_LowThresholdOptions;
    AcxElm_LowThresholdOptions.SNRThreshold         = pRoamingTriggerCmd->snrThreshold ;        
    AcxElm_LowThresholdOptions.SNRFilterWeight      = pRoamingTriggerCmd->snrFilterWeight ; 
    AcxElm_LowThresholdOptions.SNRFilterDepth       = pRoamingTriggerCmd->snrFilterDepth ;      
    AcxElm_LowThresholdOptions.LowSNREventType  = pRoamingTriggerCmd->lowSNREventType;
    
    
    return whal_hwInfoElemAcxLowSNRThresholdSet (pHwCtrl->pHwMboxConfig, &AcxElm_LowThresholdOptions);
}

/****************************************************************************
 *                      whal_hwCtrl_SetRSSIParameters()
 ****************************************************************************
 * DESCRIPTION: Set RSSI parameters used by the TNET for its calulation 
 *               that is used for generating of RSSI cross threshold interrupts.
 *
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: None
 ****************************************************************************/
int whal_hwCtrl_SetRSSIParams(HwCtrl_T *pHwCtrl, whalCtrl_roamingTriggerCmd_t* pRoamingTriggerCmd) 
{
    ACXLowRSSITriggerParameters_t AcxElm_LowRSSIThresholdOptions;
    ACXLowRSSITriggerParameters_t *pCfg = &AcxElm_LowRSSIThresholdOptions;

    pCfg->rssiFilterDepth   = pRoamingTriggerCmd->rssiFilterDepth;
    pCfg->rssiFilterWeight  = pRoamingTriggerCmd->rssiFilterWeight;
    pCfg->rssiThreshold     = pRoamingTriggerCmd->rssiThreshold;
    pCfg->LowRSSIEventType  = pRoamingTriggerCmd->lowRSSIEventType;
    
    return whal_hwInfoElemAcxLowRSSIThresholdSet (pHwCtrl->pHwMboxConfig, pCfg);
}

/****************************************************************************
 *                      whal_hwCtrl_SetMaxTxRetryParameters()
 ****************************************************************************
 * DESCRIPTION: Set Max Tx retry parmaters.
 *
 * INPUTS:  
 *      maxTxRetry             max Tx Retry
 *
 * OUTPUT:  None
 *
 * RETURNS: None
 ****************************************************************************/
int whal_hwCtrl_SetMaxTxRetryParams(HwCtrl_T *pHwCtrl, whalCtrl_roamingTriggerCmd_t* pRoamingTriggerCmd)
{
    ACXConsTxFailureTriggerParameters_t AcxElm_MaxTxRetry;
    ACXConsTxFailureTriggerParameters_t *pCfg = &AcxElm_MaxTxRetry;

    pCfg->maxTxRetry    = pRoamingTriggerCmd->maxTxRetry;
    
    return whal_hwInfoElemAcxSetMaxTxRetrySet (pHwCtrl->pHwMboxConfig, pCfg);
}


/****************************************************************************
 *                      whal_hwCtrl_GetAsynRSSI ()
 ****************************************************************************
 * DESCRIPTION: Get the Average RSSI
 * 
 * INPUTS:  
 * 
 * OUTPUT:  
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_GetAsynRSSI (HwCtrl_T *pHwCtrl,void *CB_Func, TI_HANDLE CB_handle, PUINT8 CB_Buf)
{
    int Stt;

    Stt = whal_hwInfoElemRSSIGet (pHwCtrl->pHwMboxConfig, CB_Func, CB_handle, CB_Buf);
                                         
    return Stt;
}



/****************************************************************************
 *                      whal_hwCtrl_SetBssLossTsfThresholdParams()
 ****************************************************************************
 * DESCRIPTION: 
 *              
 *              
 *
 * INPUTS:  
 *
 * OUTPUT:  None
 *
 * RETURNS: None
 ****************************************************************************/
int whal_hwCtrl_SetBssLossTsfThresholdParams(   HwCtrl_T *pHwCtrl,whalCtrl_roamingTriggerCmd_t* pRoamingTriggerCmd) 
{
    AcxConnectionMonitorOptions AcxElm_BssLossTsfSynchronize;
    AcxConnectionMonitorOptions *pCfg = &AcxElm_BssLossTsfSynchronize;

    pCfg->BSSLossTimeout        = pRoamingTriggerCmd->BssLossTimeout;
    pCfg->TSFMissedThreshold    = pRoamingTriggerCmd->TsfMissThreshold;
    
    return whal_hwInfoElemAcxBssLossTsfThresholdSet (pHwCtrl->pHwMboxConfig, pCfg);
}

/****************************************************************************
 *                      whal_hwCtrl_FwDisconnect()
 ****************************************************************************
 * DESCRIPTION: Disconnect. 
 * 
 * INPUTS: None
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_FwDisconnect(HwCtrl_T *pHwCtrl, uint32 ConfigOptions, uint32 FilterOptions)
{
    HwMboxCmd_T *pHwMboxCmd = pHwCtrl->pHwMboxCmd;

    return whal_hwMboxCmd_FwDisconnect(pHwMboxCmd, ConfigOptions, FilterOptions);

} /* whal_hwCtrl_FwDisconnect()*/




/****************************************************************************
 *                      whal_hwCtrl_measurementParams()
 ****************************************************************************
 * DESCRIPTION: send Command for measurement configuration 
 *              to the mailbox
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_measurement (HwCtrl_T *pHwCtrl, whalCtrl_MeasurementParameters_t* pMeasurementParams,
                             void* MeasureCommandResponseCB, TI_HANDLE CB_handle)
{
    WLAN_REPORT_INFORMATION(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwCtrl_measurementParams\n"));
    return whal_hwMboxCmd_measurement (pHwCtrl->pHwMboxCmd, pMeasurementParams, 
                                       MeasureCommandResponseCB, CB_handle);
}


/****************************************************************************
 *                      whal_hwCtrl_measurementStop()
 ****************************************************************************
 * DESCRIPTION: send Command for stoping measurement  
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_measurementStop (HwCtrl_T *pHwCtrl, void* MeasureCommandResponseCB, TI_HANDLE CB_handle)
{
    WLAN_REPORT_INFORMATION(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwCtrl_measurementStop\n"));

    return whal_hwMboxCmd_measurementStop (pHwCtrl->pHwMboxCmd, MeasureCommandResponseCB, CB_handle);
}

/****************************************************************************
 *                      whal_hwCtrl_ApDiscovery()
 ****************************************************************************
 * DESCRIPTION: send Command for AP Discovery 
 *              to the mailbox
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_ApDiscovery (HwCtrl_T *pHwCtrl, whalCtrl_ApDiscoveryParameters_t* pApDiscoveryParams)

{
    WLAN_REPORT_INFORMATION(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwCtrl_ApDiscovery\n"));

    return( whal_hwMboxCmd_ApDiscovery (pHwCtrl->pHwMboxCmd, pApDiscoveryParams));

}
/****************************************************************************
 *                      whal_hwCtrl_ApDiscoveryStop()
 ****************************************************************************
 * DESCRIPTION: send Command for stoping AP Discovery
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_ApDiscoveryStop (HwCtrl_T *pHwCtrl)


{
    WLAN_REPORT_INFORMATION(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwCtrl_ApDiscoveryStop\n"));
    
    return(whal_hwMboxCmd_ApDiscoveryStop (pHwCtrl->pHwMboxCmd));
}

/****************************************************************************
 *                      whal_hwCtrl_healthCheck()
 ****************************************************************************
 * DESCRIPTION: 
 * 
 * INPUTS:  
 * 
 * OUTPUT:  
 * 
 * RETURNS: 
 ****************************************************************************/
int whal_hwCtrl_healthCheck (HwCtrl_T *pHwCtrl)
{
    WLAN_REPORT_INFORMATION(pHwCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwMboxCmd_HealthCheck\n"));
    
    return(whal_hwMboxCmd_HealthCheck(pHwCtrl->pHwMboxCmd));
}


/****************************************************************************
 *                      whal_hwCtrl_SoftGeminiEnable()
 ****************************************************************************
 * DESCRIPTION: Save Soft Gemini enable parameter
 *
 * INPUTS:
 *
 * OUTPUT:
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_SoftGeminiEnable (HwCtrl_T *pHwCtrl, 
                                             SoftGeminiEnableModes_e SgEnable )
{
    /* copy params for recovery */
    pHwCtrl->pWhalParams->WlanParams.SoftGeminiEnable = SgEnable;
                    
    return whal_hwInfoElemSoftGeminiEnableSet (pHwCtrl->pHwMboxConfig, SgEnable);
}

/****************************************************************************
 *                      whal_hwCtrl_SetSoftGeminiParams()
 ****************************************************************************
 * DESCRIPTION: Save Soft Gemini config parameter
 *
 * INPUTS:
 *
 * OUTPUT:
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_SetSoftGeminiParams (HwCtrl_T *pHwCtrl, 
                                             SoftGeminiParam_t *SgParam )
{
    /* copy params for recovery */
    os_memoryCopy(pHwCtrl->hOs,(void*)&pHwCtrl->pWhalParams->WlanParams.SoftGeminiParams,
                    (void*)SgParam,sizeof(SoftGeminiParam_t));
                  
    return whal_hwInfoElemSoftGeminiParamsSet (pHwCtrl->pHwMboxConfig,SgParam);
}

/****************************************************************************
 *                      whal_hwCtrl_GetSoftGeminiParams()
 ****************************************************************************
 * DESCRIPTION: Get Soft Gemini config parameter
 *
 * INPUTS:
 *
 * OUTPUT:
 *
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_GetSoftGeminiParams (HwCtrl_T *pHwCtrl, void *CB_Func, TI_HANDLE CB_handle, void* CB_Buf)
{                  
    return whal_hwInfoElemSoftGeminiParamsGet (pHwCtrl->pHwMboxConfig, CB_Func, CB_handle, CB_Buf);
}
/****************************************************************************
 *                      whal_hwCtrl_GxRatePolicy()
 ****************************************************************************
 * DESCRIPTION: Get TxRatePolicy params
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: txRatePolicy_t* - the TX rate policy
 ****************************************************************************/

txRatePolicy_t* whal_hwCtrl_GetTxRatePolicy(TI_HANDLE hHwCtrl)
{
    HwCtrl_T *pHwCtrl = (HwCtrl_T*)hHwCtrl;
    return whal_ParamsGetTxRateClassParams(pHwCtrl->pWhalParams);
}


/****************************************************************************
 *                      whal_hwCtrl_TxRatePolicy()
 ****************************************************************************
 * DESCRIPTION: configure TxRatePolicy params
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/

int whal_hwCtrl_TxRatePolicy(TI_HANDLE hHwCtrl,txRatePolicy_t *pTxRatePolicy)
{
    TI_STATUS   status = NOK;
    HwCtrl_T *pHwCtrl = (HwCtrl_T*)hHwCtrl;

    UINT8 index;
    txRateClass_t *pTxRateClass = pTxRatePolicy->rateClass;

    WLAN_REPORT_DEBUG_CONTROL(pHwCtrl->hReport,
        ("%s_1, Num of classes = 0x%x\n",__FUNCTION__, pTxRatePolicy->numOfRateClasses));   

    for(index = 0; index < pTxRatePolicy->numOfRateClasses; index++, pTxRateClass++)
    {
            WLAN_REPORT_DEBUG_CONTROL(pHwCtrl->hReport,
                            ("%s_2loop, Index = %d, Short R = 0x%x, Long R = 0x%x, Flags = 0x%x Rates(HexDump) = \n",
                            __FUNCTION__, index, 
                            pTxRateClass->shortRetryLimit, pTxRateClass->longRetryLimit, pTxRateClass->flags));
             
            WLAN_REPORT_HEX_INFORMATION(pHwCtrl->hReport,
                            HAL_HW_CTRL_MODULE_LOG,
                            (PUINT8)pTxRateClass->txRate, sizeof(pTxRateClass->txRate));
    }

    /* Setting Rate Policy configuration into the HW */
    status = (TI_STATUS)whal_hwInfoElemTxRatePolicyConfigurationSet (pHwCtrl->pHwMboxConfig, pTxRatePolicy);  

    /* Set the Policy param object database fields according to the succeeded configuration (for recovery) */
    if (status == OK)
    {
            whal_ParamsSetTxRateClassParams(pHwCtrl->pWhalParams,(txRatePolicy_t *)pTxRatePolicy);
    }

    return status;
}


/****************************************************************************
 *                      whal_hwCtrl_ReJoinBss()
 ****************************************************************************
 * DESCRIPTION: 
 * 
 * INPUTS: None 
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_ReJoinBss (TI_HANDLE hHwCtrl)
{
    HwCtrl_T *pHwCtrl = (HwCtrl_T*)hHwCtrl;
    BssInfoParams_T *pBssInfoParams = &pHwCtrl->pWhalParams->BssInfoParams;
    TemplateListParams_T *pWhalTemplates = &pHwCtrl->pWhalParams->TemplateList;
    HwMboxCmd_T *pHwCmd = whal_hwCtrl_GetMboxCmd (pHwCtrl);

    /*
     * Set the templates 
     */
    if (pWhalTemplates->Beacon.Size != 0)
        whal_hwMboxCmd_ConfigureTemplateFrame (pHwCmd, 
                                               pWhalTemplates->Beacon.Buffer, 
                                               (UINT16)pWhalTemplates->Beacon.Size,
                                               CMD_BEACON,
                                               NULL,
                                               NULL);
    
    if (pWhalTemplates->ProbeResp.Size != 0)
        whal_hwMboxCmd_ConfigureTemplateFrame (pHwCmd, 
                                               pWhalTemplates->ProbeResp.Buffer, 
                                               (UINT16)pWhalTemplates->ProbeResp.Size,
                                               CMD_PROBE_RESP,
                                               NULL,
                                               NULL);

    if (pWhalTemplates->ProbeReq.Size != 0)
        whal_hwMboxCmd_ConfigureTemplateFrame (pHwCmd, 
                                               pWhalTemplates->ProbeReq.Buffer, 
                                               (UINT16)pWhalTemplates->ProbeReq.Size,
                                               CMD_PROBE_REQ,
                                               NULL,
                                               NULL);
    
    if (pWhalTemplates->NullData.Size != 0)
        whal_hwMboxCmd_ConfigureTemplateFrame (pHwCmd, 
                                               pWhalTemplates->NullData.Buffer, 
                                               (UINT16)pWhalTemplates->NullData.Size,
                                               CMD_NULL_DATA,
                                               NULL,
                                               NULL);
    
    if (pWhalTemplates->PsPoll.Size != 0)
        whal_hwMboxCmd_ConfigureTemplateFrame (pHwCmd, 
                                               pWhalTemplates->PsPoll.Buffer, 
                                               (UINT16)pWhalTemplates->PsPoll.Size,
                                               CMD_PS_POLL,
                                               NULL,
                                               NULL);

    /*
     * Call the hardware to start/join the BSS 
     */
    return whal_hwCtrl_StartJoin (pHwCtrl, (bssType_e)pBssInfoParams->ReqBssType, NULL, NULL);
}

/****************************************************************************
 *                 whal_hwCtrl_setBetParams()
 ****************************************************************************
 * DESCRIPTION: Configures Beacon Early Termination information element.
 * 
 * Input    :   enabled               - 0 to disable BET, 0 to disable BET
 *              MaximumConsecutiveET  - Max number of consecutive beacons
 *                                      that may be early terminated.
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwCtrl_setBetParams(HwCtrl_T * pHwCtrl, UINT8 Enable, UINT8 MaximumConsecutiveET)
{
    return whal_hwInfoElemBETSet(pHwCtrl->pHwMboxConfig, Enable, MaximumConsecutiveET);
}


