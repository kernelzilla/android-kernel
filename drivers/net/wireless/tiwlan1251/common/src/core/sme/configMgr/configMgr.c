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

/** \file configMgr.c
 *  \brief Driver interface to OS abstraction layer
 *
 *  \see srcApi.h
 */

#include "osTIType.h"
#include "osApi.h"
#include "paramOut.h"
#include "paramIn.h"
#include "srcApi.h"
#include "report.h"
#include "whalCtrl_api.h"
#include "connApi.h"
#include "siteMgrApi.h"
#include "smeSmApi.h"
#include "utils.h"
#include "fsm.h"
#include "configMgr.h"
#include "DataCtrl_Api.h"
#include "rsnApi.h"
#include "scrApi.h"
#include "MacServices_api.h"  
#include "ScanCncnApi.h"
#include "scanMngrApi.h"
#include "regulatoryDomainApi.h"
#include "measurementMgrApi.h"
#ifdef EXC_MODULE_INCLUDED
#include "excMngr.h"
#endif
#include "SoftGeminiApi.h"
#include "roamingMngrApi.h"
#include "qosMngr_API.h"
#include "whalCtrl.h"
#include "TrafficMonitor.h"
#include "PowerMgr_API.h"
#include "EvHandler.h"
#include "apConn.h"
#include "currBss.h"
#include "SwitchChannelApi.h"
#include "ScanCncnAppApi.h"
#include "healthMonitor.h"
#include "wspVer.h"
#include "Ethernet.h"
#include "Core_AdaptTx.h"
#include "TNETW_Driver_api.h"
#include "rx.h"
#include "Ctrl.h"

#include "recoveryMgr_API.h"

/****************************************************/
/*      Local Functions                             */
/****************************************************/
static configMgr_t *createDriver(TI_HANDLE hOs,void *pWLAN_Images, initTable_t *pInitTable);

static void configMgr_config (TI_HANDLE  hConfigManager);

static int createCore(configMgr_t *pConfigManager, TI_HANDLE hOs, initTable_t *pInitTable);

static  void configMgr_RetrieveFWInfo(configMgr_t *pConfigManager, initTable_t *pInitTable, whalCtrl_chip_t *pChip_Version);

static TI_STATUS configCore(configMgr_t *pConfigManager, whalCtrl_chip_t *pChipVer);

static void release_module(configMgr_t *pConfigManager);

static void configParamsAccessTable(configMgr_t *pConfigManager);

UINT32 configMgr_RegisterEvent(TI_HANDLE  hConfigMgr, PUCHAR pData, ULONG Length)
{
    configMgr_t *pConfigManager= (configMgr_t *)hConfigMgr;
    
    return EvHandlerRegisterEvent(pConfigManager->hEvHandler,pData,Length);
}

UINT32 configMgr_UnRegisterEvent(TI_HANDLE hConfigMgr, TI_HANDLE uEventID)
{
    configMgr_t *pConfigManager= (configMgr_t *)hConfigMgr;

    return EvHandlerUnRegisterEvent(pConfigManager->hEvHandler,uEventID);
}

UINT32 configMgr_GetEventData(TI_HANDLE hConfigMgr, PUCHAR pData, ULONG* pLength)
{
   configMgr_t *pConfigManager= (configMgr_t *)hConfigMgr;

   return EvHandlerGetEventData(pConfigManager->hEvHandler,pData,pLength);
}


/****************************************************/
/*      Interface Functions Implementation          */
/****************************************************/

/************************************************************************
 *                        configMgr_create                              *
 ************************************************************************
DESCRIPTION: Driver creation & configuration function, called by the OS abstraction layer, performs the following:
                -   Create the driver
                -   Configure the driver

INPUT:      hOs -           Handle to OS
            pInitTable -    Pointer to the init table as received from the OS abstraction layer

OUTPUT:     pMac- MAC address of the device as read from the chip

RETURN:     Handle to the driver

************************************************************************/
TI_HANDLE configMgr_create (TI_HANDLE         hOs,
                            void             *pWLAN_Images,
                            initTable_t      *pInitTable,
                            macAddress_t     *pMac)
{
    configMgr_t  *pConfigManager;

    /**************** 
    Create the Driver 
    *****************/
    pConfigManager = createDriver(hOs, pWLAN_Images, pInitTable);

    if (pConfigManager == NULL)
    {
        WLAN_OS_REPORT(("\n.....Configuration manager creation failure \n"));
        return NULL;
    }

    WLAN_REPORT_INIT(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,  
        ("CREATED DRIVER\n"));

    return (TI_HANDLE)pConfigManager;
}


/************************************************************************
 *                        configMgr_init                                *
 ************************************************************************
DESCRIPTION: FW Code Download in partition

INPUT:      hOs -           Handle to OS
            TI_HANDLE       hConfigManager
            pInitTable -    Pointer to the init table as received from the OS abstraction layer

OUTPUT:     pMac- MAC address of the device as read from the chip

RETURN:     Handle to the driver

************************************************************************/
TI_HANDLE configMgr_init (TI_HANDLE     hOs, 
                          TI_HANDLE     hConfigManager, 
                          void         *pWLAN_Images,
                          initTable_t  *pInitTable,
                          macAddress_t *pMacAddr)
{
    configMgr_t  *pConfigManager = (configMgr_t *)hConfigManager;

    pConfigManager->pInitTable = pInitTable; 
    pConfigManager->pMacAddr = pMacAddr;

    /*
     * Init the module and Download the FW code in partition
     * At this stage it is very important that the pConfigManager has been created and linked to the HAL Ctrl
     * so that if the first DMA ends before the user returns to wait for the end of DMA
     * then the pConfigManager will have a valid value 
     */
    if (TnetwDrv_Init (pConfigManager->hTnetwDrv, 
                       pConfigManager->hReport,
                       pConfigManager->hMemMgr,
                       hConfigManager,
                       pWLAN_Images,
                       &pInitTable->TnetwDrv_InitParams,
                       configMgr_config) == TNETWIF_ERROR)
    {
        WLAN_OS_REPORT(("\n.....TNETW_Driver_Initialize: TNETW_Driver_Init failure \n"));
        return NULL;
    }

    return pConfigManager->hTnetwDrv;
}


/************************************************************************
DESCRIPTION: Driver creation & configuration function, called by the OS abstraction layer, performs the following:
                -   Create the driver
                -   Configure the driver

INPUT:      hOs -           Handle to OS
            pInitTable -    Pointer to the init table as received from the OS abstraction layer

OUTPUT:     pMac- MAC address of the device as read from the chip

RETURN:     Handle to the driver

************************************************************************/
static void configMgr_config (TI_HANDLE  hConfigManager)
{
    configMgr_t     *pConfigManager = (configMgr_t *)hConfigManager;
    whalCtrl_chip_t  chipVer;
    UINT8           *pMac;

    /**************** 
    Config the Driver 
    *****************/
    WLAN_OS_REPORT(("Initializing Config Manager...\n"));

    if (configCore (pConfigManager, &chipVer) == NOK)
    {
        WLAN_OS_REPORT(("\n.....Configuration manager configuration failure\n"));
        release_module (pConfigManager);
        return;
    }

    pMac = (UINT8 *)chipVer.macAddress.addr;

    os_memoryCopy (pConfigManager->hOs, 
                   pConfigManager->pMacAddr, 
                   (void *)pConfigManager->pInitTable->ctrlDataInitParams.ctrlDataDeviceMacAddress.addr, 
                   MAC_ADDR_LEN);

    pConfigManager->state = CFG_MGR_STATE_IDLE;     

    /* Print the driver and firmware version and the mac address */
    WLAN_OS_REPORT(("\n"));
    WLAN_OS_REPORT(("--------------------------------------------------------------------\n"));
    WLAN_OS_REPORT(("Driver Version  : %s\n", SW_VERSION_STR));
    WLAN_OS_REPORT(("Firmware Version: %s\n", chipVer.fwVer));
    WLAN_OS_REPORT(("Station ID      : %02X-%02X-%02X-%02X-%02X-%02X\n",
                    pMac[0], pMac[1], pMac[2], pMac[3], pMac[4], pMac[5]));
    WLAN_OS_REPORT(("--------------------------------------------------------------------\n"));
    WLAN_OS_REPORT(("\n"));
}


/************************************************************************
 *                        configMgr_start                               *
 ************************************************************************
DESCRIPTION: Driver start function, called by the OS abstraction layer in
            order to start the driver after creation.
            It enables the ISR and sends a start event to the driver's main state machine
            If the the driver was in IDLE state, it sends a 'WakeUp' command to the chip.

INPUT:      hConfigMgr -    Handle to the driver

OUTPUT:

RETURN:     OK on success, NOK on failure

************************************************************************/
TI_STATUS configMgr_start(TI_HANDLE hConfigMgr)
{
    TI_STATUS  status = PARAM_VALUE_NOT_VALID;

    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;

    switch (pConfigManager->state)
   {
    case CFG_MGR_STATE_IDLE :
        pConfigManager->state = CFG_MGR_STATE_RUNNING;
        WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,
                              ("<STATE_IDLE, EVENT_START> --> STATE_RUNNING\n\n"));
        status = smeSm_start(pConfigManager->hSmeSm);

#ifdef SDIO_INTERRUPT_HANDLING_ON
        whalCtr_SlaveAckMaskNotification(pConfigManager->hHalCtrl);
#endif
        break;


   case CFG_MGR_STATE_RUNNING :
        WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,
                              ("Got start command while not in RUNNING, ignoring the command"));
        break;


   case CFG_MGR_STATE_STOPPED:
        pConfigManager->state = CFG_MGR_STATE_RUNNING;
        WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,
                         ("<STATE_STOPPED, EVENT_START> --> STATE_RUNNING\n\n"));
        status = smeSm_start(pConfigManager->hSmeSm);
        break;

    }

    return status;
}


/************************************************************************
 *                        configMgr_stop                                *
 ************************************************************************
DESCRIPTION: Driver stop function, called by the OS abstraction layer in
            order to stop the driver.
            It sends a stop event to the driver main state mmachine
            If the the driver was in RUNNING state, it sends a 'Sleep' command to the chip.

INPUT:      hConfigMgr -    Handle to the driver

OUTPUT:

RETURN:     OK on success, NOK on failure

************************************************************************/
TI_STATUS configMgr_stop(TI_HANDLE hConfigMgr)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;
    TI_STATUS   status  = PARAM_VALUE_NOT_VALID;

 
   switch (pConfigManager->state)
    {
    case CFG_MGR_STATE_IDLE :
        WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,
                         ("<STATE_IDLE, EVENT_STOP> --> STATE_IDLE\n\n"));  
        status = STATION_IS_NOT_RUNNING;
        break;

   case CFG_MGR_STATE_RUNNING :
        pConfigManager->state = CFG_MGR_STATE_STOPPED;
        WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,
                         ("<STATE_RUNNING, EVENT_STOP> --> STATE_STOPPED\n\n"));
        smeSm_stop(pConfigManager->hSmeSm);
        break;
 
   case CFG_MGR_STATE_STOPPED:
        WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,
                         ("<STATE_STOPPED, EVENT_STOP> --> STATE_STOPPED\n\n"));
        status = STATION_IS_NOT_RUNNING;
        break;

    }

   return status;
}

/****************************************************************************************
 *                        configMgr_setParam                                            *
 ****************************************************************************************
DESCRIPTION:    Called by the OS abstraction layer in order to set a parameter the driver.
                If the parameter can not be set from outside the driver it returns a fialure status
                The parameters is set to the module that uses as its father in the system
                (refer to the file paramOut.h for more explanations)
                If the father returns a RE_SCAN_NEEDED status, it restarts the main
                state machine of the driver.


INPUT:          hConfigMgr -    Handle to the driver
                pParam  -       Pointer to the parameter

OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
TI_STATUS configMgr_setParam(TI_HANDLE hConfigMgr, paramInfo_t *pParam)                          
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;
    TI_STATUS    status;
    UINT32       moduleNumber;

    if (pConfigManager->state != CFG_MGR_STATE_RUNNING)
    {
        WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,  ("State machine error, EVENT_SET_PARAM while in IDLE state \n\n"));
        return STATION_IS_NOT_RUNNING;
    }

    WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,
                              ("<STATE_RUNNING, EVENT_SET_PARAM> --> STATE_RUNNING\n\n"));


    if (!EXTERNAL_SET_ENABLE(pParam->paramType))
        return EXTERNAL_SET_PARAM_DENIED;

    moduleNumber = GET_PARAM_MODULE_NUMBER(pParam->paramType);

    if  (moduleNumber > MAX_PARAM_MODULE_NUMBER)
        return PARAM_MODULE_NUMBER_INVALID;

    WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,
                   ("ParamType=0x%x ModuleNumber=0x%x\n\n",pParam->paramType));

    status = pConfigManager->paramAccessTable[moduleNumber - 1].set(pConfigManager->paramAccessTable[moduleNumber - 1].handle, pParam);
    
    if(status == RE_SCAN_NEEDED)
        return smeSm_reselect(pConfigManager->hSmeSm);
    else
        return status;

}

/****************************************************************************************
 *                        configMgr_getParam                                            *
 ****************************************************************************************
DESCRIPTION:    Called by the OS abstraction layer in order to get a parameter the driver.
                If the parameter can not be get from outside the driver it returns a fialure status
                The parameters is get from the module that uses as its father in the system
                (refer to the file paramOut.h for more explanations)


INPUT:          hConfigMgr -    Handle to the driver
                pParam  -       Pointer to the parameter

OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
TI_STATUS configMgr_getParam (TI_HANDLE hConfigMgr, paramInfo_t *pParam)
                          
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;
    UINT32       moduleNumber;

    /* This is a unique parameter, it checks the driver running status, therefore we handle it here. */
    if (pParam->paramType == DRIVER_STATUS_PARAM)
    {
        if (pConfigManager->state == CFG_MGR_STATE_RUNNING)
            pParam->content.driverStatus = DRIVER_STATUS_RUNNING;
        else
            pParam->content.driverStatus = DRIVER_STATUS_IDLE;
        return OK;
    }

    if (pConfigManager->state != CFG_MGR_STATE_RUNNING)
    {
        WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,  ("State machine error, EVENT_GET_PARAM while in IDLE state \n\n"));
        return STATION_IS_NOT_RUNNING;
    }

    WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,
                              ("<STATE_RUNNING, EVENT_GET_PARAM> --> STATE_RUNNING\n\n"));

    if (!EXTERNAL_GET_ENABLE(pParam->paramType))
        return EXTERNAL_GET_PARAM_DENIED;

    moduleNumber = GET_PARAM_MODULE_NUMBER(pParam->paramType);

    if  (moduleNumber > MAX_PARAM_MODULE_NUMBER)
        return PARAM_MODULE_NUMBER_INVALID;

    return pConfigManager->paramAccessTable[moduleNumber - 1].get(pConfigManager->paramAccessTable[moduleNumber - 1].handle, pParam);
}


/****************************************************************************************
 *                        configMgr_checkTxQueueSize                                            *
 ****************************************************************************************
DESCRIPTION:    Check Tx queue size


INPUT:          hConfigMgr  -   Handle to the driver

OUTPUT:

RETURN:         OK on success, NOK on queue full

************************************************************************/
TI_STATUS configMgr_checkTxQueueSize(TI_HANDLE hConfigMgr,UINT8 qIndex)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;
    return txData_checkQueueSize(pConfigManager->hTxData, qIndex);
}


/****************************************************************************************
 *                        configMgr_printTxQueues                                            *
 ****************************************************************************************
DESCRIPTION:    Tx queues print


INPUT:          hConfigMgr  -   Handle to the driver


************************************************************************/
#ifdef TI_DBG
void configMgr_printTxQueuesAndMemPolls(TI_HANDLE hConfigMgr)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;
    txData_printTxQosCounters(pConfigManager->hTxData);
    txData_fullPrintDataMsduList(pConfigManager->hTxData);
    memMngrPrint(pConfigManager->hMemMgr);
}
#endif


/****************************************************************************************
 *                        configMgr_sendMsdu                                            *
 ****************************************************************************************
DESCRIPTION:    Called by the OS abstract in layer in order to send a MSDU to the wlan network.


INPUT:          hConfigMgr  -   Handle to the driver
                pMsdu       -   Pointer to the MSDU
                packet_DTag -   NDIS packet user priority tag

OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
TI_STATUS configMgr_sendMsdu (TI_HANDLE      hConfigMgr,
                              mem_MSDU_T    *pMsdu,
                              UINT8          packet_DTag)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;
    TI_STATUS  status = OK;     

#ifdef TI_DBG
    /* Add time stamp */
    wlan_memMngrAddTimeStamp (pConfigManager->hMemMgr, pMsdu);

    if (pConfigManager->state != CFG_MGR_STATE_RUNNING)
    {
        WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,  ("State machine error, EVENT_SEND_MSDU while in IDLE state \n\n"));
        return STATION_IS_NOT_RUNNING;
    }

    WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,
                              ("<STATE_RUNNING, EVENT_SEND_MSDU> --> STATE_RUNNING\n\n"));
#endif

   WLAN_REPORT_DEBUG_TX(pConfigManager->hReport,("configMgr_sendMsdu Sending packet Lenght\n",pMsdu->dataLen));
   status =  txData_sendPktToWlan(pConfigManager->hTxData, pMsdu, packet_DTag);
   return status;
}


/****************************************************************************************
 *                        configMgr_PollApPackets                                           *
 ****************************************************************************************
DESCRIPTION:    Called by the OS abstraction layer in order to send VAD frame.
                Calls the txData module corresponding function.


INPUT:          hConfigMgr  -   Handle to the driver

OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
TI_STATUS configMgr_PollApPackets(TI_HANDLE     hConfigMgr)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;

    if (pConfigManager->state != CFG_MGR_STATE_RUNNING)
    {
        WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,  ("State machine error, EVENT_SEND_MSDU while in IDLE state \n\n"));
        return STATION_IS_NOT_RUNNING;
    }

    WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,
                              ("<STATE_RUNNING, EVENT_SEND_VAD_FRAME> --> STATE_RUNNING\n\n"));

    return txData_sendVadFrame(pConfigManager->hTxData, POLL_AP_PACKETS_FORCE_PS_POLL);
}



/****************************************************************************************
 *                        configMgr_HandleBusTxn_Complete                                    *
 ****************************************************************************************
DESCRIPTION:    Called by the OS abstraction layer in order notify the driver that the DMA has finished


INPUT:          hConfigMgr  -   Handle to the driver

OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
TI_STATUS configMgr_HandleBusTxn_Complete(TI_HANDLE hConfigMgr)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;

    whalCtrl_HandleBusTxn_Complete(pConfigManager->hHalCtrl);
    return OK;
}

/****************************************************************************************
 *                        configMgr_handleInterrupts                                    *
 ****************************************************************************************
DESCRIPTION:    Called by the OS abstraction layer in order notify the driver that a ISR arrived


INPUT:          hConfigMgr  -   Handle to the driver

OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
TI_STATUS configMgr_handleInterrupts(TI_HANDLE hConfigMgr)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;

    return (TI_STATUS)whalCtrl_HandleInterrupts(pConfigManager->hHalCtrl);
}

/****************************************************************************************
 *                        configMgr_enableInterrupts                                    *
 ****************************************************************************************
DESCRIPTION:    Called by the OS abstraction layer in order to enable interrupts


INPUT:          hConfigMgr  -   Handle to the driver

OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
TI_STATUS configMgr_enableInterrupts(TI_HANDLE hConfigMgr)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;

    whalCtrl_EnableInterrupts(pConfigManager->hHalCtrl);
    return OK;
}

/****************************************************************************************
 *                        configMgr_disableInterrupts                                   *
 ****************************************************************************************
DESCRIPTION:    Called by the OS abstraction layer in order to disable interrupts


INPUT:          hConfigMgr  -   Handle to the driver

OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
TI_STATUS configMgr_disableInterrupts(TI_HANDLE hConfigMgr)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;

    whalCtrl_DisableInterrupts(pConfigManager->hHalCtrl);
    return OK;
}


/****************************************************************************************
 *                        configMgr_disableRadio                                    *
 ****************************************************************************************
DESCRIPTION:    Called by the OS abstraction layer in order to disable Radio


INPUT:          hConfigMgr  -   Handle to the driver

OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
TI_STATUS configMgr_disableRadio(TI_HANDLE hConfigMgr)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;


    WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,
                              ("<STATE_RUNNING, EVENT_DISABLE_RADIO> --> STATE_RUNNING\n\n"));

    /* Disable radio command is no longer active, and should be directed to the SME module. */
    /* whalCtrl_DisableRadio(pConfigManager->hHalCtrl); */
    return OK;

}


/****************************************************************************************
 *                        configMgr_checkInterrupts                                     *
 ****************************************************************************************
DESCRIPTION:    Called by the OS abstraction layer in order to let the driver check if
                the receive interrupt is a driver's ISR.


INPUT:          hConfigMgr  -   Handle to the driver

OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
UINT32 configMgr_checkInterrupts(TI_HANDLE hConfigMgr)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;

    return whalCtrl_CheckInterrupts(pConfigManager->hHalCtrl);
}


/****************************************************************************************
 *                        configMgr_ReadMacRegister                                     *
 ****************************************************************************************
DESCRIPTION:    API function for registers read/write


INPUT:          hConfigMgr  -   Handle to the driver

OUTPUT:

RETURN:
************************************************************************/
UINT32 configMgr_ReadMacRegister(TI_HANDLE hConfigMgr, UINT32   addr)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;

    return whalCtrlReadMacReg(pConfigManager->hHalCtrl, addr);
}

void  configMgr_WriteMacRegister(TI_HANDLE hConfigMgr, UINT32   addr, UINT32    val)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;

    whalCtrlWriteMacReg(pConfigManager->hHalCtrl, addr, val);
}

UINT32 configMgr_ReadPhyRegister(TI_HANDLE hConfigMgr, UINT32   addr)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;

        return whalCtrlReadPhyReg(pConfigManager->hHalCtrl, addr);
}

void configMgr_WritePhyRegister(TI_HANDLE hConfigMgr, UINT32    addr, UINT32    val)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;

    whalCtrlWritePhyReg(pConfigManager->hHalCtrl, addr, val);
}

/****************************************************************************************
 *                        configMgr_isCardExist                                         *
 ****************************************************************************************
DESCRIPTION:    Called by the OS abstraction layer in order to check if the card is inserted.


INPUT:          hConfigMgr  -   Handle to the driver

OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
BOOL configMgr_isCardExist(TI_HANDLE hConfigMgr)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;

    if (whalCtrl_isCardIn(pConfigManager->hHalCtrl) == TRUE)
        return TRUE;
    else
    {
        wlan_memMngrFreeAllOsAlocatesBuffer(pConfigManager->hMemMgr);
        return FALSE;
    }
}

/****************************************************************************************
 *                        configMgr_areInputsFromOsDisabled                                         *
 ****************************************************************************************
DESCRIPTION:    Called by the OS abstractin layer in order to 
                check if Inputs From OS are Disabled.


INPUT:          hConfigMgr  -   Handle to the driver

OUTPUT:

RETURN:         TRUE  - recovery is in process, 
                FALSE - recovery is not in process

************************************************************************/
BOOL configMgr_areInputsFromOsDisabled(TI_HANDLE hConfigMgr)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;

    return (recoveryMgr_areInputsFromOsDisabled(pConfigManager->hRecoveryMgr));
}

/****************************************************************************************
 *                        configMgr_allocBDs                                            *
 ****************************************************************************************
DESCRIPTION:    Called by the OS abstraction in order to allocate BDs from the memory manager pool.
                Calls the memory manager corresponding function.



INPUT:          hConfigMgr  -   Handle to the driver
                bdNumber -      Number of BDs to allocate

OUTPUT:         bdPtr    -      Pointer to return the link list of allocated BDs.

RETURN:         OK on success, NOK on failure

************************************************************************/
TI_STATUS configMgr_allocBDs(TI_HANDLE  hConfigMgr,
                             UINT32     bdNumber,
                             mem_BD_T** bdPtr)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;
    TI_STATUS Status;

#ifdef TI_DBG
    if (pConfigManager->state != CFG_MGR_STATE_RUNNING)
    {
        WLAN_REPORT_ERROR(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,  ("State machine error, EVENT_ALLOC_BDS while in IDLE state \n\n"));
        return STATION_IS_NOT_RUNNING;
    }

    WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,
                              ("<STATE_RUNNING, EVENT_ALLOC_BDS> --> STATE_RUNNING\n\n"));
#endif

    Status = wlan_memMngrAllocBDs(pConfigManager->hMemMgr, bdNumber, bdPtr);
    return Status;
}

/****************************************************************************************
 *                        configMgr_allocMSDU                                           *
 ****************************************************************************************
DESCRIPTION:    Called by the OS abstraction layer in order to allocate a MSDU and its associated
                BDs and data buffers.
                Calls the memory manager corresponding function.

INPUT:          hConfigMgr  -   Handle to the driver
                len         -   the length of the required data buffer
                module      -   The module that requests the allocation.

OUTPUT:         MSDUPtr  -      Pointer to return he allocated MSDU.

RETURN:         OK on success, NOK on failure

************************************************************************/
TI_STATUS configMgr_allocMSDU(TI_HANDLE    hConfigMgr,
                              mem_MSDU_T** MSDUPtr,
                              UINT32       len,
                              allocatingModule_e module)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;

#ifdef TI_DBG
    if (pConfigManager->state != CFG_MGR_STATE_RUNNING)
    {
        WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,  ("State machine error, EVENT_ALLOC_MSDU while in IDLE state \n\n"));
        return STATION_IS_NOT_RUNNING;
    }

    WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,
                              ("<STATE_RUNNING, EVENT_ALLOC_MSDU> --> STATE_RUNNING\n\n"));
#endif

    return wlan_memMngrAllocMSDU(pConfigManager->hMemMgr, MSDUPtr, len, OS_ABS_TX_MODULE);
}

/****************************************************************************************
 *                        configMgr_allocMSDUBufferOnly                                         *
 ****************************************************************************************
DESCRIPTION:    Called by the OS abstraction layer in order to allocate a MSDU only.
                Calls the memory manager corresponding function.


INPUT:          hConfigMgr  -   Handle to the driver
                module      -   The module that requests the allocation.

OUTPUT:         MSDUPtr  -      Pointer to return he allocated MSDU.

RETURN:         OK on success, NOK on failure

************************************************************************/
TI_STATUS configMgr_allocMSDUBufferOnly(TI_HANDLE    hConfigMgr,
                                        mem_MSDU_T** MSDUPtr,
                                        allocatingModule_e module)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;

#ifdef TI_DBG
    if (pConfigManager->state != CFG_MGR_STATE_RUNNING)
    {
        WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,  ("State machine error, EVENT_ALLOC_MSDU_BUFFER_ONLY while in IDLE state \n\n"));
        return STATION_IS_NOT_RUNNING;
    }

    WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,
                              ("<STATE_RUNNING, EVENT_ALLOC_MSDU_BUFFER_ONLY> --> STATE_RUNNING\n\n"));
#endif

    return wlan_memMngrAllocMSDUBufferOnly(pConfigManager->hMemMgr, MSDUPtr, OS_ABS_TX_MODULE);
}

/****************************************************************************************
 *                        configMgr_memMngrFreeMSDU                                         *
 ****************************************************************************************
DESCRIPTION:    Called by the OS abstraction layer in order to free a MSDU.
                Calls the memory manager corresponding function.


INPUT:          hConfigMgr  -   Handle to the driver
                handle      -   handle of the MSDU.

OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
TI_STATUS configMgr_memMngrFreeMSDU(TI_HANDLE hConfigMgr, UINT32 handle)                                    
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigMgr;

#ifdef TI_DBG
    WLAN_REPORT_SM(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,
                              ("<STATE_RUNNING, EVENT_FREE_MSDU> --> STATE_RUNNING\n\n"));
#endif

    wlan_memMngrFreeMSDU(pConfigManager->hMemMgr, handle);
    return OK;
}

/****************************************************************************************
 *                        configMgr_unLoad                                              *
 ****************************************************************************************
DESCRIPTION:    Driver unload function


INPUT:          hConfigMgr  -   Handle to the driver

OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
TI_STATUS configMgr_unLoad(TI_HANDLE hConfigMgr)
{
    configMgr_t     *pConfigManager = (configMgr_t *)hConfigMgr;

    if (!pConfigManager)
        return NOK;

    whalCtrl_Stop(pConfigManager->hHalCtrl);

    WLAN_OS_REPORT(("\nCONFIG_MGR,  UNLOAD:   *****  DESTROYING THE DRIVER  *****\n\n\n"));
    release_module(pConfigManager);

    return OK;
}


/****************************************************************************************
 *                        configMgr_InitiateUnload                                              *
 ****************************************************************************************
DESCRIPTION:    Driver unload function


INPUT:          hConfigMgr  -   Handle to the driver

OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
TI_STATUS configMgr_InitiateUnload(TI_HANDLE hConfigMgr)
{
    configMgr_t     *pConfigManager = (configMgr_t *)hConfigMgr;

    if (!pConfigManager)
        return NOK;

    smeSm_stopAndShutdown(pConfigManager->hSmeSm);

    WLAN_OS_REPORT(("\nCONFIG_MGR,  UNLOAD:   *****  DESTROYING THE DRIVER  *****\n\n\n"));

    return OK;
}


/****************************************************************************************
 *                        configMgr_UnloadModules                                              *
 ****************************************************************************************
DESCRIPTION:    Driver unload function


INPUT:          hConfigMgr  -   Handle to the driver

OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
TI_STATUS configMgr_UnloadModules(TI_HANDLE hConfigMgr)
{
    configMgr_t     *pConfigManager = (configMgr_t *)hConfigMgr;

    if (!pConfigManager)
        return NOK;

    release_module(pConfigManager);

    return OK;
}

/****************************************************************************************
 *                        configMgr_DriverShutdownStatus                                              *
 ****************************************************************************************
DESCRIPTION:    return status of driver shutdown process 

INPUT:          hConfigMgr  -   Handle to the driver

OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
UINT8 configMgr_DriverShutdownStatus(TI_HANDLE hConfigMgr)
{
   configMgr_t     *pConfigManager = (configMgr_t *)hConfigMgr;

   return smeSm_getDriverShutdownStatus(pConfigManager->hSmeSm);
}

/****************************************************************************************
 *                        configMgr_SlaveAckMaskNotification                                   *
 ****************************************************************************************
DESCRIPTION:    

INPUT:          hConfigMgr  -   Handle to the driver

OUTPUT:        

RETURN:         void

************************************************************************/

void configMgr_SlaveAckMaskNotification(TI_HANDLE hConfigMgr)
{
    configMgr_t     *pConfigManager = (configMgr_t *)hConfigMgr;
    whalCtr_SlaveAckMaskNotification(pConfigManager->hHalCtrl); 
}

/****************************************************************************************
 *                        configMgr_getPacketHeaderLength                                   *
 ****************************************************************************************
DESCRIPTION:    Called by os to know reserved space for WLAN header.

INPUT:          hConfigMgr  -   Handle to the driver
                data        -   pointer to packet
                txFlags     -   whether this frame is data or management
OUTPUT:          

RETURN:         void

************************************************************************/

UINT32 configMgr_getPacketHeaderLength(TI_HANDLE      hConfigMgr,
                                       void           *pData,
                                       UINT32         txFlags)
{
    configMgr_t          *pConfigManager = (configMgr_t *)hConfigMgr;
    
    /* returns TxDescriptor size + reserved place for the bus txn operation + actual header length */
    return TX_TOTAL_OFFSET_BEFORE_DATA +
        txData_GetWlanHeaderLength (pConfigManager->hTxData, pData, txFlags);
}


/****************************************************/
/*      Local Functions Implementations             */
/****************************************************/
/****************************************************************************************
 *                        createDriver                                              *
 ****************************************************************************************
DESCRIPTION:    Driver creation function. Performs the following:
                -   Calls the create function of each module.
                -   Each module returns a handle if successful and NULL otherwise.
                -   If the creation of all the modules succeeded, the driver main handle is configured with the
                        modules handles. Then the driver main handle is returned to the caller.
                -   If one of the modules fails in creation, the function calls the release function of the driver
                    and all the modules handles are free, including the driver main handle.
                -   Some of the modules are called with an init table as a parameter.
                -   The callbacks table is filled with the callbacks returned from the core, in the configuration phase,
                    the HAL is configured with those callbacks.


INPUT:          hOs        -    Handle to the OS
                pInitTable -    Pointer to the init table as read from registry
                pCoreCallbacks   -  Table of core callbacks functions to filled by the CORE

OUTPUT:         Main Handle to the driver.

RETURN:         Handle to the driver on success, NOK on failure

************************************************************************/
static configMgr_t *createDriver(TI_HANDLE hOs, void *pWLAN_Images, initTable_t *pInitTable)
{
    configMgr_t             *pConfigManager;
    
    /************************ 
    Create the Config Manager 
    *************************/
    pConfigManager = os_memoryAlloc(hOs, sizeof(configMgr_t));
    if (pConfigManager == NULL)
    {
        return NULL;
    }
    os_memoryZero (hOs, pConfigManager, sizeof(configMgr_t));
    pConfigManager->hOs = hOs;

    /************************** 
    Create all the CORE modules 
    ***************************/
    if (createCore (pConfigManager ,hOs,  pInitTable) != OK)
    {
        WLAN_OS_REPORT(("\n createCore() Failed!!! \n"));
        release_module(pConfigManager);
        return NULL;
    }
    
    WLAN_INIT_REPORT(("\n createDriver(): pConfigManager->hOs %x pConfigManager->hMemMgr %x!!! :\n\n",pConfigManager->hOs,pConfigManager->hMemMgr));

    /************************** 
    Configure the Report module 
    ***************************/
    if (report_config(pConfigManager->hReport, hOs, &pInitTable->TnetwDrv_InitParams.reportParams) != OK)
    {
        WLAN_OS_REPORT(("\n Report configuration failure \n"));
        release_module(pConfigManager);
        return NULL;
    }

    /********************** 
    Create the TNETW Driver 
    ***********************/
    pConfigManager->hTnetwDrv = TnetwDrv_Create(hOs);

    if (pConfigManager->hTnetwDrv == NULL)
    {
        WLAN_OS_REPORT(("\n createDriver() !!! TnetwDrv_Create failed !!! \n"));
        release_module(pConfigManager);
        return NULL;
    }

    WLAN_INIT_REPORT(("\n createDriver(): pConfigManager %x pConfigManager->hTnetwDrv %x!!! :\n\n",pConfigManager,pConfigManager->hTnetwDrv));

    /*************************************************************** 
    TEMPORARY!!  -  get TNETW-Driver internal modules handles untill  
                    the new TNETW-Driver architecture is completed!!
    ****************************************************************/
    TnetwDrv_TEMP_GetHandles (pConfigManager->hTnetwDrv, 
                              &pConfigManager->hHalCtrl,
                              &pConfigManager->hMacServices);
    
    WLAN_INIT_REPORT(("\nCONFIG_MGR,  INIT:       *****   CREATION SUCCESS    *****\n\n\n"));
      
    return pConfigManager;
}

/****************************************************************************************
 *                        createCore                                   *
 ****************************************************************************************
DESCRIPTION:   Called by the CreateDriver to 
                        - Create the CORE modules
                        - Create the CORE Adapter module
                        - Config the CORE Adapter module by giving it CORE callbacks

INPUT:          TI_HANDLE hOs  -   Handle to OS
                initTable_t *pInitTable - pointer to the Init table filled by the registry
                coreCallbacks_t *pCoreCallbacks  - pointer to the CORE callbacks to be used by each module to fll it (Scan,CtrlData ..)

OUTPUT:         TI_STATUS - OK on success else NOK

RETURN:         void

************************************************************************/
static int createCore(configMgr_t *pConfigManager, TI_HANDLE hOs, initTable_t *pInitTable)
{
    /* Report module */
    pConfigManager->hReport = report_create(hOs);
    if (pConfigManager->hReport == NULL)
    {
        return NOK;
    }

    /* SCR module */
    pConfigManager->hSCR = scr_create(hOs);
    if (pConfigManager->hSCR == NULL)
    {
        return NOK;
    }

    /* Event Handler module */
    pConfigManager->hEvHandler = EvHandlerInit(hOs);
    if (pConfigManager->hEvHandler == NULL)
    {
        return NOK;
    }

    /* Connection module */
    pConfigManager->hConn = conn_create(hOs);
    if (pConfigManager->hConn == NULL)
    {
        return NOK;
    }

    /* Scan Concentrator module */
    pConfigManager->hScanCncn = scanConcentrator_create(hOs);
    if (pConfigManager->hScanCncn == NULL)
    {
        return NOK;
    }

    /* SME state machine module */
    pConfigManager->hSmeSm = smeSm_create(hOs);
    if (pConfigManager->hSmeSm == NULL)
    {
        return NOK;
    }

    /* Site manager module */
    pConfigManager->hSiteMgr = siteMgr_create(hOs);
    if (pConfigManager->hSiteMgr == NULL)
    {
        return NOK;
    }

    /* MLME SM module */
    pConfigManager->hMlmeSm = mlme_create(hOs);
    if (pConfigManager->hMlmeSm == NULL)
    {
        return NOK;
    }

    /* AUTH module */
    pConfigManager->hAuth = auth_create(hOs);
    if (pConfigManager->hAuth == NULL)
    {
        return NOK;
    }

    /* ASSOC module */
    pConfigManager->hAssoc = assoc_create(hOs);
    if (pConfigManager->hAssoc == NULL)
    {
        return NOK;
    }

    /* Rx data module */
    pConfigManager->hRxData = rxData_create(hOs);
    if (pConfigManager->hRxData == NULL)
    {
        return NOK;
    }

    /* Tx data module */
    pConfigManager->hTxData = txData_create (&pInitTable->txDataInitParams, hOs);                                           
    if (pConfigManager->hTxData == NULL)
    {
        return NOK;
    }

    /* Ctrl data module */
    pConfigManager->hCtrlData = ctrlData_create(hOs);
    if (pConfigManager->hCtrlData == NULL)
    {
        return NOK;
    }

    /* Traffic Monitor  */
    pConfigManager->hTrafficMon = TrafficMonitor_create(hOs);
    if (pConfigManager->hTrafficMon == NULL)
    {
        return NOK;
    }

    /* Memory Manager */
    pConfigManager->hMemMgr = wlan_memMngrInit(hOs);
    if (pConfigManager->hMemMgr == NULL)
    {
        return NOK;
    }

    /* RSN create code */
    pConfigManager->hRsn = rsn_create(hOs);
    if (pConfigManager->hRsn == NULL)
    {
        return NOK;
    }

    /* Regulatory Domain module */
    pConfigManager->hRegulatoryDomain = regulatoryDomain_create(hOs);
    if (pConfigManager->hRegulatoryDomain == NULL)
    {
        return NOK;
    }

    /* MeasurementMgr module */
    pConfigManager->hMeasurementMgr = measurementMgr_create(hOs);
    if (pConfigManager->hMeasurementMgr == NULL)
    {
        return NOK;
    }

    /* Soft Gemini module */
    pConfigManager->hSoftGemini = SoftGemini_create(hOs);
    if (pConfigManager->hSoftGemini == NULL)
    {
        return NOK;
    }


#ifdef EXC_MODULE_INCLUDED
    pConfigManager->hExcMngr = excMngr_create(hOs);
    if (pConfigManager->hExcMngr == NULL)
    {
        return NOK;
    }
#else
    pConfigManager->hExcMngr = NULL;
#endif

    pConfigManager->hRoamingMngr = roamingMngr_create(hOs);
    if (pConfigManager->hRoamingMngr == NULL)
    {
        return NOK;
    }

    pConfigManager->hAPConnection = apConn_create(hOs);
    if (pConfigManager->hAPConnection == NULL)
    {
        return NOK;
    }

    pConfigManager->hCurrBss = currBSS_create(hOs);
    if (pConfigManager->hCurrBss == NULL)
    {
        return NOK;
    }

    pConfigManager->hQosMngr = qosMngr_create(hOs);
    if (pConfigManager->hQosMngr == NULL)
    {
        return NOK;
    }

    pConfigManager->hPowerMgr = PowerMgr_create(hOs);
    if (pConfigManager->hPowerMgr == NULL)
    {
        return NOK;
    }

    pConfigManager->hSwitchChannel = switchChannel_create(hOs);
    if (pConfigManager->hSwitchChannel == NULL)
    {
        return NOK;
    }

    pConfigManager->hScanMngr = scanMngr_create(hOs);
    if (NULL == pConfigManager->hScanMngr)
    {
        return NOK;
    }

    pConfigManager->hHealthMonitor = healthMonitor_create(hOs);
    if (NULL == pConfigManager->hHealthMonitor)
    {
        return NOK;
    }

    /* CORE ADAPTER CREATION */    

    /* ADD CORE ADAPTER Tx CREATION */
    CORE_AdaptTx_handle = CORE_AdaptTx_Create(hOs);
    if (CORE_AdaptTx_handle == NULL)
    {
        return NOK;
    }  

    pConfigManager->hRecoveryMgr = recoveryMgr_create(hOs);
#ifdef USE_RECOVERY
    if (NULL == pConfigManager->hRecoveryMgr)
    {
        return NOK;
    }
#endif
    WLAN_INIT_REPORT(("\nCONFIG_MGR,  INIT:       *****   CORE CREATION SUCCESS    *****\n\n\n"));

    return OK;
}


/****************************************************************************************
 *                        configCore                                              *
 ****************************************************************************************
DESCRIPTION:    Core configuration function. Performs the following:
                -   Calls the config Params Access function.
                -   Calls the config function of each module.
                -   Each module is configured with the following parameters:
                    -   List of handles to other modules that supply him services
                    -   Init table (optional).
                    -   Callbacks to be used (in the HAL cases)
                -   In addition, the following parameters are read from the chip by the HAL and forwarded to the CORE:
                    -   Chip MAC Address
                    -   Chip regulatory domain
                    -   Chip preamble
                    -   Chip FW version and EEPROM version
                    -   Radio type

INPUT:          pConfigManager     -    Driver main handle
                pInitTable          -   Pointer to the init table as read from registry

OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
static TI_STATUS configCore (configMgr_t *pConfigManager, whalCtrl_chip_t *pChipVer)
{
    /************************/
    /*      CONFIGURATION   */
    /************************/
    /*radioDisableParams_t    radioDisableParams;*/
    Core_AdaptTx_config_t   Core_AdaptTx_config;

    /* we first initialize the setParamTable & getParamTable based on the moduleParam_e */
    configParamsAccessTable (pConfigManager);

    /*
     * Retrieve FW information from HAL
     */
    configMgr_RetrieveFWInfo (pConfigManager, pConfigManager->pInitTable, pChipVer);

    /* CORE Adapter configuration is setting callback to each module */       

    /* CORE ADAPTER TX module */
    Core_AdaptTx_config.hMemMgr = pConfigManager->hMemMgr;  
    Core_AdaptTx_config.hReport = pConfigManager->hReport;  
    Core_AdaptTx_config.hTnetwDrv = pConfigManager->hTnetwDrv;
    Core_AdaptTx_config.hTxData = pConfigManager->hTxData;
    Core_AdaptTx_config.hCtrlData = pConfigManager->hCtrlData;

    /* sendPacketComplete event callback */
    /*Core_AdaptTx_config.TxCmplt_CB = pCoreCallbacks->ctrlData_TxCompleteStatusCB;*/
    /*Core_AdaptTx_config.TxCmplt_CB_handle = pConfigManager->hCtrlData;*/

    /* sendPacketTransfer event callback */
    /*Core_AdaptTx_config.PacketTranfer_CB = pCoreCallbacks->txData_SendPacketTranferCB;*/
    /*Core_AdaptTx_config.PacketTranfer_CB_handle = pConfigManager->hTxData;*/

    /* queueFreeEvent event callback */
    /*Core_AdaptTx_config.QueueFreeEvent_CB = pCoreCallbacks->txData_QueueFreeEventCB;*/
    /*Core_AdaptTx_config.QueueFreeEvent_CB_handle = pConfigManager->hTxData;*/
    WLAN_OS_REPORT(("Initializing Core Adapter Tx...\n"));
    if (CORE_AdaptTx_Config (CORE_AdaptTx_handle, 
                             &Core_AdaptTx_config, 
                             &pConfigManager->pInitTable->txDataInitParams) != OK)
    {
        WLAN_OS_REPORT(("\n.....CORE ADAPTER TX configuration failure \n"));
            return NOK;
    }
    
    /* SCR module */
    WLAN_OS_REPORT(("Initializing SCR...\n"));
    scr_init (pConfigManager->hSCR, pConfigManager->hReport);
    
    /* connection module */
    WLAN_OS_REPORT(("Initializing Conn...\n"));
    if (conn_config (pConfigManager->hConn, 
                     pConfigManager->hSiteMgr,
                     pConfigManager->hSmeSm, 
                     pConfigManager->hMlmeSm,
                     pConfigManager->hRsn, 
                     pConfigManager->hRxData,
                     pConfigManager->hTxData, 
                     pConfigManager->hReport,
                     pConfigManager->hOs, 
                     pConfigManager->hPowerMgr,
                     pConfigManager->hCtrlData, 
                     pConfigManager->hMeasurementMgr,
                     pConfigManager->hTrafficMon, 
                     pConfigManager->hSCR,
                     pConfigManager->hExcMngr, 
                     pConfigManager->hQosMngr,
                     pConfigManager->hHalCtrl, 
                     pConfigManager->hScanCncn,    
                     pConfigManager->hCurrBss, 
                     pConfigManager->hSwitchChannel,
                     pConfigManager->hEvHandler, 
                     pConfigManager->hHealthMonitor,
                     pConfigManager->hMacServices,
                     pConfigManager->hRegulatoryDomain,
                     pConfigManager->hSoftGemini,
                     &pConfigManager->pInitTable->connInitParams) != OK)
    {
        WLAN_OS_REPORT(("\n.....Conn configuration failure \n"));
        return NOK;
    }

    /* Ctrl data module */
    WLAN_OS_REPORT(("Initializing Ctrl Data...\n"));
    if (ctrlData_config (pConfigManager->hCtrlData, 
                         pConfigManager->hHalCtrl, 
                         pConfigManager->hSiteMgr,
                         pConfigManager->hTxData, 
                         pConfigManager->hRxData, 
                         pConfigManager->hOs, 
                         pConfigManager->hReport, 
                         pConfigManager->hMemMgr, 
                         pConfigManager->hEvHandler, 
                         pConfigManager->hAPConnection, 
                         pConfigManager->hTrafficMon,
                         conn_disConnFrameSentCBFunc, 
                         pConfigManager->hConn, 
                         &pConfigManager->pInitTable->ctrlDataInitParams) != OK)
    {
        WLAN_OS_REPORT(("\n.....CTRL DATA configuration failure \n"));
        return NOK;
    }

    /* Site manager module */
    WLAN_OS_REPORT(("Initializing Site Manager...\n"));
    if (siteMgr_config (pConfigManager->hSiteMgr, 
                        pConfigManager->hConn, 
                        pConfigManager->hSmeSm,
                        pConfigManager->hCtrlData,
                        pConfigManager->hRxData, 
                        pConfigManager->hTxData,
                        pConfigManager->hRsn, 
                        pConfigManager->hAuth, 
                        pConfigManager->hAssoc,
                        pConfigManager->hHalCtrl,
                        pConfigManager->hMlmeSm, 
                        pConfigManager->hRegulatoryDomain,
                        pConfigManager->hMeasurementMgr,
                        pConfigManager->hAPConnection,
                        pConfigManager->hCurrBss,
                        pConfigManager->hReport, 
                        pConfigManager->hOs,
                        pConfigManager->hExcMngr,
                        pConfigManager->hQosMngr,
                        pConfigManager->hPowerMgr,
                        pConfigManager->hSCR, 
                        pConfigManager->hEvHandler, 
                        pConfigManager->hMacServices,
                        &pConfigManager->pInitTable->siteMgrInitParams) != OK)
    {
        WLAN_OS_REPORT(("\n.....Site manager configuration failure \n"));
        return NOK;
    }

    /* Note: must be configured after Site Manager object coonfiguration */
    /* regulatory Domain module */
    WLAN_OS_REPORT(("Initializing Regulatory Domain...\n"));
    if (regulatoryDomain_config (pConfigManager->hRegulatoryDomain,
                                 pConfigManager->hSiteMgr,
                                 pConfigManager->hHalCtrl,
                                 pConfigManager->hReport,
                                 pConfigManager->hOs, 
                                 pConfigManager->hSwitchChannel,
                                 &pConfigManager->pInitTable->regulatoryDomainInitParams) != OK)
    {
        WLAN_OS_REPORT(("\n.....Regulatory Domain configuration failure \n"));
        return NOK;
    }

    /* scan concentrator module */
    /* Note: must be configured after RegulatoryDomain object coonfiguration */
    WLAN_OS_REPORT(("Initializing Scan Concentrator...\n"));
    scanConcentrator_init (pConfigManager->hScanCncn, 
                           pConfigManager->hHalCtrl,
                           pConfigManager->hReport, 
                           pConfigManager->hRegulatoryDomain,
                           pConfigManager->hSiteMgr, 
                           pConfigManager->hSCR, 
                           pConfigManager->hMacServices,
                           pConfigManager->hAPConnection, 
                           pConfigManager->hEvHandler, 
                           pConfigManager->hMlmeSm, 
                           pConfigManager->hCtrlData,             
                           pConfigManager->hHealthMonitor,
                           &pConfigManager->pInitTable->scanConcentratorInitParams);
    
    /* AUTH module */
    WLAN_OS_REPORT(("Initializing Auth...\n"));
    if (auth_config (pConfigManager->hAuth, 
                     pConfigManager->hMlmeSm, 
                     pConfigManager->hRsn,
                     pConfigManager->hReport, 
                     pConfigManager->hOs,
                     &pConfigManager->pInitTable->authInitParams) != OK)
    {
        WLAN_OS_REPORT(("\n.....AUTH configuration failure \n"));
        return NOK;
    }

    /* MLME SM module */
    WLAN_OS_REPORT(("Initializing MLME...\n"));
    if (mlme_config (pConfigManager->hMlmeSm, 
                     pConfigManager->hAuth, 
                     pConfigManager->hAssoc,
                     pConfigManager->hSiteMgr, 
                     pConfigManager->hCtrlData, 
                     pConfigManager->hConn,
                     pConfigManager->hTxData, 
                     pConfigManager->hHalCtrl, 
                     pConfigManager->hMemMgr,
                     pConfigManager->hMeasurementMgr, 
                     pConfigManager->hSwitchChannel,
                     pConfigManager->hRegulatoryDomain, 
                     pConfigManager->hReport, 
                     pConfigManager->hOs,
                     pConfigManager->hCurrBss, 
                     pConfigManager->hAPConnection,
                     pConfigManager->hScanCncn, 
                     pConfigManager->hQosMngr, 
                     (TI_HANDLE)pConfigManager) != OK)    
    {
        WLAN_OS_REPORT(("\n.....MLME SM configuration failure \n"));
        return NOK;
    }

    /* ASSOC module */
    WLAN_OS_REPORT(("Initializing Assoc...\n"));
    if (assoc_config (pConfigManager->hAssoc,   
                      pConfigManager->hMlmeSm, 
                      pConfigManager->hRegulatoryDomain,
                      pConfigManager->hSiteMgr, 
                      pConfigManager->hCtrlData, 
                      pConfigManager->hTxData,
                      pConfigManager->hHalCtrl, 
                      pConfigManager->hRsn, 
                      pConfigManager->hReport,
                      pConfigManager->hOs, 
                      pConfigManager->hExcMngr,
                      pConfigManager->hQosMngr,
                      pConfigManager->hMeasurementMgr, 
                      pConfigManager->hAPConnection, 
                      &pConfigManager->pInitTable->assocInitParams) != OK)
    {
        WLAN_OS_REPORT(("\n.....ASSOC configuration failure \n"));
        return NOK;
    }

    /* Rx data module */
    WLAN_OS_REPORT(("Initializing Rx Data...\n"));
    if (rxData_config (pConfigManager->hRxData, 
                       pConfigManager->hCtrlData, 
                       pConfigManager->hTxData,
                       pConfigManager->hTnetwDrv, 
                       pConfigManager->hHalCtrl,
                       pConfigManager->hMlmeSm,
                       pConfigManager->hRsn, 
                       pConfigManager->hSiteMgr,
                       pConfigManager->hExcMngr, 
                       pConfigManager->hOs,
                       pConfigManager->hReport, 
                       pConfigManager->hMemMgr,
                       pConfigManager->hEvHandler,
                       &pConfigManager->pInitTable->rxDataInitParams) != OK)
    {
        WLAN_OS_REPORT(("\n.....RX DATA configuration failure \n"));
        return NOK;
    }

    /* Tx data module */
    WLAN_OS_REPORT(("Initializing Tx Data...\n"));
    if (txData_config (pConfigManager->hTxData, 
                       pConfigManager->hCtrlData, 
                       pConfigManager->hTnetwDrv, 
                       pConfigManager->hHalCtrl,
                       pConfigManager->hOs, 
                       pConfigManager->hReport, 
                       pConfigManager->hMemMgr, 
                       pConfigManager->hSiteMgr, 
                       pConfigManager->hEvHandler, 
                       pConfigManager->hQosMngr,
                       pConfigManager->hPowerMgr) != OK)
    {
        WLAN_OS_REPORT(("\n.....TX DATA configuration failure \n"));
        return NOK;
    }

    /* Traffic data module */
    WLAN_OS_REPORT(("Initializing Traffic Monitor...\n"));
    if (TrafficMonitor_Init(pConfigManager->hTrafficMon,
                            pConfigManager->hRxData,
                            pConfigManager->hTxData) != OK)
    {
        WLAN_OS_REPORT(("\n..... TRAFFIC MONITOR  configuration failure \n"));
        return NOK;
    }

    /* Memory Manager module */
    WLAN_OS_REPORT(("Initializing Memory Manager...\n"));
    if (wlan_memMngrConfigure (pConfigManager->hMemMgr, pConfigManager->hOs, pConfigManager->hReport) != OK)
    {
        WLAN_OS_REPORT(("\n.....MEM MNGR configuration failure \n"));
        return NOK;
    }

    /* sme state machine module */
    WLAN_OS_REPORT(("Initializing SME...\n"));
    if (smeSm_config (pConfigManager->hSmeSm, 
                      pConfigManager->hConn, 
                      pConfigManager->hScanCncn,
                      pConfigManager->hSiteMgr, 
                      pConfigManager->hHalCtrl, 
                      pConfigManager->hReport, 
                      pConfigManager->hOs, 
                      pConfigManager->hEvHandler, 
                      pConfigManager->hSCR, 
                      pConfigManager->hAPConnection, 
                      pConfigManager->hCurrBss,
                      pConfigManager->hPowerMgr,
                      pConfigManager->hRegulatoryDomain,
                      &pConfigManager->pInitTable->smeInitParams) != OK)
    {
        WLAN_OS_REPORT(("\n.....Sme state machine configuration failure \n"));
        return NOK;
    }

    /* Rsn module */
    WLAN_OS_REPORT(("Initializing RSN...\n"));
    if (rsn_config (pConfigManager->hRsn, 
                    pConfigManager->hTxData, 
                    pConfigManager->hRxData, 
                    pConfigManager->hConn,
                    pConfigManager->hMlmeSm, 
                    pConfigManager->hCtrlData, 
                    pConfigManager->hHalCtrl,
                    pConfigManager->hMemMgr, 
                    pConfigManager->hSiteMgr,
                    pConfigManager->hReport, 
                    pConfigManager->hOs, 
                    pConfigManager->hExcMngr,
                    pConfigManager->hPowerMgr,
                    pConfigManager->hEvHandler,
                    pConfigManager->hSmeSm, 
                    pConfigManager->hAPConnection,
                    &pConfigManager->pInitTable->rsnInitParams) != OK)
    {
        WLAN_OS_REPORT(("\n.....RSN configuration failure \n"));
        return NOK;
    }

    /* MeasurementMgr module */
    /* Note: must be configured after RegulatoryDomain object coonfiguration */
    WLAN_OS_REPORT(("Initializing Measurement Manager...\n"));
    if (measurementMgr_config (pConfigManager->hMeasurementMgr, 
                               pConfigManager->hMacServices,
                               pConfigManager->hRegulatoryDomain, 
                               pConfigManager->hExcMngr,
                               pConfigManager->hSiteMgr, 
                               pConfigManager->hHalCtrl,
                               pConfigManager->hMlmeSm, 
                               pConfigManager->hTrafficMon,
                               pConfigManager->hReport, 
                               pConfigManager->hOs,
                               pConfigManager->hSCR,
                               pConfigManager->hHealthMonitor,
                               pConfigManager->hAPConnection, 
                               pConfigManager->hTxData,
                               &pConfigManager->pInitTable->measurementInitParams) != OK)
    {
        WLAN_OS_REPORT(("\n.....MeasurementMgr configuration failure \n"));
        return NOK;
    }

#ifdef EXC_MODULE_INCLUDED

    WLAN_OS_REPORT(("Initializing EXC Manager...\n"));
    if (excMngr_config (pConfigManager->hExcMngr, 
                        pConfigManager->hReport, 
                        pConfigManager->hOs,
                        pConfigManager->hRsn, 
                        pConfigManager->hMemMgr,
                        pConfigManager->hCtrlData, 
                        pConfigManager->hTxData,
                        pConfigManager->hSiteMgr, 
                        pConfigManager->hAPConnection, 
                        pConfigManager->hEvHandler,
                        (TI_HANDLE)pConfigManager, 
                        pConfigManager->hMeasurementMgr, 
                        pConfigManager->hQosMngr, 
                        &pConfigManager->pInitTable->excMngrParams) != OK)
    {
        WLAN_OS_REPORT(("\n.....excMngr_config configuration failure \n"));
        return NOK;
    }

#endif
    /* Scan manager */
    WLAN_OS_REPORT(("Initializing Scan Manager...\n"));
    scanMngr_init (pConfigManager->hScanMngr, 
                   pConfigManager->hReport, 
                   pConfigManager->hRegulatoryDomain,
                   pConfigManager->hScanCncn, 
                   pConfigManager->hRoamingMngr, 
                   pConfigManager->hSiteMgr,
                   pConfigManager->hHalCtrl);



    WLAN_OS_REPORT(("Initializing CurrBSS...\n"));
    if (currBSS_init (pConfigManager->hCurrBss, 
                      pConfigManager->hMlmeSm,
                      pConfigManager->hPowerMgr,
                      pConfigManager->hAPConnection, 
                      pConfigManager->hSmeSm,
                      pConfigManager->hHalCtrl, 
                      pConfigManager->hReport, 
                      pConfigManager->hMemMgr, 
                      pConfigManager->hTxData, 
                      pConfigManager->hSiteMgr, 
                      pConfigManager->hScanMngr, 
                      pConfigManager->hMacServices) != OK)
    {
        WLAN_OS_REPORT(("\n.....currBSS_init configuration failure \n"));
        return NOK;
    }

    WLAN_OS_REPORT(("Initializing AP Conn...\n"));
    if (apConn_config (pConfigManager->hAPConnection, 
                       pConfigManager->hReport, 
                       pConfigManager->hCurrBss,
                       pConfigManager->hRoamingMngr,
                       pConfigManager->hSmeSm, 
                       pConfigManager->hSiteMgr,
                       pConfigManager->hExcMngr, 
                       pConfigManager->hConn,
                       pConfigManager->hRsn, 
                       pConfigManager->hQosMngr,
                       pConfigManager->hCtrlData,
                       pConfigManager->hEvHandler,
                       pConfigManager->hSCR,
                       pConfigManager->hAssoc,
                       pConfigManager->hRegulatoryDomain,
                       &pConfigManager->pInitTable->apConnParams) != OK)
    {
        WLAN_OS_REPORT(("\n.....apConn_config configuration failure \n"));
        return NOK;
    }

    WLAN_OS_REPORT(("Initializing Roaming Manager...\n"));
    if (roamingMngr_init (pConfigManager->hRoamingMngr, 
                          pConfigManager->hReport,
                          pConfigManager->hScanMngr,
                          pConfigManager->hAPConnection) != OK)
    {
        WLAN_OS_REPORT(("\n.....roamingMngr_config configuration failure \n"));
        return NOK;
    }

    /* NOTE: must be after siteMgr & whalCtrl configurations !!!! */
    WLAN_OS_REPORT(("Initializing QoS Manager...\n"));
    if (qosMngr_config (pConfigManager->hQosMngr,   
                        pConfigManager->hHalCtrl,
                        pConfigManager->hSiteMgr, 
                        pConfigManager->hReport,
                        pConfigManager->hOs, 
                        pConfigManager->hTxData,
                        pConfigManager->hMeasurementMgr,
                        pConfigManager->hSmeSm, 
                        pConfigManager->hMemMgr,
                        pConfigManager->hCtrlData,
                        pConfigManager->hEvHandler, 
                        pConfigManager->hExcMngr, 
                        &pConfigManager->pInitTable->qosMngrInitParams) != OK)
    {
        WLAN_OS_REPORT(("\n.....Qos Mngr configuration failure \n"));
        return NOK;
    }

    WLAN_OS_REPORT(("Initializing Switch Channel...\n"));
    if (switchChannel_config (pConfigManager->hSwitchChannel,
                              pConfigManager->hHalCtrl,
                              pConfigManager->hSiteMgr,
                              pConfigManager->hSCR,
                              pConfigManager->hRegulatoryDomain,
                              pConfigManager->hAPConnection,
                              pConfigManager->hReport,
                              pConfigManager->hOs,
                              pConfigManager->hHealthMonitor,
                              &pConfigManager->pInitTable->SwitchChannelInitParams) != OK)
    {
        WLAN_OS_REPORT(("\n.....SwitchChannel_config configuration failure \n"));
        return NOK;
    }

    WLAN_OS_REPORT(("Initializing Health Monitor...\n"));
    if (healthMonitor_config (pConfigManager->hHealthMonitor,
                              pConfigManager->hReport,
                              pConfigManager->hHalCtrl,
                              pConfigManager->hSiteMgr,
                              pConfigManager->hSCR,
                              pConfigManager->hSoftGemini,
                              pConfigManager->hTnetwDrv,
                              pConfigManager->hMemMgr,
                              (TI_HANDLE)pConfigManager,
                              pConfigManager->hTxData,
                              pConfigManager->hCurrBss,
                              pConfigManager->hRsn,
                              &pConfigManager->pInitTable->healthMonitorInitParams,
                              pConfigManager->hRecoveryMgr) != OK)
    {
        WLAN_OS_REPORT(("\n.....healthMonitor_config configuration failure \n"));
        return NOK;
    }

    WLAN_OS_REPORT(("Initializing Power Manager...\n"));
    if (PowerMgr_init (pConfigManager->hPowerMgr,
                       pConfigManager->hMacServices,
                       pConfigManager->hReport,
                       pConfigManager->hSiteMgr,
                       pConfigManager->hHalCtrl,
                       pConfigManager->hTrafficMon,
                       pConfigManager->hSoftGemini,
                       &pConfigManager->pInitTable->PowerMgrInitParams) != OK)
    {
        WLAN_OS_REPORT(("\n.....PowerMgr_init configuration failure \n"));
        return NOK;
    }

    WLAN_OS_REPORT(("Initializing Recovery Mgr...\n"));
    if (recoveryMgr_config(pConfigManager->hRecoveryMgr, 
                           pConfigManager->hReport,
                           pConfigManager->hTxData,
                           pConfigManager->hTnetwDrv,
                           pConfigManager->hSCR,     
                           pConfigManager->hCurrBss, 
                           pConfigManager->hPowerMgr,
                           pConfigManager->hHealthMonitor,
						   pConfigManager->hSoftGemini) != OK)
    {
        WLAN_OS_REPORT(("\n.....RecoveryMgr configuration failure \n"));
        return NOK;
    }

	/* This must be called before calling SoftGemini_config, as the SG may trigger events from FW
		which are enabled in this fucntion */
	whalCtrl_exitFromInitModePart1(pConfigManager->hHalCtrl);

    /* Soft Gemini module , should be configured after all the modules it uses */
    WLAN_OS_REPORT(("Initializing Soft Gemini...\n"));
    if (SoftGemini_config (pConfigManager->hSoftGemini,
                           pConfigManager->hCtrlData,
                           pConfigManager->hHalCtrl,
                           pConfigManager->hReport,
                           pConfigManager->hSCR,
                           pConfigManager->hPowerMgr,
                           (TI_HANDLE)pConfigManager,
                           pConfigManager->hScanCncn,
                           pConfigManager->hCurrBss,
                           pConfigManager->hEvHandler,
                           &pConfigManager->pInitTable->SoftGeminiInitParams) != OK)                                                                                
    {
        WLAN_OS_REPORT(("\n.....SoftGemini configuration failure \n"));
        return NOK;
    }

    /* Notify the power authorization so the first min power level will be sent to the FW */
       /* This will update the FW Power Level according to the defaultPowerLevel configured in Registry */
    MacServices_powerAutho_ExitFromInit(pConfigManager->hMacServices);

    /*
    *  Exit from init mode should be before smeSM starts. this enable us to send
    *  command to the MboxQueue(that store the command) while the interrupts are masked.
    *  the interrupt would be enable at the end of the init process.
    */
	whalCtrl_exitFromInitModePart2(pConfigManager->hHalCtrl);
    
    WLAN_OS_REPORT(("Finished initializing modules.\n"));

    WLAN_REPORT_INIT(pConfigManager->hReport, CONFIG_MGR_MODULE_LOG,  
        ("EXIT FROM INIT\n"));

    return OK;
}


/*
 * configMgr_RetrieveFWInfo: 
 * Retrieve FW information from HAL which the SNWSASettings returned 
 * by GWSI_Configure does not contained
 */
static  void configMgr_RetrieveFWInfo(configMgr_t *pConfigManager, initTable_t *pInitTable, whalCtrl_chip_t *pChip_Version)
{
    /*
     * Retrieve FW information from HAL
     */
    whalCtrl_GetFWInfo(pConfigManager->hHalCtrl, pChip_Version);

    /* 
     * Update complete the init table update 
     * with the chip information received from the HAL
     */
    os_memoryCopy(pConfigManager->hOs, (void *)pInitTable->ctrlDataInitParams.ctrlDataDeviceMacAddress.addr, (void *)pChip_Version->macAddress.addr, MAC_ADDR_LEN);
    os_memoryCopy(pConfigManager->hOs, pInitTable->siteMgrInitParams.siteMgrFwVersion, pChip_Version->fwVer, FW_VERSION_LEN);
    os_memoryCopy(pConfigManager->hOs, &(pInitTable->siteMgrInitParams.siteMgrEEpromVersion), &(pChip_Version->e2Ver), sizeof(e2Version_t));
    pInitTable->siteMgrInitParams.siteMgrRadioValues.siteMgr_radioType = pChip_Version->radioType;   
}


/****************************************************************************************
 *                        configMgr_GetInitParams                                                *
 ****************************************************************************************
DESCRIPTION:    Retreive init table

INPUT:          pConfigManager     - driver main handle

OUTPUT:         ioBuffer           - init table
                outBufLen          - init table length

RETURN:         none

************************************************************************/
void configMgr_GetInitParams (TI_HANDLE hConfigManager, UINT8* ioBuffer, UINT16 *outBufLen)
{
    configMgr_t *pConfigManager = (configMgr_t *)hConfigManager;

    TnetwDrv_GetInitParams (pConfigManager->hTnetwDrv, ioBuffer, outBufLen);
}


/****************************************************************************************
 *                        release_module                                                *
 ****************************************************************************************
DESCRIPTION:    Driver free function. Performs the following:
                -   Go over the vector, for each bit that is set, release the corresponding module.

INPUT:          pConfigManager     -    Driver main handle
                initVec         -       Vector that contains the bits of the modules which have to be free


OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
static void release_module(configMgr_t *pConfigManager)
{
    if (pConfigManager->hScanMngr != NULL)
    {
        scanMngr_unload( pConfigManager->hScanMngr);
    }

    if (pConfigManager->hSiteMgr != NULL)
    {
        siteMgr_unLoad(pConfigManager->hSiteMgr);
    }

    if (pConfigManager->hSmeSm != NULL)
    {
        smeSm_unLoad(pConfigManager->hSmeSm);
    }

    if (pConfigManager->hConn != NULL)
    {
        conn_unLoad(pConfigManager->hConn);
    }

    if (pConfigManager->hTnetwDrv != NULL)
    {
        TnetwDrv_Destroy(pConfigManager->hTnetwDrv);
    }

    if (pConfigManager->hScanCncn != NULL)
    {
        scanConcentrator_release(pConfigManager->hScanCncn);
    }

    if (pConfigManager->hTrafficMon != NULL)
    {
        TrafficMonitor_Destroy(pConfigManager->hTrafficMon);
    }

    if (pConfigManager->hCtrlData != NULL)
    {
        ctrlData_unLoad(pConfigManager->hCtrlData);
    }

    if (pConfigManager->hTxData != NULL)
    {
        txData_unLoad(pConfigManager->hTxData);
    }

    if (pConfigManager->hRxData != NULL)
    {
        rxData_unLoad(pConfigManager->hRxData);
    }

    if (pConfigManager->hAssoc != NULL)
    {
        assoc_unload(pConfigManager->hAssoc);
    }

    if (pConfigManager->hAuth != NULL)
    {
        auth_unload(pConfigManager->hAuth);
    }

    if (pConfigManager->hMlmeSm != NULL)
    {
        mlme_unload(pConfigManager->hMlmeSm);
    }

    if (pConfigManager->hSCR != NULL)
    {
        scr_release(pConfigManager->hSCR);
    }

    if (pConfigManager->hEvHandler != NULL)
    {
         EvHandlerUnload(pConfigManager->hEvHandler);
    }

    if (pConfigManager->hMemMgr != NULL)
    {
        wlan_memMngrDestroy(pConfigManager->hMemMgr);
    }

    if (pConfigManager->hRsn != NULL)
    {
        rsn_unload(pConfigManager->hRsn);
    }

    if (pConfigManager->hRegulatoryDomain != NULL)
    {
        regulatoryDomain_destroy(pConfigManager->hRegulatoryDomain);
    }

    if (pConfigManager->hMeasurementMgr != NULL)
    {
        measurementMgr_destroy(pConfigManager->hMeasurementMgr);
    }

    if (pConfigManager->hSoftGemini != NULL)
    {
        SoftGemini_destroy(pConfigManager->hSoftGemini);
    }

#ifdef EXC_MODULE_INCLUDED
    if (pConfigManager->hExcMngr != NULL)
    {
        excMngr_unload(pConfigManager->hExcMngr);
    }
#endif

    if (pConfigManager->hRoamingMngr != NULL)
    {
        roamingMngr_unload(pConfigManager->hRoamingMngr);
    }

    if (pConfigManager->hQosMngr != NULL)
    {
        qosMngr_destroy(pConfigManager->hQosMngr);
    }

    if (pConfigManager->hPowerMgr != NULL)
    {
        PowerMgr_destroy(pConfigManager->hPowerMgr);
    }

    if (pConfigManager->hAPConnection != NULL)
    {
        apConn_unload(pConfigManager->hAPConnection);
    }

    if (pConfigManager->hCurrBss != NULL)
    {
        currBSS_unload(pConfigManager->hCurrBss);
    }

    if (pConfigManager->hSwitchChannel != NULL)
    {
        switchChannel_unload(pConfigManager->hSwitchChannel);
    }

    if (pConfigManager->hHealthMonitor != NULL)
    {
        healthMonitor_unload(pConfigManager->hHealthMonitor);
    }

    if (pConfigManager->hRecoveryMgr != NULL)
    {
        recoveryMgr_destroy(pConfigManager->hRecoveryMgr);
    }

    if (pConfigManager->hReport != NULL)
    {
        report_unLoad(pConfigManager->hReport);
    }
   
    /***************************************************************
    This is the config manager, it should be always the last module 
    to release
    ****************************************************************/
    
    utils_nullMemoryFree(pConfigManager->hOs, pConfigManager, sizeof(configMgr_t));
}

/****************************************************************************************
 *                        configParamsAccessTable                                       *
 ****************************************************************************************
DESCRIPTION:    Called in the configuration phase by the driver, performs the following:
                -   For each module that supply a Get/Set services to his parameters, fill the corresponding entry
                    in the params access table with the following:
                        -   Get function
                        -   Set function
                        -   Handle to the module
                This table is used when Getting/Setting a parameter from the OS abstraction layer.

INPUT:          pConfigManager     -    Driver main handle

OUTPUT:

RETURN:         OK on success, NOK on failure

************************************************************************/
static void configParamsAccessTable(configMgr_t *pConfigManager)
{
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(AUTH_MODULE_PARAM) - 1].set = auth_setParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(AUTH_MODULE_PARAM) - 1].get = auth_getParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(AUTH_MODULE_PARAM) - 1].handle = pConfigManager->hAuth;

    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(ASSOC_MODULE_PARAM) - 1].set = assoc_setParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(ASSOC_MODULE_PARAM) - 1].get = assoc_getParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(ASSOC_MODULE_PARAM) - 1].handle = pConfigManager->hAssoc;

    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(RX_DATA_MODULE_PARAM) - 1].set = rxData_setParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(RX_DATA_MODULE_PARAM) - 1].get = rxData_getParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(RX_DATA_MODULE_PARAM) - 1].handle = pConfigManager->hRxData;

    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(TX_DATA_MODULE_PARAM) - 1].set = txData_setParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(TX_DATA_MODULE_PARAM) - 1].get = txData_getParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(TX_DATA_MODULE_PARAM) - 1].handle = pConfigManager->hTxData;

    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(CTRL_DATA_MODULE_PARAM) - 1].set = ctrlData_setParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(CTRL_DATA_MODULE_PARAM) - 1].get = ctrlData_getParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(CTRL_DATA_MODULE_PARAM) - 1].handle = pConfigManager->hCtrlData;

    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(SITE_MGR_MODULE_PARAM) - 1].set = siteMgr_setParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(SITE_MGR_MODULE_PARAM) - 1].get = siteMgr_getParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(SITE_MGR_MODULE_PARAM) - 1].handle = pConfigManager->hSiteMgr;

    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(CONN_MODULE_PARAM) - 1].set = conn_setParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(CONN_MODULE_PARAM) - 1].get = conn_getParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(CONN_MODULE_PARAM) - 1].handle = pConfigManager->hConn;

    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(RSN_MODULE_PARAM) - 1].set = rsn_setParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(RSN_MODULE_PARAM) - 1].get = rsn_getParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(RSN_MODULE_PARAM) - 1].handle= pConfigManager->hRsn;

    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(HAL_CTRL_MODULE_PARAM) - 1].set = (paramFunc_t)whalCtrl_SetParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(HAL_CTRL_MODULE_PARAM) - 1].get = (paramFunc_t)whalCtrl_GetParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(HAL_CTRL_MODULE_PARAM) - 1].handle = pConfigManager->hHalCtrl;

    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(REPORT_MODULE_PARAM) - 1].set = (paramFunc_t)report_setParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(REPORT_MODULE_PARAM) - 1].get = (paramFunc_t)report_getParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(REPORT_MODULE_PARAM) - 1].handle = pConfigManager->hReport;

    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(SME_SM_MODULE_PARAM) - 1].set = smeSm_setParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(SME_SM_MODULE_PARAM) - 1].get = smeSm_getParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(SME_SM_MODULE_PARAM) - 1].handle = pConfigManager->hSmeSm;

    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(SCAN_CNCN_PARAM) - 1].set = scanConcentrator_setParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(SCAN_CNCN_PARAM) - 1].get = scanConcentrator_getParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(SCAN_CNCN_PARAM) - 1].handle = pConfigManager->hScanCncn;
    
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(SCAN_MNGR_PARAM) - 1].set = scanMngr_setParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(SCAN_MNGR_PARAM) - 1].get = scanMngr_getParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(SCAN_MNGR_PARAM) - 1].handle = pConfigManager->hScanMngr;
    
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(MLME_SM_MODULE_PARAM) - 1].set = mlme_setParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(MLME_SM_MODULE_PARAM) - 1].get = mlme_getParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(MLME_SM_MODULE_PARAM) - 1].handle = pConfigManager->hMlmeSm;

    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(REGULATORY_DOMAIN_MODULE_PARAM) - 1].set = regulatoryDomain_setParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(REGULATORY_DOMAIN_MODULE_PARAM) - 1].get = regulatoryDomain_getParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(REGULATORY_DOMAIN_MODULE_PARAM) - 1].handle = pConfigManager->hRegulatoryDomain;

    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(MEASUREMENT_MODULE_PARAM) - 1].set = measurementMgr_setParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(MEASUREMENT_MODULE_PARAM) - 1].get = measurementMgr_getParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(MEASUREMENT_MODULE_PARAM) - 1].handle = pConfigManager->hMeasurementMgr;

#ifdef EXC_MODULE_INCLUDED
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(EXC_MANAGER_MODULE_PARAM) - 1].set = excMngr_setParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(EXC_MANAGER_MODULE_PARAM) - 1].get = excMngr_getParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(EXC_MANAGER_MODULE_PARAM) - 1].handle = pConfigManager->hExcMngr;
#endif

    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(ROAMING_MANAGER_MODULE_PARAM) - 1].set = roamingMngr_setParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(ROAMING_MANAGER_MODULE_PARAM) - 1].get = roamingMngr_getParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(ROAMING_MANAGER_MODULE_PARAM) - 1].handle = pConfigManager->hRoamingMngr;

    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(SOFT_GEMINI_PARAM) - 1].set = SoftGemini_setParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(SOFT_GEMINI_PARAM) - 1].get = SoftGemini_getParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(SOFT_GEMINI_PARAM) - 1].handle = pConfigManager->hSoftGemini;

    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(QOS_MANAGER_PARAM) - 1].set = qosMngr_setParams;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(QOS_MANAGER_PARAM) - 1].get = qosMngr_getParams;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(QOS_MANAGER_PARAM) - 1].handle = pConfigManager->hQosMngr;

    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(POWER_MANAGER_PARAM) - 1].set = powerMgr_setParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(POWER_MANAGER_PARAM) - 1].get = powerMgr_getParam;
    pConfigManager->paramAccessTable[GET_PARAM_MODULE_NUMBER(POWER_MANAGER_PARAM) - 1].handle = pConfigManager->hPowerMgr;
}

