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


/*******************************************************************************/
/*                                                                             */
/*  MODULE:  hwInit.c                                                          */
/*  PURPOSE: HwInit module manages the init process of the TNETW, included     */
/*           firmware download process. It shall perform Hard Reset the chip   */
/*           if possible (this will require a Reset line to be connected to    */
/*           the host); Start InterfaceCtrl; Download NVS and FW               */
/*                                                                             */
/*                                                                             */
/*******************************************************************************/

#include "paramOut.h"
#include "osApi.h"
#include "report.h"
#include "HwInit.h"
#include "HwInit_api.h"

#include "whalCommon.h"
#include "whalBus_Api.h"
#include "shmBus.h"

#include "FwEvent_api.h"
#include "TNETWIF.h"
#include "shmFwCtrl.h"
#include "TNETW_Driver.h"
#include "TNETW_Driver_api.h"
#include "whalHwCtrl.h"

#include "whalHwAccess.h"
#include "eventMbox_api.h"

/* static function */
#ifdef USE_RECOVERY
static TI_STATUS InitHw_StartInit(TI_HANDLE hHwCtrl);
#endif /* USE_RECOVERY */

/*******************************************************************************
*                       PUBLIC  FUNCTIONS  IMPLEMENTATION                      *
********************************************************************************/


/*************************************************************************
*                        hwInit_create                              *
**************************************************************************
* DESCRIPTION:  This function initializes the HwInit module.
*
* INPUT:        hOs - handle to Os Abstraction Layer
*               
* RETURN:       Handle to the allocated HwInit module
*************************************************************************/
TI_HANDLE hwInit_create(TI_HANDLE hOs)
{
#ifdef USE_RECOVERY
    hwInit_t *hHwInit;

    /* allocate HwInit module */
    hHwInit = os_memoryAlloc(hOs, (sizeof(hwInit_t)));

    if(!hHwInit)
    {
        WLAN_OS_REPORT(("Error allocating the HwInit Module\n"));
        return NULL;
    }

    /* Reset HwInit module */
    os_memoryZero(hOs, hHwInit, (sizeof(hwInit_t)));

    hHwInit->hOs = hOs;

    return(hHwInit);
#else
    return NULL;
#endif /* USE_RECOVERY */
} /* hwInit_create */


/***************************************************************************
*                           hwInit_config                             *
****************************************************************************
* DESCRIPTION:  This function configures the hwInit module
*
* RETURNS:      OK - Configuration successful
*               NOK - Configuration unsuccessful
***************************************************************************/
TI_STATUS hwInit_config(TI_HANDLE hHwInit,
                        TI_HANDLE hReport,
                        TI_HANDLE hTNETWIF)
{
#ifdef USE_RECOVERY
    hwInit_t *pHwInit = (hwInit_t *)hHwInit;

    /* configure modules handles */
    pHwInit->hReport = hReport;
    pHwInit->hTNETWIF = hTNETWIF;

    pHwInit->smState = HW_INIT_STATE_IDLE;
    
    WLAN_REPORT_INIT(pHwInit->hReport, HW_INIT_MODULE_LOG,
        (".....HwInit configured successfully\n"));
#endif /* USE_RECOVERY */
    return OK;
    
} /* hwInit_config */


/***************************************************************************
*                           hwInit_destroy                            *
****************************************************************************
* DESCRIPTION:  This function unload the HwInit module. 
*
* INPUTS:       hHwInit - the object
*
* OUTPUT:
*
* RETURNS:      OK - Unload succesfull
*               NOK - Unload unsuccesfull
***************************************************************************/
TI_STATUS hwInit_destroy(TI_HANDLE hHwInit)
{
#ifdef USE_RECOVERY
    hwInit_t *pHwInit = (hwInit_t *)hHwInit;

    /* free HwInit Module */
    os_memoryFree(pHwInit->hOs, pHwInit, sizeof(hwInit_t));
#endif /* USE_RECOVERY */
    return OK;
}

/***************************************************************************
*                           hwInit_recovery                                *
****************************************************************************
* DESCRIPTION:  Start HW init process after recovery. 
*
* INPUTS:       hHwInit - the object
*
* OUTPUT:
*
* RETURNS:      
*    TNETWIF_COMPLETE - if completed, i.e. Synchronous mode.
*    TNETWIF_PENDING  - if pending, i.e. Asynchronous mode (callback function will be called). 
***************************************************************************/
#ifdef USE_RECOVERY
TI_STATUS hwInit_recovery(TI_HANDLE hHwInit, TI_HANDLE hHwCtrl, void *funcCB, TI_HANDLE hRecoveryCtrl)
{
    hwInit_t *pHwInit = (hwInit_t *)hHwInit;
    HwCtrl_T *pHwCtrl = (HwCtrl_T *)hHwCtrl;

    pHwInit->hHwCtrl = hHwCtrl;
    pHwInit->recoveryProcess = TRUE;
    pHwInit->hRecoveryCtrl = hRecoveryCtrl;
    pHwInit->endOfHwInitCB = (EndOfHwInitCB_t)funcCB;

    eventMbox_Stop(pHwCtrl->hEventMbox);

    TNETWIF_Start (pHwInit->hTNETWIF, HAL_INIT_MODULE_ID, hHwCtrl, (TNETWIF_callback_t)InitHw_StartInit);

    return TNETWIF_PENDING;
}
#endif /* USE_RECOVERY */

/****************************************************************************
 *                      InitHw_FinalizeDownload()
 ****************************************************************************
 * DESCRIPTION: Different FinalizeDownload for Init and Recovery. (Don't call
 *              for "config" functions after recovery).
 * 
 * INPUTS:  hHwInit - the object    
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
TI_STATUS InitHw_FinalizeDownload(TI_HANDLE hHwInit)
{
#ifdef USE_RECOVERY
    hwInit_t *pHwInit = (hwInit_t *)hHwInit;
    TI_STATUS status;

    status = (TI_STATUS)whal_hwCtrl_ConfigHw(pHwInit->hHwCtrl, (void *)pHwInit->endOfHwInitCB, pHwInit->hRecoveryCtrl, TRUE);
    if (status != OK)
    {
        WLAN_REPORT_INFORMATION (pHwInit->hReport, HW_INIT_MODULE_LOG,
            ("InitHw_FinalizeDownload: whal_hwCtrl_ConfigHw failed\n"));
        return TNETWIF_ERROR;
    }
#endif /* USE_RECOVERY */
    return TNETWIF_COMPLETE;

}


/****************************************************************************
 *                      InitHw_StartInit()
 ****************************************************************************
 * DESCRIPTION: start init process for recovery
 * 
 * INPUTS:  hHwInit - the object    
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
#ifdef USE_RECOVERY
static TI_STATUS InitHw_StartInit(TI_HANDLE hHwCtrl)
{
    HwCtrl_T *pHwCtrl = (HwCtrl_T *)hHwCtrl;
    WlanParams_T *pWlanParams;
    BootAttr_T BootAttr;
    TI_STATUS retStatus;

    whalBus_T *pWhalBus;
    PUINT8  FwBuf, EEpromBuf;
    UINT32  FwSize, EEpromSize;
    UINT32 *pWLAN_Images[4];

    pWlanParams = whal_ParamsGetWlanParams(pHwCtrl->pWhalParams);

    BootAttr.MacClock          = pWlanParams->MacClock;
    BootAttr.ArmClock          = pWlanParams->ArmClock;

    {
        /* Get the FW image */
        os_getFirmwareImage (pHwCtrl->hOs, &FwBuf, &FwSize, 0);
        os_getRadioImage (pHwCtrl->hOs, &EEpromBuf, &EEpromSize, 0);
        pWhalBus = (whalBus_T *)pHwCtrl->hWhalBus;
        pWLAN_Images[0] = (UINT32 *)FwBuf;
        pWLAN_Images[1] = (UINT32 *)FwSize;
        pWLAN_Images[2] = (UINT32 *)EEpromBuf;
        pWLAN_Images[3] = (UINT32 *)EEpromSize;
    
        pHwCtrl->uFwBuf = (UINT32)pWLAN_Images[0]; /* Firmware Image ptr */ 
        pHwCtrl->uFwAddr = (UINT32)pWLAN_Images[1]; /* Firmware Image length */
        pHwCtrl->uEEEPROMBuf = (UINT32)pWLAN_Images[2]; /* EEPROM Image ptr */
        pHwCtrl->uEEEPROMLen = (UINT32)pWLAN_Images[3]; /* EEPROM Image length */    
    }

    /* Reset the TNETW by the reset line */
    WLAN_OS_REPORT(("HARD RESET before\n"));
    os_hardResetTnetw ();
    WLAN_OS_REPORT(("HARD RESET after\n"));

    pWhalBus->recoveryProcess = TRUE;

    /* SDIO_enumerate */
    {
        TNETWIF_t *pTNETWIF = (TNETWIF_t *)(pWhalBus->hTNETWIF);
        whal_hwAccess_ReConfig(pTNETWIF->hHwAccess); /* SDIO enumerate*/
    }
    /* set the working partition to its "running" mode offset */
#if (defined(HW_ACCESS_SDIO)|defined(HW_ACCESS_WSPI))

    TNETWIF_SetPartitions (pWhalBus->hTNETWIF, HW_ACCESS_DOWNLOAD, HW_ACCESS_DOWN_PART0_ADDR);

#endif
    TNETWIF_RegSetBitVal(pWhalBus->hTNETWIF,  ACX_REG_ECPU_CONTROL, ECPU_CONTROL_HALT);

    retStatus = whalBus_FwCtrl_Boot (pHwCtrl->hWhalBus, (TI_HANDLE)pHwCtrl, &BootAttr);

    /* release FW and NVS images */
    /*
     * Note: This is done assuming the boot process is complelty synchronous!!!
     * Also, this is not done in GWSI becaues in GWSI these functions are not defined,
     * because the load process is different. This has also to be fixed...
     */
    os_closeFirmwareImage (pHwCtrl->hOs);
    os_closeRadioImage (pHwCtrl->hOs);

    return(retStatus);
}
#endif /* USE_RECOVERY */


