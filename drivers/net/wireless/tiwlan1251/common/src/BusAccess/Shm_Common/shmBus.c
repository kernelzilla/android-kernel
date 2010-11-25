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
 *   MODULE:  whalBus_Api.c
 *   PURPOSE: shared memory bus access component API
 *
 ****************************************************************************/
#include "whalCommon.h"
#include "whalCtrl.h"
#include "whalBus_Api.h"
#include "shmBus.h"
#include "TNETWIF.h"
#include "TNETWArb.h"
#include "TNETW_Driver.h"
#include "whalHwAccess.h"
#include "CmdMBox_api.h"
#include "eventMbox_api.h"
#include "FwEvent_api.h"


/* Handle return status inside a state machine */
#define EXCEPT(pwhalbus,status)                                 \
    switch (status) {                                           \
        case OK:                                                \
        case TNETWIF_COMPLETE:                                  \
             break;                                             \
        case TNETWIF_PENDING:                                   \
             return;                                            \
        default:                                                \
             whal_hwCtrl_FinalizeOnFailure (pwhalbus->hHwCtrl); \
             return;                                            \
    }


 /****************************************************************************
 *                      static function declaration
 *****************************************************************************/
static void whalBus_ConfigSm (TI_HANDLE hWhalBus, UINT8 module_id, TI_STATUS status);

 /****************************************************************************
 *                      whalBus_Create()
 ****************************************************************************
 * DESCRIPTION: Create the Bus access component
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: The Created object
 ****************************************************************************/
TI_HANDLE whalBus_Create (TI_HANDLE hOs)
{
    whalBus_T *pWhalBus;

    pWhalBus = os_memoryAlloc (hOs, sizeof(whalBus_T));
    if (pWhalBus == NULL)
        return NULL;

    os_memoryZero (hOs, pWhalBus, sizeof(whalBus_T));

    pWhalBus->hOs = hOs;

    pWhalBus->hTNETWIF  = TNETWIF_Create (hOs);
    pWhalBus->pHwEeprom = whal_hwEeprom_Create (hOs);

  #ifdef TI_DBG
    pWhalBus->pTrc      = whal_traceCreate(hOs);
  #else
    pWhalBus->pTrc      = NULL;
  #endif
    
    if (!pWhalBus->hTNETWIF || !pWhalBus->pHwEeprom)
    {
        whalBus_Destroy ((TI_HANDLE)pWhalBus);
        return NULL;
    }

    return (TI_HANDLE)pWhalBus;
}

/****************************************************************************
 *                      whalBus_Destroy()
 ****************************************************************************
 * DESCRIPTION: Destroy the object 
 * 
 * INPUTS:  
 *      hWhalBus        The object to free
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whalBus_Destroy(TI_HANDLE hWhalBus)
{
    whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;

    if (pWhalBus == NULL)
        return OK;

    whal_hwEeprom_Destroy(pWhalBus->pHwEeprom);
    
#ifdef TI_DBG
    whal_traceDestroy(pWhalBus->pTrc);
#endif
    TNETWIF_Destroy(pWhalBus->hTNETWIF);

    os_memoryFree(pWhalBus->hOs, pWhalBus, sizeof(whalBus_T));
    return OK;
}


/****************************************************************************
 *                      whalBus_ConfigSm()
 ****************************************************************************
 * DESCRIPTION: Config the object 
 * 
 * INPUTS:  
 *      hWhalBus        The object to free
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
static void whalBus_ConfigSm (TI_HANDLE hWhalBus, UINT8 module_id, TI_STATUS status)
{
    whalBus_T  *pWhalBus  = (whalBus_T *)hWhalBus;
    WHAL_CTRL  *pWhalCtrl = (WHAL_CTRL *)pWhalBus->hWhalCtrl;   
    TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)pWhalCtrl->hTNETW_Driver;

    /* Pass the TNETWIF handle in each SHM Bus Module : HwIntr,HwRx,HwTx,Hw_Mbox,Hw_EventMbox */
    /* From now on these modules will  be using only the TNETWIF handle and this one will send the request to the HwAccess module */
    switch (pWhalBus->uInitStage)
    {
    case 0:
        pWhalBus->uInitStage ++;
        whal_hwEeprom_Config (pWhalBus->pHwEeprom, pWhalBus->hTNETWIF, pWhalBus->hReport);

        /* disable interrupts */
        status = TNETWIF_WriteRegOpt (pTnetwDrv->hTNETWIF, 
                                      ACX_REG_INTERRUPT_MASK, 
                                      ACX_INTR_ALL,
                                      HAL_INIT_MODULE_ID,
                                      whalBus_ConfigSm,
                                      hWhalBus);
        EXCEPT (pWhalBus, status)

    case 1:
        pWhalBus->uInitStage = 0;

        CmdMBox_Config (pTnetwDrv->hCmdMBox, 
                        pWhalBus->hTNETWIF, 
                        pTnetwDrv->hFwEvent, 
                        pTnetwDrv->hCmdQueue, 
                        pWhalBus->hReport);

        eventMbox_Config (pTnetwDrv->hEventMbox, 
                          pTnetwDrv->hTNETWIF, 
                          pTnetwDrv->hHwIntr, 
                          pTnetwDrv->hReport,
                          pTnetwDrv->hFwEvent,
                          pTnetwDrv->hHalCtrl);

      #ifdef TI_DBG
        /* Initiate the trace object */
        whal_traceConfig (pWhalBus->pTrc, pWhalBus->hTNETWIF, pWhalBus->hReport);
      #endif 

        /* Call upper module callback */
        pWhalBus->fCb (pWhalBus->hCb, status);

        WLAN_REPORT_INIT (pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG,  
                          ("whalBus_Config: EXITING SUCCESS !!!\n"));
    }
}


/****************************************************************************
 *                      whalBus_Config()
 ****************************************************************************
 * DESCRIPTION: Config the object 
 * 
 * INPUTS:  
 *      hWhalBus        The object to free
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
TI_STATUS whalBus_Config
(
    TI_HANDLE hWhalBus, 
    TI_HANDLE hWhalCtrl,
    UINT8     AccessMode, 
    UINT32    RegBaseAddr, 
    UINT32    MemBaseAddr, 
    TI_HANDLE hReport, 
    TI_HANDLE hMemMgr,
    fnotify_t fCb,
    TI_HANDLE hCb
)
{
    whalBus_T  *pWhalBus  = (whalBus_T *)hWhalBus;
    WHAL_CTRL  *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)pWhalCtrl->hTNETW_Driver;

    if (pWhalBus == NULL)
        return OK;

    pWhalBus->hReport = hReport;
    pWhalBus->hTnetwDrv = (TI_HANDLE)pTnetwDrv;
    pWhalBus->hWhalCtrl = hWhalCtrl;
    pWhalBus->fCb = fCb;
    pWhalBus->hCb = hCb;
    pWhalBus->uInitStage = 0;

    /* Call the TNETWIF Configuration */
    return TNETWIF_Config (pWhalBus->hTNETWIF, 
                           hReport, 
                           RegBaseAddr, 
                           MemBaseAddr, 
                           whalBus_ConfigSm, 
                           hWhalBus);   
}


/****************************************************************************
 *                      whalBus_GetTnentwifHandle()
 ****************************************************************************
 * DESCRIPTION: Return TNETWIF handle
 * 
 * INPUTS:  
 *      hWhalBus        The object handle
 * 
 * OUTPUT:  None
 * 
 * RETURNS: TNETWIF handle
 ****************************************************************************/
TI_HANDLE whalBus_GetTnentwifHandle (TI_HANDLE hWhalBus)
{
    whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;

    return pWhalBus->hTNETWIF;
}

/****************************************************************************
 *                      whalBus_ExitFromInitMode()
 ****************************************************************************
 * DESCRIPTION: Change the state of the Bus Access After init
 * 
 * INPUTS:  
 *      hWhalBus        The object handle
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whalBus_ExitFromInitMode(TI_HANDLE hWhalBus)
{
    whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;
    TnetwDrv_t *pTnetwDrv = (TnetwDrv_t *)pWhalBus->hTnetwDrv;
  #if defined(USE_SYNC_API)
    UINT32      uIntVect;
  #endif

    /* Set The Bus Access Mbox to Work in Async Mode */
    CmdMBox_SetModeNormal (pTnetwDrv->hCmdMBox);

  #if defined(USE_SYNC_API)

    uIntVect = FwEvent_GetEnabled (pTnetwDrv->hFwEvent);

    /* Clearing all the interrupt status register sources */
    TNETWIF_WriteRegSync (pWhalBus->hTNETWIF, ACX_REG_INTERRUPT_MASK, ~uIntVect); 

  #endif

    return OK;
}
/*
 * --------------------------------------------------------------
 *                  Registers/Memory access API
 * --------------------------------------------------------------
 */ 

UINT32 whalBus_MacRegRead(TI_HANDLE hWhalBus, UINT32 RegAddr)
{
    UINT32 data = 0;
  #ifdef USE_SYNC_API
    TNETWIF_ReadRegSync(((whalBus_T *)hWhalBus)->hTNETWIF,RegAddr,(UINT32 *)&data);
  #endif
    return data;
}

void whalBus_MacRegWrite(TI_HANDLE hWhalBus, UINT32 RegAddr, UINT32 Val)
{
  #ifdef USE_SYNC_API
    TNETWIF_WriteRegSync(((whalBus_T *)hWhalBus)->hTNETWIF, RegAddr, Val);
  #endif
}

void whalBus_MemCopyTo (TI_HANDLE hWhalBus, char *DestOffset, char *Src, int Len)
{
  #ifdef USE_SYNC_API
    TNETWIF_WriteMemSync(((whalBus_T *)hWhalBus)->hTNETWIF,(UINT32)DestOffset,(UINT8*)Src,Len);
  #endif
}
 
void whalBus_MemCopyFrom (TI_HANDLE hWhalBus, UINT8 *Dest, char *SrcOffset, int Len)
{
  #ifdef USE_SYNC_API
    TNETWIF_ReadMemSync(((whalBus_T *)hWhalBus)->hTNETWIF,(UINT32)SrcOffset,Dest,Len);
  #endif
}

#define WRITE_PHY_NUM_RETRIES     4

void    whalBus_PhyRegWrite      (TI_HANDLE hWhalBus, UINT32 PhyRegAddr, UINT32 DataVal)
{
  #ifdef USE_SYNC_API
    whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;
    int NumRetries=1;
    UINT32 data;
    TNETWIF_WriteRegSync(pWhalBus->hTNETWIF, ACX_PHY_ADDR_REG, PhyRegAddr);
    TNETWIF_WriteRegSync(pWhalBus->hTNETWIF, ACX_PHY_DATA_REG, DataVal);
    TNETWIF_WriteRegSync(pWhalBus->hTNETWIF, ACX_PHY_CTRL_REG, ACX_PHY_REG_WR_MASK);

    os_StalluSec(pWhalBus->hOs, 10000);

    /* wait for write complete */
        TNETWIF_ReadRegSync(pWhalBus->hTNETWIF,ACX_PHY_CTRL_REG,&data);
    while (data  && (NumRetries < WRITE_PHY_NUM_RETRIES))
    {
        NumRetries++;
        WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG,  
                          ("ACX_PHY_CTRL_REG Write, Addr - %#x  Data - %#x, retry\n", PhyRegAddr, DataVal));
        os_StalluSec(pWhalBus->hOs, 10000);
                TNETWIF_ReadRegSync(pWhalBus->hTNETWIF,ACX_PHY_CTRL_REG,&data);
    }
  #endif
} 

UINT32  whalBus_PhyRegRead (TI_HANDLE hWhalBus, UINT32 PhyRegAddr)
{
    UINT32  DataVal = 0;
  #ifdef USE_SYNC_API
    whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;
    int NumRetries=1;
    UINT32 data;
    TNETWIF_WriteRegSync(pWhalBus->hTNETWIF, ACX_PHY_ADDR_REG, PhyRegAddr);
    TNETWIF_WriteRegSync(pWhalBus->hTNETWIF, ACX_PHY_CTRL_REG, ACX_PHY_REG_RD_MASK);
    os_StalluSec(pWhalBus->hOs, 10000);

    /* wait for write complete */
    TNETWIF_ReadRegSync(pWhalBus->hTNETWIF,ACX_PHY_CTRL_REG,&data);
    while ( data  && (NumRetries < WRITE_PHY_NUM_RETRIES))
    {
        NumRetries++;
        WLAN_REPORT_REPLY(pWhalBus->hReport, HAL_HW_CTRL_MODULE_LOG,  
                          ("ACX_PHY_CTRL_REG Read, Addr - %#x  retry\n", PhyRegAddr));
        os_StalluSec(pWhalBus->hOs, 10000);
        TNETWIF_ReadRegSync(pWhalBus->hTNETWIF,ACX_PHY_CTRL_REG,&data);
    }

    TNETWIF_ReadRegSync(pWhalBus->hTNETWIF,ACX_PHY_DATA_REG,&DataVal);
  #endif
    return DataVal;
}


/*
 * --------------------------------------------------------------
 *                  Interrupt handler API
 * --------------------------------------------------------------
 */                  
void whalBus_TNETWIF_HandleBusTxn_Complete  (TI_HANDLE hWhalBus)
{
    TNETWIF_BusTxn_Complete (((whalBus_T *)hWhalBus)->hTNETWIF);
}

void    whalBus_performHealthMonitorTest(TI_HANDLE hWhalBus, UINT32 test)
{
#ifdef TI_DBG
    switch (test) {

    case 1:
        WLAN_OS_REPORT(("HAL Perform Health Monitor MBOX Test\n"));
        break;
#if 0   
    case 2:
        WLAN_OS_REPORT(("HAL Perform Health Monitor TX STUCK Test\n"));
        whal_hwTx_performHealthMonitorTest(((whalBus_T *)hWhalBus)->pHwTx);
        break;
#endif
    }
#endif
}

/* Dummy function */
/*
 * --------------------------------------------------------------
 *                  Debug API
 * --------------------------------------------------------------
 */              
#ifdef TI_DBG
void whalBus_PrintInfo(TI_HANDLE hWhalBus, UINT32 funcType, void *pParam)
{
    whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;

    switch (funcType)
    {
    case BUS_PRINT_ARBITER:
        TNETWArb_PrintStat (((TNETWIF_t*)pWhalBus->hTNETWIF)->hTNETWArb);
        break;

    default:
        WLAN_OS_REPORT(("%s: Invalid function type: %d\n\n", __FUNCTION__, funcType));
        break;
    }
}
#endif



/****************************************************************************
 *                      whalBus_ReConfig()
 ****************************************************************************
 * DESCRIPTION: ReConfig the object (In case of recovery) 
 * 
 * INPUTS:  
 *      hWhalBus        The object to free
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whalBus_ReConfig(TI_HANDLE hWhalBus )
{
    whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;
    /*Add a function in HwAccess that Reconfig (SDIO_Stop/SDIO_Start) and also in SPI */
    TNETWIF_ReConfig(pWhalBus->hTNETWIF);
    return OK;
}

/****************************************************************************
 *                      whalBus_TNETWIF_ElpCtrl_SetMode()
 ****************************************************************************
 * DESCRIPTION: wrapper function for the lower TNETWIF_ElpCtrl_Mode
 * 
 * INPUTS:  
 *      hWhalBus        The current context handle
 *      mode            The ElpCtrl mode
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whalBus_TNETWIF_ElpCtrl_SetMode(TI_HANDLE hWhalBus, elpCtrl_Mode_e mode)
{
    whalBus_T *pWhalBus = (whalBus_T *)hWhalBus;
    
    return TNETWIF_ElpCtrl_Mode(pWhalBus->hTNETWIF,mode);
}


