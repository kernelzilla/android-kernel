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
 *   MODULE:  GWSI_Synchronizer.c
 *   PURPOSE: GWSI Synchronizer used to synchronize between the CMD,INT,TX 
 *
 ****************************************************************************/

#include "commonTypes.h"
#include "report.h"
#include "osApi.h"
/* Need tthe TNETWARB API Function to create and confit it */
#include "TNETWArb.h"
#include "TNETWIF.h"
/* Need the HwAcess API Function to create and confit it */
#include "whalHwAccess.h"
/* Need the ELP Control API Function to create and confit it */
#include "ElpCtrl.h"

/*
 * Special debug mode to trace all calls to TNETWIF_XXX
 */
#ifdef TNETWIF_DEBUG

typedef enum
{
	No_FUNC,
	Start_FUNC,
	Restart_FUNC,
	Finish_FUNC,
	WriteMem_FUNC,
	ReadMem_FUNC,
	WriteReg_FUNC,
	ReadReg_FUNC,
	WriteElp_FUNC,
	ReadElp_FUNC,
	BusComplete_FUNC,
	UnMux_FUNC
} ETNETIF_Functions;

typedef struct
{
	UINT32				uData;
	UINT32				uModule;
	UINT32				uTS;
	ETNETIF_Functions	eFunc;

} TTNETWIF_DEBUG;

/* Should be the max value of uDebugCounter */
#define TNETWIF_SIZE_OF_DEBUG_ARRAY 256

/* Global variables for debug */
UINT8				uDebugCounter ;
TTNETWIF_DEBUG		tDebug[TNETWIF_SIZE_OF_DEBUG_ARRAY];

#define TNETWIF_TRACE(hOs,func,module,data) \
		tDebug[uDebugCounter].uData   = (UINT32)(data); \
		tDebug[uDebugCounter].uModule = module; \
		tDebug[uDebugCounter].uTS     = os_timeStampUs(hOs); \
		tDebug[uDebugCounter].eFunc   = func; \
		uDebugCounter++;

/****************************************************************************
*                      TNETWIF_FuncEnumdToString()
*****************************************************************************/
char* TNETWIF_FuncEnumToString(ETNETIF_Functions eFunc)
{
	switch (eFunc)
	{
	case No_FUNC:			return "(No_FUNC)           ";
	case Start_FUNC:		return "(Start_FUNC)        ";
	case Restart_FUNC:		return "(Restart_FUNC)      ";
	case Finish_FUNC:		return "(Finish_FUNC)       ";
	case WriteMem_FUNC:		return "(WriteMem_FUNC)     ";
	case ReadMem_FUNC:		return "(ReadMem_FUNC)      ";
	case WriteReg_FUNC:		return "(WriteReg_FUNC)     ";
	case ReadReg_FUNC:		return "(ReadReg_FUNC)      ";
	case WriteElp_FUNC:		return "(WriteElp_FUNC)     ";
	case ReadElp_FUNC:		return "(ReadElp_FUNC)      ";
	case BusComplete_FUNC:  return "(BusComplete_FUNC)  ";
	case UnMux_FUNC:		return "(UnMux_FUNC)        ";
	default :
		return " No_FUNC???)";
	}
}
/****************************************************************************
*                      TNETWIF_printErrorLog()
*****************************************************************************/
void TNETWIF_printErrorLog(void)
{
	int i;
	WLAN_OS_REPORT(("\n%s\n",__FUNCTION__));
	WLAN_OS_REPORT(("Counter at %d (i.e. last operation is %d)\n",uDebugCounter,uDebugCounter-1));
	for ( i = 0  ; i < TNETWIF_SIZE_OF_DEBUG_ARRAY ; i++)
	{
		WLAN_OS_REPORT(("%03d %s: %s TS(Diff) = %07d data = 0x%x\n",i,
			TNETWIF_FuncEnumToString(tDebug[i].eFunc), 
			TNETWIF_ModuleIdToString(tDebug[i].uModule),
			tDebug[i].uTS - tDebug[(UINT8)(i-1)].uTS,
			tDebug[i].uData));
	}
}

#else /* TNETWIF_DEBUG */

#define TNETWIF_TRACE(hOs,func,module_id,data)
void TNETWIF_printErrorLog(void) { WLAN_OS_REPORT(("%s define TNETWIF_DEBUG to debug error\n",__FUNCTION__)); }

#endif

extern void os_TNETWIF_BusTxn_Complete(TI_HANDLE OsContext,int status);


/***********************************************************************************
 Internal TNETWIF function use



**************************************************************************************/
/****************************************************************************/
/*                      TNETWIF_Create()
 ****************************************************************************
 * DESCRIPTION: Request The Bus
 *
 * INPUTS:  
 *
 * OUTPUT:  String the name of the Queue
 *
 * RETURNS: 
 ****************************************************************************/
TI_HANDLE TNETWIF_Create (TI_HANDLE hOs)
{
    TNETWIF_t *pTNETWIF;

    /* Allocate the TNETWIF module */
    pTNETWIF = os_memoryAlloc (hOs, sizeof(TNETWIF_t));

    if (pTNETWIF == NULL)
        return NULL;

    os_memoryZero (hOs, pTNETWIF, sizeof(TNETWIF_t));

    pTNETWIF->hOs = hOs;

    /* Create the TNETW Arbiter module */
    pTNETWIF->hTNETWArb = TNETWArb_Init (hOs);

    /* Create the Hw Access module */
    pTNETWIF->hHwAccess = whal_hwAccess_Create (hOs);

    /* Create the ELP Controller module */
    pTNETWIF->hELPCtrl = elpCtrl_Create (hOs);

    return (TI_HANDLE)pTNETWIF;
}


/****************************************************************************/
/*                      TNETWIF_Destroy()
 ****************************************************************************
 * DESCRIPTION: Destroy The TNETWIF Module
 *
 * INPUTS:  
 *
 * OUTPUT:  String the name of the Queue
 *
 * RETURNS: 
 ****************************************************************************/
TI_STATUS TNETWIF_Destroy (TI_HANDLE hTNETWIF)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

    /* Destroy the ELP Controller module */ 
    elpCtrl_Destroy (pTNETWIF->hELPCtrl);

    /* Destroy the HwAccess Module */
    whal_hwAccess_Destroy (pTNETWIF->hHwAccess);

    /* Destroy the TNETW Arbiter */
    TNETWArb_Destroy (pTNETWIF->hTNETWArb);

    /* Free the TNETWIF Memory */
    os_memoryFree (pTNETWIF->hOs, pTNETWIF, sizeof(TNETWIF_t));
        

    return OK;
}


/****************************************************************************/
/*                      TNETWIF_ConfigCb()
 ****************************************************************************
 * DESCRIPTION: TNETWIF module configuration state machine
 *
 * INPUTS:  
 *
 * OUTPUT:  
 *
 * RETURNS: 
 ****************************************************************************/
static void TNETWIF_ConfigCb (TI_HANDLE hTNETWIF, UINT8 module_id, TI_STATUS status)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

    switch (pTNETWIF->uInitStage)
    {
    case 0:
        pTNETWIF->uInitStage ++;

	#ifdef GWSI_SPI_TEST
		/*
		* This is a special build meant only for testing the SPI bus. We don't need anything 
		* but initiating the TNETWIF. That's why we skip partition and ELP phase
		*/
		pTNETWIF->uInitStage = 3;
	#endif /* GWSI_SPI_TEST */

        /* Register the arbiter callback */
        TNETWArb_register_handler (pTNETWIF->hTNETWArb, 
                                   HAL_INIT_MODULE_ID, 
                                   TNETWIF_ConfigCb, 
                                   hTNETWIF);

        /* Configure the HwAccess with the DMA Done callback */
        status = (TI_STATUS)whal_hwAccess_Config (pTNETWIF->hHwAccess, 
                                       pTNETWIF->hReport, 
                                       pTNETWIF->uRegBaseAddr, 
                                       pTNETWIF->uMemBaseAddr, 
                                       os_TNETWIF_BusTxn_Complete, 
                                       pTNETWIF->hOs);
     
        if (status == TNETWIF_PENDING)
        {
            pTNETWIF->status = TNETWIF_PENDING;
            return;
        }
		else if (status == TNETWIF_ERROR)
		{	/* This case is mainly used for the SPI test */
			pTNETWIF->status = TNETWIF_ERROR;
		}
        else 
        {
            pTNETWIF->status = TNETWIF_COMPLETE;
        }

    case 1:
        pTNETWIF->uInitStage ++;

      #if defined(HW_ACCESS_SDIO) || defined(HW_ACCESS_WSPI)
        status = TNETWIF_SetPartitionsOpt (hTNETWIF, 
                                           HW_ACCESS_DOWNLOAD, 
                                           HW_ACCESS_DOWN_PART0_ADDR,
                                           module_id,
                                           (TNETWIF_callback_t)TNETWIF_ConfigCb, 
                                           hTNETWIF);
        if (status == TNETWIF_PENDING)
        {
            pTNETWIF->status = TNETWIF_PENDING;
            return;
        }
        else 
        {
            pTNETWIF->status = TNETWIF_COMPLETE;
        }
      #endif

    case 2:
        pTNETWIF->uInitStage ++;

        /* Awake firmware */
        if (TNETWIF_WriteELPOpt (hTNETWIF, 
                                 ELPCTRL_WAKE_UP,
                                 HAL_INIT_MODULE_ID,
                                 TNETWIF_ConfigCb,
                                 hTNETWIF,
                                 TRUE) == TNETWIF_PENDING)
        {
            pTNETWIF->status = TNETWIF_PENDING;
            return;
        }
        else
        {
            pTNETWIF->status = TNETWIF_COMPLETE;
        }

    case 3:
        /* Call upper module callback */
        pTNETWIF->fCb (pTNETWIF->hCb, module_id, pTNETWIF->status);
        pTNETWIF->uInitStage = 0;

        WLAN_REPORT_INIT (pTNETWIF->hReport, TNETW_IF_MODULE_LOG,
                          ("%s(%d) - TNETWIF Initialized\n", __FILE__, __LINE__));
    }
}


/****************************************************************************/
/*                      TNETWIF_Config()
 ****************************************************************************
 * DESCRIPTION: Configure the TNETWIF module
 *
 * INPUTS:  
 *
 * OUTPUT:  
 *
 * RETURNS: 
 ****************************************************************************/
TI_STATUS TNETWIF_Config (TI_HANDLE hTNETWIF, TI_HANDLE hReport, UINT32 uRegBaseAddr, UINT32 uMemBaseAddr, TNETWIF_callback_t fCb, TI_HANDLE hCb)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

    pTNETWIF->hReport = hReport;
    pTNETWIF->uInitStage = 0;
    pTNETWIF->uRegBaseAddr = uRegBaseAddr;
    pTNETWIF->uMemBaseAddr = uMemBaseAddr;
    pTNETWIF->fCb = fCb;
    pTNETWIF->hCb = hCb;
    pTNETWIF->status = TNETWIF_PENDING;
#ifdef TNETWIF_DEBUG
	uDebugCounter = 0;
	os_memoryZero(pTNETWIF->hOs, (void*)tDebug, sizeof(tDebug));
	WLAN_OS_REPORT(("Using Debug Arbiter\n"));
#endif

    /* Send the ELP Controller handle to the TNETW Arbiter */
    TNETWArb_Config (pTNETWIF->hTNETWArb, hReport, pTNETWIF->hELPCtrl);

    /* Configure ELP control before the 1st TNETWIF_Start call */
    elpCtrl_Configure (pTNETWIF->hELPCtrl, hTNETWIF, TNETWArb_TxnCb);

    /* Start TNETWIF config state machine */
    TNETWIF_Start (hTNETWIF, HAL_INIT_MODULE_ID, hTNETWIF, TNETWIF_ConfigCb);

    return pTNETWIF->status;
}


/****************************************************************************/
/*                      TNETWIF_ReConfig()
 ****************************************************************************
 * DESCRIPTION: stop the SDIO in Recovery process
 *
 * INPUTS:  
 *
 * OUTPUT:  status
 *
 * RETURNS: 
 ****************************************************************************/
TI_STATUS TNETWIF_ReConfig (TI_HANDLE hTNETWIF)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;
    TI_STATUS status = OK;
    #if (defined(_WINDOWS) && defined(HW_ACCESS_SDIO))
    #else 
    whal_hwAccess_Stop (pTNETWIF->hHwAccess);
    #endif 

    return (status);
}


/****************************************************************************/
/*                      TNETWIF_Start()
 ****************************************************************************
 * DESCRIPTION: Request The Bus
 *
 * INPUTS:  
 *
 * OUTPUT:  String the name of the Queue
 *
 * RETURNS: 
 ****************************************************************************/
TI_STATUS TNETWIF_Start (TI_HANDLE hTNETWIF, UINT8 module_id, TI_HANDLE hCb, TNETWIF_callback_t fCb)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;
    TI_STATUS  status;

	TNETWIF_TRACE(pTNETWIF->hOs,Start_FUNC,module_id,fCb)

    /* Call the TNETW Arbiter for the start operation - the client requests access the bus */
    status = TNETWArb_Start (pTNETWIF->hTNETWArb, module_id, hCb, fCb);

    return status;
}


/****************************************************************************/
/*                      TNETWIF_Restart()
 ****************************************************************************
 * DESCRIPTION: Re-Request The Bus
 *
 * INPUTS:  
 *
 * OUTPUT: 
 *
 * RETURNS: 
 ****************************************************************************/
TI_STATUS TNETWIF_Restart (TI_HANDLE hTNETWIF, UINT8 module_id, TI_HANDLE hCb, TNETWIF_callback_t fCb)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;
    TI_STATUS  status;

	TNETWIF_TRACE(pTNETWIF->hOs,Restart_FUNC,module_id,fCb)

    /* Call the TNETW Arbiter for the restart operation - the client requests access to the bus */
    status = TNETWArb_Restart (pTNETWIF->hTNETWArb, module_id, hCb, fCb);       

    return status;
}


/****************************************************************************/
/*                      TNETWIF_Finish()
 ****************************************************************************
 * DESCRIPTION: Release The Bus
 *
 * INPUTS:  
 *
 * OUTPUT:  
 *
 * RETURNS: 
 ****************************************************************************/
TI_STATUS TNETWIF_Finish (TI_HANDLE hTNETWIF, UINT8 module_id, TI_HANDLE hCb, TNETWIF_callback_t fCb)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;
    TI_STATUS  status;

	TNETWIF_TRACE(pTNETWIF->hOs,Finish_FUNC,module_id,fCb)

    /* Call the TNETW Arbiter for the finish operation - the client frees the bus */
    status = TNETWArb_Finish (pTNETWIF->hTNETWArb, module_id, hCb, fCb);        

    return status;
}

/******************************************************************************
**                                                                           **
**  Function Name: TNETWIF_UnMux                                   **
**                                                                           **
**  Description: This should be called FwEvent to switch MUX from WLAN_READY to FwEvent.        **
**                                                                           **
******************************************************************************/
TI_STATUS TNETWIF_UnMux (TI_HANDLE hTNETWIF)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

	TNETWIF_TRACE(pTNETWIF->hOs,UnMux_FUNC,0xFF,0)

    return (TI_STATUS)elpCtrl_UnMux(pTNETWIF->hELPCtrl);
    
}


/******************************************************************************
**                                                                           **
**  Function Name: TNETWIF_BusTxn_Complete                                   **
**                                                                           **
**  Description: This should be called now from the tasklet  
                Distribute SPI interrupt to all running modules     .        **
**                                                                           **
******************************************************************************/
void TNETWIF_BusTxn_Complete (TI_HANDLE hTNETWIF)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

	TNETWIF_TRACE(pTNETWIF->hOs,BusComplete_FUNC,0xFF,0)

    TNETWArb_CallTxnCb (pTNETWIF->hTNETWArb);
}


/******************************************************************************
**                                                                           **
**  Function Name: TNETWIF_ElpCtrl_Mode                                      **
**                                                                           **
**  Description:    this function changes the mode of the ElpCtrl            **
**                                                                           **
******************************************************************************/
int TNETWIF_ElpCtrl_Mode (TI_HANDLE hTNETWIF, elpCtrl_Mode_e mode)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

    elpCtrl_Mode (pTNETWIF->hELPCtrl, mode);

    return OK;

}


/******************************************************************************
**                                                                           **
**  Function Name: TNETWIF_ElpCtrl_HostIF_required                                       **
**                                                                           **
**  Description:    
**                                                                           **
******************************************************************************/
int TNETWIF_ElpCtrl_HostIF_required (TI_HANDLE hTNETWIF, int flag)
{
    /* TODO: what is the purpose of this API? */
    return OK;
}


/******************************************************************************
                  IO Operations : There are 3 modes of operation:
                                1) Synchronous mode : The caller context is blocked till the Transcation has finished
                                2) Asynchronous mode : The caller context is unblocked and the Transcation will end later by DMA
******************************************************************************/
#ifdef USE_SYNC_API

/******* Synchronous IO mode **************************************************/
/****************************************************************************
 *                      TNETWIF_WriteMemSync()
 ****************************************************************************
 * DESCRIPTION: Request an Synchronous IO with the Bus
 *
 * INPUTS:  
 *
 * OUTPUT:  
 *
 * RETURNS: 
 ****************************************************************************/
TI_STATUS   TNETWIF_WriteMemSync (TI_HANDLE hTNETWIF, UINT32 addr, UINT8* data, UINT32 len)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;
    int  status;

    /* Then Call the HwAccess to operate the I/O */
    status = whal_hwAccess_WriteMem (pTNETWIF->hHwAccess, addr, data, len);

    /* The return status could be TNETWIF_COMPLETE in case of Success 
                                  TNETWIF_ERROR in case of ERROR */
    if (status == OK)
        return TNETWIF_COMPLETE;
    else
        return TNETWIF_ERROR;
}


/****************************************************************************
 *                      TNETWIF_ReadMemSync()
 ****************************************************************************
 * DESCRIPTION: Request an Synchronous IO with the Bus
 *
 * INPUTS:  
 *
 * OUTPUT:  
 *
 * RETURNS: 
 ****************************************************************************/
TI_STATUS   TNETWIF_ReadMemSync (TI_HANDLE hTNETWIF, UINT32 addr, UINT8* data, UINT32 len)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;
    TI_STATUS  status;

    /* Then Call the HwAccess to operate the I/O */
    status = (TI_STATUS)whal_hwAccess_ReadMem (pTNETWIF->hHwAccess, addr, data, len);

    /* The return status could be TNETWIF_COMPLETE in case of Success 
                                  TNETWIF_ERROR in case of ERROR */
    return status;
}


/****************************************************************************
 *                      TNETWIF_ReadRegSync()
 ****************************************************************************
 * DESCRIPTION: Request an Synchronous IO with the Bus
 *
 * INPUTS:  
 *
 * OUTPUT: 
 *
 * RETURNS: 
 ****************************************************************************/
TI_STATUS   TNETWIF_ReadRegSync (TI_HANDLE hTNETWIF, UINT32 addr, UINT32* data)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;
    TI_STATUS  status;

    /* Then Call the HwAccess to operate the I/O */
    status = (TI_STATUS)whal_hwAccess_ReadReg (pTNETWIF->hHwAccess, addr, data);

    /* The return status could be TNETWIF_COMPLETE in case of Success 
                                  TNETWIF_ERROR in case of ERROR */
    return status;
}


/****************************************************************************
 *                      TNETWIF_WriteRegSync()
 ****************************************************************************
 * DESCRIPTION: Request an Synchronous IO with the Bus
 *
 * INPUTS:  
 *
 * OUTPUT:  TI_STATUS    TNETWIF_COMPLETE or TNETWIF_ERROR
 *
 * RETURNS: 
 ****************************************************************************/
TI_STATUS   TNETWIF_WriteRegSync (TI_HANDLE hTNETWIF, UINT32 addr, UINT32 data)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;
    TI_STATUS  status;

    /* Then Call the HwAccess to operate the I/O */
    status = (TI_STATUS)whal_hwAccess_WriteReg (pTNETWIF->hHwAccess, addr, data);

    /* The return status could be TNETWIF_COMPLETE in case of Success 
                                  TNETWIF_ERROR in case of ERROR */
    return status;
}
#endif /* USE_SYNC_API */


/******* Optimized IO mode : In this mode the SDIO/SPI Driver will decide with its inner thresholds if to make a DMA or not  **************************************************/

/****************************************************************************
 *                      TNETWIF_ReadMemOpt()
 ****************************************************************************
 * DESCRIPTION: Request an Unspecified Read Memory IO with the Bus
 *              Note: Currently, only Sync read is implemented!
 *
 * INPUTS:  
 *
 * OUTPUT:  TI_STATUS    TNETWIF_COMPLETE or TNETWIF_ERROR
 *
 * RETURNS: 
 ****************************************************************************/
TI_STATUS   TNETWIF_ReadMemOpt 
(
    TI_HANDLE          hTNETWIF, 
    UINT32             addr, 
    UINT8             *data, 
    UINT32             len, 
    UINT8              module_id, 
    TNETWIF_callback_t fCb, 
    TI_HANDLE          hCb)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

	TNETWIF_TRACE(pTNETWIF->hOs,ReadMem_FUNC,module_id,data)

    /* First Register the Callback to the TNET Arbiter where the DMA DONE Callback will be received */
    TNETWArb_register_handler (pTNETWIF->hTNETWArb, module_id, fCb, hCb);

    /* Call the HwAccess to operate the I/O */
    return (TI_STATUS)whal_hwAccess_ReadMemAsync (pTNETWIF->hHwAccess, addr, data, len);
}


/****************************************************************************
 *                      TNETWIF_WriteMemOpt()
 ****************************************************************************
 * DESCRIPTION: Request an Unspecified Write Memory IO with the Bus
 *
 * INPUTS:  
 *
 * OUTPUT:  TI_STATUS    TNETWIF_COMPLETE or TNETWIF_ERROR
 *
 * RETURNS: 
 ****************************************************************************/
TI_STATUS   TNETWIF_WriteMemOpt 
(
    TI_HANDLE          hTNETWIF, 
    UINT32             addr, 
    UINT8             *data, 
    UINT32             len, 
    UINT8              module_id, 
    TNETWIF_callback_t fCb, 
    TI_HANDLE          hCb)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

	TNETWIF_TRACE(pTNETWIF->hOs,WriteMem_FUNC,module_id,*(UINT32*)(data + TNETWIF_WRITE_OFFSET_BYTES))

    /* First Register the Callback to the TNET Arbiter where the DMA DONE Callback will be received */
    TNETWArb_register_handler (pTNETWIF->hTNETWArb, module_id, fCb, hCb);
        
    /* Then Call the HwAccess to operate the I/O */
    return whal_hwAccess_WriteMemAsync (pTNETWIF->hHwAccess, addr, data, len);
}


/****************************************************************************
 *                      TNETWIF_ReadRegOpt()
 ****************************************************************************
 * DESCRIPTION: Request an Unspecified Read Register IO with the Bus
 *
 * INPUTS:  
 *
 * OUTPUT:  TI_STATUS    TNETWIF_COMPLETE or TNETWIF_ERROR or TNETWIF_PENDING
 *
 * RETURNS:   
 ****************************************************************************/
TI_STATUS  TNETWIF_ReadRegOpt (TI_HANDLE hTNETWIF, UINT32 addr, UINT32* data, UINT8 module_id, TNETWIF_callback_t fCb, TI_HANDLE hCb)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

	TNETWIF_TRACE(pTNETWIF->hOs,ReadReg_FUNC,module_id,data)

    /* First Register the Callback to the TNET Arbiter where the DMA DONE Callback will be received */
    TNETWArb_register_handler (pTNETWIF->hTNETWArb, module_id, fCb, hCb);
    
   /* Call the HwAccess to operate the I/O */
    return (TI_STATUS)whal_hwAccess_ReadRegAsync (pTNETWIF->hHwAccess, addr, data);
}


/****************************************************************************
 *                      TNETWIF_WriteRegOpt()
 ****************************************************************************
 * DESCRIPTION: Request an Unspecified Read Register IO with the Bus
 *
 * INPUTS:  
 *
 * OUTPUT:  TI_STATUS    TNETWIF_COMPLETE or TNETWIF_ERROR or TNETWIF_PENDING
 *
 * RETURNS: 
 ****************************************************************************/
TI_STATUS  TNETWIF_WriteRegOpt (TI_HANDLE hTNETWIF, UINT32 addr, UINT32 data, UINT8 module_id, TNETWIF_callback_t fCb, TI_HANDLE hCb)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

	TNETWIF_TRACE(pTNETWIF->hOs,WriteReg_FUNC,module_id,data)

    /* First Register the Callback to the TNET Arbiter where the DMA DONE Callback will be received */
    TNETWArb_register_handler (pTNETWIF->hTNETWArb, module_id, fCb, hCb);

    /* Then Call the HwAccess to operate the I/O */
    return (TI_STATUS)whal_hwAccess_WriteRegAsync (pTNETWIF->hHwAccess, addr, data);
}

#ifdef USE_SYNC_API
/****************************************************************************
 *                      TNETWIF_WriteELPSync()
 ****************************************************************************
 * DESCRIPTION: Request an Unspecified Read Register IO with the Bus
 *
 * INPUTS:  
 *
 * OUTPUT:  TI_STATUS    TNETWIF_COMPLETE or TNETWIF_ERROR or TNETWIF_PENDING
 *
 * RETURNS: 
 ****************************************************************************/
TI_STATUS   TNETWIF_WriteELPSync (TI_HANDLE hTNETWIF, UINT32 data)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

    /* Call the HwAccess to operate the I/O */
    return (TI_STATUS)whal_hwAccess_WriteELP (pTNETWIF->hHwAccess, data);
}
#endif /* USE_SYNC_API */


/****************************************************************************
 *                      TNETWIF_WriteELPOpt()
 ****************************************************************************
 * DESCRIPTION: Request an Unspecified Read Register IO with the Bus
 *
 * INPUTS:  bMore - indicate whether more txn on the bus are about to happen
 *
 * OUTPUT:  TI_STATUS    TNETWIF_COMPLETE or TNETWIF_ERROR or TNETWIF_PENDING
 *
 * RETURNS: 
 ****************************************************************************/
TI_STATUS   TNETWIF_WriteELPOpt (TI_HANDLE hTNETWIF, UINT32 data, UINT8 module_id, TNETWIF_callback_t fCb, TI_HANDLE hCb, BOOL bMore)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

	TNETWIF_TRACE(pTNETWIF->hOs,WriteElp_FUNC,module_id,data)

    /* Register a callback */
    if(fCb)
      TNETWArb_register_handler (pTNETWIF->hTNETWArb, module_id, fCb, hCb);

    /* Call the HwAccess to operate the I/O */
    return  (TI_STATUS)whal_hwAccess_WriteELPAsync (pTNETWIF->hHwAccess, data, fCb != NULL, bMore);
}

/****************************************************************************
 *                      TNETWIF_ReadELPOpt()
 ****************************************************************************
 * DESCRIPTION: Request an Unspecified Read Register IO with the Bus
 *
 * INPUTS:  bMore - indicate whether more txn on the bus are about to happen
 *
 * OUTPUT:  TI_STATUS    TNETWIF_COMPLETE or TNETWIF_ERROR or TNETWIF_PENDING
 *
 * RETURNS: 
 ****************************************************************************/
TI_STATUS   TNETWIF_ReadELPOpt (TI_HANDLE hTNETWIF, UINT8 *data, UINT8 module_id, TNETWIF_callback_t fCb, TI_HANDLE hCb, BOOL bMore)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

	TNETWIF_TRACE(pTNETWIF->hOs,ReadElp_FUNC,module_id,data)

    /* Register a callback */
    if(fCb)
      TNETWArb_register_handler (pTNETWIF->hTNETWArb, module_id, fCb, hCb);

    /* Call the HwAccess to operate the I/O */
    return  (TI_STATUS)whal_hwAccess_ReadELPAsync (pTNETWIF->hHwAccess, data, fCb != NULL, bMore);
}


/*****************************************************************************************************
*             Registers API : All the following API are synchronous only and the return value can be:
                                ** TNEDTWIF_ERROR - In case the action did not succeed
                                ** TNETWIF_COMPLETE - In case the action succeeded

*******************************************************************************************************/
#ifdef USE_SYNC_API
/****************************************************************************
 *                      TNETWIF_GetU08()
 ****************************************************************************
 * DESCRIPTION: Request an U8 Value from the Hw Access
 *
 * INPUTS:  
 *
 * OUTPUT:  String the name of the Queue
 *
 * RETURNS: 
 ****************************************************************************/
UINT8  TNETWIF_GetU08 (TI_HANDLE hTNETWIF, UINT32 Addr)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;
    UINT8      Value;

    Value = whal_hwAccess_GetU08 (pTNETWIF->hHwAccess, Addr);

    return Value;

}


/****************************************************************************
 *                      TNETWIF_SetU08()
 ****************************************************************************
 * DESCRIPTION: Set an U8 Value from the Hw Access
 *
 * INPUTS:  
 *
 * OUTPUT:  String the name of the Queue
 *
 * RETURNS: 
 ****************************************************************************/
void  TNETWIF_SetU08 (TI_HANDLE hTNETWIF, UINT32 Addr, UINT8 Val)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

    whal_hwAccess_SetU08 (pTNETWIF->hHwAccess, Addr, Val);
}


void  TNETWIF_ResetU08_Bits (TI_HANDLE hTNETWIF, UINT32 Addr, UINT8  BitsVal)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

    whal_hwAccess_ResetU08_Bits (pTNETWIF->hHwAccess, Addr, BitsVal);

}


UINT16  TNETWIF_GetU16 (TI_HANDLE hTNETWIF, UINT32 Addr)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;
    UINT16     U16Value;

    U16Value = whal_hwAccess_GetU16 (pTNETWIF->hHwAccess,Addr);

    return U16Value;
}


void  TNETWIF_SetU16 (TI_HANDLE hTNETWIF, UINT32 Addr, UINT16 Val)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

    whal_hwAccess_SetU16 (pTNETWIF->hHwAccess, Addr, Val);
}


void  TNETWIF_SetU16_Bits (TI_HANDLE hTNETWIF, UINT32 Addr, UINT16 BitsVal)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

    whal_hwAccess_SetU16_Bits (pTNETWIF->hHwAccess,Addr,BitsVal);
}


void  TNETWIF_ResetU16_Bits (TI_HANDLE hTNETWIF, UINT32 Addr, UINT16 BitsVal)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

    whal_hwAccess_ResetU16_Bits (pTNETWIF->hHwAccess, Addr, BitsVal);
}


UINT32  TNETWIF_GetU32 (TI_HANDLE hTNETWIF, UINT32 Addr)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;
    UINT32     U32Value;

    U32Value = whal_hwAccess_GetU32 (pTNETWIF->hHwAccess, Addr);

    return U32Value;
}


void TNETWIF_SetU32 (TI_HANDLE hTNETWIF, UINT32 Addr, UINT32 Val)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

    whal_hwAccess_SetU32 (pTNETWIF->hHwAccess ,Addr, Val);

}


void  TNETWIF_SetU32_Bits (TI_HANDLE hTNETWIF, UINT32 Addr, UINT32 BitsVal)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

    whal_hwAccess_SetU32_Bits (pTNETWIF->hHwAccess, Addr, BitsVal);

}


void  TNETWIF_ResetU32_Bits (TI_HANDLE hTNETWIF, UINT32 Addr, UINT32 BitsVal)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;
    
    whal_hwAccess_ResetU32_Bits (pTNETWIF->hHwAccess, Addr, BitsVal);

}


/*
 * Hardware Registers Api
 */
void  TNETWIF_RegSetBitVal (TI_HANDLE hTNETWIF, UINT32 RegAddr, UINT32 BitVal)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;
    
    whal_hwAccess_RegSetBitVal (pTNETWIF->hHwAccess, RegAddr, BitVal);

}


void  TNETWIF_RegResetBitVal (TI_HANDLE hTNETWIF, UINT32 RegAddr, UINT32 BitVal)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;
    
    whal_hwAccess_RegResetBitVal (pTNETWIF->hHwAccess, RegAddr, BitVal);

}


int  TNETWIF_RegIsBitSet (TI_HANDLE hTNETWIF, UINT32 RegAddr, UINT32 BitVal)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;
    int        status;
    
    status = whal_hwAccess_RegIsBitSet (pTNETWIF->hHwAccess, RegAddr, BitVal);
    return status;
}

#endif /* USE_SYNC_API */


/***********************************************************************************
**                           Client implementation 
***********************************************************************************/
/******************************************************************************
**                                                                           **
**  Function Name: gwsi_rx_start_instance                                    **
**                                                                           **
**  Description: GWSI Rx Data Path Start Instance.                           **
**                                                                           **
******************************************************************************/
void TNETWIF_rx_start_instance (TI_HANDLE CB_Handle, TI_HANDLE module_id, TI_STATUS status)
{
/*  WLAN_REPORT_ERROR (GWSI_handle->hReport, TNETW_IF_MODULE_LOG,
                        ("\n gwsi_rx_start_instance() : Not implemented Yet !!! \n\n"));*/

    return;
}


#ifdef USE_SYNC_API

TI_STATUS TNETWIF_SetPartitions (TI_HANDLE hTNETWIF, TNETIF_HwAccess_SetPartition_mode_e partitionMode, UINT32 partition_start)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

    return (TI_STATUS)whal_hwAccess_SetPartitions (pTNETWIF->hHwAccess, partitionMode, partition_start);
}

#endif /* USE_SYNC_API */


TI_STATUS TNETWIF_SetPartitionsOpt (TI_HANDLE hTNETWIF, TNETIF_HwAccess_SetPartition_mode_e partitionMode, UINT32 partition_start, UINT8 module_id, TNETWIF_callback_t fCb, TI_HANDLE hCb)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

    /* Register a callback */
    TNETWArb_register_handler (pTNETWIF->hTNETWArb, module_id, fCb, hCb);

    return (TI_STATUS)whal_hwAccess_SetPartitionsAsync (pTNETWIF->hHwAccess, partitionMode, partition_start);
}


/****************************************************************************************
 *                       TNETWIF_RegisterFailureEventCB                                                 *
 ****************************************************************************************
DESCRIPTION: Registers a failure event callback to the elp controler.
                
                                                                                                                   
INPUT:      - hTNETWIF      - handle to the TNETWIF object.     
            - failureEventCB    - the failure event callback function.\n
            - hFailureEventObj - handle to the object passed to the failure event callback function.

OUTPUT: 
RETURN:    void.
****************************************************************************************/

void TNETWIF_RegisterFailureEventCB (TI_HANDLE  hTNETWIF, 
                                     void      *failureEventCB, 
                                     TI_HANDLE  hFailureEventObj )
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;

    elpCtrl_RegisterFailureEventCB (pTNETWIF->hELPCtrl, failureEventCB, hFailureEventObj);    
}


/****************************************************************************************
 *                       TNETWIF_RegisterBusFailureEventCB                                                  *
 ****************************************************************************************
DESCRIPTION: Registers a failure event to the hal ctrl for case of SDIO bus fail.
                
                                                                                                                   
INPUT:      - hTNETWIF      - handle to the TNETWIF object.     
            - failureEventCB    - the failure event callback function.\n
            - hFailureEventObj - handle to the object passed to the failure event callback function.

OUTPUT: 
RETURN:    void.
****************************************************************************************/

void TNETWIF_RegisterBusFailureEventCB (TI_HANDLE  hTNETWIF, 
                                        void      *failureEventCB, 
                                        TI_HANDLE  hFailureEventObj)
{
    TNETWIF_t *pTNETWIF = (TNETWIF_t *)hTNETWIF;
    whal_hwAccess_RegisterForErrorCB (pTNETWIF->hHwAccess, failureEventCB, hFailureEventObj);
    
}

/****************************************************************************
*                      TNETWIF_ModuleIdToString()
****************************************************************************
* DESCRIPTION: Convert the module ID to the Name of the module in string
*
* INPUTS:  UINT32 module_id
*
* OUTPUT:  String the name of the module
*
* RETURNS: 
****************************************************************************/
char* TNETWIF_ModuleIdToString(UINT32 module_id)
{
	switch (module_id)
	{
	case DEFAULT_MODULE_ID:  return "(DEFAULT_MODULE_ID)  ";
	case TX_XFER_MODULE_ID:  return "(TX_XFER_MODULE_ID)  ";
	case HAL_RX_MODULE_ID:   return "(HAL_RX_MODULE_ID )  ";
	case HAL_INT_MODULE_ID:  return "(HAL_INT_MODULE_ID)  ";
	case HAL_CMD_MODULE_ID:  return "(HAL_CMD_MODULE_ID ) ";
	case FW_EVENT_MODULE_ID: return "(FW_EVENT_MODULE_ID) ";
	case HAL_INIT_MODULE_ID: return "(HAL_INIT_MODULE_ID )";

	default : 
		return "(NOT_SUPPORTED)      ";
	}

}

