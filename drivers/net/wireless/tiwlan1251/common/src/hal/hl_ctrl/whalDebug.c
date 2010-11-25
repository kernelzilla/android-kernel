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
 *   MODULE:  whalDebug.c
 *   PURPOSE: Handle Debug requests in the Hal
 *
 *	Register Mac/Phy Read/Write
 *	Register Dump
 *	Register RxDump
 *	Register TxDump
 *	Debug trace print
 *	
 ****************************************************************************/
#include "whalCommon.h"
#include "whalCtrl_api.h"
#include "whalCtrl.h"
#include "whalSecurity.h"
#include "commonTypes.h"
#include "CmdQueue_api.h"
#include "whalBus_Api.h"
#include "TNETW_Driver.h"

/************************************************
 *                  definitions                 *
 ************************************************/
#define BB_REGISTER_ADDR_BASE           0x820000  /*phony address used by host access*/

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrlReadMacReg
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  : Done 
 * -----------------------------------------------------------------------------
 */
UINT32 whalCtrlReadMacReg(TI_HANDLE hWhalCtrl, UINT32 addr)
{
	WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
	
	return whalBus_MacRegRead(pWhalCtrl->pHwCtrl->hWhalBus, addr);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrlWriteMacReg
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  : Done 
 * -----------------------------------------------------------------------------
 */
void whalCtrlWriteMacReg(TI_HANDLE hWhalCtrl, UINT32 addr, UINT32	val)
{
	WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

	whalBus_MacRegWrite(pWhalCtrl->pHwCtrl->hWhalBus, addr, val);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrlReadPhyReg
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  : Done 
 * -----------------------------------------------------------------------------
 */
UINT32 whalCtrlReadPhyReg(TI_HANDLE hWhalCtrl, UINT32 addr)
{
	WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

	return whalBus_PhyRegRead(pWhalCtrl->pHwCtrl->hWhalBus, addr);
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrlWritePhyReg
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  : Done 
 * -----------------------------------------------------------------------------
 */
void whalCtrlWritePhyReg(TI_HANDLE hWhalCtrl, UINT32 addr, UINT32	val)
{
	WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

	whalBus_PhyRegWrite(pWhalCtrl->pHwCtrl->hWhalBus, addr, val);
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_PrintAll
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  : Done 
 * -----------------------------------------------------------------------------
 */
#ifdef TI_DBG
void whalCtrl_PrintAll (TI_HANDLE hWhalCtrl)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

	WLAN_OS_REPORT((" whalCtrl_PrintAll: \n\n"));
	
	CmdQueue_Print(((TnetwDrv_t*)pWhalCtrl->hTNETW_Driver)->hCmdQueue);
	
	whalBus_PrintInfo(pWhalCtrl->hWhalBus, BUS_PRINT_ALL, 0);

	whal_hwInfoElemMemoryMapPrint(pWhalCtrl->pHwCtrl->pHwMboxConfig);			
	whal_hwInfoElemStatisticsPrint(pWhalCtrl->pHwCtrl->pHwMboxConfig);
	
	whal_ParamsHwNvramPrint(pWhalCtrl->pWhalParams);			
}
#endif


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_getMaxNumberOfCommandsInQueue
 *
 * Input    : 
 * Output   :
 * Process  : returns the maximum number of commands in the mailbox queue ever
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */
int whalCtrl_getMaxNumberOfCommandsInQueue (TI_HANDLE hWhalCtrl)
{
   WHAL_CTRL   *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;	
   return (CmdQueue_GetMaxNumberOfCommands(((TnetwDrv_t*)pWhalCtrl->hTNETW_Driver)->hCmdQueue));
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_PrintMailBoxHistory
 *
 * Input    : 
 * Output   :
 * Process  : Prints the mailbox command history
 * Note(s)  :  Done
 * -----------------------------------------------------------------------------
 */

void whalCtrl_PrintMailBoxHistory (TI_HANDLE hWhalCtrl)
{
#ifdef TI_DBG
    WHAL_CTRL   *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;	
	CmdQueue_PrintHistory(((TnetwDrv_t*)pWhalCtrl->hTNETW_Driver)->hCmdQueue, 5);
#endif
}
                          
/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_PrintMem_Regs_CB
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  : Done 
 * -----------------------------------------------------------------------------
 */
void whalCtrl_PrintMem_Regs_CB (TI_HANDLE hWhalCtrl, UINT32 cmdCbStatus)
{
    int i;
    UINT8   *pBuf;
    UINT32  result;
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

    if (cmdCbStatus != OK)
    {
        WLAN_OS_REPORT((" whalCtrl_PrintMem_Regs_CB: command complete error \n\n"));
        return;
    }

    result = (((UINT32)pWhalCtrl->printRegsBuf.addr)&0xFFFF0000);
    
    switch (result)
    {
        case ACX_MAC_REG_READ_WRITE_PREFIX:
                WLAN_OS_REPORT(("MAC REGS (Base=0x%08x) = 0x%08x\n", ((UINT32)pWhalCtrl->printRegsBuf.addr)&0xFFFF,
                                *(UINT32*)(pWhalCtrl->printRegsBuf.value)));
                break;

        case BB_REGISTER_ADDR_BASE:
                WLAN_OS_REPORT(("PHY REGS (Base=0x%08x) = 0x%08x\n", ((UINT32)pWhalCtrl->printRegsBuf.addr)&0xFFFF,
                            *(UINT32*)(pWhalCtrl->printRegsBuf.value)));
                break;

        default: /* Memory*/
                for (i=0, pBuf=pWhalCtrl->printRegsBuf.value; i<256; i+=16, pBuf+=16)
                {
                    WLAN_REPORT_REPLY(pWhalCtrl->hReport, HAL_HW_CTRL_MODULE_LOG,  
                                      ("PrintBuf: 0x%08x: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", 
                                       pWhalCtrl->printRegsBuf.addr+i, 
                                       pBuf[0], pBuf[1], pBuf[2], pBuf[3], pBuf[4], pBuf[5], pBuf[6], pBuf[7], 
                                       pBuf[8], pBuf[9], pBuf[10], pBuf[11], pBuf[12], pBuf[13], pBuf[14], pBuf[15]));
                }
                break;
    }
}

/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_Print_Mem_Regs
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  : Reading from memory or registers is done through IE (interrogate)
 * -----------------------------------------------------------------------------
 */
void whalCtrl_Print_Mem_Regs (TI_HANDLE hWhalCtrl, UINT32 address, UINT32 len, readWrite_MemoryType_e memType)
{
    ReadWriteCommand_t  AcxCmd_ReadMemory; 
    ReadWriteCommand_t* pCmd = &AcxCmd_ReadMemory;

    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;

	os_memoryZero(pWhalCtrl->hOs, (void *)pCmd, sizeof(*pCmd));
    
    switch (memType)
    {
        case TNETW_INTERNAL_RAM:
            pCmd->addr = (UINT32)ENDIAN_HANDLE_LONG(address);
            pCmd->size = ENDIAN_HANDLE_LONG(len);
            break;
             
        case TNETW_MAC_REGISTERS:
            pCmd->addr = (UINT32)ENDIAN_HANDLE_LONG(( (address&0xFFFF) | ACX_MAC_REG_READ_WRITE_PREFIX ));
            pCmd->size = 4;
            break;
    
        case TNETW_PHY_REGISTERS:
            pCmd->addr = (UINT32)ENDIAN_HANDLE_LONG(( (address&0xFFFF) | BB_REGISTER_ADDR_BASE ));
            pCmd->size = 4;
            break;
    
        default:
            WLAN_OS_REPORT((" whalCtrl_Print_Mem_Regs: ERROR, wrong memory type %d\n\n", memType));
            return;
    }
    
    os_memoryZero (pWhalCtrl->hOs, (void *)&pWhalCtrl->printRegsBuf, sizeof(pWhalCtrl->printRegsBuf));
    
    CmdQueue_CommandWithCb(((TnetwDrv_t*)pWhalCtrl->hTNETW_Driver)->hCmdQueue, CMD_READ_MEMORY, (char *)pCmd, 
                                sizeof(*pCmd), (void *)whalCtrl_PrintMem_Regs_CB, hWhalCtrl, 
                                &pWhalCtrl->printRegsBuf);
}


/*
 * ----------------------------------------------------------------------------
 * Function : whalCtrl_Set_Mem_Regs
 *
 * Input    : 
 * Output   :
 * Process  :
 * Note(s)  : Writing to memory or registers is done through command to the ACX
 *            This function overrides the len parameter and write only 4 bytes!
 * -----------------------------------------------------------------------------
 */
int whalCtrl_Set_Mem_Regs (TI_HANDLE hWhalCtrl, UINT32 address, UINT32 len, UINT32 aWriteVal, 
                           readWrite_MemoryType_e memType)
{
    WHAL_CTRL *pWhalCtrl = (WHAL_CTRL *)hWhalCtrl;
    ReadWriteCommand_t  AcxCmd_WriteMemory; 
    ReadWriteCommand_t* pCmd = &AcxCmd_WriteMemory;

	os_memoryZero(pWhalCtrl->hOs, (void *)pCmd, sizeof(*pCmd));
    
    switch (memType)
    {
        case TNETW_INTERNAL_RAM:
            pCmd->addr = (UINT32)ENDIAN_HANDLE_LONG(address);
            pCmd->size = ENDIAN_HANDLE_LONG(len);
            WLAN_OS_REPORT((" whalCtrl_Set_Mem_Regs: write to Internal Ram addr=0x%x Val=0x%x\n\n", pCmd->addr, aWriteVal));
            break;
             
        case TNETW_MAC_REGISTERS:
            pCmd->addr = (UINT32)ENDIAN_HANDLE_LONG( (address&0xFFFF) | ACX_MAC_REG_READ_WRITE_PREFIX);
            pCmd->size = ENDIAN_HANDLE_LONG(len);
            WLAN_OS_REPORT((" whalCtrl_Set_Mem_Regs: write to Mac register addr=0x%x Val=0x%x\n\n", pCmd->addr, aWriteVal));
            break;
    
        case TNETW_PHY_REGISTERS:
            pCmd->addr = (UINT32)ENDIAN_HANDLE_LONG( (address&0xFFFF) | BB_REGISTER_ADDR_BASE);
            pCmd->size = ENDIAN_HANDLE_LONG(len);
            WLAN_OS_REPORT((" whalCtrl_Set_Mem_Regs: write to Phy register addr=0x%x Val=0x%x\n\n", pCmd->addr, aWriteVal));
            break;
    
        default:
            WLAN_OS_REPORT((" whalCtrl_Print_Mem_Regs: ERROR, wrong memory type 0x%x\n\n", memType));
            return NOK;
    }
    
    os_memoryCopy (pWhalCtrl->hOs, (void *)pCmd->value, (void *)&aWriteVal, pCmd->size);

    return (CmdQueue_Command(((TnetwDrv_t*)pWhalCtrl->hTNETW_Driver)->hCmdQueue, CMD_WRITE_MEMORY, (char *)pCmd, sizeof(*pCmd)));
}

                          

