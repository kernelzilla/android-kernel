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
 *   MODULE:  hwAccess.c
 *   PURPOSE: Support access to the wlan hardware registers and memory
 *
 ****************************************************************************/

#ifndef __WHAL_HW_ACCESS_H__
#define __WHAL_HW_ACCESS_H__

/************************************************************************
 * partition addresses
 * 
 * each access region will have its base address and the HAL will use only the logical offset
 * there isnt an access region for the registers. 
 * for the mem the access regions are defined for their functionality : double-buffer,tx registers, rx registers etc...
 *
 * EXAMPLE : 
 *	if the HAL wants to read register at offset 0xA the the call will be :
 *  whal_hwAccess_ReadReg(hHwAccess, 0xA, &RegVal);
 *	if the HAL wants to read 4bytes from mem at offset 0x10 in the double-buffer region the the call will be:
 *  whal_hwAccess_ReadMem(hHwAccess, HW_ACCESS_BASE_DOUBLE_BUFFER + 0x10, &RegVal , 4);
 ************************************************************************/

#define HW_ACCESS_BASE_DOUBLE_BUFFER	0
#define HW_ACCESS_BASE_CMD_MBX			0

#define HEALTH_REPORT_BUS_ERROR BIT_2

/************************************************************************
 * Return codes
 ************************************************************************/
#define ERROR_HW_ACCEESS_ADDR	10
#define ERROR_HW_ACCEESS_LEN	11

/************************************************************************
 * Types
 ************************************************************************/
typedef void (*HwAccess_callback_t)(void *data,int status);

/************************************************************************
 * partition addresses
 ************************************************************************/
  
/* Download phase */
#define HW_ACCESS_DOWN_PART0_SIZE		0x16800
#define HW_ACCESS_DOWN_PART0_ADDR		0x0
#define HW_ACCESS_DOWN_PART1_SIZE		0x8800
#define HW_ACCESS_DOWN_PART1_ADDR		0x300000

/* Working phase */
#ifdef TNETW1251
#define HW_ACCESS_WORK_PART0_SIZE		0x14000
#define HW_ACCESS_WORK_PART0_ADDR		0x28000
#define HW_ACCESS_WORK_PART1_SIZE		0xB000
#else
#define HW_ACCESS_WORK_PART0_SIZE		0x16800
#define HW_ACCESS_WORK_PART0_ADDR		0xF000
#define HW_ACCESS_WORK_PART1_SIZE		0x8800
#endif
#define HW_ACCESS_WORK_PART1_ADDR		0x300000

#define HW_ACCESS_MAX_PARTITIONS 		2

#ifndef HW_ACCESS_MEMORY_MAX_RANGE  /* this macro is already defined in SDIO client driver */
  #define HW_ACCESS_MEMORY_MAX_RANGE 		0x1FFC0
#endif /* ifndef HW_ACCESS_MEMORY_MAX_RANGE */

#define HW_ACCESS_PRAM_MAX_RANGE 		0x3c000

/************************************************************************
 * Functions
 ************************************************************************/

extern int whal_hwAccess_ReConfig(TI_HANDLE hHwAccess);
extern int whal_hwAccess_Stop(TI_HANDLE hHwAccess);

#if (defined (HW_ACCESS_SDIO) || defined (HW_ACCESS_WSPI))
  TI_STATUS whal_hwAccess_RecreateInterface(TI_HANDLE hHwAccess);
#endif

/* new API */
/***********/

TI_HANDLE	whal_hwAccess_Create(TI_HANDLE hOs);
int			whal_hwAccess_Destroy(TI_HANDLE hHwAccess);
int			whal_hwAccess_Config(TI_HANDLE hHwAccess, TI_HANDLE hReport,UINT32 RegBaseAddr, UINT32 MemBaseAddr, HwAccess_callback_t CBFunc,void* CBArg);

int			whal_hwAccess_SetPartitions(TI_HANDLE hHwAccess, TNETIF_HwAccess_SetPartition_mode_e partitionMode, UINT32 partition_start);
int			whal_hwAccess_SetPartitionsAsync(TI_HANDLE hHwAccess, TNETIF_HwAccess_SetPartition_mode_e partitionMode, UINT32 partition_start);
UINT8		whal_hwAccess_Get_Async_Mode(TI_HANDLE hHwAccess);
int			whal_hwAccess_WriteELP(TI_HANDLE hHwAccess, UINT32 data);
int			whal_hwAccess_WriteELPAsync(TI_HANDLE hHwAccess, UINT32 data, BOOL bCb, BOOL bMore);
int			whal_hwAccess_ReadELPAsync (TI_HANDLE hHwAccess, UINT8 *data, BOOL bCb, BOOL bMore);

int			whal_hwAccess_ReadMem_Align(TI_HANDLE hHwAccess, UINT32 addr, UINT8* data, UINT16 len);
int			whal_hwAccess_WriteMem_Align(TI_HANDLE hHwAccess, UINT32 addr, UINT8* data, UINT16 len);
int			whal_hwAccess_ReadMemAsync_Align(TI_HANDLE hHwAccess, UINT32 addr, UINT8* data, UINT16 len);
int			whal_hwAccess_WriteMemAsync_Align(TI_HANDLE hHwAccess, UINT32 addr, UINT8* data, UINT16 len);

int			whal_hwAccess_ReadMem(TI_HANDLE hHwAccess, UINT32 addr, UINT8* data, UINT16 len);
int			whal_hwAccess_WriteMem(TI_HANDLE hHwAccess, UINT32 addr, UINT8* data, UINT16 len);
int			whal_hwAccess_ReadMemAsync(TI_HANDLE hHwAccess, UINT32 addr, UINT8* data, UINT16 len);
TI_STATUS	whal_hwAccess_WriteMemAsync(TI_HANDLE hHwAccess, UINT32 addr, UINT8* data, UINT16 len);

int			whal_hwAccess_ReadReg(TI_HANDLE hHwAccess, UINT32 addr, UINT32* data);
int			whal_hwAccess_WriteReg(TI_HANDLE hHwAccess, UINT32 addr, UINT32 data);
int			whal_hwAccess_ReadRegAsync(TI_HANDLE hHwAccess, UINT32 addr, UINT32* data);
int			whal_hwAccess_WriteRegAsync(TI_HANDLE hHwAccess, UINT32 addr, UINT32 data);

void			whal_hwAccess_Print(TI_HANDLE hHwAccess);

/*old - new*/
/*
 * Hardware memory Api
 */
UINT8   		whal_hwAccess_GetU08       (TI_HANDLE hHwAccess, UINT32 Addr);
void     		whal_hwAccess_SetU08       (TI_HANDLE hHwAccess, UINT32 Addr, UINT8  Val);
void     		whal_hwAccess_SetU08_Bits  (TI_HANDLE hHwAccess, UINT32 Addr, UINT8  BitsVal);
void     		whal_hwAccess_ResetU08_Bits(TI_HANDLE hHwAccess, UINT32 Addr, UINT8  BitsVal);
UINT16   		whal_hwAccess_GetU16       (TI_HANDLE hHwAccess, UINT32 Addr);
void     		whal_hwAccess_SetU16       (TI_HANDLE hHwAccess, UINT32 Addr, UINT16 Val);
void     		whal_hwAccess_SetU16_Bits  (TI_HANDLE hHwAccess, UINT32 Addr, UINT16 BitsVal);
void     		whal_hwAccess_ResetU16_Bits(TI_HANDLE hHwAccess, UINT32 Addr, UINT16 BitsVal);
UINT32   		whal_hwAccess_GetU32       (TI_HANDLE hHwAccess, UINT32 Addr);
void     		whal_hwAccess_SetU32       (TI_HANDLE hHwAccess, UINT32 Addr, UINT32 Val);
void     		whal_hwAccess_SetU32_Bits  (TI_HANDLE hHwAccess, UINT32 Addr, UINT32 BitsVal);
void     		whal_hwAccess_ResetU32_Bits(TI_HANDLE hHwAccess, UINT32 Addr, UINT32 BitsVal);

/*
 * Hardware Registers Api
 */
void     		whal_hwAccess_RegSetBitVal  (TI_HANDLE hHwAccess, UINT32 RegAddr, UINT32 BitVal);
void     		whal_hwAccess_RegResetBitVal(TI_HANDLE hHwAccess, UINT32 RegAddr, UINT32 BitVal);
int      		whal_hwAccess_RegIsBitSet   (TI_HANDLE hHwAccess, UINT32 RegAddr, UINT32 BitVal);




TI_STATUS whal_hwAccess_RegisterForErrorCB(TI_HANDLE hHwAccess,void* CbFunc,TI_HANDLE CbObj);


#endif /*__WHAL_HW_ACCESS_H__*/
