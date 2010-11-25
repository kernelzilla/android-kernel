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

/*--------------------------------------------------------------------------*/
/* Module:      OSAPI.H*/
/**/
/* Purpose:     This module defines unified interface to the OS specific*/
/*              sources and services.*/
/**/
/*--------------------------------------------------------------------------*/

#ifndef __OS_API_H__
#define __OS_API_H__


#include "osTIType.h"
#include "TI_IPC_Api.h"
//TRS:MEB
#ifdef _WINDOWS
#endif

//TRS end


/****************************************************************************************
						START OF OS API (Common to all GWSI LIB, Driver and TI Driver)				
*****************************************************************************************/


#define OS_API_MEM_ADRR  0x0000000
#define OS_API_REG_ADRR  0x300000


/****************************************************************************************
                        OS HW API NEEDED BY DRIVER              
*****************************************************************************************/


#define OS_PAGE_SIZE 4096

/* 
Clear the WLAN Ready Interrupt Line stored in the PIC Controller
*/
VOID os_clearWlanReady(void);

/* 
Read the WLAN_IRQ line
*/
UINT32 os_senseIrqLine( TI_HANDLE OsContext );

/* TRS: CDB Needed for osApi.C */
#ifdef BSP_H5
#define WLAN_GPIO_INTERRUPT_LINE		9
#define WLAN_GPIO_POWER_ENABLET_LINE    10
#else
#define WLAN_GPIO_INTERRUPT_LINE    134
#define WLAN_GPIO_POWER_ENABLET_LINE    71
#endif


VOID
os_disableIrq( 
    TI_HANDLE OsContext 
    );

VOID
os_enableIrq( 
    TI_HANDLE OsContext
    );

/****************************************************************************************
 *                        																*
 *						OS Report API													*       
 *																						*
 ****************************************************************************************/

VOID 
os_setDebugMode(
	BOOL enable
	);

VOID 
os_printf(
	const char *format ,...);

VOID os_setDebugOutputToTicon(BOOL value);

#define os_report   os_printf
#define osPrintf os_printf  //TRS::CDB

/****************************************************************************************
 *                        																*
 *							OS DMA CALLBACK API											*
 ****************************************************************************************
 *	Callback directly called at an IRQ context from the SPI modue						*
 *	This should triger a tasklet_schedule so that the End of DMA will be handled		*
 *	in a tasklet  context and then be directed to the TNETWIF to call the Client 		*
 *																						*       
 *																						*
 ****************************************************************************************/

VOID 
os_TNETWIF_BusTxn_Complete(
	TI_HANDLE OsContext,
	int status
	);


/****************************************************************************************
 *                        																*
 *						OS Memory API													*       
 *																						*
 ****************************************************************************************/

PVOID
os_memoryAlloc(
    TI_HANDLE OsContext,
    UINT32 Size
    );

PVOID
os_memoryPreAlloc(
    TI_HANDLE OsContext,
    int section,
    UINT32 Size
    );

PVOID
os_memoryCAlloc(
    TI_HANDLE OsContext,
    UINT32 Number,
    UINT32 Size
    );

VOID
os_memorySet(
    TI_HANDLE OsContext,
    PVOID pMemPtr,
    INT32 Value,
    UINT32 Length
    );

VOID
os_memoryZero(
    TI_HANDLE OsContext,
    PVOID pMemPtr,
    UINT32 Length
   );

VOID
os_memoryCopy(
    TI_HANDLE pOsContext,
    PVOID pDestination,
    PVOID pSource,
    UINT32 Size
   );

VOID
os_memoryMove(
    TI_HANDLE pOsContext,
    PVOID pDestination,
    PVOID pSource,
    UINT32 Size
   );

VOID
os_memoryFree(
    TI_HANDLE pOsContext,
    PVOID pMemPtr,
    UINT32 Size
    );

INT32
os_memoryCompare(
    TI_HANDLE OsContext,
    PUINT8 Buf1,
    PUINT8 Buf2,
    INT32 Count
    );

PVOID
os_memoryAlloc4HwDma(
    TI_HANDLE pOsContext,
    UINT32 Size
    );

VOID
os_memory4HwDmaFree(
    TI_HANDLE pOsContext,
    PVOID pPMem_ptr,
    UINT32 Size
    );


/****************************************************************************************
 *                        																*
 *							OS TIMER API												*
 *																						*
 ****************************************************************************************/
typedef void (*PTIMER_FUNCTION)(TI_HANDLE Context);

TI_HANDLE
os_timerCreate(
    TI_HANDLE OsContext,
    PTIMER_FUNCTION pRoutine,
    TI_HANDLE Context
    );

VOID
os_timerDestroy(
    TI_HANDLE OsContext,
    TI_HANDLE TimerHandle
    );

VOID
os_timerStart(
    TI_HANDLE OsContext,
    TI_HANDLE TimerHandle,
    UINT32 DelayMs,
    BOOL bPeriodic
    );

VOID
os_timerStop(
    TI_HANDLE OsContext,
    TI_HANDLE TimerHandle
    );

VOID
os_periodicIntrTimerStart(
	TI_HANDLE OsContext
	);

UINT32
os_timeStampMs(
    TI_HANDLE OsContext
    );

UINT32
os_timeStampUs(
    TI_HANDLE OsContext
    );

VOID
os_StalluSec(
    TI_HANDLE OsContext,
    UINT32 uSec
    );

void os_ToggleDebugGPIO(int count);


/****************************************************************************************
 *                        																*
 *							Hardware access functions	API								*
 *																						*
 ****************************************************************************************/
PVOID
os_hwGetRegistersAddr(
    TI_HANDLE OsContext
    );


PVOID
os_hwGetMemoryAddr(
    TI_HANDLE OsContext
    );


/****************************************************************************************
 *                        																*
 *							Protection services	API										*
 *																						*
 ****************************************************************************************
 * OS protection is implemented as dummy functions because								*
 * all driver code is executed in context of a single tasklet,							*
 * except IOCTL handlers and xmition.													*
 * Protection in IOCTL handlers and hard_start_xmit is done by different				*
 * means.																				*
 ****************************************************************************************/
TI_HANDLE
os_protectCreate(
    TI_HANDLE OsContext
    );

VOID
os_protectDestroy(
    TI_HANDLE OsContext,
    TI_HANDLE ProtectContext
    );

VOID
os_protectLock(
    TI_HANDLE OsContext,
    TI_HANDLE ProtectContext
    );

VOID
os_protectUnlock(
    TI_HANDLE OsContext,
    TI_HANDLE ProtectContext
    );



#ifdef DRIVER_PROFILING
  void _os_profile (TI_HANDLE OsContext, UINT32 fn, UINT32 par);
  #define os_profile(hos,fn,par) _os_profile (hos, fn, par)
#else
  #define os_profile(hos,fn,par)
#endif


/****************************************************************************************
						START OF GWSI DRIVER API				
*****************************************************************************************/
VOID 
os_Complete(
	TI_HANDLE OsContext
	);

#ifndef GWSI_LIB

VOID 
os_WaitComplete(
	TI_HANDLE OsContext
	);

UINT32
os_memoryGetPhysicalLow (OS_PHYSICAL_ADDRESS pAddr);
UINT32 
os_memoryGetPhysicalHigh (OS_PHYSICAL_ADDRESS pAddr);

/* MEB use native NDIS functions */
#ifdef _WINDOWS
#else

  UINT32 os_memoryGetPhysicalLow(OS_PHYSICAL_ADDRESS pAddr);
  UINT32 os_memoryGetPhysicalHigh(OS_PHYSICAL_ADDRESS pAddr);

#endif

  VOID os_hardResetTnetw(void);

#endif


/****************************************************************************************
						START OF TI DRIVER API				
*****************************************************************************************/
#if !defined(GWSI_DRIVER) && !defined(GWSI_LIB)

typedef struct {
    UINT32      Event;
    UINT8*      Data;
} TI_CONNECTION_STATUS, *PTI_CONNECTION_STATUS;



PVOID
os_memoryAlloc4HwCopy(
    TI_HANDLE pOsContext,
    UINT32 Size
    );

VOID
os_memorySharedFree(
    TI_HANDLE OsContext,
    PVOID pVirtual,
    UINT32 Size,
    OS_PHYSICAL_ADDRESS pPhysical
    );

PVOID
os_memorySharedAlloc(
    TI_HANDLE OsContext,
    UINT32 Size,
    OS_PHYSICAL_ADDRESS *pPhysical
    );

VOID
os_memoryMoveToHw(
    TI_HANDLE OsContext,
    PVOID pTarget,
    PVOID pSource,
    UINT32 Size
    );

VOID
os_memoryMoveFromHw(
    TI_HANDLE OsContext,
    PVOID pTarget,
    PVOID pSource,
    UINT32 Size
    );


/**/
/* Register access functions*/
/**/
VOID
os_hwReadMemRegisterUINT32(
    TI_HANDLE OsContext,
    PUINT32 Register,
    PUINT32 Data
    );

VOID
os_hwWriteMemRegisterUINT32(
    TI_HANDLE OsContext,
    PUINT32 Register,
    UINT32 Data
    );

VOID
os_hwReadMemRegisterUINT16(
    TI_HANDLE OsContext,
    PUINT16 Register,
    PUINT16 Data
    );

VOID
os_hwWriteMemRegisterUINT16(
    TI_HANDLE OsContext,
    PUINT16 Register,
    UINT16 Data
    );

VOID
os_hwReadMemRegisterUINT8(
    TI_HANDLE OsContext,
    PUINT8 Register,
    PUINT8 Data
    );

VOID
os_hwWriteMemRegisterUINT8(
    TI_HANDLE OsContext,
    PUINT8 Register,
    UINT8 Data
    );

int
os_getFirmwareImage(
    TI_HANDLE OsContext,
    PUINT8 *pBuffer,
    PUINT32 Length,
    UINT8 RadioType
    );

int
os_getRadioImage(
    TI_HANDLE OsContext,
    PUINT8 *pBuffer,
    PUINT32 Length,
    UINT8 RadioType
    );

VOID
os_closeFirmwareImage( TI_HANDLE OsContext );

VOID
os_closeRadioImage( TI_HANDLE OsContext );

BOOL
os_receivePacket(
    TI_HANDLE OsContext,
    PVOID pPacket,
    UINT16 Length
    );

INT32
os_sendPacket(
        TI_HANDLE OsContext,
        PVOID pPacket,
        UINT16 Length
        );

tiINT32
os_IndicateEvent(
    IPC_EV_DATA* pData
    );

VOID
os_powerStateBusy(
    TI_HANDLE OsContext
    );

VOID
os_powerStateIdle(
    TI_HANDLE OsContext
    );

VOID
os_setWakeOnGpio(
    TI_HANDLE OsContext
    );

VOID
os_resetWakeOnGpio(
    TI_HANDLE OsContext
    );

BOOL
os_getEeepromImage(
    TI_HANDLE OsContext,
    PUINT8* pBuffer,
    PUINT32 length
    );

// TRS:JCG missing in 4.03
VOID 
os_setPowerOfTnetw(
    BOOL bPowerOn
    );

#endif

#endif
