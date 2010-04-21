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
 *   MODULE:  shmBus.h
 *   PURPOSE: Shared memory Bus access object
 *
 ****************************************************************************/

#ifndef _WHAL_SHM_BUS_H
#define _WHAL_SHM_BUS_H

#include "whalHwEeprom.h"
#include "whalBus_Api.h"
#include "TNETWIF.h"
#include "whalTrace.h" 


/* 
 * Use this flag to optimize FW download.
 * By default this flag is disabled, because the 
 * FW image may be stored at ROM addresses
 */
#undef  USE_NO_CHUNK_COPY


/* Firmware image load chunk size */
#define CHUNK_SIZE          512


/* CLASS whalBus */
typedef struct _whalBus_T
{
    TI_HANDLE               hTNETWIF;
    TI_HANDLE               hTnetwDrv;  
    HwEeprom_T             *pHwEeprom;
    WhalTrace_T            *pTrc;
    TI_HANDLE               hOs;
    TI_HANDLE               hReport;
    TI_HANDLE               hWhalCtrl;
    UINT8                  *pFwBuf;       /* Firmware image ptr */
    UINT32                  uFwLastAddr;  /* Firmware image length */
    UINT8                  *pEEPROMBuf;   /* EEPROM image ptr */
    UINT32                  uEEPROMLen;   /* EEPROM image length */
    UINT8                  *pEEPROMCurPtr;
    UINT32                  uEEPROMCurLen;
    BootAttr_T              BootAttr;
    TI_HANDLE               hHwCtrl;
    TI_STATUS               DownloadStatus;
    fnotify_t               fCb;          /* Upper module callback for the init stage */
    TI_HANDLE               hCb;          /* Upper module handle for the init stage */
    UINT32                  uInitStage;   /* Init stage */
    UINT32                  uResetStage;  /* Reset statge */ 
    UINT32                  uEEPROMStage; /* EEPROM burst stage */
    UINT32                  uInitData;    /* Init state machine temporary data */
    UINT32                  uElpCmd;      /* ELP command image */
    UINT32                  uChipId;      /* Chip ID */
    UINT32                  uBootData;    /* Boot state machine temporary data */
    UINT32                  uSelfClearTime;
    UINT8                   uEEPROMBurstLen;
    UINT8                   uEEPROMBurstLoop;
    UINT32                  uEEPROMRegAddr;
    TI_STATUS               uEEPROMStatus;
    UINT32                  uNVSStartAddr;
    UINT32                  uNVSNumChar;
    UINT32                  uNVSNumByte;
    /* use a struct to write buffers on the bus - used for extra bytes reserving */
    PADDING (UINT32         uNVSTempWord)
    TI_STATUS               uNVSStatus;
    UINT32                  uScrPad6;
    UINT32                  uRefFreq; 
    UINT32                  uInitSeqStage;
    TI_STATUS               uInitSeqStatus;
    UINT32                  uLoadStage;
    UINT32                  uChunkNum;
    UINT32                  uPartitionLimit;
    UINT32                  uFinStage;
    UINT32                  uFinData;
    UINT32                  uFinLoop; 
    /* Temporary buffer for FW chunk storage */
  #ifdef USE_NO_CHUNK_COPY
    UINT8                   auFwTmpBuf [TNETWIF_WRITE_OFFSET_BYTES];
  #else
    UINT8                   auFwTmpBuf [TNETWIF_WRITE_OFFSET_BYTES + CHUNK_SIZE];
  #endif
    /* size of the Fw image, retrieved from the image itself */         
    UINT32                  uFwDataLen; 

	BOOL					recoveryProcess;

} whalBus_T;


/* Debug */
void shmDebug_registerDump(TI_HANDLE hWhalBus);
int  shmDebug_PrintRxRegs(TI_HANDLE hWhalBus);
int  shmDebug_PrintTxRegs(TI_HANDLE hWhalBus);
int  shmDebug_PrintScrPadRegs(TI_HANDLE hWhalBus);
int  shmDebug_PrintListRegs(TI_HANDLE hWhalBus, UINT32 RegAddr);
void shmDebug_MemPrint(TI_HANDLE hWhalBus, UINT32 MemAddr);
void whalBus_MemCopyFrom (TI_HANDLE hWhalBus, UINT8 *Dest, char *SrcOffset, int Len);
void shmDebug_macRegisterDump(TI_HANDLE hWhalBus);
void shmDebug_phyRegisterDump(TI_HANDLE hWhalBus);


#endif /* _WHAL_SHM_BUS_H */
