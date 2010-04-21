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
 *  Direct Slave mode:   
 *  -----------------
 *
 *      1. 08 bit function 
 *              - access 16 bit (WA100 has no access to 8 bits)
 *              - set/get the relevant byte according to the address (odd or even)
 *              + ((char *)&DataShort)[Addr&0x1]
 *              - no endian handle 
 *      2. 16 bit function 
 *              - access 16 bit 
 *              - short endian handle 
 *      3. 32 bit function 
 *              - access 32 bit 
 *              - long endian handle 
 *      4. buffers copy to (stream of bytes)
 *              - addresses must be even
 *              - copy buffer as stream of 16 bits (in case of src/dst address ends with 0x2)
 *              - handle case of more bytes to copy
 *              * TempWord = (*shortDest & 0x00ff) | (*shortSrc & 0xff00);
 *              - no endian handle 
 *      5. registers
 *              - access 32 bit 
 *              - long endian handle 
 *              - no use of wlan hardware capability to swap endian
 *
 *  Indirect Slave mode:
 *  -------------------
 *
 *      1. 08 bit function 
 *              - access 16 bit (WA100 has no access to 8 bits)
 *              - set/get the relevant byte according to the address (odd or even)
 *              + ((char *)&DataLong)[Addr&0x3]
 *              - no endian handle  
 *      2. 16 bit function 
 *              - access 32 bit (set addr reg , get data reg) 
 *              - set/get the relevant short according to the address (00 or 02) 
 *              + ((short *)&DataLong)[(Addr>>1)&0x1])
 *              - short endian handle 
 *      3. 32 bit function 
 *              - access 32 bit (set addr reg , get data reg) 
 *              - long endian handle 
 *      4. buffers copy to (stream of bytes)
 *              - addresses must be even
 *              - handle case of dest(wlan hardware) address ends with 0x2 - read 32 from 0x0, set only high short
 *              - now the dest(wlan hardware) address is long address
 *              - use Auto Increment Mode
 *              - copy buffer as stream of 16 bits (in case of source address ends with 0x2)
 *              - handle case of more bytes to copy
 *              * i=0..Len&3 ==> ((char *)&DataLong)[i] = ((char *)shortSrc)[i]
 *              - no endian handle 
 *      5. buffers copy from (stream of bytes)
 *              - addresses must be even
 *              - handle case of source(wlan hardware) address ends with 0x2 - read 32 from 0x0, set only high short
 *              - now the source(wlan hardware) address is long address
 *              - use Auto Increment Mode
 *              - copy buffer as stream of 16 bits (in case of dest address ends with 0x2)
 *              - handle case of more bytes to copy
 *              * i=0..Len&3 ==> ((char *)shortDest)[i] = ((char *)&DataLong)[i]
 *              - no endian handle 
 *      6. registers
 *              - access 32 bit 
 *              - long endian handle 
 *              - no use of wlan hardware capability to swap endian
 *
 ****************************************************************************/
#include "osTIType.h"
#include "osApi.h"
#include "whalCommon.h"
#include "whalHwDefs.h"
#ifdef HW_ACCESS_SDIO 

  #ifndef _WINDOWS  /*Linux, Symbian, RVCT */

#include "mmc_omap_api.h"
#include "mmc_tnetw1150_api.h"

  #else /* ifdef _WINDOWS */
  #endif /* ifdef _WINDOWS */

#elif HW_ACCESS_WSPI

#include "wspi.h" 

#endif
#include "TNETWIF.h"
#include "whalHwAccess.h"

/* #define __HWACCESS_DEBUG__ */

/*
 * Define this flag to support SDIO asynchronous mode
 */
#undef HW_ACCESS_SDIO_ASYNC_SUPPORT


/************************************************************************
 * Types
 ************************************************************************/
typedef struct _HWAccess_CB_T
{
    HwAccess_callback_t CBFunc;
    void* CBArg;
} HWAccess_CB_T;

typedef void (*HwAccessErrorHandle)(TI_HANDLE theObjectHandle,char* Report , UINT32 strLen);

typedef  struct _partition_t
{
    UINT32 size;
    UINT32 start;
} partition_t;


/* HwAccess context */
typedef struct _HwAccess_T_new
{
    void       *hProtect;

    TI_HANDLE   hOs;           
    TI_HANDLE   hReport;

#if (defined(HW_ACCESS_SDIO)|defined(HW_ACCESS_WSPI))
    TI_HANDLE   hDriver;
    UINT32      MemRegionAddr;
    UINT32      RegisterRegionAddr;
    UINT32      workingPartUpperLimit;
    UINT32      registerPartUpperLimit;
#else /* HW_ACCESS_CARDBUS */   
    UINT32      RegBaseAddr;
    UINT32      MemBaseAddr;
#endif

    HWAccess_CB_T CB;
    
    UINT8       AsyncMode;

    UINT32      uBusError;
    HwAccessErrorHandle hwAccesserror_Cb;
    TI_HANDLE   hBackReference;

    PADDING (partition_t partition [2])

} HwAccess_T_new;


/************************************************************************
 * Defines
 ************************************************************************/

#ifdef HW_ACCESS_WSPI

/*
 *  Converts status from WSPI into TI_STATUS
 */
#define WSPI2TNETWIF(pHwAccess,status,addr)                                   \
    switch(status)  {                                                         \
        case WSPI_TXN_PENDING:  status = TNETWIF_PENDING; break;              \
        case WSPI_TXN_COMPLETE: status = TNETWIF_COMPLETE; break;             \
        default:                                                              \
            WLAN_REPORT_ERROR (pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,    \
                               ("whal_hwAccess: Error in read/write async, addr=0x%08x status=%d\n", \
                               addr, status));                                \
            status = TNETWIF_ERROR; break;                                    \
}
              
/* 
* Indicate the index position at which we should check if the HW is up - 
* i.e. (buf[HW_ACCESS_WSPI_FIXED_BUSY_LEN] & 0x1 == TRUE)
*/
#ifdef TNETW1251
#define HW_ACCESS_WSPI_FIXED_BUSY_LEN       ((TNETWIF_READ_OFFSET_BYTES - 4 ) / sizeof(UINT32))
#else
#define HW_ACCESS_WSPI_FIXED_BUSY_LEN       0
#endif

#endif /* HW_ACCESS_WSPI */

#define HW_ACCESS_WSPI_INIT_CMD_MASK        0

#define HW_ACCESS_WSPI_ALIGNED_SIZE         4
#define HW_ACCESS_NUM_OF_BIT_IN_BYTE        8

#define HW_ACCESS_REGISTER_SIZE             4


/* ELP CTRL  register */
#define HW_ACCESS_ELP_CTRL_REG_ADDR         0x1FFFC


#define HW_ACCESS_1_BYTE_REMINDE_MASK       0x000000FF
#define HW_ACCESS_2_BYTE_REMINDE_MASK       0x0000FFFF
#define HW_ACCESS_3_BYTE_REMINDE_MASK       0x00FFFFFF

/* translation registers */
#define HW_ACCESS_PART0_SIZE_ADDR           0x1FFC0
#define HW_ACCESS_PART0_START_ADDR          0x1FFC4
#define HW_ACCESS_PART1_SIZE_ADDR           0x1FFC8
#define HW_ACCESS_PART1_START_ADDR          0x1FFCC


/************************************************************************
 * Macros
 ************************************************************************/
#define EXTRACT_BYTE_FROM_WORD(DataShort, Addr)     (((char *)&DataShort)[((int)Addr)&0x1])
#define EXTRACT_BYTE_FROM_LONG(DataLong, Addr)      (((char *)&DataLong )[((int)Addr)&0x3])
#define EXTRACT_WORD_FROM_LONG(DataLong, Addr)      (((short *)&DataLong)[(((int)Addr>>1))&0x1])
#define EXTRACT_BYTE_LONG(DataLong, i)              (((char *)&DataLong)[i])

#define HW_MEM_SHORT(pHwAccess, Addr)  (*(volatile UINT16 *)(pHwAccess->MemBaseAddr + (UINT32)(Addr)))
#define HW_MEM_LONG(pHwAccess, Addr)   (*(volatile UINT32 *)(pHwAccess->MemBaseAddr + (UINT32)(Addr)))

#define TRANSLATE_ADDRESS_MEM(addr) ((addr) - pHwAccess->MemRegionAddr)
#define TRANSLATE_ADDRESS_REG(addr) ((addr) + pHwAccess->RegisterRegionAddr)

#if 1 /* 0 */
    #if (defined(HW_ACCESS_SDIO)|defined(HW_ACCESS_WSPI)) /* 1 */
        void HW_REG_LONG_WRITE(HwAccess_T_new *pHwAccess, UINT32 RegAddr, UINT32 BitVal);
        void HW_REG_LONG_READ(HwAccess_T_new *pHwAccess, UINT32 RegAddr, UINT32 *Val);
    #else /* 1 */
        #define HW_REG_SHORT_WRITE(pHwAccess, Addr, Data) ((*(volatile UINT16 *)(pHwAccess->RegBaseAddr + (UINT32)(Addr))) = (UINT16)(Data))
        #define HW_REG_SHORT_READ(pHwAccess, Addr, Data)  ((*(Data)) = (*(volatile UINT16 *)(pHwAccess->RegBaseAddr + (UINT32)(Addr))) )
        #ifdef NOT_SUPPORT_32_BIT_ACCESS_COMMAND /* for example: iPAQ model 38xx */ /* 2 */
            #define HW_REG_LONG_WRITE(pHwAccess, Addr, Data)  HW_REG_SHORT_WRITE(pHwAccess, Addr, Data); HW_REG_SHORT_WRITE(pHwAccess, Addr+2, ((UINT16)(Data>>16)))
            #define HW_REG_LONG_READ(pHwAccess, Addr, pData)   HW_REG_SHORT_READ(pHwAccess, Addr, pData); HW_REG_SHORT_READ(pHwAccess, Addr+2, ((UINT16 *)pData+1))
        #else /* 2 */
            #define HW_REG_LONG_WRITE(pHwAccess, Addr, Data)  ((*(volatile UINT32 *)(pHwAccess->RegBaseAddr + (UINT32)(Addr))) = (UINT32)(Data))
            #define HW_REG_LONG_READ(pHwAccess, Addr, Data)   ((*(Data)) = (*(volatile UINT32 *)(pHwAccess->RegBaseAddr + (UINT32)(Addr))) )
        #endif /* 2 */
    #endif /* 1 */
#else  /* 0 */
#endif /* 0 */



/************************************************************************
 * Functions
 ************************************************************************/

#if !defined(HW_ACCESS_SDIO) && !defined(HW_ACCESS_WSPI)
static void whal_hwAccess_DirectCopy_new(HwAccess_T_new *pHwAccess, UINT8* Dest, UINT8* Src, UINT32 Len);
#endif
#ifdef HW_ACCESS_SDIO
static void sdio_transaction_notify_read(struct SDIO_Request *req, int status);
static void sdio_transaction_notify_write(struct SDIO_Request *req, int status);
static void sdio_transaction_error(struct SDIO_Request *req, int stat);
#ifdef CONFIG_ASYNC_API
static void sdio_async_transaction_notify(struct SDIO_Request *req, int status);
static void sdio_async_transaction_error(struct SDIO_Request *req, int status);
#endif
#endif

/* 
** Read/Write interface
**----------------------------
**
** the memory space shell be divided to 2 Partions: Memory, and Registers.
** 1.   The memory Region will be set at init to point to the FW Ram, 
**      and after FW init complete, the Memory Region will be set to point the Packet Ram.
** 2.   Registry Region.
** 
** 
*/
 
 

/************************************************************************
 * new API
 ************************************************************************/



/****************************************************************************
 *                      whal_hwAccess_Create
 ****************************************************************************
 * DESCRIPTION: create the HwAccess module. allocate the module context and create the sublayers
 *
 * INPUTS:  hOs - handle to the OS module
 *
 * OUTPUT:  TI_HANDLE - the handle to the context that was created
 *
 * RETURNS: NULL = failure.
 *          otherwise = success
 ****************************************************************************/
TI_HANDLE   whal_hwAccess_Create(TI_HANDLE hOs)
{
    HwAccess_T_new *pHwAccess;
    int status = OK;
#ifdef HW_ACCESS_SDIO
    SDIO_ConfigParams configParams;
#endif  
    pHwAccess = os_memoryAlloc(hOs, sizeof(HwAccess_T_new));
    if (pHwAccess == NULL)
        return NULL;
    
    os_memoryZero(hOs, pHwAccess, sizeof(HwAccess_T_new));
    
    pHwAccess->hOs = hOs;

    pHwAccess->hProtect = os_protectCreate(pHwAccess->hOs);
    if (pHwAccess->hProtect == NULL)
    {
        whal_hwAccess_Destroy(pHwAccess);
        return NULL;
    }
    
#ifdef HW_ACCESS_SDIO

    pHwAccess->AsyncMode = FALSE;
    
    os_memoryZero(hOs, &configParams, sizeof(SDIO_ConfigParams));
    configParams.fnotify_read = sdio_transaction_notify_read; 
    configParams.fnotify_write = sdio_transaction_notify_write; 
    configParams.ferror = sdio_transaction_error;  
    configParams.fconfig_peripheral = SDIO_TNETWConfig; 
    configParams.fconvert = NULL;
    configParams.owner = pHwAccess;

    status = SDIO_Init(&configParams, &pHwAccess->hDriver); 
    
#elif HW_ACCESS_WSPI

    pHwAccess->AsyncMode = TRUE;

    pHwAccess->hDriver = WSPI_Open (pHwAccess->hOs);
    status = pHwAccess->hDriver == NULL;    
    
#else

    pHwAccess->AsyncMode = FALSE;
    
#endif

    if (status != 0) 
    {
        if (pHwAccess->hProtect)
            os_protectDestroy(pHwAccess->hOs, pHwAccess->hProtect);
        os_memoryFree(pHwAccess->hOs, pHwAccess, sizeof(HwAccess_T_new));
        return NULL;
    }
    
    return pHwAccess;
}

/****************************************************************************
 *                      whal_hwAccess_Destroy
 ****************************************************************************
 * DESCRIPTION: destroy the module. deallocate the cmodule context.
 *
 * INPUTS:  hHwAccess - handle to the module context
 *
 * OUTPUT:  none.
 *
 * RETURNS: one of the error codes (0 => OK)
 ****************************************************************************/
int whal_hwAccess_Destroy(TI_HANDLE hHwAccess)
{
    HwAccess_T_new *pHwAccess = (HwAccess_T_new*)hHwAccess;

    if (pHwAccess)
    {
#ifdef HW_ACCESS_SDIO

        SDIO_Stop(pHwAccess->hDriver, 0);
        SDIO_Shutdown(pHwAccess->hDriver);  

#elif HW_ACCESS_WSPI

        WSPI_Close(pHwAccess->hDriver);

#endif      
    
    
        if (pHwAccess->hProtect)
            os_protectDestroy(pHwAccess->hOs, pHwAccess->hProtect);
        os_memoryFree(pHwAccess->hOs, pHwAccess, sizeof(HwAccess_T_new));       
    }
    return OK;
}


/****************************************************************************
 *                      whal_hwAccess_Config
 ****************************************************************************
 * DESCRIPTION: config the module.
 *
 * INPUTS:  hHwAccess   - handle to the module context
 *          hReport     - handle to report module context that is used when we output debug messages
 *
 * OUTPUT:  none.
 *
 * RETURNS: one of the error codes (0 => OK)
 ****************************************************************************/
int whal_hwAccess_Config(TI_HANDLE hHwAccess, TI_HANDLE hReport,UINT32 RegBaseAddr, UINT32 MemBaseAddr, HwAccess_callback_t CBFunc,void* CBArg)
{
    HwAccess_T_new *pHwAccess = (HwAccess_T_new*) hHwAccess;
    int status = OK;

#ifdef HW_ACCESS_WSPI
    WSPIConfig_t wspi_config;        
#endif

    pHwAccess->hReport = hReport;

#ifdef GWSI_SPI_TEST
	/* For GWSI_API_TEST this parameter should be maximum allowed because we don't use setPartition */
	pHwAccess->workingPartUpperLimit = 0xFFFFFFFF;
#endif /* GWSI_API_TEST */
    /*
    Wait 200 usec for memory repair process to finish and device is ready.
    */
    os_StalluSec(pHwAccess->hOs, 200);
    
    pHwAccess->CB.CBFunc = CBFunc;
    pHwAccess->CB.CBArg = CBArg;

#ifdef HW_ACCESS_SDIO

    pHwAccess->RegisterRegionAddr = HW_ACCESS_DOWN_PART0_SIZE;
    pHwAccess->MemRegionAddr = HW_ACCESS_DOWN_PART0_ADDR;
    pHwAccess->uBusError = 0;

    status = SDIO_Start (pHwAccess->hDriver);

    status = (status == SDIO_SUCCESS) ? TNETWIF_COMPLETE : TNETWIF_ERROR;

#elif HW_ACCESS_WSPI

    wspi_config.isFixedAddress = FALSE;
    wspi_config.fixedBusyLength = HW_ACCESS_WSPI_FIXED_BUSY_LEN;
    wspi_config.mask = HW_ACCESS_WSPI_INIT_CMD_MASK;
        
    status = WSPI_Configure (pHwAccess->hDriver, 
                             pHwAccess->hReport, 
                             &wspi_config, 
                             (WSPI_CB_T*)&pHwAccess->CB);

	WSPI_SetErrLog(pHwAccess->hDriver, TNETWIF_printErrorLog);

    WSPI2TNETWIF (pHwAccess, status, 0x0);

#else /* HW_ACCESS_CARDBUS */   
    pHwAccess->RegBaseAddr = RegBaseAddr;
    pHwAccess->MemBaseAddr = MemBaseAddr;       
#endif

    return status;
}

/****************************************************************************
 *                      whal_hwAccess_ReConfig()
 ****************************************************************************
 * DESCRIPTION: 
 * 
 * INPUTS:  None
 * 
 * OUTPUT:  None
 * 
 * RETURNS: 
 ****************************************************************************/
int whal_hwAccess_ReConfig(TI_HANDLE hHwAccess)
{
    HwAccess_T_new* pHwAccess = (HwAccess_T_new*)hHwAccess;

#ifdef _WINDOWS
#else /* _WINDOWS */
    /* unbclock the access to the bus */
    pHwAccess->uBusError = 0;

#ifdef HW_ACCESS_SDIO
    SDIO_Stop (pHwAccess->hDriver, 0);
    SDIO_Start (pHwAccess->hDriver);
#elif HW_ACCESS_WSPI
    /* TODO*/
#endif
#endif /* _WINDOWS */
    return OK;
}


#ifdef USE_SYNC_API
/****************************************************************************
 *                      whal_hwAccess_WriteELP
 ****************************************************************************
 * DESCRIPTION: write data synchronously to the TNET ELP register (1byte)
 *
 * INPUTS:  pHwAccess - TI_HANDLE * - the HwAccess context
 *          data - UINT8 - the data to write
 *
 * OUTPUT:  none
 *
 * RETURNS: one of the error codes (0 => OK)
 ****************************************************************************/
int whal_hwAccess_WriteELP (TI_HANDLE hHwAccess, UINT32 data)
{
    HwAccess_T_new* pHwAccess = (HwAccess_T_new*)hHwAccess;
    int status = OK;

    os_profile (pHwAccess->hOs, 2, 0);

#ifdef HW_ACCESS_SDIO
    status = SDIO_TNETW_Set_ELP_Reg(pHwAccess->hDriver, HW_ACCESS_ELP_CTRL_REG_ADDR, data);
#elif HW_ACCESS_WSPI
    status = WSPI_WriteSync (pHwAccess->hDriver, HW_ACCESS_ELP_CTRL_REG_ADDR, (UINT8*)&data, HW_ACCESS_REGISTER_SIZE); 
#endif

    os_profile (pHwAccess->hOs, 3, 0);

    if (status != OK)
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
                          ("whal_hwAccess_WriteReg_ELP: Error in ELP reg write status=%d\n",
                          status));
        return NOK; 
    }

    return OK;
}
#endif /* USE_SYNC_API */

/****************************************************************************
 *                      whal_hwAccess_WriteELPAsync
 ****************************************************************************
 * DESCRIPTION: write data synchronously from the TNET using the defined access (WSPI/SDIO). 
 *              the length of data is always 4bytes cause this is the size of the TNET registers
 *              the function is passed a call-back function that will be called after the read request ends.
 *
 * INPUTS:  pHwAccess - TI_HANDLE * - the HwAccess context
 *          addr - UINT32 - the address offset inside the TNET
 *          data - UINT8* - a pointer to the buffer that holds the data to write
 *          bMore - indicate whether more txn on the bus are about to happen (FALSE only when setting
 *                  the HW to sleep).
 *
 * OUTPUT:  none                                                                
 *
 * RETURNS: one of the error codes (0 => OK)
 ****************************************************************************/
int whal_hwAccess_WriteELPAsync (TI_HANDLE hHwAccess, UINT32 data, BOOL bCb, BOOL bMore)
{
#if defined(HW_ACCESS_SDIO)
 
  #if defined(HW_ACCESS_SDIO_ASYNC_SUPPORT)

    #error "SDIO asynchronous mode is not supported"

    /* Not implemented yet */
    return OK;

  #else

     /* Just call to synchronous API */
    return (whal_hwAccess_WriteELP (hHwAccess, data) == OK) ? 
           TNETWIF_COMPLETE :
           TNETWIF_ERROR;


  #endif

#else /* HW_ACCESS_WSPI */

    HwAccess_T_new* pHwAccess = (HwAccess_T_new*)hHwAccess;
    WSPI_CB_T Cb = { NULL, NULL }, *pCb;
    int status;

    pCb = (bCb) ? ((WSPI_CB_T*)&pHwAccess->CB) : &Cb;

    os_profile (pHwAccess->hOs, 2, 0);

    /* since we are writing a register - no extra space is needed */
    status = WSPI_WriteAsync (pHwAccess->hDriver, 
                              HW_ACCESS_ELP_CTRL_REG_ADDR, 
                              (UINT8*)&data, 
                              HW_ACCESS_REGISTER_SIZE,
                              pCb,
                              bMore,
                              FALSE);   

    os_profile (pHwAccess->hOs, 3, 0);

    WSPI2TNETWIF (pHwAccess, status, HW_ACCESS_ELP_CTRL_REG_ADDR);

    return status;

#endif
}

/****************************************************************************
 *                      whal_hwAccess_ReadELPAsync
 ****************************************************************************
 * DESCRIPTION: Read the ELP register
 *
 * INPUTS:  pHwAccess - TI_HANDLE * - the HwAccess context
 *          addr - UINT32 - the address offset inside the TNET
 *          data - UINT8* - a pointer to the buffer to read data into
 *          bMore - indicate whether more txn on the bus are about to happen (FALSE only when setting
 *                  the HW to sleep).
 *
 * OUTPUT:  none                                                                
 *
 * RETURNS: one of the error codes (0 => OK)
 ****************************************************************************/
int whal_hwAccess_ReadELPAsync (TI_HANDLE hHwAccess, UINT8 *data, BOOL bCb, BOOL bMore)
{
    HwAccess_T_new* pHwAccess = (HwAccess_T_new*)hHwAccess;
    int status;      

#ifdef HW_ACCESS_SDIO

	#ifndef _WINDOWS
	status = SDIO_TNETW_Get_ELP_Reg(pHwAccess->hDriver, HW_ACCESS_ELP_CTRL_REG_ADDR, (UINT32*)data);
	#else
	#endif

	if (status != OK)
	{
		WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
			("whal_hwAccess_ReadELPAsync: Error in ELP reg raed status=%d\n",
			status));
		return TNETWIF_ERROR; 
	}
	return TNETWIF_COMPLETE;

#else /* HW_ACCESS_WSPI */
        
        os_profile (pHwAccess->hOs, 2, 0);
        
        /* In registers we don't save place */
        status = WSPI_ReadAsync (pHwAccess->hDriver, 
            HW_ACCESS_ELP_CTRL_REG_ADDR, 
            (UINT8*)data, 
            HW_ACCESS_REGISTER_SIZE,
            (WSPI_CB_T*)&pHwAccess->CB,
            TRUE,
            FALSE);   
        
        os_profile (pHwAccess->hOs, 3, 0);
        
        WSPI2TNETWIF (pHwAccess, status, HW_ACCESS_ELP_CTRL_REG_ADDR);
        
        return status;
        
#endif
}


#ifdef USE_SYNC_API
/****************************************************************************
 *                      whal_hwAccess_ReadMem_Align
 ****************************************************************************
 * DESCRIPTION: read data synchronously from the TNET using the defined access (WSPI/SDIO). 
 *              the length of data specified is rounded up so the length will be multiple of 4 (bytes)
 *
 * INPUTS:  hHwAccess - the handle of HwAccess module
 *          addr - UINT32 - the address offset inside the TNET
 *          len - int - the length of the data to read
 *
 * OUTPUT:  data - UINT8* - a pointer to the buffer to fill with the read data
 *
 * RETURNS: one of the error codes (0 => OK)
 ****************************************************************************/
int         whal_hwAccess_ReadMem_Align(TI_HANDLE hHwAccess, UINT32 addr, UINT8* data, UINT16 len)
{                                                          
    HwAccess_T_new* pHwAccess = (HwAccess_T_new*)hHwAccess;
    int status = OK;
#ifdef HW_ACCESS_SDIO
    struct SDIO_Request request;
#endif
    
    /* round up the length so it will be multiple of 4bytes */
    if(len&0x3)
        len = (len&0xFFFFFFFC)+4;

#ifdef HW_ACCESS_SDIO

    /* check address */
    if (((addr+len) > pHwAccess->workingPartUpperLimit) || (addr < pHwAccess->MemRegionAddr))
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwAccess_ReadMem_Align: Error in addr 0x%x (lower:0x%x,upper:0x%x)\n",
            addr, pHwAccess->MemRegionAddr, pHwAccess->workingPartUpperLimit));
        return ERROR_HW_ACCEESS_ADDR;
    }

    request.buffer = data; /* Pointer to the data buffer aligned address.  */
    request.buffer_len = len; /* Data buffer length in bytes */ 
    request.status = SDIO_Request_None;
    request.peripheral_addr = (SDIO_Address)TRANSLATE_ADDRESS_MEM(addr); /*SDIO peripheral address*/
    request.acquire_window = 0;  /*Time out value is not set*/
    request.block_len = 0;       /*Block length. Assigned by driver*/
    request.physical_buffer = 0; /*Physical address of data buffer is not set*/
    request.owner = (SDIO_Owner) pHwAccess;
    request.mode = MMC_DEV_BYTE_INCREMENTAL_MODE;
    request.access_flag = 1;

    os_profile (pHwAccess->hOs, 2, 0);

    status = SDIO_SyncRead(pHwAccess->hDriver, &request);

    os_profile (pHwAccess->hOs, 3, 0);

    
#elif HW_ACCESS_WSPI    

    /* check address */
    if (((addr+len) > pHwAccess->workingPartUpperLimit) || (addr < pHwAccess->MemRegionAddr))
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwAccess_ReadMem_Align: Error in addr 0x%x (lower:0x%x,upper:0x%x)\n",
            addr, pHwAccess->MemRegionAddr, pHwAccess->workingPartUpperLimit));
        return ERROR_HW_ACCEESS_ADDR;
    }

    os_profile (pHwAccess->hOs, 2, 0);

    status = WSPI_ReadSync(pHwAccess->hDriver,TRANSLATE_ADDRESS_MEM(addr),data,len);    

    os_profile (pHwAccess->hOs, 3, 0);

#else
    whal_hwAccess_DirectCopy_new(pHwAccess, data, (UINT8*)addr, len);
#endif

    if (status != OK)
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
                             ("whal_hwAccess_ReadMem_Align: Error in read, addr=0x%08x status=%d\n",
                             addr, status));

#ifdef HW_ACCESS_SDIO
    if (pHwAccess->uBusError)
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwAccess_ReadMem_Align: SDIO Error status=%d\n",
        request.status));
        if (pHwAccess->hwAccesserror_Cb) 
        {
            UINT8 failure_reason = HEALTH_REPORT_BUS_ERROR;
            pHwAccess->hwAccesserror_Cb(pHwAccess->hBackReference,(char*)&failure_reason,2);
        }
        else
            pHwAccess->uBusError = 0;
    }
#endif

    return status;
}

/****************************************************************************
 *                      whal_hwAccess_WriteMem_Align
 ****************************************************************************
 * DESCRIPTION: write data synchronously to the TNET using the defined access (WSPI/SDIO). 
 *              the length of data specified is rounded up so the length will be multiple of 4 (bytes)
 *
 * INPUTS:  hHwAccess - the handle of HwAccess module
 *          addr - UINT32 - the address offset inside the TNET
 *          data - UINT8* - a pointer to the buffer that holds the data to write
 *          len - int - the length of the data to read
 *
 * OUTPUT:  none
 *
 * RETURNS: one of the error codes (0 => OK)
 ****************************************************************************/
int         whal_hwAccess_WriteMem_Align(TI_HANDLE hHwAccess, UINT32 addr, UINT8* data, UINT16 len)
{
    HwAccess_T_new* pHwAccess = (HwAccess_T_new*)hHwAccess;
    int status = OK;
#ifdef HW_ACCESS_SDIO
    struct SDIO_Request request;
#endif
    
    /* round the length so it will be multiple of 4bytes */
    if(len&0x3)
        len = (len&0xFFFFFFFC)+4;
    
#ifdef HW_ACCESS_SDIO

    /* check address */
    if (((addr+len) > pHwAccess->workingPartUpperLimit) || (addr < pHwAccess->MemRegionAddr))
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwAccess_WriteMem_Align: Error in addr 0x%x (lower:0x%x,upper:0x%x)\n",
            addr, pHwAccess->MemRegionAddr, pHwAccess->workingPartUpperLimit));
        return ERROR_HW_ACCEESS_ADDR;
    }
    
    request.buffer = data; /* Pointer to the data buffer aligned address.  */
    request.buffer_len = len; /* Data buffer length in bytes */ 
    request.status = SDIO_Request_None;
    request.peripheral_addr = (SDIO_Address)TRANSLATE_ADDRESS_MEM(addr); /*SDIO peripheral address*/
    request.acquire_window = 0;  /*Time out value is not set*/
    request.block_len = 0;       /*Block length. Assigned by driver*/
    request.physical_buffer = 0; /*Physical address of data buffer is not set*/
    request.owner = (SDIO_Owner) pHwAccess;
    request.mode = MMC_DEV_BYTE_INCREMENTAL_MODE;
    request.access_flag = 0;

    os_profile (pHwAccess->hOs, 2, 0);

    status = SDIO_SyncWrite(pHwAccess->hDriver, &request);  

    os_profile (pHwAccess->hOs, 3, 0);
 

#elif HW_ACCESS_WSPI

    /* check address */
    if (((addr+len) > pHwAccess->workingPartUpperLimit) || (addr < pHwAccess->MemRegionAddr))
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwAccess_WriteMem_Align: Error in addr 0x%x (lower:0x%x,upper:0x%x)\n",
            addr, pHwAccess->MemRegionAddr, pHwAccess->workingPartUpperLimit));
        return ERROR_HW_ACCEESS_ADDR;
    }

    os_profile (pHwAccess->hOs, 2, 0);

    status = WSPI_WriteSync(pHwAccess->hDriver,TRANSLATE_ADDRESS_MEM(addr),data,len);

    os_profile (pHwAccess->hOs, 3, 0);
                
#else
    whal_hwAccess_DirectCopy_new(pHwAccess, (UINT8*)addr, data, len);
#endif

    if (status != OK)
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
                             ("whal_hwAccess_WriteMem_Align: Error in write, addr=0x%08x status=%d\n",
                             addr, status));

#ifdef HW_ACCESS_SDIO
    if (pHwAccess->uBusError) 
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwAccess_WriteMem_Align: SDIO Error in write status=%d\n",
        request.status));
        if (pHwAccess->hwAccesserror_Cb) 
        {
            UINT8 failure_reason = HEALTH_REPORT_BUS_ERROR;
            pHwAccess->hwAccesserror_Cb(pHwAccess->hBackReference,(char*)&failure_reason,2);
        }
        else 
            pHwAccess->uBusError = 0;
    }
#endif

    return status;
}
#endif /* USE_SYNC_API */

/****************************************************************************
 *                      whal_hwAccess_ReadMemAsync_Align
 ****************************************************************************
 * DESCRIPTION: read data asynchronously from the TNET using the defined access (WSPI/SDIO). 
 *              the length of data specified is rounded up so the length will be multiple of 4 (bytes)
 *              the function is passed a call-back function that will be called after the read request ends.
 *
 * INPUTS:  hHwAccess - the handle of HwAccess module
 *          addr - UINT32 - the address offset inside the TNET
 *          len - int - the length of the data to read
 *          CB - HWAccess_CB_T* - a pointer to a structure that holds the CB function and the passed arg.
 *
 * OUTPUT:  data - UINT8* - a pointer to the buffer to fill with the read data
 *
 * RETURNS: one of the error codes (0 => OK)
 ****************************************************************************/
int         whal_hwAccess_ReadMemAsync_Align(TI_HANDLE hHwAccess, UINT32 addr, UINT8* data, UINT16 len)
{       
    HwAccess_T_new* pHwAccess = (HwAccess_T_new*)hHwAccess;

    /* Round the length so it will be multiple of 4bytes */
    if ((len & 0x3) != 0)
        len = (len & ~3) + 4;

    /* Check address */
    if (addr + len > pHwAccess->workingPartUpperLimit || addr < pHwAccess->MemRegionAddr)
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwAccess_ReadMemAsync_Align: Error in addr 0x%x (lower:0x%x,upper:0x%x)\n",
            addr, pHwAccess->MemRegionAddr, pHwAccess->workingPartUpperLimit));
        return ERROR_HW_ACCEESS_ADDR;
    }

#if defined(HW_ACCESS_SDIO)

  #if defined(HW_ACCESS_SDIO_ASYNC_SUPPORT)
    {   
        struct SDIO_Request request;    
        int status = OK;

        request.buffer = data; /* Pointer to the data buffer aligned address.  */
        request.buffer_len = len; /* Data buffer length in bytes */ 
        request.status = SDIO_Request_None;
        request.peripheral_addr = (SDIO_Address)TRANSLATE_ADDRESS_MEM(addr); /*SDIO peripheral address*/
        request.acquire_window = 0;  /*Time out value is not set*/
        request.block_len = 0;       /*Block length. Assigned by driver*/
        request.physical_buffer = 0; /*Physical address of data buffer is not set*/
        request.owner = (SDIO_Owner) pHwAccess;
        request.mode = MMC_DEV_BYTE_INCREMENTAL_MODE;
        request.access_flag = 1;
        request.fnotify = sdio_async_transaction_notify; /* completion notification */
        request.ferror = sdio_async_transaction_error; /* error notification */

        os_profile (pHwAccess->hOs, 2, 0);

        status = SDIO_AsyncRead (pHwAccess->hDriver, &request);  

        os_profile (pHwAccess->hOs, 3, 0);

        if (status != OK)
        {
            WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
                             ("whal_hwAccess_ReadMemAsync_Align: Error in read async, addr=0x%08x status=%d\n",
                             addr, status));
            return TNETWIF_ERROR;
        }

        if (pHwAccess->uBusError)
        {
            WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
                              ("whal_hwAccess_ReadMemAsync_Align: SDIO Error status=%d\n",
                              request.status));
            if (pHwAccess->hwAccesserror_Cb) 
            {
                UINT16 failure_reason = HEALTH_REPORT_BUS_ERROR;
                pHwAccess->hwAccesserror_Cb(pHwAccess->hBackReference,&failure_reason,2);
            }
            else
                pHwAccess->uBusError = 0;
        }

        return TNETWIF_COMPLETE;
    }

  #else

    return TNETWIF_COMPLETE;
  
  #endif
            
#else /*HW_ACCESS_WSPI*/
    {
        int status;
        
        os_profile (pHwAccess->hOs, 2, 0);

        status = WSPI_ReadAsync (pHwAccess->hDriver,
                                 TRANSLATE_ADDRESS_MEM(addr),
                                 data,
                                 len,
                                 (WSPI_CB_T*)&pHwAccess->CB,
                                 TRUE,
                                 0);

        os_profile (pHwAccess->hOs, 3, 0);

        WSPI2TNETWIF (pHwAccess, status, addr);

        return status; 
    }
  
#endif
}

/****************************************************************************                       
 *                      whal_hwAccess_WriteAsync_Align
 ****************************************************************************
 * DESCRIPTION: write data synchronously from the TNET using the defined access (WSPI/SDIO). 
 *              the length of data specified is rounded up so the length will be multiple of 4 (bytes)
 *              the function is passed a call-back function that will be called after the read request ends.
 *
 * INPUTS:  pHwAccess - HwAccess_T* - the HwAccess context
 *          addr - UINT32 - the address offset inside the TNET
 *          data - UINT8* - a pointer to the buffer that holds the data to write
 *          Len - int - the length of the data to read
 *          CB - HWAccess_CB_T* - a pointer to a structure that holds the CB function and the passed arg.
 *
 * OUTPUT:  none
 *
 * RETURNS: one of the error codes (0 => OK)
 ****************************************************************************/
 int         whal_hwAccess_WriteMemAsync_Align(TI_HANDLE hHwAccess, UINT32 addr, UINT8* data, UINT16 len)
{           
    HwAccess_T_new* pHwAccess = (HwAccess_T_new*)hHwAccess;

    /* Round the length so it will be multiple of 4bytes */
    if ((len & 0x3) != 0)
        len = (len & ~3) + 4;

    /* Check address */
    if (addr + len > pHwAccess->workingPartUpperLimit || addr < pHwAccess->MemRegionAddr)
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
                          ("whal_hwAccess_WriteMemAsync_Align: Error in addr 0x%x (lower:0x%x,upper:0x%x)\n",
                          addr, pHwAccess->MemRegionAddr, pHwAccess->workingPartUpperLimit));
        return ERROR_HW_ACCEESS_ADDR;
    }

#if defined(HW_ACCESS_SDIO)

  #if defined(HW_ACCESS_SDIO_ASYNC_SUPPORT)
    {    
        struct SDIO_Request request;    
        int status = OK;

        request.buffer = data; /*Pointer to the data buffer aligned address*/
        request.buffer_len = len; /*Data buffer length in bytes*/ 
        request.status = SDIO_Request_None;
        request.peripheral_addr = (SDIO_Address)TRANSLATE_ADDRESS_MEM(addr); /*SDIO peripheral address*/
        request.acquire_window = 0;  /* Time out value is not set */
        request.block_len = 0;       /* Block length. Assigned by driver */
        request.physical_buffer = 0; /* Physical address of data buffer is not set */
        request.owner = (SDIO_Owner) pHwAccess;
        request.mode = MMC_DEV_BYTE_INCREMENTAL_MODE;
        request.access_flag = 0;
        request.fnotify = sdio_async_transaction_notify; /* completion notification */
        request.ferror = sdio_async_transaction_error; /* error notification */

        os_profile (pHwAccess->hOs, 2, 0);

        status = SDIO_AsyncWrite (pHwAccess->hDriver, &request);    

        os_profile (pHwAccess->hOs, 3, 0);

        if (status != OK)
        {
            WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
                              ("whal_hwAccess_WriteMemAsync_Align: Error in write async, addr=0x%08x status=%d\n",
                              addr, status));

            return TNETWIF_ERROR; 
        }

        return TNETWIF_COMPLETE;
    }

  #else

    return TNETWIF_COMPLETE;

  #endif
            
#else /*HW_ACCESS_WSPI*/
    {
        int status;

        os_profile (pHwAccess->hOs, 2, 0);

        status = WSPI_WriteAsync (pHwAccess->hDriver,
                                  TRANSLATE_ADDRESS_MEM(addr),
                                  data,
                                  len,
                                  (WSPI_CB_T*)&pHwAccess->CB,
                                  TRUE,
                                  FALSE);   

        os_profile (pHwAccess->hOs, 3, 0);
            
        WSPI2TNETWIF (pHwAccess, status, addr);

        return status;
    }
#endif
}


#ifdef USE_SYNC_API
/****************************************************************************
 *                      whal_hwAccess_ReadMem
 ****************************************************************************
 * DESCRIPTION: read data synchronously from the TNET using the defined access (WSPI/SDIO). 
 *              the length of data is checked and the remnant (length%4) is completed with read-modify
 *
 * INPUTS:  pHwAccess - HwAccess_T* - the HwAccess context
 *          AddrOffset - UINT32 - the address offset inside the TNET
 *          Len - int - the length of the data to read
 *
 * OUTPUT:  data - UINT8* - a pointer to the buffer to fill with the read data
 *
 * RETURNS: one of the error codes (0 => OK)
 ****************************************************************************/
int         whal_hwAccess_ReadMem(TI_HANDLE hHwAccess, UINT32 addr, UINT8* data, UINT16 len)
{
    int status = OK;    
    HwAccess_T_new* pHwAccess = (HwAccess_T_new*)hHwAccess;

#ifdef HW_ACCESS_SDIO
    struct SDIO_Request request;
#elif HW_ACCESS_WSPI
    int reminder = len%HW_ACCESS_WSPI_ALIGNED_SIZE;
    int tempLen = len - reminder;
    UINT32 mask = 0;
    status = whal_hwAccess_ReadMemAsync(hHwAccess, addr, data, len);
    if (status == TNETWIF_COMPLETE)
    {
        status = OK;
    }
    return status;
#endif

    /* access is blocked */
    if (pHwAccess->uBusError) 
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_CTRL_MODULE_LOG,
                              ("Bus is blocked \n"));
        return ERROR_HW_ACCEESS_ADDR;
    }

#ifdef __HWACCESS_DEBUG__
    /* check address alignment */
    if(addr & 0x3)
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwAccess_ReadMem: addr is not aligned 0x%x\n",
            addr));     
    }
    
#endif
        
#ifdef HW_ACCESS_SDIO

    /* check address */
    if (((addr+len) > pHwAccess->workingPartUpperLimit) || (addr < pHwAccess->MemRegionAddr))
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwAccess_ReadMem: Error in addr 0x%x (lower:0x%x,upper:0x%x)\n",
            addr, pHwAccess->MemRegionAddr, pHwAccess->workingPartUpperLimit));
        return ERROR_HW_ACCEESS_ADDR;
    }

    request.buffer = data; /* Pointer to the data buffer aligned address.  */
    request.buffer_len = len; /* Data buffer length in bytes */ 
    request.status = SDIO_Request_None;
    request.peripheral_addr = (SDIO_Address)TRANSLATE_ADDRESS_MEM(addr); /*SDIO peripheral address*/
    request.acquire_window = 0;  /*Time out value is not set*/
    request.block_len = 0;       /*Block length. Assigned by driver*/
    request.physical_buffer = 0; /*Physical address of data buffer is not set*/
    request.owner = (SDIO_Owner) pHwAccess;
    request.mode = MMC_DEV_BYTE_INCREMENTAL_MODE;
    request.access_flag = 1;

    os_profile (pHwAccess->hOs, 2, 0);

    status = SDIO_SyncRead(pHwAccess->hDriver, &request);

    os_profile (pHwAccess->hOs, 3, 0);

    if (status != OK)
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
                             ("whal_hwAccess_ReadMem: SDIO Error in read\n"));
        return status;
    }

#elif HW_ACCESS_WSPI 

    /* check address */
    if (((addr+len) > pHwAccess->workingPartUpperLimit) || (addr < pHwAccess->MemRegionAddr))
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwAccess_ReadMem: Error in addr 0x%x (lower:0x%x,upper:0x%x)\n",
            addr, pHwAccess->MemRegionAddr, pHwAccess->workingPartUpperLimit));
        return ERROR_HW_ACCEESS_ADDR;
    }
    
    os_profile (pHwAccess->hOs, 2, 0);

    /* read the aligned size */
    status = WSPI_ReadSync(pHwAccess->hDriver,TRANSLATE_ADDRESS_MEM(addr),data,tempLen);

    os_profile (pHwAccess->hOs, 3, 0);

    if (status != OK)
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
                             ("whal_hwAccess_ReadMem: WSPI Error in read\n"));
        return status;
    }
    
    /* read the non aligned reminder */
    if(reminder)
    {
        UINT32 tempVal = 0;     
        
        os_profile (pHwAccess->hOs, 2, 0);

        /* read the extra data*/
        status |= WSPI_ReadSync(pHwAccess->hDriver,TRANSLATE_ADDRESS_MEM(addr+tempLen),(UINT8*)&tempVal,HW_ACCESS_WSPI_ALIGNED_SIZE);

        os_profile (pHwAccess->hOs, 3, 0);

        if (status != OK)
        {
            WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
                             ("whal_hwAccess_ReadMem: WSPI Error in read\n"));
            return status;
        }
        
        /* extract the relevant data */
        switch(reminder) 
        {
            case 1:
                mask = HW_ACCESS_1_BYTE_REMINDE_MASK;
                break;
            case 2:
                mask = HW_ACCESS_2_BYTE_REMINDE_MASK;
                break;
            case 3:
                mask = HW_ACCESS_3_BYTE_REMINDE_MASK;
                break;
        }                           
        *(UINT32*)&data[tempLen] &= ~mask;
        *(UINT32*)&data[tempLen] |= tempVal & mask;
    }

#else
    whal_hwAccess_DirectCopy_new(pHwAccess, data, (UINT8*)(pHwAccess->MemBaseAddr+addr), len);
#endif

#ifdef HW_ACCESS_SDIO
    if (pHwAccess->uBusError)
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwAccess_ReadMem: SDIO Error status=%d\n",
        request.status));
        if (pHwAccess->hwAccesserror_Cb) 
        {
            UINT8 failure_reason = HEALTH_REPORT_BUS_ERROR;
            pHwAccess->hwAccesserror_Cb(pHwAccess->hBackReference,(char*)&failure_reason,2);
        }
        else
            pHwAccess->uBusError = 0;
    }
#endif

    return OK;
}



/****************************************************************************
 *                      whal_hwAccess_WriteMem
 ****************************************************************************
 * DESCRIPTION: write data synchronously to the TNET using the defined access (WSPI/SDIO). 
 *              the length of data is checked and the remnant (length%4) is completed with read-modify-write
 *
 * INPUTS:  pHwAccess - TI_HANDLE* - the HwAccess context
 *          addr - UINT32 - the address offset inside the TNET
 *          data - UINT8* - a pointer to the buffer that holds the data to write
 *          Len - int - the length of the data to read
 *
 * OUTPUT:  none
 *
 * RETURNS: one of the error codes (0 => OK)
 ****************************************************************************/
int         whal_hwAccess_WriteMem(TI_HANDLE hHwAccess, UINT32 addr, UINT8* data, UINT16 len)
{                                                         
    int status = OK;    
    HwAccess_T_new* pHwAccess = (HwAccess_T_new*)hHwAccess;
    
#ifdef HW_ACCESS_SDIO
    struct SDIO_Request request;
#elif HW_ACCESS_WSPI    
    int reminder = len % HW_ACCESS_WSPI_ALIGNED_SIZE;
    int tempLen = len - reminder;
    UINT32 mask = 0;
    status = whal_hwAccess_WriteMemAsync(hHwAccess, addr, data,  len);
    if (status == TNETWIF_COMPLETE)
    {
        status = OK;
    }
    return status;
#endif

    /* access is blocked */
    if (pHwAccess->uBusError) 
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_CTRL_MODULE_LOG,
                              ("Bus is blocked \n"));
        return ERROR_HW_ACCEESS_ADDR;
    }

#ifdef __HWACCESS_DEBUG__
    /* check address alignment */
    if(addr & 0x3)
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwAccess_WriteMem: addr is not aligned 0x%x\n",
            addr));
#endif
    
#ifdef HW_ACCESS_SDIO                                       

    /* check address */
    if (((addr+len) > pHwAccess->workingPartUpperLimit) || (addr < pHwAccess->MemRegionAddr))
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwAccess_WriteMem: Error in addr 0x%x (lower:0x%x,upper:0x%x)\n",
            addr, pHwAccess->MemRegionAddr, pHwAccess->workingPartUpperLimit));
        return ERROR_HW_ACCEESS_ADDR;
    }

    request.buffer = data; /* Pointer to the data buffer aligned address.  */
    request.buffer_len = len; /* Data buffer length in bytes */ 
    request.status = SDIO_Request_None;
    request.peripheral_addr = (SDIO_Address)TRANSLATE_ADDRESS_MEM(addr); /*SDIO peripheral address*/
    request.acquire_window = 0;  /*Time out value is not set*/
    request.block_len = 0;       /*Block length. Assigned by driver*/
    request.physical_buffer = 0; /*Physical address of data buffer is not set*/
    request.owner = (SDIO_Owner) pHwAccess;
    request.mode = MMC_DEV_BYTE_INCREMENTAL_MODE;
    request.access_flag = 0;

    os_profile (pHwAccess->hOs, 2, 0);

    status = SDIO_SyncWrite(pHwAccess->hDriver, &request);  

    os_profile (pHwAccess->hOs, 3, 0);

    if (status != OK)
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
                             ("whal_hwAccess_WriteMem: SDIO Error in write (%d)\n", status));
        return status;
    }

#elif HW_ACCESS_WSPI

    /* check address */
    if (((addr+len) > pHwAccess->workingPartUpperLimit) || (addr < pHwAccess->MemRegionAddr))
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwAccess_WriteMem: Error in addr 0x%x (lower:0x%x,upper:0x%x)\n",
            addr, pHwAccess->MemRegionAddr, pHwAccess->workingPartUpperLimit));
        return ERROR_HW_ACCEESS_ADDR;
    }

    os_profile (pHwAccess->hOs, 2, 0);

    /* write the aligned size */
    status = WSPI_WriteSync(pHwAccess->hDriver,TRANSLATE_ADDRESS_MEM(addr),data,tempLen);

    os_profile (pHwAccess->hOs, 3, 0);


    if (status != OK)
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
                             ("whal_hwAccess_WriteMem: WSPI Error in write\n"));
        return status;
    }
    
    /* read the non aligned reminder */
    if(reminder)
    {
        UINT32 tempVal;     
        
        os_profile (pHwAccess->hOs, 2, 0);

        /* read the extra data*/
        status |= WSPI_ReadSync(pHwAccess->hDriver,TRANSLATE_ADDRESS_MEM(addr + tempLen),(UINT8*)&tempVal,HW_ACCESS_WSPI_ALIGNED_SIZE);

        os_profile (pHwAccess->hOs, 3, 0);

        if (status != OK)
        {
            WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
                             ("whal_hwAccess_WriteMem: WSPI Error in read\n"));
            return status;
        }
        
        /* extract the relevant data */
        switch(reminder) 
        {
            case 1:
                mask = HW_ACCESS_1_BYTE_REMINDE_MASK;
                break;
            case 2:
                mask = HW_ACCESS_2_BYTE_REMINDE_MASK;
                break;
            case 3:
                mask = HW_ACCESS_3_BYTE_REMINDE_MASK;
                break;
        }

        tempVal &= ~mask;
        tempVal |= *(UINT32*)&data[tempLen] & mask;

        os_profile (pHwAccess->hOs, 2, 0);

        /* write the modified extra data */
        status = WSPI_WriteSync(pHwAccess->hDriver,TRANSLATE_ADDRESS_MEM(addr + tempLen),(UINT8*)&tempVal,HW_ACCESS_WSPI_ALIGNED_SIZE);     

        os_profile (pHwAccess->hOs, 3, 0);

        if (status != OK)
        {
            WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
                             ("whal_hwAccess_WriteMem: WSPI Error in write\n"));
            return status;
        }                                       
    }   
            
#else
    whal_hwAccess_DirectCopy_new(pHwAccess, (UINT8*)(pHwAccess->MemBaseAddr+addr), data, len);
#endif

#ifdef HW_ACCESS_SDIO
    if (pHwAccess->uBusError) 
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwAccess_WriteMem: SDIO Error in write status=%d\n",
        request.status));
        if (pHwAccess->hwAccesserror_Cb) 
        {
            UINT8 failure_reason = HEALTH_REPORT_BUS_ERROR;
            pHwAccess->hwAccesserror_Cb(pHwAccess->hBackReference,(char*)&failure_reason,2);
        }
        else 
            pHwAccess->uBusError = 0;

    }
#endif
    return OK;
}
#endif /* USE_SYNC_API */ 


/****************************************************************************
 *                      whal_hwAccess_WriteMemAsync
 ****************************************************************************
 * DESCRIPTION: write data synchronously from the TNET using the defined access (WSPI/SDIO). 
 *              the length of data is checked and the remnant (length%4) is completed with read-modify-write
 *              the function is passed a call-back function that will be called after the read request ends.
 *
 * INPUTS:  pHwAccess - TI_HANDLE* - the HwAccess context
 *          addr - UINT32 - the address offset inside the TNET
 *          data - UINT8* - a pointer to the buffer that holds the data to write
 *          Len - int - the length of the data to read
 *          CB - HWAccess_CB_T* - a pointer to a structure that holds the CB function and the passed arg.
 *
 * OUTPUT:  none
 *
 * RETURNS: one of the error codes (0 => OK)
 ****************************************************************************/
TI_STATUS           whal_hwAccess_WriteMemAsync(TI_HANDLE hHwAccess, UINT32 addr, UINT8* data, UINT16 len)
{   
#if defined(HW_ACCESS_SDIO) && !defined(HW_ACCESS_SDIO_ASYNC_SUPPORT)

    /* Just call to synchronous API - add the offset that was added to the WSPI bus - only if it was reserved*/
    return (whal_hwAccess_WriteMem (hHwAccess, addr, data + TNETWIF_WRITE_OFFSET_BYTES, len) == OK) 
               ? TNETWIF_COMPLETE
               : TNETWIF_ERROR;   

#else

    HwAccess_T_new* pHwAccess = (HwAccess_T_new*)hHwAccess;
    int status = OK;

    /* Access is blocked */
    if (pHwAccess->uBusError) 
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_CTRL_MODULE_LOG,
                          ("Bus is blocked \n"));
        return (TI_STATUS)ERROR_HW_ACCEESS_ADDR;
    }

    /* Check length */
    if ((len & 0x3) != 0)
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
                          ("whal_hwAccess_WriteMemAsync: Error in length = %d\n",
                          len));
        return (TI_STATUS)ERROR_HW_ACCEESS_LEN;
    }

    /* Check address */
    if (addr + len > pHwAccess->workingPartUpperLimit || 
        addr < pHwAccess->MemRegionAddr)
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
                          ("whal_hwAccess_WriteMemAsync: Error in addr 0x%x (lower:0x%x,upper:0x%x)\n",
                          addr, pHwAccess->MemRegionAddr, pHwAccess->workingPartUpperLimit));
        return (TI_STATUS)ERROR_HW_ACCEESS_ADDR;
    }

#if defined(HW_ACCESS_SDIO)
    {
        struct SDIO_Request request;    

        request.buffer = data + TNETWIF_WRITE_OFFSET_BYTES; /*Pointer to the data buffer aligned address*/
        request.buffer_len = len; /*Data buffer length in bytes*/ 
        request.status = SDIO_Request_None;
        request.peripheral_addr = (SDIO_Address)TRANSLATE_ADDRESS_MEM(addr); /*SDIO peripheral address*/
        request.acquire_window = 0;  /* Time out value is not set */
        request.block_len = 0;       /* Block length. Assigned by driver */
        request.physical_buffer = 0; /* Physical address of data buffer is not set */
        request.owner = (SDIO_Owner) pHwAccess;
        request.mode = MMC_DEV_BYTE_INCREMENTAL_MODE;
        request.access_flag = 0;
        request.fnotify = sdio_async_transaction_notify; /* completion notification */
        request.ferror = sdio_async_transaction_error; /* error notification */

        os_profile (pHwAccess->hOs, 2, 0);

        status = SDIO_AsyncWrite (pHwAccess->hDriver, &request); 

        os_profile (pHwAccess->hOs, 3, 0);

        if (status != OK)
        {
            WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
                              ("whal_hwAccess_WriteMemAsync: Error in write async, addr=0x%08x status=%d\n",
                              addr, status));
            return TNETWIF_ERROR;
        }

        if (pHwAccess->uBusError) 
        {
            WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwAccess_WriteMemAsync: SDIO Error in write status=%d\n",
            request.status));
            if (pHwAccess->hwAccesserror_Cb) 
            {
                UINT8 failure_reason = HEALTH_REPORT_BUS_ERROR;
                pHwAccess->hwAccesserror_Cb(pHwAccess->hBackReference,&failure_reason,2);
            }
            else 
                pHwAccess->uBusError = 0;
        
        }

        return TNETWIF_COMPLETE;
    }
        
#elif defined(HW_ACCESS_WSPI)

    os_profile (pHwAccess->hOs, 2, 0);

    status = WSPI_WriteAsync (pHwAccess->hDriver,
                              TRANSLATE_ADDRESS_MEM(addr),
                              data,
                              len,
                              (WSPI_CB_T*)&pHwAccess->CB,
                              TRUE, 
                              TRUE); /* place is always reserved in write mem operation */   

    os_profile (pHwAccess->hOs, 3, 0);
                
    WSPI2TNETWIF (pHwAccess, status, addr);

    return (TI_STATUS)status;

#else

    /* By now since the CB is a SYNCH interface then call the SYNCH interface */
    whal_hwAccess_DirectCopy_new(pHwAccess, (UINT8*)(pHwAccess->MemBaseAddr+addr), data, len);

    return OK;

#endif

#endif
}


/****************************************************************************
 *                      whal_hwAccess_ReadMemAsync
 ****************************************************************************
 * DESCRIPTION: read data asynchronously from the TNET using the defined access (WSPI/SDIO). 
 *              the length of data is checked and the remnant (length%4) is completed with read-modify
 *              the function is passed a call-back function that will be called after the read request ends.
 *
 * INPUTS:  pHwAccess - TI_HANDLE* - the HwAccess context
 *          addr - UINT32 - the address offset inside the TNET
 *          Len - int - the length of the data to read
 *          CB - HWAccess_CB_T* - a pointer to a structure that holds the CB function and the passed arg.
 *
 * OUTPUT:  data - UINT8* - a pointer to the buffer to fill with the read data
 *
 * RETURNS: one of the error codes (0 => OK)
 ****************************************************************************/
int whal_hwAccess_ReadMemAsync (TI_HANDLE hHwAccess, UINT32 addr, UINT8* data, UINT16 len)
{
#if defined(HW_ACCESS_SDIO) && !defined(HW_ACCESS_SDIO_ASYNC_SUPPORT)  
  
    /* Just call to synchronous API - add the offset that was added to the WSPI bus */
    return (whal_hwAccess_ReadMem (hHwAccess, addr, data  + TNETWIF_READ_OFFSET_BYTES, len) == OK)
               ? TNETWIF_COMPLETE
               : TNETWIF_ERROR;

#else

    HwAccess_T_new* pHwAccess = (HwAccess_T_new*)hHwAccess;
    int status = OK;

    /* Access is blocked */
    if (pHwAccess->uBusError) 
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_CTRL_MODULE_LOG,
                              ("Bus is blocked \n"));
        return ERROR_HW_ACCEESS_ADDR;
    }

    /* Check length */
    if ((len & 0x3) != 0)
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwAccess_ReadMemAsync: Error in length = %d\n",
            len));
        return ERROR_HW_ACCEESS_LEN;
    }
    
    /* Check address */
    if (addr + len > pHwAccess->workingPartUpperLimit || 
        addr < pHwAccess->MemRegionAddr)
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwAccess_ReadMemAsync: Error in addr 0x%x (lower:0x%x,upper:0x%x)\n",
            addr, pHwAccess->MemRegionAddr, pHwAccess->workingPartUpperLimit));
        return ERROR_HW_ACCEESS_ADDR;
    }

#if defined(HW_ACCESS_SDIO)
    {
        struct SDIO_Request request;    

        request.buffer = data + TNETWIF_READ_OFFSET_BYTES; /*Pointer to the data buffer aligned address*/
        request.buffer_len = len; /*Data buffer length in bytes*/ 
        request.status = SDIO_Request_None;
        request.peripheral_addr = (SDIO_Address)TRANSLATE_ADDRESS_MEM(addr); /*SDIO peripheral address*/
        request.acquire_window = 0;  /* Time out value is not set */
        request.block_len = 0;       /* Block length. Assigned by driver */
        request.physical_buffer = 0; /* Physical address of data buffer is not set */
        request.owner = (SDIO_Owner) pHwAccess;
        request.mode = MMC_DEV_BYTE_INCREMENTAL_MODE;
        request.access_flag = 0;
        request.fnotify = sdio_async_transaction_notify; /* completion notification */
        request.ferror = sdio_async_transaction_error; /* error notification */

        os_profile (pHwAccess->hOs, 2, 0);

        status = SDIO_AsyncRead (pHwAccess->hDriver, &request);  

        os_profile (pHwAccess->hOs, 3, 0);

        if (status != OK)
        {
            WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
                              ("whal_hwAccess_ReadMemAsync: Error in write async, addr=0x%08x status=%d\n",
                              addr, status));
            return TNETWIF_ERROR;
        }

        if (pHwAccess->uBusError)
        {
            WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
                              ("whal_hwAccess_ReadMemAsync: SDIO Error in write SrcOffset=0x%08x status=%d\n",
                              SrcOffset, request.status));
            if (pHwAccess->hwAccesserror_Cb) 
            {
                UINT16 failure_reason = HEALTH_REPORT_BUS_ERROR;
                pHwAccess->hwAccesserror_Cb(pHwAccess->hBackReference,&failure_reason,2);
            }
            else
                pHwAccess->uBusError = 0;
        }

        return TNETWIF_COMPLETE;
    }
            
#elif HW_ACCESS_WSPI

    os_profile (pHwAccess->hOs, 2, 0);

    status = WSPI_ReadAsync (pHwAccess->hDriver, 
                             TRANSLATE_ADDRESS_MEM(addr), 
                             data, 
                             len, 
                             (WSPI_CB_T*)&pHwAccess->CB,
                             TRUE,
                             TRUE); /* place is always reserved in readMem */    
            
    os_profile (pHwAccess->hOs, 3, 0);

    WSPI2TNETWIF (pHwAccess, status, addr);

    return status;

#endif

#endif
}


#ifdef USE_SYNC_API
/****************************************************************************
 *                      whal_hwAccess_ReadReg
 ****************************************************************************
 * DESCRIPTION: read data synchronously from the TNET using the defined access (WSPI/SDIO). 
 *              the length of data is always 4bytes cause this is the size of the TNET registers
 *
 * INPUTS:  pHwAccess - HwAccess_T_new* - the HwAccess context
 *          addr - UINT32 - the address offset inside the TNET
 *          Len - int - the length of the data to read
 *
 * OUTPUT:  data - UINT8* - a pointer to the buffer to fill with the read data
 *
 * RETURNS: one of the error codes (0 => OK)
 ****************************************************************************/
int         whal_hwAccess_ReadReg(TI_HANDLE hHwAccess, UINT32 addr, UINT32* data)
{
    HwAccess_T_new* pHwAccess = (HwAccess_T_new*)hHwAccess;
    int status = OK;

#ifdef HW_ACCESS_SDIO
    struct SDIO_Request request;

    /* check address */
    if (addr > pHwAccess->registerPartUpperLimit)
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwAccess_ReadReg: Error in addr 0x%x (lower:0x%x,upper:0x%x)\n",
            addr, pHwAccess->RegisterRegionAddr, pHwAccess->registerPartUpperLimit));
        return ERROR_HW_ACCEESS_ADDR;
    }

    request.buffer = (UINT8*)data; /*Pointer to the data buffer aligned address.*/
    request.buffer_len = HW_ACCESS_REGISTER_SIZE; /*Data buffer length in bytes*/ 
    request.status = SDIO_Request_None;
    request.peripheral_addr = TRANSLATE_ADDRESS_REG(addr); 
    request.acquire_window = 0;  /* Time out value is not set */
    request.block_len = 0;       /* Block length. Assigned by driver */
    request.owner = (SDIO_Owner) pHwAccess;
    request.physical_buffer = 0; /* Physical address of data buffer is not set */
    request.mode = MMC_DEV_BYTE_INCREMENTAL_MODE;
    request.access_flag = 1;

    os_profile (pHwAccess->hOs, 2, 0);

    status = SDIO_SyncRead(pHwAccess->hDriver, &request);

    os_profile (pHwAccess->hOs, 3, 0);


#elif HW_ACCESS_WSPI

    status = whal_hwAccess_ReadRegAsync(hHwAccess, addr, data);
    if (status == TNETWIF_COMPLETE)
    {
        status = OK;
    }
    return status;
    /* check address */
    if ((addr > pHwAccess->registerPartUpperLimit) || (addr < 0))
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwAccess_ReadReg: Error in addr 0x%x (lower:0x%x,upper:0x%x)\n",
            addr, pHwAccess->RegisterRegionAddr, pHwAccess->registerPartUpperLimit));
        return ERROR_HW_ACCEESS_ADDR;
    }

    os_profile (pHwAccess->hOs, 2, 0);

    status = WSPI_ReadSync(pHwAccess->hDriver, TRANSLATE_ADDRESS_REG(addr), (UINT8*)data, HW_ACCESS_REGISTER_SIZE);

    os_profile (pHwAccess->hOs, 3, 0);

#else

    HW_REG_LONG_READ(pHwAccess,addr,data);

#endif

    if (status != OK)
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwAccess_ReadReg: Error in reg read RegAddr=0x%08x status=%d\n",
        addr, status));
        
    }

#ifdef HW_ACCESS_SDIO           
    if (pHwAccess->uBusError)
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwAccess_ReadReg: SDIO Error status=%d\n",
        request.status));
        if (pHwAccess->hwAccesserror_Cb) 
        {
            UINT8 failure_reason = HEALTH_REPORT_BUS_ERROR;
            pHwAccess->hwAccesserror_Cb(pHwAccess->hBackReference,(char *)&failure_reason,2);
        }
        else
            pHwAccess->uBusError = 0;
    }
#endif
    return status;
}
#endif /* USE_SYNC_API */


/****************************************************************************
 *                      whal_hwAccess_Stop()
 ****************************************************************************
 * DESCRIPTION: stops the bus driver
 * 
 * INPUTS:  None
 * 
 * OUTPUT:  None
 * 
 * RETURNS: 
 ****************************************************************************/
int whal_hwAccess_Stop(TI_HANDLE hHwAccess)
{
    HwAccess_T_new* pHwAccess = (HwAccess_T_new*)hHwAccess;

    /* unbclock the access to the bus */
    pHwAccess->uBusError = 0;

#ifdef HW_ACCESS_SDIO
    SDIO_Stop(pHwAccess->hDriver,0);
#elif defined (HW_ACCESS_WSPI)
    /* TODO*/
#endif
    return OK;
}


#ifdef USE_SYNC_API

/****************************************************************************
 *                      whal_hwAccess_WriteReg
 ****************************************************************************
 * DESCRIPTION: write data synchronously to the TNET using the defined access (WSPI/SDIO). 
 *              the length of data is always 4bytes cause this is the size of the TNET registers
 *
 * INPUTS:  pHwAccess - TI_HANDLE * - the HwAccess context
 *          addr - UINT32 - the address offset inside the TNET
 *          data - UINT8* - a pointer to the buffer that holds the data to write
 *          Len - int - the length of the data to read
 *
 * OUTPUT:  none
 *
 * RETURNS: one of the error codes (0 => OK)
 ****************************************************************************/
int         whal_hwAccess_WriteReg(TI_HANDLE hHwAccess, UINT32 addr, UINT32 data)
{
    HwAccess_T_new* pHwAccess = (HwAccess_T_new*)hHwAccess;
    int status = OK;

#ifdef HW_ACCESS_SDIO
    struct SDIO_Request request;

    /* check address */
    if (addr > pHwAccess->registerPartUpperLimit)
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwAccess_WriteReg: Error in addr 0x%x (lower:0x%x,upper:0x%x)\n",
            addr, pHwAccess->RegisterRegionAddr, pHwAccess->registerPartUpperLimit));
        return ERROR_HW_ACCEESS_ADDR;
    }

    request.buffer = (UINT8*)&data; /*Pointer to the data buffer aligned address.*/
    request.buffer_len = HW_ACCESS_REGISTER_SIZE; /*Data buffer length in bytes*/ 
    request.status = SDIO_Request_None;
    request.peripheral_addr = TRANSLATE_ADDRESS_REG(addr);
    request.acquire_window = 0;  /* Time out value is not set */
    request.block_len = 0;       /* Block length. Assigned by driver */
    request.owner = (SDIO_Owner) pHwAccess;
    request.physical_buffer = 0; /* Physical address of data buffer is not set */
    request.mode = MMC_DEV_BYTE_INCREMENTAL_MODE;
    request.access_flag = 0;

    os_profile (pHwAccess->hOs, 2, 0);

    status = SDIO_SyncWrite(pHwAccess->hDriver, &request);

    os_profile (pHwAccess->hOs, 3, 0);

#elif HW_ACCESS_WSPI
    status = whal_hwAccess_WriteRegAsync(hHwAccess, addr,  data);
    if (status == TNETWIF_COMPLETE)
    {
        status = OK;
    }
    return status;
    /* check address */
    if ((addr > pHwAccess->registerPartUpperLimit) || (addr < 0))
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwAccess_WriteReg: Error in addr 0x%x (lower:0x%x,upper:0x%x)\n",
            addr, pHwAccess->RegisterRegionAddr, pHwAccess->registerPartUpperLimit));
        return ERROR_HW_ACCEESS_ADDR;
    }

    os_profile (pHwAccess->hOs, 2, 0);

    status = WSPI_WriteSync(pHwAccess->hDriver, TRANSLATE_ADDRESS_REG(addr), (UINT8*)&data, HW_ACCESS_REGISTER_SIZE); 

    os_profile (pHwAccess->hOs, 3, 0);

#else

    HW_REG_LONG_WRITE(pHwAccess, addr, data);

#endif

    if (status != OK)
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwAccess_WriteReg: Error in reg write RegAddr=0x%08x status=%d\n",
        addr, status));

#ifdef HW_ACCESS_SDIO
    if (pHwAccess->uBusError) 
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
        ("whal_hwAccess_WriteReg: SDIO Error in write status=%d\n",
        request.status));
        if (pHwAccess->hwAccesserror_Cb) 
        {
            UINT8 failure_reason = HEALTH_REPORT_BUS_ERROR;
            pHwAccess->hwAccesserror_Cb(pHwAccess->hBackReference,(char *)&failure_reason,2);
        }
        else 
            pHwAccess->uBusError = 0;
    
    }
#endif
    return status;
}
#endif /* USE_SYNC_API */


/****************************************************************************
 *                      whal_hwAccess_ReadRegAsync
 ****************************************************************************
 * DESCRIPTION: read data asynchronously from the TNET using the defined access (WSPI/SDIO). 
 *              the length of data is always 4bytes cause this is the size of the TNET registers
 *              the function is passed a call-back function that will be called after the read request ends.
 *
 * INPUTS:  pHwAccess - TI_HANDLE * - the HwAccess context
 *          addr - UINT32 - the address offset inside the TNET
 *
 * OUTPUT:  data - UINT8* - a pointer to the buffer to fill with the read data
 *
 * RETURNS: one of the error codes (0 => OK)
 ****************************************************************************/
int         whal_hwAccess_ReadRegAsync(TI_HANDLE hHwAccess, UINT32 addr, UINT32* data)
{
#if defined(HW_ACCESS_SDIO)
 
  #if defined(HW_ACCESS_SDIO_ASYNC_SUPPORT)

    #error "SDIO asynchronous mode is not supported"

    /* Not implemented yet */
    return OK;

  #else

    /* Just call to synchronous API */
    return (whal_hwAccess_ReadReg (hHwAccess, addr, data) == OK) 
               ? TNETWIF_COMPLETE
               : TNETWIF_ERROR;

  #endif

#else /* HW_ACCESS_WSPI */

    HwAccess_T_new* pHwAccess = (HwAccess_T_new*)hHwAccess;
    int status;

    /* Check address */
    if (addr > pHwAccess->registerPartUpperLimit)
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwAccess_ReadRegAsync: Error in addr 0x%x (lower:0x%x,upper:0x%x)\n",
            addr, pHwAccess->RegisterRegionAddr, pHwAccess->registerPartUpperLimit));
        return ERROR_HW_ACCEESS_ADDR;
    }

    os_profile (pHwAccess->hOs, 2, 0);

    /* In registers we don't save place */
    status = WSPI_ReadAsync (pHwAccess->hDriver, 
                             TRANSLATE_ADDRESS_REG(addr), 
                             (UINT8*)data, 
                             HW_ACCESS_REGISTER_SIZE,
                             (WSPI_CB_T*)&pHwAccess->CB,
                             TRUE,
                             FALSE);   

    os_profile (pHwAccess->hOs, 3, 0);

    WSPI2TNETWIF (pHwAccess, status, addr);

    return status;

#endif
}

/****************************************************************************
 *                      whal_hwAccess_WriteRegAsync
 ****************************************************************************
 * DESCRIPTION: write data synchronously from the TNET using the defined access (WSPI/SDIO). 
 *              the length of data is always 4bytes cause this is the size of the TNET registers
 *              the function is passed a call-back function that will be called after the read request ends.
 *
 * INPUTS:  pHwAccess - TI_HANDLE * - the HwAccess context
 *          addr - UINT32 - the address offset inside the TNET
 *          data - UINT8* - a pointer to the buffer that holds the data to write
 *
 * OUTPUT:  none                                                                
 *
 * RETURNS: one of the error codes (0 => OK)
 ****************************************************************************/
int         whal_hwAccess_WriteRegAsync(TI_HANDLE hHwAccess, UINT32 addr, UINT32 data)
{
#if defined(HW_ACCESS_SDIO)
 
  #if defined(HW_ACCESS_SDIO_ASYNC_SUPPORT)

    #error "SDIO asynchronous mode is not supported"

    /* Not implemented yet */
    return OK;

  #else

    /* Just call to synchronous API */
    return (whal_hwAccess_WriteReg (hHwAccess, addr, data) == OK)
               ? TNETWIF_COMPLETE 
               : TNETWIF_ERROR;

  #endif

#else /* HW_ACCESS_WSPI */

    HwAccess_T_new* pHwAccess = (HwAccess_T_new*)hHwAccess;
    int status;

    /* Check address */
    if (addr > pHwAccess->registerPartUpperLimit)
    {
        WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
            ("whal_hwAccess_WriteRegAsync: Error in addr 0x%x (lower:0x%x,upper:0x%x)\n",
            addr, pHwAccess->RegisterRegionAddr, pHwAccess->registerPartUpperLimit));
        return ERROR_HW_ACCEESS_ADDR;
    }

    os_profile (pHwAccess->hOs, 2, 0);

    status = WSPI_WriteAsync (pHwAccess->hDriver, 
                              TRANSLATE_ADDRESS_REG(addr), 
                              (UINT8*)&data, 
                              HW_ACCESS_REGISTER_SIZE,
                              (WSPI_CB_T*)&pHwAccess->CB,
                              TRUE,
                              FALSE);   
    os_profile (pHwAccess->hOs, 3, 0);

    WSPI2TNETWIF (pHwAccess, status, addr);

    return status;
#endif
}


#ifdef USE_SYNC_API
/****************************************************************************
 *                      whal_hwAccess_SetPartitions
 ****************************************************************************
 * DESCRIPTION: the client always sees two partitions
 *              1 - the working partition that the client access it with mem functions (for this partition the client supplies the base address)
 *              2 - the registers partition that is always set on HW_ACCESS_REG_ADDR and its size is HW_ACCESS_REG_SIZE.
 *
 * INPUTS:  pHwAccess - a pointer to the module context
 *          partitionMode - the mode to set the partitions
 *          partition_start - the start address of the working partition
 *
 * OUTPUT:  none
 *
 * RETURNS: one of the error codes (0 => OK)
 ****************************************************************************/
int whal_hwAccess_SetPartitions (TI_HANDLE hHwAccess, TNETIF_HwAccess_SetPartition_mode_e partitionMode, UINT32 partition_start)
{                                   
    HwAccess_T_new *pHwAccess = (HwAccess_T_new*) hHwAccess;
    int status = OK;

#ifdef HW_ACCESS_SDIO   
    SDIO_TNETWConfigParams sdioConfParam;
    Peripheral_ConfigParams *cfg;
#endif

    pHwAccess->partition[0].size  = 0;
    pHwAccess->partition[0].start = partition_start;
    pHwAccess->partition[1].size  = 0;
    pHwAccess->partition[1].start = 0;

    switch(partitionMode)
    {
        case HW_ACCESS_DOWNLOAD:
            pHwAccess->partition[0].size  = HW_ACCESS_MEMORY_MAX_RANGE - HW_ACCESS_DOWN_PART1_SIZE;
            pHwAccess->partition[1].start = HW_ACCESS_DOWN_PART1_ADDR;
            pHwAccess->partition[1].size  = HW_ACCESS_DOWN_PART1_SIZE;
            break;
        case HW_ACCESS_WORKING:
            pHwAccess->partition[0].size  = HW_ACCESS_MEMORY_MAX_RANGE - HW_ACCESS_WORK_PART1_SIZE;
            pHwAccess->partition[1].start = HW_ACCESS_WORK_PART1_ADDR;
            pHwAccess->partition[1].size  = HW_ACCESS_WORK_PART1_SIZE;          
            break;
    }

#if (defined(HW_ACCESS_SDIO) || defined(HW_ACCESS_WSPI))
    /* guarantee that the working partition wont overlap the registers partition */
    if((pHwAccess->partition[0].start + pHwAccess->partition[0].size) > pHwAccess->partition[1].start)
        pHwAccess->partition[0].size =  pHwAccess->partition[1].start -  pHwAccess->partition[0].start;

    /* guarantee that the working partition won't exceed the packet RAM */
    if((pHwAccess->partition[0].start + pHwAccess->partition[0].size) > HW_ACCESS_PRAM_MAX_RANGE)
        pHwAccess->partition[0].size =  HW_ACCESS_PRAM_MAX_RANGE -  pHwAccess->partition[0].start;

    pHwAccess->workingPartUpperLimit = pHwAccess->partition[0].start + pHwAccess->partition[0].size;
    pHwAccess->registerPartUpperLimit = pHwAccess->partition[1].size;

    pHwAccess->RegisterRegionAddr = pHwAccess->partition[0].size;
    pHwAccess->MemRegionAddr = pHwAccess->partition[0].start;  

#ifdef HW_ACCESS_SDIO

    os_memoryZero (pHwAccess->hOs, &sdioConfParam, sizeof(SDIO_TNETWConfigParams));
    sdioConfParam.num_of_parts = 2;
    sdioConfParam.map_reg[0].reg_size   = pHwAccess->partition[0].size;
    sdioConfParam.map_reg[0].scr_offset = pHwAccess->partition[0].start;
    sdioConfParam.map_reg[1].reg_size   = pHwAccess->partition[1].size;
    sdioConfParam.map_reg[1].scr_offset = pHwAccess->partition[1].start;

    status = SDIO_TNETWInit(&sdioConfParam);    
    status = (status == SDIO_SUCCESS) ? SDIO_TNETWConfig(pHwAccess->hDriver, &cfg) : status;  
    status = (status == SDIO_SUCCESS) ? TNETWIF_COMPLETE : TNETWIF_ERROR;

#elif HW_ACCESS_WSPI
    /* 
     * IMPORTANT NOTE (TODO): the ASYNC API is used here assuming the call will always
     * be completed synchronously. This is done because only the Async API is wokring
     * in 4.0.4. In future versions, the whole recovery process should be made asynchronous.
     */
    /* May use only one write because the addresses in the firmware are sequential */
    status = WSPI_WriteAsync (pHwAccess->hDriver, 
                              HW_ACCESS_PART0_SIZE_ADDR, 
                              PADWRITE (pHwAccess->partition), 
                              sizeof(partition_t) * 2,
                              (WSPI_CB_T*)&pHwAccess->CB,
                              TRUE, TRUE);

    WSPI2TNETWIF (pHwAccess, status, HW_ACCESS_PART0_SIZE_ADDR);
#endif

#endif
    
    return status;
}
#endif /* USE_SYNC_API */


/****************************************************************************
 *                      whal_hwAccess_SetPartitionsAsync
 ****************************************************************************
 * DESCRIPTION: the client always sees two partitions
 *              1 - the working partition that the client access it with mem functions (for this partition the client supplies the base address)
 *              2 - the registers partition that is always set on HW_ACCESS_REG_ADDR and its size is HW_ACCESS_REG_SIZE.
 *
 * INPUTS:  pHwAccess - a pointer to the module context
 *          partitionMode - the mode to set the partitions
 *          partition_start - the start address of the working partition
 *
 * OUTPUT:  none
 *
 * RETURNS: one of the error codes (0 => OK)
 ****************************************************************************/
int whal_hwAccess_SetPartitionsAsync (TI_HANDLE hHwAccess, TNETIF_HwAccess_SetPartition_mode_e partitionMode, UINT32 partition_start)
{
    HwAccess_T_new *pHwAccess = (HwAccess_T_new*) hHwAccess;
    int status = OK;

#ifdef HW_ACCESS_SDIO 
    SDIO_TNETWConfigParams sdioConfParam;
    Peripheral_ConfigParams *cfg;
#endif

    pHwAccess->partition[0].size  = 0;
    pHwAccess->partition[0].start = partition_start;
    pHwAccess->partition[1].size  = 0;
    pHwAccess->partition[1].start = 0;

    switch(partitionMode)
    {
        case HW_ACCESS_DOWNLOAD:
            pHwAccess->partition[0].size  = HW_ACCESS_MEMORY_MAX_RANGE - HW_ACCESS_DOWN_PART1_SIZE;
            pHwAccess->partition[1].start = HW_ACCESS_DOWN_PART1_ADDR;
            pHwAccess->partition[1].size  = HW_ACCESS_DOWN_PART1_SIZE;
            break;
        case HW_ACCESS_WORKING:
            pHwAccess->partition[0].size  = HW_ACCESS_MEMORY_MAX_RANGE - HW_ACCESS_WORK_PART1_SIZE;
            pHwAccess->partition[1].start = HW_ACCESS_WORK_PART1_ADDR;
            pHwAccess->partition[1].size  = HW_ACCESS_WORK_PART1_SIZE;          
            break;
    }

#if (defined(HW_ACCESS_SDIO) || defined(HW_ACCESS_WSPI))
    /* guarantee that the working partition wont overlap the registers partition */
    if((pHwAccess->partition[0].start + pHwAccess->partition[0].size) > pHwAccess->partition[1].start)
        pHwAccess->partition[0].size =  pHwAccess->partition[1].start -  pHwAccess->partition[0].start;
    
    /* guarantee that the working partition won't exceed the packet RAM */
    if((pHwAccess->partition[0].start + pHwAccess->partition[0].size) > HW_ACCESS_PRAM_MAX_RANGE)
        pHwAccess->partition[0].size =  HW_ACCESS_PRAM_MAX_RANGE -  pHwAccess->partition[0].start;

    pHwAccess->workingPartUpperLimit = pHwAccess->partition[0].start + pHwAccess->partition[0].size;
    pHwAccess->registerPartUpperLimit = pHwAccess->partition[1].size;

    pHwAccess->RegisterRegionAddr = pHwAccess->partition[0].size;
    pHwAccess->MemRegionAddr = pHwAccess->partition[0].start;  

#ifdef HW_ACCESS_SDIO

    os_memoryZero (pHwAccess->hOs, &sdioConfParam, sizeof(SDIO_TNETWConfigParams));
    sdioConfParam.num_of_parts = 2;
    sdioConfParam.map_reg[0].reg_size   = pHwAccess->partition[0].size;
    sdioConfParam.map_reg[0].scr_offset = pHwAccess->partition[0].start;
    sdioConfParam.map_reg[1].reg_size   = pHwAccess->partition[1].size;
    sdioConfParam.map_reg[1].scr_offset = pHwAccess->partition[1].start;

    status = SDIO_TNETWInit(&sdioConfParam);    
    status = (status == SDIO_SUCCESS) ? SDIO_TNETWConfig(pHwAccess->hDriver, &cfg) : status;  
    status = (status == SDIO_SUCCESS) ? TNETWIF_COMPLETE : TNETWIF_ERROR;

#elif HW_ACCESS_WSPI

    /* May use only one write because the addresses in the firmware are sequential */
    status = WSPI_WriteAsync (pHwAccess->hDriver, 
                              HW_ACCESS_PART0_SIZE_ADDR, 
                              PADWRITE (pHwAccess->partition), 
                              sizeof(partition_t) * 2,
                              (WSPI_CB_T*)&pHwAccess->CB,
                              TRUE,
                              TRUE);   

    WSPI2TNETWIF (pHwAccess, status, HW_ACCESS_PART0_SIZE_ADDR);

#endif

#endif
    
    return status;
}


#if !defined(HW_ACCESS_SDIO) && !defined(HW_ACCESS_WSPI)
/****************************************************************************
 *                      whal_hwAccess_DirectCopy_new()
 ****************************************************************************
 * DESCRIPTION: Direct Copy sequence of bytes to/from the hardware
 * 
 * INPUTS:
 *      pHwAccess   The object
 *      Dest        Destination address
 *      Src         Source address
 *      Len         The length of the data to copy
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
static void whal_hwAccess_DirectCopy_new(HwAccess_T_new *pHwAccess, UINT8* Dest, UINT8* Src, UINT32 Len)
{

#ifdef NOT_SUPPORT_08_BIT_ACCESS_COMMAND

    /* first see if dest addr is 4 bytes 
       align if not we are using local copy and not 
       os_memoryCopy 
     */
    if(((UINT32)Dest & 0x00000001))
    {
    /* TBD - Currently unaligned destination address is not handled.
       it means that CKIP will not work currently.
    */
        
    }else
    if(((UINT32)Dest & 0x00000002))
    {
    register volatile unsigned short *shortDest = (volatile unsigned short *)Dest;
    register volatile unsigned short *shortSrc = (unsigned short *)Src;
    register int Size = Len >> 1;

    unsigned short TempWord;
    while (Size--)
        *shortDest++ = *shortSrc++;

    if (Len&1)
    {
        TempWord = ENDIAN_HANDLE_WORD((*shortSrc & 0x00ff) | (*shortDest & 0xff00));
        *shortDest = TempWord;
    }
    }else
    {
        /* The address line A0 is not connected 
           For arm/linux sake not 2, but 4-alignment is enforced.
           (Linux memcpy uses longs copy logic)
        */ 
        int Len1 = Len & 0xfffffffc;

        os_memoryCopy(pHwAccess->hOs, (void*)Dest, (void*)Src, Len1);

        if (Len1 != Len)
        {
            int ShortSize=Len1 >> 1;
            volatile unsigned short *shortDest = ((unsigned short *)Dest) + ShortSize;
            volatile unsigned short *shortSrc =  ((unsigned short *)Src) + ShortSize;
                if (Len - Len1 >= 2)
                    *shortDest++ = *shortSrc++;
                if (Len & 1)
                    *shortDest = ENDIAN_HANDLE_WORD((*shortSrc & 0x00ff) | (*shortDest & 0xff00));
        }
    }          

#else
    os_memoryCopy(pHwAccess->hOs, (void*)Dest, (void*)Src, Len);
#endif
}
#endif

 
#ifdef HW_ACCESS_SDIO

/* callback notify read/write transaction function */
static void sdio_transaction_notify_read(struct SDIO_Request *req, int status)
{
}

static void sdio_transaction_notify_write(struct SDIO_Request *req, int status)
{
}

/* callback notify error read/write transaction function */
static void sdio_transaction_error(struct SDIO_Request *req, int stat)
{
    HwAccess_T_new* pHwAccess = (HwAccess_T_new*)req->owner;

    WLAN_REPORT_ERROR (pHwAccess->hReport, HAL_CTRL_MODULE_LOG, ("sdio_transaction_error\n"));                    

    pHwAccess->uBusError = 1;
}


#if defined(HW_ACCESS_SDIO_ASYNC_SUPPORT)

static void sdio_async_transaction_notify(struct SDIO_Request *req, int status)
{   
    HwAccess_T_new* pHwAccess = (HwAccess_T_new*)req->owner;

    /* call the CB */
    pHwAccess->CB.CBFunc(pHwAccess->CB.CBArg,status);

    WLAN_REPORT_INFORMATION(pHwAccess->hReport,
                            HAL_HW_CTRL_MODULE_LOG,
                            ("sdio_async_transaction_notify - request=%p, buf=%p, sdio_addr=0x%08lx, len=%u\n",
                            req, req->buffer, req->peripheral_addr, req->block_len));

}

/* callback notify error read/write transaction function */
static void sdio_async_transaction_error(struct SDIO_Request *req, int status)
{
    HwAccess_T_new* pHwAccess = (HwAccess_T_new*)req->owner;

    WLAN_REPORT_ERROR(pHwAccess->hReport, HAL_HW_CTRL_MODULE_LOG,  
                    ("sdio_async_transaction_error: request=%p, buf=%p, sdio_addr=0x%08lx, len=%u\n", 
                    req, req->buffer, req->peripheral_addr, req->block_len));
}

#endif /*HW_ACCESS_SDIO_ASYNC_SUPPORT*/

#endif /*HW_ACCESS_SDIO*/

TI_STATUS whal_hwAccess_RegisterForErrorCB(TI_HANDLE hHwAccess,
                                                 void* CbFunc,
                                                 TI_HANDLE CbObj)
{
    HwAccess_T_new *pHwAccess = (HwAccess_T_new *)hHwAccess;
    pHwAccess->hwAccesserror_Cb = (HwAccessErrorHandle)CbFunc;
    pHwAccess->hBackReference = CbObj;
    return OK;
}


/***********************************************************/


#ifdef USE_SYNC_API
/****************************************************************************
 *                      whal_hwAccess_RegSetBitVal()
 ****************************************************************************
 * DESCRIPTION: wlan hardware registers bits access 
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
void whal_hwAccess_RegSetBitVal(TI_HANDLE hHwAccess, UINT32 RegAddr, UINT32 BitVal)
{
    UINT32 rVal;

    whal_hwAccess_ReadReg(hHwAccess,RegAddr,&rVal);
    rVal |= BitVal;
    whal_hwAccess_WriteReg(hHwAccess, RegAddr, rVal);
}

/****************************************************************************
 *                      whal_hwAccess_RegResetBitVal()
 ****************************************************************************
 * DESCRIPTION: wlan hardware registers bits access 
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
void whal_hwAccess_RegResetBitVal(TI_HANDLE hHwAccess, UINT32 RegAddr, UINT32 BitVal)
{
    UINT32 rVal;

    whal_hwAccess_ReadReg(hHwAccess,RegAddr,&rVal);
    rVal &= ~BitVal;
    whal_hwAccess_WriteReg(hHwAccess, RegAddr, rVal);
}

/****************************************************************************
 *                      whal_hwAccess_RegIsBitSet()
 ****************************************************************************
 * DESCRIPTION: wlan hardware registers bits access 
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
int whal_hwAccess_RegIsBitSet(TI_HANDLE hHwAccess, UINT32 RegAddr, UINT32 BitVal)
{
    UINT32 RegVal;

    whal_hwAccess_ReadReg(hHwAccess,RegAddr,&RegVal);
    if (RegVal & BitVal)
        return 1;

    return 0;
}


/****************************************************************************
 *                      whal_hwAccess_(Get/Set/SetBits/ResetBits)U08()
 ****************************************************************************
 * DESCRIPTION: wlan hardware memory access to bytes
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
UINT8 whal_hwAccess_GetU08(TI_HANDLE hHwAccess, UINT32 Addr)
{


#ifdef HW_ACCESS_SDIO
    UINT32 SrcOffset ;
    UINT16 DataShort;

    SrcOffset = (Addr & 0xfffffffe);        
    whal_hwAccess_ReadMem(hHwAccess,(UINT32)((char *)SrcOffset),(UINT8*)((char *)&DataShort),sizeof(UINT16));
    return (EXTRACT_BYTE_FROM_WORD(DataShort, Addr));

#else /* HW_ACCESS_SDIO */

    UINT32 data = 0;

    whal_hwAccess_ReadMem(hHwAccess,(UINT32)Addr,(UINT8*)&data,sizeof(data));   
    return (data & 0x000000FF);

#endif /* HW_ACCESS_SDIO */

 }

void whal_hwAccess_SetU08(TI_HANDLE hHwAccess, UINT32 Addr, UINT8 Val)
{


#ifdef HW_ACCESS_SDIO

    UINT32 sdioAddr;
    UINT16 DataShort;

    sdioAddr = (Addr & 0xfffffffe);
    whal_hwAccess_ReadMem(hHwAccess,(UINT32)((char *)sdioAddr),(UINT8*)((char *)&DataShort),sizeof(UINT16));        
    EXTRACT_BYTE_FROM_WORD(DataShort, Addr) = Val;  
    whal_hwAccess_WriteMem(hHwAccess,(UINT32)((char *)sdioAddr),(UINT8*)((char *)&DataShort),sizeof(UINT16));

#else /* HW_ACCESS_SDIO */

    UINT32 data = 0;
    
    whal_hwAccess_ReadMem(hHwAccess,(UINT32)Addr,(UINT8*)&data,sizeof(data));   
    data &= 0xFFFFFF00;
    data |= Val;
    whal_hwAccess_WriteMem(hHwAccess,(UINT32)Addr,(UINT8*)&data,sizeof(data));
    
#endif /* HW_ACCESS_SDIO */
    
 }


void whal_hwAccess_ResetU08_Bits(TI_HANDLE hHwAccess, UINT32 Addr, UINT8 BitsVal)
{


#ifdef HW_ACCESS_SDIO

    UINT32 sdioAddr;
    UINT16 DataShort;

    sdioAddr = (Addr & 0xfffffffe);
    whal_hwAccess_ReadMem(hHwAccess,(UINT32)((char *)sdioAddr),(UINT8*)((char *)&DataShort),sizeof(UINT16));
    EXTRACT_BYTE_FROM_WORD(DataShort, Addr) &= ~BitsVal;
    whal_hwAccess_WriteMem(hHwAccess,(UINT32)((char *)sdioAddr),(UINT8*)((char *)&DataShort),sizeof(UINT16));

#else /* HW_ACCESS_SDIO */
    
    UINT32 data = 0;

    whal_hwAccess_ReadMem(hHwAccess,(UINT32)Addr,(UINT8*)&data,sizeof(data));       
    data &= ~BitsVal;
    whal_hwAccess_WriteMem(hHwAccess,(UINT32)Addr,(UINT8*)&data,sizeof(data));  

#endif /* HW_ACCESS_SDIO */
            
 }

/****************************************************************************
 *                      whal_hwAccess_(Get/Set/SetBits/ResetBits)U16()
 ****************************************************************************
 * DESCRIPTION: wlan hardware memory access to shorts
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
UINT16 whal_hwAccess_GetU16(TI_HANDLE hHwAccess, UINT32 Addr)
{


#ifdef HW_ACCESS_SDIO

    UINT32 sdioAddr;    
    UINT16 DataShort;

    sdioAddr = Addr;
    whal_hwAccess_ReadMem(hHwAccess,(UINT32)((char *)sdioAddr),(UINT8*)((char *)&DataShort),sizeof(UINT16));

    return (ENDIAN_HANDLE_WORD(DataShort));

#else /* HW_ACCESS_SDIO */
    
    UINT32 data = 0;

    whal_hwAccess_ReadMem(hHwAccess,(UINT32)Addr,(UINT8*)&data,sizeof(data));   
    return (data & 0x0000FFFF);

#endif /* HW_ACCESS_SDIO */
        
 }

void whal_hwAccess_SetU16(TI_HANDLE hHwAccess, UINT32 Addr, UINT16 Val)
{

    
#ifdef HW_ACCESS_SDIO

    UINT32 sdioAddr;
    UINT16 sdioVal;

    sdioAddr = (Addr & 0xfffffffe);
    sdioVal = ENDIAN_HANDLE_WORD(Val);
    whal_hwAccess_WriteMem(hHwAccess,(UINT32)((char *)sdioAddr),(UINT8*)((char *)&sdioVal),sizeof(UINT16));

#else /* HW_ACCESS_SDIO */
    
    UINT32 data = 0;
    UINT32 sdioAddr;

    sdioAddr = (Addr & 0xfffffffc);
    whal_hwAccess_ReadMem(hHwAccess,(UINT32)sdioAddr,(UINT8*)&data,sizeof(data));

    if((Addr&0x3) == 2)
    {
        data &= 0x0000FFFF;
        data |= ((UINT32)Val)<<16;
    }
    else
    {
        data &= 0xFFFF0000;
        data |= Val;
    }
    whal_hwAccess_WriteMem(hHwAccess,(UINT32)sdioAddr,(UINT8*)&data,sizeof(data));  
    
#endif /* HW_ACCESS_SDIO */
        
 }

void whal_hwAccess_SetU16_Bits(TI_HANDLE hHwAccess, UINT32 Addr, UINT16 Val)
{


#ifdef HW_ACCESS_SDIO

    UINT32 sdioAddr;
    UINT16 sdioVal;   

    sdioAddr = Addr;
    whal_hwAccess_ReadMem(hHwAccess,(UINT32)((char *)sdioAddr),(UINT8*)((char *)&sdioVal),sizeof(UINT16));
    sdioVal |= ENDIAN_HANDLE_WORD(Val);
    whal_hwAccess_WriteMem(hHwAccess,(UINT32)((char *)sdioAddr),(UINT8*)((char *)&sdioVal),sizeof(UINT16));

#else /* HW_ACCESS_SDIO */
    
    UINT32 data = 0;

    whal_hwAccess_ReadMem(hHwAccess,(UINT32)Addr,(UINT8*)&data,sizeof(data));
    data |= Val;
    whal_hwAccess_WriteMem(hHwAccess,(UINT32)Addr,(UINT8*)&data,sizeof(data));
    
#endif /* HW_ACCESS_SDIO */
        
 }

void whal_hwAccess_ResetU16_Bits(TI_HANDLE hHwAccess, UINT32 Addr, UINT16 Val)
{


#ifdef HW_ACCESS_SDIO

    UINT32 sdioAddr;
    UINT16 sdioVal;

    sdioAddr = Addr;
    whal_hwAccess_ReadMem(hHwAccess,(UINT32)((char *)sdioAddr),(UINT8*)((char *)&sdioVal),sizeof(UINT16));
    sdioVal &= ~ENDIAN_HANDLE_WORD(Val);
    whal_hwAccess_WriteMem(hHwAccess,(UINT32)((char *)sdioAddr),(UINT8*)((char *)&sdioVal),sizeof(UINT16));

#else /* HW_ACCESS_SDIO */
    
    UINT32 data = 0;

    whal_hwAccess_ReadMem(hHwAccess,(UINT32)Addr,(UINT8*)&data,sizeof(data));
    data &= ~Val;
    whal_hwAccess_WriteMem(hHwAccess,(UINT32)Addr,(UINT8*)&data,sizeof(data));
    
#endif /* HW_ACCESS_SDIO */

 }

/****************************************************************************
 *                      whal_hwAccess_(Get/Set/SetBits/ResetBits)U32()
 ****************************************************************************
 * DESCRIPTION: wlan hardware memory access to longs
 * 
 * INPUTS:  
 * 
 * OUTPUT:  None
 * 
 * RETURNS: OK or NOK
 ****************************************************************************/
UINT32 whal_hwAccess_GetU32(TI_HANDLE hHwAccess, UINT32 Addr)
{


#ifdef HW_ACCESS_SDIO

    UINT32 DataLong;
    UINT32 sdioAddr;

    sdioAddr = Addr;    
    whal_hwAccess_ReadMem(hHwAccess,(UINT32)((char *)sdioAddr),(UINT8*)((char *)&DataLong),sizeof(UINT32));
    return (ENDIAN_HANDLE_LONG(DataLong));

#else /* HW_ACCESS_SDIO */
    
    UINT32 data = 0;

    whal_hwAccess_ReadMem(hHwAccess,(UINT32)Addr,(UINT8*)&data,sizeof(data));
    return data;
    
#endif /* HW_ACCESS_SDIO */
    
 }


void whal_hwAccess_SetU32(TI_HANDLE hHwAccess, UINT32 Addr, UINT32 Val)
{


#ifdef HW_ACCESS_SDIO

    UINT32 sdioAddr;
    UINT32 sdioVal;

    sdioAddr = Addr;
    sdioVal = ENDIAN_HANDLE_WORD(Val);
    whal_hwAccess_WriteMem(hHwAccess,(UINT32)((char *)sdioAddr),(UINT8*)((char *)&sdioVal),sizeof(UINT32));

#else /* HW_ACCESS_SDIO */
    
    whal_hwAccess_WriteMem(hHwAccess,(UINT32)Addr,(UINT8*)&Val,sizeof(Val));    
    
#endif /* HW_ACCESS_SDIO */
        
 }

void whal_hwAccess_SetU32_Bits(TI_HANDLE hHwAccess, UINT32 Addr, UINT32 Val)
{


#ifdef HW_ACCESS_SDIO

    UINT32 sdioAddr;
    UINT32 DataShort;
    
    sdioAddr = (Addr & 0xfffffffe);
    whal_hwAccess_ReadMem(hHwAccess,(UINT32)((char *)sdioAddr),(UINT8*)((char *)&DataShort),sizeof(UINT32));
    DataShort |= ENDIAN_HANDLE_LONG(Val);
    whal_hwAccess_WriteMem(hHwAccess,(UINT32)((char *)sdioAddr),(UINT8*)((char *)&DataShort),sizeof(UINT32));

#else /* HW_ACCESS_SDIO */
    
    UINT32 data = 0;

    whal_hwAccess_ReadMem(hHwAccess,(UINT32)Addr,(UINT8*)&data,sizeof(data));
    data |= Val;
    whal_hwAccess_WriteMem(hHwAccess,(UINT32)Addr,(UINT8*)&data,sizeof(data));  
    
#endif /* HW_ACCESS_SDIO */
    
 }

void whal_hwAccess_ResetU32_Bits(TI_HANDLE hHwAccess, UINT32 Addr, UINT32 Val)
{


#ifdef HW_ACCESS_SDIO

    UINT32 sdioAddr;
    UINT32 DataShort;

    sdioAddr = (Addr & 0xfffffffe);
    whal_hwAccess_ReadMem(hHwAccess,(UINT32)((char *)sdioAddr),(UINT8*)((char *)&DataShort),sizeof(UINT32));
    DataShort &= ~ENDIAN_HANDLE_LONG(Val);      
    whal_hwAccess_WriteMem(hHwAccess,(UINT32)((char *)sdioAddr),(UINT8*)((char *)&DataShort),sizeof(UINT32));

#else /* HW_ACCESS_SDIO */
    
    UINT32 data = 0;

    whal_hwAccess_ReadMem(hHwAccess,(UINT32)Addr,(UINT8*)&data,sizeof(data));
    data &= ~Val;
    whal_hwAccess_WriteMem(hHwAccess,(UINT32)Addr,(UINT8*)&data,sizeof(data));  
    
#endif /* HW_ACCESS_SDIO */
        
 }
 
#endif /* USE_SYNC_API */


/****************************************************************************
 *                      whal_hwAccess_Print()
 ****************************************************************************
 * DESCRIPTION: Print interrupts information
 * 
 * INPUTS:  None
 * 
 * OUTPUT:  None
 * 
 * RETURNS: 
 ****************************************************************************/
void whal_hwAccess_Print(TI_HANDLE hHwAccess)
{
    /* will be in use only if a print function will be neaded.
    HwAccess_T_new *pHwAccess = (HwAccess_T_new*) hHwAccess;
    */
    WLAN_OS_REPORT(("whal_hwAccess_Print ----------------------------\n")); 
    WLAN_OS_REPORT(("    NOTHING RIGHTNOW\n")); 
}

/************************************************************************
 * Internal functions
 ************************************************************************/

#ifdef HW_ACCESS_SDIO
  TI_STATUS whal_hwAccess_RecreateInterface(TI_HANDLE hHwAccess)
  {
      SDIO_ConfigParams configParams;
      HwAccess_T_new *pHwAccess = (HwAccess_T_new*)hHwAccess;
      SDIO_Status   status;
  
     os_memoryZero( pHwAccess->hOs, &configParams, sizeof(SDIO_ConfigParams));
      configParams.fnotify_read = sdio_transaction_notify_read; 
      configParams.fnotify_write = sdio_transaction_notify_write; 
      configParams.ferror = sdio_transaction_error;  
      configParams.fconfig_peripheral = SDIO_TNETWConfig; 
      configParams.fconvert = NULL;
      configParams.owner = pHwAccess;
  
      status = SDIO_Init(&configParams, &pHwAccess->hDriver);   
    
      return (status == SDIO_SUCCESS) ? OK : NOK;
  }
#elif defined (HW_ACCESS_WSPI)
  TI_STATUS whal_hwAccess_RecreateInterface(TI_HANDLE hHwAccess)
  {
      HwAccess_T_new *pHwAccess = (HwAccess_T_new*)hHwAccess;
      int             status;

      /* re-configure the WSPi interface, using NULL to keep old parameters */
      status = WSPI_Configure (pHwAccess->hDriver , NULL, NULL, NULL);

#ifndef _WINDOWS
      WSPI2TNETWIF (pHwAccess, status, 0x0);
	  return (TI_STATUS)status;
#else
#endif
  }
#else   /* ifdef HW_ACCESS_SDIO */
  TI_STATUS whal_hwAccess_RecreateInterface(TI_HANDLE hHwAccess)
  {
      TI_STATUS status = NOK;

    /*#error whal_hwAccess_RecreateInterface function is not implemented for HW interface other than SDIO.*/

      return (status);
  }
#endif  /* ifdef HW_ACCESS_SDIO */


