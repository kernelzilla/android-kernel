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
 *   MODULE:  wspi.c
 *   PURPOSE: 
 *
 ****************************************************************************/


#include "osTIType.h"
#include "osApi.h"
#include "whalCommon.h"
#include "spi_api.h"
#include "wspi.h"

/* WSPI INIT CMD 
-----------------------------------------------------------------
| start |  tx   |                command                        |
-----------------------------------------------------------------
    47      46      45                                      40

-----------------------------------------------------------------
|                           reserve                             |
-----------------------------------------------------------------
    39                                                      32

-----------------------------------------------------------------
|                           reserve                             |
-----------------------------------------------------------------
    31                                                      24

-----------------------------------------------------------------
|           reserve              |  1250 |   fixed busy length  | 
-----------------------------------------------------------------
    23                                                      16
                                                                                    
-----------------------------------------------------------------
| fixed |       |       |       |       |       |       |       |
| busy  | iod   |   ip  |   cs  |   ws  |   bs  |  de   | wspi  |
| enable|       |       |       |       |       |       |       |
-----------------------------------------------------------------
    15      14      13      12      11      10      9       8

-----------------------------------------------------------------
|                         CRC7                            | end |
-----------------------------------------------------------------
    7                                               1        0
*/

#define WSPI_SIZEOF_UINT32          sizeof (UINT32)

#define WSPI_INIT_CMD_CRC_LEN       5

#define WSPI_INIT_CMD_START         0x00
#define WSPI_INIT_CMD_TX            0x40
#define WSPI_INIT_CMD_BYPASS_BIT    0x80 /* the extra bypass bit is sampled by the TNET as '1' */
#define WSPI_INIT_CMD_FIXEDBUSY_LEN 0x07
#define WSPI_INIT_CMD_EN_FIXEDBUSY  0x80
#define WSPI_INIT_CMD_DIS_FIXEDBUSY 0x00
#define WSPI_INIT_CMD_IOD           0x40
#define WSPI_INIT_CMD_IP            0x20
#define WSPI_INIT_CMD_CS            0x10
#define WSPI_INIT_CMD_WS            0x08
#define WSPI_INIT_CMD_WSPI          0x01
#define WSPI_INIT_CMD_END           0x01  
#define WSPI_INIT_CMD_CRC_INPUT_LEN 40

#define PRINT_TEST_REGION_SIZE      100
#define PRINT_TEST_REGION_START     0x305674 /* the device ID register */
#define PRINT_TEST_SIZE_REG         0x1FFC0
#define PRINT_TEST_ADDR_REG         0x1FFC4


/* WSPI CMD                                                                         
-----------------------------------------------------------------
| start |  tx   | fixed |         byte length                   |
-----------------------------------------------------------------
    31      30      29      28                              24

-----------------------------------------------------------------
|                           byte length                 |   a16 |
-----------------------------------------------------------------
    23                                              17      16

-----------------------------------------------------------------
|                           byte address                        |
-----------------------------------------------------------------
    15                                                      8

-----------------------------------------------------------------
|                           byte address                        | 
-----------------------------------------------------------------
    7                                                       0
*/


/* The 31's bit in the WSPI cmd is read bit */
#define WSPI_CMD_READ                 0x40000000 
#define WSPI_CMD_WRITE                0x00000000
#define WSPI_CMD_FIXED                0x20000000
#define WSPI_CMD_BYTE_LENGTH          0x1FFE0000 
#define WSPI_CMD_BYTE_LENGTH_OFFSET   17
#define WSPI_CMD_BYTE_ADDR            0x0001FFFF 


#define WSPI_FIXED_BUSY_TIMEOUT       100
#define WSPI_SYNC_OVER_ASYNC_TIMEOUT  100


/* For FixedBusy Handle */
#define NOT_IN_USE					  0

static UINT8 WSPI_GenerateCRC7 (UINT8* bits, UINT32 len);
#ifndef USE_WRITE_READ_API
static void  WSPI_WriteCmdCb      (TI_HANDLE hWSPI, int status);
#endif
static void  WSPI_ReadDataCb      (TI_HANDLE hWSPI, int status);
#ifdef USE_SYNC_API
#ifdef USE_SYNC_OVER_ASYNC
static void  WSPI_SyncOverAsyncCb (TI_HANDLE hWSPI, int status);
#endif
#endif 
static void  WSPI_ReadAsyncCb      (TI_HANDLE hWSPI, int status);
static void  WSPI_ConfigureResetCb (TI_HANDLE hWSPI, int status);

#ifdef TI_DBG
	int DebugFixedBusy[10];
#endif


/*
 * ----------------------------------------------------------------------------
 * Function :   WSPI_Open
 *
 * Input    :   none
 *
 * Output   :   void**  phWSPI - a pointer to the handle of the WSPI
 *
 * ReturnVal:   one of the error codes (zero is success)
 *
 * Note(s)  :   allocates the WSPI driver context and creates the SPI sublayer
 * -----------------------------------------------------------------------------
 */
TI_HANDLE WSPI_Open (TI_HANDLE hOs)
{   
    WSPI_t* pWSPI;

    /* Allocate WSPI memory */
    pWSPI = (WSPI_t *) os_memoryAlloc (hOs, sizeof(WSPI_t));
    if (pWSPI == NULL)
    {
        return NULL;
    }
   
    pWSPI->hOs = hOs;
    pWSPI->pTempBuf = 0;
	pWSPI->pExtraFixedBusyBuf = 0;
        
    /* Call to SPI_Open */
    pWSPI->hSPI = SPI_Open ();   

    return (TI_HANDLE)pWSPI;
}
    

/*
 * ----------------------------------------------------------------------------
 * Function :   WSPI_Close
 *
 * Input    :   void* hWSPI - the WSPI handle
 *
 * Output   :   none
 *
 * ReturnVal:   one of the error codes (zero is success)
 *
 * Note(s)  :   frees the WSPI driver context and closes the SPI sublayer
 * -----------------------------------------------------------------------------
 */
int WSPI_Close (TI_HANDLE hWSPI)
{
    WSPI_t* pWSPI = (WSPI_t*)hWSPI;

    /* Call SPI_Close */
    SPI_Close (pWSPI->hSPI);

    /* Free temporary buffer */
    if (pWSPI->pTempBuf)
    {
        os_memoryFree (pWSPI->hOs, pWSPI->pTempBuf, WSPI_NO_EXTRA_ALLOC_SIZE + pWSPI->uFixedBusyBytes);
    }


	if (pWSPI->pExtraFixedBusyBuf)
    {
        os_memoryFree (pWSPI->hOs, pWSPI->pExtraFixedBusyBuf, WSPI_EXTRA_BUFFER_ALLOC_SIZE);
    }

    /* Free allocated memory */
    os_memoryFree (pWSPI->hOs, hWSPI, sizeof(WSPI_t));

    return WSPI_OK;
}


/*
 * ----------------------------------------------------------------------------
 * Function :   WSPI_Configure
 *
 * Input    :   void* hWSPI - the WSPI handle
 *              const WSPIConfig_t* aConfig -  a structure that holds the configuration data 
                WSPI_CB_T* cb - callback 
 *
 * Output   :   
 *
 * ReturnVal:   one of the error codes (zero is success)
 *
 * Note(s)  :   1. configures the SPI sublayer
 *              2. sends the WSPI init word
 * -----------------------------------------------------------------------------
 */
int WSPI_Configure (TI_HANDLE hWSPI, TI_HANDLE hReport, const WSPIConfig_t* aConfig, WSPI_CB_T* cb)
{
    WSPI_t *pWSPI = (WSPI_t*)hWSPI;
    int i;
    
	#ifdef TI_DBG
		for (i=0;i<10;i++) 
		{
			DebugFixedBusy[i] = 0;
		}
	#endif
    
    /* 
     * This function is called during initialization and recovery. In recovery,
     * The old values are used, so there's no need to re-configure everything again.
     */
    if (NULL != aConfig)
    {
    /* Set WSPI_t parameters */
    pWSPI->uConfigMask = aConfig->mask;
    pWSPI->bFixedAddr = aConfig->isFixedAddress; 
    pWSPI->uFixedBusyLen = aConfig->fixedBusyLength;   
    pWSPI->uFixedBusyBytes = pWSPI->uFixedBusyLen * 4 + 4;
    /* Will be set to TRUE only when no extra bytes were allocated */
    pWSPI->bUseTempBuf = FALSE; 
    pWSPI->fErr = 0;  
    pWSPI->hReport = hReport; 
    /* Save CB for the WSPI_ConfigureResetCb() */
    pWSPI->fCb = cb->CBFunc;
    pWSPI->pCb = cb->CBArg;

    /* Allocate the temporary buffer that will hold buffers with no extra room */
    if (pWSPI->uFixedBusyLen != 0)  
    {
        if ((pWSPI->pTempBuf = os_memoryAlloc (pWSPI->hOs, WSPI_NO_EXTRA_ALLOC_SIZE + pWSPI->uFixedBusyBytes)) == NULL)
        {
            return WSPI_ERR_ALLOC_MEM;
        }
    }
    }
    
	/* Allocate the temporary buffer that will hold buffers with no extra room */
	if ((pWSPI->pExtraFixedBusyBuf = os_memoryAlloc (pWSPI->hOs, WSPI_EXTRA_BUFFER_ALLOC_SIZE)) == NULL)
    {
        return WSPI_ERR_ALLOC_MEM;
    }
	pWSPI->ExtraBufLength = 0;

    /* This CMD is used to reset the HW SM */
    for (i = 0; i < WSPI_INIT_CMD_LEN; i++)
    {
        pWSPI->auInitCmd[i] = 0xff;
    }
    
    /* Write the reset CMD */
    pWSPI->returnStatus =  SPI_Write (pWSPI->hSPI, pWSPI->auInitCmd, WSPI_INIT_CMD_LEN, (request_callback_t)WSPI_ConfigureResetCb, hWSPI, 1);
    if (pWSPI->returnStatus == SPI_TXN_COMPLETE)
    {
		/* Note that in the next function call, pWSPI->returnStatus is going to be changed */
        WSPI_ConfigureResetCb(hWSPI, OK);
    }

    return (pWSPI->returnStatus == SPI_TXN_COMPLETE ? WSPI_TXN_COMPLETE : 
			(pWSPI->returnStatus == SPI_TXN_PENDING ? WSPI_TXN_PENDING : WSPI_ERR_UNKNOWN));
}

static void WSPI_ConfigureResetCb (TI_HANDLE hWSPI, int status)
{ 
    WSPI_t *pWSPI = (WSPI_t*)hWSPI;
    UINT8   auCRCBuffer [WSPI_INIT_CMD_CRC_LEN];

    /*
     * Set WSPI_INIT_COMMAND       
     * the data is being send from the MSB to LSB
     */
    pWSPI->auInitCmd[2] = 0xff;
    pWSPI->auInitCmd[3] = 0xff;
    pWSPI->auInitCmd[1] = WSPI_INIT_CMD_START | WSPI_INIT_CMD_TX;
    pWSPI->auInitCmd[0] = 0;
    pWSPI->auInitCmd[7] = 0;
    pWSPI->auInitCmd[6] = 
        (pWSPI->uConfigMask << 3) | (pWSPI->uFixedBusyLen & WSPI_INIT_CMD_FIXEDBUSY_LEN);
    pWSPI->auInitCmd[5] = 
        ((pWSPI->uFixedBusyLen == 0) ? 
            WSPI_INIT_CMD_DIS_FIXEDBUSY : 
            WSPI_INIT_CMD_EN_FIXEDBUSY) 
        | WSPI_INIT_CMD_IOD 
        | WSPI_INIT_CMD_IP 
        | WSPI_INIT_CMD_CS 
       #ifndef SPI_16_BIT
        | WSPI_INIT_CMD_WS
       #endif
        | WSPI_INIT_CMD_WSPI;

    auCRCBuffer[0] = pWSPI->auInitCmd[1];
    auCRCBuffer[1] = pWSPI->auInitCmd[0];
    auCRCBuffer[2] = pWSPI->auInitCmd[7];
    auCRCBuffer[3] = pWSPI->auInitCmd[6];
    auCRCBuffer[4] = pWSPI->auInitCmd[5]; 

    pWSPI->auInitCmd[4] = 
        (WSPI_GenerateCRC7 (auCRCBuffer, WSPI_INIT_CMD_CRC_INPUT_LEN) << 1) 
        | WSPI_INIT_CMD_END;   
   
    pWSPI->returnStatus = SPI_Write (pWSPI->hSPI, pWSPI->auInitCmd, WSPI_INIT_CMD_LEN, pWSPI->fCb, pWSPI->pCb, 1);
}


/*
 * ----------------------------------------------------------------------------
 * Function :   WSPI_ReadSync
 *
 * Input    :   void* hWSPI    - the WSPI handle
 *              UINT32 address - the address (in bytes) in the firmware to read the data from
 *              UINT32 length  - the number of bytes to read
 *              WSPI_CB_T cb   - callback parameters
 *              BOOL bMore     - more read/write transaction will follow   
 *
 * Output   :   UINT8* data - the buffer to put the read data
 *
 * ReturnVal:   one of the error codes (zero is success)
 *
 * Note(s)  :   1. set the WSPI cammand+address word
 *              2. send the WSPIcammand+address word
 *              3. read the fixed busy
 *              4. read the data
 *              5. check that the fixed busy was OK
 *
 *              because of the TNET state machine the data is always sent and only after 
 *              we read it we validate that it's OK
 * -----------------------------------------------------------------------------
 */
int WSPI_ReadSync (TI_HANDLE hWSPI, UINT32 address, UINT8* data, UINT32 length)
{
  #ifdef USE_SYNC_OVER_ASYNC
    
    WSPI_t    *pWSPI = (WSPI_t*)hWSPI;
    WSPI_CB_T  cb = { (request_callback_t)WSPI_SyncOverAsyncCb, hWSPI };
    UINT32        i, timeout;

    if (length == 0) 
    {
        return WSPI_ERR_WRONG_LENGTH;
    }
      
    /* Set the sync flag */
    pWSPI->bSyncFlag = 0;
        
    WSPI_ReadAsync (hWSPI, address, data, length, &cb, 1, 0);

    /* Wait to end of the asynchronous read */
    timeout = WSPI_SYNC_OVER_ASYNC_TIMEOUT * length;
    for (i = 0; i < timeout; i++)
    {
        if (pWSPI->bSyncFlag)
        {
            break;
        }
    }
    if (i == timeout)
    {
        /* Reached the timeout criteria without ending the asynchronous read */
        WLAN_REPORT_ERROR (pWSPI->hReport, HAL_HW_CTRL_MODULE_LOG, ("WSPI ASYNC READ DIDN'T FINISH\n")); 
                       
        return  WSPI_ERR_ASYNC_TIMEOUT;
    }

    return WSPI_OK;

  #else /* USE_SYNC_OVER_ASYNC */

    WSPI_t* pWSPI = (WSPI_t*)hWSPI;
    UINT32     i;
    UINT32  uFixedBusy;
    int     ret = WSPI_OK;

    if (length == 0) 
    {
        return WSPI_ERR_WRONG_LENGTH;
    }
         
    /*****************************/
    /* Write the command+address */
    /*****************************/
    pWSPI->uCmd = WSPI_CMD_READ;

    /* Set bFixedAddr */
    if (pWSPI->bFixedAddr)
    {
        pWSPI->uCmd |= WSPI_CMD_FIXED;   
    }
    
    /* Set length */
    pWSPI->uCmd |= (length << WSPI_CMD_BYTE_LENGTH_OFFSET) & WSPI_CMD_BYTE_LENGTH;
    /* Set address */
    pWSPI->uCmd |= address & WSPI_CMD_BYTE_ADDR;        
    
    /* Write command+address */
    ret |= SPI_WriteSync (pWSPI->hSPI, (UINT8*)&pWSPI->uCmd, WSPI_SIZEOF_UINT32);   

    if (pWSPI->uFixedBusyLen == 0)
    {
        /* 1150 */
        /* For 1150 read until we get the not-busy word */
        for (i = 0; i < WSPI_FIXED_BUSY_TIMEOUT; i++)  
        {
            ret |= SPI_ReadSync (pWSPI->hSPI, (UINT8*)&uFixedBusy, WSPI_SIZEOF_UINT32); 
            if ((uFixedBusy & 0x1) != 0)
            {
                break;
            }
        }
        
        if (i == WSPI_FIXED_BUSY_TIMEOUT)
        {
            WLAN_REPORT_ERROR (pWSPI->hReport, HAL_HW_CTRL_MODULE_LOG, ("WSPI FIXED BUSY (SYNC)\n")); 
                               
            return WSPI_ERR_BUS_BUSY;
        }
    }
    else
    {
        /* 1251 */
        /* For 1251 read the predefined number of busy words and at the end the not-busy word */  
        for (i = 0; i < pWSPI->uFixedBusyLen; i++) 
            ret |= SPI_ReadSync (pWSPI->hSPI, (UINT8*)&uFixedBusy, WSPI_SIZEOF_UINT32);
        ret |= SPI_ReadSync (pWSPI->hSPI, (UINT8*)&uFixedBusy, WSPI_SIZEOF_UINT32);
    }

    /* Read data */
    ret |= SPI_ReadSync (pWSPI->hSPI, data, length);

    if (pWSPI->uFixedBusyLen)
    { 
        /* 1251 */
        /* Check the fixed busy */
        if (!(uFixedBusy & 0x1))
        {
            WLAN_REPORT_ERROR (pWSPI->hReport, HAL_HW_CTRL_MODULE_LOG, ("WSPI FIXED BUSY (SYNC)\n")); 
                               
            ret = WSPI_ERR_BUS_BUSY;        
        }
    }

    return ret; 
  #endif
}


/*
 * ----------------------------------------------------------------------------
 * Function :   WSPI_ReadAsync
 *
 * Input    :   void* hWSPI    - the WSPI handle
 *              UINT32 address - the address (in bytes) in the firmware to read the data from
 *              UINT32 length  - the number of bytes to read
 *              WSPI_CB_T cb   - callback parameters
 *              BOOL bMore     - more read/write transaction will follow   
 *              BOOL bSpaceReserved 
 *                             - extra space padding has been reserved by an upper layer
 *
 * Output   :   UINT8* data    - the buffer to put the read data
 *
 * ReturnVal:   one of the error codes (zero is success)
 *
 * -----------------------------------------------------------------------------
 */
int WSPI_ReadAsyncOld (TI_HANDLE hWSPI, UINT32 address, UINT8* data, UINT32 length, WSPI_CB_T* cb, BOOL bMore, BOOL bSpaceReserved)
{
    WSPI_t       *pWSPI = (WSPI_t*)hWSPI;
    int           ret = WSPI_OK;

    if (length == 0) 
    {
        return WSPI_ERR_WRONG_LENGTH;
    }

    /*
     * Pass to the CB function of the read data also the fixed busy response so
     * to put all the required data in subsequent places in the buffer of the WSPI context 
     * first the fixed busy, second the CBFunc, third the CBArg
     * here we guarantee that the data will be sequential
     */  

    /*****************************/
    /* Write the command+address */
    /*****************************/
    pWSPI->uCmd = WSPI_CMD_READ;

    /* Set bFixedAddr */
    if (pWSPI->bFixedAddr)
    {
        pWSPI->uCmd |= WSPI_CMD_FIXED;   
    }
    
    /* Set the length */
    pWSPI->uCmd |= (length << WSPI_CMD_BYTE_LENGTH_OFFSET) & WSPI_CMD_BYTE_LENGTH;
    /* Set the address */
    pWSPI->uCmd |= address & WSPI_CMD_BYTE_ADDR;        
    
    /* Write command+address */
    ret |= SPI_WriteSync (pWSPI->hSPI, (UINT8*)&pWSPI->uCmd, WSPI_SIZEOF_UINT32);   

    if (pWSPI->uFixedBusyLen == 0)
    {
        UINT32 uFixedBusy, i;
        /* 1150 */
        /* For 1150 read until we get the not-busy word */
        for (i = 0; i < WSPI_FIXED_BUSY_TIMEOUT; i++)  
        {
            ret |= SPI_ReadSync (pWSPI->hSPI, (UINT8*)&uFixedBusy, WSPI_SIZEOF_UINT32);     
            if ((uFixedBusy & 0x1) != 0)
            {
                break;
            }
        }
        
        if (i == WSPI_FIXED_BUSY_TIMEOUT)
        {
            WLAN_REPORT_ERROR (pWSPI->hReport, HAL_HW_CTRL_MODULE_LOG, ("WSPI FIXED BUSY (ASYNC)\n")); 

            if (pWSPI->fErr) 
            { 
                (*pWSPI->fErr) (); 
            }

            return WSPI_ERR_BUS_BUSY;
        }
    }
    else
    {
        UINT32 uFixedBusy, i;
        /* 1251 */
        /* For 1251 read the predefined number of busy words and at the end the not-busy word */  
        for (i = 0; i < pWSPI->uFixedBusyLen; i++) 
            ret |= SPI_ReadSync (pWSPI->hSPI, (UINT8*)&uFixedBusy, WSPI_SIZEOF_UINT32);
        ret |= SPI_ReadSync (pWSPI->hSPI, (UINT8*)&pWSPI->uFixedBusy, WSPI_SIZEOF_UINT32);
    }  

    /* Read the data */
    pWSPI->fCb = cb->CBFunc;
    pWSPI->pCb = cb->CBArg;

    /* Tell the lower machine to start executing after this submission */  
    ret |= SPI_Read (pWSPI->hSPI, 
                     data + bSpaceReserved * pWSPI->uFixedBusyBytes, 
                     length, 
                     WSPI_ReadAsyncCb, 
                     pWSPI, 
                     1);
   
    return ret; 
}


/*
 * ----------------------------------------------------------------------------
 * Function :   WSPI_WriteSync
 *
 * Input    :   void* hWSPI - the WSPI handle
 *              UINT32 address - the address (in bytes) in the firmware to read the data from
 *              UINT8* data - the buffer that holds the data to write
 *              UINT32 length - the number of bytes to read
 *
 * Output   :   none
 *
 * ReturnVal:   one of the error codes (zero is success)
 *
 * Note(s)  :   1. set the WSPI cammand+address word
 *              2. send the WSPIcammand+address word
 *              3. send the data 
 *
 * -----------------------------------------------------------------------------
 */
int WSPI_WriteSync (TI_HANDLE hWSPI, UINT32 address, UINT8* data, UINT32 length)
{
  #ifdef USE_SYNC_OVER_ASYNC
    
    WSPI_t*   pWSPI = (WSPI_t*)hWSPI;
    WSPI_CB_T cb = { (request_callback_t)WSPI_SyncOverAsyncCb, hWSPI };
    int       i, timeout;

    if (length == 0) 
    {
        return WSPI_ERR_WRONG_LENGTH;
    }
        
    /* Set the sync flag */
    pWSPI->bSyncFlag = 0;
        
    WSPI_WriteAsync (hWSPI, address, data, length, &cb, 1, 0);

    /* Wait to end of the asynchronous write */
    timeout = WSPI_SYNC_OVER_ASYNC_TIMEOUT * length;
    for (i = 0; i < timeout; i++)
    {
        if (pWSPI->bSyncFlag)
        {
            break;
        }
    }
    if (i == timeout)
    {
        /* Reached the timeout criteria without ending the asynchronous write */
        WLAN_REPORT_ERROR (pWSPI->hReport, HAL_HW_CTRL_MODULE_LOG, ("WSPI ASYNC WRITE DIDN'T END\n")); 
                           
        return  WSPI_ERR_ASYNC_TIMEOUT;
    }

    return WSPI_OK;

  #else

    WSPI_t* pWSPI = (WSPI_t*)hWSPI;
    int     ret = WSPI_OK;
      
    if (length == 0) 
    {
        return WSPI_ERR_WRONG_LENGTH;
    }

    /*****************************/
    /* Write the command+address */
    /*****************************/
    pWSPI->uCmd = WSPI_CMD_WRITE;

    /* Set bFixedAddr */
    if (pWSPI->bFixedAddr)
    {
        pWSPI->uCmd |= WSPI_CMD_FIXED;
    }
        
    /* Set length */
    pWSPI->uCmd |= ((UINT32)length << WSPI_CMD_BYTE_LENGTH_OFFSET) & WSPI_CMD_BYTE_LENGTH;
    /* Set address */
    pWSPI->uCmd |= address & WSPI_CMD_BYTE_ADDR;        
    
    /* Write */
    ret |= SPI_WriteSync (pWSPI->hSPI, (UINT8*)&pWSPI->uCmd, WSPI_SIZEOF_UINT32);  

    /**************/
    /* Write data */
    /**************/
    ret |= SPI_WriteSync (pWSPI->hSPI, data, length);   
    
    return ret; 

  #endif
}


/*
 * ----------------------------------------------------------------------------
 * Function :   WSPI_ReadAsync
 *
 * Input    :   void* hWSPI    - the WSPI handle
 *              UINT32 address - the address (in bytes) in the firmware to read the data from
 *              UINT32 length  - the number of bytes to read
 *              WSPI_CB_T cb   - callback parameters
 *              BOOL bMore     - more read/write transaction will follow   
 *              BOOL bSpaceReserved 
 *                             - extra space padding has been reserved by an upper layer
 *
 * Output   :   UINT8* data    - the buffer to put the read data
 *
 * ReturnVal:   one of the error codes (zero is success)
 *
 * Note(s)  :   1. set the WSPI cammand+address word
 *              2. set the command+address request struct
 *              3. submit the request
 *              4. set the read fixed busy request struct
 *              5. submit the request
 *              6. set the read data request struct
 *              7. submit the request with execute flag
 *
 *              the fixed response is read into the buffer of the WSPI context
 * -----------------------------------------------------------------------------
 */
int WSPI_ReadAsync (TI_HANDLE hWSPI, UINT32 address, UINT8* data, UINT32 length, WSPI_CB_T* cb, BOOL bMore, BOOL bSpaceReserved)                   
{
    WSPI_t       *pWSPI = (WSPI_t*)hWSPI;
    int           status;
        
    if (length == 0) 
    {
        return WSPI_ERR_WRONG_LENGTH;
    }
    
    /* Save parameters */
    pWSPI->length = (UINT32)length; 
    pWSPI->data = data;
    pWSPI->bMore = bMore; 
    pWSPI->bSpaceReserved = bSpaceReserved;
    pWSPI->fCb = cb->CBFunc;
    pWSPI->pCb = cb->CBArg;

    /**********************************/
    /* Prepare the CMD for the SM use */
    /**********************************/
    pWSPI->uCmd = WSPI_CMD_READ;

    /* Set bFixedAddr */
    if (pWSPI->bFixedAddr)
    {
        pWSPI->uCmd |= WSPI_CMD_FIXED;   
    }
    
    /* Set length */
    pWSPI->uCmd |= ((UINT32)length << WSPI_CMD_BYTE_LENGTH_OFFSET) & WSPI_CMD_BYTE_LENGTH;
    /* Set address */
    pWSPI->uCmd |= address & WSPI_CMD_BYTE_ADDR;        
    
  #ifdef USE_WRITE_READ_API

    if (bSpaceReserved)
    {
        /* Indicate that that the temporary buffer is NOT used */
        pWSPI->bUseTempBuf = 0;
    }

    else
    {
        if (pWSPI->length <= WSPI_NO_EXTRA_ALLOC_SIZE)
        {   
            /* Indicate that that the temporary buffer is used */
            data = pWSPI->pTempBuf;
            pWSPI->bUseTempBuf = 1;
        }
        else
        {
            WLAN_REPORT_ERROR (pWSPI->hReport, HAL_HW_CTRL_MODULE_LOG, ("No space reserved for command\n"));
                               
            return WSPI_ERR_UNKNOWN;
        }
    }

    status = SPI_WriteRead (pWSPI->hSPI,
                            (UINT8*)&pWSPI->uCmd,
                            WSPI_SIZEOF_CMD,
                            data,
                            length + pWSPI->uFixedBusyBytes,
                            WSPI_ReadDataCb,
                            pWSPI,
                            bMore);

    /* In case of synchronous transaction completion call read callback explicitly */
    if (SPI_TXN_COMPLETE == status)
    {
        pWSPI->fCb = NULL;
        WSPI_ReadDataCb (hWSPI, status);
    }

    return status;

  #else

    /* Write command and indicate that more is ON */
    status = SPI_Write (pWSPI->hSPI, 
                        (UINT8*)&pWSPI->uCmd,
                        WSPI_SIZEOF_CMD,
                        WSPI_WriteCmdCb, 
                        hWSPI, 
                        1);  

    /* Don't pend, call directly to the callback */
    if (SPI_TXN_COMPLETE == status) 
    {
        WSPI_WriteCmdCb (hWSPI, WSPI_OK);
        /* WSPI_WriteCmdCb updates the pWSPI->status */
        return pWSPI->status;
    }

    return status;

  #endif
}


#ifndef USE_WRITE_READ_API
/*
 * ----------------------------------------------------------------------------
 * Function :   WSPI_WriteCmdCb
 *
 * Input    :   void* hWSPI - the WSPI handle
 *              int status  - SPI request status 
 *
 * Note(s)  :   CB from writing CMD in WSPI_ReadAsync().
 *              Read the data according to bSpaceReserved (indicates whether extra bytes were allocated
 *              in the buffer for the FixedBusyWord).
 *              if (!bSpaceReserved) but the length is short (WSPI_NO_EXTRA_ALLOC_SIZE) than copy it to 
 *              temporary file and read the data
 *              else - ERROR !
 * -----------------------------------------------------------------------------
 */
void WSPI_WriteCmdCb (TI_HANDLE hWSPI, int status)
{
    WSPI_t *pWSPI = (WSPI_t*)hWSPI;

    /* Extra room was saved - set the pointer as is */
    if (pWSPI->bSpaceReserved)
    {
        /* Indicate that that the temporary buffer is NOT used */
        pWSPI->bUseTempBuf = 0;

        /* Execute read with extra fixed busy bytes */  
        pWSPI->status = SPI_Read (pWSPI->hSPI, 
                                  pWSPI->data, 
                                  pWSPI->length + pWSPI->uFixedBusyBytes, 
                                  WSPI_ReadDataCb, 
                                  hWSPI,
                                  pWSPI->bMore); 
    }

    /* 
     * This case is used for handling buffers which have no extra room for the fixed busy words.
     * Use a temporary buffer and than copy the results            
     */
    else if (pWSPI->length <= WSPI_NO_EXTRA_ALLOC_SIZE)
    {   
        /* Indicate that that the temporary buffer is used */
        pWSPI->bUseTempBuf = 1;
        
        /* Read fixed busy words and a data in one transaction */ 
        pWSPI->status = SPI_Read (pWSPI->hSPI, 
                                  pWSPI->pTempBuf, 
                                  pWSPI->length + pWSPI->uFixedBusyBytes, 
                                  WSPI_ReadDataCb, 
                                  hWSPI,
                                  pWSPI->bMore); 
    }
    else    
    {
        WLAN_REPORT_ERROR (pWSPI->hReport, HAL_HW_CTRL_MODULE_LOG, ("No space reserved command\n"));                          
    }
    
    /* Check the return status; if not pending - call directly */
    if (SPI_TXN_COMPLETE == pWSPI->status)
    {   
        WSPI_ReadDataCb (hWSPI, WSPI_OK);
    }
}
#endif


/*
 * ----------------------------------------------------------------------------
 * Function :   WSPI_ReadDataCb
 *
 * Input    :   void* hWSPI - the WSPI handle
 *              int status  - SPI request status 
 *
 * Note(s)  :   CB from WSPI_WriteCmdCb().
 *              Copy the data if needed and call original CB
 * -----------------------------------------------------------------------------
 */
void WSPI_ReadDataCb (TI_HANDLE hWSPI, int status)
{
    WSPI_t *pWSPI = (WSPI_t*)hWSPI;
    UINT32 *pRbuf;

    /* If we are using the temp buffer and its fixedBusyWord is OK */
    pRbuf = (pWSPI->bUseTempBuf) ? (UINT32 *)pWSPI->pTempBuf 
                                 : (UINT32 *)pWSPI->data;

    /* No fixed busy */
    if ((pRbuf[pWSPI->uFixedBusyLen] & 0x1) != 0)
    {
        /* Copy data to the original buffer */
        if (pWSPI->bUseTempBuf)
        {
          #if WSPI_NO_EXTRA_ALLOC_SIZE == 4 /* used for optimization */
            /* In case the data size is 4 bytes copy it directly from the temp buffer */
            *((UINT32*)pWSPI->data) = pRbuf[pWSPI->uFixedBusyLen + 1];
          #else
            os_memoryCopy (pWSPI->hOs, pWSPI->data, &pRbuf[pWSPI->uFixedBusyLen + 1], pWSPI->length);            
          #endif 
        }

        /* Call user callback */
        if (pWSPI->fCb != NULL)
        {
            pWSPI->fCb (pWSPI->pCb, 0);
        } 
    }
    /* Handle fixed busy */
    else
    {
		WLAN_REPORT_WARNING (pWSPI->hReport, HAL_HW_CTRL_MODULE_LOG, 
			("WSPI FIXED BUSY (READ CALLBACK) Cmd = 0x%x, data pointer = %p data = 0x%x length = %d\n",
			pWSPI->uCmd, pRbuf, pRbuf[pWSPI->uFixedBusyLen + 1], pWSPI->length));

		WSPI_HandleFixedBusy(hWSPI, pRbuf);
    }       
}

/*
* ----------------------------------------------------------------------------
* Function :   WSPI_ReadDataAfterNotBusyCb
*
* Input    :   void* hWSPI - the WSPI handle
*              int status  - Not in use.
*
* Note(s)  :   Called from WSPI_HandleFixedBusy after the last read operation had the ~busy
*				word, but some of the data is missing
* -----------------------------------------------------------------------------
*/
void WSPI_ReadDataAfterNotBusyCb (TI_HANDLE hWSPI, int status)
{
	WSPI_t  *pWSPI = (WSPI_t*)hWSPI;
	UINT32 *pRbuf;
	UINT32 iExtraBufByteLength = pWSPI->ExtraBufLength*WSPI_SIZEOF_UINT32;
	UINT32 iTotalLength		   = pWSPI->length + pWSPI->uFixedBusyBytes;

	/* If we are using the temp buffer and its fixedBusyWord is OK */
	pRbuf = (pWSPI->bUseTempBuf) ? (UINT32 *)pWSPI->pTempBuf : (UINT32 *)pWSPI->data;

	if (pWSPI->bUseTempBuf)
	{
	   #if WSPI_NO_EXTRA_ALLOC_SIZE == 4 /* used for optimization */
		/* In case the data size is 4 bytes copy it directly from the temp buffer */
		*((UINT32*)pWSPI->data) = *((UINT32*)pWSPI->pExtraFixedBusyBuf);
	   #else
		os_memoryCopy (pWSPI->hOs, pWSPI->data, &(pRbuf[pWSPI->uFixedBusyLen + 1 + pWSPI->ExtraBufLength]), pWSPI->length - iExtraBufByteLength);            
		os_memoryCopy (pWSPI->hOs, &(pWSPI->data[pWSPI->length - iExtraBufByteLength]), pWSPI->pExtraFixedBusyBuf, iExtraBufByteLength);            
	   #endif 
	}
	else
	{
		os_memoryCopy(pWSPI->hOs, pRbuf,  &(pRbuf[pWSPI->ExtraBufLength]),iTotalLength - iExtraBufByteLength);
		pWSPI->data = (UINT8*)pRbuf;
		os_memoryCopy (pWSPI->hOs, &(pWSPI->data[iTotalLength - iExtraBufByteLength]), pWSPI->pExtraFixedBusyBuf, iExtraBufByteLength);            
	}

	/* Call user callback */
	if (pWSPI->fCb != NULL)
	{
		pWSPI->fCb (pWSPI->pCb, 0);
	}
}

/*
* ----------------------------------------------------------------------------
* Function :   WSPI_ReadNotBusyAndDataCb
*
* Input    :   void* hWSPI - the WSPI handle
*              int status  - Not in use.
*
* Note(s)  :   Called from WSPI_HandleFixedBusy after the last read operation had no response
*			    from the CHIP. i.e. the ~busy word is missing too
* -----------------------------------------------------------------------------
*/
void WSPI_ReadNotBusyAndDataCb (TI_HANDLE hWSPI, int status)
{
	WSPI_t  *pWSPI = (WSPI_t*)hWSPI;
	int i = 0;

	if ( pWSPI->bUseTempBuf )
	{
		while (i < (WSPI_EXTRA_READ_AFTER_NO_RESPONSE) )
		{
			/* Check if this is the ~busy word */
			if ( (*(UINT32*)(pWSPI->pExtraFixedBusyBuf + i)) & 0x1 )
			{	
				/* we have reached the fixed busy word */
			   #if WSPI_NO_EXTRA_ALLOC_SIZE == 4 /* used for optimization */
				/* In case the data size is 4 bytes copy it directly from the temp buffer */
				*((UINT32*)pWSPI->data) = *((UINT32*)(pWSPI->pExtraFixedBusyBuf+i+4));
			   #else
				os_memoryCopy (pWSPI->hOs, pWSPI->data, &(pWSPI->pTempBuf[pWSPI->uFixedBusyLen + 1 + pWSPI->ExtraBufLength]), pWSPI->length - iExtraBufByteLength);            
				os_memoryCopy (pWSPI->hOs, &(pWSPI->data[pWSPI->length - iExtraBufByteLength]), pWSPI->pExtraFixedBusyBuf, iExtraBufByteLength);            
			   #endif 

				if (pWSPI->fCb != NULL)
				{
					pWSPI->fCb (pWSPI->pCb, 0);
				}
				return;
			}
			i+=4;
		}
	}
	/* Oh boy, We couldn't find the ~Busy word */
	WLAN_REPORT_ERROR (pWSPI->hReport, HAL_HW_CTRL_MODULE_LOG, 
		("WSPI FIXED BUSY (READ CALLBACK) Cmd = 0x%x, length = %d\n",
		pWSPI->uCmd,pWSPI->length));

	if (pWSPI->fErr) 
	{
		(*pWSPI->fErr) ();
	}
	if (pWSPI->fCb != NULL)
	{
		pWSPI->fCb (pWSPI->pCb, WSPI_ERR_BUS_BUSY);
	}
}

/*
 * ----------------------------------------------------------------------------
 * Function :   WSPI_HandleFixedBusy
 *
 * Input    :   void* hWSPI - the WSPI handle
 *              UINT32 *pRbuf  - The temp read buffer 
 *
 * Note(s)  :   Called from WSPI_ReadDataCb() when a Fixed_Busy error accurse.
 *              Shifting the data to the expected place and reading the missing data
 * -----------------------------------------------------------------------------
 */
void WSPI_HandleFixedBusy(TI_HANDLE hWSPI, UINT32 *pRbuf)
{
	WSPI_t  *pWSPI = (WSPI_t*)hWSPI;
    UINT8   iTotalLength = pWSPI->length + pWSPI->uFixedBusyBytes;
	UINT8   iExtraBufByteLength = 0;
	int		status;

	pWSPI->ExtraBufLength = 0;

	/* Find the Data beginning */
	while (((pRbuf[pWSPI->uFixedBusyLen + pWSPI->ExtraBufLength] & 0x1) == 0) &&
		   ((pWSPI->uFixedBusyBytes + (pWSPI->ExtraBufLength * WSPI_SIZEOF_UINT32))  < iTotalLength) &&
		   ((pWSPI->ExtraBufLength * WSPI_SIZEOF_UINT32 ) < WSPI_EXTRA_BUFFER_ALLOC_SIZE )) 
	{
		pWSPI->ExtraBufLength++;
	}

	#ifdef TI_DBG
		if (pWSPI->ExtraBufLength > 0) 
		{
			DebugFixedBusy[pWSPI->ExtraBufLength - 1]++;
		}
	#endif
    
	/* if we had less then WSPI_EXTRA_BUFFER_ALLOC_SIZE FixedBusy words */
	if ((pRbuf[pWSPI->uFixedBusyLen + pWSPI->ExtraBufLength] & 0x1) != 0)
	{
		iExtraBufByteLength = pWSPI->ExtraBufLength*WSPI_SIZEOF_UINT32;

		/* Execute read with extra fixed busy bytes and without CMD*/  
		status = SPI_WriteRead(pWSPI->hSPI,
								NULL,
								0, 
								pWSPI->pExtraFixedBusyBuf, 
								iExtraBufByteLength, 
								WSPI_ReadDataAfterNotBusyCb,
								pWSPI,
								1);
        /* In case of synchronous transaction call the handling function implicitly */
		if (SPI_TXN_COMPLETE == status)
		{
			WSPI_ReadDataAfterNotBusyCb(hWSPI, OK);
		}
	}
	/* In the next case we try to read again even though the SPI line doesn't indicate any response  */
	else if ( pWSPI->bUseTempBuf )
	{	/* This case is handled for TempBuf only */ 
		/* Execute read without CMD */  
		status = SPI_WriteRead(pWSPI->hSPI,
								NULL,
								0, 
								pWSPI->pExtraFixedBusyBuf, 
								WSPI_EXTRA_READ_AFTER_NO_RESPONSE, 
								WSPI_ReadNotBusyAndDataCb,
								pWSPI,
								1);
		/* In case of synchronous transaction call the handling function implicitly */
		if (SPI_TXN_COMPLETE == status)
		{
			WSPI_ReadNotBusyAndDataCb(hWSPI, OK);
		}
	}
	else	/* Recovery from this one can be done by adjusting the length */
	{
		WLAN_REPORT_ERROR (pWSPI->hReport, HAL_HW_CTRL_MODULE_LOG, 
			("WSPI FIXED BUSY (READ CALLBACK) Cmd = 0x%x, data pointer = %p data = 0x%x length = %d\n",
			pWSPI->uCmd, pRbuf, pRbuf[pWSPI->uFixedBusyLen + 1],pWSPI->length));

        if (pWSPI->fErr) 
        {
            (*pWSPI->fErr) ();
        }

        if (pWSPI->fCb != NULL)
        {
            pWSPI->fCb (pWSPI->pCb, WSPI_ERR_BUS_BUSY);
        }
    }       
}


/*
 * ----------------------------------------------------------------------------
 * Function :   WSPI_WriteAsync
 *
 * Input    :   void* hWSPI    - the WSPI handle
 *              UINT32 address - the address (in bytes) in the firmware to read the data from
 *              UINT8* data    - the buffer that holds the data to write
 *              UINT32 length  - the number of bytes to read
 *              WSPI_CB_T cb   - callback parameters
 *              BOOL bMore     - more read/write transaction will follow   
 *              BOOL bSpaceReserved 
                               - extra space padding has been reserved by an upper layer
 *
 * Output   :   none
 *
 * ReturnVal:   WSPI_Status_e
 *
 * Note(s)  :   1. set the WSPI cammand+address word
 *              2. set the command+address request struct
 *              3. submit the request
 *              4. set the write data request struct
 *              5. submit the request with execute flag
 *
 * -----------------------------------------------------------------------------
 */
int WSPI_WriteAsyncOld (TI_HANDLE hWSPI, UINT32 address, UINT8* data, UINT32 length, WSPI_CB_T* cb, BOOL bMore, BOOL bSpaceReserved)
{
    WSPI_t       *pWSPI = (WSPI_t*) hWSPI;
    int           ret = WSPI_OK;

    if (length == 0) 
    {
        return WSPI_ERR_WRONG_LENGTH;
    }
        
    /*****************************/
    /* Write the command+address */
    /*****************************/
    pWSPI->uCmd = WSPI_CMD_WRITE;

    /* Set bFixedAddr */
    if (pWSPI->bFixedAddr)
    {
        pWSPI->uCmd |= WSPI_CMD_FIXED;
    } 
        
    /* Set the length */
    pWSPI->uCmd |= ((UINT32)length << WSPI_CMD_BYTE_LENGTH_OFFSET) & WSPI_CMD_BYTE_LENGTH;
    /* Set the address */
    pWSPI->uCmd |= address & WSPI_CMD_BYTE_ADDR;        
    
    /* Write */
    ret |= SPI_WriteSync (pWSPI->hSPI, (UINT8*)&pWSPI->uCmd, WSPI_SIZEOF_UINT32);  

    /******************/
    /* Write the data */
    /******************/

    /* Tell the lower machine to start executing after this submission */   
    ret |= SPI_Write (pWSPI->hSPI, 
                      data + bSpaceReserved * WSPI_SIZEOF_CMD, 
                      length, 
                      cb->CBFunc, 
                      cb->CBArg, 
                      1); 

    return ret; 
}


/*
 * ----------------------------------------------------------------------------
 * Function :   WSPI_WriteAsync
 *
 * Input    :   void* hWSPI    - the WSPI handle
 *              UINT32 address - the address (in bytes) in the firmware to read the data from
 *              UINT8* data    - the buffer that holds the data to write
 *              UINT32 length  - the number of bytes to read
 *              WSPI_CB_T cb   - callback parameters
 *              BOOL bMore     - more read/write transaction will follow   
 *              BOOL bSpaceReserved 
                               - extra space padding has been reserved by an upper layer
 *
 * Output   :   none
 *
 * ReturnVal:   WSPI_Status_e
 *
 * Note(s)  :   3 options are checked in this function:
 *              1) (bSpaceReserved == TRUE)
 *                  write the command and the data in one chunk
 *              2) else (length <= WSPI_NO_EXTRA_ALLOC_SIZE)
 *                  copy Command + data to tempBuffer and write it in one chunk
 *              3) else - error !!!
 *
 * -----------------------------------------------------------------------------
 */
int WSPI_WriteAsync (TI_HANDLE hWSPI, UINT32 address, UINT8* data, UINT32 length, WSPI_CB_T* cb, BOOL bMore, BOOL bSpaceReserved)                    
{
    WSPI_t       *pWSPI = (WSPI_t*) hWSPI;
    int           status;

    if (length == 0) 
    {
        return WSPI_ERR_WRONG_LENGTH;
    }

    /*****************************/
    /* Write the command+address */
    /*****************************/
    pWSPI->uCmd = WSPI_CMD_WRITE;

    /* Set bFixedAddr */
    if (pWSPI->bFixedAddr)
    {
        pWSPI->uCmd |= WSPI_CMD_FIXED;
    }
        
    /* Set the length */
    pWSPI->uCmd |= (length << WSPI_CMD_BYTE_LENGTH_OFFSET) & WSPI_CMD_BYTE_LENGTH;
    /* Set the address */
    pWSPI->uCmd |= address & WSPI_CMD_BYTE_ADDR;        

    if (bSpaceReserved)
    {
        /* If extra place was saved, transfer command and data in one transaction */
        *(UINT32*)data = pWSPI->uCmd;
        
         status = SPI_Write (pWSPI->hSPI, 
                             data, 
                             length + WSPI_SIZEOF_CMD, 
                             cb->CBFunc, 
                             cb->CBArg, 
                             bMore); 
    }

    /* If extra room was not saved for the command use temporary buffer */       
    else if (length <= WSPI_NO_EXTRA_ALLOC_SIZE)
    {
      #if WSPI_NO_EXTRA_ALLOC_SIZE == 4 /* used for optimization */
        /* Copy data to the temporary buffer */
        *((UINT32*)(pWSPI->pTempBuf + WSPI_SIZEOF_CMD)) = *((UINT32*)data);
      #else
        os_memoryCopy (pWSPI->hOs, pWSPI->pTempBuf + WSPI_SIZEOF_CMD, data, length);            
      #endif 
    
        /* Put the command at the beginning of the temporary buffer */
        *((UINT32*)pWSPI->pTempBuf) = pWSPI->uCmd;

        status = SPI_Write (pWSPI->hSPI, 
                            pWSPI->pTempBuf, 
                            length + WSPI_SIZEOF_CMD, 
                            cb->CBFunc, 
                            cb->CBArg, 
                            bMore); 
    }
    else 
    {   
        WLAN_REPORT_ERROR (pWSPI->hReport, HAL_HW_CTRL_MODULE_LOG, ("No space reserved for command\n"));
                           
        return WSPI_ERR_UNKNOWN;
    }

    return status; 
}


/*
 * ----------------------------------------------------------------------------
 * Function :   WSPI_GenerateCRC7
 *
 * Input    :   UINT8* bits - a string that holds the bits, in the order from the first to last
 *              UINT32 len - the number of bits
 *
 * Output   :   none
 *
 * ReturnVal:   UINT8 - the CRC word
 *
 * Note(s)  :   calculates the CRC7 for the WSPI init command
 * -----------------------------------------------------------------------------
 */
static UINT8 WSPI_GenerateCRC7 (UINT8* bits, UINT32 len)
{
    UINT8 CRC, CRC0, CRC1, CRC2, CRC3, CRC4, CRC5, CRC6;
    UINT8 bit, temp;
    UINT32   i;

    CRC0 = CRC1 = CRC2 = CRC3 = CRC4 = CRC5 = CRC6 = 0;

    /* Calculate the CRC with the formula : G(x) = x^7 + x^3 + 1 */
    for (i = 0; i < len; i++)
    {
        bit = (bits[i / 8] & (1 << (7 - (i % 8)))) >> (7 - (i % 8));
        temp = CRC6;        
        CRC6 = CRC5;
        CRC5 = CRC4;
        CRC4 = CRC3;
        CRC3 = CRC2;
        CRC2 = CRC1;
        CRC1 = CRC0;
        CRC0 = temp ^ bit;
        CRC3 = CRC0 ^ CRC3;
    }

    CRC = CRC0;
    CRC |= CRC1 << 1;
    CRC |= CRC2 << 2;
    CRC |= CRC3 << 3;
    CRC |= CRC4 << 4;
    CRC |= CRC5 << 5;
    CRC |= CRC6 << 6; 

    return CRC;
}


/*
 * ----------------------------------------------------------------------------
 * Function :   WSPI_ReadAsyncCb
 *
 * Input    :   void* handle - a handle that was passed to the asynchronous request (usually the WSPI handle)
 *              int status - the status of the SPI request
 *
 * Output   :   none
 *
 * ReturnVal:   none
 *
 * Note(s)  :   this is a temp function for passing as a CB function to the SPI layer for checking the async mechanism
 * -----------------------------------------------------------------------------
 */
static void WSPI_ReadAsyncCb (void* hWSPI, int status)
{
    WSPI_t *pWSPI = (WSPI_t*) hWSPI;
    /* Check the fixed busy word */
    if ((pWSPI->uFixedBusy & 0x1) != 0) 
    {
        pWSPI->fCb (pWSPI->pCb, 0);
    }
    else
    {
        WLAN_REPORT_ERROR (pWSPI->hReport, HAL_HW_CTRL_MODULE_LOG, ("WSPI FIXED BUSY (CALLBACK)\n"));

        if (pWSPI->fErr) 
        {
            (*pWSPI->fErr) ();
        }

        pWSPI->fCb (pWSPI->pCb, -1);
    }               
}


/*
 * ----------------------------------------------------------------------------
 * Function :   WSPI_SyncOverAsyncCb
 *
 * Input    :   void* handle - a handle that was passed to the asynchronous request
 *              int status - the status that the SPI request ended
 *
 * Output   :   none
 *
 * ReturnVal:   none
 *
 * Note(s)  :   this function just sets a flag so the sync SPI read/write function 
 *              can return
 * -----------------------------------------------------------------------------
 */
#ifdef USE_SYNC_OVER_ASYNC
static void WSPI_SyncOverAsyncCb (void* hWSPI, int status)
{
    WSPI_t* pWSPI = (WSPI_t*) hWSPI;

  #ifdef USE_SPI_DEBUG  
    {
        SPI_t *pSPI = (SPI_t*) pWSPI->hSPI;
        pSPI->stat.uNumOfSyncOverAsync ++;
    }
  #endif

    pWSPI->bSyncFlag = 1;
}
#endif


void WSPI_SetErrLog (void* hWSPI, void (*fErr)(void))
{
    WSPI_t* pWSPI = (WSPI_t*) hWSPI;

    pWSPI->fErr = fErr;
}

