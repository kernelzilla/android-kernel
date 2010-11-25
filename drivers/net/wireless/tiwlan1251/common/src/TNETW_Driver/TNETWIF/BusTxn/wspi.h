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
                                 
#ifndef _WSPI_H_
#define _WSPI_H_


/*
 * Define this flag to use write-read SPI driver API
 */
#define USE_WRITE_READ_API

/*
 * Define this flag to use sync-over-async mode
 */
#undef  USE_SYNC_OVER_ASYNC


/* Return codes */
typedef enum 
{
    WSPI_OK                =  0,
    WSPI_TXN_COMPLETE      =  0,
    WSPI_TXN_PENDING       =  1,
    WSPI_ERR_UNKNOWN       = -1,
    WSPI_ERR_BUS_BUSY      = -2,
    WSPI_ERR_QUEUE_FULL    = -3,
    WSPI_ERR_ALLOC_MEM     = -4,
    WSPI_ERR_ASYNC_TIMEOUT = -5, 
    WSPI_ERR_WRONG_LENGTH  = -6,

} WSPI_Status_e; 


typedef struct _WSPI_CB_t
{
    void (*CBFunc) (void *data, int status);
	void  *CBArg;

} WSPI_CB_T;


typedef struct _WSPIConfig_t
{
	int isFixedAddress;
	int fixedBusyLength;
	UINT8 mask;

} WSPIConfig_t;


/* WSPI API */
TI_HANDLE WSPI_Open       (TI_HANDLE hOs);
int       WSPI_Close      (TI_HANDLE hWSPI);
int       WSPI_Configure  (TI_HANDLE hWSPI, TI_HANDLE hReport, const WSPIConfig_t* aConfig, WSPI_CB_T* CB);
int       WSPI_ReadAsync  (TI_HANDLE hWSPI, UINT32 address, UINT8* data, UINT32 length, WSPI_CB_T* CB, BOOL bMore, BOOL bSpaceReserved);  
int       WSPI_WriteAsync (TI_HANDLE hWSPI, UINT32 address, UINT8* data, UINT32 length, WSPI_CB_T* CB, BOOL bMore, BOOL bSpaceReserved);
void 	  WSPI_HandleFixedBusy(TI_HANDLE hWSPI, UINT32 *pRbuf);
#ifdef USE_SYNC_API
int       WSPI_ReadSync   (TI_HANDLE hWSPI, UINT32 address, UINT8* data, UINT32 length);
int       WSPI_WriteSync  (TI_HANDLE hWSPI, UINT32 address, UINT8* data, UINT32 length);
#endif
void WSPI_SetErrLog (void* hWSPI, void (*fErr)(void));



/* Size of WSPI command */
#define WSPI_SIZEOF_CMD             4

/* Used to decide whether we want to use the temp buffer in order to read/write the data */ 
#define WSPI_NO_EXTRA_ALLOC_SIZE    4 

/* Init command length */
#define WSPI_INIT_CMD_LEN           8

/* Size of extra buffer size : 																			*/
/* NumOfExtraFixedBusy * WordLen + DataLost(beacuse of fixed busy) * WordLen  = 10 * 4 + 10 * 4  = 80	*/
#define WSPI_EXTRA_BUFFER_ALLOC_SIZE    80

/* Define the number of bytes to be read after FIXED BUSY error without the ~Busy word */
#define WSPI_EXTRA_READ_AFTER_NO_RESPONSE (28 - WSPI_NO_EXTRA_ALLOC_SIZE)

/* WSPI handle */
typedef struct _WSPI_t
{       
    TI_HANDLE           hOs;              /* OS wrapper */ 
    TI_HANDLE           hReport;          /* Report handler */ 
    TI_HANDLE           hSPI;             /* SPI driver handle */
    BOOL                bFixedAddr;       /* use fixed address */
    UINT32              uFixedBusyLen;    /* number of fixed busy cycles */
    UINT32              uFixedBusyBytes;  /* number of fixed busy bytes */ 
    UINT32              uFixedBusy;       /* fixed busy buffer */
    UINT8               uConfigMask;      /* configurable Mask for the Init CMD */
    UINT32              uCmd;             /* command word */
    UINT8               auInitCmd [WSPI_INIT_CMD_LEN]; 
                                          /* init command buffer */
    UINT8*              pTempBuf;         /* used for buffers with no extra room for command/fixed busy */
    UINT8*              data;             /* save the pointer to the data for read */
    UINT32              length;           /* save length of the buffer to be read */
    void              (*fCb) (void *data, int status);  
                                          /* callback function */          
    void               *pCb;              /* callback argument */
    WSPI_Status_e       status;           /* holds the current status */
    BOOL                bSpaceReserved;   /* extra room was saved for the fixed busy or command */
    BOOL                bMore;            /* indicate whether more use of the SPI is about to take place */
    BOOL                bUseTempBuf;      /* use temporary buffer */
    void              (*fErr) (void);     /* user debug error handler */ 

	UINT8*              pExtraFixedBusyBuf; 
										  /* used for a seconed SPI read */
										  /* for calls that returend with more FixedBusy words than allowed */
	UINT32              ExtraBufLength;   /* save length of the extra buffer to be read */
	int					returnStatus;	  /* return status for SPI_XXX functions. Note that return status is from type int */

  #ifdef USE_SYNC_OVER_ASYNC
    /*
     * NOTE: This flag MUST be volatile because its value is changed
     *       in the context of ISR. So, the compiler is prohibited
     *       to make register optimization inside the routine it polls for 
     */
    volatile int        bSyncFlag;
  #endif
   
} WSPI_t;


#endif /* _WSPI_H_ */

