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


#ifndef MMC_OMAP_API_H
#define MMC_OMAP_API_H

#ifdef __LINUX__
#include <asm/io.h>	/* include for SDIO Compilation */
#endif

/*
   SDIO_SUCCESS,
   SDIO_FAILURE - API return code (means success or failure of command invocation 
   in unblocking mode and success or failure of command completion in blocking mode).
   SDIO_BUSY - Busy means that bus is not avilable.
   SDIO_BAD_ALIGNMENT - Bad alignment means that data passed for DMA transaction 
   has been not aligned.
   SDIO_TIMEOUT - Timeout menas that attempt to acquire bus was failed.
   SDIO_BAD_INPUT_PARAMS - Bad input parameters means that parameters failed validation.
   SDIO_BAD_ADDRESS - Input peripheral address is not in valid range.
   SDIO_PACKET_SIZE_AND_MODE_INCONSISTENCY - Packet size inconsistency means that packet 
   size does not fit the transaction mode (synchronous or asynchronous).
   SDIO_CONTEXT_AND_MODE_INCONSISTENCY - Context inconsistency means that API has been 
   called from context that has some conflicts with API input parameters.
   SDIO_NON_AFFECTED - action was not failed, but it has no influence. 
*/
typedef enum {
	SDIO_SUCCESS=0,
	SDIO_FAILURE,
	SDIO_BUSY,
	SDIO_BAD_ALIGNMENT,
	SDIO_TIMEOUT,
	SDIO_BAD_INPUT_PARAMS,
	SDIO_BAD_PERIPHERAL_ADDRESS,
	SDIO_PACKET_SIZE_AND_MODE_INCONSISTENCY,
	SDIO_CONTEXT_AND_MODE_INCONSISTENCY,
	SDIO_NO_RESOURCES,
	SDIO_NON_AFFECTED,
} SDIO_Status;
 
/* Pointer to the SDIO memory-mapped address space */
typedef unsigned long SDIO_Address; 

/* Peripheral address space */
typedef unsigned long Peripheral_Address;

/* 
   SDIO_BufferLength: The length of the transaction. 
   In byte-basis operation, this length is the number of 
   bytes in the transaction. In block-basis operation, this 
   length is the number of blocks in the transaction.
*/
typedef unsigned int SDIO_BufferLength;
 
/* SDIO device driver activation states */
typedef enum {
	SDIO_Activation_InactiveState,    /* non initialized state */
	SDIO_Activation_InitializedState, /* state after initialization or after stop */
	SDIO_Activation_ActiveState,      /* state after start */
	SDIO_Activation_PowerSaveState,   /* to be defined */
	SDIO_Activation_EnchancedPowerSaveState,/* to be defined */
	SDIO_Activation_None,
} SDIO_ActivationState;

/* Bus statuses */
typedef enum {
	SDIO_Bus_Busy,           /* in processing */
	SDIO_Bus_Wait_Queue,     /* in processing - wait for the request from queue */
	SDIO_Bus_Wait_Async,	 /* in processing - wait for the asynchronous response */
	SDIO_Bus_Available,      /* available for use */
	SDIO_Bus_None,
} SDIO_BusStatus;
 
/* SDIO request statuses */
typedef enum {
	SDIO_Request_Completed = 0, /* last request has been successfully completed */
	SDIO_Request_InProgress,    /* last request is still in progress */
	SDIO_Request_FailedTimeOut, /* last request has been failed - timeout */
	SDIO_Request_FailedCRC,     /* last request has been failed - CRC */
	SDIO_Request_FailedSubmit,  /* last request has been failed - submit request */
	SDIO_Request_Rejected,	    /* last request has been rejected */
	SDIO_Request_ReadFailed,    /* last read request has been failed */
	SDIO_Request_WriteFailed,   /* last write request has been failed */
	SDIO_Request_None,          /* no request has been submitted */
} SDIO_RequestStatus;

/* 
   SDIO_RequestMode - how to proccess request.
   SDIO read or write transaction mode.
   OP Code 0 is used to read or write multiple bytes of 
   data to/from a single I/O register address. 
   This command is useful when I/O data is transferred 
   using a FIFO inside of the I/O device. In this case, 
   multiple bytes of data are transferred to/from a single 
   register address.  
   OP Code 1 is used to read or write multiple bytes of 
   data to/from an I/O register address that increment 
   by 1after each operation. This command is used used when 
   large amounts of I/O data exist within the I/O card in a 
   RAM like data buffer.
*/

typedef char SDIO_FunctionNumber; /* SDIO protocol function Id */
typedef unsigned int SDIO_RequestMode;
typedef unsigned int SDIO_BlockSize; /* Block size in case of SDIO is working in block mode */
typedef unsigned int SDIO_ReportLevel; /* Debug report level */
typedef unsigned int SDIO_DMAThreshold; /* Threshold with regards to DMA read and write operations */
typedef void* SDIO_Handle; /* SDIO handle as returned from init routine (SDIO_Init) */
typedef void* SDIO_Owner;  /* handle of the object called SDIO_Init */
typedef struct SDIO_Request SDIO_Request_t;
/* 
   User callback function for request completion notification 
*/
typedef void (*FNotify)(struct SDIO_Request *, int status);

/* 
   User callback function for request error notification 
*/
typedef void (*FError)(struct SDIO_Request *, int status);

/* Request external format */
struct SDIO_Request {
   u16 		 block_len;      /* Block length. Zero means byte mode */
   u8            *buffer;        /* Pointer to the data buffer. Data must be 
                                    contiguously allocated and write and release
                                    protected by application. */
   unsigned long physical_buffer;/* Physical address of the same buffer */
   u16           buffer_len;     /* Data buffer length in bytes */
   SDIO_Address  peripheral_addr;/* SDIO peripheral address */
   u32           acquire_window; /* Time out value. If not zero, means in case SDIO resources 
                                    are not available, try to acquire it within acquire_window
                                    time. Return with busy resources indication in case 
                                    of expiration. */
   u8		 enqueue_flag;   /* 0 - reject request if bus is not avaialable;
				    1 - put request into waiting list.
				    This field is relevant only for synchronous requests.
				    In case acquire_window is not zero - the attempt to wait
				    for free bus is to be performed and this flag is ignored. */
   SDIO_RequestMode mode;	 /* One of the following values: 
				    - MMC_DEV_BYTE_INCREMENTAL_MODE     
				    - MMC_DEV_BLOCK_INCREMENTAL_MODE  
				    - MMC_DEV_BYTE_FIXED_MODE    
				    - MMC_DEV_BLOCK_FIXED_MODE */ 
   SDIO_RequestStatus status;	 /* Request complition or progress status */
   SDIO_Handle 	 handle;	 /* SDIO object handle */
   SDIO_Owner 	 owner;	 	 /* SDIO object owner handle */
   void          *user;          /* Lower level handle */
   u8            access_flag;    /* 0 means write operation !0 - read */
   FNotify       fnotify;	 /* callback for request completion notification */
   FError        ferror;         /* callback for request error notification */
};

/* 
   Peripheral configuration function 
*/
typedef void Peripheral_ConfigParams;
typedef SDIO_Status (*FPeripheral)(SDIO_Handle, Peripheral_ConfigParams **);

/* 
   Convert peripheral adddress into 17 bits SDIO address.
   Default empty function is provided for the case when a caller 
   supplies already converted address.
*/
typedef SDIO_Address (*FConvert)(Peripheral_Address, SDIO_BufferLength);

/* Constants that are used for device driver configuration and tuning */
#define MMC_DEV_TIWIPP_CONFIG_SDIO1          0x00000001 /* use SDIO port 1 */
#define MMC_DEV_TIWIPP_CONFIG_SDIO2          0x00000002 /* use SDIO port 2 */
#define MMC_DEV_TIWIPP_CONFIG_SDDETECT_EN    0x00000004 /* auto detection is enable */
#define MMC_DEV_INIT_BE_MODE                 0x00000010 /* big endian mode */
#define MMC_DEV_INIT_LE_MODE                 0x00000020 /* little endian mode */
#define MMC_DEV_IRQ_NOTIFY_MODE              0x00000100 /* request notify in irq */
#define MMC_DEV_TASKLET_NOTIFY_MODE          0x00000200 /* request notify in tasklet */
#define MMC_DEV_SDIO_BUS_WIDTH_1             0x00001000 /* 1-bit bus mode */
#define MMC_DEV_SDIO_BUS_WIDTH_4             0x00002000 /* 4-bit bus mode */
#define MMC_DEV_CALLBACK_BLOCKING_MODE       0x00010000 /* callbacks in blocking mode */
#define MMC_DEV_CALLBACK_ASYNC_MODE          0x00020000 /* callbacks in asynchronous mode */
#define MMC_DEV_DMA_ONE_CHUNK_MODE           0x01000000 /* buffer size <= transaction */
#define MMC_DEV_DMA_MULTI_CHUNK_MODE         0x02000000 /* buffer size = n*transaction */
#define MMC_DEV_CONTIGUOUSE_MEMORY_MODE      0x10000000 /* memory convergence is required */
#define MMC_DEV_NON_CONTIGUOUSE_MEMORY_MODE  0x20000000 /* memory copy is required */
#define MMC_DEV_VIRTUAL_MEMORY_MODE  	     0x40000000 /* application passes virtual address */
#define MMC_DEV_PHYSICAL_MEMORY_MODE  	     0x80000000 /* application passes physical address*/
#define MMC_DEV_DEFAULT_CLOCK_RATE           24000000	/* clock rate value (24MHz) */
#define MMC_DEV_DEFAULT_AFL_SIZE	              2 /* 16-bit words used by DMA write */
#define MMC_DEV_DEFAULT_AEL_SIZE	              2 /* 16-bit words used by DMA read */
#define MMC_DEV_DEFAULT_VDD_VOLTAGE_WINDOW   0xFFFFC0   /* 0xFFFF00 is 2.0-3.6 volts window */
#define MMC_DEV_MAX_FIFO_SIZE                        64 /* controller size in bytes */
#define MMC_DEV_MIN_AFL_AEL_SIZE                      2 /* 2 16-bit words used by DMA */
/* Static configuration parameters.*/
typedef struct {
	SDIO_Owner owner;	       /* caller id */
	unsigned int config_flags;     /* combination of the above "mode" values */
	unsigned int clock_rate;       /* initial clock rate of SDIO bus in microseconds */
	unsigned int size_for_afl;     /* AFL of MMC_BUF register (number of 16-bit words) */
	unsigned int size_for_ael;     /* AEL of MMC_BUF register (number of 16-bit words) */
	unsigned int vdd_voltage_window; /* VDD voltage window - OCR values */
	/* User callback function for request completion notification */
	FNotify fnotify_read;/* callback for read request completion notification */
	FNotify fnotify_write;/* callback for write request completion notification */
	FError ferror; /* callback for request error notification */
	FPeripheral fconfig_peripheral; /* to be assigned to configuration peripheral target */
	FConvert fconvert; /* convert peripheral address to host address function */
} SDIO_ConfigParams;

/* Clock and DMA threshold values – should be tuned */
#define MMC_DEV_MASTER_CLOCK                48000000 /* deviser=MASTER_CLOCK/CLOCK_RATE */
#define MMC_DEV_DATA_ALIGNMENT_DEFAULT		   4
#define MMC_DEV_DATA_BLOCK_SIZE_DEFAULT		 512
#define MMC_DEV_WRITE_BYTE_DMA_THRESHOLD_DEFUALT   MMC_DEV_DEFAULT_AFL_SIZE
						   /* use DMA for packet size > threshold */
#define MMC_DEV_READ_BYTE_DMA_THRESHOLD_DEFUALT    MMC_DEV_DEFAULT_AEL_SIZE
						   /* use DMA for packet size > threshold */
#define MMC_DEV_FUNCTION_NUMBER_DEFUALT   	   1 /* default function code Id */
#define MMC_DEV_BLOCK_MODE                0x01000000 /* transaction size in blocks */
#define MMC_DEV_BYTE_MODE                 0x02000000 /* transaction size in bytes */
#define MMC_DEV_INCREMENTAL_ADDRESS       0x10000000 /* transaction with OP code 0 */
#define MMC_DEV_FIXED_ADDRESS             0x20000000 /* transaction with OP code 1 */
#define MMC_DEV_BYTE_INCREMENTAL_MODE     (MMC_DEV_BYTE_MODE|MMC_DEV_INCREMENTAL_ADDRESS)
#define MMC_DEV_BLOCK_INCREMENTAL_MODE    (MMC_DEV_BLOCK_MODE|MMC_DEV_INCREMENTAL_ADDRESS)
#define MMC_DEV_BYTE_FIXED_MODE           (MMC_DEV_BYTE_MODE|MMC_DEV_FIXED_ADDRESS)
#define MMC_DEV_BLOCK_FIXED_MODE          (MMC_DEV_BLOCK_MODE|MMC_DEV_FIXED_ADDRESS)
/* Dynamic configuration parameters */
typedef struct {
       unsigned int write_bytes_threshold;
       unsigned int read_bytes_threshold;
       char card_function_code; /* Card function code */
#ifdef CONFIG_MMC_DEBUG
       unsigned int report_level;
#endif
} SDIO_DynamicParams;

/* 
  This function initializes the SDIO host controller. 
  After the function is completed SDIO controller is 
  ready for SDIO interface activating.  
  The function creates SDIO object and returns a pointer to SDIO handler.
  This function does not communicate with SDIO controller.
*/
SDIO_Status SDIO_Init(SDIO_ConfigParams*, SDIO_Handle*);
 
/* 
  This function frees all resources and destroys SDIO object.
*/
SDIO_Status SDIO_Shutdown(SDIO_Handle);
 
/*
   This function activates configured setup for the SDIO device.
*/
SDIO_Status SDIO_Start(SDIO_Handle);
 
/*
   This function disconnects the host CPU from the slave SDIO device. 
   wait_window if non zero means to wait until last transaction is
   completed or until wait_window is expired and only after that 
   disconnect. 
   wait_window has to be always zero in case API is called from
   hardware interrupt context.
*/
SDIO_Status SDIO_Stop(SDIO_Handle, unsigned long wait_window);

/*
   This function re-activates configured setup for the SDIO device.
   This function can be used for recovery purpose.
   It performs the same set of actions like SDIO_Start with regard to 
   SDIO controller. In addition it reset the bus state to the initial 
   value, it drops a non-competed transaction if exist, it drops whole 
   SDIO request (in case the request is multi transaction), in case such 
   request is in progress.
*/
SDIO_Status SDIO_Reset(SDIO_Handle);
 
/*
   This function reads from SDIO interface. 
   The input address is SDIO_Address, which is a pointer to 
   the SDIO address space. The output address is local pointer in the virtual 
   address space. 
   The driver should convert the virtual address space to physical address 
   space in case of DMA operation.
   Read operation can't be processed in case SDIO interface is occupied by 
   other transaction. But in case acquire_window in input request is not zero,
   routine should try to acquire SDIO interface within acquire window.

   request->buf            - pointer to the data buffer. Data must be contiguously allocated 
                             and write and release protected by application.
   request->physical_buffer- physical address of the same buffer.
   request->buf_len        - data buffer length in bytes. It contains the number of 
                             bytes that is requested to be read.
                             The number of bytes that have been read within each transaction 
                             is to be returned (one request can contain several transactions).
   request->peripheral_addr- SDIO peripheral address.
   request->acquire_window - time out value. If not zero, means in case SDIO resources 
                             are not available, try to acquire it within acquire_window
                             time. Return with busy resources indication in case  of
                             expiration.
   request->enqueue_flag   - flag indicating whether to put or not syncronouse request into
			     device waiting list.
   request->mode           - flag that is a valid combination of the following constants:  
                             - MMC_DEV_BLOCK_MODE or MMC_DEV_BYTE_MODE;         
                             - MMC_DEV_INCREMENTAL_ADDRESS or MMC_DEV_FIXED_ADDRESS.
  
   Upon request completion, the result is to be returned to the request 
   originator.
*/
SDIO_Status SDIO_SyncRead(SDIO_Handle, SDIO_Request_t*);

/*
   The input address is a local pointer in the virtual address space.
   The output address is SDIO_Address, which is a pointer to the SDIO 
   address space.
   Write operation can't be processed in case SDIO interface is occupied by 
   other transaction. But in case acquire_window in input request is not zero,
   routine should try to acquire SDIO interface within acquire window.

   request->buf            - pointer to the data buffer. Data must be contiguously allocated 
                             and write and release protected by application. 
   request->physical_buffer- physical address of the same buffer. 
   request->buf_len        - data buffer length in bytes. It contains the number of 
                             bytes that is requested to be read.
                             The number of bytes that have been written within each transaction 
                             is to be returned (one request can contain several transactions).
   request->peripheral_addr- SDIO peripheral address.
   request->acquire_window - time out value. If not zero, means in case SDIO resources 
                             are not available, try to acquire it within acquire_window
                             time. Return with busy resources indication in case  of
                             expiration.
   request->enqueue_flag   - flag indicating whether to put or not syncronouse request into
			     device waiting list.
   request->mode           - flag that is a valid combination of the following constants:  
                             - MMC_DEV_BLOCK_MODE or MMC_DEV_BYTE_MODE;         
                             - MMC_DEV_INCREMENTAL_ADDRESS or MMC_DEV_FIXED_ADDRESS.
  
   Upon request completion, the result is to be returned to the request 
   originator.
*/
SDIO_Status SDIO_SyncWrite(SDIO_Handle, SDIO_Request_t*);

#ifdef CONFIG_ASYNC_API
/*
   This function reads from SDIO interface. 
   Read operation request is to be proceeded in a chunks of 
   pre-configured size (this size is set at the initialization time).
   Upon part of the request completion (up to one chunk), the result 
   is to be indicated through the related DMA callback.
   The request control data, like number of bytes or blocks that 
   should be read within transaction, bus status etc, are to be stored 
   within request template.
*/
SDIO_Status SDIO_AsyncRead(SDIO_Handle, SDIO_Request_t*);

/*
   This function writes to SDIO interface.
   Write operation request is to be proceeded in a chunks of 
   pre-configured size (this size is set at the initialization time).
   Upon part of the request completion (up to one chunk), the result is to 
   be indicated through the related DMA callback.
   The request control data, like number of bytes or blocks that should be 
   read within transaction, bus status etc, are to be stored within request 
   template.
*/
SDIO_Status SDIO_AsyncWrite(SDIO_Handle, SDIO_Request_t*);

/* The same as SDIO_SyncKillRequest, but for asynchronous mode.
   This function drops a non-competed transaction in case such transaction 
   is exist, it drops whole SDIO request, in case such request is in progress.
   A host communicating with SDIO device can abort data transfer just by 
   writing to the ASx bits in the CCCR. Normally, the abort is used to 
   stop an infinite block transfer (block count=0). If an exact number of 
   blocks to be transferred, it is recommended that the host issue a block 
   command with the correct block count, rather than using an infinite count 
   and aborting the data at the correct time.
   Additionally function runs user iterator function over each item from
   driver asynchronous request queue. 
 */
SDIO_Status SDIO_AsyncKillRequest(SDIO_Handle sdioHandle, void(*fiterator)(void*, int));

/* Go through asynchronous request queue and call fiterator function 
   for each non empty entry.
   Iterator function could be anything according to application needs: delete
   entry, modify entry, print entry etc from asynchronous request queue.
*/
SDIO_Status SDIO_AsyncIterateQueue(SDIO_Handle sdioHandle, void(*fiterator)(void*, int));
#endif /*CONFIG_ASYNC_API*/
 
/*
   Set activation mode - write it into sdio object control structure
*/
SDIO_Status SDIO_SetActivationState(SDIO_Handle, SDIO_ActivationState);

/*
   Get activation mode - read from sdio object control structure
*/
SDIO_ActivationState SDIO_GetActivationState(SDIO_Handle);
 
/*
   Set function number - write it into sdio object control structure
*/
SDIO_Status SDIO_SetFunctionNumber(SDIO_Handle, SDIO_FunctionNumber);

/*
   Get function number - read it from sdio object control structure
*/
SDIO_FunctionNumber SDIO_GetFunctionNumber(SDIO_Handle sdioHandle);
 
/*
   Set DMA threshold parameter for read transactions - write it into 
   SDIO object control structure
*/
SDIO_Status SDIO_SetDMAReadThreshold(SDIO_Handle, SDIO_DMAThreshold);

/*
   Set DMA threshold parameter for write transactions - write it into 
   SDIO object control structure
*/
SDIO_Status SDIO_SetDMAWriteThreshold(SDIO_Handle, SDIO_DMAThreshold);

/*
   Set sdio object debug report level - write it into sdio object control structure
*/
SDIO_Status SDIO_SetReportLevel(SDIO_Handle, SDIO_ReportLevel);

/*
   Set sdio object static parameters - write it into sdio object control 
   structure. Initialization time only.
*/
SDIO_Status SDIO_SetStaticParams(SDIO_Handle sdioHandle, SDIO_ConfigParams *staticParams);

/*
   Get sdio object static parameters.
   This function can be invoked at the initialization time only, it sets SDIO 
   static parameters:
   - sdio port (controller id);
   - Card auto detection mode;
   - Endian mode;
   - Device driving mode (interrupt, polling or blind);
   - Sdio bus width (1 or 4);
   - Callback mode (blocking or non-blocking);
   - Transfer mode (single or multi chunks);
   - DMA channel allocation method (dynamic or static);
   - Clock size.
   SDIO object static parameters setting will be updated after validation, 
   as following: object->static_params |= static_params.
*/
SDIO_ConfigParams *SDIO_GetStaticParams(SDIO_Handle sdioHandle);

/*
   Get sdio object dynamic parameters. 
   This function can be invoked at the run-time, it sets SDIO parameters, 
   for example:
   - Activation mode;
   - Function number;
   - Read DMA threshold;
   - Write DMA threshold;
   - Report level;
   - Transaction size (chunk);
   SDIO object dynamic parameters setting will be updated after validation, 
   as following: object->dynamic_params |= dynamic_params;
*/
SDIO_Status SDIO_SetDynamicParams(SDIO_Handle sdioHandle, SDIO_DynamicParams *dynamicParams);

 
/*
   Get sdio object dynamic parameters - write it into sdio 
   object control structure.
*/
SDIO_DynamicParams *SDIO_GetDynamicParams(SDIO_Handle sdioHandle);
 
/*
   This function read bus status from SDIO object and returns it to 
   the caller.
   Note that bus status returned to the caller could be different 
   from the real status, because this function does not run in a lock mode.
*/
SDIO_BusStatus SDIO_GetBusStatus(SDIO_Handle);
 
/*
   Set size of the block for the blocking mode.
   The SDIO device may transfer data in an optional block format. 
   The SDIO specification allows any block size from 1 byte to 2048 bytes
   (power of 2) in order to accommodate the various natural block sizes 
   for IO functions. Note that an SDIO card function may define a maximum 
   block size or byte count in the CIS that is smaller than the maximum 
   values described above.
*/
SDIO_Status SDIO_SetBlockSize(SDIO_Handle, SDIO_BlockSize);
 
/*
   This function can be used for recovery purpose.
   This function drops a non-competed transaction in case such transaction 
   is exist, it drops whole SDIO request, in case such request is in progress.
   A host communicating with SDIO device can abort data transfer just by 
   writing to the ASx bits in the CCCR. Normally, the abort is used to 
   stop an infinite block transfer (block count=0). If an exact number of 
   blocks to be transferred, it is recommended that the host issue a block 
   command with the correct block count, rather than using an infinite count 
   and aborting the data at the correct time.
*/
SDIO_Status SDIO_SyncKillRequest(SDIO_Handle);
 
/*
   Get status of the request executed last time.
*/
SDIO_RequestStatus SDIO_GetLastRequestStatus(SDIO_Handle);

#ifdef CONFIG_FORCE_API
/*
   Like SDIO_SyncRead and SDIO_SyncWrite, but with enforcement to perform
   transactions through DMA (only one chunck per transaction).
   Both APIs must be called only in case the length of the buffer fits
   the next condtition: 
   MMC_DEV_MAX_FIFO_SIZE <= length <= MMC_DEV_DATA_BLOCK_SIZE_DEFAULT
   For example:
   sync_read_request.buffer = dest_addr;
   sync_read_request.peripheral_addr = perph_addr;
   sync_read_request.buffer_len = length;
   if(lenght<MMC_DEV_MAX_FIFO_SIZE || length>MMC_DEV_DATA_BLOCK_SIZE_DEFAULT)
      status=SDIO_SyncRead(pHwAccess->hSdio,&sync_read_request);
   else
      status=SDIO_ForceRead(pHwAccess->hSdio,&sync_read_request);
   These APIs is faster then SDIO_SyncRead/SDIO_SyncWrite, but can be used
   only for synchronous mode.
*/
SDIO_Status SDIO_ForceRead(SDIO_Handle sdioHandle, SDIO_Request_t *req);
SDIO_Status SDIO_ForceWrite(SDIO_Handle sdioHandle, SDIO_Request_t *req);
#endif

#endif /* MMC_OMAP_API_H */









 
