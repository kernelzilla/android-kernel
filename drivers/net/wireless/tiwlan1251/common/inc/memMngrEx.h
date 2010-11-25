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

/**************************************************************************/
/*                                                                        */
/*	MODULE:		memMngr.h                                         */
/*      PURPOSE:        Driver memory management                          */
/*                                                                        */
/**************************************************************************/
#ifndef _MEM_MNGR_H_
#define _MEM_MNGR_H_

#include "osTIType.h"
#include "commonTypes.h"
#include "osApi.h"
#include "TNETW_Driver_types.h"

/*Ronnie: set # of MSDUs and BDs to be used in memMngrEx.c*/
#define		DEF_NUMBER_OF_MSDUS							200  /* Total number of packets queued in driver. */
#define		DEF_NUMBER_OF_BDS							400  /* Assuming typical MSDU uses 2 or 3 BDs. */

#define		MIN_NUMBER_OF_BUF_POOLS						1
#define		MAX_NUMBER_OF_BUF_POOLS						3

#ifdef SUPPORT_4X
#define		DEF_NUMBER_OF_BUF_POOLS						3
#define		DEF_BUFFER_LENGTH_POOL_1					64
#define		DEF_BUFFER_LENGTH_POOL_2					2048
#define		DEF_BUFFER_LENGTH_POOL_3					4096
#else
#define		DEF_NUMBER_OF_BUF_POOLS						3
#define		DEF_BUFFER_LENGTH_POOL_1					64
#define		DEF_BUFFER_LENGTH_POOL_2					256
#define		DEF_BUFFER_LENGTH_POOL_3					2048
#endif

#define		MIN_BUFFER_LENGTH							64
#define		MAX_BUFFER_LENGTH							4096
#define		DEF_NUMBER_OF_BUFFERS_IN_POOL_1				160
#define		DEF_NUMBER_OF_BUFFERS_IN_POOL_2				160
#define		DEF_NUMBER_OF_BUFFERS_IN_POOL_3				160


#define WLAN_DRV_NULL_MEM_HANDLE						0xffffffff 
	
#define NUM_OF_FREE_ARGS								5

#define MAX_NUM_OF_TIME_STAMPS                          8 

#define memMgr_BufLength(BufAddr) ( ((mem_BD_T *)BufAddr)->length   )
#define memMgr_BufOffset(BufAddr) ( ((mem_BD_T *)BufAddr)->dataOffset )
#define memMgr_BufData(BufAddr)   ( ((mem_BD_T *)BufAddr)->data )
#define memMgr_BufNext(BufAddr)   ( ((mem_BD_T *)BufAddr)->nextBDPtr )

#define memMgr_MsduHdrLen(MsduAddr)		( ((mem_MSDU_T *)MsduAddr)->headerLen )
#define memMgr_MsduFirstLen(MsduAddr)	( ((mem_MSDU_T *)MsduAddr)->firstBDPtr->length )
#define memMgr_MsduHandle(MsduAddr)		( ((mem_MSDU_T *)MsduAddr)->handle )
/*
 * Header resides after the Descriptor
 */
#define memMgr_MsduHdrAddr(MsduAddr)    ( memMgr_BufData(((mem_MSDU_T *)MsduAddr)->firstBDPtr) + \
                                          memMgr_BufOffset(((mem_MSDU_T *)MsduAddr)->firstBDPtr) + sizeof(DbTescriptor))

#define memMgr_MsduNextAddr(MsduAddr)	( ((mem_MSDU_T *)MsduAddr)->firstBDPtr->nextBDPtr )
#define memMgr_MsduDataAddr(MsduAddr)	( ((mem_MSDU_T *)MsduAddr)->firstBDPtr )
#define memMgr_MsduDataSize(MsduAddr)	( ((mem_MSDU_T *)MsduAddr)->dataLen )
#define memMgr_MsduNextGet(MsduAddr)	( ((mem_MSDU_T *)MsduAddr)->nextMSDUinList)
#define memMgr_MsduFreeFuncGet(MsduAddr)( ((mem_MSDU_T *)MsduAddr)->freeFunc)
#define memMgr_MsduFreeArg0Get(MsduAddr)( ((mem_MSDU_T *)MsduAddr)->freeArgs[0])
#define memMgr_MsduFreeArg1Get(MsduAddr)( ((mem_MSDU_T *)MsduAddr)->freeArgs[1])
#define memMgr_MsduFreeArg2Get(MsduAddr)( ((mem_MSDU_T *)MsduAddr)->freeArgs[2])
#define memMgr_MsduFreeArg3Get(MsduAddr)( ((mem_MSDU_T *)MsduAddr)->freeArgs[3])
#define memMgr_MsduFreeArg4Get(MsduAddr)( ((mem_MSDU_T *)MsduAddr)->freeArgs[4])

typedef enum
{
	/*
	 * Allocate on Tx
	 */
	MLME_MODULE		= 0,
	OS_ABS_TX_MODULE,
	RSN_MODULE,
	HAL_TX_MODULE,
	CONCAT_MODULE,
	DE_CONCAT_MODULE,
	ACK_EMUL_MODULE,
	TX_MODULE,
	MEASUREMENT_MODULE,
	SITE_MGR_MODULE,
	EXC_MANAGER_MODULE,
	TRACE_BUFFER_MODULE,
	ADM_CTRL_QOS_MODULE,
	CURRENT_BSS_MODULE,
	/*
	 * Allocate on Rx
	 */
	HAL_RX_MODULE,
	CORE_RX_MODULE,
	MLME_RX_MODULE,    
	OS_ABS_RX_MODULE, 
	RSN_RX_MODULE,  
	MEASUREMENT_RX_MODULE, 
	SITE_MGR_RX_MODULE,    
	EXC_MANAGER_RX_MODULE, 

	HAL_WEP1_RX,
	HAL_WEP2_RX,
	HAL_DEFRAG_RX,
	HAL_DUPLICA_RX,

	/*
    DO NOT TOUCH - MODULE_FREE_MSDU, MAX_NUMBER_OF_MODULE!
    */
	MODULE_FREE_MSDU,
    MAX_NUMBER_OF_MODULE

}allocatingModule_e;


typedef void (*ap_FreeMemFunc)(TI_HANDLE, TI_HANDLE, TI_STATUS);

#ifdef TNETW_MASTER_MODE
typedef void (*bd_FreeMemFunc)( UINT32 , UINT32, UINT32, UINT32, UINT32 );
#endif

typedef struct mem_DataBuf_T mem_DataBuf_T;
struct mem_DataBuf_T {
    /* READ ONLY */
	/* The user MUST not change the following fields */
	UINT32			handle;				/* Hanlde of this Data Buffer Structure */
    mem_DataBuf_T*  nextDataBuf;    /* pointer to the next free DataBuf
                                           when this DataBuf is in Free mode */
	UINT32			refCount;			/* number of instances of this Data Buf */
	/* PUBLIC - For the use of the User */
	UINT32			poolIndex;		/* the buffer pool index */
	UINT8			*data;			/* pointer to the Data */
#if defined TNETW_MASTER_MODE
	OS_PHYSICAL_ADDRESS	data_physical; /* Physical address of the data */
#endif
};

typedef struct mem_BD_T mem_BD_T;
struct mem_BD_T {
    /* READ ONLY */
	/* The user MUST not change the following fields */
	UINT32			handle;			/* Hanlde of this BD Data Structure */
	UINT32			refCount;		/* number of instances of this BD */
	mem_DataBuf_T*  dataBuf;        /* pointer to the Data Buffer */
	/* PUBLIC - For the use of the User */
    char*           data;           /* Pointer to the Data */
	UINT32			dataOffset;		/* offset of the data */
	UINT32			length;			/* Tx : the length of the entire data (including TxDescriptor,TNETWIF_WRITE_OFFSET_BYTES etc..) */
									/* Rx : the length of the data (excluding TNETWIF_READ_OFFSET_BYTES)							*/					
	mem_BD_T*		nextBDPtr;		/* pointer to the next BD */

#if defined TNETW_MASTER_MODE
	UINT32	data_physical_low;		/* Physical address (low) of the data */
    bd_FreeMemFunc	freeFunc;		/* pointer to the Data Buffer free function */
	UINT32			freeArgs[NUM_OF_FREE_ARGS];	/* arguments to be send with the free function */
#endif

};

typedef struct mem_MSDU_T mem_MSDU_T;
struct mem_MSDU_T {
    /* READ ONLY */
	/* The user MUST not change the following fields */
	UINT32				handle;			/* handle of this MSDU data structure */
	mem_MSDU_T *		nextFreeMSDU;   /* pointer to the next Free MSDU when
									   this MSDU Buffer is in Free mode */
	/* PUBLIC - For the use of the User */
    UINT32				headerLen;      /* the length of the 802.11 header */
	mem_BD_T *			firstBDPtr;		/* pointer to the first BD */
 	mem_BD_T *			lastBDPtr;		/* pointer to the last BD */
	ap_FreeMemFunc		freeFunc;		/* pointer to the Data Buffer free function */
	UINT32				freeArgs[NUM_OF_FREE_ARGS];	/* arguments to be send with the free function */
	UINT32				dataLen;		/* length of the data (only data) of the firstBDPtr */
	allocatingModule_e	module;			/* the allocating module */

	/* support Msdu List */
    mem_MSDU_T *		nextMSDUinList; /* pointer to the next MSDU in Tx queue link list. */
    mem_MSDU_T *		prevMSDUinList; /* pointer to the previos MSDU in Tx queue link list. */
	
	
	UINT32				txFlags;		/* Tx flags */
	UINT8				txCompleteFlags;		/* Tx complete flags */
	UINT32              insertionTime;  /* time of msdu insersion to driver. */
	UINT8               qosTag;         /* 802.11d qos tag */
#ifdef DM_USE_WORKQUEUE
    mem_MSDU_T *        msdu_next;      /* Used for Workqueue list */
#endif /* DM_USE_WORKQUEUE */
#ifdef TI_DBG
    UINT32              timeStamp [MAX_NUM_OF_TIME_STAMPS];   
                                        /* array of time stamps */ 
    UINT32              timeStampNum;   /* number of time stamps */ 
#endif
};

typedef struct
{
	UINT32			buffersSize;		/* the size of the buffers in the pool */
	UINT32			numFreeDataBuf;		/* number of free data buffers */
	UINT32			dataBufMaxNumber;	/* maximum number of buffers */
	mem_DataBuf_T* 	firstFreeDataBuf;	/* pointer to the first free Data Buffer */
	mem_DataBuf_T*	dataBufPool;		/* list of Data Buffers */
#ifdef TNETW_MASTER_MODE
	OS_PHYSICAL_ADDRESS	physicalDataBufPoolPtr;
#endif
	UINT8*				dataBufPoolPtr;

}buffersPool_t;

/* structures for initialization of Memory manager */
typedef struct
{
	UINT32	numOfbuffers;
	UINT32	buffersSize;
}bufPoolInit_t;

typedef struct
{
	UINT8	numOfPools;
	bufPoolInit_t	bufPoolInit[MAX_NUMBER_OF_BUF_POOLS];
}memMngrInit_t;

/* MemMngr Control Block */
typedef struct 
{
	TI_HANDLE		hReport;			/* report handle		*/
	TI_HANDLE		hOs;				/* Os handle			*/
	TI_HANDLE		hCriticalSectionProtect;

	UINT32			currentNumberOfPools;

	UINT32			msduMaxNumber;		/* maximum number of MSDUs */
	UINT32			bdMaxNumber;		/* maximum number of BD;s */

	mem_MSDU_T* 	msduPool;			/* list of MSDU Buffer Desciptors	*/
	mem_BD_T* 		bdPool;				/* list of BD Buffer Descriptors	*/

	mem_MSDU_T* 	firstFreeMSDU; 		/* pointer to the first free MSDU	*/
	mem_BD_T* 		firstFreeBD;		/* pointer to the first free BD		*/

	UINT32			numFreeMSDU;		/* number of free MSDU's */
	UINT32			numFreeBD;			/* number of free BD's */

	UINT32			moduleAllocCount[MAX_NUMBER_OF_MODULE]; /* counters of allocated */
															/* msdu per module		 */

	buffersPool_t	buffersPool[MAX_NUMBER_OF_BUF_POOLS];	/* Pools of Data Buffers */

}memMngr_t;

typedef struct 
{
	UINT32 numOfFreeBufPool1;
	UINT32 numOfFreeBufPool2;
	UINT32 numOfFreeBufPool3;
	UINT32 numOfFreeBDs;
	UINT32 numOfFreeMsdu;
}memMgrResources_t;

/*************************************************************************
 *                        wlan_memMngrInit          		             *
 *************************************************************************
DESCRIPTION: Init of the Memory Manager module

INPUT:
OUTPUT:
RETURN:      OK/NOK
**************************************************************************/
TI_HANDLE wlan_memMngrInit(TI_HANDLE hOs);

/*************************************************************************
 *                        wlan_memMngrDestroy                            *
 *************************************************************************
DESCRIPTION:



INPUT:      
OUTPUT:     

RETURN:     OK/NOK
**************************************************************************/
TI_STATUS wlan_memMngrDestroy(TI_HANDLE hMemMngr);

/*************************************************************************
 *                        wlan_memMngrConfigure                           *
 *************************************************************************
DESCRIPTION:



INPUT:      
OUTPUT:    

RETURN:     OK/NOK
**************************************************************************/
TI_STATUS wlan_memMngrConfigure(TI_HANDLE hMemMngr, TI_HANDLE hOs, TI_HANDLE hReport);

/*************************************************************************
 *                        wlan_memMngrAllocDataBuf                           *
 *************************************************************************
DESCRIPTION:This function allocates BDs and Data Buffers according to the
			required length. The memory manager will allocate the Data
			Buffers, update the buffer pointer in the BD structure and link
			the BDs when more than one Data Buffer is required.

INPUT:      len - the length of the required data buffer
OUTPUT:     BDPtr - a pointer in which this function will return a pointer
					to the allocated BD
RETURN:     OK/NOK
**************************************************************************/
TI_STATUS wlan_memMngrAllocDataBuf(TI_HANDLE hMemMngr, mem_BD_T** bdPtr, UINT32 len);

/*************************************************************************
 *                        wlan_memMngrAllocBDs                        		 *
 *************************************************************************
DESCRIPTION:This function allocates and returns a pointer to an array of BDs.
			This function does not allocate any memory buffers.

INPUT:      BDsNumber - number of required BDs
OUTPUT:     BDsPtr - a pointer in which this function will return a pointer
					 to an array of BD pointers
RETURN:     OK/NOK
**************************************************************************/
TI_STATUS wlan_memMngrAllocBDs(TI_HANDLE hMemMngr, UINT32 bdNumber, mem_BD_T** bdPtr);

/*************************************************************************
 *                        wlan_memMngrAllocMSDU                          *
 *************************************************************************
DESCRIPTION:This function allocates MPDU structure.

INPUT:		len - the length of the required data buffer
                    if len=0, than only MSDU buffer will be allocated
OUTPUT:     MSDUPtr - a pointer in which this function will return a pointer
					  to the MSDU structure
RETURN:     OK/NOK
**************************************************************************/
TI_STATUS wlan_memMngrAllocMSDU (TI_HANDLE hMemMngr, mem_MSDU_T** MSDUPtr,	
							  UINT32 len, allocatingModule_e module);

/*************************************************************************
 *                        wlan_memMngrAllocMSDUBufferOnly	             *
 *************************************************************************
DESCRIPTION:This function allocates MPDU structure - without Data Buffers

INPUT:
OUTPUT:     MSDUPtr - a pointer in which this function will return a pointer
					  to the MSDU structure
RETURN:     OK/NOK
**************************************************************************/
TI_STATUS wlan_memMngrAllocMSDUBufferOnly(TI_HANDLE hMemMngr, mem_MSDU_T** MSDUPtr, 
									   allocatingModule_e module);

/*************************************************************************
 *                        wlan_memMngrDuplicateMSDU                      *
 *************************************************************************
DESCRIPTION:This function duplicates the MSDU.

INPUT:      handle - handle of the MSDU the user want to duplicate
OUTPUT:     newHandle - pointer in which this function sets the handle of
                    the duplicated MSDU.
RETURN:     OK/NOK
**************************************************************************/
TI_STATUS wlan_memMngrDuplicateMSDU(TI_HANDLE hMemMngr, UINT32 handle, UINT32* newHandle);

/*************************************************************************
 *                        wlan_memMngrFreeMSDU         	                 *
 *************************************************************************
DESCRIPTION:Free MSDU structure. This function will free all BDs and Data
			Buffers that are bind to this MSDU.

INPUT:      handle - handle of this MSDU
OUTPUT:
RETURN:     OK/NOK
**************************************************************************/
TI_STATUS wlan_memMngrFreeMSDU(TI_HANDLE hMemMngr, UINT32 handle);

/*************************************************************************
 *														                 *
 *************************************************************************
DESCRIPTION:
INPUT:      
OUTPUT:
RETURN:     OK/NOK
**************************************************************************/
TI_STATUS wlan_memMngrFreeListOfMSDU(TI_HANDLE hMemMngr, UINT32 handle);


/*************************************************************************
 *                        wlan_memMngrFreeBD          	                 *
 *************************************************************************
DESCRIPTION:Free BD structure. This function will free a list of BD
			structures and the Data Buffer that is being pointed by these BD
			if any. (e.g. - free MPDU)

INPUT:      handle - handle of this BD
OUTPUT:
RETURN:     freeFlag - return TRUE if this BD list was freed
                       return FALSE if this BD list was not freed (refCount>0)
**************************************************************************/
UINT32 wlan_memMngrFreeBD(TI_HANDLE hMemMngr, UINT32 handle);

/*************************************************************************
 *                                  							         *
 *************************************************************************
DESCRIPTION:

INPUT:      
OUTPUT:
RETURN:     
**************************************************************************/
TI_STATUS wlan_memMngrFreeAllOsAlocatesBuffer(TI_HANDLE hMemMngr);

/*************************************************************************
 *                                  							         *
 *************************************************************************
DESCRIPTION:

INPUT:      
OUTPUT:
RETURN:     
**************************************************************************/
TI_STATUS wlan_memMngrCopyMsduFreeFunc(TI_HANDLE hMemMngr, UINT32 destMsduHandle, UINT32 sourceMsduHandle);

/*************************************************************************
 *                                  							         *
 *************************************************************************
DESCRIPTION:

INPUT:      
OUTPUT:
RETURN:     
**************************************************************************/
TI_STATUS wlan_memMngrGetMemMgrResources(TI_HANDLE hMemMngr, memMgrResources_t* memMgrResources);

/*************************************************************************
 *                                  							         *
 *************************************************************************
DESCRIPTION:

INPUT:      
OUTPUT:
RETURN:     
**************************************************************************/
TI_STATUS wlan_memMngrChangeMsduOwner(TI_HANDLE hMemMngr,allocatingModule_e newModule,mem_MSDU_T *pMsdu);


TI_STATUS wlan_memMngrSwapMsdu(TI_HANDLE hMemMngr, mem_MSDU_T *pMsdu_1, mem_MSDU_T *pMsdu_2);


TI_STATUS wlan_memMngrAddTimeStamp (TI_HANDLE hMemMngr, mem_MSDU_T *pMsdu);

/*************************************************************************
 *					          TEST_FUNCTIONS         	                 *
 *************************************************************************/
void memMngrPrintHandle(TI_HANDLE hMemMngr, UINT32 handle);
void memMngrFullPrint(TI_HANDLE hMemMngr);
void memMngrPrint(TI_HANDLE hMemMngr);

/*test function*/
TI_STATUS txDataSTUB_txSendMsdu(TI_HANDLE hMemMngr, mem_MSDU_T *pMsdu);
void print_MsduDataHeader(TI_HANDLE hMemMngr, mem_MSDU_T *pMsdu);
void memMngrPrintMSDUWithItsBds(mem_MSDU_T* pMsdu );


#endif
