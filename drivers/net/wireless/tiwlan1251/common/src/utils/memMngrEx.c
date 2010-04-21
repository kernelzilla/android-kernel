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
/*  MODULE:  memMngr.c                                                */
/*  PURPOSE: manage the SDRAM buffers for MSDU Data Buffers allocations   */
/*                                                                        */
/**************************************************************************/

#include "memMngrEx.h"
#include "osApi.h"
#include "report.h"


/*************************************************************************
*                        wlan_memMngrInit                                *
**************************************************************************
* DESCRIPTION:  Init of the Memory Manager module. This function allocated
*               all memroy resources needed for the MemMngr. It tallocate
*               a pool of Msdu structure, pool of Bd structure, and
*               number of pools of data buffers.
*               
* INPUT:        hOs - handle to Os abstraction layer
*
* OUTPUT:
*
* RETURN:       Handle to the allocated MemMngr control block
**************************************************************************/
TI_HANDLE wlan_memMngrInit(TI_HANDLE hOs)
{
    memMngr_t   *pMemMngr;
    UINT32      count,i;
    memMngrInit_t pMemMngrInit;

    if( hOs  == NULL )
    {
        WLAN_OS_REPORT(("wlan_memMngrInit() : FATAL ERROR: OS handle Error - Aborting\n"));
        return NULL;
    }

    /* structures for initialization of Memory manager */
    pMemMngrInit.numOfPools = DEF_NUMBER_OF_BUF_POOLS;
    pMemMngrInit.bufPoolInit[0].buffersSize = DEF_BUFFER_LENGTH_POOL_1;
    pMemMngrInit.bufPoolInit[0].numOfbuffers = DEF_NUMBER_OF_BUFFERS_IN_POOL_1;
    pMemMngrInit.bufPoolInit[1].buffersSize = DEF_BUFFER_LENGTH_POOL_2;
    pMemMngrInit.bufPoolInit[1].numOfbuffers = DEF_NUMBER_OF_BUFFERS_IN_POOL_2;
    pMemMngrInit.bufPoolInit[2].buffersSize = DEF_BUFFER_LENGTH_POOL_3;
    pMemMngrInit.bufPoolInit[2].numOfbuffers = DEF_NUMBER_OF_BUFFERS_IN_POOL_3;

    for( count = 0 ; count  < pMemMngrInit.numOfPools ; count++ )
    {
        if( pMemMngrInit.bufPoolInit[count].buffersSize > MAX_BUFFER_LENGTH ||
            pMemMngrInit.bufPoolInit[count].buffersSize < MIN_BUFFER_LENGTH)
        {
            WLAN_OS_REPORT(("wlan_memMngrInit() : FATAL ERROR: Buffer length out of range - Aborting\n"));
            return NULL;
        }
        if( count != 0 )
        {
            if(pMemMngrInit.bufPoolInit[count].buffersSize < pMemMngrInit.bufPoolInit[count-1].buffersSize )
            {
                WLAN_OS_REPORT(("wlan_memMngrInit() : FATAL ERROR: Buffer length's out of order - Aborting\n"));
                return NULL;
            }
        }
    }

    /* alocate MemMngr module control block */
    pMemMngr = os_memoryAlloc(hOs, (sizeof(memMngr_t)));
    if(!pMemMngr) {
        WLAN_OS_REPORT(("FATAL ERROR: Could not allocate pMemMngr - Aborting\n"));
        return NULL;
    }

    os_memoryZero(hOs, pMemMngr, sizeof(memMngr_t));

    pMemMngr->hOs = hOs;

    pMemMngr->msduMaxNumber = DEF_NUMBER_OF_MSDUS; 
    pMemMngr->bdMaxNumber = DEF_NUMBER_OF_BDS; 
    pMemMngr->numFreeMSDU = pMemMngr->msduMaxNumber;
    pMemMngr->numFreeBD = pMemMngr->bdMaxNumber;

    pMemMngr->msduPool = (mem_MSDU_T*)os_memoryCAlloc(hOs, pMemMngr->msduMaxNumber, sizeof(mem_MSDU_T));
    os_profile (hOs, 8, pMemMngr->msduMaxNumber * sizeof(mem_MSDU_T));

    if (pMemMngr->msduPool == NULL) 
    {
        wlan_memMngrDestroy(pMemMngr);
        WLAN_OS_REPORT(("FATAL ERROR: Could not allocate memory for MEM MNGR - Aborting\n"));
        return NULL;
    }

    pMemMngr->bdPool = (mem_BD_T*)os_memoryCAlloc(hOs, pMemMngr->bdMaxNumber, sizeof(mem_BD_T));
    os_profile (hOs, 8, pMemMngr->bdMaxNumber * sizeof(mem_BD_T));
    
    if (pMemMngr->bdPool == NULL) 
    {
        wlan_memMngrDestroy(pMemMngr);
        WLAN_OS_REPORT(("FATAL ERROR: Could not allocate memory for MEM MNGR - Aborting\n"));
        return NULL;
    }

    /* initialize buffer pools objects */
    pMemMngr->currentNumberOfPools = pMemMngrInit.numOfPools;
    for( count = 0 ; count  < pMemMngr->currentNumberOfPools  ; count++ )
    {
        pMemMngr->buffersPool[count].buffersSize = pMemMngrInit.bufPoolInit[count].buffersSize;

        pMemMngr->buffersPool[count].numFreeDataBuf = pMemMngrInit.bufPoolInit[count].numOfbuffers;
        
        pMemMngr->buffersPool[count].dataBufMaxNumber = pMemMngrInit.bufPoolInit[count].numOfbuffers;
        
        if((pMemMngr->buffersPool[count].dataBufPool = (mem_DataBuf_T*)os_memoryCAlloc(hOs, 
            pMemMngr->buffersPool[count].dataBufMaxNumber, sizeof(mem_DataBuf_T))) == NULL)
        {
            wlan_memMngrDestroy(pMemMngr);
            WLAN_OS_REPORT(("FATAL ERROR: Could not allocate buffer pools  for MEM MNGR - Aborting\n"));
            return NULL;
        }
        os_profile (hOs, 8, pMemMngr->buffersPool[count].dataBufMaxNumber * sizeof(mem_DataBuf_T));

        pMemMngr->buffersPool[count].firstFreeDataBuf = pMemMngr->buffersPool[count].dataBufPool;
        
        os_memoryZero(hOs, pMemMngr->buffersPool[count].dataBufPool, 
            (pMemMngr->buffersPool[count].numFreeDataBuf * sizeof(mem_DataBuf_T)));

#ifdef TNETW_MASTER_MODE
        if((pMemMngr->buffersPool[count].dataBufPoolPtr = (UINT8 *)os_memorySharedAlloc(hOs,
            pMemMngr->buffersPool[count].buffersSize * pMemMngr->buffersPool[count].dataBufMaxNumber,
            (void *)&pMemMngr->buffersPool[count].physicalDataBufPoolPtr)) == NULL)
        {
            wlan_memMngrDestroy(pMemMngr);
            WLAN_OS_REPORT(("FATAL ERROR: Could not allocate buffers for MEM MNGR (count=%d / %d, size=%d) - Aborting\n",
                count, pMemMngr->currentNumberOfPools, 
                pMemMngr->buffersPool[count].buffersSize * pMemMngr->buffersPool[count].dataBufMaxNumber));
            return NULL;
        }
#else
        if((pMemMngr->buffersPool[count].dataBufPoolPtr = (UINT8 *)os_memoryPreAlloc(hOs, count,
            pMemMngr->buffersPool[count].buffersSize * pMemMngr->buffersPool[count].dataBufMaxNumber)) == NULL)
        {
            wlan_memMngrDestroy(pMemMngr);
            WLAN_OS_REPORT(("FATAL ERROR: Could not allocate buffers for MEM MNGR - Aborting\n"));
            return NULL;
        }
#endif
        os_profile (hOs, 8, pMemMngr->buffersPool[count].buffersSize * pMemMngr->buffersPool[count].dataBufMaxNumber);

	/* alocate the buffers */
        for (i = 0; i < pMemMngr->buffersPool[count].dataBufMaxNumber; ++i) 
        {
#ifdef TNETW_MASTER_MODE
            pMemMngr->buffersPool[count].dataBufPool[i].data = (UINT8 *)
                (pMemMngr->buffersPool[count].dataBufPoolPtr
                + i*pMemMngr->buffersPool[count].buffersSize);

            pMemMngr->buffersPool[count].dataBufPool[i].data_physical.u.LowPart = (ULONG)
                (pMemMngr->buffersPool[count].physicalDataBufPoolPtr.u.LowPart + i*pMemMngr->buffersPool[count].buffersSize);
#else

            pMemMngr->buffersPool[count].dataBufPool[i].data = (UINT8 *)
                (pMemMngr->buffersPool[count].dataBufPoolPtr
                + i*pMemMngr->buffersPool[count].buffersSize);

#endif

            pMemMngr->buffersPool[count].dataBufPool[i].poolIndex = count;
        }
    }

    /* chain the items in each list */
    for (count = 0; count < pMemMngr->msduMaxNumber; ++count) {
        pMemMngr->msduPool[count].handle = count;
        if (count < pMemMngr->msduMaxNumber-1)  /* update next pointer except of the last one */
            pMemMngr->msduPool[count].nextFreeMSDU = &(pMemMngr->msduPool[count+1]);
    }
    for (count = 0; count < pMemMngr->bdMaxNumber; ++count) {
        pMemMngr->bdPool[count].handle = count;
        if (count < pMemMngr->bdMaxNumber-1)    /* update next pointer except of the last one */
            pMemMngr->bdPool[count].nextBDPtr = &(pMemMngr->bdPool[count+1]);
    }
    for (i = 0; i < pMemMngr->currentNumberOfPools; ++i) {
        for (count = 0; count < pMemMngr->buffersPool[i].dataBufMaxNumber; ++count) {
            pMemMngr->buffersPool[i].dataBufPool[count].handle = count;
            if (count < pMemMngr->buffersPool[i].dataBufMaxNumber-1)        /* update next pointer except of the last one */
                pMemMngr->buffersPool[i].dataBufPool[count].nextDataBuf = &(pMemMngr->buffersPool[i].dataBufPool[count+1]);
        }
    }

    /* assign a pointer for the start of each list */
    pMemMngr->firstFreeMSDU = pMemMngr->msduPool;
    pMemMngr->firstFreeBD = pMemMngr->bdPool;

    for(count=0 ; count < MAX_NUMBER_OF_MODULE; count++)
        pMemMngr->moduleAllocCount[count] = 0;

    if(( pMemMngr->hCriticalSectionProtect = os_protectCreate(hOs)) == NULL)
    {
        wlan_memMngrDestroy(pMemMngr);
        WLAN_OS_REPORT(("FATAL ERROR: Could not Create Critical Section Protection for MEM MNGR - Aborting\n"));
        return NULL;
    }

    return pMemMngr;
}
/***************************************************************************
*                       wlan_memMngrConfigure                              *
****************************************************************************
* DESCRIPTION:  This function configures MemMngr module     
* 
* INPUTS:       hMemMngr - The object 
*               hOs - Handle to the Os Abstraction Layer
*               hReport - Handle to the Report object
* OUTPUT:       
* 
* RETURNS:      OK - Configuration succesfull
*               NOK - Configuration unsuccesfull
***************************************************************************/
TI_STATUS wlan_memMngrConfigure(TI_HANDLE hMemMngr, TI_HANDLE hOs, TI_HANDLE hReport)
{
    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;

    pMemMngr->hReport = hReport;

    WLAN_REPORT_INIT(pMemMngr->hReport, MEM_MGR_MODULE_LOG,
        (".....MemMngr configured successfully\n"));

    return OK;
}
/***************************************************************************
*                           wlan_memMngrDestroy                            *
****************************************************************************
* DESCRIPTION:  This function unload the tMemMngr module. It first free  
*               the msdu pool, bd pool, data buffers pools and
*               then free the tMemMngr control block
* 
* INPUTS:       hMemMngr - the object
*       
* OUTPUT:       
* 
* RETURNS:      OK - Unload succesfull
*               NOK - Unload unsuccesfull
***************************************************************************/

TI_STATUS wlan_memMngrDestroy(TI_HANDLE hMemMngr)
{
    UINT32 count;
    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;

    /* Free Msdu pool */
    if(pMemMngr->msduPool)
    {
        os_memoryFree(pMemMngr->hOs, pMemMngr->msduPool,
            sizeof(mem_MSDU_T)*pMemMngr->msduMaxNumber); 
    }
    /* Free Bd pool */
    if(pMemMngr->bdPool)
    {
        os_memoryFree(pMemMngr->hOs, pMemMngr->bdPool,
            sizeof(mem_BD_T)*pMemMngr->bdMaxNumber); 
    }

    /* free data buf pools according to the number of pools */
    for( count = 0 ; count  < pMemMngr->currentNumberOfPools  ; count++ )
    {
#ifdef TNETW_MASTER_MODE
        if(pMemMngr->buffersPool[count].dataBufPoolPtr)
        {
            os_memorySharedFree(pMemMngr->hOs,pMemMngr->buffersPool[count].dataBufPoolPtr,
                pMemMngr->buffersPool[count].buffersSize*pMemMngr->buffersPool[count].dataBufMaxNumber,
                pMemMngr->buffersPool[count].physicalDataBufPoolPtr);
        }
#else
        if(pMemMngr->buffersPool[count].dataBufPoolPtr)
        {
            os_memoryFree(pMemMngr->hOs,pMemMngr->buffersPool[count].dataBufPoolPtr,
                pMemMngr->buffersPool[count].buffersSize*pMemMngr->buffersPool[count].dataBufMaxNumber);
        }
#endif

        if(pMemMngr->buffersPool[count].dataBufPool)
        {
            os_memoryFree(pMemMngr->hOs, pMemMngr->buffersPool[count].dataBufPool,
                sizeof(mem_DataBuf_T)*pMemMngr->buffersPool[count].dataBufMaxNumber); 
        }
    }

    /* free os_protect resources */
    if(pMemMngr->hCriticalSectionProtect)
        os_protectDestroy(pMemMngr->hOs,pMemMngr->hCriticalSectionProtect);

    /* free the MemMngr control block */
    os_memoryFree(pMemMngr->hOs, pMemMngr,sizeof(memMngr_t)); 

    return OK;
}

/*************************************************************************
*                        wlan_memMngrAllocDataBuf                        *
**************************************************************************
* DESCRIPTION:  This function allocates BDs and Data Buffers according 
*               to the required length. The memory manager will allocate 
*               the Data Buffers, update the buffer pointer in the BD 
*               structure and link the BDs when more than one Data 
*               Buffer is required. The Buffer length is selected that 
*               minimum beffer len will allocted.
*
* INPUT:        hMemMngr - the object
*               len - the length of the required data buffer
*
* OUTPUT:       BDPtr - a pointer in which this function will return 
*                   to the allocated BD
* 
*RETURN:        OK/NOK
**************************************************************************/
TI_STATUS wlan_memMngrAllocDataBuf(TI_HANDLE hMemMngr, mem_BD_T** bdPtr, UINT32 len)
{
    UINT32          poolIndex,count,dataBufNum;
    mem_BD_T*       allocBdTmp;         /* pointer to the current allocated BD in the new list */
    mem_DataBuf_T*  allocDataBufTmp;    /* pointer to the current allocated Data Buf */
    buffersPool_t*  tempBuffersPool;

    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;

    /* calculate the length and the number of Data Buffers we need allocate */
    for (poolIndex = 0; poolIndex < pMemMngr->currentNumberOfPools-1; poolIndex++) 
    {
        if(len < pMemMngr->buffersPool[poolIndex].buffersSize)
            break;
    }

    /* the selected buffer pool */
    tempBuffersPool = &pMemMngr->buffersPool[poolIndex];

    /* calculate the number of buffers needed */
    dataBufNum = (len / tempBuffersPool->buffersSize) + 1;

    os_protectLock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect); /* START OF CRITICAL SECTION */
    
    allocBdTmp = pMemMngr->firstFreeBD;
    *bdPtr = pMemMngr->firstFreeBD;

    allocDataBufTmp = tempBuffersPool->firstFreeDataBuf;

    /* check if we have enough memory - Data buffers (in the selected pool) and Bds */
    if ((pMemMngr->numFreeBD < dataBufNum) || (tempBuffersPool->numFreeDataBuf < dataBufNum)) 
    {
    	os_protectUnlock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect);  /* END OF CRITICAL SECTION */
        WLAN_REPORT_ERROR(pMemMngr->hReport, MEM_MGR_MODULE_LOG, 
            ("DB: mem_allocDataBuf: not enough memory numFreeBD=%d numFreeDataBuf=%d in Pool number=%d req DataBufs=%d\n",
                pMemMngr->numFreeBD, tempBuffersPool->numFreeDataBuf,poolIndex, dataBufNum));
        *bdPtr = NULL;
        return NOK;
    }

    /* update the pointers to the head of the list */
    for (count = 0 ; count < dataBufNum ; ++count) 
    {
        allocBdTmp->refCount = 1;
        allocBdTmp->dataBuf = allocDataBufTmp;
        allocBdTmp->data = (char*)(allocDataBufTmp->data);
#ifdef TNETW_MASTER_MODE
        allocBdTmp->data_physical_low = os_memoryGetPhysicalLow(allocDataBufTmp->data_physical);
#endif
        allocDataBufTmp->refCount = 1;
        allocBdTmp->length = tempBuffersPool->buffersSize;
        if (count == (dataBufNum-1)) 
        {
            /* the last BD in the allocated list */
            pMemMngr->firstFreeBD = allocBdTmp->nextBDPtr;
            tempBuffersPool->firstFreeDataBuf = allocDataBufTmp->nextDataBuf;
            allocBdTmp->nextBDPtr = NULL;
            allocDataBufTmp->nextDataBuf = NULL;
        }
        else 
        {
            allocBdTmp = allocBdTmp->nextBDPtr;
            allocDataBufTmp = allocDataBufTmp->nextDataBuf;
        }
    }

    /* update counter of free Bds and Data buffers */
    pMemMngr->numFreeBD -= dataBufNum;
    tempBuffersPool->numFreeDataBuf -= dataBufNum;

    os_protectUnlock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect);  /* END OF CRITICAL SECTION */

    return OK;
}


/*************************************************************************
*                         wlan_memMngrAllocBDs                           *
**************************************************************************
* DESCRIPTION:  This function allocates and returns a pointer to a link
*               list of BDs. This function allocates only Bds structure
*               and does not allocate any memory buffers.
*
* INPUT:        hMemMngr - The object
*               bdNumber - number of required BDs
*               
* OUTPUT:       bdPtr - a pointer in which this function will return 
*                    to the first Bd in the allocated list
*
* RETURN:     OK/NOK
**************************************************************************/
TI_STATUS wlan_memMngrAllocBDs(TI_HANDLE hMemMngr, UINT32 bdNumber, mem_BD_T** bdPtr)
{
    UINT32          count;
    mem_BD_T*       allocBdTmp; /* pointer to the current allocated BD in the new list */

    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;

    if (bdNumber == 0) 
    {
        *bdPtr = NULL;
        return NOK;
    }

    os_protectLock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect); /* START OF CRITICAL SECTION */
    allocBdTmp = pMemMngr->firstFreeBD;
    *bdPtr = pMemMngr->firstFreeBD;

    /* check if we have enough Bds */
    if (pMemMngr->numFreeBD < bdNumber) 
    {
        os_protectUnlock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect);  /* END OF CRITICAL SECTION */
        WLAN_REPORT_ERROR(pMemMngr->hReport, MEM_MGR_MODULE_LOG,  
            ("DB: wlan_memMngrAllocBDs: not enough memory\n"));
        *bdPtr = NULL;
        return NOK;
    }

    /* update the pointers to the head of the list */
    for (count = 0 ; count < bdNumber ; ++count) 
    {
        allocBdTmp->refCount = 1;
        if (count == (bdNumber-1)) 
        {
            /* the last bd in the allocated list */
            pMemMngr->firstFreeBD = allocBdTmp->nextBDPtr;
            allocBdTmp->nextBDPtr = NULL;
        }
        else 
        {
            allocBdTmp = allocBdTmp->nextBDPtr;
        }
    }

    /* update counter of free Bds  */
    pMemMngr->numFreeBD -= bdNumber;

    os_protectUnlock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect); /* END OF CRITICAL SECTION */

    return OK;
}


/*************************************************************************
*                        wlan_memMngrAllocMSDU                           *
**************************************************************************
* DESCRIPTION:  This function allocates MSDU structure with a number of
*               BDs and Data Buffers as required by 'len'.
*
* INPUT:        hMemMngr - The object
*               len - the length of the required data buffer
*                   if len=0, than only MSDU buffer will be allocated
*               module - the module that allocate this Msdu
*
* OUTPUT:       MSDUPtr - a pointer in which this function will 
*                   return to the allocated MSDU structure
*
* RETURN:     OK/NOK
**************************************************************************/
TI_STATUS wlan_memMngrAllocMSDU (TI_HANDLE hMemMngr, mem_MSDU_T** MSDUPtr,  
                              UINT32 len, allocatingModule_e module)
{
    UINT32      rc;
    mem_BD_T*   bdTmp;

    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;

    if (pMemMngr->msduPool == NULL)
    {
        /* object not initiated yet (!!!) */
        *MSDUPtr = NULL;
        WLAN_REPORT_ERROR(pMemMngr->hReport, MEM_MGR_MODULE_LOG,  
            ("wlan_memMngrAllocMSDU: failed!\n"));
        memMngrPrint(hMemMngr);
        return NOK;
    }

    if (len > 0) 
    {   
        /* we need to allocate BD and Data Buffers */
        rc = wlan_memMngrAllocDataBuf(hMemMngr,&bdTmp, len);
        if (rc == NOK)
        {
            *MSDUPtr = NULL;
            WLAN_REPORT_ERROR(pMemMngr->hReport, MEM_MGR_MODULE_LOG,  
                ("wlan_memMngrAllocMSDU: failed! no data bufs\n"));
            memMngrPrint(hMemMngr);
            return NOK;
        }
    }
    else 
    {
        /* len = 0 - need to allocate msdu structure only */
        rc = wlan_memMngrAllocMSDUBufferOnly(hMemMngr, MSDUPtr, module);
        if (rc == NOK)
        {
            *MSDUPtr = NULL;
            WLAN_REPORT_ERROR(pMemMngr->hReport, MEM_MGR_MODULE_LOG,  
                ("wlan_memMngrAllocMSDU: failed to alloc buffer only!\n"));
            memMngrPrint(hMemMngr);
            return NOK;
        }
        return OK;
        
    }

    os_protectLock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect);  /* START OF CRITICAL SECTION */
    
    /* check if we have enough free Msdu's */
     if (pMemMngr->firstFreeMSDU == NULL) 
    {
        /* no free MSDU buffers */
        WLAN_REPORT_ERROR(pMemMngr->hReport, MEM_MGR_MODULE_LOG,  
                ("wlan_memMngrAllocMSDU no free MSDU in MemMngr !!!\n"));
    	os_protectUnlock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect); /* END OF CRITICAL SECTION */
        memMngrPrint(hMemMngr);
        
        /* In case we dont have free msdu - free the allocated Bds */
        wlan_memMngrFreeBD(hMemMngr,bdTmp->handle);
        *MSDUPtr = NULL;
        return NOK;
    }

    *MSDUPtr = pMemMngr->firstFreeMSDU;
    pMemMngr->firstFreeMSDU = pMemMngr->firstFreeMSDU->nextFreeMSDU;
    pMemMngr->moduleAllocCount[module]++;

    /* update counter of free msdu's  */
    pMemMngr->numFreeMSDU--;

    os_protectUnlock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect); /* END OF CRITICAL SECTION */

    (*MSDUPtr)->nextFreeMSDU = NULL;
    (*MSDUPtr)->freeFunc = NULL;
    (*MSDUPtr)->firstBDPtr = bdTmp;
    (*MSDUPtr)->lastBDPtr = bdTmp;
    (*MSDUPtr)->dataLen = len;
    (*MSDUPtr)->nextMSDUinList = NULL; 
    (*MSDUPtr)->prevMSDUinList = NULL; 
    (*MSDUPtr)->txFlags = 0;        
    (*MSDUPtr)->txCompleteFlags = 0;
    (*MSDUPtr)->module = module;
  #ifdef TI_DBG
    (*MSDUPtr)->timeStampNum = 0;
  #endif

    return OK;
}

/*************************************************************************
*                 wlan_memMngrAllocMSDUBufferOnly                        *
**************************************************************************
* DESCRIPTION:  This function allocates MSDU structure - without 
*                   Bds and Data Buffers
*
* INPUT:        hMemMngr - The object
*
* OUTPUT:       MSDUPtr - a pointer in which this function will return 
*                     to the allocated MSDU structure
*               module - the module that allocate this Msdu
*
* RETURN:       OK/NOK
**************************************************************************/
TI_STATUS wlan_memMngrAllocMSDUBufferOnly(TI_HANDLE hMemMngr, mem_MSDU_T** MSDUPtr, allocatingModule_e module)
{
    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;

    os_protectLock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect); /* START OF CRITICAL SECTION */
    
    if (pMemMngr->firstFreeMSDU == NULL) 
    {
        /* no free MSDU buffers */
        WLAN_REPORT_ERROR(pMemMngr->hReport, MEM_MGR_MODULE_LOG,  
                ("wlan_memMngrAllocMSDUBufferOnly no free MSDU in MemMngr !!!\n"));
        os_protectUnlock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect); /* END OF CRITICAL SECTION */
        *MSDUPtr = NULL;
        return NOK;
    }

    *MSDUPtr = pMemMngr->firstFreeMSDU;
    pMemMngr->firstFreeMSDU = pMemMngr->firstFreeMSDU->nextFreeMSDU;
    pMemMngr->moduleAllocCount[module]++;

    /* update counter of free msdu's  */
    pMemMngr->numFreeMSDU--;

    os_protectUnlock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect);  /* END OF CRITICAL SECTION */

    (*MSDUPtr)->nextFreeMSDU = NULL;
    (*MSDUPtr)->freeFunc = NULL;
    (*MSDUPtr)->firstBDPtr = NULL;
    (*MSDUPtr)->lastBDPtr = NULL;
    (*MSDUPtr)->dataLen = 0;
    (*MSDUPtr)->nextMSDUinList = NULL; 
    (*MSDUPtr)->prevMSDUinList = NULL; 
    (*MSDUPtr)->txFlags = 0;
    (*MSDUPtr)->txCompleteFlags = 0;
    (*MSDUPtr)->module = module;
  #ifdef TI_DBG
    (*MSDUPtr)->timeStampNum = 0;
  #endif

    return OK;
}

/*************************************************************************
*                     wlan_memMngrFreeListOfMSDU                         *
**************************************************************************
* DESCRIPTION:  Free list of MSDUs structure. This function will run 
*               over the MSDU list (if exist) and free all MSDU's with 
*               all BDs and Data Buffers that are bind to this MSDU.
*
* INPUT:        hMemMngr - The object
*               handle - handle to the first MSDU in the list
*
* OUTPUT:       
*
* RETURN:       OK/NOK
**************************************************************************/
TI_STATUS wlan_memMngrFreeListOfMSDU(TI_HANDLE hMemMngr, UINT32 handle)
{
    mem_MSDU_T          *msduTmp,*nextTmpMsdu;  
    
    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;

    msduTmp = &(pMemMngr->msduPool[handle]);

    while (msduTmp != NULL) 
    {
        nextTmpMsdu = msduTmp->nextMSDUinList;
        if(wlan_memMngrFreeMSDU(hMemMngr,memMgr_MsduHandle(msduTmp)) != OK)
        {
            WLAN_REPORT_ERROR(pMemMngr->hReport, MEM_MGR_MODULE_LOG,  
                ("wlan_memMngrFreeListOfMSDU This MSDU is already free\n"));
		    //os_protectUnlock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect); /* END OF CRITICAL SECTION */
            return NOK;
        }
        msduTmp = nextTmpMsdu;
    }

    return OK;
}

/*************************************************************************
*                        wlan_memMngrFreeMSDU                            *
**************************************************************************
* DESCRIPTION:  Free ONE MSDU structure. This function will free all 
*               BDs and Data Buffers that are bind to this MSDU.
*
* INPUT:        hMemMngr - The object
*               handle - handle of the MSDU
*
* OUTPUT:
*
* RETURN:       OK/NOK
**************************************************************************/
TI_STATUS wlan_memMngrFreeMSDU(TI_HANDLE hMemMngr, UINT32 handle)
{
    UINT32 freeFlag;
    ap_FreeMemFunc      freeFunc = NULL;        /* pointer to the Data Buffer free function */
    UINT32              freeArgs[NUM_OF_FREE_ARGS]; /* arguments to be send with the free function */
    int i;

    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;

    if( handle == WLAN_DRV_NULL_MEM_HANDLE )
        return OK;

    /* check if the msdu is already free */
    if(pMemMngr->msduPool[handle].module == MODULE_FREE_MSDU)
    {
        WLAN_REPORT_ERROR(pMemMngr->hReport, MEM_MGR_MODULE_LOG,  
                ("wlan_memMngrFreeMSDU This MSDU is already free\n"));
        return NOK;

    }

    if (pMemMngr->msduPool[handle].firstBDPtr != NULL) 
    {
        /* free all BDs and Data Buffers */
        freeFlag = wlan_memMngrFreeBD(hMemMngr, pMemMngr->msduPool[handle].firstBDPtr->handle);

        if ((freeFlag == TRUE) && (pMemMngr->msduPool[handle].freeFunc != NULL)) 
        {
            /* save the free parameters to do it at the end of the function */
            freeFunc = pMemMngr->msduPool[handle].freeFunc;
            for (i=0; i<NUM_OF_FREE_ARGS; i++)
                freeArgs[i] = pMemMngr->msduPool[handle].freeArgs[i];
        }
    }

    os_protectLock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect);  /* START OF CRITICAL SECTION */

    /* reset the fields of the MSDU buffer */
    pMemMngr->msduPool[handle].firstBDPtr = NULL;
    pMemMngr->msduPool[handle].freeFunc = NULL;
    pMemMngr->msduPool[handle].freeArgs[0] = 0;
    pMemMngr->msduPool[handle].freeArgs[1] = 0;
    pMemMngr->msduPool[handle].freeArgs[2] = 0;
    pMemMngr->msduPool[handle].dataLen = 0;
    pMemMngr->msduPool[handle].headerLen = 0;
    pMemMngr->msduPool[handle].txFlags = 0;
    pMemMngr->msduPool[handle].txCompleteFlags = 0;
    pMemMngr->msduPool[handle].nextMSDUinList = 0;
    pMemMngr->msduPool[handle].prevMSDUinList = 0;
    pMemMngr->numFreeMSDU++;

    pMemMngr->moduleAllocCount[pMemMngr->msduPool[handle].module]--;
    
    pMemMngr->msduPool[handle].module = MODULE_FREE_MSDU;

    /* add the MSDU to the free MSDU list */
    pMemMngr->msduPool[handle].nextFreeMSDU = pMemMngr->firstFreeMSDU;
    pMemMngr->firstFreeMSDU = &(pMemMngr->msduPool[handle]);

    os_protectUnlock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect); /* END OF CRITICAL SECTION */

    /* !!!!!!!! The free should be only after os_protectUnlock !!!!!!!! */
    if (freeFunc != NULL) 
    {
        /* call free function */
        freeFunc((TI_HANDLE)(freeArgs[0]),
                 (TI_HANDLE)(freeArgs[1]),
                 (TI_STATUS)(freeArgs[2]));
    }

    return OK;
}


/*************************************************************************
*                            allocDataBuf                                *
**************************************************************************
* DESCRIPTION:  Allocate Data Buffer
* 
* INPUT:        hMemMngr - The object
*               dataBuf - pointer to the new allocated Data Buffer
*               poolIndex - The index of the pool to allocate from
*
* OUTPUT:
*
* RETURN:       OK/NOK
**************************************************************************/
#if 0
static TI_STATUS allocDataBuf(TI_HANDLE hMemMngr, mem_DataBuf_T* dataBuf, UINT32 poolIndex)
{
    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;

    os_protectLock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect); /* START OF CRITICAL SECTION */
    if (pMemMngr->buffersPool[poolIndex].firstFreeDataBuf == NULL) {
        os_protectUnlock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect);/* END OF CRITICAL SECTION */
        return NOK;
    }
    dataBuf = pMemMngr->buffersPool[poolIndex].firstFreeDataBuf;
    pMemMngr->buffersPool[poolIndex].firstFreeDataBuf = pMemMngr->buffersPool[poolIndex].firstFreeDataBuf->nextDataBuf;
    pMemMngr->buffersPool[poolIndex].numFreeDataBuf--;
    os_protectUnlock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect);/* END OF CRITICAL SECTION */

    return OK;
}
#endif


/*************************************************************************
*                          freeDataBuf                                   *
**************************************************************************
* DESCRIPTION:  Free Data Buffer. 
*
* INPUT:        hMemMngr - The object
*               dataBuf - pointer to the Data Buffer
*
* OUTPUT:
*
* RETURN:       OK/NOK
**************************************************************************/
static TI_STATUS freeDataBuf(TI_HANDLE hMemMngr, mem_DataBuf_T* dataBuf)
{
    buffersPool_t   *tempBuffersPool;
    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;

    if (dataBuf->refCount == 0) {
        WLAN_REPORT_ERROR(pMemMngr->hReport, MEM_MGR_MODULE_LOG,  
            ("DB: freeDataBuf FATAL ERROR: dataBuf->refCount < 0\n"));
        return NOK;
    }

    if (--(dataBuf->refCount) == 0) {
        tempBuffersPool = &pMemMngr->buffersPool[dataBuf->poolIndex];
        /* add this Data Buffer to the free list of the correct pool*/
        dataBuf->nextDataBuf = tempBuffersPool->firstFreeDataBuf;
        tempBuffersPool->firstFreeDataBuf = dataBuf;
        tempBuffersPool->numFreeDataBuf++;
    }

    return OK;
}


/*************************************************************************
*                        wlan_memMngrFreeBD                              *
**************************************************************************
* DESCRIPTION:  Free BD structure. This function will free a list of BD
*               structures and the Data Buffer that is being pointed by 
*               these BD if any.
*
* INPUT:        hMemMngr - The object
*               handle - handle of this BD
* OUTPUT:
* RETURN:       freeFlag - return TRUE if this BD list was freed
*                   return FALSE if this BD list was not freed (refCount>0)
**************************************************************************/
UINT32 wlan_memMngrFreeBD(TI_HANDLE hMemMngr, UINT32 handle)
{
    UINT32              rc = FALSE;
    mem_DataBuf_T*      dataBuf;
    mem_BD_T*           bdTmp;     /* pointer to the current BD we need to free */
    mem_BD_T*           nextBdTmp; /* pointer to the next BD we need to free    */

    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;

    bdTmp = &(pMemMngr->bdPool[handle]);

    os_protectLock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect); /* START OF CRITICAL SECTION */    
    
    while (bdTmp != NULL) 
    {
        dataBuf = bdTmp->dataBuf;
        nextBdTmp = bdTmp->nextBDPtr;
        if (bdTmp->refCount == 0) 
        {
            WLAN_REPORT_ERROR(pMemMngr->hReport, MEM_MGR_MODULE_LOG,  
                ("DB: wlan_memMngrFreeBD FATAL ERROR: bdTmp->refCount < 0\n"));
            os_protectUnlock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect);/* END OF CRITICAL SECTION */
            return FALSE;
        }
        if (dataBuf != NULL) 
        {
            freeDataBuf(hMemMngr, dataBuf);
        }

#ifdef TNETW_MASTER_MODE
        if( bdTmp->freeFunc != NULL)
        {
            bdTmp->freeFunc(  bdTmp->freeArgs[0], bdTmp->freeArgs[1], bdTmp->freeArgs[2],
                              bdTmp->freeArgs[3], bdTmp->freeArgs[4]);
        }
#endif

        if (--(bdTmp->refCount) == 0) 
        {
            bdTmp->dataBuf = NULL;
            bdTmp->data = NULL;

#ifdef TNETW_MASTER_MODE
            bdTmp->data_physical_low = 0;
            bdTmp->freeFunc     = NULL;
            os_memoryZero(pMemMngr->hOs, bdTmp->freeArgs, sizeof(UINT32)*NUM_OF_FREE_ARGS);
#endif
            bdTmp->dataOffset = 0;
            bdTmp->length = 0; 
            /* adding the free BD to the free BD list */
            bdTmp->nextBDPtr = pMemMngr->firstFreeBD;
            pMemMngr->firstFreeBD = bdTmp;
            pMemMngr->numFreeBD++;
        }
        if (nextBdTmp == NULL)
        {
            if (bdTmp->refCount <= 0)
            {
                rc = TRUE;
            }
        }
        bdTmp = nextBdTmp;
    }

    os_protectUnlock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect); /* END OF CRITICAL SECTION */
    return rc;
}


/*************************************************************************
*                wlan_memMngrFreeAllOsAlocatesBuffer                     *
**************************************************************************
* DESCRIPTION:  This function run over the all msdus in the MemMngr 
*               and call the free function of the os allocated buffers
*
* INPUT:        hMemMngr - The object
*           
* OUTPUT:
*               
* RETURN:       OK
**************************************************************************/

TI_STATUS wlan_memMngrFreeAllOsAlocatesBuffer(TI_HANDLE hMemMngr)
{
    UINT32 count;

    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;

    os_protectLock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect);/* START OF CRITICAL SECTION */

    for(count = 0 ; count < pMemMngr->msduMaxNumber ; count++)
    {
        if (pMemMngr->msduPool[count].freeFunc)
        {
            WLAN_OS_REPORT(("wlan_memMngrFreeAllOsAlocatesBuffer() - Call Os free func */*/*/**/*/*/*/*/*/*/*/*/*/*/*/*/*/*/*/ :\n"));
#ifndef TNETW_MASTER_MODE
            pMemMngr->msduPool[count].freeArgs[2] = NOK;
#endif
            /* call free function */
            pMemMngr->msduPool[count].freeFunc((TI_HANDLE)(pMemMngr->msduPool[count].freeArgs[0]),
                                                (TI_HANDLE)(pMemMngr->msduPool[count].freeArgs[1]),
                                                (TI_STATUS)(pMemMngr->msduPool[count].freeArgs[2]));

            pMemMngr->msduPool[count].freeFunc = NULL;
            pMemMngr->msduPool[count].freeArgs[0] = 0;
            pMemMngr->msduPool[count].freeArgs[1] = 0;
            pMemMngr->msduPool[count].freeArgs[2] = 0;
        }
    }

    os_protectUnlock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect);/* END OF CRITICAL SECTION */

    return OK;
}

/*************************************************************************
*                 wlan_memMngrCopyMsduFreeFunc                           *
**************************************************************************
* DESCRIPTION:  Copy The free function and the free arguments from on 
*               Msdu to another 
*
* INPUT:        hMemMngr - The object
*               destMsduHandle - the handle of the destination msdu
*               sourceMsduHandle - the handle of the source msdu    
* 
* OUTPUT:
*
* RETURN:       OK
**************************************************************************/

TI_STATUS wlan_memMngrCopyMsduFreeFunc(TI_HANDLE hMemMngr, UINT32 destMsduHandle, UINT32 sourceMsduHandle)
{
    mem_MSDU_T*             sourceMsdu; 
    mem_MSDU_T*             destMsdu;   

    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;

     if( destMsduHandle == WLAN_DRV_NULL_MEM_HANDLE || sourceMsduHandle == WLAN_DRV_NULL_MEM_HANDLE )
         return NOK;

    os_protectLock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect);  /* START OF CRITICAL SECTION */

    sourceMsdu = &(pMemMngr->msduPool[sourceMsduHandle]);
    destMsdu = &(pMemMngr->msduPool[destMsduHandle]);

    destMsdu->freeFunc = sourceMsdu->freeFunc;
    
    os_memoryCopy(pMemMngr->hOs, (void *)destMsdu->freeArgs, (void *)sourceMsdu->freeArgs,(NUM_OF_FREE_ARGS*sizeof(UINT32)));

    os_protectUnlock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect);/* END OF CRITICAL SECTION */

    return OK;
}

/*************************************************************************
*                 wlan_memMngrChangeMsduOwner                            *
**************************************************************************
* DESCRIPTION:  the function changes the msdu module owner.
*
* INPUT:        hMemMngr - The object
*               newModule - msdu new module owner
*               pMsdu - the msdu to be changed  
* 
* OUTPUT:
*
* RETURN:       OK
**************************************************************************/

TI_STATUS wlan_memMngrChangeMsduOwner(TI_HANDLE hMemMngr,allocatingModule_e newModule,mem_MSDU_T *pMsdu)
{
    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;
    allocatingModule_e oldModule;

    if(pMsdu == NULL)
    {
        WLAN_REPORT_ERROR(pMemMngr->hReport, MEM_MGR_MODULE_LOG,
            ("wlan_memMngrChangeMsduOwner: pMsdu == NULL\n"));
        return NOK;

    }

    oldModule = pMsdu->module;

    if(pMemMngr->moduleAllocCount[oldModule] > 0)
    {
        pMemMngr->moduleAllocCount[oldModule]--;
    }
    else
    {
        WLAN_REPORT_ERROR(pMemMngr->hReport, MEM_MGR_MODULE_LOG,
                            ("wlan_memMngrChangeMsduOwner: oldModule %d  AllocCount < 0 ,newModule %d\n", oldModule,newModule));
        return NOK;
    }
    

    pMemMngr->moduleAllocCount[newModule]++;

    pMsdu->module = newModule;

    WLAN_REPORT_INFORMATION(pMemMngr->hReport, MEM_MGR_MODULE_LOG,
                        ("wlan_memMngrChangeMsduOwner: oldModule: %d , newModule: %d\n", oldModule, newModule));


    return OK;



    


}


/*************************************************************************
*                 wlan_memMngrSwapMsdu                                  *
**************************************************************************
* DESCRIPTION:  Swap two Msdu, only the MSDU descriptor and not all fields
*                
* INPUT:        
* 
* OUTPUT:
*
* RETURN:       OK
**************************************************************************/
TI_STATUS wlan_memMngrSwapMsdu(TI_HANDLE hMemMngr, mem_MSDU_T *pMsdu_1, mem_MSDU_T *pMsdu_2)
{
    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;
    mem_MSDU_T Msdu_tmp;

    os_protectLock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect);  /* START OF CRITICAL SECTION */

    /* copy msdu 1 to Temporary msdu */
    Msdu_tmp.freeFunc = pMsdu_1->freeFunc;  
    os_memoryCopy(pMemMngr->hOs, (void *)Msdu_tmp.freeArgs, (void *)pMsdu_1->freeArgs,(NUM_OF_FREE_ARGS*sizeof(UINT32)));
    Msdu_tmp.dataLen = pMsdu_1->dataLen;
    Msdu_tmp.headerLen = pMsdu_1->headerLen;
    Msdu_tmp.firstBDPtr = pMsdu_1->firstBDPtr;
    Msdu_tmp.lastBDPtr  = pMsdu_1->lastBDPtr;

    /* copy msdu 2 to msdu 1 */
    pMsdu_1->freeFunc = pMsdu_2->freeFunc;  
    os_memoryCopy(pMemMngr->hOs, (void *)pMsdu_1->freeArgs, (void *)pMsdu_2->freeArgs,(NUM_OF_FREE_ARGS*sizeof(UINT32)));
    pMsdu_1->dataLen = pMsdu_2->dataLen;
    pMsdu_1->headerLen = pMsdu_2->headerLen;
    pMsdu_1->firstBDPtr = pMsdu_2->firstBDPtr;
    pMsdu_1->lastBDPtr  = pMsdu_2->lastBDPtr;

    /* copy Temporary msdu to msdu 2 */
    pMsdu_2->freeFunc = Msdu_tmp.freeFunc;  
    os_memoryCopy(pMemMngr->hOs, (void *)pMsdu_2->freeArgs, (void *)Msdu_tmp.freeArgs,(NUM_OF_FREE_ARGS*sizeof(UINT32)));
    pMsdu_2->dataLen = Msdu_tmp.dataLen;
    pMsdu_2->headerLen = Msdu_tmp.headerLen;
    pMsdu_2->firstBDPtr = Msdu_tmp.firstBDPtr;
    pMsdu_2->lastBDPtr  = Msdu_tmp.lastBDPtr;
        
    os_protectUnlock(pMemMngr->hOs, pMemMngr->hCriticalSectionProtect);/* END OF CRITICAL SECTION */

    return OK;
}

TI_STATUS wlan_memMngrGetMemMgrResources(TI_HANDLE hMemMngr, memMgrResources_t* memMgrResources)
{
    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;

    memMgrResources->numOfFreeBDs = pMemMngr->numFreeBD;
    memMgrResources->numOfFreeMsdu = pMemMngr->numFreeMSDU;
    memMgrResources->numOfFreeBufPool1 = pMemMngr->buffersPool[0].numFreeDataBuf;
    memMgrResources->numOfFreeBufPool2 = pMemMngr->buffersPool[1].numFreeDataBuf;
    memMgrResources->numOfFreeBufPool3 = pMemMngr->buffersPool[2].numFreeDataBuf;
    
    return OK;
}


TI_STATUS wlan_memMngrAddTimeStamp (TI_HANDLE hMemMngr, mem_MSDU_T *pMsdu)
{
  #ifdef TI_DBG
    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;

    if (pMsdu->timeStampNum < MAX_NUM_OF_TIME_STAMPS)
        pMsdu->timeStamp[pMsdu->timeStampNum ++] =
            os_timeStampUs (pMemMngr->hOs);
  #endif

    return OK;
}


/*************************************************************************
**************************************************************************
*                                                                        *
*                            TEST FUNCTIONS                              *
*                                                                        *
**************************************************************************
**************************************************************************/

void memMngrPrintMSDU(mem_MSDU_T* pMsdu )
{
    WLAN_OS_REPORT(("\nPrinting MSDU :\n"));
    WLAN_OS_REPORT(("handle          = %X\n",pMsdu->handle));
    WLAN_OS_REPORT(("nextFreeMSDU	   = %X\n",pMsdu->nextFreeMSDU));     
    WLAN_OS_REPORT(("headerLen       = %d\n",pMsdu->headerLen));    
    WLAN_OS_REPORT(("firstBDPtr      = %X\n",pMsdu->firstBDPtr));   
    WLAN_OS_REPORT(("lastBDPtr       = %X\n",pMsdu->lastBDPtr));     
    WLAN_OS_REPORT(("freeFunc        = %X\n",pMsdu->freeFunc));      
    WLAN_OS_REPORT(("freeArgs[0]     = %X\n",pMsdu->freeArgs[0]));   
    WLAN_OS_REPORT(("freeArgs[1]     = %X\n",pMsdu->freeArgs[1]));   
    WLAN_OS_REPORT(("freeArgs[2]     = %X\n",pMsdu->freeArgs[2]));   
    WLAN_OS_REPORT(("freeArgs[3]     = %X\n",pMsdu->freeArgs[3]));   
    WLAN_OS_REPORT(("freeArgs[4]     = %X\n",pMsdu->freeArgs[4]));   
    WLAN_OS_REPORT(("dataLen         = %d\n",pMsdu->dataLen));       
    WLAN_OS_REPORT(("module          = %d\n",pMsdu->module));       
    WLAN_OS_REPORT(("nextMSDUinList  = %X\n",pMsdu->nextMSDUinList));
    WLAN_OS_REPORT(("prevMSDUinList  = %X\n",pMsdu->prevMSDUinList));
    WLAN_OS_REPORT(("txFlags         = %X\n",pMsdu->txFlags));       
    WLAN_OS_REPORT(("txCompleteFlags = %X\n",pMsdu->txCompleteFlags));       

}

void memMngrPrintBD(mem_BD_T * pBd )
{
    WLAN_OS_REPORT(("\nPrinting BD \n"));
    WLAN_OS_REPORT(("handle           = %X\n",pBd->handle));     
    WLAN_OS_REPORT(("refCount         = %d\n",pBd->refCount));   
    WLAN_OS_REPORT(("dataBuf          = %X\n",pBd->dataBuf));    
    WLAN_OS_REPORT(("data             = %X\n",pBd->data));       
    WLAN_OS_REPORT(("dataOffset       = %d\n",pBd->dataOffset)); 
    WLAN_OS_REPORT(("length           = %d\n",pBd->length));     
    WLAN_OS_REPORT(("nextBDPtr        = %X\n",pBd->nextBDPtr));
#ifdef TNETW_MASTER_MODE 
    WLAN_OS_REPORT(("data_physical_low = %X\n",pBd->data_physical_low));
#endif
}

void memMngrPrintDataBuf(mem_DataBuf_T* pDataBuf )
{
    WLAN_OS_REPORT(("\nPrinting DataBuf \n"));
    WLAN_OS_REPORT(("handle      = %X\n",pDataBuf->handle));     
    WLAN_OS_REPORT(("nextDataBuf = %X\n",pDataBuf->nextDataBuf));    
    WLAN_OS_REPORT(("refCount    = %d\n",pDataBuf->refCount));  
    WLAN_OS_REPORT(("poolIndex	= %X\n",pDataBuf->poolIndex));       
    WLAN_OS_REPORT(("data		= %d\n",pDataBuf->data)); 
}

void memMngrPrintMSDUWithItsBds(mem_MSDU_T* pMsdu )
{
    mem_BD_T *bdTmp = pMsdu->firstBDPtr;

    memMngrPrintMSDU(pMsdu);

    while(bdTmp != NULL)
    {
        memMngrPrintBD(bdTmp);
        bdTmp = bdTmp->nextBDPtr;
    }
}

void memMngrPrintHandle(TI_HANDLE hMemMngr, UINT32 handle)
{
    mem_BD_T*       tmpBD;

    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;

    WLAN_REPORT_INFORMATION(pMemMngr->hReport, MEM_MGR_MODULE_LOG, 
        ("MSDU handle = %d firstBDPtr=%X length=%d\n", handle,  
                            pMemMngr->msduPool[handle].firstBDPtr,
                            pMemMngr->msduPool[handle].dataLen));

    tmpBD = pMemMngr->msduPool[handle].firstBDPtr;
    while (tmpBD != NULL) {
        WLAN_REPORT_INFORMATION(pMemMngr->hReport, MEM_MGR_MODULE_LOG,
            ("MSDU BD=%X handle=%d refCount=%d\n", tmpBD, tmpBD->handle, tmpBD->refCount));
        tmpBD = tmpBD->nextBDPtr;
    }

}

void memMngrFullPrint(TI_HANDLE hMemMngr)
{
    mem_MSDU_T*     tmpMSDU;
    mem_BD_T*       tmpBD;
    mem_DataBuf_T*  tmpDataBuf;
    UINT32  j,i=0;

    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;

    WLAN_OS_REPORT(("memMngrPrint\n"));
    WLAN_OS_REPORT(("numFreeMSDU %d numFreeBD %d  \n", pMemMngr->numFreeMSDU, pMemMngr->numFreeBD));
    for(j = 0 ;j < pMemMngr->currentNumberOfPools; j++)
        WLAN_OS_REPORT(("Pool Num %d   buffer length %d   numFreeDataBuf %d \n",
        j, pMemMngr->buffersPool[j].buffersSize, pMemMngr->buffersPool[j].numFreeDataBuf));

    WLAN_OS_REPORT(("\nAllocated by modules : MLME=%d, OS_ABS=%d,  RSN=%d,  HAL_RX=%d\n",
        pMemMngr->moduleAllocCount[MLME_MODULE],pMemMngr->moduleAllocCount[OS_ABS_TX_MODULE],
        pMemMngr->moduleAllocCount[RSN_MODULE],pMemMngr->moduleAllocCount[HAL_RX_MODULE]));


    WLAN_OS_REPORT(("\nfirstFreeMSDU=%X\n",pMemMngr->firstFreeMSDU));
    tmpMSDU = pMemMngr->firstFreeMSDU;
    while (++i, tmpMSDU != NULL) {
        WLAN_OS_REPORT(("tmpMSDU %d = %X handle=%d tmpMSDU->nextMSDU=%X\n", 
            i, tmpMSDU, tmpMSDU->handle, tmpMSDU->nextFreeMSDU));
        tmpMSDU = tmpMSDU->nextFreeMSDU;
    }

    WLAN_OS_REPORT(("\nfirstFreeBD=%X\n",pMemMngr->firstFreeBD));
    i = 0;
    tmpBD = pMemMngr->firstFreeBD;
    while (++i, tmpBD != NULL) {
        WLAN_OS_REPORT(("tmpBD %d = %X handle=%d tmpBD->nextBDPtr=%X\n",
            i, tmpBD, tmpBD->handle, tmpBD->nextBDPtr));
        tmpBD = tmpBD->nextBDPtr;
    }
    WLAN_OS_REPORT(("\n"));

    for(j = 0 ;j < pMemMngr->currentNumberOfPools; j++) {
        i = 0;
        tmpDataBuf = pMemMngr->buffersPool[j].firstFreeDataBuf;
        WLAN_OS_REPORT(("\npoolIndex=%d  firstFreeDataBuf=%X\n",j,pMemMngr->buffersPool[j].firstFreeDataBuf));
        while (++i, tmpDataBuf != NULL) {
            WLAN_OS_REPORT(("Buf %d = %X handle=%d  next=%X poolIndex=%d pData=%X\n", i, tmpDataBuf,
                tmpDataBuf->handle,tmpDataBuf->nextDataBuf, tmpDataBuf->poolIndex, tmpDataBuf->data));
            tmpDataBuf = tmpDataBuf->nextDataBuf;
        }
        WLAN_OS_REPORT(("\n"));
    }
    WLAN_OS_REPORT(("\n"));
}


void memMngrPrint(TI_HANDLE hMemMngr)
{
#if 0
    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;

    WLAN_OS_REPORT(("memMngrPrint\n"));
    WLAN_OS_REPORT(("numFreeMSDU %d numFreeBD %d  \n", pMemMngr->numFreeMSDU, pMemMngr->numFreeBD));
    for(j = 0 ;j < pMemMngr->currentNumberOfPools; j++)
        WLAN_OS_REPORT(("Pool Num %d   buffer length %d   numFreeDataBuf %d \n", j, 
            pMemMngr->buffersPool[j].buffersSize, pMemMngr->buffersPool[j].numFreeDataBuf));

    WLAN_OS_REPORT(("\nAllocated by modules on Tx\n"));


    WLAN_OS_REPORT(("\nAllocated by modules : MLME=%d, OS_ABS=%d,  RSN=%d  \n", 
                                            pMemMngr->moduleAllocCount[MLME_MODULE],
                                            pMemMngr->moduleAllocCount[OS_ABS_TX_MODULE],
                                            pMemMngr->moduleAllocCount[RSN_MODULE]));

    WLAN_OS_REPORT(("\nAllocated by modules : HAL_TX=%d, CONCAT=%d,  DE_CONCAT=%d,  TX=%d\n", 
                                            pMemMngr->moduleAllocCount[HAL_TX_MODULE],
                                            pMemMngr->moduleAllocCount[CONCAT_MODULE],
                                            pMemMngr->moduleAllocCount[DE_CONCAT_MODULE],
                                            pMemMngr->moduleAllocCount[TX_MODULE]));

    WLAN_OS_REPORT(("\nAllocated by modules : ACK_EMUL=%d, MEASUREMENT=%d,  SITE_MGR=%d,  EXC_MANAGER=%d\n", 
                                            pMemMngr->moduleAllocCount[ACK_EMUL_MODULE],
                                            pMemMngr->moduleAllocCount[MEASUREMENT_MODULE],
                                            pMemMngr->moduleAllocCount[SITE_MGR_MODULE],
                                            pMemMngr->moduleAllocCount[EXC_MANAGER_MODULE]));

    WLAN_OS_REPORT(("\nAllocated by modules on Rx\n"));

    WLAN_OS_REPORT(("\nAllocated by modules : HAL_RX=%d, CORE_RX=%d,  MLME_RX=%d,  OS_ABS_RX=%d\n", 
                                            pMemMngr->moduleAllocCount[HAL_RX_MODULE],
                                            pMemMngr->moduleAllocCount[CORE_RX_MODULE],
                                            pMemMngr->moduleAllocCount[MLME_RX_MODULE],
                                            pMemMngr->moduleAllocCount[OS_ABS_RX_MODULE]));

    WLAN_OS_REPORT(("\nAllocated by modules : RSN_RX=%d, MEASUREMENT_RX=%d,  SITE_MGR_RX=%d,  EXC_MANAGER_RX=%d\n", 
                                            pMemMngr->moduleAllocCount[RSN_RX_MODULE],
                                            pMemMngr->moduleAllocCount[MEASUREMENT_RX_MODULE],
                                            pMemMngr->moduleAllocCount[SITE_MGR_RX_MODULE],
                                            pMemMngr->moduleAllocCount[EXC_MANAGER_RX_MODULE]));



    WLAN_OS_REPORT(("\nAllocated by modules : HAL_WEP1_RX=%d, HAL_WEP2_RX=%d, HAL_DEFRAG_RX=%d,  HAL_DUPLICA_RX=%d\n", 
                                            pMemMngr->moduleAllocCount[HAL_WEP1_RX],
                                            pMemMngr->moduleAllocCount[HAL_WEP2_RX],
                                            pMemMngr->moduleAllocCount[HAL_DEFRAG_RX],
                                            pMemMngr->moduleAllocCount[HAL_DUPLICA_RX]));




    WLAN_OS_REPORT(("\nAllocated by modules : FREE_MSDU=%d\n", 
                                        pMemMngr->moduleAllocCount[MODULE_FREE_MSDU]));

#endif
}

void print_MsduDataHeader(TI_HANDLE hMemMngr, mem_MSDU_T *pMsdu)
{
    mem_BD_T            *pBd;
    UINT8               tempBuffer[40],*pTempBuffer;
    UINT32              lengthToPrint = 40;
    UINT32              i;

    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;

    pTempBuffer = tempBuffer;
    pBd =  pMsdu->firstBDPtr;

    while (lengthToPrint != 0)
    {
        if (pBd->length < lengthToPrint)    
        {
            os_memoryCopy(pMemMngr->hOs, (void *)pTempBuffer, (void *)pBd->data, pBd->length);
            lengthToPrint -= pBd->length;
            pTempBuffer += pBd->length;         
            pBd = pBd->nextBDPtr;
        }
        else                            /* enough place in current BD*/
        {
            os_memoryCopy(pMemMngr->hOs, (void *)pTempBuffer, (void *)pBd->data, lengthToPrint);
            lengthToPrint = 0;
        }
    }
    WLAN_OS_REPORT(("\n"));
    for(i = 0 ; i < 60 ; i++)
    {
        WLAN_OS_REPORT(("%02X ",tempBuffer[i]));
    }
    WLAN_OS_REPORT(("\n\n"));
}
/*void DumpMemory(char* data, int size)
{
    char NumStr[60], CharStr[20], ResStr[81];
    int bank, i, space;
    
    bank = 0;
    
    for(i=0; i<size; i++) {
        
        sprintf(&NumStr[bank*3], "%02X ", (UCHAR)data[i]);
        CharStr[bank] = (data[i]>=0x20 && data[i]<=0x7E) ? data[i] : '.';
        
        if(++bank == 16) {
            CharStr[bank] = 0;
            sprintf(ResStr, "%s        %s", NumStr, CharStr);
            printf("%s\n",ResStr);
            bank = 0;
        }
        
    }
    
    if(bank) {
        CharStr[bank] = 0;
        printf("%s", NumStr);
        if(size < 16)
            space = 6;
        else
            space = 56-strlen(NumStr);
        for(i=0; i<space; i++)
            printf(" ");
        printf("%s\n",CharStr);
    }
    
}
*/

TI_STATUS txDataSTUB_txSendMsdu(TI_HANDLE hMemMngr, mem_MSDU_T *pMsdu)
{
    TI_STATUS status;
    memMngr_t *pMemMngr = (memMngr_t *)hMemMngr;

    status = wlan_memMngrFreeMSDU(pMemMngr, memMgr_MsduHandle(pMsdu));
    if(status != OK)
        return NOK;
    
    return OK;
}

