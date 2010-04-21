/** \file bufferPool.c
 *  \brief This file include buffer pool module implementation
 *  \author Ronen Kalish
 *  \date 05-December-2005
 */

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

#include "bufferPool.h"
#include "osApi.h"
#include "report.h"

/**
 * \author Ronen Kalish\n
 * \date 05-December-2005\n
 * \brief Creates a buffer pool object
 *
 * Function Scope \e Public.\n
 * \param hOS - handle to the OS object.\n
 * \param numOfBuffers - the number of buffers to allocate for this pool.\n
 * \param bufferSize - the size of each buffer in this pool.\n
 * \return a handle to a buffer pool object, NULL if an error occurred.\n
 */
TI_HANDLE bufferPool_create( TI_HANDLE hOS, UINT32 numOfBuffers, UINT32 bufferSize )
{
    /* allocate the buffer pool object */
    bufferPool_t *pBufferPool = os_memoryAlloc( hOS, sizeof(bufferPool_t) );
    if ( NULL == pBufferPool )
    {
        WLAN_OS_REPORT( ("ERROR: Failed to create buffer pool object") );
        return NULL;
    }

    /* 
     * adjust buffer size if necessary - the buffer must at least store the pointer to the
     * next free buffer when it is free.
     */
    if ( sizeof( bufferPool_buffer_t ) > bufferSize )
    {
        bufferSize = sizeof( bufferPool_buffer_t );
    }

    /* nullify the buffer pool object */
    os_memoryZero( hOS, pBufferPool, sizeof( bufferPool_t ) );

    /* allocate the buffers */
    pBufferPool->firstBuffer = pBufferPool->firstFreeBuffer = os_memoryAlloc( hOS, numOfBuffers * bufferSize );
    if ( NULL == pBufferPool->firstBuffer )
    {
        WLAN_OS_REPORT( ("ERROR: Failed to allocate buffer storage space") );
        bufferPool_destroy( (TI_HANDLE)pBufferPool );
        return NULL;
    }

    /* store the OS handle */
    pBufferPool->hOS = hOS;

    /* store buffer pool information */
    pBufferPool->bufferSize = bufferSize;
    pBufferPool->numberOfBuffers = numOfBuffers;

    /* initialize the free buffers list */
    bufferPool_reinit( (TI_HANDLE)pBufferPool );

#ifdef TI_DBG
	/* initialize debug information */
	os_memoryZero( pBufferPool->hOS, &(pBufferPool->bufferPoolDbg), sizeof( bufferPoolDbg_t ) );
#endif /* TI_DBG */

    return pBufferPool;
}

/**
 * \author Ronen Kalish\n
 * \date 05-December-2005\n
 * \brief Configures a buffer pool object.\n
 *
 * Function Scope \e Public.\n
 * \param hbufferPool - handle to a buffer pool object.\n
 * \param hReport - handle to the report module.\n
 */
void bufferPool_config( TI_HANDLE hBufferPool, TI_HANDLE hReport )
{
    bufferPool_t* pBufferPool = (bufferPool_t*)hBufferPool;

    /* keep the report handle */
    pBufferPool->hReport = hReport;
}

/**
 * \author Ronen Kalish\n
 * \date 05-December-2005\n
 * \brief releasing a buffer pool object.\n
 *
 * Function Scope \e Public.\n
 * \param hbufferPool - handle to a buffer pool object.\n
 */
void bufferPool_destroy( TI_HANDLE hBufferPool )
{
    bufferPool_t* pBufferPool = (bufferPool_t*)hBufferPool;

    /* release the buffers storage space */
    os_memoryFree( pBufferPool->hOS, pBufferPool->firstBuffer,
                   pBufferPool->bufferSize * pBufferPool->numberOfBuffers );

    /* release the buffer pool object */
    os_memoryFree( pBufferPool->hOS, pBufferPool, sizeof(bufferPool_t) );
}

/**
 * \author Ronen Kalish\n
 * \date 05-December-2005\n
 * \brief Reinitializes the buffer pool object, by marking all buffers 
 * \brief as unallocated.\n
 *
 * Function Scope \e Public.\n
 * \param hbufferPool - handle to a buffer pool object.\n
 */
void bufferPool_reinit( TI_HANDLE hBufferPool )
{
    bufferPool_t* pBufferPool = (bufferPool_t*)hBufferPool;
    UINT32 i;

    /* make the first free buffer point to the first buffer */
    pBufferPool->firstFreeBuffer = pBufferPool->firstBuffer;

    /* insert all buffers to free list - last buffer must point to NULL */
    for ( i = 0; i < pBufferPool->numberOfBuffers - 1; i++ )
    {
        /* make the i'th buffer point to the next buffer */
        *(bufferPool_buffer_t*)((char*)pBufferPool->firstBuffer + (pBufferPool->bufferSize * i)) = 
            (bufferPool_buffer_t)((char*)pBufferPool->firstBuffer + (pBufferPool->bufferSize * (i+1)));
    }

    /* make the last buffer point to NULL */
    *(bufferPool_buffer_t*)((char*)pBufferPool->firstBuffer + 
        (pBufferPool->bufferSize * (pBufferPool->numberOfBuffers - 1))) = 
            BUFFER_POOL_NO_BUFFER;

#ifdef TI_DBG
	/* mark that no buffers are allocated */
	pBufferPool->bufferPoolDbg.numberOfUsedBuffers = 0;

	/* mark all buffers as unallocated in the debug information */
	for ( i = 0; i < BUFFER_POOL_MAX_NUM_OF_BUFFERS_FOR_DBG; i++ )
	{
		pBufferPool->bufferPoolDbg.bAllocated[ i ] = FALSE;
	}
#endif /* TI_DBG */
}

/**
 * \author Ronen Kalish\n
 * \date 05-December-2005\n
 * \brief Allocates a buffer.\n
 *
 * Function Scope \e Public.\n
 * \param hbufferPool - handle to a buffer pool object.\n
 * \return a buffer object, BUFFER_POOL_NO_BUFFER indication if non is available.\n
 */
bufferPool_buffer_t bufferPool_allocateBuffer( TI_HANDLE hBufferPool )
{
    bufferPool_t* pBufferPool = (bufferPool_t*)hBufferPool;
    bufferPool_buffer_t buffer;
#ifdef TI_DBG
	UINT32 bufferIndex;
#endif

    /* check if a buffer is available */
    if ( BUFFER_POOL_NO_BUFFER == pBufferPool->firstFreeBuffer )
    {
#ifdef TI_DBG
		pBufferPool->bufferPoolDbg.NumberOfDeclinedAllocations++;
#endif /* TI_DBG */
		return BUFFER_POOL_NO_BUFFER;
    }

    /* temporarily keep the first free buffer */
    buffer = pBufferPool->firstFreeBuffer;

    /* advance the first free buffer to point to the next buffer in the list */
    pBufferPool->firstFreeBuffer = *(bufferPool_buffer_t*)((pBufferPool->firstFreeBuffer));

#ifdef TI_DBG
	/* increase the buffer in use count */
	pBufferPool->bufferPoolDbg.numberOfUsedBuffers++;

	/* count the successful allocation */
	pBufferPool->bufferPoolDbg.NumberOfSuccessfulAllocations++;

	/* mark the specific buffer as used */
	bufferIndex = ((UINT8*)buffer - (UINT8*)pBufferPool->firstBuffer) / pBufferPool->bufferSize;
	if ( bufferIndex < BUFFER_POOL_MAX_NUM_OF_BUFFERS_FOR_DBG )
	{
		pBufferPool->bufferPoolDbg.bAllocated[ bufferIndex ] = TRUE;
	}
#endif /* TI_DBG */

    return buffer;
}

/**
 * \author Ronen Kalish\n
 * \date 05-December-2005\n
 * \brief Returns a buffer to the pool.\n
 *
 * Function Scope \e Public.\n
 * \param hbufferPool - handle to a buffer pool object.\n
 * \param buffer - the buffer object to return to the pool.\n
 */
void bufferPool_releaseBuffer( TI_HANDLE hBufferPool, bufferPool_buffer_t buffer )
{
    bufferPool_t* pBufferPool = (bufferPool_t*)hBufferPool;
#ifdef TI_DBG
	UINT32 bufferIndex;

	/* check if the buffer is currently allocated */
	bufferIndex = ((UINT8*)buffer - (UINT8*)pBufferPool->firstBuffer) / pBufferPool->bufferSize;

	if ( (bufferIndex < BUFFER_POOL_MAX_NUM_OF_BUFFERS_FOR_DBG) &&
		 (FALSE == pBufferPool->bufferPoolDbg.bAllocated[ bufferIndex ]) )
	{
		/* count number of free attempts for already free buffers */
		pBufferPool->bufferPoolDbg.NumberOfFreeBufferRefreed++;

		WLAN_OS_REPORT(("%s: Trying to re-free Buffer %d\n", __FUNCTION__, bufferIndex));
		return;
	}
	else
	{
		/* decrease the buffers in use count */
		pBufferPool->bufferPoolDbg.numberOfUsedBuffers--;

		/* mark that the specific buffer is not in use */
		pBufferPool->bufferPoolDbg.bAllocated[ bufferIndex ] = FALSE;
	}
#endif /* TI_DBG */

    /* make the newly released buffer point to the current first free buffer */
    *((bufferPool_buffer_t*)buffer) = pBufferPool->firstFreeBuffer;

    /* make the newly release buffer the first free buffer */
    pBufferPool->firstFreeBuffer = buffer;
}

#ifdef TI_DBG
/**
 * \author Ronen Kalish\n
 * \date 29-December-2005\n
 * \brief Returns the buffer pool debug structure.\n
 *
 * Function Scope \e Public.\n
 * \param hbufferPool - handle to a buffer pool object.\n
 */
bufferPoolDbg_t *bufferPool_getDebugInformation( TI_HANDLE hBufferPool )
{
    bufferPool_t* pBufferPool = (bufferPool_t*)hBufferPool;

	return &(pBufferPool->bufferPoolDbg);
}
#endif

