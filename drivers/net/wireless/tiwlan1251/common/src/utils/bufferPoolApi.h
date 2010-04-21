/** \file bufferPoolApi.h
 *  \brief This file include public definitions for the buffer pool data structure, 
 *  \brief comprising its API.
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

#include "osTIType.h"

#ifndef __BUFFER_POOL_API_H__
#define __BUFFER_POOL_API_H__
/*
 ***********************************************************************
 *	Constant definitions.
 ***********************************************************************
 */
#define BUFFER_POOL_NO_BUFFER NULL

#ifdef TI_DBG
/* Maximum number of buffers being tracked in debug mode */
#define BUFFER_POOL_MAX_NUM_OF_BUFFERS_FOR_DBG 1000
#endif /* TI_DBG */

/*
 ***********************************************************************
 *	Enums.
 ***********************************************************************
 */

/*
 ***********************************************************************
 *	Typedefs.
 ***********************************************************************
 */

 /** \typedef bufferPool_buffer_t
  * \brief Defines a buffer data type (pointer to void)
  */
typedef void* bufferPool_buffer_t;

/*
 ***********************************************************************
 *	Structure definitions.
 ***********************************************************************
 */
#ifdef TI_DBG
/** \struct bufferPoolDbg_t
 * \brief This structure holds buffer pool debug information.\n
 */
typedef struct
{
    UINT32	 				numberOfUsedBuffers;     			/**< 
																 * The number of buffers currently 
																 * not allocated in this pool
																 */
	UINT32					NumberOfSuccessfulAllocations;		/**< 
																 * Number of allocation requests 
																 * ended successfuly
																 */
	UINT32					NumberOfDeclinedAllocations;		/**< 
																 * Number of failed allocation requests
																 * due to no memory
																 */
	UINT32					NumberOfFreeBufferRefreed;			/**<
																 * Number of buffers for which free was 
																 * called when they were already free
																 */
	BOOLEAN					bAllocated[ BUFFER_POOL_MAX_NUM_OF_BUFFERS_FOR_DBG ];
																/**< 
																 * A boolean array indicating for each
																 * buffer if its allocation status
																 */
} bufferPoolDbg_t;
#endif /* TI_DBG */

/*
 ***********************************************************************
 *	External data definitions.
 ***********************************************************************
 */

/*
 ***********************************************************************
 *	External functions definitions
 ***********************************************************************
 */
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
TI_HANDLE bufferPool_create( TI_HANDLE hOS, UINT32 numOfBuffers, UINT32 bufferSize );

/**
 * \author Ronen Kalish\n
 * \date 05-December-2005\n
 * \brief Configures a buffer pool object.\n
 *
 * Function Scope \e Public.\n
 * \param hbufferPool - handle to a buffer pool object.\n
 * \param hReport - handle to the report module.\n
 */
void bufferPool_config( TI_HANDLE hBufferPool, TI_HANDLE hReport );

/**
 * \author Ronen Kalish\n
 * \date 05-December-2005\n
 * \brief releasing a buffer pool object.\n
 *
 * Function Scope \e Public.\n
 * \param hbufferPool - handle to a buffer pool object.\n
 */
void bufferPool_destroy( TI_HANDLE hBufferPool );

/**
 * \author Ronen Kalish\n
 * \date 05-December-2005\n
 * \brief Reinitializes the buffer pool object, by marking all buffers 
 * \brief as unallocated.\n
 *
 * Function Scope \e Public.\n
 * \param hbufferPool - handle to a buffer pool object.\n
 */
void bufferPool_reinit( TI_HANDLE hBufferPool );

/**
 * \author Ronen Kalish\n
 * \date 05-December-2005\n
 * \brief Allocates a buffer.\n
 *
 * Function Scope \e Public.\n
 * \param hbufferPool - handle to a buffer pool object.\n
 * \return a buffer object, BUFFER_POOL_NO_BUFFER indication if non is available.\n
 */
bufferPool_buffer_t bufferPool_allocateBuffer( TI_HANDLE hBufferPool );

/**
 * \author Ronen Kalish\n
 * \date 05-December-2005\n
 * \brief returns a buffer to the pool.\n
 *
 * Function Scope \e Public.\n
 * \param hbufferPool - handle to a buffer pool object.\n
 * \param buffer - the buffer object to return to the pool.\n
 */
void bufferPool_releaseBuffer( TI_HANDLE hBufferPool, bufferPool_buffer_t buffer );

#ifdef TI_DBG
/**
 * \author Ronen Kalish\n
 * \date 29-December-2005\n
 * \brief Returns the buffer pool debug structure.\n
 *
 * Function Scope \e Public.\n
 * \param hbufferPool - handle to a buffer pool object.\n
 */
bufferPoolDbg_t *bufferPool_getDebugInformation( TI_HANDLE hBufferPool );
#endif /* TI_DBG */

#endif /* __BUFFER_POOL_API_H__ */

