/** \file bufferPool.h
 *  \brief This file include private definitions for the buffer pool object
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
#include "bufferPoolApi.h"

#ifndef __BUFFER_POOL_H__
#define __BUFFER_POOL_H__

/** \struct bufferPool__t
 * \brief This structure comprise the buffer pool object.\n
 */
typedef struct
{
    TI_HANDLE               hOS;                /**< handle to the OS object */
    TI_HANDLE               hReport;            /**< handle to the report object */
	UINT32 					numberOfBuffers;    /**< The total number of buffers allocated for this pool */
	UINT32 					bufferSize;         /**< The size of buffers in this pool */
    bufferPool_buffer_t     firstBuffer;        /**< memory block holding all buffers */
    bufferPool_buffer_t     firstFreeBuffer;    /**< pointer to the first free buffer */
#ifdef TI_DBG
	bufferPoolDbg_t			bufferPoolDbg;		/**< debug information */
#endif /* TI_DBG */
} bufferPool_t;

#endif /* __BUFFER_POOL_H__ */

