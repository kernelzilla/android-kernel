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


#ifndef TNETWARB_H
#define TNETWARB_H

#include "osTIType.h"



/*----------------------------------------------------------------------*/
/*                      GKI Definitions                                 */
/*----------------------------------------------------------------------*/


/* Define the total number of buffer pools supported, fixed and dynamic.
*/
#define NUM_TNETWARB_TOTAL_BUF_POOLS			3


/* Set this flag to non zero if you want to do buffer ownership checks.
** Note that to do these checks, you should have all tasks that free
** buffers as GKI tasks.
*/
#define TNETWARB_ENABLE_OWNER_CHECK			0


/* Set this flag to non zero if you want to do buffer corruption checks.
** If set, GKI will check buffer tail corruption every time it processes
** a buffer. This is very useful for debug, and is minimal overhead in
** a running system.
*/
#define TNETWARB_ENABLE_BUF_CORRUPTION_CHECK  0


/* Set this flag to non zero if you want to do pool id checks.
** If set, GKI will check if pool id exceeds the number of pools.
** This is useful for debug, but not for release version.
*/
#define TNETWARB_ENABLE_POOL_ID_CHECK		 0


/***********************************************************************
** This queue is a general purpose buffer queue, for application use.
*************************************************************************/
typedef struct 
{
    void    *p_first;
    void    *p_last;
    UINT16  count;
} BUFFER_Q;

#define TNETWARB_IS_QUEUE_EMPTY(p_q) (p_q.count == 0)

/* Define the value that create pool will return if it fails
*/
#define TNETWARB_INVALID_POOL 0xFF


/********************************************************************/
/**  Buffer Management Data Structures                             **/
/********************************************************************/

typedef struct _buffer_hdr
{
	struct _buffer_hdr *p_next;   
	UINT8   q_id;
    UINT8   status;				
	
} BUFFER_HDR_T;


/* Buffer related defines
*/
#define BUFFER_HDR_SIZE		 (sizeof(BUFFER_HDR_T))
#define MAGIC_NO			 0xAA55AA55


#define BUF_STATUS_UNLINKED				0  
#define BUF_STATUS_FREE				    1  
#define BUF_STATUS_QUEUED				2  



/***********************************************************************
** Function prototypes
*/

#ifdef __cplusplus
extern "C" {
#endif


/* To get and release buffers, change owner and get size
*/

void	TNETWArb_buffer_init(UINT8 *pTNETWArb_Client_Array);
void	*TNETWArb_getpoolbuf (TI_HANDLE hTNETWArb,UINT8 module_id);
void	TNETWArb_freebuf(void *bptr);

/* User buffer queue management
*/
 void    TNETWArb_init_q (BUFFER_Q *p_q);
 void    TNETWArb_Enqueue (BUFFER_Q *p_q, void *p_buf);
 void    TNETWArb_Enqueue_head (BUFFER_Q *p_q, void *p_buf);
 void   *TNETWArb_Dequeue  (BUFFER_Q *p_q);
 void   *TNETWArb_remove_from_queue (BUFFER_Q *p_q, void *p_buf);
 void   *TNETWArb_getfirst (BUFFER_Q *p_q);
 void   *TNETWArb_getnext (void *p_buf);
 

#ifdef __cplusplus
}
#endif /* of __cplusplus */


#endif /* of GKI_H */
