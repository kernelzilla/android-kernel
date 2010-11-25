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


#include "commonTypes.h"
#include "memMngrEx.h" /* MSDU */
#include "report.h"

#include "TNETWArb_buffer.h"
#include "TNETWArb.h"
#include "TNETWIF.h"


/* Each module has its own buffer in the TNETW Arbiter synchronizer */

/*******************************************************************************
**
** Function         TNETWArb_init_q
**
** Description      Called by an application to initialize a buffer queue.
**
** Returns          void
**
*******************************************************************************/
void TNETWArb_init_q (BUFFER_Q *p_q)
{
    p_q->p_first = p_q->p_last = NULL;
    p_q->count = 0;
}



/*******************************************************************************
**
** Function         TNETWArb_getfirst
**
** Description      Return a pointer to the first buffer in a queue
**
** Returns          NULL if queue is empty, else buffer address
**
*******************************************************************************/
void *TNETWArb_getfirst (BUFFER_Q *p_q)
{
    return (p_q->p_first);
}

/*******************************************************************************
**
** Function         TNETWArb_getnext
**
** Description      Return a pointer to the next buffer in a queue
**
** Returns          NULL if no more buffers in the queue, else next buffer address
**
*******************************************************************************/
void *TNETWArb_getnext (void *p_buf)
{
    BUFFER_HDR_T    *p_hdr;

    p_hdr = (BUFFER_HDR_T *) ((UINT8 *) p_buf - BUFFER_HDR_SIZE);

	 if (p_hdr->p_next)
	{
        return ((UINT8 *)p_hdr->p_next + BUFFER_HDR_SIZE);
	}
    else
	{
		return (NULL);
	}
}


/*******************************************************************************
**
** Function         TNETWArb_remove_from_queue
**
** Description      Dequeue a buffer from the middle of the queue
**
** Returns          NULL if queue is empty, else buffer
**
*******************************************************************************/
void *TNETWArb_remove_from_queue (BUFFER_Q *p_q, void *p_buf)
{
    BUFFER_HDR_T *p_hdr;

    if (!p_q->count)
        return (NULL);

    p_hdr = (BUFFER_HDR_T *)(p_q->p_first) - 1;

    if ((void *)(p_hdr + 1) == p_buf)
        return (TNETWArb_Dequeue (p_q));

	for ( ; p_hdr; p_hdr = p_hdr->p_next)
    {
        if ((void *)(p_hdr->p_next + 1) == p_buf)
        {
            p_hdr->p_next = ((BUFFER_HDR_T *)p_buf - 1)->p_next;
            p_q->count--;

            /* Unlink the buffer since it has been removed from the queue */
            ((BUFFER_HDR_T *)p_buf - 1)->status = BUF_STATUS_UNLINKED;	
		
            return (p_buf);
        }
    }

    return (NULL);
}



/*******************************************************************************
**
** Function         TNETWArb_Dequeue
**
** Description      Dequeue a buffer from the head of a queue
**                  CAUTION This function Is not protected againt reentrance : see GKI_dequeue
**
** Returns          NULL if queue is empty, else buffer that is dequeued
**
*******************************************************************************/
void *TNETWArb_Dequeue (BUFFER_Q *p_q)
{
    BUFFER_HDR_T    *p_hdr;

	/*WLAN_OS_REPORT(("\n TNETWArb_Dequeue  p_q %x  !!!!  \n", p_q));*/

    if (!p_q->count)
        return (NULL);

    p_hdr = (BUFFER_HDR_T *)((UINT8 *)p_q->p_first - BUFFER_HDR_SIZE);


	if (p_hdr->status != BUF_STATUS_QUEUED)
    {
		WLAN_OS_REPORT(("\n GKI_Dequeue ==>  ERROR  p_q->p_first %x BUF_STATUS_QUEUED NOT QUEUED!!!!  %x\n", p_q->p_first));
        return NULL;
    }


    /* Keep buffers such that GKI header is invisible
    */
	if (p_hdr->p_next)
	{
        p_q->p_first = ((UINT8 *)p_hdr->p_next + BUFFER_HDR_SIZE);
	}
    else
    {
        p_q->p_first = NULL;
        p_q->p_last  = NULL;
    }

    p_q->count--;

    p_hdr->p_next = NULL;
	p_hdr->status = BUF_STATUS_UNLINKED;

    return ((UINT8 *)p_hdr + BUFFER_HDR_SIZE);
}


/*******************************************************************************
**
** Function         TNETWArb_Enqueue
**
** Description      Enqueue a buffer at the tail of the queue.
**                  CAUTION This function Is not protected againt reentrance 
**
** Returns          void
**
*******************************************************************************/
void TNETWArb_Enqueue (BUFFER_Q *p_q, void *p_buf)
{
    BUFFER_HDR_T    *p_hdr;

    p_hdr = (BUFFER_HDR_T *) ((UINT8 *) p_buf - BUFFER_HDR_SIZE);


	if (p_hdr->status != BUF_STATUS_UNLINKED)
    {
		WLAN_OS_REPORT(("\n GKI_Enqueue ==>  ERROR  p_buf %x BUF_STATUS_UNLINKED!!!!  %x\n", p_buf));
        return;
    }

    /* Since the queue is exposed (C vs C++), keep the pointers in exposed format */
    if (p_q->p_first)
    {
		BUFFER_HDR_T *p_last_hdr = (BUFFER_HDR_T *)((UINT8 *)p_q->p_last - BUFFER_HDR_SIZE);
        
		p_last_hdr->p_next = p_hdr;

    }
    else
	{
        p_q->p_first = p_buf;
	}

    p_q->p_last = p_buf;
    p_q->count++;
    
	p_hdr->p_next = NULL;
	p_hdr->status = BUF_STATUS_QUEUED;


}


/*******************************************************************************
**
** Function         TNETWArb_Enqueue_head
**
** Description      Enqueue a buffer at the head of the queue
**
** Returns          void
**
*******************************************************************************/
void TNETWArb_Enqueue_head (BUFFER_Q *p_q, void *p_buf)
{
    BUFFER_HDR_T    *p_hdr;


    p_hdr = (BUFFER_HDR_T *) ((UINT8 *) p_buf - BUFFER_HDR_SIZE);

	if (p_hdr->status != BUF_STATUS_UNLINKED)	
    {
		WLAN_OS_REPORT(("\n GKI_Enqueue ==>  ERROR  p_buf %x BUF_STATUS_UNLINKED!!!!  %x\n", p_buf));
        return;
    }

    if (p_q->p_first)
    {

        p_hdr->p_next = (BUFFER_HDR_T *)((UINT8 *)p_q->p_first - BUFFER_HDR_SIZE);
        p_q->p_first = p_buf;
    }
    else
    {
        p_q->p_first = p_buf;
        p_q->p_last  = p_buf;
        p_hdr->p_next = NULL;


    }

    p_q->count++;

	p_hdr->status = BUF_STATUS_QUEUED;

}









/************************* NEW GKI FOR WLAN ***********************************/

/*******************************************************************************
**
** Function         TNETWArb_buffer_init
**
** Description      Called once internally by GKI at startup to initialize all
**                  buffers and free buffer pools.
**
** Returns          void
**
*******************************************************************************/
void TNETWArb_buffer_init(UINT8 *pTNETWArb_Client_Array)
{
    BUFFER_HDR_T  *p_hdr;
	void		  *p_buf;
	UINT8			module_id;


	/*
	** Resetting the buffer to STATUS_FREE
	*/
    for (module_id = 0; module_id < NUM_OF_TNETWIF_MODULES; module_id++)
    {
		/* Pick up corresponding buffer */
		p_buf = (void *)(&(pTNETWArb_Client_Array[module_id*(BUFFER_HDR_SIZE+TNETWARB_INSTANCE_SIZE)]) + BUFFER_HDR_SIZE);
		p_hdr = (BUFFER_HDR_T *) ((UINT8 *) p_buf - BUFFER_HDR_SIZE);

		p_hdr->p_next  = NULL;   /* There is no next buffer of the last one*/
		p_hdr->status  = BUF_STATUS_FREE; /* Update the status of the released buffer*/
	}

}




/*******************BUFFER ALLOCATION******************************************/

/*******************************************************************************
**
** Function         TNETWArb_getpoolbuf
**
** Description      Called by an application to get a free buffer from
**                  a specific buffer pool should be used in sections which no interrupts 
**					protection is needed.
**
** Returns          A pointer to the buffer, or NULL if none available
**
*******************************************************************************/
void *TNETWArb_getpoolbuf (TI_HANDLE hTNETWArb,UINT8 module_id)
{
	/* Handle to TNETW Arbiter struct */
	TNETWArb_t	  *pTNETWArb = (TNETWArb_t *)hTNETWArb;	
    BUFFER_HDR_T  *p_hdr;
	void		  *p_buf;

	p_buf = (void *)((&(pTNETWArb->TNETWArb_Client_Instance_Array[module_id][0])) + BUFFER_HDR_SIZE);

    p_hdr = (BUFFER_HDR_T *) ((UINT8 *) p_buf - BUFFER_HDR_SIZE);

	if (p_hdr->status != BUF_STATUS_FREE)	
	{	
		/*WLAN_OS_REPORT(("\n GKI_getpoolbuf ==>  ERROR  p_hdr %x NOT FREE Status %d module_id %d !!!\n", p_hdr,p_hdr->status,module_id));*/
		return NULL;
	}

    p_hdr->status  = BUF_STATUS_UNLINKED;
    p_hdr->p_next  = NULL;
		
	return ((void *) ((UINT8 *)p_hdr + BUFFER_HDR_SIZE));

}


/*******************************************************************************
**
** Function         TNETWArb_freebuf
**
** Description      Called by an application to return a buffer to the free pool.
**					To be used in sections which no interrupts protection is needed. 
**
** Returns          void
**
*******************************************************************************/
void TNETWArb_freebuf(void *bptr)
{
	BUFFER_HDR_T	*p_hdr;


	p_hdr = (BUFFER_HDR_T *) ((UINT8 *)bptr - BUFFER_HDR_SIZE);

	if (p_hdr->status != BUF_STATUS_UNLINKED)	
	{	
		WLAN_OS_REPORT(("\n GKI_freebuf ==>  ERROR  bptr %x BUF_STATUS_UNLINKED!!!!  %x\n", bptr));
		return;
	}

	/*
	** Resetting the buffer to STATUS_FREE
	*/
	p_hdr->p_next  = NULL;   /* There is no next buffer of the last one*/
	p_hdr->status  = BUF_STATUS_FREE; /* Update the status of the released buffer*/
		
}




