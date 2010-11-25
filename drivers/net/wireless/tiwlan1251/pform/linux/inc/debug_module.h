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

#ifndef _DEBUG_MODULE_H
#define _DEBUG_MODULE_H

#include "osTIType.h"
#include "debug_module_ioctl.h"

/*****************************************************/
/*****************************************************/
/* Debug mode - use this define to debug this module */
/* #define DEBUG_THIS_MODULE*/

#ifdef DEBUG_THIS_MODULE
  #ifdef _WINDOWS
  #else  /* ifdef __LINUX__ */
	#define DEBUG_MESSAGE(_msg) (report_d _msg)
  #endif
#else /* DEBUG_THIS_MODULE */
	#define DEBUG_MESSAGE(_msg)
#endif /* DEBUG_THIS_MODULE */

/*****************************************************/
/*****************************************************/

#define MAX_CHAR_DRIVERS					 (2)

#define MAX_CHAR_DRIVER_NAME				 (30)
#define SPI_CHAR_DRIVER_NAME				 ("SPI_char_driver")
#define DEBUG_MSG_CHAR_DRIVER_NAME			 ("Debug_messages_char_driver")

#define SPI_CHAR_DRIVER						 (0)
#define DEBUG_MSG_CHAR_DRIVER				 (1)

#define ENQUEUE_MESSAGE_CONTROL_SIZE		 (1)
#define CONTROL_CODE_TYPE_MESSAGE			 (0)

#define REPORT_MESSAGE_PARAMS				 (3)
#define REPORT_MESSAGE_LEN					 (sizeof(unsigned int) * REPORT_MESSAGE_PARAMS)

#define DEBUG_MODULE_TRACE_QUEUE_ID			 (0)

typedef struct  
{
	unsigned int read_pointer;		/* Current read pointer */
	unsigned int write_pointer;		/* Current write pointer */
	unsigned int used_size;			/* The used data size */
	unsigned int size;				/* Total size of the buffer */
	unsigned int overrun_count;     /* Number of packets that were run over */

	tiBOOL		 waiting_for_data;	/* TRUE if the user mode is pending */ 
} t_queue_status;

extern tiBOOL module_enable;
extern int *buffers_default_size[MAX_CHAR_DRIVERS];
extern UINT8  *buffers[MAX_CHAR_DRIVERS];									/* Pointer to the allocated buffder */


void init_queue(UINT8  device_index);

unsigned char get_message_from_queue(UINT8 device_index, t_ioctol1_format *ioctl_data);
void get_queue_information(UINT8 device_index, unsigned int *report_msg);
tiBOOL debug_module_enqueue_message(UINT8 queue_id, UINT8 param, void *p_msg, unsigned int len, UINT8  message_type);

tiBOOL debug_module_logic_allocate_queue(UINT8 device_index, UINT8 size_in_k);
void debug_module_logic_deallocate_queue(UINT8 device_index, UINT8 size_in_k);

/* OS depended function prototypes */
void os_cmd_spin_lock_irqsave(void);
void os_cmd_spin_unlock_irqrestore(void);
void os_cmd_up(void);
void os_cmd_down_interruptible(void);
void os_cmd_copy_to_user(void *destination, const void *source, unsigned int size);
void *os_malloc (UINT32 size);
void os_free (void *p, UINT32 size);


#endif /* _DEBUG_MODULE_H */
