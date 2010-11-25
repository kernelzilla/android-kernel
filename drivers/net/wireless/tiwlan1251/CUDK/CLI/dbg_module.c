/*******************************************************************************
**+--------------------------------------------------------------------------+**
**|                                                                          |**
**| Copyright 1998-2008 Texas Instruments, Inc. - http://www.ti.com/         |**
**|                                                                          |**
**| Licensed under the Apache License, Version 2.0 (the "License");          |**
**| you may not use this file except in compliance with the License.         |**
**| You may obtain a copy of the License at                                  |**
**|                                                                          |**
**|     http://www.apache.org/licenses/LICENSE-2.0                           |**
**|                                                                          |**
**| Unless required by applicable law or agreed to in writing, software      |**
**| distributed under the License is distributed on an "AS IS" BASIS,        |**
**| WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. |**
**| See the License for the specific language governing permissions and      |**
**| limitations under the License.                                           |**
**|                                                                          |**
**+--------------------------------------------------------------------------+**
*******************************************************************************/


/****************************************************************************************************/
/*                                                                                                  */
/*      MODULE:     dbg_module.c                                                                    */
/*      PURPOSE:    Handle the debug messages 		                                                */
/*      Note:	    This module is for LINUX compilation only!										*/
/*                                                                                                  */
/****************************************************************************************************/


#include <stdarg.h> 
#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <fcntl.h>
#include <stdarg.h>

#include <sys/time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/resource.h>

#include "ipc.h"
#include "ticon.h"
#include "console.h"
#include "dbg_module.h"
#include "eth_utils.h"
#include "debug_module_ioctl.h"

	/************/
	/* Defines */
	/**********/

#define DEBUG_MODULE_BUFFER_SIZE	(255)

	/*********************/
	/* Global variables */
	/*******************/

int debug_module_dev_file = -1;
int debug_module_process_pid = 0;

	/********************************/
	/* static functions prototypes */
	/******************************/

unsigned char file_exists(const char *file_name);

	/**************/
	/* Functions */
	/************/

/************************************************************************
 *                        debug_module_init		                        *
 ************************************************************************
DESCRIPTION: Initialize the debug module user mode process

  CONTEXT	   : main process only!
************************************************************************/
void debug_module_init(void)
{
	int return_value;

	/*********************************/
	/* Create the debug device file */
	/*******************************/

	if (!file_exists("/dev/debug_msg_dev"))
	{
		/* Create the debug device file */
		return_value = system("mknod /dev/debug_msg_dev c 254 0");
	}
	
	debug_module_dev_file = open("/dev/debug_msg_dev", O_RDWR);
	
	if (debug_module_dev_file == -1)
	{
		console_printf_terminal("Debug module, error opening \"/dev/debug_msg_dev\"\n");

		return;
	}
	
	/* Create another instance of this process */
	debug_module_process_pid = fork();
	
	if (debug_module_process_pid == 0)
	{
		/******************/
		/* Child process */
		/****************/
		
		unsigned char bytes_read;
		unsigned char ioctl_return_size;
		t_ioctol1_format ioctl_data;
		unsigned char in_debug_buffer[DEBUG_MODULE_BUFFER_SIZE + 1];
	
		/* Set the priority of this process to the lowest */
/*		setpriority(PRIO_PROCESS, getpid(), -20);*/
		
		console_printf_terminal("Debug module, Hello from child process (pid = %d).\n", getpid());
		
		/*******************************/
		/* Prepare the ioctl's fields */
		/*****************************/
		
		ioctl_data.return_size = &ioctl_return_size;
		ioctl_data.buffer = (in_debug_buffer + 1);
		ioctl_data.buffer_size = DEBUG_MODULE_BUFFER_SIZE;
		
		while (TRUE)
		{
			/*console_printf_terminal("Debug module, before ioctl\n");*/
			
			/* Read data from device file - blocking command */
			return_value = ioctl(debug_module_dev_file, 10, &ioctl_data);
			
			/*console_printf_terminal("Debug module, after ioctl (%d)\n", return_value);*/
			
			if (return_value != -1)
			{
				bytes_read = ioctl_return_size;
				
				if (bytes_read > 0)
				{
					/* Mark that this is log message */ 
					in_debug_buffer[0] = 0;

					/* Put '0' in the end of the string */
					in_debug_buffer[bytes_read + 1] = 0;
					
					console_send_buffer_to_host(ETHERNET_UTILS_LOGGER_MODULE_ID, in_debug_buffer, (bytes_read + 1));
				}
			}
			else
			{
				console_printf_terminal("Debug module, error reading from device file.\n");
			}
		}
	}
}

/************************************************************************
 *                        debug_module_deinit                           *
 ************************************************************************
DESCRIPTION: Deinitialize the debug module user mode process

CONTEXT    : main process only!
************************************************************************/
void debug_module_deinit(void)
{
	if (debug_module_process_pid)
	{
		kill(debug_module_process_pid, SIGKILL);
	}
}

/************************************************************************
 *                        debug_module_get_queue_status                 *
 ************************************************************************
DESCRIPTION: Gets the status of the debug module queue 

CONTEXT    : main process only!
************************************************************************/

struct q_report {
	unsigned int q_size;
	unsigned int q_used;
	unsigned int q_overrun;
};

#define QUEUE_STATUS_LEN (12)
void debug_module_get_queue_status(void)
{
	int return_value;
	char status_result[QUEUE_STATUS_LEN + 1];
	struct q_report report;

	/* Read data from device file - blocking command */
	return_value = ioctl(debug_module_dev_file, 11, &report);

	if (return_value != -1)		   
	{
		/* console_printf_terminal("Debug module, debug_module_get_queue_status. (size = %d, used = %d, overrun = %d)\n", report.q_size , report.q_used, report.q_overrun);  */

		memcpy(status_result + 1, &report, sizeof(report));

		/* Mark that this is report message */
		status_result[0] = 1;

		console_send_buffer_to_host(ETHERNET_UTILS_LOGGER_MODULE_ID, (tiUINT8*) status_result, QUEUE_STATUS_LEN + 1);
	}
	else
	{
		console_printf_terminal("Debug module, error reading from device file.\n");
	}
}

/************************************************************************
 *                        file_exists		                            *
 ************************************************************************
DESCRIPTION: Check if a specific file exists

CONTEXT    : All process.

Returns:     TRUE if the file exists (FALSE otherwise).
************************************************************************/
unsigned char file_exists(const char *file_name)
{
	int test_file;
	tiBOOL result = FALSE;;

	/* Try to open the file */
	test_file = open(file_name, O_RDONLY);

	if (test_file != -1)
	{
		/*************************************************/
		/* The file was successfullu opened - it exists */
		/***********************************************/

		close(test_file);

		result = TRUE;
	}

	return result;
}

