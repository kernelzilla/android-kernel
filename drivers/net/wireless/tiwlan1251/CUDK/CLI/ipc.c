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
/*      MODULE:     ipc.c                                                                           */
/*      PURPOSE:    Inter Process Communication utils                                               */
/*      Note:	    This module is for LINUX compilation only!										*/
/*                                                                                                  */
/****************************************************************************************************/

#include <sys/mman.h>
#include <sys/ipc.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>

#include "console.h"
#include "ticon.h"
#include "ipc.h"

	/*********************/
	/* Global variables */
	/*******************/

int ethernet_wipp_process_pid = 0;
int ethernet_g_tester_process_pid = 0;
int ethernet_logger_process_pid = 0;


int ethernet_wipp_control_pipe[2];
int ethernet_g_tester_pipe[2];
int ethernet_logger_pipe[2];

int ipc_pipe[2];

void *p_shared_memory;


/************************************************************************
 *                        ipc_initialize				                *
 ************************************************************************
DESCRIPTION: Initialize the IPC 

CONTEXT:  main process only!
************************************************************************/
int ipc_initialize()
{
	/*****************************/
	/* Create IPC shared memory */
	/***************************/

	if ((p_shared_memory = mmap(0, SHARED_MEMORY_SIZE, PROT_READ | PROT_WRITE, MAP_ANON | MAP_SHARED, -1, 0)) == (void *)-1) 
	{
		/* I should use the regular 'printf' function, because the 
		   'console_printf_terminal' is not working w/o shared memory */
		printf("IPC Error, can't create shared memory mapping (%s)!\n", strerror(errno));

		return -1;
	}

	SHARED_MEMORY_TERMINAL_OUTPUT_PATH() = OUTPUT_PATH_SIMPLE_UART;

	/************************/
	/* Create the IPC pipe */
	/**********************/

	if (pipe(ipc_pipe) < 0)
	{
		console_printf_terminal("IPC Error, can't create pipe\n");

		return -1;
	}

	/* Close the write direction of the pipe  - because i only read information from this pipe. */ 
	/*close(ipc_pipe[1]);*/

	return 0;
}

/************************************************************************
 *                        ipc_deinitialize				                *
 ************************************************************************
DESCRIPTION: Deinitialize the IPC 

CONTEXT:  main process only!
************************************************************************/
void ipc_deinitialize()
{
	/* Close the read direction of the pipe */ 
	close(ipc_pipe[0]);
}

/************************************************************************
 *                        ipc_send_command_to_main_process              *
 ************************************************************************
DESCRIPTION: Handles the 'SIGUSR1' signal 

CONTEXT:  All child process - NOT FROM parent process!!!!
************************************************************************/
void ipc_send_command_to_main_process(int module_index, unsigned char *command, int size)
{
	int pid = getpid();

	/*********************************************************************/
	/* Flow control 													*/
	/* The pid of the caller process is inserted, so the main process  */
	/* will signal it back and release the waiting condition		  */
	/*****************************************************************/

	command[0] = command[1] = 0xFF;

	switch (module_index)
	{
	case ETHERNET_UTILS_G_TESTER_MODULE_ID:
		command[0] = (pid & 0x00FF);
		command[1] = ((pid & 0xFF00) >> 8);
		command[2] = '-';
		break;

	case ETHERNET_UTILS_WIPP_MODULE_ID:
	case ETHERNET_UTILS_LOGGER_MODULE_ID:
		command[0] = (pid & 0x00FF);
		command[1] = ((pid & 0xFF00) >> 8);
		command[2] = '+';
		break;

	case GENERAL_PROCESS_MODULE_ID:
		command[2] = '!';
		break;
	}

	/* Send the buffer to the main process */
	write(ipc_pipe[1], command, size);

	/* Wait for 300usec (probably the signal will release us earlier) */
	usleep(300);
}

