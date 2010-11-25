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
/*      MODULE:     wipp_ctrl.c                                                                     */
/*      PURPOSE:    WIPP Control utilities			                                                */
/*      Note:	    This module is for LINUX compilation only!										*/
/*                                                                                                  */
/****************************************************************************************************/

#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <stdarg.h> 
#include <stdlib.h>
#include <termios.h>
#include <dirent.h>


#include <sys/wait.h>
#include <sys/time.h>
#include <sys/resource.h>

#include "ipc.h"
#include "ticon.h"
#include "cu_cmd.h"
#include "console.h"
#include "eth_utils.h"
#include "wipp_ctrl.h"
#include "dbg_module.h"
#include "TI_AdapterApiC.h"

	/************/
	/* Defines */
	/**********/

#define WIPP_HEADER(_buffer, _op)	({_buffer[0] = '+'; _buffer[1] = _op;})
#define WIPP_CONTROL_HEADER_SIZE	(2)

#define TEMP_BUFFER_SIZE		(256)
#define MAX_PROCESS_TABLE_INDEX	(50)
	/*********************/
	/* Global variables */
	/*******************/
int wipp_control_general_process_pid = 0;
int wipp_control_general_process_out_pipe[2];
int wipp_control_general_process_in_pipe[2];

int process_table_current_index = 0;
int process_table[MAX_PROCESS_TABLE_INDEX][2];


	/********************************/
	/* static functions prototypes */
	/******************************/

void wipp_control_initialize_general_process(void);
int wipp_control_send_message(unsigned char *buffer, unsigned int length);
void wipp_control_cpu_usage(ConParm_t parm[], U16 nParms);
void wipp_control_mem_usage(ConParm_t parm[], U16 nParms);
void wipp_control_report_version(ConParm_t parm[], U16 nParms);
void wipp_control_set_pp_mode(ConParm_t parm[], U16 nParms);
void wipp_control_disable_term_echo(void);
void wipp_control_set_baud_rate(int rate_index);
void wipp_control_active_iperf(char *iperf_cmd_line);
void wipp_control_deactive_iperf(void);
void wipp_control_kill_family_process(int father_pid);
void wipp_control_fill_process_table(void);
void  wipp_control_get_version(void);


	/**************/
	/* Functions */
	/************/

/************************************************************************
 *                        wipp_control_init  	                        *
 ************************************************************************
DESCRIPTION: Initialize the wipp control module.
			 The iperf control process is prepared during this stage.

CONTEXT	   : main process only!
************************************************************************/
void wipp_control_init(void)
{
	wipp_control_initialize_general_process();
}

/************************************************************************
*                        wipp_control_deinit  	                        *
 ************************************************************************
DESCRIPTION: Deinitialize the wipp control module.
			 
			 Kill all the process involved 
************************************************************************/
void wipp_control_deinit(void)
{							
	/* Kill the general process process */
	if (wipp_control_general_process_pid > 0)
	{
		/* Update the process list */
		wipp_control_fill_process_table();

		/* Kill all the child's of the general process */
		wipp_control_kill_family_process(wipp_control_general_process_pid);

		wipp_control_general_process_pid = 0;
	}
}

/************************************************************************
*                        wipp_control_initialize_general_process  	                        *
 ************************************************************************
DESCRIPTION: 
************************************************************************/
void wipp_control_initialize_general_process()
{
	/***********************************/
	/* Initialize the general process */
	/*********************************/

	SHARED_MEMORY_RUN_PROCESS_RUNNING() = FALSE;
	
	if (pipe(wipp_control_general_process_out_pipe) < 0)
	{
		console_printf_terminal("wipp_control, wipp_control_initialize_general_process - Error creating out pipe!\n");
		
		return;
	}

	if (pipe(wipp_control_general_process_in_pipe) < 0)
	{
		console_printf_terminal("wipp_control, wipp_control_initialize_general_process - Error creating in pipe!\n");
		
		return;
	}

	/* Create a child process */
	wipp_control_general_process_pid = fork();
	
	if (0 == wipp_control_general_process_pid)
	{
		/******************/
		/* Child process */
		/****************/
		
		int result;
		char in_buffer[512];
		char command_buffer[4] = {0x0, 0x00, 0x00, WIPP_CONTROL_FROM_GENERAL_PROCESS_SEND_TERMINATE};

		/* Close the read direction of the pipe  - because i only write information from this pipe. */ 
		close(wipp_control_general_process_out_pipe[0]); 
		
		/* Close the write direction of the pipe  - because i only write information from this pipe. */ 
		close(wipp_control_general_process_in_pipe[1]); 

		console_printf_terminal("wipp_control, Hello from general process child module (pid = %d).\n", getpid());
		
		/* Redirect the output to the control pipe */
		result = dup2(wipp_control_general_process_out_pipe[1], 1);

		if (result == 1)
		{
			/* The duplication is ok! - we can start working */
			
			while (TRUE)
			{
				result = read(wipp_control_general_process_in_pipe[0], in_buffer, sizeof(in_buffer));

				in_buffer[result] = 0;
				
				/* Mark that the process is running */
				SHARED_MEMORY_RUN_PROCESS_RUNNING() = TRUE; 
				
				/* Run the command line */
				result = system(in_buffer);

				/* Save the running result */
				SHARED_MEMORY_RUN_PROCESS_RESULT() = result;
				
				/* Mark that the process is not running */
				SHARED_MEMORY_RUN_PROCESS_RUNNING() = FALSE; 

				/* Send indication to the main process that the general process is terminated */
				ipc_send_command_to_main_process(GENERAL_PROCESS_MODULE_ID,(unsigned char*) command_buffer, 4);
			}
		}
		else
		{
			/* Terminate the process ... */
			exit(0);
		}
	}
	else
	{
		/* Close the write direction of the pipe  - because i only write information from this pipe. */ 
		close(wipp_control_general_process_out_pipe[1]); 
		
		/* Close the read direction of the pipe  - because i only write information from this pipe. */ 
		close(wipp_control_general_process_in_pipe[0]); 
	}
}

/************************************************************************
 *                        wipp_control_send_message                     *
 ************************************************************************
DESCRIPTION: Sends a text message to host 

NOTE:		 Maximum length is 'WIPP_CONTROL_MESSAGE_MAX_LENGTH' bytes!

CONTEXT:  Main process only!
************************************************************************/
#define WIPP_CONTROL_MESSAGE_MAX_LENGTH	(512)

int wipp_control_send_message(unsigned char *buffer, unsigned int length)
{
	/*************************/
	/* send message to host */
	/***********************/

	return console_send_buffer_to_host(ETHERNET_UTILS_WIPP_MODULE_ID, buffer, length);
}

/************************************************************************
 *                        wipp_control_check_command                    *
 ************************************************************************
DESCRIPTION: Handle the wipp control specific commands 
			 
CONTEXT    : main process only! 			 
************************************************************************/
unsigned char wipp_control_check_command(char *input_string)
{
	ConParm_t param;
	int result;
	unsigned char return_value = FALSE;

	if (input_string[0] == '!')
	{
		switch (input_string[1])
		{
		case WIPP_CONTROL_FROM_GENERAL_PROCESS_SEND_TERMINATE:
			wipp_control_send_iperf_results_to_host(WIPP_CONTROL_EVT_RUN_PROCESS_TERMINATE, NULL, 0);
			break;

		case WIPP_CONTROL_FROM_GENERAL_PROCESS_DEACTIVATE_IPERF:
			wipp_control_deactive_iperf();
			break;
		}

		return_value = TRUE;
	}

	if (input_string[0] == '+')
	{
		switch (input_string[1])
		{
		case WIPP_CONTROL_CMD_CPU_USAGE:
			wipp_control_cpu_usage((ConParm_t *)NULL, (UINT)NULL);
			break;
			
		case WIPP_CONTROL_CMD_MEM_USAGE:
			wipp_control_mem_usage((ConParm_t *)NULL, (UINT)NULL);
			break;
			
		case WIPP_CONTROL_CMD_REPORT_VERSION:
			wipp_control_report_version((ConParm_t *)NULL, (UINT)NULL);
			break;
			
		case WIPP_CONTROL_CMD_DEBUG_CONTROL:
			
			param.value = input_string[3] - '0';
						
			switch (input_string[2])
			{
			case '0':
				/****************************/
				/* Enable the debug module */
				/**************************/
				
				console_printf_terminal("WIPP Control, Enable debug kernal module.\n");

				result = system("echo b > /dev/debug_msg_dev");
				
				if (result == -1)
				{
					console_printf_terminal("Error enabling debug module");
				}
				
				break;
				
			case '1':
				/*****************************/
				/* Disable the debug module */
				/***************************/

				console_printf_terminal("WIPP Control, Disable debug kernel module.\n");
				
				result = system("echo c > /dev/debug_msg_dev");
				
				if (result == -1)
				{
					console_printf_terminal("Error disabling debug module");
				}
				
				break;
				
        #ifdef TI_DBG

			case '2':
				/***********************/
				/* Select debug level */
				/*********************/
				
				param.value = (U32)&input_string[3];
				/*console_printf_terminal("WIPP Control, Set debug mask (%s).\n", (char *)param.value);*/
				
				cmd_report_severity_table(&param, 1);
				
				break;
				
			case '3':
				/*********************/
				/* Add module debug */
				/*******************/

				cmd_report_add(&param, 1);
				
				break;
				
			case '4':
				/***********************/
				/* Clear module debug */
				/*********************/

				cmd_report_clear(&param, 1);
				
				break;

			case '5':									 
				/*********************/
				/* Set module debug */
				/*******************/

				param.value = (U32)&input_string[3];
				/*console_printf_terminal("WIPP Control, Set module mask (%s).\n", param.value);*/
			
				cmd_report_set(&param, 1);

				break;

        #endif /* TI_DBG */

			case '6':
				/***************************************/
				/* Get debug module buffer statistics */
				/*************************************/

				debug_module_get_queue_status();

				break;

			case '7':

				/* Translate the value from range of '0-4' to range of '-20 - 20' */
				param.value <<= 3;
				param.value -= 20;

				console_printf_terminal("WIPP Control, Set debug module priority (%d).\n", param.value);

				errno = 0;

				/* Change the debug process priority */
/*				if (setpriority(PRIO_PROCESS, debug_module_process_pid, param.value) != 0)*/
/*				{*/
/*					console_printf_terminal("WIPP Control, Error changing priority!\n");*/
/*				}*/

				break;
				
			default:
				console_printf_terminal("++ command error (debug switch)!\n");
				break;

			}
			break;
			
		case WIPP_CONTROL_CMD_DEBUG_PATH:
			param.value = input_string[2] - '0';

			console_printf_terminal("WIPP Control, Activate debug API (%d).\n", param.value);
			
			wipp_control_set_pp_mode(&param, 1);
			break;

		case WIPP_CONTROL_CMD_ACTIVATE_PROCESS:
			wipp_control_active_iperf((char *)&input_string[1]);
			break;

		case WIPP_CONTROL_CMD_TERMINATE_PROCESS:
			wipp_control_deactive_iperf();
			break;
            
		case WIPP_CONTROL_CMD_GET_VERSIONS:
			wipp_control_get_version();
			break;
	
		default:
			console_printf_terminal("wipp control ++ command error!\n");
			break;
		}

		return_value = TRUE;
	}

	return return_value;
}

/************************************************************************
 *                        wipp_control_cpu_usage	                    *
 ************************************************************************
DESCRIPTION: Report to the host the current CPU usage
************************************************************************/

#define MAX_CPU_INFO_LINE_LEN (50)

void wipp_control_cpu_usage(ConParm_t parm[], U16 nParms)
{
	FILE *stat_file;
	char cpu_stat_line[MAX_CPU_INFO_LINE_LEN];

	stat_file = fopen("/proc/stat", "r");	

	if (stat_file != NULL)
	{
		if (fgets((cpu_stat_line + WIPP_CONTROL_HEADER_SIZE), MAX_CPU_INFO_LINE_LEN, stat_file) != NULL)
		{
			WIPP_HEADER(cpu_stat_line, '1');
			wipp_control_send_message((unsigned char*)cpu_stat_line, strlen(cpu_stat_line));
		}
		else
		{
			cpu_stat_line[WIPP_CONTROL_HEADER_SIZE] = 0;
			wipp_control_send_message((unsigned char*)cpu_stat_line, 3);
		}

		/**********************************************/
		/* Output the message to the standard output */
		/********************************************/

		fclose(stat_file);
	}
	else
	{
		cpu_stat_line[WIPP_CONTROL_HEADER_SIZE] = 0;
		wipp_control_send_message((unsigned char*)cpu_stat_line, 3);
	}
}

/************************************************************************
 *                        wipp_control_report_version	                *
 ************************************************************************
DESCRIPTION: Report to the host the current Memory usage
************************************************************************/

#define  MAX_MEM_INFO_LINE_LEN (250)

void wipp_control_mem_usage(ConParm_t parm[], U16 nParms)
{
	FILE *ver_file;
	char mem_stat_line[MAX_MEM_INFO_LINE_LEN];

	/* Proceed only in 'pp' mode */
	ver_file = fopen("/proc/meminfo", "r");	

	if (ver_file != NULL)
	{
		while (fgets((mem_stat_line + WIPP_CONTROL_HEADER_SIZE), MAX_MEM_INFO_LINE_LEN, ver_file) != NULL)
		{
			WIPP_HEADER(mem_stat_line, '2');
			mem_stat_line[WIPP_CONTROL_HEADER_SIZE] = 0;
			wipp_control_send_message((unsigned char*)mem_stat_line, strlen(mem_stat_line));
		}

		fclose(ver_file);
	}															   
}

/************************************************************************
 *                        wipp_control_set_pp_mode		                *
 ************************************************************************
DESCRIPTION: Set the driver to work with debug_module
************************************************************************/
void wipp_control_set_pp_mode(ConParm_t parm[], U16 nParms)
{
  #ifdef TI_DBG
	TI_SetReportPPMode(g_id_adapter, parm[0].value); 
  #endif
}

/************************************************************************
 *                        wipp_control_mem_usage	                    *
 ************************************************************************
DESCRIPTION: Report to the host the current Memory usage
************************************************************************/

#define MAX_VER_INFO_LINE_LEN (250)

void wipp_control_report_version(ConParm_t parm[], U16 nParms)
{
	FILE *ver_file;
	char ver_line[MAX_VER_INFO_LINE_LEN];

	ver_file = fopen("/proc/version", "r");	

	if (ver_file != NULL)
	{
		if (fgets((ver_line + WIPP_CONTROL_HEADER_SIZE), MAX_VER_INFO_LINE_LEN, ver_file) != NULL)
		{
			WIPP_HEADER(ver_line, '3');
			ver_line[125] = 0;
			wipp_control_send_message((unsigned char*)ver_line, strlen(ver_line));
		}

		fclose(ver_file);
	}
	else
	{
		/* Write the protocol prefix - mark error ('0' in len) */
		WIPP_HEADER(ver_line, '3');
		ver_line[WIPP_CONTROL_HEADER_SIZE] = 0;
		wipp_control_send_message((unsigned char*)ver_line, strlen(ver_line));
	}
}

/************************************************************************
 *                        wipp_control_disable_term_echo                *
 ************************************************************************
DESCRIPTION: 
************************************************************************/
void wipp_control_disable_term_echo()
{
	struct termios term;

	if (isatty(STDIN_FILENO) == 0)
	{
		console_printf_terminal("wipp control, Error, standart input is not a terminal device\n");
	}
	else
	{
		if (tcgetattr(STDIN_FILENO, &term) != -1)
		{
			/* Disable the echoing */
			term.c_lflag &= ~(ECHO);

			if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &term) != -1)
			{
				console_printf_terminal("wipp control, Echo is disabled!\n");
			}
			else
			{
				console_printf_terminal("wipp control, Error writing terminal attributes\n");
			}
		}
		else
		{
			console_printf_terminal("wipp control, Error reading terminal attributes\n");
		}
	}
}

/************************************************************************
 *                        wipp_control_set_baud_rate                    *
 ************************************************************************
DESCRIPTION: Set the UART baudrate
************************************************************************/
void wipp_control_set_baud_rate(int rate_index)
{
	struct termios term;
	int r1,r2;

	if (isatty(STDIN_FILENO) == 0)
	{
		console_printf_terminal("wipp control, Error, standart input is not a terminal device\n");
	}
	else
	{
		if (tcgetattr(STDIN_FILENO, &term) != -1)
		{
			/* Change the baud rate*/

			r1 = cfsetispeed(&term, B230400);
			r2 = cfsetospeed(&term, B230400);

			console_printf_terminal("wipp control, results: %d, %d\n", r1, r2);
		}
		else
		{
			console_printf_terminal("wipp control, Error reading terminal attributes\n");
		}
	}
}

/************************************************************************
 *                        wipp_control_active_iperf		                *
 ************************************************************************
DESCRIPTION: Start the iperf process
************************************************************************/
void wipp_control_active_iperf(char *iperf_cmd_line)
{
	unsigned int command_length;

	if (SHARED_MEMORY_RUN_PROCESS_RUNNING())
	{
		console_printf_terminal("wipp control, wipp_control_active_iperf: Error: process already running!\n");

		wipp_control_send_iperf_results_to_host(WIPP_CONTROL_EVT_RUN_PROCESS_IS_RUNING, NULL, 0);;
	}
	else
	{
		iperf_cmd_line++;

		command_length = *(iperf_cmd_line + 0) | (*(iperf_cmd_line + 1) << 8) | (*(iperf_cmd_line + 2) << 16) | (*(iperf_cmd_line + 3) << 24);
		iperf_cmd_line += 4;
		iperf_cmd_line[command_length] = 0;

		console_printf_terminal("wipp control, wipp_control_active_iperf (Len = %d, Cmd = %s)\n", command_length, iperf_cmd_line);

		/* Send the command to the process */
		write(wipp_control_general_process_in_pipe[1], iperf_cmd_line, command_length);
	}
}

/************************************************************************
 *                        wipp_control_deactive_iperf	                *
 ************************************************************************
DESCRIPTION: Stop the iperf process (if it's running) 
************************************************************************/
void wipp_control_deactive_iperf()
{
	if (!SHARED_MEMORY_RUN_PROCESS_RUNNING())
	{
		console_printf_terminal("wipp control, wipp_control_active_iperf: Error: process is not running!\n");

		wipp_control_send_iperf_results_to_host(WIPP_CONTROL_EVT_RUN_PROCESS_IS_NOT_RUNING, NULL, 0);
	}
	else
	{
		console_printf_terminal("wipp control, wipp_control_deactive_iperf\n");

		/* Kill the general process process */
		if (wipp_control_general_process_pid > 0)
		{
			/* Update the process list */
			wipp_control_fill_process_table();

			/* Kill all the child's of the general process */
			wipp_control_kill_family_process(wipp_control_general_process_pid);

			wipp_control_general_process_pid = 0;
		}

		close(wipp_control_general_process_out_pipe[0]); 
		close(wipp_control_general_process_in_pipe[1]); 
		
		/* Initialize the general process */
		wipp_control_initialize_general_process();
	}
}

/************************************************************************
 *                        wipp_control_send_iperf_results_to_host       *
 ************************************************************************
DESCRIPTION: Send the run process results to the host via ethernet.
			 The first parameter is the event type (one of the defines), 
			 it is inserted in the ___ byte of the buffer. Therefor the input
			 buffer ("inbuf") actual data should start at the ___ byte of the buffer.

CONTEXT	   : main process only!
************************************************************************/
void wipp_control_send_iperf_results_to_host(unsigned char event, char *inbuf, int result)
{
	/* console_printf_terminal("wipp_control, wipp_control_send_iperf_results_to_host (event = %d)\n", event); */
	unsigned char temp_buffer[5] = {'+', '4', '0', '0', '0'};
	temp_buffer[2] = event;

	switch (event)
	{
	case WIPP_CONTROL_EVT_RUN_PROCESS_STDOUT:
		WIPP_HEADER(inbuf, '4');
		inbuf[2] = event;
		wipp_control_send_message((unsigned char*)inbuf, result + 3);
		break;

	case WIPP_CONTROL_EVT_RUN_PROCESS_TERMINATE:
		temp_buffer[3] = (SHARED_MEMORY_RUN_PROCESS_RESULT() & 0x00FF);
		temp_buffer[4] = ((SHARED_MEMORY_RUN_PROCESS_RESULT() & 0xFF00) >> 8);
		wipp_control_send_message(temp_buffer, 5);		
		break;

	case WIPP_CONTROL_EVT_RUN_PROCESS_IS_RUNING:
	case WIPP_CONTROL_EVT_RUN_PROCESS_IS_NOT_RUNING:
		wipp_control_send_message(temp_buffer, 3);		
		break;
	}
}

/************************************************************************
 *                        wipp_control_kill_family_process		        *
 ************************************************************************
DESCRIPTION: Recursive kill of all the process child's.
	

CONTEXT	   : 
************************************************************************/
void wipp_control_kill_family_process(int father_pid)
{
	int index;
	int status;
	
	/* Check to see this process is a parent */
	for (index = 0; index < process_table_current_index; index++)
	{
		if (process_table[index][1] == father_pid)
		{
			wipp_control_kill_family_process(process_table[index][0]);
		}
	}

	/* Search for the process and kill it */

	for (index = 0; index < process_table_current_index; index++)
	{
		if (process_table[index][0] == father_pid)
		{
			/* Kill the process */
			kill(process_table[index][0] ,SIGKILL);

			waitpid(process_table[index][0], &status, WUNTRACED);

			/*console_printf_terminal("wipp_control, wipp_control_kill_family_process, Killing index %d, [%d][%d]\n",process_table[index][0], process_table[index][1]);*/

			/* Mark the process as killed */
			process_table[index][0] = process_table[index][1] = -1;
						
		}
	}
}

/************************************************************************
 *                        wipp_control_fill_process_table		        *
 ************************************************************************
DESCRIPTION: Update the table with the current processs information
	

CONTEXT	   : 
************************************************************************/
void wipp_control_fill_process_table()
{
	DIR *p_directory;
	struct dirent *p_dir_entry;
	int direcroty_string_lengh;
	int index;
	int is_valid_process_directory;

	/* Clear the table */
	process_table_current_index = 0;

	/* Open the directory */
	if ((p_directory = opendir("/proc/")) == NULL)
	{
		console_printf_terminal("wipp_control, wipp_control_fill_process_table, Can't open directory!");
		return;
	}

	/* Go over all the files in the directory */
	while ((p_dir_entry = readdir(p_directory)) != NULL)
	{
		/* Get rid of the unwanted file names */
		if ((strcmp(p_dir_entry->d_name, ".") == 0) || 
			(strcmp(p_dir_entry->d_name, "..") == 0)) 
		{
			continue;
		}
								
		direcroty_string_lengh = strlen(p_dir_entry->d_name);
		
		is_valid_process_directory = 1;

		/* Check if it is valid process directory name */
		for (index = 0; index < direcroty_string_lengh; index++)
		{
			if ((p_dir_entry->d_name[index] > '9') || (p_dir_entry->d_name[index] < '0'))
			{
				is_valid_process_directory = 0;

				break;
			}
		}

		if (is_valid_process_directory)
		{
			FILE *file_des;
			int temp_number;
			char temp_buffer[TEMP_BUFFER_SIZE];

			if (snprintf(temp_buffer, TEMP_BUFFER_SIZE, "/proc/%s/status", p_dir_entry->d_name) > 0)
			{
				/* Get the information out of the status file */
				if ((file_des = fopen(temp_buffer, "r")) > 0)
				{
					/* Skip the first four lines */
					fgets(temp_buffer, TEMP_BUFFER_SIZE, file_des);
					fgets(temp_buffer, TEMP_BUFFER_SIZE, file_des);
					fgets(temp_buffer, TEMP_BUFFER_SIZE, file_des);
					fgets(temp_buffer, TEMP_BUFFER_SIZE, file_des);

					/* Get the process id */
					fgets(temp_buffer, TEMP_BUFFER_SIZE, file_des);
					sscanf(temp_buffer, "%s %d", &temp_buffer[TEMP_BUFFER_SIZE / 2], &temp_number);
					process_table[process_table_current_index][0] = temp_number;

					/* Get the parent process id */					
					fgets(temp_buffer, TEMP_BUFFER_SIZE, file_des);
					sscanf(temp_buffer, "%s %d", &temp_buffer[TEMP_BUFFER_SIZE / 2], &temp_number);
					process_table[process_table_current_index][1] = temp_number;

					fclose(file_des);

					process_table_current_index++;
					if (process_table_current_index == MAX_PROCESS_TABLE_INDEX)
					{
						console_printf_terminal("wipp_control, wipp_control_fill_process_table, Error, reached maximum process table index\n");

						break;
					}
				}
			}
		}
	}

	closedir(p_directory);
}

/************************************************************************
 *                        wipp_control_get_version                     *
 ************************************************************************
DESCRIPTION: 
			 
CONTEXT    :  			 
************************************************************************/
void  wipp_control_get_version(void)
{
	/***************************
	Return buffer structure :

	Byte  0    : Header ('+')
    Byte  1    : OpCode ('a')
	Byte  2	   : Command Status
	Byte  3-6  : Driver Version
	Byte  7-10 : FW Version
	***************************/
	
	tiUINT32 ret;
	tiUINT8 return_buffer[11];

    TIWLN_VERSION_EX VersionData;

	console_printf_terminal("WIPP control,  wipp_control__get_version.\n");

    ret = (tiUINT8)TI_GetDriverVersion(g_id_adapter, &VersionData);

    memset(return_buffer, 0, sizeof(return_buffer) / sizeof(return_buffer[0]));
    WIPP_HEADER(return_buffer, 'a');
	return_buffer[2] = (UINT8)ret;
    if (ret == OK)
	{
        memcpy(return_buffer + 3, &(VersionData.DrvVersion), 4);
        memcpy(return_buffer + 7, &(VersionData.FWVersion), 4);
	}
	wipp_control_send_message(return_buffer, sizeof(return_buffer));
}

