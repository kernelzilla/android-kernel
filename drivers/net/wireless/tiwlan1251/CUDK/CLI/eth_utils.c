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
/*      MODULE:     eth_utils.c                                                                     */
/*      PURPOSE:    Ethernet communication utilities                                                */
/*                                                                                                  */
/****************************************************************************************************/

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <unistd.h>
#include <arpa/inet.h>

#include "ipc.h"
#include "ticon.h"							  
#include "errno.h"
#include "signal.h"
#include "eth_utils.h"

extern char* inet_ntoa(struct  in_addr);
	/************/
	/* Defines */
	/**********/

#define LOOGER_ETHERNET_PORT	(700)
#define WIPP_ETHERNET_PORT		(701)
#define G_TESTER_ETHERNET_PORT	(702)

#define READ_STATE_GET_HEADER		(0x00)
#define READ_STATE_GET_LENGTH1		(0x01)
#define READ_STATE_GET_LENGTH2		(0x02)
#define READ_STATE_GET_PAYLOAD		(0x03)

#define PACKET_PREFIX			(0xFF)

#define IN_BUFFER_SIZE			(1024)
#define OUT_BUFFER_SIZE			(512)

	/*********************/
	/* Global variables */
	/*******************/

unsigned char ethernet_utils_welcome_message[] = {'W', 2, 0, 2, ETH_UTILS_PROTOCOL_VERSION};

char ethernet_utils_module_names[ETHERNET_UTILS_NUMBER_OF_MODULES][20] =
{
	"terminal",
	"logger",
	"wipp control",
	"g tester"
};

	/********************************/
	/* static functions prototypes */
	/******************************/

int ethernet_utils_init_module(int module_index, u_short port_number, int *module_pipe, unsigned char mux_uart_id);
void ethernet_utils_signal_handler(int signal);
int ethernet_utils_process_in_command(int module_index, unsigned char *read_state, unsigned short *packet_size, unsigned char *in_buffer, unsigned int *in_buffer_offset);
int ethernet_utils_write_socket(int module_index, int length, unsigned char *buffer);
void ethernet_utils_disconnect_socket(int module_index);

	/**************/
	/* Functions */
	/************/

/************************************************************************
 *                        ethernet_utils_init                           *
 ************************************************************************
DESCRIPTION: Initialize the ethernet communication

CONTEXT	   : main process only!
************************************************************************/
void ethernet_utils_init()
{
	ethernet_logger_process_pid = ethernet_utils_init_module(ETHERNET_UTILS_LOGGER_MODULE_ID, LOOGER_ETHERNET_PORT, ethernet_logger_pipe, LOGGER_MUX_UART_ID);
	ethernet_wipp_process_pid = ethernet_utils_init_module(ETHERNET_UTILS_WIPP_MODULE_ID, WIPP_ETHERNET_PORT, ethernet_wipp_control_pipe, WIPP_CONTROL_MUX_UART_ID);
	ethernet_g_tester_process_pid = ethernet_utils_init_module(ETHERNET_UTILS_G_TESTER_MODULE_ID, G_TESTER_ETHERNET_PORT, ethernet_g_tester_pipe, G_TESTER_MUX_UART_ID);
}
		  
/************************************************************************
 *                        ethernet_utils_deinit                         *
 ************************************************************************
DESCRIPTION: Deinitialize the ethernet communication

CONTEXT	   : main process only!
************************************************************************/
void ethernet_utils_deinit()
{
	/* Kill the logger process */
	if (ethernet_logger_process_pid > 0)
	{
		kill(ethernet_logger_process_pid, SIGKILL);
	}

	/* Kill the wipp control process */
	if (ethernet_wipp_process_pid > 0)
	{
		kill(ethernet_wipp_process_pid, SIGKILL);
	}

	/* Kill the wipp control process */
	if (ethernet_g_tester_process_pid > 0)
	{
		kill(ethernet_g_tester_process_pid, SIGKILL);
	}
}

/************************************************************************
 *                        ethernet_utils_signal_handler                 *
 ************************************************************************
DESCRIPTION: Signal handler - receive the USER

CONTEXT	   : Signal owner 
************************************************************************/
void ethernet_utils_signal_handler(int signal) 
{ 
}

/************************************************************************
 *                        ethernet_utils_init_module                    *
 ************************************************************************
DESCRIPTION: Initialize ethernet communication

RETURNS:   : Process ID of the new process or -1 if error.

CONTEXT	   : main process only!
************************************************************************/
int ethernet_utils_init_module(int module_index, u_short port_number, int *module_pipe, unsigned char mux_uart_id)
{
	int child_process_id;

	/***************************************/
	/* Create a pipe to control the child */
	/*************************************/

	if (pipe(module_pipe) < 0)
	{
		console_printf_terminal("eth_utils, error creating pipe\n");

		return -1;
	}

	/* Set the shared memory variables */
	SHARED_MEMORY_IPC_PIPE(module_index) = module_pipe[1];
	SHARED_MEMORY_MUX_UART_ID(module_index) = mux_uart_id;

	/* Create a child process */
	child_process_id = fork();

	if (0 == child_process_id)
	{
		/******************/
		/* Child process */
		/****************/

		int result;
		int socket_id;
		int optval = 1;
		int socket_alive;
		int max_fd_index;
		socklen_t client_addr_len;
		fd_set read_set;
		unsigned char out_buffer[OUT_BUFFER_SIZE];
		unsigned char in_buffer[IN_BUFFER_SIZE];
		unsigned char read_state;
		unsigned short packet_size;
		unsigned int in_buffer_offset;
		struct sockaddr_in server_addr;
		struct sockaddr_in client_addr;

		console_printf_terminal("eth_utils, Hello from %s child module (pid = %d).\n", ethernet_utils_module_names[module_index], getpid());

		/* Close the write direction of the pipe  - because i only read information from this pipe. */ 
		close(module_pipe[1]); 

		SHARED_MEMORY_OUTPUT_PATH(module_index) = OUTPUT_PATH_SIMPLE_UART;

		/* Set the signal handler for the 'SIGUSR1' signal */
		signal(SIGUSR1, ethernet_utils_signal_handler); 

		while (TRUE)
		{
			/******************/
			/* Open a socket */
			/****************/

			socket_id = socket(PF_INET, SOCK_STREAM, 0);

			if (!socket_id)
			{
				/* Error opening socket */
				
				console_printf_terminal("eth_utils, error opening %s socket. (errno = %d)\n", ethernet_utils_module_names[module_index], errno);

				_exit(1);
			}

			/*************************/
			/* Configure the socket */
			/***********************/

			if (setsockopt(socket_id, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) == -1)
			{
				/* Error setting socket option */
				
				console_printf_terminal("eth_utils, error setting %s socket option.  (errno = %d)\n", ethernet_utils_module_names[module_index], errno);

				_exit(1);
			}

			/********************/
			/* Bind the socket */
			/******************/

			server_addr.sin_family = AF_INET;
			server_addr.sin_addr.s_addr = htonl (INADDR_ANY);
			server_addr.sin_port = htons(port_number);

			result = bind(socket_id, (struct sockaddr *)&server_addr, sizeof(server_addr));

			if (result != 0)
			{
				/* Error binding socket */
				
				console_printf_terminal("eth_utils, error binding %s socket. (errno = %d)\n", ethernet_utils_module_names[module_index], errno);

				_exit(1);
			}

			/****************************/
			/* Listening to the socket */
			/**************************/

			result = listen(socket_id, 5);

			if (-1 == result)
			{
				/* Error listening to socket */
				
				console_printf_terminal("eth_utils, error listening to %s socket. (errno = %d)\n", ethernet_utils_module_names[module_index], errno);

				_exit(1);
			}

			/**********************/
			/* Accept connection */
			/********************/

			client_addr_len = sizeof(client_addr);

			/* We suppose to get new socket id after accept (blocking action) */
			result = accept(socket_id, (struct sockaddr *)&client_addr, &client_addr_len);

			if (-1 == result)
			{			
				/* Error accepting connection */
				
				console_printf_terminal("eth_utils, error accepting %s connection. (errno = %d)\n", ethernet_utils_module_names[module_index], errno);

				_exit(1);
			}

			close(socket_id);

			socket_id = result;

			read_state = READ_STATE_GET_HEADER;

			SHARED_MEMORY_ETHERNET_SOCKET_ID(module_index) = socket_id;

			console_printf_terminal("\n***********************************************\n");
			console_printf_terminal("* eth_utils, %s connection accepted. \n", ethernet_utils_module_names[module_index]);
			console_printf_terminal("*\n* Remote IP: %s\n***********************************************\n", inet_ntoa(client_addr.sin_addr));

			/* Switch to ethernet output */
			SHARED_MEMORY_OUTPUT_PATH(module_index) = OUTPUT_PATH_ETHERNET;

			/*********************************/
			/* Send Hello message to client */
			/*******************************/

			ethernet_utils_write_socket(module_index, sizeof(ethernet_utils_welcome_message), ethernet_utils_welcome_message);

			/**********************/
			/* Manage connection */
			/********************/

			socket_alive = TRUE;

			while (socket_alive)
			{
				/***********************************************************************************/   
				/* Wait for one of two external events: 									      */
				/* -----------------------------------											 */
				/*																			    */
				/* 1. Data received from TCP client and should be transfered to parent process */
				/* 2. Data received from parent process and should be transfered to client 	  */
				/*****************************************************************************/   

				/* Prepare the read set fields */
				FD_ZERO(&read_set);
				FD_SET(socket_id, &read_set);
				FD_SET(module_pipe[0], &read_set);

				/* Determine the maximum index of the file descriptor */
				max_fd_index = (max(socket_id, module_pipe[0]) + 1);

				/* Wait for event - blocking */
				result = select(max_fd_index, &read_set, NULL, NULL, NULL);

				/************************/
				/* Check event results */
				/**********************/

				if (result > 0)
				{
					if (FD_ISSET(socket_id, &read_set))
					{
						/******************************/
						/* Data received from client */
						/****************************/
						
						/* Process the input command */
						if (ethernet_utils_process_in_command(module_index, &read_state, &packet_size, in_buffer, &in_buffer_offset) <= 0) 
						{
							socket_alive = FALSE;
						}
					}

					if (FD_ISSET(module_pipe[0], &read_set))
                    {
						/**************************************/
						/* Data received from parent process */
						/************************************/

						result = read(module_pipe[0], out_buffer, sizeof(out_buffer));
                        if( result < 0 )
                        {
                            console_printf_terminal("eth_utils, read error (err=%d)\n", result);
                            socket_alive = FALSE;
                        }
				
						if (ethernet_utils_write_socket(module_index, result, out_buffer) == -1)
						{
							socket_alive = FALSE;
						}
					}
				}
				else
				{
					console_printf_terminal("eth_utils, 'select' command error\n");

					socket_alive = FALSE;
				}
			}

			/* Disconnect the socket */
			ethernet_utils_disconnect_socket(module_index);
		}
	}

	/* Close the read direction of the pipe  - because i only write information from this pipe. (to child process) */ 
	close(module_pipe[0]); 

	/* return the process id, I will use it later when i want to kill this process */
	return child_process_id;
}

/************************************************************************
 *                        ethernet_utils_process_in_command             *
 ************************************************************************
DESCRIPTION: Handle In commands

CONTEXT	   : Only the same process that is reading from the socket
************************************************************************/
int ethernet_utils_process_in_command(int module_index, unsigned char *read_state, unsigned short *packet_size, unsigned char *in_buffer, unsigned int *in_buffer_offset)
{	
	unsigned char prefix;
	int socket_id = SHARED_MEMORY_ETHERNET_SOCKET_ID(module_index);
    int result = 1; 

	/*console_printf_terminal("ethernet_utils_process_in_command (socket = %d)\n", socket_id); */

	/* Continue while there is data in the RX buffer */
	switch (*read_state)
	{
	case READ_STATE_GET_HEADER:

		/* Read the packet prefix - one byte */
		result = read(socket_id, &prefix, sizeof(prefix));
		/*console_printf_terminal("ethernet_utils_process_in_command (State = READ_STATE_GET_HEADER, length = %d)\n", result); */

		if (result > 0)
		{
			if (prefix == PACKET_PREFIX)
			{
				*read_state = READ_STATE_GET_LENGTH1;
			}
			else
			{
				console_printf_terminal("ethernet_utils_process_in_command, Error: protocol sync error! \n"); 

				result = -1;
			}
		}

		break;

	case READ_STATE_GET_LENGTH1:

		/* Read the packet size first byte */
		result = read(socket_id, (void *)((unsigned char *)(packet_size) + 0), sizeof(unsigned char));
		/*console_printf_terminal("ethernet_utils_process_in_command (State = READ_STATE_GET_LENGTH1, length = %d)\n", result); */
		
		if (result > 0)
		{
			*read_state = READ_STATE_GET_LENGTH2;
		}

		break;

	case READ_STATE_GET_LENGTH2:

		/* Read the packet size second byte */
		result = read(socket_id, (void *)((unsigned char *)(packet_size) + 1), sizeof(unsigned char));
		/*console_printf_terminal("ethernet_utils_process_in_command (State = READ_STATE_GET_LENGTH2, length = %d, packet_size = %d)\n", result, *packet_size); */
		
		if (result > 0)
		{
			/* Sanity check on the length */

			*in_buffer_offset = 0;

			*read_state = READ_STATE_GET_PAYLOAD;
		}

		break;


	case READ_STATE_GET_PAYLOAD:

		/* Read the packet size second byte */
		result = read(socket_id, (in_buffer + 3 + *in_buffer_offset), (*packet_size - *in_buffer_offset));
		/*console_printf_terminal("ethernet_utils_process_in_command (State = READ_STATE_GET_PAYLOAD, offset = %d, length = %d)\n", *in_buffer_offset, result); */
		
		if (result > 0)
		{
			*in_buffer_offset += result;

			if (*packet_size == *in_buffer_offset)
			{
				/* All the packet has arrived */

				*read_state = READ_STATE_GET_HEADER;

				/* Send it to the main process */

				ipc_send_command_to_main_process(module_index, in_buffer, (*packet_size + 3));
			}
		}

		break;
	}

	return result;
}

/************************************************************************
 *                        ethernet_utils_write_socket     			    *
 ************************************************************************
DESCRIPTION: write data to socket

CONTEXT    : Only the same process that is writing to the socket
************************************************************************/
int ethernet_utils_write_socket(int module_index, int length, unsigned char *buffer)
{
	int result;

/*	console_printf_terminal("eth_utils, ethernet_utils_wipp_write() (length = %d).\n", length); */

	/* Write to the socket */
	result = write(SHARED_MEMORY_ETHERNET_SOCKET_ID(module_index), buffer, length);

/*	console_printf_terminal("eth_utils, ethernet_utils_wipp_write() (result = %d).\n", result); */

	if (result != length)
	{
		/**************************/
		/* Error writing to port */
		/************************/
		
		console_printf_terminal("eth_utils, Error writing to %s socket (result = %d, errno = %d, error = %s)\n", ethernet_utils_module_names[module_index], result, errno, strerror(errno));

		result = -1;
	}

	return result;
}

/************************************************************************
 *                        ethernet_utils_disconnect_socket 			    *
 ************************************************************************
DESCRIPTION: Disconnect a socket

CONTEXT    : Only the process that is disconnecting
************************************************************************/
void ethernet_utils_disconnect_socket(int module_index)
{
	char temp_buf[6] = {'x','x','x', '5', '0', 0};

	/* Switch to UART output */
	SHARED_MEMORY_SWITCH_TO_UART_OUTPUT(module_index);

	console_printf_terminal("eth_utils, disconnecting from %s socket.\n", ethernet_utils_module_names[module_index]);

	switch (module_index)
	{  
	case ETHERNET_UTILS_LOGGER_MODULE_ID:
		/* Set debug path back to UART */
		ipc_send_command_to_main_process(ETHERNET_UTILS_LOGGER_MODULE_ID, (unsigned char*)temp_buf, 5);
		break;

	case ETHERNET_UTILS_WIPP_MODULE_ID:
		temp_buf[3] = WIPP_CONTROL_FROM_GENERAL_PROCESS_DEACTIVATE_IPERF;
		ipc_send_command_to_main_process(GENERAL_PROCESS_MODULE_ID,(unsigned char*) temp_buf, 5);
		break;
	}

	/* Close the socket */
	close(SHARED_MEMORY_ETHERNET_SOCKET_ID(module_index));
}
