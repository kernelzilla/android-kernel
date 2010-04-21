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

#ifndef IPC_H
#define IPC_H

#define TERMINAL_MUX_UART_ID		(28)
#define LOGGER_MUX_UART_ID			(30)
#define WIPP_CONTROL_MUX_UART_ID	(25)
#define G_TESTER_MUX_UART_ID		(23)


#define ETHERNET_UTILS_NUMBER_OF_MODULES	(4)
#define ETHERNET_UTILS_TERMINAL_MODULE_ID	(0)
#define ETHERNET_UTILS_LOGGER_MODULE_ID		(1)
#define ETHERNET_UTILS_WIPP_MODULE_ID		(2)
#define ETHERNET_UTILS_G_TESTER_MODULE_ID	(3)

#define GENERAL_PROCESS_MODULE_ID			(4)

#define WIPP_CONTROL_FROM_GENERAL_PROCESS_SEND_TERMINATE	(0x00)
#define WIPP_CONTROL_FROM_GENERAL_PROCESS_DEACTIVATE_IPERF	(0x01)

extern void *p_shared_memory;
extern int ipc_semaphore_id;

extern int ethernet_logger_process_pid;
extern int ethernet_g_tester_process_pid;
extern int ethernet_wipp_process_pid;

extern int ethernet_wipp_control_pipe[2];
extern int ethernet_g_tester_pipe[2];
extern int ethernet_logger_pipe[2];

extern int ipc_pipe[2];

typedef struct t_shared_memory_rec
{
#define OUTPUT_PATH_SIMPLE_UART	(0)
#define OUTPUT_PATH_MUX_UART	(1)
#define OUTPUT_PATH_ETHERNET	(2)
	unsigned char output_paths[ETHERNET_UTILS_NUMBER_OF_MODULES];
	unsigned char mux_uart_id[ETHERNET_UTILS_NUMBER_OF_MODULES];
	int ethernet_socket_ids[ETHERNET_UTILS_NUMBER_OF_MODULES]; 
	int ipc_pipe[ETHERNET_UTILS_NUMBER_OF_MODULES];
	unsigned char uart_mux_enabled;
	int debug_module_dev_file;
	int run_process_state;
	int run_process_result;
} shared_memory_reco;

#define SHARED_MEMORY_SIZE	(sizeof(shared_memory_reco))


#define SHARED_MEMORY_OUTPUT_PATH(_module_index)		(((shared_memory_reco *)(p_shared_memory))->output_paths[_module_index])	
#define SHARED_MEMORY_TERMINAL_OUTPUT_PATH()			(((shared_memory_reco *)(p_shared_memory))->output_paths[ETHERNET_UTILS_TERMINAL_MODULE_ID])	
#define SHARED_MEMORY_LOGGER_OUTPUT_PATH()				(((shared_memory_reco *)(p_shared_memory))->output_paths[ETHERNET_UTILS_LOGGER_MODULE_ID])	
#define SHARED_MEMORY_WIPP_OUTPUT_PATH()				(((shared_memory_reco *)(p_shared_memory))->output_paths[ETHERNET_UTILS_WIPP_MODULE_ID])	

#define SHARED_MEMORY_ETHERNET_SOCKET_ID(_module_index) (((shared_memory_reco *)(p_shared_memory))->ethernet_socket_ids[_module_index])	
#define SHARED_MEMORY_ETHERNET_TERMINAL_SOCKET_ID()		(((shared_memory_reco *)(p_shared_memory))->ethernet_socket_ids[ETHERNET_UTILS_TERMINAL_MODULE_ID])		
#define SHARED_MEMORY_ETHERNET_LOGGER_SOCKET_ID()		(((shared_memory_reco *)(p_shared_memory))->ethernet_socket_ids[ETHERNET_UTILS_LOGGER_MODULE_ID])		
#define SHARED_MEMORY_ETHERNET_WIPP_SOCKET_ID()			(((shared_memory_reco *)(p_shared_memory))->ethernet_socket_ids[ETHERNET_UTILS_WIPP_MODULE_ID])

#define SHARED_MEMORY_IPC_PIPE(_module_index)			(((shared_memory_reco *)(p_shared_memory))->ipc_pipe[_module_index])
#define SHARED_MEMORY_MUX_UART_ID(_module_index)		(((shared_memory_reco *)(p_shared_memory))->mux_uart_id[_module_index])

#define SHARED_MEMORY_UART_MUX_ENABLED()				(((shared_memory_reco *)(p_shared_memory))->uart_mux_enabled)	

#define SHARED_MEMORY_RUN_PROCESS_RUNNING()				(((shared_memory_reco *)(p_shared_memory))->run_process_state)	
#define SHARED_MEMORY_RUN_PROCESS_RESULT()				(((shared_memory_reco *)(p_shared_memory))->run_process_result)	

#define SHARED_MEMORY_SWITCH_TO_UART_OUTPUT(_module_index)	(SHARED_MEMORY_OUTPUT_PATH(_module_index) = (SHARED_MEMORY_UART_MUX_ENABLED() ? OUTPUT_PATH_MUX_UART : OUTPUT_PATH_SIMPLE_UART)) 

#define SHARED_MEMORY_DBG_DEV_FILE()				    (((shared_memory_reco *)(p_shared_memory))->debug_module_dev_file) 


int ipc_initialize(void);
void ipc_deinitialize(void);
void ipc_send_command_to_main_process(int module_index, unsigned char *commnad, int size);

#endif /* IPC_H */
