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

#ifndef WIPP_CTRL_H
#define WIPP_CTRL_H

extern int wipp_control_general_process_pid;
extern int wipp_control_general_process_out_pipe[2];


#define WIPP_CONTROL_CMD_CPU_USAGE				('1')
#define WIPP_CONTROL_CMD_MEM_USAGE				('2')
#define WIPP_CONTROL_CMD_REPORT_VERSION			('3')
#define WIPP_CONTROL_CMD_DEBUG_CONTROL			('4')
#define WIPP_CONTROL_CMD_DEBUG_PATH				('5')
#define WIPP_CONTROL_CMD_ACTIVATE_PROCESS		('6')
#define WIPP_CONTROL_CMD_TERMINATE_PROCESS		('7')
#define WIPP_CONTROL_CMD_GET_VERSIONS		    ('a')


#define WIPP_CONTROL_EVT_RUN_PROCESS_STDOUT			(0x00)
#define WIPP_CONTROL_EVT_RUN_PROCESS_TERMINATE		(0x01)
#define WIPP_CONTROL_EVT_RUN_PROCESS_IS_RUNING		(0x02)
#define WIPP_CONTROL_EVT_RUN_PROCESS_IS_NOT_RUNING  (0x03)
void wipp_control_send_iperf_results_to_host(unsigned char event, char *inbuf, int result);

unsigned char wipp_control_check_command(char *input_string);

void wipp_control_init(void);
void wipp_control_deinit(void);

#endif /* #define WIPP_CTRL_H */
